#![allow(dead_code)]
//! Loom-based exhaustive concurrency tests for Watchdog primitives.
//!
//! These tests complement the ring-buffer tests in `loom_ring_buffers.rs`.
//! Together these files provide loom coverage for concurrency fixes in
//! the HORUS Security & Safety Hardening audit.
//!
//! Each primitive is re-implemented with loom's instrumented atomics so loom
//! can track every load/store/CAS and exhaustively verify all interleavings.
//!
//! # Primitives covered
//!
//! | File                     | Primitive          | Fix verified                                  |
//! |:-------------------------|:-------------------|:----------------------------------------------|
//! | `loom_seqlock.rs`*       | TransformFrame seqlock     | Release/Acquire ordering, no torn reads       |
//! | **this file**            | Watchdog           | AtomicU64 heartbeat, no TOCTOU false-positive |
//!
//! *`loom_seqlock.rs` is in `horus_library/tests/`, not `horus_core/tests/`.
//!
//! # Running
//!
//! ```sh
//! cargo test -p horus_core --test loom_concurrency -- --nocapture
//! ```
//!
//! Loom is O(exponential) in the number of thread switches.  All tests here
//! use at most 2 concurrent threads and minimal operation counts to keep
//! the exploration tractable (seconds, not hours).

use loom::sync::atomic::{AtomicU64, Ordering};
use loom::sync::Arc;

// ── SECTION 1: Watchdog (AtomicU64 heartbeat timestamp) ──────────────────────
//
// # Fix under test
//
// Before the fix, Watchdog stored `last_heartbeat: Mutex<Instant>`.  The
// check() function locked the mutex, read the Instant, released the mutex,
// then called .elapsed().  A heartbeat arriving between unlock and elapsed()
// was missed: check() used a stale timestamp and reported a false-positive
// expiry even though the watchdog was healthy.
//
// The fix replaced `Mutex<Instant>` with `AtomicU64` (nanoseconds since
// UNIX_EPOCH).  feed() is a single store(Release); check() is a single
// load(Acquire) followed immediately by arithmetic.  There is no unlock
// window in which a heartbeat can slip in undetected.
//
// # Loom model
//
// Wall-clock time cannot be used in loom.  We model it with an abstract
// tick counter passed as an argument.  The watchdog is pre-initialized
// at construction time with a tick that is already within the timeout
// window of the check tick, mirroring the production code that initialises
// `last_heartbeat_ns = now_ns()` at creation.
//
// The key property tested: feed() is a single store(Release) and check() is
// a single load(Acquire) + arithmetic.  For any value that B observes from
// that load — either the pre-init tick or a later feed tick — the elapsed
// computation is correct and consistent.  No "in-between" value is ever
// observed because the load and arithmetic are in the same thread with no
// yield points between them.

struct LoomWatchdog {
    /// Tick at which the watchdog was last fed (Release store in feed).
    last_feed_tick: AtomicU64,
    /// Timeout in abstract ticks.
    timeout_ticks: u64,
}

impl LoomWatchdog {
    /// Create a watchdog pre-initialised at `init_tick` (mirrors production
    /// `last_heartbeat_ns = now_ns()` at construction time).
    fn new(timeout_ticks: u64, init_tick: u64) -> Arc<Self> {
        Arc::new(Self {
            last_feed_tick: AtomicU64::new(init_tick),
            timeout_ticks,
        })
    }

    /// Feed the watchdog at the given virtual tick (mirrors production store(Release)).
    fn feed(&self, current_tick: u64) {
        self.last_feed_tick.store(current_tick, Ordering::Release);
    }

    /// Check if the watchdog has expired at `current_tick`.
    ///
    /// Single Acquire load + arithmetic: no window for a heartbeat to slip in.
    fn check(&self, current_tick: u64) -> bool {
        let last = self.last_feed_tick.load(Ordering::Acquire);
        current_tick.saturating_sub(last) > self.timeout_ticks
    }
}

/// Loom test: when BOTH the initial feed tick AND Thread A's feed tick are
/// within the timeout window, check() must never report expiry regardless
/// of which value it observes.
///
/// Timeout = 10. Init = 90. A feeds at 95. B checks at 99.
///   - If B loads 90 (init): elapsed = 9 ≤ 10 → not expired ✓
///   - If B loads 95 (A's store): elapsed = 4 ≤ 10 → not expired ✓
///
/// This verifies the single-load atomicity guarantee: any value B observes
/// from the one Acquire load leads to a consistent, correct result.
#[test]
fn loom_watchdog_both_feeds_within_timeout_no_expiry() {
    loom::model(|| {
        // Init at tick 90; timeout = 10; check tick = 99.
        // Both 90 and 95 are fresh (elapsed ≤ 10 from tick 99).
        let wd = LoomWatchdog::new(10, 90);

        let wd_a = wd.clone();
        let a = loom::thread::spawn(move || {
            wd_a.feed(95); // refresh — still within timeout at check tick 99
        });

        let wd_b = wd.clone();
        let b = loom::thread::spawn(move || {
            // B sees either 90 or 95; both give elapsed ≤ 10.
            let expired = wd_b.check(99);
            assert!(
                !expired,
                "check(99) must not expire: init=90 and feed=95 are both within timeout 10"
            );
        });

        a.join().unwrap();
        b.join().unwrap();
    });
}

/// Loom test: two concurrent feed() calls; check() after join must observe
/// the LATER of the two feeds (or the earlier — both are within timeout).
///
/// Timeout = 20. Thread A feeds at 100. Thread B feeds at 110.
/// Check at tick 115 after both join: either value → elapsed ≤ 20 → not expired.
#[test]
fn loom_watchdog_two_concurrent_feeds_both_within_timeout() {
    loom::model(|| {
        let wd = LoomWatchdog::new(20, 95); // init within timeout of check=115

        let wd_a = wd.clone();
        let a = loom::thread::spawn(move || {
            wd_a.feed(100);
        });

        let wd_b = wd.clone();
        let b = loom::thread::spawn(move || {
            wd_b.feed(110);
        });

        a.join().unwrap();
        b.join().unwrap();

        // Sequential check after both threads done: see either 100 or 110.
        // elapsed = 115 - 100 = 15 ≤ 20 or 115 - 110 = 5 ≤ 20.
        // Also init=95: elapsed = 20 ≤ 20 (≤ not >, so not expired).
        let expired = wd.check(115);
        assert!(
            !expired,
            "check must not expire; all possible feeds within timeout 20"
        );
    });
}

/// Loom test: check() correctly reports expiry when the init tick is far
/// in the past and no concurrent feed() fires.
///
/// Confirms the implementation is not trivially "always false".
#[test]
fn loom_watchdog_expired_when_feed_too_old() {
    loom::model(|| {
        // Init at tick 0; timeout = 10; check at tick 100. No feed.
        let wd = LoomWatchdog::new(10, 0);

        let wd2 = wd.clone();
        let checker = loom::thread::spawn(move || {
            let expired = wd2.check(100);
            assert!(
                expired,
                "check must expire when elapsed (100) > timeout (10)"
            );
        });

        checker.join().unwrap();
    });
}

/// Loom test: feed() happens-before check() via join → check must observe
/// the fed value and report no expiry.
///
/// Thread A: feed(99) then signals done via Mutex+CondVar.
/// Thread B: waits for done, then checks at 104 with timeout 10.
///
/// Because A's feed is sequenced-before A's store to the done flag, and
/// B's load of the done flag synchronises-with A's store, B must see
/// last_feed_tick = 99 → elapsed = 5 ≤ 10 → not expired.
#[test]
fn loom_watchdog_feed_join_check_no_expiry() {
    loom::model(|| {
        // Init at tick 0; timeout = 10; A feeds at 99; check at 104.
        // Without join, B might see 0 → elapsed = 104 > 10 → expired.
        // With join, B must see 99 → elapsed = 5 ≤ 10 → not expired.
        let wd = LoomWatchdog::new(10, 0);

        let wd_a = wd.clone();
        let a = loom::thread::spawn(move || {
            wd_a.feed(99);
        });

        // join establishes happens-before: A's feed(99) is visible to B.
        a.join().unwrap();

        let expired = wd.check(104);
        assert!(
            !expired,
            "after join, check must see feed(99); elapsed=5 ≤ timeout=10"
        );
    });
}
