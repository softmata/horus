//! Loom-based exhaustive concurrency tests for Watchdog, PRNG, and
//! DeterministicClock concurrency primitives.
//!
//! These tests complement the ring-buffer tests in `loom_ring_buffers.rs`.
//! Together these files provide loom coverage for all concurrency fixes in
//! the HORUS Security & Safety Hardening audit.
//!
//! Each primitive is re-implemented with loom's instrumented atomics so loom
//! can track every load/store/CAS and exhaustively verify all interleavings.
//!
//! # Primitives covered
//!
//! | File                     | Primitive          | Fix verified                                  |
//! |:-------------------------|:-------------------|:----------------------------------------------|
//! | `loom_seqlock.rs`*       | HFrame seqlock     | Release/Acquire ordering, no torn reads       |
//! | **this file**            | Watchdog           | AtomicU64 heartbeat, no TOCTOU false-positive |
//! | **this file**            | PRNG (xorshift64)  | CAS loop, unique values across threads        |
//! | **this file**            | DeterministicClock | fetch_add monotonicity, no lost tick updates  |
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

// ── SECTION 2: PRNG — xorshift64 CAS loop (unique values per thread) ─────────
//
// # Fix under test
//
// DeterministicClock::random_u64() uses a CAS loop to atomically advance
// the xorshift64 state.  The old implementation used load + compute + store
// (three separate operations), which had a lost-update race: two threads
// loading the same `old_state` would compute the same `new_state`, both
// store it, and return identical random numbers.
//
// The CAS-loop fix guarantees: only the thread that wins the CAS may use
// that particular `old_state` as the basis for its return value. Losers
// reload and retry, obtaining a different `old_state` and thus a different
// `new_state`.  Every returned value is unique within the xorshift64 cycle.
//
// # Loom model
//
// Re-implement `random_u64()` with loom AtomicU64.  Run two threads
// concurrently; assert the returned values differ.

/// xorshift64 bijection — same formula as DeterministicClock::random_u64.
#[inline(always)]
fn xorshift64(mut x: u64) -> u64 {
    x ^= x << 13;
    x ^= x >> 7;
    x ^= x << 17;
    x
}

struct LoomPrng {
    state: AtomicU64,
}

impl LoomPrng {
    fn new(seed: u64) -> Arc<Self> {
        Arc::new(Self {
            state: AtomicU64::new(seed),
        })
    }

    /// CAS-loop xorshift64 — mirrors DeterministicClock::random_u64.
    fn next(&self) -> u64 {
        loop {
            let old = self.state.load(Ordering::Acquire);
            let new = xorshift64(old);
            if self
                .state
                .compare_exchange(old, new, Ordering::AcqRel, Ordering::Acquire)
                .is_ok()
            {
                return new;
            }
            // Another thread advanced the state; retry with the new value.
        }
    }
}

/// Loom test: two concurrent calls to random_u64() must return distinct values.
///
/// Invariant: because xorshift64 is a bijection and the CAS ensures each thread
/// operates on a distinct `old_state`, both returned values are different.
#[test]
fn loom_prng_concurrent_calls_return_unique_values() {
    loom::model(|| {
        let prng = LoomPrng::new(42);

        let p1 = prng.clone();
        let t1 = loom::thread::spawn(move || p1.next());

        let p2 = prng.clone();
        let t2 = loom::thread::spawn(move || p2.next());

        let v1 = t1.join().unwrap();
        let v2 = t2.join().unwrap();

        assert_ne!(
            v1, v2,
            "concurrent random_u64() calls must return distinct values; both returned {}",
            v1
        );
    });
}

/// Loom test: two sequential calls (single thread) return distinct values.
///
/// Basic sanity check: xorshift64 period is 2^64−1 so consecutive values
/// from the same seed are always different.
#[test]
fn loom_prng_sequential_calls_return_unique_values() {
    loom::model(|| {
        let prng = LoomPrng::new(1);
        let v1 = prng.next();
        let v2 = prng.next();
        assert_ne!(
            v1, v2,
            "sequential random_u64() calls must return distinct values"
        );
    });
}

/// Loom test: three concurrent calls all return distinct values.
///
/// Uses loom's 3-thread model.  With 3 threads, loom explores all 3! = 6
/// execution orderings plus interleaved variants.  In every outcome, the
/// CAS serialises them so each sees a unique old_state.
#[test]
fn loom_prng_three_concurrent_calls_all_unique() {
    loom::model(|| {
        let prng = LoomPrng::new(0xDEAD_BEEF_CAFE_BABE);

        let p1 = prng.clone();
        let t1 = loom::thread::spawn(move || p1.next());

        let p2 = prng.clone();
        let t2 = loom::thread::spawn(move || p2.next());

        let p3 = prng.clone();
        let t3 = loom::thread::spawn(move || p3.next());

        let mut values = vec![t1.join().unwrap(), t2.join().unwrap(), t3.join().unwrap()];
        values.sort();
        values.dedup();

        assert_eq!(
            values.len(),
            3,
            "all 3 concurrent random_u64() calls must return distinct values"
        );
    });
}

// ── SECTION 3: DeterministicClock — advance_tick monotonicity ─────────────────
//
// # Fix under test
//
// The original advance_tick() used:
//   let tick = self.tick.fetch_add(1, Relaxed);
//   self.virtual_time_ns.store(tick * duration, Relaxed);  // ← WRONG
//
// The store() on virtual_time_ns is not atomic with respect to the fetch_add()
// on tick: if thread A fetches tick=99 and thread B fetches tick=100, and B's
// store (100 * T) commits before A's store (99 * T), A's store overwrites B's,
// making virtual_time_ns go backwards.
//
// The fix uses fetch_add(AcqRel) on virtual_time_ns (same as tick), so each
// advance adds exactly one `tick_duration_ns` to the total, regardless of
// scheduling order.  Virtual time is now a monotonically increasing sum, not
// a racy product.
//
// # Loom model
//
// Re-implement just the atomic operations using loom AtomicU64.  Two threads
// each call advance_tick() once.  Assertions:
//   - final tick == 2 (exactly 2 fetch_adds)
//   - final virtual_time == 2 * TICK_DURATION (no lost update)
//   - Each call returns a distinct tick value (no duplicate tick numbers)

const TICK_DURATION: u64 = 1_000_000; // 1 ms in ns (arbitrary for test)

struct LoomClock {
    tick: AtomicU64,
    virtual_time_ns: AtomicU64,
}

impl LoomClock {
    fn new() -> Arc<Self> {
        Arc::new(Self {
            tick: AtomicU64::new(0),
            virtual_time_ns: AtomicU64::new(0),
        })
    }

    /// Advance by one tick, returns the new tick number.
    ///
    /// Uses fetch_add(AcqRel) on both counters so concurrent calls each
    /// contribute exactly once to both counters (no lost updates, no ABA).
    fn advance_tick(&self) -> u64 {
        // Return value is new_tick (fetch returns old, so add 1).
        let new_tick = self.tick.fetch_add(1, Ordering::AcqRel) + 1;
        self.virtual_time_ns
            .fetch_add(TICK_DURATION, Ordering::AcqRel);
        new_tick
    }

    fn now_ns(&self) -> u64 {
        self.virtual_time_ns.load(Ordering::Acquire)
    }

    fn current_tick(&self) -> u64 {
        self.tick.load(Ordering::Acquire)
    }
}

/// Loom test: two concurrent advance_tick() calls; final tick == 2 and
/// virtual_time == 2 * TICK_DURATION (no lost update on either counter).
#[test]
fn loom_clock_two_concurrent_ticks_no_lost_update() {
    loom::model(|| {
        let clock = LoomClock::new();

        let c1 = clock.clone();
        let t1 = loom::thread::spawn(move || c1.advance_tick());

        let c2 = clock.clone();
        let t2 = loom::thread::spawn(move || c2.advance_tick());

        let tick_a = t1.join().unwrap();
        let tick_b = t2.join().unwrap();

        // Tick values must be distinct (1 and 2 in some order; never both 1).
        assert_ne!(
            tick_a, tick_b,
            "two concurrent advance_tick() calls must return distinct tick values; \
             both returned {}",
            tick_a
        );
        let mut ticks = [tick_a, tick_b];
        ticks.sort();
        assert_eq!(
            ticks,
            [1, 2],
            "tick values must be {{1, 2}} in some order, got {:?}",
            ticks
        );

        // Final counters: exactly 2 fetch_adds applied to each.
        assert_eq!(
            clock.current_tick(),
            2,
            "tick counter must be 2 after two advance_tick() calls"
        );
        assert_eq!(
            clock.now_ns(),
            2 * TICK_DURATION,
            "virtual_time_ns must be 2 * TICK_DURATION; no lost update"
        );
    });
}

/// Loom test: one advance_tick() and one now_ns() racing concurrently.
///
/// Invariant: now_ns() returns either 0 (snapshot before advance) or
/// TICK_DURATION (snapshot after advance).  It must never return any
/// other value (no partial update visible).
///
/// This specifically guards against the old `store(tick * duration)` pattern
/// where an intermediate store could produce an arbitrary value if the tick
/// read was stale.  With fetch_add, the only observable values are 0 and T.
#[test]
fn loom_clock_advance_tick_and_read_concurrent() {
    loom::model(|| {
        let clock = LoomClock::new();

        let c1 = clock.clone();
        let advancer = loom::thread::spawn(move || {
            c1.advance_tick();
        });

        let c2 = clock.clone();
        let reader = loom::thread::spawn(move || c2.now_ns());

        advancer.join().unwrap();
        let t = reader.join().unwrap();

        // Acceptable values: 0 (read before advance) or TICK_DURATION (after).
        assert!(
            t == 0 || t == TICK_DURATION,
            "now_ns() must be 0 or TICK_DURATION; got {} (no partial update allowed)",
            t
        );
    });
}

/// Loom test: three concurrent advance_tick() calls; all tick values distinct,
/// final tick == 3, final virtual_time == 3 * TICK_DURATION.
#[test]
fn loom_clock_three_concurrent_ticks_exact_count() {
    loom::model(|| {
        let clock = LoomClock::new();

        let c1 = clock.clone();
        let t1 = loom::thread::spawn(move || c1.advance_tick());

        let c2 = clock.clone();
        let t2 = loom::thread::spawn(move || c2.advance_tick());

        let c3 = clock.clone();
        let t3 = loom::thread::spawn(move || c3.advance_tick());

        let mut ticks = vec![t1.join().unwrap(), t2.join().unwrap(), t3.join().unwrap()];
        ticks.sort();
        assert_eq!(
            ticks,
            [1, 2, 3],
            "three concurrent advance_tick() calls must return {{1, 2, 3}}"
        );

        assert_eq!(clock.current_tick(), 3);
        assert_eq!(
            clock.now_ns(),
            3 * TICK_DURATION,
            "virtual_time_ns must equal 3 * TICK_DURATION; no tick lost"
        );
    });
}
