//! Loom-based exhaustive concurrency test for the drop-oldest (latest-wins)
//! seqlock fanout ring shared by `FanoutRing` (heap) and `ShmFanoutRing` (SHM).
//!
//! This models the per-slot-versioned SPSC protocol in
//! `horus_core::communication::topic::seqlock` with loom's atomics, exploring
//! producer/consumer interleavings — including the producer OVERWRITING the very
//! slot the consumer is copying (the torn-read window).
//!
//! # What is verified
//!
//! A single consumer read that races a producer must return either `None` or a
//! value the producer actually sent — NEVER a torn/garbage value, and never a
//! stale value while the slot's version says otherwise. (Cross-read FIFO order is
//! a *sequential* property of the single consumer, exercised by the real
//! multi-thread stress tests in `fanout.rs` / `shm_fanout.rs`, not a concurrency
//! property loom needs to explore.)
//!
//! # Modeling notes
//!
//! - Each data slot is an `AtomicU64` (Relaxed) so loom does not flag the seqlock's
//!   intentional concurrent slot access; the version-stamp protocol governs which
//!   reads are accepted. A distinct nonzero value per position lets a torn read be
//!   caught as "value that doesn't match its position".
//! - The producer/consumer use the Boehm seqlock fence pairing (release fence after
//!   the writing marker; acquire fence after the data read) — the same ordering the
//!   production `seqlock` module uses — so a consumer that observes overwritten data
//!   is forced to also observe the newer version stamp and reject it.
//! - A preemption bound keeps exploration tractable while still covering the
//!   overwrite-during-read interleavings.
//!
//! Run with: `cargo test --test loom_fanout -- --nocapture`

use loom::sync::atomic::{fence, AtomicU64, Ordering};
use loom::sync::Arc;

/// A capacity-`CAP` seqlock ring (single producer, single consumer), mirroring
/// `topic::seqlock`. Data slots are `AtomicU64` for loom (see module note).
struct SeqRing<const CAP: usize> {
    head: AtomicU64,
    tail: AtomicU64,
    versions: [AtomicU64; CAP],
    data: [AtomicU64; CAP],
}

impl<const CAP: usize> SeqRing<CAP> {
    const MASK: u64 = (CAP as u64) - 1;

    fn new() -> Self {
        assert!(CAP.is_power_of_two());
        Self {
            head: AtomicU64::new(0),
            tail: AtomicU64::new(0),
            versions: std::array::from_fn(|_| AtomicU64::new(0)),
            data: std::array::from_fn(|_| AtomicU64::new(0)),
        }
    }

    /// Producer: publish `val` at the next position (never fails, overwrites).
    /// Mirrors `seqlock_publish` (Boehm fence pairing).
    fn publish(&self, val: u64) {
        let pos = self.head.load(Ordering::Relaxed);
        let idx = (pos & Self::MASK) as usize;
        self.versions[idx].store((pos << 1) | 1, Ordering::Relaxed); // writing (odd)
        fence(Ordering::Release);
        self.data[idx].store(val, Ordering::Relaxed);
        self.versions[idx].store(pos << 1, Ordering::Release); // done, position-tagged
        self.head.store(pos.wrapping_add(1), Ordering::Release);
    }

    /// Consumer: read the next available value (latest-wins), or `None`.
    /// Mirrors `seqlock_consume`. Returns `(value, position)` so the test can
    /// verify the value matches the position it was delivered for.
    fn consume(&self) -> Option<(u64, u64)> {
        let mut next = self.tail.load(Ordering::Acquire);
        for _ in 0..4 {
            let h = self.head.load(Ordering::Acquire);
            let avail = h.wrapping_sub(next);
            if avail == 0 {
                return None;
            }
            if avail > CAP as u64 {
                next = h.wrapping_sub(CAP as u64);
            }
            let idx = (next & Self::MASK) as usize;
            let expected = next << 1;
            if self.versions[idx].load(Ordering::Acquire) != expected {
                continue;
            }
            let val = self.data[idx].load(Ordering::Relaxed);
            fence(Ordering::Acquire);
            if self.versions[idx].load(Ordering::Relaxed) != expected {
                continue; // torn/lapped — discard
            }
            self.tail.store(next.wrapping_add(1), Ordering::Release);
            return Some((val, next));
        }
        None
    }
}

/// One producer sending `SENDS` values while one consumer does a single read.
/// The consumer must return `None` or a `(value, position)` where the value is
/// exactly the one sent for that position — i.e. never a torn/garbage read.
fn run<const CAP: usize>(sends: u64) {
    // Preemption bound keeps the interleaving space tractable while still
    // covering the overwrite-during-read window this test targets.
    let mut builder = loom::model::Builder::new();
    builder.preemption_bound = Some(3);
    builder.check(move || {
        let ring = Arc::new(SeqRing::<CAP>::new());

        let p = ring.clone();
        let producer = loom::thread::spawn(move || {
            // Position `pos` carries value `pos + 1` (all nonzero, distinct), so a
            // stale slot (0) can never masquerade as a real message and a value
            // uniquely identifies its position.
            for pos in 0..sends {
                p.publish(pos + 1);
            }
        });

        let c = ring.clone();
        let consumer = loom::thread::spawn(move || c.consume());

        producer.join().unwrap();
        if let Some((val, pos)) = consumer.join().unwrap() {
            assert_eq!(
                val,
                pos + 1,
                "torn read: value {val} does not match its position {pos}"
            );
        }
    });
}

// ─────────────────────────────────────────────────────────────────────────────
// Drop-ownership model: is producer-drop-on-overwrite safe? (roadmap
// roadmap-mrgqzlmb-ixl127 phase 2)
//
// For a non-POD payload each heap allocation must be dropped EXACTLY ONCE — by the
// consumer if it cleanly reads the value, or by the producer if it overwrites the
// value before the consumer takes it, never both (double-free) and never neither
// (leak). This models the candidate "producer drops the shed value on overwrite"
// with the ordering the advisor suggested (odd-stamp BEFORE sampling tail) and asks
// loom whether any interleaving double-frees a position (drops[p] > 1).
// ─────────────────────────────────────────────────────────────────────────────

/// Same seqlock ring, plus a per-position drop counter. `publish` drops the value
/// it sheds on overwrite iff the slot's prior position looks unconsumed
/// (`p >= tail`); `consume` drops the value it cleanly reads. Exactly-once ⇒ every
/// counter is ≤ 1.
struct DropRing<const CAP: usize> {
    head: AtomicU64,
    tail: AtomicU64,
    versions: [AtomicU64; CAP],
    data: [AtomicU64; CAP],
    drops: [AtomicU64; 4],
}

impl<const CAP: usize> DropRing<CAP> {
    const MASK: u64 = (CAP as u64) - 1;

    fn new() -> Self {
        assert!(CAP.is_power_of_two());
        Self {
            head: AtomicU64::new(0),
            tail: AtomicU64::new(0),
            versions: std::array::from_fn(|_| AtomicU64::new(0)),
            data: std::array::from_fn(|_| AtomicU64::new(0)),
            drops: std::array::from_fn(|_| AtomicU64::new(0)),
        }
    }

    fn publish(&self, val: u64) {
        let pos = self.head.load(Ordering::Relaxed);
        let idx = (pos & Self::MASK) as usize;
        self.versions[idx].store((pos << 1) | 1, Ordering::Relaxed); // writing (odd)
        fence(Ordering::Release);
        // Candidate drop-on-overwrite: the shed slot held position `pos - CAP`.
        // Odd-stamp for `pos` is already published above, so (the claim under test)
        // any consumer that still commits `pos - CAP` should be observable via tail.
        if pos >= CAP as u64 {
            let shed = pos - CAP as u64;
            let t = self.tail.load(Ordering::Acquire);
            if shed >= t {
                // Looks unconsumed → producer reclaims the shed value.
                self.drops[shed as usize].fetch_add(1, Ordering::Relaxed);
            }
        }
        self.data[idx].store(val, Ordering::Relaxed);
        self.versions[idx].store(pos << 1, Ordering::Release); // done
        self.head.store(pos.wrapping_add(1), Ordering::Release);
    }

    fn consume(&self) -> Option<u64> {
        let mut next = self.tail.load(Ordering::Acquire);
        for _ in 0..4 {
            let h = self.head.load(Ordering::Acquire);
            let avail = h.wrapping_sub(next);
            if avail == 0 {
                return None;
            }
            if avail > CAP as u64 {
                next = h.wrapping_sub(CAP as u64);
            }
            let idx = (next & Self::MASK) as usize;
            let expected = next << 1;
            if self.versions[idx].load(Ordering::Acquire) != expected {
                continue;
            }
            let _val = self.data[idx].load(Ordering::Relaxed);
            fence(Ordering::Acquire);
            if self.versions[idx].load(Ordering::Relaxed) != expected {
                continue;
            }
            self.tail.store(next.wrapping_add(1), Ordering::Release);
            // Consumer took a clean bitwise copy of `next` → it owns and drops it.
            self.drops[next as usize].fetch_add(1, Ordering::Relaxed);
            return Some(next);
        }
        None
    }
}

fn run_drop<const CAP: usize>(sends: u64) {
    let mut builder = loom::model::Builder::new();
    builder.preemption_bound = Some(3);
    builder.check(move || {
        let ring = Arc::new(DropRing::<CAP>::new());
        let p = ring.clone();
        let producer = loom::thread::spawn(move || {
            for pos in 0..sends {
                p.publish(pos + 1);
            }
        });
        let c = ring.clone();
        let consumer = loom::thread::spawn(move || c.consume());
        producer.join().unwrap();
        consumer.join().unwrap();
        // The safety property: no position is dropped more than once (no double-free).
        for (pos, d) in ring.drops.iter().enumerate() {
            let n = d.load(Ordering::Relaxed);
            assert!(n <= 1, "double-free: position {pos} dropped {n} times");
        }
    });
}

#[test]
#[should_panic(expected = "double-free")]
fn loom_proof_producer_reclaim_on_overwrite_double_frees() {
    // NEGATIVE PROOF (roadmap roadmap-mrgqzlmb-ixl127 phase 2): producer-side
    // reclaim of a shed value on overwrite is UNSAFE and cannot be fixed by fence
    // tuning. Loom finds an interleaving where the consumer cleanly bitwise-copies
    // pos0 (its version recheck reads the old stamp BEFORE the producer's odd-stamp)
    // while the producer, having stamped odd and sampled tail == 0, also reclaims
    // pos0 → the same heap allocation is dropped twice.
    //
    // Root cause: a bitwise-copy seqlock consumer can cleanly copy a value the
    // producer is simultaneously shedding, and nothing orders the producer's tail
    // sample after the consumer's tail advance in that window. This is why
    // SpscChannel::send does NOT drop on overwrite; the shed value is leaked instead
    // (see fanout.rs). This test guards against a future naive "just drop it" fix.
    run_drop::<1>(2);
}

#[test]
fn loom_fanout_cap1_overwrite() {
    // Single slot: pos1 overwrites pos0 in the same slot — the tightest
    // overwrite-during-read window.
    run::<1>(2);
}

#[test]
fn loom_fanout_cap2_no_lap() {
    // Two sends fill a 2-slot ring exactly (no lap) but still race the reader.
    run::<2>(2);
}

#[test]
fn loom_fanout_cap2_lap() {
    // Three sends into a 2-slot ring force pos2 to overwrite slot 0 while the
    // consumer may be reading it — lap + torn detection.
    run::<2>(3);
}
