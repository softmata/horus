//! Loom-based exhaustive concurrency test for the SpscShm -> MpscShm ready-flag
//! conversion (softmata-brain 1327, the sp->mp flag gap).
//!
//! # Background
//!
//! A topic written under `SpscShm` uses `send_shm_sp_pod`, which publishes each
//! message via `header.sequence_or_head` (Release). When a second producer joins,
//! the topic migrates to `MpscShm`, and the consumer switches to
//! `recv_shm_mpsc_pod`, which gates on the *per-slot ready-flag array*
//! (`seq_array[idx] == tail+1`), NOT on `sequence_or_head`. If the SP send path
//! never wrote those per-slot flags, every message buffered under `SpscShm`
//! becomes permanently unreadable after the switch — the bug.
//!
//! The fix makes `send_shm_sp_pod` publish the per-slot ready flag too (one extra
//! Release store by the same single producer). This test models exactly that send
//! path and an MPSC consumer, and asserts:
//!
//! - **No message loss:** every value the SP producer sent is recoverable by the
//!   MPSC (ready-flag-gated) consumer, in FIFO order.
//! - **No stale/torn read:** the ordering `data.store(Relaxed) -> ready.store(Release)`
//!   in the producer, paired with `ready.load(Acquire) -> data.load(Relaxed)` in the
//!   consumer, means a consumer that observes `ready == tail+1` always observes the
//!   fully-written data (never the zero-initialized slot). Values are `pos+1`
//!   (nonzero, distinct), so a stale slot (0) or a mis-delivered value is caught.
//!
//! # Gate validation
//!
//! Delete the `ready` store in `sp_send` (reverting the fix) and this test goes
//! RED: the post-join deterministic drain finds every `ready` flag still zero,
//! `mp_recv` returns `None`, and the "no message loss" assertion fails. Crucially
//! the model still TERMINATES without the fix (the drain breaks on `None` rather
//! than spinning), so the gate fails cleanly instead of hanging.
//!
//! # SCOPE LIMITATION — why green here is NOT sufficient to ship
//!
//! This model uses a MONOTONIC tail: the single consumer's `tail` only advances.
//! It therefore does NOT model migration/late-join *resync*, where a handle's
//! `local_tail` is reset to the shared `header.tail` (`topic/mod.rs` `sync_local`),
//! which is BATCHED (flushed only every `capacity/2` reads) and so can regress
//! behind the true consumed position. The ready-flag protocol encodes only
//! present-vs-absent per slot, never consumed-vs-unconsumed — that lives in `tail`
//! alone. So a flag-gated consumer reading from a regressed `tail` re-reads
//! already-consumed slots. Making the SP path set flags (the fix this gate proves)
//! widens the set of re-deliverable slots to include consumed ones, turning a
//! previously-masked resync hazard into active stale re-delivery (observed:
//! `migration_lock_data_flows_during_migration` re-delivers a consumed value once
//! the SP flag is written). The eventual full fix must make `header.tail` reliable
//! at resync AND extend this model to cover tail regression. Until then the fix
//! this file guards is CORRECT-BUT-INSUFFICIENT (see softmata-brain 1327).
//!
//! Run with: `cargo test --test loom_sp_mp_flag -- --nocapture`

use loom::sync::atomic::{AtomicU64, Ordering};
use loom::sync::Arc;

/// A capacity-`CAP` single-producer SHM-style ring mirroring the POD dispatch
/// paths. `head` is `sequence_or_head`; `ready` is the per-slot flag array that
/// `recv_shm_mpsc_pod` gates on; `data` slots are `AtomicU64` for loom.
struct SpRing<const CAP: usize> {
    head: AtomicU64,
    tail: AtomicU64,
    ready: [AtomicU64; CAP],
    data: [AtomicU64; CAP],
}

impl<const CAP: usize> SpRing<CAP> {
    const MASK: u64 = (CAP as u64) - 1;

    fn new() -> Self {
        assert!(CAP.is_power_of_two());
        Self {
            head: AtomicU64::new(0),
            tail: AtomicU64::new(0),
            ready: std::array::from_fn(|_| AtomicU64::new(0)),
            data: std::array::from_fn(|_| AtomicU64::new(0)),
        }
    }

    /// Single-producer POD send — models `send_shm_sp_pod` WITH the fix.
    ///
    /// The single producer owns `head` (its cached `local_head`), so it reads it
    /// Relaxed. Data is written first, then the per-slot ready flag (Release),
    /// then `head` (Release) — matching the production ordering.
    fn sp_send(&self, val: u64) {
        let seq = self.head.load(Ordering::Relaxed);
        let idx = (seq & Self::MASK) as usize;
        let new_seq = seq.wrapping_add(1);
        self.data[idx].store(val, Ordering::Relaxed);
        // THE FIX: publish the per-slot ready flag so an MpscShm consumer can read
        // this message after a SpscShm -> MpscShm migration. Delete this line to
        // validate the gate (test must go RED).
        self.ready[idx].store(new_seq, Ordering::Release);
        self.head.store(new_seq, Ordering::Release);
    }

    /// Single-consumer MPSC recv — models `recv_shm_mpsc_pod`.
    ///
    /// Gates on the per-slot ready flag: `ready[idx] == tail+1`. The Acquire load
    /// of `ready` synchronizes with the producer's Release store, so the Relaxed
    /// data load below observes the fully-written slot.
    fn mp_recv(&self) -> Option<(u64, u64)> {
        let tail = self.tail.load(Ordering::Relaxed);
        let head = self.head.load(Ordering::Acquire);
        if head.wrapping_sub(tail) == 0 {
            return None;
        }
        let idx = (tail & Self::MASK) as usize;
        if self.ready[idx].load(Ordering::Acquire) != tail.wrapping_add(1) {
            return None;
        }
        let val = self.data[idx].load(Ordering::Relaxed);
        self.tail.store(tail.wrapping_add(1), Ordering::Release);
        Some((val, tail))
    }
}

/// One SP producer sends `SENDS` values while one MPSC consumer races it, then a
/// deterministic post-join drain recovers the rest. Every value must be delivered
/// exactly once, in FIFO order, with data intact.
///
/// `sends <= CAP` is required: this models the real scenario (a handful of
/// messages buffered under SpscShm that fit in the ring, then read after
/// migration). The production `send_shm_sp_pod` applies backpressure when the ring
/// is full; this model omits it, so `sends > CAP` would show legitimate overwrite
/// loss unrelated to the flag conversion. Ring-full backpressure is covered by the
/// multi-thread stress tests, not this loom gate.
fn run<const CAP: usize>(sends: u64) {
    assert!(sends <= CAP as u64, "model is faithful only without slot reuse");
    let mut builder = loom::model::Builder::new();
    builder.preemption_bound = Some(3);
    builder.check(move || {
        let ring = Arc::new(SpRing::<CAP>::new());

        let p = ring.clone();
        let producer = loom::thread::spawn(move || {
            // Position `pos` carries value `pos + 1` (nonzero, distinct) so a stale
            // slot (0) can never masquerade as a real message and a value uniquely
            // identifies its position.
            for pos in 0..sends {
                p.sp_send(pos + 1);
            }
        });

        let c = ring.clone();
        let consumer = loom::thread::spawn(move || {
            // A SINGLE concurrent recv racing the producer. This is what keeps the
            // interleaving space tractable (a loop would explode it) while still
            // exercising the ordering window: if the consumer observes `ready ==
            // tail+1`, it must also observe the fully-written data, never the
            // zero-initialized slot. Whatever it does or doesn't get, the post-join
            // drain recovers the rest.
            c.mp_recv()
        });

        producer.join().unwrap();
        let mut got = Vec::new();
        if let Some((v, pos)) = consumer.join().unwrap() {
            assert_eq!(v, pos + 1, "torn/misdelivered: value {v} at position {pos}");
            got.push(pos);
        }

        // Post-join deterministic drain: the producer is done (head == sends) and,
        // WITH the fix, every ready flag in [tail, sends) is set, so mp_recv returns
        // Some for each remaining message and None only when truly drained. WITHOUT
        // the fix the flags are zero, mp_recv returns None immediately, and the
        // length assertion below fails — the gate.
        while let Some((v, pos)) = ring.mp_recv() {
            assert_eq!(v, pos + 1, "torn/misdelivered on drain: value {v} at position {pos}");
            got.push(pos);
        }

        // No message lost.
        assert_eq!(
            got.len() as u64,
            sends,
            "message lost across sp->mp flag conversion: recovered {:?} of {sends}",
            got
        );
        // FIFO: positions delivered in order 0,1,2,...
        for (i, &pos) in got.iter().enumerate() {
            assert_eq!(pos, i as u64, "out-of-order delivery: {got:?}");
        }
    });
}

#[test]
fn loom_sp_mp_cap2_one_send() {
    // Single message buffered under SpscShm, read back via the MPSC (flag-gated)
    // path — the minimal flag-conversion case.
    run::<2>(1);
}

#[test]
fn loom_sp_mp_cap2_exact_fill() {
    // Two sends fill a 2-slot ring exactly while the consumer races — every slot's
    // flag must be observed correctly across the interleaving.
    run::<2>(2);
}

#[test]
fn loom_sp_mp_cap4_three() {
    // Three messages buffered in a 4-slot ring (the crash-test shape: a few
    // messages sent under SpscShm, then read after a second producer forces
    // MpscShm), consumer racing the producer.
    run::<4>(3);
}
