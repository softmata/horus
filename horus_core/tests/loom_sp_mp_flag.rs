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
//! # Two coupled properties, two gates in this file
//!
//! The SP flag write alone is CORRECT-BUT-INSUFFICIENT: it makes SpscShm messages
//! MpscShm-readable, but the MPSC recv is flag-gated and the flag encodes only
//! present-vs-absent per slot, never consumed-vs-unconsumed. Consumed-ness lives
//! only in `tail`, and the shared `header.tail` is BATCHED (flushed every
//! `capacity/2` reads), so on a migration resync `sync_local` could regress a
//! consumer's tail behind its true position and re-deliver already-consumed slots
//! (softmata-brain 1327 scenario X — a stale command re-sent to hardware). The fix
//! therefore has TWO parts, each gated here:
//!
//!   1. `run*` (loom_sp_mp_*): the SP send path publishes the per-slot ready flag,
//!      correctly ordered (data before flag-Release; flag-Acquire before data).
//!      Gate: delete the flag store -> RED (message loss).
//!   2. `run_resync` (loom_resync_*): the resync tail policy takes
//!      `max(local_tail, header_tail)` so the consumed frontier never rewinds.
//!      Gate: adopt `header_tail` directly -> RED (re-delivery across resync).
//!
//! The real-code counterpart of gate 2 is the deterministic
//! `same_consumer_no_redelivery_across_sp_to_mp_migration` test (exercises the
//! actual `sync_local` + `force_migrate` path). Data-plane-CHANGE resyncs
//! (intra<->SHM, capacity grow) still reset the tail (positions are rewritten);
//! that discrimination is `is_cross_process()`-gated in `sync_local`, not modeled
//! here (it is a mode check, not a concurrency property).
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
    /// The BATCHED shared consumed position (`header.tail`). SPSC/MPSC recv flush
    /// it only every capacity/2 reads, so it lags a single consumer's true
    /// position. The resync scenario keeps it at 0 (never flushed) to model that
    /// lag at its worst.
    header_tail: AtomicU64,
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
            header_tail: AtomicU64::new(0),
            ready: std::array::from_fn(|_| AtomicU64::new(0)),
            data: std::array::from_fn(|_| AtomicU64::new(0)),
        }
    }

    /// Drain every slot that is currently ready, starting at `tail`, appending the
    /// delivered POSITION to `out` and returning the advanced tail. Non-spinning:
    /// it stops at the first not-yet-ready slot, so it always terminates (bounded
    /// by the number of ready slots). Models a single MPSC consumer's read burst.
    fn drain_ready(&self, mut tail: u64, out: &mut Vec<u64>) -> u64 {
        loop {
            let head = self.head.load(Ordering::Acquire);
            if head.wrapping_sub(tail) == 0 {
                break;
            }
            let idx = (tail & Self::MASK) as usize;
            if self.ready[idx].load(Ordering::Acquire) != tail.wrapping_add(1) {
                break;
            }
            let val = self.data[idx].load(Ordering::Relaxed);
            assert_eq!(val, tail + 1, "torn read at position {tail}: value {val}");
            out.push(tail);
            tail = tail.wrapping_add(1);
        }
        tail
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

/// Scenario X (softmata-brain 1327): a consumer that has already consumed
/// messages must NOT be re-delivered them across a migration resync.
///
/// Models `sync_local`'s data-plane-aware tail policy. The consumer reads a burst,
/// then RESYNCS — recomputing its tail from its own position and the batched
/// shared `header_tail` (which lags, kept at 0 here). The production fix takes
/// `max(local_tail, header_tail)`, so the tail never rewinds; then it reads on.
/// The delivered positions across BOTH bursts must contain no duplicate — a
/// re-delivered position is a stale command re-sent to hardware.
///
/// GATE: change the resync line below from `.max(...)` to adopting `header_tail`
/// directly (the old, buggy `sync_local` behavior) and this goes RED — the second
/// burst re-delivers positions the first already yielded.
fn run_resync<const CAP: usize>(sends: u64) {
    assert!(sends <= CAP as u64, "model is faithful only without slot reuse");
    let mut builder = loom::model::Builder::new();
    builder.preemption_bound = Some(3);
    builder.check(move || {
        let ring = Arc::new(SpRing::<CAP>::new());

        let p = ring.clone();
        let producer = loom::thread::spawn(move || {
            for pos in 0..sends {
                p.sp_send(pos + 1);
            }
        });

        let c = ring.clone();
        let consumer = loom::thread::spawn(move || {
            let mut delivered = Vec::new();
            // First read burst (races the producer): drains whatever is ready.
            let tail = c.drain_ready(0, &mut delivered);
            // Resync (models sync_local across a same-data-plane migration). The
            // fix: never rewind behind our own consumed position. header_tail lags
            // (0), so this is a no-op here — which is exactly the point: the old
            // code adopted header_tail and rewound to 0.
            let tail = tail.max(c.header_tail.load(Ordering::Acquire));
            // Second read burst after resync.
            let _ = c.drain_ready(tail, &mut delivered);
            delivered
        });

        producer.join().unwrap();
        let delivered = consumer.join().unwrap();

        // No position delivered twice across the resync.
        let mut seen = std::collections::BTreeSet::new();
        for &pos in &delivered {
            assert!(
                seen.insert(pos),
                "re-delivered position {pos} across resync: {delivered:?}"
            );
        }
    });
}

#[test]
fn loom_resync_no_redelivery_cap2() {
    run_resync::<2>(2);
}

#[test]
fn loom_resync_no_redelivery_cap4() {
    run_resync::<4>(3);
}
