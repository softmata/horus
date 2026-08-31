//! Loom model of the MPSC claim path *while a migration resynchronises the
//! consumer's cached position* — the one combination the existing models leave
//! uncovered, and the combination ThreadSanitizer flags.
//!
//! # Why this model exists
//!
//! `a_clone_growing_the_mapping_does_not_strand_its_siblings` trips two TSan
//! warnings. Four ablations bounded the cause:
//!
//! | variant                                   | TSan warnings   |
//! |-------------------------------------------|-----------------|
//! | baseline: 3 producers + wrap + migration  | 2               |
//! | migration removed                         | 0               |
//! | ring wrap removed (capacity 4096)         | 0               |
//! | mapping-retention fix reverted            | 0 — SEGV instead|
//!
//! So it is slot **reuse across a lap during a migration** — not two producers
//! claiming one sequence, which `loom_mp_claim` already rules out.
//!
//! Neither existing model can see it. `loom_mp_claim` reloads `tail` from the
//! ring on every send and keeps no cached state, so nothing can go stale.
//! `loom_migration` models the epoch/lock — the control plane — with no data
//! plane at all. The hazard needs a producer gating on a *cached* tail that a
//! migration then rewrites, which is what this file adds.
//!
//! # What is modelled
//!
//! The consumer mirrors the real `recv` path: a cached `local_tail`, a *batched*
//! publish of that position to the shared `tail` (so the shared value lags the
//! true consumed count), and a mid-drain resync through the real
//! `resynced_tail` formula from `topic/mod.rs:800`, reproduced verbatim below
//! rather than paraphrased.
//!
//! Producers mirror `send_shm_mp_pod`: CAS-claim with an in-loop room check
//! against the *shared* tail, then Release the ready flag.
//!
//! # Gate — this model is not vacuous
//!
//! Removing the `max` from `resynced_tail` (adopting the shared tail wholesale,
//! the exact bug its comment in `topic/mod.rs` describes) turns both cases RED:
//!
//! ```text
//! cap1: read 2 messages but only 1 were sent — delivered twice across the migration
//! cap2: read 3 messages but only 2 were sent — delivered twice across the migration
//! ```
//!
//! That is the same inversion the real comment records from
//! `recv_never_reorders_or_duplicates_when_lapped` ("consumer 6: 23929 then
//! 23497, backward by 432"), reproduced here exhaustively instead of
//! statistically.
//!
//! Two earlier drafts of this file passed *vacuously*, and both are worth
//! naming because each looked correct:
//!
//! 1. Joining the producers before draining. The consumer then never ran
//!    concurrently with a producer, so no slot could be reused under it. Loom
//!    explored the model in 0.03s instead of 33s — the runtime was the tell.
//! 2. Publishing the consumer position on every message. That kept
//!    `shared_tail == local_tail` at all times, making the `max` a no-op, so
//!    the ablation above did NOT go red. Batching is what makes the shared
//!    value lag, and the lag is the whole hazard.
//!
//! # What this does and does not settle
//!
//! Loom models weak memory ordering, so a pass here is evidence the resync
//! preserves conservation on a weakly-ordered machine — which is exactly the
//! evidence x86 TSO cannot provide, and the reason the TSan finding was left
//! open rather than dismissed. It does NOT close that finding: this models a
//! single consumer, where `shared_tail` is that consumer's own position. The
//! broadcast backend, where several consumers hold independent positions and
//! the shared tail trails the slowest, is where the `max` earns its keep and is
//! still unmodelled.
//!
//! # Faithfulness caveat
//!
//! This is a model, and a model that drifts from the real claim path proves
//! something about the model. Two guards against that: `resynced_tail` is
//! copied verbatim and asserted against the same cases the unit tests pin, and
//! the batching + cached-tail structure is what makes the shared tail lag —
//! the property the real consumer has and `loom_mp_claim` does not. What it
//! does NOT model: the mapping swap itself, multiple consumers with
//! independent positions (the broadcast backend), or growth.
//!
//! Run: `cargo test -p horus_core --test loom_migration_data_plane -- --nocapture`

use loom::sync::atomic::{AtomicU64, Ordering};
use loom::sync::Arc;

/// Verbatim from `horus_core::communication::topic::resynced_tail`
/// (topic/mod.rs:800). Kept byte-identical so this model cannot silently drift
/// from the function it is meant to exercise.
fn resynced_tail(local_tail: u64, shared_tail: u64, new_head: u64, delivered: bool) -> u64 {
    if !delivered {
        return shared_tail.min(new_head);
    }
    local_tail.max(shared_tail).min(new_head)
}

struct Ring<const CAP: usize> {
    head: AtomicU64,
    /// The shared consumed-position. Producers gate on this; the consumer
    /// publishes to it in batches, so it lags the consumer's true position.
    tail: AtomicU64,
    ready: [AtomicU64; CAP],
    data: [AtomicU64; CAP],
    /// Bumped to signal a migration; the consumer resyncs when it observes it.
    epoch: AtomicU64,
}

impl<const CAP: usize> Ring<CAP> {
    const MASK: u64 = (CAP as u64) - 1;

    fn new() -> Self {
        assert!(CAP.is_power_of_two());
        Self {
            head: AtomicU64::new(0),
            tail: AtomicU64::new(0),
            ready: std::array::from_fn(|_| AtomicU64::new(0)),
            data: std::array::from_fn(|_| AtomicU64::new(0)),
            epoch: AtomicU64::new(0),
        }
    }

    /// CAS-claim, mirroring `send_shm_mp_pod`. Gates on the SHARED tail, which
    /// is what a producer can actually see.
    fn try_send(&self) -> bool {
        loop {
            let head = self.head.load(Ordering::Acquire);
            let tail = self.tail.load(Ordering::Acquire);
            if head.wrapping_sub(tail) >= CAP as u64 {
                return false; // full — never overshoot
            }
            if self
                .head
                .compare_exchange(head, head + 1, Ordering::Relaxed, Ordering::Relaxed)
                .is_ok()
            {
                let idx = (head & Self::MASK) as usize;
                // Value is position-encoded: an overwrite lands the wrong value.
                self.data[idx].store(head + 1, Ordering::Relaxed);
                self.ready[idx].store(head + 1, Ordering::Release);
                return true;
            }
        }
    }
}

/// Consumer with the real cached-position + batched-publish structure.
struct Consumer<'a, const CAP: usize> {
    ring: &'a Ring<CAP>,
    local_tail: u64,
    delivered: u64,
    seen_epoch: u64,
    unpublished: u64,
}

impl<'a, const CAP: usize> Consumer<'a, CAP> {
    fn new(ring: &'a Ring<CAP>) -> Self {
        Self {
            ring,
            local_tail: 0,
            delivered: 0,
            seen_epoch: 0,
            unpublished: 0,
        }
    }

    /// Mirrors the migration branch: reload head/tail and resync the cached
    /// position through `resynced_tail`.
    fn check_migration(&mut self) {
        let epoch = self.ring.epoch.load(Ordering::Acquire);
        if epoch == self.seen_epoch {
            return;
        }
        self.seen_epoch = epoch;
        let new_head = self.ring.head.load(Ordering::Acquire);
        let shared_tail = self.ring.tail.load(Ordering::Acquire);
        self.local_tail = resynced_tail(self.local_tail, shared_tail, new_head, self.delivered > 0);
    }

    /// Publish the cached position to the shared tail. Batched: the real
    /// consumer does not store on every message, so the shared value lags.
    ///
    /// The batch threshold matters. An earlier draft published on every
    /// message, which made `shared_tail == local_tail` at all times and turned
    /// the `max` in `resynced_tail` into a no-op -- the model then could not
    /// distinguish the real formula from one with the max removed, i.e. it was
    /// vacuous for the property it exists to check. Publishing every other
    /// message is what makes the shared value actually lag.
    const BATCH: u64 = 2;

    fn publish(&mut self) {
        if self.unpublished >= Self::BATCH {
            self.ring.tail.store(self.local_tail, Ordering::Release);
            self.unpublished = 0;
        }
    }

    /// Flush whatever the batch threshold is still holding back.
    fn publish_final(&mut self) {
        if self.unpublished > 0 {
            self.ring.tail.store(self.local_tail, Ordering::Release);
            self.unpublished = 0;
        }
    }

    fn recv(&mut self) -> Option<u64> {
        let head = self.ring.head.load(Ordering::Acquire);
        if head.wrapping_sub(self.local_tail) == 0 {
            return None;
        }
        let idx = (self.local_tail & Ring::<CAP>::MASK) as usize;
        if self.ring.ready[idx].load(Ordering::Acquire) != self.local_tail.wrapping_add(1) {
            return None; // claimed but not yet written
        }
        let val = self.ring.data[idx].load(Ordering::Relaxed);
        // Position-encoded check: a slot reused under us lands the wrong value.
        assert_eq!(
            val,
            self.local_tail + 1,
            "OVERWRITE: slot {idx} held {val} but position {} expects {}. A \
             producer reused this slot before the consumer read it — the \
             migration resync moved the shared tail forward past unread data.",
            self.local_tail,
            self.local_tail + 1
        );
        self.local_tail = self.local_tail.wrapping_add(1);
        self.delivered += 1;
        self.unpublished += 1;
        Some(val)
    }
}

/// One producer thread pair + a consumer that drains across a migration.
/// Three threads total, per the loom job's stated budget.
fn run<const CAP: usize>(sends_per_producer: usize) {
    let mut builder = loom::model::Builder::new();
    builder.preemption_bound = Some(3);
    builder.check(move || {
        let ring = Arc::new(Ring::<CAP>::new());

        let p1 = {
            let r = ring.clone();
            loom::thread::spawn(move || {
                let mut ok = 0;
                for _ in 0..sends_per_producer {
                    if r.try_send() {
                        ok += 1;
                    }
                }
                ok
            })
        };
        let p2 = {
            let r = ring.clone();
            loom::thread::spawn(move || {
                // This producer also triggers the migration mid-stream, so the
                // resync lands while claims are in flight rather than between
                // quiescent phases.
                let mut ok = 0;
                for i in 0..sends_per_producer {
                    if r.try_send() {
                        ok += 1;
                    }
                    if i == 0 {
                        r.epoch.store(1, Ordering::Release);
                    }
                }
                ok
            })
        };

        // Drain CONCURRENTLY with the producers. Joining first (as an earlier
        // draft did) makes the whole model sequential: the consumer can never
        // be mid-slot when a producer reuses it, so the hazard this file exists
        // to explore becomes unreachable and the model passes vacuously.
        let mut c = Consumer::<CAP>::new(&ring);
        let mut got = 0u32;
        let want = (sends_per_producer * 2) as u32;
        // Bounded spin: loom explores interleavings, so this must terminate on
        // every schedule, including ones where a producer never gets to run.
        for _ in 0..(want * 4) {
            c.check_migration();
            if c.recv().is_some() {
                got += 1;
                c.publish();
            }
            if got == want {
                break;
            }
            loom::thread::yield_now();
        }

        let sent: u32 = p1.join().unwrap() + p2.join().unwrap();

        // Anything the producers landed after our last look is still readable.
        loop {
            c.check_migration();
            match c.recv() {
                Some(_) => {
                    got += 1;
                    c.publish();
                }
                None => break,
            }
        }
        c.publish_final();

        // Conservation: the resync may legitimately skip the consumer forward
        // (a documented, counted escape), but it must never let a slot be read
        // with the wrong value — that assert lives in `recv`. Here we only
        // require that we never read MORE than was sent.
        assert!(
            got <= sent,
            "read {got} messages but only {sent} were sent — a slot was \
             delivered twice across the migration boundary"
        );
    });
}

#[test]
fn loom_migration_resync_does_not_expose_unread_slot_cap1() {
    // One slot: every send after the first must wrap onto a slot the consumer
    // may not have read yet. The tightest reuse pressure available.
    run::<1>(2);
}

#[test]
fn loom_migration_resync_does_not_expose_unread_slot_cap2() {
    // Two slots, two producers: reuse still forced, one more interleaving layer.
    run::<2>(2);
}

/// Pins `resynced_tail` against the same cases the in-crate unit tests assert,
/// so a drift in the copy above is caught here rather than silently modelling
/// a different function.
#[test]
fn resynced_tail_copy_matches_the_real_one() {
    assert_eq!(resynced_tail(23_929, 23_497, 30_000, true), 23_929);
    assert_eq!(resynced_tail(0, 5_000, 6_000, false), 5_000);
    assert_eq!(resynced_tail(9_000, 5_000, 6_000, false), 5_000);
    assert_eq!(resynced_tail(23_929, 0, 12, true), 12);
    assert_eq!(resynced_tail(4_096, 4_096, 8_192, true), 4_096);
}
