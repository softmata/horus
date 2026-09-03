//! Shared seqlock ring protocol for **drop-oldest (latest-wins)** fanout.
//!
//! The SHM [`ShmFanoutRing`](super::shm_fanout) uses this single implementation
//! of the per-slot-versioned SPSC ring so the tricky lock-free logic lives in
//! exactly one place (and is covered by one loom model: `tests/loom_fanout.rs`).
//!
//! # Semantics
//!
//! Latest-wins: the producer NEVER blocks and NEVER fails. When the ring is
//! full it overwrites the oldest slot. A slow consumer that has fallen more
//! than `capacity` behind fast-forwards to the newest window (dropping the
//! messages it missed). This is the correct semantic for robotics pub/sub — a
//! 30 Hz node reading a 500 Hz sensor should get the MOST RECENT data, never
//! stale buffered data, and the fast producer is never throttled by a slow
//! subscriber.
//!
//! # Correctness — why this needs a *writing* phase (unlike `pod_broadcast`)
//!
//! Each slot carries an `AtomicU64` version stamp that encodes both the logical
//! position occupying the slot AND whether a write is in progress:
//!
//! ```text
//!   writing position `pos`:  (pos << 1) | 1     (odd)
//!   done, holding `pos`:     (pos << 1)         (even, position-tagged)
//! ```
//!
//! The producer sets the odd (writing) stamp BEFORE touching the data, then the
//! even (done) stamp AFTER. A consumer reading position `next` expects to see
//! exactly `next << 1` both before AND after copying the slot (a seqlock). If
//! the stamp differs on either side, the slot is either mid-write or has been
//! overwritten by a later position (a lap) — so the copy is discarded and the
//! consumer re-evaluates. This closes the torn-read window that the production
//! `recv_shm_pod_broadcast` leaves open (it stamps only after the write and
//! never re-checks, so a consumer can read a slot mid-overwrite).
//!
//! Publication ordering (done-stamp Release, THEN head Release) guarantees that
//! whenever a consumer observes `head > next` the done stamp for `next` is
//! already visible — the producer only advances `head` after stamping done.
//!
//! # Bitwise-copy constraint
//!
//! A seqlock read is a bitwise copy that may have to be discarded (torn/lapped).
//! Discarding must NOT run `Drop` on a possibly-torn value, and the producer's
//! overwrite must NOT run `Drop` on the value it clobbers. Callers therefore
//! pass a `discard` closure: POD/byte payloads drop harmlessly, while the heap
//! ring storing an arbitrary `T` uses `mem::forget` (accepting the same leak on
//! drop-oldest that the previous `send_overwrite` already documented).

use std::sync::atomic::{fence, AtomicU64, Ordering};

/// Bounded retry count for a single `seqlock_consume` call. A consumer only
/// retries when it observes a mid-write or a lap racing its read; a handful of
/// attempts absorbs that transient without spinning unboundedly (a genuinely
/// stuck read simply returns `None` and the polling caller tries again).
pub(super) const SEQLOCK_MAX_ATTEMPTS: usize = 16;

/// Publish `pos` into the ring (single producer, never fails, overwrites oldest).
///
/// `write_slot(idx)` performs the raw data write into slot `idx`; it must use a
/// non-dropping write (e.g. `ptr::write` / `copy_nonoverlapping`) so the value
/// previously in the slot is clobbered without running its destructor.
///
/// # Safety
/// - Exactly one producer thread may call this for a given ring (SPSC-producer).
/// - `versions` must point to an array of at least `mask + 1` `AtomicU64`.
/// - `head` is the producer-owned monotonic position counter.
#[inline]
pub(super) unsafe fn seqlock_publish(
    versions: *const AtomicU64,
    mask: u64,
    head: &AtomicU64,
    pos: u64,
    write_slot: impl FnOnce(usize),
) {
    let idx = (pos & mask) as usize;
    // SAFETY: idx <= mask, so it is within the versions array (caller contract).
    let v = &*versions.add(idx);
    // Mark the slot as being written (odd, position-tagged). A consumer reading
    // this position now sees a stamp != its expected even value and backs off.
    v.store((pos << 1) | 1, Ordering::Relaxed);
    // Release fence (Boehm seqlock): pairs with the consumer's Acquire fence so
    // that a consumer which observes the data written below is forced to also
    // observe this writing marker — closing the torn-read window where the reader
    // could otherwise see new data alongside the old (matching) version stamp.
    // Verified by tests/loom_fanout.rs (the naive all-Release form fails loom).
    fence(Ordering::Release);
    write_slot(idx);
    // Publish: done stamp (even) makes the data visible to an Acquire loader,
    // then head advance authorizes the consumer to look at `pos` at all.
    v.store(pos << 1, Ordering::Release);
    head.store(pos.wrapping_add(1), Ordering::Release);
}

/// Consume the next available position (single consumer), or `None` if empty.
///
/// `tail` is the consumer-owned position counter (read at entry, advanced past
/// the returned message). `read_slot(idx)` returns a bitwise copy of slot `idx`;
/// `discard(val)` disposes of a copy that turned out to be torn/lapped WITHOUT
/// running a destructor that would be unsound on torn bytes.
///
/// # Safety
/// - Exactly one consumer thread may call this for a given ring (SPSC-consumer).
/// - `versions` must point to an array of at least `mask + 1` `AtomicU64`.
/// - `capacity == mask + 1`.
#[inline]
pub(super) unsafe fn seqlock_consume<R>(
    versions: *const AtomicU64,
    mask: u64,
    capacity: u64,
    head: &AtomicU64,
    tail: &AtomicU64,
    read_slot: impl Fn(usize) -> R,
    discard: impl Fn(R),
    // Incremented by the number of messages skipped when the producer laps the
    // consumer. Drop-oldest is the designed behaviour under overload, but it was
    // previously invisible: nothing counted the gap, and `dropped_count()`
    // reports send failures, which a never-failing producer never has. A slow
    // subscriber lost data with every observability surface reading zero.
    skipped: &mut u64,
) -> Option<R> {
    let mut next = tail.load(Ordering::Acquire);
    for _ in 0..SEQLOCK_MAX_ATTEMPTS {
        let h = head.load(Ordering::Acquire);
        let avail = h.wrapping_sub(next);
        if avail == 0 {
            return None; // caught up — nothing new
        }
        if avail > capacity {
            // Lapped: the producer is more than a full ring ahead, so slot
            // `next` was overwritten. Fast-forward to the oldest still-live
            // position (drop-oldest), counting what we skipped past.
            let resume = h.wrapping_sub(capacity);
            *skipped = skipped.wrapping_add(resume.wrapping_sub(next));
            next = resume;
            // Commit the fast-forward HERE, not only on the success path below:
            // the gap is billed to `skipped` above, so the cursor has to move
            // with it, or an attempt-exhausted `None` leaves `tail` at its
            // pre-lap value and the next call re-derives and re-bills the same
            // gap. That give-up is the common path, not a corner — capacity is a
            // power of two and `resume = h - capacity`, so
            // `resume & mask == h & mask`: the fast-forward lands on exactly the
            // slot the producer overwrites next, and the version check below
            // loses to a saturated producer far more often than it wins.
            // Nothing extra is dropped (`resume` is the oldest position still in
            // the ring), `tail` is consumer-owned so the store is uncontended,
            // and the success-path store below supersedes it.
            tail.store(next, Ordering::Release);
        }
        let idx = (next & mask) as usize;
        let expected = next << 1;
        // SAFETY: idx <= mask (versions array bound, caller contract).
        let v = &*versions.add(idx);
        if v.load(Ordering::Acquire) != expected {
            // Mid-write of a newer position, or already lapped past `next`.
            continue;
        }
        let val = read_slot(idx);
        // Acquire fence (Boehm seqlock): pairs with the producer's Release fence.
        // If `val` came from an in-progress overwrite, the re-check below is forced
        // to observe the producer's newer (odd/lapped) stamp and reject the copy.
        fence(Ordering::Acquire);
        if v.load(Ordering::Relaxed) != expected {
            // The producer overwrote this slot while we copied it — torn read.
            discard(val);
            continue;
        }
        tail.store(next.wrapping_add(1), Ordering::Release);
        return Some(val);
    }
    None
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::cell::Cell;

    /// A capacity-`CAP` ring on the heap: the same version array, `head` and
    /// `tail` the SHM channel hands to `seqlock_publish`/`seqlock_consume`,
    /// without the SHM plumbing. Single-threaded, so the data slots are `Cell`s.
    struct Ring {
        versions: Vec<AtomicU64>,
        data: Vec<Cell<u64>>,
        head: AtomicU64,
        tail: AtomicU64,
        capacity: u64,
    }

    impl Ring {
        fn new(capacity: u64) -> Self {
            assert!(capacity.is_power_of_two());
            Self {
                versions: (0..capacity).map(|_| AtomicU64::new(0)).collect(),
                data: (0..capacity).map(|_| Cell::new(0)).collect(),
                head: AtomicU64::new(0),
                tail: AtomicU64::new(0),
                capacity,
            }
        }

        /// Publish the next position, storing the position itself as the payload
        /// so a delivered value identifies the position it came from.
        fn publish(&self) {
            let pos = self.head.load(Ordering::Relaxed);
            // SAFETY: `versions` has `capacity == mask + 1` entries and this test
            // is the only producer.
            unsafe {
                seqlock_publish(
                    self.versions.as_ptr(),
                    self.capacity - 1,
                    &self.head,
                    pos,
                    |idx| self.data[idx].set(pos),
                );
            }
        }

        fn consume(&self, skipped: &mut u64) -> Option<u64> {
            // SAFETY: same array bounds, and this test is the only consumer.
            unsafe {
                seqlock_consume(
                    self.versions.as_ptr(),
                    self.capacity - 1,
                    self.capacity,
                    &self.head,
                    &self.tail,
                    |idx| self.data[idx].get(),
                    drop,
                    skipped,
                )
            }
        }
    }

    /// A lapped consumer that gives up (every attempt lost the version check)
    /// must not bill the caller for the same lap again on its next call.
    ///
    /// The give-up is not a corner case: `resume = h - capacity` and capacity is
    /// a power of two, so `resume & mask == h & mask` — the fast-forward lands on
    /// exactly the slot the producer is about to overwrite. Against a saturated
    /// producer that read loses far more often than it wins, and each loss used
    /// to re-bank the whole gap, so the reported loss ran orders of magnitude
    /// above the number of messages ever sent.
    #[test]
    fn a_lapped_consumer_that_gives_up_does_not_re_count_the_gap() {
        const CAP: u64 = 16;
        let ring = Ring::new(CAP);

        // 20 sent into 16 slots: positions 0-3 are gone, the consumer is at 0.
        for _ in 0..20 {
            ring.publish();
        }
        // The producer is now mid-write of position 20 — the state the
        // fast-forward lands in, since it targets slot `head & mask`. Stamped
        // odd, data not written yet, `head` not advanced yet (`seqlock_publish`).
        ring.versions[(20 & (CAP - 1)) as usize].store((20 << 1) | 1, Ordering::Release);

        let mut missed = 0u64;
        assert_eq!(
            ring.consume(&mut missed),
            None,
            "the slot the producer is writing must not be delivered"
        );
        assert_eq!(missed, 4, "20 into a 16-slot ring skips 4 positions (0-3)");
        assert_eq!(ring.consume(&mut missed), None, "still mid-write");
        assert_eq!(
            missed, 4,
            "the second give-up re-counted a lap that was already banked: the \
             fast-forward must commit `tail`, not leave it for the success path"
        );

        // The producer finishes position 20; the consumer drains what is left.
        ring.data[(20 & (CAP - 1)) as usize].set(20);
        ring.versions[(20 & (CAP - 1)) as usize].store(20 << 1, Ordering::Release);
        ring.head.store(21, Ordering::Release);

        let mut got = Vec::new();
        while let Some(v) = ring.consume(&mut missed) {
            got.push(v);
        }
        assert_eq!(
            got.len() as u64 + missed,
            21,
            "received + missed must account for every send and nothing more \
             (got {got:?}, missed {missed})"
        );
    }
}
