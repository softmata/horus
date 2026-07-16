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
            // position (drop-oldest).
            next = h.wrapping_sub(capacity);
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
