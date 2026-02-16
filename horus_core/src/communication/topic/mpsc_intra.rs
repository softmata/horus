//! MpscIntra backend — same-process MP-1C (~26ns target).
//!
//! Multiple producers CAS on head to claim write slots.
//! Single consumer reads from tail with a simple store.
//! Per-slot sequence numbers ensure the consumer doesn't read a slot
//! before the producer has finished writing.

use std::cell::UnsafeCell;
use std::mem::MaybeUninit;
use std::sync::atomic::{AtomicU64, Ordering};

use super::primitives::{CachePadded, MpSlot};

/// Heap-backed MPSC ring buffer for N producers, 1 consumer.
pub(crate) struct MpscRing<T> {
    /// Shared head — producers CAS to claim write slots
    head: CachePadded<AtomicU64>,
    /// Consumer-owned tail (separate cache line)
    tail: CachePadded<AtomicU64>,
    /// Capacity mask
    mask: u64,
    /// Capacity
    capacity: u64,
    /// Slots with per-slot sequence tracking
    slots: Box<[MpSlot<T>]>,
}

unsafe impl<T: Send> Send for MpscRing<T> {}
unsafe impl<T: Send + Sync> Sync for MpscRing<T> {}

impl<T> MpscRing<T> {
    pub fn new(capacity: u32) -> Self {
        let cap = capacity.next_power_of_two() as usize;
        let mut slots = Vec::with_capacity(cap);
        for i in 0..cap {
            slots.push(MpSlot {
                // Initialize sequence to slot index so the first round of writes works:
                // slot[i].sequence = i means "slot i is available for sequence i"
                sequence: AtomicU64::new(i as u64),
                data: UnsafeCell::new(MaybeUninit::uninit()),
            });
        }
        Self {
            head: CachePadded(AtomicU64::new(0)),
            tail: CachePadded(AtomicU64::new(0)),
            mask: (cap - 1) as u64,
            capacity: cap as u64,
            slots: slots.into_boxed_slice(),
        }
    }

    /// Check how many messages are pending in the ring.
    #[inline]
    pub fn pending_count(&self) -> u64 {
        let head = self.head.0.load(Ordering::Acquire);
        let tail = self.tail.0.load(Ordering::Acquire);
        head.wrapping_sub(tail)
    }

    /// Try to send (multiple producers — CAS on head to claim slot). Delegates to shared `mp_try_send!` macro.
    #[inline(always)]
    pub fn try_send(&self, msg: T) -> Result<(), T> {
        mp_try_send!(self, msg)
    }

    /// Try to receive (single consumer only).
    #[inline(always)]
    pub fn try_recv(&self) -> Option<T> {
        let tail = self.tail.0.load(Ordering::Relaxed);
        let index = (tail & self.mask) as usize;
        // SAFETY: index is within bounds
        let slot = unsafe { self.slots.get_unchecked(index) };
        let seq = slot.sequence.load(Ordering::Acquire);

        // Data is ready when slot sequence equals tail + 1
        if seq == tail.wrapping_add(1) {
            // SAFETY: producer finished writing (sequence was set after write)
            let msg = unsafe { (*slot.data.get()).assume_init_read() };
            // Mark slot as available for reuse: set sequence to tail + capacity
            slot.sequence
                .store(tail.wrapping_add(self.capacity), Ordering::Release);
            self.tail.0.store(tail.wrapping_add(1), Ordering::Release);
            Some(msg)
        } else {
            None
        }
    }

    /// Read the most recent message without advancing the consumer.
    ///
    /// Returns a clone of the latest value. Returns `None` if the ring is empty,
    /// nothing was ever published, or the latest slot's write isn't yet complete.
    ///
    /// Uses `Clone` instead of moving to avoid double-free: the slot remains
    /// valid for `try_recv` to consume later.
    pub fn read_latest(&self) -> Option<T>
    where
        T: Clone,
    {
        let head = self.head.0.load(Ordering::Acquire);
        let tail = self.tail.0.load(Ordering::Acquire);
        if tail >= head {
            return None;
        }
        let prev = head.wrapping_sub(1);
        let index = (prev & self.mask) as usize;
        // SAFETY: index within bounds
        let slot = unsafe { self.slots.get_unchecked(index) };
        let seq = slot.sequence.load(Ordering::Acquire);
        // Slot is readable when sequence == prev + 1 (write completed)
        if seq == prev.wrapping_add(1) {
            // SAFETY: producer finished writing (sequence confirms).
            // We clone instead of moving to preserve the slot for try_recv.
            let msg = unsafe { (*slot.data.get()).assume_init_ref().clone() };
            Some(msg)
        } else {
            None
        }
    }
}

impl<T> Drop for MpscRing<T> {
    fn drop(&mut self) {
        let head = *self.head.0.get_mut();
        let tail = *self.tail.0.get_mut();
        // Drop all initialized but unconsumed messages in [tail, head).
        // Check sequence numbers to handle the edge case where a producer
        // panicked between CAS-claiming a slot and completing the write.
        for i in tail..head {
            let index = (i & self.mask) as usize;
            let slot = &mut self.slots[index];
            let seq = *slot.sequence.get_mut();
            // Slot is fully written when seq == i + 1
            if seq == i.wrapping_add(1) {
                // SAFETY: we have &mut self (exclusive access) and the
                // sequence number confirms the write completed.
                unsafe {
                    slot.data.get_mut().assume_init_drop();
                }
            }
        }
    }
}
