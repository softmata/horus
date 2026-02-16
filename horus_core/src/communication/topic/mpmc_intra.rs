//! MpmcIntra backend — same-process MPMC (~36ns target).
//!
//! Multiple producers CAS on head, multiple consumers CAS on tail.
//! Uses per-slot sequence numbers (Lamport-style) to coordinate:
//! - Producer: CAS head to claim write slot, write data, set slot.sequence = head + 1
//! - Consumer: CAS tail to claim read slot, read data, set slot.sequence = tail + capacity

use std::cell::UnsafeCell;
use std::mem::MaybeUninit;
use std::sync::atomic::{AtomicU64, Ordering};

use super::primitives::{CachePadded, MpSlot};

/// Heap-backed MPMC ring buffer for N producers, N consumers.
pub(crate) struct MpmcRing<T> {
    head: CachePadded<AtomicU64>,
    tail: CachePadded<AtomicU64>,
    mask: u64,
    capacity: u64,
    slots: Box<[MpSlot<T>]>,
}

unsafe impl<T: Send> Send for MpmcRing<T> {}
unsafe impl<T: Send + Sync> Sync for MpmcRing<T> {}

impl<T> MpmcRing<T> {
    pub fn new(capacity: u32) -> Self {
        let cap = capacity.next_power_of_two() as usize;
        let mut slots = Vec::with_capacity(cap);
        for i in 0..cap {
            slots.push(MpSlot {
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

    /// Try to send (multiple producers — CAS on head). Delegates to shared `mp_try_send!` macro.
    #[inline(always)]
    pub fn try_send(&self, msg: T) -> Result<(), T> {
        mp_try_send!(self, msg)
    }

    /// Try to receive (multiple consumers — CAS on tail).
    #[inline(always)]
    pub fn try_recv(&self) -> Option<T> {
        loop {
            let tail = self.tail.0.load(Ordering::Relaxed);
            let index = (tail & self.mask) as usize;
            let slot = unsafe { self.slots.get_unchecked(index) };
            let seq = slot.sequence.load(Ordering::Acquire);

            if seq == tail.wrapping_add(1) {
                if self
                    .tail
                    .0
                    .compare_exchange_weak(
                        tail,
                        tail.wrapping_add(1),
                        Ordering::Relaxed,
                        Ordering::Relaxed,
                    )
                    .is_ok()
                {
                    // SAFETY: producer finished writing (sequence was set after write)
                    let msg = unsafe { (*slot.data.get()).assume_init_read() };
                    slot.sequence
                        .store(tail.wrapping_add(self.capacity), Ordering::Release);
                    return Some(msg);
                }
                std::hint::spin_loop();
            } else if seq.wrapping_sub(tail.wrapping_add(1)) > self.capacity {
                // No data available
                return None;
            } else {
                // Data not ready yet — producer still writing
                std::hint::spin_loop();
            }
        }
    }

    /// Read the most recent message without advancing any consumer.
    ///
    /// Returns a copy of the latest value. Returns `None` if the ring is empty,
    /// nothing was ever published, or the latest slot's write isn't yet complete.
    ///
    /// # Safety invariant: requires `T: Copy`
    ///
    /// In MPMC, multiple consumers CAS on tail concurrently. Between the
    /// sequence check and reading the slot data, a consumer can consume the
    /// slot, drop the value (freeing heap memory), and a producer can rewrite
    /// it on the next ring pass. `T: Copy` guarantees no heap pointers — the
    /// bytes are always safe to read regardless of consumption state.
    pub fn read_latest(&self) -> Option<T>
    where
        T: Copy,
    {
        let head = self.head.0.load(Ordering::Acquire);
        let tail = self.tail.0.load(Ordering::Acquire);
        if tail >= head {
            return None;
        }
        let prev = head.wrapping_sub(1);
        let index = (prev & self.mask) as usize;
        let slot = unsafe { self.slots.get_unchecked(index) };
        let seq = slot.sequence.load(Ordering::Acquire);
        // Slot is readable when sequence == prev + 1 (write completed).
        // Even if a consumer has consumed this slot between our check and
        // the read, T: Copy means no heap pointers to dangle.
        if seq == prev.wrapping_add(1) {
            // SAFETY: T is Copy. The sequence check confirms the producer
            // finished writing. Even if a consumer consumes this slot
            // concurrently, the bytes remain valid (no Drop/dealloc).
            let msg = unsafe { (*slot.data.get()).assume_init_read() };
            Some(msg)
        } else {
            None
        }
    }
}

impl<T> Drop for MpmcRing<T> {
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
