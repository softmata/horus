//! DirectChannel backend — same-thread 1P1C (~3ns target).
//!
//! Uses heap memory with zero atomics for data transfer. The producer writes
//! directly to a ring buffer slot and bumps a local counter. The consumer reads
//! with a simple array index. Synchronization is through the header's atomic
//! sequence_or_head and tail (which on x86 are just compiler barriers via
//! Release/Acquire).
//!
//! This is the fastest possible IPC path — just a ptr::write + ptr::read
//! through heap memory (L1 cache hit guaranteed for same-thread access).

use std::cell::UnsafeCell;
use std::mem::MaybeUninit;

/// Heap-backed ring buffer for same-thread 1P1C communication.
///
/// No atomics needed for the data plane because producer and consumer are
/// guaranteed to be on the same thread. Ordering is enforced by the header's
/// atomic sequence_or_head / tail fields (which the dispatch code already uses).
pub(crate) struct DirectSlot<T> {
    /// Ring buffer slots
    buffer: Box<[UnsafeCell<MaybeUninit<T>>]>,
    /// Capacity mask for fast modulo (capacity - 1)
    mask: u64,
}

// SAFETY: DirectSlot is only used from a single thread (same-thread guarantee).
// The Send bound is needed because AdaptiveTopic is Send, but DirectSlot will
// only be accessed from the owning thread.
unsafe impl<T: Send> Send for DirectSlot<T> {}
unsafe impl<T: Send + Sync> Sync for DirectSlot<T> {}

impl<T> DirectSlot<T> {
    /// Create a new direct channel with the given capacity (must be power of 2).
    pub fn new(capacity: u32) -> Self {
        let cap = capacity.next_power_of_two() as usize;
        let mut buffer = Vec::with_capacity(cap);
        for _ in 0..cap {
            buffer.push(UnsafeCell::new(MaybeUninit::uninit()));
        }
        Self {
            buffer: buffer.into_boxed_slice(),
            mask: (cap - 1) as u64,
        }
    }

    /// Write a value to the given sequence position.
    ///
    /// # Safety
    /// - `seq` must be a valid sequence number (caller ensures no overflow)
    /// - Only one writer at a time (same-thread guarantee)
    #[inline(always)]
    pub unsafe fn write(&self, seq: u64, value: T) {
        let index = (seq & self.mask) as usize;
        let slot = &*self.buffer.get_unchecked(index);
        slot.get().write(MaybeUninit::new(value));
    }

    /// Read a value from the given sequence position.
    ///
    /// # Safety
    /// - `seq` must point to a slot that was previously written
    /// - Only one reader at a time (same-thread guarantee)
    #[inline(always)]
    pub unsafe fn read(&self, seq: u64) -> T {
        let index = (seq & self.mask) as usize;
        let slot = &*self.buffer.get_unchecked(index);
        (*slot.get()).assume_init_read()
    }

    /// Get the capacity mask.
    #[inline(always)]
    pub fn mask(&self) -> u64 {
        self.mask
    }
}
