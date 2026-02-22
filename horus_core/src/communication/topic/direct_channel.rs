//! DirectChannel backend — same-thread 1P1C (~3ns target).
//!
//! Uses heap memory with zero cross-core contention for data transfer.
//! Self-contained head/tail counters live on the heap alongside the ring buffer,
//! so neither send nor recv ever touches the mmap'd SHM header on the hot path.
//!
//! This is the fastest possible IPC path — just a ptr::write + ptr::read
//! through heap memory (L1 cache hit guaranteed for same-thread access).

use std::cell::UnsafeCell;
use std::mem::MaybeUninit;
use std::sync::atomic::{AtomicU64, Ordering};

/// Heap-backed ring buffer for same-thread 1P1C communication.
///
/// Self-contained: head, tail, and data all live on the heap.
/// No SHM header updates on the hot path. On x86, Relaxed loads/stores
/// are plain MOV instructions — zero overhead beyond the cache line hit.
pub(crate) struct DirectSlot<T> {
    /// Producer-owned head counter (heap)
    pub(crate) head: AtomicU64,
    /// Consumer-owned tail counter (heap)
    pub(crate) tail: AtomicU64,
    /// Ring buffer slots
    pub(crate) buffer: Box<[UnsafeCell<MaybeUninit<T>>]>,
    /// Capacity mask for fast modulo (capacity - 1)
    pub(crate) mask: u64,
    /// Capacity for full check
    pub(crate) capacity: u64,
}

// SAFETY: DirectSlot is only used from a single thread (same-thread guarantee).
// The Send bound is needed because Topic is Send, but DirectSlot will
// only be accessed from the owning thread.
unsafe impl<T: Send> Send for DirectSlot<T> {}
unsafe impl<T: Send + Sync> Sync for DirectSlot<T> {}

impl<T> DirectSlot<T> {
    /// Create a new direct channel with the given capacity (must be power of 2).
    pub(crate) fn new(capacity: u32) -> Self {
        let cap = capacity.next_power_of_two() as usize;
        let mut buffer = Vec::with_capacity(cap);
        for _ in 0..cap {
            buffer.push(UnsafeCell::new(MaybeUninit::uninit()));
        }
        Self {
            head: AtomicU64::new(0),
            tail: AtomicU64::new(0),
            buffer: buffer.into_boxed_slice(),
            mask: (cap - 1) as u64,
            capacity: cap as u64,
        }
    }

    /// Try to send a message. Returns Err(msg) if the buffer is full.
    ///
    /// Same-thread only — no atomics needed for correctness, but we use
    /// Relaxed ordering (plain MOV on x86) to keep the AtomicU64 API.
    #[inline(always)]
    pub(crate) fn try_send(&self, msg: T) -> Result<(), T> {
        let head = self.head.load(Ordering::Relaxed);
        let tail = self.tail.load(Ordering::Relaxed);
        if head.wrapping_sub(tail) >= self.capacity {
            return Err(msg);
        }
        let index = (head & self.mask) as usize;
        // SAFETY: single-thread guarantee; index within bounds
        unsafe {
            let slot = self.buffer.get_unchecked(index);
            slot.get().write(MaybeUninit::new(msg));
        }
        self.head.store(head.wrapping_add(1), Ordering::Relaxed);
        Ok(())
    }

    /// Check how many messages are pending.
    #[inline]
    pub(crate) fn pending_count(&self) -> u64 {
        let head = self.head.load(Ordering::Relaxed);
        let tail = self.tail.load(Ordering::Relaxed);
        head.wrapping_sub(tail)
    }
}

impl<T> Drop for DirectSlot<T> {
    fn drop(&mut self) {
        let head = *self.head.get_mut();
        let tail = *self.tail.get_mut();
        // Drop all initialized but unconsumed messages in [tail, head)
        for i in tail..head {
            let index = (i & self.mask) as usize;
            // SAFETY: we have &mut self (exclusive access). All slots in
            // [tail, head) were written by the producer and not yet consumed.
            unsafe {
                self.buffer[index].get_mut().assume_init_drop();
            }
        }
    }
}
