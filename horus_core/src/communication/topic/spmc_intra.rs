//! SpmcIntra backend — same-process 1P-MC (~24ns target).
//!
//! Single producer, multiple consumers. Producer writes to head with a simple
//! store. Consumers compete for tail using CAS.

use std::cell::UnsafeCell;
use std::mem::MaybeUninit;
use std::sync::atomic::{AtomicU64, Ordering};

use super::primitives::{alloc_uninit_buffer, CachePadded};

/// Heap-backed SPMC ring buffer for 1 producer, N consumers.
///
/// Producer uses local head + Release store (no contention).
/// Consumers CAS on tail to claim the next slot.
/// Lazy tail caching on producer side avoids cross-core cache bounce.
pub(crate) struct SpmcRing<T> {
    /// Producer-owned head (separate cache line)
    head: CachePadded<AtomicU64>,
    /// Shared tail — consumers CAS to claim slots
    tail: CachePadded<AtomicU64>,
    /// Producer-side cached tail (avoids cross-core load on every send)
    cached_send_tail: CachePadded<std::cell::Cell<u64>>,
    /// Capacity mask
    mask: u64,
    /// Capacity
    capacity: u64,
    /// Ring buffer slots
    buffer: Box<[UnsafeCell<MaybeUninit<T>>]>,
}

unsafe impl<T: Send> Send for SpmcRing<T> {}
unsafe impl<T: Send + Sync> Sync for SpmcRing<T> {}

impl<T> SpmcRing<T> {
    pub fn new(capacity: u32) -> Self {
        let cap = capacity.next_power_of_two() as usize;
        Self {
            head: CachePadded(AtomicU64::new(0)),
            tail: CachePadded(AtomicU64::new(0)),
            cached_send_tail: CachePadded(std::cell::Cell::new(0)),
            mask: (cap - 1) as u64,
            capacity: cap as u64,
            buffer: alloc_uninit_buffer(cap),
        }
    }

    /// Try to send (single producer only). Delegates to shared `sp_try_send!` macro.
    #[inline(always)]
    pub fn try_send(&self, msg: T) -> Result<(), T> {
        sp_try_send!(self, msg)
    }

    /// Check how many messages are pending in the ring.
    #[inline]
    pub fn pending_count(&self) -> u64 {
        ring_pending_count!(self)
    }

    /// Try to receive (multiple consumers — CAS on tail).
    ///
    /// Bounded retry: up to 8 CAS attempts to prevent unbounded spinning
    /// under high consumer contention. Returns None if all attempts fail;
    /// the caller retries on the next poll.
    #[inline(always)]
    pub fn try_recv(&self) -> Option<T> {
        for _attempt in 0..8 {
            let tail = self.tail.0.load(Ordering::Acquire);
            let head = self.head.0.load(Ordering::Acquire);
            if tail >= head {
                return None;
            }
            if self
                .tail
                .0
                .compare_exchange_weak(
                    tail,
                    tail.wrapping_add(1),
                    Ordering::AcqRel,
                    Ordering::Relaxed,
                )
                .is_ok()
            {
                let index = (tail & self.mask) as usize;
                // SAFETY: we successfully claimed this slot via CAS;
                // data was written by producer and visible via Release/Acquire on head
                let msg = unsafe {
                    let slot = self.buffer.get_unchecked(index);
                    (*slot.get()).assume_init_read()
                };
                return Some(msg);
            }
            std::hint::spin_loop();
        }
        None
    }

    /// Read the most recent message without advancing any consumer.
    ///
    /// Returns a copy of the latest value. Returns `None` if the ring is empty
    /// (all messages consumed) or nothing was ever published.
    ///
    /// # Safety invariant: requires `T: Copy`
    ///
    /// In SPMC, multiple consumers CAS on tail concurrently. Between loading
    /// `head` and reading the slot, a consumer can consume the slot at `head-1`
    /// and drop the value. For types with heap allocations (String, Vec), this
    /// would be use-after-free. `T: Copy` guarantees no heap pointers — the
    /// bytes in the slot are always safe to read even after logical consumption.
    pub fn read_latest(&self) -> Option<T>
    where
        T: Copy,
    {
        let head = self.head.0.load(Ordering::Acquire);
        let tail = self.tail.0.load(Ordering::Acquire);
        if tail >= head {
            return None;
        }
        let index = ((head.wrapping_sub(1)) & self.mask) as usize;
        // SAFETY: T is Copy (no heap pointers, no Drop). Even if a consumer
        // has logically consumed this slot via CAS on tail, the bytes remain
        // valid because Copy types have no destructor that frees memory.
        // The producer won't overwrite this slot until head wraps around
        // (requires capacity writes), making concurrent producer writes
        // extremely unlikely during a single ptr::read.
        let msg = unsafe {
            let slot = self.buffer.get_unchecked(index);
            (*slot.get()).assume_init_read()
        };
        Some(msg)
    }
}

impl<T> Drop for SpmcRing<T> {
    fn drop(&mut self) {
        sp_drop_ring!(self);
    }
}
