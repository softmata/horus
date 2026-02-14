//! SpmcIntra backend — same-process 1P-MC (~24ns target).
//!
//! Single producer, multiple consumers. Producer writes to head with a simple
//! store. Consumers compete for tail using CAS.

use std::cell::UnsafeCell;
use std::mem::MaybeUninit;
use std::sync::atomic::{AtomicU64, Ordering};

#[repr(C, align(64))]
struct CachePadded<T>(T);

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
        let mut buffer = Vec::with_capacity(cap);
        for _ in 0..cap {
            buffer.push(UnsafeCell::new(MaybeUninit::uninit()));
        }
        Self {
            head: CachePadded(AtomicU64::new(0)),
            tail: CachePadded(AtomicU64::new(0)),
            cached_send_tail: CachePadded(std::cell::Cell::new(0)),
            mask: (cap - 1) as u64,
            capacity: cap as u64,
            buffer: buffer.into_boxed_slice(),
        }
    }

    /// Try to send (single producer only).
    ///
    /// Uses lazy tail caching: only fetches the consumers' tail atomic when
    /// the cached check says "full". This avoids a cross-core cache line
    /// bounce on every send (~25-50ns savings per call).
    #[inline(always)]
    pub fn try_send(&self, msg: T) -> Result<(), T> {
        let head = self.head.0.load(Ordering::Relaxed);
        // Lazy capacity check: use cached tail first, refresh only when needed
        let mut tail = self.cached_send_tail.0.get();
        if head.wrapping_sub(tail) >= self.capacity {
            tail = self.tail.0.load(Ordering::Acquire);
            self.cached_send_tail.0.set(tail);
            if head.wrapping_sub(tail) >= self.capacity {
                return Err(msg);
            }
        }
        let index = (head & self.mask) as usize;
        // SAFETY: single producer guarantee; index within bounds
        unsafe {
            let slot = &*self.buffer.get_unchecked(index);
            slot.get().write(MaybeUninit::new(msg));
        }
        self.head.0.store(head.wrapping_add(1), Ordering::Release);
        Ok(())
    }

    /// Check how many messages are pending in the ring.
    #[inline]
    pub fn pending_count(&self) -> u64 {
        let head = self.head.0.load(Ordering::Acquire);
        let tail = self.tail.0.load(Ordering::Acquire);
        head.wrapping_sub(tail)
    }

    /// Try to receive (multiple consumers — CAS on tail).
    #[inline(always)]
    pub fn try_recv(&self) -> Option<T> {
        loop {
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
                    let slot = &*self.buffer.get_unchecked(index);
                    (*slot.get()).assume_init_read()
                };
                return Some(msg);
            }
            std::hint::spin_loop();
        }
    }
}
