//! SpscIntra backend — same-process 1P1C (~18ns target).
//!
//! Heap-backed ring buffer with cache-line separated head and tail atomics.
//! Producer owns the head cache line, consumer owns the tail cache line.
//! No CAS needed — single producer uses store, single consumer uses store.

use std::cell::UnsafeCell;
use std::mem::MaybeUninit;
use std::sync::atomic::{AtomicU64, Ordering};

use super::primitives::CachePadded;

/// Heap-backed SPSC ring buffer for cross-thread 1P1C communication.
///
/// The head (producer-owned) and tail (consumer-owned) are on separate cache
/// lines to eliminate false sharing — the dominant source of latency in
/// cross-thread ring buffers.
///
/// Lazy tail caching: the producer caches the consumer's tail locally and only
/// refreshes it from the atomic when the cache says "full". This avoids a
/// cross-core cache line bounce on every send (~25-50ns savings).
pub(crate) struct SpscRing<T> {
    /// Producer-owned sequence counter (separate cache line)
    head: CachePadded<AtomicU64>,
    /// Consumer-owned sequence counter (separate cache line)
    tail: CachePadded<AtomicU64>,
    /// Producer-side cached tail value (avoids cross-core tail.load on every send)
    cached_send_tail: CachePadded<std::cell::Cell<u64>>,
    /// Consumer-side cached head value (avoids cross-core head.load on every recv)
    cached_recv_head: CachePadded<std::cell::Cell<u64>>,
    /// Capacity mask for fast modulo
    mask: u64,
    /// Capacity for backpressure checks
    capacity: u64,
    /// Ring buffer slots
    buffer: Box<[UnsafeCell<MaybeUninit<T>>]>,
}

unsafe impl<T: Send> Send for SpscRing<T> {}
unsafe impl<T: Send + Sync> Sync for SpscRing<T> {}

impl<T> SpscRing<T> {
    /// Create a new SPSC ring with the given capacity (must be power of 2).
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
            cached_recv_head: CachePadded(std::cell::Cell::new(0)),
            mask: (cap - 1) as u64,
            capacity: cap as u64,
            buffer: buffer.into_boxed_slice(),
        }
    }

    /// Try to send a message. Returns Err(msg) if the buffer is full.
    /// Delegates to shared `sp_try_send!` macro (single-producer with lazy tail caching).
    #[inline(always)]
    pub fn try_send(&self, msg: T) -> Result<(), T> {
        sp_try_send!(self, msg)
    }

    /// Check how many messages are pending in the ring.
    #[inline]
    pub fn pending_count(&self) -> u64 {
        let head = self.head.0.load(Ordering::Acquire);
        let tail = self.tail.0.load(Ordering::Acquire);
        head.wrapping_sub(tail)
    }

    /// Try to receive a message. Returns None if the buffer is empty.
    ///
    /// Only one consumer should call this (SPSC guarantee).
    /// Uses lazy head loading: only fetches the producer's head atomic when
    /// the cached check says "empty".
    #[inline(always)]
    pub fn try_recv(&self) -> Option<T> {
        let tail = self.tail.0.load(Ordering::Relaxed);
        // Lazy: use cached head first, refresh only when "empty"
        let mut head = self.cached_recv_head.0.get();
        if tail >= head {
            head = self.head.0.load(Ordering::Acquire);
            self.cached_recv_head.0.set(head);
            if tail >= head {
                return None;
            }
        }
        let index = (tail & self.mask) as usize;
        // SAFETY: index is within bounds; data was written by producer and
        // made visible via Release/Acquire on head
        let msg = unsafe {
            let slot = &*self.buffer.get_unchecked(index);
            (*slot.get()).assume_init_read()
        };
        self.tail.0.store(tail.wrapping_add(1), Ordering::Release);
        Some(msg)
    }

    /// Read the most recent message without advancing the consumer.
    ///
    /// Returns a clone of the latest value. Returns `None` if the ring is empty
    /// (all messages consumed) or nothing was ever published.
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
        let index = ((head.wrapping_sub(1)) & self.mask) as usize;
        // SAFETY: slot is in [tail, head) — initialized and not yet consumed.
        // We clone instead of moving to preserve the slot for try_recv.
        let msg = unsafe {
            let slot = &*self.buffer.get_unchecked(index);
            (*slot.get()).assume_init_ref().clone()
        };
        Some(msg)
    }
}

impl<T> Drop for SpscRing<T> {
    fn drop(&mut self) {
        let head = *self.head.0.get_mut();
        let tail = *self.tail.0.get_mut();
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
