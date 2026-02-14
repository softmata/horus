//! MpmcIntra backend — same-process MPMC (~36ns target).
//!
//! Multiple producers CAS on head, multiple consumers CAS on tail.
//! Uses per-slot sequence numbers (Lamport-style) to coordinate:
//! - Producer: CAS head to claim write slot, write data, set slot.sequence = head + 1
//! - Consumer: CAS tail to claim read slot, read data, set slot.sequence = tail + capacity

use std::cell::UnsafeCell;
use std::mem::MaybeUninit;
use std::sync::atomic::{AtomicU64, Ordering};

#[repr(C, align(64))]
struct CachePadded<T>(T);

/// Per-slot metadata for MPMC coordination.
pub(crate) struct MpmcSlot<T> {
    sequence: AtomicU64,
    data: UnsafeCell<MaybeUninit<T>>,
}

/// Heap-backed MPMC ring buffer for N producers, N consumers.
pub(crate) struct MpmcRing<T> {
    head: CachePadded<AtomicU64>,
    tail: CachePadded<AtomicU64>,
    mask: u64,
    capacity: u64,
    slots: Box<[MpmcSlot<T>]>,
}

unsafe impl<T: Send> Send for MpmcRing<T> {}
unsafe impl<T: Send + Sync> Sync for MpmcRing<T> {}

impl<T> MpmcRing<T> {
    pub fn new(capacity: u32) -> Self {
        let cap = capacity.next_power_of_two() as usize;
        let mut slots = Vec::with_capacity(cap);
        for i in 0..cap {
            slots.push(MpmcSlot {
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

    /// Try to send (multiple producers — CAS on head).
    #[inline(always)]
    pub fn try_send(&self, msg: T) -> Result<(), T> {
        loop {
            let head = self.head.0.load(Ordering::Relaxed);
            let index = (head & self.mask) as usize;
            let slot = unsafe { self.slots.get_unchecked(index) };
            let seq = slot.sequence.load(Ordering::Acquire);

            if seq == head {
                if self
                    .head
                    .0
                    .compare_exchange_weak(
                        head,
                        head.wrapping_add(1),
                        Ordering::Relaxed,
                        Ordering::Relaxed,
                    )
                    .is_ok()
                {
                    // SAFETY: we claimed this slot via CAS
                    unsafe {
                        slot.data.get().write(MaybeUninit::new(msg));
                    }
                    slot.sequence
                        .store(head.wrapping_add(1), Ordering::Release);
                    return Ok(());
                }
                std::hint::spin_loop();
            } else if seq.wrapping_sub(head) > self.capacity {
                return Err(msg);
            } else {
                std::hint::spin_loop();
            }
        }
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
}
