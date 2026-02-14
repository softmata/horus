//! MpscIntra backend — same-process MP-1C (~26ns target).
//!
//! Multiple producers CAS on head to claim write slots.
//! Single consumer reads from tail with a simple store.
//! Per-slot sequence numbers ensure the consumer doesn't read a slot
//! before the producer has finished writing.

use std::cell::UnsafeCell;
use std::mem::MaybeUninit;
use std::sync::atomic::{AtomicU64, Ordering};

#[repr(C, align(64))]
struct CachePadded<T>(T);

/// Per-slot metadata for producer write completion tracking.
pub(crate) struct MpscSlot<T> {
    /// Sequence number — set to `claimed_seq + 1` after the producer finishes writing.
    /// Consumer waits until this equals `expected_seq` before reading.
    sequence: AtomicU64,
    /// Data slot
    data: UnsafeCell<MaybeUninit<T>>,
}

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
    slots: Box<[MpscSlot<T>]>,
}

unsafe impl<T: Send> Send for MpscRing<T> {}
unsafe impl<T: Send + Sync> Sync for MpscRing<T> {}

impl<T> MpscRing<T> {
    pub fn new(capacity: u32) -> Self {
        let cap = capacity.next_power_of_two() as usize;
        let mut slots = Vec::with_capacity(cap);
        for i in 0..cap {
            slots.push(MpscSlot {
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

    /// Try to send (multiple producers — CAS on head to claim slot).
    #[inline(always)]
    pub fn try_send(&self, msg: T) -> Result<(), T> {
        loop {
            let head = self.head.0.load(Ordering::Relaxed);
            let index = (head & self.mask) as usize;
            // SAFETY: index is within bounds (head & mask < capacity)
            let slot = unsafe { self.slots.get_unchecked(index) };
            let seq = slot.sequence.load(Ordering::Acquire);

            // Slot is available when its sequence equals head (wraps correctly)
            if seq == head {
                // Try to claim this slot
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
                    // Write data then publish by setting sequence to head + 1
                    // SAFETY: we claimed this slot via CAS; no other producer writes here
                    unsafe {
                        slot.data.get().write(MaybeUninit::new(msg));
                    }
                    slot.sequence
                        .store(head.wrapping_add(1), Ordering::Release);
                    return Ok(());
                }
                // CAS failed — another producer claimed it, retry
                std::hint::spin_loop();
            } else if seq.wrapping_sub(head) > self.capacity {
                // seq < head means buffer is full (wrapped around)
                return Err(msg);
            } else {
                // Slot not ready yet (another producer is writing), spin
                std::hint::spin_loop();
            }
        }
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
}
