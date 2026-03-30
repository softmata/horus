//! Shared primitives for ring buffer backends.
//!
//! Contains `CachePadded<T>` for false-sharing avoidance, `MpSlot<T>` for
//! multi-producer/consumer sequence-coordinated slots, and the `sp_try_send!`
//! and `mp_try_send!` macros shared across ring buffer implementations.

use std::cell::UnsafeCell;
use std::mem::MaybeUninit;
use std::sync::atomic::AtomicU64;

/// Cache-line aligned wrapper to prevent false sharing between producer and consumer.
#[repr(C, align(64))]
pub(crate) struct CachePadded<T>(pub T);

// === Shared allocation helpers ===

/// Allocate an uninitialized buffer for single-producer ring buffers (SPSC, SPMC).
pub(crate) fn alloc_uninit_buffer<T>(cap: usize) -> Box<[UnsafeCell<MaybeUninit<T>>]> {
    let mut buffer = Vec::with_capacity(cap);
    for _ in 0..cap {
        buffer.push(UnsafeCell::new(MaybeUninit::uninit()));
    }
    buffer.into_boxed_slice()
}

/// Allocate sequence-tracked slots for multi-producer ring buffers (MPSC, MPMC).
///
/// Each slot's sequence is initialized to its index, meaning "slot i is
/// available for sequence i" on the first round of writes.
pub(crate) fn alloc_mp_slots<T>(cap: usize) -> Box<[MpSlot<T>]> {
    let mut slots = Vec::with_capacity(cap);
    for i in 0..cap {
        slots.push(MpSlot {
            sequence: AtomicU64::new(i as u64),
            data: UnsafeCell::new(MaybeUninit::uninit()),
        });
    }
    slots.into_boxed_slice()
}

// === Shared ring buffer macros ===

/// Pending count implementation shared by all ring buffer variants.
///
/// Expects `$self` to have fields: `head` (CachePadded<AtomicU64>),
/// `tail` (CachePadded<AtomicU64>).
macro_rules! ring_pending_count {
    ($self:expr) => {{
        let head = $self.head.0.load(Ordering::Acquire);
        let tail = $self.tail.0.load(Ordering::Acquire);
        head.wrapping_sub(tail)
    }};
}

/// Drop implementation for single-producer ring buffers (SPSC, SPMC).
///
/// Expects `$self` to have fields: `head`, `tail`, `mask`, `buffer`
/// (Box<[UnsafeCell<MaybeUninit<T>>]>).
macro_rules! sp_drop_ring {
    ($self:expr) => {{
        let head = *$self.head.0.get_mut();
        let tail = *$self.tail.0.get_mut();
        for i in tail..head {
            let index = (i & $self.mask) as usize;
            // SAFETY: we have &mut self (exclusive access). All slots in
            // [tail, head) were written by the producer and not yet consumed.
            unsafe {
                $self.buffer[index].get_mut().assume_init_drop();
            }
        }
    }};
}

/// Drop implementation for multi-producer ring buffers (MPSC, MPMC).
///
/// Checks sequence numbers to handle the edge case where a producer
/// panicked between claiming a slot and completing the write.
///
/// Expects `$self` to have fields: `head`, `tail`, `mask`, `slots`
/// (Box<[MpSlot<T>]>).
macro_rules! mp_drop_ring {
    ($self:expr) => {{
        let head = *$self.head.0.get_mut();
        let tail = *$self.tail.0.get_mut();
        for i in tail..head {
            let index = (i & $self.mask) as usize;
            let slot = &mut $self.slots[index];
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
    }};
}

/// Single-producer try_send implementation shared by SPSC and SPMC rings.
///
/// Expects `$self` to have fields: `head`, `tail`, `cached_send_tail`,
/// `mask`, `capacity`, `buffer` (Box<[UnsafeCell<MaybeUninit<T>>]>).
///
/// Uses lazy tail caching: only fetches the consumer's tail atomic when
/// the cached check says "full". This avoids a cross-core cache line
/// bounce on every send (~25-50ns savings per call).
macro_rules! sp_try_send {
    ($self:expr, $msg:expr) => {{
        let head = $self.head.0.load(Ordering::Relaxed);
        // Lazy capacity check: use cached tail first, refresh only when needed
        let mut tail = $self.cached_send_tail.0.get();
        if head.wrapping_sub(tail) >= $self.capacity {
            tail = $self.tail.0.load(Ordering::Acquire);
            $self.cached_send_tail.0.set(tail);
            if head.wrapping_sub(tail) >= $self.capacity {
                return Err($msg);
            }
        }
        let index = (head & $self.mask) as usize;
        // SAFETY: single producer guarantee; index within bounds
        unsafe {
            let slot = &*$self.buffer.get_unchecked(index);
            slot.get().write(std::mem::MaybeUninit::new($msg));
        }
        $self.head.0.store(head.wrapping_add(1), Ordering::Release);
        Ok(())
    }};
}

/// Per-slot metadata for multi-producer ring buffers (MPSC and MPMC).
///
/// The sequence number coordinates producer writes and consumer reads:
/// - Initialized to slot index `i` (slot available for sequence `i`)
/// - Producer sets `sequence = claimed_seq + 1` after writing data
/// - Consumer sets `sequence = consumed_seq + capacity` after reading
pub(crate) struct MpSlot<T> {
    pub sequence: AtomicU64,
    pub data: UnsafeCell<MaybeUninit<T>>,
}

/// Multi-producer try_send implementation shared by MPSC and MPMC rings.
///
/// Expects `$self` to have fields: `head`, `mask`, `capacity`, `slots` (Box<[MpSlot<T>]>).
///
/// Two-phase design eliminates CAS contention:
/// 1. Pre-check: Load head + check slot sequence. If ring is full, return Err
///    immediately without incrementing head (no commit).
/// 2. Claim: `fetch_add(1)` on head — always succeeds in one atomic op, each
///    producer gets a unique slot. Unlike CAS, no retry loop needed.
/// 3. Brief spin: If another producer raced between pre-check and fetch_add,
///    spin-wait for consumer to free the claimed slot (bounded, near-instant).
macro_rules! mp_try_send {
    ($self:expr, $msg:expr) => {{
        // Phase 1: Non-binding capacity check. If the ring appears full,
        // return Err without incrementing head (no fetch_add = no commit).
        let current = $self.head.0.load(Ordering::Relaxed);
        let index = (current & $self.mask) as usize;
        // SAFETY: index is within bounds (current & mask < capacity)
        let slot = unsafe { $self.slots.get_unchecked(index) };
        let seq = slot.sequence.load(Ordering::Acquire);

        if seq != current {
            // Slot not available: either full (seq wrapped far behind)
            // or another producer is mid-write (seq == current-1+capacity).
            return Err($msg);
        }

        // Phase 2: Slot looks available — claim a unique position via fetch_add.
        // Unlike CAS, fetch_add never fails/retries, eliminating contention overhead.
        let claimed = $self.head.0.fetch_add(1, Ordering::Relaxed);
        let c_index = (claimed & $self.mask) as usize;
        // SAFETY: c_index is within bounds (claimed & mask < capacity)
        let c_slot = unsafe { $self.slots.get_unchecked(c_index) };

        // Phase 3: If we got a different slot than pre-checked (race with another
        // producer between load and fetch_add), brief spin until consumer frees it.
        // With a 256-slot ring and consumer at ~20ns/msg, wait is near-instant.
        if claimed != current {
            while c_slot.sequence.load(Ordering::Acquire) != claimed {
                std::hint::spin_loop();
            }
        }

        // SAFETY: we claimed this slot via fetch_add; no other producer writes here
        unsafe {
            c_slot.data.get().write(std::mem::MaybeUninit::new($msg));
        }
        c_slot
            .sequence
            .store(claimed.wrapping_add(1), Ordering::Release);
        return Ok(());
    }};
}
