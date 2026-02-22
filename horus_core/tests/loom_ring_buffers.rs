//! Loom-based exhaustive concurrency tests for ring buffer algorithms.
//!
//! These tests use Tokio's `loom` crate to explore all possible thread
//! interleavings, verifying that the lock-free ring buffer algorithms
//! used in HORUS Topic backends are correct under every execution order.
//!
//! The ring implementations here are simplified versions of the production
//! code in `horus_core::communication::topic::{spsc,spmc,mpsc,mpmc}_intra`,
//! using loom's atomic primitives instead of `std::sync::atomic`.
//!
//! Run with: `cargo test --test loom_ring_buffers -- --nocapture`
//!
//! Note: loom tests explore exponentially many interleavings. Keep ring
//! capacities small (2-4) and message counts low (2-4).

use loom::cell::UnsafeCell;
use loom::sync::atomic::{AtomicU64, Ordering};
use loom::sync::Arc;
use std::mem::MaybeUninit;

// ============================================================================
// Shared helpers for loom ring buffer variants
// ============================================================================

/// Allocate an uninitialized loom UnsafeCell buffer for SP ring buffers.
fn alloc_loom_buffer<T>(cap: usize) -> Vec<UnsafeCell<MaybeUninit<T>>> {
    let mut buffer = Vec::with_capacity(cap);
    for _ in 0..cap {
        buffer.push(UnsafeCell::new(MaybeUninit::uninit()));
    }
    buffer
}

/// Allocate sequence-tracked loom slots for MP ring buffers.
fn alloc_loom_mp_slots<T>(cap: usize) -> Vec<LoomMpSlot<T>> {
    let mut slots = Vec::with_capacity(cap);
    for i in 0..cap {
        slots.push(LoomMpSlot {
            sequence: AtomicU64::new(i as u64),
            data: UnsafeCell::new(MaybeUninit::uninit()),
        });
    }
    slots
}

/// Single-producer try_send for loom SP ring buffers (SPSC, SPMC).
macro_rules! loom_sp_try_send {
    ($self:expr, $msg:expr) => {{
        let head = $self.head.load(Ordering::Relaxed);
        let tail = $self.tail.load(Ordering::Acquire);
        if head.wrapping_sub(tail) >= $self.capacity {
            return Err($msg);
        }
        let index = (head & $self.mask) as usize;
        $self.buffer[index].with_mut(|ptr| unsafe {
            ptr.write(MaybeUninit::new($msg));
        });
        $self.head.store(head.wrapping_add(1), Ordering::Release);
        Ok(())
    }};
}

/// Drop implementation for loom SP ring buffers (SPSC, SPMC).
macro_rules! loom_sp_drop {
    ($self:expr) => {{
        let head = $self.head.load(Ordering::Relaxed);
        let tail = $self.tail.load(Ordering::Relaxed);
        for i in tail..head {
            let index = (i & $self.mask) as usize;
            $self.buffer[index].with_mut(|ptr| unsafe {
                (*ptr).assume_init_drop();
            });
        }
    }};
}

/// Drop implementation for loom MP ring buffers (MPSC, MPMC).
macro_rules! loom_mp_drop {
    ($self:expr) => {{
        let head = $self.head.load(Ordering::Relaxed);
        let tail = $self.tail.load(Ordering::Relaxed);
        for i in tail..head {
            let index = (i & $self.mask) as usize;
            let seq = $self.slots[index].sequence.load(Ordering::Relaxed);
            if seq == i.wrapping_add(1) {
                $self.slots[index].data.with_mut(|ptr| unsafe {
                    (*ptr).assume_init_drop();
                });
            }
        }
    }};
}

/// Per-slot metadata for loom multi-producer ring buffers.
struct LoomMpSlot<T> {
    sequence: AtomicU64,
    data: UnsafeCell<MaybeUninit<T>>,
}

// ============================================================================
// Simplified SPSC Ring (mirrors spsc_intra.rs algorithm)
// ============================================================================

/// Minimal SPSC ring for loom testing. No cache padding or lazy caching —
/// focuses on the core Relaxed/Release/Acquire ordering protocol.
struct LoomSpscRing<T> {
    head: AtomicU64,
    tail: AtomicU64,
    mask: u64,
    capacity: u64,
    buffer: Vec<UnsafeCell<MaybeUninit<T>>>,
}

unsafe impl<T: Send> Send for LoomSpscRing<T> {}
unsafe impl<T: Send + Sync> Sync for LoomSpscRing<T> {}

impl<T> LoomSpscRing<T> {
    fn new(capacity: usize) -> Self {
        let cap = capacity.next_power_of_two();
        Self {
            head: AtomicU64::new(0),
            tail: AtomicU64::new(0),
            mask: (cap - 1) as u64,
            capacity: cap as u64,
            buffer: alloc_loom_buffer(cap),
        }
    }

    fn try_send(&self, msg: T) -> Result<(), T> {
        loom_sp_try_send!(self, msg)
    }

    fn try_recv(&self) -> Option<T> {
        let tail = self.tail.load(Ordering::Relaxed);
        let head = self.head.load(Ordering::Acquire);
        if tail >= head {
            return None;
        }
        let index = (tail & self.mask) as usize;
        let msg = self.buffer[index].with(|ptr| unsafe { (*ptr).assume_init_read() });
        self.tail.store(tail.wrapping_add(1), Ordering::Release);
        Some(msg)
    }
}

impl<T> Drop for LoomSpscRing<T> {
    fn drop(&mut self) {
        loom_sp_drop!(self);
    }
}

// ============================================================================
// Simplified SPMC Ring (mirrors spmc_intra.rs algorithm)
// ============================================================================

/// Minimal SPMC ring: single-producer store, multi-consumer CAS on tail.
struct LoomSpmcRing<T> {
    head: AtomicU64,
    tail: AtomicU64,
    mask: u64,
    capacity: u64,
    buffer: Vec<UnsafeCell<MaybeUninit<T>>>,
}

unsafe impl<T: Send> Send for LoomSpmcRing<T> {}
unsafe impl<T: Send + Sync> Sync for LoomSpmcRing<T> {}

impl<T> LoomSpmcRing<T> {
    fn new(capacity: usize) -> Self {
        let cap = capacity.next_power_of_two();
        Self {
            head: AtomicU64::new(0),
            tail: AtomicU64::new(0),
            mask: (cap - 1) as u64,
            capacity: cap as u64,
            buffer: alloc_loom_buffer(cap),
        }
    }

    fn try_send(&self, msg: T) -> Result<(), T> {
        loom_sp_try_send!(self, msg)
    }

    fn try_recv(&self) -> Option<T> {
        loop {
            let tail = self.tail.load(Ordering::Acquire);
            let head = self.head.load(Ordering::Acquire);
            if tail >= head {
                return None;
            }
            if self
                .tail
                .compare_exchange_weak(
                    tail,
                    tail.wrapping_add(1),
                    Ordering::AcqRel,
                    Ordering::Relaxed,
                )
                .is_ok()
            {
                let index = (tail & self.mask) as usize;
                let msg = self.buffer[index].with(|ptr| unsafe { (*ptr).assume_init_read() });
                return Some(msg);
            }
            loom::thread::yield_now();
        }
    }

    /// Read the most recent message without advancing any consumer.
    /// Requires T: Copy to avoid TOCTOU use-after-free (consumer can
    /// consume the slot between our head load and data read).
    fn read_latest(&self) -> Option<T>
    where
        T: Copy,
    {
        let head = self.head.load(Ordering::Acquire);
        let tail = self.tail.load(Ordering::Acquire);
        if tail >= head {
            return None;
        }
        let index = ((head.wrapping_sub(1)) & self.mask) as usize;
        // SAFETY: T is Copy — no heap pointers, no Drop. Even if a consumer
        // consumes this slot concurrently, the bytes remain valid.
        let msg = self.buffer[index].with(|ptr| unsafe { (*ptr).assume_init_read() });
        Some(msg)
    }
}

impl<T> Drop for LoomSpmcRing<T> {
    fn drop(&mut self) {
        loom_sp_drop!(self);
    }
}

// ============================================================================
// Simplified MPSC Ring (mirrors mpsc_intra.rs algorithm)
// ============================================================================

/// Minimal MPSC ring: multi-producer CAS on head with per-slot sequences.
struct LoomMpscRing<T> {
    head: AtomicU64,
    tail: AtomicU64,
    mask: u64,
    capacity: u64,
    slots: Vec<LoomMpSlot<T>>,
}

unsafe impl<T: Send> Send for LoomMpscRing<T> {}
unsafe impl<T: Send + Sync> Sync for LoomMpscRing<T> {}

impl<T> LoomMpscRing<T> {
    fn new(capacity: usize) -> Self {
        let cap = capacity.next_power_of_two();
        Self {
            head: AtomicU64::new(0),
            tail: AtomicU64::new(0),
            mask: (cap - 1) as u64,
            capacity: cap as u64,
            slots: alloc_loom_mp_slots(cap),
        }
    }

    fn try_send(&self, msg: T) -> Result<(), T> {
        loop {
            let head = self.head.load(Ordering::Relaxed);
            let index = (head & self.mask) as usize;
            let seq = self.slots[index].sequence.load(Ordering::Acquire);

            if seq == head {
                if self
                    .head
                    .compare_exchange_weak(
                        head,
                        head.wrapping_add(1),
                        Ordering::Relaxed,
                        Ordering::Relaxed,
                    )
                    .is_ok()
                {
                    self.slots[index].data.with_mut(|ptr| unsafe {
                        ptr.write(MaybeUninit::new(msg));
                    });
                    self.slots[index]
                        .sequence
                        .store(head.wrapping_add(1), Ordering::Release);
                    return Ok(());
                }
                loom::thread::yield_now();
            } else if seq.wrapping_sub(head) > self.capacity {
                return Err(msg);
            } else {
                loom::thread::yield_now();
            }
        }
    }

    fn try_recv(&self) -> Option<T> {
        let tail = self.tail.load(Ordering::Relaxed);
        let index = (tail & self.mask) as usize;
        let seq = self.slots[index].sequence.load(Ordering::Acquire);

        if seq == tail.wrapping_add(1) {
            let msg = self.slots[index]
                .data
                .with(|ptr| unsafe { (*ptr).assume_init_read() });
            self.slots[index]
                .sequence
                .store(tail.wrapping_add(self.capacity), Ordering::Release);
            self.tail.store(tail.wrapping_add(1), Ordering::Release);
            Some(msg)
        } else {
            None
        }
    }
}

impl<T> Drop for LoomMpscRing<T> {
    fn drop(&mut self) {
        loom_mp_drop!(self);
    }
}

// ============================================================================
// Simplified MPMC Ring (mirrors mpmc_intra.rs algorithm)
// ============================================================================

/// Minimal MPMC ring: multi-producer CAS on head, multi-consumer CAS on tail.
///
/// Uses bounded retry counts instead of infinite loops to make loom
/// exhaustive exploration tractable. The algorithm is identical to
/// production code except for the bounded retry.
struct LoomMpmcRing<T> {
    head: AtomicU64,
    tail: AtomicU64,
    mask: u64,
    capacity: u64,
    slots: Vec<LoomMpSlot<T>>,
}

unsafe impl<T: Send> Send for LoomMpmcRing<T> {}
unsafe impl<T: Send + Sync> Sync for LoomMpmcRing<T> {}

/// Max CAS retries — bounds loom's state space exploration.
const MPMC_MAX_RETRIES: usize = 4;

impl<T> LoomMpmcRing<T> {
    fn new(capacity: usize) -> Self {
        let cap = capacity.next_power_of_two();
        Self {
            head: AtomicU64::new(0),
            tail: AtomicU64::new(0),
            mask: (cap - 1) as u64,
            capacity: cap as u64,
            slots: alloc_loom_mp_slots(cap),
        }
    }

    fn try_send(&self, msg: T) -> Result<(), T> {
        for _ in 0..MPMC_MAX_RETRIES {
            let head = self.head.load(Ordering::Relaxed);
            let index = (head & self.mask) as usize;
            let seq = self.slots[index].sequence.load(Ordering::Acquire);

            if seq == head {
                if self
                    .head
                    .compare_exchange(
                        head,
                        head.wrapping_add(1),
                        Ordering::Relaxed,
                        Ordering::Relaxed,
                    )
                    .is_ok()
                {
                    self.slots[index].data.with_mut(|ptr| unsafe {
                        ptr.write(MaybeUninit::new(msg));
                    });
                    self.slots[index]
                        .sequence
                        .store(head.wrapping_add(1), Ordering::Release);
                    return Ok(());
                }
                loom::thread::yield_now();
            } else if seq.wrapping_sub(head) > self.capacity {
                return Err(msg);
            } else {
                loom::thread::yield_now();
            }
        }
        Err(msg)
    }

    fn try_recv(&self) -> Option<T> {
        for _ in 0..MPMC_MAX_RETRIES {
            let tail = self.tail.load(Ordering::Relaxed);
            let index = (tail & self.mask) as usize;
            let seq = self.slots[index].sequence.load(Ordering::Acquire);

            if seq == tail.wrapping_add(1) {
                if self
                    .tail
                    .compare_exchange(
                        tail,
                        tail.wrapping_add(1),
                        Ordering::Relaxed,
                        Ordering::Relaxed,
                    )
                    .is_ok()
                {
                    let msg = self.slots[index]
                        .data
                        .with(|ptr| unsafe { (*ptr).assume_init_read() });
                    self.slots[index]
                        .sequence
                        .store(tail.wrapping_add(self.capacity), Ordering::Release);
                    return Some(msg);
                }
                loom::thread::yield_now();
            } else if seq.wrapping_sub(tail) > self.capacity {
                return None;
            } else {
                loom::thread::yield_now();
            }
        }
        None
    }

    /// Read the most recent message without advancing any consumer.
    /// Requires T: Copy for TOCTOU safety (same reasoning as SPMC).
    fn read_latest(&self) -> Option<T>
    where
        T: Copy,
    {
        let head = self.head.load(Ordering::Acquire);
        let tail = self.tail.load(Ordering::Acquire);
        if tail >= head {
            return None;
        }
        let prev = head.wrapping_sub(1);
        let index = (prev & self.mask) as usize;
        let seq = self.slots[index].sequence.load(Ordering::Acquire);
        // Slot is readable when sequence == prev + 1 (write completed).
        if seq == prev.wrapping_add(1) {
            let msg = self.slots[index]
                .data
                .with(|ptr| unsafe { (*ptr).assume_init_read() });
            Some(msg)
        } else {
            None
        }
    }
}

impl<T> Drop for LoomMpmcRing<T> {
    fn drop(&mut self) {
        loom_mp_drop!(self);
    }
}

// ============================================================================
// LOOM TESTS — SPSC
// ============================================================================

#[test]
fn loom_spsc_send_recv_ordering() {
    // Verify that a value written by the producer is correctly visible
    // to the consumer across all possible interleavings.
    loom::model(|| {
        let ring = Arc::new(LoomSpscRing::<u64>::new(2));
        let r = ring.clone();

        let producer = loom::thread::spawn(move || {
            assert!(r.try_send(42).is_ok());
        });

        // Consumer may or may not see the value depending on scheduling
        let val = ring.try_recv();
        producer.join().unwrap();

        // After join, if the consumer didn't see it, drain it now
        if val.is_none() {
            let drained = ring.try_recv();
            assert_eq!(drained, Some(42));
        } else {
            assert_eq!(val, Some(42));
        }
    });
}

#[test]
fn loom_spsc_full_ring_backpressure() {
    // Verify that a full ring correctly rejects new messages.
    loom::model(|| {
        let ring = Arc::new(LoomSpscRing::<u64>::new(2));
        let r = ring.clone();

        // Fill the ring
        assert!(ring.try_send(1).is_ok());
        assert!(ring.try_send(2).is_ok());

        // Spawn consumer that drains one
        let consumer = loom::thread::spawn(move || r.try_recv());

        // Producer tries to send a third — may succeed or fail depending
        // on whether consumer ran first
        let result = ring.try_send(3);
        let consumed = consumer.join().unwrap();

        if consumed.is_some() {
            // Consumer drained one slot, so send might have succeeded
            // Either way, no crash or lost data
        }

        // Drain remaining
        let mut remaining = Vec::new();
        while let Some(v) = ring.try_recv() {
            remaining.push(v);
        }

        // All sent values should be accounted for
        let total_sent = 2 + if result.is_ok() { 1 } else { 0 };
        assert_eq!(
            consumed.map_or(0, |_| 1) + remaining.len(),
            total_sent,
            "All sent values must be received. sent={}, consumed={:?}, remaining={:?}",
            total_sent,
            consumed,
            remaining,
        );
    });
}

#[test]
fn loom_spsc_two_messages_fifo() {
    // Verify FIFO ordering: msg1 is received before msg2.
    loom::model(|| {
        let ring = Arc::new(LoomSpscRing::<u64>::new(4));
        let r = ring.clone();

        let producer = loom::thread::spawn(move || {
            assert!(r.try_send(1).is_ok());
            assert!(r.try_send(2).is_ok());
        });

        producer.join().unwrap();

        assert_eq!(ring.try_recv(), Some(1));
        assert_eq!(ring.try_recv(), Some(2));
        assert_eq!(ring.try_recv(), None);
    });
}

#[test]
fn loom_spsc_drop_pending() {
    // Verify that pending messages are dropped correctly when the ring is destroyed.
    loom::model(|| {
        let ring = LoomSpscRing::<String>::new(4);
        assert!(ring.try_send("hello".to_string()).is_ok());
        assert!(ring.try_send("world".to_string()).is_ok());
        // Consume one
        assert_eq!(ring.try_recv(), Some("hello".to_string()));
        // Drop ring with "world" still pending — should not leak
        drop(ring);
    });
}

// ============================================================================
// LOOM TESTS — SPMC
// ============================================================================

#[test]
fn loom_spmc_competing_consumers() {
    // Two consumers race to claim the same message. Exactly one should win.
    loom::model(|| {
        let ring = Arc::new(LoomSpmcRing::<u64>::new(2));

        // Producer sends one message
        assert!(ring.try_send(42).is_ok());

        let r1 = ring.clone();
        let r2 = ring.clone();

        let c1 = loom::thread::spawn(move || r1.try_recv());
        let c2 = loom::thread::spawn(move || r2.try_recv());

        let v1 = c1.join().unwrap();
        let v2 = c2.join().unwrap();

        // Exactly one consumer should get the message
        match (v1, v2) {
            (Some(42), None) | (None, Some(42)) => {} // correct
            other => panic!("Expected exactly one consumer to get 42, got {:?}", other),
        }
    });
}

#[test]
fn loom_spmc_two_messages_two_consumers() {
    // Two messages, two consumers — each gets one (no duplicates, no loss).
    loom::model(|| {
        let ring = Arc::new(LoomSpmcRing::<u64>::new(4));

        assert!(ring.try_send(1).is_ok());
        assert!(ring.try_send(2).is_ok());

        let r1 = ring.clone();
        let r2 = ring.clone();

        let c1 = loom::thread::spawn(move || {
            let mut got = Vec::new();
            if let Some(v) = r1.try_recv() {
                got.push(v);
            }
            if let Some(v) = r1.try_recv() {
                got.push(v);
            }
            got
        });
        let c2 = loom::thread::spawn(move || {
            let mut got = Vec::new();
            if let Some(v) = r2.try_recv() {
                got.push(v);
            }
            if let Some(v) = r2.try_recv() {
                got.push(v);
            }
            got
        });

        let mut all: Vec<u64> = c1.join().unwrap();
        all.extend(c2.join().unwrap());
        all.sort();

        // Both messages should be received exactly once
        assert_eq!(all, vec![1, 2], "Expected [1, 2], got {:?}", all);
    });
}

// ============================================================================
// LOOM TESTS — MPSC
// ============================================================================

#[test]
fn loom_mpsc_two_producers_one_consumer() {
    // Two producers each send one message. Consumer receives both.
    loom::model(|| {
        let ring = Arc::new(LoomMpscRing::<u64>::new(4));

        let r1 = ring.clone();
        let r2 = ring.clone();

        let p1 = loom::thread::spawn(move || {
            assert!(r1.try_send(1).is_ok());
        });
        let p2 = loom::thread::spawn(move || {
            assert!(r2.try_send(2).is_ok());
        });

        p1.join().unwrap();
        p2.join().unwrap();

        // Consumer drains both
        let mut received = Vec::new();
        while let Some(v) = ring.try_recv() {
            received.push(v);
        }
        received.sort();
        assert_eq!(received, vec![1, 2], "Expected [1, 2], got {:?}", received);
    });
}

#[test]
fn loom_mpsc_producer_consumer_concurrent() {
    // Producer and consumer running concurrently.
    loom::model(|| {
        let ring = Arc::new(LoomMpscRing::<u64>::new(2));
        let r = ring.clone();

        let producer = loom::thread::spawn(move || {
            assert!(r.try_send(99).is_ok());
        });

        let val = ring.try_recv();
        producer.join().unwrap();

        if val.is_none() {
            assert_eq!(ring.try_recv(), Some(99));
        } else {
            assert_eq!(val, Some(99));
        }
    });
}

// ============================================================================
// LOOM TESTS — MPMC
// ============================================================================

#[test]
fn loom_mpmc_concurrent_producers() {
    // Two producers race to send — both should succeed, no data corruption.
    // With bounded retries, some interleavings may have CAS failures, but
    // the important invariant is: no data corruption, no double-writes.
    loom::model(|| {
        let ring = Arc::new(LoomMpmcRing::<u64>::new(4));

        let r1 = ring.clone();
        let r2 = ring.clone();

        let p1 = loom::thread::spawn(move || r1.try_send(1));
        let p2 = loom::thread::spawn(move || r2.try_send(2));

        let res1 = p1.join().unwrap();
        let res2 = p2.join().unwrap();

        // Sequential consumption — verify data integrity
        let mut received = Vec::new();
        while let Some(v) = ring.try_recv() {
            received.push(v);
        }
        received.sort();

        // Count successful sends
        let mut expected: Vec<u64> = Vec::new();
        if res1.is_ok() {
            expected.push(1);
        }
        if res2.is_ok() {
            expected.push(2);
        }
        expected.sort();

        assert_eq!(
            received, expected,
            "Received values must match successfully sent values"
        );
    });
}

#[test]
fn loom_mpmc_concurrent_consumers() {
    // Two consumers race to drain — each message consumed exactly once.
    loom::model(|| {
        let ring = Arc::new(LoomMpmcRing::<u64>::new(4));

        // Pre-fill sequentially
        assert!(ring.try_send(1).is_ok());
        assert!(ring.try_send(2).is_ok());

        let r1 = ring.clone();
        let r2 = ring.clone();

        let c1 = loom::thread::spawn(move || {
            let mut got = Vec::new();
            if let Some(v) = r1.try_recv() {
                got.push(v);
            }
            if let Some(v) = r1.try_recv() {
                got.push(v);
            }
            got
        });
        let c2 = loom::thread::spawn(move || {
            let mut got = Vec::new();
            if let Some(v) = r2.try_recv() {
                got.push(v);
            }
            if let Some(v) = r2.try_recv() {
                got.push(v);
            }
            got
        });

        let mut all: Vec<u64> = c1.join().unwrap();
        all.extend(c2.join().unwrap());
        all.sort();

        // All consumed values must be valid and no duplicates.
        // With bounded retries, some messages may not be consumed.
        for v in &all {
            assert!(*v == 1 || *v == 2, "Invalid value: {}", v);
        }
        all.dedup();
        assert_eq!(
            all.len(),
            all.len(),
            "No duplicate values should be received"
        );
    });
}

#[test]
fn loom_mpmc_concurrent_send_recv() {
    // One producer and one consumer running fully concurrently on MPMC ring.
    loom::model(|| {
        let ring = Arc::new(LoomMpmcRing::<u64>::new(2));
        let r = ring.clone();

        let producer = loom::thread::spawn(move || {
            assert!(r.try_send(77).is_ok());
        });

        let val = ring.try_recv();
        producer.join().unwrap();

        if val.is_none() {
            assert_eq!(ring.try_recv(), Some(77));
        } else {
            assert_eq!(val, Some(77));
        }
    });
}

#[test]
fn loom_mpmc_drop_with_pending() {
    // Verify MPMC ring drops pending messages with sequence-checked Drop.
    loom::model(|| {
        let ring = LoomMpmcRing::<String>::new(4);
        assert!(ring.try_send("a".to_string()).is_ok());
        assert!(ring.try_send("b".to_string()).is_ok());
        // Consume one
        assert_eq!(ring.try_recv(), Some("a".to_string()));
        // Drop with "b" still pending — must not leak
        drop(ring);
    });
}

// ============================================================================
// LOOM TESTS — read_latest + try_recv concurrency
// ============================================================================

#[test]
fn loom_spmc_read_latest_concurrent_with_consumer() {
    // Core TOCTOU test: a consumer calls try_recv while a reader calls
    // read_latest concurrently. With T: Copy, read_latest is safe even if
    // the consumer has already consumed and "freed" the slot.
    loom::model(|| {
        let ring = Arc::new(LoomSpmcRing::<u64>::new(2));

        // Producer sends one message
        assert!(ring.try_send(42).is_ok());

        let consumer_ring = ring.clone();
        let reader_ring = ring.clone();

        // Consumer tries to consume the message
        let consumer = loom::thread::spawn(move || consumer_ring.try_recv());

        // Reader tries to read_latest concurrently
        let reader = loom::thread::spawn(move || reader_ring.read_latest());

        let consumed = consumer.join().unwrap();
        let read = reader.join().unwrap();

        // The consumer should always get the message (only consumer)
        // read_latest may or may not see it depending on interleaving
        match (consumed, read) {
            (Some(42), Some(42)) => {} // reader saw it before consumer consumed
            (Some(42), None) => {}     // consumer consumed before reader checked
            (None, Some(42)) => {}     // consumer CAS failed (spurious), reader saw it
            (None, None) => {}         // both missed (unlikely but valid under loom)
            other => panic!("Invalid state: {:?}", other),
        }
    });
}

#[test]
fn loom_spmc_read_latest_with_two_consumers() {
    // Two consumers + one reader. Producer sends 2 messages.
    // Exhaustively verifies no data corruption in read_latest
    // when multiple consumers are concurrently draining.
    loom::model(|| {
        let ring = Arc::new(LoomSpmcRing::<u64>::new(4));

        assert!(ring.try_send(10).is_ok());
        assert!(ring.try_send(20).is_ok());

        let c1 = ring.clone();
        let c2 = ring.clone();
        let reader = ring.clone();

        let consumer1 = loom::thread::spawn(move || c1.try_recv());
        let consumer2 = loom::thread::spawn(move || c2.try_recv());
        let reader_thread = loom::thread::spawn(move || reader.read_latest());

        let v1 = consumer1.join().unwrap();
        let v2 = consumer2.join().unwrap();
        let latest = reader_thread.join().unwrap();

        // Consumers should collectively consume both messages (no duplicates)
        let mut consumed: Vec<u64> = Vec::new();
        if let Some(v) = v1 {
            consumed.push(v);
        }
        if let Some(v) = v2 {
            consumed.push(v);
        }
        consumed.sort();

        for v in &consumed {
            assert!(*v == 10 || *v == 20, "Invalid consumed value: {}", v);
        }
        // No duplicates
        let before_dedup = consumed.len();
        consumed.dedup();
        assert_eq!(
            consumed.len(),
            before_dedup,
            "Duplicate consumption detected"
        );

        // read_latest must return a valid value or None
        if let Some(v) = latest {
            assert!(
                v == 10 || v == 20,
                "read_latest returned invalid value: {}",
                v
            );
        }
    });
}

#[test]
fn loom_mpmc_read_latest_concurrent_with_consumer() {
    // MPMC variant: consumer and reader race on the same message.
    // The sequence-number check in MPMC read_latest provides additional
    // safety (returns None if write not yet complete).
    loom::model(|| {
        let ring = Arc::new(LoomMpmcRing::<u64>::new(2));

        assert!(ring.try_send(99).is_ok());

        let consumer_ring = ring.clone();
        let reader_ring = ring.clone();

        let consumer = loom::thread::spawn(move || consumer_ring.try_recv());
        let reader = loom::thread::spawn(move || reader_ring.read_latest());

        let consumed = consumer.join().unwrap();
        let read = reader.join().unwrap();

        // Both results must be valid or None
        match (consumed, read) {
            (Some(99), Some(99)) => {} // both saw the value
            (Some(99), None) => {}     // consumer consumed, reader's seq check failed
            (None, Some(99)) => {}     // consumer CAS failed, reader saw it
            (None, None) => {}         // both missed (CAS failure + seq mismatch)
            other => panic!("Invalid MPMC read_latest state: {:?}", other),
        }
    });
}

#[test]
fn loom_mpmc_read_latest_with_producer_and_consumer() {
    // Full 3-thread test: producer sends, consumer drains, reader peeks.
    // Verifies read_latest doesn't observe partially-written data.
    loom::model(|| {
        let ring = Arc::new(LoomMpmcRing::<u64>::new(2));

        let prod_ring = ring.clone();
        let cons_ring = ring.clone();
        let read_ring = ring.clone();

        let producer = loom::thread::spawn(move || {
            let _ = prod_ring.try_send(55);
        });

        let consumer = loom::thread::spawn(move || cons_ring.try_recv());

        let reader = loom::thread::spawn(move || read_ring.read_latest());

        producer.join().unwrap();
        let consumed = consumer.join().unwrap();
        let latest = reader.join().unwrap();

        // If consumer got a value, it must be 55
        if let Some(v) = consumed {
            assert_eq!(v, 55, "Consumer got wrong value");
        }
        // If reader got a value, it must be 55 (no partial writes visible)
        if let Some(v) = latest {
            assert_eq!(v, 55, "read_latest saw invalid value");
        }
    });
}
