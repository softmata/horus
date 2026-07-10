//! Loom-based exhaustive concurrency test for the TensorPool free-slot allocator.
//!
//! Models the lock-free free-slot protocol in
//! `horus_core::communication`… `memory::tensor_pool` (the Treiber free stack +
//! `pop_and_claim` + `push_free_slot`) with loom atomics, exploring every
//! interleaving of concurrent alloc/release.
//!
//! # What is verified (the softmata-brain 1333 invariants)
//!
//!  1. **No double-allocation**: two concurrent allocs NEVER receive the same
//!     slot (the old dual free-stack + linear-scan design did — memory aliasing /
//!     data corruption). Enforced via a per-slot "occupied" flag: claiming swaps
//!     it true and asserts it was false.
//!  2. **No leak / no `next_free` cycle**: after all threads finish, the free
//!     stack drains to exactly the original slot count (a double-push would form
//!     a cycle and desync push/pop counts).
//!
//! # Modeling note
//!
//! This mirrors the production algorithm — all slots pushed at init, pop is the
//! SOLE claim — with loom atomics, exactly as `loom_fanout.rs` mirrors the
//! seqlock ring. A preemption bound keeps exploration tractable.
//!
//! Run with: `cargo test --test loom_tensor_pool -- --nocapture`

use loom::sync::atomic::{AtomicBool, AtomicU32, AtomicU64, Ordering};
use loom::sync::Arc;

const INVALID: u32 = u32::MAX;
const FREE: u32 = 0;
const ALLOCATED: u32 = 1;

fn pack(generation: u32, slot_id: u32) -> u64 {
    ((generation as u64) << 32) | slot_id as u64
}
fn unpack(tagged: u64) -> (u32, u32) {
    ((tagged >> 32) as u32, (tagged & 0xFFFF_FFFF) as u32)
}

struct Slot {
    next_free: AtomicU64, // holds a slot_id (or INVALID) in the low bits
    flags: AtomicU32,
    owner: AtomicU32,
    /// Test-only sentinel: true while a thread believes it exclusively holds
    /// this slot. A correct allocator keeps this single-owner at all times.
    occupied: AtomicBool,
}

struct PoolModel {
    free_stack_head: AtomicU64,
    slots: Vec<Slot>,
}

impl PoolModel {
    /// Build a pool with `n` slots, ALL pushed onto the free stack (mirrors the
    /// production `initialize()`; single-threaded, no CAS).
    fn new(n: u32) -> Self {
        let mut slots = Vec::new();
        let mut head = INVALID;
        for _ in 0..n {
            slots.push(Slot {
                next_free: AtomicU64::new(0),
                flags: AtomicU32::new(FREE),
                owner: AtomicU32::new(0),
                occupied: AtomicBool::new(false),
            });
        }
        for i in 0..n {
            slots[i as usize].next_free.store(head as u64, Ordering::Relaxed);
            head = i;
        }
        Self {
            free_stack_head: AtomicU64::new(pack(0, head)),
            slots,
        }
    }

    /// Mirror of production `pop_and_claim`: Treiber pop is the sole claim.
    fn pop_and_claim(&self) -> Option<u32> {
        loop {
            let tagged = self.free_stack_head.load(Ordering::Acquire);
            let (gen, slot_id) = unpack(tagged);
            if slot_id == INVALID {
                return None;
            }
            let next = self.slots[slot_id as usize].next_free.load(Ordering::Acquire) as u32;
            let new_tagged = pack(gen.wrapping_add(1), next);
            if self
                .free_stack_head
                .compare_exchange_weak(tagged, new_tagged, Ordering::AcqRel, Ordering::Relaxed)
                .is_ok()
            {
                let slot = &self.slots[slot_id as usize];
                slot.owner.store(1, Ordering::Release);
                slot.flags.store(ALLOCATED, Ordering::Release);
                return Some(slot_id);
            }
        }
    }

    /// Mirror of production `return_slot` + `push_free_slot`.
    fn release(&self, slot_id: u32) {
        let slot = &self.slots[slot_id as usize];
        slot.flags.store(FREE, Ordering::Release);
        slot.owner.store(0, Ordering::Release);
        loop {
            let tagged = self.free_stack_head.load(Ordering::Acquire);
            let (gen, head) = unpack(tagged);
            slot.next_free.store(head as u64, Ordering::Release);
            let new_tagged = pack(gen.wrapping_add(1), slot_id);
            if self
                .free_stack_head
                .compare_exchange_weak(tagged, new_tagged, Ordering::AcqRel, Ordering::Relaxed)
                .is_ok()
            {
                break;
            }
        }
    }

    /// Drain the free stack, returning the number of slots on it. A `next_free`
    /// cycle (from a double-push) would exceed `bound` and trip the assert.
    fn drain_count(&self, bound: u32) -> u32 {
        let mut count = 0;
        let (_, mut head) = unpack(self.free_stack_head.load(Ordering::Acquire));
        while head != INVALID {
            count += 1;
            assert!(count <= bound, "free-stack cycle detected (double-push)");
            head = self.slots[head as usize].next_free.load(Ordering::Acquire) as u32;
        }
        count
    }
}

/// One alloc→release cycle with the exclusive-occupancy check.
fn alloc_release(pool: &PoolModel) {
    if let Some(slot) = pool.pop_and_claim() {
        // Exclusive-ownership invariant: no other thread may hold this slot.
        let was = pool.slots[slot as usize].occupied.swap(true, Ordering::AcqRel);
        assert!(!was, "DOUBLE-ALLOCATION: slot {slot} handed to two threads");
        pool.slots[slot as usize].occupied.store(false, Ordering::Release);
        pool.release(slot);
    }
}

fn run(n_slots: u32, n_threads: usize) {
    let mut builder = loom::model::Builder::new();
    builder.preemption_bound = Some(3);
    builder.check(move || {
        let pool = Arc::new(PoolModel::new(n_slots));

        let handles: Vec<_> = (0..n_threads)
            .map(|_| {
                let p = pool.clone();
                loom::thread::spawn(move || alloc_release(&p))
            })
            .collect();
        for h in handles {
            h.join().unwrap();
        }

        // No leak / no cycle: every slot is back on the free stack.
        assert_eq!(
            pool.drain_count(n_slots + 1),
            n_slots,
            "slot leak: free stack does not hold all {n_slots} slots after release"
        );
    });
}

#[test]
fn loom_pool_2slots_2threads() {
    // Fewer slots than needed to trivially isolate — forces threads to contend
    // for the same slots across pop/push.
    run(2, 2);
}

#[test]
fn loom_pool_1slot_2threads() {
    // One slot, two threads: the tightest contention (both fight for slot 0).
    run(1, 2);
}
