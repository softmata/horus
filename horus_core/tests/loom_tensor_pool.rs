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

// ───────────────────────────────────────────────────────────────────────────
// COMM-H2 fix: refcount + generation retain/release safety.
//
// The multi-subscriber zero-copy fix has each subscriber `try_retain` the shared
// pool slot (owning its own reference) and the producer release the keep-alive on
// its NEXT send. A late subscriber can therefore `try_retain` a descriptor whose
// slot the producer has already released AND reallocated for the next frame (its
// generation bumped) — the `try_retain` ABA. The hardened `try_retain` closes it
// by re-checking the generation AFTER the increment and undoing on mismatch.
//
// This models that race and verifies, under every interleaving: (1) a successful
// retain is always on the descriptor's OWN generation — never a reused slot (no
// wrong-retain); (2) no double-free or reuse-of-live slot — caught inline by an
// `occupied` sentinel, exercised down to the undo's return-slot branch by the
// producer's gen-2 release; (3) no leak — every allocation is returned and the
// refcount drains to 0. Reverting the re-check trips (1); removing the CAS's
// refcount-0 guard trips (2) as a double-free.
// ───────────────────────────────────────────────────────────────────────────

struct RcSlot {
    generation: AtomicU64,
    refcount: AtomicU64,
    /// Test sentinel (mirrors the existing `loom_pool` `occupied` flag): `true`
    /// while the slot holds a live allocation. `return_slot` flips it `false` and
    /// asserts it was `true` (a double-free — returning an already-free slot —
    /// trips here); `reuse` flips it `true` and asserts it was `false`.
    occupied: AtomicBool,
}

impl RcSlot {
    /// Return the slot to the free list (mirror of `TensorPool::return_slot`),
    /// asserting it was actually allocated — a second free of the same
    /// allocation trips this at the exact moment it happens.
    fn return_slot(&self) {
        assert!(
            self.occupied.swap(false, Ordering::AcqRel),
            "double-free: returned a slot that was already free"
        );
    }

    /// Mirror of the hardened `TensorPool::try_retain`: generation pre-check,
    /// CAS-increment-if-`> 0`, then a generation RE-CHECK that closes the ABA —
    /// on mismatch the phantom increment is undone directly (not via the
    /// generation-checking `release`), returning the slot if we held the last ref.
    fn try_retain(&self, desc_gen: u64) -> bool {
        if self.generation.load(Ordering::Acquire) != desc_gen {
            return false;
        }
        loop {
            let cur = self.refcount.load(Ordering::Acquire);
            if cur == 0 {
                return false;
            }
            if self
                .refcount
                .compare_exchange_weak(cur, cur + 1, Ordering::AcqRel, Ordering::Relaxed)
                .is_ok()
            {
                // ABA close: if the slot was freed+reallocated in the CAS window,
                // we incremented the reused slot — undo directly (fetch_sub, and
                // return the slot if our decrement was the one that reached 0).
                if self.generation.load(Ordering::Acquire) != desc_gen {
                    if self.refcount.fetch_sub(1, Ordering::AcqRel) == 1 {
                        self.return_slot();
                    }
                    return false;
                }
                return true;
            }
        }
    }

    /// Mirror of `TensorPool::release`: generation-check (no-op on a stale
    /// descriptor — the load-bearing safety fact), then CAS-decrement-if-`> 0`,
    /// returning the slot on reaching 0.
    fn release(&self, desc_gen: u64) {
        if self.generation.load(Ordering::Acquire) != desc_gen {
            return; // stale descriptor — never decrements a reused slot
        }
        loop {
            let cur = self.refcount.load(Ordering::Acquire);
            if cur == 0 {
                return;
            }
            if self
                .refcount
                .compare_exchange_weak(cur, cur - 1, Ordering::AcqRel, Ordering::Relaxed)
                .is_ok()
            {
                if cur == 1 {
                    self.return_slot();
                }
                return;
            }
        }
    }

    /// Mirror of `pop_and_claim`: atomically claim the slot IF it is free (i.e.
    /// on the free list), then bump the generation and set refcount to 1. Returns
    /// whether the claim succeeded. Gating on the occupancy CAS — not a racy
    /// `rc == 0` peek — mirrors popping the slot off the free stack, which
    /// `return_slot` pushes to only AFTER clearing occupancy.
    fn reuse(&self) -> bool {
        if self
            .occupied
            .compare_exchange(false, true, Ordering::AcqRel, Ordering::Acquire)
            .is_err()
        {
            return false; // still live / not yet returned — cannot reuse
        }
        self.generation.fetch_add(1, Ordering::AcqRel);
        self.refcount.store(1, Ordering::Release);
        true
    }
}

#[test]
fn loom_comm_h2_stale_retain_never_double_frees() {
    let mut builder = loom::model::Builder::new();
    builder.preemption_bound = Some(3);
    builder.check(|| {
        // Producer holds the keep-alive of the message published at generation 1
        // (slot live); a subscriber has the matching descriptor and is about to read.
        let slot = Arc::new(RcSlot {
            generation: AtomicU64::new(1),
            refcount: AtomicU64::new(1),
            occupied: AtomicBool::new(true),
        });
        let desc_gen = 1u64;

        let p = slot.clone();
        let producer = loom::thread::spawn(move || {
            // Supersede on next send: release the keep-alive. If that returned the
            // slot to the free list, the next frame claims it (gen bump) AND later
            // drops it too — that second release is what drives the consumer's ABA
            // undo down to its return-slot (`prev == 1`) branch.
            p.release(desc_gen);
            if p.reuse() {
                p.release(2);
            }
        });

        let c = slot.clone();
        let consumer = loom::thread::spawn(move || {
            // Late subscriber takes its own reference off the (possibly stale)
            // descriptor, then releases it on drop.
            if c.try_retain(desc_gen) {
                // The hardened try_retain guarantees a successful retain is on OUR
                // generation, never a reused slot. We now hold a reference, so the
                // generation is pinned — this read is stable, not racy. (Before the
                // fix, the ABA could make this a gen-2 slot: assertion would trip.)
                assert_eq!(
                    c.generation.load(Ordering::Acquire),
                    desc_gen,
                    "wrong-generation retain — the try_retain ABA is not closed"
                );
                c.release(desc_gen);
            }
        });

        producer.join().unwrap();
        consumer.join().unwrap();

        // Double-free and reuse-of-live are caught inline by the `occupied` sentinel
        // (including on the undo's return-slot branch, which the gen-2 release above
        // now exercises). No leak: every allocation was returned and the refcount
        // drained to 0 — a leaked phantom ref (the un-hardened ABA) would leave rc > 0.
        assert!(
            !slot.occupied.load(Ordering::Acquire),
            "leak: slot left allocated after all references dropped"
        );
        assert_eq!(
            slot.refcount.load(Ordering::Acquire),
            0,
            "refcount not drained to 0 — leaked or underflowed reference"
        );
    });
}
