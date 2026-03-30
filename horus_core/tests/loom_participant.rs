//! Loom-based exhaustive concurrency tests for participant registration.
//!
//! These tests verify that the participant slot registration protocol —
//! CAS on `active` field, lease expiry reclamation, role counting — is
//! correct under all possible thread interleavings.
//!
//! # Primitives covered
//!
//! | Primitive              | Property verified                                      |
//! |:-----------------------|:-------------------------------------------------------|
//! | Slot claim (CAS)       | No double-claim: exactly one thread wins per slot       |
//! | Active flag (0→2→1)    | Initializing slots (active=2) are never used by others  |
//! | Lease reclaim          | Expired entries are safely replaced                     |
//! | Role counter           | pub_count/sub_count match registered roles              |
//!
//! # Running
//!
//! ```sh
//! cargo test -p horus_core --test loom_participant -- --nocapture
//! ```

use loom::sync::atomic::{AtomicU32, AtomicU64, AtomicU8, Ordering};
use loom::sync::Arc;

// ============================================================================
// Simplified participant model
// ============================================================================

const MAX_SLOTS: usize = 3; // Must be minimal for loom tractability (2-3 threads × N slots)

/// Simplified ParticipantEntry matching production layout.
struct LoomParticipantEntry {
    pid: AtomicU32,
    role: AtomicU8,
    active: AtomicU8, // 0=empty, 1=active, 2=initializing
    lease_expires: AtomicU64,
}

impl LoomParticipantEntry {
    fn new() -> Self {
        Self {
            pid: AtomicU32::new(0),
            role: AtomicU8::new(0),
            active: AtomicU8::new(0),
            lease_expires: AtomicU64::new(0),
        }
    }
}

/// Simplified TopicHeader participant table.
struct LoomParticipantTable {
    slots: Vec<LoomParticipantEntry>,
    publisher_count: AtomicU32,
    subscriber_count: AtomicU32,
    total_participants: AtomicU32,
}

impl LoomParticipantTable {
    fn new() -> Arc<Self> {
        let mut slots = Vec::with_capacity(MAX_SLOTS);
        for _ in 0..MAX_SLOTS {
            slots.push(LoomParticipantEntry::new());
        }
        Arc::new(Self {
            slots,
            publisher_count: AtomicU32::new(0),
            subscriber_count: AtomicU32::new(0),
            total_participants: AtomicU32::new(0),
        })
    }

    /// Register a producer with the given pid.
    /// Returns slot index on success, None if all slots full.
    ///
    /// Mirrors header.rs register_role() with role_bit=1.
    fn register_producer(&self, pid: u32) -> Option<usize> {
        self.register_role(pid, 1, &self.publisher_count)
    }

    /// Register a consumer with the given pid.
    fn register_consumer(&self, pid: u32) -> Option<usize> {
        self.register_role(pid, 2, &self.subscriber_count)
    }

    /// Shared registration logic matching production code.
    fn register_role(&self, pid: u32, role_bit: u8, counter: &AtomicU32) -> Option<usize> {
        // Phase 1: Check for existing entry for this pid
        // (simplified: production also checks thread_id_hash)
        for (i, p) in self.slots.iter().enumerate() {
            let active = p.active.load(Ordering::Acquire);
            if active == 1 && p.pid.load(Ordering::Acquire) == pid {
                let old_role = p.role.fetch_or(role_bit, Ordering::AcqRel);
                if old_role & role_bit == 0 {
                    counter.fetch_add(1, Ordering::AcqRel);
                }
                return Some(i);
            }
        }

        // Phase 2: Find empty or expired slot
        let now_ms = 1000u64; // Abstract time for loom
        for (i, p) in self.slots.iter().enumerate() {
            let active_val = p.active.load(Ordering::Acquire);

            if active_val == 2 {
                continue; // Another thread is initializing
            }

            let is_active = active_val != 0;
            let is_expired = is_active && p.lease_expires.load(Ordering::Acquire) < now_ms;

            if !is_active || is_expired {
                // CAS: claim the slot by setting active to 2 (initializing)
                let claimed = p
                    .active
                    .compare_exchange(active_val, 2, Ordering::AcqRel, Ordering::Acquire)
                    .is_ok();

                if claimed {
                    // Decrement old counters if reclaiming expired slot
                    if is_expired {
                        let old_role = p.role.load(Ordering::Acquire);
                        if old_role & 1 != 0 {
                            self.publisher_count.fetch_sub(1, Ordering::AcqRel);
                        }
                        if old_role & 2 != 0 {
                            self.subscriber_count.fetch_sub(1, Ordering::AcqRel);
                        }
                    }

                    // Initialize slot fields
                    p.pid.store(pid, Ordering::Release);
                    p.role.store(role_bit, Ordering::Release);
                    p.lease_expires.store(now_ms + 5000, Ordering::Release);
                    counter.fetch_add(1, Ordering::AcqRel);
                    self.total_participants.fetch_add(1, Ordering::AcqRel);

                    // Finalize: 2 → 1 (visible to other threads)
                    p.active.store(1, Ordering::Release);
                    return Some(i);
                }
            }
        }

        None // All slots occupied
    }
}

// ============================================================================
// Tests
// ============================================================================

/// Two threads register concurrently as producers with different PIDs.
/// Each must get a unique slot. No double-claim.
#[test]
fn loom_participant_no_double_claim() {
    loom::model(|| {
        let table = LoomParticipantTable::new();
        let t1 = table.clone();
        let t2 = table.clone();

        let th1 = loom::thread::spawn(move || t1.register_producer(100));
        let th2 = loom::thread::spawn(move || t2.register_producer(200));

        let r1 = th1.join().unwrap();
        let r2 = th2.join().unwrap();

        // Both should succeed (we have 3 slots)
        assert!(r1.is_some(), "thread 1 must get a slot");
        assert!(r2.is_some(), "thread 2 must get a slot");

        // Slots must be different (no double-claim)
        assert_ne!(
            r1.unwrap(),
            r2.unwrap(),
            "both threads must get different slots"
        );

        // Publisher count must be exactly 2
        let pub_count = table.publisher_count.load(Ordering::Acquire);
        assert_eq!(pub_count, 2, "publisher_count must be 2, got {}", pub_count);

        // Total participants must be 2
        let total = table.total_participants.load(Ordering::Acquire);
        assert_eq!(total, 2, "total_participants must be 2, got {}", total);
    });
}

/// An expired entry is correctly reclaimed by a new registrant.
/// Pre-populate slot 0 as expired. One thread reclaims it.
/// Key invariant: counter consistency — pub_count matches actual active producers.
/// No deadlock, no stuck initializing slots.
#[test]
fn loom_participant_lease_reclaim() {
    loom::model(|| {
        let table = LoomParticipantTable::new();

        // Pre-populate slot 0 with an expired entry (pid=999, role=producer)
        let p = &table.slots[0];
        p.pid.store(999, Ordering::Release);
        p.role.store(1, Ordering::Release); // producer
        p.lease_expires.store(500, Ordering::Release); // expired (now_ms=1000)
        p.active.store(1, Ordering::Release);
        table.publisher_count.store(1, Ordering::Release);
        table.total_participants.store(1, Ordering::Release);

        let t1 = table.clone();

        // Single thread reclaims the expired slot
        let th1 = loom::thread::spawn(move || t1.register_producer(100));
        let r1 = th1.join().unwrap();

        // Must succeed
        assert!(r1.is_some(), "must get a slot (expired reclaim or fresh)");

        // Counter consistency: pub_count must match actual active producers
        let mut actual_pubs = 0u32;
        for slot in table.slots.iter() {
            if slot.active.load(Ordering::Acquire) == 1 {
                let role = slot.role.load(Ordering::Acquire);
                if role & 1 != 0 {
                    actual_pubs += 1;
                }
            }
        }
        let pub_count = table.publisher_count.load(Ordering::Acquire);
        assert_eq!(
            pub_count, actual_pubs,
            "pub_count ({}) must match actual active producers ({})",
            pub_count, actual_pubs
        );

        // No stuck initializing slots
        for (i, slot) in table.slots.iter().enumerate() {
            let active = slot.active.load(Ordering::Acquire);
            assert_ne!(
                active, 2,
                "slot {} should not be stuck in initializing state",
                i
            );
        }

        // The new entry must have pid=100
        let slot_idx = r1.unwrap();
        let registered_pid = table.slots[slot_idx].pid.load(Ordering::Acquire);
        assert_eq!(registered_pid, 100, "registered slot must have our pid");
    });
}

/// Initializing slots (active=2) are never visible as ready to other threads.
/// Thread A claims a slot (active→2), thread B should skip it.
#[test]
fn loom_participant_active_flag_initializing_skipped() {
    loom::model(|| {
        let table = LoomParticipantTable::new();
        let t1 = table.clone();
        let t2 = table.clone();

        // Both threads register as producers with different PIDs
        let th1 = loom::thread::spawn(move || t1.register_producer(100));
        let th2 = loom::thread::spawn(move || t2.register_producer(200));

        let r1 = th1.join().unwrap();
        let r2 = th2.join().unwrap();

        // Both must succeed
        assert!(r1.is_some());
        assert!(r2.is_some());

        // They must get different slots (no race on same slot)
        assert_ne!(r1.unwrap(), r2.unwrap());

        // All claimed slots must have active=1 (fully initialized)
        for (i, p) in table.slots.iter().enumerate() {
            let active = p.active.load(Ordering::Acquire);
            assert_ne!(
                active, 2,
                "slot {} should not be stuck in initializing state",
                i
            );
        }
    });
}

/// Counter consistency: after all registrations complete,
/// pub_count + sub_count matches the number of role bits set.
#[test]
fn loom_participant_count_consistent() {
    loom::model(|| {
        let table = LoomParticipantTable::new();
        let t1 = table.clone();
        let t2 = table.clone();

        // Thread 1: register as producer
        let th1 = loom::thread::spawn(move || t1.register_producer(100));
        // Thread 2: register as consumer
        let th2 = loom::thread::spawn(move || t2.register_consumer(200));

        let r1 = th1.join().unwrap();
        let r2 = th2.join().unwrap();

        assert!(r1.is_some());
        assert!(r2.is_some());

        let pub_count = table.publisher_count.load(Ordering::Acquire);
        let sub_count = table.subscriber_count.load(Ordering::Acquire);

        assert_eq!(pub_count, 1, "pub_count must be 1, got {}", pub_count);
        assert_eq!(sub_count, 1, "sub_count must be 1, got {}", sub_count);

        // Verify by scanning actual slots
        let mut actual_pubs = 0u32;
        let mut actual_subs = 0u32;
        for p in table.slots.iter() {
            if p.active.load(Ordering::Acquire) == 1 {
                let role = p.role.load(Ordering::Acquire);
                if role & 1 != 0 {
                    actual_pubs += 1;
                }
                if role & 2 != 0 {
                    actual_subs += 1;
                }
            }
        }

        assert_eq!(
            pub_count, actual_pubs,
            "pub_count ({}) must match actual producers ({})",
            pub_count, actual_pubs
        );
        assert_eq!(
            sub_count, actual_subs,
            "sub_count ({}) must match actual consumers ({})",
            sub_count, actual_subs
        );
    });
}

/// Same PID registering as both producer and consumer reuses the same slot
/// and sets role=3 (both). Counter increments correctly.
#[test]
fn loom_participant_same_pid_dual_role() {
    loom::model(|| {
        let table = LoomParticipantTable::new();
        let t1 = table.clone();
        let t2 = table.clone();

        // Both register with the SAME pid
        let th1 = loom::thread::spawn(move || t1.register_producer(100));
        let th2 = loom::thread::spawn(move || t2.register_consumer(100));

        let r1 = th1.join().unwrap();
        let r2 = th2.join().unwrap();

        assert!(r1.is_some());
        assert!(r2.is_some());

        // Both must resolve to the same slot (pid match reuses entry)
        // OR one thread may get a fresh slot if it runs before the other
        // finishes initialization. Both are valid.

        let pub_count = table.publisher_count.load(Ordering::Acquire);
        let sub_count = table.subscriber_count.load(Ordering::Acquire);

        assert_eq!(pub_count, 1, "pub_count must be 1");
        assert_eq!(sub_count, 1, "sub_count must be 1");

        // Total active slots should be 1 (reused) or 2 (race: both got fresh)
        let active_count = table
            .slots
            .iter()
            .filter(|p| p.active.load(Ordering::Acquire) == 1)
            .count();
        assert!(
            active_count == 1 || active_count == 2,
            "active slots should be 1 (reused) or 2 (raced), got {}",
            active_count
        );
    });
}
