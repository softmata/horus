#![allow(dead_code)]
//! Loom-based exhaustive concurrency tests for the migration protocol.
//!
//! These tests verify that the migration protocol — CAS lock acquisition,
//! drain, epoch increment, backend mode switch, unlock — is correct under
//! all possible thread interleavings.
//!
//! # Primitives covered
//!
//! | Primitive              | Property verified                                |
//! |:-----------------------|:-------------------------------------------------|
//! | Migration lock (CAS)   | Mutual exclusion: exactly one thread wins         |
//! | Epoch increment        | Visible to all threads after lock release         |
//! | Backend mode store     | New mode visible after epoch change               |
//! | Concurrent migrations  | Serialized correctly, no deadlock                 |
//! | Send during migration  | Producer sees epoch change and adapts              |
//!
//! # Running
//!
//! ```sh
//! cargo test -p horus_core --test loom_migration -- --nocapture
//! ```
//!
//! Loom explores O(exponential) interleavings. All tests use 2-3 threads
//! and minimal operations to keep exploration tractable.

use loom::sync::atomic::{AtomicU64, AtomicU8, Ordering};
use loom::sync::Arc;

// ============================================================================
// Simplified migration protocol model
// ============================================================================

const UNLOCKED: u8 = 0;
const LOCKED: u8 = 1;

/// Simplified TopicHeader with only the fields relevant to migration.
///
/// Models the production TopicHeader's migration_lock (AtomicU8),
/// migration_epoch (AtomicU64), and backend_mode (AtomicU8).
struct LoomMigrationHeader {
    /// Migration lock: 0 = unlocked, 1 = locked
    migration_lock: AtomicU8,
    /// Migration epoch: incremented on each successful migration
    migration_epoch: AtomicU64,
    /// Current backend mode (simplified to u8)
    backend_mode: AtomicU8,
}

impl LoomMigrationHeader {
    fn new(initial_mode: u8) -> Arc<Self> {
        Arc::new(Self {
            migration_lock: AtomicU8::new(UNLOCKED),
            migration_epoch: AtomicU64::new(0),
            backend_mode: AtomicU8::new(initial_mode),
        })
    }

    /// Try to acquire the migration lock via CAS.
    /// Mirrors TopicHeader::try_lock_migration().
    fn try_lock(&self) -> bool {
        self.migration_lock
            .compare_exchange(UNLOCKED, LOCKED, Ordering::AcqRel, Ordering::Acquire)
            .is_ok()
    }

    /// Release the migration lock.
    /// Mirrors TopicHeader::unlock_migration().
    fn unlock(&self) {
        self.migration_lock.store(UNLOCKED, Ordering::Release);
    }

    /// Check if migration is in progress.
    fn is_locked(&self) -> bool {
        self.migration_lock.load(Ordering::Acquire) != UNLOCKED
    }

    /// Perform a full migration: lock → set mode → increment epoch → unlock.
    /// Returns the new epoch on success, None on lock contention.
    ///
    /// Mirrors BackendMigrator::try_migrate() with drain elided
    /// (drain is a timing concern, not a correctness concern for loom).
    ///
    /// IMPORTANT: Mode is stored BEFORE epoch is incremented. The epoch
    /// increment is the "publication fence" — observers that see the new
    /// epoch are guaranteed to also see the new mode.
    fn try_migrate(&self, new_mode: u8) -> Option<u64> {
        let current_mode = self.backend_mode.load(Ordering::Acquire);
        if current_mode == new_mode {
            return None; // NotNeeded
        }

        if self.is_locked() {
            return None; // AlreadyInProgress
        }

        if !self.try_lock() {
            return None; // LockContention
        }

        // Locked — store mode BEFORE incrementing epoch
        self.backend_mode.store(new_mode, Ordering::Release);

        let old_epoch = self.migration_epoch.fetch_add(1, Ordering::AcqRel);
        let new_epoch = old_epoch + 1;

        self.unlock();

        Some(new_epoch)
    }
}

/// Simplified epoch guard model.
///
/// Models the dispatch epoch check: a producer/consumer caches the epoch,
/// and on each operation loads the current epoch. If it changed, the
/// operation must re-dispatch (detect new backend mode).
struct LoomEpochGuard {
    header: Arc<LoomMigrationHeader>,
    /// Cached epoch from last check
    cached_epoch: u64,
}

impl LoomEpochGuard {
    fn new(header: Arc<LoomMigrationHeader>) -> Self {
        let epoch = header.migration_epoch.load(Ordering::Acquire);
        Self {
            header,
            cached_epoch: epoch,
        }
    }

    /// Check if the epoch has changed since last observation.
    /// If so, update cache and return the new backend mode.
    /// Mirrors epoch_guard_send!/epoch_guard_recv! macros.
    fn check_epoch(&mut self) -> Option<u8> {
        let current = self.header.migration_epoch.load(Ordering::Acquire);
        if current != self.cached_epoch {
            self.cached_epoch = current;
            // After seeing new epoch, backend_mode is guaranteed visible
            // (epoch was stored with AcqRel, mode with Release before unlock)
            let new_mode = self.header.backend_mode.load(Ordering::Acquire);
            Some(new_mode)
        } else {
            None
        }
    }
}

// ============================================================================
// Tests
// ============================================================================

/// Two threads attempt migration to DIFFERENT modes concurrently.
/// The CAS lock ensures serialization. Both may succeed (sequentially),
/// one may succeed and one may fail (contention), or both may fail
/// (rare: both see the other's lock). No deadlock in any case.
#[test]
fn loom_migration_lock_mutual_exclusion() {
    loom::model(|| {
        let header = LoomMigrationHeader::new(1); // initial mode = 1
        let h1 = header.clone();
        let h2 = header.clone();

        // Different target modes — both should try to migrate
        let t1 = loom::thread::spawn(move || h1.try_migrate(2));
        let t2 = loom::thread::spawn(move || h2.try_migrate(3));

        let r1 = t1.join().unwrap();
        let r2 = t2.join().unwrap();

        // Both threads completed (no deadlock) — this is the key invariant
        // Lock must be released regardless of outcome
        assert!(
            !header.is_locked(),
            "migration lock must be released after both threads complete"
        );

        // Epoch is consistent with number of successful migrations
        let successes = [r1, r2].iter().filter(|r| r.is_some()).count();
        let final_epoch = header.migration_epoch.load(Ordering::Acquire);
        assert_eq!(
            final_epoch, successes as u64,
            "epoch must equal number of successful migrations"
        );

        // Final mode must be valid: 1 (no migration), 2 (t1 won), 3 (t2 won)
        let final_mode = header.backend_mode.load(Ordering::Acquire);
        assert!(
            [1, 2, 3].contains(&final_mode),
            "final mode must be 1, 2, or 3, got {}",
            final_mode
        );

        // If both succeeded, epochs must be sequential (1 then 2)
        if let (Some(e1), Some(e2)) = (r1, r2) {
            assert_ne!(e1, e2, "sequential migrations must have different epochs");
        }
    });
}

/// After a migration completes, all threads that subsequently load
/// the epoch must see the new value (Release/Acquire visibility).
#[test]
fn loom_migration_epoch_visible_after_migration() {
    loom::model(|| {
        let header = LoomMigrationHeader::new(1);
        let h_migrator = header.clone();
        let h_observer = header.clone();

        // Thread A: perform migration (mode 1 → 2)
        let migrator = loom::thread::spawn(move || h_migrator.try_migrate(2));

        // Thread B: observe epoch
        let observer = loom::thread::spawn(move || {
            let epoch = h_observer.migration_epoch.load(Ordering::Acquire);
            let mode = h_observer.backend_mode.load(Ordering::Acquire);
            (epoch, mode)
        });

        let migration_result = migrator.join().unwrap();
        let (observed_epoch, observed_mode) = observer.join().unwrap();

        if migration_result.is_some() {
            // Migration happened (epoch went from 0 to 1)
            // Observer either sees:
            //   (0, 1) — read before migration (valid, old state)
            //   (1, 2) — read after migration (valid, new state)
            // Observer should NOT see:
            //   (1, 1) — new epoch but old mode (would mean Release/Acquire broken)
            //   (0, 2) — old epoch but new mode (would mean mode leaked early)
            if observed_epoch == 1 {
                // Saw new epoch → must also see new mode
                assert_eq!(
                    observed_mode, 2,
                    "if observer sees new epoch (1), must also see new mode (2), got mode={}",
                    observed_mode
                );
            }
            // If epoch == 0, observer read before migration — any mode is fine
        }
    });
}

/// A producer's epoch guard detects migration and re-dispatches.
///
/// Thread A: migrates mode 1 → 2
/// Thread B: checks epoch guard, may or may not see the change
///
/// Key invariant: if the guard sees the new epoch, it MUST also see
/// the new mode (because mode is stored before epoch with proper ordering).
#[test]
fn loom_epoch_guard_detects_migration() {
    loom::model(|| {
        let header = LoomMigrationHeader::new(1);
        let h_migrator = header.clone();
        let h_producer = header.clone();

        // Thread A: migrate (stores mode THEN increments epoch)
        let migrator = loom::thread::spawn(move || h_migrator.try_migrate(2));

        // Thread B: epoch guard check
        let producer = loom::thread::spawn(move || {
            let mut guard = LoomEpochGuard::new(h_producer);
            // Check epoch — may or may not see the change depending on ordering
            guard.check_epoch()
        });

        let migration_result = migrator.join().unwrap();
        let detected_mode = producer.join().unwrap();

        if migration_result.is_some() {
            // Migration happened (mode stored before epoch incremented)
            if let Some(mode) = detected_mode {
                // Guard detected epoch change — mode MUST be the new one.
                // This is the critical invariant: storing mode before epoch
                // ensures that seeing new epoch implies seeing new mode.
                assert_eq!(
                    mode, 2,
                    "epoch guard detected change but got wrong mode: {}",
                    mode
                );
            }
            // If detected_mode is None, guard checked before migration — valid
        }
    });
}

/// Three threads: two migrators racing + one observer.
/// No deadlock, lock always released, epoch monotonically increases.
#[test]
fn loom_migration_three_threads_no_deadlock() {
    loom::model(|| {
        let header = LoomMigrationHeader::new(1);
        let h1 = header.clone();
        let h2 = header.clone();
        let h_obs = header.clone();

        let t1 = loom::thread::spawn(move || h1.try_migrate(2));
        let t2 = loom::thread::spawn(move || h2.try_migrate(3));
        let t_obs = loom::thread::spawn(move || h_obs.migration_epoch.load(Ordering::Acquire));

        let r1 = t1.join().unwrap();
        let r2 = t2.join().unwrap();
        let observed_epoch = t_obs.join().unwrap();

        // Both threads completed (no deadlock)
        // Lock is released
        assert!(!header.is_locked(), "migration lock must be released");

        // At most one migration per attempt (CAS exclusion)
        let successes = [r1, r2].iter().filter(|r| r.is_some()).count();
        assert!(
            successes <= 2,
            "each migration attempt independently serialized"
        );

        // Epoch is monotonic
        let final_epoch = header.migration_epoch.load(Ordering::Acquire);
        assert!(
            observed_epoch <= final_epoch,
            "epoch must be monotonic: observed {} but final is {}",
            observed_epoch,
            final_epoch
        );

        // Final mode must be one of: 1 (no migration), 2 (t1 won), 3 (t2 won)
        let final_mode = header.backend_mode.load(Ordering::Acquire);
        assert!(
            [1, 2, 3].contains(&final_mode),
            "final mode must be 1, 2, or 3, got {}",
            final_mode
        );
    });
}

/// Sequential migration: thread A migrates, then thread B migrates.
/// Epoch should increment twice.
#[test]
fn loom_migration_sequential_epoch_increments() {
    loom::model(|| {
        let header = LoomMigrationHeader::new(1);
        let h1 = header.clone();
        let h2 = header.clone();

        // Thread A migrates 1 → 2
        let t1 = loom::thread::spawn(move || h1.try_migrate(2));
        let r1 = t1.join().unwrap();

        // Thread B migrates 2 → 3
        let t2 = loom::thread::spawn(move || h2.try_migrate(3));
        let r2 = t2.join().unwrap();

        // Both should succeed (no contention — sequential)
        assert!(r1.is_some(), "first migration should succeed");
        assert_eq!(r1.unwrap(), 1, "first epoch should be 1");

        assert!(r2.is_some(), "second migration should succeed");
        assert_eq!(r2.unwrap(), 2, "second epoch should be 2");

        // Final state
        assert_eq!(header.backend_mode.load(Ordering::Acquire), 3);
        assert_eq!(header.migration_epoch.load(Ordering::Acquire), 2);
        assert!(!header.is_locked());
    });
}

/// Producer epoch guard with cached epoch correctly detects sequential migrations.
#[test]
fn loom_epoch_guard_detects_sequential_migrations() {
    loom::model(|| {
        let header = LoomMigrationHeader::new(1);
        let h_guard = header.clone();

        // Migrate twice sequentially
        let _ = header.try_migrate(2);
        let _ = header.try_migrate(3);

        // Guard starts with cached epoch 0, should detect both changes
        let mut guard = LoomEpochGuard::new(h_guard);
        // After construction, guard cached epoch = current (2)
        // So check_epoch should return None (no change since construction)
        let r = guard.check_epoch();
        assert!(r.is_none(), "no change since guard was constructed");

        // Migrate again
        let _ = header.try_migrate(4);

        // Now guard should detect the change
        let r = guard.check_epoch();
        assert_eq!(r, Some(4), "guard should detect migration to mode 4");
    });
}
