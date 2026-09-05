#![allow(private_interfaces)]
//! Backend migration coordinator.
//!
//! Handles backend transitions with CAS-based locking, epoch versioning, and a
//! bounded settle before the switch.
//!
//! NOT a drain. This header said "drain logic for in-flight messages" while the
//! struct doc below already retracted exactly that claim: nothing in
//! `TopicHeader` counts in-flight operations, so there is nothing to drain
//! against, and a message in flight across a switch can be lost. Rustdoc shows
//! this block first, so the retraction 35 lines down was the half nobody read.

use std::sync::atomic::Ordering;

use super::header::{current_time_ms, TopicHeader};
use super::types::BackendMode;

// ============================================================================
// Migration Result
// ============================================================================

/// Result of a migration attempt
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum MigrationResult {
    /// Migration successful, new epoch returned
    Success { new_epoch: u64 },
    /// No migration needed (already at optimal backend)
    NotNeeded,
    /// Another participant is currently migrating
    AlreadyInProgress,
    /// Migration lock acquisition failed (contention)
    LockContention,
}

// ============================================================================
// Backend Migrator
// ============================================================================

/// Backend migration coordinator.
///
/// Handles backend transitions with:
/// - CAS-based locking to prevent concurrent migrations
/// - Epoch versioning for reader/writer coordination
/// - A bounded settle before the switch
///
/// It does NOT guarantee that no message is lost across a switch, and this
/// used to say it did ("Drain logic to ensure no in-flight message loss").
/// Nothing in `TopicHeader` counts in-flight operations, so there is nothing
/// to drain against — see [`Self::settle_before_switch`]. A real guarantee
/// needs an in-flight counter incremented and decremented on the send and
/// recv hot paths of a real-time framework, which is a deliberate design
/// decision with a measurable cost. Until that is taken, the honest statement
/// is that a topology change switches backends under a lock and a message in
/// flight across the switch can be lost. The topic's own test suite already
/// works around it, pre-warming backends "to avoid migration losses".
pub(crate) struct BackendMigrator<'a> {
    header: &'a TopicHeader,
    /// Spin count before yielding during drain
    spin_count: u32,
}

impl<'a> BackendMigrator<'a> {
    /// Default spin count before yield.
    ///
    /// 1000 spins ≈ 1-10μs depending on CPU, covering typical OS preemption
    /// windows where a producer could be suspended mid-write.
    pub const DEFAULT_SPIN_COUNT: u32 = 1000;

    /// Create a new migrator for the given header
    pub fn new(header: &'a TopicHeader) -> Self {
        Self {
            header,
            spin_count: Self::DEFAULT_SPIN_COUNT,
        }
    }

    /// Check if migration is currently in progress
    #[inline]
    pub fn is_migration_in_progress(&self) -> bool {
        self.header.migration_lock.load(Ordering::Acquire) != 0
    }

    /// Attempt to migrate to a new backend mode.
    pub fn try_migrate(&self, new_mode: BackendMode) -> MigrationResult {
        let current_mode = self.header.mode();

        if current_mode == new_mode {
            return MigrationResult::NotNeeded;
        }

        if self.is_migration_in_progress() {
            return MigrationResult::AlreadyInProgress;
        }

        if !self.header.try_lock_migration() {
            return MigrationResult::LockContention;
        }

        let result = self.perform_migration(new_mode);

        self.header.unlock_migration();

        result
    }

    /// Perform the actual migration (must hold lock).
    ///
    /// Settles briefly, then switches the backend mode.
    ///
    /// This used to say it "drains in-flight operations before switching" and
    /// "retries once before failing". Both halves guarded a return value that
    /// no longer exists: `settle_before_switch` cannot observe an in-flight
    /// operation and so cannot report one, which made the retry and the
    /// `Failed` result unreachable code describing a guarantee nothing made.
    fn perform_migration(&self, new_mode: BackendMode) -> MigrationResult {
        self.settle_before_switch();

        // IMPORTANT: Store backend_mode BEFORE incrementing epoch.
        // The epoch increment is the "publication fence" — observers that
        // see the new epoch (via Acquire) are guaranteed to also see the
        // new mode, because the mode store (Release) happens-before the
        // epoch fetch_add (AcqRel), which happens-before the observer's
        // epoch load (Acquire).
        //
        // Previously, epoch was incremented first, creating a window where
        // an observer could see the new epoch but read the old mode.
        // Discovered via loom exhaustive interleaving testing.
        self.header
            .backend_mode
            .store(new_mode as u8, Ordering::Release);

        self.header
            .last_topology_change_ms
            .store(current_time_ms(), Ordering::Release);

        let old_epoch = self.header.migration_epoch.fetch_add(1, Ordering::AcqRel);
        let new_epoch = old_epoch + 1;

        MigrationResult::Success { new_epoch }
    }

    /// Bounded settle before the backend mode is switched.
    ///
    /// NOT a drain, despite what this was called. Send and recv are lock-free
    /// and complete in under 200 ns, so a bounded spin plus a few yields gives
    /// an in-progress operation a good chance to finish — but "a good chance"
    /// is the whole of it. Nothing here can confirm one finished, and nothing
    /// here can fail.
    fn settle_before_switch(&self) {
        // Give any in-flight send/recv time to finish. They complete in under
        // 200 ns, so a bounded spin plus a few yields is the whole mechanism.
        //
        // There is no wall-clock abort here, and there used to be: 25 ms, with a
        // comment saying it existed to bound "GENUINELY-stuck producers (e.g. a
        // thread suspended by a debugger)".
        //
        // It could not do that. Nothing in `TopicHeader` counts in-flight
        // operations — there is no such field — so this function has never had
        // anything to observe. A wedged producer and an idle topic run the same
        // number of iterations and both return `true`. The deadline could
        // therefore only fire when *this loop* was preempted, which made the
        // single observable effect of the timeout "abort migrations when the
        // machine is busy":
        //
        //     assertion `left == right` failed
        //       left: Failed
        //      right: Success { new_epoch: 3 }
        //
        // Full parallel runs failed migration tests that way, never the same one
        // twice. Removing the abort removes a failure mode and no protection,
        // because the protection was never there.
        //
        // How much it helps is not established. A first comparison looked
        // decisive — 5 of 8 runs with migration failures before, 0 of 8 after —
        // but those batches ran hours apart under different machine load. Re-run
        // back-to-back, the baseline showed 1 of 8 and the fix 0 of 8, which
        // decides nothing. The argument for this change is the code, not the
        // measurement: a function with no in-flight counter cannot detect an
        // in-flight operation, so a deadline on its own spin loop can only
        // report on the scheduler.
        //
        // Restoring a real bound means counting in-flight operations, which is
        // an increment and a decrement on the send and recv hot paths of a
        // real-time framework. That is a deliberate design decision with a
        // measurable cost, not something to reintroduce by leaving a deadline
        // here that only looks like it does the job.
        // The deadline is still here, and still bounds how long this spins —
        // that part was always legitimate, and removing it made
        // `send_blocking` block measurably longer under load (a test asserting
        // an immediate return went from failing 2 runs in 8 to 7 in 8). What is
        // gone is the `return false` it used to trigger: stopping early is
        // right, reporting failure because of it is not.
        let start = std::time::Instant::now();
        let budget = std::time::Duration::from_millis(25);

        for _ in 0..self.spin_count {
            std::sync::atomic::fence(Ordering::SeqCst);
            std::hint::spin_loop();
            if start.elapsed() > budget {
                return;
            }
        }
        // Yield to let preempted producer threads complete.
        for _ in 0..3 {
            std::thread::yield_now();
            if start.elapsed() > budget {
                return;
            }
        }
    }

    /// Force a migration to the detected optimal backend.
    pub fn migrate_to_optimal(&self) -> MigrationResult {
        let optimal = self.header.detect_optimal_backend();
        self.try_migrate(optimal)
    }

    /// Check if the current backend is optimal for the current topology.
    pub fn is_optimal(&self) -> bool {
        let current = self.header.mode();
        let optimal = self.header.detect_optimal_backend();
        current == optimal
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The settle is bounded, and that bound is the only thing it guarantees.
    ///
    /// It cannot observe an in-flight operation — nothing in `TopicHeader`
    /// counts them — so it can neither confirm a drain nor report a failure.
    /// What it can do is not spin forever, and that part matters: removing the
    /// budget made `send_blocking` block measurably longer under load (a test
    /// asserting an immediate return went from failing 2 runs in 8 to 7 in 8).
    #[test]
    fn the_settle_is_bounded() {
        let header = make_header(true);
        let migrator = BackendMigrator::new(&header);

        let started = std::time::Instant::now();
        migrator.settle_before_switch();
        let elapsed = started.elapsed();

        assert!(
            elapsed < std::time::Duration::from_millis(250),
            "the settle must stay bounded; it took {elapsed:?}"
        );
    }

    /// A migration cannot report failure, so nothing may branch on one.
    ///
    /// `perform_migration` used to retry a drain and return
    /// `MigrationResult::Failed` when it "timed out" — guarding a return value
    /// `drain_in_flight` had stopped producing, so both the retry and the
    /// result were unreachable code standing for a guarantee nothing made.
    /// Callers wrote handlers for a case that could not occur.
    #[test]
    fn an_uncontended_migration_always_completes() {
        let header = make_header(true);
        header.publisher_count.store(1, Ordering::Release);
        header.subscriber_count.store(2, Ordering::Release);
        let migrator = BackendMigrator::new(&header);

        // Single-threaded, so there is nothing to contend with.
        match migrator.migrate_to_optimal() {
            MigrationResult::Success { .. } | MigrationResult::NotNeeded => {}
            other => panic!("an uncontended migration reported {other:?}"),
        }
    }
    use std::sync::atomic::Ordering;

    fn make_header(is_pod: bool) -> TopicHeader {
        let mut h = TopicHeader::zeroed();
        h.init(8, 8, is_pod, 16, 16, "TestType", 0);
        h
    }

    // ── MigrationResult ─────────────────────────────────────────────────

    #[test]
    fn migration_result_variants_are_distinct() {
        let results = [
            MigrationResult::Success { new_epoch: 1 },
            MigrationResult::NotNeeded,
            MigrationResult::AlreadyInProgress,
            MigrationResult::LockContention,
        ];
        for (i, a) in results.iter().enumerate() {
            for (j, b) in results.iter().enumerate() {
                if i == j {
                    assert_eq!(a, b);
                } else {
                    assert_ne!(a, b);
                }
            }
        }
    }

    #[test]
    fn migration_result_debug_format() {
        let r = MigrationResult::Success { new_epoch: 42 };
        let dbg = format!("{:?}", r);
        assert!(dbg.contains("Success"));
        assert!(dbg.contains("42"));
    }

    // ── BackendMigrator construction ────────────────────────────────────

    #[test]
    fn migrator_default_spin_count() {
        let h = make_header(true);
        let m = BackendMigrator::new(&h);
        assert_eq!(m.spin_count, BackendMigrator::DEFAULT_SPIN_COUNT);
    }

    #[test]
    fn migrator_not_in_progress_initially() {
        let h = make_header(true);
        let m = BackendMigrator::new(&h);
        assert!(!m.is_migration_in_progress());
    }

    // ── try_migrate: same mode → NotNeeded ──────────────────────────────

    #[test]
    fn migrate_to_same_mode_returns_not_needed() {
        let h = make_header(true);
        h.backend_mode
            .store(BackendMode::SpscShm as u8, Ordering::Relaxed);
        let m = BackendMigrator::new(&h);
        assert_eq!(
            m.try_migrate(BackendMode::SpscShm),
            MigrationResult::NotNeeded
        );
    }

    #[test]
    fn migrate_unknown_to_unknown_is_not_needed() {
        let h = make_header(true);
        // After init, mode is Unknown
        let m = BackendMigrator::new(&h);
        assert_eq!(
            m.try_migrate(BackendMode::Unknown),
            MigrationResult::NotNeeded
        );
    }

    // ── try_migrate: valid transitions ──────────────────────────────────

    #[test]
    fn migrate_unknown_to_pod_shm() {
        let h = make_header(true);
        let m = BackendMigrator::new(&h);
        let result = m.try_migrate(BackendMode::PodShm);
        assert_eq!(result, MigrationResult::Success { new_epoch: 1 });
        assert_eq!(h.mode(), BackendMode::PodShm);
    }

    #[test]
    fn migrate_unknown_to_spsc_shm() {
        let h = make_header(true);
        let m = BackendMigrator::new(&h);
        let result = m.try_migrate(BackendMode::SpscShm);
        assert_eq!(result, MigrationResult::Success { new_epoch: 1 });
        assert_eq!(h.mode(), BackendMode::SpscShm);
    }

    #[test]
    fn migrate_pod_shm_to_spsc_shm() {
        let h = make_header(true);
        h.backend_mode
            .store(BackendMode::PodShm as u8, Ordering::Relaxed);
        let m = BackendMigrator::new(&h);
        let result = m.try_migrate(BackendMode::SpscShm);
        assert_eq!(result, MigrationResult::Success { new_epoch: 1 });
        assert_eq!(h.mode(), BackendMode::SpscShm);
    }

    #[test]
    fn migrate_spsc_shm_to_mpsc_shm() {
        let h = make_header(true);
        h.backend_mode
            .store(BackendMode::SpscShm as u8, Ordering::Relaxed);
        let m = BackendMigrator::new(&h);
        let result = m.try_migrate(BackendMode::MpscShm);
        assert_eq!(result, MigrationResult::Success { new_epoch: 1 });
        assert_eq!(h.mode(), BackendMode::MpscShm);
    }

    #[test]
    fn migrate_spsc_shm_to_spmc_shm() {
        let h = make_header(true);
        h.backend_mode
            .store(BackendMode::SpscShm as u8, Ordering::Relaxed);
        let m = BackendMigrator::new(&h);
        let result = m.try_migrate(BackendMode::SpmcShm);
        assert_eq!(result, MigrationResult::Success { new_epoch: 1 });
        assert_eq!(h.mode(), BackendMode::SpmcShm);
    }

    #[test]
    fn migrate_across_shm_modes() {
        let h = make_header(true);
        h.backend_mode
            .store(BackendMode::MpscShm as u8, Ordering::Relaxed);
        let m = BackendMigrator::new(&h);
        let result = m.try_migrate(BackendMode::SpscShm);
        assert_eq!(result, MigrationResult::Success { new_epoch: 1 });
        assert_eq!(h.mode(), BackendMode::SpscShm);
    }

    #[test]
    fn migrate_to_fanout() {
        let h = make_header(true);
        h.backend_mode
            .store(BackendMode::SpscShm as u8, Ordering::Relaxed);
        let m = BackendMigrator::new(&h);
        let result = m.try_migrate(BackendMode::FanoutShm);
        assert_eq!(result, MigrationResult::Success { new_epoch: 1 });
        assert_eq!(h.mode(), BackendMode::FanoutShm);
    }

    #[test]
    fn migrate_to_all_shm_variants() {
        let shm_modes = [
            BackendMode::PodShm,
            BackendMode::MpscShm,
            BackendMode::SpmcShm,
            BackendMode::SpscShm,
            BackendMode::FanoutShm,
        ];
        for target in shm_modes {
            let h = make_header(true);
            let m = BackendMigrator::new(&h);
            let result = m.try_migrate(target);
            assert_eq!(result, MigrationResult::Success { new_epoch: 1 });
            assert_eq!(h.mode(), target);
        }
    }

    // ── Epoch tracking ──────────────────────────────────────────────────

    #[test]
    fn epoch_increments_on_each_migration() {
        let h = make_header(true);
        let m = BackendMigrator::new(&h);

        let r1 = m.try_migrate(BackendMode::SpscShm);
        assert_eq!(r1, MigrationResult::Success { new_epoch: 1 });
        assert_eq!(h.migration_epoch.load(Ordering::Relaxed), 1);

        let r2 = m.try_migrate(BackendMode::MpscShm);
        assert_eq!(r2, MigrationResult::Success { new_epoch: 2 });
        assert_eq!(h.migration_epoch.load(Ordering::Relaxed), 2);

        let r3 = m.try_migrate(BackendMode::FanoutShm);
        assert_eq!(r3, MigrationResult::Success { new_epoch: 3 });
        assert_eq!(h.migration_epoch.load(Ordering::Relaxed), 3);
    }

    #[test]
    fn epoch_unchanged_when_not_needed() {
        let h = make_header(true);
        h.backend_mode
            .store(BackendMode::SpscShm as u8, Ordering::Relaxed);
        h.migration_epoch.store(5, Ordering::Relaxed);
        let m = BackendMigrator::new(&h);

        let result = m.try_migrate(BackendMode::SpscShm);
        assert_eq!(result, MigrationResult::NotNeeded);
        assert_eq!(h.migration_epoch.load(Ordering::Relaxed), 5);
    }

    // ── Lock contention ─────────────────────────────────────────────────

    #[test]
    fn migrate_returns_already_in_progress_when_locked() {
        let h = make_header(true);
        // Pre-lock the migration
        assert!(h.try_lock_migration());
        let m = BackendMigrator::new(&h);
        let result = m.try_migrate(BackendMode::SpscShm);
        assert_eq!(result, MigrationResult::AlreadyInProgress);
        // Mode unchanged
        assert_eq!(h.mode(), BackendMode::Unknown);
        h.unlock_migration();
    }

    #[test]
    fn lock_released_after_successful_migration() {
        let h = make_header(true);
        let m = BackendMigrator::new(&h);
        m.try_migrate(BackendMode::SpscShm);
        assert!(
            !m.is_migration_in_progress(),
            "Lock should be released after migration"
        );
        // Second migration should also succeed
        let result = m.try_migrate(BackendMode::MpscShm);
        assert_eq!(result, MigrationResult::Success { new_epoch: 2 });
    }

    // ── migrate_to_optimal ──────────────────────────────────────────────

    #[test]
    fn migrate_to_optimal_no_participants() {
        let h = make_header(true);
        let m = BackendMigrator::new(&h);
        // No participants → optimal is Unknown → same as current → NotNeeded
        let result = m.migrate_to_optimal();
        assert_eq!(result, MigrationResult::NotNeeded);
    }

    #[test]
    fn migrate_to_optimal_with_same_thread_pod_producer_consumer() {
        let h = make_header(true);
        h.register_producer().unwrap();
        h.register_consumer().unwrap();
        let m = BackendMigrator::new(&h);
        // POD, 1P:1C → SpscShm (every topic is SHM-backed)
        let result = m.migrate_to_optimal();
        assert_eq!(result, MigrationResult::Success { new_epoch: 1 });
        assert_eq!(h.mode(), BackendMode::SpscShm);
    }

    #[test]
    fn migrate_to_optimal_with_same_thread_non_pod() {
        let h = make_header(false);
        h.register_producer().unwrap();
        h.register_consumer().unwrap();
        let m = BackendMigrator::new(&h);
        // Same thread, non-POD, 1P:1C → SpscShm (real topics are shm_backed
        // regardless of pod-ness; SHM supports serde via send_shm_sp_serde).
        let result = m.migrate_to_optimal();
        assert_eq!(result, MigrationResult::Success { new_epoch: 1 });
        assert_eq!(h.mode(), BackendMode::SpscShm);
    }

    // ── is_optimal ──────────────────────────────────────────────────────

    #[test]
    fn is_optimal_true_when_matching() {
        let h = make_header(true);
        // No participants → optimal = Unknown = current
        let m = BackendMigrator::new(&h);
        assert!(m.is_optimal());
    }

    #[test]
    fn is_optimal_false_after_topology_change() {
        let h = make_header(true);
        h.register_producer().unwrap();
        h.register_consumer().unwrap();
        // Optimal is SpscShm but current is still Unknown
        let m = BackendMigrator::new(&h);
        assert!(!m.is_optimal());
    }

    #[test]
    fn is_optimal_true_after_migration() {
        let h = make_header(true);
        h.register_producer().unwrap();
        h.register_consumer().unwrap();
        let m = BackendMigrator::new(&h);
        m.migrate_to_optimal();
        assert!(m.is_optimal());
    }

    // ── Concurrent migration ────────────────────────────────────────────

    #[test]
    fn concurrent_migrations_only_one_succeeds() {
        use std::sync::{Arc, Barrier};

        let h = make_header(true);
        let header_ptr = &h as *const TopicHeader as usize;
        let barrier = Arc::new(Barrier::new(4));

        let handles: Vec<_> = (0..4)
            .map(|i| {
                let barrier = barrier.clone();
                std::thread::spawn(move || {
                    // SAFETY: `header_ptr` was derived from a stack-local `TopicHeader`
                    // that outlives all spawned threads (they are joined below).
                    // The header's fields are atomic, so concurrent access is safe.
                    let h = unsafe { &*(header_ptr as *const TopicHeader) };
                    let m = BackendMigrator::new(h);
                    barrier.wait();
                    // All threads try to migrate at once
                    let target = match i {
                        0 => BackendMode::SpscShm,
                        1 => BackendMode::MpscShm,
                        2 => BackendMode::SpmcShm,
                        _ => BackendMode::FanoutShm,
                    };
                    m.try_migrate(target)
                })
            })
            .collect();

        let results: Vec<_> = handles.into_iter().map(|h| h.join().unwrap()).collect();

        let successes = results
            .iter()
            .filter(|r| matches!(r, MigrationResult::Success { .. }))
            .count();
        let contentions = results
            .iter()
            .filter(|r| {
                matches!(
                    r,
                    MigrationResult::AlreadyInProgress | MigrationResult::LockContention
                )
            })
            .count();

        // At least one should succeed, rest should see contention
        assert!(successes >= 1, "At least one migration should succeed");
        assert!(
            successes + contentions == 4,
            "All attempts should either succeed or hit contention, got {:?}",
            results
        );
        // Epoch should match the number of successes
        assert_eq!(h.migration_epoch.load(Ordering::Relaxed), successes as u64);
    }

    // ── Sequential migrations ───────────────────────────────────────────

    #[test]
    fn sequential_migration_chain() {
        let h = make_header(true);
        let m = BackendMigrator::new(&h);

        let chain = [
            BackendMode::PodShm,
            BackendMode::SpscShm,
            BackendMode::MpscShm,
            BackendMode::FanoutShm,
            BackendMode::SpscShm,
            BackendMode::FanoutShm,
        ];

        for (i, &target) in chain.iter().enumerate() {
            let result = m.try_migrate(target);
            assert_eq!(
                result,
                MigrationResult::Success {
                    new_epoch: (i as u64) + 1
                }
            );
            assert_eq!(h.mode(), target);
        }
        assert_eq!(
            h.migration_epoch.load(Ordering::Relaxed),
            chain.len() as u64
        );
    }

    #[test]
    fn downgrade_migration_allowed() {
        // Migration from a "higher" mode back to a "lower" one should work
        let h = make_header(true);
        h.backend_mode
            .store(BackendMode::FanoutShm as u8, Ordering::Relaxed);
        let m = BackendMigrator::new(&h);
        let result = m.try_migrate(BackendMode::PodShm);
        assert_eq!(result, MigrationResult::Success { new_epoch: 1 });
        assert_eq!(h.mode(), BackendMode::PodShm);
    }

    // ── Topology change timestamp ───────────────────────────────────────

    #[test]
    fn migration_updates_topology_timestamp() {
        let h = make_header(true);
        let before = current_time_ms();
        let m = BackendMigrator::new(&h);
        m.try_migrate(BackendMode::SpscShm);
        let after = current_time_ms();

        let ts = h.last_topology_change_ms.load(Ordering::Relaxed);
        assert!(ts >= before && ts <= after);
    }
}
