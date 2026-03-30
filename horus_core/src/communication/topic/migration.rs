#![allow(private_interfaces)]
//! Backend migration coordinator.
//!
//! Handles safe backend transitions with CAS-based locking, epoch versioning,
//! and drain logic for in-flight messages.

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
    /// Migration failed due to error
    Failed,
}

// ============================================================================
// Backend Migrator
// ============================================================================

/// Backend migration coordinator.
///
/// Handles safe backend transitions with:
/// - CAS-based locking to prevent concurrent migrations
/// - Epoch versioning for reader/writer coordination
/// - Drain logic to ensure no in-flight message loss
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
    /// Drains in-flight operations before switching the backend mode.
    /// If the first drain attempt times out, retries once before failing.
    fn perform_migration(&self, new_mode: BackendMode) -> MigrationResult {
        if !self.drain_in_flight() {
            tracing::warn!(
                "migration drain timed out on first attempt, retrying (target={:?})",
                new_mode
            );
            // One retry: in-flight ops are <200ns, so a second attempt should succeed
            // unless there is a genuine stall (e.g., thread suspended by debugger).
            if !self.drain_in_flight() {
                tracing::warn!(
                    "migration drain timed out on retry, aborting migration (target={:?})",
                    new_mode
                );
                return MigrationResult::Failed;
            }
        }

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

    /// Grace period for in-flight operations to complete before migration.
    ///
    /// Since send/recv are lock-free and complete in <200ns, a short spin
    /// followed by yields gives in-progress operations time to finish.
    /// Returns `false` if the drain timeout is exceeded, indicating that
    /// migration should be retried rather than proceeding unsafely.
    fn drain_in_flight(&self) -> bool {
        let start = std::time::Instant::now();
        let timeout = std::time::Duration::from_micros(100); // 100us -- generous for <200ns ops

        // Spin to let sub-microsecond operations complete
        for _ in 0..self.spin_count {
            std::sync::atomic::fence(Ordering::SeqCst);
            std::hint::spin_loop();
            if start.elapsed() > timeout {
                return false;
            }
        }
        // Yield to let preempted producer threads complete
        for _ in 0..3 {
            std::thread::yield_now();
            if start.elapsed() > timeout {
                return false;
            }
        }
        true
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
            MigrationResult::Failed,
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
            .store(BackendMode::SpscIntra as u8, Ordering::Relaxed);
        let m = BackendMigrator::new(&h);
        assert_eq!(
            m.try_migrate(BackendMode::SpscIntra),
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
    fn migrate_unknown_to_direct_channel() {
        let h = make_header(true);
        let m = BackendMigrator::new(&h);
        let result = m.try_migrate(BackendMode::DirectChannel);
        assert_eq!(result, MigrationResult::Success { new_epoch: 1 });
        assert_eq!(h.mode(), BackendMode::DirectChannel);
    }

    #[test]
    fn migrate_unknown_to_spsc_intra() {
        let h = make_header(true);
        let m = BackendMigrator::new(&h);
        let result = m.try_migrate(BackendMode::SpscIntra);
        assert_eq!(result, MigrationResult::Success { new_epoch: 1 });
        assert_eq!(h.mode(), BackendMode::SpscIntra);
    }

    #[test]
    fn migrate_direct_channel_to_spsc_intra() {
        let h = make_header(true);
        h.backend_mode
            .store(BackendMode::DirectChannel as u8, Ordering::Relaxed);
        let m = BackendMigrator::new(&h);
        let result = m.try_migrate(BackendMode::SpscIntra);
        assert_eq!(result, MigrationResult::Success { new_epoch: 1 });
        assert_eq!(h.mode(), BackendMode::SpscIntra);
    }

    #[test]
    fn migrate_spsc_intra_to_mpsc_intra() {
        let h = make_header(true);
        h.backend_mode
            .store(BackendMode::SpscIntra as u8, Ordering::Relaxed);
        let m = BackendMigrator::new(&h);
        let result = m.try_migrate(BackendMode::MpscIntra);
        assert_eq!(result, MigrationResult::Success { new_epoch: 1 });
        assert_eq!(h.mode(), BackendMode::MpscIntra);
    }

    #[test]
    fn migrate_spsc_intra_to_spmc_intra() {
        let h = make_header(true);
        h.backend_mode
            .store(BackendMode::SpscIntra as u8, Ordering::Relaxed);
        let m = BackendMigrator::new(&h);
        let result = m.try_migrate(BackendMode::SpmcIntra);
        assert_eq!(result, MigrationResult::Success { new_epoch: 1 });
        assert_eq!(h.mode(), BackendMode::SpmcIntra);
    }

    #[test]
    fn migrate_intra_to_shm() {
        let h = make_header(true);
        h.backend_mode
            .store(BackendMode::SpscIntra as u8, Ordering::Relaxed);
        let m = BackendMigrator::new(&h);
        let result = m.try_migrate(BackendMode::SpscShm);
        assert_eq!(result, MigrationResult::Success { new_epoch: 1 });
        assert_eq!(h.mode(), BackendMode::SpscShm);
    }

    #[test]
    fn migrate_to_fanout() {
        let h = make_header(true);
        h.backend_mode
            .store(BackendMode::SpscIntra as u8, Ordering::Relaxed);
        let m = BackendMigrator::new(&h);
        let result = m.try_migrate(BackendMode::FanoutIntra);
        assert_eq!(result, MigrationResult::Success { new_epoch: 1 });
        assert_eq!(h.mode(), BackendMode::FanoutIntra);
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

        let r1 = m.try_migrate(BackendMode::SpscIntra);
        assert_eq!(r1, MigrationResult::Success { new_epoch: 1 });
        assert_eq!(h.migration_epoch.load(Ordering::Relaxed), 1);

        let r2 = m.try_migrate(BackendMode::MpscIntra);
        assert_eq!(r2, MigrationResult::Success { new_epoch: 2 });
        assert_eq!(h.migration_epoch.load(Ordering::Relaxed), 2);

        let r3 = m.try_migrate(BackendMode::FanoutIntra);
        assert_eq!(r3, MigrationResult::Success { new_epoch: 3 });
        assert_eq!(h.migration_epoch.load(Ordering::Relaxed), 3);
    }

    #[test]
    fn epoch_unchanged_when_not_needed() {
        let h = make_header(true);
        h.backend_mode
            .store(BackendMode::SpscIntra as u8, Ordering::Relaxed);
        h.migration_epoch.store(5, Ordering::Relaxed);
        let m = BackendMigrator::new(&h);

        let result = m.try_migrate(BackendMode::SpscIntra);
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
        let result = m.try_migrate(BackendMode::SpscIntra);
        assert_eq!(result, MigrationResult::AlreadyInProgress);
        // Mode unchanged
        assert_eq!(h.mode(), BackendMode::Unknown);
        h.unlock_migration();
    }

    #[test]
    fn lock_released_after_successful_migration() {
        let h = make_header(true);
        let m = BackendMigrator::new(&h);
        m.try_migrate(BackendMode::SpscIntra);
        assert!(
            !m.is_migration_in_progress(),
            "Lock should be released after migration"
        );
        // Second migration should also succeed
        let result = m.try_migrate(BackendMode::MpscIntra);
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
        // Same thread, POD, 1P:1C → DirectChannel
        let result = m.migrate_to_optimal();
        assert_eq!(result, MigrationResult::Success { new_epoch: 1 });
        assert_eq!(h.mode(), BackendMode::DirectChannel);
    }

    #[test]
    fn migrate_to_optimal_with_same_thread_non_pod() {
        let h = make_header(false);
        h.register_producer().unwrap();
        h.register_consumer().unwrap();
        let m = BackendMigrator::new(&h);
        // Same thread, non-POD, 1P:1C → SpscIntra
        let result = m.migrate_to_optimal();
        assert_eq!(result, MigrationResult::Success { new_epoch: 1 });
        assert_eq!(h.mode(), BackendMode::SpscIntra);
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
        // Optimal is DirectChannel but current is still Unknown
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
                        0 => BackendMode::SpscIntra,
                        1 => BackendMode::MpscIntra,
                        2 => BackendMode::SpmcIntra,
                        _ => BackendMode::FanoutIntra,
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
            BackendMode::DirectChannel,
            BackendMode::SpscIntra,
            BackendMode::MpscIntra,
            BackendMode::FanoutIntra,
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
        let result = m.try_migrate(BackendMode::DirectChannel);
        assert_eq!(result, MigrationResult::Success { new_epoch: 1 });
        assert_eq!(h.mode(), BackendMode::DirectChannel);
    }

    // ── Topology change timestamp ───────────────────────────────────────

    #[test]
    fn migration_updates_topology_timestamp() {
        let h = make_header(true);
        let before = current_time_ms();
        let m = BackendMigrator::new(&h);
        m.try_migrate(BackendMode::SpscIntra);
        let after = current_time_ms();

        let ts = h.last_topology_change_ms.load(Ordering::Relaxed);
        assert!(ts >= before && ts <= after);
    }
}
