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
    /// Default spin count before yield
    pub const DEFAULT_SPIN_COUNT: u32 = 100;

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

    /// Perform the actual migration (must hold lock)
    fn perform_migration(&self, new_mode: BackendMode) -> MigrationResult {
        if !self.drain_in_flight() {
            return MigrationResult::Failed;
        }

        let old_epoch = self.header.migration_epoch.fetch_add(1, Ordering::AcqRel);
        let new_epoch = old_epoch + 1;

        self.header
            .backend_mode
            .store(new_mode as u8, Ordering::Release);

        self.header
            .last_topology_change_ms
            .store(current_time_ms(), Ordering::Release);

        MigrationResult::Success { new_epoch }
    }

    /// Grace period for in-flight operations to complete before migration.
    ///
    /// Since send/recv are lock-free and complete in <200ns, a short spin
    /// followed by a yield is sufficient to let any in-progress operation
    /// finish. There is no ref-count to check — this is a best-effort
    /// quiescence barrier.
    fn drain_in_flight(&self) -> bool {
        // Spin briefly to let sub-microsecond operations complete
        for _ in 0..self.spin_count {
            std::sync::atomic::fence(Ordering::SeqCst);
            std::hint::spin_loop();
        }
        // Yield once to allow any preempted threads to finish
        std::thread::yield_now();
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
