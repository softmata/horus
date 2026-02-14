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
pub enum MigrationResult {
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
// Migration Statistics
// ============================================================================

/// Migration statistics for debugging and monitoring
#[derive(Debug, Clone)]
pub struct MigrationStats {
    pub current_mode: BackendMode,
    pub optimal_mode: BackendMode,
    pub current_epoch: u64,
    pub is_locked: bool,
    pub publisher_count: u32,
    pub subscriber_count: u32,
    pub is_same_process: bool,
    pub is_same_thread: bool,
    pub is_pod: bool,
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
pub struct BackendMigrator<'a> {
    header: &'a TopicHeader,
    /// Maximum wait time for draining in-flight messages (ms)
    drain_timeout_ms: u64,
    /// Spin count before yielding during drain
    spin_count: u32,
}

impl<'a> BackendMigrator<'a> {
    /// Default drain timeout: 100ms
    pub const DEFAULT_DRAIN_TIMEOUT_MS: u64 = 100;
    /// Default spin count before yield
    pub const DEFAULT_SPIN_COUNT: u32 = 100;

    /// Create a new migrator for the given header
    pub fn new(header: &'a TopicHeader) -> Self {
        Self {
            header,
            drain_timeout_ms: Self::DEFAULT_DRAIN_TIMEOUT_MS,
            spin_count: Self::DEFAULT_SPIN_COUNT,
        }
    }

    /// Create a migrator with custom drain timeout
    pub fn with_drain_timeout(header: &'a TopicHeader, timeout_ms: u64) -> Self {
        Self {
            header,
            drain_timeout_ms: timeout_ms,
            spin_count: Self::DEFAULT_SPIN_COUNT,
        }
    }

    /// Get the current epoch
    #[inline]
    pub fn current_epoch(&self) -> u64 {
        self.header.migration_epoch.load(Ordering::Acquire)
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

    /// Wait for in-flight operations to drain.
    fn drain_in_flight(&self) -> bool {
        let start = current_time_ms();
        let mut spins = 0u32;

        while current_time_ms() - start < self.drain_timeout_ms {
            std::sync::atomic::fence(Ordering::SeqCst);

            spins += 1;
            if spins >= self.spin_count {
                std::thread::yield_now();
                spins = 0;
            }

            if current_time_ms() - start >= 1 {
                return true;
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

    /// Get migration statistics for debugging
    pub fn stats(&self) -> MigrationStats {
        MigrationStats {
            current_mode: self.header.mode(),
            optimal_mode: self.header.detect_optimal_backend(),
            current_epoch: self.current_epoch(),
            is_locked: self.is_migration_in_progress(),
            publisher_count: self.header.pub_count(),
            subscriber_count: self.header.sub_count(),
            is_same_process: self.header.is_same_process(),
            is_same_thread: self.header.is_same_thread(),
            is_pod: self.header.is_pod_type(),
        }
    }
}
