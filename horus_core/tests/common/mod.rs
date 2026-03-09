//! Shared test utilities for horus_core integration tests
//!
//! # Test Isolation Requirements
//!
//! HORUS tests share global state through shared memory (`/dev/shm/horus/`),
//! the `OnceLock`-based `REGISTRY`, and `EPOCH_NOTIFY`. To avoid interference
//! when running tests in parallel (`--test-threads=N`):
//!
//! 1. **Always use unique topic/pool names.** Call [`unique()`] or the
//!    unit-test helper `unique(prefix)` (in `topic/tests.rs`) to generate
//!    names that include the PID and a monotonic counter. Never hardcode
//!    topic names unless the test specifically validates the `topics!` macro.
//!
//! 2. **Create Topic handles on the main thread** when testing cross-thread
//!    send/recv. This ensures the adaptive backend migration (DirectChannel
//!    → SpscIntra) completes before threads start transferring data. Creating
//!    topics inside `thread::spawn` closures can race with migration.
//!
//! 3. **Use unique pool IDs** for `TensorPool` tests (e.g., 9600–9699) to
//!    avoid SHM collisions with other test pools.
//!
//! 4. **Call `cleanup_stale_shm()`** at the start of integration tests that
//!    create SHM files, to remove leftover state from previous runs.
//!
//! 5. **Do not depend on test execution order.** Tests must be independent
//!    and pass when run individually or in any order with any thread count.

/// Clean up shared memory files before each test.
///
/// Stale `/dev/shm/horus/` files from previous tests can cause SIGSEGV
/// when mapped with incompatible layouts. Each test must start with a
/// clean SHM state to avoid cross-test interference.
pub fn cleanup_stale_shm() {
    let _ = std::fs::remove_dir_all("/dev/shm/horus/topics");
    let _ = std::fs::remove_dir_all("/dev/shm/horus/nodes");
}

/// Generate a unique name for integration tests.
///
/// Includes the PID and a monotonic counter to avoid collisions between
/// parallel test threads and concurrent test processes.
#[allow(dead_code)]
pub fn unique(prefix: &str) -> String {
    use std::sync::atomic::{AtomicU64, Ordering};
    static COUNTER: AtomicU64 = AtomicU64::new(0);
    format!(
        "{}_{}_{}",
        prefix,
        std::process::id(),
        COUNTER.fetch_add(1, Ordering::Relaxed)
    )
}

/// RAII guard for temporary directories used in tests.
///
/// Creates a unique temporary directory on construction and removes it
/// (including all contents) on drop — even if the test panics. This
/// prevents leftover test artifacts from accumulating in `/tmp`.
///
/// # Example
/// ```ignore
/// let tmp = TestTempDir::new("my_test");
/// // use tmp.path() for file operations
/// // directory is automatically cleaned up when `tmp` goes out of scope
/// ```
#[allow(dead_code)]
pub struct TestTempDir {
    path: std::path::PathBuf,
}

#[allow(dead_code)]
impl TestTempDir {
    /// Create a new temporary directory with the given prefix.
    ///
    /// The directory name includes the PID and a counter to avoid collisions.
    pub fn new(prefix: &str) -> Self {
        let path = std::env::temp_dir().join(unique(prefix));
        let _ = std::fs::remove_dir_all(&path); // clean stale data
        std::fs::create_dir_all(&path).expect("failed to create test temp dir");
        Self { path }
    }

    /// Return the path to the temporary directory.
    pub fn path(&self) -> &std::path::Path {
        &self.path
    }
}

impl Drop for TestTempDir {
    fn drop(&mut self) {
        let _ = std::fs::remove_dir_all(&self.path);
    }
}

/// RAII guard that cleans up HORUS shared memory on drop.
///
/// Removes `/dev/shm/horus/topics` and `/dev/shm/horus/nodes` when the
/// guard is dropped, ensuring no stale SHM state leaks between tests.
/// Also cleans on construction so the test starts with a fresh state.
///
/// # Example
/// ```ignore
/// let _shm = ShmCleanupGuard::new();
/// // SHM is cleaned on creation and again on drop (even on panic)
/// ```
#[allow(dead_code)]
pub struct ShmCleanupGuard;

#[allow(dead_code)]
impl ShmCleanupGuard {
    pub fn new() -> Self {
        cleanup_stale_shm();
        Self
    }
}

impl Drop for ShmCleanupGuard {
    fn drop(&mut self) {
        cleanup_stale_shm();
    }
}
