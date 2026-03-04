//! Shared test utilities for horus_core integration tests

/// Clean up shared memory files before each test.
///
/// Stale `/dev/shm/horus/` files from previous tests can cause SIGSEGV
/// when mapped with incompatible layouts. Each test must start with a
/// clean SHM state to avoid cross-test interference.
pub fn cleanup_stale_shm() {
    let _ = std::fs::remove_dir_all("/dev/shm/horus/topics");
    let _ = std::fs::remove_dir_all("/dev/shm/horus/nodes");
}
