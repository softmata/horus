//! Shared test utilities for horus_core integration tests

/// Clean up stale shared memory files from previous test runs.
///
/// Stale `/dev/shm/horus/` files from previous test runs can cause SIGSEGV
/// when mapped with incompatible layouts. This function removes them once
/// per test process.
pub fn cleanup_stale_shm() {
    static ONCE: std::sync::Once = std::sync::Once::new();
    ONCE.call_once(|| {
        let _ = std::fs::remove_dir_all("/dev/shm/horus/topics");
        let _ = std::fs::remove_dir_all("/dev/shm/horus/nodes");
    });
}
