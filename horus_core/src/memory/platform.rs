// Cross-platform shared memory — delegates to horus_sys::shm
//
// All platform-specific SHM code now lives in horus_sys::shm.
// This module re-exports the public API for backward compatibility.

// ============================================================================
// Re-exports from horus_sys::shm
// ============================================================================

pub use horus_sys::shm::{
    cleanup_stale_namespaces, has_native_shm, is_namespace_stale_by_flock, is_shm_file_stale,
    list_all_horus_namespaces, shm_base_dir, shm_logs_path, shm_namespace,
    shm_network_dir, shm_nodes_dir, shm_scheduler_dir, shm_topics_dir,
    NamespaceCleanupResult, NamespaceInfo,
};

// Re-export for tests and internal use
#[cfg(test)]
pub use horus_sys::shm::{generate_namespace, parse_namespace_sid, session_alive};
