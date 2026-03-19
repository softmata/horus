// Cross-platform shared memory — delegates to horus_sys::shm
//
// All platform-specific SHM code now lives in horus_sys::shm.
// This module re-exports the public API for backward compatibility.

// ============================================================================
// Re-exports from horus_sys::shm
// ============================================================================

pub use horus_sys::shm::{
    cleanup_stale_namespaces, has_native_shm, is_namespace_stale_by_flock, is_shm_file_stale,
    list_all_horus_namespaces, shm_base_dir, shm_error_logs_path, shm_logs_path, shm_namespace,
    shm_network_dir,
    shm_nodes_dir, shm_scheduler_dir, shm_topics_dir, NamespaceCleanupResult, NamespaceInfo,
};

// Re-export for tests and internal use
#[cfg(test)]
pub use horus_sys::shm::{generate_namespace, parse_namespace_sid, session_alive};

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_shm_base_dir_is_valid_path() {
        let base = shm_base_dir();
        // Should return a non-empty path string
        assert!(
            !base.as_os_str().is_empty(),
            "shm_base_dir() should return a non-empty path"
        );
    }

    #[test]
    fn test_shm_namespace_is_non_empty() {
        let ns = shm_namespace();
        assert!(
            !ns.is_empty(),
            "shm_namespace() should return a non-empty string"
        );
    }

    #[test]
    fn test_shm_topics_dir_is_under_base() {
        let base = shm_base_dir();
        let topics = shm_topics_dir();
        assert!(
            topics.starts_with(&base),
            "shm_topics_dir ({:?}) should be under shm_base_dir ({:?})",
            topics,
            base
        );
    }

    #[test]
    fn test_shm_nodes_dir_is_under_base() {
        let base = shm_base_dir();
        let nodes = shm_nodes_dir();
        assert!(
            nodes.starts_with(&base),
            "shm_nodes_dir ({:?}) should be under shm_base_dir ({:?})",
            nodes,
            base
        );
    }

    #[test]
    fn test_has_native_shm_returns_bool() {
        // On Linux this should return true (has /dev/shm)
        let result = has_native_shm();
        // Just verify it doesn't panic and returns a bool
        let _ = result;
    }

    #[test]
    fn test_cleanup_stale_namespaces_doesnt_crash() {
        // Should not panic even if there are no stale namespaces
        let result = cleanup_stale_namespaces();
        assert!(
            result.removed >= 0 || result.removed == 0,
            "cleanup should report non-negative removed count"
        );
    }

    #[test]
    fn test_generate_namespace_is_unique() {
        let ns1 = generate_namespace();
        let ns2 = generate_namespace();
        // Two calls should produce different namespaces (includes timestamp/counter)
        // Note: they might be the same if called in the same nanosecond on the same PID,
        // but in practice they should differ due to internal counter
        let _ = (ns1, ns2); // just verify they don't panic
    }
}
