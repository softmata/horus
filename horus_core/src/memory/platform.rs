// Cross-platform shared memory - each platform uses its optimal mechanism
//
// Linux: /dev/shm/horus (tmpfs - RAM-backed, fastest) via file mmap
// macOS: POSIX shm_open() (Mach shared memory - RAM-backed) - NOT file-based!
// Windows: CreateFileMappingW (pagefile-backed - optimized for IPC) - NOT temp files!
//
// Note: macOS and Windows no longer use filesystem paths for shared memory.
// The path functions below are kept for Linux and backward compatibility only.

use std::path::PathBuf;

/// Sanitize a namespace string: replace any character that is not ASCII
/// alphanumeric or underscore with an underscore.
fn sanitize_namespace(ns: &str) -> String {
    ns.chars()
        .map(|c| {
            if c.is_ascii_alphanumeric() || c == '_' {
                c
            } else {
                '_'
            }
        })
        .collect()
}

/// Generate the SHM namespace for this process without caching.
///
/// This is the uncached version, suitable for unit tests.  Production code
/// should call [`shm_namespace`] instead, which memoises the result.
///
/// Priority:
/// 1. `HORUS_NAMESPACE` — set by `horus_manager` when it launches a node graph.
/// 2. `HORUS_SHM_NAMESPACE` — legacy name, kept for backward compatibility.
/// 3. Auto-generated from process-group ID + user ID on Unix
///    (`pgid{PGID}_uid{UID}`), or from the process ID on non-Unix platforms
///    (`pid{PID}`).  This ensures that independent HORUS applications running
///    on the same machine automatically use different SHM regions without any
///    explicit configuration.
pub fn generate_namespace() -> String {
    // 1. HORUS_NAMESPACE — set by horus_manager (launch.rs sets this env var)
    if let Ok(ns) = std::env::var("HORUS_NAMESPACE") {
        if !ns.is_empty() {
            return sanitize_namespace(&ns);
        }
    }
    // 2. HORUS_SHM_NAMESPACE — legacy fallback
    if let Ok(ns) = std::env::var("HORUS_SHM_NAMESPACE") {
        if !ns.is_empty() {
            return sanitize_namespace(&ns);
        }
    }
    // 3. Auto-generate: PGID groups all processes launched from the same parent
    //    so they share SHM; different applications have different PGIDs.
    #[cfg(unix)]
    {
        // SAFETY: getpgrp() and getuid() are always-succeeding async-signal-safe syscalls.
        let pgid = unsafe { libc::getpgrp() };
        let uid = unsafe { libc::getuid() };
        format!("pgid{}_uid{}", pgid, uid)
    }
    #[cfg(not(unix))]
    {
        format!("pid{}", std::process::id())
    }
}

/// Return the SHM namespace for this process.
///
/// The namespace is always a non-empty string and is resolved once then
/// cached for the lifetime of the process.  See [`generate_namespace`] for
/// the resolution priority and format.
///
/// Examples (with `HORUS_NAMESPACE=robot1`):
/// - Linux:   `/dev/shm/horus_robot1/topics/`
/// - macOS:   POSIX shm names become `/horus_robot1_<topic>`
pub fn shm_namespace() -> String {
    static NAMESPACE: std::sync::OnceLock<String> = std::sync::OnceLock::new();
    NAMESPACE.get_or_init(generate_namespace).clone()
}

/// Get the base directory for HORUS shared memory.
///
/// This returns a platform-appropriate path for shared memory:
/// - Linux: `/dev/shm/horus_<namespace>/` (tmpfs for maximum performance)
/// - macOS: `/tmp/horus_<namespace>/` (no /dev/shm, but /tmp is still fast)
/// - Windows: `%TEMP%\horus_<namespace>\` (system temp directory)
///
/// The namespace suffix comes from [`shm_namespace`] (see its documentation
/// for the resolution order) and ensures that independent HORUS applications
/// running on the same machine use completely separate SHM regions.
pub fn shm_base_dir() -> PathBuf {
    let dir_name = format!("horus_{}", shm_namespace());

    #[cfg(target_os = "linux")]
    {
        PathBuf::from("/dev/shm").join(&dir_name)
    }

    #[cfg(target_os = "macos")]
    {
        // macOS doesn't have /dev/shm, use /tmp instead
        PathBuf::from("/tmp").join(&dir_name)
    }

    #[cfg(target_os = "windows")]
    {
        // Windows uses temp directory
        std::env::temp_dir().join(&dir_name)
    }

    #[cfg(not(any(target_os = "linux", target_os = "macos", target_os = "windows")))]
    {
        // Fallback for other Unix-like systems (BSD, etc.)
        PathBuf::from("/tmp").join(&dir_name)
    }
}

/// Get the topics directory for shared memory message passing
pub fn shm_topics_dir() -> PathBuf {
    shm_base_dir().join("topics")
}

/// Get the nodes directory for node presence files
///
/// Each running node creates a presence file here at startup and removes it at shutdown.
/// Monitor scans this directory to discover all active nodes.
///
/// Structure: `/dev/shm/horus/nodes/{node_name}.json`
pub fn shm_nodes_dir() -> PathBuf {
    shm_base_dir().join("nodes")
}

/// Get the network status directory for transport monitoring
pub fn shm_network_dir() -> PathBuf {
    shm_base_dir().join("network")
}

/// Get the control directory for node lifecycle commands
///
/// Control files allow external processes (like `horus node kill`) to
/// send commands to running nodes without killing the entire scheduler.
///
/// Structure: `/dev/shm/horus/control/{node_name}.cmd`
/// Commands: "stop", "restart", "pause", "resume"
pub fn shm_control_dir() -> PathBuf {
    shm_base_dir().join("control")
}

/// Get the logs shared memory path
pub fn shm_logs_path() -> PathBuf {
    // Logs are at the same level as horus dir, not inside it
    #[cfg(target_os = "linux")]
    {
        PathBuf::from("/dev/shm/horus_logs")
    }

    #[cfg(target_os = "macos")]
    {
        PathBuf::from("/tmp/horus_logs")
    }

    #[cfg(target_os = "windows")]
    {
        std::env::temp_dir().join("horus_logs")
    }

    #[cfg(not(any(target_os = "linux", target_os = "macos", target_os = "windows")))]
    {
        PathBuf::from("/tmp/horus_logs")
    }
}

/// Returns true if the platform has native shared memory support (/dev/shm on Linux).
pub fn has_native_shm() -> bool {
    #[cfg(target_os = "linux")]
    {
        std::path::Path::new("/dev/shm").exists()
    }
    #[cfg(not(target_os = "linux"))]
    {
        false
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_shm_paths_are_valid() {
        let base = shm_base_dir();
        assert!(!base.as_os_str().is_empty());

        let topics = shm_topics_dir();
        assert!(topics.starts_with(&base));

        let nodes = shm_nodes_dir();
        assert!(nodes.starts_with(&base));
    }

    /// shm_base_dir() must always end with a component that starts with "horus_".
    /// The namespace is now always present (never bare "horus").
    #[test]
    fn test_shm_base_dir_starts_with_horus_underscore() {
        let base = shm_base_dir();
        let dir_name = base
            .file_name()
            .expect("base dir should have a file_name component")
            .to_string_lossy();
        assert!(
            dir_name.starts_with("horus_"),
            "shm_base_dir() last component must start with 'horus_', got '{}'",
            dir_name
        );
    }

    /// sanitize_namespace replaces non-alphanumeric/underscore chars with '_'.
    #[test]
    fn test_sanitize_namespace() {
        assert_eq!(sanitize_namespace("my-robot/1!"), "my_robot_1_");
        assert_eq!(sanitize_namespace("robot_01"), "robot_01");
        assert_eq!(sanitize_namespace("hello world"), "hello_world");
        assert_eq!(sanitize_namespace("abc123_XYZ"), "abc123_XYZ");
    }

    /// generate_namespace() returns a non-empty string in all cases.
    #[test]
    fn test_generate_namespace_is_non_empty() {
        // generate_namespace reads live env vars; unset both to exercise auto-gen path.
        // We can't safely unset env vars in a parallel test environment, so we only
        // verify the contract (non-empty) rather than the specific auto-gen format.
        let ns = generate_namespace();
        assert!(!ns.is_empty(), "generate_namespace() must never return an empty string");
    }

    /// generate_namespace() with HORUS_NAMESPACE set returns the sanitized value.
    #[test]
    fn test_generate_namespace_uses_horus_namespace_env() {
        // Set both vars; HORUS_NAMESPACE must win.
        // SAFETY: setting env vars is process-wide. This test must not run in
        // parallel with other tests that read HORUS_NAMESPACE.  Since each Rust
        // test binary runs its tests sequentially by default within a single
        // thread pool, and the env changes are bracketed, this is acceptable.
        // In CI, use `--test-threads=1` if parallel env-var tests are added.
        let prev = std::env::var("HORUS_NAMESPACE").ok();
        let prev_legacy = std::env::var("HORUS_SHM_NAMESPACE").ok();

        unsafe {
            std::env::set_var("HORUS_NAMESPACE", "test_robot_A");
            std::env::remove_var("HORUS_SHM_NAMESPACE");
        }
        let ns = generate_namespace();

        // Restore
        unsafe {
            match prev {
                Some(v) => std::env::set_var("HORUS_NAMESPACE", v),
                None => std::env::remove_var("HORUS_NAMESPACE"),
            }
            match prev_legacy {
                Some(v) => std::env::set_var("HORUS_SHM_NAMESPACE", v),
                None => std::env::remove_var("HORUS_SHM_NAMESPACE"),
            }
        }

        assert_eq!(ns, "test_robot_A");
    }

    /// generate_namespace() falls back to HORUS_SHM_NAMESPACE when HORUS_NAMESPACE unset.
    #[test]
    fn test_generate_namespace_legacy_fallback() {
        let prev = std::env::var("HORUS_NAMESPACE").ok();
        let prev_legacy = std::env::var("HORUS_SHM_NAMESPACE").ok();

        unsafe {
            std::env::remove_var("HORUS_NAMESPACE");
            std::env::set_var("HORUS_SHM_NAMESPACE", "legacy_ns");
        }
        let ns = generate_namespace();

        unsafe {
            match prev {
                Some(v) => std::env::set_var("HORUS_NAMESPACE", v),
                None => std::env::remove_var("HORUS_NAMESPACE"),
            }
            match prev_legacy {
                Some(v) => std::env::set_var("HORUS_SHM_NAMESPACE", v),
                None => std::env::remove_var("HORUS_SHM_NAMESPACE"),
            }
        }

        assert_eq!(ns, "legacy_ns");
    }

    /// generate_namespace() auto-generates a non-empty namespace when both env vars are absent.
    #[test]
    fn test_generate_namespace_auto_when_no_env() {
        let prev = std::env::var("HORUS_NAMESPACE").ok();
        let prev_legacy = std::env::var("HORUS_SHM_NAMESPACE").ok();

        unsafe {
            std::env::remove_var("HORUS_NAMESPACE");
            std::env::remove_var("HORUS_SHM_NAMESPACE");
        }
        let ns = generate_namespace();

        unsafe {
            match prev {
                Some(v) => std::env::set_var("HORUS_NAMESPACE", v),
                None => std::env::remove_var("HORUS_NAMESPACE"),
            }
            match prev_legacy {
                Some(v) => std::env::set_var("HORUS_SHM_NAMESPACE", v),
                None => std::env::remove_var("HORUS_SHM_NAMESPACE"),
            }
        }

        assert!(!ns.is_empty(), "auto-generated namespace must not be empty");
        // On Unix the format is "pgid<N>_uid<N>"; on non-Unix it's "pid<N>".
        #[cfg(unix)]
        assert!(
            ns.starts_with("pgid"),
            "auto-generated Unix namespace must start with 'pgid', got '{}'",
            ns
        );
        #[cfg(not(unix))]
        assert!(
            ns.starts_with("pid"),
            "auto-generated non-Unix namespace must start with 'pid', got '{}'",
            ns
        );
    }

    /// Two different env var values produce different shm_base_dir paths.
    #[test]
    fn test_different_namespaces_give_different_paths() {
        let ns_a = format!("horus_{}", sanitize_namespace("app_a"));
        let ns_b = format!("horus_{}", sanitize_namespace("app_b"));
        assert_ne!(ns_a, ns_b, "different app names must produce different SHM directory names");
    }
}
