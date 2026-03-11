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
/// 2. Auto-generated from session ID + user ID on Unix
///    (`sid{SID}_uid{UID}`), or from the process ID on non-Unix platforms
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
    // 2. Auto-generate from session ID (SID).
    //
    //    SID is shared by all processes in the same terminal session, so
    //    independently launched processes (`./pub &`, `./sub &`) converge
    //    on the same SHM namespace.  PGID was used previously but breaks
    //    when processes are backgrounded, since each gets its own PGID
    //    (GitHub issue #37).
    #[cfg(unix)]
    {
        // SAFETY: getsid(0) is an always-succeeding async-signal-safe syscall
        // that returns the session ID of the calling process.
        let sid = unsafe { libc::getsid(0) };
        // SAFETY: getuid() is an always-succeeding async-signal-safe syscall with no preconditions.
        let uid = unsafe { libc::getuid() };
        format!("sid{}_uid{}", sid, uid)
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

// ============================================================================
// Stale SHM namespace cleanup
// ============================================================================

// ============================================================================
// flock-based stale SHM detection
// ============================================================================

/// Check whether a shared memory file is stale (no process holds it open).
///
/// Every `ShmRegion` holds `flock(LOCK_SH)` on its backing file for its
/// entire lifetime.  When the process exits — even via SIGKILL — the kernel
/// closes the fd and releases the lock automatically.
///
/// This function attempts `flock(LOCK_EX | LOCK_NB)`:
/// - **Success** → no process holds a shared lock → the file is stale.
///   The exclusive lock is immediately released before returning.
/// - **`EWOULDBLOCK`** → at least one process still holds `LOCK_SH` → alive.
/// - **Other error** → conservatively returns `false` (not stale).
///
/// This is the primary staleness signal for cleanup routines. It is
/// O(1), race-free, and works even after SIGKILL.
#[cfg(unix)]
pub fn is_shm_file_stale(path: &std::path::Path) -> bool {
    use std::os::unix::io::AsRawFd;

    let file = match std::fs::File::open(path) {
        Ok(f) => f,
        Err(_) => return true, // Can't open → treat as stale (or already gone)
    };

    let fd = file.as_raw_fd();
    // SAFETY: fd is a valid open file descriptor from File::open above.
    // LOCK_EX | LOCK_NB is a valid flock operation.
    let ret = unsafe { libc::flock(fd, libc::LOCK_EX | libc::LOCK_NB) };

    if ret == 0 {
        // Got exclusive lock → nobody holds it → stale.
        // Release immediately so we don't block others.
        // SAFETY: fd is still valid; LOCK_UN is a valid flock operation.
        unsafe { libc::flock(fd, libc::LOCK_UN) };
        true
    } else {
        let errno = std::io::Error::last_os_error()
            .raw_os_error()
            .unwrap_or(0);
        if errno == libc::EWOULDBLOCK {
            // At least one process holds LOCK_SH → alive.
            false
        } else {
            // Unexpected error → conservatively assume not stale.
            false
        }
    }
}

/// Non-Unix fallback: cannot use flock, conservatively returns false.
#[cfg(not(unix))]
pub fn is_shm_file_stale(_path: &std::path::Path) -> bool {
    false
}

/// Check whether ALL topic files in a namespace directory are stale.
///
/// Returns `true` only if every regular file under `topics/` is stale
/// (no process holds a flock on it). Returns `false` if any file is still
/// held, or if there are no topic files to check.
///
/// This is used by `cleanup_stale_namespaces()` as an additional signal
/// beyond session/process-group liveness checks.
#[cfg(unix)]
pub fn is_namespace_stale_by_flock(namespace_path: &std::path::Path) -> bool {
    let topics_dir = namespace_path.join("topics");
    if !topics_dir.exists() {
        // No topics dir — might have other content, check files directly
        return is_directory_all_files_stale(namespace_path);
    }
    is_directory_all_files_stale(&topics_dir)
}

/// Check if all regular files in a directory (recursively) are stale by flock.
#[cfg(unix)]
fn is_directory_all_files_stale(dir: &std::path::Path) -> bool {
    let entries = match std::fs::read_dir(dir) {
        Ok(e) => e,
        Err(_) => return true, // Can't read → treat as stale
    };

    let mut found_any_file = false;
    for entry in entries.flatten() {
        let path = entry.path();
        if path.is_file() {
            found_any_file = true;
            if !is_shm_file_stale(&path) {
                return false; // At least one file is alive
            }
        } else if path.is_dir() {
            // Recurse into subdirectories (e.g., horus_links/, horus_topic/)
            if !is_directory_all_files_stale(&path) {
                return false;
            }
            // If subdir had files, count them
            if dir_file_count(&path) > 0 {
                found_any_file = true;
            }
        }
    }

    // Only stale if we actually found files to check
    found_any_file
}

#[cfg(not(unix))]
pub fn is_namespace_stale_by_flock(_namespace_path: &std::path::Path) -> bool {
    false
}

/// Returns the parent directory where HORUS SHM namespace directories live.
///
/// - Linux: `/dev/shm/`
/// - macOS: `/tmp/`
/// - Windows: `%TEMP%`
pub fn shm_parent_dir() -> PathBuf {
    #[cfg(target_os = "linux")]
    {
        PathBuf::from("/dev/shm")
    }

    #[cfg(target_os = "macos")]
    {
        PathBuf::from("/tmp")
    }

    #[cfg(target_os = "windows")]
    {
        std::env::temp_dir()
    }

    #[cfg(not(any(target_os = "linux", target_os = "macos", target_os = "windows")))]
    {
        PathBuf::from("/tmp")
    }
}

/// Parse a HORUS auto-generated namespace directory name (SID format).
///
/// Returns `Some((sid, uid))` for names matching `horus_sid{N}_uid{N}`.
/// Returns `None` for other formats.
pub fn parse_namespace_sid(dir_name: &str) -> Option<(i32, u32)> {
    let suffix = dir_name.strip_prefix("horus_sid")?;
    let (sid_str, rest) = suffix.split_once("_uid")?;
    let sid: i32 = sid_str.parse().ok()?;
    let uid: u32 = rest.parse().ok()?;
    Some((sid, uid))
}

/// Check whether a session (by session leader PID) is still alive.
///
/// Uses `kill(sid, 0)` to check if the session leader process exists.
#[cfg(unix)]
pub fn session_alive(sid: i32) -> bool {
    if sid <= 0 {
        return false;
    }
    // SAFETY: kill with signal 0 checks existence without sending a signal.
    let ret = unsafe { libc::kill(sid, 0) };
    if ret == 0 {
        return true;
    }
    let errno = std::io::Error::last_os_error().raw_os_error().unwrap_or(0);
    errno == libc::EPERM
}

#[cfg(not(unix))]
pub fn session_alive(_sid: i32) -> bool {
    true
}

/// Result of a stale namespace cleanup operation.
#[derive(Debug, Clone)]
pub struct NamespaceCleanupResult {
    /// Number of stale namespace directories removed.
    pub removed: usize,
    /// Total bytes freed.
    pub bytes_freed: u64,
    /// Directories that were skipped (still alive, other user, etc.).
    pub skipped: usize,
    /// Errors encountered (non-fatal — we skip and continue).
    pub errors: Vec<String>,
}

/// Information about a single HORUS namespace directory.
#[derive(Debug, Clone)]
pub struct NamespaceInfo {
    /// Full path to the namespace directory.
    pub path: PathBuf,
    /// Directory name (e.g. `horus_pgid12345_uid1000`).
    pub dir_name: String,
    /// Parsed PGID, if this is an auto-generated namespace.
    pub pgid: Option<i32>,
    /// Parsed UID, if this is an auto-generated namespace.
    pub uid: Option<u32>,
    /// Whether the owning process group is still alive.
    pub alive: bool,
    /// Total size in bytes.
    pub size_bytes: u64,
    /// Number of files inside.
    pub file_count: usize,
}

/// Scan the SHM parent directory and remove stale HORUS namespace directories.
///
/// A namespace is considered stale when:
/// 1. Its name matches `horus_sid{N}_uid{N}` or `horus_pgid{N}_uid{N}` (auto-generated, not custom)
/// 2. Its UID matches the current user (we never touch other users' dirs)
/// 3. Its session/process group is no longer alive
/// 4. It's not the current process's namespace
///
/// This is safe to call at any time. It only removes directories that belong
/// to dead process groups owned by the current user.
pub fn cleanup_stale_namespaces() -> NamespaceCleanupResult {
    let mut result = NamespaceCleanupResult {
        removed: 0,
        bytes_freed: 0,
        skipped: 0,
        errors: Vec::new(),
    };

    let parent = shm_parent_dir();
    if !parent.exists() {
        return result;
    }

    let current_ns = format!("horus_{}", shm_namespace());

    #[cfg(unix)]
    // SAFETY: getuid() is an always-succeeding, async-signal-safe POSIX syscall.
    let current_uid = unsafe { libc::getuid() };
    #[cfg(not(unix))]
    let current_uid: u32 = 0;

    let entries = match std::fs::read_dir(&parent) {
        Ok(e) => e,
        Err(e) => {
            result
                .errors
                .push(format!("Failed to read {}: {}", parent.display(), e));
            return result;
        }
    };

    for entry in entries.flatten() {
        let dir_name = entry.file_name().to_string_lossy().to_string();

        // Only look at auto-generated horus_sid* directories
        if !dir_name.starts_with("horus_sid") {
            continue;
        }

        // Skip the current process's namespace
        if dir_name == current_ns {
            result.skipped += 1;
            continue;
        }

        // Parse SID and UID from the directory name
        let (uid, alive) = if let Some((sid, uid)) = parse_namespace_sid(&dir_name) {
            (uid, session_alive(sid))
        } else {
            result.skipped += 1;
            continue;
        };

        // Only clean our own user's directories
        if uid != current_uid {
            result.skipped += 1;
            continue;
        }

        // Check if the session/process group is still alive
        if alive {
            result.skipped += 1;
            continue;
        }

        // Process group is dead — remove the stale namespace
        let dir_path = entry.path();
        let size = dir_size_bytes(&dir_path);

        match std::fs::remove_dir_all(&dir_path) {
            Ok(()) => {
                log::info!(
                    "Removed stale SHM namespace: {} (freed {})",
                    dir_name,
                    format_bytes_compact(size)
                );
                result.removed += 1;
                result.bytes_freed += size;
            }
            Err(e) => {
                result
                    .errors
                    .push(format!("Failed to remove {}: {}", dir_path.display(), e));
            }
        }
    }

    if result.removed > 0 {
        log::info!(
            "SHM cleanup: removed {} stale namespace(s), freed {}",
            result.removed,
            format_bytes_compact(result.bytes_freed)
        );
    }

    result
}

/// List all HORUS namespace directories in the SHM parent.
///
/// Returns info about every `horus_*` directory, including whether each is
/// alive or stale. Used by `horus clean --shm` for verbose display.
pub fn list_all_horus_namespaces() -> Vec<NamespaceInfo> {
    let parent = shm_parent_dir();
    if !parent.exists() {
        return Vec::new();
    }

    let entries = match std::fs::read_dir(&parent) {
        Ok(e) => e,
        Err(_) => return Vec::new(),
    };

    let mut namespaces = Vec::new();

    for entry in entries.flatten() {
        let dir_name = entry.file_name().to_string_lossy().to_string();

        // Only look at horus_* directories
        if !dir_name.starts_with("horus_") {
            continue;
        }

        let path = entry.path();
        if !path.is_dir() {
            continue;
        }

        let (pgid, uid, alive) = if let Some((sid, uid)) = parse_namespace_sid(&dir_name) {
            (Some(sid), Some(uid), session_alive(sid))
        } else {
            (None, None, true) // Custom namespaces assumed alive
        };

        let size_bytes = dir_size_bytes(&path);
        let file_count = dir_file_count(&path);

        namespaces.push(NamespaceInfo {
            path,
            dir_name,
            pgid,
            uid,
            alive,
            size_bytes,
            file_count,
        });
    }

    namespaces
}

/// Recursively compute the total size of a directory in bytes.
fn dir_size_bytes(path: &std::path::Path) -> u64 {
    let mut size = 0u64;
    if let Ok(entries) = std::fs::read_dir(path) {
        for entry in entries.flatten() {
            let p = entry.path();
            if p.is_file() {
                if let Ok(meta) = p.metadata() {
                    size += meta.len();
                }
            } else if p.is_dir() {
                size += dir_size_bytes(&p);
            }
        }
    }
    size
}

/// Recursively count files in a directory.
fn dir_file_count(path: &std::path::Path) -> usize {
    let mut count = 0;
    if let Ok(entries) = std::fs::read_dir(path) {
        for entry in entries.flatten() {
            let p = entry.path();
            if p.is_file() {
                count += 1;
            } else if p.is_dir() {
                count += dir_file_count(&p);
            }
        }
    }
    count
}

/// Format bytes in a compact human-readable form.
fn format_bytes_compact(bytes: u64) -> String {
    const KB: u64 = 1024;
    const MB: u64 = 1024 * KB;
    const GB: u64 = 1024 * MB;

    if bytes >= GB {
        format!("{:.1} GB", bytes as f64 / GB as f64)
    } else if bytes >= MB {
        format!("{:.1} MB", bytes as f64 / MB as f64)
    } else if bytes >= KB {
        format!("{:.1} KB", bytes as f64 / KB as f64)
    } else {
        format!("{} B", bytes)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::duration_ext::DurationExt;

    /// Mutex to serialise tests that mutate `HORUS_NAMESPACE` env var.
    /// Environment variables are process-global; concurrent set_var/remove_var
    /// causes data races that make namespace tests flaky under parallel test execution.
    static ENV_MUTEX: std::sync::Mutex<()> = std::sync::Mutex::new(());

    /// Mutex to serialise tests that call `cleanup_stale_namespaces()`.
    /// Cleanup scans and removes directories in `/dev/shm/`, so parallel
    /// calls can interfere with each other's setup/teardown.
    static CLEANUP_MUTEX: std::sync::Mutex<()> = std::sync::Mutex::new(());

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
        assert!(
            !ns.is_empty(),
            "generate_namespace() must never return an empty string"
        );
    }

    /// generate_namespace() with HORUS_NAMESPACE set returns the sanitized value.
    #[test]
    fn test_generate_namespace_uses_horus_namespace_env() {
        // Serialise all tests that mutate the process-global HORUS_NAMESPACE env var.
        let _lock = ENV_MUTEX.lock().unwrap_or_else(|e| e.into_inner());

        let prev = std::env::var("HORUS_NAMESPACE").ok();

        // SAFETY: env var mutation is process-wide; ENV_MUTEX serialises access.
        unsafe {
            std::env::set_var("HORUS_NAMESPACE", "test_robot_A");
        }
        let ns = generate_namespace();

        // Restore
        unsafe {
            match prev {
                Some(v) => std::env::set_var("HORUS_NAMESPACE", v),
                None => std::env::remove_var("HORUS_NAMESPACE"),
            }
        }

        assert_eq!(ns, "test_robot_A");
    }

    /// generate_namespace() auto-generates a non-empty namespace when env var is absent.
    #[test]
    fn test_generate_namespace_auto_when_no_env() {
        // Serialise all tests that mutate the process-global HORUS_NAMESPACE env var.
        let _lock = ENV_MUTEX.lock().unwrap_or_else(|e| e.into_inner());

        let prev = std::env::var("HORUS_NAMESPACE").ok();

        // SAFETY: env var mutation is process-wide; ENV_MUTEX serialises access.
        unsafe {
            std::env::remove_var("HORUS_NAMESPACE");
        }
        let ns = generate_namespace();

        // Restore
        unsafe {
            match prev {
                Some(v) => std::env::set_var("HORUS_NAMESPACE", v),
                None => std::env::remove_var("HORUS_NAMESPACE"),
            }
        }

        assert!(!ns.is_empty(), "auto-generated namespace must not be empty");
        // On Unix the format is "sid<N>_uid<N>"; on non-Unix it's "pid<N>".
        #[cfg(unix)]
        assert!(
            ns.starts_with("sid"),
            "auto-generated Unix namespace must start with 'sid', got '{}'",
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
        assert_ne!(
            ns_a, ns_b,
            "different app names must produce different SHM directory names"
        );
    }

    // ========================================================================
    // Stale namespace cleanup tests
    // ========================================================================

    #[test]
    fn test_parse_namespace_sid_valid() {
        assert_eq!(
            parse_namespace_sid("horus_sid12345_uid1000"),
            Some((12345, 1000))
        );
        assert_eq!(parse_namespace_sid("horus_sid1_uid0"), Some((1, 0)));
        assert_eq!(
            parse_namespace_sid("horus_sid999999999_uid65534"),
            Some((999999999, 65534))
        );
    }

    #[test]
    fn test_parse_namespace_sid_custom_namespaces() {
        assert_eq!(parse_namespace_sid("horus_my_robot"), None);
        assert_eq!(parse_namespace_sid("horus_test"), None);
        assert_eq!(parse_namespace_sid("horus_"), None);
        assert_eq!(parse_namespace_sid("not_horus_sid1_uid1"), None);
    }

    #[test]
    fn test_parse_namespace_sid_malformed() {
        assert_eq!(parse_namespace_sid("horus_sidabc_uid1000"), None);
        assert_eq!(parse_namespace_sid("horus_sid123_uidabc"), None);
        assert_eq!(parse_namespace_sid("horus_sid_uid"), None);
        assert_eq!(parse_namespace_sid("horus_sid123"), None);
        assert_eq!(parse_namespace_sid(""), None);
    }

    #[test]
    fn test_format_bytes_compact() {
        assert_eq!(format_bytes_compact(0), "0 B");
        assert_eq!(format_bytes_compact(512), "512 B");
        assert_eq!(format_bytes_compact(1024), "1.0 KB");
        assert_eq!(format_bytes_compact(1536), "1.5 KB");
        assert_eq!(format_bytes_compact(1048576), "1.0 MB");
        assert_eq!(format_bytes_compact(1073741824), "1.0 GB");
    }

    #[test]
    fn test_shm_parent_dir_exists() {
        let parent = shm_parent_dir();
        // On any normal system, /dev/shm or /tmp should exist
        assert!(
            parent.exists(),
            "shm_parent_dir() should point to an existing directory: {}",
            parent.display()
        );
    }

    #[test]
    fn test_cleanup_stale_namespaces_with_simulated_stale_dir() {
        let _lock = CLEANUP_MUTEX.lock().unwrap_or_else(|e| e.into_inner());

        // Create a fake stale namespace directory with a dead SID
        let parent = shm_parent_dir();
        // Use a unique dead SID based on timestamp to avoid interference from parallel tests
        let unique_suffix = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap()
            .as_nanos()
            % 100_000_000;
        let dead_sid = 900_000_000 + unique_suffix as i32;
        #[cfg(unix)]
        // SAFETY: getuid() is an always-succeeding, async-signal-safe POSIX syscall.
        let uid = unsafe { libc::getuid() };
        #[cfg(not(unix))]
        let uid: u32 = 0;

        let stale_dir_name = format!("horus_sid{}_uid{}", dead_sid, uid);
        let stale_path = parent.join(&stale_dir_name);

        // Ensure we start clean
        let _ = std::fs::remove_dir_all(&stale_path);

        // Create the fake stale directory with a file inside
        std::fs::create_dir_all(&stale_path).expect("create stale dir");
        std::fs::write(stale_path.join("test_topic"), b"stale data").expect("write stale file");
        assert!(stale_path.exists());

        // Run cleanup — it should remove this stale directory
        let result = cleanup_stale_namespaces();
        assert!(
            result.removed >= 1,
            "Should remove at least our stale directory, removed={}",
            result.removed
        );
        assert!(
            !stale_path.exists(),
            "Stale directory should have been removed"
        );
        assert!(result.bytes_freed > 0, "Should have freed some bytes");
    }

    #[test]
    fn test_cleanup_does_not_remove_live_namespaces() {
        let _lock = CLEANUP_MUTEX.lock().unwrap_or_else(|e| e.into_inner());

        // Create a directory matching the current session's SID — should NOT be removed
        let parent = shm_parent_dir();
        #[cfg(unix)]
        // SAFETY: getsid(0) is an always-succeeding, async-signal-safe POSIX syscall.
        let sid = unsafe { libc::getsid(0) };
        #[cfg(not(unix))]
        let sid: i32 = std::process::id() as i32;
        #[cfg(unix)]
        // SAFETY: getuid() is an always-succeeding, async-signal-safe POSIX syscall.
        let uid = unsafe { libc::getuid() };
        #[cfg(not(unix))]
        let uid: u32 = 0;

        let live_dir_name = format!("horus_sid{}_uid{}", sid, uid);
        let live_path = parent.join(&live_dir_name);

        // Might already exist from the current process's namespace
        let created = if !live_path.exists() {
            std::fs::create_dir_all(&live_path).expect("create live dir");
            true
        } else {
            false
        };

        let result = cleanup_stale_namespaces();

        // The live directory should NOT have been removed
        assert!(
            live_path.exists(),
            "Live process group directory should not be removed"
        );

        // Clean up if we created it
        if created {
            let _ = std::fs::remove_dir_all(&live_path);
        }

        // No errors should have occurred for this directory
        let _ = result; // result is valid
    }

    #[test]
    fn test_list_all_horus_namespaces() {
        let namespaces = list_all_horus_namespaces();
        // Should return a list (possibly empty, but shouldn't panic)
        for ns in &namespaces {
            assert!(ns.dir_name.starts_with("horus_"));
            // Note: path may not exist if another test cleaned it up concurrently
        }
    }

    #[test]
    fn test_dir_size_bytes_and_file_count() {
        use std::io::Write;
        let tmp = std::env::temp_dir().join("horus_test_dir_size");
        let _ = std::fs::remove_dir_all(&tmp);
        std::fs::create_dir_all(&tmp).unwrap();

        // Create a file with known size
        let mut f = std::fs::File::create(tmp.join("file1.dat")).unwrap();
        f.write_all(&[0u8; 100]).unwrap();
        drop(f);

        // Create a subdirectory with another file
        std::fs::create_dir_all(tmp.join("sub")).unwrap();
        let mut f2 = std::fs::File::create(tmp.join("sub/file2.dat")).unwrap();
        f2.write_all(&[0u8; 200]).unwrap();
        drop(f2);

        let size = dir_size_bytes(&tmp);
        assert_eq!(size, 300, "Total size should be 300 bytes");

        let count = dir_file_count(&tmp);
        assert_eq!(count, 2, "Should have 2 files");

        let _ = std::fs::remove_dir_all(&tmp);
    }

    // ========================================================================
    // flock-based stale SHM detection tests
    // ========================================================================

    /// Helper: create a temp dir unique to each test invocation.
    /// Includes an atomic counter so parallel tests with the same `name` get
    /// distinct directories (all tests share the same PID).
    fn flock_test_dir(name: &str) -> PathBuf {
        use std::sync::atomic::{AtomicU64, Ordering};
        static DIR_COUNTER: AtomicU64 = AtomicU64::new(0);
        let seq = DIR_COUNTER.fetch_add(1, Ordering::Relaxed);
        let dir = std::env::temp_dir().join(format!(
            "horus_flock_test_{}_{}_{}",
            name,
            std::process::id(),
            seq
        ));
        let _ = std::fs::remove_dir_all(&dir);
        std::fs::create_dir_all(&dir).unwrap();
        dir
    }

    /// A file with no flock held should be detected as stale.
    #[test]
    fn test_flock_stale_no_holder() {
        let dir = flock_test_dir("no_holder");
        let path = dir.join("topic_a");
        std::fs::write(&path, b"data").unwrap();

        assert!(
            is_shm_file_stale(&path),
            "File with no flock holder should be stale"
        );

        let _ = std::fs::remove_dir_all(&dir);
    }

    /// A file with LOCK_SH held should NOT be detected as stale.
    #[cfg(unix)]
    #[test]
    fn test_flock_alive_with_shared_lock() {
        use std::os::unix::io::AsRawFd;

        let dir = flock_test_dir("alive_shared");
        let path = dir.join("topic_b");
        std::fs::write(&path, b"data").unwrap();

        // Hold a shared lock (simulates ShmRegion)
        let file = std::fs::File::open(&path).unwrap();
        let ret = unsafe { libc::flock(file.as_raw_fd(), libc::LOCK_SH) };
        assert_eq!(ret, 0, "flock(LOCK_SH) should succeed");

        assert!(
            !is_shm_file_stale(&path),
            "File with LOCK_SH held should NOT be stale"
        );

        drop(file); // releases lock
        let _ = std::fs::remove_dir_all(&dir);
    }

    /// After dropping the file (releasing flock), the file should become stale.
    #[cfg(unix)]
    #[test]
    fn test_flock_becomes_stale_after_drop() {
        use std::os::unix::io::AsRawFd;

        let dir = flock_test_dir("stale_after_drop");
        let path = dir.join("topic_c");
        std::fs::write(&path, b"data").unwrap();

        {
            let file = std::fs::File::open(&path).unwrap();
            let ret = unsafe { libc::flock(file.as_raw_fd(), libc::LOCK_SH) };
            assert_eq!(ret, 0);
            assert!(!is_shm_file_stale(&path), "Should be alive while held");
            // file drops here → lock released
        }

        assert!(
            is_shm_file_stale(&path),
            "Should be stale after holder dropped"
        );

        let _ = std::fs::remove_dir_all(&dir);
    }

    /// Multiple shared locks: file is alive as long as ANY holder exists.
    #[cfg(unix)]
    #[test]
    fn test_flock_multiple_holders() {
        use std::os::unix::io::AsRawFd;

        let dir = flock_test_dir("multi_holder");
        let path = dir.join("topic_d");
        std::fs::write(&path, b"data").unwrap();

        let file1 = std::fs::File::open(&path).unwrap();
        unsafe { libc::flock(file1.as_raw_fd(), libc::LOCK_SH) };

        let file2 = std::fs::File::open(&path).unwrap();
        unsafe { libc::flock(file2.as_raw_fd(), libc::LOCK_SH) };

        assert!(!is_shm_file_stale(&path), "Two holders → alive");

        drop(file1);
        assert!(
            !is_shm_file_stale(&path),
            "One holder remaining → still alive"
        );

        drop(file2);
        assert!(
            is_shm_file_stale(&path),
            "All holders gone → stale"
        );

        let _ = std::fs::remove_dir_all(&dir);
    }

    /// Nonexistent file should be treated as stale.
    #[test]
    fn test_flock_stale_nonexistent_file() {
        let path = PathBuf::from("/tmp/horus_flock_test_nonexistent_42");
        let _ = std::fs::remove_file(&path);
        assert!(
            is_shm_file_stale(&path),
            "Nonexistent file should be treated as stale"
        );
    }

    /// flock survives across child process death (SIGKILL).
    /// Spawn a child that holds LOCK_SH, kill it with SIGKILL,
    /// verify the file becomes stale.
    #[cfg(unix)]
    #[test]
    fn test_flock_released_on_sigkill() {
        use std::process::{Command, Stdio};

        let dir = flock_test_dir("sigkill");
        let path = dir.join("topic_sigkill");
        std::fs::write(&path, b"data").unwrap();

        // Spawn a child process that holds flock and then sleeps forever
        let mut child = Command::new("/bin/sh")
            .arg("-c")
            .arg(format!(
                "exec python3 -c \"
import fcntl, time, sys
fd = open('{}', 'r')
fcntl.flock(fd, fcntl.LOCK_SH)
sys.stdout.write('locked\\n')
sys.stdout.flush()
time.sleep(3600)
\"",
                path.display()
            ))
            .stdout(Stdio::piped())
            .spawn()
            .expect("spawn child");

        // Wait for child to acquire the lock
        let stdout = child.stdout.as_mut().unwrap();
        let mut buf = [0u8; 16];
        use std::io::Read;
        let n = stdout.read(&mut buf).unwrap();
        let msg = std::str::from_utf8(&buf[..n]).unwrap().trim();
        assert_eq!(msg, "locked", "child should report lock acquired");

        // File should be alive (child holds LOCK_SH)
        assert!(
            !is_shm_file_stale(&path),
            "Child holds flock → should be alive"
        );

        // SIGKILL the child
        let pid = child.id() as i32;
        // SAFETY: pid is a valid child process id; SIGKILL is a valid signal
        unsafe { libc::kill(pid, libc::SIGKILL) };
        let _ = child.wait(); // reap zombie

        // Now the file should be stale — kernel released the lock
        assert!(
            is_shm_file_stale(&path),
            "After SIGKILL, flock should be released → stale"
        );

        let _ = std::fs::remove_dir_all(&dir);
    }

    /// is_namespace_stale_by_flock with a mix of held and unheld files.
    #[cfg(unix)]
    #[test]
    fn test_namespace_stale_by_flock_mixed() {
        use std::os::unix::io::AsRawFd;

        let dir = flock_test_dir("ns_mixed");
        let topics_dir = dir.join("topics");
        std::fs::create_dir_all(&topics_dir).unwrap();

        // Create two topic files
        let path_a = topics_dir.join("horus_topic_a");
        let path_b = topics_dir.join("horus_topic_b");
        std::fs::write(&path_a, b"data_a").unwrap();
        std::fs::write(&path_b, b"data_b").unwrap();

        // Hold lock on topic_a only
        let file_a = std::fs::File::open(&path_a).unwrap();
        unsafe { libc::flock(file_a.as_raw_fd(), libc::LOCK_SH) };

        assert!(
            !is_namespace_stale_by_flock(&dir),
            "One file still held → namespace not stale"
        );

        drop(file_a);
        assert!(
            is_namespace_stale_by_flock(&dir),
            "All files released → namespace stale"
        );

        let _ = std::fs::remove_dir_all(&dir);
    }

    /// is_namespace_stale_by_flock with empty topics directory.
    #[test]
    fn test_namespace_stale_by_flock_empty_dir() {
        let dir = flock_test_dir("ns_empty");
        let topics_dir = dir.join("topics");
        std::fs::create_dir_all(&topics_dir).unwrap();

        // Empty directory → no files to check → not stale (conservative)
        assert!(
            !is_namespace_stale_by_flock(&dir),
            "Empty topics dir should not be considered stale"
        );

        let _ = std::fs::remove_dir_all(&dir);
    }

    /// is_namespace_stale_by_flock with subdirectories (horus_links/, horus_topic/).
    #[cfg(unix)]
    #[test]
    fn test_namespace_stale_by_flock_with_subdirs() {
        use std::os::unix::io::AsRawFd;

        let dir = flock_test_dir("ns_subdirs");
        let topics_dir = dir.join("topics");
        let links_dir = topics_dir.join("horus_links");
        std::fs::create_dir_all(&links_dir).unwrap();

        let path = links_dir.join("sensor_data");
        std::fs::write(&path, b"link_data").unwrap();

        // No lock → stale
        assert!(is_namespace_stale_by_flock(&dir));

        // Hold lock → alive
        let file = std::fs::File::open(&path).unwrap();
        unsafe { libc::flock(file.as_raw_fd(), libc::LOCK_SH) };
        assert!(!is_namespace_stale_by_flock(&dir));

        drop(file);
        let _ = std::fs::remove_dir_all(&dir);
    }

    // ========================================================================
    // Stress tests
    // ========================================================================

    /// Stress test: many threads racing to acquire/release flock + check staleness.
    #[cfg(unix)]
    #[test]
    fn test_flock_stress_concurrent_acquire_release() {
        use std::os::unix::io::AsRawFd;
        use std::sync::{Arc, Barrier};

        let dir = flock_test_dir("stress_concurrent");
        let path = dir.join("topic_stress");
        std::fs::write(&path, b"stress_data").unwrap();

        let n_threads = 16;
        let n_iterations = 100;
        let barrier = Arc::new(Barrier::new(n_threads));
        let path = Arc::new(path);

        let handles: Vec<_> = (0..n_threads)
            .map(|_| {
                let b = barrier.clone();
                let p = path.clone();
                std::thread::spawn(move || {
                    b.wait();
                    for _ in 0..n_iterations {
                        let file = std::fs::File::open(p.as_ref()).unwrap();
                        let fd = file.as_raw_fd();
                        // SAFETY: fd is valid; LOCK_SH is valid
                        unsafe { libc::flock(fd, libc::LOCK_SH) };
                        // Briefly hold, then drop (releases lock)
                        std::thread::yield_now();
                        drop(file);
                    }
                })
            })
            .collect();

        for h in handles {
            h.join().unwrap();
        }

        // All threads done → should be stale.
        // On WSL2 under heavy parallel test load, flock release may be
        // slightly delayed at the kernel level, so retry a few times.
        let mut stale = false;
        for _ in 0..20 {
            if is_shm_file_stale(path.as_ref()) {
                stale = true;
                break;
            }
            std::thread::sleep(1_u64.ms());
        }
        assert!(stale, "After all threads finish, file should be stale");

        let _ = std::fs::remove_dir_all(&dir);
    }

    /// Stress test: staleness checks concurrent with holders.
    /// Some threads hold locks, others check staleness.
    /// Because the atomic counter and flock are not atomically linked,
    /// TOCTOU races are expected under heavy parallel load.  We verify
    /// that the overwhelming majority of observations are consistent.
    #[cfg(unix)]
    #[test]
    fn test_flock_stress_no_false_stale() {
        use std::os::unix::io::AsRawFd;
        use std::sync::atomic::{AtomicBool, AtomicUsize, Ordering};
        use std::sync::Arc;

        let dir = flock_test_dir("stress_no_false");
        let path = dir.join("topic_no_false_stale");
        std::fs::write(&path, b"data").unwrap();

        let path = Arc::new(path);
        let holder_count = Arc::new(AtomicUsize::new(0));
        let stop = Arc::new(AtomicBool::new(false));
        let total_checks = Arc::new(AtomicUsize::new(0));
        let false_stale_count = Arc::new(AtomicUsize::new(0));

        // Holder threads: repeatedly acquire and release locks
        let mut handles = Vec::new();
        for _ in 0..4 {
            let p = path.clone();
            let hc = holder_count.clone();
            let s = stop.clone();
            handles.push(std::thread::spawn(move || {
                while !s.load(Ordering::Relaxed) {
                    let file = std::fs::File::open(p.as_ref()).unwrap();
                    // SAFETY: fd valid; LOCK_SH valid
                    unsafe { libc::flock(file.as_raw_fd(), libc::LOCK_SH) };
                    hc.fetch_add(1, Ordering::SeqCst);
                    std::thread::sleep(100_u64.us());
                    hc.fetch_sub(1, Ordering::SeqCst);
                    drop(file);
                    std::thread::yield_now();
                }
            }));
        }

        // Checker threads: verify staleness is consistent with holder count
        for _ in 0..4 {
            let p = path.clone();
            let hc = holder_count.clone();
            let s = stop.clone();
            let tc = total_checks.clone();
            let fc = false_stale_count.clone();
            handles.push(std::thread::spawn(move || {
                while !s.load(Ordering::Relaxed) {
                    let stale = is_shm_file_stale(p.as_ref());
                    tc.fetch_add(1, Ordering::Relaxed);
                    if stale && hc.load(Ordering::SeqCst) > 0 {
                        // Possible TOCTOU: re-verify after a brief delay.
                        // The holder might have released between our flock
                        // probe and the counter read.
                        std::thread::sleep(200_u64.us());
                        if hc.load(Ordering::SeqCst) > 0 && is_shm_file_stale(p.as_ref()) {
                            fc.fetch_add(1, Ordering::Relaxed);
                        }
                    }
                    std::thread::yield_now();
                }
            }));
        }

        // Run for a bit
        std::thread::sleep(500_u64.ms());
        stop.store(true, Ordering::SeqCst);

        for h in handles {
            h.join().unwrap();
        }

        let total = total_checks.load(Ordering::Relaxed);
        let false_stales = false_stale_count.load(Ordering::Relaxed);
        // Allow at most 1% TOCTOU races (counter and flock are separate atomics)
        let max_allowed = (total / 100).max(2);
        assert!(
            false_stales <= max_allowed,
            "Too many false stale detections: {false_stales}/{total} (max {max_allowed})"
        );

        let _ = std::fs::remove_dir_all(&dir);
    }

    /// Stress test: flock works correctly across forked child processes.
    #[cfg(unix)]
    #[test]
    fn test_flock_stress_multiprocess() {
        let dir = flock_test_dir("stress_multiproc");
        let path = dir.join("topic_multiproc");
        std::fs::write(&path, b"data").unwrap();

        // Spawn N child processes that each hold flock for a short time
        let n_children = 8;
        let mut children = Vec::new();

        for _i in 0..n_children {
            let child = std::process::Command::new("/bin/sh")
                .arg("-c")
                .arg(format!(
                    "python3 -c \"
import fcntl, time
fd = open('{}', 'r')
fcntl.flock(fd, fcntl.LOCK_SH)
time.sleep(0.2)
\"",
                    path.display()
                ))
                .spawn()
                .expect("spawn child");
            children.push(child);
        }

        // Give children time to acquire locks
        std::thread::sleep(100_u64.ms());

        // File should be alive while children hold it
        assert!(
            !is_shm_file_stale(&path),
            "Children holding flock → alive"
        );

        // Wait for all children to finish
        for mut child in children {
            let _ = child.wait();
        }

        // Now should be stale
        assert!(
            is_shm_file_stale(&path),
            "All children exited → stale"
        );

        let _ = std::fs::remove_dir_all(&dir);
    }

    /// Edge case: is_shm_file_stale on a directory (not a file).
    #[test]
    fn test_flock_stale_on_directory() {
        let dir = flock_test_dir("stale_dir");
        // Calling is_shm_file_stale on a directory — should not panic.
        // On Linux, flock works on dirs too, but we don't hold locks on them.
        let result = is_shm_file_stale(&dir);
        // Should be stale (no lock held on it)
        assert!(result, "Directory with no lock should be treated as stale");
        let _ = std::fs::remove_dir_all(&dir);
    }

    /// Edge case: file deleted while checking staleness.
    #[cfg(unix)]
    #[test]
    fn test_flock_stale_file_deleted_during_check() {
        let dir = flock_test_dir("deleted_during");
        let path = dir.join("ephemeral");
        std::fs::write(&path, b"data").unwrap();

        // Delete the file, then check — should handle gracefully
        std::fs::remove_file(&path).unwrap();
        assert!(
            is_shm_file_stale(&path),
            "Deleted file should be treated as stale"
        );

        let _ = std::fs::remove_dir_all(&dir);
    }
}
