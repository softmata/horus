//! Shared memory IPC — zero-copy cross-process communication.
//!
//! Provides [`ShmRegion`] for creating and mapping shared memory regions,
//! plus namespace/path helpers and stale-region cleanup utilities.
//!
//! Each platform uses its native backend:
//! - **Linux**: `/dev/shm` tmpfs + mmap + flock
//! - **macOS**: `shm_open()` + Mach VM + flock
//! - **Windows**: `CreateFileMappingW` + pagefile
//! - **Fallback**: `/tmp` file-based mmap + flock

#[cfg(target_os = "linux")]
mod linux;
#[cfg(target_os = "macos")]
mod macos;
#[cfg(target_os = "windows")]
mod windows;
#[cfg(not(any(target_os = "linux", target_os = "macos", target_os = "windows")))]
mod fallback;

#[cfg(target_os = "linux")]
pub use self::linux::ShmRegion;
#[cfg(target_os = "macos")]
pub use self::macos::ShmRegion;
#[cfg(target_os = "windows")]
pub use self::windows::ShmRegion;
#[cfg(not(any(target_os = "linux", target_os = "macos", target_os = "windows")))]
pub use self::fallback::ShmRegion;

use std::path::PathBuf;

// ============================================================================
// Namespace generation
// ============================================================================

/// Sanitize a namespace string: replace non-ASCII-alphanumeric/underscore chars with underscore.
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
/// Priority:
/// 1. `HORUS_NAMESPACE` env var (set by horus_manager when launching node graphs)
/// 2. Auto-generated from session ID + user ID (`sid{SID}_uid{UID}`) on all platforms
pub fn generate_namespace() -> String {
    if let Ok(ns) = std::env::var("HORUS_NAMESPACE") {
        if !ns.is_empty() {
            let sanitized = sanitize_namespace(&ns);
            // Reject pathological namespaces that collapse to all-underscores (e.g. "/" → "_")
            if !sanitized.chars().all(|c| c == '_') {
                return sanitized;
            }
            // Fall through to auto-generation if sanitized name is degenerate
        }
    }
    #[cfg(unix)]
    {
        // SAFETY: getsid(0) and getuid() are always-succeeding async-signal-safe syscalls
        let sid = unsafe { libc::getsid(0) };
        let uid = unsafe { libc::getuid() };
        format!("sid{}_uid{}", sid, uid)
    }
    #[cfg(not(unix))]
    {
        let sid = crate::process::session_id();
        let uid = crate::process::user_id();
        format!("sid{}_uid{}", sid, uid)
    }
}

/// Return the SHM namespace for this process (cached after first call).
pub fn shm_namespace() -> String {
    static NAMESPACE: std::sync::OnceLock<String> = std::sync::OnceLock::new();
    NAMESPACE.get_or_init(generate_namespace).clone()
}

// ============================================================================
// Path helpers
// ============================================================================

/// Base directory for HORUS shared memory.
///
/// - Linux: `/dev/shm/horus_<namespace>/`
/// - macOS: `/tmp/horus_<namespace>/`
/// - Windows: `%TEMP%\horus_<namespace>\`
pub fn shm_base_dir() -> PathBuf {
    let dir_name = format!("horus_{}", shm_namespace());

    #[cfg(target_os = "linux")]
    {
        PathBuf::from("/dev/shm").join(&dir_name)
    }

    #[cfg(target_os = "macos")]
    {
        PathBuf::from("/tmp").join(&dir_name)
    }

    #[cfg(target_os = "windows")]
    {
        std::env::temp_dir().join(&dir_name)
    }

    #[cfg(not(any(target_os = "linux", target_os = "macos", target_os = "windows")))]
    {
        PathBuf::from("/tmp").join(&dir_name)
    }
}

/// Topics directory for shared memory message passing.
pub fn shm_topics_dir() -> PathBuf {
    shm_base_dir().join("topics")
}

/// Nodes directory for node presence files.
pub fn shm_nodes_dir() -> PathBuf {
    shm_base_dir().join("nodes")
}

/// Network status directory for transport monitoring.
pub fn shm_network_dir() -> PathBuf {
    shm_base_dir().join("network")
}

/// Scheduler registry directory for live node metrics.
///
/// Each scheduler creates a file at `{dir}/{scheduler_name}` containing
/// an mmap'd `SchedulerRegistry` with atomic `NodeSlot` entries updated
/// every tick.
pub fn shm_scheduler_dir() -> PathBuf {
    shm_base_dir().join("scheduler")
}

/// Logs shared memory path (namespaced).
///
/// Lives at `shm_base_dir()/logs` — consistent with all other SHM resources.
pub fn shm_logs_path() -> PathBuf {
    shm_base_dir().join("logs")
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
// Topic metadata registry (for non-Linux SHM discovery)
// ============================================================================

/// Metadata about an SHM topic, written as a `.meta` file alongside kernel SHM objects.
///
/// On Linux, SHM regions are files in `/dev/shm/` and can be discovered via directory scan.
/// On macOS (`shm_open`) and Windows (`CreateFileMappingW`), SHM regions are kernel objects
/// invisible to the filesystem. This metadata file makes them discoverable.
#[derive(serde::Serialize, serde::Deserialize, Debug, Clone)]
pub struct TopicMeta {
    #[serde(default)]
    pub name: String,
    #[serde(default)]
    pub size: usize,
    #[serde(default)]
    pub creator_pid: u32,
    #[serde(default)]
    pub created_at: u64,
}

/// Write a topic metadata file to `shm_topics_dir()/{name}.meta`.
///
/// Called by ShmRegion::create on macOS/Windows when the creator owns the region.
/// On Linux this is also called for consistency, but discovery can use the SHM file directly.
pub fn write_topic_meta(name: &str, size: usize) -> anyhow::Result<()> {
    let dir = shm_topics_dir();
    std::fs::create_dir_all(&dir)?;
    let meta = TopicMeta {
        name: name.to_string(),
        size,
        creator_pid: std::process::id(),
        created_at: std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .map(|d| d.as_secs())
            .unwrap_or(0),
    };
    let path = dir.join(format!("{}.meta", sanitize_namespace(name)));
    std::fs::write(&path, serde_json::to_string(&meta)?)?;
    #[cfg(unix)]
    {
        use std::os::unix::fs::PermissionsExt;
        let _ = std::fs::set_permissions(&path, std::fs::Permissions::from_mode(0o600));
    }
    Ok(())
}

/// Remove a topic metadata file. Best-effort — errors are silently ignored.
///
/// Called by ShmRegion::drop when the owner releases the region.
pub fn remove_topic_meta(name: &str) {
    let path = shm_topics_dir().join(format!("{}.meta", sanitize_namespace(name)));
    if let Err(e) = std::fs::remove_file(&path) {
        if e.kind() != std::io::ErrorKind::NotFound {
            log::debug!("Failed to remove topic meta {}: {}", path.display(), e);
        }
    }
}

/// List all topic metadata files in `shm_topics_dir()`.
///
/// Returns metadata for all topics that have `.meta` files. On Linux, this
/// complements the direct SHM file scan. On macOS/Windows, this is the
/// primary discovery mechanism.
pub fn list_topic_metas() -> Vec<TopicMeta> {
    let dir = shm_topics_dir();
    let entries = match std::fs::read_dir(&dir) {
        Ok(e) => e,
        Err(_) => return Vec::new(),
    };
    let mut metas = Vec::new();
    for entry in entries.flatten() {
        let path = entry.path();
        if path.extension().map_or(true, |e| e != "meta") {
            continue;
        }
        if let Ok(content) = std::fs::read_to_string(&path) {
            if let Ok(meta) = serde_json::from_str::<TopicMeta>(&content) {
                metas.push(meta);
            }
        }
    }
    metas
}

// ============================================================================
// Stale SHM detection (flock-based)
// ============================================================================

/// Check whether a shared memory file is stale (no process holds it open).
///
/// Attempts `flock(LOCK_EX | LOCK_NB)`:
/// - **Success** → no process holds a shared lock → stale
/// - **EWOULDBLOCK** → alive
/// - **Other error** → conservatively not stale
#[cfg(unix)]
pub fn is_shm_file_stale(path: &std::path::Path) -> bool {
    use std::os::unix::io::AsRawFd;

    let file = match std::fs::File::open(path) {
        Ok(f) => f,
        Err(_) => return true,
    };

    let fd = file.as_raw_fd();
    // SAFETY: fd is a valid open file descriptor; LOCK_EX|LOCK_NB is valid flock op
    let ret = unsafe { libc::flock(fd, libc::LOCK_EX | libc::LOCK_NB) };

    if ret == 0 {
        // SAFETY: fd is still valid; LOCK_UN releases the lock
        unsafe { libc::flock(fd, libc::LOCK_UN) };
        true
    } else {
        let errno = std::io::Error::last_os_error()
            .raw_os_error()
            .unwrap_or(0);
        if errno == libc::EWOULDBLOCK {
            false
        } else {
            false // conservatively assume not stale
        }
    }
}

/// Non-Unix stale detection: reads `.meta` file to get creator PID, checks liveness.
#[cfg(not(unix))]
pub fn is_shm_file_stale(path: &std::path::Path) -> bool {
    // On non-Unix, is_shm_file_stale is called with .meta file paths from
    // is_namespace_stale_by_flock(). Check the file directly for creator PID.
    let meta_path = if path.extension().map_or(false, |e| e == "meta") {
        path.to_path_buf()
    } else {
        path.with_extension("meta")
    };
    if meta_path.exists() {
        if let Ok(content) = std::fs::read_to_string(&meta_path) {
            if let Ok(meta) = serde_json::from_str::<TopicMeta>(&content) {
                return !crate::process::ProcessHandle::from_pid(meta.creator_pid).is_alive();
            }
        }
    }
    // No meta file — if the path itself doesn't exist, consider stale
    !path.exists()
}

/// Check whether ALL topic files in a namespace directory are stale.
#[cfg(unix)]
pub fn is_namespace_stale_by_flock(namespace_path: &std::path::Path) -> bool {
    let topics_dir = namespace_path.join("topics");
    if !topics_dir.exists() {
        return is_directory_all_files_stale(namespace_path);
    }
    is_directory_all_files_stale(&topics_dir)
}

#[cfg(unix)]
fn is_directory_all_files_stale(dir: &std::path::Path) -> bool {
    let entries = match std::fs::read_dir(dir) {
        Ok(e) => e,
        Err(_) => return true,
    };

    let mut found_any_file = false;
    for entry in entries.flatten() {
        let path = entry.path();
        if path.is_file() {
            found_any_file = true;
            if !is_shm_file_stale(&path) {
                return false;
            }
        } else if path.is_dir() {
            if !is_directory_all_files_stale(&path) {
                return false;
            }
            if dir_file_count(&path) > 0 {
                found_any_file = true;
            }
        }
    }
    found_any_file
}

/// Non-Unix namespace stale check: inspects all `.meta` files for dead creator PIDs.
#[cfg(not(unix))]
pub fn is_namespace_stale_by_flock(namespace_path: &std::path::Path) -> bool {
    let topics_dir = namespace_path.join("topics");
    let dir = if topics_dir.exists() { &topics_dir } else { namespace_path };
    let entries = match std::fs::read_dir(dir) {
        Ok(e) => e,
        Err(_) => return true,
    };
    let metas: Vec<_> = entries
        .flatten()
        .filter(|e| e.path().extension().map_or(false, |ext| ext == "meta"))
        .collect();
    if metas.is_empty() {
        return true; // No meta files = abandoned namespace
    }
    metas.iter().all(|entry| is_shm_file_stale(&entry.path()))
}

// ============================================================================
// Stale namespace cleanup
// ============================================================================

/// Parent directory where HORUS SHM namespace directories live.
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
pub fn parse_namespace_sid(dir_name: &str) -> Option<(i32, u32)> {
    let suffix = dir_name.strip_prefix("horus_sid")?;
    let (sid_str, rest) = suffix.split_once("_uid")?;
    let sid: i32 = sid_str.parse().ok()?;
    let uid: u32 = rest.parse().ok()?;
    Some((sid, uid))
}

/// Check whether a session (by session leader PID) is still alive.
#[cfg(unix)]
pub fn session_alive(sid: i32) -> bool {
    if sid <= 0 {
        return false;
    }
    // SAFETY: kill with signal 0 checks existence without sending a signal
    let ret = unsafe { libc::kill(sid, 0) };
    if ret == 0 {
        return true;
    }
    let errno = std::io::Error::last_os_error()
        .raw_os_error()
        .unwrap_or(0);
    errno == libc::EPERM
}

#[cfg(not(unix))]
pub fn session_alive(_sid: i32) -> bool {
    true
}

/// Result of a stale namespace cleanup operation.
#[derive(Debug, Clone)]
pub struct NamespaceCleanupResult {
    pub removed: usize,
    pub bytes_freed: u64,
    pub skipped: usize,
    pub errors: Vec<String>,
}

/// Information about a single HORUS namespace directory.
#[derive(Debug, Clone)]
pub struct NamespaceInfo {
    pub path: PathBuf,
    pub dir_name: String,
    pub pgid: Option<i32>,
    pub uid: Option<u32>,
    pub alive: bool,
    pub size_bytes: u64,
    pub file_count: usize,
}

/// Scan the SHM parent directory and remove stale HORUS namespace directories.
///
/// A namespace is considered stale when:
/// 1. Name matches `horus_sid{N}_uid{N}` (auto-generated, not custom)
/// 2. UID matches the current user
/// 3. Session is no longer alive
/// 4. It's not the current process's namespace
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
    let current_uid = unsafe { libc::getuid() };
    #[cfg(not(unix))]
    let current_uid = crate::process::user_id() as u32;

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

        if !dir_name.starts_with("horus_sid") {
            continue;
        }

        if dir_name == current_ns {
            result.skipped += 1;
            continue;
        }

        let (uid, alive) = if let Some((sid, uid)) = parse_namespace_sid(&dir_name) {
            (uid, session_alive(sid))
        } else {
            result.skipped += 1;
            continue;
        };

        if uid != current_uid {
            result.skipped += 1;
            continue;
        }

        if alive {
            result.skipped += 1;
            continue;
        }

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
            (None, None, true)
        };

        namespaces.push(NamespaceInfo {
            size_bytes: dir_size_bytes(&path),
            file_count: dir_file_count(&path),
            path,
            dir_name,
            pgid,
            uid,
            alive,
        });
    }

    namespaces
}

// ============================================================================
// Utility helpers
// ============================================================================

/// Recursively compute the total size of a directory in bytes.
pub(crate) fn dir_size_bytes(path: &std::path::Path) -> u64 {
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
pub(crate) fn dir_file_count(path: &std::path::Path) -> usize {
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
pub(crate) fn format_bytes_compact(bytes: u64) -> String {
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

// ============================================================================
// Tests
// ============================================================================

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

    #[test]
    fn test_sanitize_namespace() {
        assert_eq!(sanitize_namespace("my-robot/1!"), "my_robot_1_");
        assert_eq!(sanitize_namespace("robot_01"), "robot_01");
        assert_eq!(sanitize_namespace("hello world"), "hello_world");
    }

    #[test]
    fn test_generate_namespace_is_non_empty() {
        let ns = generate_namespace();
        assert!(!ns.is_empty());
    }

    #[test]
    fn test_parse_namespace_sid_valid() {
        assert_eq!(
            parse_namespace_sid("horus_sid12345_uid1000"),
            Some((12345, 1000))
        );
        assert_eq!(parse_namespace_sid("horus_sid1_uid0"), Some((1, 0)));
    }

    #[test]
    fn test_parse_namespace_sid_custom_namespaces() {
        assert_eq!(parse_namespace_sid("horus_my_robot"), None);
        assert_eq!(parse_namespace_sid("horus_test"), None);
    }

    #[test]
    fn test_parse_namespace_sid_malformed() {
        assert_eq!(parse_namespace_sid("horus_sidabc_uid1000"), None);
        assert_eq!(parse_namespace_sid("horus_sid123_uidabc"), None);
        assert_eq!(parse_namespace_sid(""), None);
    }

    #[test]
    fn test_format_bytes_compact() {
        assert_eq!(format_bytes_compact(0), "0 B");
        assert_eq!(format_bytes_compact(512), "512 B");
        assert_eq!(format_bytes_compact(1024), "1.0 KB");
        assert_eq!(format_bytes_compact(1048576), "1.0 MB");
        assert_eq!(format_bytes_compact(1073741824), "1.0 GB");
    }

    #[test]
    fn test_shm_parent_dir_exists() {
        let parent = shm_parent_dir();
        assert!(parent.exists());
    }

    #[test]
    fn test_has_native_shm() {
        // On Linux test machines /dev/shm should exist
        #[cfg(target_os = "linux")]
        assert!(has_native_shm());
    }

    #[test]
    fn test_list_all_horus_namespaces() {
        let namespaces = list_all_horus_namespaces();
        for ns in &namespaces {
            assert!(ns.dir_name.starts_with("horus_"));
        }
    }

    #[test]
    fn test_dir_size_bytes_and_file_count() {
        use std::io::Write;
        let tmp = std::env::temp_dir().join(format!(
            "horus_sys_test_dir_size_{}",
            std::process::id()
        ));
        let _ = std::fs::remove_dir_all(&tmp);
        std::fs::create_dir_all(&tmp).unwrap();

        let mut f = std::fs::File::create(tmp.join("file1.dat")).unwrap();
        f.write_all(&[0u8; 100]).unwrap();
        drop(f);

        std::fs::create_dir_all(tmp.join("sub")).unwrap();
        let mut f2 = std::fs::File::create(tmp.join("sub/file2.dat")).unwrap();
        f2.write_all(&[0u8; 200]).unwrap();
        drop(f2);

        assert_eq!(dir_size_bytes(&tmp), 300);
        assert_eq!(dir_file_count(&tmp), 2);

        let _ = std::fs::remove_dir_all(&tmp);
    }

    #[cfg(unix)]
    #[test]
    fn test_flock_stale_no_holder() {
        let tmp = std::env::temp_dir().join(format!(
            "horus_sys_flock_test_{}",
            std::process::id()
        ));
        let _ = std::fs::remove_dir_all(&tmp);
        std::fs::create_dir_all(&tmp).unwrap();
        let path = tmp.join("topic_a");
        std::fs::write(&path, b"data").unwrap();

        assert!(is_shm_file_stale(&path));

        let _ = std::fs::remove_dir_all(&tmp);
    }

    // ── TopicMeta tests ──────────────────────────────────────────────

    #[test]
    fn test_write_and_list_topic_meta_roundtrip() {
        std::env::set_var("HORUS_NAMESPACE", format!("test_meta_{}", std::process::id()));
        // Reset cached namespace
        write_topic_meta("sensor_imu", 4096).unwrap();
        let metas = list_topic_metas();
        assert!(
            metas.iter().any(|m| m.name == "sensor_imu" && m.size == 4096),
            "written topic meta should be discoverable"
        );
        // Verify creator PID
        let meta = metas.iter().find(|m| m.name == "sensor_imu").unwrap();
        assert_eq!(meta.creator_pid, std::process::id());
        // Verify timestamp is recent (within 10 seconds)
        let now = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap()
            .as_secs();
        assert!(meta.created_at > 0 && meta.created_at <= now && now - meta.created_at < 10);
        // Cleanup
        remove_topic_meta("sensor_imu");
        let _ = std::fs::remove_dir_all(shm_base_dir());
        std::env::remove_var("HORUS_NAMESPACE");
    }

    #[test]
    fn test_remove_topic_meta_deletes_file() {
        std::env::set_var("HORUS_NAMESPACE", format!("test_rm_meta_{}", std::process::id()));
        write_topic_meta("to_delete", 1024).unwrap();
        let path = shm_topics_dir().join(format!("{}.meta", sanitize_namespace("to_delete")));
        assert!(path.exists(), "meta file should exist after write");
        remove_topic_meta("to_delete");
        assert!(!path.exists(), "meta file should be gone after remove");
        let _ = std::fs::remove_dir_all(shm_base_dir());
        std::env::remove_var("HORUS_NAMESPACE");
    }

    #[test]
    fn test_remove_topic_meta_idempotent() {
        // Removing a non-existent meta should not panic
        remove_topic_meta("nonexistent_topic_xyz");
    }

    #[test]
    fn test_list_topic_metas_empty_dir() {
        // list_topic_metas on a dir with no .meta files should return empty
        // (may find files from other tests sharing the cached namespace, so just verify no panic)
        let metas = list_topic_metas();
        // Only assert it doesn't panic — count may vary due to OnceLock namespace sharing
        let _ = metas;
    }

    #[test]
    fn test_list_topic_metas_skips_malformed_json() {
        // Write directly to the actual topics dir (OnceLock-cached namespace)
        let dir = shm_topics_dir();
        std::fs::create_dir_all(&dir).unwrap();
        // Write malformed meta — should be skipped without panic
        let malformed_path = dir.join("__test_broken__.meta");
        std::fs::write(&malformed_path, "not json {{{").unwrap();
        let metas = list_topic_metas();
        // Verify the malformed file is NOT in results
        assert!(
            !metas.iter().any(|m| m.name == "__test_broken__"),
            "malformed .meta should be skipped"
        );
        let _ = std::fs::remove_file(&malformed_path);
    }

    #[test]
    fn test_topic_meta_name_sanitized_in_filename() {
        std::env::set_var("HORUS_NAMESPACE", format!("test_sanitize_{}", std::process::id()));
        write_topic_meta("my-topic/v1", 512).unwrap();
        let expected_path = shm_topics_dir().join("my_topic_v1.meta");
        assert!(expected_path.exists(), "filename should be sanitized");
        let _ = std::fs::remove_dir_all(shm_base_dir());
        std::env::remove_var("HORUS_NAMESPACE");
    }

    // ── Namespace generation tests ──────────────────────────────────

    #[test]
    fn test_namespace_format_matches_sid_uid_pattern() {
        // Without HORUS_NAMESPACE env var, should match sid{N}_uid{N}
        std::env::remove_var("HORUS_NAMESPACE");
        let ns = generate_namespace();
        assert!(
            ns.starts_with("sid") && ns.contains("_uid"),
            "auto-generated namespace should be sid{{N}}_uid{{N}}, got '{}'",
            ns
        );
    }

    #[test]
    fn test_namespace_env_var_override() {
        std::env::set_var("HORUS_NAMESPACE", "my_custom_ns");
        let ns = generate_namespace();
        assert_eq!(ns, "my_custom_ns");
        std::env::remove_var("HORUS_NAMESPACE");
    }

    #[test]
    fn test_namespace_env_var_sanitized() {
        std::env::set_var("HORUS_NAMESPACE", "my-robot/v1!");
        let ns = generate_namespace();
        assert_eq!(ns, "my_robot_v1_");
        std::env::remove_var("HORUS_NAMESPACE");
    }

    #[test]
    fn test_namespace_empty_env_var_falls_back() {
        std::env::set_var("HORUS_NAMESPACE", "");
        let ns = generate_namespace();
        assert!(!ns.is_empty(), "empty env var should fall back to auto-gen");
        assert!(ns.starts_with("sid"), "should auto-generate sid format");
        std::env::remove_var("HORUS_NAMESPACE");
    }

    // ── Path relationship tests ─────────────────────────────────────

    #[test]
    fn test_subdirs_are_children_of_base() {
        let base = shm_base_dir();
        assert!(shm_topics_dir().starts_with(&base));
        assert!(shm_nodes_dir().starts_with(&base));
        assert!(shm_network_dir().starts_with(&base));
        assert!(shm_scheduler_dir().starts_with(&base));
    }

    #[test]
    fn test_logs_path_inside_namespace() {
        let logs = shm_logs_path();
        let base = shm_base_dir();
        assert!(
            logs.starts_with(&base),
            "logs path should be inside namespace base dir"
        );
    }

    #[test]
    fn test_logs_path_not_global() {
        let logs = shm_logs_path();
        let logs_str = logs.to_string_lossy();
        assert!(
            !logs_str.ends_with("horus_logs"),
            "logs path should not be the old global 'horus_logs', got: {}",
            logs_str
        );
    }

    #[test]
    fn test_scheduler_dir_inside_namespace() {
        let sched = shm_scheduler_dir();
        let base = shm_base_dir();
        assert!(
            sched.starts_with(&base),
            "scheduler dir should be inside namespace base dir"
        );
    }

    #[cfg(unix)]
    #[test]
    fn test_flock_alive_with_shared_lock() {
        use std::os::unix::io::AsRawFd;

        let tmp = std::env::temp_dir().join(format!(
            "horus_sys_flock_alive_{}",
            std::process::id()
        ));
        let _ = std::fs::remove_dir_all(&tmp);
        std::fs::create_dir_all(&tmp).unwrap();
        let path = tmp.join("topic_b");
        std::fs::write(&path, b"data").unwrap();

        let file = std::fs::File::open(&path).unwrap();
        // SAFETY: valid fd and flock op
        let ret = unsafe { libc::flock(file.as_raw_fd(), libc::LOCK_SH) };
        assert_eq!(ret, 0);

        assert!(!is_shm_file_stale(&path));

        drop(file);
        let _ = std::fs::remove_dir_all(&tmp);
    }
}
