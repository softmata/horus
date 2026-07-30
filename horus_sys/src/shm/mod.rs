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

#[cfg(not(any(target_os = "linux", target_os = "macos", target_os = "windows")))]
mod fallback;
#[cfg(target_os = "linux")]
mod linux;
#[cfg(target_os = "macos")]
mod macos;
#[cfg(target_os = "windows")]
mod windows;

#[cfg(not(any(target_os = "linux", target_os = "macos", target_os = "windows")))]
pub use self::fallback::ShmRegion;
#[cfg(target_os = "linux")]
pub use self::linux::ShmRegion;
#[cfg(target_os = "macos")]
pub use self::macos::ShmRegion;
#[cfg(target_os = "windows")]
pub use self::windows::ShmRegion;

use std::path::{Path, PathBuf};

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
/// 2. Otherwise the fixed shared namespace `"default"` (ROS 2 domain-0 model:
///    every process on the machine shares one namespace). Multi-robot isolation
///    is opt-in — set `HORUS_NAMESPACE` (or a launch-config `namespace`) per robot.
pub fn generate_namespace() -> String {
    // Check for explicit override first (for isolation when needed)
    if let Ok(ns) = std::env::var("HORUS_NAMESPACE") {
        if !ns.is_empty() {
            let sanitized = sanitize_namespace(&ns);
            if !sanitized.chars().all(|c| c == '_') {
                return sanitized;
            }
        }
    }

    // Default: fixed namespace shared by all terminals on this machine.
    // Like ROS 2's default domain_id=0 — everything sees everything.
    // Set HORUS_NAMESPACE to isolate when explicitly needed.
    "default".to_string()
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

/// Canonical on-disk path of a topic's SHM backing file.
///
/// This is the **single definition** of the topic-name → path mapping. Anything
/// that opens a topic's backing file directly — notably `horus_net`'s SHM
/// reader and writer — must go through here rather than reconstructing the name,
/// which is how the replication seam silently diverged for four months (the
/// network side was still building `horus_<name>` after the prefix was dropped
/// here in 2026-03-29).
///
/// Callers must have validated `name` with [`validate_region_name`] first;
/// `ShmRegion::new` does so on every path that reaches it.
pub fn topic_shm_path(name: &str) -> PathBuf {
    shm_topics_dir().join(name)
}

// ============================================================================
// Permission hardening
// ============================================================================

/// Directory mode for every HORUS SHM directory: owner-only.
pub const SHM_DIR_MODE: u32 = 0o700;

/// File mode for every HORUS SHM file: owner read/write only.
pub const SHM_FILE_MODE: u32 = 0o600;

/// Create `path` and its parents with owner-only permissions, and tighten any
/// that already exist too loosely.
///
/// `ShmRegion::new` has always used mode 0o700, but several other call sites
/// reached the same tree through a bare `std::fs::create_dir_all`, which applies
/// `0o777 & !umask` — typically 0o755, or 0o775 under the common `umask 002`.
/// Whichever code path ran *first* therefore decided the mode for everything
/// underneath, and in practice the loose one usually won: on a machine running
/// HORUS, `/dev/shm/horus_<ns>` is observed as `drwxrwxr-x`, not `drwx------`.
///
/// That makes the namespace listable by every local user and writable by
/// everyone in the owner's group, which contradicts the "Process Isolation:
/// Shared memory with proper permissions" claim in SECURITY.md.
///
/// The repair pass only ever *removes* permission bits, and only on paths this
/// user owns, so it can never widen access or fight another user's ownership.
pub fn create_shm_dir_all(path: &Path) -> std::io::Result<()> {
    #[cfg(unix)]
    {
        use std::os::unix::fs::DirBuilderExt;
        std::fs::DirBuilder::new()
            .recursive(true)
            .mode(SHM_DIR_MODE)
            .create(path)?;
    }
    #[cfg(not(unix))]
    {
        std::fs::create_dir_all(path)?;
    }

    // Repair directories created by an older, looser code path. Walk from the
    // SHM base down to `path` so an already-existing base gets tightened too.
    harden_existing_dir(&shm_base_dir());
    let mut cursor = shm_base_dir();
    if let Ok(rest) = path.strip_prefix(&cursor) {
        for component in rest.components() {
            cursor = cursor.join(component);
            harden_existing_dir(&cursor);
        }
    } else {
        harden_existing_dir(path);
    }
    Ok(())
}

/// Tighten one existing directory to [`SHM_DIR_MODE`], if we own it and it is
/// currently looser. Silent no-op otherwise.
fn harden_existing_dir(path: &Path) {
    #[cfg(unix)]
    {
        use std::os::unix::fs::MetadataExt;
        use std::os::unix::fs::PermissionsExt;
        // symlink_metadata: never follow a symlink someone planted here.
        let Ok(meta) = std::fs::symlink_metadata(path) else {
            return;
        };
        if !meta.is_dir() {
            return;
        }
        // SAFETY: geteuid is always safe; it takes no arguments and cannot fail.
        let euid = unsafe { libc::geteuid() };
        if meta.uid() != euid {
            return; // not ours to change
        }
        let mode = meta.permissions().mode() & 0o777;
        if mode & !SHM_DIR_MODE != 0 {
            let _ = std::fs::set_permissions(path, std::fs::Permissions::from_mode(SHM_DIR_MODE));
        }
    }
    #[cfg(not(unix))]
    {
        let _ = path;
    }
}

/// Restrict an already-created SHM file to [`SHM_FILE_MODE`].
///
/// For files created through an `OpenOptions` that cannot carry a mode — or
/// that already exist from an older run — this drops the group and world bits.
/// Only ever narrows, and only on files this user owns.
pub fn harden_shm_file(path: &Path) {
    #[cfg(unix)]
    {
        use std::os::unix::fs::MetadataExt;
        use std::os::unix::fs::PermissionsExt;
        let Ok(meta) = std::fs::symlink_metadata(path) else {
            return;
        };
        if !meta.is_file() {
            return;
        }
        // SAFETY: geteuid takes no arguments and cannot fail.
        let euid = unsafe { libc::geteuid() };
        if meta.uid() != euid {
            return;
        }
        let mode = meta.permissions().mode() & 0o777;
        if mode & !SHM_FILE_MODE != 0 {
            let _ = std::fs::set_permissions(path, std::fs::Permissions::from_mode(SHM_FILE_MODE));
        }
    }
    #[cfg(not(unix))]
    {
        let _ = path;
    }
}

/// Write `bytes` to `path` with owner-only permissions.
///
/// `std::fs::write` creates at `0o666 & !umask` — 0o644, or 0o664 under the
/// common `umask 002`. The service and action JSON gateways carry request and
/// response payloads, so leaving them world-readable discloses whatever a
/// caller passed to a service.
///
/// `create_new` is deliberate: it refuses to follow or overwrite a file another
/// process planted at this path.
pub fn write_shm_file_new(path: &Path, bytes: &[u8]) -> std::io::Result<()> {
    use std::io::Write;
    let mut opts = std::fs::OpenOptions::new();
    opts.write(true).create_new(true);
    #[cfg(unix)]
    {
        use std::os::unix::fs::OpenOptionsExt;
        opts.mode(SHM_FILE_MODE);
    }
    let mut f = opts.open(path)?;
    f.write_all(bytes)?;
    f.sync_data()
}

/// Maximum length of a single path component, matching Linux `NAME_MAX`.
const NAME_MAX: usize = 255;

/// Maximum total length of a region name.
const REGION_NAME_MAX: usize = 512;

/// Reject region names that could escape the topics directory.
///
/// `ShmRegion::new` used to pass the caller's name straight into
/// `shm_topics_dir().join(name)` with no checks. Two properties of
/// `PathBuf::join` make that dangerous:
///
///   * joining an **absolute** path discards the base entirely, so
///     `Topic::new("/home/user/.bashrc")` targets that file, not a topic;
///   * `..` components are not normalised away, so
///     `Topic::new("../../../tmp/x")` escapes the topics directory.
///
/// Either one turns a topic name from any untrusted source — a CLI argument, a
/// launch file, `horus.toml`, a Python caller — into a create-and-mmap
/// primitive over an arbitrary path, as the running user.
///
/// A single embedded `/` remains legal: legacy hierarchical topic names rely on
/// it, and `ShmRegion::new` creates the intermediate directory. Only traversal
/// and absolute/rooted forms are rejected.
pub fn validate_region_name(name: &str) -> anyhow::Result<()> {
    anyhow::ensure!(!name.is_empty(), "SHM region name must not be empty");
    anyhow::ensure!(
        name.len() <= REGION_NAME_MAX,
        "SHM region name is too long ({} bytes, max {REGION_NAME_MAX})",
        name.len()
    );
    anyhow::ensure!(
        !name.contains('\0'),
        "SHM region name must not contain a NUL byte"
    );
    // Absolute (POSIX or Windows UNC) — `join` would discard the topics dir.
    anyhow::ensure!(
        !name.starts_with('/') && !name.starts_with('\\'),
        "SHM region name must be relative, got absolute path {name:?}"
    );
    // Windows drive-relative, e.g. `C:evil` — also escapes the base.
    let bytes = name.as_bytes();
    anyhow::ensure!(
        !(bytes.len() >= 2 && bytes[1] == b':'),
        "SHM region name must not be drive-qualified, got {name:?}"
    );

    for component in name.split(['/', '\\']) {
        anyhow::ensure!(
            component != ".." && component != ".",
            "SHM region name must not contain a {component:?} path component, got {name:?}"
        );
        anyhow::ensure!(
            component.len() <= NAME_MAX,
            "SHM region name component exceeds {NAME_MAX} bytes in {name:?}"
        );
    }
    Ok(())
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

/// Error logs shared memory path (namespaced).
///
/// Separate persistent buffer for Error/Warning entries that survives
/// sensor pub/sub volume flooding. Lives alongside the main log buffer.
pub fn shm_error_logs_path() -> PathBuf {
    shm_base_dir().join("error_logs")
}

/// Remote logs shared memory path (namespaced).
///
/// Separate buffer for log entries received from remote machines via horus_net.
/// Prevents remote log flooding from evicting local logs in GLOBAL_LOG_BUFFER.
pub fn shm_remote_logs_path() -> PathBuf {
    shm_base_dir().join("remote_logs")
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
        let errno = std::io::Error::last_os_error().raw_os_error().unwrap_or(0);
        // Both EWOULDBLOCK (lock held → stale) and other errors map to false:
        // EWOULDBLOCK means the creator still holds the lock (not stale);
        // other errors conservatively assume not stale to avoid false eviction.
        #[allow(clippy::if_same_then_else)]
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
    let dir = if topics_dir.exists() {
        &topics_dir
    } else {
        namespace_path
    };
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

/// Serializes tests that touch process-global SHM state — the `HORUS_NAMESPACE`
/// env var, the `OnceLock`-cached `shm_namespace()`, and the shared `/dev/shm`
/// paths derived from them. Multiple crate modules' tests (`shm`, `discover`)
/// share this one lock, because they all read/write the same cached namespace
/// dir; without it they race under `--test-threads>1` (one test's
/// `remove_dir_all` wipes a dir another is reading). Poisoning is irrelevant to
/// test setup, so a poisoned lock is recovered rather than propagated.
#[cfg(test)]
pub(crate) fn env_lock() -> std::sync::MutexGuard<'static, ()> {
    static ENV_LOCK: std::sync::Mutex<()> = std::sync::Mutex::new(());
    ENV_LOCK.lock().unwrap_or_else(|e| e.into_inner())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_shm_paths_are_valid() {
        let _guard = env_lock();
        let base = shm_base_dir();
        assert!(!base.as_os_str().is_empty());

        let topics = shm_topics_dir();
        assert!(topics.starts_with(&base));

        let nodes = shm_nodes_dir();
        assert!(nodes.starts_with(&base));
    }

    #[test]
    fn test_shm_base_dir_starts_with_horus_underscore() {
        let _guard = env_lock();
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
        let _guard = env_lock();
        use std::io::Write;
        let tmp =
            std::env::temp_dir().join(format!("horus_sys_test_dir_size_{}", std::process::id()));
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
        let _guard = env_lock();
        let tmp = std::env::temp_dir().join(format!("horus_sys_flock_test_{}", std::process::id()));
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
        let _guard = env_lock();
        std::env::set_var(
            "HORUS_NAMESPACE",
            format!("test_meta_{}", std::process::id()),
        );
        // Reset cached namespace
        write_topic_meta("sensor_imu", 4096).unwrap();
        let metas = list_topic_metas();
        assert!(
            metas
                .iter()
                .any(|m| m.name == "sensor_imu" && m.size == 4096),
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
        let _guard = env_lock();
        std::env::set_var(
            "HORUS_NAMESPACE",
            format!("test_rm_meta_{}", std::process::id()),
        );
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
        let _guard = env_lock();
        // Removing a non-existent meta should not panic
        remove_topic_meta("nonexistent_topic_xyz");
    }

    #[test]
    fn test_list_topic_metas_empty_dir() {
        let _guard = env_lock();
        // list_topic_metas on a dir with no .meta files should return empty
        // (may find files from other tests sharing the cached namespace, so just verify no panic)
        let metas = list_topic_metas();
        // Only assert it doesn't panic — count may vary due to OnceLock namespace sharing
        let _ = metas;
    }

    #[test]
    fn test_list_topic_metas_skips_malformed_json() {
        let _guard = env_lock();
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
        let _guard = env_lock();
        std::env::set_var(
            "HORUS_NAMESPACE",
            format!("test_sanitize_{}", std::process::id()),
        );
        write_topic_meta("my-topic/v1", 512).unwrap();
        let expected_path = shm_topics_dir().join("my_topic_v1.meta");
        assert!(expected_path.exists(), "filename should be sanitized");
        let _ = std::fs::remove_dir_all(shm_base_dir());
        std::env::remove_var("HORUS_NAMESPACE");
    }

    // ── Namespace generation tests ──────────────────────────────────

    #[test]
    fn test_namespace_defaults_to_shared_when_unset() {
        // Contract (ROS 2 domain-0 model): with no HORUS_NAMESPACE override the
        // namespace is the fixed shared "default". Isolation is opt-in via the
        // env var / launch-config namespace (see test_namespace_env_var_override).
        let _guard = env_lock();
        std::env::remove_var("HORUS_NAMESPACE");
        assert_eq!(generate_namespace(), "default");
    }

    #[test]
    fn test_namespace_env_var_override() {
        let _guard = env_lock();
        std::env::set_var("HORUS_NAMESPACE", "my_custom_ns");
        let ns = generate_namespace();
        assert_eq!(ns, "my_custom_ns");
        std::env::remove_var("HORUS_NAMESPACE");
    }

    #[test]
    fn test_namespace_env_var_sanitized() {
        let _guard = env_lock();
        std::env::set_var("HORUS_NAMESPACE", "my-robot/v1!");
        let ns = generate_namespace();
        assert_eq!(ns, "my_robot_v1_");
        std::env::remove_var("HORUS_NAMESPACE");
    }

    #[test]
    fn test_namespace_empty_env_var_falls_back() {
        // An empty (or whitespace/all-separator) override must not yield an empty
        // namespace — it falls back to the shared "default".
        let _guard = env_lock();
        std::env::set_var("HORUS_NAMESPACE", "");
        let ns = generate_namespace();
        assert!(
            !ns.is_empty(),
            "empty env var must not yield an empty namespace"
        );
        assert_eq!(ns, "default", "empty override falls back to shared default");
        std::env::remove_var("HORUS_NAMESPACE");
    }

    // ── Path relationship tests ─────────────────────────────────────

    #[test]
    fn test_subdirs_are_children_of_base() {
        let _guard = env_lock();
        let base = shm_base_dir();
        assert!(shm_topics_dir().starts_with(&base));
        assert!(shm_nodes_dir().starts_with(&base));
        assert!(shm_network_dir().starts_with(&base));
        assert!(shm_scheduler_dir().starts_with(&base));
    }

    #[test]
    fn test_logs_path_inside_namespace() {
        let _guard = env_lock();
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
        let _guard = env_lock();
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
        let _guard = env_lock();
        use std::os::unix::io::AsRawFd;

        let tmp =
            std::env::temp_dir().join(format!("horus_sys_flock_alive_{}", std::process::id()));
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

    // ─── Region-name validation (path-traversal hardening) ──────────────────

    #[test]
    fn validate_accepts_ordinary_topic_names() {
        for name in [
            "cmd_vel",
            "robot.imu",
            "robot.front_lidar.scan",
            "_horus.estop",
            "legacy/hierarchical/name", // single slashes remain legal
            "a",
        ] {
            assert!(
                validate_region_name(name).is_ok(),
                "{name:?} is a legitimate topic name and must be accepted"
            );
        }
    }

    #[test]
    fn validate_rejects_absolute_paths() {
        // PathBuf::join with an absolute path DISCARDS the base, so this used to
        // target the named file directly rather than a topic.
        for name in ["/tmp/pwned", "/etc/passwd", "/", "//x"] {
            assert!(
                validate_region_name(name).is_err(),
                "{name:?} must be rejected — join() would discard the topics dir"
            );
        }
    }

    #[test]
    fn validate_rejects_parent_traversal() {
        for name in [
            "../pwned",
            "../../../tmp/horus_pathaudit_out/pwned",
            "a/../../b",
            "..",
            "sub/..",
        ] {
            assert!(
                validate_region_name(name).is_err(),
                "{name:?} must be rejected — it escapes the topics directory"
            );
        }
    }

    #[test]
    fn validate_rejects_single_dot_components() {
        for name in ["./x", "a/./b", "."] {
            assert!(validate_region_name(name).is_err(), "{name:?}");
        }
    }

    #[test]
    fn validate_rejects_backslash_traversal() {
        // Windows separators must be treated as separators, not as ordinary
        // filename characters, or the check is bypassable on that platform.
        for name in ["..\\pwned", "a\\..\\..\\b", "\\absolute"] {
            assert!(validate_region_name(name).is_err(), "{name:?}");
        }
    }

    #[test]
    fn validate_rejects_drive_qualified_names() {
        for name in ["C:evil", "c:/windows/system32/x"] {
            assert!(validate_region_name(name).is_err(), "{name:?}");
        }
    }

    #[test]
    fn validate_rejects_empty_nul_and_overlong() {
        assert!(validate_region_name("").is_err(), "empty");
        assert!(validate_region_name("a\0b").is_err(), "embedded NUL");
        assert!(
            validate_region_name(&"x".repeat(256)).is_err(),
            "component longer than NAME_MAX"
        );
        assert!(
            validate_region_name(&"x".repeat(255)).is_ok(),
            "exactly NAME_MAX must still be accepted"
        );
    }

    #[test]
    fn validate_allows_dots_inside_a_component() {
        // Topic names are dot-separated by convention; only a component that IS
        // "." or ".." is a traversal.
        assert!(validate_region_name("robot..imu").is_ok());
        assert!(validate_region_name("...").is_ok());
    }

    #[test]
    fn shm_region_new_refuses_to_escape_the_topics_dir() {
        // End-to-end version of the abandoned probe left at
        // horus_core/tests/zz_pathaudit_tmp.rs by an earlier session.
        let marker = std::env::temp_dir().join(format!(
            "horus_traversal_guard_{}_{}",
            std::process::id(),
            "pwned"
        ));
        let _ = std::fs::remove_file(&marker);

        let rel = format!("../../..{}", marker.display());
        assert!(
            ShmRegion::new(&rel, 4096).is_err(),
            "traversal name must be refused"
        );
        assert!(
            ShmRegion::new(marker.to_str().unwrap(), 4096).is_err(),
            "absolute name must be refused"
        );
        assert!(
            !marker.exists(),
            "no file may be created outside the topics directory at {}",
            marker.display()
        );
    }

    // ─── Permission hardening ───────────────────────────────────────────────

    #[cfg(unix)]
    #[test]
    fn create_shm_dir_all_makes_owner_only_dirs() {
        use std::os::unix::fs::PermissionsExt;
        let root = std::env::temp_dir().join(format!("horus_perm_new_{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&root);
        let nested = root.join("a").join("b");
        create_shm_dir_all(&nested).unwrap();

        for p in [&root, &root.join("a"), &nested] {
            let mode = std::fs::metadata(p).unwrap().permissions().mode() & 0o777;
            assert_eq!(mode, SHM_DIR_MODE, "{} should be 0700, was {mode:o}", p.display());
        }
        let _ = std::fs::remove_dir_all(&root);
    }

    #[cfg(unix)]
    #[test]
    fn create_shm_dir_all_tightens_a_preexisting_loose_dir() {
        // The observed real-world state: an earlier bare create_dir_all left the
        // directory at 0775 under umask 002. Re-entering must repair it.
        use std::os::unix::fs::PermissionsExt;
        let dir = std::env::temp_dir().join(format!("horus_perm_fix_{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&dir);
        std::fs::create_dir_all(&dir).unwrap();
        std::fs::set_permissions(&dir, std::fs::Permissions::from_mode(0o775)).unwrap();
        assert_eq!(
            std::fs::metadata(&dir).unwrap().permissions().mode() & 0o777,
            0o775
        );

        create_shm_dir_all(&dir).unwrap();
        assert_eq!(
            std::fs::metadata(&dir).unwrap().permissions().mode() & 0o777,
            SHM_DIR_MODE,
            "a pre-existing loose directory must be tightened, not left alone"
        );
        let _ = std::fs::remove_dir_all(&dir);
    }

    #[cfg(unix)]
    #[test]
    fn write_shm_file_new_is_owner_only_and_refuses_to_clobber() {
        use std::os::unix::fs::PermissionsExt;
        let path = std::env::temp_dir().join(format!("horus_perm_file_{}", std::process::id()));
        let _ = std::fs::remove_file(&path);

        write_shm_file_new(&path, b"secret payload").unwrap();
        let mode = std::fs::metadata(&path).unwrap().permissions().mode() & 0o777;
        assert_eq!(mode, SHM_FILE_MODE, "gateway files must be 0600, was {mode:o}");
        assert_eq!(std::fs::read(&path).unwrap(), b"secret payload");

        // create_new semantics: a second write must fail rather than follow or
        // overwrite whatever is already sitting at the path.
        assert!(write_shm_file_new(&path, b"other").is_err());

        let _ = std::fs::remove_file(&path);
    }

    #[cfg(unix)]
    #[test]
    fn harden_shm_file_narrows_an_existing_world_readable_file() {
        use std::os::unix::fs::PermissionsExt;
        let path = std::env::temp_dir().join(format!("horus_perm_narrow_{}", std::process::id()));
        std::fs::write(&path, b"x").unwrap();
        std::fs::set_permissions(&path, std::fs::Permissions::from_mode(0o664)).unwrap();

        harden_shm_file(&path);
        assert_eq!(
            std::fs::metadata(&path).unwrap().permissions().mode() & 0o777,
            SHM_FILE_MODE
        );
        let _ = std::fs::remove_file(&path);
    }

    #[cfg(unix)]
    #[test]
    fn hardening_never_widens_permissions() {
        use std::os::unix::fs::PermissionsExt;
        let path = std::env::temp_dir().join(format!("horus_perm_keep_{}", std::process::id()));
        std::fs::write(&path, b"x").unwrap();
        std::fs::set_permissions(&path, std::fs::Permissions::from_mode(0o400)).unwrap();

        harden_shm_file(&path);
        assert_eq!(
            std::fs::metadata(&path).unwrap().permissions().mode() & 0o777,
            0o400,
            "an already-stricter mode must be left alone, not relaxed to 0600"
        );
        let _ = std::fs::remove_file(&path);
    }

    #[test]
    fn topic_shm_path_is_the_raw_name_under_the_topics_dir() {
        // The mapping horus_net must use. Any prefix or character mangling here
        // re-breaks the replication seam.
        let p = topic_shm_path("robot.imu");
        assert_eq!(p.parent().unwrap(), shm_topics_dir());
        assert_eq!(p.file_name().unwrap(), "robot.imu");
    }
}
