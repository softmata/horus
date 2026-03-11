//! Node presence files for monitor detection
//!
//! Each node creates a presence file at startup and removes it at shutdown.
//! Monitor scans this directory to discover all active nodes — like rqt but simpler.
//!
//! ## Structure
//!
//! Presence files live at `/dev/shm/horus/nodes/{node_name}.json` with mode 0600.
//! Writes use atomic rename (`{name}.json.tmp` → `{name}.json`) to prevent
//! partial reads.
//!
//! ## Node Name Rules
//!
//! Node names must satisfy [`validate_node_name`]:
//! - 1–255 ASCII characters
//! - Allowed: `a-z A-Z 0-9 _ - .`
//! - Forbidden: `/`, `\`, `..`, spaces, control characters
//!
//! ## Liveness Detection
//!
//! [`NodePresence::read_all`] verifies each entry by checking:
//! 1. `kill(pid, 0)` — process still exists
//! 2. PID start time comparison — detects PID reuse after node crash
//!
//! Stale and corrupt files are automatically cleaned up during reads.

use crate::core::node::TopicMetadata;
use crate::memory::platform::shm_nodes_dir;
use serde::{Deserialize, Serialize};
use std::fs;
#[cfg(unix)]
use std::os::unix::fs::{DirBuilderExt, OpenOptionsExt};
use std::path::PathBuf;
use std::time::{SystemTime, UNIX_EPOCH};

/// Node presence data written to shared memory
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct NodePresence {
    name: String,
    pid: u32,
    scheduler: Option<String>,
    publishers: Vec<TopicMetadata>,
    subscribers: Vec<TopicMetadata>,
    start_time: u64,
    priority: u32,
    rate_hz: Option<f64>,
    #[serde(default)]
    pid_start_time: u64,
    /// Node health status (Healthy/Warning/Error/Critical)
    #[serde(default)]
    health_status: Option<String>,
    /// Cumulative tick count since node start
    #[serde(default)]
    tick_count: u64,
    /// Cumulative error count since node start
    #[serde(default)]
    error_count: u32,
    /// Services provided by this node (e.g. ["set_params", "get_status"])
    #[serde(default)]
    services: Vec<String>,
    /// Actions provided by this node (e.g. ["navigate", "pick_place"])
    #[serde(default)]
    actions: Vec<String>,
}

impl NodePresence {
    pub fn name(&self) -> &str {
        &self.name
    }

    pub fn pid(&self) -> u32 {
        self.pid
    }

    pub fn scheduler(&self) -> Option<&str> {
        self.scheduler.as_deref()
    }

    pub fn publishers(&self) -> &[TopicMetadata] {
        &self.publishers
    }

    pub fn subscribers(&self) -> &[TopicMetadata] {
        &self.subscribers
    }

    pub fn start_time(&self) -> u64 {
        self.start_time
    }

    pub fn priority(&self) -> u32 {
        self.priority
    }

    pub fn rate_hz(&self) -> Option<f64> {
        self.rate_hz
    }

    pub fn pid_start_time(&self) -> u64 {
        self.pid_start_time
    }

    pub fn health_status(&self) -> Option<&str> {
        self.health_status.as_deref()
    }

    pub fn tick_count(&self) -> u64 {
        self.tick_count
    }

    pub fn error_count(&self) -> u32 {
        self.error_count
    }

    pub fn services(&self) -> &[String] {
        &self.services
    }

    pub fn actions(&self) -> &[String] {
        &self.actions
    }
}

/// Validate a node name for use in presence file paths.
///
/// Rules:
/// - 1 to 255 characters
/// - Only ASCII alphanumeric, `_`, `-`, `.`
/// - Must not contain `/`, `\`, or `..`
/// - Must not be empty
///
/// Returns `Ok(())` if valid, `Err(reason)` if invalid.
pub fn validate_node_name(name: &str) -> Result<(), String> {
    if name.is_empty() {
        return Err("node name must not be empty".into());
    }
    if name.len() > 255 {
        return Err(format!(
            "node name too long ({} chars, max 255)",
            name.len()
        ));
    }
    if name.contains("..") {
        return Err("node name must not contain '..'".into());
    }
    if !name
        .bytes()
        .all(|b| b.is_ascii_alphanumeric() || matches!(b, b'_' | b'-' | b'.'))
    {
        return Err(format!(
            "node name '{}' contains invalid characters (allowed: a-z A-Z 0-9 _ - .)",
            name
        ));
    }
    Ok(())
}

impl NodePresence {
    /// Create a new presence record.
    ///
    /// # Panics
    ///
    /// Panics if `name` is invalid (empty, too long, contains `/` or `..`).
    /// Use [`validate_node_name`] to check before calling.
    pub(crate) fn new(
        name: &str,
        scheduler: Option<&str>,
        publishers: Vec<TopicMetadata>,
        subscribers: Vec<TopicMetadata>,
        priority: u32,
        rate_hz: Option<f64>,
    ) -> Self {
        if let Err(e) = validate_node_name(name) {
            panic!("invalid node name '{}': {}", name, e);
        }
        let pid = std::process::id();
        Self {
            name: name.to_string(),
            pid,
            scheduler: scheduler.map(|s| s.to_string()),
            publishers,
            subscribers,
            start_time: SystemTime::now()
                .duration_since(UNIX_EPOCH)
                .unwrap_or_default()
                .as_secs(),
            priority,
            rate_hz,
            pid_start_time: read_pid_start_time(pid),
            health_status: None,
            tick_count: 0,
            error_count: 0,
            services: Vec::new(),
            actions: Vec::new(),
        }
    }

    /// Get the path for this node's presence file
    fn presence_path(node_name: &str) -> PathBuf {
        shm_nodes_dir().join(format!("{}.json", node_name))
    }

    /// Write presence file to shared memory (called at node start).
    ///
    /// The nodes directory is created with mode 0o700 (owner only) so that
    /// other local users cannot enumerate running nodes.  The presence file
    /// itself is written with mode 0o600 (owner read/write only).
    ///
    /// If a presence file already exists for this node name and its process
    /// is still alive, a warning is logged before overwriting.
    pub(crate) fn write(&self) -> std::io::Result<()> {
        // Detect duplicate: another live process already owns this node name
        let path = Self::presence_path(&self.name);
        if path.exists() {
            if let Ok(content) = fs::read_to_string(&path) {
                if let Ok(existing) = serde_json::from_str::<NodePresence>(&content) {
                    if existing.pid != self.pid
                        && is_alive(existing.pid, existing.pid_start_time)
                    {
                        eprintln!(
                            "[horus] WARNING: node '{}' already registered by PID {} \
                             (this is PID {}). Overwriting presence file — \
                             duplicate node names cause unreliable discovery.",
                            self.name, existing.pid, self.pid
                        );
                    }
                }
            }
        }

        let dir = shm_nodes_dir();

        // Create directory hierarchy with owner-only access.
        #[cfg(unix)]
        fs::DirBuilder::new()
            .recursive(true)
            .mode(0o700)
            .create(&dir)?;
        #[cfg(not(unix))]
        fs::create_dir_all(&dir)?;

        let dest = Self::presence_path(&self.name);
        let json = serde_json::to_string_pretty(self)
            .map_err(|e| std::io::Error::new(std::io::ErrorKind::InvalidData, e))?;

        // Atomic write: write to a temporary file then rename, so readers
        // never see a partially-written presence file.
        let tmp_path = dest.with_extension("json.tmp");

        #[cfg(unix)]
        {
            use std::io::Write;
            let mut file = fs::OpenOptions::new()
                .write(true)
                .create(true)
                .truncate(true)
                .mode(0o600)
                .open(&tmp_path)?;
            file.write_all(json.as_bytes())?;
            file.flush()?;
        }
        #[cfg(not(unix))]
        fs::write(&tmp_path, &json)?;

        fs::rename(&tmp_path, &dest)
    }

    /// Update an existing presence file with new topic lists.
    ///
    /// Used when a node dynamically adds or removes publishers/subscribers
    /// at runtime.  The file is rewritten atomically.
    pub(crate) fn update_topics(
        node_name: &str,
        publishers: Vec<TopicMetadata>,
        subscribers: Vec<TopicMetadata>,
    ) -> std::io::Result<()> {
        let path = Self::presence_path(node_name);
        let content = fs::read_to_string(&path)?;
        let mut presence: NodePresence = serde_json::from_str(&content)
            .map_err(|e| std::io::Error::new(std::io::ErrorKind::InvalidData, e))?;
        presence.publishers = publishers;
        presence.subscribers = subscribers;
        presence.write()
    }

    /// Update runtime metrics (health, tick count, error count) in the presence file.
    ///
    /// Called periodically by the scheduler to keep monitor data fresh.
    pub(crate) fn update_metrics(
        node_name: &str,
        health: &str,
        ticks: u64,
        errors: u32,
    ) -> std::io::Result<()> {
        let path = Self::presence_path(node_name);
        let content = fs::read_to_string(&path)?;
        let mut presence: NodePresence = serde_json::from_str(&content)
            .map_err(|e| std::io::Error::new(std::io::ErrorKind::InvalidData, e))?;
        presence.health_status = Some(health.to_string());
        presence.tick_count = ticks;
        presence.error_count = errors;
        presence.write()
    }

    /// Remove presence file from shared memory (called at node shutdown)
    pub(crate) fn remove(node_name: &str) -> std::io::Result<()> {
        let path = Self::presence_path(node_name);
        if path.exists() {
            fs::remove_file(path)?;
        }
        Ok(())
    }

    /// Read presence file for a specific node (used by monitor)
    pub fn read(node_name: &str) -> Option<Self> {
        let path = Self::presence_path(node_name);
        let content = fs::read_to_string(&path).ok()?;
        serde_json::from_str(&content).ok()
    }

    /// Read all presence files (used by monitor to discover all nodes)
    pub fn read_all() -> Vec<Self> {
        let dir = shm_nodes_dir();
        if !dir.exists() {
            return Vec::new();
        }

        let mut nodes = Vec::new();
        if let Ok(entries) = fs::read_dir(&dir) {
            for entry in entries.flatten() {
                let path = entry.path();
                if path.extension().is_some_and(|ext| ext == "json") {
                    match fs::read_to_string(&path) {
                        Ok(content) => {
                            match serde_json::from_str::<NodePresence>(&content) {
                                Ok(presence) => {
                                    // Verify process is still alive and is the same process
                                    // that wrote this file (not a recycled PID).
                                    if is_alive(presence.pid, presence.pid_start_time) {
                                        nodes.push(presence);
                                    } else {
                                        // Clean up stale presence file
                                        let _ = fs::remove_file(&path);
                                    }
                                }
                                Err(_) => {
                                    // Corrupt JSON — remove the file so it doesn't persist
                                    let _ = fs::remove_file(&path);
                                }
                            }
                        }
                        Err(_) => {
                            // Unreadable file (permissions, I/O error) — skip silently
                        }
                    }
                }
            }
        }
        nodes
    }
}

/// Read the OS-level start time of a process to detect PID reuse.
///
/// Returns 0 when unavailable (non-Linux/macOS platforms or read failure).
fn read_pid_start_time(pid: u32) -> u64 {
    #[cfg(target_os = "linux")]
    {
        read_pid_start_time_linux(pid)
    }

    #[cfg(target_os = "macos")]
    {
        read_pid_start_time_macos(pid)
    }

    #[cfg(not(any(target_os = "linux", target_os = "macos")))]
    {
        let _ = pid;
        0
    }
}

/// Linux implementation: parse `/proc/{pid}/stat` and extract `starttime`
/// (field 22 in the file, 1-indexed — 20th token after the closing `)` of
/// the comm field).  Returned value is in jiffies since system boot.
#[cfg(target_os = "linux")]
fn read_pid_start_time_linux(pid: u32) -> u64 {
    let content = match std::fs::read_to_string(format!("/proc/{}/stat", pid)) {
        Ok(c) => c,
        Err(_) => return 0,
    };

    // The `comm` field may contain spaces and/or parentheses.  Find the last
    // `)` to safely skip it and start parsing the remaining fields.
    let after_comm = match content.rfind(')') {
        Some(pos) => &content[pos + 1..],
        None => return 0,
    };

    // Fields after comm (split_whitespace handles leading/trailing spaces):
    //   [0]  state       [1]  ppid        [2]  pgrp
    //   [3]  session     [4]  tty_nr      [5]  tpgid
    //   [6]  flags       [7]  minflt      [8]  cminflt
    //   [9]  majflt      [10] cmajflt     [11] utime
    //   [12] stime       [13] cutime      [14] cstime
    //   [15] priority    [16] nice        [17] num_threads
    //   [18] itrealvalue [19] starttime   ← this is what we need
    after_comm
        .split_whitespace()
        .nth(19)
        .and_then(|s| s.parse::<u64>().ok())
        .unwrap_or(0)
}

/// macOS implementation: use `sysctl(KERN_PROC_PID)` to obtain
/// `kinfo_proc.kp_proc.p_starttime` (a `timeval`).
/// Returns microseconds (tv_sec * 1_000_000 + tv_usec).
#[cfg(target_os = "macos")]
fn read_pid_start_time_macos(pid: u32) -> u64 {
    use libc::{c_int, kinfo_proc, CTL_KERN, KERN_PROC, KERN_PROC_PID};
    // SAFETY: zeroed() is valid for a POD C struct; sysctl fills it in-place.
    let mut info: kinfo_proc = unsafe { std::mem::zeroed() };
    let mut size = std::mem::size_of::<kinfo_proc>();
    let mut mib: [c_int; 4] = [CTL_KERN, KERN_PROC, KERN_PROC_PID, pid as c_int];
    // SAFETY: mib is valid, info/size are properly initialised, null pointers
    // for newp/newlen are correct (read-only query).
    let ret = unsafe {
        libc::sysctl(
            mib.as_mut_ptr(),
            4,
            &mut info as *mut _ as *mut libc::c_void,
            &mut size,
            std::ptr::null_mut(),
            0,
        )
    };
    if ret != 0 || size == 0 {
        return 0;
    }
    // SAFETY: `sysctl` succeeded (ret == 0) and wrote `size` bytes into `info`,
    // so `kp_proc.p_starttime` is fully initialized.
    let tv = unsafe { info.kp_proc.p_starttime };
    (tv.tv_sec as u64)
        .saturating_mul(1_000_000)
        .saturating_add(tv.tv_usec as u64)
}

/// Check whether `pid` is still alive **and** is the same process that wrote
/// the presence file.
///
/// When `expected_start_time` is 0 (absent from old presence files), the
/// check falls back to `kill(pid, 0)` only — no PID-reuse protection, but
/// backward compatible with files written before this field was added.
///
/// When `expected_start_time` is non-zero, the check additionally verifies
/// that the current process with `pid` has the same OS start time.  A
/// mismatch means the original node has exited and the OS has recycled the PID
/// for an unrelated process.
fn is_alive(pid: u32, expected_start_time: u64) -> bool {
    if !pid_exists(pid) {
        return false;
    }
    if expected_start_time != 0 {
        let actual = read_pid_start_time(pid);
        // If we could read a start time, it must match.  If read failed
        // (actual == 0) we conservatively trust the kill(0) result.
        if actual != 0 && actual != expected_start_time {
            return false;
        }
    }
    true
}

/// Signal-0 liveness check: returns true iff the OS reports the PID exists.
fn pid_exists(pid: u32) -> bool {
    #[cfg(unix)]
    {
        // SAFETY: kill(pid, 0) sends no signal; only checks if process exists.
        unsafe { libc::kill(pid as i32, 0) == 0 }
    }

    #[cfg(windows)]
    {
        // SAFETY: OpenProcess is safe for existence check; handle closed immediately.
        unsafe {
            let handle = winapi::um::processthreadsapi::OpenProcess(
                winapi::um::winnt::PROCESS_QUERY_LIMITED_INFORMATION,
                0,
                pid,
            );
            if handle.is_null() {
                false
            } else {
                winapi::um::handleapi::CloseHandle(handle);
                true
            }
        }
    }

    #[cfg(not(any(unix, windows)))]
    {
        let _ = pid;
        true
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_node_presence_create() {
        let presence = NodePresence::new(
            "test_node",
            Some("test_scheduler"),
            vec![],
            vec![],
            10,
            Some(60.0),
        );

        assert_eq!(presence.name, "test_node");
        assert_eq!(presence.scheduler, Some("test_scheduler".to_string()));
        assert_eq!(presence.priority, 10);
        assert!(presence.start_time > 0);
    }

    /// A presence record for the current process must show it as alive.
    #[test]
    fn test_is_alive_current_process() {
        let pid = std::process::id();
        let start = read_pid_start_time(pid);
        assert!(
            is_alive(pid, start),
            "current process must be considered alive with its own start time"
        );
    }

    /// A non-existent PID must not be considered alive.
    #[test]
    fn test_is_alive_nonexistent_pid() {
        assert!(!is_alive(999_999_999, 0), "PID 999999999 should not exist");
    }

    /// Simulated PID reuse: the PID exists but the stored start time does not
    /// match the current start time → is_alive must return false.
    #[cfg(target_os = "linux")]
    #[test]
    fn test_pid_reuse_detected_via_start_time_mismatch() {
        let pid = std::process::id();
        let actual_start = read_pid_start_time(pid);
        assert!(
            actual_start > 0,
            "Linux must be able to read /proc/PID/stat for the current process"
        );

        // Simulate PID reuse: presence file was written with a different
        // start time (e.g. the original node started one jiffy earlier).
        let wrong_start = actual_start.wrapping_add(1);
        assert!(
            !is_alive(pid, wrong_start),
            "PID with wrong start time must be detected as a recycled PID"
        );
    }

    /// NodePresence::new should capture a non-zero pid_start_time on Linux.
    #[cfg(target_os = "linux")]
    #[test]
    fn test_new_captures_pid_start_time() {
        let presence = NodePresence::new("start_time_test", None, vec![], vec![], 0, None);
        assert!(
            presence.pid_start_time > 0,
            "pid_start_time must be populated on Linux; got 0"
        );
    }

    /// Presence files must be created with mode 0o600 (owner read/write only).
    ///
    /// This prevents other local users from reading node presence data (PIDs,
    /// topic names, scheduling metadata).
    #[cfg(unix)]
    #[test]
    fn test_presence_file_created_with_mode_0600() {
        use std::os::unix::fs::PermissionsExt;

        // Use a unique name to avoid collisions with parallel test runs.
        let node_name = format!(
            "perm_test_{}",
            std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap_or_default()
                .subsec_nanos()
        );

        let presence = NodePresence::new(&node_name, None, vec![], vec![], 0, None);
        presence
            .write()
            .expect("write() must succeed in test environment");

        let path = NodePresence::presence_path(&node_name);
        let meta = std::fs::metadata(&path).expect("presence file must exist after write()");

        // Mode bits: mask off the file-type bits, keep only permission bits.
        let mode = meta.permissions().mode() & 0o777;
        assert_eq!(
            mode,
            0o600,
            "presence file '{}' must have mode 0600, got {:03o}",
            path.display(),
            mode
        );

        // Clean up.
        let _ = NodePresence::remove(&node_name);
    }

    /// The nodes directory must be created with mode 0o700 (owner only).
    #[cfg(unix)]
    #[test]
    fn test_presence_dir_created_with_mode_0700() {
        use std::os::unix::fs::PermissionsExt;

        // Write a presence file to ensure the directory exists.
        let node_name = format!(
            "dirperm_test_{}",
            std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap_or_default()
                .subsec_nanos()
        );
        let presence = NodePresence::new(&node_name, None, vec![], vec![], 0, None);
        presence.write().expect("write() must succeed");

        let dir = shm_nodes_dir();
        let meta = std::fs::metadata(&dir).expect("nodes directory must exist after write()");

        let mode = meta.permissions().mode() & 0o777;
        assert_eq!(
            mode,
            0o700,
            "nodes directory '{}' must have mode 0700, got {:03o}",
            dir.display(),
            mode
        );

        let _ = NodePresence::remove(&node_name);
    }
}
