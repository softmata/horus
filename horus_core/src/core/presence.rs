//! Node presence files for monitor detection
//!
//! Each node creates a presence file at startup and removes it at shutdown.
//! Monitor scans this directory to discover all active nodes - like rqt but simpler.
//!
//! Structure: `/dev/shm/horus/nodes/{node_name}.json`

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
    /// Node name
    pub name: String,
    /// Process ID
    pub pid: u32,
    /// Scheduler name (if running under a scheduler)
    pub scheduler: Option<String>,
    /// Topics this node publishes to
    pub publishers: Vec<TopicMetadata>,
    /// Topics this node subscribes to
    pub subscribers: Vec<TopicMetadata>,
    /// Unix timestamp when node started (seconds)
    pub start_time: u64,
    /// Node priority (from scheduler)
    pub priority: u32,
    /// Target tick rate in Hz
    pub rate_hz: Option<f64>,
    /// OS-level process start time for PID-reuse detection.
    ///
    /// On Linux this is the `starttime` field from `/proc/PID/stat` (jiffies
    /// since system boot).  On macOS it is microseconds derived from
    /// `kinfo_proc.kp_proc.p_starttime`.  On other platforms it is 0.
    ///
    /// A value of 0 means "start-time unavailable"; in that case the liveness
    /// check falls back to `kill(pid, 0)` alone (no PID-reuse protection).
    ///
    /// `#[serde(default)]` ensures presence files written before this field
    /// was added deserialise with `pid_start_time = 0` (backward compatible).
    #[serde(default)]
    pub pid_start_time: u64,
}

impl NodePresence {
    /// Create a new presence record
    pub(crate) fn new(
        name: &str,
        scheduler: Option<&str>,
        publishers: Vec<TopicMetadata>,
        subscribers: Vec<TopicMetadata>,
        priority: u32,
        rate_hz: Option<f64>,
    ) -> Self {
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
    pub(crate) fn write(&self) -> std::io::Result<()> {
        let dir = shm_nodes_dir();

        // Create directory hierarchy with owner-only access.
        #[cfg(unix)]
        fs::DirBuilder::new()
            .recursive(true)
            .mode(0o700)
            .create(&dir)?;
        #[cfg(not(unix))]
        fs::create_dir_all(&dir)?;

        let path = Self::presence_path(&self.name);
        let json = serde_json::to_string_pretty(self)
            .map_err(|e| std::io::Error::new(std::io::ErrorKind::InvalidData, e))?;

        // Write with owner-only permissions: other local users must not read
        // presence data (contains PIDs, topic names, and timing information).
        #[cfg(unix)]
        {
            use std::io::Write;
            let mut file = fs::OpenOptions::new()
                .write(true)
                .create(true)
                .truncate(true)
                .mode(0o600)
                .open(&path)?;
            file.write_all(json.as_bytes())
        }
        #[cfg(not(unix))]
        fs::write(&path, json)
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
                    if let Ok(content) = fs::read_to_string(&path) {
                        if let Ok(presence) = serde_json::from_str::<NodePresence>(&content) {
                            // Verify process is still alive and is the same process
                            // that wrote this file (not a recycled PID).
                            if is_alive(presence.pid, presence.pid_start_time) {
                                nodes.push(presence);
                            } else {
                                // Clean up stale presence file
                                let _ = fs::remove_file(&path);
                            }
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
