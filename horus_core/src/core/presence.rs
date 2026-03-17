//! Node presence files for monitor detection
//!
//! Each node creates a presence file at startup and removes it at shutdown.
//! Monitor scans this directory to discover all active nodes — like rqt but simpler.
//!
//! ## Structure
//!
//! Presence files live at `shm_nodes_dir()/{node_name}.json` (platform-specific path via horus_sys).
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
    /// # Errors
    ///
    /// Returns [`ValidationError::InvalidFormat`] if `name` is empty, too long,
    /// or contains characters other than `a-z A-Z 0-9 _ - .`.
    pub(crate) fn new(
        name: &str,
        scheduler: Option<&str>,
        publishers: Vec<TopicMetadata>,
        subscribers: Vec<TopicMetadata>,
        priority: u32,
        rate_hz: Option<f64>,
    ) -> crate::error::HorusResult<Self> {
        validate_node_name(name).map_err(|_| {
            crate::error::HorusError::InvalidInput(crate::error::ValidationError::InvalidFormat {
                field: "node_name".into(),
                expected_format:
                    "1-255 chars, alphanumeric + underscore/hyphen/dot (a-z A-Z 0-9 _ - .)".into(),
                actual: name.into(),
            })
        })?;
        let pid = std::process::id();
        Ok(Self {
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
        })
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
                    if existing.pid != self.pid && is_alive(existing.pid, existing.pid_start_time) {
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
        horus_sys::fs::create_dir_secure(&dir)
            .map_err(|e| std::io::Error::new(std::io::ErrorKind::Other, e))?;

        let dest = Self::presence_path(&self.name);
        let json = serde_json::to_string_pretty(self)
            .map_err(|e| std::io::Error::new(std::io::ErrorKind::InvalidData, e))?;

        // Atomic write: write to a temporary file then rename, so readers
        // never see a partially-written presence file.
        let tmp_path = dest.with_extension("json.tmp");

        {
            use std::io::Write;
            let mut file = horus_sys::fs::open_private(&tmp_path)
                .map_err(|e| std::io::Error::new(std::io::ErrorKind::Other, e))?;
            file.write_all(json.as_bytes())?;
            file.flush()?;
        }

        fs::rename(&tmp_path, &dest)
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
/// Delegates to [`horus_sys::process::pid_start_time`].
fn read_pid_start_time(pid: u32) -> u64 {
    horus_sys::process::pid_start_time(pid)
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

/// Liveness check: returns true iff the OS reports the PID exists.
fn pid_exists(pid: u32) -> bool {
    horus_sys::process::ProcessHandle::from_pid(pid).is_alive()
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
        )
        .expect("valid node name should succeed");

        assert_eq!(presence.name, "test_node");
        assert_eq!(presence.scheduler, Some("test_scheduler".to_string()));
        assert_eq!(presence.priority, 10);
        assert!(presence.start_time > 0);
    }

    #[test]
    fn test_invalid_node_name_returns_error() {
        let result = NodePresence::new("invalid/name", None, vec![], vec![], 0, None);
        assert!(result.is_err(), "node name with '/' should fail");
        let err = result.unwrap_err();
        let msg = err.to_string();
        assert!(
            msg.contains("Invalid format"),
            "should be InvalidFormat error: {}",
            msg
        );
        assert!(err.help().is_some(), "InvalidFormat should have help text");
    }

    #[test]
    fn test_empty_node_name_returns_error() {
        let result = NodePresence::new("", None, vec![], vec![], 0, None);
        assert!(result.is_err(), "empty node name should fail");
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
        let presence = NodePresence::new("start_time_test", None, vec![], vec![], 0, None)
            .expect("valid name");
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

        let presence =
            NodePresence::new(&node_name, None, vec![], vec![], 0, None).expect("valid name");
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
        let presence =
            NodePresence::new(&node_name, None, vec![], vec![], 0, None).expect("valid name");
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

    // ── Write/Read roundtrip ────────────────────────────────────

    #[test]
    fn test_write_then_read_roundtrip() {
        let name = format!("rt_test_{}", std::process::id());
        let presence = NodePresence::new(
            &name,
            Some("sched_1"),
            vec![TopicMetadata { topic_name: "cmd_vel".into(), type_name: "CmdVel".into() }],
            vec![TopicMetadata { topic_name: "odom".into(), type_name: "Odometry".into() }],
            42,
            Some(100.0),
        ).unwrap();
        presence.write().unwrap();

        let read_back = NodePresence::read(&name);
        assert!(read_back.is_some(), "read should find written presence");
        let p = read_back.unwrap();
        assert_eq!(p.name(), &name);
        assert_eq!(p.scheduler(), Some("sched_1"));
        assert_eq!(p.priority(), 42);
        assert_eq!(p.publishers().len(), 1);
        assert_eq!(p.publishers()[0].topic_name, "cmd_vel");
        assert_eq!(p.subscribers().len(), 1);
        assert_eq!(p.subscribers()[0].topic_name, "odom");

        let _ = NodePresence::remove(&name);
    }

    #[test]
    fn test_read_nonexistent_returns_none() {
        assert!(NodePresence::read("nonexistent_node_xyz_12345").is_none());
    }

    #[test]
    fn test_remove_deletes_file() {
        let name = format!("rm_test_{}", std::process::id());
        let presence = NodePresence::new(&name, None, vec![], vec![], 0, None).unwrap();
        presence.write().unwrap();
        assert!(NodePresence::read(&name).is_some());

        NodePresence::remove(&name).unwrap();
        assert!(NodePresence::read(&name).is_none(), "should be gone after remove");
    }

    #[test]
    fn test_remove_nonexistent_is_ok() {
        // Should not panic
        let _ = NodePresence::remove("nonexistent_remove_test");
    }

    // ── Getters ─────────────────────────────────────────────────

    #[test]
    fn test_all_getters() {
        let presence = NodePresence::new(
            "getter_test",
            Some("my_sched"),
            vec![TopicMetadata { topic_name: "t1".into(), type_name: "T1".into() }],
            vec![],
            99,
            Some(50.0),
        ).unwrap();

        assert_eq!(presence.name(), "getter_test");
        assert_eq!(presence.pid(), std::process::id());
        assert_eq!(presence.scheduler(), Some("my_sched"));
        assert_eq!(presence.publishers().len(), 1);
        assert!(presence.subscribers().is_empty());
        assert!(presence.start_time() > 0);
        assert_eq!(presence.priority(), 99);
        assert_eq!(presence.rate_hz(), Some(50.0));
        assert_eq!(presence.tick_count(), 0);
        assert_eq!(presence.error_count(), 0);
        assert!(presence.services().is_empty());
        assert!(presence.actions().is_empty());
    }

    // ── Validation edge cases ───────────────────────────────────

    #[test]
    fn test_validate_node_name_valid_chars() {
        assert!(validate_node_name("my_node").is_ok());
        assert!(validate_node_name("sensor-1").is_ok());
        assert!(validate_node_name("arm.joint0").is_ok());
        assert!(validate_node_name("A").is_ok());
    }

    #[test]
    fn test_validate_node_name_rejects_path_traversal() {
        assert!(validate_node_name("..").is_err());
        assert!(validate_node_name("../etc/passwd").is_err());
        assert!(validate_node_name("a\\b").is_err());
    }

    #[test]
    fn test_validate_node_name_rejects_spaces() {
        assert!(validate_node_name("my node").is_err());
    }

    #[test]
    fn test_validate_node_name_rejects_long_name() {
        let long = "x".repeat(256);
        assert!(validate_node_name(&long).is_err());
    }

    // ── read_all ────────────────────────────────────────────────

    #[test]
    fn test_read_all_includes_live_nodes() {
        let name = format!("readall_test_{}", std::process::id());
        let presence = NodePresence::new(&name, None, vec![], vec![], 0, None).unwrap();
        presence.write().unwrap();

        let all = NodePresence::read_all();
        assert!(
            all.iter().any(|p| p.name() == name),
            "read_all should include our live node"
        );

        let _ = NodePresence::remove(&name);
    }
}
