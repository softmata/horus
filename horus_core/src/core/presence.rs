//! Node presence files for monitor detection
//!
//! Each node creates a presence file at startup and removes it at shutdown.
//! Monitor scans this directory to discover all active nodes - like rqt but simpler.
//!
//! Structure: `/dev/shm/horus/nodes/{node_name}.json`

use crate::core::node::TopicMetadata;
use crate::memory::shm_nodes_dir;
use serde::{Deserialize, Serialize};
use std::fs;
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
}

impl NodePresence {
    /// Create a new presence record
    pub fn new(
        name: &str,
        scheduler: Option<&str>,
        publishers: Vec<TopicMetadata>,
        subscribers: Vec<TopicMetadata>,
        priority: u32,
        rate_hz: Option<f64>,
    ) -> Self {
        Self {
            name: name.to_string(),
            pid: std::process::id(),
            scheduler: scheduler.map(|s| s.to_string()),
            publishers,
            subscribers,
            start_time: SystemTime::now()
                .duration_since(UNIX_EPOCH)
                .unwrap_or_default()
                .as_secs(),
            priority,
            rate_hz,
        }
    }

    /// Get the path for this node's presence file
    fn presence_path(node_name: &str) -> PathBuf {
        shm_nodes_dir().join(format!("{}.json", node_name))
    }

    /// Write presence file to shared memory (called at node start)
    pub fn write(&self) -> std::io::Result<()> {
        let dir = shm_nodes_dir();
        fs::create_dir_all(&dir)?;

        let path = Self::presence_path(&self.name);
        let json = serde_json::to_string_pretty(self)
            .map_err(|e| std::io::Error::new(std::io::ErrorKind::InvalidData, e))?;

        fs::write(&path, json)
    }

    /// Remove presence file from shared memory (called at node shutdown)
    pub fn remove(node_name: &str) -> std::io::Result<()> {
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
                            // Verify process is still alive
                            if process_exists(presence.pid) {
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

/// Check if a process exists
fn process_exists(pid: u32) -> bool {
    #[cfg(unix)]
    {
        // SAFETY: kill(pid, 0) sends no signal; only checks if process exists. No memory is accessed.
        unsafe { libc::kill(pid as i32, 0) == 0 }
    }

    #[cfg(windows)]
    {
        use std::ptr::null_mut;
        // SAFETY: OpenProcess with PROCESS_QUERY_LIMITED_INFORMATION is safe for existence check.
        // Handle is closed immediately after.
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
        // Fallback: assume process exists
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

    #[test]
    fn test_process_exists() {
        // Current process should exist
        assert!(process_exists(std::process::id()));

        // PID 0 should not exist (or be special)
        // PID 999999999 almost certainly doesn't exist
        assert!(!process_exists(999999999));
    }
}
