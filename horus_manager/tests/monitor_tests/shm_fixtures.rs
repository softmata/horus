//! Shared memory fixture management for integration-level monitor tests.
//!
//! These functions create and clean up real files in `/dev/shm/horus_<ns>/`
//! (or the platform-appropriate equivalent) so that the discovery functions
//! (`discover_nodes`, `discover_shared_memory`) return predictable data.
//!
//! # Safety
//!
//! All test fixtures use a `__test_` prefix to avoid collisions with real
//! HORUS nodes.  The [`cleanup_test_shm`] function removes only files that
//! match this prefix.
//!
//! # Example
//!
//! ```ignore
//! let guard = TestShmGuard::new();
//! guard.create_presence("test_lidar", std::process::id());
//! guard.create_topic("test_camera_rgb", 65536);
//! // ... run tests that call discover_nodes() / discover_shared_memory() ...
//! // guard is dropped → all __test_ files are cleaned up
//! ```

use horus_core::memory::{shm_base_dir, shm_topics_dir};
use std::fs;
use std::path::PathBuf;

/// Get the SHM nodes directory.
///
/// `shm_nodes_dir()` is `pub(crate)` inside `horus_core`, so we reconstruct
/// the path from the publicly available `shm_base_dir()`.
fn shm_nodes_dir() -> PathBuf {
    shm_base_dir().join("nodes")
}

/// Prefix for all test SHM fixtures to avoid collisions with real nodes.
const TEST_PREFIX: &str = "__test_";

// ─── Standalone functions ───────────────────────────────────────────────────

/// Create a test node presence file in `/dev/shm/horus_<ns>/nodes/`.
///
/// The presence file is a JSON document matching the `NodePresence` schema
/// that `discover_nodes()` reads.
///
/// # Arguments
///
/// * `name` — Logical node name (will be prefixed with `__test_`).
/// * `pid`  — Process ID to write into the presence file.  Use
///   `std::process::id()` so the liveness check passes.
///
/// Returns the full path to the created file.
pub fn create_test_presence(name: &str, pid: u32) -> PathBuf {
    let dir = shm_nodes_dir();
    fs::create_dir_all(&dir).expect("failed to create SHM nodes directory");

    let full_name = format!("{}{}", TEST_PREFIX, name);
    let path = dir.join(format!("{}.json", full_name));

    let presence_json = serde_json::json!({
        "name": full_name,
        "pid": pid,
        "scheduler": null,
        "publishers": [],
        "subscribers": [],
        "start_time": std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap_or_default()
            .as_secs(),
        "priority": 10,
        "rate_hz": 30.0,
        "pid_start_time": read_pid_start_time(pid),
    });

    fs::write(&path, serde_json::to_string_pretty(&presence_json).unwrap())
        .unwrap_or_else(|e| panic!("failed to write test presence file {:?}: {}", path, e));

    path
}

/// Create a test presence file with publisher/subscriber metadata.
///
/// More realistic than [`create_test_presence`] — includes topic metadata
/// that `discover_nodes()` uses to build the pub/sub graph.
pub fn create_test_presence_with_topics(
    name: &str,
    pid: u32,
    publishers: &[(&str, &str)],
    subscribers: &[(&str, &str)],
) -> PathBuf {
    let dir = shm_nodes_dir();
    fs::create_dir_all(&dir).expect("failed to create SHM nodes directory");

    let full_name = format!("{}{}", TEST_PREFIX, name);
    let path = dir.join(format!("{}.json", full_name));

    let pubs: Vec<serde_json::Value> = publishers
        .iter()
        .map(|(topic, type_name)| {
            serde_json::json!({
                "topic": topic,
                "type_name": type_name,
            })
        })
        .collect();

    let subs: Vec<serde_json::Value> = subscribers
        .iter()
        .map(|(topic, type_name)| {
            serde_json::json!({
                "topic": topic,
                "type_name": type_name,
            })
        })
        .collect();

    let presence_json = serde_json::json!({
        "name": full_name,
        "pid": pid,
        "scheduler": "test_scheduler",
        "publishers": pubs,
        "subscribers": subs,
        "start_time": std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap_or_default()
            .as_secs(),
        "priority": 10,
        "rate_hz": 30.0,
        "pid_start_time": read_pid_start_time(pid),
    });

    fs::write(&path, serde_json::to_string_pretty(&presence_json).unwrap())
        .unwrap_or_else(|e| panic!("failed to write test presence file {:?}: {}", path, e));

    path
}

/// Create a test shared memory topic file in `/dev/shm/horus_<ns>/topics/`.
///
/// This creates a real file of the given size.  The discovery function reads
/// file metadata (size, modification time) and the file name to populate
/// `SharedMemoryInfo`.
///
/// # Arguments
///
/// * `name` — Topic name (will be prefixed with `__test_`).
/// * `size` — File size in bytes (the file is filled with zeros).
///
/// Returns the full path to the created file.
pub fn create_test_shm_topic(name: &str, size: u64) -> PathBuf {
    let dir = shm_topics_dir();
    fs::create_dir_all(&dir).expect("failed to create SHM topics directory");

    let full_name = format!("{}{}", TEST_PREFIX, name);
    let path = dir.join(&full_name);

    // Create a file of the specified size
    let file = fs::File::create(&path)
        .unwrap_or_else(|e| panic!("failed to create test topic file {:?}: {}", path, e));
    file.set_len(size)
        .unwrap_or_else(|e| panic!("failed to set topic file size {:?}: {}", path, e));

    path
}

/// Remove all test SHM fixtures (files whose names start with `__test_`).
///
/// This cleans up both the nodes and topics directories.  It is safe to call
/// even if no fixtures exist.
pub fn cleanup_test_shm() {
    cleanup_dir_with_prefix(&shm_nodes_dir(), TEST_PREFIX);
    cleanup_dir_with_prefix(&shm_topics_dir(), TEST_PREFIX);
}

/// Remove files in `dir` whose name starts with `prefix`.
fn cleanup_dir_with_prefix(dir: &std::path::Path, prefix: &str) {
    if !dir.exists() {
        return;
    }

    if let Ok(entries) = fs::read_dir(dir) {
        for entry in entries.flatten() {
            let file_name = entry.file_name();
            let name_str = file_name.to_string_lossy();
            if name_str.starts_with(prefix) {
                let _ = fs::remove_file(entry.path());
            }
        }
    }
}

// ─── RAII guard ─────────────────────────────────────────────────────────────

/// RAII guard that automatically cleans up all test SHM fixtures on drop.
///
/// This is the recommended way to manage test fixtures — create one at the
/// start of a test and let it go out of scope at the end.
///
/// ```ignore
/// #[tokio::test]
/// async fn my_test() {
///     let guard = TestShmGuard::new();
///     guard.create_presence("my_node", std::process::id());
///     // ... test code ...
/// } // guard dropped, fixtures cleaned up
/// ```
pub struct TestShmGuard {
    /// Paths to files created by this guard instance (for targeted cleanup).
    created_files: Vec<PathBuf>,
}

impl TestShmGuard {
    /// Create a new guard.  No files are created until you call one of the
    /// `create_*` methods.
    pub fn new() -> Self {
        Self {
            created_files: Vec::new(),
        }
    }

    /// Create a test presence file and track it for cleanup.
    pub fn create_presence(&mut self, name: &str, pid: u32) -> PathBuf {
        let path = create_test_presence(name, pid);
        self.created_files.push(path.clone());
        path
    }

    /// Create a test presence file with topics and track it for cleanup.
    pub fn create_presence_with_topics(
        &mut self,
        name: &str,
        pid: u32,
        publishers: &[(&str, &str)],
        subscribers: &[(&str, &str)],
    ) -> PathBuf {
        let path = create_test_presence_with_topics(name, pid, publishers, subscribers);
        self.created_files.push(path.clone());
        path
    }

    /// Create a test SHM topic file and track it for cleanup.
    pub fn create_topic(&mut self, name: &str, size: u64) -> PathBuf {
        let path = create_test_shm_topic(name, size);
        self.created_files.push(path.clone());
        path
    }

    /// Return the list of files created by this guard.
    pub fn created_files(&self) -> &[PathBuf] {
        &self.created_files
    }
}

impl Drop for TestShmGuard {
    fn drop(&mut self) {
        // Remove only the files this guard created (not all __test_ files,
        // which could belong to a parallel test).
        for path in &self.created_files {
            let _ = fs::remove_file(path);
        }
    }
}

// ─── Internal helpers ───────────────────────────────────────────────────────

/// Read the OS-level process start time for PID-reuse detection.
///
/// Mirrors the logic in `horus_core::core::presence::read_pid_start_time`.
fn read_pid_start_time(pid: u32) -> u64 {
    #[cfg(target_os = "linux")]
    {
        read_pid_start_time_linux(pid)
    }

    #[cfg(not(target_os = "linux"))]
    {
        let _ = pid;
        0
    }
}

#[cfg(target_os = "linux")]
fn read_pid_start_time_linux(pid: u32) -> u64 {
    let content = match std::fs::read_to_string(format!("/proc/{}/stat", pid)) {
        Ok(c) => c,
        Err(_) => return 0,
    };

    let after_comm = match content.rfind(')') {
        Some(pos) => &content[pos + 1..],
        None => return 0,
    };

    after_comm
        .split_whitespace()
        .nth(19)
        .and_then(|s| s.parse::<u64>().ok())
        .unwrap_or(0)
}
