//! Live test runtime that writes real SHM artefacts for acceptance testing.

use super::fixtures::TestNodeConfig;

use horus_core::memory::{shm_base_dir, shm_topics_dir};
use horus_core::scheduling::{BlackBoxEvent, BlackBoxRecord};
use horus_core::{NodePresence, TopicMetadata};

use std::fs;
use std::path::PathBuf;
use std::time::{Duration, SystemTime, UNIX_EPOCH};
use horus_core::core::DurationExt;

/// Handle for a single simulated node.
#[derive(Debug)]
#[allow(dead_code)]
struct TestNodeHandle {
    /// Node name (used for log messages and cleanup).
    name: String,
    /// Path to the presence file on disk.
    presence_path: PathBuf,
}

/// Live test runtime that creates real presence files, SHM topic files,
/// and blackbox WAL entries.
///
/// All artefacts are tracked and removed when the runtime is dropped.
/// Cleanup is panic-safe: [`Drop`] catches panics in sub-operations.
pub struct HorusTestRuntime {
    nodes: Vec<TestNodeHandle>,
    /// SHM topic files created by [`add_topic`].
    shm_files: Vec<PathBuf>,
    /// Presence files created by [`add_node`] (redundant with node handles,
    /// but kept for the explicit cleanup contract).
    presence_files: Vec<PathBuf>,
    /// Optional blackbox WAL directory (created lazily).
    blackbox_dir: Option<PathBuf>,
}

impl HorusTestRuntime {
    /// Create a new, empty test runtime.
    pub fn new() -> Self {
        Self {
            nodes: Vec::new(),
            shm_files: Vec::new(),
            presence_files: Vec::new(),
            blackbox_dir: None,
        }
    }

    // ── Node management ─────────────────────────────────────────────────

    /// Add a simulated node by writing a presence file.
    ///
    /// The presence file is written to the same directory that
    /// `NodePresence::read_all()` scans, using the current process's PID
    /// so the liveness check passes.
    pub fn add_node(&mut self, config: TestNodeConfig) -> &mut Self {
        let nodes_dir = shm_base_dir().join("nodes");
        fs::create_dir_all(&nodes_dir).expect("failed to create nodes dir");

        let publishers: Vec<TopicMetadata> = config
            .publishers
            .iter()
            .map(|(topic, typ)| TopicMetadata {
                topic_name: topic.clone(),
                type_name: typ.clone(),
            })
            .collect();

        let subscribers: Vec<TopicMetadata> = config
            .subscribers
            .iter()
            .map(|(topic, typ)| TopicMetadata {
                topic_name: topic.clone(),
                type_name: typ.clone(),
            })
            .collect();

        // Build a NodePresence-compatible JSON manually so we control
        // every field (including pid_start_time for the current process).
        let presence = serde_json::json!({
            "name": config.name,
            "pid": config.pid,
            "scheduler": config.scheduler,
            "publishers": publishers,
            "subscribers": subscribers,
            "start_time": SystemTime::now()
                .duration_since(UNIX_EPOCH)
                .unwrap_or_default()
                .as_secs(),
            "priority": config.priority,
            "rate_hz": config.rate_hz,
            "pid_start_time": read_pid_start_time(config.pid),
        });

        let path = nodes_dir.join(format!("{}.json", config.name));
        let json = serde_json::to_string_pretty(&presence).expect("failed to serialise presence");

        // Write with owner-only permissions on Unix (matches real writer).
        #[cfg(unix)]
        {
            use std::io::Write;
            use std::os::unix::fs::OpenOptionsExt;
            let mut file = fs::OpenOptions::new()
                .write(true)
                .create(true)
                .truncate(true)
                .mode(0o600)
                .open(&path)
                .expect("failed to write presence file");
            file.write_all(json.as_bytes())
                .expect("failed to write presence data");
        }
        #[cfg(not(unix))]
        {
            fs::write(&path, &json).expect("failed to write presence file");
        }

        self.presence_files.push(path.clone());
        self.nodes.push(TestNodeHandle {
            name: config.name,
            presence_path: path,
        });

        self
    }

    // ── Topic management ────────────────────────────────────────────────

    /// Create a SHM topic file.
    ///
    /// This writes a zero-filled file of the given `size` under the
    /// topics directory so that `discover_shared_memory()` picks it up.
    pub fn add_topic(&mut self, name: &str, size: usize) -> &mut Self {
        let topics_dir = shm_topics_dir();
        // Topic API topics live under horus_topic/ subdirectory
        let topic_api_dir = topics_dir.join("horus_topic");
        fs::create_dir_all(&topic_api_dir).expect("failed to create topics dir");

        let path = topic_api_dir.join(name);
        fs::write(&path, vec![0u8; size]).expect("failed to write topic file");

        self.shm_files.push(path);
        self
    }

    /// Create a raw SHM topic file at the top level of the topics directory.
    pub fn add_raw_topic(&mut self, name: &str, size: usize) -> &mut Self {
        let topics_dir = shm_topics_dir();
        fs::create_dir_all(&topics_dir).expect("failed to create topics dir");

        let path = topics_dir.join(name);
        fs::write(&path, vec![0u8; size]).expect("failed to write topic file");

        self.shm_files.push(path);
        self
    }

    // ── Blackbox event injection ────────────────────────────────────────

    /// Write a blackbox event to the WAL file.
    ///
    /// The WAL is created under `.horus/blackbox/` in the system temp
    /// directory (not the real project dir) to avoid polluting real data.
    pub fn inject_blackbox_event(&mut self, event: BlackBoxEvent) {
        let dir = self.ensure_blackbox_dir();
        let wal_path = dir.join("blackbox.wal");

        let record = BlackBoxRecord {
            timestamp_us: SystemTime::now()
                .duration_since(UNIX_EPOCH)
                .unwrap_or_default()
                .as_micros() as u64,
            tick: 0,
            event,
        };

        let json = serde_json::to_string(&record).expect("failed to serialise blackbox record");
        use std::io::Write;
        let mut file = fs::OpenOptions::new()
            .create(true)
            .append(true)
            .open(&wal_path)
            .expect("failed to open blackbox WAL");
        writeln!(file, "{}", json).expect("failed to write blackbox WAL entry");
    }

    /// Inject a convenience "custom" blackbox event with a string message.
    pub fn inject_blackbox_custom(&mut self, category: &str, message: &str) {
        self.inject_blackbox_event(BlackBoxEvent::Custom {
            category: category.to_string(),
            message: message.to_string(),
        });
    }

    // ── Log injection ───────────────────────────────────────────────────

    /// Push a log entry into the global shared-memory log buffer.
    ///
    /// Uses `horus_core::core::log_buffer::GLOBAL_LOG_BUFFER` directly.
    pub fn inject_log(&self, node: &str, severity: &str, message: &str) {
        use horus_core::core::log_buffer::{LogEntry, LogType, GLOBAL_LOG_BUFFER};

        let log_type = match severity.to_lowercase().as_str() {
            "error" => LogType::Error,
            "warning" | "warn" => LogType::Warning,
            "debug" => LogType::Debug,
            "publish" | "pub" => LogType::Publish,
            "subscribe" | "sub" => LogType::Subscribe,
            _ => LogType::Info,
        };

        let entry = LogEntry {
            timestamp: chrono::Utc::now().to_rfc3339(),
            tick_number: 0,
            node_name: node.to_string(),
            log_type,
            topic: None,
            message: message.to_string(),
            tick_us: 0,
            ipc_ns: 0,
        };

        GLOBAL_LOG_BUFFER.push(entry);
    }

    // ── Discovery cache refresh ─────────────────────────────────────────

    /// Force the discovery cache to treat its data as stale so the next
    /// `discover_nodes()` / `discover_shared_memory()` call re-scans.
    #[allow(dead_code)]
    pub fn refresh_discovery(&self) {
        // The cache is a module-private lazy_static in horus_manager.
        // We cannot access it directly from an integration test.  Instead
        // we simply wait long enough for the cache TTL (250 ms) to expire.
        std::thread::sleep(300_u64.ms());
    }

    // ── Readiness ───────────────────────────────────────────────────────

    /// Wait until all added nodes are discoverable via `NodePresence::read_all()`.
    ///
    /// Returns `true` if all nodes appeared before `timeout`, `false` otherwise.
    pub fn wait_ready(&self, timeout: Duration) -> bool {
        let deadline = std::time::Instant::now() + timeout;
        let expected_names: Vec<&str> = self.nodes.iter().map(|n| n.name.as_str()).collect();

        if expected_names.is_empty() {
            return true;
        }

        while std::time::Instant::now() < deadline {
            let presences = NodePresence::read_all();
            let found_all = expected_names
                .iter()
                .all(|name| presences.iter().any(|p| p.name() == *name));
            if found_all {
                return true;
            }
            std::thread::sleep(50_u64.ms());
        }

        false
    }

    /// Return the names of all nodes added so far.
    pub fn node_names(&self) -> Vec<&str> {
        self.nodes.iter().map(|n| n.name.as_str()).collect()
    }

    /// Return the paths of all presence files.
    pub fn presence_paths(&self) -> &[PathBuf] {
        &self.presence_files
    }

    /// Return the paths of all SHM topic files.
    pub fn topic_paths(&self) -> &[PathBuf] {
        &self.shm_files
    }

    /// Return the blackbox directory (if any events have been injected).
    pub fn blackbox_dir(&self) -> Option<&PathBuf> {
        self.blackbox_dir.as_ref()
    }

    // ── Private helpers ─────────────────────────────────────────────────

    fn ensure_blackbox_dir(&mut self) -> PathBuf {
        if let Some(ref dir) = self.blackbox_dir {
            return dir.clone();
        }
        let dir = std::env::temp_dir().join(format!(
            "horus_test_blackbox_{}_{}",
            std::process::id(),
            SystemTime::now()
                .duration_since(UNIX_EPOCH)
                .unwrap_or_default()
                .subsec_nanos()
        ));
        fs::create_dir_all(&dir).expect("failed to create blackbox dir");
        self.blackbox_dir = Some(dir.clone());
        dir
    }
}

impl Drop for HorusTestRuntime {
    fn drop(&mut self) {
        // Remove presence files first (most important for cleanup).
        for path in &self.presence_files {
            if let Err(e) = fs::remove_file(path) {
                // Don't panic in drop — just warn.
                eprintln!(
                    "[HorusTestRuntime] warning: failed to remove presence file {}: {}",
                    path.display(),
                    e
                );
            }
        }

        // Remove SHM topic files.
        for path in &self.shm_files {
            if let Err(e) = fs::remove_file(path) {
                eprintln!(
                    "[HorusTestRuntime] warning: failed to remove SHM file {}: {}",
                    path.display(),
                    e
                );
            }
        }

        // Remove blackbox directory.
        if let Some(ref dir) = self.blackbox_dir {
            if let Err(e) = fs::remove_dir_all(dir) {
                eprintln!(
                    "[HorusTestRuntime] warning: failed to remove blackbox dir {}: {}",
                    dir.display(),
                    e
                );
            }
        }
    }
}

// ── PID start-time helper (mirrors horus_core::core::presence logic) ────────

/// Read the OS-level start time for a given PID.
///
/// On Linux this reads `/proc/<pid>/stat` field 22 (starttime in jiffies).
/// Returns 0 on failure or on non-Linux platforms.
fn read_pid_start_time(pid: u32) -> u64 {
    #[cfg(target_os = "linux")]
    {
        let content = match fs::read_to_string(format!("/proc/{}/stat", pid)) {
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
    #[cfg(not(target_os = "linux"))]
    {
        let _ = pid;
        0
    }
}
