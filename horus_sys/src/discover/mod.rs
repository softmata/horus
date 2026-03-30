//! Node and process discovery — find running processes and their stats.
//!
//! Provides [`ProcessInfo`] with CPU/memory/cmdline for any process:
//! - **Linux**: `/proc/{pid}/stat`, `/proc/{pid}/cmdline`, `/proc/{pid}/cwd`
//! - **macOS**: `sysctl(KERN_PROC)` + `libproc`
//! - **Windows**: `CreateToolhelp32Snapshot` + `Process32First/Next`

use std::time::Duration;

#[cfg(target_os = "linux")]
mod linux;
#[cfg(target_os = "macos")]
mod macos;
#[cfg(target_os = "windows")]
mod windows;

// ── ProcessInfo ──────────────────────────────────────────────────────────

/// Cross-platform process information.
#[derive(Debug, Clone, Default)]
pub struct ProcessInfo {
    /// Process ID.
    pub pid: u32,
    /// Command line arguments (space-separated).
    pub cmdline: String,
    /// Working directory path.
    pub working_dir: String,
    /// CPU usage as a percentage (0.0–100.0+).
    pub cpu_percent: f32,
    /// Resident memory in KB.
    pub memory_kb: u64,
    /// Human-readable start time / uptime string (e.g. "5m ago", "2h").
    pub start_time: String,
}

// ── Public API ───────────────────────────────────────────────────────────

/// Get detailed information about a process by PID.
///
/// Returns `ProcessInfo` with cmdline, working directory, CPU%, memory, and
/// start time. Fields may be empty/zero if the OS doesn't expose them or
/// the process has exited.
pub fn process_info(pid: u32) -> anyhow::Result<ProcessInfo> {
    #[cfg(target_os = "linux")]
    {
        linux::get_process_info(pid)
    }
    #[cfg(target_os = "macos")]
    {
        macos::get_process_info(pid)
    }
    #[cfg(target_os = "windows")]
    {
        windows::get_process_info(pid)
    }
    #[cfg(not(any(target_os = "linux", target_os = "macos", target_os = "windows")))]
    {
        let _ = pid;
        Ok(ProcessInfo::default())
    }
}

/// Check if a process is alive (delegates to [`crate::process::ProcessHandle`]).
pub fn is_process_alive(pid: u32) -> bool {
    crate::process::ProcessHandle::from_pid(pid).is_alive()
}

/// List all process IDs on the system.
///
/// - **Linux**: reads `/proc/` directory entries
/// - **macOS**: uses `sysctl(KERN_PROC_ALL)`
/// - **Windows**: uses `CreateToolhelp32Snapshot`
pub fn list_pids() -> Vec<u32> {
    #[cfg(target_os = "linux")]
    {
        linux::list_pids()
    }
    #[cfg(target_os = "macos")]
    {
        macos::list_pids()
    }
    #[cfg(target_os = "windows")]
    {
        windows::list_pids()
    }
    #[cfg(not(any(target_os = "linux", target_os = "macos", target_os = "windows")))]
    {
        Vec::new()
    }
}

/// Format a [`Duration`] as a human-readable string (e.g. "5s", "3m", "2h", "1d").
pub fn format_duration(duration: Duration) -> String {
    let total_secs = duration.as_secs();
    if total_secs < 60 {
        format!("{}s", total_secs)
    } else if total_secs < 3600 {
        format!("{}m", total_secs / 60)
    } else if total_secs < 86400 {
        format!("{}h", total_secs / 3600)
    } else {
        format!("{}d", total_secs / 86400)
    }
}

// ── Node & Topic Discovery ─────────────────────────────────────────────

/// A topic reference from a node's presence file (publisher or subscriber entry).
#[derive(Debug, Clone, serde::Deserialize)]
pub struct TopicRef {
    /// Topic name (e.g., "cmd_vel", "lidar_scan").
    /// Accepts both "topic" and "topic_name" (horus_core writes "topic_name").
    #[serde(default, alias = "topic_name")]
    pub topic: String,
    /// Message type name (e.g., "CmdVel", "LaserScan").
    #[serde(default)]
    pub type_name: String,
}

/// Discovered node information, combining presence file data with process enrichment.
#[derive(Debug, Clone)]
pub struct NodeInfo {
    pub name: String,
    pub pid: u32,
    pub scheduler: Option<String>,
    pub publishers: Vec<TopicRef>,
    pub subscribers: Vec<TopicRef>,
    pub priority: u32,
    pub rate_hz: Option<f64>,
    pub health: String,
    pub tick_count: u64,
    pub error_count: u32,
    pub services: Vec<String>,
    pub actions: Vec<String>,
    // Enriched from process_info():
    pub cpu_percent: f32,
    pub memory_kb: u64,
    pub cmdline: String,
    pub working_dir: String,
    pub start_time: String,
}

/// JSON format of node presence files (matches horus_core::core::presence::NodePresence).
#[derive(serde::Deserialize)]
#[allow(dead_code)] // Fields deserialized from JSON but not all read directly
struct PresenceFile {
    #[serde(default)]
    name: String,
    #[serde(default)]
    pid: u32,
    #[serde(default)]
    scheduler: Option<String>,
    #[serde(default)]
    publishers: Vec<TopicRef>,
    #[serde(default)]
    subscribers: Vec<TopicRef>,
    #[serde(default)]
    priority: u32,
    #[serde(default)]
    rate_hz: Option<f64>,
    #[serde(default)]
    pid_start_time: u64,
    #[serde(default)]
    health_status: Option<String>,
    #[serde(default)]
    tick_count: u64,
    #[serde(default)]
    error_count: u32,
    #[serde(default)]
    services: Vec<String>,
    #[serde(default)]
    actions: Vec<String>,
}

/// Find all running HORUS nodes by reading presence files from `shm_nodes_dir()`.
///
/// Cross-platform: presence files are regular JSON files on all platforms.
/// Each node is validated for liveness (PID check) and enriched with CPU/memory
/// via the platform-specific `process_info()` backend.
pub fn find_nodes() -> Vec<NodeInfo> {
    let nodes_dir = crate::shm::shm_nodes_dir();
    let entries = match std::fs::read_dir(&nodes_dir) {
        Ok(e) => e,
        Err(_) => return Vec::new(),
    };

    let mut nodes = Vec::new();
    for entry in entries.flatten() {
        let path = entry.path();
        if path.extension().map_or(true, |e| e != "json") {
            continue;
        }
        let content = match std::fs::read_to_string(&path) {
            Ok(c) => c,
            Err(_) => continue,
        };
        let pf: PresenceFile = match serde_json::from_str(&content) {
            Ok(p) => p,
            Err(_) => continue,
        };

        // Skip dead nodes
        if pf.pid == 0 || !is_process_alive(pf.pid) {
            continue;
        }

        // Enrich with process info (CPU, memory, cmdline, working_dir)
        let (cpu, mem, cmd, wd, st) = match process_info(pf.pid) {
            Ok(pi) => (
                pi.cpu_percent,
                pi.memory_kb,
                pi.cmdline,
                pi.working_dir,
                pi.start_time,
            ),
            Err(_) => (0.0, 0, String::new(), String::new(), String::new()),
        };

        nodes.push(NodeInfo {
            name: pf.name,
            pid: pf.pid,
            scheduler: pf.scheduler,
            publishers: pf.publishers,
            subscribers: pf.subscribers,
            priority: pf.priority,
            rate_hz: pf.rate_hz,
            health: pf.health_status.unwrap_or_else(|| "unknown".into()),
            tick_count: pf.tick_count,
            error_count: pf.error_count,
            services: pf.services,
            actions: pf.actions,
            cpu_percent: cpu,
            memory_kb: mem,
            cmdline: cmd,
            working_dir: wd,
            start_time: st,
        });
    }

    nodes
}

/// Discovered topic information, combining SHM metadata with presence data.
#[derive(Debug, Clone)]
pub struct DiscoveredTopic {
    pub name: String,
    pub size: usize,
    pub creator_pid: u32,
    pub is_alive: bool,
    pub publishers: Vec<String>,
    pub subscribers: Vec<String>,
    pub message_type: Option<String>,
    pub created_at: u64,
}

/// Find all active HORUS topics by scanning SHM metadata and presence files.
///
/// - **Linux**: Scans `shm_topics_dir()` for actual SHM files + `.meta` files
/// - **macOS/Windows**: Scans `shm_topics_dir()` for `.meta` files only
/// - **All platforms**: Merges publisher/subscriber info from node presence files
pub fn find_topics() -> Vec<DiscoveredTopic> {
    let mut topics: std::collections::HashMap<String, DiscoveredTopic> =
        std::collections::HashMap::new();

    let topics_dir = crate::shm::shm_topics_dir();

    // Source 1: Scan filesystem for SHM files and .meta files
    if let Ok(entries) = std::fs::read_dir(&topics_dir) {
        for entry in entries.flatten() {
            let path = entry.path();
            if !path.is_file() {
                continue;
            }

            // .meta files: cross-platform topic metadata
            if path.extension().is_some_and(|e| e == "meta") {
                if let Ok(content) = std::fs::read_to_string(&path) {
                    if let Ok(meta) = serde_json::from_str::<crate::shm::TopicMeta>(&content) {
                        let alive = is_process_alive(meta.creator_pid);
                        topics.entry(meta.name.clone()).or_insert(DiscoveredTopic {
                            name: meta.name,
                            size: meta.size,
                            creator_pid: meta.creator_pid,
                            is_alive: alive,
                            publishers: Vec::new(),
                            subscribers: Vec::new(),
                            message_type: None,
                            created_at: meta.created_at,
                        });
                    }
                }
                continue;
            }

            // On Linux: raw SHM files are actual shared memory regions.
            // Skip .meta files (handled above) but include files with dots
            // in topic names (e.g., "sensor.imu", "robot.state").
            #[cfg(target_os = "linux")]
            {
                if let Some(name) = path.file_name().and_then(|n| n.to_str()) {
                    if !name.ends_with(".meta") {
                        if let Ok(meta) = std::fs::metadata(&path) {
                            topics.entry(name.to_string()).or_insert(DiscoveredTopic {
                                name: name.to_string(),
                                size: meta.len() as usize,
                                creator_pid: 0,
                                is_alive: !crate::shm::is_shm_file_stale(&path),
                                publishers: Vec::new(),
                                subscribers: Vec::new(),
                                message_type: None,
                                created_at: 0,
                            });
                        }
                    }
                }
            }
        }
    }

    // Also scan subdirectories (horus_links/, horus_topic/)
    for subdir_name in &["horus_links", "horus_topic"] {
        let subdir = topics_dir.join(subdir_name);
        if let Ok(entries) = std::fs::read_dir(&subdir) {
            for entry in entries.flatten() {
                let path = entry.path();
                if path.extension().is_some_and(|e| e == "meta") {
                    if let Ok(content) = std::fs::read_to_string(&path) {
                        if let Ok(meta) = serde_json::from_str::<crate::shm::TopicMeta>(&content) {
                            let alive = is_process_alive(meta.creator_pid);
                            topics.entry(meta.name.clone()).or_insert(DiscoveredTopic {
                                name: meta.name,
                                size: meta.size,
                                creator_pid: meta.creator_pid,
                                is_alive: alive,
                                publishers: Vec::new(),
                                subscribers: Vec::new(),
                                message_type: None,
                                created_at: meta.created_at,
                            });
                        }
                    }
                }
            }
        }
    }

    // Source 2: Merge pub/sub from node presence files.
    // Topics that appear in presence but have no .meta file are added as
    // presence-only topics (e.g., network topics, topics on other hosts).
    let nodes = find_nodes();
    for node in &nodes {
        for pub_topic in &node.publishers {
            let ti = topics.entry(pub_topic.topic.clone()).or_insert_with(|| {
                DiscoveredTopic {
                    name: pub_topic.topic.clone(),
                    size: 0,
                    creator_pid: 0,
                    is_alive: true, // presence exists = node is alive
                    publishers: Vec::new(),
                    subscribers: Vec::new(),
                    message_type: None,
                    created_at: 0,
                }
            });
            if !ti.publishers.contains(&node.name) {
                ti.publishers.push(node.name.clone());
            }
            if ti.message_type.is_none() && !pub_topic.type_name.is_empty() {
                ti.message_type = Some(pub_topic.type_name.clone());
            }
        }
        for sub_topic in &node.subscribers {
            let ti = topics
                .entry(sub_topic.topic.clone())
                .or_insert_with(|| DiscoveredTopic {
                    name: sub_topic.topic.clone(),
                    size: 0,
                    creator_pid: 0,
                    is_alive: true,
                    publishers: Vec::new(),
                    subscribers: Vec::new(),
                    message_type: None,
                    created_at: 0,
                });
            if !ti.subscribers.contains(&node.name) {
                ti.subscribers.push(node.name.clone());
            }
        }
    }

    let mut result: Vec<_> = topics.into_values().collect();
    result.sort_by(|a, b| a.name.cmp(&b.name));
    result
}

// ── Tests ────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn process_info_current_process() {
        let pid = std::process::id();
        let info = process_info(pid).expect("should get info for current process");
        assert_eq!(info.pid, pid);
        // cmdline should be non-empty for the test runner
        assert!(!info.cmdline.is_empty(), "cmdline should not be empty");
    }

    #[test]
    fn is_process_alive_current() {
        assert!(is_process_alive(std::process::id()));
    }

    #[test]
    fn is_process_alive_invalid() {
        assert!(!is_process_alive(99_999_999));
    }

    #[test]
    fn list_pids_returns_current() {
        let pids = list_pids();
        assert!(
            pids.contains(&std::process::id()),
            "list_pids should contain current process"
        );
    }

    #[test]
    fn format_duration_units() {
        assert_eq!(format_duration(std::time::Duration::from_secs(30)), "30s");
        assert_eq!(format_duration(std::time::Duration::from_secs(120)), "2m");
        assert_eq!(format_duration(std::time::Duration::from_secs(7200)), "2h");
        assert_eq!(
            format_duration(std::time::Duration::from_secs(172800)),
            "2d"
        );
    }

    // ── find_nodes tests ────────────────────────────────────────────

    #[test]
    fn find_nodes_empty_dir_returns_empty() {
        std::env::set_var(
            "HORUS_NAMESPACE",
            format!("test_nodes_empty_{}", std::process::id()),
        );
        let nodes = find_nodes();
        assert!(nodes.is_empty(), "no presence files = no nodes");
        std::env::remove_var("HORUS_NAMESPACE");
    }

    #[test]
    fn find_nodes_with_valid_presence_file() {
        let ns = format!("test_nodes_valid_{}", std::process::id());
        std::env::set_var("HORUS_NAMESPACE", &ns);
        let nodes_dir = crate::shm::shm_nodes_dir();
        std::fs::create_dir_all(&nodes_dir).unwrap();

        // Write a presence file for the current process (so liveness check passes)
        let presence = serde_json::json!({
            "name": "test_node",
            "pid": std::process::id(),
            "scheduler": "test_scheduler",
            "publishers": [{"topic": "cmd_vel", "type_name": "CmdVel"}],
            "subscribers": [{"topic": "odom", "type_name": "Odometry"}],
            "priority": 10,
            "rate_hz": 100.0,
            "health_status": "Healthy",
            "tick_count": 42,
            "error_count": 0,
            "services": ["get_state"],
            "actions": ["navigate"],
        });
        std::fs::write(
            nodes_dir.join("test_node.json"),
            serde_json::to_string(&presence).unwrap(),
        )
        .unwrap();

        let nodes = find_nodes();
        assert_eq!(nodes.len(), 1, "should discover the written presence file");
        let node = &nodes[0];
        assert_eq!(node.name, "test_node");
        assert_eq!(node.pid, std::process::id());
        assert_eq!(node.publishers.len(), 1);
        assert_eq!(node.publishers[0].topic, "cmd_vel");
        assert_eq!(node.subscribers.len(), 1);
        assert_eq!(node.subscribers[0].topic, "odom");
        assert_eq!(node.priority, 10);
        assert_eq!(node.tick_count, 42);
        assert_eq!(node.services, vec!["get_state"]);
        assert_eq!(node.actions, vec!["navigate"]);
        // Process enrichment should populate cmdline
        assert!(
            !node.cmdline.is_empty(),
            "cmdline should be enriched from process_info"
        );

        let _ = std::fs::remove_dir_all(crate::shm::shm_base_dir());
        std::env::remove_var("HORUS_NAMESPACE");
    }

    #[test]
    fn find_nodes_skips_dead_pid() {
        let ns = format!("test_nodes_dead_{}", std::process::id());
        std::env::set_var("HORUS_NAMESPACE", &ns);
        let nodes_dir = crate::shm::shm_nodes_dir();
        std::fs::create_dir_all(&nodes_dir).unwrap();

        // Write presence file with a dead PID
        let presence = serde_json::json!({
            "name": "dead_node",
            "pid": 99999999,
        });
        std::fs::write(
            nodes_dir.join("dead_node.json"),
            serde_json::to_string(&presence).unwrap(),
        )
        .unwrap();

        let nodes = find_nodes();
        assert!(nodes.is_empty(), "dead PID should be filtered out");

        let _ = std::fs::remove_dir_all(crate::shm::shm_base_dir());
        std::env::remove_var("HORUS_NAMESPACE");
    }

    #[test]
    fn find_nodes_skips_non_json_files() {
        let ns = format!("test_nodes_nonjson_{}", std::process::id());
        std::env::set_var("HORUS_NAMESPACE", &ns);
        let nodes_dir = crate::shm::shm_nodes_dir();
        std::fs::create_dir_all(&nodes_dir).unwrap();

        std::fs::write(nodes_dir.join("not_a_node.txt"), "hello").unwrap();
        std::fs::write(nodes_dir.join("also_not.meta"), "{}").unwrap();

        let nodes = find_nodes();
        assert!(nodes.is_empty(), "non-json files should be skipped");

        let _ = std::fs::remove_dir_all(crate::shm::shm_base_dir());
        std::env::remove_var("HORUS_NAMESPACE");
    }

    // ── find_topics tests ───────────────────────────────────────────

    #[test]
    fn find_topics_empty_returns_empty() {
        std::env::set_var(
            "HORUS_NAMESPACE",
            format!("test_topics_empty_{}", std::process::id()),
        );
        let topics = find_topics();
        assert!(topics.is_empty(), "no meta files = no topics");
        std::env::remove_var("HORUS_NAMESPACE");
    }

    #[test]
    fn find_topics_discovers_meta_files() {
        let ns = format!("test_topics_meta_{}", std::process::id());
        std::env::set_var("HORUS_NAMESPACE", &ns);

        // Write a topic meta
        crate::shm::write_topic_meta("lidar_scan", 8192).unwrap();

        let topics = find_topics();
        assert!(
            topics
                .iter()
                .any(|t| t.name == "lidar_scan" && t.size == 8192),
            "should discover topic from .meta file"
        );

        let _ = std::fs::remove_dir_all(crate::shm::shm_base_dir());
        std::env::remove_var("HORUS_NAMESPACE");
    }

    #[test]
    fn find_topics_checks_creator_liveness() {
        let ns = format!("test_topics_alive_{}", std::process::id());
        std::env::set_var("HORUS_NAMESPACE", &ns);
        let dir = crate::shm::shm_topics_dir();
        std::fs::create_dir_all(&dir).unwrap();

        // Write meta with dead PID
        let meta = crate::shm::TopicMeta {
            name: "dead_topic".into(),
            size: 1024,
            creator_pid: 99999999,
            created_at: 0,
        };
        std::fs::write(
            dir.join("dead_topic.meta"),
            serde_json::to_string(&meta).unwrap(),
        )
        .unwrap();

        let topics = find_topics();
        let topic = topics.iter().find(|t| t.name == "dead_topic");
        assert!(topic.is_some(), "dead topic should still be listed");
        assert!(
            !topic.unwrap().is_alive,
            "dead creator PID should mark topic as not alive"
        );

        let _ = std::fs::remove_dir_all(crate::shm::shm_base_dir());
        std::env::remove_var("HORUS_NAMESPACE");
    }

    // ── process_info tests ──────────────────────────────────────────

    #[test]
    fn process_info_dead_pid_returns_error() {
        let result = process_info(99_999_999);
        // Dead PIDs should fail (can't read /proc/99999999 or equivalent)
        assert!(result.is_err() || result.unwrap().cmdline.is_empty());
    }

    #[test]
    fn process_info_current_has_memory() {
        let info = process_info(std::process::id()).unwrap();
        assert!(
            info.memory_kb > 0,
            "current process should have some memory"
        );
    }

    #[test]
    fn list_pids_has_reasonable_count() {
        let pids = list_pids();
        assert!(pids.len() > 1, "system should have multiple processes");
        assert!(pids.len() < 1_000_000, "sanity check: not millions of PIDs");
    }
}
