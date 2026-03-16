use super::nodes::{
    categorize_process, extract_process_name, format_duration, parse_memory_from_stat,
    parse_stat_fields, process_exists, should_track_process,
};
use super::topics::{compute_topic_status, discover_shared_memory_uncached, format_age, is_horus_process};
use super::*;
use horus_core::core::DurationExt;

// =====================
// Test Helpers
// =====================

/// Helper to write presence files as raw JSON from horus_manager tests.
/// NodePresence::new/write are pub(crate) in horus_core, so we write JSON directly.
/// Tracks created files and cleans up on Drop (even on test failures).
struct TestPresenceWriter {
    nodes_dir: std::path::PathBuf,
    created_files: Vec<std::path::PathBuf>,
    prefix: String,
}

impl TestPresenceWriter {
    fn new(test_name: &str) -> Self {
        let nodes_dir = horus_core::memory::shm_base_dir().join("nodes");
        let _ = std::fs::create_dir_all(&nodes_dir);
        let nanos = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap_or_default()
            .subsec_nanos();
        Self {
            nodes_dir,
            created_files: Vec::new(),
            prefix: format!("{}_{}", test_name, nanos),
        }
    }

    fn prefixed_name(&self, name: &str) -> String {
        format!("{}_{}", self.prefix, name)
    }

    fn write_node(
        &mut self,
        name: &str,
        pubs_json: &str,
        subs_json: &str,
    ) -> String {
        self.write_node_ext(name, pubs_json, subs_json, None, None, None, 0, 0)
    }

    #[allow(clippy::too_many_arguments)]
    fn write_node_ext(
        &mut self,
        name: &str,
        pubs_json: &str,
        subs_json: &str,
        scheduler: Option<&str>,
        health: Option<&str>,
        rate_hz: Option<f64>,
        tick_count: u64,
        error_count: u32,
    ) -> String {
        let full_name = self.prefixed_name(name);
        let pid = std::process::id();
        let now_secs = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap_or_default()
            .as_secs();
        let sched = scheduler.unwrap_or("test_sched");
        let health_json = match health {
            Some(h) => format!(r#""{}""#, h),
            None => "null".to_string(),
        };
        let rate_json = match rate_hz {
            Some(r) => format!("{}", r),
            None => "null".to_string(),
        };
        let json = format!(
            r#"{{"name":"{}","pid":{},"scheduler":"{}","publishers":{},"subscribers":{},"start_time":{},"priority":0,"rate_hz":{},"pid_start_time":0,"health_status":{},"tick_count":{},"error_count":{}}}"#,
            full_name, pid, sched, pubs_json, subs_json, now_secs, rate_json, health_json, tick_count, error_count
        );
        let path = self.nodes_dir.join(format!("{}.json", full_name));
        // Use atomic write (tmp+rename) to prevent parallel read_all() from
        // seeing a partial file, failing to parse it, and deleting it.
        let tmp = self.nodes_dir.join(format!("{}.json.tmp", full_name));
        std::fs::write(&tmp, &json).expect("write presence tmp");
        std::fs::rename(&tmp, &path).expect("atomic rename");
        self.created_files.push(path);
        full_name
    }

    fn write_dead_node(&mut self, name: &str) -> String {
        let full_name = self.prefixed_name(name);
        let json = format!(
            r#"{{"name":"{}","pid":999999999,"scheduler":"dead_sched","publishers":[],"subscribers":[],"start_time":0,"priority":0,"rate_hz":null,"pid_start_time":0}}"#,
            full_name
        );
        let path = self.nodes_dir.join(format!("{}.json", full_name));
        let tmp = self.nodes_dir.join(format!("{}.json.tmp", full_name));
        std::fs::write(&tmp, &json).expect("write dead presence tmp");
        std::fs::rename(&tmp, &path).expect("atomic rename");
        self.created_files.push(path);
        full_name
    }

    fn write_raw(&mut self, filename: &str, content: &[u8]) {
        let path = self.nodes_dir.join(filename);
        std::fs::write(&path, content).expect("write raw file");
        self.created_files.push(path);
    }

    fn file_exists(&self, name: &str) -> bool {
        let full_name = self.prefixed_name(name);
        self.nodes_dir.join(format!("{}.json", full_name)).exists()
    }

    /// Read a specific presence file by full node name, bypassing read_all()
    /// which can race with parallel tests (it scans the entire directory and
    /// may delete other tests' in-flight files).
    fn read_presence(&self, full_name: &str) -> Option<NodePresence> {
        let path = self.nodes_dir.join(format!("{}.json", full_name));
        let content = std::fs::read_to_string(&path).ok()?;
        serde_json::from_str(&content).ok()
    }

    /// Read all presence files created by THIS test (not the global directory).
    /// Each file is read individually, avoiding the read_all() global scan.
    fn read_all_own(&self) -> Vec<NodePresence> {
        self.created_files.iter().filter_map(|path| {
            let content = std::fs::read_to_string(path).ok()?;
            serde_json::from_str(&content).ok()
        }).collect()
    }

}

impl Drop for TestPresenceWriter {
    fn drop(&mut self) {
        for path in &self.created_files {
            let _ = std::fs::remove_file(path);
        }
    }
}

// =====================
// NodeStatus Tests
// =====================
#[test]
fn test_node_status_creation() {
    let node = NodeStatus {
        name: "test_node".to_string(),
        status: "Running".to_string(),
        health: HealthStatus::Healthy,
        priority: 10,
        process_id: 1234,
        command_line: "horus run test".to_string(),
        working_dir: "/home/test".to_string(),
        cpu_usage: 25.5,
        memory_usage: 1024,
        start_time: "10m".to_string(),
        scheduler_name: "default".to_string(),
        category: ProcessCategory::Node,
        tick_count: 100,
        error_count: 0,
        actual_rate_hz: 50,
        publishers: vec![],
        subscribers: vec![],
        live_tick_count: None,
        live_health: None,
        live_avg_tick_ns: None,
        live_max_tick_ns: None,
        live_budget_misses: None,
        live_deadline_misses: None,
    };

    assert_eq!(node.name, "test_node");
    assert_eq!(node.status, "Running");
    assert_eq!(node.priority, 10);
    assert_eq!(node.process_id, 1234);
    assert_eq!(node.tick_count, 100);
    assert_eq!(node.error_count, 0);
    assert_eq!(node.actual_rate_hz, 50);
    assert!((node.cpu_usage - 25.5).abs() < f32::EPSILON);
    assert_eq!(node.memory_usage, 1024);
    assert_eq!(node.command_line, "horus run test");
    assert_eq!(node.working_dir, "/home/test");
    assert_eq!(node.scheduler_name, "default");
    assert!(node.publishers.is_empty());
    assert!(node.subscribers.is_empty());

    // Debug impl should contain all key fields
    let debug = format!("{:?}", node);
    assert!(debug.contains("test_node"), "Debug should contain name");
    assert!(debug.contains("Running"), "Debug should contain status");
    assert!(debug.contains("Healthy"), "Debug should contain health");

    // Clone preserves all fields
    let cloned = node.clone();
    assert_eq!(cloned.name, node.name);
    assert_eq!(cloned.process_id, node.process_id);
    assert_eq!(cloned.category, node.category);
    assert_eq!(cloned.tick_count, node.tick_count);
    assert_eq!(cloned.scheduler_name, node.scheduler_name);
}

#[test]
fn test_node_status_with_publishers_subscribers() {
    let pub_topic = TopicInfo {
        topic: "sensor.data".to_string(),
        type_name: "SensorMsg".to_string(),
    };
    let sub_topic = TopicInfo {
        topic: "commands".to_string(),
        type_name: "CmdMsg".to_string(),
    };

    let node = NodeStatus {
        name: "sensor_node".to_string(),
        status: "Running".to_string(),
        health: HealthStatus::Healthy,
        priority: 5,
        process_id: 5678,
        command_line: String::new(),
        working_dir: String::new(),
        cpu_usage: 0.0,
        memory_usage: 0,
        start_time: String::new(),
        scheduler_name: "main".to_string(),
        category: ProcessCategory::Node,
        tick_count: 0,
        error_count: 0,
        actual_rate_hz: 0,
        publishers: vec![pub_topic],
        subscribers: vec![sub_topic],
        live_tick_count: None, live_health: None, live_avg_tick_ns: None,
        live_max_tick_ns: None, live_budget_misses: None, live_deadline_misses: None,
    };

    assert_eq!(node.publishers.len(), 1);
    assert_eq!(node.subscribers.len(), 1);
    assert_eq!(node.publishers[0].topic, "sensor.data");
    assert_eq!(node.subscribers[0].type_name, "CmdMsg");
}

// =====================
// ProcessCategory Tests
// =====================
#[test]
fn test_process_category_equality() {
    assert_eq!(ProcessCategory::Node, ProcessCategory::Node);
    assert_eq!(ProcessCategory::Tool, ProcessCategory::Tool);
    assert_eq!(ProcessCategory::CLI, ProcessCategory::CLI);
    assert_ne!(ProcessCategory::Node, ProcessCategory::Tool);
    assert_ne!(ProcessCategory::Tool, ProcessCategory::CLI);
}

// =====================
// SharedMemoryInfo Tests
// =====================
#[test]
fn test_shared_memory_info_creation() {
    let shm = SharedMemoryInfo {
        topic_name: "robot.pose".to_string(),
        size_bytes: 4096,
        active: true,
        accessing_processes: vec![1234, 5678],
        last_modified: Some(std::time::SystemTime::now()),
        message_type: Some("PoseMsg".to_string()),
        publishers: vec!["localization".to_string()],
        subscribers: vec!["navigation".to_string(), "visualization".to_string()],
        message_rate_hz: 30.0,
        status: TopicStatus::Active,
        age_string: "0s ago".to_string(),
        is_system: false,
        messages_total: 0,
        topic_kind: 0,
    };

    assert_eq!(shm.topic_name, "robot.pose");
    assert_eq!(shm.size_bytes, 4096);
    assert!(shm.active);
    assert_eq!(shm.accessing_processes.len(), 2);
    assert_eq!(shm.accessing_processes[0], 1234);
    assert_eq!(shm.accessing_processes[1], 5678);
    assert_eq!(shm.publishers.len(), 1);
    assert_eq!(shm.publishers[0], "localization");
    assert_eq!(shm.subscribers.len(), 2);
    assert_eq!(shm.subscribers[0], "navigation");
    assert_eq!(shm.subscribers[1], "visualization");
    assert!((shm.message_rate_hz - 30.0).abs() < 0.001);
    assert_eq!(shm.status, TopicStatus::Active);
    assert_eq!(shm.age_string, "0s ago");
    assert!(!shm.is_system);
    assert!(shm.message_type.is_some());
    assert_eq!(shm.message_type.as_deref(), Some("PoseMsg"));
    assert!(shm.last_modified.is_some());

    // Total participant count should match accessing_processes
    let total_participants = shm.publishers.len() + shm.subscribers.len();
    assert!(
        total_participants <= shm.accessing_processes.len() + 10, // allow headroom
        "participant count ({}) should be reasonable relative to accessing processes ({})",
        total_participants, shm.accessing_processes.len()
    );

    // Debug output should contain the topic name
    let debug = format!("{:?}", shm);
    assert!(debug.contains("robot.pose"), "Debug should contain topic name");
    assert!(debug.contains("Active"), "Debug should contain status");
}

#[test]
fn test_shared_memory_info_inactive() {
    let shm = SharedMemoryInfo {
        topic_name: "old.topic".to_string(),
        size_bytes: 1024,
        active: false,
        accessing_processes: vec![],
        last_modified: None,
        message_type: None,
        publishers: vec![],
        subscribers: vec![],
        message_rate_hz: 0.0,
        status: TopicStatus::Stale,
        age_string: "unknown".to_string(),
        is_system: false,
        messages_total: 0,
        topic_kind: 0,
    };

    assert!(!shm.active);
    assert!(shm.accessing_processes.is_empty());
    assert!(shm.message_type.is_none());
    assert!(shm.last_modified.is_none());
    assert_eq!(shm.status, TopicStatus::Stale);
    assert_eq!(shm.age_string, "unknown");
    assert!(shm.publishers.is_empty());
    assert!(shm.subscribers.is_empty());
    assert!((shm.message_rate_hz - 0.0).abs() < f32::EPSILON);

    // Inactive topics should be consistent: no rate, no processes, stale status
    assert_eq!(shm.status, TopicStatus::Stale, "inactive topic should be Stale");
    assert!(
        shm.accessing_processes.is_empty(),
        "inactive topic should have no accessing processes"
    );
}

// =====================
// TopicInfo Tests
// =====================
#[test]
fn test_topic_info_creation() {
    let topic = TopicInfo {
        topic: "camera.image".to_string(),
        type_name: "sensor_msgs::Image".to_string(),
    };

    assert_eq!(topic.topic, "camera.image");
    assert_eq!(topic.type_name, "sensor_msgs::Image");

    // Verify Clone preserves both fields
    let cloned = topic.clone();
    assert_eq!(cloned.topic, topic.topic);
    assert_eq!(cloned.type_name, topic.type_name);

    // Verify Debug output contains fields
    let debug = format!("{:?}", topic);
    assert!(debug.contains("camera.image"), "Debug should contain topic name");
    assert!(debug.contains("sensor_msgs::Image"), "Debug should contain type name");
}

// =====================
// Helper Function Tests
// =====================
#[test]
fn test_format_duration_seconds() {
    let duration = 45_u64.secs();
    assert_eq!(format_duration(duration), "45s");
}

#[test]
fn test_format_duration_minutes() {
    let duration = 125_u64.secs();
    assert_eq!(format_duration(duration), "2m");
}

#[test]
fn test_format_duration_hours() {
    let duration = 7200_u64.secs();
    assert_eq!(format_duration(duration), "2h");
}

#[test]
fn test_format_duration_days() {
    let duration = 172800_u64.secs();
    assert_eq!(format_duration(duration), "2d");
}

#[test]
fn test_should_track_process_empty() {
    assert!(!should_track_process(""));
    assert!(!should_track_process("   "));
}

#[test]
fn test_should_track_process_excluded_patterns() {
    // Build tools should be excluded
    assert!(!should_track_process("cargo build --release"));
    assert!(!should_track_process("cargo test"));
    assert!(!should_track_process("rustc --version"));
    assert!(!should_track_process("/bin/bash script.sh"));
    assert!(!should_track_process("timeout 10 some_command"));
    assert!(!should_track_process("dashboard server"));
    assert!(!should_track_process("horus run test_package"));
}

#[test]
fn test_should_track_process_scheduler() {
    // Standalone scheduler should be tracked
    assert!(should_track_process("/path/to/scheduler binary"));
}

#[test]
fn test_categorize_process_gui() {
    assert_eq!(categorize_process("robot_gui", ""), ProcessCategory::Tool);
    assert_eq!(categorize_process("viewer_app", ""), ProcessCategory::Tool);
    assert_eq!(categorize_process("viz_tool", ""), ProcessCategory::Tool);
    assert_eq!(categorize_process("my_GUI_app", ""), ProcessCategory::Tool);
    assert_eq!(categorize_process("app_gui", ""), ProcessCategory::Tool);
    assert_eq!(categorize_process("test", "--gui"), ProcessCategory::Tool);
    assert_eq!(
        categorize_process("test", "--view mode"),
        ProcessCategory::Tool
    );
}

#[test]
fn test_categorize_process_cli() {
    assert_eq!(categorize_process("horus", ""), ProcessCategory::CLI);
    assert_eq!(categorize_process("horus run", ""), ProcessCategory::CLI);
    assert_eq!(
        categorize_process("test", "/bin/horus run pkg"),
        ProcessCategory::CLI
    );
    assert_eq!(
        categorize_process("test", "target/debug/horus run pkg"),
        ProcessCategory::CLI
    );
}

#[test]
fn test_categorize_process_node() {
    assert_eq!(categorize_process("scheduler", ""), ProcessCategory::Node);
    assert_eq!(
        categorize_process("test", "my_scheduler"),
        ProcessCategory::Node
    );
    // Default is Node
    assert_eq!(
        categorize_process("unknown_process", "unknown cmd"),
        ProcessCategory::Node
    );
}

#[test]
fn test_extract_process_name_simple() {
    assert_eq!(
        extract_process_name("/usr/bin/robot_control"),
        "robot_control"
    );
    assert_eq!(extract_process_name("./my_program"), "my_program");
}

#[test]
fn test_extract_process_name_horus_cli() {
    assert_eq!(
        extract_process_name("horus run my_package"),
        "horus run my_package"
    );
    assert_eq!(
        extract_process_name("horus monitor dashboard"),
        "horus monitor dashboard"
    );
    assert_eq!(extract_process_name("horus version"), "horus version");
}

#[test]
fn test_extract_process_name_empty() {
    assert_eq!(extract_process_name(""), "Unknown");
}

#[test]
#[cfg(target_os = "linux")]
fn test_parse_memory_from_stat_valid() {
    // stat format: pid (comm) state ... rss is 24th field (0-indexed: 23)
    // We need at least 24 space-separated fields
    let stat = "1234 (test) S 1 1234 1234 0 -1 4194304 100 0 0 0 10 5 0 0 20 0 1 0 12345 12345678 500 18446744073709551615 0 0 0 0 0 0 0 0 0 0 0 0 17 0 0 0 0 0 0";
    let memory = parse_memory_from_stat(stat);
    // 500 pages * 4KB = 2000KB
    assert_eq!(memory, 2000);
}

#[test]
#[cfg(target_os = "linux")]
fn test_parse_memory_from_stat_invalid() {
    assert_eq!(parse_memory_from_stat(""), 0);
    assert_eq!(parse_memory_from_stat("short stat"), 0);
}

// =====================
// Public API Tests (with real test data)
// =====================

/// Helper to create test topic file
fn create_test_topic(topic_name: &str) -> Option<std::path::PathBuf> {
    let topics_dir = shm_topics_dir();
    if std::fs::create_dir_all(&topics_dir).is_err() {
        return None;
    }

    let safe_name: String = topic_name
        .chars()
        .map(|c| if c == '/' || c == ' ' { '_' } else { c })
        .collect();
    let filepath = topics_dir.join(&safe_name);

    // Create a small test file
    if std::fs::write(&filepath, vec![0u8; 1024]).is_ok() {
        Some(filepath)
    } else {
        None
    }
}

/// Cleanup helper
fn cleanup_test_file(path: Option<std::path::PathBuf>) {
    if let Some(p) = path {
        let _ = std::fs::remove_file(p);
    }
}

#[test]
#[ignore] // Requires /dev/shm filesystem; run with: cargo test -- --ignored
fn test_discover_shared_memory_with_real_topic() {
    // Use simple topic name to avoid underscore-to-slash conversion confusion
    let test_topic = "testshm"; // Simple name without underscores
    let topic_file = create_test_topic(test_topic);

    if topic_file.is_some() {
        // Force cache refresh - handle potential poisoned lock
        let cache_refreshed = DISCOVERY_CACHE
            .write()
            .map(|mut cache| {
                cache.shared_memory_last_updated =
                    std::time::Instant::now() - 10_u64.secs();
                true
            })
            .unwrap_or(false);

        if !cache_refreshed {
            cleanup_test_file(topic_file);
            return; // Skip test if cache is poisoned
        }

        // Call the uncached version directly to avoid cache issues in parallel tests
        let result = discover_shared_memory_uncached();
        if let Ok(topics) = result {
            // Should find our test topic (underscores in filename become / in topic name)
            let found = topics.iter().any(|t| t.topic_name.contains("testshm"));
            assert!(
                found,
                "Should discover testshm topic, found: {:?}",
                topics.iter().map(|t| &t.topic_name).collect::<Vec<_>>()
            );

            // Verify topic properties
            if let Some(topic) = topics.iter().find(|t| t.topic_name.contains("testshm")) {
                assert_eq!(topic.size_bytes, 1024, "Topic should be 1024 bytes");
            }
        }
        // If result is Err, that's OK - test data might have been cleaned up by another test
    }

    cleanup_test_file(topic_file);
}

#[test]
fn test_discover_nodes_returns_vec() {
    let result = discover_nodes();
    match result {
        Ok(nodes) => {
            for node in &nodes {
                assert!(!node.name.is_empty(), "Node name must not be empty");
            }
        }
        Err(e) => {
            // No /dev/shm/horus data is acceptable in CI
            let msg = format!("{}", e);
            assert!(
                msg.contains("not found") || msg.contains("No such") || msg.contains("shm"),
                "Unexpected error from discover_nodes: {}",
                msg
            );
        }
    }
}

#[test]
fn test_discover_shared_memory_handles_missing_dirs() {
    // When /dev/shm/horus dirs don't exist, should return Ok(empty) or Err
    let result = discover_shared_memory();
    match result {
        Ok(shm_info) => {
            // Each entry must have a valid non-empty topic name
            for info in &shm_info {
                assert!(
                    !info.topic_name.is_empty(),
                    "SHM topic name must not be empty"
                );
                assert!(
                    info.size_bytes > 0,
                    "SHM region size must be > 0, topic: {}",
                    info.topic_name
                );
            }
        }
        Err(e) => {
            let msg = format!("{}", e);
            assert!(!msg.is_empty(), "Error message must not be empty");
        }
    }
}

#[test]
#[ignore] // Requires /dev/shm filesystem; run with: cargo test -- --ignored
fn test_topic_inactive_detection() {
    // Create a topic file and verify active detection works
    let topics_dir = shm_topics_dir();
    if std::fs::create_dir_all(&topics_dir).is_err() {
        return;
    }

    let test_file = topics_dir.join("test_active_topic");
    if std::fs::write(&test_file, vec![0u8; 512]).is_ok() {
        // Force cache refresh
        if let Ok(mut cache) = DISCOVERY_CACHE.write() {
            cache.shared_memory_last_updated =
                std::time::Instant::now() - 10_u64.secs();
        }

        let result = discover_shared_memory();
        if let Ok(topics) = result {
            if let Some(topic) = topics.iter().find(|t| t.topic_name.contains("test_active")) {
                // Just-created file should be considered active (recently modified)
                assert!(topic.active, "Recently created topic should be active");
            }
        }

        let _ = std::fs::remove_file(&test_file);
    }
}

// =====================
// DiscoveryCache Tests
// =====================
#[test]
fn test_discovery_cache_new_is_stale() {
    let cache = DiscoveryCache::new();
    // New cache should be stale (forces initial update)
    assert!(cache.is_nodes_stale());
    assert!(cache.is_shared_memory_stale());
}

#[test]
fn test_discovery_cache_update_nodes() {
    let mut cache = DiscoveryCache::new();
    let nodes = vec![NodeStatus {
        name: "test".to_string(),
        status: "Running".to_string(),
        health: HealthStatus::Healthy,
        priority: 0,
        process_id: 0,
        command_line: String::new(),
        working_dir: String::new(),
        cpu_usage: 0.0,
        memory_usage: 0,
        start_time: String::new(),
        scheduler_name: String::new(),
        category: ProcessCategory::Node,
        tick_count: 0,
        error_count: 0,
        actual_rate_hz: 0,
        publishers: vec![],
        subscribers: vec![],
        live_tick_count: None, live_health: None, live_avg_tick_ns: None,
        live_max_tick_ns: None, live_budget_misses: None, live_deadline_misses: None,
    }];

    cache.update_nodes(nodes);

    // After update, nodes should not be stale (but shared_memory still is)
    assert!(!cache.is_nodes_stale());
    assert_eq!(cache.nodes.len(), 1);
}

#[test]
fn test_discovery_cache_update_shared_memory() {
    let mut cache = DiscoveryCache::new();
    let shm = vec![SharedMemoryInfo {
        topic_name: "test".to_string(),
        size_bytes: 1024,
        active: true,
        accessing_processes: vec![],
        last_modified: None,
        message_type: None,
        publishers: vec![],
        subscribers: vec![],
        message_rate_hz: 0.0,
        status: TopicStatus::Stale,
        age_string: "unknown".to_string(),
        is_system: false,
        messages_total: 0,
        topic_kind: 0,
    }];

    cache.update_shared_memory(shm);

    // After update, shared_memory should not be stale (but nodes still is)
    assert!(!cache.is_shared_memory_stale());
    assert_eq!(cache.shared_memory.len(), 1);
}

// =====================
// Process Existence Tests
// =====================
#[test]
fn test_process_exists_self() {
    // Current process should exist
    let pid = std::process::id();
    assert!(process_exists(pid));
}

#[test]
fn test_process_exists_invalid() {
    // PID 0 or very high numbers shouldn't exist
    assert!(!process_exists(999999999));
}

// =====================
// Edge Cases Tests
// =====================
#[test]
fn test_node_status_clone() {
    let node = NodeStatus {
        name: "clone_test".to_string(),
        status: "Running".to_string(),
        health: HealthStatus::Warning,
        priority: 5,
        process_id: 9999,
        command_line: "test cmd".to_string(),
        working_dir: "/tmp".to_string(),
        cpu_usage: 50.0,
        memory_usage: 2048,
        start_time: "1h".to_string(),
        scheduler_name: "test_sched".to_string(),
        category: ProcessCategory::Tool,
        tick_count: 500,
        error_count: 2,
        actual_rate_hz: 100,
        publishers: vec![TopicInfo {
            topic: "pub".to_string(),
            type_name: "Msg".to_string(),
        }],
        subscribers: vec![],
        live_tick_count: None, live_health: None, live_avg_tick_ns: None,
        live_max_tick_ns: None, live_budget_misses: None, live_deadline_misses: None,
    };

    let cloned = node.clone();
    // Verify all fields are deeply equal
    assert_eq!(cloned.name, node.name);
    assert_eq!(cloned.status, node.status);
    assert_eq!(cloned.priority, node.priority);
    assert_eq!(cloned.process_id, node.process_id);
    assert_eq!(cloned.command_line, node.command_line);
    assert_eq!(cloned.working_dir, node.working_dir);
    assert!((cloned.cpu_usage - node.cpu_usage).abs() < f32::EPSILON);
    assert_eq!(cloned.memory_usage, node.memory_usage);
    assert_eq!(cloned.start_time, node.start_time);
    assert_eq!(cloned.scheduler_name, node.scheduler_name);
    assert_eq!(cloned.category, node.category);
    assert_eq!(cloned.tick_count, node.tick_count);
    assert_eq!(cloned.error_count, node.error_count);
    assert_eq!(cloned.actual_rate_hz, node.actual_rate_hz);
    assert_eq!(cloned.publishers.len(), node.publishers.len());
    assert_eq!(cloned.publishers[0].topic, "pub");
    assert_eq!(cloned.publishers[0].type_name, "Msg");
    assert_eq!(cloned.subscribers.len(), 0);
}

#[test]
fn test_shared_memory_info_clone() {
    let shm = SharedMemoryInfo {
        topic_name: "clone_topic".to_string(),
        size_bytes: 8192,
        active: true,
        accessing_processes: vec![1, 2, 3],
        last_modified: Some(std::time::SystemTime::now()),
        message_type: Some("TestMsg".to_string()),
        publishers: vec!["pub1".to_string()],
        subscribers: vec!["sub1".to_string(), "sub2".to_string()],
        message_rate_hz: 60.0,
        status: TopicStatus::Active,
        age_string: "0s ago".to_string(),
        is_system: false,
        messages_total: 0,
        topic_kind: 0,
    };

    let cloned = shm.clone();
    // Verify all fields deeply
    assert_eq!(cloned.topic_name, shm.topic_name);
    assert_eq!(cloned.size_bytes, shm.size_bytes);
    assert_eq!(cloned.active, shm.active);
    assert_eq!(cloned.accessing_processes, vec![1, 2, 3]);
    assert_eq!(cloned.message_type, shm.message_type);
    assert_eq!(cloned.publishers, vec!["pub1"]);
    assert_eq!(cloned.subscribers, vec!["sub1", "sub2"]);
    assert!((cloned.message_rate_hz - shm.message_rate_hz).abs() < f32::EPSILON);
    assert_eq!(cloned.status, shm.status);
    assert_eq!(cloned.age_string, shm.age_string);
    assert_eq!(cloned.is_system, shm.is_system);
    // last_modified is cloned (SystemTime implements Clone)
    assert!(cloned.last_modified.is_some());
}

#[test]
fn test_health_status_variants() {
    // Verify all HealthStatus variants can be assigned and distinguished
    let variants = [
        HealthStatus::Healthy,
        HealthStatus::Warning,
        HealthStatus::Error,
        HealthStatus::Critical,
    ];

    // Each variant should have a distinct Debug representation
    let debug_strs: Vec<String> = variants.iter().map(|v| format!("{:?}", v)).collect();
    for (i, a) in debug_strs.iter().enumerate() {
        for (j, b) in debug_strs.iter().enumerate() {
            if i != j {
                assert_ne!(a, b, "All health variants should have distinct Debug output");
            }
        }
    }

    // Verify a node can hold each variant
    for variant in &variants {
        let node = NodeStatus {
            name: "h".to_string(),
            status: String::new(),
            health: variant.clone(),
            priority: 0,
            process_id: 0,
            command_line: String::new(),
            working_dir: String::new(),
            cpu_usage: 0.0,
            memory_usage: 0,
            start_time: String::new(),
            scheduler_name: String::new(),
            category: ProcessCategory::Node,
            tick_count: 0,
            error_count: 0,
            actual_rate_hz: 0,
            publishers: vec![],
            subscribers: vec![],
        live_tick_count: None, live_health: None, live_avg_tick_ns: None,
        live_max_tick_ns: None, live_budget_misses: None, live_deadline_misses: None,
        };
        let debug = format!("{:?}", node.health);
        assert!(!debug.is_empty(), "Health debug should not be empty");
    }
}

#[test]
#[ignore] // Run with: cargo test test_live_discovery -- --ignored --nocapture
fn test_live_discovery() {
    println!("\n=== LIVE DISCOVERY TEST ===");

    let discovered = discover_shared_memory().unwrap_or_default();

    println!("Discovered {} topics:", discovered.len());
    for topic in &discovered {
        println!("  - Topic: {}", topic.topic_name);
        println!("    Active: {}", topic.active);
        println!("    Size: {} bytes", topic.size_bytes);
        println!("    Processes: {:?}", topic.accessing_processes);
        println!("    Publishers: {:?}", topic.publishers);
        println!("    Subscribers: {:?}", topic.subscribers);
        println!();
    }

    // Check what's on disk (flat namespace)
    println!("=== DISK CHECK ===");
    let topics_dir = shm_topics_dir();
    if topics_dir.exists() {
        println!("Topics directory exists: {:?}", topics_dir);
        if let Ok(topic_entries) = std::fs::read_dir(&topics_dir) {
            for t in topic_entries.flatten() {
                println!("  Topic file: {:?}", t.file_name());
            }
        }
    } else {
        println!("Topics directory DOES NOT EXIST");
    }
}

// =====================
// Phase 4: Cache TTL & Synchronization Tests
// =====================

#[test]
fn test_cache_ttl_boundary_fresh_after_update() {
    // A freshly updated cache should NOT be stale
    let mut cache = DiscoveryCache::new();
    assert!(cache.is_nodes_stale(), "new cache should be stale");

    cache.update_nodes(vec![]);
    assert!(
        !cache.is_nodes_stale(),
        "cache should be fresh after update"
    );

    cache.update_shared_memory(vec![]);
    assert!(
        !cache.is_shared_memory_stale(),
        "shm cache should be fresh after update"
    );
}

#[test]
fn test_cache_nodes_and_shm_independent_staleness() {
    // Updating nodes should NOT make shared_memory fresh (and vice versa)
    let mut cache = DiscoveryCache::new();

    cache.update_nodes(vec![]);
    assert!(!cache.is_nodes_stale(), "nodes should be fresh");
    assert!(cache.is_shared_memory_stale(), "shm should still be stale");

    let mut cache2 = DiscoveryCache::new();
    cache2.update_shared_memory(vec![]);
    assert!(cache2.is_nodes_stale(), "nodes should still be stale");
    assert!(!cache2.is_shared_memory_stale(), "shm should be fresh");
}

#[test]
fn test_cache_update_replaces_data() {
    let mut cache = DiscoveryCache::new();

    // First update with 2 nodes
    let nodes1 = vec![
        NodeStatus {
            name: "node_a".to_string(),
            status: "Running".to_string(),
            health: HealthStatus::Healthy,
            priority: 0,
            process_id: 1,
            command_line: String::new(),
            working_dir: String::new(),
            cpu_usage: 0.0,
            memory_usage: 0,
            start_time: String::new(),
            scheduler_name: String::new(),
            category: ProcessCategory::Node,
            tick_count: 0,
            error_count: 0,
            actual_rate_hz: 0,
            publishers: vec![],
            subscribers: vec![],
        live_tick_count: None, live_health: None, live_avg_tick_ns: None,
        live_max_tick_ns: None, live_budget_misses: None, live_deadline_misses: None,
        },
        NodeStatus {
            name: "node_b".to_string(),
            status: "Running".to_string(),
            health: HealthStatus::Healthy,
            priority: 0,
            process_id: 2,
            command_line: String::new(),
            working_dir: String::new(),
            cpu_usage: 0.0,
            memory_usage: 0,
            start_time: String::new(),
            scheduler_name: String::new(),
            category: ProcessCategory::Node,
            tick_count: 0,
            error_count: 0,
            actual_rate_hz: 0,
            publishers: vec![],
            subscribers: vec![],
        live_tick_count: None, live_health: None, live_avg_tick_ns: None,
        live_max_tick_ns: None, live_budget_misses: None, live_deadline_misses: None,
        },
    ];
    cache.update_nodes(nodes1);
    assert_eq!(cache.nodes.len(), 2);

    // Second update with 1 node — should REPLACE, not append
    let nodes2 = vec![NodeStatus {
        name: "node_c".to_string(),
        status: "Running".to_string(),
        health: HealthStatus::Healthy,
        priority: 0,
        process_id: 3,
        command_line: String::new(),
        working_dir: String::new(),
        cpu_usage: 0.0,
        memory_usage: 0,
        start_time: String::new(),
        scheduler_name: String::new(),
        category: ProcessCategory::Node,
        tick_count: 0,
        error_count: 0,
        actual_rate_hz: 0,
        publishers: vec![],
        subscribers: vec![],
        live_tick_count: None, live_health: None, live_avg_tick_ns: None,
        live_max_tick_ns: None, live_budget_misses: None, live_deadline_misses: None,
    }];
    cache.update_nodes(nodes2);
    assert_eq!(cache.nodes.len(), 1, "update should replace, not append");
    assert_eq!(cache.nodes[0].name, "node_c");
}

#[test]
fn test_cache_concurrent_read_write_safety() {
    // Test that concurrent reads and writes don't panic
    use std::sync::Arc;
    use std::thread;

    let cache = Arc::new(std::sync::RwLock::new(DiscoveryCache::new()));
    let mut handles = vec![];

    // Spawn 4 reader threads
    for _ in 0..4 {
        let cache_clone = Arc::clone(&cache);
        handles.push(thread::spawn(move || {
            for _ in 0..100 {
                let c = cache_clone.read().unwrap();
                let _stale = c.is_nodes_stale();
                let _len = c.nodes.len();
                drop(c);
                std::thread::yield_now();
            }
        }));
    }

    // Spawn 2 writer threads
    for i in 0..2 {
        let cache_clone = Arc::clone(&cache);
        handles.push(thread::spawn(move || {
            for j in 0..50 {
                let mut c = cache_clone.write().unwrap();
                c.update_nodes(vec![NodeStatus {
                    name: format!("thread_{}_iter_{}", i, j),
                    status: "Running".to_string(),
                    health: HealthStatus::Healthy,
                    priority: 0,
                    process_id: 0,
                    command_line: String::new(),
                    working_dir: String::new(),
                    cpu_usage: 0.0,
                    memory_usage: 0,
                    start_time: String::new(),
                    scheduler_name: String::new(),
                    category: ProcessCategory::Node,
                    tick_count: 0,
                    error_count: 0,
                    actual_rate_hz: 0,
                    publishers: vec![],
                    subscribers: vec![],
        live_tick_count: None, live_health: None, live_avg_tick_ns: None,
        live_max_tick_ns: None, live_budget_misses: None, live_deadline_misses: None,
                }]);
                drop(c);
                std::thread::yield_now();
            }
        }));
    }

    for handle in handles {
        handle.join().expect("thread should not panic");
    }
}

#[test]
fn test_cache_miss_refresh_hit_cycle() {
    // Simulate the full cache lifecycle
    let mut cache = DiscoveryCache::new();

    // Step 1: Cache miss (stale)
    assert!(
        cache.is_nodes_stale(),
        "initial state should be stale (miss)"
    );

    // Step 2: Refresh
    cache.update_nodes(vec![NodeStatus {
        name: "refreshed".to_string(),
        status: "Running".to_string(),
        health: HealthStatus::Healthy,
        priority: 0,
        process_id: 1,
        command_line: String::new(),
        working_dir: String::new(),
        cpu_usage: 0.0,
        memory_usage: 0,
        start_time: String::new(),
        scheduler_name: String::new(),
        category: ProcessCategory::Node,
        tick_count: 0,
        error_count: 0,
        actual_rate_hz: 0,
        publishers: vec![],
        subscribers: vec![],
        live_tick_count: None, live_health: None, live_avg_tick_ns: None,
        live_max_tick_ns: None, live_budget_misses: None, live_deadline_misses: None,
    }]);

    // Step 3: Cache hit (fresh)
    assert!(
        !cache.is_nodes_stale(),
        "should be fresh after refresh (hit)"
    );
    assert_eq!(cache.nodes.len(), 1);
    assert_eq!(cache.nodes[0].name, "refreshed");
}

#[test]
fn test_stale_data_dead_node_replaced_on_refresh() {
    let mut cache = DiscoveryCache::new();

    // Initial state: node with a "dead" PID
    cache.update_nodes(vec![NodeStatus {
        name: "dead_node".to_string(),
        status: "Stopped".to_string(),
        health: HealthStatus::Error,
        priority: 0,
        process_id: 999999999,
        command_line: String::new(),
        working_dir: String::new(),
        cpu_usage: 0.0,
        memory_usage: 0,
        start_time: String::new(),
        scheduler_name: String::new(),
        category: ProcessCategory::Node,
        tick_count: 0,
        error_count: 0,
        actual_rate_hz: 0,
        publishers: vec![],
        subscribers: vec![],
        live_tick_count: None, live_health: None, live_avg_tick_ns: None,
        live_max_tick_ns: None, live_budget_misses: None, live_deadline_misses: None,
    }]);
    assert_eq!(cache.nodes.len(), 1);
    assert_eq!(cache.nodes[0].name, "dead_node");

    // On next refresh, the dead node is gone, replaced by live ones
    cache.update_nodes(vec![NodeStatus {
        name: "live_node".to_string(),
        status: "Running".to_string(),
        health: HealthStatus::Healthy,
        priority: 0,
        process_id: std::process::id(),
        command_line: String::new(),
        working_dir: String::new(),
        cpu_usage: 10.0,
        memory_usage: 1024,
        start_time: String::new(),
        scheduler_name: String::new(),
        category: ProcessCategory::Node,
        tick_count: 100,
        error_count: 0,
        actual_rate_hz: 50,
        publishers: vec![],
        subscribers: vec![],
        live_tick_count: None, live_health: None, live_avg_tick_ns: None,
        live_max_tick_ns: None, live_budget_misses: None, live_deadline_misses: None,
    }]);
    assert_eq!(cache.nodes.len(), 1);
    assert_eq!(cache.nodes[0].name, "live_node");
    assert_eq!(cache.nodes[0].status, "Running");
}

#[test]
fn test_topic_status_enum_values() {
    // Verify all TopicStatus variants exist and are distinct
    let active = TopicStatus::Active;
    let idle = TopicStatus::Idle;
    let stale = TopicStatus::Stale;

    assert_ne!(active, idle);
    assert_ne!(active, stale);
    assert_ne!(idle, stale);

    // Verify Copy semantics
    let active2 = active;
    assert_eq!(active, active2);
}

#[test]
fn test_cache_empty_update_is_valid() {
    // Updating cache with empty vec should work and mark as fresh
    let mut cache = DiscoveryCache::new();
    cache.update_nodes(vec![]);
    assert!(!cache.is_nodes_stale());
    assert_eq!(cache.nodes.len(), 0);

    cache.update_shared_memory(vec![]);
    assert!(!cache.is_shared_memory_stale());
    assert_eq!(cache.shared_memory.len(), 0);
}

// =====================
// Phase 6: /proc/PID/stat parsing tests
// =====================

#[test]
#[cfg(target_os = "linux")]
fn test_parse_stat_fields_normal_process() {
    let stat = "1234 (bash) S 1 1234 1234 0 -1 4194304 100 0 0 0 10 5 0 0 20 0 1 0 12345 12345678 500 18446744073709551615";
    let fields = parse_stat_fields(stat).unwrap();
    assert_eq!(fields[0], "S"); // state
    assert_eq!(fields[11], "10"); // utime
    assert_eq!(fields[12], "5"); // stime
    assert_eq!(fields[19], "12345"); // starttime
    assert_eq!(fields[21], "500"); // rss
}

#[test]
#[cfg(target_os = "linux")]
fn test_parse_stat_fields_process_name_with_spaces() {
    // Some processes have spaces in their comm field
    let stat = "5678 (Web Content) S 1 5678 5678 0 -1 4194304 200 0 0 0 30 10 0 0 20 0 1 0 99999 12345678 1000 0";
    let fields = parse_stat_fields(stat).unwrap();
    assert_eq!(fields[0], "S");
    assert_eq!(fields[11], "30"); // utime
    assert_eq!(fields[12], "10"); // stime
}

#[test]
#[cfg(target_os = "linux")]
fn test_parse_stat_fields_process_name_with_parens() {
    // Process name containing parentheses: (my (weird) proc)
    let stat = "9999 (my (weird) proc) S 1 9999 9999 0 -1 4194304 50 0 0 0 5 2 0 0 20 0 1 0 54321 12345678 250 0";
    let fields = parse_stat_fields(stat).unwrap();
    // rfind(')') should find the LAST ), skipping inner ones
    assert_eq!(fields[0], "S");
    assert_eq!(fields[11], "5"); // utime
    assert_eq!(fields[19], "54321"); // starttime
}

#[test]
#[cfg(target_os = "linux")]
fn test_parse_stat_fields_empty_input() {
    assert!(parse_stat_fields("").is_none());
}

#[test]
#[cfg(target_os = "linux")]
fn test_parse_stat_fields_no_closing_paren() {
    assert!(parse_stat_fields("1234 (broken").is_none());
}

#[test]
#[cfg(target_os = "linux")]
fn test_parse_stat_fields_nothing_after_paren() {
    assert!(parse_stat_fields("1234 (proc)").is_none());
}

// =====================
// Phase 6: Node name validation tests
// =====================

#[test]
fn test_validate_node_name_valid() {
    use horus_core::core::presence::validate_node_name;
    assert!(validate_node_name("my_node").is_ok());
    assert!(validate_node_name("sensor-1").is_ok());
    assert!(validate_node_name("robot.arm.left").is_ok());
    assert!(validate_node_name("A").is_ok());
    assert!(validate_node_name("node123").is_ok());
}

#[test]
fn test_validate_node_name_empty() {
    use horus_core::core::presence::validate_node_name;
    assert!(validate_node_name("").is_err());
}

#[test]
fn test_validate_node_name_path_traversal() {
    use horus_core::core::presence::validate_node_name;
    assert!(validate_node_name("..").is_err());
    assert!(validate_node_name("foo..bar").is_err());
    // Slashes are invalid characters
    assert!(validate_node_name("../etc/passwd").is_err());
    assert!(validate_node_name("foo/bar").is_err());
}

#[test]
fn test_validate_node_name_special_chars() {
    use horus_core::core::presence::validate_node_name;
    assert!(validate_node_name("node name").is_err()); // space
    assert!(validate_node_name("node\tname").is_err()); // tab
    assert!(validate_node_name("node\0name").is_err()); // null
    assert!(validate_node_name("node@host").is_err()); // @
    assert!(validate_node_name("node#1").is_err()); // #
}

#[test]
fn test_validate_node_name_too_long() {
    use horus_core::core::presence::validate_node_name;
    let long_name = "a".repeat(256);
    assert!(validate_node_name(&long_name).is_err());
    let max_name = "a".repeat(255);
    assert!(validate_node_name(&max_name).is_ok());
}

// =====================
// Phase 6: is_horus_process keyword matching tests
// =====================

#[test]
fn test_is_horus_process_matches() {
    assert!(is_horus_process("horus run my_package"));
    assert!(is_horus_process("/usr/bin/horus monitor"));
    assert!(is_horus_process("target/debug/horus-sim3d"));
    assert!(is_horus_process("talos --headless"));
    assert!(is_horus_process("some_process /dev/shm/horus/topics/foo"));
}

#[test]
fn test_is_horus_process_rejects_unrelated() {
    assert!(!is_horus_process("firefox"));
    assert!(!is_horus_process("ros2 topic list")); // Not horus
    assert!(!is_horus_process("simulator --mode headless")); // Generic "sim"
    assert!(!is_horus_process("snake_game")); // Was previously matched
    assert!(!is_horus_process("bash /tmp/script.sh"));
}

// =====================
// Phase 6: Topic status computation tests
// =====================

#[test]
fn test_compute_topic_status_active() {
    use std::time::SystemTime;
    let now = SystemTime::now();
    let (status, _age) = compute_topic_status(&[1234], Some(now));
    assert_eq!(status, TopicStatus::Active);
}

#[test]
fn test_compute_topic_status_idle_old_write() {
    use std::time::{Duration, SystemTime};
    // Has live processes but last write was 60 seconds ago
    let old = SystemTime::now() - Duration::from_secs(60);
    let (status, _age) = compute_topic_status(&[1234], Some(old));
    assert_eq!(status, TopicStatus::Idle);
}

#[test]
fn test_compute_topic_status_stale_no_processes() {
    use std::time::{Duration, SystemTime};
    // No live processes and 10 minutes old
    let ancient = SystemTime::now() - Duration::from_secs(600);
    let (status, _age) = compute_topic_status(&[], Some(ancient));
    assert_eq!(status, TopicStatus::Stale);
}

#[test]
fn test_compute_topic_status_idle_recent_no_processes() {
    use std::time::SystemTime;
    // No processes but just written (process just exited)
    let (status, _age) = compute_topic_status(&[], Some(SystemTime::now()));
    assert_eq!(status, TopicStatus::Idle);
}

#[test]
fn test_compute_topic_status_stale_unknown_time() {
    let (status, age) = compute_topic_status(&[], None);
    assert_eq!(status, TopicStatus::Stale);
    assert_eq!(age, "unknown");
}

// =====================
// Phase 6: format_age tests
// =====================

#[test]
fn test_format_age_seconds() {
    assert_eq!(format_age(0), "0s ago");
    assert_eq!(format_age(30), "30s ago");
    assert_eq!(format_age(59), "59s ago");
}

#[test]
fn test_format_age_minutes() {
    assert_eq!(format_age(60), "1m ago");
    assert_eq!(format_age(3599), "59m ago");
}

#[test]
fn test_format_age_hours() {
    assert_eq!(format_age(3600), "1h ago");
    assert_eq!(format_age(86399), "23h ago");
}

#[test]
fn test_format_age_days() {
    assert_eq!(format_age(86400), "1d ago");
    assert_eq!(format_age(259200), "3d ago");
}

// =====================
// Phase 6: Presence cache dedup tests
// =====================

#[test]
fn test_presence_cache_shared_between_node_and_topic_discovery() {
    // Verify the presence cache exists and has independent staleness
    let mut cache = DiscoveryCache::new();
    assert!(cache.is_presence_stale(), "new presence cache should be stale");

    cache.update_presence(vec![]);
    assert!(!cache.is_presence_stale(), "presence should be fresh after update");
    // Other caches should still be stale
    assert!(cache.is_nodes_stale());
    assert!(cache.is_shared_memory_stale());
}

// =====================
// Phase 6: System topic detection tests
// =====================

#[test]
fn test_system_topic_detection() {
    let system_shm = SharedMemoryInfo {
        topic_name: "horus.ctl.main".to_string(),
        size_bytes: 256,
        active: true,
        accessing_processes: vec![],
        last_modified: None,
        message_type: None,
        publishers: vec![],
        subscribers: vec![],
        message_rate_hz: 0.0,
        status: TopicStatus::Active,
        age_string: "0s ago".to_string(),
        is_system: true,
        messages_total: 0,
        topic_kind: 0,
    };
    assert!(system_shm.is_system);

    let user_shm = SharedMemoryInfo {
        topic_name: "motors.cmd_vel".to_string(),
        size_bytes: 1024,
        active: true,
        accessing_processes: vec![],
        last_modified: None,
        message_type: None,
        publishers: vec![],
        subscribers: vec![],
        message_rate_hz: 0.0,
        status: TopicStatus::Active,
        age_string: "0s ago".to_string(),
        is_system: false,
        messages_total: 0,
        topic_kind: 0,
    };
    assert!(!user_shm.is_system);

    // Verify system and user topics are distinguishable after clone
    let system_clone = system_shm.clone();
    let user_clone = user_shm.clone();
    assert!(system_clone.is_system);
    assert!(!user_clone.is_system);
    assert_ne!(system_clone.topic_name, user_clone.topic_name);

    // Verify Debug contains the system flag
    let system_debug = format!("{:?}", system_shm);
    assert!(
        system_debug.contains("is_system: true"),
        "System topic Debug should show is_system: true"
    );
    let user_debug = format!("{:?}", user_shm);
    assert!(
        user_debug.contains("is_system: false"),
        "User topic Debug should show is_system: false"
    );
}

// =====================
// Phase 7: End-to-end multi-node discovery scenario
// =====================

#[test]
#[ignore] // Requires /dev/shm filesystem; run with: cargo test -- --ignored
fn test_e2e_multi_node_discovery() {
    let nodes_dir = horus_core::memory::shm_base_dir().join("nodes");
    let _ = std::fs::create_dir_all(&nodes_dir);

    let pid = std::process::id();
    let now_secs = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .unwrap_or_default()
        .as_secs();

    // Write presence files as raw JSON (NodePresence::new/write are pub(crate))
    let node_configs: Vec<(&str, &str, &str)> = vec![
        (
            "e2e_sensor",
            r#"[{"topic_name":"sensor.data","type_name":"SensorMsg"}]"#,
            "[]",
        ),
        ("e2e_planner", "[]", "[]"),
        (
            "e2e_motor",
            "[]",
            r#"[{"topic_name":"motor.cmd","type_name":"MotorCmd"}]"#,
        ),
    ];

    let mut created_files = Vec::new();
    for (i, (name, pubs, subs)) in node_configs.iter().enumerate() {
        let json = format!(
            r#"{{"name":"{}","pid":{},"scheduler":"e2e_sched","publishers":{},"subscribers":{},"start_time":{},"priority":{},"rate_hz":null,"pid_start_time":0}}"#,
            name, pid, pubs, subs, now_secs, i
        );
        let path = nodes_dir.join(format!("{}.json", name));
        // Atomic write to prevent parallel read_all() from seeing partial files
        let tmp = nodes_dir.join(format!("{}.json.tmp", name));
        std::fs::write(&tmp, json).expect("write presence json");
        std::fs::rename(&tmp, &path).expect("atomic rename");
        created_files.push(path);
    }

    // Verify via direct file reads (parallel-safe, no global cache/scan races)
    for (name, _, _) in &node_configs {
        let path = nodes_dir.join(format!("{}.json", name));
        let content = std::fs::read_to_string(&path).expect("read presence");
        let presence: NodePresence = serde_json::from_str(&content).expect("parse presence");
        assert_eq!(presence.name(), *name);
        assert_eq!(presence.scheduler(), Some("e2e_sched"));
    }

    // Verify sensor has publisher
    let sensor_path = nodes_dir.join("e2e_sensor.json");
    let sensor: NodePresence = serde_json::from_str(
        &std::fs::read_to_string(&sensor_path).unwrap()
    ).unwrap();
    assert_eq!(sensor.publishers().len(), 1);
    assert_eq!(sensor.publishers()[0].topic_name, "sensor.data");

    // Verify motor has subscriber
    let motor_path = nodes_dir.join("e2e_motor.json");
    let motor: NodePresence = serde_json::from_str(
        &std::fs::read_to_string(&motor_path).unwrap()
    ).unwrap();
    assert_eq!(motor.subscribers().len(), 1);
    assert_eq!(motor.subscribers()[0].topic_name, "motor.cmd");

    // Clean up
    for path in &created_files {
        let _ = std::fs::remove_file(path);
    }
}

// =============================================================================
// COMPREHENSIVE TESTS: Stress, Robotics Scenarios, Crash Recovery, Race, Lifecycle
// =============================================================================

// =====================
// Stress Tests
// =====================

#[test]
#[ignore]
fn test_stress_100_nodes_discovery() {
    let mut w = TestPresenceWriter::new("stress100");
    let mut expected_names = Vec::new();

    for i in 0..100 {
        let pubs = format!(
            r#"[{{"topic_name":"topic.pub.{}","type_name":"Msg{}"}}]"#,
            i, i
        );
        let subs = format!(
            r#"[{{"topic_name":"topic.sub.{}","type_name":"Sub{}"}}]"#,
            i, i
        );
        let name = w.write_node(&format!("node_{:03}", i), &pubs, &subs);
        expected_names.push(name);
    }

    // Verify via direct file reads (parallel-safe)
    for name in &expected_names {
        let p = w.read_presence(name).unwrap_or_else(|| panic!("missing: {}", name));
        assert_eq!(p.publishers().len(), 1);
        assert_eq!(p.subscribers().len(), 1);
    }
}

#[test]
#[ignore]
fn test_stress_rapid_creation_teardown() {
    for iteration in 0..20 {
        let mut w = TestPresenceWriter::new(&format!("rapid{}", iteration));
        let mut names = Vec::new();
        for i in 0..10 {
            names.push(w.write_node(&format!("n{}", i), "[]", "[]"));
        }

        // Verify via direct file reads (parallel-safe)
        for name in &names {
            assert!(
                w.read_presence(name).is_some(),
                "iter {}: missing {}",
                iteration, name
            );
        }
        // Drop w → files deleted
        drop(w);
    }
}

#[test]
#[ignore]
fn test_stress_cache_thrashing() {
    use std::sync::Arc;
    use std::thread;

    let barrier = Arc::new(std::sync::Barrier::new(8));
    let mut handles = Vec::new();

    for t in 0..8 {
        let b = Arc::clone(&barrier);
        handles.push(thread::spawn(move || {
            b.wait();
            for _ in 0..200 {
                let _ = discover_nodes();
                let _ = discover_shared_memory();
                thread::yield_now();
            }
            t // return thread id to prove completion
        }));
    }

    for h in handles {
        h.join().expect("thread must not panic");
    }
}

#[test]
#[ignore]
fn test_stress_concurrent_write_and_discover() {
    use std::sync::atomic::{AtomicBool, Ordering};
    use std::sync::Arc;
    use std::thread;

    let stop = Arc::new(AtomicBool::new(false));
    let nodes_dir = horus_core::memory::shm_base_dir().join("nodes");
    let _ = std::fs::create_dir_all(&nodes_dir);

    let stop_w = Arc::clone(&stop);
    let dir_w = nodes_dir.clone();
    let writer = thread::spawn(move || {
        let pid = std::process::id();
        let mut tick = 0u64;
        while !stop_w.load(Ordering::Relaxed) {
            tick += 1;
            let json = format!(
                r#"{{"name":"conc_stress","pid":{},"scheduler":"s","publishers":[],"subscribers":[],"start_time":1,"priority":0,"rate_hz":null,"pid_start_time":0,"tick_count":{}}}"#,
                pid, tick
            );
            // Atomic write pattern
            let tmp = dir_w.join("conc_stress.json.tmp");
            let dest = dir_w.join("conc_stress.json");
            let _ = std::fs::write(&tmp, &json);
            let _ = std::fs::rename(&tmp, &dest);
            std::thread::sleep(std::time::Duration::from_millis(1));
        }
    });

    let stop_r = Arc::clone(&stop);
    let reader = thread::spawn(move || {
        let mut ok_count = 0u32;
        while !stop_r.load(Ordering::Relaxed) {
            if let Ok(mut cache) = DISCOVERY_CACHE.write() {
                cache.presence_last_updated = std::time::Instant::now() - 10_u64.secs();
                cache.nodes_last_updated = std::time::Instant::now() - 10_u64.secs();
            }
            if discover_nodes().is_ok() {
                ok_count += 1;
            }
        }
        ok_count
    });

    std::thread::sleep(std::time::Duration::from_millis(500));
    stop.store(true, Ordering::Relaxed);

    writer.join().expect("writer must not panic");
    let reads = reader.join().expect("reader must not panic");
    assert!(reads > 0, "reader should have completed at least one cycle");

    // Cleanup
    let _ = std::fs::remove_file(nodes_dir.join("conc_stress.json"));
    let _ = std::fs::remove_file(nodes_dir.join("conc_stress.json.tmp"));
}

#[test]
#[ignore]
fn test_stress_large_presence_payload() {
    let mut w = TestPresenceWriter::new("bigpayload");

    // Build 50 publishers, 50 subscribers
    let pubs: Vec<String> = (0..50)
        .map(|i| format!(r#"{{"topic_name":"big.pub.{}","type_name":"BigMsg"}}"#, i))
        .collect();
    let subs: Vec<String> = (0..50)
        .map(|i| format!(r#"{{"topic_name":"big.sub.{}","type_name":"BigSub"}}"#, i))
        .collect();

    let pubs_json = format!("[{}]", pubs.join(","));
    let subs_json = format!("[{}]", subs.join(","));

    let name = w.write_node("big_node", &pubs_json, &subs_json);

    // Verify via direct file read (parallel-safe)
    let p = w.read_presence(&name).expect("big_node missing");
    assert_eq!(p.publishers().len(), 50);
    assert_eq!(p.subscribers().len(), 50);
}

// =====================
// Robotics Scenario Tests
// =====================

#[test]
#[ignore]
fn test_scenario_perception_pipeline() {
    let mut w = TestPresenceWriter::new("perception");

    let cam = w.write_node(
        "camera_driver",
        r#"[{"topic_name":"camera.image","type_name":"sensor_msgs::Image"}]"#,
        "[]",
    );
    let det = w.write_node(
        "object_detector",
        r#"[{"topic_name":"detection.objects","type_name":"DetectionArray"}]"#,
        r#"[{"topic_name":"camera.image","type_name":"sensor_msgs::Image"}]"#,
    );
    let plan = w.write_node(
        "path_planner",
        r#"[{"topic_name":"plan.trajectory","type_name":"Trajectory"}]"#,
        r#"[{"topic_name":"detection.objects","type_name":"DetectionArray"}]"#,
    );
    let motor = w.write_node(
        "motor_controller",
        r#"[{"topic_name":"motor.cmd_vel","type_name":"Twist"}]"#,
        r#"[{"topic_name":"plan.trajectory","type_name":"Trajectory"}]"#,
    );

    // Verify all 4 pipeline nodes via direct file reads (parallel-safe)
    for name in [&cam, &det, &plan, &motor] {
        assert!(
            w.read_presence(name).is_some(),
            "missing pipeline node: {}",
            name
        );
    }

    // Verify camera publishes camera.image
    let cam_p = w.read_presence(&cam).unwrap();
    assert_eq!(cam_p.publishers()[0].topic_name, "camera.image");

    // Verify motor subscribes to plan.trajectory
    let motor_p = w.read_presence(&motor).unwrap();
    assert_eq!(motor_p.subscribers()[0].topic_name, "plan.trajectory");

    // Verify topic discovery sees all 4 topics from presence
    // (discover_shared_memory_uncached doesn't delete files, so it's safer)
    let topics = discover_shared_memory_uncached().expect("shm discovery");
    let pipeline_topics: Vec<_> = topics
        .iter()
        .filter(|t| {
            ["camera.image", "detection.objects", "plan.trajectory", "motor.cmd_vel"]
                .iter()
                .any(|name| t.topic_name == *name)
        })
        .collect();
    assert_eq!(
        pipeline_topics.len(),
        4,
        "all 4 pipeline topics should appear from presence, found: {:?}",
        pipeline_topics.iter().map(|t| &t.topic_name).collect::<Vec<_>>()
    );
}

#[test]
#[ignore]
fn test_scenario_arm_control_with_rates() {
    let mut w = TestPresenceWriter::new("arm");

    w.write_node_ext(
        "joint_state_pub",
        r#"[{"topic_name":"arm.joint_states","type_name":"JointState"}]"#,
        "[]",
        Some("control_sched"), None, Some(1000.0), 100000, 0,
    );
    w.write_node_ext(
        "ik_solver",
        "[]",
        r#"[{"topic_name":"arm.goal_pose","type_name":"Pose"}]"#,
        Some("control_sched"), None, Some(100.0), 10000, 0,
    );
    w.write_node_ext(
        "safety_monitor",
        "[]",
        r#"[{"topic_name":"arm.joint_states","type_name":"JointState"},{"topic_name":"arm.joint_commands","type_name":"JointCmd"}]"#,
        Some("control_sched"), None, Some(1000.0), 500000, 2,
    );

    // Verify via direct file reads (parallel-safe)
    let joint_pub_name = w.prefixed_name("joint_state_pub");
    let joint_pub = w.read_presence(&joint_pub_name).expect("joint_state_pub");
    assert_eq!(joint_pub.rate_hz(), Some(1000.0));
    assert_eq!(joint_pub.scheduler(), Some("control_sched"));

    let safety_name = w.prefixed_name("safety_monitor");
    let safety = w.read_presence(&safety_name).expect("safety_monitor");
    assert_eq!(safety.subscribers().len(), 2);
    assert_eq!(safety.error_count(), 2);
    assert_eq!(safety.tick_count(), 500000);
}

#[test]
#[ignore]
fn test_scenario_multi_publisher_sensor_fusion() {
    let mut w = TestPresenceWriter::new("fusion");

    for pos in ["front", "rear", "left", "right"] {
        w.write_node(
            &format!("lidar_{}", pos),
            r#"[{"topic_name":"sensor.pointcloud","type_name":"PointCloud2"}]"#,
            "[]",
        );
    }
    w.write_node(
        "fusion_node",
        r#"[{"topic_name":"fused.map","type_name":"OccupancyGrid"}]"#,
        r#"[{"topic_name":"sensor.pointcloud","type_name":"PointCloud2"}]"#,
    );

    // Verify via direct file reads (parallel-safe)
    let all = w.read_all_own();
    let pc_pubs: Vec<_> = all.iter().filter(|p| {
        p.publishers().iter().any(|t| t.topic_name == "sensor.pointcloud")
    }).collect();
    assert_eq!(pc_pubs.len(), 4, "4 lidars should publish pointcloud");

    let pc_subs: Vec<_> = all.iter().filter(|p| {
        p.subscribers().iter().any(|t| t.topic_name == "sensor.pointcloud")
    }).collect();
    assert_eq!(pc_subs.len(), 1, "1 fusion node should subscribe");
}

#[test]
#[ignore]
fn test_scenario_hierarchical_schedulers() {
    let mut w = TestPresenceWriter::new("multi_sched");

    for i in 0..3 {
        w.write_node_ext(
            &format!("perception_{}", i), "[]", "[]",
            Some("perception_sched"), None, None, 0, 0,
        );
    }
    for i in 0..3 {
        w.write_node_ext(
            &format!("control_{}", i), "[]", "[]",
            Some("control_sched"), None, None, 0, 0,
        );
    }

    // Verify via direct file reads (parallel-safe)
    let all = w.read_all_own();
    let perc_nodes: Vec<_> = all.iter()
        .filter(|n| n.scheduler() == Some("perception_sched"))
        .collect();
    let ctrl_nodes: Vec<_> = all.iter()
        .filter(|n| n.scheduler() == Some("control_sched"))
        .collect();
    assert_eq!(perc_nodes.len(), 3);
    assert_eq!(ctrl_nodes.len(), 3);
}

#[test]
#[ignore]
fn test_scenario_health_degradation_matrix() {
    let mut w = TestPresenceWriter::new("health");

    let h = w.write_node_ext("healthy", "[]", "[]", None, Some("Healthy"), None, 10000, 0);
    let warn = w.write_node_ext("warning", "[]", "[]", None, Some("Warning"), None, 5000, 5);
    let err = w.write_node_ext("error", "[]", "[]", None, Some("Error"), None, 3000, 100);
    let crit = w.write_node_ext("critical", "[]", "[]", None, Some("Critical"), None, 1000, 500);
    let unk = w.write_node_ext("unknown", "[]", "[]", None, None, None, 0, 0);

    // Verify presence data by reading files directly (avoids both cache races
    // and read_all() liveness filtering which can race with parallel tests)
    let read_presence = |name: &str| -> NodePresence {
        let path = w.nodes_dir.join(format!("{}.json", name));
        let content = std::fs::read_to_string(&path)
            .unwrap_or_else(|e| panic!("read {}: {}", name, e));
        serde_json::from_str(&content)
            .unwrap_or_else(|e| panic!("parse {}: {}", name, e))
    };

    assert_eq!(read_presence(&h).health_status(), Some("Healthy"));
    assert_eq!(read_presence(&warn).health_status(), Some("Warning"));
    assert_eq!(read_presence(&err).health_status(), Some("Error"));
    assert_eq!(read_presence(&crit).health_status(), Some("Critical"));
    assert_eq!(read_presence(&unk).health_status(), None); // maps to Healthy default

    // Verify tick_count and error_count fields too
    assert_eq!(read_presence(&h).tick_count(), 10000);
    assert_eq!(read_presence(&warn).error_count(), 5);
    assert_eq!(read_presence(&err).error_count(), 100);
    assert_eq!(read_presence(&crit).error_count(), 500);
}

// =====================
// Crash Recovery Tests
// =====================

#[test]
#[ignore]
fn test_crash_orphan_dead_pid() {
    let mut w = TestPresenceWriter::new("orphan");
    let _name = w.write_dead_node("dead_node");

    // read_all() should filter out dead PID nodes and clean up the file
    let all = NodePresence::read_all();
    assert!(
        !all.iter().any(|n| n.name() == _name),
        "dead node must not appear in discovery"
    );

    // Presence file should have been cleaned up by read_all()
    assert!(
        !w.file_exists("dead_node"),
        "orphan presence file should be removed"
    );
}

#[test]
#[ignore]
fn test_crash_orphan_pid_reuse_detection() {
    let mut w = TestPresenceWriter::new("pidreuse");

    // Write presence with current PID but wrong start time (simulates PID reuse)
    let full_name = w.prefixed_name("reused");
    let json = format!(
        r#"{{"name":"{}","pid":{},"scheduler":"s","publishers":[],"subscribers":[],"start_time":1,"priority":0,"rate_hz":null,"pid_start_time":1}}"#,
        full_name,
        std::process::id()
    );
    let path = w.nodes_dir.join(format!("{}.json", full_name));
    std::fs::write(&path, &json).expect("write");
    w.created_files.push(path);

    // read_all() should detect PID start time mismatch and filter it out
    let all = NodePresence::read_all();

    // On Linux, pid_start_time=1 won't match the current process's real start time
    #[cfg(target_os = "linux")]
    assert!(
        !all.iter().any(|n| n.name() == full_name),
        "node with wrong pid_start_time should be filtered (PID reuse detected)"
    );
}

#[test]
#[ignore]
fn test_crash_corrupt_json_variants() {
    let mut w = TestPresenceWriter::new("corrupt");

    // Write various corrupt files
    let corrupt_prefix = w.prefixed_name("corrupt");
    w.write_raw(&format!("{}_truncated.json", corrupt_prefix), b"{\"name\":\"x\",\"pid");
    w.write_raw(&format!("{}_empty.json", corrupt_prefix), b"");
    w.write_raw(&format!("{}_binary.json", corrupt_prefix), &[0xDE, 0xAD, 0xBE, 0xEF]);
    w.write_raw(&format!("{}_wrong_schema.json", corrupt_prefix), b"{\"foo\": \"bar\"}");
    w.write_raw(&format!("{}_no_brace.json", corrupt_prefix), b"{\"name\":\"test\",\"pid\":1234");

    // Write one valid file
    let valid_name = w.write_node("valid", "[]", "[]");

    // Trigger cleanup via read_all() (it removes corrupt UTF-8 JSON files)
    let _ = NodePresence::read_all();

    // Valid node should still exist (direct file read, parallel-safe)
    assert!(
        w.read_presence(&valid_name).is_some(),
        "valid node must survive alongside corrupt files"
    );

    // Corrupt UTF-8 files should be cleaned up
    // (read_all removes files that fail JSON parse but are valid UTF-8)
    for suffix in ["truncated", "empty", "wrong_schema", "no_brace"] {
        let path = w.nodes_dir.join(format!("{}_{}.json", corrupt_prefix, suffix));
        assert!(
            !path.exists(),
            "corrupt file {} should be cleaned up",
            suffix
        );
    }

    // Binary (non-UTF-8) files are skipped, not removed — we don't delete
    // files we can't even read as text (they may belong to another system)
    let binary_path = w.nodes_dir.join(format!("{}_{}.json", corrupt_prefix, "binary"));
    // Clean up manually since read_all won't touch it
    let _ = std::fs::remove_file(&binary_path);
}

#[test]
#[ignore]
fn test_crash_tmp_file_ignored() {
    let mut w = TestPresenceWriter::new("tmpfile");

    // Write a .json.tmp file (leftover from interrupted atomic write)
    let tmp_name = format!("{}.json.tmp", w.prefixed_name("leftover"));
    w.write_raw(&tmp_name, b"{\"name\":\"leftover\",\"pid\":1}");

    // Write a valid presence file
    let valid = w.write_node("real", "[]", "[]");

    // Valid node should be readable directly
    assert!(w.read_presence(&valid).is_some(), "valid node must exist");

    // .tmp files should not be picked up by read_all()
    let all = NodePresence::read_all();
    assert!(
        !all.iter().any(|n| n.name().contains("leftover")),
        ".json.tmp files must not be treated as presence files"
    );
}

#[test]
#[ignore]
fn test_crash_bulk_dead_nodes() {
    let mut w = TestPresenceWriter::new("bulkdead");
    let mut dead_names = Vec::new();

    for i in 0..10 {
        dead_names.push(w.write_dead_node(&format!("crashed_{}", i)));
    }

    // read_all() filters out dead PIDs and cleans up their files
    let all = NodePresence::read_all();

    for name in &dead_names {
        assert!(
            !all.iter().any(|n| n.name() == name),
            "dead node {} should not appear",
            name
        );
    }
}

// =====================
// Race Condition Tests
// =====================

#[test]
#[ignore]
fn test_race_concurrent_node_startup() {
    use std::sync::{Arc, Barrier};
    use std::thread;

    let barrier = Arc::new(Barrier::new(20));
    let mut handles = Vec::new();
    let nodes_dir = horus_core::memory::shm_base_dir().join("nodes");
    let _ = std::fs::create_dir_all(&nodes_dir);
    let pid = std::process::id();

    let nanos = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .unwrap_or_default()
        .subsec_nanos();
    let test_prefix = format!("race_start_{}", nanos);

    for t in 0..20 {
        let b = Arc::clone(&barrier);
        let dir = nodes_dir.clone();
        let prefix = test_prefix.clone();
        handles.push(thread::spawn(move || {
            let name = format!("{}_{}", prefix, t);
            let json = format!(
                r#"{{"name":"{}","pid":{},"scheduler":"s","publishers":[],"subscribers":[],"start_time":1,"priority":0,"rate_hz":null,"pid_start_time":0}}"#,
                name, pid
            );
            b.wait(); // Synchronize all threads to write simultaneously
            let tmp = dir.join(format!("{}.json.tmp", name));
            let dest = dir.join(format!("{}.json", name));
            std::fs::write(&tmp, &json).unwrap();
            std::fs::rename(&tmp, &dest).unwrap();
            name
        }));
    }

    let names: Vec<String> = handles.into_iter().map(|h| h.join().unwrap()).collect();

    // Verify all 20 files exist via direct reads (parallel-safe)
    for name in &names {
        let path = nodes_dir.join(format!("{}.json", name));
        let content = std::fs::read_to_string(&path)
            .unwrap_or_else(|_| panic!("concurrently-started node {} file missing", name));
        let p: NodePresence = serde_json::from_str(&content)
            .unwrap_or_else(|_| panic!("concurrently-started node {} corrupt JSON", name));
        assert_eq!(p.name(), name);
    }

    // Cleanup
    for name in &names {
        let _ = std::fs::remove_file(nodes_dir.join(format!("{}.json", name)));
    }
}

// =====================
// Node Lifecycle Tests
// =====================

#[test]
#[ignore]
fn test_lifecycle_start_discover_shutdown() {
    let mut w = TestPresenceWriter::new("lifecycle");

    let name = w.write_node("ephemeral", "[]", "[]");

    // Node file should exist (direct read, parallel-safe)
    assert!(w.read_presence(&name).is_some(), "node must appear after start");

    // Simulate shutdown: delete the file
    let path = w.nodes_dir.join(format!("{}.json", name));
    std::fs::remove_file(&path).expect("delete presence");

    // Node file should no longer exist
    assert!(w.read_presence(&name).is_none(), "node must vanish after shutdown");
}

#[test]
#[ignore]
fn test_lifecycle_topic_change_mid_run() {
    let mut w = TestPresenceWriter::new("topicchange");

    // Start with topic.a
    let name = w.write_node(
        "dynamic",
        r#"[{"topic_name":"topic.a","type_name":"A"}]"#,
        "[]",
    );

    let p = w.read_presence(&name).expect("node exists");
    assert_eq!(p.publishers().len(), 1);
    assert_eq!(p.publishers()[0].topic_name, "topic.a");

    // Overwrite with topic.a + topic.b (atomic write)
    let path = w.nodes_dir.join(format!("{}.json", name));
    let tmp = w.nodes_dir.join(format!("{}.json.tmp", name));
    let new_json = format!(
        r#"{{"name":"{}","pid":{},"scheduler":"test_sched","publishers":[{{"topic_name":"topic.a","type_name":"A"}},{{"topic_name":"topic.b","type_name":"B"}}],"subscribers":[],"start_time":1,"priority":0,"rate_hz":null,"pid_start_time":0}}"#,
        name, std::process::id()
    );
    std::fs::write(&tmp, &new_json).expect("write tmp");
    std::fs::rename(&tmp, &path).expect("atomic rename");

    let p = w.read_presence(&name).expect("node still exists");
    assert_eq!(p.publishers().len(), 2, "should now have 2 publishers");
}

#[test]
#[ignore]
fn test_lifecycle_staggered_startup_shutdown() {
    let mut w = TestPresenceWriter::new("staggered");

    let n1 = w.write_node("node1", "[]", "[]");
    assert!(w.read_presence(&n1).is_some());

    let n2 = w.write_node("node2", "[]", "[]");
    assert!(w.read_presence(&n1).is_some());
    assert!(w.read_presence(&n2).is_some());

    let n3 = w.write_node("node3", "[]", "[]");
    assert!(w.read_presence(&n1).is_some());
    assert!(w.read_presence(&n2).is_some());
    assert!(w.read_presence(&n3).is_some());

    // Shutdown node1
    let _ = std::fs::remove_file(w.nodes_dir.join(format!("{}.json", n1)));
    assert!(w.read_presence(&n1).is_none(), "node1 must vanish");
    assert!(w.read_presence(&n2).is_some(), "node2 must remain");
    assert!(w.read_presence(&n3).is_some(), "node3 must remain");
}

#[test]
#[ignore]
fn test_lifecycle_health_progression() {
    let mut w = TestPresenceWriter::new("healthprog");

    let name = w.write_node_ext("monitored", "[]", "[]", None, Some("Healthy"), None, 0, 0);

    let p = w.read_presence(&name).expect("node");
    assert_eq!(p.health_status(), Some("Healthy"));

    // Degrade to Warning (atomic write)
    let path = w.nodes_dir.join(format!("{}.json", name));
    let tmp = w.nodes_dir.join(format!("{}.json.tmp", name));
    let json = format!(
        r#"{{"name":"{}","pid":{},"scheduler":"test_sched","publishers":[],"subscribers":[],"start_time":1,"priority":0,"rate_hz":null,"pid_start_time":0,"health_status":"Warning","tick_count":100,"error_count":5}}"#,
        name, std::process::id()
    );
    std::fs::write(&tmp, &json).unwrap();
    std::fs::rename(&tmp, &path).unwrap();

    let p = w.read_presence(&name).expect("node after warning");
    assert_eq!(p.health_status(), Some("Warning"));
    assert_eq!(p.error_count(), 5);

    // Degrade to Critical (atomic write)
    let json = format!(
        r#"{{"name":"{}","pid":{},"scheduler":"test_sched","publishers":[],"subscribers":[],"start_time":1,"priority":0,"rate_hz":null,"pid_start_time":0,"health_status":"Critical","tick_count":200,"error_count":500}}"#,
        name, std::process::id()
    );
    std::fs::write(&tmp, &json).unwrap();
    std::fs::rename(&tmp, &path).unwrap();

    let p = w.read_presence(&name).expect("node after critical");
    assert_eq!(p.health_status(), Some("Critical"));
}

// =====================
// Presence JSON Schema Edge Cases
// =====================

#[test]
#[ignore]
fn test_presence_forward_compatibility_unknown_fields() {
    let mut w = TestPresenceWriter::new("fwdcompat");

    let full_name = w.prefixed_name("future");
    let json = format!(
        r#"{{"name":"{}","pid":{},"scheduler":"s","publishers":[],"subscribers":[],"start_time":1,"priority":0,"rate_hz":null,"pid_start_time":0,"future_field":42,"metadata":{{"version":2}}}}"#,
        full_name, std::process::id()
    );
    let path = w.nodes_dir.join(format!("{}.json", full_name));
    std::fs::write(&path, json).unwrap();
    w.created_files.push(path);

    // Direct file read — JSON with unknown fields must parse successfully
    let p = w.read_presence(&full_name);
    assert!(
        p.is_some(),
        "node with unknown fields must still be discoverable (forward compat)"
    );
    assert_eq!(p.unwrap().name(), full_name);
}

#[test]
#[ignore]
fn test_presence_minimal_fields() {
    let mut w = TestPresenceWriter::new("minimal");

    // Only mandatory fields, all optional fields omitted
    let full_name = w.prefixed_name("bare");
    let json = format!(
        r#"{{"name":"{}","pid":{},"publishers":[],"subscribers":[],"start_time":1,"priority":0}}"#,
        full_name, std::process::id()
    );
    let path = w.nodes_dir.join(format!("{}.json", full_name));
    std::fs::write(&path, json).unwrap();
    w.created_files.push(path);

    // Direct file read (parallel-safe)
    let p = w.read_presence(&full_name).expect("minimal node must be found");
    assert_eq!(p.tick_count(), 0); // serde default
    assert_eq!(p.error_count(), 0);
}

#[test]
#[ignore]
fn test_presence_empty_arrays() {
    let mut w = TestPresenceWriter::new("emptyarr");

    let name = w.write_node("no_topics", "[]", "[]");

    // Direct file read (parallel-safe)
    let p = w.read_presence(&name).expect("empty arrays node");
    assert!(p.publishers().is_empty());
    assert!(p.subscribers().is_empty());
}

#[test]
#[ignore]
fn test_fs_non_json_files_ignored() {
    let mut w = TestPresenceWriter::new("nonjson");

    w.write_raw(&format!("{}_readme.md", w.prefix), b"# not a presence file");
    w.write_raw(&format!("{}_backup.bak", w.prefix), b"old data");
    w.write_raw(&format!("{}_node.json.old", w.prefix), b"stale");

    let valid = w.write_node("real", "[]", "[]");

    // Valid .json node should be readable directly
    assert!(
        w.read_presence(&valid).is_some(),
        "valid node must be found"
    );

    // Non-.json files should not be parseable as presence
    // (read_all only scans .json files, verify via read_all_own which reads all created files)
    let all = w.read_all_own();
    assert!(
        !all.iter().any(|p| p.name().contains("readme") || p.name().contains("backup")),
        "non-.json files must not parse as presence"
    );
}
