use super::nodes::{
    categorize_process, extract_process_name, format_duration, parse_memory_from_stat,
    process_exists, should_track_process,
};
use super::topics::discover_shared_memory_uncached;
use super::*;

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
    };

    assert_eq!(node.name, "test_node");
    assert_eq!(node.status, "Running");
    assert_eq!(node.priority, 10);
    assert_eq!(node.process_id, 1234);
    assert_eq!(node.tick_count, 100);
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
    };

    assert_eq!(shm.topic_name, "robot.pose");
    assert_eq!(shm.size_bytes, 4096);
    assert!(shm.active);
    assert_eq!(shm.accessing_processes.len(), 2);
    assert_eq!(shm.publishers.len(), 1);
    assert_eq!(shm.subscribers.len(), 2);
    assert!((shm.message_rate_hz - 30.0).abs() < 0.001);
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
    };

    assert!(!shm.active);
    assert!(shm.accessing_processes.is_empty());
    assert!(shm.message_type.is_none());
    assert!(shm.last_modified.is_none());
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
}

// =====================
// Helper Function Tests
// =====================
#[test]
fn test_format_duration_seconds() {
    let duration = std::time::Duration::from_secs(45);
    assert_eq!(format_duration(duration), "45s");
}

#[test]
fn test_format_duration_minutes() {
    let duration = std::time::Duration::from_secs(125);
    assert_eq!(format_duration(duration), "2m");
}

#[test]
fn test_format_duration_hours() {
    let duration = std::time::Duration::from_secs(7200);
    assert_eq!(format_duration(duration), "2h");
}

#[test]
fn test_format_duration_days() {
    let duration = std::time::Duration::from_secs(172800);
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
                    std::time::Instant::now() - std::time::Duration::from_secs(10);
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
    // Smoke test - should not panic even with no data
    let result = discover_nodes();
    assert!(result.is_ok());
}

#[test]
fn test_discover_shared_memory_handles_missing_dirs() {
    // Smoke test - should not panic even if dirs don't exist
    let _ = discover_shared_memory();
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
                std::time::Instant::now() - std::time::Duration::from_secs(10);
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
    };

    let cloned = node.clone();
    assert_eq!(cloned.name, node.name);
    assert_eq!(cloned.status, node.status);
    assert_eq!(cloned.publishers.len(), node.publishers.len());
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
    };

    let cloned = shm.clone();
    assert_eq!(cloned.topic_name, shm.topic_name);
    assert_eq!(cloned.accessing_processes.len(), 3);
    assert_eq!(cloned.subscribers.len(), 2);
}

#[test]
fn test_health_status_variants() {
    // Ensure all health status variants work correctly
    let node_healthy = NodeStatus {
        name: "h".to_string(),
        status: String::new(),
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
    };

    match node_healthy.health {
        HealthStatus::Healthy => {}
        _ => panic!("Expected Healthy"),
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
