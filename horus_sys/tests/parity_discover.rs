//! Discovery Module Behavioral Parity Tests
//!
//! Verifies find_nodes, find_topics, process_info, list_pids
//! produce identical observable behavior on all platforms.

use horus_sys::discover::*;
use horus_sys::shm;

// ═══════════════════════════════════════════════════════════════════════════
// process_info
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn test_process_info_current_has_cmdline() {
    let info = process_info(std::process::id()).expect("should get current process info");
    assert_eq!(info.pid, std::process::id());
    assert!(!info.cmdline.is_empty(), "cmdline should be populated");
}

#[test]
fn test_process_info_current_has_memory() {
    let info = process_info(std::process::id()).unwrap();
    assert!(
        info.memory_kb > 0,
        "current process should have memory usage"
    );
}

#[test]
fn test_process_info_dead_pid() {
    let result = process_info(99_999_999);
    // Dead PID should either error or return empty defaults
    match result {
        Ok(info) => assert!(
            info.cmdline.is_empty(),
            "dead PID should have empty cmdline"
        ),
        Err(_) => {} // error is also acceptable
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// list_pids
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn test_list_pids_includes_current() {
    let pids = list_pids();
    assert!(
        pids.contains(&std::process::id()),
        "list_pids must include current process on all platforms"
    );
}

#[test]
fn test_list_pids_has_reasonable_count() {
    let pids = list_pids();
    assert!(pids.len() > 1, "system should have multiple processes");
    assert!(pids.len() < 1_000_000, "sanity: not millions");
}

#[test]
fn test_is_process_alive_current() {
    assert!(is_process_alive(std::process::id()));
}

#[test]
fn test_is_process_alive_dead() {
    assert!(!is_process_alive(99_999_999));
}

// ═══════════════════════════════════════════════════════════════════════════
// find_nodes
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn test_find_nodes_with_presence_file() {
    let nodes_dir = shm::shm_nodes_dir();
    std::fs::create_dir_all(&nodes_dir).unwrap();

    // Write presence file using horus_core format (topic_name field)
    let presence = serde_json::json!({
        "name": "parity_test_node",
        "pid": std::process::id(),
        "scheduler": "test_scheduler",
        "publishers": [{"topic_name": "cmd_vel", "type_name": "CmdVel"}],
        "subscribers": [{"topic_name": "odom", "type_name": "Odometry"}],
        "priority": 10,
        "rate_hz": 100.0,
        "health_status": "Healthy",
        "tick_count": 42,
        "error_count": 0,
        "services": ["get_state"],
        "actions": ["navigate"],
    });
    let path = nodes_dir.join("parity_test_node.json");
    std::fs::write(&path, serde_json::to_string(&presence).unwrap()).unwrap();

    let nodes = find_nodes();
    let node = nodes.iter().find(|n| n.name == "parity_test_node");
    assert!(
        node.is_some(),
        "presence file should be discovered on all platforms"
    );

    let node = node.unwrap();
    assert_eq!(node.pid, std::process::id());
    // Verify TopicRef alias works: "topic_name" in JSON → "topic" in struct
    assert_eq!(node.publishers.len(), 1);
    assert_eq!(
        node.publishers[0].topic, "cmd_vel",
        "topic_name alias must deserialize to topic field"
    );
    assert_eq!(node.subscribers.len(), 1);
    assert_eq!(node.subscribers[0].topic, "odom");
    assert_eq!(node.priority, 10);
    assert_eq!(node.tick_count, 42);
    // Process enrichment should populate cmdline on all platforms
    assert!(!node.cmdline.is_empty(), "cmdline should be enriched");

    let _ = std::fs::remove_file(&path);
}

#[test]
fn test_find_nodes_skips_dead_pid() {
    let nodes_dir = shm::shm_nodes_dir();
    std::fs::create_dir_all(&nodes_dir).unwrap();

    let presence = serde_json::json!({
        "name": "dead_parity_node",
        "pid": 99999999,
    });
    let path = nodes_dir.join("dead_parity_node.json");
    std::fs::write(&path, serde_json::to_string(&presence).unwrap()).unwrap();

    let nodes = find_nodes();
    assert!(
        !nodes.iter().any(|n| n.name == "dead_parity_node"),
        "dead PID should be filtered on all platforms"
    );

    let _ = std::fs::remove_file(&path);
}

#[test]
fn test_find_nodes_skips_non_json() {
    let nodes_dir = shm::shm_nodes_dir();
    std::fs::create_dir_all(&nodes_dir).unwrap();

    let path = nodes_dir.join("not_a_node.txt");
    std::fs::write(&path, "hello").unwrap();

    let nodes = find_nodes();
    assert!(
        !nodes.iter().any(|n| n.name == "not_a_node"),
        "non-JSON files should be skipped"
    );

    let _ = std::fs::remove_file(&path);
}

// ═══════════════════════════════════════════════════════════════════════════
// find_topics
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn test_find_topics_discovers_meta_file() {
    shm::write_topic_meta("parity_lidar", 8192).unwrap();

    let topics = find_topics();
    assert!(
        topics.iter().any(|t| t.name == "parity_lidar"),
        ".meta file should be discoverable on all platforms"
    );

    shm::remove_topic_meta("parity_lidar");
}

#[test]
fn test_find_topics_checks_creator_liveness() {
    let dir = shm::shm_topics_dir();
    std::fs::create_dir_all(&dir).unwrap();

    let meta = shm::TopicMeta {
        name: "parity_dead_topic".into(),
        size: 1024,
        creator_pid: 99999999,
        created_at: 0,
    };
    std::fs::write(
        dir.join("parity_dead_topic.meta"),
        serde_json::to_string(&meta).unwrap(),
    )
    .unwrap();

    let topics = find_topics();
    let topic = topics.iter().find(|t| t.name == "parity_dead_topic");
    assert!(topic.is_some(), "dead topic should still be listed");
    assert!(
        !topic.unwrap().is_alive,
        "dead creator should mark topic not alive on all platforms"
    );

    let _ = std::fs::remove_file(dir.join("parity_dead_topic.meta"));
}

#[test]
fn test_find_topics_sorted_by_name() {
    shm::write_topic_meta("parity_z_topic", 1024).unwrap();
    shm::write_topic_meta("parity_a_topic", 1024).unwrap();
    shm::write_topic_meta("parity_m_topic", 1024).unwrap();

    let topics = find_topics();
    let parity_topics: Vec<&str> = topics
        .iter()
        .filter(|t| t.name.starts_with("parity_"))
        .map(|t| t.name.as_str())
        .collect();

    // Verify sorted
    let mut sorted = parity_topics.clone();
    sorted.sort();
    assert_eq!(
        parity_topics, sorted,
        "find_topics should return sorted results for determinism"
    );

    shm::remove_topic_meta("parity_z_topic");
    shm::remove_topic_meta("parity_a_topic");
    shm::remove_topic_meta("parity_m_topic");
}

// ═══════════════════════════════════════════════════════════════════════════
// format_duration
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn test_format_duration_units() {
    assert_eq!(format_duration(std::time::Duration::from_secs(30)), "30s");
    assert_eq!(format_duration(std::time::Duration::from_secs(120)), "2m");
    assert_eq!(format_duration(std::time::Duration::from_secs(7200)), "2h");
    assert_eq!(
        format_duration(std::time::Duration::from_secs(172800)),
        "2d"
    );
}
