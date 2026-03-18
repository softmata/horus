//! Cross-Crate Integration Parity Tests
//!
//! Verifies data flows correctly across crate boundaries:
//! - horus_core writes TopicMetadata with `topic_name` field
//! - horus_sys reads as TopicRef with `topic` field (via serde alias)
//! - horus_manager maps NodeInfo → NodeStatus correctly
//!
//! These tests prevent regression of the P7.1 bug (TopicRef field mismatch
//! causing silent data loss in publisher/subscriber discovery).
//!
//! Run: `cargo test --no-default-features -p horus_manager --test parity_integration -- --test-threads=1`

// ═══════════════════════════════════════════════════════════════════════════
// Serde Contract: TopicMetadata → TopicRef
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn test_topic_metadata_field_alias_topic_name_to_topic() {
    // horus_core writes this JSON format (TopicMetadata uses "topic_name")
    let core_format = r#"{"topic_name": "cmd_vel", "type_name": "CmdVel"}"#;

    // horus_sys reads it as TopicRef (field is "topic" with alias "topic_name")
    let tr: horus_sys::discover::TopicRef = serde_json::from_str(core_format).unwrap();

    assert_eq!(
        tr.topic, "cmd_vel",
        "CRITICAL: topic_name alias must map to topic field — \
         without this, all publisher/subscriber lists are empty"
    );
    assert_eq!(tr.type_name, "CmdVel");
}

#[test]
fn test_topic_ref_also_accepts_topic_field_directly() {
    // Some code might write "topic" directly (e.g., test fixtures)
    let direct_format = r#"{"topic": "lidar_scan", "type_name": "LaserScan"}"#;

    let tr: horus_sys::discover::TopicRef = serde_json::from_str(direct_format).unwrap();
    assert_eq!(tr.topic, "lidar_scan");
    assert_eq!(tr.type_name, "LaserScan");
}

#[test]
fn test_topic_ref_defaults_on_missing_fields() {
    // Minimal JSON — all fields should default to empty string
    let minimal = r#"{}"#;
    let tr: horus_sys::discover::TopicRef = serde_json::from_str(minimal).unwrap();
    assert_eq!(tr.topic, "", "missing topic should default to empty");
    assert_eq!(
        tr.type_name, "",
        "missing type_name should default to empty"
    );
}

// ═══════════════════════════════════════════════════════════════════════════
// Presence File Format: horus_core → horus_sys
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn test_full_presence_json_deserialization() {
    // Simulate EXACTLY what horus_core::NodePresence::write() produces
    let core_presence = serde_json::json!({
        "name": "motor_controller",
        "pid": std::process::id(),
        "scheduler": "main_scheduler",
        "publishers": [
            {"topic_name": "motor_cmd", "type_name": "MotorCommand"},
            {"topic_name": "motor_status", "type_name": "MotorStatus"}
        ],
        "subscribers": [
            {"topic_name": "cmd_vel", "type_name": "CmdVel"},
            {"topic_name": "emergency_stop", "type_name": "Bool"}
        ],
        "start_time": 1710000000,
        "priority": 50,
        "rate_hz": 1000.0,
        "pid_start_time": 123456789,
        "health_status": "Healthy",
        "tick_count": 50000,
        "error_count": 2,
        "services": ["get_state", "calibrate"],
        "actions": ["home", "move_to"]
    });

    // Write to nodes dir
    let nodes_dir = horus_sys::shm::shm_nodes_dir();
    std::fs::create_dir_all(&nodes_dir).unwrap();
    let path = nodes_dir.join("motor_controller.json");
    std::fs::write(&path, serde_json::to_string(&core_presence).unwrap()).unwrap();

    // Discover via horus_sys
    let nodes = horus_sys::discover::find_nodes();
    let node = nodes.iter().find(|n| n.name == "motor_controller");

    assert!(node.is_some(), "presence file must be discoverable");
    let node = node.unwrap();

    // Verify all fields survived the cross-crate boundary
    assert_eq!(node.pid, std::process::id());
    assert_eq!(node.scheduler, Some("main_scheduler".to_string()));
    assert_eq!(node.priority, 50);
    assert_eq!(node.tick_count, 50000);
    assert_eq!(node.error_count, 2);
    assert_eq!(node.services, vec!["get_state", "calibrate"]);
    assert_eq!(node.actions, vec!["home", "move_to"]);

    // CRITICAL: Verify publishers use topic_name alias correctly
    assert_eq!(node.publishers.len(), 2);
    assert_eq!(node.publishers[0].topic, "motor_cmd");
    assert_eq!(node.publishers[0].type_name, "MotorCommand");
    assert_eq!(node.publishers[1].topic, "motor_status");

    // CRITICAL: Verify subscribers use topic_name alias correctly
    assert_eq!(node.subscribers.len(), 2);
    assert_eq!(node.subscribers[0].topic, "cmd_vel");
    assert_eq!(node.subscribers[1].topic, "emergency_stop");

    // Process enrichment should work
    assert!(
        !node.cmdline.is_empty(),
        "cmdline should be enriched from process_info"
    );

    let _ = std::fs::remove_file(&path);
}

#[test]
fn test_presence_with_missing_optional_fields() {
    // Minimal presence — only required fields
    let minimal = serde_json::json!({
        "name": "minimal_node",
        "pid": std::process::id()
    });

    let nodes_dir = horus_sys::shm::shm_nodes_dir();
    std::fs::create_dir_all(&nodes_dir).unwrap();
    let path = nodes_dir.join("minimal_node.json");
    std::fs::write(&path, serde_json::to_string(&minimal).unwrap()).unwrap();

    let nodes = horus_sys::discover::find_nodes();
    let node = nodes.iter().find(|n| n.name == "minimal_node");

    assert!(node.is_some(), "minimal presence should be discoverable");
    let node = node.unwrap();
    assert!(node.publishers.is_empty());
    assert!(node.subscribers.is_empty());
    assert_eq!(node.priority, 0);
    assert_eq!(node.tick_count, 0);

    let _ = std::fs::remove_file(&path);
}

// ═══════════════════════════════════════════════════════════════════════════
// TopicMeta → DiscoveredTopic → SharedMemoryInfo Pipeline
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn test_topic_meta_to_discovered_topic() {
    // Write .meta file (what ShmRegion::new writes on macOS/Windows)
    horus_sys::shm::write_topic_meta("parity_pipeline_topic", 16384).unwrap();

    // Discover via horus_sys
    let topics = horus_sys::discover::find_topics();
    let topic = topics.iter().find(|t| t.name == "parity_pipeline_topic");

    assert!(topic.is_some(), ".meta file should produce DiscoveredTopic");
    let topic = topic.unwrap();
    assert_eq!(topic.size, 16384);
    assert!(topic.is_alive, "current process is alive");
    assert_eq!(topic.creator_pid, std::process::id());

    horus_sys::shm::remove_topic_meta("parity_pipeline_topic");
}

#[test]
fn test_topic_discovery_merges_presence_pubsub() {
    // Write a .meta file for a topic
    horus_sys::shm::write_topic_meta("parity_merge_topic", 4096).unwrap();

    // Write a presence file with a node that publishes to this topic
    let nodes_dir = horus_sys::shm::shm_nodes_dir();
    std::fs::create_dir_all(&nodes_dir).unwrap();
    let presence = serde_json::json!({
        "name": "merge_test_node",
        "pid": std::process::id(),
        "publishers": [{"topic_name": "parity_merge_topic", "type_name": "MergeMsg"}],
        "subscribers": []
    });
    let node_path = nodes_dir.join("merge_test_node.json");
    std::fs::write(&node_path, serde_json::to_string(&presence).unwrap()).unwrap();

    // find_topics should merge presence pub/sub into the DiscoveredTopic
    let topics = horus_sys::discover::find_topics();
    let topic = topics.iter().find(|t| t.name == "parity_merge_topic");

    assert!(topic.is_some());
    let topic = topic.unwrap();
    assert!(
        topic.publishers.contains(&"merge_test_node".to_string()),
        "publisher from presence should be merged into topic"
    );
    assert_eq!(
        topic.message_type,
        Some("MergeMsg".to_string()),
        "message type should come from presence publisher"
    );

    horus_sys::shm::remove_topic_meta("parity_merge_topic");
    let _ = std::fs::remove_file(&node_path);
}

// ═══════════════════════════════════════════════════════════════════════════
// TopicMeta Serde Forward Compatibility
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn test_topic_meta_ignores_unknown_fields() {
    // Future TopicMeta might have extra fields — old code should still parse
    let future_json = r#"{
        "name": "future_topic",
        "size": 2048,
        "creator_pid": 12345,
        "created_at": 1710000000,
        "new_field_v2": "some value",
        "another_new_field": 42
    }"#;

    let meta: horus_sys::shm::TopicMeta = serde_json::from_str(future_json).unwrap();
    assert_eq!(meta.name, "future_topic");
    assert_eq!(meta.size, 2048);
}

#[test]
fn test_topic_meta_defaults_on_missing_fields() {
    // Old TopicMeta with fewer fields — new code should handle gracefully
    let old_json = r#"{"name": "old_topic"}"#;

    let meta: horus_sys::shm::TopicMeta = serde_json::from_str(old_json).unwrap();
    assert_eq!(meta.name, "old_topic");
    assert_eq!(meta.size, 0); // serde(default)
    assert_eq!(meta.creator_pid, 0); // serde(default)
    assert_eq!(meta.created_at, 0); // serde(default)
}
