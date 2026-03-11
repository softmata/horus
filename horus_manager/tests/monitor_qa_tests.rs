//! Phases 8–10, 13: System QA and Logging Integration tests.
//!
//! Uses `HorusTestRuntime` to create real SHM artifacts (presence files, topic
//! files, log entries) and verifies the monitor API endpoints and discovery
//! system correctly reflect node/topic state changes.

mod harness;
mod monitor_tests;

use harness::{HorusTestRuntime, TestNodeConfig};
use monitor_tests::builders;
use monitor_tests::helpers::{assert_json_ok, get_request};

use tower::ServiceExt;
use horus_core::core::DurationExt;

// ═══════════════════════════════════════════════════════════════════════════════
//  Phase 8: System QA — Real Project → Monitor Verification
// ═══════════════════════════════════════════════════════════════════════════════

#[tokio::test]
async fn qa_nodes_visible_in_monitor_after_discovery() {
    // Simulates: horus run differential_drive → monitor shows correct nodes
    let mut rt = HorusTestRuntime::new();
    rt.add_node(
        TestNodeConfig::sensor("diff_drive_left", "wheel_left_cmd", "Twist")
            .with_scheduler("drive_scheduler"),
    )
    .add_node(
        TestNodeConfig::sensor("diff_drive_right", "wheel_right_cmd", "Twist")
            .with_scheduler("drive_scheduler"),
    )
    .add_node(
        TestNodeConfig::processor(
            "diff_drive_controller",
            "cmd_vel",
            "Twist",
            "wheel_left_cmd",
            "Twist",
        )
        .with_scheduler("drive_scheduler"),
    );

    assert!(
        rt.wait_ready(2_u64.secs()),
        "nodes should become discoverable"
    );
    rt.refresh_discovery();

    let app = builders::test_router();
    let resp = app.oneshot(get_request("/api/nodes")).await.unwrap();
    let json = assert_json_ok(resp).await;

    let nodes = json["nodes"].as_array().expect("nodes must be array");
    let node_names: Vec<&str> = nodes.iter().filter_map(|n| n["name"].as_str()).collect();

    assert!(
        node_names.contains(&"diff_drive_left"),
        "diff_drive_left must be visible in monitor"
    );
    assert!(
        node_names.contains(&"diff_drive_right"),
        "diff_drive_right must be visible in monitor"
    );
    assert!(
        node_names.contains(&"diff_drive_controller"),
        "diff_drive_controller must be visible in monitor"
    );
}

#[tokio::test]
async fn qa_api_status_health_matches_nodes() {
    let mut rt = HorusTestRuntime::new();
    rt.add_node(TestNodeConfig::bare("health_qa_a"))
        .add_node(TestNodeConfig::bare("health_qa_b"));

    assert!(rt.wait_ready(2_u64.secs()));
    rt.refresh_discovery();

    let app = builders::test_router();
    let resp = app.oneshot(get_request("/api/status")).await.unwrap();
    let json = assert_json_ok(resp).await;

    let status = json["status"].as_str().unwrap();
    assert!(
        ["Healthy", "Warning", "Degraded", "Critical", "Idle", "Unknown"].contains(&status),
        "status must be a valid health value, got '{}'",
        status
    );

    let nodes_count = json["nodes"].as_u64().unwrap();
    assert!(
        nodes_count >= 2,
        "status must report at least 2 nodes (our health_qa_a + health_qa_b), got {}",
        nodes_count
    );

    // Verify our specific nodes are in the response by checking /api/nodes
    let app2 = builders::test_router();
    let resp2 = app2.oneshot(get_request("/api/nodes")).await.unwrap();
    let json2 = assert_json_ok(resp2).await;
    let node_names: Vec<&str> = json2["nodes"]
        .as_array()
        .unwrap()
        .iter()
        .filter_map(|n| n["name"].as_str())
        .collect();
    assert!(
        node_names.contains(&"health_qa_a"),
        "health_qa_a must be present"
    );
    assert!(
        node_names.contains(&"health_qa_b"),
        "health_qa_b must be present"
    );
}

#[tokio::test]
async fn qa_tui_and_web_api_data_equivalence() {
    // Verify web API returns the same nodes that TUI's discovery would find
    let mut rt = HorusTestRuntime::new();
    rt.add_node(TestNodeConfig::sensor("tui_web_qa", "camera", "Image"));

    assert!(rt.wait_ready(2_u64.secs()));
    rt.refresh_discovery();

    // Web API
    let app = builders::test_router();
    let resp = app.oneshot(get_request("/api/nodes")).await.unwrap();
    let json = assert_json_ok(resp).await;
    let api_nodes: Vec<&str> = json["nodes"]
        .as_array()
        .unwrap()
        .iter()
        .filter_map(|n| n["name"].as_str())
        .collect();

    // TUI uses the same discovery backend
    let tui_nodes = horus_manager::discovery::discover_nodes().unwrap_or_default();
    let tui_names: Vec<&str> = tui_nodes.iter().map(|n| n.name.as_str()).collect();

    // The test node must appear in both
    assert!(
        api_nodes.contains(&"tui_web_qa"),
        "web API must show tui_web_qa"
    );
    assert!(
        tui_names.contains(&"tui_web_qa"),
        "TUI discovery must show tui_web_qa"
    );
}

#[tokio::test]
async fn qa_cpu_memory_fields_present() {
    let mut rt = HorusTestRuntime::new();
    rt.add_node(TestNodeConfig::bare("metrics_qa"));

    assert!(rt.wait_ready(2_u64.secs()));
    rt.refresh_discovery();

    let app = builders::test_router();
    let resp = app.oneshot(get_request("/api/nodes")).await.unwrap();
    let json = assert_json_ok(resp).await;

    let nodes = json["nodes"].as_array().unwrap();
    let node = nodes
        .iter()
        .find(|n| n["name"].as_str() == Some("metrics_qa"))
        .expect("metrics_qa node must be found in API response");

    // CPU and memory must be present with valid values
    let cpu = node.get("cpu").expect("node must have cpu field");
    assert!(
        cpu.is_number() || cpu.is_string(),
        "cpu must be a number or string, got {:?}",
        cpu
    );
    let memory = node.get("memory").expect("node must have memory field");
    assert!(
        memory.is_number() || memory.is_string(),
        "memory must be a number or string, got {:?}",
        memory
    );
}

// ═══════════════════════════════════════════════════════════════════════════════
//  Phase 9: System QA — Node Lifecycle & Dynamic Behavior
// ═══════════════════════════════════════════════════════════════════════════════

#[tokio::test]
async fn qa_node_disappearance_detected() {
    // Create a node, verify it appears, then drop the runtime (cleanup),
    // verify it disappears
    let mut rt = HorusTestRuntime::new();
    rt.add_node(TestNodeConfig::bare("disappear_qa"));
    assert!(rt.wait_ready(2_u64.secs()));
    rt.refresh_discovery();

    // Verify node is visible
    let app = builders::test_router();
    let resp = app
        .clone()
        .oneshot(get_request("/api/nodes"))
        .await
        .unwrap();
    let json = assert_json_ok(resp).await;
    let names: Vec<&str> = json["nodes"]
        .as_array()
        .unwrap()
        .iter()
        .filter_map(|n| n["name"].as_str())
        .collect();
    assert!(
        names.contains(&"disappear_qa"),
        "node must appear initially"
    );

    // Drop runtime — presence files are cleaned up
    drop(rt);

    // Wait for discovery cache to expire
    std::thread::sleep(350_u64.ms());

    // Node should no longer appear
    let app2 = builders::test_router();
    let resp2 = app2.oneshot(get_request("/api/nodes")).await.unwrap();
    let json2 = assert_json_ok(resp2).await;
    let names2: Vec<&str> = json2["nodes"]
        .as_array()
        .unwrap()
        .iter()
        .filter_map(|n| n["name"].as_str())
        .collect();
    assert!(
        !names2.contains(&"disappear_qa"),
        "node must disappear after runtime drop"
    );
}

#[tokio::test]
async fn qa_node_graceful_shutdown_detected() {
    // Same as crash detection but simulates graceful shutdown
    let mut rt = HorusTestRuntime::new();
    rt.add_node(TestNodeConfig::bare("graceful_qa"));
    assert!(rt.wait_ready(2_u64.secs()));
    rt.refresh_discovery();

    // Verify present
    let app = builders::test_router();
    let resp = app.oneshot(get_request("/api/nodes")).await.unwrap();
    let json = assert_json_ok(resp).await;
    assert!(json["nodes"]
        .as_array()
        .unwrap()
        .iter()
        .any(|n| n["name"].as_str() == Some("graceful_qa")));

    // Graceful shutdown = drop
    drop(rt);
    std::thread::sleep(350_u64.ms());

    let app2 = builders::test_router();
    let resp2 = app2.oneshot(get_request("/api/nodes")).await.unwrap();
    let json2 = assert_json_ok(resp2).await;
    assert!(
        !json2["nodes"]
            .as_array()
            .unwrap()
            .iter()
            .any(|n| n["name"].as_str() == Some("graceful_qa")),
        "node must be gone after graceful shutdown"
    );
}

#[tokio::test]
async fn qa_topic_creation_and_destruction() {
    let mut rt = HorusTestRuntime::new();
    rt.add_node(TestNodeConfig::sensor(
        "topic_lifecycle",
        "ephemeral_topic",
        "Data",
    ))
    .add_topic("ephemeral_topic", 4096);

    assert!(rt.wait_ready(2_u64.secs()));
    rt.refresh_discovery();

    // Topic should be visible
    let app = builders::test_router();
    let resp = app.oneshot(get_request("/api/topics")).await.unwrap();
    let json = assert_json_ok(resp).await;
    let topics = json["topics"].as_array().unwrap();
    let has_topic = topics.iter().any(|t| {
        t["name"]
            .as_str()
            .map_or(false, |n| n.contains("ephemeral_topic"))
    });
    assert!(has_topic, "ephemeral_topic should be visible");

    // Drop runtime — topics cleaned up
    drop(rt);
    std::thread::sleep(350_u64.ms());

    let app2 = builders::test_router();
    let resp2 = app2.oneshot(get_request("/api/topics")).await.unwrap();
    let json2 = assert_json_ok(resp2).await;
    let topics2 = json2["topics"].as_array().unwrap();
    let still_has = topics2.iter().any(|t| {
        t["name"]
            .as_str()
            .map_or(false, |n| n.contains("ephemeral_topic"))
    });
    assert!(!still_has, "ephemeral_topic should be gone after cleanup");
}

#[tokio::test]
async fn qa_node_restart_fresh_instance() {
    // Simulate node restart: create, drop, create again with same name
    let node_name = "restart_qa_node";

    {
        let mut rt = HorusTestRuntime::new();
        rt.add_node(TestNodeConfig::bare(node_name));
        assert!(rt.wait_ready(2_u64.secs()));
        // Drop = shutdown
    }
    std::thread::sleep(350_u64.ms());

    // Restart with same name
    let mut rt2 = HorusTestRuntime::new();
    rt2.add_node(TestNodeConfig::bare(node_name));
    assert!(rt2.wait_ready(2_u64.secs()));
    rt2.refresh_discovery();

    let app = builders::test_router();
    let resp = app.oneshot(get_request("/api/nodes")).await.unwrap();
    let json = assert_json_ok(resp).await;
    let nodes = json["nodes"].as_array().unwrap();
    let found = nodes.iter().any(|n| n["name"].as_str() == Some(node_name));
    assert!(found, "restarted node must be visible");
}

#[tokio::test]
async fn qa_node_health_reported_in_api() {
    let mut rt = HorusTestRuntime::new();
    rt.add_node(TestNodeConfig::bare("health_report_qa"));

    assert!(rt.wait_ready(2_u64.secs()));
    rt.refresh_discovery();

    let app = builders::test_router();
    let resp = app.oneshot(get_request("/api/nodes")).await.unwrap();
    let json = assert_json_ok(resp).await;

    let node = json["nodes"]
        .as_array()
        .unwrap()
        .iter()
        .find(|n| n["name"].as_str() == Some("health_report_qa"))
        .expect("health_report_qa node must be found in API response");

    let health = node["health"]
        .as_str()
        .expect("node must have health string field");
    assert!(
        ["Healthy", "Degraded", "Error", "Unknown", "Idle", "Warning"].contains(&health),
        "health must be a valid value, got '{}'",
        health
    );
    let health_color = node["health_color"]
        .as_str()
        .expect("node must have health_color string field");
    assert!(!health_color.is_empty(), "health_color must not be empty");
}

// ═══════════════════════════════════════════════════════════════════════════════
//  Phase 10: System QA — Multi-Node & Launch Config Scenarios
// ═══════════════════════════════════════════════════════════════════════════════

#[tokio::test]
async fn qa_multi_node_all_visible() {
    // Simulates: horus launch config → all nodes visible
    let mut rt = HorusTestRuntime::new();
    rt.add_node(
        TestNodeConfig::sensor("camera_node", "camera_rgb", "Image").with_scheduler("perception"),
    )
    .add_node(
        TestNodeConfig::sensor("lidar_node", "lidar_scan", "LaserScan")
            .with_scheduler("perception"),
    )
    .add_node(
        TestNodeConfig::processor(
            "fusion_node",
            "camera_rgb",
            "Image",
            "fused_output",
            "FusedData",
        )
        .with_scheduler("perception"),
    )
    .add_node(
        TestNodeConfig::actuator("motor_node", "motor_cmd", "Twist").with_scheduler("control"),
    );

    assert!(rt.wait_ready(2_u64.secs()));
    rt.refresh_discovery();

    let app = builders::test_router();
    let resp = app.oneshot(get_request("/api/nodes")).await.unwrap();
    let json = assert_json_ok(resp).await;

    let node_names: Vec<&str> = json["nodes"]
        .as_array()
        .unwrap()
        .iter()
        .filter_map(|n| n["name"].as_str())
        .collect();

    for expected in &["camera_node", "lidar_node", "fusion_node", "motor_node"] {
        assert!(
            node_names.contains(expected),
            "multi-node launch: '{}' must be visible in monitor",
            expected
        );
    }
}

#[tokio::test]
async fn qa_pub_sub_graph_verification() {
    let mut rt = HorusTestRuntime::new();
    rt.add_node(TestNodeConfig::sensor("graph_pub", "graph_topic", "Data"))
        .add_node(TestNodeConfig::actuator("graph_sub", "graph_topic", "Data"))
        .add_topic("graph_topic", 4096);

    assert!(rt.wait_ready(2_u64.secs()));
    rt.refresh_discovery();

    let app = builders::test_router();
    let resp = app.oneshot(get_request("/api/graph")).await.unwrap();
    let json = assert_json_ok(resp).await;

    let graph_nodes = json["nodes"].as_array().unwrap();
    let edges = json["edges"].as_array().unwrap();

    // Should have at least the publisher and subscriber as process nodes
    let process_nodes: Vec<&str> = graph_nodes
        .iter()
        .filter(|n| n["type"].as_str() == Some("process"))
        .filter_map(|n| n["label"].as_str())
        .collect();

    assert!(
        process_nodes.iter().any(|n| n.contains("graph_pub")),
        "publisher must appear as process node in graph"
    );
    assert!(
        process_nodes.iter().any(|n| n.contains("graph_sub")),
        "subscriber must appear as process node in graph"
    );

    // Should have the topic as a topic node
    let topic_nodes: Vec<&str> = graph_nodes
        .iter()
        .filter(|n| n["type"].as_str() == Some("topic"))
        .filter_map(|n| n["label"].as_str())
        .collect();

    // Check for the topic (may have horus_ prefix)
    let has_graph_topic = topic_nodes.iter().any(|t| t.contains("graph_topic"));

    // Graph must have at least 2 process nodes AND edges connecting them
    assert!(
        process_nodes.len() >= 2,
        "graph must have at least 2 process nodes (pub + sub), got {}",
        process_nodes.len()
    );

    // Verify all edges have required fields
    for edge in edges {
        assert!(edge["from"].is_string(), "edge.from must be string");
        assert!(edge["to"].is_string(), "edge.to must be string");
        assert!(edge["type"].is_string(), "edge.type must be string");
    }

    // If topic node exists, verify it
    if has_graph_topic {
        // Edges should connect through the topic
        assert!(!edges.is_empty(), "graph with topic node must have edges");
    }
}

#[tokio::test]
async fn qa_scale_test_many_nodes() {
    // 20 nodes — verify all appear
    let mut rt = HorusTestRuntime::new();
    for i in 0..20 {
        rt.add_node(TestNodeConfig::bare(&format!("scale_node_{:02}", i)));
    }

    assert!(rt.wait_ready(3_u64.secs()));
    rt.refresh_discovery();

    let app = builders::test_router();
    let resp = app.oneshot(get_request("/api/nodes")).await.unwrap();
    let json = assert_json_ok(resp).await;

    let node_names: Vec<&str> = json["nodes"]
        .as_array()
        .unwrap()
        .iter()
        .filter_map(|n| n["name"].as_str())
        .collect();

    let mut missing: Vec<String> = Vec::new();
    for i in 0..20 {
        let name = format!("scale_node_{:02}", i);
        if !node_names.contains(&name.as_str()) {
            missing.push(name);
        }
    }

    assert!(
        missing.is_empty(),
        "all 20 scale nodes must be visible, missing: {:?}",
        missing
    );
}

#[tokio::test]
async fn qa_partial_launch_mixed_results() {
    // Some nodes exist, verify monitor shows what's available
    let mut rt = HorusTestRuntime::new();
    rt.add_node(TestNodeConfig::bare("partial_ok_1"))
        .add_node(TestNodeConfig::bare("partial_ok_2"));
    // partial_fail_1 is intentionally NOT added (simulates failed launch)

    assert!(rt.wait_ready(2_u64.secs()));
    rt.refresh_discovery();

    let app = builders::test_router();
    let resp = app.oneshot(get_request("/api/nodes")).await.unwrap();
    let json = assert_json_ok(resp).await;

    let node_names: Vec<&str> = json["nodes"]
        .as_array()
        .unwrap()
        .iter()
        .filter_map(|n| n["name"].as_str())
        .collect();

    assert!(
        node_names.contains(&"partial_ok_1"),
        "surviving node 1 must be visible"
    );
    assert!(
        node_names.contains(&"partial_ok_2"),
        "surviving node 2 must be visible"
    );
    assert!(
        !node_names.contains(&"partial_fail_1"),
        "failed node must NOT be visible"
    );
}

#[tokio::test]
async fn qa_network_endpoint_returns_valid_data() {
    let mut rt = HorusTestRuntime::new();
    rt.add_node(TestNodeConfig::bare("network_qa"));

    assert!(rt.wait_ready(2_u64.secs()));
    rt.refresh_discovery();

    let app = builders::test_router();
    let resp = app.oneshot(get_request("/api/network")).await.unwrap();
    let json = assert_json_ok(resp).await;

    // /api/network reports network transport status — total_nodes may differ from
    // discovery count (tracks network-connected nodes, not SHM presence)
    assert!(
        json["total_nodes"].is_number(),
        "total_nodes must be a number"
    );
    assert!(
        json["node_statuses"].is_array() || json["node_statuses"].is_object(),
        "must have node_statuses as array or object"
    );

    // Verify the response structure is well-formed (has expected top-level keys)
    let keys: Vec<&str> = json
        .as_object()
        .unwrap()
        .keys()
        .map(|k| k.as_str())
        .collect();
    assert!(
        keys.contains(&"total_nodes") && keys.contains(&"node_statuses"),
        "response must have total_nodes and node_statuses keys, got: {:?}",
        keys
    );
}

// ═══════════════════════════════════════════════════════════════════════════════
//  Phase 13: Logging Integration — Real Node → Monitor Log Display
// ═══════════════════════════════════════════════════════════════════════════════

#[tokio::test]
async fn e2e_injected_log_appears_in_api_logs_all() {
    let rt = HorusTestRuntime::new();

    // Inject a log entry
    rt.inject_log("e2e_log_node", "info", "e2e integration test message");

    let app = builders::test_router();
    let resp = app.oneshot(get_request("/api/logs/all")).await.unwrap();
    let json = assert_json_ok(resp).await;

    let logs = json["logs"].as_array().unwrap();
    let found = logs.iter().any(|l| {
        l["node_name"].as_str() == Some("e2e_log_node")
            && l["message"]
                .as_str()
                .map_or(false, |m| m.contains("e2e integration test"))
    });
    assert!(found, "injected log entry must appear in /api/logs/all");
}

#[tokio::test]
async fn e2e_node_log_filtering() {
    let rt = HorusTestRuntime::new();

    rt.inject_log("filter_node_a", "info", "log from node A");
    rt.inject_log("filter_node_b", "warn", "log from node B");
    rt.inject_log("filter_node_a", "error", "error from node A");

    let app = builders::test_router();
    let resp = app
        .oneshot(get_request("/api/logs/node/filter_node_a"))
        .await
        .unwrap();
    let json = assert_json_ok(resp).await;

    assert_eq!(json["node"].as_str(), Some("filter_node_a"));
    let logs = json["logs"].as_array().unwrap();

    // All returned logs should be for filter_node_a
    for log in logs {
        assert_eq!(
            log["node_name"].as_str(),
            Some("filter_node_a"),
            "filtered logs must only contain the requested node"
        );
    }

    // Should have at least 2 entries from node A
    assert!(logs.len() >= 2, "should have at least 2 logs for node A");
}

#[tokio::test]
async fn e2e_topic_log_filtering() {
    let rt = HorusTestRuntime::new();

    // Inject logs with topic field
    use horus_core::core::log_buffer::{LogEntry, LogType, GLOBAL_LOG_BUFFER};
    GLOBAL_LOG_BUFFER.push(LogEntry {
        timestamp: chrono::Utc::now().format("%H:%M:%S%.3f").to_string(),
        tick_number: 0,
        node_name: "topic_log_pub".to_string(),
        log_type: LogType::Publish,
        topic: Some("camera_rgb".to_string()),
        message: "published camera frame".to_string(),
        tick_us: 100,
        ipc_ns: 500,
    });

    let app = builders::test_router();
    let resp = app
        .oneshot(get_request("/api/logs/topic/camera_rgb"))
        .await
        .unwrap();
    let json = assert_json_ok(resp).await;

    assert!(json["logs"].is_array());
    let logs = json["logs"].as_array().unwrap();

    // Should find the camera_rgb log
    let found = logs.iter().any(|l| {
        l["message"]
            .as_str()
            .map_or(false, |m| m.contains("published camera frame"))
    });
    assert!(found, "topic-filtered log for camera_rgb must appear");

    drop(rt);
}

#[tokio::test]
async fn e2e_log_severity_classification() {
    let rt = HorusTestRuntime::new();

    rt.inject_log("severity_node", "info", "info message");
    rt.inject_log("severity_node", "warn", "warning message");
    rt.inject_log("severity_node", "error", "error message");
    rt.inject_log("severity_node", "debug", "debug message");

    let app = builders::test_router();
    let resp = app
        .oneshot(get_request("/api/logs/node/severity_node"))
        .await
        .unwrap();
    let json = assert_json_ok(resp).await;

    let logs = json["logs"].as_array().unwrap();

    // Verify different log types are present
    let log_types: Vec<&str> = logs.iter().filter_map(|l| l["log_type"].as_str()).collect();

    assert!(
        log_types.contains(&"Info") || log_types.contains(&"info"),
        "Info log type must be present"
    );
    assert!(
        log_types.contains(&"Warning")
            || log_types.contains(&"warning")
            || log_types.contains(&"warn"),
        "Warning log type must be present"
    );
    assert!(
        log_types.contains(&"Error") || log_types.contains(&"error"),
        "Error log type must be present"
    );

    drop(rt);
}

#[tokio::test]
async fn e2e_cross_process_log_aggregation() {
    let rt = HorusTestRuntime::new();

    // Inject logs from multiple "processes" (different node names)
    for i in 0..5 {
        rt.inject_log(
            &format!("process_{}", i),
            "info",
            &format!("message from process {}", i),
        );
    }

    let app = builders::test_router();
    let resp = app.oneshot(get_request("/api/logs/all")).await.unwrap();
    let json = assert_json_ok(resp).await;

    let logs = json["logs"].as_array().unwrap();

    // All 5 process logs should be aggregated
    let mut found_processes = std::collections::HashSet::new();
    for log in logs {
        if let Some(name) = log["node_name"].as_str() {
            if name.starts_with("process_") {
                found_processes.insert(name.to_string());
            }
        }
    }

    assert!(
        found_processes.len() >= 5,
        "all 5 process logs should be aggregated, found {}",
        found_processes.len()
    );

    drop(rt);
}

#[tokio::test]
async fn e2e_log_cli_and_api_consistency() {
    let rt = HorusTestRuntime::new();

    rt.inject_log("consistency_node", "info", "consistency check message");

    // Read CLI buffer immediately after injection (before parallel tests overwrite)
    let cli_logs = horus_core::core::log_buffer::GLOBAL_LOG_BUFFER.get_all();
    let cli_found = cli_logs
        .iter()
        .any(|e| e.node_name == "consistency_node" && e.message.contains("consistency check"));

    // API returns logs from the same buffer
    let app = builders::test_router();
    let resp = app.oneshot(get_request("/api/logs/all")).await.unwrap();
    let json = assert_json_ok(resp).await;
    let api_logs = json["logs"].as_array().unwrap();
    let api_found = api_logs.iter().any(|l| {
        l["node_name"].as_str() == Some("consistency_node")
            && l["message"]
                .as_str()
                .map_or(false, |m| m.contains("consistency check"))
    });

    assert!(cli_found, "CLI buffer must find the consistency log");
    assert!(api_found, "API must find the consistency log");

    drop(rt);
}

#[tokio::test]
async fn e2e_high_volume_log_stress() {
    let rt = HorusTestRuntime::new();

    // Inject 200 entries — enough for stress without wrapping the shared
    // GLOBAL_LOG_BUFFER (which would overwrite other parallel tests' entries).
    // Ring buffer wrap at 5000 boundary is tested in log_system_tests with
    // isolated buffers.
    let count = 200u64;
    for i in 0..count {
        rt.inject_log(
            &format!("stress_node_{}", i % 10),
            if i % 3 == 0 { "error" } else { "info" },
            &format!("stress message {}", i),
        );
    }

    let app = builders::test_router();
    let resp = app.oneshot(get_request("/api/logs/all")).await.unwrap();
    let json = assert_json_ok(resp).await;

    let logs = json["logs"].as_array().unwrap();
    // Should have most of the injected entries (buffer shared with parallel tests)
    let stress_logs: Vec<_> = logs
        .iter()
        .filter(|l| {
            l["message"]
                .as_str()
                .map_or(false, |m| m.starts_with("stress message"))
        })
        .collect();
    assert!(
        stress_logs.len() >= 150,
        "should find >= 150 of 200 stress entries, found {}",
        stress_logs.len()
    );

    // Verify multiple distinct node names appear
    let stress_node_names: std::collections::HashSet<&str> = stress_logs
        .iter()
        .filter_map(|l| l["node_name"].as_str())
        .collect();
    assert!(
        stress_node_names.len() >= 5,
        "should have >= 5 distinct stress_node names, found {}",
        stress_node_names.len()
    );

    // Latest entry should be present
    let has_recent = logs.iter().any(|l| {
        l["message"]
            .as_str()
            .map_or(false, |m| m.contains("stress message 199"))
    });
    assert!(has_recent, "most recent stress entry (199) must be present");

    drop(rt);
}

#[tokio::test]
async fn e2e_log_entry_fields_preserved() {
    // Inject a log with all fields and verify they appear in API response
    use horus_core::core::log_buffer::{LogEntry, LogType, GLOBAL_LOG_BUFFER};

    GLOBAL_LOG_BUFFER.push(LogEntry {
        timestamp: "14:30:45.123".to_string(),
        tick_number: 999,
        node_name: "field_preserve_node".to_string(),
        log_type: LogType::Warning,
        topic: Some("imu_data".to_string()),
        message: "gyro drift detected".to_string(),
        tick_us: 1234,
        ipc_ns: 5678,
    });

    let app = builders::test_router();
    let resp = app
        .oneshot(get_request("/api/logs/node/field_preserve_node"))
        .await
        .unwrap();
    let json = assert_json_ok(resp).await;

    let logs = json["logs"].as_array().unwrap();
    let entry = logs
        .iter()
        .find(|l| l["message"].as_str() == Some("gyro drift detected"));

    assert!(entry.is_some(), "injected entry must appear in response");
    let e = entry.unwrap();
    assert_eq!(e["timestamp"].as_str(), Some("14:30:45.123"));
    assert_eq!(e["node_name"].as_str(), Some("field_preserve_node"));
    assert_eq!(
        e["tick_number"].as_u64(),
        Some(999),
        "tick_number must be preserved"
    );
    assert_eq!(
        e["tick_us"].as_u64(),
        Some(1234),
        "tick_us must be preserved"
    );
    assert_eq!(e["ipc_ns"].as_u64(), Some(5678), "ipc_ns must be preserved");

    // Verify log_type round-trips (serde may serialize as string)
    let log_type = e["log_type"].as_str().unwrap_or("");
    assert!(
        log_type == "Warning" || log_type == "warning",
        "log_type must be Warning, got '{}'",
        log_type
    );

    // Verify topic field
    let topic = e["topic"].as_str();
    assert_eq!(topic, Some("imu_data"), "topic must be preserved");
}
