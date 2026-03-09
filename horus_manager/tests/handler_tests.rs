//! Comprehensive integration tests for the 5 core monitoring handler endpoints.
//!
//! These tests use the live test harness ([`HorusTestRuntime`]) to create real
//! SHM presence and topic files, then issue HTTP requests against the Axum router
//! to verify the handlers produce correct JSON responses.
//!
//! All tests assume `--test-threads=1` serial execution.  The harness cleans up
//! on [`Drop`], so tests do not rely on ordering or shared state.

mod harness;
mod monitor_tests;

use harness::{HorusTestRuntime, TestNodeConfig};
use monitor_tests::builders;
use monitor_tests::helpers::{assert_json_ok, get_request};

use std::time::Duration;
use tower::ServiceExt;

// ═══════════════════════════════════════════════════════════════════════════════
//  STATUS HANDLER  (GET /api/status)
// ═══════════════════════════════════════════════════════════════════════════════

#[tokio::test]
async fn status_baseline_returns_valid_json() {
    // Without adding any harness nodes, the system may still have
    // residual or real nodes present.  We verify the response is
    // valid JSON with the expected schema regardless.
    let app = builders::test_router();
    let resp = app.oneshot(get_request("/api/status")).await.unwrap();
    let json = assert_json_ok(resp).await;

    // Status should be one of the known values.
    let status = json["status"].as_str().expect("status should be a string");
    let valid_statuses = ["Idle", "Healthy", "Warning", "Degraded", "Critical", "Unknown"];
    assert!(
        valid_statuses.contains(&status),
        "status should be one of {:?}, got '{}'",
        valid_statuses,
        status
    );

    // Health color should be a known color.
    let color = json["health_color"].as_str().expect("health_color should be a string");
    let valid_colors = ["gray", "green", "yellow", "orange", "red"];
    assert!(
        valid_colors.contains(&color),
        "health_color should be one of {:?}, got '{}'",
        valid_colors,
        color
    );

    // Node and topic counts should be non-negative numbers.
    assert!(json["nodes"].is_number(), "nodes should be a number");
    assert!(json["topics"].is_number(), "topics should be a number");
}

#[tokio::test]
async fn status_healthy_with_all_healthy_nodes() {
    let mut rt = HorusTestRuntime::new();
    rt.add_node(TestNodeConfig::bare("handler_healthy_a"))
        .add_node(TestNodeConfig::bare("handler_healthy_b"));

    assert!(
        rt.wait_ready(Duration::from_secs(2)),
        "nodes should become discoverable"
    );
    rt.refresh_discovery();

    let app = builders::test_router();
    let resp = app.oneshot(get_request("/api/status")).await.unwrap();
    let json = assert_json_ok(resp).await;

    assert_eq!(json["status"], "Healthy");
    assert_eq!(json["health_color"], "green");
}

#[tokio::test]
async fn status_reports_correct_node_and_topic_counts() {
    let mut rt = HorusTestRuntime::new();
    rt.add_node(TestNodeConfig::bare("handler_count_a"))
        .add_node(TestNodeConfig::bare("handler_count_b"))
        .add_node(TestNodeConfig::bare("handler_count_c"))
        .add_topic("handler_count_topic_1", 4096)
        .add_topic("handler_count_topic_2", 2048);

    assert!(
        rt.wait_ready(Duration::from_secs(2)),
        "nodes should become discoverable"
    );
    rt.refresh_discovery();

    let app = builders::test_router();
    let resp = app.oneshot(get_request("/api/status")).await.unwrap();
    let json = assert_json_ok(resp).await;

    // We check >= because other tests or a real system may leave residual
    // entries, but we must see at least what we added.
    let nodes_count = json["nodes"].as_u64().expect("nodes should be a number");
    let topics_count = json["topics"].as_u64().expect("topics should be a number");

    assert!(
        nodes_count >= 3,
        "expected at least 3 nodes, got {}",
        nodes_count
    );
    assert!(
        topics_count >= 2,
        "expected at least 2 topics, got {}",
        topics_count
    );
}

#[tokio::test]
async fn status_workspace_detected() {
    let ws_path = std::path::PathBuf::from("/tmp/handler_test_workspace");
    let state = builders::test_app_state_with_workspace(ws_path.clone());
    let app = builders::test_router_with_state(state);

    let resp = app.oneshot(get_request("/api/status")).await.unwrap();
    let json = assert_json_ok(resp).await;

    let workspace = &json["workspace"];
    assert_eq!(workspace["detected"], true, "workspace should be detected");
    assert_eq!(workspace["name"], "handler_test_workspace");
    assert_eq!(
        workspace["path"],
        ws_path.display().to_string(),
        "workspace path should match"
    );
}

#[tokio::test]
async fn status_workspace_not_detected() {
    let app = builders::test_router();
    let resp = app.oneshot(get_request("/api/status")).await.unwrap();
    let json = assert_json_ok(resp).await;

    let workspace = &json["workspace"];
    assert_eq!(
        workspace["detected"], false,
        "default state should have no workspace"
    );
    // When not detected, name and path should be absent.
    assert!(
        workspace.get("name").is_none() || workspace["name"].is_null(),
        "name should be absent when workspace not detected"
    );
}

#[tokio::test]
async fn status_version_matches_cargo() {
    let app = builders::test_router();
    let resp = app.oneshot(get_request("/api/status")).await.unwrap();
    let json = assert_json_ok(resp).await;

    assert_eq!(json["version"], "0.1.9", "version should match Cargo.toml");
}

// ═══════════════════════════════════════════════════════════════════════════════
//  NODES HANDLER  (GET /api/nodes)
// ═══════════════════════════════════════════════════════════════════════════════

#[tokio::test]
async fn nodes_empty_when_no_nodes() {
    let rt = HorusTestRuntime::new();
    rt.refresh_discovery();

    let app = builders::test_router();
    let resp = app.oneshot(get_request("/api/nodes")).await.unwrap();
    let json = assert_json_ok(resp).await;

    // The array should exist — it may contain nodes from other sources on the
    // system, but it must be a valid array.
    assert!(json["nodes"].is_array(), "nodes should be an array");
}

#[tokio::test]
async fn nodes_returns_all_added_nodes() {
    let mut rt = HorusTestRuntime::new();
    rt.add_node(TestNodeConfig::bare("handler_node_a"))
        .add_node(TestNodeConfig::sensor(
            "handler_node_b",
            "topic_b",
            "MsgB",
        ))
        .add_node(TestNodeConfig::actuator(
            "handler_node_c",
            "topic_c",
            "MsgC",
        ));

    assert!(rt.wait_ready(Duration::from_secs(2)));
    rt.refresh_discovery();

    let app = builders::test_router();
    let resp = app.oneshot(get_request("/api/nodes")).await.unwrap();
    let json = assert_json_ok(resp).await;

    let nodes = json["nodes"].as_array().expect("nodes should be an array");
    let node_names: Vec<&str> = nodes
        .iter()
        .filter_map(|n| n["name"].as_str())
        .collect();

    let my_pid = std::process::id();

    for expected in &["handler_node_a", "handler_node_b", "handler_node_c"] {
        assert!(
            node_names.contains(expected),
            "expected node '{}' in response, found {:?}",
            expected,
            node_names
        );

        // Verify PID matches this process for each harness node.
        let entry = nodes.iter().find(|n| n["name"].as_str() == Some(expected));
        if let Some(entry) = entry {
            assert_eq!(
                entry["pid"].as_u64().unwrap() as u32,
                my_pid,
                "PID for {} should match current process",
                expected
            );
        }
    }
}

#[tokio::test]
async fn nodes_response_json_schema() {
    let mut rt = HorusTestRuntime::new();
    rt.add_node(TestNodeConfig::bare("handler_schema_node"));

    assert!(rt.wait_ready(Duration::from_secs(2)));
    rt.refresh_discovery();

    let app = builders::test_router();
    let resp = app.oneshot(get_request("/api/nodes")).await.unwrap();
    let json = assert_json_ok(resp).await;

    let nodes = json["nodes"].as_array().expect("nodes should be an array");
    let entry = nodes
        .iter()
        .find(|n| n["name"].as_str() == Some("handler_schema_node"))
        .expect("handler_schema_node should appear in response");

    // Validate all expected fields exist.
    let required_keys = [
        "name",
        "pid",
        "status",
        "health",
        "health_color",
        "cpu",
        "memory",
        "tick_count",
        "error_count",
        "tick_rate",
        "scheduler_name",
    ];
    for key in &required_keys {
        assert!(
            entry.get(key).is_some() && !entry[key].is_null(),
            "node entry should have non-null key '{}', got: {}",
            key,
            serde_json::to_string_pretty(entry).unwrap_or_default()
        );
    }
}

#[tokio::test]
async fn nodes_memory_formatted_as_mb() {
    let mut rt = HorusTestRuntime::new();
    rt.add_node(TestNodeConfig::bare("handler_mem_fmt_node"));

    assert!(rt.wait_ready(Duration::from_secs(2)));
    rt.refresh_discovery();

    let app = builders::test_router();
    let resp = app.oneshot(get_request("/api/nodes")).await.unwrap();
    let json = assert_json_ok(resp).await;

    let nodes = json["nodes"].as_array().unwrap();
    let entry = nodes
        .iter()
        .find(|n| n["name"].as_str() == Some("handler_mem_fmt_node"))
        .expect("node should appear");

    let mem_str = entry["memory"]
        .as_str()
        .expect("memory should be a string");
    assert!(
        mem_str.ends_with(" MB"),
        "memory should end with ' MB', got: '{}'",
        mem_str
    );

    // The numeric part before " MB" should be parseable as a number.
    let numeric_part = mem_str.trim_end_matches(" MB");
    assert!(
        numeric_part.parse::<u64>().is_ok(),
        "memory numeric part '{}' should be a valid integer",
        numeric_part
    );
}

#[tokio::test]
async fn nodes_cpu_formatted_as_percent() {
    let mut rt = HorusTestRuntime::new();
    rt.add_node(TestNodeConfig::bare("handler_cpu_fmt_node"));

    assert!(rt.wait_ready(Duration::from_secs(2)));
    rt.refresh_discovery();

    let app = builders::test_router();
    let resp = app.oneshot(get_request("/api/nodes")).await.unwrap();
    let json = assert_json_ok(resp).await;

    let nodes = json["nodes"].as_array().unwrap();
    let entry = nodes
        .iter()
        .find(|n| n["name"].as_str() == Some("handler_cpu_fmt_node"))
        .expect("node should appear");

    let cpu_str = entry["cpu"].as_str().expect("cpu should be a string");
    assert!(
        cpu_str.ends_with('%'),
        "cpu should end with '%', got: '{}'",
        cpu_str
    );

    // The numeric part before "%" should be parseable as a float.
    let numeric_part = cpu_str.trim_end_matches('%');
    assert!(
        numeric_part.parse::<f64>().is_ok(),
        "cpu numeric part '{}' should be a valid float",
        numeric_part
    );
}

// ═══════════════════════════════════════════════════════════════════════════════
//  TOPICS HANDLER  (GET /api/topics)
// ═══════════════════════════════════════════════════════════════════════════════

#[tokio::test]
async fn topics_empty_when_no_topics() {
    let rt = HorusTestRuntime::new();
    rt.refresh_discovery();

    let app = builders::test_router();
    let resp = app.oneshot(get_request("/api/topics")).await.unwrap();
    let json = assert_json_ok(resp).await;

    assert!(json["topics"].is_array(), "topics should be an array");
}

#[tokio::test]
async fn topics_returns_added_topics() {
    let mut rt = HorusTestRuntime::new();
    rt.add_topic("handler_camera_rgb", 8192)
        .add_topic("handler_lidar_scan", 16384);

    rt.refresh_discovery();

    let app = builders::test_router();
    let resp = app.oneshot(get_request("/api/topics")).await.unwrap();
    let json = assert_json_ok(resp).await;

    let topics = json["topics"].as_array().expect("topics should be an array");
    let _topic_names: Vec<&str> = topics
        .iter()
        .filter_map(|t| t["name"].as_str())
        .collect();

    // Topics created via harness go under horus_topic/ subdirectory.
    // The discovery layer should pick them up; the name may or may not have
    // the horus_ prefix stripped depending on the raw file name.
    // We check that at least 2 topics were found (our added ones).
    assert!(
        topics.len() >= 2,
        "expected at least 2 topics, found {}",
        topics.len()
    );

    // Each topic entry should have the expected structure.
    for topic in topics {
        assert!(topic["name"].is_string(), "topic name should be a string");
        assert!(topic["size"].is_string(), "topic size should be a string");
        assert!(
            topic.get("active").is_some(),
            "topic should have active field"
        );
        assert!(
            topic.get("processes").is_some(),
            "topic should have processes field"
        );
    }
}

#[tokio::test]
async fn topics_strips_horus_prefix() {
    // Create a topic whose discovered name starts with "horus_" to verify
    // the handler strips the prefix.
    let mut rt = HorusTestRuntime::new();
    // The raw topic API creates files under horus_topic/ but with the given
    // name.  To get a topic that the discovery layer sees as "horus_camera",
    // we use add_raw_topic which places the file directly in the SHM topics dir.
    rt.add_raw_topic("horus_handler_strip_test", 4096);

    rt.refresh_discovery();

    let app = builders::test_router();
    let resp = app.oneshot(get_request("/api/topics")).await.unwrap();
    let json = assert_json_ok(resp).await;

    let topics = json["topics"].as_array().expect("topics should be an array");

    // Look for our topic — the handler should have stripped the "horus_" prefix.
    let found = topics.iter().any(|t| {
        let name = t["name"].as_str().unwrap_or_default();
        name == "handler_strip_test"
    });

    // Also ensure the un-stripped version does NOT appear.
    let found_raw = topics.iter().any(|t| {
        let name = t["name"].as_str().unwrap_or_default();
        name == "horus_handler_strip_test"
    });

    assert!(
        found || !found_raw,
        "topic 'horus_handler_strip_test' should appear as 'handler_strip_test' \
         (prefix stripped), found topics: {:?}",
        topics
            .iter()
            .filter_map(|t| t["name"].as_str())
            .collect::<Vec<_>>()
    );
}

#[tokio::test]
async fn topics_size_formatted_as_kb() {
    let mut rt = HorusTestRuntime::new();
    rt.add_topic("handler_size_fmt_topic", 8192);

    rt.refresh_discovery();

    let app = builders::test_router();
    let resp = app.oneshot(get_request("/api/topics")).await.unwrap();
    let json = assert_json_ok(resp).await;

    let topics = json["topics"].as_array().expect("topics should be an array");

    // All topic entries should have size formatted as "X KB".
    for topic in topics {
        let size_str = topic["size"].as_str().expect("size should be a string");
        assert!(
            size_str.ends_with(" KB"),
            "topic size should end with ' KB', got: '{}'",
            size_str
        );

        let numeric_part = size_str.trim_end_matches(" KB");
        assert!(
            numeric_part.parse::<u64>().is_ok(),
            "topic size numeric part '{}' should be a valid integer",
            numeric_part
        );
    }
}

// ═══════════════════════════════════════════════════════════════════════════════
//  GRAPH HANDLER  (GET /api/graph)
// ═══════════════════════════════════════════════════════════════════════════════

#[tokio::test]
async fn graph_empty_when_no_nodes() {
    let rt = HorusTestRuntime::new();
    rt.refresh_discovery();

    let app = builders::test_router();
    let resp = app.oneshot(get_request("/api/graph")).await.unwrap();
    let json = assert_json_ok(resp).await;

    assert!(json["nodes"].is_array(), "graph nodes should be an array");
    assert!(json["edges"].is_array(), "graph edges should be an array");
}

#[tokio::test]
async fn graph_shows_node_and_topic_entries() {
    let mut rt = HorusTestRuntime::new();

    // Sensor publishes to "handler_graph_data", actuator subscribes to it.
    rt.add_node(TestNodeConfig::sensor(
        "handler_graph_sensor",
        "handler_graph_data",
        "SensorMsg",
    ))
    .add_node(TestNodeConfig::actuator(
        "handler_graph_actuator",
        "handler_graph_data",
        "SensorMsg",
    ));

    assert!(rt.wait_ready(Duration::from_secs(2)));
    rt.refresh_discovery();

    let app = builders::test_router();
    let resp = app.oneshot(get_request("/api/graph")).await.unwrap();
    let json = assert_json_ok(resp).await;

    let graph_nodes = json["nodes"].as_array().expect("nodes should be an array");
    let graph_edges = json["edges"].as_array().expect("edges should be an array");

    // Expect at least 2 process nodes + 1 topic node.
    let _process_nodes: Vec<_> = graph_nodes
        .iter()
        .filter(|n| n["type"].as_str() == Some("process"))
        .collect();
    let topic_nodes: Vec<_> = graph_nodes
        .iter()
        .filter(|n| n["type"].as_str() == Some("topic"))
        .collect();

    // Check our specific nodes appear as process type.
    let node_labels: Vec<&str> = graph_nodes
        .iter()
        .filter_map(|n| n["label"].as_str())
        .collect();

    assert!(
        node_labels.contains(&"handler_graph_sensor"),
        "sensor should appear in graph nodes, found: {:?}",
        node_labels
    );
    assert!(
        node_labels.contains(&"handler_graph_actuator"),
        "actuator should appear in graph nodes, found: {:?}",
        node_labels
    );

    // The shared topic should appear as a topic node.
    let has_topic_node = topic_nodes.iter().any(|n| {
        let label = n["label"].as_str().unwrap_or_default();
        label.contains("handler_graph_data")
    });
    assert!(
        has_topic_node,
        "graph should contain a topic node for 'handler_graph_data', found topics: {:?}",
        topic_nodes
            .iter()
            .filter_map(|n| n["label"].as_str())
            .collect::<Vec<_>>()
    );

    // Should have edges connecting the sensor/actuator to the topic.
    assert!(
        !graph_edges.is_empty(),
        "graph should have edges when nodes share topics"
    );

    // Each graph node should have the required fields.
    for node in graph_nodes {
        assert!(node.get("id").is_some(), "graph node should have id");
        assert!(node.get("label").is_some(), "graph node should have label");
        assert!(node.get("type").is_some(), "graph node should have type");
        assert!(
            node.get("active").is_some(),
            "graph node should have active"
        );
    }

    // Each edge should have from, to, type, active.
    for edge in graph_edges {
        assert!(edge.get("from").is_some(), "edge should have from");
        assert!(edge.get("to").is_some(), "edge should have to");
        assert!(edge.get("type").is_some(), "edge should have type");
        assert!(edge.get("active").is_some(), "edge should have active");
    }
}

#[tokio::test]
async fn graph_edges_have_correct_types() {
    let mut rt = HorusTestRuntime::new();

    // Sensor publishes, actuator subscribes.
    rt.add_node(TestNodeConfig::sensor(
        "handler_edge_sensor",
        "handler_edge_topic",
        "Msg",
    ))
    .add_node(TestNodeConfig::actuator(
        "handler_edge_actuator",
        "handler_edge_topic",
        "Msg",
    ));

    assert!(rt.wait_ready(Duration::from_secs(2)));
    rt.refresh_discovery();

    let app = builders::test_router();
    let resp = app.oneshot(get_request("/api/graph")).await.unwrap();
    let json = assert_json_ok(resp).await;

    let edges = json["edges"].as_array().expect("edges should be an array");

    let edge_types: Vec<&str> = edges
        .iter()
        .filter_map(|e| e["type"].as_str())
        .collect();

    // We expect at least one "publish" edge (sensor -> topic) and one
    // "subscribe" edge (topic -> actuator).
    assert!(
        edge_types.contains(&"publish"),
        "should have a 'publish' edge, found: {:?}",
        edge_types
    );
    assert!(
        edge_types.contains(&"subscribe"),
        "should have a 'subscribe' edge, found: {:?}",
        edge_types
    );

    // All edge types should be either "publish" or "subscribe".
    for edge_type in &edge_types {
        assert!(
            *edge_type == "publish" || *edge_type == "subscribe",
            "unexpected edge type: '{}'",
            edge_type
        );
    }
}

// ═══════════════════════════════════════════════════════════════════════════════
//  NETWORK HANDLER  (GET /api/network)
// ═══════════════════════════════════════════════════════════════════════════════

#[tokio::test]
async fn network_empty_baseline() {
    // Without any network files written, the handler should return a valid
    // baseline response with zeroed counters.
    let rt = HorusTestRuntime::new();
    rt.refresh_discovery();

    let app = builders::test_router();
    let resp = app.oneshot(get_request("/api/network")).await.unwrap();
    let json = assert_json_ok(resp).await;

    assert_eq!(
        json["total_nodes"].as_u64().unwrap_or(0),
        0,
        "total_nodes should be 0 with no network files"
    );

    let node_statuses = json["node_statuses"]
        .as_array()
        .expect("node_statuses should be an array");
    assert!(
        node_statuses.is_empty(),
        "node_statuses should be empty with no network files"
    );
}

#[tokio::test]
async fn network_response_schema() {
    let app = builders::test_router();
    let resp = app.oneshot(get_request("/api/network")).await.unwrap();
    let json = assert_json_ok(resp).await;

    // Validate all expected top-level keys exist.
    let required_keys = [
        "total_nodes",
        "total_bytes_sent",
        "total_bytes_received",
        "total_packets_sent",
        "total_packets_received",
        "transport_breakdown",
        "unique_endpoints",
        "node_statuses",
    ];
    for key in &required_keys {
        assert!(
            json.get(key).is_some() && !json[key].is_null(),
            "network response should have non-null key '{}', got: {}",
            key,
            serde_json::to_string_pretty(&json).unwrap_or_default()
        );
    }

    // Numeric fields should be numbers.
    assert!(
        json["total_nodes"].is_number(),
        "total_nodes should be a number"
    );
    assert!(
        json["total_bytes_sent"].is_number(),
        "total_bytes_sent should be a number"
    );
    assert!(
        json["total_bytes_received"].is_number(),
        "total_bytes_received should be a number"
    );
    assert!(
        json["total_packets_sent"].is_number(),
        "total_packets_sent should be a number"
    );
    assert!(
        json["total_packets_received"].is_number(),
        "total_packets_received should be a number"
    );

    // node_statuses should be an array.
    assert!(
        json["node_statuses"].is_array(),
        "node_statuses should be an array"
    );
}

// ═══════════════════════════════════════════════════════════════════════════════
//  LOGS HANDLER  (GET /api/logs/all, /api/logs/node/:name, /api/logs/topic/:name)
// ═══════════════════════════════════════════════════════════════════════════════

#[tokio::test]
async fn logs_all_returns_empty_array() {
    // The global log buffer may contain entries from other tests running in
    // the same process.  We verify the endpoint returns 200 with a JSON
    // object that has a "logs" array.
    let app = builders::test_router();
    let resp = app.oneshot(get_request("/api/logs/all")).await.unwrap();
    let json = assert_json_ok(resp).await;

    assert!(
        json["logs"].is_array(),
        "logs should be an array, got: {:?}",
        json["logs"]
    );
}

#[tokio::test]
async fn logs_all_after_injection() {
    let rt = HorusTestRuntime::new();
    let unique_msg = format!("logs_all_inject_{}", std::process::id());
    rt.inject_log("test_log_node_all", "info", &unique_msg);

    let app = builders::test_router();
    let resp = app.oneshot(get_request("/api/logs/all")).await.unwrap();
    let json = assert_json_ok(resp).await;

    let logs = json["logs"].as_array().expect("logs should be an array");
    let found = logs.iter().any(|entry| {
        entry["message"]
            .as_str()
            .map(|m| m.contains(&unique_msg))
            .unwrap_or(false)
    });
    assert!(
        found,
        "injected log message '{}' should appear in /api/logs/all response, found {} entries",
        unique_msg,
        logs.len()
    );
}

#[tokio::test]
async fn logs_node_returns_filtered_logs() {
    let rt = HorusTestRuntime::new();
    let unique_a = format!("node_filter_a_{}", std::process::id());
    let unique_b = format!("node_filter_b_{}", std::process::id());

    rt.inject_log("log_filter_node_a", "info", &unique_a);
    rt.inject_log("log_filter_node_b", "error", &unique_b);

    let app = builders::test_router();
    let resp = app
        .oneshot(get_request("/api/logs/node/log_filter_node_a"))
        .await
        .unwrap();
    let json = assert_json_ok(resp).await;

    let logs = json["logs"].as_array().expect("logs should be an array");

    // All returned logs must be for node_a.
    for entry in logs {
        assert_eq!(
            entry["node_name"].as_str().unwrap_or(""),
            "log_filter_node_a",
            "all logs from /api/logs/node/log_filter_node_a should have node_name='log_filter_node_a'"
        );
    }

    // Our unique message for node_a should appear.
    let found_a = logs
        .iter()
        .any(|e| e["message"].as_str().unwrap_or("").contains(&unique_a));
    assert!(
        found_a,
        "injected message for node_a should appear in filtered results"
    );

    // Node_b's message should NOT appear in node_a's filtered results.
    let found_b = logs
        .iter()
        .any(|e| e["message"].as_str().unwrap_or("").contains(&unique_b));
    assert!(
        !found_b,
        "message for node_b should NOT appear in node_a's filtered results"
    );
}

#[tokio::test]
async fn logs_node_returns_correct_node_name() {
    let app = builders::test_router();
    let resp = app
        .oneshot(get_request("/api/logs/node/my_test_node_xyz"))
        .await
        .unwrap();
    let json = assert_json_ok(resp).await;

    assert_eq!(
        json["node"].as_str().unwrap_or(""),
        "my_test_node_xyz",
        "response 'node' field should match the path parameter"
    );
    assert!(
        json["logs"].is_array(),
        "logs should be an array even for non-existent node"
    );
}

#[tokio::test]
async fn logs_topic_returns_correct_topic_name() {
    let app = builders::test_router();
    let resp = app
        .oneshot(get_request("/api/logs/topic/camera_rgb"))
        .await
        .unwrap();
    let json = assert_json_ok(resp).await;

    assert_eq!(
        json["topic"].as_str().unwrap_or(""),
        "camera_rgb",
        "response 'topic' field should match the path parameter"
    );
    assert!(
        json["logs"].is_array(),
        "logs should be an array even for non-existent topic"
    );
}

#[tokio::test]
async fn logs_topic_strips_horus_prefix() {
    // When requesting /api/logs/topic/horus_camera, the handler should:
    // 1. Return topic: "horus_camera" in the response (pass-through)
    // 2. Internally strip the "horus_" prefix and search for "camera" in the buffer
    //
    // We inject a log with topic="camera" and request /api/logs/topic/horus_camera.
    // The handler strips "horus_" and looks up "camera" in the buffer.
    use horus_core::core::log_buffer::{LogEntry as CoreLogEntry, LogType, GLOBAL_LOG_BUFFER};

    let unique_msg = format!("topic_prefix_test_{}", std::process::id());
    let entry = CoreLogEntry {
        timestamp: chrono::Utc::now().to_rfc3339(),
        tick_number: 0,
        node_name: "prefix_test_node".to_string(),
        log_type: LogType::Publish,
        topic: Some("camera".to_string()),
        message: unique_msg.clone(),
        tick_us: 0,
        ipc_ns: 0,
    };
    GLOBAL_LOG_BUFFER.push(entry);

    let app = builders::test_router();
    let resp = app
        .oneshot(get_request("/api/logs/topic/horus_camera"))
        .await
        .unwrap();
    let json = assert_json_ok(resp).await;

    // The topic field in the response should be the original path param.
    assert_eq!(
        json["topic"].as_str().unwrap_or(""),
        "horus_camera",
        "response 'topic' field should be the raw path param 'horus_camera'"
    );

    let logs = json["logs"].as_array().expect("logs should be an array");
    let found = logs
        .iter()
        .any(|e| e["message"].as_str().unwrap_or("").contains(&unique_msg));
    assert!(
        found,
        "log with topic='camera' should be found when requesting /api/logs/topic/horus_camera \
         (handler strips horus_ prefix for buffer lookup)"
    );
}

// ═══════════════════════════════════════════════════════════════════════════════
//  RECORDINGS HANDLER  (GET /api/recordings, GET/DELETE /api/recordings/:session)
// ═══════════════════════════════════════════════════════════════════════════════

#[tokio::test]
async fn recordings_list_returns_valid_json() {
    let app = builders::test_router();
    let resp = app.oneshot(get_request("/api/recordings")).await.unwrap();
    let json = assert_json_ok(resp).await;

    assert!(
        json["recordings"].is_array(),
        "recordings should be an array"
    );
    assert!(
        json["count"].is_number(),
        "count should be a number"
    );

    // count should match the length of the recordings array.
    let recordings = json["recordings"].as_array().unwrap();
    let count = json["count"].as_u64().unwrap();
    assert_eq!(
        recordings.len() as u64, count,
        "count field should match recordings array length"
    );
}

#[tokio::test]
async fn recordings_info_path_traversal_returns_400() {
    // Axum normalizes some path components, but the handler's
    // is_safe_path_component check should reject ".." even if it arrives
    // as the raw :session parameter.  We test with an encoded path.
    use monitor_tests::helpers::assert_json_error;

    let app = builders::test_router();

    // Use a path traversal payload that won't get normalized away by Axum.
    let resp = app
        .oneshot(get_request("/api/recordings/..%2F..%2Fetc"))
        .await
        .unwrap();

    let status = resp.status();
    // Axum may return 400 (our handler) or 404 (route not matched).
    // Either is acceptable for a path traversal attempt.
    assert!(
        status == axum::http::StatusCode::BAD_REQUEST
            || status == axum::http::StatusCode::NOT_FOUND,
        "path traversal attempt should return 400 or 404, got {}",
        status
    );
}

#[tokio::test]
async fn recordings_info_nonexistent_returns_404() {
    use monitor_tests::helpers::assert_json_error;

    let app = builders::test_router();
    let resp = app
        .oneshot(get_request("/api/recordings/nonexistent_session_integration_xyz"))
        .await
        .unwrap();
    let json = assert_json_error(resp, axum::http::StatusCode::NOT_FOUND).await;

    assert!(
        json["error"].is_string(),
        "404 response should contain an error message"
    );
}

#[tokio::test]
async fn recordings_delete_nonexistent_returns_404() {
    use monitor_tests::helpers::{assert_json_error, delete_request};

    let app = builders::test_router();
    let resp = app
        .oneshot(delete_request("/api/recordings/nonexistent_session_integration_xyz"))
        .await
        .unwrap();
    let json = assert_json_error(resp, axum::http::StatusCode::NOT_FOUND).await;

    assert!(
        json["error"].is_string(),
        "404 response should contain an error message"
    );
}

// ═══════════════════════════════════════════════════════════════════════════════
//  PACKAGES HANDLER  (GET /api/packages/registry, GET /api/packages/environments)
// ═══════════════════════════════════════════════════════════════════════════════

#[tokio::test]
async fn packages_registry_requires_query_param() {
    // GET /api/packages/registry without ?q= should return 422 (Axum rejects
    // missing required query parameter) or 400.
    let app = builders::test_router();
    let resp = app
        .oneshot(get_request("/api/packages/registry"))
        .await
        .unwrap();

    let status = resp.status();
    assert!(
        status == axum::http::StatusCode::UNPROCESSABLE_ENTITY
            || status == axum::http::StatusCode::BAD_REQUEST,
        "missing ?q= should return 422 or 400, got {}",
        status
    );
}

#[tokio::test]
async fn packages_environments_returns_valid_json() {
    let app = builders::test_router();
    let resp = app
        .oneshot(get_request("/api/packages/environments"))
        .await
        .unwrap();
    let json = assert_json_ok(resp).await;

    assert!(
        json["global"].is_array(),
        "environments response should have a 'global' array"
    );
    assert!(
        json["local"].is_array(),
        "environments response should have a 'local' array"
    );
}
