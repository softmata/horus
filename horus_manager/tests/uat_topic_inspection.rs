//! UAT: Topic inspection and graph visualization E2E.
//!
//! Verifies the monitor correctly lists SHM topics with sizes and types,
//! and builds the correct pub/sub graph from node presence data.
//!
//! Each test uses unique node/topic name prefixes for parallel safety.

mod harness;
mod monitor_tests;

use harness::{HorusTestRuntime, TestNodeConfig};
use monitor_tests::builders;
use monitor_tests::helpers::{assert_json_ok, get_request};

use std::sync::atomic::{AtomicU32, Ordering};
use std::time::Duration;
use tower::ServiceExt;

static TEST_COUNTER: AtomicU32 = AtomicU32::new(100);

fn unique_prefix() -> String {
    let id = TEST_COUNTER.fetch_add(1, Ordering::Relaxed);
    format!("uatt{id}_")
}

/// Build a 3-topic, 4-node scenario:
///   - sensor_node publishes "imu" (Pod type)
///   - camera_node publishes "rgb" (Image type)
///   - lidar_node publishes "points" (PointCloud type)
///   - planner_node subscribes to "imu" and "rgb"
fn build_topic_scenario(pfx: &str) -> HorusTestRuntime {
    let mut rt = HorusTestRuntime::new();
    rt.add_node(TestNodeConfig::sensor(
        &format!("{pfx}sensor_node"),
        "imu",
        "Pod",
    ))
    .add_node(TestNodeConfig::sensor(
        &format!("{pfx}camera_node"),
        "rgb",
        "Image",
    ))
    .add_node(TestNodeConfig::sensor(
        &format!("{pfx}lidar_node"),
        "points",
        "PointCloud",
    ))
    .add_node(
        TestNodeConfig::bare(&format!("{pfx}planner_node"))
            .with_subscriber("imu", "Pod")
            .with_subscriber("rgb", "Image"),
    )
    // Create SHM topic files so discover_shared_memory() finds them
    .add_topic(&format!("{pfx}imu"), 4096)
    .add_topic(&format!("{pfx}rgb"), 65536)
    .add_topic(&format!("{pfx}points"), 131072);
    rt
}

// ═══════════════════════════════════════════════════════════════════════════════
//  Topic listing via /api/topics
// ═══════════════════════════════════════════════════════════════════════════════

#[tokio::test]
async fn api_topics_lists_all_created_topics() {
    let pfx = unique_prefix();
    let mut rt = build_topic_scenario(&pfx);
    assert!(rt.wait_ready(Duration::from_secs(3)));
    rt.refresh_discovery();

    let app = builders::test_router();
    let resp = app.oneshot(get_request("/api/topics")).await.unwrap();
    let json = assert_json_ok(resp).await;

    let topics = json["topics"]
        .as_array()
        .expect("/api/topics must return a topics array");

    // Our topics should appear (possibly with horus_ prefix stripped)
    for suffix in &[
        format!("{pfx}imu"),
        format!("{pfx}rgb"),
        format!("{pfx}points"),
    ] {
        let found = topics.iter().any(|t| {
            let name = t["name"].as_str().unwrap_or("");
            name.contains(suffix.as_str())
        });
        assert!(
            found,
            "topic containing '{}' not found in /api/topics",
            suffix
        );
    }
}

#[tokio::test]
async fn api_topics_have_non_zero_sizes() {
    let pfx = unique_prefix();
    let mut rt = build_topic_scenario(&pfx);
    assert!(rt.wait_ready(Duration::from_secs(3)));
    rt.refresh_discovery();

    let app = builders::test_router();
    let resp = app.oneshot(get_request("/api/topics")).await.unwrap();
    let json = assert_json_ok(resp).await;

    let topics = json["topics"].as_array().unwrap();

    // Find our topics and check sizes
    for suffix in &[
        format!("{pfx}imu"),
        format!("{pfx}rgb"),
        format!("{pfx}points"),
    ] {
        if let Some(topic) = topics
            .iter()
            .find(|t| t["name"].as_str().unwrap_or("").contains(suffix.as_str()))
        {
            // Size is formatted as "X KB" string
            let size_str = topic["size"].as_str().unwrap_or("0 KB");
            assert!(
                !size_str.starts_with("0 "),
                "topic '{}' should have non-zero size, got '{}'",
                suffix,
                size_str
            );
        }
    }
}

#[tokio::test]
async fn api_topics_different_sizes_reflect_actual() {
    let pfx = unique_prefix();
    let mut rt = build_topic_scenario(&pfx);
    assert!(rt.wait_ready(Duration::from_secs(3)));
    rt.refresh_discovery();

    let app = builders::test_router();
    let resp = app.oneshot(get_request("/api/topics")).await.unwrap();
    let json = assert_json_ok(resp).await;

    let topics = json["topics"].as_array().unwrap();

    // rgb was 65536 bytes (64 KB), points was 131072 (128 KB), imu was 4096 (4 KB)
    // The handler formats as "X KB", so we check relative ordering
    let find_topic_size_kb = |name_contains: &str| -> Option<u64> {
        topics.iter().find_map(|t| {
            let name = t["name"].as_str().unwrap_or("");
            if name.contains(name_contains) {
                let size_str = t["size"].as_str().unwrap_or("0 KB");
                // Parse "X KB" format
                size_str.trim_end_matches(" KB").parse::<u64>().ok()
            } else {
                None
            }
        })
    };

    let imu_kb = find_topic_size_kb(&format!("{pfx}imu"));
    let rgb_kb = find_topic_size_kb(&format!("{pfx}rgb"));
    let points_kb = find_topic_size_kb(&format!("{pfx}points"));

    if let (Some(imu), Some(rgb), Some(points)) = (imu_kb, rgb_kb, points_kb) {
        assert!(
            imu < rgb,
            "imu ({} KB) should be smaller than rgb ({} KB)",
            imu,
            rgb
        );
        assert!(
            rgb < points,
            "rgb ({} KB) should be smaller than points ({} KB)",
            rgb,
            points
        );
    }
}

// ═══════════════════════════════════════════════════════════════════════════════
//  Topic removal
// ═══════════════════════════════════════════════════════════════════════════════

#[tokio::test]
async fn destroyed_topic_disappears_from_listing() {
    let pfx = unique_prefix();
    let mut rt = build_topic_scenario(&pfx);
    assert!(rt.wait_ready(Duration::from_secs(3)));
    rt.refresh_discovery();

    let imu_topic_name = format!("{pfx}imu");

    // Verify imu topic exists initially
    let app = builders::test_router();
    let resp = app.oneshot(get_request("/api/topics")).await.unwrap();
    let json = assert_json_ok(resp).await;
    let topics = json["topics"].as_array().unwrap();
    assert!(
        topics
            .iter()
            .any(|t| t["name"].as_str().unwrap_or("").contains(&imu_topic_name)),
        "imu topic must exist initially"
    );

    // Remove the imu topic file
    let imu_path = rt
        .topic_paths()
        .iter()
        .find(|p| p.to_string_lossy().contains(&imu_topic_name))
        .cloned()
        .expect("imu topic path must exist");
    std::fs::remove_file(&imu_path).expect("removing topic file should work");

    // Wait for discovery cache to expire
    rt.refresh_discovery();

    // Verify imu topic is gone
    let app = builders::test_router();
    let resp = app.oneshot(get_request("/api/topics")).await.unwrap();
    let json = assert_json_ok(resp).await;
    let topics = json["topics"].as_array().unwrap();
    assert!(
        !topics
            .iter()
            .any(|t| t["name"].as_str().unwrap_or("").contains(&imu_topic_name)),
        "destroyed imu topic must not appear in listing"
    );
}

// ═══════════════════════════════════════════════════════════════════════════════
//  Graph visualization via /api/graph
// ═══════════════════════════════════════════════════════════════════════════════

#[tokio::test]
async fn api_graph_returns_nodes_and_edges() {
    let pfx = unique_prefix();
    let mut rt = build_topic_scenario(&pfx);
    assert!(rt.wait_ready(Duration::from_secs(3)));
    rt.refresh_discovery();

    let app = builders::test_router();
    let resp = app.oneshot(get_request("/api/graph")).await.unwrap();
    let json = assert_json_ok(resp).await;

    assert!(json["nodes"].is_array(), "graph must have nodes array");
    assert!(json["edges"].is_array(), "graph must have edges array");
}

#[tokio::test]
async fn api_graph_contains_process_nodes() {
    let pfx = unique_prefix();
    let mut rt = build_topic_scenario(&pfx);
    assert!(rt.wait_ready(Duration::from_secs(3)));
    rt.refresh_discovery();

    let app = builders::test_router();
    let resp = app.oneshot(get_request("/api/graph")).await.unwrap();
    let json = assert_json_ok(resp).await;

    let nodes = json["nodes"].as_array().unwrap();
    let process_nodes: Vec<_> = nodes
        .iter()
        .filter(|n| n["type"].as_str() == Some("process"))
        .collect();

    // At least our 4 process nodes should appear
    for suffix in &["sensor_node", "camera_node", "lidar_node", "planner_node"] {
        let name = format!("{pfx}{suffix}");
        assert!(
            process_nodes
                .iter()
                .any(|n| n["label"].as_str() == Some(&name)),
            "process node '{}' not found in graph",
            name
        );
    }
}

#[tokio::test]
async fn api_graph_contains_topic_nodes() {
    let pfx = unique_prefix();
    let mut rt = build_topic_scenario(&pfx);
    assert!(rt.wait_ready(Duration::from_secs(3)));
    rt.refresh_discovery();

    let app = builders::test_router();
    let resp = app.oneshot(get_request("/api/graph")).await.unwrap();
    let json = assert_json_ok(resp).await;

    let nodes = json["nodes"].as_array().unwrap();
    let topic_nodes: Vec<_> = nodes
        .iter()
        .filter(|n| n["type"].as_str() == Some("topic"))
        .collect();

    // Topics from node presence data (imu, rgb, points)
    for topic_name in &["imu", "rgb", "points"] {
        assert!(
            topic_nodes.iter().any(|n| {
                let label = n["label"].as_str().unwrap_or("");
                label == *topic_name
            }),
            "topic node '{}' not found in graph",
            topic_name
        );
    }
}

#[tokio::test]
async fn api_graph_has_publish_edges() {
    let pfx = unique_prefix();
    let mut rt = build_topic_scenario(&pfx);
    assert!(rt.wait_ready(Duration::from_secs(3)));
    rt.refresh_discovery();

    let app = builders::test_router();
    let resp = app.oneshot(get_request("/api/graph")).await.unwrap();
    let json = assert_json_ok(resp).await;

    let edges = json["edges"].as_array().unwrap();
    let publish_edges: Vec<_> = edges
        .iter()
        .filter(|e| e["type"].as_str() == Some("publish"))
        .collect();

    // sensor_node → imu, camera_node → rgb, lidar_node → points
    assert!(
        publish_edges.len() >= 3,
        "expected at least 3 publish edges, got {}",
        publish_edges.len()
    );
}

#[tokio::test]
async fn api_graph_has_subscribe_edges() {
    let pfx = unique_prefix();
    let mut rt = build_topic_scenario(&pfx);
    assert!(rt.wait_ready(Duration::from_secs(3)));
    rt.refresh_discovery();

    let app = builders::test_router();
    let resp = app.oneshot(get_request("/api/graph")).await.unwrap();
    let json = assert_json_ok(resp).await;

    let edges = json["edges"].as_array().unwrap();
    let subscribe_edges: Vec<_> = edges
        .iter()
        .filter(|e| e["type"].as_str() == Some("subscribe"))
        .collect();

    // planner_node subscribes to imu and rgb → 2 subscribe edges
    assert!(
        subscribe_edges.len() >= 2,
        "expected at least 2 subscribe edges, got {}",
        subscribe_edges.len()
    );
}

#[tokio::test]
async fn api_graph_edges_reference_valid_node_ids() {
    let pfx = unique_prefix();
    let mut rt = build_topic_scenario(&pfx);
    assert!(rt.wait_ready(Duration::from_secs(3)));
    rt.refresh_discovery();

    let app = builders::test_router();
    let resp = app.oneshot(get_request("/api/graph")).await.unwrap();
    let json = assert_json_ok(resp).await;

    let nodes = json["nodes"].as_array().unwrap();
    let edges = json["edges"].as_array().unwrap();

    let node_ids: Vec<String> = nodes
        .iter()
        .filter_map(|n| n["id"].as_str().map(|s| s.to_string()))
        .collect();

    for edge in edges {
        let from = edge["from"].as_str().expect("edge must have 'from'");
        let to = edge["to"].as_str().expect("edge must have 'to'");

        assert!(
            node_ids.contains(&from.to_string()),
            "edge 'from' id '{}' not found in graph nodes",
            from
        );
        assert!(
            node_ids.contains(&to.to_string()),
            "edge 'to' id '{}' not found in graph nodes",
            to
        );
    }
}

// ═══════════════════════════════════════════════════════════════════════════════
//  Topology change reflected in graph
// ═══════════════════════════════════════════════════════════════════════════════

#[tokio::test]
async fn graph_updates_after_node_removal() {
    let pfx = unique_prefix();
    let mut rt = build_topic_scenario(&pfx);
    assert!(rt.wait_ready(Duration::from_secs(3)));
    rt.refresh_discovery();

    let lidar_name = format!("{pfx}lidar_node");

    // Get initial graph
    let app = builders::test_router();
    let resp = app.oneshot(get_request("/api/graph")).await.unwrap();
    let json = assert_json_ok(resp).await;
    let initial_nodes = json["nodes"].as_array().unwrap();
    assert!(
        initial_nodes
            .iter()
            .any(|n| n["label"].as_str() == Some(&lidar_name)),
        "lidar_node must be in initial graph"
    );

    // Remove lidar node presence file
    let lidar_presence = rt
        .presence_paths()
        .iter()
        .find(|p| p.to_string_lossy().contains(&lidar_name))
        .cloned()
        .expect("lidar presence path must exist");
    std::fs::remove_file(&lidar_presence).expect("removing presence file should work");

    rt.refresh_discovery();

    // Graph should no longer contain the lidar process node
    let app = builders::test_router();
    let resp = app.oneshot(get_request("/api/graph")).await.unwrap();
    let json = assert_json_ok(resp).await;
    let updated_nodes = json["nodes"].as_array().unwrap();
    assert!(
        !updated_nodes
            .iter()
            .any(|n| n["label"].as_str() == Some(&lidar_name)),
        "lidar_node must not be in graph after removal"
    );
}

#[tokio::test]
async fn graph_updates_after_node_addition() {
    let pfx = unique_prefix();
    let mut rt = build_topic_scenario(&pfx);
    assert!(rt.wait_ready(Duration::from_secs(3)));
    rt.refresh_discovery();

    let new_name = format!("{pfx}new_actuator");
    rt.add_node(TestNodeConfig::actuator(&new_name, "points", "PointCloud"));
    rt.refresh_discovery();

    let app = builders::test_router();
    let resp = app.oneshot(get_request("/api/graph")).await.unwrap();
    let json = assert_json_ok(resp).await;

    let nodes = json["nodes"].as_array().unwrap();
    assert!(
        nodes.iter().any(|n| n["label"].as_str() == Some(&new_name)),
        "dynamically added node must appear in graph"
    );

    // Should also create a subscribe edge to "points"
    let edges = json["edges"].as_array().unwrap();
    let sub_to_new = edges.iter().any(|e| {
        e["type"].as_str() == Some("subscribe")
            && e["to"].as_str().map_or(false, |s| s.contains(&new_name))
    });
    assert!(
        sub_to_new,
        "new actuator must have a subscribe edge to 'points'"
    );
}

// ═══════════════════════════════════════════════════════════════════════════════
//  SHM cleanup
// ═══════════════════════════════════════════════════════════════════════════════

#[test]
fn no_topic_file_leak_after_drop() {
    let pfx = unique_prefix();
    let topic_paths;
    let presence_paths;
    {
        let rt = build_topic_scenario(&pfx);
        assert!(rt.wait_ready(Duration::from_secs(3)));
        topic_paths = rt.topic_paths().to_vec();
        presence_paths = rt.presence_paths().to_vec();
    }
    for p in &topic_paths {
        assert!(
            !p.exists(),
            "topic file leak: {} still exists after drop",
            p.display()
        );
    }
    for p in &presence_paths {
        assert!(
            !p.exists(),
            "presence file leak: {} still exists after drop",
            p.display()
        );
    }
}
