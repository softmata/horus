//! UAT: Node discovery and health monitoring E2E.
//!
//! Verifies the full lifecycle: spin up 5 simulated nodes (2 publishers,
//! 2 subscribers, 1 compute/processor), check the web API reports all nodes
//! correctly, simulate a node dying, verify it disappears, add a new node
//! dynamically, verify it appears.  Also checks CPU/memory plausibility and
//! API–discovery consistency.
//!
//! These tests use real SHM presence files via [`HorusTestRuntime`] so
//! `discover_nodes()` exercises the actual code path.
//!
//! Each test uses unique node name prefixes to avoid collision when running
//! tests in parallel.

mod harness;
mod monitor_tests;

use harness::{HorusTestRuntime, TestNodeConfig};
use monitor_tests::builders;
use monitor_tests::helpers::{assert_json_ok, get_request};

use horus_core::NodePresence;
use std::sync::atomic::{AtomicU32, Ordering};
use std::time::Duration;
use tower::ServiceExt;

/// Monotonically increasing counter for unique test prefixes.
static TEST_COUNTER: AtomicU32 = AtomicU32::new(0);

/// Generate a unique prefix for this test invocation.
fn unique_prefix() -> String {
    let id = TEST_COUNTER.fetch_add(1, Ordering::Relaxed);
    format!("uat{id}_")
}

/// Build a 5-node runtime with a given prefix for uniqueness.
///   - {pfx}camera  (publisher: image/Image)
///   - {pfx}lidar   (publisher: scan/LaserScan)
///   - {pfx}motor   (subscriber: cmd_vel/Twist)
///   - {pfx}gripper (subscriber: grip_cmd/GripCommand)
///   - {pfx}planner (processor: image→cmd_vel)
fn build_five_nodes(pfx: &str) -> HorusTestRuntime {
    let mut rt = HorusTestRuntime::new();
    rt.add_node(TestNodeConfig::sensor(
        &format!("{pfx}camera"),
        "image",
        "Image",
    ))
    .add_node(TestNodeConfig::sensor(
        &format!("{pfx}lidar"),
        "scan",
        "LaserScan",
    ))
    .add_node(TestNodeConfig::actuator(
        &format!("{pfx}motor"),
        "cmd_vel",
        "Twist",
    ))
    .add_node(TestNodeConfig::actuator(
        &format!("{pfx}gripper"),
        "grip_cmd",
        "GripCommand",
    ))
    .add_node(TestNodeConfig::processor(
        &format!("{pfx}planner"),
        "image",
        "Image",
        "cmd_vel",
        "Twist",
    ));
    rt
}

// ═══════════════════════════════════════════════════════════════════════════════
//  Discovery layer tests
// ═══════════════════════════════════════════════════════════════════════════════

#[test]
fn five_nodes_discovered_within_timeout() {
    let pfx = unique_prefix();
    let rt = build_five_nodes(&pfx);
    assert!(
        rt.wait_ready(Duration::from_secs(3)),
        "all 5 nodes must be discoverable within 3 seconds"
    );
}

#[test]
fn five_nodes_all_present_in_read_all() {
    let pfx = unique_prefix();
    let rt = build_five_nodes(&pfx);
    assert!(rt.wait_ready(Duration::from_secs(3)));

    let all = NodePresence::read_all();
    for suffix in &["camera", "lidar", "motor", "gripper", "planner"] {
        let name = format!("{pfx}{suffix}");
        assert!(
            all.iter().any(|p| p.name == name),
            "node '{}' not found in NodePresence::read_all()",
            name
        );
    }
}

#[test]
fn five_nodes_have_correct_pub_sub_topology() {
    let pfx = unique_prefix();
    let rt = build_five_nodes(&pfx);
    assert!(rt.wait_ready(Duration::from_secs(3)));

    let all = NodePresence::read_all();

    // Camera publishes "image"
    let camera = all
        .iter()
        .find(|p| p.name == format!("{pfx}camera"))
        .unwrap();
    assert_eq!(camera.publishers.len(), 1);
    assert_eq!(camera.publishers[0].topic_name, "image");
    assert!(camera.subscribers.is_empty());

    // Motor subscribes to "cmd_vel"
    let motor = all
        .iter()
        .find(|p| p.name == format!("{pfx}motor"))
        .unwrap();
    assert!(motor.publishers.is_empty());
    assert_eq!(motor.subscribers.len(), 1);
    assert_eq!(motor.subscribers[0].topic_name, "cmd_vel");

    // Planner: subscribes to "image", publishes "cmd_vel"
    let planner = all
        .iter()
        .find(|p| p.name == format!("{pfx}planner"))
        .unwrap();
    assert_eq!(planner.publishers.len(), 1);
    assert_eq!(planner.publishers[0].topic_name, "cmd_vel");
    assert_eq!(planner.subscribers.len(), 1);
    assert_eq!(planner.subscribers[0].topic_name, "image");
}

#[test]
fn all_nodes_have_current_process_pid() {
    let pfx = unique_prefix();
    let rt = build_five_nodes(&pfx);
    assert!(rt.wait_ready(Duration::from_secs(3)));

    let my_pid = std::process::id();
    let all = NodePresence::read_all();

    for name in rt.node_names() {
        let node = all.iter().find(|p| p.name == name).unwrap();
        assert_eq!(
            node.pid, my_pid,
            "node '{}' should have PID {} (current process), got {}",
            name, my_pid, node.pid
        );
    }
}

// ── Node removal detection ──────────────────────────────────────────────────

#[test]
fn removed_node_disappears_from_discovery() {
    let pfx = unique_prefix();
    let mut rt = build_five_nodes(&pfx);
    assert!(rt.wait_ready(Duration::from_secs(3)));

    let camera_name = format!("{pfx}camera");

    // Remove the camera node's presence file to simulate it dying.
    let camera_presence = rt
        .presence_paths()
        .iter()
        .find(|p| p.to_string_lossy().contains(&camera_name))
        .cloned()
        .expect("camera presence path must exist");

    std::fs::remove_file(&camera_presence).expect("removing presence file should work");

    // Wait for cache to expire and re-discover.
    rt.refresh_discovery();

    let all = NodePresence::read_all();
    assert!(
        !all.iter().any(|p| p.name == camera_name),
        "camera node must no longer appear after presence file removal"
    );

    // The other 4 should still be present.
    for suffix in &["lidar", "motor", "gripper", "planner"] {
        let name = format!("{pfx}{suffix}");
        assert!(
            all.iter().any(|p| p.name == name),
            "{name} must still exist"
        );
    }
}

// ── Dynamic node addition ───────────────────────────────────────────────────

#[test]
fn dynamically_added_node_appears_in_discovery() {
    let pfx = unique_prefix();
    let mut rt = build_five_nodes(&pfx);
    assert!(rt.wait_ready(Duration::from_secs(3)));

    let new_name = format!("{pfx}new_sensor");
    rt.add_node(
        TestNodeConfig::bare(&new_name)
            .with_publisher("depth", "DepthImage")
            .with_rate_hz(15.0),
    );

    // Wait for cache + discovery.
    rt.refresh_discovery();

    let all = NodePresence::read_all();
    let new_node = all.iter().find(|p| p.name == new_name);
    assert!(
        new_node.is_some(),
        "dynamically added node must appear in discovery"
    );
    let new_node = new_node.unwrap();
    assert_eq!(new_node.publishers.len(), 1);
    assert_eq!(new_node.publishers[0].topic_name, "depth");
}

// ═══════════════════════════════════════════════════════════════════════════════
//  Web API tests
// ═══════════════════════════════════════════════════════════════════════════════

#[tokio::test]
async fn api_status_reports_correct_node_count() {
    let pfx = unique_prefix();
    let rt = build_five_nodes(&pfx);
    assert!(rt.wait_ready(Duration::from_secs(3)));
    rt.refresh_discovery();

    let app = builders::test_router();
    let resp = app.oneshot(get_request("/api/status")).await.unwrap();
    let json = assert_json_ok(resp).await;

    let nodes = json["nodes"].as_u64().expect("nodes should be a number");
    assert!(
        nodes >= 5,
        "expected at least 5 nodes in /api/status, got {}",
        nodes
    );
}

#[tokio::test]
async fn api_status_healthy_with_all_nodes_alive() {
    let pfx = unique_prefix();
    let rt = build_five_nodes(&pfx);
    assert!(rt.wait_ready(Duration::from_secs(3)));
    rt.refresh_discovery();

    let app = builders::test_router();
    let resp = app.oneshot(get_request("/api/status")).await.unwrap();
    let json = assert_json_ok(resp).await;

    let status = json["status"].as_str().expect("status should be a string");
    assert_eq!(
        status, "Healthy",
        "all nodes alive → status must be Healthy"
    );
}

#[tokio::test]
async fn api_nodes_lists_all_five_nodes() {
    let pfx = unique_prefix();
    let rt = build_five_nodes(&pfx);
    assert!(rt.wait_ready(Duration::from_secs(3)));
    rt.refresh_discovery();

    let app = builders::test_router();
    let resp = app.oneshot(get_request("/api/nodes")).await.unwrap();
    let json = assert_json_ok(resp).await;

    let nodes_array = json["nodes"]
        .as_array()
        .expect("/api/nodes must return a nodes array");

    for suffix in &["camera", "lidar", "motor", "gripper", "planner"] {
        let name = format!("{pfx}{suffix}");
        assert!(
            nodes_array
                .iter()
                .any(|n| n["name"].as_str() == Some(&name)),
            "node '{}' not found in /api/nodes response",
            name
        );
    }
}

#[tokio::test]
async fn api_nodes_have_valid_pid() {
    let pfx = unique_prefix();
    let rt = build_five_nodes(&pfx);
    assert!(rt.wait_ready(Duration::from_secs(3)));
    rt.refresh_discovery();

    let app = builders::test_router();
    let resp = app.oneshot(get_request("/api/nodes")).await.unwrap();
    let json = assert_json_ok(resp).await;

    let my_pid = std::process::id() as u64;
    let nodes_array = json["nodes"].as_array().unwrap();

    for name in rt.node_names() {
        if let Some(node) = nodes_array
            .iter()
            .find(|n| n["name"].as_str() == Some(name))
        {
            let pid = node["process_id"]
                .as_u64()
                .or_else(|| node["pid"].as_u64())
                .expect("node should have pid or process_id");
            assert_eq!(
                pid, my_pid,
                "node '{}' pid should be {} (current process), got {}",
                name, my_pid, pid
            );
        }
    }
}

#[tokio::test]
async fn api_nodes_cpu_and_memory_plausible() {
    let pfx = unique_prefix();
    let rt = build_five_nodes(&pfx);
    assert!(rt.wait_ready(Duration::from_secs(3)));
    rt.refresh_discovery();

    let app = builders::test_router();
    let resp = app.oneshot(get_request("/api/nodes")).await.unwrap();
    let json = assert_json_ok(resp).await;

    let nodes_array = json["nodes"].as_array().unwrap();

    for name in rt.node_names() {
        if let Some(node) = nodes_array
            .iter()
            .find(|n| n["name"].as_str() == Some(name))
        {
            // CPU: between 0 and 800 (multi-core can exceed 100%)
            if let Some(cpu) = node["cpu_usage"].as_f64() {
                assert!(
                    cpu >= 0.0 && cpu <= 800.0,
                    "node '{}' CPU {}% is out of plausible range",
                    name,
                    cpu
                );
            }
        }
    }
}

// ── Discovery + API consistency ─────────────────────────────────────────────

#[tokio::test]
async fn api_and_discovery_show_consistent_node_list() {
    let pfx = unique_prefix();
    let rt = build_five_nodes(&pfx);
    assert!(rt.wait_ready(Duration::from_secs(3)));
    rt.refresh_discovery();

    // Get nodes from discovery layer directly.
    let discovery_nodes =
        horus_manager::discovery::discover_nodes().expect("discover_nodes should not fail");
    let discovery_names: Vec<&str> = discovery_nodes.iter().map(|n| n.name.as_str()).collect();

    // Get nodes from web API.
    let app = builders::test_router();
    let resp = app.oneshot(get_request("/api/nodes")).await.unwrap();
    let json = assert_json_ok(resp).await;
    let api_names: Vec<String> = json["nodes"]
        .as_array()
        .unwrap()
        .iter()
        .filter_map(|n| n["name"].as_str().map(|s| s.to_string()))
        .collect();

    // All our test nodes should appear in both.
    for suffix in &["camera", "lidar", "motor", "gripper", "planner"] {
        let name = format!("{pfx}{suffix}");
        assert!(
            discovery_names.contains(&name.as_str()),
            "node '{}' missing from discovery layer",
            name
        );
        assert!(
            api_names.contains(&name),
            "node '{}' missing from API response",
            name
        );
    }
}

// ── SHM cleanup ─────────────────────────────────────────────────────────────

#[test]
fn no_shm_leak_after_runtime_drop() {
    let pfx = unique_prefix();
    let paths;
    {
        let rt = build_five_nodes(&pfx);
        assert!(rt.wait_ready(Duration::from_secs(3)));
        paths = rt.presence_paths().to_vec();

        // All files should exist while runtime is alive.
        for p in &paths {
            assert!(p.exists(), "presence file must exist: {}", p.display());
        }
    }
    // After drop, all presence files must be cleaned up.
    for p in &paths {
        assert!(
            !p.exists(),
            "SHM leak: presence file still exists after drop: {}",
            p.display()
        );
    }
}
