//! Stress test: rapid node connect/disconnect churn.
//!
//! Simulates rapid node lifecycle changes — nodes appearing and disappearing
//! — while concurrently polling API endpoints.
//!
//! Run with: cargo test -p horus_manager --test stress_churn -- --test-threads=1

mod harness;
mod monitor_tests;

use harness::{HorusTestRuntime, TestNodeConfig};
use monitor_tests::builders;
use monitor_tests::helpers::get_request;

use std::sync::atomic::{AtomicU32, Ordering};
use std::time::Instant;
use tower::ServiceExt;
use horus_core::core::DurationExt;

static CHURN_COUNTER: AtomicU32 = AtomicU32::new(0);

fn unique_churn_name(prefix: &str) -> String {
    let id = CHURN_COUNTER.fetch_add(1, Ordering::Relaxed);
    format!("{}_{:06}", prefix, id)
}

// ═══════════════════════════════════════════════════════════════════════════════
//  Basic churn — add and remove nodes rapidly
// ═══════════════════════════════════════════════════════════════════════════════

#[test]
fn churn_add_remove_50_nodes_sequentially() {
    let nodes_dir = horus_core::memory::shm_base_dir().join("nodes");
    std::fs::create_dir_all(&nodes_dir).ok();

    let mut created_files = Vec::new();
    let start = Instant::now();

    for _cycle in 0..50 {
        // Create a node
        let name = unique_churn_name("churn");
        let mut rt = HorusTestRuntime::new();
        rt.add_node(TestNodeConfig::bare(&name));

        let path = nodes_dir.join(format!("{}.json", name));
        created_files.push(path.clone());

        // Remove it immediately
        drop(rt); // Drop triggers cleanup
    }

    let elapsed = start.elapsed();
    assert!(
        elapsed.as_secs() < 10,
        "50 add/remove cycles must complete in <10s, took {:?}",
        elapsed
    );

    // All files should be cleaned up
    for f in &created_files {
        assert!(!f.exists(), "churn file must be cleaned up: {:?}", f);
    }
}

// ═══════════════════════════════════════════════════════════════════════════════
//  API stability during churn
// ═══════════════════════════════════════════════════════════════════════════════

#[tokio::test]
async fn churn_api_nodes_stable_during_lifecycle() {
    let app = builders::test_router();

    // Create initial set of stable nodes
    let mut stable_rt = HorusTestRuntime::new();
    for i in 0..10 {
        stable_rt.add_node(TestNodeConfig::bare(&format!("stable_churn_{}", i)));
    }
    stable_rt.wait_ready(5_u64.secs());

    // Poll API while churning additional nodes
    for cycle in 0..20 {
        // Create a transient node
        let mut transient = HorusTestRuntime::new();
        transient.add_node(TestNodeConfig::bare(&unique_churn_name("transient")));

        // Poll API — must always return valid JSON
        let resp = app
            .clone()
            .oneshot(get_request("/api/nodes"))
            .await
            .unwrap();
        assert_eq!(
            resp.status(),
            axum::http::StatusCode::OK,
            "cycle {}: /api/nodes must return 200 during churn",
            cycle
        );

        let body = axum::body::to_bytes(resp.into_body(), 10 * 1024 * 1024)
            .await
            .unwrap();
        let json: serde_json::Value =
            serde_json::from_slice(&body).expect("response must be valid JSON during churn");
        assert!(
            json["nodes"].is_array(),
            "cycle {}: must have nodes array",
            cycle
        );

        // Remove transient node
        drop(transient);
    }
}

#[tokio::test]
async fn churn_api_graph_stable_during_lifecycle() {
    let app = builders::test_router();

    let mut stable_rt = HorusTestRuntime::new();
    for i in 0..5 {
        stable_rt.add_node(TestNodeConfig::sensor(
            &format!("graph_churn_pub_{}", i),
            &format!("churn_topic_{}", i),
            "ChurnData",
        ));
    }
    stable_rt.wait_ready(5_u64.secs());

    for cycle in 0..15 {
        let mut transient = HorusTestRuntime::new();
        transient.add_node(TestNodeConfig::sensor(
            &unique_churn_name("graph_transient"),
            "transient_topic",
            "Data",
        ));

        let resp = app
            .clone()
            .oneshot(get_request("/api/graph"))
            .await
            .unwrap();
        assert_eq!(
            resp.status(),
            axum::http::StatusCode::OK,
            "cycle {}: /api/graph must return 200 during churn",
            cycle
        );

        let body = axum::body::to_bytes(resp.into_body(), 10 * 1024 * 1024)
            .await
            .unwrap();
        let json: serde_json::Value =
            serde_json::from_slice(&body).expect("graph response must be valid JSON during churn");
        assert!(
            json["nodes"].is_array() || json["graph"].is_object(),
            "cycle {}: graph must have valid structure",
            cycle
        );

        drop(transient);
    }
}

#[tokio::test]
async fn churn_api_topics_stable_during_lifecycle() {
    let app = builders::test_router();

    for cycle in 0..15 {
        let mut rt = HorusTestRuntime::new();
        rt.add_topic(&unique_churn_name("churn_topic"), 2048);

        let resp = app
            .clone()
            .oneshot(get_request("/api/topics"))
            .await
            .unwrap();
        assert_eq!(
            resp.status(),
            axum::http::StatusCode::OK,
            "cycle {}: /api/topics must return 200 during churn",
            cycle
        );

        let body = axum::body::to_bytes(resp.into_body(), 10 * 1024 * 1024)
            .await
            .unwrap();
        let json: serde_json::Value =
            serde_json::from_slice(&body).expect("topics response must be valid JSON during churn");
        assert!(
            json["topics"].is_array(),
            "cycle {}: must have topics array",
            cycle
        );

        drop(rt);
    }
}

// ═══════════════════════════════════════════════════════════════════════════════
//  WebSocket stability during churn
// ═══════════════════════════════════════════════════════════════════════════════

#[tokio::test]
async fn churn_websocket_stable_during_lifecycle() {
    let app = builders::test_router();
    let listener = tokio::net::TcpListener::bind("127.0.0.1:0").await.unwrap();
    let addr = listener.local_addr().unwrap();
    tokio::spawn(async move {
        axum::serve(listener, app).await.unwrap();
    });

    let url = format!("ws://{}/api/ws", addr);
    let (mut ws, _) = tokio_tungstenite::connect_async(&url)
        .await
        .expect("WS connect must succeed");

    use futures_util::StreamExt;

    // Create and destroy nodes while receiving WS broadcasts
    for cycle in 0..5 {
        let mut rt = HorusTestRuntime::new();
        rt.add_node(TestNodeConfig::bare(&unique_churn_name("ws_churn")));

        // Receive a broadcast
        let msg = tokio::time::timeout(3_u64.secs(), ws.next())
            .await
            .unwrap_or_else(|_| panic!("WS broadcast {} timed out during churn", cycle))
            .unwrap()
            .unwrap();

        let text = msg.into_text().unwrap();
        let json: serde_json::Value =
            serde_json::from_str(&text).expect("WS broadcast must be valid JSON during churn");
        assert_eq!(json["type"], "update");

        drop(rt);
    }
}

// ═══════════════════════════════════════════════════════════════════════════════
//  SHM cleanup after churn — baseline restored
// ═══════════════════════════════════════════════════════════════════════════════

#[test]
fn churn_shm_baseline_restored_after_churn() {
    let nodes_dir = horus_core::memory::shm_base_dir().join("nodes");
    std::fs::create_dir_all(&nodes_dir).ok();

    // Count baseline presence files
    let baseline = std::fs::read_dir(&nodes_dir)
        .map(|rd| rd.count())
        .unwrap_or(0);

    // Run churn
    for _ in 0..30 {
        let mut rt = HorusTestRuntime::new();
        rt.add_node(TestNodeConfig::bare(&unique_churn_name("shm_churn")));
        drop(rt);
    }

    // Count after churn — should be back to baseline
    let after = std::fs::read_dir(&nodes_dir)
        .map(|rd| rd.count())
        .unwrap_or(0);

    assert_eq!(
        after, baseline,
        "SHM node count must return to baseline after churn: before={}, after={}",
        baseline, after
    );
}

// ═══════════════════════════════════════════════════════════════════════════════
//  Concurrent creation — no file conflicts
// ═══════════════════════════════════════════════════════════════════════════════

#[test]
fn churn_concurrent_runtimes_no_conflicts() {
    // Create multiple runtimes simultaneously (unique names via counter)
    let mut runtimes = Vec::new();
    for _ in 0..20 {
        let mut rt = HorusTestRuntime::new();
        rt.add_node(TestNodeConfig::bare(&unique_churn_name("concurrent")));
        runtimes.push(rt);
    }

    // All should be discoverable simultaneously
    let presences = horus_core::NodePresence::read_all();
    let concurrent_count = presences
        .iter()
        .filter(|p| p.name.starts_with("concurrent_"))
        .count();
    assert!(
        concurrent_count >= 15,
        "most concurrent nodes must be visible: found {} of 20",
        concurrent_count
    );

    // Drop all — cleanup should succeed
    drop(runtimes);
}

// ═══════════════════════════════════════════════════════════════════════════════
//  Rapid status polling — no degradation
// ═══════════════════════════════════════════════════════════════════════════════

#[tokio::test]
async fn churn_rapid_status_polling_no_degradation() {
    let app = builders::test_router();

    let mut stable_rt = HorusTestRuntime::new();
    for i in 0..20 {
        stable_rt.add_node(TestNodeConfig::bare(&format!("rapid_poll_{}", i)));
    }
    stable_rt.wait_ready(5_u64.secs());

    // Poll 50 times as fast as possible
    let start = Instant::now();
    for _ in 0..50 {
        let resp = app
            .clone()
            .oneshot(get_request("/api/status"))
            .await
            .unwrap();
        assert_eq!(resp.status(), axum::http::StatusCode::OK);
    }
    let elapsed = start.elapsed();

    assert!(
        elapsed.as_secs() < 30,
        "50 rapid status polls must complete in <30s, took {:?}",
        elapsed
    );
}
