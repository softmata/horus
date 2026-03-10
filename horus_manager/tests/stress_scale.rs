//! Stress test: 1000+ nodes, high topic count.
//!
//! Verifies that discovery and API endpoints scale to extreme node/topic
//! counts without panics, OOM, or unacceptable latency.
//!
//! Run with: cargo test -p horus_manager --test stress_scale -- --test-threads=1

mod harness;
mod monitor_tests;

use harness::{HorusTestRuntime, TestNodeConfig};
use monitor_tests::builders;
use monitor_tests::helpers::get_request;

use std::time::Instant;
use tower::ServiceExt;

/// Create a runtime with `n` nodes and `t` topics.
fn build_large_runtime(n: usize, t: usize) -> HorusTestRuntime {
    let mut rt = HorusTestRuntime::new();

    for i in 0..n {
        let name = format!("stress_node_{:04}", i);
        let mut config = TestNodeConfig::bare(&name);
        // Give every 10th node a publisher topic
        if i % 10 == 0 {
            let topic = format!("stress_topic_{:04}", i / 10);
            config = config.with_publisher(&topic, "StressData");
        }
        // Give every 15th node a subscriber topic
        if i % 15 == 0 {
            let topic = format!("stress_topic_{:04}", (i / 15) % t);
            config = config.with_subscriber(&topic, "StressData");
        }
        rt.add_node(config);
    }

    for i in 0..t {
        let name = format!("stress_topic_{:04}", i);
        rt.add_topic(&name, 4096);
    }

    rt
}

// ═══════════════════════════════════════════════════════════════════════════════
//  Scale creation — verify 1000 nodes can be created without panic
// ═══════════════════════════════════════════════════════════════════════════════

#[test]
fn stress_create_1000_nodes_500_topics() {
    let start = Instant::now();
    let rt = build_large_runtime(1000, 500);
    let elapsed = start.elapsed();

    assert_eq!(rt.node_names().len(), 1000, "must have 1000 nodes");
    assert!(
        elapsed.as_secs() < 30,
        "creating 1000 nodes + 500 topics must take <30s, took {:?}",
        elapsed
    );

    // Cleanup happens on drop
    drop(rt);
}

// ═══════════════════════════════════════════════════════════════════════════════
//  Discovery at scale — verify all nodes discoverable
// ═══════════════════════════════════════════════════════════════════════════════

#[test]
fn stress_discover_1000_nodes() {
    let rt = build_large_runtime(1000, 100);

    let start = Instant::now();
    let found = rt.wait_ready(std::time::Duration::from_secs(10));
    let elapsed = start.elapsed();

    assert!(
        found,
        "all 1000 nodes must be discoverable within 10s (took {:?})",
        elapsed
    );
}

// ═══════════════════════════════════════════════════════════════════════════════
//  API response time at scale
// ═══════════════════════════════════════════════════════════════════════════════

#[tokio::test]
async fn stress_api_status_response_time() {
    let _rt = build_large_runtime(500, 200);
    // Wait for discovery
    _rt.wait_ready(std::time::Duration::from_secs(10));

    let app = builders::test_router();

    let start = Instant::now();
    let resp = app.oneshot(get_request("/api/status")).await.unwrap();
    let elapsed = start.elapsed();

    assert_eq!(resp.status(), axum::http::StatusCode::OK);
    assert!(
        elapsed.as_millis() < 2000,
        "/api/status must respond in <2s with 500 nodes, took {:?}",
        elapsed
    );
}

#[tokio::test]
async fn stress_api_nodes_response_time() {
    let _rt = build_large_runtime(500, 200);
    _rt.wait_ready(std::time::Duration::from_secs(10));

    let app = builders::test_router();

    let start = Instant::now();
    let resp = app.oneshot(get_request("/api/nodes")).await.unwrap();
    let elapsed = start.elapsed();

    assert_eq!(resp.status(), axum::http::StatusCode::OK);

    let body = axum::body::to_bytes(resp.into_body(), 10 * 1024 * 1024)
        .await
        .unwrap();
    let json: serde_json::Value = serde_json::from_slice(&body).unwrap();
    let nodes = json["nodes"].as_array().unwrap();

    assert!(
        nodes.len() >= 100,
        "should discover many nodes, got {}",
        nodes.len()
    );
    assert!(
        elapsed.as_millis() < 5000,
        "/api/nodes must respond in <5s with 500 nodes, took {:?}",
        elapsed
    );
}

#[tokio::test]
async fn stress_api_topics_response_time() {
    let _rt = build_large_runtime(200, 500);
    _rt.wait_ready(std::time::Duration::from_secs(10));

    let app = builders::test_router();

    let start = Instant::now();
    let resp = app.oneshot(get_request("/api/topics")).await.unwrap();
    let elapsed = start.elapsed();

    assert_eq!(resp.status(), axum::http::StatusCode::OK);
    assert!(
        elapsed.as_millis() < 5000,
        "/api/topics must respond in <5s with 500 topics, took {:?}",
        elapsed
    );
}

#[tokio::test]
async fn stress_api_graph_response_time() {
    let _rt = build_large_runtime(200, 100);
    _rt.wait_ready(std::time::Duration::from_secs(10));

    let app = builders::test_router();

    let start = Instant::now();
    let resp = app.oneshot(get_request("/api/graph")).await.unwrap();
    let elapsed = start.elapsed();

    assert_eq!(resp.status(), axum::http::StatusCode::OK);
    assert!(
        elapsed.as_millis() < 5000,
        "/api/graph must respond in <5s with 200 nodes, took {:?}",
        elapsed
    );
}

// ═══════════════════════════════════════════════════════════════════════════════
//  Repeated polling — cache effectiveness
// ═══════════════════════════════════════════════════════════════════════════════

#[tokio::test]
async fn stress_repeated_polling_stable() {
    let _rt = build_large_runtime(200, 100);
    _rt.wait_ready(std::time::Duration::from_secs(10));

    let app = builders::test_router();

    // Poll /api/status 20 times rapidly
    let start = Instant::now();
    for _ in 0..20 {
        let resp = app
            .clone()
            .oneshot(get_request("/api/status"))
            .await
            .unwrap();
        assert_eq!(resp.status(), axum::http::StatusCode::OK);
    }
    let elapsed = start.elapsed();

    // 20 requests should complete quickly with caching
    assert!(
        elapsed.as_secs() < 30,
        "20 rapid /api/status polls must complete in <30s, took {:?}",
        elapsed
    );
}

#[tokio::test]
async fn stress_repeated_nodes_polling_stable() {
    let _rt = build_large_runtime(200, 50);
    _rt.wait_ready(std::time::Duration::from_secs(10));

    let app = builders::test_router();

    // Poll /api/nodes 10 times
    for _ in 0..10 {
        let resp = app
            .clone()
            .oneshot(get_request("/api/nodes"))
            .await
            .unwrap();
        assert_eq!(resp.status(), axum::http::StatusCode::OK);
    }
}

// ═══════════════════════════════════════════════════════════════════════════════
//  Large JSON response — no truncation or OOM
// ═══════════════════════════════════════════════════════════════════════════════

#[tokio::test]
async fn stress_large_nodes_response_parseable() {
    let _rt = build_large_runtime(500, 100);
    _rt.wait_ready(std::time::Duration::from_secs(10));

    let app = builders::test_router();
    let resp = app.oneshot(get_request("/api/nodes")).await.unwrap();
    assert_eq!(resp.status(), axum::http::StatusCode::OK);

    // Read the full body (may be large)
    let body = axum::body::to_bytes(resp.into_body(), 50 * 1024 * 1024)
        .await
        .unwrap();

    // Must be valid JSON
    let json: serde_json::Value =
        serde_json::from_slice(&body).expect("large nodes response must be valid JSON");

    assert!(json["nodes"].is_array(), "must have nodes array");
}

// ═══════════════════════════════════════════════════════════════════════════════
//  WebSocket at scale
// ═══════════════════════════════════════════════════════════════════════════════

#[tokio::test]
async fn stress_websocket_broadcast_at_scale() {
    let _rt = build_large_runtime(200, 100);
    _rt.wait_ready(std::time::Duration::from_secs(10));

    let app = builders::test_router();
    let listener = tokio::net::TcpListener::bind("127.0.0.1:0").await.unwrap();
    let addr = listener.local_addr().unwrap();
    tokio::spawn(async move {
        axum::serve(listener, app).await.unwrap();
    });

    let url = format!("ws://{}/api/ws", addr);
    let (mut ws, _resp) = tokio_tungstenite::connect_async(&url)
        .await
        .expect("WS connect must succeed");

    use futures_util::StreamExt;

    // Receive 3 broadcasts and verify they're valid JSON
    for i in 0..3 {
        let msg = tokio::time::timeout(std::time::Duration::from_secs(5), ws.next())
            .await
            .unwrap_or_else(|_| panic!("broadcast {} timed out at scale", i))
            .unwrap()
            .unwrap();

        let text = msg.into_text().unwrap();
        let json: serde_json::Value =
            serde_json::from_str(&text).expect("WS broadcast at scale must be valid JSON");

        assert_eq!(json["type"], "update");
        assert!(json["data"]["nodes"].is_array());
        assert!(json["data"]["topics"].is_array());
    }
}

// ═══════════════════════════════════════════════════════════════════════════════
//  Cleanup at scale — no leftover files
// ═══════════════════════════════════════════════════════════════════════════════

#[test]
fn stress_cleanup_1000_nodes() {
    let rt = build_large_runtime(1000, 500);
    let presence_paths: Vec<_> = rt.presence_paths().to_vec();
    let topic_paths: Vec<_> = rt.topic_paths().to_vec();

    // All files should exist
    for p in &presence_paths[..5] {
        assert!(
            p.exists(),
            "presence file must exist before cleanup: {:?}",
            p
        );
    }
    for p in &topic_paths[..5] {
        assert!(p.exists(), "topic file must exist before cleanup: {:?}", p);
    }

    // Drop triggers cleanup
    drop(rt);

    // Verify cleanup
    for p in &presence_paths {
        assert!(!p.exists(), "presence file must be cleaned up: {:?}", p);
    }
    for p in &topic_paths {
        assert!(!p.exists(), "topic file must be cleaned up: {:?}", p);
    }
}
