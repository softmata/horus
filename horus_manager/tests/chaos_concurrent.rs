//! Chaos test: concurrent API abuse.
//!
//! Hits all monitor endpoints concurrently with high parallelism.
//! Verifies no deadlocks, data corruption, or crashes.

mod harness;
mod monitor_tests;

use harness::{HorusTestRuntime, TestNodeConfig};
use monitor_tests::builders;
use monitor_tests::helpers::{get_request, post_json_request};

use std::sync::Arc;
use tower::ServiceExt;

// ═══════════════════════════════════════════════════════════════════════════════
//  Concurrent GET /api/nodes
// ═══════════════════════════════════════════════════════════════════════════════

#[tokio::test]
async fn concurrent_50_get_nodes() {
    let mut rt = HorusTestRuntime::new();
    for i in 0..10 {
        rt.add_node(TestNodeConfig::bare(&format!("conc_node_{}", i)));
    }
    rt.wait_ready(std::time::Duration::from_secs(5));

    let app = builders::test_router();

    let mut handles = Vec::new();
    for _ in 0..50 {
        let app = app.clone();
        handles.push(tokio::spawn(async move {
            let resp = app.oneshot(get_request("/api/nodes")).await.unwrap();
            assert_eq!(resp.status(), axum::http::StatusCode::OK);
            let body = axum::body::to_bytes(resp.into_body(), 10 * 1024 * 1024)
                .await
                .unwrap();
            let json: serde_json::Value = serde_json::from_slice(&body)
                .expect("concurrent /api/nodes must return valid JSON");
            assert!(json["nodes"].is_array());
        }));
    }

    let results = futures_util::future::join_all(handles).await;
    for r in results {
        r.expect("concurrent GET /api/nodes must not panic");
    }
}

// ═══════════════════════════════════════════════════════════════════════════════
//  Concurrent GET /api/topics
// ═══════════════════════════════════════════════════════════════════════════════

#[tokio::test]
async fn concurrent_50_get_topics() {
    let mut rt = HorusTestRuntime::new();
    for i in 0..5 {
        rt.add_topic(&format!("conc_topic_{}", i), 2048);
    }

    let app = builders::test_router();

    let mut handles = Vec::new();
    for _ in 0..50 {
        let app = app.clone();
        handles.push(tokio::spawn(async move {
            let resp = app.oneshot(get_request("/api/topics")).await.unwrap();
            assert_eq!(resp.status(), axum::http::StatusCode::OK);
            let body = axum::body::to_bytes(resp.into_body(), 10 * 1024 * 1024)
                .await
                .unwrap();
            let json: serde_json::Value = serde_json::from_slice(&body)
                .expect("concurrent /api/topics must return valid JSON");
            assert!(json["topics"].is_array());
        }));
    }

    let results = futures_util::future::join_all(handles).await;
    for r in results {
        r.expect("concurrent GET /api/topics must not panic");
    }
}

// ═══════════════════════════════════════════════════════════════════════════════
//  Concurrent POST /api/params with different keys
// ═══════════════════════════════════════════════════════════════════════════════

#[tokio::test]
async fn concurrent_20_set_different_params() {
    let app = builders::test_router();

    let mut handles = Vec::new();
    for i in 0..20 {
        let app = app.clone();
        handles.push(tokio::spawn(async move {
            let key = format!("conc_param_{}", i);
            let body = format!(r#"{{"value": {}}}"#, i);
            let resp = app
                .oneshot(post_json_request(&format!("/api/params/{}", key), &body))
                .await
                .unwrap();
            let status = resp.status();
            assert!(
                status == axum::http::StatusCode::OK || status == axum::http::StatusCode::CREATED,
                "setting param {} must succeed, got {}",
                key,
                status
            );
        }));
    }

    let results = futures_util::future::join_all(handles).await;
    for r in results {
        r.expect("concurrent param set must not panic");
    }
}

// ═══════════════════════════════════════════════════════════════════════════════
//  Concurrent POST /api/params with SAME key (race condition)
// ═══════════════════════════════════════════════════════════════════════════════

#[tokio::test]
async fn concurrent_10_set_same_param_race() {
    let app = builders::test_router();

    // First set the param so it exists
    let resp = app
        .clone()
        .oneshot(post_json_request("/api/params/race_key", r#"{"value": 0}"#))
        .await
        .unwrap();
    assert_eq!(resp.status(), axum::http::StatusCode::OK);

    // Race: 10 concurrent writes to same key (no version check)
    let mut handles = Vec::new();
    for i in 0..10 {
        let app = app.clone();
        handles.push(tokio::spawn(async move {
            let body = format!(r#"{{"value": {}}}"#, i);
            let resp = app
                .oneshot(post_json_request("/api/params/race_key", &body))
                .await
                .unwrap();
            let status = resp.status();
            // Some may get 409 Conflict due to version check, that's acceptable
            assert!(
                status == axum::http::StatusCode::OK || status == axum::http::StatusCode::CONFLICT,
                "race write must return OK or Conflict, got {}",
                status
            );
        }));
    }

    let results = futures_util::future::join_all(handles).await;
    for r in results {
        r.expect("concurrent same-key param race must not panic");
    }

    // Verify final value is consistent (some valid value, not corrupt)
    let resp = app
        .oneshot(get_request("/api/params/race_key"))
        .await
        .unwrap();
    assert_eq!(resp.status(), axum::http::StatusCode::OK);
    let body = axum::body::to_bytes(resp.into_body(), 1024 * 1024)
        .await
        .unwrap();
    let json: serde_json::Value = serde_json::from_slice(&body).unwrap();
    assert!(
        json["value"].is_number(),
        "final race_key value must be a valid number"
    );
}

// ═══════════════════════════════════════════════════════════════════════════════
//  Concurrent WebSocket connections
// ═══════════════════════════════════════════════════════════════════════════════

#[tokio::test]
async fn concurrent_10_websocket_connections() {
    let app = builders::test_router();
    let listener = tokio::net::TcpListener::bind("127.0.0.1:0").await.unwrap();
    let addr = listener.local_addr().unwrap();
    tokio::spawn(async move {
        axum::serve(listener, app).await.unwrap();
    });

    use futures_util::StreamExt;

    let mut handles = Vec::new();
    for i in 0..10 {
        let addr = addr;
        handles.push(tokio::spawn(async move {
            let url = format!("ws://{}/api/ws", addr);
            let (mut ws, _) = tokio_tungstenite::connect_async(&url)
                .await
                .unwrap_or_else(|e| panic!("WS client {} connect failed: {}", i, e));

            // Each client should receive at least one broadcast
            let msg = tokio::time::timeout(std::time::Duration::from_secs(5), ws.next())
                .await
                .unwrap_or_else(|_| panic!("WS client {} timed out", i))
                .unwrap()
                .unwrap();

            let text = msg.into_text().unwrap();
            let json: serde_json::Value =
                serde_json::from_str(&text).expect("WS broadcast must be valid JSON");
            assert_eq!(json["type"], "update", "client {}: type must be update", i);
        }));
    }

    let results = futures_util::future::join_all(handles).await;
    for (i, r) in results.iter().enumerate() {
        r.as_ref()
            .unwrap_or_else(|e| panic!("WS client {} panicked: {:?}", i, e));
    }
}

// ═══════════════════════════════════════════════════════════════════════════════
//  Mixed concurrent requests — all endpoint types simultaneously
// ═══════════════════════════════════════════════════════════════════════════════

#[tokio::test]
async fn concurrent_mixed_endpoints() {
    let mut rt = HorusTestRuntime::new();
    for i in 0..5 {
        rt.add_node(TestNodeConfig::bare(&format!("mixed_node_{}", i)));
        rt.add_topic(&format!("mixed_topic_{}", i), 1024);
    }
    rt.wait_ready(std::time::Duration::from_secs(5));

    let app = builders::test_router();
    let mut handles = Vec::new();

    // 20 GET /api/nodes
    for _ in 0..20 {
        let app = app.clone();
        handles.push(tokio::spawn(async move {
            let resp = app.oneshot(get_request("/api/nodes")).await.unwrap();
            assert_eq!(resp.status(), axum::http::StatusCode::OK);
        }));
    }

    // 20 GET /api/topics
    for _ in 0..20 {
        let app = app.clone();
        handles.push(tokio::spawn(async move {
            let resp = app.oneshot(get_request("/api/topics")).await.unwrap();
            assert_eq!(resp.status(), axum::http::StatusCode::OK);
        }));
    }

    // 20 GET /api/status
    for _ in 0..20 {
        let app = app.clone();
        handles.push(tokio::spawn(async move {
            let resp = app.oneshot(get_request("/api/status")).await.unwrap();
            assert_eq!(resp.status(), axum::http::StatusCode::OK);
        }));
    }

    // 20 GET /api/graph
    for _ in 0..20 {
        let app = app.clone();
        handles.push(tokio::spawn(async move {
            let resp = app.oneshot(get_request("/api/graph")).await.unwrap();
            assert_eq!(resp.status(), axum::http::StatusCode::OK);
        }));
    }

    // 10 POST params
    for i in 0..10 {
        let app = app.clone();
        handles.push(tokio::spawn(async move {
            let key = format!("mixed_param_{}", i);
            let body = format!(r#"{{"value": {}}}"#, i);
            let resp = app
                .oneshot(post_json_request(&format!("/api/params/{}", key), &body))
                .await
                .unwrap();
            assert_eq!(resp.status(), axum::http::StatusCode::OK);
        }));
    }

    // 10 GET /api/logs/all
    for _ in 0..10 {
        let app = app.clone();
        handles.push(tokio::spawn(async move {
            let resp = app.oneshot(get_request("/api/logs/all")).await.unwrap();
            assert_eq!(resp.status(), axum::http::StatusCode::OK);
        }));
    }

    // Total: 100 concurrent requests
    assert_eq!(handles.len(), 100, "must have 100 concurrent requests");

    let timeout = tokio::time::timeout(
        std::time::Duration::from_secs(30),
        futures_util::future::join_all(handles),
    )
    .await
    .expect("all 100 concurrent requests must complete within 30s");

    for (i, r) in timeout.iter().enumerate() {
        r.as_ref()
            .unwrap_or_else(|e| panic!("request {} panicked: {:?}", i, e));
    }
}

// ═══════════════════════════════════════════════════════════════════════════════
//  Concurrent GET and DELETE params — no corruption
// ═══════════════════════════════════════════════════════════════════════════════

#[tokio::test]
async fn concurrent_get_delete_params_no_corruption() {
    let app = builders::test_router();

    // Create 10 params
    for i in 0..10 {
        let resp = app
            .clone()
            .oneshot(post_json_request(
                &format!("/api/params/conc_del_{}", i),
                &format!(r#"{{"value": {}}}"#, i),
            ))
            .await
            .unwrap();
        assert_eq!(resp.status(), axum::http::StatusCode::OK);
    }

    // Concurrently GET and DELETE
    let mut handles = Vec::new();

    // 10 GETs
    for i in 0..10 {
        let app = app.clone();
        handles.push(tokio::spawn(async move {
            let resp = app
                .oneshot(get_request(&format!("/api/params/conc_del_{}", i)))
                .await
                .unwrap();
            // May be 200 or 404 (if deleted first)
            let status = resp.status();
            assert!(
                status == axum::http::StatusCode::OK || status == axum::http::StatusCode::NOT_FOUND,
                "concurrent get must return 200 or 404, got {}",
                status
            );
        }));
    }

    // 10 DELETEs
    for i in 0..10 {
        let app = app.clone();
        handles.push(tokio::spawn(async move {
            let resp = app
                .oneshot(
                    axum::http::Request::builder()
                        .method("DELETE")
                        .uri(&format!("/api/params/conc_del_{}", i))
                        .body(axum::body::Body::empty())
                        .unwrap(),
                )
                .await
                .unwrap();
            let status = resp.status();
            assert!(
                status == axum::http::StatusCode::OK || status == axum::http::StatusCode::NOT_FOUND,
                "concurrent delete must return 200 or 404, got {}",
                status
            );
        }));
    }

    let results = futures_util::future::join_all(handles).await;
    for r in results {
        r.expect("concurrent GET/DELETE must not panic");
    }

    // Params list must still work
    let resp = app.oneshot(get_request("/api/params")).await.unwrap();
    assert_eq!(resp.status(), axum::http::StatusCode::OK);
}
