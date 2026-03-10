//! UAT: WebSocket live dashboard E2E.
//!
//! Verifies that the /api/ws endpoint correctly upgrades to a WebSocket
//! connection, broadcasts structured JSON with nodes/topics/graph data,
//! and that the message format matches the documented contract.
//!
//! Tests bind to a random port, start a real axum server, and connect
//! using a WebSocket client.

mod harness;
mod monitor_tests;

use harness::HorusTestRuntime;
use monitor_tests::builders;

use futures_util::StreamExt;
use std::net::SocketAddr;
use tokio::net::TcpListener;

/// Start a test server on a random port and return the bound address.
async fn start_test_server() -> SocketAddr {
    let app = builders::test_router();
    let listener = TcpListener::bind("127.0.0.1:0").await.unwrap();
    let addr = listener.local_addr().unwrap();
    tokio::spawn(async move {
        axum::serve(listener, app).await.unwrap();
    });
    addr
}

/// Connect to the WebSocket endpoint and return the stream.
async fn ws_connect(
    addr: SocketAddr,
) -> tokio_tungstenite::WebSocketStream<tokio_tungstenite::MaybeTlsStream<tokio::net::TcpStream>> {
    let url = format!("ws://{}/api/ws", addr);
    let (ws_stream, _resp) = tokio_tungstenite::connect_async(&url)
        .await
        .expect("WebSocket connection must succeed");
    ws_stream
}

// ═══════════════════════════════════════════════════════════════════════════════
//  Basic connectivity
// ═══════════════════════════════════════════════════════════════════════════════

#[tokio::test]
async fn ws_connects_successfully() {
    let addr = start_test_server().await;
    let _ws = ws_connect(addr).await;
    // Connection succeeded — test passes
}

#[tokio::test]
async fn ws_receives_first_broadcast() {
    let addr = start_test_server().await;
    let mut ws = ws_connect(addr).await;

    // Wait for the first message (should arrive within broadcast interval)
    let msg = tokio::time::timeout(std::time::Duration::from_secs(2), ws.next())
        .await
        .expect("must receive message within timeout")
        .expect("stream must not end")
        .expect("message must not be an error");

    let text = msg.into_text().expect("message must be text");
    let json: serde_json::Value = serde_json::from_str(&text).expect("message must be valid JSON");

    assert_eq!(json["type"], "update", "message type must be 'update'");
    assert!(json["timestamp"].is_string(), "must have timestamp");
    assert!(json["data"].is_object(), "must have data object");
}

// ═══════════════════════════════════════════════════════════════════════════════
//  Broadcast message structure validation
// ═══════════════════════════════════════════════════════════════════════════════

#[tokio::test]
async fn ws_broadcast_has_correct_structure() {
    let addr = start_test_server().await;
    let mut ws = ws_connect(addr).await;

    let msg = tokio::time::timeout(std::time::Duration::from_secs(2), ws.next())
        .await
        .unwrap()
        .unwrap()
        .unwrap();

    let json: serde_json::Value = serde_json::from_str(&msg.into_text().unwrap()).unwrap();

    let data = &json["data"];
    assert!(data["nodes"].is_array(), "data.nodes must be an array");
    assert!(data["topics"].is_array(), "data.topics must be an array");
    assert!(data["graph"].is_object(), "data.graph must be an object");
    assert!(
        data["graph"]["nodes"].is_array(),
        "data.graph.nodes must be an array"
    );
    assert!(
        data["graph"]["edges"].is_array(),
        "data.graph.edges must be an array"
    );
}

#[tokio::test]
async fn ws_broadcast_node_fields() {
    let addr = start_test_server().await;

    // Create some SHM presence so there are nodes to broadcast
    let mut _rt = HorusTestRuntime::new();
    _rt.add_node(harness::TestNodeConfig::sensor(
        "ws_test_node",
        "test.topic",
        "SensorData",
    ));

    let mut ws = ws_connect(addr).await;

    // Read a few messages to let discovery pick up the node
    let mut found = false;
    for _ in 0..5 {
        let msg = tokio::time::timeout(std::time::Duration::from_secs(2), ws.next())
            .await
            .unwrap()
            .unwrap()
            .unwrap();
        let json: serde_json::Value = serde_json::from_str(&msg.into_text().unwrap()).unwrap();
        let nodes = json["data"]["nodes"].as_array().unwrap();
        if let Some(node) = nodes
            .iter()
            .find(|n| n["name"].as_str() == Some("ws_test_node"))
        {
            // Validate node fields
            assert!(node["name"].is_string(), "node must have name");
            assert!(node["status"].is_string(), "node must have status");
            assert!(node["health"].is_string(), "node must have health");
            assert!(
                node["health_color"].is_string(),
                "node must have health_color"
            );
            assert!(node["cpu"].is_string(), "node must have cpu");
            assert!(node["memory"].is_string(), "node must have memory");
            assert!(
                node["scheduler_name"].is_string(),
                "node must have scheduler_name"
            );
            // PID must NOT be present
            assert!(
                node.get("pid").is_none() || node["pid"].is_null(),
                "node must NOT expose PID"
            );
            assert!(
                node.get("process_id").is_none() || node["process_id"].is_null(),
                "node must NOT expose process_id"
            );
            found = true;
            break;
        }
    }
    assert!(
        found,
        "ws_test_node must appear in WS broadcast within 5 cycles"
    );
}

#[tokio::test]
async fn ws_broadcast_topic_fields() {
    let addr = start_test_server().await;

    let mut _rt = HorusTestRuntime::new();
    _rt.add_node(harness::TestNodeConfig::sensor(
        "ws_topic_pub",
        "ws_test_topic",
        "SensorData",
    ));

    let mut ws = ws_connect(addr).await;

    let mut found = false;
    for _ in 0..5 {
        let msg = tokio::time::timeout(std::time::Duration::from_secs(2), ws.next())
            .await
            .unwrap()
            .unwrap()
            .unwrap();
        let json: serde_json::Value = serde_json::from_str(&msg.into_text().unwrap()).unwrap();
        let topics = json["data"]["topics"].as_array().unwrap();
        if let Some(topic) = topics.iter().find(|t| {
            t["name"]
                .as_str()
                .map_or(false, |n| n.contains("ws_test_topic"))
        }) {
            assert!(topic["name"].is_string(), "topic must have name");
            assert!(topic["size"].is_string(), "topic must have size");
            // active can be bool
            assert!(
                topic["active"].is_boolean(),
                "topic must have boolean active field"
            );
            assert!(
                topic["processes"].is_number(),
                "topic must have numeric processes field"
            );
            found = true;
            break;
        }
    }
    assert!(
        found,
        "ws_test_topic must appear in WS broadcast within 5 cycles"
    );
}

// ═══════════════════════════════════════════════════════════════════════════════
//  No PID in any broadcast field
// ═══════════════════════════════════════════════════════════════════════════════

#[tokio::test]
async fn ws_broadcast_no_pid_anywhere() {
    let addr = start_test_server().await;
    let mut ws = ws_connect(addr).await;

    let msg = tokio::time::timeout(std::time::Duration::from_secs(2), ws.next())
        .await
        .unwrap()
        .unwrap()
        .unwrap();

    let text = msg.into_text().unwrap();
    let json: serde_json::Value = serde_json::from_str(&text).unwrap();

    // Check nodes
    if let Some(nodes) = json["data"]["nodes"].as_array() {
        for node in nodes {
            assert!(
                node.get("pid").is_none() || node["pid"].is_null(),
                "node broadcast must not contain 'pid' field"
            );
            assert!(
                node.get("process_id").is_none() || node["process_id"].is_null(),
                "node broadcast must not contain 'process_id' field"
            );
        }
    }

    // Check graph nodes
    if let Some(graph_nodes) = json["data"]["graph"]["nodes"].as_array() {
        for gn in graph_nodes {
            assert!(
                gn.get("pid").is_none() || gn["pid"].is_null(),
                "graph node must not contain 'pid' field"
            );
        }
    }
}

// ═══════════════════════════════════════════════════════════════════════════════
//  Graph data in broadcast
// ═══════════════════════════════════════════════════════════════════════════════

#[tokio::test]
async fn ws_broadcast_graph_node_fields() {
    let addr = start_test_server().await;
    let mut ws = ws_connect(addr).await;

    // Read messages until we get one with graph nodes
    for _ in 0..5 {
        let msg = tokio::time::timeout(std::time::Duration::from_secs(2), ws.next())
            .await
            .unwrap()
            .unwrap()
            .unwrap();
        let json: serde_json::Value = serde_json::from_str(&msg.into_text().unwrap()).unwrap();
        let graph_nodes = json["data"]["graph"]["nodes"].as_array().unwrap();
        if !graph_nodes.is_empty() {
            let gn = &graph_nodes[0];
            assert!(gn["id"].is_string(), "graph node must have id");
            assert!(gn["label"].is_string(), "graph node must have label");
            assert!(gn["type"].is_string(), "graph node must have type");
            let node_type = gn["type"].as_str().unwrap();
            assert!(
                node_type == "process" || node_type == "topic",
                "graph node type must be 'process' or 'topic', got '{}'",
                node_type
            );
            assert!(
                gn["active"].is_boolean(),
                "graph node must have boolean active"
            );
            return;
        }
    }
    // If no graph nodes found in 5 cycles, that's ok — empty system has no graph nodes
}

#[tokio::test]
async fn ws_broadcast_graph_edge_fields() {
    let addr = start_test_server().await;
    let mut ws = ws_connect(addr).await;

    for _ in 0..5 {
        let msg = tokio::time::timeout(std::time::Duration::from_secs(2), ws.next())
            .await
            .unwrap()
            .unwrap()
            .unwrap();
        let json: serde_json::Value = serde_json::from_str(&msg.into_text().unwrap()).unwrap();
        let edges = json["data"]["graph"]["edges"].as_array().unwrap();
        if !edges.is_empty() {
            let edge = &edges[0];
            assert!(edge["from"].is_string(), "edge must have from");
            assert!(edge["to"].is_string(), "edge must have to");
            assert!(edge["type"].is_string(), "edge must have type");
            let edge_type = edge["type"].as_str().unwrap();
            assert!(
                edge_type == "publish" || edge_type == "subscribe",
                "edge type must be 'publish' or 'subscribe', got '{}'",
                edge_type
            );
            assert!(edge["active"].is_boolean(), "edge must have boolean active");
            return;
        }
    }
    // If no edges found in 5 cycles, that's ok — empty system has no edges
}

// ═══════════════════════════════════════════════════════════════════════════════
//  Multiple messages — broadcast interval
// ═══════════════════════════════════════════════════════════════════════════════

#[tokio::test]
async fn ws_receives_multiple_broadcasts() {
    let addr = start_test_server().await;
    let mut ws = ws_connect(addr).await;

    // Should receive at least 3 messages within a few seconds
    let mut count = 0;
    let deadline = tokio::time::Instant::now() + std::time::Duration::from_secs(3);
    while tokio::time::Instant::now() < deadline && count < 3 {
        match tokio::time::timeout(std::time::Duration::from_secs(2), ws.next()).await {
            Ok(Some(Ok(msg))) => {
                let text = msg.into_text().unwrap();
                let json: serde_json::Value = serde_json::from_str(&text).unwrap();
                assert_eq!(json["type"], "update");
                count += 1;
            }
            _ => break,
        }
    }
    assert!(
        count >= 3,
        "should receive at least 3 broadcasts within 3 seconds (250ms interval), got {}",
        count
    );
}

#[tokio::test]
async fn ws_broadcast_timestamps_increase() {
    let addr = start_test_server().await;
    let mut ws = ws_connect(addr).await;

    let mut timestamps = Vec::new();
    for _ in 0..3 {
        let msg = tokio::time::timeout(std::time::Duration::from_secs(2), ws.next())
            .await
            .unwrap()
            .unwrap()
            .unwrap();
        let json: serde_json::Value = serde_json::from_str(&msg.into_text().unwrap()).unwrap();
        timestamps.push(json["timestamp"].as_str().unwrap().to_string());
    }

    // Timestamps should be monotonically increasing
    for i in 1..timestamps.len() {
        assert!(
            timestamps[i] > timestamps[i - 1],
            "timestamps must increase: {} should be after {}",
            timestamps[i],
            timestamps[i - 1]
        );
    }
}

// ═══════════════════════════════════════════════════════════════════════════════
//  Non-GET request to /api/ws should fail
// ═══════════════════════════════════════════════════════════════════════════════

#[tokio::test]
async fn ws_post_request_rejected() {
    use tower::ServiceExt;
    let app = builders::test_router();
    let resp = app
        .oneshot(
            axum::http::Request::builder()
                .method("POST")
                .uri("/api/ws")
                .body(axum::body::Body::empty())
                .unwrap(),
        )
        .await
        .unwrap();
    // POST to a WebSocket endpoint should not succeed
    assert_ne!(
        resp.status(),
        axum::http::StatusCode::SWITCHING_PROTOCOLS,
        "POST to /api/ws must not trigger WebSocket upgrade"
    );
}
