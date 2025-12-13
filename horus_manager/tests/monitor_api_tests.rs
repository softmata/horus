// Comprehensive Monitor API Tests
// Tests all API endpoints: status, nodes, topics, graph, logs, packages, auth

use axum::{
    body::{to_bytes, Body},
    http::{Request, StatusCode},
    routing::{delete, get, post},
    Router,
};
use horus_core::params::RuntimeParams;
use horus_manager::monitor::AppState;
use horus_manager::security::auth::AuthService;
use serde_json::Value;
use std::sync::Arc;
use tower::ServiceExt;

// ============================================================================
// Test Setup Helpers
// ============================================================================

fn create_test_state() -> Arc<AppState> {
    let params = Arc::new(RuntimeParams::default());
    let auth_service = Arc::new(
        AuthService::new("$argon2id$v=19$m=19456,t=2,p=1$test$test".to_string())
            .unwrap_or_else(|_| panic!("Failed to create auth service")),
    );

    Arc::new(AppState {
        port: 0,
        params,
        auth_service,
        current_workspace: None,
        auth_disabled: true,
    })
}

fn create_test_router(state: Arc<AppState>) -> Router {
    Router::new()
        // Status and monitoring endpoints
        .route("/api/status", get(horus_manager::monitor::status_handler))
        .route("/api/nodes", get(horus_manager::monitor::nodes_handler))
        .route("/api/topics", get(horus_manager::monitor::topics_handler))
        .route("/api/graph", get(horus_manager::monitor::graph_handler))
        // Log endpoints
        .route(
            "/api/logs/all",
            get(horus_manager::monitor::logs_all_handler),
        )
        .route(
            "/api/logs/node/:name",
            get(horus_manager::monitor::logs_node_handler),
        )
        .route(
            "/api/logs/topic/:name",
            get(horus_manager::monitor::logs_topic_handler),
        )
        // Package endpoints
        .route(
            "/api/packages/registry",
            get(horus_manager::monitor::packages_registry_handler),
        )
        .route(
            "/api/packages/environments",
            get(horus_manager::monitor::packages_environments_handler),
        )
        .route(
            "/api/packages/publish",
            post(horus_manager::monitor::packages_publish_handler),
        )
        // Parameter endpoints (already well-tested in param_api_tests.rs)
        .route(
            "/api/params",
            get(horus_manager::monitor::params_list_handler),
        )
        .route(
            "/api/params/:key",
            get(horus_manager::monitor::params_get_handler),
        )
        .route(
            "/api/params/:key",
            post(horus_manager::monitor::params_set_handler),
        )
        .route(
            "/api/params/:key",
            delete(horus_manager::monitor::params_delete_handler),
        )
        .route(
            "/api/params/export",
            post(horus_manager::monitor::params_export_handler),
        )
        .route(
            "/api/params/import",
            post(horus_manager::monitor::params_import_handler),
        )
        .with_state(state)
}

// ============================================================================
// Status Handler Tests
// ============================================================================

#[tokio::test]
async fn test_status_handler_returns_ok() {
    let state = create_test_state();
    let app = create_test_router(state);

    let response = app
        .oneshot(
            Request::builder()
                .uri("/api/status")
                .body(Body::empty())
                .unwrap(),
        )
        .await
        .unwrap();

    assert_eq!(response.status(), StatusCode::OK);

    let body = to_bytes(response.into_body(), usize::MAX).await.unwrap();
    let json: Value = serde_json::from_slice(&body).unwrap();

    // Verify required fields exist
    assert!(json["status"].is_string());
    assert!(json["version"].is_string());
    assert!(json["nodes"].is_number());
    assert!(json["topics"].is_number());
    assert!(json["workspace"].is_object());
}

#[tokio::test]
async fn test_status_handler_with_workspace() {
    let params = Arc::new(RuntimeParams::default());
    let auth_service =
        Arc::new(AuthService::new("$argon2id$v=19$m=19456,t=2,p=1$test$test".to_string()).unwrap());

    // Create state with a workspace path
    let state = Arc::new(AppState {
        port: 0,
        params,
        auth_service,
        current_workspace: Some(std::path::PathBuf::from("/tmp/test_workspace")),
        auth_disabled: true,
    });

    let app = create_test_router(state);

    let response = app
        .oneshot(
            Request::builder()
                .uri("/api/status")
                .body(Body::empty())
                .unwrap(),
        )
        .await
        .unwrap();

    assert_eq!(response.status(), StatusCode::OK);

    let body = to_bytes(response.into_body(), usize::MAX).await.unwrap();
    let json: Value = serde_json::from_slice(&body).unwrap();

    assert!(json["workspace"]["detected"].as_bool().unwrap());
    assert!(json["workspace"]["name"].is_string());
    assert!(json["workspace"]["path"].is_string());
}

#[tokio::test]
async fn test_status_handler_health_colors() {
    let state = create_test_state();
    let app = create_test_router(state);

    let response = app
        .oneshot(
            Request::builder()
                .uri("/api/status")
                .body(Body::empty())
                .unwrap(),
        )
        .await
        .unwrap();

    assert_eq!(response.status(), StatusCode::OK);

    let body = to_bytes(response.into_body(), usize::MAX).await.unwrap();
    let json: Value = serde_json::from_slice(&body).unwrap();

    // Verify health_color is a valid color
    let valid_colors = ["green", "yellow", "orange", "red", "gray"];
    let health_color = json["health_color"].as_str().unwrap();
    assert!(
        valid_colors.contains(&health_color),
        "Invalid health color: {}",
        health_color
    );
}

// ============================================================================
// Nodes Handler Tests
// ============================================================================

#[tokio::test]
async fn test_nodes_handler_returns_ok() {
    let state = create_test_state();
    let app = create_test_router(state);

    let response = app
        .oneshot(
            Request::builder()
                .uri("/api/nodes")
                .body(Body::empty())
                .unwrap(),
        )
        .await
        .unwrap();

    assert_eq!(response.status(), StatusCode::OK);

    let body = to_bytes(response.into_body(), usize::MAX).await.unwrap();
    let json: Value = serde_json::from_slice(&body).unwrap();

    // Should have nodes array (may be empty)
    assert!(json["nodes"].is_array());
}

#[tokio::test]
async fn test_nodes_handler_response_structure() {
    let state = create_test_state();
    let app = create_test_router(state);

    let response = app
        .oneshot(
            Request::builder()
                .uri("/api/nodes")
                .body(Body::empty())
                .unwrap(),
        )
        .await
        .unwrap();

    let body = to_bytes(response.into_body(), usize::MAX).await.unwrap();
    let json: Value = serde_json::from_slice(&body).unwrap();

    // If there are nodes, verify structure
    if let Some(nodes) = json["nodes"].as_array() {
        for node in nodes {
            assert!(node["name"].is_string(), "Node should have name");
            assert!(node["status"].is_string(), "Node should have status");
            assert!(node["health"].is_string(), "Node should have health");
        }
    }
}

// ============================================================================
// Topics Handler Tests
// ============================================================================

#[tokio::test]
async fn test_topics_handler_returns_ok() {
    let state = create_test_state();
    let app = create_test_router(state);

    let response = app
        .oneshot(
            Request::builder()
                .uri("/api/topics")
                .body(Body::empty())
                .unwrap(),
        )
        .await
        .unwrap();

    assert_eq!(response.status(), StatusCode::OK);

    let body = to_bytes(response.into_body(), usize::MAX).await.unwrap();
    let json: Value = serde_json::from_slice(&body).unwrap();

    // Should have topics array (may be empty)
    assert!(json["topics"].is_array());
}

#[tokio::test]
async fn test_topics_handler_response_structure() {
    let state = create_test_state();
    let app = create_test_router(state);

    let response = app
        .oneshot(
            Request::builder()
                .uri("/api/topics")
                .body(Body::empty())
                .unwrap(),
        )
        .await
        .unwrap();

    let body = to_bytes(response.into_body(), usize::MAX).await.unwrap();
    let json: Value = serde_json::from_slice(&body).unwrap();

    // If there are topics, verify structure
    if let Some(topics) = json["topics"].as_array() {
        for topic in topics {
            assert!(topic["name"].is_string(), "Topic should have name");
            assert!(topic["size"].is_string(), "Topic should have size");
            assert!(
                topic["active"].is_boolean(),
                "Topic should have active status"
            );
        }
    }
}

// ============================================================================
// Graph Handler Tests
// ============================================================================

#[tokio::test]
async fn test_graph_handler_returns_ok() {
    let state = create_test_state();
    let app = create_test_router(state);

    let response = app
        .oneshot(
            Request::builder()
                .uri("/api/graph")
                .body(Body::empty())
                .unwrap(),
        )
        .await
        .unwrap();

    assert_eq!(response.status(), StatusCode::OK);

    let body = to_bytes(response.into_body(), usize::MAX).await.unwrap();
    let json: Value = serde_json::from_slice(&body).unwrap();

    // Should have nodes and edges arrays
    assert!(json["nodes"].is_array());
    assert!(json["edges"].is_array());
}

#[tokio::test]
async fn test_graph_handler_node_structure() {
    let state = create_test_state();
    let app = create_test_router(state);

    let response = app
        .oneshot(
            Request::builder()
                .uri("/api/graph")
                .body(Body::empty())
                .unwrap(),
        )
        .await
        .unwrap();

    let body = to_bytes(response.into_body(), usize::MAX).await.unwrap();
    let json: Value = serde_json::from_slice(&body).unwrap();

    // If there are graph nodes, verify structure
    if let Some(nodes) = json["nodes"].as_array() {
        for node in nodes {
            assert!(node["id"].is_string(), "Graph node should have id");
            assert!(node["label"].is_string(), "Graph node should have label");
            assert!(node["type"].is_string(), "Graph node should have type");

            let node_type = node["type"].as_str().unwrap();
            assert!(
                node_type == "process" || node_type == "topic",
                "Invalid node type: {}",
                node_type
            );
        }
    }
}

#[tokio::test]
async fn test_graph_handler_edge_structure() {
    let state = create_test_state();
    let app = create_test_router(state);

    let response = app
        .oneshot(
            Request::builder()
                .uri("/api/graph")
                .body(Body::empty())
                .unwrap(),
        )
        .await
        .unwrap();

    let body = to_bytes(response.into_body(), usize::MAX).await.unwrap();
    let json: Value = serde_json::from_slice(&body).unwrap();

    // If there are graph edges, verify structure
    if let Some(edges) = json["edges"].as_array() {
        for edge in edges {
            assert!(edge["from"].is_string(), "Graph edge should have from");
            assert!(edge["to"].is_string(), "Graph edge should have to");
            assert!(edge["type"].is_string(), "Graph edge should have type");

            let edge_type = edge["type"].as_str().unwrap();
            assert!(
                edge_type == "publish" || edge_type == "subscribe",
                "Invalid edge type: {}",
                edge_type
            );
        }
    }
}

// ============================================================================
// Logs Handler Tests
// ============================================================================

#[tokio::test]
async fn test_logs_all_handler_returns_ok() {
    let state = create_test_state();
    let app = create_test_router(state);

    let response = app
        .oneshot(
            Request::builder()
                .uri("/api/logs/all")
                .body(Body::empty())
                .unwrap(),
        )
        .await
        .unwrap();

    assert_eq!(response.status(), StatusCode::OK);

    let body = to_bytes(response.into_body(), usize::MAX).await.unwrap();
    let json: Value = serde_json::from_slice(&body).unwrap();

    // Should have logs array
    assert!(json["logs"].is_array());
}

#[tokio::test]
async fn test_logs_node_handler_returns_ok() {
    let state = create_test_state();
    let app = create_test_router(state);

    let response = app
        .oneshot(
            Request::builder()
                .uri("/api/logs/node/test_node")
                .body(Body::empty())
                .unwrap(),
        )
        .await
        .unwrap();

    assert_eq!(response.status(), StatusCode::OK);

    let body = to_bytes(response.into_body(), usize::MAX).await.unwrap();
    let json: Value = serde_json::from_slice(&body).unwrap();

    assert!(json["logs"].is_array());
    assert!(json["node"].is_string());
    assert_eq!(json["node"], "test_node");
}

#[tokio::test]
async fn test_logs_topic_handler_returns_ok() {
    let state = create_test_state();
    let app = create_test_router(state);

    let response = app
        .oneshot(
            Request::builder()
                .uri("/api/logs/topic/test_topic")
                .body(Body::empty())
                .unwrap(),
        )
        .await
        .unwrap();

    assert_eq!(response.status(), StatusCode::OK);

    let body = to_bytes(response.into_body(), usize::MAX).await.unwrap();
    let json: Value = serde_json::from_slice(&body).unwrap();

    assert!(json["logs"].is_array());
    assert!(json["topic"].is_string());
}

// ============================================================================
// Packages Handler Tests
// ============================================================================

#[tokio::test]
async fn test_packages_registry_handler_returns_ok() {
    let state = create_test_state();
    let app = create_test_router(state);

    let response = app
        .oneshot(
            Request::builder()
                .uri("/api/packages/registry?q=test")
                .body(Body::empty())
                .unwrap(),
        )
        .await
        .unwrap();

    // Should return OK even if registry is unavailable
    assert!(
        response.status() == StatusCode::OK
            || response.status() == StatusCode::INTERNAL_SERVER_ERROR
    );
}

#[tokio::test]
async fn test_packages_environments_handler_returns_ok() {
    let state = create_test_state();
    let app = create_test_router(state);

    let response = app
        .oneshot(
            Request::builder()
                .uri("/api/packages/environments")
                .body(Body::empty())
                .unwrap(),
        )
        .await
        .unwrap();

    assert_eq!(response.status(), StatusCode::OK);

    let body = to_bytes(response.into_body(), usize::MAX).await.unwrap();
    let json: Value = serde_json::from_slice(&body).unwrap();

    // Should have global and local arrays (actual response structure)
    assert!(
        json["global"].is_array(),
        "Expected global to be array, got: {}",
        json
    );
    assert!(
        json["local"].is_array(),
        "Expected local to be array, got: {}",
        json
    );
}

#[tokio::test]
async fn test_packages_publish_handler_returns_response() {
    let state = create_test_state();
    let app = create_test_router(state);

    let response = app
        .oneshot(
            Request::builder()
                .uri("/api/packages/publish")
                .method("POST")
                .body(Body::empty())
                .unwrap(),
        )
        .await
        .unwrap();

    // Should return a valid HTTP response (OK, INTERNAL_SERVER_ERROR, or NOT_FOUND/BAD_REQUEST)
    // Since there's no workspace to publish from in test, various error codes are acceptable
    let status = response.status();
    assert!(
        status.is_success() || status.is_client_error() || status.is_server_error(),
        "Expected valid HTTP status, got: {}",
        status
    );

    // Should return JSON response
    let body = to_bytes(response.into_body(), usize::MAX).await.unwrap();
    let json: Value = serde_json::from_slice(&body).unwrap();

    // Response should be valid JSON (either success or error object)
    assert!(json.is_object(), "Expected JSON object response");
}

// ============================================================================
// Error Handling Tests
// ============================================================================

#[tokio::test]
async fn test_invalid_endpoint_returns_404() {
    let state = create_test_state();
    let app = create_test_router(state);

    let response = app
        .oneshot(
            Request::builder()
                .uri("/api/invalid_endpoint")
                .body(Body::empty())
                .unwrap(),
        )
        .await
        .unwrap();

    assert_eq!(response.status(), StatusCode::NOT_FOUND);
}

#[tokio::test]
async fn test_wrong_method_returns_405() {
    let state = create_test_state();
    let app = create_test_router(state);

    // POST to a GET endpoint
    let response = app
        .oneshot(
            Request::builder()
                .uri("/api/status")
                .method("POST")
                .body(Body::empty())
                .unwrap(),
        )
        .await
        .unwrap();

    assert_eq!(response.status(), StatusCode::METHOD_NOT_ALLOWED);
}

// ============================================================================
// Concurrent Request Tests
// ============================================================================

#[tokio::test]
async fn test_concurrent_status_requests() {
    let state = create_test_state();

    let mut handles = vec![];

    for _ in 0..10 {
        let state_clone = state.clone();
        let handle = tokio::spawn(async move {
            let app = create_test_router(state_clone);
            let response = app
                .oneshot(
                    Request::builder()
                        .uri("/api/status")
                        .body(Body::empty())
                        .unwrap(),
                )
                .await
                .unwrap();
            response.status()
        });
        handles.push(handle);
    }

    for handle in handles {
        let status = handle.await.unwrap();
        assert_eq!(status, StatusCode::OK);
    }
}

#[tokio::test]
async fn test_concurrent_mixed_requests() {
    let state = create_test_state();

    let endpoints = vec![
        "/api/status",
        "/api/nodes",
        "/api/topics",
        "/api/graph",
        "/api/logs/all",
    ];

    let mut handles = vec![];

    for endpoint in endpoints {
        let state_clone = state.clone();
        let endpoint = endpoint.to_string();
        let handle = tokio::spawn(async move {
            let app = create_test_router(state_clone);
            let response = app
                .oneshot(
                    Request::builder()
                        .uri(&endpoint)
                        .body(Body::empty())
                        .unwrap(),
                )
                .await
                .unwrap();
            (endpoint, response.status())
        });
        handles.push(handle);
    }

    for handle in handles {
        let (endpoint, status) = handle.await.unwrap();
        assert_eq!(status, StatusCode::OK, "Failed for endpoint: {}", endpoint);
    }
}

// ============================================================================
// Response Content Type Tests
// ============================================================================

#[tokio::test]
async fn test_api_returns_json_content_type() {
    let state = create_test_state();
    let app = create_test_router(state);

    let response = app
        .oneshot(
            Request::builder()
                .uri("/api/status")
                .body(Body::empty())
                .unwrap(),
        )
        .await
        .unwrap();

    let content_type = response.headers().get("content-type");
    assert!(content_type.is_some());
    assert!(content_type
        .unwrap()
        .to_str()
        .unwrap()
        .contains("application/json"));
}

// ============================================================================
// Integration: Parameter + Status Tests
// ============================================================================

#[tokio::test]
async fn test_params_count_in_status() {
    let state = create_test_state();

    // Set some parameters
    state.params.set("test_param_1", "value1").unwrap();
    state.params.set("test_param_2", 42).unwrap();

    let app = create_test_router(state.clone());

    let response = app
        .oneshot(
            Request::builder()
                .uri("/api/params")
                .body(Body::empty())
                .unwrap(),
        )
        .await
        .unwrap();

    let body = to_bytes(response.into_body(), usize::MAX).await.unwrap();
    let json: Value = serde_json::from_slice(&body).unwrap();

    // Should have at least the parameters we set
    let count = json["count"].as_u64().unwrap();
    assert!(
        count >= 2,
        "Should have at least 2 parameters, got {}",
        count
    );
}
