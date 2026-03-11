//! Integration test entry point for monitor test utilities.
//!
//! This file declares the `monitor_tests` module (which lives in
//! `tests/monitor_tests/`) and contains smoke tests that verify the test
//! utilities themselves compile and produce valid data.
#![cfg(feature = "monitor")]

mod monitor_tests;

use tower::ServiceExt; // for .oneshot()

// ─── Smoke tests for builders ───────────────────────────────────────────────

#[test]
fn test_app_state_creates_valid_state() {
    let state = monitor_tests::builders::test_app_state();
    assert_eq!(state.port, 0);
    assert!(state.auth_disabled);
    assert!(!state.trust_proxy);
    assert!(state.current_workspace.is_none());
}

#[test]
fn test_app_state_with_auth_has_auth_enabled() {
    let state = monitor_tests::builders::test_app_state_with_auth();
    assert!(!state.auth_disabled);
}

#[test]
fn test_app_state_with_workspace_sets_path() {
    let ws = std::path::PathBuf::from("/tmp/test_workspace");
    let state = monitor_tests::builders::test_app_state_with_workspace(ws.clone());
    assert_eq!(state.current_workspace.as_ref().unwrap(), &ws);
    assert!(state.auth_disabled);
}

#[test]
fn test_router_builds_without_panic() {
    let _router = monitor_tests::builders::test_router();
}

// ─── Smoke tests for helpers ────────────────────────────────────────────────

#[test]
fn make_test_node_creates_valid_node() {
    use horus_core::core::HealthStatus;

    let node = monitor_tests::helpers::make_test_node(
        "lidar_driver",
        HealthStatus::Healthy,
        25.5,
        50 * 1024 * 1024,
    );
    assert_eq!(node.name, "lidar_driver");
    assert_eq!(node.health, HealthStatus::Healthy);
    assert!((node.cpu_usage - 25.5).abs() < f32::EPSILON);
    assert_eq!(node.memory_usage, 50 * 1024 * 1024);
    assert_eq!(node.status, "Running");
}

#[test]
fn make_test_node_full_sets_all_fields() {
    use horus_core::core::HealthStatus;

    let node = monitor_tests::helpers::make_test_node_full(
        "camera_node",
        HealthStatus::Warning,
        60.0,
        100 * 1024 * 1024,
        5000,
        3,
        vec![("camera_rgb", "Image"), ("camera_depth", "DepthImage")],
        vec![("config", "Config")],
    );
    assert_eq!(node.name, "camera_node");
    assert_eq!(node.tick_count, 5000);
    assert_eq!(node.error_count, 3);
    assert_eq!(node.publishers.len(), 2);
    assert_eq!(node.subscribers.len(), 1);
    assert_eq!(node.publishers[0].topic, "camera_rgb");
    assert_eq!(node.subscribers[0].topic, "config");
}

#[test]
fn make_test_topic_creates_valid_topic() {
    let topic = monitor_tests::helpers::make_test_topic("horus_camera_rgb", 65536, true, 3);
    assert_eq!(topic.topic_name, "horus_camera_rgb");
    assert_eq!(topic.size_bytes, 65536);
    assert!(topic.active);
    assert_eq!(topic.accessing_processes.len(), 3);
    assert_eq!(topic.status, horus_manager::discovery::TopicStatus::Active);
}

#[test]
fn make_test_topic_inactive() {
    let topic = monitor_tests::helpers::make_test_topic("horus_old_topic", 1024, false, 0);
    assert!(!topic.active);
    assert_eq!(topic.status, horus_manager::discovery::TopicStatus::Stale);
    assert_eq!(topic.accessing_processes.len(), 0);
}

#[test]
fn make_test_blackbox_event_tick() {
    let record = monitor_tests::helpers::make_test_blackbox_event("tick", "sensor_node", 42);
    assert_eq!(record.tick, 42);
    assert!(matches!(
        record.event,
        horus_core::scheduling::BlackBoxEvent::NodeTick { .. }
    ));
}

#[test]
fn make_test_blackbox_event_error() {
    let record = monitor_tests::helpers::make_test_blackbox_event("error", "motor_node", 7);
    assert_eq!(record.tick, 7);
    assert!(matches!(
        record.event,
        horus_core::scheduling::BlackBoxEvent::NodeError { .. }
    ));
}

#[test]
fn make_test_blackbox_event_all_types() {
    // Verify all event types can be created without panicking
    let types = [
        "tick",
        "error",
        "deadline_miss",
        "budget_violation",
        "scheduler_start",
        "scheduler_stop",
        "node_added",
        "emergency_stop",
        "custom",
    ];
    for (i, t) in types.iter().enumerate() {
        let record = monitor_tests::helpers::make_test_blackbox_event(t, "node", i as u64);
        assert_eq!(record.tick, i as u64);
    }
}

#[test]
fn make_test_blackbox_batch_creates_correct_count() {
    let batch = monitor_tests::helpers::make_test_blackbox_batch("sensor", 15);
    assert_eq!(batch.len(), 15);
    // Tick 0 should be "error" (0 % 3 == 0), tick 1 should be "tick"
    assert!(matches!(
        batch[0].event,
        horus_core::scheduling::BlackBoxEvent::NodeError { .. }
    ));
    assert!(matches!(
        batch[1].event,
        horus_core::scheduling::BlackBoxEvent::NodeTick { .. }
    ));
}

// ─── Request builder helper smoke tests ─────────────────────────────────────

#[test]
fn get_request_builds_correctly() {
    let req = monitor_tests::helpers::get_request("/api/status");
    assert_eq!(req.method(), "GET");
    assert_eq!(req.uri(), "/api/status");
}

#[test]
fn get_request_authed_includes_cookie() {
    let req = monitor_tests::helpers::get_request_authed("/api/nodes", "my_token_123");
    assert_eq!(req.method(), "GET");
    assert!(req
        .headers()
        .get("Cookie")
        .unwrap()
        .to_str()
        .unwrap()
        .contains("session_token=my_token_123"));
}

#[test]
fn post_json_request_sets_content_type() {
    let req = monitor_tests::helpers::post_json_request("/api/login", r#"{"password":"test"}"#);
    assert_eq!(req.method(), "POST");
    assert!(req
        .headers()
        .get("Content-Type")
        .unwrap()
        .to_str()
        .unwrap()
        .contains("application/json"));
}

#[test]
fn post_json_request_authed_includes_csrf() {
    let req = monitor_tests::helpers::post_json_request_authed(
        "/api/params/test",
        r#"{"value":42}"#,
        "session_tok",
        "csrf_tok",
    );
    assert_eq!(req.method(), "POST");
    assert!(req.headers().get("X-CSRF-Token").is_some());
    assert!(req.headers().get("Cookie").is_some());
}

#[test]
fn delete_request_authed_includes_headers() {
    let req = monitor_tests::helpers::delete_request_authed("/api/recordings/old", "sess", "csrf");
    assert_eq!(req.method(), "DELETE");
    assert!(req.headers().get("X-CSRF-Token").is_some());
    assert!(req.headers().get("Cookie").is_some());
}

// ─── Router integration smoke tests ─────────────────────────────────────────

#[tokio::test]
async fn router_status_endpoint_returns_200() {
    let app = monitor_tests::builders::test_router();
    let req = monitor_tests::helpers::get_request("/api/status");
    let resp = app.oneshot(req).await.unwrap();
    let json = monitor_tests::helpers::assert_json_ok(resp).await;

    // Validate expected JSON structure
    monitor_tests::helpers::assert_json_has_key(&json, "status");
    monitor_tests::helpers::assert_json_has_key(&json, "health");
    monitor_tests::helpers::assert_json_has_key(&json, "version");
    monitor_tests::helpers::assert_json_has_key(&json, "nodes");
    monitor_tests::helpers::assert_json_has_key(&json, "topics");
    monitor_tests::helpers::assert_json_has_key(&json, "workspace");
}

#[tokio::test]
async fn router_nodes_endpoint_returns_200() {
    let app = monitor_tests::builders::test_router();
    let req = monitor_tests::helpers::get_request("/api/nodes");
    let resp = app.oneshot(req).await.unwrap();
    let json = monitor_tests::helpers::assert_json_ok(resp).await;

    // nodes must be an array (may be empty if no real nodes running)
    assert!(json["nodes"].is_array(), "nodes must be an array");
}

#[tokio::test]
async fn router_topics_endpoint_returns_200() {
    let app = monitor_tests::builders::test_router();
    let req = monitor_tests::helpers::get_request("/api/topics");
    let resp = app.oneshot(req).await.unwrap();
    let json = monitor_tests::helpers::assert_json_ok(resp).await;

    assert!(json["topics"].is_array(), "topics must be an array");
}

#[tokio::test]
async fn router_graph_endpoint_returns_200() {
    let app = monitor_tests::builders::test_router();
    let req = monitor_tests::helpers::get_request("/api/graph");
    let resp = app.oneshot(req).await.unwrap();
    let json = monitor_tests::helpers::assert_json_ok(resp).await;

    assert!(json["nodes"].is_array());
    assert!(json["edges"].is_array());
}

#[tokio::test]
async fn router_network_endpoint_returns_200() {
    let app = monitor_tests::builders::test_router();
    let req = monitor_tests::helpers::get_request("/api/network");
    let resp = app.oneshot(req).await.unwrap();
    let json = monitor_tests::helpers::assert_json_ok(resp).await;

    monitor_tests::helpers::assert_json_has_key(&json, "total_nodes");
    monitor_tests::helpers::assert_json_has_key(&json, "node_statuses");
}

#[tokio::test]
async fn router_logs_all_returns_200() {
    let app = monitor_tests::builders::test_router();
    let req = monitor_tests::helpers::get_request("/api/logs/all");
    let resp = app.oneshot(req).await.unwrap();
    let json = monitor_tests::helpers::assert_json_ok(resp).await;

    assert!(json["logs"].is_array());
}

#[tokio::test]
async fn router_logs_node_returns_200() {
    let app = monitor_tests::builders::test_router();
    let req = monitor_tests::helpers::get_request("/api/logs/node/test_node");
    let resp = app.oneshot(req).await.unwrap();
    let json = monitor_tests::helpers::assert_json_ok(resp).await;

    assert_eq!(json["node"], "test_node");
    assert!(json["logs"].is_array());
}

#[tokio::test]
async fn router_debug_sessions_returns_200() {
    let app = monitor_tests::builders::test_router();
    let req = monitor_tests::helpers::get_request("/api/debug/sessions");
    let resp = app.oneshot(req).await.unwrap();
    let json = monitor_tests::helpers::assert_json_ok(resp).await;

    assert!(json["sessions"].is_array());
    monitor_tests::helpers::assert_json_has_key(&json, "count");
}

#[tokio::test]
async fn router_blackbox_returns_200() {
    let app = monitor_tests::builders::test_router();
    let req = monitor_tests::helpers::get_request("/api/blackbox");
    let resp = app.oneshot(req).await.unwrap();
    // May return 200 or 500 depending on blackbox dir existence; check it does not panic
    let status = resp.status();
    assert!(
        status == axum::http::StatusCode::OK
            || status == axum::http::StatusCode::INTERNAL_SERVER_ERROR,
        "blackbox endpoint should return 200 or 500, got {}",
        status
    );
}

#[tokio::test]
async fn router_recordings_returns_200() {
    let app = monitor_tests::builders::test_router();
    let req = monitor_tests::helpers::get_request("/api/recordings");
    let resp = app.oneshot(req).await.unwrap();
    let json = monitor_tests::helpers::assert_json_ok(resp).await;

    assert!(json["recordings"].is_array());
}

#[tokio::test]
async fn router_params_list_returns_200() {
    let app = monitor_tests::builders::test_router();
    let req = monitor_tests::helpers::get_request("/api/params");
    let resp = app.oneshot(req).await.unwrap();
    let json = monitor_tests::helpers::assert_json_ok(resp).await;

    assert!(json["params"].is_array() || json["params"].is_object());
}

#[tokio::test]
async fn router_nonexistent_route_returns_404() {
    let app = monitor_tests::builders::test_router();
    let req = monitor_tests::helpers::get_request("/api/does_not_exist");
    let resp = app.oneshot(req).await.unwrap();
    assert_eq!(resp.status(), axum::http::StatusCode::NOT_FOUND);
}

// ─── Auth integration tests ─────────────────────────────────────────────────

#[tokio::test]
async fn auth_router_login_and_access() {
    let state = monitor_tests::builders::test_app_state_with_auth();
    let app = monitor_tests::builders::test_router_with_state(state);

    // Without auth middleware applied (since test_router_with_state does not
    // add it), the routes are open.  This test verifies the login handler
    // works correctly in isolation.
    let req = monitor_tests::helpers::post_json_request(
        "/api/login",
        &format!(
            r#"{{"password":"{}"}}"#,
            monitor_tests::builders::TEST_PASSWORD
        ),
    );
    let resp = app.clone().oneshot(req).await.unwrap();
    let json = monitor_tests::helpers::assert_json_ok(resp).await;

    assert_eq!(json["success"], true);
    assert!(json["session_token"].is_string());
    assert!(json["csrf_token"].is_string());
}

#[tokio::test]
async fn auth_router_wrong_password_returns_401() {
    let state = monitor_tests::builders::test_app_state_with_auth();
    let app = monitor_tests::builders::test_router_with_state(state);

    let req = monitor_tests::helpers::post_json_request(
        "/api/login",
        r#"{"password":"wrong_password_here"}"#,
    );
    let resp = app.oneshot(req).await.unwrap();
    let json =
        monitor_tests::helpers::assert_json_error(resp, axum::http::StatusCode::UNAUTHORIZED).await;

    assert_eq!(json["success"], false);
    assert!(json["session_token"].is_null());
}

#[tokio::test]
async fn login_helper_returns_valid_tokens() {
    let state = monitor_tests::builders::test_app_state_with_auth();
    let app = monitor_tests::builders::test_router_with_state(state);

    let (session_token, csrf_token) = monitor_tests::builders::login(&app).await;
    assert!(!session_token.is_empty());
    assert!(!csrf_token.is_empty());
    // Tokens should be base64url-encoded (43 chars for 32 bytes)
    assert_eq!(session_token.len(), 43);
    assert_eq!(csrf_token.len(), 43);
}

// ─── SHM fixture smoke tests ───────────────────────────────────────────────

#[test]
fn shm_guard_creates_and_cleans_up() {
    let mut guard = monitor_tests::shm_fixtures::TestShmGuard::new();
    let path = guard.create_presence("guard_test", std::process::id());
    assert!(path.exists(), "presence file should exist after creation");

    // Verify it is valid JSON
    let content = std::fs::read_to_string(&path).unwrap();
    let json: serde_json::Value = serde_json::from_str(&content).unwrap();
    assert!(json["name"].as_str().unwrap().starts_with("__test_"));
    assert_eq!(json["pid"], std::process::id());

    drop(guard);
    assert!(
        !path.exists(),
        "presence file should be removed after guard drop"
    );
}

#[test]
fn shm_guard_creates_topic_file() {
    let mut guard = monitor_tests::shm_fixtures::TestShmGuard::new();
    let path = guard.create_topic("topic_test", 4096);
    assert!(path.exists(), "topic file should exist after creation");

    let metadata = std::fs::metadata(&path).unwrap();
    assert_eq!(metadata.len(), 4096);

    drop(guard);
    assert!(
        !path.exists(),
        "topic file should be removed after guard drop"
    );
}

#[test]
fn shm_guard_creates_presence_with_topics() {
    let mut guard = monitor_tests::shm_fixtures::TestShmGuard::new();
    let path = guard.create_presence_with_topics(
        "pub_sub_test",
        std::process::id(),
        &[("camera_rgb", "Image"), ("lidar", "PointCloud")],
        &[("command", "Twist")],
    );
    assert!(path.exists());

    let content = std::fs::read_to_string(&path).unwrap();
    let json: serde_json::Value = serde_json::from_str(&content).unwrap();
    assert_eq!(json["publishers"].as_array().unwrap().len(), 2);
    assert_eq!(json["subscribers"].as_array().unwrap().len(), 1);
    assert_eq!(json["scheduler"], "test_scheduler");

    drop(guard);
}

#[test]
fn cleanup_test_shm_removes_test_files() {
    // Create some test files manually
    let path1 = monitor_tests::shm_fixtures::create_test_presence("cleanup_a", std::process::id());
    let path2 = monitor_tests::shm_fixtures::create_test_shm_topic("cleanup_b", 1024);
    assert!(path1.exists());
    assert!(path2.exists());

    monitor_tests::shm_fixtures::cleanup_test_shm();

    assert!(!path1.exists(), "cleanup should remove test presence files");
    assert!(!path2.exists(), "cleanup should remove test topic files");
}

// ─── JSON assertion helper tests ────────────────────────────────────────────

#[test]
fn assert_json_has_key_passes_for_present_key() {
    let json = serde_json::json!({"foo": "bar", "count": 42});
    monitor_tests::helpers::assert_json_has_key(&json, "foo");
    monitor_tests::helpers::assert_json_has_key(&json, "count");
}

#[test]
#[should_panic(expected = "expected JSON to have non-null key")]
fn assert_json_has_key_panics_for_missing_key() {
    let json = serde_json::json!({"foo": "bar"});
    monitor_tests::helpers::assert_json_has_key(&json, "missing");
}

#[test]
fn assert_json_array_len_passes_for_correct_length() {
    let json = serde_json::json!({"items": [1, 2, 3]});
    monitor_tests::helpers::assert_json_array_len(&json, "items", 3);
}

#[test]
#[should_panic(expected = "array length mismatch")]
fn assert_json_array_len_panics_for_wrong_length() {
    let json = serde_json::json!({"items": [1, 2]});
    monitor_tests::helpers::assert_json_array_len(&json, "items", 5);
}
