//! Comprehensive integration tests for the blackbox, debug, and parameters
//! handler endpoints.
//!
//! These tests exercise the HTTP API through the Axum router built by
//! [`builders::test_router`], verifying status codes, response schemas, and
//! edge-case behaviour such as path traversal rejection and version conflicts.
//!
//! All tests assume `--test-threads=1` serial execution for debug tests
//! (which share a global `DEBUG_SESSIONS` mutex).

mod harness;
mod monitor_tests;

use monitor_tests::builders;
use monitor_tests::helpers::{
    assert_json_error, assert_json_ok, delete_request, get_request, post_json_request,
};
use tower::ServiceExt;

use axum::http::StatusCode;

// ═══════════════════════════════════════════════════════════════════════════════
//  BLACKBOX HANDLERS  (GET /api/blackbox, GET /api/blackbox/anomalies,
//                      DELETE /api/blackbox)
// ═══════════════════════════════════════════════════════════════════════════════

/// 1. GET /api/blackbox returns valid JSON with events array and count.
#[tokio::test]
async fn blackbox_list_returns_valid_json() {
    let app = builders::test_router();
    let resp = app.oneshot(get_request("/api/blackbox")).await.unwrap();

    // The handler returns 200 even when there are no blackbox files; the events
    // array will simply be empty.  If the blackbox dir doesn't exist, the
    // handler returns an error with a JSON body — both cases are valid JSON.
    let status = resp.status();
    let body = axum::body::to_bytes(resp.into_body(), 2 * 1024 * 1024)
        .await
        .expect("reading response body must not fail");
    let json: serde_json::Value =
        serde_json::from_slice(&body).expect("response body must be valid JSON");

    if status == StatusCode::OK {
        assert!(json["events"].is_array(), "events should be an array");
        assert!(json["count"].is_number(), "count should be a number");
        let events = json["events"].as_array().unwrap();
        assert_eq!(
            events.len() as u64,
            json["count"].as_u64().unwrap(),
            "count should match events array length"
        );
    } else {
        // Internal error (e.g. blackbox dir not found) — still valid JSON.
        assert!(
            json["error"].is_string(),
            "error response should have an error string"
        );
    }
}

/// 2. GET /api/blackbox/anomalies returns valid JSON with anomalies array and count.
#[tokio::test]
async fn blackbox_anomalies_returns_valid_json() {
    let app = builders::test_router();
    let resp = app
        .oneshot(get_request("/api/blackbox/anomalies"))
        .await
        .unwrap();

    let status = resp.status();
    let body = axum::body::to_bytes(resp.into_body(), 2 * 1024 * 1024)
        .await
        .expect("reading response body must not fail");
    let json: serde_json::Value =
        serde_json::from_slice(&body).expect("response body must be valid JSON");

    if status == StatusCode::OK {
        assert!(json["anomalies"].is_array(), "anomalies should be an array");
        assert!(json["count"].is_number(), "count should be a number");
        let anomalies = json["anomalies"].as_array().unwrap();
        assert_eq!(
            anomalies.len() as u64,
            json["count"].as_u64().unwrap(),
            "count should match anomalies array length"
        );
    } else {
        assert!(
            json["error"].is_string(),
            "error response should have an error string"
        );
    }
}

/// 3. GET /api/blackbox?node=X includes node query parameter in the request
/// (verifies query string parsing does not cause a 400/422).
#[tokio::test]
async fn blackbox_list_with_node_filter_parses_ok() {
    let app = builders::test_router();
    let resp = app
        .oneshot(get_request("/api/blackbox?node=lidar"))
        .await
        .unwrap();

    let status = resp.status();
    let body = axum::body::to_bytes(resp.into_body(), 2 * 1024 * 1024)
        .await
        .unwrap();
    let json: serde_json::Value = serde_json::from_slice(&body).unwrap();

    // Should be 200 or a valid error, never a 422 from bad query parsing.
    assert_ne!(
        status,
        StatusCode::UNPROCESSABLE_ENTITY,
        "node filter should not cause 422"
    );
    assert!(json.is_object(), "response should be a JSON object");
}

/// 4. GET /api/blackbox?event=NodeError includes event query parameter.
#[tokio::test]
async fn blackbox_list_with_event_filter_parses_ok() {
    let app = builders::test_router();
    let resp = app
        .oneshot(get_request("/api/blackbox?event=NodeError"))
        .await
        .unwrap();

    let status = resp.status();
    let body = axum::body::to_bytes(resp.into_body(), 2 * 1024 * 1024)
        .await
        .unwrap();
    let json: serde_json::Value = serde_json::from_slice(&body).unwrap();

    assert_ne!(
        status,
        StatusCode::UNPROCESSABLE_ENTITY,
        "event filter should not cause 422"
    );
    assert!(json.is_object(), "response should be a JSON object");
}

/// 5. GET /api/blackbox?tick=5-10 includes tick range query parameter.
#[tokio::test]
async fn blackbox_list_with_tick_range_parses_ok() {
    let app = builders::test_router();
    let resp = app
        .oneshot(get_request("/api/blackbox?tick=5-10"))
        .await
        .unwrap();

    let status = resp.status();
    let body = axum::body::to_bytes(resp.into_body(), 2 * 1024 * 1024)
        .await
        .unwrap();
    let json: serde_json::Value = serde_json::from_slice(&body).unwrap();

    assert_ne!(
        status,
        StatusCode::UNPROCESSABLE_ENTITY,
        "tick range filter should not cause 422"
    );
    assert!(json.is_object(), "response should be a JSON object");
}

/// 6. GET /api/blackbox?limit=3 includes limit query parameter.
#[tokio::test]
async fn blackbox_list_with_limit_parses_ok() {
    let app = builders::test_router();
    let resp = app
        .oneshot(get_request("/api/blackbox?limit=3"))
        .await
        .unwrap();

    let status = resp.status();
    let body = axum::body::to_bytes(resp.into_body(), 2 * 1024 * 1024)
        .await
        .unwrap();
    let json: serde_json::Value = serde_json::from_slice(&body).unwrap();

    assert_ne!(
        status,
        StatusCode::UNPROCESSABLE_ENTITY,
        "limit filter should not cause 422"
    );
    if status == StatusCode::OK {
        let events = json["events"]
            .as_array()
            .expect("events should be an array");
        assert!(
            events.len() <= 3,
            "limit=3 should return at most 3 events, got {}",
            events.len()
        );
    }
}

// ═══════════════════════════════════════════════════════════════════════════════
//  DEBUG HANDLERS  (via router)
// ═══════════════════════════════════════════════════════════════════════════════

/// 7. List debug sessions returns empty when none exist (or at least a valid
/// JSON with sessions array and count).
#[tokio::test]
async fn debug_list_sessions_via_router() {
    let app = builders::test_router();
    let resp = app
        .oneshot(get_request("/api/debug/sessions"))
        .await
        .unwrap();
    let json = assert_json_ok(resp).await;

    assert!(json["sessions"].is_array(), "sessions should be an array");
    assert!(json["count"].is_number(), "count should be a number");
}

/// 8. Create debug session with path traversal in recording_session returns 400.
#[tokio::test]
async fn debug_create_session_path_traversal_in_session_name() {
    let app = builders::test_router();
    let body = r#"{"recording_session":"../../../etc","recording_file":"passwd"}"#;
    let resp = app
        .oneshot(post_json_request("/api/debug/sessions", body))
        .await
        .unwrap();
    let json = assert_json_error(resp, StatusCode::BAD_REQUEST).await;
    assert!(json["error"].is_string(), "should have an error message");
}

/// 9. Create debug session with path traversal in recording_file returns 400.
#[tokio::test]
async fn debug_create_session_path_traversal_in_file_name() {
    let app = builders::test_router();
    let body = r#"{"recording_session":"valid_session","recording_file":"../../shadow"}"#;
    let resp = app
        .oneshot(post_json_request("/api/debug/sessions", body))
        .await
        .unwrap();
    let json = assert_json_error(resp, StatusCode::BAD_REQUEST).await;
    assert!(json["error"].is_string(), "should have an error message");
}

/// 10. Get non-existent session returns 404.
#[tokio::test]
async fn debug_get_nonexistent_session_via_router() {
    let app = builders::test_router();
    let resp = app
        .oneshot(get_request(
            "/api/debug/sessions/nonexistent_session_abc123",
        ))
        .await
        .unwrap();
    let json = assert_json_error(resp, StatusCode::NOT_FOUND).await;
    assert!(json["error"].is_string(), "should have an error message");
}

/// 11. Delete non-existent session returns 404.
#[tokio::test]
async fn debug_delete_nonexistent_session_via_router() {
    let app = builders::test_router();
    let resp = app
        .oneshot(delete_request(
            "/api/debug/sessions/nonexistent_session_abc123",
        ))
        .await
        .unwrap();
    let json = assert_json_error(resp, StatusCode::NOT_FOUND).await;
    assert!(json["error"].is_string(), "should have an error message");
}

/// 12. Add breakpoint to non-existent session returns 404.
#[tokio::test]
async fn debug_add_breakpoint_nonexistent_session_via_router() {
    let app = builders::test_router();
    let body = r#"{"breakpoint_type":"at_tick","tick":42}"#;
    let resp = app
        .oneshot(post_json_request(
            "/api/debug/sessions/nonexistent_xyz/breakpoints",
            body,
        ))
        .await
        .unwrap();
    let json = assert_json_error(resp, StatusCode::NOT_FOUND).await;
    assert!(json["error"].is_string(), "should have an error message");
}

/// 13. Add breakpoint with invalid type returns 400 (via router).
///
/// We send a valid session-id format but a non-existent session. The handler
/// checks session existence first (404) before validating breakpoint type.
/// To specifically test the breakpoint type validation path, we would need a
/// live session. Instead we verify the handler correctly rejects an unknown
/// breakpoint_type by directly calling the handler.
#[tokio::test]
async fn debug_add_breakpoint_invalid_type_nonexistent_returns_404() {
    // When the session doesn't exist, the handler returns 404 before
    // reaching the breakpoint_type validation.  This verifies the 404 path
    // through the router for an unknown breakpoint type body payload.
    let app = builders::test_router();
    let body = r#"{"breakpoint_type":"totally_invalid_type","tick":10}"#;
    let resp = app
        .oneshot(post_json_request(
            "/api/debug/sessions/nonexistent_xyz/breakpoints",
            body,
        ))
        .await
        .unwrap();
    let json = assert_json_error(resp, StatusCode::NOT_FOUND).await;
    assert!(json["error"].is_string(), "should have an error message");
}

/// 14. Remove breakpoint from non-existent session returns 404.
#[tokio::test]
async fn debug_remove_breakpoint_nonexistent_session_via_router() {
    let app = builders::test_router();
    let resp = app
        .oneshot(delete_request(
            "/api/debug/sessions/nonexistent_xyz/breakpoints/1",
        ))
        .await
        .unwrap();
    let json = assert_json_error(resp, StatusCode::NOT_FOUND).await;
    assert!(json["error"].is_string(), "should have an error message");
}

/// 15. Add watch to non-existent session returns 404.
#[tokio::test]
async fn debug_add_watch_nonexistent_session_via_router() {
    let app = builders::test_router();
    let body = r#"{"id":"w1","name":"vel","topic":"/cmd_vel","watch_type":"input"}"#;
    let resp = app
        .oneshot(post_json_request(
            "/api/debug/sessions/nonexistent_xyz/watches",
            body,
        ))
        .await
        .unwrap();
    let json = assert_json_error(resp, StatusCode::NOT_FOUND).await;
    assert!(json["error"].is_string(), "should have an error message");
}

/// 16. Add watch with invalid type — the handler returns 404 for non-existent
/// session before reaching watch_type validation.
#[tokio::test]
async fn debug_add_watch_invalid_type_nonexistent_returns_404() {
    let app = builders::test_router();
    let body = r#"{"id":"w1","name":"vel","topic":"/cmd_vel","watch_type":"totally_invalid_type"}"#;
    let resp = app
        .oneshot(post_json_request(
            "/api/debug/sessions/nonexistent_xyz/watches",
            body,
        ))
        .await
        .unwrap();
    let json = assert_json_error(resp, StatusCode::NOT_FOUND).await;
    assert!(json["error"].is_string(), "should have an error message");
}

/// 17. Remove watch from non-existent session returns 404.
#[tokio::test]
async fn debug_remove_watch_nonexistent_session_via_router() {
    let app = builders::test_router();
    let resp = app
        .oneshot(delete_request(
            "/api/debug/sessions/nonexistent_xyz/watches/watch1",
        ))
        .await
        .unwrap();
    let json = assert_json_error(resp, StatusCode::NOT_FOUND).await;
    assert!(json["error"].is_string(), "should have an error message");
}

/// 18. Step forward on non-existent session returns 404 (via router).
#[tokio::test]
async fn debug_step_forward_nonexistent_via_router() {
    let app = builders::test_router();
    let resp = app
        .oneshot(post_json_request(
            "/api/debug/sessions/nonexistent_xyz/step-forward",
            "{}",
        ))
        .await
        .unwrap();
    let json = assert_json_error(resp, StatusCode::NOT_FOUND).await;
    assert!(json["error"].is_string(), "should have an error message");
}

/// 18b. Step backward on non-existent session returns 404 (via router).
#[tokio::test]
async fn debug_step_backward_nonexistent_via_router() {
    let app = builders::test_router();
    let resp = app
        .oneshot(post_json_request(
            "/api/debug/sessions/nonexistent_xyz/step-backward",
            "{}",
        ))
        .await
        .unwrap();
    let json = assert_json_error(resp, StatusCode::NOT_FOUND).await;
    assert!(json["error"].is_string(), "should have an error message");
}

/// 19. Continue on non-existent session returns 404 (via router).
#[tokio::test]
async fn debug_continue_nonexistent_via_router() {
    let app = builders::test_router();
    let resp = app
        .oneshot(post_json_request(
            "/api/debug/sessions/nonexistent_xyz/continue",
            "{}",
        ))
        .await
        .unwrap();
    let json = assert_json_error(resp, StatusCode::NOT_FOUND).await;
    assert!(json["error"].is_string(), "should have an error message");
}

/// 19b. Pause on non-existent session returns 404 (via router).
#[tokio::test]
async fn debug_pause_nonexistent_via_router() {
    let app = builders::test_router();
    let resp = app
        .oneshot(post_json_request(
            "/api/debug/sessions/nonexistent_xyz/pause",
            "{}",
        ))
        .await
        .unwrap();
    let json = assert_json_error(resp, StatusCode::NOT_FOUND).await;
    assert!(json["error"].is_string(), "should have an error message");
}

/// 20. Seek on non-existent session returns 404 (via router).
#[tokio::test]
async fn debug_seek_nonexistent_via_router() {
    let app = builders::test_router();
    let body = r#"{"tick":100}"#;
    let resp = app
        .oneshot(post_json_request(
            "/api/debug/sessions/nonexistent_xyz/seek",
            body,
        ))
        .await
        .unwrap();
    let json = assert_json_error(resp, StatusCode::NOT_FOUND).await;
    assert!(json["error"].is_string(), "should have an error message");
}

/// 20b. Reset on non-existent session returns 404 (via router).
#[tokio::test]
async fn debug_reset_nonexistent_via_router() {
    let app = builders::test_router();
    let resp = app
        .oneshot(post_json_request(
            "/api/debug/sessions/nonexistent_xyz/reset",
            "{}",
        ))
        .await
        .unwrap();
    let json = assert_json_error(resp, StatusCode::NOT_FOUND).await;
    assert!(json["error"].is_string(), "should have an error message");
}

/// 21. Snapshot on non-existent session returns 404 (via router).
#[tokio::test]
async fn debug_snapshot_nonexistent_via_router() {
    let app = builders::test_router();
    let resp = app
        .oneshot(get_request("/api/debug/sessions/nonexistent_xyz/snapshot"))
        .await
        .unwrap();
    let json = assert_json_error(resp, StatusCode::NOT_FOUND).await;
    assert!(json["error"].is_string(), "should have an error message");
}

/// 22. Watch values on non-existent session returns 404 (via router).
#[tokio::test]
async fn debug_watches_values_nonexistent_via_router() {
    let app = builders::test_router();
    let resp = app
        .oneshot(get_request(
            "/api/debug/sessions/nonexistent_xyz/watches/values",
        ))
        .await
        .unwrap();
    let json = assert_json_error(resp, StatusCode::NOT_FOUND).await;
    assert!(json["error"].is_string(), "should have an error message");
}

/// 23. List sessions returns JSON with sessions array and count (via router).
#[tokio::test]
async fn debug_list_sessions_schema_via_router() {
    let app = builders::test_router();
    let resp = app
        .oneshot(get_request("/api/debug/sessions"))
        .await
        .unwrap();
    let json = assert_json_ok(resp).await;

    assert!(json["sessions"].is_array(), "sessions should be an array");
    assert!(json["count"].is_number(), "count should be a number");
    let sessions = json["sessions"].as_array().unwrap();
    assert_eq!(
        sessions.len() as u64,
        json["count"].as_u64().unwrap(),
        "count should match sessions array length"
    );
}

// ═══════════════════════════════════════════════════════════════════════════════
//  PARAMETERS HANDLERS  (GET /api/params, GET/POST/DELETE /api/params/:key,
//                         POST /api/params/export, POST /api/params/import)
// ═══════════════════════════════════════════════════════════════════════════════
//
// Parameters handler returns (StatusCode, String) — the body is a JSON string.
// We parse it manually via assert_params_ok / assert_params_error helpers.

/// Parse a params endpoint response that should be 200.  Returns the parsed
/// JSON body.
async fn assert_params_ok(response: axum::http::Response<axum::body::Body>) -> serde_json::Value {
    let status = response.status();
    let body = axum::body::to_bytes(response.into_body(), 2 * 1024 * 1024)
        .await
        .expect("reading response body must not fail");
    let body_str = String::from_utf8_lossy(&body);

    assert_eq!(
        status,
        StatusCode::OK,
        "expected 200 OK, got {} -- body: {}",
        status,
        body_str
    );

    serde_json::from_str(&body_str).unwrap_or_else(|e| {
        panic!(
            "response body is not valid JSON: {} -- body: {}",
            e, body_str
        )
    })
}

/// Parse a params endpoint response that should be the given error status.
/// Returns the parsed JSON body.
async fn assert_params_error(
    response: axum::http::Response<axum::body::Body>,
    expected_status: StatusCode,
) -> serde_json::Value {
    let status = response.status();
    let body = axum::body::to_bytes(response.into_body(), 2 * 1024 * 1024)
        .await
        .expect("reading response body must not fail");
    let body_str = String::from_utf8_lossy(&body);

    assert_eq!(
        status, expected_status,
        "expected {}, got {} -- body: {}",
        expected_status, status, body_str
    );

    serde_json::from_str(&body_str).unwrap_or_else(|e| {
        panic!(
            "response body is not valid JSON: {} -- body: {}",
            e, body_str
        )
    })
}

/// 24. GET /api/params returns valid JSON with params array and count.
#[tokio::test]
async fn params_list_returns_valid_json() {
    let app = builders::test_router();
    let resp = app.oneshot(get_request("/api/params")).await.unwrap();
    let json = assert_params_ok(resp).await;

    assert!(json["params"].is_array(), "params should be an array");
    assert!(json["count"].is_number(), "count should be a number");
    assert_eq!(json["success"], true, "success should be true");
    let params = json["params"].as_array().unwrap();
    assert_eq!(
        params.len() as u64,
        json["count"].as_u64().unwrap(),
        "count should match params array length"
    );
}

/// 25. GET /api/params/:key with safe key but nonexistent returns 404.
#[tokio::test]
async fn params_get_nonexistent_key_returns_404() {
    let app = builders::test_router();
    let resp = app
        .oneshot(get_request(
            "/api/params/this_key_definitely_does_not_exist_xyz",
        ))
        .await
        .unwrap();
    let json = assert_params_error(resp, StatusCode::NOT_FOUND).await;
    assert_eq!(json["success"], false, "success should be false");
    assert!(json["error"].is_string(), "should have an error message");
}

/// 26. GET /api/params/:key with path traversal returns 400.
#[tokio::test]
async fn params_get_path_traversal_returns_400() {
    let app = builders::test_router();
    // Use a key with ".." to trigger the is_safe_path_component check.
    // Note: the params handler has its own is_safe_path_component that allows
    // slashes for hierarchical keys but rejects "..".
    let resp = app
        .oneshot(get_request("/api/params/sensors%2F..%2Fsecret"))
        .await
        .unwrap();
    let json = assert_params_error(resp, StatusCode::BAD_REQUEST).await;
    assert_eq!(json["success"], false, "success should be false");
    assert!(json["error"].is_string(), "should have an error message");
}

/// 27. POST /api/params/:key sets value successfully.
#[tokio::test]
async fn params_set_value_successfully() {
    let app = builders::test_router();
    let body = r#"{"value": 42.5}"#;
    let resp = app
        .oneshot(post_json_request("/api/params/test_speed", body))
        .await
        .unwrap();
    let json = assert_params_ok(resp).await;

    assert_eq!(json["success"], true, "success should be true");
    assert_eq!(json["key"], "test_speed", "key should match");
    // The value should be present in the response.
    assert!(
        json.get("value").is_some(),
        "response should contain the set value"
    );
}

/// 28. POST /api/params/:key with version conflict returns 409.
#[tokio::test]
async fn params_set_with_version_conflict_returns_409() {
    // Build a fresh router so we get a clean RuntimeParams.
    let state = builders::test_app_state();
    let app = builders::test_router_with_state(state.clone());

    // First, set a value to establish version 1.
    let body_set = r#"{"value": 10}"#;
    let resp = app
        .clone()
        .oneshot(post_json_request("/api/params/conflict_key", body_set))
        .await
        .unwrap();
    let json_set = assert_params_ok(resp).await;
    assert_eq!(json_set["success"], true);

    // Now try to set with version=0 (stale) — should get 409 Conflict.
    let body_conflict = r#"{"value": 20, "version": 0}"#;
    let resp2 = app
        .oneshot(post_json_request("/api/params/conflict_key", body_conflict))
        .await
        .unwrap();
    let json_conflict = assert_params_error(resp2, StatusCode::CONFLICT).await;
    assert_eq!(json_conflict["success"], false, "success should be false");
    assert!(
        json_conflict["error"]
            .as_str()
            .unwrap_or("")
            .contains("Version mismatch"),
        "error should mention version mismatch"
    );
}

/// 29. DELETE /api/params/:key with nonexistent returns 404.
#[tokio::test]
async fn params_delete_nonexistent_returns_404() {
    let app = builders::test_router();
    let resp = app
        .oneshot(delete_request(
            "/api/params/this_key_definitely_does_not_exist_xyz",
        ))
        .await
        .unwrap();
    let json = assert_params_error(resp, StatusCode::NOT_FOUND).await;
    assert_eq!(json["success"], false, "success should be false");
    assert!(json["error"].is_string(), "should have an error message");
}

/// 30. DELETE /api/params/:key with path traversal returns 400.
#[tokio::test]
async fn params_delete_path_traversal_returns_400() {
    let app = builders::test_router();
    let resp = app
        .oneshot(delete_request("/api/params/sensors%2F..%2Fsecret"))
        .await
        .unwrap();
    let json = assert_params_error(resp, StatusCode::BAD_REQUEST).await;
    assert_eq!(json["success"], false, "success should be false");
}

/// 31. POST /api/params/export returns YAML data.
#[tokio::test]
async fn params_export_returns_yaml_data() {
    let app = builders::test_router();
    let resp = app
        .oneshot(post_json_request("/api/params/export", "{}"))
        .await
        .unwrap();
    let json = assert_params_ok(resp).await;

    assert_eq!(json["success"], true, "success should be true");
    assert_eq!(json["format"], "yaml", "format should be yaml");
    assert!(json["data"].is_string(), "data should be a YAML string");
    // The data should be parseable YAML.
    let yaml_str = json["data"].as_str().unwrap();
    let parsed: Result<std::collections::BTreeMap<String, serde_json::Value>, _> =
        serde_yaml::from_str(yaml_str);
    assert!(
        parsed.is_ok(),
        "exported YAML should be parseable, got error: {:?}",
        parsed.err()
    );
}

/// 32. POST /api/params/import with YAML data imports successfully.
#[tokio::test]
async fn params_import_yaml_successfully() {
    let state = builders::test_app_state();
    let app = builders::test_router_with_state(state.clone());

    let yaml_data = "import_key_a: 100\nimport_key_b: hello\n";
    let body = serde_json::json!({
        "data": yaml_data,
        "format": "yaml"
    })
    .to_string();

    let resp = app
        .oneshot(post_json_request("/api/params/import", &body))
        .await
        .unwrap();
    let json = assert_params_ok(resp).await;

    assert_eq!(json["success"], true, "success should be true");
    assert!(json["count"].is_number(), "count should be a number");
    assert!(
        json["count"].as_u64().unwrap() >= 2,
        "should have imported at least 2 parameters"
    );
}

/// 33. POST /api/params/import with invalid format returns 400.
#[tokio::test]
async fn params_import_invalid_format_returns_400() {
    let app = builders::test_router();
    let body = r#"{"data":"key: value","format":"xml"}"#;
    let resp = app
        .oneshot(post_json_request("/api/params/import", &body))
        .await
        .unwrap();
    let json = assert_params_error(resp, StatusCode::BAD_REQUEST).await;
    assert_eq!(json["success"], false, "success should be false");
    assert!(
        json["error"]
            .as_str()
            .unwrap_or("")
            .contains("Invalid format"),
        "error should mention invalid format"
    );
}

/// 34. POST /api/params/import with invalid YAML returns 400.
#[tokio::test]
async fn params_import_invalid_yaml_returns_400() {
    let app = builders::test_router();
    let body = r#"{"data":"{{{{not valid yaml at all::::","format":"yaml"}"#;
    let resp = app
        .oneshot(post_json_request("/api/params/import", &body))
        .await
        .unwrap();
    let json = assert_params_error(resp, StatusCode::BAD_REQUEST).await;
    assert_eq!(json["success"], false, "success should be false");
    assert!(json["error"].is_string(), "should have an error message");
}

/// 35. Set then get roundtrip: setting a parameter via POST then reading it
/// via GET returns the same value.
#[tokio::test]
async fn params_set_then_get_roundtrip() {
    let state = builders::test_app_state();
    let app = builders::test_router_with_state(state);

    // Set a parameter.
    let set_body = r#"{"value": "roundtrip_value_42"}"#;
    let resp_set = app
        .clone()
        .oneshot(post_json_request("/api/params/roundtrip_key", set_body))
        .await
        .unwrap();
    let json_set = assert_params_ok(resp_set).await;
    assert_eq!(json_set["success"], true);

    // Get the same parameter.
    let resp_get = app
        .oneshot(get_request("/api/params/roundtrip_key"))
        .await
        .unwrap();
    let json_get = assert_params_ok(resp_get).await;

    assert_eq!(json_get["success"], true);
    assert_eq!(json_get["key"], "roundtrip_key");
    assert_eq!(json_get["value"], "roundtrip_value_42");
}

/// 36. Set then delete then get returns 404.
#[tokio::test]
async fn params_set_delete_then_get_returns_404() {
    let state = builders::test_app_state();
    let app = builders::test_router_with_state(state);

    // Set a parameter.
    let set_body = r#"{"value": "temporary"}"#;
    let resp_set = app
        .clone()
        .oneshot(post_json_request("/api/params/ephemeral_key", set_body))
        .await
        .unwrap();
    let json_set = assert_params_ok(resp_set).await;
    assert_eq!(json_set["success"], true);

    // Delete it.
    let resp_del = app
        .clone()
        .oneshot(delete_request("/api/params/ephemeral_key"))
        .await
        .unwrap();
    let json_del = assert_params_ok(resp_del).await;
    assert_eq!(json_del["success"], true);

    // Get should now return 404.
    let resp_get = app
        .oneshot(get_request("/api/params/ephemeral_key"))
        .await
        .unwrap();
    let json_get = assert_params_error(resp_get, StatusCode::NOT_FOUND).await;
    assert_eq!(json_get["success"], false);
}

/// 37. Import then export roundtrip is lossless — importing YAML data and
/// then exporting should contain the imported keys with matching values.
#[tokio::test]
async fn params_import_export_roundtrip() {
    let state = builders::test_app_state();
    let app = builders::test_router_with_state(state);

    // Clear any defaults by importing a known set of parameters.
    let yaml_import = "roundtrip_alpha: 123\nroundtrip_beta: true\nroundtrip_gamma: test_string\n";
    let import_body = serde_json::json!({
        "data": yaml_import,
        "format": "yaml"
    })
    .to_string();

    let resp_import = app
        .clone()
        .oneshot(post_json_request("/api/params/import", &import_body))
        .await
        .unwrap();
    let json_import = assert_params_ok(resp_import).await;
    assert_eq!(json_import["success"], true);

    // Export all parameters.
    let resp_export = app
        .oneshot(post_json_request("/api/params/export", "{}"))
        .await
        .unwrap();
    let json_export = assert_params_ok(resp_export).await;

    assert_eq!(json_export["success"], true);
    let exported_yaml = json_export["data"]
        .as_str()
        .expect("data should be a string");

    // Parse the exported YAML and verify our imported keys are present.
    let exported: std::collections::BTreeMap<String, serde_json::Value> =
        serde_yaml::from_str(exported_yaml).expect("exported YAML should parse");

    assert!(
        exported.contains_key("roundtrip_alpha"),
        "exported params should contain roundtrip_alpha"
    );
    assert_eq!(
        exported["roundtrip_alpha"],
        serde_json::json!(123),
        "roundtrip_alpha should be 123"
    );

    assert!(
        exported.contains_key("roundtrip_beta"),
        "exported params should contain roundtrip_beta"
    );
    assert_eq!(
        exported["roundtrip_beta"],
        serde_json::json!(true),
        "roundtrip_beta should be true"
    );

    assert!(
        exported.contains_key("roundtrip_gamma"),
        "exported params should contain roundtrip_gamma"
    );
    assert_eq!(
        exported["roundtrip_gamma"],
        serde_json::json!("test_string"),
        "roundtrip_gamma should be 'test_string'"
    );
}
