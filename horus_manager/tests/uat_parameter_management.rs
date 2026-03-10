//! UAT: Parameter management E2E.
//!
//! Verifies full CRUD lifecycle for runtime parameters via the HTTP API:
//! set, get, update, delete, export, and import.

mod harness;
mod monitor_tests;

use monitor_tests::builders;
use monitor_tests::helpers::{get_request, post_json_request};

use tower::ServiceExt;

/// Helper to build a DELETE request.
fn delete_request(uri: &str) -> axum::http::Request<axum::body::Body> {
    axum::http::Request::builder()
        .method("DELETE")
        .uri(uri)
        .body(axum::body::Body::empty())
        .unwrap()
}

// ═══════════════════════════════════════════════════════════════════════════════
//  SET + GET — basic CRUD
// ═══════════════════════════════════════════════════════════════════════════════

#[tokio::test]
async fn set_and_get_parameter() {
    let app = builders::test_router();

    // Set a parameter
    let resp = app
        .clone()
        .oneshot(post_json_request(
            "/api/params/uat_test_key",
            r#"{"value": 42}"#,
        ))
        .await
        .unwrap();
    let json: serde_json::Value = serde_json::from_slice(
        &axum::body::to_bytes(resp.into_body(), 1024 * 1024)
            .await
            .unwrap(),
    )
    .unwrap();
    assert_eq!(json["success"], true, "set must succeed");

    // Get the parameter
    let resp = app
        .oneshot(get_request("/api/params/uat_test_key"))
        .await
        .unwrap();
    let body = axum::body::to_bytes(resp.into_body(), 1024 * 1024)
        .await
        .unwrap();
    let json: serde_json::Value = serde_json::from_slice(&body).unwrap();
    assert_eq!(json["success"], true);
    assert_eq!(json["key"], "uat_test_key");
    assert_eq!(json["value"], 42);
}

#[tokio::test]
async fn set_string_parameter() {
    let app = builders::test_router();

    let resp = app
        .clone()
        .oneshot(post_json_request(
            "/api/params/uat_string_key",
            r#"{"value": "hello world"}"#,
        ))
        .await
        .unwrap();
    let json: serde_json::Value = serde_json::from_slice(
        &axum::body::to_bytes(resp.into_body(), 1024 * 1024)
            .await
            .unwrap(),
    )
    .unwrap();
    assert_eq!(json["success"], true);

    let resp = app
        .oneshot(get_request("/api/params/uat_string_key"))
        .await
        .unwrap();
    let body = axum::body::to_bytes(resp.into_body(), 1024 * 1024)
        .await
        .unwrap();
    let json: serde_json::Value = serde_json::from_slice(&body).unwrap();
    assert_eq!(json["value"], "hello world");
}

#[tokio::test]
async fn set_json_object_parameter() {
    let app = builders::test_router();

    let resp = app
        .clone()
        .oneshot(post_json_request(
            "/api/params/uat_obj_key",
            r#"{"value": {"nested": true, "count": 5}}"#,
        ))
        .await
        .unwrap();
    let json: serde_json::Value = serde_json::from_slice(
        &axum::body::to_bytes(resp.into_body(), 1024 * 1024)
            .await
            .unwrap(),
    )
    .unwrap();
    assert_eq!(json["success"], true);

    let resp = app
        .oneshot(get_request("/api/params/uat_obj_key"))
        .await
        .unwrap();
    let body = axum::body::to_bytes(resp.into_body(), 1024 * 1024)
        .await
        .unwrap();
    let json: serde_json::Value = serde_json::from_slice(&body).unwrap();
    assert_eq!(json["value"]["nested"], true);
    assert_eq!(json["value"]["count"], 5);
}

// ═══════════════════════════════════════════════════════════════════════════════
//  UPDATE — version increments
// ═══════════════════════════════════════════════════════════════════════════════

#[tokio::test]
async fn update_parameter_increments_version() {
    let app = builders::test_router();

    // Set initial value
    let resp = app
        .clone()
        .oneshot(post_json_request(
            "/api/params/uat_version_key",
            r#"{"value": 1}"#,
        ))
        .await
        .unwrap();
    let json: serde_json::Value = serde_json::from_slice(
        &axum::body::to_bytes(resp.into_body(), 1024 * 1024)
            .await
            .unwrap(),
    )
    .unwrap();
    let v1 = json["version"].as_u64().unwrap_or(0);

    // Update
    let resp = app
        .clone()
        .oneshot(post_json_request(
            "/api/params/uat_version_key",
            r#"{"value": 2}"#,
        ))
        .await
        .unwrap();
    let json: serde_json::Value = serde_json::from_slice(
        &axum::body::to_bytes(resp.into_body(), 1024 * 1024)
            .await
            .unwrap(),
    )
    .unwrap();
    let v2 = json["version"].as_u64().unwrap_or(0);

    assert!(
        v2 > v1,
        "version must increment after update: v1={}, v2={}",
        v1,
        v2
    );
}

// ═══════════════════════════════════════════════════════════════════════════════
//  DELETE
// ═══════════════════════════════════════════════════════════════════════════════

#[tokio::test]
async fn delete_parameter_removes_it() {
    let app = builders::test_router();

    // Set
    app.clone()
        .oneshot(post_json_request(
            "/api/params/uat_delete_key",
            r#"{"value": "to_delete"}"#,
        ))
        .await
        .unwrap();

    // Delete
    let resp = app
        .clone()
        .oneshot(delete_request("/api/params/uat_delete_key"))
        .await
        .unwrap();
    let body = axum::body::to_bytes(resp.into_body(), 1024 * 1024)
        .await
        .unwrap();
    let json: serde_json::Value = serde_json::from_slice(&body).unwrap();
    assert_eq!(json["success"], true);

    // Verify gone
    let resp = app
        .oneshot(get_request("/api/params/uat_delete_key"))
        .await
        .unwrap();
    assert_eq!(
        resp.status(),
        axum::http::StatusCode::NOT_FOUND,
        "deleted parameter should return 404"
    );
}

#[tokio::test]
async fn delete_nonexistent_returns_404() {
    let app = builders::test_router();

    let resp = app
        .oneshot(delete_request("/api/params/uat_nonexistent_xyz"))
        .await
        .unwrap();
    assert_eq!(resp.status(), axum::http::StatusCode::NOT_FOUND);
}

// ═══════════════════════════════════════════════════════════════════════════════
//  GET nonexistent
// ═══════════════════════════════════════════════════════════════════════════════

#[tokio::test]
async fn get_nonexistent_returns_404() {
    let app = builders::test_router();

    let resp = app
        .oneshot(get_request("/api/params/uat_missing_key_abc"))
        .await
        .unwrap();
    assert_eq!(resp.status(), axum::http::StatusCode::NOT_FOUND);
}

// ═══════════════════════════════════════════════════════════════════════════════
//  LIST
// ═══════════════════════════════════════════════════════════════════════════════

#[tokio::test]
async fn list_parameters_includes_set_keys() {
    let app = builders::test_router();

    // Set a unique key
    app.clone()
        .oneshot(post_json_request(
            "/api/params/uat_list_check",
            r#"{"value": 99}"#,
        ))
        .await
        .unwrap();

    let resp = app.oneshot(get_request("/api/params")).await.unwrap();
    let body = axum::body::to_bytes(resp.into_body(), 1024 * 1024)
        .await
        .unwrap();
    let json: serde_json::Value = serde_json::from_slice(&body).unwrap();

    assert_eq!(json["success"], true);
    let params = json["params"].as_array().expect("params must be an array");
    assert!(
        params
            .iter()
            .any(|p| p["key"].as_str() == Some("uat_list_check")),
        "uat_list_check must appear in params list"
    );
}

// ═══════════════════════════════════════════════════════════════════════════════
//  EXPORT + IMPORT — roundtrip
// ═══════════════════════════════════════════════════════════════════════════════

#[tokio::test]
async fn export_and_import_roundtrip() {
    let app = builders::test_router();

    // Set some params
    app.clone()
        .oneshot(post_json_request(
            "/api/params/uat_export_a",
            r#"{"value": 10}"#,
        ))
        .await
        .unwrap();
    app.clone()
        .oneshot(post_json_request(
            "/api/params/uat_export_b",
            r#"{"value": "hello"}"#,
        ))
        .await
        .unwrap();

    // Export
    let resp = app
        .clone()
        .oneshot(post_json_request("/api/params/export", "{}"))
        .await
        .unwrap();
    let body = axum::body::to_bytes(resp.into_body(), 1024 * 1024)
        .await
        .unwrap();
    let json: serde_json::Value = serde_json::from_slice(&body).unwrap();
    assert_eq!(json["success"], true);
    assert_eq!(json["format"], "yaml");
    let yaml_data = json["data"].as_str().expect("data must be a YAML string");
    assert!(
        yaml_data.contains("uat_export_a"),
        "exported YAML must contain uat_export_a"
    );
    assert!(
        yaml_data.contains("uat_export_b"),
        "exported YAML must contain uat_export_b"
    );

    // Delete both params
    app.clone()
        .oneshot(delete_request("/api/params/uat_export_a"))
        .await
        .unwrap();
    app.clone()
        .oneshot(delete_request("/api/params/uat_export_b"))
        .await
        .unwrap();

    // Import the YAML back
    let import_body = serde_json::json!({
        "data": yaml_data,
        "format": "yaml"
    })
    .to_string();
    let resp = app
        .clone()
        .oneshot(post_json_request("/api/params/import", &import_body))
        .await
        .unwrap();
    let body = axum::body::to_bytes(resp.into_body(), 1024 * 1024)
        .await
        .unwrap();
    let json: serde_json::Value = serde_json::from_slice(&body).unwrap();
    assert_eq!(json["success"], true);
    assert!(
        json["count"].as_u64().unwrap() >= 2,
        "should have imported at least 2 params"
    );

    // Verify params are restored
    let resp = app
        .clone()
        .oneshot(get_request("/api/params/uat_export_a"))
        .await
        .unwrap();
    let body = axum::body::to_bytes(resp.into_body(), 1024 * 1024)
        .await
        .unwrap();
    let json: serde_json::Value = serde_json::from_slice(&body).unwrap();
    assert_eq!(json["value"], 10, "uat_export_a must be restored to 10");

    let resp = app
        .oneshot(get_request("/api/params/uat_export_b"))
        .await
        .unwrap();
    let body = axum::body::to_bytes(resp.into_body(), 1024 * 1024)
        .await
        .unwrap();
    let json: serde_json::Value = serde_json::from_slice(&body).unwrap();
    assert_eq!(
        json["value"], "hello",
        "uat_export_b must be restored to 'hello'"
    );
}

// ═══════════════════════════════════════════════════════════════════════════════
//  Hierarchical keys
// ═══════════════════════════════════════════════════════════════════════════════

#[tokio::test]
async fn dotted_hierarchical_key_set_and_get() {
    let app = builders::test_router();

    // Use dot-separated key since route captures a single path segment
    let resp = app
        .clone()
        .oneshot(post_json_request(
            "/api/params/sensors.lidar.rate",
            r#"{"value": 30.0}"#,
        ))
        .await
        .unwrap();
    let body = axum::body::to_bytes(resp.into_body(), 1024 * 1024)
        .await
        .unwrap();
    let json: serde_json::Value = serde_json::from_slice(&body).unwrap();
    assert_eq!(json["success"], true);

    let resp = app
        .oneshot(get_request("/api/params/sensors.lidar.rate"))
        .await
        .unwrap();
    let body = axum::body::to_bytes(resp.into_body(), 1024 * 1024)
        .await
        .unwrap();
    let json: serde_json::Value = serde_json::from_slice(&body).unwrap();
    assert_eq!(json["value"], 30.0);
}

// ═══════════════════════════════════════════════════════════════════════════════
//  Version conflict — stale update returns 409
// ═══════════════════════════════════════════════════════════════════════════════

#[tokio::test]
async fn version_conflict_returns_409() {
    let app = builders::test_router();

    // Set initial
    app.clone()
        .oneshot(post_json_request(
            "/api/params/uat_conflict_key",
            r#"{"value": 1}"#,
        ))
        .await
        .unwrap();

    // Try to update with stale version (version 0 when current is 1+)
    let resp = app
        .oneshot(post_json_request(
            "/api/params/uat_conflict_key",
            r#"{"value": 2, "version": 0}"#,
        ))
        .await
        .unwrap();

    assert_eq!(
        resp.status(),
        axum::http::StatusCode::CONFLICT,
        "stale version update must return 409 Conflict"
    );
}

// ═══════════════════════════════════════════════════════════════════════════════
//  Path traversal rejection
// ═══════════════════════════════════════════════════════════════════════════════

#[tokio::test]
async fn path_traversal_rejected() {
    let app = builders::test_router();

    let resp = app
        .oneshot(get_request("/api/params/../../etc/passwd"))
        .await
        .unwrap();
    let status = resp.status();
    assert!(
        status == axum::http::StatusCode::BAD_REQUEST
            || status == axum::http::StatusCode::NOT_FOUND,
        "path traversal must be rejected with 400 or 404, got {}",
        status
    );
}
