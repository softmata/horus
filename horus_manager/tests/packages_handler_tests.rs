//! Comprehensive unit tests for the packages handler endpoints.
//!
//! Exercises the five package management routes via the test router:
//!
//! - `GET  /api/packages/registry?q=...`  — search the remote registry
//! - `GET  /api/packages/environments`    — list global + local environments
//! - `POST /api/packages/install`         — install a package
//! - `POST /api/packages/uninstall`       — uninstall a package
//! - `POST /api/packages/publish`         — publish the current workspace
//!
//! Tests focus on input validation (path traversal, null bytes, empty names,
//! missing fields), expected error codes from Axum's extractor layer, and
//! structural correctness of success responses.

mod harness;
mod monitor_tests;

use monitor_tests::builders;
use monitor_tests::helpers::{assert_json_error, assert_json_ok, get_request, post_json_request};
use tower::ServiceExt;

// ═══════════════════════════════════════════════════════════════════════════════
//  REGISTRY HANDLER  (GET /api/packages/registry)
// ═══════════════════════════════════════════════════════════════════════════════

/// Missing `?q=` query parameter should be rejected by Axum's `Query` extractor
/// with a 422 Unprocessable Entity (or 400 Bad Request).
#[tokio::test]
async fn registry_missing_query_param_returns_422() {
    let app = builders::test_router();
    let resp = app
        .oneshot(get_request("/api/packages/registry"))
        .await
        .unwrap();

    let status = resp.status();
    assert!(
        status == axum::http::StatusCode::UNPROCESSABLE_ENTITY
            || status == axum::http::StatusCode::BAD_REQUEST,
        "missing ?q= should return 422 or 400, got {}",
        status
    );
}

/// An empty `?q=` is structurally valid (the `q` field is present as an empty
/// string).  The handler should accept it and return a JSON response — it may
/// succeed with empty results or fail at the registry level, but should not be
/// rejected by the framework.
#[tokio::test]
async fn registry_empty_query_returns_json() {
    let app = builders::test_router();
    let resp = app
        .oneshot(get_request("/api/packages/registry?q="))
        .await
        .unwrap();

    let status = resp.status();
    // The handler returns 200 on success or 500 if the registry client fails.
    // Both are valid JSON responses — the key point is it is NOT 422.
    assert_ne!(
        status,
        axum::http::StatusCode::UNPROCESSABLE_ENTITY,
        "empty ?q= should not be rejected as missing parameter"
    );

    // The body should be parseable JSON regardless of status.
    let body = axum::body::to_bytes(resp.into_body(), 2 * 1024 * 1024)
        .await
        .expect("reading body must not fail");
    let json: serde_json::Value =
        serde_json::from_slice(&body).expect("response should be valid JSON");

    // Either we get a "packages" array (success) or an "error" string (failure).
    assert!(
        json.get("packages").is_some() || json.get("error").is_some(),
        "response should contain 'packages' or 'error', got: {}",
        json
    );
}

/// A non-empty query parameter should reach the handler and return structured
/// JSON.  The registry may be unreachable (500), but the response must be JSON
/// with either a `packages` array or an `error` field.
#[tokio::test]
async fn registry_with_query_returns_structured_json() {
    let app = builders::test_router();
    let resp = app
        .oneshot(get_request("/api/packages/registry?q=test_nonexistent_pkg"))
        .await
        .unwrap();

    let status = resp.status();
    // 200 (found packages) or 500 (registry unreachable) — both are valid.
    assert!(
        status == axum::http::StatusCode::OK
            || status == axum::http::StatusCode::INTERNAL_SERVER_ERROR,
        "expected 200 or 500, got {}",
        status
    );

    let body = axum::body::to_bytes(resp.into_body(), 2 * 1024 * 1024)
        .await
        .unwrap();
    let json: serde_json::Value = serde_json::from_slice(&body).unwrap();

    assert!(
        json.get("packages").is_some() || json.get("error").is_some(),
        "response should contain 'packages' or 'error'"
    );
}

// ═══════════════════════════════════════════════════════════════════════════════
//  ENVIRONMENTS HANDLER  (GET /api/packages/environments)
// ═══════════════════════════════════════════════════════════════════════════════

/// The environments handler should return 200 with a JSON body containing
/// `global` and `local` arrays.
#[tokio::test]
async fn environments_returns_valid_json() {
    let app = builders::test_router();
    let resp = app
        .oneshot(get_request("/api/packages/environments"))
        .await
        .unwrap();
    let json = assert_json_ok(resp).await;

    assert!(
        json["global"].is_array(),
        "environments response should have a 'global' array, got: {:?}",
        json.get("global")
    );
    assert!(
        json["local"].is_array(),
        "environments response should have a 'local' array, got: {:?}",
        json.get("local")
    );
}

/// Verify the `global` key exists and is an array (may be empty if no packages
/// are installed globally).
#[tokio::test]
async fn environments_response_contains_global_array() {
    let app = builders::test_router();
    let resp = app
        .oneshot(get_request("/api/packages/environments"))
        .await
        .unwrap();
    let json = assert_json_ok(resp).await;

    let global = json["global"]
        .as_array()
        .expect("'global' should be a JSON array");

    // Each entry in the global array (if any) should have name and version.
    for entry in global {
        assert!(
            entry["name"].is_string(),
            "global package entry should have a 'name' string, got: {}",
            entry
        );
        assert!(
            entry["version"].is_string(),
            "global package entry should have a 'version' string, got: {}",
            entry
        );
    }
}

/// Verify the `local` key exists and is an array.  Each local environment entry
/// should have the expected schema: name, path, packages, package_count,
/// dependencies.
#[tokio::test]
async fn environments_response_contains_local_array() {
    let app = builders::test_router();
    let resp = app
        .oneshot(get_request("/api/packages/environments"))
        .await
        .unwrap();
    let json = assert_json_ok(resp).await;

    let local = json["local"]
        .as_array()
        .expect("'local' should be a JSON array");

    // Each entry in the local array (if any) should have the expected schema.
    for entry in local {
        assert!(
            entry["name"].is_string(),
            "local env entry should have 'name', got: {}",
            entry
        );
        assert!(
            entry["path"].is_string(),
            "local env entry should have 'path', got: {}",
            entry
        );
        assert!(
            entry["packages"].is_array(),
            "local env entry should have 'packages' array, got: {}",
            entry
        );
        assert!(
            entry["package_count"].is_number(),
            "local env entry should have 'package_count' number, got: {}",
            entry
        );
        // Note: 'dependencies' field is in horus.toml [dependencies] section,
        // not in the API response.
    }
}

// ═══════════════════════════════════════════════════════════════════════════════
//  INSTALL HANDLER  (POST /api/packages/install)
// ═══════════════════════════════════════════════════════════════════════════════

/// Path traversal in the `package` field should be caught by
/// `is_safe_path_component` and return 400 Bad Request.
#[tokio::test]
async fn install_path_traversal_returns_400() {
    let app = builders::test_router();
    let resp = app
        .oneshot(post_json_request(
            "/api/packages/install",
            r#"{"package":"../evil"}"#,
        ))
        .await
        .unwrap();
    let json = assert_json_error(resp, axum::http::StatusCode::BAD_REQUEST).await;

    assert_eq!(
        json["error"].as_str().unwrap_or(""),
        "Invalid package name",
        "error message should indicate invalid package name"
    );
}

/// Deeper path traversal pattern should also be rejected.
#[tokio::test]
async fn install_deep_path_traversal_returns_400() {
    let app = builders::test_router();
    let resp = app
        .oneshot(post_json_request(
            "/api/packages/install",
            r#"{"package":"../../etc/passwd"}"#,
        ))
        .await
        .unwrap();
    let json = assert_json_error(resp, axum::http::StatusCode::BAD_REQUEST).await;

    assert_eq!(json["error"].as_str().unwrap_or(""), "Invalid package name");
}

/// Null byte in the package name should be caught by `is_safe_path_component`.
#[tokio::test]
async fn install_null_byte_returns_400() {
    let app = builders::test_router();
    let resp = app
        .oneshot(post_json_request(
            "/api/packages/install",
            r#"{"package":"evil\u0000pkg"}"#,
        ))
        .await
        .unwrap();
    let json = assert_json_error(resp, axum::http::StatusCode::BAD_REQUEST).await;

    assert_eq!(json["error"].as_str().unwrap_or(""), "Invalid package name");
}

/// Empty package name should be rejected by `is_safe_path_component` (which
/// rejects empty strings).
#[tokio::test]
async fn install_empty_package_name_returns_400() {
    let app = builders::test_router();
    let resp = app
        .oneshot(post_json_request(
            "/api/packages/install",
            r#"{"package":""}"#,
        ))
        .await
        .unwrap();
    let json = assert_json_error(resp, axum::http::StatusCode::BAD_REQUEST).await;

    assert_eq!(json["error"].as_str().unwrap_or(""), "Invalid package name");
}

/// Backslash in the package name should be rejected.
#[tokio::test]
async fn install_backslash_in_name_returns_400() {
    let app = builders::test_router();
    let resp = app
        .oneshot(post_json_request(
            "/api/packages/install",
            r#"{"package":"foo\\bar"}"#,
        ))
        .await
        .unwrap();
    let json = assert_json_error(resp, axum::http::StatusCode::BAD_REQUEST).await;

    assert_eq!(json["error"].as_str().unwrap_or(""), "Invalid package name");
}

/// Forward slash in the package name should be rejected.
#[tokio::test]
async fn install_forward_slash_in_name_returns_400() {
    let app = builders::test_router();
    let resp = app
        .oneshot(post_json_request(
            "/api/packages/install",
            r#"{"package":"foo/bar"}"#,
        ))
        .await
        .unwrap();
    let json = assert_json_error(resp, axum::http::StatusCode::BAD_REQUEST).await;

    assert_eq!(json["error"].as_str().unwrap_or(""), "Invalid package name");
}

/// A valid-looking package name should pass the `is_safe_path_component` check
/// and proceed to the registry client.  The registry call will likely fail (no
/// server), but the response should NOT be 400 — it should be either 200
/// (success) or 400/500 with a registry-level error (not "Invalid package
/// name").
#[tokio::test]
async fn install_valid_name_passes_validation() {
    let app = builders::test_router();
    let resp = app
        .oneshot(post_json_request(
            "/api/packages/install",
            r#"{"package":"my-valid-package"}"#,
        ))
        .await
        .unwrap();

    let status = resp.status();
    let body = axum::body::to_bytes(resp.into_body(), 2 * 1024 * 1024)
        .await
        .unwrap();
    let json: serde_json::Value = serde_json::from_slice(&body).unwrap();

    // If it is 400, the error should NOT be "Invalid package name" — that would
    // mean validation incorrectly rejected a valid name.
    if status == axum::http::StatusCode::BAD_REQUEST {
        let error_msg = json["error"].as_str().unwrap_or("");
        assert_ne!(
            error_msg, "Invalid package name",
            "a valid package name should not trigger the path-component validator"
        );
    }
    // 200 (installed) or 400/500 (registry failure) are all acceptable.
}

/// A package name with hyphens, underscores, and dots should pass validation.
#[tokio::test]
async fn install_name_with_allowed_chars_passes_validation() {
    let app = builders::test_router();
    let resp = app
        .oneshot(post_json_request(
            "/api/packages/install",
            r#"{"package":"my_pkg-v1.2.3"}"#,
        ))
        .await
        .unwrap();

    let status = resp.status();
    let body = axum::body::to_bytes(resp.into_body(), 2 * 1024 * 1024)
        .await
        .unwrap();
    let json: serde_json::Value = serde_json::from_slice(&body).unwrap();

    if status == axum::http::StatusCode::BAD_REQUEST {
        let error_msg = json["error"].as_str().unwrap_or("");
        assert_ne!(
            error_msg, "Invalid package name",
            "'my_pkg-v1.2.3' should pass validation"
        );
    }
}

/// Missing the required `package` field in the JSON body should be rejected by
/// Axum's `Json<InstallRequest>` extractor with 422 (or 400).
#[tokio::test]
async fn install_missing_package_field_returns_error() {
    let app = builders::test_router();
    let resp = app
        .oneshot(post_json_request(
            "/api/packages/install",
            r#"{"target":"global"}"#,
        ))
        .await
        .unwrap();

    let status = resp.status();
    assert!(
        status == axum::http::StatusCode::UNPROCESSABLE_ENTITY
            || status == axum::http::StatusCode::BAD_REQUEST,
        "missing 'package' field should return 422 or 400, got {}",
        status
    );
}

/// Completely invalid JSON body should be rejected by Axum's JSON extractor.
#[tokio::test]
async fn install_invalid_json_returns_error() {
    let app = builders::test_router();
    let resp = app
        .oneshot(post_json_request(
            "/api/packages/install",
            "not valid json at all",
        ))
        .await
        .unwrap();

    let status = resp.status();
    assert!(
        status == axum::http::StatusCode::UNPROCESSABLE_ENTITY
            || status == axum::http::StatusCode::BAD_REQUEST,
        "invalid JSON body should return 422 or 400, got {}",
        status
    );
}

/// Empty JSON body should be rejected.
#[tokio::test]
async fn install_empty_body_returns_error() {
    let app = builders::test_router();
    let resp = app
        .oneshot(post_json_request("/api/packages/install", ""))
        .await
        .unwrap();

    let status = resp.status();
    assert!(
        status == axum::http::StatusCode::UNPROCESSABLE_ENTITY
            || status == axum::http::StatusCode::BAD_REQUEST,
        "empty body should return 422 or 400, got {}",
        status
    );
}

/// Install with a valid package name and `target: "global"` should pass
/// validation and attempt global install (will likely fail at registry level).
#[tokio::test]
async fn install_with_global_target_passes_validation() {
    let app = builders::test_router();
    let resp = app
        .oneshot(post_json_request(
            "/api/packages/install",
            r#"{"package":"some-package","target":"global"}"#,
        ))
        .await
        .unwrap();

    let status = resp.status();
    let body = axum::body::to_bytes(resp.into_body(), 2 * 1024 * 1024)
        .await
        .unwrap();
    let json: serde_json::Value = serde_json::from_slice(&body).unwrap();

    // Should not be rejected at the validation level.
    if status == axum::http::StatusCode::BAD_REQUEST {
        let error_msg = json["error"].as_str().unwrap_or("");
        assert_ne!(
            error_msg, "Invalid package name",
            "valid package with target=global should pass validation"
        );
    }
}

// ═══════════════════════════════════════════════════════════════════════════════
//  UNINSTALL HANDLER  (POST /api/packages/uninstall)
// ═══════════════════════════════════════════════════════════════════════════════

/// Path traversal in `parent_package` should be rejected with 400.
#[tokio::test]
async fn uninstall_parent_path_traversal_returns_400() {
    let app = builders::test_router();
    let resp = app
        .oneshot(post_json_request(
            "/api/packages/uninstall",
            r#"{"parent_package":"../evil","package":"child"}"#,
        ))
        .await
        .unwrap();
    let json = assert_json_error(resp, axum::http::StatusCode::BAD_REQUEST).await;

    assert_eq!(json["error"].as_str().unwrap_or(""), "Invalid package name");
}

/// Path traversal in `package` (child) should be rejected with 400.
#[tokio::test]
async fn uninstall_package_path_traversal_returns_400() {
    let app = builders::test_router();
    let resp = app
        .oneshot(post_json_request(
            "/api/packages/uninstall",
            r#"{"parent_package":"legit-parent","package":"../../etc/shadow"}"#,
        ))
        .await
        .unwrap();
    let json = assert_json_error(resp, axum::http::StatusCode::BAD_REQUEST).await;

    assert_eq!(json["error"].as_str().unwrap_or(""), "Invalid package name");
}

/// Path traversal in BOTH fields should be rejected with 400 (the handler
/// checks with OR logic — either being unsafe triggers rejection).
#[tokio::test]
async fn uninstall_both_fields_path_traversal_returns_400() {
    let app = builders::test_router();
    let resp = app
        .oneshot(post_json_request(
            "/api/packages/uninstall",
            r#"{"parent_package":"../a","package":"../b"}"#,
        ))
        .await
        .unwrap();
    let json = assert_json_error(resp, axum::http::StatusCode::BAD_REQUEST).await;

    assert_eq!(json["error"].as_str().unwrap_or(""), "Invalid package name");
}

/// Null byte in `parent_package` should be rejected.
#[tokio::test]
async fn uninstall_null_byte_in_parent_returns_400() {
    let app = builders::test_router();
    let resp = app
        .oneshot(post_json_request(
            "/api/packages/uninstall",
            r#"{"parent_package":"evil\u0000parent","package":"child"}"#,
        ))
        .await
        .unwrap();
    let json = assert_json_error(resp, axum::http::StatusCode::BAD_REQUEST).await;

    assert_eq!(json["error"].as_str().unwrap_or(""), "Invalid package name");
}

/// Null byte in `package` (child) should be rejected.
#[tokio::test]
async fn uninstall_null_byte_in_package_returns_400() {
    let app = builders::test_router();
    let resp = app
        .oneshot(post_json_request(
            "/api/packages/uninstall",
            r#"{"parent_package":"parent","package":"evil\u0000child"}"#,
        ))
        .await
        .unwrap();
    let json = assert_json_error(resp, axum::http::StatusCode::BAD_REQUEST).await;

    assert_eq!(json["error"].as_str().unwrap_or(""), "Invalid package name");
}

/// Empty `parent_package` should be rejected (is_safe_path_component rejects
/// empty strings).
#[tokio::test]
async fn uninstall_empty_parent_package_returns_400() {
    let app = builders::test_router();
    let resp = app
        .oneshot(post_json_request(
            "/api/packages/uninstall",
            r#"{"parent_package":"","package":"child"}"#,
        ))
        .await
        .unwrap();
    let json = assert_json_error(resp, axum::http::StatusCode::BAD_REQUEST).await;

    assert_eq!(json["error"].as_str().unwrap_or(""), "Invalid package name");
}

/// Empty `package` should be rejected.
#[tokio::test]
async fn uninstall_empty_package_returns_400() {
    let app = builders::test_router();
    let resp = app
        .oneshot(post_json_request(
            "/api/packages/uninstall",
            r#"{"parent_package":"parent","package":""}"#,
        ))
        .await
        .unwrap();
    let json = assert_json_error(resp, axum::http::StatusCode::BAD_REQUEST).await;

    assert_eq!(json["error"].as_str().unwrap_or(""), "Invalid package name");
}

/// Missing required fields in the uninstall request body should be rejected by
/// Axum's `Json<UninstallRequest>` extractor.
#[tokio::test]
async fn uninstall_missing_fields_returns_error() {
    let app = builders::test_router();
    // Missing both required fields.
    let resp = app
        .oneshot(post_json_request("/api/packages/uninstall", r#"{}"#))
        .await
        .unwrap();

    let status = resp.status();
    assert!(
        status == axum::http::StatusCode::UNPROCESSABLE_ENTITY
            || status == axum::http::StatusCode::BAD_REQUEST,
        "missing required fields should return 422 or 400, got {}",
        status
    );
}

/// Missing only `package` field should still be rejected.
#[tokio::test]
async fn uninstall_missing_package_field_returns_error() {
    let app = builders::test_router();
    let resp = app
        .oneshot(post_json_request(
            "/api/packages/uninstall",
            r#"{"parent_package":"parent"}"#,
        ))
        .await
        .unwrap();

    let status = resp.status();
    assert!(
        status == axum::http::StatusCode::UNPROCESSABLE_ENTITY
            || status == axum::http::StatusCode::BAD_REQUEST,
        "missing 'package' field should return 422 or 400, got {}",
        status
    );
}

/// Missing only `parent_package` field should still be rejected.
#[tokio::test]
async fn uninstall_missing_parent_package_field_returns_error() {
    let app = builders::test_router();
    let resp = app
        .oneshot(post_json_request(
            "/api/packages/uninstall",
            r#"{"package":"child"}"#,
        ))
        .await
        .unwrap();

    let status = resp.status();
    assert!(
        status == axum::http::StatusCode::UNPROCESSABLE_ENTITY
            || status == axum::http::StatusCode::BAD_REQUEST,
        "missing 'parent_package' field should return 422 or 400, got {}",
        status
    );
}

/// Valid-looking uninstall request should pass validation (will fail at the
/// filesystem level since the package does not exist, returning 400 with a
/// registry-level error, NOT "Invalid package name").
#[tokio::test]
async fn uninstall_valid_names_pass_validation() {
    let app = builders::test_router();
    let resp = app
        .oneshot(post_json_request(
            "/api/packages/uninstall",
            r#"{"parent_package":"my-parent","package":"my-child"}"#,
        ))
        .await
        .unwrap();

    let status = resp.status();
    let body = axum::body::to_bytes(resp.into_body(), 2 * 1024 * 1024)
        .await
        .unwrap();
    let json: serde_json::Value = serde_json::from_slice(&body).unwrap();

    // Should not be rejected at the validation level.
    if status == axum::http::StatusCode::BAD_REQUEST {
        let error_msg = json["error"].as_str().unwrap_or("");
        assert_ne!(
            error_msg, "Invalid package name",
            "valid package names should pass the path-component validator"
        );
    }
}

// ═══════════════════════════════════════════════════════════════════════════════
//  PUBLISH HANDLER  (POST /api/packages/publish)
// ═══════════════════════════════════════════════════════════════════════════════

/// The publish handler accepts no body parameters and should return structured
/// JSON.  It will likely fail (no workspace or registry), but should return
/// either 200, 400, or 500 with a JSON body.
#[tokio::test]
async fn publish_returns_structured_json() {
    let app = builders::test_router();
    let resp = app
        .oneshot(post_json_request("/api/packages/publish", "{}"))
        .await
        .unwrap();

    let status = resp.status();
    // 200 (published), 400 (bad request), or 500 (registry/workspace error).
    assert!(
        status == axum::http::StatusCode::OK
            || status == axum::http::StatusCode::BAD_REQUEST
            || status == axum::http::StatusCode::INTERNAL_SERVER_ERROR,
        "publish should return 200, 400, or 500, got {}",
        status
    );

    let body = axum::body::to_bytes(resp.into_body(), 2 * 1024 * 1024)
        .await
        .unwrap();
    let json: serde_json::Value =
        serde_json::from_slice(&body).expect("publish response should be valid JSON");

    // The response should contain either success info or an error.
    assert!(
        json.get("success").is_some() || json.get("error").is_some(),
        "publish response should contain 'success' or 'error', got: {}",
        json
    );
}

/// Publish with an empty body (no JSON) should still work since the handler
/// does not read a body — it uses `spawn_blocking` directly.
#[tokio::test]
async fn publish_empty_body_returns_json() {
    let app = builders::test_router();

    // Build a POST request without Content-Type or body.
    let req = axum::http::Request::builder()
        .method("POST")
        .uri("/api/packages/publish")
        .body(axum::body::Body::empty())
        .unwrap();

    let resp = app.oneshot(req).await.unwrap();

    let status = resp.status();
    assert!(
        status == axum::http::StatusCode::OK
            || status == axum::http::StatusCode::BAD_REQUEST
            || status == axum::http::StatusCode::INTERNAL_SERVER_ERROR,
        "publish should return 200, 400, or 500 even with empty body, got {}",
        status
    );
}

/// Verify the publish handler's success response has the expected shape when
/// it does return (the `success` field should be a boolean and `message` or
/// `error` should be a string).
#[tokio::test]
async fn publish_response_has_expected_fields() {
    let app = builders::test_router();
    let resp = app
        .oneshot(post_json_request("/api/packages/publish", "{}"))
        .await
        .unwrap();

    let body = axum::body::to_bytes(resp.into_body(), 2 * 1024 * 1024)
        .await
        .unwrap();
    let json: serde_json::Value =
        serde_json::from_slice(&body).expect("response should be valid JSON");

    if let Some(success) = json.get("success") {
        assert!(
            success.is_boolean(),
            "'success' field should be a boolean, got: {}",
            success
        );
    }

    // Either 'message' (on success) or 'error' (on failure) should be a string.
    if let Some(msg) = json.get("message") {
        assert!(
            msg.is_string(),
            "'message' should be a string, got: {}",
            msg
        );
    }
    if let Some(err) = json.get("error") {
        assert!(err.is_string(), "'error' should be a string, got: {}", err);
    }
}
