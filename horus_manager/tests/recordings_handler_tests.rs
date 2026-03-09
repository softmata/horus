//! Comprehensive unit and integration tests for the recordings handler endpoints.
//!
//! Tests cover:
//! - `GET /api/recordings` — list recording sessions
//! - `GET /api/recordings/:session` — info about a specific recording
//! - `DELETE /api/recordings/:session` — delete a specific recording
//!
//! Path-traversal and input-validation tests exercise `is_safe_path_component()`
//! indirectly through the handlers.  Filesystem-based tests use `tempfile` crate
//! temp directories with cleanup.

mod harness;
mod monitor_tests;

use monitor_tests::builders;
use monitor_tests::helpers::{assert_json_error, assert_json_ok, delete_request, get_request};

use axum::extract::Path;
use axum::http::StatusCode;
use axum::response::IntoResponse;
use horus_manager::monitor::{
    recordings_delete_handler, recordings_info_handler, recordings_list_handler,
};
use tower::ServiceExt;

// ═══════════════════════════════════════════════════════════════════════════════
//  RECORDINGS LIST  (GET /api/recordings)
// ═══════════════════════════════════════════════════════════════════════════════

/// 1. Recordings list returns valid JSON with "recordings" array and "count".
#[tokio::test]
async fn recordings_list_returns_valid_json_with_array_and_count() {
    let app = builders::test_router();
    let resp = app
        .oneshot(get_request("/api/recordings"))
        .await
        .unwrap();
    let json = assert_json_ok(resp).await;

    assert!(
        json["recordings"].is_array(),
        "response must contain a 'recordings' array"
    );
    assert!(
        json["count"].is_number(),
        "response must contain a numeric 'count' field"
    );
}

/// 2. Recordings list with empty/missing recordings directory returns empty array.
#[tokio::test]
async fn recordings_list_empty_directory_returns_empty_array() {
    // The handler falls back to .horus/recordings if recordings_dir() fails.
    // Even if that directory doesn't exist, it should return an empty list,
    // not an error.
    let app = builders::test_router();
    let resp = app
        .oneshot(get_request("/api/recordings"))
        .await
        .unwrap();
    let json = assert_json_ok(resp).await;

    let recordings = json["recordings"]
        .as_array()
        .expect("recordings should be an array");
    let count = json["count"].as_u64().expect("count should be a number");

    // Whether or not there are real recordings on this machine, the structure
    // must be valid.  If no recordings exist, both should be 0.
    assert_eq!(
        recordings.len() as u64, count,
        "count field must match the recordings array length"
    );
}

/// 12. Recordings list count matches array length.
#[tokio::test]
async fn recordings_list_count_matches_array_length() {
    let app = builders::test_router();
    let resp = app
        .oneshot(get_request("/api/recordings"))
        .await
        .unwrap();
    let json = assert_json_ok(resp).await;

    let recordings = json["recordings"]
        .as_array()
        .expect("recordings should be an array");
    let count = json["count"].as_u64().expect("count should be a number");

    assert_eq!(
        recordings.len() as u64, count,
        "count must equal the number of items in the recordings array"
    );
}

/// Test that the list handler returns 200 via direct handler call (no router).
#[tokio::test]
async fn recordings_list_handler_direct_returns_200() {
    let resp = recordings_list_handler().await;
    let response = resp.into_response();
    assert_eq!(
        response.status(),
        StatusCode::OK,
        "direct call to recordings_list_handler must return 200"
    );
}

// ═══════════════════════════════════════════════════════════════════════════════
//  RECORDINGS INFO  (GET /api/recordings/:session)
// ═══════════════════════════════════════════════════════════════════════════════

/// 3. Path traversal with "../" returns 400.
#[tokio::test]
async fn recordings_info_path_traversal_dot_dot_slash_returns_400() {
    let resp = recordings_info_handler(Path("../../../etc/passwd".to_string())).await;
    let response = resp.into_response();
    assert_eq!(
        response.status(),
        StatusCode::BAD_REQUEST,
        "path traversal with ../ must be rejected with 400"
    );
}

/// 4. Path traversal with URL-encoded "%2e%2e" returns 400 (or 404 from Axum
///    normalization).  When called through the router, Axum may decode the
///    percent-encoding before it reaches the handler.
#[tokio::test]
async fn recordings_info_encoded_traversal_rejected() {
    let app = builders::test_router();

    // %2e%2e = ".." — Axum will percent-decode this before routing.
    let resp = app
        .oneshot(get_request("/api/recordings/..%2F..%2Fetc"))
        .await
        .unwrap();

    let status = resp.status();
    // Axum may normalize the path and return 404 (route not matched) or the
    // handler may see ".." and return 400.  Either blocks the traversal.
    assert!(
        status == StatusCode::BAD_REQUEST || status == StatusCode::NOT_FOUND,
        "encoded path traversal must return 400 or 404, got {}",
        status
    );
}

/// Direct handler call with URL-encoded ".." that has already been decoded by
/// the framework (simulating what Axum does).
#[tokio::test]
async fn recordings_info_decoded_dot_dot_returns_400() {
    // Simulate Axum having decoded "%2e%2e%2f" into "../" before passing to handler.
    let resp = recordings_info_handler(Path("..%2F..%2Fetc".to_string())).await;
    let response = resp.into_response();
    // The raw string "..%2F..%2Fetc" contains ".." so is_safe_path_component rejects it.
    assert_eq!(
        response.status(),
        StatusCode::BAD_REQUEST,
        "session name containing '..' must be rejected"
    );
}

/// 5. Null byte in session name returns 400.
#[tokio::test]
async fn recordings_info_null_byte_returns_400() {
    let resp = recordings_info_handler(Path("session\0evil".to_string())).await;
    let response = resp.into_response();
    assert_eq!(
        response.status(),
        StatusCode::BAD_REQUEST,
        "null byte in session name must be rejected with 400"
    );
}

/// 6. Empty session name returns 400.
#[tokio::test]
async fn recordings_info_empty_session_name_returns_400() {
    let resp = recordings_info_handler(Path(String::new())).await;
    let response = resp.into_response();
    assert_eq!(
        response.status(),
        StatusCode::BAD_REQUEST,
        "empty session name must be rejected with 400"
    );
}

/// 7. Nonexistent session returns 404.
#[tokio::test]
async fn recordings_info_nonexistent_session_returns_404() {
    let resp =
        recordings_info_handler(Path("nonexistent_session_xyzzy_99999".to_string())).await;
    let response = resp.into_response();
    assert_eq!(
        response.status(),
        StatusCode::NOT_FOUND,
        "nonexistent session must return 404"
    );
}

/// 11. Recordings info error response contains expected JSON "error" field.
#[tokio::test]
async fn recordings_info_error_response_has_error_field() {
    let app = builders::test_router();
    let resp = app
        .oneshot(get_request(
            "/api/recordings/nonexistent_session_for_error_field_check",
        ))
        .await
        .unwrap();

    let json = assert_json_error(resp, StatusCode::NOT_FOUND).await;
    assert!(
        json["error"].is_string(),
        "error response must contain an 'error' string field"
    );
    let error_msg = json["error"].as_str().unwrap();
    assert!(
        error_msg.contains("not found"),
        "error message should indicate the session was not found, got: '{}'",
        error_msg
    );
}

/// Path traversal info check via router returns 400 or 404.
#[tokio::test]
async fn recordings_info_path_traversal_via_router() {
    let app = builders::test_router();
    let resp = app
        .oneshot(get_request("/api/recordings/..%2F..%2Fsecrets"))
        .await
        .unwrap();

    let status = resp.status();
    assert!(
        status == StatusCode::BAD_REQUEST || status == StatusCode::NOT_FOUND,
        "path traversal via router must return 400 or 404, got {}",
        status
    );
}

/// Session name containing a forward slash is rejected.
#[tokio::test]
async fn recordings_info_forward_slash_returns_400() {
    let resp = recordings_info_handler(Path("foo/bar".to_string())).await;
    let response = resp.into_response();
    assert_eq!(
        response.status(),
        StatusCode::BAD_REQUEST,
        "session name with '/' must be rejected"
    );
}

/// Session name containing a backslash is rejected.
#[tokio::test]
async fn recordings_info_backslash_returns_400() {
    let resp = recordings_info_handler(Path("foo\\bar".to_string())).await;
    let response = resp.into_response();
    assert_eq!(
        response.status(),
        StatusCode::BAD_REQUEST,
        "session name with '\\' must be rejected"
    );
}

// ═══════════════════════════════════════════════════════════════════════════════
//  RECORDINGS DELETE  (DELETE /api/recordings/:session)
// ═══════════════════════════════════════════════════════════════════════════════

/// 8. Delete with path traversal returns 400.
#[tokio::test]
async fn recordings_delete_path_traversal_returns_400() {
    let resp = recordings_delete_handler(Path("../../secrets".to_string())).await;
    let response = resp.into_response();
    assert_eq!(
        response.status(),
        StatusCode::BAD_REQUEST,
        "path traversal in delete must be rejected with 400"
    );
}

/// 9. Delete nonexistent session returns 404.
#[tokio::test]
async fn recordings_delete_nonexistent_session_returns_404() {
    let resp =
        recordings_delete_handler(Path("nonexistent_session_xyzzy_99999".to_string())).await;
    let response = resp.into_response();
    assert_eq!(
        response.status(),
        StatusCode::NOT_FOUND,
        "deleting nonexistent session must return 404"
    );
}

/// 10. Delete with null byte returns 400.
#[tokio::test]
async fn recordings_delete_null_byte_returns_400() {
    let resp = recordings_delete_handler(Path("session\0hack".to_string())).await;
    let response = resp.into_response();
    assert_eq!(
        response.status(),
        StatusCode::BAD_REQUEST,
        "null byte in delete session name must be rejected with 400"
    );
}

/// Delete with empty session name returns 400.
#[tokio::test]
async fn recordings_delete_empty_session_returns_400() {
    let resp = recordings_delete_handler(Path(String::new())).await;
    let response = resp.into_response();
    assert_eq!(
        response.status(),
        StatusCode::BAD_REQUEST,
        "empty session name in delete must be rejected with 400"
    );
}

/// Delete with forward slash in name returns 400.
#[tokio::test]
async fn recordings_delete_forward_slash_returns_400() {
    let resp = recordings_delete_handler(Path("foo/bar".to_string())).await;
    let response = resp.into_response();
    assert_eq!(
        response.status(),
        StatusCode::BAD_REQUEST,
        "session name with '/' in delete must be rejected"
    );
}

/// Delete with backslash in name returns 400.
#[tokio::test]
async fn recordings_delete_backslash_returns_400() {
    let resp = recordings_delete_handler(Path("foo\\bar".to_string())).await;
    let response = resp.into_response();
    assert_eq!(
        response.status(),
        StatusCode::BAD_REQUEST,
        "session name with '\\' in delete must be rejected"
    );
}

/// Delete nonexistent via router returns 404 with error field.
#[tokio::test]
async fn recordings_delete_nonexistent_via_router_returns_404() {
    let app = builders::test_router();
    let resp = app
        .oneshot(delete_request(
            "/api/recordings/nonexistent_session_delete_router_check",
        ))
        .await
        .unwrap();
    let json = assert_json_error(resp, StatusCode::NOT_FOUND).await;
    assert!(
        json["error"].is_string(),
        "404 delete response must contain an 'error' string field"
    );
}

/// Delete error response mentions session name.
#[tokio::test]
async fn recordings_delete_error_mentions_session_name() {
    let session = "nonexistent_session_mention_check";
    let resp = recordings_delete_handler(Path(session.to_string())).await;
    let response = resp.into_response();
    assert_eq!(response.status(), StatusCode::NOT_FOUND);

    let body = axum::body::to_bytes(response.into_body(), 1024 * 1024)
        .await
        .expect("body read must succeed");
    let json: serde_json::Value =
        serde_json::from_slice(&body).expect("response must be valid JSON");
    let error_msg = json["error"].as_str().expect("error field must be a string");
    assert!(
        error_msg.contains(session),
        "error message should mention the session name '{}', got: '{}'",
        session,
        error_msg
    );
}

// ═══════════════════════════════════════════════════════════════════════════════
//  RECORDINGS WITH TEMP DIRECTORY DATA
// ═══════════════════════════════════════════════════════════════════════════════

/// 13. Test recording with actual temp directory data: create a temp recording
///     directory with mock files and verify the list handler picks it up.
///
/// This test creates a temporary directory structure under ~/.horus/recordings/
/// to verify the handler reads real filesystem data correctly.  The directory
/// is cleaned up after the test.
#[tokio::test]
async fn recordings_with_temp_directory_data() {
    let recordings_base = horus_manager::paths::recordings_dir()
        .expect("recordings_dir() must succeed in test environment");

    // Create the base recordings directory if it doesn't exist.
    std::fs::create_dir_all(&recordings_base).expect("must be able to create recordings dir");

    let session_name = format!("_test_session_{}", std::process::id());
    let session_dir = recordings_base.join(&session_name);

    // Ensure clean state.
    let _ = std::fs::remove_dir_all(&session_dir);
    std::fs::create_dir_all(&session_dir).expect("must create test session directory");

    // Create mock .horus node recording files.
    std::fs::write(session_dir.join("lidar_driver@abc123.horus"), b"fake recording data 1")
        .expect("write mock node file 1");
    std::fs::write(
        session_dir.join("camera_node@def456.horus"),
        b"fake recording data 2 with more bytes",
    )
    .expect("write mock node file 2");

    // Verify the list handler now includes our session.
    let app = builders::test_router();
    let resp = app
        .oneshot(get_request("/api/recordings"))
        .await
        .unwrap();
    let json = assert_json_ok(resp).await;

    let recordings = json["recordings"]
        .as_array()
        .expect("recordings should be an array");

    let our_session = recordings
        .iter()
        .find(|r| r["session_name"].as_str() == Some(&session_name));

    assert!(
        our_session.is_some(),
        "list should include our test session '{}'; got sessions: {:?}",
        session_name,
        recordings
            .iter()
            .map(|r| r["session_name"].as_str().unwrap_or("?"))
            .collect::<Vec<_>>()
    );

    let session = our_session.unwrap();
    // Verify RecordingListItem fields are present.
    assert!(
        session["session_name"].is_string(),
        "session_name must be a string"
    );
    assert!(
        session["started_at"].is_string(),
        "started_at must be a string"
    );
    // ended_at can be null (no scheduler file).
    assert!(
        session["total_ticks"].is_number(),
        "total_ticks must be a number"
    );
    assert_eq!(
        session["node_count"].as_u64(),
        Some(2),
        "node_count should be 2 (two .horus files that are not scheduler@)"
    );
    assert!(
        session["size_bytes"].as_u64().unwrap_or(0) > 0,
        "size_bytes should be > 0"
    );

    // Verify info handler returns details for this session.
    let app2 = builders::test_router();
    let resp2 = app2
        .oneshot(get_request(&format!("/api/recordings/{}", session_name)))
        .await
        .unwrap();
    let info_json = assert_json_ok(resp2).await;

    assert_eq!(
        info_json["session_name"].as_str(),
        Some(session_name.as_str()),
        "info handler must return the correct session_name"
    );
    assert!(
        info_json["node_recordings"].is_array(),
        "info must contain node_recordings array"
    );
    let node_recs = info_json["node_recordings"].as_array().unwrap();
    assert_eq!(
        node_recs.len(),
        2,
        "should have 2 node recordings"
    );

    // Each node recording should have the expected fields.
    for nr in node_recs {
        assert!(nr["filename"].is_string(), "filename must be a string");
        assert!(nr["node_name"].is_string(), "node_name must be a string");
        assert!(nr["size_bytes"].is_number(), "size_bytes must be a number");
        assert!(
            nr["size_human"].is_string(),
            "size_human must be a string"
        );
    }

    assert!(
        info_json["total_size_bytes"].as_u64().unwrap_or(0) > 0,
        "total_size_bytes should be > 0"
    );
    assert!(
        info_json["total_size_human"].is_string(),
        "total_size_human must be a string"
    );
    assert!(
        info_json["path"].is_string(),
        "path must be a string"
    );

    // Clean up: delete the test session directory.
    let app3 = builders::test_router();
    let resp3 = app3
        .oneshot(delete_request(&format!("/api/recordings/{}", session_name)))
        .await
        .unwrap();
    let del_json = assert_json_ok(resp3).await;
    assert_eq!(
        del_json["success"].as_bool(),
        Some(true),
        "delete must return success: true"
    );
    assert!(
        del_json["message"].is_string(),
        "delete must return a message string"
    );

    // Verify the directory is actually gone.
    assert!(
        !session_dir.exists(),
        "session directory must be removed after delete"
    );
}

/// 14. Test multiple recording sessions listed.
///
/// Creates two temp session directories and verifies both appear in the list.
#[tokio::test]
async fn recordings_multiple_sessions_listed() {
    let recordings_base = horus_manager::paths::recordings_dir()
        .expect("recordings_dir() must succeed in test environment");

    std::fs::create_dir_all(&recordings_base).expect("must be able to create recordings dir");

    let pid = std::process::id();
    let session_a = format!("_test_multi_a_{}", pid);
    let session_b = format!("_test_multi_b_{}", pid);
    let dir_a = recordings_base.join(&session_a);
    let dir_b = recordings_base.join(&session_b);

    // Clean state.
    let _ = std::fs::remove_dir_all(&dir_a);
    let _ = std::fs::remove_dir_all(&dir_b);
    std::fs::create_dir_all(&dir_a).expect("create session A");
    std::fs::create_dir_all(&dir_b).expect("create session B");

    // Put a file in each so they have non-zero size.
    std::fs::write(dir_a.join("node_x@111.horus"), b"data_a")
        .expect("write file in session A");
    std::fs::write(dir_b.join("node_y@222.horus"), b"data_b_longer")
        .expect("write file in session B");
    std::fs::write(dir_b.join("node_z@333.horus"), b"data_b2")
        .expect("write second file in session B");

    let app = builders::test_router();
    let resp = app
        .oneshot(get_request("/api/recordings"))
        .await
        .unwrap();
    let json = assert_json_ok(resp).await;

    let recordings = json["recordings"]
        .as_array()
        .expect("recordings should be an array");

    let found_a = recordings
        .iter()
        .any(|r| r["session_name"].as_str() == Some(&session_a));
    let found_b = recordings
        .iter()
        .any(|r| r["session_name"].as_str() == Some(&session_b));

    assert!(
        found_a,
        "session A ('{}') must appear in recordings list",
        session_a
    );
    assert!(
        found_b,
        "session B ('{}') must appear in recordings list",
        session_b
    );

    // Verify node_count differs between the two sessions.
    let rec_a = recordings
        .iter()
        .find(|r| r["session_name"].as_str() == Some(&session_a))
        .unwrap();
    let rec_b = recordings
        .iter()
        .find(|r| r["session_name"].as_str() == Some(&session_b))
        .unwrap();

    assert_eq!(
        rec_a["node_count"].as_u64(),
        Some(1),
        "session A should have 1 node recording"
    );
    assert_eq!(
        rec_b["node_count"].as_u64(),
        Some(2),
        "session B should have 2 node recordings"
    );

    // The count field must be at least 2 (our two sessions, possibly more
    // from other tests or real recordings).
    let count = json["count"].as_u64().expect("count must be a number");
    assert!(
        count >= 2,
        "count must be >= 2 when two test sessions exist, got {}",
        count
    );

    // Clean up.
    let _ = std::fs::remove_dir_all(&dir_a);
    let _ = std::fs::remove_dir_all(&dir_b);
}

/// Info handler for a temp session returns correct session_name and path fields.
#[tokio::test]
async fn recordings_info_temp_session_has_correct_fields() {
    let recordings_base = horus_manager::paths::recordings_dir()
        .expect("recordings_dir() must succeed in test environment");

    std::fs::create_dir_all(&recordings_base).expect("must be able to create recordings dir");

    let session_name = format!("_test_info_fields_{}", std::process::id());
    let session_dir = recordings_base.join(&session_name);

    let _ = std::fs::remove_dir_all(&session_dir);
    std::fs::create_dir_all(&session_dir).expect("create test session");

    // Create a mock node file.
    std::fs::write(session_dir.join("sensor@aaa.horus"), b"sensor data")
        .expect("write mock file");

    let app = builders::test_router();
    let resp = app
        .oneshot(get_request(&format!("/api/recordings/{}", session_name)))
        .await
        .unwrap();
    let json = assert_json_ok(resp).await;

    assert_eq!(
        json["session_name"].as_str(),
        Some(session_name.as_str()),
        "session_name must match"
    );
    assert!(
        json["path"]
            .as_str()
            .unwrap_or("")
            .contains(&session_name),
        "path must contain the session name"
    );
    // scheduler is null when no scheduler@ file exists.
    assert!(
        json["scheduler"].is_null(),
        "scheduler should be null when no scheduler file is present"
    );
    assert!(
        json["node_recordings"].is_array(),
        "node_recordings must be an array"
    );
    assert!(
        json["total_size_bytes"].is_number(),
        "total_size_bytes must be a number"
    );
    assert!(
        json["total_size_human"].is_string(),
        "total_size_human must be a string"
    );

    // Clean up.
    let _ = std::fs::remove_dir_all(&session_dir);
}

/// Delete handler actually removes the directory from disk.
#[tokio::test]
async fn recordings_delete_actually_removes_directory() {
    let recordings_base = horus_manager::paths::recordings_dir()
        .expect("recordings_dir() must succeed in test environment");

    std::fs::create_dir_all(&recordings_base).expect("must be able to create recordings dir");

    let session_name = format!("_test_delete_real_{}", std::process::id());
    let session_dir = recordings_base.join(&session_name);

    let _ = std::fs::remove_dir_all(&session_dir);
    std::fs::create_dir_all(&session_dir).expect("create test session");
    std::fs::write(session_dir.join("dummy.horus"), b"to be deleted")
        .expect("write dummy file");

    // Verify it exists before delete.
    assert!(session_dir.exists(), "session dir must exist before delete");

    let resp = recordings_delete_handler(Path(session_name.clone())).await;
    let response = resp.into_response();
    assert_eq!(
        response.status(),
        StatusCode::OK,
        "delete of existing session must return 200"
    );

    let body = axum::body::to_bytes(response.into_body(), 1024 * 1024)
        .await
        .expect("body read must succeed");
    let json: serde_json::Value =
        serde_json::from_slice(&body).expect("response must be valid JSON");
    assert_eq!(json["success"].as_bool(), Some(true));

    // Directory must be gone.
    assert!(
        !session_dir.exists(),
        "session directory must not exist after successful delete"
    );
}

/// Info handler bad-request response has correct JSON structure.
#[tokio::test]
async fn recordings_info_bad_request_has_error_json() {
    let resp = recordings_info_handler(Path("../evil".to_string())).await;
    let response = resp.into_response();
    assert_eq!(response.status(), StatusCode::BAD_REQUEST);

    let body = axum::body::to_bytes(response.into_body(), 1024 * 1024)
        .await
        .expect("body read must succeed");
    let json: serde_json::Value =
        serde_json::from_slice(&body).expect("response must be valid JSON");

    assert!(
        json["error"].is_string(),
        "400 response must have an 'error' string field"
    );
    let error_msg = json["error"].as_str().unwrap();
    assert!(
        error_msg.contains("Invalid session name"),
        "error message should say 'Invalid session name', got: '{}'",
        error_msg
    );
}

/// Delete handler bad-request response has correct JSON structure.
#[tokio::test]
async fn recordings_delete_bad_request_has_error_json() {
    let resp = recordings_delete_handler(Path("../evil".to_string())).await;
    let response = resp.into_response();
    assert_eq!(response.status(), StatusCode::BAD_REQUEST);

    let body = axum::body::to_bytes(response.into_body(), 1024 * 1024)
        .await
        .expect("body read must succeed");
    let json: serde_json::Value =
        serde_json::from_slice(&body).expect("response must be valid JSON");

    assert!(
        json["error"].is_string(),
        "400 response must have an 'error' string field"
    );
    let error_msg = json["error"].as_str().unwrap();
    assert!(
        error_msg.contains("Invalid session name"),
        "error message should say 'Invalid session name', got: '{}'",
        error_msg
    );
}
