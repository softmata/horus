//! UAT: Recording and time-travel debug E2E.
//!
//! Verifies the recording listing API and debug session management.
//! Tests focus on API contract validation: listing, creating sessions,
//! stepping, seeking, breakpoints, watches, and cleanup.
//!
//! Some tests require recording files on disk; those that don't work with
//! the API's schema validation and error handling.
#![cfg(feature = "monitor")]

mod harness;
mod monitor_tests;

use monitor_tests::builders;
use monitor_tests::helpers::{assert_json_ok, delete_request, get_request, post_json_request};

use tower::ServiceExt;

// ═══════════════════════════════════════════════════════════════════════════════
//  GET /api/recordings — listing
// ═══════════════════════════════════════════════════════════════════════════════

#[tokio::test]
async fn recordings_list_returns_valid_structure() {
    let app = builders::test_router();
    let resp = app.oneshot(get_request("/api/recordings")).await.unwrap();
    let json = assert_json_ok(resp).await;

    assert!(json["recordings"].is_array(), "recordings must be an array");
    assert!(json["count"].is_number(), "count must be a number");
    let recordings = json["recordings"].as_array().unwrap();
    assert_eq!(
        recordings.len() as u64,
        json["count"].as_u64().unwrap(),
        "count must match array length"
    );
}

#[tokio::test]
async fn recordings_info_missing_session_returns_error() {
    let app = builders::test_router();
    let resp = app
        .oneshot(get_request("/api/recordings/nonexistent_session_xyz"))
        .await
        .unwrap();
    // Should return 404 or error
    let status = resp.status();
    assert!(
        status == axum::http::StatusCode::NOT_FOUND
            || status == axum::http::StatusCode::INTERNAL_SERVER_ERROR,
        "missing recording session should return error, got {}",
        status
    );
}

#[tokio::test]
async fn recordings_delete_missing_returns_error() {
    let app = builders::test_router();
    let resp = app
        .oneshot(delete_request("/api/recordings/nonexistent_session_xyz"))
        .await
        .unwrap();
    let status = resp.status();
    assert!(
        status == axum::http::StatusCode::NOT_FOUND
            || status == axum::http::StatusCode::INTERNAL_SERVER_ERROR,
        "deleting missing recording should return error, got {}",
        status
    );
}

// ═══════════════════════════════════════════════════════════════════════════════
//  Path traversal protection
// ═══════════════════════════════════════════════════════════════════════════════

#[tokio::test]
async fn recordings_info_path_traversal_rejected() {
    let app = builders::test_router();
    let resp = app
        .oneshot(get_request("/api/recordings/../../etc"))
        .await
        .unwrap();
    let status = resp.status();
    assert!(
        status == axum::http::StatusCode::BAD_REQUEST
            || status == axum::http::StatusCode::NOT_FOUND,
        "path traversal must be rejected, got {}",
        status
    );
}

#[tokio::test]
async fn recordings_delete_path_traversal_rejected() {
    let app = builders::test_router();
    let resp = app
        .oneshot(delete_request("/api/recordings/../../../tmp"))
        .await
        .unwrap();
    let status = resp.status();
    assert!(
        status == axum::http::StatusCode::BAD_REQUEST
            || status == axum::http::StatusCode::NOT_FOUND,
        "path traversal must be rejected, got {}",
        status
    );
}

// ═══════════════════════════════════════════════════════════════════════════════
//  Debug sessions — list
// ═══════════════════════════════════════════════════════════════════════════════

#[tokio::test]
async fn debug_sessions_list_returns_valid_structure() {
    let app = builders::test_router();
    let resp = app
        .oneshot(get_request("/api/debug/sessions"))
        .await
        .unwrap();
    let json = assert_json_ok(resp).await;

    assert!(json["sessions"].is_array(), "sessions must be an array");
    assert!(json["count"].is_number(), "count must be a number");
}

// ═══════════════════════════════════════════════════════════════════════════════
//  Debug session creation — missing recording returns error
// ═══════════════════════════════════════════════════════════════════════════════

#[tokio::test]
async fn debug_create_missing_recording_returns_error() {
    let app = builders::test_router();
    let resp = app
        .oneshot(post_json_request(
            "/api/debug/sessions",
            r#"{"recording_session": "nonexistent", "recording_file": "test.bin"}"#,
        ))
        .await
        .unwrap();
    let status = resp.status();
    assert!(
        status == axum::http::StatusCode::NOT_FOUND
            || status == axum::http::StatusCode::BAD_REQUEST
            || status == axum::http::StatusCode::INTERNAL_SERVER_ERROR,
        "creating debug session with missing recording should return error, got {}",
        status
    );
}

#[tokio::test]
async fn debug_create_path_traversal_rejected() {
    let app = builders::test_router();
    let resp = app
        .oneshot(post_json_request(
            "/api/debug/sessions",
            r#"{"recording_session": "../../etc", "recording_file": "passwd"}"#,
        ))
        .await
        .unwrap();
    assert_eq!(
        resp.status(),
        axum::http::StatusCode::BAD_REQUEST,
        "path traversal in debug session creation must be rejected"
    );
}

// ═══════════════════════════════════════════════════════════════════════════════
//  Debug session operations — missing session returns error
// ═══════════════════════════════════════════════════════════════════════════════

#[tokio::test]
async fn debug_get_missing_session_returns_error() {
    let app = builders::test_router();
    let resp = app
        .oneshot(get_request("/api/debug/sessions/nonexistent_id"))
        .await
        .unwrap();
    let status = resp.status();
    assert!(
        status == axum::http::StatusCode::NOT_FOUND
            || status == axum::http::StatusCode::BAD_REQUEST,
        "getting missing debug session should return error, got {}",
        status
    );
}

#[tokio::test]
async fn debug_step_forward_missing_session_returns_error() {
    let app = builders::test_router();
    let resp = app
        .oneshot(post_json_request(
            "/api/debug/sessions/nonexistent_id/step-forward",
            "{}",
        ))
        .await
        .unwrap();
    let status = resp.status();
    assert!(
        status == axum::http::StatusCode::NOT_FOUND
            || status == axum::http::StatusCode::BAD_REQUEST,
        "step forward on missing session should return error, got {}",
        status
    );
}

#[tokio::test]
async fn debug_step_backward_missing_session_returns_error() {
    let app = builders::test_router();
    let resp = app
        .oneshot(post_json_request(
            "/api/debug/sessions/nonexistent_id/step-backward",
            "{}",
        ))
        .await
        .unwrap();
    let status = resp.status();
    assert!(
        status == axum::http::StatusCode::NOT_FOUND
            || status == axum::http::StatusCode::BAD_REQUEST,
        "step backward on missing session should return error, got {}",
        status
    );
}

#[tokio::test]
async fn debug_seek_missing_session_returns_error() {
    let app = builders::test_router();
    let resp = app
        .oneshot(post_json_request(
            "/api/debug/sessions/nonexistent_id/seek",
            r#"{"tick": 50}"#,
        ))
        .await
        .unwrap();
    let status = resp.status();
    assert!(
        status == axum::http::StatusCode::NOT_FOUND
            || status == axum::http::StatusCode::BAD_REQUEST,
        "seek on missing session should return error, got {}",
        status
    );
}

#[tokio::test]
async fn debug_continue_missing_session_returns_error() {
    let app = builders::test_router();
    let resp = app
        .oneshot(post_json_request(
            "/api/debug/sessions/nonexistent_id/continue",
            "{}",
        ))
        .await
        .unwrap();
    let status = resp.status();
    assert!(
        status == axum::http::StatusCode::NOT_FOUND
            || status == axum::http::StatusCode::BAD_REQUEST,
        "continue on missing session should return error, got {}",
        status
    );
}

#[tokio::test]
async fn debug_pause_missing_session_returns_error() {
    let app = builders::test_router();
    let resp = app
        .oneshot(post_json_request(
            "/api/debug/sessions/nonexistent_id/pause",
            "{}",
        ))
        .await
        .unwrap();
    let status = resp.status();
    assert!(
        status == axum::http::StatusCode::NOT_FOUND
            || status == axum::http::StatusCode::BAD_REQUEST,
        "pause on missing session should return error, got {}",
        status
    );
}

#[tokio::test]
async fn debug_reset_missing_session_returns_error() {
    let app = builders::test_router();
    let resp = app
        .oneshot(post_json_request(
            "/api/debug/sessions/nonexistent_id/reset",
            "{}",
        ))
        .await
        .unwrap();
    let status = resp.status();
    assert!(
        status == axum::http::StatusCode::NOT_FOUND
            || status == axum::http::StatusCode::BAD_REQUEST,
        "reset on missing session should return error, got {}",
        status
    );
}

#[tokio::test]
async fn debug_snapshot_missing_session_returns_error() {
    let app = builders::test_router();
    let resp = app
        .oneshot(get_request("/api/debug/sessions/nonexistent_id/snapshot"))
        .await
        .unwrap();
    let status = resp.status();
    assert!(
        status == axum::http::StatusCode::NOT_FOUND
            || status == axum::http::StatusCode::BAD_REQUEST,
        "snapshot on missing session should return error, got {}",
        status
    );
}

#[tokio::test]
async fn debug_delete_missing_session_returns_error() {
    let app = builders::test_router();
    let resp = app
        .oneshot(delete_request("/api/debug/sessions/nonexistent_id"))
        .await
        .unwrap();
    let status = resp.status();
    assert!(
        status == axum::http::StatusCode::NOT_FOUND
            || status == axum::http::StatusCode::BAD_REQUEST,
        "deleting missing debug session should return error, got {}",
        status
    );
}

// ═══════════════════════════════════════════════════════════════════════════════
//  Breakpoints/watches on missing session
// ═══════════════════════════════════════════════════════════════════════════════

#[tokio::test]
async fn debug_add_breakpoint_missing_session_returns_error() {
    let app = builders::test_router();
    let resp = app
        .oneshot(post_json_request(
            "/api/debug/sessions/nonexistent_id/breakpoints",
            r#"{"breakpoint_type": "tick", "tick": 10}"#,
        ))
        .await
        .unwrap();
    let status = resp.status();
    assert!(
        status == axum::http::StatusCode::NOT_FOUND
            || status == axum::http::StatusCode::BAD_REQUEST,
        "adding breakpoint on missing session should return error, got {}",
        status
    );
}

#[tokio::test]
async fn debug_add_watch_missing_session_returns_error() {
    let app = builders::test_router();
    let resp = app
        .oneshot(post_json_request(
            "/api/debug/sessions/nonexistent_id/watches",
            r#"{"id": "w1", "name": "test", "topic": "imu", "watch_type": "raw"}"#,
        ))
        .await
        .unwrap();
    let status = resp.status();
    assert!(
        status == axum::http::StatusCode::NOT_FOUND
            || status == axum::http::StatusCode::BAD_REQUEST,
        "adding watch on missing session should return error, got {}",
        status
    );
}
