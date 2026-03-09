//! UAT: Log streaming E2E.
//!
//! Verifies the monitor correctly streams log entries from nodes via the
//! global log buffer, filters by node name and topic, and handles high volume.

mod harness;
mod monitor_tests;

use harness::HorusTestRuntime;
use monitor_tests::builders;
use monitor_tests::helpers::{assert_json_ok, get_request};

use horus_core::core::log_buffer::{LogEntry, LogType, GLOBAL_LOG_BUFFER};
use tower::ServiceExt;

/// Push a test log entry.
fn push_log(node: &str, log_type: LogType, message: &str, topic: Option<&str>) {
    GLOBAL_LOG_BUFFER.push(LogEntry {
        timestamp: chrono::Utc::now().to_rfc3339(),
        tick_number: 0,
        node_name: node.to_string(),
        log_type,
        topic: topic.map(|t| t.to_string()),
        message: message.to_string(),
        tick_us: 0,
        ipc_ns: 0,
    });
}

// ═══════════════════════════════════════════════════════════════════════════════
//  GET /api/logs/all
// ═══════════════════════════════════════════════════════════════════════════════

#[tokio::test]
async fn logs_all_returns_valid_json() {
    let app = builders::test_router();
    let resp = app.oneshot(get_request("/api/logs/all")).await.unwrap();
    let json = assert_json_ok(resp).await;

    assert!(json["logs"].is_array(), "logs must be an array");
}

#[tokio::test]
async fn logs_all_contains_injected_entries() {
    let _rt = HorusTestRuntime::new();
    push_log("uat_log_nodeA", LogType::Info, "test info message", None);
    push_log(
        "uat_log_nodeB",
        LogType::Error,
        "test error message",
        None,
    );
    push_log(
        "uat_log_nodeC",
        LogType::Warning,
        "test warning message",
        None,
    );

    let app = builders::test_router();
    let resp = app.oneshot(get_request("/api/logs/all")).await.unwrap();
    let json = assert_json_ok(resp).await;

    let logs = json["logs"].as_array().unwrap();

    // Our injected entries should be present (buffer may also have entries from other tests)
    let has_info = logs
        .iter()
        .any(|l| l["message"].as_str() == Some("test info message"));
    let has_error = logs
        .iter()
        .any(|l| l["message"].as_str() == Some("test error message"));
    let has_warning = logs
        .iter()
        .any(|l| l["message"].as_str() == Some("test warning message"));

    assert!(has_info, "info log must appear in /api/logs/all");
    assert!(has_error, "error log must appear in /api/logs/all");
    assert!(has_warning, "warning log must appear in /api/logs/all");
}

#[tokio::test]
async fn logs_have_correct_fields() {
    push_log(
        "uat_log_fields",
        LogType::Info,
        "field check message",
        None,
    );

    let app = builders::test_router();
    let resp = app.oneshot(get_request("/api/logs/all")).await.unwrap();
    let json = assert_json_ok(resp).await;

    let logs = json["logs"].as_array().unwrap();
    let entry = logs
        .iter()
        .find(|l| l["message"].as_str() == Some("field check message"));
    assert!(entry.is_some(), "injected log must be found");
    let entry = entry.unwrap();

    // Verify required fields
    assert!(
        entry["timestamp"].is_string(),
        "log must have string timestamp"
    );
    assert!(
        entry["node_name"].is_string(),
        "log must have string node_name"
    );
    assert_eq!(entry["node_name"], "uat_log_fields");
}

// ═══════════════════════════════════════════════════════════════════════════════
//  GET /api/logs/node/:name — filtered by node
// ═══════════════════════════════════════════════════════════════════════════════

#[tokio::test]
async fn logs_node_filter_returns_only_matching() {
    push_log(
        "uat_log_sensor",
        LogType::Info,
        "sensor reading 42",
        None,
    );
    push_log(
        "uat_log_motor",
        LogType::Info,
        "motor command applied",
        None,
    );
    push_log(
        "uat_log_sensor",
        LogType::Warning,
        "sensor drift detected",
        None,
    );

    let app = builders::test_router();
    let resp = app
        .oneshot(get_request("/api/logs/node/uat_log_sensor"))
        .await
        .unwrap();
    let json = assert_json_ok(resp).await;

    assert_eq!(json["node"], "uat_log_sensor");
    let logs = json["logs"].as_array().unwrap();

    // All returned logs must be from uat_log_sensor
    for log in logs {
        assert_eq!(
            log["node_name"].as_str(),
            Some("uat_log_sensor"),
            "filtered logs must only contain sensor node entries"
        );
    }
    assert!(logs.len() >= 2, "should have at least 2 sensor logs");
}

#[tokio::test]
async fn logs_node_unknown_returns_empty() {
    let app = builders::test_router();
    let resp = app
        .oneshot(get_request("/api/logs/node/nonexistent_node_xyz"))
        .await
        .unwrap();
    let json = assert_json_ok(resp).await;

    let logs = json["logs"].as_array().unwrap();
    assert!(logs.is_empty(), "unknown node should return empty logs");
}

// ═══════════════════════════════════════════════════════════════════════════════
//  GET /api/logs/topic/:name — filtered by topic
// ═══════════════════════════════════════════════════════════════════════════════

#[tokio::test]
async fn logs_topic_filter_returns_matching() {
    push_log(
        "uat_log_pub",
        LogType::Publish,
        "published frame",
        Some("camera.rgb"),
    );
    push_log(
        "uat_log_sub",
        LogType::Subscribe,
        "received frame",
        Some("camera.rgb"),
    );
    push_log(
        "uat_log_pub",
        LogType::Publish,
        "published scan",
        Some("lidar.scan"),
    );

    let app = builders::test_router();
    let resp = app
        .oneshot(get_request("/api/logs/topic/camera.rgb"))
        .await
        .unwrap();
    let json = assert_json_ok(resp).await;

    assert_eq!(json["topic"], "camera.rgb");
    let logs = json["logs"].as_array().unwrap();

    // All returned logs must have topic "camera.rgb"
    for log in logs {
        assert_eq!(
            log["topic"].as_str(),
            Some("camera.rgb"),
            "filtered logs must only contain camera.rgb topic entries"
        );
    }
    assert!(logs.len() >= 2, "should have at least 2 camera.rgb logs");
}

// ═══════════════════════════════════════════════════════════════════════════════
//  Log severity classification
// ═══════════════════════════════════════════════════════════════════════════════

#[tokio::test]
async fn log_severity_correctly_classified() {
    push_log(
        "uat_log_sev",
        LogType::Info,
        "sev_info_marker",
        None,
    );
    push_log(
        "uat_log_sev",
        LogType::Warning,
        "sev_warn_marker",
        None,
    );
    push_log(
        "uat_log_sev",
        LogType::Error,
        "sev_error_marker",
        None,
    );

    let app = builders::test_router();
    let resp = app
        .oneshot(get_request("/api/logs/node/uat_log_sev"))
        .await
        .unwrap();
    let json = assert_json_ok(resp).await;

    let logs = json["logs"].as_array().unwrap();

    let info = logs
        .iter()
        .find(|l| l["message"].as_str() == Some("sev_info_marker"));
    let warn = logs
        .iter()
        .find(|l| l["message"].as_str() == Some("sev_warn_marker"));
    let error = logs
        .iter()
        .find(|l| l["message"].as_str() == Some("sev_error_marker"));

    assert!(info.is_some(), "info log must be present");
    assert!(warn.is_some(), "warning log must be present");
    assert!(error.is_some(), "error log must be present");

    // Verify log_type field reflects severity
    assert_eq!(info.unwrap()["log_type"], "Info");
    assert_eq!(warn.unwrap()["log_type"], "Warning");
    assert_eq!(error.unwrap()["log_type"], "Error");
}

// ═══════════════════════════════════════════════════════════════════════════════
//  High volume — latest entries always present
// ═══════════════════════════════════════════════════════════════════════════════

#[tokio::test]
async fn high_volume_latest_entries_present() {
    // Push many log entries
    for i in 0..200 {
        push_log(
            "uat_log_volume",
            LogType::Info,
            &format!("volume_entry_{}", i),
            None,
        );
    }

    let app = builders::test_router();
    let resp = app
        .oneshot(get_request("/api/logs/node/uat_log_volume"))
        .await
        .unwrap();
    let json = assert_json_ok(resp).await;

    let logs = json["logs"].as_array().unwrap();
    assert!(
        !logs.is_empty(),
        "should have some logs after high volume injection"
    );

    // The most recent entry should be present (buffer may wrap but keeps newest)
    let has_latest = logs
        .iter()
        .any(|l| l["message"].as_str() == Some("volume_entry_199"));
    assert!(
        has_latest,
        "latest entry (volume_entry_199) must be present even after high volume"
    );
}
