//! Factory functions and assertion helpers for monitor handler tests.
//!
//! These utilities make it easy to construct realistic test data and assert on
//! HTTP responses without repeating boilerplate in every test.

use horus_core::core::HealthStatus;
use horus_core::error::Severity;
use horus_core::scheduling::{BlackBoxEvent, BlackBoxRecord};
use horus_manager::discovery::{
    NodeStatus, ProcessCategory, SharedMemoryInfo, TopicInfo, TopicStatus,
};
use std::time::SystemTime;

// ─── NodeStatus factory ─────────────────────────────────────────────────────

/// Create a `NodeStatus` with the given parameters and sensible defaults for
/// everything else.
///
/// # Arguments
///
/// * `name`   — Node name (e.g., `"lidar_driver"`).
/// * `health` — Health status enum variant.
/// * `cpu`    — CPU usage percentage (e.g., `45.2`).
/// * `memory` — Memory usage in bytes (e.g., `50 * 1024 * 1024` for 50 MB).
pub fn make_test_node(name: &str, health: HealthStatus, cpu: f32, memory: u64) -> NodeStatus {
    NodeStatus {
        name: name.to_string(),
        status: "Running".to_string(),
        health,
        priority: 10,
        process_id: std::process::id(),
        command_line: format!("/usr/bin/{}", name),
        working_dir: "/tmp".to_string(),
        cpu_usage: cpu,
        memory_usage: memory,
        start_time: "2026-01-01T00:00:00Z".to_string(),
        scheduler_name: "default".to_string(),
        category: ProcessCategory::Node,
        tick_count: 1000,
        error_count: 0,
        actual_rate_hz: 30,
        publishers: vec![],
        subscribers: vec![],
    }
}

/// Extended node builder that also sets publishers, subscribers, and error
/// count.
pub fn make_test_node_full(
    name: &str,
    health: HealthStatus,
    cpu: f32,
    memory: u64,
    tick_count: u64,
    error_count: u32,
    publishers: Vec<(&str, &str)>,
    subscribers: Vec<(&str, &str)>,
) -> NodeStatus {
    let mut node = make_test_node(name, health, cpu, memory);
    node.tick_count = tick_count;
    node.error_count = error_count;
    node.publishers = publishers
        .into_iter()
        .map(|(topic, type_name)| TopicInfo {
            topic: topic.to_string(),
            type_name: type_name.to_string(),
        })
        .collect();
    node.subscribers = subscribers
        .into_iter()
        .map(|(topic, type_name)| TopicInfo {
            topic: topic.to_string(),
            type_name: type_name.to_string(),
        })
        .collect();
    node
}

// ─── SharedMemoryInfo factory ───────────────────────────────────────────────

/// Create a `SharedMemoryInfo` with the given parameters and sensible defaults.
///
/// # Arguments
///
/// * `name`          — Topic name (e.g., `"horus_camera_rgb"`).
/// * `size`          — Size in bytes.
/// * `active`        — Whether the topic is currently active.
/// * `num_processes` — Number of accessing processes (PIDs will be synthetic).
pub fn make_test_topic(
    name: &str,
    size: u64,
    active: bool,
    num_processes: usize,
) -> SharedMemoryInfo {
    SharedMemoryInfo {
        topic_name: name.to_string(),
        size_bytes: size,
        active,
        accessing_processes: (1..=num_processes as u32).collect(),
        last_modified: Some(SystemTime::now()),
        message_type: Some("SensorData".to_string()),
        publishers: vec!["node_a".to_string()],
        subscribers: vec!["node_b".to_string()],
        message_rate_hz: if active { 30.0 } else { 0.0 },
        status: if active {
            TopicStatus::Active
        } else {
            TopicStatus::Stale
        },
        age_string: if active {
            "2s ago".to_string()
        } else {
            "5m ago".to_string()
        },
    }
}

/// Extended topic builder with full control over all fields.
pub fn make_test_topic_full(
    name: &str,
    size: u64,
    active: bool,
    accessing_pids: Vec<u32>,
    message_type: Option<&str>,
    publishers: Vec<&str>,
    subscribers: Vec<&str>,
    rate_hz: f32,
    status: TopicStatus,
    age_string: &str,
) -> SharedMemoryInfo {
    SharedMemoryInfo {
        topic_name: name.to_string(),
        size_bytes: size,
        active,
        accessing_processes: accessing_pids,
        last_modified: Some(SystemTime::now()),
        message_type: message_type.map(|s| s.to_string()),
        publishers: publishers.into_iter().map(|s| s.to_string()).collect(),
        subscribers: subscribers.into_iter().map(|s| s.to_string()).collect(),
        message_rate_hz: rate_hz,
        status,
        age_string: age_string.to_string(),
    }
}

// ─── BlackBoxEvent factory ──────────────────────────────────────────────────

/// Create a `BlackBoxEvent` of the given type for testing.
///
/// # Arguments
///
/// * `event_type` — One of: `"tick"`, `"error"`, `"deadline_miss"`,
///   `"budget_violation"`, `"scheduler_start"`, `"scheduler_stop"`,
///   `"node_added"`, `"emergency_stop"`, `"custom"`.
/// * `node` — Node name associated with the event.
/// * `tick` — Tick number for the event's `BlackBoxRecord`.
///
/// Returns a `BlackBoxRecord` (event + timestamp + tick) ready for use.
pub fn make_test_blackbox_event(event_type: &str, node: &str, tick: u64) -> BlackBoxRecord {
    let event = match event_type {
        "tick" => BlackBoxEvent::NodeTick {
            name: node.to_string(),
            duration_us: 500,
            success: true,
        },
        "error" => BlackBoxEvent::NodeError {
            name: node.to_string(),
            error: "test error".to_string(),
            severity: Severity::Transient,
        },
        "deadline_miss" => BlackBoxEvent::DeadlineMiss {
            name: node.to_string(),
            deadline_us: 1000,
            actual_us: 2500,
        },
        "budget_violation" => BlackBoxEvent::BudgetViolation {
            name: node.to_string(),
            budget_us: 800,
            actual_us: 1200,
        },
        "scheduler_start" => BlackBoxEvent::SchedulerStart {
            name: node.to_string(),
            node_count: 5,
            config: "default".to_string(),
        },
        "scheduler_stop" => BlackBoxEvent::SchedulerStop {
            reason: "clean shutdown".to_string(),
            total_ticks: tick,
        },
        "node_added" => BlackBoxEvent::NodeAdded {
            name: node.to_string(),
            order: 1,
        },
        "emergency_stop" => BlackBoxEvent::EmergencyStop {
            reason: format!("test emergency from {}", node),
        },
        "custom" | _ => BlackBoxEvent::Custom {
            category: "test".to_string(),
            message: format!("custom event for {} at tick {}", node, tick),
        },
    };

    BlackBoxRecord {
        timestamp_us: tick * 1000, // synthetic timestamp
        tick,
        event,
    }
}

/// Create a batch of N `BlackBoxRecord` entries with sequential ticks,
/// alternating between tick and error events.
pub fn make_test_blackbox_batch(node: &str, count: usize) -> Vec<BlackBoxRecord> {
    (0..count)
        .map(|i| {
            let event_type = if i % 3 == 0 { "error" } else { "tick" };
            make_test_blackbox_event(event_type, node, i as u64)
        })
        .collect()
}

// ─── Response assertion helpers ─────────────────────────────────────────────

/// Assert that the response has status 200 and return the parsed JSON body.
///
/// Panics with a descriptive message if the status is not 200 or the body
/// cannot be parsed as JSON.
pub async fn assert_json_ok(response: axum::http::Response<axum::body::Body>) -> serde_json::Value {
    let status = response.status();
    let body = axum::body::to_bytes(response.into_body(), 2 * 1024 * 1024)
        .await
        .expect("reading response body must not fail");
    let body_str = String::from_utf8_lossy(&body);

    assert_eq!(
        status,
        axum::http::StatusCode::OK,
        "expected 200 OK, got {} — body: {}",
        status,
        body_str
    );

    serde_json::from_slice(&body).unwrap_or_else(|e| {
        panic!(
            "response body is not valid JSON: {} — body: {}",
            e, body_str
        )
    })
}

/// Assert that the response has the given error status code and return the
/// parsed JSON body.
///
/// Panics if the status does not match or the body is not valid JSON.
pub async fn assert_json_error(
    response: axum::http::Response<axum::body::Body>,
    expected_status: axum::http::StatusCode,
) -> serde_json::Value {
    let status = response.status();
    let body = axum::body::to_bytes(response.into_body(), 2 * 1024 * 1024)
        .await
        .expect("reading response body must not fail");
    let body_str = String::from_utf8_lossy(&body);

    assert_eq!(
        status, expected_status,
        "expected {}, got {} — body: {}",
        expected_status, status, body_str
    );

    serde_json::from_slice(&body).unwrap_or_else(|e| {
        panic!(
            "response body is not valid JSON: {} — body: {}",
            e, body_str
        )
    })
}

/// Assert that a JSON value contains the expected key and that its value is
/// non-null.
pub fn assert_json_has_key(json: &serde_json::Value, key: &str) {
    assert!(
        json.get(key).is_some() && !json[key].is_null(),
        "expected JSON to have non-null key '{}', got: {}",
        key,
        serde_json::to_string_pretty(json).unwrap_or_default()
    );
}

/// Assert that a JSON value contains an array at `key` with the expected
/// length.
pub fn assert_json_array_len(json: &serde_json::Value, key: &str, expected_len: usize) {
    let arr = json[key]
        .as_array()
        .unwrap_or_else(|| panic!("expected '{}' to be an array, got: {:?}", key, json[key]));
    assert_eq!(
        arr.len(),
        expected_len,
        "'{}' array length mismatch: expected {}, got {}",
        key,
        expected_len,
        arr.len()
    );
}

// ─── Request builder helpers ────────────────────────────────────────────────

/// Build a GET request to the given URI with no headers.
pub fn get_request(uri: &str) -> axum::http::Request<axum::body::Body> {
    axum::http::Request::builder()
        .method("GET")
        .uri(uri)
        .body(axum::body::Body::empty())
        .expect("building GET request must not fail")
}

/// Build an authenticated GET request with a session token cookie.
pub fn get_request_authed(uri: &str, session_token: &str) -> axum::http::Request<axum::body::Body> {
    axum::http::Request::builder()
        .method("GET")
        .uri(uri)
        .header("Cookie", format!("session_token={}", session_token))
        .body(axum::body::Body::empty())
        .expect("building authed GET request must not fail")
}

/// Build a POST request with JSON body.
pub fn post_json_request(uri: &str, json_body: &str) -> axum::http::Request<axum::body::Body> {
    axum::http::Request::builder()
        .method("POST")
        .uri(uri)
        .header("Content-Type", "application/json")
        .body(axum::body::Body::from(json_body.to_string()))
        .expect("building POST request must not fail")
}

/// Build an authenticated POST request with JSON body, session token, and CSRF
/// token.
pub fn post_json_request_authed(
    uri: &str,
    json_body: &str,
    session_token: &str,
    csrf_token: &str,
) -> axum::http::Request<axum::body::Body> {
    axum::http::Request::builder()
        .method("POST")
        .uri(uri)
        .header("Content-Type", "application/json")
        .header("Cookie", format!("session_token={}", session_token))
        .header("X-CSRF-Token", csrf_token)
        .body(axum::body::Body::from(json_body.to_string()))
        .expect("building authed POST request must not fail")
}

/// Build a DELETE request to the given URI.
pub fn delete_request(uri: &str) -> axum::http::Request<axum::body::Body> {
    axum::http::Request::builder()
        .method("DELETE")
        .uri(uri)
        .body(axum::body::Body::empty())
        .expect("building DELETE request must not fail")
}

/// Build an authenticated DELETE request with session token and CSRF token.
pub fn delete_request_authed(
    uri: &str,
    session_token: &str,
    csrf_token: &str,
) -> axum::http::Request<axum::body::Body> {
    axum::http::Request::builder()
        .method("DELETE")
        .uri(uri)
        .header("Cookie", format!("session_token={}", session_token))
        .header("X-CSRF-Token", csrf_token)
        .body(axum::body::Body::empty())
        .expect("building authed DELETE request must not fail")
}
