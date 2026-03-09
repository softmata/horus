//! UAT: Blackbox anomaly detection E2E.
//!
//! Verifies the blackbox correctly captures anomaly events (deadline misses,
//! node errors, circuit breaker trips) and that the API filters them properly.
//!
//! These tests write directly to the blackbox WAL file that the handler reads,
//! then verify via the HTTP API.

mod harness;
mod monitor_tests;

use monitor_tests::builders;
use monitor_tests::helpers::{assert_json_ok, delete_request, get_request};

use horus_core::error::Severity;
use horus_core::scheduling::{BlackBoxEvent, BlackBoxRecord};

use std::io::Write;
use tower::ServiceExt;

/// Helper: get the blackbox WAL path (same logic as handler).
fn blackbox_wal_path() -> std::path::PathBuf {
    let local = std::path::Path::new(".horus/blackbox");
    if local.is_dir() {
        return local.join("blackbox.wal");
    }
    dirs::home_dir()
        .expect("home dir must exist")
        .join(".horus/blackbox/blackbox.wal")
}

/// Helper: write events to the blackbox WAL.
fn write_wal_events(events: &[BlackBoxRecord]) {
    let wal_path = blackbox_wal_path();
    std::fs::create_dir_all(wal_path.parent().unwrap()).expect("create blackbox dir");

    let mut file = std::fs::OpenOptions::new()
        .create(true)
        .write(true)
        .truncate(true)
        .open(&wal_path)
        .expect("open WAL for writing");

    for record in events {
        let json = serde_json::to_string(record).expect("serialize record");
        writeln!(file, "{}", json).expect("write WAL line");
    }
}

/// Helper: remove the WAL file.
fn remove_wal() {
    let wal_path = blackbox_wal_path();
    let _ = std::fs::remove_file(&wal_path);
}

/// Build a set of test events including normal ticks and anomalies.
fn build_test_events() -> Vec<BlackBoxRecord> {
    vec![
        // Normal tick events (should NOT appear in anomalies)
        BlackBoxRecord {
            timestamp_us: 1000,
            tick: 1,
            event: BlackBoxEvent::NodeTick {
                name: "sensor_node".to_string(),
                duration_us: 500,
                success: true,
            },
        },
        BlackBoxRecord {
            timestamp_us: 2000,
            tick: 2,
            event: BlackBoxEvent::NodeTick {
                name: "motor_node".to_string(),
                duration_us: 300,
                success: true,
            },
        },
        // Deadline miss anomaly
        BlackBoxRecord {
            timestamp_us: 3000,
            tick: 3,
            event: BlackBoxEvent::DeadlineMiss {
                name: "slow_node".to_string(),
                deadline_us: 1000,
                actual_us: 2500,
            },
        },
        // Node error anomaly
        BlackBoxRecord {
            timestamp_us: 4000,
            tick: 4,
            event: BlackBoxEvent::NodeError {
                name: "flaky_node".to_string(),
                error: "segmentation fault".to_string(),
                severity: Severity::Permanent,
            },
        },
        // Circuit breaker anomaly
        BlackBoxRecord {
            timestamp_us: 5000,
            tick: 5,
            event: BlackBoxEvent::CircuitBreakerChange {
                name: "overloaded_node".to_string(),
                new_state: "Open".to_string(),
                failure_count: 5,
            },
        },
        // More normal ticks
        BlackBoxRecord {
            timestamp_us: 6000,
            tick: 6,
            event: BlackBoxEvent::NodeTick {
                name: "sensor_node".to_string(),
                duration_us: 450,
                success: true,
            },
        },
        // budget violation anomaly
        BlackBoxRecord {
            timestamp_us: 7000,
            tick: 7,
            event: BlackBoxEvent::BudgetViolation {
                name: "slow_node".to_string(),
                budget_us: 800,
                actual_us: 1600,
            },
        },
        // Scheduler start (not an anomaly)
        BlackBoxRecord {
            timestamp_us: 8000,
            tick: 8,
            event: BlackBoxEvent::SchedulerStart {
                name: "main_sched".to_string(),
                node_count: 4,
                config: "default".to_string(),
            },
        },
        // Another node error from a different node
        BlackBoxRecord {
            timestamp_us: 9000,
            tick: 9,
            event: BlackBoxEvent::NodeError {
                name: "sensor_node".to_string(),
                error: "timeout reading device".to_string(),
                severity: Severity::Transient,
            },
        },
    ]
}

// ═══════════════════════════════════════════════════════════════════════════════
//  GET /api/blackbox — full event listing
// ═══════════════════════════════════════════════════════════════════════════════

#[tokio::test]
async fn blackbox_list_returns_all_events() {
    let events = build_test_events();
    write_wal_events(&events);

    let app = builders::test_router();
    let resp = app.oneshot(get_request("/api/blackbox")).await.unwrap();
    let json = assert_json_ok(resp).await;

    let count = json["count"].as_u64().expect("count must be a number");
    assert_eq!(count, 9, "should return all 9 events");

    let events_arr = json["events"].as_array().unwrap();
    assert_eq!(events_arr.len(), 9);

    remove_wal();
}

#[tokio::test]
async fn blackbox_captures_deadline_miss() {
    let events = build_test_events();
    write_wal_events(&events);

    let app = builders::test_router();
    let resp = app
        .oneshot(get_request("/api/blackbox?event=DeadlineMiss"))
        .await
        .unwrap();
    let json = assert_json_ok(resp).await;

    let events_arr = json["events"].as_array().unwrap();
    assert_eq!(events_arr.len(), 1, "should have exactly 1 DeadlineMiss");
    assert_eq!(events_arr[0]["tick"], 3);

    // Verify the event contains the correct fields
    let event = &events_arr[0]["event"];
    assert!(
        event.to_string().contains("slow_node"),
        "DeadlineMiss must reference slow_node"
    );

    remove_wal();
}

#[tokio::test]
async fn blackbox_captures_node_errors() {
    let events = build_test_events();
    write_wal_events(&events);

    let app = builders::test_router();
    let resp = app
        .oneshot(get_request("/api/blackbox?event=NodeError"))
        .await
        .unwrap();
    let json = assert_json_ok(resp).await;

    let events_arr = json["events"].as_array().unwrap();
    assert_eq!(events_arr.len(), 2, "should have 2 NodeError events");

    remove_wal();
}

#[tokio::test]
async fn blackbox_captures_circuit_breaker() {
    let events = build_test_events();
    write_wal_events(&events);

    let app = builders::test_router();
    let resp = app
        .oneshot(get_request("/api/blackbox?event=CircuitBreakerChange"))
        .await
        .unwrap();
    let json = assert_json_ok(resp).await;

    let events_arr = json["events"].as_array().unwrap();
    assert_eq!(
        events_arr.len(),
        1,
        "should have exactly 1 CircuitBreakerChange"
    );
    assert_eq!(events_arr[0]["tick"], 5);

    let event_str = events_arr[0]["event"].to_string();
    assert!(event_str.contains("overloaded_node"));
    assert!(event_str.contains("Open"));

    remove_wal();
}

// ═══════════════════════════════════════════════════════════════════════════════
//  GET /api/blackbox/anomalies — anomalies-only filter
// ═══════════════════════════════════════════════════════════════════════════════

#[tokio::test]
async fn anomalies_endpoint_filters_out_normal_events() {
    let events = build_test_events();
    write_wal_events(&events);

    let app = builders::test_router();
    let resp = app
        .oneshot(get_request("/api/blackbox/anomalies"))
        .await
        .unwrap();
    let json = assert_json_ok(resp).await;

    let anomalies = json["anomalies"].as_array().unwrap();
    // Should have: 1 DeadlineMiss + 2 NodeError + 1 CircuitBreakerChange + 1 BudgetViolation = 5
    assert_eq!(
        anomalies.len(),
        5,
        "anomalies should contain exactly 5 anomaly events, got {}",
        anomalies.len()
    );

    // Verify none of them are NodeTick or SchedulerStart
    for anomaly in anomalies {
        let event_str = anomaly["event"].to_string();
        assert!(
            !event_str.contains("NodeTick"),
            "NodeTick should not appear in anomalies"
        );
        assert!(
            !event_str.contains("SchedulerStart"),
            "SchedulerStart should not appear in anomalies"
        );
    }

    remove_wal();
}

#[tokio::test]
async fn anomalies_count_matches_array_length() {
    let events = build_test_events();
    write_wal_events(&events);

    let app = builders::test_router();
    let resp = app
        .oneshot(get_request("/api/blackbox/anomalies"))
        .await
        .unwrap();
    let json = assert_json_ok(resp).await;

    let anomalies = json["anomalies"].as_array().unwrap();
    let count = json["count"].as_u64().unwrap();
    assert_eq!(
        anomalies.len() as u64,
        count,
        "count must match anomalies array length"
    );

    remove_wal();
}

// ═══════════════════════════════════════════════════════════════════════════════
//  Filtering by node name
// ═══════════════════════════════════════════════════════════════════════════════

#[tokio::test]
async fn filter_by_node_returns_only_that_nodes_events() {
    let events = build_test_events();
    write_wal_events(&events);

    let app = builders::test_router();
    let resp = app
        .oneshot(get_request("/api/blackbox?node=slow_node"))
        .await
        .unwrap();
    let json = assert_json_ok(resp).await;

    let events_arr = json["events"].as_array().unwrap();
    // slow_node has: 1 DeadlineMiss (tick 3) + 1 BudgetViolation (tick 7) = 2
    assert_eq!(
        events_arr.len(),
        2,
        "slow_node should have 2 events, got {}",
        events_arr.len()
    );

    for event in events_arr {
        let event_str = event["event"].to_string();
        assert!(
            event_str.contains("slow_node"),
            "all filtered events must be from slow_node"
        );
    }

    remove_wal();
}

#[tokio::test]
async fn filter_by_node_sensor_returns_correct_events() {
    let events = build_test_events();
    write_wal_events(&events);

    let app = builders::test_router();
    let resp = app
        .oneshot(get_request("/api/blackbox?node=sensor_node"))
        .await
        .unwrap();
    let json = assert_json_ok(resp).await;

    let events_arr = json["events"].as_array().unwrap();
    // sensor_node: tick 1 (NodeTick), tick 6 (NodeTick), tick 9 (NodeError) = 3
    assert_eq!(events_arr.len(), 3);

    remove_wal();
}

// ═══════════════════════════════════════════════════════════════════════════════
//  Filtering by tick range
// ═══════════════════════════════════════════════════════════════════════════════

#[tokio::test]
async fn filter_by_tick_range_returns_correct_window() {
    let events = build_test_events();
    write_wal_events(&events);

    let app = builders::test_router();
    let resp = app
        .oneshot(get_request("/api/blackbox?tick=3-5"))
        .await
        .unwrap();
    let json = assert_json_ok(resp).await;

    let events_arr = json["events"].as_array().unwrap();
    // Ticks 3, 4, 5 = 3 events
    assert_eq!(events_arr.len(), 3, "tick range 3-5 should return 3 events");
    for event in events_arr {
        let tick = event["tick"].as_u64().unwrap();
        assert!(tick >= 3 && tick <= 5, "tick {} out of range 3-5", tick);
    }

    remove_wal();
}

// ═══════════════════════════════════════════════════════════════════════════════
//  DELETE /api/blackbox — clear all events
// ═══════════════════════════════════════════════════════════════════════════════

#[tokio::test]
async fn clear_endpoint_removes_all_events() {
    let events = build_test_events();
    write_wal_events(&events);

    // Verify events exist first
    let app = builders::test_router();
    let resp = app.oneshot(get_request("/api/blackbox")).await.unwrap();
    let json = assert_json_ok(resp).await;
    assert!(json["count"].as_u64().unwrap() > 0, "events should exist before clear");

    // Clear
    let app = builders::test_router();
    let resp = app.oneshot(delete_request("/api/blackbox")).await.unwrap();
    let json = assert_json_ok(resp).await;
    assert_eq!(json["cleared"], true);

    // Verify empty after clear
    let app = builders::test_router();
    let resp = app.oneshot(get_request("/api/blackbox")).await.unwrap();
    let json = assert_json_ok(resp).await;
    let count = json["count"].as_u64().unwrap_or(0);
    assert_eq!(count, 0, "events should be empty after clear");

    // No need to remove_wal — clear already did it
}

// ═══════════════════════════════════════════════════════════════════════════════
//  WAL persistence
// ═══════════════════════════════════════════════════════════════════════════════

#[tokio::test]
async fn wal_file_persists_and_is_readable() {
    let events = build_test_events();
    write_wal_events(&events);

    // Verify the WAL file exists on disk
    let wal_path = blackbox_wal_path();
    assert!(wal_path.exists(), "WAL file should exist on disk");
    let content = std::fs::read_to_string(&wal_path).unwrap();
    let lines: Vec<&str> = content.lines().collect();
    assert_eq!(lines.len(), 9, "WAL should have 9 lines");

    // Verify each line is valid JSON
    for (i, line) in lines.iter().enumerate() {
        let parsed: Result<BlackBoxRecord, _> = serde_json::from_str(line);
        assert!(
            parsed.is_ok(),
            "WAL line {} is not valid BlackBoxRecord JSON: {}",
            i,
            line
        );
    }

    // Read via API (simulating "restart" — handler re-reads from file)
    let app = builders::test_router();
    let resp = app.oneshot(get_request("/api/blackbox")).await.unwrap();
    let json = assert_json_ok(resp).await;
    assert_eq!(json["count"].as_u64().unwrap(), 9);

    remove_wal();
}

#[tokio::test]
async fn wal_grows_with_appended_events() {
    // Write initial events
    let mut events = build_test_events();
    write_wal_events(&events);

    let wal_path = blackbox_wal_path();
    let initial_size = std::fs::metadata(&wal_path).unwrap().len();

    // Append more events
    let new_event = BlackBoxRecord {
        timestamp_us: 10000,
        tick: 10,
        event: BlackBoxEvent::EmergencyStop {
            reason: "test emergency".to_string(),
        },
    };
    events.push(new_event);
    write_wal_events(&events);

    let new_size = std::fs::metadata(&wal_path).unwrap().len();
    assert!(
        new_size > initial_size,
        "WAL should grow after appending events"
    );

    // Verify the new event is visible via API
    let app = builders::test_router();
    let resp = app.oneshot(get_request("/api/blackbox")).await.unwrap();
    let json = assert_json_ok(resp).await;
    assert_eq!(json["count"].as_u64().unwrap(), 10);

    remove_wal();
}

// ═══════════════════════════════════════════════════════════════════════════════
//  Combined filters
// ═══════════════════════════════════════════════════════════════════════════════

#[tokio::test]
async fn combined_node_and_event_filter() {
    let events = build_test_events();
    write_wal_events(&events);

    let app = builders::test_router();
    let resp = app
        .oneshot(get_request(
            "/api/blackbox?node=sensor_node&event=NodeError",
        ))
        .await
        .unwrap();
    let json = assert_json_ok(resp).await;

    let events_arr = json["events"].as_array().unwrap();
    // sensor_node has 1 NodeError at tick 9
    assert_eq!(events_arr.len(), 1);
    assert_eq!(events_arr[0]["tick"], 9);

    remove_wal();
}

#[tokio::test]
async fn limit_parameter_caps_results() {
    let events = build_test_events();
    write_wal_events(&events);

    let app = builders::test_router();
    let resp = app
        .oneshot(get_request("/api/blackbox?limit=3"))
        .await
        .unwrap();
    let json = assert_json_ok(resp).await;

    let events_arr = json["events"].as_array().unwrap();
    assert_eq!(events_arr.len(), 3, "limit=3 should return exactly 3 events");

    remove_wal();
}
