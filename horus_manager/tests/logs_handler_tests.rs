//! Comprehensive unit tests for the logs handler endpoints.
//!
//! Tests all three logs handlers:
//! - `GET /api/logs/all`     — returns all entries from GLOBAL_LOG_BUFFER
//! - `GET /api/logs/node/:name` — returns entries filtered by node name
//! - `GET /api/logs/topic/:name` — strips "horus_" prefix, filters by topic
//!
//! Each test uses a unique identifier (PID + test name) in log messages and
//! node/topic names to avoid collision with parallel tests that share the
//! global ring buffer.
#![cfg(feature = "monitor")]

mod harness;
mod monitor_tests;

use harness::HorusTestRuntime;
use monitor_tests::builders;
use monitor_tests::helpers::{assert_json_ok, get_request};

use horus_core::core::log_buffer::{LogEntry as CoreLogEntry, LogType, GLOBAL_LOG_BUFFER};
use tower::ServiceExt;

// ─── Helpers ─────────────────────────────────────────────────────────────────

/// Create a unique tag incorporating the PID and a caller-supplied suffix.
/// This prevents cross-test collisions in the shared GLOBAL_LOG_BUFFER.
fn uid(suffix: &str) -> String {
    format!("logtest_{}_{}", std::process::id(), suffix)
}

/// Push a log entry with full control over all fields.
fn push_log(node: &str, log_type: LogType, topic: Option<&str>, message: &str, tick_number: u64) {
    let entry = CoreLogEntry {
        timestamp: chrono::Utc::now().to_rfc3339(),
        tick_number,
        node_name: node.to_string(),
        log_type,
        topic: topic.map(|t| t.to_string()),
        message: message.to_string(),
        tick_us: tick_number * 100,
        ipc_ns: tick_number * 50,
    };
    GLOBAL_LOG_BUFFER.push(entry);
}

// ═══════════════════════════════════════════════════════════════════════════════
//  1. EMPTY BUFFER — all three endpoints
// ═══════════════════════════════════════════════════════════════════════════════

#[tokio::test]
async fn logs_all_empty_buffer_returns_logs_array() {
    let app = builders::test_router();
    let resp = app.oneshot(get_request("/api/logs/all")).await.unwrap();
    let json = assert_json_ok(resp).await;

    assert!(
        json["logs"].is_array(),
        "response must contain a 'logs' array, got: {:?}",
        json
    );
}

#[tokio::test]
async fn logs_node_empty_buffer_returns_empty_array() {
    let node = uid("empty_node");
    let app = builders::test_router();
    let resp = app
        .oneshot(get_request(&format!("/api/logs/node/{}", node)))
        .await
        .unwrap();
    let json = assert_json_ok(resp).await;

    assert!(json["logs"].is_array(), "logs must be an array");
    // A never-seen node should return an empty array.
    let logs = json["logs"].as_array().unwrap();
    let matching: Vec<_> = logs
        .iter()
        .filter(|e| e["node_name"].as_str() == Some(node.as_str()))
        .collect();
    assert!(
        matching.is_empty(),
        "no logs should match an invented node name"
    );
}

#[tokio::test]
async fn logs_topic_empty_buffer_returns_empty_array() {
    let topic = uid("empty_topic");
    let app = builders::test_router();
    let resp = app
        .oneshot(get_request(&format!("/api/logs/topic/{}", topic)))
        .await
        .unwrap();
    let json = assert_json_ok(resp).await;

    assert!(json["logs"].is_array(), "logs must be an array");
    let logs = json["logs"].as_array().unwrap();
    let matching: Vec<_> = logs
        .iter()
        .filter(|e| e["topic"].as_str().map(|t| t == topic).unwrap_or(false))
        .collect();
    assert!(
        matching.is_empty(),
        "no logs should match an invented topic name"
    );
}

// ═══════════════════════════════════════════════════════════════════════════════
//  2. logs_all after injecting multiple logs from different nodes
// ═══════════════════════════════════════════════════════════════════════════════

#[tokio::test]
async fn logs_all_returns_injected_entries_from_multiple_nodes() {
    let node_a = uid("multi_a");
    let node_b = uid("multi_b");
    let msg_a = uid("msg_multi_a");
    let msg_b = uid("msg_multi_b");

    push_log(&node_a, LogType::Info, None, &msg_a, 10);
    push_log(&node_b, LogType::Warning, None, &msg_b, 11);

    let app = builders::test_router();
    let resp = app.oneshot(get_request("/api/logs/all")).await.unwrap();
    let json = assert_json_ok(resp).await;
    let logs = json["logs"].as_array().expect("logs must be an array");

    let found_a = logs.iter().any(|e| {
        e["message"]
            .as_str()
            .map(|m| m.contains(&msg_a))
            .unwrap_or(false)
    });
    let found_b = logs.iter().any(|e| {
        e["message"]
            .as_str()
            .map(|m| m.contains(&msg_b))
            .unwrap_or(false)
    });

    assert!(found_a, "log from node_a ('{}') must appear", msg_a);
    assert!(found_b, "log from node_b ('{}') must appear", msg_b);
}

// ═══════════════════════════════════════════════════════════════════════════════
//  3. logs_node filters correctly — only matching node returned
// ═══════════════════════════════════════════════════════════════════════════════

#[tokio::test]
async fn logs_node_filters_only_matching_node() {
    let node_yes = uid("filter_yes");
    let node_no = uid("filter_no");
    let msg_yes = uid("msg_yes");
    let msg_no = uid("msg_no");

    push_log(&node_yes, LogType::Info, None, &msg_yes, 20);
    push_log(&node_no, LogType::Error, None, &msg_no, 21);

    let app = builders::test_router();
    let resp = app
        .oneshot(get_request(&format!("/api/logs/node/{}", node_yes)))
        .await
        .unwrap();
    let json = assert_json_ok(resp).await;
    let logs = json["logs"].as_array().expect("logs must be an array");

    // Every returned entry must belong to node_yes.
    for entry in logs {
        assert_eq!(
            entry["node_name"].as_str().unwrap_or(""),
            node_yes,
            "all entries must have node_name == '{}'",
            node_yes
        );
    }

    // Our specific message for node_yes should be present.
    let found_yes = logs
        .iter()
        .any(|e| e["message"].as_str().unwrap_or("").contains(&msg_yes));
    assert!(found_yes, "log for node_yes must appear in filtered result");

    // node_no's message must NOT appear.
    let found_no = logs
        .iter()
        .any(|e| e["message"].as_str().unwrap_or("").contains(&msg_no));
    assert!(
        !found_no,
        "log for node_no must NOT appear in node_yes filter"
    );
}

// ═══════════════════════════════════════════════════════════════════════════════
//  4. logs_node with nonexistent node returns empty array
// ═══════════════════════════════════════════════════════════════════════════════

#[tokio::test]
async fn logs_node_nonexistent_returns_empty() {
    let node = uid("nonexistent_node_xyz_42");
    let app = builders::test_router();
    let resp = app
        .oneshot(get_request(&format!("/api/logs/node/{}", node)))
        .await
        .unwrap();
    let json = assert_json_ok(resp).await;

    let logs = json["logs"].as_array().expect("logs must be an array");
    let matching: Vec<_> = logs
        .iter()
        .filter(|e| e["node_name"].as_str() == Some(node.as_str()))
        .collect();
    assert!(
        matching.is_empty(),
        "nonexistent node must return zero matching logs"
    );
}

// ═══════════════════════════════════════════════════════════════════════════════
//  5. logs_topic filters by topic field
// ═══════════════════════════════════════════════════════════════════════════════

#[tokio::test]
async fn logs_topic_filters_by_topic_field() {
    let topic = uid("camera_rgb");
    let other_topic = uid("lidar_scan");
    let msg_match = uid("topic_match_msg");
    let msg_other = uid("topic_other_msg");
    let node = uid("topic_filter_node");

    push_log(&node, LogType::Publish, Some(&topic), &msg_match, 30);
    push_log(
        &node,
        LogType::Subscribe,
        Some(&other_topic),
        &msg_other,
        31,
    );

    let app = builders::test_router();
    let resp = app
        .oneshot(get_request(&format!("/api/logs/topic/{}", topic)))
        .await
        .unwrap();
    let json = assert_json_ok(resp).await;
    let logs = json["logs"].as_array().expect("logs must be an array");

    let found_match = logs
        .iter()
        .any(|e| e["message"].as_str().unwrap_or("").contains(&msg_match));
    assert!(found_match, "log with matching topic must appear");

    let found_other = logs
        .iter()
        .any(|e| e["message"].as_str().unwrap_or("").contains(&msg_other));
    assert!(
        !found_other,
        "log with different topic must NOT appear in topic filter"
    );
}

// ═══════════════════════════════════════════════════════════════════════════════
//  6. logs_topic strips horus_ prefix correctly
// ═══════════════════════════════════════════════════════════════════════════════

#[tokio::test]
async fn logs_topic_strips_horus_prefix() {
    // The handler strips "horus_" from the path param before querying the buffer.
    // So a log with topic="my_sensor" should be found via /api/logs/topic/horus_my_sensor.
    let bare_topic = uid("stripped_sensor");
    let prefixed_topic = format!("horus_{}", bare_topic);
    let msg = uid("horus_prefix_msg");
    let node = uid("prefix_node");

    push_log(&node, LogType::Publish, Some(&bare_topic), &msg, 40);

    let app = builders::test_router();
    let resp = app
        .oneshot(get_request(&format!("/api/logs/topic/{}", prefixed_topic)))
        .await
        .unwrap();
    let json = assert_json_ok(resp).await;
    let logs = json["logs"].as_array().expect("logs must be an array");

    let found = logs
        .iter()
        .any(|e| e["message"].as_str().unwrap_or("").contains(&msg));
    assert!(
        found,
        "requesting /api/logs/topic/{} should find logs with topic='{}' \
         because the handler strips the 'horus_' prefix",
        prefixed_topic, bare_topic
    );

    // The response's "topic" field should echo the original path parameter.
    assert_eq!(
        json["topic"].as_str().unwrap_or(""),
        prefixed_topic,
        "response 'topic' field must echo the raw path param"
    );
}

// ═══════════════════════════════════════════════════════════════════════════════
//  7. logs_topic without horus_ prefix works as-is
// ═══════════════════════════════════════════════════════════════════════════════

#[tokio::test]
async fn logs_topic_without_prefix_works_directly() {
    // When the topic name does NOT start with "horus_", strip_prefix returns
    // None and unwrap_or passes through the original name unchanged.
    let topic = uid("direct_topic");
    let msg = uid("direct_topic_msg");
    let node = uid("direct_topic_node");

    push_log(&node, LogType::Info, Some(&topic), &msg, 50);

    let app = builders::test_router();
    let resp = app
        .oneshot(get_request(&format!("/api/logs/topic/{}", topic)))
        .await
        .unwrap();
    let json = assert_json_ok(resp).await;
    let logs = json["logs"].as_array().expect("logs must be an array");

    let found = logs
        .iter()
        .any(|e| e["message"].as_str().unwrap_or("").contains(&msg));
    assert!(found, "topic without horus_ prefix should be queried as-is");
}

// ═══════════════════════════════════════════════════════════════════════════════
//  8. Response JSON schema has correct fields
// ═══════════════════════════════════════════════════════════════════════════════

#[tokio::test]
async fn logs_all_schema_has_logs_array() {
    let app = builders::test_router();
    let resp = app.oneshot(get_request("/api/logs/all")).await.unwrap();
    let json = assert_json_ok(resp).await;

    assert!(
        json.get("logs").is_some(),
        "/api/logs/all must have a 'logs' key"
    );
    assert!(json["logs"].is_array(), "'logs' must be an array");
}

#[tokio::test]
async fn logs_node_schema_has_node_and_logs_fields() {
    let node = uid("schema_node");
    let app = builders::test_router();
    let resp = app
        .oneshot(get_request(&format!("/api/logs/node/{}", node)))
        .await
        .unwrap();
    let json = assert_json_ok(resp).await;

    assert!(
        json.get("node").is_some() && json["node"].is_string(),
        "response must have a string 'node' field"
    );
    assert!(
        json.get("logs").is_some() && json["logs"].is_array(),
        "response must have a 'logs' array"
    );
}

#[tokio::test]
async fn logs_topic_schema_has_topic_and_logs_fields() {
    let topic = uid("schema_topic");
    let app = builders::test_router();
    let resp = app
        .oneshot(get_request(&format!("/api/logs/topic/{}", topic)))
        .await
        .unwrap();
    let json = assert_json_ok(resp).await;

    assert!(
        json.get("topic").is_some() && json["topic"].is_string(),
        "response must have a string 'topic' field"
    );
    assert!(
        json.get("logs").is_some() && json["logs"].is_array(),
        "response must have a 'logs' array"
    );
}

#[tokio::test]
async fn log_entry_schema_has_expected_fields() {
    let node = uid("entry_schema_node");
    let topic = uid("entry_schema_topic");
    let msg = uid("entry_schema_msg");

    push_log(&node, LogType::Publish, Some(&topic), &msg, 60);

    let app = builders::test_router();
    let resp = app.oneshot(get_request("/api/logs/all")).await.unwrap();
    let json = assert_json_ok(resp).await;
    let logs = json["logs"].as_array().expect("logs must be an array");

    // Find our specific entry.
    let entry = logs
        .iter()
        .find(|e| e["message"].as_str().unwrap_or("").contains(&msg))
        .expect("injected entry must be present");

    // Verify all expected fields exist.
    let required_keys = [
        "timestamp",
        "tick_number",
        "node_name",
        "log_type",
        "message",
        "tick_us",
        "ipc_ns",
    ];
    for key in &required_keys {
        assert!(
            entry.get(key).is_some() && !entry[key].is_null(),
            "log entry must have non-null key '{}', got: {}",
            key,
            serde_json::to_string_pretty(entry).unwrap_or_default()
        );
    }

    // topic may be null for entries without a topic, but ours should have one.
    assert!(
        entry.get("topic").is_some(),
        "log entry must have a 'topic' field"
    );
    assert_eq!(
        entry["topic"].as_str().unwrap_or(""),
        topic,
        "topic field must match"
    );
}

// ═══════════════════════════════════════════════════════════════════════════════
//  9. Concurrent log injection and reading
// ═══════════════════════════════════════════════════════════════════════════════

#[tokio::test]
async fn concurrent_inject_and_read() {
    let node = uid("concurrent_node");
    let base_msg = uid("concurrent_msg");

    // Inject 50 entries from multiple threads concurrently.
    let handles: Vec<_> = (0..5)
        .map(|thread_idx| {
            let n = node.clone();
            let m = base_msg.clone();
            std::thread::spawn(move || {
                for i in 0..10 {
                    let tick = (thread_idx * 10 + i) as u64;
                    push_log(
                        &n,
                        LogType::Info,
                        None,
                        &format!("{}_{}", m, tick),
                        1000 + tick,
                    );
                }
            })
        })
        .collect();

    for h in handles {
        h.join().expect("writer thread must not panic");
    }

    // Now read via the handler — all 50 entries should be visible.
    let app = builders::test_router();
    let resp = app
        .oneshot(get_request(&format!("/api/logs/node/{}", node)))
        .await
        .unwrap();
    let json = assert_json_ok(resp).await;
    let logs = json["logs"].as_array().expect("logs must be an array");

    let our_logs: Vec<_> = logs
        .iter()
        .filter(|e| {
            e["message"]
                .as_str()
                .map(|m| m.contains(&base_msg))
                .unwrap_or(false)
        })
        .collect();

    assert_eq!(
        our_logs.len(),
        50,
        "all 50 concurrently injected logs must be readable, found {}",
        our_logs.len()
    );
}

// ═══════════════════════════════════════════════════════════════════════════════
//  10. Different LogTypes (Publish, Subscribe, Info, Warning, Error, Debug)
// ═══════════════════════════════════════════════════════════════════════════════

#[tokio::test]
async fn logs_all_different_log_types() {
    let node = uid("logtype_node");

    let types = [
        (LogType::Publish, "Publish"),
        (LogType::Subscribe, "Subscribe"),
        (LogType::Info, "Info"),
        (LogType::Warning, "Warning"),
        (LogType::Error, "Error"),
        (LogType::Debug, "Debug"),
    ];

    for (i, (log_type, name)) in types.iter().enumerate() {
        let msg = format!("{}_type_{}", uid("lt"), name);
        push_log(&node, log_type.clone(), None, &msg, 200 + i as u64);
    }

    let app = builders::test_router();
    let resp = app
        .oneshot(get_request(&format!("/api/logs/node/{}", node)))
        .await
        .unwrap();
    let json = assert_json_ok(resp).await;
    let logs = json["logs"].as_array().expect("logs must be an array");

    // All 6 entries should be present for our node.
    let our_logs: Vec<_> = logs
        .iter()
        .filter(|e| e["node_name"].as_str() == Some(node.as_str()))
        .collect();

    assert!(
        our_logs.len() >= 6,
        "expected at least 6 entries (one per LogType), found {}",
        our_logs.len()
    );

    // Check that the log_type field is serialized correctly for each type.
    let log_type_values: Vec<&str> = our_logs
        .iter()
        .filter_map(|e| e["log_type"].as_str())
        .collect();

    for expected_type in &["Publish", "Subscribe", "Info", "Warning", "Error", "Debug"] {
        assert!(
            log_type_values.contains(expected_type),
            "log_type '{}' must appear in results, found: {:?}",
            expected_type,
            log_type_values
        );
    }
}

#[tokio::test]
async fn logs_inject_via_harness_with_different_severities() {
    // Verify the HorusTestRuntime::inject_log helper also produces readable
    // entries via the handler.
    let rt = HorusTestRuntime::new();
    let msg_info = uid("harness_info");
    let msg_err = uid("harness_error");
    let msg_warn = uid("harness_warn");
    let msg_debug = uid("harness_debug");
    let msg_pub = uid("harness_pub");
    let msg_sub = uid("harness_sub");

    let node = uid("harness_sev_node");
    rt.inject_log(&node, "info", &msg_info);
    rt.inject_log(&node, "error", &msg_err);
    rt.inject_log(&node, "warning", &msg_warn);
    rt.inject_log(&node, "debug", &msg_debug);
    rt.inject_log(&node, "publish", &msg_pub);
    rt.inject_log(&node, "subscribe", &msg_sub);

    let app = builders::test_router();
    let resp = app
        .oneshot(get_request(&format!("/api/logs/node/{}", node)))
        .await
        .unwrap();
    let json = assert_json_ok(resp).await;
    let logs = json["logs"].as_array().expect("logs must be an array");

    for expected_msg in [
        &msg_info, &msg_err, &msg_warn, &msg_debug, &msg_pub, &msg_sub,
    ] {
        let found = logs
            .iter()
            .any(|e| e["message"].as_str().unwrap_or("").contains(expected_msg));
        assert!(
            found,
            "harness-injected log '{}' must appear in results",
            expected_msg
        );
    }
}

// ═══════════════════════════════════════════════════════════════════════════════
//  11. Timestamp ordering — most recent entries appear
// ═══════════════════════════════════════════════════════════════════════════════

#[tokio::test]
async fn logs_all_recent_entries_present() {
    let node = uid("order_node");

    // Push entries with increasing tick numbers.
    for tick in 300..310u64 {
        let msg = format!("{}_tick_{}", uid("order"), tick);
        push_log(&node, LogType::Info, None, &msg, tick);
    }

    let app = builders::test_router();
    let resp = app
        .oneshot(get_request(&format!("/api/logs/node/{}", node)))
        .await
        .unwrap();
    let json = assert_json_ok(resp).await;
    let logs = json["logs"].as_array().expect("logs must be an array");

    // All 10 entries should be present.
    let our_ticks: Vec<u64> = logs
        .iter()
        .filter(|e| e["node_name"].as_str() == Some(node.as_str()))
        .filter_map(|e| e["tick_number"].as_u64())
        .filter(|t| (300..310).contains(t))
        .collect();

    assert_eq!(
        our_ticks.len(),
        10,
        "all 10 sequentially injected entries must be readable, found {:?}",
        our_ticks
    );

    // The entries should contain ticks 300..310.
    for tick in 300..310u64 {
        assert!(
            our_ticks.contains(&tick),
            "tick {} must be present in results",
            tick
        );
    }
}

// ═══════════════════════════════════════════════════════════════════════════════
//  12. Node name is echoed back in logs_node response
// ═══════════════════════════════════════════════════════════════════════════════

#[tokio::test]
async fn logs_node_echoes_node_name_in_response() {
    let node = uid("echo_node");
    let app = builders::test_router();
    let resp = app
        .oneshot(get_request(&format!("/api/logs/node/{}", node)))
        .await
        .unwrap();
    let json = assert_json_ok(resp).await;

    assert_eq!(
        json["node"].as_str().unwrap_or(""),
        node,
        "response 'node' field must match the requested node name"
    );
}

#[tokio::test]
async fn logs_node_echoes_arbitrary_name() {
    // Try a few different names to ensure the handler always echoes
    // the path parameter back.
    for name in &["sensor_driver", "motor_ctrl_42", "perception.lidar"] {
        let app = builders::test_router();
        let resp = app
            .oneshot(get_request(&format!("/api/logs/node/{}", name)))
            .await
            .unwrap();
        let json = assert_json_ok(resp).await;

        assert_eq!(
            json["node"].as_str().unwrap_or(""),
            *name,
            "response 'node' must echo '{}'",
            name
        );
    }
}

// ═══════════════════════════════════════════════════════════════════════════════
//  BONUS: topic echoed back in logs_topic response
// ═══════════════════════════════════════════════════════════════════════════════

#[tokio::test]
async fn logs_topic_echoes_topic_name_in_response() {
    let topic = uid("echo_topic");
    let app = builders::test_router();
    let resp = app
        .oneshot(get_request(&format!("/api/logs/topic/{}", topic)))
        .await
        .unwrap();
    let json = assert_json_ok(resp).await;

    assert_eq!(
        json["topic"].as_str().unwrap_or(""),
        topic,
        "response 'topic' field must match the requested topic name"
    );
}

#[tokio::test]
async fn logs_topic_echoes_prefixed_name_verbatim() {
    // When requesting with "horus_" prefix, the response should echo the
    // original prefixed name, NOT the stripped version.
    let prefixed = format!("horus_{}", uid("echo_prefixed"));
    let app = builders::test_router();
    let resp = app
        .oneshot(get_request(&format!("/api/logs/topic/{}", prefixed)))
        .await
        .unwrap();
    let json = assert_json_ok(resp).await;

    assert_eq!(
        json["topic"].as_str().unwrap_or(""),
        prefixed,
        "response 'topic' must echo the raw prefixed path param"
    );
}

// ═══════════════════════════════════════════════════════════════════════════════
//  BONUS: tick_us and ipc_ns fields are preserved
// ═══════════════════════════════════════════════════════════════════════════════

#[tokio::test]
async fn log_entry_preserves_timing_fields() {
    let node = uid("timing_node");
    let msg = uid("timing_msg");

    push_log(&node, LogType::Info, None, &msg, 77);

    let app = builders::test_router();
    let resp = app.oneshot(get_request("/api/logs/all")).await.unwrap();
    let json = assert_json_ok(resp).await;
    let logs = json["logs"].as_array().expect("logs must be an array");

    let entry = logs
        .iter()
        .find(|e| e["message"].as_str().unwrap_or("").contains(&msg))
        .expect("injected entry must be present");

    // tick_us = 77*100 = 7700, ipc_ns = 77*50 = 3850
    assert_eq!(
        entry["tick_us"].as_u64(),
        Some(7700),
        "tick_us must be preserved"
    );
    assert_eq!(
        entry["ipc_ns"].as_u64(),
        Some(3850),
        "ipc_ns must be preserved"
    );
    assert_eq!(
        entry["tick_number"].as_u64(),
        Some(77),
        "tick_number must be preserved"
    );
}
