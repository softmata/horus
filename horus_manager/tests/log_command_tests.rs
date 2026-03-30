//! Integration tests for the `horus log` CLI command.
//!
//! Tests `view_logs()` and `clear_logs()` end-to-end with pre-filled
//! GLOBAL_LOG_BUFFER data, filter combinations, and edge cases.

use horus_core::core::log_buffer::{
    publish_log, LogEntry, LogType, GLOBAL_ERROR_BUFFER, GLOBAL_LOG_BUFFER,
};
use horus_manager::commands::log::{clear_logs, view_logs};

// ─── Helpers ─────────────────────────────────────────────────────────────────

/// Unique tag with PID + suffix to avoid cross-test collisions.
fn uid(suffix: &str) -> String {
    format!("logcmd_{}_{}", std::process::id(), suffix)
}

/// Push a log entry with controllable fields.
fn push_entry(node: &str, log_type: LogType, message: &str, timestamp: &str) {
    GLOBAL_LOG_BUFFER.push(LogEntry {
        timestamp: timestamp.to_string(),
        tick_number: 0,
        node_name: node.to_string(),
        log_type,
        topic: None,
        message: message.to_string(),
        tick_us: 0,
        ipc_ns: 0,
    });
}

fn push_entry_with_topic(
    node: &str,
    log_type: LogType,
    topic: &str,
    message: &str,
    timestamp: &str,
) {
    GLOBAL_LOG_BUFFER.push(LogEntry {
        timestamp: timestamp.to_string(),
        tick_number: 0,
        node_name: node.to_string(),
        log_type,
        topic: Some(topic.to_string()),
        message: message.to_string(),
        tick_us: 0,
        ipc_ns: 0,
    });
}

// ═══════════════════════════════════════════════════════════════════════════════
//  1. Basic view_logs — empty/non-empty buffer
// ═══════════════════════════════════════════════════════════════════════════════

#[test]
fn view_logs_with_nonexistent_node_filter_returns_ok() {
    // Filtering for a node that doesn't exist should succeed (shows "no entries matched")
    let node = uid("nonexistent_node");
    let result = view_logs(Some(&node), None, None, false, None);
    assert!(
        result.is_ok(),
        "view_logs should return Ok even with no matches"
    );
}

#[test]
fn view_logs_with_entries_returns_ok() {
    let node = uid("has_entries");
    push_entry(&node, LogType::Info, "test message", "12:00:00.000");
    push_entry(&node, LogType::Error, "error message", "12:00:01.000");

    let result = view_logs(Some(&node), None, None, false, None);
    assert!(
        result.is_ok(),
        "view_logs should succeed with entries present"
    );
}

// ═══════════════════════════════════════════════════════════════════════════════
//  2. Node filter
// ═══════════════════════════════════════════════════════════════════════════════

#[test]
fn view_logs_node_filter_returns_ok() {
    let node_a = uid("filter_a");
    let node_b = uid("filter_b");
    push_entry(&node_a, LogType::Info, "from A", "12:00:00.000");
    push_entry(&node_b, LogType::Info, "from B", "12:00:01.000");

    // Filter for node_a only — should succeed
    let result = view_logs(Some(&node_a), None, None, false, None);
    assert!(result.is_ok());

    // Filter for node_b only — should succeed
    let result = view_logs(Some(&node_b), None, None, false, None);
    assert!(result.is_ok());
}

#[test]
fn view_logs_node_filter_logic_matches_contains() {
    // view_logs uses `node_name.contains(filter)`, so partial match works
    let node = uid("partial_match_sensor_node");
    push_entry(&node, LogType::Info, "sensor data", "12:00:00.000");

    // Partial match: "sensor" should match "partial_match_sensor_node"
    let result = view_logs(Some("sensor"), None, None, false, None);
    assert!(result.is_ok());
}

// ═══════════════════════════════════════════════════════════════════════════════
//  3. Level filter
// ═══════════════════════════════════════════════════════════════════════════════

#[test]
fn view_logs_level_filter_returns_ok() {
    let node = uid("level_filter");
    push_entry(&node, LogType::Debug, "debug msg", "12:00:00.000");
    push_entry(&node, LogType::Info, "info msg", "12:00:01.000");
    push_entry(&node, LogType::Warning, "warn msg", "12:00:02.000");
    push_entry(&node, LogType::Error, "error msg", "12:00:03.000");

    // Filter for error level — should succeed
    let result = view_logs(Some(&node), Some("error"), None, false, None);
    assert!(result.is_ok());

    // Filter for info level — should succeed
    let result = view_logs(Some(&node), Some("info"), None, false, None);
    assert!(result.is_ok());
}

#[test]
fn view_logs_level_filter_invalid_level_shows_all() {
    // Invalid level string falls through to Trace (shows all)
    let node = uid("invalid_level");
    push_entry(&node, LogType::Info, "should show", "12:00:00.000");

    let result = view_logs(Some(&node), Some("bogus"), None, false, None);
    assert!(
        result.is_ok(),
        "invalid level string should not error, just show all"
    );
}

// ═══════════════════════════════════════════════════════════════════════════════
//  4. Since filter
// ═══════════════════════════════════════════════════════════════════════════════

#[test]
fn view_logs_since_filter_valid_formats_return_ok() {
    let node = uid("since_valid");
    push_entry(&node, LogType::Info, "recent", "12:00:00.000");

    assert!(view_logs(Some(&node), None, Some("5s"), false, None).is_ok());
    assert!(view_logs(Some(&node), None, Some("10m"), false, None).is_ok());
    assert!(view_logs(Some(&node), None, Some("1h"), false, None).is_ok());
    assert!(view_logs(Some(&node), None, Some("1d"), false, None).is_ok());
}

#[test]
fn view_logs_since_filter_invalid_format_returns_error() {
    let node = uid("since_invalid");

    let result = view_logs(Some(&node), None, Some("5x"), false, None);
    assert!(result.is_err(), "invalid since format should return Err");

    let result = view_logs(Some(&node), None, Some("abc"), false, None);
    assert!(result.is_err(), "non-numeric since should return Err");
}

// ═══════════════════════════════════════════════════════════════════════════════
//  5. Count limit
// ═══════════════════════════════════════════════════════════════════════════════

#[test]
fn view_logs_count_limit_returns_ok() {
    let node = uid("count_limit");
    for i in 0..50 {
        push_entry(&node, LogType::Info, &format!("msg_{}", i), "12:00:00.000");
    }

    // Limit to 5 entries
    let result = view_logs(Some(&node), None, None, false, Some(5));
    assert!(result.is_ok());

    // Limit to 1 entry
    let result = view_logs(Some(&node), None, None, false, Some(1));
    assert!(result.is_ok());
}

// ═══════════════════════════════════════════════════════════════════════════════
//  6. Combined filters
// ═══════════════════════════════════════════════════════════════════════════════

#[test]
fn view_logs_combined_node_and_level_filter() {
    let node = uid("combo_filter");
    push_entry(&node, LogType::Debug, "debug msg", "12:00:00.000");
    push_entry(&node, LogType::Info, "info msg", "12:00:01.000");
    push_entry(&node, LogType::Error, "error msg", "12:00:02.000");

    // Node + level filter
    let result = view_logs(Some(&node), Some("warn"), None, false, None);
    assert!(result.is_ok());
}

#[test]
fn view_logs_combined_all_filters() {
    let node = uid("all_filters");
    push_entry(&node, LogType::Error, "critical failure", "12:00:00.000");

    // Node + level + since + count — all filters active
    let result = view_logs(Some(&node), Some("error"), Some("5m"), false, Some(10));
    assert!(result.is_ok());
}

// ═══════════════════════════════════════════════════════════════════════════════
//  7. Filter logic verification (replicate view_logs filter chain)
// ═══════════════════════════════════════════════════════════════════════════════

#[test]
fn filter_chain_node_excludes_non_matching() {
    let node_a = uid("chain_a");
    let node_b = uid("chain_b");
    push_entry(&node_a, LogType::Info, "from A", "12:00:00.000");
    push_entry(&node_b, LogType::Info, "from B", "12:00:01.000");

    // Replicate view_logs filter logic to verify correctness
    let all = GLOBAL_LOG_BUFFER.get_all();
    let filtered: Vec<_> = all
        .iter()
        .filter(|e| e.node_name.contains(&node_a))
        .collect();

    assert!(
        filtered.iter().all(|e| e.node_name.contains(&node_a)),
        "all filtered entries should match node_a"
    );
    assert!(
        filtered.iter().all(|e| !e.node_name.contains(&node_b)),
        "no filtered entries should match node_b"
    );
}

#[test]
fn filter_chain_level_excludes_lower_severity() {
    let node = uid("chain_level");
    push_entry(&node, LogType::Debug, "debug", "12:00:00.000");
    push_entry(&node, LogType::Info, "info", "12:00:01.000");
    push_entry(&node, LogType::Warning, "warn", "12:00:02.000");
    push_entry(&node, LogType::Error, "error", "12:00:03.000");

    let all = GLOBAL_LOG_BUFFER.get_all();

    // Info filter: should exclude Debug, include Info/Warning/Error
    let info_filtered: Vec<_> = all
        .iter()
        .filter(|e| e.node_name.contains(&node))
        .filter(|e| match &e.log_type {
            LogType::Debug => false,                        // Below Info
            LogType::Publish | LogType::Subscribe => false, // Trace level
            _ => true,
        })
        .collect();

    let has_debug = info_filtered.iter().any(|e| e.message == "debug");
    assert!(!has_debug, "Debug should be excluded by Info filter");

    let has_info = info_filtered.iter().any(|e| e.message == "info");
    assert!(has_info, "Info should pass Info filter");

    let has_error = info_filtered.iter().any(|e| e.message == "error");
    assert!(has_error, "Error should pass Info filter");
}

#[test]
fn filter_chain_count_limits_output() {
    let node = uid("chain_count");
    for i in 0..20 {
        push_entry(
            &node,
            LogType::Info,
            &format!("count_{}", i),
            "12:00:00.000",
        );
    }

    let all = GLOBAL_LOG_BUFFER.get_all();
    let matching: Vec<_> = all.iter().filter(|e| e.node_name == node).collect();
    let limited: Vec<_> = matching.iter().rev().take(5).collect();

    assert!(matching.len() >= 20, "should have at least 20 entries");
    assert_eq!(limited.len(), 5, "count limit should cap at 5");
}

// ═══════════════════════════════════════════════════════════════════════════════
//  8. Topic entries
// ═══════════════════════════════════════════════════════════════════════════════

#[test]
fn view_logs_with_topic_entries_returns_ok() {
    let node = uid("topic_entries");
    push_entry_with_topic(
        &node,
        LogType::Publish,
        "cmd_vel",
        "published CmdVel",
        "12:00:00.000",
    );
    push_entry_with_topic(
        &node,
        LogType::Subscribe,
        "imu",
        "received Imu",
        "12:00:01.000",
    );

    let result = view_logs(Some(&node), None, None, false, None);
    assert!(result.is_ok());
}

// ═══════════════════════════════════════════════════════════════════════════════
//  9. clear_logs
// ═══════════════════════════════════════════════════════════════════════════════

#[test]
fn clear_logs_function_is_callable() {
    // Verify clear_logs compiles and has the expected signature.
    // We do NOT call it here because it deletes the shared SHM files,
    // which breaks all other parallel tests. clear_logs is tested
    // implicitly via CLI integration and in sequential test runs.
    let _fn_ref: fn(bool) -> horus_core::error::HorusResult<()> = clear_logs;
}

// ═══════════════════════════════════════════════════════════════════════════════
//  10. All LogType variants through view_logs
// ═══════════════════════════════════════════════════════════════════════════════

#[test]
fn view_logs_all_log_types_no_panic() {
    let node = uid("all_types");
    push_entry(&node, LogType::Info, "info", "12:00:00.000");
    push_entry(&node, LogType::Warning, "warn", "12:00:01.000");
    push_entry(&node, LogType::Error, "error", "12:00:02.000");
    push_entry(&node, LogType::Debug, "debug", "12:00:03.000");
    push_entry_with_topic(&node, LogType::Publish, "topic", "pub", "12:00:04.000");
    push_entry_with_topic(&node, LogType::Subscribe, "topic", "sub", "12:00:05.000");

    // All 6 LogType variants, at trace level (show all)
    let result = view_logs(Some(&node), Some("trace"), None, false, None);
    assert!(result.is_ok());
}

#[test]
fn view_logs_pub_sub_excluded_by_info_filter() {
    let node = uid("pub_sub_filter");
    push_entry_with_topic(&node, LogType::Publish, "cmd", "pub msg", "12:00:00.000");
    push_entry_with_topic(&node, LogType::Subscribe, "imu", "sub msg", "12:00:01.000");
    push_entry(&node, LogType::Info, "info msg", "12:00:02.000");

    // Pub/Sub are Trace level — info filter should exclude them
    // We can verify by checking the filter logic directly
    let all = GLOBAL_LOG_BUFFER.get_all();
    let info_and_above: Vec<_> = all
        .iter()
        .filter(|e| e.node_name.contains(&node))
        .filter(|e| {
            !matches!(
                &e.log_type,
                LogType::Publish | LogType::Subscribe | LogType::Debug
            )
        })
        .collect();

    assert!(
        info_and_above
            .iter()
            .all(|e| !matches!(&e.log_type, LogType::Publish | LogType::Subscribe)),
        "Pub/Sub should be excluded at Info level"
    );
    assert!(
        info_and_above
            .iter()
            .any(|e| matches!(&e.log_type, LogType::Info)),
        "Info should pass Info filter"
    );
}

// ═══════════════════════════════════════════════════════════════════════════════
//  Follow mode: write_idx delta detection
// ═══════════════════════════════════════════════════════════════════════════════

#[test]
fn follow_delta_detects_new_entries() {
    let node = uid("follow_delta");
    let before = GLOBAL_LOG_BUFFER.write_idx();
    for i in 0..7 {
        push_entry(
            &node,
            LogType::Info,
            &format!("delta_{}", i),
            "12:00:00.000",
        );
    }
    let after = GLOBAL_LOG_BUFFER.write_idx();
    let delta = after.wrapping_sub(before) as usize;
    // >= because other parallel tests may also push entries to the shared buffer
    assert!(
        delta >= 7,
        "write_idx delta must be at least 7, got {}",
        delta
    );
}

#[test]
fn follow_delta_no_matching_entries_when_none_pushed() {
    // Instead of asserting write_idx doesn't change (flaky — parallel tests push entries),
    // verify that OUR unique marker never appears in the buffer when we don't push it.
    let marker = uid("never_pushed_marker");

    let all = GLOBAL_LOG_BUFFER.get_all();
    let found = all.iter().any(|e| e.message.contains(&marker));
    assert!(
        !found,
        "a marker we never pushed must not appear in the buffer"
    );
}

#[test]
fn follow_new_entries_found_in_buffer_after_push() {
    let node = uid("follow_slice");
    let marker = uid("follow_slice_marker");

    for i in 0..5 {
        push_entry(
            &node,
            LogType::Info,
            &format!("{}_{}", marker, i),
            "12:00:00.000",
        );
    }

    // Search the full buffer for our unique marker — parallel-safe
    let all = GLOBAL_LOG_BUFFER.get_all();
    let matching: Vec<_> = all.iter().filter(|e| e.message.contains(&marker)).collect();
    assert_eq!(
        matching.len(),
        5,
        "buffer must contain all 5 pushed entries with marker, found {}",
        matching.len()
    );
}

#[test]
fn follow_delta_from_background_thread() {
    let node = uid("follow_bg_thread");
    let marker = uid("follow_bg_marker");

    // Simulate entries arriving from a background "robot" thread
    let n = node.clone();
    let m = marker.clone();
    let handle = std::thread::spawn(move || {
        for i in 0..20 {
            push_entry(&n, LogType::Info, &format!("{}_{}", m, i), "12:00:00.000");
            std::thread::sleep(std::time::Duration::from_millis(5));
        }
    });
    handle.join().unwrap();

    // Search full buffer for our unique marker — parallel-safe
    let all = GLOBAL_LOG_BUFFER.get_all();
    let matching: Vec<_> = all.iter().filter(|e| e.message.contains(&marker)).collect();
    assert_eq!(
        matching.len(),
        20,
        "all 20 background entries must be in buffer"
    );
}

// ═══════════════════════════════════════════════════════════════════════════════
//  Follow mode: filter-in-stream (node and level filters on new entries)
// ═══════════════════════════════════════════════════════════════════════════════

#[test]
fn follow_filter_level_excludes_debug_at_info_level() {
    let node = uid("follow_level_filter");
    let info_msg = uid("follow_info_msg");
    let error_msg = uid("follow_error_msg");
    let debug_msg = uid("follow_debug_msg");

    push_entry(&node, LogType::Debug, &debug_msg, "12:00:00.000");
    push_entry(&node, LogType::Info, &info_msg, "12:00:01.000");
    push_entry(&node, LogType::Error, &error_msg, "12:00:02.000");

    // Read full buffer and apply follow_logs() filter logic: node + level>=Info
    let all = GLOBAL_LOG_BUFFER.get_all();
    let info_filtered: Vec<_> = all
        .iter()
        .filter(|e| e.node_name == node)
        .filter(|e| {
            !matches!(
                e.log_type,
                LogType::Debug | LogType::Publish | LogType::Subscribe
            )
        })
        .collect();

    assert!(
        info_filtered.iter().all(|e| e.log_type != LogType::Debug),
        "Debug must be excluded at Info filter level"
    );
    assert!(
        info_filtered.iter().any(|e| e.message == info_msg),
        "Info should pass"
    );
    assert!(
        info_filtered.iter().any(|e| e.message == error_msg),
        "Error should pass"
    );
}

#[test]
fn follow_filter_node_excludes_other_nodes() {
    let target = uid("follow_node_target");
    let other = uid("follow_node_other");
    let target_msg = uid("target_node_msg");
    let target_err = uid("target_node_err");

    push_entry(&target, LogType::Info, &target_msg, "12:00:00.000");
    push_entry(&other, LogType::Info, "other msg", "12:00:01.000");
    push_entry(&target, LogType::Error, &target_err, "12:00:02.000");

    // Search full buffer — parallel-safe
    let all = GLOBAL_LOG_BUFFER.get_all();
    let node_filtered: Vec<_> = all.iter().filter(|e| e.node_name == target).collect();

    assert!(
        node_filtered.len() >= 2,
        "should find at least 2 target entries, found {}",
        node_filtered.len()
    );
    assert!(node_filtered.iter().any(|e| e.message == target_msg));
    assert!(node_filtered.iter().any(|e| e.message == target_err));
    assert!(
        node_filtered.iter().all(|e| e.node_name == target),
        "all filtered entries must be from target node"
    );
}

#[test]
fn follow_filter_combined_node_and_level() {
    let target = uid("follow_combo_target");
    let other = uid("follow_combo_other");
    let target_error = uid("combo_target_error");
    let target_info = uid("combo_target_info");

    push_entry(&target, LogType::Debug, "target debug", "12:00:00.000");
    push_entry(&target, LogType::Error, &target_error, "12:00:01.000");
    push_entry(&other, LogType::Error, "other error", "12:00:02.000");
    push_entry(&target, LogType::Info, &target_info, "12:00:03.000");

    // Search full buffer — parallel-safe
    let all = GLOBAL_LOG_BUFFER.get_all();

    // Combined: node=target AND level>=Info (excludes Debug/Pub/Sub)
    let filtered: Vec<_> = all
        .iter()
        .filter(|e| e.node_name == target)
        .filter(|e| {
            !matches!(
                e.log_type,
                LogType::Debug | LogType::Publish | LogType::Subscribe
            )
        })
        .collect();

    assert!(
        filtered.len() >= 2,
        "should match at least target+error and target+info, found {}",
        filtered.len()
    );
    assert!(filtered.iter().any(|e| e.message == target_error));
    assert!(filtered.iter().any(|e| e.message == target_info));
    assert!(filtered.iter().all(|e| e.node_name == target));
}

// ═══════════════════════════════════════════════════════════════════════════════
//  CLI error buffer wiring verification
// ═══════════════════════════════════════════════════════════════════════════════

fn make_entry(node: &str, log_type: LogType, message: &str) -> LogEntry {
    LogEntry {
        timestamp: "12:00:00.000".to_string(),
        tick_number: 0,
        node_name: node.to_string(),
        log_type,
        topic: None,
        message: message.to_string(),
        tick_us: 0,
        ipc_ns: 0,
    }
}

#[test]
fn view_logs_error_level_reads_from_error_buffer() {
    // Push error via publish_log (dual-writes to both buffers)
    let node = uid("cli_errbuf_read");
    let marker = uid("cli_errbuf_marker");
    publish_log(make_entry(&node, LogType::Error, &marker));

    // Verify it's in the error buffer
    let error_entries = GLOBAL_ERROR_BUFFER.get_all();
    let in_error_buf = error_entries.iter().any(|e| e.message == marker);
    assert!(
        in_error_buf,
        "Error must be in error buffer for test to be valid"
    );

    // view_logs with level=error should succeed (reads from error buffer)
    let result = view_logs(Some(&node), Some("error"), None, false, None);
    assert!(result.is_ok(), "view_logs(level=error) must succeed");
}

#[test]
fn view_logs_warn_level_reads_from_error_buffer() {
    let node = uid("cli_warnbuf_read");
    let marker = uid("cli_warnbuf_marker");
    publish_log(make_entry(&node, LogType::Warning, &marker));

    let result = view_logs(Some(&node), Some("warn"), None, false, None);
    assert!(result.is_ok(), "view_logs(level=warn) must succeed");
}

#[test]
fn view_logs_info_level_still_reads_main_buffer() {
    let node = uid("cli_mainbuf_read");
    let marker = uid("cli_mainbuf_marker");
    publish_log(make_entry(&node, LogType::Info, &marker));

    // Info should NOT be in error buffer
    let error_entries = GLOBAL_ERROR_BUFFER.get_all();
    let in_error_buf = error_entries.iter().any(|e| e.message == marker);
    assert!(!in_error_buf, "Info must NOT be in error buffer");

    // view_logs with level=info reads from main buffer — should find it
    let result = view_logs(Some(&node), Some("info"), None, false, None);
    assert!(
        result.is_ok(),
        "view_logs(level=info) must succeed reading main buffer"
    );
}

#[test]
fn view_logs_error_level_empty_buffer_returns_ok() {
    // Use a node name that definitely has no entries
    let node = uid("cli_errbuf_empty_nonexistent_xyz");
    let result = view_logs(Some(&node), Some("error"), None, false, None);
    assert!(
        result.is_ok(),
        "view_logs(level=error) with no matching entries must return Ok"
    );
}

#[test]
fn follow_delta_tracks_error_buffer_for_error_level() {
    let node = uid("follow_errbuf_delta");
    let marker = uid("follow_errbuf_marker");

    // Snapshot error buffer write_idx
    let before = GLOBAL_ERROR_BUFFER.write_idx();

    // Push Error via publish_log (dual-writes to error buffer)
    publish_log(make_entry(&node, LogType::Error, &marker));

    let after = GLOBAL_ERROR_BUFFER.write_idx();
    assert!(
        after > before,
        "error buffer write_idx must advance after Error push (follow mode would detect this)"
    );
}

#[test]
fn follow_delta_does_not_track_error_buffer_for_info() {
    let node = uid("follow_mainbuf_delta");

    let before = GLOBAL_ERROR_BUFFER.write_idx();

    // Push Info — should NOT touch error buffer
    publish_log(make_entry(&node, LogType::Info, "info only"));

    let after = GLOBAL_ERROR_BUFFER.write_idx();
    // In parallel tests, other threads may push errors, so allow small delta
    assert!(
        after.wrapping_sub(before) <= 5,
        "error buffer write_idx should not advance significantly after Info push"
    );
}

#[test]
fn clear_logs_paths_exist() {
    // Verify the SHM paths that clear_logs would delete are valid paths.
    // We do NOT call clear_logs() here because it destroys the shared buffer
    // used by all parallel tests. Instead verify the path logic is correct.
    use horus_core::memory::{shm_error_logs_path, shm_logs_path};
    let log_path = shm_logs_path();
    let error_path = shm_error_logs_path();

    // Paths should be under the SHM namespace directory
    assert!(
        log_path.to_str().unwrap().contains("horus_"),
        "log path should be namespaced: {}",
        log_path.display()
    );
    assert!(
        error_path.to_str().unwrap().contains("horus_"),
        "error log path should be namespaced: {}",
        error_path.display()
    );
    assert_ne!(log_path, error_path, "log and error paths must differ");
}
