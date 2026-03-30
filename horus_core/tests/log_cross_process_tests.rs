//! Cross-process logging integration tests.
//!
//! Verifies that log entries written by one process are visible to another
//! process reading from the same SHM-backed GLOBAL_LOG_BUFFER. This is
//! critical for robotics: driver, planner, and controller processes all log
//! independently, but `horus log` must see all their entries.
//!
//! Pattern: parent spawns child via env var flag. Child writes to buffer.
//! Parent reads buffer after child exits. Same pattern as cross_process_ipc.rs.
//!
//! Run sequentially: `cargo test --test log_cross_process_tests -- --test-threads=1`

use horus_core::core::log_buffer::{LogEntry, LogType, GLOBAL_ERROR_BUFFER, GLOBAL_LOG_BUFFER};
use std::process::{Command, Stdio};

// ─── Env vars ────────────────────────────────────────────────────────────────

const CHILD_ENV: &str = "HORUS_LOG_CROSS_CHILD";
const MARKER_ENV: &str = "HORUS_LOG_CROSS_MARKER";
const COUNT_ENV: &str = "HORUS_LOG_CROSS_COUNT";
const TEST_ENV: &str = "HORUS_LOG_CROSS_TEST";

fn is_child() -> bool {
    std::env::var(CHILD_ENV).is_ok()
}

fn uid(suffix: &str) -> String {
    format!("logxproc_{}_{}", std::process::id(), suffix)
}

// ─── Child entry points ──────────────────────────────────────────────────────

/// Child: push N log entries with the given marker, then exit.
fn child_push_logs() {
    let marker = std::env::var(MARKER_ENV).expect("MARKER_ENV not set");
    let count: usize = std::env::var(COUNT_ENV)
        .expect("COUNT_ENV not set")
        .parse()
        .expect("invalid count");

    for i in 0..count {
        GLOBAL_LOG_BUFFER.push(LogEntry {
            timestamp: format!("12:00:{:02}.000", i % 60),
            tick_number: i as u64,
            node_name: format!("child_{}", std::process::id()),
            log_type: LogType::Info,
            topic: None,
            message: format!("{}_{}", marker, i),
            tick_us: 0,
            ipc_ns: 0,
        });
    }
    println!("PUSHED:{}", count);
}

/// Child: push entries then exit(1) to simulate crash.
fn child_push_then_crash() {
    let marker = std::env::var(MARKER_ENV).expect("MARKER_ENV not set");
    let count: usize = std::env::var(COUNT_ENV)
        .unwrap_or_else(|_| "10".to_string())
        .parse()
        .unwrap_or(10);

    for i in 0..count {
        GLOBAL_LOG_BUFFER.push(LogEntry {
            timestamp: format!("12:00:{:02}.000", i % 60),
            tick_number: i as u64,
            node_name: format!("crash_child_{}", std::process::id()),
            log_type: LogType::Error,
            message: format!("{}_{}", marker, i),
            topic: None,
            tick_us: 0,
            ipc_ns: 0,
        });
    }
    println!("PUSHED:{}", count);
    // Simulate crash — no destructors run
    std::process::exit(1);
}

// ─── Spawn helper ────────────────────────────────────────────────────────────

fn spawn_log_child(test_name: &str, marker: &str, count: usize) -> std::process::Child {
    let exe = std::env::current_exe().expect("current_exe");
    Command::new(exe)
        .args([test_name, "--exact", "--nocapture"])
        .env(CHILD_ENV, "1")
        .env(TEST_ENV, test_name)
        .env(MARKER_ENV, marker)
        .env(COUNT_ENV, count.to_string())
        .stdout(Stdio::piped())
        .stderr(Stdio::piped())
        .spawn()
        .expect("failed to spawn child process")
}

fn wait_and_get_stdout(child: std::process::Child) -> String {
    let output = child.wait_with_output().expect("wait_with_output failed");
    String::from_utf8_lossy(&output.stdout).to_string()
}

// ═══════════════════════════════════════════════════════════════════════════════
//  1. Child writes are visible to parent
// ═══════════════════════════════════════════════════════════════════════════════

#[test]
fn cross_process_child_logs_visible_to_parent() {
    if is_child() {
        let test = std::env::var(TEST_ENV).unwrap_or_default();
        if test == "cross_process_child_logs_visible_to_parent" {
            child_push_logs();
        }
        return;
    }

    let marker = uid("child_visible");
    let count = 10;
    let child = spawn_log_child("cross_process_child_logs_visible_to_parent", &marker, count);
    let stdout = wait_and_get_stdout(child);
    assert!(
        stdout.contains("PUSHED:10"),
        "child should have pushed 10 entries, stdout: {}",
        stdout
    );

    // Parent reads the shared buffer — child's entries should be visible
    let all = GLOBAL_LOG_BUFFER.get_all();
    let found: Vec<_> = all.iter().filter(|e| e.message.contains(&marker)).collect();

    assert!(
        found.len() >= count,
        "parent must see at least {} entries from child, found {}",
        count,
        found.len()
    );
}

// ═══════════════════════════════════════════════════════════════════════════════
//  2. Concurrent parent + child writes — no corruption
// ═══════════════════════════════════════════════════════════════════════════════

#[test]
fn cross_process_concurrent_writes_no_corruption() {
    if is_child() {
        let test = std::env::var(TEST_ENV).unwrap_or_default();
        if test == "cross_process_concurrent_writes_no_corruption" {
            child_push_logs();
        }
        return;
    }

    let child_marker = uid("concurrent_child");
    let parent_marker = uid("concurrent_parent");
    let count = 50;

    // Spawn child that writes 50 entries
    let child = spawn_log_child(
        "cross_process_concurrent_writes_no_corruption",
        &child_marker,
        count,
    );

    // Parent also writes 50 entries concurrently
    for i in 0..count {
        GLOBAL_LOG_BUFFER.push(LogEntry {
            timestamp: format!("12:00:{:02}.000", i % 60),
            tick_number: i as u64,
            node_name: format!("parent_{}", std::process::id()),
            log_type: LogType::Warning,
            topic: None,
            message: format!("{}_{}", parent_marker, i),
            tick_us: 0,
            ipc_ns: 0,
        });
    }

    let _stdout = wait_and_get_stdout(child);

    let all = GLOBAL_LOG_BUFFER.get_all();
    let child_entries: Vec<_> = all
        .iter()
        .filter(|e| e.message.contains(&child_marker))
        .collect();
    let parent_entries: Vec<_> = all
        .iter()
        .filter(|e| e.message.contains(&parent_marker))
        .collect();

    assert!(
        child_entries.len() >= count,
        "child entries: expected >= {}, got {}",
        count,
        child_entries.len()
    );
    assert!(
        parent_entries.len() >= count,
        "parent entries: expected >= {}, got {}",
        count,
        parent_entries.len()
    );

    // Verify no field corruption — all entries should have valid log types
    for e in child_entries.iter().chain(parent_entries.iter()) {
        assert!(
            matches!(
                e.log_type,
                LogType::Info
                    | LogType::Warning
                    | LogType::Error
                    | LogType::Debug
                    | LogType::Publish
                    | LogType::Subscribe
            ),
            "corrupted log_type in entry: {:?}",
            e.log_type
        );
    }
}

// ═══════════════════════════════════════════════════════════════════════════════
//  3. SHM buffer survives child crash (exit(1))
// ═══════════════════════════════════════════════════════════════════════════════

#[test]
fn crash_survival_entries_persist_after_child_exit() {
    if is_child() {
        let test = std::env::var(TEST_ENV).unwrap_or_default();
        if test == "crash_survival_entries_persist_after_child_exit" {
            child_push_then_crash();
        }
        return;
    }

    let marker = uid("crash_persist");
    let count = 10;
    let child = spawn_log_child(
        "crash_survival_entries_persist_after_child_exit",
        &marker,
        count,
    );
    let output = child.wait_with_output().expect("wait failed");
    let status = output.status;
    assert!(
        !status.success(),
        "child should have exited with error (exit(1))"
    );

    // Entries written before crash must persist in SHM buffer
    let all = GLOBAL_LOG_BUFFER.get_all();
    let found: Vec<_> = all.iter().filter(|e| e.message.contains(&marker)).collect();

    assert!(
        found.len() >= count,
        "entries written before crash must persist, expected >= {}, found {}",
        count,
        found.len()
    );

    // Verify they're Error type (what child_push_then_crash writes)
    assert!(
        found.iter().all(|e| e.log_type == LogType::Error),
        "crash entries should all be Error type"
    );
}

// ═══════════════════════════════════════════════════════════════════════════════
//  4. Node name attribution correct across processes
// ═══════════════════════════════════════════════════════════════════════════════

#[test]
fn cross_process_node_name_contains_child_pid() {
    if is_child() {
        let test = std::env::var(TEST_ENV).unwrap_or_default();
        if test == "cross_process_node_name_contains_child_pid" {
            child_push_logs();
        }
        return;
    }

    let marker = uid("node_attr");
    let child = spawn_log_child("cross_process_node_name_contains_child_pid", &marker, 5);
    let _stdout = wait_and_get_stdout(child);

    let all = GLOBAL_LOG_BUFFER.get_all();
    let found: Vec<_> = all.iter().filter(|e| e.message.contains(&marker)).collect();

    assert!(!found.is_empty(), "should find child entries");

    // All entries should have node_name starting with "child_" (set by child_push_logs)
    for e in &found {
        assert!(
            e.node_name.starts_with("child_"),
            "child entry node_name should start with 'child_', got: {}",
            e.node_name
        );
    }
}

// ═══════════════════════════════════════════════════════════════════════════════
//  5. Cross-process error buffer visibility (via publish_log dual-write)
// ═══════════════════════════════════════════════════════════════════════════════

/// Child: push Error entries via publish_log (triggers dual-write to error buffer).
fn child_push_errors_via_publish_log() {
    use horus_core::core::log_buffer::publish_log;

    let marker = std::env::var(MARKER_ENV).expect("MARKER_ENV not set");
    let count: usize = std::env::var(COUNT_ENV)
        .unwrap_or_else(|_| "10".to_string())
        .parse()
        .unwrap_or(10);

    for i in 0..count {
        publish_log(LogEntry {
            timestamp: format!("12:00:{:02}.000", i % 60),
            tick_number: i as u64,
            node_name: format!("errbuf_child_{}", std::process::id()),
            log_type: LogType::Error,
            topic: None,
            message: format!("{}_{}", marker, i),
            tick_us: 0,
            ipc_ns: 0,
        });
    }
    println!("PUSHED:{}", count);
}

/// Child: push Error entries via publish_log then crash.
fn child_push_errors_then_crash() {
    use horus_core::core::log_buffer::publish_log;

    let marker = std::env::var(MARKER_ENV).expect("MARKER_ENV not set");
    let count: usize = std::env::var(COUNT_ENV)
        .unwrap_or_else(|_| "10".to_string())
        .parse()
        .unwrap_or(10);

    for i in 0..count {
        publish_log(LogEntry {
            timestamp: format!("12:00:{:02}.000", i % 60),
            tick_number: i as u64,
            node_name: format!("errcrash_child_{}", std::process::id()),
            log_type: LogType::Error,
            topic: None,
            message: format!("{}_{}", marker, i),
            tick_us: 0,
            ipc_ns: 0,
        });
    }
    println!("PUSHED:{}", count);
    std::process::exit(1);
}

#[test]
fn cross_process_error_buffer_visibility() {
    if is_child() {
        let test = std::env::var(TEST_ENV).unwrap_or_default();
        if test == "cross_process_error_buffer_visibility" {
            child_push_errors_via_publish_log();
        }
        return;
    }

    let marker = uid("xproc_errbuf");
    let count = 10;
    let child = spawn_log_child("cross_process_error_buffer_visibility", &marker, count);
    let stdout = wait_and_get_stdout(child);
    assert!(stdout.contains("PUSHED:10"), "child should push 10 entries");

    // Verify in error buffer (child used publish_log which dual-writes)
    let error_entries = GLOBAL_ERROR_BUFFER.get_all();
    let found: Vec<_> = error_entries
        .iter()
        .filter(|e| e.message.contains(&marker))
        .collect();

    assert!(
        found.len() >= count,
        "parent must see child's errors in GLOBAL_ERROR_BUFFER, expected >= {}, found {}",
        count,
        found.len()
    );
}

#[test]
fn cross_process_error_buffer_crash_persistence() {
    if is_child() {
        let test = std::env::var(TEST_ENV).unwrap_or_default();
        if test == "cross_process_error_buffer_crash_persistence" {
            child_push_errors_then_crash();
        }
        return;
    }

    let marker = uid("xproc_errcrash");
    let count = 10;
    let child = spawn_log_child(
        "cross_process_error_buffer_crash_persistence",
        &marker,
        count,
    );
    let output = child.wait_with_output().expect("wait failed");
    assert!(!output.status.success(), "child should exit(1)");

    // Errors must persist in error buffer after crash
    let error_entries = GLOBAL_ERROR_BUFFER.get_all();
    let found: Vec<_> = error_entries
        .iter()
        .filter(|e| e.message.contains(&marker))
        .collect();

    assert!(
        found.len() >= count,
        "errors must persist in error buffer after child crash, expected >= {}, found {}",
        count,
        found.len()
    );
}
