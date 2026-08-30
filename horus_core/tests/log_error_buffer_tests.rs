#![allow(dead_code)]
//! Comprehensive tests for the dedicated error ring buffer.
//!
//! Proves: dual-write routing, error buffer isolation, volume survival,
//! cross-process visibility, concurrent access, and latency overhead.

use horus_core::core::log_buffer::{
    publish_log, LogEntry, LogType, GLOBAL_ERROR_BUFFER, GLOBAL_LOG_BUFFER,
};
use std::sync::Arc;

fn uid(suffix: &str) -> String {
    format!("errbuf_{}_{}", std::process::id(), suffix)
}

/// Serializes tests that read process-global buffer state.
///
/// `GLOBAL_LOG_BUFFER` and `GLOBAL_ERROR_BUFFER` are process-wide, and every
/// test in this file writes to them. Three tests here also *read* aggregate
/// state — a write index, a survivor count, a timing ratio — which any
/// concurrent test invalidates. They failed under the default parallel harness
/// and passed under `--test-threads=1`, so a developer saw red on a full run
/// and green on the retry, which teaches people to rerun instead of read.
///
/// One test even documented the race in its own assertion message ("parallel
/// tests may add a few") and picked a fudge factor of 5, which the suite then
/// exceeded anyway. A tolerance wide enough to absorb the race is also wide
/// enough to absorb the bug it is meant to catch; serializing is what makes the
/// assertion mean something.
static GLOBAL_BUFFER_LOCK: std::sync::Mutex<()> = std::sync::Mutex::new(());

fn exclusive() -> std::sync::MutexGuard<'static, ()> {
    GLOBAL_BUFFER_LOCK
        .lock()
        .unwrap_or_else(|poisoned| poisoned.into_inner())
}

/// Run one test in a child process with a private `HORUS_NAMESPACE`.
///
/// Returns `true` in the child ("do the work"); in the parent it spawns that
/// child, asserts it passed, and returns `false`.
///
/// `GLOBAL_ERROR_BUFFER` is not a process-local static — `log_buffer` maps it
/// out of `shm_logs_path()`, so it is a fixed-size ring shared by every HORUS
/// process in the namespace. The `exclusive()` lock below serializes the tests
/// in THIS binary against each other, but it cannot stop another test binary
/// running concurrently from writing errors into the same ring and evicting
/// this test's entries before it counts them. That is how
/// `concurrent_dual_write_no_corruption` failed in a full workspace run while
/// passing on its own.
///
/// Only the tests that count their own surviving entries need this. The ones
/// that filter by a unique node name and assert presence are unaffected by
/// other writers.
fn count_in_isolated_child(test_name: &str) -> bool {
    const MARKER: &str = "HORUS_ERRBUF_CHILD";
    if std::env::var(MARKER).is_ok() {
        return true;
    }
    // Hold the file's lock across the spawn. The child is a separate process
    // doing real work, so without this it runs concurrently with whatever else
    // this binary is executing — and `dual_write_overhead_acceptable` next door
    // times a sub-microsecond operation. Isolating one test must not perturb its
    // neighbours.
    let _serialize = exclusive();
    let exe = std::env::current_exe().expect("current_exe");
    let ns = format!("errbuf_{}_{}", std::process::id(), test_name);
    let out = std::process::Command::new(exe)
        .args([test_name, "--exact", "--test-threads=1"])
        .env(MARKER, "1")
        .env("HORUS_NAMESPACE", &ns)
        .stderr(std::process::Stdio::null())
        .output()
        .expect("spawn isolated child");
    let stdout = String::from_utf8_lossy(&out.stdout);
    assert!(
        out.status.success(),
        "{test_name} failed in an isolated child with a private log buffer, so \
         this is the behaviour and not another binary evicting entries:\n{stdout}"
    );
    // Anti-vacuity: a filter that stopped matching would run zero tests, exit
    // zero, and this test would pass having checked nothing.
    assert!(
        stdout.contains("1 passed"),
        "the isolated child ran no test — `--exact {test_name}` matched nothing:\n{stdout}"
    );
    false
}

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

// ═══════════════════════════════════════════════════════════════════════════════
//  1. Dual-write routing: Error/Warning → both buffers, others → main only
// ═══════════════════════════════════════════════════════════════════════════════

#[test]
fn publish_log_error_appears_in_both_buffers() {
    let node = uid("dual_error");
    let marker = uid("dual_error_msg");

    publish_log(make_entry(&node, LogType::Error, &marker));

    let main = GLOBAL_LOG_BUFFER.get_all();
    let error = GLOBAL_ERROR_BUFFER.get_all();

    assert!(
        main.iter().any(|e| e.message == marker),
        "Error entry must appear in main buffer"
    );
    assert!(
        error.iter().any(|e| e.message == marker),
        "Error entry must appear in error buffer"
    );
}

#[test]
fn publish_log_warning_appears_in_both_buffers() {
    let node = uid("dual_warn");
    let marker = uid("dual_warn_msg");

    publish_log(make_entry(&node, LogType::Warning, &marker));

    let main = GLOBAL_LOG_BUFFER.get_all();
    let error = GLOBAL_ERROR_BUFFER.get_all();

    assert!(
        main.iter().any(|e| e.message == marker),
        "Warning entry must appear in main buffer"
    );
    assert!(
        error.iter().any(|e| e.message == marker),
        "Warning entry must appear in error buffer"
    );
}

#[test]
fn publish_log_info_only_in_main_buffer() {
    let node = uid("info_only");
    let marker = uid("info_only_msg");

    publish_log(make_entry(&node, LogType::Info, &marker));

    let main = GLOBAL_LOG_BUFFER.get_all();
    let error = GLOBAL_ERROR_BUFFER.get_all();

    assert!(
        main.iter().any(|e| e.message == marker),
        "Info entry must appear in main buffer"
    );
    assert!(
        !error.iter().any(|e| e.message == marker),
        "Info entry must NOT appear in error buffer"
    );
}

#[test]
fn publish_log_publish_only_in_main_buffer() {
    let node = uid("pub_only");
    let marker = uid("pub_only_msg");

    publish_log(LogEntry {
        topic: Some("sensor".to_string()),
        ..make_entry(&node, LogType::Publish, &marker)
    });

    let main = GLOBAL_LOG_BUFFER.get_all();
    let error = GLOBAL_ERROR_BUFFER.get_all();

    assert!(
        main.iter().any(|e| e.message == marker),
        "Publish entry must appear in main buffer"
    );
    assert!(
        !error.iter().any(|e| e.message == marker),
        "Publish entry must NOT appear in error buffer"
    );
}

#[test]
fn publish_log_debug_only_in_main_buffer() {
    let node = uid("debug_only");
    let marker = uid("debug_only_msg");

    publish_log(make_entry(&node, LogType::Debug, &marker));

    let error = GLOBAL_ERROR_BUFFER.get_all();
    assert!(
        !error.iter().any(|e| e.message == marker),
        "Debug entry must NOT appear in error buffer"
    );
}

// ═══════════════════════════════════════════════════════════════════════════════
//  2. THE KEY TEST: errors survive pub/sub flood in error buffer
// ═══════════════════════════════════════════════════════════════════════════════

#[test]
fn errors_survive_pub_sub_flood_in_error_buffer() {
    let _exclusive = exclusive();
    let node = uid("flood_survival");

    // Push 50 error entries via publish_log (dual-write)
    for i in 0..50 {
        publish_log(make_entry(
            &node,
            LogType::Error,
            &format!("{}_error_{}", node, i),
        ));
    }

    // Flood with 6000 pub entries (exceeds main buffer's 5000 capacity)
    for i in 0..6000 {
        publish_log(LogEntry {
            topic: Some("sensor_data".to_string()),
            ..make_entry(&node, LogType::Publish, &format!("{}_pub_{}", node, i))
        });
    }

    // Main buffer: errors likely evicted (FIFO, 6000 pub > 5000 capacity)
    let main_errors = GLOBAL_LOG_BUFFER
        .get_all()
        .iter()
        .filter(|e| e.node_name == node && e.log_type == LogType::Error)
        .count();

    // Error buffer: ALL 50 errors survive (only 50 entries in 500-slot buffer)
    let error_errors = GLOBAL_ERROR_BUFFER
        .get_all()
        .iter()
        .filter(|e| e.node_name == node && e.log_type == LogType::Error)
        .count();

    println!(
        "FLOOD SURVIVAL: main={}/50 errors, error_buffer={}/50 errors",
        main_errors, error_errors
    );

    // When running in parallel with other tests, the 500-slot error buffer may
    // have entries from concurrent tests too. Our 50 errors should be present
    // unless the total error count from all parallel tests exceeds 500.
    // Run with --test-threads=1 for guaranteed 50/50.
    assert!(
        error_errors >= 50 || error_errors > main_errors,
        "Error buffer must retain more errors than main buffer. \
         error_buffer={}/50, main={}/50. \
         If < 50, run with --test-threads=1 to eliminate parallel interference.",
        error_errors,
        main_errors
    );
}

// ═══════════════════════════════════════════════════════════════════════════════
//  3. Error buffer write_idx tracks independently
// ═══════════════════════════════════════════════════════════════════════════════

#[test]
fn error_buffer_write_idx_tracks_independently() {
    let _exclusive = exclusive();
    let node = uid("idx_independent");

    let main_before = GLOBAL_LOG_BUFFER.write_idx();
    let error_before = GLOBAL_ERROR_BUFFER.write_idx();

    // Push Info (main only)
    publish_log(make_entry(&node, LogType::Info, "info_only"));

    let main_after_info = GLOBAL_LOG_BUFFER.write_idx();
    let error_after_info = GLOBAL_ERROR_BUFFER.write_idx();

    assert!(
        main_after_info > main_before,
        "main write_idx should increment after Info"
    );
    // The claim is that *this* Info entry did not reach the error buffer. Test
    // exactly that, by name.
    //
    // Comparing the global write index instead assumes nothing else in the
    // process writes an error between the two snapshots. `exclusive()` orders
    // the tests in this binary against each other, but not against background
    // threads a test left running, and the buffer is process-global: the
    // assertion failed with `1502 != 1497` — five errors that had nothing to do
    // with this test — and reported it as a routing bug.
    assert!(
        GLOBAL_ERROR_BUFFER
            .for_node(&node)
            .iter()
            .all(|e| e.log_type != LogType::Info),
        "an Info entry reached the error buffer — dual-write routing is sending \
         non-errors to the error path"
    );
    assert!(
        error_after_info >= error_before,
        "the error buffer's write index went backwards"
    );

    // Push Error (both)
    publish_log(make_entry(&node, LogType::Error, "error_both"));

    let main_after_error = GLOBAL_LOG_BUFFER.write_idx();
    let error_after_error = GLOBAL_ERROR_BUFFER.write_idx();

    assert!(
        main_after_error > main_after_info,
        "main write_idx should increment after Error"
    );
    assert!(
        error_after_error > error_after_info,
        "error write_idx should increment after Error"
    );
    // ...and by name, so a concurrent writer cannot satisfy this for us.
    assert!(
        GLOBAL_ERROR_BUFFER
            .for_node(&node)
            .iter()
            .any(|e| e.message.contains("error_both")),
        "the Error entry did not reach the error buffer"
    );
}

// ═══════════════════════════════════════════════════════════════════════════════
//  4. Error buffer for_node/for_type filtering works
// ═══════════════════════════════════════════════════════════════════════════════

#[test]
fn error_buffer_for_node_returns_only_errors_for_node() {
    let target = uid("errnode_target");
    let other = uid("errnode_other");

    publish_log(make_entry(&target, LogType::Error, "target error"));
    publish_log(make_entry(&other, LogType::Error, "other error"));

    let filtered = GLOBAL_ERROR_BUFFER.for_node(&target);
    assert!(
        filtered.iter().all(|e| e.node_name == target),
        "for_node on error buffer must filter correctly"
    );
    assert!(!filtered.is_empty(), "should find errors for target node");
}

// ═══════════════════════════════════════════════════════════════════════════════
//  5. Concurrent dual-write — no corruption
// ═══════════════════════════════════════════════════════════════════════════════

#[test]
fn concurrent_dual_write_no_corruption() {
    if !count_in_isolated_child("concurrent_dual_write_no_corruption") {
        return;
    }
    // Serialize against the other tests in this binary. `GLOBAL_ERROR_BUFFER` is
    // a 500-entry ring shared by every test in the process, so a sibling writing
    // errors concurrently can EVICT this test's entries before it counts them —
    // which is how this failed in a full workspace run. The lock at the top of
    // this file exists for exactly this, and its own doc makes the argument: a
    // tolerance wide enough to absorb the race is wide enough to absorb the bug.
    let _guard = exclusive();
    let base = uid("concurrent_dual");
    let barrier = Arc::new(std::sync::Barrier::new(4));

    let handles: Vec<_> = (0..4)
        .map(|t| {
            let b = barrier.clone();
            let node = format!("{}_t{}", base, t);
            std::thread::spawn(move || {
                b.wait();
                for i in 0..10 {
                    publish_log(make_entry(
                        &node,
                        LogType::Error,
                        &format!("err_{}_{}", t, i),
                    ));
                }
            })
        })
        .collect();

    for h in handles {
        h.join().unwrap();
    }

    // 4 threads × 10 errors = 40 total errors (well under 500 capacity)
    let error_entries: Vec<_> = GLOBAL_ERROR_BUFFER
        .get_all()
        .into_iter()
        .filter(|e| e.node_name.contains(&base))
        .collect();

    assert!(
        error_entries.len() >= 40,
        "all 40 concurrent error entries should be in error buffer, found {}",
        error_entries.len()
    );

    // Verify no corruption
    for e in &error_entries {
        assert_eq!(e.log_type, LogType::Error, "all entries should be Error");
        assert!(
            e.node_name.starts_with(&base),
            "node_name should start with base"
        );
    }
}

// ═══════════════════════════════════════════════════════════════════════════════
//  6. Dual-write latency overhead
// ═══════════════════════════════════════════════════════════════════════════════

#[test]
fn dual_write_overhead_acceptable() {
    let _exclusive = exclusive();
    let node = uid("overhead_bench");

    // A single 200-iteration sample of a sub-microsecond operation is mostly
    // noise: one scheduler preemption or allocator slow path lands entirely in
    // whichever half it interrupts. This measured 97.89x on a loaded machine,
    // reporting a 30x regression that did not exist.
    //
    // So: warm up, then take the *minimum* over several rounds. Minimum is the
    // right estimator here because interference can only ever make a sample
    // slower — the fastest round is the one that came closest to running
    // undisturbed. Both halves are measured within each round so a machine that
    // slows down mid-test slows both.
    const ROUNDS: usize = 7;
    const ITERS: usize = 200;

    let bench = |log_type: LogType, tag: &str| -> std::time::Duration {
        let start = std::time::Instant::now();
        for i in 0..ITERS {
            publish_log(make_entry(&node, log_type.clone(), &format!("{tag}_{i}")));
        }
        start.elapsed()
    };

    // Warm up: first-touch page faults and lazy statics belong to neither half.
    bench(LogType::Info, "warmup_info");
    bench(LogType::Error, "warmup_error");

    let mut best_ratio = f64::MAX;
    let mut best_info = std::time::Duration::MAX;
    let mut best_error = std::time::Duration::MAX;
    for _ in 0..ROUNDS {
        // Keep the PAIR from the best round, not the best of each half taken
        // independently. The comment above says both halves are measured within
        // a round so that a machine slowing mid-test slows both — but minimising
        // them separately throws that pairing away: min(error) and min(info) can
        // come from different rounds, and a round that was clean for info and
        // dirty for error inflates the ratio out of nothing. That is what failed
        // under load. Ranking rounds BY RATIO keeps the two halves together.
        let info = bench(LogType::Info, "info");
        let error = bench(LogType::Error, "error");
        let ratio = error.as_nanos() as f64 / info.as_nanos().max(1) as f64;
        if ratio < best_ratio {
            best_ratio = ratio;
            best_info = info;
            best_error = error;
        }
    }

    let info_elapsed = best_info;
    let error_elapsed = best_error;
    let overhead_ratio = best_ratio;

    println!(
        "DUAL-WRITE OVERHEAD (best of {ROUNDS}): Info={:?}, Error={:?}, ratio={overhead_ratio:.2}x",
        info_elapsed, error_elapsed,
    );

    // Error should be < 3x Info (clone + second push)
    assert!(
        overhead_ratio < 3.0,
        "dual-write overhead should be < 3x, got {:.2}x",
        overhead_ratio
    );
}

// ═══════════════════════════════════════════════════════════════════════════════
//  7. Error buffer overflow/wrap at 500 boundary
// ═══════════════════════════════════════════════════════════════════════════════

#[test]
fn error_buffer_wraps_at_500_boundary() {
    if !count_in_isolated_child("error_buffer_wraps_at_500_boundary") {
        return;
    }
    // Serialize against the other tests in this binary. `GLOBAL_ERROR_BUFFER` is
    // a 500-entry ring shared by every test in the process, so a sibling writing
    // errors concurrently can EVICT this test's entries before it counts them —
    // which is how this failed in a full workspace run. The lock at the top of
    // this file exists for exactly this, and its own doc makes the argument: a
    // tolerance wide enough to absorb the race is wide enough to absorb the bug.
    let _guard = exclusive();
    let node = uid("wrap_500");

    // Push 600 errors — exceeds 500-slot capacity
    for i in 0..600u64 {
        publish_log(make_entry(
            &node,
            LogType::Error,
            &format!("{}_wrap_{}", node, i),
        ));
    }

    let all = GLOBAL_ERROR_BUFFER.get_all();
    let our_entries: Vec<_> = all.iter().filter(|e| e.node_name == node).collect();

    // Should have at most 500 entries (ring wraps)
    assert!(
        our_entries.len() <= 500,
        "error buffer should hold at most 500 entries, found {}",
        our_entries.len()
    );

    // Latest entry (599) should be present
    let has_latest = our_entries
        .iter()
        .any(|e| e.message.contains(&format!("{}_wrap_599", node)));
    assert!(has_latest, "latest entry (599) must survive ring wrap");

    // Earliest entry (0) should be evicted
    let has_earliest = our_entries
        .iter()
        .any(|e| e.message.contains(&format!("{}_wrap_0", node)));
    assert!(
        !has_earliest,
        "earliest entry (0) must be evicted after 600 pushes into 500-slot buffer"
    );
}

// ═══════════════════════════════════════════════════════════════════════════════
//  8. Subscribe LogType NOT routed to error buffer
// ═══════════════════════════════════════════════════════════════════════════════

#[test]
fn publish_log_subscribe_not_in_error_buffer() {
    let node = uid("sub_excluded");
    let marker = uid("sub_excluded_msg");

    publish_log(LogEntry {
        topic: Some("sensor_data".to_string()),
        ..make_entry(&node, LogType::Subscribe, &marker)
    });

    let error = GLOBAL_ERROR_BUFFER.get_all();
    assert!(
        !error.iter().any(|e| e.message == marker),
        "Subscribe entry must NOT appear in error buffer"
    );
}

// ═══════════════════════════════════════════════════════════════════════════════
//  9. Mixed Error + Warning batch — correct counts
// ═══════════════════════════════════════════════════════════════════════════════

#[test]
fn mixed_error_warning_batch_correct_counts() {
    if !count_in_isolated_child("mixed_error_warning_batch_correct_counts") {
        return;
    }
    // Serialize against the other tests in this binary. `GLOBAL_ERROR_BUFFER` is
    // a 500-entry ring shared by every test in the process, so a sibling writing
    // errors concurrently can EVICT this test's entries before it counts them —
    // which is how this failed in a full workspace run. The lock at the top of
    // this file exists for exactly this, and its own doc makes the argument: a
    // tolerance wide enough to absorb the race is wide enough to absorb the bug.
    let _guard = exclusive();
    let node = uid("mixed_batch");

    // Push 30 Errors + 20 Warnings
    for i in 0..30 {
        publish_log(make_entry(
            &node,
            LogType::Error,
            &format!("{}_err_{}", node, i),
        ));
    }
    for i in 0..20 {
        publish_log(make_entry(
            &node,
            LogType::Warning,
            &format!("{}_warn_{}", node, i),
        ));
    }

    let all = GLOBAL_ERROR_BUFFER.get_all();
    let our_entries: Vec<_> = all.iter().filter(|e| e.node_name == node).collect();

    assert!(
        our_entries.len() >= 50,
        "error buffer should have 30 errors + 20 warnings = 50, found {}",
        our_entries.len()
    );

    let error_count = our_entries
        .iter()
        .filter(|e| e.log_type == LogType::Error)
        .count();
    let warn_count = our_entries
        .iter()
        .filter(|e| e.log_type == LogType::Warning)
        .count();

    assert!(
        error_count >= 30,
        "should have 30 errors, found {}",
        error_count
    );
    assert!(
        warn_count >= 20,
        "should have 20 warnings, found {}",
        warn_count
    );
}

// ═══════════════════════════════════════════════════════════════════════════════
//  10. Error buffer for_topic filtering
// ═══════════════════════════════════════════════════════════════════════════════

#[test]
fn error_buffer_for_topic_filtering() {
    let node = uid("errtopic");
    let target_topic = uid("motor_fault");

    // Push Error with topic
    publish_log(LogEntry {
        topic: Some(target_topic.clone()),
        ..make_entry(&node, LogType::Error, "error with topic")
    });
    // Push Error without topic
    publish_log(make_entry(&node, LogType::Error, "error no topic"));

    let filtered = GLOBAL_ERROR_BUFFER.for_topic(&target_topic);
    assert!(
        !filtered.is_empty(),
        "for_topic on error buffer should find the entry with topic"
    );
    assert!(
        filtered
            .iter()
            .all(|e| e.topic.as_ref() == Some(&target_topic)),
        "all filtered entries must have matching topic"
    );
}

// ═══════════════════════════════════════════════════════════════════════════════
//  11. Unicode error message roundtrip
// ═══════════════════════════════════════════════════════════════════════════════

#[test]
fn error_buffer_unicode_message_roundtrip() {
    let node = uid("unicode_err");
    let unicode_msg = format!("{}_Motor stall: héllo wörld", node);

    publish_log(make_entry(&node, LogType::Error, &unicode_msg));

    let all = GLOBAL_ERROR_BUFFER.get_all();
    let entry = all
        .iter()
        .find(|e| e.node_name == node && e.message.contains("héllo"));
    assert!(
        entry.is_some(),
        "unicode error message must round-trip through error buffer"
    );
    assert!(
        entry.unwrap().message.contains("wörld"),
        "unicode characters must be preserved"
    );
}

// ═══════════════════════════════════════════════════════════════════════════════
//  12. Long error message truncated correctly
// ═══════════════════════════════════════════════════════════════════════════════

#[test]
fn error_buffer_long_message_truncated() {
    let node = uid("truncate_err");
    let long_msg = format!("{}_{}", node, "X".repeat(1000));

    publish_log(make_entry(&node, LogType::Error, &long_msg));

    let all = GLOBAL_ERROR_BUFFER.get_all();
    let entry = all.iter().find(|e| e.node_name == node);
    assert!(entry.is_some(), "truncated entry must still be in buffer");

    let msg = &entry.unwrap().message;
    // MAX_MESSAGE_LEN = 280, truncation adds "..." (3 bytes)
    assert!(
        msg.len() <= 283,
        "message should be truncated to <= 283 bytes, got {}",
        msg.len()
    );
    assert!(
        msg.ends_with("..."),
        "truncated message should end with '...'"
    );
}
