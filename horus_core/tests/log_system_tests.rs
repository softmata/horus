//! Phases 11–12: Logging core and features integration tests.
//!
//! Covers ring buffer overflow at 5000 boundary, cross-process concurrent writes,
//! message truncation, HorusLogBridge routing, mmap cleanup/recreation, seqlock
//! concurrent read/write, log bridge idempotency, hlog macros (hlog_every!,
//! hlog_once!), and node context propagation.

use horus_core::core::hlog::{
    clear_node_context, current_node_name, current_tick_number, log_with_context, set_node_context,
};
use horus_core::core::log_buffer::{LogEntry, LogType, SharedLogBuffer};
use std::sync::Arc;

// ─── Helpers ────────────────────────────────────────────────────────────────

fn make_entry(node: &str, tick: u64, msg: &str) -> LogEntry {
    LogEntry {
        timestamp: format!("12:00:{:02}.000", tick % 60),
        tick_number: tick,
        node_name: node.to_string(),
        log_type: LogType::Info,
        topic: None,
        message: msg.to_string(),
        tick_us: 0,
        ipc_ns: 0,
    }
}

fn make_entry_with_type(node: &str, tick: u64, log_type: LogType) -> LogEntry {
    LogEntry {
        timestamp: format!("12:00:{:02}.000", tick % 60),
        tick_number: tick,
        node_name: node.to_string(),
        log_type,
        topic: None,
        message: format!("entry_{}", tick),
        tick_us: tick * 10,
        ipc_ns: tick * 100,
    }
}

fn temp_buf(test_id: &str) -> (SharedLogBuffer, std::path::PathBuf) {
    let path = std::env::temp_dir().join(format!(
        "horus_log_sys_test_{}_{}.bin",
        std::process::id(),
        test_id
    ));
    // Remove any stale file from a previous crashed test run
    let _ = std::fs::remove_file(&path);
    let buf = SharedLogBuffer::new_at_path(&path).unwrap();
    (buf, path)
}

// ═══════════════════════════════════════════════════════════════════════════════
//  Phase 11: Ring buffer overflow/wrap at exactly 5000 entries
// ═══════════════════════════════════════════════════════════════════════════════

#[test]
fn ring_buffer_exact_5000_boundary() {
    let (buf, path) = temp_buf("p11_5000");

    // Push exactly 5000 entries (MAX_LOG_ENTRIES)
    for i in 0..5000u64 {
        buf.push(make_entry("boundary_node", i, &format!("msg_{}", i)));
    }

    assert_eq!(buf.write_idx(), 5000);
    let all = buf.get_all();
    assert_eq!(all.len(), 5000, "exactly 5000 entries should fit");

    // All entries should be readable
    for e in &all {
        assert_eq!(e.node_name, "boundary_node");
    }

    let _ = std::fs::remove_file(path);
}

#[test]
fn ring_buffer_wrap_at_5001() {
    let (buf, path) = temp_buf("p11_5001");

    // Push 5001 entries — first entry is overwritten
    for i in 0..5001u64 {
        buf.push(make_entry("wrap_node", i, &format!("msg_{}", i)));
    }

    assert_eq!(buf.write_idx(), 5001);
    let all = buf.get_all();
    assert_eq!(all.len(), 5000, "should return at most 5000 after wrap");

    // The oldest entry (tick 0) should be gone, newest (tick 5000) should be present
    assert!(
        all.iter().any(|e| e.tick_number == 5000),
        "newest entry (tick 5000) must be present"
    );
    // After wrap, tick 0 is overwritten by tick 5000 in the same slot
    assert!(
        !all.iter().any(|e| e.tick_number == 0),
        "oldest entry (tick 0) should be overwritten after wrap"
    );

    let _ = std::fs::remove_file(path);
}

#[test]
fn ring_buffer_double_wrap() {
    let (buf, path) = temp_buf("p11_double_wrap");

    // Push 12000 entries — wraps more than twice
    for i in 0..12000u64 {
        buf.push(make_entry("dbl_wrap", i, &format!("m{}", i)));
    }

    assert_eq!(buf.write_idx(), 12000);
    let all = buf.get_all();
    assert_eq!(all.len(), 5000);

    // Should contain entries 7000–11999 (the last 5000)
    let min_tick = all.iter().map(|e| e.tick_number).min().unwrap();
    let max_tick = all.iter().map(|e| e.tick_number).max().unwrap();
    assert_eq!(max_tick, 11999);
    assert!(
        min_tick >= 7000,
        "oldest tick should be >= 7000, got {}",
        min_tick
    );

    let _ = std::fs::remove_file(path);
}

// ═══════════════════════════════════════════════════════════════════════════════
//  Phase 11: Cross-process concurrent writes (simulated via threads)
// ═══════════════════════════════════════════════════════════════════════════════

#[test]
fn concurrent_writes_16_threads_500_each() {
    let (buf, path) = temp_buf("p11_conc16");
    let buf = Arc::new(buf);

    const THREADS: usize = 16;
    const PER_THREAD: usize = 500;
    const TOTAL: usize = THREADS * PER_THREAD;

    let handles: Vec<_> = (0..THREADS)
        .map(|t| {
            let b = buf.clone();
            std::thread::spawn(move || {
                for i in 0..PER_THREAD {
                    b.push(make_entry(
                        &format!("t{}", t),
                        (t * PER_THREAD + i) as u64,
                        "concurrent",
                    ));
                }
            })
        })
        .collect();

    for h in handles {
        h.join().unwrap();
    }

    assert_eq!(buf.write_idx(), TOTAL as u64);

    let all = buf.get_all();
    // Total is 8000 > 5000, so only last 5000 are readable
    assert_eq!(all.len(), 5000);

    // All tick numbers in the output should be unique
    let mut seen = std::collections::HashSet::new();
    for e in &all {
        assert!(
            seen.insert(e.tick_number),
            "duplicate tick {}",
            e.tick_number
        );
    }

    let _ = std::fs::remove_file(path);
}

#[test]
fn concurrent_read_write_no_panic() {
    // Stress: readers and writers simultaneously
    let (buf, path) = temp_buf("p11_rw_conc");
    let buf = Arc::new(buf);

    let writer_buf = buf.clone();
    let writer = std::thread::spawn(move || {
        for i in 0..2000u64 {
            writer_buf.push(make_entry("writer", i, "data"));
        }
    });

    // Multiple reader threads
    let readers: Vec<_> = (0..4)
        .map(|_| {
            let b = buf.clone();
            std::thread::spawn(move || {
                for _ in 0..100 {
                    let entries = b.get_all();
                    // All entries must be valid (no torn reads)
                    for e in &entries {
                        assert!(!e.node_name.is_empty(), "torn read: empty node_name");
                    }
                    std::thread::yield_now();
                }
            })
        })
        .collect();

    writer.join().unwrap();
    for r in readers {
        r.join().unwrap();
    }

    let _ = std::fs::remove_file(path);
}

// ═══════════════════════════════════════════════════════════════════════════════
//  Phase 11: Message truncation at 300-byte limit
// ═══════════════════════════════════════════════════════════════════════════════

#[test]
fn truncation_exactly_300_bytes() {
    let (buf, path) = temp_buf("p11_trunc300");

    // Exactly 300 bytes — should NOT be truncated
    let msg_300 = "X".repeat(300);
    buf.push(make_entry("trunc", 0, &msg_300));

    let all = buf.get_all();
    assert_eq!(all.len(), 1);
    assert_eq!(
        all[0].message.len(),
        300,
        "300-byte message should not be truncated"
    );
    assert!(!all[0].message.ends_with("..."));

    let _ = std::fs::remove_file(path);
}

#[test]
fn truncation_301_bytes() {
    let (buf, path) = temp_buf("p11_trunc301");

    // 301 bytes — should be truncated to 300 with "..."
    let msg_301 = "Y".repeat(301);
    buf.push(make_entry("trunc", 0, &msg_301));

    let all = buf.get_all();
    assert_eq!(all.len(), 1);
    assert!(all[0].message.len() <= 300, "must be truncated to <= 300");
    assert!(all[0].message.ends_with("..."), "must end with '...'");

    let _ = std::fs::remove_file(path);
}

#[test]
fn truncation_unicode_boundary() {
    let (buf, path) = temp_buf("p11_trunc_uni");

    // Multi-byte unicode near the boundary — must not split a codepoint
    let mut msg = "A".repeat(296);
    msg.push('🤖'); // 4-byte character at position 296–299
    msg.push('X'); // position 300 — triggers truncation
    assert!(msg.len() > 300);

    buf.push(make_entry("trunc_uni", 0, &msg));

    let all = buf.get_all();
    assert_eq!(all.len(), 1);
    // Must be valid UTF-8 (no panics from string operations)
    assert!(all[0].message.is_char_boundary(all[0].message.len()));
    assert!(all[0].message.ends_with("..."));

    let _ = std::fs::remove_file(path);
}

// ═══════════════════════════════════════════════════════════════════════════════
//  Phase 11: HorusLogBridge routes log:: calls to ring buffer
// ═══════════════════════════════════════════════════════════════════════════════

#[test]
fn log_bridge_routes_to_global_buffer() {
    // Initialize the bridge (may already be set — try_init is a no-op then)
    horus_core::core::log_bridge::try_init_log_bridge("debug");

    // Record write_idx before logging
    let idx_before = horus_core::core::log_buffer::GLOBAL_LOG_BUFFER.write_idx();

    // Use log:: crate to emit a message
    log::info!(target: "bridge_test_node", "bridge routing test message");

    // write_idx should have increased
    let idx_after = horus_core::core::log_buffer::GLOBAL_LOG_BUFFER.write_idx();
    assert!(
        idx_after > idx_before,
        "write_idx must increase after log::info! (bridge must route to buffer)"
    );

    // The entry should be findable in the buffer
    let all = horus_core::core::log_buffer::GLOBAL_LOG_BUFFER.get_all();
    let found = all
        .iter()
        .any(|e| e.node_name == "bridge_test_node" && e.message.contains("bridge routing test"));
    assert!(found, "log::info! message must appear in GLOBAL_LOG_BUFFER");
}

#[test]
fn log_bridge_level_mapping() {
    horus_core::core::log_bridge::try_init_log_bridge("debug");
    // Ensure max level is Debug so all levels are forwarded
    log::set_max_level(log::LevelFilter::Debug);

    log::error!(target: "level_map_node", "error level test");
    log::warn!(target: "level_map_node", "warn level test");
    log::info!(target: "level_map_node", "info level test");
    log::debug!(target: "level_map_node", "debug level test");

    let all = horus_core::core::log_buffer::GLOBAL_LOG_BUFFER.get_all();
    let level_entries: Vec<_> = all
        .iter()
        .filter(|e| e.node_name == "level_map_node")
        .collect();

    // Verify level mapping
    let has_error = level_entries.iter().any(|e| e.log_type == LogType::Error);
    let has_warn = level_entries.iter().any(|e| e.log_type == LogType::Warning);
    let has_info = level_entries.iter().any(|e| e.log_type == LogType::Info);
    let has_debug = level_entries.iter().any(|e| e.log_type == LogType::Debug);

    assert!(has_error, "log::error! should map to LogType::Error");
    assert!(has_warn, "log::warn! should map to LogType::Warning");
    assert!(has_info, "log::info! should map to LogType::Info");
    assert!(has_debug, "log::debug! should map to LogType::Debug");
}

// ═══════════════════════════════════════════════════════════════════════════════
//  Phase 11: mmap file cleanup and recreation after crash
// ═══════════════════════════════════════════════════════════════════════════════

#[test]
fn mmap_recreation_after_deletion() {
    let path = std::env::temp_dir().join(format!(
        "horus_log_mmap_recreate_{}.bin",
        std::process::id()
    ));
    let _ = std::fs::remove_file(&path);

    // Create buffer and write some data
    {
        let buf = SharedLogBuffer::new_at_path(&path).unwrap();
        buf.push(make_entry("pre_crash", 1, "before crash"));
        assert_eq!(buf.write_idx(), 1);
    }
    // Buffer dropped, but file persists

    // Simulate crash cleanup: delete the mmap file
    assert!(path.exists(), "mmap file should exist after writes");
    std::fs::remove_file(&path).unwrap();
    assert!(!path.exists(), "mmap file should be deleted");

    // Re-create buffer from scratch — simulates restart after crash
    let buf2 = SharedLogBuffer::new_at_path(&path).unwrap();
    assert_eq!(
        buf2.write_idx(),
        0,
        "fresh buffer after recreation must start at 0"
    );

    let all = buf2.get_all();
    assert!(all.is_empty(), "fresh buffer must be empty");

    // New writes should work normally
    buf2.push(make_entry("post_crash", 1, "after crash"));
    assert_eq!(buf2.write_idx(), 1);
    let all = buf2.get_all();
    assert_eq!(all.len(), 1);
    assert_eq!(all[0].node_name, "post_crash");

    let _ = std::fs::remove_file(path);
}

#[test]
fn mmap_reopen_preserves_data() {
    let path =
        std::env::temp_dir().join(format!("horus_log_mmap_reopen_{}.bin", std::process::id()));
    let _ = std::fs::remove_file(&path);

    // Write data with first buffer instance
    {
        let buf = SharedLogBuffer::new_at_path(&path).unwrap();
        for i in 0..10u64 {
            buf.push(make_entry("persist_node", i, &format!("msg_{}", i)));
        }
    }

    // Open a second instance of the same file (simulates another process or restart)
    let buf2 = SharedLogBuffer::new_at_path(&path).unwrap();
    let all = buf2.get_all();
    assert_eq!(all.len(), 10, "data must persist across re-open");

    // Verify content integrity, not just count
    for (i, entry) in all.iter().enumerate() {
        assert_eq!(
            entry.node_name, "persist_node",
            "entry {} node_name must survive re-open",
            i
        );
        assert_eq!(
            entry.message,
            format!("msg_{}", i),
            "entry {} message must survive re-open",
            i
        );
        assert_eq!(
            entry.tick_number, i as u64,
            "entry {} tick_number must survive re-open",
            i
        );
    }

    let _ = std::fs::remove_file(path);
}

// ═══════════════════════════════════════════════════════════════════════════════
//  Phase 11: Seqlock correctness under concurrent read/write
// ═══════════════════════════════════════════════════════════════════════════════

#[test]
fn seqlock_concurrent_readers_never_see_torn_data() {
    let (buf, path) = temp_buf("p11_seqlock");
    let buf = Arc::new(buf);

    // Pre-fill some data
    for i in 0..100u64 {
        buf.push(make_entry("seqlock_node", i, &format!("data_{}", i)));
    }

    // Spawn writer and multiple readers concurrently
    let writer_buf = buf.clone();
    let writer = std::thread::spawn(move || {
        for i in 100..3000u64 {
            writer_buf.push(make_entry("seqlock_node", i, &format!("data_{}", i)));
        }
    });

    let mut reader_handles = vec![];
    for r in 0..8 {
        let b = buf.clone();
        reader_handles.push(std::thread::spawn(move || {
            let mut reads = 0u64;
            for _ in 0..200 {
                let entries = b.get_all();
                for e in &entries {
                    // Verify no torn reads: all fields should be consistent
                    assert_eq!(e.node_name, "seqlock_node", "reader {}: torn node_name", r);
                    assert!(
                        e.message.starts_with("data_"),
                        "reader {}: torn message: {}",
                        r,
                        e.message
                    );
                    reads += 1;
                }
            }
            reads
        }));
    }

    writer.join().unwrap();
    let total_reads: u64 = reader_handles.into_iter().map(|h| h.join().unwrap()).sum();
    assert!(total_reads > 0, "readers must have read entries");

    let _ = std::fs::remove_file(path);
}

// ═══════════════════════════════════════════════════════════════════════════════
//  Phase 11: Log bridge initialization idempotency
// ═══════════════════════════════════════════════════════════════════════════════

#[test]
fn log_bridge_try_init_idempotent() {
    // First call succeeds or is already initialized
    horus_core::core::log_bridge::try_init_log_bridge("info");

    // Subsequent calls must not panic
    horus_core::core::log_bridge::try_init_log_bridge("debug");
    horus_core::core::log_bridge::try_init_log_bridge("error");
    horus_core::core::log_bridge::try_init_log_bridge("warn");

    // Logging should still work
    let idx_before = horus_core::core::log_buffer::GLOBAL_LOG_BUFFER.write_idx();
    log::warn!(target: "idempotent_test", "still works after multiple inits");
    let idx_after = horus_core::core::log_buffer::GLOBAL_LOG_BUFFER.write_idx();
    assert!(idx_after > idx_before);
}

#[test]
fn log_bridge_init_returns_err_on_second_call() {
    // Ensure logger is installed first
    horus_core::core::log_bridge::try_init_log_bridge("info");

    // Now init_log_bridge should return Err (logger already set)
    let result = horus_core::core::log_bridge::init_log_bridge("info");
    assert!(
        result.is_err(),
        "init_log_bridge should return Err if logger already set"
    );
}

// ═══════════════════════════════════════════════════════════════════════════════
//  Phase 11: LogEntry serialization edge cases (additional)
// ═══════════════════════════════════════════════════════════════════════════════

#[test]
fn serialization_all_log_types() {
    let (buf, path) = temp_buf("p11_all_types");

    let types = [
        LogType::Info,
        LogType::Warning,
        LogType::Error,
        LogType::Debug,
        LogType::Publish,
        LogType::Subscribe,
    ];

    for (i, lt) in types.iter().enumerate() {
        buf.push(make_entry_with_type("type_node", i as u64, lt.clone()));
    }

    let all = buf.get_all();
    assert_eq!(all.len(), 6);

    // Verify each type round-trips correctly
    for (i, lt) in types.iter().enumerate() {
        let entry = all.iter().find(|e| e.tick_number == i as u64).unwrap();
        assert_eq!(&entry.log_type, lt, "LogType must round-trip for {:?}", lt);
    }

    let _ = std::fs::remove_file(path);
}

#[test]
fn serialization_max_field_sizes() {
    let (buf, path) = temp_buf("p11_max_fields");

    // Push entry with fields sized to fit within 504-byte slot data limit
    // Serialized JSON overhead (keys, braces, commas) ≈ ~150 bytes
    // So keep total content ≈ 350 bytes
    let entry = LogEntry {
        timestamp: "23:59:59.999".to_string(),
        tick_number: u64::MAX,
        node_name: "n".repeat(30),
        log_type: LogType::Error,
        topic: Some("t".repeat(30)),
        message: "m".repeat(200),
        tick_us: u64::MAX,
        ipc_ns: u64::MAX,
    };
    buf.push(entry);

    let all = buf.get_all();
    assert_eq!(all.len(), 1);
    assert_eq!(all[0].tick_number, u64::MAX);
    assert_eq!(all[0].tick_us, u64::MAX);
    assert_eq!(all[0].ipc_ns, u64::MAX);

    let _ = std::fs::remove_file(path);
}

// ═══════════════════════════════════════════════════════════════════════════════
//  Phase 12: hlog! node context propagation through scheduler
// ═══════════════════════════════════════════════════════════════════════════════

#[test]
fn hlog_context_propagation() {
    // Simulate scheduler setting context before node tick
    set_node_context("lidar_node", 42);

    assert_eq!(current_node_name(), "lidar_node");
    assert_eq!(current_tick_number(), 42);

    let idx_before = horus_core::core::log_buffer::GLOBAL_LOG_BUFFER.write_idx();

    // hlog! should capture the node context
    log_with_context(LogType::Info, "context propagation test".to_string());

    let idx_after = horus_core::core::log_buffer::GLOBAL_LOG_BUFFER.write_idx();
    assert!(idx_after > idx_before);

    let all = horus_core::core::log_buffer::GLOBAL_LOG_BUFFER.get_all();
    let found = all
        .iter()
        .rev()
        .find(|e| e.node_name == "lidar_node" && e.message.contains("context propagation test"));
    assert!(found.is_some(), "hlog entry must capture node context");
    let entry = found.unwrap();
    assert_eq!(entry.tick_number, 42);
    // tick_us is elapsed microseconds since set_node_context — should be small
    assert!(
        entry.tick_us < 1_000_000,
        "tick_us should be < 1s (elapsed since context set), got {}us",
        entry.tick_us
    );
    assert_eq!(entry.log_type, LogType::Info, "log type must be Info");

    clear_node_context();
}

#[test]
fn hlog_without_context_uses_unknown() {
    clear_node_context();
    assert_eq!(current_node_name(), "unknown");

    let idx_before = horus_core::core::log_buffer::GLOBAL_LOG_BUFFER.write_idx();
    log_with_context(LogType::Warning, "no context warning".to_string());
    let idx_after = horus_core::core::log_buffer::GLOBAL_LOG_BUFFER.write_idx();
    assert!(idx_after > idx_before);

    let all = horus_core::core::log_buffer::GLOBAL_LOG_BUFFER.get_all();
    let found = all
        .iter()
        .rev()
        .find(|e| e.message.contains("no context warning"));
    assert!(found.is_some());
    assert_eq!(found.unwrap().node_name, "unknown");
}

#[test]
fn hlog_context_switch_between_nodes() {
    // Simulate scheduler switching between nodes
    let idx_before = horus_core::core::log_buffer::GLOBAL_LOG_BUFFER.write_idx();

    set_node_context("node_a", 10);
    log_with_context(LogType::Info, "from node_a".to_string());

    set_node_context("node_b", 20);
    log_with_context(LogType::Info, "from node_b".to_string());

    clear_node_context();

    let idx_after = horus_core::core::log_buffer::GLOBAL_LOG_BUFFER.write_idx();
    assert!(idx_after >= idx_before + 2);

    let all = horus_core::core::log_buffer::GLOBAL_LOG_BUFFER.get_all();
    let a = all.iter().rev().find(|e| e.message == "from node_a");
    let b = all.iter().rev().find(|e| e.message == "from node_b");

    assert!(a.is_some());
    assert!(b.is_some());
    assert_eq!(a.unwrap().node_name, "node_a");
    assert_eq!(b.unwrap().node_name, "node_b");
    assert_eq!(a.unwrap().tick_number, 10);
    assert_eq!(b.unwrap().tick_number, 20);
}

// ═══════════════════════════════════════════════════════════════════════════════
//  Phase 12: hlog_every! throttle accuracy
// ═══════════════════════════════════════════════════════════════════════════════

#[test]
fn hlog_every_throttle() {
    set_node_context("throttle_node", 1);

    // Call hlog_every! with 5000ms interval 10 times in rapid succession
    // Only the FIRST call should produce a log entry (subsequent ones within interval)
    for i in 0..10 {
        horus_core::hlog_every!(5000, info, "throttled_unique_marker_{}", i);
    }

    let all = horus_core::core::log_buffer::GLOBAL_LOG_BUFFER.get_all();
    let throttled_entries: Vec<_> = all
        .iter()
        .filter(|e| e.message.starts_with("throttled_unique_marker_"))
        .collect();

    // Should have exactly 1 entry (first call logs, rest are throttled within 5s)
    assert_eq!(
        throttled_entries.len(),
        1,
        "hlog_every! with 5s interval should only log once in rapid succession, got {}",
        throttled_entries.len()
    );
    // The first message should be "throttled_unique_marker_0"
    assert!(throttled_entries[0].message.contains("_0"));

    clear_node_context();
}

// ═══════════════════════════════════════════════════════════════════════════════
//  Phase 12: hlog_once! deduplication
// ═══════════════════════════════════════════════════════════════════════════════

#[test]
fn hlog_once_deduplication() {
    set_node_context("once_node", 1);

    // Call hlog_once! multiple times — only first should log
    for _ in 0..10 {
        horus_core::hlog_once!(info, "once_unique_calibration_marker");
    }

    let all = horus_core::core::log_buffer::GLOBAL_LOG_BUFFER.get_all();
    let once_entries: Vec<_> = all
        .iter()
        .filter(|e| e.message == "once_unique_calibration_marker")
        .collect();

    assert_eq!(
        once_entries.len(),
        1,
        "hlog_once! should produce exactly 1 entry across 10 calls, got {}",
        once_entries.len()
    );

    clear_node_context();
}

// ═══════════════════════════════════════════════════════════════════════════════
//  Phase 12: stderr mirror output format
// ═══════════════════════════════════════════════════════════════════════════════

#[test]
fn stderr_mirror_does_not_panic() {
    set_node_context("stderr_test", 1);

    let idx_before = horus_core::core::log_buffer::GLOBAL_LOG_BUFFER.write_idx();

    // All log types should produce output without panicking and write to buffer
    log_with_context(LogType::Info, "stderr_info_verify".to_string());
    log_with_context(LogType::Warning, "stderr_warn_verify".to_string());
    log_with_context(LogType::Error, "stderr_error_verify".to_string());
    log_with_context(LogType::Debug, "stderr_debug_verify".to_string());
    // Publish/Subscribe should NOT mirror to stderr (only shared memory)
    log_with_context(LogType::Publish, "stderr_pub_verify".to_string());
    log_with_context(LogType::Subscribe, "stderr_sub_verify".to_string());

    let idx_after = horus_core::core::log_buffer::GLOBAL_LOG_BUFFER.write_idx();

    // All 6 log calls must write to the buffer
    assert!(
        idx_after >= idx_before + 6,
        "all 6 log_with_context calls must write to buffer, got {} increase",
        idx_after - idx_before
    );

    // Verify each entry is in the buffer with the correct log type
    let all = horus_core::core::log_buffer::GLOBAL_LOG_BUFFER.get_all();
    let stderr_entries: Vec<_> = all
        .iter()
        .filter(|e| e.message.starts_with("stderr_") && e.message.ends_with("_verify"))
        .collect();

    assert!(
        stderr_entries.len() >= 6,
        "should have at least 6 stderr_*_verify entries, got {}",
        stderr_entries.len()
    );

    // Mirrored types (Info, Warning, Error, Debug) must have correct log types
    assert!(
        stderr_entries
            .iter()
            .any(|e| e.message == "stderr_info_verify" && e.log_type == LogType::Info),
        "Info log must be in buffer with correct type"
    );
    assert!(
        stderr_entries
            .iter()
            .any(|e| e.message == "stderr_warn_verify" && e.log_type == LogType::Warning),
        "Warning log must be in buffer with correct type"
    );
    assert!(
        stderr_entries
            .iter()
            .any(|e| e.message == "stderr_error_verify" && e.log_type == LogType::Error),
        "Error log must be in buffer with correct type"
    );
    assert!(
        stderr_entries
            .iter()
            .any(|e| e.message == "stderr_debug_verify" && e.log_type == LogType::Debug),
        "Debug log must be in buffer with correct type"
    );
    // Non-mirrored types must still be in buffer (even though not on stderr)
    assert!(
        stderr_entries
            .iter()
            .any(|e| e.message == "stderr_pub_verify" && e.log_type == LogType::Publish),
        "Publish log must be in buffer even without stderr mirror"
    );
    assert!(
        stderr_entries
            .iter()
            .any(|e| e.message == "stderr_sub_verify" && e.log_type == LogType::Subscribe),
        "Subscribe log must be in buffer even without stderr mirror"
    );

    clear_node_context();
}

// ═══════════════════════════════════════════════════════════════════════════════
//  Phase 12: for_node and for_topic filtering
// ═══════════════════════════════════════════════════════════════════════════════

#[test]
fn for_node_filters_correctly() {
    let (buf, path) = temp_buf("p12_for_node");

    buf.push(make_entry("alpha", 0, "alpha msg"));
    buf.push(make_entry("beta", 1, "beta msg"));
    buf.push(make_entry("alpha", 2, "alpha msg 2"));

    let alpha = buf.for_node("alpha");
    assert_eq!(alpha.len(), 2);
    assert!(alpha.iter().all(|e| e.node_name == "alpha"));

    let beta = buf.for_node("beta");
    assert_eq!(beta.len(), 1);

    let none = buf.for_node("gamma");
    assert!(none.is_empty());

    let _ = std::fs::remove_file(path);
}

#[test]
fn for_topic_filters_correctly() {
    let (buf, path) = temp_buf("p12_for_topic");

    buf.push(LogEntry {
        timestamp: "12:00:00.000".to_string(),
        tick_number: 0,
        node_name: "pub_node".to_string(),
        log_type: LogType::Publish,
        topic: Some("camera_rgb".to_string()),
        message: "published frame".to_string(),
        tick_us: 0,
        ipc_ns: 100,
    });
    buf.push(LogEntry {
        timestamp: "12:00:01.000".to_string(),
        tick_number: 1,
        node_name: "sub_node".to_string(),
        log_type: LogType::Subscribe,
        topic: Some("lidar_scan".to_string()),
        message: "received scan".to_string(),
        tick_us: 0,
        ipc_ns: 200,
    });
    buf.push(make_entry("no_topic", 2, "no topic entry"));

    let camera = buf.for_topic("camera_rgb");
    assert_eq!(camera.len(), 1);
    assert_eq!(camera[0].topic.as_deref(), Some("camera_rgb"));

    let lidar = buf.for_topic("lidar_scan");
    assert_eq!(lidar.len(), 1);

    let none = buf.for_topic("nonexistent");
    assert!(none.is_empty());

    let _ = std::fs::remove_file(path);
}
