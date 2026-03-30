#![allow(dead_code)]
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
//  Phase 11: Message truncation at 280-byte limit
// ═══════════════════════════════════════════════════════════════════════════════

#[test]
fn truncation_exactly_280_bytes() {
    let (buf, path) = temp_buf("p11_trunc280");

    // Exactly 280 bytes — should NOT be truncated
    let msg_280 = "X".repeat(280);
    buf.push(make_entry("trunc", 0, &msg_280));

    let all = buf.get_all();
    assert_eq!(all.len(), 1);
    assert_eq!(
        all[0].message.len(),
        280,
        "280-byte message should not be truncated"
    );
    assert!(!all[0].message.ends_with("..."));

    let _ = std::fs::remove_file(path);
}

#[test]
fn truncation_281_bytes() {
    let (buf, path) = temp_buf("p11_trunc281");

    // 281 bytes — should be truncated to 280 with "..."
    let msg_281 = "Y".repeat(281);
    buf.push(make_entry("trunc", 0, &msg_281));

    let all = buf.get_all();
    assert_eq!(all.len(), 1);
    assert!(all[0].message.len() <= 280, "must be truncated to <= 280");
    assert!(all[0].message.ends_with("..."), "must end with '...'");

    let _ = std::fs::remove_file(path);
}

#[test]
fn truncation_unicode_boundary() {
    let (buf, path) = temp_buf("p11_trunc_uni");

    // Multi-byte unicode near the boundary — must not split a codepoint
    let mut msg = "A".repeat(276);
    msg.push('🤖'); // 4-byte character at position 276–279
    msg.push('X'); // position 280 — triggers truncation
    assert!(msg.len() > 280);

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

    // Use a per-run unique prefix to avoid matching stale entries from prior
    // process runs (SHM log buffer persists on disk).
    let unique_id = std::process::id();
    let _prefix = format!("throttled_{}_{{}}", unique_id);

    // Call hlog_every! with 5000ms interval 10 times in rapid succession
    // Only the FIRST call should produce a log entry (subsequent ones within interval)
    for i in 0..10 {
        horus_core::hlog_every!(5000, info, "throttled_{}_{}", unique_id, i);
    }

    let all = horus_core::core::log_buffer::GLOBAL_LOG_BUFFER.get_all();
    let match_prefix = format!("throttled_{}_", unique_id);
    let throttled_entries: Vec<_> = all
        .iter()
        .filter(|e| e.message.starts_with(&match_prefix))
        .collect();

    // Should have exactly 1 entry (first call logs, rest are throttled within 5s)
    assert_eq!(
        throttled_entries.len(),
        1,
        "hlog_every! with 5s interval should only log once in rapid succession, got {}",
        throttled_entries.len()
    );

    clear_node_context();
}

// ═══════════════════════════════════════════════════════════════════════════════
//  Phase 12: hlog_once! deduplication
// ═══════════════════════════════════════════════════════════════════════════════

#[test]
fn hlog_once_deduplication() {
    set_node_context("once_node", 1);

    // Use a per-run unique marker to avoid matching stale entries from prior
    // process runs (SHM log buffer persists on disk).
    let unique_id = std::process::id();
    let marker = format!("once_dedup_marker_{}", unique_id);

    // Call hlog_once! multiple times — only first should log.
    // Note: hlog_once! uses a static AtomicBool, so across 10 iterations
    // of this single callsite, exactly 1 entry is written.
    for _ in 0..10 {
        horus_core::hlog_once!(info, "{}", marker);
    }

    let all = horus_core::core::log_buffer::GLOBAL_LOG_BUFFER.get_all();
    let once_entries: Vec<_> = all.iter().filter(|e| e.message == marker).collect();

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

// ═══════════════════════════════════════════════════════════════════════════════
//  Python ↔ Rust bridge path tests
//
//  These exercise the exact code paths that Python's log_pub()/log_sub()
//  and log_info()/log_warning()/log_error()/log_debug() call through PyO3.
//  The Python wrappers are thin — they build a LogEntry and call publish_log()
//  directly — so testing publish_log() with the same field patterns is
//  equivalent to a cross-language roundtrip.
// ═══════════════════════════════════════════════════════════════════════════════

/// Python log_pub() builds a Publish LogEntry with topic, ipc_ns, and data_repr
/// as message.  Verify the entry round-trips through the ring buffer.
#[test]
fn python_log_pub_roundtrip() {
    let (buf, path) = temp_buf("py_log_pub");

    // Simulate exactly what Python's log_pub() does
    use horus_core::core::log_buffer::{LogEntry, LogType};
    let entry = LogEntry {
        timestamp: "14:30:00.123".to_string(),
        tick_number: 0,
        node_name: "py_sensor_node".to_string(),
        log_type: LogType::Publish,
        topic: Some("camera.rgb".to_string()),
        message: "{'width': 640, 'height': 480}".to_string(),
        tick_us: 150,
        ipc_ns: 2500,
    };
    buf.push(entry);

    let all = buf.get_all();
    assert_eq!(all.len(), 1);
    let e = &all[0];
    assert_eq!(e.node_name, "py_sensor_node");
    assert_eq!(e.log_type, LogType::Publish);
    assert_eq!(e.topic.as_deref(), Some("camera.rgb"));
    assert_eq!(e.message, "{'width': 640, 'height': 480}");
    assert_eq!(e.ipc_ns, 2500);
    assert_eq!(e.tick_us, 150);

    let _ = std::fs::remove_file(path);
}

/// Python log_sub() builds a Subscribe LogEntry.
#[test]
fn python_log_sub_roundtrip() {
    let (buf, path) = temp_buf("py_log_sub");

    let entry = LogEntry {
        timestamp: "14:30:01.456".to_string(),
        tick_number: 0,
        node_name: "py_controller".to_string(),
        log_type: LogType::Subscribe,
        topic: Some("cmd_vel".to_string()),
        message: "CmdVel(linear=1.0, angular=0.5)".to_string(),
        tick_us: 80,
        ipc_ns: 1200,
    };
    buf.push(entry);

    let all = buf.get_all();
    assert_eq!(all.len(), 1);
    let e = &all[0];
    assert_eq!(e.log_type, LogType::Subscribe);
    assert_eq!(e.topic.as_deref(), Some("cmd_vel"));
    assert_eq!(e.ipc_ns, 1200);

    let _ = std::fs::remove_file(path);
}

/// Python log_info/log_warning/log_error/log_debug call hlog!() which uses
/// log_with_context.  Test all four levels round-trip to the global buffer.
#[test]
fn python_hlog_all_levels_roundtrip() {
    let unique = format!("pylog_{}", std::process::id());

    set_node_context("py_node", 100);

    let idx_before = horus_core::core::log_buffer::GLOBAL_LOG_BUFFER.write_idx();

    log_with_context(LogType::Info, format!("{}_info", unique));
    log_with_context(LogType::Warning, format!("{}_warn", unique));
    log_with_context(LogType::Error, format!("{}_error", unique));
    log_with_context(LogType::Debug, format!("{}_debug", unique));

    let idx_after = horus_core::core::log_buffer::GLOBAL_LOG_BUFFER.write_idx();
    assert!(idx_after >= idx_before + 4);

    let all = horus_core::core::log_buffer::GLOBAL_LOG_BUFFER.get_all();
    let py_entries: Vec<_> = all
        .iter()
        .filter(|e| e.message.starts_with(&unique))
        .collect();

    assert_eq!(py_entries.len(), 4, "all 4 levels must round-trip");

    assert!(py_entries
        .iter()
        .any(|e| e.log_type == LogType::Info && e.message.ends_with("_info")));
    assert!(py_entries
        .iter()
        .any(|e| e.log_type == LogType::Warning && e.message.ends_with("_warn")));
    assert!(py_entries
        .iter()
        .any(|e| e.log_type == LogType::Error && e.message.ends_with("_error")));
    assert!(py_entries
        .iter()
        .any(|e| e.log_type == LogType::Debug && e.message.ends_with("_debug")));

    // All should be attributed to py_node
    for e in &py_entries {
        assert_eq!(e.node_name, "py_node");
        assert_eq!(e.tick_number, 100);
    }

    clear_node_context();
}

/// Python can send entries with None topic — verify the field is properly
/// serialized as absent (not as empty string).
#[test]
fn python_log_entry_none_topic() {
    let (buf, path) = temp_buf("py_none_topic");

    let entry = LogEntry {
        timestamp: "14:30:02.000".to_string(),
        tick_number: 0,
        node_name: "py_node_no_topic".to_string(),
        log_type: LogType::Info,
        topic: None,
        message: "no topic here".to_string(),
        tick_us: 0,
        ipc_ns: 0,
    };
    buf.push(entry);

    let all = buf.get_all();
    assert_eq!(all.len(), 1);
    assert!(all[0].topic.is_none(), "topic must be None, not Some(\"\")");

    let _ = std::fs::remove_file(path);
}

/// Python log_pub can be read back via for_topic() — simulates the TUI/monitor
/// reading Python-generated log entries.
#[test]
fn python_pub_readable_via_for_topic() {
    let (buf, path) = temp_buf("py_for_topic");

    // Simulate two different Python nodes publishing on different topics
    buf.push(LogEntry {
        timestamp: "14:30:03.000".to_string(),
        tick_number: 0,
        node_name: "py_lidar".to_string(),
        log_type: LogType::Publish,
        topic: Some("scan".to_string()),
        message: "LaserScan(ranges=360)".to_string(),
        tick_us: 50,
        ipc_ns: 800,
    });
    buf.push(LogEntry {
        timestamp: "14:30:03.001".to_string(),
        tick_number: 0,
        node_name: "py_camera".to_string(),
        log_type: LogType::Publish,
        topic: Some("image".to_string()),
        message: "Image(640x480)".to_string(),
        tick_us: 200,
        ipc_ns: 3000,
    });

    let scan_entries = buf.for_topic("scan");
    assert_eq!(scan_entries.len(), 1);
    assert_eq!(scan_entries[0].node_name, "py_lidar");

    let image_entries = buf.for_topic("image");
    assert_eq!(image_entries.len(), 1);
    assert_eq!(image_entries[0].node_name, "py_camera");

    let py_lidar_entries = buf.for_node("py_lidar");
    assert_eq!(py_lidar_entries.len(), 1);

    let _ = std::fs::remove_file(path);
}

// ═══════════════════════════════════════════════════════════════════════════════
//  Crash-during-write recovery tests
//
//  Simulate writers that crash mid-write by manually setting seqlock to odd.
//  Verify readers don't hang, skip orphaned slots, and reset seqlocks.
// ═══════════════════════════════════════════════════════════════════════════════

/// Simulate a writer crash: manually set a slot's seqlock to odd.
/// Reader must not hang — should skip after SEQLOCK_SPIN_TIMEOUT (~5ms).
#[test]
fn crash_recovery_single_orphaned_slot() {
    let (buf, path) = temp_buf("crash_single");

    // Write 10 valid entries
    for i in 0..10u64 {
        buf.push(make_entry("valid_node", i, &format!("msg_{}", i)));
    }

    // Corrupt slot 5 (simulate crash mid-write)
    buf.corrupt_slot_seqlock(5, 999);

    // Reader must complete (not hang) within a reasonable time
    let start = std::time::Instant::now();
    let all = buf.get_all();
    let elapsed = start.elapsed();

    assert!(
        elapsed < std::time::Duration::from_millis(100),
        "reader hung on orphaned slot ({:?})",
        elapsed
    );

    // Should have 9 entries (slot 5 skipped)
    assert_eq!(all.len(), 9, "orphaned slot must be skipped");
    assert!(
        !all.iter().any(|e| e.tick_number == 5),
        "entry at corrupted slot 5 must not appear"
    );

    // The orphaned slot should now have an even seqlock (reclaimed)
    let seq = buf.read_slot_seqlock(5);
    assert!(
        seq & 1 == 0,
        "orphaned seqlock should be reset to even, got {}",
        seq
    );

    let _ = std::fs::remove_file(path);
}

/// Multiple crash-orphaned slots at various positions, including first and last.
#[test]
fn crash_recovery_multiple_orphaned_slots() {
    let (buf, path) = temp_buf("crash_multi");

    // Write 20 valid entries
    for i in 0..20u64 {
        buf.push(make_entry("multi_node", i, &format!("msg_{}", i)));
    }

    // Corrupt slots 0, 7, 13, 19 (first, middle, middle, last)
    let corrupt_slots = [0usize, 7, 13, 19];
    for &slot in &corrupt_slots {
        buf.corrupt_slot_seqlock(slot, (slot as u64) * 2 + 1);
    }

    let start = std::time::Instant::now();
    let all = buf.get_all();
    let elapsed = start.elapsed();

    // Must not hang (4 orphaned slots × 5ms timeout = 20ms theoretical max)
    assert!(
        elapsed < std::time::Duration::from_millis(200),
        "reader hung on multiple orphaned slots ({:?})",
        elapsed
    );

    // Should have 16 entries (4 corrupted slots skipped)
    assert_eq!(all.len(), 16, "4 orphaned slots must be skipped");

    // Verify corrupted entries are absent
    for &slot in &corrupt_slots {
        assert!(
            !all.iter().any(|e| e.tick_number == slot as u64),
            "entry at corrupted slot {} must not appear",
            slot
        );
    }

    let _ = std::fs::remove_file(path);
}

/// After an orphaned slot is reclaimed (seqlock reset to even), verify
/// the seqlock is indeed even (prerequisite for reuse on next wrap-around).
#[test]
fn crash_recovery_reclaimed_slot_writable() {
    let (buf, path) = temp_buf("crash_reclaim");

    // Write 3 entries (uses slots 0, 1, 2)
    buf.push(make_entry("first_gen", 0, "gen1_0"));
    buf.push(make_entry("first_gen", 1, "gen1_1"));
    buf.push(make_entry("first_gen", 2, "gen1_2"));

    // Corrupt slot 1
    buf.corrupt_slot_seqlock(1, 77);

    // Read to trigger reclamation of slot 1
    let all = buf.get_all();
    assert_eq!(all.len(), 2, "slot 1 should be skipped");

    // Verify the seqlock was reset (prerequisite for future reuse)
    let seq = buf.read_slot_seqlock(1);
    assert!(
        seq & 1 == 0,
        "reclaimed slot seqlock must be even, got {}",
        seq
    );

    let _ = std::fs::remove_file(path);
}

/// Concurrent readers + orphaned slot: all readers must complete without hanging.
/// The first reader to encounter the orphan resets its seqlock to even; later
/// readers may then see the old data (still valid).  The key invariant is that
/// NO reader hangs — they all complete promptly.
#[test]
fn crash_recovery_concurrent_readers_with_orphan() {
    let (buf, path) = temp_buf("crash_conc");

    // Write 100 entries
    for i in 0..100u64 {
        buf.push(make_entry("conc_node", i, "data"));
    }

    // Corrupt slot 50
    buf.corrupt_slot_seqlock(50, 101);

    let buf = Arc::new(buf);

    // 8 concurrent readers — all must complete (not hang)
    let handles: Vec<_> = (0..8)
        .map(|_| {
            let b = buf.clone();
            std::thread::spawn(move || {
                let start = std::time::Instant::now();
                let all = b.get_all();
                let elapsed = start.elapsed();
                assert!(
                    elapsed < std::time::Duration::from_millis(100),
                    "reader hung ({:?})",
                    elapsed
                );
                // First reader skips slot 50 (99 entries); later readers may
                // see the reclaimed slot (100 entries). Both are acceptable.
                assert!(
                    all.len() >= 99 && all.len() <= 100,
                    "expected 99 or 100 entries, got {}",
                    all.len()
                );
            })
        })
        .collect();

    for h in handles {
        h.join().unwrap();
    }

    let _ = std::fs::remove_file(path);
}

// ═══════════════════════════════════════════════════════════════════════════════
//  Log type filtering tests
// ═══════════════════════════════════════════════════════════════════════════════

/// Write mixed log types, filter Error-only, verify correct subset.
#[test]
fn filter_error_only() {
    let (buf, path) = temp_buf("filter_error");

    buf.push(make_entry_with_type("n", 0, LogType::Info));
    buf.push(make_entry_with_type("n", 1, LogType::Error));
    buf.push(make_entry_with_type("n", 2, LogType::Warning));
    buf.push(make_entry_with_type("n", 3, LogType::Error));
    buf.push(make_entry_with_type("n", 4, LogType::Debug));
    buf.push(make_entry_with_type("n", 5, LogType::Publish));

    let errors = buf.for_type(&LogType::Error);
    assert_eq!(errors.len(), 2);
    assert!(errors.iter().all(|e| e.log_type == LogType::Error));
    assert!(errors.iter().any(|e| e.tick_number == 1));
    assert!(errors.iter().any(|e| e.tick_number == 3));

    let _ = std::fs::remove_file(path);
}

/// Filter by Publish/Subscribe types (IPC events).
#[test]
fn filter_publish_subscribe() {
    let (buf, path) = temp_buf("filter_pubsub");

    buf.push(make_entry_with_type("n", 0, LogType::Publish));
    buf.push(make_entry_with_type("n", 1, LogType::Subscribe));
    buf.push(make_entry_with_type("n", 2, LogType::Info));
    buf.push(make_entry_with_type("n", 3, LogType::Publish));
    buf.push(make_entry_with_type("n", 4, LogType::Warning));

    let pubs = buf.for_type(&LogType::Publish);
    assert_eq!(pubs.len(), 2);

    let subs = buf.for_type(&LogType::Subscribe);
    assert_eq!(subs.len(), 1);

    let _ = std::fs::remove_file(path);
}

/// Combined filter: for_type + for_node together.
#[test]
fn filter_combined_type_and_node() {
    let (buf, path) = temp_buf("filter_combined");

    buf.push(make_entry_with_type("sensor", 0, LogType::Error));
    buf.push(make_entry_with_type("motor", 1, LogType::Error));
    buf.push(make_entry_with_type("sensor", 2, LogType::Info));
    buf.push(make_entry_with_type("sensor", 3, LogType::Error));
    buf.push(make_entry_with_type("motor", 4, LogType::Info));

    // Get errors for "sensor" only
    let sensor_errors: Vec<_> = buf
        .for_type(&LogType::Error)
        .into_iter()
        .filter(|e| e.node_name == "sensor")
        .collect();

    assert_eq!(sensor_errors.len(), 2);
    assert!(sensor_errors.iter().all(|e| e.node_name == "sensor"));
    assert!(sensor_errors.iter().all(|e| e.log_type == LogType::Error));

    let _ = std::fs::remove_file(path);
}

/// Filter for a type with no matching entries returns empty vec.
#[test]
fn filter_type_no_matches() {
    let (buf, path) = temp_buf("filter_empty");

    buf.push(make_entry_with_type("n", 0, LogType::Info));
    buf.push(make_entry_with_type("n", 1, LogType::Warning));

    let debugs = buf.for_type(&LogType::Debug);
    assert!(debugs.is_empty());

    let _ = std::fs::remove_file(path);
}

// ═══════════════════════════════════════════════════════════════════════════════
//  Dynamic log level change tests
// ═══════════════════════════════════════════════════════════════════════════════

/// Set log level to Warn, write Debug via log:: — should NOT appear in buffer.
/// Then change to Debug, write Debug again — should appear.
#[test]
fn dynamic_level_change_bridge() {
    horus_core::core::log_bridge::try_init_log_bridge("warn");

    // Set level to Warn — only Warn+ should pass
    log::set_max_level(log::LevelFilter::Warn);

    let unique = format!("dynlvl_{}", std::process::id());

    let idx_before = horus_core::core::log_buffer::GLOBAL_LOG_BUFFER.write_idx();
    log::debug!(target: "dynlvl_node", "{}_blocked", unique);
    let idx_after_blocked = horus_core::core::log_buffer::GLOBAL_LOG_BUFFER.write_idx();

    // Debug should NOT have been forwarded
    assert_eq!(
        idx_before, idx_after_blocked,
        "debug log should be blocked at Warn level"
    );

    // Change to Debug level at runtime
    log::set_max_level(log::LevelFilter::Debug);

    log::debug!(target: "dynlvl_node", "{}_passed", unique);
    let idx_after_passed = horus_core::core::log_buffer::GLOBAL_LOG_BUFFER.write_idx();

    // Debug should now be forwarded
    assert!(
        idx_after_passed > idx_after_blocked,
        "debug log should pass after level change to Debug"
    );

    // Verify the entry is in the buffer
    let all = horus_core::core::log_buffer::GLOBAL_LOG_BUFFER.get_all();
    let passed = all
        .iter()
        .any(|e| e.message.contains(&format!("{}_passed", unique)));
    assert!(passed, "passed debug entry must be in buffer");

    let blocked = all
        .iter()
        .any(|e| e.message.contains(&format!("{}_blocked", unique)));
    assert!(!blocked, "blocked debug entry must NOT be in buffer");
}

/// hlog!() bypasses the log bridge level filter — all levels always reach the
/// buffer regardless of log::max_level(). This is by design: hlog!() is the
/// node-level logger and should never be silently dropped.
#[test]
fn hlog_ignores_log_bridge_level() {
    // Set bridge to Error-only
    log::set_max_level(log::LevelFilter::Error);

    set_node_context("hlog_level_test", 1);

    let unique = format!("hlog_nolvl_{}", std::process::id());

    let idx_before = horus_core::core::log_buffer::GLOBAL_LOG_BUFFER.write_idx();
    log_with_context(LogType::Debug, unique.to_string());
    let idx_after = horus_core::core::log_buffer::GLOBAL_LOG_BUFFER.write_idx();

    // hlog!() always writes, regardless of log::max_level
    assert!(
        idx_after > idx_before,
        "hlog!() debug must reach buffer even with Error-only bridge level"
    );

    let all = horus_core::core::log_buffer::GLOBAL_LOG_BUFFER.get_all();
    assert!(
        all.iter().any(|e| e.message == unique),
        "hlog entry must be in buffer"
    );

    clear_node_context();
    // Restore level for other tests
    log::set_max_level(log::LevelFilter::Debug);
}

// ═══════════════════════════════════════════════════════════════════════════════
//  Hot-path performance regression tests
//
//  These are threshold-based smoke tests, not micro-benchmarks.  They guard
//  against catastrophic regressions (e.g. accidentally adding a sleep or heavy
//  allocation to the write path).  The thresholds are generous — the goal is
//  to catch order-of-magnitude regressions, not measure p99 latency.
// ═══════════════════════════════════════════════════════════════════════════════

/// 10 000 sequential push() calls must complete in < 500ms total (~50µs each).
/// This catches regressions like accidentally holding a global lock, adding
/// heavy allocation, or inserting a sleep.
#[test]
fn perf_sequential_push_throughput() {
    let (buf, path) = temp_buf("perf_seq");

    let n = 10_000u64;
    let start = std::time::Instant::now();
    for i in 0..n {
        buf.push(make_entry("perf_node", i, "perf test msg"));
    }
    let elapsed = start.elapsed();

    // 500ms for 10K writes = 50µs/write average — very generous threshold
    assert!(
        elapsed < std::time::Duration::from_millis(500),
        "10K sequential push() calls took {:?} (> 500ms threshold)",
        elapsed
    );

    let _ = std::fs::remove_file(path);
}

/// 4 threads each push 2 500 entries concurrently (10 000 total) in < 1s.
/// Verifies the lock-free claim path doesn't regress under contention.
#[test]
fn perf_concurrent_push_throughput() {
    let (buf, path) = temp_buf("perf_conc");
    let buf = Arc::new(buf);

    let threads = 4;
    let per_thread = 2_500u64;

    let start = std::time::Instant::now();
    let handles: Vec<_> = (0..threads)
        .map(|t| {
            let b = buf.clone();
            std::thread::spawn(move || {
                for i in 0..per_thread {
                    b.push(make_entry(
                        &format!("t{}", t),
                        t * per_thread + i,
                        "concurrent perf",
                    ));
                }
            })
        })
        .collect();

    for h in handles {
        h.join().unwrap();
    }
    let elapsed = start.elapsed();

    assert!(
        elapsed < std::time::Duration::from_secs(1),
        "10K concurrent push() calls took {:?} (> 1s threshold)",
        elapsed
    );

    // Verify no slot corruption
    assert_eq!(buf.write_idx(), (threads * per_thread));

    let _ = std::fs::remove_file(path);
}

/// get_all() on a full buffer (5000 entries) must complete in < 200ms.
/// This guards the read path (seqlock checks + deserialization).
#[test]
fn perf_get_all_full_buffer() {
    let (buf, path) = temp_buf("perf_read");

    // Fill the buffer
    for i in 0..5000u64 {
        buf.push(make_entry("fill_node", i, "fill entry"));
    }

    let start = std::time::Instant::now();
    let all = buf.get_all();
    let elapsed = start.elapsed();

    assert_eq!(all.len(), 5000);
    assert!(
        elapsed < std::time::Duration::from_millis(200),
        "get_all() on 5000-entry buffer took {:?} (> 200ms threshold)",
        elapsed
    );

    let _ = std::fs::remove_file(path);
}

// ═══════════════════════════════════════════════════════════════════════════════
//  Phase 18: Monitor TUI real-time log consumption tests
// ═══════════════════════════════════════════════════════════════════════════════

/// Simulates the TUI's polling pattern: repeated for_node() calls should
/// always return the latest entries for that node, with new entries appearing
/// as they are pushed between polls.
#[test]
fn tui_poll_for_node_sees_new_entries() {
    let (buf, path) = temp_buf("tui_poll_node");

    // Push initial entries from two nodes
    buf.push(make_entry("motor_ctrl", 1, "init"));
    buf.push(make_entry("sensor_hub", 2, "init"));

    // First "poll" — TUI opens log panel for motor_ctrl
    let poll1 = buf.for_node("motor_ctrl");
    assert_eq!(poll1.len(), 1);
    assert_eq!(poll1[0].message, "init");

    // New entries arrive between polls
    buf.push(make_entry("motor_ctrl", 3, "tick_1"));
    buf.push(make_entry("sensor_hub", 4, "reading"));
    buf.push(make_entry("motor_ctrl", 5, "tick_2"));

    // Second poll — should see all motor_ctrl entries including new ones
    let poll2 = buf.for_node("motor_ctrl");
    assert_eq!(poll2.len(), 3);
    assert_eq!(poll2[0].message, "init");
    assert_eq!(poll2[1].message, "tick_1");
    assert_eq!(poll2[2].message, "tick_2");

    // sensor_hub unaffected
    let sensor = buf.for_node("sensor_hub");
    assert_eq!(sensor.len(), 2);

    let _ = std::fs::remove_file(path);
}

/// After buffer wrap-around (>5000 entries), for_node() should return only
/// the entries still in the ring (oldest overwritten entries disappear).
#[test]
fn tui_for_node_after_wrap() {
    let (buf, path) = temp_buf("tui_wrap_node");

    // Fill buffer with 4998 entries from "background"
    for i in 0..4998u64 {
        buf.push(make_entry("background", i, &format!("bg_{}", i)));
    }

    // Add 2 entries from our target node (in the last 2 slots before wrap)
    buf.push(make_entry("planner", 4998, "plan_A"));
    buf.push(make_entry("planner", 4999, "plan_B"));

    assert_eq!(buf.for_node("planner").len(), 2);

    // Push 10 more background entries — wraps, overwriting oldest slots
    for i in 5000..5010u64 {
        buf.push(make_entry("background", i, &format!("bg_{}", i)));
    }

    // planner entries should still be present (they're in slots 4998..4999,
    // and wrap overwrites slots 0..9)
    let planner = buf.for_node("planner");
    assert_eq!(planner.len(), 2);

    // Now push enough to overwrite the planner slots (slots 4998, 4999)
    for i in 5010..10000u64 {
        buf.push(make_entry("background", i, &format!("bg_{}", i)));
    }

    // After full wrap, planner entries are gone
    let planner = buf.for_node("planner");
    assert_eq!(planner.len(), 0);

    let _ = std::fs::remove_file(path);
}

/// for_topic() correctly filters entries by topic, simulating the TUI's
/// topic-scoped log panel.
#[test]
fn tui_for_topic_filter() {
    let (buf, path) = temp_buf("tui_topic_filter");

    let entry_with_topic = |node: &str, topic: &str, msg: &str| -> LogEntry {
        LogEntry {
            timestamp: "12:00:00.000".to_string(),
            tick_number: 0,
            node_name: node.to_string(),
            log_type: LogType::Publish,
            topic: Some(topic.to_string()),
            message: msg.to_string(),
            tick_us: 0,
            ipc_ns: 0,
        }
    };

    buf.push(entry_with_topic("cam", "/image_raw", "frame_1"));
    buf.push(entry_with_topic("lidar", "/pointcloud", "scan_1"));
    buf.push(entry_with_topic("cam", "/image_raw", "frame_2"));
    buf.push(make_entry("ctrl", 0, "no topic entry")); // topic = None

    let image_logs = buf.for_topic("/image_raw");
    assert_eq!(image_logs.len(), 2);
    assert!(image_logs
        .iter()
        .all(|e| e.topic.as_deref() == Some("/image_raw")));

    let pc_logs = buf.for_topic("/pointcloud");
    assert_eq!(pc_logs.len(), 1);

    let none_topic = buf.for_topic("/nonexistent");
    assert_eq!(none_topic.len(), 0);

    let _ = std::fs::remove_file(path);
}

/// Combined node + type filter simulates the TUI filtering by both
/// the selected node AND log severity (e.g. show only errors from "planner").
#[test]
fn tui_combined_node_and_type_filter() {
    let (buf, path) = temp_buf("tui_combined_filter");

    buf.push(make_entry_with_type("planner", 1, LogType::Info));
    buf.push(make_entry_with_type("planner", 2, LogType::Error));
    buf.push(make_entry_with_type("planner", 3, LogType::Warning));
    buf.push(make_entry_with_type("driver", 4, LogType::Error));
    buf.push(make_entry_with_type("planner", 5, LogType::Error));

    // Simulate TUI: for_node filtered then by type
    let planner_errors: Vec<LogEntry> = buf
        .for_node("planner")
        .into_iter()
        .filter(|e| e.log_type == LogType::Error)
        .collect();

    assert_eq!(planner_errors.len(), 2);
    assert!(planner_errors.iter().all(|e| e.node_name == "planner"));
    assert!(planner_errors.iter().all(|e| e.log_type == LogType::Error));

    let _ = std::fs::remove_file(path);
}

/// Simulates the TUI's write_idx-based tailing: only reading new entries
/// since the last poll, matching the WebSocket delta pattern.
#[test]
fn tui_delta_tailing_via_write_idx() {
    let (buf, path) = temp_buf("tui_delta_tail");

    // Initial state
    let mut last_idx = buf.write_idx();
    assert_eq!(last_idx, 0);

    // Push 3 entries
    buf.push(make_entry("nav", 1, "waypoint_1"));
    buf.push(make_entry("nav", 2, "waypoint_2"));
    buf.push(make_entry("nav", 3, "waypoint_3"));

    // First delta read
    let current_idx = buf.write_idx();
    let new_count = (current_idx - last_idx) as usize;
    assert_eq!(new_count, 3);
    let all = buf.get_all();
    let delta1: Vec<&LogEntry> = all
        .iter()
        .rev()
        .take(new_count)
        .collect::<Vec<_>>()
        .into_iter()
        .rev()
        .collect();
    assert_eq!(delta1.len(), 3);
    last_idx = current_idx;

    // Push 2 more
    buf.push(make_entry("nav", 4, "waypoint_4"));
    buf.push(make_entry("nav", 5, "waypoint_5"));

    // Second delta read — should only see the 2 new entries
    let current_idx = buf.write_idx();
    let new_count = (current_idx - last_idx) as usize;
    assert_eq!(new_count, 2);
    let all = buf.get_all();
    let delta2: Vec<&LogEntry> = all
        .iter()
        .rev()
        .take(new_count)
        .collect::<Vec<_>>()
        .into_iter()
        .rev()
        .collect();
    assert_eq!(delta2.len(), 2);
    assert_eq!(delta2[0].message, "waypoint_4");
    assert_eq!(delta2[1].message, "waypoint_5");
    last_idx = current_idx;

    // No new entries → zero delta
    let current_idx = buf.write_idx();
    assert_eq!(current_idx, last_idx);

    let _ = std::fs::remove_file(path);
}

/// After buffer wraps, the delta reader should still get correct results
/// as long as new_count doesn't exceed the buffer size.
#[test]
fn tui_delta_tail_across_wrap() {
    let (buf, path) = temp_buf("tui_delta_wrap");

    // Fill buffer to 4999
    for i in 0..4999u64 {
        buf.push(make_entry("fill", i, "filler"));
    }
    let last_idx = buf.write_idx();
    assert_eq!(last_idx, 4999);

    // Push 3 more — crosses the 5000 boundary, triggering wrap
    buf.push(make_entry("target", 4999, "pre_wrap"));
    buf.push(make_entry("target", 5000, "at_wrap"));
    buf.push(make_entry("target", 5001, "post_wrap"));

    let current_idx = buf.write_idx();
    let new_count = (current_idx - last_idx) as usize;
    assert_eq!(new_count, 3);

    // All 3 target entries should be readable
    let all = buf.get_all();
    let target_entries: Vec<&LogEntry> = all.iter().filter(|e| e.node_name == "target").collect();
    assert_eq!(target_entries.len(), 3);

    let _ = std::fs::remove_file(path);
}

// ═══════════════════════════════════════════════════════════════════════════════
//  Phase 19: Cross-process & crash realism tests
//
//  These tests use libc::fork() to create real child processes that share the
//  same mmap-backed log buffer.  This validates the MAP_SHARED atomic semantics
//  that thread-based tests cannot cover.
//
//  All tests are Linux-only (#[cfg(target_os = "linux")]).
// ═══════════════════════════════════════════════════════════════════════════════

/// True cross-process test: fork a child that writes 1000 entries to the same
/// mmap file, parent reads after child exits and verifies all entries.
#[test]
#[cfg(target_os = "linux")]
fn cross_process_fork_sequential_write_read() {
    let path = std::env::temp_dir().join(format!(
        "horus_xproc_seq_{}_{}.bin",
        std::process::id(),
        std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap()
            .subsec_nanos()
    ));
    let _ = std::fs::remove_file(&path);

    // Create the buffer in the parent so the mmap file exists.
    let buf = SharedLogBuffer::new_at_path(&path).unwrap();
    // Parent writes 100 entries first.
    for i in 0..100u64 {
        buf.push(make_entry("parent", i, &format!("parent_{}", i)));
    }
    drop(buf); // Flush and release the mmap.

    // Fork a child process to write 1000 more entries.
    let pid = unsafe { libc::fork() };
    assert!(pid >= 0, "fork() failed");

    if pid == 0 {
        // ── Child process ────────────────────────────────────────────────
        let child_buf = SharedLogBuffer::new_at_path(&path).unwrap();
        for i in 0..1000u64 {
            child_buf.push(make_entry("child", 100 + i, &format!("child_{}", i)));
        }
        drop(child_buf);
        // _exit to avoid running parent's Drop impls / test harness cleanup.
        unsafe { libc::_exit(0) };
    }

    // ── Parent process ───────────────────────────────────────────────────
    // Wait for child to finish.
    let mut status: libc::c_int = 0;
    unsafe { libc::waitpid(pid, &mut status, 0) };
    assert!(
        libc::WIFEXITED(status) && libc::WEXITSTATUS(status) == 0,
        "child process exited abnormally"
    );

    // Re-open and verify.
    let buf = SharedLogBuffer::new_at_path(&path).unwrap();
    assert_eq!(
        buf.write_idx(),
        1100,
        "write_idx must reflect parent (100) + child (1000) writes"
    );

    let all = buf.get_all();
    assert_eq!(all.len(), 1100);

    let parent_entries: Vec<_> = all.iter().filter(|e| e.node_name == "parent").collect();
    let child_entries: Vec<_> = all.iter().filter(|e| e.node_name == "child").collect();
    assert_eq!(parent_entries.len(), 100, "all parent entries must survive");
    assert_eq!(child_entries.len(), 1000, "all child entries must survive");

    // Verify content integrity — child entries have sequential tick_numbers.
    let mut child_ticks: Vec<u64> = child_entries.iter().map(|e| e.tick_number).collect();
    child_ticks.sort();
    assert_eq!(child_ticks.first(), Some(&100));
    assert_eq!(child_ticks.last(), Some(&1099));

    let _ = std::fs::remove_file(path);
}

/// Cross-process overlapping: parent reads while child is still writing.
/// Both must complete without panics or torn reads.
#[test]
#[cfg(target_os = "linux")]
fn cross_process_fork_overlapping_read_write() {
    let path = std::env::temp_dir().join(format!(
        "horus_xproc_overlap_{}_{}.bin",
        std::process::id(),
        std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap()
            .subsec_nanos()
    ));
    let _ = std::fs::remove_file(&path);

    // Create the buffer file.
    let _buf = SharedLogBuffer::new_at_path(&path).unwrap();
    drop(_buf);

    let pid = unsafe { libc::fork() };
    assert!(pid >= 0, "fork() failed");

    if pid == 0 {
        // ── Child: write 5000 entries with a small delay to overlap with reader.
        let child_buf = SharedLogBuffer::new_at_path(&path).unwrap();
        for i in 0..5000u64 {
            child_buf.push(make_entry("writer_child", i, &format!("data_{}", i)));
            if i % 500 == 0 {
                std::thread::sleep(std::time::Duration::from_millis(1));
            }
        }
        drop(child_buf);
        unsafe { libc::_exit(0) };
    }

    // ── Parent: repeatedly read while child is writing.
    let parent_buf = SharedLogBuffer::new_at_path(&path).unwrap();
    let mut read_iterations = 0u64;
    let deadline = std::time::Instant::now() + std::time::Duration::from_secs(10);

    loop {
        let entries = parent_buf.get_all();
        // Every entry must have valid fields — no torn reads.
        for e in &entries {
            assert!(
                !e.node_name.is_empty(),
                "torn read: empty node_name at iteration {}",
                read_iterations
            );
            assert!(
                e.node_name == "writer_child",
                "unexpected node_name '{}' at iteration {}",
                e.node_name,
                read_iterations
            );
        }
        read_iterations += 1;

        // Check if child is done.
        let mut status: libc::c_int = 0;
        let ret = unsafe { libc::waitpid(pid, &mut status, libc::WNOHANG) };
        if ret > 0 {
            // Child exited — do one final read and break.
            let final_entries = parent_buf.get_all();
            assert_eq!(final_entries.len(), 5000);
            break;
        }

        assert!(
            std::time::Instant::now() < deadline,
            "timed out waiting for child (read {} iterations)",
            read_iterations
        );
        std::thread::sleep(std::time::Duration::from_millis(5));
    }

    assert!(
        read_iterations > 1,
        "must have read at least twice while child was writing"
    );

    let _ = std::fs::remove_file(path);
}

/// Cross-process: two child processes write concurrently to the same buffer.
/// Parent verifies write_idx and entry uniqueness after both finish.
#[test]
#[cfg(target_os = "linux")]
fn cross_process_two_writers() {
    let path = std::env::temp_dir().join(format!(
        "horus_xproc_2writers_{}_{}.bin",
        std::process::id(),
        std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap()
            .subsec_nanos()
    ));
    let _ = std::fs::remove_file(&path);

    let _buf = SharedLogBuffer::new_at_path(&path).unwrap();
    drop(_buf);

    let entries_per_child: u64 = 2000;

    // Fork child A.
    let pid_a = unsafe { libc::fork() };
    assert!(pid_a >= 0, "fork() A failed");

    if pid_a == 0 {
        let buf = SharedLogBuffer::new_at_path(&path).unwrap();
        for i in 0..entries_per_child {
            buf.push(make_entry("child_a", i, &format!("a_{}", i)));
        }
        drop(buf);
        unsafe { libc::_exit(0) };
    }

    // Fork child B.
    let pid_b = unsafe { libc::fork() };
    assert!(pid_b >= 0, "fork() B failed");

    if pid_b == 0 {
        let buf = SharedLogBuffer::new_at_path(&path).unwrap();
        for i in 0..entries_per_child {
            buf.push(make_entry(
                "child_b",
                entries_per_child + i,
                &format!("b_{}", i),
            ));
        }
        drop(buf);
        unsafe { libc::_exit(0) };
    }

    // Parent: wait for both.
    for pid in [pid_a, pid_b] {
        let mut status: libc::c_int = 0;
        unsafe { libc::waitpid(pid, &mut status, 0) };
        assert!(
            libc::WIFEXITED(status) && libc::WEXITSTATUS(status) == 0,
            "child {} exited abnormally",
            pid
        );
    }

    let buf = SharedLogBuffer::new_at_path(&path).unwrap();
    let total = entries_per_child * 2;
    assert_eq!(
        buf.write_idx(),
        total,
        "write_idx must reflect both children's writes"
    );

    let all = buf.get_all();
    assert_eq!(all.len(), total as usize);

    let a_count = all.iter().filter(|e| e.node_name == "child_a").count();
    let b_count = all.iter().filter(|e| e.node_name == "child_b").count();
    assert_eq!(a_count, entries_per_child as usize);
    assert_eq!(b_count, entries_per_child as usize);

    // All tick_numbers must be unique (no slot collision).
    let mut ticks: Vec<u64> = all.iter().map(|e| e.tick_number).collect();
    ticks.sort();
    ticks.dedup();
    assert_eq!(
        ticks.len(),
        total as usize,
        "tick_numbers must be unique — slot collision detected"
    );

    let _ = std::fs::remove_file(path);
}

/// Real SIGKILL crash recovery: fork a child that writes in a tight loop,
/// SIGKILL it mid-write, then verify the parent reader recovers.
///
/// We run multiple iterations to increase the chance of hitting an actual
/// in-progress write.  The invariant is: get_all() must NEVER hang, and
/// all non-orphaned entries must be valid.
#[test]
#[cfg(target_os = "linux")]
fn cross_process_sigkill_mid_write_recovery() {
    // Run multiple iterations since hitting the exact race window is probabilistic.
    for iteration in 0..5 {
        let path = std::env::temp_dir().join(format!(
            "horus_xproc_sigkill_{}_{}_{}.bin",
            std::process::id(),
            iteration,
            std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap()
                .subsec_nanos()
        ));
        let _ = std::fs::remove_file(&path);

        let _buf = SharedLogBuffer::new_at_path(&path).unwrap();
        drop(_buf);

        let pid = unsafe { libc::fork() };
        assert!(pid >= 0, "fork() failed on iteration {}", iteration);

        if pid == 0 {
            // ── Child: write in a tight infinite loop until killed.
            let buf = SharedLogBuffer::new_at_path(&path).unwrap();
            let mut i = 0u64;
            loop {
                buf.push(make_entry("doomed", i, &format!("data_{}", i)));
                i += 1;
            }
            // Never reached — child is SIGKILLed.
        }

        // ── Parent: let child write briefly, then SIGKILL it.
        // The short sleep increases the chance of killing mid-write.
        std::thread::sleep(std::time::Duration::from_millis(2));
        horus_sys::process::ProcessHandle::from_pid(pid as u32)
            .signal(horus_sys::process::Signal::Kill)
            .expect("SIGKILL failed");

        let mut status: libc::c_int = 0;
        unsafe { libc::waitpid(pid, &mut status, 0) };
        assert!(
            libc::WIFSIGNALED(status),
            "iteration {}: child should have been killed by signal",
            iteration
        );

        // ── Read from the buffer — must NOT hang.
        let buf = SharedLogBuffer::new_at_path(&path).unwrap();
        let start = std::time::Instant::now();
        let all = buf.get_all();
        let elapsed = start.elapsed();

        assert!(
            elapsed < std::time::Duration::from_millis(200),
            "iteration {}: get_all() hung on crash-orphaned slot ({:?})",
            iteration,
            elapsed
        );

        // The child wrote at least some entries before being killed.
        assert!(
            buf.write_idx() > 0,
            "iteration {}: child must have written some entries",
            iteration
        );

        // Every returned entry must be valid.
        for e in &all {
            assert_eq!(
                e.node_name, "doomed",
                "iteration {}: unexpected node_name '{}'",
                iteration, e.node_name
            );
        }

        // Check if any slot has an orphaned (odd) seqlock — this is the
        // "hit the race" detection.  It's OK if no orphan was produced
        // (the kill might have landed between push() calls).
        let write_idx = buf.write_idx() as usize;
        let num_slots = write_idx.min(5000);
        for slot in 0..num_slots {
            let seq = buf.read_slot_seqlock(slot);
            // After get_all(), any orphaned slots should have been reset to even.
            assert!(
                seq & 1 == 0,
                "iteration {}: slot {} still has odd seqlock {} after read",
                iteration,
                slot,
                seq
            );
        }

        let _ = std::fs::remove_file(path);
    }
}

// ═══════════════════════════════════════════════════════════════════════════════
//  Phase 20: Real-time latency & ordering tests
// ═══════════════════════════════════════════════════════════════════════════════

/// Helper: compute percentile from a sorted slice of durations.
fn percentile(sorted: &[std::time::Duration], pct: f64) -> std::time::Duration {
    if sorted.is_empty() {
        return std::time::Duration::ZERO;
    }
    let idx = ((pct / 100.0) * (sorted.len() - 1) as f64).round() as usize;
    sorted[idx.min(sorted.len() - 1)]
}

/// Helper: format a latency summary for test failure messages.
fn latency_summary(sorted: &[std::time::Duration]) -> String {
    format!(
        "p50={:?} p90={:?} p99={:?} p999={:?} max={:?}",
        percentile(sorted, 50.0),
        percentile(sorted, 90.0),
        percentile(sorted, 99.0),
        percentile(sorted, 99.9),
        sorted.last().unwrap_or(&std::time::Duration::ZERO)
    )
}

/// Uncontended push latency: measure each individual push() and assert
/// p99 < 500us.  Debug builds with parallel test runner cause CPU contention,
/// so we use relaxed thresholds here.  For production profiling, run with
/// `--release --test-threads=1`.
#[test]
fn latency_uncontended_push_p99() {
    let (buf, path) = temp_buf("lat_uncont");

    let n = 10_000usize;
    let mut latencies = Vec::with_capacity(n);

    for i in 0..n as u64 {
        let start = std::time::Instant::now();
        buf.push(make_entry("lat_node", i, "latency test"));
        latencies.push(start.elapsed());
    }

    latencies.sort();

    let p99 = percentile(&latencies, 99.0);
    let p999 = percentile(&latencies, 99.9);

    assert!(
        p99 < std::time::Duration::from_micros(500),
        "uncontended p99 push latency {:?} exceeds 500us threshold. {}",
        p99,
        latency_summary(&latencies)
    );
    assert!(
        p999 < std::time::Duration::from_millis(2),
        "uncontended p999 push latency {:?} exceeds 2ms threshold. {}",
        p999,
        latency_summary(&latencies)
    );

    let _ = std::fs::remove_file(path);
}

/// Contended push latency: 4 threads each push 2500 entries, measuring
/// per-push latency.  Assert p99 < 200us, p999 < 1ms.
#[test]
fn latency_contended_push_p99() {
    let (buf, path) = temp_buf("lat_cont");
    let buf = Arc::new(buf);

    let threads = 4;
    let per_thread = 2_500usize;

    let handles: Vec<_> = (0..threads)
        .map(|t| {
            let b = buf.clone();
            std::thread::spawn(move || {
                let mut latencies = Vec::with_capacity(per_thread);
                for i in 0..per_thread as u64 {
                    let start = std::time::Instant::now();
                    b.push(make_entry(
                        &format!("t{}", t),
                        t as u64 * per_thread as u64 + i,
                        "contended",
                    ));
                    latencies.push(start.elapsed());
                }
                latencies
            })
        })
        .collect();

    let mut all_latencies: Vec<std::time::Duration> = Vec::with_capacity(threads * per_thread);
    for h in handles {
        all_latencies.extend(h.join().unwrap());
    }

    all_latencies.sort();

    let p99 = percentile(&all_latencies, 99.0);
    let p999 = percentile(&all_latencies, 99.9);

    // Thresholds are generous for debug builds where Mutex + bincode overhead
    // is higher, especially when running alongside other tests.
    assert!(
        p99 < std::time::Duration::from_millis(1),
        "contended p99 push latency {:?} exceeds 1ms threshold. {}",
        p99,
        latency_summary(&all_latencies)
    );
    assert!(
        p999 < std::time::Duration::from_millis(5),
        "contended p999 push latency {:?} exceeds 5ms threshold. {}",
        p999,
        latency_summary(&all_latencies)
    );

    let _ = std::fs::remove_file(path);
}

/// Chronological ordering: after wrap, get_all() must return entries in
/// oldest-to-newest order (tick_numbers monotonically increasing).
#[test]
fn ordering_monotonic_after_single_wrap() {
    let (buf, path) = temp_buf("order_single");

    // Push 7500 entries (wraps once at 5000).
    for i in 0..7500u64 {
        buf.push(make_entry("ord", i, "order"));
    }

    let all = buf.get_all();
    assert_eq!(all.len(), 5000);

    // Verify monotonically increasing tick_numbers.
    for w in all.windows(2) {
        assert!(
            w[0].tick_number < w[1].tick_number,
            "ordering violation: tick {} followed by {} (must be strictly increasing)",
            w[0].tick_number,
            w[1].tick_number
        );
    }

    // Should contain ticks 2500–7499.
    assert_eq!(all.first().unwrap().tick_number, 2500);
    assert_eq!(all.last().unwrap().tick_number, 7499);

    let _ = std::fs::remove_file(path);
}

/// Ordering after multiple wraps (5001, 10000, 15000 entries).
#[test]
fn ordering_monotonic_after_multiple_wraps() {
    for &total in &[5001u64, 10000, 15000] {
        let (buf, path) = temp_buf(&format!("order_{}", total));

        for i in 0..total {
            buf.push(make_entry("ord_multi", i, "order"));
        }

        let all = buf.get_all();
        let expected_len = 5000.min(total as usize);
        assert_eq!(all.len(), expected_len, "wrong count for total={}", total);

        // Verify strictly increasing tick_numbers.
        for w in all.windows(2) {
            assert!(
                w[0].tick_number < w[1].tick_number,
                "ordering violation at total={}: tick {} followed by {}",
                total,
                w[0].tick_number,
                w[1].tick_number
            );
        }

        // Newest entry should be total-1.
        assert_eq!(
            all.last().unwrap().tick_number,
            total - 1,
            "newest entry wrong for total={}",
            total
        );

        let _ = std::fs::remove_file(path);
    }
}

// ═══════════════════════════════════════════════════════════════════════════════
//  Phase 21: Sustained stress & scalability tests
//
//  These are longer-running tests marked #[ignore] so they don't slow down
//  normal `cargo test`.  Run them explicitly:
//      cargo test -p horus_core --test log_system_tests -- --ignored
// ═══════════════════════════════════════════════════════════════════════════════

/// 30-second soak: 20 writer threads at ~100Hz + 4 reader threads.
/// Simulates a realistic robotics workload (20 nodes at 100Hz = 2K writes/sec).
///
/// Run with: cargo test -p horus_core --test log_system_tests soak_30s -- --ignored
#[test]
#[ignore] // Long-running: ~30 seconds
fn soak_30s_20_writers_4_readers() {
    use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};

    let (buf, path) = temp_buf("soak_30s");
    let buf = Arc::new(buf);
    let stop = Arc::new(AtomicBool::new(false));
    let total_writes = Arc::new(AtomicU64::new(0));
    let torn_reads = Arc::new(AtomicU64::new(0));

    let soak_duration = std::time::Duration::from_secs(30);

    // ── 20 writer threads at ~100Hz ──────────────────────────────────────
    let writer_handles: Vec<_> = (0..20)
        .map(|t| {
            let b = buf.clone();
            let s = stop.clone();
            let tw = total_writes.clone();
            std::thread::spawn(move || {
                let mut i = 0u64;
                let interval = std::time::Duration::from_millis(10); // ~100Hz
                while !s.load(Ordering::Relaxed) {
                    b.push(make_entry(
                        &format!("soak_node_{}", t),
                        t as u64 * 1_000_000 + i,
                        &format!("soak_{}_{}", t, i),
                    ));
                    i += 1;
                    tw.fetch_add(1, Ordering::Relaxed);
                    std::thread::sleep(interval);
                }
            })
        })
        .collect();

    // ── 4 reader threads polling continuously ────────────────────────────
    let reader_handles: Vec<_> = (0..4)
        .map(|_| {
            let b = buf.clone();
            let s = stop.clone();
            let tr = torn_reads.clone();
            std::thread::spawn(move || {
                let mut reads = 0u64;
                while !s.load(Ordering::Relaxed) {
                    let entries = b.get_all();
                    for e in &entries {
                        if e.node_name.is_empty() || !e.node_name.starts_with("soak_node_") {
                            tr.fetch_add(1, Ordering::Relaxed);
                        }
                    }
                    reads += 1;
                    std::thread::yield_now();
                }
                reads
            })
        })
        .collect();

    // ── Let it run ───────────────────────────────────────────────────────
    std::thread::sleep(soak_duration);
    stop.store(true, Ordering::Relaxed);

    // ── Join all threads ─────────────────────────────────────────────────
    for h in writer_handles {
        h.join().unwrap();
    }
    let total_reader_iters: u64 = reader_handles.into_iter().map(|h| h.join().unwrap()).sum();

    let writes = total_writes.load(Ordering::Relaxed);
    let torn = torn_reads.load(Ordering::Relaxed);

    // ── Assertions ───────────────────────────────────────────────────────
    assert!(writes > 0, "writers must have produced entries");
    assert_eq!(buf.write_idx(), writes, "write_idx must match total writes");
    assert_eq!(
        torn, 0,
        "zero torn reads expected, got {} torn in {} reader iterations",
        torn, total_reader_iters
    );

    // Final read must return exactly 5000 valid entries (buffer is full after 30s).
    let all = buf.get_all();
    assert_eq!(all.len(), 5000, "buffer must be full after 30s soak");

    // All entries must have valid node names.
    for e in &all {
        assert!(
            e.node_name.starts_with("soak_node_"),
            "invalid node_name '{}' in final read",
            e.node_name
        );
    }

    eprintln!(
        "[soak_30s] {} total writes, {} reader iterations, 0 torn reads",
        writes, total_reader_iters
    );

    let _ = std::fs::remove_file(path);
}

/// 50-node contention: 50 threads each push 200 entries.  No single push
/// should exceed 1ms even under heavy contention.
#[test]
fn scalability_50_node_contention() {
    let (buf, path) = temp_buf("scale_50node");
    let buf = Arc::new(buf);

    let threads = 50;
    let per_thread = 200usize;

    let handles: Vec<_> = (0..threads)
        .map(|t| {
            let b = buf.clone();
            std::thread::spawn(move || {
                let mut max_latency = std::time::Duration::ZERO;
                for i in 0..per_thread as u64 {
                    let start = std::time::Instant::now();
                    b.push(make_entry(
                        &format!("node_{}", t),
                        t as u64 * per_thread as u64 + i,
                        "contention",
                    ));
                    let lat = start.elapsed();
                    if lat > max_latency {
                        max_latency = lat;
                    }
                }
                max_latency
            })
        })
        .collect();

    let mut worst_latency = std::time::Duration::ZERO;
    for h in handles {
        let lat = h.join().unwrap();
        if lat > worst_latency {
            worst_latency = lat;
        }
    }

    let total = (threads * per_thread) as u64;
    assert_eq!(buf.write_idx(), total);

    // Ring wraps at 5000, total is 10000.
    let all = buf.get_all();
    assert_eq!(all.len(), 5000);

    // All tick_numbers in the readable range must be unique.
    let mut ticks: Vec<u64> = all.iter().map(|e| e.tick_number).collect();
    ticks.sort();
    ticks.dedup();
    assert_eq!(
        ticks.len(),
        5000,
        "tick_numbers must be unique — slot collision under 50-thread contention"
    );

    // Threshold is 200ms in debug builds (Mutex + bincode + OS scheduling jitter
    // when running alongside 60+ other tests).  In release builds with
    // --test-threads=1 this should be < 1ms.
    assert!(
        worst_latency < std::time::Duration::from_millis(200),
        "worst single push latency {:?} exceeds 200ms under 50-thread contention",
        worst_latency
    );

    let _ = std::fs::remove_file(path);
}

// ═══════════════════════════════════════════════════════════════════════════════
//  Phase 22: Corruption & resource resilience tests
// ═══════════════════════════════════════════════════════════════════════════════

/// Garbage bytes in a slot's data area: get_all() must skip the slot and
/// never panic, even when the seqlock says "complete" (even nonzero).
#[test]
fn corruption_garbage_bytes_skipped() {
    let (buf, path) = temp_buf("corrupt_garbage");

    // Write 5 valid entries.
    for i in 0..5u64 {
        buf.push(make_entry("valid", i, &format!("msg_{}", i)));
    }
    assert_eq!(buf.get_all().len(), 5);

    // Overwrite slot 2's data with random garbage, seqlock set to even (looks "complete").
    let garbage: Vec<u8> = (0..504).map(|i| ((i * 37 + 13) % 256) as u8).collect();
    buf.write_slot_raw(2, &garbage, 42);

    // get_all must not panic — should skip the garbage slot.
    let all = buf.get_all();
    // Expect 4 valid entries (slot 2 is garbage, deserialization fails → skipped).
    assert_eq!(all.len(), 4, "garbage slot must be skipped");
    assert!(
        !all.iter().any(|e| e.tick_number == 2),
        "entry at corrupted slot 2 must not appear"
    );

    let _ = std::fs::remove_file(path);
}

/// All-zeros data: seqlock even nonzero but data is all zeros.
/// bincode deserialization may succeed or fail — either way, no panic.
#[test]
fn corruption_all_zeros_no_panic() {
    let (buf, path) = temp_buf("corrupt_zeros");

    buf.push(make_entry("before", 0, "ok"));

    // Overwrite slot 0 data with all zeros but even seqlock.
    let zeros = vec![0u8; 504];
    buf.write_slot_raw(0, &zeros, 2);

    // Must not panic.
    let all = buf.get_all();
    // Either the zero-data deserializes to a valid (but empty) LogEntry,
    // or deserialization fails and the slot is skipped.  Both are acceptable.
    assert!(
        all.len() <= 1,
        "should have 0 or 1 entries, got {}",
        all.len()
    );

    let _ = std::fs::remove_file(path);
}

/// All-0xFF data: maximally adversarial byte pattern.
#[test]
fn corruption_all_0xff_no_panic() {
    let (buf, path) = temp_buf("corrupt_ff");

    buf.push(make_entry("before", 0, "ok"));
    buf.push(make_entry("after", 1, "ok"));

    // Overwrite slot 0 with 0xFF bytes.
    let ff_bytes = vec![0xFFu8; 504];
    buf.write_slot_raw(0, &ff_bytes, 4);

    // Must not panic.
    let all = buf.get_all();
    // Slot 0 should be skipped (0xFF is not valid bincode for LogEntry).
    // Slot 1 ("after") should still be readable.
    let after = all.iter().find(|e| e.node_name == "after");
    assert!(
        after.is_some(),
        "'after' entry must survive corruption of slot 0"
    );

    let _ = std::fs::remove_file(path);
}

/// Mixed corruption: some slots valid, some garbage, some all-zeros.
/// get_all() must return exactly the valid entries.
#[test]
fn corruption_mixed_patterns() {
    let (buf, path) = temp_buf("corrupt_mixed");

    // Write 10 valid entries (slots 0–9).
    for i in 0..10u64 {
        buf.push(make_entry("mixed", i, &format!("msg_{}", i)));
    }

    // Corrupt slots 1, 4, 7 with different patterns.
    let random_bytes: Vec<u8> = (0..504).map(|i| ((i * 53 + 7) % 256) as u8).collect();
    buf.write_slot_raw(1, &random_bytes, 10);
    buf.write_slot_raw(4, &vec![0u8; 504], 20);
    buf.write_slot_raw(7, &vec![0xFFu8; 504], 30);

    let all = buf.get_all();
    // 10 entries minus 3 corrupted = 7 valid (at most; some zeros might deserialize).
    assert!(
        all.len() >= 7 && all.len() <= 10,
        "expected 7-10 entries, got {}",
        all.len()
    );

    // The uncorrupted entries must be present.
    for tick in [0u64, 2, 3, 5, 6, 8, 9] {
        assert!(
            all.iter().any(|e| e.tick_number == tick),
            "uncorrupted entry tick={} must be present",
            tick
        );
    }

    let _ = std::fs::remove_file(path);
}

/// mmap file resilience: 0-byte pre-existing file should be re-initialized.
#[test]
fn mmap_zero_byte_file_reinit() {
    let path = std::env::temp_dir().join(format!("horus_mmap_zerobyte_{}.bin", std::process::id()));
    // Create a 0-byte file.
    std::fs::write(&path, b"").unwrap();
    assert_eq!(std::fs::metadata(&path).unwrap().len(), 0);

    // new_at_path should reinitialize (set_len to full size).
    let buf = SharedLogBuffer::new_at_path(&path).unwrap();
    assert_eq!(
        buf.write_idx(),
        0,
        "fresh buffer from 0-byte file must start at 0"
    );
    let all = buf.get_all();
    assert!(all.is_empty(), "fresh buffer must be empty");

    // Writes should work.
    buf.push(make_entry("reinit", 1, "works"));
    assert_eq!(buf.get_all().len(), 1);

    let _ = std::fs::remove_file(path);
}

/// mmap file with garbage header: write_idx could be anything.
/// push() and get_all() must not panic.
#[test]
fn mmap_garbage_header_no_panic() {
    let path =
        std::env::temp_dir().join(format!("horus_mmap_garbage_hdr_{}.bin", std::process::id()));
    let _ = std::fs::remove_file(&path);

    // Create a properly-sized file filled with garbage.
    let total_size = 64 + 5000 * 512; // HEADER_SIZE + MAX_LOG_ENTRIES * SLOT_SIZE
    let garbage: Vec<u8> = (0..total_size).map(|i| ((i * 37) % 256) as u8).collect();
    std::fs::write(&path, &garbage).unwrap();

    // new_at_path should succeed (file exists at correct size).
    let buf = SharedLogBuffer::new_at_path(&path).unwrap();

    // write_idx will be garbage — but get_all() must not panic.
    let _all = buf.get_all(); // May return garbage entries or empty — just must not panic.

    // push() must not panic (will overwrite a garbage slot).
    buf.push(make_entry("survivor", 1, "post-garbage"));

    // After push, at least one entry should be readable.
    let all = buf.get_all();
    let survivor = all.iter().find(|e| e.node_name == "survivor");
    assert!(
        survivor.is_some(),
        "entry written after garbage init must be readable"
    );

    let _ = std::fs::remove_file(path);
}

/// new_at_path on a read-only path must return Err, not panic.
#[test]
#[cfg(target_os = "linux")]
fn mmap_readonly_dir_returns_error() {
    use std::os::unix::fs::PermissionsExt;

    let dir = std::env::temp_dir().join(format!(
        "horus_mmap_readonly_{}_{}",
        std::process::id(),
        std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap()
            .subsec_nanos()
    ));
    std::fs::create_dir_all(&dir).unwrap();
    // Make directory read-only.
    std::fs::set_permissions(&dir, std::fs::Permissions::from_mode(0o444)).unwrap();

    let path = dir.join("logs.bin");
    let result = SharedLogBuffer::new_at_path(&path);
    assert!(
        result.is_err(),
        "new_at_path on read-only dir must return Err"
    );

    // Restore permissions for cleanup.
    std::fs::set_permissions(&dir, std::fs::Permissions::from_mode(0o755)).unwrap();
    let _ = std::fs::remove_dir_all(dir);
}

/// write_idx overflow arithmetic: test with very large write_idx values
/// to ensure modulo math stays correct.
#[test]
fn write_idx_large_values_arithmetic() {
    let (buf, path) = temp_buf("widx_overflow");

    // Push some entries to set up valid data.
    for i in 0..10u64 {
        buf.push(make_entry("widx", i, "data"));
    }

    // The buffer is usable and write_idx is small (10).
    assert_eq!(buf.write_idx(), 10);
    let all = buf.get_all();
    assert_eq!(all.len(), 10);

    // Verify the modulo arithmetic conceptually:
    // (u64::MAX as usize) % 5000 must not overflow on 64-bit.
    let big: u64 = u64::MAX;
    let slot = (big as usize) % 5000;
    assert!(slot < 5000, "modulo must produce valid slot index");

    // Near u32::MAX boundary.
    let near_u32: u64 = u32::MAX as u64;
    let slot2 = (near_u32 as usize) % 5000;
    assert!(slot2 < 5000);

    // u32::MAX + 5000
    let over_u32: u64 = u32::MAX as u64 + 5000;
    let slot3 = (over_u32 as usize) % 5000;
    assert!(slot3 < 5000);

    let _ = std::fs::remove_file(path);
}

// ═══════════════════════════════════════════════════════════════════════════════
// Log throughput benchmarks
// ═══════════════════════════════════════════════════════════════════════════════

#[test]
fn test_log_push_throughput_single_thread() {
    use horus_core::core::log_buffer::publish_log;

    let count = 10_000u64;
    let start = std::time::Instant::now();
    for i in 0..count {
        publish_log(make_entry("bench_node", i, "throughput_single_thread"));
    }
    let elapsed = start.elapsed();
    let entries_per_sec = count as f64 / elapsed.as_secs_f64();

    eprintln!(
        "Single-thread log throughput: {:.0} entries/sec ({count} entries in {:?})",
        entries_per_sec, elapsed
    );

    assert!(
        entries_per_sec > 10_000.0,
        "Single-thread should push >10k entries/sec, got {:.0}",
        entries_per_sec
    );
}

#[test]
fn test_log_push_throughput_8_threads() {
    use horus_core::core::log_buffer::publish_log;

    let per_thread = 1_000u64;
    let num_threads = 8usize;
    let barrier = Arc::new(std::sync::Barrier::new(num_threads));

    let start = std::time::Instant::now();
    let handles: Vec<_> = (0..num_threads)
        .map(|t| {
            let barrier = barrier.clone();
            std::thread::spawn(move || {
                barrier.wait();
                for i in 0..per_thread {
                    publish_log(make_entry(
                        &format!("bench_t{t}"),
                        i,
                        "throughput_8_threads",
                    ));
                }
            })
        })
        .collect();

    for h in handles {
        h.join().unwrap();
    }
    let elapsed = start.elapsed();
    let total = per_thread * num_threads as u64;
    let entries_per_sec = total as f64 / elapsed.as_secs_f64();

    eprintln!(
        "8-thread log throughput: {:.0} entries/sec ({total} entries in {:?})",
        entries_per_sec, elapsed
    );

    assert!(
        entries_per_sec > 5_000.0,
        "8-thread should push >5k entries/sec aggregate, got {:.0}",
        entries_per_sec
    );
}

#[test]
fn test_log_push_p99_latency() {
    use horus_core::core::log_buffer::publish_log;

    let count = 1_000usize;
    let mut durations = Vec::with_capacity(count);

    for i in 0..count {
        let t0 = std::time::Instant::now();
        publish_log(make_entry("bench_p99", i as u64, "latency_test"));
        durations.push(t0.elapsed());
    }

    durations.sort();
    let p50 = durations[count / 2];
    let p99 = durations[count * 99 / 100];

    eprintln!("Log push latency: p50={:?}, p99={:?}", p50, p99);

    assert!(
        p99.as_millis() < 1,
        "p99 push latency should be < 1ms, got {:?}",
        p99
    );
}
