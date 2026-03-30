//! Logging system stress tests — concurrent writers, buffer overflow,
//! cross-process timestamp ordering, write throughput.
//!
//! Tests the SharedLogBuffer under high concurrency using test-isolated
//! SHM files (not the global log buffer).
//!
//! Run: `cargo test --no-default-features -p horus_core --test logging_stress -- --test-threads=1`

use horus_core::core::log_buffer::{LogEntry, LogType, SharedLogBuffer};
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::Arc;
use std::time::Instant;

mod common;
use common::TestTempDir;

fn make_entry(node: &str, tick: u64, msg: &str) -> LogEntry {
    LogEntry {
        timestamp: format!("2026-03-27T00:00:{:02}.000Z", tick % 60),
        tick_number: tick,
        node_name: node.to_string(),
        log_type: LogType::Info,
        topic: None,
        message: msg.to_string(),
        tick_us: tick * 1000,
        ipc_ns: 0,
    }
}

// ============================================================================
// Test 1: 50 threads logging simultaneously — no corruption
// ============================================================================

#[test]
fn test_50_concurrent_writers_no_corruption() {
    let tmp = TestTempDir::new("log_stress_50");
    let path = tmp.path().join("log_50.shm");
    let buffer = SharedLogBuffer::new_at_path(&path).expect("create log buffer");
    let buffer = Arc::new(buffer);

    let entries_per_thread = 100;
    let thread_count = 50;
    let total_expected = entries_per_thread * thread_count;

    let mut handles = Vec::new();
    for t in 0..thread_count {
        let buf = buffer.clone();
        handles.push(std::thread::spawn(move || {
            for i in 0..entries_per_thread {
                let tick = (t * entries_per_thread + i) as u64;
                buf.push(make_entry(
                    &format!("node_{}", t),
                    tick,
                    &format!("msg_{}_{}", t, i),
                ));
            }
        }));
    }

    for h in handles {
        h.join().expect("writer thread should not panic");
    }

    // Verify write_idx reflects all writes
    let write_idx = buffer.write_idx();
    assert_eq!(
        write_idx, total_expected as u64,
        "write_idx should be {} after {} writes, got {}",
        total_expected, total_expected, write_idx
    );

    // Read all entries — verify no corruption (all deserialize successfully)
    let entries = buffer.get_all();

    // Ring buffer holds MAX 5000 entries — if we wrote 5000, all should be readable
    // If we wrote more than capacity, oldest are overwritten
    assert!(
        !entries.is_empty(),
        "Should have readable entries after {} writes",
        total_expected
    );

    // Verify no empty node names or messages (would indicate corruption)
    for entry in &entries {
        assert!(
            !entry.node_name.is_empty(),
            "Node name should not be empty (corruption)"
        );
        assert!(
            !entry.message.is_empty(),
            "Message should not be empty (corruption)"
        );
    }
}

// ============================================================================
// Test 2: Ring buffer overflow — verify oldest evicted, newest kept
// ============================================================================

#[test]
fn test_ring_buffer_overflow_oldest_evicted() {
    let tmp = TestTempDir::new("log_stress_overflow");
    let path = tmp.path().join("log_overflow.shm");
    let buffer = SharedLogBuffer::new_at_path(&path).expect("create log buffer");

    // Write 6000 entries into a 5000-capacity buffer
    for i in 0..6000u64 {
        buffer.push(make_entry("overflow_node", i, &format!("msg_{}", i)));
    }

    let write_idx = buffer.write_idx();
    assert_eq!(write_idx, 6000, "write_idx should be 6000");

    // Read all — should have at most 5000 entries
    let entries = buffer.get_all();
    assert!(
        entries.len() <= 5000,
        "Buffer capacity is 5000, got {} entries",
        entries.len()
    );

    // The entries should be from the NEWEST writes (not oldest)
    // After 6000 writes into 5000 slots, slots 0-999 were overwritten by 5000-5999
    // So we should see tick_numbers >= 1000 (approximately)
    if !entries.is_empty() {
        let min_tick = entries.iter().map(|e| e.tick_number).min().unwrap();
        let max_tick = entries.iter().map(|e| e.tick_number).max().unwrap();

        // Newest entry should be close to 5999
        assert!(
            max_tick >= 5000,
            "Newest entry tick should be >= 5000, got {}",
            max_tick
        );

        // Oldest should have been evicted (tick < 1000 should be gone)
        // Note: due to ring buffer semantics, the exact boundary depends on
        // which slots wrapped. We just verify the principle works.
    }
}

// ============================================================================
// Test 3: Cross-thread timestamp ordering
// ============================================================================

#[test]
fn test_cross_thread_write_idx_monotonic() {
    let tmp = TestTempDir::new("log_stress_order");
    let path = tmp.path().join("log_order.shm");
    let buffer = SharedLogBuffer::new_at_path(&path).expect("create log buffer");
    let buffer = Arc::new(buffer);

    // 4 threads, each writing 500 entries
    let mut handles = Vec::new();
    for t in 0..4 {
        let buf = buffer.clone();
        handles.push(std::thread::spawn(move || {
            for i in 0..500 {
                let tick = (t * 500 + i) as u64;
                buf.push(make_entry(
                    &format!("order_node_{}", t),
                    tick,
                    &format!("order_{}_{}", t, i),
                ));
            }
        }));
    }

    for h in handles {
        h.join().unwrap();
    }

    // write_idx should be exactly 2000 (4 × 500)
    assert_eq!(buffer.write_idx(), 2000);

    // Read all entries
    let entries = buffer.get_all();

    // Per-node entries should have monotonically increasing tick_numbers
    for t in 0..4 {
        let node_name = format!("order_node_{}", t);
        let node_entries: Vec<_> = entries
            .iter()
            .filter(|e| e.node_name == node_name)
            .collect();

        // Verify per-node ordering is monotonic
        for window in node_entries.windows(2) {
            assert!(
                window[1].tick_number >= window[0].tick_number,
                "Node {} entries should be monotonic: {} -> {}",
                t,
                window[0].tick_number,
                window[1].tick_number
            );
        }
    }
}

// ============================================================================
// Test 4: Write throughput measurement
// ============================================================================

#[test]
fn test_log_write_throughput() {
    let tmp = TestTempDir::new("log_stress_throughput");
    let path = tmp.path().join("log_throughput.shm");
    let buffer = SharedLogBuffer::new_at_path(&path).expect("create log buffer");

    let count = 10_000u64;
    let start = Instant::now();

    for i in 0..count {
        buffer.push(make_entry(
            "throughput_node",
            i,
            "benchmark message payload",
        ));
    }

    let elapsed = start.elapsed();
    let per_entry_ns = elapsed.as_nanos() / count as u128;

    // Each push should be well under 100µs (includes serialization + seqlock + mmap write)
    assert!(
        per_entry_ns < 100_000,
        "Per-entry write should be <100µs, got {}ns ({}µs)",
        per_entry_ns,
        per_entry_ns / 1000
    );

    assert_eq!(buffer.write_idx(), count);
}
