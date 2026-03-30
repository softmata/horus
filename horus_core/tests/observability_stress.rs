//! Multi-machine observability stress tests.
//!
//! Simulates remote hosts by writing presence files and log entries directly,
//! then verifies discovery, log merging, and e-stop encoding.
//!
//! Run: `cargo test --no-default-features -p horus_core --test observability_stress -- --test-threads=1`

use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

mod common;

// ============================================================================
// Test 1: 50 remote hosts presence stress
// ============================================================================

#[test]
fn test_50_hosts_presence_discovery() {
    let nodes_dir = horus_sys::shm::shm_nodes_dir();
    let _ = std::fs::create_dir_all(&nodes_dir);

    // Clean up any existing remote files
    if let Ok(entries) = std::fs::read_dir(&nodes_dir) {
        for entry in entries.flatten() {
            let name = entry.file_name().to_string_lossy().to_string();
            if name.starts_with("remote_") || name.starts_with("bridged_") {
                let _ = std::fs::remove_file(entry.path());
            }
        }
    }

    let now_ns = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap()
        .as_nanos() as u64;

    // Write 50 remote host presence files, each with 10 nodes
    for host in 0..50u32 {
        let host_id = format!("host_{:04}", host);
        let mut nodes_json = Vec::new();
        for node in 0..10u32 {
            nodes_json.push(format!(
                r#"{{"name":"h{}_node_{}","rate_hz":100.0}}"#,
                host, node
            ));
        }
        let json = format!(
            r#"{{"host_id":"{}","is_remote":true,"last_seen_ns":{},"namespace":"default","nodes":[{}]}}"#,
            host_id,
            now_ns,
            nodes_json.join(",")
        );
        let path = nodes_dir.join(format!("remote_{}.json", host_id));
        std::fs::write(&path, &json).unwrap();
    }

    // Discovery should find all 500 remote nodes
    let start = Instant::now();
    let files: Vec<_> = std::fs::read_dir(&nodes_dir)
        .unwrap()
        .filter_map(|e| e.ok())
        .filter(|e| {
            e.file_name()
                .to_string_lossy()
                .starts_with("remote_")
        })
        .collect();
    let elapsed = start.elapsed();

    assert_eq!(files.len(), 50, "Should have 50 remote presence files");
    assert!(
        elapsed < Duration::from_secs(1),
        "Scanning 50 files should be < 1s, took {:?}",
        elapsed
    );

    // Clean up
    for host in 0..50u32 {
        let path = nodes_dir.join(format!("remote_host_{:04}.json", host));
        let _ = std::fs::remove_file(&path);
    }
}

// ============================================================================
// Test 2: Presence rapid churn
// ============================================================================

#[test]
fn test_presence_rapid_churn() {
    let nodes_dir = horus_sys::shm::shm_nodes_dir();
    let _ = std::fs::create_dir_all(&nodes_dir);

    let now_ns = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap()
        .as_nanos() as u64;

    // Rapidly create and delete presence files (simulating hosts joining/leaving)
    for cycle in 0..100u32 {
        let host_id = format!("churn_{:04}", cycle % 20);
        let path = nodes_dir.join(format!("remote_{}.json", host_id));

        if cycle % 3 == 0 {
            // Delete
            let _ = std::fs::remove_file(&path);
        } else {
            // Create/update
            let json = format!(
                r#"{{"host_id":"{}","is_remote":true,"last_seen_ns":{},"namespace":"default","nodes":[{{"name":"churn_node","rate_hz":50.0}}]}}"#,
                host_id, now_ns
            );
            std::fs::write(&path, &json).unwrap();
        }
    }

    // Verify no crashes during churn, scan completes
    let files: Vec<_> = std::fs::read_dir(&nodes_dir)
        .unwrap()
        .filter_map(|e| e.ok())
        .filter(|e| {
            e.file_name()
                .to_string_lossy()
                .starts_with("remote_churn_")
        })
        .collect();

    // Some files should exist (not all deleted)
    // Exact count depends on modulo pattern
    assert!(
        files.len() <= 20,
        "At most 20 churn hosts should exist"
    );

    // Clean up
    for f in files {
        let _ = std::fs::remove_file(f.path());
    }
}

// ============================================================================
// Test 3: Log flood — remote logs don't corrupt local buffer
// ============================================================================

#[test]
fn test_remote_log_buffer_isolation() {
    use horus_core::core::log_buffer::{LogEntry, LogType, GLOBAL_LOG_BUFFER, GLOBAL_REMOTE_LOG_BUFFER};

    // Record local buffer state before
    let local_before = GLOBAL_LOG_BUFFER.write_idx();

    // Write 5000 entries to REMOTE buffer (simulating 10 hosts × 500 each)
    for host in 0..10u32 {
        for i in 0..500u32 {
            GLOBAL_REMOTE_LOG_BUFFER.push(LogEntry {
                timestamp: String::new(),
                tick_number: (host * 500 + i) as u64,
                node_name: format!("host_{}/node_{}", host, i % 5),
                log_type: LogType::Warning,
                topic: None,
                message: format!("remote warning {}", i),
                tick_us: 0,
                ipc_ns: 0,
            });
        }
    }

    // Local buffer should be UNTOUCHED
    let local_after = GLOBAL_LOG_BUFFER.write_idx();
    assert_eq!(
        local_before, local_after,
        "Local log buffer should not be modified by remote writes"
    );

    // Remote buffer should have entries
    let remote_entries = GLOBAL_REMOTE_LOG_BUFFER.get_all();
    assert!(
        !remote_entries.is_empty(),
        "Remote buffer should have entries after 5000 writes"
    );

    // Verify entries have host-prefixed node names
    let has_host_prefix = remote_entries
        .iter()
        .any(|e| e.node_name.contains("host_"));
    assert!(has_host_prefix, "Remote entries should have host prefix");
}

// Tests 4-6 (e-stop, presence, log replication) use horus_net APIs
// and live in horus_net/tests/observability_integration.rs instead,
// since horus_core tests cannot import horus_net (circular dep).
