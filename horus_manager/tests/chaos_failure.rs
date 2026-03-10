//! Chaos test: corrupt SHM, permission errors, corrupt data.
//!
//! Injects failures into the system and verifies the monitor degrades
//! gracefully without crashing.
//!
//! Run with: cargo test -p horus_manager --test chaos_failure -- --test-threads=1

mod harness;
mod monitor_tests;

use harness::{HorusTestRuntime, TestNodeConfig};
use monitor_tests::builders;
use monitor_tests::helpers::get_request;

use std::sync::atomic::{AtomicU32, Ordering};
use tower::ServiceExt;

static CHAOS_COUNTER: AtomicU32 = AtomicU32::new(0);

fn unique_name(prefix: &str) -> String {
    let id = CHAOS_COUNTER.fetch_add(1, Ordering::Relaxed);
    format!("{}_{:04}", prefix, id)
}

// ═══════════════════════════════════════════════════════════════════════════════
//  Corrupt presence file — garbage JSON
// ═══════════════════════════════════════════════════════════════════════════════

#[tokio::test]
async fn chaos_corrupt_presence_file_no_panic() {
    let nodes_dir = horus_core::memory::shm_base_dir().join("nodes");
    std::fs::create_dir_all(&nodes_dir).ok();

    // Write garbage as a "presence file"
    let name = unique_name("corrupt_node");
    let path = nodes_dir.join(format!("{}.json", name));
    std::fs::write(&path, b"THIS IS NOT JSON {{{ garbage !!!").unwrap();

    // Also write some valid nodes
    let mut rt = HorusTestRuntime::new();
    rt.add_node(TestNodeConfig::bare(&unique_name("good_node")));
    rt.wait_ready(std::time::Duration::from_secs(5));

    // API must still respond
    let app = builders::test_router();
    let resp = app.oneshot(get_request("/api/nodes")).await.unwrap();
    assert_eq!(
        resp.status(),
        axum::http::StatusCode::OK,
        "corrupt presence file must not crash /api/nodes"
    );

    let body = axum::body::to_bytes(resp.into_body(), 10 * 1024 * 1024)
        .await
        .unwrap();
    let json: serde_json::Value = serde_json::from_slice(&body)
        .expect("response must be valid JSON even with corrupt presence file");
    assert!(json["nodes"].is_array());

    // Cleanup
    std::fs::remove_file(&path).ok();
}

// ═══════════════════════════════════════════════════════════════════════════════
//  Empty presence file — zero bytes
// ═══════════════════════════════════════════════════════════════════════════════

#[tokio::test]
async fn chaos_empty_presence_file_no_panic() {
    let nodes_dir = horus_core::memory::shm_base_dir().join("nodes");
    std::fs::create_dir_all(&nodes_dir).ok();

    let name = unique_name("empty_node");
    let path = nodes_dir.join(format!("{}.json", name));
    std::fs::write(&path, b"").unwrap();

    let app = builders::test_router();
    let resp = app.oneshot(get_request("/api/nodes")).await.unwrap();
    assert_eq!(resp.status(), axum::http::StatusCode::OK);

    std::fs::remove_file(&path).ok();
}

// ═══════════════════════════════════════════════════════════════════════════════
//  Truncated JSON presence file
// ═══════════════════════════════════════════════════════════════════════════════

#[tokio::test]
async fn chaos_truncated_json_presence_no_panic() {
    let nodes_dir = horus_core::memory::shm_base_dir().join("nodes");
    std::fs::create_dir_all(&nodes_dir).ok();

    let name = unique_name("truncated_node");
    let path = nodes_dir.join(format!("{}.json", name));
    // Valid JSON start, but truncated
    std::fs::write(&path, r#"{"name": "foo", "pid": 1234, "sched"#).unwrap();

    let app = builders::test_router();
    let resp = app.oneshot(get_request("/api/nodes")).await.unwrap();
    assert_eq!(
        resp.status(),
        axum::http::StatusCode::OK,
        "truncated JSON must not crash /api/nodes"
    );

    std::fs::remove_file(&path).ok();
}

// ═══════════════════════════════════════════════════════════════════════════════
//  Presence file with wrong PID (dead process)
// ═══════════════════════════════════════════════════════════════════════════════

#[tokio::test]
async fn chaos_dead_pid_presence_no_panic() {
    let nodes_dir = horus_core::memory::shm_base_dir().join("nodes");
    std::fs::create_dir_all(&nodes_dir).ok();

    let name = unique_name("dead_pid_node");
    let path = nodes_dir.join(format!("{}.json", name));
    // Use a PID that definitely doesn't exist
    let presence = serde_json::json!({
        "name": name,
        "pid": 999999,
        "publishers": [],
        "subscribers": [],
        "start_time": 0,
        "priority": 0,
        "pid_start_time": 0
    });
    std::fs::write(&path, serde_json::to_string(&presence).unwrap()).unwrap();

    let app = builders::test_router();
    let resp = app.oneshot(get_request("/api/nodes")).await.unwrap();
    assert_eq!(
        resp.status(),
        axum::http::StatusCode::OK,
        "dead PID presence must not crash /api/nodes"
    );

    std::fs::remove_file(&path).ok();
}

// ═══════════════════════════════════════════════════════════════════════════════
//  Permission denied on presence file (Unix only)
// ═══════════════════════════════════════════════════════════════════════════════

#[cfg(unix)]
#[tokio::test]
async fn chaos_permission_denied_presence_no_panic() {
    use std::os::unix::fs::PermissionsExt;

    let nodes_dir = horus_core::memory::shm_base_dir().join("nodes");
    std::fs::create_dir_all(&nodes_dir).ok();

    let name = unique_name("perm_denied_node");
    let path = nodes_dir.join(format!("{}.json", name));
    std::fs::write(&path, r#"{"name":"test","pid":1}"#).unwrap();
    std::fs::set_permissions(&path, std::fs::Permissions::from_mode(0o000)).unwrap();

    let app = builders::test_router();
    let resp = app.oneshot(get_request("/api/nodes")).await.unwrap();
    assert_eq!(
        resp.status(),
        axum::http::StatusCode::OK,
        "permission denied must not crash /api/nodes"
    );

    // Restore permissions for cleanup
    std::fs::set_permissions(&path, std::fs::Permissions::from_mode(0o644)).unwrap();
    std::fs::remove_file(&path).ok();
}

// ═══════════════════════════════════════════════════════════════════════════════
//  Corrupt SHM topic file — garbage bytes
// ═══════════════════════════════════════════════════════════════════════════════

#[tokio::test]
async fn chaos_corrupt_shm_topic_no_panic() {
    let mut rt = HorusTestRuntime::new();
    // Create a valid topic alongside
    rt.add_topic(&unique_name("good_topic"), 4096);

    // Create a corrupt topic file directly
    let topics_dir = horus_core::memory::shm_topics_dir().join("horus_topic");
    std::fs::create_dir_all(&topics_dir).ok();
    let corrupt_name = unique_name("corrupt_topic");
    let corrupt_path = topics_dir.join(&corrupt_name);
    // Write random garbage
    std::fs::write(&corrupt_path, vec![0xDE, 0xAD, 0xBE, 0xEF, 0xFF]).unwrap();

    let app = builders::test_router();
    let resp = app.oneshot(get_request("/api/topics")).await.unwrap();
    assert_eq!(
        resp.status(),
        axum::http::StatusCode::OK,
        "corrupt SHM topic must not crash /api/topics"
    );

    std::fs::remove_file(&corrupt_path).ok();
}

// ═══════════════════════════════════════════════════════════════════════════════
//  Zero-size SHM topic file
// ═══════════════════════════════════════════════════════════════════════════════

#[tokio::test]
async fn chaos_zero_size_topic_no_panic() {
    let topics_dir = horus_core::memory::shm_topics_dir().join("horus_topic");
    std::fs::create_dir_all(&topics_dir).ok();
    let name = unique_name("zero_topic");
    let path = topics_dir.join(&name);
    std::fs::write(&path, b"").unwrap();

    let app = builders::test_router();
    let resp = app.oneshot(get_request("/api/topics")).await.unwrap();
    assert_eq!(
        resp.status(),
        axum::http::StatusCode::OK,
        "zero-size topic must not crash /api/topics"
    );

    std::fs::remove_file(&path).ok();
}

// ═══════════════════════════════════════════════════════════════════════════════
//  Missing nodes directory — no crash
// ═══════════════════════════════════════════════════════════════════════════════

#[tokio::test]
async fn chaos_api_status_without_nodes_dir() {
    // Even if SHM dir doesn't exist, /api/status should work
    let app = builders::test_router();
    let resp = app.oneshot(get_request("/api/status")).await.unwrap();
    assert_eq!(
        resp.status(),
        axum::http::StatusCode::OK,
        "/api/status must work even without presence data"
    );
}

// ═══════════════════════════════════════════════════════════════════════════════
//  Corrupt blackbox WAL — partial read
// ═══════════════════════════════════════════════════════════════════════════════

#[tokio::test]
async fn chaos_corrupt_wal_partial_read() {
    // Create blackbox dir
    let bb_dir = dirs::home_dir().expect("home dir").join(".horus/blackbox");
    std::fs::create_dir_all(&bb_dir).ok();
    let wal_path = bb_dir.join("blackbox.wal");

    // Backup existing WAL if any
    let backup = if wal_path.exists() {
        std::fs::read(&wal_path).ok()
    } else {
        None
    };

    // Write a mix of valid and corrupt lines
    use std::io::Write;
    let mut f = std::fs::OpenOptions::new()
        .create(true)
        .write(true)
        .truncate(true)
        .open(&wal_path)
        .unwrap();
    // Valid line
    writeln!(
        f,
        r#"{{"timestamp_us":1000,"tick":1,"event":{{"Custom":{{"category":"test","message":"valid line"}}}}}}"#
    ).unwrap();
    // Corrupt line
    writeln!(f, "THIS IS GARBAGE {{{{").unwrap();
    // Another valid line
    writeln!(
        f,
        r#"{{"timestamp_us":2000,"tick":2,"event":{{"Custom":{{"category":"test","message":"second valid"}}}}}}"#
    ).unwrap();

    let app = builders::test_router();
    let resp = app.oneshot(get_request("/api/blackbox")).await.unwrap();
    assert_eq!(
        resp.status(),
        axum::http::StatusCode::OK,
        "corrupt WAL lines must not crash /api/blackbox"
    );

    // Restore backup
    if let Some(data) = backup {
        std::fs::write(&wal_path, data).ok();
    } else {
        std::fs::remove_file(&wal_path).ok();
    }
}

// ═══════════════════════════════════════════════════════════════════════════════
//  Multiple simultaneous corrupt files — no cascade failure
// ═══════════════════════════════════════════════════════════════════════════════

#[tokio::test]
async fn chaos_multiple_corrupt_files_no_cascade() {
    let nodes_dir = horus_core::memory::shm_base_dir().join("nodes");
    std::fs::create_dir_all(&nodes_dir).ok();

    // Create 10 corrupt + 5 valid presence files
    let mut corrupt_files = Vec::new();
    for _ in 0..10 {
        let name = unique_name("multi_corrupt");
        let path = nodes_dir.join(format!("{}.json", name));
        std::fs::write(&path, "GARBAGE").unwrap();
        corrupt_files.push(path);
    }

    let mut rt = HorusTestRuntime::new();
    for i in 0..5 {
        rt.add_node(TestNodeConfig::bare(&format!("multi_good_{}", i)));
    }

    let app = builders::test_router();
    let resp = app.oneshot(get_request("/api/nodes")).await.unwrap();
    assert_eq!(
        resp.status(),
        axum::http::StatusCode::OK,
        "multiple corrupt files must not cause cascade failure"
    );

    // Cleanup corrupt files
    for f in &corrupt_files {
        std::fs::remove_file(f).ok();
    }
}

// ═══════════════════════════════════════════════════════════════════════════════
//  Graph endpoint with corrupt topology
// ═══════════════════════════════════════════════════════════════════════════════

#[tokio::test]
async fn chaos_graph_with_corrupt_topology() {
    let nodes_dir = horus_core::memory::shm_base_dir().join("nodes");
    std::fs::create_dir_all(&nodes_dir).ok();

    // Create a presence file referencing non-existent topics
    let name = unique_name("orphan_pub");
    let path = nodes_dir.join(format!("{}.json", name));
    let presence = serde_json::json!({
        "name": name,
        "pid": std::process::id(),
        "publishers": [{"topic_name": "nonexistent_xyz", "type_name": "Void"}],
        "subscribers": [{"topic_name": "also_missing", "type_name": "Void"}],
        "start_time": 0,
        "priority": 0,
        "pid_start_time": 0
    });
    std::fs::write(&path, serde_json::to_string(&presence).unwrap()).unwrap();

    let app = builders::test_router();
    let resp = app.oneshot(get_request("/api/graph")).await.unwrap();
    assert_eq!(
        resp.status(),
        axum::http::StatusCode::OK,
        "graph with orphaned topic refs must not panic"
    );

    std::fs::remove_file(&path).ok();
}

// ═══════════════════════════════════════════════════════════════════════════════
//  Very large presence file — boundary check
// ═══════════════════════════════════════════════════════════════════════════════

#[tokio::test]
async fn chaos_very_large_presence_file_no_panic() {
    let nodes_dir = horus_core::memory::shm_base_dir().join("nodes");
    std::fs::create_dir_all(&nodes_dir).ok();

    let name = unique_name("large_presence");
    let path = nodes_dir.join(format!("{}.json", name));

    // Create a valid JSON with 1000 publisher topics
    let mut publishers = Vec::new();
    for i in 0..1000 {
        publishers.push(serde_json::json!({
            "topic_name": format!("topic_{}", i),
            "type_name": "BigData"
        }));
    }
    let presence = serde_json::json!({
        "name": name,
        "pid": std::process::id(),
        "publishers": publishers,
        "subscribers": [],
        "start_time": 0,
        "priority": 0,
        "pid_start_time": 0
    });
    std::fs::write(&path, serde_json::to_string(&presence).unwrap()).unwrap();

    let app = builders::test_router();
    let resp = app.oneshot(get_request("/api/nodes")).await.unwrap();
    assert_eq!(
        resp.status(),
        axum::http::StatusCode::OK,
        "very large presence file must not panic"
    );

    std::fs::remove_file(&path).ok();
}

// ═══════════════════════════════════════════════════════════════════════════════
//  Recordings missing dir — empty list, not crash
// ═══════════════════════════════════════════════════════════════════════════════

#[tokio::test]
async fn chaos_missing_recordings_dir_returns_empty() {
    let app = builders::test_router();
    let resp = app.oneshot(get_request("/api/recordings")).await.unwrap();
    assert_eq!(
        resp.status(),
        axum::http::StatusCode::OK,
        "missing recordings dir must not crash"
    );

    let body = axum::body::to_bytes(resp.into_body(), 1024 * 1024)
        .await
        .unwrap();
    let json: serde_json::Value = serde_json::from_slice(&body).unwrap();
    assert!(json["recordings"].is_array());
}
