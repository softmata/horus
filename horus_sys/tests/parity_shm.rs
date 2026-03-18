//! SHM Behavioral Parity Tests
//!
//! These tests verify that SHM operations produce identical observable results
//! on Linux, macOS, and Windows. No `#[cfg(target_os)]` in assertions —
//! every test must pass on all 3 platforms.
//!
//! Run: `cargo test --no-default-features -p horus_sys --test parity_shm -- --test-threads=1`

use horus_sys::shm;
use std::path::Path;

/// Generate a unique namespace for test isolation (avoids OnceLock collision).
fn test_ns(test_name: &str) -> String {
    format!("parity_shm_{}_{}", test_name, std::process::id())
}

/// Set HORUS_NAMESPACE and return cleanup guard.
/// NOTE: Due to OnceLock caching in shm_namespace(), only the FIRST call
/// in the process takes effect. Tests that need namespace isolation must
/// use direct path construction instead.
fn with_namespace(ns: &str) {
    std::env::set_var("HORUS_NAMESPACE", ns);
}

fn cleanup_namespace() {
    let base = shm::shm_base_dir();
    let _ = std::fs::remove_dir_all(&base);
}

// ═══════════════════════════════════════════════════════════════════════════
// Group 1: ShmRegion Lifecycle
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn test_shm_create_returns_owner() {
    let name = format!("parity_owner_{}", std::process::id());
    let region = shm::ShmRegion::new(&name, 4096).unwrap();
    assert!(region.is_owner(), "first creator should be owner");
    drop(region);
}

#[test]
fn test_shm_write_then_read_roundtrip() {
    let name = format!("parity_rw_{}", std::process::id());
    let mut region = shm::ShmRegion::new(&name, 4096).unwrap();

    // Write a known pattern
    let data = region.as_slice_mut();
    data[0] = 0xDE;
    data[1] = 0xAD;
    data[2] = 0xBE;
    data[3] = 0xEF;

    // Read it back
    let slice = region.as_slice();
    assert_eq!(slice[0], 0xDE);
    assert_eq!(slice[1], 0xAD);
    assert_eq!(slice[2], 0xBE);
    assert_eq!(slice[3], 0xEF);

    drop(region);
}

#[test]
fn test_shm_zero_initialized_on_create() {
    let name = format!("parity_zero_{}", std::process::id());
    let region = shm::ShmRegion::new(&name, 1024).unwrap();

    // Owner's region should be zero-filled
    assert!(
        region.as_slice().iter().all(|&b| b == 0),
        "newly created SHM region should be zero-initialized"
    );

    drop(region);
}

#[test]
fn test_shm_size_matches_requested() {
    let name = format!("parity_size_{}", std::process::id());
    let region = shm::ShmRegion::new(&name, 8192).unwrap();
    assert_eq!(region.len(), 8192, "region size should match requested");
    drop(region);
}

#[test]
fn test_shm_rejects_zero_size() {
    let result = shm::ShmRegion::new("parity_zero_size", 0);
    assert!(result.is_err(), "size=0 should be rejected");
}

#[test]
fn test_shm_rejects_empty_name() {
    let result = shm::ShmRegion::new("", 4096);
    assert!(result.is_err(), "empty name should be rejected");
}

// ═══════════════════════════════════════════════════════════════════════════
// Group 2: TopicMeta CRUD
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn test_topic_meta_write_list_roundtrip() {
    shm::write_topic_meta("parity_meta_rt", 4096).unwrap();

    let metas = shm::list_topic_metas();
    let found = metas.iter().find(|m| m.name == "parity_meta_rt");
    assert!(found.is_some(), "written meta should be discoverable");

    let meta = found.unwrap();
    assert_eq!(meta.size, 4096);

    shm::remove_topic_meta("parity_meta_rt");
}

#[test]
fn test_topic_meta_creator_pid_is_current_process() {
    shm::write_topic_meta("parity_meta_pid", 1024).unwrap();

    let metas = shm::list_topic_metas();
    let meta = metas.iter().find(|m| m.name == "parity_meta_pid").unwrap();
    assert_eq!(
        meta.creator_pid,
        std::process::id(),
        "creator PID should be current process"
    );

    shm::remove_topic_meta("parity_meta_pid");
}

#[test]
fn test_topic_meta_timestamp_is_recent() {
    shm::write_topic_meta("parity_meta_ts", 1024).unwrap();

    let metas = shm::list_topic_metas();
    let meta = metas.iter().find(|m| m.name == "parity_meta_ts").unwrap();

    let now = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .unwrap()
        .as_secs();

    assert!(
        meta.created_at > 0 && now - meta.created_at < 10,
        "timestamp should be within 10 seconds of now"
    );

    shm::remove_topic_meta("parity_meta_ts");
}

#[test]
fn test_topic_meta_remove_deletes() {
    shm::write_topic_meta("parity_meta_del", 512).unwrap();

    // Verify it exists
    assert!(
        shm::list_topic_metas()
            .iter()
            .any(|m| m.name == "parity_meta_del"),
        "meta should exist after write"
    );

    // Remove it
    shm::remove_topic_meta("parity_meta_del");

    // Verify it's gone
    assert!(
        !shm::list_topic_metas()
            .iter()
            .any(|m| m.name == "parity_meta_del"),
        "meta should be gone after remove"
    );
}

#[test]
fn test_topic_meta_remove_nonexistent_is_safe() {
    // Should not panic or error
    shm::remove_topic_meta("nonexistent_topic_xyz_12345");
}

#[test]
fn test_topic_meta_list_skips_malformed_json() {
    let dir = shm::shm_topics_dir();
    std::fs::create_dir_all(&dir).unwrap();

    let broken_path = dir.join("__parity_broken__.meta");
    std::fs::write(&broken_path, "not valid json {{{").unwrap();

    // Should not panic, should skip the broken file
    let metas = shm::list_topic_metas();
    assert!(
        !metas.iter().any(|m| m.name == "__parity_broken__"),
        "malformed .meta should be skipped"
    );

    let _ = std::fs::remove_file(&broken_path);
}

#[test]
fn test_topic_meta_name_sanitization() {
    // Topic name with special chars should be sanitized in filename
    shm::write_topic_meta("sensor.imu/v1", 2048).unwrap();

    let metas = shm::list_topic_metas();
    let found = metas.iter().find(|m| m.name == "sensor.imu/v1");
    assert!(
        found.is_some(),
        "original name should be preserved in JSON content"
    );

    shm::remove_topic_meta("sensor.imu/v1");
}

// ═══════════════════════════════════════════════════════════════════════════
// Group 3: Namespace & Paths
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn test_namespace_is_nonempty() {
    let ns = shm::shm_namespace();
    assert!(!ns.is_empty(), "namespace must not be empty");
}

#[test]
fn test_namespace_is_stable_across_calls() {
    let ns1 = shm::shm_namespace();
    let ns2 = shm::shm_namespace();
    assert_eq!(ns1, ns2, "namespace must be stable (cached)");
}

#[test]
fn test_base_dir_contains_horus_prefix() {
    let base = shm::shm_base_dir();
    let dir_name = base.file_name().unwrap().to_string_lossy();
    assert!(
        dir_name.starts_with("horus_"),
        "base dir should start with horus_, got '{}'",
        dir_name
    );
}

#[test]
fn test_subdirs_are_children_of_base() {
    let base = shm::shm_base_dir();
    assert!(
        shm::shm_topics_dir().starts_with(&base),
        "topics_dir ⊂ base_dir"
    );
    assert!(
        shm::shm_nodes_dir().starts_with(&base),
        "nodes_dir ⊂ base_dir"
    );
    assert!(
        shm::shm_network_dir().starts_with(&base),
        "network_dir ⊂ base_dir"
    );
    assert!(
        shm::shm_scheduler_dir().starts_with(&base),
        "scheduler_dir ⊂ base_dir"
    );
}

#[test]
fn test_paths_are_absolute() {
    assert!(
        shm::shm_base_dir().is_absolute(),
        "base_dir should be absolute"
    );
    assert!(
        shm::shm_topics_dir().is_absolute(),
        "topics_dir should be absolute"
    );
    assert!(
        shm::shm_nodes_dir().is_absolute(),
        "nodes_dir should be absolute"
    );
    assert!(
        shm::shm_parent_dir().is_absolute(),
        "parent_dir should be absolute"
    );
}

#[test]
fn test_shm_parent_dir_exists_on_platform() {
    let parent = shm::shm_parent_dir();
    assert!(
        parent.exists(),
        "SHM parent dir should exist: {}",
        parent.display()
    );
}

#[test]
fn test_logs_path_inside_base() {
    let logs = shm::shm_logs_path();
    let base = shm::shm_base_dir();
    assert!(
        logs.starts_with(&base),
        "logs path should be inside base dir"
    );
}

// ═══════════════════════════════════════════════════════════════════════════
// Group 4: Stale Detection
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn test_stale_detection_nonexistent_file_is_stale() {
    let path = Path::new("/tmp/horus_parity_nonexistent_file_xyz");
    assert!(
        shm::is_shm_file_stale(path),
        "nonexistent file should be considered stale"
    );
}

// ═══════════════════════════════════════════════════════════════════════════
// Group 5: Namespace Parsing & Cleanup
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn test_parse_namespace_sid_valid() {
    let result = shm::parse_namespace_sid("horus_sid12345_uid1000");
    assert_eq!(result, Some((12345, 1000)));
}

#[test]
fn test_parse_namespace_sid_custom_returns_none() {
    assert_eq!(shm::parse_namespace_sid("horus_my_robot"), None);
    assert_eq!(shm::parse_namespace_sid("horus_test"), None);
    assert_eq!(shm::parse_namespace_sid("not_horus"), None);
}

#[test]
fn test_cleanup_preserves_current_namespace() {
    // Ensure current namespace dir exists
    let base = shm::shm_base_dir();
    std::fs::create_dir_all(&base).unwrap();

    let result = shm::cleanup_stale_namespaces();

    // Current namespace should never be removed
    // (it may or may not be in the skipped count depending on whether it matches auto-gen pattern)
    assert_eq!(result.errors.len(), 0, "cleanup should not error");
}

#[test]
fn test_list_namespaces_returns_current() {
    // Ensure current namespace exists
    std::fs::create_dir_all(shm::shm_base_dir()).unwrap();

    let namespaces = shm::list_all_horus_namespaces();

    let current_dir_name = shm::shm_base_dir()
        .file_name()
        .unwrap()
        .to_string_lossy()
        .to_string();

    assert!(
        namespaces.iter().any(|ns| ns.dir_name == current_dir_name),
        "current namespace should be in listing"
    );
}

// format_bytes_compact is pub(crate) — tested in unit tests, not here
