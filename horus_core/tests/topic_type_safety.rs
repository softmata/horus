//! Topic type safety integration tests
//!
//! Verifies that the POD type size check at topic/mod.rs:449 works correctly:
//! opening a topic with a different POD type size produces an error, while
//! reopening with the same type succeeds.

mod common;

use horus_core::communication::Topic;

/// Generate a unique topic name per test using PID + test-specific suffix
fn unique_topic(suffix: &str) -> String {
    format!("ci_type_safety_{}_{}", std::process::id(), suffix)
}

/// Test: Opening a topic with mismatched POD type sizes produces an error.
///
/// Create Topic<u32> then try Topic<u64> on the same name.
/// The second open should fail with "Type size mismatch".
#[test]
fn test_pod_type_size_mismatch_detected() {
    common::cleanup_stale_shm();

    let name = unique_topic("mismatch");

    // First open as u32 (4 bytes) — should succeed
    let _topic_u32 = Topic::<u32>::new(&name)
        .expect("First open as u32 should succeed");

    // Second open as u64 (8 bytes) — should fail with type size mismatch
    let result = Topic::<u64>::new(&name);
    let err_msg = match result {
        Err(e) => e.to_string(),
        Ok(_) => panic!("Opening same topic with different POD type size should fail"),
    };
    assert!(
        err_msg.contains("Type size mismatch"),
        "Error should mention 'Type size mismatch', got: {}",
        err_msg
    );
}

/// Test: Reopening a topic with the same POD type succeeds and works.
///
/// Create Topic<u32>, send a value, then open another Topic<u32> on the
/// same name and verify recv works.
#[test]
fn test_same_pod_type_reopens() {
    common::cleanup_stale_shm();

    let name = unique_topic("same_type");

    let pub_topic = Topic::<u32>::new(&name)
        .expect("First open as u32 should succeed");

    pub_topic.send(42u32);

    // Reopen with same type — should succeed
    let sub_topic = Topic::<u32>::new(&name)
        .expect("Second open as u32 should succeed");

    let msg = sub_topic.recv();
    assert_eq!(msg, Some(42u32), "Should receive the sent message");
}

/// Test: Non-POD types skip the size check.
///
/// Open Topic<String> then Topic<Vec<u8>> on the same name.
/// Since neither is POD (both have heap allocations / Drop),
/// the type size check is skipped and both opens should succeed.
#[test]
fn test_non_pod_skips_size_check() {
    common::cleanup_stale_shm();

    let name = unique_topic("non_pod");

    let _topic_string = Topic::<String>::new(&name)
        .expect("First open as String should succeed");

    // Open with a different non-POD type — should NOT error because is_pod=false
    let result = Topic::<Vec<u8>>::new(&name);
    match result {
        Ok(_) => {} // expected
        Err(e) => panic!("Non-POD type mismatch should not produce an error, got: {}", e),
    }
}
