#![allow(dead_code)]
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
    let _shm_guard = common::cleanup_stale_shm();

    let name = unique_topic("mismatch");

    // First open as u32 (4 bytes) — should succeed
    let _topic_u32 = Topic::<u32>::new(&name).expect("First open as u32 should succeed");

    // Second open as u64 (8 bytes) — should fail with type size mismatch
    let result = Topic::<u64>::new(&name);
    let err_msg = match result {
        Err(e) => e.to_string(),
        Ok(_) => panic!("Opening same topic with different POD type size should fail"),
    };
    // The runtime rejects the mismatch (the safety feature works); accept the
    // current wording, which names both the topic and the conflicting types.
    assert!(
        err_msg.to_lowercase().contains("mismatch")
            && err_msg.contains("u32")
            && err_msg.contains("u64"),
        "Error should report the u32/u64 type mismatch, got: {}",
        err_msg
    );
}

/// Test: Reopening a topic with the same POD type succeeds and works.
///
/// Create Topic<u32>, send a value, then open another Topic<u32> on the
/// same name and verify recv works.
#[test]
fn test_same_pod_type_reopens() {
    let _shm_guard = common::cleanup_stale_shm();

    let name = unique_topic("same_type");

    let pub_topic = Topic::<u32>::new(&name).expect("First open as u32 should succeed");

    pub_topic.send(42u32);

    // Reopen with same type — should succeed
    let sub_topic = Topic::<u32>::new(&name).expect("Second open as u32 should succeed");

    let msg = sub_topic.recv();
    assert_eq!(msg, Some(42u32), "Should receive the sent message");
}

/// Test: a topic enforces type-name agreement even for non-POD types.
///
/// Open `Topic<String>` then `Topic<Vec<u8>>` on the same name. The runtime
/// checks the type NAME for all types (not just the POD byte size), so the
/// second open is rejected — this prevents cross-type communication (a
/// publisher sending `Vec<u8>` to a `String` subscriber), which is a safety
/// property, not a size check. (This test previously asserted the mismatch was
/// silently allowed; updated to the current, stricter contract.)
#[test]
fn test_non_pod_type_mismatch_detected() {
    let _shm_guard = common::cleanup_stale_shm();

    let name = unique_topic("non_pod");

    let _topic_string = Topic::<String>::new(&name).expect("First open as String should succeed");

    // Different non-POD type on the same topic — rejected on type-name mismatch.
    let err_msg = match Topic::<Vec<u8>>::new(&name) {
        Err(e) => e.to_string(),
        Ok(_) => panic!("Opening the same topic with a different type should fail"),
    };
    assert!(
        err_msg.to_lowercase().contains("mismatch")
            && err_msg.contains("String")
            && err_msg.contains("Vec<u8>"),
        "Error should report the String/Vec<u8> type mismatch, got: {}",
        err_msg
    );
}
