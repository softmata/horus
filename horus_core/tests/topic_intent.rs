//! Level 7 Intent Tests — Topics
//!
//! These tests verify **behavioral intent**, not implementation details.
//! Each test documents WHY a behavior matters and what user-visible guarantee
//! it protects.
//!
//! Covers:
//! - No data loss under normal load
//! - read_latest() returns the most recent value
//! - Multiple subscribers all receive the same data
//! - Type safety enforcement

use horus_core::communication::Topic;

mod common;
use common::cleanup_stale_shm;

// ============================================================================
// Test 1: No data loss under normal load
// ============================================================================

/// INTENT: "Under normal load (< ring buffer capacity), no messages are lost."
///
/// Robotics guarantee: when a sensor publishes 100 readings and the consumer
/// is keeping up, ALL 100 readings must arrive. Dropped sensor data leads to
/// incorrect state estimation and dangerous control decisions.
#[test]
fn test_topic_intent_no_data_loss_under_normal_load() {
    cleanup_stale_shm();

    let name = common::unique("intent.no_loss");
    let topic = Topic::<u64>::new(&name).expect("create topic");

    let total: u64 = 100;
    // Default ring buffer capacity is 512, so 100 messages is well within limits.
    // Send all 100 messages in batches to stay within ring buffer capacity.
    let batch_size: u64 = 50;
    let mut next_send: u64 = 0;
    let mut next_recv: u64 = 0;
    let mut received = Vec::new();

    while next_recv < total {
        // Send a batch
        let send_end = (next_send + batch_size).min(total);
        while next_send < send_end {
            topic.send(next_send);
            next_send += 1;
        }

        // Drain everything available
        while next_recv < next_send {
            let msg = topic.recv().unwrap_or_else(|| {
                panic!(
                    "recv returned None at index {} (sent up to {})",
                    next_recv, next_send
                )
            });
            received.push(msg);
            next_recv += 1;
        }
    }

    // Verify count
    assert_eq!(
        received.len(),
        total as usize,
        "Expected {} messages, received {}",
        total,
        received.len()
    );

    // Verify all values are correct and in order
    for (i, val) in received.iter().enumerate() {
        assert_eq!(
            *val, i as u64,
            "Message {} has wrong value: expected {}, got {}",
            i, i, val
        );
    }

    // Verify no extra messages remain
    assert_eq!(
        topic.recv(),
        None,
        "Topic should be empty after draining all messages"
    );
}

// ============================================================================
// Test 2: read_latest() always returns the most recently sent value
// ============================================================================

/// INTENT: "read_latest() always returns the most recently sent value."
///
/// Robotics guarantee: when a controller reads the latest transform or
/// setpoint, it must get the MOST RECENT value, not a stale one. Getting a
/// stale transform means the robot acts on outdated position data.
///
/// Note: read_latest() requires T: Copy (safety constraint for lock-free reads).
/// We test with u64 which is Copy.
#[test]
fn test_topic_intent_latest_value_always_available() {
    cleanup_stale_shm();

    let name = common::unique("intent.latest");
    let topic = Topic::<u64>::new(&name).expect("create topic");

    // Send values 1 through 5
    for i in 1..=5u64 {
        topic.send(i);
    }

    // read_latest() should return the most recently sent value
    let latest = topic.read_latest();
    assert_eq!(
        latest,
        Some(5),
        "read_latest() should return 5 (the most recently sent value), got {:?}",
        latest
    );

    // Send one more value
    topic.send(42);

    let latest = topic.read_latest();
    assert_eq!(
        latest,
        Some(42),
        "read_latest() should return 42 after sending a new value, got {:?}",
        latest
    );

    // read_latest() should NOT consume the value — calling it again returns
    // the same result
    let latest_again = topic.read_latest();
    assert_eq!(
        latest_again,
        Some(42),
        "read_latest() should be idempotent (not consume), got {:?}",
        latest_again
    );
}

// ============================================================================
// Test 3: Multiple subscribers all receive the same data
// ============================================================================

/// INTENT: "Multiple subscribers on the same topic all receive the same data."
///
/// Robotics guarantee: when a LiDAR driver publishes a scan, BOTH the SLAM
/// algorithm AND the obstacle avoidance module must receive it. If either
/// misses messages, the robot cannot navigate safely.
///
/// Implementation note: horus topics use shared memory, so multiple Topic
/// handles on the same name share the same ring buffer. The exact multi-
/// subscriber semantics depend on the backend (SPMC, MPMC). This test
/// verifies the user-visible guarantee using read_latest() which provides
/// non-consuming reads suitable for fan-out patterns.
#[test]
fn test_topic_intent_multiple_subscribers_all_receive() {
    cleanup_stale_shm();

    let name = common::unique("intent.multi_sub");

    // Create one publisher and three subscriber handles
    let publisher = Topic::<u64>::new(&name).expect("create publisher topic");
    let sub_a = Topic::<u64>::new(&name).expect("create subscriber A");
    let sub_b = Topic::<u64>::new(&name).expect("create subscriber B");
    let sub_c = Topic::<u64>::new(&name).expect("create subscriber C");

    let total = 10u64;

    // Send messages and verify all subscribers can read the latest after each send
    for i in 1..=total {
        publisher.send(i);

        // All subscribers should see the latest value via read_latest()
        let a = sub_a.read_latest();
        let b = sub_b.read_latest();
        let c = sub_c.read_latest();

        assert_eq!(
            a,
            Some(i),
            "Subscriber A should see value {} via read_latest(), got {:?}",
            i,
            a
        );
        assert_eq!(
            b,
            Some(i),
            "Subscriber B should see value {} via read_latest(), got {:?}",
            i,
            b
        );
        assert_eq!(
            c,
            Some(i),
            "Subscriber C should see value {} via read_latest(), got {:?}",
            i,
            c
        );
    }
}

// ============================================================================
// Test 4: Type safety enforcement
// ============================================================================

/// INTENT: "Topics enforce type safety — you cannot accidentally read wrong-typed data."
///
/// Robotics guarantee: if a node publishes CmdVel (velocity command) on topic
/// "cmd", another node subscribing to a different topic with a different type
/// cannot accidentally receive that data. Misinterpreting a velocity as a
/// position would be catastrophic.
///
/// Rust's type system enforces this at compile time: Topic<u64> and
/// Topic<String> are entirely different types, so you cannot call
/// `.recv()` on a Topic<String> and get a u64. This test verifies the
/// runtime behavior: separate typed topics with different names carry
/// independent data, and each topic only returns its own type.
#[test]
fn test_topic_intent_type_safety_enforced() {
    cleanup_stale_shm();

    let name_u64 = common::unique("intent.typed.u64");
    let name_string = common::unique("intent.typed.string");

    // Create two topics with DIFFERENT names and DIFFERENT types
    let topic_u64 = Topic::<u64>::new(&name_u64).expect("create u64 topic");
    let topic_string = Topic::<String>::new(&name_string).expect("create String topic");

    // Send typed data to each topic
    topic_u64.send(42u64);
    topic_string.send("hello".to_string());

    // Each topic only returns its own type — no cross-contamination.
    // u64 topic returns u64, String topic returns String.
    let u64_val = topic_u64.recv();
    assert_eq!(
        u64_val,
        Some(42u64),
        "u64 topic should return u64 value 42, got {:?}",
        u64_val
    );

    let string_val = topic_string.recv();
    assert_eq!(
        string_val,
        Some("hello".to_string()),
        "String topic should return String value 'hello', got {:?}",
        string_val
    );

    // u64 topic must NOT have "hello" in it, String topic must NOT have 42
    assert_eq!(
        topic_u64.recv(),
        None,
        "u64 topic should be empty (no cross-contamination from String topic)"
    );
    assert_eq!(
        topic_string.recv(),
        None,
        "String topic should be empty (no cross-contamination from u64 topic)"
    );

    // Verify a second round-trip works correctly for each type
    topic_u64.send(999u64);
    topic_string.send("world".to_string());

    assert_eq!(
        topic_u64.recv(),
        Some(999u64),
        "u64 topic second roundtrip failed"
    );
    assert_eq!(
        topic_string.recv(),
        Some("world".to_string()),
        "String topic second roundtrip failed"
    );
}
