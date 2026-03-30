//! Topic lifecycle integration tests
//!
//! Tests topic creation/drop/recreation, multi-type pub/sub, FIFO ordering,
//! empty recv semantics, high-throughput same-thread, and Pod message roundtrip.

mod common;
use common::cleanup_stale_shm;

use horus_core::communication::Topic;

/// Generate a unique topic name per test using PID + monotonic counter.
fn unique_topic(suffix: &str) -> String {
    common::unique(suffix)
}

/// Test 1: Create a Topic<u64>, send messages, drop it. Recreate with the same
/// name and verify the new topic works correctly for send/recv.
#[test]
fn test_topic_recreate_after_drop() {
    cleanup_stale_shm();

    let name = unique_topic("lifecycle.recreate");

    // Phase 1: create, send, then drop
    {
        let topic = Topic::<u64>::new(&name).expect("create topic (first)");
        topic.send(111);
        topic.send(222);
        // topic is dropped here
    }

    // Phase 2: recreate with the same name
    let topic = Topic::<u64>::new(&name).expect("create topic (second)");

    // Send fresh data through the recreated topic
    topic.send(333);
    topic.send(444);

    let a = topic.recv();
    let b = topic.recv();

    assert_eq!(
        a,
        Some(333),
        "First recv on recreated topic should return 333"
    );
    assert_eq!(
        b,
        Some(444),
        "Second recv on recreated topic should return 444"
    );
}

/// Test 2: Send and receive through topics typed with several basic types.
/// For each type, create a topic, send a value, recv it, and assert equality.
#[test]
fn test_topic_send_recv_basic_types() {
    cleanup_stale_shm();

    // u32
    {
        let name = unique_topic("lifecycle.u32");
        let topic = Topic::<u32>::new(&name).expect("create u32 topic");
        topic.send(42u32);
        assert_eq!(topic.recv(), Some(42u32), "u32 roundtrip failed");
    }

    // f64
    {
        let name = unique_topic("lifecycle.f64");
        let topic = Topic::<f64>::new(&name).expect("create f64 topic");
        topic.send(std::f64::consts::PI);
        let received = topic.recv().expect("f64 recv should not be None");
        assert!(
            (received - std::f64::consts::PI).abs() < f64::EPSILON,
            "f64 roundtrip mismatch: expected PI, got {}",
            received
        );
    }

    // String
    {
        let name = unique_topic("lifecycle.string");
        let topic = Topic::<String>::new(&name).expect("create String topic");
        let msg = "hello horus".to_string();
        topic.send(msg.clone());
        assert_eq!(topic.recv(), Some(msg), "String roundtrip failed");
    }

    // Vec<u8>
    {
        let name = unique_topic("lifecycle.vec_u8");
        let topic = Topic::<Vec<u8>>::new(&name).expect("create Vec<u8> topic");
        let msg: Vec<u8> = vec![0xDE, 0xAD, 0xBE, 0xEF];
        topic.send(msg.clone());
        assert_eq!(topic.recv(), Some(msg), "Vec<u8> roundtrip failed");
    }
}

/// Test 3: Send 100 messages in order, recv all 100, assert FIFO ordering.
#[test]
fn test_topic_multiple_messages_fifo_order() {
    cleanup_stale_shm();

    let name = unique_topic("lifecycle.fifo");
    let topic = Topic::<u64>::new(&name).expect("create fifo topic");

    let count = 100u64;
    for i in 0..count {
        topic.send(i);
    }

    for i in 0..count {
        let msg = topic
            .recv()
            .unwrap_or_else(|| panic!("recv returned None at index {}", i));
        assert_eq!(
            msg, i,
            "FIFO violation at index {}: expected {}, got {}",
            i, i, msg
        );
    }

    // Verify no extra messages remain
    assert_eq!(
        topic.recv(),
        None,
        "Topic should be empty after draining all 100 messages"
    );
}

/// Test 4: recv on a topic with no messages sent should return None.
#[test]
fn test_topic_recv_returns_none_when_empty() {
    cleanup_stale_shm();

    let name = unique_topic("lifecycle.empty");
    let topic = Topic::<u64>::new(&name).expect("create empty topic");

    assert_eq!(
        topic.recv(),
        None,
        "First recv on empty topic should be None"
    );
    assert_eq!(
        topic.recv(),
        None,
        "Second recv on empty topic should still be None"
    );
    assert_eq!(
        topic.recv(),
        None,
        "Third recv on empty topic should still be None"
    );
}

/// Test 5: Send 10,000 u64 messages on a single topic, same thread. Assert
/// all are received correctly (no data loss) and elapsed time is < 100ms.
///
/// Uses batched send/recv to stay within the ring buffer's default capacity.
#[test]
fn test_topic_high_throughput_same_thread() {
    cleanup_stale_shm();

    let name = unique_topic("lifecycle.throughput");
    let topic = Topic::<u64>::new(&name).expect("create throughput topic");

    let total: u64 = 10_000;
    let batch_size: u64 = 256; // well within default ring capacity

    let start = std::time::Instant::now();
    let mut next_send: u64 = 0;
    let mut next_recv: u64 = 0;

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
            assert_eq!(msg, next_recv, "Data mismatch at index {}", next_recv);
            next_recv += 1;
        }
    }

    let elapsed = start.elapsed();

    assert_eq!(
        next_recv, total,
        "Should have received all {} messages",
        total
    );
    assert!(
        elapsed.as_millis() < 100,
        "10,000 send/recv should complete in < 100ms, took {}ms",
        elapsed.as_millis()
    );

    // Verify no extra messages
    assert_eq!(topic.recv(), None, "Topic should be empty after drain");
}

/// Test 6: Send a Pod message type (CmdVel) through a topic and verify all
/// fields match after roundtrip.
#[test]
fn test_topic_pod_message_roundtrip() {
    cleanup_stale_shm();

    use horus_robotics::CmdVel;

    let name = unique_topic("lifecycle.pod");
    let topic = Topic::<CmdVel>::new(&name).expect("create CmdVel topic");

    let sent = CmdVel::with_timestamp(1.5, -0.3, 123456789);
    topic.send(sent);

    let received = topic.recv().expect("CmdVel recv should not be None");

    assert!(
        (received.linear - sent.linear).abs() < f32::EPSILON,
        "CmdVel linear mismatch: expected {}, got {}",
        sent.linear,
        received.linear
    );
    assert!(
        (received.angular - sent.angular).abs() < f32::EPSILON,
        "CmdVel angular mismatch: expected {}, got {}",
        sent.angular,
        received.angular
    );
    assert_eq!(
        received.timestamp_ns, sent.timestamp_ns,
        "CmdVel timestamp mismatch"
    );
}
