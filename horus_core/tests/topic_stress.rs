#![allow(dead_code)]
//! Topic IPC stress tests
//!
//! High-throughput, multi-topic, large-message, and ring buffer overwrite
//! scenarios that push the Topic system beyond normal operating conditions.
//! Every test verifies data integrity under load.

mod common;
use common::cleanup_stale_shm;

use horus_core::communication::Topic;
use serde::{Deserialize, Serialize};

/// Generate a unique topic name per test using PID + monotonic counter.
fn unique_topic(suffix: &str) -> String {
    common::unique(suffix)
}

// ============================================================================
// Test 1: Rapid batched send/recv of 10,000 u64 messages
// ============================================================================

/// INTENT: The topic system must sustain high-throughput bursts without data
/// loss. Robotics control loops (1 kHz+) accumulate many messages between
/// consumer ticks — every single one must arrive intact.
#[test]
fn test_topic_stress_rapid_send_recv_10k() {
    let _shm_guard = cleanup_stale_shm();

    let name = unique_topic("stress.rapid10k");
    let topic = Topic::<u64>::new(&name).expect("create topic");

    let total: u64 = 10_000;
    let batch_size: u64 = 256;
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
            assert_eq!(
                msg, next_recv,
                "Data mismatch at index {}: expected {}, got {}",
                next_recv, next_recv, msg
            );
            next_recv += 1;
        }
    }

    assert_eq!(
        next_recv, total,
        "Should have received all {} messages, got {}",
        total, next_recv
    );

    // Verify no extra messages remain
    assert_eq!(
        topic.recv(),
        None,
        "Topic should be empty after draining all 10,000 messages"
    );
}

// ============================================================================
// Test 2: 10 concurrent topics, 100 messages each (1,000 total)
// ============================================================================

/// INTENT: The system must support many independent topics without cross-talk
/// or resource contention. A real robot has dozens of topics (sensors, commands,
/// diagnostics) active simultaneously — none may interfere with each other.
#[test]
fn test_topic_stress_multiple_topics_concurrent() {
    let _shm_guard = cleanup_stale_shm();

    let topic_count = 10;
    let msgs_per_topic = 100u64;

    // Create all topics upfront
    let topics: Vec<_> = (0..topic_count)
        .map(|i| {
            let name = unique_topic(&format!("stress.multi.{}", i));
            Topic::<u64>::new(&name).expect("create topic")
        })
        .collect();

    // Send 100 messages to each topic, using topic_index * 1000 + msg_index
    // as the value so we can verify provenance.
    for (topic_idx, topic) in topics.iter().enumerate() {
        for msg_idx in 0..msgs_per_topic {
            let value = (topic_idx as u64) * 1000 + msg_idx;
            topic.send(value);
        }
    }

    // Drain each topic and verify all messages arrived correctly
    let mut total_received: u64 = 0;
    for (topic_idx, topic) in topics.iter().enumerate() {
        let base = (topic_idx as u64) * 1000;
        // Batch drain within ring buffer capacity
        let mut batch_sent = 0u64;
        let mut batch_recv = 0u64;
        let batch_size = 50u64;

        while batch_recv < msgs_per_topic {
            let drain_end = (batch_sent + batch_size).min(msgs_per_topic);
            // Messages already sent above, just need to recv in batches
            // that fit within what's available
            while batch_recv < drain_end {
                let msg = topic.recv().unwrap_or_else(|| {
                    panic!(
                        "Topic {} recv returned None at msg {} (expected {})",
                        topic_idx,
                        batch_recv,
                        base + batch_recv
                    )
                });
                assert_eq!(
                    msg,
                    base + batch_recv,
                    "Topic {} message {} wrong: expected {}, got {}",
                    topic_idx,
                    batch_recv,
                    base + batch_recv,
                    msg
                );
                batch_recv += 1;
                total_received += 1;
            }
            batch_sent = drain_end;
        }

        // No leftovers
        assert_eq!(
            topic.recv(),
            None,
            "Topic {} should be empty after drain",
            topic_idx
        );
    }

    assert_eq!(
        total_received,
        (topic_count as u64) * msgs_per_topic,
        "Total messages received should be {}",
        (topic_count as u64) * msgs_per_topic
    );
}

// ============================================================================
// Test 3: Large message (100 KB struct) serde roundtrip through a topic
// ============================================================================

/// INTENT: Topics must handle messages much larger than a cache line without
/// corruption. Camera frames, point clouds, and maps can be tens to hundreds
/// of kilobytes. Every byte must survive the IPC roundtrip.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
struct LargePayload {
    data: Vec<u8>,
    checksum: u64,
}

#[test]
fn test_topic_stress_large_message_serde() {
    let _shm_guard = cleanup_stale_shm();

    let name = unique_topic("stress.large_msg");
    let topic = Topic::<LargePayload>::new(&name).expect("create large-message topic");

    // Build a 100 KB payload with a predictable pattern
    let size = 100 * 1024; // 100 KB
    let data: Vec<u8> = (0..size).map(|i| (i % 251) as u8).collect();
    let checksum: u64 = data.iter().map(|&b| b as u64).sum();

    let sent = LargePayload {
        data: data.clone(),
        checksum,
    };

    topic.send(sent.clone());

    let received = topic.recv().expect("recv should return the large message");

    assert_eq!(
        received.data.len(),
        size,
        "Received data length mismatch: expected {}, got {}",
        size,
        received.data.len()
    );
    assert_eq!(
        received.checksum, checksum,
        "Checksum mismatch: expected {}, got {}",
        checksum, received.checksum
    );
    assert_eq!(
        received.data, data,
        "Byte-level data mismatch in 100 KB payload"
    );
}

// ============================================================================
// Test 4: Alternating single send/recv 1,000 times
// ============================================================================

/// INTENT: The topic must work correctly in a strict one-at-a-time pattern
/// where each message is consumed before the next is sent. This models the
/// common case of a 1:1 producer-consumer pair running in lock-step.
/// No message may be skipped, duplicated, or corrupted.
#[test]
fn test_topic_stress_alternating_send_recv() {
    let _shm_guard = cleanup_stale_shm();

    let name = unique_topic("stress.alternate");
    let topic = Topic::<u64>::new(&name).expect("create topic");

    let total = 1_000u64;

    for i in 0..total {
        topic.send(i);
        let msg = topic
            .recv()
            .unwrap_or_else(|| panic!("recv returned None at alternation {}", i));
        assert_eq!(
            msg, i,
            "Alternation {}: expected {}, got {} (skip or duplicate)",
            i, i, msg
        );

        // Verify no extra message was produced
        assert_eq!(
            topic.recv(),
            None,
            "Alternation {}: topic should be empty after single recv",
            i
        );
    }
}

// ============================================================================
// Test 5: Overwrite ring buffer — send more than capacity
// ============================================================================

/// INTENT: When a producer sends more messages than the ring can hold and
/// nothing is draining it, `send` — the lossy publish — retires the OLDEST
/// slot and takes it. A consumer that attaches afterwards is handed the
/// newest `capacity` messages, not the first `capacity` ever sent.
///
/// This is the correct behavior for robotics: a subscriber attaching to a
/// sensor topic wants the most recent sample, and a ring nobody is reading
/// must not hold the newest data hostage. `Topic::send`'s contract spells
/// this out — see the keep-last-N reclaim in `send_lossy_retry`, and
/// `Topic::missed_count` ("drop-oldest under overload is by design").
/// `try_send` and `send_blocking` are the calls that refuse instead of
/// dropping; `send` never fails and never blocks, so it drops.
///
/// NOTE — THIS TEST'S OLD EXPECTATION WAS WRONG. It used to assert
/// `drained[0] == 0`, "FIFO — earliest messages kept", which pins the
/// pre-keep-last-N behaviour: an unattended full ring froze holding the FIRST
/// `capacity` messages for the life of the segment while the publisher kept
/// counting sends, so `topic list` showed a live rate while `topic echo`
/// replayed minutes-old payloads. The assertions below pin what the topic
/// actually guarantees, including the part that matters most — the newest
/// message IS present.
#[test]
fn test_topic_stress_overwrite_ring_buffer() {
    let _shm_guard = cleanup_stale_shm();

    let name = unique_topic("stress.overwrite");
    // Default capacity for u64 is 512. Send 1000 messages without consuming.
    let topic = Topic::<u64>::new(&name).expect("create topic");

    let send_count = 1_000u64;
    const RING_CAPACITY: u64 = 512;

    for i in 0..send_count {
        topic.send(i);
    }

    // Drain whatever is available in the ring buffer.
    let mut drained = Vec::new();
    while let Some(val) = topic.recv() {
        drained.push(val);
    }

    // The ring is bounded: it cannot have kept all 1000.
    assert!(
        !drained.is_empty(),
        "Should have received at least some messages after sending {}",
        send_count
    );
    assert!(
        drained.len() as u64 <= RING_CAPACITY,
        "Drained {} messages but ring capacity should be at most {}",
        drained.len(),
        RING_CAPACITY
    );
    assert!(
        (drained.len() as u64) < send_count,
        "Should have received fewer messages ({}) than sent ({}), \
         proving the ring buffer is bounded",
        drained.len(),
        send_count
    );

    // Every value must be one we actually sent.
    for &val in &drained {
        assert!(
            val < send_count,
            "Drained value {} exceeds send_count {}",
            val,
            send_count
        );
    }

    // What survived is a CONTIGUOUS ascending run of the sent sequence. The
    // ring retires whole messages off one end; it never reorders them, never
    // duplicates one, and never leaves a hole in the middle of what it kept.
    for window in drained.windows(2) {
        assert_eq!(
            window[1],
            window[0] + 1,
            "Drained values must be a contiguous ascending run, got {} after {}",
            window[1],
            window[0]
        );
    }

    // ...and the end it retires from is the OLD end. The newest message must
    // be in the ring: that is the entire point of retiring the oldest slot
    // instead of refusing the send. If this regresses, an unread topic
    // silently freezes on stale data while the publisher keeps counting sends
    // — the failure keep-last-N exists to prevent.
    assert_eq!(
        *drained.last().unwrap(),
        send_count - 1,
        "Newest message must survive on an undrained ring (keep-last-N); \
         drained {} messages, {}..={}",
        drained.len(),
        drained[0],
        drained.last().unwrap()
    );

    // The inverse of the old, wrong assertion, stated explicitly so a
    // regression back to "keep the first N forever" cannot pass silently.
    assert!(
        drained[0] > 0,
        "Oldest messages must have been retired, not kept — \
         first drained value was {} after sending {}",
        drained[0],
        send_count
    );
}
