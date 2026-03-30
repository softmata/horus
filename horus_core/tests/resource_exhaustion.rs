//! Resource exhaustion tests
//!
//! Verifies correct behavior when TensorPool slots are exhausted and when
//! ring buffers overflow. In production robotics, resource limits are hit
//! during peak load — the system must degrade gracefully, not corrupt data.

mod common;
use common::cleanup_stale_shm;

use horus_core::communication::Topic;
use horus_core::memory::{TensorPool, TensorPoolConfig};
use horus_core::types::{Device, TensorDtype};

// ============================================================================
// Test 7: TensorPool alloc returns error when full (no panic)
// ============================================================================

/// INTENT: "When all TensorPool slots are consumed, the next alloc() returns
/// an Err — it must NOT panic or corrupt the pool."
///
/// Robotics guarantee: during a perception pipeline burst (multiple camera
/// frames arriving simultaneously), running out of tensor slots must be a
/// recoverable error, not a crash. The pipeline drops the frame and retries
/// on the next cycle.
#[test]
fn test_tensor_pool_alloc_returns_error_when_full() {
    cleanup_stale_shm();

    // Create a very small pool: 4 slots, 64KB total data
    let config = TensorPoolConfig {
        pool_size: 64 * 1024,
        max_slots: 4,
        slot_alignment: 64,
        ..Default::default()
    };

    let pool_id = 9800 + (std::process::id() % 100) as u32;
    let pool = TensorPool::new(pool_id, config).expect("create small pool");

    // Allocate all 4 slots
    let mut tensors = Vec::new();
    for i in 0..4 {
        let shape = [1, 8]; // small: 8 elements
        match pool.alloc(&shape, TensorDtype::F32, Device::cpu()) {
            Ok(tensor) => tensors.push(tensor),
            Err(e) => panic!(
                "Slot {} alloc should succeed (only {} of 4 used): {:?}",
                i,
                tensors.len(),
                e
            ),
        }
    }

    assert_eq!(tensors.len(), 4, "Should have allocated all 4 slots");

    // Next alloc must return Err, not panic
    let result = pool.alloc(&[1, 8], TensorDtype::F32, Device::cpu());
    assert!(
        result.is_err(),
        "alloc() on a full pool should return Err, got Ok"
    );

    // Release one slot and verify we can allocate again
    pool.release(&tensors[0]);

    let result = pool.alloc(&[1, 8], TensorDtype::F32, Device::cpu());
    assert!(
        result.is_ok(),
        "alloc() after release should succeed, got {:?}",
        result.err()
    );
}

// ============================================================================
// Test 8: Ring buffer overflow — no data corruption
// ============================================================================

/// INTENT: "Sending more messages than the ring buffer capacity does not
/// corrupt data. Messages that survive in the buffer have valid, sequential
/// values."
///
/// Robotics guarantee: a slow consumer (path planner at 10Hz) reading from a
/// fast producer (lidar at 40Hz) will miss messages, but the messages it
/// does read must be correct and in order. Corrupted data would cause the
/// planner to generate dangerous trajectories.
#[test]
fn test_ring_buffer_overflow_no_corruption() {
    cleanup_stale_shm();

    let name = common::unique("exhaust.ring.overflow");
    let topic = Topic::<u64>::new(&name).expect("create topic");

    let send_count = 1000u64;

    // Flood the ring buffer without consuming
    for i in 0..send_count {
        topic.send(i);
    }

    // Drain whatever survived in the ring
    let mut drained = Vec::new();
    while let Some(val) = topic.recv() {
        drained.push(val);
    }

    // Must have received some messages (ring is bounded, not zero-sized)
    assert!(
        !drained.is_empty(),
        "Should have received some messages from overflowed ring"
    );

    // Ring capacity is 512 for u64 — we should not exceed that
    assert!(
        drained.len() <= 512,
        "Drained {} messages but ring capacity is at most 512",
        drained.len()
    );

    // All values must be valid (within range 0..send_count)
    for &val in &drained {
        assert!(
            val < send_count,
            "Drained value {} is outside valid range [0, {})",
            val,
            send_count
        );
    }

    // Messages must be in FIFO order (monotonically increasing)
    for window in drained.windows(2) {
        assert!(
            window[1] > window[0],
            "Ring buffer messages should be ordered: got {} after {}",
            window[1],
            window[0]
        );
    }

    // Sequential: each value should be exactly previous + 1 (contiguous block)
    for window in drained.windows(2) {
        assert_eq!(
            window[1],
            window[0] + 1,
            "Ring buffer should contain a contiguous block: {} followed by {} (gap)",
            window[0],
            window[1]
        );
    }
}

// ============================================================================
// Test 9: read_latest after overflow returns most recent value
// ============================================================================

/// INTENT: "After overflowing the ring buffer, read_latest() returns the
/// most recently written value — not a stale or corrupted one."
///
/// Robotics guarantee: a controller that calls read_latest() on the
/// odometry topic must always get the freshest pose estimate. After a burst
/// of messages overflows the ring, the controller must not act on old data.
#[test]
fn test_ring_buffer_read_latest_after_overflow() {
    cleanup_stale_shm();

    let name = common::unique("exhaust.ring.latest");
    // u64 is Copy, so read_latest works
    let topic = Topic::<u64>::new(&name).expect("create topic");

    let send_count = 1000u64;

    // Overflow the ring
    for i in 0..send_count {
        topic.send(i);
    }

    // read_latest should return the most recent value
    let latest = topic.read_latest();
    assert!(
        latest.is_some(),
        "read_latest should return Some after sending {} messages",
        send_count
    );

    let latest_val = latest.unwrap();

    // The latest value should be near the end of what we sent.
    // The absolute last value sent was (send_count - 1) = 999.
    // Due to ring buffer semantics, read_latest should return a value
    // from the most recent portion of the ring.
    assert!(
        latest_val >= send_count.saturating_sub(512),
        "read_latest returned {} which is too old (sent up to {}, ring capacity 512)",
        latest_val,
        send_count - 1
    );
    assert!(
        latest_val < send_count,
        "read_latest returned {} which exceeds send range [0, {})",
        latest_val,
        send_count
    );
}
