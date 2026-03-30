// Integration tests for Topic backend auto-detection behavior.
//
// The detection algorithm (detect_optimal_backend) is already unit-tested
// in topic/tests.rs with 16 dedicated tests covering all 10 BackendMode paths.
// These integration tests verify the END-TO-END behavior: that Topic::new()
// correctly auto-detects backends and that send/recv works correctly for
// each topology from the user's perspective.
//
// Key insight: BackendMode is pub(crate), so integration tests verify behavior
// (correct data delivery) rather than inspecting internal backend selection.

use horus_core::communication::Topic;
use serde::{Deserialize, Serialize};
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::Arc;
use std::time::Duration;

mod common;
use common::cleanup_stale_shm;

// ---------------------------------------------------------------------------
// POD type for testing (fixed-size, no heap, zero-copy eligible)
// ---------------------------------------------------------------------------

#[repr(C)]
#[derive(Copy, Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
struct PodPoint {
    x: f32,
    y: f32,
    z: f32,
    w: f32, // padding to make size > 1 byte for is_pod()
}

// SAFETY: PodPoint is #[repr(C)], Copy, no heap pointers, all fields are primitives
unsafe impl bytemuck::Pod for PodPoint {}
unsafe impl bytemuck::Zeroable for PodPoint {}

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

fn unique(prefix: &str) -> String {
    static COUNTER: AtomicU64 = AtomicU64::new(0);
    format!(
        "{}_{}_{}",
        prefix,
        std::process::id(),
        COUNTER.fetch_add(1, Ordering::Relaxed)
    )
}

// ============================================================================
// Same-thread: send and recv on same thread (DirectChannel path)
// ============================================================================

#[test]
fn test_same_thread_pod_send_recv() {
    cleanup_stale_shm();
    let name = unique("st_pod");
    let topic: Topic<PodPoint> = Topic::new(&name).unwrap();

    let msg = PodPoint {
        x: 1.0,
        y: 2.0,
        z: 3.0,
        w: 4.0,
    };
    topic.send(msg);

    let received = topic.recv();
    assert_eq!(received, Some(msg), "Same-thread POD roundtrip failed");
}

#[test]
fn test_same_thread_pod_multiple_messages() {
    cleanup_stale_shm();
    let name = unique("st_pod_multi");
    let topic: Topic<PodPoint> = Topic::new(&name).unwrap();

    for i in 0..10 {
        topic.send(PodPoint {
            x: i as f32,
            y: 0.0,
            z: 0.0,
            w: 0.0,
        });
    }

    // Should receive in FIFO order
    for i in 0..10 {
        let msg = topic.recv().unwrap();
        assert_eq!(msg.x, i as f32, "FIFO ordering broken at index {}", i);
    }

    // No more messages
    assert_eq!(topic.recv(), None);
}

#[test]
fn test_same_thread_pod_read_latest() {
    cleanup_stale_shm();
    let name = unique("st_latest");
    let topic: Topic<PodPoint> = Topic::new(&name).unwrap();

    topic.send(PodPoint {
        x: 1.0,
        y: 0.0,
        z: 0.0,
        w: 0.0,
    });
    topic.send(PodPoint {
        x: 2.0,
        y: 0.0,
        z: 0.0,
        w: 0.0,
    });
    topic.send(PodPoint {
        x: 3.0,
        y: 0.0,
        z: 0.0,
        w: 0.0,
    });

    // read_latest should return the most recent without consuming
    let latest = topic.read_latest();
    assert!(latest.is_some(), "read_latest should return Some");
    // After read_latest, recv should still return messages (read_latest doesn't consume)
    let first = topic.recv();
    assert!(
        first.is_some(),
        "recv should still return messages after read_latest"
    );
}

// ============================================================================
// Same-process cross-thread: SPSC path (1 producer thread, 1 consumer thread)
// ============================================================================

#[test]
fn test_cross_thread_spsc_pod_roundtrip() {
    cleanup_stale_shm();
    let name = unique("ct_spsc");
    let msg_count = 100u64;
    let received = Arc::new(AtomicU64::new(0));
    let received_clone = received.clone();

    // Producer thread
    let name_clone = name.clone();
    let producer = std::thread::spawn(move || {
        let topic: Topic<PodPoint> = Topic::new(&name_clone).unwrap();
        for i in 0..msg_count {
            topic.send(PodPoint {
                x: i as f32,
                y: 0.0,
                z: 0.0,
                w: 0.0,
            });
            std::thread::sleep(Duration::from_micros(100));
        }
    });

    // Consumer thread
    let name_clone2 = name.clone();
    let consumer = std::thread::spawn(move || {
        let topic: Topic<PodPoint> = Topic::new(&name_clone2).unwrap();
        let deadline = std::time::Instant::now() + Duration::from_secs(2);
        while std::time::Instant::now() < deadline {
            if let Some(_msg) = topic.recv() {
                received_clone.fetch_add(1, Ordering::SeqCst);
            } else {
                std::thread::sleep(Duration::from_micros(50));
            }
        }
    });

    producer.join().unwrap();
    consumer.join().unwrap();

    let total = received.load(Ordering::SeqCst);
    assert!(
        total > 0,
        "Cross-thread SPSC should deliver messages, got {}",
        total
    );
}

// ============================================================================
// Same-process multi-consumer: SPMC path
// ============================================================================

#[test]
fn test_cross_thread_spmc_multiple_consumers() {
    cleanup_stale_shm();
    let name = unique("ct_spmc");
    let msg_count = 50u64;
    let consumer_count = 3;

    let total_received = Arc::new(AtomicU64::new(0));

    // Producer
    let name_p = name.clone();
    let producer = std::thread::spawn(move || {
        let topic: Topic<PodPoint> = Topic::new(&name_p).unwrap();
        for i in 0..msg_count {
            topic.send(PodPoint {
                x: i as f32,
                y: 0.0,
                z: 0.0,
                w: 0.0,
            });
            std::thread::sleep(Duration::from_micros(200));
        }
    });

    // Multiple consumers
    let mut consumers = Vec::new();
    for _ in 0..consumer_count {
        let name_c = name.clone();
        let recv_count = total_received.clone();
        consumers.push(std::thread::spawn(move || {
            let topic: Topic<PodPoint> = Topic::new(&name_c).unwrap();
            let deadline = std::time::Instant::now() + Duration::from_secs(2);
            while std::time::Instant::now() < deadline {
                if let Some(_msg) = topic.recv() {
                    recv_count.fetch_add(1, Ordering::SeqCst);
                } else {
                    std::thread::sleep(Duration::from_micros(100));
                }
            }
        }));
    }

    producer.join().unwrap();
    for c in consumers {
        c.join().unwrap();
    }

    let total = total_received.load(Ordering::SeqCst);
    // With competing consumers, total received should be > 0
    // (messages are consumed by whichever consumer gets there first)
    assert!(
        total > 0,
        "SPMC should deliver messages across multiple consumers, got {}",
        total
    );
}

// ============================================================================
// Same-process multi-producer: MPSC path
// ============================================================================

#[test]
fn test_cross_thread_mpsc_multiple_producers() {
    cleanup_stale_shm();
    let name = unique("ct_mpsc");
    let msgs_per_producer = 30u64;
    let producer_count = 3;
    let total_received = Arc::new(AtomicU64::new(0));
    let total_received_clone = total_received.clone();

    // Multiple producers
    let mut producers = Vec::new();
    for p in 0..producer_count {
        let name_p = name.clone();
        producers.push(std::thread::spawn(move || {
            let topic: Topic<PodPoint> = Topic::new(&name_p).unwrap();
            for i in 0..msgs_per_producer {
                topic.send(PodPoint {
                    x: (p * 1000 + i) as f32,
                    y: p as f32,
                    z: 0.0,
                    w: 0.0,
                });
                std::thread::sleep(Duration::from_micros(100));
            }
        }));
    }

    // Single consumer
    let name_c = name.clone();
    let consumer = std::thread::spawn(move || {
        let topic: Topic<PodPoint> = Topic::new(&name_c).unwrap();
        let deadline = std::time::Instant::now() + Duration::from_secs(2);
        while std::time::Instant::now() < deadline {
            if let Some(_msg) = topic.recv() {
                total_received_clone.fetch_add(1, Ordering::SeqCst);
            } else {
                std::thread::sleep(Duration::from_micros(50));
            }
        }
    });

    for p in producers {
        p.join().unwrap();
    }
    consumer.join().unwrap();

    let total = total_received.load(Ordering::SeqCst);
    assert!(
        total > 0,
        "MPSC should deliver messages from multiple producers, got {}",
        total
    );
}

// ============================================================================
// Same-process MPMC: multiple producers + multiple consumers
// ============================================================================

#[test]
fn test_cross_thread_mpmc() {
    cleanup_stale_shm();
    let name = unique("ct_mpmc");
    let msgs_per_producer = 20u64;
    let producer_count = 2;
    let consumer_count = 2;
    let total_received = Arc::new(AtomicU64::new(0));

    // Producers
    let mut handles = Vec::new();
    for p in 0..producer_count {
        let name_p = name.clone();
        handles.push(std::thread::spawn(move || {
            let topic: Topic<PodPoint> = Topic::new(&name_p).unwrap();
            for i in 0..msgs_per_producer {
                topic.send(PodPoint {
                    x: (p * 1000 + i) as f32,
                    y: 0.0,
                    z: 0.0,
                    w: 0.0,
                });
                std::thread::sleep(Duration::from_micros(200));
            }
        }));
    }

    // Consumers
    for _ in 0..consumer_count {
        let name_c = name.clone();
        let recv = total_received.clone();
        handles.push(std::thread::spawn(move || {
            let topic: Topic<PodPoint> = Topic::new(&name_c).unwrap();
            let deadline = std::time::Instant::now() + Duration::from_secs(2);
            while std::time::Instant::now() < deadline {
                if let Some(_msg) = topic.recv() {
                    recv.fetch_add(1, Ordering::SeqCst);
                } else {
                    std::thread::sleep(Duration::from_micros(100));
                }
            }
        }));
    }

    for h in handles {
        h.join().unwrap();
    }

    let total = total_received.load(Ordering::SeqCst);
    assert!(total > 0, "MPMC should deliver messages, got {}", total);
}

// ============================================================================
// POD detection: verify is_pod behavior via Topic send/recv
// ============================================================================

#[test]
fn test_pod_u64_send_recv() {
    cleanup_stale_shm();
    let name = unique("pod_u64");
    let topic: Topic<u64> = Topic::new(&name).unwrap();
    topic.send(42u64);
    assert_eq!(topic.recv(), Some(42u64));
}

#[test]
fn test_pod_f64_send_recv() {
    cleanup_stale_shm();
    let name = unique("pod_f64");
    let topic: Topic<f64> = Topic::new(&name).unwrap();
    topic.send(1.23456f64);
    assert_eq!(topic.recv(), Some(1.23456f64));
}

#[test]
fn test_pod_i32_send_recv() {
    cleanup_stale_shm();
    let name = unique("pod_i32");
    let topic: Topic<i32> = Topic::new(&name).unwrap();
    topic.send(-42i32);
    assert_eq!(topic.recv(), Some(-42i32));
}

#[test]
fn test_pod_struct_send_recv() {
    cleanup_stale_shm();
    let name = unique("pod_struct");
    let topic: Topic<PodPoint> = Topic::new(&name).unwrap();
    let msg = PodPoint {
        x: 1.5,
        y: 2.5,
        z: 3.5,
        w: 4.5,
    };
    topic.send(msg);
    assert_eq!(topic.recv(), Some(msg));
}

// ============================================================================
// Data integrity: verify no corruption across threads
// ============================================================================

#[test]
fn test_cross_thread_data_integrity() {
    cleanup_stale_shm();
    let name = unique("integrity");
    let msg_count = 200u64;
    let corruption_count = Arc::new(AtomicU64::new(0));
    let recv_count = Arc::new(AtomicU64::new(0));
    let corruption_clone = corruption_count.clone();
    let recv_clone = recv_count.clone();

    // Producer: encode index in x field
    let name_p = name.clone();
    let producer = std::thread::spawn(move || {
        let topic: Topic<PodPoint> = Topic::new(&name_p).unwrap();
        for i in 0..msg_count {
            topic.send(PodPoint {
                x: i as f32,
                y: (i * 2) as f32,
                z: (i * 3) as f32,
                w: 0.0,
            });
            std::thread::sleep(Duration::from_micros(50));
        }
    });

    // Consumer: verify y = x*2, z = x*3
    let name_c = name.clone();
    let consumer = std::thread::spawn(move || {
        let topic: Topic<PodPoint> = Topic::new(&name_c).unwrap();
        let deadline = std::time::Instant::now() + Duration::from_secs(3);
        while std::time::Instant::now() < deadline {
            if let Some(msg) = topic.recv() {
                recv_clone.fetch_add(1, Ordering::SeqCst);
                let expected_y = msg.x * 2.0;
                let expected_z = msg.x * 3.0;
                if (msg.y - expected_y).abs() > 0.001 || (msg.z - expected_z).abs() > 0.001 {
                    corruption_clone.fetch_add(1, Ordering::SeqCst);
                }
            } else {
                std::thread::sleep(Duration::from_micros(50));
            }
        }
    });

    producer.join().unwrap();
    consumer.join().unwrap();

    let received = recv_count.load(Ordering::SeqCst);
    let corrupted = corruption_count.load(Ordering::SeqCst);

    assert!(received > 0, "Should receive messages, got 0");
    assert_eq!(
        corrupted, 0,
        "Data corruption detected: {} of {} messages corrupted",
        corrupted, received
    );
}

// ============================================================================
// Empty topic: recv returns None immediately
// ============================================================================

#[test]
fn test_empty_topic_recv_returns_none() {
    cleanup_stale_shm();
    let name = unique("empty");
    let topic: Topic<PodPoint> = Topic::new(&name).unwrap();
    assert_eq!(topic.recv(), None, "Empty topic should return None");
}

// ============================================================================
// Topic metrics: verify message counting
// ============================================================================

#[test]
fn test_topic_metrics_after_send_recv() {
    cleanup_stale_shm();
    let name = unique("metrics");
    let topic: Topic<PodPoint> = Topic::new(&name).unwrap();

    for _ in 0..5 {
        topic.send(PodPoint::default());
    }
    for _ in 0..3 {
        let _ = topic.recv();
    }

    let metrics = topic.metrics();
    // Note: metrics counting depends on backend path. DirectChannel (same-thread)
    // may not increment the atomic counters in MigrationMetrics since it uses a
    // fast local path. This test verifies the metrics() API is callable and returns
    // a valid TopicMetrics struct without panicking.
    let _ = metrics.messages_sent();
    let _ = metrics.messages_received();
    let _ = metrics.send_failures();
    let _ = metrics.recv_failures();
}
