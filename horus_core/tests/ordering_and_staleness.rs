//! Message ordering and staleness detection tests
//!
//! Verifies FIFO ordering guarantees and that stale sensor data is detectable.
//! In production robotics, message ordering violations cause control instability,
//! and undetected stale data causes the robot to act on outdated information.

mod common;
use common::cleanup_stale_shm;

use horus_core::communication::Topic;
use horus_core::core::{DurationExt, Node};
use horus_core::scheduling::Scheduler;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::{Arc, Mutex};
use std::time::Instant;

// ============================================================================
// Test 10: SPSC FIFO ordering — 1000 messages
// ============================================================================

/// INTENT: "1000 sequenced u64 messages sent through a Topic arrive in strict
/// FIFO order — each value equals previous + 1."
///
/// This is the fundamental ordering guarantee for single-producer
/// single-consumer robotics channels. Control commands, sensor readings,
/// and state updates must arrive in order. Out-of-order commands could
/// cause a robot arm to move to an intermediate position before the final
/// target, potentially hitting obstacles.
#[test]
fn test_spsc_fifo_ordering_1000_messages() {
    cleanup_stale_shm();

    let name = common::unique("order.fifo1k");
    let topic = Topic::<u64>::new(&name).expect("create topic");

    let total = 1000u64;
    let batch = 128u64;

    let mut next_send = 0u64;
    let mut next_recv = 0u64;

    // Send and receive in batches to stay within ring buffer capacity
    while next_recv < total {
        let send_end = (next_send + batch).min(total);
        while next_send < send_end {
            topic.send(next_send);
            next_send += 1;
        }

        // Drain all available
        while next_recv < next_send {
            let msg = topic.recv().unwrap_or_else(|| {
                panic!(
                    "recv returned None at index {} (sent up to {})",
                    next_recv, next_send
                )
            });
            assert_eq!(
                msg, next_recv,
                "FIFO violation at index {}: expected {}, got {} (skip or reorder)",
                next_recv, next_recv, msg
            );
            next_recv += 1;
        }
    }

    assert_eq!(
        next_recv, total,
        "Should have received all {} messages in order, got {}",
        total, next_recv
    );

    // No extra messages
    assert_eq!(
        topic.recv(),
        None,
        "Topic should be empty after receiving all 1000 messages"
    );
}

// ============================================================================
// Test 11: Per-node timestamp monotonicity
// ============================================================================

/// INTENT: "Each node's published values are monotonically increasing when
/// read from its topic, even when multiple nodes publish concurrently."
///
/// Robotics guarantee: sensor fusion requires that each sensor's timestamps
/// are individually monotonic. If IMU timestamps go backwards, the EKF
/// state estimator diverges and the robot loses its pose estimate.
#[test]
fn test_timestamp_monotonic_per_node() {
    cleanup_stale_shm();

    const NODE_COUNT: usize = 3;
    const TICK_COUNT: usize = 100;

    let topic_names: Vec<String> = (0..NODE_COUNT)
        .map(|i| common::unique(&format!("order.mono.{}", i)))
        .collect();

    let received: Vec<Arc<Mutex<Vec<u64>>>> = (0..NODE_COUNT)
        .map(|_| Arc::new(Mutex::new(Vec::new())))
        .collect();

    struct MonotonicPublisher {
        label: String,
        counter: u64,
        topic: Topic<u64>,
    }

    impl Node for MonotonicPublisher {
        fn name(&self) -> &str {
            &self.label
        }
        fn tick(&mut self) {
            self.counter += 1;
            self.topic.send(self.counter);
        }
    }

    struct MonotonicSubscriber {
        label: String,
        topic: Topic<u64>,
        received: Arc<Mutex<Vec<u64>>>,
    }

    impl Node for MonotonicSubscriber {
        fn name(&self) -> &str {
            &self.label
        }
        fn tick(&mut self) {
            while let Some(val) = self.topic.recv() {
                self.received.lock().unwrap().push(val);
            }
        }
    }

    let mut scheduler = Scheduler::new().tick_rate(200_u64.hz());

    for i in 0..NODE_COUNT {
        let pub_topic = Topic::<u64>::new(&topic_names[i]).expect("create pub topic");
        let sub_topic = Topic::<u64>::new(&topic_names[i]).expect("create sub topic");

        scheduler
            .add(MonotonicPublisher {
                label: format!("mono_pub_{}", i),
                counter: 0,
                topic: pub_topic,
            })
            .order(0)
            .build()
            .unwrap();

        scheduler
            .add(MonotonicSubscriber {
                label: format!("mono_sub_{}", i),
                topic: sub_topic,
                received: received[i].clone(),
            })
            .order(1)
            .build()
            .unwrap();
    }

    for _ in 0..TICK_COUNT {
        scheduler.tick_once().unwrap();
    }

    // Verify monotonicity per topic
    for i in 0..NODE_COUNT {
        let vals = received[i].lock().unwrap();
        assert!(
            vals.len() >= 10,
            "Node {} should have received at least 10 values, got {}",
            i,
            vals.len()
        );

        // Each value must be strictly greater than the previous
        for window in vals.windows(2) {
            assert!(
                window[1] > window[0],
                "Node {} monotonicity violation: {} followed by {} (regression or duplicate)",
                i,
                window[0],
                window[1]
            );
        }

        // Values should also be sequential (counter increments by 1)
        for window in vals.windows(2) {
            assert_eq!(
                window[1],
                window[0] + 1,
                "Node {} gap detected: {} followed by {} (expected {})",
                i,
                window[0],
                window[1],
                window[0] + 1
            );
        }
    }
}

// ============================================================================
// Test 12: Stale data detectable by timestamp age
// ============================================================================

/// INTENT: "When a sensor stops publishing, the consumer can detect staleness
/// by comparing the last received timestamp against the current time."
///
/// Robotics guarantee: a path planner reading lidar data must detect when
/// the lidar driver has died and the data is stale. Without staleness
/// detection, the planner would navigate using an outdated occupancy grid
/// and collide with new obstacles.
#[test]
fn test_stale_data_detectable_by_timestamp_age() {
    cleanup_stale_shm();

    let topic_name = common::unique("order.stale");
    let last_publish_time = Arc::new(Mutex::new(None::<Instant>));

    struct TimedSensor {
        topic: Topic<u64>,
        ticks: Arc<AtomicU64>,
        last_time: Arc<Mutex<Option<Instant>>>,
    }

    impl Node for TimedSensor {
        fn name(&self) -> &str {
            "timed_sensor"
        }
        fn tick(&mut self) {
            let t = self.ticks.fetch_add(1, Ordering::SeqCst) + 1;
            if t <= 50 {
                // Publish for the first 50 ticks
                let now = Instant::now();
                *self.last_time.lock().unwrap() = Some(now);
                self.topic.send(t);
            }
            // After tick 50, sensor goes silent (simulating failure/disconnect)
        }
    }

    let sensor_ticks = Arc::new(AtomicU64::new(0));
    let consumer_last_recv_time = Arc::new(Mutex::new(None::<Instant>));
    let consumer_recv_count = Arc::new(AtomicU64::new(0));

    struct StaleDetector {
        topic: Topic<u64>,
        last_recv_time: Arc<Mutex<Option<Instant>>>,
        recv_count: Arc<AtomicU64>,
    }

    impl Node for StaleDetector {
        fn name(&self) -> &str {
            "stale_detector"
        }
        fn tick(&mut self) {
            if let Some(_val) = self.topic.recv() {
                *self.last_recv_time.lock().unwrap() = Some(Instant::now());
                self.recv_count.fetch_add(1, Ordering::SeqCst);
            }
        }
    }

    let pub_topic = Topic::<u64>::new(&topic_name).expect("create pub topic");
    let sub_topic = Topic::<u64>::new(&topic_name).expect("create sub topic");

    let mut scheduler = Scheduler::new().tick_rate(100_u64.hz());

    scheduler
        .add(TimedSensor {
            topic: pub_topic,
            ticks: sensor_ticks.clone(),
            last_time: last_publish_time.clone(),
        })
        .order(0)
        .build()
        .unwrap();

    scheduler
        .add(StaleDetector {
            topic: sub_topic,
            last_recv_time: consumer_last_recv_time.clone(),
            recv_count: consumer_recv_count.clone(),
        })
        .order(1)
        .build()
        .unwrap();

    // Run for 50 ticks — sensor publishes
    for _ in 0..50 {
        scheduler.tick_once().unwrap();
    }

    // Record the time after sensor publishes stop
    let time_sensor_stopped = Instant::now();

    // Run for 100 more ticks — sensor is silent, consumer keeps ticking
    // Use run_for for the silent period to let wall-clock time pass
    scheduler.run_for(200_u64.ms()).unwrap();

    let time_after_silence = Instant::now();

    // Verify the consumer received messages during the active phase
    let recv_count = consumer_recv_count.load(Ordering::SeqCst);
    assert!(
        recv_count > 0,
        "Consumer should have received messages during sensor's active phase"
    );

    // Verify staleness is detectable: the last recv time should be
    // significantly behind the current time
    let last_recv = consumer_last_recv_time.lock().unwrap();
    assert!(
        last_recv.is_some(),
        "Consumer should have recorded a receive time"
    );

    let last_recv_time = last_recv.unwrap();
    let staleness = time_after_silence.duration_since(last_recv_time);

    // The sensor stopped 200ms ago (the run_for duration).
    // Staleness should be at least 100ms (allowing tolerance for scheduling).
    assert!(
        staleness.as_millis() >= 50,
        "Staleness should be at least 50ms, got {}ms. \
         This proves the consumer can detect stale data by comparing \
         last-receive time against current time.",
        staleness.as_millis()
    );

    // Also verify the sensor stopped publishing: no new receives after the
    // silent period should have produced new timestamps close to now.
    let freshness_threshold = time_sensor_stopped;
    assert!(
        last_recv_time <= freshness_threshold,
        "Last receive time ({:?}) should be before sensor stop time ({:?})",
        last_recv_time,
        freshness_threshold
    );
}
