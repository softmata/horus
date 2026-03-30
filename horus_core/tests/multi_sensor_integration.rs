//! Multi-sensor integration tests
//!
//! Verifies that multiple sensors publishing simultaneously on separate topics
//! do not interfere with each other. These scenarios model real robot sensor
//! arrays (IMU, odometry, lidar, camera, GPS) running concurrently.

mod common;
use common::cleanup_stale_shm;

use horus_core::communication::Topic;
use horus_core::core::{DurationExt, Node};
use horus_core::scheduling::Scheduler;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::{Arc, Mutex};

// ============================================================================
// Test 1: 5 sensors concurrent — no crosstalk
// ============================================================================

/// INTENT: "Five independent sensor nodes, each publishing to its own topic,
/// must never leak data into a sibling's topic."
///
/// This is the foundational isolation guarantee for multi-sensor robots.
/// If IMU data appears on the odometry topic, the controller computes
/// incorrect commands and the robot crashes.
#[test]
fn test_5_sensors_concurrent_no_crosstalk() {
    cleanup_stale_shm();

    // Each sensor publishes its own ID as the u64 value.
    // After 50 ticks, each subscriber must have received ONLY its sensor's ID.
    const SENSOR_COUNT: usize = 5;
    const TICK_COUNT: usize = 50;

    let sensor_names = [
        "sensor.imu",
        "sensor.odom",
        "sensor.lidar",
        "sensor.cam",
        "sensor.gps",
    ];

    // Create unique topic names
    let topic_names: Vec<String> = sensor_names.iter().map(|s| common::unique(s)).collect();

    // Shared receive buffers — one per sensor
    let received: Vec<Arc<Mutex<Vec<u64>>>> = (0..SENSOR_COUNT)
        .map(|_| Arc::new(Mutex::new(Vec::new())))
        .collect();

    struct SensorPublisher {
        label: String,
        id: u64,
        topic: Topic<u64>,
    }

    impl Node for SensorPublisher {
        fn name(&self) -> &str {
            &self.label
        }
        fn tick(&mut self) {
            self.topic.send(self.id);
        }
    }

    struct SensorSubscriber {
        label: String,
        topic: Topic<u64>,
        received: Arc<Mutex<Vec<u64>>>,
    }

    impl Node for SensorSubscriber {
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

    // Register publishers (order 0) and subscribers (order 1)
    for i in 0..SENSOR_COUNT {
        let pub_topic = Topic::<u64>::new(&topic_names[i]).expect("create pub topic");
        let sub_topic = Topic::<u64>::new(&topic_names[i]).expect("create sub topic");

        scheduler
            .add(SensorPublisher {
                label: format!("pub_{}", sensor_names[i]),
                id: i as u64,
                topic: pub_topic,
            })
            .order(0)
            .build()
            .unwrap();

        scheduler
            .add(SensorSubscriber {
                label: format!("sub_{}", sensor_names[i]),
                topic: sub_topic,
                received: received[i].clone(),
            })
            .order(1)
            .build()
            .unwrap();
    }

    // Run for enough ticks
    for _ in 0..TICK_COUNT {
        scheduler.tick_once().unwrap();
    }

    // Verify: each subscriber received ONLY its own sensor's ID
    for i in 0..SENSOR_COUNT {
        let vals = received[i].lock().unwrap();
        assert!(
            !vals.is_empty(),
            "Sensor {} ({}) received no messages",
            i,
            sensor_names[i]
        );
        for (msg_idx, &val) in vals.iter().enumerate() {
            assert_eq!(
                val, i as u64,
                "Crosstalk detected: sensor {} ({}) received value {} at message {} \
                 (expected {}). Data leaked from another topic.",
                i, sensor_names[i], val, msg_idx, i
            );
        }
    }
}

// ============================================================================
// Test 2: Multi-rate nodes — proportional tick counts
// ============================================================================

/// INTENT: "Nodes configured at different rates tick proportionally to their
/// configured frequencies."
///
/// A real robot runs perception at 30Hz, control at 200Hz, and logging at 10Hz.
/// The scheduler must respect these rates — a 200Hz node must tick roughly
/// 4x more than a 50Hz node over the same time window.
#[test]
fn test_multi_rate_proportional_ticks() {
    cleanup_stale_shm();

    let fast_ticks = Arc::new(AtomicU64::new(0));
    let medium_ticks = Arc::new(AtomicU64::new(0));
    let slow_ticks = Arc::new(AtomicU64::new(0));

    struct RateNode {
        label: &'static str,
        ticks: Arc<AtomicU64>,
    }

    impl Node for RateNode {
        fn name(&self) -> &str {
            self.label
        }
        fn tick(&mut self) {
            self.ticks.fetch_add(1, Ordering::SeqCst);
        }
    }

    let mut scheduler = Scheduler::new().tick_rate(200_u64.hz());

    scheduler
        .add(RateNode {
            label: "fast_200hz",
            ticks: fast_ticks.clone(),
        })
        .rate(200_u64.hz())
        .order(0)
        .build()
        .unwrap();

    scheduler
        .add(RateNode {
            label: "medium_50hz",
            ticks: medium_ticks.clone(),
        })
        .rate(50_u64.hz())
        .order(1)
        .build()
        .unwrap();

    scheduler
        .add(RateNode {
            label: "slow_10hz",
            ticks: slow_ticks.clone(),
        })
        .rate(10_u64.hz())
        .order(2)
        .build()
        .unwrap();

    // Run for 500ms
    scheduler.run_for(500_u64.ms()).unwrap();

    let fast = fast_ticks.load(Ordering::SeqCst);
    let medium = medium_ticks.load(Ordering::SeqCst);
    let slow = slow_ticks.load(Ordering::SeqCst);

    // All nodes must have ticked
    assert!(fast > 0, "Fast node did not tick");
    assert!(medium > 0, "Medium node did not tick");
    assert!(slow > 0, "Slow node did not tick");

    // Fast should tick more than medium, medium more than slow.
    // Ideal ratio: fast/medium=4, medium/slow=5.
    // Allow wide tolerance (within 3x of ideal) due to scheduling jitter.
    assert!(
        fast > medium,
        "Fast node ({}) should tick more than medium ({})",
        fast,
        medium
    );
    assert!(
        medium > slow,
        "Medium node ({}) should tick more than slow ({})",
        medium,
        slow
    );

    // Verify approximate ratios (within 3x tolerance of ideal)
    let fast_medium_ratio = fast as f64 / medium as f64;
    assert!(
        fast_medium_ratio > 1.0 && fast_medium_ratio < 12.0,
        "Fast/medium ratio {} is outside reasonable bounds (expected ~4.0)",
        fast_medium_ratio
    );

    let medium_slow_ratio = medium as f64 / slow as f64;
    assert!(
        medium_slow_ratio > 1.0 && medium_slow_ratio < 15.0,
        "Medium/slow ratio {} is outside reasonable bounds (expected ~5.0)",
        medium_slow_ratio
    );
}

// ============================================================================
// Test 3: Sensor timestamps are strictly monotonically increasing
// ============================================================================

/// INTENT: "Each sensor topic's sequence of values is strictly monotonically
/// increasing — no reordering, no duplication, no regression."
///
/// Timestamp monotonicity is a hard requirement for sensor fusion.
/// If a LiDAR scan arrives with a timestamp older than the previous one,
/// the SLAM algorithm produces a corrupted map.
#[test]
fn test_sensor_timestamps_monotonic() {
    cleanup_stale_shm();

    const NODE_COUNT: usize = 3;
    const TICK_COUNT: usize = 100;

    let topic_names: Vec<String> = (0..NODE_COUNT)
        .map(|i| common::unique(&format!("sensor.ts.{}", i)))
        .collect();

    let received: Vec<Arc<Mutex<Vec<u64>>>> = (0..NODE_COUNT)
        .map(|_| Arc::new(Mutex::new(Vec::new())))
        .collect();

    struct TimestampPublisher {
        label: String,
        counter: u64,
        topic: Topic<u64>,
    }

    impl Node for TimestampPublisher {
        fn name(&self) -> &str {
            &self.label
        }
        fn tick(&mut self) {
            self.counter += 1;
            self.topic.send(self.counter);
        }
    }

    struct TimestampSubscriber {
        label: String,
        topic: Topic<u64>,
        received: Arc<Mutex<Vec<u64>>>,
    }

    impl Node for TimestampSubscriber {
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
            .add(TimestampPublisher {
                label: format!("ts_pub_{}", i),
                counter: i as u64 * 10000, // offset per sensor to make values unique
                topic: pub_topic,
            })
            .order(0)
            .build()
            .unwrap();

        scheduler
            .add(TimestampSubscriber {
                label: format!("ts_sub_{}", i),
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

    // Verify strict monotonic increase per topic
    for i in 0..NODE_COUNT {
        let vals = received[i].lock().unwrap();
        assert!(
            vals.len() >= 10,
            "Sensor {} should have received at least 10 timestamps, got {}",
            i,
            vals.len()
        );

        for window in vals.windows(2) {
            assert!(
                window[1] > window[0],
                "Sensor {} timestamps not monotonically increasing: {} followed by {}",
                i,
                window[0],
                window[1]
            );
        }
    }
}
