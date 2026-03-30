#![allow(dead_code)]
//! Failure injection tests
//!
//! Verifies that the scheduler degrades gracefully when nodes crash.
//! In production robotics, individual nodes (diagnostics, logging, non-critical
//! sensors) may fail — the system must keep running.

mod common;
use common::cleanup_stale_shm;

use horus_core::communication::Topic;
use horus_core::core::{DurationExt, Node};
use horus_core::scheduling::{FailurePolicy, Scheduler};
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::{Arc, Mutex};

// ============================================================================
// Test 4: Node crash — system survives
// ============================================================================

/// INTENT: "When one node panics, the remaining healthy nodes keep ticking."
///
/// Robotics guarantee: if a non-critical diagnostics node crashes, the motor
/// controller and IMU driver must keep running. The scheduler uses
/// catch_unwind to isolate panicking nodes.
#[test]
fn test_node_crash_system_survives() {
    cleanup_stale_shm();

    let lidar_ticks = Arc::new(AtomicU64::new(0));
    let imu_ticks = Arc::new(AtomicU64::new(0));
    let controller_ticks = Arc::new(AtomicU64::new(0));

    struct HealthyNode {
        label: &'static str,
        ticks: Arc<AtomicU64>,
    }

    impl Node for HealthyNode {
        fn name(&self) -> &str {
            self.label
        }
        fn tick(&mut self) {
            self.ticks.fetch_add(1, Ordering::SeqCst);
        }
    }

    struct CrashingLidar {
        ticks: Arc<AtomicU64>,
    }

    impl Node for CrashingLidar {
        fn name(&self) -> &str {
            "lidar"
        }
        fn tick(&mut self) {
            let t = self.ticks.fetch_add(1, Ordering::SeqCst) + 1;
            if t >= 10 {
                panic!("lidar hardware fault at tick {}", t);
            }
        }
    }

    let mut scheduler = Scheduler::new().tick_rate(100_u64.hz());

    let _ = scheduler
        .add(CrashingLidar {
            ticks: lidar_ticks.clone(),
        })
        .order(0)
        .failure_policy(FailurePolicy::Ignore)
        .build();

    scheduler
        .add(HealthyNode {
            label: "imu",
            ticks: imu_ticks.clone(),
        })
        .order(1)
        .build()
        .unwrap();

    scheduler
        .add(HealthyNode {
            label: "controller",
            ticks: controller_ticks.clone(),
        })
        .order(2)
        .build()
        .unwrap();

    // Run for 200ms — at 100Hz that is ~20 ticks.
    // Lidar panics at tick 10; imu and controller should keep going.
    let _ = scheduler.run_for(200_u64.ms());

    let lidar = lidar_ticks.load(Ordering::SeqCst);
    let imu = imu_ticks.load(Ordering::SeqCst);
    let controller = controller_ticks.load(Ordering::SeqCst);

    // Lidar should have ticked at least 10 times before crashing
    assert!(
        lidar >= 10,
        "Lidar should have ticked at least 10 times before crash, got {}",
        lidar
    );

    // Healthy nodes should have continued well past the crash point
    assert!(
        imu > 10,
        "IMU should have kept ticking after lidar crash, got {} ticks",
        imu
    );
    assert!(
        controller > 10,
        "Controller should have kept ticking after lidar crash, got {} ticks",
        controller
    );
}

// ============================================================================
// Test 5: Multiple node crashes — system still stable
// ============================================================================

/// INTENT: "When multiple nodes crash at different times, the surviving nodes
/// keep running and the scheduler does not hang or deadlock."
///
/// Stress-tests fault isolation: 2 of 5 nodes crash, but the remaining 3
/// continue processing. The scheduler must not enter an undefined state.
#[test]
fn test_multiple_node_crashes_still_stable() {
    cleanup_stale_shm();

    let tick_counts: Vec<Arc<AtomicU64>> = (0..5).map(|_| Arc::new(AtomicU64::new(0))).collect();

    struct CountingNode {
        label: String,
        ticks: Arc<AtomicU64>,
        crash_at: Option<u64>,
    }

    impl Node for CountingNode {
        fn name(&self) -> &str {
            &self.label
        }
        fn tick(&mut self) {
            let t = self.ticks.fetch_add(1, Ordering::SeqCst) + 1;
            if let Some(crash_tick) = self.crash_at {
                if t >= crash_tick {
                    panic!("{} crashed at tick {}", self.label, t);
                }
            }
        }
    }

    let mut scheduler = Scheduler::new().tick_rate(100_u64.hz());

    let configs: Vec<(&str, Option<u64>)> = vec![
        ("nav_node", None),       // healthy
        ("crash_early", Some(5)), // crashes at tick 5
        ("perception", None),     // healthy
        ("crash_late", Some(15)), // crashes at tick 15
        ("motor_ctrl", None),     // healthy
    ];

    for (i, (label, crash_at)) in configs.iter().enumerate() {
        let builder = scheduler
            .add(CountingNode {
                label: label.to_string(),
                ticks: tick_counts[i].clone(),
                crash_at: *crash_at,
            })
            .order(i as u32)
            .failure_policy(FailurePolicy::Ignore);
        let _ = builder.build();
    }

    // Run for 300ms — enough for all crashes to happen and survivors to keep going
    let _ = scheduler.run_for(300_u64.ms());

    let counts: Vec<u64> = tick_counts
        .iter()
        .map(|c| c.load(Ordering::SeqCst))
        .collect();

    // Healthy nodes (indices 0, 2, 4) should have ticked well past crash points
    for &healthy_idx in &[0usize, 2, 4] {
        assert!(
            counts[healthy_idx] > 15,
            "Healthy node {} ({}) should have ticked >15 times, got {}",
            healthy_idx,
            configs[healthy_idx].0,
            counts[healthy_idx]
        );
    }

    // Crashing nodes should have ticked at least up to their crash point
    assert!(
        counts[1] >= 5,
        "crash_early should have ticked at least 5 times, got {}",
        counts[1]
    );
    assert!(
        counts[3] >= 15,
        "crash_late should have ticked at least 15 times, got {}",
        counts[3]
    );
}

// ============================================================================
// Test 6: Crashed node — consumer recv returns None
// ============================================================================

/// INTENT: "After a publishing node crashes, consumers reading from its topic
/// eventually get None — no stale data is endlessly repeated."
///
/// Robotics guarantee: if the lidar driver dies, the path planner must detect
/// data staleness and switch to a safe mode. Endlessly replaying the last
/// scan would cause the robot to drive into newly-appeared obstacles.
#[test]
fn test_crashed_node_recv_returns_none() {
    cleanup_stale_shm();

    let topic_name = common::unique("crash.sensor");
    let sensor_ticks = Arc::new(AtomicU64::new(0));
    let consumer_received = Arc::new(Mutex::new(Vec::<u64>::new()));

    struct CrashingSensor {
        ticks: Arc<AtomicU64>,
        topic: Topic<u64>,
    }

    impl Node for CrashingSensor {
        fn name(&self) -> &str {
            "crash_sensor"
        }
        fn tick(&mut self) {
            let t = self.ticks.fetch_add(1, Ordering::SeqCst) + 1;
            if t > 10 {
                panic!("sensor hardware fault at tick {}", t);
            }
            self.topic.send(t);
        }
    }

    struct Consumer {
        topic: Topic<u64>,
        received: Arc<Mutex<Vec<u64>>>,
        none_count: u64,
    }

    impl Node for Consumer {
        fn name(&self) -> &str {
            "consumer"
        }
        fn tick(&mut self) {
            match self.topic.recv() {
                Some(val) => {
                    self.received.lock().unwrap().push(val);
                    self.none_count = 0;
                }
                None => {
                    self.none_count += 1;
                }
            }
        }
    }

    let pub_topic = Topic::<u64>::new(&topic_name).expect("create pub topic");
    let sub_topic = Topic::<u64>::new(&topic_name).expect("create sub topic");

    let mut scheduler = Scheduler::new().tick_rate(100_u64.hz());

    let _ = scheduler
        .add(CrashingSensor {
            ticks: sensor_ticks.clone(),
            topic: pub_topic,
        })
        .order(0)
        .failure_policy(FailurePolicy::Ignore)
        .build();

    scheduler
        .add(Consumer {
            topic: sub_topic,
            received: consumer_received.clone(),
            none_count: 0,
        })
        .order(1)
        .build()
        .unwrap();

    // Run for 300ms — sensor publishes for 10 ticks then crashes.
    // Consumer should receive 10 messages then get None for the rest.
    let _ = scheduler.run_for(300_u64.ms());

    let received = consumer_received.lock().unwrap();

    // Consumer must have received some messages (from the 10 ticks before crash)
    assert!(
        !received.is_empty(),
        "Consumer should have received messages before sensor crashed"
    );

    // All received values should be valid (1..=10)
    for &val in received.iter() {
        assert!(
            (1..=10).contains(&val),
            "Consumer received unexpected value {}: should be in range 1..=10",
            val
        );
    }

    // The received count should be bounded by what the sensor sent (at most 10)
    assert!(
        received.len() <= 10,
        "Consumer received {} messages but sensor only sent 10 before crashing",
        received.len()
    );
}
