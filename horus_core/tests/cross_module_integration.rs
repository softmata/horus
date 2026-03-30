//! Cross-module integration tests
//!
//! Verifies that multiple horus_core subsystems (scheduler, topics, nodes,
//! services) compose correctly when used together. Each test exercises a
//! realistic multi-module pipeline.

mod common;
use common::cleanup_stale_shm;

use horus_core::communication::Topic;
use horus_core::core::{DurationExt, Node};
use horus_core::scheduling::Scheduler;
use horus_core::service;
use horus_core::services::{ServiceClient, ServiceServerBuilder};
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::{Arc, Mutex};

/// Local test-only Twist type (mirrors TestTwist layout).
#[repr(C)]
#[derive(Debug, Clone, Copy, Default, serde::Serialize, serde::Deserialize)]
struct TestTwist {
    linear: [f64; 3],
    angular: [f64; 3],
    timestamp_ns: u64,
}

// ============================================================================
// Test 6: Full producer-consumer pipeline via scheduler
// ============================================================================

/// INTENT: "A producer node publishing Twist messages through the scheduler
/// delivers data that a consumer node can read within the same scheduler run."
///
/// This is the fundamental robotics data pipeline: sensor -> processing ->
/// actuator, all orchestrated by the scheduler. If data does not flow between
/// nodes, the robot cannot function.
#[test]
fn test_scheduler_topics_nodes_full_pipeline() {
    cleanup_stale_shm();

    let topic_name = common::unique("cross.pipeline");
    let received_values = Arc::new(Mutex::new(Vec::<[f64; 3]>::new()));

    // Producer: publishes Twist-like data (linear velocity) each tick
    struct TwistProducer {
        counter: u64,
        topic: Topic<TestTwist>,
    }

    impl Node for TwistProducer {
        fn name(&self) -> &str {
            "twist_producer"
        }
        fn tick(&mut self) {
            self.counter += 1;
            let vel = self.counter as f64 * 0.1;
            let twist = TestTwist {
                linear: [vel, 0.0, 0.0],
                angular: [0.0, 0.0, vel * 0.5],
                timestamp_ns: self.counter,
            };
            self.topic.send(twist);
        }
    }

    // Consumer: reads Twist messages and records the linear[0] values
    struct TwistConsumer {
        topic: Topic<TestTwist>,
        received: Arc<Mutex<Vec<[f64; 3]>>>,
    }

    impl Node for TwistConsumer {
        fn name(&self) -> &str {
            "twist_consumer"
        }
        fn tick(&mut self) {
            while let Some(twist) = self.topic.recv() {
                self.received.lock().unwrap().push(twist.linear);
            }
        }
    }

    // Create topics on the main thread (per test isolation rules)
    let pub_topic = Topic::<TestTwist>::new(&topic_name).expect("create pub topic");
    let sub_topic = Topic::<TestTwist>::new(&topic_name).expect("create sub topic");

    let mut scheduler = Scheduler::new().tick_rate(200_u64.hz());

    scheduler
        .add(TwistProducer {
            counter: 0,
            topic: pub_topic,
        })
        .order(0)
        .build()
        .unwrap();

    scheduler
        .add(TwistConsumer {
            topic: sub_topic,
            received: received_values.clone(),
        })
        .order(1)
        .build()
        .unwrap();

    // Run for 200ms — at 200Hz that is ~40 ticks
    scheduler.run_for(200_u64.ms()).unwrap();

    let values = received_values.lock().unwrap();

    // Consumer must have received multiple Twist messages
    assert!(
        values.len() >= 5,
        "Consumer should have received at least 5 Twist messages from 200ms run, got {}",
        values.len()
    );

    // Verify data integrity: each linear[0] should be positive and increasing
    for (i, linear) in values.iter().enumerate() {
        assert!(
            linear[0] > 0.0,
            "Twist {} linear[0] should be positive, got {}",
            i,
            linear[0]
        );
    }

    // Verify monotonic increase (producer increments counter each tick)
    for window in values.windows(2) {
        assert!(
            window[1][0] > window[0][0],
            "Twist linear[0] should be monotonically increasing, got {} after {}",
            window[1][0],
            window[0][0]
        );
    }
}

// ============================================================================
// Test 7: Panicking node does not crash healthy siblings
// ============================================================================

/// INTENT: "A node that panics is isolated — other healthy nodes keep running."
///
/// Robotics guarantee: if a non-critical diagnostics node crashes, the motor
/// controller and safety monitor must keep running. The scheduler uses
/// catch_unwind to isolate panicking nodes.
#[test]
fn test_node_error_does_not_crash_other_nodes() {
    cleanup_stale_shm();

    let healthy_a_ticks = Arc::new(AtomicU64::new(0));
    let healthy_b_ticks = Arc::new(AtomicU64::new(0));
    let panicker_ticks = Arc::new(AtomicU64::new(0));

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

    struct PanickingNode {
        ticks: Arc<AtomicU64>,
        panic_after: u64,
    }

    impl Node for PanickingNode {
        fn name(&self) -> &str {
            "panicker"
        }
        fn tick(&mut self) {
            let t = self.ticks.fetch_add(1, Ordering::SeqCst) + 1;
            if t >= self.panic_after {
                panic!("intentional panic at tick {}", t);
            }
        }
    }

    let mut scheduler = Scheduler::new().tick_rate(100_u64.hz());

    scheduler
        .add(HealthyNode {
            label: "healthy_a",
            ticks: healthy_a_ticks.clone(),
        })
        .order(0)
        .build()
        .unwrap();

    let _ = scheduler
        .add(PanickingNode {
            ticks: panicker_ticks.clone(),
            panic_after: 5,
        })
        .order(1)
        .failure_policy(horus_core::scheduling::FailurePolicy::Ignore)
        .build();

    scheduler
        .add(HealthyNode {
            label: "healthy_b",
            ticks: healthy_b_ticks.clone(),
        })
        .order(2)
        .build()
        .unwrap();

    // Run for 200ms — panicker dies at tick 5, but healthy nodes keep going
    let _ = scheduler.run_for(200_u64.ms());

    let a_count = healthy_a_ticks.load(Ordering::SeqCst);
    let b_count = healthy_b_ticks.load(Ordering::SeqCst);
    let p_count = panicker_ticks.load(Ordering::SeqCst);

    // Panicker should have ticked at least 5 times before crashing
    assert!(
        p_count >= 5,
        "Panicker should have ticked at least 5 times, got {}",
        p_count
    );

    // Healthy nodes should have ticked well beyond the panicker's crash point.
    // At 100Hz for 200ms, ideal is ~20 ticks. Allow some tolerance.
    assert!(
        a_count > 5,
        "Healthy node A should have ticked more than 5 times (continued after panicker crash), got {}",
        a_count
    );
    assert!(
        b_count > 5,
        "Healthy node B should have ticked more than 5 times (continued after panicker crash), got {}",
        b_count
    );
}

// ============================================================================
// Test 8: Service server + client while scheduler is running
// ============================================================================

// Unique service type for this test — avoids SHM collisions with other tests.
service! { CrossModuleAdd { request { a: i64, b: i64 } response { sum: i64 } } }

/// INTENT: "Services work correctly alongside the scheduler."
///
/// Robotics guarantee: a calibration service or parameter query must respond
/// correctly even while the main control loop is ticking. Services and
/// the scheduler must not deadlock or interfere.
#[test]
fn test_scheduler_with_services_inline() {
    cleanup_stale_shm();

    let shutdown_flag = Arc::new(AtomicBool::new(false));
    let service_result = Arc::new(Mutex::new(None::<i64>));

    // Start the service server (runs in a background thread)
    let _server = ServiceServerBuilder::<CrossModuleAdd>::new()
        .on_request(|req| Ok(CrossModuleAddResponse { sum: req.a + req.b }))
        .build()
        .expect("create service server");

    // Node that just ticks (representing ongoing work)
    struct BusyNode {
        ticks: Arc<AtomicU64>,
    }

    impl Node for BusyNode {
        fn name(&self) -> &str {
            "busy_worker"
        }
        fn tick(&mut self) {
            self.ticks.fetch_add(1, Ordering::SeqCst);
        }
    }

    let worker_ticks = Arc::new(AtomicU64::new(0));

    // Start scheduler in a background thread
    let _shutdown = shutdown_flag.clone();
    let ticks = worker_ticks.clone();
    let sched_handle = std::thread::spawn(move || {
        let mut scheduler = Scheduler::new().tick_rate(100_u64.hz());
        scheduler
            .add(BusyNode {
                ticks: ticks.clone(),
            })
            .order(0)
            .build()
            .unwrap();

        // Run until shutdown flag is set (max 500ms as safety timeout)
        scheduler.run_for(500_u64.ms()).unwrap();
    });

    // Give the scheduler a moment to start ticking
    std::thread::sleep(std::time::Duration::from_millis(50));

    // Call the service while the scheduler is running
    let mut client = ServiceClient::<CrossModuleAdd>::new().expect("create service client");
    let response = client
        .call(CrossModuleAddRequest { a: 17, b: 25 }, 1_u64.secs())
        .expect("service call should succeed");

    *service_result.lock().unwrap() = Some(response.sum);

    // Signal shutdown and wait for scheduler thread
    shutdown_flag.store(true, Ordering::SeqCst);
    let _ = sched_handle.join();

    // Verify service response
    let result = service_result.lock().unwrap();
    assert_eq!(
        *result,
        Some(42),
        "Service should return 17 + 25 = 42, got {:?}",
        *result
    );

    // Verify the scheduler was running concurrently (worker ticked)
    let final_ticks = worker_ticks.load(Ordering::SeqCst);
    assert!(
        final_ticks > 0,
        "Worker node should have ticked while service was called, got {} ticks",
        final_ticks
    );
}
