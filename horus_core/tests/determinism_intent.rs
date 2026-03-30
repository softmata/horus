#![allow(dead_code)]
//! Level 7 Intent Tests — Determinism
//!
//! These tests verify **behavioral intent**, not implementation details.
//! Each test documents WHY a behavior matters and what user-visible guarantee
//! it protects.
//!
//! Covers:
//! - Same inputs produce same outputs (reproducibility)
//! - tick_once() produces exactly one tick per node
//! - Node tick order is stable across runs
//! - Topic data integrity under rapid tick_once()

use horus_core::communication::Topic;
use horus_core::core::{DurationExt, Node};
use horus_core::scheduling::Scheduler;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::{Arc, Mutex};

mod common;
use common::cleanup_stale_shm;

// ============================================================================
// Test 1: Same nodes with same inputs produce same outputs
// ============================================================================

/// INTENT: "Same nodes with same inputs produce same outputs."
///
/// Robotics guarantee: a deterministic scheduler must be reproducible. Two
/// identical systems configured identically and run for the same duration
/// must produce approximately the same number of ticks. Without this,
/// simulation replay, unit testing, and hardware-in-the-loop validation
/// are unreliable.
#[test]
fn test_determinism_intent_same_input_same_output() {
    cleanup_stale_shm();

    struct CounterNode {
        name: String,
        count: Arc<AtomicU64>,
    }

    impl Node for CounterNode {
        fn name(&self) -> &str {
            Box::leak(self.name.clone().into_boxed_str())
        }
        fn tick(&mut self) {
            self.count.fetch_add(1, Ordering::SeqCst);
        }
    }

    // Run A
    let count_a = Arc::new(AtomicU64::new(0));
    let mut scheduler_a = Scheduler::new().tick_rate(100_u64.hz());
    scheduler_a
        .add(CounterNode {
            name: "counter_a".to_string(),
            count: count_a.clone(),
        })
        .order(0)
        .build()
        .unwrap();
    scheduler_a.run_for(500_u64.ms()).unwrap();
    let ticks_a = count_a.load(Ordering::SeqCst);

    cleanup_stale_shm();

    // Run B — identical configuration
    let count_b = Arc::new(AtomicU64::new(0));
    let mut scheduler_b = Scheduler::new().tick_rate(100_u64.hz());
    scheduler_b
        .add(CounterNode {
            name: "counter_b".to_string(),
            count: count_b.clone(),
        })
        .order(0)
        .build()
        .unwrap();
    scheduler_b.run_for(500_u64.ms()).unwrap();
    let ticks_b = count_b.load(Ordering::SeqCst);

    // Both must have ticked at least once
    assert!(
        ticks_a > 0,
        "Scheduler A should have ticked at least once, got 0"
    );
    assert!(
        ticks_b > 0,
        "Scheduler B should have ticked at least once, got 0"
    );

    // Tick counts should be approximately equal (within 20%)
    let max = ticks_a.max(ticks_b) as f64;
    let min = ticks_a.min(ticks_b) as f64;
    let ratio = min / max;
    assert!(
        ratio >= 0.8,
        "Tick counts should be within 20%: A={}, B={}, ratio={:.2}",
        ticks_a,
        ticks_b,
        ratio
    );
}

// ============================================================================
// Test 2: tick_once() always produces exactly one tick per node
// ============================================================================

/// INTENT: "tick_once() always produces exactly one tick per node."
///
/// Robotics guarantee: simulation loops call tick_once() to advance one
/// physics step. If a call produces zero ticks, the simulation freezes.
/// If it produces two ticks, the physics double-steps and state diverges
/// from reality. Exactly one tick per call is the contract.
#[test]
fn test_determinism_intent_tick_once_is_deterministic() {
    cleanup_stale_shm();

    let count_x = Arc::new(AtomicU64::new(0));
    let count_y = Arc::new(AtomicU64::new(0));

    struct TickCounter {
        name: String,
        count: Arc<AtomicU64>,
    }

    impl Node for TickCounter {
        fn name(&self) -> &str {
            Box::leak(self.name.clone().into_boxed_str())
        }
        fn tick(&mut self) {
            self.count.fetch_add(1, Ordering::SeqCst);
        }
    }

    let mut scheduler = Scheduler::new().tick_rate(100_u64.hz());

    scheduler
        .add(TickCounter {
            name: "tick_x".to_string(),
            count: count_x.clone(),
        })
        .order(0)
        .build()
        .unwrap();

    scheduler
        .add(TickCounter {
            name: "tick_y".to_string(),
            count: count_y.clone(),
        })
        .order(1)
        .build()
        .unwrap();

    // Call tick_once() exactly 10 times
    for _ in 0..10 {
        scheduler.tick_once().unwrap();
    }

    let x = count_x.load(Ordering::SeqCst);
    let y = count_y.load(Ordering::SeqCst);

    assert_eq!(
        x, 10,
        "Node X should have ticked exactly 10 times (once per tick_once call), got {}",
        x
    );
    assert_eq!(
        y, 10,
        "Node Y should have ticked exactly 10 times (once per tick_once call), got {}",
        y
    );
}

// ============================================================================
// Test 3: Node tick order is stable across multiple runs
// ============================================================================

/// INTENT: "Node tick order is stable across multiple runs."
///
/// Robotics guarantee: if node A reads from a sensor and node B fuses that
/// data, B must always tick after A. If the order varies between runs, the
/// system is non-deterministic — test results depend on luck, and production
/// behavior is unpredictable. The order must be identical every time.
#[test]
fn test_determinism_intent_ordering_stable_across_runs() {
    cleanup_stale_shm();

    struct OrderRecorder {
        name: String,
        log: Arc<Mutex<Vec<String>>>,
    }

    impl Node for OrderRecorder {
        fn name(&self) -> &str {
            Box::leak(self.name.clone().into_boxed_str())
        }
        fn tick(&mut self) {
            self.log.lock().unwrap().push(self.name.clone());
        }
    }

    // Run 1: record tick order
    let log_1 = Arc::new(Mutex::new(Vec::<String>::new()));
    let mut sched_1 = Scheduler::new().tick_rate(100_u64.hz());
    sched_1
        .add(OrderRecorder {
            name: "sensor".to_string(),
            log: log_1.clone(),
        })
        .order(1)
        .build()
        .unwrap();
    sched_1
        .add(OrderRecorder {
            name: "planner".to_string(),
            log: log_1.clone(),
        })
        .order(3)
        .build()
        .unwrap();
    sched_1
        .add(OrderRecorder {
            name: "fusion".to_string(),
            log: log_1.clone(),
        })
        .order(2)
        .build()
        .unwrap();

    for _ in 0..5 {
        sched_1.tick_once().unwrap();
    }
    let order_1: Vec<String> = log_1.lock().unwrap().clone();

    cleanup_stale_shm();

    // Run 2: identical configuration, record tick order again
    let log_2 = Arc::new(Mutex::new(Vec::<String>::new()));
    let mut sched_2 = Scheduler::new().tick_rate(100_u64.hz());
    sched_2
        .add(OrderRecorder {
            name: "sensor".to_string(),
            log: log_2.clone(),
        })
        .order(1)
        .build()
        .unwrap();
    sched_2
        .add(OrderRecorder {
            name: "planner".to_string(),
            log: log_2.clone(),
        })
        .order(3)
        .build()
        .unwrap();
    sched_2
        .add(OrderRecorder {
            name: "fusion".to_string(),
            log: log_2.clone(),
        })
        .order(2)
        .build()
        .unwrap();

    for _ in 0..5 {
        sched_2.tick_once().unwrap();
    }
    let order_2: Vec<String> = log_2.lock().unwrap().clone();

    // Both runs should produce the exact same tick ordering
    assert_eq!(
        order_1.len(),
        order_2.len(),
        "Both runs should produce the same number of ticks: run1={}, run2={}",
        order_1.len(),
        order_2.len()
    );

    assert_eq!(
        order_1, order_2,
        "Tick order must be identical across runs.\nRun 1: {:?}\nRun 2: {:?}",
        order_1, order_2
    );

    // Verify the expected order within each tick: sensor(1), fusion(2), planner(3)
    for chunk in order_1.chunks(3) {
        assert_eq!(
            chunk,
            &["sensor", "fusion", "planner"],
            "Expected order [sensor, fusion, planner] per tick, got {:?}",
            chunk
        );
    }
}

// ============================================================================
// Test 4: Topic data is not corrupted under rapid tick_once
// ============================================================================

/// INTENT: "Topic data is not corrupted under rapid tick_once."
///
/// Robotics guarantee: a producer node writes a unique value per tick, and
/// a consumer reads it. Under rapid deterministic ticking, there must be
/// no data corruption, no interleaving of partial writes, and no reordering.
/// Values seen by the consumer must be monotonically increasing, proving
/// the shared memory transport preserves data integrity.
#[test]
fn test_determinism_intent_no_data_race_in_topic() {
    cleanup_stale_shm();

    let topic_name = common::unique("determinism.norace");
    let received = Arc::new(Mutex::new(Vec::<u64>::new()));

    struct MonotonicProducer {
        counter: u64,
        topic: Topic<u64>,
    }

    impl Node for MonotonicProducer {
        fn name(&self) -> &str {
            "mono_producer"
        }
        fn tick(&mut self) {
            self.counter += 1;
            self.topic.send(self.counter);
        }
    }

    struct MonotonicConsumer {
        topic: Topic<u64>,
        received: Arc<Mutex<Vec<u64>>>,
    }

    impl Node for MonotonicConsumer {
        fn name(&self) -> &str {
            "mono_consumer"
        }
        fn tick(&mut self) {
            while let Some(val) = self.topic.recv() {
                self.received.lock().unwrap().push(val);
            }
        }
    }

    // Create topics on the main thread before scheduler use
    let pub_topic = Topic::<u64>::new(&topic_name).expect("create publisher topic");
    let sub_topic = Topic::<u64>::new(&topic_name).expect("create subscriber topic");

    let mut scheduler = Scheduler::new().tick_rate(100_u64.hz());

    // Producer ticks first (order 0), consumer second (order 1)
    scheduler
        .add(MonotonicProducer {
            counter: 0,
            topic: pub_topic,
        })
        .order(0)
        .build()
        .unwrap();

    scheduler
        .add(MonotonicConsumer {
            topic: sub_topic,
            received: received.clone(),
        })
        .order(1)
        .build()
        .unwrap();

    // Run 100 tick_once() calls — much more aggressive than typical tests
    for _ in 0..100 {
        scheduler.tick_once().unwrap();
    }

    let values = received.lock().unwrap();

    // Must have received a substantial number of values
    assert!(
        values.len() >= 50,
        "Consumer should have received at least 50 values from 100 ticks, got {}",
        values.len()
    );

    // All received values must be monotonically increasing (no corruption/reordering)
    for window in values.windows(2) {
        assert!(
            window[1] > window[0],
            "Values must be monotonically increasing: got {} after {} \
             (indicates data corruption or interleaving)",
            window[1],
            window[0]
        );
    }

    // Values must be from the expected range [1, 100]
    for &v in values.iter() {
        assert!(
            (1..=100).contains(&v),
            "Value {} is outside expected range [1, 100] (indicates corruption)",
            v
        );
    }
}
