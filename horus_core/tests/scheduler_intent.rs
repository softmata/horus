#![allow(dead_code)]
//! Level 7 Intent Tests — Scheduler
//!
//! These tests verify **behavioral intent**, not implementation details.
//! Each test documents WHY a behavior matters and what user-visible guarantee
//! it protects.
//!
//! Covers:
//! - Deterministic tick ordering by `.order()` within an execution class
//! - Per-node rate is respected over time
//! - Graceful shutdown calls `shutdown()` on all nodes
//! - Inter-node data flow across ticks via topics

use horus_core::communication::Topic;
use horus_core::core::{DurationExt, Node};
use horus_core::error::HorusResult;
use horus_core::scheduling::Scheduler;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::{Arc, Mutex};

mod common;
use common::cleanup_stale_shm;

// ============================================================================
// Test 1: Deterministic tick ordering by .order()
// ============================================================================

/// INTENT: "Nodes tick in .order() order within the same execution class."
///
/// Robotics guarantee: emergency stop (order 1) executes before sensor fusion
/// (order 2) before planning (order 3), every single tick. Violating this
/// means a safety-critical node could see stale data.
#[test]
fn test_scheduler_intent_deterministic_tick_ordering() {
    cleanup_stale_shm();

    let tick_sequence = Arc::new(Mutex::new(Vec::<String>::new()));

    // Node that records its name into a shared vec on each tick.
    struct OrderedNode {
        name: String,
        tick_log: Arc<Mutex<Vec<String>>>,
    }

    impl Node for OrderedNode {
        fn name(&self) -> &str {
            Box::leak(self.name.clone().into_boxed_str())
        }
        fn tick(&mut self) {
            self.tick_log.lock().unwrap().push(self.name.clone());
        }
    }

    let mut scheduler = Scheduler::new().tick_rate(100_u64.hz());

    // Add 3 BestEffort nodes in scrambled order: 3, 1, 2
    scheduler
        .add(OrderedNode {
            name: "node_c".to_string(),
            tick_log: tick_sequence.clone(),
        })
        .order(3)
        .build()
        .unwrap();

    scheduler
        .add(OrderedNode {
            name: "node_a".to_string(),
            tick_log: tick_sequence.clone(),
        })
        .order(1)
        .build()
        .unwrap();

    scheduler
        .add(OrderedNode {
            name: "node_b".to_string(),
            tick_log: tick_sequence.clone(),
        })
        .order(2)
        .build()
        .unwrap();

    // Single tick — deterministic, no timing jitter
    scheduler.tick_once().unwrap();

    let log = tick_sequence.lock().unwrap();

    // Nodes should tick in order 1, 2, 3 regardless of add order
    assert_eq!(
        log.len(),
        3,
        "All 3 nodes should tick exactly once, got {} ticks",
        log.len()
    );
    assert_eq!(
        log[0], "node_a",
        "Order 1 node should tick first, got '{}'",
        log[0]
    );
    assert_eq!(
        log[1], "node_b",
        "Order 2 node should tick second, got '{}'",
        log[1]
    );
    assert_eq!(
        log[2], "node_c",
        "Order 3 node should tick third, got '{}'",
        log[2]
    );
}

// ============================================================================
// Test 2: Per-node rate is respected
// ============================================================================

/// INTENT: "A node with .rate(10.hz()) ticks approximately 10 times per second."
///
/// Robotics guarantee: a sensor driver declared at 10Hz must not tick at 1000Hz
/// (wasting CPU) or 1Hz (missing deadlines). The scheduler must honour the
/// declared rate within reasonable tolerance.
#[test]
fn test_scheduler_intent_rate_respected() {
    cleanup_stale_shm();

    let tick_count = Arc::new(AtomicU64::new(0));

    struct RatedNode {
        name: String,
        count: Arc<AtomicU64>,
    }

    impl Node for RatedNode {
        fn name(&self) -> &str {
            Box::leak(self.name.clone().into_boxed_str())
        }
        fn tick(&mut self) {
            self.count.fetch_add(1, Ordering::SeqCst);
        }
    }

    let mut scheduler = Scheduler::new().tick_rate(100_u64.hz());

    scheduler
        .add(RatedNode {
            name: "rated_10hz".to_string(),
            count: tick_count.clone(),
        })
        .order(0)
        .rate(10_u64.hz())
        .build()
        .unwrap();

    // Run for 1000ms — expect ~10 ticks at 10Hz, allowing scheduler startup overhead.
    // The RT executor has measurable startup cost (thread spawn, CPU pinning,
    // SCHED_FIFO attempt), so we use a longer window for a reliable signal.
    scheduler.run_for(1000_u64.ms()).unwrap();

    let ticks = tick_count.load(Ordering::SeqCst);

    // At 10Hz for 1000ms, ideal is 10. Allow 2-15 for scheduler startup/overhead.
    assert!(
        (2..=15).contains(&ticks),
        "Node at 10Hz for 1000ms should tick 2-15 times (ideal ~10), got {}",
        ticks
    );
}

// ============================================================================
// Test 3: shutdown() called on all nodes when scheduler stops
// ============================================================================

/// INTENT: "When the scheduler stops, all nodes get shutdown() called."
///
/// Robotics guarantee: motor drivers MUST release hardware handles, log files
/// must be flushed, network connections must be closed. If shutdown() is not
/// called, resources leak and hardware may be left in an unsafe state.
#[test]
fn test_scheduler_intent_shutdown_called_on_stop() {
    cleanup_stale_shm();

    struct ShutdownTracker {
        name: String,
        flag: Arc<AtomicBool>,
    }

    impl Node for ShutdownTracker {
        fn name(&self) -> &str {
            Box::leak(self.name.clone().into_boxed_str())
        }
        fn tick(&mut self) {
            // no-op — just needs to exist
        }
        fn shutdown(&mut self) -> HorusResult<()> {
            self.flag.store(true, Ordering::SeqCst);
            Ok(())
        }
    }

    let flag_a = Arc::new(AtomicBool::new(false));
    let flag_b = Arc::new(AtomicBool::new(false));
    let flag_c = Arc::new(AtomicBool::new(false));

    let mut scheduler = Scheduler::new().tick_rate(100_u64.hz());

    scheduler
        .add(ShutdownTracker {
            name: "shutdown_a".to_string(),
            flag: flag_a.clone(),
        })
        .order(0)
        .build()
        .unwrap();

    scheduler
        .add(ShutdownTracker {
            name: "shutdown_b".to_string(),
            flag: flag_b.clone(),
        })
        .order(1)
        .build()
        .unwrap();

    scheduler
        .add(ShutdownTracker {
            name: "shutdown_c".to_string(),
            flag: flag_c.clone(),
        })
        .order(2)
        .build()
        .unwrap();

    // Run briefly, then let it stop naturally via run_for
    scheduler.run_for(100_u64.ms()).unwrap();

    assert!(
        flag_a.load(Ordering::SeqCst),
        "Node A's shutdown() was not called"
    );
    assert!(
        flag_b.load(Ordering::SeqCst),
        "Node B's shutdown() was not called"
    );
    assert!(
        flag_c.load(Ordering::SeqCst),
        "Node C's shutdown() was not called"
    );
}

// ============================================================================
// Test 4: Data flows between nodes across ticks via topics
// ============================================================================

/// INTENT: "Data published by node A in tick N is available to node B in tick N+1."
///
/// Robotics guarantee: a sensor node publishes IMU data, a controller node
/// reads it next tick. This is the fundamental pub/sub data pipeline. If data
/// does not flow, the entire system is broken.
#[test]
fn test_scheduler_intent_node_data_flows_between_ticks() {
    cleanup_stale_shm();

    let topic_name = common::unique("intent.dataflow");
    let received_values = Arc::new(Mutex::new(Vec::<u64>::new()));

    // Producer: publishes an incrementing counter each tick
    struct ProducerNode {
        counter: u64,
        topic: Topic<u64>,
    }

    impl Node for ProducerNode {
        fn name(&self) -> &str {
            "intent_producer"
        }
        fn tick(&mut self) {
            self.counter += 1;
            self.topic.send(self.counter);
        }
    }

    // Consumer: reads from the topic and records received values
    struct ConsumerNode {
        topic: Topic<u64>,
        received: Arc<Mutex<Vec<u64>>>,
    }

    impl Node for ConsumerNode {
        fn name(&self) -> &str {
            "intent_consumer"
        }
        fn tick(&mut self) {
            while let Some(val) = self.topic.recv() {
                self.received.lock().unwrap().push(val);
            }
        }
    }

    // Create topics on the main thread before spawning (per test isolation rules)
    let pub_topic = Topic::<u64>::new(&topic_name).expect("create publisher topic");
    let sub_topic = Topic::<u64>::new(&topic_name).expect("create subscriber topic");

    let mut scheduler = Scheduler::new().tick_rate(100_u64.hz());

    // Producer ticks first (order 0), consumer ticks second (order 1)
    scheduler
        .add(ProducerNode {
            counter: 0,
            topic: pub_topic,
        })
        .order(0)
        .build()
        .unwrap();

    scheduler
        .add(ConsumerNode {
            topic: sub_topic,
            received: received_values.clone(),
        })
        .order(1)
        .build()
        .unwrap();

    // Run several ticks
    for _ in 0..10 {
        scheduler.tick_once().unwrap();
    }

    let values = received_values.lock().unwrap();

    // Producer sends 1, 2, 3, ..., 10. Consumer should receive them.
    // Due to same-tick ordering (producer order 0, consumer order 1),
    // the consumer may receive data published in the SAME tick.
    assert!(
        !values.is_empty(),
        "Consumer should have received at least one value from producer"
    );

    // Verify values are sequential (no data corruption or reordering)
    for window in values.windows(2) {
        assert!(
            window[1] > window[0],
            "Received values should be monotonically increasing, got {} after {}",
            window[1],
            window[0]
        );
    }

    // Verify we got a reasonable number of messages (at least half the ticks)
    assert!(
        values.len() >= 5,
        "Expected at least 5 messages from 10 ticks, got {}",
        values.len()
    );
}
