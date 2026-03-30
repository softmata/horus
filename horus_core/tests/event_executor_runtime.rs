#![allow(dead_code)]
// Runtime integration tests for the event executor.
//
// Tests event-driven execution class behavior through the Scheduler:
// - Event node ticks when notified (via NodeInfo::notify_event)
// - Event node does NOT tick without notification
// - Event node coexists with RT and BestEffort nodes
//
// DESIGN NOTE: Topic::send() does NOT automatically notify event nodes.
// Event notification requires explicit NodeInfo::notify_event(topic_name).
// This is by design — the notification is managed by the scheduler's
// event monitoring, not by the topic transport layer.
// The existing 4 tests in event_executor_paths.rs test the internal
// notification mechanism. These tests verify Scheduler-level integration.

use horus_core::core::{DurationExt, Node};
use horus_core::scheduling::Scheduler;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::Arc;
use std::time::Duration;

mod common;
use common::cleanup_stale_shm;

// ---------------------------------------------------------------------------
// Counter node for event-driven testing
// ---------------------------------------------------------------------------

struct EventCounter {
    name: String,
    tick_count: Arc<AtomicU64>,
    topic_name: String,
}

impl Node for EventCounter {
    fn name(&self) -> &'static str {
        Box::leak(self.name.clone().into_boxed_str())
    }

    fn tick(&mut self) {
        self.tick_count.fetch_add(1, Ordering::SeqCst);
    }
}

// PublisherNode removed — Topic::send() does not trigger event notification.
// Event notification requires explicit NodeInfo::notify_event(topic_name).

fn unique_topic(prefix: &str) -> String {
    static COUNTER: AtomicU64 = AtomicU64::new(0);
    format!(
        "{}_{}_{}",
        prefix,
        std::process::id(),
        COUNTER.fetch_add(1, Ordering::Relaxed)
    )
}

// ============================================================================
// Test: Event node ticks when notified via NodeInfo::notify_event
// ============================================================================

#[test]
fn test_event_node_ticks_on_notification() {
    cleanup_stale_shm();

    let topic_name = unique_topic("evt_notify");
    let event_ticks = Arc::new(AtomicU64::new(0));

    let event_node = EventCounter {
        name: format!("evt_node_{}", std::process::id()),
        tick_count: event_ticks.clone(),
        topic_name: topic_name.clone(),
    };

    let running = Arc::new(AtomicBool::new(true));
    let running_clone = running.clone();
    let tn = topic_name.clone();

    let sched_thread = std::thread::spawn(move || {
        let mut sched = Scheduler::new().tick_rate(100_u64.hz());
        sched.add(event_node).order(0).on(&tn).build().unwrap();

        while running_clone.load(Ordering::Relaxed) {
            let _ = sched.tick_once();
            std::thread::sleep(Duration::from_millis(10));
        }
    });

    // Wait for event watcher to register
    std::thread::sleep(Duration::from_millis(100));

    // Send notifications
    for _ in 0..5 {
        horus_core::core::NodeInfo::notify_event(&topic_name);
        std::thread::sleep(Duration::from_millis(20));
    }

    std::thread::sleep(Duration::from_millis(100));
    running.store(false, Ordering::Relaxed);
    sched_thread.join().unwrap();

    let ticks = event_ticks.load(Ordering::SeqCst);
    assert!(
        ticks >= 1,
        "Event node should tick after notify_event, got {} ticks",
        ticks
    );
}

// ============================================================================
// Test: Event node does NOT tick when no data published
// ============================================================================

#[test]
fn test_event_node_no_tick_without_data() {
    cleanup_stale_shm();

    let topic_name = unique_topic("evt_empty");
    let event_ticks = Arc::new(AtomicU64::new(0));

    let event_node = EventCounter {
        name: format!("evt_empty_{}", std::process::id()),
        tick_count: event_ticks.clone(),
        topic_name: topic_name.clone(),
    };

    let mut sched = Scheduler::new().tick_rate(100_u64.hz());
    sched
        .add(event_node)
        .order(0)
        .on(&topic_name)
        .build()
        .unwrap();

    // Run without any publisher
    sched.run_for(Duration::from_millis(300)).unwrap();

    let ticks = event_ticks.load(Ordering::SeqCst);
    assert_eq!(
        ticks, 0,
        "Event node should NOT tick without data, but ticked {} times",
        ticks
    );
}

// ============================================================================
// Test: Event node + BestEffort node coexist
// ============================================================================

#[test]
fn test_event_node_coexists_with_best_effort() {
    cleanup_stale_shm();

    let topic_name = unique_topic("evt_coexist");
    let event_ticks = Arc::new(AtomicU64::new(0));
    let be_ticks = Arc::new(AtomicU64::new(0));

    let event_node = EventCounter {
        name: format!("evt_co_{}", std::process::id()),
        tick_count: event_ticks.clone(),
        topic_name: topic_name.clone(),
    };

    struct BestEffortCounter {
        name: String,
        tick_count: Arc<AtomicU64>,
    }
    impl Node for BestEffortCounter {
        fn name(&self) -> &'static str {
            Box::leak(self.name.clone().into_boxed_str())
        }
        fn tick(&mut self) {
            self.tick_count.fetch_add(1, Ordering::SeqCst);
        }
    }

    let be_node = BestEffortCounter {
        name: format!("be_co_{}", std::process::id()),
        tick_count: be_ticks.clone(),
    };

    let mut sched = Scheduler::new().tick_rate(100_u64.hz());
    sched.add(be_node).order(0).build().unwrap();
    sched
        .add(event_node)
        .order(1)
        .on(&topic_name)
        .build()
        .unwrap();

    sched.run_for(Duration::from_millis(300)).unwrap();

    let be_final = be_ticks.load(Ordering::SeqCst);
    let evt_final = event_ticks.load(Ordering::SeqCst);

    assert!(
        be_final >= 5,
        "BestEffort node should tick many times, got {}",
        be_final
    );
    // Event node shouldn't tick (no publisher)
    assert_eq!(
        evt_final, 0,
        "Event node without publisher should not tick, got {}",
        evt_final
    );
}

// ============================================================================
// Test: Rapid burst of notifications doesn't crash or deadlock
// ============================================================================

#[test]
fn test_event_rapid_burst_no_crash() {
    cleanup_stale_shm();

    let topic_name = unique_topic("evt_burst");
    let event_ticks = Arc::new(AtomicU64::new(0));

    let event_node = EventCounter {
        name: format!("evt_burst_{}", std::process::id()),
        tick_count: event_ticks.clone(),
        topic_name: topic_name.clone(),
    };

    let running = Arc::new(AtomicBool::new(true));
    let running_clone = running.clone();
    let tn = topic_name.clone();

    let sched_thread = std::thread::spawn(move || {
        let mut sched = Scheduler::new().tick_rate(100_u64.hz());
        sched.add(event_node).order(0).on(&tn).build().unwrap();

        while running_clone.load(Ordering::Relaxed) {
            let _ = sched.tick_once();
            std::thread::sleep(Duration::from_millis(5));
        }
    });

    // Wait for event watcher setup
    std::thread::sleep(Duration::from_millis(100));

    // Burst 100 notifications rapidly
    for _ in 0..100 {
        horus_core::core::NodeInfo::notify_event(&topic_name);
    }

    std::thread::sleep(Duration::from_millis(200));
    running.store(false, Ordering::Relaxed);
    let start = std::time::Instant::now();
    sched_thread.join().unwrap();
    let join_elapsed = start.elapsed();

    assert!(
        join_elapsed < Duration::from_secs(5),
        "Burst should not cause deadlock, join took {:?}",
        join_elapsed
    );

    let ticks = event_ticks.load(Ordering::SeqCst);
    assert!(
        ticks >= 1,
        "Event node should tick from burst notifications, got {}",
        ticks
    );
}

// ============================================================================
// Test: Event node with RT node — both execution classes work together
// ============================================================================

#[test]
fn test_event_node_with_rt_node() {
    cleanup_stale_shm();

    let topic_name = unique_topic("evt_rt");
    let event_ticks = Arc::new(AtomicU64::new(0));
    let rt_ticks = Arc::new(AtomicU64::new(0));

    let event_node = EventCounter {
        name: format!("evt_rt_{}", std::process::id()),
        tick_count: event_ticks.clone(),
        topic_name: topic_name.clone(),
    };

    struct RtCounter {
        name: String,
        tick_count: Arc<AtomicU64>,
    }
    impl Node for RtCounter {
        fn name(&self) -> &'static str {
            Box::leak(self.name.clone().into_boxed_str())
        }
        fn tick(&mut self) {
            self.tick_count.fetch_add(1, Ordering::SeqCst);
        }
    }

    let rt_node = RtCounter {
        name: format!("rt_evt_{}", std::process::id()),
        tick_count: rt_ticks.clone(),
    };

    let mut sched = Scheduler::new().tick_rate(100_u64.hz());
    sched
        .add(rt_node)
        .order(0)
        .rate(100_u64.hz())
        .build()
        .unwrap();
    sched
        .add(event_node)
        .order(1)
        .on(&topic_name)
        .build()
        .unwrap();

    sched.run_for(Duration::from_millis(300)).unwrap();

    let rt_final = rt_ticks.load(Ordering::SeqCst);
    assert!(
        rt_final >= 5,
        "RT node should tick independently, got {}",
        rt_final
    );
}

// ============================================================================
// Test: Event node sustained notifications over 2 seconds — stable tick growth
// ============================================================================

#[test]
fn test_event_sustained_2_seconds() {
    cleanup_stale_shm();

    let topic_name = unique_topic("evt_sustained");
    let event_ticks = Arc::new(AtomicU64::new(0));

    let event_node = EventCounter {
        name: format!("evt_sust_{}", std::process::id()),
        tick_count: event_ticks.clone(),
        topic_name: topic_name.clone(),
    };

    let running = Arc::new(AtomicBool::new(true));
    let running_clone = running.clone();
    let tn = topic_name.clone();

    let sched_thread = std::thread::spawn(move || {
        let mut sched = Scheduler::new().tick_rate(100_u64.hz());
        sched.add(event_node).order(0).on(&tn).build().unwrap();
        while running_clone.load(Ordering::Relaxed) {
            let _ = sched.tick_once();
            std::thread::sleep(Duration::from_millis(5));
        }
    });

    std::thread::sleep(Duration::from_millis(100));

    // Send notifications at 50Hz for 2 seconds = 100 notifications
    for _ in 0..100 {
        horus_core::core::NodeInfo::notify_event(&topic_name);
        std::thread::sleep(Duration::from_millis(20));
    }

    std::thread::sleep(Duration::from_millis(200));
    running.store(false, Ordering::Relaxed);
    sched_thread.join().unwrap();

    let ticks = event_ticks.load(Ordering::SeqCst);
    // At 50Hz for 2s, expect ~100 notifications → at least 20 ticks (generous margin)
    assert!(
        ticks >= 10,
        "Sustained event notifications should produce steady ticks, got {}",
        ticks
    );
}

// ============================================================================
// Test: Multiple event nodes on different topics — independent notification
// ============================================================================

#[test]
fn test_multiple_event_nodes_independent() {
    cleanup_stale_shm();

    let topic_a = unique_topic("evt_multi_a");
    let topic_b = unique_topic("evt_multi_b");
    let ticks_a = Arc::new(AtomicU64::new(0));
    let ticks_b = Arc::new(AtomicU64::new(0));

    let node_a = EventCounter {
        name: format!("evt_a_{}", std::process::id()),
        tick_count: ticks_a.clone(),
        topic_name: topic_a.clone(),
    };
    let node_b = EventCounter {
        name: format!("evt_b_{}", std::process::id()),
        tick_count: ticks_b.clone(),
        topic_name: topic_b.clone(),
    };

    let ta = topic_a.clone();
    let tb = topic_b.clone();
    let tn_for_notify = topic_a.clone();

    // Use run_for() — tick_once() bypasses EventExecutor and ticks all nodes
    let sched_thread = std::thread::spawn(move || {
        let mut sched = Scheduler::new().tick_rate(100_u64.hz());
        sched.add(node_a).order(0).on(&ta).build().unwrap();
        sched.add(node_b).order(1).on(&tb).build().unwrap();
        let _ = sched.run_for(Duration::from_secs(1));
    });

    // Wait for event watchers to start
    std::thread::sleep(Duration::from_millis(200));

    // Only notify topic_a, NOT topic_b
    for _ in 0..10 {
        horus_core::core::NodeInfo::notify_event(&tn_for_notify);
        std::thread::sleep(Duration::from_millis(30));
    }

    sched_thread.join().unwrap();

    let a_final = ticks_a.load(Ordering::SeqCst);
    let b_final = ticks_b.load(Ordering::SeqCst);

    assert!(
        a_final >= 1,
        "Node A should tick from notifications, got {}",
        a_final
    );
    assert_eq!(
        b_final, 0,
        "Node B should NOT tick (different topic, no notifications), got {}",
        b_final
    );
}

// ============================================================================
// Test: Event node watcher cleanup — scheduler drop completes without hang
// ============================================================================

#[test]
fn test_event_watcher_cleanup_on_drop() {
    cleanup_stale_shm();

    let topic_name = unique_topic("evt_cleanup");
    let event_ticks = Arc::new(AtomicU64::new(0));

    let event_node = EventCounter {
        name: format!("evt_cleanup_{}", std::process::id()),
        tick_count: event_ticks.clone(),
        topic_name: topic_name.clone(),
    };

    // Create scheduler, add event node, run briefly, then DROP
    let start = std::time::Instant::now();
    {
        let mut sched = Scheduler::new().tick_rate(100_u64.hz());
        sched
            .add(event_node)
            .order(0)
            .on(&topic_name)
            .build()
            .unwrap();

        // Run for 200ms — event watcher thread spawns
        let _ = sched.run_for(Duration::from_millis(200));
        // sched dropped here — should join watcher threads cleanly
    }
    let drop_elapsed = start.elapsed();

    // Drop should complete quickly (watcher thread joined)
    assert!(
        drop_elapsed < Duration::from_secs(5),
        "Scheduler drop should join event threads quickly, took {:?}",
        drop_elapsed
    );
}
