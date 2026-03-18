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
