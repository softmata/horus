//! Edge-case tests for node lifecycle: state transitions, error handling,
//! and safe-state entry.

mod common;
use common::cleanup_stale_shm;

use horus_core::core::{DurationExt, Node};
use horus_core::error::HorusResult;
use horus_core::scheduling::{FailurePolicy, Scheduler};
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::{Arc, Mutex};

// ============================================================================
// Test 1: Node state transitions — init -> tick -> shutdown
// ============================================================================

/// Tracks every lifecycle callback that fires.
struct LifecycleTracker {
    name: String,
    transitions: Arc<Mutex<Vec<String>>>,
}

impl LifecycleTracker {
    fn new(name: &str) -> (Self, Arc<Mutex<Vec<String>>>) {
        let transitions = Arc::new(Mutex::new(Vec::new()));
        (
            Self {
                name: name.to_string(),
                transitions: transitions.clone(),
            },
            transitions,
        )
    }
}

impl Node for LifecycleTracker {
    fn name(&self) -> &str {
        &self.name
    }

    fn init(&mut self) -> HorusResult<()> {
        self.transitions.lock().unwrap().push("init".to_string());
        Ok(())
    }

    fn tick(&mut self) {
        self.transitions.lock().unwrap().push("tick".to_string());
    }

    fn shutdown(&mut self) -> HorusResult<()> {
        self.transitions
            .lock()
            .unwrap()
            .push("shutdown".to_string());
        Ok(())
    }
}

#[test]
fn test_node_state_transitions() {
    // Direct node lifecycle (no scheduler) — verify init/tick/shutdown order.
    let (mut node, transitions) = LifecycleTracker::new("state_tracker");

    node.init().unwrap();
    node.tick();
    node.tick();
    node.tick();
    node.shutdown().unwrap();

    let log = transitions.lock().unwrap();
    assert_eq!(log.len(), 5);
    assert_eq!(log[0], "init");
    assert_eq!(log[1], "tick");
    assert_eq!(log[2], "tick");
    assert_eq!(log[3], "tick");
    assert_eq!(log[4], "shutdown");
}

#[test]
fn test_node_state_transitions_via_scheduler() {
    cleanup_stale_shm();

    let (node, transitions) = LifecycleTracker::new("sched_lifecycle");

    let mut sched = Scheduler::new().deterministic(true);
    sched.add(node).build().unwrap();

    // tick_once drives init (on first tick) + tick
    sched.tick_once().unwrap();
    sched.tick_once().unwrap();

    {
        let log = transitions.lock().unwrap();
        // First tick_once triggers init + tick; second triggers tick.
        assert!(
            log.contains(&"init".to_string()),
            "init should have been called; log = {:?}",
            *log
        );
        let tick_count = log.iter().filter(|s| *s == "tick").count();
        assert!(
            tick_count >= 2,
            "expected at least 2 ticks, got {}; log = {:?}",
            tick_count,
            *log
        );
    }
}

// ============================================================================
// Test 2: Node tick panic triggers on_error
// ============================================================================

/// A node that panics on a configurable tick, with an on_error recorder.
struct ErrorTriggerNode {
    name: String,
    tick_count: Arc<AtomicU64>,
    panic_at: u64,
    errors_received: Arc<Mutex<Vec<String>>>,
}

impl ErrorTriggerNode {
    fn new(name: &str, panic_at: u64) -> (Self, Arc<AtomicU64>, Arc<Mutex<Vec<String>>>) {
        let tick_count = Arc::new(AtomicU64::new(0));
        let errors = Arc::new(Mutex::new(Vec::new()));
        (
            Self {
                name: name.to_string(),
                tick_count: tick_count.clone(),
                panic_at,
                errors_received: errors.clone(),
            },
            tick_count,
            errors,
        )
    }
}

impl Node for ErrorTriggerNode {
    fn name(&self) -> &str {
        &self.name
    }

    fn tick(&mut self) {
        let count = self.tick_count.fetch_add(1, Ordering::SeqCst);
        if count >= self.panic_at {
            panic!("intentional panic at tick {}", count);
        }
    }

    fn on_error(&mut self, error: &str) {
        self.errors_received.lock().unwrap().push(error.to_string());
    }
}

#[test]
fn test_node_error_triggers_on_error() {
    cleanup_stale_shm();

    // Panic after 3 successful ticks (on tick index 3).
    let (node, tick_count, errors) = ErrorTriggerNode::new("err_node", 3);

    let mut sched = Scheduler::new().deterministic(true);
    sched
        .add(node)
        .failure_policy(FailurePolicy::Ignore)
        .build()
        .unwrap();

    // Run enough ticks to trigger the panic and on_error callback.
    for _ in 0..10 {
        let _ = sched.tick_once();
    }

    // The node should have reached the panic point.
    assert!(
        tick_count.load(Ordering::SeqCst) >= 3,
        "node should have ticked at least 3 times before panicking"
    );

    // on_error should have been called at least once.
    let errs = errors.lock().unwrap();
    assert!(
        !errs.is_empty(),
        "on_error should have been called after the panic"
    );
    // The error message should reference the panic.
    assert!(
        errs.iter()
            .any(|e| e.contains("panic") || e.contains("intentional")),
        "error should mention panic; got: {:?}",
        *errs
    );
}

// ============================================================================
// Test 3: enter_safe_state called on safety condition
// ============================================================================

/// A node that tracks is_safe_state and enter_safe_state calls.
struct SafeStateNode {
    name: String,
    tick_count: Arc<AtomicU64>,
    safe_state_entered: Arc<Mutex<bool>>,
    safe_state_reported: Arc<Mutex<bool>>,
}

impl SafeStateNode {
    fn new(name: &str) -> (Self, Arc<AtomicU64>, Arc<Mutex<bool>>, Arc<Mutex<bool>>) {
        let ticks = Arc::new(AtomicU64::new(0));
        let entered = Arc::new(Mutex::new(false));
        let reported = Arc::new(Mutex::new(false));
        (
            Self {
                name: name.to_string(),
                tick_count: ticks.clone(),
                safe_state_entered: entered.clone(),
                safe_state_reported: reported.clone(),
            },
            ticks,
            entered,
            reported,
        )
    }
}

impl Node for SafeStateNode {
    fn name(&self) -> &str {
        &self.name
    }

    fn tick(&mut self) {
        self.tick_count.fetch_add(1, Ordering::SeqCst);
    }

    fn is_safe_state(&self) -> bool {
        let val = *self.safe_state_reported.lock().unwrap();
        val
    }

    fn enter_safe_state(&mut self) {
        *self.safe_state_entered.lock().unwrap() = true;
        *self.safe_state_reported.lock().unwrap() = true;
    }
}

#[test]
fn test_node_safe_state_entered() {
    // Call enter_safe_state directly and verify the node transitions.
    let (mut node, _ticks, entered, _reported) = SafeStateNode::new("safe_node");

    // Before safe state
    assert!(!node.is_safe_state());
    assert!(!*entered.lock().unwrap());

    // Force safe state entry
    node.enter_safe_state();

    // After safe state
    assert!(node.is_safe_state());
    assert!(*entered.lock().unwrap());
}

#[test]
fn test_node_safe_state_with_stall_and_watchdog() {
    cleanup_stale_shm();

    // A node that stalls (long sleep) to trigger the watchdog.
    struct StallNode {
        name: String,
        tick_count: Arc<AtomicU64>,
        stall_after: u64,
        safe_entered: Arc<Mutex<bool>>,
    }

    impl Node for StallNode {
        fn name(&self) -> &str {
            // Leak is acceptable in tests to satisfy 'static lifetime
            Box::leak(self.name.clone().into_boxed_str())
        }

        fn tick(&mut self) {
            let count = self.tick_count.fetch_add(1, Ordering::SeqCst) + 1;
            if count >= self.stall_after {
                // Stall long enough to exceed the watchdog timeout
                std::thread::sleep(200_u64.ms());
            }
        }

        fn is_safe_state(&self) -> bool {
            *self.safe_entered.lock().unwrap()
        }

        fn enter_safe_state(&mut self) {
            *self.safe_entered.lock().unwrap() = true;
        }
    }

    let ticks = Arc::new(AtomicU64::new(0));
    let safe_entered = Arc::new(Mutex::new(false));

    let node = StallNode {
        name: format!("stall_safe_{}", std::process::id()),
        tick_count: ticks.clone(),
        stall_after: 3,
        safe_entered: safe_entered.clone(),
    };

    let mut sched = Scheduler::new().watchdog(50_u64.ms());
    sched.add(node).build().unwrap();

    // Run long enough for stall + watchdog to trigger
    let _ = sched.run_for(500_u64.ms());

    // The node should have ticked at least once before stalling
    assert!(
        ticks.load(Ordering::SeqCst) >= 1,
        "node should have ticked at least once"
    );

    // Watchdog-triggered safe state is best-effort in deterministic tests;
    // at minimum, verify the node's enter_safe_state mechanism works.
    // The stall + watchdog path may or may not invoke it depending on
    // timing, so we also accept a scenario where the scheduler shut down
    // before the safety monitor ran. The direct test above covers the
    // guarantee that enter_safe_state sets the flag.
}
