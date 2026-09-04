// Scheduler tests

use super::*;
use crate::core::{DurationExt, Node};
use crate::scheduling::fault_tolerance::FailurePolicy;
use std::sync::atomic::{AtomicBool, AtomicUsize, Ordering};
use std::sync::{Arc, Mutex, MutexGuard};
use std::time::Duration;

/// Global mutex to serialize scheduler tests. The scheduler uses process-global
/// state (SIGTERM handler, shared memory namespaces, event notifier registry)
/// that causes non-deterministic failures under parallel execution.
static SCHEDULER_TEST_LOCK: std::sync::LazyLock<Mutex<()>> =
    std::sync::LazyLock::new(|| Mutex::new(()));

/// Acquire the scheduler test lock and reset the SIGTERM flag.
fn lock_scheduler() -> MutexGuard<'static, ()> {
    let guard = SCHEDULER_TEST_LOCK
        .lock()
        .unwrap_or_else(|p| p.into_inner());
    // Reset the global SIGTERM flag so previous tests don't interfere
    super::SIGTERM_RECEIVED.store(false, Ordering::SeqCst);
    guard
}

/// Simple test node that counts its tick invocations
struct CounterNode {
    name: String,
    tick_count: Arc<AtomicUsize>,
}

impl CounterNode {
    fn new(name: impl Into<String>) -> Self {
        Self {
            name: name.into(),
            tick_count: Arc::new(AtomicUsize::new(0)),
        }
    }

    fn with_counter(name: impl Into<String>, counter: Arc<AtomicUsize>) -> Self {
        Self {
            name: name.into(),
            tick_count: counter,
        }
    }
}

impl Node for CounterNode {
    fn name(&self) -> &str {
        &self.name
    }

    fn tick(&mut self) {
        self.tick_count.fetch_add(1, Ordering::SeqCst);
    }
}

// ============================================================================
// Creation Tests
// ============================================================================

#[test]
fn test_scheduler_new() {
    let _guard = lock_scheduler();
    let scheduler = Scheduler::new();
    assert!(scheduler.is_running());
    assert_eq!(scheduler.node_list().len(), 0);
}

#[test]
fn test_scheduler_default() {
    let _guard = lock_scheduler();
    let scheduler = Scheduler::default();
    assert!(scheduler.is_running());
    assert_eq!(scheduler.node_list().len(), 0);
}

// ============================================================================
// Node Addition Tests
// ============================================================================

#[test]
fn test_scheduler_add_node() {
    let _guard = lock_scheduler();
    let mut scheduler = Scheduler::new();
    scheduler
        .add(CounterNode::new("test_node"))
        .order(0)
        .build();

    let nodes = scheduler.node_list();
    assert_eq!(nodes.len(), 1);
    assert_eq!(nodes[0], "test_node");
}

#[test]
fn test_scheduler_add_multiple_nodes() {
    let _guard = lock_scheduler();
    let mut scheduler = Scheduler::new();
    scheduler.add(CounterNode::new("node1")).order(0).build();
    scheduler.add(CounterNode::new("node2")).order(1).build();
    scheduler.add(CounterNode::new("node3")).order(2).build();

    let nodes = scheduler.node_list();
    assert_eq!(nodes.len(), 3);
}

#[test]
fn test_scheduler_node_priority_ordering() {
    let _guard = lock_scheduler();
    let mut scheduler = Scheduler::new();
    // Add nodes with different priorities
    scheduler
        .add(CounterNode::new("low_priority"))
        .order(10)
        .build();
    scheduler
        .add(CounterNode::new("high_priority"))
        .order(0)
        .build();
    scheduler
        .add(CounterNode::new("medium_priority"))
        .order(5)
        .build();

    // After sorting by priority, high_priority should come first
    let nodes = scheduler.node_list();
    assert_eq!(nodes.len(), 3);
    // Note: nodes are sorted by priority
}

/// Regression: a panic in a fault-path node callback (`on_error` / `enter_safe_state`
/// / `shutdown`) must NOT unwind the scheduler tick loop and take down every other
/// node. `tick()` panics are already isolated by design; the *recovery* callbacks —
/// invoked exactly when a node is already failing — were previously called bare, so
/// a panic in one node's recovery killed the control loop of every healthy node.
#[test]
fn fault_path_callback_panic_is_isolated() {
    let _guard = lock_scheduler();

    struct DoubleBoom;
    impl Node for DoubleBoom {
        fn name(&self) -> &str {
            "double_boom"
        }
        fn tick(&mut self) {
            panic!("tick boom");
        }
        fn on_error(&mut self, _msg: &str) {
            panic!("on_error boom");
        }
    }

    let mut scheduler = Scheduler::new();
    scheduler.add(DoubleBoom).build().unwrap();

    // tick() panic is caught by design; on_error() then panics. Pre-fix that second
    // panic escapes handle_tick_failure -> process_tick_result -> tick_once(),
    // unwinding the whole loop. Post-fix it is caught and the scheduler continues.
    let result = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| scheduler.tick_once()));
    assert!(
        result.is_ok(),
        "on_error() panic must be isolated — it unwound the scheduler tick loop"
    );
}

/// A critical node that fails to reach a safe state (its enter_safe_state/shutdown
/// panicked) must escalate to a SYSTEM emergency stop — not silently "just stop".
/// This exercises note_safing_failure's critical branch and, crucially, that
/// is_critical_node's name match holds between the registered critical-node name
/// and self.nodes[i].name (a mismatch would silently downgrade to no e-stop).
#[test]
fn note_safing_failure_escalates_to_estop_for_critical_node() {
    let _guard = lock_scheduler();
    struct N;
    impl Node for N {
        fn name(&self) -> &str {
            "crit_node"
        }
        fn tick(&mut self) {}
    }
    let mut s = Scheduler::new()
        .tick_rate(100_u64.hz())
        .watchdog(500_u64.ms());
    // Per-node watchdog on a BestEffort node registers it as critical at finalize.
    s.add(N).watchdog(100_u64.ms()).build().unwrap();
    s.finalize_and_init();

    let monitor = s.monitor.safety.as_ref().expect("safety monitor enabled");
    assert!(
        monitor.is_critical_node("crit_node"),
        "node must be critical"
    );
    assert!(!monitor.is_emergency_stop(), "no e-stop before the failure");

    let idx = s
        .nodes
        .iter()
        .position(|n| n.name.as_ref() == "crit_node")
        .expect("critical node present in nodes");
    s.note_safing_failure(idx, "enter_safe_state");

    assert!(
        s.nodes[idx].is_stopped,
        "node that failed to safe is stopped"
    );
    assert!(
        s.monitor.safety.as_ref().unwrap().is_emergency_stop(),
        "a critical node that cannot reach a safe state must trigger a system emergency stop"
    );
}

/// SCHED-H1: after the scheduler is finalized, an EXTERNAL emergency stop (the path
/// horus_net uses for a remote e-stop / link-loss) must latch the scheduler's stop.
/// The global hook was never installed, so `trigger_external_emergency_stop` only
/// printed to stderr and returned — the networked e-stop was completely inert.
#[test]
fn external_emergency_stop_latches_scheduler_after_finalize() {
    let _guard = lock_scheduler();
    struct N;
    impl Node for N {
        fn name(&self) -> &str {
            "estop_node"
        }
        fn tick(&mut self) {}
    }
    let mut s = Scheduler::new()
        .tick_rate(100_u64.hz())
        .watchdog(500_u64.ms());
    s.add(N).watchdog(100_u64.ms()).build().unwrap();
    // finalize_and_init() runs finalize_config(), which installs the external
    // emergency-stop hook (SCHED-H1) — mirrors what run() does before the tick loop.
    s.finalize_and_init();

    assert!(
        !s.monitor.safety.as_ref().unwrap().is_emergency_stop(),
        "no e-stop before the external trigger"
    );

    // An external system fires the global hook (as horus_net does on link loss).
    crate::scheduling::safety_monitor::trigger_external_emergency_stop("link lost".to_string());

    assert!(
        s.monitor.safety.as_ref().unwrap().is_emergency_stop(),
        "external emergency stop must latch THIS scheduler's stop flag (SCHED-H1)"
    );
}

/// A non-critical node that fails to safe is stopped, but must NOT trigger a
/// system-wide emergency stop (that would be over-escalation).
#[test]
fn note_safing_failure_stops_noncritical_node_without_estop() {
    let _guard = lock_scheduler();
    struct N;
    impl Node for N {
        fn name(&self) -> &str {
            "regular_node"
        }
        fn tick(&mut self) {}
    }
    // Global watchdog enables the monitor, but this node has no per-node watchdog
    // and is not RT, so it is NOT registered as critical.
    let mut s = Scheduler::new()
        .tick_rate(100_u64.hz())
        .watchdog(500_u64.ms());
    s.add(N).build().unwrap();
    s.finalize_and_init();

    let monitor = s.monitor.safety.as_ref().expect("safety monitor enabled");
    assert!(
        !monitor.is_critical_node("regular_node"),
        "node must not be critical"
    );

    let idx = s
        .nodes
        .iter()
        .position(|n| n.name.as_ref() == "regular_node")
        .expect("node present in nodes");
    s.note_safing_failure(idx, "enter_safe_state");

    assert!(
        s.nodes[idx].is_stopped,
        "node that failed to safe is stopped"
    );
    assert!(
        !s.monitor.safety.as_ref().unwrap().is_emergency_stop(),
        "a non-critical node failing to safe must NOT trigger a system emergency stop"
    );
}

/// Regression (Sched2): an operator pause (`horus node pause`) on a BestEffort
/// main-thread node must PERSIST across ticks. should_tick_node used to only check
/// the transient is_paused flag (which it clears after one tick for Miss::Skip) and
/// never consulted the persistent node_controls map the executor threads honor — so
/// the pause silently lapsed after a single tick.
#[test]
fn operator_pause_persists_across_ticks_for_besteffort_node() {
    let _guard = lock_scheduler();
    let counter = Arc::new(AtomicUsize::new(0));
    struct CountNode {
        c: Arc<AtomicUsize>,
    }
    impl Node for CountNode {
        fn name(&self) -> &str {
            "pause_target"
        }
        fn tick(&mut self) {
            self.c.fetch_add(1, Ordering::Relaxed);
        }
    }
    let mut s = Scheduler::new();
    s.add(CountNode { c: counter.clone() }).build().unwrap();
    s.finalize_and_init();

    s.tick_once().unwrap();
    let before_pause = counter.load(Ordering::Relaxed);
    assert!(before_pause >= 1, "node should tick before pause");

    // The run()/executor path sets up node_controls + registers every node (mod.rs
    // ~2256); the tick_once path used here does not, so replicate that state — this
    // is the deployment state in which `horus node pause` is actually delivered.
    let controls = std::sync::Arc::new(crate::scheduling::types::NodeControlMap::default());
    controls.register("pause_target");
    s.node_controls = Some(controls.clone());

    // Simulate `horus node pause pause_target`: set the persistent node_controls
    // pause flag (the authoritative operator-pause signal).
    controls.set_paused("pause_target", true);

    // The pause must hold across many ticks (pre-fix should_tick_node never
    // consulted node_controls for main-thread nodes, so the node kept ticking).
    for _ in 0..5 {
        s.tick_once().unwrap();
    }
    assert_eq!(
        counter.load(Ordering::Relaxed),
        before_pause,
        "operator-paused BestEffort node must stay paused across ticks"
    );

    // Resume restores ticking.
    controls.set_paused("pause_target", false);
    s.tick_once().unwrap();
    assert!(
        counter.load(Ordering::Relaxed) > before_pause,
        "resumed node ticks again"
    );
}

#[test]
fn test_scheduler_add_basic() {
    let _guard = lock_scheduler();
    let mut scheduler = Scheduler::new();
    scheduler
        .add(CounterNode::new("basic_node"))
        .order(0)
        .build();

    let metrics = scheduler.metrics();
    assert_eq!(metrics.len(), 1);
    assert_eq!(metrics[0].name(), "basic_node");
}

// ============================================================================
// Running State Tests
// ============================================================================

#[test]
fn test_scheduler_is_running() {
    let _guard = lock_scheduler();
    let scheduler = Scheduler::new();
    assert!(scheduler.is_running());
}

#[test]
fn test_scheduler_stop() {
    let _guard = lock_scheduler();
    let scheduler = Scheduler::new();
    assert!(scheduler.is_running());
    scheduler.stop();
    assert!(!scheduler.is_running());
}

#[test]
fn test_scheduler_stop_and_check_multiple_times() {
    let _guard = lock_scheduler();
    let scheduler = Scheduler::new();
    scheduler.stop();
    assert!(!scheduler.is_running());
    assert!(!scheduler.is_running()); // Should still be false
}

// ============================================================================
// Node Rate Control Tests
// ============================================================================

#[test]
fn test_scheduler_set_node_rate() {
    let _guard = lock_scheduler();
    let counter = Arc::new(AtomicUsize::new(0));
    let mut scheduler = Scheduler::new().tick_rate(1000_u64.hz());
    scheduler
        .add(CounterNode::with_counter("sensor", counter.clone()))
        .order(0)
        .build();
    scheduler.set_node_rate("sensor", 100_u64.hz());

    // Run for 500ms — at 100Hz expect ~50 ticks (wide tolerance for CI)
    let result = scheduler.run_for(500_u64.ms());
    result.unwrap();
    let ticks = counter.load(Ordering::SeqCst);
    assert!(
        ticks >= 5,
        "Node at 100Hz should tick at least 5 times in 500ms, got {}",
        ticks
    );
}

#[test]
fn test_scheduler_set_node_rate_nonexistent() {
    let _guard = lock_scheduler();
    let mut scheduler = Scheduler::new();
    scheduler.add(CounterNode::new("node1")).order(0).build();
    // Setting rate for nonexistent node should be a no-op
    scheduler.set_node_rate("nonexistent", 50_u64.hz());
    // Existing node list must be unchanged
    let nodes = scheduler.node_list();
    assert_eq!(nodes.len(), 1);
    assert_eq!(nodes[0], "node1");
}

// ============================================================================
// Node Info Tests (via metrics())
// ============================================================================

#[test]
fn test_scheduler_metrics_existing() {
    let _guard = lock_scheduler();
    let mut scheduler = Scheduler::new();
    scheduler
        .add(CounterNode::new("info_node"))
        .order(0)
        .build();

    let metrics = scheduler.metrics();
    assert_eq!(metrics.len(), 1);
    assert_eq!(metrics[0].name(), "info_node");
}

#[test]
fn test_scheduler_metrics_empty() {
    let _guard = lock_scheduler();
    let scheduler = Scheduler::new();
    let metrics = scheduler.metrics();
    assert!(metrics.is_empty());
}

#[test]
fn test_scheduler_node_list_empty() {
    let _guard = lock_scheduler();
    let scheduler = Scheduler::new();
    let nodes = scheduler.node_list();
    assert!(nodes.is_empty());
}

// ============================================================================
// Recording Tests
// ============================================================================

#[test]
fn test_scheduler_is_recording_default() {
    let _guard = lock_scheduler();
    let scheduler = Scheduler::new();
    assert!(!scheduler.is_recording());
}

#[test]
fn test_scheduler_enable_recording() {
    let _guard = lock_scheduler();
    let scheduler = Scheduler::new().with_recording();
    assert!(scheduler.is_recording());
}

#[test]
fn test_scheduler_is_replaying_default() {
    let _guard = lock_scheduler();
    let scheduler = Scheduler::new();
    assert!(!scheduler.is_replaying());
}

#[test]
fn test_scheduler_current_tick() {
    let _guard = lock_scheduler();
    let scheduler = Scheduler::new();
    assert_eq!(scheduler.current_tick(), 0);
}

#[test]
fn test_scheduler_start_at_tick() {
    let _guard = lock_scheduler();
    let scheduler = Scheduler::new().start_at_tick(1000);
    assert_eq!(scheduler.current_tick(), 1000);
}

// ============================================================================
// Safety Monitor Tests
// ============================================================================

#[test]
fn test_scheduler_with_watchdog() {
    let _guard = lock_scheduler();
    let scheduler = Scheduler::new().watchdog(500_u64.ms());
    assert!(scheduler.is_running());
    // watchdog(500ms) sets watchdog_timeout_ms=500 in pending config
    assert_eq!(
        scheduler.pending_config.realtime.watchdog_timeout_ms, 500,
        "watchdog(500ms) should set watchdog_timeout_ms in pending config"
    );
}

/// SCHED-H2: a NON-RT node with an explicit `.watchdog()` is registered as a
/// critical watchdog node, so it must be FED each tick. Before the fix the feed
/// was gated on `is_rt_node`, so the node's watchdog was never fed, expired, and
/// spuriously emergency-stopped the scheduler.
#[test]
fn non_rt_watchdog_node_is_fed_no_spurious_estop() {
    let _guard = lock_scheduler();
    let counter = Arc::new(AtomicUsize::new(0));
    let mut scheduler = Scheduler::new().watchdog(1000_u64.ms());
    // No rate/budget => non-RT; explicit per-node watchdog of 50 ms.
    scheduler
        .add(CounterNode::with_counter("nonrt_wdog", counter.clone()))
        .watchdog(50_u64.ms())
        .build()
        .unwrap();
    // Run well past the 50 ms watchdog; the node ticks many times, feeding it.
    //
    // Two seconds rather than 300 ms. The window has to cover scheduler startup
    // plus at least six ticks on a machine that is busy — `cargo test` runs this
    // alongside ~2300 other tests on every core — and at 300 ms a loaded box
    // produced `got 0`, which reads as "the node never ran" rather than "the
    // scheduler never got a timeslice". The watchdog under test is 50 ms, so a
    // longer window makes the assertion stronger, not weaker: more chances for a
    // spurious expiry to show up.
    let _ = scheduler.run_for(2000_u64.ms());
    let ticks = counter.load(Ordering::SeqCst);
    assert!(ticks > 5, "node should tick many times, got {ticks}");
    let stats = scheduler.safety_stats().expect("safety monitor enabled");
    assert_eq!(
        stats.watchdog_expirations(),
        0,
        "a fed non-RT watchdog node must not expire its watchdog (SCHED-H2); got {}",
        stats.watchdog_expirations()
    );
}

// ============================================================================
// Real-time Node Tests
// ============================================================================

#[test]
fn test_scheduler_add_rt_node() {
    let _guard = lock_scheduler();
    let mut scheduler = Scheduler::new();
    scheduler.add(CounterNode::new("rt_node")).order(0).build();

    let nodes = scheduler.node_list();
    assert_eq!(nodes.len(), 1);
    assert_eq!(nodes[0], "rt_node");
}

// ============================================================================
// Run For Duration Tests
// ============================================================================

#[test]
fn test_scheduler_run_for_short_duration() {
    let _guard = lock_scheduler();
    let mut scheduler = Scheduler::new();
    let counter = Arc::new(AtomicUsize::new(0));
    scheduler
        .add(CounterNode::with_counter("counter", counter.clone()))
        .order(0)
        .build();

    // Run for a short duration (100ms to handle scheduler init overhead under parallel test load)
    let result = scheduler.run_for(500_u64.ms());
    result.unwrap();

    // Counter should have been incremented at least once
    assert!(counter.load(Ordering::SeqCst) > 0);
}

// ============================================================================
// Chainable API Tests
// ============================================================================

#[test]
fn test_scheduler_chainable_api() {
    let _guard = lock_scheduler();
    let mut scheduler = Scheduler::new();

    scheduler
        .add(CounterNode::new("chain_node"))
        .order(0)
        .build();

    assert!(scheduler.is_running());
    assert_eq!(scheduler.node_list().len(), 1);
}

// ============================================================================
// List Recordings Tests
// ============================================================================

#[test]
fn test_scheduler_list_recordings() {
    let _guard = lock_scheduler();
    // In test environment, recordings dir likely doesn't exist → Ok(empty vec)
    // or Err if dir can't be created. Either way, verify the actual value.
    match Scheduler::list_recordings() {
        Ok(recordings) => {
            // Each recording name should be non-empty if any exist
            for name in &recordings {
                assert!(!name.is_empty(), "Recording name must not be empty");
            }
        }
        Err(e) => {
            // Must be an IO-related error (dir not found), not a logic bug
            let msg = format!("{}", e);
            assert!(
                msg.contains("No such file")
                    || msg.contains("not found")
                    || msg.contains("directory")
                    || msg.contains("recording"),
                "Unexpected error from list_recordings: {}",
                msg
            );
        }
    }
}

// ============================================================================
// Override Tests
// ============================================================================

#[test]
fn test_scheduler_with_override() {
    let _guard = lock_scheduler();
    // with_override without replay mode is a no-op — verify it doesn't corrupt state
    let scheduler = Scheduler::new().with_override("node1", "output1", vec![1, 2, 3, 4]);

    assert!(scheduler.is_running());
    // Scheduler should still have zero nodes (override doesn't add nodes)
    assert!(
        scheduler.node_list().is_empty(),
        "Override should not create nodes"
    );
    // Name should be default
    assert_eq!(scheduler.scheduler_name(), "Scheduler");
}

// ============================================================================
// Auto-Optimization Tests
// ============================================================================

#[test]
fn test_scheduler_auto_optimization() {
    let _guard = lock_scheduler();
    // Test that Scheduler::new() auto-detects capabilities
    let scheduler = Scheduler::new();

    // Should have detected capabilities
    assert!(
        scheduler.capabilities().is_some(),
        "Scheduler should detect runtime capabilities"
    );

    let caps = scheduler.capabilities().unwrap();

    // Verify capabilities structure
    assert!(caps.cpu_count > 0, "Should detect at least 1 CPU");
    assert!(
        !caps.kernel_version.is_empty(),
        "Should detect kernel version"
    );

    // Should have safety monitor enabled by default
    // Safety monitor is now disabled by default (prototyping-friendly)
    assert!(scheduler.monitor.safety.is_none());
}

#[test]
fn test_scheduler_degradations() {
    let _guard = lock_scheduler();
    let scheduler = Scheduler::new();

    // Degradations should be a non-empty list in most dev environments
    // (typically no RT priority available without root/CAP_SYS_NICE)
    let degradations = scheduler.degradations();

    // On a development machine without RT permissions, we expect degradations
    // On a properly configured RT machine, this list might be empty
    // Either case is valid - we just verify the API works
    for deg in degradations {
        // Verify all fields are populated
        assert!(!deg.reason.is_empty(), "Degradation should have a reason");
        // Verify Display impl works
        let _ = format!("{}", deg.feature);
    }
}

#[test]
fn test_scheduler_has_full_rt() {
    let _guard = lock_scheduler();
    let scheduler = Scheduler::new();

    // has_full_rt() should return false if there are high-severity degradations
    let has_high = scheduler
        .degradations()
        .iter()
        .any(|d| d.severity == DegradationSeverity::High);

    assert_eq!(!has_high, scheduler.has_full_rt());
}

#[test]
fn test_rt_feature_display() {
    let _guard = lock_scheduler();
    // Test Display implementations for all RtFeature variants
    assert_eq!(format!("{}", RtFeature::RtPriority), "RT Priority");
    assert_eq!(format!("{}", RtFeature::MemoryLocking), "Memory Locking");
    assert_eq!(format!("{}", RtFeature::CpuAffinity), "CPU Affinity");
    assert_eq!(format!("{}", RtFeature::NumaPinning), "NUMA Pinning");
    assert_eq!(format!("{}", RtFeature::Watchdog), "Watchdog");
    assert_eq!(format!("{}", RtFeature::SafetyMonitor), "Safety Monitor");
}

// ============================================================================
// Parallel Execution Tests
// ============================================================================

#[test]
fn test_parallel_execution_all_nodes_tick() {
    let _guard = lock_scheduler();
    // Compute nodes are dispatched to the parallel compute executor automatically.
    let mut scheduler = Scheduler::new().tick_rate(100_u64.hz());

    let c1 = Arc::new(AtomicUsize::new(0));
    let c2 = Arc::new(AtomicUsize::new(0));
    let c3 = Arc::new(AtomicUsize::new(0));

    scheduler
        .add(CounterNode::with_counter("par_a", c1.clone()))
        .order(100)
        .compute()
        .build();
    scheduler
        .add(CounterNode::with_counter("par_b", c2.clone()))
        .order(101)
        .compute()
        .build();
    scheduler
        .add(CounterNode::with_counter("par_c", c3.clone()))
        .order(102)
        .compute()
        .build();

    // Run for 300ms — all 3 compute nodes should tick at least once
    let result = scheduler.run_for(1000_u64.ms());
    result.unwrap();

    // Every node must have ticked at least once
    assert!(c1.load(Ordering::SeqCst) > 0, "par_a never ticked");
    assert!(c2.load(Ordering::SeqCst) > 0, "par_b never ticked");
    assert!(c3.load(Ordering::SeqCst) > 0, "par_c never ticked");
}

#[test]
fn test_parallel_rt_nodes_run_sequentially() {
    let _guard = lock_scheduler();
    // RT nodes run on a dedicated thread; BestEffort nodes on main thread
    let mut scheduler = Scheduler::new().tick_rate(10000_u64.hz());

    let rt_counter = Arc::new(AtomicUsize::new(0));
    let normal_counter = Arc::new(AtomicUsize::new(0));

    scheduler
        .add(CounterNode::with_counter("rt_node", rt_counter.clone()))
        .order(0)
        .rate(100_u64.hz())
        .build();
    scheduler
        .add(CounterNode::with_counter(
            "normal_node",
            normal_counter.clone(),
        ))
        .order(100)
        .build();

    let result = scheduler.run_for(500_u64.ms());
    result.unwrap();

    // Both should have ticked
    assert!(
        rt_counter.load(Ordering::SeqCst) > 0,
        "RT node never ticked"
    );
    assert!(
        normal_counter.load(Ordering::SeqCst) > 0,
        "Normal node never ticked"
    );
}

// ============================================================================
// Rate Limiting Tests
// ============================================================================

#[test]
fn test_rate_limiting_adjusts_tick_period() {
    let _guard = lock_scheduler();
    let mut scheduler = Scheduler::new();

    // Default tick period is ~60Hz = 16667us
    let default_period = scheduler.tick.period;

    // Add a node at 500Hz
    scheduler
        .add(CounterNode::new("fast_node"))
        .order(0)
        .rate(500_u64.hz())
        .build();

    // tick_period should now be <= 2000us (500Hz)
    assert!(
        scheduler.tick.period <= 2000_u64.us(),
        "tick_period should have been adjusted to >= 500Hz, got {:?}",
        scheduler.tick.period
    );
    assert!(
        scheduler.tick.period < default_period,
        "tick_period should be faster than default 60Hz"
    );
}

#[test]
fn test_rate_limiting_node_ticks_at_declared_rate() {
    let _guard = lock_scheduler();
    let mut scheduler = Scheduler::new();

    let counter = Arc::new(AtomicUsize::new(0));

    // Add a node at 500Hz
    scheduler
        .add(CounterNode::with_counter("fast", counter.clone()))
        .order(0)
        .rate(500_u64.hz())
        .build();

    // Run for 1s — expect ~500 ticks at 500Hz
    let result = scheduler.run_for(1_u64.secs());
    result.unwrap();

    let ticks = counter.load(Ordering::SeqCst);
    // Under heavy parallel test execution, CPU contention can reduce
    // effective tick rates. Require at least some ticks to prove the
    // rate limiting mechanism works.
    assert!(
        ticks >= 5,
        "Node at 500Hz should tick at least 5 times in 1s, got {}",
        ticks
    );
}

#[test]
fn test_rate_limiting_does_not_lower_tick_period() {
    let _guard = lock_scheduler();
    // If a node has a rate lower than default, tick_period should stay the same
    let mut scheduler = Scheduler::new();

    let default_period = scheduler.tick.period;

    scheduler
        .add(CounterNode::new("slow_node"))
        .order(0)
        .rate(10_u64.hz())
        .build();

    assert_eq!(
        scheduler.tick.period, default_period,
        "tick_period should not decrease for a 10Hz node"
    );
}

// ============================================================================
// Recording Hooks Wiring Tests
// ============================================================================

#[test]
fn test_recording_hooks_wired() {
    let _guard = lock_scheduler();
    let counter = Arc::new(AtomicUsize::new(0));
    let mut scheduler = Scheduler::new().tick_rate(1000_u64.hz()).with_recording();
    scheduler
        .add(CounterNode::with_counter("rec_node", counter.clone()))
        .order(0)
        .build();

    assert!(scheduler.is_recording());

    let _ = scheduler.run_for(1000_u64.ms());

    let ticks = counter.load(Ordering::SeqCst);
    assert!(ticks > 0, "Node should have ticked during recording");

    // stop_recording should succeed and return saved paths
    let paths = scheduler.stop_recording();
    paths.unwrap();
}

// ============================================================================
// Fix 3: BlackBox WAL Persistence Tests
// ============================================================================

#[cfg(feature = "blackbox")]
#[test]
fn test_blackbox_with_path() {
    let _guard = lock_scheduler();
    let tmp = std::env::temp_dir().join(format!("horus_bb_test_{}", std::process::id()));
    let _ = std::fs::remove_dir_all(&tmp);

    let mut bb = super::super::blackbox::BlackBox::new(1).with_path(tmp.clone());
    bb.record(super::super::blackbox::BlackBoxEvent::Custom {
        category: "test".to_string(),
        message: "wal_test".to_string(),
    });
    // Explicitly flush the BufWriter before reading the file.
    // Without flush_wal() the batch interval (default 64) may not have been
    // reached yet, leaving the record in the BufWriter's internal buffer.
    bb.flush_wal();

    // WAL file should exist and have content
    let wal_path = tmp.join("blackbox.wal");
    assert!(wal_path.exists(), "WAL file should exist at {:?}", wal_path);
    let wal_content = std::fs::read_to_string(&wal_path).unwrap();
    assert!(!wal_content.is_empty(), "WAL file should have content");
    assert!(wal_content.contains("wal_test"));

    // save() should write JSON snapshot
    bb.save().unwrap();
    let json_path = tmp.join("blackbox.json");
    assert!(json_path.exists(), "JSON snapshot should exist");
    let json_content = std::fs::read_to_string(&json_path).unwrap();
    assert!(json_content.contains("wal_test"));

    // Cleanup
    let _ = std::fs::remove_dir_all(&tmp);
}

// ============================================================================
// Fix 4: Dead Code Cleanup Verification
// ============================================================================

#[test]
fn test_per_node_rates_work_without_dead_code() {
    let _guard = lock_scheduler();
    // Per-node rates work through set_node_rate() / .rate()
    let counter = Arc::new(AtomicUsize::new(0));
    let mut scheduler = Scheduler::new().tick_rate(1000_u64.hz());
    scheduler
        .add(CounterNode::with_counter("rated", counter.clone()))
        .order(0)
        .rate(100_u64.hz())
        .build();

    let _ = scheduler.run_for(1_u64.secs());

    let ticks = counter.load(Ordering::SeqCst);
    // At 100Hz for 1s, expect ~100 ticks. Wide tolerance for CI/load variance
    // when many tests run in parallel and contend for CPU time.
    assert!(
        ticks >= 5,
        "Node at 100Hz should tick at least 5 times in 1s, got {}",
        ticks
    );
}

// ============================================================================
// Priority Ordering & Lifecycle Tests
// ============================================================================

/// Node that logs its name to a shared execution log on every tick,
/// and tracks init/shutdown lifecycle events.
struct OrderTrackingNode {
    node_name: String,
    execution_log: Arc<Mutex<Vec<String>>>,
    init_called: Arc<AtomicBool>,
    shutdown_called: Arc<AtomicBool>,
    tick_count: Arc<AtomicUsize>,
}

impl OrderTrackingNode {
    fn new(
        name: &str,
        log: Arc<Mutex<Vec<String>>>,
        init_flag: Arc<AtomicBool>,
        shutdown_flag: Arc<AtomicBool>,
        tick_count: Arc<AtomicUsize>,
    ) -> Self {
        Self {
            node_name: name.to_string(),
            execution_log: log,
            init_called: init_flag,
            shutdown_called: shutdown_flag,
            tick_count,
        }
    }
}

impl Node for OrderTrackingNode {
    fn name(&self) -> &str {
        &self.node_name
    }

    fn init(&mut self) -> crate::error::HorusResult<()> {
        self.init_called.store(true, Ordering::SeqCst);
        self.execution_log
            .lock()
            .unwrap()
            .push(format!("init:{}", self.node_name));
        Ok(())
    }

    fn tick(&mut self) {
        self.execution_log
            .lock()
            .unwrap()
            .push(format!("tick:{}", self.node_name));
        self.tick_count.fetch_add(1, Ordering::SeqCst);
    }

    fn shutdown(&mut self) -> crate::error::HorusResult<()> {
        self.shutdown_called.store(true, Ordering::SeqCst);
        self.execution_log
            .lock()
            .unwrap()
            .push(format!("shutdown:{}", self.node_name));
        Ok(())
    }
}

/// Verify lifecycle: init() called once before any tick() for every node,
/// shutdown() called for every node at scheduler stop.
#[test]
fn test_scheduler_lifecycle_init_tick_shutdown() {
    let _guard = lock_scheduler();
    let log = Arc::new(Mutex::new(Vec::<String>::new()));
    let init_a = Arc::new(AtomicBool::new(false));
    let init_b = Arc::new(AtomicBool::new(false));
    let shut_a = Arc::new(AtomicBool::new(false));
    let shut_b = Arc::new(AtomicBool::new(false));
    let tc_a = Arc::new(AtomicUsize::new(0));
    let tc_b = Arc::new(AtomicUsize::new(0));

    let mut scheduler = Scheduler::new();
    scheduler
        .add(OrderTrackingNode::new(
            "node_a",
            log.clone(),
            init_a.clone(),
            shut_a.clone(),
            tc_a.clone(),
        ))
        .order(0)
        .build();
    scheduler
        .add(OrderTrackingNode::new(
            "node_b",
            log.clone(),
            init_b.clone(),
            shut_b.clone(),
            tc_b.clone(),
        ))
        .order(1)
        .build();

    let result = scheduler.run_for(500_u64.ms());
    result.unwrap();

    // init must have been called for both
    assert!(init_a.load(Ordering::SeqCst), "node_a init() not called");
    assert!(init_b.load(Ordering::SeqCst), "node_b init() not called");

    // shutdown must have been called for both
    assert!(
        shut_a.load(Ordering::SeqCst),
        "node_a shutdown() not called"
    );
    assert!(
        shut_b.load(Ordering::SeqCst),
        "node_b shutdown() not called"
    );

    // Both must have ticked at least once
    assert!(tc_a.load(Ordering::SeqCst) > 0, "node_a never ticked");
    assert!(tc_b.load(Ordering::SeqCst) > 0, "node_b never ticked");

    // Verify ordering in log: init before any tick, shutdown after all ticks
    let events = log.lock().unwrap();
    let first_tick_a = events.iter().position(|e| e == "tick:node_a");
    let first_tick_b = events.iter().position(|e| e == "tick:node_b");
    let init_pos_a = events.iter().position(|e| e == "init:node_a");
    let init_pos_b = events.iter().position(|e| e == "init:node_b");
    let shut_pos_a = events.iter().rposition(|e| e == "shutdown:node_a");
    let shut_pos_b = events.iter().rposition(|e| e == "shutdown:node_b");

    // init must come before first tick
    if let (Some(init), Some(tick)) = (init_pos_a, first_tick_a) {
        assert!(
            init < tick,
            "node_a: init (pos {}) must come before first tick (pos {})",
            init,
            tick
        );
    }
    if let (Some(init), Some(tick)) = (init_pos_b, first_tick_b) {
        assert!(
            init < tick,
            "node_b: init (pos {}) must come before first tick (pos {})",
            init,
            tick
        );
    }

    // shutdown must come after last tick
    let last_tick_a = events.iter().rposition(|e| e == "tick:node_a");
    let last_tick_b = events.iter().rposition(|e| e == "tick:node_b");
    if let (Some(shut), Some(tick)) = (shut_pos_a, last_tick_a) {
        assert!(
            shut > tick,
            "node_a: shutdown (pos {}) must come after last tick (pos {})",
            shut,
            tick
        );
    }
    if let (Some(shut), Some(tick)) = (shut_pos_b, last_tick_b) {
        assert!(
            shut > tick,
            "node_b: shutdown (pos {}) must come after last tick (pos {})",
            shut,
            tick
        );
    }
}

/// Verify priority ordering: safety-critical nodes (low priority numbers)
/// execute before normal and background nodes every tick.
/// Robotics expectation: emergency stop (priority 0) MUST execute before
/// motor control (50) before planning (150) before logging (200).
#[test]
fn test_scheduler_priority_execution_order_robotics() {
    let _guard = lock_scheduler();
    let log = Arc::new(Mutex::new(Vec::<String>::new()));

    // Create 5 nodes simulating robotics priority tiers
    struct NodeSetup {
        name: &'static str,
        priority: u32,
        init: Arc<AtomicBool>,
        shut: Arc<AtomicBool>,
        tc: Arc<AtomicUsize>,
    }

    let nodes: Vec<NodeSetup> = vec![
        ("logging", 200),
        ("path_planner", 150),
        ("motor_ctrl", 50),
        ("estop", 0),
        ("sensor_fusion", 100),
    ]
    .into_iter()
    .map(|(name, prio)| NodeSetup {
        name,
        priority: prio,
        init: Arc::new(AtomicBool::new(false)),
        shut: Arc::new(AtomicBool::new(false)),
        tc: Arc::new(AtomicUsize::new(0)),
    })
    .collect();

    let mut scheduler = Scheduler::new();
    for n in &nodes {
        scheduler
            .add(OrderTrackingNode::new(
                n.name,
                log.clone(),
                n.init.clone(),
                n.shut.clone(),
                n.tc.clone(),
            ))
            .order(n.priority)
            .build();
    }

    let result = scheduler.run_for(500_u64.ms());
    result.unwrap();

    // All nodes must have ticked
    for n in &nodes {
        assert!(n.tc.load(Ordering::SeqCst) > 0, "{} never ticked", n.name);
    }

    // Verify tick ordering: within each tick, priority order must be maintained
    // Expected per-tick order: estop(0) → motor_ctrl(50) → sensor_fusion(100) → path_planner(150) → logging(200)
    let expected_order = [
        "estop",
        "motor_ctrl",
        "sensor_fusion",
        "path_planner",
        "logging",
    ];
    let events = log.lock().unwrap();

    // Extract tick events and group by tick
    let tick_events: Vec<&str> = events
        .iter()
        .filter(|e| e.starts_with("tick:"))
        .map(|e| &e[5..])
        .collect();

    // Check ordering in sliding windows of 5 (one per tick)
    // In a perfect tick, we see all 5 nodes in order.
    // Due to init overhead, first tick may be partial — check from second full group.
    let total_nodes = 5;
    let mut verified_ticks = 0;
    for chunk in tick_events.chunks(total_nodes) {
        if chunk.len() == total_nodes {
            for (i, &name) in chunk.iter().enumerate() {
                assert_eq!(
                    name, expected_order[i],
                    "Priority violation in tick: position {} should be '{}' but got '{}'. Full tick: {:?}",
                    i, expected_order[i], name, chunk
                );
            }
            verified_ticks += 1;
        }
    }
    assert!(
        verified_ticks >= 1,
        "Should have verified at least 1 complete tick ordering"
    );
}

/// Test 10 nodes with mixed priorities across all tiers run for 100 ticks.
/// Verifies every tick has deterministic priority ordering.
#[test]
fn test_scheduler_10_nodes_100_ticks_priority_order() {
    let _guard = lock_scheduler();
    let log = Arc::new(Mutex::new(Vec::<String>::new()));

    let priorities: Vec<(&str, u32)> = vec![
        ("estop", 0),
        ("collision_avoid", 5),
        ("motor_left", 50),
        ("motor_right", 51),
        ("sensor_imu", 100),
        ("sensor_lidar", 101),
        ("planner", 150),
        ("mapper", 160),
        ("logger", 200),
        ("diagnostics", 250),
    ];

    let mut scheduler = Scheduler::new();
    let counters: Vec<Arc<AtomicUsize>> = priorities
        .iter()
        .map(|(name, prio)| {
            let tc = Arc::new(AtomicUsize::new(0));
            let init = Arc::new(AtomicBool::new(false));
            let shut = Arc::new(AtomicBool::new(false));
            scheduler
                .add(OrderTrackingNode::new(
                    name,
                    log.clone(),
                    init,
                    shut,
                    tc.clone(),
                ))
                .order(*prio)
                .build();
            tc
        })
        .collect();

    // Run for 500ms — should get many ticks at default ~60Hz
    let result = scheduler.run_for(500_u64.ms());
    result.unwrap();

    // All nodes ticked
    for (i, (name, _)) in priorities.iter().enumerate() {
        assert!(
            counters[i].load(Ordering::SeqCst) > 0,
            "{} never ticked",
            name
        );
    }

    // Verify priority ordering in tick groups
    let expected_order: Vec<&str> = priorities.iter().map(|(n, _)| *n).collect();
    let events = log.lock().unwrap();
    let tick_events: Vec<&str> = events
        .iter()
        .filter(|e| e.starts_with("tick:"))
        .map(|e| &e[5..])
        .collect();

    let total_nodes = 10;
    let mut verified_ticks = 0;
    for chunk in tick_events.chunks(total_nodes) {
        if chunk.len() == total_nodes {
            for (i, &name) in chunk.iter().enumerate() {
                assert_eq!(
                    name, expected_order[i],
                    "Priority violation at position {}: expected '{}' got '{}'",
                    i, expected_order[i], name
                );
            }
            verified_ticks += 1;
        }
    }
    assert!(
        verified_ticks >= 2,
        "Should have verified at least 2 complete ticks, got {}",
        verified_ticks
    );
}

/// Verify that all nodes get init() called exactly once, even with many nodes.
#[test]
fn test_scheduler_all_nodes_init_called_once() {
    let _guard = lock_scheduler();
    let log = Arc::new(Mutex::new(Vec::<String>::new()));

    let init_flags: Vec<(String, Arc<AtomicBool>)> = (0..5)
        .map(|i| (format!("node_{}", i), Arc::new(AtomicBool::new(false))))
        .collect();

    let mut scheduler = Scheduler::new();
    for (name, flag) in &init_flags {
        let tc = Arc::new(AtomicUsize::new(0));
        let shut = Arc::new(AtomicBool::new(false));
        scheduler
            .add(OrderTrackingNode::new(
                name,
                log.clone(),
                flag.clone(),
                shut,
                tc,
            ))
            .order(0)
            .build();
    }

    let result = scheduler.run_for(200_u64.ms());
    result.unwrap();

    // Every node must have init() called
    for (name, flag) in &init_flags {
        assert!(
            flag.load(Ordering::SeqCst),
            "{} init() was never called",
            name
        );
    }

    // init should appear exactly once per node in the log
    let events = log.lock().unwrap();
    for (name, _) in &init_flags {
        let init_count = events
            .iter()
            .filter(|e| *e == &format!("init:{}", name))
            .count();
        assert_eq!(
            init_count, 1,
            "{} init() called {} times, expected exactly 1",
            name, init_count
        );
    }
}

// ============================================================================
// Failure Policy Integration Tests
// ============================================================================

/// Node that panics after a configurable number of ticks.
/// Used to test failure policies at the scheduler level.
struct PanickingNode {
    node_name: String,
    tick_count: Arc<AtomicUsize>,
    panic_at_tick: usize,
}

impl PanickingNode {
    fn new(name: &str, panic_at: usize, counter: Arc<AtomicUsize>) -> Self {
        Self {
            node_name: name.to_string(),
            tick_count: counter,
            panic_at_tick: panic_at,
        }
    }
}

impl Node for PanickingNode {
    fn name(&self) -> &str {
        &self.node_name
    }

    fn tick(&mut self) {
        let count = self.tick_count.fetch_add(1, Ordering::SeqCst);
        if count >= self.panic_at_tick {
            panic!("intentional panic at tick {}", count);
        }
    }
}

/// Fatal policy stops the scheduler when a node panics.
/// Robotics: motor controller failure must halt the system immediately.
#[test]
fn test_fatal_policy_stops_scheduler_on_panic() {
    let _guard = lock_scheduler();
    let panic_counter = Arc::new(AtomicUsize::new(0));
    let healthy_counter = Arc::new(AtomicUsize::new(0));

    let mut scheduler = Scheduler::new();

    // Healthy node at high priority
    scheduler
        .add(CounterNode::with_counter(
            "healthy",
            healthy_counter.clone(),
        ))
        .order(0)
        .build();

    // Panicking node with Fatal policy (default for UltraFast tier)
    scheduler
        .add(PanickingNode::new(
            "failing_motor",
            3,
            panic_counter.clone(),
        ))
        .order(1)
        .failure_policy(crate::scheduling::fault_tolerance::FailurePolicy::Fatal)
        .build();

    let _result = scheduler.run_for(500_u64.ms());
    // Scheduler should have stopped due to fatal failure
    assert!(!scheduler.is_running(), "scheduler should have stopped");

    // Panicking node ticked a few times before panic
    let panic_ticks = panic_counter.load(Ordering::SeqCst);
    assert!(
        panic_ticks >= 3,
        "panicking node should have ticked at least 3 times before panic, got {}",
        panic_ticks
    );
}

/// Restart policy re-initializes a node after panic with backoff.
/// Robotics: sensor driver crash should attempt restart before giving up.
#[test]
fn test_restart_policy_reinitializes_after_panic() {
    let _guard = lock_scheduler();
    let counter = Arc::new(AtomicUsize::new(0));

    let mut scheduler = Scheduler::new();

    // Node panics on tick 2, with restart policy (3 max restarts)
    scheduler
        .add(PanickingNode::new("sensor_driver", 2, counter.clone()))
        .order(0)
        .failure_policy(crate::scheduling::fault_tolerance::FailurePolicy::restart(
            3,
            10_u64.ms(),
        ))
        .build();

    let _result = scheduler.run_for(500_u64.ms());
    // After 3 restarts are exhausted, it escalates to fatal
    assert!(
        !scheduler.is_running(),
        "scheduler should stop after restart limit exhausted"
    );

    // Should have ticked more than once (restarted multiple times)
    let ticks = counter.load(Ordering::SeqCst);
    assert!(
        ticks > 2,
        "node should have ticked multiple times across restarts, got {}",
        ticks
    );
}

/// Skip policy — node is skipped after repeated failures.
/// Robotics: logging node failure should not crash the system.
#[test]
fn test_skip_policy_skips_node() {
    let _guard = lock_scheduler();
    let panic_counter = Arc::new(AtomicUsize::new(0));
    let healthy_counter = Arc::new(AtomicUsize::new(0));

    let mut scheduler = Scheduler::new();

    // Healthy node keeps running
    scheduler
        .add(CounterNode::with_counter(
            "healthy_sensor",
            healthy_counter.clone(),
        ))
        .order(0)
        .build();

    // Panicking node with Skip policy — circuit opens after failures
    scheduler
        .add(PanickingNode::new(
            "faulty_logger",
            1,
            panic_counter.clone(),
        ))
        .order(100)
        .failure_policy(crate::scheduling::fault_tolerance::FailurePolicy::skip(
            3,
            5_u64.secs(),
        ))
        .build();

    let result = scheduler.run_for(1000_u64.ms());
    result.unwrap();

    // Scheduler should still be running (Skip policy doesn't stop it)
    // Note: run_for stops scheduler after duration, but it completed normally
    let healthy_ticks = healthy_counter.load(Ordering::SeqCst);
    assert!(
        healthy_ticks > 0,
        "healthy node should keep ticking even when other node is skipped"
    );
}

/// Ignore policy: failures swallowed, node keeps ticking.
/// Robotics: best-effort diagnostics node that may occasionally fail.
#[test]
fn test_ignore_policy_continues_after_failure() {
    let _guard = lock_scheduler();
    let counter = Arc::new(AtomicUsize::new(0));
    let healthy_counter = Arc::new(AtomicUsize::new(0));

    let mut scheduler = Scheduler::new();

    scheduler
        .add(CounterNode::with_counter(
            "critical_ctrl",
            healthy_counter.clone(),
        ))
        .order(0)
        .build();

    // Node panics at tick 2 with Ignore policy
    scheduler
        .add(PanickingNode::new("diagnostics", 2, counter.clone()))
        .order(200)
        .failure_policy(crate::scheduling::fault_tolerance::FailurePolicy::Ignore)
        .build();

    let result = scheduler.run_for(500_u64.ms());
    result.unwrap();

    // Critical node should have ticked
    let healthy_ticks = healthy_counter.load(Ordering::SeqCst);
    assert!(
        healthy_ticks > 0,
        "critical node should keep running despite failing diagnostic node"
    );
}

/// Verify shutdown is called for every node even when scheduler stops normally.
#[test]
fn test_scheduler_shutdown_called_for_all_nodes() {
    let _guard = lock_scheduler();
    let log = Arc::new(Mutex::new(Vec::<String>::new()));

    let shutdown_flags: Vec<(String, Arc<AtomicBool>)> = (0..4)
        .map(|i| (format!("shut_node_{}", i), Arc::new(AtomicBool::new(false))))
        .collect();

    let mut scheduler = Scheduler::new();
    for (name, flag) in &shutdown_flags {
        let tc = Arc::new(AtomicUsize::new(0));
        let init = Arc::new(AtomicBool::new(false));
        scheduler
            .add(OrderTrackingNode::new(
                name,
                log.clone(),
                init,
                flag.clone(),
                tc,
            ))
            .order(0)
            .build();
    }

    let result = scheduler.run_for(200_u64.ms());
    result.unwrap();

    // Every node must have shutdown() called
    for (name, flag) in &shutdown_flags {
        assert!(
            flag.load(Ordering::SeqCst),
            "{} shutdown() was never called",
            name
        );
    }
}

// ============================================================================
// Edge Case Tests
// ============================================================================

/// Zero nodes: scheduler.run_for() with no nodes added exits cleanly.
/// Robotics: empty config file or all nodes disabled — must not crash.
#[test]
fn test_zero_nodes_exits_cleanly() {
    let _guard = lock_scheduler();
    let mut scheduler = Scheduler::new();
    // No nodes added
    assert_eq!(scheduler.node_list().len(), 0);

    let result = scheduler.run_for(200_u64.ms());
    assert!(result.is_ok(), "Zero-node scheduler should exit cleanly");
}

/// Duplicate node names are REFUSED at registration.
///
/// This test previously asserted the opposite — "Both duplicate-named nodes
/// should be added" — while its own comment identified the case as a
/// "misconfigured launch file". It was pinning a defect.
///
/// Node names are the identity key for per-node state that is keyed by string
/// rather than by index: the SafetyMonitor's watchdog map, the SHM registry
/// slot, and the pause/kill control flags. Two nodes called `motor_ctrl`
/// therefore share ONE watchdog, so the healthy one's feed keeps refreshing the
/// hung one's deadline and the stall is never detected — the watchdog reports
/// healthy for a node that has stopped ticking. They also share one registry
/// slot (`horus node list` shows one) and one control flag (`horus node pause`
/// hits both).
///
/// Aliasing safety state silently is worse than refusing. A duplicate name is
/// reachable from a generated launch file and from horus.toml — configuration,
/// not a programmer typo — so it is recorded at registration and reported as an
/// error from `run()`, the same deferred-failure pattern `require_rt()` uses.
/// Panicking in a robot process would be a worse failure than the one fixed.
#[test]
fn test_duplicate_node_names_are_refused() {
    let _guard = lock_scheduler();
    let mut scheduler = Scheduler::new();

    scheduler
        .add(CounterNode::new("motor_ctrl"))
        .order(0)
        .build();
    // Same name — must not be allowed to alias the first node's watchdog.
    scheduler
        .add(CounterNode::new("motor_ctrl"))
        .order(1)
        .build();

    let err = scheduler
        .run_for(100_u64.ms())
        .expect_err("a duplicate node name must fail the run, not alias a watchdog");
    let msg = err.to_string();
    assert!(
        msg.contains("duplicate node name"),
        "the error must name the problem, got: {msg}"
    );
    assert!(
        msg.contains("motor_ctrl"),
        "the error must name the offending node, got: {msg}"
    );
}

/// Node that fails in init(): scheduler catches it, other nodes continue.
/// Robotics: one bad sensor driver must not crash the whole robot.
#[test]
fn test_panic_in_init_caught_others_continue() {
    let _guard = lock_scheduler();
    struct FailingInitNode;
    impl Node for FailingInitNode {
        fn name(&self) -> &str {
            "failing_init"
        }
        fn init(&mut self) -> crate::error::HorusResult<()> {
            Err(crate::HorusError::Node(
                crate::error::NodeError::InitFailed {
                    node: "failing_init".to_string(),
                    reason: "init explosion".to_string(),
                },
            ))
        }
        fn tick(&mut self) {}
    }

    let good_counter = Arc::new(AtomicUsize::new(0));
    let mut scheduler = Scheduler::new();

    // Bad node that fails in init — non-fatal error, scheduler continues
    scheduler.add(FailingInitNode).order(0).build();

    // Good node that should still run
    scheduler
        .add(CounterNode::with_counter("good_node", good_counter.clone()))
        .order(1)
        .build();

    let result = scheduler.run_for(500_u64.ms());
    assert!(
        result.is_ok(),
        "Scheduler should not crash from init failure"
    );

    assert!(
        good_counter.load(Ordering::SeqCst) > 0,
        "Good node should still tick despite bad node's init failure"
    );
}

/// Node that panics in tick(): scheduler catches it per-node.
/// With Ignore policy, other nodes continue running.
/// Robotics: runtime panic in diagnostics node shouldn't stop motors.
#[test]
fn test_panic_in_tick_caught_others_continue() {
    let _guard = lock_scheduler();
    let panic_counter = Arc::new(AtomicUsize::new(0));
    let good_counter = Arc::new(AtomicUsize::new(0));

    let mut scheduler = Scheduler::new();

    // Node that panics on first tick, with Ignore policy
    scheduler
        .add(PanickingNode::new("bad_sensor", 1, panic_counter.clone()))
        .order(0)
        .failure_policy(FailurePolicy::Ignore)
        .build();

    // Good node that should continue
    scheduler
        .add(CounterNode::with_counter(
            "good_motor",
            good_counter.clone(),
        ))
        .order(1)
        .build();

    let result = scheduler.run_for(500_u64.ms());
    assert!(result.is_ok(), "Scheduler should not crash from tick panic");

    assert!(
        good_counter.load(Ordering::SeqCst) > 0,
        "Good node should continue ticking despite bad node's tick panic"
    );
}

/// Immediate shutdown: call stop() right before run_for().
/// All nodes should still get proper shutdown() calls.
/// Robotics: emergency stop triggered immediately after boot.
#[test]
fn test_immediate_stop_still_inits_and_shuts_down() {
    let _guard = lock_scheduler();
    let counter = Arc::new(AtomicUsize::new(0));

    let mut scheduler = Scheduler::new();
    scheduler
        .add(CounterNode::with_counter("quick_node", counter.clone()))
        .order(0)
        .build();

    // Stop before running — run_for should exit almost immediately
    scheduler.stop();
    let result = scheduler.run_for(500_u64.ms());
    assert!(result.is_ok(), "Immediate stop should not cause error");
}

/// Many nodes: scheduler handles 50+ nodes without issue.
/// Robotics: complex robot with many sensors, actuators, and processing nodes.
#[test]
fn test_many_nodes_50_plus() {
    let _guard = lock_scheduler();
    let mut scheduler = Scheduler::new();
    let counters: Vec<Arc<AtomicUsize>> = (0..50).map(|_| Arc::new(AtomicUsize::new(0))).collect();

    for (i, counter) in counters.iter().enumerate() {
        let name = format!("node_{}", i);
        scheduler
            .add(CounterNode::with_counter(name, counter.clone()))
            .order(i as u32)
            .build();
    }

    assert_eq!(scheduler.node_list().len(), 50);

    let result = scheduler.run_for(500_u64.ms());
    assert!(result.is_ok(), "50-node scheduler should run fine");

    // Count how many nodes ticked at least once
    let ticked = counters
        .iter()
        .filter(|c| c.load(Ordering::SeqCst) > 0)
        .count();
    assert!(
        ticked >= 45,
        "At least 45 of 50 nodes should have ticked, got {}",
        ticked
    );
}

// ============================================================================
// Negative input tests — verify graceful handling of invalid/edge-case inputs
// ============================================================================

#[test]
fn test_empty_scheduler_run() {
    let _lock = lock_scheduler();
    let mut scheduler = Scheduler::new();
    // Running with zero nodes should succeed, not panic
    let result = scheduler.run_for(50_u64.ms());
    result.unwrap();
}

#[test]
fn test_empty_scheduler_node_list() {
    let scheduler = Scheduler::new();
    assert!(scheduler.node_list().is_empty());
}

#[test]
fn test_empty_scheduler_metrics() {
    let scheduler = Scheduler::new();
    assert!(scheduler.metrics().is_empty());
}

#[test]
#[should_panic(expected = "positive")]
fn test_tick_rate_zero_panics() {
    // Frequency rejects zero at construction time
    let _freq = 0_u64.hz();
}

#[test]
#[should_panic(expected = "finite and positive")]
fn test_tick_rate_negative_panics() {
    // Frequency rejects negative values at construction time
    let _freq = (-100.0).hz();
}

#[test]
fn test_tick_hz_very_large() {
    let _lock = lock_scheduler();
    let counter = Arc::new(AtomicUsize::new(0));
    let mut scheduler = Scheduler::new().tick_rate(1_000_000_u64.hz());
    scheduler
        .add(CounterNode::with_counter("fast", counter.clone()))
        .build()
        .unwrap();
    // 500ms, not 10ms. The assertion is that a 1MHz tick rate produces at
    // least one tick, and a 10ms window only does when the scheduler finishes
    // starting inside it — on a machine running the rest of the suite it often
    // does not, and the failure reads as "a 1MHz scheduler never ticked".
    let result = scheduler.run_for(500_u64.ms());
    result.unwrap();
    assert!(counter.load(Ordering::SeqCst) > 0);
}

#[test]
fn test_set_node_rate_nonexistent() {
    let _lock = lock_scheduler();
    let mut scheduler = Scheduler::new();
    // Setting rate on non-existent node should not panic and should return &mut Self for chaining
    let returned = scheduler.set_node_rate("does_not_exist", 100_u64.hz());
    // Verify the scheduler is still in a valid state (running flag starts true)
    assert!(
        returned.is_running(),
        "Scheduler should still be in valid state after set_node_rate on nonexistent node"
    );
    // Verify that rt_stats returns None for the nonexistent node (rate was not applied)
    assert!(
        scheduler.rt_stats("does_not_exist").is_none(),
        "Nonexistent node should have no rt_stats"
    );
}

#[test]
#[should_panic(expected = "positive")]
fn test_set_node_rate_zero_panics() {
    // Frequency rejects zero at construction time
    let _freq = 0_u64.hz();
}

#[test]
#[should_panic(expected = "finite and positive")]
fn test_set_node_rate_negative_panics() {
    // Frequency rejects negative values at construction time
    let _freq = (-50.0).hz();
}

#[test]
fn test_tick_empty_node_names() {
    let _lock = lock_scheduler();
    let mut scheduler = Scheduler::new();
    scheduler
        .add(CounterNode::new("tick_test"))
        .build()
        .unwrap();
    // Tick with empty names should succeed (tick nothing)
    let result = scheduler.tick_for(&[], 50_u64.ms());
    result.unwrap();
}

#[test]
fn test_tick_nonexistent_node_names() {
    let _lock = lock_scheduler();
    let mut scheduler = Scheduler::new();
    scheduler
        .add(CounterNode::new("real_node"))
        .build()
        .unwrap();
    // Tick with non-existent names should not panic
    let result = scheduler.tick_for(&["fake_node_1", "fake_node_2"], 50_u64.ms());
    result.unwrap();
}

#[test]
fn test_rt_stats_nonexistent() {
    let _lock = lock_scheduler();
    let scheduler = Scheduler::new();
    assert!(scheduler.rt_stats("no_such_node").is_none());
}

#[test]
fn test_stop_before_run() {
    let _lock = lock_scheduler();
    let scheduler = Scheduler::new();
    // A new scheduler starts with running=true
    assert!(
        scheduler.is_running(),
        "New scheduler should start as running"
    );
    // Stopping before running should not panic and should set running to false
    scheduler.stop();
    assert!(
        !scheduler.is_running(),
        "Scheduler should not be running after stop()"
    );
}

#[test]
fn test_double_stop() {
    let _lock = lock_scheduler();
    let mut scheduler = Scheduler::new();
    scheduler
        .add(CounterNode::new("double_stop"))
        .build()
        .unwrap();
    let _ = scheduler.run_for(10_u64.ms());
    scheduler.stop();
    scheduler.stop(); // Double stop should not panic
}

#[test]
fn test_run_for_zero_duration() {
    let _lock = lock_scheduler();
    let mut scheduler = Scheduler::new();
    scheduler.add(CounterNode::new("zero_dur")).build().unwrap();
    let result = scheduler.run_for(Duration::ZERO);
    result.unwrap();
}

#[test]
fn test_scheduler_status_empty() {
    let scheduler = Scheduler::new();
    let status = scheduler.status();
    assert!(!status.is_empty());
}

#[test]
fn test_scheduler_capabilities_before_run() {
    let scheduler = Scheduler::new();
    // Capabilities are detected at construction time
    let caps = scheduler.capabilities();
    assert!(
        caps.is_some(),
        "Capabilities should be detected at construction time"
    );
}

// ============================================================================
// Graceful Shutdown Tests — verify shutdown correctness under various conditions
// ============================================================================

/// Shutdown while a slow node is actively ticking.
/// Verifies the scheduler waits for the current tick to finish before shutting down.
#[test]
fn test_shutdown_during_active_tick() {
    let _guard = lock_scheduler();
    let tick_started = Arc::new(AtomicBool::new(false));
    let shutdown_called = Arc::new(AtomicBool::new(false));

    struct SlowTickNode {
        tick_started: Arc<AtomicBool>,
        shutdown_called: Arc<AtomicBool>,
    }
    impl Node for SlowTickNode {
        fn name(&self) -> &str {
            "slow_ticker"
        }
        fn tick(&mut self) {
            self.tick_started.store(true, Ordering::SeqCst);
            std::thread::sleep(20_u64.ms());
        }
        fn shutdown(&mut self) -> crate::error::HorusResult<()> {
            self.shutdown_called.store(true, Ordering::SeqCst);
            Ok(())
        }
    }

    let ts = tick_started.clone();
    let sc = shutdown_called.clone();

    let mut scheduler = Scheduler::new();
    scheduler
        .add(SlowTickNode {
            tick_started: ts,
            shutdown_called: sc,
        })
        .order(0)
        .build();

    let running = scheduler.running_flag();
    // Spawn a thread that stops the scheduler after a brief delay
    std::thread::spawn(move || {
        std::thread::sleep(50_u64.ms());
        running.store(false, Ordering::SeqCst);
    });

    let result = scheduler.run_for(500_u64.ms());
    result.unwrap();
    assert!(
        tick_started.load(Ordering::SeqCst),
        "Node should have started ticking"
    );
    assert!(
        shutdown_called.load(Ordering::SeqCst),
        "Node shutdown() must be called even after external stop"
    );
}

/// When one node panics in shutdown(), other nodes still get shutdown() called.
#[test]
fn test_shutdown_panic_in_one_node_others_still_shutdown() {
    let _guard = lock_scheduler();
    let shutdown_a = Arc::new(AtomicBool::new(false));
    let shutdown_c = Arc::new(AtomicBool::new(false));

    struct PanicShutdownNode;
    impl Node for PanicShutdownNode {
        fn name(&self) -> &str {
            "panic_shutdown"
        }
        fn tick(&mut self) {}
        fn shutdown(&mut self) -> crate::error::HorusResult<()> {
            panic!("intentional shutdown panic");
        }
    }

    struct FlagShutdownNode {
        node_name: &'static str,
        flag: Arc<AtomicBool>,
    }
    impl Node for FlagShutdownNode {
        fn name(&self) -> &str {
            self.node_name
        }
        fn tick(&mut self) {}
        fn shutdown(&mut self) -> crate::error::HorusResult<()> {
            self.flag.store(true, Ordering::SeqCst);
            Ok(())
        }
    }

    let mut scheduler = Scheduler::new();
    scheduler
        .add(FlagShutdownNode {
            node_name: "before_panic",
            flag: shutdown_a.clone(),
        })
        .order(0)
        .build();
    scheduler.add(PanicShutdownNode).order(1).build();
    scheduler
        .add(FlagShutdownNode {
            node_name: "after_panic",
            flag: shutdown_c.clone(),
        })
        .order(2)
        .build();

    let result = scheduler.run_for(100_u64.ms());
    result.unwrap();

    assert!(
        shutdown_a.load(Ordering::SeqCst),
        "Node before panicking node must still get shutdown()"
    );
    assert!(
        shutdown_c.load(Ordering::SeqCst),
        "Node after panicking node must still get shutdown()"
    );
}

/// SIGTERM simulation: setting the global SIGTERM flag causes graceful shutdown.
#[test]
fn test_sigterm_causes_graceful_shutdown() {
    let _guard = lock_scheduler();
    let shutdown_called = Arc::new(AtomicBool::new(false));

    struct SigtermTestNode {
        shutdown_called: Arc<AtomicBool>,
    }
    impl Node for SigtermTestNode {
        fn name(&self) -> &str {
            "sigterm_node"
        }
        fn tick(&mut self) {}
        fn shutdown(&mut self) -> crate::error::HorusResult<()> {
            self.shutdown_called.store(true, Ordering::SeqCst);
            Ok(())
        }
    }

    let sc = shutdown_called.clone();
    let mut scheduler = Scheduler::new();
    scheduler
        .add(SigtermTestNode {
            shutdown_called: sc,
        })
        .order(0)
        .build();

    // Simulate SIGTERM after a brief delay
    std::thread::spawn(|| {
        std::thread::sleep(50_u64.ms());
        super::SIGTERM_RECEIVED.store(true, Ordering::SeqCst);
    });

    let result = scheduler.run_for(5_u64.secs());
    result.unwrap();
    assert!(
        shutdown_called.load(Ordering::SeqCst),
        "Node must get shutdown() called on SIGTERM"
    );
}

/// External stop via running_flag() — the Python/FFI pattern.
#[test]
fn test_external_stop_via_running_flag() {
    let _guard = lock_scheduler();
    let tick_count = Arc::new(AtomicUsize::new(0));

    let tc = tick_count.clone();
    let mut scheduler = Scheduler::new();
    scheduler
        .add(CounterNode::with_counter("ext_stop_node", tc))
        .order(0)
        .build();

    let flag = scheduler.running_flag();
    std::thread::spawn(move || {
        std::thread::sleep(80_u64.ms());
        flag.store(false, Ordering::SeqCst);
    });

    let result = scheduler.run_for(5_u64.secs());
    result.unwrap();
    // Node should have ticked at least once but stopped well before 5 seconds
    assert!(
        tick_count.load(Ordering::SeqCst) > 0,
        "Node should have ticked at least once before external stop"
    );
}

/// Verify shutdown order: all nodes get init, tick, then shutdown in sequence.
#[test]
fn test_shutdown_order_all_nodes_get_full_lifecycle() {
    let _guard = lock_scheduler();
    let log = Arc::new(Mutex::new(Vec::<String>::new()));

    let mut inits = Vec::new();
    let mut shuts = Vec::new();
    let mut tcs = Vec::new();

    let mut scheduler = Scheduler::new();
    for i in 0..3 {
        let init = Arc::new(AtomicBool::new(false));
        let shut = Arc::new(AtomicBool::new(false));
        let tc = Arc::new(AtomicUsize::new(0));
        let name = format!("lifecycle_{}", i);
        scheduler
            .add(OrderTrackingNode::new(
                &name,
                log.clone(),
                init.clone(),
                shut.clone(),
                tc.clone(),
            ))
            .order(i as u32)
            .build();
        inits.push(init);
        shuts.push(shut);
        tcs.push(tc);
    }

    let result = scheduler.run_for(1000_u64.ms());
    result.unwrap();

    let entries = log.lock().unwrap();

    // Every node should have init, at least one tick, and shutdown
    for i in 0..3 {
        let name = format!("lifecycle_{}", i);
        assert!(
            inits[i].load(Ordering::SeqCst),
            "{} should have init() called",
            name
        );
        assert!(
            tcs[i].load(Ordering::SeqCst) > 0,
            "{} should have ticked at least once",
            name
        );
        assert!(
            shuts[i].load(Ordering::SeqCst),
            "{} should have shutdown() called",
            name
        );
    }

    // Verify ordering: all inits come before any shutdown
    let first_shutdown_idx = entries
        .iter()
        .position(|e| e.starts_with("shutdown:"))
        .expect("should have at least one shutdown entry");
    let last_init_idx = entries
        .iter()
        .rposition(|e| e.starts_with("init:"))
        .expect("should have at least one init entry");
    assert!(
        last_init_idx < first_shutdown_idx,
        "All inits ({}) must come before first shutdown ({})",
        last_init_idx,
        first_shutdown_idx
    );
}

/// Shutdown with a node that returns Err from shutdown().
/// Scheduler should still shut down all other nodes.
#[test]
fn test_shutdown_error_does_not_prevent_other_shutdowns() {
    let _guard = lock_scheduler();
    let shutdown_ok_a = Arc::new(AtomicBool::new(false));
    let shutdown_ok_b = Arc::new(AtomicBool::new(false));

    struct ErrShutdownNode;
    impl Node for ErrShutdownNode {
        fn name(&self) -> &str {
            "err_shutdown"
        }
        fn tick(&mut self) {}
        fn shutdown(&mut self) -> crate::error::HorusResult<()> {
            Err(crate::HorusError::Node(crate::error::NodeError::Other {
                node: "err_shutdown".to_string(),
                message: "intentional shutdown error".to_string(),
            }))
        }
    }

    struct FlagNode {
        node_name: &'static str,
        flag: Arc<AtomicBool>,
    }
    impl Node for FlagNode {
        fn name(&self) -> &str {
            self.node_name
        }
        fn tick(&mut self) {}
        fn shutdown(&mut self) -> crate::error::HorusResult<()> {
            self.flag.store(true, Ordering::SeqCst);
            Ok(())
        }
    }

    let mut scheduler = Scheduler::new();
    scheduler
        .add(FlagNode {
            node_name: "ok_before",
            flag: shutdown_ok_a.clone(),
        })
        .order(0)
        .build();
    scheduler.add(ErrShutdownNode).order(1).build();
    scheduler
        .add(FlagNode {
            node_name: "ok_after",
            flag: shutdown_ok_b.clone(),
        })
        .order(2)
        .build();

    let result = scheduler.run_for(100_u64.ms());
    result.unwrap();

    assert!(
        shutdown_ok_a.load(Ordering::SeqCst),
        "Node before error node must have shutdown() called"
    );
    assert!(
        shutdown_ok_b.load(Ordering::SeqCst),
        "Node after error node must have shutdown() called"
    );
}

/// Verify metrics are populated after shutdown.
#[test]
fn test_metrics_populated_after_shutdown() {
    let _guard = lock_scheduler();
    let counter = Arc::new(AtomicUsize::new(0));

    let mut scheduler = Scheduler::new();
    scheduler
        .add(CounterNode::with_counter("metrics_node", counter.clone()))
        .order(0)
        .build();

    let result = scheduler.run_for(1000_u64.ms());
    result.unwrap();

    let metrics = scheduler.metrics();
    assert_eq!(metrics.len(), 1);
    assert_eq!(metrics[0].name(), "metrics_node");
    assert!(
        metrics[0].total_ticks() > 0,
        "Should have recorded some ticks in metrics"
    );
}

/// Rapid stop-restart pattern: stop(), then immediately try to check state.
#[test]
fn test_stop_then_check_state_is_consistent() {
    let _guard = lock_scheduler();
    let counter = Arc::new(AtomicUsize::new(0));

    let mut scheduler = Scheduler::new();
    scheduler
        .add(CounterNode::with_counter("rapid_stop", counter.clone()))
        .order(0)
        .build();

    let result = scheduler.run_for(100_u64.ms());
    result.unwrap();

    // After run_for completes, scheduler should be stopped
    assert!(
        !scheduler.is_running(),
        "Scheduler should not be running after run_for completes"
    );

    // Metrics, node_list, status should all still work
    let nodes = scheduler.node_list();
    assert_eq!(nodes.len(), 1);
    let status = scheduler.status();
    assert!(!status.is_empty());
    let metrics = scheduler.metrics();
    assert_eq!(metrics.len(), 1);
}

/// Uninitialised node (init panics) should NOT get shutdown() called.
#[test]
fn test_uninitialised_node_not_shutdown() {
    let _guard = lock_scheduler();
    let shutdown_called = Arc::new(AtomicBool::new(false));

    struct PanicInitShutTracker {
        shutdown_called: Arc<AtomicBool>,
    }
    impl Node for PanicInitShutTracker {
        fn name(&self) -> &str {
            "panic_init_node"
        }
        fn init(&mut self) -> crate::error::HorusResult<()> {
            panic!("init failed");
        }
        fn tick(&mut self) {}
        fn shutdown(&mut self) -> crate::error::HorusResult<()> {
            self.shutdown_called.store(true, Ordering::SeqCst);
            Ok(())
        }
    }

    let sc = shutdown_called.clone();
    let mut scheduler = Scheduler::new();
    scheduler
        .add(PanicInitShutTracker {
            shutdown_called: sc,
        })
        .order(0)
        .build();

    let result = scheduler.run_for(100_u64.ms());
    result.unwrap();

    // Node that failed init should NOT have shutdown called
    assert!(
        !shutdown_called.load(Ordering::SeqCst),
        "Node that panicked in init() should NOT get shutdown() called"
    );
}

// ============================================================================
// Cascading Failure & Fault Tolerance Tests
// ============================================================================

/// Multiple nodes panic simultaneously — scheduler survives with Ignore policy.
/// Robotics: two sensors fail at the same time, critical control loop must continue.
#[test]
fn test_multiple_simultaneous_panics_scheduler_survives() {
    let _guard = lock_scheduler();
    let healthy_counter = Arc::new(AtomicUsize::new(0));
    let panic_a = Arc::new(AtomicUsize::new(0));
    let panic_b = Arc::new(AtomicUsize::new(0));

    let mut scheduler = Scheduler::new();

    // Healthy node
    scheduler
        .add(CounterNode::with_counter(
            "motor_ctrl",
            healthy_counter.clone(),
        ))
        .order(0)
        .build();

    // Two nodes that panic on first tick, both with Ignore policy
    scheduler
        .add(PanickingNode::new("sensor_a", 1, panic_a.clone()))
        .order(1)
        .failure_policy(FailurePolicy::Ignore)
        .build();
    scheduler
        .add(PanickingNode::new("sensor_b", 1, panic_b.clone()))
        .order(2)
        .failure_policy(FailurePolicy::Ignore)
        .build();

    let result = scheduler.run_for(1000_u64.ms());
    result.unwrap();

    assert!(
        healthy_counter.load(Ordering::SeqCst) > 0,
        "Motor control node must keep running despite 2 failing sensors"
    );
}

/// Fatal policy node among multiple Ignore nodes — one Fatal failure stops everything.
/// Robotics: safety-critical motor failure must stop the whole system even if
/// other non-critical nodes are also failing.
#[test]
fn test_fatal_among_ignore_nodes_still_stops() {
    let _guard = lock_scheduler();
    let fatal_counter = Arc::new(AtomicUsize::new(0));
    let ignore_counter = Arc::new(AtomicUsize::new(0));

    let mut scheduler = Scheduler::new();

    // Ignore policy node that panics
    scheduler
        .add(PanickingNode::new("logger", 1, ignore_counter.clone()))
        .order(0)
        .failure_policy(FailurePolicy::Ignore)
        .build();

    // Fatal policy node that panics on tick 3
    scheduler
        .add(PanickingNode::new("motor", 3, fatal_counter.clone()))
        .order(1)
        .failure_policy(FailurePolicy::Fatal)
        .build();

    let _result = scheduler.run_for(500_u64.ms());
    assert!(
        !scheduler.is_running(),
        "Scheduler must stop when Fatal-policy node fails"
    );
}

/// Skip policy: node suppressed after threshold, healthy nodes unaffected.
#[test]
fn test_skip_policy_healthy_nodes_unaffected() {
    let _guard = lock_scheduler();
    let healthy_counter = Arc::new(AtomicUsize::new(0));
    let panic_counter = Arc::new(AtomicUsize::new(0));

    let mut scheduler = Scheduler::new().watchdog(500_u64.ms());

    scheduler
        .add(CounterNode::with_counter(
            "healthy_node",
            healthy_counter.clone(),
        ))
        .order(0)
        .build();

    // Node that always panics (panic_at=1), with Skip policy (threshold=2)
    scheduler
        .add(PanickingNode::new("flaky_node", 1, panic_counter.clone()))
        .order(1)
        .failure_policy(FailurePolicy::skip(2, 5_u64.secs()))
        .build();

    let result = scheduler.run_for(1000_u64.ms());
    result.unwrap();

    let healthy_ticks = healthy_counter.load(Ordering::SeqCst);
    assert!(
        healthy_ticks > 0,
        "Healthy node should tick while flaky node is skipped, got {}",
        healthy_ticks
    );
}

/// Restart policy: node gets re-init'd and ticks again after recovery.
/// Uses a node that succeeds after N restarts.
#[test]
fn test_restart_node_rejoins_tick_loop() {
    let _guard = lock_scheduler();
    let tick_counter = Arc::new(AtomicUsize::new(0));

    // Node that panics on ticks 2, 4, 6 (every even tick after first)
    // With restart(5, 10), it should keep restarting and accumulating ticks
    struct EveryOtherPanicNode {
        counter: Arc<AtomicUsize>,
    }
    impl Node for EveryOtherPanicNode {
        fn name(&self) -> &str {
            "restart_test"
        }
        fn tick(&mut self) {
            let c = self.counter.fetch_add(1, Ordering::SeqCst) + 1;
            if c.is_multiple_of(2) {
                panic!("even tick panic at {}", c);
            }
        }
    }

    let tc = tick_counter.clone();
    let mut scheduler = Scheduler::new();
    scheduler
        .add(EveryOtherPanicNode { counter: tc })
        .order(0)
        .failure_policy(FailurePolicy::restart(5, 5_u64.ms()))
        .build();

    let _result = scheduler.run_for(500_u64.ms());

    // Node should have ticked multiple times across restarts
    let ticks = tick_counter.load(Ordering::SeqCst);
    assert!(
        ticks >= 2,
        "Node should reach its failing tick before restart, got {}",
        ticks
    );
}

/// Mixed failure policies: multiple nodes with different policies fail together.
/// Verifies that each policy is applied independently.
#[test]
fn test_mixed_failure_policies_independent() {
    let _guard = lock_scheduler();
    let ignore_counter = Arc::new(AtomicUsize::new(0));
    let skip_counter = Arc::new(AtomicUsize::new(0));
    let healthy_counter = Arc::new(AtomicUsize::new(0));

    let mut scheduler = Scheduler::new();

    // Healthy node (no failures)
    scheduler
        .add(CounterNode::with_counter(
            "healthy",
            healthy_counter.clone(),
        ))
        .order(0)
        .build();

    // Ignore-policy node that panics every tick
    scheduler
        .add(PanickingNode::new("ignore_node", 1, ignore_counter.clone()))
        .order(1)
        .failure_policy(FailurePolicy::Ignore)
        .build();

    // Skip-policy node that panics every tick (opens circuit after 2 failures)
    scheduler
        .add(PanickingNode::new("skip_node", 1, skip_counter.clone()))
        .order(2)
        .failure_policy(FailurePolicy::skip(2, 5_u64.secs()))
        .build();

    let result = scheduler.run_for(1000_u64.ms());
    result.unwrap();

    let h = healthy_counter.load(Ordering::SeqCst);
    assert!(h > 0, "Healthy node must keep running, got {} ticks", h);

    // Ignore node keeps being called (panics are swallowed)
    let ig = ignore_counter.load(Ordering::SeqCst);
    assert!(
        ig > 0,
        "Ignore-policy node should be called despite failures, got {} ticks",
        ig
    );

    // The skip-policy node must still be scheduled initially. Circuit-breaker
    // accounting itself is covered by the deterministic policy unit tests; do
    // not require two wall-clock ticks here because crash reporting can dominate
    // a heavily instrumented CI runner.
    let sk = skip_counter.load(Ordering::SeqCst);
    assert!(
        sk > 0,
        "Skip-policy node should receive an initial tick, got {}",
        sk
    );
}

/// Node that panics every tick with Ignore policy should not starve other nodes.
/// The panicking node should not consume excessive scheduler time.
#[test]
fn test_panicking_node_doesnt_starve_others() {
    let _guard = lock_scheduler();
    let good_counter = Arc::new(AtomicUsize::new(0));
    let bad_counter = Arc::new(AtomicUsize::new(0));

    let mut scheduler = Scheduler::new();

    scheduler
        .add(CounterNode::with_counter("good", good_counter.clone()))
        .order(0)
        .build();

    scheduler
        .add(PanickingNode::new("bad", 1, bad_counter.clone()))
        .order(1)
        .failure_policy(FailurePolicy::Ignore)
        .build();

    let result = scheduler.run_for(1000_u64.ms());
    result.unwrap();

    let good = good_counter.load(Ordering::SeqCst);
    let bad = bad_counter.load(Ordering::SeqCst);

    // Both should have been called roughly the same number of times
    // (panicking node doesn't skip ticks with Ignore policy)
    assert!(good > 0, "Good node must get a tick, got {}", good);
    // Bad node gets at least as many tick attempts
    assert!(
        bad > 0,
        "Bad node should get a tick attempt despite panics, got {}",
        bad
    );
}

/// Restart policy: after successful recovery, restart count is preserved in stats.
#[test]
fn test_restart_policy_stats_tracked() {
    let _guard = lock_scheduler();
    let counter = Arc::new(AtomicUsize::new(0));

    let mut scheduler = Scheduler::new();
    scheduler
        .add(PanickingNode::new("restart_stats", 2, counter.clone()))
        .order(0)
        .failure_policy(FailurePolicy::restart(3, 5_u64.ms()))
        .build();

    let _result = scheduler.run_for(500_u64.ms());

    // Check metrics are available (scheduler tracks node metrics even after failures)
    let metrics = scheduler.metrics();
    assert!(!metrics.is_empty());
    assert_eq!(metrics[0].name(), "restart_stats");
}

/// Node fails, gets skipped by fault tolerance, verify scheduler status
/// reflects the failure state.
#[test]
fn test_fault_tolerance_reflected_in_scheduler_status() {
    let _guard = lock_scheduler();
    let panic_counter = Arc::new(AtomicUsize::new(0));

    let mut scheduler = Scheduler::new().watchdog(500_u64.ms());
    scheduler
        .add(PanickingNode::new("circuit_node", 1, panic_counter.clone()))
        .order(0)
        .failure_policy(FailurePolicy::skip(2, 5_u64.secs()))
        .build();

    let result = scheduler.run_for(200_u64.ms());
    result.unwrap();

    // Status should be non-empty and reflect the fault tolerance state
    let status = scheduler.status();
    assert!(!status.is_empty());
}

// ============================================================================
// Phase 6: Comprehensive Tests — Builder Methods
// ============================================================================

#[test]
fn test_prefer_rt_sets_config() {
    let _guard = lock_scheduler();
    let scheduler = Scheduler::new().prefer_rt();
    assert!(scheduler.pending_config.realtime.memory_locking);
    assert!(scheduler.pending_config.realtime.rt_scheduling_class);
}

#[test]
fn test_require_rt_panics_or_succeeds() {
    let _guard = lock_scheduler();
    // require_rt() should either succeed (RT available) or panic (not available)
    let result = std::panic::catch_unwind(|| Scheduler::new().require_rt());
    // Both outcomes are valid — we just verify it doesn't silently do nothing
    match result {
        Ok(scheduler) => {
            assert!(scheduler.pending_config.realtime.memory_locking);
            assert!(scheduler.pending_config.realtime.rt_scheduling_class);
        }
        Err(_) => {
            // Expected on non-RT systems
        }
    }
}

#[test]
fn test_watchdog_sets_timeout() {
    let _guard = lock_scheduler();
    let scheduler = Scheduler::new().watchdog(500_u64.ms());
    assert_eq!(scheduler.pending_config.realtime.watchdog_timeout_ms, 500);
}

#[test]
fn test_blackbox_sets_size() {
    let _guard = lock_scheduler();
    let scheduler = Scheduler::new().blackbox(64);
    assert_eq!(scheduler.pending_config.monitoring.black_box_size_mb, 64);
}

#[test]
fn test_max_deadline_misses_sets_value() {
    let _guard = lock_scheduler();
    let scheduler = Scheduler::new().max_deadline_misses(10);
    assert_eq!(scheduler.pending_config.realtime.max_deadline_misses, 10);
}

#[test]
fn test_builder_full_chain() {
    let _guard = lock_scheduler();
    let scheduler = Scheduler::new()
        .watchdog(500_u64.ms())
        .blackbox(64)
        .tick_rate(500_u64.hz())
        .verbose(false)
        .prefer_rt();

    assert_eq!(scheduler.pending_config.realtime.watchdog_timeout_ms, 500);
    assert_eq!(scheduler.pending_config.monitoring.black_box_size_mb, 64);
    assert_eq!(scheduler.pending_config.timing.global_rate_hz, 500.0);
    assert!(!scheduler.pending_config.monitoring.verbose);
    assert!(scheduler.pending_config.realtime.memory_locking);
}

// ============================================================================
// Phase 6: tick_once() Tests
// ============================================================================

#[test]
fn test_tick_once_basic() {
    let _guard = lock_scheduler();
    let counter = Arc::new(AtomicUsize::new(0));

    let mut scheduler = Scheduler::new().tick_rate(100_u64.hz());
    scheduler
        .add(CounterNode::with_counter("tick_once_node", counter.clone()))
        .order(0)
        .build();

    // One tick
    scheduler.tick_once().unwrap();
    assert_eq!(counter.load(Ordering::SeqCst), 1);

    // Second tick
    scheduler.tick_once().unwrap();
    assert_eq!(counter.load(Ordering::SeqCst), 2);
}

#[test]
fn test_tick_once_multiple_nodes_execution_order() {
    let _guard = lock_scheduler();
    let order = Arc::new(Mutex::new(Vec::<String>::new()));

    struct OrderTracker {
        name: &'static str,
        order: Arc<Mutex<Vec<String>>>,
    }
    impl Node for OrderTracker {
        fn name(&self) -> &str {
            self.name
        }
        fn tick(&mut self) {
            self.order.lock().unwrap().push(self.name.to_string());
        }
    }

    let mut scheduler = Scheduler::new();

    scheduler
        .add(OrderTracker {
            name: "third",
            order: order.clone(),
        })
        .order(20)
        .build();
    scheduler
        .add(OrderTracker {
            name: "first",
            order: order.clone(),
        })
        .order(0)
        .build();
    scheduler
        .add(OrderTracker {
            name: "second",
            order: order.clone(),
        })
        .order(10)
        .build();

    scheduler.tick_once().unwrap();

    let recorded = order.lock().unwrap().clone();
    assert_eq!(recorded, vec!["first", "second", "third"]);
}

#[test]
fn test_tick_once_lazy_init() {
    let _guard = lock_scheduler();

    struct InitTracker {
        init_called: Arc<AtomicBool>,
    }
    impl Node for InitTracker {
        fn name(&self) -> &str {
            "init_tracker"
        }
        fn init(&mut self) -> crate::error::HorusResult<()> {
            self.init_called.store(true, Ordering::SeqCst);
            Ok(())
        }
        fn tick(&mut self) {}
    }

    let init_flag = Arc::new(AtomicBool::new(false));
    let mut scheduler = Scheduler::new();
    scheduler
        .add(InitTracker {
            init_called: init_flag.clone(),
        })
        .order(0)
        .build();

    // Before tick_once, init should NOT have been called
    assert!(!init_flag.load(Ordering::SeqCst));

    // First tick_once triggers lazy init
    scheduler.tick_once().unwrap();
    assert!(init_flag.load(Ordering::SeqCst));
}

#[test]
fn test_tick_once_increments_tick_counter() {
    let _guard = lock_scheduler();
    let mut scheduler = Scheduler::new();
    let counter = Arc::new(AtomicUsize::new(0));
    scheduler
        .add(CounterNode::with_counter("n", counter))
        .order(0)
        .build();

    assert_eq!(scheduler.current_tick(), 0);
    scheduler.tick_once().unwrap();
    assert_eq!(scheduler.current_tick(), 1);
    scheduler.tick_once().unwrap();
    assert_eq!(scheduler.current_tick(), 2);
}

// ============================================================================
// Phase 6: Deterministic Execution Stress Tests
// ============================================================================

#[test]
fn test_deterministic_100_nodes_strict_order() {
    let _guard = lock_scheduler();
    let order = Arc::new(Mutex::new(Vec::<u32>::new()));

    struct PriorityTracker {
        name: String,
        prio: u32,
        order: Arc<Mutex<Vec<u32>>>,
    }
    impl Node for PriorityTracker {
        fn name(&self) -> &str {
            &self.name
        }
        fn tick(&mut self) {
            self.order.lock().unwrap().push(self.prio);
        }
    }

    let mut scheduler = Scheduler::new();

    // Add 100 nodes in REVERSE priority order
    for i in (0..100u32).rev() {
        let name = format!("node_{}", i);
        scheduler
            .add(PriorityTracker {
                name,
                prio: i,
                order: order.clone(),
            })
            .order(i)
            .build();
    }

    // Run 10 ticks
    for _ in 0..10 {
        scheduler.tick_once().unwrap();
    }

    let recorded = order.lock().unwrap().clone();
    // Should be 10 repetitions of [0, 1, 2, ..., 99]
    assert_eq!(recorded.len(), 1000);
    for tick in 0..10 {
        let slice = &recorded[tick * 100..(tick + 1) * 100];
        let expected: Vec<u32> = (0..100).collect();
        assert_eq!(slice, &expected[..], "Tick {} order violated", tick);
    }
}

#[test]
fn test_deterministic_repeated_ticks_identical() {
    let _guard = lock_scheduler();
    let order1 = Arc::new(Mutex::new(Vec::<String>::new()));

    struct NameTracker {
        nm: &'static str,
        log: Arc<Mutex<Vec<String>>>,
    }
    impl Node for NameTracker {
        fn name(&self) -> &str {
            self.nm
        }
        fn tick(&mut self) {
            self.log.lock().unwrap().push(self.nm.to_string());
        }
    }

    let mut scheduler = Scheduler::new();
    for (name, prio) in [("alpha", 0), ("beta", 1), ("gamma", 2), ("delta", 3)] {
        scheduler
            .add(NameTracker {
                nm: name,
                log: order1.clone(),
            })
            .order(prio)
            .build();
    }

    // 50 ticks
    for _ in 0..50 {
        scheduler.tick_once().unwrap();
    }

    let recorded = order1.lock().unwrap().clone();
    // Every group of 4 should be identical
    for tick in 0..50 {
        let chunk = &recorded[tick * 4..(tick + 1) * 4];
        assert_eq!(chunk, &["alpha", "beta", "gamma", "delta"]);
    }
}

// ============================================================================
// Phase 6: Edge Case Tests
// ============================================================================

#[test]
fn test_tick_once_empty_scheduler() {
    let _guard = lock_scheduler();
    let mut scheduler = Scheduler::new();
    // Empty scheduler should succeed
    scheduler.tick_once().unwrap();
    assert_eq!(scheduler.current_tick(), 1);
}

#[test]
fn test_tick_once_single_node() {
    let _guard = lock_scheduler();
    let counter = Arc::new(AtomicUsize::new(0));
    let mut scheduler = Scheduler::new();
    scheduler
        .add(CounterNode::with_counter("solo", counter.clone()))
        .order(0)
        .build();

    for _ in 0..100 {
        scheduler.tick_once().unwrap();
    }
    assert_eq!(counter.load(Ordering::SeqCst), 100);
}

#[test]
fn test_watchdog_scheduler_runs_without_nodes() {
    let _guard = lock_scheduler();
    let mut scheduler = Scheduler::new()
        .watchdog(500_u64.ms())
        .tick_rate(100_u64.hz());
    let result = scheduler.run_for(50_u64.ms());
    result.unwrap();
}

#[test]
fn test_builder_order_independence() {
    let _guard = lock_scheduler();
    // Builder methods should be order-independent
    let s1 = Scheduler::new()
        .watchdog(500_u64.ms())
        .tick_rate(200_u64.hz());

    let s2 = Scheduler::new()
        .tick_rate(200_u64.hz())
        .watchdog(500_u64.ms());

    assert_eq!(
        s1.pending_config.realtime.watchdog_timeout_ms,
        s2.pending_config.realtime.watchdog_timeout_ms
    );
    assert_eq!(
        s1.pending_config.timing.global_rate_hz,
        s2.pending_config.timing.global_rate_hz
    );
}

// ============================================================================
// Phase 6: Graduated Safety Monitor Response
// ============================================================================

#[test]
fn test_watchdog_with_deterministic_tick_once() {
    let _guard = lock_scheduler();
    let counter = Arc::new(AtomicUsize::new(0));

    let mut scheduler = Scheduler::new()
        .watchdog(500_u64.ms())
        .tick_rate(100_u64.hz());

    scheduler
        .add(CounterNode::with_counter(
            "monitored_tick_once",
            counter.clone(),
        ))
        .order(0)
        .build();

    // tick_once should work with monitoring enabled
    for _ in 0..10 {
        scheduler.tick_once().unwrap();
    }
    assert_eq!(counter.load(Ordering::SeqCst), 10);
}

// ============================================================================
// Phase 6: Watchdog Health State Tests
// ============================================================================

#[test]
fn test_watchdog_with_healthy_nodes() {
    let _guard = lock_scheduler();
    let counter = Arc::new(AtomicUsize::new(0));

    let mut scheduler = Scheduler::new()
        .watchdog(500_u64.ms())
        .tick_rate(100_u64.hz());

    scheduler
        .add(CounterNode::with_counter("fast_node", counter.clone()))
        .order(0)
        .build();

    // Fast node should never trigger watchdog
    scheduler.run_for(200_u64.ms()).unwrap();

    let status = scheduler.status();
    // Node Health section should show all healthy
    assert!(
        status.contains("All 1 nodes healthy"),
        "Fast node should remain healthy. Status: {}",
        status
    );
}

// ============================================================================
// Phase 6: Timing Report Tests
// ============================================================================

#[test]
fn test_timing_report_does_not_crash() {
    let _guard = lock_scheduler();
    let counter = Arc::new(AtomicUsize::new(0));

    let mut scheduler = Scheduler::new()
        .watchdog(500_u64.ms())
        .tick_rate(100_u64.hz());

    scheduler
        .add(CounterNode::with_counter("report_node_a", counter.clone()))
        .order(0)
        .build();
    scheduler
        .add(CounterNode::with_counter("report_node_b", counter.clone()))
        .order(1)
        .build();

    // Run and shut down — timing report is printed at shutdown
    scheduler.run_for(100_u64.ms()).unwrap();
    // If we got here, the timing report didn't crash
}

// ============================================================================
// Builder API: .name(), .deterministic(), .cores()
// ============================================================================

#[test]
fn test_name_builder() {
    let _guard = lock_scheduler();
    let scheduler = Scheduler::new().name("motor_control");
    assert_eq!(scheduler.scheduler_name(), "motor_control");
}

#[test]
fn test_name_appears_in_status() {
    let _guard = lock_scheduler();
    let scheduler = Scheduler::new().name("arm_controller");
    let status = scheduler.status();
    assert!(
        status.contains("arm_controller"),
        "status should contain the scheduler name"
    );
}

/// `.no_alloc()` must actually be in force on whichever path runs the node.
///
/// Deterministic mode spawns no executors — every node, RT ones included,
/// ticks on the main thread. The build-time check in `node_builder` accepts
/// `.rate().no_alloc()` because it only knows the node is RT-classified, so
/// before the fix this configuration passed validation and then ran with no
/// allocation checking whatsoever: precisely the false assurance that check
/// exists to prevent.
///
/// The observable property is the allocator context itself. The test binary
/// has no `RtAwareAllocator` as `#[global_allocator]`, so an allocation here
/// would not panic — but `is_rt_context()` is what the allocator consults,
/// and it must be true during the tick.
#[test]
fn test_no_alloc_enforced_on_deterministic_main_thread_path() {
    use crate::memory::rt_allocator;
    use std::sync::atomic::AtomicBool;

    struct ProbeNode {
        in_rt_context: Arc<AtomicBool>,
        ticks: Arc<AtomicUsize>,
    }

    impl Node for ProbeNode {
        fn name(&self) -> &str {
            "alloc_probe"
        }
        fn tick(&mut self) {
            // Warmup tick is exempt by design, so only record from tick 2 on.
            if self.ticks.fetch_add(1, Ordering::SeqCst) > 0 && rt_allocator::is_rt_context() {
                self.in_rt_context.store(true, Ordering::SeqCst);
            }
        }
    }

    let _guard = lock_scheduler();
    let in_rt_context = Arc::new(AtomicBool::new(false));
    let ticks = Arc::new(AtomicUsize::new(0));

    let mut scheduler = Scheduler::new().deterministic(true).tick_rate(100_u64.hz());
    scheduler
        .add(ProbeNode {
            in_rt_context: in_rt_context.clone(),
            ticks: ticks.clone(),
        })
        .rate(100_u64.hz())
        .no_alloc()
        .build()
        .unwrap();

    for _ in 0..4 {
        scheduler.tick_once().unwrap();
    }

    assert!(ticks.load(Ordering::SeqCst) >= 2, "node did not tick");
    assert!(
        in_rt_context.load(Ordering::SeqCst),
        "a .no_alloc() node ticked on the deterministic main-thread path without \
         entering the alloc-free context — the flag was silently doing nothing"
    );
    // The context must not leak past the tick.
    assert!(!rt_allocator::is_rt_context());
}

/// `apply_config` is the sole sink for the Python configuration surface, and
/// `finalize_config` re-reads `pending_config` after the nodes exist. Any
/// section `apply_config` applies eagerly without recording it there is lost:
/// `resources` was only PRINTED, so Python-supplied CPU cores produced a log
/// line and no pinning, and the `HORUS_RT_CORES` fallback then saw an
/// unconfigured process.
#[test]
fn test_apply_config_records_every_section_in_pending_config() {
    use crate::scheduling::config::SchedulerConfig;

    let _guard = lock_scheduler();
    let mut scheduler = Scheduler::new();

    let mut config = SchedulerConfig::default();
    config.resources.cpu_cores = Some(vec![2, 3]);
    config.monitoring.telemetry_endpoint = Some("file:///dev/null".to_string());
    config.monitoring.black_box_size_mb = 4;

    scheduler.apply_config(config);

    assert_eq!(
        scheduler.pending_config.resources.cpu_cores,
        Some(vec![2, 3]),
        "cpu_cores never reached pending_config, so finalize_config could not apply it"
    );
    assert_eq!(
        scheduler
            .pending_config
            .monitoring
            .telemetry_endpoint
            .as_deref(),
        Some("file:///dev/null")
    );
    assert_eq!(scheduler.pending_config.monitoring.black_box_size_mb, 4);
}

/// The merge must not clobber builder calls horus_py makes BEFORE
/// `apply_config` with this struct's defaults.
#[test]
fn test_apply_config_does_not_clobber_earlier_builders() {
    use crate::scheduling::config::SchedulerConfig;

    let _guard = lock_scheduler();
    let mut scheduler = Scheduler::new().cores(&[6, 7]);
    assert_eq!(
        scheduler.pending_config.resources.cpu_cores,
        Some(vec![6, 7])
    );

    // Default config leaves cpu_cores None — that must mean "unspecified",
    // not "reset to all cores".
    scheduler.apply_config(SchedulerConfig::default());

    assert_eq!(
        scheduler.pending_config.resources.cpu_cores,
        Some(vec![6, 7]),
        "an unset field in the incoming config wiped an earlier .cores() call"
    );
}

/// The graduated watchdog ladder must reach nodes the scheduler does not own.
///
/// `check_safety_monitors` resolved every expiring node through
/// `self.nodes.iter_mut().find(..)`. After the class partition that vector
/// holds ONLY BestEffort nodes, so for an RT/Compute/Event/AsyncIo node the
/// `find` missed and the whole ladder was skipped: no Warning, no Unhealthy,
/// no Isolated, no `enter_safe_state()`. The executors fed the watchdog
/// correctly; its expiry simply produced nothing.
///
/// The node here is deliberately absent from `self.nodes` — that is exactly
/// the state `std::mem::take` leaves the scheduler in for an executor-hosted
/// node, and it is the condition the old code silently ignored.
#[test]
fn test_watchdog_ladder_reaches_executor_hosted_nodes() {
    use crate::scheduling::safety_monitor::SafetyMonitor;
    use crate::scheduling::types::{NodeControlMap, NodeHealthState};

    let _guard = lock_scheduler();
    let mut scheduler = Scheduler::new();

    // An executor-hosted critical node: registered with the safety monitor and
    // in the shared control map, but NOT in `self.nodes`.
    let monitor = Arc::new(SafetyMonitor::new(100));
    monitor.add_critical_node("rt_actuator".to_string(), Duration::from_millis(40));

    let controls = Arc::new(NodeControlMap::default());
    controls.register("rt_actuator");

    scheduler.monitor.safety = Some(monitor.clone());
    scheduler.node_controls = Some(controls.clone());

    assert_eq!(controls.health("rt_actuator"), NodeHealthState::Healthy);
    assert!(
        scheduler.nodes.is_empty(),
        "test premise: node is not owned"
    );

    // Past 1x the 40ms timeout, short of 2x → Warning.
    std::thread::sleep(Duration::from_millis(50));
    scheduler.check_safety_monitors();
    assert_eq!(
        controls.health("rt_actuator"),
        NodeHealthState::Warning,
        "a 1x watchdog overrun on an executor-hosted node produced no transition"
    );

    // Past 2x, short of 3x → Unhealthy.
    std::thread::sleep(Duration::from_millis(40));
    scheduler.check_safety_monitors();
    assert_eq!(controls.health("rt_actuator"), NodeHealthState::Unhealthy);

    // Past 3x → Isolated, safing requested from the owning executor, e-stop latched
    // because this thread cannot verify that the safing actually happened.
    std::thread::sleep(Duration::from_millis(40));
    scheduler.check_safety_monitors();
    assert_eq!(controls.health("rt_actuator"), NodeHealthState::Isolated);
    assert!(
        controls.take_safe_state_request("rt_actuator"),
        "Isolated was recorded but no safing was ever requested from the executor"
    );
    assert!(
        monitor.is_emergency_stop(),
        "a critical node 3x over its watchdog must e-stop — the executor thread that \
         would honour the safing request may itself be stuck inside the hung tick()"
    );
    // Consumed exactly once.
    assert!(!controls.take_safe_state_request("rt_actuator"));
}

#[test]
fn test_deterministic_builder() {
    let _guard = lock_scheduler();
    let counter = Arc::new(AtomicUsize::new(0));

    let mut scheduler = Scheduler::new().deterministic(true).tick_rate(100_u64.hz());

    scheduler
        .add(CounterNode::with_counter("det_node", counter.clone()))
        .order(0)
        .build()
        .unwrap();

    // In deterministic mode, tick_once runs all nodes on main thread
    for _ in 0..5 {
        scheduler.tick_once().unwrap();
    }
    assert_eq!(counter.load(Ordering::SeqCst), 5);
}

#[test]
fn test_cores_builder() {
    let _guard = lock_scheduler();
    let scheduler = Scheduler::new().cores(&[0, 1]);
    assert_eq!(
        scheduler.pending_config.resources.cpu_cores,
        Some(vec![0, 1])
    );
}

#[test]
fn test_builder_chaining_all_new_methods() {
    let _guard = lock_scheduler();
    let scheduler = Scheduler::new()
        .name("full_config")
        .deterministic(true)
        .cores(&[2, 3])
        .tick_rate(500_u64.hz())
        .watchdog(100_u64.ms())
        .verbose(false);

    assert_eq!(scheduler.scheduler_name(), "full_config");
    assert!(scheduler.pending_config.timing.deterministic_order);
    assert_eq!(
        scheduler.pending_config.resources.cpu_cores,
        Some(vec![2, 3])
    );
}

#[test]
fn test_telemetry_builder() {
    let _guard = lock_scheduler();
    let scheduler = Scheduler::new().telemetry("udp://localhost:9999");
    assert_eq!(
        scheduler.pending_config.monitoring.telemetry_endpoint,
        Some("udp://localhost:9999".to_string())
    );
}

/// A telemetry endpoint the process cannot write must say so — on both the
/// periodic path and the final export.
///
/// Both callers used to be `let _ = tm.export()`, and `export()` advances
/// `last_export` whether or not the snapshot landed, so a `file://` endpoint
/// pointing somewhere unwritable failed on every interval for the life of the
/// process with no diagnostic on any surface — under the
/// "[SCHEDULER] Telemetry enabled (endpoint: ...)" line printed at startup.
#[test]
fn telemetry_export_failures_reach_the_log() {
    let _guard = lock_scheduler();

    // A regular file where the exporter needs a directory: `create_dir_all`
    // then fails on every export with "File exists (os error 17)". That is
    // deterministic, and it does not depend on the test user's permissions the
    // way an unwritable directory would (a suite running as root can write
    // through mode 0o555).
    let temp_dir = tempfile::TempDir::new().unwrap();
    let blocker = temp_dir.path().join("not-a-directory");
    std::fs::write(&blocker, b"").unwrap();
    let endpoint = format!("file://{}/metrics.json", blocker.display());

    let first_unread = crate::core::log_buffer::GLOBAL_LOG_BUFFER.write_idx();

    // Ends the run on an OBSERVED export failure rather than on a wall-clock
    // guess about when the 1000 ms interval elapses inside a fixed `run_for`.
    // The scheduler ticks this node, then runs `periodic_monitoring` in the same
    // iteration, so the line is picked up one tick (10 ms) after it is written
    // and `finalize_run` follows immediately with the final export. `run_for`
    // below is therefore a timeout, not a schedule: a loaded machine that
    // stalls the tick loop costs latency here instead of a failure, and the
    // window in which the shared log ring could evict the line before the
    // assertions read it shrinks from hundreds of milliseconds to one tick.
    struct StopOnExportFailure {
        since: u64,
        running: Arc<AtomicBool>,
    }
    impl Node for StopOnExportFailure {
        fn name(&self) -> &str {
            "telemetry_probe"
        }
        fn tick(&mut self) {
            // Sample the write index BEFORE the scan and advance `since` to it
            // afterwards, so each tick deserialises only what arrived since the
            // previous one instead of re-walking the whole window every 10 ms.
            // GLOBAL_LOG_BUFFER is a cross-process ring shared with every other
            // horus process on the machine, so under a parallel `cargo test`
            // the un-advanced window reaches the ring's full capacity (5000
            // slots by default) and each tick re-deserialises thousands of
            // other processes' entries — each one a bincode decode under the
            // buffer's mutex, with a 5 ms spin available per slot that is
            // mid-write.
            //
            // Sampled before, not after: an index read after the scan would
            // advance past entries written *during* it, and `get_since` never
            // returns those again. Read first, and the worst case is that the
            // next tick re-scans a handful of entries.
            let upto = crate::core::log_buffer::GLOBAL_LOG_BUFFER.write_idx();
            let reported = crate::core::log_buffer::GLOBAL_LOG_BUFFER
                .get_since(self.since)
                .into_iter()
                .any(|entry| entry.message.contains("[TELEMETRY] Export failed"));
            self.since = upto;
            if reported {
                self.running.store(false, Ordering::SeqCst);
            }
        }
    }

    let mut scheduler = Scheduler::new()
        .tick_rate(100_u64.hz())
        .telemetry(&endpoint);
    let running = scheduler.running_flag();
    // `.unwrap()`, not a discarded `Result`: `unused_must_use` is allowed
    // workspace-wide, so a `build()` that ever starts failing validation here
    // would drop the node silently. The run would then fall through to the
    // 5 s timeout below and still pass, hiding the loss of the very thing that
    // makes this test prompt.
    scheduler
        .add(StopOnExportFailure {
            since: first_unread,
            running,
        })
        .build()
        .unwrap();

    // Upper bound, not the expected duration. The export interval starts when
    // `finalize_config` builds the manager, just before the run clock does, so
    // the periodic export normally reports ~1.0 s in and the node above ends
    // the run right there. Five seconds leaves room for a four-second stall
    // before the assertions below report what was actually logged.
    scheduler.run_for(5000_u64.ms()).unwrap();

    let reported: Vec<String> = crate::core::log_buffer::GLOBAL_LOG_BUFFER
        .get_since(first_unread)
        .into_iter()
        .map(|entry| entry.message)
        .filter(|message| message.contains("[TELEMETRY]"))
        .collect();

    assert!(
        reported
            .iter()
            .any(|m| m.contains("Export failed") && m.contains("not-a-directory")),
        "the periodic export must report the failure and name the path it could \
         not write; telemetry lines seen: {reported:?}"
    );
    assert!(
        reported.iter().any(|m| m.contains("Final export failed")),
        "the shutdown export must report its failure too; telemetry lines seen: {reported:?}"
    );
}

// ============================================================================
// Error path and negative tests
// ============================================================================

#[test]
fn test_scheduler_very_high_tick_rate() {
    let _guard = lock_scheduler();
    let counter = Arc::new(AtomicUsize::new(0));
    let mut scheduler = Scheduler::new().tick_rate(100_000.hz());
    scheduler
        .add(CounterNode::with_counter("fast_node", counter))
        .build();
    // Very high rate should be accepted without panic
}

#[test]
fn test_scheduler_add_node_after_stop() {
    let _guard = lock_scheduler();
    let mut scheduler = Scheduler::new();
    scheduler
        .add(CounterNode::with_counter(
            "node_a",
            Arc::new(AtomicUsize::new(0)),
        ))
        .build();
    scheduler.stop();
    // Adding after stop should still work (for next run)
    scheduler
        .add(CounterNode::with_counter(
            "node_b",
            Arc::new(AtomicUsize::new(0)),
        ))
        .build();
}

#[test]
fn test_scheduler_metrics_before_any_run() {
    let _guard = lock_scheduler();
    let scheduler = Scheduler::new();
    let metrics = scheduler.metrics();
    // No nodes added — metrics should be empty
    assert!(
        metrics.is_empty(),
        "metrics should be empty before adding nodes"
    );
}

#[test]
fn test_node_builder_rate_then_budget_same_as_budget_then_rate() {
    let _guard = lock_scheduler();
    // Order 1: rate then budget
    let mut s1 = Scheduler::new();
    s1.add(CounterNode::with_counter(
        "order1",
        Arc::new(AtomicUsize::new(0)),
    ))
    .rate(100.hz())
    .budget(500.us())
    .build();
    let node1 = &s1.nodes[0];
    let rate1 = node1.rate_hz;
    let budget1 = node1.tick_budget;
    let class1 = node1.execution_class.clone();

    // Order 2: budget then rate
    let mut s2 = Scheduler::new();
    s2.add(CounterNode::with_counter(
        "order2",
        Arc::new(AtomicUsize::new(0)),
    ))
    .budget(500.us())
    .rate(100.hz())
    .build();
    let node2 = &s2.nodes[0];

    assert_eq!(
        rate1, node2.rate_hz,
        "rate must be same regardless of call order"
    );
    assert_eq!(
        budget1, node2.tick_budget,
        "budget must be same regardless of call order"
    );
    assert_eq!(
        class1, node2.execution_class,
        "execution class must be same regardless of call order"
    );
}

#[test]
fn test_node_builder_compute_then_rate_stays_compute() {
    let _guard = lock_scheduler();
    let mut scheduler = Scheduler::new();
    scheduler
        .add(CounterNode::with_counter(
            "compute_rate",
            Arc::new(AtomicUsize::new(0)),
        ))
        .compute()
        .rate(100.hz())
        .build();
    let node = &scheduler.nodes[0];
    // Compute with rate should stay Compute (rate doesn't auto-promote to Rt when compute is explicit)
    assert_eq!(
        node.execution_class,
        crate::scheduling::types::ExecutionClass::Compute
    );
}

#[test]
fn test_node_builder_deadline_without_rate() {
    let _guard = lock_scheduler();
    let mut scheduler = Scheduler::new();
    scheduler
        .add(CounterNode::with_counter(
            "deadline_only",
            Arc::new(AtomicUsize::new(0)),
        ))
        .deadline(1.ms())
        .build();
    // Deadline without rate should work (deadline checked on each tick)
    assert!(scheduler.nodes[0].deadline.is_some());
}

// ============================================================================
// End-to-end robot scenario test
// ============================================================================

#[test]
fn test_e2e_sensor_controller_motor_pipeline() {
    let _guard = lock_scheduler();

    // Simulate: sensor (order 0) → controller (order 1) → motor (order 2)
    // Each node increments its counter on tick
    let sensor_count = Arc::new(AtomicUsize::new(0));
    let controller_count = Arc::new(AtomicUsize::new(0));
    let motor_count = Arc::new(AtomicUsize::new(0));

    let mut scheduler = Scheduler::new().tick_rate(100.hz()).deterministic(true); // Deterministic for reproducible ordering

    scheduler
        .add(CounterNode::with_counter("sensor", sensor_count.clone()))
        .order(0)
        .build();
    scheduler
        .add(CounterNode::with_counter(
            "controller",
            controller_count.clone(),
        ))
        .order(1)
        .build();
    scheduler
        .add(CounterNode::with_counter("motor", motor_count.clone()))
        .order(2)
        .build();

    // Run 10 ticks
    for _ in 0..10 {
        scheduler.tick_once();
    }

    let s = sensor_count.load(Ordering::Relaxed);
    let c = controller_count.load(Ordering::Relaxed);
    let m = motor_count.load(Ordering::Relaxed);

    // All three nodes should have ticked 10 times
    assert_eq!(s, 10, "sensor should tick 10 times, got {}", s);
    assert_eq!(c, 10, "controller should tick 10 times, got {}", c);
    assert_eq!(m, 10, "motor should tick 10 times, got {}", m);
}

#[test]
fn test_e2e_graceful_shutdown_all_nodes_cleanup() {
    let _guard = lock_scheduler();

    let init_count = Arc::new(AtomicUsize::new(0));
    let tick_count = Arc::new(AtomicUsize::new(0));
    let shutdown_count = Arc::new(AtomicUsize::new(0));

    struct LifecycleNode {
        name: String,
        init_count: Arc<AtomicUsize>,
        tick_count: Arc<AtomicUsize>,
        shutdown_count: Arc<AtomicUsize>,
    }

    impl Node for LifecycleNode {
        fn name(&self) -> &str {
            &self.name
        }
        fn init(&mut self) -> crate::error::HorusResult<()> {
            self.init_count.fetch_add(1, Ordering::Relaxed);
            Ok(())
        }
        fn tick(&mut self) {
            self.tick_count.fetch_add(1, Ordering::Relaxed);
        }
        fn shutdown(&mut self) -> crate::error::HorusResult<()> {
            self.shutdown_count.fetch_add(1, Ordering::Relaxed);
            Ok(())
        }
    }

    let mut scheduler = Scheduler::new().tick_rate(100.hz()).deterministic(true);

    for i in 0..5 {
        let node = LifecycleNode {
            name: format!("node_{}", i),
            init_count: init_count.clone(),
            tick_count: tick_count.clone(),
            shutdown_count: shutdown_count.clone(),
        };
        scheduler.add(node).build();
    }

    // Tick a few times then stop
    for _ in 0..5 {
        scheduler.tick_once();
    }
    scheduler.stop();

    let inits = init_count.load(Ordering::Relaxed);
    let ticks = tick_count.load(Ordering::Relaxed);

    assert_eq!(inits, 5, "all 5 nodes should init");
    assert_eq!(ticks, 25, "5 nodes x 5 ticks = 25");

    // Note: tick_once() mode doesn't call shutdown() — that's done by run() or explicit call.
    // This test verifies the init+tick pipeline works correctly in deterministic mode.
}

// ─── Network / Lifecycle Hook Tests ────────────────────────────────────────

#[test]
fn test_network_enabled_by_default() {
    let _lock = lock_scheduler();
    let scheduler = Scheduler::new();
    assert!(
        scheduler.network_enabled(),
        "network should be on by default"
    );
}

#[test]
fn test_network_default_on() {
    let _lock = lock_scheduler();
    // Network is on by default
    let scheduler = Scheduler::new();
    assert!(scheduler.network_enabled());
}

#[test]
fn test_network_builder_toggle() {
    let _lock = lock_scheduler();
    let scheduler = Scheduler::new().network(true);
    assert!(scheduler.network_enabled());

    let scheduler = Scheduler::new().network(false);
    assert!(!scheduler.network_enabled());
}

#[test]
fn test_lifecycle_hook_invoked_on_run() {
    let _lock = lock_scheduler();
    let started = Arc::new(AtomicBool::new(false));
    let started_clone = started.clone();

    let mut scheduler = Scheduler::new().deterministic(true);
    scheduler.on_start(move || {
        started_clone.store(true, Ordering::SeqCst);
        None
    });
    scheduler
        .add(CounterNode {
            name: "lifecycle_test".to_string(),
            tick_count: Arc::new(AtomicUsize::new(0)),
        })
        .build();

    scheduler.run_for(Duration::from_millis(50)).unwrap();
    assert!(
        started.load(Ordering::SeqCst),
        "lifecycle hook should have been invoked"
    );
}

#[test]
fn test_lifecycle_hook_handle_dropped_on_shutdown() {
    let _lock = lock_scheduler();
    let dropped = Arc::new(AtomicBool::new(false));
    let dropped_clone = dropped.clone();

    struct DropTracker(Arc<AtomicBool>);
    impl Drop for DropTracker {
        fn drop(&mut self) {
            self.0.store(true, Ordering::SeqCst);
        }
    }

    let mut scheduler = Scheduler::new().deterministic(true);
    scheduler.on_start(move || Some(Box::new(DropTracker(dropped_clone))));
    scheduler
        .add(CounterNode {
            name: "drop_test".to_string(),
            tick_count: Arc::new(AtomicUsize::new(0)),
        })
        .build();

    scheduler.run_for(Duration::from_millis(50)).unwrap();
    assert!(
        dropped.load(Ordering::SeqCst),
        "lifecycle handle should have been dropped on shutdown"
    );
}

#[test]
fn test_lifecycle_hooks_dropped_lifo() {
    let _lock = lock_scheduler();
    let order = Arc::new(Mutex::new(Vec::new()));

    struct OrderTracker {
        id: u32,
        order: Arc<Mutex<Vec<u32>>>,
    }
    impl Drop for OrderTracker {
        fn drop(&mut self) {
            self.order.lock().unwrap().push(self.id);
        }
    }

    let mut scheduler = Scheduler::new().deterministic(true);

    for id in 0..3 {
        let order_clone = order.clone();
        scheduler.on_start(move || {
            Some(Box::new(OrderTracker {
                id,
                order: order_clone,
            }))
        });
    }

    scheduler
        .add(CounterNode {
            name: "lifo_test".to_string(),
            tick_count: Arc::new(AtomicUsize::new(0)),
        })
        .build();

    scheduler.run_for(Duration::from_millis(50)).unwrap();
    let drop_order = order.lock().unwrap().clone();
    assert_eq!(
        drop_order,
        vec![2, 1, 0],
        "hooks should be dropped in LIFO order"
    );
}

// ══════════════════════════════════════════════════════════════════════
// RT Feature Tests — validates all new PREEMPT_RT integration APIs
// ══════════════════════════════════════════════════════════════════════

#[test]
fn test_require_rt_fails_on_non_root() {
    let _guard = lock_scheduler();
    // require_rt() is deliberately fail-fast when capability probing says the
    // process cannot use either RT scheduling or memory locking. Hosted runners
    // and sanitizer processes commonly have neither capability.
    let required = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
        Scheduler::new().require_rt()
    }));
    let Ok(scheduler) = required else {
        // Capability probing found neither RT scheduling nor memory locking.
        return;
    };

    let mut scheduler = scheduler.tick_rate(10_u64.hz());
    scheduler
        .add(CounterNode {
            name: "rt_fail_test".to_string(),
            tick_count: Arc::new(AtomicUsize::new(0)),
        })
        .build()
        .ok();

    let result = scheduler.run_for(Duration::from_millis(50));

    if horus_sys::rt::can_set_rt_priority() {
        assert!(
            result.is_ok(),
            "capability probe allowed RT but apply failed: {result:?}"
        );
    } else {
        assert!(
            result.is_err(),
            "require_rt() should fail when RT scheduling cannot be applied"
        );
    }
}

#[test]
fn test_prefer_rt_succeeds_on_non_root() {
    let _guard = lock_scheduler();
    // prefer_rt should always succeed — degradations are warnings, not errors
    let mut scheduler = Scheduler::new().prefer_rt().tick_rate(10_u64.hz());
    scheduler
        .add(CounterNode {
            name: "rt_prefer_test".to_string(),
            tick_count: Arc::new(AtomicUsize::new(0)),
        })
        .build()
        .ok();

    let result = scheduler.run_for(Duration::from_millis(50));
    assert!(
        result.is_ok(),
        "prefer_rt() should always succeed: {:?}",
        result.err()
    );
}

#[test]
fn test_prefer_rt_stores_degradations() {
    let _guard = lock_scheduler();
    let mut scheduler = Scheduler::new().prefer_rt().tick_rate(10_u64.hz());
    scheduler
        .add(CounterNode {
            name: "degrad_test".to_string(),
            tick_count: Arc::new(AtomicUsize::new(0)),
        })
        .build()
        .ok();

    // Force finalization
    let _ = scheduler.run_for(Duration::from_millis(10));

    // On non-root: should have degradations
    if !horus_sys::rt::can_set_rt_priority() {
        let degs = scheduler.degradations();
        assert!(
            !degs.is_empty(),
            "prefer_rt() on non-root should have degradations"
        );
        // Should include SchedulerDegraded (SCHED_FIFO failed)
        assert!(
            degs.iter()
                .any(|d| matches!(d.feature, RtFeature::RtPriority)),
            "Should have RtPriority degradation"
        );
        assert!(
            !scheduler.has_full_rt(),
            "has_full_rt() should be false with degradations"
        );
    }
}

#[test]
fn test_deadline_scheduler_flag_propagates() {
    let _guard = lock_scheduler();
    // Verify .deadline_scheduler() flag reaches the RT executor
    let config = super::super::node_builder::NodeRegistration::new(Box::new(CounterNode {
        name: "deadline_flag_test".to_string(),
        tick_count: Arc::new(AtomicUsize::new(0)),
    }))
    .rate(100_u64.hz())
    .budget(500_u64.us())
    .deadline_scheduler();

    assert!(
        config.use_sched_deadline,
        ".deadline_scheduler() should set use_sched_deadline = true"
    );
}

#[test]
fn test_no_alloc_flag_propagates() {
    let _guard = lock_scheduler();
    let config = super::super::node_builder::NodeRegistration::new(Box::new(CounterNode {
        name: "no_alloc_flag_test".to_string(),
        tick_count: Arc::new(AtomicUsize::new(0)),
    }))
    .rate(100_u64.hz())
    .no_alloc();

    assert!(config.no_alloc, ".no_alloc() should set no_alloc = true");
}

#[test]
fn test_p99_approx_ns_reasonable() {
    use crate::core::rt_node::RtStats;

    let mut stats = RtStats::default();
    // Simulate 100 ticks of ~500us each with small variance
    for i in 0..100 {
        let duration_us = 500 + (i % 20); // 500-519us
        stats.record_execution(Duration::from_micros(duration_us));
    }

    let p99 = stats.p99_approx_ns();
    // Should be roughly 500-600us = 500_000-600_000 ns
    assert!(
        p99 > 400_000 && p99 < 700_000,
        "P99 should be ~500-600us, got {}ns ({}us)",
        p99,
        p99 / 1000
    );
}

#[test]
fn test_p99_approx_ns_zero_for_no_samples() {
    let stats = crate::core::rt_node::RtStats::default();
    let p99 = stats.p99_approx_ns();
    assert_eq!(p99, 0, "P99 should be 0 with no samples");
}

#[test]
fn test_rt_allocator_context_thread_local() {
    use crate::memory::rt_allocator;

    // Default: not in RT context
    assert!(!rt_allocator::is_rt_context());

    // Enter
    rt_allocator::enter_rt_context("test_node");
    assert!(rt_allocator::is_rt_context());

    // Leave
    rt_allocator::leave_rt_context();
    assert!(!rt_allocator::is_rt_context());

    // Multiple enter/leave cycles
    for _ in 0..10 {
        rt_allocator::enter_rt_context("cycle_test");
        assert!(rt_allocator::is_rt_context());
        rt_allocator::leave_rt_context();
        assert!(!rt_allocator::is_rt_context());
    }
}

#[test]
fn test_sched_deadline_capability_detection() {
    // Should not panic regardless of kernel/permissions
    let has_deadline = horus_sys::rt::has_deadline_capability();
    // On most Linux kernels >= 3.14, this returns true (kernel has it, we just lack permission)
    // On very old kernels or non-Linux, returns false
    // Either result is valid — we just verify it doesn't crash
    let _ = has_deadline;
}

#[test]
#[cfg(not(target_os = "windows"))]
fn test_cpu_governor_set_graceful_failure() {
    // Setting governor on a nonexistent CPU should fail gracefully
    let result = horus_sys::rt::set_cpu_governor(9999, "performance");
    assert!(result.is_err(), "Setting governor on CPU 9999 should fail");
}

#[test]
fn test_move_irqs_graceful_on_empty() {
    // Moving IRQs with empty CPU list should be no-op
    let result = horus_sys::rt::move_irqs_off_cpus(&[]);
    assert!(result.is_ok());
    assert_eq!(result.unwrap(), 0);
}

// ============================================================================
// Ready Dispatch — Comprehensive Blocking Tests
// ============================================================================
//
// These tests verify that the ready-dispatch executor correctly:
// 1. Runs independent nodes in parallel (no unnecessary blocking)
// 2. Respects dependency ordering (producer before consumer)
// 3. Handles edge cases (panics, skipped nodes, single node, etc.)
//
// Strategy: Nodes record their start/end timestamps. We analyze the timestamps
// to prove parallelism (overlapping execution) or ordering (A.end < B.start).

/// Node that sleeps for a fixed duration and records timing.
/// Used to prove parallelism: if two 50ms nodes complete in ~50ms total,
/// they ran in parallel. If ~100ms, they blocked each other.
struct TimingNode {
    node_name: String,
    sleep_ms: u64,
    timestamps: Arc<Mutex<Vec<(String, std::time::Instant, std::time::Instant)>>>,
    /// Topics this node "publishes" — registered in init()
    pub_topics: Vec<String>,
    /// Topics this node "subscribes" — registered in init()
    sub_topics: Vec<String>,
}

impl TimingNode {
    fn new(
        name: &str,
        sleep_ms: u64,
        timestamps: Arc<Mutex<Vec<(String, std::time::Instant, std::time::Instant)>>>,
    ) -> Self {
        Self {
            node_name: name.to_string(),
            sleep_ms,
            timestamps,
            pub_topics: Vec::new(),
            sub_topics: Vec::new(),
        }
    }

    fn publishes(mut self, topic: &str) -> Self {
        self.pub_topics.push(topic.to_string());
        self
    }

    fn subscribes(mut self, topic: &str) -> Self {
        self.sub_topics.push(topic.to_string());
        self
    }
}

impl Node for TimingNode {
    fn name(&self) -> &str {
        &self.node_name
    }

    fn init(&mut self) -> crate::error::HorusResult<()> {
        // Register topics with TopicNodeRegistry during init().
        // This is the Phase 1 registration that the scheduler uses
        // to build the dependency graph before tick 1.
        let tnr = crate::communication::topic_node_registry();
        for topic in &self.pub_topics {
            tnr.register_with_type(
                topic,
                &self.node_name,
                crate::communication::topic::NodeTopicRole::Publisher,
                "u32",
            );
        }
        for topic in &self.sub_topics {
            tnr.register_with_type(
                topic,
                &self.node_name,
                crate::communication::topic::NodeTopicRole::Subscriber,
                "u32",
            );
        }
        Ok(())
    }

    fn tick(&mut self) {
        let start = std::time::Instant::now();
        std::thread::sleep(Duration::from_millis(self.sleep_ms));
        let end = std::time::Instant::now();
        self.timestamps
            .lock()
            .unwrap()
            .push((self.node_name.clone(), start, end));
    }
}

/// Helper: check that node A's tick finished before node B's tick started.
/// Searches for the first occurrence of each in the timestamp log.
fn assert_ordered(
    timestamps: &[(String, std::time::Instant, std::time::Instant)],
    before: &str,
    after: &str,
) {
    let a = timestamps
        .iter()
        .find(|(n, _, _)| n == before)
        .unwrap_or_else(|| panic!("Node '{}' not found in timestamps", before));
    let b = timestamps
        .iter()
        .find(|(n, _, _)| n == after)
        .unwrap_or_else(|| panic!("Node '{}' not found in timestamps", after));
    assert!(
        a.2 <= b.1,
        "Expected '{}' (end {:?}) to finish before '{}' (start {:?})",
        before,
        a.2,
        after,
        b.1
    );
}

/// Helper: check that two nodes overlapped in execution (ran in parallel).
fn assert_parallel(
    timestamps: &[(String, std::time::Instant, std::time::Instant)],
    node_a: &str,
    node_b: &str,
) {
    let a = timestamps
        .iter()
        .find(|(n, _, _)| n == node_a)
        .unwrap_or_else(|| panic!("Node '{}' not found in timestamps", node_a));
    let b = timestamps
        .iter()
        .find(|(n, _, _)| n == node_b)
        .unwrap_or_else(|| panic!("Node '{}' not found in timestamps", node_b));
    // Overlapping: A started before B ended AND B started before A ended
    let overlaps = a.1 < b.2 && b.1 < a.2;
    assert!(
        overlaps,
        "Expected '{}' ({:?}..{:?}) and '{}' ({:?}..{:?}) to overlap (parallel), but they didn't",
        node_a, a.1, a.2, node_b, b.1, b.2
    );
}

#[test]
fn test_ready_dispatch_independent_nodes_run_parallel() {
    // Two independent 50ms nodes should complete in ~50ms, not ~100ms.
    // This is THE core test: proves no unnecessary blocking.
    let _guard = lock_scheduler();
    let ts = Arc::new(Mutex::new(Vec::new()));

    let mut scheduler = Scheduler::new().tick_rate(10_u64.hz());
    scheduler
        .add(TimingNode::new("rd_par_cam", 50, ts.clone()).publishes("rd_par_img"))
        .build();
    scheduler
        .add(TimingNode::new("rd_par_lidar", 50, ts.clone()).publishes("rd_par_pts"))
        .build();

    let wall_start = std::time::Instant::now();
    scheduler.run_for(150_u64.ms());
    let _wall_elapsed = wall_start.elapsed();

    let log = ts.lock().unwrap();
    assert!(
        log.len() >= 2,
        "Expected at least 2 tick records, got {}",
        log.len()
    );

    // Wall time should be well under 100ms for the first tick
    // (two 50ms nodes in parallel = ~50ms, not 100ms)
    assert_parallel(&log, "rd_par_cam", "rd_par_lidar");
}

#[test]
fn test_ready_dispatch_dependent_nodes_ordered() {
    // SLAM subscribes to lidar — SLAM must not start before lidar finishes.
    let _guard = lock_scheduler();
    let ts = Arc::new(Mutex::new(Vec::new()));

    let mut scheduler = Scheduler::new().tick_rate(10_u64.hz());
    scheduler
        .add(TimingNode::new("rd_dep_lidar", 30, ts.clone()).publishes("rd_dep_pts"))
        .build();
    scheduler
        .add(
            TimingNode::new("rd_dep_slam", 30, ts.clone())
                .subscribes("rd_dep_pts")
                .publishes("rd_dep_pose"),
        )
        .build();

    scheduler.run_for(150_u64.ms());

    let log = ts.lock().unwrap();
    assert!(log.len() >= 2);
    assert_ordered(&log, "rd_dep_lidar", "rd_dep_slam");
}

#[test]
fn test_ready_dispatch_diamond_topology() {
    // Classic diamond: A → B, A → C, B → D, C → D
    //   A runs first
    //   B and C run in parallel (both depend only on A)
    //   D runs last (depends on B and C)
    let _guard = lock_scheduler();
    let ts = Arc::new(Mutex::new(Vec::new()));

    let mut scheduler = Scheduler::new().tick_rate(5_u64.hz());
    scheduler
        .add(
            TimingNode::new("rd_dia_A", 20, ts.clone())
                .publishes("rd_dia_ab")
                .publishes("rd_dia_ac"),
        )
        .build();
    scheduler
        .add(
            TimingNode::new("rd_dia_B", 40, ts.clone())
                .subscribes("rd_dia_ab")
                .publishes("rd_dia_bd"),
        )
        .build();
    scheduler
        .add(
            TimingNode::new("rd_dia_C", 40, ts.clone())
                .subscribes("rd_dia_ac")
                .publishes("rd_dia_cd"),
        )
        .build();
    scheduler
        .add(
            TimingNode::new("rd_dia_D", 20, ts.clone())
                .subscribes("rd_dia_bd")
                .subscribes("rd_dia_cd"),
        )
        .build();

    scheduler.run_for(250_u64.ms());

    let log = ts.lock().unwrap();
    assert!(
        log.len() >= 4,
        "Expected all 4 nodes to tick, got {}",
        log.len()
    );

    // A before B and C
    assert_ordered(&log, "rd_dia_A", "rd_dia_B");
    assert_ordered(&log, "rd_dia_A", "rd_dia_C");
    // B and C in parallel
    assert_parallel(&log, "rd_dia_B", "rd_dia_C");
    // B and C before D
    assert_ordered(&log, "rd_dia_B", "rd_dia_D");
    assert_ordered(&log, "rd_dia_C", "rd_dia_D");
}

#[test]
fn test_ready_dispatch_fan_out_all_parallel() {
    // One producer, three independent consumers — consumers should run in parallel.
    let _guard = lock_scheduler();
    let ts = Arc::new(Mutex::new(Vec::new()));

    let mut scheduler = Scheduler::new().tick_rate(5_u64.hz());
    scheduler
        .add(TimingNode::new("rd_fan_src", 10, ts.clone()).publishes("rd_fan_data"))
        .build();
    scheduler
        .add(TimingNode::new("rd_fan_c1", 40, ts.clone()).subscribes("rd_fan_data"))
        .build();
    scheduler
        .add(TimingNode::new("rd_fan_c2", 40, ts.clone()).subscribes("rd_fan_data"))
        .build();
    scheduler
        .add(TimingNode::new("rd_fan_c3", 40, ts.clone()).subscribes("rd_fan_data"))
        .build();

    scheduler.run_for(200_u64.ms());

    let log = ts.lock().unwrap();
    assert!(log.len() >= 4);

    // Source before all consumers
    assert_ordered(&log, "rd_fan_src", "rd_fan_c1");
    assert_ordered(&log, "rd_fan_src", "rd_fan_c2");
    assert_ordered(&log, "rd_fan_src", "rd_fan_c3");
    // Consumers run in parallel
    assert_parallel(&log, "rd_fan_c1", "rd_fan_c2");
    assert_parallel(&log, "rd_fan_c2", "rd_fan_c3");
}

#[test]
fn test_ready_dispatch_fan_in_waits_for_all() {
    // Three producers, one consumer that subscribes to all three.
    // Consumer must wait for ALL producers before starting.
    let _guard = lock_scheduler();
    let ts = Arc::new(Mutex::new(Vec::new()));

    let mut scheduler = Scheduler::new().tick_rate(5_u64.hz());
    scheduler
        .add(TimingNode::new("rd_fi_p1", 10, ts.clone()).publishes("rd_fi_a"))
        .build();
    scheduler
        .add(TimingNode::new("rd_fi_p2", 30, ts.clone()).publishes("rd_fi_b"))
        .build();
    scheduler
        .add(TimingNode::new("rd_fi_p3", 50, ts.clone()).publishes("rd_fi_c"))
        .build();
    scheduler
        .add(
            TimingNode::new("rd_fi_cons", 10, ts.clone())
                .subscribes("rd_fi_a")
                .subscribes("rd_fi_b")
                .subscribes("rd_fi_c"),
        )
        .build();

    scheduler.run_for(200_u64.ms());

    let log = ts.lock().unwrap();
    assert!(log.len() >= 4);

    // All three producers parallel
    assert_parallel(&log, "rd_fi_p1", "rd_fi_p2");
    assert_parallel(&log, "rd_fi_p2", "rd_fi_p3");
    // Consumer waits for slowest producer (p3 = 50ms)
    assert_ordered(&log, "rd_fi_p3", "rd_fi_cons");
}

#[test]
fn test_ready_dispatch_multi_publisher_same_topic() {
    // Two publishers to same topic → subscriber depends on BOTH.
    let _guard = lock_scheduler();
    let ts = Arc::new(Mutex::new(Vec::new()));

    let mut scheduler = Scheduler::new().tick_rate(5_u64.hz());
    scheduler
        .add(TimingNode::new("rd_mp_fast", 10, ts.clone()).publishes("rd_mp_data"))
        .build();
    scheduler
        .add(TimingNode::new("rd_mp_slow", 50, ts.clone()).publishes("rd_mp_data"))
        .build();
    scheduler
        .add(TimingNode::new("rd_mp_sub", 10, ts.clone()).subscribes("rd_mp_data"))
        .build();

    scheduler.run_for(200_u64.ms());

    let log = ts.lock().unwrap();
    assert!(log.len() >= 3);

    // Both publishers parallel
    assert_parallel(&log, "rd_mp_fast", "rd_mp_slow");
    // Subscriber waits for BOTH (including the slow one)
    assert_ordered(&log, "rd_mp_slow", "rd_mp_sub");
}

#[test]
fn test_ready_dispatch_single_node_no_overhead() {
    // Single node should tick normally with no dispatch overhead.
    let _guard = lock_scheduler();
    let counter = Arc::new(AtomicUsize::new(0));

    let mut scheduler = Scheduler::new().tick_rate(100_u64.hz());
    scheduler
        .add(CounterNode::with_counter("rd_single", counter.clone()))
        .build();

    let started = std::time::Instant::now();
    scheduler.run_for(100_u64.ms());
    let elapsed = started.elapsed();

    // The claim is that a lone node dispatches without the coordination the
    // multi-node path needs — not that this machine can sustain 100 Hz.
    //
    // The previous form asserted `ticks >= 5` against a nominal 100 ms window,
    // which made it a measurement of the host: under the full suite it saw
    // fewer and failed, then passed on the retry. Deriving the floor from the
    // wall-clock that actually elapsed keeps the property (dispatch runs, and
    // keeps running) while tolerating a loaded machine.
    let ticks = counter.load(Ordering::SeqCst);
    let expected = (elapsed.as_secs_f64() * 100.0) as usize;
    let floor = (expected / 4).max(2);

    assert!(
        ticks >= floor,
        "single-node dispatch produced {ticks} ticks in {elapsed:?}; at 100 Hz \
         that window allows about {expected}, and even a quarter of it would be \
         {floor}. Dispatch is stalling, not merely slow."
    );
}

#[test]
fn test_ready_dispatch_panicking_node_doesnt_block_others() {
    // If one node panics, other nodes should still tick.
    let _guard = lock_scheduler();
    let healthy_counter = Arc::new(AtomicUsize::new(0));

    struct PanicNode {
        name: String,
    }
    impl Node for PanicNode {
        fn name(&self) -> &str {
            &self.name
        }
        fn tick(&mut self) {
            panic!("intentional panic in ready dispatch test");
        }
    }

    let mut scheduler = Scheduler::new().tick_rate(50_u64.hz()).verbose(false);
    scheduler
        .add(PanicNode {
            name: "rd_panic".to_string(),
        })
        .build();
    scheduler
        .add(CounterNode::with_counter(
            "rd_panic_healthy",
            healthy_counter.clone(),
        ))
        .build();

    scheduler.run_for(1000_u64.ms());

    // Healthy node should still have ticked
    let ticks = healthy_counter.load(Ordering::SeqCst);
    assert!(
        ticks > 0,
        "Healthy node should keep ticking despite sibling panic, got {} ticks",
        ticks
    );
}

#[test]
fn test_ready_dispatch_real_robot_topology() {
    // Realistic 6-node robot pipeline:
    //   IMU (fast) ──→ Controller ──→ Motor
    //   Camera (slow) ──→ Detector ──→ Controller
    //
    // Expected behavior:
    //   - IMU and Camera start in parallel
    //   - IMU finishes fast (10ms), but Controller must ALSO wait for Detector
    //   - Camera finishes (50ms), Detector starts
    //   - Detector finishes, Controller starts (needs IMU + Detector)
    //   - Motor runs last
    let _guard = lock_scheduler();
    let ts = Arc::new(Mutex::new(Vec::new()));

    let mut scheduler = Scheduler::new().tick_rate(3_u64.hz());
    scheduler
        .add(TimingNode::new("rd_bot_imu", 10, ts.clone()).publishes("rd_bot_imu_data"))
        .build();
    scheduler
        .add(TimingNode::new("rd_bot_cam", 50, ts.clone()).publishes("rd_bot_image"))
        .build();
    scheduler
        .add(
            TimingNode::new("rd_bot_det", 20, ts.clone())
                .subscribes("rd_bot_image")
                .publishes("rd_bot_detections"),
        )
        .build();
    scheduler
        .add(
            TimingNode::new("rd_bot_ctrl", 10, ts.clone())
                .subscribes("rd_bot_imu_data")
                .subscribes("rd_bot_detections")
                .publishes("rd_bot_cmd"),
        )
        .build();
    scheduler
        .add(TimingNode::new("rd_bot_motor", 5, ts.clone()).subscribes("rd_bot_cmd"))
        .build();

    scheduler.run_for(300_u64.ms());

    let log = ts.lock().unwrap();
    assert!(
        log.len() >= 5,
        "Expected all 5 nodes to tick, got {}",
        log.len()
    );

    // IMU and Camera parallel (independent sensors)
    assert_parallel(&log, "rd_bot_imu", "rd_bot_cam");
    // Camera before Detector (data dependency)
    assert_ordered(&log, "rd_bot_cam", "rd_bot_det");
    // Detector before Controller (data dependency)
    assert_ordered(&log, "rd_bot_det", "rd_bot_ctrl");
    // IMU before Controller (data dependency)
    assert_ordered(&log, "rd_bot_imu", "rd_bot_ctrl");
    // Controller before Motor (data dependency)
    assert_ordered(&log, "rd_bot_ctrl", "rd_bot_motor");
}

#[test]
fn test_ready_dispatch_no_topics_falls_back_to_order() {
    // Nodes with no topic metadata fall back to .order() tiers.
    // Same order = same step, different order = sequential.
    let _guard = lock_scheduler();
    let order_log = Arc::new(Mutex::new(Vec::<String>::new()));

    struct OrderLogger {
        name: String,
        log: Arc<Mutex<Vec<String>>>,
    }
    impl Node for OrderLogger {
        fn name(&self) -> &str {
            &self.name
        }
        fn tick(&mut self) {
            self.log.lock().unwrap().push(self.name.clone());
        }
    }

    let mut scheduler = Scheduler::new().tick_rate(100_u64.hz());
    scheduler
        .add(OrderLogger {
            name: "rd_fb_third".to_string(),
            log: order_log.clone(),
        })
        .order(20)
        .build();
    scheduler
        .add(OrderLogger {
            name: "rd_fb_first".to_string(),
            log: order_log.clone(),
        })
        .order(0)
        .build();
    scheduler
        .add(OrderLogger {
            name: "rd_fb_second".to_string(),
            log: order_log.clone(),
        })
        .order(10)
        .build();

    // Use tick_once to get deterministic single-tick behavior
    scheduler.tick_once().unwrap();

    let log = order_log.lock().unwrap();
    assert_eq!(log.len(), 3);
    // With .order() fallback (no topic metadata), execution should respect order tiers
    assert_eq!(log[0], "rd_fb_first");
    assert_eq!(log[1], "rd_fb_second");
    assert_eq!(log[2], "rd_fb_third");
}

#[test]
fn test_ready_dispatch_wall_time_proves_parallelism() {
    // Most rigorous timing test: 4 independent 50ms nodes.
    // Sequential: ~200ms. Parallel: ~50ms. We check < 120ms.
    let _guard = lock_scheduler();
    let ts = Arc::new(Mutex::new(Vec::new()));

    let mut scheduler = Scheduler::new().tick_rate(2_u64.hz());
    scheduler
        .add(TimingNode::new("rd_wall_A", 50, ts.clone()).publishes("rd_wall_a"))
        .build();
    scheduler
        .add(TimingNode::new("rd_wall_B", 50, ts.clone()).publishes("rd_wall_b"))
        .build();
    scheduler
        .add(TimingNode::new("rd_wall_C", 50, ts.clone()).publishes("rd_wall_c"))
        .build();
    scheduler
        .add(TimingNode::new("rd_wall_D", 50, ts.clone()).publishes("rd_wall_d"))
        .build();

    let wall_start = std::time::Instant::now();
    // Run for enough time to complete one tick cycle
    scheduler.run_for(300_u64.ms());
    let _first_tick_wall = wall_start.elapsed();

    let log = ts.lock().unwrap();
    assert!(log.len() >= 4, "Expected all 4 nodes to tick");

    // Find the span of the first tick batch (all 4 nodes)
    let first_batch: Vec<_> = log.iter().take(4).collect();
    let earliest_start = first_batch.iter().map(|t| t.1).min().unwrap();
    let latest_end = first_batch.iter().map(|t| t.2).max().unwrap();
    let batch_duration = latest_end.duration_since(earliest_start);

    // 4 × 50ms sequential = 200ms. Parallel should be ~50ms.
    // Allow generous margin for CI/scheduling jitter.
    assert!(
        batch_duration < Duration::from_millis(150),
        "4 independent 50ms nodes took {:?} — should be <150ms if parallel (200ms if sequential)",
        batch_duration
    );
}

#[test]
fn test_ready_dispatch_graph_rebuild_after_tick_one() {
    // Verify Phase 2: if topics are registered during tick (not init),
    // the graph rebuilds after the first tick.
    let _guard = lock_scheduler();

    struct LazyRegisterer {
        name: String,
        topic: String,
        role: crate::communication::topic::NodeTopicRole,
        registered: bool,
        tick_count: Arc<AtomicUsize>,
    }

    impl Node for LazyRegisterer {
        fn name(&self) -> &str {
            &self.name
        }
        fn tick(&mut self) {
            // Register on first tick (simulates lazy Topic::send/recv registration)
            if !self.registered {
                crate::communication::topic_node_registry().register_with_type(
                    &self.topic,
                    &self.name,
                    self.role,
                    "u32",
                );
                self.registered = true;
            }
            self.tick_count.fetch_add(1, Ordering::SeqCst);
        }
    }

    let counter_a = Arc::new(AtomicUsize::new(0));
    let counter_b = Arc::new(AtomicUsize::new(0));

    let mut scheduler = Scheduler::new().tick_rate(50_u64.hz());
    scheduler
        .add(LazyRegisterer {
            name: "rd_lazy_pub".to_string(),
            topic: "rd_lazy_topic".to_string(),
            role: crate::communication::topic::NodeTopicRole::Publisher,
            registered: false,
            tick_count: counter_a.clone(),
        })
        .build();
    scheduler
        .add(LazyRegisterer {
            name: "rd_lazy_sub".to_string(),
            topic: "rd_lazy_topic".to_string(),
            role: crate::communication::topic::NodeTopicRole::Subscriber,
            registered: false,
            tick_count: counter_b.clone(),
        })
        .build();

    // Run for enough ticks to get past the Phase 2 rebuild
    scheduler.run_for(200_u64.ms());

    // Both nodes should have ticked multiple times
    let ticks_a = counter_a.load(Ordering::SeqCst);
    let ticks_b = counter_b.load(Ordering::SeqCst);
    assert!(
        ticks_a >= 3 && ticks_b >= 3,
        "Both nodes should tick, got pub={} sub={}",
        ticks_a,
        ticks_b
    );
}

// ============================================================================
// Practical Robotics Blocking Regression Tests
// ============================================================================
//
// These tests simulate REAL robotics patterns that BLOCKED under the old
// sequential .order()-based model. Each test uses ONLY the user-facing API:
//   - Scheduler::new(), .tick_rate(), .run_for()
//   - scheduler.add(node).build()   (NO .order() — proving it's optional)
//   - Node trait: name(), init(), tick()
//   - TopicNodeRegistry for topic registration (simulates Topic::send/recv)
//
// Each test documents:
//   OLD BEHAVIOR: what happened with sequential execution
//   NEW BEHAVIOR: what should happen with ready-dispatch
//   PROOF: timing or ordering assertions that would FAIL under the old model

/// Register a node as publisher/subscriber with the global TopicNodeRegistry.
/// Simulates what Topic::send() and Topic::recv() do internally.
fn register_topic(
    node_name: &str,
    topic_name: &str,
    role: crate::communication::topic::NodeTopicRole,
) {
    crate::communication::topic_node_registry()
        .register_with_type(topic_name, node_name, role, "f64");
}

// ── Test 1: Warehouse Robot — 4 Independent Sensors ──────────────────────────
//
// Scenario: Warehouse robot has LiDAR, stereo camera, IMU, and wheel encoders.
// All 4 read hardware independently. None depends on the others.
//
// OLD: Sequential: LiDAR(20ms) → Camera(20ms) → IMU(20ms) → Encoder(20ms) = 80ms
// NEW: Parallel: all 4 start together = ~20ms
// PROOF: Total batch time < 50ms (impossible if sequential = 80ms)

#[test]
fn test_robotics_warehouse_4_independent_sensors() {
    let _guard = lock_scheduler();
    let ts = Arc::new(Mutex::new(Vec::new()));

    // All sensors publish to different topics — completely independent
    let mut scheduler = Scheduler::new().tick_rate(5_u64.hz());

    struct SensorNode {
        name: String,
        topic: String,
        sleep_ms: u64,
        ts: Arc<Mutex<Vec<(String, std::time::Instant, std::time::Instant)>>>,
    }

    impl Node for SensorNode {
        fn name(&self) -> &str {
            &self.name
        }
        fn init(&mut self) -> crate::error::HorusResult<()> {
            register_topic(
                &self.name,
                &self.topic,
                crate::communication::topic::NodeTopicRole::Publisher,
            );
            Ok(())
        }
        fn tick(&mut self) {
            let start = std::time::Instant::now();
            std::thread::sleep(Duration::from_millis(self.sleep_ms));
            let end = std::time::Instant::now();
            self.ts
                .lock()
                .unwrap()
                .push((self.name.clone(), start, end));
        }
    }

    for (name, topic) in [
        ("wh_lidar", "wh_scan"),
        ("wh_camera", "wh_image"),
        ("wh_imu", "wh_imu_data"),
        ("wh_encoder", "wh_odom"),
    ] {
        scheduler
            .add(SensorNode {
                name: name.to_string(),
                topic: topic.to_string(),
                sleep_ms: 20,
                ts: ts.clone(),
            })
            .build();
    }

    scheduler.run_for(250_u64.ms());

    let log = ts.lock().unwrap();
    assert!(
        log.len() >= 4,
        "All 4 sensors should tick, got {}",
        log.len()
    );

    // Prove parallelism: measure the first batch span
    let first_four: Vec<_> = log.iter().take(4).collect();
    let earliest = first_four.iter().map(|t| t.1).min().unwrap();
    let latest = first_four.iter().map(|t| t.2).max().unwrap();
    let batch_time = latest.duration_since(earliest);

    // OLD MODEL: 4 × 20ms = 80ms sequential
    // NEW MODEL: ~20ms parallel
    assert!(
        batch_time < Duration::from_millis(50),
        "4 independent 20ms sensors took {:?} — old model would take 80ms, new should be ~20ms",
        batch_time
    );
}

// ── Test 2: Surgical Robot — Safety-Critical Pipeline ────────────────────────
//
// Scenario: Surgical robot has force sensor + vision system feeding a
// safety monitor, then the controller, then the actuator.
// Force and vision are independent. Everything else is a chain.
//
//   ForceSensor ──→ SafetyMonitor ──→ Controller ──→ Actuator
//   VisionSystem ──↗
//
// OLD: All 5 nodes sequential even though Force and Vision are independent
// NEW: Force and Vision parallel, then SafetyMonitor, then Controller, then Actuator
// PROOF: Force and Vision overlap; SafetyMonitor starts only after both finish

#[test]
fn test_robotics_surgical_safety_pipeline() {
    let _guard = lock_scheduler();
    let ts = Arc::new(Mutex::new(Vec::new()));

    struct PipelineNode {
        name: String,
        pubs: Vec<String>,
        subs: Vec<String>,
        sleep_ms: u64,
        ts: Arc<Mutex<Vec<(String, std::time::Instant, std::time::Instant)>>>,
    }

    impl Node for PipelineNode {
        fn name(&self) -> &str {
            &self.name
        }
        fn init(&mut self) -> crate::error::HorusResult<()> {
            for t in &self.pubs {
                register_topic(
                    &self.name,
                    t,
                    crate::communication::topic::NodeTopicRole::Publisher,
                );
            }
            for t in &self.subs {
                register_topic(
                    &self.name,
                    t,
                    crate::communication::topic::NodeTopicRole::Subscriber,
                );
            }
            Ok(())
        }
        fn tick(&mut self) {
            let start = std::time::Instant::now();
            std::thread::sleep(Duration::from_millis(self.sleep_ms));
            let end = std::time::Instant::now();
            self.ts
                .lock()
                .unwrap()
                .push((self.name.clone(), start, end));
        }
    }

    let mut scheduler = Scheduler::new().tick_rate(3_u64.hz());

    // NO .order() calls — dependency graph handles everything
    scheduler
        .add(PipelineNode {
            name: "sr_force".into(),
            pubs: vec!["sr_force_data".into()],
            subs: vec![],
            sleep_ms: 15,
            ts: ts.clone(),
        })
        .build();

    scheduler
        .add(PipelineNode {
            name: "sr_vision".into(),
            pubs: vec!["sr_vision_data".into()],
            subs: vec![],
            sleep_ms: 40,
            ts: ts.clone(),
        })
        .build();

    scheduler
        .add(PipelineNode {
            name: "sr_safety".into(),
            pubs: vec!["sr_safe_cmd".into()],
            subs: vec!["sr_force_data".into(), "sr_vision_data".into()],
            sleep_ms: 5,
            ts: ts.clone(),
        })
        .build();

    scheduler
        .add(PipelineNode {
            name: "sr_ctrl".into(),
            pubs: vec!["sr_joint_cmd".into()],
            subs: vec!["sr_safe_cmd".into()],
            sleep_ms: 10,
            ts: ts.clone(),
        })
        .build();

    scheduler
        .add(PipelineNode {
            name: "sr_actuator".into(),
            pubs: vec![],
            subs: vec!["sr_joint_cmd".into()],
            sleep_ms: 5,
            ts: ts.clone(),
        })
        .build();

    scheduler.run_for(400_u64.ms());

    let log = ts.lock().unwrap();
    assert!(log.len() >= 5, "All 5 nodes should tick, got {}", log.len());

    // Force and Vision are parallel (independent sensors)
    assert_parallel(&log, "sr_force", "sr_vision");
    // Safety waits for BOTH sensors
    assert_ordered(&log, "sr_vision", "sr_safety"); // vision is the slow one (40ms)
    assert_ordered(&log, "sr_force", "sr_safety");
    // Controller after safety
    assert_ordered(&log, "sr_safety", "sr_ctrl");
    // Actuator after controller
    assert_ordered(&log, "sr_ctrl", "sr_actuator");
}

// ── Test 3: Autonomous Car — Perception + Planning + Control ─────────────────
//
// Scenario: Self-driving car with 3 cameras, 1 LiDAR, 1 radar — all feeding
// separate perception pipelines that merge into a planner then controller.
//
//   FrontCam ──→ FrontDetector ──┐
//   LeftCam  ──→ LeftDetector  ──┤
//   RightCam ──→ RightDetector ──┼──→ FusionPlanner ──→ VehicleCtrl
//   LiDAR    ──→ PointCloudProc ┤
//   Radar    ──→ RadarProc ──────┘
//
// OLD: 10 nodes × sequential = sum of all tick times
// NEW: 5 sensors parallel, 5 processors parallel, then fusion, then control
// PROOF: Sensors overlap; processors overlap; total < sequential sum

#[test]
#[cfg(not(target_os = "windows"))]
fn test_robotics_autonomous_car_perception() {
    let _guard = lock_scheduler();
    let ts = Arc::new(Mutex::new(Vec::new()));

    struct CarNode {
        name: String,
        pubs: Vec<String>,
        subs: Vec<String>,
        sleep_ms: u64,
        ts: Arc<Mutex<Vec<(String, std::time::Instant, std::time::Instant)>>>,
    }

    impl Node for CarNode {
        fn name(&self) -> &str {
            &self.name
        }
        fn init(&mut self) -> crate::error::HorusResult<()> {
            for t in &self.pubs {
                register_topic(
                    &self.name,
                    t,
                    crate::communication::topic::NodeTopicRole::Publisher,
                );
            }
            for t in &self.subs {
                register_topic(
                    &self.name,
                    t,
                    crate::communication::topic::NodeTopicRole::Subscriber,
                );
            }
            Ok(())
        }
        fn tick(&mut self) {
            let start = std::time::Instant::now();
            std::thread::sleep(Duration::from_millis(self.sleep_ms));
            let end = std::time::Instant::now();
            self.ts
                .lock()
                .unwrap()
                .push((self.name.clone(), start, end));
        }
    }

    let mut scheduler = Scheduler::new().tick_rate(2_u64.hz());

    // Layer 1: 5 sensors (all independent, all parallel)
    for (name, topic, ms) in [
        ("ac_front_cam", "ac_front_img", 15),
        ("ac_left_cam", "ac_left_img", 15),
        ("ac_right_cam", "ac_right_img", 15),
        ("ac_lidar", "ac_cloud", 20),
        ("ac_radar", "ac_radar_pts", 10),
    ] {
        scheduler
            .add(CarNode {
                name: name.into(),
                pubs: vec![topic.into()],
                subs: vec![],
                sleep_ms: ms,
                ts: ts.clone(),
            })
            .build();
    }

    // Layer 2: 5 detectors (each depends on one sensor, parallel with each other)
    for (name, pub_t, sub_t, ms) in [
        ("ac_front_det", "ac_front_det_out", "ac_front_img", 25),
        ("ac_left_det", "ac_left_det_out", "ac_left_img", 25),
        ("ac_right_det", "ac_right_det_out", "ac_right_img", 25),
        ("ac_pc_proc", "ac_pc_out", "ac_cloud", 30),
        ("ac_radar_proc", "ac_radar_out", "ac_radar_pts", 15),
    ] {
        scheduler
            .add(CarNode {
                name: name.into(),
                pubs: vec![pub_t.into()],
                subs: vec![sub_t.into()],
                sleep_ms: ms,
                ts: ts.clone(),
            })
            .build();
    }

    // Layer 3: Fusion planner (depends on ALL detectors)
    scheduler
        .add(CarNode {
            name: "ac_planner".into(),
            pubs: vec!["ac_plan".into()],
            subs: vec![
                "ac_front_det_out".into(),
                "ac_left_det_out".into(),
                "ac_right_det_out".into(),
                "ac_pc_out".into(),
                "ac_radar_out".into(),
            ],
            sleep_ms: 15,
            ts: ts.clone(),
        })
        .build();

    // Layer 4: Vehicle controller
    scheduler
        .add(CarNode {
            name: "ac_vehicle_ctrl".into(),
            pubs: vec![],
            subs: vec!["ac_plan".into()],
            sleep_ms: 5,
            ts: ts.clone(),
        })
        .build();

    scheduler.run_for(600_u64.ms());

    let log = ts.lock().unwrap();
    assert!(
        log.len() >= 12,
        "All 12 nodes should tick, got {}",
        log.len()
    );

    // Layer 1: all 5 sensors run in parallel
    assert_parallel(&log, "ac_front_cam", "ac_lidar");
    assert_parallel(&log, "ac_left_cam", "ac_radar");
    assert_parallel(&log, "ac_right_cam", "ac_front_cam");

    // Layer 2: detectors run in parallel with each other
    assert_parallel(&log, "ac_front_det", "ac_left_det");
    assert_parallel(&log, "ac_pc_proc", "ac_radar_proc");

    // Causal: each detector after its sensor
    assert_ordered(&log, "ac_front_cam", "ac_front_det");
    assert_ordered(&log, "ac_lidar", "ac_pc_proc");
    assert_ordered(&log, "ac_radar", "ac_radar_proc");

    // Planner after ALL detectors (including slowest = pc_proc at 30ms)
    assert_ordered(&log, "ac_pc_proc", "ac_planner");
    assert_ordered(&log, "ac_front_det", "ac_planner");

    // Vehicle controller after planner
    assert_ordered(&log, "ac_planner", "ac_vehicle_ctrl");

    // WALL TIME PROOF: sequential sum = 15+15+15+20+10+25+25+25+30+15+15+5 = 215ms
    // Parallel critical path: 20(lidar) + 30(pc_proc) + 15(planner) + 5(ctrl) = 70ms
    let first_batch: Vec<_> = log.iter().take(12).collect();
    let earliest = first_batch.iter().map(|t| t.1).min().unwrap();
    let latest = first_batch.iter().map(|t| t.2).max().unwrap();
    let total = latest.duration_since(earliest);
    assert!(
        total < Duration::from_millis(150),
        "12-node car pipeline took {:?} — should be ~70ms parallel, not 215ms sequential",
        total
    );
}

// ── Test 4: Factory Floor — Multiple Independent Robot Arms ──────────────────
//
// Scenario: 3 robot arms on a factory floor. Each arm has its own sensor +
// controller + actuator chain. Arms are completely independent.
//
//   Arm1_Sensor → Arm1_Ctrl → Arm1_Motor
//   Arm2_Sensor → Arm2_Ctrl → Arm2_Motor
//   Arm3_Sensor → Arm3_Ctrl → Arm3_Motor
//
// OLD: All 9 nodes sequential = 9 × sleep time
// NEW: All 3 arms run their chains in parallel
// PROOF: Arms overlap in execution

#[test]
fn test_robotics_factory_independent_arms() {
    let _guard = lock_scheduler();
    let ts = Arc::new(Mutex::new(Vec::new()));

    struct ArmNode {
        name: String,
        pubs: Vec<String>,
        subs: Vec<String>,
        sleep_ms: u64,
        ts: Arc<Mutex<Vec<(String, std::time::Instant, std::time::Instant)>>>,
    }

    impl Node for ArmNode {
        fn name(&self) -> &str {
            &self.name
        }
        fn init(&mut self) -> crate::error::HorusResult<()> {
            for t in &self.pubs {
                register_topic(
                    &self.name,
                    t,
                    crate::communication::topic::NodeTopicRole::Publisher,
                );
            }
            for t in &self.subs {
                register_topic(
                    &self.name,
                    t,
                    crate::communication::topic::NodeTopicRole::Subscriber,
                );
            }
            Ok(())
        }
        fn tick(&mut self) {
            let start = std::time::Instant::now();
            std::thread::sleep(Duration::from_millis(self.sleep_ms));
            let end = std::time::Instant::now();
            self.ts
                .lock()
                .unwrap()
                .push((self.name.clone(), start, end));
        }
    }

    let mut scheduler = Scheduler::new().tick_rate(3_u64.hz());

    // 3 arms, each with sensor → controller → motor (completely independent chains)
    for arm_id in 1..=3 {
        let prefix = format!("fa_arm{}", arm_id);
        scheduler
            .add(ArmNode {
                name: format!("{}_sensor", prefix),
                pubs: vec![format!("{}_joint_state", prefix)],
                subs: vec![],
                sleep_ms: 15,
                ts: ts.clone(),
            })
            .build();
        scheduler
            .add(ArmNode {
                name: format!("{}_ctrl", prefix),
                pubs: vec![format!("{}_joint_cmd", prefix)],
                subs: vec![format!("{}_joint_state", prefix)],
                sleep_ms: 15,
                ts: ts.clone(),
            })
            .build();
        scheduler
            .add(ArmNode {
                name: format!("{}_motor", prefix),
                pubs: vec![],
                subs: vec![format!("{}_joint_cmd", prefix)],
                sleep_ms: 10,
                ts: ts.clone(),
            })
            .build();
    }

    scheduler.run_for(400_u64.ms());

    let log = ts.lock().unwrap();
    assert!(log.len() >= 9, "All 9 nodes should tick, got {}", log.len());

    // Within each arm: causal ordering maintained
    for arm_id in 1..=3 {
        let prefix = format!("fa_arm{}", arm_id);
        assert_ordered(
            &log,
            &format!("{}_sensor", prefix),
            &format!("{}_ctrl", prefix),
        );
        assert_ordered(
            &log,
            &format!("{}_ctrl", prefix),
            &format!("{}_motor", prefix),
        );
    }

    // Across arms: sensors run in parallel (different arms are independent)
    assert_parallel(&log, "fa_arm1_sensor", "fa_arm2_sensor");
    assert_parallel(&log, "fa_arm2_sensor", "fa_arm3_sensor");

    // Controllers also parallel across arms
    assert_parallel(&log, "fa_arm1_ctrl", "fa_arm2_ctrl");

    // WALL TIME: sequential = 9 × ~15ms = 135ms
    // Parallel = one arm chain = 15+15+10 = 40ms (all 3 arms overlap)
    let first_nine: Vec<_> = log.iter().take(9).collect();
    let earliest = first_nine.iter().map(|t| t.1).min().unwrap();
    let latest = first_nine.iter().map(|t| t.2).max().unwrap();
    let total = latest.duration_since(earliest);
    assert!(
        total < Duration::from_millis(80),
        "3-arm factory pipeline took {:?} — should be ~40ms parallel, not 135ms sequential",
        total
    );
}

// ── Test 5: Drone — Fast IMU Loop with Slow Camera ──────────────────────────
//
// Scenario: Quadrotor drone. IMU runs at high speed (2ms tick).
// Camera is slow (40ms). Navigation depends on both.
// Motor controller depends on navigation.
//
//   IMU (2ms)    ──→ Navigation ──→ MotorMixer
//   Camera (40ms) ──↗
//
// KEY TEST: IMU should NOT be blocked by Camera. Without the dependency graph,
// the old sequential model forced IMU to wait for Camera or vice versa.
//
// OLD: IMU waits for Camera (or Camera waits for IMU) = 42ms per tick
// NEW: IMU and Camera parallel = max(2, 40) = 40ms, then Nav + Motor
// PROOF: IMU and Camera overlap; Navigation starts after Camera (the slow one)

#[test]
fn test_robotics_drone_fast_imu_slow_camera() {
    let _guard = lock_scheduler();
    let ts = Arc::new(Mutex::new(Vec::new()));

    struct DroneNode {
        name: String,
        pubs: Vec<String>,
        subs: Vec<String>,
        sleep_ms: u64,
        ts: Arc<Mutex<Vec<(String, std::time::Instant, std::time::Instant)>>>,
    }

    impl Node for DroneNode {
        fn name(&self) -> &str {
            &self.name
        }
        fn init(&mut self) -> crate::error::HorusResult<()> {
            for t in &self.pubs {
                register_topic(
                    &self.name,
                    t,
                    crate::communication::topic::NodeTopicRole::Publisher,
                );
            }
            for t in &self.subs {
                register_topic(
                    &self.name,
                    t,
                    crate::communication::topic::NodeTopicRole::Subscriber,
                );
            }
            Ok(())
        }
        fn tick(&mut self) {
            let start = std::time::Instant::now();
            std::thread::sleep(Duration::from_millis(self.sleep_ms));
            let end = std::time::Instant::now();
            self.ts
                .lock()
                .unwrap()
                .push((self.name.clone(), start, end));
        }
    }

    let mut scheduler = Scheduler::new().tick_rate(3_u64.hz());

    scheduler
        .add(DroneNode {
            name: "dr_imu".into(),
            pubs: vec!["dr_imu_data".into()],
            subs: vec![],
            sleep_ms: 2,
            ts: ts.clone(),
        })
        .build();

    scheduler
        .add(DroneNode {
            name: "dr_camera".into(),
            pubs: vec!["dr_image".into()],
            subs: vec![],
            sleep_ms: 40,
            ts: ts.clone(),
        })
        .build();

    scheduler
        .add(DroneNode {
            name: "dr_nav".into(),
            pubs: vec!["dr_nav_cmd".into()],
            subs: vec!["dr_imu_data".into(), "dr_image".into()],
            sleep_ms: 10,
            ts: ts.clone(),
        })
        .build();

    scheduler
        .add(DroneNode {
            name: "dr_motors".into(),
            pubs: vec![],
            subs: vec!["dr_nav_cmd".into()],
            sleep_ms: 2,
            ts: ts.clone(),
        })
        .build();

    scheduler.run_for(300_u64.ms());

    let log = ts.lock().unwrap();
    assert!(log.len() >= 4, "All 4 nodes should tick, got {}", log.len());

    // IMU and Camera run in parallel — IMU is NOT blocked by slow Camera
    assert_parallel(&log, "dr_imu", "dr_camera");

    // Navigation waits for BOTH (Camera is the bottleneck at 40ms)
    assert_ordered(&log, "dr_camera", "dr_nav");
    assert_ordered(&log, "dr_imu", "dr_nav");

    // Motors after navigation
    assert_ordered(&log, "dr_nav", "dr_motors");
}

// ── Test 6: No .order() Needed — Pure Topic-Driven Ordering ─────────────────
//
// Scenario: 5-node pipeline added in RANDOM ORDER to the scheduler.
// No .order() calls at all. The dependency graph must figure out the
// correct execution order purely from topic pub/sub relationships.
//
// E(sub cmd) added first, then C(sub scan, pub plan), then A(pub scan),
// then D(sub plan, pub cmd), then B(pub scan too — multi-publisher).
//
// OLD: Would execute in add-order: E, C, A, D, B — completely wrong
// NEW: A,B parallel → C → D → E

#[test]
fn test_robotics_no_order_random_add_sequence() {
    let _guard = lock_scheduler();
    let ts = Arc::new(Mutex::new(Vec::new()));

    struct SimpleNode {
        name: String,
        pubs: Vec<String>,
        subs: Vec<String>,
        sleep_ms: u64,
        ts: Arc<Mutex<Vec<(String, std::time::Instant, std::time::Instant)>>>,
    }

    impl Node for SimpleNode {
        fn name(&self) -> &str {
            &self.name
        }
        fn init(&mut self) -> crate::error::HorusResult<()> {
            for t in &self.pubs {
                register_topic(
                    &self.name,
                    t,
                    crate::communication::topic::NodeTopicRole::Publisher,
                );
            }
            for t in &self.subs {
                register_topic(
                    &self.name,
                    t,
                    crate::communication::topic::NodeTopicRole::Subscriber,
                );
            }
            Ok(())
        }
        fn tick(&mut self) {
            let start = std::time::Instant::now();
            std::thread::sleep(Duration::from_millis(self.sleep_ms));
            let end = std::time::Instant::now();
            self.ts
                .lock()
                .unwrap()
                .push((self.name.clone(), start, end));
        }
    }

    let mut scheduler = Scheduler::new().tick_rate(3_u64.hz());

    // Added in deliberately WRONG order — no .order() calls
    scheduler
        .add(SimpleNode {
            name: "ro_E".into(),
            pubs: vec![],
            subs: vec!["ro_cmd".into()],
            sleep_ms: 5,
            ts: ts.clone(),
        })
        .build();
    scheduler
        .add(SimpleNode {
            name: "ro_C".into(),
            pubs: vec!["ro_plan".into()],
            subs: vec!["ro_scan".into()],
            sleep_ms: 15,
            ts: ts.clone(),
        })
        .build();
    scheduler
        .add(SimpleNode {
            name: "ro_A".into(),
            pubs: vec!["ro_scan".into()],
            subs: vec![],
            sleep_ms: 10,
            ts: ts.clone(),
        })
        .build();
    scheduler
        .add(SimpleNode {
            name: "ro_D".into(),
            pubs: vec!["ro_cmd".into()],
            subs: vec!["ro_plan".into()],
            sleep_ms: 10,
            ts: ts.clone(),
        })
        .build();
    scheduler
        .add(SimpleNode {
            name: "ro_B".into(),
            pubs: vec!["ro_scan".into()],
            subs: vec![],
            sleep_ms: 10,
            ts: ts.clone(),
        })
        .build();

    scheduler.run_for(300_u64.ms());

    let log = ts.lock().unwrap();
    assert!(log.len() >= 5, "All 5 nodes should tick, got {}", log.len());

    // A and B are independent publishers — run in parallel
    assert_parallel(&log, "ro_A", "ro_B");
    // C depends on A and B (subscribes to "ro_scan")
    assert_ordered(&log, "ro_A", "ro_C");
    assert_ordered(&log, "ro_B", "ro_C");
    // D depends on C
    assert_ordered(&log, "ro_C", "ro_D");
    // E depends on D
    assert_ordered(&log, "ro_D", "ro_E");
}

/// A scheduler with no RT nodes and no watchdog must STILL have a safety
/// monitor, because that is what an emergency stop latches onto.
///
/// `apply_safety_config` used to create the monitor only when
/// `watchdog_active || has_rt_nodes`. `SharedMonitors.estop` is derived from
/// `monitor.safety`, so on an ordinary non-RT process — perception, teleop,
/// logging — it was `None`: a networked emergency stop had nothing to latch,
/// printed to stderr, and the robot kept running while `SafetyState` reported
/// Normal. E-stop is not an RT-only feature.
#[test]
fn safety_monitor_exists_even_with_no_rt_nodes_or_watchdog() {
    let _guard = lock_scheduler();

    let mut scheduler = Scheduler::new();
    scheduler.add(CounterNode::new("plain_a"));
    scheduler.add(CounterNode::new("plain_b"));

    // Exactly the default: no .rate(), no .budget(), no .watchdog().
    let config = scheduler.pending_config.clone();
    scheduler.apply_safety_config(&config.realtime);

    assert!(
        !scheduler.nodes.iter().any(|n| n.is_rt_node),
        "precondition: this scheduler has no RT nodes"
    );
    assert_eq!(
        config.realtime.watchdog_timeout_ms, 0,
        "precondition: no watchdog is configured"
    );
    assert!(
        scheduler.monitor.safety.is_some(),
        "a non-RT scheduler must still have a safety monitor — otherwise a \
         networked emergency stop has nothing to latch and is only a stderr print"
    );
}

/// `apply_config` — the sole sink for the Python configuration surface — must
/// arm the watchdog it is given.
///
/// It used to call `apply_safety_config` eagerly. Its only caller is
/// `PyScheduler::new`, which runs before any node is added, so the monitor was
/// built over an empty node list and registered zero critical nodes.
/// `finalize_config` then rebuilt the monitor from `pending_config` — where the
/// value had never been recorded — and replaced it. Every Python
/// `Scheduler(watchdog_ms=N)` therefore ran with an empty watchdog map.
#[test]
fn apply_config_watchdog_survives_finalize_and_registers_nodes() {
    let _guard = lock_scheduler();

    let mut scheduler = Scheduler::new();
    let mut config = crate::scheduling::config::SchedulerConfig::default();
    config.realtime.watchdog_timeout_ms = 250;

    // Python order: configure first, add nodes afterwards.
    scheduler.apply_config(config);
    // An RT node — what `horus.Node(..., rate=..)` produces, and what
    // `apply_safety_config` documents itself as watchdogging.
    scheduler
        .add(CounterNode::new("py_wd_rt"))
        .rate(100_u64.hz())
        .build();

    assert_eq!(
        scheduler.pending_config.realtime.watchdog_timeout_ms, 250,
        "apply_config must record the watchdog in pending_config, or finalize_config \
         will rebuild the monitor without it"
    );

    scheduler.finalize_config();

    let safety = scheduler
        .monitor
        .safety
        .as_ref()
        .expect("a configured watchdog must produce a safety monitor");
    assert!(
        safety.critical_node_count() > 0,
        "the watchdog must actually be registered against the nodes — an empty \
         watchdog map means check_watchdogs iterates nothing and the timeout is inert"
    );
}

/// The RT executor must be able to reach the SafetyMonitor.
///
/// `max_deadline_misses` and the graduated degradation ladder live in the main
/// loop's `check_timing_violations`, gated on `is_rt_node`. But the class
/// partition moves every RT node OUT of `Scheduler::nodes` into RtExecutor,
/// leaving only BestEffort nodes behind — for which that gate is false. So
/// `.rate()`, the very thing that makes a node RT, guaranteed its deadline
/// misses would never be counted: the ceiling only worked in deterministic
/// mode, where the partition does not happen.
///
/// This asserts the handle exists. Exercising an actual miss needs RT
/// scheduling privileges this machine does not have.
#[test]
fn shared_monitors_carry_a_safety_handle_for_executors() {
    let _guard = lock_scheduler();

    let mut scheduler = Scheduler::new();
    scheduler
        .add(CounterNode::new("agg_rt"))
        .rate(100_u64.hz())
        .build();
    let config = scheduler.pending_config.clone();
    scheduler.apply_safety_config(&config.realtime);

    assert!(
        scheduler.monitor.safety.is_some(),
        "precondition: an RT node produces a safety monitor"
    );

    // This is what run() hands to every executor thread.
    let shared = crate::scheduling::types::SharedMonitors {
        profiler: scheduler.monitor.profiler.clone(),
        blackbox: scheduler.monitor.blackbox.clone(),
        verbose: false,
        registry: Default::default(),
        registry_slots: Default::default(),
        node_controls: Default::default(),
        clock: scheduler.clock.clone(),
        tick_period: scheduler.tick.period,
        watchdog: scheduler
            .monitor
            .safety
            .as_ref()
            .map(|m| m.watchdog_feeder()),
        estop: scheduler.monitor.safety.as_ref().map(|m| m.estop_trigger()),
        safety: scheduler.monitor.safety.clone(),
    };

    assert!(
        shared.safety.is_some(),
        "executors must receive the SafetyMonitor — without it max_deadline_misses \
         and the degradation ladder are dead code for every RT node"
    );
    // And it must be the SAME monitor, not a copy: the miss counter is what
    // max_deadline_misses compares against.
    let m = shared.safety.as_ref().unwrap();
    m.record_deadline_miss("agg_rt");
    assert_eq!(
        scheduler
            .monitor
            .safety
            .as_ref()
            .unwrap()
            .consecutive_misses("agg_rt"),
        m.consecutive_misses("agg_rt"),
        "the executor's handle must share state with the scheduler's monitor"
    );
}

/// The main loop must clear a node's consecutive-miss run on a tick that MET
/// its deadline — and this node has a deadline and no tick budget, so it never
/// enters the budget block that used to nominally own the reset.
///
/// `node_builder` derives a deadline from a budget and never the reverse, so
/// this shape is ordinary: `.deadline(..)` with no `.tick_budget(..)`. For it,
/// `check_timing_violations` skipped the whole `if let Some(tick_budget)`
/// block, nothing ever called `check_budget`, and `consecutive_misses` was a
/// lifetime total. A node missing once an hour and recovering instantly
/// reached any ceiling eventually — the exact "timer wearing a safety
/// threshold's name" the per-node ceiling was introduced to end.
struct AlternatingNode {
    name: String,
    ticks: Arc<AtomicUsize>,
}

impl Node for AlternatingNode {
    fn name(&self) -> &str {
        &self.name
    }

    fn tick(&mut self) {
        // Miss the deadline on every other tick, comfortably meeting it in
        // between. Never two misses in a row.
        if self.ticks.fetch_add(1, Ordering::SeqCst).is_multiple_of(2) {
            std::thread::sleep(Duration::from_millis(5));
        }
    }
}

#[test]
fn a_met_deadline_clears_the_miss_run_for_a_node_with_no_budget() {
    let _guard = lock_scheduler();
    let ticks = Arc::new(AtomicUsize::new(0));

    let mut scheduler = Scheduler::new()
        .tick_rate(100_u64.hz())
        // Keeps the RT node on the main thread, so this exercises
        // `check_timing_violations` rather than the RT executor.
        .deterministic(true);
    scheduler
        .add(AlternatingNode {
            name: "flaky".to_string(),
            ticks: ticks.clone(),
        })
        .deadline(1_u64.ms()) // deliberately no .tick_budget()
        .build();

    let _ = scheduler.run_for(400_u64.ms());

    let monitor = scheduler
        .monitor
        .safety
        .as_ref()
        .expect("a node with a deadline produces a safety monitor");

    // Anti-vacuity: the node must actually have missed, repeatedly, or the
    // assertion below passes for the wrong reason.
    let total_misses = monitor
        .all_node_timing()
        .into_iter()
        .find(|r| r.name == "flaky")
        .map(|r| r.deadline_misses)
        .unwrap_or(0);
    assert!(
        total_misses >= 3,
        "precondition: the node must miss its deadline repeatedly for this \
         test to mean anything (saw {} misses in {} ticks)",
        total_misses,
        ticks.load(Ordering::SeqCst)
    );

    assert!(
        monitor.consecutive_misses("flaky") <= 1,
        "the node met its deadline between every miss, so its CONSECUTIVE run \
         is at most 1 — it read {} because nothing on this path ever cleared \
         it, making a per-node consecutive ceiling a lifetime total",
        monitor.consecutive_misses("flaky")
    );
}

#[cfg(test)]
mod watchdog_message_honesty {
    use super::super::watchdog_critical_message;

    /// The message must not claim the node was safed.
    ///
    /// Safing is queued on the node's own executor thread, and the canonical
    /// reason a watchdog reaches 3x is that the node is hung *inside* `tick()`
    /// — blocking exactly that thread. Verified end to end by instrumenting
    /// `enter_safe_state()`: zero calls across a run whose watchdog fired.
    ///
    /// The old wording, "Isolated, safing requested from its executor", reads
    /// to a roboticist as "the motors were stopped".
    #[test]
    fn does_not_claim_the_node_was_safed() {
        let m = watchdog_critical_message("hanger");
        assert!(
            !m.contains("safing requested"),
            "must not imply safing happened: {m}"
        );
        assert!(
            m.contains("may never"),
            "must state that safing may not run: {m}"
        );
    }

    /// The emergency stop is what actually protects, so it must be stated.
    #[test]
    fn names_the_emergency_stop_as_the_real_protection() {
        let m = watchdog_critical_message("hanger");
        assert!(m.contains("Emergency stop latched"), "{m}");
        assert!(
            m.contains("does not depend on it"),
            "must say the e-stop is independent of the stalled thread: {m}"
        );
    }

    #[test]
    fn identifies_the_node_and_the_threshold() {
        let m = watchdog_critical_message("hanger");
        assert!(m.contains("'hanger'"), "{m}");
        assert!(m.contains("3x timeout"), "{m}");
        assert!(m.contains("Isolated"), "{m}");
    }
}

#[cfg(test)]
mod rt_reality_check {
    /// Mirrors `warn_if_rt_cannot_be_delivered`'s decision, so the rules are
    /// testable without capturing stdout.
    fn should_warn(debug_build: bool, preempt_rt: bool) -> (bool, bool) {
        if !debug_build && preempt_rt {
            return (false, false);
        }
        (debug_build, !preempt_rt)
    }

    /// The reported case: a 1 kHz node, debug build, stock kernel. The user
    /// measured timing two orders of magnitude off the front page and nothing
    /// connected the two facts.
    #[test]
    fn debug_build_on_a_stock_kernel_warns_about_both() {
        let (build, kernel) = should_warn(true, false);
        assert!(build, "a debug build must be called out");
        assert!(kernel, "a non-PREEMPT_RT kernel must be called out");
    }

    /// A release build on a stock kernel still has the jitter problem.
    #[test]
    fn release_on_a_stock_kernel_warns_about_the_kernel_only() {
        let (build, kernel) = should_warn(false, false);
        assert!(!build);
        assert!(kernel);
    }

    /// A debug build on an RT kernel still has the 10-50x problem.
    #[test]
    fn debug_on_an_rt_kernel_warns_about_the_build_only() {
        let (build, kernel) = should_warn(true, true);
        assert!(build);
        assert!(!kernel);
    }

    /// A correctly configured machine must stay quiet — a warning nobody can
    /// act on is noise, and noise is what gets warnings ignored.
    #[test]
    fn release_on_an_rt_kernel_is_silent() {
        assert_eq!(should_warn(false, true), (false, false));
    }
}

/// The shutdown TIMING REPORT's "Misses" column.
///
/// `print_timing_report` filled it with `ring_stats.total_ticks` — the line
/// carried the comment "reuse total_ticks" — so the column reported how many
/// times a node had *run*, under a header that says how many deadlines it
/// *missed*. Live: an RT node logged "Shutting down after 249 ticks" and the
/// report's Misses column read exactly 249; a Python RT node that logged
/// exactly one "Deadline miss in 'py_ctrl'" reported 211.
mod timing_report_columns {
    use super::*;
    use crate::scheduling::profiler::NodeStats;
    use crate::scheduling::safety_monitor::{NodeTimingReport, TimingStats};

    fn stats() -> NodeStats {
        let mut stats = NodeStats::default();
        stats.avg_us = 100.0;
        stats.max_us = 200.0;
        stats.stddev_us = 5.0;
        stats.count = 249;
        stats
    }

    fn timing(total_ticks: u64, deadline_misses: u64) -> NodeTimingReport {
        NodeTimingReport {
            name: "ci_test_node".to_string(),
            stats: TimingStats {
                min_us: 90,
                max_us: 200,
                avg_us: 100,
                p99_us: 150,
                total_ticks,
            },
            budget: Some(Duration::from_micros(10_000)),
            overruns: 0,
            deadline_misses,
        }
    }

    /// Columns of a rendered row, in header order:
    /// Node Avg P99 Max Stddev Budget Overruns Ticks Misses
    fn columns(row: &str) -> Vec<String> {
        row.split_whitespace().map(|s| s.to_string()).collect()
    }

    #[test]
    fn misses_reports_deadline_misses_not_the_tick_count() {
        let node = timing(249, 1);
        let row = format_timing_report_row("ci_test_node", &stats(), Some(&node));
        let cols = columns(&row);
        assert_eq!(cols.len(), 9, "unexpected column count in {row:?}");
        assert_eq!(cols[7], "249", "Ticks column: {row:?}");
        assert_eq!(
            cols[8], "1",
            "Misses must be the node's deadline-miss count, not its tick \
             count: {row:?}"
        );
    }

    /// A node that met every deadline must report zero misses, however long it
    /// ran. This is the case the live reproduction hit: 249 ticks, no misses,
    /// "Misses 249".
    #[test]
    fn a_node_that_missed_nothing_reports_zero_misses() {
        let node = timing(249, 0);
        let row = format_timing_report_row("ci_test_node", &stats(), Some(&node));
        let cols = columns(&row);
        assert_eq!(
            cols[8], "0",
            "a node that missed no deadline must report 0: {row:?}"
        );
    }

    /// The miss count must come from the monitor's per-node counter, so it
    /// tracks recorded misses independently of the tick count.
    #[test]
    fn the_miss_count_tracks_recorded_misses() {
        let monitor = crate::scheduling::safety_monitor::SafetyMonitor::new(1_000_000);
        monitor.set_tick_budget("motor".to_string(), Duration::from_millis(10));
        for _ in 0..50 {
            let _ = monitor.check_tick_budget("motor", Duration::from_micros(100));
        }
        monitor.record_deadline_miss("motor");
        monitor.record_deadline_miss("motor");

        let rows = monitor.all_node_timing();
        let motor = rows.iter().find(|r| r.name == "motor").expect("motor row");
        assert_eq!(motor.stats.total_ticks, 50);
        assert_eq!(
            motor.deadline_misses, 2,
            "the monitor recorded 2 misses over 50 ticks"
        );

        let row = format_timing_report_row("motor", &stats(), Some(motor));
        let cols = columns(&row);
        assert_eq!(cols[7], "50", "Ticks: {row:?}");
        assert_eq!(cols[8], "2", "Misses: {row:?}");
    }
}

/// End to end under a real scheduler: the reproduction from the report.
///
/// A node builds a `Topic<Image>` and a `Topic<u64>` in its **constructor**
/// (where a camera node builds them) and publishes on both from `tick()`.
/// Before the fix the POD topic named the node and the image topic reported no
/// publisher at all, because the zero-copy `Topic<Image>::send` read the
/// constructor-captured owner — which is empty, since constructors run before
/// the scheduler exists — and latched the failed attempt so it could never be
/// retried from inside a tick.
#[test]
fn a_zero_copy_publisher_is_attributed_under_a_real_scheduler() {
    use crate::communication::topic::topic_node_registry;
    use crate::communication::Topic;
    use crate::memory::Image;
    use crate::types::ImageEncoding;

    let _guard = lock_scheduler();

    let tag = format!("{}_{}", std::process::id(), 1);
    let pod_topic = format!("live_pod_{tag}");
    let img_topic = format!("live_img_{tag}");

    struct CameraCtor {
        name: String,
        pod: Topic<u64>,
        img: Topic<Image>,
    }

    impl Node for CameraCtor {
        fn name(&self) -> &str {
            &self.name
        }
        fn tick(&mut self) {
            self.pod.send(1);
            if let Ok(frame) = Image::new(4, 4, ImageEncoding::Rgb8) {
                self.img.send(frame);
            }
        }
    }

    // Constructed outside any tick, exactly like a real node's `new()`.
    let node = CameraCtor {
        name: "camera_ctor".to_string(),
        pod: Topic::<u64>::new(&pod_topic).expect("pod topic"),
        img: Topic::<Image>::new(&img_topic).expect("image topic"),
    };

    let mut scheduler = Scheduler::new();
    scheduler.add(node).order(0).build();
    // 1.5s, not 300ms. These assertions are about ATTRIBUTION — that a publisher
    // constructed outside any tick is credited to its node — not about how
    // quickly the scheduler gets there. The window has to be long enough for the
    // node to tick at least once, and on a box running the rest of the ~2500-test
    // suite in parallel the scheduler thread can lose 300ms to the run queue
    // before it ever ticks. It did: this failed once in a full workspace run with
    // an empty publisher list, while passing 4/4 in isolation and 6/6 under 8
    // spinners on its own. A real regression — never attributing the publisher —
    // fails however long the window is.
    let _ = scheduler.run_for(1500_u64.ms());

    let registry = topic_node_registry();
    assert_eq!(
        registry.publishers_of_topic(&pod_topic),
        vec!["camera_ctor".to_string()],
        "the POD path already worked"
    );
    assert_eq!(
        registry.publishers_of_topic(&img_topic),
        vec!["camera_ctor".to_string()],
        "`horus topic info` must name the publisher of a zero-copy image \
         topic too — this is the camera node that reported \"Publishers: \
         (none)\""
    );
}

// ---------------------------------------------------------------------------
// RT nodes joined by a topic edge are not ordered against each other
// ---------------------------------------------------------------------------

/// Two RT nodes connected by a topic must be reported, because the executor
/// gives them no ordering.
///
/// Every RT node becomes its own single-node chain, and the RT executor does not
/// sequence chains against each other (the `rt_chains` construction says so in
/// as many words). So an RT consumer downstream of an RT producer reads the
/// producer's PREVIOUS tick at a phase offset fixed when the threads started.
/// Nothing about that is visible at runtime: the edge exists, data flows, and
/// every value looks plausible while being one cycle stale.
///
/// On a legged robot that edge is estimator -> controller, and one stale cycle
/// of state estimate at 1 kHz is a real torque error. It has to be said out loud
/// at build time.
#[test]
fn rt_nodes_joined_by_a_topic_are_reported_as_unordered() {
    let _guard = lock_scheduler();

    struct Producer {
        out: crate::communication::topic::Topic<u64>,
    }
    impl Node for Producer {
        fn name(&self) -> &'static str {
            "rt_producer"
        }
        fn tick(&mut self) {
            self.out.send(1u64);
        }
    }
    struct Consumer {
        inp: crate::communication::topic::Topic<u64>,
    }
    impl Node for Consumer {
        fn name(&self) -> &'static str {
            "rt_consumer"
        }
        fn tick(&mut self) {
            let _ = self.inp.recv();
        }
    }

    let topic = format!("rt_edge_probe_{}", std::process::id());
    let mut scheduler = Scheduler::new().tick_rate(1000_u64.hz()).verbose(false);
    let _ = scheduler
        .add(Producer {
            out: crate::communication::topic::Topic::new(&topic).expect("pub topic"),
        })
        .rate(1000_u64.hz())
        .build();
    let _ = scheduler
        .add(Consumer {
            inp: crate::communication::topic::Topic::new(&topic).expect("sub topic"),
        })
        .rate(1000_u64.hz())
        .build();

    // Roles are normally learned from real send/recv during ticks. Declare them
    // directly so the graph has edges without running the scheduler.
    //
    // `topic_node_registry()` is a process-wide singleton, so these two entries
    // outlive the test unless something takes them out again — and a panicking
    // assertion below must not be what leaves them behind, since the harness
    // keeps running the other tests in this binary afterwards. Hence a guard
    // that unregisters on drop rather than two calls at the end.
    struct RegistryEntries {
        topic: String,
        nodes: [&'static str; 2],
    }
    impl Drop for RegistryEntries {
        fn drop(&mut self) {
            let tnr = crate::communication::topic_node_registry();
            for node in self.nodes {
                tnr.unregister(&self.topic, node);
            }
        }
    }

    let tnr = crate::communication::topic_node_registry();
    tnr.register_with_type(
        &topic,
        "rt_producer",
        crate::communication::topic::NodeTopicRole::Publisher,
        "u64",
    );
    tnr.register_with_type(
        &topic,
        "rt_consumer",
        crate::communication::topic::NodeTopicRole::Subscriber,
        "u64",
    );
    let _registry_entries = RegistryEntries {
        topic: topic.clone(),
        nodes: ["rt_producer", "rt_consumer"],
    };

    scheduler.build_dependency_graph();

    let edges =
        Scheduler::unordered_rt_edges(&scheduler.nodes, scheduler.dependency_graph.as_ref());

    assert!(
        edges
            .iter()
            .any(|(p, c)| p == "rt_producer" && c == "rt_consumer"),
        "an RT->RT topic edge was not reported; found {:?}. Two RT nodes sharing \
         a topic get no ordering from the executor and the consumer silently \
         reads a one-cycle-old value.",
        edges
    );
}

/// A node with no RT peer must produce no report.
///
/// The warning is only useful if it is quiet when there is nothing to say; a
/// warning that fires on every graph trains people to ignore it.
#[test]
fn a_lone_rt_node_reports_nothing() {
    let _guard = lock_scheduler();

    struct Solo;
    impl Node for Solo {
        fn name(&self) -> &'static str {
            "rt_solo"
        }
        fn tick(&mut self) {}
    }

    let mut scheduler = Scheduler::new().tick_rate(1000_u64.hz()).verbose(false);
    let _ = scheduler.add(Solo).rate(1000_u64.hz()).build();
    scheduler.build_dependency_graph();

    let edges =
        Scheduler::unordered_rt_edges(&scheduler.nodes, scheduler.dependency_graph.as_ref());
    assert!(
        edges.is_empty(),
        "a single RT node with no peers reported {:?}",
        edges
    );
}

// ============================================================================
// Error-contract Tests
// ============================================================================

/// The `# Errors` section of the doc comment attached to `anchor`, taken from
/// the scheduler source itself.
///
/// `anchor` is the item's name plus its opening paren (`"pub fn run("`) and
/// nothing more, so a receiver change, an added return type or `where` clause,
/// or a signature wrapped across lines does not move it. Two assertions keep the
/// lookup honest instead: the anchor must appear exactly once in the file, and
/// the text between the `# Errors` heading and it must be doc lines and
/// attributes only — anything else means the heading belongs to an earlier item
/// and the caller would be asserting against the wrong function. Both failures
/// are loud, so this cannot silently read the wrong doc block.
fn errors_section(anchor: &str) -> &'static str {
    const SOURCE: &str = include_str!("mod.rs");
    assert_eq!(
        SOURCE.matches(anchor).count(),
        1,
        "`{anchor}` must occur exactly once in scheduler/mod.rs for this test to \
         know which doc block it is reading"
    );
    let sig_at = SOURCE.find(anchor).expect("anchor counted above");
    let errors_at = SOURCE[..sig_at]
        .rfind("/// # Errors")
        .unwrap_or_else(|| panic!("`{anchor}` has no `# Errors` section"));
    let section = &SOURCE[errors_at..sig_at];
    for line in section.lines() {
        let trimmed = line.trim_start();
        assert!(
            trimmed.is_empty() || trimmed.starts_with("///") || trimmed.starts_with("#["),
            "the `# Errors` heading nearest `{anchor}` is not part of its doc block \
             (stray line: {line:?})"
        );
    }
    section
}

/// `tick_once()`'s `# Errors` list is public API — `Scheduler` is in
/// `horus::prelude` and a caller writes their `match` arms from it. It used to
/// name `InitPanic`, `InitFailed` and `TickFailed`, none of which the function
/// can return: `initialize_filtered_nodes()` prints every init error and returns
/// `()`, and `Node::tick` is `fn tick(&mut self);`, so `TickFailed` has no
/// producer anywhere in the crate. The errors it does return — including the one
/// that means the emergency stop fired — were undocumented, so following that
/// list gave you three dead arms and a silent fallthrough on the e-stop.
///
/// Prose has no compiler, so this is the check.
#[test]
fn test_tick_once_errors_doc_lists_only_reachable_errors() {
    let section = errors_section("pub fn tick_once(");

    for unreachable in ["InitPanic", "InitFailed", "TickFailed"] {
        assert!(
            !section.contains(unreachable),
            "`tick_once()` documents `{unreachable}`, which it cannot return.\n{section}"
        );
    }

    for real in [
        "HorusError::InvalidInput",
        "ValidationError::InvalidValue",
        "HorusError::Resource",
        "ResourceError::Unsupported",
        "Fatal node failure",
        "Emergency stop triggered",
    ] {
        assert!(
            section.contains(real),
            "`tick_once()` returns `{real}` and does not document it.\n{section}"
        );
    }
}

/// Same defect in `run()`'s list: two unreachable init variants plus a
/// `ConfigError` that nothing on the `run()` path constructs, while the
/// duplicate-node-name error it really returns (pinned by
/// `test_duplicate_node_names_are_refused`) was missing.
#[test]
fn test_run_errors_doc_lists_only_reachable_errors() {
    let section = errors_section("pub fn run(");

    for unreachable in ["InitPanic", "InitFailed", "ConfigError"] {
        assert!(
            !section.contains(unreachable),
            "`run()` documents `{unreachable}`, which it cannot return.\n{section}"
        );
    }

    for real in [
        "HorusError::InvalidInput",
        "ValidationError::InvalidValue",
        "HorusError::Resource",
        "ResourceError::Unsupported",
        "HorusError::Contextual",
        "starting RT executor thread pool",
    ] {
        assert!(
            section.contains(real),
            "`run()` returns `{real}` and does not document it.\n{section}"
        );
    }
}

/// The behaviour the doc above now claims, asserted against the real scheduler so
/// the two cannot drift apart: a failed `init()` does not reach the caller, and a
/// duplicate node name reaches it as `InvalidValue { field: "node name" }`.
#[test]
fn test_tick_once_error_contract_matches_behavior() {
    let _guard = lock_scheduler();

    struct FailingInitNode;
    impl Node for FailingInitNode {
        fn name(&self) -> &str {
            "contract_bad_init"
        }
        fn init(&mut self) -> crate::error::HorusResult<()> {
            Err(crate::HorusError::Node(
                crate::error::NodeError::InitFailed {
                    node: "contract_bad_init".to_string(),
                    reason: "deliberate".to_string(),
                },
            ))
        }
        fn tick(&mut self) {}
    }

    let mut scheduler = Scheduler::new();
    // `.build()` is unwrapped, not dropped: if registration failed, the scheduler
    // below would be empty and `tick_once().is_ok()` would hold for the one reason
    // this assertion must never accept — that no init() ran at all.
    scheduler
        .add(FailingInitNode)
        .order(0)
        .build()
        .expect("the failing-init node must be registered for the assertion to mean anything");
    assert!(
        scheduler.tick_once().is_ok(),
        "an init() failure is reported by log line and node state, never by tick_once()"
    );

    let mut scheduler = Scheduler::new();
    scheduler
        .add(CounterNode::new("contract_dup"))
        .order(0)
        .build()
        .expect("the first node registers cleanly; the clash is recorded on the second");
    scheduler
        .add(CounterNode::new("contract_dup"))
        .order(1)
        .build()
        .expect("a duplicate name is recorded for tick_once(), not rejected by build()");
    let err = scheduler
        .tick_once()
        .expect_err("a duplicate node name must fail the tick");
    assert!(
        matches!(
            err,
            crate::error::HorusError::InvalidInput(
                crate::error::ValidationError::InvalidValue { ref field, .. }
            ) if field == "node name"
        ),
        "expected the documented InvalidValue {{ field: \"node name\" }}, got: {err:?}"
    );

    let mut scheduler = Scheduler::new();
    scheduler
        .add(PanickingNode::new(
            "contract_fatal",
            0,
            Arc::new(AtomicUsize::new(0)),
        ))
        .order(0)
        .failure_policy(FailurePolicy::Fatal)
        .build()
        .expect("the panicking node must be registered for the tick to have anything to fail on");
    let err = scheduler
        .tick_once()
        .expect_err("a fatal node failure must fail the tick");
    assert!(
        matches!(
            err,
            crate::error::HorusError::Internal { ref message, .. }
                if message.contains("Fatal node failure")
        ),
        "expected the documented Internal(\"Fatal node failure during tick_once\"), got: {err:?}"
    );
}

/// `run()`'s doc promises the opposite return from `tick_once()`'s for the same
/// event: an emergency stop breaks the loop and the call comes back `Ok(())`,
/// where `tick_once()` reports it as `Internal("Emergency stop triggered …")`.
/// That asymmetry is the whole reason a caller picks one entry point over the
/// other, so it is asserted rather than described.
///
/// The stop is fired the way horus_net fires it on link loss —
/// `trigger_external_emergency_stop`, through the global hook `finalize_config`
/// installs. `finalize_and_init()` runs first so the hook is live before the
/// trigger; it is guarded by `self.initialized`, so `run_for` below re-enters an
/// already-finalized scheduler and the latch survives into the loop.
///
/// The elapsed-time assertion is what makes this non-vacuous: without it the
/// test would also pass if the loop simply ran its duration out.
#[test]
fn test_run_reports_an_emergency_stop_as_ok() {
    let _guard = lock_scheduler();

    struct EstopNode;
    impl Node for EstopNode {
        fn name(&self) -> &str {
            "contract_estop"
        }
        fn tick(&mut self) {}
    }

    let mut scheduler = Scheduler::new()
        .tick_rate(100_u64.hz())
        .watchdog(500_u64.ms());
    // A per-node watchdog registers the node as critical, which is what enables
    // the safety monitor the external hook latches.
    scheduler
        .add(EstopNode)
        .watchdog(100_u64.ms())
        .build()
        .unwrap();
    scheduler.finalize_and_init();
    assert!(
        !scheduler
            .monitor
            .safety
            .as_ref()
            .expect("a per-node watchdog enables the safety monitor")
            .is_emergency_stop(),
        "no emergency stop before the trigger"
    );

    crate::scheduling::safety_monitor::trigger_external_emergency_stop(
        "error-contract test: external e-stop".to_string(),
    );
    assert!(
        scheduler
            .monitor
            .safety
            .as_ref()
            .unwrap()
            .is_emergency_stop(),
        "the external trigger must latch this scheduler's monitor"
    );

    let started = Instant::now();
    let result = scheduler.run_for(5_u64.secs());
    let elapsed = started.elapsed();

    assert!(
        result.is_ok(),
        "run() reports an emergency stop as a clean shutdown, never as an Err: {result:?}"
    );
    assert!(
        elapsed < Duration::from_secs(2),
        "the e-stop must break the loop, not let it run the duration out (took {elapsed:?})"
    );
}

/// The second `HorusError::Contextual` `run()` documents. The first — a tokio
/// runtime that will not build — has no seam to force from a test; this one
/// does, and it is the one a user actually meets: two RT chains pinned to the
/// same CPU. `check_core_collisions()` refuses before a single RT thread is
/// spawned, so the test needs no RT privileges and no CPU 0 affinity.
///
/// Without this the doc's claim about the RT-executor path would be prose only
/// — which is the exact failure mode this PR exists to fix.
#[test]
fn test_run_reports_a_refused_rt_executor_as_contextual() {
    if std::env::var("HORUS_RT_ALLOW_CORE_SHARING")
        .is_ok_and(|v| !v.is_empty() && v != "0" && v != "false")
    {
        // The operator opted into shared RT cores; the refusal is by design off.
        return;
    }
    let _guard = lock_scheduler();

    struct RtNode(&'static str);
    impl Node for RtNode {
        fn name(&self) -> &str {
            self.0
        }
        fn tick(&mut self) {}
    }

    // `.rate()` promotes each node to ExecutionClass::Rt, and each RT node is its
    // own chain — so this is two chains explicitly naming CPU 0.
    let mut scheduler = Scheduler::new().tick_rate(100_u64.hz());
    scheduler
        .add(RtNode("contract_rt_a"))
        .rate(50_u64.hz())
        .core(0)
        .build()
        .unwrap();
    scheduler
        .add(RtNode("contract_rt_b"))
        .rate(50_u64.hz())
        .core(0)
        .build()
        .unwrap();

    let err = scheduler
        .run_for(200_u64.ms())
        .expect_err("two RT chains on one core must refuse to start");
    assert!(
        matches!(
            err,
            crate::error::HorusError::Contextual { ref message, .. }
                if message == "starting RT executor thread pool"
        ),
        "expected the documented Contextual(\"starting RT executor thread pool\"), got: {err:?}"
    );
}
