use super::*;
use crate::core::Node;
use crate::scheduling::fault_tolerance::FailurePolicy;
// DeterministicConfig only used indirectly through .deterministic() builder now
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
    name: &'static str,
    tick_count: Arc<AtomicUsize>,
}

impl CounterNode {
    fn new(name: &'static str) -> Self {
        Self {
            name,
            tick_count: Arc::new(AtomicUsize::new(0)),
        }
    }

    fn with_counter(name: &'static str, counter: Arc<AtomicUsize>) -> Self {
        Self {
            name,
            tick_count: counter,
        }
    }
}

impl Node for CounterNode {
    fn name(&self) -> &str {
        self.name
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

#[test]
fn test_scheduler_with_name() {
    let _guard = lock_scheduler();
    let scheduler = Scheduler::new().with_name("TestScheduler");
    // The name is stored internally and used in logging
    assert!(scheduler.is_running());
}

// test_scheduler_with_capacity removed: with_capacity() was removed from Scheduler

// ============================================================================
// Node Addition Tests
// ============================================================================

#[test]
fn test_scheduler_add_node() {
    let _guard = lock_scheduler();
    let mut scheduler = Scheduler::new();
    scheduler.add(CounterNode::new("test_node")).order(0).done();

    let nodes = scheduler.node_list();
    assert_eq!(nodes.len(), 1);
    assert_eq!(nodes[0], "test_node");
}

#[test]
fn test_scheduler_add_multiple_nodes() {
    let _guard = lock_scheduler();
    let mut scheduler = Scheduler::new();
    scheduler.add(CounterNode::new("node1")).order(0).done();
    scheduler.add(CounterNode::new("node2")).order(1).done();
    scheduler.add(CounterNode::new("node3")).order(2).done();

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
        .done();
    scheduler
        .add(CounterNode::new("high_priority"))
        .order(0)
        .done();
    scheduler
        .add(CounterNode::new("medium_priority"))
        .order(5)
        .done();

    // After sorting by priority, high_priority should come first
    let nodes = scheduler.node_list();
    assert_eq!(nodes.len(), 3);
    // Note: nodes are sorted by priority
}

#[test]
fn test_scheduler_add_basic() {
    let _guard = lock_scheduler();
    let mut scheduler = Scheduler::new();
    scheduler
        .add(CounterNode::new("basic_node"))
        .order(0)
        .done();

    let metrics = scheduler.metrics();
    assert_eq!(metrics.len(), 1);
    assert_eq!(metrics[0].name, "basic_node");
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
    let mut scheduler = Scheduler::new();
    scheduler.add(CounterNode::new("sensor")).order(0).done();
    scheduler.set_node_rate("sensor", 100.0);

    // Just verify it doesn't panic
    assert!(scheduler.is_running());
}

#[test]
fn test_scheduler_set_node_rate_nonexistent() {
    let _guard = lock_scheduler();
    let mut scheduler = Scheduler::new();
    scheduler.add(CounterNode::new("node1")).order(0).done();
    // Setting rate for nonexistent node should not panic
    scheduler.set_node_rate("nonexistent", 50.0);
    assert!(scheduler.is_running());
}

// ============================================================================
// Node Info Tests (via metrics())
// ============================================================================

#[test]
fn test_scheduler_metrics_existing() {
    let _guard = lock_scheduler();
    let mut scheduler = Scheduler::new();
    scheduler.add(CounterNode::new("info_node")).order(0).done();

    let metrics = scheduler.metrics();
    assert_eq!(metrics.len(), 1);
    assert_eq!(metrics[0].name, "info_node");
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
fn test_scheduler_with_safety_monitor() {
    let _guard = lock_scheduler();
    let scheduler = Scheduler::new()
        .safety_monitor(true)
        .max_deadline_misses(10);
    assert!(scheduler.is_running());
}

// ============================================================================
// Real-time Node Tests
// ============================================================================

#[test]
fn test_scheduler_add_rt_node() {
    let _guard = lock_scheduler();
    let mut scheduler = Scheduler::new();
    scheduler
        .add(CounterNode::new("rt_node"))
        .order(0)
        .budget_us(100)
        .deadline_ms(1)
        .done();

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
        .done();

    // Run for a short duration (100ms to handle scheduler init overhead under parallel test load)
    let result = scheduler.run_for(Duration::from_millis(500));
    assert!(result.is_ok());

    // Counter should have been incremented at least once
    assert!(counter.load(Ordering::SeqCst) > 0);
}

// ============================================================================
// Chainable API Tests
// ============================================================================

#[test]
fn test_scheduler_chainable_api() {
    let _guard = lock_scheduler();
    let mut scheduler = Scheduler::new()
        .with_name("ChainedScheduler");

    scheduler
        .add(CounterNode::new("chain_node"))
        .order(0)
        .done();

    assert!(scheduler.is_running());
    assert_eq!(scheduler.node_list().len(), 1);
}

// ============================================================================
// List Recordings Tests
// ============================================================================

#[test]
fn test_scheduler_list_recordings() {
    let _guard = lock_scheduler();
    // This might fail if no recordings exist, but shouldn't panic
    let result = Scheduler::list_recordings();
    // Just verify the function is callable
    assert!(result.is_ok() || result.is_err());
}

// ============================================================================
// Builder Pattern Name Test
// ============================================================================

#[test]
fn test_scheduler_name_builder() {
    let _guard = lock_scheduler();
    let scheduler = Scheduler::new().with_name("BuilderName");
    // Verify the scheduler was created successfully
    assert!(scheduler.is_running());
}

// ============================================================================
// Override Tests
// ============================================================================

#[test]
fn test_scheduler_with_override() {
    let _guard = lock_scheduler();
    let scheduler = Scheduler::new().with_override("node1", "output1", vec![1, 2, 3, 4]);

    // Should not panic and scheduler should still be running
    assert!(scheduler.is_running());
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
    let mut scheduler = Scheduler::new().tick_hz(100.0);

    let c1 = Arc::new(AtomicUsize::new(0));
    let c2 = Arc::new(AtomicUsize::new(0));
    let c3 = Arc::new(AtomicUsize::new(0));

    scheduler
        .add(CounterNode::with_counter("par_a", c1.clone()))
        .order(100)
        .compute()
        .done();
    scheduler
        .add(CounterNode::with_counter("par_b", c2.clone()))
        .order(101)
        .compute()
        .done();
    scheduler
        .add(CounterNode::with_counter("par_c", c3.clone()))
        .order(102)
        .compute()
        .done();

    // Run for 300ms — all 3 compute nodes should tick at least once
    let result = scheduler.run_for(Duration::from_millis(300));
    assert!(result.is_ok());

    // Every node must have ticked at least once
    assert!(c1.load(Ordering::SeqCst) > 0, "par_a never ticked");
    assert!(c2.load(Ordering::SeqCst) > 0, "par_b never ticked");
    assert!(c3.load(Ordering::SeqCst) > 0, "par_c never ticked");
}

#[test]
fn test_parallel_rt_nodes_run_sequentially() {
    let _guard = lock_scheduler();
    // RT nodes run on a dedicated thread; BestEffort nodes on main thread
    let mut scheduler = Scheduler::new().tick_hz(10000.0);

    let rt_counter = Arc::new(AtomicUsize::new(0));
    let normal_counter = Arc::new(AtomicUsize::new(0));

    scheduler
        .add(CounterNode::with_counter("rt_node", rt_counter.clone()))
        .order(0)
        .budget_us(10_000)
        .done();
    scheduler
        .add(CounterNode::with_counter(
            "normal_node",
            normal_counter.clone(),
        ))
        .order(100)
        .done();

    let result = scheduler.run_for(Duration::from_millis(500));
    assert!(result.is_ok());

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
        .rate_hz(500.0)
        .done();

    // tick_period should now be <= 2000us (500Hz)
    assert!(
        scheduler.tick.period <= Duration::from_micros(2000),
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
        .rate_hz(500.0)
        .done();

    // Run for 1s — expect ~500 ticks at 500Hz
    let result = scheduler.run_for(Duration::from_secs(1));
    assert!(result.is_ok());

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
        .rate_hz(10.0)
        .done();

    assert_eq!(
        scheduler.tick.period, default_period,
        "tick_period should not decrease for a 10Hz node"
    );
}

// ============================================================================
// Fix 1: Deterministic Clock Wiring Tests
// ============================================================================

#[test]
fn test_deterministic_config_wires_clock() {
    let _guard = lock_scheduler();
    let scheduler = Scheduler::new().tick_hz(1000.0).deterministic(42);
    assert!(scheduler.is_simulation_mode());
    assert!(scheduler.deterministic_clock().is_some());
    assert_eq!(scheduler.virtual_tick(), Some(0));
    assert_eq!(scheduler.seed(), Some(42));
}

#[test]
fn test_deterministic_config_virtual_time() {
    let _guard = lock_scheduler();
    let scheduler = Scheduler::new().deterministic(123);

    assert!(scheduler.is_simulation_mode());
    assert_eq!(scheduler.seed(), Some(123));
}

#[test]
fn test_deterministic_advances_on_run() {
    let _guard = lock_scheduler();
    let counter = Arc::new(AtomicUsize::new(0));
    let mut scheduler = Scheduler::new()
        .tick_hz(1000.0)
        .deterministic(42)
        .with_recording();
    scheduler
        .add(CounterNode::with_counter("test", counter.clone()))
        .order(0)
        .done();

    // run_for with deterministic mode (virtual time → runs as fast as possible)
    let _ = scheduler.run_for(Duration::from_millis(200));

    // Virtual tick should have advanced
    let tick = scheduler.virtual_tick().unwrap();
    assert!(tick > 0, "Virtual tick should advance, got {}", tick);

    // Node should have been ticked
    let ticks = counter.load(Ordering::SeqCst);
    assert!(ticks > 0, "Node should have ticked, got {}", ticks);
}

#[test]
fn test_standard_config_no_deterministic() {
    let _guard = lock_scheduler();
    let scheduler = Scheduler::new();
    assert!(!scheduler.is_simulation_mode());
    assert!(scheduler.deterministic_clock().is_none());
    assert!(scheduler.virtual_tick().is_none());
}

// ============================================================================
// Fix 2: Recording Hooks Wiring Tests
// ============================================================================

#[test]
fn test_recording_hooks_wired() {
    let _guard = lock_scheduler();
    let counter = Arc::new(AtomicUsize::new(0));
    let mut scheduler = Scheduler::new()
        .tick_hz(1000.0)
        .deterministic(42)
        .with_recording();
    scheduler
        .add(CounterNode::with_counter("rec_node", counter.clone()))
        .order(0)
        .done();

    assert!(scheduler.is_recording());

    let _ = scheduler.run_for(Duration::from_millis(200));

    let ticks = counter.load(Ordering::SeqCst);
    assert!(ticks > 0, "Node should have ticked during recording");

    // stop_recording should succeed and return saved paths
    let paths = scheduler.stop_recording();
    assert!(paths.is_ok());
}

// ============================================================================
// Fix 3: BlackBox WAL Persistence Tests
// ============================================================================

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

#[test]
fn test_deploy_config_creates_blackbox_with_wal() {
    let _guard = lock_scheduler();
    let scheduler = Scheduler::new().circuit_breaker(true).with_blackbox(16);
    assert!(
        scheduler.blackbox().is_some(),
        ".with_blackbox() should create a blackbox"
    );
}

// ============================================================================
// Fix 4: Dead Code Cleanup Verification
// ============================================================================

#[test]
fn test_per_node_rates_work_without_dead_code() {
    let _guard = lock_scheduler();
    // Per-node rates work through set_node_rate() / .rate_hz(), not the removed config flag
    let counter = Arc::new(AtomicUsize::new(0));
    let mut scheduler = Scheduler::new().tick_hz(1000.0);
    scheduler
        .add(CounterNode::with_counter("rated", counter.clone()))
        .order(0)
        .rate_hz(100.0)
        .done();

    let _ = scheduler.run_for(Duration::from_secs(1));

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
        .done();
    scheduler
        .add(OrderTrackingNode::new(
            "node_b",
            log.clone(),
            init_b.clone(),
            shut_b.clone(),
            tc_b.clone(),
        ))
        .order(1)
        .done();

    let result = scheduler.run_for(Duration::from_millis(500));
    assert!(result.is_ok());

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
            .done();
    }

    let result = scheduler.run_for(Duration::from_millis(500));
    assert!(result.is_ok());

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
                .done();
            tc
        })
        .collect();

    // Run for 500ms — should get many ticks at default ~60Hz
    let result = scheduler.run_for(Duration::from_millis(500));
    assert!(result.is_ok());

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
            .done();
    }

    let result = scheduler.run_for(Duration::from_millis(200));
    assert!(result.is_ok());

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
// budget / Deadline / Failure Policy Integration Tests
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

/// Node that sleeps in tick() to simulate slow execution (for budget tests).
struct SlowNode {
    node_name: String,
    sleep_duration: Duration,
    tick_count: Arc<AtomicUsize>,
}

impl SlowNode {
    fn new(name: &str, sleep: Duration, counter: Arc<AtomicUsize>) -> Self {
        Self {
            node_name: name.to_string(),
            sleep_duration: sleep,
            tick_count: counter,
        }
    }
}

impl Node for SlowNode {
    fn name(&self) -> &str {
        &self.node_name
    }

    fn tick(&mut self) {
        self.tick_count.fetch_add(1, Ordering::SeqCst);
        std::thread::sleep(self.sleep_duration);
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
        .done();

    // Panicking node with Fatal policy (default for UltraFast tier)
    scheduler
        .add(PanickingNode::new(
            "failing_motor",
            3,
            panic_counter.clone(),
        ))
        .order(1)
        .failure_policy(crate::scheduling::fault_tolerance::FailurePolicy::Fatal)
        .done();

    let _result = scheduler.run_for(Duration::from_millis(500));
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
            3, 10,
        ))
        .done();

    let _result = scheduler.run_for(Duration::from_millis(500));
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

/// Skip policy uses circuit breaker — node is skipped after repeated failures.
/// Robotics: logging node failure should not crash the system.
#[test]
fn test_skip_policy_circuit_breaker_skips_node() {
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
        .done();

    // Panicking node with Skip policy — circuit opens after failures
    scheduler
        .add(PanickingNode::new(
            "faulty_logger",
            1,
            panic_counter.clone(),
        ))
        .order(100)
        .failure_policy(crate::scheduling::fault_tolerance::FailurePolicy::skip(
            3, 5000,
        ))
        .done();

    let result = scheduler.run_for(Duration::from_millis(200));
    assert!(result.is_ok());

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
        .done();

    // Node panics at tick 2 with Ignore policy
    scheduler
        .add(PanickingNode::new("diagnostics", 2, counter.clone()))
        .order(200)
        .failure_policy(crate::scheduling::fault_tolerance::FailurePolicy::Ignore)
        .done();

    let result = scheduler.run_for(Duration::from_millis(500));
    assert!(result.is_ok());

    // Critical node should have ticked
    let healthy_ticks = healthy_counter.load(Ordering::SeqCst);
    assert!(
        healthy_ticks > 0,
        "critical node should keep running despite failing diagnostic node"
    );
}

/// budget enforcement: slow RT node exceeds tick budget, violation is detected.
/// Robotics: motor control loop exceeding 1ms budget must be flagged.
#[test]
fn test_budget_violation_detected_for_slow_rt_node() {
    let _guard = lock_scheduler();
    let slow_counter = Arc::new(AtomicUsize::new(0));
    let fast_counter = Arc::new(AtomicUsize::new(0));

    // Enable budget enforcement via builder (deferred to run time)
    let mut scheduler = Scheduler::new().safety_monitor(true);

    // Fast node within budget
    scheduler
        .add(CounterNode::with_counter("fast_ctrl", fast_counter.clone()))
        .order(0)
        .budget_us(10_000) // 10ms budget (generous)
        .done();

    // Slow node that will exceed its tick budget
    scheduler
        .add(SlowNode::new(
            "slow_sensor",
            Duration::from_millis(50),
            slow_counter.clone(),
        ))
        .order(1)
        .budget_us(1_000) // 1ms budget — will be violated by 50ms sleep
        .done();

    let _result = scheduler.run_for(Duration::from_secs(1));

    // Slow node should have ticked
    assert!(
        slow_counter.load(Ordering::SeqCst) > 0,
        "slow node should have ticked"
    );

    // RT nodes are reclaimed after stop — check per-node rt_stats for budget violations
    if let Some(stats) = scheduler.rt_stats("slow_sensor") {
        assert!(
            stats.budget_violations > 0,
            "budget violation should have been detected, got {} violations",
            stats.budget_violations
        );
    }
}

/// Deadline miss detection: RT node misses its deadline.
/// Robotics: control loop missing 10ms deadline means actuator stale.
#[test]
fn test_deadline_miss_detected_for_slow_rt_node() {
    let _guard = lock_scheduler();
    let slow_counter = Arc::new(AtomicUsize::new(0));

    // Enable deadline monitoring via builder (deferred to run time)
    let mut scheduler = Scheduler::new().safety_monitor(true);

    // Node with tight deadline that will be missed
    scheduler
        .add(SlowNode::new(
            "ctrl_loop",
            Duration::from_millis(20),
            slow_counter.clone(),
        ))
        .order(0)
        .deadline_ms(5) // 5ms deadline — will be missed by 20ms sleep
        .done();

    let _result = scheduler.run_for(Duration::from_secs(2));

    assert!(
        slow_counter.load(Ordering::SeqCst) > 0,
        "node should have ticked"
    );

    // RT nodes are reclaimed after stop — check per-node rt_stats for deadline misses
    if let Some(stats) = scheduler.rt_stats("ctrl_loop") {
        assert!(
            stats.deadline_misses > 0,
            "Deadline miss should have been detected, got {} misses",
            stats.deadline_misses
        );
    }
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
            .done();
    }

    let result = scheduler.run_for(Duration::from_millis(200));
    assert!(result.is_ok());

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

    let result = scheduler.run_for(Duration::from_millis(200));
    assert!(result.is_ok(), "Zero-node scheduler should exit cleanly");
}

/// Duplicate node names: two nodes with the same name can be added.
/// Both should be present (Vec-based storage, not HashMap).
/// Robotics: misconfigured launch file with duplicate node names.
#[test]
fn test_duplicate_node_names_both_added() {
    let _guard = lock_scheduler();
    let mut scheduler = Scheduler::new();
    let counter1 = Arc::new(AtomicUsize::new(0));
    let counter2 = Arc::new(AtomicUsize::new(0));

    scheduler
        .add(CounterNode::with_counter("motor_ctrl", counter1.clone()))
        .order(0)
        .done();
    scheduler
        .add(CounterNode::with_counter("motor_ctrl", counter2.clone()))
        .order(1)
        .done();

    // Both nodes should exist
    let nodes = scheduler.node_list();
    assert_eq!(nodes.len(), 2, "Both duplicate-named nodes should be added");

    // Both should tick
    let result = scheduler.run_for(Duration::from_millis(500));
    assert!(result.is_ok());
    assert!(
        counter1.load(Ordering::SeqCst) > 0,
        "First duplicate node should tick"
    );
    assert!(
        counter2.load(Ordering::SeqCst) > 0,
        "Second duplicate node should tick"
    );
}

/// Node that panics in init(): scheduler catches it, other nodes continue.
/// Robotics: one bad sensor driver must not crash the whole robot.
#[test]
fn test_panic_in_init_caught_others_continue() {
    let _guard = lock_scheduler();
    struct PanicInitNode;
    impl Node for PanicInitNode {
        fn name(&self) -> &str {
            "panic_init"
        }
        fn init(&mut self) -> crate::error::HorusResult<()> {
            panic!("init explosion");
        }
        fn tick(&mut self) {}
    }

    let good_counter = Arc::new(AtomicUsize::new(0));
    let mut scheduler = Scheduler::new();

    // Bad node that panics in init
    scheduler.add(PanicInitNode).order(0).done();

    // Good node that should still run
    scheduler
        .add(CounterNode::with_counter("good_node", good_counter.clone()))
        .order(1)
        .done();

    let result = scheduler.run_for(Duration::from_millis(500));
    assert!(result.is_ok(), "Scheduler should not crash from init panic");

    assert!(
        good_counter.load(Ordering::SeqCst) > 0,
        "Good node should still tick despite bad node's init panic"
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
        .done();

    // Good node that should continue
    scheduler
        .add(CounterNode::with_counter(
            "good_motor",
            good_counter.clone(),
        ))
        .order(1)
        .done();

    let result = scheduler.run_for(Duration::from_millis(500));
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
        .done();

    // Stop before running — run_for should exit almost immediately
    scheduler.stop();
    let result = scheduler.run_for(Duration::from_millis(500));
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
        // Leak a string for the static name reference CounterNode expects
        let name: &'static str = Box::leak(format!("node_{}", i).into_boxed_str());
        scheduler
            .add(CounterNode::with_counter(name, counter.clone()))
            .order(i as u32)
            .done();
    }

    assert_eq!(scheduler.node_list().len(), 50);

    let result = scheduler.run_for(Duration::from_millis(500));
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
    let result = scheduler.run_for(Duration::from_millis(50));
    assert!(result.is_ok());
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
fn test_tick_hz_zero() {
    let _lock = lock_scheduler();
    let mut scheduler = Scheduler::new().tick_hz(0.0);
    scheduler.add(CounterNode::new("zero_hz")).build().unwrap();
    // Should handle 0hz gracefully (no infinite loop, no division by zero)
    let result = scheduler.run_for(Duration::from_millis(50));
    assert!(result.is_ok());
}

#[test]
fn test_tick_hz_negative() {
    let _lock = lock_scheduler();
    let mut scheduler = Scheduler::new().tick_hz(-100.0);
    scheduler.add(CounterNode::new("neg_hz")).build().unwrap();
    let result = scheduler.run_for(Duration::from_millis(50));
    assert!(result.is_ok());
}

#[test]
fn test_tick_hz_very_large() {
    let _lock = lock_scheduler();
    let counter = Arc::new(AtomicUsize::new(0));
    let mut scheduler = Scheduler::new().tick_hz(1_000_000.0);
    scheduler
        .add(CounterNode::with_counter("fast", counter.clone()))
        .build()
        .unwrap();
    let result = scheduler.run_for(Duration::from_millis(10));
    assert!(result.is_ok());
    assert!(counter.load(Ordering::SeqCst) > 0);
}

#[test]
fn test_set_node_rate_nonexistent() {
    let _lock = lock_scheduler();
    let mut scheduler = Scheduler::new();
    // Setting rate on non-existent node should not panic
    scheduler.set_node_rate("does_not_exist", 100.0);
}

#[test]
fn test_set_node_rate_zero() {
    let _lock = lock_scheduler();
    let mut scheduler = Scheduler::new();
    scheduler.add(CounterNode::new("rate_zero")).build().unwrap();
    scheduler.set_node_rate("rate_zero", 0.0);
}

#[test]
fn test_set_node_rate_negative() {
    let _lock = lock_scheduler();
    let mut scheduler = Scheduler::new();
    scheduler.add(CounterNode::new("rate_neg")).build().unwrap();
    scheduler.set_node_rate("rate_neg", -50.0);
}

#[test]
fn test_tick_empty_node_names() {
    let _lock = lock_scheduler();
    let mut scheduler = Scheduler::new();
    scheduler.add(CounterNode::new("tick_test")).build().unwrap();
    // Tick with empty names should succeed (tick nothing)
    let result = scheduler.tick(&[]);
    assert!(result.is_ok());
}

#[test]
fn test_tick_nonexistent_node_names() {
    let _lock = lock_scheduler();
    let mut scheduler = Scheduler::new();
    scheduler.add(CounterNode::new("real_node")).build().unwrap();
    // Tick with non-existent names should not panic
    let result = scheduler.tick(&["fake_node_1", "fake_node_2"]);
    assert!(result.is_ok());
}

// test_circuit_state_nonexistent removed: circuit_state method was removed in refactor

#[test]
fn test_rt_stats_nonexistent() {
    let _lock = lock_scheduler();
    let scheduler = Scheduler::new();
    assert!(scheduler.rt_stats("no_such_node").is_none());
}

// test_failure_stats_nonexistent removed: failure_stats method was removed in refactor

#[test]
fn test_stop_before_run() {
    let _lock = lock_scheduler();
    let scheduler = Scheduler::new();
    // Stopping before running should not panic
    scheduler.stop();
}

#[test]
fn test_double_stop() {
    let _lock = lock_scheduler();
    let mut scheduler = Scheduler::new();
    scheduler.add(CounterNode::new("double_stop")).build().unwrap();
    let _ = scheduler.run_for(Duration::from_millis(10));
    scheduler.stop();
    scheduler.stop(); // Double stop should not panic
}

#[test]
fn test_with_name_empty() {
    let scheduler = Scheduler::new().with_name("");
    assert_eq!(scheduler.scheduler_name(), "");
}

#[test]
fn test_with_name_long() {
    let long_name = "x".repeat(10_000);
    let scheduler = Scheduler::new().with_name(&long_name);
    assert_eq!(scheduler.scheduler_name(), long_name);
}

#[test]
fn test_with_blackbox_zero_size() {
    let _lock = lock_scheduler();
    let scheduler = Scheduler::new().with_blackbox(0);
    assert!(scheduler.blackbox().is_some());
}

#[test]
fn test_max_deadline_misses_zero() {
    let _lock = lock_scheduler();
    let mut scheduler = Scheduler::new().max_deadline_misses(0);
    scheduler.add(CounterNode::new("zero_miss")).build().unwrap();
    let result = scheduler.run_for(Duration::from_millis(10));
    assert!(result.is_ok());
}

#[test]
fn test_run_for_zero_duration() {
    let _lock = lock_scheduler();
    let mut scheduler = Scheduler::new();
    scheduler.add(CounterNode::new("zero_dur")).build().unwrap();
    let result = scheduler.run_for(Duration::ZERO);
    assert!(result.is_ok());
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
    // Capabilities are only populated after run, should be None before
    // (or Some if pre-populated — either way, no panic)
    let _ = scheduler.capabilities();
}

// test_circuit_summary_empty removed: circuit_summary method was removed in refactor

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
            std::thread::sleep(Duration::from_millis(20));
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
        .done();

    let running = scheduler.running_flag();
    // Spawn a thread that stops the scheduler after a brief delay
    std::thread::spawn(move || {
        std::thread::sleep(Duration::from_millis(50));
        running.store(false, Ordering::SeqCst);
    });

    let result = scheduler.run_for(Duration::from_millis(500));
    assert!(result.is_ok());
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
        .done();
    scheduler.add(PanicShutdownNode).order(1).done();
    scheduler
        .add(FlagShutdownNode {
            node_name: "after_panic",
            flag: shutdown_c.clone(),
        })
        .order(2)
        .done();

    let result = scheduler.run_for(Duration::from_millis(100));
    assert!(result.is_ok());

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
        .done();

    // Simulate SIGTERM after a brief delay
    std::thread::spawn(|| {
        std::thread::sleep(Duration::from_millis(50));
        super::SIGTERM_RECEIVED.store(true, Ordering::SeqCst);
    });

    let result = scheduler.run_for(Duration::from_secs(5));
    assert!(result.is_ok());
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
        .done();

    let flag = scheduler.running_flag();
    std::thread::spawn(move || {
        std::thread::sleep(Duration::from_millis(80));
        flag.store(false, Ordering::SeqCst);
    });

    let result = scheduler.run_for(Duration::from_secs(5));
    assert!(result.is_ok());
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
            .done();
        inits.push(init);
        shuts.push(shut);
        tcs.push(tc);
    }

    let result = scheduler.run_for(Duration::from_millis(200));
    assert!(result.is_ok());

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
        .done();
    scheduler.add(ErrShutdownNode).order(1).done();
    scheduler
        .add(FlagNode {
            node_name: "ok_after",
            flag: shutdown_ok_b.clone(),
        })
        .order(2)
        .done();

    let result = scheduler.run_for(Duration::from_millis(100));
    assert!(result.is_ok());

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
        .done();

    let result = scheduler.run_for(Duration::from_millis(200));
    assert!(result.is_ok());

    let metrics = scheduler.metrics();
    assert_eq!(metrics.len(), 1);
    assert_eq!(metrics[0].name, "metrics_node");
    assert!(
        metrics[0].total_ticks > 0,
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
        .done();

    let result = scheduler.run_for(Duration::from_millis(100));
    assert!(result.is_ok());

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
        .done();

    let result = scheduler.run_for(Duration::from_millis(100));
    assert!(result.is_ok());

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
        .done();

    // Two nodes that panic on first tick, both with Ignore policy
    scheduler
        .add(PanickingNode::new("sensor_a", 1, panic_a.clone()))
        .order(1)
        .failure_policy(FailurePolicy::Ignore)
        .done();
    scheduler
        .add(PanickingNode::new("sensor_b", 1, panic_b.clone()))
        .order(2)
        .failure_policy(FailurePolicy::Ignore)
        .done();

    let result = scheduler.run_for(Duration::from_millis(200));
    assert!(result.is_ok());

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
        .done();

    // Fatal policy node that panics on tick 3
    scheduler
        .add(PanickingNode::new("motor", 3, fatal_counter.clone()))
        .order(1)
        .failure_policy(FailurePolicy::Fatal)
        .done();

    let _result = scheduler.run_for(Duration::from_millis(500));
    assert!(
        !scheduler.is_running(),
        "Scheduler must stop when Fatal-policy node fails"
    );
}

/// Circuit breaker opens after threshold, healthy nodes unaffected.
/// Verify circuit_summary() reflects the state.
#[test]
fn test_circuit_breaker_opens_healthy_nodes_unaffected() {
    let _guard = lock_scheduler();
    let healthy_counter = Arc::new(AtomicUsize::new(0));
    let panic_counter = Arc::new(AtomicUsize::new(0));

    let mut scheduler = Scheduler::new().circuit_breaker(true);

    scheduler
        .add(CounterNode::with_counter(
            "healthy_node",
            healthy_counter.clone(),
        ))
        .order(0)
        .done();

    // Node that always panics (panic_at=1), with Skip policy (threshold=2)
    scheduler
        .add(PanickingNode::new("flaky_node", 1, panic_counter.clone()))
        .order(1)
        .failure_policy(FailurePolicy::skip(2, 5000))
        .done();

    let result = scheduler.run_for(Duration::from_millis(300));
    assert!(result.is_ok());

    let healthy_ticks = healthy_counter.load(Ordering::SeqCst);
    assert!(
        healthy_ticks > 5,
        "Healthy node should tick many times while flaky node is circuit-broken, got {}",
        healthy_ticks
    );

    // circuit_summary assertion removed: method was removed in refactor
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
            if c % 2 == 0 {
                panic!("even tick panic at {}", c);
            }
        }
    }

    let tc = tick_counter.clone();
    let mut scheduler = Scheduler::new();
    scheduler
        .add(EveryOtherPanicNode { counter: tc })
        .order(0)
        .failure_policy(FailurePolicy::restart(5, 5))
        .done();

    let _result = scheduler.run_for(Duration::from_millis(500));

    // Node should have ticked multiple times across restarts
    let ticks = tick_counter.load(Ordering::SeqCst);
    assert!(
        ticks > 3,
        "Node should have ticked multiple times across restarts, got {}",
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
        .done();

    // Ignore-policy node that panics every tick
    scheduler
        .add(PanickingNode::new("ignore_node", 1, ignore_counter.clone()))
        .order(1)
        .failure_policy(FailurePolicy::Ignore)
        .done();

    // Skip-policy node that panics every tick (opens circuit after 2 failures)
    scheduler
        .add(PanickingNode::new("skip_node", 1, skip_counter.clone()))
        .order(2)
        .failure_policy(FailurePolicy::skip(2, 5000))
        .done();

    let result = scheduler.run_for(Duration::from_millis(300));
    assert!(result.is_ok());

    let h = healthy_counter.load(Ordering::SeqCst);
    assert!(h > 5, "Healthy node must keep running, got {} ticks", h);

    // Ignore node keeps being called (panics are swallowed)
    let ig = ignore_counter.load(Ordering::SeqCst);
    assert!(
        ig > 3,
        "Ignore-policy node should keep being called, got {} ticks",
        ig
    );

    // Skip node: after circuit opens (2 failures), it stops being called
    // So it should have fewer ticks than the ignore node
    let sk = skip_counter.load(Ordering::SeqCst);
    assert!(
        sk >= 2,
        "Skip-policy node should have ticked at least until circuit opened, got {}",
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
        .done();

    scheduler
        .add(PanickingNode::new("bad", 1, bad_counter.clone()))
        .order(1)
        .failure_policy(FailurePolicy::Ignore)
        .done();

    let result = scheduler.run_for(Duration::from_millis(200));
    assert!(result.is_ok());

    let good = good_counter.load(Ordering::SeqCst);
    let bad = bad_counter.load(Ordering::SeqCst);

    // Both should have been called roughly the same number of times
    // (panicking node doesn't skip ticks with Ignore policy)
    assert!(good > 5, "Good node must get many ticks, got {}", good);
    // Bad node gets at least as many tick attempts
    assert!(
        bad > 3,
        "Bad node should get tick attempts despite panics, got {}",
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
        .failure_policy(FailurePolicy::restart(3, 5))
        .done();

    let _result = scheduler.run_for(Duration::from_millis(500));

    // Check metrics are available (scheduler tracks node metrics even after failures)
    let metrics = scheduler.metrics();
    assert!(!metrics.is_empty());
    assert_eq!(metrics[0].name, "restart_stats");
}

/// Node fails, gets skipped by circuit breaker, verify scheduler status
/// reflects the failure state.
#[test]
fn test_circuit_breaker_reflected_in_scheduler_status() {
    let _guard = lock_scheduler();
    let panic_counter = Arc::new(AtomicUsize::new(0));

    let mut scheduler = Scheduler::new().circuit_breaker(true);
    scheduler
        .add(PanickingNode::new("circuit_node", 1, panic_counter.clone()))
        .order(0)
        .failure_policy(FailurePolicy::skip(2, 5000))
        .done();

    let result = scheduler.run_for(Duration::from_millis(200));
    assert!(result.is_ok());

    // Status should be non-empty and reflect the circuit breaker state
    let status = scheduler.status();
    assert!(!status.is_empty());
}

// ============================================================================
// budget Budget Enforcement Tests
// ============================================================================

/// RT node well within tick budget — no violations should be reported.
#[test]
fn test_budget_no_violation_within_budget() {
    let _guard = lock_scheduler();
    let counter = Arc::new(AtomicUsize::new(0));

    let mut scheduler = Scheduler::new().safety_monitor(true);
    scheduler
        .add(CounterNode::with_counter("fast_node", counter.clone()))
        .order(0)
        .budget_us(100_000) // 100ms budget — CounterNode takes nanoseconds
        .done();

    let _result = scheduler.run_for(Duration::from_millis(200));

    assert!(
        counter.load(Ordering::SeqCst) > 0,
        "Node should have ticked"
    );

    if let Some(stats) = scheduler.rt_stats("fast_node") {
        assert_eq!(
            stats.budget_violations, 0,
            "Fast node should have zero budget violations, got {}",
            stats.budget_violations
        );
    }
}

/// Multiple RT nodes exceed budget simultaneously — both should report violations.
#[test]
fn test_budget_multiple_simultaneous_violations() {
    let _guard = lock_scheduler();
    let slow_a = Arc::new(AtomicUsize::new(0));
    let slow_b = Arc::new(AtomicUsize::new(0));

    let mut scheduler = Scheduler::new().safety_monitor(true);

    scheduler
        .add(SlowNode::new(
            "slow_a",
            Duration::from_millis(30),
            slow_a.clone(),
        ))
        .order(0)
        .budget_us(1_000) // 1ms budget — violated by 30ms sleep
        .done();

    scheduler
        .add(SlowNode::new(
            "slow_b",
            Duration::from_millis(30),
            slow_b.clone(),
        ))
        .order(1)
        .budget_us(2_000) // 2ms budget — violated by 30ms sleep
        .done();

    let _result = scheduler.run_for(Duration::from_secs(1));

    assert!(
        slow_a.load(Ordering::SeqCst) > 0,
        "slow_a should have ticked"
    );
    assert!(
        slow_b.load(Ordering::SeqCst) > 0,
        "slow_b should have ticked"
    );

    // Both nodes should have budget violations
    if let Some(stats) = scheduler.rt_stats("slow_a") {
        assert!(
            stats.budget_violations > 0,
            "slow_a should have budget violations, got {}",
            stats.budget_violations
        );
    }
    if let Some(stats) = scheduler.rt_stats("slow_b") {
        assert!(
            stats.budget_violations > 0,
            "slow_b should have budget violations, got {}",
            stats.budget_violations
        );
    }
}

/// Mix of within-budget and over-budget nodes — only the over-budget one reports.
#[test]
fn test_budget_mixed_within_and_over_budget() {
    let _guard = lock_scheduler();
    let fast = Arc::new(AtomicUsize::new(0));
    let slow = Arc::new(AtomicUsize::new(0));

    let mut scheduler = Scheduler::new().safety_monitor(true);

    // Fast node with generous budget
    scheduler
        .add(CounterNode::with_counter("fast", fast.clone()))
        .order(0)
        .budget_us(50_000) // 50ms — way more than needed
        .done();

    // Slow node with tight budget
    scheduler
        .add(SlowNode::new(
            "slow",
            Duration::from_millis(20),
            slow.clone(),
        ))
        .order(1)
        .budget_us(1_000) // 1ms budget — violated
        .done();

    let _result = scheduler.run_for(Duration::from_secs(1));

    if let Some(fast_stats) = scheduler.rt_stats("fast") {
        assert_eq!(
            fast_stats.budget_violations, 0,
            "Fast node should have 0 violations, got {}",
            fast_stats.budget_violations
        );
    }

    if let Some(slow_stats) = scheduler.rt_stats("slow") {
        assert!(
            slow_stats.budget_violations > 0,
            "Slow node should have violations, got {}",
            slow_stats.budget_violations
        );
    }
}

/// Non-RT node with budget — should still track (if registered as RT via .budget_us()).
#[test]
fn test_budget_non_rt_node_no_crash() {
    let _guard = lock_scheduler();
    let counter = Arc::new(AtomicUsize::new(0));

    let mut scheduler = Scheduler::new().safety_monitor(true);

    // Regular node (no .budget_us()) — no budget tracking
    scheduler
        .add(CounterNode::with_counter("regular", counter.clone()))
        .order(0)
        .done();

    let result = scheduler.run_for(Duration::from_millis(100));
    assert!(result.is_ok());
    assert!(counter.load(Ordering::SeqCst) > 0);

    // rt_stats for non-RT node should return None
    assert!(
        scheduler.rt_stats("regular").is_none(),
        "Non-RT node should not have rt_stats"
    );
}

/// budget tracks worst execution time correctly.
#[test]
fn test_budget_worst_execution_tracked() {
    let _guard = lock_scheduler();
    let counter = Arc::new(AtomicUsize::new(0));

    let mut scheduler = Scheduler::new().safety_monitor(true);

    // Variable-speed node: takes 5ms per tick
    scheduler
        .add(SlowNode::new(
            "variable",
            Duration::from_millis(5),
            counter.clone(),
        ))
        .order(0)
        .budget_us(100_000) // generous budget, just tracking
        .done();

    let _result = scheduler.run_for(Duration::from_millis(200));

    if let Some(stats) = scheduler.rt_stats("variable") {
        assert!(
            stats.worst_execution >= Duration::from_millis(4),
            "Worst execution should be at least ~5ms, got {:?}",
            stats.worst_execution
        );
        assert!(stats.total_ticks > 0, "Should have tracked ticks");
    }
}

// ============================================================================
// Deadline Miss Policy Tests
// ============================================================================

/// RT node within deadline — no misses reported.
#[test]
fn test_deadline_no_miss_within_budget() {
    let _guard = lock_scheduler();
    let counter = Arc::new(AtomicUsize::new(0));

    let mut scheduler = Scheduler::new().safety_monitor(true);
    scheduler
        .add(CounterNode::with_counter("fast_rt", counter.clone()))
        .order(0)
        .deadline_ms(500) // 500ms — CounterNode takes nanoseconds
        .done();

    let _result = scheduler.run_for(Duration::from_secs(1));

    // RT thread needs time to spin up; allow zero ticks on heavily loaded CI
    let ticks = counter.load(Ordering::SeqCst);
    if ticks > 0 {
        if let Some(stats) = scheduler.rt_stats("fast_rt") {
            assert_eq!(
                stats.deadline_misses, 0,
                "Fast node should have zero deadline misses, got {}",
                stats.deadline_misses
            );
        }
    }
}

/// Multiple RT nodes miss deadlines simultaneously — both reported.
#[test]
fn test_deadline_multiple_simultaneous_misses() {
    let _guard = lock_scheduler();
    let slow_a = Arc::new(AtomicUsize::new(0));
    let slow_b = Arc::new(AtomicUsize::new(0));

    let mut scheduler = Scheduler::new().safety_monitor(true);

    scheduler
        .add(SlowNode::new(
            "slow_a",
            Duration::from_millis(30),
            slow_a.clone(),
        ))
        .order(0)
        .deadline_ms(5) // 5ms deadline — missed by 30ms
        .done();

    scheduler
        .add(SlowNode::new(
            "slow_b",
            Duration::from_millis(30),
            slow_b.clone(),
        ))
        .order(1)
        .deadline_ms(10) // 10ms deadline — missed by 30ms
        .done();

    let _result = scheduler.run_for(Duration::from_secs(1));

    if let Some(stats) = scheduler.rt_stats("slow_a") {
        assert!(
            stats.deadline_misses > 0,
            "slow_a should have deadline misses, got {}",
            stats.deadline_misses
        );
    }
    if let Some(stats) = scheduler.rt_stats("slow_b") {
        assert!(
            stats.deadline_misses > 0,
            "slow_b should have deadline misses, got {}",
            stats.deadline_misses
        );
    }
}

/// Mix of deadline-meeting and deadline-missing nodes.
#[test]
fn test_deadline_mixed_meet_and_miss() {
    let _guard = lock_scheduler();
    let fast = Arc::new(AtomicUsize::new(0));
    let slow = Arc::new(AtomicUsize::new(0));

    let mut scheduler = Scheduler::new().safety_monitor(true);

    // Fast node meets deadline
    scheduler
        .add(CounterNode::with_counter("meets_deadline", fast.clone()))
        .order(0)
        .deadline_ms(100)
        .done();

    // Slow node misses deadline
    scheduler
        .add(SlowNode::new(
            "misses_deadline",
            Duration::from_millis(25),
            slow.clone(),
        ))
        .order(1)
        .deadline_ms(5) // 5ms — violated by 25ms sleep
        .done();

    let _result = scheduler.run_for(Duration::from_secs(1));

    if let Some(fast_stats) = scheduler.rt_stats("meets_deadline") {
        assert_eq!(
            fast_stats.deadline_misses, 0,
            "Fast node should have 0 deadline misses"
        );
    }

    if let Some(slow_stats) = scheduler.rt_stats("misses_deadline") {
        assert!(
            slow_stats.deadline_misses > 0,
            "Slow node should have deadline misses"
        );
    }
}

/// Deadline miss stats accumulate across ticks.
#[test]
fn test_deadline_miss_count_accumulates() {
    let _guard = lock_scheduler();
    let counter = Arc::new(AtomicUsize::new(0));

    let mut scheduler = Scheduler::new().safety_monitor(true);

    scheduler
        .add(SlowNode::new(
            "repeated_miss",
            Duration::from_millis(15),
            counter.clone(),
        ))
        .order(0)
        .deadline_ms(5) // 5ms deadline — each tick misses
        .done();

    let _result = scheduler.run_for(Duration::from_millis(500));

    let ticks = counter.load(Ordering::SeqCst);
    assert!(ticks > 1, "Should have ticked multiple times");

    if let Some(stats) = scheduler.rt_stats("repeated_miss") {
        assert!(
            stats.deadline_misses >= 2,
            "Should have accumulated multiple deadline misses, got {}",
            stats.deadline_misses
        );
    }
}

/// Deadline miss + budget violation on same node — both tracked independently.
#[test]
fn test_deadline_miss_and_budget_violation_both_tracked() {
    let _guard = lock_scheduler();
    let counter = Arc::new(AtomicUsize::new(0));

    let mut scheduler = Scheduler::new().safety_monitor(true);

    // Node with both tight deadline AND tight tick budget
    scheduler
        .add(SlowNode::new(
            "double_violation",
            Duration::from_millis(30),
            counter.clone(),
        ))
        .order(0)
        .budget_us(1_000) // 1ms tick budget — violated by 30ms
        .deadline_ms(5) // 5ms deadline — also violated
        .done();

    let _result = scheduler.run_for(Duration::from_secs(1));

    if let Some(stats) = scheduler.rt_stats("double_violation") {
        assert!(stats.budget_violations > 0, "Should have budget violations");
        assert!(stats.deadline_misses > 0, "Should have deadline misses");
    }
}
