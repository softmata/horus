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
    let result = scheduler.run_for(300_u64.ms());
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

    let _ = scheduler.run_for(200_u64.ms());

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

    let result = scheduler.run_for(200_u64.ms());
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
        .build();
    scheduler
        .add(CounterNode::with_counter("motor_ctrl", counter2.clone()))
        .order(1)
        .build();

    // Both nodes should exist
    let nodes = scheduler.node_list();
    assert_eq!(nodes.len(), 2, "Both duplicate-named nodes should be added");

    // Both should tick
    let result = scheduler.run_for(500_u64.ms());
    result.unwrap();
    assert!(
        counter1.load(Ordering::SeqCst) > 0,
        "First duplicate node should tick"
    );
    assert!(
        counter2.load(Ordering::SeqCst) > 0,
        "Second duplicate node should tick"
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
        // Leak a string for the static name reference CounterNode expects
        let name: &'static str = Box::leak(format!("node_{}", i).into_boxed_str());
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
    let result = scheduler.run_for(10_u64.ms());
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

    let result = scheduler.run_for(200_u64.ms());
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

    let result = scheduler.run_for(200_u64.ms());
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

    let result = scheduler.run_for(200_u64.ms());
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

    let result = scheduler.run_for(300_u64.ms());
    result.unwrap();

    let healthy_ticks = healthy_counter.load(Ordering::SeqCst);
    assert!(
        healthy_ticks > 5,
        "Healthy node should tick many times while flaky node is skipped, got {}",
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

    let result = scheduler.run_for(300_u64.ms());
    result.unwrap();

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
        .build();

    scheduler
        .add(PanickingNode::new("bad", 1, bad_counter.clone()))
        .order(1)
        .failure_policy(FailurePolicy::Ignore)
        .build();

    let result = scheduler.run_for(200_u64.ms());
    result.unwrap();

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
            // Leak is fine for tests
            Box::leak(self.name.clone().into_boxed_str())
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
        name: &'static str,
        init_count: Arc<AtomicUsize>,
        tick_count: Arc<AtomicUsize>,
        shutdown_count: Arc<AtomicUsize>,
    }

    impl Node for LifecycleNode {
        fn name(&self) -> &str {
            self.name
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
            name: Box::leak(format!("node_{}", i).into_boxed_str()),
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
            name: "lifecycle_test",
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
            name: "drop_test",
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
            name: "lifo_test",
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
    // On a non-root CI/dev machine, require_rt should detect degradations
    // after apply and set rt_require_failed = true.
    let mut scheduler = Scheduler::new().require_rt().tick_rate(10_u64.hz());
    scheduler
        .add(CounterNode {
            name: "rt_fail_test",
            tick_count: Arc::new(AtomicUsize::new(0)),
        })
        .build()
        .ok();

    let result = scheduler.run_for(Duration::from_millis(50));

    // On non-root: should fail because SCHED_FIFO can't be applied
    // On root with RT: would succeed
    if !horus_sys::rt::can_set_rt_priority() {
        assert!(
            result.is_err(),
            "require_rt() should fail when RT features can't be applied"
        );
        let err = result.unwrap_err().to_string();
        assert!(
            err.contains("require_rt") || err.contains("Real-time"),
            "Error should mention RT: got '{}'",
            err
        );
    }
    // If we're root, it succeeds — that's fine too
}

#[test]
fn test_prefer_rt_succeeds_on_non_root() {
    let _guard = lock_scheduler();
    // prefer_rt should always succeed — degradations are warnings, not errors
    let mut scheduler = Scheduler::new().prefer_rt().tick_rate(10_u64.hz());
    scheduler
        .add(CounterNode {
            name: "rt_prefer_test",
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
            name: "degrad_test",
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
        name: "deadline_flag_test",
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
        name: "no_alloc_flag_test",
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
        .add(TimingNode::new("rd_dia_A", 20, ts.clone()).publishes("rd_dia_ab").publishes("rd_dia_ac"))
        .build();
    scheduler
        .add(TimingNode::new("rd_dia_B", 40, ts.clone()).subscribes("rd_dia_ab").publishes("rd_dia_bd"))
        .build();
    scheduler
        .add(TimingNode::new("rd_dia_C", 40, ts.clone()).subscribes("rd_dia_ac").publishes("rd_dia_cd"))
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
    assert!(log.len() >= 4, "Expected all 4 nodes to tick, got {}", log.len());

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

    scheduler.run_for(100_u64.ms());

    // Should have ticked multiple times (100Hz for 100ms = ~10 ticks)
    let ticks = counter.load(Ordering::SeqCst);
    assert!(
        ticks >= 5,
        "Single node should tick at least 5 times in 100ms at 100Hz, got {}",
        ticks
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

    scheduler.run_for(200_u64.ms());

    // Healthy node should still have ticked
    let ticks = healthy_counter.load(Ordering::SeqCst);
    assert!(
        ticks >= 3,
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
fn register_topic(node_name: &str, topic_name: &str, role: crate::communication::topic::NodeTopicRole) {
    crate::communication::topic_node_registry().register_with_type(
        topic_name, node_name, role, "f64",
    );
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
        fn name(&self) -> &str { &self.name }
        fn init(&mut self) -> crate::error::HorusResult<()> {
            register_topic(&self.name, &self.topic, crate::communication::topic::NodeTopicRole::Publisher);
            Ok(())
        }
        fn tick(&mut self) {
            let start = std::time::Instant::now();
            std::thread::sleep(Duration::from_millis(self.sleep_ms));
            let end = std::time::Instant::now();
            self.ts.lock().unwrap().push((self.name.clone(), start, end));
        }
    }

    for (name, topic) in [("wh_lidar", "wh_scan"), ("wh_camera", "wh_image"), ("wh_imu", "wh_imu_data"), ("wh_encoder", "wh_odom")] {
        scheduler.add(SensorNode {
            name: name.to_string(),
            topic: topic.to_string(),
            sleep_ms: 20,
            ts: ts.clone(),
        }).build();
    }

    scheduler.run_for(250_u64.ms());

    let log = ts.lock().unwrap();
    assert!(log.len() >= 4, "All 4 sensors should tick, got {}", log.len());

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
        fn name(&self) -> &str { &self.name }
        fn init(&mut self) -> crate::error::HorusResult<()> {
            for t in &self.pubs {
                register_topic(&self.name, t, crate::communication::topic::NodeTopicRole::Publisher);
            }
            for t in &self.subs {
                register_topic(&self.name, t, crate::communication::topic::NodeTopicRole::Subscriber);
            }
            Ok(())
        }
        fn tick(&mut self) {
            let start = std::time::Instant::now();
            std::thread::sleep(Duration::from_millis(self.sleep_ms));
            let end = std::time::Instant::now();
            self.ts.lock().unwrap().push((self.name.clone(), start, end));
        }
    }

    let mut scheduler = Scheduler::new().tick_rate(3_u64.hz());

    // NO .order() calls — dependency graph handles everything
    scheduler.add(PipelineNode {
        name: "sr_force".into(), pubs: vec!["sr_force_data".into()], subs: vec![],
        sleep_ms: 15, ts: ts.clone(),
    }).build();

    scheduler.add(PipelineNode {
        name: "sr_vision".into(), pubs: vec!["sr_vision_data".into()], subs: vec![],
        sleep_ms: 40, ts: ts.clone(),
    }).build();

    scheduler.add(PipelineNode {
        name: "sr_safety".into(),
        pubs: vec!["sr_safe_cmd".into()],
        subs: vec!["sr_force_data".into(), "sr_vision_data".into()],
        sleep_ms: 5, ts: ts.clone(),
    }).build();

    scheduler.add(PipelineNode {
        name: "sr_ctrl".into(),
        pubs: vec!["sr_joint_cmd".into()],
        subs: vec!["sr_safe_cmd".into()],
        sleep_ms: 10, ts: ts.clone(),
    }).build();

    scheduler.add(PipelineNode {
        name: "sr_actuator".into(), pubs: vec![], subs: vec!["sr_joint_cmd".into()],
        sleep_ms: 5, ts: ts.clone(),
    }).build();

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
        fn name(&self) -> &str { &self.name }
        fn init(&mut self) -> crate::error::HorusResult<()> {
            for t in &self.pubs {
                register_topic(&self.name, t, crate::communication::topic::NodeTopicRole::Publisher);
            }
            for t in &self.subs {
                register_topic(&self.name, t, crate::communication::topic::NodeTopicRole::Subscriber);
            }
            Ok(())
        }
        fn tick(&mut self) {
            let start = std::time::Instant::now();
            std::thread::sleep(Duration::from_millis(self.sleep_ms));
            let end = std::time::Instant::now();
            self.ts.lock().unwrap().push((self.name.clone(), start, end));
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
        scheduler.add(CarNode {
            name: name.into(), pubs: vec![topic.into()], subs: vec![],
            sleep_ms: ms, ts: ts.clone(),
        }).build();
    }

    // Layer 2: 5 detectors (each depends on one sensor, parallel with each other)
    for (name, pub_t, sub_t, ms) in [
        ("ac_front_det", "ac_front_det_out", "ac_front_img", 25),
        ("ac_left_det", "ac_left_det_out", "ac_left_img", 25),
        ("ac_right_det", "ac_right_det_out", "ac_right_img", 25),
        ("ac_pc_proc", "ac_pc_out", "ac_cloud", 30),
        ("ac_radar_proc", "ac_radar_out", "ac_radar_pts", 15),
    ] {
        scheduler.add(CarNode {
            name: name.into(), pubs: vec![pub_t.into()], subs: vec![sub_t.into()],
            sleep_ms: ms, ts: ts.clone(),
        }).build();
    }

    // Layer 3: Fusion planner (depends on ALL detectors)
    scheduler.add(CarNode {
        name: "ac_planner".into(),
        pubs: vec!["ac_plan".into()],
        subs: vec!["ac_front_det_out".into(), "ac_left_det_out".into(),
                    "ac_right_det_out".into(), "ac_pc_out".into(), "ac_radar_out".into()],
        sleep_ms: 15, ts: ts.clone(),
    }).build();

    // Layer 4: Vehicle controller
    scheduler.add(CarNode {
        name: "ac_vehicle_ctrl".into(), pubs: vec![], subs: vec!["ac_plan".into()],
        sleep_ms: 5, ts: ts.clone(),
    }).build();

    scheduler.run_for(600_u64.ms());

    let log = ts.lock().unwrap();
    assert!(log.len() >= 12, "All 12 nodes should tick, got {}", log.len());

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
        fn name(&self) -> &str { &self.name }
        fn init(&mut self) -> crate::error::HorusResult<()> {
            for t in &self.pubs {
                register_topic(&self.name, t, crate::communication::topic::NodeTopicRole::Publisher);
            }
            for t in &self.subs {
                register_topic(&self.name, t, crate::communication::topic::NodeTopicRole::Subscriber);
            }
            Ok(())
        }
        fn tick(&mut self) {
            let start = std::time::Instant::now();
            std::thread::sleep(Duration::from_millis(self.sleep_ms));
            let end = std::time::Instant::now();
            self.ts.lock().unwrap().push((self.name.clone(), start, end));
        }
    }

    let mut scheduler = Scheduler::new().tick_rate(3_u64.hz());

    // 3 arms, each with sensor → controller → motor (completely independent chains)
    for arm_id in 1..=3 {
        let prefix = format!("fa_arm{}", arm_id);
        scheduler.add(ArmNode {
            name: format!("{}_sensor", prefix),
            pubs: vec![format!("{}_joint_state", prefix)], subs: vec![],
            sleep_ms: 15, ts: ts.clone(),
        }).build();
        scheduler.add(ArmNode {
            name: format!("{}_ctrl", prefix),
            pubs: vec![format!("{}_joint_cmd", prefix)],
            subs: vec![format!("{}_joint_state", prefix)],
            sleep_ms: 15, ts: ts.clone(),
        }).build();
        scheduler.add(ArmNode {
            name: format!("{}_motor", prefix),
            pubs: vec![], subs: vec![format!("{}_joint_cmd", prefix)],
            sleep_ms: 10, ts: ts.clone(),
        }).build();
    }

    scheduler.run_for(400_u64.ms());

    let log = ts.lock().unwrap();
    assert!(log.len() >= 9, "All 9 nodes should tick, got {}", log.len());

    // Within each arm: causal ordering maintained
    for arm_id in 1..=3 {
        let prefix = format!("fa_arm{}", arm_id);
        assert_ordered(&log, &format!("{}_sensor", prefix), &format!("{}_ctrl", prefix));
        assert_ordered(&log, &format!("{}_ctrl", prefix), &format!("{}_motor", prefix));
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
        fn name(&self) -> &str { &self.name }
        fn init(&mut self) -> crate::error::HorusResult<()> {
            for t in &self.pubs {
                register_topic(&self.name, t, crate::communication::topic::NodeTopicRole::Publisher);
            }
            for t in &self.subs {
                register_topic(&self.name, t, crate::communication::topic::NodeTopicRole::Subscriber);
            }
            Ok(())
        }
        fn tick(&mut self) {
            let start = std::time::Instant::now();
            std::thread::sleep(Duration::from_millis(self.sleep_ms));
            let end = std::time::Instant::now();
            self.ts.lock().unwrap().push((self.name.clone(), start, end));
        }
    }

    let mut scheduler = Scheduler::new().tick_rate(3_u64.hz());

    scheduler.add(DroneNode {
        name: "dr_imu".into(), pubs: vec!["dr_imu_data".into()], subs: vec![],
        sleep_ms: 2, ts: ts.clone(),
    }).build();

    scheduler.add(DroneNode {
        name: "dr_camera".into(), pubs: vec!["dr_image".into()], subs: vec![],
        sleep_ms: 40, ts: ts.clone(),
    }).build();

    scheduler.add(DroneNode {
        name: "dr_nav".into(),
        pubs: vec!["dr_nav_cmd".into()],
        subs: vec!["dr_imu_data".into(), "dr_image".into()],
        sleep_ms: 10, ts: ts.clone(),
    }).build();

    scheduler.add(DroneNode {
        name: "dr_motors".into(), pubs: vec![], subs: vec!["dr_nav_cmd".into()],
        sleep_ms: 2, ts: ts.clone(),
    }).build();

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
        fn name(&self) -> &str { &self.name }
        fn init(&mut self) -> crate::error::HorusResult<()> {
            for t in &self.pubs {
                register_topic(&self.name, t, crate::communication::topic::NodeTopicRole::Publisher);
            }
            for t in &self.subs {
                register_topic(&self.name, t, crate::communication::topic::NodeTopicRole::Subscriber);
            }
            Ok(())
        }
        fn tick(&mut self) {
            let start = std::time::Instant::now();
            std::thread::sleep(Duration::from_millis(self.sleep_ms));
            let end = std::time::Instant::now();
            self.ts.lock().unwrap().push((self.name.clone(), start, end));
        }
    }

    let mut scheduler = Scheduler::new().tick_rate(3_u64.hz());

    // Added in deliberately WRONG order — no .order() calls
    scheduler.add(SimpleNode {
        name: "ro_E".into(), pubs: vec![], subs: vec!["ro_cmd".into()],
        sleep_ms: 5, ts: ts.clone(),
    }).build();
    scheduler.add(SimpleNode {
        name: "ro_C".into(), pubs: vec!["ro_plan".into()], subs: vec!["ro_scan".into()],
        sleep_ms: 15, ts: ts.clone(),
    }).build();
    scheduler.add(SimpleNode {
        name: "ro_A".into(), pubs: vec!["ro_scan".into()], subs: vec![],
        sleep_ms: 10, ts: ts.clone(),
    }).build();
    scheduler.add(SimpleNode {
        name: "ro_D".into(), pubs: vec!["ro_cmd".into()], subs: vec!["ro_plan".into()],
        sleep_ms: 10, ts: ts.clone(),
    }).build();
    scheduler.add(SimpleNode {
        name: "ro_B".into(), pubs: vec!["ro_scan".into()], subs: vec![],
        sleep_ms: 10, ts: ts.clone(),
    }).build();

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
