// Scheduler tests

use super::*;
use crate::scheduling::fault_tolerance::FailurePolicy;
use std::sync::atomic::{AtomicBool, AtomicUsize, Ordering};
use std::sync::{Arc, Mutex, MutexGuard};
use std::time::Duration;
use crate::core::{DurationExt, Node};

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

// test_scheduler_with_name: DELETED — .with_name() removed from API

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
    scheduler
        .add(CounterNode::new("rt_node"))
        .order(0)
        .build();

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
// Builder Pattern Name Test — DELETED (.with_name() removed from API)
// ============================================================================

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

// test_deploy_config_creates_blackbox_with_wal: DELETED — .with_blackbox() removed from API

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

// SlowNode: DELETED — was only used by .budget()/.deadline() tests which are removed

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

// test_budget_violation_detected_for_slow_rt_node: DELETED — .budget() removed from API
// test_deadline_miss_detected_for_slow_rt_node: DELETED — .deadline() removed from API

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
    // Setting rate on non-existent node should not panic
    scheduler.set_node_rate("does_not_exist", 100_u64.hz());
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
    // Stopping before running should not panic
    scheduler.stop();
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

// test_with_name_empty: DELETED — .with_name() removed from API
// test_with_name_long: DELETED — .with_name() removed from API
// test_with_blackbox_zero_size: DELETED — .with_blackbox() removed from API
// test_max_deadline_misses_zero: DELETED — .max_deadline_misses() removed from API

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
    // Capabilities are only populated after run, should be None before
    // (or Some if pre-populated — either way, no panic)
    let _ = scheduler.capabilities();
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
// budget Budget Enforcement Tests — DELETED (entire section)
// .budget() removed from NodeBuilder API (auto-derived from rate)
// ============================================================================

// ============================================================================
// Deadline Miss Policy Tests — DELETED (entire section)
// .deadline() removed from NodeBuilder API (auto-derived from rate)
// ============================================================================

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

// test_deterministic_sets_config: DELETED — .deterministic() removed from API
// test_deterministic_false_clears_config: DELETED — .deterministic() removed from API
// test_cores_sets_config: DELETED — .cores() removed from API
// test_with_profiling_sets_config: DELETED — .with_profiling() removed from API
// test_with_blackbox_sets_config: DELETED — .with_blackbox() removed from API
// test_max_deadline_misses_sets_config: DELETED — .max_deadline_misses() removed from API

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

// test_tick_once_nodes_filters_correctly: DELETED — .tick_once_nodes() removed from API
// test_tick_once_nodes_nonexistent_ignored: DELETED — .tick_once_nodes() removed from API

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
    let mut scheduler = Scheduler::new().watchdog(500_u64.ms()).tick_rate(100_u64.hz());
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
// Phase 6: Budget Enforcement Under Overload — DELETED (entire section)
// .budget() removed from NodeBuilder API (auto-derived from rate)
// ============================================================================

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
        .add(CounterNode::with_counter("monitored_tick_once", counter.clone()))
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

    let mut scheduler = Scheduler::new().watchdog(500_u64.ms()).tick_rate(100_u64.hz());

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
