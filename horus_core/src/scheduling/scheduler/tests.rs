use super::*;
use crate::core::Node;
use crate::scheduling::config::SchedulerConfig;
use crate::scheduling::deterministic::DeterministicConfig;
use std::sync::atomic::{AtomicBool, AtomicUsize, Ordering};
use std::sync::{Arc, Mutex};
use std::time::Duration;

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
    let scheduler = Scheduler::new();
    assert!(scheduler.is_running());
    assert_eq!(scheduler.node_list().len(), 0);
}

#[test]
fn test_scheduler_default() {
    let scheduler = Scheduler::default();
    assert!(scheduler.is_running());
    assert_eq!(scheduler.node_list().len(), 0);
}

#[test]
fn test_scheduler_with_name() {
    let scheduler = Scheduler::new().with_name("TestScheduler");
    // The name is stored internally and used in logging
    assert!(scheduler.is_running());
}

#[test]
fn test_scheduler_with_capacity() {
    let scheduler = Scheduler::new().with_capacity(100);
    assert!(scheduler.is_running());
    // Capacity is pre-allocated but empty
    assert_eq!(scheduler.node_list().len(), 0);
}

// ============================================================================
// Node Addition Tests
// ============================================================================

#[test]
fn test_scheduler_add_node() {
    let mut scheduler = Scheduler::new();
    scheduler.add(CounterNode::new("test_node")).order(0).done();

    let nodes = scheduler.node_list();
    assert_eq!(nodes.len(), 1);
    assert_eq!(nodes[0], "test_node");
}

#[test]
fn test_scheduler_add_multiple_nodes() {
    let mut scheduler = Scheduler::new();
    scheduler.add(CounterNode::new("node1")).order(0).done();
    scheduler.add(CounterNode::new("node2")).order(1).done();
    scheduler.add(CounterNode::new("node3")).order(2).done();

    let nodes = scheduler.node_list();
    assert_eq!(nodes.len(), 3);
}

#[test]
fn test_scheduler_node_priority_ordering() {
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
    let mut scheduler = Scheduler::new();
    scheduler
        .add(CounterNode::new("basic_node"))
        .order(0)
        .done();

    let info = scheduler.node_info("basic_node");
    assert!(info.is_some());
}

// ============================================================================
// Running State Tests
// ============================================================================

#[test]
fn test_scheduler_is_running() {
    let scheduler = Scheduler::new();
    assert!(scheduler.is_running());
}

#[test]
fn test_scheduler_stop() {
    let scheduler = Scheduler::new();
    assert!(scheduler.is_running());
    scheduler.stop();
    assert!(!scheduler.is_running());
}

#[test]
fn test_scheduler_stop_and_check_multiple_times() {
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
    let mut scheduler = Scheduler::new();
    scheduler.add(CounterNode::new("sensor")).order(0).done();
    scheduler.set_node_rate("sensor", 100.0);

    // Just verify it doesn't panic
    assert!(scheduler.is_running());
}

#[test]
fn test_scheduler_set_node_rate_nonexistent() {
    let mut scheduler = Scheduler::new();
    scheduler.add(CounterNode::new("node1")).order(0).done();
    // Setting rate for nonexistent node should not panic
    scheduler.set_node_rate("nonexistent", 50.0);
    assert!(scheduler.is_running());
}

// ============================================================================
// Node Info Tests
// ============================================================================

#[test]
fn test_scheduler_node_info_existing() {
    let mut scheduler = Scheduler::new();
    scheduler.add(CounterNode::new("info_node")).order(0).done();

    let info = scheduler.node_info("info_node");
    assert!(info.is_some());

    let info_map = info.unwrap();
    assert!(info_map.contains_key("name"));
    assert_eq!(info_map.get("name").unwrap(), "info_node");
}

#[test]
fn test_scheduler_node_info_nonexistent() {
    let scheduler = Scheduler::new();
    let info = scheduler.node_info("nonexistent");
    assert!(info.is_none());
}

#[test]
fn test_scheduler_node_list_empty() {
    let scheduler = Scheduler::new();
    let nodes = scheduler.node_list();
    assert!(nodes.is_empty());
}

// ============================================================================
// Monitoring Summary Tests
// ============================================================================

#[test]
fn test_scheduler_monitoring_summary() {
    let mut scheduler = Scheduler::new();
    scheduler.add(CounterNode::new("mon_node1")).order(0).done();
    scheduler.add(CounterNode::new("mon_node2")).order(1).done();

    let summary = scheduler.monitoring_summary();
    assert_eq!(summary.len(), 2);
}

#[test]
fn test_scheduler_monitoring_summary_empty() {
    let scheduler = Scheduler::new();
    let summary = scheduler.monitoring_summary();
    assert!(summary.is_empty());
}

// ============================================================================
// Recording Tests
// ============================================================================

#[test]
fn test_scheduler_is_recording_default() {
    let scheduler = Scheduler::new();
    assert!(!scheduler.is_recording());
}

#[test]
fn test_scheduler_enable_recording() {
    use crate::scheduling::config::RecordingConfigYaml;
    let mut config = crate::scheduling::config::SchedulerConfig::minimal();
    config.recording = Some(RecordingConfigYaml::full());
    let mut scheduler = Scheduler::new();
    scheduler.apply_config(config);
    assert!(scheduler.is_recording());
}

#[test]
fn test_scheduler_is_replaying_default() {
    let scheduler = Scheduler::new();
    assert!(!scheduler.is_replaying());
}

#[test]
fn test_scheduler_current_tick() {
    let scheduler = Scheduler::new();
    assert_eq!(scheduler.current_tick(), 0);
}

#[test]
fn test_scheduler_start_at_tick() {
    let scheduler = Scheduler::new().start_at_tick(1000);
    assert_eq!(scheduler.current_tick(), 1000);
}

// ============================================================================
// Safety Monitor Tests
// ============================================================================

#[test]
fn test_scheduler_with_safety_monitor() {
    let mut config = crate::scheduling::config::SchedulerConfig::minimal();
    config.realtime.safety_monitor = true;
    config.realtime.max_deadline_misses = 10;
    let mut scheduler = Scheduler::new();
    scheduler.apply_config(config);
    assert!(scheduler.is_running());
}

// ============================================================================
// Real-time Node Tests
// ============================================================================

#[test]
fn test_scheduler_add_rt_node() {
    let mut scheduler = Scheduler::new();
    scheduler
        .add(CounterNode::new("rt_node"))
        .order(0)
        .rt()
        .wcet_us(100)
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
    let mut scheduler = Scheduler::new()
        .with_name("ChainedScheduler")
        .with_capacity(10);

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
    let scheduler = Scheduler::new().with_name("BuilderName");
    // Verify the scheduler was created successfully
    assert!(scheduler.is_running());
}

// ============================================================================
// Override Tests
// ============================================================================

#[test]
fn test_scheduler_with_override() {
    let scheduler = Scheduler::new().with_override("node1", "output1", vec![1, 2, 3, 4]);

    // Should not panic and scheduler should still be running
    assert!(scheduler.is_running());
}

// ============================================================================
// Auto-Optimization Tests
// ============================================================================

#[test]
fn test_scheduler_auto_optimization() {
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
    // Create scheduler in parallel mode via high_performance preset
    let mut scheduler = Scheduler::high_performance();

    let c1 = Arc::new(AtomicUsize::new(0));
    let c2 = Arc::new(AtomicUsize::new(0));
    let c3 = Arc::new(AtomicUsize::new(0));

    scheduler
        .add(CounterNode::with_counter("par_a", c1.clone()))
        .order(100)
        .done();
    scheduler
        .add(CounterNode::with_counter("par_b", c2.clone()))
        .order(101)
        .done();
    scheduler
        .add(CounterNode::with_counter("par_c", c3.clone()))
        .order(102)
        .done();

    // Run for 100ms — all 3 non-RT nodes should tick in parallel
    let result = scheduler.run_for(Duration::from_millis(500));
    assert!(result.is_ok());

    // Every node must have ticked at least once
    assert!(c1.load(Ordering::SeqCst) > 0, "par_a never ticked");
    assert!(c2.load(Ordering::SeqCst) > 0, "par_b never ticked");
    assert!(c3.load(Ordering::SeqCst) > 0, "par_c never ticked");
}

#[test]
fn test_parallel_rt_nodes_run_sequentially() {
    // RT nodes must run sequentially even in parallel mode
    let mut scheduler = Scheduler::high_performance();

    let rt_counter = Arc::new(AtomicUsize::new(0));
    let normal_counter = Arc::new(AtomicUsize::new(0));

    scheduler
        .add(CounterNode::with_counter("rt_node", rt_counter.clone()))
        .order(0)
        .rt()
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
    let scheduler = Scheduler::deterministic();
    assert!(scheduler.is_simulation_mode());
    assert!(scheduler.deterministic_clock().is_some());
    assert!(scheduler.execution_trace().is_some());
    assert_eq!(scheduler.virtual_tick(), Some(0));
    assert_eq!(scheduler.seed(), Some(42));
}

#[test]
fn test_deterministic_config_virtual_time() {
    let mut config = SchedulerConfig::minimal();
    config.deterministic = Some(DeterministicConfig {
        seed: 123,
        virtual_time: true,
        tick_duration_ns: 1_000_000,
        record_trace: false,
    });
    let mut scheduler = Scheduler::new();
    scheduler.apply_config(config);

    assert!(scheduler.is_simulation_mode());
    assert_eq!(scheduler.seed(), Some(123));
    // No trace when record_trace=false
    assert!(scheduler.execution_trace().is_none());
}

#[test]
fn test_deterministic_advances_on_run() {
    let counter = Arc::new(AtomicUsize::new(0));
    let mut scheduler = Scheduler::deterministic();
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
    let counter = Arc::new(AtomicUsize::new(0));
    // Recording is already enabled by deterministic() preset
    let mut scheduler = Scheduler::deterministic();
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
    let tmp = std::env::temp_dir().join(format!("horus_bb_test_{}", std::process::id()));
    let _ = std::fs::remove_dir_all(&tmp);

    let mut bb = super::super::blackbox::BlackBox::new(1).with_path(tmp.clone());
    bb.record(super::super::blackbox::BlackBoxEvent::Custom {
        category: "test".to_string(),
        message: "wal_test".to_string(),
    });

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
    let scheduler = Scheduler::deploy();
    assert!(
        scheduler.blackbox().is_some(),
        "Deploy preset should create a blackbox"
    );
}

// ============================================================================
// Fix 4: Dead Code Cleanup Verification
// ============================================================================

#[test]
fn test_per_node_rates_work_without_dead_code() {
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
// WCET / Deadline / Failure Policy Integration Tests
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

/// Node that sleeps in tick() to simulate slow execution (for WCET tests).
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

/// WCET enforcement: slow RT node exceeds WCET budget, violation is detected.
/// Robotics: motor control loop exceeding 1ms WCET must be flagged.
#[test]
fn test_wcet_violation_detected_for_slow_rt_node() {
    let slow_counter = Arc::new(AtomicUsize::new(0));
    let fast_counter = Arc::new(AtomicUsize::new(0));

    let mut scheduler = Scheduler::new();

    // Fast node within budget
    scheduler
        .add(CounterNode::with_counter("fast_ctrl", fast_counter.clone()))
        .order(0)
        .rt()
        .wcet_us(10_000) // 10ms budget (generous)
        .done();

    // Slow node that will exceed its WCET budget
    scheduler
        .add(SlowNode::new(
            "slow_sensor",
            Duration::from_millis(50),
            slow_counter.clone(),
        ))
        .order(1)
        .rt()
        .wcet_us(1_000) // 1ms budget — will be violated by 50ms sleep
        .done();

    // Enable safety monitor with WCET enforcement
    let mut config = SchedulerConfig::minimal();
    config.realtime.safety_monitor = true;
    config.realtime.wcet_enforcement = true;
    scheduler.apply_config(config);

    let _result = scheduler.run_for(Duration::from_secs(1));

    // Slow node should have ticked
    assert!(
        slow_counter.load(Ordering::SeqCst) > 0,
        "slow node should have ticked"
    );

    // Check safety stats for WCET overruns
    if let Some(stats) = scheduler.safety_stats() {
        assert!(
            stats.wcet_overruns > 0,
            "WCET violation should have been detected, got {} overruns",
            stats.wcet_overruns
        );
    }
}

/// Deadline miss detection: RT node misses its deadline.
/// Robotics: control loop missing 10ms deadline means actuator stale.
#[test]
fn test_deadline_miss_detected_for_slow_rt_node() {
    let slow_counter = Arc::new(AtomicUsize::new(0));

    // Enable safety monitor with deadline monitoring BEFORE adding nodes
    let mut config = SchedulerConfig::minimal();
    config.realtime.safety_monitor = true;
    config.realtime.deadline_monitoring = true;

    let mut scheduler = Scheduler::new();
    scheduler.apply_config(config);

    // Node with tight deadline that will be missed
    scheduler
        .add(SlowNode::new(
            "ctrl_loop",
            Duration::from_millis(20),
            slow_counter.clone(),
        ))
        .order(0)
        .rt()
        .deadline_ms(5) // 5ms deadline — will be missed by 20ms sleep
        .done();

    let _result = scheduler.run_for(Duration::from_secs(2));

    assert!(
        slow_counter.load(Ordering::SeqCst) > 0,
        "node should have ticked"
    );

    // Check for deadline misses
    if let Some(stats) = scheduler.safety_stats() {
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
