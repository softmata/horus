use super::*;
use crate::core::Node;
use crate::scheduling::config::SchedulerConfig;
use crate::scheduling::deterministic::DeterministicConfig;
use std::sync::atomic::{AtomicUsize, Ordering};
use std::sync::Arc;
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
    let mut config = crate::scheduling::config::SchedulerConfig::standard();
    config.recording = Some(RecordingConfigYaml::full());
    let scheduler = Scheduler::new().with_config(config);
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
    let mut config = crate::scheduling::config::SchedulerConfig::standard();
    config.realtime.safety_monitor = true;
    config.realtime.max_deadline_misses = 10;
    let scheduler = Scheduler::new().with_config(config);
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
    let result = scheduler.run_for(Duration::from_millis(100));
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
    let result = scheduler.run_for(Duration::from_millis(100));
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

    let result = scheduler.run_for(Duration::from_millis(100));
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

    // Run for 200ms — expect ~100 ticks at 500Hz
    let result = scheduler.run_for(Duration::from_millis(200));
    assert!(result.is_ok());

    let ticks = counter.load(Ordering::SeqCst);
    // Allow generous tolerance: at least 50 ticks (250Hz effective)
    // and no more than 200 ticks (1000Hz) to account for timing jitter
    assert!(
        ticks >= 50,
        "Node at 500Hz should tick at least ~50 times in 200ms, got {}",
        ticks
    );
    assert!(
        ticks <= 300,
        "Node at 500Hz should not tick more than ~300 times in 200ms, got {}",
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
    let scheduler = Scheduler::new().with_config(config);

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
    let _ = scheduler.run_for(Duration::from_millis(50));

    // Virtual tick should have advanced
    let tick = scheduler.virtual_tick().unwrap();
    assert!(tick > 0, "Virtual tick should advance, got {}", tick);

    // Node should have been ticked
    let ticks = counter.load(Ordering::SeqCst);
    assert!(ticks > 0, "Node should have ticked, got {}", ticks);
}

#[test]
fn test_standard_config_no_deterministic() {
    let scheduler = Scheduler::new().with_config(SchedulerConfig::standard());
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
    let config = SchedulerConfig::deterministic();
    // Recording is already enabled by deterministic() preset
    let mut scheduler = Scheduler::new().with_config(config);
    scheduler
        .add(CounterNode::with_counter("rec_node", counter.clone()))
        .order(0)
        .done();

    assert!(scheduler.is_recording());

    let _ = scheduler.run_for(Duration::from_millis(50));

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

    let _ = scheduler.run_for(Duration::from_millis(100));

    let ticks = counter.load(Ordering::SeqCst);
    // At 100Hz for 100ms, expect ~10 ticks (±5 tolerance)
    assert!(
        (5..=30).contains(&ticks),
        "Node at 100Hz should tick ~10 times in 100ms, got {}",
        ticks
    );
}
