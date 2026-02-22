// Test comprehensive scheduler configuration
use horus_core::core::Node;
use horus_core::error::HorusResult as Result;
use horus_core::hlog;
use horus_core::scheduling::{ExecutionMode, Scheduler, SchedulerConfig};

/// Verify all presets produce the exact expected field values.
/// Guards against regressions in the mutation-based preset construction.
#[test]
fn test_preset_field_values() {
    // --- minimal ---
    let m = SchedulerConfig::minimal();
    assert!(matches!(m.execution, ExecutionMode::Sequential));
    assert_eq!(m.timing.global_rate_hz, 60.0);
    assert!(!m.circuit_breaker);
    assert!(!m.realtime.wcet_enforcement);
    assert!(!m.realtime.deadline_monitoring);
    assert!(!m.realtime.watchdog_enabled);
    assert_eq!(m.realtime.watchdog_timeout_ms, 1000);
    assert!(!m.realtime.safety_monitor);
    assert_eq!(m.realtime.max_deadline_misses, 100);
    assert!(!m.realtime.memory_locking);
    assert!(!m.realtime.rt_scheduling_class);
    assert!(m.resources.cpu_cores.is_none());
    assert!(!m.resources.numa_aware);
    assert!(!m.monitoring.profiling_enabled);
    assert_eq!(m.monitoring.metrics_interval_ms, 1000);
    assert!(!m.monitoring.black_box_enabled);
    assert_eq!(m.monitoring.black_box_size_mb, 0);
    assert!(m.monitoring.telemetry_endpoint.is_none());
    assert!(m.recording.is_none());
    assert!(m.deterministic.is_none());

    // --- standard = minimal + circuit_breaker + profiling ---
    let s = SchedulerConfig::standard();
    assert!(matches!(s.execution, ExecutionMode::Sequential));
    assert_eq!(s.timing.global_rate_hz, 60.0);
    assert!(s.circuit_breaker);
    assert!(!s.realtime.wcet_enforcement);
    assert!(s.monitoring.profiling_enabled);
    assert!(s.recording.is_none());

    // --- deploy = standard + RT flags + blackbox ---
    let d = SchedulerConfig::deploy();
    assert!(d.circuit_breaker);
    assert!(d.realtime.deadline_monitoring);
    assert!(d.realtime.watchdog_enabled);
    assert!(d.realtime.memory_locking);
    assert!(d.realtime.rt_scheduling_class);
    assert!(d.monitoring.profiling_enabled);
    assert!(d.monitoring.black_box_enabled);
    assert_eq!(d.monitoring.black_box_size_mb, 16);

    // --- deterministic ---
    let det = SchedulerConfig::deterministic();
    assert_eq!(det.timing.global_rate_hz, 1000.0);
    assert!(!det.circuit_breaker);
    assert!(det.realtime.deadline_monitoring);
    assert_eq!(det.realtime.max_deadline_misses, 3);
    assert_eq!(det.monitoring.metrics_interval_ms, 100);
    assert!(det.monitoring.black_box_enabled);
    assert_eq!(det.monitoring.black_box_size_mb, 100);
    assert!(det.recording.is_some());
    assert!(det.deterministic.is_some());

    // --- safety_critical ---
    let sc = SchedulerConfig::safety_critical();
    assert_eq!(sc.timing.global_rate_hz, 1000.0);
    assert!(!sc.circuit_breaker);
    assert!(sc.realtime.wcet_enforcement);
    assert!(sc.realtime.deadline_monitoring);
    assert!(sc.realtime.watchdog_enabled);
    assert_eq!(sc.realtime.watchdog_timeout_ms, 100);
    assert!(sc.realtime.safety_monitor);
    assert_eq!(sc.realtime.max_deadline_misses, 0);
    assert!(sc.realtime.memory_locking);
    assert!(sc.realtime.rt_scheduling_class);
    assert_eq!(sc.resources.cpu_cores, Some(vec![0, 1]));
    assert!(sc.resources.numa_aware);
    assert!(!sc.monitoring.profiling_enabled);
    assert_eq!(sc.monitoring.metrics_interval_ms, 10);
    assert!(sc.monitoring.black_box_enabled);
    assert_eq!(sc.monitoring.black_box_size_mb, 1024);
    assert!(sc.recording.is_some());

    // --- high_performance ---
    let hp = SchedulerConfig::high_performance();
    assert!(matches!(hp.execution, ExecutionMode::Parallel));
    assert_eq!(hp.timing.global_rate_hz, 10000.0);
    assert!(hp.circuit_breaker);
    assert!(hp.realtime.wcet_enforcement);
    assert!(hp.realtime.deadline_monitoring);
    assert!(!hp.realtime.watchdog_enabled);
    assert_eq!(hp.realtime.watchdog_timeout_ms, 0);
    assert_eq!(hp.realtime.max_deadline_misses, 10);
    assert!(hp.realtime.memory_locking);
    assert!(hp.realtime.rt_scheduling_class);
    assert!(hp.resources.numa_aware);
    assert_eq!(hp.monitoring.metrics_interval_ms, 10000);
    assert!(!hp.monitoring.black_box_enabled);
    assert!(hp.recording.is_none());

    // --- hard_realtime ---
    let hr = SchedulerConfig::hard_realtime();
    assert!(matches!(hr.execution, ExecutionMode::Parallel));
    assert_eq!(hr.timing.global_rate_hz, 1000.0);
    assert!(hr.circuit_breaker);
    assert!(hr.realtime.wcet_enforcement);
    assert!(hr.realtime.deadline_monitoring);
    assert!(hr.realtime.watchdog_enabled);
    assert_eq!(hr.realtime.watchdog_timeout_ms, 10);
    assert!(hr.realtime.safety_monitor);
    assert_eq!(hr.realtime.max_deadline_misses, 3);
    assert!(hr.realtime.memory_locking);
    assert!(hr.realtime.rt_scheduling_class);
    assert!(!hr.monitoring.profiling_enabled);
    assert!(hr.monitoring.black_box_enabled);
    assert_eq!(hr.monitoring.black_box_size_mb, 100);
    assert!(hr.recording.is_some());
}

mod common;
use common::cleanup_stale_shm;

/// Simple test node for configuration testing
struct TestNode {
    name: String,
    tick_count: usize,
}

impl TestNode {
    fn new(name: &str) -> Self {
        Self {
            name: name.to_string(),
            tick_count: 0,
        }
    }
}

impl Node for TestNode {
    fn name(&self) -> &'static str {
        Box::leak(self.name.clone().into_boxed_str())
    }

    fn init(&mut self) -> Result<()> {
        hlog!(info, "{} initialized", self.name);
        Ok(())
    }

    fn tick(&mut self) {
        self.tick_count += 1;
    }

    fn shutdown(&mut self) -> Result<()> {
        hlog!(
            info,
            "{} shutdown after {} ticks",
            self.name,
            self.tick_count
        );
        Ok(())
    }
}

#[test]
fn test_standard_config() {
    cleanup_stale_shm();
    // Apply standard robot configuration
    let mut scheduler = Scheduler::new().with_config(SchedulerConfig::standard());

    scheduler.add(TestNode::new("sensor1")).order(0).done();
    scheduler.add(TestNode::new("controller1")).order(1).done();

    // Run for a short duration to test
    let result = scheduler.run_for(std::time::Duration::from_millis(100));
    assert!(result.is_ok());
}

#[test]
fn test_safety_critical_config() {
    cleanup_stale_shm();
    // Apply safety-critical robot configuration
    let mut scheduler = Scheduler::new().with_config(SchedulerConfig::safety_critical());

    scheduler
        .add(TestNode::new("safety_monitor"))
        .order(0)
        .done();
    scheduler
        .add(TestNode::new("emergency_stop"))
        .order(0)
        .done();

    let result = scheduler.run_for(std::time::Duration::from_millis(50));
    assert!(result.is_ok());
}

#[test]
fn test_high_performance_config() {
    cleanup_stale_shm();
    // Apply high-performance robot configuration
    let mut scheduler = Scheduler::new().with_config(SchedulerConfig::high_performance());

    scheduler.add(TestNode::new("fast_sensor")).order(0).done();
    scheduler.add(TestNode::new("fast_control")).order(1).done();

    let result = scheduler.run_for(std::time::Duration::from_millis(50));
    assert!(result.is_ok());
}

#[test]
fn test_space_robot_config() {
    cleanup_stale_shm();
    // Apply space robot configuration using standard config
    let mut scheduler = Scheduler::new().with_config(SchedulerConfig::standard());

    scheduler.add(TestNode::new("navigation")).order(0).done();
    scheduler.add(TestNode::new("solar_panel")).order(5).done();

    let result = scheduler.run_for(std::time::Duration::from_millis(100));
    assert!(result.is_ok());
}

#[test]
fn test_custom_exotic_robot_config() {
    cleanup_stale_shm();
    // Create custom configuration by mutating a preset's fields directly
    let mut config = SchedulerConfig::standard();
    config.timing.global_rate_hz = 500.0;
    config.realtime.wcet_enforcement = true;
    config.realtime.safety_monitor = true;

    let mut scheduler = Scheduler::new().with_config(config);

    scheduler.add(TestNode::new("bio_sensor")).order(0).done();
    scheduler
        .add(TestNode::new("quantum_controller"))
        .order(1)
        .done();

    let result = scheduler.run_for(std::time::Duration::from_millis(100));
    assert!(result.is_ok());
}

#[test]
fn test_execution_modes() {
    cleanup_stale_shm();
    // Test Sequential mode
    {
        let mut config = SchedulerConfig::standard();
        config.execution = ExecutionMode::Sequential;
        let mut scheduler = Scheduler::new().with_config(config);

        scheduler.add(TestNode::new("seq_node")).order(0).done();
        let result = scheduler.run_for(std::time::Duration::from_millis(50));
        assert!(result.is_ok());
    }

    // Test Parallel mode
    {
        let mut config = SchedulerConfig::standard();
        config.execution = ExecutionMode::Parallel;
        let mut scheduler = Scheduler::new().with_config(config);

        scheduler.add(TestNode::new("par_node1")).order(0).done();
        scheduler.add(TestNode::new("par_node2")).order(0).done();
        let result = scheduler.run_for(std::time::Duration::from_millis(50));
        assert!(result.is_ok());
    }
}

#[test]
fn test_swarm_config() {
    cleanup_stale_shm();
    // Apply swarm robotics configuration using standard config
    let mut scheduler = Scheduler::new().with_config(SchedulerConfig::standard());

    scheduler.add(TestNode::new("swarm_comm")).order(0).done();
    scheduler
        .add(TestNode::new("swarm_behavior"))
        .order(1)
        .done();

    let result = scheduler.run_for(std::time::Duration::from_millis(100));
    assert!(result.is_ok());
}

#[test]
fn test_soft_robotics_config() {
    cleanup_stale_shm();
    // Apply soft robotics configuration using standard config
    let mut scheduler = Scheduler::new().with_config(SchedulerConfig::standard());

    scheduler
        .add(TestNode::new("pressure_sensor"))
        .order(0)
        .done();
    scheduler
        .add(TestNode::new("soft_actuator"))
        .order(1)
        .done();

    let result = scheduler.run_for(std::time::Duration::from_millis(100));
    assert!(result.is_ok());
}

#[test]
fn test_high_performance_optimizer_nodes() {
    cleanup_stale_shm();
    // Test high-performance configuration with optimizer nodes
    let mut scheduler = Scheduler::new().with_config(SchedulerConfig::high_performance());

    scheduler.add(TestNode::new("perf_sensor")).order(0).done();
    scheduler
        .add(TestNode::new("perf_optimizer"))
        .order(1)
        .done();

    let result = scheduler.run_for(std::time::Duration::from_millis(100));
    assert!(result.is_ok());
}
