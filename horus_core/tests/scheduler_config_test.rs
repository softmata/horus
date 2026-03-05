// Test comprehensive scheduler configuration
use horus_core::core::Node;
use horus_core::error::HorusResult as Result;
use horus_core::hlog;
use horus_core::scheduling::Scheduler;

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
    // Apply default robot configuration
    let mut scheduler = Scheduler::new();

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
    let mut scheduler = Scheduler::new().tick_hz(1000.0);

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
    let mut scheduler = Scheduler::new().tick_hz(10000.0);

    scheduler.add(TestNode::new("fast_sensor")).order(0).done();
    scheduler.add(TestNode::new("fast_control")).order(1).done();

    let result = scheduler.run_for(std::time::Duration::from_millis(50));
    assert!(result.is_ok());
}

#[test]
fn test_space_robot_config() {
    cleanup_stale_shm();
    // Apply space robot configuration using default scheduler
    let mut scheduler = Scheduler::new();

    scheduler.add(TestNode::new("navigation")).order(0).done();
    scheduler.add(TestNode::new("solar_panel")).order(5).done();

    let result = scheduler.run_for(std::time::Duration::from_millis(100));
    assert!(result.is_ok());
}

#[test]
fn test_custom_exotic_robot_config() {
    cleanup_stale_shm();
    // Create custom configuration using builder methods
    let mut scheduler = Scheduler::new().tick_hz(500.0).safety_monitor(true);

    scheduler.add(TestNode::new("bio_sensor")).order(0).done();
    scheduler
        .add(TestNode::new("quantum_controller"))
        .order(1)
        .done();

    let result = scheduler.run_for(std::time::Duration::from_millis(100));
    assert!(result.is_ok());
}

#[test]
fn test_execution_classes() {
    cleanup_stale_shm();
    // Test BestEffort nodes (main thread)
    {
        let mut scheduler = Scheduler::new();
        scheduler.add(TestNode::new("seq_node")).order(0).done();
        let result = scheduler.run_for(std::time::Duration::from_millis(50));
        assert!(result.is_ok());
    }

    // Test mixed execution classes: compute + best-effort
    {
        let mut scheduler = Scheduler::new();
        scheduler
            .add(TestNode::new("compute_node"))
            .order(0)
            .compute()
            .done();
        scheduler.add(TestNode::new("main_node")).order(1).done();
        let result = scheduler.run_for(std::time::Duration::from_millis(50));
        assert!(result.is_ok());
    }
}

#[test]
fn test_swarm_config() {
    cleanup_stale_shm();
    // Apply swarm robotics configuration using default scheduler
    let mut scheduler = Scheduler::new();

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
    // Apply soft robotics configuration using default scheduler
    let mut scheduler = Scheduler::new();

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
    let mut scheduler = Scheduler::new().tick_hz(10000.0);

    scheduler.add(TestNode::new("perf_sensor")).order(0).done();
    scheduler
        .add(TestNode::new("perf_optimizer"))
        .order(1)
        .done();

    let result = scheduler.run_for(std::time::Duration::from_millis(100));
    assert!(result.is_ok());
}
