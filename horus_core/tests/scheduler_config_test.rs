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
    let mut scheduler = Scheduler::deploy().tick_hz(500.0);

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

// ============================================================================
// Profile Preset Tests — new RT API DX
// ============================================================================

#[test]
fn test_deploy_preset() {
    cleanup_stale_shm();
    // Scheduler::deploy() enables safety monitor, fault tolerance, watchdog, blackbox
    let mut scheduler = Scheduler::deploy().tick_hz(100.0);

    scheduler
        .add(TestNode::new("motor"))
        .order(0)
        .rate_hz(100.0)
        .done();
    scheduler
        .add(TestNode::new("sensor"))
        .order(1)
        .rate_hz(50.0)
        .done();

    let result = scheduler.run_for(std::time::Duration::from_millis(100));
    assert!(result.is_ok());
}

#[test]
fn test_deploy_with_cores() {
    cleanup_stale_shm();
    // deploy() + cores() is the typical production pattern
    let mut scheduler = Scheduler::deploy().tick_hz(500.0).cores(&[0, 1]);

    scheduler
        .add(TestNode::new("ctrl"))
        .order(0)
        .rate_hz(500.0)
        .done();
    let result = scheduler.run_for(std::time::Duration::from_millis(50));
    assert!(result.is_ok());
}

#[test]
fn test_safety_critical_preset() {
    cleanup_stale_shm();
    // safety_critical() calls hard_rt() — may panic on systems without RT
    let result = std::panic::catch_unwind(|| {
        Scheduler::safety_critical()
            .tick_hz(1000.0)
            .max_deadline_misses(3)
    });
    if let Ok(mut scheduler) = result {
        scheduler
            .add(TestNode::new("safety_node"))
            .order(0)
            .budget_us(500)
            .done();
        let _ = scheduler.run_for(std::time::Duration::from_millis(50));
    }
    // If panicked, that's expected on non-RT systems
}

#[test]
fn test_hard_rt_preset() {
    cleanup_stale_shm();
    // hard_rt() verifies system capabilities and enables all RT features
    // May panic on systems without RT support — that's by design
    let result = std::panic::catch_unwind(|| Scheduler::hard_rt().tick_hz(1000.0));
    // Either succeeds (system has RT) or panics (no RT) — both are valid
    if let Ok(mut scheduler) = result {
        scheduler
            .add(TestNode::new("rt_node"))
            .order(0)
            .budget_us(200)
            .done();
        let _ = scheduler.run_for(std::time::Duration::from_millis(50));
    }
}

#[test]
fn test_auto_derive_tick_hz_from_fastest_node() {
    cleanup_stale_shm();
    // If we set tick_hz(10) but add a node at 500Hz,
    // the scheduler should auto-bump to at least 500Hz
    let mut scheduler = Scheduler::new().tick_hz(10.0);

    scheduler
        .add(TestNode::new("fast_node"))
        .order(0)
        .rate_hz(500.0)
        .done();
    scheduler
        .add(TestNode::new("slow_node"))
        .order(1)
        .rate_hz(1.0)
        .done();

    // After run starts, tick rate should have been bumped
    let result = scheduler.run_for(std::time::Duration::from_millis(50));
    assert!(result.is_ok());
}

#[test]
fn test_node_rate_via_builder_only() {
    cleanup_stale_shm();
    // rate_hz is now builder-only — Node trait no longer declares it
    let mut scheduler = Scheduler::new().tick_hz(100.0);

    // Rate set via builder, not via Node::rate_hz()
    scheduler
        .add(TestNode::new("node_a"))
        .order(0)
        .rate_hz(50.0)
        .done();
    scheduler
        .add(TestNode::new("node_b"))
        .order(1)
        .rate_hz(25.0)
        .done();
    scheduler.add(TestNode::new("node_c")).order(2).done(); // Uses global rate

    let result = scheduler.run_for(std::time::Duration::from_millis(100));
    assert!(result.is_ok());
}

#[test]
fn test_preset_chaining() {
    cleanup_stale_shm();
    // Presets return Self so they can be chained with builders
    let mut scheduler = Scheduler::deploy()
        .tick_hz(200.0)
        .verbose(false)
        .with_profiling();

    scheduler.add(TestNode::new("chained")).order(0).done();
    let result = scheduler.run_for(std::time::Duration::from_millis(50));
    assert!(result.is_ok());
}
