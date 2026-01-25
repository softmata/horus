// Test comprehensive scheduler configuration
use horus_core::core::Node;
use horus_core::error::Result;
use horus_core::hlog;
use horus_core::scheduling::{ConfigValue, ExecutionMode, Scheduler, SchedulerConfig};

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
        hlog!(info, "{} shutdown after {} ticks", self.name, self.tick_count);
        Ok(())
    }
}

#[test]
fn test_standard_config() {
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
    // Apply safety-critical robot configuration
    let mut scheduler = Scheduler::new().with_config(SchedulerConfig::safety_critical());

    scheduler.add(TestNode::new("safety_monitor")).order(0).done();
    scheduler.add(TestNode::new("emergency_stop")).order(0).done();

    let result = scheduler.run_for(std::time::Duration::from_millis(50));
    assert!(result.is_ok());
}

#[test]
fn test_high_performance_config() {
    // Apply high-performance robot configuration
    let mut scheduler = Scheduler::new().with_config(SchedulerConfig::high_performance());

    scheduler.add(TestNode::new("fast_sensor")).order(0).done();
    scheduler.add(TestNode::new("fast_control")).order(1).done();

    let result = scheduler.run_for(std::time::Duration::from_millis(50));
    assert!(result.is_ok());
}

#[test]
fn test_space_robot_config() {
    // Apply space robot configuration using standard config
    let mut scheduler = Scheduler::new().with_config(SchedulerConfig::standard());

    scheduler.add(TestNode::new("navigation")).order(0).done();
    scheduler.add(TestNode::new("solar_panel")).order(5).done();

    let result = scheduler.run_for(std::time::Duration::from_millis(100));
    assert!(result.is_ok());
}

#[test]
fn test_custom_exotic_robot_config() {
    // Create fully custom configuration for an exotic robot type
    let mut config = SchedulerConfig::standard();
    config
        .custom
        .insert("bio_neural_network".to_string(), ConfigValue::Bool(true));
    config.custom.insert(
        "quantum_processor".to_string(),
        ConfigValue::String("entangled".to_string()),
    );
    config
        .custom
        .insert("organic_actuators".to_string(), ConfigValue::Integer(8));
    config.custom.insert(
        "photosynthesis_efficiency".to_string(),
        ConfigValue::Float(0.85),
    );

    let mut scheduler = Scheduler::new().with_config(config);

    scheduler.add(TestNode::new("bio_sensor")).order(0).done();
    scheduler.add(TestNode::new("quantum_controller")).order(1).done();

    let result = scheduler.run_for(std::time::Duration::from_millis(100));
    assert!(result.is_ok());
}

#[test]
fn test_execution_modes() {
    // Test JIT optimized mode
    {
        let mut config = SchedulerConfig::standard();
        config.execution = ExecutionMode::JITOptimized;
        let mut scheduler = Scheduler::new().with_config(config);

        scheduler.add(TestNode::new("jit_node")).order(0).done();
        let result = scheduler.run_for(std::time::Duration::from_millis(50));
        assert!(result.is_ok());
    }

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
    // Apply swarm robotics configuration using standard config
    let mut scheduler = Scheduler::new().with_config(SchedulerConfig::standard());

    scheduler.add(TestNode::new("swarm_comm")).order(0).done();
    scheduler.add(TestNode::new("swarm_behavior")).order(1).done();

    let result = scheduler.run_for(std::time::Duration::from_millis(100));
    assert!(result.is_ok());
}

#[test]
fn test_soft_robotics_config() {
    // Apply soft robotics configuration using standard config
    let mut scheduler = Scheduler::new().with_config(SchedulerConfig::standard());

    scheduler.add(TestNode::new("pressure_sensor")).order(0).done();
    scheduler.add(TestNode::new("soft_actuator")).order(1).done();

    let result = scheduler.run_for(std::time::Duration::from_millis(100));
    assert!(result.is_ok());
}

#[test]
fn test_high_performance_optimizer_nodes() {
    // Test high-performance configuration with optimizer nodes
    let mut scheduler = Scheduler::new().with_config(SchedulerConfig::high_performance());

    scheduler.add(TestNode::new("perf_sensor")).order(0).done();
    scheduler.add(TestNode::new("perf_optimizer")).order(1).done();

    let result = scheduler.run_for(std::time::Duration::from_millis(100));
    assert!(result.is_ok());
}
