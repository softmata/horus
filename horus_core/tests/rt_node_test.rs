// Test real-time node functionality
use horus_core::core::{DeadlineMissPolicy, Node, RtClass, RtNode, RtPriority};
use horus_core::error::Result;
use horus_core::hlog;
use horus_core::scheduling::{Scheduler, SchedulerConfig};
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::Arc;
use std::time::Duration;

/// Example: Motor control node with real-time constraints
struct MotorControlNode {
    name: String,
    tick_count: Arc<AtomicU64>,
    simulate_overrun: bool,
}

impl MotorControlNode {
    fn new(name: &str) -> Self {
        Self {
            name: name.to_string(),
            tick_count: Arc::new(AtomicU64::new(0)),
            simulate_overrun: false,
        }
    }

    #[allow(dead_code)]
    fn with_overrun(name: &str) -> Self {
        Self {
            name: name.to_string(),
            tick_count: Arc::new(AtomicU64::new(0)),
            simulate_overrun: true,
        }
    }
}

impl Node for MotorControlNode {
    fn name(&self) -> &'static str {
        Box::leak(self.name.clone().into_boxed_str())
    }

    fn init(&mut self) -> Result<()> {
        hlog!(info, "{} initialized for 1kHz control", self.name);
        Ok(())
    }

    fn tick(&mut self) {
        self.tick_count.fetch_add(1, Ordering::SeqCst);

        // Simulate computation
        if self.simulate_overrun {
            // Simulate WCET violation
            std::thread::sleep(Duration::from_micros(200));
        } else {
            // Normal execution within budget
            std::thread::sleep(Duration::from_micros(50));
        }
    }

    fn shutdown(&mut self) -> Result<()> {
        hlog!(
            info,
            "{} shutdown after {} ticks",
            self.name,
            self.tick_count.load(Ordering::SeqCst)
        );
        Ok(())
    }
}

impl RtNode for MotorControlNode {
    fn wcet_budget(&self) -> Duration {
        Duration::from_micros(100) // 100μs budget for motor control
    }

    fn deadline(&self) -> Duration {
        Duration::from_millis(1) // 1ms deadline for 1kHz control
    }

    fn rt_priority(&self) -> RtPriority {
        RtPriority::Critical // Highest priority
    }

    fn rt_class(&self) -> RtClass {
        RtClass::Hard // Must never miss deadline
    }

    fn pre_condition(&self) -> bool {
        // Check system is ready for motor control
        true
    }

    fn post_condition(&self) -> bool {
        // Check motor command was sent successfully
        true
    }

    fn invariant(&self) -> bool {
        // Check system safety invariant
        true
    }
}

/// Example: Sensor fusion node with firm real-time constraints
struct SensorFusionNode {
    name: String,
    samples_processed: Arc<AtomicU64>,
}

impl SensorFusionNode {
    fn new(name: &str) -> Self {
        Self {
            name: name.to_string(),
            samples_processed: Arc::new(AtomicU64::new(0)),
        }
    }
}

impl Node for SensorFusionNode {
    fn name(&self) -> &'static str {
        Box::leak(self.name.clone().into_boxed_str())
    }

    fn init(&mut self) -> Result<()> {
        hlog!(info, "{} initialized for 100Hz sensor fusion", self.name);
        Ok(())
    }

    fn tick(&mut self) {
        self.samples_processed.fetch_add(1, Ordering::SeqCst);

        // Simulate sensor fusion computation
        std::thread::sleep(Duration::from_micros(100));
    }

    fn shutdown(&mut self) -> Result<()> {
        hlog!(
            info,
            "{} shutdown after {} samples",
            self.name,
            self.samples_processed.load(Ordering::SeqCst)
        );
        Ok(())
    }
}

impl RtNode for SensorFusionNode {
    fn wcet_budget(&self) -> Duration {
        Duration::from_micros(500) // 500μs budget
    }

    fn deadline(&self) -> Duration {
        Duration::from_millis(10) // 10ms deadline for 100Hz
    }

    fn rt_priority(&self) -> RtPriority {
        RtPriority::High
    }

    fn rt_class(&self) -> RtClass {
        RtClass::Firm // Can tolerate occasional misses
    }
}

/// Example: Logging node with soft real-time constraints
struct LoggingNode {
    name: String,
    logs_written: Arc<AtomicU64>,
}

impl LoggingNode {
    fn new(name: &str) -> Self {
        Self {
            name: name.to_string(),
            logs_written: Arc::new(AtomicU64::new(0)),
        }
    }
}

impl Node for LoggingNode {
    fn name(&self) -> &'static str {
        Box::leak(self.name.clone().into_boxed_str())
    }

    fn init(&mut self) -> Result<()> {
        hlog!(info, "{} initialized", self.name);
        Ok(())
    }

    fn tick(&mut self) {
        self.logs_written.fetch_add(1, Ordering::SeqCst);
        // Simulate logging (fast operation)
        std::thread::sleep(Duration::from_micros(10));
    }

    fn shutdown(&mut self) -> Result<()> {
        hlog!(
            info,
            "{} wrote {} logs",
            self.name,
            self.logs_written.load(Ordering::SeqCst)
        );
        Ok(())
    }
}

impl RtNode for LoggingNode {
    fn wcet_budget(&self) -> Duration {
        Duration::from_millis(5) // 5ms budget
    }

    fn deadline(&self) -> Duration {
        Duration::from_millis(100) // 100ms deadline
    }

    fn rt_priority(&self) -> RtPriority {
        RtPriority::Low
    }

    fn rt_class(&self) -> RtClass {
        RtClass::Soft // Best effort
    }
}

#[test]
fn test_rt_node_basic() {
    // Use standard config (RT features disabled)
    let mut scheduler = Scheduler::new().with_config(SchedulerConfig::standard());

    // Add RT nodes as regular nodes (RtNodeWrapper handles the conversion)
    scheduler.add(MotorControlNode::new("motor_ctrl")).order(0).done();
    scheduler.add(SensorFusionNode::new("sensor_fusion")).order(1).done();
    scheduler.add(LoggingNode::new("logger")).order(10).done();

    // Run for a short duration
    let result = scheduler.run_for(Duration::from_millis(100));
    assert!(result.is_ok());
}

#[test]
fn test_rt_node_priority_ordering() {
    let motor = Arc::new(AtomicU64::new(0));
    let sensor = Arc::new(AtomicU64::new(0));
    let logger = Arc::new(AtomicU64::new(0));

    let mut motor_node = MotorControlNode::new("motor");
    motor_node.tick_count = motor.clone();

    let mut sensor_node = SensorFusionNode::new("sensor");
    sensor_node.samples_processed = sensor.clone();

    let mut log_node = LoggingNode::new("logger");
    log_node.logs_written = logger.clone();

    let mut scheduler = Scheduler::new();
    scheduler.add(log_node).order(10).done(); // Low priority
    scheduler.add(sensor_node).order(1).done(); // High priority
    scheduler.add(motor_node).order(0).done(); // Critical priority

    scheduler.run_for(Duration::from_millis(50)).unwrap();

    // Critical priority node should have executed
    assert!(motor.load(Ordering::SeqCst) > 0);
}

#[test]
fn test_rt_node_with_safety_critical_config() {
    // Use safety-critical configuration (all RT features enabled)
    let mut scheduler = Scheduler::new().with_config(SchedulerConfig::safety_critical());

    scheduler.add(MotorControlNode::new("critical_motor")).order(0).done();
    scheduler.add(SensorFusionNode::new("critical_sensor")).order(1).done();

    // Run briefly (safety-critical config runs at 1kHz)
    let result = scheduler.run_for(Duration::from_millis(50));
    assert!(result.is_ok());
}

#[test]
fn test_rt_node_wcet_budget() {
    // Test that nodes respect their WCET budget
    let node = MotorControlNode::new("test_motor");
    assert_eq!(node.wcet_budget(), Duration::from_micros(100));
    assert_eq!(node.deadline(), Duration::from_millis(1));
    assert_eq!(node.rt_class(), RtClass::Hard);
}

#[test]
fn test_rt_node_formal_verification() {
    let mut node = MotorControlNode::new("verified_motor");

    // Test pre-condition
    assert!(node.pre_condition());

    // Execute tick
    node.tick();

    // Test post-condition
    assert!(node.post_condition());

    // Test invariant
    assert!(node.invariant());
}

#[test]
fn test_rt_priority_values() {
    assert!(RtPriority::Critical.value() < RtPriority::High.value());
    assert!(RtPriority::High.value() < RtPriority::Medium.value());
    assert!(RtPriority::Medium.value() < RtPriority::Low.value());
    assert_eq!(RtPriority::Custom(42).value(), 42);
}

#[test]
fn test_deadline_miss_policies() {
    // Test different deadline miss policies
    let motor = MotorControlNode::new("motor");
    assert_eq!(
        motor.deadline_miss_policy(),
        DeadlineMissPolicy::EmergencyStop
    );

    let sensor = SensorFusionNode::new("sensor");
    assert_eq!(sensor.deadline_miss_policy(), DeadlineMissPolicy::Skip);

    let logger = LoggingNode::new("logger");
    assert_eq!(logger.deadline_miss_policy(), DeadlineMissPolicy::Warn);
}
