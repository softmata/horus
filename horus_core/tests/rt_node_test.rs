// Test real-time node functionality
use horus_core::error::HorusResult as Result;
use horus_core::hlog;
use horus_core::scheduling::Scheduler;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::Arc;

mod common;
use common::cleanup_stale_shm;
use horus_core::core::{DurationExt, Miss, Node};

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
            // Simulate budget violation
            std::thread::sleep(200_u64.us());
        } else {
            // Normal execution within budget
            std::thread::sleep(50_u64.us());
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
        std::thread::sleep(100_u64.us());
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
        std::thread::sleep(10_u64.us());
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

#[test]
fn test_rt_node_basic() {
    cleanup_stale_shm();
    // Use default config (RT features disabled)
    let mut scheduler = Scheduler::new();

    // Add RT nodes as regular nodes
    scheduler
        .add(MotorControlNode::new("motor_ctrl"))
        .order(0)
        .budget(100_u64.us())
        .deadline(1_u64.ms())
        .on_miss(Miss::Stop)
        .build();
    scheduler
        .add(SensorFusionNode::new("sensor_fusion"))
        .order(1)
        .budget(500_u64.us())
        .deadline(10_u64.ms())
        .on_miss(Miss::Skip)
        .build();
    scheduler
        .add(LoggingNode::new("logger"))
        .order(10)
        .budget(5000_u64.us())
        .deadline(100_u64.ms())
        .build();

    // Run for a short duration
    let result = scheduler.run_for(100_u64.ms());
    result.unwrap();
}

#[test]
fn test_rt_node_priority_ordering() {
    cleanup_stale_shm();
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
    scheduler
        .add(log_node)
        .order(10)
        .budget(5000_u64.us())
        .deadline(100_u64.ms())
        .build(); // Low priority
    scheduler
        .add(sensor_node)
        .order(1)
        .budget(500_u64.us())
        .deadline(10_u64.ms())
        .on_miss(Miss::Skip)
        .build(); // High priority
    scheduler
        .add(motor_node)
        .order(0)
        .budget(100_u64.us())
        .deadline(1_u64.ms())
        .on_miss(Miss::Stop)
        .build(); // Critical priority

    scheduler.run_for(200_u64.ms()).unwrap();

    // Critical priority node should have executed
    assert!(motor.load(Ordering::SeqCst) > 0);
}

#[test]
fn test_rt_node_with_safety_critical_config() {
    cleanup_stale_shm();
    // Use safety-critical configuration (all RT features enabled)
    let mut scheduler = Scheduler::new().tick_rate(1000_u64.hz());

    scheduler
        .add(MotorControlNode::new("critical_motor"))
        .order(0)
        .budget(100_u64.us())
        .deadline(1_u64.ms())
        .on_miss(Miss::Stop)
        .build();
    scheduler
        .add(SensorFusionNode::new("critical_sensor"))
        .order(1)
        .budget(500_u64.us())
        .deadline(10_u64.ms())
        .on_miss(Miss::Skip)
        .build();

    // Run briefly (safety-critical config runs at 1kHz)
    let result = scheduler.run_for(200_u64.ms());
    result.unwrap();
}
