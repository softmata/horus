// Test real-time node functionality
use horus_core::error::Result;
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
    let mut scheduler = Scheduler::new();

    let motor = MotorControlNode::new("motor_ctrl");
    let motor_ticks = Arc::clone(&motor.tick_count);
    let fusion = SensorFusionNode::new("sensor_fusion");
    let fusion_ticks = Arc::clone(&fusion.samples_processed);
    let logger = LoggingNode::new("logger");
    let logger_ticks = Arc::clone(&logger.logs_written);

    scheduler.add(motor).order(0).on_miss(Miss::Stop).build();
    scheduler.add(fusion).order(1).on_miss(Miss::Skip).build();
    scheduler.add(logger).order(10).build();

    scheduler.run_for(100_u64.ms()).unwrap();

    assert!(
        motor_ticks.load(Ordering::SeqCst) > 0,
        "Motor control node should have ticked"
    );
    assert!(
        fusion_ticks.load(Ordering::SeqCst) > 0,
        "Sensor fusion node should have ticked"
    );
    assert!(
        logger_ticks.load(Ordering::SeqCst) > 0,
        "Logger node should have ticked"
    );
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
    scheduler.add(log_node).order(10).build(); // Low priority
    scheduler
        .add(sensor_node)
        .order(1)
        .on_miss(Miss::Skip)
        .build(); // High priority
    scheduler
        .add(motor_node)
        .order(0)
        .on_miss(Miss::Stop)
        .build(); // Critical priority

    scheduler.run_for(200_u64.ms()).unwrap();

    let motor_count = motor.load(Ordering::SeqCst);
    let sensor_count = sensor.load(Ordering::SeqCst);
    let logger_count = logger.load(Ordering::SeqCst);

    // All nodes should have executed
    assert!(motor_count > 0, "Critical motor node never ticked");
    assert!(sensor_count > 0, "Sensor node never ticked");
    assert!(logger_count > 0, "Logger node never ticked");

    // Motor (50μs exec, order 0) should tick at least as much as sensor (100μs exec, order 1)
    assert!(
        motor_count >= sensor_count,
        "Motor ({}, 50μs) should tick >= sensor ({}, 100μs) due to lower exec time",
        motor_count,
        sensor_count
    );
}

#[test]
fn test_rt_node_with_safety_critical_config() {
    cleanup_stale_shm();
    let mut scheduler = Scheduler::new().tick_rate(1000_u64.hz());

    let motor = MotorControlNode::new("critical_motor");
    let motor_ticks = Arc::clone(&motor.tick_count);
    let sensor = SensorFusionNode::new("critical_sensor");
    let sensor_ticks = Arc::clone(&sensor.samples_processed);

    scheduler.add(motor).order(0).on_miss(Miss::Stop).build();
    scheduler.add(sensor).order(1).on_miss(Miss::Skip).build();

    // Run at 1kHz for 200ms — expect ~200 ticks ideally
    scheduler.run_for(200_u64.ms()).unwrap();

    let motor_count = motor_ticks.load(Ordering::SeqCst);
    let sensor_count = sensor_ticks.load(Ordering::SeqCst);

    assert!(motor_count > 0, "Critical motor should have ticked at 1kHz");
    assert!(
        sensor_count > 0,
        "Critical sensor should have ticked at 1kHz"
    );
}
