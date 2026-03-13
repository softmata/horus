// Integration test for real-time scheduler features
use horus_core::error::HorusResult as Result;
use horus_core::hlog;
use horus_core::scheduling::Scheduler;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::Arc;

mod common;
use common::cleanup_stale_shm;
use horus_core::core::{DurationExt, Node};

/// Critical control node that must never miss deadlines
struct CriticalControlNode {
    name: String,
    tick_count: Arc<AtomicU64>,
    execution_time_us: u64,
}

impl CriticalControlNode {
    fn new(name: &str, execution_time_us: u64) -> Self {
        Self {
            name: name.to_string(),
            tick_count: Arc::new(AtomicU64::new(0)),
            execution_time_us,
        }
    }
}

impl Node for CriticalControlNode {
    fn name(&self) -> &'static str {
        Box::leak(self.name.clone().into_boxed_str())
    }

    fn init(&mut self) -> Result<()> {
        hlog!(
            info,
            "{} initialized for safety-critical operation",
            self.name
        );
        Ok(())
    }

    fn tick(&mut self) {
        self.tick_count.fetch_add(1, Ordering::SeqCst);

        // Simulate computation
        std::thread::sleep(self.execution_time_us.us());
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

#[test]
fn test_scheduler_with_rt_nodes() {
    cleanup_stale_shm();
    let mut scheduler = Scheduler::new();

    let motor = CriticalControlNode::new("motor_control", 50);
    let motor_ticks = Arc::clone(&motor.tick_count);
    let fusion = CriticalControlNode::new("sensor_fusion", 30);
    let fusion_ticks = Arc::clone(&fusion.tick_count);
    let logger = CriticalControlNode::new("logger", 10);
    let logger_ticks = Arc::clone(&logger.tick_count);

    scheduler.add(motor).order(0).build();
    scheduler.add(fusion).order(1).build();
    scheduler.add(logger).order(10).build();

    scheduler.run_for(100_u64.ms()).unwrap();

    let motor_count = motor_ticks.load(Ordering::SeqCst);
    let fusion_count = fusion_ticks.load(Ordering::SeqCst);
    let logger_count = logger_ticks.load(Ordering::SeqCst);

    assert!(motor_count > 0, "RT motor_control node never ticked");
    assert!(fusion_count > 0, "RT sensor_fusion node never ticked");
    assert!(logger_count > 0, "Regular logger node never ticked");
}

#[test]
fn test_scheduler_with_safety_critical_config() {
    cleanup_stale_shm();
    let mut scheduler = Scheduler::new().tick_rate(1000_u64.hz());

    let flight = CriticalControlNode::new("flight_control", 80);
    let flight_ticks = Arc::clone(&flight.tick_count);
    let nav = CriticalControlNode::new("navigation", 60);
    let nav_ticks = Arc::clone(&nav.tick_count);

    scheduler.add(flight).order(0).build();
    scheduler.add(nav).order(1).build();

    scheduler.run_for(50_u64.ms()).unwrap();

    let flight_count = flight_ticks.load(Ordering::SeqCst);
    let nav_count = nav_ticks.load(Ordering::SeqCst);

    assert!(flight_count > 0, "flight_control node never ticked in safety-critical config");
    assert!(nav_count > 0, "navigation node never ticked in safety-critical config");
}

#[test]
fn test_mixed_rt_and_normal_nodes() {
    cleanup_stale_shm();
    let mut scheduler = Scheduler::new();

    let rt_crit = CriticalControlNode::new("rt_critical", 50);
    let rt_crit_ticks = Arc::clone(&rt_crit.tick_count);
    let normal = CriticalControlNode::new("normal_processing", 200);
    let normal_ticks = Arc::clone(&normal.tick_count);
    let rt_sensor = CriticalControlNode::new("rt_sensor", 30);
    let rt_sensor_ticks = Arc::clone(&rt_sensor.tick_count);
    let background = CriticalControlNode::new("background_task", 500);
    let bg_ticks = Arc::clone(&background.tick_count);

    scheduler.add(rt_crit).order(0).build();
    scheduler.add(normal).order(5).build();
    scheduler.add(rt_sensor).order(1).build();
    scheduler.add(background).order(20).build();

    scheduler.run_for(100_u64.ms()).unwrap();

    let rt_count = rt_crit_ticks.load(Ordering::SeqCst);
    let sensor_count = rt_sensor_ticks.load(Ordering::SeqCst);
    let normal_count = normal_ticks.load(Ordering::SeqCst);
    let bg_count = bg_ticks.load(Ordering::SeqCst);

    assert!(rt_count > 0, "RT critical node never ticked");
    assert!(sensor_count > 0, "RT sensor node never ticked");
    assert!(normal_count > 0, "Normal processing node never ticked");
    assert!(bg_count > 0, "Background task node never ticked");

    // All nodes share the same sequential tick loop, so tick counts should
    // be similar. Verify RT nodes tick at least as much as background nodes.
    assert!(
        rt_count >= bg_count,
        "RT critical ({}) should tick at least as much as background ({})",
        rt_count, bg_count
    );
}

#[test]
fn test_watchdog_functionality() {
    cleanup_stale_shm();
    let mut scheduler = Scheduler::new().watchdog(500_u64.ms());

    let node = CriticalControlNode::new("watchdog_monitored", 10);
    let ticks = Arc::clone(&node.tick_count);

    scheduler.add(node).order(0).build();

    // Run normally — watchdog should be fed and not expire
    scheduler.run_for(100_u64.ms()).unwrap();

    let tick_count = ticks.load(Ordering::SeqCst);
    assert!(
        tick_count > 0,
        "Watchdog-monitored node should tick normally when execution is within budget"
    );
}

#[test]
fn test_high_performance_rt_config() {
    cleanup_stale_shm();
    // Configure for high-performance racing robot at 10kHz
    let mut scheduler = Scheduler::new().tick_rate(10000_u64.hz());

    let traction = CriticalControlNode::new("traction_control", 10);
    let traction_ticks = Arc::clone(&traction.tick_count);
    let stability = CriticalControlNode::new("stability_control", 15);
    let stability_ticks = Arc::clone(&stability.tick_count);

    scheduler.add(traction).order(0).build();
    scheduler.add(stability).order(1).build();

    scheduler.run_for(50_u64.ms()).unwrap();

    let traction_count = traction_ticks.load(Ordering::SeqCst);
    let stability_count = stability_ticks.load(Ordering::SeqCst);

    // At 10kHz for 50ms, expect ~500 ticks ideally. Use conservative threshold for CI.
    assert!(
        traction_count > 10,
        "Traction control ticked {} times in 50ms at 10kHz (expected many more)",
        traction_count
    );
    assert!(
        stability_count > 10,
        "Stability control ticked {} times in 50ms at 10kHz (expected many more)",
        stability_count
    );
}
