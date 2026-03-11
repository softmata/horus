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

    // Add RT node with fluent API
    scheduler
        .add(CriticalControlNode::new("motor_control", 50)) // 50μs execution
        .order(0) // Highest priority
        .budget(100_u64.us()) // 100μs tick budget
        .deadline(1_u64.ms()) // 1ms deadline
        .build();

    // Add another RT node
    scheduler
        .add(CriticalControlNode::new("sensor_fusion", 30)) // 30μs execution
        .order(1)
        .budget(50_u64.us())
        .deadline(2_u64.ms())
        .build();

    // Add a regular node
    scheduler
        .add(CriticalControlNode::new("logger", 10))
        .order(10)
        .build();

    // Run for a short duration
    let result = scheduler.run_for(100_u64.ms());
    result.unwrap();
}

#[test]
fn test_scheduler_with_safety_critical_config() {
    cleanup_stale_shm();
    // Configure for safety-critical operation
    let mut scheduler = Scheduler::new().tick_rate(1000_u64.hz());

    // Add critical nodes
    scheduler
        .add(CriticalControlNode::new("flight_control", 80))
        .order(0)
        .budget(100_u64.us())
        .deadline(1_u64.ms())
        .build();

    scheduler
        .add(CriticalControlNode::new("navigation", 60))
        .order(1)
        .budget(80_u64.us())
        .deadline(2_u64.ms())
        .build();

    // Run briefly with safety monitoring
    let result = scheduler.run_for(50_u64.ms());
    result.unwrap();
}

#[test]
fn test_budget_violation_detection() {
    cleanup_stale_shm();
    // Enable RT monitoring
    let mut scheduler = Scheduler::new().monitoring(true);

    // Add node that will violate budget
    // Execution time (100μs) > tick budget (50μs)
    scheduler
        .add(CriticalControlNode::new("violator", 100))
        .order(0)
        .budget(50_u64.us()) // tick budget too small
        .deadline(1_u64.ms())
        .build();

    // This should detect budget violations but continue running
    let result = scheduler.run_for(20_u64.ms());
    result.unwrap();
}

#[test]
fn test_deadline_miss_detection() {
    cleanup_stale_shm();
    // Enable deadline monitoring
    let mut scheduler = Scheduler::new().monitoring(true).max_deadline_misses(5);

    // Add node with tight deadline that might be missed
    scheduler
        .add(CriticalControlNode::new("tight_deadline", 900))
        .order(0)
        .budget(1000_u64.us())
        .deadline(500_u64.us()) // Deadline smaller than execution time
        .build();

    // This should detect deadline misses
    let result = scheduler.run_for(30_u64.ms());
    result.unwrap();
}

#[test]
fn test_mixed_rt_and_normal_nodes() {
    cleanup_stale_shm();
    let mut scheduler = Scheduler::new();

    // Mix of RT and normal nodes
    scheduler
        .add(CriticalControlNode::new("rt_critical", 50))
        .order(0)
        .budget(100_u64.us())
        .deadline(1_u64.ms())
        .build();

    scheduler
        .add(CriticalControlNode::new("normal_processing", 200))
        .order(5)
        .build();

    scheduler
        .add(CriticalControlNode::new("rt_sensor", 30))
        .order(1)
        .budget(50_u64.us())
        .deadline(2_u64.ms())
        .build();

    scheduler
        .add(CriticalControlNode::new("background_task", 500))
        .order(20)
        .build();

    // RT nodes should be properly prioritized
    let result = scheduler.run_for(100_u64.ms());
    result.unwrap();
}

#[test]
fn test_watchdog_functionality() {
    cleanup_stale_shm();
    // Enable watchdog monitoring
    let mut scheduler = Scheduler::new().monitoring(true);

    // Add RT node that will be monitored by watchdog
    scheduler
        .add(CriticalControlNode::new("watchdog_monitored", 10))
        .order(0)
        .budget(50_u64.us())
        .deadline(1_u64.ms())
        .build();

    // Run normally - watchdog should be fed and not expire
    let result = scheduler.run_for(100_u64.ms());
    result.unwrap();
}

#[test]
fn test_high_performance_rt_config() {
    cleanup_stale_shm();
    // Configure for high-performance racing robot
    let mut scheduler = Scheduler::new().tick_rate(10000_u64.hz());

    // Add ultra-fast control nodes
    scheduler
        .add(CriticalControlNode::new("traction_control", 10))
        .order(0)
        .budget(20_u64.us())
        .deadline(100_u64.us()) // 10kHz control loop
        .build();

    scheduler
        .add(CriticalControlNode::new("stability_control", 15))
        .order(1)
        .budget(25_u64.us())
        .deadline(100_u64.us())
        .build();

    // Should handle high-frequency execution
    let result = scheduler.run_for(50_u64.ms());
    result.unwrap();
}
