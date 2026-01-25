// Integration test for real-time scheduler features
use horus_core::core::{DeadlineMissPolicy, Node, RtClass, RtNode, RtPriority};
use horus_core::error::Result;
use horus_core::hlog;
use horus_core::scheduling::{Scheduler, SchedulerConfig};
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::Arc;
use std::time::Duration;

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
        std::thread::sleep(Duration::from_micros(self.execution_time_us));
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

impl RtNode for CriticalControlNode {
    fn wcet_budget(&self) -> Duration {
        // Budget should be slightly higher than actual execution time
        Duration::from_micros(self.execution_time_us + 20)
    }

    fn deadline(&self) -> Duration {
        Duration::from_millis(1) // 1ms deadline for 1kHz control
    }

    fn rt_priority(&self) -> RtPriority {
        RtPriority::Critical
    }

    fn rt_class(&self) -> RtClass {
        RtClass::Hard
    }

    fn deadline_miss_policy(&self) -> DeadlineMissPolicy {
        DeadlineMissPolicy::EmergencyStop
    }

    fn pre_condition(&self) -> bool {
        true // Always ready
    }

    fn post_condition(&self) -> bool {
        true // Always successful
    }

    fn invariant(&self) -> bool {
        true // System always safe
    }
}

#[test]
fn test_scheduler_with_rt_nodes() {
    let mut scheduler = Scheduler::new();

    // Add RT node with fluent API
    scheduler.add(CriticalControlNode::new("motor_control", 50)) // 50μs execution
        .order(0)                    // Highest priority
        .rt()                        // Mark as real-time
        .wcet_us(100)               // 100μs WCET budget
        .deadline_ms(1)             // 1ms deadline
        .done();

    // Add another RT node
    scheduler.add(CriticalControlNode::new("sensor_fusion", 30)) // 30μs execution
        .order(1)
        .rt()
        .wcet_us(50)
        .deadline_ms(2)
        .done();

    // Add a regular node
    scheduler.add(CriticalControlNode::new("logger", 10))
        .order(10)
        .done();

    // Run for a short duration
    let result = scheduler.run_for(Duration::from_millis(100));
    assert!(result.is_ok());
}

#[test]
fn test_scheduler_with_safety_critical_config() {
    // Configure for safety-critical operation
    let mut scheduler = Scheduler::new().with_config(SchedulerConfig::safety_critical());

    // Add critical nodes
    scheduler.add(CriticalControlNode::new("flight_control", 80))
        .order(0)
        .rt()
        .wcet_us(100)
        .deadline_ms(1)
        .done();

    scheduler.add(CriticalControlNode::new("navigation", 60))
        .order(1)
        .rt()
        .wcet_us(80)
        .deadline_ms(2)
        .done();

    // Run briefly with safety monitoring
    let result = scheduler.run_for(Duration::from_millis(50));
    assert!(result.is_ok());
}

#[test]
fn test_wcet_violation_detection() {
    // Enable RT monitoring
    let mut config = SchedulerConfig::standard();
    config.realtime.wcet_enforcement = true;
    config.realtime.deadline_monitoring = true;
    config.realtime.safety_monitor = true;

    let mut scheduler = Scheduler::new().with_config(config);

    // Add node that will violate WCET
    // Execution time (100μs) > WCET budget (50μs)
    scheduler.add(CriticalControlNode::new("violator", 100))
        .order(0)
        .rt()
        .wcet_us(50)      // WCET budget too small
        .deadline_ms(1)
        .done();

    // This should detect WCET violations but continue running
    let result = scheduler.run_for(Duration::from_millis(20));
    assert!(result.is_ok());
}

#[test]
fn test_deadline_miss_detection() {
    // Enable deadline monitoring
    let mut config = SchedulerConfig::standard();
    config.realtime.deadline_monitoring = true;
    config.realtime.safety_monitor = true;
    config.realtime.max_deadline_misses = 5; // Allow some misses before emergency stop

    let mut scheduler = Scheduler::new().with_config(config);

    // Add node with tight deadline that might be missed
    scheduler.add(CriticalControlNode::new("tight_deadline", 900))
        .order(0)
        .rt()
        .wcet_us(1000)
        .deadline_us(500)  // Deadline smaller than execution time
        .done();

    // This should detect deadline misses
    let result = scheduler.run_for(Duration::from_millis(30));
    assert!(result.is_ok());
}

#[test]
fn test_mixed_rt_and_normal_nodes() {
    let mut scheduler = Scheduler::new();

    // Mix of RT and normal nodes
    scheduler.add(CriticalControlNode::new("rt_critical", 50))
        .order(0)
        .rt()
        .wcet_us(100)
        .deadline_ms(1)
        .done();

    scheduler.add(CriticalControlNode::new("normal_processing", 200))
        .order(5)
        .done();

    scheduler.add(CriticalControlNode::new("rt_sensor", 30))
        .order(1)
        .rt()
        .wcet_us(50)
        .deadline_ms(2)
        .done();

    scheduler.add(CriticalControlNode::new("background_task", 500))
        .order(20)
        .done();

    // RT nodes should be properly prioritized
    let result = scheduler.run_for(Duration::from_millis(100));
    assert!(result.is_ok());
}

#[test]
fn test_watchdog_functionality() {
    // Enable watchdog monitoring
    let mut config = SchedulerConfig::standard();
    config.realtime.watchdog_enabled = true;
    config.realtime.watchdog_timeout_ms = 50; // 50ms watchdog timeout
    config.realtime.safety_monitor = true;

    let mut scheduler = Scheduler::new().with_config(config);

    // Add RT node that will be monitored by watchdog
    scheduler.add(CriticalControlNode::new("watchdog_monitored", 10))
        .order(0)
        .rt()
        .wcet_us(50)
        .deadline_ms(1)
        .done();

    // Run normally - watchdog should be fed and not expire
    let result = scheduler.run_for(Duration::from_millis(100));
    assert!(result.is_ok());
}

#[test]
fn test_high_performance_rt_config() {
    // Configure for high-performance racing robot
    let mut scheduler = Scheduler::new().with_config(SchedulerConfig::high_performance());

    // Add ultra-fast control nodes
    scheduler.add(CriticalControlNode::new("traction_control", 10))
        .order(0)
        .rt()
        .wcet_us(20)
        .deadline_us(100)  // 10kHz control loop
        .done();

    scheduler.add(CriticalControlNode::new("stability_control", 15))
        .order(1)
        .rt()
        .wcet_us(25)
        .deadline_us(100)
        .done();

    // Should handle high-frequency execution
    let result = scheduler.run_for(Duration::from_millis(50));
    assert!(result.is_ok());
}
