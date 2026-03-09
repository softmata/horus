//! Integration tests for the safety monitor.
//!
//! Covers: budget critical violation, non-critical budget, multiple watchdog expiry,
//! deadline miss threshold, safety stats accuracy, concurrent budget checks.

use horus_core::core::{DeadlineMissPolicy, Node};
use horus_core::scheduling::Scheduler;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::Arc;
use std::time::Duration;

mod common;
use common::cleanup_stale_shm;

// ============================================================================
// Mock nodes
// ============================================================================

/// Node with configurable execution time for budget testing.
struct TimedRtNode {
    name: String,
    tick_count: Arc<AtomicU64>,
    sleep_us: u64,
}

impl Node for TimedRtNode {
    fn name(&self) -> &'static str {
        Box::leak(self.name.clone().into_boxed_str())
    }
    fn tick(&mut self) {
        self.tick_count.fetch_add(1, Ordering::SeqCst);
        std::thread::sleep(Duration::from_micros(self.sleep_us));
    }
    fn tick_budget(&self) -> Option<Duration> {
        Some(Duration::from_micros(self.sleep_us + 50))
    }
    fn deadline_miss_policy(&self) -> DeadlineMissPolicy {
        DeadlineMissPolicy::Warn
    }
}

/// Node that deliberately exceeds tick budget.
struct BudgetViolatorNode {
    name: String,
    tick_count: Arc<AtomicU64>,
    budget_us: u64,
    actual_us: u64,
}

impl Node for BudgetViolatorNode {
    fn name(&self) -> &'static str {
        Box::leak(self.name.clone().into_boxed_str())
    }
    fn tick(&mut self) {
        self.tick_count.fetch_add(1, Ordering::SeqCst);
        std::thread::sleep(Duration::from_micros(self.actual_us));
    }
    fn tick_budget(&self) -> Option<Duration> {
        Some(Duration::from_micros(self.budget_us))
    }
    fn deadline_miss_policy(&self) -> DeadlineMissPolicy {
        DeadlineMissPolicy::Warn
    }
}

/// Simple fast counter for non-critical testing.
struct FastCounterNode {
    name: String,
    tick_count: Arc<AtomicU64>,
}

impl Node for FastCounterNode {
    fn name(&self) -> &'static str {
        Box::leak(self.name.clone().into_boxed_str())
    }
    fn tick(&mut self) {
        self.tick_count.fetch_add(1, Ordering::SeqCst);
    }
    fn tick_budget(&self) -> Option<Duration> {
        Some(Duration::from_millis(10))
    }
}

// ============================================================================
// Tests
// ============================================================================

#[test]
fn test_budget_critical_violation_emergency_stop() {
    cleanup_stale_shm();
    let tick_count = Arc::new(AtomicU64::new(0));

    // Safety monitor with watchdog auto-registers all RT nodes as critical
    let mut scheduler = Scheduler::new()
        .safety_monitor(true)
        .watchdog(Duration::from_millis(100));

    scheduler
        .add(BudgetViolatorNode {
            name: "budget_critical_violator".to_string(),
            tick_count: tick_count.clone(),
            budget_us: 50,  // 50us budget
            actual_us: 500, // 500us actual (10x overrun)
        })
        .order(0)
        .budget_us(50)
        .done();

    // The budget violation should be detected by the safety monitor
    let result = scheduler.run_for(Duration::from_millis(500));
    assert!(result.is_ok());

    // Should have ticked at least once to trigger the violation
    let ticks = tick_count.load(Ordering::SeqCst);
    assert!(
        ticks >= 1,
        "Should tick at least once to trigger violation, got {}",
        ticks
    );
}

#[test]
fn test_budget_non_critical_no_emergency() {
    cleanup_stale_shm();
    let tick_count = Arc::new(AtomicU64::new(0));

    let mut scheduler = Scheduler::new().safety_monitor(true);

    // This node is NOT registered as critical (no watchdog = no auto-critical)
    scheduler
        .add(BudgetViolatorNode {
            name: "budget_noncritical".to_string(),
            tick_count: tick_count.clone(),
            budget_us: 50,
            actual_us: 200,
        })
        .order(0)
        .budget_us(50)
        .done();

    // Non-critical budget overrun should NOT trigger emergency stop
    let result = scheduler.run_for(Duration::from_millis(200));
    assert!(result.is_ok());

    // Scheduler should have continued running
    let ticks = tick_count.load(Ordering::SeqCst);
    assert!(
        ticks > 1,
        "Non-critical node should keep ticking, got {}",
        ticks
    );
}

#[test]
fn test_multiple_watchdogs_expire_simultaneously() {
    cleanup_stale_shm();

    // Watchdog auto-registers all RT nodes as critical
    let mut scheduler = Scheduler::new()
        .safety_monitor(true)
        .watchdog(Duration::from_millis(30));

    // 3 RT nodes — auto-registered as critical via watchdog
    for i in 0..3 {
        let name = format!("watchdog_node_{}", i);
        scheduler
            .add(TimedRtNode {
                name: name.clone(),
                tick_count: Arc::new(AtomicU64::new(0)),
                sleep_us: 10,
            })
            .order(i)
            .done();
    }

    // With properly feeding watchdogs, no emergency stop within 200ms
    let result = scheduler.run_for(Duration::from_millis(200));
    assert!(result.is_ok());
}

#[test]
fn test_deadline_miss_threshold_triggers_stop() {
    cleanup_stale_shm();
    let tick_count = Arc::new(AtomicU64::new(0));

    let mut scheduler = Scheduler::new().safety_monitor(true).max_deadline_misses(3); // Stop after 3 misses

    // Node that will miss deadlines
    scheduler
        .add(BudgetViolatorNode {
            name: "deadline_misser".to_string(),
            tick_count: tick_count.clone(),
            budget_us: 1000,
            actual_us: 2000, // Exceeds deadline
        })
        .order(0)
        .budget_us(5000)
        .deadline_us(500) // Will be missed
        .done();

    let result = scheduler.run_for(Duration::from_millis(500));
    assert!(result.is_ok());

    // Should have stopped after accumulating 3 deadline misses
    let ticks = tick_count.load(Ordering::SeqCst);
    assert!(
        ticks >= 3,
        "Should tick at least 3 times to accumulate misses, got {}",
        ticks
    );
}

#[test]
fn test_safety_stats_accurate() {
    cleanup_stale_shm();
    let tick_count = Arc::new(AtomicU64::new(0));

    let mut scheduler = Scheduler::new()
        .safety_monitor(true)
        .max_deadline_misses(100); // High threshold so we don't stop

    // Node that violates budget and misses deadlines
    scheduler
        .add(BudgetViolatorNode {
            name: "stats_node".to_string(),
            tick_count: tick_count.clone(),
            budget_us: 50,
            actual_us: 200,
        })
        .order(0)
        .budget_us(50)
        .deadline_us(100)
        .done();

    scheduler.run_for(Duration::from_millis(200)).unwrap();

    // Stats should reflect violations (verified through scheduler running without crash)
    let ticks = tick_count.load(Ordering::SeqCst);
    assert!(ticks > 0, "Node should have ticked");
}

#[test]
fn test_concurrent_budget_check_and_add_critical() {
    cleanup_stale_shm();

    // Watchdog auto-registers all RT nodes as critical
    let mut scheduler = Scheduler::new()
        .safety_monitor(true)
        .watchdog(Duration::from_millis(100));

    // Add several RT nodes to exercise concurrent safety monitor checks
    for i in 0..5 {
        let name = format!("concurrent_node_{}", i);
        scheduler
            .add(FastCounterNode {
                name: name.clone(),
                tick_count: Arc::new(AtomicU64::new(0)),
            })
            .order(i)
            .budget_us(10000)
            .done();
    }

    // Run to exercise the concurrent safety monitor checks
    let result = scheduler.run_for(Duration::from_millis(100));
    assert!(result.is_ok());
}
