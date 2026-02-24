//! Integration tests for Scheduler DX (Developer Experience) API
//!
//! Tests the preset constructors:
//! - `Scheduler::safety_critical()`
//! - `Scheduler::high_performance()`
//! - `Scheduler::deterministic()`
//! - `Scheduler::hard_realtime()`

use horus_core::core::Node;
use horus_core::error::HorusResult as Result;
use horus_core::scheduling::Scheduler;
use std::sync::atomic::{AtomicU32, Ordering};
use std::sync::Arc;
use std::time::Duration;

mod common;
use common::cleanup_stale_shm;

/// Simple test node that counts ticks
struct TickCounterNode {
    name: &'static str,
    tick_count: Arc<AtomicU32>,
}

impl TickCounterNode {
    fn new(name: &'static str, counter: Arc<AtomicU32>) -> Self {
        Self {
            name,
            tick_count: counter,
        }
    }
}

impl Node for TickCounterNode {
    fn name(&self) -> &'static str {
        self.name
    }

    fn init(&mut self) -> Result<()> {
        Ok(())
    }

    fn tick(&mut self) {
        self.tick_count.fetch_add(1, Ordering::SeqCst);
    }

    fn shutdown(&mut self) -> Result<()> {
        Ok(())
    }
}

// =============================================================================
// Scheduler::new() Tests - Auto-Optimization
// =============================================================================

#[test]
fn test_new_creates_scheduler() {
    cleanup_stale_shm();
    let mut scheduler = Scheduler::new();
    let counter = Arc::new(AtomicU32::new(0));

    scheduler
        .add(TickCounterNode::new("test_node", counter.clone()))
        .order(0)
        .done();

    let result = scheduler.run_for(Duration::from_millis(500));
    assert!(
        result.is_ok(),
        "Scheduler::new() should create a working scheduler"
    );
    assert!(
        counter.load(Ordering::SeqCst) > 0,
        "Node should have ticked"
    );
}

#[test]
fn test_new_detects_capabilities() {
    cleanup_stale_shm();
    let scheduler = Scheduler::new();

    let caps = scheduler.capabilities();
    assert!(
        caps.is_some(),
        "Scheduler::new() should detect capabilities"
    );

    let caps = caps.unwrap();
    assert!(caps.cpu_count > 0, "Should detect at least 1 CPU core");
}

#[test]
fn test_new_records_degradations_gracefully() {
    cleanup_stale_shm();
    let scheduler = Scheduler::new();
    let degradations = scheduler.degradations();
    let _ = degradations.len();
}

#[test]
fn test_new_with_capacity() {
    cleanup_stale_shm();
    let mut scheduler = Scheduler::new().with_capacity(100);
    let counter = Arc::new(AtomicU32::new(0));

    scheduler
        .add(TickCounterNode::new("cap_node", counter.clone()))
        .order(0)
        .done();

    let result = scheduler.run_for(Duration::from_millis(100));
    assert!(result.is_ok());
}

// =============================================================================
// Preset Constructor Tests
// =============================================================================

#[test]
fn test_safety_critical_creates_scheduler() {
    cleanup_stale_shm();
    let mut scheduler = Scheduler::safety_critical();
    let counter = Arc::new(AtomicU32::new(0));

    scheduler
        .add(TickCounterNode::new("safety_node", counter.clone()))
        .order(0)
        .done();

    let result = scheduler.run_for(Duration::from_millis(100));
    assert!(result.is_ok());
}

#[test]
fn test_high_performance_creates_scheduler() {
    cleanup_stale_shm();
    let mut scheduler = Scheduler::high_performance();
    let counter = Arc::new(AtomicU32::new(0));

    scheduler
        .add(TickCounterNode::new("perf_node", counter.clone()))
        .order(0)
        .done();

    let result = scheduler.run_for(Duration::from_millis(100));
    assert!(result.is_ok());
}

#[test]
fn test_deterministic_creates_scheduler() {
    cleanup_stale_shm();
    let mut scheduler = Scheduler::deterministic();
    let counter = Arc::new(AtomicU32::new(0));

    scheduler
        .add(TickCounterNode::new("det_node", counter.clone()))
        .order(0)
        .done();

    let result = scheduler.run_for(Duration::from_millis(100));
    assert!(result.is_ok());
}

#[test]
fn test_hard_realtime_creates_scheduler() {
    cleanup_stale_shm();
    let mut scheduler = Scheduler::hard_realtime();
    let counter = Arc::new(AtomicU32::new(0));

    scheduler
        .add(TickCounterNode::new("hrt_node", counter.clone()))
        .order(0)
        .done();

    let result = scheduler.run_for(Duration::from_millis(100));
    assert!(result.is_ok());
}

// =============================================================================
// Status Introspection Tests
// =============================================================================

#[test]
fn test_status_returns_formatted_string() {
    cleanup_stale_shm();
    let scheduler = Scheduler::new();
    let status = scheduler.status();

    assert!(
        status.contains("SCHEDULER STATUS:"),
        "status() should have header"
    );
    assert!(
        status.contains("Execution Mode:"),
        "status() should show execution mode"
    );
    assert!(
        status.contains("Safety Features:"),
        "status() should show safety features"
    );
}

#[test]
fn test_status_shows_capabilities() {
    cleanup_stale_shm();
    let scheduler = Scheduler::new();
    let status = scheduler.status();

    assert!(
        status.contains("Platform:"),
        "status() should show platform for new()"
    );
    assert!(
        status.contains("RT Features:"),
        "status() should show RT features"
    );
}

// =============================================================================
// BlackBox Flight Recorder Tests
// =============================================================================

#[test]
fn test_new_no_blackbox_by_default() {
    cleanup_stale_shm();
    let scheduler = Scheduler::new();

    assert!(
        scheduler.blackbox().is_none(),
        "new() should NOT auto-create BlackBox (opt-in via config.monitoring.black_box_enabled)"
    );
}

#[test]
fn test_with_blackbox_creates_blackbox() {
    cleanup_stale_shm();
    let mut config = horus_core::scheduling::SchedulerConfig::minimal();
    config.monitoring.black_box_enabled = true;
    config.monitoring.black_box_size_mb = 16;
    let mut scheduler = Scheduler::new();
    scheduler.apply_config(config);

    assert!(
        scheduler.blackbox().is_some(),
        "config.monitoring.black_box_enabled should create BlackBox"
    );
}

#[test]
fn test_blackbox_mut_accessor() {
    cleanup_stale_shm();
    use horus_core::scheduling::BlackBoxEvent;

    let mut config = horus_core::scheduling::SchedulerConfig::minimal();
    config.monitoring.black_box_enabled = true;
    config.monitoring.black_box_size_mb = 16;
    let mut scheduler = Scheduler::new();
    scheduler.apply_config(config);

    let bb = scheduler.blackbox_mut().expect("BlackBox should exist");
    bb.record(BlackBoxEvent::Custom {
        category: "test".to_string(),
        message: "Integration test event".to_string(),
    });
}

#[test]
fn test_status_shows_blackbox() {
    cleanup_stale_shm();
    let mut config = horus_core::scheduling::SchedulerConfig::minimal();
    config.monitoring.black_box_enabled = true;
    config.monitoring.black_box_size_mb = 16;
    let mut scheduler = Scheduler::new();
    scheduler.apply_config(config);
    let status = scheduler.status();

    assert!(
        status.contains("BlackBox"),
        "status() should mention BlackBox"
    );
    assert!(
        status.contains("[x] BlackBox"),
        "status() should show BlackBox as enabled: {}",
        status
    );
}

#[test]
fn test_status_no_blackbox_by_default() {
    cleanup_stale_shm();
    let scheduler = Scheduler::new();
    let status = scheduler.status();

    assert!(
        status.contains("[ ] BlackBox"),
        "status() should show BlackBox as disabled by default: {}",
        status
    );
}

// =============================================================================
// Failure Policy Integration Tests
// =============================================================================

#[test]
fn test_failure_policies_enabled_by_default() {
    cleanup_stale_shm();
    let scheduler = Scheduler::new();
    let status = scheduler.status();

    assert!(
        status.contains("Failure Policies"),
        "status() should mention Failure Policies: {}",
        status
    );
    assert!(
        status.contains("[x] Failure Policies"),
        "status() should show Failure Policies as enabled: {}",
        status
    );
}

#[test]
fn test_circuit_summary_empty_scheduler() {
    cleanup_stale_shm();
    let scheduler = Scheduler::new();

    let (healthy, suppressed, recovering) = scheduler.circuit_summary();
    assert_eq!(healthy, 0, "No nodes should mean 0 healthy");
    assert_eq!(suppressed, 0, "No nodes should mean 0 suppressed");
    assert_eq!(recovering, 0, "No nodes should mean 0 recovering");
}

#[test]
fn test_failure_stats_nonexistent_node() {
    cleanup_stale_shm();
    let scheduler = Scheduler::new();

    assert!(
        scheduler.failure_stats("nonexistent_node").is_none(),
        "Should return None for nonexistent node"
    );
    assert!(
        scheduler.circuit_state("nonexistent_node").is_none(),
        "Should return None for nonexistent node (compat API)"
    );
}

// =============================================================================
// WCET Enforcement Tests
// =============================================================================

#[test]
fn test_status_shows_wcet_enforcement() {
    cleanup_stale_shm();
    let scheduler = Scheduler::new();
    let status = scheduler.status();

    assert!(status.contains("WCET Enforcement"));
}
