//! Integration tests for Scheduler DX (Developer Experience) API

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

// test_new_with_capacity removed: with_capacity() was removed from Scheduler

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
    let scheduler = Scheduler::new().with_blackbox(16);

    assert!(
        scheduler.blackbox().is_some(),
        ".with_blackbox() should create BlackBox"
    );
}

#[test]
fn test_blackbox_accessor_record() {
    cleanup_stale_shm();
    use horus_core::scheduling::BlackBoxEvent;

    let scheduler = Scheduler::new().with_blackbox(16);

    let bb = scheduler.blackbox().expect("BlackBox should exist");
    bb.lock().unwrap().record(BlackBoxEvent::Custom {
        category: "test".to_string(),
        message: "Integration test event".to_string(),
    });
}

#[test]
fn test_status_shows_blackbox() {
    cleanup_stale_shm();
    let scheduler = Scheduler::new().with_blackbox(16);
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

// test_circuit_summary_empty_scheduler removed: circuit_summary() was removed from Scheduler
// test_failure_stats_nonexistent_node removed: failure_stats() and circuit_state() were removed from Scheduler

// =============================================================================
// budget Enforcement Tests
// =============================================================================

#[test]
fn test_status_shows_budget_enforcement() {
    cleanup_stale_shm();
    let scheduler = Scheduler::new();
    let status = scheduler.status();

    assert!(status.contains("budget Enforcement"));
}
