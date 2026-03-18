//! Integration tests for Scheduler DX (Developer Experience) API

use horus_core::core::Node;
use horus_core::error::Result;
use horus_core::scheduling::Scheduler;
use std::sync::atomic::{AtomicU32, Ordering};
use std::sync::Arc;

mod common;
use common::cleanup_stale_shm;
use horus_core::core::DurationExt;

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
        .build();

    let result = scheduler.run_for(500_u64.ms());
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
    assert_eq!(
        degradations.len(),
        0,
        "A fresh scheduler should have no degradations, got: {:?}",
        degradations
    );
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
        scheduler.get_blackbox().is_none(),
        "new() should NOT auto-create BlackBox (opt-in via .blackbox(size_mb))"
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
        status.contains("Error Handling"),
        "status() should mention Error Handling: {}",
        status
    );
    assert!(
        status.contains("[x] Error Handling"),
        "status() should show Error Handling as enabled: {}",
        status
    );
}

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
