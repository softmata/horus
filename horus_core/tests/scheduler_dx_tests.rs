//! Integration tests for Scheduler DX (Developer Experience) API
//!
//! Tests the intent-based constructors:
//! - `Scheduler::new()` - Auto-optimizing with capability detection
//! - `Scheduler::safety_critical()` - Safety-critical preset
//! - `Scheduler::high_performance()` - High-performance preset
//! - `Scheduler::deterministic()` - Deterministic preset
//! - `Scheduler::hard_realtime()` - Hard real-time preset

use horus_core::core::Node;
use horus_core::error::Result;
use horus_core::scheduling::Scheduler;
use std::sync::atomic::{AtomicU32, Ordering};
use std::sync::Arc;
use std::time::Duration;

/// Clean up stale shared memory from previous test runs to prevent SIGSEGV.
/// The discovery topic persists in /dev/shm and can have incompatible layouts
/// across different test binaries.
fn cleanup_stale_shm() {
    static ONCE: std::sync::Once = std::sync::Once::new();
    ONCE.call_once(|| {
        let _ = std::fs::remove_dir_all("/dev/shm/horus/topics");
        let _ = std::fs::remove_dir_all("/dev/shm/horus/nodes");
    });
}

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

    let result = scheduler.run_for(Duration::from_millis(100));
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
fn test_new_auto_creates_blackbox() {
    cleanup_stale_shm();
    let scheduler = Scheduler::new();

    assert!(
        scheduler.blackbox().is_some(),
        "new() should auto-create BlackBox for crash analysis"
    );
}

#[test]
fn test_blackbox_mut_accessor() {
    cleanup_stale_shm();
    use horus_core::scheduling::BlackBoxEvent;

    let mut scheduler = Scheduler::new();

    let bb = scheduler.blackbox_mut().expect("BlackBox should exist");
    bb.record(BlackBoxEvent::Custom {
        category: "test".to_string(),
        message: "Integration test event".to_string(),
    });
}

#[test]
fn test_status_shows_blackbox() {
    cleanup_stale_shm();
    let scheduler = Scheduler::new();
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

// =============================================================================
// Circuit Breaker Integration Tests
// =============================================================================

#[test]
fn test_circuit_breakers_enabled_by_default() {
    cleanup_stale_shm();
    let scheduler = Scheduler::new();
    let status = scheduler.status();

    assert!(
        status.contains("Circuit Breakers"),
        "status() should mention Circuit Breakers"
    );
    assert!(
        status.contains("[x] Circuit Breakers"),
        "status() should show Circuit Breakers as enabled"
    );
}

#[test]
fn test_circuit_summary_empty_scheduler() {
    cleanup_stale_shm();
    let scheduler = Scheduler::new();

    let (closed, open, half_open) = scheduler.circuit_summary();
    assert_eq!(closed, 0, "No nodes should mean 0 closed circuits");
    assert_eq!(open, 0, "No nodes should mean 0 open circuits");
    assert_eq!(half_open, 0, "No nodes should mean 0 half-open circuits");
}

#[test]
fn test_circuit_state_nonexistent_node() {
    cleanup_stale_shm();
    let scheduler = Scheduler::new();

    assert!(
        scheduler.circuit_state("nonexistent_node").is_none(),
        "Should return None for nonexistent node"
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
