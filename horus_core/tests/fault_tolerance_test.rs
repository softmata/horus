// Regression test: verify fault tolerance via FailurePolicy
//
// After removing NodeConfig restart fields and collapsing FaultConfig into
// SchedulerConfig.circuit_breaker, this test verifies:
// 1. circuit_breaker=false forces all nodes to Ignore policy
// 2. Per-node failure_policy overrides work through the builder API

use horus_core::core::Node;
use horus_core::error::HorusResult as Result;
use horus_core::scheduling::{FailurePolicy, Scheduler, SchedulerConfig};
use std::time::Duration;

mod common;
use common::cleanup_stale_shm;

/// A node that panics after a configurable number of ticks
struct PanickingNode {
    name: String,
    tick_count: usize,
    panic_at: usize,
}

impl PanickingNode {
    fn new(name: &str, panic_at: usize) -> Self {
        Self {
            name: name.to_string(),
            tick_count: 0,
            panic_at,
        }
    }
}

impl Node for PanickingNode {
    fn name(&self) -> &'static str {
        Box::leak(self.name.clone().into_boxed_str())
    }

    fn init(&mut self) -> Result<()> {
        Ok(())
    }

    fn tick(&mut self) {
        self.tick_count += 1;
        if self.tick_count == self.panic_at {
            panic!("intentional test panic at tick {}", self.tick_count);
        }
    }

    fn shutdown(&mut self) -> Result<()> {
        Ok(())
    }
}

/// A well-behaved node
struct GoodNode {
    name: String,
    tick_count: usize,
}

impl GoodNode {
    fn new(name: &str) -> Self {
        Self {
            name: name.to_string(),
            tick_count: 0,
        }
    }
}

impl Node for GoodNode {
    fn name(&self) -> &'static str {
        Box::leak(self.name.clone().into_boxed_str())
    }

    fn tick(&mut self) {
        self.tick_count += 1;
    }
}

// ============================================================
// Integration test: circuit_breaker=false forces Ignore on all nodes
// ============================================================

#[test]
fn test_circuit_breaker_false_forces_ignore() {
    cleanup_stale_shm();

    let mut config = SchedulerConfig::minimal();
    config.circuit_breaker = false;

    let mut scheduler = Scheduler::new().with_config(config);

    // Add a panicking node — without fault tolerance, Ignore policy should
    // swallow the panic and the scheduler should complete normally
    scheduler
        .add(PanickingNode::new("panic_node_ignore", 2))
        .order(0)
        .failure_policy(FailurePolicy::Fatal) // would normally stop scheduler
        .done();

    scheduler.add(GoodNode::new("good_node_1")).order(1).done();

    // With circuit_breaker=false, apply_config overrides Fatal → Ignore,
    // so the panic should be swallowed
    let result = scheduler.run_for(Duration::from_millis(200));
    assert!(
        result.is_ok(),
        "scheduler should not stop when circuit_breaker=false"
    );
}

// ============================================================
// Integration test: circuit_breaker=true with Ignore policy survives panics
// ============================================================

#[test]
fn test_circuit_breaker_true_with_ignore_survives() {
    cleanup_stale_shm();

    let mut config = SchedulerConfig::minimal();
    config.circuit_breaker = true;

    let mut scheduler = Scheduler::new().with_config(config);

    scheduler
        .add(PanickingNode::new("panic_node_ok", 2))
        .order(0)
        .failure_policy(FailurePolicy::Ignore)
        .done();

    scheduler.add(GoodNode::new("good_node_2")).order(1).done();

    let result = scheduler.run_for(Duration::from_millis(200));
    assert!(result.is_ok(), "Ignore policy should swallow panics");
}

// ============================================================
// Integration test: circuit_breaker=true with Skip policy skips failing node
// ============================================================

#[test]
fn test_circuit_breaker_true_with_skip_policy() {
    cleanup_stale_shm();

    let mut config = SchedulerConfig::minimal();
    config.circuit_breaker = true;

    let mut scheduler = Scheduler::new().with_config(config);

    // This node panics on tick 1 — Skip policy should circuit-break it
    scheduler
        .add(PanickingNode::new("panic_node_skip", 1))
        .order(0)
        .failure_policy(FailurePolicy::skip(2, 5000))
        .done();

    scheduler.add(GoodNode::new("good_node_3")).order(1).done();

    // Should complete because Skip policy suppresses the failing node
    let result = scheduler.run_for(Duration::from_millis(200));
    assert!(result.is_ok(), "Skip policy should suppress failing node");
}
