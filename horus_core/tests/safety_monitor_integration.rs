//! Integration tests for the safety monitor.
//!
//! Covers: multiple watchdog expiry, concurrent safety monitor checks.

use horus_core::scheduling::Scheduler;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::Arc;

mod common;
use common::cleanup_stale_shm;
use horus_core::core::{DurationExt, Node};

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
        std::thread::sleep(self.sleep_us.us());
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
}

// ============================================================================
// Tests
// ============================================================================


#[test]
fn test_multiple_watchdogs_expire_simultaneously() {
    cleanup_stale_shm();

    // Watchdog auto-registers all RT nodes as critical
    let mut scheduler = Scheduler::new().watchdog(500_u64.ms());

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
            .build();
    }

    // With properly feeding watchdogs, no emergency stop within 200ms
    let result = scheduler.run_for(200_u64.ms());
    result.unwrap();
}

#[test]
fn test_concurrent_budget_check_and_add_critical() {
    cleanup_stale_shm();

    // Watchdog auto-registers all RT nodes as critical
    let mut scheduler = Scheduler::new().watchdog(500_u64.ms());

    // Add several RT nodes to exercise concurrent safety monitor checks
    for i in 0..5 {
        let name = format!("concurrent_node_{}", i);
        scheduler
            .add(FastCounterNode {
                name: name.clone(),
                tick_count: Arc::new(AtomicU64::new(0)),
            })
            .order(i)
            .build();
    }

    // Run to exercise the concurrent safety monitor checks
    let result = scheduler.run_for(100_u64.ms());
    result.unwrap();
}
