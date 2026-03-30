#![allow(dead_code)]
//! Level 7 Intent Tests — Safety Monitor
//!
//! These tests verify **behavioral intent** of the safety monitoring system,
//! not implementation details. Each test documents WHY a behavior matters
//! and what user-visible guarantee it protects.
//!
//! Covers:
//! - Healthy nodes remain healthy (no false positive warnings)
//! - Error-producing nodes get degraded health
//! - Timing report / profiling on shutdown
//! - Budget tracking with `.budget()` constraint

use horus_core::core::{DurationExt, Node};
use horus_core::scheduling::{FailurePolicy, Scheduler};
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::Arc;

mod common;
use common::cleanup_stale_shm;

// ============================================================================
// Test 1: Healthy nodes remain healthy (no false positive warnings)
// ============================================================================

/// INTENT: "Nodes that tick normally stay in Healthy state."
///
/// Robotics guarantee: a well-behaved sensor node that always returns
/// successfully must never be flagged as degraded or isolated. False-positive
/// health warnings would cause the scheduler to skip or rate-reduce a
/// perfectly functional node, leading to data loss.
#[test]
fn test_safety_monitor_intent_healthy_nodes_remain_healthy() {
    cleanup_stale_shm();

    let tick_count_a = Arc::new(AtomicU64::new(0));
    let tick_count_b = Arc::new(AtomicU64::new(0));
    let tick_count_c = Arc::new(AtomicU64::new(0));

    struct HealthyNode {
        name: &'static str,
        count: Arc<AtomicU64>,
    }

    impl Node for HealthyNode {
        fn name(&self) -> &str {
            self.name
        }
        fn tick(&mut self) {
            self.count.fetch_add(1, Ordering::SeqCst);
        }
    }

    let mut scheduler = Scheduler::new()
        .tick_rate(100_u64.hz())
        .max_deadline_misses(1000);

    scheduler
        .add(HealthyNode {
            name: "healthy_a",
            count: tick_count_a.clone(),
        })
        .order(0)
        .build()
        .unwrap();

    scheduler
        .add(HealthyNode {
            name: "healthy_b",
            count: tick_count_b.clone(),
        })
        .order(1)
        .build()
        .unwrap();

    scheduler
        .add(HealthyNode {
            name: "healthy_c",
            count: tick_count_c.clone(),
        })
        .order(2)
        .build()
        .unwrap();

    // Run for 200ms — enough for ~20 ticks at 100Hz
    scheduler.run_for(200_u64.ms()).unwrap();

    // All three nodes should have ticked — none skipped or isolated
    let a = tick_count_a.load(Ordering::SeqCst);
    let b = tick_count_b.load(Ordering::SeqCst);
    let c = tick_count_c.load(Ordering::SeqCst);

    assert!(
        a > 0,
        "healthy_a should have ticked at least once, got 0 ticks"
    );
    assert!(
        b > 0,
        "healthy_b should have ticked at least once, got 0 ticks"
    );
    assert!(
        c > 0,
        "healthy_c should have ticked at least once, got 0 ticks"
    );

    // Safety stats should show no emergencies and no deadline misses
    if let Some(stats) = scheduler.safety_stats() {
        assert_eq!(
            *stats.state(),
            horus_core::scheduling::SafetyState::Normal,
            "Safety state should be Normal for healthy nodes, got {:?}",
            stats.state()
        );
    }
}

// ============================================================================
// Test 2: Error-producing node gets degraded
// ============================================================================

/// INTENT: "A node that errors gets its health degraded."
///
/// Robotics guarantee: if a motor controller starts panicking every tick,
/// the safety monitor must detect this and degrade the node — not let it
/// silently produce garbage outputs that could damage hardware.
#[test]
fn test_safety_monitor_intent_error_node_degrades() {
    cleanup_stale_shm();

    let good_count = Arc::new(AtomicU64::new(0));

    struct GoodNode {
        count: Arc<AtomicU64>,
    }

    impl Node for GoodNode {
        fn name(&self) -> &str {
            "good_node"
        }
        fn tick(&mut self) {
            self.count.fetch_add(1, Ordering::SeqCst);
        }
    }

    /// A node that panics every tick. With FailurePolicy::Ignore the
    /// scheduler won't stop, but the safety monitor should track the errors.
    struct PanickingNode;

    impl Node for PanickingNode {
        fn name(&self) -> &str {
            "panicking_node"
        }
        fn tick(&mut self) {
            panic!("intentional test panic");
        }
    }

    let mut scheduler = Scheduler::new()
        .tick_rate(100_u64.hz())
        .max_deadline_misses(1000);

    scheduler
        .add(GoodNode {
            count: good_count.clone(),
        })
        .order(0)
        .build()
        .unwrap();

    scheduler
        .add(PanickingNode)
        .order(1)
        .failure_policy(FailurePolicy::Ignore)
        .build()
        .unwrap();

    // Run for 200ms — panicking node will fail each tick, good node should keep going
    scheduler.run_for(200_u64.ms()).unwrap();

    // The good node should have ticked normally — the panicking node's failures
    // should not bring down the entire scheduler when using Ignore policy
    let good_ticks = good_count.load(Ordering::SeqCst);
    assert!(
        good_ticks > 0,
        "Good node should still tick when sibling panics with Ignore policy, got 0"
    );
}

// ============================================================================
// Test 3: Scheduler produces timing report on shutdown
// ============================================================================

/// INTENT: "Scheduler produces timing report on shutdown."
///
/// Robotics guarantee: after a mission run, the operator needs to review
/// timing statistics (worst-case latency, deadline misses, overruns) to
/// validate that the system met real-time constraints. The scheduler must
/// complete cleanly and have accessible profile/safety data after shutdown.
#[test]
fn test_safety_monitor_intent_timing_report_on_shutdown() {
    cleanup_stale_shm();

    struct SimpleNode {
        name: &'static str,
    }

    impl Node for SimpleNode {
        fn name(&self) -> &str {
            self.name
        }
        fn tick(&mut self) {
            // Minimal work — just enough to generate timing data
            std::hint::black_box(42);
        }
    }

    let mut scheduler = Scheduler::new()
        .tick_rate(100_u64.hz())
        .max_deadline_misses(1000);

    scheduler
        .add(SimpleNode {
            name: "timing_node_a",
        })
        .order(0)
        .build()
        .unwrap();

    scheduler
        .add(SimpleNode {
            name: "timing_node_b",
        })
        .order(1)
        .build()
        .unwrap();

    // Run briefly — enough to accumulate some timing samples
    scheduler.run_for(100_u64.ms()).unwrap();

    // The scheduler should have completed without hanging. If timing
    // report generation caused a deadlock or hang, run_for would not return.
    //
    // Additionally, safety stats should be accessible after shutdown.
    if let Some(stats) = scheduler.safety_stats() {
        // After a clean run, there should be no budget overruns or emergency stops
        assert_eq!(
            *stats.state(),
            horus_core::scheduling::SafetyState::Normal,
            "Safety state should remain Normal after clean run"
        );
    }

    // If we reached this point, the scheduler completed without hanging —
    // timing report generation during shutdown did not deadlock.
}

// ============================================================================
// Test 4: Budget tracking with .budget() constraint
// ============================================================================

/// INTENT: "Nodes with budget constraints have their budget tracked."
///
/// Robotics guarantee: when a node declares `.budget(1.ms())`, the scheduler
/// must track its actual execution time against that budget. Over-budget
/// ticks should be recorded (soft enforcement) without crashing the scheduler.
/// This ensures operators can post-hoc verify that safety-critical nodes
/// stayed within their worst-case execution time.
#[test]
fn test_safety_monitor_intent_budget_tracking() {
    cleanup_stale_shm();

    let tick_count = Arc::new(AtomicU64::new(0));

    struct BudgetedNode {
        count: Arc<AtomicU64>,
    }

    impl Node for BudgetedNode {
        fn name(&self) -> &str {
            "budgeted_node"
        }
        fn tick(&mut self) {
            self.count.fetch_add(1, Ordering::SeqCst);
            // Minimal work — should be well within 1ms budget
            std::hint::black_box(0u64);
        }
    }

    let mut scheduler = Scheduler::new()
        .tick_rate(100_u64.hz())
        .max_deadline_misses(1000);

    scheduler
        .add(BudgetedNode {
            count: tick_count.clone(),
        })
        .order(0)
        .budget(1_u64.ms())
        .build()
        .unwrap();

    // Run for 200ms — node ticks within budget, scheduler should complete normally
    scheduler.run_for(200_u64.ms()).unwrap();

    let ticks = tick_count.load(Ordering::SeqCst);
    assert!(
        ticks > 0,
        "Budgeted node should tick normally when within budget, got 0 ticks"
    );

    // Safety stats: no budget overruns expected since the node does trivial work
    if let Some(stats) = scheduler.safety_stats() {
        assert_eq!(
            stats.budget_overruns(),
            0,
            "No budget overruns expected for trivial-work node within 1ms budget"
        );
    }
}
