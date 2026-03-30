#![allow(dead_code)]
//! Level 7 Intent Tests — Watchdog Behavior
//!
//! These tests verify **behavioral intent** of the watchdog and safety
//! monitoring integrated into the scheduler, not implementation details.
//! Each test documents WHY a behavior matters and what user-visible
//! guarantee it protects.
//!
//! Covers:
//! - Healthy nodes produce no watchdog warnings (no false positives)
//! - Slow nodes that exceed their budget are detected

use horus_core::core::{DurationExt, Node};
use horus_core::scheduling::Scheduler;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::Arc;

mod common;
use common::cleanup_stale_shm;

// ============================================================================
// Test 4: Healthy nodes produce no watchdog warnings
// ============================================================================

/// INTENT: "Nodes completing on time produce no watchdog warnings."
///
/// Robotics guarantee: a robot with three well-behaved sensor nodes
/// (camera, lidar, IMU) running within their time budgets must never
/// trigger a watchdog warning or safety degradation. False-positive
/// warnings would cause the scheduler to skip perfectly functional
/// nodes, leading to sensor data loss and degraded autonomy.
#[test]
fn test_watchdog_intent_healthy_no_warnings() {
    cleanup_stale_shm();

    let count_a = Arc::new(AtomicU64::new(0));
    let count_b = Arc::new(AtomicU64::new(0));
    let count_c = Arc::new(AtomicU64::new(0));

    struct FastNode {
        name: &'static str,
        count: Arc<AtomicU64>,
    }

    impl Node for FastNode {
        fn name(&self) -> &str {
            self.name
        }
        fn tick(&mut self) {
            // Minimal work — well within any reasonable budget
            self.count.fetch_add(1, Ordering::SeqCst);
            std::hint::black_box(42);
        }
    }

    let mut scheduler = Scheduler::new()
        .tick_rate(100_u64.hz())
        .max_deadline_misses(1000);

    scheduler
        .add(FastNode {
            name: "camera_fast",
            count: count_a.clone(),
        })
        .order(0)
        .build()
        .unwrap();

    scheduler
        .add(FastNode {
            name: "lidar_fast",
            count: count_b.clone(),
        })
        .order(1)
        .build()
        .unwrap();

    scheduler
        .add(FastNode {
            name: "imu_fast",
            count: count_c.clone(),
        })
        .order(2)
        .build()
        .unwrap();

    // Run for 200ms — enough for ~20 ticks at 100Hz
    scheduler.run_for(200_u64.ms()).unwrap();

    // All three nodes should have ticked
    let a = count_a.load(Ordering::SeqCst);
    let b = count_b.load(Ordering::SeqCst);
    let c = count_c.load(Ordering::SeqCst);

    assert!(a > 0, "camera_fast should have ticked, got 0");
    assert!(b > 0, "lidar_fast should have ticked, got 0");
    assert!(c > 0, "imu_fast should have ticked, got 0");

    // Safety state must remain Normal — no false positive warnings
    if let Some(stats) = scheduler.safety_stats() {
        assert_eq!(
            *stats.state(),
            horus_core::scheduling::SafetyState::Normal,
            "Safety state should be Normal for healthy nodes, got {:?}",
            stats.state()
        );
        assert_eq!(
            stats.watchdog_expirations(),
            0,
            "No watchdog expirations expected for healthy nodes"
        );
    }
}

// ============================================================================
// Test 5: Slow node detected by watchdog
// ============================================================================

/// INTENT: "A node that exceeds its budget is detected by the watchdog."
///
/// Robotics guarantee: if a path planner node that declared a 1ms budget
/// suddenly starts taking 50ms per tick (e.g., due to a pathological
/// input), the safety monitor must detect this as deadline misses. Failing
/// to detect it means the slow node silently steals time from other
/// nodes, causing cascading deadline misses across the entire system.
///
/// Budget violations on RT nodes are tracked per-node and surfaced as
/// deadline misses in safety_stats (since budget auto-derives deadline).
#[test]
fn test_watchdog_intent_slow_node_detected() {
    cleanup_stale_shm();

    let fast_count = Arc::new(AtomicU64::new(0));

    struct FastNode {
        count: Arc<AtomicU64>,
    }

    impl Node for FastNode {
        fn name(&self) -> &str {
            "fast_node"
        }
        fn tick(&mut self) {
            self.count.fetch_add(1, Ordering::SeqCst);
        }
    }

    struct SlowNode;

    impl Node for SlowNode {
        fn name(&self) -> &str {
            "slow_node"
        }
        fn tick(&mut self) {
            // Sleep 50ms — far exceeding the 1ms budget/deadline
            std::thread::sleep(std::time::Duration::from_millis(50));
        }
    }

    let mut scheduler = Scheduler::new()
        .tick_rate(50_u64.hz())
        .max_deadline_misses(1000);

    scheduler
        .add(FastNode {
            count: fast_count.clone(),
        })
        .order(0)
        .build()
        .unwrap();

    // Setting .budget(1ms) auto-derives .deadline(1ms) for RT nodes.
    // The node sleeping 50ms will miss both budget and deadline every tick.
    scheduler
        .add(SlowNode)
        .order(1)
        .budget(1_u64.ms())
        .build()
        .unwrap();

    // Run for 500ms — the slow node will blow its budget/deadline on every tick
    scheduler.run_for(500_u64.ms()).unwrap();

    // The fast node should still have ticked (scheduler doesn't stop on
    // budget violations alone — max_deadline_misses is set high)
    let fast_ticks = fast_count.load(Ordering::SeqCst);
    assert!(
        fast_ticks > 0,
        "Fast node should still tick despite sibling's budget violations, got 0"
    );

    // RT nodes run on a dedicated thread; their budget/deadline violations
    // are tracked in per-node RtStats (not the global SafetyMonitor).
    // Check the per-node stats to verify the slow node was detected.
    if let Some(rt_stats) = scheduler.rt_stats("slow_node") {
        let has_violations = rt_stats.budget_violations() > 0 || rt_stats.deadline_misses() > 0;
        assert!(
            has_violations,
            "RT stats should show violations for a node sleeping 50ms with 1ms budget. \
             budget_violations={}, deadline_misses={}",
            rt_stats.budget_violations(),
            rt_stats.deadline_misses()
        );
    } else {
        // If rt_stats is not available, fall back to safety_stats
        if let Some(stats) = scheduler.safety_stats() {
            let has_degradation = stats.budget_overruns() > 0
                || stats.deadline_misses() > 0
                || stats.degrade_activations() > 0;
            assert!(
                has_degradation,
                "Safety stats should show degradation for a node sleeping 50ms with 1ms budget. \
                 budget_overruns={}, deadline_misses={}, degrade_activations={}",
                stats.budget_overruns(),
                stats.deadline_misses(),
                stats.degrade_activations()
            );
        }
    }
}
