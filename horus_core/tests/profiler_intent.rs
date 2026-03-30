#![allow(dead_code)]
//! Level 7 Intent Tests — Profiler / Metrics
//!
//! These tests verify **behavioral intent** of the runtime profiler
//! and node metrics, not implementation details. Each test documents
//! WHY a behavior matters and what user-visible guarantee it protects.
//!
//! Uses `Scheduler::run_for()` + `Scheduler::metrics()` — the public
//! timing-data API available to external consumers.
//!
//! Covers:
//! - Metrics collect timing data for each node after running
//! - Per-node timing statistics maintain correct ordering invariant
//! - Idle (no-op) nodes have near-zero tick durations

use horus_core::core::{DurationExt, Node};
use horus_core::scheduling::Scheduler;

mod common;
use common::cleanup_stale_shm;

// ============================================================================
// Test 1: After running nodes, profiler has timing data for each node
// ============================================================================

/// INTENT: "After running nodes, profiler has timing data for each node."
///
/// Robotics guarantee: operators need per-node timing to diagnose which node
/// is the bottleneck. If the profiler silently drops nodes, the operator
/// has a blind spot and cannot tune the system.
#[test]
fn test_profiler_intent_collects_timing_data() {
    cleanup_stale_shm();

    struct TimedNode {
        name: &'static str,
    }

    impl Node for TimedNode {
        fn name(&self) -> &str {
            self.name
        }
        fn tick(&mut self) {
            // Minimal work so the node registers timing samples
            std::hint::black_box(42);
        }
    }

    let mut scheduler = Scheduler::new().tick_rate(100_u64.hz());

    scheduler
        .add(TimedNode { name: "sensor" })
        .order(0)
        .build()
        .unwrap();

    scheduler
        .add(TimedNode { name: "planner" })
        .order(1)
        .build()
        .unwrap();

    // Run for 200ms — enough for multiple ticks at 100Hz
    scheduler.run_for(200_u64.ms()).unwrap();

    let metrics = scheduler.metrics();

    // Both nodes should appear in metrics
    assert!(
        metrics.len() >= 2,
        "Expected at least 2 node metrics entries, got {}",
        metrics.len()
    );

    let node_names: Vec<&str> = metrics.iter().map(|m| m.name()).collect();
    assert!(
        node_names.contains(&"sensor"),
        "Metrics should contain 'sensor' node, got {:?}",
        node_names
    );
    assert!(
        node_names.contains(&"planner"),
        "Metrics should contain 'planner' node, got {:?}",
        node_names
    );

    // Each node should have been ticked at least once
    for m in &metrics {
        assert!(
            m.total_ticks() > 0,
            "Node '{}' should have at least 1 tick, got {}",
            m.name(),
            m.total_ticks()
        );
        // avg_tick_duration should be non-negative (nodes did run)
        assert!(
            m.avg_tick_duration_ms() >= 0.0,
            "Node '{}' avg_tick_duration_ms should be >= 0, got {}",
            m.name(),
            m.avg_tick_duration_ms()
        );
    }
}

// ============================================================================
// Test 2: Profiler statistics are ordered: min <= avg <= max
// ============================================================================

/// INTENT: "Profiler percentiles are ordered: min <= avg <= max."
///
/// Robotics guarantee: if statistics are misordered, CI gates that assert
/// `max < 1ms` could silently pass even when the system is too slow. The
/// ordering invariant ensures the numbers are trustworthy for performance
/// gating. (min <= average <= max is a mathematical invariant that must hold.)
#[test]
fn test_profiler_intent_percentiles_ordered() {
    cleanup_stale_shm();

    struct WorkNode {
        name: &'static str,
    }

    impl Node for WorkNode {
        fn name(&self) -> &str {
            self.name
        }
        fn tick(&mut self) {
            // Do a small amount of variable work so timings differ
            let mut sum = 0u64;
            for i in 0..100 {
                sum = sum.wrapping_add(std::hint::black_box(i));
            }
            std::hint::black_box(sum);
        }
    }

    let mut scheduler = Scheduler::new().tick_rate(200_u64.hz());

    scheduler
        .add(WorkNode { name: "worker" })
        .order(0)
        .build()
        .unwrap();

    // Run for 200ms to collect enough samples
    scheduler.run_for(200_u64.ms()).unwrap();

    let metrics = scheduler.metrics();
    assert!(!metrics.is_empty(), "Should have at least one node metric");

    let m = &metrics[0];
    assert!(
        m.total_ticks() > 1,
        "Need multiple ticks for meaningful min/avg/max, got {}",
        m.total_ticks()
    );

    let min = m.min_tick_duration_ms();
    let avg = m.avg_tick_duration_ms();
    let max = m.max_tick_duration_ms();

    // The fundamental ordering invariant: min <= avg <= max
    assert!(
        min <= avg,
        "min ({:.4}ms) should be <= avg ({:.4}ms) for node '{}'",
        min,
        avg,
        m.name()
    );
    assert!(
        avg <= max,
        "avg ({:.4}ms) should be <= max ({:.4}ms) for node '{}'",
        avg,
        max,
        m.name()
    );
}

// ============================================================================
// Test 3: Idle (no-op) node has near-zero tick duration
// ============================================================================

/// INTENT: "A node that does nothing has near-zero tick duration."
///
/// Robotics guarantee: the scheduler overhead per node should be negligible.
/// If a no-op node shows >1ms max tick duration consistently, it means the
/// scheduler itself is adding latency, which steals budget from real work
/// nodes. This test catches scheduler regressions that inflate per-node overhead.
#[test]
fn test_profiler_intent_idle_node_fast_profile() {
    cleanup_stale_shm();

    struct IdleNode;

    impl Node for IdleNode {
        fn name(&self) -> &str {
            "idle"
        }
        fn tick(&mut self) {
            // Intentionally empty — measures pure scheduler overhead
        }
    }

    let mut scheduler = Scheduler::new().tick_rate(500_u64.hz());

    scheduler.add(IdleNode).order(0).build().unwrap();

    // Run for 200ms to collect enough ticks
    scheduler.run_for(200_u64.ms()).unwrap();

    let metrics = scheduler.metrics();
    assert!(!metrics.is_empty(), "Should have at least one node metric");

    let m = &metrics[0];
    assert!(
        m.total_ticks() > 0,
        "Idle node should have ticked at least once"
    );

    // avg tick duration should be well under 1ms for a no-op node.
    // Even on slow CI runners this should hold — a no-op tick is just
    // function dispatch overhead.
    assert!(
        m.avg_tick_duration_ms() < 1.0,
        "No-op node avg tick duration should be < 1ms, got {:.4}ms. \
         This suggests scheduler overhead is too high.",
        m.avg_tick_duration_ms()
    );

    // min should also be very small
    assert!(
        m.min_tick_duration_ms() < 1.0,
        "No-op node min tick duration should be < 1ms, got {:.4}ms",
        m.min_tick_duration_ms()
    );
}
