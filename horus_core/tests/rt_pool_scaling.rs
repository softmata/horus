//! RT thread pool scaling and per-node configuration tests.
//!
//! Tests the RT executor pool via the Scheduler public API.

use horus_core::core::duration_ext::DurationExt;
use horus_core::core::node::Node;
use horus_core::scheduling::Scheduler;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::Arc;

// ── Helpers ──────────────────────────────────────────────────────────────────

struct RtCounterNode {
    name: String,
    count: Arc<AtomicU64>,
}

impl RtCounterNode {
    fn new(name: &str, count: Arc<AtomicU64>) -> Self {
        Self {
            name: name.to_string(),
            count,
        }
    }
}

impl Node for RtCounterNode {
    fn name(&self) -> &str {
        &self.name
    }
    fn tick(&mut self) {
        self.count.fetch_add(1, Ordering::Relaxed);
    }
}

// ── Tests ────────────────────────────────────────────────────────────────────

#[test]
fn single_rt_node_runs() {
    let count = Arc::new(AtomicU64::new(0));

    let mut scheduler = Scheduler::new().tick_rate(100_u64.hz());
    scheduler
        .add(RtCounterNode::new("motor", count.clone()))
        .order(0)
        .rate(100_u64.hz())
        .build()
        .unwrap();

    scheduler.run_for(50_u64.ms()).unwrap();

    assert!(
        count.load(Ordering::Relaxed) > 0,
        "RT node should have ticked"
    );
}

#[test]
fn multiple_independent_rt_nodes_run() {
    let counts: Vec<_> = (0..4).map(|_| Arc::new(AtomicU64::new(0))).collect();

    let mut scheduler = Scheduler::new().tick_rate(100_u64.hz());

    for (i, c) in counts.iter().enumerate() {
        scheduler
            .add(RtCounterNode::new(&format!("joint_{}", i), c.clone()))
            .order(0)
            .rate(100_u64.hz())
            .build()
            .unwrap();
    }

    scheduler.run_for(50_u64.ms()).unwrap();

    for (i, c) in counts.iter().enumerate() {
        assert!(
            c.load(Ordering::Relaxed) > 0,
            "RT node {} should have ticked",
            i
        );
    }
}

#[test]
fn rt_node_with_priority() {
    let count = Arc::new(AtomicU64::new(0));

    let mut scheduler = Scheduler::new().tick_rate(100_u64.hz());
    scheduler
        .add(RtCounterNode::new("safety", count.clone()))
        .order(0)
        .rate(100_u64.hz())
        .priority(99)
        .build()
        .unwrap();

    // Should not crash even if SCHED_FIFO unavailable
    scheduler.run_for(30_u64.ms()).unwrap();

    assert!(count.load(Ordering::Relaxed) > 0);
}

#[test]
fn rt_node_with_core_pinning() {
    let count = Arc::new(AtomicU64::new(0));

    let mut scheduler = Scheduler::new().tick_rate(100_u64.hz());
    scheduler
        .add(RtCounterNode::new("motor", count.clone()))
        .order(0)
        .rate(100_u64.hz())
        .core(0)
        .build()
        .unwrap();

    // Should not crash even if core 0 is unavailable for pinning
    scheduler.run_for(30_u64.ms()).unwrap();

    assert!(count.load(Ordering::Relaxed) > 0);
}

#[test]
fn rt_node_with_per_node_watchdog() {
    let count = Arc::new(AtomicU64::new(0));

    let mut scheduler = Scheduler::new()
        .tick_rate(100_u64.hz())
        .watchdog(500_u64.ms()); // global 500ms

    scheduler
        .add(RtCounterNode::new("safety_critical", count.clone()))
        .order(0)
        .rate(100_u64.hz())
        .watchdog(10_u64.ms()) // per-node 10ms override
        .build()
        .unwrap();

    scheduler.run_for(30_u64.ms()).unwrap();

    assert!(count.load(Ordering::Relaxed) > 0);
}

#[test]
fn non_rt_node_with_watchdog() {
    let count = Arc::new(AtomicU64::new(0));

    let mut scheduler = Scheduler::new().tick_rate(100_u64.hz());

    // Non-RT node (no .rate()) but with per-node watchdog
    scheduler
        .add(RtCounterNode::new("logger", count.clone()))
        .order(10)
        .watchdog(5_u64.secs())
        .build()
        .unwrap();

    scheduler.run_for(30_u64.ms()).unwrap();

    assert!(count.load(Ordering::Relaxed) > 0);
}

#[test]
fn deterministic_mode_ticks_all_rt_nodes() {
    // In deterministic mode (tick_once), RT nodes should tick on main thread
    let counts: Vec<_> = (0..3).map(|_| Arc::new(AtomicU64::new(0))).collect();

    let mut scheduler = Scheduler::new().deterministic(true).tick_rate(100_u64.hz());

    for (i, c) in counts.iter().enumerate() {
        scheduler
            .add(RtCounterNode::new(&format!("rt_{}", i), c.clone()))
            .order(0)
            .rate(100_u64.hz())
            .build()
            .unwrap();
    }

    for _ in 0..10 {
        scheduler.tick_once().unwrap();
    }

    for (i, c) in counts.iter().enumerate() {
        assert_eq!(
            c.load(Ordering::Relaxed),
            10,
            "RT node {} should tick 10 times in deterministic mode",
            i
        );
    }
}
