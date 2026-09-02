#![allow(dead_code)]
//! Deterministic mode integration tests.
//!
//! Verifies that deterministic mode preserves execution class behavior,
//! failure policies, and watchdog — and that independent nodes are correctly
//! grouped while dependent nodes are correctly ordered.

use horus_core::core::duration_ext::DurationExt;
use horus_core::core::node::Node;
use horus_core::scheduling::Scheduler;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::{Arc, Mutex};

// ── Helpers ──────────────────────────────────────────────────────────────────

struct TrackedNode {
    name: &'static str,
    pubs: Vec<&'static str>,
    subs: Vec<&'static str>,
    log: Arc<Mutex<Vec<String>>>,
    tick_count: Arc<AtomicU64>,
}

impl TrackedNode {
    fn new(
        name: &'static str,
        pubs: Vec<&'static str>,
        subs: Vec<&'static str>,
        log: Arc<Mutex<Vec<String>>>,
        tick_count: Arc<AtomicU64>,
    ) -> Self {
        Self {
            name,
            pubs,
            subs,
            log,
            tick_count,
        }
    }
}

impl Node for TrackedNode {
    fn name(&self) -> &str {
        self.name
    }
    fn tick(&mut self) {
        self.log.lock().unwrap().push(self.name.to_string());
        self.tick_count.fetch_add(1, Ordering::Relaxed);
    }
}

// ── Tests ────────────────────────────────────────────────────────────────────

#[test]
fn deterministic_dependency_chain_ordering() {
    // A -> B -> C: must execute in order regardless of add order
    let log = Arc::new(Mutex::new(Vec::new()));
    let counts: Vec<_> = (0..3).map(|_| Arc::new(AtomicU64::new(0))).collect();

    let mut scheduler = Scheduler::new().deterministic(true).tick_rate(100_u64.hz());

    // Add in REVERSE order — dependency graph should fix it
    scheduler
        .add(TrackedNode::new(
            "C",
            vec![],
            vec!["bc"],
            log.clone(),
            counts[2].clone(),
        ))
        .order(2)
        .build()
        .unwrap();
    scheduler
        .add(TrackedNode::new(
            "B",
            vec!["bc"],
            vec!["ab"],
            log.clone(),
            counts[1].clone(),
        ))
        .order(1)
        .build()
        .unwrap();
    scheduler
        .add(TrackedNode::new(
            "A",
            vec!["ab"],
            vec![],
            log.clone(),
            counts[0].clone(),
        ))
        .order(0)
        .build()
        .unwrap();

    for _ in 0..5 {
        scheduler.tick_once().unwrap();
    }

    let entries = log.lock().unwrap();
    // Every 3 entries should be A, B, C in order
    for tick in 0..5 {
        let base = tick * 3;
        assert_eq!(entries[base], "A", "tick {}: A should be first", tick);
        assert_eq!(entries[base + 1], "B", "tick {}: B should be second", tick);
        assert_eq!(entries[base + 2], "C", "tick {}: C should be third", tick);
    }

    // All ticked 5 times
    for (i, c) in counts.iter().enumerate() {
        assert_eq!(
            c.load(Ordering::Relaxed),
            5,
            "Node {} should tick 5 times",
            i
        );
    }
}

#[test]
fn deterministic_independent_nodes_same_step() {
    // A and B are independent (no shared topics), C depends on both
    let log = Arc::new(Mutex::new(Vec::new()));
    let count = Arc::new(AtomicU64::new(0));

    let mut scheduler = Scheduler::new().deterministic(true).tick_rate(100_u64.hz());

    scheduler
        .add(TrackedNode::new(
            "A",
            vec!["a_out"],
            vec![],
            log.clone(),
            count.clone(),
        ))
        .order(0)
        .build()
        .unwrap();
    scheduler
        .add(TrackedNode::new(
            "B",
            vec!["b_out"],
            vec![],
            log.clone(),
            count.clone(),
        ))
        .order(0)
        .build()
        .unwrap();
    scheduler
        .add(TrackedNode::new(
            "C",
            vec![],
            vec!["a_out", "b_out"],
            log.clone(),
            count.clone(),
        ))
        .order(1)
        .build()
        .unwrap();

    scheduler.tick_once().unwrap();

    let entries = log.lock().unwrap();
    assert_eq!(entries.len(), 3);

    // A and B must both execute BEFORE C
    let a_pos = entries.iter().position(|n| n == "A").unwrap();
    let b_pos = entries.iter().position(|n| n == "B").unwrap();
    let c_pos = entries.iter().position(|n| n == "C").unwrap();

    assert!(a_pos < c_pos, "A must execute before C");
    assert!(b_pos < c_pos, "B must execute before C");
}

#[test]
fn deterministic_diamond_dependency() {
    // Diamond: A -> B, A -> C, B -> D, C -> D
    let log = Arc::new(Mutex::new(Vec::new()));
    let count = Arc::new(AtomicU64::new(0));

    let mut scheduler = Scheduler::new().deterministic(true).tick_rate(100_u64.hz());

    scheduler
        .add(TrackedNode::new(
            "A",
            vec!["a_b", "a_c"],
            vec![],
            log.clone(),
            count.clone(),
        ))
        .order(0)
        .build()
        .unwrap();
    scheduler
        .add(TrackedNode::new(
            "B",
            vec!["b_d"],
            vec!["a_b"],
            log.clone(),
            count.clone(),
        ))
        .order(1)
        .build()
        .unwrap();
    scheduler
        .add(TrackedNode::new(
            "C",
            vec!["c_d"],
            vec!["a_c"],
            log.clone(),
            count.clone(),
        ))
        .order(1)
        .build()
        .unwrap();
    scheduler
        .add(TrackedNode::new(
            "D",
            vec![],
            vec!["b_d", "c_d"],
            log.clone(),
            count.clone(),
        ))
        .order(2)
        .build()
        .unwrap();

    scheduler.tick_once().unwrap();

    let entries = log.lock().unwrap();
    assert_eq!(entries.len(), 4);

    let a_pos = entries.iter().position(|n| n == "A").unwrap();
    let b_pos = entries.iter().position(|n| n == "B").unwrap();
    let c_pos = entries.iter().position(|n| n == "C").unwrap();
    let d_pos = entries.iter().position(|n| n == "D").unwrap();

    assert_eq!(a_pos, 0, "A must be first (no dependencies)");
    assert!(b_pos < d_pos, "B must execute before D");
    assert!(c_pos < d_pos, "C must execute before D");
    assert_eq!(d_pos, 3, "D must be last (depends on B and C)");
}

#[test]
fn deterministic_ordering_stable_across_runs() {
    // Run the same dependency graph 20 times — order must be identical every time
    let mut all_orders: Vec<Vec<String>> = Vec::new();

    for _ in 0..20 {
        let log = Arc::new(Mutex::new(Vec::new()));
        let count = Arc::new(AtomicU64::new(0));

        let mut scheduler = Scheduler::new().deterministic(true).tick_rate(100_u64.hz());

        scheduler
            .add(TrackedNode::new(
                "sensor",
                vec!["scan"],
                vec![],
                log.clone(),
                count.clone(),
            ))
            .order(0)
            .build()
            .unwrap();
        scheduler
            .add(TrackedNode::new(
                "filter",
                vec!["filtered"],
                vec!["scan"],
                log.clone(),
                count.clone(),
            ))
            .order(1)
            .build()
            .unwrap();
        scheduler
            .add(TrackedNode::new(
                "controller",
                vec!["cmd"],
                vec!["filtered"],
                log.clone(),
                count.clone(),
            ))
            .order(2)
            .build()
            .unwrap();
        scheduler
            .add(TrackedNode::new(
                "motor",
                vec![],
                vec!["cmd"],
                log.clone(),
                count.clone(),
            ))
            .order(3)
            .build()
            .unwrap();

        scheduler.tick_once().unwrap();

        all_orders.push(log.lock().unwrap().clone());
    }

    for (i, order) in all_orders.iter().enumerate().skip(1) {
        assert_eq!(
            order, &all_orders[0],
            "Run {} has different order than run 0",
            i
        );
    }
}

#[test]
fn deterministic_mode_with_failure_policy_restart() {
    struct FailingNode {
        fail_count: u32,
        max_fails: u32,
        ticked: Arc<AtomicU64>,
    }

    impl Node for FailingNode {
        fn name(&self) -> &str {
            "failing"
        }
        fn tick(&mut self) {
            self.fail_count += 1;
            self.ticked.fetch_add(1, Ordering::Relaxed);
            if self.fail_count <= self.max_fails {
                panic!("intentional failure {}", self.fail_count);
            }
        }
    }

    let ticked = Arc::new(AtomicU64::new(0));

    let mut scheduler = Scheduler::new().deterministic(true).tick_rate(100_u64.hz());

    scheduler
        .add(FailingNode {
            fail_count: 0,
            max_fails: 2,
            ticked: ticked.clone(),
        })
        .order(0)
        .failure_policy(horus_core::scheduling::FailurePolicy::Ignore)
        .build()
        .unwrap();

    // Should not crash despite panics — Ignore policy swallows them
    for _ in 0..5 {
        let _ = scheduler.tick_once();
    }

    assert!(
        ticked.load(Ordering::Relaxed) >= 3,
        "Node should have ticked at least 3 times"
    );
}

#[test]
fn deterministic_fallback_to_order_tiers() {
    // Nodes without pub/sub metadata — should group by .order() tiers
    let log = Arc::new(Mutex::new(Vec::new()));
    let count = Arc::new(AtomicU64::new(0));

    struct SimpleNode {
        name: String,
        log: Arc<Mutex<Vec<String>>>,
        count: Arc<AtomicU64>,
    }

    impl Node for SimpleNode {
        fn name(&self) -> &str {
            &self.name
        }
        fn tick(&mut self) {
            self.log.lock().unwrap().push(self.name.clone());
            self.count.fetch_add(1, Ordering::Relaxed);
        }
    }

    let mut scheduler = Scheduler::new().deterministic(true).tick_rate(100_u64.hz());

    // Order 10 nodes should run AFTER order 0 nodes
    scheduler
        .add(SimpleNode {
            name: "late".into(),
            log: log.clone(),
            count: count.clone(),
        })
        .order(10)
        .build()
        .unwrap();
    scheduler
        .add(SimpleNode {
            name: "early".into(),
            log: log.clone(),
            count: count.clone(),
        })
        .order(0)
        .build()
        .unwrap();

    scheduler.tick_once().unwrap();

    let entries = log.lock().unwrap();
    assert_eq!(entries[0], "early");
    assert_eq!(entries[1], "late");
}

// ── Simulated time advances per TICK, not per dependency-graph step ─────────

/// Build `n` nodes chained A->B->C..., so the dependency graph has `n` steps.
///
/// Node and topic names are unique per chain length. Topics are process-global,
/// so reusing the `ab`/`bc` names the ordering tests above already registered
/// would let their participants join this chain's graph and change its shape --
/// which is exactly the confound this test is trying to measure.
fn chained(n: usize, count: Arc<AtomicU64>) -> Scheduler {
    // The name and edge tables below are fixed-size and hand-written, so this
    // helper is only correct for the two shapes it was built for. Without this
    // the next caller to try `chained(6, ..)` gets an out-of-bounds panic
    // pointing at an array literal rather than at their call.
    assert!(
        n == 2 || n == 4,
        "chained() supports a 2- or 4-node chain, got {n}"
    );
    let names: [[&str; 4]; 2] = [
        ["clk2_a", "clk2_b", "clk2_c", "clk2_d"],
        ["clk4_a", "clk4_b", "clk4_c", "clk4_d"],
    ];
    let edges: [[&str; 3]; 2] = [
        ["clk2_ab", "clk2_bc", "clk2_cd"],
        ["clk4_ab", "clk4_bc", "clk4_cd"],
    ];
    let v = if n <= 2 { 0 } else { 1 };
    let log = Arc::new(Mutex::new(Vec::new()));
    let mut s = Scheduler::new().deterministic(true).tick_rate(100_u64.hz());
    for i in 0..n {
        let pubs = if i + 1 < n { vec![edges[v][i]] } else { vec![] };
        let subs = if i > 0 { vec![edges[v][i - 1]] } else { vec![] };
        s.add(TrackedNode::new(
            names[v][i],
            pubs,
            subs,
            log.clone(),
            count.clone(),
        ))
        .order(i as u32)
        .build()
        .unwrap();
    }
    s
}

/// Simulated time must advance once per TICK, whatever shape the graph is.
///
/// `should_stop_loop` states this invariant in as many words -- "in
/// deterministic mode `self.clock` is a `SimClock` that advances by exactly one
/// tick period per tick, and the point of the mode is that a run does not depend
/// on how fast the machine is" -- and `run_for` is bounded by that clock. But
/// `clock.advance(period)` sat INSIDE the `for step in &steps` loop, so a graph
/// with three steps ran simulated time forward three periods per tick.
///
/// The consequences were both visible to a user: `horus::now()` moved at 3x rate
/// inside a chained pipeline, and `run_for(d)` in deterministic mode executed
/// `d / (steps * period)` ticks instead of `d / period` -- so adding a node to
/// the end of a pipeline silently cut the number of ticks a fixed-duration run
/// performed.
///
/// The invariant under test is the one that makes the mode worth having: tick
/// count for a fixed duration is a property of the RATE, not of the graph shape.
#[test]
fn simulated_time_does_not_run_faster_when_the_graph_has_more_steps() {
    let two = Arc::new(AtomicU64::new(0));
    chained(2, two.clone()).run_for(500_u64.ms()).unwrap();
    // Each node ticks once per scheduler tick.
    let two_ticks = two.load(Ordering::Relaxed) / 2;

    let four = Arc::new(AtomicU64::new(0));
    chained(4, four.clone()).run_for(500_u64.ms()).unwrap();
    let four_ticks = four.load(Ordering::Relaxed) / 4;

    assert_eq!(
        two_ticks, four_ticks,
        "a 4-step chain ran {four_ticks} ticks where a 2-step chain ran \
         {two_ticks} for the same simulated duration. Simulated time is \
         advancing per dependency-graph STEP rather than per tick, so the \
         length of a fixed-duration deterministic run depends on the shape of \
         the pipeline."
    );

    // Anti-vacuity: 500ms at 100Hz is 50 ticks. If both were 0 the equality
    // above would hold while proving nothing.
    assert!(
        two_ticks >= 40,
        "expected ~50 ticks from 500ms at 100Hz, got {two_ticks} — the run \
         ended early and this test proves nothing"
    );
}
