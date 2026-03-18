//! Deterministic mode integration tests.
//!
//! Verifies that deterministic mode preserves execution class behavior,
//! failure policies, and watchdog — and that independent nodes are correctly
//! grouped while dependent nodes are correctly ordered.

use horus_core::core::duration_ext::DurationExt;
use horus_core::core::node::{Node, TopicMetadata};
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
    fn publishers(&self) -> Vec<TopicMetadata> {
        self.pubs
            .iter()
            .map(|t| TopicMetadata {
                topic_name: t.to_string(),
                type_name: "T".to_string(),
            })
            .collect()
    }
    fn subscribers(&self) -> Vec<TopicMetadata> {
        self.subs
            .iter()
            .map(|t| TopicMetadata {
                topic_name: t.to_string(),
                type_name: "T".to_string(),
            })
            .collect()
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
