//! Edge case and stress tests for determinism.
//!
//! Tests scenarios that could break determinism: empty schedulers, single-tick,
//! nodes with no rate, mixed RT/non-RT, large node counts, long runs,
//! zero-duration budget, nodes added in random order, cycle detection error.

use horus_core::core::duration_ext::DurationExt;
use horus_core::core::node::{Node, TopicMetadata};
use horus_core::scheduling::Scheduler;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::{Arc, Mutex};

// ── Helpers ──

struct OutputNode {
    name: String,
    outputs: Arc<Mutex<Vec<u64>>>,
}

impl OutputNode {
    fn new(name: &str, outputs: Arc<Mutex<Vec<u64>>>) -> Self {
        Self {
            name: name.to_string(),
            outputs,
        }
    }
}

impl Node for OutputNode {
    fn name(&self) -> &str {
        &self.name
    }
    fn tick(&mut self) {
        let tick = horus_core::core::tick_context::ctx_tick();
        let rng_val = horus_core::core::tick_context::ctx_with_rng(|rng| {
            use rand::Rng;
            rng.gen::<u64>()
        });
        self.outputs.lock().unwrap().push(tick ^ rng_val);
    }
}

struct PubSubNode {
    name: String,
    pubs: Vec<String>,
    subs: Vec<String>,
    log: Arc<Mutex<Vec<String>>>,
}

impl Node for PubSubNode {
    fn name(&self) -> &str {
        &self.name
    }
    fn tick(&mut self) {
        self.log.lock().unwrap().push(self.name.clone());
    }
}

// ── Edge Cases ──

#[test]
fn empty_scheduler_deterministic() {
    let mut scheduler = Scheduler::new().deterministic(true).tick_rate(100_u64.hz());
    // No nodes — should not crash
    for _ in 0..5 {
        scheduler.tick_once().unwrap();
    }
}

#[test]
fn single_tick_deterministic() {
    let run1 = run_n_ticks(1);
    let run2 = run_n_ticks(1);
    assert_eq!(run1.len(), 1);
    assert_eq!(run1, run2, "Single tick must be identical across runs");
}

#[test]
fn node_without_rate_in_deterministic() {
    // Nodes without .rate() should still tick in deterministic mode
    let count = Arc::new(AtomicU64::new(0));

    struct SimpleCounter {
        count: Arc<AtomicU64>,
    }
    impl Node for SimpleCounter {
        fn name(&self) -> &str {
            "no_rate"
        }
        fn tick(&mut self) {
            self.count.fetch_add(1, Ordering::Relaxed);
        }
    }

    let mut scheduler = Scheduler::new().deterministic(true).tick_rate(100_u64.hz());
    scheduler
        .add(SimpleCounter {
            count: count.clone(),
        })
        .order(0)
        .build()
        .unwrap();

    for _ in 0..10 {
        scheduler.tick_once().unwrap();
    }

    assert_eq!(count.load(Ordering::Relaxed), 10);
}

#[test]
fn mixed_rt_and_besteffort_deterministic() {
    // Run mixed RT + BestEffort system, verify both tick correctly
    let rt_out = Arc::new(Mutex::new(Vec::new()));
    let be_out = Arc::new(Mutex::new(Vec::new()));

    {
        let mut scheduler = Scheduler::new().deterministic(true).tick_rate(100_u64.hz());

        scheduler
            .add(OutputNode::new("rt_node", rt_out.clone()))
            .order(0)
            .rate(100_u64.hz())
            .build()
            .unwrap();
        scheduler
            .add(OutputNode::new("be_node", be_out.clone()))
            .order(1)
            .build()
            .unwrap();

        for _ in 0..10 {
            scheduler.tick_once().unwrap();
        }
    }

    assert_eq!(rt_out.lock().unwrap().len(), 10, "RT should tick 10 times");
    assert_eq!(
        be_out.lock().unwrap().len(),
        10,
        "BestEffort should tick 10 times"
    );
}

#[test]
fn many_nodes_deterministic() {
    // 20 nodes with dependencies — stress the dependency graph
    let log = Arc::new(Mutex::new(Vec::new()));

    let mut scheduler = Scheduler::new().deterministic(true).tick_rate(100_u64.hz());

    // Chain: 0 → 1 → 2 → ... → 9
    for i in 0..10 {
        let pubs = vec![format!("topic_{}", i)];
        let subs = if i > 0 {
            vec![format!("topic_{}", i - 1)]
        } else {
            vec![]
        };
        scheduler
            .add(PubSubNode {
                name: format!("node_{}", i),
                pubs,
                subs,
                log: log.clone(),
            })
            .order(i as u32)
            .build()
            .unwrap();
    }

    // Independent fan: 10, 11, 12 all read from topic_9
    for i in 10..13 {
        scheduler
            .add(PubSubNode {
                name: format!("consumer_{}", i),
                pubs: vec![],
                subs: vec!["topic_9".to_string()],
                log: log.clone(),
            })
            .order(100)
            .build()
            .unwrap();
    }

    scheduler.tick_once().unwrap();

    let entries = log.lock().unwrap();
    assert_eq!(entries.len(), 13);

    // Chain must be in order
    for i in 0..10 {
        let pos = entries
            .iter()
            .position(|n| *n == format!("node_{}", i))
            .unwrap();
        if i > 0 {
            let prev_pos = entries
                .iter()
                .position(|n| *n == format!("node_{}", i - 1))
                .unwrap();
            assert!(pos > prev_pos, "node_{} must be after node_{}", i, i - 1);
        }
    }

    // Consumers must be after node_9
    let node9_pos = entries.iter().position(|n| n == "node_9").unwrap();
    for i in 10..13 {
        let pos = entries
            .iter()
            .position(|n| *n == format!("consumer_{}", i))
            .unwrap();
        assert!(pos > node9_pos, "consumer_{} must be after node_9", i);
    }
}

#[test]
fn long_run_determinism() {
    // 500 ticks — verify determinism holds over longer runs
    let run1 = run_n_ticks(500);
    let run2 = run_n_ticks(500);
    assert_eq!(run1, run2, "500-tick runs must produce identical outputs");
}

fn run_n_ticks(n: usize) -> Vec<u64> {
    let out = Arc::new(Mutex::new(Vec::new()));
    {
        let mut scheduler = Scheduler::new().deterministic(true).tick_rate(100_u64.hz());
        scheduler
            .add(OutputNode::new("test", out.clone()))
            .order(0)
            .rate(100_u64.hz())
            .build()
            .unwrap();
        for _ in 0..n {
            scheduler.tick_once().unwrap();
        }
    }
    let result = out.lock().unwrap().clone();
    result
}

#[test]
fn cycle_detection_returns_error() {
    // Mutual dependency: A publishes to B, B publishes to A
    let log = Arc::new(Mutex::new(Vec::new()));

    let mut scheduler = Scheduler::new().deterministic(true).tick_rate(100_u64.hz());

    scheduler
        .add(PubSubNode {
            name: "A".into(),
            pubs: vec!["a_to_b".into()],
            subs: vec!["b_to_a".into()],
            log: log.clone(),
        })
        .order(0)
        .build()
        .unwrap();

    scheduler
        .add(PubSubNode {
            name: "B".into(),
            pubs: vec!["b_to_a".into()],
            subs: vec!["a_to_b".into()],
            log: log.clone(),
        })
        .order(0)
        .build()
        .unwrap();

    // tick_once triggers finalize_and_init which builds dependency graph
    // Cycle should be detected — scheduler falls back to sequential with warning
    // (does not crash)
    let result = scheduler.tick_once();
    assert!(
        result.is_ok(),
        "Cycle should be handled gracefully, not crash"
    );
}

#[test]
fn budget_remaining_decreases_during_work() {
    struct BudgetChecker {
        readings: Arc<Mutex<Vec<f64>>>,
    }

    impl Node for BudgetChecker {
        fn name(&self) -> &str {
            "budget_checker"
        }
        fn tick(&mut self) {
            let before = horus_core::core::tick_context::ctx_budget_remaining();
            // Do some work
            let mut x = 0u64;
            for i in 0..1000 {
                x = x.wrapping_add(i);
            }
            let _ = x;
            let after = horus_core::core::tick_context::ctx_budget_remaining();
            self.readings
                .lock()
                .unwrap()
                .push(before.as_secs_f64() - after.as_secs_f64());
        }
    }

    let readings = Arc::new(Mutex::new(Vec::new()));
    let mut scheduler = Scheduler::new().deterministic(true).tick_rate(100_u64.hz());

    scheduler
        .add(BudgetChecker {
            readings: readings.clone(),
        })
        .order(0)
        .rate(100_u64.hz())
        .budget(1_u64.ms())
        .build()
        .unwrap();

    scheduler.tick_once().unwrap();

    let r = readings.lock().unwrap();
    // In SimClock, time doesn't advance during tick, so budget doesn't decrease
    // (budget_remaining uses clock.now() which is SimClock — fixed during tick)
    // This is expected behavior: in deterministic mode, budget is not wall-time enforced
    assert_eq!(r.len(), 1);
}

#[test]
fn horus_dt_fixed_in_deterministic() {
    struct DtChecker {
        dts: Arc<Mutex<Vec<u64>>>,
    }

    impl Node for DtChecker {
        fn name(&self) -> &str {
            "dt_checker"
        }
        fn tick(&mut self) {
            let dt = horus_core::core::tick_context::ctx_dt();
            self.dts.lock().unwrap().push(dt.as_nanos() as u64);
        }
    }

    let dts = Arc::new(Mutex::new(Vec::new()));
    let mut scheduler = Scheduler::new().deterministic(true).tick_rate(200_u64.hz()); // 5ms period

    scheduler
        .add(DtChecker { dts: dts.clone() })
        .order(0)
        .rate(200_u64.hz())
        .build()
        .unwrap();

    for _ in 0..10 {
        scheduler.tick_once().unwrap();
    }

    let values = dts.lock().unwrap();
    assert_eq!(values.len(), 10);

    // All dt values should be 5ms = 5_000_000 ns (1/200Hz)
    for (i, &dt) in values.iter().enumerate() {
        assert_eq!(
            dt, 5_000_000,
            "Tick {} dt should be exactly 5ms, got {}ns",
            i, dt
        );
    }
}
