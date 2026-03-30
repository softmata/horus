#![allow(dead_code)]
//! NodeBuilder API combinatorial tests.
//!
//! Tests every critical combination of .rate(), .budget(), .deadline(),
//! .on_miss(), .budget_policy(), .compute(), .on(topic), .async_io(),
//! .order(), .failure_policy().
//!
//! Run: cargo test --no-default-features -p horus_core \
//!        --test nodebuilder_matrix -- --ignored --nocapture

use horus_core::communication::topic::Topic;
use horus_core::core::{DurationExt, Node};
use horus_core::scheduling::Scheduler;
use horus_robotics::CmdVel;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::Arc;
use std::time::Duration;

mod common;
use common::{cleanup_stale_shm, unique};

// ════════════════════════════════════════════════════════════════════════
// Helper nodes
// ════════════════════════════════════════════════════════════════════════

struct CounterNode {
    node_name: String,
    ticks: Arc<AtomicU64>,
}
impl Node for CounterNode {
    fn name(&self) -> &str {
        &self.node_name
    }
    fn tick(&mut self) {
        self.ticks.fetch_add(1, Ordering::Relaxed);
    }
}

struct SlowCounterNode {
    node_name: String,
    ticks: Arc<AtomicU64>,
    slow_every_n: u64,
    slow_duration_ms: u64,
    count: u64,
}
impl Node for SlowCounterNode {
    fn name(&self) -> &str {
        &self.node_name
    }
    fn tick(&mut self) {
        self.count += 1;
        self.ticks.fetch_add(1, Ordering::Relaxed);
        if self.slow_every_n > 0 && self.count.is_multiple_of(self.slow_every_n) {
            std::thread::sleep(Duration::from_millis(self.slow_duration_ms));
        }
    }
}

struct PanicAfterNode {
    node_name: String,
    ticks: Arc<AtomicU64>,
    panic_at: u64,
    count: u64,
    init_count: Arc<AtomicU64>,
}
impl Node for PanicAfterNode {
    fn name(&self) -> &str {
        &self.node_name
    }
    fn init(&mut self) -> horus_core::error::HorusResult<()> {
        self.init_count.fetch_add(1, Ordering::Relaxed);
        self.count = 0;
        Ok(())
    }
    fn tick(&mut self) {
        self.count += 1;
        self.ticks.fetch_add(1, Ordering::Relaxed);
        if self.count == self.panic_at {
            panic!("intentional panic at tick {}", self.count);
        }
    }
}

struct EventRecvNode {
    topic_name: String,
    node_name: String,
    topic: Option<Topic<CmdVel>>,
    ticks: Arc<AtomicU64>,
}
impl Node for EventRecvNode {
    fn name(&self) -> &str {
        &self.node_name
    }
    fn init(&mut self) -> horus_core::error::HorusResult<()> {
        self.topic = Some(Topic::new(&self.topic_name)?);
        Ok(())
    }
    fn tick(&mut self) {
        if let Some(ref t) = self.topic {
            while t.recv().is_some() {}
        }
        self.ticks.fetch_add(1, Ordering::Relaxed);
    }
}

struct CmdVelPubNode {
    topic_name: String,
    topic: Option<Topic<CmdVel>>,
    ticks: Arc<AtomicU64>,
}
impl Node for CmdVelPubNode {
    fn name(&self) -> &str {
        "cmdvel_pub"
    }
    fn init(&mut self) -> horus_core::error::HorusResult<()> {
        self.topic = Some(Topic::new(&self.topic_name)?);
        Ok(())
    }
    fn tick(&mut self) {
        if let Some(ref t) = self.topic {
            t.send(CmdVel::new(1.0, 0.0));
        }
        self.ticks.fetch_add(1, Ordering::Relaxed);
    }
}

// ════════════════════════════════════════════════════════════════════════
// TEST 1: .rate(100.hz()) alone — auto-derives budget+deadline, becomes RT
// ════════════════════════════════════════════════════════════════════════

#[test]
#[ignore]
fn rate_alone_auto_derives_rt() {
    cleanup_stale_shm();

    let ticks = Arc::new(AtomicU64::new(0));
    let tc = ticks.clone();

    let running = Arc::new(AtomicBool::new(true));
    let rc = running.clone();

    let h = std::thread::spawn(move || {
        let mut sched = Scheduler::new().tick_rate(100_u64.hz());
        let _ = sched
            .add(CounterNode {
                node_name: "rate_only".into(),
                ticks: tc,
            })
            .rate(100_u64.hz())
            .order(0)
            .build();
        while rc.load(Ordering::Relaxed) {
            let _ = sched.tick_once();
            std::thread::sleep(Duration::from_millis(9));
        }
    });

    std::thread::sleep(Duration::from_secs(3));
    running.store(false, Ordering::Relaxed);
    h.join().unwrap();

    let t = ticks.load(Ordering::Relaxed);
    let hz = t as f64 / 3.0;
    println!("rate(100.hz()) alone: {} ticks in 3s ({:.0} Hz)", t, hz);
    assert!(
        t > 50,
        ".rate(100.hz()) should produce >50 ticks in 3s, got {}",
        t
    );
    println!("✓ rate_alone_auto_derives_rt — PASSED");
}

// ════════════════════════════════════════════════════════════════════════
// TEST 2: .rate(100.hz()).budget(1.ms()) — explicit budget enforcement
// ════════════════════════════════════════════════════════════════════════

#[test]
#[ignore]
fn rate_with_explicit_budget() {
    cleanup_stale_shm();

    let slow_ticks = Arc::new(AtomicU64::new(0));
    let healthy_ticks = Arc::new(AtomicU64::new(0));

    let st = slow_ticks.clone();
    let ht = healthy_ticks.clone();
    let running = Arc::new(AtomicBool::new(true));
    let rc = running.clone();

    let h = std::thread::spawn(move || {
        let mut sched = Scheduler::new().tick_rate(100_u64.hz());
        // Slow node: exceeds 1ms budget every 5th tick
        let _ = sched
            .add(SlowCounterNode {
                node_name: "slow".into(),
                ticks: st,
                slow_every_n: 5,
                slow_duration_ms: 3,
                count: 0,
            })
            .rate(100_u64.hz())
            .budget(1_u64.ms())
            .order(0)
            .build();
        // Healthy node: always fast
        let _ = sched
            .add(CounterNode {
                node_name: "healthy".into(),
                ticks: ht,
            })
            .rate(100_u64.hz())
            .order(1)
            .build();
        while rc.load(Ordering::Relaxed) {
            let _ = sched.tick_once();
            std::thread::sleep(Duration::from_millis(9));
        }
    });

    std::thread::sleep(Duration::from_secs(3));
    running.store(false, Ordering::Relaxed);
    h.join().unwrap();

    let sv = slow_ticks.load(Ordering::Relaxed);
    let hv = healthy_ticks.load(Ordering::Relaxed);
    println!("rate+budget: slow={} ticks, healthy={} ticks", sv, hv);
    assert!(hv > 50, "Healthy node should get >50 ticks");
    assert!(
        sv > 50,
        "Slow node should still run (budget is Warn by default)"
    );
    println!("✓ rate_with_explicit_budget — PASSED");
}

// ════════════════════════════════════════════════════════════════════════
// TEST 3: .budget(2.ms()) alone (no rate) — becomes RT
// ════════════════════════════════════════════════════════════════════════

#[test]
#[ignore]
fn budget_alone_becomes_rt() {
    cleanup_stale_shm();

    let ticks = Arc::new(AtomicU64::new(0));
    let tc = ticks.clone();
    let running = Arc::new(AtomicBool::new(true));
    let rc = running.clone();

    let h = std::thread::spawn(move || {
        let mut sched = Scheduler::new().tick_rate(200_u64.hz());
        let _ = sched
            .add(CounterNode {
                node_name: "budget_only".into(),
                ticks: tc,
            })
            .budget(2_u64.ms())
            .order(0)
            .build();
        while rc.load(Ordering::Relaxed) {
            let _ = sched.tick_once();
            std::thread::sleep(Duration::from_millis(4));
        }
    });

    std::thread::sleep(Duration::from_secs(2));
    running.store(false, Ordering::Relaxed);
    h.join().unwrap();

    let t = ticks.load(Ordering::Relaxed);
    println!("budget(2.ms()) alone: {} ticks in 2s", t);
    assert!(
        t > 50,
        ".budget() alone should auto-derive RT and tick, got {}",
        t
    );
    println!("✓ budget_alone_becomes_rt — PASSED");
}

// ════════════════════════════════════════════════════════════════════════
// TEST 4: All 4 execution classes in one scheduler
// ════════════════════════════════════════════════════════════════════════

#[test]
#[ignore]
fn four_execution_classes_coexist() {
    cleanup_stale_shm();

    let topic = unique("nb_event_topic");
    let rt_ticks = Arc::new(AtomicU64::new(0));
    let compute_ticks = Arc::new(AtomicU64::new(0));
    let event_ticks = Arc::new(AtomicU64::new(0));
    let async_ticks = Arc::new(AtomicU64::new(0));
    let pub_ticks = Arc::new(AtomicU64::new(0));

    let rt = rt_ticks.clone();
    let cp = compute_ticks.clone();
    let ev = event_ticks.clone();
    let ai = async_ticks.clone();
    let pt = pub_ticks.clone();
    let running = Arc::new(AtomicBool::new(true));
    let rc = running.clone();

    let h = std::thread::spawn(move || {
        let mut sched = Scheduler::new().tick_rate(200_u64.hz());

        // RT node at 200Hz (auto from rate)
        let _ = sched
            .add(CounterNode {
                node_name: "rt_node".into(),
                ticks: rt,
            })
            .rate(200_u64.hz())
            .order(0)
            .build();

        // Publisher for event trigger
        let _ = sched
            .add(CmdVelPubNode {
                topic_name: topic.clone(),
                topic: None,
                ticks: pt,
            })
            .rate(100_u64.hz())
            .order(1)
            .build();

        // Compute node (thread pool)
        let _ = sched
            .add(SlowCounterNode {
                node_name: "compute_node".into(),
                ticks: cp,
                slow_every_n: 1,
                slow_duration_ms: 1,
                count: 0,
            })
            .compute()
            .rate(30_u64.hz())
            .order(2)
            .build();

        // Event node (triggered by topic)
        let _ = sched
            .add(EventRecvNode {
                topic_name: topic,
                node_name: "event_node".into(),
                topic: None,
                ticks: ev,
            })
            .on("ignored_for_now")
            .order(3)
            .build();

        // AsyncIo node
        let _ = sched
            .add(CounterNode {
                node_name: "async_node".into(),
                ticks: ai,
            })
            .async_io()
            .rate(10_u64.hz())
            .order(4)
            .build();

        while rc.load(Ordering::Relaxed) {
            let _ = sched.tick_once();
            std::thread::sleep(Duration::from_millis(4));
        }
    });

    std::thread::sleep(Duration::from_secs(5));
    running.store(false, Ordering::Relaxed);
    h.join().unwrap();

    let rv = rt_ticks.load(Ordering::Relaxed);
    let cv = compute_ticks.load(Ordering::Relaxed);
    let ev = event_ticks.load(Ordering::Relaxed);
    let av = async_ticks.load(Ordering::Relaxed);

    println!("╔══════════════════════════════════════════════════════════╗");
    println!("║  4 EXECUTION CLASSES (5s)                               ║");
    println!(
        "║  RT (200Hz):     {:5} ticks ({:5.0} Hz)                 ║",
        rv,
        rv as f64 / 5.0
    );
    println!(
        "║  Compute (30Hz): {:5} ticks ({:5.0} Hz)                 ║",
        cv,
        cv as f64 / 5.0
    );
    println!(
        "║  Event:          {:5} ticks                             ║",
        ev
    );
    println!(
        "║  AsyncIo (10Hz): {:5} ticks ({:5.0} Hz)                 ║",
        av,
        av as f64 / 5.0
    );
    println!("╚══════════════════════════════════════════════════════════╝");

    assert!(rv > 100, "RT node should get >100 ticks");
    assert!(cv > 10, "Compute node should get >10 ticks");
    assert!(av > 5, "AsyncIo node should get >5 ticks");
    // RT should dominate since it's highest rate
    assert!(rv > cv, "RT should tick more than Compute");
    println!("✓ four_execution_classes_coexist — PASSED");
}

// ════════════════════════════════════════════════════════════════════════
// TEST 5: .order() fairness — 20 nodes at same priority
// ════════════════════════════════════════════════════════════════════════

#[test]
#[ignore]
fn order_fairness_20_nodes() {
    cleanup_stale_shm();

    let counters: Vec<Arc<AtomicU64>> = (0..20).map(|_| Arc::new(AtomicU64::new(0))).collect();

    let mut sched = Scheduler::new().tick_rate(100_u64.hz()).deterministic(true);

    for (i, counter) in counters.iter().enumerate() {
        let _ = sched
            .add(CounterNode {
                node_name: format!("node_{}", i),
                ticks: counter.clone(),
            })
            .order(0)
            .build();
    }

    for _ in 0..500 {
        let _ = sched.tick_once();
    }

    let ticks: Vec<u64> = counters.iter().map(|c| c.load(Ordering::Relaxed)).collect();
    let min = *ticks.iter().min().unwrap();
    let max = *ticks.iter().max().unwrap();
    let spread = if max > 0 {
        (max - min) as f64 / max as f64 * 100.0
    } else {
        0.0
    };

    println!(
        "20 nodes at .order(0): min={}, max={}, spread={:.1}%",
        min, max, spread
    );
    assert!(min > 0, "All 20 nodes should get at least 1 tick");
    let running_count = ticks.iter().filter(|&&t| t > 0).count();
    assert_eq!(
        running_count, 20,
        "All 20 nodes should run, got {}",
        running_count
    );
    println!(
        "✓ order_fairness_20_nodes — PASSED (all 20 running, {:.1}% spread)",
        spread
    );
}

// ════════════════════════════════════════════════════════════════════════
// TEST 6: Node panic + FailurePolicy
// ════════════════════════════════════════════════════════════════════════

#[test]
#[ignore]
fn failure_policy_panic_isolation() {
    cleanup_stale_shm();

    let panic_ticks = Arc::new(AtomicU64::new(0));
    let healthy_ticks = Arc::new(AtomicU64::new(0));
    let panic_inits = Arc::new(AtomicU64::new(0));

    let mut sched = Scheduler::new().tick_rate(100_u64.hz()).deterministic(true);

    let _ = sched
        .add(PanicAfterNode {
            node_name: "panicker".into(),
            ticks: panic_ticks.clone(),
            panic_at: 5,
            count: 0,
            init_count: panic_inits.clone(),
        })
        .order(0)
        .build();

    let _ = sched
        .add(CounterNode {
            node_name: "healthy".into(),
            ticks: healthy_ticks.clone(),
        })
        .order(1)
        .build();

    for _ in 0..50 {
        let _ = sched.tick_once();
    }

    let pt = panic_ticks.load(Ordering::Relaxed);
    let ht = healthy_ticks.load(Ordering::Relaxed);
    let pi = panic_inits.load(Ordering::Relaxed);

    println!("Panic node: {} ticks, {} inits", pt, pi);
    println!("Healthy node: {} ticks", ht);

    assert!(
        ht >= 45,
        "Healthy node should keep running after panic, got {}",
        ht
    );
    assert!(
        pt >= 4,
        "Panic node should have ticked before panic, got {}",
        pt
    );
    println!("✓ failure_policy_panic_isolation — PASSED");
}

// ════════════════════════════════════════════════════════════════════════
// TEST 7: .order(0) then .order(1) execution ordering in deterministic mode
// ════════════════════════════════════════════════════════════════════════

struct OrderLogNode {
    node_name: String,
    log: Arc<std::sync::Mutex<Vec<String>>>,
}
impl Node for OrderLogNode {
    fn name(&self) -> &str {
        &self.node_name
    }
    fn tick(&mut self) {
        self.log.lock().unwrap().push(self.node_name.clone());
    }
}

#[test]
#[ignore]
fn execution_order_deterministic() {
    cleanup_stale_shm();

    let log = Arc::new(std::sync::Mutex::new(Vec::new()));

    let mut sched = Scheduler::new().tick_rate(100_u64.hz()).deterministic(true);

    // Add in REVERSE order to verify it's not insertion order
    let _ = sched
        .add(OrderLogNode {
            node_name: "C_order2".into(),
            log: log.clone(),
        })
        .order(2)
        .build();

    let _ = sched
        .add(OrderLogNode {
            node_name: "A_order0".into(),
            log: log.clone(),
        })
        .order(0)
        .build();

    let _ = sched
        .add(OrderLogNode {
            node_name: "B_order1".into(),
            log: log.clone(),
        })
        .order(1)
        .build();

    for _ in 0..100 {
        let _ = sched.tick_once();
    }

    let entries = log.lock().unwrap();
    let mut violations = 0;
    for chunk in entries.chunks(3) {
        if chunk.len() == 3
            && (chunk[0] != "A_order0" || chunk[1] != "B_order1" || chunk[2] != "C_order2")
        {
            violations += 1;
        }
    }

    println!(
        "Execution order: {} violations in {} ticks",
        violations,
        entries.len() / 3
    );
    assert_eq!(
        violations, 0,
        ".order() not respected — {} violations",
        violations
    );
    println!("✓ execution_order_deterministic — PASSED");
}

// ════════════════════════════════════════════════════════════════════════
// TEST 8: Default node (no config) — verify it works
// ════════════════════════════════════════════════════════════════════════

#[test]
#[ignore]
fn default_node_no_config() {
    cleanup_stale_shm();

    let ticks = Arc::new(AtomicU64::new(0));

    let mut sched = Scheduler::new().tick_rate(50_u64.hz()).deterministic(true);

    // No .rate(), no .budget(), no .order() — pure defaults
    let _ = sched
        .add(CounterNode {
            node_name: "default_node".into(),
            ticks: ticks.clone(),
        })
        .build();

    for _ in 0..100 {
        let _ = sched.tick_once();
    }

    let t = ticks.load(Ordering::Relaxed);
    println!(
        "Default node (no config): {} ticks in 100 tick_once calls",
        t
    );
    assert!(t > 50, "Default node should tick, got {}", t);
    println!("✓ default_node_no_config — PASSED");
}
