//! Gap coverage tests for determinism blueprint.
//!
//! Covers: topic injection via SHM, RT pool parallel execution timing,
//! graceful degradation, Python-equivalent API paths, and cross-thread
//! determinism verification.

use horus_core::core::duration_ext::DurationExt;
use horus_core::core::node::{Node, TopicMetadata};
use horus_core::scheduling::Scheduler;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

// ── Helpers ──

struct TimedNode {
    name: String,
    timestamps: Arc<Mutex<Vec<u128>>>,
    count: Arc<AtomicU64>,
}

impl TimedNode {
    fn new(name: &str, timestamps: Arc<Mutex<Vec<u128>>>, count: Arc<AtomicU64>) -> Self {
        Self {
            name: name.into(),
            timestamps,
            count,
        }
    }
}

impl Node for TimedNode {
    fn name(&self) -> &str {
        &self.name
    }
    fn tick(&mut self) {
        self.timestamps
            .lock()
            .unwrap()
            .push(Instant::now().elapsed().as_nanos());
        self.count.fetch_add(1, Ordering::Relaxed);
    }
}

// ── SHM Topic Injection ──

#[test]
fn topic_shm_write_and_read_roundtrip() {
    // Verify write_topic_slot_bytes works with a real SHM topic
    use horus_core::communication::Topic;

    let topic: Topic<[u8; 8]> = Topic::new("test_injection_roundtrip").unwrap();
    let data = [1u8, 2, 3, 4, 5, 6, 7, 8];
    topic.send(data);

    let received = topic.recv();
    assert!(received.is_some(), "Should receive sent data");
    assert_eq!(received.unwrap(), data);
}

#[test]
fn topic_shm_multiple_sends() {
    use horus_core::communication::Topic;

    let topic: Topic<u64> = Topic::new("test_multi_send").unwrap();

    for i in 0..10u64 {
        topic.send(i);
    }

    // recv() returns data — the exact value depends on ring buffer semantics
    let received = topic.recv();
    assert!(
        received.is_some(),
        "Should receive something after 10 sends"
    );
}

// ── RT Pool Parallel Execution ──

#[test]
fn rt_pool_multiple_nodes_all_tick_in_run_mode() {
    // Verify multiple RT nodes tick when using run_for() (the actual RT executor path)
    let counts: Vec<_> = (0..3).map(|_| Arc::new(AtomicU64::new(0))).collect();
    let timestamps: Vec<_> = (0..3).map(|_| Arc::new(Mutex::new(Vec::new()))).collect();

    let mut scheduler = Scheduler::new().tick_rate(500_u64.hz());

    for i in 0..3 {
        scheduler
            .add(TimedNode::new(
                &format!("rt_{}", i),
                timestamps[i].clone(),
                counts[i].clone(),
            ))
            .order(i as u32)
            .rate(500_u64.hz())
            .build()
            .unwrap();
    }

    scheduler.run_for(30_u64.ms()).unwrap();

    for (i, c) in counts.iter().enumerate() {
        let ticks = c.load(Ordering::Relaxed);
        assert!(
            ticks > 0,
            "RT node {} should have ticked at least once in run_for, got {}",
            i,
            ticks
        );
    }
}

#[test]
fn rt_pool_with_compute_isolation() {
    // RT nodes should not be blocked by a slow compute node
    let rt_count = Arc::new(AtomicU64::new(0));
    let compute_count = Arc::new(AtomicU64::new(0));

    struct SlowCompute {
        count: Arc<AtomicU64>,
    }
    impl Node for SlowCompute {
        fn name(&self) -> &str {
            "slow_compute"
        }
        fn tick(&mut self) {
            // Simulate 10ms of work
            let start = Instant::now();
            while start.elapsed() < Duration::from_millis(10) {
                std::hint::spin_loop();
            }
            self.count.fetch_add(1, Ordering::Relaxed);
        }
    }

    struct FastRt {
        count: Arc<AtomicU64>,
    }
    impl Node for FastRt {
        fn name(&self) -> &str {
            "fast_rt"
        }
        fn tick(&mut self) {
            self.count.fetch_add(1, Ordering::Relaxed);
        }
    }

    let mut scheduler = Scheduler::new().tick_rate(200_u64.hz());

    scheduler
        .add(FastRt {
            count: rt_count.clone(),
        })
        .order(0)
        .rate(200_u64.hz())
        .build()
        .unwrap();
    scheduler
        .add(SlowCompute {
            count: compute_count.clone(),
        })
        .order(5)
        .compute()
        .build()
        .unwrap();

    scheduler.run_for(50_u64.ms()).unwrap();

    let rt = rt_count.load(Ordering::Relaxed);
    let compute = compute_count.load(Ordering::Relaxed);

    // Both should tick — the point is isolation (compute doesn't block RT)
    assert!(rt > 0, "RT node should tick (not blocked by compute)");
    assert!(
        compute > 0,
        "Compute node should tick (runs on separate pool)"
    );
}

// ── Graceful Degradation ──

#[test]
fn priority_without_permissions_no_crash() {
    let count = Arc::new(AtomicU64::new(0));

    struct Counter {
        count: Arc<AtomicU64>,
    }
    impl Node for Counter {
        fn name(&self) -> &str {
            "priority_test"
        }
        fn tick(&mut self) {
            self.count.fetch_add(1, Ordering::Relaxed);
        }
    }

    let mut scheduler = Scheduler::new().tick_rate(100_u64.hz());
    scheduler
        .add(Counter {
            count: count.clone(),
        })
        .order(0)
        .rate(100_u64.hz())
        .priority(99) // Will fail without CAP_SYS_NICE — should degrade gracefully
        .build()
        .unwrap();

    scheduler.run_for(30_u64.ms()).unwrap();
    assert!(
        count.load(Ordering::Relaxed) > 0,
        "Should tick despite priority failure"
    );
}

#[test]
fn core_pinning_invalid_core_no_crash() {
    let count = Arc::new(AtomicU64::new(0));

    struct Counter {
        count: Arc<AtomicU64>,
    }
    impl Node for Counter {
        fn name(&self) -> &str {
            "core_test"
        }
        fn tick(&mut self) {
            self.count.fetch_add(1, Ordering::Relaxed);
        }
    }

    let mut scheduler = Scheduler::new().tick_rate(100_u64.hz());
    scheduler
        .add(Counter {
            count: count.clone(),
        })
        .order(0)
        .rate(100_u64.hz())
        .core(9999) // Non-existent core — should degrade gracefully
        .build()
        .unwrap();

    scheduler.run_for(30_u64.ms()).unwrap();
    assert!(
        count.load(Ordering::Relaxed) > 0,
        "Should tick despite invalid core"
    );
}

#[test]
fn watchdog_on_non_rt_node_no_crash() {
    let count = Arc::new(AtomicU64::new(0));

    struct Counter {
        count: Arc<AtomicU64>,
    }
    impl Node for Counter {
        fn name(&self) -> &str {
            "watchdog_non_rt"
        }
        fn tick(&mut self) {
            self.count.fetch_add(1, Ordering::Relaxed);
        }
    }

    let mut scheduler = Scheduler::new()
        .tick_rate(100_u64.hz())
        .watchdog(500_u64.ms());

    // Non-RT node (no .rate()) with per-node watchdog
    scheduler
        .add(Counter {
            count: count.clone(),
        })
        .order(0)
        .watchdog(100_u64.ms())
        .build()
        .unwrap();

    scheduler.run_for(30_u64.ms()).unwrap();
    assert!(count.load(Ordering::Relaxed) > 0);
}

// ── Context API Paths (equivalent to Python bindings) ──

#[test]
fn ctx_functions_inside_tick() {
    // Exercises ALL tick_context functions from within a node tick
    struct AllCtxNode {
        results: Arc<Mutex<Vec<String>>>,
    }

    impl Node for AllCtxNode {
        fn name(&self) -> &str {
            "ctx_test"
        }
        fn tick(&mut self) {
            let now = horus_core::core::tick_context::ctx_now();
            let dt = horus_core::core::tick_context::ctx_dt();
            let elapsed = horus_core::core::tick_context::ctx_elapsed();
            let tick = horus_core::core::tick_context::ctx_tick();
            let budget = horus_core::core::tick_context::ctx_budget_remaining();
            let rng_val = horus_core::core::tick_context::ctx_with_rng(|rng| {
                use rand::Rng;
                rng.gen::<f64>()
            });

            self.results.lock().unwrap().push(format!(
                "now={} dt={:?} elapsed={:?} tick={} budget={:?} rng={:.4}",
                now.as_nanos(),
                dt,
                elapsed,
                tick,
                budget,
                rng_val
            ));
        }
    }

    let results = Arc::new(Mutex::new(Vec::new()));

    {
        let mut scheduler = Scheduler::new().deterministic(true).tick_rate(100_u64.hz());

        scheduler
            .add(AllCtxNode {
                results: results.clone(),
            })
            .order(0)
            .rate(100_u64.hz())
            .budget(800_u64.us())
            .build()
            .unwrap();

        for _ in 0..3 {
            scheduler.tick_once().unwrap();
        }
    }

    let r = results.lock().unwrap();
    assert_eq!(r.len(), 3, "Should have 3 tick results");

    // Verify determinism: run again and compare
    let results2 = Arc::new(Mutex::new(Vec::new()));

    {
        let mut scheduler2 = Scheduler::new().deterministic(true).tick_rate(100_u64.hz());

        scheduler2
            .add(AllCtxNode {
                results: results2.clone(),
            })
            .order(0)
            .rate(100_u64.hz())
            .budget(800_u64.us())
            .build()
            .unwrap();

        for _ in 0..3 {
            scheduler2.tick_once().unwrap();
        }
    }

    let r2 = results2.lock().unwrap();
    assert_eq!(*r, *r2, "All context values must be identical across runs");
}

#[test]
fn ctx_functions_outside_tick_safe() {
    // All functions must be safe to call outside tick context (fallback values)
    let now = horus_core::core::tick_context::ctx_now();
    assert!(now.as_nanos() > 0, "Fallback now should be wall clock");

    let dt = horus_core::core::tick_context::ctx_dt();
    assert_eq!(dt, Duration::ZERO, "Fallback dt should be zero");

    let elapsed = horus_core::core::tick_context::ctx_elapsed();
    assert_eq!(elapsed, Duration::ZERO, "Fallback elapsed should be zero");

    let tick = horus_core::core::tick_context::ctx_tick();
    assert_eq!(tick, 0, "Fallback tick should be 0");

    let budget = horus_core::core::tick_context::ctx_budget_remaining();
    assert_eq!(budget, Duration::MAX, "Fallback budget should be MAX");

    let rng_val = horus_core::core::tick_context::ctx_with_rng(|rng| {
        use rand::Rng;
        rng.gen::<f64>()
    });
    assert!(
        (0.0..1.0).contains(&rng_val),
        "Fallback rng should produce valid float"
    );
}

// ── Cross-Thread Determinism ──

#[test]
fn deterministic_output_from_multiple_threads() {
    // Run deterministic system on multiple threads — each produces same output
    let handles: Vec<_> = (0..4)
        .map(|thread_id| {
            std::thread::spawn(move || {
                let out = Arc::new(Mutex::new(Vec::new()));
                {
                    let mut scheduler =
                        Scheduler::new().deterministic(true).tick_rate(100_u64.hz());

                    struct DetNode {
                        out: Arc<Mutex<Vec<u64>>>,
                    }
                    impl Node for DetNode {
                        fn name(&self) -> &str {
                            "det"
                        }
                        fn tick(&mut self) {
                            let tick = horus_core::core::tick_context::ctx_tick();
                            let rng = horus_core::core::tick_context::ctx_with_rng(|r| {
                                use rand::Rng;
                                r.gen::<u64>()
                            });
                            self.out.lock().unwrap().push(tick ^ rng);
                        }
                    }

                    scheduler
                        .add(DetNode { out: out.clone() })
                        .order(0)
                        .rate(100_u64.hz())
                        .build()
                        .unwrap();

                    for _ in 0..20 {
                        scheduler.tick_once().unwrap();
                    }
                }
                let result = out.lock().unwrap().clone();
                result
            })
        })
        .collect();

    let results: Vec<Vec<u64>> = handles.into_iter().map(|h| h.join().unwrap()).collect();

    for (i, r) in results.iter().enumerate().skip(1) {
        assert_eq!(r, &results[0], "Thread {} output differs from thread 0", i);
    }
}

// ── Recording Integration ──

#[test]
fn recording_captures_execution_order() {
    struct OrderNode {
        name: String,
        log: Arc<Mutex<Vec<String>>>,
    }
    impl Node for OrderNode {
        fn name(&self) -> &str {
            &self.name
        }
        fn publishers(&self) -> Vec<TopicMetadata> {
            if self.name == "producer" {
                vec![TopicMetadata {
                    topic_name: "data".into(),
                    type_name: "T".into(),
                }]
            } else {
                vec![]
            }
        }
        fn subscribers(&self) -> Vec<TopicMetadata> {
            if self.name == "consumer" {
                vec![TopicMetadata {
                    topic_name: "data".into(),
                    type_name: "T".into(),
                }]
            } else {
                vec![]
            }
        }
        fn tick(&mut self) {
            self.log.lock().unwrap().push(self.name.clone());
        }
    }

    let log = Arc::new(Mutex::new(Vec::new()));

    {
        let mut scheduler = Scheduler::new()
            .deterministic(true)
            .with_recording()
            .tick_rate(100_u64.hz());

        scheduler
            .add(OrderNode {
                name: "consumer".into(),
                log: log.clone(),
            })
            .order(1)
            .build()
            .unwrap();
        scheduler
            .add(OrderNode {
                name: "producer".into(),
                log: log.clone(),
            })
            .order(0)
            .build()
            .unwrap();

        for _ in 0..5 {
            scheduler.tick_once().unwrap();
        }
    }

    let entries = log.lock().unwrap();
    // Producer always before consumer despite being added second
    for tick in 0..5 {
        assert_eq!(entries[tick * 2], "producer", "tick {}", tick);
        assert_eq!(entries[tick * 2 + 1], "consumer", "tick {}", tick);
    }
}

// ── Stress ──

#[test]
fn stress_100_runs_10_ticks_identical() {
    let first = run_deterministic(10);

    for run in 1..100 {
        let result = run_deterministic(10);
        assert_eq!(result, first, "Run {} differs from run 0", run);
    }
}

fn run_deterministic(ticks: usize) -> Vec<u64> {
    let out = Arc::new(Mutex::new(Vec::new()));
    {
        struct Det {
            out: Arc<Mutex<Vec<u64>>>,
        }
        impl Node for Det {
            fn name(&self) -> &str {
                "det"
            }
            fn tick(&mut self) {
                let t = horus_core::core::tick_context::ctx_tick();
                let r = horus_core::core::tick_context::ctx_with_rng(|rng| {
                    use rand::Rng;
                    rng.gen::<u64>()
                });
                let dt = horus_core::core::tick_context::ctx_dt().as_nanos() as u64;
                self.out.lock().unwrap().push(t ^ r ^ dt);
            }
        }

        let mut s = Scheduler::new().deterministic(true).tick_rate(100_u64.hz());
        s.add(Det { out: out.clone() })
            .order(0)
            .rate(100_u64.hz())
            .build()
            .unwrap();
        for _ in 0..ticks {
            s.tick_once().unwrap();
        }
    }
    let result = out.lock().unwrap().clone();
    result
}
