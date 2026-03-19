//! Combined Scheduler + IPC Latency Benchmarks
//!
//! Measures Topic send→recv latency while a Scheduler is actively running
//! multiple nodes. Answers the question: does scheduling overhead degrade
//! IPC performance under production load?
//!
//! Run with: cargo bench -p horus_benchmarks -- scheduler_ipc

use criterion::{criterion_group, criterion_main, BenchmarkId, Criterion};
use horus_core::communication::Topic;
use horus_core::core::{DurationExt, Node};
use horus_core::scheduling::Scheduler;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::Arc;
use std::time::Instant;

// ============================================================================
// Load Nodes (background work to simulate production)
// ============================================================================

/// A node that does configurable CPU work per tick (simulates real node computation).
struct LoadNode {
    name: String,
    work_iterations: u64,
    counter: Arc<AtomicU64>,
}

impl LoadNode {
    fn new(name: &str, work_iterations: u64) -> Self {
        Self {
            name: name.to_string(),
            work_iterations,
            counter: Arc::new(AtomicU64::new(0)),
        }
    }
}

impl Node for LoadNode {
    fn name(&self) -> &str {
        &self.name
    }

    fn tick(&mut self) {
        // Simulate computation (prevent optimizer from removing)
        let mut x = 1u64;
        for _ in 0..self.work_iterations {
            x = x.wrapping_mul(6364136223846793005).wrapping_add(1);
        }
        self.counter.store(x, Ordering::Relaxed); // prevent dead code elimination
    }
}

fn unique(prefix: &str) -> String {
    static COUNTER: AtomicU64 = AtomicU64::new(0);
    format!(
        "{}_{}_{}",
        prefix,
        std::process::id(),
        COUNTER.fetch_add(1, Ordering::Relaxed)
    )
}

fn cleanup_shm() {
    let _ = std::fs::remove_dir_all(horus_core::memory::shm_topics_dir());
    // shm_nodes_dir not re-exported; use base_dir + "nodes"
    let mut nodes = horus_core::memory::shm_base_dir();
    nodes.push("nodes");
    let _ = std::fs::remove_dir_all(nodes);
}

// ============================================================================
// Benchmarks
// ============================================================================

/// Benchmark IPC latency with varying number of background scheduler nodes.
/// Measures: Topic<u64> send→recv roundtrip while scheduler runs N load nodes.
fn bench_latency_by_node_count(c: &mut Criterion) {
    let mut group = c.benchmark_group("scheduler_ipc_latency_by_nodes");
    group.measurement_time(5_u64.secs());
    group.sample_size(10);

    for node_count in [0, 1, 5, 10, 20] {
        group.bench_with_input(
            BenchmarkId::new("nodes", node_count),
            &node_count,
            |b, &n| {
                cleanup_shm();

                let measurement_topic = unique("bench_measure");

                // Create measurement topics BEFORE scheduler starts
                let pub_topic: Topic<u64> =
                    Topic::new(&measurement_topic).expect("pub topic");
                let sub_topic: Topic<u64> =
                    Topic::new(&measurement_topic).expect("sub topic");

                // Start scheduler with N load nodes in background
                let running = Arc::new(AtomicBool::new(true));
                let sched_thread = if n > 0 {
                    let mut scheduler = Scheduler::new().tick_rate(1000_u64.hz());
                    for i in 0..n {
                        let name = format!("load_{i}");
                        let name_static: &'static str = Box::leak(name.into_boxed_str());
                        scheduler
                            .add(LoadNode::new(name_static, 100))
                            .order(i as u32)
                            .build();
                    }

                    Some(std::thread::spawn(move || {
                        let _ = scheduler.run_for(8_u64.secs()); // runs until test completes
                    }))
                } else {
                    None
                };

                // Wait for scheduler to start
                std::thread::sleep(100_u64.ms());

                // Benchmark: measure send→recv roundtrip
                b.iter(|| {
                    let start = Instant::now();
                    pub_topic.send(42u64);
                    // Spin-wait for message (tight loop)
                    loop {
                        if let Some(val) = sub_topic.recv() {
                            assert_eq!(val, 42);
                            break;
                        }
                    }
                    start.elapsed().as_nanos()
                });

                // Cleanup
                running.store(false, Ordering::SeqCst);
                if let Some(handle) = sched_thread {
                    // Scheduler will stop on its own via run_for timeout
                    let _ = handle.join();
                }
            },
        );
    }

    group.finish();
}

/// Benchmark IPC latency with varying number of concurrent topics.
/// Measures: latency on the "measurement" topic while N other topics
/// have active publishers generating traffic.
fn bench_latency_by_topic_count(c: &mut Criterion) {
    let mut group = c.benchmark_group("scheduler_ipc_latency_by_topics");
    group.measurement_time(5_u64.secs());
    group.sample_size(10);

    for topic_count in [0, 1, 5, 10] {
        group.bench_with_input(
            BenchmarkId::new("topics", topic_count),
            &topic_count,
            |b, &n| {
                cleanup_shm();

                let measurement_topic = unique("bench_measure_t");

                // Create measurement topics
                let pub_topic: Topic<u64> =
                    Topic::new(&measurement_topic).expect("pub topic");
                let sub_topic: Topic<u64> =
                    Topic::new(&measurement_topic).expect("sub topic");

                // Create N background traffic topics with publisher threads
                let running = Arc::new(AtomicBool::new(true));
                let mut traffic_threads = Vec::new();

                for i in 0..n {
                    let topic_name = unique(&format!("traffic_{i}"));
                    let r = running.clone();
                    traffic_threads.push(std::thread::spawn(move || {
                        let t: Topic<u64> = Topic::new(&topic_name).expect("traffic topic");
                        let mut counter = 0u64;
                        while r.load(Ordering::Relaxed) {
                            t.send(counter);
                            counter += 1;
                            // Publish at ~10kHz
                            std::thread::sleep(std::time::Duration::from_micros(100));
                        }
                    }));
                }

                std::thread::sleep(50_u64.ms());

                // Benchmark: measure send→recv on the measurement topic
                b.iter(|| {
                    let start = Instant::now();
                    pub_topic.send(42u64);
                    loop {
                        if let Some(val) = sub_topic.recv() {
                            assert_eq!(val, 42);
                            break;
                        }
                    }
                    start.elapsed().as_nanos()
                });

                // Cleanup
                running.store(false, Ordering::SeqCst);
                for handle in traffic_threads {
                    let _ = handle.join();
                }
            },
        );
    }

    group.finish();
}

/// Benchmark IPC latency with a full production-like setup:
/// 10 nodes at 1kHz + 5 active topics with traffic.
fn bench_production_load(c: &mut Criterion) {
    let mut group = c.benchmark_group("scheduler_ipc_production_load");
    group.measurement_time(5_u64.secs());
    group.sample_size(10);

    group.bench_function("10_nodes_1khz_5_topics", |b| {
        cleanup_shm();

        let measurement_topic = unique("bench_prod_measure");

        let pub_topic: Topic<u64> =
            Topic::new(&measurement_topic).expect("pub topic");
        let sub_topic: Topic<u64> =
            Topic::new(&measurement_topic).expect("sub topic");

        // Start scheduler with 10 nodes
        let sched_thread = {
            let mut scheduler = Scheduler::new().tick_rate(1000_u64.hz());
            for i in 0..10 {
                let name = format!("prod_node_{i}");
                let name_static: &'static str = Box::leak(name.into_boxed_str());
                scheduler
                    .add(LoadNode::new(name_static, 200))
                    .order(i)
                    .build();
            }
            std::thread::spawn(move || {
                let _ = scheduler.run_for(8_u64.secs());
            })
        };

        // Start 5 background traffic topics
        let running = Arc::new(AtomicBool::new(true));
        let mut traffic_threads = Vec::new();
        for i in 0..5 {
            let topic_name = unique(&format!("prod_traffic_{i}"));
            let r = running.clone();
            traffic_threads.push(std::thread::spawn(move || {
                let t: Topic<u64> = Topic::new(&topic_name).expect("traffic topic");
                let mut counter = 0u64;
                while r.load(Ordering::Relaxed) {
                    t.send(counter);
                    counter += 1;
                    std::thread::sleep(std::time::Duration::from_micros(100));
                }
            }));
        }

        std::thread::sleep(200_u64.ms());

        b.iter(|| {
            let start = Instant::now();
            pub_topic.send(42u64);
            loop {
                if let Some(val) = sub_topic.recv() {
                    assert_eq!(val, 42);
                    break;
                }
            }
            start.elapsed().as_nanos()
        });

        running.store(false, Ordering::SeqCst);
        for handle in traffic_threads {
            let _ = handle.join();
        }
        let _ = sched_thread.join();
    });

    group.finish();
}

criterion_group!(
    benches,
    bench_latency_by_node_count,
    bench_latency_by_topic_count,
    bench_production_load,
);
criterion_main!(benches);
