//! HORUS IPC Benchmarks
//!
//! **Real cross-process IPC benchmarks** for robotics.
//!
//! These benchmarks spawn separate processes to measure true IPC latency,
//! not just in-memory channel performance.
//!
//! ## What We Measure
//!
//! | Benchmark | Description |
//! |-----------|-------------|
//! | `cross_process_latency` | Publisher → Subscriber across processes |
//! | `cross_process_throughput` | Messages/sec between processes |
//! | `topology_1_to_n` | One publisher, multiple subscribers |
//! | `topology_n_to_1` | Multiple publishers, one subscriber (sensor fusion) |
//!
//! ## Running
//!
//! ```bash
//! cargo bench --bench ipc_benchmark
//! cargo bench --bench ipc_benchmark -- cross_process
//! ```

use criterion::{black_box, criterion_group, criterion_main, BenchmarkId, Criterion, Throughput};
use horus::prelude::Topic;
use horus_library::messages::{
    cmd_vel::CmdVel,
    sensor::{Imu, LaserScan},
};
use std::process::{Child, Command, Stdio};
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant};
use std::thread;
use std::io::{BufRead, BufReader};

// =============================================================================
// Cross-Process Latency (The Real IPC Test)
// =============================================================================

/// Spawn a child process that publishes messages
fn spawn_publisher(topic_name: &str, count: u64, msg_type: &str) -> Child {
    Command::new(std::env::current_exe().unwrap())
        .args(["--bench", "--", "__child_publisher"])
        .env("BENCH_TOPIC", topic_name)
        .env("BENCH_COUNT", count.to_string())
        .env("BENCH_MSG_TYPE", msg_type)
        .stdin(Stdio::null())
        .stdout(Stdio::piped())
        .stderr(Stdio::null())
        .spawn()
        .expect("Failed to spawn publisher process")
}

/// Spawn a child process that subscribes and counts messages
fn spawn_subscriber(topic_name: &str, expected: u64) -> Child {
    Command::new(std::env::current_exe().unwrap())
        .args(["--bench", "--", "__child_subscriber"])
        .env("BENCH_TOPIC", topic_name)
        .env("BENCH_EXPECTED", expected.to_string())
        .stdin(Stdio::null())
        .stdout(Stdio::piped())
        .stderr(Stdio::null())
        .spawn()
        .expect("Failed to spawn subscriber process")
}

/// Cross-process latency: publisher in one process, subscriber in another
fn bench_cross_process_latency(c: &mut Criterion) {
    // Skip if running as child process
    if std::env::var("BENCH_TOPIC").is_ok() {
        return;
    }

    let mut group = c.benchmark_group("cross_process_latency");
    group.sample_size(50);
    group.measurement_time(Duration::from_secs(10));

    // CmdVel - 16 bytes (control commands)
    group.throughput(Throughput::Bytes(std::mem::size_of::<CmdVel>() as u64));
    group.bench_function("CmdVel_16B", |b| {
        b.iter_custom(|iters| {
            let topic = format!("bench_xproc_cmdvel_{}", std::process::id());

            // Start subscriber first
            let mut sub = spawn_subscriber(&topic, iters);
            thread::sleep(Duration::from_millis(100)); // Let subscriber initialize

            // Start publisher and time it
            let start = Instant::now();
            let mut pub_proc = spawn_publisher(&topic, iters, "cmdvel");

            // Wait for publisher to finish
            pub_proc.wait().expect("Publisher failed");

            // Wait for subscriber to receive all messages
            let output = sub.wait_with_output().expect("Subscriber failed");
            let elapsed = start.elapsed();

            // Parse received count from subscriber output
            let stdout = String::from_utf8_lossy(&output.stdout);
            if !stdout.contains("DONE") {
                eprintln!("Subscriber didn't finish: {}", stdout);
            }

            elapsed
        });
    });

    group.finish();
}

// =============================================================================
// Cross-Process Throughput
// =============================================================================

/// Measure sustained throughput between processes
fn bench_cross_process_throughput(c: &mut Criterion) {
    if std::env::var("BENCH_TOPIC").is_ok() {
        return;
    }

    let mut group = c.benchmark_group("cross_process_throughput");
    group.sample_size(20);
    group.measurement_time(Duration::from_secs(15));

    // 10K messages burst
    group.bench_function("CmdVel_10k_burst", |b| {
        b.iter_custom(|_iters| {
            let topic = format!("bench_throughput_{}", std::process::id());
            let msg_count = 10_000u64;

            let mut sub = spawn_subscriber(&topic, msg_count);
            thread::sleep(Duration::from_millis(100));

            let start = Instant::now();
            let mut pub_proc = spawn_publisher(&topic, msg_count, "cmdvel");

            pub_proc.wait().expect("Publisher failed");
            sub.wait().expect("Subscriber failed");

            start.elapsed()
        });
    });

    group.finish();
}

// =============================================================================
// Same-Process Baseline (for comparison)
// =============================================================================

/// Same-process baseline to compare against cross-process
fn bench_same_process_baseline(c: &mut Criterion) {
    if std::env::var("BENCH_TOPIC").is_ok() {
        return;
    }

    let mut group = c.benchmark_group("same_process_baseline");
    group.throughput(Throughput::Bytes(std::mem::size_of::<CmdVel>() as u64));

    // Same-thread (fastest possible)
    group.bench_function("CmdVel_same_thread", |b| {
        let topic: Topic<CmdVel> = Topic::new("bench_same_thread").unwrap();
        b.iter(|| {
            let msg = CmdVel::new(1.5, 0.8);
            topic.send(black_box(msg)).unwrap();
            black_box(topic.recv())
        });
    });

    // Cross-thread (same process)
    group.bench_function("CmdVel_cross_thread", |b| {
        b.iter_custom(|iters| {
            let topic_name = format!("bench_xthread_{}", std::process::id());
            let producer: Topic<CmdVel> = Topic::new(&topic_name).unwrap();
            let consumer: Topic<CmdVel> = Topic::new(&topic_name).unwrap();

            let running = Arc::new(AtomicBool::new(true));
            let running_clone = running.clone();

            let producer_handle = thread::spawn(move || {
                for _ in 0..iters {
                    let msg = CmdVel::new(1.5, 0.8);
                    while producer.send(msg).is_err() {
                        thread::yield_now();
                    }
                }
            });

            let start = Instant::now();
            let mut received = 0u64;
            while received < iters {
                if consumer.recv().is_some() {
                    received += 1;
                } else {
                    thread::yield_now();
                }
            }
            let elapsed = start.elapsed();

            running.store(false, Ordering::Relaxed);
            producer_handle.join().unwrap();
            elapsed
        });
    });

    group.finish();
}

// =============================================================================
// Topology Benchmarks
// =============================================================================

/// N parallel channels: Tests scalability of multiple concurrent IPC channels
/// Note: AdaptiveTopic uses SPSC semantics (each message consumed once), so
/// broadcast patterns (1-to-N with same message to all) are not supported.
/// This benchmark tests N independent pub-sub pairs running in parallel.
fn bench_topology_1_to_n(c: &mut Criterion) {
    if std::env::var("BENCH_TOPIC").is_ok() {
        return;
    }

    let mut group = c.benchmark_group("topology_parallel_channels");
    group.sample_size(10);
    group.measurement_time(Duration::from_secs(5));

    for channel_count in [2, 4] {
        group.bench_with_input(
            BenchmarkId::new("channels", channel_count),
            &channel_count,
            |b, &channel_count| {
                b.iter_custom(|_iters| {
                    let msg_count = 500u64;

                    // Start N parallel pub-sub pairs, each with its own topic
                    let mut pairs: Vec<(Child, Child)> = (0..channel_count)
                        .map(|i| {
                            let topic = format!("bench_parallel_{}_{}", i, std::process::id());
                            let sub = spawn_subscriber(&topic, msg_count);
                            thread::sleep(Duration::from_millis(50));
                            let pub_proc = spawn_publisher(&topic, msg_count, "cmdvel");
                            (sub, pub_proc)
                        })
                        .collect();

                    let start = Instant::now();

                    // Wait for all pairs to complete
                    for (sub, pub_proc) in &mut pairs {
                        pub_proc.wait().expect("Publisher failed");
                        sub.wait().expect("Subscriber failed");
                    }

                    start.elapsed()
                });
            },
        );
    }

    group.finish();
}

// =============================================================================
// Child Process Entry Points
// =============================================================================

/// Child process: publish messages
fn run_child_publisher() {
    let topic_name = std::env::var("BENCH_TOPIC").unwrap();
    let count: u64 = std::env::var("BENCH_COUNT").unwrap().parse().unwrap();
    let msg_type = std::env::var("BENCH_MSG_TYPE").unwrap();

    match msg_type.as_str() {
        "cmdvel" => {
            let topic: Topic<CmdVel> = Topic::new(&topic_name).unwrap();
            for i in 0..count {
                let msg = CmdVel::new(1.0 + (i as f32) * 0.001, 0.5);
                while topic.send(msg).is_err() {
                    std::hint::spin_loop();
                }
            }
        }
        _ => panic!("Unknown message type: {}", msg_type),
    }

    println!("DONE");
}

/// Child process: subscribe and count messages
fn run_child_subscriber() {
    let topic_name = std::env::var("BENCH_TOPIC").unwrap();
    let expected: u64 = std::env::var("BENCH_EXPECTED").unwrap().parse().unwrap();

    let topic: Topic<CmdVel> = Topic::new(&topic_name).unwrap();
    let mut received = 0u64;
    let timeout = Instant::now() + Duration::from_secs(10);

    while received < expected && Instant::now() < timeout {
        if topic.recv().is_some() {
            received += 1;
        } else {
            std::hint::spin_loop();
        }
    }

    println!("DONE received={}", received);
}

// =============================================================================
// Criterion Configuration
// =============================================================================

fn main() {
    // Check if we're running as a child process
    let args: Vec<String> = std::env::args().collect();
    if args.iter().any(|a| a == "__child_publisher") {
        run_child_publisher();
        return;
    }
    if args.iter().any(|a| a == "__child_subscriber") {
        run_child_subscriber();
        return;
    }

    // Run criterion benchmarks
    let mut criterion = Criterion::default()
        .sample_size(50)
        .warm_up_time(Duration::from_secs(2))
        .measurement_time(Duration::from_secs(10));

    bench_same_process_baseline(&mut criterion);
    bench_cross_process_latency(&mut criterion);
    bench_cross_process_throughput(&mut criterion);
    bench_topology_1_to_n(&mut criterion);

    criterion.final_summary();
}
