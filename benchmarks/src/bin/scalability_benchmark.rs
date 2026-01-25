//! Scalability Benchmark
//!
//! Measures how HORUS IPC performance scales with:
//! - Number of producer threads
//! - Number of consumer threads
//! - Total core count utilization
//!
//! ## Methodology
//!
//! Tests MPMC backends with varying thread counts to produce
//! scalability curves suitable for research publications.
//!
//! ## Running
//!
//! ```bash
//! cargo run --release --bin scalability_benchmark
//! cargo run --release --bin scalability_benchmark -- --max-threads 16
//! cargo run --release --bin scalability_benchmark -- --json results.json
//! ```

use horus::prelude::Topic;
use horus_benchmarks::{
    detect_platform, set_cpu_affinity, write_json_report, BenchmarkConfig,
    BenchmarkReport, BenchmarkResult, DeterminismMetrics, Statistics, ThroughputMetrics,
};
use serde::{Deserialize, Serialize};
use serde_arrays;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::Arc;
use std::thread;
use std::time::{Duration, Instant};

const MEASUREMENT_DURATION_SECS: u64 = 5;
const WARMUP_DURATION_SECS: u64 = 1;

/// Benchmark payload (64 bytes total)
#[derive(Clone, Copy, Debug, Serialize, Deserialize)]
struct ScalabilityMsg {
    producer_id: u32,
    seq: u64,
    timestamp: u64,
    // Use multiple smaller arrays for serde compatibility
    #[serde(with = "serde_arrays")]
    padding1: [u8; 32],
    #[serde(with = "serde_arrays")]
    padding2: [u8; 16],
}

impl Default for ScalabilityMsg {
    fn default() -> Self {
        Self {
            producer_id: 0,
            seq: 0,
            timestamp: 0,
            padding1: [0u8; 32],
            padding2: [0u8; 16],
        }
    }
}

impl horus_core::core::LogSummary for ScalabilityMsg {
    fn log_summary(&self) -> String {
        format!("ScalabilityMsg(p={}, seq={})", self.producer_id, self.seq)
    }
}

fn main() {
    let args: Vec<String> = std::env::args().collect();

    // Parse arguments
    let mut json_output: Option<String> = None;
    let max_threads = num_cpus::get();

    let mut i = 1;
    while i < args.len() {
        match args[i].as_str() {
            "--json" => {
                json_output = args.get(i + 1).cloned();
                i += 2;
            }
            _ => {
                i += 1;
            }
        }
    }

    println!("╔══════════════════════════════════════════════════════════════════╗");
    println!("║           HORUS Scalability Analysis                             ║");
    println!("╠══════════════════════════════════════════════════════════════════╣");
    println!("║  Measuring throughput scaling with thread count                  ║");
    println!("╚══════════════════════════════════════════════════════════════════╝");
    println!();

    // Detect platform
    let platform = detect_platform();
    println!("Platform: {} ({} logical cores)", platform.cpu.model, platform.cpu.logical_cores);
    println!("Max threads to test: {}", max_threads);
    println!();

    let mut report = BenchmarkReport::new(platform.clone());

    // Test configurations: (producers, consumers)
    let configs: Vec<(usize, usize)> = vec![
        (1, 1),   // SPSC baseline
        (2, 1),   // 2 producers
        (4, 1),   // 4 producers
        (1, 2),   // 2 consumers
        (1, 4),   // 4 consumers
        (2, 2),   // 2x2 balanced
        (4, 4),   // 4x4 balanced
        (8, 8),   // 8x8 if available
    ]
    .into_iter()
    .filter(|(p, c)| p + c <= max_threads)
    .collect();

    println!("╔═══════════════════════════════════════════════════════════════════════════════╗");
    println!("║ Producers │ Consumers │ Throughput (M msg/s) │ Per-Thread │ Scaling Efficiency ║");
    println!("╠═══════════════════════════════════════════════════════════════════════════════╣");

    let baseline_throughput = Arc::new(AtomicU64::new(0));

    for (num_producers, num_consumers) in &configs {
        let result = run_scalability_test(
            *num_producers,
            *num_consumers,
            &platform,
            baseline_throughput.clone(),
        );

        // Print row
        let throughput_m = result.throughput.messages_per_sec / 1_000_000.0;
        let per_thread = throughput_m / (*num_producers + *num_consumers) as f64;

        let efficiency = if *num_producers == 1 && *num_consumers == 1 {
            baseline_throughput.store(
                (result.throughput.messages_per_sec * 1000.0) as u64,
                Ordering::Relaxed,
            );
            100.0
        } else {
            let base = baseline_throughput.load(Ordering::Relaxed) as f64 / 1000.0;
            let expected = base * (*num_producers.min(num_consumers)) as f64;
            (result.throughput.messages_per_sec / expected) * 100.0
        };

        println!(
            "║    {:>2}      │     {:>2}     │       {:>8.2}       │   {:>6.2}   │       {:>6.1}%       ║",
            num_producers, num_consumers, throughput_m, per_thread, efficiency
        );

        report.add_result(result);
    }

    println!("╚═══════════════════════════════════════════════════════════════════════════════╝");

    // Producer scaling curve
    println!("\n╔══════════════════════════════════════════════════════════════════╗");
    println!("║              Producer Scaling (1 Consumer)                       ║");
    println!("╠══════════════════════════════════════════════════════════════════╣");

    let producer_counts: Vec<usize> = (1..=max_threads.min(8)).collect();
    for &num_producers in &producer_counts {
        let result = run_scalability_test(num_producers, 1, &platform, baseline_throughput.clone());
        let throughput_m = result.throughput.messages_per_sec / 1_000_000.0;
        let bar_len = ((throughput_m / 10.0) * 40.0).min(40.0) as usize;
        let bar: String = "█".repeat(bar_len);

        println!(
            "║ {:>2} producers: {:>6.2} M/s │{}",
            num_producers, throughput_m, bar
        );

        report.add_result(result);
    }
    println!("╚══════════════════════════════════════════════════════════════════╝");

    // Consumer scaling curve
    println!("\n╔══════════════════════════════════════════════════════════════════╗");
    println!("║              Consumer Scaling (1 Producer)                       ║");
    println!("╠══════════════════════════════════════════════════════════════════╣");

    let consumer_counts: Vec<usize> = (1..=max_threads.min(8)).collect();
    for &num_consumers in &consumer_counts {
        let result = run_scalability_test(1, num_consumers, &platform, baseline_throughput.clone());
        let throughput_m = result.throughput.messages_per_sec / 1_000_000.0;
        let bar_len = ((throughput_m / 10.0) * 40.0).min(40.0) as usize;
        let bar: String = "█".repeat(bar_len);

        println!(
            "║ {:>2} consumers: {:>6.2} M/s │{}",
            num_consumers, throughput_m, bar
        );

        report.add_result(result);
    }
    println!("╚══════════════════════════════════════════════════════════════════╝");

    // Write JSON output if requested
    if let Some(path) = json_output {
        match write_json_report(&report, &path) {
            Ok(_) => println!("\nResults written to: {}", path),
            Err(e) => eprintln!("\nFailed to write JSON: {}", e),
        }
    }
}

fn run_scalability_test(
    num_producers: usize,
    num_consumers: usize,
    platform: &horus_benchmarks::PlatformInfo,
    _baseline: Arc<AtomicU64>,
) -> BenchmarkResult {
    let topic_name = format!(
        "scale_p{}_c{}_{}_{}",
        num_producers,
        num_consumers,
        std::process::id(),
        rand::random::<u32>()
    );

    // Shared state
    let running = Arc::new(AtomicBool::new(false));
    let total_sent = Arc::new(AtomicU64::new(0));
    let total_received = Arc::new(AtomicU64::new(0));

    // Create topics
    let mut producer_handles = Vec::new();
    let mut consumer_handles = Vec::new();

    // Spawn producers
    for producer_id in 0..num_producers {
        let topic_name = topic_name.clone();
        let running = running.clone();
        let total_sent = total_sent.clone();

        let handle = thread::spawn(move || {
            // Pin to CPU if possible
            let _ = set_cpu_affinity(producer_id % num_cpus::get());

            let producer: Topic<ScalabilityMsg> = Topic::new(&topic_name).unwrap();

            // Wait for start signal
            while !running.load(Ordering::Acquire) {
                thread::yield_now();
            }

            let mut seq = 0u64;
            let mut local_sent = 0u64;

            while running.load(Ordering::Acquire) {
                let msg = ScalabilityMsg {
                    producer_id: producer_id as u32,
                    seq,
                    timestamp: 0,
                    padding1: [0; 32],
                    padding2: [0; 16],
                };

                if producer.send(msg).is_ok() {
                    local_sent += 1;
                    seq += 1;
                }
            }

            total_sent.fetch_add(local_sent, Ordering::Relaxed);
        });

        producer_handles.push(handle);
    }

    // Spawn consumers
    for consumer_id in 0..num_consumers {
        let topic_name = topic_name.clone();
        let running = running.clone();
        let total_received = total_received.clone();

        let handle = thread::spawn(move || {
            // Pin to CPU if possible (offset from producers)
            let _ = set_cpu_affinity((num_producers + consumer_id) % num_cpus::get());

            let consumer: Topic<ScalabilityMsg> = Topic::new(&topic_name).unwrap();

            // Wait for start signal
            while !running.load(Ordering::Acquire) {
                thread::yield_now();
            }

            let mut local_received = 0u64;

            while running.load(Ordering::Acquire) {
                if consumer.recv().is_some() {
                    local_received += 1;
                }
            }

            // Drain remaining messages
            while consumer.recv().is_some() {
                local_received += 1;
            }

            total_received.fetch_add(local_received, Ordering::Relaxed);
        });

        consumer_handles.push(handle);
    }

    // Give threads time to initialize
    thread::sleep(Duration::from_millis(100));

    // Warmup phase
    running.store(true, Ordering::Release);
    thread::sleep(Duration::from_secs(WARMUP_DURATION_SECS));

    // Reset counters for measurement
    total_sent.store(0, Ordering::Relaxed);
    total_received.store(0, Ordering::Relaxed);

    // Measurement phase
    let start = Instant::now();
    thread::sleep(Duration::from_secs(MEASUREMENT_DURATION_SECS));
    let duration = start.elapsed();

    // Stop
    running.store(false, Ordering::Release);

    // Wait for threads to finish
    for handle in producer_handles {
        handle.join().ok();
    }
    for handle in consumer_handles {
        handle.join().ok();
    }

    // Calculate throughput
    let _sent = total_sent.load(Ordering::Relaxed);
    let received = total_received.load(Ordering::Relaxed);
    let duration_secs = duration.as_secs_f64();

    let messages_per_sec = received as f64 / duration_secs;
    let bytes_per_sec = messages_per_sec * std::mem::size_of::<ScalabilityMsg>() as f64;

    // Build result
    let config = BenchmarkConfig {
        warmup_iterations: 0,
        iterations: received as usize,
        runs: 1,
        cpu_affinity: None,
        filter_outliers: false,
        confidence_level: 95.0,
    };

    let statistics = Statistics {
        count: received as usize,
        mean: 0.0,
        median: 0.0,
        std_dev: 0.0,
        min: 0,
        max: 0,
        p1: 0,
        p5: 0,
        p25: 0,
        p75: 0,
        p95: 0,
        p99: 0,
        p999: 0,
        p9999: 0,
        ci_low: 0.0,
        ci_high: 0.0,
        confidence_level: 95.0,
        outliers_removed: 0,
    };

    let throughput = ThroughputMetrics {
        messages_per_sec,
        bytes_per_sec,
        total_messages: received,
        total_bytes: received * std::mem::size_of::<ScalabilityMsg>() as u64,
        duration_secs,
    };

    let determinism = DeterminismMetrics {
        cv: 0.0,
        max_jitter_ns: 0,
        p999: 0,
        p9999: 0,
        deadline_misses: 0,
        deadline_threshold_ns: 0,
        run_variance: 0.0,
    };

    BenchmarkResult {
        name: format!("scalability_p{}_c{}", num_producers, num_consumers),
        subject: "HORUS MpmcShm".to_string(),
        message_size: std::mem::size_of::<ScalabilityMsg>(),
        config,
        platform: platform.clone(),
        timestamp: chrono::Utc::now().to_rfc3339(),
        raw_latencies_ns: vec![],
        statistics,
        throughput,
        determinism,
    }
}
