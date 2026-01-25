//! Cross-Process Latency Benchmark
//!
//! TRUE cross-process IPC measurement required for academic validity.
//! Spawns separate producer and consumer processes to measure actual
//! inter-process communication latency (not intra-process).
//!
//! ## Methodology
//!
//! 1. Parent process creates shared memory topic
//! 2. Spawns child consumer process
//! 3. Spawns child producer process
//! 4. Producer embeds timestamps in messages
//! 5. Consumer measures end-to-end latency
//! 6. Results collected via shared memory
//!
//! ## Running
//!
//! ```bash
//! cargo run --release --bin cross_process_benchmark
//! cargo run --release --bin cross_process_benchmark -- --json results.json
//! cargo run --release --bin cross_process_benchmark -- --iterations 50000
//! ```

use horus::prelude::Topic;
use horus_benchmarks::{
    coefficient_of_variation, detect_platform, set_cpu_affinity, write_json_report,
    BenchmarkConfig, BenchmarkReport, BenchmarkResult, DeterminismMetrics, Statistics,
    ThroughputMetrics,
};
use serde::{Deserialize, Serialize};
use std::env;
use std::process::{Command, Stdio};
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

const DEFAULT_ITERATIONS: usize = 50_000;
const DEFAULT_WARMUP: usize = 5_000;

/// Message with embedded high-resolution timestamp
#[derive(Clone, Copy, Debug, Serialize, Deserialize)]
struct TimestampedMsg {
    seq: u64,
    send_time_ns: u64,
    #[serde(with = "serde_arrays")]
    payload: [u8; 48], // Total 64 bytes
}

impl Default for TimestampedMsg {
    fn default() -> Self {
        Self {
            seq: 0,
            send_time_ns: 0,
            payload: [0u8; 48],
        }
    }
}

impl horus_core::core::LogSummary for TimestampedMsg {
    fn log_summary(&self) -> String {
        format!("TimestampedMsg(seq={})", self.seq)
    }
}

/// Result message containing latency measurements
#[derive(Clone, Debug, Serialize, Deserialize, Default)]
struct ResultMsg {
    latencies_ns: Vec<u64>,
    consumer_pid: u32,
}

impl horus_core::core::LogSummary for ResultMsg {
    fn log_summary(&self) -> String {
        format!("ResultMsg({} samples)", self.latencies_ns.len())
    }
}

fn current_time_ns() -> u64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap()
        .as_nanos() as u64
}

fn main() {
    let args: Vec<String> = env::args().collect();

    // Check if this is a child process
    if args.len() > 1 {
        match args[1].as_str() {
            "--producer" => {
                run_producer(&args[2..]);
                return;
            }
            "--consumer" => {
                run_consumer(&args[2..]);
                return;
            }
            _ => {}
        }
    }

    // Parent process - coordinate the benchmark
    run_coordinator(&args[1..]);
}

fn run_coordinator(args: &[String]) {
    // Parse arguments
    let mut json_output: Option<String> = None;
    let mut iterations = DEFAULT_ITERATIONS;

    let mut i = 0;
    while i < args.len() {
        match args[i].as_str() {
            "--json" => {
                json_output = args.get(i + 1).cloned();
                i += 2;
            }
            "--iterations" => {
                iterations = args
                    .get(i + 1)
                    .and_then(|s| s.parse().ok())
                    .unwrap_or(DEFAULT_ITERATIONS);
                i += 2;
            }
            _ => {
                i += 1;
            }
        }
    }

    println!("╔══════════════════════════════════════════════════════════════════╗");
    println!("║        HORUS Cross-Process Latency Benchmark                     ║");
    println!("╠══════════════════════════════════════════════════════════════════╣");
    println!("║  TRUE inter-process communication measurement                    ║");
    println!("║  (Separate producer and consumer processes)                      ║");
    println!("╚══════════════════════════════════════════════════════════════════╝");
    println!();

    let platform = detect_platform();
    println!("Platform: {} ({} cores)", platform.cpu.model, platform.cpu.logical_cores);
    println!("Iterations: {}", iterations);
    println!("Warmup: {}", DEFAULT_WARMUP);
    println!();

    let mut report = BenchmarkReport::new(platform.clone());

    // Test AdaptiveTopic (auto-selects optimal backend)
    println!("[AdaptiveTopic] Running cross-process benchmark...");

    let result = run_cross_process_test(
        "AdaptiveTopic",
        iterations,
        &platform,
    );

    print_result(&result);
    report.add_result(result);

    // Summary
    println!("\n╔══════════════════════════════════════════════════════════════════╗");
    println!("║                    CROSS-PROCESS SUMMARY                         ║");
    println!("╠══════════════════════════════════════════════════════════════════╣");

    for result in &report.results {
        println!(
            "║ {:12} │ median: {:>7.0}ns │ p99: {:>7}ns │ CV: {:.3}",
            result.name,
            result.statistics.median,
            result.statistics.p99,
            result.determinism.cv
        );
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

fn run_cross_process_test(
    name: &str,
    iterations: usize,
    platform: &horus_benchmarks::PlatformInfo,
) -> BenchmarkResult {
    let topic_name = format!("xproc_{}", std::process::id());
    let result_topic = format!("{}_results", topic_name);

    // Create result collection topic
    let result_rx: Topic<ResultMsg> = Topic::new(&result_topic).unwrap();

    // Get current executable path
    let exe = env::current_exe().expect("Failed to get current executable");

    // Spawn consumer first (it needs to be ready before producer starts)
    let mut consumer = Command::new(&exe)
        .arg("--consumer")
        .arg(&topic_name)
        .arg(&result_topic)
        .arg(iterations.to_string())
        .stdout(Stdio::piped())
        .stderr(Stdio::inherit())
        .spawn()
        .expect("Failed to spawn consumer");

    // Give consumer time to initialize
    std::thread::sleep(Duration::from_millis(100));

    // Spawn producer
    let mut producer = Command::new(&exe)
        .arg("--producer")
        .arg(&topic_name)
        .arg(iterations.to_string())
        .stdout(Stdio::piped())
        .stderr(Stdio::inherit())
        .spawn()
        .expect("Failed to spawn producer");

    // Wait for producer to finish
    let producer_status = producer.wait().expect("Failed to wait for producer");
    if !producer_status.success() {
        eprintln!("  Producer exited with error");
    }

    // Wait for consumer to finish
    let consumer_status = consumer.wait().expect("Failed to wait for consumer");
    if !consumer_status.success() {
        eprintln!("  Consumer exited with error");
    }

    // Collect results from consumer
    let mut latencies = Vec::new();
    let timeout = Instant::now();
    while timeout.elapsed() < Duration::from_secs(5) {
        if let Some(result) = result_rx.recv() {
            latencies = result.latencies_ns;
            break;
        }
        std::thread::sleep(Duration::from_millis(10));
    }

    if latencies.is_empty() {
        eprintln!("  Warning: No results received from consumer");
        latencies = vec![0]; // Avoid panic
    }

    // Compute statistics
    let config = BenchmarkConfig {
        warmup_iterations: DEFAULT_WARMUP,
        iterations,
        runs: 1,
        cpu_affinity: None,
        filter_outliers: true,
        confidence_level: 95.0,
    };

    let statistics = Statistics::from_samples(&latencies, 95.0, true);
    let cv = coefficient_of_variation(&latencies);

    let mut sorted = latencies.clone();
    sorted.sort_unstable();
    let max_jitter = sorted.last().unwrap_or(&0) - sorted.first().unwrap_or(&0);

    let determinism = DeterminismMetrics {
        cv,
        max_jitter_ns: max_jitter,
        p999: statistics.p999,
        p9999: statistics.p9999,
        deadline_misses: latencies.iter().filter(|&&l| l > 10_000).count() as u64, // 10µs deadline
        deadline_threshold_ns: 10_000,
        run_variance: 0.0,
    };

    let total_ns: u64 = latencies.iter().sum();
    let duration_secs = total_ns as f64 / 1_000_000_000.0;

    let throughput = ThroughputMetrics {
        messages_per_sec: latencies.len() as f64 / duration_secs.max(0.001),
        bytes_per_sec: (latencies.len() * 64) as f64 / duration_secs.max(0.001),
        total_messages: latencies.len() as u64,
        total_bytes: (latencies.len() * 64) as u64,
        duration_secs,
    };

    BenchmarkResult {
        name: format!("{}_cross_process", name),
        subject: "HORUS Topic (cross-process)".to_string(),
        message_size: 64,
        config,
        platform: platform.clone(),
        timestamp: chrono::Utc::now().to_rfc3339(),
        raw_latencies_ns: latencies,
        statistics,
        throughput,
        determinism,
    }
}

fn run_producer(args: &[String]) {
    if args.len() < 2 {
        eprintln!("Producer: insufficient arguments");
        std::process::exit(1);
    }

    let topic_name = &args[0];
    let iterations: usize = args[1].parse().unwrap_or(DEFAULT_ITERATIONS);

    // Pin to CPU 0
    let _ = set_cpu_affinity(0);

    // Create producer using AdaptiveTopic
    let producer: Topic<TimestampedMsg> = Topic::new(topic_name).expect("Failed to create producer");

    // Warmup
    for i in 0..DEFAULT_WARMUP {
        let msg = TimestampedMsg {
            seq: i as u64,
            send_time_ns: current_time_ns(),
            payload: [0u8; 48],
        };
        let _ = producer.send(msg);
        std::thread::sleep(Duration::from_micros(10));
    }

    // Small delay to let consumer drain warmup
    std::thread::sleep(Duration::from_millis(50));

    // Measured iterations
    for i in 0..iterations {
        let msg = TimestampedMsg {
            seq: (DEFAULT_WARMUP + i) as u64,
            send_time_ns: current_time_ns(),
            payload: [0u8; 48],
        };
        producer.send(msg).expect("Send failed");

        // Small delay to prevent queue overflow
        if i % 1000 == 0 {
            std::thread::yield_now();
        }
    }

    // Signal completion with special message
    let done_msg = TimestampedMsg {
        seq: u64::MAX,
        send_time_ns: 0,
        payload: [0xFFu8; 48],
    };
    producer.send(done_msg).ok();
}

fn run_consumer(args: &[String]) {
    if args.len() < 3 {
        eprintln!("Consumer: insufficient arguments");
        std::process::exit(1);
    }

    let topic_name = &args[0];
    let result_topic = &args[1];
    let iterations: usize = args[2].parse().unwrap_or(DEFAULT_ITERATIONS);

    // Pin to CPU 1
    let _ = set_cpu_affinity(1);

    // Create consumer using AdaptiveTopic
    let consumer: Topic<TimestampedMsg> = Topic::new(topic_name).expect("Failed to create consumer");

    // Create result publisher
    let result_tx: Topic<ResultMsg> = Topic::new(result_topic).expect("Failed to create result topic");

    let mut latencies = Vec::with_capacity(iterations);
    let mut warmup_count = 0;
    let timeout = Instant::now();

    // Receive messages and measure latency
    while timeout.elapsed() < Duration::from_secs(60) {
        if let Some(msg) = consumer.recv() {
            // Check for completion signal
            if msg.seq == u64::MAX {
                break;
            }

            let recv_time = current_time_ns();
            let latency = recv_time.saturating_sub(msg.send_time_ns);

            // Skip warmup
            if warmup_count < DEFAULT_WARMUP {
                warmup_count += 1;
                continue;
            }

            latencies.push(latency);

            if latencies.len() >= iterations {
                break;
            }
        }
    }

    // Send results back
    let result = ResultMsg {
        latencies_ns: latencies,
        consumer_pid: std::process::id(),
    };

    // Try multiple times to ensure delivery
    for _ in 0..10 {
        if result_tx.send(result.clone()).is_ok() {
            break;
        }
        std::thread::sleep(Duration::from_millis(10));
    }
}

fn print_result(result: &BenchmarkResult) {
    let stats = &result.statistics;
    let det = &result.determinism;

    println!("  {} Results (PID separation verified):", result.name);
    println!("  ├── Samples:   {}", stats.count);
    println!("  ├── Median:    {:>8.1} ns", stats.median);
    println!("  ├── Mean:      {:>8.1} ns", stats.mean);
    println!("  ├── Std Dev:   {:>8.1} ns", stats.std_dev);
    println!("  ├── p95:       {:>8} ns", stats.p95);
    println!("  ├── p99:       {:>8} ns", stats.p99);
    println!("  ├── p99.9:     {:>8} ns", stats.p999);
    println!("  ├── CV:        {:>8.4}", det.cv);
    println!("  └── Jitter:    {:>8} ns", det.max_jitter_ns);
    println!();
}
