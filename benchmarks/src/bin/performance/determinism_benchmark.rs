//! Determinism & Jitter Analysis Benchmark
//!
//! Measures real-time determinism characteristics critical for robotics:
//! - Coefficient of variation (CV) - lower is better
//! - Jitter (max - min latency)
//! - Tail latencies (p99, p999, p9999)
//! - Deadline miss rate
//! - Run-to-run variance
//!
//! ## Methodology
//!
//! Uses RDTSC for cycle-accurate timing where available, with proper
//! calibration against system clock. Performs multiple independent runs
//! to measure run-to-run variance.
//!
//! ## Running
//!
//! ```bash
//! cargo run --release --bin determinism_benchmark
//! cargo run --release --bin determinism_benchmark -- --json results.json
//! cargo run --release --bin determinism_benchmark -- --deadline 1000
//! ```

use horus::prelude::Topic;
use horus_benchmarks::{
    coefficient_of_variation, detect_platform, set_cpu_affinity,
    set_performance_governor, timing::PrecisionTimer, write_json_report, BenchmarkConfig,
    BenchmarkReport, BenchmarkResult, DeterminismMetrics, Statistics, ThroughputMetrics,
};
use serde::{Deserialize, Serialize};

const DEFAULT_ITERATIONS: usize = 100_000;
const DEFAULT_WARMUP: usize = 10_000;
const DEFAULT_RUNS: usize = 10;
const DEFAULT_DEADLINE_NS: u64 = 1_000; // 1µs deadline for control commands

/// Benchmark payload
#[derive(Clone, Copy, Debug, Default, Serialize, Deserialize)]
struct ControlCmd {
    linear_x: f32,
    angular_z: f32,
    timestamp: u64,
    seq: u64,
}

impl horus_core::core::LogSummary for ControlCmd {
    fn log_summary(&self) -> String {
        format!("ControlCmd(seq={})", self.seq)
    }
}

fn main() {
    let args: Vec<String> = std::env::args().collect();

    // Parse arguments
    let mut json_output: Option<String> = None;
    let mut deadline_ns = DEFAULT_DEADLINE_NS;
    let mut iterations = DEFAULT_ITERATIONS;
    let mut runs = DEFAULT_RUNS;

    let mut i = 1;
    while i < args.len() {
        match args[i].as_str() {
            "--json" => {
                json_output = args.get(i + 1).cloned();
                i += 2;
            }
            "--deadline" => {
                deadline_ns = args
                    .get(i + 1)
                    .and_then(|s| s.parse().ok())
                    .unwrap_or(DEFAULT_DEADLINE_NS);
                i += 2;
            }
            "--iterations" => {
                iterations = args
                    .get(i + 1)
                    .and_then(|s| s.parse().ok())
                    .unwrap_or(DEFAULT_ITERATIONS);
                i += 2;
            }
            "--runs" => {
                runs = args
                    .get(i + 1)
                    .and_then(|s| s.parse().ok())
                    .unwrap_or(DEFAULT_RUNS);
                i += 2;
            }
            _ => {
                i += 1;
            }
        }
    }

    println!("╔══════════════════════════════════════════════════════════════════╗");
    println!("║        HORUS Determinism & Jitter Analysis                       ║");
    println!("╠══════════════════════════════════════════════════════════════════╣");
    println!("║  Measuring real-time characteristics for robotics applications   ║");
    println!("╚══════════════════════════════════════════════════════════════════╝");
    println!();

    // Try to set performance governor (may require sudo)
    if let Err(e) = set_performance_governor() {
        println!("[WARN] Could not set performance governor: {}", e);
        println!("       For best results, run: sudo cpupower frequency-set -g performance");
        println!();
    }

    // Detect platform
    let platform = detect_platform();
    println!("Platform: {} ({} cores)", platform.cpu.model, platform.cpu.logical_cores);
    if let Some(gov) = &platform.cpu_governor {
        println!("CPU Governor: {}", gov);
    }
    println!();

    // Configuration
    let config = BenchmarkConfig {
        warmup_iterations: DEFAULT_WARMUP,
        iterations,
        runs,
        cpu_affinity: Some((0, 1)),
        filter_outliers: true,
        confidence_level: 95.0,
    };

    println!("Configuration:");
    println!("  Iterations:     {}", config.iterations);
    println!("  Runs:           {}", config.runs);
    println!("  Warmup:         {}", config.warmup_iterations);
    println!("  Deadline:       {} ns ({:.2} µs)", deadline_ns, deadline_ns as f64 / 1000.0);
    println!("  CPU Affinity:   {:?}", config.cpu_affinity);
    println!();

    // Initialize report
    let mut report = BenchmarkReport::new(platform.clone());

    // Run benchmarks
    println!("Running benchmarks...\n");

    // SpscIntra (fastest, same-process)
    let result = run_determinism_benchmark(
        "SpscIntra",
        &config,
        &platform,
        deadline_ns,
        |name| {
            let (tx, rx): (Topic<ControlCmd>, Topic<ControlCmd>) = Topic::spsc_intra(name);
            (tx, rx)
        },
    );
    print_determinism_result(&result);
    report.add_result(result);

    // SpscShm (cross-process SPSC)
    let result = run_determinism_benchmark(
        "SpscShm",
        &config,
        &platform,
        deadline_ns,
        |name| {
            let tx: Topic<ControlCmd> = Topic::producer(name).unwrap();
            let rx: Topic<ControlCmd> = Topic::consumer(name).unwrap();
            (tx, rx)
        },
    );
    print_determinism_result(&result);
    report.add_result(result);

    // MpmcShm (cross-process MPMC)
    let result = run_determinism_benchmark(
        "MpmcShm",
        &config,
        &platform,
        deadline_ns,
        |name| {
            let tx: Topic<ControlCmd> = Topic::new(name).unwrap();
            let rx: Topic<ControlCmd> = Topic::new(name).unwrap();
            (tx, rx)
        },
    );
    print_determinism_result(&result);
    report.add_result(result);

    // Summary
    println!("\n╔══════════════════════════════════════════════════════════════════╗");
    println!("║                        SUMMARY                                   ║");
    println!("╠══════════════════════════════════════════════════════════════════╣");

    for result in &report.results {
        let cv_status = if result.determinism.cv < 0.1 {
            "EXCELLENT"
        } else if result.determinism.cv < 0.2 {
            "GOOD"
        } else if result.determinism.cv < 0.5 {
            "ACCEPTABLE"
        } else {
            "POOR"
        };

        let deadline_status = if result.determinism.deadline_misses == 0 {
            "PASS"
        } else {
            "FAIL"
        };

        println!(
            "║ {:12} CV={:.3} ({:10}) p99={:>6}ns deadlines={:4} [{}]",
            result.name,
            result.determinism.cv,
            cv_status,
            result.statistics.p99,
            result.determinism.deadline_misses,
            deadline_status
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

fn run_determinism_benchmark<F>(
    name: &str,
    config: &BenchmarkConfig,
    platform: &horus_benchmarks::PlatformInfo,
    deadline_ns: u64,
    create_topic: F,
) -> BenchmarkResult
where
    F: Fn(&str) -> (Topic<ControlCmd>, Topic<ControlCmd>),
{
    println!("[{}] Running {} runs of {} iterations each...", name, config.runs, config.iterations);

    // Set CPU affinity if configured
    if let Some((producer_core, _consumer_core)) = config.cpu_affinity {
        if let Err(e) = set_cpu_affinity(producer_core) {
            eprintln!("  Warning: Could not set CPU affinity: {}", e);
        }
    }

    // Calibrate timer
    let timer = PrecisionTimer::new();
    println!("  Timer calibrated: {:.2} MHz", timer.calibration().freq_hz / 1_000_000.0);

    let mut all_latencies: Vec<u64> = Vec::new();
    let mut run_medians: Vec<f64> = Vec::new();

    for run in 0..config.runs {
        let topic_name = format!("det_{}_{}_run{}", name, std::process::id(), run);
        let (tx, rx) = create_topic(&topic_name);

        // Warmup
        for i in 0..config.warmup_iterations {
            let msg = ControlCmd {
                linear_x: 1.0,
                angular_z: 0.5,
                timestamp: 0,
                seq: i as u64,
            };
            tx.send(msg, &mut None).unwrap();
            let _ = rx.recv(&mut None);
        }

        // Measured iterations
        let mut run_latencies = Vec::with_capacity(config.iterations);

        for i in 0..config.iterations {
            let msg = ControlCmd {
                linear_x: 1.0,
                angular_z: 0.5,
                timestamp: 0,
                seq: i as u64,
            };

            let start = timer.start();
            tx.send(msg, &mut None).unwrap();
            let _ = rx.recv(&mut None);
            let elapsed = timer.elapsed_ns(start);

            run_latencies.push(elapsed);
        }

        // Calculate run median
        let mut sorted = run_latencies.clone();
        sorted.sort_unstable();
        let median = if sorted.len() % 2 == 0 {
            (sorted[sorted.len() / 2 - 1] + sorted[sorted.len() / 2]) as f64 / 2.0
        } else {
            sorted[sorted.len() / 2] as f64
        };
        run_medians.push(median);

        all_latencies.extend(run_latencies);

        print!(".");
        use std::io::Write;
        std::io::stdout().flush().ok();
    }
    println!(" done");

    // Compute statistics
    let statistics = Statistics::from_samples(&all_latencies, config.confidence_level, config.filter_outliers);

    // Compute determinism metrics
    let cv = coefficient_of_variation(&all_latencies);
    let mut sorted = all_latencies.clone();
    sorted.sort_unstable();
    let max_jitter = sorted.last().unwrap_or(&0) - sorted.first().unwrap_or(&0);

    let deadline_misses = all_latencies.iter().filter(|&&l| l > deadline_ns).count() as u64;

    // Run-to-run variance
    let run_mean: f64 = run_medians.iter().sum::<f64>() / run_medians.len() as f64;
    let run_variance: f64 = run_medians
        .iter()
        .map(|&m| (m - run_mean).powi(2))
        .sum::<f64>()
        / run_medians.len() as f64;

    let determinism = DeterminismMetrics {
        cv,
        max_jitter_ns: max_jitter,
        p999: statistics.p999,
        p9999: statistics.p9999,
        deadline_misses,
        deadline_threshold_ns: deadline_ns,
        run_variance: run_variance.sqrt() / run_mean, // Normalized run-to-run CV
    };

    // Throughput (from total time)
    let total_ns: u64 = all_latencies.iter().sum();
    let total_messages = all_latencies.len() as u64;
    let duration_secs = total_ns as f64 / 1_000_000_000.0;

    let throughput = ThroughputMetrics {
        messages_per_sec: total_messages as f64 / duration_secs,
        bytes_per_sec: (total_messages * std::mem::size_of::<ControlCmd>() as u64) as f64
            / duration_secs,
        total_messages,
        total_bytes: total_messages * std::mem::size_of::<ControlCmd>() as u64,
        duration_secs,
    };

    BenchmarkResult {
        name: name.to_string(),
        subject: "HORUS Topic".to_string(),
        message_size: std::mem::size_of::<ControlCmd>(),
        config: config.clone(),
        platform: platform.clone(),
        timestamp: chrono::Utc::now().to_rfc3339(),
        raw_latencies_ns: all_latencies,
        statistics,
        throughput,
        determinism,
    }
}

fn print_determinism_result(result: &BenchmarkResult) {
    let det = &result.determinism;
    let stats = &result.statistics;

    println!("\n  {} Results:", result.name);
    println!("  ├── Latency:");
    println!("  │   ├── Mean:    {:>8.1} ns", stats.mean);
    println!("  │   ├── Median:  {:>8.1} ns", stats.median);
    println!("  │   ├── Std Dev: {:>8.1} ns", stats.std_dev);
    println!("  │   ├── Min:     {:>8} ns", stats.min);
    println!("  │   └── Max:     {:>8} ns", stats.max);
    println!("  ├── Percentiles:");
    println!("  │   ├── p95:     {:>8} ns", stats.p95);
    println!("  │   ├── p99:     {:>8} ns", stats.p99);
    println!("  │   ├── p99.9:   {:>8} ns", stats.p999);
    println!("  │   └── p99.99:  {:>8} ns", stats.p9999);
    println!("  ├── Determinism:");
    println!("  │   ├── CV:      {:>8.4} (lower is better)", det.cv);
    println!("  │   ├── Jitter:  {:>8} ns (max-min)", det.max_jitter_ns);
    println!("  │   └── Run CV:  {:>8.4} (run-to-run variance)", det.run_variance);
    println!("  └── Real-Time:");
    println!(
        "      ├── Deadline: {:>6} ns",
        det.deadline_threshold_ns
    );
    println!(
        "      ├── Misses:   {:>6} ({:.4}%)",
        det.deadline_misses,
        det.deadline_misses as f64 / result.raw_latencies_ns.len() as f64 * 100.0
    );
    println!(
        "      └── CI 95%:   [{:.1}, {:.1}] ns",
        stats.ci_low, stats.ci_high
    );
}
