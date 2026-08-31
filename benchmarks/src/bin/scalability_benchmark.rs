//! Scalability Benchmark
//!
//! Measures how HORUS IPC **throughput** scales with:
//! - Number of producer threads
//! - Number of consumer threads
//! - Total core count utilization
//!
//! ## What this measures — and what it does not
//!
//! This binary measures **delivered throughput only**. It collects no latency
//! samples at all, so the `statistics` and `determinism` blocks of every result
//! it emits are empty (`count == 0`, float fields `NaN`) rather than zero. They
//! used to be written as literal zeros alongside a non-zero `count`, which made
//! every chart downstream read "0 ns median, 0 ns p99, 0 ns jitter, 0 deadline
//! misses" — the best possible result — for a run that never timed anything.
//! Use `research_latency` or `all_paths_latency` for latency and jitter.
//!
//! Producers here are unthrottled and consumers drop whatever the ring cannot
//! hold, so throughput alone cannot tell a fast configuration from a lossy one.
//! Every row therefore also reports the **delivery ratio** (received/sent). A
//! configuration that "scales" by dropping 90% of its traffic is visible as a
//! 10% delivery ratio, not as a win.
//!
//! ## Methodology
//!
//! Tests MPMC backends with varying thread counts to produce
//! scalability curves for node and topic count.
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
    detect_platform, set_cpu_affinity, write_json_report, BenchmarkConfig, BenchmarkReport,
    BenchmarkResult, DeterminismMetrics, Provenance, Statistics, ThroughputMetrics,
};
use horus_core::core::DurationExt;
use serde::{Deserialize, Serialize};
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::Arc;
use std::thread;
use std::time::Instant;

const MEASUREMENT_DURATION_SECS: u64 = 5;
const WARMUP_DURATION_SECS: u64 = 1;

/// Benchmark payload (64 bytes total)
#[derive(Clone, Copy, Debug, Serialize, Deserialize, Default)]
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

impl horus_core::core::LogSummary for ScalabilityMsg {
    fn log_summary(&self) -> String {
        format!("ScalabilityMsg(p={}, seq={})", self.producer_id, self.seq)
    }
}

fn main() {
    let args: Vec<String> = std::env::args().collect();

    // Parse arguments
    let mut json_output: Option<String> = None;
    // `--max-threads` is documented in the module header above. It used not to
    // be parsed at all: the flag was accepted silently and every run tested
    // `num_cpus::get()` threads regardless of what the caller asked for.
    let mut max_threads = num_cpus::get();

    let mut i = 1;
    while i < args.len() {
        match args[i].as_str() {
            "--json" => {
                json_output = args.get(i + 1).cloned();
                i += 2;
            }
            "--max-threads" => {
                max_threads = args
                    .get(i + 1)
                    .and_then(|s| s.parse::<usize>().ok())
                    .filter(|&n| n >= 2)
                    .unwrap_or(max_threads);
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
    println!(
        "Platform: {} ({} logical cores)",
        platform.cpu.model, platform.cpu.logical_cores
    );
    println!("Max threads to test: {}", max_threads);
    println!();

    let mut report = BenchmarkReport::new(platform.clone());

    // Test configurations: (producers, consumers)
    let configs: Vec<(usize, usize)> = vec![
        (1, 1), // SPSC baseline
        (2, 1), // 2 producers
        (4, 1), // 4 producers
        (1, 2), // 2 consumers
        (1, 4), // 4 consumers
        (2, 2), // 2x2 balanced
        (4, 4), // 4x4 balanced
        (8, 8), // 8x8 if available
    ]
    .into_iter()
    .filter(|(p, c)| p + c <= max_threads)
    .collect();

    println!("Throughput below counts DELIVERED messages. `Delivered` is received/sent:");
    println!("producers are unthrottled and the ring drops what consumers cannot keep up");
    println!("with, so a high msg/s with a low delivery ratio is loss, not speed.");
    println!();
    println!("╔══════════════════════════════════════════════════════════════════════════════════════════╗");
    println!("║ Producers │ Consumers │ Throughput (M msg/s) │ Per-Thread │ Delivered │ Scaling Efficiency ║");
    println!("╠══════════════════════════════════════════════════════════════════════════════════════════╣");

    let baseline_throughput = Arc::new(AtomicU64::new(0));

    for (num_producers, num_consumers) in &configs {
        let (result, delivery) = run_scalability_test(
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
            "║    {:>2}      │     {:>2}     │       {:>8.2}       │   {:>6.2}   │  {:>6.1}%  │       {:>6.1}%       ║",
            num_producers,
            num_consumers,
            throughput_m,
            per_thread,
            delivery * 100.0,
            efficiency
        );

        report.add_result(result);
    }

    println!("╚══════════════════════════════════════════════════════════════════════════════════════════╝");

    // Producer scaling curve
    println!("\n╔══════════════════════════════════════════════════════════════════╗");
    println!("║              Producer Scaling (1 Consumer)                       ║");
    println!("╠══════════════════════════════════════════════════════════════════╣");

    let producer_counts: Vec<usize> = (1..=max_threads.min(8)).collect();
    for &num_producers in &producer_counts {
        let (result, delivery) =
            run_scalability_test(num_producers, 1, &platform, baseline_throughput.clone());
        let throughput_m = result.throughput.messages_per_sec / 1_000_000.0;
        let bar_len = ((throughput_m / 10.0) * 40.0).min(40.0) as usize;
        let bar: String = "█".repeat(bar_len);

        println!(
            "║ {:>2} producers: {:>6.2} M/s (delivered {:>5.1}%) │{}",
            num_producers,
            throughput_m,
            delivery * 100.0,
            bar
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
        let (result, delivery) =
            run_scalability_test(1, num_consumers, &platform, baseline_throughput.clone());
        let throughput_m = result.throughput.messages_per_sec / 1_000_000.0;
        let bar_len = ((throughput_m / 10.0) * 40.0).min(40.0) as usize;
        let bar: String = "█".repeat(bar_len);

        println!(
            "║ {:>2} consumers: {:>6.2} M/s (delivered {:>5.1}%) │{}",
            num_consumers,
            throughput_m,
            delivery * 100.0,
            bar
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

/// Number of messages a worker accumulates locally before publishing its count
/// to the shared counter.
///
/// The counters used to be published once, at thread exit. That made the
/// "reset for measurement" store below dead code: nothing had been added yet,
/// so the reset cleared nothing and the final totals covered warmup *and*
/// measurement while `duration` covered measurement only — every throughput
/// figure in this binary was inflated by (warmup + measure) / measure, i.e. 20%
/// at the current 1 s / 5 s split. Flushing in chunks makes the reset mean what
/// it says, at the cost of one uncontended relaxed add per 4096 messages.
///
/// The reset can still clobber a flush that is in flight, losing up to one
/// chunk per thread. That direction under-counts, which is the safe way for a
/// throughput number to be wrong.
const COUNTER_FLUSH_CHUNK: u64 = 4096;

/// Returns the result and the delivery ratio (received / sent) over the
/// measurement window. The ratio is not part of `BenchmarkResult`, and it is
/// the only thing separating "this configuration is fast" from "this
/// configuration drops most of its traffic".
fn run_scalability_test(
    num_producers: usize,
    num_consumers: usize,
    platform: &horus_benchmarks::PlatformInfo,
    _baseline: Arc<AtomicU64>,
) -> (BenchmarkResult, f64) {
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

                producer.send(msg);
                local_sent += 1;
                seq += 1;

                if local_sent == COUNTER_FLUSH_CHUNK {
                    total_sent.fetch_add(local_sent, Ordering::Relaxed);
                    local_sent = 0;
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

                    if local_received == COUNTER_FLUSH_CHUNK {
                        total_received.fetch_add(local_received, Ordering::Relaxed);
                        local_received = 0;
                    }
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
    thread::sleep(100_u64.ms());

    // Warmup phase
    running.store(true, Ordering::Release);
    thread::sleep(WARMUP_DURATION_SECS.secs());

    // Reset counters so the measurement window excludes warmup. This only works
    // because the workers flush their local counts every COUNTER_FLUSH_CHUNK
    // messages; when they published only at thread exit, this store cleared two
    // zeros and the warmup traffic was counted against the measurement duration.
    total_sent.store(0, Ordering::Relaxed);
    total_received.store(0, Ordering::Relaxed);

    // Measurement phase
    let start = Instant::now();
    thread::sleep(MEASUREMENT_DURATION_SECS.secs());
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

    // Calculate throughput.
    //
    // `sent` used to be loaded into `_sent` and thrown away, which made message
    // loss structurally invisible: producers are unthrottled, the ring drops
    // whatever consumers cannot keep up with, and a configuration that
    // delivered 5% of its traffic reported the same shape of number as one that
    // delivered all of it. The ratio is returned to the caller and printed.
    let sent = total_sent.load(Ordering::Relaxed);
    let received = total_received.load(Ordering::Relaxed);
    let duration_secs = duration.as_secs_f64();
    let delivery_ratio = if sent > 0 {
        received as f64 / sent as f64
    } else {
        f64::NAN
    };

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

    // This benchmark times nothing per-message: it counts deliveries over a
    // fixed wall-clock window. So there are NO latency samples, and the
    // statistics block must say so.
    //
    // It previously wrote literal zeros into every order statistic while
    // setting `count: received` — a non-zero count. That defeated
    // `Statistics::is_empty()`, the one guard downstream consumers have, and
    // published "0 ns median, 0 ns p99, 0 ns p99.99, 0 ns max" for a run with
    // no timing data, stamped `Provenance::Measured`. Zero is the best possible
    // latency; a fabricated best-possible result is worse than no result.
    //
    // `count: 0` plus `NaN` floats is the same shape `Statistics::empty()`
    // produces, and serde writes the NaNs as JSON `null`.
    let statistics = Statistics {
        count: 0,
        mean: f64::NAN,
        median: f64::NAN,
        std_dev: f64::NAN,
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
        ci_low: f64::NAN,
        ci_high: f64::NAN,
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

    // Same reasoning as `statistics` above: no latency samples exist, so there
    // is no coefficient of variation and no jitter to report. `cv: 0.0` reads
    // as perfect determinism in every summary that grades it, and
    // `deadline_misses: 0` reads as a passed deadline gate that was never run.
    // `NaN` is the only honest value the f64 fields can carry; the u64 fields
    // have no NaN, so they stay 0 and are guarded by `statistics.count == 0`.
    let determinism = DeterminismMetrics {
        cv: f64::NAN,
        max_jitter_ns: 0,
        p999: 0,
        p9999: 0,
        deadline_misses: 0,
        deadline_threshold_ns: 0,
        run_variance: f64::NAN,
    };

    let result = BenchmarkResult {
        provenance: Provenance::Measured,
        name: format!("scalability_p{}_c{}", num_producers, num_consumers),
        // The backend is chosen by `Topic::new` at runtime and is never queried
        // here, so naming one would be an assertion, not an observation. The
        // previous "HORUS MpmcShm" was exactly that, and it was wrong for every
        // 1-producer configuration in the table.
        subject: "HORUS Topic (backend auto-selected, not verified)".to_string(),
        message_size: std::mem::size_of::<ScalabilityMsg>(),
        config,
        platform: platform.clone(),
        timestamp: chrono::Utc::now().to_rfc3339(),
        raw_latencies_ns: vec![],
        statistics,
        throughput,
        determinism,
    };

    (result, delivery_ratio)
}
