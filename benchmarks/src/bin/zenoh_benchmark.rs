//! Zenoh Transport Benchmark
//!
//! Comprehensive benchmarks comparing Zenoh transport with native HORUS IPC.
//!
//! ## What's Measured
//!
//! 1. **Latency**: Round-trip time at various message sizes
//! 2. **Throughput**: Messages per second sustainable rate
//! 3. **Comparison**: Zenoh vs native HORUS Topic (shared memory)
//!
//! ## Running
//!
//! ```bash
//! # Full benchmark suite
//! cargo run --release -p horus_benchmarks --features zenoh --bin zenoh_benchmark
//!
//! # With JSON output
//! cargo run --release -p horus_benchmarks --features zenoh --bin zenoh_benchmark -- --json results.json
//!
//! # Quick test (fewer iterations)
//! cargo run --release -p horus_benchmarks --features zenoh --bin zenoh_benchmark -- --quick
//! ```

use horus::prelude::Topic;
use horus_benchmarks::{
    coefficient_of_variation, detect_platform, set_cpu_affinity, write_json_report,
    BenchmarkConfig, BenchmarkReport, BenchmarkResult, DeterminismMetrics, Statistics,
    ThroughputMetrics,
};
use horus_core::communication::network::zenoh_backend::ZenohBackend;
use horus_core::communication::network::zenoh_config::{SerializationFormat, ZenohConfig};
use serde::{Deserialize, Serialize};
use std::env;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

const DEFAULT_ITERATIONS: usize = 10_000;
const DEFAULT_WARMUP: usize = 1_000;
const QUICK_ITERATIONS: usize = 1_000;
const QUICK_WARMUP: usize = 100;

/// Message with embedded high-resolution timestamp
#[derive(Clone, Debug, Serialize, Deserialize, Default)]
struct BenchMessage {
    seq: u64,
    send_time_ns: u64,
    #[serde(with = "serde_bytes")]
    payload: Vec<u8>,
}

impl horus_core::core::LogSummary for BenchMessage {
    fn log_summary(&self) -> String {
        format!(
            "BenchMessage(seq={}, size={})",
            self.seq,
            self.payload.len()
        )
    }
}

fn current_time_ns() -> u64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap_or_default()
        .as_nanos() as u64
}

fn main() {
    let args: Vec<String> = env::args().collect();

    // Parse arguments
    let mut json_output: Option<String> = None;
    let mut quick_mode = false;

    let mut i = 1;
    while i < args.len() {
        match args[i].as_str() {
            "--json" => {
                json_output = args.get(i + 1).cloned();
                i += 2;
            }
            "--quick" => {
                quick_mode = true;
                i += 1;
            }
            _ => {
                i += 1;
            }
        }
    }

    let (iterations, warmup) = if quick_mode {
        (QUICK_ITERATIONS, QUICK_WARMUP)
    } else {
        (DEFAULT_ITERATIONS, DEFAULT_WARMUP)
    };

    println!("╔══════════════════════════════════════════════════════════════════╗");
    println!("║           HORUS Zenoh Transport Benchmark                        ║");
    println!("╠══════════════════════════════════════════════════════════════════╣");
    println!("║  Comparing Zenoh transport vs native HORUS IPC                   ║");
    println!("╚══════════════════════════════════════════════════════════════════╝");
    println!();

    let platform = detect_platform();
    println!(
        "Platform: {} ({} cores)",
        platform.cpu.model, platform.cpu.logical_cores
    );
    println!("Mode: {}", if quick_mode { "Quick" } else { "Full" });
    println!("Iterations: {}", iterations);
    println!("Warmup: {}", warmup);
    println!();

    // Set CPU affinity for consistent results
    if let Err(e) = set_cpu_affinity(0) {
        eprintln!("Warning: Could not set CPU affinity: {}", e);
    }

    let mut report = BenchmarkReport::new(platform.clone());

    // Create tokio runtime for async Zenoh operations
    let rt = tokio::runtime::Runtime::new().expect("Failed to create tokio runtime");

    // ==========================================================================
    // LATENCY BENCHMARKS
    // ==========================================================================
    println!("\n=== LATENCY BENCHMARKS ===\n");

    // Test subset of message sizes for reasonable benchmark time
    let test_sizes = [
        ("control_cmd", 16),
        ("imu_reading", 128),
        ("lidar_scan", 4096),
    ];

    for (name, size) in &test_sizes {
        println!("--- {} ({} bytes) ---", name, size);

        // Benchmark native HORUS Topic
        let native_result = benchmark_native_topic(*size, iterations, warmup, &platform);
        report.results.push(native_result.clone());
        print_latency_result("Native Topic", &native_result);

        // Benchmark Zenoh (Bincode)
        let zenoh_bincode_result = rt.block_on(benchmark_zenoh(
            *size,
            iterations,
            warmup,
            &platform,
            SerializationFormat::Bincode,
        ));
        report.results.push(zenoh_bincode_result.clone());
        print_latency_result("Zenoh Bincode", &zenoh_bincode_result);

        // Benchmark Zenoh (CDR)
        let zenoh_cdr_result = rt.block_on(benchmark_zenoh(
            *size,
            iterations,
            warmup,
            &platform,
            SerializationFormat::Cdr,
        ));
        report.results.push(zenoh_cdr_result.clone());
        print_latency_result("Zenoh CDR", &zenoh_cdr_result);

        // Print comparison
        let native_median = native_result.statistics.median;
        let zenoh_median = zenoh_bincode_result.statistics.median;
        let ratio = zenoh_median / native_median;
        println!(
            "  Comparison: Zenoh is {:.1}x slower than native ({:.0} ns vs {:.0} ns)",
            ratio, zenoh_median, native_median
        );
        println!();
    }

    // ==========================================================================
    // THROUGHPUT BENCHMARKS
    // ==========================================================================
    println!("\n=== THROUGHPUT BENCHMARKS ===\n");

    let throughput_size = 128; // IMU-sized messages
    let throughput_duration = Duration::from_secs(if quick_mode { 1 } else { 5 });

    // Native throughput
    let native_throughput = benchmark_native_throughput(throughput_size, throughput_duration);
    println!(
        "Native Topic:    {:>10.0} msgs/sec ({:.2} MB/sec)",
        native_throughput.messages_per_sec,
        native_throughput.bytes_per_sec / 1_000_000.0
    );

    // Zenoh throughput
    let zenoh_throughput = rt.block_on(benchmark_zenoh_throughput(
        throughput_size,
        throughput_duration,
        SerializationFormat::Bincode,
    ));
    println!(
        "Zenoh Bincode:   {:>10.0} msgs/sec ({:.2} MB/sec)",
        zenoh_throughput.messages_per_sec,
        zenoh_throughput.bytes_per_sec / 1_000_000.0
    );

    let throughput_ratio = native_throughput.messages_per_sec / zenoh_throughput.messages_per_sec;
    println!(
        "\nNative is {:.1}x faster throughput than Zenoh (local same-process)",
        throughput_ratio
    );

    // ==========================================================================
    // SUMMARY
    // ==========================================================================
    println!("\n=== SUMMARY ===\n");
    println!("Native HORUS Topic:");
    println!("  - Uses shared memory for zero-copy IPC");
    println!("  - Sub-microsecond latency (<500ns typical)");
    println!("  - Best for same-machine, high-frequency data");
    println!();
    println!("Zenoh Transport:");
    println!("  - Adds network abstraction overhead");
    println!("  - Enables multi-robot mesh networking");
    println!("  - Supports ROS2 interop (with CDR format)");
    println!("  - Best for remote communication, cloud connectivity");
    println!();

    // Write JSON report if requested
    if let Some(path) = json_output {
        if let Err(e) = write_json_report(&report, &path) {
            eprintln!("Failed to write JSON report: {}", e);
        } else {
            println!("Results written to: {}", path);
        }
    }
}

fn print_latency_result(name: &str, result: &BenchmarkResult) {
    println!(
        "  {:<15} median: {:>6.0} ns, p99: {:>6} ns, cv: {:.3}",
        name, result.statistics.median, result.statistics.p99, result.determinism.cv
    );
}

/// Benchmark native HORUS Topic (shared memory)
fn benchmark_native_topic(
    payload_size: usize,
    iterations: usize,
    warmup: usize,
    platform: &horus_benchmarks::PlatformInfo,
) -> BenchmarkResult {
    let topic: Topic<BenchMessage> = Topic::new("bench/native").expect("Failed to create topic");

    // Warmup
    for i in 0..warmup {
        let msg = BenchMessage {
            seq: i as u64,
            send_time_ns: current_time_ns(),
            payload: vec![0u8; payload_size],
        };
        topic.send(msg);
        let _ = topic.recv();
    }

    // Measured iterations
    let mut latencies = Vec::with_capacity(iterations);

    for i in 0..iterations {
        let send_time = current_time_ns();
        let msg = BenchMessage {
            seq: i as u64,
            send_time_ns: send_time,
            payload: vec![0u8; payload_size],
        };
        topic.send(msg);

        // Spin until we receive the message
        loop {
            if let Some(received) = topic.recv() {
                let recv_time = current_time_ns();
                latencies.push(recv_time - received.send_time_ns);
                break;
            }
            std::hint::spin_loop();
        }
    }

    build_result(
        "native_topic",
        "HORUS Topic (SHM)",
        payload_size,
        iterations,
        warmup,
        latencies,
        platform,
    )
}

/// Benchmark Zenoh transport
async fn benchmark_zenoh(
    payload_size: usize,
    iterations: usize,
    warmup: usize,
    platform: &horus_benchmarks::PlatformInfo,
    format: SerializationFormat,
) -> BenchmarkResult {
    let config = ZenohConfig {
        serialization: format,
        ..Default::default()
    };

    let topic_name = format!("bench/zenoh_{:?}", format);
    let mut backend = ZenohBackend::<BenchMessage>::new(&topic_name, config)
        .await
        .expect("Failed to create Zenoh backend");

    backend
        .init_publisher()
        .await
        .expect("Failed to init publisher");
    backend
        .init_subscriber()
        .await
        .expect("Failed to init subscriber");

    // Give subscriber time to connect
    tokio::time::sleep(Duration::from_millis(100)).await;

    // Warmup
    for i in 0..warmup {
        let msg = BenchMessage {
            seq: i as u64,
            send_time_ns: current_time_ns(),
            payload: vec![0u8; payload_size],
        };
        backend.send_async(&msg).await.expect("Send failed");

        // Wait for message with timeout
        let start = Instant::now();
        while start.elapsed() < Duration::from_millis(100) {
            if backend.recv().is_some() {
                break;
            }
            tokio::time::sleep(Duration::from_micros(10)).await;
        }
    }

    // Clear buffer
    backend.clear_buffer();

    // Measured iterations
    let mut latencies = Vec::with_capacity(iterations);

    for i in 0..iterations {
        let send_time = current_time_ns();
        let msg = BenchMessage {
            seq: i as u64,
            send_time_ns: send_time,
            payload: vec![0u8; payload_size],
        };
        backend.send_async(&msg).await.expect("Send failed");

        // Wait for message with timeout
        let start = Instant::now();
        let mut received = false;
        while start.elapsed() < Duration::from_millis(100) {
            if let Some(recv_msg) = backend.recv() {
                let recv_time = current_time_ns();
                latencies.push(recv_time - recv_msg.send_time_ns);
                received = true;
                break;
            }
            tokio::time::sleep(Duration::from_micros(10)).await;
        }

        if !received {
            // Timeout - skip this sample
            continue;
        }
    }

    let format_name = match format {
        SerializationFormat::Bincode => "Bincode",
        SerializationFormat::Cdr => "CDR",
    };

    build_result(
        &format!("zenoh_{}", format_name.to_lowercase()),
        &format!("Zenoh ({})", format_name),
        payload_size,
        iterations,
        warmup,
        latencies,
        platform,
    )
}

/// Benchmark native throughput
fn benchmark_native_throughput(payload_size: usize, duration: Duration) -> ThroughputMetrics {
    let topic: Topic<BenchMessage> =
        Topic::new("bench/native_throughput").expect("Failed to create topic");

    let running = Arc::new(AtomicBool::new(true));
    let msg_count = Arc::new(AtomicU64::new(0));

    let running_clone = running.clone();
    let msg_count_clone = msg_count.clone();

    // Consumer thread
    let topic_clone: Topic<BenchMessage> =
        Topic::new("bench/native_throughput").expect("Failed to create topic");
    let consumer = std::thread::spawn(move || {
        while running_clone.load(Ordering::Relaxed) {
            if topic_clone.recv().is_some() {
                msg_count_clone.fetch_add(1, Ordering::Relaxed);
            }
            std::hint::spin_loop();
        }
    });

    // Give consumer time to start
    std::thread::sleep(Duration::from_millis(10));

    let start = Instant::now();
    let mut seq = 0u64;

    // Producer loop
    while start.elapsed() < duration {
        let msg = BenchMessage {
            seq,
            send_time_ns: 0,
            payload: vec![0u8; payload_size],
        };
        topic.send(msg);
        seq += 1;
    }

    let elapsed = start.elapsed();
    running.store(false, Ordering::Relaxed);
    let _ = consumer.join();

    let total_messages = msg_count.load(Ordering::Relaxed);
    let duration_secs = elapsed.as_secs_f64();

    ThroughputMetrics {
        messages_per_sec: total_messages as f64 / duration_secs,
        bytes_per_sec: (total_messages as f64 * payload_size as f64) / duration_secs,
        total_messages,
        total_bytes: total_messages * payload_size as u64,
        duration_secs,
    }
}

/// Benchmark Zenoh throughput
async fn benchmark_zenoh_throughput(
    payload_size: usize,
    duration: Duration,
    format: SerializationFormat,
) -> ThroughputMetrics {
    let config = ZenohConfig {
        serialization: format,
        ..Default::default()
    };

    let mut backend = ZenohBackend::<BenchMessage>::new("bench/zenoh_throughput", config)
        .await
        .expect("Failed to create Zenoh backend");

    backend
        .init_publisher()
        .await
        .expect("Failed to init publisher");
    backend
        .init_subscriber()
        .await
        .expect("Failed to init subscriber");

    // Give subscriber time to connect
    tokio::time::sleep(Duration::from_millis(100)).await;

    let start = Instant::now();
    let mut seq = 0u64;
    let mut received = 0u64;

    // Send messages as fast as possible
    while start.elapsed() < duration {
        let msg = BenchMessage {
            seq,
            send_time_ns: 0,
            payload: vec![0u8; payload_size],
        };

        if backend.send_async(&msg).await.is_ok() {
            seq += 1;
        }

        // Drain receive buffer
        while backend.recv().is_some() {
            received += 1;
        }
    }

    // Drain remaining
    tokio::time::sleep(Duration::from_millis(50)).await;
    while backend.recv().is_some() {
        received += 1;
    }

    let elapsed = start.elapsed();
    let duration_secs = elapsed.as_secs_f64();

    ThroughputMetrics {
        messages_per_sec: received as f64 / duration_secs,
        bytes_per_sec: (received as f64 * payload_size as f64) / duration_secs,
        total_messages: received,
        total_bytes: received * payload_size as u64,
        duration_secs,
    }
}

fn build_result(
    name: &str,
    subject: &str,
    message_size: usize,
    iterations: usize,
    warmup: usize,
    latencies: Vec<u64>,
    platform: &horus_benchmarks::PlatformInfo,
) -> BenchmarkResult {
    // Use Statistics::from_samples for proper outlier filtering and stats calculation
    let statistics = Statistics::from_samples(&latencies, 95.0, true);

    let cv = coefficient_of_variation(&latencies);

    // Count deadline misses (> 10us for native, > 1ms for zenoh)
    let deadline_threshold = if name.contains("native") {
        10_000 // 10us
    } else {
        1_000_000 // 1ms
    };
    let deadline_misses = latencies
        .iter()
        .filter(|&&l| l > deadline_threshold)
        .count() as u64;

    BenchmarkResult {
        name: name.to_string(),
        subject: subject.to_string(),
        message_size,
        config: BenchmarkConfig {
            warmup_iterations: warmup,
            iterations,
            runs: 1,
            cpu_affinity: Some((0, 1)),
            filter_outliers: true,
            confidence_level: 95.0,
        },
        platform: platform.clone(),
        timestamp: chrono::Utc::now().to_rfc3339(),
        raw_latencies_ns: latencies,
        statistics,
        throughput: ThroughputMetrics {
            messages_per_sec: 0.0,
            bytes_per_sec: 0.0,
            total_messages: 0,
            total_bytes: 0,
            duration_secs: 0.0,
        },
        determinism: DeterminismMetrics {
            cv,
            max_jitter_ns: 0,
            p999: 0,
            p9999: 0,
            deadline_misses,
            deadline_threshold_ns: deadline_threshold,
            run_variance: 0.0,
        },
    }
}
