//! Robotics Message Types Benchmark
//!
//! Benchmarks HORUS IPC with actual robotics message types used in
//! real-world robotic systems. Required for academic validity per
//! REP 2014 and industry benchmarking standards.
//!
//! ## What is actually timed
//!
//! Every figure here is the cost of the **`tx.send()` call** — the publisher's
//! enqueue into the ring — measured with RDTSC around that call and nothing
//! else. It is NOT publish-to-receive latency. A consumer thread on another
//! core drains in the background so the ring does not fill; the producer never
//! waits for a message to arrive anywhere, and the batch drain-waits are
//! outside the timed region.
//!
//! This matters for the suitability verdicts below. A `send()` on a warm ring
//! costs tens of nanoseconds, and the control-rate gates it is compared against
//! are 100 µs to 25 ms — four to six orders of magnitude looser. Those gates
//! cannot fail; the output now prints the headroom beside each verdict and
//! labels a gate non-discriminating rather than printing a bare PASS that reads
//! as a real-time guarantee. A real control-loop budget also has to cover the
//! receive side, the node's compute, and the scheduler's own jitter, none of
//! which this binary measures.
//!
//! ## Message Types Tested
//!
//! | Type | Size | Use Case |
//! |------|------|----------|
//! | CmdVel | 16 bytes | Velocity control commands (1kHz+) |
//! | Imu | ~296 bytes | IMU sensor data (500Hz+) |
//! | LaserScan | ~1.5KB | 2D lidar scans (10-40Hz) |
//! | JointCommand | ~1KB | Multi-DOF control (500Hz+) |
//!
//! ## Running
//!
//! ```bash
//! cargo run --release --bin robotics_messages_benchmark
//! cargo run --release --bin robotics_messages_benchmark -- --json results.json
//! ```

use horus::prelude::Topic;
use horus_benchmarks::{
    coefficient_of_variation, detect_platform, set_cpu_affinity, timing::PrecisionTimer,
    write_json_report, BenchmarkConfig, BenchmarkReport, BenchmarkResult, DeterminismMetrics,
    Provenance, Statistics, ThroughputMetrics,
};
use horus_core::core::DurationExt;
use horus_robotics::CmdVel;
use horus_robotics::{
    control::JointCommand,
    sensor::{Imu, LaserScan},
};
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::Arc;
use std::thread;

const DEFAULT_ITERATIONS: usize = 50_000;
const DEFAULT_WARMUP: usize = 5_000;

// Per-message-type deadline budgets, one control period at the type's typical
// rate. `deadline_misses` is counted from the samples against these instead of
// being written as a literal 0 next to a threshold of 0.
//
// These are whole-period budgets applied to a send()-enqueue cost, so they are
// enormously loose — see `suitability_verdict`, which prints the headroom and
// refuses to call a gate this slack a pass.
const CMDVEL_DEADLINE_NS: u64 = 1_000_000; // 1 kHz
const IMU_DEADLINE_NS: u64 = 2_000_000; // 500 Hz
const LASERSCAN_DEADLINE_NS: u64 = 25_000_000; // 40 Hz
const JOINTCMD_DEADLINE_NS: u64 = 2_000_000; // 500 Hz

/// Verdict for a control-rate gate, carrying the margin it passed by.
///
/// A bare "✓ PASS" on `p99 < 1ms` for a quantity measured in tens of
/// nanoseconds reads as a real-time guarantee, but the gate is four orders of
/// magnitude loose and could not have failed. Anything passing by 100x or more
/// is reported as non-discriminating rather than as a pass.
fn suitability_verdict(p99_ns: u64, threshold_ns: u64) -> String {
    if p99_ns >= threshold_ns {
        return "✗ FAIL".to_string();
    }
    let headroom = threshold_ns as f64 / p99_ns.max(1) as f64;
    if headroom >= 100.0 {
        format!("~ PASS by {:.0}x — gate not discriminating", headroom)
    } else {
        format!("✓ PASS by {:.1}x", headroom)
    }
}

fn main() {
    let args: Vec<String> = std::env::args().collect();

    // Parse arguments
    let mut json_output: Option<String> = None;
    let mut iterations = DEFAULT_ITERATIONS;

    let mut i = 1;
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
    println!("║        HORUS Robotics Message Types Benchmark                    ║");
    println!("╠══════════════════════════════════════════════════════════════════╣");
    println!("║  Testing real-world robotics message latency (REP 2014)          ║");
    println!("╚══════════════════════════════════════════════════════════════════╝");
    println!();

    let platform = detect_platform();
    println!(
        "Platform: {} ({} cores)",
        platform.cpu.model, platform.cpu.logical_cores
    );
    println!("Iterations: {}", iterations);
    println!("Warmup: {}", DEFAULT_WARMUP);
    println!();

    // Set CPU affinity for determinism
    if let Err(e) = set_cpu_affinity(0) {
        eprintln!("Warning: Could not set CPU affinity: {}", e);
    }

    let mut report = BenchmarkReport::new(platform.clone());

    // Message type summary
    println!("╔═════════════════════════════════════════════════════════════════════════════╗");
    println!("║ Message Type    │ Size (bytes) │ Typical Rate │ Use Case                    ║");
    println!("╠═════════════════════════════════════════════════════════════════════════════╣");
    println!("║ CmdVel          │           16 │ 1000+ Hz     │ Velocity control commands   ║");
    println!("║ Imu             │          296 │ 500+ Hz      │ IMU sensor fusion           ║");
    println!("║ LaserScan       │         1480 │ 10-40 Hz     │ 2D lidar navigation         ║");
    println!("║ JointCommand    │         1032 │ 500+ Hz      │ Manipulator control         ║");
    println!("╚═════════════════════════════════════════════════════════════════════════════╝");
    println!();

    // Benchmark each message type with Topic (Topic::new() auto-selects backend)
    println!("\n[Topic] Running benchmarks — timing tx.send() enqueue cost only");
    println!("        (cross-thread consumer drains in the background; no sample");
    println!("         waits for delivery, so these are NOT end-to-end latencies)");
    println!("─────────────────────────────────────────────────");

    // CmdVel (16 bytes) - Control commands
    let result = benchmark_cmdvel(iterations, &platform);
    print_result(&result);
    report.add_result(result);

    // Imu (296 bytes) - Sensor data
    let result = benchmark_imu(iterations, &platform);
    print_result(&result);
    report.add_result(result);

    // LaserScan (~1.5KB) - Lidar data
    let result = benchmark_laserscan(iterations / 5, &platform);
    print_result(&result);
    report.add_result(result);

    // JointCommand (~1KB) - Multi-DOF control
    let result = benchmark_jointcmd(iterations, &platform);
    print_result(&result);
    report.add_result(result);

    // Summary table
    println!("\n╔══════════════════════════════════════════════════════════════════════════════════════════╗");
    println!("║           SUMMARY BY MESSAGE TYPE — send() ENQUEUE COST, not end-to-end latency           ║");
    println!("╠══════════════════════════════════════════════════════════════════════════════════════════╣");
    println!("║ Message      │   Size │ Median (ns) │ p99 (ns) │    CV   │ Misses │ Sustained rate       ║");
    println!("╠══════════════════════════════════════════════════════════════════════════════════════════╣");

    for result in &report.results {
        // The leading "Adaptive_" of the name was previously printed as a
        // "Backend" column. `Topic::new` picks the backend at runtime and this
        // binary never asks which one it picked, so that column asserted a fact
        // it had not observed. Dropped rather than guessed.
        let msg_type = result.name.split('_').skip(1).collect::<Vec<_>>().join("_");
        println!(
            "║ {:12} │ {:>6} │ {:>11.0} │ {:>8} │ {:>7.4} │ {:>6} │ {:>9.2} M msg/s     ║",
            msg_type,
            result.message_size,
            result.statistics.median,
            result.statistics.p99,
            result.determinism.cv,
            result.determinism.deadline_misses,
            result.throughput.messages_per_sec / 1_000_000.0
        );
    }
    println!("╚══════════════════════════════════════════════════════════════════════════════════════════╝");
    println!("Sustained rate is wall-clock over the measured loop, so it includes the batch");
    println!("drain-waits. It is a publish rate for this producer/consumer pair, not a bus limit.");

    // Suitability analysis for real-time robotics
    println!("\n╔══════════════════════════════════════════════════════════════════╗");
    println!("║              REAL-TIME SUITABILITY ANALYSIS                      ║");
    println!("╠══════════════════════════════════════════════════════════════════╣");
    println!("║ These gates grade the p99 of the send() ENQUEUE COST against one ║");
    println!("║ whole control period. They are not end-to-end control-loop       ║");
    println!("║ guarantees: a real loop also pays the receive side, the node's   ║");
    println!("║ compute, and the scheduler's jitter, none of which is measured   ║");
    println!("║ here. The headroom is printed so a pass by four orders of        ║");
    println!("║ magnitude cannot be read as a tight result.                      ║");
    println!("╠══════════════════════════════════════════════════════════════════╣");

    for (needle, label, threshold) in [
        ("CmdVel", "CmdVel    (1kHz  control)", CMDVEL_DEADLINE_NS),
        ("CmdVel", "CmdVel    (10kHz control)", 100_000u64),
        ("Imu", "Imu       (500Hz fusion) ", IMU_DEADLINE_NS),
        ("LaserScan", "LaserScan (40Hz  lidar)  ", LASERSCAN_DEADLINE_NS),
        (
            "JointCommand",
            "JointCmd  (500Hz control)",
            JOINTCMD_DEADLINE_NS,
        ),
    ] {
        // `min_by` over medians used `partial_cmp(..).unwrap()`, which panics on
        // a NaN median — the value `Statistics` uses for "no samples". Fall back
        // to Equal so a run that collected nothing reports rather than aborts.
        let best = report
            .results
            .iter()
            .filter(|r| r.name.contains(needle))
            .min_by(|a, b| {
                a.statistics
                    .median
                    .partial_cmp(&b.statistics.median)
                    .unwrap_or(std::cmp::Ordering::Equal)
            });
        if let Some(best) = best {
            println!(
                "║ {}: {} (p99={}ns vs {}ns)",
                label,
                suitability_verdict(best.statistics.p99, threshold),
                best.statistics.p99,
                threshold
            );
        }
    }

    println!("╚══════════════════════════════════════════════════════════════════╝");

    // Write JSON output
    if let Some(path) = json_output {
        match write_json_report(&report, &path) {
            Ok(_) => println!("\nResults written to: {}", path),
            Err(e) => eprintln!("\nFailed to write JSON: {}", e),
        }
    }
}

fn benchmark_cmdvel(
    iterations: usize,
    platform: &horus_benchmarks::PlatformInfo,
) -> BenchmarkResult {
    let topic_name = format!("bench_cmdvel_{}", std::process::id());
    let topic_name_clone = topic_name.clone();
    let timer = PrecisionTimer::new();

    let running = Arc::new(AtomicBool::new(true));
    let consumer_ready = Arc::new(AtomicBool::new(false));
    let messages_received = Arc::new(AtomicU64::new(0));

    let running_clone = running.clone();
    let consumer_ready_clone = consumer_ready.clone();
    let messages_received_clone = messages_received.clone();

    let total_messages = DEFAULT_WARMUP + iterations;

    // Consumer thread
    let consumer_handle = thread::spawn(move || {
        let _ = set_cpu_affinity(1);
        let rx: Topic<CmdVel> = Topic::new(&topic_name_clone).unwrap();
        consumer_ready_clone.store(true, Ordering::Release);

        let mut received = 0u64;
        while running_clone.load(Ordering::Acquire) || received < total_messages as u64 {
            if rx.recv().is_some() {
                received += 1;
                messages_received_clone.fetch_add(1, Ordering::Release);
                if received >= total_messages as u64 {
                    break;
                }
            }
        }
    });

    while !consumer_ready.load(Ordering::Acquire) {
        thread::yield_now();
    }
    thread::sleep(10_u64.ms());

    let _ = set_cpu_affinity(0);
    let tx: Topic<CmdVel> = Topic::new(&topic_name).unwrap();

    // Warmup - batch to avoid buffer overflow (capacity = 64)
    const BATCH_SIZE: usize = 32;
    for i in 0..DEFAULT_WARMUP {
        let msg = CmdVel::new(1.0, 0.5);
        tx.send(msg);
        thread::yield_now();
        if (i + 1) % BATCH_SIZE == 0 {
            while messages_received.load(Ordering::Acquire) < (i + 1) as u64 {
                thread::yield_now();
            }
        }
    }

    while messages_received.load(Ordering::Acquire) < DEFAULT_WARMUP as u64 {
        thread::yield_now();
    }

    // Measurement — times tx.send() only. See the module header.
    let warmup_base = DEFAULT_WARMUP as u64;
    let mut latencies = Vec::with_capacity(iterations);
    let wall_start = std::time::Instant::now();
    for i in 0..iterations {
        let msg = CmdVel::new(1.0 + (i as f32 * 0.001), 0.5);
        let start = timer.start();
        tx.send(msg);
        latencies.push(timer.elapsed_ns(start));
        if (i + 1) % BATCH_SIZE == 0 {
            let target = warmup_base + (i + 1) as u64;
            while messages_received.load(Ordering::Acquire) < target {
                thread::yield_now();
            }
        }
    }
    let wall_secs = wall_start.elapsed().as_secs_f64();

    while messages_received.load(Ordering::Acquire) < total_messages as u64 {
        thread::yield_now();
    }

    running.store(false, Ordering::Release);
    consumer_handle.join().ok();

    build_result(
        "Adaptive_CmdVel",
        std::mem::size_of::<CmdVel>(),
        latencies,
        iterations,
        CMDVEL_DEADLINE_NS,
        wall_secs,
        platform,
    )
}

fn benchmark_imu(iterations: usize, platform: &horus_benchmarks::PlatformInfo) -> BenchmarkResult {
    let topic_name = format!("bench_imu_{}", std::process::id());
    let topic_name_clone = topic_name.clone();
    let timer = PrecisionTimer::new();

    let running = Arc::new(AtomicBool::new(true));
    let consumer_ready = Arc::new(AtomicBool::new(false));
    let messages_received = Arc::new(AtomicU64::new(0));

    let running_clone = running.clone();
    let consumer_ready_clone = consumer_ready.clone();
    let messages_received_clone = messages_received.clone();

    let total_messages = DEFAULT_WARMUP + iterations;

    let consumer_handle = thread::spawn(move || {
        let _ = set_cpu_affinity(1);
        // Use explicit capacity=128 to prevent deadlock with batch sync
        // (Imu is 296B, auto-capacity would be ~16, too small for BATCH_SIZE=8)
        let rx: Topic<Imu> = Topic::with_capacity(&topic_name_clone, 128, None).unwrap();
        consumer_ready_clone.store(true, Ordering::Release);

        let mut received = 0u64;
        while running_clone.load(Ordering::Acquire) || received < total_messages as u64 {
            if rx.recv().is_some() {
                received += 1;
                messages_received_clone.fetch_add(1, Ordering::Release);
                if received >= total_messages as u64 {
                    break;
                }
            }
        }
    });

    while !consumer_ready.load(Ordering::Acquire) {
        thread::yield_now();
    }
    thread::sleep(10_u64.ms());

    let _ = set_cpu_affinity(0);
    // Match consumer capacity
    let tx: Topic<Imu> = Topic::with_capacity(&topic_name, 128, None).unwrap();

    // Warmup - use smaller batch to stay within capacity
    const BATCH_SIZE: usize = 8;
    for i in 0..DEFAULT_WARMUP {
        let msg = Imu::new();
        tx.send(msg);
        thread::yield_now();
        if (i + 1) % BATCH_SIZE == 0 {
            while messages_received.load(Ordering::Acquire) < (i + 1) as u64 {
                thread::yield_now();
            }
        }
    }

    while messages_received.load(Ordering::Acquire) < DEFAULT_WARMUP as u64 {
        thread::yield_now();
    }

    // Measurement — times tx.send() only. See the module header.
    let warmup_base = DEFAULT_WARMUP as u64;
    let mut latencies = Vec::with_capacity(iterations);
    let wall_start = std::time::Instant::now();
    for i in 0..iterations {
        let mut msg = Imu::new();
        msg.linear_acceleration = [0.0, 0.0, 9.81];
        msg.angular_velocity = [0.01, 0.02, 0.0];
        let start = timer.start();
        tx.send(msg);
        latencies.push(timer.elapsed_ns(start));
        if (i + 1) % BATCH_SIZE == 0 {
            let target = warmup_base + (i + 1) as u64;
            while messages_received.load(Ordering::Acquire) < target {
                thread::yield_now();
            }
        }
    }
    let wall_secs = wall_start.elapsed().as_secs_f64();

    while messages_received.load(Ordering::Acquire) < total_messages as u64 {
        thread::yield_now();
    }

    running.store(false, Ordering::Release);
    consumer_handle.join().ok();

    build_result(
        "Adaptive_Imu",
        std::mem::size_of::<Imu>(),
        latencies,
        iterations,
        IMU_DEADLINE_NS,
        wall_secs,
        platform,
    )
}

fn benchmark_laserscan(
    iterations: usize,
    platform: &horus_benchmarks::PlatformInfo,
) -> BenchmarkResult {
    let topic_name = format!("bench_laser_{}", std::process::id());
    let topic_name_clone = topic_name.clone();
    let timer = PrecisionTimer::new();

    let warmup_count = DEFAULT_WARMUP / 5;
    let running = Arc::new(AtomicBool::new(true));
    let consumer_ready = Arc::new(AtomicBool::new(false));
    let messages_received = Arc::new(AtomicU64::new(0));

    let running_clone = running.clone();
    let consumer_ready_clone = consumer_ready.clone();
    let messages_received_clone = messages_received.clone();

    let total_messages = warmup_count + iterations;

    let consumer_handle = thread::spawn(move || {
        let _ = set_cpu_affinity(1);
        // LaserScan is 1480B, auto-capacity=16. Use explicit 64 to avoid deadlock.
        let rx: Topic<LaserScan> = Topic::with_capacity(&topic_name_clone, 64, None).unwrap();
        consumer_ready_clone.store(true, Ordering::Release);

        let mut received = 0u64;
        while running_clone.load(Ordering::Acquire) || received < total_messages as u64 {
            if rx.recv().is_some() {
                received += 1;
                messages_received_clone.fetch_add(1, Ordering::Release);
                if received >= total_messages as u64 {
                    break;
                }
            }
        }
    });

    while !consumer_ready.load(Ordering::Acquire) {
        thread::yield_now();
    }
    thread::sleep(10_u64.ms());

    let _ = set_cpu_affinity(0);
    let tx: Topic<LaserScan> = Topic::with_capacity(&topic_name, 64, None).unwrap();

    // Warmup - use small batch to stay within capacity
    const BATCH_SIZE: usize = 8;
    for i in 0..warmup_count {
        let mut msg = LaserScan::new();
        for j in 0..360 {
            msg.ranges[j] = 5.0 + (j as f32 * 0.01);
        }
        tx.send(msg);
        thread::yield_now();
        if (i + 1) % BATCH_SIZE == 0 {
            while messages_received.load(Ordering::Acquire) < (i + 1) as u64 {
                thread::yield_now();
            }
        }
    }

    while messages_received.load(Ordering::Acquire) < warmup_count as u64 {
        thread::yield_now();
    }

    // Measurement — times tx.send() only. See the module header.
    let warmup_base = warmup_count as u64;
    let mut latencies = Vec::with_capacity(iterations);
    let wall_start = std::time::Instant::now();
    for seq in 0..iterations {
        let mut msg = LaserScan::new();
        for j in 0..360 {
            msg.ranges[j] = 5.0 + ((j + seq) as f32 * 0.01);
        }
        let start = timer.start();
        tx.send(msg);
        latencies.push(timer.elapsed_ns(start));
        if (seq + 1) % BATCH_SIZE == 0 {
            let target = warmup_base + (seq + 1) as u64;
            while messages_received.load(Ordering::Acquire) < target {
                thread::yield_now();
            }
        }
    }
    let wall_secs = wall_start.elapsed().as_secs_f64();

    while messages_received.load(Ordering::Acquire) < total_messages as u64 {
        thread::yield_now();
    }

    running.store(false, Ordering::Release);
    consumer_handle.join().ok();

    build_result(
        "Adaptive_LaserScan",
        std::mem::size_of::<LaserScan>(),
        latencies,
        iterations,
        LASERSCAN_DEADLINE_NS,
        wall_secs,
        platform,
    )
}

fn benchmark_jointcmd(
    iterations: usize,
    platform: &horus_benchmarks::PlatformInfo,
) -> BenchmarkResult {
    let topic_name = format!("bench_joint_{}", std::process::id());
    let topic_name_clone = topic_name.clone();
    let timer = PrecisionTimer::new();

    let running = Arc::new(AtomicBool::new(true));
    let consumer_ready = Arc::new(AtomicBool::new(false));
    let messages_received = Arc::new(AtomicU64::new(0));

    let running_clone = running.clone();
    let consumer_ready_clone = consumer_ready.clone();
    let messages_received_clone = messages_received.clone();

    let total_messages = DEFAULT_WARMUP + iterations;

    let consumer_handle = thread::spawn(move || {
        let _ = set_cpu_affinity(1);
        // JointCommand is 1032B, auto-capacity=16. Use explicit 64 to avoid deadlock.
        let rx: Topic<JointCommand> = Topic::with_capacity(&topic_name_clone, 64, None).unwrap();
        consumer_ready_clone.store(true, Ordering::Release);

        let mut received = 0u64;
        while running_clone.load(Ordering::Acquire) || received < total_messages as u64 {
            if rx.recv().is_some() {
                received += 1;
                messages_received_clone.fetch_add(1, Ordering::Release);
                if received >= total_messages as u64 {
                    break;
                }
            }
        }
    });

    while !consumer_ready.load(Ordering::Acquire) {
        thread::yield_now();
    }
    thread::sleep(10_u64.ms());

    let _ = set_cpu_affinity(0);
    let tx: Topic<JointCommand> = Topic::with_capacity(&topic_name, 64, None).unwrap();

    // Warmup - use small batch to stay within capacity
    const BATCH_SIZE: usize = 8;
    for i in 0..DEFAULT_WARMUP {
        let mut msg = JointCommand::new();
        msg.add_position("shoulder_pan", 0.5).ok();
        msg.add_position("shoulder_lift", -0.3).ok();
        msg.add_position("elbow", 1.2).ok();
        msg.add_position("wrist_1", 0.0).ok();
        msg.add_position("wrist_2", -0.5).ok();
        msg.add_position("wrist_3", 0.1).ok();
        tx.send(msg);
        thread::yield_now();
        if (i + 1) % BATCH_SIZE == 0 {
            while messages_received.load(Ordering::Acquire) < (i + 1) as u64 {
                thread::yield_now();
            }
        }
    }

    while messages_received.load(Ordering::Acquire) < DEFAULT_WARMUP as u64 {
        thread::yield_now();
    }

    // Measurement — times tx.send() only. See the module header.
    let warmup_base = DEFAULT_WARMUP as u64;
    let mut latencies = Vec::with_capacity(iterations);
    let wall_start = std::time::Instant::now();
    for i in 0..iterations {
        let mut msg = JointCommand::new();
        let offset = (i as f64) * 0.001;
        msg.add_position("shoulder_pan", 0.5 + offset).ok();
        msg.add_position("shoulder_lift", -0.3 + offset).ok();
        msg.add_position("elbow", 1.2 + offset).ok();
        msg.add_position("wrist_1", offset).ok();
        msg.add_position("wrist_2", -0.5 + offset).ok();
        msg.add_position("wrist_3", 0.1 + offset).ok();
        let start = timer.start();
        tx.send(msg);
        latencies.push(timer.elapsed_ns(start));
        if (i + 1) % BATCH_SIZE == 0 {
            let target = warmup_base + (i + 1) as u64;
            while messages_received.load(Ordering::Acquire) < target {
                thread::yield_now();
            }
        }
    }
    let wall_secs = wall_start.elapsed().as_secs_f64();

    while messages_received.load(Ordering::Acquire) < total_messages as u64 {
        thread::yield_now();
    }

    running.store(false, Ordering::Release);
    consumer_handle.join().ok();

    build_result(
        "Adaptive_JointCommand",
        std::mem::size_of::<JointCommand>(),
        latencies,
        iterations,
        JOINTCMD_DEADLINE_NS,
        wall_secs,
        platform,
    )
}

/// Build a result from the measured send-enqueue samples.
///
/// `deadline_ns` is the control-rate budget for this message type; the miss
/// count is computed from the samples against it. It used to be written as the
/// literal `0` with `deadline_threshold_ns: 0` beside it and stamped
/// `Provenance::Measured` — a real-time deadline gate that reported a pass
/// without ever comparing a sample to anything.
///
/// `wall_secs` is the wall-clock duration of the measured loop, used for the
/// throughput figure. Summing the per-sample send costs instead gives
/// `1 / mean(send_ns)`, an instruction-retire rate with every drain wait
/// removed, which no caller can achieve.
fn build_result(
    name: &str,
    message_size: usize,
    latencies: Vec<u64>,
    iterations: usize,
    deadline_ns: u64,
    wall_secs: f64,
    platform: &horus_benchmarks::PlatformInfo,
) -> BenchmarkResult {
    let config = BenchmarkConfig {
        warmup_iterations: DEFAULT_WARMUP,
        iterations,
        runs: 1,
        cpu_affinity: Some((0, 0)),
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
        deadline_misses: latencies.iter().filter(|&&l| l > deadline_ns).count() as u64,
        deadline_threshold_ns: deadline_ns,
        // Single run per message type: there is no run-to-run spread. 0.0 would
        // claim perfect reproducibility across runs that were never performed.
        run_variance: f64::NAN,
    };

    let duration_secs = wall_secs;

    let throughput = ThroughputMetrics {
        messages_per_sec: latencies.len() as f64 / duration_secs.max(1e-9),
        bytes_per_sec: (latencies.len() * message_size) as f64 / duration_secs.max(1e-9),
        total_messages: latencies.len() as u64,
        total_bytes: (latencies.len() * message_size) as u64,
        duration_secs,
    };

    BenchmarkResult {
        provenance: Provenance::Measured,
        name: name.to_string(),
        subject: "HORUS Topic".to_string(),
        message_size,
        config,
        platform: platform.clone(),
        timestamp: chrono::Utc::now().to_rfc3339(),
        raw_latencies_ns: latencies,
        statistics,
        throughput,
        determinism,
    }
}

fn print_result(result: &BenchmarkResult) {
    println!(
        "  {:15} {:>6} bytes │ median: {:>7.0}ns │ p99: {:>7}ns │ CV: {:.4}",
        result.name.split('_').skip(1).collect::<Vec<_>>().join("_"),
        result.message_size,
        result.statistics.median,
        result.statistics.p99,
        result.determinism.cv
    );
}
