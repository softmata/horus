//! Robotics Message Types Benchmark
//!
//! Benchmarks HORUS IPC with actual robotics message types used in
//! real-world robotic systems. Required for academic validity per
//! REP 2014 and industry benchmarking standards.
//!
//! ## Message Types Tested
//!
//! | Type | Size | Use Case |
//! |------|------|----------|
//! | CmdVel | 16 bytes | Velocity control commands (1kHz+) |
//! | Imu | ~296 bytes | IMU sensor data (500Hz+) |
//! | LaserScan | ~1.5KB | 2D lidar scans (10-40Hz) |
//! | JointCommand | ~1KB | Multi-DOF control (500Hz+) |
//! | PointCloud | ~48KB | 3D perception (10-30Hz) |
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
    Statistics, ThroughputMetrics,
};
use horus_library::messages::{
    control::JointCommand,
    perception::PointCloud,
    sensor::{Imu, LaserScan},
};
use horus_library::messages::CmdVel;
use serde::Serialize;

const DEFAULT_ITERATIONS: usize = 50_000;
const DEFAULT_WARMUP: usize = 5_000;

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
    println!("║ PointCloud      │       ~48000 │ 10-30 Hz     │ 3D perception               ║");
    println!("╚═════════════════════════════════════════════════════════════════════════════╝");
    println!();

    // Benchmark each message type with SPSC and MPMC backends
    let backends = [
        ("SpscIntra", BackendType::SpscIntra),
        ("SpscShm", BackendType::SpscShm),
        ("MpmcShm", BackendType::MpmcShm),
    ];

    for (backend_name, backend_type) in &backends {
        println!("\n[{}] Running benchmarks...", backend_name);
        println!("─────────────────────────────────────────────────");

        // CmdVel (16 bytes) - Control commands
        let result = benchmark_cmdvel(backend_name, *backend_type, iterations, &platform);
        print_result(&result);
        report.add_result(result);

        // Imu (296 bytes) - Sensor data
        let result = benchmark_imu(backend_name, *backend_type, iterations, &platform);
        print_result(&result);
        report.add_result(result);

        // LaserScan (~1.5KB) - Lidar data
        let result = benchmark_laserscan(backend_name, *backend_type, iterations / 5, &platform);
        print_result(&result);
        report.add_result(result);

        // JointCommand (~1KB) - Multi-DOF control
        let result = benchmark_jointcmd(backend_name, *backend_type, iterations, &platform);
        print_result(&result);
        report.add_result(result);

        // PointCloud (~48KB) - 3D perception (fewer iterations due to size)
        let result = benchmark_pointcloud(backend_name, *backend_type, iterations / 10, &platform);
        print_result(&result);
        report.add_result(result);
    }

    // Summary table
    println!("\n╔══════════════════════════════════════════════════════════════════════════════════════════╗");
    println!("║                              SUMMARY BY MESSAGE TYPE                                      ║");
    println!("╠══════════════════════════════════════════════════════════════════════════════════════════╣");
    println!("║ Backend    │ Message      │   Size │ Median (ns) │ p99 (ns) │    CV   │ Throughput      ║");
    println!("╠══════════════════════════════════════════════════════════════════════════════════════════╣");

    for result in &report.results {
        let msg_type = result.name.split('_').skip(1).collect::<Vec<_>>().join("_");
        let backend = result.name.split('_').next().unwrap_or("?");
        println!(
            "║ {:10} │ {:12} │ {:>6} │ {:>11.0} │ {:>8} │ {:>7.4} │ {:>9.2} M/s  ║",
            backend,
            msg_type,
            result.message_size,
            result.statistics.median,
            result.statistics.p99,
            result.determinism.cv,
            result.throughput.messages_per_sec / 1_000_000.0
        );
    }
    println!("╚══════════════════════════════════════════════════════════════════════════════════════════╝");

    // Suitability analysis for real-time robotics
    println!("\n╔══════════════════════════════════════════════════════════════════╗");
    println!("║              REAL-TIME SUITABILITY ANALYSIS                      ║");
    println!("╠══════════════════════════════════════════════════════════════════╣");

    let cmdvel_results: Vec<_> = report.results.iter().filter(|r| r.name.contains("CmdVel")).collect();
    if let Some(best) = cmdvel_results.iter().min_by(|a, b| {
        a.statistics.median.partial_cmp(&b.statistics.median).unwrap()
    }) {
        let meets_1khz = best.statistics.p99 < 1_000_000; // 1ms = 1MHz rate
        let meets_10khz = best.statistics.p99 < 100_000; // 100µs = 10kHz rate
        println!(
            "║ CmdVel (1kHz control):  {} (p99={:.0}ns < 1ms)",
            if meets_1khz { "✓ PASS" } else { "✗ FAIL" },
            best.statistics.p99
        );
        println!(
            "║ CmdVel (10kHz control): {} (p99={:.0}ns < 100µs)",
            if meets_10khz { "✓ PASS" } else { "✗ FAIL" },
            best.statistics.p99
        );
    }

    let imu_results: Vec<_> = report.results.iter().filter(|r| r.name.contains("Imu")).collect();
    if let Some(best) = imu_results.iter().min_by(|a, b| {
        a.statistics.median.partial_cmp(&b.statistics.median).unwrap()
    }) {
        let meets_500hz = best.statistics.p99 < 2_000_000; // 2ms = 500Hz rate
        println!(
            "║ Imu (500Hz fusion):     {} (p99={:.0}ns < 2ms)",
            if meets_500hz { "✓ PASS" } else { "✗ FAIL" },
            best.statistics.p99
        );
    }

    let lidar_results: Vec<_> = report.results.iter().filter(|r| r.name.contains("LaserScan")).collect();
    if let Some(best) = lidar_results.iter().min_by(|a, b| {
        a.statistics.median.partial_cmp(&b.statistics.median).unwrap()
    }) {
        let meets_40hz = best.statistics.p99 < 25_000_000; // 25ms = 40Hz rate
        println!(
            "║ LaserScan (40Hz lidar): {} (p99={:.0}ns < 25ms)",
            if meets_40hz { "✓ PASS" } else { "✗ FAIL" },
            best.statistics.p99
        );
    }

    let pc_results: Vec<_> = report.results.iter().filter(|r| r.name.contains("PointCloud")).collect();
    if let Some(best) = pc_results.iter().min_by(|a, b| {
        a.statistics.median.partial_cmp(&b.statistics.median).unwrap()
    }) {
        let meets_30hz = best.statistics.p99 < 33_000_000; // 33ms = 30Hz rate
        println!(
            "║ PointCloud (30Hz 3D):   {} (p99={:.0}ns < 33ms)",
            if meets_30hz { "✓ PASS" } else { "✗ FAIL" },
            best.statistics.p99
        );
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

#[derive(Clone, Copy)]
enum BackendType {
    SpscIntra,
    SpscShm,
    MpmcShm,
}

fn benchmark_cmdvel(
    backend_name: &str,
    backend: BackendType,
    iterations: usize,
    platform: &horus_benchmarks::PlatformInfo,
) -> BenchmarkResult {
    let topic_name = format!("bench_cmdvel_{}_{}", backend_name, std::process::id());
    let timer = PrecisionTimer::new();

    let (tx, rx): (Topic<CmdVel>, Topic<CmdVel>) = match backend {
        BackendType::SpscIntra => Topic::spsc_intra(&topic_name),
        BackendType::SpscShm => (
            Topic::producer(&topic_name).unwrap(),
            Topic::consumer(&topic_name).unwrap(),
        ),
        BackendType::MpmcShm => (
            Topic::new(&topic_name).unwrap(),
            Topic::new(&topic_name).unwrap(),
        ),
    };

    // Warmup
    for _ in 0..DEFAULT_WARMUP {
        let msg = CmdVel::new(1.0, 0.5);
        tx.send(msg, &mut None).unwrap();
        let _ = rx.recv(&mut None);
    }

    // Measurement
    let mut latencies = Vec::with_capacity(iterations);
    for i in 0..iterations {
        let msg = CmdVel::new(1.0 + (i as f32 * 0.001), 0.5);
        let start = timer.start();
        tx.send(msg, &mut None).unwrap();
        let _ = rx.recv(&mut None);
        latencies.push(timer.elapsed_ns(start));
    }

    build_result(
        &format!("{}_{}", backend_name, "CmdVel"),
        std::mem::size_of::<CmdVel>(),
        latencies,
        iterations,
        platform,
    )
}

fn benchmark_imu(
    backend_name: &str,
    backend: BackendType,
    iterations: usize,
    platform: &horus_benchmarks::PlatformInfo,
) -> BenchmarkResult {
    let topic_name = format!("bench_imu_{}_{}", backend_name, std::process::id());
    let timer = PrecisionTimer::new();

    let (tx, rx): (Topic<Imu>, Topic<Imu>) = match backend {
        BackendType::SpscIntra => Topic::spsc_intra(&topic_name),
        BackendType::SpscShm => (
            Topic::producer(&topic_name).unwrap(),
            Topic::consumer(&topic_name).unwrap(),
        ),
        BackendType::MpmcShm => (
            Topic::new(&topic_name).unwrap(),
            Topic::new(&topic_name).unwrap(),
        ),
    };

    // Warmup
    for _ in 0..DEFAULT_WARMUP {
        let msg = Imu::new();
        tx.send(msg, &mut None).unwrap();
        let _ = rx.recv(&mut None);
    }

    // Measurement
    let mut latencies = Vec::with_capacity(iterations);
    for _ in 0..iterations {
        let mut msg = Imu::new();
        msg.linear_acceleration = [0.0, 0.0, 9.81];
        msg.angular_velocity = [0.01, 0.02, 0.0];
        let start = timer.start();
        tx.send(msg, &mut None).unwrap();
        let _ = rx.recv(&mut None);
        latencies.push(timer.elapsed_ns(start));
    }

    build_result(
        &format!("{}_{}", backend_name, "Imu"),
        std::mem::size_of::<Imu>(),
        latencies,
        iterations,
        platform,
    )
}

fn benchmark_laserscan(
    backend_name: &str,
    backend: BackendType,
    iterations: usize,
    platform: &horus_benchmarks::PlatformInfo,
) -> BenchmarkResult {
    let topic_name = format!("bench_laser_{}_{}", backend_name, std::process::id());
    let timer = PrecisionTimer::new();

    let (tx, rx): (Topic<LaserScan>, Topic<LaserScan>) = match backend {
        BackendType::SpscIntra => Topic::spsc_intra(&topic_name),
        BackendType::SpscShm => (
            Topic::producer(&topic_name).unwrap(),
            Topic::consumer(&topic_name).unwrap(),
        ),
        BackendType::MpmcShm => (
            Topic::new(&topic_name).unwrap(),
            Topic::new(&topic_name).unwrap(),
        ),
    };

    // Warmup (fewer due to size)
    for _ in 0..DEFAULT_WARMUP / 5 {
        let mut msg = LaserScan::new();
        for i in 0..360 {
            msg.ranges[i] = 5.0 + (i as f32 * 0.01);
        }
        tx.send(msg, &mut None).unwrap();
        let _ = rx.recv(&mut None);
    }

    // Measurement
    let mut latencies = Vec::with_capacity(iterations);
    for seq in 0..iterations {
        let mut msg = LaserScan::new();
        for i in 0..360 {
            msg.ranges[i] = 5.0 + ((i + seq) as f32 * 0.01);
        }
        let start = timer.start();
        tx.send(msg, &mut None).unwrap();
        let _ = rx.recv(&mut None);
        latencies.push(timer.elapsed_ns(start));
    }

    build_result(
        &format!("{}_{}", backend_name, "LaserScan"),
        std::mem::size_of::<LaserScan>(),
        latencies,
        iterations,
        platform,
    )
}

fn benchmark_jointcmd(
    backend_name: &str,
    backend: BackendType,
    iterations: usize,
    platform: &horus_benchmarks::PlatformInfo,
) -> BenchmarkResult {
    let topic_name = format!("bench_joint_{}_{}", backend_name, std::process::id());
    let timer = PrecisionTimer::new();

    let (tx, rx): (Topic<JointCommand>, Topic<JointCommand>) = match backend {
        BackendType::SpscIntra => Topic::spsc_intra(&topic_name),
        BackendType::SpscShm => (
            Topic::producer(&topic_name).unwrap(),
            Topic::consumer(&topic_name).unwrap(),
        ),
        BackendType::MpmcShm => (
            Topic::new(&topic_name).unwrap(),
            Topic::new(&topic_name).unwrap(),
        ),
    };

    // Warmup
    for _ in 0..DEFAULT_WARMUP {
        let mut msg = JointCommand::new();
        msg.add_position("shoulder_pan", 0.5).ok();
        msg.add_position("shoulder_lift", -0.3).ok();
        msg.add_position("elbow", 1.2).ok();
        msg.add_position("wrist_1", 0.0).ok();
        msg.add_position("wrist_2", -0.5).ok();
        msg.add_position("wrist_3", 0.1).ok();
        tx.send(msg, &mut None).unwrap();
        let _ = rx.recv(&mut None);
    }

    // Measurement
    let mut latencies = Vec::with_capacity(iterations);
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
        tx.send(msg, &mut None).unwrap();
        let _ = rx.recv(&mut None);
        latencies.push(timer.elapsed_ns(start));
    }

    build_result(
        &format!("{}_{}", backend_name, "JointCommand"),
        std::mem::size_of::<JointCommand>(),
        latencies,
        iterations,
        platform,
    )
}

fn benchmark_pointcloud(
    backend_name: &str,
    backend: BackendType,
    iterations: usize,
    platform: &horus_benchmarks::PlatformInfo,
) -> BenchmarkResult {
    let topic_name = format!("bench_pc_{}_{}", backend_name, std::process::id());
    let timer = PrecisionTimer::new();

    let (tx, rx): (Topic<PointCloud>, Topic<PointCloud>) = match backend {
        BackendType::SpscIntra => Topic::spsc_intra(&topic_name),
        BackendType::SpscShm => (
            Topic::producer(&topic_name).unwrap(),
            Topic::consumer(&topic_name).unwrap(),
        ),
        BackendType::MpmcShm => (
            Topic::new(&topic_name).unwrap(),
            Topic::new(&topic_name).unwrap(),
        ),
    };

    // Create a representative point cloud (4000 points = ~48KB)
    use horus_library::messages::geometry::Point3;
    let num_points = 4000;
    let points: Vec<Point3> = (0..num_points)
        .map(|i| {
            let angle = (i as f64) * std::f64::consts::PI * 2.0 / 360.0;
            let r = 5.0 + (i as f64 * 0.001);
            Point3::new(r * angle.cos(), r * angle.sin(), (i as f64) * 0.01)
        })
        .collect();

    // Warmup
    for _ in 0..iterations.min(100) {
        let msg = PointCloud::xyz(&points);
        tx.send(msg, &mut None).unwrap();
        let _ = rx.recv(&mut None);
    }

    // Measurement
    let mut latencies = Vec::with_capacity(iterations);
    for _ in 0..iterations {
        let msg = PointCloud::xyz(&points);
        let _msg_size = msg.data.len();
        let start = timer.start();
        tx.send(msg, &mut None).unwrap();
        let _ = rx.recv(&mut None);
        latencies.push(timer.elapsed_ns(start));
    }

    // PointCloud has variable size, estimate based on data
    let estimated_size = 108 + (num_points * 12); // header + xyz data

    build_result(
        &format!("{}_{}", backend_name, "PointCloud"),
        estimated_size,
        latencies,
        iterations,
        platform,
    )
}

fn build_result(
    name: &str,
    message_size: usize,
    latencies: Vec<u64>,
    iterations: usize,
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
        deadline_misses: 0,
        deadline_threshold_ns: 0,
        run_variance: 0.0,
    };

    let total_ns: u64 = latencies.iter().sum();
    let duration_secs = total_ns as f64 / 1_000_000_000.0;

    let throughput = ThroughputMetrics {
        messages_per_sec: latencies.len() as f64 / duration_secs.max(0.001),
        bytes_per_sec: (latencies.len() * message_size) as f64 / duration_secs.max(0.001),
        total_messages: latencies.len() as u64,
        total_bytes: (latencies.len() * message_size) as u64,
        duration_secs,
    };

    BenchmarkResult {
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
