//! HORUS IPC Latency Benchmark — All Backend Paths
//!
//! Measures true per-message latency across every Topic backend route.
//!
//! ## Methodology
//!
//! **Intra-process** (DirectChannel, SpscIntra, MpscIntra, SpmcIntra, MpmcIntra):
//! - Measures `send()` latency via RDTSC with overhead subtraction
//! - Producer and consumer in same process, consumer on separate core
//!
//! **Cross-process** (SpscShm, MpscShm, SpmcShm, PodShm):
//! - One-way latency via RDTSC cycle timestamps embedded in `CmdVel.stamp_nanos`
//! - Producer writes `rdtsc()` → `send()`, consumer reads `recv()` → `rdtscp()`
//! - Requires `constant_tsc` for cross-core TSC synchronization
//! - No `yield_now()` or `sleep()` in measurement hot loops
//!
//! **Note**: MpmcShm only activates for non-POD types (POD multi-pub/sub gets PodShm).
//! Since CmdVel is POD, MpmcShm is not directly benchmarked here — it shares the same
//! dispatch paths as PodShm with added serialization overhead.
//!
//! ## Statistical Analysis
//!
//! Each scenario: 100K samples, Tukey IQR outlier filtering, bootstrap 95% CI,
//! full percentile distribution (p1–p99.99), CPU-pinned processes.
//!
//! ## Usage
//!
//! ```sh
//! # Standard run (human-readable)
//! cargo run --release --bin all_paths_latency
//!
//! # Machine-readable JSON output for CI/regression tracking
//! cargo run --release --bin all_paths_latency -- --json results.json
//!
//! # Best results: set performance governor first
//! sudo cpupower frequency-set -g performance
//! ```

use horus::prelude::Topic;
use horus_benchmarks::output::{write_json_report, BenchmarkReport};
use horus_benchmarks::platform::{detect_platform, has_constant_tsc, PlatformInfo};
use horus_benchmarks::set_cpu_affinity;
use horus_benchmarks::stats::Statistics;
use horus_benchmarks::timing::{rdtsc, rdtscp, serialize, PrecisionTimer, RdtscCalibration};
use horus_benchmarks::{
    BenchmarkConfig, BenchmarkResult, DeterminismMetrics, ThroughputMetrics,
};
use horus_library::messages::cmd_vel::CmdVel;
use std::hint::spin_loop;
use std::process::{Command, Stdio};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::thread;
use std::time::{Duration, Instant};

// ============================================================================
// Configuration
// ============================================================================

const ITERATIONS: u64 = 100_000;
const WARMUP: u64 = 5_000;
const MIGRATION_BOOT: u64 = 200;
const TIMEOUT: Duration = Duration::from_secs(30);

/// PodShm publishers need far more messages than other scenarios because:
/// 1. Setup (spawn consumer, wait for topology, trigger migration) takes ~350ms
/// 2. At ~100ns/msg SHM speed, 105K msgs finishes in ~10ms
/// 3. Publishers must still be running during the measurement phase
/// 10M msgs × ~100ns = ~1s — enough headroom for setup + warmup + measurement.
const PODSHM_MSGS_PER_PUB: u64 = 10_000_000;

/// CPU core pinning assignments.
/// Spaced by 2 to avoid hyperthreading siblings on most Intel/AMD layouts.
const CORE_MAIN: usize = 0;
const CORE_AUX: usize = 2;
const CORE_PUB2: usize = 4;
const CORE_CHILD_CONS: usize = 6;
const CORE_CONS2: usize = 8;

/// Width of the output box (interior, excluding border characters)
const BOX_W: usize = 72;

// ============================================================================
// Result
// ============================================================================

struct ScenarioResult {
    name: &'static str,
    backend: String,
    expected_backend: &'static str,
    measurement: &'static str,
    latencies_ns: Vec<u64>,
    total_sent: u64,
    total_received: u64,
    note: Option<&'static str>,
}

impl ScenarioResult {
    fn stats(&self) -> Statistics {
        if self.latencies_ns.is_empty() {
            return Statistics::from_samples(&[], 95.0, false);
        }
        Statistics::from_samples(&self.latencies_ns, 95.0, true)
    }

    fn backend_ok(&self) -> bool {
        self.backend.contains(self.expected_backend)
    }

    fn loss_pct(&self) -> f64 {
        if self.total_sent == 0 {
            return 0.0;
        }
        let loss = self.total_sent.saturating_sub(self.total_received);
        loss as f64 / self.total_sent as f64 * 100.0
    }

    /// Short backend name for summary table (e.g. "SpscShm" from "SpscShm (Adaptive)")
    fn backend_short(&self) -> &str {
        self.backend
            .split_once(" (")
            .map(|(name, _)| name)
            .unwrap_or(&self.backend)
    }
}

// ============================================================================
// Main
// ============================================================================

fn main() {
    let args: Vec<String> = std::env::args().collect();

    // --- Child process entry points ---
    if args.len() >= 5 && args[1] == "--child-publisher" {
        let paced = args.get(5).map(|s| s == "--paced").unwrap_or(false);
        run_child_publisher(&args[2], args[3].parse().unwrap(), args[4].parse().unwrap(), paced);
        return;
    }
    if args.len() >= 5 && args[1] == "--child-consumer" {
        run_child_consumer(&args[2], args[3].parse().unwrap(), args[4].parse().unwrap());
        return;
    }
    if args.len() >= 4 && args[1] == "--child-atomic-writer" {
        run_child_atomic_writer(&args[2], args[3].parse().unwrap());
        return;
    }

    // --- Parse CLI flags ---
    let json_path = args
        .windows(2)
        .find(|w| w[0] == "--json")
        .map(|w| w[1].clone());

    // --- Initialization ---
    let platform = detect_platform();
    let timer = PrecisionTimer::with_calibration(500);
    let cal = timer.calibration();

    print_header(&platform, cal);
    validate_platform(&platform);
    let _ = set_cpu_affinity(CORE_MAIN);

    let mut results: Vec<ScenarioResult> = Vec::new();

    // === Intra-process (5 scenarios) ===
    println!(
        "{} Intra-Process {}",
        "───",
        "─".repeat(BOX_W - 19)
    );
    println!();

    let r = bench_direct_channel(&timer);
    print_detail(&r);
    results.push(r);

    let r = bench_spsc_intra(&timer);
    print_detail(&r);
    results.push(r);

    let r = bench_mpsc_intra(&timer);
    print_detail(&r);
    results.push(r);

    let r = bench_spmc_intra(&timer);
    print_detail(&r);
    results.push(r);

    let r = bench_mpmc_intra(&timer);
    print_detail(&r);
    results.push(r);

    // === Cross-process (4 scenarios) ===
    println!(
        "{} Cross-Process (RDTSC-in-payload) {}",
        "───",
        "─".repeat(BOX_W - 37)
    );
    println!();

    let r = bench_spsc_shm(&timer);
    print_detail(&r);
    results.push(r);

    let r = bench_mpsc_shm(&timer);
    print_detail(&r);
    results.push(r);

    let r = bench_spmc_shm(&timer);
    print_detail(&r);
    results.push(r);

    let r = bench_pod_shm(&timer);
    print_detail(&r);
    results.push(r);

    // === Raw atomic probe (hardware floor) ===
    println!();
    println!(
        "{} Hardware Floor (raw SHM atomic) {}",
        "───",
        "─".repeat(BOX_W - 38)
    );
    println!();

    let r = bench_raw_atomic(&timer);
    print_detail(&r);
    results.push(r);

    // === Summary ===
    print_summary(&results);
    print_methodology(cal);

    // === JSON output ===
    if let Some(path) = json_path {
        write_json_output(&path, &platform, &results);
    }
}

// ============================================================================
// Platform header & validation
// ============================================================================

fn print_header(platform: &PlatformInfo, cal: &RdtscCalibration) {
    let now = chrono::Local::now().format("%Y-%m-%d %H:%M:%S %Z");
    let commit = detect_git_commit_short();

    println!();
    println!("{}", box_top());
    println!("{}", box_center("HORUS IPC Latency Benchmark v2.0"));
    println!(
        "{}",
        box_center("Per-Message RDTSC-Instrumented Latency")
    );
    println!("{}", box_sep());

    let model = truncate(&platform.cpu.model, BOX_W - 8);
    println!("{}", box_left(&format!("CPU: {}", model)));
    println!(
        "{}",
        box_left(&format!(
            "Cores: {} physical / {} logical, NUMA: {} node(s)",
            platform.cpu.physical_cores, platform.cpu.logical_cores, platform.numa_nodes,
        ))
    );
    println!(
        "{}",
        box_left(&format!(
            "RDTSC: {:.2} GHz, ~{}ns overhead, constant_tsc: {}",
            cal.freq_hz / 1e9,
            cal.cycles_to_ns(cal.overhead_cycles),
            if has_constant_tsc() { "YES" } else { "NO" },
        ))
    );

    let gov = platform.cpu_governor.as_deref().unwrap_or("unknown");
    println!(
        "{}",
        box_left(&format!(
            "Governor: {}, VM: {}",
            gov,
            if platform.virtualized { "Yes" } else { "No" },
        ))
    );
    println!(
        "{}",
        box_left(&format!(
            "Pinning: main={}, aux={}, pub2={}, cons={}, cons2={}",
            CORE_MAIN, CORE_AUX, CORE_PUB2, CORE_CHILD_CONS, CORE_CONS2,
        ))
    );
    println!(
        "{}",
        box_left(&format!(
            "Config: {} iterations, {} warmup, {} boot msgs",
            ITERATIONS, WARMUP, MIGRATION_BOOT,
        ))
    );

    println!("{}", box_sep());
    println!("{}", box_left(&format!("Date: {}", now)));
    if let Some(ref c) = commit {
        println!("{}", box_left(&format!("Commit: {}", c)));
    }
    println!("{}", box_bot());
    println!();
}

fn validate_platform(platform: &PlatformInfo) {
    let mut warnings = Vec::new();

    if !has_constant_tsc() {
        warnings
            .push("constant_tsc not detected -- cross-process RDTSC may be inaccurate".to_string());
    }
    if let Some(ref gov) = platform.cpu_governor {
        if gov != "performance" {
            warnings.push(format!(
                "CPU governor is '{}' (want 'performance'). Run: sudo cpupower frequency-set -g performance",
                gov
            ));
        }
    }
    if platform.virtualized {
        warnings.push("Running in VM -- RDTSC timing may be unreliable".to_string());
    }

    if !warnings.is_empty() {
        for w in &warnings {
            eprintln!("  WARNING: {}", w);
        }
        println!();
    }
}

// ============================================================================
// Intra-Process Benchmarks
// ============================================================================

/// DirectChannel -- same thread, no atomics.
/// Measures send() latency with RDTSC overhead subtracted.
///
/// **Batched sends**: We send in batches of 128, then drain outside the
/// measurement window. This avoids icache/branch-predictor pollution from
/// alternating send/recv calls, which inflated DirectChannel latency to
/// ~36ns vs SpscIntra's ~7ns despite DirectChannel being simpler.
fn bench_direct_channel(timer: &PrecisionTimer) -> ScenarioResult {
    let topic: Topic<CmdVel> = Topic::new("bench_dc_v2").unwrap();
    let msg = CmdVel::with_timestamp(1.5, 0.8, 0);
    let cal = timer.calibration();
    let overhead = cal.overhead_cycles;

    // Warmup: batch send + drain to warm caches without polluting measurement
    for _ in 0..WARMUP {
        topic.send(msg);
    }
    while topic.recv().is_some() {}

    let backend = topic.backend_type().to_string();

    // Measure send-only latency in batches.
    // Ring capacity is 256 for CmdVel (16 bytes). Use batch of 128 (half capacity).
    // Each batch: measure 128 sends, then drain outside measurement window.
    const BATCH: u64 = 128;
    let mut latencies = Vec::with_capacity(ITERATIONS as usize);
    let mut sent = 0u64;
    while sent < ITERATIONS {
        let n = std::cmp::min(BATCH, ITERATIONS - sent);
        for _ in 0..n {
            serialize();
            let start = rdtsc();
            topic.send(std::hint::black_box(msg));
            let end = rdtscp();
            latencies.push(cal.cycles_to_ns(end.wrapping_sub(start).saturating_sub(overhead)));
        }
        sent += n;
        // Drain outside measurement window — icache stays warm for sends
        while topic.recv().is_some() {}
    }

    ScenarioResult {
        name: "SameThread",
        backend,
        expected_backend: "DirectChannel",
        measurement: "send",
        latencies_ns: latencies,
        total_sent: ITERATIONS,
        total_received: ITERATIONS,
        note: None,
    }
}

/// SpscIntra -- cross-thread 1P-1C.
/// Measures send() latency on producer thread while consumer spins on another core.
fn bench_spsc_intra(timer: &PrecisionTimer) -> ScenarioResult {
    let topic_name = format!("bench_si_v2_{}", std::process::id());
    let producer: Topic<CmdVel> = Topic::new(&topic_name).unwrap();
    let consumer: Topic<CmdVel> = Topic::new(&topic_name).unwrap();
    let msg = CmdVel::with_timestamp(1.5, 0.8, 0);
    let cal = timer.calibration();
    let overhead = cal.overhead_cycles;

    let done = Arc::new(AtomicBool::new(false));
    let done_c = done.clone();
    let backend_out = Arc::new(std::sync::Mutex::new(String::new()));
    let backend_c = backend_out.clone();
    let ready = Arc::new(AtomicBool::new(false));
    let ready_c = ready.clone();

    // Pre-fill ring to establish SpscIntra before consumer starts
    for _ in 0..2000 {
        producer.send(msg);
    }

    // Consumer thread pinned to separate core
    let handle = thread::spawn(move || {
        let _ = set_cpu_affinity(CORE_AUX);
        let mut count = 0u64;
        let deadline = Instant::now() + Duration::from_secs(5);
        while count < 1000 && Instant::now() < deadline {
            if consumer.recv().is_some() {
                count += 1;
            } else {
                spin_loop();
            }
        }
        *backend_c.lock().unwrap() = consumer.backend_type().to_string();
        ready_c.store(true, Ordering::Release);

        while !done_c.load(Ordering::Relaxed) {
            if consumer.recv().is_none() {
                spin_loop();
            }
        }
        while consumer.recv().is_some() {}
    });

    // Wait for consumer ready
    while !ready.load(Ordering::Acquire) {
        producer.send(msg);
        spin_loop();
    }

    // Extra warmup after consumer is ready
    for _ in 0..WARMUP {
        producer.send(msg);
    }
    thread::sleep(Duration::from_millis(5));

    // Measure send latency
    let mut latencies = Vec::with_capacity(ITERATIONS as usize);
    for _ in 0..ITERATIONS {
        serialize();
        let start = rdtsc();
        producer.send(msg);
        let end = rdtscp();
        latencies.push(cal.cycles_to_ns(end.wrapping_sub(start).saturating_sub(overhead)));
    }

    thread::sleep(Duration::from_millis(50));
    done.store(true, Ordering::Relaxed);
    handle.join().unwrap();

    let backend = backend_out.lock().unwrap().clone();

    ScenarioResult {
        name: "CrossThread-1P1C",
        backend,
        expected_backend: "Intra",
        measurement: "send",
        latencies_ns: latencies,
        total_sent: ITERATIONS,
        total_received: ITERATIONS,
        note: None,
    }
}

/// MpscIntra -- cross-thread 2P-1C.
/// Measures send() latency on main thread while second producer runs on another core.
/// Multi-producer contention via CAS loop.
fn bench_mpsc_intra(timer: &PrecisionTimer) -> ScenarioResult {
    let topic_name = format!("bench_mi_{}", std::process::id());
    let producer1: Topic<CmdVel> = Topic::new(&topic_name).unwrap();
    let consumer: Topic<CmdVel> = Topic::new(&topic_name).unwrap();
    let producer2: Topic<CmdVel> = Topic::new(&topic_name).unwrap();
    let msg = CmdVel::with_timestamp(1.5, 0.8, 0);
    let cal = timer.calibration();
    let overhead = cal.overhead_cycles;

    let done = Arc::new(AtomicBool::new(false));
    let cons_ready = Arc::new(AtomicBool::new(false));
    let p2_ready = Arc::new(AtomicBool::new(false));

    // Pre-fill from producer1 only (registers it on main thread)
    for _ in 0..2000 {
        producer1.send(msg);
    }

    // Consumer thread pinned to CORE_AUX
    let done_c = done.clone();
    let cons_ready_c = cons_ready.clone();
    let cons_handle = thread::spawn(move || {
        let _ = set_cpu_affinity(CORE_AUX);
        let mut count = 0u64;
        let deadline = Instant::now() + Duration::from_secs(5);
        while count < 1000 && Instant::now() < deadline {
            if consumer.recv().is_some() {
                count += 1;
            } else {
                spin_loop();
            }
        }
        cons_ready_c.store(true, Ordering::Release);
        while !done_c.load(Ordering::Relaxed) {
            if consumer.recv().is_none() {
                spin_loop();
            }
        }
        while consumer.recv().is_some() {}
    });

    // Wait for consumer to register
    while !cons_ready.load(Ordering::Acquire) {
        producer1.send(msg);
        spin_loop();
    }

    // Producer2 thread pinned to CORE_PUB2
    let done_p2 = done.clone();
    let p2_ready_c = p2_ready.clone();
    let p2_handle = thread::spawn(move || {
        let _ = set_cpu_affinity(CORE_PUB2);
        // Send to register as 2nd publisher, triggers MpscIntra migration
        for _ in 0..2000 {
            producer2.send(msg);
        }
        p2_ready_c.store(true, Ordering::Release);
        // Keep sending to create realistic multi-producer contention
        while !done_p2.load(Ordering::Relaxed) {
            producer2.send(msg);
        }
    });

    // Wait for producer2 to register
    while !p2_ready.load(Ordering::Acquire) {
        producer1.send(msg);
        spin_loop();
    }

    // Extra warmup after all participants registered
    for _ in 0..WARMUP {
        producer1.send(msg);
    }
    thread::sleep(Duration::from_millis(5));

    let backend = producer1.backend_type().to_string();

    // Measure send() latency on main thread (contended with producer2)
    let mut latencies = Vec::with_capacity(ITERATIONS as usize);
    for _ in 0..ITERATIONS {
        serialize();
        let start = rdtsc();
        producer1.send(std::hint::black_box(msg));
        let end = rdtscp();
        latencies.push(cal.cycles_to_ns(end.wrapping_sub(start).saturating_sub(overhead)));
    }

    thread::sleep(Duration::from_millis(50));
    done.store(true, Ordering::Relaxed);
    cons_handle.join().unwrap();
    p2_handle.join().unwrap();

    ScenarioResult {
        name: "CrossThread-MP1C",
        backend,
        expected_backend: "Intra",
        measurement: "send",
        latencies_ns: latencies,
        total_sent: ITERATIONS,
        total_received: ITERATIONS,
        note: Some("contended: 2 producers active"),
    }
}

/// SpmcIntra -- cross-thread 1P-2C.
/// Measures send() latency while 2 consumer threads compete via CAS.
fn bench_spmc_intra(timer: &PrecisionTimer) -> ScenarioResult {
    let topic_name = format!("bench_smi_{}", std::process::id());
    let producer: Topic<CmdVel> = Topic::new(&topic_name).unwrap();
    let consumer1: Topic<CmdVel> = Topic::new(&topic_name).unwrap();
    let consumer2: Topic<CmdVel> = Topic::new(&topic_name).unwrap();
    let msg = CmdVel::with_timestamp(1.5, 0.8, 0);
    let cal = timer.calibration();
    let overhead = cal.overhead_cycles;

    let done = Arc::new(AtomicBool::new(false));
    let c1_ready = Arc::new(AtomicBool::new(false));
    let c2_ready = Arc::new(AtomicBool::new(false));

    // Pre-fill from producer (registers on main thread)
    for _ in 0..2000 {
        producer.send(msg);
    }

    // Consumer 1 thread pinned to CORE_AUX
    let done_c1 = done.clone();
    let c1_ready_c = c1_ready.clone();
    let c1_handle = thread::spawn(move || {
        let _ = set_cpu_affinity(CORE_AUX);
        let mut count = 0u64;
        let deadline = Instant::now() + Duration::from_secs(5);
        while count < 500 && Instant::now() < deadline {
            if consumer1.recv().is_some() {
                count += 1;
            } else {
                spin_loop();
            }
        }
        c1_ready_c.store(true, Ordering::Release);
        while !done_c1.load(Ordering::Relaxed) {
            if consumer1.recv().is_none() {
                spin_loop();
            }
        }
        while consumer1.recv().is_some() {}
    });

    // Wait for consumer1 to register
    while !c1_ready.load(Ordering::Acquire) {
        producer.send(msg);
        spin_loop();
    }

    // Consumer 2 thread pinned to CORE_CONS2
    let done_c2 = done.clone();
    let c2_ready_c = c2_ready.clone();
    let c2_handle = thread::spawn(move || {
        let _ = set_cpu_affinity(CORE_CONS2);
        let mut count = 0u64;
        let deadline = Instant::now() + Duration::from_secs(5);
        while count < 500 && Instant::now() < deadline {
            if consumer2.recv().is_some() {
                count += 1;
            } else {
                spin_loop();
            }
        }
        c2_ready_c.store(true, Ordering::Release);
        while !done_c2.load(Ordering::Relaxed) {
            if consumer2.recv().is_none() {
                spin_loop();
            }
        }
        while consumer2.recv().is_some() {}
    });

    // Wait for consumer2 to register
    while !c2_ready.load(Ordering::Acquire) {
        producer.send(msg);
        spin_loop();
    }

    // Extra warmup after all participants registered
    for _ in 0..WARMUP {
        producer.send(msg);
    }
    thread::sleep(Duration::from_millis(5));

    let backend = producer.backend_type().to_string();

    // Measure send() latency (single producer, 2 consumers draining)
    let mut latencies = Vec::with_capacity(ITERATIONS as usize);
    for _ in 0..ITERATIONS {
        serialize();
        let start = rdtsc();
        producer.send(std::hint::black_box(msg));
        let end = rdtscp();
        latencies.push(cal.cycles_to_ns(end.wrapping_sub(start).saturating_sub(overhead)));
    }

    thread::sleep(Duration::from_millis(50));
    done.store(true, Ordering::Relaxed);
    c1_handle.join().unwrap();
    c2_handle.join().unwrap();

    ScenarioResult {
        name: "CrossThread-1PMC",
        backend,
        expected_backend: "Intra",
        measurement: "send",
        latencies_ns: latencies,
        total_sent: ITERATIONS,
        total_received: ITERATIONS,
        note: None,
    }
}

/// MpmcIntra -- cross-thread 2P-2C.
/// Measures send() latency with multi-producer contention and multi-consumer drain.
fn bench_mpmc_intra(timer: &PrecisionTimer) -> ScenarioResult {
    let topic_name = format!("bench_mmi_{}", std::process::id());
    let producer1: Topic<CmdVel> = Topic::new(&topic_name).unwrap();
    let consumer1: Topic<CmdVel> = Topic::new(&topic_name).unwrap();
    let consumer2: Topic<CmdVel> = Topic::new(&topic_name).unwrap();
    let producer2: Topic<CmdVel> = Topic::new(&topic_name).unwrap();
    let msg = CmdVel::with_timestamp(1.5, 0.8, 0);
    let cal = timer.calibration();
    let overhead = cal.overhead_cycles;

    let done = Arc::new(AtomicBool::new(false));
    let c1_ready = Arc::new(AtomicBool::new(false));
    let c2_ready = Arc::new(AtomicBool::new(false));
    let p2_ready = Arc::new(AtomicBool::new(false));

    // Pre-fill from producer1 (registers on main thread)
    for _ in 0..2000 {
        producer1.send(msg);
    }

    // Consumer 1 thread pinned to CORE_AUX
    let done_c1 = done.clone();
    let c1_ready_c = c1_ready.clone();
    let c1_handle = thread::spawn(move || {
        let _ = set_cpu_affinity(CORE_AUX);
        let mut count = 0u64;
        let deadline = Instant::now() + Duration::from_secs(5);
        while count < 500 && Instant::now() < deadline {
            if consumer1.recv().is_some() {
                count += 1;
            } else {
                spin_loop();
            }
        }
        c1_ready_c.store(true, Ordering::Release);
        while !done_c1.load(Ordering::Relaxed) {
            if consumer1.recv().is_none() {
                spin_loop();
            }
        }
        while consumer1.recv().is_some() {}
    });

    // Wait for consumer1
    while !c1_ready.load(Ordering::Acquire) {
        producer1.send(msg);
        spin_loop();
    }

    // Consumer 2 thread pinned to CORE_CONS2
    let done_c2 = done.clone();
    let c2_ready_c = c2_ready.clone();
    let c2_handle = thread::spawn(move || {
        let _ = set_cpu_affinity(CORE_CONS2);
        let mut count = 0u64;
        let deadline = Instant::now() + Duration::from_secs(5);
        while count < 500 && Instant::now() < deadline {
            if consumer2.recv().is_some() {
                count += 1;
            } else {
                spin_loop();
            }
        }
        c2_ready_c.store(true, Ordering::Release);
        while !done_c2.load(Ordering::Relaxed) {
            if consumer2.recv().is_none() {
                spin_loop();
            }
        }
        while consumer2.recv().is_some() {}
    });

    // Wait for consumer2
    while !c2_ready.load(Ordering::Acquire) {
        producer1.send(msg);
        spin_loop();
    }

    // Producer2 thread pinned to CORE_PUB2
    let done_p2 = done.clone();
    let p2_ready_c = p2_ready.clone();
    let p2_handle = thread::spawn(move || {
        let _ = set_cpu_affinity(CORE_PUB2);
        for _ in 0..2000 {
            producer2.send(msg);
        }
        p2_ready_c.store(true, Ordering::Release);
        while !done_p2.load(Ordering::Relaxed) {
            producer2.send(msg);
        }
    });

    // Wait for producer2
    while !p2_ready.load(Ordering::Acquire) {
        producer1.send(msg);
        spin_loop();
    }

    // Extra warmup after all registered
    for _ in 0..WARMUP {
        producer1.send(msg);
    }
    thread::sleep(Duration::from_millis(5));

    let backend = producer1.backend_type().to_string();

    // Measure send() latency (contended: 2 producers, 2 consumers)
    let mut latencies = Vec::with_capacity(ITERATIONS as usize);
    for _ in 0..ITERATIONS {
        serialize();
        let start = rdtsc();
        producer1.send(std::hint::black_box(msg));
        let end = rdtscp();
        latencies.push(cal.cycles_to_ns(end.wrapping_sub(start).saturating_sub(overhead)));
    }

    thread::sleep(Duration::from_millis(50));
    done.store(true, Ordering::Relaxed);
    c1_handle.join().unwrap();
    c2_handle.join().unwrap();
    p2_handle.join().unwrap();

    ScenarioResult {
        name: "CrossThread-MPMC",
        backend,
        expected_backend: "Intra",
        measurement: "send",
        latencies_ns: latencies,
        total_sent: ITERATIONS,
        total_received: ITERATIONS,
        note: Some("contended: 2 producers, 2 consumers"),
    }
}

// ============================================================================
// Cross-Process Benchmarks
// ============================================================================

/// SpscShm -- cross-process 1 publisher, 1 consumer.
/// Parent = consumer, child = publisher.
/// Measures one-way latency via RDTSC timestamps in message payload.
fn bench_spsc_shm(timer: &PrecisionTimer) -> ScenarioResult {
    let cal = timer.calibration();
    let topic_name = format!("bench_spsc_{}", std::process::id());
    let consumer: Topic<CmdVel> = Topic::new(&topic_name).unwrap();
    let _ = consumer.recv(); // Register as consumer

    let child_count = WARMUP + ITERATIONS;
    // Paced publisher: prevents ring overflow that causes queuing delay.
    // Without pacing, producer outruns consumer → ring fills → measured latency
    // shows queuing delay (~3µs) instead of true wire latency (~300ns).
    let mut child = spawn_paced_publisher(&topic_name, child_count, CORE_AUX);

    // Wait for migration (child sends MIGRATION_BOOT + sleeps)
    let migration_recv = wait_for_messages(&consumer, 100, Duration::from_secs(10));
    let backend = consumer.backend_type().to_string();

    // Collect: first WARMUP discarded, then ITERATIONS measured
    let (latencies, measure_recv) = collect_cross_proc(&consumer, WARMUP, ITERATIONS, cal);
    child.wait().ok();

    let total_received = migration_recv + measure_recv;
    ScenarioResult {
        name: "CrossProc-1P1C",
        backend,
        expected_backend: "Shm",
        measurement: "one-way",
        latencies_ns: latencies,
        total_sent: MIGRATION_BOOT + child_count,
        total_received,
        note: None,
    }
}

/// MpscShm -- cross-process 2 publishers, 1 consumer.
/// Parent = consumer, 2 children = publishers.
fn bench_mpsc_shm(timer: &PrecisionTimer) -> ScenarioResult {
    let cal = timer.calibration();
    let topic_name = format!("bench_mpsc_{}", std::process::id());
    let consumer: Topic<CmdVel> = Topic::new(&topic_name).unwrap();
    let _ = consumer.recv();

    let msgs_per_pub = (WARMUP + ITERATIONS) / 2;

    // Paced publishers: 2 publishers into 1 consumer would overflow the ring
    // instantly, causing 26µs queuing delay instead of true wire latency.
    let mut pub1 = spawn_paced_publisher(&topic_name, msgs_per_pub, CORE_AUX);
    let migration1 = wait_for_messages(&consumer, 50, Duration::from_secs(10));

    let mut pub2 = spawn_paced_publisher(&topic_name, msgs_per_pub, CORE_PUB2);
    let migration2 = wait_for_messages(&consumer, 50, Duration::from_secs(10));

    let backend = consumer.backend_type().to_string();

    let (latencies, measure_recv) = collect_cross_proc(&consumer, WARMUP, ITERATIONS, cal);
    pub1.wait().ok();
    pub2.wait().ok();

    let total_received = migration1 + migration2 + measure_recv;

    ScenarioResult {
        name: "CrossProc-2P1C",
        backend,
        expected_backend: "Shm",
        measurement: "one-way",
        latencies_ns: latencies,
        total_sent: MIGRATION_BOOT * 2 + msgs_per_pub * 2,
        total_received,
        note: None,
    }
}

/// SpmcShm -- cross-process 1 publisher, 2 consumers.
/// Parent = consumer 1, child = consumer 2, child = publisher.
/// Measures one-way latency via RDTSC timestamps in message payload.
fn bench_spmc_shm(timer: &PrecisionTimer) -> ScenarioResult {
    let cal = timer.calibration();
    let topic_name = format!("bench_spmc_{}", std::process::id());
    let consumer: Topic<CmdVel> = Topic::new(&topic_name).unwrap();
    let _ = consumer.recv(); // Register parent as consumer

    // Spawn child consumer first (registers as 2nd subscriber)
    let child_count = WARMUP + ITERATIONS;
    let mut child_cons = spawn_consumer(&topic_name, child_count, CORE_CHILD_CONS);
    thread::sleep(Duration::from_millis(200));

    // Paced publisher: even with 2 consumers, the parent consumer (doing measurement
    // work: RDTSC + Vec::push) is slower, so CAS contention causes bursty delivery.
    // Pacing prevents ring overflow that inflates measured latency.
    let mut child_pub = spawn_paced_publisher(&topic_name, child_count, CORE_AUX);

    // Wait for migration (1 pub, 2 subs, cross-process → SpmcShm)
    let migration_recv = wait_for_messages(&consumer, 100, Duration::from_secs(10));
    let backend = consumer.backend_type().to_string();

    // Collect one-way latencies
    let (latencies, measure_recv) = collect_cross_proc(&consumer, WARMUP, ITERATIONS, cal);
    child_pub.wait().ok();
    child_cons.wait().ok();

    let total_received = migration_recv + measure_recv;
    ScenarioResult {
        name: "CrossProc-1PMC",
        backend,
        expected_backend: "SpmcShm",
        measurement: "one-way",
        latencies_ns: latencies,
        total_sent: MIGRATION_BOOT + child_count,
        total_received,
        note: None,
    }
}

/// PodShm -- cross-process 2 publishers, 2 consumers, POD type.
/// Parent + child = consumers, 2 children = publishers.
///
/// For POD types (like CmdVel), multi-pub/multi-sub cross-process always selects
/// PodShm (zero-copy atomic slot). MpmcShm only activates for non-POD types
/// (which require serialization). Both share the same dispatch paths.
///
/// Spawn order: consumers first, then publishers. This ensures PodShm is the
/// final topology and publishers are still running during measurement (previous
/// design spawned publishers first, which caused them to finish before measurement).
fn bench_pod_shm(timer: &PrecisionTimer) -> ScenarioResult {
    let cal = timer.calibration();
    let topic_name = format!("bench_pod_{}", std::process::id());
    let consumer: Topic<CmdVel> = Topic::new(&topic_name).unwrap();
    let _ = consumer.recv(); // Register as consumer

    // PodShm needs many more messages than other scenarios — publishers must
    // still be running during measurement (setup takes ~350ms, hot loop at
    // ~100ns/msg means 105K msgs finishes in ~10ms, way too fast).
    let msgs_per_pub = PODSHM_MSGS_PER_PUB;

    // Step 1: Spawn child consumer FIRST so both consumers are present before publishers.
    // Child consumer receives from both publishers (broadcast), so count = msgs_per_pub * 2.
    let mut child_cons = spawn_consumer(&topic_name, msgs_per_pub * 2, CORE_CHILD_CONS);
    thread::sleep(Duration::from_millis(200)); // Wait for child to register

    // Step 2: Spawn pub1 (paced) → topology: 1P, 2S, cross-proc → SpmcShm
    // Paced publishers prevent ring overflow that causes the consumer to read
    // stale messages with old timestamps, inflating measured latency.
    let mut pub1 = spawn_paced_publisher(&topic_name, msgs_per_pub, CORE_AUX);
    let migration1 = wait_for_messages(&consumer, 100, Duration::from_secs(10));

    // Step 3: Spawn pub2 (paced) → topology: 2P, 2S, cross-proc, POD → PodShm migration
    let mut pub2 = spawn_paced_publisher(&topic_name, msgs_per_pub, CORE_PUB2);

    // Wait for pub2 to actually register (it sleeps 100ms at startup).
    // We need pubs=2 visible in the header before migration detection works.
    wait_for_topology(&consumer, 2, 2, Duration::from_secs(5));

    // Drain any queued messages from the SpmcShm era
    let migration2 = wait_for_messages(&consumer, 200, Duration::from_secs(5));

    // Step 4: Force migration check so parent detects PodShm before measurement
    consumer.check_migration_now();

    let backend = consumer.backend_type().to_string();

    // Step 5: Measure — publishers are still running their 10M-message hot loops
    let (latencies, measure_recv) = collect_cross_proc(&consumer, WARMUP, ITERATIONS, cal);

    // Don't wait for publishers to finish all 10M msgs — just kill them
    let _ = pub1.kill();
    let _ = pub2.kill();
    let _ = child_cons.kill();
    pub1.wait().ok();
    pub2.wait().ok();
    child_cons.wait().ok();

    let total_received = migration1 + migration2 + measure_recv;

    ScenarioResult {
        name: "CrossProc-PodShm",
        backend,
        expected_backend: "PodShm",
        measurement: "one-way",
        latencies_ns: latencies,
        total_sent: MIGRATION_BOOT * 2 + msgs_per_pub * 2,
        total_received,
        note: None,
    }
}

// ============================================================================
// Cross-Process Helpers
// ============================================================================

fn spawn_publisher(topic: &str, count: u64, core: usize) -> std::process::Child {
    Command::new(std::env::current_exe().unwrap())
        .args([
            "--child-publisher",
            topic,
            &count.to_string(),
            &core.to_string(),
        ])
        .stdin(Stdio::null())
        .stdout(Stdio::piped())
        .stderr(Stdio::inherit())
        .spawn()
        .expect("Failed to spawn child publisher")
}

/// Spawn a paced publisher that inserts spin_loops between sends.
/// Prevents ring overflow and queuing delay in 1P1C scenarios where
/// the producer would otherwise outrun the consumer.
fn spawn_paced_publisher(topic: &str, count: u64, core: usize) -> std::process::Child {
    Command::new(std::env::current_exe().unwrap())
        .args([
            "--child-publisher",
            topic,
            &count.to_string(),
            &core.to_string(),
            "--paced",
        ])
        .stdin(Stdio::null())
        .stdout(Stdio::piped())
        .stderr(Stdio::inherit())
        .spawn()
        .expect("Failed to spawn paced child publisher")
}

fn spawn_consumer(topic: &str, count: u64, core: usize) -> std::process::Child {
    Command::new(std::env::current_exe().unwrap())
        .args([
            "--child-consumer",
            topic,
            &count.to_string(),
            &core.to_string(),
        ])
        .stdin(Stdio::null())
        .stdout(Stdio::piped())
        .stderr(Stdio::inherit())
        .spawn()
        .expect("Failed to spawn child consumer")
}

/// Wait until the topic header shows at least `min_pubs` publishers and `min_subs` subscribers.
fn wait_for_topology(topic: &Topic<CmdVel>, min_pubs: u32, min_subs: u32, timeout: Duration) {
    let deadline = Instant::now() + timeout;
    while topic.pub_count() < min_pubs || topic.sub_count() < min_subs {
        // Drain any messages while waiting (keeps lease alive)
        let _ = topic.recv();
        spin_loop();
        if Instant::now() > deadline {
            eprintln!(
                "  [warn] topology timeout: wanted pubs>={} subs>={}, got pubs={} subs={}",
                min_pubs, min_subs, topic.pub_count(), topic.sub_count()
            );
            break;
        }
    }
}

/// Spin-receive until `count` messages arrive or timeout.
///
/// Deadline check is amortized (every 4096 polls) to avoid ~108ns Instant::now()
/// overhead on every iteration, which would dominate cross-process wire latency.
fn wait_for_messages(consumer: &Topic<CmdVel>, count: u64, timeout: Duration) -> u64 {
    let mut received = 0u64;
    let deadline = Instant::now() + timeout;
    let mut polls = 0u64;
    while received < count {
        polls += 1;
        if polls & 4095 == 0 && Instant::now() > deadline {
            break;
        }
        if consumer.recv().is_some() {
            received += 1;
        } else {
            spin_loop();
        }
    }
    received
}

/// Collect cross-process latencies from RDTSC timestamps in message payload.
///
/// Phase 1: Discard `warmup` messages (cache/TLB warming).
/// Phase 1.5: Drain all stale messages from the ring buffer.
/// Phase 2: Collect up to `iterations` per-message one-way latencies.
///
/// **Critical**: The drain phase between warmup and measurement eliminates stale
/// messages that accumulated in the ring while the consumer was processing warmup.
/// Without this drain, measurement would start with old-timestamp messages, inflating
/// latency by hundreds of nanoseconds.
///
/// **Critical**: Deadline and idle checks are amortized (every 4096 polls) to avoid
/// injecting ~108ns Instant::now() overhead into the measurement hot loop. This is
/// essential for accurate sub-microsecond latency measurement.
///
/// Returns (latencies_ns, total_messages_received_in_both_phases).
fn collect_cross_proc(
    consumer: &Topic<CmdVel>,
    warmup: u64,
    iterations: u64,
    cal: &RdtscCalibration,
) -> (Vec<u64>, u64) {
    let mut total = 0u64;
    let deadline = Instant::now() + TIMEOUT;
    let mut last_recv_cycles = rdtsc(); // Use RDTSC for idle detection too
    let idle_threshold = cal.ns_to_cycles(2_000_000_000); // 2 seconds in cycles (warmup)
    let idle_threshold_meas = cal.ns_to_cycles(500_000_000); // 500ms in cycles (measurement)
    let overhead = cal.overhead_cycles;

    // Phase 1: Warmup -- receive and discard (cache/TLB warming)
    let mut polls = 0u64;
    while total < warmup {
        polls += 1;
        if polls & 4095 == 0 && Instant::now() > deadline {
            break;
        }
        if consumer.recv().is_some() {
            total += 1;
            last_recv_cycles = rdtsc();
        } else {
            let now = rdtsc();
            if now.wrapping_sub(last_recv_cycles) > idle_threshold {
                break;
            }
            spin_loop();
        }
    }

    // No drain phase needed: producer pacing (~1µs/msg via 256 spin_loops) is
    // slow enough that the consumer processes each message before the next one
    // arrives. There is no queue buildup, so every message reflects true wire
    // latency (cache coherency + dispatch overhead), not queuing delay.

    // Phase 2: Measurement -- ZERO overhead hot loop
    // No Instant::now(), no unnecessary branches. Pure spin on recv().
    // RDTSC overhead (serialize+rdtsc on producer + rdtscp on consumer) is subtracted
    // from each sample, same as intra-process measurements.
    let mut latencies = Vec::with_capacity(iterations as usize);
    last_recv_cycles = rdtsc();
    polls = 0;
    while (latencies.len() as u64) < iterations {
        polls += 1;
        // Amortized deadline check: every 4096 polls (~1µs at full speed)
        if polls & 4095 == 0 && Instant::now() > deadline {
            break;
        }
        if let Some(msg) = consumer.recv() {
            let recv_cycles = rdtscp();
            let send_cycles = msg.stamp_nanos;
            let delta = recv_cycles.wrapping_sub(send_cycles).saturating_sub(overhead);
            latencies.push(cal.cycles_to_ns(delta));
            total += 1;
            last_recv_cycles = recv_cycles;
        } else {
            let now = rdtsc();
            if now.wrapping_sub(last_recv_cycles) > idle_threshold_meas {
                break; // Publisher likely finished
            }
            // No spin_loop() here — tight poll for minimum latency measurement.
            // PAUSE on Comet Lake costs ~140 cycles (~40ns), adding ~20ns avg
            // polling delay. For latency benchmarks, we want the tightest loop.
        }
    }

    (latencies, total)
}

// ============================================================================
// Child Process Entry Points
// ============================================================================

fn run_child_publisher(topic_name: &str, count: u64, core: usize, paced: bool) {
    let _ = set_cpu_affinity(core);
    let topic: Topic<CmdVel> = Topic::new(topic_name).unwrap();

    // Wait for parent consumer to register
    thread::sleep(Duration::from_millis(100));

    // Boot messages: trigger cross-process backend migration
    for _ in 0..MIGRATION_BOOT {
        serialize();
        let t = rdtsc();
        topic.send(CmdVel::with_timestamp(1.5, 0.8, t));
    }
    thread::sleep(Duration::from_millis(50));

    eprintln!(
        "  [pub] PID={} core={} backend={} pubs={} subs={}{}",
        std::process::id(),
        core,
        topic.backend_type(),
        topic.pub_count(),
        topic.sub_count(),
        if paced { " (paced)" } else { "" },
    );

    // === MEASUREMENT HOT LOOP ===
    // RDTSC timestamp embedded in payload. No yield, no sleep.
    if paced {
        // Paced mode: insert spin_loops between sends so the consumer ALWAYS
        // processes each message before the next one arrives. This ensures zero
        // queue buildup, so measured one-way latency reflects true wire latency
        // (cache coherency + dispatch overhead), not queuing delay.
        //
        // 256 spin_loops ≈ 1024ns (~1µs). Consumer processes each message in
        // ~150-300ns (SHM reads + RDTSC + Vec::push), so ~700ns of idle time
        // between messages — more than enough headroom.
        for _ in 0..count {
            serialize();
            let t = rdtsc();
            topic.send(CmdVel::with_timestamp(1.5, 0.8, t));
            for _ in 0..256 {
                spin_loop();
            }
        }
    } else {
        for _ in 0..count {
            serialize();
            let t = rdtsc();
            topic.send(CmdVel::with_timestamp(1.5, 0.8, t));
        }
    }
}

fn run_child_consumer(topic_name: &str, count: u64, core: usize) {
    let _ = set_cpu_affinity(core);
    let topic: Topic<CmdVel> = Topic::new(topic_name).unwrap();
    let _ = topic.recv(); // Register as subscriber
    thread::sleep(Duration::from_millis(50));

    let mut received = 0u64;
    let deadline = Instant::now() + TIMEOUT;
    while received < count && Instant::now() < deadline {
        if topic.recv().is_some() {
            received += 1;
        } else {
            spin_loop();
        }
    }
}

// ============================================================================
// Raw Atomic Probe: Hardware floor for cross-process latency
// ============================================================================

/// Measure the raw hardware floor for cross-process cache line transfer.
///
/// Uses a single AtomicU64 in shared memory (via HORUS Topic header's user field).
/// Writer: serialize() → rdtsc() → atomic store(ts) → paced wait
/// Reader: tight poll → rdtscp() → compute delta
///
/// This bypasses ALL framework overhead (fn ptrs, epoch guards, ring buffer logic)
/// to measure the pure cross-core cache line transfer latency.
fn bench_raw_atomic(timer: &PrecisionTimer) -> ScenarioResult {
    let cal = timer.calibration();
    let overhead = cal.overhead_cycles;

    // Create a topic just to get shared memory. We'll use a raw atomic in the
    // topic's SHM region to communicate between processes.
    let topic_name = format!("bench_raw_atom_{}", std::process::id());
    let consumer: Topic<CmdVel> = Topic::new(&topic_name).unwrap();
    let _ = consumer.recv(); // Register to get SHM allocated

    // Spawn the atomic writer child
    let exe = std::env::current_exe().unwrap();
    let total_writes = WARMUP + ITERATIONS;
    let mut child = Command::new(&exe)
        .args([
            "--child-atomic-writer",
            &topic_name,
            &total_writes.to_string(),
        ])
        .stdin(Stdio::null())
        .stdout(Stdio::piped())
        .stderr(Stdio::inherit())
        .spawn()
        .expect("Failed to spawn atomic writer");

    // Wait for child to start writing
    thread::sleep(Duration::from_millis(200));

    // Get pointer to header.sequence_or_head — we'll use this as our raw atomic.
    // Both processes access the same SHM-mapped header.
    let header_ptr = consumer.local_state_header_ptr();
    if header_ptr.is_null() {
        child.wait().ok();
        return ScenarioResult {
            name: "RawAtomic",
            backend: "shm".to_string(),
            expected_backend: "shm",
            measurement: "one-way",
            latencies_ns: vec![],
            total_sent: total_writes,
            total_received: 0,
            note: Some("header_ptr null"),
        };
    }
    let atom = unsafe { &(*header_ptr).sequence_or_head };

    // Phase 1: Warmup
    let mut last_val = 0u64;
    for _ in 0..WARMUP {
        loop {
            let v = atom.load(Ordering::Acquire);
            if v != last_val && v != 0 {
                last_val = v;
                break;
            }
        }
    }

    // Phase 2: Measurement — tight poll, no PAUSE, no framework code
    let mut latencies = Vec::with_capacity(ITERATIONS as usize);
    let deadline = Instant::now() + TIMEOUT;
    let mut polls = 0u64;
    while (latencies.len() as u64) < ITERATIONS {
        polls += 1;
        if polls & 16383 == 0 && Instant::now() > deadline {
            break;
        }
        let v = atom.load(Ordering::Acquire);
        if v != last_val && v != 0 {
            let end = rdtscp();
            let delta = end.wrapping_sub(v).saturating_sub(overhead);
            latencies.push(cal.cycles_to_ns(delta));
            last_val = v;
        }
    }

    child.wait().ok();

    ScenarioResult {
        name: "RawAtomic",
        backend: "shm".to_string(),
        expected_backend: "shm",
        measurement: "one-way",
        latencies_ns: latencies,
        total_sent: total_writes,
        total_received: WARMUP + ITERATIONS,
        note: None,
    }
}

/// Child process: write RDTSC timestamps to a raw atomic in SHM.
fn run_child_atomic_writer(topic_name: &str, count: u64) {
    let _ = set_cpu_affinity(CORE_AUX);
    let topic: Topic<CmdVel> = Topic::new(topic_name).unwrap();
    // Send a dummy message to register as publisher and trigger SHM creation
    topic.send(CmdVel::with_timestamp(0.0, 0.0, 0));
    thread::sleep(Duration::from_millis(100));

    let header_ptr = topic.local_state_header_ptr();
    if header_ptr.is_null() {
        eprintln!("  [raw-atomic] header_ptr null, aborting");
        return;
    }
    let atom = unsafe { &(*header_ptr).sequence_or_head };

    eprintln!(
        "  [raw-atomic] PID={} core={} writing {} timestamps",
        std::process::id(),
        CORE_AUX,
        count
    );

    for _ in 0..count {
        serialize();
        let t = rdtsc();
        atom.store(t, Ordering::Release);
        // Same pacing as other cross-process benchmarks
        for _ in 0..256 {
            spin_loop();
        }
    }
}

// ============================================================================
// Reporting: Detail
// ============================================================================

fn print_detail(r: &ScenarioResult) {
    let check = if r.backend_ok() { "ok" } else { "MISMATCH" };

    println!("  {} [{}]", r.name, r.measurement);
    println!(
        "  Backend: {} ({}, expected {})",
        r.backend_short(),
        check,
        r.expected_backend
    );

    if r.latencies_ns.is_empty() {
        println!("  NO SAMPLES -- topology did not route messages to parent consumer");
        println!(
            "  Messages: {}/{} received",
            r.total_received, r.total_sent
        );
        println!();
        return;
    }

    let s = r.stats();
    println!(
        "  Samples: {} (outliers removed: {})",
        s.count, s.outliers_removed
    );

    if r.total_sent != r.total_received {
        println!(
            "  Messages: {}/{} received ({:.1}% loss)",
            r.total_received,
            r.total_sent,
            r.loss_pct()
        );
    }

    if let Some(note) = r.note {
        println!("  Note: {}", note);
    }

    println!();
    println!(
        "    {:>8} {:>8} {:>8} {:>8} {:>8} {:>8}",
        "p50", "p95", "p99", "p99.9", "p99.99", "max"
    );
    println!(
        "    {:>8} {:>8} {:>8} {:>8} {:>8} {:>8}",
        fmt_ns(s.median as u64),
        fmt_ns(s.p95),
        fmt_ns(s.p99),
        fmt_ns(s.p999),
        fmt_ns(s.p9999),
        fmt_ns(s.max),
    );
    println!();

    let cv = if s.mean > 0.0 {
        s.std_dev / s.mean
    } else {
        0.0
    };
    println!(
        "    Mean: {}  CI95: [{:.1}, {:.1}]ns  StdDev: {:.1}ns  CV: {:.3}",
        fmt_ns(s.mean as u64),
        s.ci_low,
        s.ci_high,
        s.std_dev,
        cv,
    );
    println!(
        "    Jitter: {} (max-min)  IQR: [{}, {}]",
        fmt_ns(s.max - s.min),
        fmt_ns(s.p25),
        fmt_ns(s.p75),
    );
    println!();
}

// ============================================================================
// Reporting: Summary Table
// ============================================================================

fn print_summary(results: &[ScenarioResult]) {
    let w = BOX_W + 4;
    println!("{}", "=".repeat(w));
    println!(
        "  {:<18} {:>7}  {:>8}  {:>8}  {:>8}  {:>8}  {:>8}  {}",
        "Scenario", "Type", "p50", "p95", "p99", "p99.9", "max", "Backend"
    );
    println!("{}", "-".repeat(w));

    for r in results {
        if r.latencies_ns.is_empty() {
            println!(
                "  {:<18} {:>7}  {:>8}  {:>8}  {:>8}  {:>8}  {:>8}  {} {}",
                r.name,
                r.measurement,
                "--",
                "--",
                "--",
                "--",
                "--",
                if r.backend_ok() { "ok" } else { "!!" },
                r.backend_short(),
            );
            continue;
        }

        let s = r.stats();
        let mark = if r.backend_ok() { "ok" } else { "!!" };

        println!(
            "  {:<18} {:>7}  {:>8}  {:>8}  {:>8}  {:>8}  {:>8}  {} {}",
            r.name,
            r.measurement,
            fmt_ns(s.median as u64),
            fmt_ns(s.p95),
            fmt_ns(s.p99),
            fmt_ns(s.p999),
            fmt_ns(s.max),
            mark,
            r.backend_short(),
        );
    }

    println!("{}", "=".repeat(w));
    println!();
}

// ============================================================================
// Reporting: Methodology
// ============================================================================

fn print_methodology(cal: &RdtscCalibration) {
    println!(
        "{} Methodology {}",
        "───",
        "─".repeat(BOX_W - 18)
    );
    println!();
    println!("  Measurement types:");
    println!("    send    = producer-side send() latency (RDTSC, overhead subtracted)");
    println!("    one-way = producer-to-consumer via RDTSC timestamp in CmdVel.stamp_nanos");
    println!();
    println!("  Statistical processing:");
    println!("    - Tukey IQR outlier removal (1.5x fence)");
    println!("    - Bootstrap 95% CI (10K resamples, LCG PRNG)");
    println!("    - Full percentile distribution (p1 through p99.99)");
    println!();
    println!("  Timing infrastructure:");
    println!(
        "    RDTSC overhead: ~{}ns (serialize + rdtsc/rdtscp pair)",
        cal.cycles_to_ns(cal.overhead_cycles)
    );

    // Measure Instant::now() for comparison
    let mut times: Vec<u64> = Vec::with_capacity(10_000);
    for _ in 0..10_000 {
        let start = Instant::now();
        std::hint::black_box(());
        times.push(start.elapsed().as_nanos() as u64);
    }
    times.sort_unstable();
    println!(
        "    Instant::now(): ~{}ns (for comparison)",
        times[times.len() / 2]
    );
    println!("    Intra-process: RDTSC overhead subtracted from each sample");
    println!("    Cross-process: RDTSC overhead subtracted (rdtsc on producer + rdtscp on consumer)");
    println!();

    println!("  Known limitations:");
    println!("    - Topic selects backends based on topology and type (POD vs non-POD)");
    println!("    - MpmcShm only activates for non-POD types; POD multi-pub/sub uses PodShm");
    println!("    - Cross-process POD: zero-copy (memcpy + atomics). Non-POD: +bincode ser/deser");
    println!("    - Governor 'powersave' significantly inflates latencies vs 'performance'");
    println!();
}

// ============================================================================
// JSON Output
// ============================================================================

fn write_json_output(path: &str, platform: &PlatformInfo, results: &[ScenarioResult]) {
    let mut report = BenchmarkReport::new(platform.clone());

    for r in results {
        let s = r.stats();
        let cv = if s.mean > 0.0 {
            s.std_dev / s.mean
        } else {
            0.0
        };

        let result = BenchmarkResult {
            name: format!("all_paths_latency/{}", r.name),
            subject: format!("{} ({})", r.backend_short(), r.measurement),
            message_size: std::mem::size_of::<CmdVel>(),
            config: BenchmarkConfig {
                warmup_iterations: WARMUP as usize,
                iterations: ITERATIONS as usize,
                runs: 1,
                cpu_affinity: Some((CORE_MAIN, CORE_AUX)),
                filter_outliers: true,
                confidence_level: 95.0,
            },
            platform: platform.clone(),
            timestamp: chrono::Utc::now().to_rfc3339(),
            // Exclude raw latencies to keep JSON compact (~500K entries would be 7+ MB).
            // Statistics already capture full percentile distribution for regression tracking.
            raw_latencies_ns: Vec::new(),
            statistics: s.clone(),
            throughput: ThroughputMetrics {
                messages_per_sec: if s.mean > 0.0 {
                    1e9 / s.mean
                } else {
                    0.0
                },
                bytes_per_sec: if s.mean > 0.0 {
                    (std::mem::size_of::<CmdVel>() as f64) * 1e9 / s.mean
                } else {
                    0.0
                },
                total_messages: r.latencies_ns.len() as u64,
                total_bytes: (r.latencies_ns.len() * std::mem::size_of::<CmdVel>()) as u64,
                duration_secs: (s.mean * r.latencies_ns.len() as f64) / 1e9,
            },
            determinism: DeterminismMetrics {
                cv,
                max_jitter_ns: s.max - s.min,
                p999: s.p999,
                p9999: s.p9999,
                deadline_misses: 0,
                deadline_threshold_ns: 0,
                run_variance: 0.0,
            },
        };
        report.add_result(result);
    }

    match write_json_report(&report, path) {
        Ok(()) => println!("  JSON report written to: {}", path),
        Err(e) => eprintln!("  ERROR: Failed to write JSON report: {}", e),
    }
}

// ============================================================================
// Formatting Helpers
// ============================================================================

fn fmt_ns(ns: u64) -> String {
    if ns < 10_000 {
        format!("{}ns", ns)
    } else if ns < 1_000_000 {
        format!("{:.1}us", ns as f64 / 1_000.0)
    } else if ns < 1_000_000_000 {
        format!("{:.1}ms", ns as f64 / 1_000_000.0)
    } else {
        format!("{:.2}s", ns as f64 / 1_000_000_000.0)
    }
}

fn truncate(s: &str, max: usize) -> &str {
    if s.len() > max {
        &s[..max]
    } else {
        s
    }
}

// Box-drawing helpers (fixed-width interior = BOX_W)
fn box_top() -> String {
    format!("╔{}╗", "═".repeat(BOX_W + 2))
}
fn box_bot() -> String {
    format!("╚{}╝", "═".repeat(BOX_W + 2))
}
fn box_sep() -> String {
    format!("╠{}╣", "═".repeat(BOX_W + 2))
}
fn box_center(text: &str) -> String {
    let pad = BOX_W.saturating_sub(text.len());
    let left = pad / 2;
    let right = pad - left;
    format!(
        "║ {}{}{} ║",
        " ".repeat(left),
        text,
        " ".repeat(right)
    )
}
fn box_left(text: &str) -> String {
    let content = if text.len() > BOX_W {
        &text[..BOX_W]
    } else {
        text
    };
    let pad = BOX_W - content.len();
    format!("║ {}{} ║", content, " ".repeat(pad))
}

fn detect_git_commit_short() -> Option<String> {
    std::process::Command::new("git")
        .args(["rev-parse", "--short", "HEAD"])
        .output()
        .ok()
        .and_then(|output| {
            if output.status.success() {
                String::from_utf8(output.stdout)
                    .ok()
                    .map(|s| s.trim().to_string())
            } else {
                None
            }
        })
}
