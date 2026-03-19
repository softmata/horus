//! Latency Benchmark — Sustained Measurement + Message Size Sweep
//!
//! Extends the measurement methodology from `all_paths_latency` with:
//! - **Sustained mode**: 60-second continuous measurement per backend (detects tail latency)
//! - **Size sweep**: Tests all intra-process backends at 6 message sizes (8B–64KB)
//! - **CSV output**: Raw samples for external analysis (one row per measurement)
//!
//! ## Usage
//!
//! ```sh
//! # Standard run (10s per backend, all sizes)
//! cargo run --release -p horus_benchmarks --bin research_latency
//!
//! # Sustained 60-second run with CSV output
//! cargo run --release -p horus_benchmarks --bin research_latency -- --duration 60 --csv results.csv
//!
//! # Quick validation (2s per backend)
//! cargo run --release -p horus_benchmarks --bin research_latency -- --duration 2
//! ```

use horus_benchmarks::detect_platform;
use horus_core::communication::Topic;
use horus_core::core::DurationExt;
use serde::{Serialize, Deserialize};
use std::io::Write;
use std::sync::atomic::{AtomicU64, Ordering};
use std::time::{Duration, Instant};

// ============================================================================
// Message types for size sweep (must satisfy TopicMessage bounds)
// ============================================================================

macro_rules! define_msg {
    ($name:ident, $size:expr) => {
        #[repr(C)]
        #[derive(Copy, Clone, Serialize, Deserialize)]
        struct $name {
            #[serde(with = "serde_arrays")]
            data: [u8; $size],
        }
        // SAFETY: $name is repr(C) with only u8 array — trivially safe for Send+Sync
        unsafe impl Send for $name {}
        unsafe impl Sync for $name {}
        impl $name {
            fn zeroed() -> Self { Self { data: [0u8; $size] } }
        }
    };
}

define_msg!(Msg8, 8);
define_msg!(Msg64, 64);
define_msg!(Msg256, 256);
define_msg!(Msg1K, 1024);
define_msg!(Msg4K, 4096);

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
    let mut nodes = horus_core::memory::shm_base_dir();
    nodes.push("nodes");
    let _ = std::fs::remove_dir_all(nodes);
}

// ============================================================================
// Generic measurement function
// ============================================================================

/// Measure send→recv latency for a given Topic type over a duration.
/// Returns raw latency samples in nanoseconds.
fn measure_latency<T: Copy + Send + Sync + Serialize + serde::de::DeserializeOwned + 'static>(
    label: &str,
    size: usize,
    duration_secs: u64,
    make_msg: fn() -> T,
) -> Vec<u64> {
    cleanup_shm();

    let topic_name = unique(&format!("rl_{label}_{size}"));
    let msg = make_msg();

    // Create pub and sub handles
    let pub_topic: Topic<T> = Topic::new(&topic_name).expect("pub topic");
    let sub_topic: Topic<T> = Topic::new(&topic_name).expect("sub topic");

    // Warmup (1 second or 10K messages, whichever comes first)
    let warmup_deadline = Instant::now() + Duration::from_secs(1);
    let mut warmup_count = 0u64;
    while Instant::now() < warmup_deadline && warmup_count < 10_000 {
        pub_topic.send(msg);
        while sub_topic.recv().is_none() {}
        warmup_count += 1;
    }

    // Measure
    let deadline = Instant::now() + Duration::from_secs(duration_secs);
    let mut samples = Vec::with_capacity(2_000_000);

    while Instant::now() < deadline {
        let start = Instant::now();
        pub_topic.send(msg);
        while sub_topic.recv().is_none() {}
        let elapsed = start.elapsed().as_nanos() as u64;
        samples.push(elapsed);
    }

    samples
}

/// Measure with SPSC topology (1 pub, 1 sub, same process, different threads)
fn measure_spsc<T: Copy + Send + Sync + Serialize + serde::de::DeserializeOwned + 'static>(
    label: &str,
    size: usize,
    duration_secs: u64,
    make_msg: fn() -> T,
) -> Vec<u64> {
    cleanup_shm();

    let topic_name = unique(&format!("rl_spsc_{label}_{size}"));
    let msg = make_msg();

    let pub_topic: Topic<T> = Topic::new(&topic_name).expect("pub topic");

    // Consumer on separate thread (forces SpscIntra backend)
    let topic_name_clone = topic_name.clone();
    let running = std::sync::Arc::new(std::sync::atomic::AtomicBool::new(true));
    let r = running.clone();
    let consumer = std::thread::spawn(move || {
        let sub: Topic<T> = Topic::new(&topic_name_clone).expect("sub topic");
        while r.load(Ordering::Relaxed) {
            let _ = sub.recv();
            std::thread::yield_now();
        }
    });

    // Wait for consumer to register
    std::thread::sleep(100_u64.ms());

    // Measure send latency only (consumer drains in background)
    let deadline = Instant::now() + Duration::from_secs(duration_secs);
    let mut samples = Vec::with_capacity(2_000_000);

    while Instant::now() < deadline {
        let start = Instant::now();
        pub_topic.send(msg);
        let elapsed = start.elapsed().as_nanos() as u64;
        samples.push(elapsed);
    }

    running.store(false, Ordering::SeqCst);
    let _ = consumer.join();

    samples
}

/// Measure with MPSC topology (3 publishers, 1 subscriber, cross-thread)
fn measure_mpsc<T: Copy + Send + Sync + Serialize + serde::de::DeserializeOwned + 'static>(
    label: &str,
    size: usize,
    duration_secs: u64,
    make_msg: fn() -> T,
) -> Vec<u64> {
    cleanup_shm();

    let topic_name = unique(&format!("rl_mpsc_{label}_{size}"));
    let msg = make_msg();

    // Main thread is the subscriber (measures recv latency)
    let sub_topic: Topic<T> = Topic::new(&topic_name).expect("sub topic");

    // 3 publisher threads
    let running = std::sync::Arc::new(std::sync::atomic::AtomicBool::new(true));
    let mut producers = Vec::new();
    for i in 0..3 {
        let tn = topic_name.clone();
        let r = running.clone();
        producers.push(std::thread::spawn(move || {
            let pub_t: Topic<T> = Topic::new(&tn).expect("mpsc pub");
            while r.load(Ordering::Relaxed) {
                pub_t.send(msg);
                std::thread::yield_now();
            }
        }));
    }

    std::thread::sleep(200_u64.ms());

    let deadline = Instant::now() + Duration::from_secs(duration_secs);
    let mut samples = Vec::with_capacity(2_000_000);
    while Instant::now() < deadline {
        let start = Instant::now();
        while sub_topic.recv().is_none() {}
        samples.push(start.elapsed().as_nanos() as u64);
    }

    running.store(false, Ordering::SeqCst);
    for h in producers { let _ = h.join(); }
    samples
}

/// Measure with SPMC topology (1 publisher, 3 subscribers, cross-thread)
fn measure_spmc<T: Copy + Send + Sync + Serialize + serde::de::DeserializeOwned + 'static>(
    label: &str,
    size: usize,
    duration_secs: u64,
    make_msg: fn() -> T,
) -> Vec<u64> {
    cleanup_shm();

    let topic_name = unique(&format!("rl_spmc_{label}_{size}"));
    let msg = make_msg();

    // Main thread is publisher (measures send latency)
    let pub_topic: Topic<T> = Topic::new(&topic_name).expect("pub topic");

    // 3 subscriber threads draining
    let running = std::sync::Arc::new(std::sync::atomic::AtomicBool::new(true));
    let mut consumers = Vec::new();
    for _ in 0..3 {
        let tn = topic_name.clone();
        let r = running.clone();
        consumers.push(std::thread::spawn(move || {
            let sub: Topic<T> = Topic::new(&tn).expect("spmc sub");
            while r.load(Ordering::Relaxed) {
                let _ = sub.recv();
                std::thread::yield_now();
            }
        }));
    }

    std::thread::sleep(200_u64.ms());

    let deadline = Instant::now() + Duration::from_secs(duration_secs);
    let mut samples = Vec::with_capacity(2_000_000);
    while Instant::now() < deadline {
        let start = Instant::now();
        pub_topic.send(msg);
        samples.push(start.elapsed().as_nanos() as u64);
    }

    running.store(false, Ordering::SeqCst);
    for h in consumers { let _ = h.join(); }
    samples
}

/// Measure with MPMC topology (3 publishers, 3 subscribers, cross-thread)
fn measure_mpmc<T: Copy + Send + Sync + Serialize + serde::de::DeserializeOwned + 'static>(
    label: &str,
    size: usize,
    duration_secs: u64,
    make_msg: fn() -> T,
) -> Vec<u64> {
    cleanup_shm();

    let topic_name = unique(&format!("rl_mpmc_{label}_{size}"));
    let msg = make_msg();

    let running = std::sync::Arc::new(std::sync::atomic::AtomicBool::new(true));

    // 3 subscriber threads
    let mut consumers = Vec::new();
    for _ in 0..3 {
        let tn = topic_name.clone();
        let r = running.clone();
        consumers.push(std::thread::spawn(move || {
            let sub: Topic<T> = Topic::new(&tn).expect("mpmc sub");
            while r.load(Ordering::Relaxed) {
                let _ = sub.recv();
                std::thread::yield_now();
            }
        }));
    }

    // 2 background publisher threads
    let mut bg_producers = Vec::new();
    for _ in 0..2 {
        let tn = topic_name.clone();
        let r = running.clone();
        bg_producers.push(std::thread::spawn(move || {
            let pub_t: Topic<T> = Topic::new(&tn).expect("mpmc bg pub");
            while r.load(Ordering::Relaxed) {
                pub_t.send(msg);
                std::thread::yield_now();
            }
        }));
    }

    std::thread::sleep(200_u64.ms());

    // Main thread is the 3rd publisher (measures send latency under contention)
    let pub_topic: Topic<T> = Topic::new(&topic_name).expect("mpmc pub");
    let deadline = Instant::now() + Duration::from_secs(duration_secs);
    let mut samples = Vec::with_capacity(2_000_000);
    while Instant::now() < deadline {
        let start = Instant::now();
        pub_topic.send(msg);
        samples.push(start.elapsed().as_nanos() as u64);
    }

    running.store(false, Ordering::SeqCst);
    for h in consumers { let _ = h.join(); }
    for h in bg_producers { let _ = h.join(); }
    samples
}

// ============================================================================
// Statistics
// ============================================================================

struct Stats {
    count: usize,
    p50: u64,
    p95: u64,
    p99: u64,
    p999: u64,
    p9999: u64,
    max: u64,
    mean: u64,
    stddev: u64,
}

fn compute_stats(samples: &mut [u64]) -> Stats {
    samples.sort_unstable();
    let n = samples.len();
    if n == 0 {
        return Stats { count: 0, p50: 0, p95: 0, p99: 0, p999: 0, p9999: 0, max: 0, mean: 0, stddev: 0 };
    }
    let sum: u64 = samples.iter().sum();
    let mean = sum / n as u64;
    let variance: f64 = samples.iter()
        .map(|&s| { let d = s as f64 - mean as f64; d * d })
        .sum::<f64>() / n as f64;

    Stats {
        count: n,
        p50: samples[n / 2],
        p95: samples[n * 95 / 100],
        p99: samples[n * 99 / 100],
        p999: samples[n * 999 / 1000],
        p9999: samples[(n as f64 * 0.9999) as usize],
        max: samples[n - 1],
        mean,
        stddev: variance.sqrt() as u64,
    }
}

fn print_stats(label: &str, size: &str, stats: &Stats) {
    println!("{:<20} {:<6} {:>8} {:>7}ns {:>7}ns {:>7}ns {:>7}ns {:>7}ns {:>7}ns",
        label, size, format!("{}K", stats.count / 1000),
        stats.p50, stats.p95, stats.p99, stats.p999, stats.p9999, stats.max);
}

// ============================================================================
// Main
// ============================================================================

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let duration: u64 = args.iter()
        .position(|a| a == "--duration")
        .map(|i| args[i + 1].parse().unwrap_or(10))
        .unwrap_or(10);
    let csv_path = args.iter()
        .position(|a| a == "--csv")
        .map(|i| args[i + 1].clone());
    let json_path = args.iter()
        .position(|a| a == "--json")
        .map(|i| args[i + 1].clone());

    let platform = detect_platform();

    println!("╔════════════════════════════════════════════════════════════╗");
    println!("║        Research Latency — Sustained + Size Sweep           ║");
    println!("╚════════════════════════════════════════════════════════════╝");
    println!();
    println!("Platform: {}, {} cores", platform.cpu.model, platform.cpu.logical_cores);
    println!("Duration: {}s per backend×size", duration);
    println!();

    let mut csv_writer = csv_path.as_ref().map(|path| {
        let file = std::fs::File::create(path).expect("Failed to create CSV");
        let mut w = std::io::BufWriter::new(file);
        writeln!(w, "backend,topology,msg_size_bytes,latency_ns").unwrap();
        w
    });

    let mut all_results = Vec::new();

    println!("{:<20} {:<6} {:>8} {:>8} {:>8} {:>8} {:>8} {:>8} {:>8}",
        "Backend", "Size", "Samples", "p50", "p95", "p99", "p999", "p9999", "max");
    println!("{}", "─".repeat(95));

    // ── Same-thread (DirectChannel) — size sweep ──────────────────────

    macro_rules! bench_same_thread {
        ($type:ty, $size:expr, $label:expr, $ctor:expr) => {{
            let mut samples = measure_latency::<$type>("direct", $size, duration, $ctor);
            let stats = compute_stats(&mut samples);
            print_stats("DirectChannel", $label, &stats);
            if let Some(ref mut w) = csv_writer {
                for &s in &samples { writeln!(w, "DirectChannel,same_thread,{},{}", $size, s).unwrap(); }
            }
            all_results.push(($label, "DirectChannel", $size, stats));
        }};
    }

    bench_same_thread!(Msg8, 8, "8B", Msg8::zeroed);
    bench_same_thread!(Msg64, 64, "64B", Msg64::zeroed);
    bench_same_thread!(Msg256, 256, "256B", Msg256::zeroed);
    bench_same_thread!(Msg1K, 1024, "1KB", Msg1K::zeroed);
    bench_same_thread!(Msg4K, 4096, "4KB", Msg4K::zeroed);

    println!("{}", "─".repeat(95));

    // ── Cross-thread (SpscIntra) — size sweep ─────────────────────────

    macro_rules! bench_spsc {
        ($type:ty, $size:expr, $label:expr, $ctor:expr) => {{
            let mut samples = measure_spsc::<$type>("spsc", $size, duration, $ctor);
            let stats = compute_stats(&mut samples);
            print_stats("SpscIntra", $label, &stats);
            if let Some(ref mut w) = csv_writer {
                for &s in &samples { writeln!(w, "SpscIntra,cross_thread,{},{}", $size, s).unwrap(); }
            }
            all_results.push(($label, "SpscIntra", $size, stats));
        }};
    }

    bench_spsc!(Msg8, 8, "8B", Msg8::zeroed);
    bench_spsc!(Msg64, 64, "64B", Msg64::zeroed);
    bench_spsc!(Msg256, 256, "256B", Msg256::zeroed);
    bench_spsc!(Msg1K, 1024, "1KB", Msg1K::zeroed);
    bench_spsc!(Msg4K, 4096, "4KB", Msg4K::zeroed);

    println!("{}", "─".repeat(95));

    // ── MPSC (3 publishers → 1 subscriber) ──────────────────────────

    macro_rules! bench_mpsc {
        ($type:ty, $size:expr, $label:expr, $ctor:expr) => {{
            let mut samples = measure_mpsc::<$type>("mpsc", $size, duration, $ctor);
            let stats = compute_stats(&mut samples);
            print_stats("MPSC (3P:1C)", $label, &stats);
            if let Some(ref mut w) = csv_writer {
                for &s in &samples { writeln!(w, "MPSC,3pub_1sub,{},{}", $size, s).unwrap(); }
            }
            all_results.push(($label, "MPSC (3P:1C)", $size, stats));
        }};
    }

    bench_mpsc!(Msg8, 8, "8B", Msg8::zeroed);
    bench_mpsc!(Msg256, 256, "256B", Msg256::zeroed);
    bench_mpsc!(Msg1K, 1024, "1KB", Msg1K::zeroed);

    println!("{}", "─".repeat(95));

    // ── SPMC (1 publisher → 3 subscribers) ──────────────────────────

    macro_rules! bench_spmc {
        ($type:ty, $size:expr, $label:expr, $ctor:expr) => {{
            let mut samples = measure_spmc::<$type>("spmc", $size, duration, $ctor);
            let stats = compute_stats(&mut samples);
            print_stats("SPMC (1P:3C)", $label, &stats);
            if let Some(ref mut w) = csv_writer {
                for &s in &samples { writeln!(w, "SPMC,1pub_3sub,{},{}", $size, s).unwrap(); }
            }
            all_results.push(($label, "SPMC (1P:3C)", $size, stats));
        }};
    }

    bench_spmc!(Msg8, 8, "8B", Msg8::zeroed);
    bench_spmc!(Msg256, 256, "256B", Msg256::zeroed);
    bench_spmc!(Msg1K, 1024, "1KB", Msg1K::zeroed);

    println!("{}", "─".repeat(95));

    // ── MPMC (3 publishers × 3 subscribers) ─────────────────────────

    macro_rules! bench_mpmc {
        ($type:ty, $size:expr, $label:expr, $ctor:expr) => {{
            let mut samples = measure_mpmc::<$type>("mpmc", $size, duration, $ctor);
            let stats = compute_stats(&mut samples);
            print_stats("MPMC (3P:3C)", $label, &stats);
            if let Some(ref mut w) = csv_writer {
                for &s in &samples { writeln!(w, "MPMC,3pub_3sub,{},{}", $size, s).unwrap(); }
            }
            all_results.push(($label, "MPMC (3P:3C)", $size, stats));
        }};
    }

    bench_mpmc!(Msg8, 8, "8B", Msg8::zeroed);
    bench_mpmc!(Msg256, 256, "256B", Msg256::zeroed);
    bench_mpmc!(Msg1K, 1024, "1KB", Msg1K::zeroed);

    // ── Flush CSV ─────────────────────────────────────────────────────

    if let Some(ref mut w) = csv_writer {
        w.flush().unwrap();
    }

    // ── JSON output ───────────────────────────────────────────────────

    if let Some(path) = json_path {
        let report = serde_json::json!({
            "benchmark": "research_latency",
            "platform": {
                "cpu": platform.cpu.model,
                "cores": platform.cpu.logical_cores,
                "kernel": platform.kernel,
                "arch": platform.arch,
            },
            "duration_secs": duration,
            "results": all_results.iter().map(|(label, backend, size, stats)| {
                serde_json::json!({
                    "backend": backend,
                    "msg_size_label": label,
                    "msg_size_bytes": size,
                    "samples": stats.count,
                    "p50_ns": stats.p50,
                    "p95_ns": stats.p95,
                    "p99_ns": stats.p99,
                    "p999_ns": stats.p999,
                    "p9999_ns": stats.p9999,
                    "max_ns": stats.max,
                    "mean_ns": stats.mean,
                    "stddev_ns": stats.stddev,
                })
            }).collect::<Vec<_>>(),
        });
        std::fs::write(&path, serde_json::to_string_pretty(&report).unwrap()).unwrap();
        println!("\nJSON written: {path}");
    }

    if let Some(ref path) = csv_path {
        let total_samples: usize = all_results.iter().map(|(_, _, _, s)| s.count).sum();
        println!("\nCSV written: {path} ({total_samples} rows)");
    }

    println!("\nDone. {} configurations measured.", all_results.len());
}
