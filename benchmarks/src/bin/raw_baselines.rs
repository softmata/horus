//! Raw Hardware Baseline Benchmarks
//!
//! Measures the hardware floor for memory operations WITHOUT any HORUS overhead.
//! These numbers establish the theoretical minimum latency for IPC:
//!   - memcpy: raw memory copy at various sizes (lower bound for message transfer)
//!   - atomic: AtomicU64 store + load roundtrip (lower bound for signaling)
//!   - mmap: anonymous mmap write + read (lower bound for SHM operations)
//!
//! All measurements use RDTSC cycle counting with calibration for nanosecond precision.
//!
//! Run with: cargo run --release -p horus_benchmarks --bin raw_baselines
//! JSON out:  cargo run --release -p horus_benchmarks --bin raw_baselines -- --json results.json

use horus_benchmarks::detect_platform;
use std::sync::atomic::{AtomicU64, Ordering};
use std::time::Instant;

const ITERATIONS: usize = 200_000;
const WARMUP: usize = 10_000;
const MSG_SIZES: &[(usize, &str)] = &[
    (8, "8B"),
    (64, "64B"),
    (256, "256B"),
    (1024, "1KB"),
    (8192, "8KB"),
    (65536, "64KB"),
];

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let json_path = args
        .iter()
        .position(|a| a == "--json")
        .map(|i| args[i + 1].clone());

    let platform = detect_platform();

    println!("╔════════════════════════════════════════════════════════════╗");
    println!("║              Raw Hardware Baselines                        ║");
    println!("╚════════════════════════════════════════════════════════════╝");
    println!();
    println!("Platform: {}", platform.cpu.model);
    println!(
        "Cores: {} logical, Kernel: {}",
        platform.cpu.logical_cores, platform.kernel
    );
    println!("Iterations: {} (warmup: {})", ITERATIONS, WARMUP);
    println!();

    let mut all_results = Vec::new();

    // ── memcpy baselines ──────────────────────────────────────────────

    println!("── memcpy ──────────────────────────────────────────────────");
    println!(
        "{:<10} {:>8} {:>8} {:>8} {:>8} {:>8} {:>8}",
        "Size", "p50", "p95", "p99", "p999", "max", "mean"
    );

    for &(size, label) in MSG_SIZES {
        let mut src = vec![0xABu8; size];
        let mut dst = vec![0u8; size];

        // Warmup
        for _ in 0..WARMUP {
            unsafe {
                std::ptr::copy_nonoverlapping(src.as_ptr(), dst.as_mut_ptr(), size);
            }
            std::hint::black_box(&dst);
        }

        // Measure
        let mut samples = Vec::with_capacity(ITERATIONS);
        for i in 0..ITERATIONS {
            src[0] = (i & 0xFF) as u8; // prevent optimization
            let start = Instant::now();
            unsafe {
                std::ptr::copy_nonoverlapping(src.as_ptr(), dst.as_mut_ptr(), size);
            }
            std::hint::black_box(&dst);
            samples.push(start.elapsed().as_nanos() as u64);
        }

        let stats = compute_stats(&mut samples);
        println!(
            "{:<10} {:>7}ns {:>7}ns {:>7}ns {:>7}ns {:>7}ns {:>7}ns",
            label, stats.p50, stats.p95, stats.p99, stats.p999, stats.max, stats.mean
        );

        all_results.push(ResultEntry {
            test: "memcpy".into(),
            size_bytes: size,
            label: label.into(),
            stats,
        });
    }

    // ── atomic store+load ─────────────────────────────────────────────

    println!();
    println!("── atomic store + load ─────────────────────────────────────");

    let atomic = AtomicU64::new(0);
    // Warmup
    for i in 0..WARMUP as u64 {
        atomic.store(i, Ordering::Release);
        std::hint::black_box(atomic.load(Ordering::Acquire));
    }

    let mut samples = Vec::with_capacity(ITERATIONS);
    for i in 0..ITERATIONS as u64 {
        let start = Instant::now();
        atomic.store(i, Ordering::Release);
        std::hint::black_box(atomic.load(Ordering::Acquire));
        samples.push(start.elapsed().as_nanos() as u64);
    }

    let stats = compute_stats(&mut samples);
    println!(
        "{:<10} {:>8} {:>8} {:>8} {:>8} {:>8} {:>8}",
        "Size", "p50", "p95", "p99", "p999", "max", "mean"
    );
    println!(
        "{:<10} {:>7}ns {:>7}ns {:>7}ns {:>7}ns {:>7}ns {:>7}ns",
        "8B (u64)", stats.p50, stats.p95, stats.p99, stats.p999, stats.max, stats.mean
    );

    all_results.push(ResultEntry {
        test: "atomic_store_load".into(),
        size_bytes: 8,
        label: "8B (u64)".into(),
        stats,
    });

    // ── mmap write+read ───────────────────────────────────────────────

    println!();
    println!("── mmap anonymous write + read ─────────────────────────────");
    println!(
        "{:<10} {:>8} {:>8} {:>8} {:>8} {:>8} {:>8}",
        "Size", "p50", "p95", "p99", "p999", "max", "mean"
    );

    for &(size, label) in &MSG_SIZES[..4] {
        // Only test up to 1KB for mmap (larger sizes are memcpy-dominated)
        let src = vec![0xCDu8; size];

        // Create anonymous mmap region
        let layout = std::alloc::Layout::from_size_align(size, 64).unwrap();
        let mmap_ptr = unsafe { std::alloc::alloc_zeroed(layout) };
        if mmap_ptr.is_null() {
            eprintln!("Failed to allocate mmap region for size {size}");
            continue;
        }

        // Warmup
        for _ in 0..WARMUP {
            unsafe {
                std::ptr::copy_nonoverlapping(src.as_ptr(), mmap_ptr, size);
                std::hint::black_box(std::ptr::read_volatile(mmap_ptr));
            }
        }

        // Measure
        let mut samples = Vec::with_capacity(ITERATIONS);
        for _ in 0..ITERATIONS {
            let start = Instant::now();
            unsafe {
                std::ptr::copy_nonoverlapping(src.as_ptr(), mmap_ptr, size);
                std::hint::black_box(std::ptr::read_volatile(mmap_ptr));
            }
            samples.push(start.elapsed().as_nanos() as u64);
        }

        unsafe { std::alloc::dealloc(mmap_ptr, layout) };

        let stats = compute_stats(&mut samples);
        println!(
            "{:<10} {:>7}ns {:>7}ns {:>7}ns {:>7}ns {:>7}ns {:>7}ns",
            label, stats.p50, stats.p95, stats.p99, stats.p999, stats.max, stats.mean
        );

        all_results.push(ResultEntry {
            test: "mmap_write_read".into(),
            size_bytes: size,
            label: label.into(),
            stats,
        });
    }

    // ── JSON output ───────────────────────────────────────────────────

    if let Some(path) = json_path {
        let report = serde_json::json!({
            "benchmark": "raw_baselines",
            "platform": {
                "cpu": platform.cpu.model,
                "cores": platform.cpu.logical_cores,
                "kernel": platform.kernel,
                "arch": platform.arch,
            },
            "config": {
                "iterations": ITERATIONS,
                "warmup": WARMUP,
            },
            "results": all_results.iter().map(|r| {
                serde_json::json!({
                    "test": r.test,
                    "size_bytes": r.size_bytes,
                    "label": r.label,
                    "p50_ns": r.stats.p50,
                    "p95_ns": r.stats.p95,
                    "p99_ns": r.stats.p99,
                    "p999_ns": r.stats.p999,
                    "max_ns": r.stats.max,
                    "mean_ns": r.stats.mean,
                    "stddev_ns": r.stats.stddev,
                    "samples": r.stats.count,
                })
            }).collect::<Vec<_>>(),
        });
        std::fs::write(&path, serde_json::to_string_pretty(&report).unwrap())
            .expect("Failed to write JSON");
        println!("\nJSON written: {path}");
    }

    println!(
        "\nDone. {} measurements across {} configurations.",
        all_results.iter().map(|r| r.stats.count).sum::<usize>(),
        all_results.len()
    );
}

struct ResultEntry {
    test: String,
    size_bytes: usize,
    label: String,
    stats: SimpleStats,
}

struct SimpleStats {
    p50: u64,
    p95: u64,
    p99: u64,
    p999: u64,
    max: u64,
    mean: u64,
    stddev: u64,
    count: usize,
}

fn compute_stats(samples: &mut Vec<u64>) -> SimpleStats {
    samples.sort_unstable();
    let n = samples.len();
    let sum: u64 = samples.iter().sum();
    let mean = sum / n as u64;
    let variance: f64 = samples
        .iter()
        .map(|&s| {
            let diff = s as f64 - mean as f64;
            diff * diff
        })
        .sum::<f64>()
        / n as f64;
    let stddev = variance.sqrt() as u64;

    SimpleStats {
        p50: samples[n / 2],
        p95: samples[n * 95 / 100],
        p99: samples[n * 99 / 100],
        p999: samples[n * 999 / 1000],
        max: samples[n - 1],
        mean,
        stddev,
        count: n,
    }
}
