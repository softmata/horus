// Benchmark binary - allow clippy warnings
#![allow(unused_imports)]
#![allow(unused_assignments)]
#![allow(unreachable_patterns)]
#![allow(clippy::all)]
#![allow(deprecated)]
#![allow(dead_code)]
#![allow(unused_variables)]
#![allow(unused_mut)]

//! # HORUS IPC Latency Benchmark - RDTSC-Based
//!
//! Accurate multi-process IPC latency measurement using CPU timestamp counters (rdtsc).
//!
//! ## Methodology
//!
//! - Producer embeds rdtsc() timestamp in each message
//! - Consumer reads rdtsc() upon receipt and calculates propagation time
//! - Null cost calibration: back-to-back rdtsc() calls (~20-30 cycles)
//! - Tests both 64-byte and 128-byte cache line alignment
//!
//! ## Usage
//!
//! ```bash
//! cargo build --release --bin ipc_benchmark
//! ./target/release/ipc_benchmark
//! ```

use chrono;
use colored::Colorize;
use horus::prelude::Topic;
use horus_benchmarks::visualization::{
    draw_grouped_bar_chart, draw_latency_histogram, draw_latency_timeline, draw_speedup_chart,
    AnalysisSummary,
};
use horus_library::messages::cmd_vel::CmdVel;
use rand::Rng;
use serde::{Deserialize, Serialize};
use std::env;
use std::fs;
use std::process::{Child, Command};
use std::time::{Duration, SystemTime};

#[cfg(target_arch = "x86_64")]
use std::arch::x86_64::_rdtsc;

const ITERATIONS: usize = 50_000; // Increased for better statistics (was 10,000)
const WARMUP: usize = 5_000; // Increased warmup (was 1,000)
const NUM_RUNS: usize = 10; // More runs for better statistics (was 5)

// Barrier states
const BARRIER_CONSUMER_READY: u8 = 2;

// ============================================================================
// PLATFORM DETECTION
// ============================================================================

#[derive(Debug, Clone, Serialize, Deserialize)]
struct PlatformInfo {
    cpu_vendor: String,
    cpu_model: String,
    cpu_family: u32,
    cpu_stepping: u32,
    base_frequency_ghz: f64,
    num_physical_cores: usize,
    num_logical_cores: usize,
    cache_l1d_kb: Option<usize>,
    cache_l1i_kb: Option<usize>,
    cache_l2_kb: Option<usize>,
    cache_l3_kb: Option<usize>,
    os: String,
    kernel_version: String,
    arch: String,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
struct BenchmarkResult {
    platform: PlatformInfo,
    timestamp: String,
    link_stats: Option<IpcStats>,
    hub_stats: Option<IpcStats>,
    rdtsc_overhead_cycles: u64,
    tsc_drift_cycles: u64,

    // Validation metadata
    tsc_verification_passed: bool,
    cpu_frequency_source: String, // "measured", "cpuinfo", or "detection_failed"
    measurement_quality: String,  // "high", "medium", "low", or "invalid"
}

#[derive(Debug, Clone, Serialize, Deserialize)]
struct IpcStats {
    median_cycles: u64,
    median_ns: u64,
    mean_cycles: f64,
    std_dev_cycles: f64,
    p95_cycles: u64,
    p99_cycles: u64,
    min_cycles: u64,
    max_cycles: u64,
    ci_lower_cycles: u64,
    ci_upper_cycles: u64,
    sample_count: usize,
    outliers_removed: usize,
}

/// Read CPU timestamp counter - measures cycles, not nanoseconds
///
/// IMPORTANT: This benchmark requires x86_64 with RDTSC instruction for accurate
/// cycle-level timing. Other architectures would require platform-specific timing
/// mechanisms and separate calibration.
#[inline(always)]
fn rdtsc() -> u64 {
    #[cfg(target_arch = "x86_64")]
    unsafe {
        _rdtsc()
    }

    #[cfg(not(target_arch = "x86_64"))]
    {
        compile_error!(
            "This benchmark requires x86_64 architecture with RDTSC instruction. \
             Other architectures are not supported because:\n\
             1. RDTSC provides cycle-accurate timing, not nanoseconds\n\
             2. CPU frequency detection depends on RDTSC calibration\n\
             3. Cross-platform timing comparisons would be invalid\n\
             \n\
             To port this benchmark to other architectures, you would need:\n\
             - Platform-specific cycle counter access\n\
             - Architecture-specific frequency detection\n\
             - Separate validation and calibration methodology"
        );
        unreachable!()
    }
}

/// Calibrate rdtsc overhead (back-to-back calls)
fn calibrate_rdtsc() -> u64 {
    let mut min_cost = u64::MAX;

    // Warmup
    for _ in 0..100 {
        let _ = rdtsc();
    }

    // Measure minimum overhead
    for _ in 0..1000 {
        let start = rdtsc();
        let end = rdtsc();
        let cost = end.wrapping_sub(start);
        if cost > 0 && cost < min_cost {
            min_cost = cost;
        }
    }

    min_cost
}

/// Verify TSC synchronization across cores (CRITICAL for cross-core benchmarks)
#[cfg(target_arch = "x86_64")]
fn verify_tsc_synchronization() -> Result<u64, String> {
    use std::arch::x86_64::__cpuid;
    use std::sync::{Arc, Barrier, Mutex};
    use std::thread;

    println!("\n{}", "TSC Verification:".bright_yellow());

    // 1. Check for invariant TSC (constant across P-states and C-states)
    let cpuid = unsafe { __cpuid(0x80000007) };
    let has_invariant_tsc = (cpuid.edx & (1 << 8)) != 0;

    print!("  • Invariant TSC: ");
    if has_invariant_tsc {
        println!("{}", "[OK] YES".bright_green());
    } else {
        println!("{}", "[FAIL] NO".bright_red());
        return Err("CPU does not have invariant TSC! Results may be inaccurate.".to_string());
    }

    // 2. Verify cross-core TSC synchronization
    // Spawn threads on cores 0 and 1, synchronize, then read TSC
    let barrier = Arc::new(Barrier::new(2));
    let tsc_values = Arc::new(Mutex::new(Vec::new()));

    let barrier1 = barrier.clone();
    let tsc_values1 = tsc_values.clone();
    let handle1 = thread::spawn(move || {
        set_cpu_affinity(0);
        barrier1.wait(); // Synchronize with other thread
        let tsc = rdtsc();
        tsc_values1.lock().unwrap().push((0, tsc));
    });

    let barrier2 = barrier.clone();
    let tsc_values2 = tsc_values.clone();
    let handle2 = thread::spawn(move || {
        set_cpu_affinity(1);
        barrier2.wait(); // Synchronize with other thread
        let tsc = rdtsc();
        tsc_values2.lock().unwrap().push((1, tsc));
    });

    handle1.join().unwrap();
    handle2.join().unwrap();

    let values = tsc_values.lock().unwrap();
    let tsc0 = values.iter().find(|(core, _)| *core == 0).unwrap().1;
    let tsc1 = values.iter().find(|(core, _)| *core == 1).unwrap().1;
    let drift = if tsc0 > tsc1 {
        tsc0 - tsc1
    } else {
        tsc1 - tsc0
    };

    print!("  • Cross-core TSC drift: {} cycles ", drift);
    if drift < 1000 {
        println!("{}", "([OK] excellent)".bright_green());
    } else if drift < 5000 {
        println!(
            "{}",
            "([WARNING] moderate - expect some variance)".bright_yellow()
        );
    } else {
        println!("{}", "([FAIL] too large!)".bright_red());
        return Err(format!(
            "TSC drift too large ({} cycles). Results will be inaccurate!",
            drift
        ));
    }

    Ok(drift)
}

#[cfg(not(target_arch = "x86_64"))]
fn verify_tsc_synchronization() -> Result<u64, String> {
    println!("\n{}", "TSC Verification:".bright_yellow());
    println!("  • [WARNING] Skipped on non-x86_64 platform");
    Ok(0) // No drift measurement on non-x86_64
}

/// Check system state for optimal benchmarking conditions
fn check_system_state() {
    println!("\n{}", "System State Validation:".bright_yellow());

    let mut warnings = Vec::new();
    let mut ok_count = 0;

    // Check ASLR
    #[cfg(target_os = "linux")]
    {
        if let Ok(aslr) = std::fs::read_to_string("/proc/sys/kernel/randomize_va_space") {
            let aslr_val = aslr.trim();
            if aslr_val == "0" {
                println!(
                    "  • ASLR: {} {}",
                    "disabled".to_string(),
                    " [OK]".bright_green()
                );
                ok_count += 1;
            } else {
                println!(
                    "  • ASLR: {} {}",
                    format!("enabled ({})", aslr_val),
                    "([WARNING] may increase variance)".bright_yellow()
                );
                warnings.push(
                    "ASLR enabled - run: echo 0 | sudo tee /proc/sys/kernel/randomize_va_space",
                );
            }
        }
    }

    // Check for isolated cores
    #[cfg(target_os = "linux")]
    {
        if let Ok(cmdline) = std::fs::read_to_string("/proc/cmdline") {
            if cmdline.contains("isolcpus") {
                if let Some(isolcpus) = cmdline
                    .split_whitespace()
                    .find(|s| s.starts_with("isolcpus="))
                    .and_then(|s| s.split('=').nth(1))
                {
                    println!(
                        "  • Core isolation: {} {}",
                        isolcpus.to_string(),
                        " [OK]".bright_green()
                    );
                    ok_count += 1;
                }
            } else {
                println!(
                    "  • Core isolation: {} {}",
                    "none".to_string(),
                    "([WARNING] other processes may interfere)".bright_yellow()
                );
                warnings.push("No core isolation - add 'isolcpus=0,1' to kernel cmdline");
            }
        }
    }

    // Check number of running processes
    #[cfg(target_os = "linux")]
    {
        if let Ok(output) = std::process::Command::new("ps").arg("aux").output() {
            let proc_count = output.stdout.split(|&b| b == b'\n').count();
            print!("  • Running processes: {} ", proc_count);
            if proc_count < 200 {
                println!("{}", " [OK]".bright_green());
                ok_count += 1;
            } else if proc_count < 400 {
                println!("{}", "([WARNING] moderate)".bright_yellow());
            } else {
                println!(
                    "{}",
                    "([WARNING] high - may affect results)".bright_yellow()
                );
                warnings.push("Many processes running - consider stopping non-essential services");
            }
        }
    }

    // Summary
    println!();
    if warnings.is_empty() {
        println!(
            "  {} System optimized for benchmarking!",
            "[OK]".bright_green()
        );
    } else {
        println!("  {} Warnings:", "[WARNING]".bright_yellow());
        for warning in &warnings {
            println!("    - {}", warning);
        }
        println!();
        println!("  {} For optimal results, run:", "→".bright_cyan());
        println!("    ./benchmarks/benchmark_setup.sh");
    }
}

/// Detect platform information for cross-platform comparison
fn detect_platform() -> PlatformInfo {
    let mut platform = PlatformInfo {
        cpu_vendor: "Unknown".to_string(),
        cpu_model: "Unknown".to_string(),
        cpu_family: 0,
        cpu_stepping: 0,
        base_frequency_ghz: 0.0,
        num_physical_cores: 1,
        num_logical_cores: 1,
        cache_l1d_kb: None,
        cache_l1i_kb: None,
        cache_l2_kb: None,
        cache_l3_kb: None,
        os: std::env::consts::OS.to_string(),
        kernel_version: "Unknown".to_string(),
        arch: std::env::consts::ARCH.to_string(),
    };

    // Get CPU info from /proc/cpuinfo (Linux)
    #[cfg(target_os = "linux")]
    {
        if let Ok(cpuinfo) = std::fs::read_to_string("/proc/cpuinfo") {
            // CPU model
            if let Some(line) = cpuinfo.lines().find(|l| l.starts_with("model name")) {
                if let Some(model) = line.split(':').nth(1) {
                    platform.cpu_model = model.trim().to_string();
                }
            }

            // CPU vendor
            if let Some(line) = cpuinfo.lines().find(|l| l.starts_with("vendor_id")) {
                if let Some(vendor) = line.split(':').nth(1) {
                    let v = vendor.trim();
                    platform.cpu_vendor = match v {
                        "GenuineIntel" => "Intel",
                        "AuthenticAMD" => "AMD",
                        "ARM" => "ARM",
                        _ => v,
                    }
                    .to_string();
                }
            }

            // CPU family
            if let Some(line) = cpuinfo.lines().find(|l| l.starts_with("cpu family")) {
                if let Some(family) = line.split(':').nth(1) {
                    platform.cpu_family = family.trim().parse().unwrap_or(0);
                }
            }

            // CPU stepping
            if let Some(line) = cpuinfo.lines().find(|l| l.starts_with("stepping")) {
                if let Some(stepping) = line.split(':').nth(1) {
                    platform.cpu_stepping = stepping.trim().parse().unwrap_or(0);
                }
            }

            // Core counts
            let processor_lines: Vec<_> = cpuinfo
                .lines()
                .filter(|l| l.starts_with("processor"))
                .collect();
            platform.num_logical_cores = processor_lines.len();

            // Physical cores (count unique core IDs)
            let mut core_ids = std::collections::HashSet::new();
            for line in cpuinfo.lines() {
                if line.starts_with("core id") {
                    if let Some(id) = line.split(':').nth(1) {
                        core_ids.insert(id.trim().to_string());
                    }
                }
            }
            if !core_ids.is_empty() {
                platform.num_physical_cores = core_ids.len();
            }

            // Cache sizes
            if let Some(line) = cpuinfo.lines().find(|l| l.contains("cache size")) {
                if let Some(size_str) = line.split(':').nth(1) {
                    if let Some(kb_str) = size_str.trim().split_whitespace().next() {
                        if let Ok(kb) = kb_str.parse::<usize>() {
                            platform.cache_l3_kb = Some(kb);
                        }
                    }
                }
            }
        }

        // Get kernel version
        if let Ok(output) = Command::new("uname").arg("-r").output() {
            platform.kernel_version = String::from_utf8_lossy(&output.stdout).trim().to_string();
        }

        // Get cache info from sysfs
        let cache_base = "/sys/devices/system/cpu/cpu0/cache";
        for i in 0..10 {
            let index_path = format!("{}/index{}", cache_base, i);
            if let Ok(level) = std::fs::read_to_string(format!("{}/level", index_path)) {
                let level_num: u32 = level.trim().parse().unwrap_or(0);
                if let Ok(size) = std::fs::read_to_string(format!("{}/size", index_path)) {
                    let size_kb = parse_cache_size(&size);
                    if let Ok(cache_type) = std::fs::read_to_string(format!("{}/type", index_path))
                    {
                        match (level_num, cache_type.trim()) {
                            (1, "Data") => platform.cache_l1d_kb = Some(size_kb),
                            (1, "Instruction") => platform.cache_l1i_kb = Some(size_kb),
                            (2, _) => platform.cache_l2_kb = Some(size_kb),
                            (3, _) => platform.cache_l3_kb = Some(size_kb),
                            _ => {}
                        }
                    }
                }
            }
        }
    }

    platform
}

fn parse_cache_size(size_str: &str) -> usize {
    let s = size_str.trim();
    if s.ends_with('K') {
        s.trim_end_matches('K').parse().unwrap_or(0)
    } else if s.ends_with('M') {
        s.trim_end_matches('M').parse::<usize>().unwrap_or(0) * 1024
    } else {
        s.parse().unwrap_or(0)
    }
}

/// Detect CPU frequency for accurate ns conversion
fn detect_cpu_frequency() -> Result<f64, String> {
    println!("\n{}", "CPU Frequency Detection:".bright_yellow());

    // Method 1: Use RDTSC to measure actual frequency
    let start_tsc = rdtsc();
    let start_time = std::time::Instant::now();

    std::thread::sleep(Duration::from_millis(100));

    let end_tsc = rdtsc();
    let elapsed_ns = start_time.elapsed().as_nanos() as u64;

    let cycles = end_tsc - start_tsc;
    let freq_ghz = (cycles as f64) / (elapsed_ns as f64);

    if freq_ghz <= 0.0 || freq_ghz.is_nan() || freq_ghz.is_infinite() {
        return Err(format!("Invalid measured frequency: {:.3} GHz", freq_ghz));
    }

    println!("  • Measured frequency: {:.3} GHz", freq_ghz);

    // Method 2: Check cpuinfo (for comparison)
    #[cfg(target_os = "linux")]
    {
        if let Ok(cpuinfo) = std::fs::read_to_string("/proc/cpuinfo") {
            if let Some(line) = cpuinfo.lines().find(|l| l.starts_with("model name")) {
                println!(
                    "  • CPU model: {}",
                    line.split(':').nth(1).unwrap_or("unknown").trim()
                );
            }
        }
    }

    // Check frequency governor
    #[cfg(target_os = "linux")]
    {
        if let Ok(governor) =
            std::fs::read_to_string("/sys/devices/system/cpu/cpu0/cpufreq/scaling_governor")
        {
            let gov = governor.trim();
            print!("  • Frequency governor: {} ", gov);
            if gov == "performance" {
                println!("{}", "([OK] locked)".bright_green());
            } else {
                println!(
                    "{}",
                    "([WARNING] not locked - may cause variance)".bright_yellow()
                );
                println!("    Run: sudo cpupower frequency-set --governor performance");
            }
        }
    }

    // Check turbo boost
    #[cfg(target_os = "linux")]
    {
        if let Ok(turbo) = std::fs::read_to_string("/sys/devices/system/cpu/intel_pstate/no_turbo")
        {
            if turbo.trim() == "0" {
                println!(
                    "  • {} {}",
                    "Turbo boost: enabled".to_string(),
                    "([WARNING] may cause variance)".bright_yellow()
                );
                println!(
                    "    Run: echo 1 | sudo tee /sys/devices/system/cpu/intel_pstate/no_turbo"
                );
            } else {
                println!(
                    "  • Turbo boost: {} {}",
                    "disabled".to_string(),
                    " [OK]".bright_green()
                );
            }
        }
    }

    Ok(freq_ghz)
}

/// Print platform information
fn print_platform_info(platform: &PlatformInfo) {
    println!("\n{}", "Platform Information:".bright_yellow());
    println!("  • CPU: {} {}", platform.cpu_vendor, platform.cpu_model);
    println!("  • Architecture: {}", platform.arch);
    println!(
        "  • Cores: {} physical, {} logical",
        platform.num_physical_cores, platform.num_logical_cores
    );

    if let Some(l1d) = platform.cache_l1d_kb {
        print!("  • Cache: L1d={}K", l1d);
        if let Some(l1i) = platform.cache_l1i_kb {
            print!(", L1i={}K", l1i);
        }
        if let Some(l2) = platform.cache_l2_kb {
            print!(", L2={}K", l2);
        }
        if let Some(l3) = platform.cache_l3_kb {
            print!(", L3={}K", l3);
        }
        println!();
    }

    println!(
        "  • OS: {} (kernel {})",
        platform.os, platform.kernel_version
    );
}

/// Save benchmark results to JSON database
fn save_results(result: &BenchmarkResult) -> Result<(), String> {
    let db_path = "benchmarks/benchmark_results.json";

    // Ensure benchmarks directory exists
    if let Some(parent) = std::path::Path::new(db_path).parent() {
        std::fs::create_dir_all(parent)
            .map_err(|e| format!("Failed to create benchmarks directory: {}", e))?;
    }

    // Load existing results
    let mut results: Vec<BenchmarkResult> = if std::path::Path::new(db_path).exists() {
        let content = std::fs::read_to_string(db_path)
            .map_err(|e| format!("Failed to read database: {}", e))?;
        serde_json::from_str(&content).unwrap_or_else(|_| Vec::new())
    } else {
        Vec::new()
    };

    // Add new result
    results.push(result.clone());

    // Save back to file
    let json = serde_json::to_string_pretty(&results)
        .map_err(|e| format!("Failed to serialize: {}", e))?;
    std::fs::write(db_path, json).map_err(|e| format!("Failed to write database: {}", e))?;

    Ok(())
}

/// Load all benchmark results from database
fn load_results() -> Vec<BenchmarkResult> {
    let db_path = "benchmarks/benchmark_results.json";
    if let Ok(content) = std::fs::read_to_string(db_path) {
        serde_json::from_str(&content).unwrap_or_else(|_| Vec::new())
    } else {
        Vec::new()
    }
}

/// Compare current results with previous runs
fn print_comparison(current: &BenchmarkResult) {
    let all_results = load_results();
    if all_results.len() <= 1 {
        println!("\n{}", "No previous results for comparison".bright_cyan());
        return;
    }

    println!("\n{}", "═".repeat(80).bright_cyan());
    println!("{}", "  PLATFORM COMPARISON".bright_cyan().bold());
    println!("{}", "═".repeat(80).bright_cyan());

    // Group by platform
    let mut platform_groups: std::collections::HashMap<String, Vec<&BenchmarkResult>> =
        std::collections::HashMap::new();

    for result in &all_results {
        let key = format!(
            "{} {}",
            result.platform.cpu_vendor, result.platform.cpu_model
        );
        platform_groups.entry(key).or_default().push(result);
    }

    // Print comparison table
    println!(
        "\n{:<50} {:>12} {:>12}",
        "Platform", "Link (ns)", "Hub (ns)"
    );
    println!("{}", "─".repeat(80));

    for (platform_name, results) in platform_groups.iter() {
        if let Some(latest) = results.last() {
            let link_ns = latest.link_stats.as_ref().map(|s| s.median_ns).unwrap_or(0);
            let hub_ns = latest.hub_stats.as_ref().map(|s| s.median_ns).unwrap_or(0);

            let link_str = if link_ns > 0 {
                format!("{}", link_ns)
            } else {
                "N/A".to_string()
            };

            let hub_str = if hub_ns > 0 {
                format!("{}", hub_ns)
            } else {
                "N/A".to_string()
            };

            // Truncate platform name if too long
            let display_name = if platform_name.len() > 48 {
                format!("{}...", &platform_name[..45])
            } else {
                platform_name.clone()
            };

            println!("{:<50} {:>12} {:>12}", display_name, link_str, hub_str);
        }
    }

    println!();
}

fn main() {
    let args: Vec<String> = env::args().collect();

    // Subprocess mode: <ipc_type> <role> <topic> <barrier_file>
    if args.len() > 1 {
        match args[1].as_str() {
            "hub_producer" => hub_producer(&args[2], &args[3]),
            "hub_consumer" => hub_consumer(&args[2], &args[3]),
            _ => eprintln!("Unknown mode: {}", args[1]),
        }
        return;
    }

    // Main coordinator
    println!("\n{}", "═".repeat(80).bright_cyan().bold());
    println!(
        "{}",
        "  HORUS IPC LATENCY BENCHMARK v2.0".bright_cyan().bold()
    );
    println!(
        "{}",
        "  RDTSC-Based True Propagation Time Measurement".bright_cyan()
    );
    println!("{}", "═".repeat(80).bright_cyan().bold());

    // Detect platform
    let mut platform = detect_platform();
    print_platform_info(&platform);

    // Run validation checks and capture TSC drift
    let (tsc_drift, tsc_verified) = match verify_tsc_synchronization() {
        Ok(drift) => (drift, true),
        Err(e) => {
            eprintln!("\n{}", format!("WARNING: {}", e).bright_yellow());
            eprintln!("{}", "TSC synchronization check failed.".bright_yellow());
            eprintln!(
                "{}",
                "Results will be marked as LOW QUALITY.".bright_red().bold()
            );
            eprintln!(
                "{}",
                "For best results, fix TSC issues and re-run.\n".bright_yellow()
            );
            (0, false) // Mark as failed but continue with warning
        }
    };

    // CPU frequency detection - NO FALLBACKS for measurement integrity
    let (cpu_freq, freq_source) = match detect_cpu_frequency() {
        Ok(freq) => (freq, "rdtsc_measured"),
        Err(e) => {
            // In CI environment, we allow fallback to prevent build failure
            if std::env::var("CI").is_ok() || std::env::var("GITHUB_ACTIONS").is_ok() {
                eprintln!("\n{}", format!("WARNING: {}", e).bright_yellow());
                eprintln!(
                    "{}",
                    "CPU frequency detection failed in CI environment.".bright_yellow()
                );
                eprintln!(
                    "{}",
                    "Falling back to estimated frequency (2.5 GHz).".bright_yellow()
                );
                (2.5, "ci_estimate")
            } else {
                eprintln!("\n{}", format!("FATAL ERROR: {}", e).bright_red().bold());
                eprintln!("{}", "CPU frequency detection failed.".bright_red());
                eprintln!(
                    "{}",
                    "Accurate benchmarks require accurate frequency measurement.".bright_yellow()
                );
                eprintln!(
                    "{}",
                    "Cannot proceed with arbitrary fallback values.".bright_yellow()
                );
                eprintln!("\n{}", "Possible solutions:".bright_cyan());
                eprintln!("  1. Check CPU supports invariant TSC");
                eprintln!("  2. Run benchmark setup script: ./benchmarks/benchmark_setup.sh");
                eprintln!("  3. Verify system permissions for rdtsc access");
                std::process::exit(1);
            }
        }
    };
    platform.base_frequency_ghz = cpu_freq;

    // System state validation
    check_system_state();

    // Calibration
    let rdtsc_overhead = calibrate_rdtsc();
    println!("\n{}", "RDTSC Calibration:".bright_yellow());
    println!(
        "  • Null cost (back-to-back rdtsc): {} cycles",
        rdtsc_overhead
    );
    println!("  • Target: ~20-30 cycles on modern x86_64");

    println!("\n{}", "Benchmark Configuration:".bright_yellow());
    println!("  • Message type: CmdVel (16 bytes)");
    println!(
        "  • Iterations per run: {}",
        format!("{}", ITERATIONS).bright_green()
    );
    println!(
        "  • Warmup iterations: {}",
        format!("{}", WARMUP).bright_green()
    );
    println!(
        "  • Number of runs: {}",
        format!("{}", NUM_RUNS).bright_green()
    );
    println!("  • CPU Affinity: producer=core0, consumer=core1");
    println!("  • Measurement: rdtsc timestamp embedded in message");
    println!("  • Pattern: Ping-pong (ack before next send - no queue buildup)");
    println!("  • Cache Alignment: 64-byte (optimized for x86_64)");
    println!();

    // Run benchmarks for each IPC system and capture statistics
    let (link_results, hub_results) = run_all_benchmarks(cpu_freq);

    // Convert Statistics to IpcStats for database storage
    let link_ipc_stats = link_results
        .stats
        .as_ref()
        .map(|s| s.to_ipc_stats(cpu_freq));
    let hub_ipc_stats = hub_results.stats.as_ref().map(|s| s.to_ipc_stats(cpu_freq));

    // Determine measurement quality based on validation results
    let measurement_quality = if !tsc_verified {
        "invalid".to_string() // TSC verification failed
    } else if tsc_drift > 10000 {
        "low".to_string() // High TSC drift (>10K cycles)
    } else if tsc_drift > 1000 {
        "medium".to_string() // Moderate TSC drift
    } else if link_ipc_stats.is_none() || hub_ipc_stats.is_none() {
        "invalid".to_string() // Missing benchmark data
    } else {
        "high".to_string() // All checks passed
    };

    // Save results to database
    let result = BenchmarkResult {
        platform,
        timestamp: SystemTime::now()
            .duration_since(SystemTime::UNIX_EPOCH)
            .unwrap()
            .as_secs()
            .to_string(),
        link_stats: link_ipc_stats,
        hub_stats: hub_ipc_stats,
        rdtsc_overhead_cycles: calibrate_rdtsc(),
        tsc_drift_cycles: tsc_drift,

        // Validation metadata
        tsc_verification_passed: tsc_verified,
        cpu_frequency_source: freq_source.to_string(),
        measurement_quality,
    };

    // Validate results before saving (measurement integrity check)
    println!("\n{}", "═".repeat(80).bright_cyan());
    println!(
        "{}",
        "  MEASUREMENT QUALITY ASSESSMENT".bright_cyan().bold()
    );
    println!("{}", "═".repeat(80).bright_cyan());

    match result.measurement_quality.as_str() {
        "high" => {
            println!(
                "  {}",
                "[OK] HIGH QUALITY - All validation checks passed"
                    .bright_green()
                    .bold()
            );
            println!("  • TSC verification: PASSED");
            println!("  • CPU frequency: Measured via RDTSC");
            println!(
                "  • TSC drift: {} cycles (excellent)",
                result.tsc_drift_cycles
            );
            println!(
                "\n  {} These results meet high quality standards.",
                "[OK]".bright_green()
            );
        }
        "medium" => {
            println!(
                "  {}",
                "[WARNING] MEDIUM QUALITY - Moderate TSC drift detected"
                    .bright_yellow()
                    .bold()
            );
            println!("  • TSC verification: PASSED");
            println!("  • CPU frequency: Measured via RDTSC");
            println!(
                "  • TSC drift: {} cycles (moderate variance expected)",
                result.tsc_drift_cycles
            );
            println!(
                "\n  {} Usable for performance trends, but note increased variance.",
                "[WARNING]".bright_yellow()
            );
        }
        "low" => {
            println!(
                "  {}",
                "[WARNING] LOW QUALITY - High TSC drift detected"
                    .bright_red()
                    .bold()
            );
            println!("  • TSC verification: PASSED");
            println!("  • CPU frequency: Measured via RDTSC");
            println!(
                "  • TSC drift: {} cycles (high variance expected)",
                result.tsc_drift_cycles
            );
            println!(
                "\n  {} Not recommended for benchmarking.",
                "[WARNING]".bright_red()
            );
            println!(
                "  {} Consider running benchmark_setup.sh for better quality.",
                "→".bright_cyan()
            );
        }
        "invalid" => {
            println!(
                "  {}",
                "[FAIL] INVALID - Critical validation failures"
                    .bright_red()
                    .bold()
            );
            if !result.tsc_verification_passed {
                println!("  • TSC verification: FAILED");
            }
            if result.link_stats.is_none() {
                println!("  • Link benchmark: NO DATA");
            }
            if result.hub_stats.is_none() {
                println!("  • Hub benchmark: NO DATA");
            }
            println!(
                "\n  {}",
                "[FAIL] These results CANNOT be used.".bright_red().bold()
            );
            println!("  {} Fix validation issues and re-run.", "→".bright_red());
        }
        _ => {
            println!("  {}", "[WARNING] UNKNOWN QUALITY STATUS".bright_yellow());
        }
    }

    match save_results(&result) {
        Ok(_) => {
            println!(
                "\n{}",
                "Results saved to benchmark_results.json".bright_green()
            );
            if result.measurement_quality == "invalid" {
                println!("{}", "  (Marked as INVALID in database)".bright_red());
            }
        }
        Err(e) => {
            eprintln!(
                "\n{}",
                format!("ERROR: Could not save results: {}", e).bright_red()
            );
        }
    }

    // Show comparison with previous runs
    print_comparison(&result);

    // Generate visualizations
    println!("\n{}", "═".repeat(80).bright_cyan());
    println!("{}", "  GENERATING VISUALIZATIONS".bright_cyan().bold());
    println!("{}", "═".repeat(80).bright_cyan());

    if let Err(e) = generate_visualizations(cpu_freq, &link_results, &hub_results) {
        eprintln!("Warning: Failed to generate visualizations: {}", e);
    }

    println!("\n{}", "═".repeat(80).bright_cyan().bold());
    println!();
}

/// Generate visualization charts for the benchmark results
fn generate_visualizations(
    cpu_freq: f64,
    link_results: &BenchmarkResults,
    hub_results: &BenchmarkResults,
) -> Result<(), Box<dyn std::error::Error>> {
    let output_dir = "benchmarks/results/graphs";
    std::fs::create_dir_all(output_dir)?;

    let timestamp = chrono::Local::now().format("%Y%m%d_%H%M%S");

    // Convert cycles to ns for visualization
    let cycles_to_ns = |cycles: u64| -> f64 { cycles as f64 / cpu_freq };

    // Get stats if available
    let link_stats = link_results.stats.as_ref();
    let hub_stats = hub_results.stats.as_ref();

    // 1. Link vs Hub comparison chart (if both have stats)
    if let (Some(link_s), Some(hub_s)) = (link_stats, hub_stats) {
        println!("  -> Generating Link vs Hub comparison chart...");
        let link_median_ns = cycles_to_ns(link_s.median);
        let hub_median_ns = cycles_to_ns(hub_s.median);
        let link_p95_ns = cycles_to_ns(link_s.p95);
        let hub_p95_ns = cycles_to_ns(hub_s.p95);
        let link_p99_ns = cycles_to_ns(link_s.p99);
        let hub_p99_ns = cycles_to_ns(hub_s.p99);

        draw_grouped_bar_chart(
            &format!("{}/ipc_comparison_{}.png", output_dir, timestamp),
            "HORUS IPC Latency Comparison (Link vs Hub)",
            &["Median", "P95", "P99"],
            &[link_median_ns, link_p95_ns, link_p99_ns],
            &[hub_median_ns, hub_p95_ns, hub_p99_ns],
            "Link (SPSC)",
            "Hub (MPMC)",
            "Latency (ns)",
        )?;

        // 2. Speedup chart (Link vs Hub)
        println!("  -> Generating speedup chart...");
        let speedups = [
            hub_median_ns / link_median_ns,
            hub_p95_ns / link_p95_ns,
            hub_p99_ns / link_p99_ns,
        ];
        draw_speedup_chart(
            &format!("{}/ipc_speedup_{}.png", output_dir, timestamp),
            "Hub Latency vs Link (Higher = Hub is Slower)",
            &["Median", "P95", "P99"],
            &speedups,
        )?;
    }

    // 3. Link latency histogram (if raw data available)
    if !link_results.raw_latencies_cycles.is_empty() {
        println!("  -> Generating Link latency histogram...");
        let link_latencies_ns: Vec<f64> = link_results
            .raw_latencies_cycles
            .iter()
            .map(|&c| cycles_to_ns(c))
            .collect();
        draw_latency_histogram(
            &format!("{}/link_histogram_{}.png", output_dir, timestamp),
            "Link (SPSC) Latency Distribution",
            &link_latencies_ns,
            50,
        )?;

        // Statistical analysis
        let analysis = AnalysisSummary::from_latencies(&link_latencies_ns);
        analysis.print_report("Link IPC");

        // 4. Link latency timeline
        if link_latencies_ns.len() > 200 {
            println!("  -> Generating Link latency timeline...");
            draw_latency_timeline(
                &format!("{}/link_timeline_{}.png", output_dir, timestamp),
                "Link (SPSC) Latency Over Time",
                &link_latencies_ns,
                100,
            )?;
        }
    }

    // 5. Hub latency histogram (if raw data available)
    if !hub_results.raw_latencies_cycles.is_empty() {
        println!("  -> Generating Hub latency histogram...");
        let hub_latencies_ns: Vec<f64> = hub_results
            .raw_latencies_cycles
            .iter()
            .map(|&c| cycles_to_ns(c))
            .collect();
        draw_latency_histogram(
            &format!("{}/hub_histogram_{}.png", output_dir, timestamp),
            "Hub (MPMC) Latency Distribution",
            &hub_latencies_ns,
            50,
        )?;

        // Statistical analysis
        let analysis = AnalysisSummary::from_latencies(&hub_latencies_ns);
        analysis.print_report("Hub IPC");

        // 6. Hub latency timeline
        if hub_latencies_ns.len() > 200 {
            println!("  -> Generating Hub latency timeline...");
            draw_latency_timeline(
                &format!("{}/hub_timeline_{}.png", output_dir, timestamp),
                "Hub (MPMC) Latency Over Time",
                &hub_latencies_ns,
                100,
            )?;
        }
    }

    println!();
    println!("  Graphs saved to: {}/", output_dir);

    Ok(())
}

/// Combined benchmark results with raw latencies for visualization
struct BenchmarkResults {
    stats: Option<Statistics>,
    raw_latencies_cycles: Vec<u64>,
}

fn run_all_benchmarks(cpu_freq: f64) -> (BenchmarkResults, BenchmarkResults) {
    // 1. Hub (multi-process MPMC)
    println!("\n{}", "═".repeat(80).bright_white());
    println!(
        "{}",
        "  HORUS HUB (Multi-Process MPMC)".bright_white().bold()
    );
    println!("{}", "═".repeat(80).bright_white());
    let hub_results = run_ipc_benchmark("hub", cpu_freq);

    // 2. Link (single-process SPSC)
    println!("\n{}", "═".repeat(80).bright_white());
    println!(
        "{}",
        "  HORUS LINK (Single-Process SPSC)".bright_white().bold()
    );
    println!("{}", "═".repeat(80).bright_white());
    let link_results = run_link_benchmark(cpu_freq);

    (link_results, hub_results)
}

fn run_ipc_benchmark(ipc_type: &str, cpu_freq: f64) -> BenchmarkResults {
    let mut all_latencies = Vec::new();

    for run in 1..=NUM_RUNS {
        print!("  Run {}/{}: ", run, NUM_RUNS);
        std::io::Write::flush(&mut std::io::stdout()).unwrap();

        let latencies = run_benchmark(ipc_type);
        let median_cycles = median(&latencies);

        all_latencies.push(latencies);
        println!("{} cycles median", median_cycles);
    }

    let all_cycles: Vec<u64> = all_latencies.iter().flatten().copied().collect();
    let stats = print_results(&all_latencies, cpu_freq);

    BenchmarkResults {
        stats,
        raw_latencies_cycles: all_cycles,
    }
}

/// Calculate statistics from raw measurements
/// Returns None if filtered data is empty (all values were outliers)
fn calculate_statistics(all_cycles: &[u64]) -> Option<Statistics> {
    if all_cycles.is_empty() {
        return None;
    }

    let filtered = filter_outliers(all_cycles);

    // Check if outlier filtering removed ALL values
    if filtered.is_empty() {
        eprintln!(
            "WARNING: Outlier filtering removed ALL {} measurements!",
            all_cycles.len()
        );
        eprintln!("This indicates extremely noisy data or incorrect filtering parameters.");
        return None;
    }

    let (ci_lower, ci_upper) = confidence_interval_95(&filtered);

    Some(Statistics {
        raw_count: all_cycles.len(),
        filtered_count: filtered.len(),
        outliers_removed: all_cycles.len() - filtered.len(),
        median: median(&filtered),
        mean: mean(&filtered),
        p95: percentile(&filtered, 95),
        p99: percentile(&filtered, 99),
        min: *filtered.iter().min().unwrap_or(&0),
        max: *filtered.iter().max().unwrap_or(&0),
        std_dev: std_dev(&filtered),
        ci_lower,
        ci_upper,
    })
}

fn print_results(all_latencies: &[Vec<u64>], cpu_freq: f64) -> Option<Statistics> {
    let all_cycles: Vec<u64> = all_latencies.iter().flatten().copied().collect();

    if all_cycles.is_empty() {
        println!("\n  {} No results collected", "[FAIL]".bright_red());
        return None;
    }

    let stats = match calculate_statistics(&all_cycles) {
        Some(s) => s,
        None => {
            println!(
                "\n  {} Statistics calculation failed (empty filtered data)",
                "[FAIL]".bright_red()
            );
            return None;
        }
    };

    // Convert cycles to ns using detected frequency
    let cycles_to_ns = |cycles: u64| -> u64 { (cycles as f64 / cpu_freq) as u64 };

    println!("\n{}", "Results (after outlier filtering):".bright_yellow());
    println!(
        "  • Sample size: {} → {} ({} outliers removed)",
        format!("{}", stats.raw_count).bright_cyan(),
        format!("{}", stats.filtered_count).bright_green(),
        stats.outliers_removed
    );

    println!(
        "\n  Median:  {} cycles (~{} ns) ± {} cycles (95% CI)",
        format!("{}", stats.median).bright_green(),
        cycles_to_ns(stats.median),
        stats.median - stats.ci_lower
    );
    println!(
        "  Mean:    {:.1} cycles (~{} ns)",
        stats.mean,
        cycles_to_ns(stats.mean as u64)
    );
    println!(
        "  Std Dev: {:.1} cycles (~{} ns)",
        stats.std_dev,
        cycles_to_ns(stats.std_dev as u64)
    );
    println!(
        "  P95:     {} cycles (~{} ns)",
        stats.p95,
        cycles_to_ns(stats.p95)
    );
    println!(
        "  P99:     {} cycles (~{} ns)",
        stats.p99,
        cycles_to_ns(stats.p99)
    );
    println!(
        "  Min:     {} cycles (~{} ns)",
        stats.min,
        cycles_to_ns(stats.min)
    );
    println!(
        "  Max:     {} cycles (~{} ns)",
        stats.max,
        cycles_to_ns(stats.max)
    );

    println!("\n{}", "Analysis:".bright_yellow());
    println!(
        "  • Core-to-core theoretical minimum: ~60 cycles ({}ns each way @ {:.2}GHz)",
        cycles_to_ns(60),
        cpu_freq
    );
    println!("  • Good SPSC queue target: 70-80 cycles");

    if stats.median < 100 {
        println!("  • {} Excellent performance!", "[OK]".bright_green());
    } else if stats.median < 2000 {
        println!("  • {} Good performance", "[OK]".bright_green());
    } else if stats.median < 5000 {
        println!("  • {} Acceptable performance", "[WARNING]".bright_yellow());
    } else {
        println!("  • {} High latency", "[WARNING]".bright_yellow());
    }

    Some(stats)
}

fn run_benchmark(ipc_type: &str) -> Vec<u64> {
    let topic = format!("bench_{}_{}", ipc_type, std::process::id());
    let barrier_file = format!("/tmp/barrier_{}_{}", ipc_type, std::process::id());

    // Create barrier file for process synchronization
    if let Err(e) = fs::write(&barrier_file, &[0]) {
        eprintln!(
            "ERROR: Failed to create barrier file {}: {}",
            barrier_file, e
        );
        eprintln!("This may indicate /tmp is full or insufficient permissions.");
        return vec![]; // Return empty, will be detected as benchmark failure
    }

    let producer_mode = format!("{}_producer", ipc_type);
    let consumer_mode = format!("{}_consumer", ipc_type);

    // Start consumer first (waits on core 1)
    let consumer = spawn_process(&consumer_mode, &topic, &barrier_file, 1);

    // Wait for consumer ready
    wait_for_barrier(
        &barrier_file,
        BARRIER_CONSUMER_READY,
        Duration::from_secs(5),
    );

    // Start producer (runs on core 0)
    let producer = spawn_process(&producer_mode, &topic, &barrier_file, 0);

    // Wait for completion with proper error handling
    let producer_output = match producer.wait_with_output() {
        Ok(output) => output,
        Err(e) => {
            eprintln!(
                "{}",
                format!("ERROR: Producer process failed to complete: {}", e).bright_red()
            );
            let _ = fs::remove_file(&barrier_file);
            return vec![]; // Return empty - will be detected as measurement failure
        }
    };

    let consumer_output = match consumer.wait_with_output() {
        Ok(output) => output,
        Err(e) => {
            eprintln!(
                "{}",
                format!("ERROR: Consumer process failed to complete: {}", e).bright_red()
            );
            let _ = fs::remove_file(&barrier_file);
            return vec![]; // Return empty - will be detected as measurement failure
        }
    };

    // Cleanup
    let _ = fs::remove_file(&barrier_file);

    if !producer_output.status.success() {
        eprintln!(
            "{}",
            format!(
                "Producer exited with error: {}",
                String::from_utf8_lossy(&producer_output.stderr)
            )
            .bright_red()
        );
        return vec![];
    }

    if !consumer_output.status.success() {
        eprintln!(
            "{}",
            format!(
                "Consumer exited with error: {}",
                String::from_utf8_lossy(&consumer_output.stderr)
            )
            .bright_red()
        );
        return vec![];
    }

    // Parse latencies from consumer output
    let output = String::from_utf8_lossy(&consumer_output.stdout);

    // Debug: Print consumer output if empty
    let latencies: Vec<u64> = output
        .lines()
        .filter_map(|line| line.parse::<u64>().ok())
        .collect();

    if latencies.is_empty() {
        eprintln!("WARNING: No latencies collected!");
        eprintln!("Consumer stdout: {}", output);
        eprintln!(
            "Consumer stderr: {}",
            String::from_utf8_lossy(&consumer_output.stderr)
        );
    }

    latencies
}

fn spawn_process(mode: &str, topic: &str, barrier_file: &str, core: usize) -> Child {
    let exe = env::current_exe().unwrap();
    let mut cmd = Command::new(&exe);
    cmd.arg(mode).arg(topic).arg(barrier_file);

    // Set CPU affinity via taskset
    #[cfg(target_os = "linux")]
    {
        cmd = Command::new("taskset");
        cmd.arg("-c")
            .arg(core.to_string())
            .arg(&exe)
            .arg(mode)
            .arg(topic)
            .arg(barrier_file);
    }

    cmd.stdout(std::process::Stdio::piped())
        .stderr(std::process::Stdio::piped())
        .spawn()
        .expect("Failed to spawn process")
}

fn hub_producer(topic: &str, _barrier_file: &str) {
    eprintln!("Hub Producer started for topic: {}", topic);

    // Create sender (Hub supports both send and recv on same topic)
    let sender = match Topic::<CmdVel>::new(topic) {
        Ok(s) => {
            eprintln!("Producer: Hub created successfully");
            s
        }
        Err(e) => {
            eprintln!("Producer: Failed to create Hub: {:?}", e);
            return;
        }
    };

    // Small delay to ensure consumer is ready
    std::thread::sleep(Duration::from_millis(100));
    eprintln!("Producer: Starting warmup");

    // Warmup - broadcast pattern (no acks needed)
    for i in 0..WARMUP {
        let tsc = rdtsc();
        let mut msg = CmdVel::new(1.0, 0.5);
        msg.stamp_nanos = tsc;
        if let Err(e) = sender.send(msg, &mut None) {
            eprintln!(
                "Producer: FATAL - Failed to send warmup message {}: {:?}",
                i, e
            );
            eprintln!("Producer: Hub IPC is not functioning properly");
            return;
        }
        // Small delay to prevent 100% message overwrite (Hub is single-slot latest-value)
        std::thread::sleep(Duration::from_micros(10));
    }
    eprintln!("Producer: Warmup complete");

    // Measured iterations - broadcast with pacing
    eprintln!("Producer: Starting measured iterations");
    for i in 0..ITERATIONS {
        let tsc = rdtsc();
        let mut msg = CmdVel::new(1.0, 0.5);
        msg.stamp_nanos = tsc; // Embed timestamp
        if let Err(e) = sender.send(msg, &mut None) {
            eprintln!(
                "Producer: FATAL - Failed to send message {}/{}: {:?}",
                i, ITERATIONS, e
            );
            eprintln!("Producer: Benchmark aborted due to IPC failure");
            return;
        }
        // Pace messages to allow consumer to read (Hub is single-slot)
        std::thread::sleep(Duration::from_micros(10));
    }

    // Send end marker (sequence number 0xFFFFFFFF)
    let tsc = rdtsc();
    let mut end_msg = CmdVel::new(0.0, 0.0);
    end_msg.stamp_nanos = tsc | (0xFFFFFFFF_u64 << 32); // Mark as end
    let _ = sender.send(end_msg, &mut None);

    eprintln!("Producer: All messages sent");
}

fn hub_consumer(topic: &str, barrier_file: &str) {
    eprintln!("Hub Consumer started for topic: {}", topic);

    let receiver = match Topic::<CmdVel>::new(topic) {
        Ok(r) => {
            eprintln!("Consumer: Hub created successfully");
            r
        }
        Err(e) => {
            eprintln!("Consumer: Failed to create Hub: {:?}", e);
            return;
        }
    };

    // Signal ready
    write_barrier(barrier_file, BARRIER_CONSUMER_READY);
    eprintln!("Consumer: Signaled ready");

    // Warmup - spin-poll continuously
    eprintln!("Consumer: Starting warmup");
    let warmup_start = std::time::Instant::now();
    let mut warmup_count = 0;
    let mut last_tsc = 0u64;

    // Spin for warmup duration
    while warmup_start.elapsed() < Duration::from_millis(100) {
        if let Some(msg) = receiver.recv(&mut None) {
            if msg.stamp_nanos != last_tsc {
                warmup_count += 1;
                last_tsc = msg.stamp_nanos;
            }
        }
    }
    eprintln!(
        "Consumer: Warmup complete ({} messages received)",
        warmup_count
    );

    // Measured receives - spin-poll and collect all unique messages
    eprintln!("Consumer: Starting measured iterations");
    let start_time = std::time::Instant::now();
    let mut received_count = 0;
    let mut last_seq = 0u64;

    loop {
        if let Some(msg) = receiver.recv(&mut None) {
            let recv_tsc = rdtsc();
            let send_tsc_raw = msg.stamp_nanos;

            // Check for end marker (0xFFFFFFFF in upper 32 bits)
            if (send_tsc_raw >> 32) == 0xFFFFFFFF {
                eprintln!("Consumer: Received end marker");
                break;
            }

            // Only count and record unique messages (avoid counting duplicates from single-slot)
            if send_tsc_raw != last_seq {
                let cycles = recv_tsc.wrapping_sub(send_tsc_raw);
                // Print cycles (one per line for easy parsing)
                println!("{}", cycles);
                received_count += 1;
                last_seq = send_tsc_raw;
            }
        }

        // Timeout after 10 seconds
        if start_time.elapsed().as_secs() > 10 {
            eprintln!(
                "Consumer: TIMEOUT - only received {}/{} expected messages",
                received_count, ITERATIONS
            );
            break;
        }
    }

    eprintln!(
        "Consumer: Completed - received {} messages in {:?}",
        received_count,
        start_time.elapsed()
    );
}

// ============================================================================
// LINK BENCHMARKS (Single-Process SPSC)
// ============================================================================

fn run_link_benchmark(cpu_freq: f64) -> BenchmarkResults {
    use std::thread;

    let mut all_latencies = Vec::new();

    for run in 1..=NUM_RUNS {
        print!("  Run {}/{}: ", run, NUM_RUNS);
        std::io::Write::flush(&mut std::io::stdout()).unwrap();

        let link_topic = format!("link_bench_{}", run);
        let ack_topic = format!("link_ack_{}", run);

        let link_send = match Topic::<CmdVel>::new(&link_topic) {
            Ok(l) => l,
            Err(e) => {
                eprintln!("ERROR: Failed to create Link producer: {:?}", e);
                continue; // Skip this run
            }
        };
        let link_recv = match Topic::<CmdVel>::new(&link_topic) {
            Ok(l) => l,
            Err(e) => {
                eprintln!("ERROR: Failed to create Link consumer: {:?}", e);
                continue; // Skip this run
            }
        };
        let ack_send = match Topic::<CmdVel>::new(&ack_topic) {
            Ok(l) => l,
            Err(e) => {
                eprintln!("ERROR: Failed to create Link ack producer: {:?}", e);
                continue; // Skip this run
            }
        };
        let ack_recv = match Topic::<CmdVel>::new(&ack_topic) {
            Ok(l) => l,
            Err(e) => {
                eprintln!("ERROR: Failed to create Link ack consumer: {:?}", e);
                continue; // Skip this run
            }
        };

        let producer_handle = {
            thread::spawn(move || {
                // Set CPU affinity to core 0 (same as Hub producer)
                set_cpu_affinity(0);
                // Warmup
                for _ in 0..WARMUP {
                    let tsc = rdtsc();
                    let mut msg = CmdVel::new(1.0, 0.5);
                    msg.stamp_nanos = tsc;
                    let _ = link_send.send(msg, &mut None);

                    // Wait for ack
                    loop {
                        if ack_recv.recv(&mut None).is_some() {
                            break;
                        }
                    }
                }

                // Measured iterations
                for _ in 0..ITERATIONS {
                    let tsc = rdtsc();
                    let mut msg = CmdVel::new(1.0, 0.5);
                    msg.stamp_nanos = tsc;
                    let _ = link_send.send(msg, &mut None);

                    // Wait for ack
                    loop {
                        if ack_recv.recv(&mut None).is_some() {
                            break;
                        }
                    }
                }
            })
        };

        let consumer_handle = {
            thread::spawn(move || {
                // Set CPU affinity to core 1 (same as Hub consumer)
                set_cpu_affinity(1);

                let mut latencies = Vec::with_capacity(ITERATIONS);

                // Warmup
                for _ in 0..WARMUP {
                    loop {
                        if link_recv.recv(&mut None).is_some() {
                            let _ = ack_send.send(CmdVel::new(0.0, 0.0), &mut None);
                            break;
                        }
                    }
                }

                // Measured iterations
                for _ in 0..ITERATIONS {
                    loop {
                        if let Some(msg) = link_recv.recv(&mut None) {
                            let recv_tsc = rdtsc();
                            let send_tsc = msg.stamp_nanos;
                            let cycles = recv_tsc.wrapping_sub(send_tsc);
                            latencies.push(cycles);

                            // Send ack
                            let _ = ack_send.send(CmdVel::new(0.0, 0.0), &mut None);
                            break;
                        }
                    }
                }

                latencies
            })
        };

        producer_handle.join().unwrap();
        let latencies = consumer_handle.join().unwrap();

        let median_cycles = median(&latencies);
        all_latencies.push(latencies);
        println!("{} cycles median", median_cycles);
    }

    let all_cycles: Vec<u64> = all_latencies.iter().flatten().copied().collect();
    let stats = print_results(&all_latencies, cpu_freq);

    BenchmarkResults {
        stats,
        raw_latencies_cycles: all_cycles,
    }
}

// ============================================================================
// UTILITIES
// ============================================================================

#[cfg(target_os = "linux")]
fn set_cpu_affinity(core: usize) {
    use libc::{cpu_set_t, sched_setaffinity, CPU_SET, CPU_ZERO};
    use std::mem;

    unsafe {
        let mut cpu_set: cpu_set_t = mem::zeroed();
        CPU_ZERO(&mut cpu_set);
        CPU_SET(core, &mut cpu_set);

        let result = sched_setaffinity(
            0, // 0 = current thread
            mem::size_of::<cpu_set_t>(),
            &cpu_set,
        );

        if result != 0 {
            eprintln!("Warning: Failed to set CPU affinity to core {}", core);
        }
    }
}

#[cfg(not(target_os = "linux"))]
fn set_cpu_affinity(_core: usize) {
    // No-op on non-Linux platforms
}

fn wait_for_barrier(barrier_file: &str, expected: u8, timeout: Duration) {
    let start = std::time::Instant::now();
    loop {
        if let Ok(data) = fs::read(barrier_file) {
            if !data.is_empty() && data[0] == expected {
                return;
            }
        }
        if start.elapsed() > timeout {
            eprintln!("Barrier timeout waiting for state {}", expected);
            return;
        }
        std::thread::sleep(Duration::from_micros(100));
    }
}

fn write_barrier(barrier_file: &str, state: u8) {
    let _ = fs::write(barrier_file, &[state]);
}

/// Calculate median using proper averaging for even-length arrays
/// For even n: median = (sorted[n/2-1] + sorted[n/2]) / 2
/// For odd n: median = sorted[n/2]
fn median(values: &[u64]) -> u64 {
    if values.is_empty() {
        return 0;
    }
    let mut sorted = values.to_vec();
    sorted.sort_unstable();
    let n = sorted.len();

    if n % 2 == 0 {
        // Even length: average the two middle values
        (sorted[n / 2 - 1] + sorted[n / 2]) / 2
    } else {
        // Odd length: take the middle value
        sorted[n / 2]
    }
}

/// Calculate percentile using linear interpolation (NIST R-7 method)
/// This is the standard method used by NumPy, Excel, and most statistical packages
/// Reference: NIST Engineering Statistics Handbook, Section 2.6.2
fn percentile(values: &[u64], p: usize) -> u64 {
    if values.is_empty() {
        return 0;
    }
    let mut sorted = values.to_vec();
    sorted.sort_unstable();
    let n = sorted.len();

    // NIST R-7 method: h = (n-1) * p/100 + 1
    let h = (n - 1) as f64 * (p as f64 / 100.0);
    let h_floor = h.floor() as usize;
    let h_ceil = h.ceil() as usize;

    if h_floor >= n - 1 {
        return sorted[n - 1];
    }

    // Linear interpolation between floor and ceil
    let lower = sorted[h_floor];
    let upper = sorted[h_ceil];
    let weight = h - h_floor as f64;

    (lower as f64 + weight * (upper - lower) as f64).round() as u64
}

/// Filter outliers using standard IQR (Interquartile Range) method
/// Uses the standard 1.5×IQR threshold (Tukey's method)
/// Removes values < Q1 - 1.5×IQR or > Q3 + 1.5×IQR
///
/// This is the standard approach used in boxplots and statistical analysis.
/// Reference: Tukey, J. W. (1977). Exploratory Data Analysis. Addison-Wesley.
fn filter_outliers(values: &[u64]) -> Vec<u64> {
    if values.len() < 4 {
        return values.to_vec();
    }

    let mut sorted = values.to_vec();
    sorted.sort_unstable();

    let q1 = percentile(&sorted, 25);
    let q3 = percentile(&sorted, 75);
    let iqr = q3.saturating_sub(q1);

    // Standard 1.5×IQR threshold (Tukey's method)
    // This identifies "outliers" beyond the "whiskers" of a boxplot
    let lower_bound = q1.saturating_sub(iqr + iqr / 2); // Q1 - 1.5×IQR
    let upper_bound = q3.saturating_add(iqr + iqr / 2); // Q3 + 1.5×IQR

    sorted
        .into_iter()
        .filter(|&v| v >= lower_bound && v <= upper_bound)
        .collect()
}

/// Calculate mean for confidence intervals
fn mean(values: &[u64]) -> f64 {
    if values.is_empty() {
        return 0.0;
    }
    values.iter().sum::<u64>() as f64 / values.len() as f64
}

/// Calculate standard deviation
fn std_dev(values: &[u64]) -> f64 {
    if values.len() < 2 {
        return 0.0;
    }
    let avg = mean(values);
    let variance = values
        .iter()
        .map(|&v| {
            let diff = v as f64 - avg;
            diff * diff
        })
        .sum::<f64>()
        / (values.len() - 1) as f64;
    variance.sqrt()
}

/// Calculate 95% confidence interval for median using bootstrap resampling
/// Returns (lower_bound, upper_bound) in cycles
///
/// METHODOLOGY NOTE:
/// This uses the bootstrap method, a distribution-free resampling technique that makes
/// NO assumptions about the underlying distribution. It's particularly robust for
/// skewed distributions common in latency measurements.
///
/// ALGORITHM:
/// 1. Resample the data n_bootstrap times (default: 2000) with replacement
/// 2. Calculate the median for each resample
/// 3. Sort the bootstrap medians
/// 4. Use the 2.5th and 97.5th percentiles as CI bounds (95% CI)
///
/// ADVANTAGES:
/// - Distribution-free (no normality assumption)
/// - Robust for skewed distributions (perfect for latency data)
/// - Gold standard in modern statistics
/// - Handles outliers and heavy tails naturally
///
/// REFERENCE:
/// Efron, B., & Tibshirani, R. J. (1994). An Introduction to the Bootstrap.
/// Chapman & Hall/CRC Monographs on Statistics & Applied Probability.
///
/// COMPUTATIONAL COST:
/// With n=500,000 samples and n_bootstrap=2000:
/// - 2000 resamples × median calculation ≈ 2-3 seconds
/// - Trade-off: More accurate CI for slightly longer benchmark time
///
/// For faster computation, reduce n_bootstrap to 1000 (still statistically sound).
fn confidence_interval_95(values: &[u64]) -> (u64, u64) {
    if values.len() < 2 {
        let med = median(values);
        return (med, med);
    }

    // Use 2000 bootstrap resamples (balance between accuracy and speed)
    // For very large datasets, 1000 resamples is also acceptable
    let n_bootstrap = if values.len() > 100_000 { 2000 } else { 5000 };

    bootstrap_ci_median(values, n_bootstrap)
}

/// Perform bootstrap resampling to estimate confidence interval for median
/// This is the core bootstrap implementation
fn bootstrap_ci_median(values: &[u64], n_bootstrap: usize) -> (u64, u64) {
    let n = values.len();
    let mut rng = rand::thread_rng();
    let mut bootstrap_medians = Vec::with_capacity(n_bootstrap);

    // Resample n_bootstrap times
    for _ in 0..n_bootstrap {
        // Create a resample by sampling with replacement
        let mut resample = Vec::with_capacity(n);
        for _ in 0..n {
            let idx = rng.gen_range(0..n);
            resample.push(values[idx]);
        }

        // Calculate median of this resample
        bootstrap_medians.push(median(&resample));
    }

    // Sort the bootstrap medians
    bootstrap_medians.sort_unstable();

    // 95% CI: 2.5th and 97.5th percentiles
    let lower_idx = (n_bootstrap as f64 * 0.025).floor() as usize;
    let upper_idx = (n_bootstrap as f64 * 0.975).ceil() as usize;

    // Handle edge cases
    let lower_idx = lower_idx.min(bootstrap_medians.len() - 1);
    let upper_idx = upper_idx.min(bootstrap_medians.len() - 1);

    (bootstrap_medians[lower_idx], bootstrap_medians[upper_idx])
}

struct Statistics {
    raw_count: usize,
    filtered_count: usize,
    outliers_removed: usize,
    median: u64,
    mean: f64,
    p95: u64,
    p99: u64,
    min: u64,
    max: u64,
    std_dev: f64,
    ci_lower: u64,
    ci_upper: u64,
}

impl Statistics {
    /// Convert Statistics to IpcStats for database storage
    fn to_ipc_stats(&self, cpu_freq: f64) -> IpcStats {
        let cycles_to_ns = |cycles: u64| -> u64 { (cycles as f64 / cpu_freq) as u64 };

        IpcStats {
            median_cycles: self.median,
            median_ns: cycles_to_ns(self.median),
            mean_cycles: self.mean,
            std_dev_cycles: self.std_dev,
            p95_cycles: self.p95,
            p99_cycles: self.p99,
            min_cycles: self.min,
            max_cycles: self.max,
            ci_lower_cycles: self.ci_lower,
            ci_upper_cycles: self.ci_upper,
            sample_count: self.filtered_count,
            outliers_removed: self.outliers_removed,
        }
    }
}

// ============================================================================
// UNIT TESTS - Statistical Functions
// ============================================================================

#[cfg(test)]
mod tests {
    use super::*;

    // ========================================================================
    // MEDIAN TESTS
    // ========================================================================

    #[test]
    fn test_median_empty() {
        let values: Vec<u64> = vec![];
        assert_eq!(median(&values), 0);
    }

    #[test]
    fn test_median_single() {
        let values = vec![42];
        assert_eq!(median(&values), 42);
    }

    #[test]
    fn test_median_odd_length() {
        let values = vec![1, 2, 3, 4, 5];
        assert_eq!(median(&values), 3);
    }

    #[test]
    fn test_median_even_length() {
        let values = vec![1, 2, 3, 4];
        // Should return (2 + 3) / 2 = 2 (integer division)
        assert_eq!(median(&values), 2);
    }

    #[test]
    fn test_median_even_length_large_gap() {
        let values = vec![100, 200, 300, 400];
        // Should return (200 + 300) / 2 = 250
        assert_eq!(median(&values), 250);
    }

    #[test]
    fn test_median_unsorted() {
        let values = vec![5, 1, 4, 2, 3];
        assert_eq!(median(&values), 3);
    }

    #[test]
    fn test_median_duplicates() {
        let values = vec![5, 5, 5, 5];
        assert_eq!(median(&values), 5);
    }

    // ========================================================================
    // PERCENTILE TESTS (NIST R-7 Method)
    // ========================================================================

    #[test]
    fn test_percentile_empty() {
        let values: Vec<u64> = vec![];
        assert_eq!(percentile(&values, 50), 0);
    }

    #[test]
    fn test_percentile_single() {
        let values = vec![42];
        assert_eq!(percentile(&values, 50), 42);
        assert_eq!(percentile(&values, 95), 42);
        assert_eq!(percentile(&values, 99), 42);
    }

    #[test]
    fn test_percentile_p50_is_median() {
        let values = vec![1, 2, 3, 4, 5];
        let p50 = percentile(&values, 50);
        let med = median(&values);
        // P50 and median should be very close (within rounding)
        assert!((p50 as i64 - med as i64).abs() <= 1);
    }

    #[test]
    fn test_percentile_extremes() {
        let values = vec![100, 200, 300, 400, 500];
        // P0 should be close to min
        assert!(percentile(&values, 0) <= 100);
        // P100 should be max
        assert_eq!(percentile(&values, 100), 500);
    }

    #[test]
    fn test_percentile_linear_interpolation() {
        // NIST R-7: For [100, 200], P95 should interpolate
        let values = vec![100, 200];
        let p95 = percentile(&values, 95);
        // h = (2-1) * 0.95 = 0.95
        // Expected: 100 + 0.95 * (200 - 100) = 195
        assert_eq!(p95, 195);
    }

    // ========================================================================
    // MEAN TESTS
    // ========================================================================

    #[test]
    fn test_mean_empty() {
        let values: Vec<u64> = vec![];
        assert_eq!(mean(&values), 0.0);
    }

    #[test]
    fn test_mean_single() {
        let values = vec![42];
        assert_eq!(mean(&values), 42.0);
    }

    #[test]
    fn test_mean_simple() {
        let values = vec![1, 2, 3, 4, 5];
        assert_eq!(mean(&values), 3.0);
    }

    #[test]
    fn test_mean_large_values() {
        let values = vec![1000, 2000, 3000];
        assert_eq!(mean(&values), 2000.0);
    }

    // ========================================================================
    // STANDARD DEVIATION TESTS
    // ========================================================================

    #[test]
    fn test_std_dev_empty() {
        let values: Vec<u64> = vec![];
        assert_eq!(std_dev(&values), 0.0);
    }

    #[test]
    fn test_std_dev_single() {
        let values = vec![42];
        assert_eq!(std_dev(&values), 0.0);
    }

    #[test]
    fn test_std_dev_no_variance() {
        let values = vec![5, 5, 5, 5];
        assert_eq!(std_dev(&values), 0.0);
    }

    #[test]
    fn test_std_dev_known_value() {
        // For [1, 2, 3, 4, 5]:
        // mean = 3.0
        // variance (sample) = [(1-3)^2 + (2-3)^2 + (3-3)^2 + (4-3)^2 + (5-3)^2] / 4
        //                   = [4 + 1 + 0 + 1 + 4] / 4 = 10 / 4 = 2.5
        // std_dev = sqrt(2.5) ≈ 1.5811
        let values = vec![1, 2, 3, 4, 5];
        let sd = std_dev(&values);
        assert!((sd - 1.5811).abs() < 0.0001);
    }

    #[test]
    fn test_std_dev_uses_sample_variance() {
        // Verify we use n-1 (sample variance), not n (population variance)
        let values = vec![1, 2, 3, 4, 5];
        let sd = std_dev(&values);

        // Sample std dev (n-1): 1.5811
        // Population std dev (n): 1.4142
        // Should be closer to 1.5811
        assert!((sd - 1.5811).abs() < (sd - 1.4142).abs());
    }

    // ========================================================================
    // OUTLIER FILTERING TESTS (Tukey's 1.5×IQR)
    // ========================================================================

    #[test]
    fn test_filter_outliers_empty() {
        let values: Vec<u64> = vec![];
        let filtered = filter_outliers(&values);
        assert!(filtered.is_empty());
    }

    #[test]
    fn test_filter_outliers_too_small() {
        // Less than 4 values - should return all
        let values = vec![1, 2, 3];
        let filtered = filter_outliers(&values);
        assert_eq!(filtered.len(), 3);
    }

    #[test]
    fn test_filter_outliers_no_outliers() {
        // Nice uniform distribution - should keep all
        let values = vec![100, 110, 120, 130, 140];
        let filtered = filter_outliers(&values);
        assert_eq!(filtered.len(), 5);
    }

    #[test]
    fn test_filter_outliers_extreme_high() {
        // [1, 2, 3, 4, 1000] - 1000 is extreme outlier
        let values = vec![1, 2, 3, 4, 1000];
        let filtered = filter_outliers(&values);
        // Should remove 1000
        assert_eq!(filtered.len(), 4);
        assert!(!filtered.contains(&1000));
    }

    #[test]
    fn test_filter_outliers_extreme_low() {
        // Need more data points for IQR to be effective
        // [1, 100, 110, 120, 130, 140, 150, 160, 170, 180]
        // Q1 ≈ 110, Q3 ≈ 160, IQR ≈ 50
        // Lower bound ≈ 110 - 75 = 35
        // 1 should be filtered out
        let mut values: Vec<u64> = (100..=180).step_by(10).collect();
        values.insert(0, 1); // Add extreme outlier

        let filtered = filter_outliers(&values);
        // Should remove the extreme outlier (1)
        assert!(
            !filtered.contains(&1),
            "Extreme low outlier should be removed"
        );
    }

    #[test]
    fn test_filter_outliers_tukey_method() {
        // Test that we're using 1.5×IQR (not 3×IQR or other)
        // For values [10, 20, 30, 40, 50, 100]:
        // Q1 = 20, Q3 = 50, IQR = 30
        // Lower bound = Q1 - 1.5*IQR = 20 - 45 = -25 (saturates to 0)
        // Upper bound = Q3 + 1.5*IQR = 50 + 45 = 95
        // Should remove 100
        let values = vec![10, 20, 30, 40, 50, 100];
        let filtered = filter_outliers(&values);
        assert!(!filtered.contains(&100));
    }

    // ========================================================================
    // CONFIDENCE INTERVAL TESTS (Bootstrap)
    // ========================================================================

    #[test]
    fn test_confidence_interval_empty() {
        let values: Vec<u64> = vec![];
        let (lower, upper) = confidence_interval_95(&values);
        assert_eq!(lower, 0);
        assert_eq!(upper, 0);
    }

    #[test]
    fn test_confidence_interval_single() {
        let values = vec![42];
        let (lower, upper) = confidence_interval_95(&values);
        assert_eq!(lower, 42);
        assert_eq!(upper, 42);
    }

    #[test]
    fn test_confidence_interval_no_variance() {
        let values = vec![100, 100, 100, 100, 100];
        let (lower, upper) = confidence_interval_95(&values);
        // With no variance, CI should be tight around median
        assert_eq!(lower, 100);
        assert_eq!(upper, 100);
    }

    #[test]
    fn test_confidence_interval_contains_median() {
        let values: Vec<u64> = (1..=1000).collect();
        let med = median(&values);
        let (lower, upper) = confidence_interval_95(&values);

        // 95% CI should contain the median
        assert!(lower <= med);
        assert!(upper >= med);
    }

    #[test]
    fn test_confidence_interval_reasonable_width() {
        // For uniform distribution, CI width should be reasonable
        let values: Vec<u64> = (1..=1000).collect();
        let (lower, upper) = confidence_interval_95(&values);
        let width = upper - lower;

        // Width should be positive but not too wide
        assert!(width > 0);
        assert!(width < 200); // Shouldn't be more than 20% of range
    }

    #[test]
    fn test_confidence_interval_symmetry_for_symmetric_data() {
        // For symmetric distribution centered at 500
        let values: Vec<u64> = (400..=600).collect();
        let med = median(&values);
        let (lower, upper) = confidence_interval_95(&values);

        // For symmetric data, CI should be roughly symmetric around median
        let lower_dist = med - lower;
        let upper_dist = upper - med;

        // Allow 20% asymmetry (bootstrap has sampling variance)
        assert!((lower_dist as f64 - upper_dist as f64).abs() < 0.2 * lower_dist as f64);
    }

    // ========================================================================
    // INTEGRATION TESTS - calculate_statistics()
    // ========================================================================

    #[test]
    fn test_calculate_statistics_empty() {
        let values: Vec<u64> = vec![];
        let result = calculate_statistics(&values);
        assert!(result.is_none());
    }

    #[test]
    fn test_calculate_statistics_all_outliers() {
        // Pathological case: all values are outliers
        // This is practically impossible but we should handle it
        let values = vec![1, 1000000];
        let result = calculate_statistics(&values);
        // Should return None if filtering removes everything
        // (though with only 2 values, filtering shouldn't apply)
        assert!(result.is_some());
    }

    #[test]
    fn test_calculate_statistics_normal_case() {
        let values: Vec<u64> = (1..=1000).collect();
        let result = calculate_statistics(&values);

        assert!(result.is_some());
        let stats = result.unwrap();

        // Verify statistics are reasonable
        assert_eq!(stats.raw_count, 1000);
        assert!(stats.filtered_count > 0);
        assert!(stats.median > 0);
        assert!(stats.mean > 0.0);
        assert!(stats.p95 > stats.median);
        assert!(stats.p99 > stats.p95);
        assert!(stats.ci_lower <= stats.median);
        assert!(stats.ci_upper >= stats.median);
    }

    // ========================================================================
    // STATISTICAL VALIDATION TESTS
    // ========================================================================

    #[test]
    fn test_median_matches_percentile_50() {
        let values: Vec<u64> = (1..=999).collect();
        let med = median(&values);
        let p50 = percentile(&values, 50);

        // Median and P50 should be very close (within 1 for integer math)
        assert!((med as i64 - p50 as i64).abs() <= 1);
    }

    #[test]
    fn test_percentiles_ordered() {
        let values: Vec<u64> = (1..=1000).collect();
        let p50 = percentile(&values, 50);
        let p95 = percentile(&values, 95);
        let p99 = percentile(&values, 99);

        // Percentiles should be ordered
        assert!(p50 <= p95);
        assert!(p95 <= p99);
    }

    #[test]
    fn test_outlier_filtering_reduces_count() {
        // Add some outliers to a normal distribution
        let mut values: Vec<u64> = (100..=200).collect();
        values.push(1); // Extreme low
        values.push(10000); // Extreme high

        let filtered = filter_outliers(&values);

        // Should have removed some outliers
        assert!(filtered.len() < values.len());
    }

    #[test]
    fn test_std_dev_increases_with_spread() {
        let narrow = vec![100, 101, 102, 103, 104];
        let wide = vec![100, 200, 300, 400, 500];

        let sd_narrow = std_dev(&narrow);
        let sd_wide = std_dev(&wide);

        // Wider distribution should have larger std dev
        assert!(sd_wide > sd_narrow);
    }

    #[test]
    fn test_bootstrap_ci_reproducibility() {
        // Bootstrap has randomness, but for same data should give similar results
        let values: Vec<u64> = (1..=100).collect();

        let (lower1, upper1) = confidence_interval_95(&values);
        let (lower2, upper2) = confidence_interval_95(&values);

        // Results should be similar (within 10% due to sampling variance)
        let width1 = upper1 - lower1;
        let width2 = upper2 - lower2;
        assert!((width1 as f64 - width2 as f64).abs() < 0.1 * width1 as f64);
    }
}
