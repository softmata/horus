//! # HORUS Benchmark Suite
//!
//! A rigorous, industry-grade benchmark suite for the HORUS robotics framework.
//!
//! ## Architecture
//!
//! - **benches/**: Criterion-based microbenchmarks (latency, throughput, scalability)
//! - **tests/**: Integration tests (determinism, real-time, stress)
//! - **compare/**: Competitive comparisons (vs crossbeam, channels, etc.)
//!
//! ## Methodology
//!
//! All benchmarks follow these principles:
//! - **Statistical rigor**: Bootstrap confidence intervals, outlier filtering
//! - **Platform awareness**: CPU detection, frequency measurement, NUMA topology
//! - **Reproducibility**: JSON output for regression tracking, determinism metrics
//! - **Real-world relevance**: Robotics message types, realistic workloads

pub mod output;
pub mod platform;
pub mod stats;
pub mod timing;

use serde::{Deserialize, Serialize};

// Re-exports for convenience
pub use output::{write_csv_report, write_json_report, BenchmarkReport};
pub use platform::{detect_cpu_frequency, detect_platform, CpuInfo, PlatformInfo};
pub use stats::{
    bootstrap_ci, calculate_percentile, coefficient_of_variation, excess_kurtosis, filter_outliers,
    jarque_bera_test, median, skewness, std_dev, NormalityAnalysis, Statistics,
};
pub use timing::{calibrate_rdtsc, cycles_to_ns, rdtsc};

/// Benchmark configuration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct BenchmarkConfig {
    /// Number of warmup iterations
    pub warmup_iterations: usize,
    /// Number of measured iterations
    pub iterations: usize,
    /// Number of independent runs for variance analysis
    pub runs: usize,
    /// CPU cores to pin producer/consumer
    pub cpu_affinity: Option<(usize, usize)>,
    /// Whether to filter outliers
    pub filter_outliers: bool,
    /// Confidence interval percentage (e.g., 95.0)
    pub confidence_level: f64,
}

impl Default for BenchmarkConfig {
    fn default() -> Self {
        Self {
            warmup_iterations: 5_000,
            iterations: 50_000,
            runs: 10,
            cpu_affinity: Some((0, 1)),
            filter_outliers: true,
            confidence_level: 95.0,
        }
    }
}

/// Full benchmark result with all metrics
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct BenchmarkResult {
    /// Benchmark name
    pub name: String,
    /// What was tested (e.g., "HORUS Topic", "crossbeam channel")
    pub subject: String,
    /// Message size in bytes
    pub message_size: usize,
    /// Configuration used
    pub config: BenchmarkConfig,
    /// Platform information
    pub platform: PlatformInfo,
    /// Timestamp when benchmark was run
    pub timestamp: String,
    /// Raw latencies in nanoseconds
    pub raw_latencies_ns: Vec<u64>,
    /// Computed statistics
    pub statistics: Statistics,
    /// Throughput metrics
    pub throughput: ThroughputMetrics,
    /// Determinism metrics
    pub determinism: DeterminismMetrics,
}

/// Throughput measurements
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ThroughputMetrics {
    /// Messages per second
    pub messages_per_sec: f64,
    /// Bytes per second
    pub bytes_per_sec: f64,
    /// Total messages sent
    pub total_messages: u64,
    /// Total bytes transferred
    pub total_bytes: u64,
    /// Duration of throughput test
    pub duration_secs: f64,
}

/// Determinism/jitter metrics for real-time analysis
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DeterminismMetrics {
    /// Coefficient of variation (std_dev / mean) - lower is better
    pub cv: f64,
    /// Maximum observed jitter (max - min)
    pub max_jitter_ns: u64,
    /// 99.9th percentile latency
    pub p999: u64,
    /// 99.99th percentile latency
    pub p9999: u64,
    /// Number of deadline misses (latency > threshold)
    pub deadline_misses: u64,
    /// Deadline threshold used (ns)
    pub deadline_threshold_ns: u64,
    /// Run-to-run variance (variance of median across runs)
    pub run_variance: f64,
}

/// CPU governor management for consistent benchmarks
#[cfg(target_os = "linux")]
pub fn set_performance_governor() -> Result<(), Box<dyn std::error::Error>> {
    use std::process::Command;
    Command::new("sudo")
        .args(["cpupower", "frequency-set", "-g", "performance"])
        .output()?;
    Ok(())
}

#[cfg(not(target_os = "linux"))]
pub fn set_performance_governor() -> Result<(), Box<dyn std::error::Error>> {
    Ok(()) // No-op on non-Linux
}

/// Set CPU affinity for current thread
#[cfg(target_os = "linux")]
pub fn set_cpu_affinity(core: usize) -> Result<(), Box<dyn std::error::Error>> {
    use libc::{cpu_set_t, sched_setaffinity, CPU_SET, CPU_ZERO};
    use std::mem;

    // SAFETY: cpu_set is stack-allocated and zeroed before use. CPU_SET sets the
    // specified core bit. sched_setaffinity(0, ...) targets the current thread.
    // All pointers reference valid stack memory with correct sizes.
    unsafe {
        let mut cpu_set: cpu_set_t = mem::zeroed();
        CPU_ZERO(&mut cpu_set);
        CPU_SET(core, &mut cpu_set);

        let result = sched_setaffinity(0, mem::size_of::<cpu_set_t>(), &cpu_set);
        if result != 0 {
            return Err(format!("Failed to set CPU affinity to core {}", core).into());
        }
    }
    Ok(())
}

#[cfg(not(target_os = "linux"))]
pub fn set_cpu_affinity(_core: usize) -> Result<(), Box<dyn std::error::Error>> {
    Ok(()) // No-op on non-Linux
}

/// Warmup iterations to stabilize cache and branch prediction
pub fn warmup<F>(iterations: usize, mut f: F)
where
    F: FnMut(),
{
    for _ in 0..iterations {
        f();
        std::hint::black_box(());
    }
}

