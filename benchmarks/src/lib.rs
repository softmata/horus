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

pub mod stats;
pub mod platform;
pub mod output;
pub mod timing;

use serde::{Deserialize, Serialize};

// Re-exports for convenience
pub use stats::{
    bootstrap_ci, calculate_percentile, coefficient_of_variation, excess_kurtosis,
    filter_outliers, jarque_bera_test, median, skewness, std_dev, NormalityAnalysis, Statistics,
};
pub use platform::{CpuInfo, PlatformInfo, detect_platform, detect_cpu_frequency};
pub use output::{BenchmarkReport, write_json_report, write_csv_report};
pub use timing::{rdtsc, calibrate_rdtsc, cycles_to_ns};

/// Standard message sizes used in robotics applications (bytes)
pub const MESSAGE_SIZES: &[(&str, usize)] = &[
    ("control_cmd", 16),       // CmdVel: 2x f32
    ("motor_cmd", 64),         // Motor command with metadata
    ("imu_reading", 128),      // Basic IMU data
    ("sensor_fusion", 256),    // Fused sensor state
    ("lidar_scan", 4096),      // Single LiDAR scan line
    ("point_cloud", 65536),    // Small point cloud
    ("camera_frame", 1_000_000), // 640x480 grayscale
    ("map_update", 10_000_000),  // Large map chunk
];

/// Common frequencies in robotics systems (Hz)
pub const FREQUENCIES: &[(&str, u32)] = &[
    ("servo_loop", 10000),     // Fast servo control
    ("control_loop", 1000),    // Standard control
    ("imu_rate", 400),         // IMU sampling
    ("planning", 100),         // Motion planning
    ("localization", 50),      // SLAM/localization
    ("perception", 30),        // Vision processing
    ("lidar", 10),             // LiDAR spin rate
];

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

/// Benchmark message for testing (variable size payload)
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct BenchmarkMessage {
    pub id: u64,
    pub timestamp_ns: u64,
    #[serde(with = "serde_bytes")]
    pub payload: Vec<u8>,
}

impl BenchmarkMessage {
    pub fn new(id: u64, payload_size: usize) -> Self {
        Self {
            id,
            timestamp_ns: std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap()
                .as_nanos() as u64,
            payload: vec![0u8; payload_size],
        }
    }

    /// Create with embedded RDTSC timestamp for cycle-accurate latency
    #[cfg(target_arch = "x86_64")]
    pub fn with_rdtsc(id: u64, payload_size: usize) -> Self {
        Self {
            id,
            timestamp_ns: timing::rdtsc(),
            payload: vec![0u8; payload_size],
        }
    }

    #[cfg(not(target_arch = "x86_64"))]
    pub fn with_rdtsc(id: u64, payload_size: usize) -> Self {
        Self::new(id, payload_size)
    }
}
