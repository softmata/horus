//! # HORUS Benchmark Suite
//!
//! Benchmark suite for the HORUS robotics framework.
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
pub use platform::{detect_platform, CpuInfo, PlatformInfo};
pub use stats::{
    bootstrap_ci, calculate_percentile, coefficient_of_variation, excess_kurtosis, filter_outliers,
    jarque_bera_test, median, skewness, std_dev, NormalityAnalysis, Statistics,
};
pub use timing::{cycles_to_ns, rdtsc};

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
    #[serde(deserialize_with = "crate::stats::nan_from_null")]
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

/// Where a result's numbers came from.
///
/// `dds_comparison_benchmark` emitted entries for ROS2, CycloneDDS, FastDDS and
/// iceoryx when the `dds` feature was off — which was the default, and which
/// meant nothing was installed to measure. Those entries carried a full
/// percentile distribution (p1 through p99.99, plus confidence bounds), all of
/// it computed arithmetically from two hardcoded constants, and were written
/// into the JSON report alongside real measurements with nothing in the schema
/// to tell them apart. `count: 0` and an `_reference` name suffix were the only
/// hints, and neither survives a chart generator. That binary is gone; this
/// enum stays, because the moment a quoted figure enters a report again it
/// needs to arrive already marked.
///
/// A number a reader might quote in a comparison has to say where it came from.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq, Eq, Default)]
#[serde(tag = "kind", rename_all = "snake_case")]
pub enum Provenance {
    /// Produced by running the benchmark on this machine.
    #[default]
    Measured,
    /// Copied from published work. Not measured here, and not measured on this
    /// hardware, so it is not comparable to a `Measured` result without saying
    /// so.
    Literature {
        /// Where the figure came from, specifically enough to look up.
        source: String,
    },
}

/// How quiet the host was while a result was measured.
///
/// A far-tail percentile (`p99.9` and beyond) is dominated by whether the
/// process that produced it was preemptible: one involuntary context switch
/// during the measured window turns into a burst of samples that waited for
/// the scheduler, and the 0.1 % tail moves by an order of magnitude with no
/// code change. The rest of the distribution does not move — the median, p95
/// and p99 of a run in which the consumer was stalled for 46 µs are identical
/// to one without it (measured 2026-09-23: three repetitions, same 210 ns
/// median, p99.9 at 27 µs / 1.9 µs / 16 µs).
///
/// So a tail comparison is only meaningful when the evidence says the host was
/// quiet. Recording that evidence next to the numbers lets the gate decide
/// instead of inferring from the very percentile it is trying to judge.
///
/// Every field is `#[serde(default)]` so JSON written before this existed still
/// deserializes — as "no evidence", which [`Self::is_tail_valid`] reads as not
/// quiet.
#[derive(Debug, Clone, Serialize, Deserialize, Default)]
#[serde(default)]
pub struct MeasurementQuality {
    /// Whether the *measuring* process held `SCHED_FIFO` for the run.
    ///
    /// The consumer/subscriber side is the one that matters: if it is
    /// descheduled, the ring absorbs the stall and every message published
    /// during it waits its turn. A stall in the publisher merely pauses the
    /// stream, so it cannot produce a burst of delayed samples.
    pub rt_granted: bool,
    /// Whether the kernel was told to keep other tasks off the pinned cores
    /// (`isolcpus=` on the kernel command line, covering *these* cores).
    pub isolated_cores: bool,
    /// Voluntary context switches over the measured window.
    pub voluntary_ctx_switches: u64,
    /// Involuntary context switches over the measured window. The benchmark's
    /// own warning calls this "the mechanism behind the far tail".
    pub involuntary_ctx_switches: u64,
}

impl MeasurementQuality {
    /// Evidence from a measured window: `getrusage` deltas plus what this
    /// process did about scheduling.
    pub fn from_window(rt_granted: bool, cpus: &[usize], voluntary: u64, involuntary: u64) -> Self {
        Self {
            rt_granted,
            isolated_cores: isolcpus_cover(cpus),
            voluntary_ctx_switches: voluntary,
            involuntary_ctx_switches: involuntary,
        }
    }

    /// Evidence for a binary that measured something but did not attempt
    /// real-time scheduling: the host is as quiet as `isolcpus=` makes it.
    pub fn host_now(cpus: &[usize]) -> Self {
        Self::from_window(false, cpus, 0, 0)
    }

    /// Whether a far-tail number from this result describes the transport
    /// rather than the host's scheduler.
    ///
    /// Requires *evidence* that the measuring process could not be preempted
    /// (`SCHED_FIFO` or isolated cores) and that no involuntary context switch
    /// was actually observed. No evidence is not quiet: a run on a shared
    /// runner is not a run whose p99.9 means anything.
    pub fn is_tail_valid(&self) -> bool {
        (self.rt_granted || self.isolated_cores) && self.involuntary_ctx_switches == 0
    }

    /// Why [`Self::is_tail_valid`] said no, for a note or a warning line.
    pub fn tail_invalid_reason(&self) -> String {
        format!(
            "rt_granted={}, isolated_cores={}, involuntary_ctx_switches={}",
            self.rt_granted, self.isolated_cores, self.involuntary_ctx_switches
        )
    }
}

/// Whether `isolcpus=` on the kernel command line covers every CPU in `cpus`.
///
/// Reads `/proc/cmdline`. `false` when the kernel was not asked to isolate
/// anything, when the list does not cover a pinned core, or on non-Linux.
pub fn isolcpus_cover(cpus: &[usize]) -> bool {
    #[cfg(target_os = "linux")]
    {
        let Ok(cmdline) = std::fs::read_to_string("/proc/cmdline") else {
            return false;
        };
        cmdline
            .split_whitespace()
            .find_map(|token| token.strip_prefix("isolcpus="))
            .is_some_and(|spec| parse_isolcpus(spec, cpus))
    }
    #[cfg(not(target_os = "linux"))]
    {
        let _ = cpus;
        false
    }
}

/// `spec` is the value of `isolcpus=` (`2`, `2,4-7`, legacy `domain,2,4-7`).
// Only the Linux branch of `isolcpus_cover` calls this; the tests cover it everywhere.
#[cfg_attr(not(target_os = "linux"), allow(dead_code))]
pub(crate) fn parse_isolcpus(spec: &str, cpus: &[usize]) -> bool {
    if cpus.is_empty() {
        return false;
    }
    let mut parts: Vec<&str> = spec.split(',').filter(|p| !p.is_empty()).collect();
    // Legacy qualifier: `isolcpus=[domain,]<cpu-list>`.
    if parts
        .first()
        .is_some_and(|p| matches!(*p, "domain" | "managed_irq"))
    {
        parts.remove(0);
    }
    let mut ranges: Vec<(usize, usize)> = Vec::new();
    for part in parts {
        match part.split_once('-') {
            Some((lo, hi)) => {
                if let (Ok(lo), Ok(hi)) = (lo.parse::<usize>(), hi.parse::<usize>()) {
                    if lo <= hi {
                        ranges.push((lo, hi));
                    }
                }
            }
            None => {
                if let Ok(n) = part.parse::<usize>() {
                    ranges.push((n, n));
                }
            }
        }
    }
    cpus.iter()
        .all(|cpu| ranges.iter().any(|(lo, hi)| lo <= cpu && cpu <= hi))
}

/// Full benchmark result with all metrics
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct BenchmarkResult {
    /// Whether these numbers were measured here or quoted from published work.
    #[serde(default)]
    pub provenance: Provenance,
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
    /// How quiet the host was while this was measured.
    ///
    /// Absent in JSON written before this field existed, hence `default`.
    #[serde(default)]
    pub measurement_quality: MeasurementQuality,
}

/// Throughput measurements
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ThroughputMetrics {
    /// Messages per second
    #[serde(deserialize_with = "crate::stats::nan_from_null")]
    pub messages_per_sec: f64,
    /// Bytes per second
    #[serde(deserialize_with = "crate::stats::nan_from_null")]
    pub bytes_per_sec: f64,
    /// Total messages sent
    pub total_messages: u64,
    /// Total bytes transferred
    pub total_bytes: u64,
    /// Duration of throughput test
    #[serde(deserialize_with = "crate::stats::nan_from_null")]
    pub duration_secs: f64,
}

/// Determinism/jitter metrics for real-time analysis
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DeterminismMetrics {
    /// Coefficient of variation (std_dev / mean) - lower is better
    #[serde(deserialize_with = "crate::stats::nan_from_null")]
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
    #[serde(deserialize_with = "crate::stats::nan_from_null")]
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

#[cfg(test)]
mod quality_tests {
    use super::*;

    #[test]
    fn isolcpus_spec_covers_the_pinned_cores_only() {
        assert!(parse_isolcpus("2,4-7", &[2]));
        assert!(parse_isolcpus("2,4-7", &[4, 7]));
        assert!(!parse_isolcpus("2,4-7", &[3]), "a gap is not covered");
        assert!(!parse_isolcpus("2,4-7", &[8]));
        assert!(!parse_isolcpus("", &[2]));
        assert!(!parse_isolcpus("2", &[]), "no pinned cores, no claim");
        // Legacy qualifier form: `isolcpus=[domain|managed_irq,]<cpu-list>`.
        assert!(parse_isolcpus("domain,2,4-7", &[2, 5]));
        assert!(parse_isolcpus("managed_irq,3", &[3]));
    }

    #[test]
    fn tail_validity_needs_evidence_and_no_involuntary_switches() {
        assert!(MeasurementQuality {
            rt_granted: true,
            ..Default::default()
        }
        .is_tail_valid());
        assert!(MeasurementQuality {
            isolated_cores: true,
            ..Default::default()
        }
        .is_tail_valid());
        assert!(
            !MeasurementQuality {
                rt_granted: true,
                involuntary_ctx_switches: 1,
                ..Default::default()
            }
            .is_tail_valid(),
            "one observed switch means the window is not quiet"
        );
        assert!(
            !MeasurementQuality::default().is_tail_valid(),
            "no evidence is not evidence of quiet"
        );
        let reason = MeasurementQuality::default().tail_invalid_reason();
        assert!(reason.contains("rt_granted=false"), "{reason}");
        assert!(reason.contains("involuntary_ctx_switches=0"), "{reason}");
    }

    #[test]
    fn json_written_before_this_existed_still_deserializes() {
        // Individual fields missing (the struct-level default).
        let parsed: MeasurementQuality = serde_json::from_str("{}").unwrap();
        assert!(!parsed.is_tail_valid());
        assert_eq!(parsed.involuntary_ctx_switches, 0);

        // The whole block missing from a result (the field-level default).
        let result = serde_json::json!({
            "provenance": {"kind": "measured"},
            "name": "old_bench",
            "subject": "test",
            "message_size": 64,
            "config": {
                "warmup_iterations": 0,
                "iterations": 1,
                "runs": 1,
                "filter_outliers": false,
                "confidence_level": 95.0
            },
            "platform": {
                "cpu": {
                    "model": "test",
                    "physical_cores": 4,
                    "logical_cores": 8,
                    "l1d_cache_kb": null,
                    "l2_cache_kb": null,
                    "l3_cache_kb": null,
                    "base_freq_mhz": null,
                    "measured_freq_mhz": null,
                    "features": []
                },
                "memory_mb": 1,
                "os": "linux",
                "kernel": "x",
                "arch": "x86_64",
                "hostname": "h",
                "virtualized": false,
                "numa_nodes": 1,
                "cpu_governor": null
            },
            "timestamp": "2026-01-01T00:00:00Z",
            "raw_latencies_ns": [],
            "statistics": {
                "count": 0, "mean": 0.0, "median": 0.0, "std_dev": 0.0,
                "min": 0, "max": 0, "p1": 0, "p5": 0, "p25": 0, "p75": 0,
                "p95": 0, "p99": 0, "p999": 0, "p9999": 0, "ci_low": 0.0,
                "ci_high": 0.0, "confidence_level": 95.0, "outliers_removed": 0
            },
            "throughput": {
                "messages_per_sec": 0.0, "bytes_per_sec": 0.0,
                "total_messages": 0, "total_bytes": 0, "duration_secs": 0.0
            },
            "determinism": {
                "cv": 0.0, "max_jitter_ns": 0, "p999": 0, "p9999": 0,
                "deadline_misses": 0, "deadline_threshold_ns": 0, "run_variance": 0.0
            }
        })
        .to_string();
        let parsed: BenchmarkResult = serde_json::from_str(&result).unwrap();
        assert!(
            !parsed.measurement_quality.is_tail_valid(),
            "no evidence must read as not quiet"
        );
    }
}
