//! Benchmark output formats for CI integration and analysis.
//!
//! Supports:
//! - JSON output for machine parsing and regression detection
//! - CSV output for spreadsheet analysis
//! - Comparison against baselines

use crate::{BenchmarkResult, PlatformInfo};
use serde::{Deserialize, Serialize};
use std::fs::{self, File};
use std::io::{BufWriter, Write};
use std::path::Path;

/// Complete benchmark report containing multiple results
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct BenchmarkReport {
    /// Report version for forward compatibility
    pub version: String,
    /// Timestamp when report was generated
    pub generated_at: String,
    /// Platform the benchmarks ran on
    pub platform: PlatformInfo,
    /// Individual benchmark results
    pub results: Vec<BenchmarkResult>,
    /// Git commit hash (if available)
    pub git_commit: Option<String>,
    /// Git branch (if available)
    pub git_branch: Option<String>,
    /// CI run ID (if available)
    pub ci_run_id: Option<String>,
}

impl BenchmarkReport {
    /// Create a new report with current timestamp
    pub fn new(platform: PlatformInfo) -> Self {
        Self {
            version: "1.0.0".to_string(),
            generated_at: chrono::Utc::now().to_rfc3339(),
            platform,
            results: Vec::new(),
            git_commit: detect_git_commit(),
            git_branch: detect_git_branch(),
            ci_run_id: std::env::var("CI_RUN_ID")
                .or_else(|_| std::env::var("GITHUB_RUN_ID"))
                .ok(),
        }
    }

    /// Add a benchmark result
    pub fn add_result(&mut self, result: BenchmarkResult) {
        self.results.push(result);
    }

    /// Compare against a baseline report
    pub fn compare(&self, baseline: &BenchmarkReport) -> ComparisonReport {
        let mut comparisons = Vec::new();

        for result in &self.results {
            if let Some(baseline_result) = baseline
                .results
                .iter()
                .find(|r| r.name == result.name && r.message_size == result.message_size)
            {
                comparisons.push(ResultComparison {
                    name: result.name.clone(),
                    message_size: result.message_size,
                    current_median_ns: result.statistics.median,
                    baseline_median_ns: baseline_result.statistics.median,
                    change_percent: ((result.statistics.median - baseline_result.statistics.median)
                        / baseline_result.statistics.median)
                        * 100.0,
                    current_p99_ns: result.statistics.p99,
                    baseline_p99_ns: baseline_result.statistics.p99,
                    p99_change_percent: ((result.statistics.p99 as f64
                        - baseline_result.statistics.p99 as f64)
                        / baseline_result.statistics.p99 as f64)
                        * 100.0,
                    regression: result.statistics.median
                        > baseline_result.statistics.median * 1.05, // 5% threshold
                });
            }
        }

        ComparisonReport {
            current_commit: self.git_commit.clone(),
            baseline_commit: baseline.git_commit.clone(),
            comparisons,
        }
    }
}

/// Comparison between current and baseline results
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ComparisonReport {
    /// Current git commit
    pub current_commit: Option<String>,
    /// Baseline git commit
    pub baseline_commit: Option<String>,
    /// Individual result comparisons
    pub comparisons: Vec<ResultComparison>,
}

impl ComparisonReport {
    /// Check if any benchmarks regressed
    pub fn has_regressions(&self) -> bool {
        self.comparisons.iter().any(|c| c.regression)
    }

    /// Get list of regressed benchmarks
    pub fn regressions(&self) -> Vec<&ResultComparison> {
        self.comparisons.iter().filter(|c| c.regression).collect()
    }

    /// Print summary to stdout
    pub fn print_summary(&self) {
        println!("\n=== Benchmark Comparison ===\n");

        if let (Some(current), Some(baseline)) = (&self.current_commit, &self.baseline_commit) {
            println!("Current:  {}", current);
            println!("Baseline: {}", baseline);
            println!();
        }

        for comp in &self.comparisons {
            let status = if comp.regression { "REGRESSED" } else { "OK" };
            let sign = if comp.change_percent >= 0.0 { "+" } else { "" };

            println!(
                "{:40} {:>10} median: {:>8.1}ns -> {:>8.1}ns ({}{:.1}%)",
                comp.name,
                format!("[{}]", status),
                comp.baseline_median_ns,
                comp.current_median_ns,
                sign,
                comp.change_percent
            );
        }

        println!();
        if self.has_regressions() {
            println!(
                "WARNING: {} benchmark(s) regressed!",
                self.regressions().len()
            );
        } else {
            println!("All benchmarks within acceptable range.");
        }
    }
}

/// Comparison of a single benchmark result
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ResultComparison {
    /// Benchmark name
    pub name: String,
    /// Message size tested
    pub message_size: usize,
    /// Current median latency (ns)
    pub current_median_ns: f64,
    /// Baseline median latency (ns)
    pub baseline_median_ns: f64,
    /// Percent change in median (positive = slower)
    pub change_percent: f64,
    /// Current p99 latency (ns)
    pub current_p99_ns: u64,
    /// Baseline p99 latency (ns)
    pub baseline_p99_ns: u64,
    /// Percent change in p99
    pub p99_change_percent: f64,
    /// Whether this is considered a regression
    pub regression: bool,
}

/// Write benchmark report as JSON
pub fn write_json_report<P: AsRef<Path>>(
    report: &BenchmarkReport,
    path: P,
) -> std::io::Result<()> {
    let file = File::create(path)?;
    let writer = BufWriter::new(file);
    serde_json::to_writer_pretty(writer, report)?;
    Ok(())
}

/// Write benchmark results as CSV for spreadsheet analysis
pub fn write_csv_report<P: AsRef<Path>>(
    report: &BenchmarkReport,
    path: P,
) -> std::io::Result<()> {
    let file = File::create(path)?;
    let mut writer = BufWriter::new(file);

    // Header
    writeln!(
        writer,
        "name,subject,message_size,mean_ns,median_ns,std_dev_ns,min_ns,max_ns,p95_ns,p99_ns,p999_ns,ci_low_ns,ci_high_ns,throughput_msg_sec,cv,deadline_misses"
    )?;

    // Data rows
    for result in &report.results {
        writeln!(
            writer,
            "{},{},{},{:.1},{:.1},{:.1},{},{},{},{},{},{:.1},{:.1},{:.1},{:.4},{}",
            result.name,
            result.subject,
            result.message_size,
            result.statistics.mean,
            result.statistics.median,
            result.statistics.std_dev,
            result.statistics.min,
            result.statistics.max,
            result.statistics.p95,
            result.statistics.p99,
            result.statistics.p999,
            result.statistics.ci_low,
            result.statistics.ci_high,
            result.throughput.messages_per_sec,
            result.determinism.cv,
            result.determinism.deadline_misses,
        )?;
    }

    Ok(())
}

/// Load a baseline report from JSON
pub fn load_baseline<P: AsRef<Path>>(path: P) -> std::io::Result<BenchmarkReport> {
    let content = fs::read_to_string(path)?;
    let report: BenchmarkReport = serde_json::from_str(&content)?;
    Ok(report)
}

/// Detect current git commit hash
fn detect_git_commit() -> Option<String> {
    std::process::Command::new("git")
        .args(["rev-parse", "HEAD"])
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

/// Detect current git branch
fn detect_git_branch() -> Option<String> {
    std::process::Command::new("git")
        .args(["rev-parse", "--abbrev-ref", "HEAD"])
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

/// Format duration in human-readable form
pub fn format_duration_ns(ns: f64) -> String {
    if ns < 1000.0 {
        format!("{:.1} ns", ns)
    } else if ns < 1_000_000.0 {
        format!("{:.2} µs", ns / 1000.0)
    } else if ns < 1_000_000_000.0 {
        format!("{:.2} ms", ns / 1_000_000.0)
    } else {
        format!("{:.2} s", ns / 1_000_000_000.0)
    }
}

/// Format throughput in human-readable form
pub fn format_throughput(msgs_per_sec: f64) -> String {
    if msgs_per_sec < 1000.0 {
        format!("{:.1} msg/s", msgs_per_sec)
    } else if msgs_per_sec < 1_000_000.0 {
        format!("{:.2} K msg/s", msgs_per_sec / 1000.0)
    } else {
        format!("{:.2} M msg/s", msgs_per_sec / 1_000_000.0)
    }
}

/// Format bytes in human-readable form
pub fn format_bytes(bytes: f64) -> String {
    if bytes < 1024.0 {
        format!("{:.0} B", bytes)
    } else if bytes < 1024.0 * 1024.0 {
        format!("{:.2} KB", bytes / 1024.0)
    } else if bytes < 1024.0 * 1024.0 * 1024.0 {
        format!("{:.2} MB", bytes / (1024.0 * 1024.0))
    } else {
        format!("{:.2} GB", bytes / (1024.0 * 1024.0 * 1024.0))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        detect_platform, BenchmarkConfig, BenchmarkResult, DeterminismMetrics, Statistics,
        ThroughputMetrics,
    };

    fn make_test_result(name: &str, median: f64) -> BenchmarkResult {
        BenchmarkResult {
            name: name.to_string(),
            subject: "test".to_string(),
            message_size: 64,
            config: BenchmarkConfig::default(),
            platform: detect_platform(),
            timestamp: "2024-01-01T00:00:00Z".to_string(),
            raw_latencies_ns: vec![100, 110, 120],
            statistics: Statistics {
                count: 3,
                mean: median,
                median,
                std_dev: 10.0,
                min: 100,
                max: 120,
                p1: 100,
                p5: 100,
                p25: 105,
                p75: 115,
                p95: 120,
                p99: 120,
                p999: 120,
                p9999: 120,
                ci_low: median - 5.0,
                ci_high: median + 5.0,
                confidence_level: 95.0,
                outliers_removed: 0,
            },
            throughput: ThroughputMetrics {
                messages_per_sec: 1_000_000.0,
                bytes_per_sec: 64_000_000.0,
                total_messages: 100_000,
                total_bytes: 6_400_000,
                duration_secs: 0.1,
            },
            determinism: DeterminismMetrics {
                cv: 0.1,
                max_jitter_ns: 20,
                p999: 120,
                p9999: 120,
                deadline_misses: 0,
                deadline_threshold_ns: 1000,
                run_variance: 0.05,
            },
        }
    }

    #[test]
    fn test_comparison_no_regression() {
        let platform = detect_platform();
        let mut current = BenchmarkReport::new(platform.clone());
        let mut baseline = BenchmarkReport::new(platform);

        current.add_result(make_test_result("test_bench", 100.0));
        baseline.add_result(make_test_result("test_bench", 100.0));

        let comparison = current.compare(&baseline);
        assert!(!comparison.has_regressions());
    }

    #[test]
    fn test_comparison_with_regression() {
        let platform = detect_platform();
        let mut current = BenchmarkReport::new(platform.clone());
        let mut baseline = BenchmarkReport::new(platform);

        current.add_result(make_test_result("test_bench", 200.0)); // Much slower
        baseline.add_result(make_test_result("test_bench", 100.0));

        let comparison = current.compare(&baseline);
        assert!(comparison.has_regressions());
        assert_eq!(comparison.regressions().len(), 1);
    }

    #[test]
    fn test_format_duration() {
        assert_eq!(format_duration_ns(500.0), "500.0 ns");
        assert_eq!(format_duration_ns(1500.0), "1.50 µs");
        assert_eq!(format_duration_ns(1_500_000.0), "1.50 ms");
    }

    #[test]
    fn test_format_throughput() {
        assert_eq!(format_throughput(500.0), "500.0 msg/s");
        assert_eq!(format_throughput(50_000.0), "50.00 K msg/s");
        assert_eq!(format_throughput(5_000_000.0), "5.00 M msg/s");
    }
}
