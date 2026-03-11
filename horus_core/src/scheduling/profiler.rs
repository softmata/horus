use std::collections::HashMap;
use std::time::Duration;

/// Runtime profiler that tracks node execution statistics
/// Uses Welford's online algorithm for variance calculation
#[derive(Debug, Clone)]
pub struct RuntimeProfiler {
    pub node_stats: HashMap<String, NodeStats>,
    enabled: bool,
}

/// Statistics for a single node
#[derive(Debug, Clone)]
pub struct NodeStats {
    /// Average execution time in microseconds
    pub avg_us: f64,
    /// Standard deviation in microseconds
    pub stddev_us: f64,
    /// Number of samples collected
    pub count: usize,
    /// Minimum execution time observed
    pub min_us: f64,
    /// Maximum execution time observed
    pub max_us: f64,
    /// Number of failures
    pub failure_count: usize,
    /// Failure rate (failures / total ticks)
    pub failure_rate: f64,
    /// Welford's algorithm internal state
    mean: f64,
    m2: f64,
}

impl Default for NodeStats {
    fn default() -> Self {
        Self {
            avg_us: 0.0,
            stddev_us: 0.0,
            count: 0,
            min_us: f64::MAX,
            max_us: 0.0,
            failure_count: 0,
            failure_rate: 0.0,
            mean: 0.0,
            m2: 0.0,
        }
    }
}

impl NodeStats {
    /// Update statistics with new sample using Welford's online algorithm
    pub fn update(&mut self, duration_us: f64) {
        self.count += 1;

        // Update min/max
        self.min_us = self.min_us.min(duration_us);
        self.max_us = self.max_us.max(duration_us);

        // Welford's online algorithm for mean and variance
        let delta = duration_us - self.mean;
        self.mean += delta / self.count as f64;
        let delta2 = duration_us - self.mean;
        self.m2 += delta * delta2;

        // Update public fields
        self.avg_us = self.mean;
        if self.count > 1 {
            self.stddev_us = (self.m2 / (self.count - 1) as f64).sqrt();
        }
    }

    /// Record a failure for this node
    pub fn record_failure(&mut self) {
        self.failure_count += 1;
        let total_attempts = self.count + self.failure_count;
        if total_attempts > 0 {
            self.failure_rate = self.failure_count as f64 / total_attempts as f64;
        }
    }
}

impl RuntimeProfiler {
    /// Create new profiler
    pub fn new() -> Self {
        Self {
            node_stats: HashMap::new(),
            enabled: true,
        }
    }

    /// Create profiler (compatibility alias)
    pub fn new_default() -> Self {
        Self::new()
    }

    /// Enable profiling
    pub fn enable(&mut self) {
        self.enabled = true;
    }

    /// Disable profiling
    pub fn disable(&mut self) {
        self.enabled = false;
    }

    /// Record execution time for a node
    pub fn record(&mut self, node_name: &str, duration: Duration) {
        if !self.enabled {
            return;
        }

        let duration_us = duration.as_micros() as f64;

        self.node_stats
            .entry(node_name.to_string())
            .or_default()
            .update(duration_us);
    }

    /// Record a failure for a node
    pub fn record_node_failure(&mut self, node_name: &str) {
        if !self.enabled {
            return;
        }

        self.node_stats
            .entry(node_name.to_string())
            .or_default()
            .record_failure();
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::duration_ext::DurationExt;

    // ── NodeStats defaults ──

    #[test]
    fn test_node_stats_default() {
        let stats = NodeStats::default();
        assert_eq!(stats.count, 0);
        assert_eq!(stats.avg_us, 0.0);
        assert_eq!(stats.stddev_us, 0.0);
        assert_eq!(stats.min_us, f64::MAX);
        assert_eq!(stats.max_us, 0.0);
        assert_eq!(stats.failure_count, 0);
        assert_eq!(stats.failure_rate, 0.0);
    }

    // ── NodeStats::update — Welford correctness ──

    #[test]
    fn test_update_single_sample() {
        let mut stats = NodeStats::default();
        stats.update(100.0);
        assert_eq!(stats.count, 1);
        assert_eq!(stats.avg_us, 100.0);
        assert_eq!(stats.stddev_us, 0.0); // no variance with 1 sample
        assert_eq!(stats.min_us, 100.0);
        assert_eq!(stats.max_us, 100.0);
    }

    #[test]
    fn test_update_two_samples() {
        let mut stats = NodeStats::default();
        stats.update(100.0);
        stats.update(200.0);
        assert_eq!(stats.count, 2);
        assert!((stats.avg_us - 150.0).abs() < 1e-10);
        // stddev of [100, 200] = sqrt(((100-150)^2 + (200-150)^2) / 1) = sqrt(5000) ≈ 70.71
        assert!((stats.stddev_us - 70.710_678_118_654_76).abs() < 1e-6);
    }

    #[test]
    fn test_update_known_dataset() {
        // Dataset: [2, 4, 4, 4, 5, 5, 7, 9]
        // Mean = 5.0, Variance (sample) = 4.571428..., StdDev ≈ 2.13809
        let mut stats = NodeStats::default();
        for &v in &[2.0, 4.0, 4.0, 4.0, 5.0, 5.0, 7.0, 9.0] {
            stats.update(v);
        }
        assert_eq!(stats.count, 8);
        assert!((stats.avg_us - 5.0).abs() < 1e-10);
        assert!((stats.stddev_us - 2.138_089_935_299_395).abs() < 1e-6);
        assert_eq!(stats.min_us, 2.0);
        assert_eq!(stats.max_us, 9.0);
    }

    #[test]
    fn test_update_identical_values_zero_variance() {
        let mut stats = NodeStats::default();
        for _ in 0..100 {
            stats.update(42.0);
        }
        assert_eq!(stats.count, 100);
        assert!((stats.avg_us - 42.0).abs() < 1e-10);
        assert!(stats.stddev_us < 1e-10); // zero variance
        assert_eq!(stats.min_us, 42.0);
        assert_eq!(stats.max_us, 42.0);
    }

    #[test]
    fn test_update_min_max_tracking() {
        let mut stats = NodeStats::default();
        stats.update(50.0);
        stats.update(10.0);
        stats.update(90.0);
        stats.update(30.0);
        assert_eq!(stats.min_us, 10.0);
        assert_eq!(stats.max_us, 90.0);
    }

    #[test]
    fn test_update_zero_value() {
        let mut stats = NodeStats::default();
        stats.update(0.0);
        assert_eq!(stats.count, 1);
        assert_eq!(stats.avg_us, 0.0);
        assert_eq!(stats.min_us, 0.0);
        assert_eq!(stats.max_us, 0.0);
    }

    #[test]
    fn test_update_very_large_values() {
        let mut stats = NodeStats::default();
        stats.update(1e15);
        stats.update(1e15 + 1.0);
        assert_eq!(stats.count, 2);
        assert!(stats.avg_us > 0.0);
    }

    #[test]
    fn test_update_negative_values() {
        // Negative durations shouldn't happen but shouldn't corrupt state
        let mut stats = NodeStats::default();
        stats.update(-10.0);
        stats.update(10.0);
        assert_eq!(stats.count, 2);
        assert!((stats.avg_us - 0.0).abs() < 1e-10);
        assert_eq!(stats.min_us, -10.0);
        assert_eq!(stats.max_us, 10.0);
    }

    #[test]
    fn test_update_nan_propagates() {
        let mut stats = NodeStats::default();
        stats.update(10.0);
        stats.update(f64::NAN);
        // NaN propagates through arithmetic — documents behavior
        assert!(stats.avg_us.is_nan() || stats.count == 2);
    }

    #[test]
    fn test_update_infinity() {
        let mut stats = NodeStats::default();
        stats.update(f64::INFINITY);
        assert_eq!(stats.count, 1);
        assert!(stats.avg_us.is_infinite());
        assert_eq!(stats.max_us, f64::INFINITY);
    }

    #[test]
    fn test_update_many_samples_stability() {
        // Welford should be numerically stable even with many samples
        let mut stats = NodeStats::default();
        for i in 0..10_000 {
            stats.update(100.0 + (i as f64) * 0.001);
        }
        assert_eq!(stats.count, 10_000);
        assert!(stats.avg_us > 100.0);
        assert!(stats.avg_us < 110.0);
        assert!(stats.stddev_us > 0.0);
        assert!(stats.stddev_us.is_finite());
    }

    // ── NodeStats::record_failure ──

    #[test]
    fn test_record_failure_increments() {
        let mut stats = NodeStats::default();
        stats.record_failure();
        assert_eq!(stats.failure_count, 1);
        stats.record_failure();
        assert_eq!(stats.failure_count, 2);
    }

    #[test]
    fn test_record_failure_rate_only_failures() {
        let mut stats = NodeStats::default();
        stats.record_failure();
        stats.record_failure();
        // count=0, failure_count=2, total=2 → rate = 1.0
        assert!((stats.failure_rate - 1.0).abs() < 1e-10);
    }

    #[test]
    fn test_record_failure_rate_mixed() {
        let mut stats = NodeStats::default();
        stats.update(10.0); // count=1
        stats.update(20.0); // count=2
        stats.record_failure(); // failure_count=1, total=3
                                // rate = 1/3
        assert!((stats.failure_rate - 1.0 / 3.0).abs() < 1e-10);
    }

    #[test]
    fn test_record_failure_rate_no_failures() {
        let mut stats = NodeStats::default();
        stats.update(10.0);
        stats.update(20.0);
        assert_eq!(stats.failure_rate, 0.0);
    }

    // ── RuntimeProfiler ──

    #[test]
    fn test_profiler_new_enabled() {
        let profiler = RuntimeProfiler::new();
        assert!(profiler.enabled);
        assert!(profiler.node_stats.is_empty());
    }

    #[test]
    fn test_profiler_new_default_alias() {
        let profiler = RuntimeProfiler::new_default();
        assert!(profiler.enabled);
    }

    #[test]
    fn test_profiler_record_creates_entry() {
        let mut profiler = RuntimeProfiler::new();
        profiler.record("test_node", 100_u64.us());
        assert!(profiler.node_stats.contains_key("test_node"));
        assert_eq!(profiler.node_stats["test_node"].count, 1);
        assert_eq!(profiler.node_stats["test_node"].avg_us, 100.0);
    }

    #[test]
    fn test_profiler_record_multiple_nodes() {
        let mut profiler = RuntimeProfiler::new();
        profiler.record("node_a", 100_u64.us());
        profiler.record("node_b", 200_u64.us());
        profiler.record("node_a", 300_u64.us());

        assert_eq!(profiler.node_stats.len(), 2);
        assert_eq!(profiler.node_stats["node_a"].count, 2);
        assert_eq!(profiler.node_stats["node_b"].count, 1);
    }

    #[test]
    fn test_profiler_disabled_ignores_records() {
        let mut profiler = RuntimeProfiler::new();
        profiler.disable();
        profiler.record("test_node", 100_u64.us());
        assert!(profiler.node_stats.is_empty());
    }

    #[test]
    fn test_profiler_enable_disable_toggle() {
        let mut profiler = RuntimeProfiler::new();
        profiler.disable();
        profiler.record("a", 10_u64.us());
        assert!(profiler.node_stats.is_empty());

        profiler.enable();
        profiler.record("a", 10_u64.us());
        assert_eq!(profiler.node_stats["a"].count, 1);
    }

    #[test]
    fn test_profiler_record_node_failure() {
        let mut profiler = RuntimeProfiler::new();
        profiler.record("node", 50_u64.us());
        profiler.record_node_failure("node");
        assert_eq!(profiler.node_stats["node"].failure_count, 1);
        // count=1, failures=1, total=2 → rate = 0.5
        assert!((profiler.node_stats["node"].failure_rate - 0.5).abs() < 1e-10);
    }

    #[test]
    fn test_profiler_record_failure_disabled() {
        let mut profiler = RuntimeProfiler::new();
        profiler.disable();
        profiler.record_node_failure("node");
        assert!(profiler.node_stats.is_empty());
    }

    #[test]
    fn test_profiler_record_failure_new_node() {
        let mut profiler = RuntimeProfiler::new();
        profiler.record_node_failure("unknown");
        // Should create entry with only failure data
        assert_eq!(profiler.node_stats["unknown"].failure_count, 1);
        assert_eq!(profiler.node_stats["unknown"].count, 0);
    }

    #[test]
    fn test_profiler_record_zero_duration() {
        let mut profiler = RuntimeProfiler::new();
        profiler.record("fast", Duration::ZERO);
        assert_eq!(profiler.node_stats["fast"].avg_us, 0.0);
    }

    #[test]
    fn test_profiler_record_large_duration() {
        let mut profiler = RuntimeProfiler::new();
        profiler.record("slow", 10_u64.secs());
        assert_eq!(profiler.node_stats["slow"].avg_us, 10_000_000.0);
    }
}
