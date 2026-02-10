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
