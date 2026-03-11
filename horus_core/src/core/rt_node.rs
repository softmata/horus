// Real-time types for time-critical applications
use std::time::Duration;

/// What to do when a node misses its deadline.
///
/// Set via `.on_miss()` on the node builder.
///
/// # Examples
///
/// ```rust,ignore
/// use horus::prelude::*;
///
/// // Video encoder: drop frame, keep streaming
/// scheduler.add(encoder).rate(30.hz()).on_miss(Miss::Skip).build()?;
///
/// // Motor controller: degrade to safe mode
/// scheduler.add(motor).rate(1000.hz()).on_miss(Miss::SafeMode).build()?;
/// ```
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Miss {
    /// Log warning and continue normally.
    #[default]
    Warn,
    /// Skip this tick, resume next cycle.
    Skip,
    /// Call `enter_safe_state()` on the node, continue ticking in degraded mode.
    /// The scheduler checks `is_safe_state()` each tick for recovery.
    SafeMode,
    /// Stop the entire scheduler (last resort).
    Stop,
}

/// Policy for handling deadline misses.
///
/// Configure via `.on_miss()` on the node builder, which accepts [`Miss`].
/// This enum is the internal representation; prefer `Miss` in new code.
///
/// # Example
///
/// ```rust,ignore
/// use horus::prelude::*;
///
/// scheduler.add(encoder)
///     .rate(30.hz())
///     .on_miss(Miss::Skip)  // Drop frame, keep streaming
///     .build()?;
/// ```
#[doc(hidden)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DeadlineMissPolicy {
    /// Log warning and continue
    Warn,
    /// Skip the node for this tick
    Skip,
    /// Emergency stop - trigger safety shutdown
    EmergencyStop,
}

// ── Conversions between Miss and DeadlineMissPolicy ──

impl From<Miss> for DeadlineMissPolicy {
    fn from(miss: Miss) -> Self {
        match miss {
            Miss::Warn => DeadlineMissPolicy::Warn,
            Miss::Skip => DeadlineMissPolicy::Skip,
            // SafeMode has no direct equivalent in old API — map to Warn
            // (the executor handles SafeMode separately via Miss)
            Miss::SafeMode => DeadlineMissPolicy::Warn,
            Miss::Stop => DeadlineMissPolicy::EmergencyStop,
        }
    }
}

impl From<DeadlineMissPolicy> for Miss {
    fn from(policy: DeadlineMissPolicy) -> Self {
        match policy {
            DeadlineMissPolicy::Warn => Miss::Warn,
            DeadlineMissPolicy::Skip => Miss::Skip,
            DeadlineMissPolicy::EmergencyStop => Miss::Stop,
        }
    }
}

/// Tick budget violation — node exceeded its allowed execution time.
#[derive(Debug, Clone)]
pub struct BudgetViolation {
    node_name: String,
    budget: Duration,
    actual: Duration,
    overrun: Duration,
}

impl BudgetViolation {
    /// Create a new budget violation. Overrun is computed automatically.
    pub fn new(node_name: String, budget: Duration, actual: Duration) -> Self {
        Self {
            node_name,
            budget,
            actual,
            overrun: actual.saturating_sub(budget),
        }
    }

    /// Node that violated its budget.
    pub fn node_name(&self) -> &str {
        &self.node_name
    }

    /// The configured tick budget.
    pub fn budget(&self) -> Duration {
        self.budget
    }

    /// Actual execution time.
    pub fn actual(&self) -> Duration {
        self.actual
    }

    /// How much the budget was exceeded by.
    pub fn overrun(&self) -> Duration {
        self.overrun
    }
}

/// Real-time statistics for a node.
///
/// Tracks execution time metrics using exponential moving average (EMA).
///
/// # Example
///
/// ```rust
/// use horus_core::core::RtStats;
/// use std::time::Duration;
///
/// let mut stats = RtStats::default();
///
/// // Record execution times
/// stats.record_execution(Duration::from_micros(95));
/// stats.record_execution(Duration::from_micros(105));
/// stats.record_execution(Duration::from_micros(100));
///
/// assert_eq!(stats.total_ticks, 3);
/// assert_eq!(stats.worst_execution, Duration::from_micros(105));
/// println!("{}", stats.summary());
/// ```
#[derive(Debug, Clone, Default)]
pub struct RtStats {
    deadline_misses: u64,
    budget_violations: u64,
    worst_execution: Duration,
    last_execution: Duration,
    jitter_us: f64,
    avg_execution_us: f64,
    total_ticks: u64,
}

impl RtStats {
    /// Number of deadline misses.
    pub fn deadline_misses(&self) -> u64 {
        self.deadline_misses
    }

    /// Number of budget violations.
    pub fn budget_violations(&self) -> u64 {
        self.budget_violations
    }

    /// Worst observed execution time.
    pub fn worst_execution(&self) -> Duration {
        self.worst_execution
    }

    /// Last execution time.
    pub fn last_execution(&self) -> Duration {
        self.last_execution
    }

    /// Jitter (variance in execution time) in microseconds.
    pub fn jitter_us(&self) -> f64 {
        self.jitter_us
    }

    /// Average execution time in microseconds.
    pub fn avg_execution_us(&self) -> f64 {
        self.avg_execution_us
    }

    /// Total ticks recorded.
    pub fn total_ticks(&self) -> u64 {
        self.total_ticks
    }

    /// Record a budget violation.
    pub fn record_budget_violation(&mut self) {
        self.budget_violations += 1;
    }

    /// Update statistics with new execution time
    pub fn record_execution(&mut self, duration: Duration) {
        let duration_us = duration.as_micros() as f64;

        // Update worst case
        if duration > self.worst_execution {
            self.worst_execution = duration;
        }

        // Update last
        self.last_execution = duration;

        // Increment tick count
        self.total_ticks += 1;

        // Calculate moving average (EMA with alpha=0.1)
        if self.total_ticks == 1 {
            self.avg_execution_us = duration_us;
            self.jitter_us = 0.0;
        } else {
            let alpha = 0.1;
            let prev_avg = self.avg_execution_us;
            self.avg_execution_us = (alpha * duration_us) + ((1.0 - alpha) * prev_avg);

            // Calculate jitter as absolute deviation from average
            let deviation = (duration_us - self.avg_execution_us).abs();
            self.jitter_us = (alpha * deviation) + ((1.0 - alpha) * self.jitter_us);
        }
    }

    /// Record a deadline miss
    pub fn record_deadline_miss(&mut self) {
        self.deadline_misses += 1;
    }

    /// Get human-readable statistics
    pub fn summary(&self) -> String {
        format!(
            "Ticks: {}, Worst: {:.1}μs, Avg: {:.1}μs, Jitter: {:.1}μs, Deadline misses: {}, Budget violations: {}",
            self.total_ticks,
            self.worst_execution.as_micros(),
            self.avg_execution_us,
            self.jitter_us,
            self.deadline_misses,
            self.budget_violations
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::node::Node;

    // =========================================================================
    // Test helper: minimal RtNode implementation
    // =========================================================================

    /// Minimal node for testing trait defaults.
    struct TestMotorNode;

    impl Node for TestMotorNode {
        fn tick(&mut self) {}
    }

    // =========================================================================
    // Section 1: Node trait default tests
    // =========================================================================

    /// Default safe state: node reports safe, enter_safe_state is a no-op.
    #[test]
    fn default_safe_state() {
        let mut node = TestMotorNode;
        assert!(node.is_safe_state());
        node.enter_safe_state(); // Should not panic
    }

    // =========================================================================
    // RtStats EMA calculations
    // =========================================================================

    /// First recording sets avg to the duration, jitter to 0.
    /// Robotics: first tick establishes baseline — no jitter measured yet.
    #[test]
    fn rtstats_first_recording_sets_baseline() {
        let mut stats = RtStats::default();
        stats.record_execution(Duration::from_micros(100));

        assert_eq!(stats.total_ticks(), 1);
        assert!((stats.avg_execution_us() - 100.0).abs() < 1e-6);
        assert!((stats.jitter_us() - 0.0).abs() < 1e-6);
        assert_eq!(stats.worst_execution(), Duration::from_micros(100));
        assert_eq!(stats.last_execution(), Duration::from_micros(100));
    }

    /// EMA with alpha=0.1 over a known 10-element sequence.
    /// Verify avg and jitter match hand-calculated expected values.
    ///
    /// Robotics: motor controller tick times: [100, 110, 95, 120, 100, 105, 90, 115, 100, 108] µs
    #[test]
    fn rtstats_ema_10_element_sequence() {
        let mut stats = RtStats::default();
        let durations_us: Vec<u64> = vec![100, 110, 95, 120, 100, 105, 90, 115, 100, 108];

        let alpha = 0.1;
        let mut expected_avg = 0.0_f64;
        let mut expected_jitter = 0.0_f64;

        for (i, &dur_us) in durations_us.iter().enumerate() {
            let d = dur_us as f64;
            stats.record_execution(Duration::from_micros(dur_us));

            if i == 0 {
                expected_avg = d;
                expected_jitter = 0.0;
            } else {
                let new_avg = alpha * d + (1.0 - alpha) * expected_avg;
                let deviation = (d - new_avg).abs();
                expected_jitter = alpha * deviation + (1.0 - alpha) * expected_jitter;
                expected_avg = new_avg;
            }
        }

        assert!(
            (stats.avg_execution_us() - expected_avg).abs() < 1e-6,
            "EMA avg mismatch: got {}, expected {}",
            stats.avg_execution_us(),
            expected_avg
        );
        assert!(
            (stats.jitter_us() - expected_jitter).abs() < 1e-6,
            "Jitter mismatch: got {}, expected {}",
            stats.jitter_us(),
            expected_jitter
        );
        assert_eq!(stats.total_ticks(), 10);
    }

    /// Constant execution time → jitter converges to zero.
    /// Robotics: perfectly stable node (e.g., pure computation with no I/O).
    #[test]
    fn rtstats_constant_duration_jitter_converges_to_zero() {
        let mut stats = RtStats::default();

        // 50 ticks of exactly 200µs
        for _ in 0..50 {
            stats.record_execution(Duration::from_micros(200));
        }

        assert!(
            (stats.avg_execution_us() - 200.0).abs() < 1e-6,
            "Avg should converge to constant: {}",
            stats.avg_execution_us()
        );
        assert!(
            stats.jitter_us() < 0.01,
            "Jitter should be near zero for constant input: {}",
            stats.jitter_us()
        );
    }

    /// Worst-case tracking always keeps the absolute maximum.
    /// Robotics: budget monitoring — one spike sets the record permanently.
    #[test]
    fn rtstats_worst_case_tracks_maximum() {
        let mut stats = RtStats::default();

        stats.record_execution(Duration::from_micros(100));
        assert_eq!(stats.worst_execution(), Duration::from_micros(100));

        stats.record_execution(Duration::from_micros(50));
        assert_eq!(
            stats.worst_execution(),
            Duration::from_micros(100),
            "Worst should not decrease"
        );

        stats.record_execution(Duration::from_micros(500));
        assert_eq!(stats.worst_execution(), Duration::from_micros(500));

        stats.record_execution(Duration::from_micros(200));
        assert_eq!(
            stats.worst_execution(),
            Duration::from_micros(500),
            "Worst should stay at 500"
        );

        // Record many smaller values — worst must remain 500
        for _ in 0..100 {
            stats.record_execution(Duration::from_micros(100));
        }
        assert_eq!(stats.worst_execution(), Duration::from_micros(500));
    }

    /// Last execution always tracks the most recent value.
    #[test]
    fn rtstats_last_execution_tracks_most_recent() {
        let mut stats = RtStats::default();

        stats.record_execution(Duration::from_micros(100));
        assert_eq!(stats.last_execution(), Duration::from_micros(100));

        stats.record_execution(Duration::from_micros(250));
        assert_eq!(stats.last_execution(), Duration::from_micros(250));

        stats.record_execution(Duration::from_micros(50));
        assert_eq!(stats.last_execution(), Duration::from_micros(50));
    }

    /// Deadline miss counter increments correctly.
    #[test]
    fn rtstats_deadline_miss_counter() {
        let mut stats = RtStats::default();
        assert_eq!(stats.deadline_misses(), 0);

        stats.record_deadline_miss();
        assert_eq!(stats.deadline_misses(), 1);

        stats.record_deadline_miss();
        stats.record_deadline_miss();
        assert_eq!(stats.deadline_misses(), 3);
    }

    /// Summary string includes all key metrics.
    #[test]
    fn rtstats_summary_format() {
        let mut stats = RtStats::default();
        stats.record_execution(Duration::from_micros(100));
        stats.record_deadline_miss();

        let summary = stats.summary();
        assert!(summary.contains("Ticks: 1"));
        assert!(summary.contains("Deadline misses: 1"));
    }

    /// EMA responds to a step change — avg moves toward new value.
    /// Robotics: sensor switches from low-latency to high-latency mode.
    #[test]
    fn rtstats_ema_responds_to_step_change() {
        let mut stats = RtStats::default();

        // 20 ticks at 100µs — avg converges to 100
        for _ in 0..20 {
            stats.record_execution(Duration::from_micros(100));
        }
        let avg_before = stats.avg_execution_us();
        assert!((avg_before - 100.0).abs() < 1.0);

        // Step to 500µs — avg should increase
        for _ in 0..20 {
            stats.record_execution(Duration::from_micros(500));
        }
        assert!(
            stats.avg_execution_us() > avg_before + 50.0,
            "EMA should respond to step change: avg={}, was={}",
            stats.avg_execution_us(),
            avg_before
        );
    }

    // =========================================================================
    // Section 4: BudgetViolation struct
    // =========================================================================

    /// BudgetViolation stores correct values and auto-computes overrun.
    #[test]
    fn budget_violation_fields() {
        let v = BudgetViolation::new(
            "motor_ctrl".to_string(),
            Duration::from_micros(100),
            Duration::from_micros(250),
        );
        assert_eq!(v.node_name(), "motor_ctrl");
        assert_eq!(v.budget(), Duration::from_micros(100));
        assert_eq!(v.actual(), Duration::from_micros(250));
        assert_eq!(v.overrun(), Duration::from_micros(150));
    }

    // =========================================================================
    // Section 5: Miss enum
    // =========================================================================

    #[test]
    fn miss_default_is_warn() {
        assert_eq!(Miss::default(), Miss::Warn);
    }

    #[test]
    fn miss_to_deadline_miss_policy() {
        assert_eq!(
            DeadlineMissPolicy::from(Miss::Warn),
            DeadlineMissPolicy::Warn
        );
        assert_eq!(
            DeadlineMissPolicy::from(Miss::Skip),
            DeadlineMissPolicy::Skip
        );
        assert_eq!(
            DeadlineMissPolicy::from(Miss::SafeMode),
            DeadlineMissPolicy::Warn
        );
        assert_eq!(
            DeadlineMissPolicy::from(Miss::Stop),
            DeadlineMissPolicy::EmergencyStop
        );
    }

    #[test]
    fn deadline_miss_policy_to_miss() {
        assert_eq!(Miss::from(DeadlineMissPolicy::Warn), Miss::Warn);
        assert_eq!(Miss::from(DeadlineMissPolicy::Skip), Miss::Skip);
        assert_eq!(Miss::from(DeadlineMissPolicy::EmergencyStop), Miss::Stop);
    }

    #[test]
    fn miss_roundtrip_preserves_identity() {
        // Warn and Skip roundtrip cleanly
        assert_eq!(Miss::from(DeadlineMissPolicy::from(Miss::Warn)), Miss::Warn);
        assert_eq!(Miss::from(DeadlineMissPolicy::from(Miss::Skip)), Miss::Skip);
        assert_eq!(Miss::from(DeadlineMissPolicy::from(Miss::Stop)), Miss::Stop);
        // SafeMode maps to Warn in old API (no equivalent), so roundtrip loses info
        assert_eq!(
            Miss::from(DeadlineMissPolicy::from(Miss::SafeMode)),
            Miss::Warn
        );
    }
}
