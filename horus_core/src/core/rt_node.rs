// Real-time node trait for time-critical applications
use super::Node;
use std::time::Duration;

/// Priority levels for real-time scheduling
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum RtPriority {
    /// Highest priority - critical control loops
    Critical,
    /// High priority - important sensors
    High,
    /// Medium priority - normal processing
    Medium,
    /// Low priority - background tasks
    Low,
    /// Custom priority value
    Custom(u32),
}

impl RtPriority {
    pub fn value(&self) -> u32 {
        match self {
            RtPriority::Critical => 0,
            RtPriority::High => 10,
            RtPriority::Medium => 50,
            RtPriority::Low => 100,
            RtPriority::Custom(v) => *v,
        }
    }
}

/// Real-time class for deadline handling
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RtClass {
    /// Must never miss deadline (safety-critical, surgical robots)
    Hard,
    /// Occasional miss tolerated (video streaming, VR)
    Firm,
    /// Best effort timing (gaming, UI, monitoring)
    Soft,
}

/// Deadline miss policy
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DeadlineMissPolicy {
    /// Log warning and continue
    Warn,
    /// Skip the node for this tick
    Skip,
    /// Emergency stop - trigger safety shutdown
    EmergencyStop,
    /// Downgrade priority for future ticks
    Degrade,
    /// Switch to fallback node
    Fallback,
}

/// WCET (Worst-Case Execution Time) violation handling
#[derive(Debug, Clone)]
pub struct WCETViolation {
    pub node_name: String,
    pub budget: Duration,
    pub actual: Duration,
    pub overrun: Duration,
}

/// Real-time statistics for a node
#[derive(Debug, Clone, Default)]
pub struct RtStats {
    /// Number of deadline misses
    pub deadline_misses: u64,
    /// Number of WCET violations
    pub wcet_violations: u64,
    /// Worst observed execution time
    pub worst_execution: Duration,
    /// Last execution time
    pub last_execution: Duration,
    /// Jitter (variance in execution time) in microseconds
    pub jitter_us: f64,
    /// Average execution time (for jitter calculation)
    pub avg_execution_us: f64,
    /// Total ticks (for statistics)
    pub total_ticks: u64,
}

impl RtStats {
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
            "Ticks: {}, Worst: {:.1}μs, Avg: {:.1}μs, Jitter: {:.1}μs, Deadline misses: {}, WCET violations: {}",
            self.total_ticks,
            self.worst_execution.as_micros(),
            self.avg_execution_us,
            self.jitter_us,
            self.deadline_misses,
            self.wcet_violations
        )
    }
}

/// Real-time node trait for time-critical applications
///
/// This trait extends the base Node trait with real-time guarantees.
/// Implementing this trait allows the scheduler to provide:
/// - WCET budget enforcement
/// - Deadline monitoring
/// - Priority-based preemption
/// - Formal verification support
///
/// # Example
/// ```ignore
/// impl RtNode for MotorControlNode {
///     fn wcet_budget(&self) -> Duration {
///         Duration::from_micros(100) // 100μs max execution
///     }
///
///     fn deadline(&self) -> Duration {
///         Duration::from_millis(1) // 1ms deadline for 1kHz control
///     }
/// }
/// ```
pub trait RtNode: Node {
    /// Worst-case execution time budget
    fn wcet_budget(&self) -> Duration;

    /// Deadline for completion (from start of tick)
    fn deadline(&self) -> Duration {
        self.wcet_budget() * 2 // Default: 2x WCET
    }

    /// Real-time priority (lower value = higher priority)
    fn rt_priority(&self) -> RtPriority {
        RtPriority::Medium
    }

    /// Real-time class (Hard/Firm/Soft)
    fn rt_class(&self) -> RtClass {
        RtClass::Soft
    }

    /// What to do if deadline is missed
    fn deadline_miss_policy(&self) -> DeadlineMissPolicy {
        match self.rt_class() {
            RtClass::Hard => DeadlineMissPolicy::EmergencyStop,
            RtClass::Firm => DeadlineMissPolicy::Skip,
            RtClass::Soft => DeadlineMissPolicy::Warn,
        }
    }

    /// Pre-condition that must be true before tick (for formal verification)
    fn pre_condition(&self) -> bool {
        true // Default: no precondition
    }

    /// Post-condition that must be true after tick (for formal verification)
    fn post_condition(&self) -> bool {
        true // Default: no postcondition
    }

    /// System invariant that must always be true (for formal verification)
    fn invariant(&self) -> bool {
        true // Default: no invariant
    }

    /// Called when WCET budget is exceeded
    fn on_wcet_violation(&mut self, violation: &WCETViolation) {
        // Default: log error
        eprintln!(
            "WCET violation in {}: budget={:?}, actual={:?}, overrun={:?}",
            violation.node_name, violation.budget, violation.actual, violation.overrun
        );
    }

    /// Called when deadline is missed
    fn on_deadline_miss(&mut self, elapsed: Duration, deadline: Duration) {
        // Default: log error
        eprintln!(
            "Deadline miss in {}: deadline={:?}, elapsed={:?}",
            self.name(),
            deadline,
            elapsed
        );
    }

    /// Get fallback node for redundancy (N-version programming)
    fn fallback_node(&self) -> Option<Box<dyn RtNode>> {
        None // Default: no fallback
    }

    /// Check if node is in safe state (for safety monitor)
    fn is_safe_state(&self) -> bool {
        true // Default: assume safe
    }

    /// Transition to safe state (for emergency stop)
    fn enter_safe_state(&mut self) {
        // Default: do nothing (already safe)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::node::Node;

    // =========================================================================
    // Test helper: minimal RtNode implementation
    // =========================================================================

    /// Minimal RtNode for testing trait defaults.
    /// Simulates a motor controller with a fixed WCET budget.
    struct TestMotorNode {
        wcet: Duration,
    }

    impl TestMotorNode {
        fn new(wcet_us: u64) -> Self {
            Self {
                wcet: Duration::from_micros(wcet_us),
            }
        }
    }

    impl Node for TestMotorNode {
        fn tick(&mut self) {}
    }

    impl RtNode for TestMotorNode {
        fn wcet_budget(&self) -> Duration {
            self.wcet
        }
    }

    // =========================================================================
    // Section 1: RtNode trait default tests
    // =========================================================================

    /// Default deadline is 2x WCET budget.
    /// Robotics: standard safety margin — motor controller with 100µs WCET
    /// gets a 200µs deadline, allowing for cache misses and interrupts.
    #[test]
    fn default_deadline_is_2x_wcet() {
        let node = TestMotorNode::new(100);
        assert_eq!(node.deadline(), Duration::from_micros(200));

        // 1ms WCET → 2ms deadline
        let node2 = TestMotorNode::new(1000);
        assert_eq!(node2.deadline(), Duration::from_micros(2000));
    }

    /// Default priority is Medium — not Critical, which is reserved for
    /// safety-critical control loops (e.g. e-stop, watchdog).
    #[test]
    fn default_priority_is_medium() {
        let node = TestMotorNode::new(100);
        assert_eq!(node.rt_priority(), RtPriority::Medium);
        assert_eq!(node.rt_priority().value(), 50);
    }

    /// Default RT class is Soft — hard real-time requires explicit opt-in
    /// because it triggers EmergencyStop on deadline miss.
    #[test]
    fn default_class_is_soft() {
        let node = TestMotorNode::new(100);
        assert_eq!(node.rt_class(), RtClass::Soft);
    }

    /// Deadline miss policy follows RT class.
    /// Hard → EmergencyStop (safety shutdown)
    /// Firm → Skip (drop this tick, continue)
    /// Soft → Warn (log and continue)
    #[test]
    fn deadline_miss_policy_follows_class() {
        // Default (Soft) → Warn
        let node = TestMotorNode::new(100);
        assert_eq!(node.deadline_miss_policy(), DeadlineMissPolicy::Warn);

        // Custom Hard node → EmergencyStop
        struct HardNode;
        impl Node for HardNode {
            fn tick(&mut self) {}
        }
        impl RtNode for HardNode {
            fn wcet_budget(&self) -> Duration {
                Duration::from_micros(50)
            }
            fn rt_class(&self) -> RtClass {
                RtClass::Hard
            }
        }
        let hard = HardNode;
        assert_eq!(
            hard.deadline_miss_policy(),
            DeadlineMissPolicy::EmergencyStop
        );

        // Custom Firm node → Skip
        struct FirmNode;
        impl Node for FirmNode {
            fn tick(&mut self) {}
        }
        impl RtNode for FirmNode {
            fn wcet_budget(&self) -> Duration {
                Duration::from_micros(50)
            }
            fn rt_class(&self) -> RtClass {
                RtClass::Firm
            }
        }
        let firm = FirmNode;
        assert_eq!(firm.deadline_miss_policy(), DeadlineMissPolicy::Skip);
    }

    /// Formal verification defaults: pre/post conditions and invariants
    /// all return true — no constraints unless overridden.
    #[test]
    fn default_formal_verification_all_true() {
        let node = TestMotorNode::new(100);
        assert!(node.pre_condition());
        assert!(node.post_condition());
        assert!(node.invariant());
    }

    /// Default fallback is None — no redundant node unless explicitly provided.
    /// Robotics: N-version programming requires explicit fallback configuration.
    #[test]
    fn default_fallback_is_none() {
        let node = TestMotorNode::new(100);
        assert!(node.fallback_node().is_none());
    }

    /// Default safe state: node reports safe, enter_safe_state is a no-op.
    #[test]
    fn default_safe_state() {
        let mut node = TestMotorNode::new(100);
        assert!(node.is_safe_state());
        node.enter_safe_state(); // Should not panic
    }

    // =========================================================================
    // Section 2: RtPriority ordering and values
    // =========================================================================

    /// Priority values: Critical(0) < High(10) < Medium(50) < Low(100).
    /// Lower value = higher priority (matches POSIX convention).
    #[test]
    fn priority_values() {
        assert_eq!(RtPriority::Critical.value(), 0);
        assert_eq!(RtPriority::High.value(), 10);
        assert_eq!(RtPriority::Medium.value(), 50);
        assert_eq!(RtPriority::Low.value(), 100);
    }

    /// Custom priority carries its exact value.
    #[test]
    fn custom_priority_value() {
        assert_eq!(RtPriority::Custom(42).value(), 42);
        assert_eq!(RtPriority::Custom(0).value(), 0);
        assert_eq!(RtPriority::Custom(999).value(), 999);
    }

    /// RtPriority derives PartialOrd/Ord — Critical < High < Medium < Low
    /// (enum discriminant order matches priority importance).
    #[test]
    fn priority_ordering() {
        assert!(RtPriority::Critical < RtPriority::High);
        assert!(RtPriority::High < RtPriority::Medium);
        assert!(RtPriority::Medium < RtPriority::Low);
        assert!(RtPriority::Low < RtPriority::Custom(0));
    }

    // =========================================================================
    // Section 3: RtStats EMA calculations
    // =========================================================================

    /// First recording sets avg to the duration, jitter to 0.
    /// Robotics: first tick establishes baseline — no jitter measured yet.
    #[test]
    fn rtstats_first_recording_sets_baseline() {
        let mut stats = RtStats::default();
        stats.record_execution(Duration::from_micros(100));

        assert_eq!(stats.total_ticks, 1);
        assert!((stats.avg_execution_us - 100.0).abs() < 1e-6);
        assert!((stats.jitter_us - 0.0).abs() < 1e-6);
        assert_eq!(stats.worst_execution, Duration::from_micros(100));
        assert_eq!(stats.last_execution, Duration::from_micros(100));
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
            (stats.avg_execution_us - expected_avg).abs() < 1e-6,
            "EMA avg mismatch: got {}, expected {}",
            stats.avg_execution_us,
            expected_avg
        );
        assert!(
            (stats.jitter_us - expected_jitter).abs() < 1e-6,
            "Jitter mismatch: got {}, expected {}",
            stats.jitter_us,
            expected_jitter
        );
        assert_eq!(stats.total_ticks, 10);
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
            (stats.avg_execution_us - 200.0).abs() < 1e-6,
            "Avg should converge to constant: {}",
            stats.avg_execution_us
        );
        assert!(
            stats.jitter_us < 0.01,
            "Jitter should be near zero for constant input: {}",
            stats.jitter_us
        );
    }

    /// Worst-case tracking always keeps the absolute maximum.
    /// Robotics: WCET monitoring — one spike sets the record permanently.
    #[test]
    fn rtstats_worst_case_tracks_maximum() {
        let mut stats = RtStats::default();

        stats.record_execution(Duration::from_micros(100));
        assert_eq!(stats.worst_execution, Duration::from_micros(100));

        stats.record_execution(Duration::from_micros(50));
        assert_eq!(
            stats.worst_execution,
            Duration::from_micros(100),
            "Worst should not decrease"
        );

        stats.record_execution(Duration::from_micros(500));
        assert_eq!(stats.worst_execution, Duration::from_micros(500));

        stats.record_execution(Duration::from_micros(200));
        assert_eq!(
            stats.worst_execution,
            Duration::from_micros(500),
            "Worst should stay at 500"
        );

        // Record many smaller values — worst must remain 500
        for _ in 0..100 {
            stats.record_execution(Duration::from_micros(100));
        }
        assert_eq!(stats.worst_execution, Duration::from_micros(500));
    }

    /// Last execution always tracks the most recent value.
    #[test]
    fn rtstats_last_execution_tracks_most_recent() {
        let mut stats = RtStats::default();

        stats.record_execution(Duration::from_micros(100));
        assert_eq!(stats.last_execution, Duration::from_micros(100));

        stats.record_execution(Duration::from_micros(250));
        assert_eq!(stats.last_execution, Duration::from_micros(250));

        stats.record_execution(Duration::from_micros(50));
        assert_eq!(stats.last_execution, Duration::from_micros(50));
    }

    /// Deadline miss counter increments correctly.
    #[test]
    fn rtstats_deadline_miss_counter() {
        let mut stats = RtStats::default();
        assert_eq!(stats.deadline_misses, 0);

        stats.record_deadline_miss();
        assert_eq!(stats.deadline_misses, 1);

        stats.record_deadline_miss();
        stats.record_deadline_miss();
        assert_eq!(stats.deadline_misses, 3);
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
        let avg_before = stats.avg_execution_us;
        assert!((avg_before - 100.0).abs() < 1.0);

        // Step to 500µs — avg should increase
        for _ in 0..20 {
            stats.record_execution(Duration::from_micros(500));
        }
        assert!(
            stats.avg_execution_us > avg_before + 50.0,
            "EMA should respond to step change: avg={}, was={}",
            stats.avg_execution_us,
            avg_before
        );
    }

    // =========================================================================
    // Section 4: WCETViolation struct
    // =========================================================================

    /// WCETViolation stores correct values.
    #[test]
    fn wcet_violation_fields() {
        let v = WCETViolation {
            node_name: "motor_ctrl".to_string(),
            budget: Duration::from_micros(100),
            actual: Duration::from_micros(250),
            overrun: Duration::from_micros(150),
        };
        assert_eq!(v.node_name, "motor_ctrl");
        assert_eq!(v.budget, Duration::from_micros(100));
        assert_eq!(v.actual, Duration::from_micros(250));
        assert_eq!(v.overrun, Duration::from_micros(150));
    }
}
