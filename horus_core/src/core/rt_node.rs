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

    /// Record a WCET violation
    pub fn record_wcet_violation(&mut self) {
        self.wcet_violations += 1;
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

/// Wrapper to make RtNode work with existing Node-based scheduler
pub struct RtNodeWrapper {
    node: Box<dyn RtNode>,
    stats: RtStats,
    degraded: bool,
}

impl RtNodeWrapper {
    pub fn new(node: Box<dyn RtNode>) -> Self {
        Self {
            node,
            stats: RtStats::default(),
            degraded: false,
        }
    }

    pub fn stats(&self) -> &RtStats {
        &self.stats
    }

    pub fn is_degraded(&self) -> bool {
        self.degraded
    }

    pub fn degrade(&mut self) {
        self.degraded = true;
    }
}

impl Node for RtNodeWrapper {
    fn name(&self) -> &'static str {
        self.node.name()
    }

    fn init(&mut self) -> crate::error::HorusResult<()> {
        self.node.init()
    }

    fn tick(&mut self) {
        // Pre-condition check
        debug_assert!(
            self.node.pre_condition(),
            "Pre-condition failed for {}",
            self.name()
        );

        // Execute the real-time node
        self.node.tick();

        // Post-condition check
        debug_assert!(
            self.node.post_condition(),
            "Post-condition failed for {}",
            self.name()
        );

        // Invariant check
        debug_assert!(
            self.node.invariant(),
            "Invariant violated for {}",
            self.name()
        );
    }

    fn shutdown(&mut self) -> crate::error::HorusResult<()> {
        self.node.shutdown()
    }

    fn rate_hz(&self) -> Option<f64> {
        self.node.rate_hz()
    }
}
