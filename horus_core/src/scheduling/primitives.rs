//! Low-level execution primitives for the scheduler.
//!
//! These are the building blocks that the scheduler composes for different
//! execution strategies (sequential, parallel, future RT-thread, etc.).

use std::time::{Duration, Instant};

use super::types::NodeKind;
use crate::core::rt_node::BudgetViolation;
use crate::core::Miss;

/// Result of a single node tick execution.
///
/// Contains the raw timing and panic information. Higher-level concerns
/// (budget checks, failure policies) are handled by the scheduler.
pub(crate) struct TickResult {
    /// The instant when the tick started (needed for deadline checks).
    pub tick_start: Instant,
    /// Wall-clock duration of the tick.
    pub duration: Duration,
    /// The raw result from `catch_unwind` — `Ok(())` on success, `Err(Box<dyn Any>)` on panic.
    pub result: std::thread::Result<()>,
}

/// Executes a single node tick with timing measurement and panic isolation.
///
/// This is the minimal execution primitive: measure time, call `tick()`, catch panics.
/// It does NOT handle rate limiting, failure policies, watchdog feeding,
/// or recording — those are scheduler-level concerns.
pub(crate) struct NodeRunner;

impl NodeRunner {
    /// Run a single tick on the given node.
    ///
    /// Measures wall-clock duration and catches any panics from the node's `tick()` method.
    /// The caller is responsible for all pre-tick checks and post-tick processing.
    #[inline]
    pub fn run_tick(node: &mut NodeKind) -> TickResult {
        let tick_start = Instant::now();
        let result = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
            node.tick();
        }));
        let duration = tick_start.elapsed();
        TickResult {
            tick_start,
            duration,
            result,
        }
    }
}

/// Action to take after a budget violation is detected.
#[derive(Debug)]
pub(crate) struct BudgetViolationResult {
    pub violation: BudgetViolation,
}

/// Action to take after a deadline miss is detected.
#[derive(Debug)]
pub(crate) enum DeadlineAction {
    /// Log warning, no further action.
    Warn,
    /// Pause the node for one tick.
    Skip,
    /// Call `enter_safe_state()` on the node, continue ticking in safe mode.
    SafeMode,
    /// Trigger emergency stop — caller must stop the scheduler.
    EmergencyStop,
}

/// Result of a deadline check when a miss is detected.
#[derive(Debug)]
pub(crate) struct DeadlineMissResult {
    pub elapsed: Duration,
    pub deadline: Duration,
    pub action: DeadlineAction,
}

/// Stateless timing enforcer for budget and deadline checks.
///
/// Extracts the timing violation logic from both the scheduler and RT executor
/// into a single reusable struct. Callers provide the raw tick data and node
/// configuration; the enforcer returns structured results describing what
/// happened and what action to take.
pub(crate) struct TimingEnforcer;

impl TimingEnforcer {
    /// Check whether a tick exceeded its budget.
    ///
    /// Returns `Some(BudgetViolationResult)` if the tick duration exceeds the budget,
    /// `None` otherwise.
    #[inline]
    pub fn check_tick_budget(
        node_name: &str,
        tick_duration: Duration,
        tick_budget: Duration,
    ) -> Option<BudgetViolationResult> {
        if tick_duration > tick_budget {
            Some(BudgetViolationResult {
                violation: BudgetViolation::new(
                    node_name.to_string(),
                    tick_budget,
                    tick_duration,
                ),
            })
        } else {
            None
        }
    }

    /// Check whether a tick missed its deadline.
    ///
    /// `tick_start` is the `Instant` when the tick began; the elapsed time since
    /// then is compared against `deadline`. The `miss` policy determines the action.
    ///
    /// Returns `Some(DeadlineMissResult)` if the deadline was missed, `None` otherwise.
    #[inline]
    pub fn check_deadline(
        tick_start: Instant,
        deadline: Duration,
        miss: Miss,
    ) -> Option<DeadlineMissResult> {
        let elapsed = tick_start.elapsed();
        if elapsed > deadline {
            let action = match miss {
                Miss::Warn => DeadlineAction::Warn,
                Miss::Skip => DeadlineAction::Skip,
                Miss::SafeMode => DeadlineAction::SafeMode,
                Miss::Stop => DeadlineAction::EmergencyStop,
            };
            Some(DeadlineMissResult {
                elapsed,
                deadline,
                action,
            })
        } else {
            None
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::duration_ext::DurationExt;
    use crate::core::Node;

    struct OkNode;
    impl Node for OkNode {
        fn name(&self) -> &str {
            "ok_node"
        }
        fn tick(&mut self) {}
    }

    struct PanicNode;
    impl Node for PanicNode {
        fn name(&self) -> &str {
            "panic_node"
        }
        fn tick(&mut self) {
            panic!("intentional panic");
        }
    }

    struct SlowNode {
        work_us: u64,
    }
    impl Node for SlowNode {
        fn name(&self) -> &str {
            "slow_node"
        }
        fn tick(&mut self) {
            std::thread::sleep(self.work_us.us());
        }
    }

    #[test]
    fn test_run_tick_success() {
        let mut node = NodeKind::new(Box::new(OkNode));
        let result = NodeRunner::run_tick(&mut node);
        result.result.unwrap();
    }

    #[test]
    fn test_run_tick_catches_panic() {
        let mut node = NodeKind::new(Box::new(PanicNode));
        let result = NodeRunner::run_tick(&mut node);
        assert!(result.result.is_err());
    }

    #[test]
    fn test_run_tick_measures_duration() {
        let mut node = NodeKind::new(Box::new(SlowNode { work_us: 1000 }));
        let result = NodeRunner::run_tick(&mut node);
        result.result.unwrap();
        // Should be at least ~1ms (allowing some scheduling jitter)
        assert!(result.duration.as_micros() >= 500);
    }

    // ====================================================================
    // TimingEnforcer tests
    // ====================================================================

    #[test]
    fn test_check_budget_no_violation() {
        let result = TimingEnforcer::check_tick_budget(
            "node",
            100_u64.us(),
            500_u64.us(),
        );
        assert!(result.is_none());
    }

    #[test]
    fn test_check_budget_violation() {
        let result = TimingEnforcer::check_tick_budget(
            "motor_ctrl",
            800_u64.us(),
            500_u64.us(),
        );
        let v = result.expect("should detect violation");
        assert_eq!(v.violation.node_name(), "motor_ctrl");
        assert_eq!(v.violation.budget(), 500_u64.us());
        assert_eq!(v.violation.actual(), 800_u64.us());
        assert_eq!(v.violation.overrun(), 300_u64.us());
    }

    #[test]
    fn test_check_budget_exact_no_violation() {
        let result = TimingEnforcer::check_tick_budget(
            "node",
            500_u64.us(),
            500_u64.us(),
        );
        assert!(result.is_none(), "exact budget should not be a violation");
    }

    #[test]
    fn test_check_deadline_no_miss() {
        // tick_start is now — elapsed is ~0, well within any deadline
        let tick_start = Instant::now();
        let result =
            TimingEnforcer::check_deadline(tick_start, 100_u64.ms(), Miss::Warn);
        assert!(result.is_none());
    }

    #[test]
    fn test_check_deadline_miss() {
        // tick_start was 50ms ago — deadline is 10ms
        let tick_start = Instant::now() - 50_u64.ms();
        let result =
            TimingEnforcer::check_deadline(tick_start, 10_u64.ms(), Miss::Stop);
        let dm = result.expect("should detect deadline miss");
        assert!(dm.elapsed >= 50_u64.ms());
        assert_eq!(dm.deadline, 10_u64.ms());
        assert!(matches!(dm.action, DeadlineAction::EmergencyStop));
    }

    #[test]
    fn test_check_deadline_miss_policy_mapping() {
        let tick_start = Instant::now() - 50_u64.ms();
        let deadline = 10_u64.ms();

        let policies_and_expected = [
            (Miss::Warn, "Warn"),
            (Miss::Skip, "Skip"),
            (Miss::SafeMode, "SafeMode"),
            (Miss::Stop, "EmergencyStop"),
        ];

        for (miss, expected_name) in policies_and_expected {
            let result = TimingEnforcer::check_deadline(tick_start, deadline, miss);
            let dm = result.unwrap_or_else(|| panic!("should detect miss for {}", expected_name));
            let action_name = format!("{:?}", dm.action);
            assert_eq!(action_name, expected_name);
        }
    }
}
