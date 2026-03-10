//! Failure policies for fault tolerance
//!
//! Defines how the scheduler responds when a node fails:
//!
//! - **Fatal**: Stops the scheduler.
//! - **Restart**: Re-initializes the node with exponential backoff.
//! - **Skip**: Skip the node after repeated failures with a cooldown period.
//! - **Ignore**: Failures are swallowed; the node keeps ticking.

use crate::error::Severity;
use std::time::{Duration, Instant};

/// How the scheduler should respond when a node fails.
///
/// Set per-node via the builder API:
///
/// ```rust,ignore
/// scheduler.add(motor_node)
///     .failure_policy(FailurePolicy::Fatal)
///     .build();
///
/// scheduler.add(logger_node)
///     .failure_policy(FailurePolicy::skip(10, 60_000))
///     .build();
/// ```
#[derive(Debug, Clone)]
pub enum FailurePolicy {
    /// Node failure stops the scheduler immediately.
    ///
    /// Use for: motor controllers, safety systems, anything in the control loop.
    Fatal,

    /// Node failure triggers re-initialization with exponential backoff.
    /// After `max_restarts` exhausted, escalates to a fatal stop.
    ///
    /// Use for: sensor drivers, perception pipelines, recoverable nodes.
    Restart {
        max_restarts: u32,
        initial_backoff_ms: u64,
    },

    /// Node failure is tolerated with a cooldown period.
    /// After `max_failures` consecutive failures, the node is skipped
    /// for `cooldown_ms` before being retried.
    ///
    /// Use for: logging, telemetry, diagnostics, non-critical tasks.
    Skip { max_failures: u32, cooldown_ms: u64 },

    /// Failures are completely ignored. The node keeps ticking every cycle.
    ///
    /// Use for: best-effort nodes where partial results are acceptable.
    Ignore,
}

impl FailurePolicy {
    /// Create a `Restart` policy with common defaults.
    pub fn restart(max_restarts: u32, initial_backoff_ms: u64) -> Self {
        Self::Restart {
            max_restarts,
            initial_backoff_ms,
        }
    }

    /// Create a `Skip` policy with common defaults.
    pub fn skip(max_failures: u32, cooldown_ms: u64) -> Self {
        Self::Skip {
            max_failures,
            cooldown_ms,
        }
    }
}

/// Runtime state for a node's failure handling.
///
/// Created from a [`FailurePolicy`] and maintained by the scheduler
/// during execution. This is the mutable counterpart to the immutable policy.
///
/// Currently not wired into the scheduler's tick loop — `handle_tick_failure()`
/// needs to consult this handler instead of unconditionally continuing.
/// Tracked as a follow-up to re-integrate into `RegisteredNode`.
pub struct FailureHandler {
    /// The policy that governs behavior
    policy: FailurePolicy,
    /// State for the active policy
    state: FailureHandlerState,
}

enum FailureHandlerState {
    /// Fatal: no state needed, first failure stops everything
    Fatal,
    /// Restart: tracks restart count and backoff
    Restart {
        restart_count: u32,
        max_restarts: u32,
        initial_backoff_ms: u64,
        /// When the current backoff period expires (None = not in backoff)
        backoff_until: Option<Instant>,
    },
    /// Skip: tracks failures and suppresses the node after threshold
    Skip {
        failure_count: u32,
        max_failures: u32,
        cooldown_ms: u64,
        /// When the cooldown period expires (None = not suppressed)
        suppressed_until: Option<Instant>,
    },
    /// Ignore: no state needed
    Ignore,
}

/// What the scheduler should do after a node failure.
///
/// ## RT-safe: no heap allocation
///
/// All variants are unit variants (no payload).  The enum derives `Copy`,
/// ensuring it is returned from `record_failure()` entirely on the stack.
/// **Never add a `String` or `Vec` payload to any variant** — that would
/// introduce a heap allocation on the RT scheduling hot path.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FailureAction {
    /// Stop the scheduler immediately
    StopScheduler,
    /// Re-initialize the node
    RestartNode,
    /// Skip the node (failure threshold exceeded)
    SkipNode,
    /// Do nothing, continue as normal
    Continue,
    /// Restart limit exhausted — escalate to fatal stop
    FatalAfterRestarts,
}

/// Summary of a node's failure handler state for monitoring
#[derive(Debug, Clone)]
pub struct FailureHandlerStats {
    /// The policy kind
    pub policy: String,
    /// Current state description
    pub state: String,
    /// Number of failures recorded
    pub failure_count: u32,
    /// Number of restarts (for Restart policy)
    pub restart_count: u32,
    /// Whether the node is currently being skipped/in backoff
    pub is_suppressed: bool,
}

impl FailureHandler {
    /// Create a new handler from a policy.
    pub fn new(policy: FailurePolicy) -> Self {
        let state = match &policy {
            FailurePolicy::Fatal => FailureHandlerState::Fatal,
            FailurePolicy::Restart {
                max_restarts,
                initial_backoff_ms,
            } => FailureHandlerState::Restart {
                restart_count: 0,
                max_restarts: *max_restarts,
                initial_backoff_ms: *initial_backoff_ms,
                backoff_until: None,
            },
            FailurePolicy::Skip {
                max_failures,
                cooldown_ms,
            } => FailureHandlerState::Skip {
                failure_count: 0,
                max_failures: *max_failures,
                cooldown_ms: *cooldown_ms,
                suppressed_until: None,
            },
            FailurePolicy::Ignore => FailureHandlerState::Ignore,
        };
        Self { policy, state }
    }

    /// Check if the node should be allowed to tick this cycle.
    ///
    /// Returns `false` if the node is in a backoff/cooldown period.
    pub fn should_allow(&self) -> bool {
        match &self.state {
            FailureHandlerState::Fatal => true,
            FailureHandlerState::Restart { backoff_until, .. } => match backoff_until {
                Some(until) => Instant::now() >= *until,
                None => true,
            },
            FailureHandlerState::Skip {
                suppressed_until, ..
            } => match suppressed_until {
                Some(until) => Instant::now() >= *until,
                None => true,
            },
            FailureHandlerState::Ignore => true,
        }
    }

    /// Record a successful tick.
    pub fn record_success(&mut self) {
        match &mut self.state {
            FailureHandlerState::Fatal => {}
            FailureHandlerState::Restart {
                restart_count,
                backoff_until,
                ..
            } => {
                // Successful tick after restart: reset backoff, keep restart_count for stats
                *backoff_until = None;
                // Gradual trust recovery: after 10 consecutive successes, reduce restart count
                // (not implemented yet — keep it simple for now)
                let _ = restart_count;
            }
            FailureHandlerState::Skip {
                failure_count,
                suppressed_until,
                ..
            } => {
                *failure_count = 0;
                *suppressed_until = None;
            }
            FailureHandlerState::Ignore => {}
        }
    }

    /// Record a failed tick and return the action the scheduler must take.
    ///
    /// ## RT-safe: no heap allocation
    ///
    /// This method runs on the real-time scheduling hot path and must **never**
    /// allocate heap memory:
    ///
    /// - Returns `FailureAction` by value — `Copy` enum, stack-only.
    /// - All arithmetic uses saturating integer ops (no panics, no allocs).
    /// - `Instant::now()` is stack-allocated.
    /// - Skip handler uses only integer ops and `Instant::now()` (no allocation).
    ///
    /// **Maintenance rule**: do not add `format!()`, `String::new()`,
    /// `Vec::new()`, `Box::new()`, or any other allocating call to this method
    /// or any function it calls on the hot path.
    pub fn record_failure(&mut self) -> FailureAction {
        match &mut self.state {
            FailureHandlerState::Fatal => FailureAction::StopScheduler,
            FailureHandlerState::Restart {
                restart_count,
                max_restarts,
                initial_backoff_ms,
                backoff_until,
            } => {
                *restart_count += 1;
                if *restart_count > *max_restarts {
                    return FailureAction::FatalAfterRestarts;
                }
                // Exponential backoff: initial * 2^(restart_count - 1)
                let backoff_ms =
                    *initial_backoff_ms * 2u64.saturating_pow(restart_count.saturating_sub(1));
                *backoff_until = Some(Instant::now() + Duration::from_millis(backoff_ms));
                FailureAction::RestartNode
            }
            FailureHandlerState::Skip {
                failure_count,
                max_failures,
                cooldown_ms,
                suppressed_until,
            } => {
                *failure_count += 1;
                if *failure_count >= *max_failures {
                    *suppressed_until = Some(Instant::now() + Duration::from_millis(*cooldown_ms));
                    *failure_count = 0;
                    FailureAction::SkipNode
                } else {
                    FailureAction::Continue
                }
            }
            FailureHandlerState::Ignore => FailureAction::Continue,
        }
    }

    /// Record a failure with severity classification from [`HorusError::severity()`].
    ///
    /// Severity overrides the configured policy when it provides a stronger signal:
    ///
    /// - **Fatal**: always returns `StopScheduler`, regardless of policy.
    ///   A fatal error (e.g., shared-memory corruption, node panic) means the
    ///   system is in an unrecoverable state — ignoring or restarting would be unsafe.
    ///
    /// - **Transient** + `Fatal` policy: de-escalates to `RestartNode`.
    ///   A transient error (e.g., topic full, network timeout) on a safety-critical
    ///   node should still attempt recovery rather than killing the scheduler.
    ///
    /// - **Transient** + other policies, or **Permanent**: delegates to the normal
    ///   [`record_failure()`](Self::record_failure) path.
    ///
    /// ## RT-safe: no heap allocation
    ///
    /// Same guarantees as `record_failure()` — `Severity` is `Copy`, no allocations.
    pub fn record_failure_with_severity(&mut self, severity: Severity) -> FailureAction {
        match severity {
            // Fatal always stops — even if the policy says Ignore or Restart.
            Severity::Fatal => FailureAction::StopScheduler,

            // Transient + Fatal policy: de-escalate to restart instead of killing.
            Severity::Transient => match &self.state {
                FailureHandlerState::Fatal => FailureAction::RestartNode,
                _ => self.record_failure(),
            },

            // Permanent: follow the configured policy.
            Severity::Permanent => self.record_failure(),
        }
    }

    /// Get statistics for monitoring/status output.
    pub fn stats(&self) -> FailureHandlerStats {
        match &self.state {
            FailureHandlerState::Fatal => FailureHandlerStats {
                policy: "Fatal".to_string(),
                state: "armed".to_string(),
                failure_count: 0,
                restart_count: 0,
                is_suppressed: false,
            },
            FailureHandlerState::Restart {
                restart_count,
                max_restarts,
                backoff_until,
                ..
            } => {
                let in_backoff = backoff_until
                    .map(|until| Instant::now() < until)
                    .unwrap_or(false);
                FailureHandlerStats {
                    policy: "Restart".to_string(),
                    state: if in_backoff {
                        format!("backoff ({}/{})", restart_count, max_restarts)
                    } else if *restart_count > 0 {
                        format!("recovered ({}/{})", restart_count, max_restarts)
                    } else {
                        "healthy".to_string()
                    },
                    failure_count: *restart_count,
                    restart_count: *restart_count,
                    is_suppressed: in_backoff,
                }
            }
            FailureHandlerState::Skip {
                failure_count,
                suppressed_until,
                ..
            } => {
                let is_suppressed = suppressed_until
                    .map(|until| Instant::now() < until)
                    .unwrap_or(false);
                FailureHandlerStats {
                    policy: "Skip".to_string(),
                    state: if is_suppressed {
                        "suppressed".to_string()
                    } else {
                        "active".to_string()
                    },
                    failure_count: *failure_count,
                    restart_count: 0,
                    is_suppressed,
                }
            }
            FailureHandlerState::Ignore => FailureHandlerStats {
                policy: "Ignore".to_string(),
                state: "active".to_string(),
                failure_count: 0,
                restart_count: 0,
                is_suppressed: false,
            },
        }
    }

    /// Reset the handler to its initial state.
    pub fn reset(&mut self) {
        self.state = match &self.policy {
            FailurePolicy::Fatal => FailureHandlerState::Fatal,
            FailurePolicy::Restart {
                max_restarts,
                initial_backoff_ms,
            } => FailureHandlerState::Restart {
                restart_count: 0,
                max_restarts: *max_restarts,
                initial_backoff_ms: *initial_backoff_ms,
                backoff_until: None,
            },
            FailurePolicy::Skip {
                max_failures,
                cooldown_ms,
            } => FailureHandlerState::Skip {
                failure_count: 0,
                max_failures: *max_failures,
                cooldown_ms: *cooldown_ms,
                suppressed_until: None,
            },
            FailurePolicy::Ignore => FailureHandlerState::Ignore,
        };
    }
}

impl std::fmt::Debug for FailureHandler {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let stats = self.stats();
        f.debug_struct("FailureHandler")
            .field("policy", &stats.policy)
            .field("state", &stats.state)
            .field("suppressed", &stats.is_suppressed)
            .finish()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_fatal_policy_stops_on_first_failure() {
        let mut handler = FailureHandler::new(FailurePolicy::Fatal);
        assert!(handler.should_allow());
        assert_eq!(handler.record_failure(), FailureAction::StopScheduler);
    }

    #[test]
    fn test_restart_policy_with_backoff() {
        let mut handler = FailureHandler::new(FailurePolicy::restart(3, 10));
        assert!(handler.should_allow());

        // First failure: restart with 10ms backoff
        assert_eq!(handler.record_failure(), FailureAction::RestartNode);
        // Should be in backoff
        assert!(!handler.should_allow());

        // Wait for backoff to expire
        std::thread::sleep(Duration::from_millis(15));
        assert!(handler.should_allow());

        // Second failure: restart with 20ms backoff
        assert_eq!(handler.record_failure(), FailureAction::RestartNode);

        // Third failure: restart with 40ms backoff
        std::thread::sleep(Duration::from_millis(25));
        assert_eq!(handler.record_failure(), FailureAction::RestartNode);

        // Fourth failure: exceeds max_restarts (3), escalate to fatal
        std::thread::sleep(Duration::from_millis(45));
        assert_eq!(handler.record_failure(), FailureAction::FatalAfterRestarts);
    }

    #[test]
    fn test_skip_policy_skips_after_threshold() {
        let mut handler = FailureHandler::new(FailurePolicy::skip(3, 100));
        assert!(handler.should_allow());

        // Record failures up to threshold
        handler.record_failure();
        handler.record_failure();
        // Third failure should suppress the node
        let action = handler.record_failure();
        assert_eq!(action, FailureAction::SkipNode);
        assert!(!handler.should_allow());
    }

    #[test]
    fn test_ignore_policy_always_continues() {
        let mut handler = FailureHandler::new(FailurePolicy::Ignore);
        assert!(handler.should_allow());

        for _ in 0..100 {
            assert_eq!(handler.record_failure(), FailureAction::Continue);
        }
        assert!(handler.should_allow());
    }

    #[test]
    fn test_restart_success_clears_backoff() {
        let mut handler = FailureHandler::new(FailurePolicy::restart(5, 10));

        // Fail once, enter backoff
        handler.record_failure();
        assert!(!handler.should_allow());

        // Wait for backoff, succeed
        std::thread::sleep(Duration::from_millis(15));
        handler.record_success();

        // Should be allowed immediately (backoff cleared)
        assert!(handler.should_allow());
    }

    #[test]
    fn test_stats_reporting() {
        let handler = FailureHandler::new(FailurePolicy::Fatal);
        let stats = handler.stats();
        assert_eq!(stats.policy, "Fatal");
        assert!(!stats.is_suppressed);

        let handler = FailureHandler::new(FailurePolicy::skip(5, 30_000));
        let stats = handler.stats();
        assert_eq!(stats.policy, "Skip");
        assert_eq!(stats.failure_count, 0);
    }

    #[test]
    fn test_reset() {
        let mut handler = FailureHandler::new(FailurePolicy::restart(3, 10));
        handler.record_failure();
        handler.record_failure();

        let stats = handler.stats();
        assert_eq!(stats.restart_count, 2);

        handler.reset();
        let stats = handler.stats();
        assert_eq!(stats.restart_count, 0);
    }

    /// Exponential backoff increases with each restart: 10ms, 20ms, 40ms, 80ms...
    /// Robotics: sensor driver restarts with increasing delays to avoid
    /// hammering a disconnected device.
    #[test]
    fn restart_exponential_backoff_increases() {
        let mut handler = FailureHandler::new(FailurePolicy::restart(5, 10));

        // Failure 1: 10ms backoff (10 * 2^0)
        assert_eq!(handler.record_failure(), FailureAction::RestartNode);
        assert!(!handler.should_allow());
        std::thread::sleep(Duration::from_millis(15));
        assert!(
            handler.should_allow(),
            "Should be allowed after 10ms backoff"
        );

        // Failure 2: 20ms backoff (10 * 2^1)
        assert_eq!(handler.record_failure(), FailureAction::RestartNode);
        assert!(!handler.should_allow());
        std::thread::sleep(Duration::from_millis(10));
        assert!(!handler.should_allow(), "Should still be in 20ms backoff");
        std::thread::sleep(Duration::from_millis(15));
        assert!(
            handler.should_allow(),
            "Should be allowed after 20ms backoff"
        );

        // Failure 3: 40ms backoff (10 * 2^2)
        assert_eq!(handler.record_failure(), FailureAction::RestartNode);
        assert!(!handler.should_allow());
        std::thread::sleep(Duration::from_millis(20));
        assert!(!handler.should_allow(), "Should still be in 40ms backoff");
        std::thread::sleep(Duration::from_millis(25));
        assert!(
            handler.should_allow(),
            "Should be allowed after 40ms backoff"
        );
    }

    /// Skip policy full cycle: fail → skip → cooldown → recover.
    ///
    /// Robotics: logging node fails to write to disk, gets skipped, disk
    /// recovers, logging resumes.
    #[test]
    fn skip_policy_full_cycle() {
        let mut handler = FailureHandler::new(FailurePolicy::skip(2, 50));

        // Normal operation
        assert!(handler.should_allow());

        // Two failures → SkipNode
        handler.record_failure();
        let action = handler.record_failure();
        assert_eq!(action, FailureAction::SkipNode);
        assert!(!handler.should_allow());

        // Wait for cooldown
        std::thread::sleep(Duration::from_millis(60));

        // should_allow returns true after cooldown expires
        assert!(handler.should_allow());

        // Success resets failure count
        handler.record_success();
        assert!(handler.should_allow());

        // Back to normal: a single failure doesn't immediately skip
        let action = handler.record_failure();
        assert_eq!(action, FailureAction::Continue);
    }

    /// Restart exhaustion → FatalAfterRestarts escalation.
    /// After max_restarts failures, the policy escalates to fatal.
    ///
    /// Robotics: motor driver fails 3 times with increasing backoff,
    /// on the 4th failure the scheduler stops to prevent damage.
    #[test]
    fn restart_exhaustion_escalates_to_fatal() {
        let mut handler = FailureHandler::new(FailurePolicy::restart(3, 5));

        for i in 0..3 {
            let action = handler.record_failure();
            assert_eq!(
                action,
                FailureAction::RestartNode,
                "Failure {} should be RestartNode",
                i + 1
            );
            // Wait for backoff before next failure
            std::thread::sleep(Duration::from_millis(50));
        }

        // 4th failure exceeds max_restarts → fatal
        let action = handler.record_failure();
        assert_eq!(action, FailureAction::FatalAfterRestarts);
    }

    /// Handler stats correctly report suppression state.
    #[test]
    fn stats_report_suppression_correctly() {
        // Restart handler in backoff is suppressed
        let mut handler = FailureHandler::new(FailurePolicy::restart(5, 100));
        handler.record_failure();
        let stats = handler.stats();
        assert!(stats.is_suppressed, "Should be suppressed during backoff");

        // Skip handler suppressed after threshold
        let mut handler = FailureHandler::new(FailurePolicy::skip(1, 5000));
        handler.record_failure(); // exceeds threshold (max_failures=1)
        let stats = handler.stats();
        assert!(
            stats.is_suppressed,
            "Should be suppressed after failure threshold"
        );

        // Fatal handler is never suppressed
        let handler = FailureHandler::new(FailurePolicy::Fatal);
        assert!(!handler.stats().is_suppressed);

        // Ignore handler is never suppressed
        let handler = FailureHandler::new(FailurePolicy::Ignore);
        assert!(!handler.stats().is_suppressed);
    }

    // =================================================================
    // Severity-aware failure handling
    // =================================================================

    /// Fatal severity always stops, even with Ignore policy.
    #[test]
    fn severity_fatal_overrides_ignore_policy() {
        let mut handler = FailureHandler::new(FailurePolicy::Ignore);
        assert_eq!(
            handler.record_failure_with_severity(Severity::Fatal),
            FailureAction::StopScheduler,
        );
    }

    /// Fatal severity always stops, even with Restart policy.
    #[test]
    fn severity_fatal_overrides_restart_policy() {
        let mut handler = FailureHandler::new(FailurePolicy::restart(5, 100));
        assert_eq!(
            handler.record_failure_with_severity(Severity::Fatal),
            FailureAction::StopScheduler,
        );
    }

    /// Transient severity de-escalates Fatal policy to RestartNode.
    ///
    /// Robotics scenario: safety-critical motor controller (Fatal policy) gets
    /// a transient TopicFull error — should attempt recovery, not kill the
    /// scheduler.
    #[test]
    fn severity_transient_deescalates_fatal_policy() {
        let mut handler = FailureHandler::new(FailurePolicy::Fatal);
        assert_eq!(
            handler.record_failure_with_severity(Severity::Transient),
            FailureAction::RestartNode,
        );
    }

    /// Transient severity with Restart policy follows normal record_failure().
    #[test]
    fn severity_transient_follows_restart_policy() {
        let mut handler = FailureHandler::new(FailurePolicy::restart(3, 10));
        assert_eq!(
            handler.record_failure_with_severity(Severity::Transient),
            FailureAction::RestartNode,
        );
    }

    /// Permanent severity follows the configured policy (Fatal → Stop).
    #[test]
    fn severity_permanent_follows_configured_policy() {
        let mut handler = FailureHandler::new(FailurePolicy::Fatal);
        assert_eq!(
            handler.record_failure_with_severity(Severity::Permanent),
            FailureAction::StopScheduler,
        );

        let mut handler = FailureHandler::new(FailurePolicy::Ignore);
        assert_eq!(
            handler.record_failure_with_severity(Severity::Permanent),
            FailureAction::Continue,
        );
    }
}
