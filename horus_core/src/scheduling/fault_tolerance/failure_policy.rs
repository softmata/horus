//! Failure policies for tier-aware fault tolerance
//!
//! Defines how the scheduler responds when a node fails, with defaults
//! that are appropriate for each execution tier:
//!
//! - **Fatal**: Stops the scheduler. Default for `UltraFast` and `Fast` tiers.
//! - **Restart**: Re-initializes the node with exponential backoff. Default for `Normal`.
//! - **Skip**: Circuit breaker pattern — skip the node after repeated failures.
//! - **Ignore**: Failures are swallowed; the node keeps ticking.

use super::circuit_breaker::CircuitBreaker;
use std::time::{Duration, Instant};

/// How the scheduler should respond when a node fails.
///
/// Each [`NodeTier`](super::super::types::NodeTier) has a sensible default policy,
/// but it can be overridden per-node via the builder API:
///
/// ```rust,ignore
/// scheduler.add(motor_node)
///     .tier(NodeTier::Fast)
///     .failure_policy(FailurePolicy::Fatal)
///     .done();
///
/// scheduler.add(logger_node)
///     .tier(NodeTier::Normal)
///     .failure_policy(FailurePolicy::skip(10, 60_000))
///     .done();
/// ```
#[derive(Debug, Clone)]
pub enum FailurePolicy {
    /// Node failure stops the scheduler immediately.
    ///
    /// Use for: motor controllers, safety systems, anything in the control loop.
    /// This is the default for `UltraFast` and `Fast` tiers.
    Fatal,

    /// Node failure triggers re-initialization with exponential backoff.
    /// After `max_restarts` exhausted, escalates to a fatal stop.
    ///
    /// Use for: sensor drivers, perception pipelines, recoverable nodes.
    /// This is the default for `Normal`, `Isolated`, and `Auto` tiers.
    Restart {
        max_restarts: u32,
        initial_backoff_ms: u64,
    },

    /// Node failure is tolerated via circuit breaker pattern.
    /// After `max_failures` consecutive failures, the node is skipped
    /// for `cooldown_ms` before being retried.
    ///
    /// Use for: logging, telemetry, diagnostics, non-critical tasks.
    /// This is the default for `Background` and `AsyncIO` tiers.
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
    /// Skip: delegates to CircuitBreaker
    Skip { breaker: CircuitBreaker },
    /// Ignore: no state needed
    Ignore,
}

/// What the scheduler should do after a node failure
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FailureAction {
    /// Stop the scheduler immediately
    StopScheduler,
    /// Re-initialize the node
    RestartNode,
    /// Skip the node (circuit breaker opened)
    SkipNode,
    /// Do nothing, continue as normal
    Continue,
    /// Restart limit exhausted — escalate to fatal
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
                breaker: CircuitBreaker::new(*max_failures, 3, *cooldown_ms),
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
            FailureHandlerState::Skip { breaker } => breaker.should_allow(),
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
            FailureHandlerState::Skip { breaker } => breaker.record_success(),
            FailureHandlerState::Ignore => {}
        }
    }

    /// Record a failed tick and get the action the scheduler should take.
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
            FailureHandlerState::Skip { breaker } => {
                breaker.record_failure();
                // If the breaker just opened, return SkipNode to inform the scheduler
                if !breaker.should_allow() {
                    FailureAction::SkipNode
                } else {
                    FailureAction::Continue
                }
            }
            FailureHandlerState::Ignore => FailureAction::Continue,
        }
    }

    /// Get the underlying policy.
    pub fn policy(&self) -> &FailurePolicy {
        &self.policy
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
            FailureHandlerState::Skip { breaker } => {
                let breaker_stats = breaker.stats();
                let is_open = matches!(
                    breaker_stats.state,
                    super::circuit_breaker::CircuitState::Open
                );
                let is_half_open = matches!(
                    breaker_stats.state,
                    super::circuit_breaker::CircuitState::HalfOpen
                );
                FailureHandlerStats {
                    policy: "Skip".to_string(),
                    state: format!("{:?}", breaker_stats.state),
                    failure_count: breaker_stats.failure_count,
                    restart_count: 0,
                    is_suppressed: is_open || is_half_open,
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
                breaker: CircuitBreaker::new(*max_failures, 3, *cooldown_ms),
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
    fn test_skip_policy_uses_circuit_breaker() {
        let mut handler = FailureHandler::new(FailurePolicy::skip(3, 100));
        assert!(handler.should_allow());

        // Record failures up to threshold
        handler.record_failure();
        handler.record_failure();
        // Third failure should trip the breaker
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

    /// Skip policy with full circuit breaker cycle: fail → skip → cooldown →
    /// half-open probe → recover.
    ///
    /// Robotics: logging node fails to write to disk, gets skipped, disk
    /// recovers, logging resumes.
    #[test]
    fn skip_policy_full_circuit_breaker_cycle() {
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

        // should_allow transitions breaker to HalfOpen
        assert!(handler.should_allow());

        // Success → circuit closes
        handler.record_success();
        handler.record_success();
        handler.record_success(); // CircuitBreaker success_threshold=3 (default in Skip)
        assert!(handler.should_allow());

        // Back to normal: record a failure doesn't immediately skip
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

        // Skip handler with open breaker is suppressed
        let mut handler = FailureHandler::new(FailurePolicy::skip(1, 5000));
        handler.record_failure(); // trips breaker (threshold=1)
        let stats = handler.stats();
        assert!(
            stats.is_suppressed,
            "Should be suppressed when breaker is open"
        );

        // Fatal handler is never suppressed
        let handler = FailureHandler::new(FailurePolicy::Fatal);
        assert!(!handler.stats().is_suppressed);

        // Ignore handler is never suppressed
        let handler = FailureHandler::new(FailurePolicy::Ignore);
        assert!(!handler.stats().is_suppressed);
    }
}
