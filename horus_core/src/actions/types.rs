//! Core types for the HORUS Action system.
//!
//! This module provides the fundamental types for long-running task management:
//! - [`GoalId`] - Unique identifier for goals
//! - [`GoalStatus`] - Current state of a goal
//! - [`Action`] - Trait defining an action with Goal/Feedback/Result types
//! - [`GoalResponse`] / [`CancelResponse`] - Server responses to client requests

use crate::core::LogSummary;
use serde::{Deserialize, Serialize};
use std::fmt;
use std::time::Duration;
use uuid::Uuid;

/// Unique identifier for a goal.
///
/// Each goal sent to an action server has a unique ID that can be used
/// to track progress, cancel, or query status.
///
/// # Example
/// ```rust
/// use horus_core::actions::GoalId;
///
/// let goal_id = GoalId::new();
/// println!("Goal ID: {}", goal_id);
/// ```
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub struct GoalId(pub Uuid);

impl GoalId {
    /// Create a new unique goal ID.
    pub fn new() -> Self {
        GoalId(Uuid::new_v4())
    }

    /// Create a goal ID from a specific UUID.
    pub fn from_uuid(uuid: Uuid) -> Self {
        GoalId(uuid)
    }

    /// Get the underlying UUID.
    pub fn as_uuid(&self) -> &Uuid {
        &self.0
    }
}

impl Default for GoalId {
    fn default() -> Self {
        Self::new()
    }
}

impl fmt::Display for GoalId {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}", self.0)
    }
}

impl From<Uuid> for GoalId {
    fn from(uuid: Uuid) -> Self {
        GoalId(uuid)
    }
}

/// Status of a goal in the action server.
///
/// Goals progress through states from Pending to a terminal state
/// (Succeeded, Aborted, Canceled, Preempted, or Rejected).
///
/// # State Machine
/// ```text
/// Pending -> Active -> Succeeded
///              |-> Aborted
///              |-> Canceled
///              |-> Preempted
/// Pending -> Rejected
/// ```
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum GoalStatus {
    /// Goal has been received but not yet processed.
    Pending,
    /// Goal is currently being executed.
    Active,
    /// Goal completed successfully.
    Succeeded,
    /// Goal was aborted by the server (execution failed).
    Aborted,
    /// Goal was canceled by the client.
    Canceled,
    /// Goal was preempted by a higher-priority goal.
    Preempted,
    /// Goal was rejected by the server (validation failed).
    Rejected,
}

impl GoalStatus {
    /// Check if the goal is still active (Pending or Active).
    pub fn is_active(&self) -> bool {
        matches!(self, GoalStatus::Pending | GoalStatus::Active)
    }

    /// Check if the goal has reached a terminal state.
    pub fn is_terminal(&self) -> bool {
        matches!(
            self,
            GoalStatus::Succeeded
                | GoalStatus::Aborted
                | GoalStatus::Canceled
                | GoalStatus::Preempted
                | GoalStatus::Rejected
        )
    }

    /// Check if the goal succeeded.
    pub fn is_success(&self) -> bool {
        matches!(self, GoalStatus::Succeeded)
    }

    /// Check if the goal failed (aborted, canceled, preempted, or rejected).
    pub fn is_failure(&self) -> bool {
        matches!(
            self,
            GoalStatus::Aborted
                | GoalStatus::Canceled
                | GoalStatus::Preempted
                | GoalStatus::Rejected
        )
    }
}

impl fmt::Display for GoalStatus {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            GoalStatus::Pending => write!(f, "PENDING"),
            GoalStatus::Active => write!(f, "ACTIVE"),
            GoalStatus::Succeeded => write!(f, "SUCCEEDED"),
            GoalStatus::Aborted => write!(f, "ABORTED"),
            GoalStatus::Canceled => write!(f, "CANCELED"),
            GoalStatus::Preempted => write!(f, "PREEMPTED"),
            GoalStatus::Rejected => write!(f, "REJECTED"),
        }
    }
}

/// Response from the server when a goal is submitted.
#[derive(Clone, Debug)]
pub enum GoalResponse {
    /// Goal was accepted and will be executed.
    Accept,
    /// Goal was rejected with a reason.
    Reject(String),
}

impl GoalResponse {
    /// Check if the goal was accepted.
    pub fn is_accepted(&self) -> bool {
        matches!(self, GoalResponse::Accept)
    }

    /// Check if the goal was rejected.
    pub fn is_rejected(&self) -> bool {
        matches!(self, GoalResponse::Reject(_))
    }

    /// Get the rejection reason, if any.
    pub fn rejection_reason(&self) -> Option<&str> {
        match self {
            GoalResponse::Reject(reason) => Some(reason),
            GoalResponse::Accept => None,
        }
    }
}

/// Response from the server when a cancel request is made.
#[derive(Clone, Debug)]
pub enum CancelResponse {
    /// Cancel request was accepted.
    Accept,
    /// Cancel request was rejected with a reason.
    Reject(String),
}

impl CancelResponse {
    /// Check if the cancel request was accepted.
    pub fn is_accepted(&self) -> bool {
        matches!(self, CancelResponse::Accept)
    }

    /// Check if the cancel request was rejected.
    pub fn is_rejected(&self) -> bool {
        matches!(self, CancelResponse::Reject(_))
    }

    /// Get the rejection reason, if any.
    pub fn rejection_reason(&self) -> Option<&str> {
        match self {
            CancelResponse::Reject(reason) => Some(reason),
            CancelResponse::Accept => None,
        }
    }
}

/// Defines an action with Goal, Feedback, and Result types.
///
/// An action represents a long-running task that:
/// - Accepts a **Goal** request
/// - Provides periodic **Feedback** during execution
/// - Returns a **Result** on completion
///
/// # Example
/// ```rust,ignore
/// use horus_core::actions::Action;
/// use serde::{Deserialize, Serialize};
///
/// // Define the action types
/// #[derive(Clone, Debug, Serialize, Deserialize)]
/// pub struct NavigateGoal {
///     pub target_x: f64,
///     pub target_y: f64,
/// }
///
/// #[derive(Clone, Debug, Serialize, Deserialize)]
/// pub struct NavigateFeedback {
///     pub distance_remaining: f64,
///     pub progress_percent: f32,
/// }
///
/// #[derive(Clone, Debug, Serialize, Deserialize)]
/// pub struct NavigateResult {
///     pub success: bool,
///     pub final_x: f64,
///     pub final_y: f64,
/// }
///
/// // Implement the Action trait
/// pub struct NavigateToGoal;
///
/// impl Action for NavigateToGoal {
///     type Goal = NavigateGoal;
///     type Feedback = NavigateFeedback;
///     type Result = NavigateResult;
///
///     fn name() -> &'static str {
///         "navigate_to_goal"
///     }
/// }
/// ```
pub trait Action: Send + Sync + 'static {
    /// The goal request type.
    type Goal: Clone
        + fmt::Debug
        + Send
        + Sync
        + Serialize
        + for<'de> Deserialize<'de>
        + LogSummary
        + 'static;

    /// The feedback type sent during execution.
    type Feedback: Clone
        + fmt::Debug
        + Send
        + Sync
        + Serialize
        + for<'de> Deserialize<'de>
        + LogSummary
        + 'static;

    /// The result type sent on completion.
    type Result: Clone
        + fmt::Debug
        + Send
        + Sync
        + Serialize
        + for<'de> Deserialize<'de>
        + LogSummary
        + 'static;

    /// Action name (used for topic names).
    fn name() -> &'static str;

    /// Get the goal topic name.
    fn goal_topic() -> String {
        format!("{}/goal", Self::name())
    }

    /// Get the cancel topic name.
    fn cancel_topic() -> String {
        format!("{}/cancel", Self::name())
    }

    /// Get the result topic name.
    fn result_topic() -> String {
        format!("{}/result", Self::name())
    }

    /// Get the feedback topic name.
    fn feedback_topic() -> String {
        format!("{}/feedback", Self::name())
    }

    /// Get the status topic name.
    fn status_topic() -> String {
        format!("{}/status", Self::name())
    }
}

/// Priority level for goal preemption.
///
/// Lower values indicate higher priority (0 is highest).
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
pub struct GoalPriority(pub u8);

impl GoalPriority {
    /// Highest priority (0).
    pub const HIGHEST: Self = GoalPriority(0);
    /// High priority (64).
    pub const HIGH: Self = GoalPriority(64);
    /// Normal/default priority (128).
    pub const NORMAL: Self = GoalPriority(128);
    /// Low priority (192).
    pub const LOW: Self = GoalPriority(192);
    /// Lowest priority (255).
    pub const LOWEST: Self = GoalPriority(255);

    /// Check if this priority is higher than another (lower value = higher priority).
    pub fn is_higher_than(&self, other: &Self) -> bool {
        self.0 < other.0
    }
}

impl Default for GoalPriority {
    fn default() -> Self {
        Self::NORMAL
    }
}

/// Preemption policy for action servers.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum PreemptionPolicy {
    /// New goals are rejected while one is active.
    RejectNew,
    /// New goals preempt (cancel) active goals.
    #[default]
    PreemptOld,
    /// Goals have priority - higher priority preempts lower.
    Priority,
    /// Queue goals (FIFO) up to a maximum size.
    Queue { max_size: usize },
}

/// Configuration for an action server.
#[derive(Clone, Debug)]
pub struct ActionServerConfig {
    /// Maximum number of concurrent goals (None = unlimited).
    pub max_concurrent_goals: Option<usize>,
    /// Feedback rate limit in Hz.
    pub feedback_rate_hz: f64,
    /// Goal timeout (None = no timeout).
    pub goal_timeout: Option<Duration>,
    /// Preemption policy.
    pub preemption_policy: PreemptionPolicy,
    /// Result history size (how many completed goals to keep).
    pub result_history_size: usize,
}

impl Default for ActionServerConfig {
    fn default() -> Self {
        Self {
            max_concurrent_goals: Some(1), // Single-goal by default
            feedback_rate_hz: 10.0,
            goal_timeout: None,
            preemption_policy: PreemptionPolicy::PreemptOld,
            result_history_size: 100,
        }
    }
}

impl ActionServerConfig {
    /// Create a new config with default values.
    pub fn new() -> Self {
        Self::default()
    }

    /// Allow unlimited concurrent goals.
    pub fn unlimited_goals(mut self) -> Self {
        self.max_concurrent_goals = None;
        self
    }

    /// Set maximum concurrent goals.
    pub fn max_goals(mut self, max: usize) -> Self {
        self.max_concurrent_goals = Some(max);
        self
    }

    /// Set feedback rate in Hz.
    pub fn feedback_rate(mut self, rate_hz: f64) -> Self {
        self.feedback_rate_hz = rate_hz;
        self
    }

    /// Set goal timeout.
    pub fn timeout(mut self, timeout: Duration) -> Self {
        self.goal_timeout = Some(timeout);
        self
    }

    /// Set preemption policy.
    pub fn preemption(mut self, policy: PreemptionPolicy) -> Self {
        self.preemption_policy = policy;
        self
    }

    /// Set result history size.
    pub fn history_size(mut self, size: usize) -> Self {
        self.result_history_size = size;
        self
    }
}

/// Error types specific to actions.
#[derive(Clone, Debug)]
pub enum ActionError {
    /// Goal was rejected by the server.
    GoalRejected(String),
    /// Goal was canceled.
    GoalCanceled,
    /// Goal was preempted.
    GoalPreempted,
    /// Goal timed out.
    GoalTimeout,
    /// Action server not available.
    ServerUnavailable,
    /// Communication error.
    CommunicationError(String),
    /// Execution error.
    ExecutionError(String),
    /// Invalid goal.
    InvalidGoal(String),
    /// Goal not found.
    GoalNotFound(GoalId),
}

impl fmt::Display for ActionError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            ActionError::GoalRejected(reason) => write!(f, "Goal rejected: {}", reason),
            ActionError::GoalCanceled => write!(f, "Goal was canceled"),
            ActionError::GoalPreempted => write!(f, "Goal was preempted"),
            ActionError::GoalTimeout => write!(f, "Goal timed out"),
            ActionError::ServerUnavailable => write!(f, "Action server unavailable"),
            ActionError::CommunicationError(msg) => write!(f, "Communication error: {}", msg),
            ActionError::ExecutionError(msg) => write!(f, "Execution error: {}", msg),
            ActionError::InvalidGoal(msg) => write!(f, "Invalid goal: {}", msg),
            ActionError::GoalNotFound(id) => write!(f, "Goal not found: {}", id),
        }
    }
}

impl std::error::Error for ActionError {}

impl From<ActionError> for crate::HorusError {
    fn from(err: ActionError) -> Self {
        crate::HorusError::Communication(err.to_string())
    }
}

/// Wrapper for goal requests sent over topics.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct GoalRequest<G> {
    /// Unique goal identifier.
    pub goal_id: GoalId,
    /// Goal data.
    pub goal: G,
    /// Goal priority (for priority-based preemption).
    pub priority: GoalPriority,
    /// Request timestamp.
    #[serde(with = "duration_serde")]
    pub timestamp: Duration,
}

impl<G> GoalRequest<G> {
    /// Create a new goal request.
    pub fn new(goal: G) -> Self {
        Self {
            goal_id: GoalId::new(),
            goal,
            priority: GoalPriority::NORMAL,
            timestamp: std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap_or_default(),
        }
    }

    /// Create a goal request with a specific priority.
    pub fn with_priority(goal: G, priority: GoalPriority) -> Self {
        Self {
            goal_id: GoalId::new(),
            goal,
            priority,
            timestamp: std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap_or_default(),
        }
    }
}

/// Request to cancel a goal.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct CancelRequest {
    /// Goal ID to cancel.
    pub goal_id: GoalId,
    /// Request timestamp.
    #[serde(with = "duration_serde")]
    pub timestamp: Duration,
}

impl CancelRequest {
    /// Create a new cancel request.
    pub fn new(goal_id: GoalId) -> Self {
        Self {
            goal_id,
            timestamp: std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap_or_default(),
        }
    }
}

/// Result message sent when a goal completes.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct ActionResult<R> {
    /// Goal ID this result is for.
    pub goal_id: GoalId,
    /// Final status.
    pub status: GoalStatus,
    /// Result data.
    pub result: R,
    /// Completion timestamp.
    #[serde(with = "duration_serde")]
    pub timestamp: Duration,
}

impl<R> ActionResult<R> {
    /// Create a successful result.
    pub fn succeeded(goal_id: GoalId, result: R) -> Self {
        Self {
            goal_id,
            status: GoalStatus::Succeeded,
            result,
            timestamp: std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap_or_default(),
        }
    }

    /// Create an aborted result.
    pub fn aborted(goal_id: GoalId, result: R) -> Self {
        Self {
            goal_id,
            status: GoalStatus::Aborted,
            result,
            timestamp: std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap_or_default(),
        }
    }

    /// Create a canceled result.
    pub fn canceled(goal_id: GoalId, result: R) -> Self {
        Self {
            goal_id,
            status: GoalStatus::Canceled,
            result,
            timestamp: std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap_or_default(),
        }
    }

    /// Create a preempted result.
    pub fn preempted(goal_id: GoalId, result: R) -> Self {
        Self {
            goal_id,
            status: GoalStatus::Preempted,
            result,
            timestamp: std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap_or_default(),
        }
    }
}

/// Feedback message sent during goal execution.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct ActionFeedback<F> {
    /// Goal ID this feedback is for.
    pub goal_id: GoalId,
    /// Feedback data.
    pub feedback: F,
    /// Feedback timestamp.
    #[serde(with = "duration_serde")]
    pub timestamp: Duration,
}

impl<F> ActionFeedback<F> {
    /// Create a new feedback message.
    pub fn new(goal_id: GoalId, feedback: F) -> Self {
        Self {
            goal_id,
            feedback,
            timestamp: std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap_or_default(),
        }
    }
}

/// Status update message.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct GoalStatusUpdate {
    /// Goal ID.
    pub goal_id: GoalId,
    /// Current status.
    pub status: GoalStatus,
    /// Status update timestamp.
    #[serde(with = "duration_serde")]
    pub timestamp: Duration,
}

impl GoalStatusUpdate {
    /// Create a new status update.
    pub fn new(goal_id: GoalId, status: GoalStatus) -> Self {
        Self {
            goal_id,
            status,
            timestamp: std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap_or_default(),
        }
    }
}

/// Helper module for Duration serde serialization.
mod duration_serde {
    use serde::{Deserialize, Deserializer, Serialize, Serializer};
    use std::time::Duration;

    pub fn serialize<S>(duration: &Duration, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: Serializer,
    {
        // Serialize as (secs, nanos)
        (duration.as_secs(), duration.subsec_nanos()).serialize(serializer)
    }

    pub fn deserialize<'de, D>(deserializer: D) -> Result<Duration, D::Error>
    where
        D: Deserializer<'de>,
    {
        let (secs, nanos) = <(u64, u32)>::deserialize(deserializer)?;
        Ok(Duration::new(secs, nanos))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_goal_id() {
        let id1 = GoalId::new();
        let id2 = GoalId::new();
        assert_ne!(id1, id2);

        let id_str = format!("{}", id1);
        assert!(!id_str.is_empty());
    }

    #[test]
    fn test_goal_status() {
        assert!(GoalStatus::Pending.is_active());
        assert!(GoalStatus::Active.is_active());
        assert!(!GoalStatus::Succeeded.is_active());

        assert!(GoalStatus::Succeeded.is_terminal());
        assert!(GoalStatus::Aborted.is_terminal());
        assert!(!GoalStatus::Pending.is_terminal());

        assert!(GoalStatus::Succeeded.is_success());
        assert!(!GoalStatus::Aborted.is_success());

        assert!(GoalStatus::Aborted.is_failure());
        assert!(!GoalStatus::Succeeded.is_failure());
    }

    #[test]
    fn test_goal_priority() {
        assert!(GoalPriority::HIGHEST.is_higher_than(&GoalPriority::NORMAL));
        assert!(GoalPriority::NORMAL.is_higher_than(&GoalPriority::LOW));
        assert!(!GoalPriority::LOW.is_higher_than(&GoalPriority::HIGH));
    }

    #[test]
    fn test_goal_response() {
        let accept = GoalResponse::Accept;
        assert!(accept.is_accepted());
        assert!(!accept.is_rejected());
        assert!(accept.rejection_reason().is_none());

        let reject = GoalResponse::Reject("Too busy".to_string());
        assert!(!reject.is_accepted());
        assert!(reject.is_rejected());
        assert_eq!(reject.rejection_reason(), Some("Too busy"));
    }
}
