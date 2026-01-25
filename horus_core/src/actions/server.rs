//! Action Server implementation for HORUS.
//!
//! Provides the server-side of the Action pattern for long-running tasks.
//! The action server:
//! - Receives goal requests from clients
//! - Manages goal lifecycle (pending → active → terminal)
//! - Publishes feedback during execution
//! - Handles cancellation requests
//! - Supports preemption policies
//!
//! # Example
//!
//! ```rust,ignore
//! use horus_core::actions::{Action, ActionServer, GoalResponse, CancelResponse, ServerGoalHandle};
//!
//! let server = ActionServer::<NavigateAction>::new()
//!     .on_goal(|goal| {
//!         if goal.target_x.is_finite() {
//!             GoalResponse::Accept
//!         } else {
//!             GoalResponse::Reject("Invalid target".into())
//!         }
//!     })
//!     .on_cancel(|goal_id| CancelResponse::Accept)
//!     .on_execute(|mut handle| {
//!         // Execute the goal
//!         while !handle.is_cancel_requested() {
//!             // Do work...
//!             handle.publish_feedback(NavigateFeedback { progress: 0.5 });
//!         }
//!         handle.succeed(NavigateResult { success: true })
//!     })
//!     .build();
//! ```

use crate::actions::types::{
    Action, ActionFeedback, ActionResult, ActionServerConfig, CancelRequest, CancelResponse,
    GoalId, GoalPriority, GoalRequest, GoalResponse, GoalStatus, GoalStatusUpdate,
    PreemptionPolicy,
};
use crate::communication::Topic;
use crate::core::{LogSummary, Node};
use crate::HorusResult;

use parking_lot::RwLock;
use serde::{de::DeserializeOwned, Serialize};
use std::collections::{HashMap, VecDeque};
use std::fmt::Debug;
use std::marker::PhantomData;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant};

/// Result of goal execution.
///
/// Returned by the execute callback to indicate how the goal completed.
#[derive(Debug)]
pub enum GoalOutcome<A: Action> {
    /// Goal completed successfully.
    Succeeded(A::Result),
    /// Goal was aborted by the server.
    Aborted(A::Result),
    /// Goal was canceled by the client.
    Canceled(A::Result),
    /// Goal was preempted by a higher-priority goal.
    Preempted(A::Result),
}

impl<A: Action> GoalOutcome<A> {
    /// Get the final status for this outcome.
    pub fn status(&self) -> GoalStatus {
        match self {
            GoalOutcome::Succeeded(_) => GoalStatus::Succeeded,
            GoalOutcome::Aborted(_) => GoalStatus::Aborted,
            GoalOutcome::Canceled(_) => GoalStatus::Canceled,
            GoalOutcome::Preempted(_) => GoalStatus::Preempted,
        }
    }

    /// Extract the result from this outcome.
    pub fn into_result(self) -> A::Result {
        match self {
            GoalOutcome::Succeeded(r)
            | GoalOutcome::Aborted(r)
            | GoalOutcome::Canceled(r)
            | GoalOutcome::Preempted(r) => r,
        }
    }
}

/// Handle to an active goal on the server side.
///
/// Provides methods for:
/// - Accessing goal data
/// - Publishing feedback
/// - Checking for cancellation
/// - Completing the goal (succeed/abort/cancel)
pub struct ServerGoalHandle<A: Action> {
    goal_id: GoalId,
    goal: A::Goal,
    priority: GoalPriority,
    status: Arc<RwLock<GoalStatus>>,
    cancel_requested: Arc<AtomicBool>,
    preempt_requested: Arc<AtomicBool>,
    feedback_sender: FeedbackSender<A>,
    started_at: Instant,
}

impl<A: Action> ServerGoalHandle<A> {
    /// Get the goal ID.
    pub fn goal_id(&self) -> GoalId {
        self.goal_id
    }

    /// Get the goal data.
    pub fn goal(&self) -> &A::Goal {
        &self.goal
    }

    /// Get the goal priority.
    pub fn priority(&self) -> GoalPriority {
        self.priority
    }

    /// Get the current status.
    pub fn status(&self) -> GoalStatus {
        *self.status.read()
    }

    /// Check if cancellation was requested.
    pub fn is_cancel_requested(&self) -> bool {
        self.cancel_requested.load(Ordering::Acquire)
    }

    /// Check if preemption was requested.
    pub fn is_preempt_requested(&self) -> bool {
        self.preempt_requested.load(Ordering::Acquire)
    }

    /// Check if the goal should stop execution (canceled or preempted).
    pub fn should_abort(&self) -> bool {
        self.is_cancel_requested() || self.is_preempt_requested()
    }

    /// Get the time since the goal started.
    pub fn elapsed(&self) -> Duration {
        self.started_at.elapsed()
    }

    /// Publish feedback for this goal.
    ///
    /// Feedback is rate-limited according to the server's configuration.
    pub fn publish_feedback(&self, feedback: A::Feedback) {
        self.feedback_sender.send(self.goal_id, feedback);
    }

    /// Mark the goal as succeeded with the given result.
    pub fn succeed(self, result: A::Result) -> GoalOutcome<A> {
        *self.status.write() = GoalStatus::Succeeded;
        GoalOutcome::Succeeded(result)
    }

    /// Mark the goal as aborted with the given result.
    pub fn abort(self, result: A::Result) -> GoalOutcome<A> {
        *self.status.write() = GoalStatus::Aborted;
        GoalOutcome::Aborted(result)
    }

    /// Mark the goal as canceled with the given result.
    ///
    /// Use this when responding to a cancel request.
    pub fn canceled(self, result: A::Result) -> GoalOutcome<A> {
        *self.status.write() = GoalStatus::Canceled;
        GoalOutcome::Canceled(result)
    }

    /// Mark the goal as preempted with the given result.
    ///
    /// Use this when the goal was preempted by a higher-priority goal.
    pub fn preempted(self, result: A::Result) -> GoalOutcome<A> {
        *self.status.write() = GoalStatus::Preempted;
        GoalOutcome::Preempted(result)
    }
}

impl<A: Action> Debug for ServerGoalHandle<A> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("ServerGoalHandle")
            .field("goal_id", &self.goal_id)
            .field("priority", &self.priority)
            .field("status", &self.status())
            .field("cancel_requested", &self.is_cancel_requested())
            .field("elapsed", &self.elapsed())
            .finish()
    }
}

/// Internal feedback sender that handles rate limiting.
#[allow(clippy::type_complexity)]
struct FeedbackSender<A: Action> {
    link: Arc<RwLock<Option<Topic<ActionFeedback<A::Feedback>>>>>,
    last_send: Arc<RwLock<Instant>>,
    min_interval: Duration,
}

impl<A: Action> FeedbackSender<A>
where
    A::Feedback: Clone + Send + Sync + Serialize + DeserializeOwned + Debug + LogSummary + 'static,
{
    #[allow(clippy::type_complexity)]
    fn new(link: Arc<RwLock<Option<Topic<ActionFeedback<A::Feedback>>>>>, rate_hz: f64) -> Self {
        let min_interval = if rate_hz > 0.0 {
            Duration::from_secs_f64(1.0 / rate_hz)
        } else {
            Duration::ZERO
        };
        Self {
            link,
            last_send: Arc::new(RwLock::new(Instant::now() - min_interval)),
            min_interval,
        }
    }

    fn send(&self, goal_id: GoalId, feedback: A::Feedback) {
        let now = Instant::now();
        let elapsed = now.duration_since(*self.last_send.read());

        if elapsed >= self.min_interval {
            if let Some(ref link) = *self.link.read() {
                let msg = ActionFeedback::new(goal_id, feedback);
                let _ = link.send(msg);
                *self.last_send.write() = now;
            }
        }
    }
}

impl<A: Action> Clone for FeedbackSender<A> {
    fn clone(&self) -> Self {
        Self {
            link: self.link.clone(),
            last_send: self.last_send.clone(),
            min_interval: self.min_interval,
        }
    }
}

/// Internal state for tracking an active goal.
#[allow(dead_code)]
struct GoalState<A: Action> {
    goal: A::Goal,
    priority: GoalPriority,
    status: Arc<RwLock<GoalStatus>>,
    cancel_requested: Arc<AtomicBool>,
    preempt_requested: Arc<AtomicBool>,
    started_at: Instant,
    received_at: Instant,
}

/// Callback types for action server.
pub type GoalCallback<A> = Box<dyn Fn(&<A as Action>::Goal) -> GoalResponse + Send + Sync>;
pub type CancelCallback = Box<dyn Fn(GoalId) -> CancelResponse + Send + Sync>;
pub type ExecuteCallback<A> = Box<dyn Fn(ServerGoalHandle<A>) -> GoalOutcome<A> + Send + Sync>;

/// Builder for creating action servers.
///
/// Use this to configure callbacks and settings before building the server.
pub struct ActionServerBuilder<A: Action> {
    goal_callback: Option<GoalCallback<A>>,
    cancel_callback: Option<CancelCallback>,
    execute_callback: Option<ExecuteCallback<A>>,
    config: ActionServerConfig,
    _phantom: PhantomData<A>,
}

impl<A: Action> ActionServerBuilder<A>
where
    A::Goal: Clone + Send + Sync + Serialize + DeserializeOwned + Debug + LogSummary + 'static,
    A::Feedback: Clone + Send + Sync + Serialize + DeserializeOwned + Debug + LogSummary + 'static,
    A::Result: Clone + Send + Sync + Serialize + DeserializeOwned + Debug + LogSummary + 'static,
{
    /// Create a new action server builder.
    pub fn new() -> Self {
        Self {
            goal_callback: None,
            cancel_callback: None,
            execute_callback: None,
            config: ActionServerConfig::default(),
            _phantom: PhantomData,
        }
    }

    /// Set the goal acceptance callback.
    ///
    /// This callback is invoked when a new goal is received.
    /// Return `GoalResponse::Accept` to accept or `GoalResponse::Reject(reason)` to reject.
    pub fn on_goal<F>(mut self, callback: F) -> Self
    where
        F: Fn(&A::Goal) -> GoalResponse + Send + Sync + 'static,
    {
        self.goal_callback = Some(Box::new(callback));
        self
    }

    /// Set the cancel callback.
    ///
    /// This callback is invoked when a cancel request is received.
    /// Return `CancelResponse::Accept` to accept or `CancelResponse::Reject(reason)` to reject.
    pub fn on_cancel<F>(mut self, callback: F) -> Self
    where
        F: Fn(GoalId) -> CancelResponse + Send + Sync + 'static,
    {
        self.cancel_callback = Some(Box::new(callback));
        self
    }

    /// Set the execute callback.
    ///
    /// This callback is invoked to execute an accepted goal.
    /// It receives a `ServerGoalHandle` and should return a `GoalOutcome`.
    pub fn on_execute<F>(mut self, callback: F) -> Self
    where
        F: Fn(ServerGoalHandle<A>) -> GoalOutcome<A> + Send + Sync + 'static,
    {
        self.execute_callback = Some(Box::new(callback));
        self
    }

    /// Set the server configuration.
    pub fn with_config(mut self, config: ActionServerConfig) -> Self {
        self.config = config;
        self
    }

    /// Set the maximum number of concurrent goals.
    pub fn max_concurrent_goals(mut self, max: Option<usize>) -> Self {
        self.config.max_concurrent_goals = max;
        self
    }

    /// Set the feedback rate in Hz.
    pub fn feedback_rate(mut self, rate_hz: f64) -> Self {
        self.config.feedback_rate_hz = rate_hz;
        self
    }

    /// Set the goal timeout.
    pub fn goal_timeout(mut self, timeout: Duration) -> Self {
        self.config.goal_timeout = Some(timeout);
        self
    }

    /// Set the preemption policy.
    pub fn preemption_policy(mut self, policy: PreemptionPolicy) -> Self {
        self.config.preemption_policy = policy;
        self
    }

    /// Build the action server node.
    ///
    /// Returns an `ActionServerNode` that can be added to a scheduler.
    pub fn build(self) -> ActionServerNode<A> {
        ActionServerNode::new(
            self.goal_callback,
            self.cancel_callback,
            self.execute_callback,
            self.config,
        )
    }
}

impl<A: Action> Default for ActionServerBuilder<A>
where
    A::Goal: Clone + Send + Sync + Serialize + DeserializeOwned + Debug + LogSummary + 'static,
    A::Feedback: Clone + Send + Sync + Serialize + DeserializeOwned + Debug + LogSummary + 'static,
    A::Result: Clone + Send + Sync + Serialize + DeserializeOwned + Debug + LogSummary + 'static,
{
    fn default() -> Self {
        Self::new()
    }
}

/// Action server wrapped as a HORUS Node.
///
/// This node handles:
/// - Receiving goal requests and cancel requests
/// - Managing goal lifecycle
/// - Publishing feedback and results
/// - Enforcing preemption policies
#[allow(clippy::type_complexity)]
pub struct ActionServerNode<A: Action> {
    name: String,

    // Callbacks
    goal_callback: Option<GoalCallback<A>>,
    cancel_callback: Option<CancelCallback>,
    execute_callback: Option<ExecuteCallback<A>>,

    // Configuration
    config: ActionServerConfig,

    // Communication Links (lazily initialized in init())
    goal_link: Option<Topic<GoalRequest<A::Goal>>>,
    cancel_link: Option<Topic<CancelRequest>>,
    result_link: Option<Topic<ActionResult<A::Result>>>,
    feedback_link: Arc<RwLock<Option<Topic<ActionFeedback<A::Feedback>>>>>,
    status_link: Option<Topic<GoalStatusUpdate>>,

    // Active goals
    active_goals: HashMap<GoalId, GoalState<A>>,
    goal_queue: VecDeque<GoalRequest<A::Goal>>,

    // Result history for client queries
    #[allow(dead_code)]
    result_history: VecDeque<(GoalId, ActionResult<A::Result>)>,

    // Metrics
    goals_received: AtomicU64,
    goals_accepted: AtomicU64,
    goals_rejected: AtomicU64,
    goals_succeeded: AtomicU64,
    goals_aborted: AtomicU64,
    goals_canceled: AtomicU64,
    goals_preempted: AtomicU64,

    _phantom: PhantomData<A>,
}

impl<A: Action> ActionServerNode<A>
where
    A::Goal: Clone + Send + Sync + Serialize + DeserializeOwned + Debug + LogSummary + 'static,
    A::Feedback: Clone + Send + Sync + Serialize + DeserializeOwned + Debug + LogSummary + 'static,
    A::Result: Clone + Send + Sync + Serialize + DeserializeOwned + Debug + LogSummary + 'static,
{
    /// Create a new action server node.
    fn new(
        goal_callback: Option<GoalCallback<A>>,
        cancel_callback: Option<CancelCallback>,
        execute_callback: Option<ExecuteCallback<A>>,
        config: ActionServerConfig,
    ) -> Self {
        Self {
            name: format!("{}_server", A::name()),
            goal_callback,
            cancel_callback,
            execute_callback,
            config,
            goal_link: None,
            cancel_link: None,
            result_link: None,
            feedback_link: Arc::new(RwLock::new(None)),
            status_link: None,
            active_goals: HashMap::new(),
            goal_queue: VecDeque::new(),
            result_history: VecDeque::new(),
            goals_received: AtomicU64::new(0),
            goals_accepted: AtomicU64::new(0),
            goals_rejected: AtomicU64::new(0),
            goals_succeeded: AtomicU64::new(0),
            goals_aborted: AtomicU64::new(0),
            goals_canceled: AtomicU64::new(0),
            goals_preempted: AtomicU64::new(0),
            _phantom: PhantomData,
        }
    }

    /// Create a builder for this action server.
    pub fn builder() -> ActionServerBuilder<A> {
        ActionServerBuilder::new()
    }

    /// Handle an incoming goal request.
    fn handle_goal(&mut self, request: GoalRequest<A::Goal>) {
        self.goals_received.fetch_add(1, Ordering::Relaxed);

        log::debug!(
            "ActionServer '{}': Received goal {}",
            A::name(),
            request.goal_id
        );

        // Check goal callback
        let response = if let Some(ref callback) = self.goal_callback {
            callback(&request.goal)
        } else {
            GoalResponse::Accept // Accept all by default
        };

        match response {
            GoalResponse::Accept => {
                // Apply preemption policy
                if !self.can_accept_goal(&request) {
                    log::debug!(
                        "ActionServer '{}': Rejecting goal {} due to preemption policy",
                        A::name(),
                        request.goal_id
                    );
                    self.reject_goal(
                        request.goal_id,
                        "Server busy, goal rejected by preemption policy",
                    );
                    return;
                }

                self.accept_goal(request);
            }
            GoalResponse::Reject(reason) => {
                self.reject_goal(request.goal_id, &reason);
            }
        }
    }

    /// Check if a goal can be accepted based on preemption policy.
    fn can_accept_goal(&mut self, request: &GoalRequest<A::Goal>) -> bool {
        let active_count = self.active_goals.len();
        let max_concurrent = self.config.max_concurrent_goals.unwrap_or(usize::MAX);

        if active_count < max_concurrent {
            return true;
        }

        // At capacity, check policy
        match self.config.preemption_policy {
            PreemptionPolicy::RejectNew => false,
            PreemptionPolicy::PreemptOld => {
                // Preempt oldest goal
                if let Some(oldest_id) = self.get_oldest_goal_id() {
                    self.preempt_goal(oldest_id);
                    true
                } else {
                    false
                }
            }
            PreemptionPolicy::Priority => {
                // Find lowest priority active goal
                if let Some((lowest_id, lowest_priority)) = self.get_lowest_priority_goal() {
                    if request.priority.is_higher_than(&lowest_priority) {
                        self.preempt_goal(lowest_id);
                        true
                    } else {
                        false
                    }
                } else {
                    false
                }
            }
            PreemptionPolicy::Queue { max_size } => {
                if self.goal_queue.len() < max_size {
                    self.goal_queue.push_back(request.clone());
                    false // Don't start now, it's queued
                } else {
                    false // Queue full
                }
            }
        }
    }

    /// Get the oldest active goal ID.
    fn get_oldest_goal_id(&self) -> Option<GoalId> {
        self.active_goals
            .iter()
            .min_by_key(|(_, state)| state.received_at)
            .map(|(id, _)| *id)
    }

    /// Get the goal with lowest priority.
    fn get_lowest_priority_goal(&self) -> Option<(GoalId, GoalPriority)> {
        self.active_goals
            .iter()
            .max_by_key(|(_, state)| state.priority.0) // Higher value = lower priority
            .map(|(id, state)| (*id, state.priority))
    }

    /// Accept and start executing a goal.
    fn accept_goal(&mut self, request: GoalRequest<A::Goal>) {
        self.goals_accepted.fetch_add(1, Ordering::Relaxed);

        let goal_id = request.goal_id;
        log::info!("ActionServer '{}': Accepted goal {}", A::name(), goal_id);

        // Create goal state
        let status = Arc::new(RwLock::new(GoalStatus::Active));
        let cancel_requested = Arc::new(AtomicBool::new(false));
        let preempt_requested = Arc::new(AtomicBool::new(false));
        let now = Instant::now();

        let state = GoalState {
            goal: request.goal.clone(),
            priority: request.priority,
            status: status.clone(),
            cancel_requested: cancel_requested.clone(),
            preempt_requested: preempt_requested.clone(),
            started_at: now,
            received_at: now,
        };

        self.active_goals.insert(goal_id, state);

        // Publish status update
        self.publish_status(goal_id, GoalStatus::Active);

        // Execute the goal
        if let Some(ref execute_callback) = self.execute_callback {
            let handle = ServerGoalHandle {
                goal_id,
                goal: request.goal,
                priority: request.priority,
                status,
                cancel_requested,
                preempt_requested,
                feedback_sender: FeedbackSender::new(
                    self.feedback_link.clone(),
                    self.config.feedback_rate_hz,
                ),
                started_at: now,
            };

            let outcome = execute_callback(handle);
            self.complete_goal(goal_id, outcome);
        }
    }

    /// Reject a goal.
    fn reject_goal(&mut self, goal_id: GoalId, reason: &str) {
        self.goals_rejected.fetch_add(1, Ordering::Relaxed);
        log::info!(
            "ActionServer '{}': Rejected goal {}: {}",
            A::name(),
            goal_id,
            reason
        );

        self.publish_status(goal_id, GoalStatus::Rejected);
    }

    /// Preempt an active goal.
    fn preempt_goal(&mut self, goal_id: GoalId) {
        if let Some(state) = self.active_goals.get(&goal_id) {
            log::info!("ActionServer '{}': Preempting goal {}", A::name(), goal_id);
            state.preempt_requested.store(true, Ordering::Release);
        }
    }

    /// Complete a goal with the given outcome.
    fn complete_goal(&mut self, goal_id: GoalId, outcome: GoalOutcome<A>) {
        let status = outcome.status();

        // Update metrics
        match status {
            GoalStatus::Succeeded => {
                self.goals_succeeded.fetch_add(1, Ordering::Relaxed);
            }
            GoalStatus::Aborted => {
                self.goals_aborted.fetch_add(1, Ordering::Relaxed);
            }
            GoalStatus::Canceled => {
                self.goals_canceled.fetch_add(1, Ordering::Relaxed);
            }
            GoalStatus::Preempted => {
                self.goals_preempted.fetch_add(1, Ordering::Relaxed);
            }
            _ => {}
        }

        log::info!(
            "ActionServer '{}': Goal {} completed with status {}",
            A::name(),
            goal_id,
            status
        );

        // Remove from active goals
        self.active_goals.remove(&goal_id);

        // Publish result
        let result = match status {
            GoalStatus::Succeeded => ActionResult::succeeded(goal_id, outcome.into_result()),
            GoalStatus::Aborted => ActionResult::aborted(goal_id, outcome.into_result()),
            GoalStatus::Canceled => ActionResult::canceled(goal_id, outcome.into_result()),
            GoalStatus::Preempted => ActionResult::preempted(goal_id, outcome.into_result()),
            _ => ActionResult::aborted(goal_id, outcome.into_result()),
        };

        self.publish_result(result);
        self.publish_status(goal_id, status);

        // Process queued goals if any
        self.process_goal_queue();
    }

    /// Handle an incoming cancel request.
    fn handle_cancel(&mut self, request: CancelRequest) {
        log::debug!(
            "ActionServer '{}': Cancel request for goal {}",
            A::name(),
            request.goal_id
        );

        // Check if goal exists
        if !self.active_goals.contains_key(&request.goal_id) {
            log::debug!(
                "ActionServer '{}': Cancel rejected - goal {} not found",
                A::name(),
                request.goal_id
            );
            return;
        }

        // Check cancel callback
        let response = if let Some(ref callback) = self.cancel_callback {
            callback(request.goal_id)
        } else {
            CancelResponse::Accept // Accept all cancels by default
        };

        match response {
            CancelResponse::Accept => {
                if let Some(state) = self.active_goals.get(&request.goal_id) {
                    log::info!(
                        "ActionServer '{}': Canceling goal {}",
                        A::name(),
                        request.goal_id
                    );
                    state.cancel_requested.store(true, Ordering::Release);
                }
            }
            CancelResponse::Reject(reason) => {
                log::debug!(
                    "ActionServer '{}': Cancel rejected for goal {}: {}",
                    A::name(),
                    request.goal_id,
                    reason
                );
            }
        }
    }

    /// Check for timed-out goals.
    fn check_timeouts(&mut self) {
        if let Some(timeout) = self.config.goal_timeout {
            let timed_out: Vec<GoalId> = self
                .active_goals
                .iter()
                .filter(|(_, state)| state.started_at.elapsed() > timeout)
                .map(|(id, _)| *id)
                .collect();

            for goal_id in timed_out {
                log::warn!("ActionServer '{}': Goal {} timed out", A::name(), goal_id);
                if let Some(state) = self.active_goals.get(&goal_id) {
                    // Signal timeout as abort
                    state.cancel_requested.store(true, Ordering::Release);
                }
            }
        }
    }

    /// Process queued goals.
    fn process_goal_queue(&mut self) {
        let max_concurrent = self.config.max_concurrent_goals.unwrap_or(usize::MAX);

        while self.active_goals.len() < max_concurrent {
            if let Some(request) = self.goal_queue.pop_front() {
                self.accept_goal(request);
            } else {
                break;
            }
        }
    }

    /// Publish a result.
    fn publish_result(&self, result: ActionResult<A::Result>) {
        if let Some(ref link) = self.result_link {
            // Store in history
            // Note: We can't modify result_history here since we only have &self
            // This would need to be handled differently in a real implementation
            let _ = link.send(result);
        }
    }

    /// Publish a status update.
    fn publish_status(&self, goal_id: GoalId, status: GoalStatus) {
        if let Some(ref link) = self.status_link {
            let update = GoalStatusUpdate::new(goal_id, status);
            let _ = link.send(update);
        }
    }

    /// Get server metrics.
    pub fn metrics(&self) -> ActionServerMetrics {
        ActionServerMetrics {
            goals_received: self.goals_received.load(Ordering::Relaxed),
            goals_accepted: self.goals_accepted.load(Ordering::Relaxed),
            goals_rejected: self.goals_rejected.load(Ordering::Relaxed),
            goals_succeeded: self.goals_succeeded.load(Ordering::Relaxed),
            goals_aborted: self.goals_aborted.load(Ordering::Relaxed),
            goals_canceled: self.goals_canceled.load(Ordering::Relaxed),
            goals_preempted: self.goals_preempted.load(Ordering::Relaxed),
            active_goals: self.active_goals.len(),
            queued_goals: self.goal_queue.len(),
        }
    }
}

/// Metrics for action server.
#[derive(Debug, Clone, Default)]
pub struct ActionServerMetrics {
    /// Total goals received.
    pub goals_received: u64,
    /// Total goals accepted.
    pub goals_accepted: u64,
    /// Total goals rejected.
    pub goals_rejected: u64,
    /// Total goals that succeeded.
    pub goals_succeeded: u64,
    /// Total goals that were aborted.
    pub goals_aborted: u64,
    /// Total goals that were canceled.
    pub goals_canceled: u64,
    /// Total goals that were preempted.
    pub goals_preempted: u64,
    /// Currently active goals.
    pub active_goals: usize,
    /// Currently queued goals.
    pub queued_goals: usize,
}

// Implement Node trait for ActionServerNode
impl<A: Action> Node for ActionServerNode<A>
where
    A::Goal: Clone + Send + Sync + Serialize + DeserializeOwned + Debug + LogSummary + 'static,
    A::Feedback: Clone + Send + Sync + Serialize + DeserializeOwned + Debug + LogSummary + 'static,
    A::Result: Clone + Send + Sync + Serialize + DeserializeOwned + Debug + LogSummary + 'static,
{
    fn name(&self) -> &'static str {
        // We need to return a static str, but our name is dynamic
        // Use the action name with "_server" suffix
        // This is a limitation - we leak memory here for the static lifetime
        Box::leak(self.name.clone().into_boxed_str())
    }

    fn init(&mut self) -> HorusResult<()> {
        let action_name = A::name();

        // Create communication links using Topic::new() - auto-detects role on first send/recv
        self.goal_link = Some(Topic::new(&A::goal_topic())?);
        self.cancel_link = Some(Topic::new(&A::cancel_topic())?);
        self.result_link = Some(Topic::new(&A::result_topic())?);
        *self.feedback_link.write() = Some(Topic::new(&A::feedback_topic())?);
        self.status_link = Some(Topic::new(&A::status_topic())?);

        log::info!(
            "ActionServer '{}': Initialized with topics: {}/{{goal,cancel,result,feedback,status}}",
            action_name,
            action_name
        );

        Ok(())
    }

    fn tick(&mut self) {
        // Collect incoming goals first to avoid borrow conflict
        let goals: Vec<_> = if let Some(ref link) = self.goal_link {
            std::iter::from_fn(|| link.recv()).collect()
        } else {
            Vec::new()
        };

        // Process collected goals
        for goal_req in goals {
            self.handle_goal(goal_req);
        }

        // Collect cancel requests first to avoid borrow conflict
        let cancels: Vec<_> = if let Some(ref link) = self.cancel_link {
            std::iter::from_fn(|| link.recv()).collect()
        } else {
            Vec::new()
        };

        // Process collected cancel requests
        for cancel_req in cancels {
            self.handle_cancel(cancel_req);
        }

        // Check for timed-out goals
        self.check_timeouts();
    }
}

// LogSummary implementations for action message types
impl<G: Debug> LogSummary for GoalRequest<G> {
    fn log_summary(&self) -> String {
        format!("GoalReq(id={}, priority={})", self.goal_id, self.priority.0)
    }
}

impl LogSummary for CancelRequest {
    fn log_summary(&self) -> String {
        format!("CancelReq(id={})", self.goal_id)
    }
}

impl<R: Debug> LogSummary for ActionResult<R> {
    fn log_summary(&self) -> String {
        format!("Result(id={}, status={})", self.goal_id, self.status)
    }
}

impl<F: Debug> LogSummary for ActionFeedback<F> {
    fn log_summary(&self) -> String {
        format!("Feedback(id={})", self.goal_id)
    }
}

impl LogSummary for GoalStatusUpdate {
    fn log_summary(&self) -> String {
        format!("Status(id={}, status={})", self.goal_id, self.status)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use serde::{Deserialize, Serialize};

    // Test action types
    #[derive(Clone, Debug, Serialize, Deserialize)]
    struct TestGoal {
        target: f64,
    }

    impl LogSummary for TestGoal {
        fn log_summary(&self) -> String {
            format!("TestGoal(target={})", self.target)
        }
    }

    #[derive(Clone, Debug, Serialize, Deserialize)]
    struct TestFeedback {
        progress: f32,
    }

    impl LogSummary for TestFeedback {
        fn log_summary(&self) -> String {
            format!("TestFeedback(progress={})", self.progress)
        }
    }

    #[derive(Clone, Debug, Serialize, Deserialize)]
    struct TestResult {
        success: bool,
    }

    impl LogSummary for TestResult {
        fn log_summary(&self) -> String {
            format!("TestResult(success={})", self.success)
        }
    }

    struct TestAction;

    impl Action for TestAction {
        type Goal = TestGoal;
        type Feedback = TestFeedback;
        type Result = TestResult;

        fn name() -> &'static str {
            "test_action"
        }
    }

    #[test]
    fn test_action_server_builder() {
        let _server = ActionServerBuilder::<TestAction>::new()
            .on_goal(|_goal| GoalResponse::Accept)
            .on_cancel(|_id| CancelResponse::Accept)
            .on_execute(|handle| handle.succeed(TestResult { success: true }))
            .max_concurrent_goals(Some(1))
            .feedback_rate(10.0)
            .build();
    }

    #[test]
    fn test_goal_outcome() {
        let outcome: GoalOutcome<TestAction> = GoalOutcome::Succeeded(TestResult { success: true });
        assert_eq!(outcome.status(), GoalStatus::Succeeded);

        let outcome: GoalOutcome<TestAction> = GoalOutcome::Aborted(TestResult { success: false });
        assert_eq!(outcome.status(), GoalStatus::Aborted);

        let outcome: GoalOutcome<TestAction> = GoalOutcome::Canceled(TestResult { success: false });
        assert_eq!(outcome.status(), GoalStatus::Canceled);

        let outcome: GoalOutcome<TestAction> =
            GoalOutcome::Preempted(TestResult { success: false });
        assert_eq!(outcome.status(), GoalStatus::Preempted);
    }

    #[test]
    fn test_server_metrics_default() {
        let metrics = ActionServerMetrics::default();
        assert_eq!(metrics.goals_received, 0);
        assert_eq!(metrics.active_goals, 0);
    }

    #[test]
    fn test_preemption_policy_default() {
        let config = ActionServerConfig::default();
        assert_eq!(config.preemption_policy, PreemptionPolicy::PreemptOld);
    }
}
