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
use crate::communication::{Topic, TopicKind};
use crate::core::{DurationExt, LogSummary, Node};
use crate::HorusResult;

use parking_lot::RwLock;
use serde::{de::DeserializeOwned, Serialize};
use std::collections::{HashMap, VecDeque};
use std::fmt::Debug;
use std::marker::PhantomData;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant};

/// Shared feedback topic link — lazily initialized, shared across clones.
type FeedbackLink<F> = Arc<RwLock<Option<Topic<ActionFeedback<F>>>>>;

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
struct FeedbackSender<A: Action> {
    link: FeedbackLink<A::Feedback>,
    last_send: Arc<RwLock<Instant>>,
    min_interval: Duration,
}

impl<A: Action> FeedbackSender<A>
where
    A::Feedback: Clone + Send + Sync + Serialize + DeserializeOwned + Debug + 'static,
{
    fn new(link: FeedbackLink<A::Feedback>, rate_hz: f64) -> Self {
        let min_interval = if rate_hz > 0.0 {
            rate_hz.hz().period()
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
                link.send(msg);
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
struct GoalState {
    priority: GoalPriority,
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
    A::Goal: Clone + Send + Sync + Serialize + DeserializeOwned + Debug + 'static,
    A::Feedback: Clone + Send + Sync + Serialize + DeserializeOwned + Debug + 'static,
    A::Result: Clone + Send + Sync + Serialize + DeserializeOwned + Debug + 'static,
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
    A::Goal: Clone + Send + Sync + Serialize + DeserializeOwned + Debug + 'static,
    A::Feedback: Clone + Send + Sync + Serialize + DeserializeOwned + Debug + 'static,
    A::Result: Clone + Send + Sync + Serialize + DeserializeOwned + Debug + 'static,
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
    feedback_link: FeedbackLink<A::Feedback>,
    status_link: Option<Topic<GoalStatusUpdate>>,

    // Active goals
    active_goals: HashMap<GoalId, GoalState>,
    goal_queue: VecDeque<GoalRequest<A::Goal>>,

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
    A::Goal: Clone + Send + Sync + Serialize + DeserializeOwned + Debug + 'static,
    A::Feedback: Clone + Send + Sync + Serialize + DeserializeOwned + Debug + 'static,
    A::Result: Clone + Send + Sync + Serialize + DeserializeOwned + Debug + 'static,
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
            priority: request.priority,
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
            link.send(result);
        }
    }

    /// Publish a status update.
    fn publish_status(&self, goal_id: GoalId, status: GoalStatus) {
        if let Some(ref link) = self.status_link {
            let update = GoalStatusUpdate::new(goal_id, status);
            link.send(update);
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
    A::Goal: Clone + Send + Sync + Serialize + DeserializeOwned + Debug + 'static,
    A::Feedback: Clone + Send + Sync + Serialize + DeserializeOwned + Debug + 'static,
    A::Result: Clone + Send + Sync + Serialize + DeserializeOwned + Debug + 'static,
{
    fn name(&self) -> &str {
        &self.name
    }

    fn init(&mut self) -> HorusResult<()> {
        let action_name = A::name();

        // Create communication links with proper TopicKind for discovery.
        // TopicKind allows `horus action list` to identify action topics
        // without relying solely on naming conventions.
        self.goal_link = Some(Topic::new_with_kind(
            A::goal_topic(),
            TopicKind::ActionGoal as u8,
        )?);
        self.cancel_link = Some(Topic::new_with_kind(
            A::cancel_topic(),
            TopicKind::ActionCancel as u8,
        )?);
        self.result_link = Some(Topic::new_with_kind(
            A::result_topic(),
            TopicKind::ActionResult as u8,
        )?);
        *self.feedback_link.write() = Some(Topic::new_with_kind(
            A::feedback_topic(),
            TopicKind::ActionFeedback as u8,
        )?);
        self.status_link = Some(Topic::new_with_kind(
            A::status_topic(),
            TopicKind::ActionStatus as u8,
        )?);

        log::info!(
            "ActionServer '{}': Initialized with topics: {}.{{goal,cancel,result,feedback,status}}",
            action_name,
            action_name
        );

        Ok(())
    }

    fn tick(&mut self) {
        // JSON gateway: accept goals from CLI (horus action send-goal).
        // CLI writes JSON to .service_gateway/{action}.goal.json.
        {
            let gateway_dir = crate::memory::shm_topics_dir().join(".service_gateway");
            let goal_file = gateway_dir.join(format!("{}.goal.json", A::name()));
            if let Ok(data) = std::fs::read(&goal_file) {
                let _ = std::fs::remove_file(&goal_file);
                if let Ok(json_val) = serde_json::from_slice::<serde_json::Value>(&data) {
                    // Extract fields from the JSON goal request
                    let goal_id_str = json_val
                        .get("goal_id")
                        .and_then(|v| v.as_str())
                        .unwrap_or("cli-goal")
                        .to_string();
                    let priority = json_val
                        .get("priority")
                        .and_then(|v| v.as_u64())
                        .unwrap_or(128) as u8;
                    if let Some(payload) = json_val.get("payload").cloned() {
                        if let Ok(goal) = serde_json::from_value::<A::Goal>(payload) {
                            let goal_uuid = uuid::Uuid::parse_str(&goal_id_str)
                                .unwrap_or_else(|_| uuid::Uuid::new_v4());
                            let goal_req = GoalRequest {
                                goal_id: GoalId(goal_uuid),
                                priority: GoalPriority(priority),
                                goal,
                                timestamp: std::time::SystemTime::now()
                                    .duration_since(std::time::UNIX_EPOCH)
                                    .unwrap_or_default(),
                            };
                            self.handle_goal(goal_req);
                        }
                    }
                }
            }
        }

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
    use crate::actions::types::ActionError;
    use serde::{Deserialize, Serialize};

    // Test action types
    #[derive(Clone, Debug, Serialize, Deserialize)]
    struct TestGoal {
        target: f64,
    }

    #[derive(Clone, Debug, Serialize, Deserialize)]
    struct TestFeedback {
        progress: f32,
    }

    #[derive(Clone, Debug, Serialize, Deserialize)]
    struct TestResult {
        success: bool,
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
        let server = ActionServerBuilder::<TestAction>::new()
            .on_goal(|_goal| GoalResponse::Accept)
            .on_cancel(|_id| CancelResponse::Accept)
            .on_execute(|handle| handle.succeed(TestResult { success: true }))
            .max_concurrent_goals(Some(1))
            .feedback_rate(10.0)
            .build();

        // Verify the server was built with expected initial state
        assert_eq!(server.name(), "test_action_server");
        let metrics = server.metrics();
        assert_eq!(metrics.goals_received, 0);
        assert_eq!(metrics.goals_accepted, 0);
        assert_eq!(metrics.goals_rejected, 0);
        assert_eq!(metrics.goals_succeeded, 0);
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

    // ========================================================================
    // Server negative and edge case tests
    // ========================================================================

    #[test]
    fn test_preemption_policy_variants() {
        assert_eq!(PreemptionPolicy::PreemptOld, PreemptionPolicy::PreemptOld);
        assert_ne!(PreemptionPolicy::PreemptOld, PreemptionPolicy::RejectNew);
        assert_ne!(
            PreemptionPolicy::PreemptOld,
            PreemptionPolicy::Queue { max_size: 10 }
        );
    }

    #[test]
    fn test_server_config_custom() {
        let config = ActionServerConfig {
            max_concurrent_goals: Some(5),
            feedback_rate_hz: 10.0,
            goal_timeout: Some(Duration::from_secs(30)),
            preemption_policy: PreemptionPolicy::Queue { max_size: 10 },
            result_history_size: 100,
        };
        assert_eq!(config.max_concurrent_goals, Some(5));
        assert_eq!(
            config.preemption_policy,
            PreemptionPolicy::Queue { max_size: 10 }
        );
        assert_eq!(config.goal_timeout, Some(Duration::from_secs(30)));
    }

    #[test]
    fn test_server_config_no_concurrent_limit() {
        let config = ActionServerConfig {
            max_concurrent_goals: None,
            feedback_rate_hz: 10.0,
            goal_timeout: None,
            preemption_policy: PreemptionPolicy::RejectNew,
            result_history_size: 100,
        };
        // None means unlimited concurrent goals
        assert_eq!(config.max_concurrent_goals, None);
    }

    #[test]
    fn test_server_metrics_initial() {
        let metrics = ActionServerMetrics::default();
        assert_eq!(metrics.goals_received, 0);
        assert_eq!(metrics.goals_succeeded, 0);
        assert_eq!(metrics.goals_aborted, 0);
        assert_eq!(metrics.goals_rejected, 0);
        assert_eq!(metrics.active_goals, 0);
    }

    #[test]
    fn test_server_builder_default() {
        let server = ActionServerBuilder::<TestAction>::new().build();
        assert_eq!(server.metrics().goals_received, 0);
    }

    #[test]
    fn test_server_builder_with_config() {
        let config = ActionServerConfig {
            max_concurrent_goals: Some(3),
            feedback_rate_hz: 10.0,
            goal_timeout: Some(Duration::from_secs(10)),
            preemption_policy: PreemptionPolicy::RejectNew,
            result_history_size: 100,
        };
        let server = ActionServerBuilder::<TestAction>::new()
            .with_config(config)
            .build();
        assert_eq!(server.metrics().goals_received, 0);
    }

    // ========================================================================
    // Error path and edge-case tests
    // ========================================================================

    // --- PreemptionPolicy edge cases ---

    #[test]
    fn test_preemption_policy_queue_max_size_zero() {
        // A queue with max_size=0 should never accept anything into the queue
        let policy = PreemptionPolicy::Queue { max_size: 0 };
        assert_eq!(policy, PreemptionPolicy::Queue { max_size: 0 });
        // Verify it's distinct from other policies
        assert_ne!(policy, PreemptionPolicy::RejectNew);
        assert_ne!(policy, PreemptionPolicy::PreemptOld);
        assert_ne!(policy, PreemptionPolicy::Priority);
    }

    #[test]
    fn test_preemption_policy_queue_max_size_one() {
        let policy = PreemptionPolicy::Queue { max_size: 1 };
        assert_eq!(policy, PreemptionPolicy::Queue { max_size: 1 });
        assert_ne!(policy, PreemptionPolicy::Queue { max_size: 0 });
        assert_ne!(policy, PreemptionPolicy::Queue { max_size: 2 });
    }

    #[test]
    fn test_preemption_policy_queue_max_size_usize_max() {
        let policy = PreemptionPolicy::Queue {
            max_size: usize::MAX,
        };
        assert_eq!(
            policy,
            PreemptionPolicy::Queue {
                max_size: usize::MAX
            }
        );
    }

    #[test]
    fn test_preemption_policy_all_variants_distinct() {
        let policies = [
            PreemptionPolicy::RejectNew,
            PreemptionPolicy::PreemptOld,
            PreemptionPolicy::Priority,
            PreemptionPolicy::Queue { max_size: 0 },
            PreemptionPolicy::Queue { max_size: 10 },
        ];
        for i in 0..policies.len() {
            for j in (i + 1)..policies.len() {
                assert_ne!(
                    policies[i], policies[j],
                    "Policies at index {} and {} should differ",
                    i, j
                );
            }
        }
    }

    #[test]
    fn test_preemption_policy_debug_format() {
        assert!(format!("{:?}", PreemptionPolicy::RejectNew).contains("RejectNew"));
        assert!(format!("{:?}", PreemptionPolicy::PreemptOld).contains("PreemptOld"));
        assert!(format!("{:?}", PreemptionPolicy::Priority).contains("Priority"));
        let q = format!("{:?}", PreemptionPolicy::Queue { max_size: 42 });
        assert!(q.contains("Queue"));
        assert!(q.contains("42"));
    }

    // --- ActionServerConfig extreme values ---

    #[test]
    fn test_config_zero_concurrent_goals() {
        let config = ActionServerConfig {
            max_concurrent_goals: Some(0),
            feedback_rate_hz: 10.0,
            goal_timeout: None,
            preemption_policy: PreemptionPolicy::RejectNew,
            result_history_size: 100,
        };
        assert_eq!(config.max_concurrent_goals, Some(0));

        // A server with max_concurrent=0 will reject everything via can_accept_goal
        let server = ActionServerBuilder::<TestAction>::new()
            .with_config(config)
            .build();
        let metrics = server.metrics();
        assert_eq!(metrics.active_goals, 0);
        assert_eq!(metrics.queued_goals, 0);
    }

    #[test]
    fn test_config_very_high_feedback_rate() {
        let config = ActionServerConfig {
            max_concurrent_goals: Some(1),
            feedback_rate_hz: 1_000_000.0,
            goal_timeout: None,
            preemption_policy: PreemptionPolicy::PreemptOld,
            result_history_size: 100,
        };
        assert_eq!(config.feedback_rate_hz, 1_000_000.0);
    }

    #[test]
    fn test_config_zero_feedback_rate() {
        let config = ActionServerConfig {
            max_concurrent_goals: Some(1),
            feedback_rate_hz: 0.0,
            goal_timeout: None,
            preemption_policy: PreemptionPolicy::PreemptOld,
            result_history_size: 100,
        };
        assert_eq!(config.feedback_rate_hz, 0.0);
    }

    #[test]
    fn test_config_very_short_timeout() {
        let config = ActionServerConfig {
            max_concurrent_goals: Some(1),
            feedback_rate_hz: 10.0,
            goal_timeout: Some(Duration::from_nanos(1)),
            preemption_policy: PreemptionPolicy::PreemptOld,
            result_history_size: 100,
        };
        assert_eq!(config.goal_timeout, Some(Duration::from_nanos(1)));
    }

    #[test]
    fn test_config_very_long_timeout() {
        let config = ActionServerConfig {
            max_concurrent_goals: Some(1),
            feedback_rate_hz: 10.0,
            goal_timeout: Some(Duration::from_secs(86400 * 365)),
            preemption_policy: PreemptionPolicy::PreemptOld,
            result_history_size: 100,
        };
        assert_eq!(config.goal_timeout, Some(Duration::from_secs(86400 * 365)));
    }

    #[test]
    fn test_config_zero_result_history() {
        let config = ActionServerConfig {
            max_concurrent_goals: Some(1),
            feedback_rate_hz: 10.0,
            goal_timeout: None,
            preemption_policy: PreemptionPolicy::PreemptOld,
            result_history_size: 0,
        };
        assert_eq!(config.result_history_size, 0);
    }

    #[test]
    fn test_config_builder_methods_chain() {
        let config = ActionServerConfig::new()
            .max_goals(5)
            .feedback_rate(50.0)
            .timeout(Duration::from_secs(60))
            .preemption(PreemptionPolicy::Priority)
            .history_size(200);
        assert_eq!(config.max_concurrent_goals, Some(5));
        assert_eq!(config.feedback_rate_hz, 50.0);
        assert_eq!(config.goal_timeout, Some(Duration::from_secs(60)));
        assert_eq!(config.preemption_policy, PreemptionPolicy::Priority);
        assert_eq!(config.result_history_size, 200);
    }

    #[test]
    fn test_config_unlimited_goals() {
        let config = ActionServerConfig::new().unlimited_goals();
        assert_eq!(config.max_concurrent_goals, None);
    }

    #[test]
    fn test_config_default_values() {
        let config = ActionServerConfig::default();
        assert_eq!(config.max_concurrent_goals, Some(1));
        assert_eq!(config.feedback_rate_hz, 10.0);
        assert_eq!(config.goal_timeout, None);
        assert_eq!(config.preemption_policy, PreemptionPolicy::PreemptOld);
        assert_eq!(config.result_history_size, 100);
    }

    // --- ActionServerMetrics accumulation ---

    #[test]
    fn test_metrics_all_fields_default_zero() {
        let m = ActionServerMetrics::default();
        assert_eq!(m.goals_received, 0);
        assert_eq!(m.goals_accepted, 0);
        assert_eq!(m.goals_rejected, 0);
        assert_eq!(m.goals_succeeded, 0);
        assert_eq!(m.goals_aborted, 0);
        assert_eq!(m.goals_canceled, 0);
        assert_eq!(m.goals_preempted, 0);
        assert_eq!(m.active_goals, 0);
        assert_eq!(m.queued_goals, 0);
    }

    #[test]
    fn test_metrics_manual_accumulation() {
        let m = ActionServerMetrics {
            goals_received: 100,
            goals_accepted: 80,
            goals_rejected: 20,
            goals_succeeded: 50,
            goals_aborted: 10,
            goals_canceled: 5,
            goals_preempted: 15,
            active_goals: 3,
            queued_goals: 7,
        };
        assert_eq!(m.goals_received, 100);
        assert_eq!(m.goals_accepted, 80);
        assert_eq!(m.goals_rejected, 20);
        assert_eq!(m.goals_succeeded, 50);
        assert_eq!(m.goals_aborted, 10);
        assert_eq!(m.goals_canceled, 5);
        assert_eq!(m.goals_preempted, 15);
        assert_eq!(m.active_goals, 3);
        assert_eq!(m.queued_goals, 7);
        // Verify totals are internally consistent:
        // received = accepted + rejected
        assert_eq!(m.goals_received, m.goals_accepted + m.goals_rejected);
    }

    #[test]
    fn test_metrics_clone() {
        let m = ActionServerMetrics {
            goals_received: 42,
            goals_accepted: 40,
            goals_rejected: 2,
            goals_succeeded: 30,
            goals_aborted: 5,
            goals_canceled: 3,
            goals_preempted: 2,
            active_goals: 1,
            queued_goals: 4,
        };
        let m2 = m.clone();
        assert_eq!(m.goals_received, m2.goals_received);
        assert_eq!(m.goals_accepted, m2.goals_accepted);
        assert_eq!(m.goals_rejected, m2.goals_rejected);
        assert_eq!(m.goals_succeeded, m2.goals_succeeded);
        assert_eq!(m.goals_aborted, m2.goals_aborted);
        assert_eq!(m.goals_canceled, m2.goals_canceled);
        assert_eq!(m.goals_preempted, m2.goals_preempted);
        assert_eq!(m.active_goals, m2.active_goals);
        assert_eq!(m.queued_goals, m2.queued_goals);
    }

    #[test]
    fn test_metrics_debug_format() {
        let m = ActionServerMetrics::default();
        let dbg = format!("{:?}", m);
        assert!(dbg.contains("goals_received"));
        assert!(dbg.contains("active_goals"));
        assert!(dbg.contains("queued_goals"));
    }

    // --- GoalResponse variant coverage ---

    #[test]
    fn test_goal_response_accept() {
        let r = GoalResponse::Accept;
        assert!(r.is_accepted());
        assert!(!r.is_rejected());
        assert!(r.rejection_reason().is_none());
    }

    #[test]
    fn test_goal_response_reject_with_empty_reason() {
        let r = GoalResponse::Reject(String::new());
        assert!(!r.is_accepted());
        assert!(r.is_rejected());
        assert_eq!(r.rejection_reason(), Some(""));
    }

    #[test]
    fn test_goal_response_reject_with_long_reason() {
        let long_reason = "x".repeat(10_000);
        let r = GoalResponse::Reject(long_reason.clone());
        assert!(r.is_rejected());
        assert_eq!(r.rejection_reason(), Some(long_reason.as_str()));
    }

    // --- CancelResponse variant coverage ---

    #[test]
    fn test_cancel_response_accept() {
        let r = CancelResponse::Accept;
        assert!(r.is_accepted());
        assert!(!r.is_rejected());
        assert!(r.rejection_reason().is_none());
    }

    #[test]
    fn test_cancel_response_reject_with_empty_reason() {
        let r = CancelResponse::Reject(String::new());
        assert!(!r.is_accepted());
        assert!(r.is_rejected());
        assert_eq!(r.rejection_reason(), Some(""));
    }

    #[test]
    fn test_cancel_response_reject_with_reason() {
        let r = CancelResponse::Reject("Goal is critical".to_string());
        assert!(!r.is_accepted());
        assert!(r.is_rejected());
        assert_eq!(r.rejection_reason(), Some("Goal is critical"));
    }

    // --- GoalStatus transitions (each state variant) ---

    #[test]
    fn test_goal_status_pending_is_active_not_terminal() {
        let s = GoalStatus::Pending;
        assert!(s.is_active());
        assert!(!s.is_terminal());
        assert!(!s.is_success());
        assert!(!s.is_failure());
    }

    #[test]
    fn test_goal_status_active_is_active_not_terminal() {
        let s = GoalStatus::Active;
        assert!(s.is_active());
        assert!(!s.is_terminal());
        assert!(!s.is_success());
        assert!(!s.is_failure());
    }

    #[test]
    fn test_goal_status_succeeded_is_terminal_success() {
        let s = GoalStatus::Succeeded;
        assert!(!s.is_active());
        assert!(s.is_terminal());
        assert!(s.is_success());
        assert!(!s.is_failure());
    }

    #[test]
    fn test_goal_status_aborted_is_terminal_failure() {
        let s = GoalStatus::Aborted;
        assert!(!s.is_active());
        assert!(s.is_terminal());
        assert!(!s.is_success());
        assert!(s.is_failure());
    }

    #[test]
    fn test_goal_status_canceled_is_terminal_failure() {
        let s = GoalStatus::Canceled;
        assert!(!s.is_active());
        assert!(s.is_terminal());
        assert!(!s.is_success());
        assert!(s.is_failure());
    }

    #[test]
    fn test_goal_status_preempted_is_terminal_failure() {
        let s = GoalStatus::Preempted;
        assert!(!s.is_active());
        assert!(s.is_terminal());
        assert!(!s.is_success());
        assert!(s.is_failure());
    }

    #[test]
    fn test_goal_status_rejected_is_terminal_failure() {
        let s = GoalStatus::Rejected;
        assert!(!s.is_active());
        assert!(s.is_terminal());
        assert!(!s.is_success());
        assert!(s.is_failure());
    }

    #[test]
    fn test_goal_status_display_all_variants() {
        assert_eq!(format!("{}", GoalStatus::Pending), "PENDING");
        assert_eq!(format!("{}", GoalStatus::Active), "ACTIVE");
        assert_eq!(format!("{}", GoalStatus::Succeeded), "SUCCEEDED");
        assert_eq!(format!("{}", GoalStatus::Aborted), "ABORTED");
        assert_eq!(format!("{}", GoalStatus::Canceled), "CANCELED");
        assert_eq!(format!("{}", GoalStatus::Preempted), "PREEMPTED");
        assert_eq!(format!("{}", GoalStatus::Rejected), "REJECTED");
    }

    // --- GoalOutcome into_result extraction ---

    #[test]
    fn test_goal_outcome_into_result_succeeded() {
        let outcome: GoalOutcome<TestAction> = GoalOutcome::Succeeded(TestResult { success: true });
        let result = outcome.into_result();
        assert!(result.success);
    }

    #[test]
    fn test_goal_outcome_into_result_aborted() {
        let outcome: GoalOutcome<TestAction> = GoalOutcome::Aborted(TestResult { success: false });
        let result = outcome.into_result();
        assert!(!result.success);
    }

    #[test]
    fn test_goal_outcome_into_result_canceled() {
        let outcome: GoalOutcome<TestAction> = GoalOutcome::Canceled(TestResult { success: false });
        let result = outcome.into_result();
        assert!(!result.success);
    }

    #[test]
    fn test_goal_outcome_into_result_preempted() {
        let outcome: GoalOutcome<TestAction> =
            GoalOutcome::Preempted(TestResult { success: false });
        let result = outcome.into_result();
        assert!(!result.success);
    }

    // --- GoalPriority edge cases ---

    #[test]
    fn test_goal_priority_constants_ordering() {
        assert!(GoalPriority::HIGHEST.is_higher_than(&GoalPriority::HIGH));
        assert!(GoalPriority::HIGH.is_higher_than(&GoalPriority::NORMAL));
        assert!(GoalPriority::NORMAL.is_higher_than(&GoalPriority::LOW));
        assert!(GoalPriority::LOW.is_higher_than(&GoalPriority::LOWEST));
    }

    #[test]
    fn test_goal_priority_not_higher_than_self() {
        assert!(!GoalPriority::HIGHEST.is_higher_than(&GoalPriority::HIGHEST));
        assert!(!GoalPriority::NORMAL.is_higher_than(&GoalPriority::NORMAL));
        assert!(!GoalPriority::LOWEST.is_higher_than(&GoalPriority::LOWEST));
    }

    #[test]
    fn test_goal_priority_lowest_not_higher_than_anything() {
        assert!(!GoalPriority::LOWEST.is_higher_than(&GoalPriority::HIGHEST));
        assert!(!GoalPriority::LOWEST.is_higher_than(&GoalPriority::HIGH));
        assert!(!GoalPriority::LOWEST.is_higher_than(&GoalPriority::NORMAL));
        assert!(!GoalPriority::LOWEST.is_higher_than(&GoalPriority::LOW));
        assert!(!GoalPriority::LOWEST.is_higher_than(&GoalPriority::LOWEST));
    }

    #[test]
    fn test_goal_priority_boundary_values() {
        let min = GoalPriority(0);
        let max = GoalPriority(255);
        assert!(min.is_higher_than(&max));
        assert!(!max.is_higher_than(&min));
        assert_eq!(min, GoalPriority::HIGHEST);
        assert_eq!(max, GoalPriority::LOWEST);
    }

    // --- ActionError variant coverage ---

    #[test]
    fn test_action_error_goal_rejected() {
        let err = ActionError::GoalRejected("bad target".to_string());
        assert!(err.to_string().contains("Goal rejected"));
        assert!(err.to_string().contains("bad target"));
    }

    #[test]
    fn test_action_error_goal_canceled() {
        let err = ActionError::GoalCanceled;
        assert!(err.to_string().contains("canceled"));
    }

    #[test]
    fn test_action_error_goal_preempted() {
        let err = ActionError::GoalPreempted;
        assert!(err.to_string().contains("preempted"));
    }

    #[test]
    fn test_action_error_goal_timeout() {
        let err = ActionError::GoalTimeout;
        assert!(err.to_string().contains("timed out"));
    }

    #[test]
    fn test_action_error_server_unavailable() {
        let err = ActionError::ServerUnavailable;
        assert!(err.to_string().contains("unavailable"));
    }

    #[test]
    fn test_action_error_communication_error() {
        let err = ActionError::CommunicationError("link broken".to_string());
        assert!(err.to_string().contains("Communication error"));
        assert!(err.to_string().contains("link broken"));
    }

    #[test]
    fn test_action_error_execution_error() {
        let err = ActionError::ExecutionError("panic in callback".to_string());
        assert!(err.to_string().contains("Execution error"));
        assert!(err.to_string().contains("panic in callback"));
    }

    #[test]
    fn test_action_error_invalid_goal() {
        let err = ActionError::InvalidGoal("NaN coordinate".to_string());
        assert!(err.to_string().contains("Invalid goal"));
        assert!(err.to_string().contains("NaN coordinate"));
    }

    #[test]
    fn test_action_error_goal_not_found() {
        let id = GoalId::new();
        let err = ActionError::GoalNotFound(id);
        let msg = err.to_string();
        assert!(msg.contains("Goal not found"));
        assert!(msg.contains(&id.to_string()));
    }

    // --- Server builder with rejection callback ---

    #[test]
    fn test_server_builder_reject_all_goals() {
        let server = ActionServerBuilder::<TestAction>::new()
            .on_goal(|_goal| GoalResponse::Reject("always reject".into()))
            .build();
        assert_eq!(server.name(), "test_action_server");
        let metrics = server.metrics();
        assert_eq!(metrics.goals_received, 0);
        assert_eq!(metrics.goals_rejected, 0);
    }

    #[test]
    fn test_server_builder_reject_all_cancels() {
        let server = ActionServerBuilder::<TestAction>::new()
            .on_cancel(|_id| CancelResponse::Reject("no cancels allowed".into()))
            .build();
        assert_eq!(server.name(), "test_action_server");
    }

    #[test]
    fn test_server_builder_goal_timeout() {
        let server = ActionServerBuilder::<TestAction>::new()
            .goal_timeout(Duration::from_millis(100))
            .build();
        assert_eq!(server.config.goal_timeout, Some(Duration::from_millis(100)));
    }

    #[test]
    fn test_server_builder_preemption_policy() {
        let server = ActionServerBuilder::<TestAction>::new()
            .preemption_policy(PreemptionPolicy::Queue { max_size: 5 })
            .build();
        assert_eq!(
            server.config.preemption_policy,
            PreemptionPolicy::Queue { max_size: 5 }
        );
    }

    // --- Server with zero max_concurrent_goals and Queue policy ---

    #[test]
    fn test_server_config_queue_with_zero_concurrent() {
        // Queue policy with zero concurrent goals: everything should get queued
        // or rejected depending on queue capacity
        let config = ActionServerConfig {
            max_concurrent_goals: Some(0),
            feedback_rate_hz: 10.0,
            goal_timeout: None,
            preemption_policy: PreemptionPolicy::Queue { max_size: 5 },
            result_history_size: 100,
        };
        let server = ActionServerBuilder::<TestAction>::new()
            .with_config(config)
            .build();
        let m = server.metrics();
        assert_eq!(m.active_goals, 0);
        assert_eq!(m.queued_goals, 0);
    }

    // --- LogSummary coverage ---

    #[test]
    fn test_log_summary_goal_request() {
        let req = GoalRequest::new(TestGoal { target: 1.0 });
        let summary = req.log_summary();
        assert!(summary.contains("GoalReq"));
        assert!(summary.contains(&req.goal_id.to_string()));
    }

    #[test]
    fn test_log_summary_cancel_request() {
        let goal_id = GoalId::new();
        let req = CancelRequest::new(goal_id);
        let summary = req.log_summary();
        assert!(summary.contains("CancelReq"));
        assert!(summary.contains(&goal_id.to_string()));
    }

    #[test]
    fn test_log_summary_action_result() {
        let goal_id = GoalId::new();
        let result = ActionResult::succeeded(goal_id, TestResult { success: true });
        let summary = result.log_summary();
        assert!(summary.contains("Result"));
        assert!(summary.contains(&goal_id.to_string()));
        assert!(summary.contains("SUCCEEDED"));
    }

    #[test]
    fn test_log_summary_action_feedback() {
        let goal_id = GoalId::new();
        let fb = ActionFeedback::new(goal_id, TestFeedback { progress: 0.5 });
        let summary = fb.log_summary();
        assert!(summary.contains("Feedback"));
        assert!(summary.contains(&goal_id.to_string()));
    }

    #[test]
    fn test_log_summary_goal_status_update() {
        let goal_id = GoalId::new();
        let update = GoalStatusUpdate::new(goal_id, GoalStatus::Aborted);
        let summary = update.log_summary();
        assert!(summary.contains("Status"));
        assert!(summary.contains(&goal_id.to_string()));
        assert!(summary.contains("ABORTED"));
    }

    // --- ActionResult factory methods ---

    #[test]
    fn test_action_result_aborted_status() {
        let id = GoalId::new();
        let r = ActionResult::aborted(id, TestResult { success: false });
        assert_eq!(r.status, GoalStatus::Aborted);
        assert_eq!(r.goal_id, id);
        assert!(!r.result.success);
    }

    #[test]
    fn test_action_result_canceled_status() {
        let id = GoalId::new();
        let r = ActionResult::canceled(id, TestResult { success: false });
        assert_eq!(r.status, GoalStatus::Canceled);
        assert_eq!(r.goal_id, id);
    }

    #[test]
    fn test_action_result_preempted_status() {
        let id = GoalId::new();
        let r = ActionResult::preempted(id, TestResult { success: false });
        assert_eq!(r.status, GoalStatus::Preempted);
        assert_eq!(r.goal_id, id);
    }

    // --- GoalRequest with priority ---

    #[test]
    fn test_goal_request_with_priority() {
        let req = GoalRequest::with_priority(TestGoal { target: 5.0 }, GoalPriority::HIGHEST);
        assert_eq!(req.priority, GoalPriority::HIGHEST);
        assert_eq!(req.goal.target, 5.0);
    }

    #[test]
    fn test_goal_request_default_priority_is_normal() {
        let req = GoalRequest::new(TestGoal { target: 1.0 });
        assert_eq!(req.priority, GoalPriority::NORMAL);
    }

    // --- GoalId edge cases ---

    #[test]
    fn test_goal_id_uniqueness_batch() {
        let ids: Vec<GoalId> = (0..100).map(|_| GoalId::new()).collect();
        for i in 0..ids.len() {
            for j in (i + 1)..ids.len() {
                assert_ne!(ids[i], ids[j], "GoalIds {} and {} collided", i, j);
            }
        }
    }

    #[test]
    fn test_goal_id_from_uuid() {
        let uuid = uuid::Uuid::new_v4();
        let id = GoalId::from_uuid(uuid);
        assert_eq!(*id.as_uuid(), uuid);
    }

    #[test]
    fn test_goal_id_display_matches_uuid() {
        let uuid = uuid::Uuid::new_v4();
        let id = GoalId::from_uuid(uuid);
        assert_eq!(format!("{}", id), format!("{}", uuid));
    }

    // --- Server name derivation ---

    #[test]
    fn test_server_name_derived_from_action_name() {
        let server = ActionServerBuilder::<TestAction>::new().build();
        assert_eq!(server.name(), "test_action_server");
    }

    // --- Builder Default trait ---

    #[test]
    fn test_builder_default_impl() {
        let builder = ActionServerBuilder::<TestAction>::default();
        let server = builder.build();
        assert_eq!(server.name(), "test_action_server");
        assert_eq!(server.metrics().goals_received, 0);
    }

    // --- Action trait topic derivation ---

    #[test]
    fn test_action_topic_names() {
        assert_eq!(TestAction::goal_topic(), "test_action.goal");
        assert_eq!(TestAction::cancel_topic(), "test_action.cancel");
        assert_eq!(TestAction::result_topic(), "test_action.result");
        assert_eq!(TestAction::feedback_topic(), "test_action.feedback");
        assert_eq!(TestAction::status_topic(), "test_action.status");
    }
}
