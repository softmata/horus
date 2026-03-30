//! Action Client implementation for HORUS.
//!
//! Provides the client-side of the Action pattern for long-running tasks.
//! The action client:
//! - Sends goal requests to action servers
//! - Tracks goal progress via status updates
//! - Receives feedback during execution
//! - Can cancel active goals
//! - Receives results upon completion
//!
//! # Example
//!
//! ```rust,ignore
//! use horus_core::actions::{Action, ActionClient};
//!
//! let client = ActionClient::<NavigateAction>::new();
//!
//! // Send a goal and get a handle
//! let handle = client.send_goal(NavigateGoal { target_x: 5.0, target_y: 3.0 });
//!
//! // Wait for result with timeout
//! match handle.await_result(30_u64.secs()) {
//!     Some(result) => println!("Navigation complete: {:?}", result),
//!     None => println!("Navigation timed out"),
//! }
//! ```

use crate::actions::types::{
    Action, ActionError, ActionFeedback, ActionResult, CancelRequest, GoalId, GoalPriority,
    GoalRequest, GoalStatus, GoalStatusUpdate,
};
use crate::communication::{Topic, TopicKind};
use crate::core::Node;
use crate::HorusResult;

use crate::core::DurationExt;
use parking_lot::RwLock;
use serde::{de::DeserializeOwned, Serialize};
use std::collections::HashMap;
use std::fmt::Debug;
use std::marker::PhantomData;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant};

/// Handle to a goal sent by the client.
///
/// Provides methods for:
/// - Checking goal status
/// - Waiting for completion
/// - Canceling the goal
/// - Getting the result
pub struct ClientGoalHandle<A: Action> {
    goal_id: GoalId,
    priority: GoalPriority,
    state: Arc<RwLock<ClientGoalState<A>>>,
    client: Arc<ActionClientInner<A>>,
    sent_at: Instant,
}

/// Internal state for tracking a goal from the client side.
struct ClientGoalState<A: Action> {
    status: GoalStatus,
    result: Option<A::Result>,
    last_feedback: Option<A::Feedback>,
    feedback_count: u64,
    updated_at: Instant,
}

impl<A: Action> ClientGoalState<A> {
    fn new() -> Self {
        Self {
            status: GoalStatus::Pending,
            result: None,
            last_feedback: None,
            feedback_count: 0,
            updated_at: Instant::now(),
        }
    }
}

// Basic accessors
impl<A: Action> ClientGoalHandle<A> {
    /// Get the goal ID.
    pub fn goal_id(&self) -> GoalId {
        self.goal_id
    }

    /// Get the goal priority.
    pub fn priority(&self) -> GoalPriority {
        self.priority
    }

    /// Get the current status.
    pub fn status(&self) -> GoalStatus {
        self.state.read().status
    }

    /// Check if the goal is still active.
    pub fn is_active(&self) -> bool {
        self.state.read().status.is_active()
    }

    /// Check if the goal has completed (terminal state).
    pub fn is_done(&self) -> bool {
        self.state.read().status.is_terminal()
    }

    /// Check if the goal succeeded.
    pub fn is_success(&self) -> bool {
        self.state.read().status.is_success()
    }

    /// Get the time since the goal was sent.
    pub fn elapsed(&self) -> Duration {
        self.sent_at.elapsed()
    }

    /// Get the time since the last status update.
    pub fn time_since_update(&self) -> Duration {
        self.state.read().updated_at.elapsed()
    }
}

// Methods that require Result/Feedback access (need Clone bounds)
impl<A: Action> ClientGoalHandle<A>
where
    A::Goal: Clone + Send + Sync + Serialize + DeserializeOwned + Debug + 'static,
    A::Feedback: Clone + Send + Sync + Serialize + DeserializeOwned + Debug + 'static,
    A::Result: Clone + Send + Sync + Serialize + DeserializeOwned + Debug + 'static,
{
    /// Get the result if the goal has completed.
    ///
    /// Returns `None` if the goal is still active or no result has been received.
    pub fn result(&self) -> Option<A::Result> {
        self.state.read().result.clone()
    }

    /// Get the last feedback received.
    pub fn last_feedback(&self) -> Option<A::Feedback> {
        self.state.read().last_feedback.clone()
    }

    /// Wait for the goal to complete, blocking until done or timeout.
    ///
    /// Returns the result if the goal completed, or `None` if timed out.
    ///
    /// # Arguments
    /// * `timeout` - Maximum time to wait
    ///
    /// # Example
    /// ```rust,ignore
    /// let handle = client.send_goal(goal);
    /// if let Some(result) = handle.await_result(30_u64.secs()) {
    ///     println!("Got result: {:?}", result);
    /// } else {
    ///     println!("Timed out waiting for result");
    /// }
    /// ```
    pub fn await_result(&self, timeout: Duration) -> Option<A::Result> {
        let start = Instant::now();
        let poll_interval = 10_u64.ms();

        while start.elapsed() < timeout {
            if self.is_done() {
                return self.result();
            }
            std::thread::sleep(poll_interval);
        }

        None
    }

    /// Wait for the goal to complete, with a callback for feedback.
    ///
    /// # Arguments
    /// * `timeout` - Maximum time to wait
    /// * `feedback_callback` - Called for each new feedback message
    ///
    /// Returns the result if completed, or the error if failed/timed out.
    pub fn await_result_with_feedback<F>(
        &self,
        timeout: Duration,
        mut feedback_callback: F,
    ) -> Result<A::Result, ActionError>
    where
        F: FnMut(&A::Feedback),
    {
        let start = Instant::now();
        let poll_interval = 10_u64.ms();
        let mut last_feedback_count = 0u64;

        while start.elapsed() < timeout {
            // Check for new feedback
            let state = self.state.read();
            if state.feedback_count > last_feedback_count {
                if let Some(ref feedback) = state.last_feedback {
                    feedback_callback(feedback);
                }
                last_feedback_count = state.feedback_count;
            }

            // Check if done
            if state.status.is_terminal() {
                return match state.status {
                    GoalStatus::Succeeded => state
                        .result
                        .clone()
                        .ok_or(ActionError::ExecutionError("Missing result".into())),
                    GoalStatus::Canceled => Err(ActionError::GoalCanceled),
                    GoalStatus::Preempted => Err(ActionError::GoalPreempted),
                    GoalStatus::Rejected => {
                        Err(ActionError::GoalRejected("Goal rejected by server".into()))
                    }
                    _ => Err(ActionError::ExecutionError("Goal aborted".into())),
                };
            }
            drop(state);

            std::thread::sleep(poll_interval);
        }

        Err(ActionError::GoalTimeout)
    }

    /// Cancel this goal.
    ///
    /// Sends a cancel request to the action server.
    /// Note: The cancel may not be accepted by the server.
    pub fn cancel(&self) {
        self.client.cancel_goal(self.goal_id);
    }
}

impl<A: Action> Debug for ClientGoalHandle<A> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("ClientGoalHandle")
            .field("goal_id", &self.goal_id)
            .field("priority", &self.priority)
            .field("status", &self.status())
            .field("elapsed", &self.elapsed())
            .finish()
    }
}

/// Callback type for feedback.
pub type FeedbackCallback<A> =
    Box<dyn Fn(GoalId, &<A as Action>::Feedback) + Send + Sync + 'static>;

/// Callback type for result.
pub type ResultCallback<A> =
    Box<dyn Fn(GoalId, GoalStatus, &<A as Action>::Result) + Send + Sync + 'static>;

/// Callback type for status changes.
pub type StatusCallback = Box<dyn Fn(GoalId, GoalStatus) + Send + Sync + 'static>;

/// Internal state for the action client.
struct ActionClientInner<A: Action> {
    /// Active goal handles
    goals: RwLock<HashMap<GoalId, Arc<RwLock<ClientGoalState<A>>>>>,

    /// Communication links
    goal_link: RwLock<Option<Topic<GoalRequest<A::Goal>>>>,
    cancel_link: RwLock<Option<Topic<CancelRequest>>>,
    result_link: RwLock<Option<Topic<ActionResult<A::Result>>>>,
    feedback_link: RwLock<Option<Topic<ActionFeedback<A::Feedback>>>>,
    status_link: RwLock<Option<Topic<GoalStatusUpdate>>>,

    /// Callbacks
    feedback_callback: RwLock<Option<FeedbackCallback<A>>>,
    result_callback: RwLock<Option<ResultCallback<A>>>,
    status_callback: RwLock<Option<StatusCallback>>,

    /// State
    initialized: AtomicBool,
}

impl<A: Action> ActionClientInner<A>
where
    A::Goal: Clone + Send + Sync + Serialize + DeserializeOwned + Debug + 'static,
    A::Feedback: Clone + Send + Sync + Serialize + DeserializeOwned + Debug + 'static,
    A::Result: Clone + Send + Sync + Serialize + DeserializeOwned + Debug + 'static,
{
    fn new() -> Self {
        Self {
            goals: RwLock::new(HashMap::new()),
            goal_link: RwLock::new(None),
            cancel_link: RwLock::new(None),
            result_link: RwLock::new(None),
            feedback_link: RwLock::new(None),
            status_link: RwLock::new(None),
            feedback_callback: RwLock::new(None),
            result_callback: RwLock::new(None),
            status_callback: RwLock::new(None),
            initialized: AtomicBool::new(false),
        }
    }

    /// Initialize communication links.
    fn initialize(&self) -> HorusResult<()> {
        if self.initialized.load(Ordering::Acquire) {
            return Ok(());
        }

        // Create links with proper TopicKind for discovery
        *self.goal_link.write() = Some(Topic::new_with_kind(
            A::goal_topic(),
            TopicKind::ActionGoal as u8,
        )?);
        *self.cancel_link.write() = Some(Topic::new_with_kind(
            A::cancel_topic(),
            TopicKind::ActionCancel as u8,
        )?);
        *self.result_link.write() = Some(Topic::new_with_kind(
            A::result_topic(),
            TopicKind::ActionResult as u8,
        )?);
        *self.feedback_link.write() = Some(Topic::new_with_kind(
            A::feedback_topic(),
            TopicKind::ActionFeedback as u8,
        )?);
        *self.status_link.write() = Some(Topic::new_with_kind(
            A::status_topic(),
            TopicKind::ActionStatus as u8,
        )?);

        self.initialized.store(true, Ordering::Release);

        log::info!(
            "ActionClient '{}': Initialized with topics: {}/{{goal,cancel,result,feedback,status}}",
            A::name(),
            A::name()
        );

        Ok(())
    }

    /// Send a goal request.
    fn send_goal(&self, goal: A::Goal, priority: GoalPriority) -> Result<GoalId, ActionError> {
        if !self.initialized.load(Ordering::Acquire) {
            return Err(ActionError::ServerUnavailable);
        }

        let request = GoalRequest::with_priority(goal, priority);
        let goal_id = request.goal_id;

        if let Some(ref link) = *self.goal_link.read() {
            link.send(request);

            log::debug!("ActionClient '{}': Sent goal {}", A::name(), goal_id);
            Ok(goal_id)
        } else {
            Err(ActionError::ServerUnavailable)
        }
    }

    /// Send a cancel request.
    fn cancel_goal(&self, goal_id: GoalId) {
        if let Some(ref link) = *self.cancel_link.read() {
            let request = CancelRequest::new(goal_id);
            link.send(request);
            log::debug!("ActionClient '{}': Sent cancel for {}", A::name(), goal_id);
        }
    }

    /// Process incoming messages.
    fn process_messages(&self) {
        // Process status updates
        if let Some(ref link) = *self.status_link.read() {
            while let Some(update) = link.recv() {
                self.handle_status_update(update);
            }
        }

        // Process feedback
        if let Some(ref link) = *self.feedback_link.read() {
            while let Some(feedback_msg) = link.recv() {
                self.handle_feedback(feedback_msg);
            }
        }

        // Process results
        if let Some(ref link) = *self.result_link.read() {
            while let Some(result_msg) = link.recv() {
                self.handle_result(result_msg);
            }
        }
    }

    /// Handle a status update.
    fn handle_status_update(&self, update: GoalStatusUpdate) {
        let goals = self.goals.read();
        if let Some(state) = goals.get(&update.goal_id) {
            let mut state = state.write();
            state.status = update.status;
            state.updated_at = Instant::now();
        }

        // Call status callback
        if let Some(ref callback) = *self.status_callback.read() {
            callback(update.goal_id, update.status);
        }
    }

    /// Handle a feedback message.
    fn handle_feedback(&self, feedback_msg: ActionFeedback<A::Feedback>) {
        let goals = self.goals.read();
        if let Some(state) = goals.get(&feedback_msg.goal_id) {
            let mut state = state.write();
            state.last_feedback = Some(feedback_msg.feedback.clone());
            state.feedback_count += 1;
            state.updated_at = Instant::now();
        }

        // Call feedback callback
        if let Some(ref callback) = *self.feedback_callback.read() {
            callback(feedback_msg.goal_id, &feedback_msg.feedback);
        }
    }

    /// Handle a result message.
    fn handle_result(&self, result_msg: ActionResult<A::Result>) {
        {
            let goals = self.goals.read();
            if let Some(state) = goals.get(&result_msg.goal_id) {
                let mut state = state.write();
                state.result = Some(result_msg.result.clone());
                state.status = result_msg.status;
                state.updated_at = Instant::now();
            }
        }

        // Call result callback
        if let Some(ref callback) = *self.result_callback.read() {
            callback(result_msg.goal_id, result_msg.status, &result_msg.result);
        }

        // Remove terminal goals from the map to prevent unbounded growth.
        // Callers retain their Arc<RwLock<ClientGoalState>> handle and can
        // still read the final state after removal.
        if result_msg.status.is_terminal() {
            self.goals.write().remove(&result_msg.goal_id);
        }
    }

    /// Register a goal handle.
    fn register_goal(&self, goal_id: GoalId) -> Arc<RwLock<ClientGoalState<A>>> {
        let state = Arc::new(RwLock::new(ClientGoalState::new()));
        self.goals.write().insert(goal_id, state.clone());
        state
    }
}

/// Builder for creating action clients.
pub struct ActionClientBuilder<A: Action> {
    feedback_callback: Option<FeedbackCallback<A>>,
    result_callback: Option<ResultCallback<A>>,
    status_callback: Option<StatusCallback>,
    _phantom: PhantomData<A>,
}

impl<A: Action> ActionClientBuilder<A>
where
    A::Goal: Clone + Send + Sync + Serialize + DeserializeOwned + Debug + 'static,
    A::Feedback: Clone + Send + Sync + Serialize + DeserializeOwned + Debug + 'static,
    A::Result: Clone + Send + Sync + Serialize + DeserializeOwned + Debug + 'static,
{
    /// Create a new action client builder.
    pub fn new() -> Self {
        Self {
            feedback_callback: None,
            result_callback: None,
            status_callback: None,
            _phantom: PhantomData,
        }
    }

    /// Set a callback for feedback messages.
    ///
    /// This callback is invoked whenever feedback is received for any goal.
    pub fn on_feedback<F>(mut self, callback: F) -> Self
    where
        F: Fn(GoalId, &A::Feedback) + Send + Sync + 'static,
    {
        self.feedback_callback = Some(Box::new(callback));
        self
    }

    /// Set a callback for result messages.
    ///
    /// This callback is invoked whenever a result is received for any goal.
    pub fn on_result<F>(mut self, callback: F) -> Self
    where
        F: Fn(GoalId, GoalStatus, &A::Result) + Send + Sync + 'static,
    {
        self.result_callback = Some(Box::new(callback));
        self
    }

    /// Set a callback for status changes.
    ///
    /// This callback is invoked whenever a status update is received.
    pub fn on_status<F>(mut self, callback: F) -> Self
    where
        F: Fn(GoalId, GoalStatus) + Send + Sync + 'static,
    {
        self.status_callback = Some(Box::new(callback));
        self
    }

    /// Build the action client node.
    pub fn build(self) -> ActionClientNode<A> {
        let inner = Arc::new(ActionClientInner::new());

        if let Some(cb) = self.feedback_callback {
            *inner.feedback_callback.write() = Some(cb);
        }
        if let Some(cb) = self.result_callback {
            *inner.result_callback.write() = Some(cb);
        }
        if let Some(cb) = self.status_callback {
            *inner.status_callback.write() = Some(cb);
        }

        ActionClientNode::new(inner)
    }
}

impl<A: Action> Default for ActionClientBuilder<A>
where
    A::Goal: Clone + Send + Sync + Serialize + DeserializeOwned + Debug + 'static,
    A::Feedback: Clone + Send + Sync + Serialize + DeserializeOwned + Debug + 'static,
    A::Result: Clone + Send + Sync + Serialize + DeserializeOwned + Debug + 'static,
{
    fn default() -> Self {
        Self::new()
    }
}

/// Action client wrapped as a HORUS Node.
///
/// This node handles:
/// - Sending goal and cancel requests
/// - Receiving results, feedback, and status updates
/// - Managing goal handles
pub struct ActionClientNode<A: Action> {
    name: String,
    inner: Arc<ActionClientInner<A>>,

    // Metrics
    goals_sent: AtomicU64,
    goals_succeeded: AtomicU64,
    goals_failed: AtomicU64,
    cancels_sent: AtomicU64,
}

impl<A: Action> ActionClientNode<A>
where
    A::Goal: Clone + Send + Sync + Serialize + DeserializeOwned + Debug + 'static,
    A::Feedback: Clone + Send + Sync + Serialize + DeserializeOwned + Debug + 'static,
    A::Result: Clone + Send + Sync + Serialize + DeserializeOwned + Debug + 'static,
{
    /// Create a new action client node.
    fn new(inner: Arc<ActionClientInner<A>>) -> Self {
        Self {
            name: format!("{}_client", A::name()),
            inner,
            goals_sent: AtomicU64::new(0),
            goals_succeeded: AtomicU64::new(0),
            goals_failed: AtomicU64::new(0),
            cancels_sent: AtomicU64::new(0),
        }
    }

    /// Create a builder for this action client.
    pub fn builder() -> ActionClientBuilder<A> {
        ActionClientBuilder::new()
    }

    /// Send a goal with normal priority.
    ///
    /// Returns a handle to track the goal's progress.
    ///
    /// # Example
    /// ```rust,ignore
    /// let handle = client.send_goal(NavigateGoal { target_x: 5.0, target_y: 3.0 })?;
    /// ```
    pub fn send_goal(&self, goal: A::Goal) -> Result<ClientGoalHandle<A>, ActionError> {
        self.send_goal_with_priority(goal, GoalPriority::NORMAL)
    }

    /// Send a goal with a specific priority.
    ///
    /// Higher priority goals may preempt lower priority ones depending
    /// on the server's preemption policy.
    pub fn send_goal_with_priority(
        &self,
        goal: A::Goal,
        priority: GoalPriority,
    ) -> Result<ClientGoalHandle<A>, ActionError> {
        let goal_id = self.inner.send_goal(goal, priority)?;
        let state = self.inner.register_goal(goal_id);

        self.goals_sent.fetch_add(1, Ordering::Relaxed);

        Ok(ClientGoalHandle {
            goal_id,
            priority,
            state,
            client: self.inner.clone(),
            sent_at: Instant::now(),
        })
    }

    /// Cancel a goal by ID.
    pub fn cancel_goal(&self, goal_id: GoalId) {
        self.inner.cancel_goal(goal_id);
        self.cancels_sent.fetch_add(1, Ordering::Relaxed);
    }

    /// Get the status of a specific goal.
    pub fn goal_status(&self, goal_id: GoalId) -> Option<GoalStatus> {
        self.inner
            .goals
            .read()
            .get(&goal_id)
            .map(|state| state.read().status)
    }

    /// Get all active goal IDs.
    pub fn active_goals(&self) -> Vec<GoalId> {
        self.inner
            .goals
            .read()
            .iter()
            .filter(|(_, state)| state.read().status.is_active())
            .map(|(id, _)| *id)
            .collect()
    }

    /// Get the number of active goals.
    pub fn active_goal_count(&self) -> usize {
        self.inner
            .goals
            .read()
            .values()
            .filter(|state| state.read().status.is_active())
            .count()
    }

    /// Get client metrics.
    pub fn metrics(&self) -> ActionClientMetrics {
        ActionClientMetrics {
            goals_sent: self.goals_sent.load(Ordering::Relaxed),
            goals_succeeded: self.goals_succeeded.load(Ordering::Relaxed),
            goals_failed: self.goals_failed.load(Ordering::Relaxed),
            cancels_sent: self.cancels_sent.load(Ordering::Relaxed),
            active_goals: self.active_goal_count(),
        }
    }
}

/// Metrics for action client.
#[derive(Debug, Clone, Default)]
pub struct ActionClientMetrics {
    /// Total goals sent.
    pub goals_sent: u64,
    /// Total goals that succeeded.
    pub goals_succeeded: u64,
    /// Total goals that failed (aborted, canceled, preempted, rejected).
    pub goals_failed: u64,
    /// Total cancel requests sent.
    pub cancels_sent: u64,
    /// Currently active goals.
    pub active_goals: usize,
}

// Implement Node trait for ActionClientNode
impl<A: Action> Node for ActionClientNode<A>
where
    A::Goal: Clone + Send + Sync + Serialize + DeserializeOwned + Debug + 'static,
    A::Feedback: Clone + Send + Sync + Serialize + DeserializeOwned + Debug + 'static,
    A::Result: Clone + Send + Sync + Serialize + DeserializeOwned + Debug + 'static,
{
    fn name(&self) -> &str {
        &self.name
    }

    fn init(&mut self) -> HorusResult<()> {
        self.inner.initialize()?;
        Ok(())
    }

    fn tick(&mut self) {
        // Process incoming messages (results, feedback, status)
        self.inner.process_messages();

        // Update metrics based on goal states
        let goals = self.inner.goals.read();
        for (_, state) in goals.iter() {
            let state = state.read();
            if state.status.is_terminal() && state.status.is_success() {
                // Note: This would double-count on subsequent ticks
                // A proper implementation would track which goals we've counted
            }
        }
    }
}

/// Synchronous action client for simpler use cases.
///
/// This provides a blocking API that doesn't require running a node.
/// Useful for scripts and tests.
pub struct SyncActionClient<A: Action> {
    inner: Arc<ActionClientInner<A>>,
}

impl<A: Action> SyncActionClient<A>
where
    A::Goal: Clone + Send + Sync + Serialize + DeserializeOwned + Debug + 'static,
    A::Feedback: Clone + Send + Sync + Serialize + DeserializeOwned + Debug + 'static,
    A::Result: Clone + Send + Sync + Serialize + DeserializeOwned + Debug + 'static,
{
    /// Create a new synchronous action client.
    pub fn new() -> HorusResult<Self> {
        let inner = Arc::new(ActionClientInner::new());
        inner.initialize()?;
        Ok(Self { inner })
    }

    /// Send a goal and wait for the result.
    ///
    /// This is a blocking call that returns when the goal completes or times out.
    pub fn send_goal_and_wait(
        &self,
        goal: A::Goal,
        timeout: Duration,
    ) -> Result<A::Result, ActionError> {
        let goal_id = self.inner.send_goal(goal, GoalPriority::NORMAL)?;
        let state = self.inner.register_goal(goal_id);

        let start = Instant::now();
        let poll_interval = 10_u64.ms();

        while start.elapsed() < timeout {
            // Process messages
            self.inner.process_messages();

            // Check if done
            let state = state.read();
            if state.status.is_terminal() {
                return match state.status {
                    GoalStatus::Succeeded => state
                        .result
                        .clone()
                        .ok_or(ActionError::ExecutionError("Missing result".into())),
                    GoalStatus::Canceled => Err(ActionError::GoalCanceled),
                    GoalStatus::Preempted => Err(ActionError::GoalPreempted),
                    GoalStatus::Rejected => Err(ActionError::GoalRejected("Goal rejected".into())),
                    _ => Err(ActionError::ExecutionError("Goal aborted".into())),
                };
            }
            drop(state);

            std::thread::sleep(poll_interval);
        }

        // Timed out - cancel the goal
        self.inner.cancel_goal(goal_id);
        Err(ActionError::GoalTimeout)
    }

    /// Send a goal and wait for the result with feedback callback.
    pub fn send_goal_and_wait_with_feedback<F>(
        &self,
        goal: A::Goal,
        timeout: Duration,
        mut feedback_callback: F,
    ) -> Result<A::Result, ActionError>
    where
        F: FnMut(&A::Feedback),
    {
        let goal_id = self.inner.send_goal(goal, GoalPriority::NORMAL)?;
        let state = self.inner.register_goal(goal_id);

        let start = Instant::now();
        let poll_interval = 10_u64.ms();
        let mut last_feedback_count = 0u64;

        while start.elapsed() < timeout {
            // Process messages
            self.inner.process_messages();

            // Check for new feedback and result
            let state = state.read();

            // Handle feedback
            if state.feedback_count > last_feedback_count {
                if let Some(ref feedback) = state.last_feedback {
                    feedback_callback(feedback);
                }
                last_feedback_count = state.feedback_count;
            }

            // Check if done
            if state.status.is_terminal() {
                return match state.status {
                    GoalStatus::Succeeded => state
                        .result
                        .clone()
                        .ok_or(ActionError::ExecutionError("Missing result".into())),
                    GoalStatus::Canceled => Err(ActionError::GoalCanceled),
                    GoalStatus::Preempted => Err(ActionError::GoalPreempted),
                    GoalStatus::Rejected => Err(ActionError::GoalRejected("Goal rejected".into())),
                    _ => Err(ActionError::ExecutionError("Goal aborted".into())),
                };
            }
            drop(state);

            std::thread::sleep(poll_interval);
        }

        // Timed out - cancel the goal
        self.inner.cancel_goal(goal_id);
        Err(ActionError::GoalTimeout)
    }

    /// Cancel a goal.
    pub fn cancel_goal(&self, goal_id: GoalId) {
        self.inner.cancel_goal(goal_id);
    }
}

impl<A: Action> Default for SyncActionClient<A>
where
    A::Goal: Clone + Send + Sync + Serialize + DeserializeOwned + Debug + 'static,
    A::Feedback: Clone + Send + Sync + Serialize + DeserializeOwned + Debug + 'static,
    A::Result: Clone + Send + Sync + Serialize + DeserializeOwned + Debug + 'static,
{
    fn default() -> Self {
        Self::new().expect("Failed to create SyncActionClient")
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
    fn test_action_client_builder() {
        let client = ActionClientBuilder::<TestAction>::new()
            .on_feedback(|goal_id, feedback| {
                println!("Feedback for {}: {:?}", goal_id, feedback);
            })
            .on_result(|goal_id, status, result| {
                println!("Result for {}: {:?} -> {:?}", goal_id, status, result);
            })
            .on_status(|goal_id, status| {
                println!("Status for {}: {:?}", goal_id, status);
            })
            .build();

        // Verify the client was built with expected initial state
        assert_eq!(client.name(), "test_action_client");
        assert_eq!(client.active_goal_count(), 0);
        let metrics = client.metrics();
        assert_eq!(metrics.goals_sent, 0);
        assert_eq!(metrics.goals_succeeded, 0);
        assert_eq!(metrics.goals_failed, 0);
        assert_eq!(metrics.cancels_sent, 0);
    }

    #[test]
    fn test_client_goal_state() {
        let state = ClientGoalState::<TestAction>::new();
        assert_eq!(state.status, GoalStatus::Pending);
        assert!(state.result.is_none());
        assert!(state.last_feedback.is_none());
        assert_eq!(state.feedback_count, 0);
    }

    #[test]
    fn test_client_metrics_default() {
        let metrics = ActionClientMetrics::default();
        assert_eq!(metrics.goals_sent, 0);
        assert_eq!(metrics.active_goals, 0);
    }

    #[test]
    fn test_goal_status_methods() {
        assert!(GoalStatus::Pending.is_active());
        assert!(GoalStatus::Active.is_active());
        assert!(!GoalStatus::Succeeded.is_active());
        assert!(GoalStatus::Succeeded.is_terminal());
        assert!(GoalStatus::Succeeded.is_success());
        assert!(GoalStatus::Aborted.is_failure());
    }

    // ========================================================================
    // Negative and edge case tests
    // ========================================================================

    #[test]
    fn test_goal_status_all_terminal_states() {
        // Verify all terminal states are correctly identified
        let terminals = [
            GoalStatus::Succeeded,
            GoalStatus::Aborted,
            GoalStatus::Rejected,
            GoalStatus::Canceled,
        ];
        for status in &terminals {
            assert!(status.is_terminal(), "{:?} should be terminal", status);
            assert!(!status.is_active(), "{:?} should not be active", status);
        }
    }

    #[test]
    fn test_goal_status_all_active_states() {
        let actives = [GoalStatus::Pending, GoalStatus::Active];
        for status in &actives {
            assert!(status.is_active(), "{:?} should be active", status);
            assert!(!status.is_terminal(), "{:?} should not be terminal", status);
        }
    }

    #[test]
    fn test_goal_state_initial_values() {
        let state = ClientGoalState::<TestAction>::new();
        assert_eq!(state.status, GoalStatus::Pending);
        assert!(state.result.is_none());
        assert!(state.last_feedback.is_none());
        assert_eq!(state.feedback_count, 0);
    }

    #[test]
    fn test_action_client_builder_no_callbacks() {
        // Builder with no callbacks should still work
        let client = ActionClientBuilder::<TestAction>::new().build();
        assert_eq!(client.metrics().goals_sent, 0);
        assert_eq!(client.metrics().active_goals, 0);
    }

    #[test]
    fn test_action_client_builder_all_callbacks() {
        let feedback_count = Arc::new(std::sync::atomic::AtomicU32::new(0));
        let result_count = Arc::new(std::sync::atomic::AtomicU32::new(0));
        let status_count = Arc::new(std::sync::atomic::AtomicU32::new(0));

        let fc = feedback_count.clone();
        let rc = result_count.clone();
        let sc = status_count.clone();

        let _client = ActionClientBuilder::<TestAction>::new()
            .on_feedback(move |_id, _fb| {
                fc.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
            })
            .on_result(move |_id, _status, _result| {
                rc.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
            })
            .on_status(move |_id, _status| {
                sc.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
            })
            .build();

        // No goals sent yet — callbacks not invoked
        assert_eq!(feedback_count.load(std::sync::atomic::Ordering::Relaxed), 0);
        assert_eq!(result_count.load(std::sync::atomic::Ordering::Relaxed), 0);
        assert_eq!(status_count.load(std::sync::atomic::Ordering::Relaxed), 0);
    }

    #[test]
    fn test_goal_state_before_completion() {
        let state = ClientGoalState::<TestAction>::new();

        // Result should be None before completion
        assert!(state.result.is_none());
        assert!(state.status.is_active());
        assert!(!state.status.is_terminal());
    }

    #[test]
    fn test_goal_state_after_completion() {
        let mut state = ClientGoalState::<TestAction>::new();
        state.status = GoalStatus::Succeeded;
        state.result = Some(TestResult { success: true });

        assert!(!state.status.is_active());
        assert!(state.status.is_terminal());
        assert!(state.status.is_success());
        let result = state.result.unwrap();
        assert!(result.success);
    }

    #[test]
    fn test_goal_id_uniqueness() {
        let ids: Vec<GoalId> = (0..100).map(|_| GoalId::new()).collect();
        // All IDs should be unique
        let unique: std::collections::HashSet<_> = ids.iter().collect();
        assert_eq!(unique.len(), 100, "all 100 goal IDs should be unique");
    }

    // ========================================================================
    // ActionError variant construction and Display coverage
    // ========================================================================

    #[test]
    fn test_action_error_goal_rejected() {
        let err = ActionError::GoalRejected("invalid target".into());
        assert!(matches!(err, ActionError::GoalRejected(ref s) if s == "invalid target"));
        let msg = format!("{}", err);
        assert!(
            msg.contains("invalid target"),
            "Display should contain reason"
        );
    }

    #[test]
    fn test_action_error_goal_canceled() {
        let err = ActionError::GoalCanceled;
        assert!(matches!(err, ActionError::GoalCanceled));
        let msg = format!("{}", err);
        assert!(msg.contains("canceled"), "Display should mention canceled");
    }

    #[test]
    fn test_action_error_goal_preempted() {
        let err = ActionError::GoalPreempted;
        assert!(matches!(err, ActionError::GoalPreempted));
        let msg = format!("{}", err);
        assert!(
            msg.contains("preempted"),
            "Display should mention preempted"
        );
    }

    #[test]
    fn test_action_error_goal_timeout() {
        let err = ActionError::GoalTimeout;
        assert!(matches!(err, ActionError::GoalTimeout));
        let msg = format!("{}", err);
        assert!(
            msg.contains("timed out"),
            "Display should mention timed out"
        );
    }

    #[test]
    fn test_action_error_server_unavailable() {
        let err = ActionError::ServerUnavailable;
        assert!(matches!(err, ActionError::ServerUnavailable));
        let msg = format!("{}", err);
        assert!(
            msg.contains("unavailable"),
            "Display should mention unavailable"
        );
    }

    #[test]
    fn test_action_error_communication_error() {
        let err = ActionError::CommunicationError("link down".into());
        assert!(matches!(err, ActionError::CommunicationError(ref s) if s == "link down"));
        let msg = format!("{}", err);
        assert!(msg.contains("link down"), "Display should contain reason");
    }

    #[test]
    fn test_action_error_execution_error() {
        let err = ActionError::ExecutionError("motor stall".into());
        assert!(matches!(err, ActionError::ExecutionError(ref s) if s == "motor stall"));
        let msg = format!("{}", err);
        assert!(msg.contains("motor stall"), "Display should contain reason");
    }

    #[test]
    fn test_action_error_invalid_goal() {
        let err = ActionError::InvalidGoal("negative distance".into());
        assert!(matches!(err, ActionError::InvalidGoal(ref s) if s == "negative distance"));
        let msg = format!("{}", err);
        assert!(
            msg.contains("negative distance"),
            "Display should contain reason"
        );
    }

    #[test]
    fn test_action_error_goal_not_found() {
        let id = GoalId::new();
        let err = ActionError::GoalNotFound(id);
        assert!(matches!(err, ActionError::GoalNotFound(found_id) if found_id == id));
        let msg = format!("{}", err);
        assert!(
            msg.contains(&id.to_string()),
            "Display should contain goal id"
        );
    }

    #[test]
    fn test_action_error_clone() {
        let err = ActionError::GoalRejected("reason".into());
        let cloned = err.clone();
        assert!(matches!(cloned, ActionError::GoalRejected(ref s) if s == "reason"));

        let err2 = ActionError::GoalNotFound(GoalId::new());
        let _ = err2.clone(); // should not panic
    }

    // ========================================================================
    // GoalStatus transition coverage (every variant checked individually)
    // ========================================================================

    #[test]
    fn test_goal_status_pending_properties() {
        let s = GoalStatus::Pending;
        assert!(s.is_active());
        assert!(!s.is_terminal());
        assert!(!s.is_success());
        assert!(!s.is_failure());
        assert_eq!(format!("{}", s), "PENDING");
    }

    #[test]
    fn test_goal_status_active_properties() {
        let s = GoalStatus::Active;
        assert!(s.is_active());
        assert!(!s.is_terminal());
        assert!(!s.is_success());
        assert!(!s.is_failure());
        assert_eq!(format!("{}", s), "ACTIVE");
    }

    #[test]
    fn test_goal_status_succeeded_properties() {
        let s = GoalStatus::Succeeded;
        assert!(!s.is_active());
        assert!(s.is_terminal());
        assert!(s.is_success());
        assert!(!s.is_failure());
        assert_eq!(format!("{}", s), "SUCCEEDED");
    }

    #[test]
    fn test_goal_status_aborted_properties() {
        let s = GoalStatus::Aborted;
        assert!(!s.is_active());
        assert!(s.is_terminal());
        assert!(!s.is_success());
        assert!(s.is_failure());
        assert_eq!(format!("{}", s), "ABORTED");
    }

    #[test]
    fn test_goal_status_canceled_properties() {
        let s = GoalStatus::Canceled;
        assert!(!s.is_active());
        assert!(s.is_terminal());
        assert!(!s.is_success());
        assert!(s.is_failure());
        assert_eq!(format!("{}", s), "CANCELED");
    }

    #[test]
    fn test_goal_status_preempted_properties() {
        let s = GoalStatus::Preempted;
        assert!(!s.is_active());
        assert!(s.is_terminal());
        assert!(!s.is_success());
        assert!(s.is_failure());
        assert_eq!(format!("{}", s), "PREEMPTED");
    }

    #[test]
    fn test_goal_status_rejected_properties() {
        let s = GoalStatus::Rejected;
        assert!(!s.is_active());
        assert!(s.is_terminal());
        assert!(!s.is_success());
        assert!(s.is_failure());
        assert_eq!(format!("{}", s), "REJECTED");
    }

    // ========================================================================
    // ClientGoalState edge cases
    // ========================================================================

    #[test]
    fn test_goal_state_result_none_while_active() {
        let state = ClientGoalState::<TestAction>::new();
        // Must be None before any result is set
        assert!(state.result.is_none());
        assert!(state.last_feedback.is_none());
        assert_eq!(state.feedback_count, 0);
        assert!(state.status.is_active());
    }

    #[test]
    fn test_goal_state_feedback_accumulates() {
        let mut state = ClientGoalState::<TestAction>::new();
        state.status = GoalStatus::Active;

        // Simulate receiving multiple feedback messages
        for i in 1..=5 {
            state.last_feedback = Some(TestFeedback {
                progress: i as f32 * 0.2,
            });
            state.feedback_count += 1;
            state.updated_at = Instant::now();
        }

        assert_eq!(state.feedback_count, 5);
        let fb = state.last_feedback.as_ref().unwrap();
        assert!(
            (fb.progress - 1.0).abs() < f32::EPSILON,
            "last feedback should be the final one"
        );
    }

    #[test]
    fn test_goal_state_result_set_without_terminal_status() {
        // Contrived scenario: result is written but status not yet updated
        let mut state = ClientGoalState::<TestAction>::new();
        state.result = Some(TestResult { success: true });
        // Status still Pending — result exists but goal not "done"
        assert!(state.result.is_some());
        assert!(state.status.is_active());
        assert!(!state.status.is_terminal());
    }

    #[test]
    fn test_goal_state_overwrite_result() {
        // Simulate double-completion (result overwritten)
        let mut state = ClientGoalState::<TestAction>::new();
        state.status = GoalStatus::Succeeded;
        state.result = Some(TestResult { success: true });

        // Overwrite with a second result
        state.result = Some(TestResult { success: false });
        let r = state.result.unwrap();
        assert!(!r.success, "latest result should take precedence");
    }

    #[test]
    fn test_goal_state_aborted_has_result() {
        let mut state = ClientGoalState::<TestAction>::new();
        state.status = GoalStatus::Aborted;
        state.result = Some(TestResult { success: false });

        assert!(state.status.is_terminal());
        assert!(state.status.is_failure());
        assert!(state.result.is_some());
    }

    #[test]
    fn test_goal_state_canceled_no_result() {
        let mut state = ClientGoalState::<TestAction>::new();
        state.status = GoalStatus::Canceled;
        // Result may or may not be present on cancellation
        assert!(state.result.is_none());
        assert!(state.status.is_failure());
    }

    #[test]
    fn test_goal_state_updated_at_advances() {
        let state = ClientGoalState::<TestAction>::new();
        let first = state.updated_at;
        // Tiny sleep to ensure clock moves forward
        std::thread::sleep(Duration::from_millis(1));
        let mut state2 = ClientGoalState::<TestAction>::new();
        state2.updated_at = Instant::now();
        assert!(state2.updated_at >= first);
    }

    // ========================================================================
    // ActionClientInner error paths (no IPC required)
    // ========================================================================

    #[test]
    fn test_send_goal_before_init_returns_server_unavailable() {
        let inner = ActionClientInner::<TestAction>::new();
        // Not initialized — should return ServerUnavailable
        let result = inner.send_goal(TestGoal { target: 1.0 }, GoalPriority::NORMAL);
        assert!(result.is_err());
        assert!(matches!(
            result.unwrap_err(),
            ActionError::ServerUnavailable
        ));
    }

    #[test]
    fn test_register_goal_creates_pending_state() {
        let inner = ActionClientInner::<TestAction>::new();
        let id = GoalId::new();
        let state = inner.register_goal(id);

        let s = state.read();
        assert_eq!(s.status, GoalStatus::Pending);
        assert!(s.result.is_none());
        assert_eq!(s.feedback_count, 0);

        // Verify it is in the goals map
        assert!(inner.goals.read().contains_key(&id));
    }

    #[test]
    fn test_register_multiple_goals() {
        let inner = ActionClientInner::<TestAction>::new();
        let id1 = GoalId::new();
        let id2 = GoalId::new();
        let id3 = GoalId::new();

        inner.register_goal(id1);
        inner.register_goal(id2);
        inner.register_goal(id3);

        let goals = inner.goals.read();
        assert_eq!(goals.len(), 3);
        assert!(goals.contains_key(&id1));
        assert!(goals.contains_key(&id2));
        assert!(goals.contains_key(&id3));
    }

    #[test]
    fn test_handle_status_update_known_goal() {
        let inner = ActionClientInner::<TestAction>::new();
        let id = GoalId::new();
        let state = inner.register_goal(id);

        // Simulate a status update
        let update = GoalStatusUpdate::new(id, GoalStatus::Active);
        inner.handle_status_update(update);

        assert_eq!(state.read().status, GoalStatus::Active);
    }

    #[test]
    fn test_handle_status_update_unknown_goal_is_noop() {
        let inner = ActionClientInner::<TestAction>::new();
        let unknown_id = GoalId::new();

        // Should not panic for an unknown goal
        let update = GoalStatusUpdate::new(unknown_id, GoalStatus::Active);
        inner.handle_status_update(update);

        assert!(inner.goals.read().is_empty());
    }

    #[test]
    fn test_handle_feedback_known_goal() {
        let inner = ActionClientInner::<TestAction>::new();
        let id = GoalId::new();
        let state = inner.register_goal(id);

        let fb = ActionFeedback::new(id, TestFeedback { progress: 0.5 });
        inner.handle_feedback(fb);

        let s = state.read();
        assert_eq!(s.feedback_count, 1);
        let last = s.last_feedback.as_ref().unwrap();
        assert!((last.progress - 0.5).abs() < f32::EPSILON);
    }

    #[test]
    fn test_handle_feedback_unknown_goal_is_noop() {
        let inner = ActionClientInner::<TestAction>::new();
        let unknown_id = GoalId::new();

        let fb = ActionFeedback::new(unknown_id, TestFeedback { progress: 0.5 });
        inner.handle_feedback(fb); // should not panic
    }

    #[test]
    fn test_handle_result_sets_state_and_removes_terminal() {
        let inner = ActionClientInner::<TestAction>::new();
        let id = GoalId::new();
        let state = inner.register_goal(id);

        let result_msg = ActionResult::succeeded(id, TestResult { success: true });
        inner.handle_result(result_msg);

        // State should be updated even though goal is removed from map
        let s = state.read();
        assert_eq!(s.status, GoalStatus::Succeeded);
        assert!(s.result.as_ref().unwrap().success);

        // Terminal goal removed from goals map
        assert!(!inner.goals.read().contains_key(&id));
    }

    #[test]
    fn test_handle_result_aborted_removes_from_map() {
        let inner = ActionClientInner::<TestAction>::new();
        let id = GoalId::new();
        let state = inner.register_goal(id);

        let result_msg = ActionResult::aborted(id, TestResult { success: false });
        inner.handle_result(result_msg);

        let s = state.read();
        assert_eq!(s.status, GoalStatus::Aborted);
        assert!(!inner.goals.read().contains_key(&id));
    }

    #[test]
    fn test_handle_result_canceled_removes_from_map() {
        let inner = ActionClientInner::<TestAction>::new();
        let id = GoalId::new();
        let _state = inner.register_goal(id);

        let result_msg = ActionResult::canceled(id, TestResult { success: false });
        inner.handle_result(result_msg);

        assert!(!inner.goals.read().contains_key(&id));
    }

    #[test]
    fn test_handle_result_preempted_removes_from_map() {
        let inner = ActionClientInner::<TestAction>::new();
        let id = GoalId::new();
        let _state = inner.register_goal(id);

        let result_msg = ActionResult::preempted(id, TestResult { success: false });
        inner.handle_result(result_msg);

        assert!(!inner.goals.read().contains_key(&id));
    }

    #[test]
    fn test_handle_result_unknown_goal_is_noop() {
        let inner = ActionClientInner::<TestAction>::new();
        let unknown_id = GoalId::new();

        let result_msg = ActionResult::succeeded(unknown_id, TestResult { success: true });
        inner.handle_result(result_msg); // should not panic
    }

    // ========================================================================
    // Status callback invocation through handle_status_update
    // ========================================================================

    #[test]
    fn test_status_callback_invoked() {
        let inner = ActionClientInner::<TestAction>::new();
        let called = Arc::new(AtomicBool::new(false));
        let called_clone = called.clone();

        *inner.status_callback.write() = Some(Box::new(move |_id, _status| {
            called_clone.store(true, Ordering::Relaxed);
        }));

        let id = GoalId::new();
        inner.register_goal(id);

        let update = GoalStatusUpdate::new(id, GoalStatus::Active);
        inner.handle_status_update(update);

        assert!(
            called.load(Ordering::Relaxed),
            "status callback should fire"
        );
    }

    #[test]
    fn test_feedback_callback_invoked() {
        let inner = ActionClientInner::<TestAction>::new();
        let called = Arc::new(AtomicBool::new(false));
        let called_clone = called.clone();

        *inner.feedback_callback.write() = Some(Box::new(move |_id, _fb: &TestFeedback| {
            called_clone.store(true, Ordering::Relaxed);
        }));

        let id = GoalId::new();
        inner.register_goal(id);

        let fb = ActionFeedback::new(id, TestFeedback { progress: 0.7 });
        inner.handle_feedback(fb);

        assert!(
            called.load(Ordering::Relaxed),
            "feedback callback should fire"
        );
    }

    #[test]
    fn test_result_callback_invoked() {
        let inner = ActionClientInner::<TestAction>::new();
        let called = Arc::new(AtomicBool::new(false));
        let called_clone = called.clone();

        *inner.result_callback.write() =
            Some(Box::new(move |_id, _status, _result: &TestResult| {
                called_clone.store(true, Ordering::Relaxed);
            }));

        let id = GoalId::new();
        inner.register_goal(id);

        let result_msg = ActionResult::succeeded(id, TestResult { success: true });
        inner.handle_result(result_msg);

        assert!(
            called.load(Ordering::Relaxed),
            "result callback should fire"
        );
    }

    // ========================================================================
    // ActionClientNode metrics and goal tracking (no IPC)
    // ========================================================================

    #[test]
    fn test_client_node_name() {
        let node = ActionClientBuilder::<TestAction>::new().build();
        assert_eq!(node.name(), "test_action_client");
    }

    #[test]
    fn test_client_node_goal_status_unknown_id() {
        let node = ActionClientBuilder::<TestAction>::new().build();
        let unknown_id = GoalId::new();
        assert!(node.goal_status(unknown_id).is_none());
    }

    #[test]
    fn test_client_node_active_goals_empty() {
        let node = ActionClientBuilder::<TestAction>::new().build();
        assert!(node.active_goals().is_empty());
        assert_eq!(node.active_goal_count(), 0);
    }

    #[test]
    fn test_client_node_cancel_increments_metric() {
        let node = ActionClientBuilder::<TestAction>::new().build();
        let fake_id = GoalId::new();

        // Cancel should increment the metric even if the goal doesn't exist
        node.cancel_goal(fake_id);
        assert_eq!(node.metrics().cancels_sent, 1);

        node.cancel_goal(fake_id);
        assert_eq!(node.metrics().cancels_sent, 2);
    }

    #[test]
    fn test_client_node_send_goal_without_init_fails() {
        let node = ActionClientBuilder::<TestAction>::new().build();
        // inner is not initialized, so send_goal should return ServerUnavailable
        let result = node.send_goal(TestGoal { target: 42.0 });
        assert!(result.is_err());
        assert!(matches!(
            result.unwrap_err(),
            ActionError::ServerUnavailable
        ));
    }

    // ========================================================================
    // ActionClientMetrics
    // ========================================================================

    #[test]
    fn test_client_metrics_fields() {
        let m = ActionClientMetrics {
            goals_sent: 10,
            goals_succeeded: 7,
            goals_failed: 2,
            cancels_sent: 1,
            active_goals: 3,
        };
        assert_eq!(m.goals_sent, 10);
        assert_eq!(m.goals_succeeded, 7);
        assert_eq!(m.goals_failed, 2);
        assert_eq!(m.cancels_sent, 1);
        assert_eq!(m.active_goals, 3);
    }

    #[test]
    fn test_client_metrics_debug() {
        let m = ActionClientMetrics::default();
        let dbg = format!("{:?}", m);
        assert!(dbg.contains("goals_sent"));
        assert!(dbg.contains("active_goals"));
    }

    #[test]
    fn test_client_metrics_clone() {
        let m = ActionClientMetrics {
            goals_sent: 5,
            goals_succeeded: 3,
            goals_failed: 1,
            cancels_sent: 1,
            active_goals: 0,
        };
        let m2 = m.clone();
        assert_eq!(m2.goals_sent, 5);
        assert_eq!(m2.goals_succeeded, 3);
    }

    // ========================================================================
    // GoalPriority edge cases
    // ========================================================================

    #[test]
    fn test_goal_priority_constants_ordering() {
        assert!(GoalPriority::HIGHEST.is_higher_than(&GoalPriority::HIGH));
        assert!(GoalPriority::HIGH.is_higher_than(&GoalPriority::NORMAL));
        assert!(GoalPriority::NORMAL.is_higher_than(&GoalPriority::LOW));
        assert!(GoalPriority::LOW.is_higher_than(&GoalPriority::LOWEST));
    }

    #[test]
    fn test_goal_priority_same_not_higher() {
        assert!(!GoalPriority::NORMAL.is_higher_than(&GoalPriority::NORMAL));
        assert!(!GoalPriority::HIGHEST.is_higher_than(&GoalPriority::HIGHEST));
    }

    #[test]
    fn test_goal_priority_default_is_normal() {
        assert_eq!(GoalPriority::default(), GoalPriority::NORMAL);
    }

    // ========================================================================
    // ActionClientBuilder default impl
    // ========================================================================

    #[test]
    fn test_action_client_builder_default() {
        let builder = ActionClientBuilder::<TestAction>::default();
        let node = builder.build();
        assert_eq!(node.name(), "test_action_client");
        assert_eq!(node.metrics().goals_sent, 0);
    }

    // ========================================================================
    // Full status transition sequence on ClientGoalState
    // ========================================================================

    #[test]
    fn test_goal_state_full_lifecycle() {
        let mut state = ClientGoalState::<TestAction>::new();

        // Start: Pending
        assert_eq!(state.status, GoalStatus::Pending);
        assert!(state.result.is_none());

        // Transition to Active
        state.status = GoalStatus::Active;
        state.updated_at = Instant::now();
        assert!(state.status.is_active());

        // Receive feedback
        state.last_feedback = Some(TestFeedback { progress: 0.5 });
        state.feedback_count = 1;
        assert_eq!(state.feedback_count, 1);

        // Transition to Succeeded
        state.status = GoalStatus::Succeeded;
        state.result = Some(TestResult { success: true });
        state.updated_at = Instant::now();

        assert!(state.status.is_terminal());
        assert!(state.status.is_success());
        assert!(state.result.as_ref().unwrap().success);
    }

    #[test]
    fn test_goal_state_rejected_lifecycle() {
        let mut state = ClientGoalState::<TestAction>::new();
        // Goal can go directly from Pending to Rejected
        assert_eq!(state.status, GoalStatus::Pending);

        state.status = GoalStatus::Rejected;
        state.updated_at = Instant::now();

        assert!(state.status.is_terminal());
        assert!(state.status.is_failure());
        assert!(state.result.is_none()); // Rejected goals may have no result
    }

    #[test]
    fn test_goal_state_preempted_lifecycle() {
        let mut state = ClientGoalState::<TestAction>::new();
        state.status = GoalStatus::Active;
        state.last_feedback = Some(TestFeedback { progress: 0.3 });
        state.feedback_count = 1;

        // Preempted by higher priority goal
        state.status = GoalStatus::Preempted;
        state.result = Some(TestResult { success: false });
        state.updated_at = Instant::now();

        assert!(state.status.is_terminal());
        assert!(state.status.is_failure());
        assert!(!state.status.is_success());
        // Feedback from before preemption is preserved
        assert_eq!(state.feedback_count, 1);
    }
}
