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
//! match handle.await_result(Duration::from_secs(30)) {
//!     Some(result) => println!("Navigation complete: {:?}", result),
//!     None => println!("Navigation timed out"),
//! }
//! ```

use crate::actions::types::{
    Action, ActionError, ActionFeedback, ActionResult, CancelRequest, GoalId, GoalPriority,
    GoalRequest, GoalStatus, GoalStatusUpdate,
};
use crate::communication::Topic;
use crate::core::{LogSummary, Node};
use crate::HorusResult;

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

// Basic accessors that don't require LogSummary bounds
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

    /// Get the number of feedback messages received.
    pub fn feedback_count(&self) -> u64 {
        self.state.read().feedback_count
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
    A::Goal: Clone + Send + Sync + Serialize + DeserializeOwned + Debug + LogSummary + 'static,
    A::Feedback: Clone + Send + Sync + Serialize + DeserializeOwned + Debug + LogSummary + 'static,
    A::Result: Clone + Send + Sync + Serialize + DeserializeOwned + Debug + LogSummary + 'static,
{
    /// Get the result if the goal has completed.
    ///
    /// Returns `None` if the goal is still active or no result has been received.
    pub fn get_result(&self) -> Option<A::Result> {
        self.state.read().result.clone()
    }

    /// Get the last feedback received.
    pub fn get_last_feedback(&self) -> Option<A::Feedback> {
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
    /// if let Some(result) = handle.await_result(Duration::from_secs(30)) {
    ///     println!("Got result: {:?}", result);
    /// } else {
    ///     println!("Timed out waiting for result");
    /// }
    /// ```
    pub fn await_result(&self, timeout: Duration) -> Option<A::Result> {
        let start = Instant::now();
        let poll_interval = Duration::from_millis(10);

        while start.elapsed() < timeout {
            if self.is_done() {
                return self.get_result();
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
        let poll_interval = Duration::from_millis(10);
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

    /// Update the goal state (called internally by the client).
    #[allow(dead_code)]
    fn update_status(&self, status: GoalStatus) {
        let mut state = self.state.write();
        state.status = status;
        state.updated_at = Instant::now();
    }

    /// Update with a result (called internally by the client).
    #[allow(dead_code)]
    fn update_result(&self, result: A::Result, status: GoalStatus) {
        let mut state = self.state.write();
        state.result = Some(result);
        state.status = status;
        state.updated_at = Instant::now();
    }

    /// Update with feedback (called internally by the client).
    #[allow(dead_code)]
    fn update_feedback(&self, feedback: A::Feedback) {
        let mut state = self.state.write();
        state.last_feedback = Some(feedback);
        state.feedback_count += 1;
        state.updated_at = Instant::now();
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
    A::Goal: Clone + Send + Sync + Serialize + DeserializeOwned + Debug + LogSummary + 'static,
    A::Feedback: Clone + Send + Sync + Serialize + DeserializeOwned + Debug + LogSummary + 'static,
    A::Result: Clone + Send + Sync + Serialize + DeserializeOwned + Debug + LogSummary + 'static,
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

        // Create links using Topic::new() - auto-detects role on first send/recv
        *self.goal_link.write() = Some(Topic::new(&A::goal_topic())?);
        *self.cancel_link.write() = Some(Topic::new(&A::cancel_topic())?);
        *self.result_link.write() = Some(Topic::new(&A::result_topic())?);
        *self.feedback_link.write() = Some(Topic::new(&A::feedback_topic())?);
        *self.status_link.write() = Some(Topic::new(&A::status_topic())?);

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
            link.send(request)
                .map_err(|_| ActionError::CommunicationError("Failed to send goal".to_string()))?;

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
            let _ = link.send(request);
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
        let goals = self.goals.read();
        if let Some(state) = goals.get(&result_msg.goal_id) {
            let mut state = state.write();
            state.result = Some(result_msg.result.clone());
            state.status = result_msg.status;
            state.updated_at = Instant::now();
        }

        // Call result callback
        if let Some(ref callback) = *self.result_callback.read() {
            callback(result_msg.goal_id, result_msg.status, &result_msg.result);
        }
    }

    /// Register a goal handle.
    fn register_goal(&self, goal_id: GoalId) -> Arc<RwLock<ClientGoalState<A>>> {
        let state = Arc::new(RwLock::new(ClientGoalState::new()));
        self.goals.write().insert(goal_id, state.clone());
        state
    }

    /// Remove a goal handle.
    #[allow(dead_code)]
    fn unregister_goal(&self, goal_id: GoalId) {
        self.goals.write().remove(&goal_id);
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
    A::Goal: Clone + Send + Sync + Serialize + DeserializeOwned + Debug + LogSummary + 'static,
    A::Feedback: Clone + Send + Sync + Serialize + DeserializeOwned + Debug + LogSummary + 'static,
    A::Result: Clone + Send + Sync + Serialize + DeserializeOwned + Debug + LogSummary + 'static,
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
    A::Goal: Clone + Send + Sync + Serialize + DeserializeOwned + Debug + LogSummary + 'static,
    A::Feedback: Clone + Send + Sync + Serialize + DeserializeOwned + Debug + LogSummary + 'static,
    A::Result: Clone + Send + Sync + Serialize + DeserializeOwned + Debug + LogSummary + 'static,
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
    A::Goal: Clone + Send + Sync + Serialize + DeserializeOwned + Debug + LogSummary + 'static,
    A::Feedback: Clone + Send + Sync + Serialize + DeserializeOwned + Debug + LogSummary + 'static,
    A::Result: Clone + Send + Sync + Serialize + DeserializeOwned + Debug + LogSummary + 'static,
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

    /// Cancel all active goals.
    pub fn cancel_all(&self) {
        let goal_ids: Vec<GoalId> = self.inner.goals.read().keys().copied().collect();
        for goal_id in goal_ids {
            self.cancel_goal(goal_id);
        }
    }

    /// Get the status of a specific goal.
    pub fn get_goal_status(&self, goal_id: GoalId) -> Option<GoalStatus> {
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

    /// Remove completed goals from tracking.
    ///
    /// Call this periodically to prevent memory growth from old goal handles.
    pub fn cleanup_completed(&self) {
        let mut goals = self.inner.goals.write();
        goals.retain(|_, state| state.read().status.is_active());
    }

    /// Wait for the server to be available.
    ///
    /// Blocks until the server is detected or timeout.
    pub fn wait_for_server(&self, timeout: Duration) -> bool {
        // For now, just check if initialized
        // In a real implementation, we'd check for server presence
        let start = Instant::now();
        while start.elapsed() < timeout {
            if self.inner.initialized.load(Ordering::Acquire) {
                return true;
            }
            std::thread::sleep(Duration::from_millis(100));
        }
        false
    }

    /// Check if the client is ready to send goals.
    pub fn is_ready(&self) -> bool {
        self.inner.initialized.load(Ordering::Acquire)
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
    A::Goal: Clone + Send + Sync + Serialize + DeserializeOwned + Debug + LogSummary + 'static,
    A::Feedback: Clone + Send + Sync + Serialize + DeserializeOwned + Debug + LogSummary + 'static,
    A::Result: Clone + Send + Sync + Serialize + DeserializeOwned + Debug + LogSummary + 'static,
{
    fn name(&self) -> &'static str {
        Box::leak(self.name.clone().into_boxed_str())
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
    A::Goal: Clone + Send + Sync + Serialize + DeserializeOwned + Debug + LogSummary + 'static,
    A::Feedback: Clone + Send + Sync + Serialize + DeserializeOwned + Debug + LogSummary + 'static,
    A::Result: Clone + Send + Sync + Serialize + DeserializeOwned + Debug + LogSummary + 'static,
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
        let poll_interval = Duration::from_millis(10);

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
        let poll_interval = Duration::from_millis(10);
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
    A::Goal: Clone + Send + Sync + Serialize + DeserializeOwned + Debug + LogSummary + 'static,
    A::Feedback: Clone + Send + Sync + Serialize + DeserializeOwned + Debug + LogSummary + 'static,
    A::Result: Clone + Send + Sync + Serialize + DeserializeOwned + Debug + LogSummary + 'static,
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
    fn test_action_client_builder() {
        let _client = ActionClientBuilder::<TestAction>::new()
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
}
