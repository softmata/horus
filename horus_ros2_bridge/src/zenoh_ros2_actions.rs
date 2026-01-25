//! ROS2 Action Protocol for Zenoh Bridge
//!
//! Implements the ROS2 action client/server pattern over Zenoh.
//!
//! # ROS2 Action Protocol
//!
//! ROS2 actions use the following topic naming convention:
//! - Send Goal: `rq/{action}/_action/send_goal`
//! - Cancel Goal: `rq/{action}/_action/cancel_goal`
//! - Get Result: `rq/{action}/_action/get_result`
//! - Goal Status: `rt/{action}/_action/status`
//! - Feedback: `rt/{action}/_action/feedback`
//!
//! The action messages are encoded using CDR (Common Data Representation)
//! for compatibility with native ROS2 nodes.
//!
//! # Example
//!
//! ```ignore
//! use horus_core::communication::network::zenoh_ros2_actions::*;
//!
//! // Create an action client
//! let client = Ros2ActionClient::<NavigateToPose>::new("navigate_to_pose", config).await?;
//!
//! // Send a goal
//! let goal = NavigateToPoseGoal { pose: target_pose };
//! let goal_handle = client.send_goal(goal).await?;
//!
//! // Wait for result with feedback
//! while let Some(feedback) = goal_handle.get_pending_feedback().await {
//!     println!("Progress: {:?}", feedback);
//! }
//! let result = goal_handle.get_result().await?;
//! ```

use std::collections::HashMap;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::Arc;
use std::time::Duration;

use parking_lot::RwLock;
use serde::{de::DeserializeOwned, Deserialize, Serialize};
use uuid::Uuid;

use horus_core::communication::network::queryable::QueryError;

// ============================================================================
// ROS2 Action Topic Naming
// ============================================================================

/// ROS2 action topic suffixes
pub const ACTION_SEND_GOAL_SUFFIX: &str = "_action/send_goal";
pub const ACTION_CANCEL_GOAL_SUFFIX: &str = "_action/cancel_goal";
pub const ACTION_GET_RESULT_SUFFIX: &str = "_action/get_result";
pub const ACTION_STATUS_SUFFIX: &str = "_action/status";
pub const ACTION_FEEDBACK_SUFFIX: &str = "_action/feedback";

/// Convert an action name to ROS2 send_goal topic
pub fn action_to_send_goal_topic(action_name: &str) -> String {
    format!(
        "rq/{}/{}",
        action_name.trim_start_matches('/'),
        ACTION_SEND_GOAL_SUFFIX
    )
}

/// Convert an action name to ROS2 cancel_goal topic
pub fn action_to_cancel_goal_topic(action_name: &str) -> String {
    format!(
        "rq/{}/{}",
        action_name.trim_start_matches('/'),
        ACTION_CANCEL_GOAL_SUFFIX
    )
}

/// Convert an action name to ROS2 get_result topic
pub fn action_to_get_result_topic(action_name: &str) -> String {
    format!(
        "rq/{}/{}",
        action_name.trim_start_matches('/'),
        ACTION_GET_RESULT_SUFFIX
    )
}

/// Convert an action name to ROS2 status topic
pub fn action_to_status_topic(action_name: &str) -> String {
    format!(
        "rt/{}/{}",
        action_name.trim_start_matches('/'),
        ACTION_STATUS_SUFFIX
    )
}

/// Convert an action name to ROS2 feedback topic
pub fn action_to_feedback_topic(action_name: &str) -> String {
    format!(
        "rt/{}/{}",
        action_name.trim_start_matches('/'),
        ACTION_FEEDBACK_SUFFIX
    )
}

/// Parse a ROS2 action topic to extract action name and topic type
pub fn parse_action_topic(topic: &str) -> Option<(String, ActionTopicType)> {
    let topic = topic.trim_start_matches('/');

    // Check for action suffixes
    for (suffix, topic_type) in [
        (ACTION_SEND_GOAL_SUFFIX, ActionTopicType::SendGoal),
        (ACTION_CANCEL_GOAL_SUFFIX, ActionTopicType::CancelGoal),
        (ACTION_GET_RESULT_SUFFIX, ActionTopicType::GetResult),
        (ACTION_STATUS_SUFFIX, ActionTopicType::Status),
        (ACTION_FEEDBACK_SUFFIX, ActionTopicType::Feedback),
    ] {
        if topic.ends_with(suffix) {
            // Extract action name (remove prefix and suffix)
            let without_suffix = &topic[..topic.len() - suffix.len() - 1]; // -1 for the /
            let action_name = if without_suffix.starts_with("rq/") {
                &without_suffix[3..]
            } else if without_suffix.starts_with("rt/") {
                &without_suffix[3..]
            } else {
                without_suffix
            };
            return Some((action_name.to_string(), topic_type));
        }
    }

    None
}

/// Apply namespace to an action name
pub fn apply_namespace_to_action(action_name: &str, namespace: &str) -> String {
    if namespace.is_empty() {
        action_name.to_string()
    } else {
        format!(
            "{}/{}",
            namespace.trim_end_matches('/'),
            action_name.trim_start_matches('/')
        )
    }
}

// ============================================================================
// Action Topic Types
// ============================================================================

/// Types of action topics
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ActionTopicType {
    /// Send goal request topic (rq)
    SendGoal,
    /// Cancel goal request topic (rq)
    CancelGoal,
    /// Get result request topic (rq)
    GetResult,
    /// Goal status array topic (rt)
    Status,
    /// Feedback topic (rt)
    Feedback,
}

// ============================================================================
// Goal ID (UUID)
// ============================================================================

/// ROS2 Goal ID using UUID
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub struct GoalId {
    pub uuid: [u8; 16],
}

impl GoalId {
    /// Create a new random goal ID
    pub fn new() -> Self {
        Self {
            uuid: *Uuid::new_v4().as_bytes(),
        }
    }

    /// Create from existing UUID bytes
    pub fn from_bytes(bytes: [u8; 16]) -> Self {
        Self { uuid: bytes }
    }

    /// Encode as CDR bytes (16 bytes)
    pub fn encode_cdr(&self) -> Vec<u8> {
        self.uuid.to_vec()
    }

    /// Decode from CDR bytes
    pub fn decode_cdr(data: &[u8]) -> Option<Self> {
        if data.len() < 16 {
            return None;
        }
        let mut uuid = [0u8; 16];
        uuid.copy_from_slice(&data[0..16]);
        Some(Self { uuid })
    }
}

impl Default for GoalId {
    fn default() -> Self {
        Self::new()
    }
}

impl std::fmt::Display for GoalId {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}", Uuid::from_bytes(self.uuid))
    }
}

// ============================================================================
// Goal States (ROS2 action_msgs/GoalStatus)
// ============================================================================

/// ROS2 Goal Status values (from action_msgs/GoalStatus)
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[repr(i8)]
pub enum GoalStatus {
    /// Unknown status (should not be seen in normal operation)
    Unknown = 0,
    /// The goal has been accepted and is awaiting execution
    Accepted = 1,
    /// The goal is currently being executed
    Executing = 2,
    /// The goal was canceled after being accepted
    Canceling = 3,
    /// The goal was achieved successfully
    Succeeded = 4,
    /// The goal was canceled before completion
    Canceled = 5,
    /// The goal was aborted due to a failure
    Aborted = 6,
}

impl GoalStatus {
    /// Check if the goal is in a terminal state
    pub fn is_terminal(&self) -> bool {
        matches!(
            self,
            GoalStatus::Succeeded | GoalStatus::Canceled | GoalStatus::Aborted
        )
    }

    /// Check if the goal is active
    pub fn is_active(&self) -> bool {
        matches!(
            self,
            GoalStatus::Accepted | GoalStatus::Executing | GoalStatus::Canceling
        )
    }

    /// Convert from i8
    pub fn from_i8(value: i8) -> Self {
        match value {
            1 => GoalStatus::Accepted,
            2 => GoalStatus::Executing,
            3 => GoalStatus::Canceling,
            4 => GoalStatus::Succeeded,
            5 => GoalStatus::Canceled,
            6 => GoalStatus::Aborted,
            _ => GoalStatus::Unknown,
        }
    }
}

impl Default for GoalStatus {
    fn default() -> Self {
        GoalStatus::Unknown
    }
}

// ============================================================================
// ROS2 Time (builtin_interfaces/Time)
// ============================================================================

/// ROS2 Time structure (builtin_interfaces/Time)
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Serialize, Deserialize)]
pub struct Ros2Time {
    /// Seconds since epoch
    pub sec: i32,
    /// Nanoseconds (0-999999999)
    pub nanosec: u32,
}

impl Ros2Time {
    /// Create from current system time
    pub fn now() -> Self {
        let now = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap_or_default();
        Self {
            sec: now.as_secs() as i32,
            nanosec: now.subsec_nanos(),
        }
    }

    /// Create from Duration
    pub fn from_duration(d: Duration) -> Self {
        Self {
            sec: d.as_secs() as i32,
            nanosec: d.subsec_nanos(),
        }
    }

    /// Convert to Duration
    pub fn to_duration(&self) -> Duration {
        Duration::new(self.sec.max(0) as u64, self.nanosec)
    }

    /// Encode as CDR bytes (8 bytes: 4 sec + 4 nanosec)
    pub fn encode_cdr(&self) -> Vec<u8> {
        let mut data = Vec::with_capacity(8);
        data.extend_from_slice(&self.sec.to_le_bytes());
        data.extend_from_slice(&self.nanosec.to_le_bytes());
        data
    }

    /// Decode from CDR bytes
    pub fn decode_cdr(data: &[u8]) -> Option<Self> {
        if data.len() < 8 {
            return None;
        }
        let sec = i32::from_le_bytes(data[0..4].try_into().ok()?);
        let nanosec = u32::from_le_bytes(data[4..8].try_into().ok()?);
        Some(Self { sec, nanosec })
    }
}

// ============================================================================
// Goal Info (action_msgs/GoalInfo)
// ============================================================================

/// ROS2 GoalInfo structure (action_msgs/GoalInfo)
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct GoalInfo {
    /// Unique goal identifier
    pub goal_id: GoalId,
    /// Time when the goal was accepted
    pub stamp: Ros2Time,
}

impl GoalInfo {
    /// Create new GoalInfo with current timestamp
    pub fn new() -> Self {
        Self {
            goal_id: GoalId::new(),
            stamp: Ros2Time::now(),
        }
    }

    /// Create with specific goal ID
    pub fn with_id(goal_id: GoalId) -> Self {
        Self {
            goal_id,
            stamp: Ros2Time::now(),
        }
    }

    /// Encode as CDR bytes (24 bytes: 16 uuid + 8 time)
    pub fn encode_cdr(&self) -> Vec<u8> {
        let mut data = Vec::with_capacity(24);
        data.extend(self.goal_id.encode_cdr());
        data.extend(self.stamp.encode_cdr());
        data
    }

    /// Decode from CDR bytes
    pub fn decode_cdr(data: &[u8]) -> Option<Self> {
        if data.len() < 24 {
            return None;
        }
        let goal_id = GoalId::decode_cdr(&data[0..16])?;
        let stamp = Ros2Time::decode_cdr(&data[16..24])?;
        Some(Self { goal_id, stamp })
    }
}

// ============================================================================
// Goal Status Info (action_msgs/GoalStatusArray element)
// ============================================================================

/// Single goal status entry
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct GoalStatusInfo {
    /// Goal information
    pub goal_info: GoalInfo,
    /// Current status
    pub status: GoalStatus,
}

impl GoalStatusInfo {
    /// Create new status info
    pub fn new(goal_id: GoalId, status: GoalStatus) -> Self {
        Self {
            goal_info: GoalInfo::with_id(goal_id),
            status,
        }
    }

    /// Encode as CDR bytes (25 bytes: 24 goal_info + 1 status)
    pub fn encode_cdr(&self) -> Vec<u8> {
        let mut data = self.goal_info.encode_cdr();
        data.push(self.status as i8 as u8);
        data
    }

    /// Decode from CDR bytes
    pub fn decode_cdr(data: &[u8]) -> Option<Self> {
        if data.len() < 25 {
            return None;
        }
        let goal_info = GoalInfo::decode_cdr(&data[0..24])?;
        let status = GoalStatus::from_i8(data[24] as i8);
        Some(Self { goal_info, status })
    }
}

// ============================================================================
// Goal Status Array (action_msgs/GoalStatusArray)
// ============================================================================

/// ROS2 GoalStatusArray message
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct GoalStatusArray {
    /// List of goal status entries
    pub status_list: Vec<GoalStatusInfo>,
}

impl GoalStatusArray {
    /// Create new empty status array
    pub fn new() -> Self {
        Self {
            status_list: Vec::new(),
        }
    }

    /// Add a status entry
    pub fn add(&mut self, goal_id: GoalId, status: GoalStatus) {
        self.status_list.push(GoalStatusInfo::new(goal_id, status));
    }

    /// Encode as CDR bytes
    pub fn encode_cdr(&self) -> Vec<u8> {
        let mut data = Vec::new();
        // Array length (4 bytes)
        data.extend_from_slice(&(self.status_list.len() as u32).to_le_bytes());
        // Array elements
        for status in &self.status_list {
            data.extend(status.encode_cdr());
        }
        data
    }

    /// Decode from CDR bytes
    pub fn decode_cdr(data: &[u8]) -> Option<Self> {
        if data.len() < 4 {
            return None;
        }
        let len = u32::from_le_bytes(data[0..4].try_into().ok()?) as usize;
        let mut status_list = Vec::with_capacity(len);
        let mut offset = 4;
        for _ in 0..len {
            if offset + 25 > data.len() {
                return None;
            }
            let status = GoalStatusInfo::decode_cdr(&data[offset..offset + 25])?;
            status_list.push(status);
            offset += 25;
        }
        Some(Self { status_list })
    }
}

// ============================================================================
// SendGoal Request/Response
// ============================================================================

/// Send Goal Request wrapper
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SendGoalRequest<G> {
    /// Goal ID for this goal
    pub goal_id: GoalId,
    /// The goal data
    pub goal: G,
}

impl<G: Serialize> SendGoalRequest<G> {
    /// Create new request with random goal ID
    pub fn new(goal: G) -> Self {
        Self {
            goal_id: GoalId::new(),
            goal,
        }
    }

    /// Create with specific goal ID
    pub fn with_id(goal_id: GoalId, goal: G) -> Self {
        Self { goal_id, goal }
    }
}

/// Send Goal Response
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SendGoalResponse {
    /// Whether the goal was accepted
    pub accepted: bool,
    /// Timestamp when goal was accepted
    pub stamp: Ros2Time,
}

impl Default for SendGoalResponse {
    fn default() -> Self {
        Self {
            accepted: false,
            stamp: Ros2Time::now(),
        }
    }
}

impl SendGoalResponse {
    /// Create accepted response
    pub fn accepted() -> Self {
        Self {
            accepted: true,
            stamp: Ros2Time::now(),
        }
    }

    /// Create rejected response
    pub fn rejected() -> Self {
        Self {
            accepted: false,
            stamp: Ros2Time::now(),
        }
    }

    /// Encode as CDR bytes (9 bytes: 1 bool + 8 time)
    pub fn encode_cdr(&self) -> Vec<u8> {
        let mut data = Vec::with_capacity(9);
        data.push(if self.accepted { 1 } else { 0 });
        data.extend(self.stamp.encode_cdr());
        data
    }

    /// Decode from CDR bytes
    pub fn decode_cdr(data: &[u8]) -> Option<Self> {
        if data.len() < 9 {
            return None;
        }
        let accepted = data[0] != 0;
        let stamp = Ros2Time::decode_cdr(&data[1..9])?;
        Some(Self { accepted, stamp })
    }
}

// ============================================================================
// CancelGoal Request/Response
// ============================================================================

/// Cancel Goal Request (action_msgs/CancelGoal_Request)
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct CancelGoalRequest {
    /// Goal info to cancel (empty goal_id = cancel all)
    pub goal_info: GoalInfo,
}

impl CancelGoalRequest {
    /// Cancel a specific goal
    pub fn cancel_goal(goal_id: GoalId) -> Self {
        Self {
            goal_info: GoalInfo::with_id(goal_id),
        }
    }

    /// Cancel all goals
    pub fn cancel_all() -> Self {
        Self::default()
    }

    /// Cancel goals before a timestamp
    pub fn cancel_before(stamp: Ros2Time) -> Self {
        Self {
            goal_info: GoalInfo {
                goal_id: GoalId::default(),
                stamp,
            },
        }
    }

    /// Encode to CDR format
    pub fn encode_cdr(&self) -> Vec<u8> {
        self.goal_info.encode_cdr()
    }

    /// Decode from CDR format
    pub fn decode_cdr(data: &[u8]) -> Option<Self> {
        let goal_info = GoalInfo::decode_cdr(data)?;
        Some(Self { goal_info })
    }
}

/// Cancel Goal Response error codes
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[repr(i8)]
pub enum CancelGoalErrorCode {
    /// No error
    None = 0,
    /// Goal was rejected (unknown goal)
    Rejected = 1,
    /// Unknown goal ID
    UnknownGoal = 2,
    /// Goal is not cancelable
    GoalNotCancelable = 3,
}

impl Default for CancelGoalErrorCode {
    fn default() -> Self {
        CancelGoalErrorCode::None
    }
}

/// Cancel Goal Response (action_msgs/CancelGoal_Response)
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct CancelGoalResponse {
    /// Error code
    pub return_code: CancelGoalErrorCode,
    /// Goals that are being canceled
    pub goals_canceling: Vec<GoalInfo>,
}

impl CancelGoalResponse {
    /// Create success response
    pub fn success(goals: Vec<GoalInfo>) -> Self {
        Self {
            return_code: CancelGoalErrorCode::None,
            goals_canceling: goals,
        }
    }

    /// Create error response
    pub fn error(code: CancelGoalErrorCode) -> Self {
        Self {
            return_code: code,
            goals_canceling: Vec::new(),
        }
    }

    /// Encode to CDR format
    pub fn encode_cdr(&self) -> Vec<u8> {
        let mut data = Vec::new();
        data.push(self.return_code as u8);
        // Encode number of goals (4 bytes, little-endian)
        data.extend((self.goals_canceling.len() as u32).to_le_bytes());
        for goal in &self.goals_canceling {
            data.extend(goal.encode_cdr());
        }
        data
    }

    /// Decode from CDR format
    pub fn decode_cdr(data: &[u8]) -> Option<Self> {
        if data.len() < 5 {
            return None;
        }
        let return_code = match data[0] as i8 {
            0 => CancelGoalErrorCode::None,
            1 => CancelGoalErrorCode::Rejected,
            2 => CancelGoalErrorCode::UnknownGoal,
            3 => CancelGoalErrorCode::GoalNotCancelable,
            _ => return None,
        };
        let count = u32::from_le_bytes([data[1], data[2], data[3], data[4]]) as usize;
        let mut goals = Vec::with_capacity(count);
        let mut offset = 5;
        for _ in 0..count {
            if offset + 24 > data.len() {
                return None;
            }
            let goal = GoalInfo::decode_cdr(&data[offset..offset + 24])?;
            goals.push(goal);
            offset += 24;
        }
        Some(Self {
            return_code,
            goals_canceling: goals,
        })
    }
}

// ============================================================================
// GetResult Request/Response
// ============================================================================

/// Get Result Request
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct GetResultRequest {
    /// Goal ID to get result for
    pub goal_id: GoalId,
}

impl GetResultRequest {
    pub fn new(goal_id: GoalId) -> Self {
        Self { goal_id }
    }
}

/// Get Result Response wrapper
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct GetResultResponse<R> {
    /// Final status of the goal
    pub status: GoalStatus,
    /// The result data
    pub result: R,
}

impl<R: Default> Default for GetResultResponse<R> {
    fn default() -> Self {
        Self {
            status: GoalStatus::Unknown,
            result: R::default(),
        }
    }
}

// ============================================================================
// Feedback Message
// ============================================================================

/// Feedback Message wrapper
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct FeedbackMessage<F> {
    /// Goal ID this feedback is for
    pub goal_id: GoalId,
    /// The feedback data
    pub feedback: F,
}

impl<F> FeedbackMessage<F> {
    pub fn new(goal_id: GoalId, feedback: F) -> Self {
        Self { goal_id, feedback }
    }
}

// ============================================================================
// Action Configuration
// ============================================================================

/// Configuration for ROS2 action communication
#[derive(Debug, Clone)]
pub struct Ros2ActionConfig {
    /// Namespace for the action
    pub namespace: Option<String>,
    /// Timeout for goal requests
    pub goal_timeout: Duration,
    /// Timeout for result requests
    pub result_timeout: Duration,
    /// Feedback queue size
    pub feedback_queue_size: usize,
    /// Status publish rate (Hz)
    pub status_publish_rate: f64,
}

impl Default for Ros2ActionConfig {
    fn default() -> Self {
        Self {
            namespace: None,
            goal_timeout: Duration::from_secs(10),
            result_timeout: Duration::from_secs(300),
            feedback_queue_size: 10,
            status_publish_rate: 10.0,
        }
    }
}

impl Ros2ActionConfig {
    /// Create with namespace
    pub fn with_namespace(namespace: impl Into<String>) -> Self {
        Self {
            namespace: Some(namespace.into()),
            ..Default::default()
        }
    }

    /// Set goal timeout
    pub fn goal_timeout(mut self, timeout: Duration) -> Self {
        self.goal_timeout = timeout;
        self
    }

    /// Set result timeout
    pub fn result_timeout(mut self, timeout: Duration) -> Self {
        self.result_timeout = timeout;
        self
    }
}

// ============================================================================
// Action Errors
// ============================================================================

/// Errors that can occur during action operations
#[derive(Debug, Clone)]
pub enum Ros2ActionError {
    /// Goal was rejected by the server
    GoalRejected,
    /// Goal was canceled
    GoalCanceled,
    /// Goal was aborted
    GoalAborted,
    /// Goal not found
    GoalNotFound,
    /// Timeout waiting for response
    Timeout,
    /// Communication error
    CommunicationError(String),
    /// Serialization error
    SerializationError(String),
    /// Server not available
    ServerUnavailable,
    /// Invalid state transition
    InvalidStateTransition(GoalStatus, GoalStatus),
}

impl std::fmt::Display for Ros2ActionError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Ros2ActionError::GoalRejected => write!(f, "Goal was rejected"),
            Ros2ActionError::GoalCanceled => write!(f, "Goal was canceled"),
            Ros2ActionError::GoalAborted => write!(f, "Goal was aborted"),
            Ros2ActionError::GoalNotFound => write!(f, "Goal not found"),
            Ros2ActionError::Timeout => write!(f, "Timeout waiting for response"),
            Ros2ActionError::CommunicationError(e) => write!(f, "Communication error: {}", e),
            Ros2ActionError::SerializationError(e) => write!(f, "Serialization error: {}", e),
            Ros2ActionError::ServerUnavailable => write!(f, "Action server unavailable"),
            Ros2ActionError::InvalidStateTransition(from, to) => {
                write!(f, "Invalid state transition: {:?} -> {:?}", from, to)
            }
        }
    }
}

impl std::error::Error for Ros2ActionError {}

impl From<QueryError> for Ros2ActionError {
    fn from(err: QueryError) -> Self {
        Ros2ActionError::CommunicationError(err.to_string())
    }
}

// ============================================================================
// Goal Handle (Client-side)
// ============================================================================

/// Handle to an active goal on the client side
#[derive(Debug)]
pub struct ClientGoalHandle<R, F> {
    /// Goal ID
    pub goal_id: GoalId,
    /// Current status
    status: Arc<RwLock<GoalStatus>>,
    /// Feedback receiver (channel based in real impl)
    feedback_queue: Arc<RwLock<Vec<F>>>,
    /// Result (set when goal completes)
    result: Arc<RwLock<Option<R>>>,
    /// Action name for topic construction
    action_name: String,
}

impl<R, F> ClientGoalHandle<R, F> {
    /// Create new goal handle
    pub fn new(goal_id: GoalId, action_name: impl Into<String>) -> Self {
        Self {
            goal_id,
            status: Arc::new(RwLock::new(GoalStatus::Accepted)),
            feedback_queue: Arc::new(RwLock::new(Vec::new())),
            result: Arc::new(RwLock::new(None)),
            action_name: action_name.into(),
        }
    }

    /// Get current goal status
    pub fn get_status(&self) -> GoalStatus {
        *self.status.read()
    }

    /// Check if goal is done
    pub fn is_done(&self) -> bool {
        self.status.read().is_terminal()
    }

    /// Update status (called internally)
    pub fn update_status(&self, status: GoalStatus) {
        *self.status.write() = status;
    }

    /// Add feedback (called internally)
    pub fn add_feedback(&self, feedback: F) {
        self.feedback_queue.write().push(feedback);
    }

    /// Get pending feedback (drains queue)
    pub fn get_pending_feedback(&self) -> Vec<F> {
        std::mem::take(&mut *self.feedback_queue.write())
    }

    /// Set result (called internally)
    pub fn set_result(&self, result: R) {
        *self.result.write() = Some(result);
    }

    /// Get result if available
    pub fn try_get_result(&self) -> Option<R>
    where
        R: Clone,
    {
        self.result.read().clone()
    }
}

// ============================================================================
// Goal Handle (Server-side)
// ============================================================================

/// Handle to an active goal on the server side
pub struct ServerGoalHandle<G, R, F> {
    /// Goal ID
    pub goal_id: GoalId,
    /// The goal request data
    pub goal: G,
    /// Current status
    status: Arc<RwLock<GoalStatus>>,
    /// Result to send
    result: Arc<RwLock<Option<R>>>,
    /// Feedback callback
    feedback_sender: Arc<dyn Fn(F) + Send + Sync>,
}

impl<G, R, F> ServerGoalHandle<G, R, F> {
    /// Get current status
    pub fn get_status(&self) -> GoalStatus {
        *self.status.read()
    }

    /// Check if cancellation was requested
    pub fn is_cancel_requested(&self) -> bool {
        *self.status.read() == GoalStatus::Canceling
    }

    /// Mark goal as executing
    pub fn execute(&self) {
        *self.status.write() = GoalStatus::Executing;
    }

    /// Publish feedback
    pub fn publish_feedback(&self, feedback: F) {
        (self.feedback_sender)(feedback);
    }

    /// Mark goal as succeeded with result
    pub fn succeed(&self, result: R) {
        *self.result.write() = Some(result);
        *self.status.write() = GoalStatus::Succeeded;
    }

    /// Mark goal as aborted with result
    pub fn abort(&self, result: R) {
        *self.result.write() = Some(result);
        *self.status.write() = GoalStatus::Aborted;
    }

    /// Mark goal as canceled with result
    pub fn canceled(&self, result: R) {
        *self.result.write() = Some(result);
        *self.status.write() = GoalStatus::Canceled;
    }
}

// ============================================================================
// Action Statistics
// ============================================================================

/// Statistics for action operations
#[derive(Debug, Default)]
pub struct ActionStats {
    /// Number of goals sent (client) or received (server)
    pub goals_count: AtomicU64,
    /// Number of goals accepted
    pub goals_accepted: AtomicU64,
    /// Number of goals rejected
    pub goals_rejected: AtomicU64,
    /// Number of goals succeeded
    pub goals_succeeded: AtomicU64,
    /// Number of goals aborted
    pub goals_aborted: AtomicU64,
    /// Number of goals canceled
    pub goals_canceled: AtomicU64,
    /// Number of feedback messages
    pub feedback_count: AtomicU64,
    /// Number of timeouts
    pub timeouts: AtomicU64,
}

impl ActionStats {
    /// Record a goal sent/received
    pub fn record_goal(&self) {
        self.goals_count.fetch_add(1, Ordering::Relaxed);
    }

    /// Record goal accepted
    pub fn record_accepted(&self) {
        self.goals_accepted.fetch_add(1, Ordering::Relaxed);
    }

    /// Record goal rejected
    pub fn record_rejected(&self) {
        self.goals_rejected.fetch_add(1, Ordering::Relaxed);
    }

    /// Record goal succeeded
    pub fn record_succeeded(&self) {
        self.goals_succeeded.fetch_add(1, Ordering::Relaxed);
    }

    /// Record goal aborted
    pub fn record_aborted(&self) {
        self.goals_aborted.fetch_add(1, Ordering::Relaxed);
    }

    /// Record goal canceled
    pub fn record_canceled(&self) {
        self.goals_canceled.fetch_add(1, Ordering::Relaxed);
    }

    /// Record feedback
    pub fn record_feedback(&self) {
        self.feedback_count.fetch_add(1, Ordering::Relaxed);
    }

    /// Record timeout
    pub fn record_timeout(&self) {
        self.timeouts.fetch_add(1, Ordering::Relaxed);
    }

    /// Get success rate
    pub fn success_rate(&self) -> f64 {
        let total = self.goals_count.load(Ordering::Relaxed);
        if total == 0 {
            return 0.0;
        }
        let succeeded = self.goals_succeeded.load(Ordering::Relaxed);
        succeeded as f64 / total as f64
    }
}

// ============================================================================
// Common Action Types (similar to common service types)
// ============================================================================

/// Empty Goal (for actions that don't need goal parameters)
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct EmptyGoal;

/// Empty Result (for actions that don't return data)
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct EmptyResult;

/// Empty Feedback (for actions that don't provide feedback)
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct EmptyFeedback;

/// Progress Feedback (common pattern for progress reporting)
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct ProgressFeedback {
    /// Progress percentage (0.0 - 100.0)
    pub progress: f32,
    /// Optional status message
    pub message: String,
}

impl ProgressFeedback {
    pub fn new(progress: f32) -> Self {
        Self {
            progress,
            message: String::new(),
        }
    }

    pub fn with_message(progress: f32, message: impl Into<String>) -> Self {
        Self {
            progress,
            message: message.into(),
        }
    }
}

// ============================================================================
// Action Trait (for type-safe action definitions)
// ============================================================================

/// Trait defining an action type with Goal, Result, and Feedback
pub trait ActionType {
    /// Goal message type
    type Goal: Serialize + DeserializeOwned + Clone + Send + Sync;
    /// Result message type
    type Result: Serialize + DeserializeOwned + Clone + Default + Send + Sync;
    /// Feedback message type
    type Feedback: Serialize + DeserializeOwned + Clone + Send + Sync;

    /// Action name for topic construction
    fn action_name() -> &'static str;
}

// ============================================================================
// Action Client (Zenoh-based)
// ============================================================================

/// ROS2 Action Client over Zenoh
///
/// Provides functionality to send goals, receive feedback, and get results
/// from a ROS2 action server via Zenoh transport.
pub struct Ros2ActionClient<G, R, F> {
    /// Action name
    action_name: String,
    /// Configuration
    config: Ros2ActionConfig,
    /// Active goals
    active_goals: Arc<RwLock<HashMap<GoalId, Arc<ClientGoalHandle<R, F>>>>>,
    /// Statistics
    stats: Arc<ActionStats>,
    /// Sequence number for requests
    sequence: AtomicU64,
    /// Marker for goal type
    _goal_marker: std::marker::PhantomData<G>,
}

impl<G, R, F> Ros2ActionClient<G, R, F>
where
    G: Serialize + DeserializeOwned + Clone + Send + Sync,
    R: Serialize + DeserializeOwned + Clone + Default + Send + Sync + 'static,
    F: Serialize + DeserializeOwned + Clone + Send + Sync + 'static,
{
    /// Create a new action client
    pub fn new(action_name: impl Into<String>, config: Ros2ActionConfig) -> Self {
        Self {
            action_name: action_name.into(),
            config,
            active_goals: Arc::new(RwLock::new(HashMap::new())),
            stats: Arc::new(ActionStats::default()),
            sequence: AtomicU64::new(0),
            _goal_marker: std::marker::PhantomData,
        }
    }

    /// Get the action name
    pub fn action_name(&self) -> &str {
        &self.action_name
    }

    /// Get action topics
    pub fn get_topics(&self) -> ActionClientTopics {
        let name = self.get_full_action_name();
        ActionClientTopics {
            send_goal: action_to_send_goal_topic(&name),
            cancel_goal: action_to_cancel_goal_topic(&name),
            get_result: action_to_get_result_topic(&name),
            status: action_to_status_topic(&name),
            feedback: action_to_feedback_topic(&name),
        }
    }

    /// Get full action name with namespace
    fn get_full_action_name(&self) -> String {
        match &self.config.namespace {
            Some(ns) => apply_namespace_to_action(&self.action_name, ns),
            None => self.action_name.clone(),
        }
    }

    /// Send a goal (returns goal handle for tracking)
    ///
    /// This creates a SendGoalRequest with a new goal ID and queues it for sending.
    /// The returned handle can be used to track progress, receive feedback, and get results.
    pub fn send_goal(&self, goal: G) -> Result<Arc<ClientGoalHandle<R, F>>, Ros2ActionError> {
        let goal_id = GoalId::new();
        self.send_goal_with_id(goal_id, goal)
    }

    /// Send a goal with a specific goal ID
    pub fn send_goal_with_id(
        &self,
        goal_id: GoalId,
        _goal: G,
    ) -> Result<Arc<ClientGoalHandle<R, F>>, Ros2ActionError> {
        self.stats.record_goal();

        // Create goal handle
        let handle = Arc::new(ClientGoalHandle::new(goal_id, &self.action_name));

        // Register in active goals
        self.active_goals.write().insert(goal_id, handle.clone());

        // In a real implementation, this would:
        // 1. Serialize the SendGoalRequest with CDR
        // 2. Publish to the send_goal topic via Zenoh
        // 3. Wait for SendGoalResponse
        // 4. Start listening for feedback and status updates

        // For now, we mark as accepted (real impl would wait for server response)
        self.stats.record_accepted();

        Ok(handle)
    }

    /// Cancel a goal by ID
    pub fn cancel_goal(&self, goal_id: GoalId) -> Result<CancelGoalResponse, Ros2ActionError> {
        let handle = self
            .active_goals
            .read()
            .get(&goal_id)
            .cloned()
            .ok_or(Ros2ActionError::GoalNotFound)?;

        handle.update_status(GoalStatus::Canceling);

        // In real impl: send CancelGoalRequest via Zenoh and wait for response
        Ok(CancelGoalResponse::success(vec![GoalInfo::with_id(
            goal_id,
        )]))
    }

    /// Cancel all active goals
    pub fn cancel_all_goals(&self) -> Result<CancelGoalResponse, Ros2ActionError> {
        let goals: Vec<GoalInfo> = self
            .active_goals
            .read()
            .keys()
            .map(|id| GoalInfo::with_id(*id))
            .collect();

        for handle in self.active_goals.read().values() {
            handle.update_status(GoalStatus::Canceling);
        }

        Ok(CancelGoalResponse::success(goals))
    }

    /// Get result for a completed goal
    pub fn get_result(&self, goal_id: GoalId) -> Result<GetResultResponse<R>, Ros2ActionError>
    where
        R: Default,
    {
        let handle = self
            .active_goals
            .read()
            .get(&goal_id)
            .cloned()
            .ok_or(Ros2ActionError::GoalNotFound)?;

        if !handle.is_done() {
            return Err(Ros2ActionError::InvalidStateTransition(
                handle.get_status(),
                GoalStatus::Succeeded,
            ));
        }

        let result = handle.try_get_result().unwrap_or_default();
        Ok(GetResultResponse {
            status: handle.get_status(),
            result,
        })
    }

    /// Get a goal handle by ID
    pub fn get_goal_handle(&self, goal_id: GoalId) -> Option<Arc<ClientGoalHandle<R, F>>> {
        self.active_goals.read().get(&goal_id).cloned()
    }

    /// Get all active goal IDs
    pub fn get_active_goals(&self) -> Vec<GoalId> {
        self.active_goals.read().keys().copied().collect()
    }

    /// Remove completed goals from tracking
    pub fn cleanup_completed(&self) {
        self.active_goals
            .write()
            .retain(|_, handle| !handle.is_done());
    }

    /// Get statistics
    pub fn stats(&self) -> &ActionStats {
        &self.stats
    }

    /// Process incoming feedback (called by transport layer)
    pub fn handle_feedback(&self, goal_id: GoalId, feedback: F) {
        if let Some(handle) = self.active_goals.read().get(&goal_id) {
            handle.add_feedback(feedback);
            self.stats.record_feedback();
        }
    }

    /// Process incoming status update (called by transport layer)
    pub fn handle_status(&self, status_array: &GoalStatusArray) {
        for status_info in &status_array.status_list {
            if let Some(handle) = self.active_goals.read().get(&status_info.goal_info.goal_id) {
                handle.update_status(status_info.status);

                // Update stats based on terminal status
                match status_info.status {
                    GoalStatus::Succeeded => self.stats.record_succeeded(),
                    GoalStatus::Canceled => self.stats.record_canceled(),
                    GoalStatus::Aborted => self.stats.record_aborted(),
                    _ => {}
                }
            }
        }
    }
}

/// Topics used by an action client
#[derive(Debug, Clone)]
pub struct ActionClientTopics {
    pub send_goal: String,
    pub cancel_goal: String,
    pub get_result: String,
    pub status: String,
    pub feedback: String,
}

// ============================================================================
// Action Server (Zenoh-based)
// ============================================================================

/// Callback type for goal acceptance
pub type GoalCallback<G> = Box<dyn Fn(&G) -> bool + Send + Sync>;

/// Callback type for cancel acceptance
pub type CancelCallback = Box<dyn Fn(GoalId) -> bool + Send + Sync>;

/// Callback type for goal execution
pub type ExecuteCallback<G, R, F> = Box<dyn Fn(Arc<ServerGoalHandle<G, R, F>>) + Send + Sync>;

/// ROS2 Action Server over Zenoh
///
/// Provides functionality to receive goals, execute them, and publish
/// feedback and results to ROS2 action clients via Zenoh transport.
pub struct Ros2ActionServer<G, R, F> {
    /// Action name
    action_name: String,
    /// Configuration
    config: Ros2ActionConfig,
    /// Active goals being processed
    active_goals: Arc<RwLock<HashMap<GoalId, Arc<ServerGoalHandle<G, R, F>>>>>,
    /// Statistics
    stats: Arc<ActionStats>,
    /// Goal acceptance callback
    goal_callback: Option<GoalCallback<G>>,
    /// Cancel acceptance callback
    cancel_callback: Option<CancelCallback>,
    /// Goal execution callback
    execute_callback: Option<ExecuteCallback<G, R, F>>,
}

impl<G, R, F> Ros2ActionServer<G, R, F>
where
    G: Serialize + DeserializeOwned + Clone + Send + Sync + 'static,
    R: Serialize + DeserializeOwned + Clone + Default + Send + Sync + 'static,
    F: Serialize + DeserializeOwned + Clone + Send + Sync + 'static,
{
    /// Create a new action server
    pub fn new(action_name: impl Into<String>, config: Ros2ActionConfig) -> Self {
        Self {
            action_name: action_name.into(),
            config,
            active_goals: Arc::new(RwLock::new(HashMap::new())),
            stats: Arc::new(ActionStats::default()),
            goal_callback: None,
            cancel_callback: None,
            execute_callback: None,
        }
    }

    /// Set the goal acceptance callback
    pub fn set_goal_callback(&mut self, callback: GoalCallback<G>) {
        self.goal_callback = Some(callback);
    }

    /// Set the cancel acceptance callback
    pub fn set_cancel_callback(&mut self, callback: CancelCallback) {
        self.cancel_callback = Some(callback);
    }

    /// Set the goal execution callback
    pub fn set_execute_callback(&mut self, callback: ExecuteCallback<G, R, F>) {
        self.execute_callback = Some(callback);
    }

    /// Get action topics
    pub fn get_topics(&self) -> ActionServerTopics {
        let name = self.get_full_action_name();
        ActionServerTopics {
            send_goal: action_to_send_goal_topic(&name),
            cancel_goal: action_to_cancel_goal_topic(&name),
            get_result: action_to_get_result_topic(&name),
            status: action_to_status_topic(&name),
            feedback: action_to_feedback_topic(&name),
        }
    }

    /// Get full action name with namespace
    fn get_full_action_name(&self) -> String {
        match &self.config.namespace {
            Some(ns) => apply_namespace_to_action(&self.action_name, ns),
            None => self.action_name.clone(),
        }
    }

    /// Handle incoming goal request
    pub fn handle_goal_request(&self, goal_id: GoalId, goal: G) -> SendGoalResponse {
        self.stats.record_goal();

        // Check if goal should be accepted
        let accepted = match &self.goal_callback {
            Some(callback) => callback(&goal),
            None => true, // Accept all by default
        };

        if !accepted {
            self.stats.record_rejected();
            return SendGoalResponse::rejected();
        }

        self.stats.record_accepted();

        // Create server goal handle
        let feedback_goals = self.active_goals.clone();
        let goal_id_for_feedback = goal_id;
        let feedback_sender: Arc<dyn Fn(F) + Send + Sync> = Arc::new(move |_feedback: F| {
            // In real impl: publish feedback to Zenoh topic
            // For now, just track that feedback was sent
            let _ = feedback_goals.read().get(&goal_id_for_feedback);
        });

        let handle = Arc::new(ServerGoalHandle {
            goal_id,
            goal,
            status: Arc::new(RwLock::new(GoalStatus::Accepted)),
            result: Arc::new(RwLock::new(None)),
            feedback_sender,
        });

        self.active_goals.write().insert(goal_id, handle.clone());

        // Trigger execution callback if set
        if let Some(callback) = &self.execute_callback {
            callback(handle);
        }

        SendGoalResponse::accepted()
    }

    /// Handle cancel request
    pub fn handle_cancel_request(&self, request: &CancelGoalRequest) -> CancelGoalResponse {
        let mut canceling = Vec::new();

        // If goal_id is zero (all zeros), cancel based on timestamp
        let cancel_all = request.goal_info.goal_id.uuid == [0u8; 16];

        for (goal_id, handle) in self.active_goals.read().iter() {
            let should_cancel = if cancel_all {
                // Cancel if goal was accepted before the specified time
                // (if stamp is zero, cancel all)
                request.goal_info.stamp.sec == 0
                    || handle.goal_info().stamp.sec <= request.goal_info.stamp.sec
            } else {
                *goal_id == request.goal_info.goal_id
            };

            if should_cancel && handle.get_status().is_active() {
                // Check with cancel callback
                let allow_cancel = match &self.cancel_callback {
                    Some(callback) => callback(*goal_id),
                    None => true,
                };

                if allow_cancel {
                    *handle.status.write() = GoalStatus::Canceling;
                    canceling.push(GoalInfo::with_id(*goal_id));
                }
            }
        }

        if canceling.is_empty() && !cancel_all {
            CancelGoalResponse::error(CancelGoalErrorCode::UnknownGoal)
        } else {
            CancelGoalResponse::success(canceling)
        }
    }

    /// Handle get result request
    pub fn handle_get_result_request(&self, goal_id: GoalId) -> Option<GetResultResponse<R>>
    where
        R: Clone + Default,
    {
        let handle = self.active_goals.read().get(&goal_id).cloned()?;

        if !handle.get_status().is_terminal() {
            // Goal not yet complete - in real impl would wait
            return None;
        }

        let result = handle.result.read().clone().unwrap_or_default();
        Some(GetResultResponse {
            status: handle.get_status(),
            result,
        })
    }

    /// Get current status array for all goals
    pub fn get_status_array(&self) -> GoalStatusArray {
        let mut array = GoalStatusArray::new();
        for (goal_id, handle) in self.active_goals.read().iter() {
            array.add(*goal_id, handle.get_status());
        }
        array
    }

    /// Get a goal handle by ID
    pub fn get_goal_handle(&self, goal_id: GoalId) -> Option<Arc<ServerGoalHandle<G, R, F>>> {
        self.active_goals.read().get(&goal_id).cloned()
    }

    /// Get all active goal IDs
    pub fn get_active_goals(&self) -> Vec<GoalId> {
        self.active_goals.read().keys().copied().collect()
    }

    /// Remove completed goals from tracking
    pub fn cleanup_completed(&self) {
        self.active_goals
            .write()
            .retain(|_, handle| !handle.get_status().is_terminal());
    }

    /// Get statistics
    pub fn stats(&self) -> &ActionStats {
        &self.stats
    }
}

// Add goal_info to ServerGoalHandle
impl<G, R, F> ServerGoalHandle<G, R, F> {
    /// Get goal info
    pub fn goal_info(&self) -> GoalInfo {
        GoalInfo::with_id(self.goal_id)
    }
}

/// Topics used by an action server
#[derive(Debug, Clone)]
pub struct ActionServerTopics {
    pub send_goal: String,
    pub cancel_goal: String,
    pub get_result: String,
    pub status: String,
    pub feedback: String,
}

// ============================================================================
// Tests
// ============================================================================

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_goal_id_generation() {
        let id1 = GoalId::new();
        let id2 = GoalId::new();
        assert_ne!(id1, id2, "Goal IDs should be unique");
    }

    #[test]
    fn test_goal_id_cdr_roundtrip() {
        let original = GoalId::new();
        let encoded = original.encode_cdr();
        let decoded = GoalId::decode_cdr(&encoded).unwrap();
        assert_eq!(original, decoded);
    }

    #[test]
    fn test_ros2_time_now() {
        let time = Ros2Time::now();
        assert!(time.sec > 0, "Time should be after epoch");
    }

    #[test]
    fn test_ros2_time_cdr_roundtrip() {
        let original = Ros2Time::now();
        let encoded = original.encode_cdr();
        let decoded = Ros2Time::decode_cdr(&encoded).unwrap();
        assert_eq!(original, decoded);
    }

    #[test]
    fn test_goal_info_cdr_roundtrip() {
        let original = GoalInfo::new();
        let encoded = original.encode_cdr();
        let decoded = GoalInfo::decode_cdr(&encoded).unwrap();
        assert_eq!(original.goal_id, decoded.goal_id);
    }

    #[test]
    fn test_goal_status_terminal() {
        assert!(!GoalStatus::Accepted.is_terminal());
        assert!(!GoalStatus::Executing.is_terminal());
        assert!(!GoalStatus::Canceling.is_terminal());
        assert!(GoalStatus::Succeeded.is_terminal());
        assert!(GoalStatus::Canceled.is_terminal());
        assert!(GoalStatus::Aborted.is_terminal());
    }

    #[test]
    fn test_goal_status_active() {
        assert!(GoalStatus::Accepted.is_active());
        assert!(GoalStatus::Executing.is_active());
        assert!(GoalStatus::Canceling.is_active());
        assert!(!GoalStatus::Succeeded.is_active());
        assert!(!GoalStatus::Canceled.is_active());
        assert!(!GoalStatus::Aborted.is_active());
    }

    #[test]
    fn test_action_topic_naming() {
        let action = "navigate_to_pose";

        assert_eq!(
            action_to_send_goal_topic(action),
            "rq/navigate_to_pose/_action/send_goal"
        );
        assert_eq!(
            action_to_cancel_goal_topic(action),
            "rq/navigate_to_pose/_action/cancel_goal"
        );
        assert_eq!(
            action_to_get_result_topic(action),
            "rq/navigate_to_pose/_action/get_result"
        );
        assert_eq!(
            action_to_status_topic(action),
            "rt/navigate_to_pose/_action/status"
        );
        assert_eq!(
            action_to_feedback_topic(action),
            "rt/navigate_to_pose/_action/feedback"
        );
    }

    #[test]
    fn test_parse_action_topic() {
        let (name, topic_type) = parse_action_topic("rq/navigate/_action/send_goal").unwrap();
        assert_eq!(name, "navigate");
        assert_eq!(topic_type, ActionTopicType::SendGoal);

        let (name, topic_type) = parse_action_topic("rt/navigate/_action/feedback").unwrap();
        assert_eq!(name, "navigate");
        assert_eq!(topic_type, ActionTopicType::Feedback);
    }

    #[test]
    fn test_send_goal_response_cdr() {
        let response = SendGoalResponse::accepted();
        let encoded = response.encode_cdr();
        let decoded = SendGoalResponse::decode_cdr(&encoded).unwrap();
        assert_eq!(response.accepted, decoded.accepted);
    }

    #[test]
    fn test_goal_status_array_cdr_roundtrip() {
        let mut array = GoalStatusArray::new();
        array.add(GoalId::new(), GoalStatus::Executing);
        array.add(GoalId::new(), GoalStatus::Succeeded);

        let encoded = array.encode_cdr();
        let decoded = GoalStatusArray::decode_cdr(&encoded).unwrap();

        assert_eq!(array.status_list.len(), decoded.status_list.len());
        assert_eq!(array.status_list[0].status, decoded.status_list[0].status);
        assert_eq!(array.status_list[1].status, decoded.status_list[1].status);
    }

    #[test]
    fn test_action_stats() {
        let stats = ActionStats::default();
        stats.record_goal();
        stats.record_accepted();
        stats.record_succeeded();

        assert_eq!(stats.goals_count.load(Ordering::Relaxed), 1);
        assert_eq!(stats.goals_accepted.load(Ordering::Relaxed), 1);
        assert_eq!(stats.goals_succeeded.load(Ordering::Relaxed), 1);
        assert_eq!(stats.success_rate(), 1.0);
    }

    #[test]
    fn test_apply_namespace() {
        assert_eq!(
            apply_namespace_to_action("navigate", "robot1"),
            "robot1/navigate"
        );
        // Note: leading / in namespace is preserved (ROS2 convention)
        assert_eq!(
            apply_namespace_to_action("/navigate", "/robot1/"),
            "/robot1/navigate"
        );
        assert_eq!(apply_namespace_to_action("navigate", ""), "navigate");
    }

    // ========================================================================
    // Integration Tests: ActionClient/ActionServer Workflow
    // ========================================================================

    /// Simple goal type for testing
    #[derive(Debug, Clone, Default, Serialize, Deserialize)]
    struct TestGoal {
        target_position: f64,
    }

    /// Simple result type for testing
    #[derive(Debug, Clone, Default, Serialize, Deserialize)]
    struct TestResult {
        final_position: f64,
        success: bool,
    }

    /// Simple feedback type for testing
    #[derive(Debug, Clone, Default, Serialize, Deserialize)]
    struct TestFeedback {
        current_position: f64,
        progress: f64,
    }

    #[test]
    fn test_action_client_server_goal_workflow() {
        // Create client and server
        let config = Ros2ActionConfig::default();
        let client: Ros2ActionClient<TestGoal, TestResult, TestFeedback> =
            Ros2ActionClient::new("test_action", config.clone());
        let server: Ros2ActionServer<TestGoal, TestResult, TestFeedback> =
            Ros2ActionServer::new("test_action", config);

        // Client sends a goal
        let goal = TestGoal {
            target_position: 10.0,
        };
        let goal_handle = client.send_goal(goal.clone()).unwrap();
        let goal_id = goal_handle.goal_id;

        // Verify client state
        assert_eq!(client.get_active_goals().len(), 1);
        assert!(client.get_goal_handle(goal_id).is_some());

        // Server receives and processes the goal
        let response = server.handle_goal_request(goal_id, goal);
        assert!(response.accepted);
        assert!(response.stamp.sec > 0);

        // Server has the goal tracked
        assert_eq!(server.get_active_goals().len(), 1);
        assert!(server.get_goal_handle(goal_id).is_some());

        // Get status array from server
        let status_array = server.get_status_array();
        assert_eq!(status_array.status_list.len(), 1);
        assert_eq!(status_array.status_list[0].status, GoalStatus::Accepted);

        // Client processes status update
        client.handle_status(&status_array);
        assert_eq!(goal_handle.get_status(), GoalStatus::Accepted);

        // Server completes the goal
        let server_handle = server.get_goal_handle(goal_id).unwrap();
        server_handle.succeed(TestResult {
            final_position: 10.0,
            success: true,
        });

        // Get updated status
        let status_array = server.get_status_array();
        assert_eq!(status_array.status_list[0].status, GoalStatus::Succeeded);

        // Client receives status update
        client.handle_status(&status_array);
        assert!(goal_handle.is_done());

        // Stats verification
        assert_eq!(client.stats().goals_count.load(Ordering::Relaxed), 1);
        assert_eq!(server.stats().goals_accepted.load(Ordering::Relaxed), 1);
    }

    #[test]
    fn test_action_client_server_feedback_flow() {
        let config = Ros2ActionConfig::default();
        let client: Ros2ActionClient<TestGoal, TestResult, TestFeedback> =
            Ros2ActionClient::new("feedback_action", config.clone());
        let server: Ros2ActionServer<TestGoal, TestResult, TestFeedback> =
            Ros2ActionServer::new("feedback_action", config);

        // Setup and send goal
        let goal = TestGoal {
            target_position: 100.0,
        };
        let handle = client.send_goal(goal.clone()).unwrap();
        let goal_id = handle.goal_id;
        server.handle_goal_request(goal_id, goal);

        // Server sends feedback
        let server_handle = server.get_goal_handle(goal_id).unwrap();
        server_handle.publish_feedback(TestFeedback {
            current_position: 25.0,
            progress: 0.25,
        });
        server_handle.publish_feedback(TestFeedback {
            current_position: 50.0,
            progress: 0.50,
        });
        server_handle.publish_feedback(TestFeedback {
            current_position: 75.0,
            progress: 0.75,
        });

        // Client receives feedback (simulating transport layer)
        client.handle_feedback(
            goal_id,
            TestFeedback {
                current_position: 25.0,
                progress: 0.25,
            },
        );
        client.handle_feedback(
            goal_id,
            TestFeedback {
                current_position: 50.0,
                progress: 0.50,
            },
        );
        client.handle_feedback(
            goal_id,
            TestFeedback {
                current_position: 75.0,
                progress: 0.75,
            },
        );

        // Verify feedback was received
        let feedback_list = handle.get_pending_feedback();
        assert_eq!(feedback_list.len(), 3);
        assert!((feedback_list[2].progress - 0.75).abs() < 0.001);

        // Check stats
        assert_eq!(client.stats().feedback_count.load(Ordering::Relaxed), 3);
    }

    #[test]
    fn test_action_client_server_cancel_workflow() {
        let config = Ros2ActionConfig::default();
        let client: Ros2ActionClient<TestGoal, TestResult, TestFeedback> =
            Ros2ActionClient::new("cancel_action", config.clone());
        let server: Ros2ActionServer<TestGoal, TestResult, TestFeedback> =
            Ros2ActionServer::new("cancel_action", config);

        // Send goal
        let goal = TestGoal {
            target_position: 100.0,
        };
        let handle = client.send_goal(goal.clone()).unwrap();
        let goal_id = handle.goal_id;
        server.handle_goal_request(goal_id, goal);

        // Set server to executing state
        let server_handle = server.get_goal_handle(goal_id).unwrap();
        server_handle.execute();

        // Client requests cancel
        let cancel_response = client.cancel_goal(goal_id).unwrap();
        assert_eq!(cancel_response.return_code, CancelGoalErrorCode::None);
        assert_eq!(cancel_response.goals_canceling.len(), 1);

        // Server receives cancel request
        let cancel_request = CancelGoalRequest {
            goal_info: GoalInfo::with_id(goal_id),
        };
        let server_cancel_response = server.handle_cancel_request(&cancel_request);
        assert_eq!(
            server_cancel_response.return_code,
            CancelGoalErrorCode::None
        );

        // Verify server handle is in canceling state
        assert_eq!(server_handle.get_status(), GoalStatus::Canceling);

        // Server finishes cancellation
        server_handle.canceled(TestResult {
            final_position: 0.0,
            success: false,
        });
        server.stats().record_canceled();
        assert_eq!(server_handle.get_status(), GoalStatus::Canceled);

        // Update client status
        let status_array = server.get_status_array();
        client.handle_status(&status_array);

        // Check stats
        assert_eq!(server.stats().goals_canceled.load(Ordering::Relaxed), 1);
    }

    #[test]
    fn test_action_client_server_multiple_goals() {
        let config = Ros2ActionConfig::default();
        let client: Ros2ActionClient<TestGoal, TestResult, TestFeedback> =
            Ros2ActionClient::new("multi_goal_action", config.clone());
        let server: Ros2ActionServer<TestGoal, TestResult, TestFeedback> =
            Ros2ActionServer::new("multi_goal_action", config);

        // Send multiple goals
        let mut goal_ids = Vec::new();
        for i in 0..5 {
            let goal = TestGoal {
                target_position: i as f64 * 10.0,
            };
            let handle = client.send_goal(goal.clone()).unwrap();
            let goal_id = handle.goal_id;
            goal_ids.push(goal_id);
            server.handle_goal_request(goal_id, goal);
        }

        // Verify all goals are tracked
        assert_eq!(client.get_active_goals().len(), 5);
        assert_eq!(server.get_active_goals().len(), 5);

        // Complete some goals
        for (i, goal_id) in goal_ids.iter().enumerate() {
            let server_handle = server.get_goal_handle(*goal_id).unwrap();
            if i % 2 == 0 {
                server_handle.succeed(TestResult {
                    final_position: (i as f64) * 10.0,
                    success: true,
                });
            } else {
                server_handle.abort(TestResult {
                    final_position: 0.0,
                    success: false,
                });
            }
        }

        // Update client status
        let status_array = server.get_status_array();
        client.handle_status(&status_array);

        // All goals should be done
        for goal_id in &goal_ids {
            let handle = client.get_goal_handle(*goal_id).unwrap();
            assert!(handle.is_done());
        }

        // Cleanup completed goals
        client.cleanup_completed();
        server.cleanup_completed();
        assert_eq!(client.get_active_goals().len(), 0);
        assert_eq!(server.get_active_goals().len(), 0);
    }

    #[test]
    fn test_action_server_goal_rejection() {
        let config = Ros2ActionConfig::default();
        let mut server: Ros2ActionServer<TestGoal, TestResult, TestFeedback> =
            Ros2ActionServer::new("reject_action", config);

        // Set goal callback that rejects goals with target > 100
        server.set_goal_callback(Box::new(|goal: &TestGoal| goal.target_position <= 100.0));

        // Goal within limit should be accepted
        let goal1 = TestGoal {
            target_position: 50.0,
        };
        let goal_id1 = GoalId::new();
        let response1 = server.handle_goal_request(goal_id1, goal1);
        assert!(response1.accepted);

        // Goal exceeding limit should be rejected
        let goal2 = TestGoal {
            target_position: 150.0,
        };
        let goal_id2 = GoalId::new();
        let response2 = server.handle_goal_request(goal_id2, goal2);
        assert!(!response2.accepted);

        // Only one goal should be tracked
        assert_eq!(server.get_active_goals().len(), 1);
        assert_eq!(server.stats().goals_rejected.load(Ordering::Relaxed), 1);
    }

    #[test]
    fn test_action_server_cancel_callback() {
        let config = Ros2ActionConfig::default();
        let mut server: Ros2ActionServer<TestGoal, TestResult, TestFeedback> =
            Ros2ActionServer::new("cancel_callback_action", config);

        // Set cancel callback that only allows cancel for goals with target < 100
        server.set_cancel_callback(Box::new(|_goal_id: GoalId| {
            // For testing, always allow cancel
            true
        }));

        // Send and accept a goal
        let goal = TestGoal {
            target_position: 50.0,
        };
        let goal_id = GoalId::new();
        server.handle_goal_request(goal_id, goal);

        // Set to executing
        let handle = server.get_goal_handle(goal_id).unwrap();
        handle.execute();

        // Cancel should be accepted
        let cancel_request = CancelGoalRequest {
            goal_info: GoalInfo::with_id(goal_id),
        };
        let response = server.handle_cancel_request(&cancel_request);
        assert_eq!(response.return_code, CancelGoalErrorCode::None);
        assert_eq!(handle.get_status(), GoalStatus::Canceling);
    }

    #[test]
    fn test_action_cancel_all_goals() {
        let config = Ros2ActionConfig::default();
        let server: Ros2ActionServer<TestGoal, TestResult, TestFeedback> =
            Ros2ActionServer::new("cancel_all_action", config);

        // Create multiple goals
        for i in 0..3 {
            let goal = TestGoal {
                target_position: (i + 1) as f64 * 10.0,
            };
            let goal_id = GoalId::new();
            server.handle_goal_request(goal_id, goal);
            // Set to executing
            let handle = server.get_goal_handle(goal_id).unwrap();
            handle.execute();
        }

        assert_eq!(server.get_active_goals().len(), 3);

        // Cancel all goals (using zero goal_id and zero stamp)
        let cancel_request = CancelGoalRequest {
            goal_info: GoalInfo {
                goal_id: GoalId { uuid: [0u8; 16] },
                stamp: Ros2Time { sec: 0, nanosec: 0 },
            },
        };
        let response = server.handle_cancel_request(&cancel_request);
        assert_eq!(response.return_code, CancelGoalErrorCode::None);
        assert_eq!(response.goals_canceling.len(), 3);

        // All goals should be canceling
        for goal_id in server.get_active_goals() {
            let handle = server.get_goal_handle(goal_id).unwrap();
            assert_eq!(handle.get_status(), GoalStatus::Canceling);
        }
    }

    #[test]
    fn test_action_get_result_states() {
        let config = Ros2ActionConfig::default();
        let client: Ros2ActionClient<TestGoal, TestResult, TestFeedback> =
            Ros2ActionClient::new("result_action", config.clone());
        let server: Ros2ActionServer<TestGoal, TestResult, TestFeedback> =
            Ros2ActionServer::new("result_action", config);

        // Send goal
        let goal = TestGoal {
            target_position: 50.0,
        };
        let client_handle = client.send_goal(goal.clone()).unwrap();
        let goal_id = client_handle.goal_id;
        server.handle_goal_request(goal_id, goal);

        // Trying to get result before completion should fail (on client)
        let result = client.get_result(goal_id);
        assert!(result.is_err());

        // Server hasn't completed yet, result should be None
        let server_result = server.handle_get_result_request(goal_id);
        assert!(server_result.is_none());

        // Complete the goal
        let server_handle = server.get_goal_handle(goal_id).unwrap();
        server_handle.succeed(TestResult {
            final_position: 50.0,
            success: true,
        });

        // Update client status
        let status_array = server.get_status_array();
        client.handle_status(&status_array);

        // Now result should be available
        let server_result = server.handle_get_result_request(goal_id);
        assert!(server_result.is_some());
        let result_response = server_result.unwrap();
        assert_eq!(result_response.status, GoalStatus::Succeeded);
        assert!(result_response.result.success);

        // Client should also be able to get result
        let client_result = client.get_result(goal_id).unwrap();
        assert_eq!(client_result.status, GoalStatus::Succeeded);
    }

    #[test]
    fn test_action_client_topics() {
        let config = Ros2ActionConfig {
            namespace: Some("robot1".to_string()),
            ..Default::default()
        };
        let client: Ros2ActionClient<TestGoal, TestResult, TestFeedback> =
            Ros2ActionClient::new("navigate", config);

        let topics = client.get_topics();
        assert_eq!(topics.send_goal, "rq/robot1/navigate/_action/send_goal");
        assert_eq!(topics.cancel_goal, "rq/robot1/navigate/_action/cancel_goal");
        assert_eq!(topics.get_result, "rq/robot1/navigate/_action/get_result");
        assert_eq!(topics.status, "rt/robot1/navigate/_action/status");
        assert_eq!(topics.feedback, "rt/robot1/navigate/_action/feedback");
    }

    #[test]
    fn test_action_server_topics() {
        let config = Ros2ActionConfig {
            namespace: Some("robot2".to_string()),
            ..Default::default()
        };
        let server: Ros2ActionServer<TestGoal, TestResult, TestFeedback> =
            Ros2ActionServer::new("arm_control", config);

        let topics = server.get_topics();
        assert_eq!(topics.send_goal, "rq/robot2/arm_control/_action/send_goal");
        assert_eq!(topics.status, "rt/robot2/arm_control/_action/status");
    }

    #[test]
    fn test_action_stats_comprehensive() {
        let config = Ros2ActionConfig::default();
        let server: Ros2ActionServer<TestGoal, TestResult, TestFeedback> =
            Ros2ActionServer::new("stats_action", config);

        // Process multiple goals with different outcomes
        let goal_id1 = GoalId::new();
        let goal_id2 = GoalId::new();
        let goal_id3 = GoalId::new();

        server.handle_goal_request(
            goal_id1,
            TestGoal {
                target_position: 10.0,
            },
        );
        server.handle_goal_request(
            goal_id2,
            TestGoal {
                target_position: 20.0,
            },
        );
        server.handle_goal_request(
            goal_id3,
            TestGoal {
                target_position: 30.0,
            },
        );

        // Different outcomes (stats must be recorded separately)
        server
            .get_goal_handle(goal_id1)
            .unwrap()
            .succeed(TestResult::default());
        server.stats().record_succeeded();
        server
            .get_goal_handle(goal_id2)
            .unwrap()
            .abort(TestResult::default());
        server.stats().record_aborted();
        server
            .get_goal_handle(goal_id3)
            .unwrap()
            .canceled(TestResult::default());
        server.stats().record_canceled();

        let stats = server.stats();
        assert_eq!(stats.goals_count.load(Ordering::Relaxed), 3);
        assert_eq!(stats.goals_accepted.load(Ordering::Relaxed), 3);
        assert_eq!(stats.goals_succeeded.load(Ordering::Relaxed), 1);
        assert_eq!(stats.goals_aborted.load(Ordering::Relaxed), 1);
        assert_eq!(stats.goals_canceled.load(Ordering::Relaxed), 1);

        // Success rate should be 1/3 = ~0.333
        assert!((stats.success_rate() - 0.333).abs() < 0.01);
    }

    #[test]
    fn test_action_goal_not_found_error() {
        let config = Ros2ActionConfig::default();
        let client: Ros2ActionClient<TestGoal, TestResult, TestFeedback> =
            Ros2ActionClient::new("error_action", config);

        // Try to cancel a non-existent goal
        let fake_goal_id = GoalId::new();
        let result = client.cancel_goal(fake_goal_id);
        assert!(matches!(result, Err(Ros2ActionError::GoalNotFound)));

        // Try to get result for non-existent goal
        let result = client.get_result(fake_goal_id);
        assert!(matches!(result, Err(Ros2ActionError::GoalNotFound)));
    }

    #[test]
    fn test_cancel_goal_request_cdr_roundtrip() {
        let request = CancelGoalRequest {
            goal_info: GoalInfo::new(),
        };
        let encoded = request.encode_cdr();
        let decoded = CancelGoalRequest::decode_cdr(&encoded).unwrap();
        assert_eq!(request.goal_info.goal_id, decoded.goal_info.goal_id);
    }

    #[test]
    fn test_cancel_goal_response_cdr_roundtrip() {
        let response = CancelGoalResponse::success(vec![GoalInfo::new(), GoalInfo::new()]);
        let encoded = response.encode_cdr();
        let decoded = CancelGoalResponse::decode_cdr(&encoded).unwrap();
        assert_eq!(response.return_code, decoded.return_code);
        assert_eq!(
            response.goals_canceling.len(),
            decoded.goals_canceling.len()
        );
    }
}
