//! Mission Planner Type Definitions
//!
//! This module defines the core types for mission planning including
//! missions, goals, tasks, and their relationships.

use std::collections::HashMap;
use std::fmt;
use std::time::{Duration, Instant};

use serde::{Deserialize, Serialize};
use thiserror::Error;

// ============================================================================
// Identifiers
// ============================================================================

/// Unique identifier for a mission.
#[derive(Debug, Clone, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub struct MissionId(String);

impl MissionId {
    /// Create a new mission ID.
    pub fn new<S: Into<String>>(id: S) -> Self {
        Self(id.into())
    }

    /// Generate a unique mission ID.
    pub fn generate() -> Self {
        Self(format!(
            "mission-{:016x}",
            std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap_or_default()
                .as_nanos() as u64
        ))
    }

    /// Get the ID as a string slice.
    pub fn as_str(&self) -> &str {
        &self.0
    }
}

impl fmt::Display for MissionId {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}", self.0)
    }
}

impl From<&str> for MissionId {
    fn from(s: &str) -> Self {
        Self::new(s)
    }
}

/// Unique identifier for a goal within a mission.
#[derive(Debug, Clone, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub struct GoalId(String);

impl GoalId {
    /// Create a new goal ID.
    pub fn new<S: Into<String>>(id: S) -> Self {
        Self(id.into())
    }

    /// Generate a unique goal ID.
    pub fn generate() -> Self {
        Self(format!(
            "goal-{:016x}",
            std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap_or_default()
                .as_nanos() as u64
        ))
    }

    /// Get the ID as a string slice.
    pub fn as_str(&self) -> &str {
        &self.0
    }
}

impl fmt::Display for GoalId {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}", self.0)
    }
}

impl From<&str> for GoalId {
    fn from(s: &str) -> Self {
        Self::new(s)
    }
}

/// Unique identifier for a task within a goal.
#[derive(Debug, Clone, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub struct TaskId(String);

impl TaskId {
    /// Create a new task ID.
    pub fn new<S: Into<String>>(id: S) -> Self {
        Self(id.into())
    }

    /// Generate a unique task ID.
    pub fn generate() -> Self {
        Self(format!(
            "task-{:016x}",
            std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap_or_default()
                .as_nanos() as u64
        ))
    }

    /// Get the ID as a string slice.
    pub fn as_str(&self) -> &str {
        &self.0
    }
}

impl fmt::Display for TaskId {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}", self.0)
    }
}

impl From<&str> for TaskId {
    fn from(s: &str) -> Self {
        Self::new(s)
    }
}

// ============================================================================
// Status and Priority
// ============================================================================

/// Status of a mission, goal, or task.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize, Default)]
pub enum ExecutionStatus {
    /// Not yet started.
    #[default]
    Pending,
    /// Waiting for dependencies to complete.
    Blocked,
    /// Ready to execute (all dependencies satisfied).
    Ready,
    /// Currently executing.
    Running,
    /// Temporarily paused.
    Paused,
    /// Completed successfully.
    Completed,
    /// Failed with an error.
    Failed,
    /// Cancelled by user or system.
    Cancelled,
    /// Skipped (e.g., optional task when condition not met).
    Skipped,
}

impl ExecutionStatus {
    /// Check if this status represents a terminal state.
    pub fn is_terminal(&self) -> bool {
        matches!(
            self,
            Self::Completed | Self::Failed | Self::Cancelled | Self::Skipped
        )
    }

    /// Check if this status represents an active state.
    pub fn is_active(&self) -> bool {
        matches!(self, Self::Running | Self::Paused)
    }

    /// Check if this status allows execution to proceed.
    pub fn can_execute(&self) -> bool {
        matches!(self, Self::Ready | Self::Running | Self::Paused)
    }
}

/// Priority level for missions, goals, and tasks.
#[derive(
    Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize, Default,
)]
pub enum Priority {
    /// Lowest priority, executed when nothing else is pending.
    Low = 0,
    /// Normal priority for regular operations.
    #[default]
    Normal = 1,
    /// Higher priority for important tasks.
    High = 2,
    /// Critical priority for urgent tasks.
    Critical = 3,
    /// Emergency priority, preempts everything else.
    Emergency = 4,
}

impl From<u8> for Priority {
    fn from(value: u8) -> Self {
        match value {
            0 => Self::Low,
            1 => Self::Normal,
            2 => Self::High,
            3 => Self::Critical,
            _ => Self::Emergency,
        }
    }
}

// ============================================================================
// Task Specification
// ============================================================================

/// Type of task executor.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum TaskExecutor {
    /// Execute using an action server.
    Action {
        /// Name of the action type.
        action_type: String,
        /// Action parameters.
        parameters: HashMap<String, serde_json::Value>,
    },
    /// Execute using a behavior tree.
    BehaviorTree {
        /// Name of the behavior tree.
        tree_name: String,
        /// Initial blackboard values.
        blackboard: HashMap<String, serde_json::Value>,
    },
    /// Execute using a state machine.
    StateMachine {
        /// Name of the state machine.
        machine_name: String,
        /// Target state to reach.
        target_state: String,
    },
    /// Execute using a custom function.
    Custom {
        /// Handler name.
        handler: String,
        /// Handler parameters.
        parameters: HashMap<String, serde_json::Value>,
    },
    /// Execute a shell command.
    Command {
        /// Command to execute.
        command: String,
        /// Command arguments.
        args: Vec<String>,
        /// Working directory.
        working_dir: Option<String>,
    },
    /// No-op task (for synchronization points).
    Noop,
}

/// Condition that must be met for a task to execute.
#[derive(Debug, Clone, Serialize, Deserialize, Default)]
pub enum TaskCondition {
    /// Always execute.
    #[default]
    Always,
    /// Never execute (disabled).
    Never,
    /// Execute if a parameter matches a value.
    ParameterEquals {
        parameter: String,
        value: serde_json::Value,
    },
    /// Execute if a parameter exists.
    ParameterExists { parameter: String },
    /// Execute if previous task succeeded.
    PreviousSucceeded { task_id: TaskId },
    /// Execute if all of these conditions are met.
    All(Vec<TaskCondition>),
    /// Execute if any of these conditions are met.
    Any(Vec<TaskCondition>),
    /// Negate a condition.
    Not(Box<TaskCondition>),
    /// Custom condition with a named predicate.
    Custom { name: String, args: Vec<String> },
}

/// Retry policy for failed tasks.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RetryPolicy {
    /// Maximum number of retry attempts.
    pub max_attempts: u32,
    /// Delay between retries.
    pub delay: Duration,
    /// Whether to use exponential backoff.
    pub exponential_backoff: bool,
    /// Maximum delay when using exponential backoff.
    pub max_delay: Duration,
    /// Error types that should trigger a retry.
    pub retry_on: Vec<String>,
}

impl Default for RetryPolicy {
    fn default() -> Self {
        Self {
            max_attempts: 3,
            delay: Duration::from_secs(1),
            exponential_backoff: true,
            max_delay: Duration::from_secs(30),
            retry_on: Vec::new(),
        }
    }
}

/// Specification for a single task.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TaskSpec {
    /// Unique identifier for this task.
    pub id: TaskId,
    /// Human-readable name.
    pub name: String,
    /// Optional description.
    pub description: Option<String>,
    /// Task priority.
    pub priority: Priority,
    /// How to execute this task.
    pub executor: TaskExecutor,
    /// Condition for execution.
    pub condition: TaskCondition,
    /// Maximum execution time.
    pub timeout: Option<Duration>,
    /// Retry policy on failure.
    pub retry_policy: Option<RetryPolicy>,
    /// Whether this task is optional.
    pub optional: bool,
    /// Whether this task can be preempted.
    pub preemptible: bool,
    /// Custom metadata.
    pub metadata: HashMap<String, serde_json::Value>,
}

impl TaskSpec {
    /// Create a new task specification.
    pub fn new<S: Into<String>>(id: S, name: S, executor: TaskExecutor) -> Self {
        Self {
            id: TaskId::new(id),
            name: name.into(),
            description: None,
            priority: Priority::Normal,
            executor,
            condition: TaskCondition::Always,
            timeout: None,
            retry_policy: None,
            optional: false,
            preemptible: true,
            metadata: HashMap::new(),
        }
    }

    /// Create a no-op task (for synchronization).
    pub fn noop<S: Into<String>>(id: S, name: S) -> Self {
        Self::new(id, name, TaskExecutor::Noop)
    }

    /// Set the description.
    pub fn with_description<S: Into<String>>(mut self, description: S) -> Self {
        self.description = Some(description.into());
        self
    }

    /// Set the priority.
    pub fn with_priority(mut self, priority: Priority) -> Self {
        self.priority = priority;
        self
    }

    /// Set the execution condition.
    pub fn with_condition(mut self, condition: TaskCondition) -> Self {
        self.condition = condition;
        self
    }

    /// Set the timeout.
    pub fn with_timeout(mut self, timeout: Duration) -> Self {
        self.timeout = Some(timeout);
        self
    }

    /// Set the retry policy.
    pub fn with_retry(mut self, policy: RetryPolicy) -> Self {
        self.retry_policy = Some(policy);
        self
    }

    /// Mark as optional.
    pub fn optional(mut self) -> Self {
        self.optional = true;
        self
    }

    /// Mark as non-preemptible.
    pub fn non_preemptible(mut self) -> Self {
        self.preemptible = false;
        self
    }

    /// Add metadata.
    pub fn with_metadata<K: Into<String>>(mut self, key: K, value: serde_json::Value) -> Self {
        self.metadata.insert(key.into(), value);
        self
    }
}

// ============================================================================
// Goal Specification
// ============================================================================

/// What should happen when a goal fails.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize, Default)]
pub enum GoalFailurePolicy {
    /// Continue with other goals in the mission.
    Continue,
    /// Pause the mission and wait for intervention.
    #[default]
    Pause,
    /// Abort the entire mission.
    AbortMission,
    /// Retry the goal from the beginning.
    Retry,
}

/// Specification for a goal (a collection of related tasks).
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct GoalSpec {
    /// Unique identifier for this goal.
    pub id: GoalId,
    /// Human-readable name.
    pub name: String,
    /// Optional description.
    pub description: Option<String>,
    /// Goal priority.
    pub priority: Priority,
    /// Tasks that make up this goal.
    pub tasks: Vec<TaskSpec>,
    /// Task dependencies (task_id -> list of dependencies).
    pub dependencies: HashMap<TaskId, Vec<TaskId>>,
    /// Condition for execution.
    pub condition: TaskCondition,
    /// What to do on failure.
    pub failure_policy: GoalFailurePolicy,
    /// Maximum execution time for the entire goal.
    pub timeout: Option<Duration>,
    /// Whether this goal is optional.
    pub optional: bool,
    /// Custom metadata.
    pub metadata: HashMap<String, serde_json::Value>,
}

impl GoalSpec {
    /// Create a new goal specification.
    pub fn new<S: Into<String>>(id: S, name: S) -> Self {
        Self {
            id: GoalId::new(id),
            name: name.into(),
            description: None,
            priority: Priority::Normal,
            tasks: Vec::new(),
            dependencies: HashMap::new(),
            condition: TaskCondition::Always,
            failure_policy: GoalFailurePolicy::Pause,
            timeout: None,
            optional: false,
            metadata: HashMap::new(),
        }
    }

    /// Set the description.
    pub fn with_description<S: Into<String>>(mut self, description: S) -> Self {
        self.description = Some(description.into());
        self
    }

    /// Set the priority.
    pub fn with_priority(mut self, priority: Priority) -> Self {
        self.priority = priority;
        self
    }

    /// Add a task with no dependencies.
    pub fn add_task(mut self, task: TaskSpec) -> Self {
        self.tasks.push(task);
        self
    }

    /// Add a task that depends on another task.
    pub fn add_task_after(mut self, task: TaskSpec, after: &TaskId) -> Self {
        let task_id = task.id.clone();
        self.tasks.push(task);
        self.dependencies
            .entry(task_id)
            .or_default()
            .push(after.clone());
        self
    }

    /// Add a task that depends on multiple tasks.
    pub fn add_task_after_all(mut self, task: TaskSpec, after: &[TaskId]) -> Self {
        let task_id = task.id.clone();
        self.tasks.push(task);
        self.dependencies
            .entry(task_id)
            .or_default()
            .extend(after.iter().cloned());
        self
    }

    /// Set the failure policy.
    pub fn with_failure_policy(mut self, policy: GoalFailurePolicy) -> Self {
        self.failure_policy = policy;
        self
    }

    /// Set the timeout.
    pub fn with_timeout(mut self, timeout: Duration) -> Self {
        self.timeout = Some(timeout);
        self
    }

    /// Mark as optional.
    pub fn optional(mut self) -> Self {
        self.optional = true;
        self
    }

    /// Get a task by ID.
    pub fn get_task(&self, id: &TaskId) -> Option<&TaskSpec> {
        self.tasks.iter().find(|t| &t.id == id)
    }

    /// Get tasks with no dependencies (entry points).
    pub fn entry_tasks(&self) -> Vec<&TaskSpec> {
        self.tasks
            .iter()
            .filter(|t| {
                self.dependencies
                    .get(&t.id)
                    .is_none_or(|deps| deps.is_empty())
            })
            .collect()
    }
}

// ============================================================================
// Mission Specification
// ============================================================================

/// Mode of mission execution.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize, Default)]
pub enum MissionMode {
    /// Execute goals sequentially in order.
    #[default]
    Sequential,
    /// Execute goals in parallel where possible.
    Parallel,
    /// Execute goals opportunistically based on conditions.
    Opportunistic,
}

/// Specification for a complete mission.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MissionSpec {
    /// Unique identifier for this mission.
    pub id: MissionId,
    /// Human-readable name.
    pub name: String,
    /// Optional description.
    pub description: Option<String>,
    /// Mission priority.
    pub priority: Priority,
    /// Execution mode.
    pub mode: MissionMode,
    /// Goals that make up this mission.
    pub goals: Vec<GoalSpec>,
    /// Goal dependencies (goal_id -> list of dependencies).
    pub dependencies: HashMap<GoalId, Vec<GoalId>>,
    /// Maximum execution time for the entire mission.
    pub timeout: Option<Duration>,
    /// Whether to continue from failures.
    pub continue_on_failure: bool,
    /// Custom metadata.
    pub metadata: HashMap<String, serde_json::Value>,
}

impl MissionSpec {
    /// Create a new mission specification.
    pub fn new<S: Into<String>>(name: S) -> Self {
        Self {
            id: MissionId::generate(),
            name: name.into(),
            description: None,
            priority: Priority::Normal,
            mode: MissionMode::Sequential,
            goals: Vec::new(),
            dependencies: HashMap::new(),
            timeout: None,
            continue_on_failure: false,
            metadata: HashMap::new(),
        }
    }

    /// Create a mission with a specific ID.
    pub fn with_id<S: Into<String>>(id: S, name: S) -> Self {
        Self {
            id: MissionId::new(id),
            name: name.into(),
            description: None,
            priority: Priority::Normal,
            mode: MissionMode::Sequential,
            goals: Vec::new(),
            dependencies: HashMap::new(),
            timeout: None,
            continue_on_failure: false,
            metadata: HashMap::new(),
        }
    }

    /// Set the description.
    pub fn with_description<S: Into<String>>(mut self, description: S) -> Self {
        self.description = Some(description.into());
        self
    }

    /// Set the priority.
    pub fn with_priority(mut self, priority: Priority) -> Self {
        self.priority = priority;
        self
    }

    /// Set the execution mode.
    pub fn with_mode(mut self, mode: MissionMode) -> Self {
        self.mode = mode;
        self
    }

    /// Add a goal with no dependencies.
    pub fn add_goal(mut self, goal: GoalSpec) -> Self {
        self.goals.push(goal);
        self
    }

    /// Add a goal that depends on another goal.
    pub fn add_goal_after(mut self, goal: GoalSpec, after: &GoalId) -> Self {
        let goal_id = goal.id.clone();
        self.goals.push(goal);
        self.dependencies
            .entry(goal_id)
            .or_default()
            .push(after.clone());
        self
    }

    /// Set the timeout.
    pub fn with_timeout(mut self, timeout: Duration) -> Self {
        self.timeout = Some(timeout);
        self
    }

    /// Enable continue-on-failure mode.
    pub fn continue_on_failure(mut self) -> Self {
        self.continue_on_failure = true;
        self
    }

    /// Get a goal by ID.
    pub fn get_goal(&self, id: &GoalId) -> Option<&GoalSpec> {
        self.goals.iter().find(|g| &g.id == id)
    }

    /// Get goals with no dependencies (entry points).
    pub fn entry_goals(&self) -> Vec<&GoalSpec> {
        self.goals
            .iter()
            .filter(|g| {
                self.dependencies
                    .get(&g.id)
                    .is_none_or(|deps| deps.is_empty())
            })
            .collect()
    }
}

// ============================================================================
// Runtime State
// ============================================================================

/// Runtime state for a task.
#[derive(Debug, Clone)]
pub struct TaskState {
    /// Task specification.
    pub spec: TaskSpec,
    /// Current status.
    pub status: ExecutionStatus,
    /// Number of retry attempts.
    pub retry_count: u32,
    /// When execution started.
    pub started_at: Option<Instant>,
    /// When execution completed.
    pub completed_at: Option<Instant>,
    /// Error message if failed.
    pub error: Option<String>,
    /// Task result data.
    pub result: Option<serde_json::Value>,
}

impl TaskState {
    /// Create a new task state from a spec.
    pub fn new(spec: TaskSpec) -> Self {
        Self {
            spec,
            status: ExecutionStatus::Pending,
            retry_count: 0,
            started_at: None,
            completed_at: None,
            error: None,
            result: None,
        }
    }

    /// Get the task ID.
    pub fn id(&self) -> &TaskId {
        &self.spec.id
    }

    /// Get the execution duration.
    pub fn duration(&self) -> Option<Duration> {
        match (self.started_at, self.completed_at) {
            (Some(start), Some(end)) => Some(end.duration_since(start)),
            (Some(start), None) => Some(start.elapsed()),
            _ => None,
        }
    }
}

/// Runtime state for a goal.
#[derive(Debug, Clone)]
pub struct GoalState {
    /// Goal specification.
    pub spec: GoalSpec,
    /// Current status.
    pub status: ExecutionStatus,
    /// Task states.
    pub tasks: HashMap<TaskId, TaskState>,
    /// When execution started.
    pub started_at: Option<Instant>,
    /// When execution completed.
    pub completed_at: Option<Instant>,
    /// Error message if failed.
    pub error: Option<String>,
}

impl GoalState {
    /// Create a new goal state from a spec.
    pub fn new(spec: GoalSpec) -> Self {
        let tasks = spec
            .tasks
            .iter()
            .map(|t| (t.id.clone(), TaskState::new(t.clone())))
            .collect();

        Self {
            spec,
            status: ExecutionStatus::Pending,
            tasks,
            started_at: None,
            completed_at: None,
            error: None,
        }
    }

    /// Get the goal ID.
    pub fn id(&self) -> &GoalId {
        &self.spec.id
    }

    /// Get a task state by ID.
    pub fn get_task(&self, id: &TaskId) -> Option<&TaskState> {
        self.tasks.get(id)
    }

    /// Get a mutable task state by ID.
    pub fn get_task_mut(&mut self, id: &TaskId) -> Option<&mut TaskState> {
        self.tasks.get_mut(id)
    }

    /// Get all tasks with a given status.
    pub fn tasks_with_status(&self, status: ExecutionStatus) -> Vec<&TaskState> {
        self.tasks.values().filter(|t| t.status == status).collect()
    }

    /// Check if all tasks are complete (success, failed, skipped, or cancelled).
    pub fn all_tasks_terminal(&self) -> bool {
        self.tasks.values().all(|t| t.status.is_terminal())
    }

    /// Check if any task has failed.
    pub fn has_failed_task(&self) -> bool {
        self.tasks
            .values()
            .any(|t| t.status == ExecutionStatus::Failed)
    }

    /// Get completion progress (0.0 to 1.0).
    pub fn progress(&self) -> f64 {
        let total = self.tasks.len();
        if total == 0 {
            return 1.0;
        }
        let completed = self
            .tasks
            .values()
            .filter(|t| t.status.is_terminal())
            .count();
        completed as f64 / total as f64
    }
}

/// Runtime state for a mission.
#[derive(Debug, Clone)]
pub struct MissionState {
    /// Mission specification.
    pub spec: MissionSpec,
    /// Current status.
    pub status: ExecutionStatus,
    /// Goal states.
    pub goals: HashMap<GoalId, GoalState>,
    /// When execution started.
    pub started_at: Option<Instant>,
    /// When execution completed.
    pub completed_at: Option<Instant>,
    /// Error message if failed.
    pub error: Option<String>,
}

impl MissionState {
    /// Create a new mission state from a spec.
    pub fn new(spec: MissionSpec) -> Self {
        let goals = spec
            .goals
            .iter()
            .map(|g| (g.id.clone(), GoalState::new(g.clone())))
            .collect();

        Self {
            spec,
            status: ExecutionStatus::Pending,
            goals,
            started_at: None,
            completed_at: None,
            error: None,
        }
    }

    /// Get the mission ID.
    pub fn id(&self) -> &MissionId {
        &self.spec.id
    }

    /// Get a goal state by ID.
    pub fn get_goal(&self, id: &GoalId) -> Option<&GoalState> {
        self.goals.get(id)
    }

    /// Get a mutable goal state by ID.
    pub fn get_goal_mut(&mut self, id: &GoalId) -> Option<&mut GoalState> {
        self.goals.get_mut(id)
    }

    /// Get all goals with a given status.
    pub fn goals_with_status(&self, status: ExecutionStatus) -> Vec<&GoalState> {
        self.goals.values().filter(|g| g.status == status).collect()
    }

    /// Check if all goals are complete.
    pub fn all_goals_terminal(&self) -> bool {
        self.goals.values().all(|g| g.status.is_terminal())
    }

    /// Get completion progress (0.0 to 1.0).
    pub fn progress(&self) -> f64 {
        let total = self.goals.len();
        if total == 0 {
            return 1.0;
        }
        let total_progress: f64 = self.goals.values().map(|g| g.progress()).sum();
        total_progress / total as f64
    }

    /// Get the execution duration.
    pub fn duration(&self) -> Option<Duration> {
        match (self.started_at, self.completed_at) {
            (Some(start), Some(end)) => Some(end.duration_since(start)),
            (Some(start), None) => Some(start.elapsed()),
            _ => None,
        }
    }
}

// ============================================================================
// Errors
// ============================================================================

/// Errors that can occur during mission planning and execution.
#[derive(Debug, Error)]
pub enum MissionPlannerError {
    /// Mission not found.
    #[error("Mission not found: {0}")]
    MissionNotFound(MissionId),

    /// Goal not found.
    #[error("Goal not found: {0}")]
    GoalNotFound(GoalId),

    /// Task not found.
    #[error("Task not found: {0}")]
    TaskNotFound(TaskId),

    /// Cyclic dependency detected.
    #[error("Cyclic dependency detected: {0}")]
    CyclicDependency(String),

    /// Invalid dependency reference.
    #[error("Invalid dependency: {from} depends on non-existent {to}")]
    InvalidDependency { from: String, to: String },

    /// Mission already running.
    #[error("Mission already running: {0}")]
    AlreadyRunning(MissionId),

    /// Mission not running.
    #[error("Mission not running: {0}")]
    NotRunning(MissionId),

    /// Task execution failed.
    #[error("Task execution failed: {task_id} - {reason}")]
    TaskFailed { task_id: TaskId, reason: String },

    /// Goal execution failed.
    #[error("Goal execution failed: {goal_id} - {reason}")]
    GoalFailed { goal_id: GoalId, reason: String },

    /// Mission timed out.
    #[error("Mission timed out: {0}")]
    Timeout(MissionId),

    /// Invalid mission specification.
    #[error("Invalid mission specification: {0}")]
    InvalidSpec(String),

    /// Executor error.
    #[error("Executor error: {0}")]
    ExecutorError(String),

    /// Internal error.
    #[error("Internal error: {0}")]
    Internal(String),
}

// ============================================================================
// Metrics
// ============================================================================

/// Metrics for mission execution.
#[derive(Debug, Clone, Default)]
pub struct MissionMetrics {
    /// Total missions executed.
    pub total_missions: u64,
    /// Successful missions.
    pub successful_missions: u64,
    /// Failed missions.
    pub failed_missions: u64,
    /// Cancelled missions.
    pub cancelled_missions: u64,
    /// Total goals executed.
    pub total_goals: u64,
    /// Successful goals.
    pub successful_goals: u64,
    /// Failed goals.
    pub failed_goals: u64,
    /// Total tasks executed.
    pub total_tasks: u64,
    /// Successful tasks.
    pub successful_tasks: u64,
    /// Failed tasks.
    pub failed_tasks: u64,
    /// Total retries.
    pub total_retries: u64,
    /// Total execution time.
    pub total_execution_time: Duration,
    /// Average mission duration.
    pub avg_mission_duration: Duration,
}

impl MissionMetrics {
    /// Reset all metrics to zero.
    pub fn reset(&mut self) {
        *self = Self::default();
    }

    /// Record a completed mission.
    pub fn record_mission(&mut self, success: bool, duration: Duration) {
        self.total_missions += 1;
        if success {
            self.successful_missions += 1;
        } else {
            self.failed_missions += 1;
        }
        self.total_execution_time += duration;
        if self.total_missions > 0 {
            self.avg_mission_duration = self.total_execution_time
                / (self.total_missions.min(u32::MAX as u64) as u32).max(1);
        }
    }

    /// Record a completed goal.
    pub fn record_goal(&mut self, success: bool) {
        self.total_goals += 1;
        if success {
            self.successful_goals += 1;
        } else {
            self.failed_goals += 1;
        }
    }

    /// Record a completed task.
    pub fn record_task(&mut self, success: bool) {
        self.total_tasks += 1;
        if success {
            self.successful_tasks += 1;
        } else {
            self.failed_tasks += 1;
        }
    }

    /// Record a retry.
    pub fn record_retry(&mut self) {
        self.total_retries += 1;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_mission_id() {
        let id = MissionId::new("test-mission");
        assert_eq!(id.as_str(), "test-mission");
        assert_eq!(format!("{}", id), "test-mission");
    }

    #[test]
    fn test_execution_status() {
        assert!(ExecutionStatus::Completed.is_terminal());
        assert!(ExecutionStatus::Failed.is_terminal());
        assert!(!ExecutionStatus::Running.is_terminal());
        assert!(ExecutionStatus::Running.is_active());
        assert!(ExecutionStatus::Ready.can_execute());
    }

    #[test]
    fn test_task_spec_builder() {
        let task = TaskSpec::new("task1", "Test Task", TaskExecutor::Noop)
            .with_description("A test task")
            .with_priority(Priority::High)
            .with_timeout(Duration::from_secs(30))
            .optional();

        assert_eq!(task.id.as_str(), "task1");
        assert_eq!(task.name, "Test Task");
        assert_eq!(task.priority, Priority::High);
        assert!(task.optional);
    }

    #[test]
    fn test_goal_spec_builder() {
        let task1 = TaskSpec::noop("t1", "Task 1");
        let task2 = TaskSpec::noop("t2", "Task 2");
        let task1_id = task1.id.clone();

        let goal = GoalSpec::new("goal1", "Test Goal")
            .add_task(task1)
            .add_task_after(task2, &task1_id);

        assert_eq!(goal.tasks.len(), 2);
        assert_eq!(goal.dependencies.get(&TaskId::new("t2")).unwrap().len(), 1);
    }

    #[test]
    fn test_mission_spec_builder() {
        let goal1 = GoalSpec::new("g1", "Goal 1");
        let goal2 = GoalSpec::new("g2", "Goal 2");
        let goal1_id = goal1.id.clone();

        let mission = MissionSpec::new("Test Mission")
            .with_description("A test mission")
            .with_mode(MissionMode::Parallel)
            .add_goal(goal1)
            .add_goal_after(goal2, &goal1_id);

        assert_eq!(mission.goals.len(), 2);
        assert_eq!(mission.mode, MissionMode::Parallel);
    }

    #[test]
    fn test_mission_state() {
        let goal = GoalSpec::new("g1", "Goal 1")
            .add_task(TaskSpec::noop("t1", "Task 1"))
            .add_task(TaskSpec::noop("t2", "Task 2"));

        let mission = MissionSpec::new("Test").add_goal(goal);
        let state = MissionState::new(mission);

        assert_eq!(state.status, ExecutionStatus::Pending);
        assert_eq!(state.goals.len(), 1);
        let goal_state = state.goals.values().next().unwrap();
        assert_eq!(goal_state.tasks.len(), 2);
    }

    #[test]
    fn test_metrics() {
        let mut metrics = MissionMetrics::default();
        metrics.record_mission(true, Duration::from_secs(10));
        metrics.record_mission(false, Duration::from_secs(5));

        assert_eq!(metrics.total_missions, 2);
        assert_eq!(metrics.successful_missions, 1);
        assert_eq!(metrics.failed_missions, 1);
    }
}
