//! Core types for HORUS Behavior Trees.
//!
//! This module provides the foundational types for building behavior trees,
//! a common pattern for robot task orchestration.
//!
//! # Key Concepts
//!
//! - **NodeStatus**: The result of ticking a node (Running, Success, Failure)
//! - **NodeId**: Unique identifier for nodes in the tree
//! - **Blackboard**: Shared data storage for nodes to communicate

use serde::{Deserialize, Serialize};
use std::collections::HashMap;
use std::fmt;
use std::time::Duration;

/// The status returned by a behavior tree node after being ticked.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum NodeStatus {
    /// The node is still executing and needs more ticks.
    Running,
    /// The node completed successfully.
    Success,
    /// The node failed.
    Failure,
}

impl NodeStatus {
    /// Check if the status is Running.
    pub fn is_running(&self) -> bool {
        matches!(self, NodeStatus::Running)
    }

    /// Check if the status is Success.
    pub fn is_success(&self) -> bool {
        matches!(self, NodeStatus::Success)
    }

    /// Check if the status is Failure.
    pub fn is_failure(&self) -> bool {
        matches!(self, NodeStatus::Failure)
    }

    /// Check if the status is terminal (Success or Failure).
    pub fn is_terminal(&self) -> bool {
        !self.is_running()
    }

    /// Invert the status (Success <-> Failure, Running stays Running).
    pub fn invert(&self) -> Self {
        match self {
            NodeStatus::Success => NodeStatus::Failure,
            NodeStatus::Failure => NodeStatus::Success,
            NodeStatus::Running => NodeStatus::Running,
        }
    }
}

impl fmt::Display for NodeStatus {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            NodeStatus::Running => write!(f, "Running"),
            NodeStatus::Success => write!(f, "Success"),
            NodeStatus::Failure => write!(f, "Failure"),
        }
    }
}

/// Unique identifier for a behavior tree node.
#[derive(Clone, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub struct NodeId(String);

impl NodeId {
    /// Create a new node ID.
    pub fn new(id: impl Into<String>) -> Self {
        NodeId(id.into())
    }

    /// Generate a unique node ID.
    pub fn generate() -> Self {
        NodeId(uuid::Uuid::new_v4().to_string())
    }

    /// Get the ID as a string slice.
    pub fn as_str(&self) -> &str {
        &self.0
    }
}

impl fmt::Display for NodeId {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}", self.0)
    }
}

impl From<&str> for NodeId {
    fn from(s: &str) -> Self {
        NodeId::new(s)
    }
}

impl From<String> for NodeId {
    fn from(s: String) -> Self {
        NodeId(s)
    }
}

/// A value that can be stored in the blackboard.
#[derive(Clone, Debug)]
pub enum BlackboardValue {
    /// Boolean value
    Bool(bool),
    /// Integer value
    Int(i64),
    /// Float value
    Float(f64),
    /// String value
    String(String),
    /// Vector of floats (for positions, etc.)
    Vec3([f64; 3]),
    /// Generic bytes
    Bytes(Vec<u8>),
    /// JSON value for complex data
    Json(serde_json::Value),
}

impl BlackboardValue {
    /// Try to get as bool.
    pub fn as_bool(&self) -> Option<bool> {
        match self {
            BlackboardValue::Bool(v) => Some(*v),
            _ => None,
        }
    }

    /// Try to get as int.
    pub fn as_int(&self) -> Option<i64> {
        match self {
            BlackboardValue::Int(v) => Some(*v),
            _ => None,
        }
    }

    /// Try to get as float.
    pub fn as_float(&self) -> Option<f64> {
        match self {
            BlackboardValue::Float(v) => Some(*v),
            BlackboardValue::Int(v) => Some(*v as f64),
            _ => None,
        }
    }

    /// Try to get as string.
    pub fn as_string(&self) -> Option<&str> {
        match self {
            BlackboardValue::String(v) => Some(v),
            _ => None,
        }
    }

    /// Try to get as Vec3.
    pub fn as_vec3(&self) -> Option<[f64; 3]> {
        match self {
            BlackboardValue::Vec3(v) => Some(*v),
            _ => None,
        }
    }
}

impl From<bool> for BlackboardValue {
    fn from(v: bool) -> Self {
        BlackboardValue::Bool(v)
    }
}

impl From<i64> for BlackboardValue {
    fn from(v: i64) -> Self {
        BlackboardValue::Int(v)
    }
}

impl From<i32> for BlackboardValue {
    fn from(v: i32) -> Self {
        BlackboardValue::Int(v as i64)
    }
}

impl From<f64> for BlackboardValue {
    fn from(v: f64) -> Self {
        BlackboardValue::Float(v)
    }
}

impl From<f32> for BlackboardValue {
    fn from(v: f32) -> Self {
        BlackboardValue::Float(v as f64)
    }
}

impl From<String> for BlackboardValue {
    fn from(v: String) -> Self {
        BlackboardValue::String(v)
    }
}

impl From<&str> for BlackboardValue {
    fn from(v: &str) -> Self {
        BlackboardValue::String(v.to_string())
    }
}

impl From<[f64; 3]> for BlackboardValue {
    fn from(v: [f64; 3]) -> Self {
        BlackboardValue::Vec3(v)
    }
}

/// Shared data storage for behavior tree nodes.
///
/// The blackboard allows nodes to communicate by reading and writing
/// named values. This enables loose coupling between nodes.
#[derive(Clone, Debug, Default)]
pub struct Blackboard {
    values: HashMap<String, BlackboardValue>,
}

impl Blackboard {
    /// Create a new empty blackboard.
    pub fn new() -> Self {
        Self::default()
    }

    /// Set a value in the blackboard.
    pub fn set(&mut self, key: impl Into<String>, value: impl Into<BlackboardValue>) {
        self.values.insert(key.into(), value.into());
    }

    /// Get a value from the blackboard.
    pub fn get(&self, key: &str) -> Option<&BlackboardValue> {
        self.values.get(key)
    }

    /// Get a boolean value.
    pub fn get_bool(&self, key: &str) -> Option<bool> {
        self.get(key).and_then(|v| v.as_bool())
    }

    /// Get an integer value.
    pub fn get_int(&self, key: &str) -> Option<i64> {
        self.get(key).and_then(|v| v.as_int())
    }

    /// Get a float value.
    pub fn get_float(&self, key: &str) -> Option<f64> {
        self.get(key).and_then(|v| v.as_float())
    }

    /// Get a string value.
    pub fn get_string(&self, key: &str) -> Option<&str> {
        self.get(key).and_then(|v| v.as_string())
    }

    /// Get a Vec3 value.
    pub fn get_vec3(&self, key: &str) -> Option<[f64; 3]> {
        self.get(key).and_then(|v| v.as_vec3())
    }

    /// Check if a key exists.
    pub fn contains(&self, key: &str) -> bool {
        self.values.contains_key(key)
    }

    /// Remove a value.
    pub fn remove(&mut self, key: &str) -> Option<BlackboardValue> {
        self.values.remove(key)
    }

    /// Clear all values.
    pub fn clear(&mut self) {
        self.values.clear();
    }

    /// Get all keys.
    pub fn keys(&self) -> impl Iterator<Item = &String> {
        self.values.keys()
    }

    /// Get the number of entries.
    pub fn len(&self) -> usize {
        self.values.len()
    }

    /// Check if empty.
    pub fn is_empty(&self) -> bool {
        self.values.is_empty()
    }
}

/// Context passed to behavior tree nodes during execution.
pub struct TickContext<'a, C> {
    /// The shared blackboard for inter-node communication.
    pub blackboard: &'a mut Blackboard,
    /// The user-defined context (robot state, etc.).
    pub context: &'a mut C,
    /// Time since the tree started ticking.
    pub elapsed: Duration,
    /// Delta time since last tick.
    pub delta_time: Duration,
    /// Current tick number.
    pub tick_count: u64,
}

impl<'a, C> TickContext<'a, C> {
    /// Create a new tick context.
    pub fn new(
        blackboard: &'a mut Blackboard,
        context: &'a mut C,
        elapsed: Duration,
        delta_time: Duration,
        tick_count: u64,
    ) -> Self {
        Self {
            blackboard,
            context,
            elapsed,
            delta_time,
            tick_count,
        }
    }
}

/// Configuration for behavior tree execution.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct BehaviorTreeConfig {
    /// Name of the tree.
    pub name: String,
    /// Maximum ticks per second (0 for unlimited).
    pub max_ticks_per_second: u32,
    /// Whether to reset running nodes on tree restart.
    pub reset_on_restart: bool,
    /// Enable debug logging.
    pub debug_logging: bool,
    /// Maximum tree depth (to prevent infinite recursion).
    pub max_depth: usize,
    /// Tick rate (time between ticks).
    #[serde(with = "optional_duration_serde", default)]
    pub tick_rate: Option<Duration>,
    /// Maximum time allowed for a single tick.
    #[serde(with = "optional_duration_serde", default)]
    pub max_tick_time: Option<Duration>,
    /// Whether to reset the tree when it completes (success or failure).
    pub reset_on_complete: bool,
    /// Whether to collect detailed metrics.
    pub collect_metrics: bool,
    /// Enable debug mode (verbose output).
    pub debug_mode: bool,
}

impl Default for BehaviorTreeConfig {
    fn default() -> Self {
        Self {
            name: "behavior_tree".to_string(),
            max_ticks_per_second: 60,
            reset_on_restart: true,
            debug_logging: false,
            max_depth: 100,
            tick_rate: None,
            max_tick_time: None,
            reset_on_complete: false,
            collect_metrics: true,
            debug_mode: false,
        }
    }
}

impl BehaviorTreeConfig {
    /// Create a new config with the given name.
    pub fn new(name: impl Into<String>) -> Self {
        Self {
            name: name.into(),
            ..Default::default()
        }
    }

    /// Set max ticks per second.
    pub fn with_max_ticks(mut self, ticks: u32) -> Self {
        self.max_ticks_per_second = ticks;
        self
    }

    /// Enable debug logging.
    pub fn with_debug(mut self, debug: bool) -> Self {
        self.debug_logging = debug;
        self
    }
}

/// Serde helper for optional Duration fields.
mod optional_duration_serde {
    use serde::{Deserialize, Deserializer, Serialize, Serializer};
    use std::time::Duration;

    pub fn serialize<S>(opt: &Option<Duration>, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: Serializer,
    {
        match opt {
            Some(duration) => duration.as_nanos().serialize(serializer),
            None => serializer.serialize_none(),
        }
    }

    pub fn deserialize<'de, D>(deserializer: D) -> Result<Option<Duration>, D::Error>
    where
        D: Deserializer<'de>,
    {
        let opt: Option<u128> = Option::deserialize(deserializer)?;
        Ok(opt.map(|nanos| Duration::from_nanos(nanos as u64)))
    }
}

/// Metrics for behavior tree monitoring.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct BehaviorTreeMetrics {
    /// Total number of ticks.
    pub total_ticks: u64,
    /// Number of successful completions.
    pub success_count: u64,
    /// Number of failures.
    pub failure_count: u64,
    /// Number of times the tree was in running state.
    pub running_count: u64,
    /// Average tick duration (nanoseconds).
    pub avg_tick_duration_ns: u64,
    /// Maximum tick duration (nanoseconds).
    pub max_tick_duration_ns: u64,
    /// Total time spent ticking.
    #[serde(with = "duration_serde")]
    pub total_tick_time: Duration,
    /// Maximum tick time.
    #[serde(with = "duration_serde")]
    pub max_tick_time: Duration,
    /// Minimum tick time.
    #[serde(with = "duration_serde")]
    pub min_tick_time: Duration,
    /// Node execution counts by ID.
    pub node_exec_counts: HashMap<String, u64>,
    /// Node success counts by ID.
    pub node_success_counts: HashMap<String, u64>,
    /// Node failure counts by ID.
    pub node_failure_counts: HashMap<String, u64>,
}

impl Default for BehaviorTreeMetrics {
    fn default() -> Self {
        Self {
            total_ticks: 0,
            success_count: 0,
            failure_count: 0,
            running_count: 0,
            avg_tick_duration_ns: 0,
            max_tick_duration_ns: 0,
            total_tick_time: Duration::ZERO,
            max_tick_time: Duration::ZERO,
            min_tick_time: Duration::ZERO,
            node_exec_counts: HashMap::new(),
            node_success_counts: HashMap::new(),
            node_failure_counts: HashMap::new(),
        }
    }
}

impl BehaviorTreeMetrics {
    /// Create new empty metrics.
    pub fn new() -> Self {
        Self::default()
    }

    /// Reset all metrics to their default values.
    pub fn reset(&mut self) {
        *self = Self::default();
    }

    /// Record a tick.
    pub fn record_tick(&mut self, duration_ns: u64) {
        self.total_ticks += 1;
        self.max_tick_duration_ns = self.max_tick_duration_ns.max(duration_ns);

        // Update rolling average
        let total = self.avg_tick_duration_ns * (self.total_ticks - 1) + duration_ns;
        self.avg_tick_duration_ns = total / self.total_ticks;
    }

    /// Record tree completion.
    pub fn record_completion(&mut self, success: bool) {
        if success {
            self.success_count += 1;
        } else {
            self.failure_count += 1;
        }
    }

    /// Record node execution.
    pub fn record_node_exec(&mut self, node_id: &NodeId, status: NodeStatus) {
        *self
            .node_exec_counts
            .entry(node_id.as_str().to_string())
            .or_insert(0) += 1;

        match status {
            NodeStatus::Success => {
                *self
                    .node_success_counts
                    .entry(node_id.as_str().to_string())
                    .or_insert(0) += 1;
            }
            NodeStatus::Failure => {
                *self
                    .node_failure_counts
                    .entry(node_id.as_str().to_string())
                    .or_insert(0) += 1;
            }
            _ => {}
        }
    }
}

/// Serde helper for Duration fields.
mod duration_serde {
    use serde::{Deserialize, Deserializer, Serialize, Serializer};
    use std::time::Duration;

    pub fn serialize<S>(duration: &Duration, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: Serializer,
    {
        duration.as_nanos().serialize(serializer)
    }

    pub fn deserialize<'de, D>(deserializer: D) -> Result<Duration, D::Error>
    where
        D: Deserializer<'de>,
    {
        let nanos: u128 = u128::deserialize(deserializer)?;
        Ok(Duration::from_nanos(nanos as u64))
    }
}

/// Error type for behavior tree operations.
#[derive(Clone, Debug, thiserror::Error)]
pub enum BehaviorTreeError {
    #[error("Node not found: {0}")]
    NodeNotFound(NodeId),

    #[error("Invalid tree structure: {0}")]
    InvalidStructure(String),

    #[error("Invalid tree: {reason}")]
    InvalidTree { reason: String },

    #[error("Tree is already running")]
    AlreadyRunning,

    #[error("Maximum depth exceeded: {depth} > {max}")]
    MaxDepthExceeded { depth: usize, max: usize },

    #[error("Tree not started")]
    NotStarted,

    #[error("Blackboard key not found: {0}")]
    BlackboardKeyNotFound(String),

    #[error("Type mismatch for blackboard key '{key}': expected {expected}")]
    TypeMismatch { key: String, expected: String },

    #[error("Node execution error: {0}")]
    ExecutionError(String),

    #[error("Internal error: {0}")]
    Internal(String),
}

/// Type of a behavior tree node.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum NodeType {
    /// Leaf node that performs an action.
    Action,
    /// Leaf node that checks a condition.
    Condition,
    /// Composite that runs children in sequence.
    Sequence,
    /// Composite that runs children until one succeeds.
    Selector,
    /// Composite that runs children in parallel.
    Parallel,
    /// Decorator that modifies child behavior.
    Decorator,
    /// A subtree reference (nested behavior tree).
    Subtree,
    /// Custom node type.
    Custom(u32),
}

impl fmt::Display for NodeType {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            NodeType::Action => write!(f, "Action"),
            NodeType::Condition => write!(f, "Condition"),
            NodeType::Sequence => write!(f, "Sequence"),
            NodeType::Selector => write!(f, "Selector"),
            NodeType::Parallel => write!(f, "Parallel"),
            NodeType::Decorator => write!(f, "Decorator"),
            NodeType::Subtree => write!(f, "Subtree"),
            NodeType::Custom(id) => write!(f, "Custom({})", id),
        }
    }
}

/// Policy for parallel node completion.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize, Default)]
pub enum ParallelPolicy {
    /// Succeed if all children succeed.
    #[default]
    RequireAll,
    /// Succeed if any child succeeds.
    RequireOne,
    /// Succeed if at least N children succeed.
    RequireN(usize),
}

/// Decorator type for modifying child behavior.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub enum DecoratorType {
    /// Invert the child's result.
    Inverter,
    /// Always return success.
    Succeeder,
    /// Always return failure.
    Failer,
    /// Repeat N times (alias: Repeat).
    Repeater(usize),
    /// Repeat until failure.
    RepeatUntilFail,
    /// Repeat until success.
    RepeatUntilSuccess,
    /// Retry on failure up to N times.
    Retry(u32),
    /// Timeout after duration.
    Timeout(Duration),
    /// Run once then skip.
    RunOnce,
    /// Delay before running child.
    Delay(Duration),
    /// Cooldown between executions.
    Cooldown(Duration),
    /// Force success after N failures.
    ForceSuccessAfter(u32),
}

impl fmt::Display for DecoratorType {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            DecoratorType::Inverter => write!(f, "Inverter"),
            DecoratorType::Succeeder => write!(f, "Succeeder"),
            DecoratorType::Failer => write!(f, "Failer"),
            DecoratorType::Repeater(n) => write!(f, "Repeater({})", n),
            DecoratorType::RepeatUntilFail => write!(f, "RepeatUntilFail"),
            DecoratorType::RepeatUntilSuccess => write!(f, "RepeatUntilSuccess"),
            DecoratorType::Retry(n) => write!(f, "Retry({})", n),
            DecoratorType::Timeout(d) => write!(f, "Timeout({:?})", d),
            DecoratorType::RunOnce => write!(f, "RunOnce"),
            DecoratorType::Delay(d) => write!(f, "Delay({:?})", d),
            DecoratorType::Cooldown(d) => write!(f, "Cooldown({:?})", d),
            DecoratorType::ForceSuccessAfter(n) => write!(f, "ForceSuccessAfter({})", n),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_node_status() {
        assert!(NodeStatus::Running.is_running());
        assert!(NodeStatus::Success.is_success());
        assert!(NodeStatus::Failure.is_failure());

        assert!(!NodeStatus::Running.is_terminal());
        assert!(NodeStatus::Success.is_terminal());
        assert!(NodeStatus::Failure.is_terminal());

        assert_eq!(NodeStatus::Success.invert(), NodeStatus::Failure);
        assert_eq!(NodeStatus::Failure.invert(), NodeStatus::Success);
        assert_eq!(NodeStatus::Running.invert(), NodeStatus::Running);
    }

    #[test]
    fn test_node_id() {
        let id1 = NodeId::new("test");
        assert_eq!(id1.as_str(), "test");

        let id2 = NodeId::generate();
        assert!(!id2.as_str().is_empty());

        let id3: NodeId = "from_str".into();
        assert_eq!(id3.as_str(), "from_str");
    }

    #[test]
    fn test_blackboard() {
        let mut bb = Blackboard::new();

        bb.set("count", 42i64);
        bb.set("flag", true);
        bb.set("name", "robot");
        bb.set("position", [1.0, 2.0, 3.0]);

        assert_eq!(bb.get_int("count"), Some(42));
        assert_eq!(bb.get_bool("flag"), Some(true));
        assert_eq!(bb.get_string("name"), Some("robot"));
        assert_eq!(bb.get_vec3("position"), Some([1.0, 2.0, 3.0]));

        assert!(bb.contains("count"));
        assert!(!bb.contains("missing"));

        bb.remove("count");
        assert!(!bb.contains("count"));
    }

    #[test]
    fn test_blackboard_value_conversions() {
        let v: BlackboardValue = true.into();
        assert_eq!(v.as_bool(), Some(true));

        let v: BlackboardValue = 42i32.into();
        assert_eq!(v.as_int(), Some(42));

        let v: BlackboardValue = 3.5f32.into();
        assert!(v.as_float().is_some());

        let v: BlackboardValue = "hello".into();
        assert_eq!(v.as_string(), Some("hello"));
    }

    #[test]
    fn test_config() {
        let config = BehaviorTreeConfig::new("test_tree")
            .with_max_ticks(30)
            .with_debug(true);

        assert_eq!(config.name, "test_tree");
        assert_eq!(config.max_ticks_per_second, 30);
        assert!(config.debug_logging);
    }

    #[test]
    fn test_metrics() {
        let mut metrics = BehaviorTreeMetrics::new();

        metrics.record_tick(1000);
        metrics.record_tick(2000);
        assert_eq!(metrics.total_ticks, 2);
        assert_eq!(metrics.avg_tick_duration_ns, 1500);
        assert_eq!(metrics.max_tick_duration_ns, 2000);

        metrics.record_completion(true);
        metrics.record_completion(false);
        assert_eq!(metrics.success_count, 1);
        assert_eq!(metrics.failure_count, 1);
    }

    #[test]
    fn test_parallel_policy() {
        let policy = ParallelPolicy::default();
        assert_eq!(policy, ParallelPolicy::RequireAll);
    }

    #[test]
    fn test_decorator_types() {
        let d = DecoratorType::Repeater(5);
        assert_eq!(format!("{}", d), "Repeater(5)");

        let d = DecoratorType::Timeout(Duration::from_secs(10));
        assert!(format!("{}", d).contains("Timeout"));
    }
}
