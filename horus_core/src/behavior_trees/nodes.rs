//! Behavior Tree Node Implementations
//!
//! This module provides the core node types for behavior trees:
//!
//! - **Leaf Nodes**: Action and Condition nodes that perform actual work
//! - **Composite Nodes**: Sequence, Selector, and Parallel for control flow
//! - **Decorator Nodes**: Inverter, Repeater, Timeout, and other modifiers

use std::sync::Arc;
use std::time::{Duration, Instant};

use super::types::{DecoratorType, NodeId, NodeStatus, NodeType, ParallelPolicy, TickContext};

#[cfg(test)]
use super::types::Blackboard;

/// Function type for action node execution.
pub type ActionFn<C> = Arc<dyn Fn(&mut TickContext<'_, C>) -> NodeStatus + Send + Sync>;

/// Function type for condition node evaluation.
pub type ConditionFn<C> = Arc<dyn Fn(&TickContext<'_, C>) -> bool + Send + Sync>;

/// Function type for guard conditions on nodes.
pub type GuardFn<C> = Arc<dyn Fn(&TickContext<'_, C>) -> bool + Send + Sync>;

/// The core trait that all behavior tree nodes must implement.
pub trait BTNode<C>: Send + Sync {
    /// Get the unique identifier for this node.
    fn id(&self) -> &NodeId;

    /// Get the name of this node.
    fn name(&self) -> &str;

    /// Get the type of this node.
    fn node_type(&self) -> NodeType;

    /// Execute one tick of this node.
    fn tick(&mut self, ctx: &mut TickContext<'_, C>) -> NodeStatus;

    /// Reset this node to its initial state.
    fn reset(&mut self);

    /// Get the current status of this node (from last tick).
    fn status(&self) -> NodeStatus;

    /// Check if this node has a guard condition.
    fn has_guard(&self) -> bool {
        false
    }

    /// Evaluate the guard condition (if any).
    fn check_guard(&self, _ctx: &TickContext<'_, C>) -> bool {
        true
    }

    /// Get child nodes (for composite nodes).
    fn children(&self) -> &[Box<dyn BTNode<C>>] {
        &[]
    }

    /// Get mutable reference to child nodes.
    fn children_mut(&mut self) -> &mut [Box<dyn BTNode<C>>] {
        &mut []
    }

    /// Called when the node is entered (before first tick).
    fn on_enter(&mut self, _ctx: &mut TickContext<'_, C>) {}

    /// Called when the node exits (after returning Success or Failure).
    fn on_exit(&mut self, _ctx: &mut TickContext<'_, C>, _status: NodeStatus) {}
}

// ============================================================================
// Leaf Nodes
// ============================================================================

/// An action node that executes a function and returns a status.
///
/// Action nodes are the "leaves" of the behavior tree that perform
/// actual work like moving the robot, sending commands, etc.
///
/// # Example
///
/// ```rust,ignore
/// let move_action = ActionNode::new("move_forward", |ctx| {
///     if ctx.context.robot.move_forward(0.5) {
///         NodeStatus::Success
///     } else {
///         NodeStatus::Running
///     }
/// });
/// ```
pub struct ActionNode<C> {
    id: NodeId,
    name: String,
    action: ActionFn<C>,
    guard: Option<GuardFn<C>>,
    status: NodeStatus,
    is_running: bool,
}

impl<C> ActionNode<C> {
    /// Create a new action node with the given name and action function.
    pub fn new<S: Into<String>>(
        name: S,
        action: impl Fn(&mut TickContext<'_, C>) -> NodeStatus + Send + Sync + 'static,
    ) -> Self {
        let name = name.into();
        Self {
            id: NodeId::new(&name),
            name,
            action: Arc::new(action),
            guard: None,
            status: NodeStatus::Failure,
            is_running: false,
        }
    }

    /// Add a guard condition to this action.
    pub fn with_guard(
        mut self,
        guard: impl Fn(&TickContext<'_, C>) -> bool + Send + Sync + 'static,
    ) -> Self {
        self.guard = Some(Arc::new(guard));
        self
    }

    /// Create an action that always succeeds.
    pub fn success<S: Into<String>>(name: S) -> Self {
        Self::new(name, |_| NodeStatus::Success)
    }

    /// Create an action that always fails.
    pub fn failure<S: Into<String>>(name: S) -> Self {
        Self::new(name, |_| NodeStatus::Failure)
    }

    /// Create an action that runs for a specified duration then succeeds.
    pub fn wait<S: Into<String>>(name: S, duration: Duration) -> Self {
        use std::sync::Mutex;
        let start_time: Mutex<Option<Instant>> = Mutex::new(None);
        Self::new(name, move |ctx| {
            let mut guard = start_time.lock().unwrap();
            let start = guard.unwrap_or_else(|| {
                let now = Instant::now();
                *guard = Some(now);
                now
            });

            if ctx.elapsed >= duration || start.elapsed() >= duration {
                *guard = None;
                NodeStatus::Success
            } else {
                NodeStatus::Running
            }
        })
    }
}

impl<C: Send + Sync> BTNode<C> for ActionNode<C> {
    fn id(&self) -> &NodeId {
        &self.id
    }

    fn name(&self) -> &str {
        &self.name
    }

    fn node_type(&self) -> NodeType {
        NodeType::Action
    }

    fn tick(&mut self, ctx: &mut TickContext<'_, C>) -> NodeStatus {
        // Check guard condition
        if let Some(guard) = &self.guard {
            if !guard(ctx) {
                self.status = NodeStatus::Failure;
                // If we were running, call on_exit for proper cleanup
                // (e.g., stop motor, release resources) and reset is_running
                // so on_enter is called again when the guard passes next time.
                if self.is_running {
                    self.on_exit(ctx, self.status);
                    self.is_running = false;
                }
                return self.status;
            }
        }

        // Call on_enter if just starting
        if !self.is_running {
            self.on_enter(ctx);
            self.is_running = true;
        }

        // Execute the action
        self.status = (self.action)(ctx);

        // Call on_exit if done
        if self.status != NodeStatus::Running {
            self.on_exit(ctx, self.status);
            self.is_running = false;
        }

        self.status
    }

    fn reset(&mut self) {
        self.status = NodeStatus::Failure;
        self.is_running = false;
    }

    fn status(&self) -> NodeStatus {
        self.status
    }

    fn has_guard(&self) -> bool {
        self.guard.is_some()
    }

    fn check_guard(&self, ctx: &TickContext<'_, C>) -> bool {
        self.guard.as_ref().is_none_or(|g| g(ctx))
    }
}

/// A condition node that evaluates a predicate.
///
/// Condition nodes check if a condition is true or false and return
/// Success or Failure immediately (never Running).
///
/// # Example
///
/// ```rust,ignore
/// let battery_ok = ConditionNode::new("battery_ok", |ctx| {
///     ctx.context.robot.battery_level() > 0.2
/// });
/// ```
pub struct ConditionNode<C> {
    id: NodeId,
    name: String,
    condition: ConditionFn<C>,
    status: NodeStatus,
}

impl<C> ConditionNode<C> {
    /// Create a new condition node with the given name and condition function.
    pub fn new<S: Into<String>>(
        name: S,
        condition: impl Fn(&TickContext<'_, C>) -> bool + Send + Sync + 'static,
    ) -> Self {
        let name = name.into();
        Self {
            id: NodeId::new(&name),
            name,
            condition: Arc::new(condition),
            status: NodeStatus::Failure,
        }
    }

    /// Create a condition that checks if a blackboard key exists.
    pub fn has_key<S: Into<String>>(name: S, key: String) -> Self {
        Self::new(name, move |ctx| ctx.blackboard.contains(&key))
    }
}

impl<C: Send + Sync> BTNode<C> for ConditionNode<C> {
    fn id(&self) -> &NodeId {
        &self.id
    }

    fn name(&self) -> &str {
        &self.name
    }

    fn node_type(&self) -> NodeType {
        NodeType::Condition
    }

    fn tick(&mut self, ctx: &mut TickContext<'_, C>) -> NodeStatus {
        self.status = if (self.condition)(ctx) {
            NodeStatus::Success
        } else {
            NodeStatus::Failure
        };
        self.status
    }

    fn reset(&mut self) {
        self.status = NodeStatus::Failure;
    }

    fn status(&self) -> NodeStatus {
        self.status
    }
}

// ============================================================================
// Composite Nodes
// ============================================================================

/// A sequence node that runs children in order until one fails.
///
/// - Ticks each child in order
/// - Returns Failure immediately if any child fails
/// - Returns Running if a child is still running
/// - Returns Success only if all children succeed
///
/// # Example
///
/// ```rust,ignore
/// let sequence = SequenceNode::new("patrol")
///     .add_child(move_to_waypoint_1)
///     .add_child(scan_area)
///     .add_child(move_to_waypoint_2);
/// ```
pub struct SequenceNode<C> {
    id: NodeId,
    name: String,
    children: Vec<Box<dyn BTNode<C>>>,
    current_child: usize,
    status: NodeStatus,
}

impl<C> SequenceNode<C> {
    /// Create a new sequence node with the given name.
    pub fn new<S: Into<String>>(name: S) -> Self {
        let name = name.into();
        Self {
            id: NodeId::new(&name),
            name,
            children: Vec::new(),
            current_child: 0,
            status: NodeStatus::Failure,
        }
    }

    /// Add a child node to this sequence.
    pub fn add_child<N: BTNode<C> + 'static>(mut self, child: N) -> Self {
        self.children.push(Box::new(child));
        self
    }

    /// Add multiple children to this sequence.
    pub fn with_children(mut self, children: Vec<Box<dyn BTNode<C>>>) -> Self {
        self.children = children;
        self
    }
}

impl<C: Send + Sync> BTNode<C> for SequenceNode<C> {
    fn id(&self) -> &NodeId {
        &self.id
    }

    fn name(&self) -> &str {
        &self.name
    }

    fn node_type(&self) -> NodeType {
        NodeType::Sequence
    }

    fn tick(&mut self, ctx: &mut TickContext<'_, C>) -> NodeStatus {
        while self.current_child < self.children.len() {
            let child = &mut self.children[self.current_child];
            let child_status = child.tick(ctx);

            match child_status {
                NodeStatus::Running => {
                    self.status = NodeStatus::Running;
                    return self.status;
                }
                NodeStatus::Failure => {
                    self.status = NodeStatus::Failure;
                    self.current_child = 0; // Reset for next run
                    return self.status;
                }
                NodeStatus::Success => {
                    self.current_child += 1;
                }
            }
        }

        // All children succeeded
        self.status = NodeStatus::Success;
        self.current_child = 0; // Reset for next run
        self.status
    }

    fn reset(&mut self) {
        self.current_child = 0;
        self.status = NodeStatus::Failure;
        for child in &mut self.children {
            child.reset();
        }
    }

    fn status(&self) -> NodeStatus {
        self.status
    }

    fn children(&self) -> &[Box<dyn BTNode<C>>] {
        &self.children
    }

    fn children_mut(&mut self) -> &mut [Box<dyn BTNode<C>>] {
        &mut self.children
    }
}

/// A selector (fallback) node that runs children until one succeeds.
///
/// - Ticks each child in order
/// - Returns Success immediately if any child succeeds
/// - Returns Running if a child is still running
/// - Returns Failure only if all children fail
///
/// # Example
///
/// ```rust,ignore
/// let selector = SelectorNode::new("find_target")
///     .add_child(check_cached_target)
///     .add_child(scan_for_target)
///     .add_child(request_target_from_operator);
/// ```
pub struct SelectorNode<C> {
    id: NodeId,
    name: String,
    children: Vec<Box<dyn BTNode<C>>>,
    current_child: usize,
    status: NodeStatus,
}

impl<C> SelectorNode<C> {
    /// Create a new selector node with the given name.
    pub fn new<S: Into<String>>(name: S) -> Self {
        let name = name.into();
        Self {
            id: NodeId::new(&name),
            name,
            children: Vec::new(),
            current_child: 0,
            status: NodeStatus::Failure,
        }
    }

    /// Add a child node to this selector.
    pub fn add_child<N: BTNode<C> + 'static>(mut self, child: N) -> Self {
        self.children.push(Box::new(child));
        self
    }

    /// Add multiple children to this selector.
    pub fn with_children(mut self, children: Vec<Box<dyn BTNode<C>>>) -> Self {
        self.children = children;
        self
    }
}

impl<C: Send + Sync> BTNode<C> for SelectorNode<C> {
    fn id(&self) -> &NodeId {
        &self.id
    }

    fn name(&self) -> &str {
        &self.name
    }

    fn node_type(&self) -> NodeType {
        NodeType::Selector
    }

    fn tick(&mut self, ctx: &mut TickContext<'_, C>) -> NodeStatus {
        while self.current_child < self.children.len() {
            let child = &mut self.children[self.current_child];
            let child_status = child.tick(ctx);

            match child_status {
                NodeStatus::Running => {
                    self.status = NodeStatus::Running;
                    return self.status;
                }
                NodeStatus::Success => {
                    self.status = NodeStatus::Success;
                    self.current_child = 0; // Reset for next run
                    return self.status;
                }
                NodeStatus::Failure => {
                    self.current_child += 1;
                }
            }
        }

        // All children failed
        self.status = NodeStatus::Failure;
        self.current_child = 0; // Reset for next run
        self.status
    }

    fn reset(&mut self) {
        self.current_child = 0;
        self.status = NodeStatus::Failure;
        for child in &mut self.children {
            child.reset();
        }
    }

    fn status(&self) -> NodeStatus {
        self.status
    }

    fn children(&self) -> &[Box<dyn BTNode<C>>] {
        &self.children
    }

    fn children_mut(&mut self) -> &mut [Box<dyn BTNode<C>>] {
        &mut self.children
    }
}

/// A parallel node that runs all children simultaneously.
///
/// The parallel node ticks all children on each tick and uses a policy
/// to determine when to return Success or Failure.
///
/// # Policies
///
/// - `RequireAll`: All children must succeed for Success
/// - `RequireOne`: Any child succeeding means Success
/// - `RequireN(n)`: At least N children must succeed
///
/// # Example
///
/// ```rust,ignore
/// let parallel = ParallelNode::new("monitor_and_move", ParallelPolicy::RequireAll)
///     .add_child(monitor_obstacles)
///     .add_child(execute_motion);
/// ```
pub struct ParallelNode<C> {
    id: NodeId,
    name: String,
    children: Vec<Box<dyn BTNode<C>>>,
    policy: ParallelPolicy,
    status: NodeStatus,
}

impl<C> ParallelNode<C> {
    /// Create a new parallel node with the given name and policy.
    pub fn new<S: Into<String>>(name: S, policy: ParallelPolicy) -> Self {
        let name = name.into();
        Self {
            id: NodeId::new(&name),
            name,
            children: Vec::new(),
            policy,
            status: NodeStatus::Failure,
        }
    }

    /// Add a child node to this parallel node.
    pub fn add_child<N: BTNode<C> + 'static>(mut self, child: N) -> Self {
        self.children.push(Box::new(child));
        self
    }

    /// Add multiple children to this parallel node.
    pub fn with_children(mut self, children: Vec<Box<dyn BTNode<C>>>) -> Self {
        self.children = children;
        self
    }
}

impl<C: Send + Sync> BTNode<C> for ParallelNode<C> {
    fn id(&self) -> &NodeId {
        &self.id
    }

    fn name(&self) -> &str {
        &self.name
    }

    fn node_type(&self) -> NodeType {
        NodeType::Parallel
    }

    fn tick(&mut self, ctx: &mut TickContext<'_, C>) -> NodeStatus {
        let mut success_count = 0;
        let mut failure_count = 0;

        // Tick all children
        for child in &mut self.children {
            let child_status = child.tick(ctx);
            match child_status {
                NodeStatus::Success => success_count += 1,
                NodeStatus::Failure => failure_count += 1,
                NodeStatus::Running => {}
            }
        }

        let total = self.children.len();

        // Apply policy
        self.status = match self.policy {
            ParallelPolicy::RequireAll => {
                if failure_count > 0 {
                    NodeStatus::Failure
                } else if success_count == total {
                    NodeStatus::Success
                } else {
                    NodeStatus::Running
                }
            }
            ParallelPolicy::RequireOne => {
                if success_count > 0 {
                    NodeStatus::Success
                } else if failure_count == total {
                    NodeStatus::Failure
                } else {
                    NodeStatus::Running
                }
            }
            ParallelPolicy::RequireN(n) => {
                let n = n.min(total);
                if success_count >= n {
                    NodeStatus::Success
                } else if total - failure_count < n {
                    // Not enough remaining children can succeed
                    NodeStatus::Failure
                } else {
                    NodeStatus::Running
                }
            }
        };

        // Reset children if we're done
        if self.status != NodeStatus::Running {
            for child in &mut self.children {
                child.reset();
            }
        }

        self.status
    }

    fn reset(&mut self) {
        self.status = NodeStatus::Failure;
        for child in &mut self.children {
            child.reset();
        }
    }

    fn status(&self) -> NodeStatus {
        self.status
    }

    fn children(&self) -> &[Box<dyn BTNode<C>>] {
        &self.children
    }

    fn children_mut(&mut self) -> &mut [Box<dyn BTNode<C>>] {
        &mut self.children
    }
}

// ============================================================================
// Decorator Nodes
// ============================================================================

/// A decorator node that wraps a single child and modifies its behavior.
///
/// Decorator types include:
/// - `Inverter`: Inverts Success/Failure
/// - `Succeeder`: Always returns Success
/// - `Failer`: Always returns Failure
/// - `Repeater(n)`: Repeats child n times
/// - `RepeatUntilFail`: Repeats until child fails
/// - `RepeatUntilSuccess`: Repeats until child succeeds
/// - `Timeout(duration)`: Fails if child takes too long
/// - `Delay(duration)`: Waits before running child
/// - `Cooldown(duration)`: Prevents re-running too quickly
///
/// # Example
///
/// ```rust,ignore
/// // Invert a condition
/// let not_obstacle = DecoratorNode::inverter("not_obstacle", check_obstacle);
///
/// // Retry an action up to 3 times
/// let retry_connect = DecoratorNode::repeater("retry_connect", 3, connect_to_server);
///
/// // Add timeout to an action
/// let timed_scan = DecoratorNode::timeout("timed_scan", Duration::from_secs(5), scan_area);
/// ```
pub struct DecoratorNode<C> {
    id: NodeId,
    name: String,
    child: Box<dyn BTNode<C>>,
    decorator_type: DecoratorType,
    status: NodeStatus,
    // State for various decorators
    repeat_count: usize,
    start_time: Option<Instant>,
    last_run_time: Option<Instant>,
    has_run: bool,
}

impl<C: Send + Sync + 'static> DecoratorNode<C> {
    /// Create a new decorator node with the given child and type.
    pub fn new<S: Into<String>>(
        name: S,
        decorator_type: DecoratorType,
        child: impl BTNode<C> + 'static,
    ) -> Self {
        let name = name.into();
        Self {
            id: NodeId::new(&name),
            name,
            child: Box::new(child),
            decorator_type,
            status: NodeStatus::Failure,
            repeat_count: 0,
            start_time: None,
            last_run_time: None,
            has_run: false,
        }
    }

    /// Create an inverter decorator.
    pub fn inverter<S: Into<String>>(name: S, child: impl BTNode<C> + 'static) -> Self {
        Self::new(name, DecoratorType::Inverter, child)
    }

    /// Create a succeeder decorator.
    pub fn succeeder<S: Into<String>>(name: S, child: impl BTNode<C> + 'static) -> Self {
        Self::new(name, DecoratorType::Succeeder, child)
    }

    /// Create a failer decorator.
    pub fn failer<S: Into<String>>(name: S, child: impl BTNode<C> + 'static) -> Self {
        Self::new(name, DecoratorType::Failer, child)
    }

    /// Create a repeater decorator.
    pub fn repeater<S: Into<String>>(
        name: S,
        count: usize,
        child: impl BTNode<C> + 'static,
    ) -> Self {
        Self::new(name, DecoratorType::Repeater(count), child)
    }

    /// Create a repeat-until-fail decorator.
    pub fn repeat_until_fail<S: Into<String>>(name: S, child: impl BTNode<C> + 'static) -> Self {
        Self::new(name, DecoratorType::RepeatUntilFail, child)
    }

    /// Create a repeat-until-success decorator.
    pub fn repeat_until_success<S: Into<String>>(name: S, child: impl BTNode<C> + 'static) -> Self {
        Self::new(name, DecoratorType::RepeatUntilSuccess, child)
    }

    /// Create a timeout decorator.
    pub fn timeout<S: Into<String>>(
        name: S,
        duration: Duration,
        child: impl BTNode<C> + 'static,
    ) -> Self {
        Self::new(name, DecoratorType::Timeout(duration), child)
    }

    /// Create a delay decorator.
    pub fn delay<S: Into<String>>(
        name: S,
        duration: Duration,
        child: impl BTNode<C> + 'static,
    ) -> Self {
        Self::new(name, DecoratorType::Delay(duration), child)
    }

    /// Create a cooldown decorator.
    pub fn cooldown<S: Into<String>>(
        name: S,
        duration: Duration,
        child: impl BTNode<C> + 'static,
    ) -> Self {
        Self::new(name, DecoratorType::Cooldown(duration), child)
    }
}

impl<C: Send + Sync> BTNode<C> for DecoratorNode<C> {
    fn id(&self) -> &NodeId {
        &self.id
    }

    fn name(&self) -> &str {
        &self.name
    }

    fn node_type(&self) -> NodeType {
        NodeType::Decorator
    }

    fn tick(&mut self, ctx: &mut TickContext<'_, C>) -> NodeStatus {
        self.status = match &self.decorator_type {
            DecoratorType::Inverter => match self.child.tick(ctx) {
                NodeStatus::Success => NodeStatus::Failure,
                NodeStatus::Failure => NodeStatus::Success,
                NodeStatus::Running => NodeStatus::Running,
            },
            DecoratorType::Succeeder => {
                let _ = self.child.tick(ctx);
                NodeStatus::Success
            }
            DecoratorType::Failer => {
                let _ = self.child.tick(ctx);
                NodeStatus::Failure
            }
            DecoratorType::Repeater(count) => {
                loop {
                    let child_status = self.child.tick(ctx);
                    match child_status {
                        NodeStatus::Running => break NodeStatus::Running,
                        NodeStatus::Success | NodeStatus::Failure => {
                            self.repeat_count += 1;
                            self.child.reset();
                            if self.repeat_count >= *count {
                                self.repeat_count = 0;
                                break NodeStatus::Success;
                            }
                            // Continue loop for next iteration
                        }
                    }
                }
            }
            DecoratorType::RepeatUntilFail => {
                loop {
                    let child_status = self.child.tick(ctx);
                    match child_status {
                        NodeStatus::Running => break NodeStatus::Running,
                        NodeStatus::Success => {
                            self.child.reset();
                            // Continue loop
                        }
                        NodeStatus::Failure => {
                            break NodeStatus::Success; // We wanted it to fail
                        }
                    }
                }
            }
            DecoratorType::RepeatUntilSuccess => {
                loop {
                    let child_status = self.child.tick(ctx);
                    match child_status {
                        NodeStatus::Running => break NodeStatus::Running,
                        NodeStatus::Failure => {
                            self.child.reset();
                            // Continue loop
                        }
                        NodeStatus::Success => {
                            break NodeStatus::Success;
                        }
                    }
                }
            }
            DecoratorType::Timeout(duration) => {
                // Initialize start time if not set
                if self.start_time.is_none() {
                    self.start_time = Some(Instant::now());
                }

                let start = self.start_time.unwrap();
                if start.elapsed() > *duration {
                    self.start_time = None;
                    self.child.reset();
                    NodeStatus::Failure
                } else {
                    let child_status = self.child.tick(ctx);
                    if child_status != NodeStatus::Running {
                        self.start_time = None;
                    }
                    child_status
                }
            }
            DecoratorType::Delay(duration) => {
                // Initialize start time if not set
                if self.start_time.is_none() {
                    self.start_time = Some(Instant::now());
                }

                let start = self.start_time.unwrap();
                if start.elapsed() < *duration {
                    NodeStatus::Running
                } else {
                    let child_status = self.child.tick(ctx);
                    if child_status != NodeStatus::Running {
                        self.start_time = None;
                    }
                    child_status
                }
            }
            DecoratorType::Cooldown(duration) => {
                // Check if we're still in cooldown
                if let Some(last_run) = self.last_run_time {
                    if last_run.elapsed() < *duration {
                        return NodeStatus::Failure;
                    }
                }

                let child_status = self.child.tick(ctx);
                if child_status != NodeStatus::Running {
                    self.last_run_time = Some(Instant::now());
                }
                child_status
            }
            DecoratorType::Retry(max_retries) => {
                let child_status = self.child.tick(ctx);
                match child_status {
                    NodeStatus::Running => NodeStatus::Running,
                    NodeStatus::Success => {
                        self.repeat_count = 0;
                        NodeStatus::Success
                    }
                    NodeStatus::Failure => {
                        self.repeat_count += 1;
                        if self.repeat_count >= *max_retries as usize {
                            self.repeat_count = 0;
                            NodeStatus::Failure
                        } else {
                            self.child.reset();
                            NodeStatus::Running // Will retry on next tick
                        }
                    }
                }
            }
            DecoratorType::RunOnce => {
                // If already run, return the cached status
                if self.has_run {
                    return self.status;
                }

                let child_status = self.child.tick(ctx);
                if child_status != NodeStatus::Running {
                    self.has_run = true;
                }
                child_status
            }
            DecoratorType::ForceSuccessAfter(max_failures) => {
                let child_status = self.child.tick(ctx);
                match child_status {
                    NodeStatus::Running => NodeStatus::Running,
                    NodeStatus::Success => {
                        self.repeat_count = 0;
                        NodeStatus::Success
                    }
                    NodeStatus::Failure => {
                        self.repeat_count += 1;
                        if self.repeat_count >= *max_failures as usize {
                            self.repeat_count = 0;
                            NodeStatus::Success // Force success after too many failures
                        } else {
                            NodeStatus::Failure
                        }
                    }
                }
            }
        };

        self.status
    }

    fn reset(&mut self) {
        self.status = NodeStatus::Failure;
        self.repeat_count = 0;
        self.start_time = None;
        self.has_run = false;
        // Note: Don't reset last_run_time for cooldown
        self.child.reset();
    }

    fn status(&self) -> NodeStatus {
        self.status
    }
}

// ============================================================================
// Utility Nodes
// ============================================================================

/// A subtree node that references another behavior tree.
///
/// This allows for modular tree composition and reuse.
pub struct SubtreeNode<C> {
    id: NodeId,
    name: String,
    root: Box<dyn BTNode<C>>,
    status: NodeStatus,
}

impl<C> SubtreeNode<C> {
    /// Create a new subtree node.
    pub fn new<S: Into<String>>(name: S, root: impl BTNode<C> + 'static) -> Self {
        let name = name.into();
        Self {
            id: NodeId::new(&name),
            name,
            root: Box::new(root),
            status: NodeStatus::Failure,
        }
    }
}

impl<C: Send + Sync> BTNode<C> for SubtreeNode<C> {
    fn id(&self) -> &NodeId {
        &self.id
    }

    fn name(&self) -> &str {
        &self.name
    }

    fn node_type(&self) -> NodeType {
        NodeType::Subtree
    }

    fn tick(&mut self, ctx: &mut TickContext<'_, C>) -> NodeStatus {
        self.status = self.root.tick(ctx);
        self.status
    }

    fn reset(&mut self) {
        self.status = NodeStatus::Failure;
        self.root.reset();
    }

    fn status(&self) -> NodeStatus {
        self.status
    }
}

/// A reactive sequence that re-evaluates earlier conditions.
///
/// Unlike a regular sequence, a reactive sequence will re-check
/// conditions at the start of each tick, allowing for more responsive
/// behavior.
pub struct ReactiveSequenceNode<C> {
    id: NodeId,
    name: String,
    children: Vec<Box<dyn BTNode<C>>>,
    running_child: Option<usize>,
    status: NodeStatus,
}

impl<C> ReactiveSequenceNode<C> {
    /// Create a new reactive sequence node.
    pub fn new<S: Into<String>>(name: S) -> Self {
        let name = name.into();
        Self {
            id: NodeId::new(&name),
            name,
            children: Vec::new(),
            running_child: None,
            status: NodeStatus::Failure,
        }
    }

    /// Add a child node to this reactive sequence.
    pub fn add_child<N: BTNode<C> + 'static>(mut self, child: N) -> Self {
        self.children.push(Box::new(child));
        self
    }
}

impl<C: Send + Sync> BTNode<C> for ReactiveSequenceNode<C> {
    fn id(&self) -> &NodeId {
        &self.id
    }

    fn name(&self) -> &str {
        &self.name
    }

    fn node_type(&self) -> NodeType {
        NodeType::Sequence // Variant of sequence
    }

    fn tick(&mut self, ctx: &mut TickContext<'_, C>) -> NodeStatus {
        // Re-evaluate from the beginning each tick
        for (i, child) in self.children.iter_mut().enumerate() {
            let child_status = child.tick(ctx);

            match child_status {
                NodeStatus::Running => {
                    // If a different child is now running, reset the previous one
                    if let Some(prev) = self.running_child {
                        if prev != i && prev < self.children.len() {
                            self.children[prev].reset();
                        }
                    }
                    self.running_child = Some(i);
                    self.status = NodeStatus::Running;
                    return self.status;
                }
                NodeStatus::Failure => {
                    self.status = NodeStatus::Failure;
                    self.running_child = None;
                    return self.status;
                }
                NodeStatus::Success => {
                    // Continue to next child
                }
            }
        }

        // All children succeeded
        self.status = NodeStatus::Success;
        self.running_child = None;
        self.status
    }

    fn reset(&mut self) {
        self.status = NodeStatus::Failure;
        self.running_child = None;
        for child in &mut self.children {
            child.reset();
        }
    }

    fn status(&self) -> NodeStatus {
        self.status
    }

    fn children(&self) -> &[Box<dyn BTNode<C>>] {
        &self.children
    }

    fn children_mut(&mut self) -> &mut [Box<dyn BTNode<C>>] {
        &mut self.children
    }
}

/// A reactive selector that re-evaluates earlier conditions.
pub struct ReactiveSelectorNode<C> {
    id: NodeId,
    name: String,
    children: Vec<Box<dyn BTNode<C>>>,
    running_child: Option<usize>,
    status: NodeStatus,
}

impl<C> ReactiveSelectorNode<C> {
    /// Create a new reactive selector node.
    pub fn new<S: Into<String>>(name: S) -> Self {
        let name = name.into();
        Self {
            id: NodeId::new(&name),
            name,
            children: Vec::new(),
            running_child: None,
            status: NodeStatus::Failure,
        }
    }

    /// Add a child node to this reactive selector.
    pub fn add_child<N: BTNode<C> + 'static>(mut self, child: N) -> Self {
        self.children.push(Box::new(child));
        self
    }
}

impl<C: Send + Sync> BTNode<C> for ReactiveSelectorNode<C> {
    fn id(&self) -> &NodeId {
        &self.id
    }

    fn name(&self) -> &str {
        &self.name
    }

    fn node_type(&self) -> NodeType {
        NodeType::Selector // Variant of selector
    }

    fn tick(&mut self, ctx: &mut TickContext<'_, C>) -> NodeStatus {
        // Re-evaluate from the beginning each tick
        for (i, child) in self.children.iter_mut().enumerate() {
            let child_status = child.tick(ctx);

            match child_status {
                NodeStatus::Running => {
                    // If a different child is now running, reset the previous one
                    if let Some(prev) = self.running_child {
                        if prev != i && prev < self.children.len() {
                            self.children[prev].reset();
                        }
                    }
                    self.running_child = Some(i);
                    self.status = NodeStatus::Running;
                    return self.status;
                }
                NodeStatus::Success => {
                    self.status = NodeStatus::Success;
                    self.running_child = None;
                    return self.status;
                }
                NodeStatus::Failure => {
                    // Continue to next child
                }
            }
        }

        // All children failed
        self.status = NodeStatus::Failure;
        self.running_child = None;
        self.status
    }

    fn reset(&mut self) {
        self.status = NodeStatus::Failure;
        self.running_child = None;
        for child in &mut self.children {
            child.reset();
        }
    }

    fn status(&self) -> NodeStatus {
        self.status
    }

    fn children(&self) -> &[Box<dyn BTNode<C>>] {
        &self.children
    }

    fn children_mut(&mut self) -> &mut [Box<dyn BTNode<C>>] {
        &mut self.children
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    struct TestContext {
        counter: i32,
        flag: bool,
    }

    #[test]
    fn test_action_node() {
        let mut action = ActionNode::new("increment", |ctx: &mut TickContext<TestContext>| {
            ctx.context.counter += 1;
            NodeStatus::Success
        });

        let mut ctx = TestContext {
            counter: 0,
            flag: false,
        };
        let mut blackboard = Blackboard::new();
        let mut tick_ctx = TickContext {
            blackboard: &mut blackboard,
            context: &mut ctx,
            elapsed: Duration::ZERO,
            delta_time: Duration::from_millis(16),
            tick_count: 0,
        };

        let status = action.tick(&mut tick_ctx);
        assert_eq!(status, NodeStatus::Success);
        assert_eq!(tick_ctx.context.counter, 1);
    }

    #[test]
    fn test_condition_node() {
        let mut condition = ConditionNode::new("check_flag", |ctx: &TickContext<TestContext>| {
            ctx.context.flag
        });

        let mut ctx = TestContext {
            counter: 0,
            flag: false,
        };
        let mut blackboard = Blackboard::new();
        let mut tick_ctx = TickContext {
            blackboard: &mut blackboard,
            context: &mut ctx,
            elapsed: Duration::ZERO,
            delta_time: Duration::from_millis(16),
            tick_count: 0,
        };

        assert_eq!(condition.tick(&mut tick_ctx), NodeStatus::Failure);

        tick_ctx.context.flag = true;
        assert_eq!(condition.tick(&mut tick_ctx), NodeStatus::Success);
    }

    #[test]
    fn test_sequence_node() {
        let sequence = SequenceNode::new("test_sequence")
            .add_child(ActionNode::new("a1", |_: &mut TickContext<TestContext>| {
                NodeStatus::Success
            }))
            .add_child(ActionNode::new("a2", |_: &mut TickContext<TestContext>| {
                NodeStatus::Success
            }))
            .add_child(ActionNode::new("a3", |_: &mut TickContext<TestContext>| {
                NodeStatus::Success
            }));

        let mut sequence = sequence;
        let mut ctx = TestContext {
            counter: 0,
            flag: false,
        };
        let mut blackboard = Blackboard::new();
        let mut tick_ctx = TickContext {
            blackboard: &mut blackboard,
            context: &mut ctx,
            elapsed: Duration::ZERO,
            delta_time: Duration::from_millis(16),
            tick_count: 0,
        };

        assert_eq!(sequence.tick(&mut tick_ctx), NodeStatus::Success);
    }

    #[test]
    fn test_sequence_failure() {
        let sequence = SequenceNode::new("test_sequence")
            .add_child(ActionNode::new("a1", |_: &mut TickContext<TestContext>| {
                NodeStatus::Success
            }))
            .add_child(ActionNode::new("a2", |_: &mut TickContext<TestContext>| {
                NodeStatus::Failure
            }))
            .add_child(ActionNode::new("a3", |_: &mut TickContext<TestContext>| {
                NodeStatus::Success
            }));

        let mut sequence = sequence;
        let mut ctx = TestContext {
            counter: 0,
            flag: false,
        };
        let mut blackboard = Blackboard::new();
        let mut tick_ctx = TickContext {
            blackboard: &mut blackboard,
            context: &mut ctx,
            elapsed: Duration::ZERO,
            delta_time: Duration::from_millis(16),
            tick_count: 0,
        };

        assert_eq!(sequence.tick(&mut tick_ctx), NodeStatus::Failure);
    }

    #[test]
    fn test_selector_node() {
        let selector = SelectorNode::new("test_selector")
            .add_child(ActionNode::new("a1", |_: &mut TickContext<TestContext>| {
                NodeStatus::Failure
            }))
            .add_child(ActionNode::new("a2", |_: &mut TickContext<TestContext>| {
                NodeStatus::Success
            }))
            .add_child(ActionNode::new("a3", |_: &mut TickContext<TestContext>| {
                NodeStatus::Failure
            }));

        let mut selector = selector;
        let mut ctx = TestContext {
            counter: 0,
            flag: false,
        };
        let mut blackboard = Blackboard::new();
        let mut tick_ctx = TickContext {
            blackboard: &mut blackboard,
            context: &mut ctx,
            elapsed: Duration::ZERO,
            delta_time: Duration::from_millis(16),
            tick_count: 0,
        };

        assert_eq!(selector.tick(&mut tick_ctx), NodeStatus::Success);
    }

    #[test]
    fn test_inverter_decorator() {
        let mut inverter = DecoratorNode::inverter(
            "invert",
            ActionNode::new("succeed", |_: &mut TickContext<TestContext>| {
                NodeStatus::Success
            }),
        );

        let mut ctx = TestContext {
            counter: 0,
            flag: false,
        };
        let mut blackboard = Blackboard::new();
        let mut tick_ctx = TickContext {
            blackboard: &mut blackboard,
            context: &mut ctx,
            elapsed: Duration::ZERO,
            delta_time: Duration::from_millis(16),
            tick_count: 0,
        };

        assert_eq!(inverter.tick(&mut tick_ctx), NodeStatus::Failure);
    }

    #[test]
    fn test_parallel_require_all() {
        let parallel = ParallelNode::new("parallel", ParallelPolicy::RequireAll)
            .add_child(ActionNode::new("a1", |_: &mut TickContext<TestContext>| {
                NodeStatus::Success
            }))
            .add_child(ActionNode::new("a2", |_: &mut TickContext<TestContext>| {
                NodeStatus::Success
            }));

        let mut parallel = parallel;
        let mut ctx = TestContext {
            counter: 0,
            flag: false,
        };
        let mut blackboard = Blackboard::new();
        let mut tick_ctx = TickContext {
            blackboard: &mut blackboard,
            context: &mut ctx,
            elapsed: Duration::ZERO,
            delta_time: Duration::from_millis(16),
            tick_count: 0,
        };

        assert_eq!(parallel.tick(&mut tick_ctx), NodeStatus::Success);
    }

    #[test]
    fn test_parallel_require_one() {
        let parallel = ParallelNode::new("parallel", ParallelPolicy::RequireOne)
            .add_child(ActionNode::new("a1", |_: &mut TickContext<TestContext>| {
                NodeStatus::Failure
            }))
            .add_child(ActionNode::new("a2", |_: &mut TickContext<TestContext>| {
                NodeStatus::Success
            }));

        let mut parallel = parallel;
        let mut ctx = TestContext {
            counter: 0,
            flag: false,
        };
        let mut blackboard = Blackboard::new();
        let mut tick_ctx = TickContext {
            blackboard: &mut blackboard,
            context: &mut ctx,
            elapsed: Duration::ZERO,
            delta_time: Duration::from_millis(16),
            tick_count: 0,
        };

        assert_eq!(parallel.tick(&mut tick_ctx), NodeStatus::Success);
    }
}
