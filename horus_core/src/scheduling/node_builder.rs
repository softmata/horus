//! Node Builder - Fluent API for adding nodes to the scheduler
//!
//! The `NodeBuilder` provides a chainable interface for configuring nodes
//! when adding them to the scheduler, replacing the positional parameter approach.
//!
//! # Example
//!
//! ```rust,ignore
//! use horus_core::Scheduler;
//!
//! let mut scheduler = Scheduler::new();
//!
//! // Old API (still works):
//! scheduler.add(Box::new(my_node), 0);
//!
//! // New fluent API:
//! scheduler.node(Box::new(my_node))
//!     .order(0)
//!     .rate_hz(100.0)
//!     .rt()
//!     .done();
//! ```

use crate::core::Node;
use std::time::Duration;

/// Configuration for a node being added to the scheduler.
///
/// Use `NodeConfig::new()` to create, then configure with chainable methods.
/// Pass to `Scheduler::add_configured()` to register the node.
pub struct NodeConfig {
    /// The node to add
    pub(crate) node: Box<dyn Node>,
    /// Execution order (lower = earlier, default: 100)
    pub(crate) order: u32,
    /// Node-specific tick rate in Hz (None = use scheduler global rate)
    pub(crate) rate_hz: Option<f64>,
    /// Whether this is a real-time node
    pub(crate) is_rt: bool,
    /// WCET budget for RT nodes
    pub(crate) wcet_budget: Option<Duration>,
    /// Deadline for RT nodes
    pub(crate) deadline: Option<Duration>,
    /// Request JIT compilation if supported
    pub(crate) request_jit: bool,
    /// Execution tier override
    pub(crate) tier: Option<super::intelligence::NodeTier>,
}

impl NodeConfig {
    /// Create a new node configuration with defaults.
    ///
    /// # Example
    /// ```rust,ignore
    /// let config = NodeConfig::new(Box::new(MyNode::new()))
    ///     .order(0)
    ///     .rate_hz(100.0);
    /// ```
    pub fn new(node: Box<dyn Node>) -> Self {
        Self {
            node,
            order: 100, // Default to medium priority
            rate_hz: None,
            is_rt: false,
            wcet_budget: None,
            deadline: None,
            request_jit: false,
            tier: None,
        }
    }

    /// Set the execution order (lower = earlier in tick sequence).
    ///
    /// # Priority Guidelines
    /// - **0-9**: Critical real-time (motor control, safety)
    /// - **10-49**: High priority (sensors, fast control loops)
    /// - **50-99**: Normal priority (processing, planning)
    /// - **100-199**: Low priority (logging, diagnostics)
    /// - **200+**: Background (telemetry, non-essential)
    ///
    /// # Example
    /// ```rust,ignore
    /// NodeConfig::new(node).order(0)  // Highest priority
    /// ```
    pub fn order(mut self, order: u32) -> Self {
        self.order = order;
        self
    }

    /// Alias for `order()` - set execution priority.
    ///
    /// Note: Lower values = higher priority (executed first).
    pub fn priority(self, priority: u32) -> Self {
        self.order(priority)
    }

    /// Set a node-specific tick rate in Hz.
    ///
    /// If not set, the node uses the scheduler's global tick rate.
    ///
    /// # Example
    /// ```rust,ignore
    /// NodeConfig::new(sensor_node)
    ///     .rate_hz(1000.0)  // 1kHz for sensor polling
    /// ```
    pub fn rate_hz(mut self, rate: f64) -> Self {
        self.rate_hz = Some(rate);
        self
    }

    /// Mark this as a real-time node.
    ///
    /// RT nodes get special treatment:
    /// - Priority scheduling
    /// - Deadline monitoring
    /// - WCET enforcement (if configured)
    ///
    /// # Example
    /// ```rust,ignore
    /// NodeConfig::new(motor_controller)
    ///     .rt()
    ///     .wcet_us(500)  // 500μs max execution time
    /// ```
    pub fn rt(mut self) -> Self {
        self.is_rt = true;
        self
    }

    /// Set the Worst-Case Execution Time budget in microseconds.
    ///
    /// Automatically marks the node as RT.
    ///
    /// # Example
    /// ```rust,ignore
    /// NodeConfig::new(pid_controller)
    ///     .wcet_us(200)  // Must complete within 200μs
    /// ```
    pub fn wcet_us(mut self, us: u64) -> Self {
        self.is_rt = true;
        self.wcet_budget = Some(Duration::from_micros(us));
        self
    }

    /// Set the deadline in microseconds.
    ///
    /// The node's tick must complete before this deadline relative to tick start.
    /// Automatically marks the node as RT.
    ///
    /// # Example
    /// ```rust,ignore
    /// NodeConfig::new(safety_monitor)
    ///     .deadline_us(1000)  // Must finish within 1ms
    /// ```
    pub fn deadline_us(mut self, us: u64) -> Self {
        self.is_rt = true;
        self.deadline = Some(Duration::from_micros(us));
        self
    }

    /// Set the deadline in milliseconds.
    ///
    /// The node's tick must complete before this deadline relative to tick start.
    /// Automatically marks the node as RT.
    ///
    /// # Example
    /// ```rust,ignore
    /// NodeConfig::new(control_loop)
    ///     .deadline_ms(10)  // Must finish within 10ms
    /// ```
    pub fn deadline_ms(mut self, ms: u64) -> Self {
        self.is_rt = true;
        self.deadline = Some(Duration::from_millis(ms));
        self
    }

    /// Set the WCET budget in milliseconds.
    ///
    /// Automatically marks the node as RT.
    ///
    /// # Example
    /// ```rust,ignore
    /// NodeConfig::new(slow_node)
    ///     .wcet_ms(5)  // Must complete within 5ms
    /// ```
    pub fn wcet_ms(mut self, ms: u64) -> Self {
        self.is_rt = true;
        self.wcet_budget = Some(Duration::from_millis(ms));
        self
    }

    /// Request JIT compilation for this node (if it supports JIT).
    ///
    /// The node must implement the JIT traits for this to have effect.
    ///
    /// # Example
    /// ```rust,ignore
    /// NodeConfig::new(arithmetic_node)
    ///     .jit()  // Request JIT compilation
    /// ```
    pub fn jit(mut self) -> Self {
        self.request_jit = true;
        self
    }

    /// Set an explicit execution tier.
    ///
    /// Overrides automatic tier classification.
    ///
    /// # Example
    /// ```rust,ignore
    /// use horus_core::scheduling::intelligence::NodeTier;
    ///
    /// NodeConfig::new(fast_node)
    ///     .tier(NodeTier::Jit)
    /// ```
    pub fn tier(mut self, tier: super::intelligence::NodeTier) -> Self {
        self.tier = Some(tier);
        self
    }

}

/// Builder for adding a node to the scheduler with fluent configuration.
///
/// Created by `Scheduler::node()`. Call `.done()` to register the node.
///
/// # Example
/// ```rust,ignore
/// scheduler.node(Box::new(my_node))
///     .order(0)
///     .rate_hz(100.0)
///     .rt()
///     .done();
/// ```
pub struct NodeBuilder<'a> {
    scheduler: &'a mut super::scheduler::Scheduler,
    config: NodeConfig,
}

impl<'a> NodeBuilder<'a> {
    /// Create a new NodeBuilder (called by Scheduler::node).
    pub(crate) fn new(scheduler: &'a mut super::scheduler::Scheduler, node: Box<dyn Node>) -> Self {
        Self {
            scheduler,
            config: NodeConfig::new(node),
        }
    }

    /// Set the execution order (lower = earlier in tick sequence).
    ///
    /// # Priority Guidelines
    /// - **0-9**: Critical real-time (motor control, safety)
    /// - **10-49**: High priority (sensors, fast control loops)
    /// - **50-99**: Normal priority (processing, planning)
    /// - **100-199**: Low priority (logging, diagnostics)
    /// - **200+**: Background (telemetry, non-essential)
    pub fn order(mut self, order: u32) -> Self {
        self.config = self.config.order(order);
        self
    }

    /// Alias for `order()` - set execution priority.
    pub fn priority(self, priority: u32) -> Self {
        self.order(priority)
    }

    /// Set a node-specific tick rate in Hz.
    pub fn rate_hz(mut self, rate: f64) -> Self {
        self.config = self.config.rate_hz(rate);
        self
    }

    /// Mark this as a real-time node.
    pub fn rt(mut self) -> Self {
        self.config = self.config.rt();
        self
    }

    /// Set the WCET budget in microseconds.
    pub fn wcet_us(mut self, us: u64) -> Self {
        self.config = self.config.wcet_us(us);
        self
    }

    /// Set the deadline in microseconds.
    pub fn deadline_us(mut self, us: u64) -> Self {
        self.config = self.config.deadline_us(us);
        self
    }

    /// Set the deadline in milliseconds.
    pub fn deadline_ms(mut self, ms: u64) -> Self {
        self.config = self.config.deadline_ms(ms);
        self
    }

    /// Set the WCET budget in milliseconds.
    pub fn wcet_ms(mut self, ms: u64) -> Self {
        self.config = self.config.wcet_ms(ms);
        self
    }

    /// Request JIT compilation for this node.
    pub fn jit(mut self) -> Self {
        self.config = self.config.jit();
        self
    }

    /// Set an explicit execution tier.
    pub fn tier(mut self, tier: super::intelligence::NodeTier) -> Self {
        self.config = self.config.tier(tier);
        self
    }

    /// Finish configuration and add the node to the scheduler.
    ///
    /// This consumes the builder and registers the node.
    ///
    /// # Example
    /// ```rust,ignore
    /// scheduler.node(Box::new(my_node))
    ///     .order(0)
    ///     .done();
    /// ```
    pub fn done(self) -> &'a mut super::scheduler::Scheduler {
        self.scheduler.add_configured(self.config)
    }

    /// Alias for `done()`.
    pub fn build(self) -> &'a mut super::scheduler::Scheduler {
        self.done()
    }

    /// Alias for `done()`.
    pub fn add(self) -> &'a mut super::scheduler::Scheduler {
        self.done()
    }
}
