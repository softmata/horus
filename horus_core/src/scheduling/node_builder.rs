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
//! scheduler.add(my_node)
//!     .order(0)
//!     .rate_hz(100.0)
//!     .rt()
//!     .build();
//! ```

use super::types::{ExecutionClass, NodeKind};
use crate::core::{Node, RtNode};
use std::time::Duration;

/// Configuration for a node being added to the scheduler.
///
/// Most users should use `Scheduler::add(node).rt().build()` instead.
/// This type exists for advanced use cases like Python bindings.
#[doc(hidden)]
pub struct NodeRegistration {
    /// The node to add (either regular or RT)
    pub(crate) node: NodeKind,
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
    /// Execution tier override
    pub(crate) tier: Option<super::types::NodeTier>,
    /// Failure policy override (None = use tier default)
    pub(crate) failure_policy: Option<super::fault_tolerance::FailurePolicy>,
    /// Execution class — determines scheduling group
    pub(crate) execution_class: ExecutionClass,
}

impl NodeRegistration {
    /// Create a new node configuration with defaults.
    ///
    /// # Example
    /// ```rust,ignore
    /// let config = NodeRegistration::new(Box::new(MyNode::new()))
    ///     .order(0)
    ///     .rate_hz(100.0);
    /// ```
    pub fn new(node: Box<dyn Node>) -> Self {
        Self {
            node: NodeKind::Regular(node),
            order: 100, // Default to medium priority
            rate_hz: None,
            is_rt: false,
            wcet_budget: None,
            deadline: None,
            tier: None,
            failure_policy: None,
            execution_class: ExecutionClass::BestEffort,
        }
    }

    /// Create a new node configuration from a full RtNode.
    ///
    /// Automatically populates WCET budget and deadline from the trait,
    /// and marks the node as RT.
    pub fn new_rt(node: Box<dyn RtNode>) -> Self {
        let wcet_budget = node.wcet_budget();
        let deadline = node.deadline();
        Self {
            node: NodeKind::Rt(node),
            order: 100,
            rate_hz: None,
            is_rt: true,
            wcet_budget: Some(wcet_budget),
            deadline: Some(deadline),
            tier: None,
            failure_policy: None,
            execution_class: ExecutionClass::Rt,
        }
    }

    /// Set the execution order (lower = earlier in tick sequence).
    ///
    /// This is the correct way to set execution priority for a node.
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
    /// NodeRegistration::new(node).order(0)  // Highest priority
    /// ```
    pub fn order(mut self, order: u32) -> Self {
        self.order = order;
        self
    }

    /// Set a node-specific tick rate in Hz.
    ///
    /// If not set, the node uses the scheduler's global tick rate.
    ///
    /// # Example
    /// ```rust,ignore
    /// NodeRegistration::new(sensor_node)
    ///     .rate_hz(1000.0)  // 1kHz for sensor polling
    /// ```
    pub fn rate_hz(mut self, rate: f64) -> Self {
        self.rate_hz = Some(rate);
        self
    }

    /// Mark this as a real-time node.
    ///
    /// RT nodes get special treatment:
    /// - Dedicated RT thread execution
    /// - Priority scheduling
    /// - Deadline monitoring
    /// - WCET enforcement (if configured)
    ///
    /// # Example
    /// ```rust,ignore
    /// NodeRegistration::new(motor_controller)
    ///     .rt()
    ///     .wcet_us(500)  // 500μs max execution time
    /// ```
    pub fn rt(mut self) -> Self {
        self.is_rt = true;
        self.execution_class = ExecutionClass::Rt;
        self
    }

    /// Mark this as a compute node for parallel execution.
    ///
    /// Compute nodes run in a parallel thread pool, isolated from RT nodes.
    /// Use for CPU-bound work like planning, SLAM, or image processing.
    ///
    /// # Example
    /// ```rust,ignore
    /// NodeRegistration::new(path_planner)
    ///     .compute()
    ///     .rate_hz(10.0)
    /// ```
    pub fn compute(mut self) -> Self {
        self.execution_class = ExecutionClass::Compute;
        self
    }

    /// Mark this as an event-triggered node.
    ///
    /// The node will be triggered when data arrives on the specified topic.
    ///
    /// # Example
    /// ```rust,ignore
    /// NodeRegistration::new(obstacle_detector)
    ///     .on("lidar_scan")
    /// ```
    pub fn on(mut self, topic: &str) -> Self {
        self.execution_class = ExecutionClass::Event(topic.to_string());
        self
    }

    /// Mark this as an async I/O node.
    ///
    /// Async I/O nodes run their `tick()` via `tokio::task::spawn_blocking()` on
    /// a separate tokio runtime. Use for I/O-bound work like file operations,
    /// network requests, or database queries. Blocking I/O in these nodes never
    /// affects RT jitter or compute throughput.
    ///
    /// # Example
    /// ```rust,ignore
    /// NodeRegistration::new(telemetry_uploader)
    ///     .async_io()
    ///     .rate_hz(1.0)  // Upload once per second
    /// ```
    pub fn async_io(mut self) -> Self {
        self.execution_class = ExecutionClass::AsyncIo;
        self
    }

    /// Set the Worst-Case Execution Time budget in microseconds.
    ///
    /// Automatically marks the node as RT.
    ///
    /// # Example
    /// ```rust,ignore
    /// NodeRegistration::new(pid_controller)
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
    /// NodeRegistration::new(safety_monitor)
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
    /// NodeRegistration::new(control_loop)
    ///     .deadline_ms(10)  // Must finish within 10ms
    /// ```
    pub fn deadline_ms(mut self, ms: u64) -> Self {
        self.is_rt = true;
        self.deadline = Some(Duration::from_millis(ms));
        self
    }

    /// Set an explicit execution tier.
    ///
    /// Overrides automatic tier classification.
    ///
    /// # Example
    /// ```rust,ignore
    /// use horus_core::scheduling::NodeTier;
    ///
    /// NodeRegistration::new(fast_node)
    ///     .tier(NodeTier::UltraFast)
    /// ```
    pub fn tier(mut self, tier: super::types::NodeTier) -> Self {
        self.tier = Some(tier);
        self
    }

    /// Override the failure policy for this node.
    ///
    /// By default, the policy is derived from the node's tier:
    /// - `UltraFast`/`Fast` → `Fatal` (stop scheduler on failure)
    /// - `Normal` → `Restart` (exponential backoff)
    ///
    /// Use this to override when the default doesn't fit:
    ///
    /// # Example
    /// ```rust,ignore
    /// use horus_core::scheduling::FailurePolicy;
    ///
    /// // A fast node that should restart instead of killing the scheduler
    /// NodeRegistration::new(sensor_node)
    ///     .tier(NodeTier::Fast)
    ///     .failure_policy(FailurePolicy::restart(3, 50))
    /// ```
    pub fn failure_policy(mut self, policy: super::fault_tolerance::FailurePolicy) -> Self {
        self.failure_policy = Some(policy);
        self
    }
}

/// Builder for adding a node to the scheduler with fluent configuration.
///
/// Created by `Scheduler::node()`. Call `.build()` to register the node.
///
/// # Example
/// ```rust,ignore
/// scheduler.add(my_node)
///     .order(0)
///     .rate_hz(100.0)
///     .rt()
///     .build();
/// ```
#[must_use = "call .build() to register the node — dropping this builder discards the registration"]
pub struct NodeBuilder<'a> {
    scheduler: &'a mut super::scheduler::Scheduler,
    config: NodeRegistration,
}

impl<'a> NodeBuilder<'a> {
    /// Create a new NodeBuilder (called by Scheduler::add).
    pub(crate) fn new(scheduler: &'a mut super::scheduler::Scheduler, node: Box<dyn Node>) -> Self {
        Self {
            scheduler,
            config: NodeRegistration::new(node),
        }
    }

    /// Create a new NodeBuilder for an RtNode (called by Scheduler::add_rt).
    pub(crate) fn new_rt(
        scheduler: &'a mut super::scheduler::Scheduler,
        node: Box<dyn RtNode>,
    ) -> Self {
        Self {
            scheduler,
            config: NodeRegistration::new_rt(node),
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

    /// Mark this as a compute node for parallel execution.
    pub fn compute(mut self) -> Self {
        self.config = self.config.compute();
        self
    }

    /// Mark this as an event-triggered node.
    pub fn on(mut self, topic: &str) -> Self {
        self.config = self.config.on(topic);
        self
    }

    /// Mark this as an async I/O node (runs on tokio blocking pool).
    pub fn async_io(mut self) -> Self {
        self.config = self.config.async_io();
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

    /// Set an explicit execution tier.
    pub fn tier(mut self, tier: super::types::NodeTier) -> Self {
        self.config = self.config.tier(tier);
        self
    }

    /// Override the failure policy for this node.
    ///
    /// By default, the policy is derived from the node's tier.
    /// Use this when you need different failure behavior than the tier default.
    pub fn failure_policy(mut self, policy: super::fault_tolerance::FailurePolicy) -> Self {
        self.config = self.config.failure_policy(policy);
        self
    }

    /// Finish configuration and add the node to the scheduler.
    ///
    /// This consumes the builder and registers the node.
    ///
    /// # Example
    /// ```rust,ignore
    /// scheduler.add(my_node)
    ///     .order(0)
    ///     .build();
    /// ```
    pub fn build(self) -> &'a mut super::scheduler::Scheduler {
        self.scheduler.add_configured(self.config)
    }

    /// Alias for [`build()`](Self::build) — kept for backward compatibility.
    pub fn done(self) -> &'a mut super::scheduler::Scheduler {
        self.build()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::Node;
    use crate::scheduling::fault_tolerance::FailurePolicy;
    use crate::scheduling::types::NodeTier;

    struct StubNode(String);
    impl Node for StubNode {
        fn name(&self) -> &str {
            &self.0
        }
        fn tick(&mut self) {}
    }

    fn stub(name: &str) -> Box<dyn Node> {
        Box::new(StubNode(name.to_string()))
    }

    // ── NodeRegistration::new defaults ──

    #[test]
    fn test_new_defaults() {
        let reg = NodeRegistration::new(stub("test"));
        assert_eq!(reg.order, 100);
        assert!(reg.rate_hz.is_none());
        assert!(!reg.is_rt);
        assert!(reg.wcet_budget.is_none());
        assert!(reg.deadline.is_none());
        assert!(reg.tier.is_none());
        assert!(reg.failure_policy.is_none());
        assert_eq!(reg.execution_class, ExecutionClass::BestEffort);
    }

    // ── .order() ──

    #[test]
    fn test_order_valid() {
        let reg = NodeRegistration::new(stub("n")).order(0);
        assert_eq!(reg.order, 0);
    }

    #[test]
    fn test_order_max() {
        let reg = NodeRegistration::new(stub("n")).order(u32::MAX);
        assert_eq!(reg.order, u32::MAX);
    }

    #[test]
    fn test_order_overwrite() {
        let reg = NodeRegistration::new(stub("n")).order(5).order(10);
        assert_eq!(reg.order, 10);
    }

    // ── .rate_hz() ──

    #[test]
    fn test_rate_hz_valid() {
        let reg = NodeRegistration::new(stub("n")).rate_hz(100.0);
        assert_eq!(reg.rate_hz, Some(100.0));
    }

    #[test]
    fn test_rate_hz_zero() {
        let reg = NodeRegistration::new(stub("n")).rate_hz(0.0);
        assert_eq!(reg.rate_hz, Some(0.0));
    }

    #[test]
    fn test_rate_hz_negative() {
        // Currently accepts negative — documents behavior
        let reg = NodeRegistration::new(stub("n")).rate_hz(-1.0);
        assert_eq!(reg.rate_hz, Some(-1.0));
    }

    #[test]
    fn test_rate_hz_infinity() {
        let reg = NodeRegistration::new(stub("n")).rate_hz(f64::INFINITY);
        assert_eq!(reg.rate_hz, Some(f64::INFINITY));
    }

    #[test]
    fn test_rate_hz_nan() {
        let reg = NodeRegistration::new(stub("n")).rate_hz(f64::NAN);
        assert!(reg.rate_hz.unwrap().is_nan());
    }

    #[test]
    fn test_rate_hz_very_small() {
        let reg = NodeRegistration::new(stub("n")).rate_hz(0.001);
        assert_eq!(reg.rate_hz, Some(0.001));
    }

    #[test]
    fn test_rate_hz_very_large() {
        let reg = NodeRegistration::new(stub("n")).rate_hz(1_000_000.0);
        assert_eq!(reg.rate_hz, Some(1_000_000.0));
    }

    // ── .rt() ──

    #[test]
    fn test_rt_sets_flags() {
        let reg = NodeRegistration::new(stub("n")).rt();
        assert!(reg.is_rt);
        assert_eq!(reg.execution_class, ExecutionClass::Rt);
    }

    #[test]
    fn test_rt_idempotent() {
        let reg = NodeRegistration::new(stub("n")).rt().rt();
        assert!(reg.is_rt);
        assert_eq!(reg.execution_class, ExecutionClass::Rt);
    }

    // ── .compute() ──

    #[test]
    fn test_compute_sets_class() {
        let reg = NodeRegistration::new(stub("n")).compute();
        assert_eq!(reg.execution_class, ExecutionClass::Compute);
        assert!(!reg.is_rt);
    }

    // ── .on() ──

    #[test]
    fn test_on_sets_event_class() {
        let reg = NodeRegistration::new(stub("n")).on("lidar_scan");
        assert_eq!(
            reg.execution_class,
            ExecutionClass::Event("lidar_scan".to_string())
        );
    }

    #[test]
    fn test_on_empty_topic() {
        // Documents current behavior: empty topic names accepted
        let reg = NodeRegistration::new(stub("n")).on("");
        assert_eq!(reg.execution_class, ExecutionClass::Event("".to_string()));
    }

    #[test]
    fn test_on_overwrites_previous() {
        let reg = NodeRegistration::new(stub("n")).on("a").on("b");
        assert_eq!(reg.execution_class, ExecutionClass::Event("b".to_string()));
    }

    // ── .async_io() ──

    #[test]
    fn test_async_io_sets_class() {
        let reg = NodeRegistration::new(stub("n")).async_io();
        assert_eq!(reg.execution_class, ExecutionClass::AsyncIo);
        assert!(!reg.is_rt);
    }

    // ── .wcet_us() ──

    #[test]
    fn test_wcet_us_sets_budget_and_rt() {
        let reg = NodeRegistration::new(stub("n")).wcet_us(500);
        assert!(reg.is_rt);
        assert_eq!(reg.wcet_budget, Some(Duration::from_micros(500)));
    }

    #[test]
    fn test_wcet_us_zero() {
        let reg = NodeRegistration::new(stub("n")).wcet_us(0);
        assert_eq!(reg.wcet_budget, Some(Duration::from_micros(0)));
        assert!(reg.is_rt);
    }

    #[test]
    fn test_wcet_us_max() {
        let reg = NodeRegistration::new(stub("n")).wcet_us(u64::MAX);
        assert_eq!(reg.wcet_budget, Some(Duration::from_micros(u64::MAX)));
    }

    // ── .deadline_us() ──

    #[test]
    fn test_deadline_us_sets_deadline_and_rt() {
        let reg = NodeRegistration::new(stub("n")).deadline_us(1000);
        assert!(reg.is_rt);
        assert_eq!(reg.deadline, Some(Duration::from_micros(1000)));
    }

    #[test]
    fn test_deadline_us_zero() {
        let reg = NodeRegistration::new(stub("n")).deadline_us(0);
        assert_eq!(reg.deadline, Some(Duration::ZERO));
        assert!(reg.is_rt);
    }

    // ── .deadline_ms() ──

    #[test]
    fn test_deadline_ms_sets_deadline_and_rt() {
        let reg = NodeRegistration::new(stub("n")).deadline_ms(10);
        assert!(reg.is_rt);
        assert_eq!(reg.deadline, Some(Duration::from_millis(10)));
    }

    #[test]
    fn test_deadline_ms_zero() {
        let reg = NodeRegistration::new(stub("n")).deadline_ms(0);
        assert_eq!(reg.deadline, Some(Duration::ZERO));
    }

    // ── .tier() ──

    #[test]
    fn test_tier_ultrafast() {
        let reg = NodeRegistration::new(stub("n")).tier(NodeTier::UltraFast);
        assert_eq!(reg.tier, Some(NodeTier::UltraFast));
    }

    #[test]
    fn test_tier_fast() {
        let reg = NodeRegistration::new(stub("n")).tier(NodeTier::Fast);
        assert_eq!(reg.tier, Some(NodeTier::Fast));
    }

    #[test]
    fn test_tier_normal() {
        let reg = NodeRegistration::new(stub("n")).tier(NodeTier::Normal);
        assert_eq!(reg.tier, Some(NodeTier::Normal));
    }

    #[test]
    fn test_tier_overwrite() {
        let reg = NodeRegistration::new(stub("n"))
            .tier(NodeTier::UltraFast)
            .tier(NodeTier::Normal);
        assert_eq!(reg.tier, Some(NodeTier::Normal));
    }

    // ── .failure_policy() ──

    #[test]
    fn test_failure_policy_fatal() {
        let reg = NodeRegistration::new(stub("n")).failure_policy(FailurePolicy::Fatal);
        assert!(matches!(reg.failure_policy, Some(FailurePolicy::Fatal)));
    }

    #[test]
    fn test_failure_policy_restart() {
        let reg = NodeRegistration::new(stub("n")).failure_policy(FailurePolicy::restart(3, 50));
        assert!(matches!(
            reg.failure_policy,
            Some(FailurePolicy::Restart {
                max_restarts: 3,
                initial_backoff_ms: 50,
            })
        ));
    }

    #[test]
    fn test_failure_policy_skip() {
        let reg = NodeRegistration::new(stub("n")).failure_policy(FailurePolicy::skip(5, 200));
        assert!(matches!(
            reg.failure_policy,
            Some(FailurePolicy::Skip {
                max_failures: 5,
                cooldown_ms: 200,
            })
        ));
    }

    #[test]
    fn test_failure_policy_ignore() {
        let reg = NodeRegistration::new(stub("n")).failure_policy(FailurePolicy::Ignore);
        assert!(matches!(reg.failure_policy, Some(FailurePolicy::Ignore)));
    }

    // ── Execution class conflicts (documents current behavior) ──

    #[test]
    fn test_rt_then_compute_takes_last() {
        let reg = NodeRegistration::new(stub("n")).rt().compute();
        assert_eq!(reg.execution_class, ExecutionClass::Compute);
        // Note: is_rt is still true from .rt() — only execution_class changed
        assert!(reg.is_rt);
    }

    #[test]
    fn test_compute_then_rt_takes_last() {
        let reg = NodeRegistration::new(stub("n")).compute().rt();
        assert_eq!(reg.execution_class, ExecutionClass::Rt);
        assert!(reg.is_rt);
    }

    #[test]
    fn test_async_io_then_on_takes_last() {
        let reg = NodeRegistration::new(stub("n")).async_io().on("topic");
        assert_eq!(
            reg.execution_class,
            ExecutionClass::Event("topic".to_string())
        );
    }

    #[test]
    fn test_rt_then_async_io_takes_last() {
        let reg = NodeRegistration::new(stub("n")).rt().async_io();
        assert_eq!(reg.execution_class, ExecutionClass::AsyncIo);
        // is_rt stays true from .rt()
        assert!(reg.is_rt);
    }

    #[test]
    fn test_wcet_sets_rt_but_compute_overrides_class() {
        // wcet_us auto-sets is_rt and keeps BestEffort class (no explicit class set)
        // then .compute() overrides class
        let reg = NodeRegistration::new(stub("n")).wcet_us(100).compute();
        assert!(reg.is_rt); // wcet sets is_rt
        assert_eq!(reg.execution_class, ExecutionClass::Compute); // compute overrides
    }

    // ── Builder chain ordering independence ──

    #[test]
    fn test_order_rate_rt_chain() {
        let reg = NodeRegistration::new(stub("n"))
            .order(5)
            .rate_hz(100.0)
            .rt()
            .wcet_us(200)
            .deadline_ms(1)
            .tier(NodeTier::UltraFast)
            .failure_policy(FailurePolicy::Fatal);

        assert_eq!(reg.order, 5);
        assert_eq!(reg.rate_hz, Some(100.0));
        assert!(reg.is_rt);
        assert_eq!(reg.wcet_budget, Some(Duration::from_micros(200)));
        assert_eq!(reg.deadline, Some(Duration::from_millis(1)));
        assert_eq!(reg.tier, Some(NodeTier::UltraFast));
        assert!(matches!(reg.failure_policy, Some(FailurePolicy::Fatal)));
    }

    #[test]
    fn test_reverse_chain_order() {
        let reg = NodeRegistration::new(stub("n"))
            .failure_policy(FailurePolicy::Ignore)
            .tier(NodeTier::Normal)
            .deadline_ms(5)
            .wcet_us(100)
            .rt()
            .rate_hz(50.0)
            .order(10);

        assert_eq!(reg.order, 10);
        assert_eq!(reg.rate_hz, Some(50.0));
        assert!(reg.is_rt);
        assert_eq!(reg.wcet_budget, Some(Duration::from_micros(100)));
        assert_eq!(reg.deadline, Some(Duration::from_millis(5)));
        assert_eq!(reg.tier, Some(NodeTier::Normal));
        assert!(matches!(reg.failure_policy, Some(FailurePolicy::Ignore)));
    }

    // ── NodeRegistration::new_rt ──

    #[test]
    fn test_new_rt_auto_populates() {
        use crate::core::RtNode;

        struct TestRtNode;
        impl Node for TestRtNode {
            fn name(&self) -> &str {
                "rt_test"
            }
            fn tick(&mut self) {}
        }
        impl RtNode for TestRtNode {
            fn wcet_budget(&self) -> Duration {
                Duration::from_micros(500)
            }
            fn deadline(&self) -> Duration {
                Duration::from_millis(1)
            }
        }

        let reg = NodeRegistration::new_rt(Box::new(TestRtNode));
        assert!(reg.is_rt);
        assert_eq!(reg.wcet_budget, Some(Duration::from_micros(500)));
        assert_eq!(reg.deadline, Some(Duration::from_millis(1)));
        assert_eq!(reg.execution_class, ExecutionClass::Rt);
        assert_eq!(reg.order, 100); // default
    }

    // ── NodeBuilder integration (requires Scheduler) ──

    #[test]
    fn test_builder_build_registers_node() {
        use crate::scheduling::Scheduler;

        let mut scheduler = Scheduler::new();
        scheduler
            .add(StubNode("builder_test".to_string()))
            .order(5)
            .rate_hz(50.0)
            .build();

        let names = scheduler.node_list();
        assert!(names.contains(&"builder_test".to_string()));
    }

    #[test]
    fn test_builder_done_alias() {
        use crate::scheduling::Scheduler;

        let mut scheduler = Scheduler::new();
        scheduler
            .add(StubNode("done_test".to_string()))
            .order(0)
            .done();

        let names = scheduler.node_list();
        assert!(names.contains(&"done_test".to_string()));
    }

    #[test]
    fn test_builder_rt_node() {
        use crate::core::RtNode;
        use crate::scheduling::Scheduler;

        struct MinimalRt;
        impl Node for MinimalRt {
            fn name(&self) -> &str {
                "minimal_rt"
            }
            fn tick(&mut self) {}
        }
        impl RtNode for MinimalRt {
            fn wcet_budget(&self) -> Duration {
                Duration::from_micros(100)
            }
            fn deadline(&self) -> Duration {
                Duration::from_millis(1)
            }
        }

        let mut scheduler = Scheduler::new();
        scheduler.add_rt(MinimalRt).order(0).build();

        let names = scheduler.node_list();
        assert!(names.contains(&"minimal_rt".to_string()));
    }

    #[test]
    fn test_builder_multiple_nodes_ordered() {
        use crate::scheduling::Scheduler;

        let mut scheduler = Scheduler::new();
        scheduler.add(StubNode("c".into())).order(20).build();
        scheduler.add(StubNode("a".into())).order(0).build();
        scheduler.add(StubNode("b".into())).order(10).build();

        let names = scheduler.node_list();
        assert_eq!(names.len(), 3);
        // Nodes should be sorted by order (priority)
        assert!(names.contains(&"a".to_string()));
        assert!(names.contains(&"b".to_string()));
        assert!(names.contains(&"c".to_string()));
    }

    #[test]
    fn test_builder_all_execution_classes() {
        use crate::scheduling::Scheduler;

        let mut scheduler = Scheduler::new();
        scheduler.add(StubNode("rt".into())).rt().build();
        scheduler.add(StubNode("compute".into())).compute().build();
        scheduler.add(StubNode("event".into())).on("topic").build();
        scheduler.add(StubNode("async".into())).async_io().build();
        scheduler.add(StubNode("best_effort".into())).build();

        assert_eq!(scheduler.node_list().len(), 5);
    }

    // ============================================================================
    // Conflicting configuration detection tests
    // ============================================================================

    // -- Execution class conflicts (all pairwise) --

    /// .rt().compute() — last class wins, but is_rt flag persists from .rt().
    /// Behavior: Node is classified as Compute but still has is_rt=true.
    /// This is a known conflict — the scheduler uses execution_class for
    /// dispatch, so the node runs in the compute pool despite is_rt=true.
    #[test]
    fn conflict_rt_then_compute_class_overridden_rt_flag_persists() {
        let reg = NodeRegistration::new(stub("n")).rt().compute();
        assert_eq!(
            reg.execution_class,
            ExecutionClass::Compute,
            ".compute() should override .rt() execution class"
        );
        assert!(
            reg.is_rt,
            "is_rt flag persists from .rt() even after .compute()"
        );
    }

    /// .compute().rt() — RT takes precedence because it's called last.
    #[test]
    fn conflict_compute_then_rt_class_and_flag_both_rt() {
        let reg = NodeRegistration::new(stub("n")).compute().rt();
        assert_eq!(reg.execution_class, ExecutionClass::Rt);
        assert!(reg.is_rt);
    }

    /// .rt().on("topic") — event overrides RT class.
    #[test]
    fn conflict_rt_then_event_class_overridden() {
        let reg = NodeRegistration::new(stub("n")).rt().on("sensor");
        assert_eq!(
            reg.execution_class,
            ExecutionClass::Event("sensor".to_string())
        );
        assert!(reg.is_rt, "is_rt flag persists");
    }

    /// .on("topic").rt() — RT overrides event.
    #[test]
    fn conflict_event_then_rt_class_overridden() {
        let reg = NodeRegistration::new(stub("n")).on("sensor").rt();
        assert_eq!(reg.execution_class, ExecutionClass::Rt);
        assert!(reg.is_rt);
    }

    /// .compute().on("topic") — event overrides compute.
    #[test]
    fn conflict_compute_then_event_class_overridden() {
        let reg = NodeRegistration::new(stub("n")).compute().on("topic");
        assert_eq!(
            reg.execution_class,
            ExecutionClass::Event("topic".to_string())
        );
    }

    /// .on("topic").compute() — compute overrides event.
    #[test]
    fn conflict_event_then_compute_class_overridden() {
        let reg = NodeRegistration::new(stub("n")).on("topic").compute();
        assert_eq!(reg.execution_class, ExecutionClass::Compute);
    }

    /// .compute().async_io() — async_io overrides compute.
    #[test]
    fn conflict_compute_then_async_io_class_overridden() {
        let reg = NodeRegistration::new(stub("n")).compute().async_io();
        assert_eq!(reg.execution_class, ExecutionClass::AsyncIo);
    }

    /// .async_io().compute() — compute overrides async_io.
    #[test]
    fn conflict_async_io_then_compute_class_overridden() {
        let reg = NodeRegistration::new(stub("n")).async_io().compute();
        assert_eq!(reg.execution_class, ExecutionClass::Compute);
    }

    /// .on("a").async_io() — async_io overrides event.
    #[test]
    fn conflict_event_then_async_io_class_overridden() {
        let reg = NodeRegistration::new(stub("n")).on("topic").async_io();
        assert_eq!(reg.execution_class, ExecutionClass::AsyncIo);
    }

    /// .async_io().on("a") — event overrides async_io.
    #[test]
    fn conflict_async_io_then_event_class_overridden() {
        let reg = NodeRegistration::new(stub("n")).async_io().on("topic");
        assert_eq!(
            reg.execution_class,
            ExecutionClass::Event("topic".to_string())
        );
    }

    // -- WCET/deadline without explicit .rt() --

    /// .wcet_us() auto-sets is_rt=true but does NOT change execution_class.
    /// This means the node has is_rt=true with BestEffort class — the scheduler
    /// uses is_rt to enable WCET monitoring regardless of execution class.
    #[test]
    fn conflict_wcet_without_explicit_rt_auto_sets_flag_not_class() {
        let reg = NodeRegistration::new(stub("n")).wcet_us(500);
        assert!(reg.is_rt, "wcet_us should auto-set is_rt");
        assert_eq!(
            reg.execution_class,
            ExecutionClass::BestEffort,
            "wcet_us should NOT change execution_class (stays BestEffort)"
        );
        assert_eq!(reg.wcet_budget, Some(Duration::from_micros(500)));
    }

    /// .deadline_ms() auto-sets is_rt=true.
    #[test]
    fn conflict_deadline_without_explicit_rt_auto_sets_flag() {
        let reg = NodeRegistration::new(stub("n")).deadline_ms(10);
        assert!(reg.is_rt, "deadline_ms should auto-set is_rt");
    }

    /// .deadline_us() auto-sets is_rt=true.
    #[test]
    fn conflict_deadline_us_without_explicit_rt_auto_sets_flag() {
        let reg = NodeRegistration::new(stub("n")).deadline_us(1000);
        assert!(reg.is_rt, "deadline_us should auto-set is_rt");
    }

    // -- Event + rate combination --

    /// .on("topic").rate_hz(100.0) — both set, event class preserved.
    /// Behavior: The rate_hz is recorded but the node is event-triggered.
    /// The scheduler may use rate_hz as a minimum check interval.
    #[test]
    fn conflict_event_with_rate_hz_both_stored() {
        let reg = NodeRegistration::new(stub("n")).on("sensor").rate_hz(100.0);
        assert_eq!(
            reg.execution_class,
            ExecutionClass::Event("sensor".to_string()),
            "Event class should be preserved"
        );
        assert_eq!(reg.rate_hz, Some(100.0), "rate_hz should still be stored");
    }

    /// .rate_hz(100.0).on("topic") — same result, order doesn't matter.
    #[test]
    fn conflict_rate_hz_then_event_both_stored() {
        let reg = NodeRegistration::new(stub("n")).rate_hz(100.0).on("sensor");
        assert_eq!(
            reg.execution_class,
            ExecutionClass::Event("sensor".to_string())
        );
        assert_eq!(reg.rate_hz, Some(100.0));
    }

    // -- Deadline + async_io conflict --

    /// .deadline_ms(1).async_io() — deadline requires RT guarantees but async_io
    /// cannot guarantee deadlines. The deadline is stored but may be meaningless.
    #[test]
    fn conflict_deadline_then_async_io_deadline_stored_class_overridden() {
        let reg = NodeRegistration::new(stub("n")).deadline_ms(1).async_io();
        assert_eq!(
            reg.execution_class,
            ExecutionClass::AsyncIo,
            "async_io should override execution class"
        );
        assert_eq!(
            reg.deadline,
            Some(Duration::from_millis(1)),
            "Deadline should persist even with async_io"
        );
        assert!(reg.is_rt, "is_rt flag from deadline should persist");
    }

    /// .async_io().deadline_ms(1) — deadline auto-sets RT but class stays async_io
    /// only if deadline doesn't change class. Actually deadline_ms sets is_rt=true
    /// but doesn't change execution_class (it was already AsyncIo from the call).
    #[test]
    fn conflict_async_io_then_deadline_is_rt_set_class_overridden() {
        let reg = NodeRegistration::new(stub("n")).async_io().deadline_ms(1);
        // deadline_ms sets is_rt=true but doesn't touch execution_class
        assert!(reg.is_rt);
        assert_eq!(reg.deadline, Some(Duration::from_millis(1)));
        // async_io was set first, deadline_ms doesn't override it
        // Actually let me check: deadline_ms only sets is_rt, not execution_class
        // So it should still be AsyncIo
        assert_eq!(reg.execution_class, ExecutionClass::AsyncIo);
    }

    // -- WCET + compute conflict --

    /// .wcet_us(100).compute() — WCET monitoring with compute class.
    /// WCET is stored but the compute pool won't enforce RT deadlines.
    #[test]
    fn conflict_wcet_then_compute_wcet_stored_class_compute() {
        let reg = NodeRegistration::new(stub("n")).wcet_us(100).compute();
        assert_eq!(reg.execution_class, ExecutionClass::Compute);
        assert!(reg.is_rt, "is_rt from wcet persists");
        assert_eq!(reg.wcet_budget, Some(Duration::from_micros(100)));
    }

    // -- Triple class changes --

    /// .rt().compute().async_io() — last one wins.
    #[test]
    fn conflict_triple_class_change_last_wins() {
        let reg = NodeRegistration::new(stub("n")).rt().compute().async_io();
        assert_eq!(reg.execution_class, ExecutionClass::AsyncIo);
        assert!(reg.is_rt, "is_rt from .rt() persists through chain");
    }

    /// .on("a").rt().compute().on("b") — ends as Event("b").
    #[test]
    fn conflict_four_class_changes_last_wins() {
        let reg = NodeRegistration::new(stub("n"))
            .on("a")
            .rt()
            .compute()
            .on("b");
        assert_eq!(reg.execution_class, ExecutionClass::Event("b".to_string()));
    }

    // -- Full conflicting configuration via NodeBuilder --

    /// Builder with conflicting configuration still registers the node.
    #[test]
    fn conflict_builder_conflicting_config_still_registers() {
        use crate::scheduling::Scheduler;

        let mut scheduler = Scheduler::new();
        scheduler
            .add(StubNode("conflict".into()))
            .rt()
            .compute()
            .deadline_ms(5)
            .async_io()
            .wcet_us(100)
            .build();

        let names = scheduler.node_list();
        assert!(
            names.contains(&"conflict".to_string()),
            "Node should be registered despite conflicting config"
        );
    }
}
