//! Node Builder - Fluent API for adding nodes to the scheduler
//!
//! The `NodeBuilder` provides a chainable interface for configuring nodes
//! when adding them to the scheduler.
//!
//! # Example
//!
//! ```rust,ignore
//! use horus::prelude::*;
//!
//! let mut scheduler = Scheduler::new();
//!
//! scheduler.add(my_node)
//!     .order(0)
//!     .rate(100_u64.hz())
//!     .budget(500_u64.us())
//!     .deadline(1_u64.ms())
//!     .build()?;
//! ```

use super::types::{ExecutionClass, NodeKind};
use crate::core::duration_ext::Frequency;
use crate::core::{Miss, Node};
use crate::error::{HorusResult, ValidationError};
use std::time::Duration;

/// Configuration for a node being added to the scheduler.
///
/// Most users should use `Scheduler::add(node).budget(N.us()).build()` instead.
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
    /// tick budget for RT nodes
    pub(crate) tick_budget: Option<Duration>,
    /// Deadline for RT nodes
    pub(crate) deadline: Option<Duration>,
    /// Failure policy override
    pub(crate) failure_policy: Option<super::fault_tolerance::FailurePolicy>,
    /// What to do on deadline miss
    pub(crate) miss_policy: Miss,
    /// Execution class — determines scheduling group
    pub(crate) execution_class: ExecutionClass,
    /// Source frequency for deferred auto-derivation at build time
    source_freq: Option<Frequency>,
}

impl NodeRegistration {
    /// Create a new node configuration with defaults.
    ///
    /// # Example
    /// ```rust,ignore
    /// let config = NodeRegistration::new(Box::new(MyNode::new()))
    ///     .order(0)
    ///     .rate(100_u64.hz());
    /// ```
    pub fn new(node: Box<dyn Node>) -> Self {
        Self {
            node: NodeKind::new(node),
            order: 100, // Default to medium priority
            rate_hz: None,
            is_rt: false,
            tick_budget: None,
            deadline: None,
            failure_policy: None,
            miss_policy: Miss::default(),
            execution_class: ExecutionClass::BestEffort,
            source_freq: None,
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

    /// Mark this as a compute node for parallel execution.
    ///
    /// Compute nodes run in a parallel thread pool, isolated from RT nodes.
    /// Use for CPU-bound work like planning, SLAM, or image processing.
    ///
    /// # Example
    /// ```rust,ignore
    /// NodeRegistration::new(path_planner)
    ///     .compute()
    ///     .rate(10_u64.hz())
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
    ///     .rate(1_u64.hz())  // Upload once per second
    /// ```
    pub fn async_io(mut self) -> Self {
        self.execution_class = ExecutionClass::AsyncIo;
        self
    }

    /// Set the node's tick rate.
    ///
    /// For nodes with default execution class (`BestEffort`), this auto-enables
    /// RT scheduling and derives budget (80%) and deadline (95%) from the period.
    /// For nodes already assigned an execution class (`.compute()`, `.on()`,
    /// `.async_io()`), this sets rate-limiting without RT implications.
    ///
    /// Method call order does not matter — auto-derivation is deferred to
    /// `.build()` time.
    ///
    /// # Example
    /// ```rust,ignore
    /// use horus::prelude::*;
    ///
    /// // RT node — auto-derives budget & deadline
    /// NodeRegistration::new(Box::new(motor_ctrl))
    ///     .rate(100_u64.hz())
    ///
    /// // Compute node — rate-limited, no RT
    /// NodeRegistration::new(Box::new(planner))
    ///     .compute()
    ///     .rate(10_u64.hz())
    /// ```
    pub fn rate(mut self, freq: Frequency) -> Self {
        self.rate_hz = Some(freq.value());
        self.source_freq = Some(freq);
        self
    }

    /// Set the tick budget as a `Duration`.
    ///
    /// Automatically marks the node as RT.
    ///
    /// # Example
    /// ```rust,ignore
    /// use horus::prelude::*;
    ///
    /// NodeRegistration::new(Box::new(pid_controller))
    ///     .budget(200_u64.us())
    /// ```
    pub fn budget(mut self, budget: Duration) -> Self {
        self.is_rt = true;
        self.tick_budget = Some(budget);
        self.execution_class = ExecutionClass::Rt;
        self
    }

    /// Set the deadline as a `Duration`.
    ///
    /// Automatically marks the node as RT.
    ///
    /// # Example
    /// ```rust,ignore
    /// use horus::prelude::*;
    ///
    /// NodeRegistration::new(Box::new(safety_monitor))
    ///     .deadline(1_u64.ms())
    /// ```
    pub fn deadline(mut self, deadline: Duration) -> Self {
        self.is_rt = true;
        self.deadline = Some(deadline);
        self.execution_class = ExecutionClass::Rt;
        self
    }

    /// Set the deadline miss policy.
    ///
    /// # Example
    /// ```rust,ignore
    /// use horus::prelude::*;
    ///
    /// NodeRegistration::new(Box::new(motor))
    ///     .rate(100_u64.hz())
    ///     .on_miss(Miss::Degrade)
    /// ```
    pub fn on_miss(mut self, policy: Miss) -> Self {
        self.miss_policy = policy;
        self
    }

    /// Override the failure policy for this node.
    ///
    /// # Example
    /// ```rust,ignore
    /// use horus_core::scheduling::FailurePolicy;
    ///
    /// NodeRegistration::new(sensor_node)
    ///     .failure_policy(FailurePolicy::restart(3, 50_u64.ms()))
    /// ```
    pub fn failure_policy(mut self, policy: super::fault_tolerance::FailurePolicy) -> Self {
        self.failure_policy = Some(policy);
        self
    }

    /// Apply deferred auto-derivation from `.rate(Frequency)`.
    ///
    /// Called automatically by [`NodeBuilder::build()`] before validation.
    /// If `.rate()` was called and no explicit execution class was set,
    /// this auto-enables RT scheduling and derives budget/deadline from
    /// the frequency. For non-RT execution classes, only the rate is kept
    /// for rate-limiting.
    pub(crate) fn finalize(&mut self) {
        if let Some(freq) = self.source_freq {
            // Auto-derive budget/deadline if RT or will become RT
            let will_be_rt =
                self.is_rt || matches!(self.execution_class, ExecutionClass::BestEffort);
            if will_be_rt {
                if self.tick_budget.is_none() {
                    self.tick_budget = Some(freq.budget_default());
                }
                if self.deadline.is_none() {
                    self.deadline = Some(freq.deadline_default());
                }
            }
            // Auto-enable RT if no explicit execution class was set
            if matches!(self.execution_class, ExecutionClass::BestEffort) {
                self.is_rt = true;
                self.execution_class = ExecutionClass::Rt;
            }
        }
    }

    /// Validate the node configuration, catching conflicts and invalid values.
    ///
    /// Calls [`finalize()`](Self::finalize) first to apply deferred auto-derivation
    /// from `.rate()`. Called automatically by [`NodeBuilder::build()`]. You only need
    /// to call this manually if you're using `NodeRegistration` directly (e.g. from
    /// Python bindings).
    pub fn validate(&mut self) -> HorusResult<()> {
        self.finalize();
        let node_name = self.node.name();

        // Validate rate_hz: must be finite and positive
        if let Some(rate) = self.rate_hz {
            if !rate.is_finite() || rate <= 0.0 {
                return Err(ValidationError::InvalidValue {
                    field: "rate_hz".into(),
                    value: format!("{}", rate),
                    reason: "must be finite and > 0".into(),
                }
                .into());
            }
        }

        // Validate deadline: must be > 0
        if let Some(deadline) = self.deadline {
            if deadline.is_zero() {
                return Err(ValidationError::InvalidValue {
                    field: "deadline".into(),
                    value: "0".into(),
                    reason:
                        "deadline must be > 0 (a zero deadline is meaningless for RT guarantees)"
                            .into(),
                }
                .into());
            }
        }

        // Validate budget: must be > 0 if set
        if let Some(budget) = self.tick_budget {
            if budget.is_zero() {
                return Err(ValidationError::InvalidValue {
                    field: "budget".into(),
                    value: "0".into(),
                    reason: "tick budget must be > 0".into(),
                }
                .into());
            }
        }

        // Validate execution class vs is_rt flag consistency
        if self.is_rt
            && !matches!(
                self.execution_class,
                ExecutionClass::Rt | ExecutionClass::BestEffort
            )
        {
            let class_name = match &self.execution_class {
                ExecutionClass::Compute => "compute",
                ExecutionClass::AsyncIo => "async_io",
                ExecutionClass::Event(_) => "event",
                _ => unreachable!(),
            };
            return Err(ValidationError::Conflict {
                field_a: "is_rt / .budget() / .deadline()".into(),
                field_b: format!(".{}()", class_name),
                reason: format!(
                    "node '{}' is marked as RT but has execution class '{}' — \
                     these are contradictory. Use either RT scheduling or {} scheduling, not both.",
                    node_name, class_name, class_name
                ),
            }
            .into());
        }

        // Validate RT-specific constraints on non-RT nodes
        if !self.is_rt && (self.tick_budget.is_some() || self.deadline.is_some()) {
            // This shouldn't happen since budget/deadline auto-set is_rt,
            // but guard against manual NodeRegistration construction.
            return Err(ValidationError::Conflict {
                field_a: "tick_budget / deadline".into(),
                field_b: "is_rt = false".into(),
                reason: format!(
                    "node '{}' has budget/deadline set but is not marked as RT",
                    node_name
                ),
            }
            .into());
        }

        Ok(())
    }
}

/// Builder for adding a node to the scheduler with fluent configuration.
///
/// Created by `Scheduler::node()`. Call `.build()` to register the node.
///
/// # Example
/// ```rust,ignore
/// use horus::prelude::*;
///
/// scheduler.add(my_node)
///     .order(0)
///     .rate(100_u64.hz())
///     .budget(500_u64.us())
///     .build()?;
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

    /// Set the tick rate from a `Frequency` — primary way to configure RT nodes.
    ///
    /// Auto-derives budget (80% of period) and deadline (95% of period).
    pub fn rate(mut self, freq: Frequency) -> Self {
        self.config = self.config.rate(freq);
        self
    }

    /// Set the tick budget as a `Duration`.
    ///
    /// Use with `DurationExt`: `.budget(200_u64.us())` or `.budget(1_u64.ms())`.
    pub fn budget(mut self, budget: Duration) -> Self {
        self.config = self.config.budget(budget);
        self
    }

    /// Set the deadline as a `Duration`.
    ///
    /// Use with `DurationExt`: `.deadline(1_u64.ms())` or `.deadline(500_u64.us())`.
    pub fn deadline(mut self, deadline: Duration) -> Self {
        self.config = self.config.deadline(deadline);
        self
    }

    /// Set the deadline miss policy.
    pub fn on_miss(mut self, policy: Miss) -> Self {
        self.config = self.config.on_miss(policy);
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

    /// Override the failure policy for this node.
    ///
    /// Use this when you need different failure behavior than the default.
    pub fn failure_policy(mut self, policy: super::fault_tolerance::FailurePolicy) -> Self {
        self.config = self.config.failure_policy(policy);
        self
    }

    /// Finish configuration and add the node to the scheduler.
    ///
    /// Validates the configuration before registering. Returns an error if
    /// the configuration is contradictory (e.g. `.budget(N.us()).compute()`) or contains
    /// invalid values (e.g. `deadline(Duration::ZERO)`).
    ///
    /// # Example
    /// ```rust,ignore
    /// scheduler.add(my_node)
    ///     .order(0)
    ///     .build()?;
    /// ```
    pub fn build(mut self) -> HorusResult<&'a mut super::scheduler::Scheduler> {
        self.config.validate()?;
        Ok(self.scheduler.add_configured(self.config))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::duration_ext::DurationExt;
    use crate::core::Node;
    use crate::scheduling::fault_tolerance::FailurePolicy;

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
        assert!(reg.tick_budget.is_none());
        assert!(reg.deadline.is_none());
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

    // ── .rate() ──

    #[test]
    fn test_rate_valid() {
        let reg = NodeRegistration::new(stub("n")).rate(100_u64.hz());
        assert_eq!(reg.rate_hz, Some(100.0));
        assert!(reg.source_freq.is_some());
    }

    #[test]
    fn test_rate_very_small() {
        let reg = NodeRegistration::new(stub("n")).rate(0.001.hz());
        assert_eq!(reg.rate_hz, Some(0.001));
    }

    #[test]
    fn test_rate_very_large() {
        let reg = NodeRegistration::new(stub("n")).rate(1_000_000_u64.hz());
        assert_eq!(reg.rate_hz, Some(1_000_000.0));
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

    // ── .budget() ──

    #[test]
    fn test_budget_sets_budget_and_rt() {
        let reg = NodeRegistration::new(stub("n")).budget(500_u64.us());
        assert!(reg.is_rt);
        assert_eq!(reg.tick_budget, Some(500_u64.us()));
    }

    #[test]
    fn test_budget_zero_stored() {
        // Builder stores zero — validation catches it later in build()
        let reg = NodeRegistration::new(stub("n")).budget(Duration::ZERO);
        assert_eq!(reg.tick_budget, Some(Duration::ZERO));
        assert!(reg.is_rt);
    }

    #[test]
    fn test_budget_max() {
        let reg = NodeRegistration::new(stub("n")).budget(u64::MAX.us());
        assert_eq!(reg.tick_budget, Some(u64::MAX.us()));
    }

    // ── .deadline() ──

    #[test]
    fn test_deadline_sets_deadline_and_rt() {
        let reg = NodeRegistration::new(stub("n")).deadline(1000_u64.us());
        assert!(reg.is_rt);
        assert_eq!(reg.deadline, Some(1000_u64.us()));
    }

    #[test]
    fn test_deadline_zero_stored() {
        // Builder stores zero — validation catches it later in build()
        let reg = NodeRegistration::new(stub("n")).deadline(Duration::ZERO);
        assert_eq!(reg.deadline, Some(Duration::ZERO));
        assert!(reg.is_rt);
    }

    #[test]
    fn test_deadline_ms_sets_deadline_and_rt() {
        let reg = NodeRegistration::new(stub("n")).deadline(10_u64.ms());
        assert!(reg.is_rt);
        assert_eq!(reg.deadline, Some(10_u64.ms()));
    }

    // ── .failure_policy() ──

    #[test]
    fn test_failure_policy_fatal() {
        let reg = NodeRegistration::new(stub("n")).failure_policy(FailurePolicy::Fatal);
        assert!(matches!(reg.failure_policy, Some(FailurePolicy::Fatal)));
    }

    #[test]
    fn test_failure_policy_restart() {
        let reg = NodeRegistration::new(stub("n"))
            .failure_policy(FailurePolicy::restart(3, 50_u64.ms()));
        assert!(matches!(
            reg.failure_policy,
            Some(FailurePolicy::Restart {
                max_restarts: 3,
                ..
            })
        ));
    }

    #[test]
    fn test_failure_policy_skip() {
        let reg = NodeRegistration::new(stub("n"))
            .failure_policy(FailurePolicy::skip(5, 200_u64.ms()));
        assert!(matches!(
            reg.failure_policy,
            Some(FailurePolicy::Skip {
                max_failures: 5,
                ..
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
    fn test_budget_then_compute_takes_last_class() {
        let reg = NodeRegistration::new(stub("n")).budget(100_u64.us()).compute();
        assert_eq!(reg.execution_class, ExecutionClass::Compute);
        assert!(reg.is_rt); // budget sets is_rt, compute overrides class
    }

    #[test]
    fn test_compute_then_budget_takes_last_class() {
        let reg = NodeRegistration::new(stub("n")).compute().budget(100_u64.us());
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
    fn test_budget_then_async_io_takes_last_class() {
        let reg = NodeRegistration::new(stub("n")).budget(100_u64.us()).async_io();
        assert_eq!(reg.execution_class, ExecutionClass::AsyncIo);
        assert!(reg.is_rt); // budget sets is_rt, async_io overrides class
    }

    // ── Builder chain ordering independence ──

    #[test]
    fn test_order_rate_budget_chain() {
        let reg = NodeRegistration::new(stub("n"))
            .order(5)
            .rate(100_u64.hz())
            .budget(200_u64.us())
            .deadline(1_u64.ms())
            .failure_policy(FailurePolicy::Fatal);

        assert_eq!(reg.order, 5);
        assert_eq!(reg.rate_hz, Some(100.0));
        assert!(reg.is_rt);
        assert_eq!(reg.tick_budget, Some(200_u64.us()));
        assert_eq!(reg.deadline, Some(1_u64.ms()));
        assert!(matches!(reg.failure_policy, Some(FailurePolicy::Fatal)));
    }

    #[test]
    fn test_reverse_chain_order() {
        let reg = NodeRegistration::new(stub("n"))
            .failure_policy(FailurePolicy::Ignore)
            .deadline(5_u64.ms())
            .budget(100_u64.us())
            .rate(50_u64.hz())
            .order(10);

        assert_eq!(reg.order, 10);
        assert_eq!(reg.rate_hz, Some(50.0));
        assert!(reg.is_rt);
        assert_eq!(reg.tick_budget, Some(100_u64.us()));
        assert_eq!(reg.deadline, Some(5_u64.ms()));
        assert!(matches!(reg.failure_policy, Some(FailurePolicy::Ignore)));
    }

    // ── NodeBuilder integration (requires Scheduler) ──

    #[test]
    fn test_builder_build_registers_node() {
        use crate::scheduling::Scheduler;

        let mut scheduler = Scheduler::new();
        scheduler
            .add(StubNode("builder_test".to_string()))
            .order(5)
            .rate(50_u64.hz())
            .build()
            .unwrap();

        let names = scheduler.node_list();
        assert!(names.contains(&"builder_test".to_string()));
    }

    #[test]
    fn test_builder_rt_node() {
        use crate::scheduling::Scheduler;

        struct MinimalRt;
        impl Node for MinimalRt {
            fn name(&self) -> &str {
                "minimal_rt"
            }
            fn tick(&mut self) {}
        }

        let mut scheduler = Scheduler::new();
        scheduler
            .add(MinimalRt)
            .order(0)
            .budget(100_u64.us())
            .deadline(1_u64.ms())
            .build()
            .unwrap();

        let names = scheduler.node_list();
        assert!(names.contains(&"minimal_rt".to_string()));
    }

    #[test]
    fn test_builder_multiple_nodes_ordered() {
        use crate::scheduling::Scheduler;

        let mut scheduler = Scheduler::new();
        scheduler
            .add(StubNode("c".into()))
            .order(20)
            .build()
            .unwrap();
        scheduler
            .add(StubNode("a".into()))
            .order(0)
            .build()
            .unwrap();
        scheduler
            .add(StubNode("b".into()))
            .order(10)
            .build()
            .unwrap();

        let names = scheduler.node_list();
        assert_eq!(names.len(), 3);
        assert!(names.contains(&"a".to_string()));
        assert!(names.contains(&"b".to_string()));
        assert!(names.contains(&"c".to_string()));
    }

    #[test]
    fn test_builder_all_execution_classes() {
        use crate::scheduling::Scheduler;

        let mut scheduler = Scheduler::new();
        scheduler
            .add(StubNode("rt".into()))
            .budget(500_u64.us())
            .build()
            .unwrap();
        scheduler
            .add(StubNode("compute".into()))
            .compute()
            .build()
            .unwrap();
        scheduler
            .add(StubNode("event".into()))
            .on("topic")
            .build()
            .unwrap();
        scheduler
            .add(StubNode("async".into()))
            .async_io()
            .build()
            .unwrap();
        scheduler
            .add(StubNode("best_effort".into()))
            .build()
            .unwrap();

        assert_eq!(scheduler.node_list().len(), 5);
    }

    // ============================================================================
    // Validation tests
    // ============================================================================

    // -- validate() on valid configurations --

    #[test]
    fn validate_default_config_ok() {
        let mut reg = NodeRegistration::new(stub("n"));
        reg.validate().unwrap();
    }

    #[test]
    fn validate_budget_only_ok() {
        let mut reg = NodeRegistration::new(stub("n")).budget(500_u64.us());
        reg.validate().unwrap();
    }

    #[test]
    fn validate_compute_only_ok() {
        let mut reg = NodeRegistration::new(stub("n")).compute();
        reg.validate().unwrap();
    }

    #[test]
    fn validate_event_only_ok() {
        let mut reg = NodeRegistration::new(stub("n")).on("topic");
        reg.validate().unwrap();
    }

    #[test]
    fn validate_async_io_only_ok() {
        let mut reg = NodeRegistration::new(stub("n")).async_io();
        reg.validate().unwrap();
    }

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

    // -- budget/deadline auto-sets RT --

    /// .budget() auto-sets is_rt=true AND execution_class=Rt.
    #[test]
    fn budget_auto_sets_rt_flag_and_class() {
        let reg = NodeRegistration::new(stub("n")).budget(500_u64.us());
        assert!(reg.is_rt, "budget should auto-set is_rt");
        assert_eq!(
            reg.execution_class,
            ExecutionClass::Rt,
            "budget should set execution_class to Rt"
        );
        assert_eq!(reg.tick_budget, Some(500_u64.us()));
    }

    /// .deadline() auto-sets is_rt=true and execution_class=Rt.
    #[test]
    fn deadline_auto_sets_rt_flag_and_class() {
        let reg = NodeRegistration::new(stub("n")).deadline(10_u64.ms());
        assert!(reg.is_rt, "deadline should auto-set is_rt");
        assert_eq!(reg.execution_class, ExecutionClass::Rt);
    }

    /// .deadline() with microseconds auto-sets is_rt=true.
    #[test]
    fn deadline_us_without_explicit_rt_auto_sets_flag() {
        let reg = NodeRegistration::new(stub("n")).deadline(1000_u64.us());
        assert!(reg.is_rt, "deadline should auto-set is_rt");
    }

    // -- Event + rate combination --

    /// .on("topic").rate(100_u64.hz()) — both set, event class preserved.
    /// Behavior: The rate is recorded but the node is event-triggered.
    /// The scheduler may use rate as a minimum check interval.
    #[test]
    fn conflict_event_with_rate_both_stored() {
        let reg = NodeRegistration::new(stub("n")).on("sensor").rate(100_u64.hz());
        assert_eq!(
            reg.execution_class,
            ExecutionClass::Event("sensor".to_string()),
            "Event class should be preserved"
        );
        assert_eq!(reg.rate_hz, Some(100.0), "rate_hz should still be stored");
    }

    /// .rate(100_u64.hz()).on("topic") — same result, order doesn't matter.
    #[test]
    fn conflict_rate_then_event_both_stored() {
        let reg = NodeRegistration::new(stub("n")).rate(100_u64.hz()).on("sensor");
        assert_eq!(
            reg.execution_class,
            ExecutionClass::Event("sensor".to_string())
        );
        assert_eq!(reg.rate_hz, Some(100.0));
    }

    // -- Deadline + async_io conflict --

    /// .deadline(1_u64.ms()).async_io() — deadline requires RT guarantees but async_io
    /// cannot guarantee deadlines. The deadline is stored but may be meaningless.
    #[test]
    fn conflict_deadline_then_async_io_deadline_stored_class_overridden() {
        let reg = NodeRegistration::new(stub("n")).deadline(1_u64.ms()).async_io();
        assert_eq!(
            reg.execution_class,
            ExecutionClass::AsyncIo,
            "async_io should override execution class"
        );
        assert_eq!(
            reg.deadline,
            Some(1_u64.ms()),
            "Deadline should persist even with async_io"
        );
        assert!(reg.is_rt, "is_rt flag from deadline should persist");
    }

    /// .async_io().deadline(1_u64.ms()) — deadline sets execution_class to Rt (last wins).
    #[test]
    fn conflict_async_io_then_deadline_class_becomes_rt() {
        let reg = NodeRegistration::new(stub("n")).async_io().deadline(1_u64.ms());
        assert!(reg.is_rt);
        assert_eq!(reg.deadline, Some(1_u64.ms()));
        assert_eq!(reg.execution_class, ExecutionClass::Rt);
    }

    // -- budget + compute conflict --

    /// .budget(100_u64.us()).compute() — budget monitoring with compute class.
    /// budget is stored but the compute pool won't enforce RT deadlines.
    #[test]
    fn conflict_budget_then_compute_budget_stored_class_compute() {
        let reg = NodeRegistration::new(stub("n")).budget(100_u64.us()).compute();
        assert_eq!(reg.execution_class, ExecutionClass::Compute);
        assert!(reg.is_rt, "is_rt from budget persists");
        assert_eq!(reg.tick_budget, Some(100_u64.us()));
    }

    // -- Triple class changes --

    /// .budget().compute().async_io() — last class wins, is_rt persists.
    #[test]
    fn conflict_triple_class_change_last_wins() {
        let reg = NodeRegistration::new(stub("n"))
            .budget(100_u64.us())
            .compute()
            .async_io();
        assert_eq!(reg.execution_class, ExecutionClass::AsyncIo);
        assert!(reg.is_rt, "is_rt from .budget() persists through chain");
    }

    /// .on("a").budget(100_u64.us()).compute().on("b") — ends as Event("b").
    #[test]
    fn conflict_four_class_changes_last_wins() {
        let reg = NodeRegistration::new(stub("n"))
            .on("a")
            .budget(100_u64.us())
            .compute()
            .on("b");
        assert_eq!(reg.execution_class, ExecutionClass::Event("b".to_string()));
    }

    #[test]
    fn validate_budget_with_deadline_ok() {
        let mut reg = NodeRegistration::new(stub("n"))
            .budget(200_u64.us())
            .deadline(1_u64.ms());
        reg.validate().unwrap();
    }

    #[test]
    fn validate_budget_alone_ok() {
        // budget auto-sets is_rt, class stays BestEffort — allowed
        let mut reg = NodeRegistration::new(stub("n")).budget(500_u64.us());
        reg.validate().unwrap();
    }

    #[test]
    fn validate_deadline_alone_ok() {
        let mut reg = NodeRegistration::new(stub("n")).deadline(10_u64.ms());
        reg.validate().unwrap();
    }

    #[test]
    fn validate_event_with_rate_ok() {
        // Event + rate_hz is valid (rate used as minimum poll interval)
        let mut reg = NodeRegistration::new(stub("n")).on("sensor").rate(100_u64.hz());
        reg.validate().unwrap();
    }

    #[test]
    fn validate_valid_rate() {
        let mut reg = NodeRegistration::new(stub("n")).rate(100_u64.hz());
        reg.validate().unwrap();
    }

    #[test]
    fn validate_very_small_rate_ok() {
        let mut reg = NodeRegistration::new(stub("n")).rate(0.001.hz());
        reg.validate().unwrap();
    }

    // -- Execution class conflicts: is_rt + non-RT class → error --

    #[test]
    fn validate_budget_then_compute_rejects() {
        let mut reg = NodeRegistration::new(stub("n")).budget(100_u64.us()).compute();
        let err = reg.validate().unwrap_err();
        let msg = err.to_string();
        assert!(
            msg.contains("compute"),
            "error should mention compute: {}",
            msg
        );
    }

    #[test]
    fn validate_budget_then_async_io_rejects() {
        let mut reg = NodeRegistration::new(stub("n")).budget(100_u64.us()).async_io();
        let err = reg.validate().unwrap_err();
        let msg = err.to_string();
        assert!(
            msg.contains("async_io"),
            "error should mention async_io: {}",
            msg
        );
    }

    #[test]
    fn validate_budget_then_event_rejects() {
        let mut reg = NodeRegistration::new(stub("n"))
            .budget(100_u64.us())
            .on("sensor");
        let err = reg.validate().unwrap_err();
        let msg = err.to_string();
        assert!(msg.contains("event"), "error should mention event: {}", msg);
    }

    #[test]
    fn validate_deadline_then_async_io_rejects() {
        let mut reg = NodeRegistration::new(stub("n")).deadline(1_u64.ms()).async_io();
        assert!(
            reg.validate().is_err(),
            "deadline + async_io should conflict"
        );
    }

    #[test]
    fn validate_async_io_then_deadline_ok() {
        // deadline overrides execution_class to Rt (last wins), so no conflict
        let mut reg = NodeRegistration::new(stub("n")).async_io().deadline(1_u64.ms());
        assert!(reg.validate().is_ok(), "deadline overrides async_io to Rt");
    }

    #[test]
    fn validate_triple_conflict_rejects() {
        let mut reg = NodeRegistration::new(stub("n"))
            .budget(100_u64.us())
            .compute()
            .async_io();
        assert!(
            reg.validate().is_err(),
            "budget + compute + async_io should conflict"
        );
    }

    /// .compute().budget() — budget called last, so class=Rt and is_rt=true — no conflict
    #[test]
    fn validate_compute_then_budget_ok() {
        let mut reg = NodeRegistration::new(stub("n")).compute().budget(100_u64.us());
        assert!(
            reg.validate().is_ok(),
            ".compute().budget() should be valid (RT wins)"
        );
    }

    /// .on("topic").budget() — budget called last, class=Rt — no conflict
    #[test]
    fn validate_event_then_budget_ok() {
        let mut reg = NodeRegistration::new(stub("n"))
            .on("sensor")
            .budget(100_u64.us());
        reg.validate().unwrap();
    }

    /// Non-RT class changes without is_rt flag are fine
    #[test]
    fn validate_compute_then_event_ok() {
        let mut reg = NodeRegistration::new(stub("n")).compute().on("topic");
        reg.validate().unwrap();
    }

    #[test]
    fn validate_async_io_then_compute_ok() {
        let mut reg = NodeRegistration::new(stub("n")).async_io().compute();
        reg.validate().unwrap();
    }

    // -- Invalid rate values rejected at Frequency construction --

    #[test]
    #[should_panic(expected = "finite and positive")]
    fn rate_nan_panics() {
        f64::NAN.hz();
    }

    #[test]
    #[should_panic(expected = "finite and positive")]
    fn rate_negative_panics() {
        (-1.0).hz();
    }

    #[test]
    #[should_panic(expected = "positive")]
    fn rate_zero_panics() {
        0_u64.hz();
    }

    #[test]
    #[should_panic(expected = "finite and positive")]
    fn rate_infinity_panics() {
        f64::INFINITY.hz();
    }

    #[test]
    #[should_panic(expected = "finite and positive")]
    fn rate_neg_infinity_panics() {
        f64::NEG_INFINITY.hz();
    }

    // -- Invalid deadlines --

    #[test]
    fn validate_deadline_zero_rejects() {
        let mut reg = NodeRegistration::new(stub("n")).deadline(Duration::ZERO);
        assert!(reg.validate().is_err(), "zero deadline should be rejected");
    }

    #[test]
    fn validate_deadline_positive_ok() {
        let mut reg = NodeRegistration::new(stub("n")).deadline(1_u64.us());
        reg.validate().unwrap();
    }

    // -- Invalid budget --

    #[test]
    fn validate_budget_zero_rejects() {
        let mut reg = NodeRegistration::new(stub("n")).budget(Duration::ZERO);
        assert!(reg.validate().is_err(), "zero budget should be rejected");
    }

    #[test]
    fn validate_budget_positive_ok() {
        let mut reg = NodeRegistration::new(stub("n")).budget(1_u64.us());
        reg.validate().unwrap();
    }

    // -- Builder integration with validation --

    #[test]
    fn validate_builder_conflict_rejects() {
        use crate::scheduling::Scheduler;

        let mut scheduler = Scheduler::new();
        let result = scheduler
            .add(StubNode("conflict".into()))
            .budget(100_u64.us())
            .compute()
            .build();

        assert!(result.is_err(), "conflicting config should fail build()");
    }

    #[test]
    #[should_panic(expected = "finite and positive")]
    fn validate_builder_invalid_rate_panics() {
        // Frequency rejects negative values at construction time
        let _freq = (-1.0).hz();
    }

    #[test]
    fn validate_builder_zero_deadline_rejects() {
        use crate::scheduling::Scheduler;

        let mut scheduler = Scheduler::new();
        let result = scheduler
            .add(StubNode("bad_deadline".into()))
            .deadline(Duration::ZERO)
            .build();

        assert!(result.is_err(), "zero deadline should fail build()");
    }

    // -- Valid full chains still pass --

    #[test]
    fn validate_full_rt_chain_ok() {
        let mut reg = NodeRegistration::new(stub("n"))
            .order(5)
            .rate(100_u64.hz())
            .budget(200_u64.us())
            .deadline(1_u64.ms())
            .failure_policy(FailurePolicy::Fatal);

        reg.validate().unwrap();
    }

    #[test]
    fn validate_builder_valid_config_registers() {
        use crate::scheduling::Scheduler;

        let mut scheduler = Scheduler::new();
        scheduler
            .add(StubNode("valid".into()))
            .order(0)
            .rate(50_u64.hz())
            .build()
            .unwrap();

        assert!(scheduler.node_list().contains(&"valid".to_string()));
    }
}
