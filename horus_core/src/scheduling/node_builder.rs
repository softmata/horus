//! Node Builder - Fluent API for adding nodes to the scheduler
//!
//! The `NodeBuilder` provides a chainable interface for configuring nodes
//! when adding them to the scheduler.
//!
//! # Execution Class Decision Guide
//!
//! Every node runs in exactly one execution class. Choose based on workload:
//!
//! ```text
//! Is your node triggered by incoming data?
//!   YES → .on("topic_name")     (Event — ticks only when topic has new data)
//!   NO  ↓
//! Does your node need guaranteed timing (motor control, sensor sampling)?
//!   YES → .rate(100.hz())       (Rt — dedicated thread, budget enforcement)
//!   NO  ↓
//! Is your node CPU-heavy (path planning, image processing, ML inference)?
//!   YES → .compute()            (Compute — parallel thread pool)
//!   NO  ↓
//! Does your node do blocking I/O (network, file, database)?
//!   YES → .async_io()           (AsyncIo — tokio blocking pool)
//!   NO  → (default)             (BestEffort — main thread, sequential)
//! ```
//!
//! | Class | Thread | Timing | Use Case |
//! |-------|--------|--------|----------|
//! | **Rt** | Dedicated | Budget + deadline enforced | Motor control, sensor sampling |
//! | **Compute** | Thread pool | No RT guarantees | Path planning, CV, ML |
//! | **Event** | Watcher thread | Triggered by topic | Data processors, filters |
//! | **AsyncIo** | Tokio pool | No impact on RT | HTTP calls, logging to disk |
//! | **BestEffort** | Main thread | Sequential | Diagnostics, telemetry |
//!
//! **Rules:**
//! - Only ONE execution class per node. Last call wins (with a warning).
//! - `.rate()` on a BestEffort node auto-promotes to Rt.
//! - `.compute().rate()` stays Compute (rate is informational, not enforced).
//! - Always end with `.build()?` — without it the node is silently dropped.
//!
//! # Example
//!
//! ```rust,ignore
//! use horus::prelude::*;
//!
//! let mut scheduler = Scheduler::new();
//!
//! // Auto-derived budget (80%) and deadline (95%) from rate
//! scheduler.add(sensor)
//!     .order(0)
//!     .rate(100_u64.hz())
//!     .build()?;
//!
//! // Explicit overrides for tighter constraints
//! scheduler.add(motor_ctrl)
//!     .order(1)
//!     .rate(1000_u64.hz())
//!     .budget(300.us())
//!     .deadline(900.us())
//!     .on_miss(Miss::Skip)
//!     .build()?;
//! ```

use super::safety_monitor::BudgetPolicy;
use super::types::{ExecutionClass, NodeKind};
use crate::core::duration_ext::Frequency;
use crate::core::{Miss, Node};
use crate::error::{HorusResult, ValidationError};

/// Trait for types that can be converted into a boxed `Node`.
///
/// Implemented for all concrete `Node` types and for `Box<dyn Node>`.
/// This allows `Scheduler::add()` to accept both:
///
/// ```rust,ignore
/// // Concrete type — most common
/// scheduler.add(MyNode::new()).build()?;
///
/// // Box<dyn Node> — from driver factories
/// scheduler.add(hw.local("conveyor")?).build()?;
/// ```
pub trait IntoNode {
    /// Convert into a boxed node.
    fn into_node(self) -> Box<dyn Node>;
}

impl<N: Node + 'static> IntoNode for N {
    fn into_node(self) -> Box<dyn Node> {
        Box::new(self)
    }
}

impl IntoNode for Box<dyn Node> {
    fn into_node(self) -> Box<dyn Node> {
        self // already boxed, no double-boxing
    }
}
use std::time::Duration;

/// Configuration for a node being added to the scheduler.
///
/// Most users should use `Scheduler::add(node).rate(N.hz()).build()` instead.
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
    /// OS-level thread priority (SCHED_FIFO 1-99). Only for RT nodes.
    pub(crate) os_priority: Option<i32>,
    /// CPU core to pin this node's RT thread to. Only for RT nodes.
    pub(crate) pinned_core: Option<usize>,
    /// Per-node watchdog timeout. Overrides scheduler global.
    pub(crate) node_watchdog: Option<Duration>,
    /// Policy for budget violation enforcement.
    pub(crate) budget_policy: BudgetPolicy,
    /// Opt-in to SCHED_DEADLINE (EDF) instead of SCHED_FIFO.
    pub(crate) use_sched_deadline: bool,
    /// Panic on heap allocation during tick() (RT allocation-free enforcement).
    pub(crate) no_alloc: bool,
    /// Per-subscription freshness watchdogs (populated via subscribe_with_timeout).
    pub(crate) subscription_freshness: Vec<super::types::SubscriptionFreshness>,
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
            os_priority: None,
            pinned_core: None,
            node_watchdog: None,
            budget_policy: BudgetPolicy::default(),
            use_sched_deadline: false,
            no_alloc: false,
            subscription_freshness: Vec::new(),
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
    /// **When to use:** CPU-bound workloads that don't need real-time guarantees.
    /// Examples: path planning, image processing, ML inference, SLAM, IK solvers.
    ///
    /// **Don't use for:** I/O-bound work (use `.async_io()`), real-time control
    /// (use `.rate()`), or data-triggered processing (use `.on()`).
    ///
    /// Compute nodes run in a parallel thread pool, isolated from RT nodes.
    ///
    /// # Example
    /// ```rust,ignore
    /// scheduler.add(path_planner).order(5).compute().rate(10.hz()).build()?;
    /// ```
    pub fn compute(mut self) -> Self {
        self.warn_class_override("compute");
        self.execution_class = ExecutionClass::Compute;
        self
    }

    /// Mark this as an event-triggered node.
    ///
    /// **When to use:** Nodes that process incoming data and should only tick
    /// when there's something to process. Examples: image detector triggered by
    /// camera frames, command processor that acts on new commands, data filters.
    ///
    /// **Don't use for:** Periodic nodes that must tick at a fixed rate regardless
    /// of data availability (use `.rate()`).
    ///
    /// The node will be triggered when data arrives on the specified topic.
    ///
    /// # Example
    /// ```rust,ignore
    /// scheduler.add(detector).order(3).on("camera.rgb").build()?;
    /// ```
    pub fn on(mut self, topic: &str) -> Self {
        self.warn_class_override("on");
        self.execution_class = ExecutionClass::Event(topic.to_string());
        self
    }

    /// Mark this as an async I/O node.
    ///
    /// **When to use:** I/O-bound workloads like network calls, file reads,
    /// database queries, HTTP API calls, logging to disk. Blocking I/O in
    /// these nodes never affects RT jitter or compute throughput.
    ///
    /// **Don't use for:** CPU-bound work (use `.compute()`), or real-time
    /// control (use `.rate()`).
    ///
    /// Runs `tick()` via `tokio::task::spawn_blocking()` on a separate runtime.
    ///
    /// # Example
    /// ```rust,ignore
    /// scheduler.add(telemetry).order(10).async_io().rate(1.hz()).build()?;
    /// ```
    pub fn async_io(mut self) -> Self {
        self.warn_class_override("async_io");
        self.execution_class = ExecutionClass::AsyncIo;
        self
    }

    /// Mark this as a GPU node.
    ///
    /// **When to use:** Nodes that launch CUDA kernels in their tick(). The
    /// scheduler manages a CUDA stream per GPU node. Access the stream inside
    /// tick() via `horus::gpu_stream()`.
    ///
    /// **Don't use for:** CPU-bound work (use `.compute()`), or nodes that
    /// merely read GPU-backed Images (those work fine as BestEffort/Compute).
    ///
    /// `.gpu()` takes precedence over `.rate()` for execution class — a GPU node
    /// with `.rate(30.hz()).gpu()` runs on the GPU executor at 30Hz, not the RT executor.
    ///
    /// # Example
    /// ```rust,ignore
    /// scheduler.add(preprocess_node).rate(30.hz()).gpu().build()?;
    /// ```
    pub fn gpu(mut self) -> Self {
        self.warn_class_override("gpu");
        self.execution_class = ExecutionClass::Gpu;
        self
    }

    /// Emit a warning if the execution class is being overridden.
    fn warn_class_override(&self, new_class: &str) {
        if !matches!(self.execution_class, ExecutionClass::BestEffort) {
            let old_class = match &self.execution_class {
                ExecutionClass::Compute => "compute",
                ExecutionClass::AsyncIo => "async_io",
                ExecutionClass::Event(t) => {
                    log::warn!(
                        "node '{}': .on(\"{}\") overridden by .{}() — only the last execution class applies",
                        self.node.name(), t, new_class
                    );
                    return;
                }
                ExecutionClass::Rt => "rt",
                ExecutionClass::Gpu => "gpu",
                ExecutionClass::BestEffort => unreachable!(),
            };
            log::warn!(
                "node '{}': .{}() overridden by .{}() — only the last execution class applies",
                self.node.name(),
                old_class,
                new_class
            );
        }
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

    /// Set an explicit tick budget — the maximum time a single tick should take.
    ///
    /// Overrides the auto-derived budget (80% of period from `.rate()`).
    /// If called without `.rate()`, this implicitly enables RT scheduling.
    ///
    /// # Example
    /// ```rust,ignore
    /// use horus::prelude::*;
    ///
    /// // Override default: motor loop must finish within 300μs
    /// NodeRegistration::new(Box::new(motor_ctrl))
    ///     .rate(1000_u64.hz())
    ///     .budget(300.us())
    /// ```
    pub fn budget(mut self, budget: Duration) -> Self {
        self.tick_budget = Some(budget);
        self
    }

    /// Set an explicit deadline — the absolute latest a tick can finish.
    ///
    /// Overrides the auto-derived deadline (95% of period from `.rate()`).
    /// If called without `.rate()`, this implicitly enables RT scheduling.
    ///
    /// # Example
    /// ```rust,ignore
    /// use horus::prelude::*;
    ///
    /// // Override default: must complete within 900μs of a 1ms period
    /// NodeRegistration::new(Box::new(motor_ctrl))
    ///     .rate(1000_u64.hz())
    ///     .deadline(900.us())
    /// ```
    pub fn deadline(mut self, deadline: Duration) -> Self {
        self.deadline = Some(deadline);
        self
    }

    /// Set the budget violation enforcement policy.
    ///
    /// - `Warn`: log only (default)
    /// - `Enforce`: stop node after 2x budget overrun
    /// - `EmergencyStop`: trigger e-stop on any budget violation
    pub fn budget_policy(mut self, policy: BudgetPolicy) -> Self {
        self.budget_policy = policy;
        self
    }

    /// Opt in to SCHED_DEADLINE (Linux EDF scheduler) instead of SCHED_FIFO.
    ///
    /// Requires `.rate()` and `.budget()` to derive kernel parameters.
    /// Falls back to SCHED_FIFO if unavailable.
    pub fn deadline_scheduler(mut self) -> Self {
        self.use_sched_deadline = true;
        self
    }

    /// Enforce zero heap allocations during `tick()`.
    ///
    /// When set, any `Vec::push()`, `String::from()`, `format!()`, `Box::new()`,
    /// or other heap allocation during `tick()` panics with a clear message.
    ///
    /// Requires `RtAwareAllocator` as `#[global_allocator]` in the binary.
    /// Without it, this flag is a no-op (safe for prototyping).
    ///
    /// # Example
    /// ```rust,ignore
    /// scheduler.add(motor_ctrl)
    ///     .rate(1000_u64.hz())
    ///     .no_alloc()  // panic if tick() allocates
    ///     .build()?;
    /// ```
    pub fn no_alloc(mut self) -> Self {
        self.no_alloc = true;
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
    ///     .on_miss(Miss::SafeMode)
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

    /// Set the OS-level thread priority for this node's RT thread.
    ///
    /// Only meaningful for RT nodes. A warning is logged at `.build()` time
    /// if used on a non-RT node (the value is accepted but ignored at runtime).
    /// Uses `SCHED_FIFO` (1-99, higher = more priority).
    /// Requires `CAP_SYS_NICE` or root. Degrades gracefully if unavailable.
    ///
    /// # Example
    /// ```rust,ignore
    /// scheduler.add(safety_monitor)
    ///     .rate(1000_u64.hz())
    ///     .priority(99)    // highest RT priority
    ///     .build()?;
    /// ```
    pub fn priority(mut self, prio: i32) -> Self {
        self.os_priority = Some(prio);
        self
    }

    /// Pin this node's RT thread to a specific CPU core.
    ///
    /// Only meaningful for RT nodes. A warning is logged at `.build()` time
    /// if used on a non-RT node (the value is accepted but ignored at runtime).
    /// Uses `sched_setaffinity`.
    /// Degrades gracefully if the core doesn't exist.
    ///
    /// # Example
    /// ```rust,ignore
    /// scheduler.add(motor_ctrl)
    ///     .rate(1000_u64.hz())
    ///     .core(2)         // pin to core 2
    ///     .build()?;
    /// ```
    pub fn core(mut self, cpu_id: usize) -> Self {
        self.pinned_core = Some(cpu_id);
        self
    }

    /// Set a per-node watchdog timeout.
    ///
    /// Overrides the scheduler's global `.watchdog()` timeout for this node.
    /// Safety-critical nodes can have tighter timeouts than logging nodes.
    ///
    /// # Example
    /// ```rust,ignore
    /// scheduler.add(emergency_stop)
    ///     .rate(1000_u64.hz())
    ///     .watchdog(2_u64.ms())    // 2ms watchdog
    ///     .build()?;
    ///
    /// scheduler.add(logger)
    ///     .async_io()
    ///     .watchdog(5_u64.secs())  // 5s tolerance
    ///     .build()?;
    /// ```
    pub fn watchdog(mut self, timeout: Duration) -> Self {
        self.node_watchdog = Some(timeout);
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
        let has_explicit_budget = self.tick_budget.is_some();
        let has_explicit_deadline = self.deadline.is_some();

        if let Some(freq) = self.source_freq {
            // Auto-derive budget/deadline if RT or will become RT
            let will_be_rt =
                self.is_rt || matches!(self.execution_class, ExecutionClass::BestEffort);
            if will_be_rt {
                // Only auto-derive if user didn't set explicitly
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

        // Auto-derive deadline from budget when deadline is not set.
        //
        // Robotics developers expect `.budget(500.us()).on_miss(Miss::Stop)` to
        // trigger the Stop policy when the budget is exceeded. Without this rule,
        // on_miss only fires on DEADLINE misses, so setting budget without deadline
        // means on_miss silently never fires. By deriving deadline = budget, the
        // user's budget IS their hard deadline — matching their mental model.
        //
        // If the user wants slack between budget and deadline (e.g., budget 500μs,
        // deadline 900μs), they set both explicitly.
        if self.tick_budget.is_some() && self.deadline.is_none() {
            self.deadline = self.tick_budget;
        }

        // Explicit budget/deadline without .rate() implies RT
        if (has_explicit_budget || has_explicit_deadline)
            && matches!(self.execution_class, ExecutionClass::BestEffort)
        {
            self.is_rt = true;
            self.execution_class = ExecutionClass::Rt;
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

        // Validate event topic name: must not be empty
        if let ExecutionClass::Event(ref topic) = self.execution_class {
            if topic.is_empty() {
                return Err(ValidationError::InvalidValue {
                    field: "on(topic)".into(),
                    value: "\"\"".into(),
                    reason: format!(
                        "node '{}': event topic name must not be empty — \
                         .on(\"\") creates a node that can never trigger.",
                        node_name
                    ),
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
                field_a: "is_rt / .rate()".into(),
                field_b: format!(".{}()", class_name),
                reason: format!(
                    "node '{}' is marked as RT but has execution class '{}' — \
                     these are contradictory. Use either RT scheduling or {} scheduling, not both.",
                    node_name, class_name, class_name
                ),
            }
            .into());
        }

        // Validate budget/deadline on non-RT execution classes
        // (finalize() auto-enables RT for BestEffort, but compute/event/async_io
        //  nodes with explicit budget/deadline are contradictory)
        if !self.is_rt && (self.tick_budget.is_some() || self.deadline.is_some()) {
            return Err(ValidationError::Conflict {
                field_a: "budget / deadline".into(),
                field_b: format!("execution class {:?}", self.execution_class),
                reason: format!(
                    "node '{}' has budget/deadline set but uses a non-RT execution class. \
                     Budget/deadline are only meaningful for RT nodes. Either remove \
                     .compute()/.async_io()/.on() or remove .budget()/.deadline().",
                    node_name
                ),
            }
            .into());
        }

        // Warn about RT-only settings on non-RT nodes
        if !self.is_rt {
            if let Some(prio) = self.os_priority {
                log::warn!(
                    "node '{}': .priority({}) has no effect — only RT nodes get SCHED_FIFO threads. \
                     Add .rate() or .budget() to make this node RT, or remove .priority().",
                    node_name, prio
                );
            }
            if let Some(cpu) = self.pinned_core {
                log::warn!(
                    "node '{}': .core({}) has no effect — only RT nodes get pinned threads. \
                     Add .rate() or .budget() to make this node RT, or remove .core().",
                    node_name,
                    cpu
                );
            }
        }

        // Warn about miss policy without a deadline
        if self.miss_policy != Miss::Warn && self.deadline.is_none() {
            log::warn!(
                "node '{}': .on_miss({:?}) has no effect without a deadline — \
                 add .rate() or .deadline() to enable deadline enforcement, \
                 or remove .on_miss().",
                node_name,
                self.miss_policy
            );
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
    /// Use `.budget()` and `.deadline()` to override the defaults.
    pub fn rate(mut self, freq: Frequency) -> Self {
        self.config = self.config.rate(freq);
        self
    }

    /// Set an explicit tick budget — the maximum time a single tick should take.
    ///
    /// Overrides the auto-derived budget (80% of period from `.rate()`).
    /// If called without `.rate()`, this implicitly enables RT scheduling.
    ///
    /// ```rust,ignore
    /// scheduler.add(motor_ctrl)
    ///     .rate(1000_u64.hz())
    ///     .budget(300.us())      // override default 800μs
    ///     .build()?;
    /// ```
    pub fn budget(mut self, budget: Duration) -> Self {
        self.config = self.config.budget(budget);
        self
    }

    /// Set an explicit deadline — the absolute latest a tick can finish.
    ///
    /// Overrides the auto-derived deadline (95% of period from `.rate()`).
    /// If called without `.rate()`, this implicitly enables RT scheduling.
    ///
    /// ```rust,ignore
    /// scheduler.add(motor_ctrl)
    ///     .rate(1000_u64.hz())
    ///     .deadline(900.us())    // override default 950μs
    ///     .build()?;
    /// ```
    pub fn deadline(mut self, deadline: Duration) -> Self {
        self.config = self.config.deadline(deadline);
        self
    }

    /// Set the budget violation enforcement policy.
    ///
    /// - `Warn`: log only (default)
    /// - `Enforce`: stop node after 2x budget overrun
    /// - `EmergencyStop`: trigger e-stop on any budget violation
    pub fn budget_policy(mut self, policy: BudgetPolicy) -> Self {
        self.config = self.config.budget_policy(policy);
        self
    }

    /// Set the deadline miss policy.
    pub fn on_miss(mut self, policy: Miss) -> Self {
        self.config = self.config.on_miss(policy);
        self
    }

    /// Opt in to SCHED_DEADLINE (Linux EDF scheduler).
    pub fn deadline_scheduler(mut self) -> Self {
        self.config = self.config.deadline_scheduler();
        self
    }

    /// Enforce zero heap allocations during tick().
    pub fn no_alloc(mut self) -> Self {
        self.config = self.config.no_alloc();
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

    /// Mark this as a GPU node (runs on GPU executor with CUDA stream).
    pub fn gpu(mut self) -> Self {
        self.config = self.config.gpu();
        self
    }

    /// Override the failure policy for this node.
    ///
    /// Use this when you need different failure behavior than the default.
    pub fn failure_policy(mut self, policy: super::fault_tolerance::FailurePolicy) -> Self {
        self.config = self.config.failure_policy(policy);
        self
    }

    /// Set the OS-level thread priority (SCHED_FIFO 1-99) for this node's RT thread.
    /// Warned at `.build()` if used on a non-RT node.
    pub fn priority(mut self, prio: i32) -> Self {
        self.config = self.config.priority(prio);
        self
    }

    /// Pin this node's RT thread to a specific CPU core.
    /// Warned at `.build()` if used on a non-RT node.
    pub fn core(mut self, cpu_id: usize) -> Self {
        self.config = self.config.core(cpu_id);
        self
    }

    /// Add a subscription freshness watchdog for the given topic.
    ///
    /// If the topic receives no new messages for `timeout`, the specified
    /// `policy` is applied:
    /// - `StalePolicy::Warn` — log warning, continue ticking
    /// - `StalePolicy::SafeState` — call `enter_safe_state()` on the node
    /// - `StalePolicy::Stop` — stop the node permanently
    ///
    /// This catches dead remote sensors, network partitions, and abandoned topics.
    ///
    /// ```rust,ignore
    /// scheduler.add(controller)
    ///     .subscribe_with_timeout("scan", 200.ms(), StalePolicy::Warn)
    ///     .subscribe_with_timeout("cmd_vel", 100.ms(), StalePolicy::SafeState)
    ///     .build()?;
    /// ```
    pub fn subscribe_with_timeout(
        mut self,
        topic: &str,
        timeout: Duration,
        policy: super::types::StalePolicy,
    ) -> Self {
        self.config
            .subscription_freshness
            .push(super::types::SubscriptionFreshness {
                topic: topic.to_string(),
                timeout,
                policy,
                last_received_ns: std::sync::atomic::AtomicU64::new(
                    std::time::SystemTime::now()
                        .duration_since(std::time::UNIX_EPOCH)
                        .unwrap_or_default()
                        .as_nanos() as u64,
                ),
            });
        self
    }

    /// Set a per-node watchdog timeout, overriding the scheduler global.
    pub fn watchdog(mut self, timeout: Duration) -> Self {
        self.config = self.config.watchdog(timeout);
        self
    }

    /// Finish configuration and add the node to the scheduler.
    ///
    /// Validates the configuration before registering. Returns an error if:
    /// - `.budget()` or `.deadline()` is used with `.compute()`, `.on()`, or `.async_io()`
    ///   (budget/deadline are only meaningful for RT nodes)
    /// - `.budget(Duration::ZERO)` or `.deadline(Duration::ZERO)` (must be > 0)
    /// - `.on("")` with an empty topic name
    /// - Rate is not finite or positive
    ///
    /// **Valid combinations that may look surprising:**
    /// - `.rate(N.hz()).compute()` — rate-limited Compute node (NOT RT)
    /// - `.rate(N.hz()).async_io()` — rate-limited AsyncIo node (NOT RT)
    /// - `.rate(N.hz()).on("topic")` — Event node with rate as poll interval hint
    ///
    /// If multiple execution classes are chained (e.g. `.compute().async_io()`),
    /// only the **last** one applies (a warning is logged).
    ///
    /// # Errors
    ///
    /// - [`ValidationError::Conflict`] — conflicting execution class (e.g., `.compute()` + `.on()`)
    /// - [`ValidationError::InvalidValue`] — rate is zero, negative, NaN, or infinite
    /// - [`ConfigError::ValidationFailed`] — budget exceeds deadline, or other constraint violations
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
    use crate::scheduling::scheduler::Scheduler;

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
    fn test_on_empty_topic_rejected() {
        let mut reg = NodeRegistration::new(stub("n")).on("");
        let result = reg.validate();
        assert!(result.is_err(), "empty topic name should be rejected");
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

    // ── .rate() auto-derives budget & deadline ──

    #[test]
    fn test_rate_auto_derives_budget_and_deadline() {
        let mut reg = NodeRegistration::new(stub("n")).rate(100_u64.hz());
        reg.finalize();
        assert!(reg.is_rt);
        assert!(
            reg.tick_budget.is_some(),
            "budget should be auto-derived from rate"
        );
        assert!(
            reg.deadline.is_some(),
            "deadline should be auto-derived from rate"
        );
    }

    // ── .budget() / .deadline() explicit override ──

    #[test]
    fn test_explicit_budget_overrides_auto_derived() {
        let mut reg = NodeRegistration::new(stub("n"))
            .rate(1000_u64.hz())
            .budget(300.us());
        reg.finalize();
        assert!(reg.is_rt);
        assert_eq!(reg.tick_budget, Some(300.us()));
        // deadline should still be auto-derived (95% of 1ms = 950μs)
        assert!(reg.deadline.is_some());
        assert_ne!(reg.deadline, Some(300.us()));
    }

    #[test]
    fn test_explicit_deadline_overrides_auto_derived() {
        let mut reg = NodeRegistration::new(stub("n"))
            .rate(1000_u64.hz())
            .deadline(900.us());
        reg.finalize();
        assert!(reg.is_rt);
        assert_eq!(reg.deadline, Some(900.us()));
        // budget should still be auto-derived (80% of 1ms = 800μs)
        assert!(reg.tick_budget.is_some());
    }

    #[test]
    fn test_explicit_both_budget_and_deadline() {
        let mut reg = NodeRegistration::new(stub("n"))
            .rate(1000_u64.hz())
            .budget(300.us())
            .deadline(900.us());
        reg.finalize();
        assert!(reg.is_rt);
        assert_eq!(reg.tick_budget, Some(300.us()));
        assert_eq!(reg.deadline, Some(900.us()));
    }

    #[test]
    fn test_budget_without_rate_enables_rt() {
        let mut reg = NodeRegistration::new(stub("n")).budget(500.us());
        reg.finalize();
        assert!(reg.is_rt);
        assert_eq!(reg.execution_class, ExecutionClass::Rt);
        assert_eq!(reg.tick_budget, Some(500.us()));
    }

    #[test]
    fn test_deadline_without_rate_enables_rt() {
        let mut reg = NodeRegistration::new(stub("n")).deadline(1.ms());
        reg.finalize();
        assert!(reg.is_rt);
        assert_eq!(reg.execution_class, ExecutionClass::Rt);
        assert_eq!(reg.deadline, Some(1.ms()));
    }

    #[test]
    fn test_budget_on_compute_node_rejected() {
        let mut reg = NodeRegistration::new(stub("n")).compute().budget(500.us());
        let result = reg.validate();
        assert!(result.is_err(), "budget on compute node should be rejected");
    }

    #[test]
    fn test_order_independent_budget_rate() {
        // .budget() before .rate() — budget should still override
        let mut reg = NodeRegistration::new(stub("n"))
            .budget(300.us())
            .rate(1000_u64.hz());
        reg.finalize();
        assert_eq!(reg.tick_budget, Some(300.us()));
    }

    // ── .failure_policy() ──

    #[test]
    fn test_failure_policy_fatal() {
        let reg = NodeRegistration::new(stub("n")).failure_policy(FailurePolicy::Fatal);
        assert!(matches!(reg.failure_policy, Some(FailurePolicy::Fatal)));
    }

    #[test]
    fn test_failure_policy_restart() {
        let reg =
            NodeRegistration::new(stub("n")).failure_policy(FailurePolicy::restart(3, 50_u64.ms()));
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
        let reg =
            NodeRegistration::new(stub("n")).failure_policy(FailurePolicy::skip(5, 200_u64.ms()));
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
    fn test_async_io_then_on_takes_last() {
        let reg = NodeRegistration::new(stub("n")).async_io().on("topic");
        assert_eq!(
            reg.execution_class,
            ExecutionClass::Event("topic".to_string())
        );
    }

    // ── Builder chain ordering independence ──

    #[test]
    fn test_order_rate_chain() {
        let mut reg = NodeRegistration::new(stub("n"))
            .order(5)
            .rate(100_u64.hz())
            .failure_policy(FailurePolicy::Fatal);

        reg.finalize();
        assert_eq!(reg.order, 5);
        assert_eq!(reg.rate_hz, Some(100.0));
        assert!(reg.is_rt);
        assert!(reg.tick_budget.is_some());
        assert!(reg.deadline.is_some());
        assert!(matches!(reg.failure_policy, Some(FailurePolicy::Fatal)));
    }

    #[test]
    fn test_reverse_chain_order() {
        let mut reg = NodeRegistration::new(stub("n"))
            .failure_policy(FailurePolicy::Ignore)
            .rate(50_u64.hz())
            .order(10);

        reg.finalize();
        assert_eq!(reg.order, 10);
        assert_eq!(reg.rate_hz, Some(50.0));
        assert!(reg.is_rt);
        assert!(reg.tick_budget.is_some());
        assert!(reg.deadline.is_some());
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
            .rate(1000_u64.hz())
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
            .rate(1000_u64.hz())
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
    fn validate_rate_only_ok() {
        let mut reg = NodeRegistration::new(stub("n")).rate(100_u64.hz());
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

    // -- .rate() auto-sets RT --

    /// .rate() auto-sets is_rt=true AND execution_class=Rt after finalize.
    #[test]
    fn rate_auto_sets_rt_flag_and_class() {
        let mut reg = NodeRegistration::new(stub("n")).rate(100_u64.hz());
        reg.finalize();
        assert!(reg.is_rt, "rate should auto-set is_rt");
        assert_eq!(
            reg.execution_class,
            ExecutionClass::Rt,
            "rate should set execution_class to Rt"
        );
        assert!(reg.tick_budget.is_some());
        assert!(reg.deadline.is_some());
    }

    // -- Event + rate combination --

    /// .on("topic").rate(100_u64.hz()) — both set, event class preserved.
    /// Behavior: The rate is recorded but the node is event-triggered.
    /// The scheduler may use rate as a minimum check interval.
    #[test]
    fn conflict_event_with_rate_both_stored() {
        let reg = NodeRegistration::new(stub("n"))
            .on("sensor")
            .rate(100_u64.hz());
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
        let reg = NodeRegistration::new(stub("n"))
            .rate(100_u64.hz())
            .on("sensor");
        assert_eq!(
            reg.execution_class,
            ExecutionClass::Event("sensor".to_string())
        );
        assert_eq!(reg.rate_hz, Some(100.0));
    }

    // -- .rate() + non-RT class conflicts --

    /// .rate().compute() — rate auto-derives RT, but compute overrides class after finalize.
    /// After finalize, is_rt is true but class is Compute — validation rejects this.
    #[test]
    fn conflict_rate_then_compute_finalize_rejects() {
        let mut reg = NodeRegistration::new(stub("n"))
            .rate(100_u64.hz())
            .compute();
        // compute() overrides execution_class before finalize — finalize sees Compute, skips RT
        reg.finalize();
        assert_eq!(reg.execution_class, ExecutionClass::Compute);
        assert!(!reg.is_rt, "compute prevents RT auto-derivation");
    }

    #[test]
    fn validate_event_with_rate_ok() {
        // Event + rate_hz is valid (rate used as minimum poll interval)
        let mut reg = NodeRegistration::new(stub("n"))
            .on("sensor")
            .rate(100_u64.hz());
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

    // -- Builder integration with validation --

    #[test]
    #[should_panic(expected = "finite and positive")]
    fn validate_builder_invalid_rate_panics() {
        // Frequency rejects negative values at construction time
        let _freq = (-1.0).hz();
    }

    // -- Valid full chains still pass --

    #[test]
    fn validate_full_rt_chain_ok() {
        let mut reg = NodeRegistration::new(stub("n"))
            .order(5)
            .rate(100_u64.hz())
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

    // ============================================================================
    // Validation warnings and new rejections
    // ============================================================================

    // -- .priority() on non-RT nodes (warning only, not error) --

    #[test]
    fn priority_on_compute_builds_ok() {
        let mut reg = NodeRegistration::new(stub("n")).compute().priority(99);
        reg.validate().unwrap(); // Should succeed with warning
    }

    #[test]
    fn priority_on_async_io_builds_ok() {
        let mut reg = NodeRegistration::new(stub("n")).async_io().priority(50);
        reg.validate().unwrap();
    }

    #[test]
    fn priority_on_best_effort_builds_ok() {
        let mut reg = NodeRegistration::new(stub("n")).priority(80);
        reg.validate().unwrap();
    }

    #[test]
    fn priority_on_rt_builds_ok_no_warning() {
        let mut reg = NodeRegistration::new(stub("n"))
            .rate(100_u64.hz())
            .priority(90);
        reg.validate().unwrap(); // RT node — no warning expected
    }

    // -- .core() on non-RT nodes (warning only, not error) --

    #[test]
    fn core_on_compute_builds_ok() {
        let mut reg = NodeRegistration::new(stub("n")).compute().core(2);
        reg.validate().unwrap();
    }

    #[test]
    fn core_on_event_builds_ok() {
        let mut reg = NodeRegistration::new(stub("n")).on("topic").core(0);
        reg.validate().unwrap();
    }

    #[test]
    fn core_on_rt_builds_ok_no_warning() {
        let mut reg = NodeRegistration::new(stub("n")).rate(500_u64.hz()).core(3);
        reg.validate().unwrap(); // RT node — no warning expected
    }

    // -- .on_miss() without deadline (warning only, not error) --

    #[test]
    fn on_miss_stop_without_deadline_builds_ok() {
        let mut reg = NodeRegistration::new(stub("n"))
            .compute()
            .on_miss(Miss::Stop);
        reg.validate().unwrap(); // Warning only
    }

    #[test]
    fn on_miss_safe_mode_without_deadline_builds_ok() {
        let mut reg = NodeRegistration::new(stub("n")).on_miss(Miss::SafeMode);
        reg.validate().unwrap();
    }

    #[test]
    fn on_miss_warn_without_deadline_no_warning() {
        // Miss::Warn is default — should not warn even without deadline
        let mut reg = NodeRegistration::new(stub("n")).on_miss(Miss::Warn);
        reg.validate().unwrap();
    }

    #[test]
    fn on_miss_skip_with_rate_no_warning() {
        // .rate() auto-derives deadline — on_miss is valid
        let mut reg = NodeRegistration::new(stub("n"))
            .rate(100_u64.hz())
            .on_miss(Miss::Skip);
        reg.validate().unwrap();
    }

    #[test]
    fn on_miss_with_explicit_deadline_no_warning() {
        let mut reg = NodeRegistration::new(stub("n"))
            .deadline(1.ms())
            .on_miss(Miss::SafeMode);
        reg.validate().unwrap();
    }

    // -- .on("") rejection --

    #[test]
    fn on_empty_topic_rejected_with_message() {
        let mut reg = NodeRegistration::new(stub("n")).on("");
        let err = reg.validate().unwrap_err();
        let msg = format!("{}", err);
        assert!(
            msg.contains("must not be empty"),
            "error should mention empty topic: {}",
            msg
        );
    }

    #[test]
    fn on_whitespace_topic_accepted() {
        // Non-empty whitespace topics are technically valid
        let mut reg = NodeRegistration::new(stub("n")).on(" ");
        reg.validate().unwrap();
    }

    // ── SLAM-generated: Property-based tests ──────────────────────

    mod proptests {
        use super::*;
        use crate::scheduling::Scheduler;
        use proptest::prelude::*;

        proptest! {
            /// Any valid order value produces a scheduler-accepted node.
            #[test]
            fn any_order_accepted_by_scheduler(order in 0u32..1000) {
                let mut scheduler = Scheduler::new();
                scheduler
                    .add(StubNode(format!("order_{}", order)))
                    .order(order)
                    .build()
                    .unwrap();
            }

            /// Any positive frequency via .rate() produces a buildable node.
            #[test]
            fn any_positive_rate_builds(hz in 1u64..10000) {
                let mut scheduler = Scheduler::new();
                scheduler
                    .add(StubNode(format!("rate_{}", hz)))
                    .order(0)
                    .rate(hz.hz())
                    .build()
                    .unwrap();
            }

            /// Compute nodes always build successfully.
            #[test]
            fn compute_always_builds(order in 0u32..100) {
                let mut scheduler = Scheduler::new();
                scheduler
                    .add(StubNode(format!("compute_{}", order)))
                    .order(order)
                    .compute()
                    .build()
                    .unwrap();
            }
        }
    }

    // ========================================================================
    // Execution class determinism tests — order independence
    // ========================================================================

    #[test]
    fn test_rate_only_produces_rt() {
        let mut scheduler = Scheduler::new();
        scheduler
            .add(StubNode("rate_only".to_string()))
            .rate(100.hz())
            .build()
            .unwrap();
        let node = &scheduler.nodes[0];
        assert!(node.is_rt_node, "rate alone should auto-detect as RT");
    }

    #[test]
    fn test_budget_only_produces_rt() {
        let mut scheduler = Scheduler::new();
        scheduler
            .add(StubNode("budget_only".to_string()))
            .budget(200.us())
            .build()
            .unwrap();
        let node = &scheduler.nodes[0];
        assert!(node.is_rt_node, "budget alone should auto-detect as RT");
    }

    #[test]
    fn test_deadline_produces_rt() {
        let mut scheduler = Scheduler::new();
        scheduler
            .add(StubNode("deadline_only".to_string()))
            .deadline(1.ms())
            .build()
            .unwrap();
        let node = &scheduler.nodes[0];
        // Deadline implies RT — the node has timing requirements
        assert!(node.is_rt_node, "deadline should auto-detect as RT");
        assert!(node.deadline.is_some());
    }

    #[test]
    fn test_compute_overrides_rate() {
        let mut scheduler = Scheduler::new();
        scheduler
            .add(StubNode("compute_with_rate".to_string()))
            .compute()
            .rate(100.hz())
            .build()
            .unwrap();
        let node = &scheduler.nodes[0];
        assert_eq!(
            node.execution_class,
            crate::scheduling::types::ExecutionClass::Compute,
            "explicit compute() should override rate's auto-RT"
        );
    }

    #[test]
    fn test_async_io_is_async() {
        let mut scheduler = Scheduler::new();
        scheduler
            .add(StubNode("async_node".to_string()))
            .async_io()
            .build()
            .unwrap();
        let node = &scheduler.nodes[0];
        assert_eq!(
            node.execution_class,
            crate::scheduling::types::ExecutionClass::AsyncIo
        );
    }

    #[test]
    fn test_no_options_is_best_effort() {
        let mut scheduler = Scheduler::new();
        scheduler
            .add(StubNode("bare_node".to_string()))
            .build()
            .unwrap();
        let node = &scheduler.nodes[0];
        assert_eq!(
            node.execution_class,
            crate::scheduling::types::ExecutionClass::BestEffort
        );
        assert!(!node.is_rt_node);
    }

    #[test]
    fn test_order_preserved() {
        let mut scheduler = Scheduler::new();
        scheduler
            .add(StubNode("z_last".to_string()))
            .order(99)
            .build()
            .unwrap();
        scheduler
            .add(StubNode("a_first".to_string()))
            .order(0)
            .build()
            .unwrap();
        scheduler
            .add(StubNode("m_middle".to_string()))
            .order(50)
            .build()
            .unwrap();

        // Priority/order should match what was set
        assert_eq!(scheduler.nodes[0].priority, 99);
        assert_eq!(scheduler.nodes[1].priority, 0);
        assert_eq!(scheduler.nodes[2].priority, 50);
    }

    // ============================================================================
    // Edge case tests — builder method ordering, double-set, conflicts, boundaries
    // ============================================================================

    // ── Method order independence: rate vs compute ──

    #[test]
    fn rate_before_compute_stays_compute() {
        let mut reg_a = NodeRegistration::new(stub("a"))
            .rate(100_u64.hz())
            .compute();
        reg_a.finalize();

        let mut reg_b = NodeRegistration::new(stub("b"))
            .compute()
            .rate(100_u64.hz());
        reg_b.finalize();

        // Both should end up as Compute, not RT
        assert_eq!(reg_a.execution_class, ExecutionClass::Compute);
        assert_eq!(reg_b.execution_class, ExecutionClass::Compute);
        assert!(!reg_a.is_rt);
        assert!(!reg_b.is_rt);
        // Both should have rate stored
        assert_eq!(reg_a.rate_hz, Some(100.0));
        assert_eq!(reg_b.rate_hz, Some(100.0));
    }

    #[test]
    fn rate_before_async_io_stays_async_io() {
        let mut reg_a = NodeRegistration::new(stub("a"))
            .rate(50_u64.hz())
            .async_io();
        reg_a.finalize();

        let mut reg_b = NodeRegistration::new(stub("b"))
            .async_io()
            .rate(50_u64.hz());
        reg_b.finalize();

        assert_eq!(reg_a.execution_class, ExecutionClass::AsyncIo);
        assert_eq!(reg_b.execution_class, ExecutionClass::AsyncIo);
        assert!(!reg_a.is_rt);
        assert!(!reg_b.is_rt);
    }

    #[test]
    fn deadline_before_rate_keeps_explicit_deadline() {
        let mut reg = NodeRegistration::new(stub("n"))
            .deadline(500.us())
            .rate(1000_u64.hz());
        reg.finalize();
        // Explicit deadline should be preserved, not overwritten by rate
        assert_eq!(reg.deadline, Some(500.us()));
        assert!(reg.is_rt);
    }

    #[test]
    fn budget_deadline_rate_any_order_same_result() {
        let mut reg_a = NodeRegistration::new(stub("a"))
            .rate(1000_u64.hz())
            .budget(300.us())
            .deadline(800.us());
        reg_a.finalize();

        let mut reg_b = NodeRegistration::new(stub("b"))
            .budget(300.us())
            .deadline(800.us())
            .rate(1000_u64.hz());
        reg_b.finalize();

        let mut reg_c = NodeRegistration::new(stub("c"))
            .deadline(800.us())
            .rate(1000_u64.hz())
            .budget(300.us());
        reg_c.finalize();

        // All three orderings should produce identical config
        assert_eq!(reg_a.tick_budget, reg_b.tick_budget);
        assert_eq!(reg_b.tick_budget, reg_c.tick_budget);
        assert_eq!(reg_a.deadline, reg_b.deadline);
        assert_eq!(reg_b.deadline, reg_c.deadline);
        assert_eq!(reg_a.tick_budget, Some(300.us()));
        assert_eq!(reg_a.deadline, Some(800.us()));
    }

    // ── Double-set scenarios ──

    #[test]
    fn rate_set_twice_last_wins() {
        let mut reg = NodeRegistration::new(stub("n"))
            .rate(100_u64.hz())
            .rate(200_u64.hz());
        reg.finalize();
        assert_eq!(reg.rate_hz, Some(200.0));
        // Budget/deadline should be derived from the second rate (200Hz)
        let expected_budget = 200_u64.hz().budget_default();
        let expected_deadline = 200_u64.hz().deadline_default();
        assert_eq!(reg.tick_budget, Some(expected_budget));
        assert_eq!(reg.deadline, Some(expected_deadline));
    }

    #[test]
    fn budget_set_twice_last_wins() {
        let mut reg = NodeRegistration::new(stub("n"))
            .rate(1000_u64.hz())
            .budget(300.us())
            .budget(500.us());
        reg.finalize();
        assert_eq!(reg.tick_budget, Some(500.us()));
    }

    #[test]
    fn deadline_set_twice_last_wins() {
        let mut reg = NodeRegistration::new(stub("n"))
            .rate(1000_u64.hz())
            .deadline(800.us())
            .deadline(900.us());
        reg.finalize();
        assert_eq!(reg.deadline, Some(900.us()));
    }

    #[test]
    fn order_set_twice_last_wins() {
        let reg = NodeRegistration::new(stub("n")).order(5).order(42);
        assert_eq!(reg.order, 42);
    }

    #[test]
    fn on_miss_set_twice_last_wins() {
        let reg = NodeRegistration::new(stub("n"))
            .on_miss(Miss::Skip)
            .on_miss(Miss::Stop);
        assert_eq!(reg.miss_policy, Miss::Stop);
    }

    #[test]
    fn failure_policy_set_twice_last_wins() {
        let reg = NodeRegistration::new(stub("n"))
            .failure_policy(FailurePolicy::Fatal)
            .failure_policy(FailurePolicy::Ignore);
        assert!(matches!(reg.failure_policy, Some(FailurePolicy::Ignore)));
    }

    // ── Conflicting settings ──

    #[test]
    fn async_io_with_budget_rejected() {
        let mut reg = NodeRegistration::new(stub("n")).async_io().budget(500.us());
        let result = reg.validate();
        assert!(
            result.is_err(),
            "budget on async_io node should be rejected"
        );
    }

    #[test]
    fn async_io_with_deadline_rejected() {
        let mut reg = NodeRegistration::new(stub("n")).async_io().deadline(1.ms());
        let result = reg.validate();
        assert!(
            result.is_err(),
            "deadline on async_io node should be rejected"
        );
    }

    #[test]
    fn event_with_budget_rejected() {
        let mut reg = NodeRegistration::new(stub("n"))
            .on("topic")
            .budget(500.us());
        let result = reg.validate();
        assert!(result.is_err(), "budget on event node should be rejected");
    }

    #[test]
    fn event_with_deadline_rejected() {
        let mut reg = NodeRegistration::new(stub("n"))
            .on("topic")
            .deadline(1.ms());
        let result = reg.validate();
        assert!(result.is_err(), "deadline on event node should be rejected");
    }

    #[test]
    fn compute_with_deadline_rejected() {
        let mut reg = NodeRegistration::new(stub("n")).compute().deadline(1.ms());
        let result = reg.validate();
        assert!(
            result.is_err(),
            "deadline on compute node should be rejected"
        );
    }

    // ── All ExecutionClass variants via NodeRegistration ──

    #[test]
    fn all_execution_classes_via_registration() {
        // BestEffort (default)
        let reg_be = NodeRegistration::new(stub("be"));
        assert_eq!(reg_be.execution_class, ExecutionClass::BestEffort);

        // Compute
        let reg_c = NodeRegistration::new(stub("c")).compute();
        assert_eq!(reg_c.execution_class, ExecutionClass::Compute);

        // AsyncIo
        let reg_a = NodeRegistration::new(stub("a")).async_io();
        assert_eq!(reg_a.execution_class, ExecutionClass::AsyncIo);

        // Event
        let reg_e = NodeRegistration::new(stub("e")).on("sensor.data");
        assert_eq!(
            reg_e.execution_class,
            ExecutionClass::Event("sensor.data".to_string())
        );

        // Rt (via finalize with rate)
        let mut reg_rt = NodeRegistration::new(stub("rt")).rate(100_u64.hz());
        reg_rt.finalize();
        assert_eq!(reg_rt.execution_class, ExecutionClass::Rt);
    }

    // ── All Miss policy variants ──

    #[test]
    fn miss_policy_warn_is_default() {
        let reg = NodeRegistration::new(stub("n"));
        assert_eq!(reg.miss_policy, Miss::Warn);
    }

    #[test]
    fn miss_policy_all_variants_accepted_on_rt() {
        for miss in [Miss::Warn, Miss::Skip, Miss::SafeMode, Miss::Stop] {
            let mut reg = NodeRegistration::new(stub("n"))
                .rate(100_u64.hz())
                .on_miss(miss);
            reg.validate().unwrap();
            assert_eq!(reg.miss_policy, miss);
        }
    }

    // ── All BudgetPolicy variants ──

    #[test]
    fn budget_policy_default_is_warn() {
        let reg = NodeRegistration::new(stub("n"));
        assert_eq!(reg.budget_policy, BudgetPolicy::Warn);
    }

    #[test]
    fn budget_policy_enforce() {
        let reg = NodeRegistration::new(stub("n")).budget_policy(BudgetPolicy::Enforce);
        assert_eq!(reg.budget_policy, BudgetPolicy::Enforce);
    }

    #[test]
    fn budget_policy_emergency_stop() {
        let reg = NodeRegistration::new(stub("n")).budget_policy(BudgetPolicy::EmergencyStop);
        assert_eq!(reg.budget_policy, BudgetPolicy::EmergencyStop);
    }

    // ── Builder with all options set simultaneously ──

    #[test]
    fn fully_loaded_rt_builder_validates() {
        let mut reg = NodeRegistration::new(stub("full"))
            .order(1)
            .rate(1000_u64.hz())
            .budget(300.us())
            .deadline(900.us())
            .on_miss(Miss::SafeMode)
            .budget_policy(BudgetPolicy::Enforce)
            .failure_policy(FailurePolicy::restart(5, 100_u64.ms()))
            .priority(80)
            .core(0)
            .watchdog(5.ms());
        reg.validate().unwrap();

        assert_eq!(reg.order, 1);
        assert_eq!(reg.rate_hz, Some(1000.0));
        assert_eq!(reg.tick_budget, Some(300.us()));
        assert_eq!(reg.deadline, Some(900.us()));
        assert_eq!(reg.miss_policy, Miss::SafeMode);
        assert_eq!(reg.budget_policy, BudgetPolicy::Enforce);
        assert!(matches!(
            reg.failure_policy,
            Some(FailurePolicy::Restart {
                max_restarts: 5,
                ..
            })
        ));
        assert_eq!(reg.os_priority, Some(80));
        assert_eq!(reg.pinned_core, Some(0));
        assert_eq!(reg.node_watchdog, Some(5.ms()));
        assert!(reg.is_rt);
        assert_eq!(reg.execution_class, ExecutionClass::Rt);
    }

    #[test]
    fn fully_loaded_rt_builder_via_scheduler() {
        let mut scheduler = Scheduler::new();
        scheduler
            .add(StubNode("full_sched".into()))
            .order(2)
            .rate(500_u64.hz())
            .budget(1.ms())
            .deadline(1800.us())
            .on_miss(Miss::Skip)
            .budget_policy(BudgetPolicy::EmergencyStop)
            .failure_policy(FailurePolicy::Fatal)
            .priority(50)
            .core(1)
            .watchdog(10.ms())
            .build()
            .unwrap();

        let node = &scheduler.nodes[0];
        assert_eq!(node.priority, 2);
        assert!(node.is_rt_node);
        assert_eq!(node.rate_hz, Some(500.0));
        assert_eq!(node.tick_budget, Some(1.ms()));
        assert_eq!(node.deadline, Some(1800.us()));
        assert_eq!(node.miss_policy, Miss::Skip);
        assert_eq!(node.execution_class, ExecutionClass::Rt);
        assert_eq!(node.os_priority, Some(50));
        assert_eq!(node.pinned_core, Some(1));
        assert_eq!(node.node_watchdog, Some(10.ms()));
    }

    // ── Builder with no options (minimal/default) ──

    #[test]
    fn minimal_registration_defaults() {
        let mut reg = NodeRegistration::new(stub("minimal"));
        reg.validate().unwrap();

        assert_eq!(reg.order, 100);
        assert!(reg.rate_hz.is_none());
        assert!(!reg.is_rt);
        assert!(reg.tick_budget.is_none());
        assert!(reg.deadline.is_none());
        assert!(reg.failure_policy.is_none());
        assert_eq!(reg.miss_policy, Miss::Warn);
        assert_eq!(reg.execution_class, ExecutionClass::BestEffort);
        assert!(reg.source_freq.is_none());
        assert!(reg.os_priority.is_none());
        assert!(reg.pinned_core.is_none());
        assert!(reg.node_watchdog.is_none());
        assert_eq!(reg.budget_policy, BudgetPolicy::Warn);
    }

    #[test]
    fn minimal_builder_via_scheduler() {
        let mut scheduler = Scheduler::new();
        scheduler
            .add(StubNode("minimal_sched".into()))
            .build()
            .unwrap();

        let node = &scheduler.nodes[0];
        assert_eq!(node.priority, 100);
        assert!(!node.is_rt_node);
        assert!(node.rate_hz.is_none());
        assert!(node.tick_budget.is_none());
        assert!(node.deadline.is_none());
        assert_eq!(node.miss_policy, Miss::Warn);
        assert_eq!(node.execution_class, ExecutionClass::BestEffort);
    }

    // ── Boundary rate values ──

    #[test]
    fn very_small_rate_auto_derives_large_budget() {
        let mut reg = NodeRegistration::new(stub("n")).rate(0.001.hz());
        reg.finalize();
        assert!(reg.is_rt);
        // 0.001 Hz = 1000s period, budget = 800s, deadline = 950s
        let budget = reg.tick_budget.unwrap();
        let deadline = reg.deadline.unwrap();
        assert_eq!(budget, Duration::from_secs(800));
        assert_eq!(deadline.as_millis(), 950_000);
    }

    #[test]
    fn very_large_rate_auto_derives_small_budget() {
        let mut reg = NodeRegistration::new(stub("n")).rate(1_000_000_u64.hz());
        reg.finalize();
        assert!(reg.is_rt);
        // 1MHz = 1μs period, budget = 800ns, deadline = 950ns
        let budget = reg.tick_budget.unwrap();
        let deadline = reg.deadline.unwrap();
        assert_eq!(budget, Duration::from_nanos(800));
        assert_eq!(deadline, Duration::from_nanos(950));
    }

    #[test]
    fn rate_1hz_derives_correct_timing() {
        let mut reg = NodeRegistration::new(stub("n")).rate(1_u64.hz());
        reg.finalize();
        assert!(reg.is_rt);
        assert_eq!(reg.tick_budget, Some(Duration::from_millis(800)));
        assert_eq!(reg.deadline, Some(Duration::from_millis(950)));
    }

    // ── Boundary budget/deadline values ──

    #[test]
    fn very_small_budget_1ns() {
        let mut reg = NodeRegistration::new(stub("n")).budget(1_u64.ns());
        reg.validate().unwrap();
        assert_eq!(reg.tick_budget, Some(1_u64.ns()));
        assert!(reg.is_rt);
    }

    #[test]
    fn very_large_budget_1_hour() {
        let mut reg = NodeRegistration::new(stub("n")).budget(3600_u64.secs());
        reg.validate().unwrap();
        assert_eq!(reg.tick_budget, Some(Duration::from_secs(3600)));
    }

    #[test]
    fn very_small_deadline_1ns() {
        let mut reg = NodeRegistration::new(stub("n")).deadline(1_u64.ns());
        reg.validate().unwrap();
        assert_eq!(reg.deadline, Some(1_u64.ns()));
        assert!(reg.is_rt);
    }

    #[test]
    fn very_large_deadline_1_hour() {
        let mut reg = NodeRegistration::new(stub("n")).deadline(3600_u64.secs());
        reg.validate().unwrap();
        assert_eq!(reg.deadline, Some(Duration::from_secs(3600)));
    }

    #[test]
    fn zero_budget_rejected() {
        let mut reg = NodeRegistration::new(stub("n"))
            .rate(100_u64.hz())
            .budget(Duration::ZERO);
        let result = reg.validate();
        assert!(result.is_err(), "zero budget should be rejected");
    }

    #[test]
    fn zero_deadline_rejected() {
        let mut reg = NodeRegistration::new(stub("n"))
            .rate(100_u64.hz())
            .deadline(Duration::ZERO);
        let result = reg.validate();
        assert!(result.is_err(), "zero deadline should be rejected");
    }

    // ── budget-only auto-derives deadline = budget ──

    #[test]
    fn budget_only_derives_deadline_equal_to_budget() {
        let mut reg = NodeRegistration::new(stub("n")).budget(500.us());
        reg.finalize();
        // When budget is set but deadline is not, deadline = budget
        assert_eq!(reg.deadline, Some(500.us()));
    }

    #[test]
    fn budget_only_with_explicit_deadline_no_override() {
        let mut reg = NodeRegistration::new(stub("n"))
            .budget(500.us())
            .deadline(900.us());
        reg.finalize();
        // Explicit deadline should NOT be overridden by budget
        assert_eq!(reg.deadline, Some(900.us()));
        assert_eq!(reg.tick_budget, Some(500.us()));
    }

    // ── finalize() idempotency ──

    #[test]
    fn finalize_called_twice_produces_same_result() {
        let mut reg = NodeRegistration::new(stub("n")).rate(100_u64.hz());
        reg.finalize();
        let budget_first = reg.tick_budget;
        let deadline_first = reg.deadline;
        let is_rt_first = reg.is_rt;
        let class_first = reg.execution_class.clone();

        reg.finalize();
        assert_eq!(reg.tick_budget, budget_first);
        assert_eq!(reg.deadline, deadline_first);
        assert_eq!(reg.is_rt, is_rt_first);
        assert_eq!(reg.execution_class, class_first);
    }

    // ── validate() calls finalize() internally ──

    #[test]
    fn validate_auto_finalizes() {
        let mut reg = NodeRegistration::new(stub("n")).rate(100_u64.hz());
        // Do NOT call finalize() — validate() should call it
        assert!(!reg.is_rt, "should not be RT before finalize");
        reg.validate().unwrap();
        assert!(reg.is_rt, "validate() should have called finalize()");
        assert_eq!(reg.execution_class, ExecutionClass::Rt);
    }

    // ── Triple execution class override (last wins) ──

    #[test]
    fn triple_class_override_last_wins() {
        let reg = NodeRegistration::new(stub("n"))
            .compute()
            .async_io()
            .on("final.topic");
        assert_eq!(
            reg.execution_class,
            ExecutionClass::Event("final.topic".to_string())
        );
    }

    #[test]
    fn quadruple_class_override_last_wins() {
        let reg = NodeRegistration::new(stub("n"))
            .on("a")
            .compute()
            .async_io()
            .compute();
        assert_eq!(reg.execution_class, ExecutionClass::Compute);
    }

    // ── Event topic name edge cases ──

    #[test]
    fn event_topic_with_dots_valid() {
        let mut reg = NodeRegistration::new(stub("n")).on("sensor.lidar.scan");
        reg.validate().unwrap();
        assert_eq!(
            reg.execution_class,
            ExecutionClass::Event("sensor.lidar.scan".to_string())
        );
    }

    #[test]
    fn event_topic_very_long_valid() {
        let long_topic = "a".repeat(1000);
        let mut reg = NodeRegistration::new(stub("n")).on(&long_topic);
        reg.validate().unwrap();
        assert_eq!(reg.execution_class, ExecutionClass::Event(long_topic));
    }

    // ── Priority and core edge values ──

    #[test]
    fn priority_min_value() {
        let reg = NodeRegistration::new(stub("n")).priority(1);
        assert_eq!(reg.os_priority, Some(1));
    }

    #[test]
    fn priority_max_value() {
        let reg = NodeRegistration::new(stub("n")).priority(99);
        assert_eq!(reg.os_priority, Some(99));
    }

    #[test]
    fn core_zero() {
        let reg = NodeRegistration::new(stub("n")).core(0);
        assert_eq!(reg.pinned_core, Some(0));
    }

    #[test]
    fn core_large_value() {
        let reg = NodeRegistration::new(stub("n")).core(127);
        assert_eq!(reg.pinned_core, Some(127));
    }

    // ── Watchdog edge values ──

    #[test]
    fn watchdog_very_small() {
        let reg = NodeRegistration::new(stub("n")).watchdog(1_u64.us());
        assert_eq!(reg.node_watchdog, Some(1_u64.us()));
    }

    #[test]
    fn watchdog_very_large() {
        let reg = NodeRegistration::new(stub("n")).watchdog(60_u64.secs());
        assert_eq!(reg.node_watchdog, Some(Duration::from_secs(60)));
    }

    // ── Compute + rate stores rate but not RT ──

    #[test]
    fn compute_with_rate_stores_rate_not_rt_after_validate() {
        let mut reg = NodeRegistration::new(stub("n"))
            .compute()
            .rate(100_u64.hz());
        reg.validate().unwrap();

        assert_eq!(reg.execution_class, ExecutionClass::Compute);
        assert!(!reg.is_rt);
        assert_eq!(reg.rate_hz, Some(100.0));
        // Compute nodes should NOT get auto-derived budget/deadline
        assert!(reg.tick_budget.is_none());
        assert!(reg.deadline.is_none());
    }

    // ── AsyncIo + rate stores rate but not RT ──

    #[test]
    fn async_io_with_rate_stores_rate_not_rt_after_validate() {
        let mut reg = NodeRegistration::new(stub("n"))
            .async_io()
            .rate(10_u64.hz());
        reg.validate().unwrap();

        assert_eq!(reg.execution_class, ExecutionClass::AsyncIo);
        assert!(!reg.is_rt);
        assert_eq!(reg.rate_hz, Some(10.0));
        assert!(reg.tick_budget.is_none());
        assert!(reg.deadline.is_none());
    }

    // ── Event + rate stores rate but not RT ──

    #[test]
    fn event_with_rate_stores_rate_not_rt_after_validate() {
        let mut reg = NodeRegistration::new(stub("n"))
            .on("cmd.vel")
            .rate(50_u64.hz());
        reg.validate().unwrap();

        assert_eq!(
            reg.execution_class,
            ExecutionClass::Event("cmd.vel".to_string())
        );
        assert!(!reg.is_rt);
        assert_eq!(reg.rate_hz, Some(50.0));
        assert!(reg.tick_budget.is_none());
        assert!(reg.deadline.is_none());
    }

    // ── BudgetPolicy set twice, last wins ──

    #[test]
    fn budget_policy_set_twice_last_wins() {
        let reg = NodeRegistration::new(stub("n"))
            .budget_policy(BudgetPolicy::EmergencyStop)
            .budget_policy(BudgetPolicy::Warn);
        assert_eq!(reg.budget_policy, BudgetPolicy::Warn);
    }

    // ── Validation error messages contain useful info ──

    #[test]
    fn zero_budget_error_mentions_budget() {
        let mut reg = NodeRegistration::new(stub("n"))
            .rate(100_u64.hz())
            .budget(Duration::ZERO);
        let err = reg.validate().unwrap_err();
        let msg = format!("{}", err);
        assert!(
            msg.contains("budget"),
            "error should mention budget: {}",
            msg
        );
    }

    #[test]
    fn zero_deadline_error_mentions_deadline() {
        let mut reg = NodeRegistration::new(stub("n"))
            .rate(100_u64.hz())
            .deadline(Duration::ZERO);
        let err = reg.validate().unwrap_err();
        let msg = format!("{}", err);
        assert!(
            msg.contains("deadline"),
            "error should mention deadline: {}",
            msg
        );
    }

    #[test]
    fn compute_budget_conflict_error_mentions_both() {
        let mut reg = NodeRegistration::new(stub("n")).compute().budget(500.us());
        let err = reg.validate().unwrap_err();
        let msg = format!("{}", err);
        assert!(
            msg.contains("budget") || msg.contains("Compute") || msg.contains("compute"),
            "error should mention the conflict: {}",
            msg
        );
    }
}
