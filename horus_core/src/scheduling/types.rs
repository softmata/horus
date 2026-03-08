//! Scheduler types and data structures
//!
//! This module contains the public types used by the scheduler.

use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use serde::{Deserialize, Serialize};

use super::fault_tolerance::FailureHandler;
use super::profiler::RuntimeProfiler;
use super::record_replay::NodeRecorder;
use crate::core::{Node, NodeInfo, RtNode, RtStats};

/// Node tier for explicit annotation by developers
///
/// Use this to declare a node's execution characteristics at compile time.
/// The tier influences the default failure policy applied to the node.
///
/// # Example
/// ```ignore
/// use horus_core::scheduling::{NodeTier, Scheduler};
///
/// let mut scheduler = Scheduler::new();
/// scheduler.add(pid_node).order(0).tier(NodeTier::UltraFast).done();
/// scheduler.add(sensor_node).order(1).tier(NodeTier::Fast).done();
/// scheduler.add(logger_node).order(5).tier(NodeTier::Normal).done();
/// ```
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize, Default)]
pub enum NodeTier {
    /// Ultra-fast nodes (<1us) - highest priority inline execution
    /// Use for: PID controllers, simple math, data transformations
    UltraFast,

    /// Fast nodes (<10us) - high priority inline execution
    /// Use for: Sensor readers, filter calculations, state machines
    #[default]
    Fast,

    /// Normal nodes (<100us+) - standard scheduling
    /// Use for: Complex algorithms, data processing, logging, diagnostics
    Normal,
}

impl NodeTier {
    /// Get the default failure policy for this tier.
    ///
    /// Each tier has a sensible default that matches its criticality:
    /// - **UltraFast/Fast**: `Fatal` — these are control-loop nodes; failure = stop.
    /// - **Normal**: `Restart` with exponential backoff (5 attempts, 100ms initial).
    pub fn default_failure_policy(&self) -> super::fault_tolerance::failure_policy::FailurePolicy {
        use super::fault_tolerance::failure_policy::FailurePolicy;
        match self {
            NodeTier::UltraFast | NodeTier::Fast => FailurePolicy::Fatal,
            NodeTier::Normal => FailurePolicy::restart(5, 100),
        }
    }
}

#[cfg(test)]
mod node_tier_tests {
    use super::*;

    #[test]
    fn test_node_tier_default() {
        assert_eq!(NodeTier::default(), NodeTier::Fast);
    }
}

#[cfg(test)]
mod execution_class_tests {
    use super::*;
    use crate::core::{Node, NodeInfo};

    #[test]
    fn test_execution_class_default() {
        assert_eq!(ExecutionClass::default(), ExecutionClass::BestEffort);
    }

    struct StubNode(String);
    impl Node for StubNode {
        fn name(&self) -> &str {
            &self.0
        }
        fn tick(&mut self) {}
    }

    fn make_node(name: &str, class: ExecutionClass) -> RegisteredNode {
        RegisteredNode {
            node: NodeKind::Regular(Box::new(StubNode(name.to_string()))),
            name: Arc::from(name),
            priority: 0,
            initialized: true,
            context: Some(NodeInfo::new(name.to_string())),
            rate_hz: None,
            last_tick: None,
            failure_handler: super::super::fault_tolerance::FailureHandler::new(
                super::super::fault_tolerance::FailurePolicy::Fatal,
            ),
            is_rt_node: matches!(class, ExecutionClass::Rt),
            wcet_budget: None,
            deadline: None,
            recorder: None,
            is_stopped: false,
            is_paused: false,
            rt_stats: None,
            execution_class: class,
            has_custom_failure_policy: false,
        }
    }

    #[test]
    fn test_group_nodes_by_class_empty() {
        let groups = group_nodes_by_class(Vec::new());
        assert!(groups.rt_nodes.is_empty());
        assert!(groups.compute_nodes.is_empty());
        assert!(groups.event_nodes.is_empty());
        assert!(groups.async_io_nodes.is_empty());
        assert!(groups.main_nodes.is_empty());
    }

    #[test]
    fn test_group_nodes_by_class_mixed() {
        let nodes = vec![
            make_node("rt_ctrl", ExecutionClass::Rt),
            make_node("planner", ExecutionClass::Compute),
            make_node("sensor", ExecutionClass::BestEffort),
            make_node("lidar_cb", ExecutionClass::Event("lidar_scan".into())),
            make_node("rt_sensor", ExecutionClass::Rt),
            make_node("logger", ExecutionClass::AsyncIo),
        ];

        let groups = group_nodes_by_class(nodes);

        assert_eq!(groups.rt_nodes.len(), 2);
        assert_eq!(&*groups.rt_nodes[0].name, "rt_ctrl");
        assert_eq!(&*groups.rt_nodes[1].name, "rt_sensor");

        assert_eq!(groups.compute_nodes.len(), 1);
        assert_eq!(&*groups.compute_nodes[0].name, "planner");

        assert_eq!(groups.event_nodes.len(), 1);
        assert_eq!(&*groups.event_nodes[0].name, "lidar_cb");

        assert_eq!(groups.async_io_nodes.len(), 1);
        assert_eq!(&*groups.async_io_nodes[0].name, "logger");

        assert_eq!(groups.main_nodes.len(), 1);
        assert_eq!(&*groups.main_nodes[0].name, "sensor");
    }

    #[test]
    fn test_group_nodes_best_effort_goes_to_main() {
        let nodes = vec![
            make_node("a", ExecutionClass::BestEffort),
            make_node("b", ExecutionClass::Compute),
        ];

        let groups = group_nodes_by_class(nodes);
        assert_eq!(groups.compute_nodes.len(), 1);
        assert_eq!(groups.main_nodes.len(), 1);
        assert!(groups.rt_nodes.is_empty());
        assert!(groups.event_nodes.is_empty());
        assert!(groups.async_io_nodes.is_empty());
    }
}

/// Execution class for a node — determines which execution group it belongs to.
///
/// Each class maps to a different scheduling strategy:
/// - **Rt**: Dedicated high-priority thread with spin-wait timing
/// - **Compute**: Parallel thread pool for CPU-bound work
/// - **Event**: Triggered by topic updates
/// - **AsyncIo**: Tokio blocking pool for I/O-bound work
/// - **BestEffort**: Default — runs in the main tick loop
///
/// # Example
///
/// ```rust,ignore
/// use horus::prelude::*;
///
/// let mut scheduler = Scheduler::new();
///
/// // Motor control on dedicated RT thread
/// scheduler.add(motor_node).order(0).rt().build();
///
/// // Path planning in parallel compute pool
/// scheduler.add(planner_node).order(5).compute().build();
///
/// // React to LiDAR scans when they arrive
/// scheduler.add(obstacle_node).on("lidar_scan").build();
///
/// // Telemetry upload on async I/O pool
/// scheduler.add(telemetry_node).async_io().rate_hz(1.0).build();
/// ```
#[derive(Debug, Clone, PartialEq, Eq, Default)]
pub enum ExecutionClass {
    /// Real-time node — runs on a dedicated RT thread with priority scheduling.
    Rt,
    /// Compute node — runs in a parallel thread pool.
    Compute,
    /// Event-triggered node — wakes when a topic is updated.
    Event(String),
    /// Async I/O node — runs via tokio::task::spawn_blocking on a tokio runtime.
    AsyncIo,
    /// Best-effort — default scheduling in the main tick loop.
    #[default]
    BestEffort,
}

/// Wrapper that unifies regular nodes and RT nodes under a single dispatch.
pub(crate) enum NodeKind {
    /// A regular Node (may still have RT metadata from builder `.rt()`)
    Regular(Box<dyn Node>),
    /// A node implementing the full RtNode trait
    Rt(Box<dyn RtNode>),
}

impl NodeKind {
    // ---- Node trait delegation ----

    #[inline]
    pub(crate) fn tick(&mut self) {
        match self {
            NodeKind::Regular(n) => n.tick(),
            NodeKind::Rt(n) => n.tick(),
        }
    }

    #[inline]
    pub(crate) fn name(&self) -> &str {
        match self {
            NodeKind::Regular(n) => n.name(),
            NodeKind::Rt(n) => n.name(),
        }
    }

    pub(crate) fn init(&mut self) -> crate::error::HorusResult<()> {
        match self {
            NodeKind::Regular(n) => n.init(),
            NodeKind::Rt(n) => n.init(),
        }
    }

    pub(crate) fn shutdown(&mut self) -> crate::error::HorusResult<()> {
        match self {
            NodeKind::Regular(n) => n.shutdown(),
            NodeKind::Rt(n) => n.shutdown(),
        }
    }

    pub(crate) fn on_error(&mut self, error: &str) {
        match self {
            NodeKind::Regular(n) => n.on_error(error),
            NodeKind::Rt(n) => n.on_error(error),
        }
    }

    pub(crate) fn rate_hz(&self) -> Option<f64> {
        match self {
            NodeKind::Regular(n) => n.rate_hz(),
            NodeKind::Rt(n) => n.rate_hz(),
        }
    }

    pub(crate) fn publishers(&self) -> Vec<crate::core::TopicMetadata> {
        match self {
            NodeKind::Regular(n) => n.publishers(),
            NodeKind::Rt(n) => n.publishers(),
        }
    }

    pub(crate) fn subscribers(&self) -> Vec<crate::core::TopicMetadata> {
        match self {
            NodeKind::Regular(n) => n.subscribers(),
            NodeKind::Rt(n) => n.subscribers(),
        }
    }

    // ---- RtNode-specific access ----

    /// Get a mutable reference to the RtNode, if this is an Rt variant.
    #[inline]
    pub(crate) fn as_rt_mut(&mut self) -> Option<&mut dyn RtNode> {
        match self {
            NodeKind::Rt(n) => Some(n.as_mut()),
            NodeKind::Regular(_) => None,
        }
    }

    /// Get a reference to the RtNode, if this is an Rt variant.
    #[inline]
    pub(crate) fn as_rt(&self) -> Option<&dyn RtNode> {
        match self {
            NodeKind::Rt(n) => Some(n.as_ref()),
            NodeKind::Regular(_) => None,
        }
    }
}

/// Result of grouping nodes by their ExecutionClass.
///
/// Used by the scheduler at init time to split nodes into their execution groups.
pub(crate) struct NodeGroups {
    /// Nodes with ExecutionClass::Rt — scheduled on the dedicated RT thread.
    pub rt_nodes: Vec<RegisteredNode>,
    /// Nodes with ExecutionClass::Compute — scheduled in the parallel thread pool.
    pub compute_nodes: Vec<RegisteredNode>,
    /// Nodes with ExecutionClass::Event — triggered by topic updates.
    pub event_nodes: Vec<RegisteredNode>,
    /// Nodes with ExecutionClass::AsyncIo — run via tokio spawn_blocking.
    pub async_io_nodes: Vec<RegisteredNode>,
    /// Nodes with ExecutionClass::BestEffort — sequential execution on main thread.
    pub main_nodes: Vec<RegisteredNode>,
}

/// Group nodes by their ExecutionClass into execution groups.
///
/// - `Rt` → `rt_nodes` (dedicated RT thread)
/// - `Compute` → `compute_nodes` (parallel thread pool)
/// - `Event(_)` → `event_nodes` (per-node watcher threads)
/// - `AsyncIo` → `async_io_nodes` (tokio blocking pool)
/// - `BestEffort` → `main_nodes` (sequential on main thread)
pub(crate) fn group_nodes_by_class(nodes: Vec<RegisteredNode>) -> NodeGroups {
    let mut rt_nodes = Vec::new();
    let mut compute_nodes = Vec::new();
    let mut event_nodes = Vec::new();
    let mut async_io_nodes = Vec::new();
    let mut main_nodes = Vec::new();

    for node in nodes {
        match &node.execution_class {
            ExecutionClass::Rt => rt_nodes.push(node),
            ExecutionClass::Compute => compute_nodes.push(node),
            ExecutionClass::Event(_) => event_nodes.push(node),
            ExecutionClass::AsyncIo => async_io_nodes.push(node),
            ExecutionClass::BestEffort => main_nodes.push(node),
        }
    }

    NodeGroups {
        rt_nodes,
        compute_nodes,
        event_nodes,
        async_io_nodes,
        main_nodes,
    }
}

/// Enhanced node registration info with lifecycle tracking and per-node rate control
pub(crate) struct RegisteredNode {
    pub(crate) node: NodeKind,
    /// Cached node name — captured once at registration, used everywhere.
    /// Uses `Arc<str>` so clones in the scheduler tick loop are cheap
    /// (atomic increment instead of heap allocation + memcpy).
    pub(crate) name: Arc<str>,
    pub(crate) priority: u32,
    pub(crate) initialized: bool,
    pub(crate) context: Option<NodeInfo>,
    pub(crate) rate_hz: Option<f64>,
    pub(crate) last_tick: Option<Instant>,
    pub(crate) failure_handler: FailureHandler,
    /// Whether the user explicitly set a failure policy (don't override in apply_fault_tolerance)
    pub(crate) has_custom_failure_policy: bool,
    pub(crate) is_rt_node: bool,
    pub(crate) wcet_budget: Option<Duration>,
    pub(crate) deadline: Option<Duration>,
    pub(crate) recorder: Option<NodeRecorder>,
    pub(crate) is_stopped: bool,
    pub(crate) is_paused: bool,
    /// Per-node real-time statistics (populated for RT nodes)
    pub(crate) rt_stats: Option<RtStats>,
    /// Execution class — determines which group this node belongs to.
    pub(crate) execution_class: ExecutionClass,
}

/// Shared monitoring references passed to executor threads.
///
/// Each executor receives a clone of the shared profiler and (optionally)
/// blackbox so that monitoring data is collected across all execution groups.
#[derive(Clone)]
pub(crate) struct SharedMonitors {
    pub profiler: Arc<Mutex<RuntimeProfiler>>,
    pub blackbox: Option<Arc<Mutex<super::blackbox::BlackBox>>>,
    /// When false, suppresses non-emergency print_line calls in executor threads.
    pub verbose: bool,
}
