//! Scheduler types and data structures
//!
//! This module contains the public types used by the scheduler.

use std::sync::atomic::{AtomicU64, AtomicU8, Ordering};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use super::fault_tolerance::FailureHandler;
use super::profiler::RuntimeProfiler;
use super::record_replay::NodeRecorder;
use super::safety_monitor::BudgetPolicy;
use crate::core::{Miss, Node, NodeInfo, RtStats};

/// Health state of a node, tracked by the watchdog system.
///
/// Transitions:
/// - `Healthy` → `Warning` (1x timeout): node is slow, logged but still ticked
/// - `Warning` → `Unhealthy` (2x timeout): node is skipped in tick loop
/// - `Unhealthy` → `Isolated` (3x timeout, critical node): `enter_safe_state()` called
/// - Any → `Healthy`: node ticks successfully (recovery)
#[doc(hidden)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum NodeHealthState {
    /// Normal operation — node is ticked every cycle.
    Healthy = 0,
    /// Watchdog warning (1x timeout elapsed) — node still ticks, but logged.
    Warning = 1,
    /// Unhealthy (2x timeout) — node is skipped in tick loop.
    Unhealthy = 2,
    /// Isolated (3x timeout on critical node) — `enter_safe_state()` called, node skipped.
    Isolated = 3,
}

impl NodeHealthState {
    /// Convert from raw u8 (for AtomicU8). Returns Healthy for unknown values.
    pub fn from_u8(v: u8) -> Self {
        match v {
            0 => Self::Healthy,
            1 => Self::Warning,
            2 => Self::Unhealthy,
            3 => Self::Isolated,
            _ => Self::Healthy,
        }
    }
}

impl std::fmt::Display for NodeHealthState {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Healthy => write!(f, "Healthy"),
            Self::Warning => write!(f, "Warning"),
            Self::Unhealthy => write!(f, "Unhealthy"),
            Self::Isolated => write!(f, "Isolated"),
        }
    }
}

/// Atomic wrapper for `NodeHealthState`, enabling lock-free per-node health tracking.
///
/// Uses `AtomicU8` internally — no lock contention on read or write.
#[doc(hidden)]
#[derive(Debug)]
pub struct AtomicHealthState(AtomicU8);

impl AtomicHealthState {
    pub fn new(state: NodeHealthState) -> Self {
        Self(AtomicU8::new(state as u8))
    }

    pub fn load(&self) -> NodeHealthState {
        NodeHealthState::from_u8(self.0.load(Ordering::Acquire))
    }

    pub fn store(&self, state: NodeHealthState) {
        self.0.store(state as u8, Ordering::Release);
    }
}

impl Default for AtomicHealthState {
    fn default() -> Self {
        Self::new(NodeHealthState::Healthy)
    }
}

#[cfg(test)]
mod execution_class_tests {
    use super::*;
    use crate::core::{Miss, Node, NodeInfo};

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
            node: NodeKind::new(Box::new(StubNode(name.to_string()))),
            name: Arc::from(name),
            priority: 0,
            initialized: true,
            context: Some(NodeInfo::new(name.to_string())),
            rate_hz: None,
            last_tick: None,
            is_rt_node: matches!(class, ExecutionClass::Rt),
            tick_budget: None,
            deadline: None,
            recorder: None,
            is_stopped: false,
            is_paused: false,
            rt_stats: None,
            miss_policy: Miss::Warn,
            execution_class: class,
            health_state: AtomicHealthState::default(),
            os_priority: None,
            pinned_core: None,
            node_watchdog: None,
            failure_handler: None,
            budget_policy: crate::scheduling::safety_monitor::BudgetPolicy::default(),
            use_sched_deadline: false,
            no_alloc: false,
            subscription_freshness: Vec::new(),
        }
    }

    #[test]
    fn test_group_nodes_by_class_empty() {
        let groups = group_nodes_by_class(Vec::new());
        assert!(groups.rt_nodes.is_empty());
        assert!(groups.compute_nodes.is_empty());
        assert!(groups.event_nodes.is_empty());
        assert!(groups.async_io_nodes.is_empty());
        assert!(groups.gpu_nodes.is_empty());
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
        assert!(groups.gpu_nodes.is_empty());
    }

    // ── NodeHealthState tests ────────────────────────────────────────────

    #[test]
    fn test_health_state_default_is_healthy() {
        let state = AtomicHealthState::default();
        assert_eq!(state.load(), NodeHealthState::Healthy);
    }

    #[test]
    fn test_health_state_store_load_roundtrip() {
        let state = AtomicHealthState::new(NodeHealthState::Healthy);

        state.store(NodeHealthState::Warning);
        assert_eq!(state.load(), NodeHealthState::Warning);

        state.store(NodeHealthState::Unhealthy);
        assert_eq!(state.load(), NodeHealthState::Unhealthy);

        state.store(NodeHealthState::Isolated);
        assert_eq!(state.load(), NodeHealthState::Isolated);

        state.store(NodeHealthState::Healthy);
        assert_eq!(state.load(), NodeHealthState::Healthy);
    }

    #[test]
    fn test_health_state_from_u8() {
        assert_eq!(NodeHealthState::from_u8(0), NodeHealthState::Healthy);
        assert_eq!(NodeHealthState::from_u8(1), NodeHealthState::Warning);
        assert_eq!(NodeHealthState::from_u8(2), NodeHealthState::Unhealthy);
        assert_eq!(NodeHealthState::from_u8(3), NodeHealthState::Isolated);
        // Unknown values default to Healthy
        assert_eq!(NodeHealthState::from_u8(255), NodeHealthState::Healthy);
    }

    #[test]
    fn test_health_state_display() {
        assert_eq!(NodeHealthState::Healthy.to_string(), "Healthy");
        assert_eq!(NodeHealthState::Warning.to_string(), "Warning");
        assert_eq!(NodeHealthState::Unhealthy.to_string(), "Unhealthy");
        assert_eq!(NodeHealthState::Isolated.to_string(), "Isolated");
    }

    #[test]
    fn test_registered_node_default_health() {
        let node = make_node("test", ExecutionClass::BestEffort);
        assert_eq!(node.health_state.load(), NodeHealthState::Healthy);
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
/// Users should not construct this directly. Use the node builder methods instead:
/// `.compute()`, `.on("topic")`, `.async_io()`, `.rate()` (auto-sets Rt).
#[doc(hidden)]
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
    /// GPU node — runs on a dedicated GPU executor thread with CUDA stream management.
    /// Kernels launched in tick() execute asynchronously on the GPU.
    Gpu,
    /// Best-effort — default scheduling in the main tick loop.
    #[default]
    BestEffort,
}

/// Thin wrapper around `Box<dyn Node>`.
///
/// All RT methods live on the `Node` trait with safe defaults.
/// The `is_rt_node` field on `RegisteredNode` tracks RT scheduling.
///
/// Implements `Deref<Target = dyn Node>` so callers can call any Node
/// method directly (e.g. `node.tick()`, `node.name()`).
pub(crate) struct NodeKind(pub(crate) Box<dyn Node>);

impl NodeKind {
    /// Wrap a boxed node.
    #[inline]
    pub(crate) fn new(node: Box<dyn Node>) -> Self {
        Self(node)
    }
}

impl std::ops::Deref for NodeKind {
    type Target = dyn Node;

    #[inline]
    fn deref(&self) -> &Self::Target {
        &*self.0
    }
}

impl std::ops::DerefMut for NodeKind {
    #[inline]
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut *self.0
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
    /// Nodes with ExecutionClass::Gpu — scheduled on the GPU executor thread.
    pub gpu_nodes: Vec<RegisteredNode>,
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
    let mut gpu_nodes = Vec::new();
    let mut main_nodes = Vec::new();

    for node in nodes {
        match &node.execution_class {
            ExecutionClass::Rt => rt_nodes.push(node),
            ExecutionClass::Compute => compute_nodes.push(node),
            ExecutionClass::Event(_) => event_nodes.push(node),
            ExecutionClass::AsyncIo => async_io_nodes.push(node),
            ExecutionClass::Gpu => gpu_nodes.push(node),
            ExecutionClass::BestEffort => main_nodes.push(node),
        }
    }

    NodeGroups {
        rt_nodes,
        compute_nodes,
        event_nodes,
        async_io_nodes,
        gpu_nodes,
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
    pub(crate) is_rt_node: bool,
    pub(crate) tick_budget: Option<Duration>,
    pub(crate) deadline: Option<Duration>,
    pub(crate) recorder: Option<NodeRecorder>,
    pub(crate) is_stopped: bool,
    pub(crate) is_paused: bool,
    /// Per-node real-time statistics (populated for RT nodes)
    pub(crate) rt_stats: Option<RtStats>,
    /// What to do when this node misses its deadline.
    pub(crate) miss_policy: Miss,
    /// Execution class — determines which group this node belongs to.
    pub(crate) execution_class: ExecutionClass,
    /// Per-node health state — lock-free AtomicU8 for zero-contention reads.
    pub(crate) health_state: AtomicHealthState,
    /// OS thread priority (SCHED_FIFO 1-99) for this node's RT thread.
    pub(crate) os_priority: Option<i32>,
    /// CPU core to pin this node's RT thread to.
    pub(crate) pinned_core: Option<usize>,
    /// Per-node watchdog timeout (overrides scheduler global).
    pub(crate) node_watchdog: Option<Duration>,
    /// Failure policy handler — tracks restart count, backoff, cooldown state.
    #[allow(dead_code)]
    // false positive: read via self.nodes[i].failure_handler in node_ops.rs
    pub(crate) failure_handler: Option<FailureHandler>,
    /// Policy for budget violation enforcement.
    pub(crate) budget_policy: BudgetPolicy,
    /// Opt-in to SCHED_DEADLINE (EDF) instead of SCHED_FIFO.
    pub(crate) use_sched_deadline: bool,
    /// Panic on heap allocation during tick().
    pub(crate) no_alloc: bool,
    /// Per-subscription freshness watchdogs.
    /// Tracks last-received timestamp and staleness policy for each subscribed topic.
    pub(crate) subscription_freshness: Vec<SubscriptionFreshness>,
}

/// Tracks freshness of a single subscription for stale data detection.
#[derive(Debug)]
pub struct SubscriptionFreshness {
    /// Topic name.
    pub topic: String,
    /// Maximum allowed time between messages.
    pub timeout: Duration,
    /// What to do when the topic goes stale.
    pub policy: StalePolicy,
    /// Last time a message was received on this topic (nanoseconds since epoch).
    /// Updated by the scheduler when Topic::recv() returns Some.
    pub last_received_ns: AtomicU64,
}

/// Policy for handling stale subscriptions (no new data within timeout).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum StalePolicy {
    /// Log a warning, continue ticking.
    Warn,
    /// Call node.enter_safe_state().
    SafeState,
    /// Stop the node (mark is_stopped = true).
    Stop,
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
    /// Live SHM registry — updated every tick (~5ns atomic writes).
    pub registry: Option<Arc<super::registry::SchedulerRegistry>>,
    /// Node name → registry slot index for O(1) updates.
    pub registry_slots: Arc<std::collections::HashMap<String, usize>>,
    /// Cross-thread node control: pause/stop flags set by main thread, read by executors.
    pub node_controls: Arc<NodeControlMap>,
}

/// Shared atomic control flags for each node, keyed by name.
/// The main thread sets flags via CLI control commands (horus node kill/pause/resume).
/// Executor threads check flags on every tick.
#[derive(Default)]
pub struct NodeControlMap {
    inner: std::sync::RwLock<std::collections::HashMap<String, NodeControl>>,
}

/// Per-node control flags.
pub struct NodeControl {
    pub paused: std::sync::atomic::AtomicBool,
    pub stopped: std::sync::atomic::AtomicBool,
}

impl NodeControlMap {
    pub fn register(&self, name: &str) {
        let mut map = self.inner.write().unwrap_or_else(|e| e.into_inner());
        map.entry(name.to_string()).or_insert_with(|| NodeControl {
            paused: std::sync::atomic::AtomicBool::new(false),
            stopped: std::sync::atomic::AtomicBool::new(false),
        });
    }

    pub fn is_paused(&self, name: &str) -> bool {
        self.inner
            .read()
            .unwrap_or_else(|e| e.into_inner())
            .get(name)
            .map(|c| c.paused.load(std::sync::atomic::Ordering::Relaxed))
            .unwrap_or(false)
    }

    pub fn is_stopped(&self, name: &str) -> bool {
        self.inner
            .read()
            .unwrap_or_else(|e| e.into_inner())
            .get(name)
            .map(|c| c.stopped.load(std::sync::atomic::Ordering::Relaxed))
            .unwrap_or(false)
    }

    pub fn set_paused(&self, name: &str, paused: bool) {
        if let Some(c) = self
            .inner
            .read()
            .unwrap_or_else(|e| e.into_inner())
            .get(name)
        {
            c.paused.store(paused, std::sync::atomic::Ordering::Relaxed);
        }
    }

    pub fn set_stopped(&self, name: &str) {
        if let Some(c) = self
            .inner
            .read()
            .unwrap_or_else(|e| e.into_inner())
            .get(name)
        {
            c.stopped.store(true, std::sync::atomic::Ordering::Relaxed);
        }
    }
}

impl SharedMonitors {
    /// Update registry with node metrics after a tick.
    /// No-op if registry is not configured (~0ns).
    #[inline]
    pub fn update_registry(&self, node: &RegisteredNode, tick_ns: u64) {
        if let Some(ref registry) = self.registry {
            if let Some(&slot) = self.registry_slots.get(node.name.as_ref()) {
                let (tick_count, error_count) = if let Some(ref ctx) = node.context {
                    let m = ctx.metrics();
                    (m.total_ticks(), m.errors_count() as u32)
                } else {
                    (0, 0)
                };
                let health = node.health_state.load() as u8;
                let (budget_misses, deadline_misses, avg_ns, max_ns, p99_ns) =
                    if let Some(ref stats) = node.rt_stats {
                        (
                            stats.budget_violations() as u32,
                            stats.deadline_misses() as u32,
                            (stats.avg_execution_us() * 1000.0) as u64,
                            stats.worst_execution().as_nanos() as u64,
                            stats.p99_approx_ns(),
                        )
                    } else {
                        (0, 0, 0, 0, 0)
                    };
                registry.update_node(
                    slot,
                    health,
                    tick_count,
                    error_count,
                    budget_misses,
                    deadline_misses,
                    health, // watchdog_severity mirrors health (0=Ok, 1=Warning, 2=Expired, 3=Critical)
                    tick_ns,
                    avg_ns,
                    max_ns,
                    p99_ns,
                );
            }
        }
    }
}
