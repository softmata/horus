//! Scheduler types and data structures
//!
//! This module contains the public types used by the scheduler.

use std::sync::atomic::{AtomicU64, AtomicU8, Ordering};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use super::fault_tolerance::{FailureAction, FailureHandler};
use super::profiler::RuntimeProfiler;
use super::record_replay::NodeRecorder;
use super::safety_monitor::BudgetPolicy;
use crate::core::{Miss, Node, NodeInfo, RtStats};

/// Health state of a node, tracked by the watchdog system.
///
/// Transitions:
/// - `Healthy` → `Warning` (1x timeout): node is slow, logged but still ticked
/// - `Warning` → `Unhealthy` (2x timeout): recorded and reported
/// - `Unhealthy` → `Isolated` (3x timeout, critical node): `enter_safe_state()` called
/// - Any → `Healthy`: node ticks successfully (recovery)
///
/// # Why watchdog-derived states do NOT suppress ticking
///
/// These states are observability, not a gate — a node whose watchdog is
/// overdue keeps being offered ticks. That is deliberate. The watchdog answers
/// "did this node tick recently", and the tick loop is what feeds it, so
/// suppressing on watchdog health is self-reinforcing: a node marked
/// `Unhealthy` at 2x would stop feeding, reach 3x, and latch a system-wide
/// emergency stop. A transient overrun — one page fault, one scheduling
/// hiccup — would reliably halt the robot a few hundred milliseconds later.
///
/// Suppressing and *also* feeding the watchdog is not the answer either: it
/// makes the 3x rung unreachable for any node that got as far as `Unhealthy`,
/// silently capping the ladder one rung short of the safing it exists to
/// perform.
///
/// Stopping a node is the job of the separate degradation ladder, whose
/// `DegradationAction::Kill` sets `is_stopped` — a flag every executor and the
/// main loop honour — and whose `Isolate` safes the node while leaving it able
/// to demonstrate recovery. The main loop additionally suppresses
/// `Unhealthy`/`Isolated` nodes with a periodic probe tick; that predates this
/// note and applies only to nodes it still owns.
#[doc(hidden)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum NodeHealthState {
    /// Normal operation — node is ticked every cycle.
    Healthy = 0,
    /// Watchdog warning (1x timeout elapsed) — node still ticks, but logged.
    Warning = 1,
    /// Unhealthy (2x timeout) — recorded and reported; see the type-level note
    /// on why this does not by itself stop the node being ticked.
    Unhealthy = 2,
    /// Isolated (3x timeout on critical node) — `enter_safe_state()` called.
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
            health_probe_counter: 0,
            is_paused: false,
            diag: Default::default(),
            in_safe_mode: false,
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
/// Which recurring RT diagnostic a throttled message belongs to.
///
/// One slot each, so a node reporting deadline misses does not suppress its own
/// budget-violation reports — they are different faults and an operator needs to
/// see both.
#[derive(Clone, Copy, PartialEq, Eq)]
pub(crate) enum Diag {
    DeadlineMiss = 0,
    BudgetViolation = 1,
    SafeStateEnter = 2,
    SafeStateLeave = 3,
}

/// Per-node, per-kind rate limiter for messages that can fire every tick.
///
/// A node that misses its deadline at 1 kHz produces a thousand identical lines
/// a second. Measured on one node alternating hit/miss around a 1 ms deadline:
/// 8044 lines and 528 KB in ten seconds — 4.6 GB/day from a single node, on a
/// robot that has thirty. The information content is one line: this node is
/// late, this often, by this much.
///
/// Emission is allowed at most once per `WINDOW` per kind. Everything in
/// between is counted, and the count rides along on the next line that is
/// allowed through, so nothing is hidden — only compressed.
#[derive(Debug, Default, Clone)]
pub(crate) struct DiagThrottle {
    /// Last emission per kind, as nanos since the process-wide clock epoch.
    last_ns: [u64; 4],
    /// Occurrences suppressed since that emission, per kind.
    suppressed: [u32; 4],
}

#[cfg(test)]
mod diag_throttle_tests {
    use super::{Diag, DiagThrottle};

    const SEC: u64 = 1_000_000_000;

    #[test]
    fn first_occurrence_is_never_delayed() {
        let mut t = DiagThrottle::default();
        // A fault must appear the moment it happens, not up to a window later.
        assert_eq!(t.allow(Diag::DeadlineMiss, 0), Some(0));
    }

    #[test]
    fn suppressed_occurrences_are_counted_and_reported() {
        let mut t = DiagThrottle::default();
        assert_eq!(t.allow(Diag::DeadlineMiss, SEC), Some(0));
        for i in 1..=500 {
            assert_eq!(t.allow(Diag::DeadlineMiss, SEC + i), None);
        }
        // The next line through carries the count, so throttling hides the
        // volume of output but never the volume of the fault.
        assert_eq!(t.allow(Diag::DeadlineMiss, SEC * 2), Some(500));
        // And the counter resets after being reported.
        assert_eq!(t.allow(Diag::DeadlineMiss, SEC * 3), Some(0));
    }

    #[test]
    fn kinds_do_not_suppress_each_other() {
        let mut t = DiagThrottle::default();
        assert_eq!(t.allow(Diag::DeadlineMiss, SEC), Some(0));
        // A node missing deadlines must still be able to report that it is also
        // overrunning its budget — different faults, different remedies.
        assert_eq!(t.allow(Diag::BudgetViolation, SEC), Some(0));
        assert_eq!(t.allow(Diag::SafeStateEnter, SEC), Some(0));
        assert_eq!(t.allow(Diag::SafeStateLeave, SEC), Some(0));
        // ...and each is independently throttled thereafter.
        assert_eq!(t.allow(Diag::DeadlineMiss, SEC + 1), None);
        assert_eq!(t.allow(Diag::BudgetViolation, SEC + 1), None);
    }

    #[test]
    fn throttle_state_is_per_node_not_per_call_site() {
        // The reason this type exists rather than `hlog_every!`: that macro
        // keeps its timestamp in a `static` at the expansion point, so on a
        // multi-node robot the first node to report silences the rest.
        let (mut a, mut b) = (DiagThrottle::default(), DiagThrottle::default());
        assert_eq!(a.allow(Diag::DeadlineMiss, SEC), Some(0));
        assert_eq!(b.allow(Diag::DeadlineMiss, SEC), Some(0));
    }
}

impl DiagThrottle {
    /// One line per second per kind. Fast enough that an operator watching a
    /// terminal sees the fault appear immediately, slow enough that a 1 kHz
    /// node cannot outrun the log.
    const WINDOW_NS: u64 = 1_000_000_000;

    /// Returns `Some(suppressed_since_last)` when the caller should emit.
    ///
    /// Takes `now_ns` rather than reading the clock so the RT caller can reuse
    /// a timestamp it already has, and so tests are deterministic.
    pub(crate) fn allow(&mut self, kind: Diag, now_ns: u64) -> Option<u32> {
        let i = kind as usize;
        let last = self.last_ns[i];
        // `last == 0` is the first occurrence: always let it through, so a fault
        // is never delayed by up to a second the first time it happens.
        if last != 0 && now_ns.saturating_sub(last) < Self::WINDOW_NS {
            self.suppressed[i] = self.suppressed[i].saturating_add(1);
            return None;
        }
        self.last_ns[i] = now_ns;
        Some(std::mem::take(&mut self.suppressed[i]))
    }
}

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
    /// Cycle counter for the suppressed-node probe tick.
    ///
    /// `Unhealthy` and `Isolated` were absorbing states: recovery is only
    /// evaluated after a tick, and the tick gate refused those states. A node
    /// degraded once could never return, even though
    /// `NodeDegradationState::recovery_counter` documents recovery as intended.
    pub(crate) health_probe_counter: u64,
    pub(crate) is_paused: bool,
    /// Rate limiting for this node's own RT diagnostics.
    ///
    /// Per node, not per call site: `hlog_every!` keeps its counter in a
    /// `static` at the macro's expansion point, so on a multi-node robot one
    /// node's message silences every other node's — which is worse than no
    /// throttle, because the survivor looks like the only node in trouble.
    pub(crate) diag: DiagThrottle,
    /// Whether `Miss::SafeMode` has already safed this node.
    ///
    /// `enter_safe_state()` is a state *transition* hook, but the SafeMode
    /// branch called it on every deadline miss with nothing recording that the
    /// node was already safed. Measured on a node sleeping 5 ms against a 1 ms
    /// deadline: 17 ticks, 18 `enter_safe_state()` calls — the node re-entered
    /// its safe state every single cycle.
    ///
    /// For the policy's own documented example ("Motor controller: enter safe
    /// mode on deadline miss") that means the motor is commanded by `tick()`
    /// and re-safed, alternating, for as long as the overload lasts. The hook
    /// now fires once on the way in, and the latch clears when the node starts
    /// meeting its deadline again.
    pub(crate) in_safe_mode: bool,
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

impl RegisteredNode {
    /// Whether this node may tick this cycle. Returns `false` when its failure
    /// policy has put it in a backoff (Restart) or cooldown (Skip) window.
    #[inline]
    pub(crate) fn failure_policy_allows_tick(&self) -> bool {
        self.failure_handler
            .as_ref()
            .is_none_or(|h| h.should_allow())
    }

    /// Record a successful tick with the failure policy (clears backoff/cooldown).
    #[inline]
    pub(crate) fn record_tick_success(&mut self) {
        if let Some(ref mut h) = self.failure_handler {
            h.record_success();
        }
    }

    /// Apply this node's failure policy after its `tick()` panicked. Returns
    /// `true` if the scheduler must STOP (Fatal, or Restart limit exhausted).
    ///
    /// - `Fatal` / `FatalAfterRestarts`: safe the faulted node immediately
    ///   (`enter_safe_state`, panic-guarded) and signal stop. The caller stops
    ///   the run loop, whose teardown then calls `shutdown()` on every node.
    /// - `Restart`: re-run `init()` (panic-guarded — it runs on partial state
    ///   and may itself panic or double-open a device).
    /// - `Skip` / `Continue`: nothing here; `failure_policy_allows_tick()` gates
    ///   subsequent ticks during the cooldown/backoff.
    ///
    /// Runs on the post-panic path (already off the no-alloc tick hot path), so
    /// the allocation `init()` may do under `Restart` is acceptable here.
    pub(crate) fn apply_failure_policy_after_panic(&mut self) -> bool {
        let action = match self.failure_handler {
            Some(ref mut h) => h.record_failure(),
            None => return false, // no policy → legacy log-and-continue
        };
        match action {
            FailureAction::StopScheduler | FailureAction::FatalAfterRestarts => {
                let _ = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
                    self.node.enter_safe_state();
                }));
                true
            }
            FailureAction::RestartNode => {
                let _ = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
                    let _ = self.node.init();
                }));
                false
            }
            FailureAction::SkipNode | FailureAction::Continue => false,
        }
    }
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
    /// Last time a message was observed on this topic (nanoseconds since epoch).
    ///
    /// Refreshed by [`SubscriptionFreshness::refresh`], which the scheduler
    /// calls before every staleness check.
    pub last_received_ns: AtomicU64,
    /// Lazily-mapped topic header, used to observe the publisher's
    /// `messages_total` counter. `None` once we have decided the topic is not
    /// mappable; re-attempted while it is still absent, because a topic may be
    /// created after the subscriber starts.
    counter_map: Mutex<Option<horus_sys::shm::ShmRegion>>,
    /// Last `messages_total` we observed, to detect that new data arrived.
    last_count: AtomicU64,
}

impl SubscriptionFreshness {
    /// Build a freshness tracker, stamped as of `now_ns`.
    pub fn new(topic: String, timeout: Duration, policy: StalePolicy, now_ns: u64) -> Self {
        Self {
            topic,
            timeout,
            policy,
            last_received_ns: AtomicU64::new(now_ns),
            counter_map: Mutex::new(None),
            last_count: AtomicU64::new(u64::MAX), // sentinel: nothing observed yet
        }
    }

    /// Observe the topic and refresh `last_received_ns` if new data has arrived.
    ///
    /// # Why this exists
    ///
    /// `last_received_ns` was stamped once at `build()` and **never written
    /// again** — the field's own doc claimed the scheduler updated it on
    /// `Topic::recv()`, and nothing did. So `.subscribe_with_timeout()`, whose
    /// entire purpose is to detect a topic that has gone quiet, degenerated
    /// into a fixed countdown from startup that fires on a perfectly healthy
    /// topic. With `StalePolicy::SafeState` or `Stop` that means a node enters
    /// its safe state, or halts, *because data is flowing normally*.
    ///
    /// The scheduler cannot see a user's `Topic::recv()` call — it happens
    /// inside `tick()`. But every publisher increments `messages_total` in the
    /// topic's SHM header on every send, so watching that counter is a true
    /// "new data exists" signal, and reading it is one relaxed atomic load from
    /// an already-mapped page.
    ///
    /// A topic that has never been created is deliberately NOT treated as
    /// fresh: no data has arrived, which is exactly what the timeout is for.
    pub fn refresh(&self, now_ns: u64) {
        use crate::communication::topic::shm_layout as layout;

        let mut guard = match self.counter_map.lock() {
            Ok(g) => g,
            Err(poisoned) => poisoned.into_inner(),
        };
        if guard.is_none() {
            // Retry while absent — the publisher may start after we do.
            if let Ok(map) =
                horus_sys::shm::ShmRegion::open_existing(&self.topic, layout::HEADER_SIZE)
            {
                let bytes = map.as_slice();
                let magic = u64::from_ne_bytes(
                    bytes[layout::OFF_MAGIC..layout::OFF_MAGIC + 8]
                        .try_into()
                        .unwrap_or([0; 8]),
                );
                if magic == layout::MAGIC {
                    *guard = Some(map);
                }
            }
        }
        let Some(ref map) = *guard else { return };

        // SAFETY: OFF_MESSAGES_TOTAL is inside the header, which we validated is
        // mapped, and the field is 8-byte aligned in a page-aligned mapping. The
        // publisher writes it as an AtomicU64, so it must be read as one.
        let total = unsafe {
            let ptr = map.as_ptr().add(layout::OFF_MESSAGES_TOTAL) as *const AtomicU64;
            (*ptr).load(Ordering::Relaxed)
        };

        let previous = self.last_count.swap(total, Ordering::Relaxed);
        // u64::MAX is the "nothing observed yet" sentinel: adopt the current
        // count without claiming data arrived, so a subscriber that starts long
        // after the publisher does not get a free refresh.
        if previous != u64::MAX && total != previous {
            // Windows' SystemTime may have coarser resolution than the nominal
            // nanosecond representation. A real publication can therefore be
            // observed while `now_ns` is identical to the previous stamp. Keep
            // the freshness clock logically monotonic so new data is always
            // distinguishable from no data, without moving it backwards when a
            // wall clock is adjusted.
            let last = self.last_received_ns.load(Ordering::Relaxed);
            self.last_received_ns
                .store(now_ns.max(last.saturating_add(1)), Ordering::Relaxed);
        }
    }
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
    /// Clock shared into executor threads so they can install the per-tick
    /// thread-local tick context (`horus::now/dt/elapsed/rng/budget_remaining`)
    /// exactly as the main loop's `run_node_tick` does. Without this, executor-
    /// run nodes saw inert fallbacks (FIX #5). Same `Arc<dyn Clock>` the
    /// scheduler ticks on, so sim/real time is consistent across threads.
    pub clock: Arc<dyn crate::core::clock::Clock>,
    /// Scheduler global tick period (`self.tick.period`) — the `node_dt`
    /// fallback for rateless nodes, identical to the value `run_node_tick` uses.
    pub tick_period: Duration,
    /// Watchdog feed handle sharing the scheduler `SafetyMonitor`'s watchdog map
    /// (`None` when no safety monitor is configured). Executor threads feed
    /// their critical nodes here each cycle; without it a healthy executor-run
    /// RT node was never fed and its watchdog spuriously e-stopped the fleet
    /// (FIX #2). Same Arc the main-thread `check_watchdogs` reads.
    pub watchdog: Option<super::safety_monitor::WatchdogFeeder>,
    /// E-stop latch handle sharing the scheduler `SafetyMonitor`'s state
    /// (`None` when no safety monitor is configured).
    ///
    /// Counterpart to `watchdog`. Executor threads own their nodes and have no
    /// `SafetyMonitor` reference, so the RT executor's emergency-stop branches
    /// previously did only `running.store(false)` — a plain shutdown flag. The
    /// latch was never set, `get_state()` kept reporting Normal, no blackbox
    /// EmergencyStop was written, and `PENDING_LOCAL_ESTOP` was never populated,
    /// so horus_net never broadcast: a real RT e-stop halted this robot silently
    /// and never told the fleet.
    pub estop: Option<super::safety_monitor::EstopTrigger>,
    /// The scheduler's `SafetyMonitor` itself, for timing AGGREGATION.
    ///
    /// `watchdog` and `estop` are narrow handles; deadline and degradation
    /// accounting is stateful across ticks and across nodes, so the executor
    /// needs the monitor.
    ///
    /// Without it, `max_deadline_misses` and the graduated degradation ladder
    /// were dead code on any normally-configured robot. Both live in
    /// `check_timing_violations`, gated on `self.nodes[i].is_rt_node` — but the
    /// class partition MOVES every RT node out of `self.nodes` into this
    /// executor, leaving only BestEffort nodes behind, for which the gate is
    /// false. So the ceiling only ever counted in deterministic/replay mode,
    /// while `.rate()` — the very thing that makes a node RT — guaranteed it
    /// would not.
    pub safety: Option<std::sync::Arc<super::safety_monitor::SafetyMonitor>>,
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
    /// Graduated watchdog health, mirrored here for EVERY node.
    ///
    /// `RegisteredNode.health_state` only exists for nodes the scheduler still
    /// owns, and after the class partition that is BestEffort nodes alone.
    /// The watchdog ladder runs on the main thread — the only one a hung node
    /// cannot block — so it needs somewhere to record the health of a node
    /// living on an executor. This map is registered for all five groups.
    pub health: AtomicHealthState,
    /// Set by the main thread when the ladder reaches Critical on a node the
    /// scheduler does not own; cleared by the executor that does, which then
    /// calls `enter_safe_state()` on it.
    ///
    /// Safing needs `&mut dyn Node`, which lives in the executor thread, so it
    /// cannot be a direct call from the ladder. The limit this leaves is real
    /// and deliberate: a node hung INSIDE `tick()` blocks the very thread that
    /// would honour the request, so it will not be safed by its own executor.
    /// That case is what the e-stop escalation exists for.
    pub safe_state_requested: std::sync::atomic::AtomicBool,
}

impl NodeControlMap {
    pub fn register(&self, name: &str) {
        let mut map = self.inner.write().unwrap_or_else(|e| e.into_inner());
        map.entry(name.to_string()).or_insert_with(|| NodeControl {
            paused: std::sync::atomic::AtomicBool::new(false),
            stopped: std::sync::atomic::AtomicBool::new(false),
            health: AtomicHealthState::default(),
            safe_state_requested: std::sync::atomic::AtomicBool::new(false),
        });
    }

    /// Record a node's graduated-watchdog health. Unknown names are ignored.
    pub fn set_health(&self, name: &str, state: NodeHealthState) {
        if let Some(c) = self
            .inner
            .read()
            .unwrap_or_else(|e| e.into_inner())
            .get(name)
        {
            c.health.store(state);
        }
    }

    /// Read a node's graduated-watchdog health. Unknown names read `Healthy`.
    pub fn health(&self, name: &str) -> NodeHealthState {
        self.inner
            .read()
            .unwrap_or_else(|e| e.into_inner())
            .get(name)
            .map(|c| c.health.load())
            .unwrap_or(NodeHealthState::Healthy)
    }

    /// Ask the executor that owns `name` to safe the node on its next pass.
    pub fn request_safe_state(&self, name: &str) {
        if let Some(c) = self
            .inner
            .read()
            .unwrap_or_else(|e| e.into_inner())
            .get(name)
        {
            c.safe_state_requested
                .store(true, std::sync::atomic::Ordering::Release);
        }
    }

    /// Consume a pending safing request, returning whether one was set.
    ///
    /// Clears the flag, so a request is honoured exactly once per raise.
    pub fn take_safe_state_request(&self, name: &str) -> bool {
        self.inner
            .read()
            .unwrap_or_else(|e| e.into_inner())
            .get(name)
            .map(|c| {
                c.safe_state_requested
                    .swap(false, std::sync::atomic::Ordering::AcqRel)
            })
            .unwrap_or(false)
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

    #[allow(dead_code)]
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

#[cfg(test)]
mod subscription_freshness_tests {
    use super::*;
    use crate::communication::Topic;

    fn now_ns() -> u64 {
        std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap()
            .as_nanos() as u64
    }

    fn unique(tag: &str) -> String {
        format!("freshness_{}_{}", tag, std::process::id())
    }

    fn last(sf: &SubscriptionFreshness) -> u64 {
        sf.last_received_ns.load(Ordering::Relaxed)
    }

    /// `last_received_ns` was stamped once at `build()` and never written again,
    /// while the field's own doc claimed the scheduler updated it on
    /// `Topic::recv()`. So a stale-data safety watchdog became a fixed countdown
    /// from startup that fired on a HEALTHY topic — and with SafeState/Stop,
    /// halted the node for it.
    #[test]
    fn publishing_refreshes_the_freshness_timestamp() {
        let name = unique("live");
        let topic: Topic<u64> = Topic::new(&name).expect("topic");

        let sf = SubscriptionFreshness::new(
            name.clone(),
            Duration::from_millis(50),
            StalePolicy::Warn,
            now_ns(),
        );
        sf.refresh(now_ns()); // adopt the current counter
        let baseline = last(&sf);

        std::thread::sleep(Duration::from_millis(5));
        topic.send(42);
        sf.refresh(now_ns());

        assert!(
            last(&sf) > baseline,
            "a published message must refresh the timestamp (was {baseline}, now {})",
            last(&sf)
        );
    }

    /// The end-to-end property: publishing faster than the timeout must never be
    /// judged stale. Before the fix this failed as soon as `timeout` elapsed
    /// from construction, no matter how much traffic there was.
    #[test]
    fn a_healthy_topic_never_exceeds_its_timeout() {
        let name = unique("healthy");
        let topic: Topic<u64> = Topic::new(&name).expect("topic");
        let timeout = Duration::from_millis(60);

        let sf = SubscriptionFreshness::new(name.clone(), timeout, StalePolicy::Warn, now_ns());
        sf.refresh(now_ns());

        for i in 0..10u64 {
            std::thread::sleep(Duration::from_millis(20));
            topic.send(i);
            let t = now_ns();
            sf.refresh(t);
            let elapsed = t.saturating_sub(last(&sf));
            assert!(
                elapsed <= timeout.as_nanos() as u64,
                "iteration {i}: a topic published every 20ms was judged stale against a \
                 60ms timeout (elapsed {}ms)",
                elapsed / 1_000_000
            );
        }
    }

    /// The converse — a genuinely quiet topic must STILL be detected, or the fix
    /// would have traded a false positive for a false negative.
    #[test]
    fn a_silent_topic_does_go_stale() {
        let name = unique("silent");
        let _topic: Topic<u64> = Topic::new(&name).expect("topic");
        let timeout = Duration::from_millis(30);

        let sf = SubscriptionFreshness::new(name.clone(), timeout, StalePolicy::Warn, now_ns());
        sf.refresh(now_ns());

        std::thread::sleep(Duration::from_millis(90));
        let t = now_ns();
        sf.refresh(t); // no traffic — must not refresh
        assert!(
            t.saturating_sub(last(&sf)) > timeout.as_nanos() as u64,
            "a topic with no traffic must still be detected as stale"
        );
    }

    /// A topic no publisher ever created has delivered no data — exactly what
    /// the timeout is for. It must not be masked as fresh.
    #[test]
    fn a_topic_that_does_not_exist_yet_is_not_treated_as_fresh() {
        let sf = SubscriptionFreshness::new(
            unique("never_created"),
            Duration::from_millis(20),
            StalePolicy::Warn,
            now_ns(),
        );
        std::thread::sleep(Duration::from_millis(60));
        let t = now_ns();
        sf.refresh(t);
        assert!(
            t.saturating_sub(last(&sf)) > Duration::from_millis(20).as_nanos() as u64,
            "an absent topic must not be reported fresh"
        );
    }
}
