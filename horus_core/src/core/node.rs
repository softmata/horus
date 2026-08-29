use std::fmt;
use std::sync::atomic::{AtomicBool, AtomicU32, Ordering};
use std::sync::{Arc, Mutex};
use std::time::Instant;

/// Compact logging summary for message types.
///
/// Used by verbose content logging and `hlog!` to produce one-line summaries
/// without cloning large data structures.
///
/// # Three ways to get `LogSummary`
///
/// **1. `message!` macro** — auto-generated, field-by-field formatting:
/// ```rust,ignore
/// message! {
///     SensorReading { temperature: f64, humidity: f64 }
/// }
/// // Produces: "SensorReading(temperature=23.5, humidity=0.65)"
/// ```
///
/// **2. `#[derive(LogSummary)]`** — uses `Debug` output (needs `#[derive(Debug)]`).
/// Requires the `macros` feature: `horus = { version = "0.1", features = ["macros"] }`
/// ```rust,ignore
/// #[derive(Debug, LogSummary)]
/// pub struct MyType { pub x: f64 }
/// // Produces: "MyType { x: 23.5 }"
/// ```
///
/// **3. Manual `impl`** — for custom compact summaries (large/zero-copy types):
/// ```rust,ignore
/// impl LogSummary for Image {
///     fn log_summary(&self) -> String {
///         format!("Image({}x{}, {:?})", self.width(), self.height(), self.encoding())
///     }
/// }
/// ```
///
/// # When to use which
///
/// | Approach | Use when |
/// |----------|----------|
/// | `message!` | Defining new message types (default) |
/// | `#[derive(LogSummary)]` | Existing `#[repr(C)]` types with `Debug` |
/// | Manual `impl` | Large types where `Debug` would be too verbose |
pub trait LogSummary {
    /// Return a compact one-line string suitable for logging.
    fn log_summary(&self) -> String;
}

impl LogSummary for crate::types::Tensor {
    fn log_summary(&self) -> String {
        let shape_str: Vec<String> = self.shape().iter().map(|d| d.to_string()).collect();
        format!(
            "Tensor([{}], dtype={:?}, device={}, pool={}/slot={})",
            shape_str.join(", "),
            self.dtype(),
            self.device(),
            self.pool_id,
            self.slot_id
        )
    }
}

/// Node states for monitoring and lifecycle management
#[derive(Debug, Clone, PartialEq)]
pub enum NodeState {
    Uninitialized,
    Initializing,
    Running,
    Stopping,
    Stopped,
    Error(String),
    Crashed(String),
}

impl fmt::Display for NodeState {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            NodeState::Uninitialized => write!(f, "Uninitialized"),
            NodeState::Initializing => write!(f, "Initializing"),
            NodeState::Running => write!(f, "Running"),
            NodeState::Stopping => write!(f, "Stopping"),
            NodeState::Stopped => write!(f, "Stopped"),
            NodeState::Error(msg) => write!(f, "Error: {}", msg),
            NodeState::Crashed(msg) => write!(f, "Crashed: {}", msg),
        }
    }
}

/// Node health status for monitoring
#[derive(Debug, Clone, Copy, PartialEq, Eq, serde::Serialize, serde::Deserialize, Default)]
#[repr(u8)]
pub enum HealthStatus {
    /// Operating normally
    Healthy = 0,
    /// Degraded performance (slow ticks, missed deadlines)
    Warning = 1,
    /// Errors occurring but still running
    Error = 2,
    /// Fatal errors, about to crash or unresponsive
    Critical = 3,
    /// Status unknown (no heartbeat received)
    #[default]
    Unknown = 4,
}

impl fmt::Display for HealthStatus {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(self.as_str())
    }
}

impl HealthStatus {
    /// Convert to string representation
    pub fn as_str(&self) -> &'static str {
        match self {
            Self::Healthy => "Healthy",
            Self::Warning => "Warning",
            Self::Error => "Error",
            Self::Critical => "Critical",
            Self::Unknown => "Unknown",
        }
    }

    /// Get color code for monitor display
    pub fn color(&self) -> &'static str {
        match self {
            Self::Healthy => "green",
            Self::Warning => "yellow",
            Self::Error => "orange",
            Self::Critical => "red",
            Self::Unknown => "gray",
        }
    }
}

impl LogSummary for HealthStatus {
    fn log_summary(&self) -> String {
        format!("{:?}", self)
    }
}

/// Performance metrics for node execution
#[derive(Debug, Clone, Default)]
pub struct NodeMetrics {
    name: String,
    order: u32,
    total_ticks: u64,
    successful_ticks: u64,
    failed_ticks: u64,
    avg_tick_duration_ms: f64,
    max_tick_duration_ms: f64,
    min_tick_duration_ms: f64,
    last_tick_duration_ms: f64,
    messages_sent: u64,
    messages_received: u64,
    errors_count: u64,
    warnings_count: u64,
    uptime_seconds: f64,
}

impl NodeMetrics {
    /// Create metrics with a name and execution order.
    pub(crate) fn new(name: String, order: u32) -> Self {
        Self {
            name,
            order,
            min_tick_duration_ms: f64::MAX,
            ..Default::default()
        }
    }

    /// Create a snapshot copy with an overridden name and order.
    pub(crate) fn snapshot(&self, name: String, order: u32) -> Self {
        Self {
            name,
            order,
            total_ticks: self.total_ticks,
            successful_ticks: self.successful_ticks,
            failed_ticks: self.failed_ticks,
            avg_tick_duration_ms: self.avg_tick_duration_ms,
            max_tick_duration_ms: self.max_tick_duration_ms,
            min_tick_duration_ms: self.min_tick_duration_ms,
            last_tick_duration_ms: self.last_tick_duration_ms,
            messages_sent: self.messages_sent,
            messages_received: self.messages_received,
            errors_count: self.errors_count,
            warnings_count: self.warnings_count,
            uptime_seconds: self.uptime_seconds,
        }
    }

    /// Node name.
    pub fn name(&self) -> &str {
        &self.name
    }

    /// Execution order (lower = earlier in tick sequence).
    pub fn order(&self) -> u32 {
        self.order
    }

    pub fn total_ticks(&self) -> u64 {
        self.total_ticks
    }

    pub fn successful_ticks(&self) -> u64 {
        self.successful_ticks
    }

    pub fn failed_ticks(&self) -> u64 {
        self.failed_ticks
    }

    pub fn avg_tick_duration_ms(&self) -> f64 {
        self.avg_tick_duration_ms
    }

    pub fn max_tick_duration_ms(&self) -> f64 {
        self.max_tick_duration_ms
    }

    pub fn min_tick_duration_ms(&self) -> f64 {
        self.min_tick_duration_ms
    }

    pub fn last_tick_duration_ms(&self) -> f64 {
        self.last_tick_duration_ms
    }

    pub fn messages_sent(&self) -> u64 {
        self.messages_sent
    }

    pub fn messages_received(&self) -> u64 {
        self.messages_received
    }

    pub fn errors_count(&self) -> u64 {
        self.errors_count
    }

    pub fn warnings_count(&self) -> u64 {
        self.warnings_count
    }

    pub fn uptime_seconds(&self) -> f64 {
        self.uptime_seconds
    }
}

impl NodeMetrics {
    /// Reset timing-related metrics for restart (preserves counts)
    #[allow(dead_code)]
    pub(crate) fn reset_timing(&mut self) {
        self.avg_tick_duration_ms = 0.0;
        self.max_tick_duration_ms = 0.0;
        self.min_tick_duration_ms = f64::MAX;
        self.last_tick_duration_ms = 0.0;
    }

    /// Calculate health status from metrics
    pub(crate) fn calculate_health(&self) -> HealthStatus {
        if self.errors_count > 10 {
            HealthStatus::Critical
        } else if self.errors_count > 3 {
            HealthStatus::Error
        } else if self.failed_ticks > 0 || self.avg_tick_duration_ms > 100.0 {
            HealthStatus::Warning
        } else {
            HealthStatus::Healthy
        }
    }
}

/// Internal scheduler context for tracking node state and metrics.
///
/// This is an internal type used by the scheduler and Python bindings.
/// Users should not construct or interact with `NodeInfo` directly.
#[doc(hidden)]
pub struct NodeInfo {
    // Identity
    name: String,

    // State management
    state: NodeState,
    previous_state: NodeState,
    state_change_time: Instant,

    // Performance tracking
    metrics: NodeMetrics,

    // Timing information
    creation_time: Instant,
    last_tick_time: Option<Instant>,
    tick_start_time: Option<Instant>,

    // Lifecycle management
    #[allow(dead_code)]
    restart_count: u32,
    error_history: Vec<(Instant, String)>,
    warning_history: Vec<(Instant, String)>,

    // Custom data store
    custom_data: std::collections::HashMap<String, String>,

    // Thread safety for metrics updates
    metrics_lock: Arc<Mutex<()>>,

    // Event notification channel — bumped by publishers to trigger event-driven
    // nodes, and (unlike the bare counter this used to be) able to *wake* the
    // watcher thread parked on it. Held here so the registry entry outlives any
    // single publish.
    event_notifier: Option<Arc<EventNotifier>>,

    // Consecutive failed ticks, reset by any success.
    //
    // Node health was driven only by the watchdog/deadline ladder, so a node
    // that panicked on every tick but never missed a *timing* target reported
    // `Health: Healthy` indefinitely. This is the counter that lets a failing
    // node be distinguished from a slow one.
    consecutive_failures: u32,
}

impl NodeInfo {
    /// Create a new NodeInfo
    #[doc(hidden)]
    pub fn new(node_name: String) -> Self {
        let now = Instant::now();

        Self {
            name: node_name,
            state: NodeState::Uninitialized,
            previous_state: NodeState::Uninitialized,
            state_change_time: now,
            metrics: NodeMetrics::new(String::new(), 0),
            creation_time: now,
            last_tick_time: None,
            tick_start_time: None,
            restart_count: 0,
            error_history: Vec::new(),
            warning_history: Vec::new(),
            custom_data: std::collections::HashMap::new(),
            metrics_lock: Arc::new(Mutex::new(())),
            event_notifier: None,
            consecutive_failures: 0,
        }
    }

    // State Management Methods
    pub fn state(&self) -> &NodeState {
        &self.state
    }

    pub(crate) fn set_state(&mut self, new_state: NodeState) {
        if self.state != new_state {
            self.previous_state = self.state.clone();
            self.state = new_state;
            self.state_change_time = Instant::now();
        }
    }

    #[doc(hidden)]
    pub fn transition_to_error(&mut self, error_msg: String) {
        crate::hlog!(error, "{}", error_msg);
        self.track_error(&error_msg);
        self.set_state(NodeState::Error(error_msg));
    }

    pub(crate) fn transition_to_crashed(&mut self, crash_msg: String) {
        crate::hlog!(error, "{}", crash_msg);
        self.track_error(&crash_msg);
        self.set_state(NodeState::Crashed(crash_msg));
    }

    pub(crate) fn transition_to_stopped(&mut self) {
        self.set_state(NodeState::Stopped);
    }

    // Lifecycle Methods
    pub(crate) fn initialize(&mut self) -> crate::error::HorusResult<()> {
        self.set_state(NodeState::Initializing);
        // Initialization logic can be added here
        self.set_state(NodeState::Running);
        Ok(())
    }

    /// Reset node context for restart (preserves identity, clears runtime state)
    #[allow(dead_code)]
    pub(crate) fn reset_for_restart(&mut self) {
        self.restart_count += 1;
        self.state = NodeState::Uninitialized;
        self.previous_state = NodeState::Stopped;
        self.state_change_time = Instant::now();
        self.last_tick_time = None;
        self.tick_start_time = None;
        // Keep metrics history but reset tick timing
        self.metrics.reset_timing();
    }

    // Tick Management
    #[doc(hidden)]
    pub fn start_tick(&mut self) {
        self.tick_start_time = Some(Instant::now());
        if self.state == NodeState::Uninitialized {
            let _ = self.initialize();
        }
    }

    #[doc(hidden)]
    /// Consecutive failed ticks; any successful tick resets this to zero.
    pub fn consecutive_failures(&self) -> u32 {
        self.consecutive_failures
    }

    pub fn record_tick(&mut self) {
        self.consecutive_failures = 0;
        let _guard = self
            .metrics_lock
            .lock()
            .unwrap_or_else(|poisoned| poisoned.into_inner());

        if let Some(start_time) = self.tick_start_time {
            let duration = start_time.elapsed();
            // `as_millis() as f64` TRUNCATES: every tick under 1 ms recorded
            // exactly 0.0. HORUS budgets are quoted in microseconds, so that is
            // essentially every RT node — avg/max/last/min tick duration all
            // read 0.0, and the documented 100 ms Warning threshold could never
            // be approached from below.
            let duration_ms = duration.as_secs_f64() * 1000.0;

            self.metrics.total_ticks += 1;
            self.metrics.successful_ticks += 1;
            self.metrics.last_tick_duration_ms = duration_ms;

            // Update min/max duration
            if duration_ms < self.metrics.min_tick_duration_ms {
                self.metrics.min_tick_duration_ms = duration_ms;
            }
            if duration_ms > self.metrics.max_tick_duration_ms {
                self.metrics.max_tick_duration_ms = duration_ms;
            }

            // Update average duration
            let total_duration =
                self.metrics.avg_tick_duration_ms * (self.metrics.successful_ticks - 1) as f64;
            self.metrics.avg_tick_duration_ms =
                (total_duration + duration_ms) / self.metrics.successful_ticks as f64;

            self.last_tick_time = Some(Instant::now());
            self.tick_start_time = None;

            // Update uptime
            self.metrics.uptime_seconds = self.creation_time.elapsed().as_secs_f64();
        }
    }

    /// Record node shutdown
    pub(crate) fn record_shutdown(&mut self) {
        self.transition_to_stopped();
    }

    pub(crate) fn record_tick_failure(&mut self, error_msg: String) {
        {
            let _guard = self
                .metrics_lock
                .lock()
                .unwrap_or_else(|poisoned| poisoned.into_inner());
            self.metrics.total_ticks += 1;
            self.metrics.failed_ticks += 1;
            self.consecutive_failures = self.consecutive_failures.saturating_add(1);

            if let Some(start_time) = self.tick_start_time {
                let duration = start_time.elapsed();
                // Sub-millisecond ticks truncated to 0.0 — see the success path.
                self.metrics.last_tick_duration_ms = duration.as_secs_f64() * 1000.0;
                self.tick_start_time = None;
            }
        }

        // Log against this node's own name rather than the ambient thread-local.
        //
        // `hlog!` reads `CURRENT_NODE`, which is empty on an executor thread —
        // and is also treated as empty once `tick_start` is cleared, which this
        // function has just done. So a node panic, the single most important
        // entry in the log, was recorded against node "unknown": it printed
        // `[ERROR] [unknown] Node 'boom' panicked: ...`, and `horus log --node
        // boom` did not return it. The name is right here.
        crate::core::hlog::log_as_node(crate::core::LogType::Error, &self.name, &error_msg);
        self.track_error(&error_msg);
    }

    /// Get elapsed time since tick started in microseconds
    pub fn tick_elapsed_us(&self) -> u64 {
        self.tick_start_time
            .map(|t| t.elapsed().as_micros() as u64)
            .unwrap_or(0)
    }

    /// Track a warning for metrics (history + count). Use hlog!(warn, ...) for logging.
    pub fn track_warning(&mut self, message: &str) {
        self.warning_history
            .push((Instant::now(), message.to_string()));
        if self.warning_history.len() > 100 {
            self.warning_history.remove(0);
        }
        self.metrics.warnings_count += 1;
    }

    /// Track an error for metrics (history + count). Use hlog!(error, ...) for logging.
    pub fn track_error(&mut self, message: &str) {
        self.error_history
            .push((Instant::now(), message.to_string()));
        if self.error_history.len() > 100 {
            self.error_history.remove(0);
        }
        self.metrics.errors_count += 1;
    }

    // Getters
    pub fn name(&self) -> &str {
        &self.name
    }
    pub fn metrics(&self) -> &NodeMetrics {
        &self.metrics
    }
    pub fn uptime(&self) -> std::time::Duration {
        self.creation_time.elapsed()
    }

    // Custom data management
    pub fn set_custom_data(&mut self, key: String, value: String) {
        self.custom_data.insert(key, value);
    }

    pub fn get_custom_data(&self, key: &str) -> Option<&String> {
        self.custom_data.get(key)
    }

    pub fn remove_custom_data(&mut self, key: &str) -> Option<String> {
        self.custom_data.remove(key)
    }

    pub fn custom_data_keys(&self) -> Vec<&str> {
        self.custom_data.keys().map(|k| k.as_str()).collect()
    }

    // Event notification methods

    /// Set the event notifier and register it under both the node name and topic name.
    ///
    /// This allows `Topic::send()` to notify event nodes by topic name,
    /// since Topic knows its own name but not which event nodes subscribe to it.
    pub(crate) fn set_event_notifier_with_topic(
        &mut self,
        notifier: Arc<EventNotifier>,
        topic_name: &str,
    ) {
        let mut registry = EVENT_NOTIFIER_REGISTRY
            .lock()
            .unwrap_or_else(|p| p.into_inner());
        // Register under both node name and topic name
        registry.insert(self.name.clone(), notifier.clone());
        registry.insert(topic_name.to_string(), notifier.clone());
        // Open the publish-side gate for both keys, while still holding the
        // registry lock. `notify_event` reads this word with a plain Relaxed
        // load and only takes the lock when a bit it cares about is set, so the
        // bits MUST be published no earlier than the entries they stand for.
        //
        // Ordering: the filter carries no data, it only gates the lock. A
        // reader that observes a bit then acquires this same mutex, whose
        // release below happens-after both `insert`s — so the entries are
        // visible to it. A reader that observes the bit as still clear returns
        // `false`, which is the same answer it would have got by winning the
        // race for the lock a moment before this registration took it; that
        // window is pre-existing and is exactly why callers such as
        // `wiring_verification::event_node_ticks_via_notify_event` spin on the
        // return value until registration lands.
        EVENT_NOTIFIER_FILTER.fetch_or(
            event_filter_bit(&self.name) | event_filter_bit(topic_name),
            std::sync::atomic::Ordering::Release,
        );
        drop(registry);
        self.event_notifier = Some(notifier);
    }

    /// Notify an event node by name (bumps its generation counter).
    ///
    /// This is the primary way to trigger an event-driven node from external code
    /// or from a topic publisher. Returns `true` if the node was found.
    ///
    /// # Why this is gated
    ///
    /// `Topic::send()` calls this on **every publish of every topic**, so the
    /// unconditional body — lock a process-global `Mutex`, probe a
    /// `HashMap<String, _>` — sat in the primary publish API of the runtime.
    /// Two costs, and the second is the one that matters:
    ///
    /// 1. The lock/probe is pure waste in any process that registers no event
    ///    node at all, which is the common configuration.
    /// 2. `std::sync::Mutex` is a futex with **no priority inheritance**. A
    ///    SCHED_FIFO publisher that arrives while a preempted SCHED_OTHER
    ///    thread holds this lock blocks for as long as the low-priority thread
    ///    takes to be scheduled again — unbounded, and unanalysable to a static
    ///    WCET tool. Nothing about the publish is allowed to depend on it.
    ///
    /// The gate is `EVENT_NOTIFIER_FILTER`, a 64-bit summary of the names
    /// present in the registry. Publishing costs one Relaxed load and a
    /// predictable not-taken branch; the lock is reached only for a name whose
    /// bit is set. False positives (two names sharing a bit) cost a lock
    /// acquire and a miss — the same answer, more slowly. False negatives are
    /// impossible: `event_filter_bit` is a pure function of the name and the
    /// bit is published under the registry lock at registration.
    ///
    /// This narrows the priority inversion from "every publish in every
    /// process" to "a publish on a name an event node is registered under".
    /// It does not eliminate it for that name — see the "Residual blocking
    /// edge" note on `EVENT_NOTIFIER_REGISTRY`.
    #[inline]
    pub fn notify_event(node_name: &str) -> bool {
        let filter = EVENT_NOTIFIER_FILTER.load(std::sync::atomic::Ordering::Relaxed);
        // No event node has ever registered in this process: no bit, no probe,
        // no lock. The mask test alone would cover this (`0 & x == 0`); the
        // explicit compare is what lets the common path skip even computing the
        // bit, leaving a load, a compare and a not-taken branch.
        if filter == 0 || filter & event_filter_bit(node_name) == 0 {
            return false;
        }
        Self::notify_event_registered(node_name)
    }

    /// Slow path of [`NodeInfo::notify_event`]: a name whose filter bit is set,
    /// so the registry has to be consulted. Outlined so the gate above inlines
    /// into `Topic::send()` as a load and a branch.
    #[cold]
    #[inline(never)]
    fn notify_event_registered(node_name: &str) -> bool {
        let registry = EVENT_NOTIFIER_REGISTRY
            .lock()
            .unwrap_or_else(|p| p.into_inner());
        let Some(notifier) = registry.get(node_name) else {
            return false;
        };

        // Bump the generation under the lock — one atomic RMW, the same cost
        // this path always paid — and find out whether anybody is parked on it.
        if !notifier.publish() {
            // Nobody parked: the watcher is awake and ticking, or between
            // ticks, and will read the new generation on its own. This is the
            // common case and it costs the publisher exactly what it cost
            // before: no syscall. Research 1.16's one hard constraint is that
            // waking must never become unconditional on the publish path.
            return true;
        }

        // Somebody IS parked, so this publish owes a FUTEX_WAKE — without it
        // the waiter sleeps out its backstop and we are back to millisecond
        // event latency. Drop the registry lock FIRST: `wake_all` is a
        // syscall, and holding a process-global `Mutex` across one would widen
        // the residual priority-inversion edge documented on
        // `EVENT_NOTIFIER_REGISTRY` from a hash probe into a trip through the
        // kernel. The `Arc` clone that buys the lock release is two refcount
        // RMWs, paid only on the path that was about to enter the kernel
        // anyway.
        let notifier = Arc::clone(notifier);
        drop(registry);
        notifier.wake_all();
        true
    }
}

/// One event node's wake-up channel: a generation counter publishers bump, plus
/// the parked-waiter count that decides whether bumping it costs a syscall.
///
/// # Why a futex and not a `Condvar`
///
/// The watcher thread in `scheduling::event_executor` used to `sleep(1 ms)` and
/// re-read the counter, so an "event-driven" node reacted `uniform(0, 1 ms)`
/// after the publish that should have woken it — three to four orders of
/// magnitude above the ~300 ns the transport itself delivers, with the whole
/// 1 ms as jitter. Replacing that with a real blocking wait has three
/// constraints, and a futex is the only candidate that meets all of them:
///
/// * **Zero CPU when idle.** A spin or a sleep-poll is a wake-up per
///   millisecond per event node, forever, on a robot that is doing nothing.
/// * **Nothing extra on the publish side.** A `Condvar` needs an associated
///   `Mutex` that the publisher must also take — a second priority-inversion
///   edge on the exact path [`NodeInfo::notify_event`] just spent an effort
///   removing one from.
/// * **Survives a move into shared memory.** `Condvar`, `thread::park` and
///   `parking_lot` are process-private by construction. This counter *is*
///   process-private today (see the cross-process note on
///   [`EVENT_NOTIFIER_REGISTRY`]), but the fix for that is to relocate this
///   word into the topic's SHM segment, and a futex is the one primitive that
///   survives the move unchanged: its identity is the address.
///
/// # Layout
///
/// `generation` must stay a `u32`: `FUTEX_WAIT` compares exactly 32 bits at the
/// address it is handed. `#[repr(C)]` and the 64-byte alignment are for that
/// same future SHM move — the offsets have to be stable across separate
/// compilations, and the line must not be shared with unrelated hot data.
#[repr(C, align(64))]
pub(crate) struct EventNotifier {
    /// Bumped once per notification. The watcher ticks once per increment it
    /// missed, so this is a *generation*, not a flag: it is never reset, and
    /// wrap-around is the read side's problem (`wrapping_sub`, never `-`).
    generation: AtomicU32,
    /// Threads currently inside [`EventNotifier::wait_for_change`].
    ///
    /// This is the gate that keeps `FUTEX_WAKE` off the publish path. Zero —
    /// the case whenever the event node is awake and ticking, and the only
    /// case in a process whose event nodes are all busy — means the publisher
    /// bumps `generation` and returns without entering the kernel.
    waiters: AtomicU32,
}

impl EventNotifier {
    pub(crate) const fn new() -> Self {
        Self {
            generation: AtomicU32::new(0),
            waiters: AtomicU32::new(0),
        }
    }

    /// The current generation. Compare it against the last one you acted on
    /// with `wrapping_sub`: the counter is 32 bits and is allowed to wrap.
    #[inline]
    pub(crate) fn generation(&self) -> u32 {
        self.generation.load(Ordering::Acquire)
    }

    /// Publisher side. Bumps the generation; returns `true` if a waiter is
    /// parked and therefore owed a [`EventNotifier::wake_all`].
    ///
    /// Deliberately split from the wake so the caller can release the registry
    /// lock before the syscall.
    #[inline]
    fn publish(&self) -> bool {
        self.generation.fetch_add(1, Ordering::SeqCst);
        // SeqCst on both halves of this pair — publisher bumps then reads
        // `waiters`, waiter writes `waiters` then re-reads `generation` — is
        // what makes "no waiter, no syscall" provable: in the single total
        // order at least one of the two sees the other's write.
        //
        // A lost wake-up would still be impossible with a Relaxed load here,
        // because `FUTEX_WAIT` re-compares the word against the waiter's
        // expected value under the kernel's bucket lock and returns `EAGAIN`
        // if it has moved. That is the real backstop. This ordering is what
        // makes the fast path something you can reason about rather than
        // something that happens to work.
        self.waiters.load(Ordering::SeqCst) != 0
    }

    /// Wake every parked waiter *without* moving the generation.
    ///
    /// A waiter woken this way sees no new generation, re-tests its run flag
    /// and either exits or parks again — so the shutdown path can use this to
    /// stop an idle event node promptly without fabricating a tick for it.
    pub(crate) fn wake_all(&self) {
        futex::wake_all(&self.generation);
    }

    /// Park until `generation` leaves `expected`, `timeout` elapses, or
    /// `keep_waiting` reads `false` — whichever comes first. May also return
    /// spuriously; the caller re-reads the generation regardless.
    ///
    /// `timeout` is a **backstop, not the mechanism**. If a wake is ever lost —
    /// a platform with no futex, a notifier that moved to a mapping this
    /// process cannot be woken on, a publisher that skipped the syscall on a
    /// stale waiter count — the caller degrades to polling at `timeout`
    /// cadence. It can never degrade to a node that stops ticking.
    pub(crate) fn wait_for_change(
        &self,
        expected: u32,
        timeout: std::time::Duration,
        keep_waiting: &AtomicBool,
    ) {
        self.waiters.fetch_add(1, Ordering::SeqCst);
        // Both re-checks MUST come after the count above is published, or they
        // race the very events they exist to catch:
        //
        //   generation   — a publish between the caller's read and this point.
        //                  (`FUTEX_WAIT`'s own compare would also catch this;
        //                  the load just avoids the syscall.)
        //   keep_waiting — a `stop()` between the same two points. Whoever
        //                  stops us stores `false` and *then* wakes; if that
        //                  wake lands before we are parked it is a no-op, and
        //                  this load is what keeps us from parking anyway.
        //
        // Residual window: a stop that lands between this load and the kernel
        // entry below still costs one `timeout` of shutdown latency — which is
        // exactly what the unconditional `sleep(POLL_INTERVAL)` this replaced
        // cost on *every* shutdown. Never worse, usually far better.
        if self.generation.load(Ordering::SeqCst) == expected && keep_waiting.load(Ordering::SeqCst)
        {
            futex::wait(&self.generation, expected, timeout);
        }
        self.waiters.fetch_sub(1, Ordering::SeqCst);
    }
}

/// Futex shims for [`EventNotifier`].
///
/// Linux only. Everywhere else the wait degrades to the timed sleep the event
/// executor used to do unconditionally, so those platforms keep their old
/// wake latency — a missing optimisation, not a missing guarantee.
mod futex {
    use std::sync::atomic::AtomicU32;
    use std::time::Duration;

    /// `FUTEX_PRIVATE_FLAG` is correct **only while the futex word lives in
    /// process-private memory**, which it does today: an `Arc<EventNotifier>`
    /// on the heap, reachable through a process-local `static` registry. The
    /// flag lets the kernel key the wait queue on the mm rather than pinning
    /// the page, which is materially cheaper on both sides.
    ///
    /// If [`super::EventNotifier`] is ever relocated into a topic's shared
    /// memory segment to close the cross-process gap, this flag must be
    /// dropped. A private futex on a shared mapping does not fail loudly — it
    /// simply never wakes the other process, and the caller's backstop absorbs
    /// it as silent millisecond polling.
    #[cfg(target_os = "linux")]
    const PRIVATE: libc::c_int = libc::FUTEX_PRIVATE_FLAG;

    #[cfg(target_os = "linux")]
    pub(super) fn wait(word: &AtomicU32, expected: u32, timeout: Duration) {
        // Relative timeout, measured by the kernel against CLOCK_MONOTONIC, so
        // it cannot be dragged by an NTP step the way a CLOCK_REALTIME
        // deadline could.
        let ts = libc::timespec {
            tv_sec: timeout.as_secs().min(libc::time_t::MAX as u64) as libc::time_t,
            tv_nsec: timeout.subsec_nanos() as _,
        };
        // SAFETY: `word` is a live, naturally aligned `AtomicU32` borrowed for
        // the duration of the call. FUTEX_WAIT reads exactly 32 bits at that
        // address to compare against `expected` and otherwise uses it only as
        // a wait-queue key; `ts` is read before the syscall returns. Every
        // error it can report — EAGAIN (value already moved), ETIMEDOUT,
        // EINTR, ENOSYS — means "re-check and loop", which is what the caller
        // does unconditionally, so the return value carries nothing we act on.
        unsafe {
            libc::syscall(
                libc::SYS_futex,
                word.as_ptr(),
                libc::FUTEX_WAIT | PRIVATE,
                expected,
                &ts as *const libc::timespec,
            );
        }
    }

    #[cfg(target_os = "linux")]
    pub(super) fn wake_all(word: &AtomicU32) {
        // SAFETY: as above. FUTEX_WAKE uses the address purely as a key and
        // never dereferences it.
        unsafe {
            libc::syscall(
                libc::SYS_futex,
                word.as_ptr(),
                libc::FUTEX_WAKE | PRIVATE,
                libc::c_int::MAX,
            );
        }
    }

    #[cfg(not(target_os = "linux"))]
    pub(super) fn wait(_word: &AtomicU32, _expected: u32, timeout: Duration) {
        // No futex here: fall back to precisely the sleep this change removed
        // on Linux, so behaviour on these platforms is unchanged rather than
        // broken.
        std::thread::sleep(timeout);
    }

    #[cfg(not(target_os = "linux"))]
    pub(super) fn wake_all(_word: &AtomicU32) {}
}

/// Bit this name occupies in [`EVENT_NOTIFIER_FILTER`].
///
/// A pure, total function of the name — the only thing correctness needs, since
/// registration and lookup both go through it. Mixes length with the first and
/// last byte through a multiplicative constant and takes the top 6 bits, which
/// separates the names that actually differ in practice (`evt_a`/`evt_b`,
/// `lidar_scan`/`lidar_pose`) for the price of one multiply. Names agreeing on
/// all three inputs share a bit; that is a false positive, not a lost
/// notification.
#[inline(always)]
fn event_filter_bit(name: &str) -> u64 {
    let b = name.as_bytes();
    let first = *b.first().unwrap_or(&0) as u64;
    let last = *b.last().unwrap_or(&0) as u64;
    let mixed =
        ((b.len() as u64) | (first << 8) | (last << 16)).wrapping_mul(0x9E37_79B9_7F4A_7C15);
    1u64 << (mixed >> 58)
}

/// Publish-side gate for [`EVENT_NOTIFIER_REGISTRY`]: bit `event_filter_bit(k)`
/// is set for every key `k` ever registered.
///
/// Zero until the first event node registers, which is the whole point — a
/// process with no event nodes never touches the registry mutex on a publish.
/// Set-only, exactly like the registry itself (nothing is ever removed from
/// it), so the gate can never go stale in the unsafe direction: a bit stays set
/// after the entry that justified it, which costs a lock, never a missed tick.
///
/// Saturation is graceful: with enough distinct registered names the word fills
/// and every publish falls through to the lock — i.e. back to the old
/// behaviour, never worse.
static EVENT_NOTIFIER_FILTER: std::sync::atomic::AtomicU64 = std::sync::atomic::AtomicU64::new(0);

/// Global registry of event notifiers keyed by node name.
///
/// Allows external code (topic publishers, tests) to notify event-driven nodes
/// without holding a direct reference to their NodeInfo.
///
/// Entries are inserted and never removed, and a re-registration under an
/// existing key replaces that key's `Arc`. Both properties are relied on by
/// [`EVENT_NOTIFIER_FILTER`], which gates the publish path.
///
/// # Residual blocking edge
///
/// This is still a `std::sync::Mutex` — a futex with no priority inheritance —
/// and a publish on a name an event node is registered under still takes it.
/// For that one name the publish path remains unbounded under preemption. The
/// filter removes the inversion from every *other* publish, which is where it
/// had no business being at all; it does not make an event-driven topic's
/// publish WCET analysable.
///
/// Closing that requires the read side to be wait-free, and the obstacle is
/// ownership, not the data structure: the watcher thread in
/// `scheduling::event_executor` creates its own `AtomicU64` and hands it here,
/// so a re-registration must swap the value a publisher may be dereferencing —
/// which no lock-free map can do without a reclamation scheme. Invert that
/// (registry owns a stable counter per name, watcher asks for it) and the
/// registry becomes an append-only array of `(name, &'static AtomicU64)` a
/// publisher can scan without any lock, with a WCET linear in the number of
/// registered event nodes. That change is in the event executor, not here, and
/// it must also seed the watcher's `last_seen_generation` from the counter it
/// adopts, or a restarted scheduler replays every notification the previous
/// run delivered.
static EVENT_NOTIFIER_REGISTRY: std::sync::LazyLock<
    Mutex<std::collections::HashMap<String, Arc<EventNotifier>>>,
> = std::sync::LazyLock::new(|| Mutex::new(std::collections::HashMap::new()));

/// Topic metadata for monitoring and introspection
#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
pub struct TopicMetadata {
    pub topic_name: String,
    pub type_name: String,
}

/// Comprehensive trait for Horus nodes with full lifecycle support
///
/// # Logging
///
/// Use the `hlog!()` macro for logging within any lifecycle method:
///
/// ```ignore
/// use horus::hlog;
///
/// fn init(&mut self) -> HorusResult<()> {
///     hlog!(info, "Initializing...");
///     Ok(())
/// }
///
/// fn tick(&mut self) {
///     hlog!(debug, "Processing tick");
/// }
/// ```
pub trait Node: Send {
    /// Get the node's name (must be unique within a scheduler).
    ///
    /// Defaults to the struct's type name (e.g. `MotorController`).
    /// Override for a custom name — string literals and `&self.name` both work.
    fn name(&self) -> &str {
        let full = std::any::type_name::<Self>();
        match full.rfind("::") {
            Some(pos) => &full[pos + 2..],
            None => full,
        }
    }

    /// Initialize the node (called once at startup).
    ///
    /// Use `hlog!()` for logging instead of the old ctx parameter.
    fn init(&mut self) -> crate::error::HorusResult<()> {
        Ok(())
    }

    /// Main execution loop (called repeatedly)
    fn tick(&mut self);

    /// Shutdown the node (called once at cleanup).
    ///
    /// Use `hlog!()` for logging instead of the old ctx parameter.
    fn shutdown(&mut self) -> crate::error::HorusResult<()> {
        Ok(())
    }

    // **DEPRECATED**: Topic associations are now automatic.
    // publishers() and subscribers() removed — topics are auto-detected
    // via TopicNodeRegistry when Topic::new() is called during tick.

    /// Handle errors (called by the scheduler on tick failure).
    ///
    /// Override to add custom error recovery logic. The default does nothing.
    ///
    /// It used to log the error again. Every caller — all four executors and
    /// the main-thread path — calls `record_tick_failure` first, which already
    /// logs at error level to the console and to the buffer `horus log` reads,
    /// so the default put the identical text on screen a second time with a
    /// redundant "Node error: " in front of it. This is a recovery hook, not a
    /// reporting one; reporting is the scheduler's job and it does it.
    #[doc(hidden)]
    fn on_error(&mut self, _error: &str) {}

    /// Check if node is in safe state (for safety monitor).
    fn is_safe_state(&self) -> bool {
        true
    }

    /// Transition to safe state (for emergency stop).
    fn enter_safe_state(&mut self) {
        // Default: no-op
    }

    /// Called when a runtime parameter changes.
    ///
    /// **NOT yet wired (stub audit 2026-07-13):** the scheduler does not
    /// currently call this method — attaching `RuntimeParams` via
    /// `.with_params()` does not route `RuntimeParams::set()` to nodes, and
    /// there is no automatic rollback on reject. Overriding this today has no
    /// effect. To react to parameter changes now, register a callback via
    /// `RuntimeParams::on_change()`. Scheduler-mediated routing is future work.
    ///
    /// When wired, the intent is: the scheduler invokes this before the next
    /// tick when a parameter changes; the node inspects old/new values and may
    /// return `Err` to reject the change (value rolls back).
    ///
    /// Default: accept all changes silently.
    ///
    /// # Example
    ///
    /// ```rust,ignore
    /// fn on_parameter_change(
    ///     &mut self,
    ///     key: &str,
    ///     _old: Option<&serde_json::Value>,
    ///     new: &serde_json::Value,
    /// ) -> Result<(), String> {
    ///     if key == "max_speed" {
    ///         self.max_speed = new.as_f64().unwrap_or(1.0);
    ///     }
    ///     Ok(())
    /// }
    /// ```
    fn on_parameter_change(
        &mut self,
        _key: &str,
        _old_value: Option<&serde_json::Value>,
        _new_value: &serde_json::Value,
    ) -> Result<(), String> {
        Ok(()) // Default: accept all changes
    }
}

// LogSummary implementations for primitive types
impl LogSummary for f32 {
    fn log_summary(&self) -> String {
        format!("{:.3}", self)
    }
}

impl LogSummary for f64 {
    fn log_summary(&self) -> String {
        format!("{:.3}", self)
    }
}

impl LogSummary for i32 {
    fn log_summary(&self) -> String {
        self.to_string()
    }
}

impl LogSummary for i64 {
    fn log_summary(&self) -> String {
        self.to_string()
    }
}

impl LogSummary for u32 {
    fn log_summary(&self) -> String {
        self.to_string()
    }
}

impl LogSummary for u64 {
    fn log_summary(&self) -> String {
        self.to_string()
    }
}

impl LogSummary for usize {
    fn log_summary(&self) -> String {
        self.to_string()
    }
}

impl LogSummary for bool {
    fn log_summary(&self) -> String {
        self.to_string()
    }
}

impl LogSummary for String {
    fn log_summary(&self) -> String {
        self.clone()
    }
}

impl<T: fmt::Debug> LogSummary for Vec<T> {
    fn log_summary(&self) -> String {
        format!("Vec[{} items]", self.len())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    // =========================================================================
    // NodeState Tests
    // =========================================================================

    #[test]
    fn test_node_state_display() {
        assert_eq!(NodeState::Uninitialized.to_string(), "Uninitialized");
        assert_eq!(NodeState::Initializing.to_string(), "Initializing");
        assert_eq!(NodeState::Running.to_string(), "Running");
        assert_eq!(NodeState::Stopping.to_string(), "Stopping");
        assert_eq!(NodeState::Stopped.to_string(), "Stopped");
        assert_eq!(
            NodeState::Error("test error".to_string()).to_string(),
            "Error: test error"
        );
        assert_eq!(
            NodeState::Crashed("crash reason".to_string()).to_string(),
            "Crashed: crash reason"
        );
    }

    #[test]
    fn test_node_state_equality() {
        assert_eq!(NodeState::Running, NodeState::Running);
        assert_ne!(NodeState::Running, NodeState::Stopping);
        assert_eq!(
            NodeState::Error("msg".to_string()),
            NodeState::Error("msg".to_string())
        );
        assert_ne!(
            NodeState::Error("msg1".to_string()),
            NodeState::Error("msg2".to_string())
        );
    }

    #[test]
    fn test_node_state_clone() {
        let state = NodeState::Error("test".to_string());
        let cloned = state.clone();
        assert_eq!(state, cloned);
    }

    // =========================================================================
    // HealthStatus Tests
    // =========================================================================

    #[test]
    fn test_health_status_as_str() {
        assert_eq!(HealthStatus::Healthy.as_str(), "Healthy");
        assert_eq!(HealthStatus::Warning.as_str(), "Warning");
        assert_eq!(HealthStatus::Error.as_str(), "Error");
        assert_eq!(HealthStatus::Critical.as_str(), "Critical");
        assert_eq!(HealthStatus::Unknown.as_str(), "Unknown");
    }

    #[test]
    fn test_health_status_color() {
        assert_eq!(HealthStatus::Healthy.color(), "green");
        assert_eq!(HealthStatus::Warning.color(), "yellow");
        assert_eq!(HealthStatus::Error.color(), "orange");
        assert_eq!(HealthStatus::Critical.color(), "red");
        assert_eq!(HealthStatus::Unknown.color(), "gray");
    }

    #[test]
    fn test_health_status_equality() {
        assert_eq!(HealthStatus::Healthy, HealthStatus::Healthy);
        assert_ne!(HealthStatus::Healthy, HealthStatus::Warning);
    }

    #[test]
    fn test_health_status_copy() {
        let status = HealthStatus::Healthy;
        let copied = status;
        assert_eq!(status, copied);
    }

    // =========================================================================
    // NodeMetrics Tests
    // =========================================================================

    #[test]
    fn test_node_metrics_default() {
        let metrics = NodeMetrics::default();
        assert_eq!(metrics.total_ticks(), 0);
        assert_eq!(metrics.successful_ticks(), 0);
        assert_eq!(metrics.failed_ticks(), 0);
        assert_eq!(metrics.avg_tick_duration_ms(), 0.0);
        assert_eq!(metrics.messages_sent(), 0);
        assert_eq!(metrics.messages_received(), 0);
        assert_eq!(metrics.errors_count(), 0);
    }

    #[test]
    fn test_node_metrics_clone() {
        let mut metrics = NodeMetrics::new("test".to_string(), 0);
        metrics.total_ticks = 100;
        metrics.successful_ticks = 95;
        metrics.failed_ticks = 5;
        metrics.avg_tick_duration_ms = 10.5;
        metrics.max_tick_duration_ms = 50.0;
        metrics.min_tick_duration_ms = 1.0;
        metrics.last_tick_duration_ms = 8.0;
        metrics.messages_sent = 500;
        metrics.messages_received = 300;
        metrics.errors_count = 2;
        metrics.warnings_count = 10;
        metrics.uptime_seconds = 3600.0;

        let cloned = metrics.clone();
        assert_eq!(cloned.total_ticks(), 100);
        assert_eq!(cloned.successful_ticks(), 95);
        assert_eq!(cloned.avg_tick_duration_ms(), 10.5);
    }

    // =========================================================================
    // Health Calculation Tests
    // =========================================================================

    #[test]
    fn test_calculate_health_healthy() {
        let metrics = NodeMetrics {
            total_ticks: 100,
            successful_ticks: 100,
            failed_ticks: 0,
            avg_tick_duration_ms: 10.0,
            errors_count: 0,
            ..NodeMetrics::default()
        };
        assert_eq!(metrics.calculate_health(), HealthStatus::Healthy);
    }

    #[test]
    fn test_calculate_health_warning() {
        let metrics = NodeMetrics {
            failed_ticks: 5, // > 0 failed ticks triggers warning
            ..NodeMetrics::default()
        };
        assert_eq!(metrics.calculate_health(), HealthStatus::Warning);
    }

    #[test]
    fn test_calculate_health_error() {
        let metrics = NodeMetrics {
            errors_count: 5, // > 3 triggers error
            ..NodeMetrics::default()
        };
        assert_eq!(metrics.calculate_health(), HealthStatus::Error);
    }

    #[test]
    fn test_calculate_health_critical() {
        let metrics = NodeMetrics {
            errors_count: 15, // > 10 triggers critical
            ..NodeMetrics::default()
        };
        assert_eq!(metrics.calculate_health(), HealthStatus::Critical);
    }

    // =========================================================================
    // NodeInfo Tests
    // =========================================================================

    #[test]
    fn test_node_info_new() {
        let info = NodeInfo::new("test_node".to_string());
        assert_eq!(info.name(), "test_node");
        assert_eq!(info.state(), &NodeState::Uninitialized);
    }

    #[test]
    fn test_node_info_state_transitions() {
        let mut info = NodeInfo::new("test_node".to_string());

        assert_eq!(info.state(), &NodeState::Uninitialized);

        info.set_state(NodeState::Initializing);
        assert_eq!(info.state(), &NodeState::Initializing);

        info.set_state(NodeState::Running);
        assert_eq!(info.state(), &NodeState::Running);

        info.set_state(NodeState::Stopping);
        assert_eq!(info.state(), &NodeState::Stopping);

        info.set_state(NodeState::Stopped);
        assert_eq!(info.state(), &NodeState::Stopped);
    }

    #[test]
    fn test_node_info_metrics_initial() {
        let info = NodeInfo::new("test_node".to_string());
        let metrics = info.metrics();
        assert_eq!(metrics.total_ticks(), 0);
        assert_eq!(metrics.successful_ticks(), 0);
        assert_eq!(metrics.failed_ticks(), 0);
    }

    #[test]
    fn test_node_info_error_tracking() {
        let mut info = NodeInfo::new("test_node".to_string());

        info.track_error("Test error 1");
        info.track_error("Test error 2");

        let metrics = info.metrics();
        assert_eq!(metrics.errors_count(), 2);
    }

    #[test]
    fn test_node_info_transition_to_error() {
        let mut info = NodeInfo::new("test_node".to_string());
        info.transition_to_error("Something went wrong".to_string());
        assert!(matches!(info.state(), &NodeState::Error(_)));
    }

    #[test]
    fn test_node_info_transition_to_crashed() {
        let mut info = NodeInfo::new("test_node".to_string());
        info.transition_to_crashed("Fatal error".to_string());
        assert!(matches!(info.state(), &NodeState::Crashed(_)));
    }

    #[test]
    fn test_node_info_initialize() {
        let mut info = NodeInfo::new("test_node".to_string());
        assert_eq!(info.state(), &NodeState::Uninitialized);

        info.initialize().unwrap();
        assert_eq!(info.state(), &NodeState::Running);
    }

    // =========================================================================
    // LogSummary Tests
    // =========================================================================

    #[test]
    fn test_log_summary_f32() {
        assert_eq!(std::f32::consts::PI.log_summary(), "3.142");
        assert_eq!(0.0f32.log_summary(), "0.000");
    }

    #[test]
    fn test_log_summary_f64() {
        assert_eq!(std::f64::consts::PI.log_summary(), "3.142");
    }

    #[test]
    fn test_log_summary_integers() {
        assert_eq!(42i32.log_summary(), "42");
        assert_eq!(42i64.log_summary(), "42");
        assert_eq!(42u32.log_summary(), "42");
        assert_eq!(42u64.log_summary(), "42");
        assert_eq!(42usize.log_summary(), "42");
    }

    #[test]
    fn test_log_summary_bool() {
        assert_eq!(true.log_summary(), "true");
        assert_eq!(false.log_summary(), "false");
    }

    #[test]
    fn test_log_summary_string() {
        assert_eq!("hello".to_string().log_summary(), "hello");
    }

    #[test]
    fn test_log_summary_vec() {
        let v: Vec<i32> = vec![1, 2, 3, 4, 5];
        assert_eq!(v.log_summary(), "Vec[5 items]");

        let empty: Vec<i32> = vec![];
        assert_eq!(empty.log_summary(), "Vec[0 items]");
    }
}

#[cfg(test)]
mod sub_millisecond_metrics_tests {
    use std::time::Duration;

    /// Sub-millisecond ticks must not record as 0.0.
    ///
    /// `as_millis() as f64` truncates, so every tick under 1 ms read exactly
    /// 0.0 — and HORUS budgets are quoted in microseconds, so that is
    /// essentially every RT node. avg/max/last/min tick duration were all zero
    /// and the documented 100 ms Warning threshold could never be approached.
    #[test]
    fn sub_millisecond_durations_are_not_truncated_to_zero() {
        let to_ms = |d: Duration| d.as_secs_f64() * 1000.0;

        assert!(
            to_ms(Duration::from_micros(200)) > 0.0,
            "a 200us tick — a typical RT budget — must not record as 0.0"
        );
        assert!(
            (to_ms(Duration::from_micros(200)) - 0.2).abs() < 1e-9,
            "200us must be 0.2ms"
        );
        assert!(
            (to_ms(Duration::from_micros(1500)) - 1.5).abs() < 1e-9,
            "1500us must be 1.5ms, not 1.0"
        );

        // The old expression, for contrast — this is what was being stored.
        assert_eq!(
            Duration::from_micros(200).as_millis() as f64,
            0.0,
            "documents the defect: truncation made a 200us tick indistinguishable \
             from no work at all"
        );
    }
}

#[cfg(test)]
mod failure_attribution_tests {
    use super::*;
    use crate::core::log_buffer::GLOBAL_LOG_BUFFER;

    /// A node's own failure must be recorded against that node.
    ///
    /// `hlog!` reads the `CURRENT_NODE` thread-local, which is empty on an
    /// executor thread and is treated as empty once the tick's `tick_start` has
    /// been cleared — which `record_tick_failure` does a few lines before
    /// logging. So a panic, the single most important entry a node produces,
    /// was filed under node "unknown": the console printed
    /// `[ERROR] [unknown] Node 'boom' panicked: ...` and `horus log --node boom`
    /// returned nothing for it.
    #[test]
    fn a_tick_failure_is_logged_against_the_node_that_failed() {
        let name = format!("attrib_probe_{}", std::process::id());
        let mut info = NodeInfo::new(name.clone());
        let marker = format!("boom marker {}", std::process::id());

        info.record_tick_failure(marker.clone());

        let mine: Vec<_> = GLOBAL_LOG_BUFFER
            .get_all()
            .into_iter()
            .filter(|e| e.message.contains(&marker))
            .collect();

        assert!(
            !mine.is_empty(),
            "the failure never reached the log buffer at all"
        );
        assert!(
            mine.iter().all(|e| e.node_name == name),
            "a tick failure was filed under {:?} instead of {name:?} — \
             `horus log --node {name}` would not find the node's own panic",
            mine.iter().map(|e| &e.node_name).collect::<Vec<_>>()
        );
    }
}

#[cfg(test)]
mod event_notifier_gate_tests {
    use super::*;
    use std::time::Duration;

    /// The safety property of the publish-side gate: anything the registry
    /// holds must still be reachable through `notify_event`.
    ///
    /// `Topic::send()` no longer takes the registry lock unconditionally — it
    /// consults `EVENT_NOTIFIER_FILTER` first — so a registration that failed
    /// to open the gate would look exactly like a working system until an event
    /// node silently stopped ticking. This pins both keys the registration
    /// writes (node name AND topic name), because `Topic::send()` only ever
    /// looks up the topic one.
    #[test]
    fn registration_opens_the_gate_for_both_keys() {
        let node_name = format!("gate_node_{}", std::process::id());
        let topic_name = format!("gate_topic_{}", std::process::id());

        // Before registration: no entry, so no notification, and — the whole
        // point — no reason to have touched the registry mutex.
        assert!(
            !NodeInfo::notify_event(&node_name),
            "unregistered name must not report a delivered notification"
        );
        assert!(!NodeInfo::notify_event(&topic_name));

        let notifier = Arc::new(EventNotifier::new());
        let mut info = NodeInfo::new(node_name.clone());
        info.set_event_notifier_with_topic(notifier.clone(), &topic_name);

        let filter = EVENT_NOTIFIER_FILTER.load(Ordering::Relaxed);
        assert_ne!(
            filter & event_filter_bit(&node_name),
            0,
            "registration left the gate shut for the node name"
        );
        assert_ne!(
            filter & event_filter_bit(&topic_name),
            0,
            "registration left the gate shut for the topic name — \
             Topic::send() would never notify this event node"
        );

        assert!(NodeInfo::notify_event(&node_name));
        assert!(
            NodeInfo::notify_event(&topic_name),
            "the key Topic::send() uses did not resolve"
        );
        assert_eq!(
            notifier.generation(),
            2,
            "one notification per call — the watcher ticks once per generation"
        );
    }

    /// The point of the whole change: a publish must *wake* a parked waiter,
    /// not merely bump the counter and leave it to time out.
    ///
    /// The backstop here is 30 s, three orders of magnitude above the 1 ms the
    /// event executor uses, so the assertion cannot be satisfied by the
    /// timeout path — only by `notify_event` actually delivering a
    /// `FUTEX_WAKE` to the waiter. If the wake is ever dropped (a publisher
    /// that stops checking `waiters`, a private-vs-shared futex mismatch) this
    /// test takes 30 s and fails, rather than silently reverting event nodes
    /// to millisecond latency the way the old sleep-poll did.
    #[test]
    fn a_publish_wakes_a_parked_waiter_rather_than_letting_it_time_out() {
        let name = format!("wake_node_{}", std::process::id());
        let notifier = Arc::new(EventNotifier::new());
        let mut info = NodeInfo::new(name.clone());
        info.set_event_notifier_with_topic(Arc::clone(&notifier), &format!("{name}_topic"));

        let keep_waiting = Arc::new(AtomicBool::new(true));
        let waiter_notifier = Arc::clone(&notifier);
        let waiter_flag = Arc::clone(&keep_waiting);
        let start_gen = notifier.generation();

        let waiter = std::thread::spawn(move || {
            let t0 = Instant::now();
            waiter_notifier.wait_for_change(start_gen, Duration::from_secs(30), &waiter_flag);
            (t0.elapsed(), waiter_notifier.generation())
        });

        // Give the waiter time to actually reach the kernel, so this exercises
        // the wake path and not the pre-park generation re-check.
        std::thread::sleep(Duration::from_millis(50));
        assert!(
            NodeInfo::notify_event(&name),
            "the notifier registered above was not reachable through notify_event"
        );

        let (elapsed, generation) = waiter.join().expect("waiter thread panicked");
        assert_eq!(generation, start_gen.wrapping_add(1));
        assert!(
            elapsed < Duration::from_secs(5),
            "waiter took {elapsed:?} to return from a 30 s wait that was notified after \
             50 ms — the publish bumped the counter but never woke the futex, so event \
             nodes are back to waiting out their backstop"
        );
    }

    /// A wake that never arrives must degrade to the backstop, never to a hang.
    ///
    /// `publish()` is called directly and `wake_all()` deliberately is not,
    /// which is exactly the shape of every way a wake can go missing:
    /// a platform with no futex, a private futex on a shared mapping, a
    /// publisher that read a stale `waiters` count. The waiter must still come
    /// back — a lost wake-up is a latency event, never a stalled node.
    #[test]
    fn a_lost_wakeup_degrades_to_the_backstop_and_never_hangs() {
        let notifier = Arc::new(EventNotifier::new());
        let keep_waiting = Arc::new(AtomicBool::new(true));
        let backstop = Duration::from_millis(200);

        let waiter_notifier = Arc::clone(&notifier);
        let waiter_flag = Arc::clone(&keep_waiting);
        let waiter = std::thread::spawn(move || {
            let t0 = Instant::now();
            waiter_notifier.wait_for_change(0, backstop, &waiter_flag);
            t0.elapsed()
        });

        std::thread::sleep(Duration::from_millis(20));
        // Bump WITHOUT waking — simulate the lost wake-up.
        assert!(
            notifier.publish(),
            "the waiter should have been visible in the waiter count by now"
        );

        let elapsed = waiter.join().expect("waiter thread panicked");
        assert!(
            elapsed < backstop * 4,
            "waiter blocked {elapsed:?} against a {backstop:?} backstop — the timeout is \
             not bounding the wait, so a lost wake-up can stall an event node"
        );
    }

    /// Shutdown must not have to wait out the backstop.
    ///
    /// `wake_all` moves no generation, so the waiter wakes, sees the same
    /// counter, re-tests its run flag and leaves without fabricating a tick.
    #[test]
    fn clearing_the_run_flag_and_waking_releases_a_parked_waiter() {
        let notifier = Arc::new(EventNotifier::new());
        let keep_waiting = Arc::new(AtomicBool::new(true));

        let waiter_notifier = Arc::clone(&notifier);
        let waiter_flag = Arc::clone(&keep_waiting);
        let waiter = std::thread::spawn(move || {
            let t0 = Instant::now();
            waiter_notifier.wait_for_change(0, Duration::from_secs(30), &waiter_flag);
            (t0.elapsed(), waiter_notifier.generation())
        });

        std::thread::sleep(Duration::from_millis(50));
        keep_waiting.store(false, Ordering::SeqCst);
        notifier.wake_all();

        let (elapsed, generation) = waiter.join().expect("waiter thread panicked");
        assert_eq!(
            generation, 0,
            "the shutdown wake moved the generation — a stopping node would tick once more"
        );
        assert!(
            elapsed < Duration::from_secs(5),
            "waiter took {elapsed:?} to notice shutdown; EventExecutor::stop would block \
             for its whole join budget on every idle event node"
        );
    }

    /// A name the registry has never seen answers `false` without a lock. The
    /// return value is load-bearing: several tests spin on it to discover when
    /// the event executor's watcher thread has finished registering.
    #[test]
    fn unregistered_name_is_rejected_by_the_gate() {
        for name in ["", "x", "no_such_event_node_1a2b3c", "another/one"] {
            assert!(
                !NodeInfo::notify_event(name),
                "notify_event({name:?}) claimed to notify a node that was never registered"
            );
        }
    }

    /// The gate's correctness rests on `event_filter_bit` being a pure, total
    /// function that names exactly one bit — the same one at registration and
    /// at lookup. An empty name must not panic (it is a legal `&str` and
    /// reaches this from `notify_event`'s public API).
    #[test]
    fn filter_bit_is_pure_total_and_single() {
        let long = "z".repeat(300);
        for name in [
            "",
            "a",
            "evt_a",
            "evt_b",
            "lidar_scan",
            "/cmd_vel",
            long.as_str(),
        ] {
            let bit = event_filter_bit(name);
            assert_eq!(
                bit.count_ones(),
                1,
                "event_filter_bit({name:?}) must select exactly one of the 64 bits"
            );
            assert_eq!(bit, event_filter_bit(name), "event_filter_bit is not pure");
        }

        // Not a correctness requirement — a false positive only costs a lock —
        // but names that differ only in their last byte are the common shape
        // (`evt_a`/`evt_b`, `cam_0`/`cam_1`), so they had better not all pile
        // onto one bit and hand every publisher the mutex back.
        let spread: u64 = ["evt_a", "evt_b", "evt_c", "evt_d"]
            .into_iter()
            .map(event_filter_bit)
            .fold(0, |acc, b| acc | b);
        assert!(
            spread.count_ones() >= 3,
            "sibling names collapsed onto {} bit(s); the filter would not gate them apart",
            spread.count_ones()
        );
    }
}
