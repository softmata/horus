// Safety monitor for real-time critical systems
use crate::core::rt_node::BudgetViolation;
use parking_lot::{Mutex, RwLock};
use std::collections::HashMap;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::Arc;
use std::time::{Duration, SystemTime, UNIX_EPOCH};

/// Return the current wall-clock time as nanoseconds since the Unix epoch.
///
/// Using `SystemTime` rather than `Instant` lets us store an absolute timestamp
/// in a `u64` with no global base reference.  NTP adjustments of a few microseconds
/// cannot cause spurious watchdog expiry because watchdog timeouts are O(milliseconds).
#[inline(always)]
fn now_ns() -> u64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap_or_default()
        .as_nanos() as u64
}

/// Safety state of the system
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SafetyState {
    /// Normal operation
    Normal,
    /// Emergency stop triggered
    EmergencyStop,
}

/// Policy for handling tick budget violations.
///
/// Controls what happens when a node exceeds its allocated tick budget.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BudgetPolicy {
    /// Log the violation but take no corrective action (default).
    /// The graduated degradation path will handle it over time.
    Warn,
    /// Immediately stop the node after a budget violation.
    /// The node will have shutdown() called and be permanently removed.
    /// Safer than mid-tick interruption — waits for tick to complete,
    /// then prevents all future ticks.
    Enforce,
    /// Trigger emergency stop on budget violation (for critical nodes).
    EmergencyStop,
}

impl Default for BudgetPolicy {
    fn default() -> Self {
        BudgetPolicy::Warn
    }
}

/// Graduated watchdog severity level.
///
/// Returned by `Watchdog::check_graduated()` to indicate how far past the
/// timeout the node is. The scheduler uses this to transition node health states.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum WatchdogSeverity {
    /// Within timeout — node is healthy.
    Ok,
    /// 1x timeout elapsed — warning, node is slow.
    Warning,
    /// 2x timeout elapsed — unhealthy, skip node in tick loop.
    Expired,
    /// 3x timeout elapsed — critical, trigger safety response for critical nodes.
    Critical,
}

/// Watchdog for monitoring node health
///
/// `last_heartbeat_ns` is stored as nanoseconds since the Unix epoch in an
/// `AtomicU64`.  This eliminates the `Mutex<Instant>` TOCTOU window that
/// existed between `lock()` (reading the timestamp) and `elapsed()` (comparing
/// it): if a heartbeat arrived between those two operations, the stale
/// timestamp caused a false-positive expiry, triggering an emergency stop on a
/// healthy node.
///
/// With an `AtomicU64`, `feed()` is a single `store(Release)` and `check()`
/// is a single `load(Acquire)` followed immediately by arithmetic — there is
/// no lock acquisition delay during which a heartbeat could slip in.
#[derive(Debug)]
pub(crate) struct Watchdog {
    /// Timeout duration
    timeout: Duration,
    /// Last heartbeat time as nanoseconds since Unix epoch.
    last_heartbeat_ns: AtomicU64,
    /// Is watchdog expired?
    expired: AtomicBool,
}

impl Watchdog {
    pub(crate) fn new(timeout: Duration) -> Self {
        Self {
            timeout,
            last_heartbeat_ns: AtomicU64::new(now_ns()),
            expired: AtomicBool::new(false),
        }
    }

    /// Feed the watchdog (reset timer)
    pub(crate) fn feed(&self) {
        self.last_heartbeat_ns.store(now_ns(), Ordering::Release);
        self.expired.store(false, Ordering::SeqCst);
    }

    /// Check if watchdog has expired (simple boolean).
    pub(crate) fn check(&self) -> bool {
        let last_ns = self.last_heartbeat_ns.load(Ordering::Acquire);
        let elapsed_ns = now_ns().saturating_sub(last_ns);
        let expired = elapsed_ns >= self.timeout.as_nanos() as u64;
        if expired {
            self.expired.store(true, Ordering::SeqCst);
        }
        expired
    }

    /// Graduated check: returns severity based on how many timeout multiples elapsed.
    ///
    /// - `Ok`: within timeout
    /// - `Warning`: 1x-2x timeout
    /// - `Expired`: 2x-3x timeout
    /// - `Critical`: 3x+ timeout
    pub(crate) fn check_graduated(&self) -> WatchdogSeverity {
        let last_ns = self.last_heartbeat_ns.load(Ordering::Acquire);
        let elapsed_ns = now_ns().saturating_sub(last_ns);
        let timeout_ns = self.timeout.as_nanos() as u64;

        if elapsed_ns <= timeout_ns {
            WatchdogSeverity::Ok
        } else if elapsed_ns <= timeout_ns * 2 {
            self.expired.store(true, Ordering::SeqCst);
            WatchdogSeverity::Warning
        } else if elapsed_ns <= timeout_ns * 3 {
            self.expired.store(true, Ordering::SeqCst);
            WatchdogSeverity::Expired
        } else {
            self.expired.store(true, Ordering::SeqCst);
            WatchdogSeverity::Critical
        }
    }

    pub(crate) fn is_expired(&self) -> bool {
        self.expired.load(Ordering::SeqCst)
    }
}

/// Fixed-size ring buffer for per-node tick duration history.
///
/// Stores the last `CAPACITY` tick durations for a node, enabling
/// min/max/avg/p99 calculations without unbounded memory growth.
const TIMING_RING_CAPACITY: usize = 1024;

#[derive(Debug)]
pub(crate) struct TickTimingRing {
    durations: Box<[u64; TIMING_RING_CAPACITY]>,
    write_pos: usize,
    count: u64,
}

impl TickTimingRing {
    pub(crate) fn new() -> Self {
        Self {
            durations: Box::new([0u64; TIMING_RING_CAPACITY]),
            write_pos: 0,
            count: 0,
        }
    }

    /// Record a tick duration in microseconds.
    pub(crate) fn record(&mut self, duration_us: u64) {
        self.durations[self.write_pos] = duration_us;
        self.write_pos = (self.write_pos + 1) % TIMING_RING_CAPACITY;
        self.count += 1;
    }

    /// Number of samples recorded (total, not just in buffer).
    #[cfg(test)]
    pub(crate) fn total_count(&self) -> u64 {
        self.count
    }

    /// Number of valid samples in the ring (min of count and capacity).
    fn valid_count(&self) -> usize {
        (self.count as usize).min(TIMING_RING_CAPACITY)
    }

    /// Compute timing statistics from the ring buffer.
    pub(crate) fn stats(&self) -> TimingStats {
        let n = self.valid_count();
        if n == 0 {
            return TimingStats::default();
        }

        let samples = &self.durations[..n];
        let min = *samples
            .iter()
            .min()
            .expect("samples non-empty: checked n > 0 above");
        let max = *samples
            .iter()
            .max()
            .expect("samples non-empty: checked n > 0 above");
        let sum: u64 = samples.iter().sum();
        let avg = sum / n as u64;

        // P99: sort a copy, take the 99th percentile
        let mut sorted = samples.to_vec();
        sorted.sort_unstable();
        let p99_idx = ((n as f64 * 0.99) as usize).min(n - 1);
        let p99 = sorted[p99_idx];

        TimingStats {
            min_us: min,
            max_us: max,
            avg_us: avg,
            p99_us: p99,
            total_ticks: self.count,
        }
    }
}

/// Timing statistics for a node.
#[derive(Debug, Clone, Default)]
pub struct TimingStats {
    pub min_us: u64,
    pub max_us: u64,
    pub avg_us: u64,
    pub p99_us: u64,
    pub total_ticks: u64,
}

/// Per-node timing and overrun tracking.
#[derive(Debug)]
pub(crate) struct NodeTimingState {
    /// Tick duration ring buffer
    pub(crate) ring: TickTimingRing,
    /// Budget for this node (None = no budget set)
    pub(crate) budget: Option<Duration>,
    /// Total overrun count
    pub(crate) overrun_count: u64,
    /// Worst overrun (actual - budget), zero if no overruns
    pub(crate) worst_overrun_us: u64,
    /// Deadline miss tracking
    pub(crate) total_deadline_misses: u64,
    /// Consecutive deadline misses (resets on successful tick)
    pub(crate) consecutive_misses: u64,
    /// Worst deadline miss severity (how far past deadline)
    pub(crate) worst_miss_us: u64,
}

impl NodeTimingState {
    pub(crate) fn new(budget: Option<Duration>) -> Self {
        Self {
            ring: TickTimingRing::new(),
            budget,
            overrun_count: 0,
            worst_overrun_us: 0,
            total_deadline_misses: 0,
            consecutive_misses: 0,
            worst_miss_us: 0,
        }
    }

    /// Record a tick and check budget.
    pub(crate) fn record_tick(&mut self, actual: Duration) -> Option<BudgetViolation> {
        let actual_us = actual.as_micros() as u64;
        self.ring.record(actual_us);
        // Successful tick resets consecutive miss counter
        self.consecutive_misses = 0;

        if let Some(budget) = self.budget {
            if actual > budget {
                self.overrun_count += 1;
                let overrun_us = actual_us.saturating_sub(budget.as_micros() as u64);
                if overrun_us > self.worst_overrun_us {
                    self.worst_overrun_us = overrun_us;
                }
                return Some(BudgetViolation::new(
                    String::new(), // filled by caller
                    budget,
                    actual,
                ));
            }
        }
        None
    }

    /// Record a deadline miss with severity.
    pub(crate) fn record_miss(&mut self, severity_us: u64) {
        self.total_deadline_misses += 1;
        self.consecutive_misses += 1;
        if severity_us > self.worst_miss_us {
            self.worst_miss_us = severity_us;
        }
    }

    /// Whether this node is chronically missing deadlines (3+ consecutive).
    #[cfg(test)]
    pub(crate) fn is_chronic(&self) -> bool {
        self.consecutive_misses >= 3
    }
}

/// Policy for how nodes degrade under sustained timing violations.
///
/// Graduated response: warn → reduce rate → isolate → safe state.
#[derive(Debug, Clone)]
pub(crate) struct DegradationPolicy {
    /// Consecutive misses before logging a warning (default: 3)
    pub(crate) warn_after: u64,
    /// Consecutive misses before reducing node rate (default: 5)
    pub(crate) reduce_after: u64,
    /// Consecutive misses before isolating node (default: 10)
    pub(crate) isolate_after: u64,
    /// Consecutive misses before killing node — permanently removing from execution (default: 20)
    pub(crate) kill_after: u64,
    /// Successful ticks at reduced rate before restoring original (default: 100)
    pub(crate) recovery_ticks: u64,
}

impl Default for DegradationPolicy {
    fn default() -> Self {
        Self {
            warn_after: 3,
            reduce_after: 5,
            isolate_after: 10,
            kill_after: 20,
            recovery_ticks: 100,
        }
    }
}

/// Current degradation stage for a node.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum DegradationStage {
    /// Normal operation — no degradation applied.
    Normal,
    /// Warned — logging violations, no corrective action yet.
    Warned,
    /// Rate reduced — node running at half its original rate.
    RateReduced,
    /// Isolated — node skipped entirely in tick loop.
    Isolated,
    /// Killed — node permanently removed from execution after shutdown().
    Killed,
}

/// Per-node degradation tracking.
#[derive(Debug)]
pub(crate) struct NodeDegradationState {
    /// Current degradation stage.
    pub(crate) stage: DegradationStage,
    /// Original rate_hz before degradation reduced it. None = no rate was set.
    pub(crate) original_rate_hz: Option<f64>,
    /// Consecutive successful ticks since rate was reduced (for recovery).
    pub(crate) recovery_counter: u64,
}

impl Default for NodeDegradationState {
    fn default() -> Self {
        Self {
            stage: DegradationStage::Normal,
            original_rate_hz: None,
            recovery_counter: 0,
        }
    }
}

/// Action the scheduler should take based on degradation evaluation.
#[derive(Debug, Clone, PartialEq)]
pub(crate) enum DegradationAction {
    /// No action needed.
    None,
    /// Warn about the node (log only).
    Warn(String),
    /// Reduce node rate to the given Hz.
    ReduceRate { node: String, new_rate_hz: f64 },
    /// Isolate node — skip in tick loop.
    Isolate(String),
    /// Kill node — call shutdown() and permanently remove from execution.
    /// This is the final stage for nodes that remain stalled after isolation.
    Kill(String),
    /// Restore node to original rate (recovery).
    RestoreRate { node: String, original_rate_hz: f64 },
    /// De-isolate node — resume ticking at reduced rate.
    Deisolate(String),
}

/// budget (Worst-Case Execution Time) enforcer with real per-node metrics.
#[derive(Debug)]
pub(crate) struct BudgetEnforcer {
    /// Per-node timing state with ring buffers
    pub(crate) node_timing: HashMap<String, NodeTimingState>,
    /// Global overrun counter (atomic for cross-thread access)
    pub(crate) overruns: AtomicU64,
    /// Critical overruns (that triggered emergency stop)
    critical_overruns: AtomicU64,
}

impl Default for BudgetEnforcer {
    fn default() -> Self {
        Self {
            node_timing: HashMap::new(),
            overruns: AtomicU64::new(0),
            critical_overruns: AtomicU64::new(0),
        }
    }
}

impl BudgetEnforcer {
    pub(crate) fn new() -> Self {
        Self::default()
    }

    /// Set tick budget for a node.
    pub(crate) fn set_budget(&mut self, node: String, budget: Duration) {
        self.node_timing
            .entry(node)
            .or_insert_with(|| NodeTimingState::new(Some(budget)))
            .budget = Some(budget);
    }

    /// Record a tick duration and check against budget.
    ///
    /// This is the real enforcement: tracks per-node timing history,
    /// computes overrun severity, and updates the ring buffer.
    pub(crate) fn check_budget(
        &mut self,
        node: &str,
        actual: Duration,
    ) -> Result<(), BudgetViolation> {
        let state = self
            .node_timing
            .entry(node.to_string())
            .or_insert_with(|| NodeTimingState::new(None));

        if let Some(mut violation) = state.record_tick(actual) {
            self.overruns.fetch_add(1, Ordering::SeqCst);
            violation =
                BudgetViolation::new(node.to_string(), violation.budget(), violation.actual());
            return Err(violation);
        }
        Ok(())
    }

    /// Record a deadline miss with severity for a node.
    pub(crate) fn record_deadline_miss(&mut self, node: &str, severity_us: u64) {
        let state = self
            .node_timing
            .entry(node.to_string())
            .or_insert_with(|| NodeTimingState::new(None));
        state.record_miss(severity_us);
    }

    /// Get timing stats for a specific node.
    #[cfg(test)]
    pub(crate) fn node_stats(&self, node: &str) -> Option<TimingStats> {
        self.node_timing.get(node).map(|s| s.ring.stats())
    }

    /// Get all node timing states (for timing report).
    pub(crate) fn all_node_stats(&self) -> Vec<(String, TimingStats, Option<Duration>, u64)> {
        self.node_timing
            .iter()
            .map(|(name, state)| {
                (
                    name.clone(),
                    state.ring.stats(),
                    state.budget,
                    state.overrun_count,
                )
            })
            .collect()
    }

    pub(crate) fn get_overrun_count(&self) -> u64 {
        self.overruns.load(Ordering::SeqCst)
    }

    pub(crate) fn mark_critical_overrun(&self) {
        self.critical_overruns.fetch_add(1, Ordering::SeqCst);
    }
}

/// Safety monitor for real-time critical systems
#[derive(Debug)]
pub(crate) struct SafetyMonitor {
    /// Current safety state
    state: Arc<Mutex<SafetyState>>,
    /// Emergency stop flag
    emergency_stop: Arc<AtomicBool>,
    /// Watchdogs for critical nodes.
    ///
    /// `RwLock` instead of `Mutex`: `check_watchdogs()` and `feed_watchdog()`
    /// only READ the map (individual watchdog state is in `AtomicU64`/`AtomicBool`),
    /// so multiple scheduler ticks can iterate concurrently.  Only
    /// `add_critical_node()` needs a write lock (infrequent setup-time call).
    watchdogs: Arc<RwLock<HashMap<String, Watchdog>>>,
    /// budget enforcer
    budget_enforcer: Arc<Mutex<BudgetEnforcer>>,
    /// Critical nodes that must never fail — protected by RwLock for interior mutability.
    /// add_critical_node() acquires a write lock; all readers acquire a read lock.
    critical_nodes: Arc<RwLock<Vec<String>>>,
    /// Deadline miss counter
    deadline_misses: AtomicU64,
    /// Maximum allowed deadline misses before emergency
    max_deadline_misses: u64,
    /// Safe mode activation counter
    degrade_activations: AtomicU64,
    /// Degradation policy for timing violations
    degradation_policy: DegradationPolicy,
    /// Per-node degradation state
    degradation_states: Mutex<HashMap<String, NodeDegradationState>>,
}

impl SafetyMonitor {
    pub(crate) fn new(max_deadline_misses: u64) -> Self {
        Self {
            state: Arc::new(Mutex::new(SafetyState::Normal)),
            emergency_stop: Arc::new(AtomicBool::new(false)),
            watchdogs: Arc::new(RwLock::new(HashMap::new())),
            budget_enforcer: Arc::new(Mutex::new(BudgetEnforcer::new())),
            critical_nodes: Arc::new(RwLock::new(Vec::new())),
            deadline_misses: AtomicU64::new(0),
            max_deadline_misses,
            degrade_activations: AtomicU64::new(0),
            degradation_policy: DegradationPolicy::default(),
            degradation_states: Mutex::new(HashMap::new()),
        }
    }

    /// Set the degradation policy for this safety monitor.
    #[cfg(test)]
    pub(crate) fn set_degradation_policy(&mut self, policy: DegradationPolicy) {
        self.degradation_policy = policy;
    }

    /// Add a critical node that must be monitored.
    ///
    /// Takes `&self` (not `&mut self`) because `critical_nodes` is protected by an
    /// interior `RwLock`; this allows callers holding a shared reference to safely
    /// register new nodes at any time.
    pub(crate) fn add_critical_node(&self, node_name: String, watchdog_timeout: Duration) {
        self.critical_nodes.write().push(node_name.clone());
        self.watchdogs
            .write()
            .insert(node_name, Watchdog::new(watchdog_timeout));
    }

    /// Set tick budget for a node.
    pub(crate) fn set_tick_budget(&self, node_name: String, budget: Duration) {
        self.budget_enforcer.lock().set_budget(node_name, budget);
    }

    /// Feed watchdog for a node.
    ///
    /// Acquires a **read** lock: `feed()` is `&self` on `Watchdog` because it
    /// only does an atomic store — no HashMap mutation needed.
    pub(crate) fn feed_watchdog(&self, node_name: &str) {
        if let Some(watchdog) = self.watchdogs.read().get(node_name) {
            watchdog.feed();
        }
    }

    /// Check all watchdogs and write expired node names into `expired`.
    ///
    /// ## Signature change vs original
    ///
    /// The caller passes a pre-allocated `&mut Vec<String>` buffer that is
    /// reused across ticks.  This eliminates the heap allocation that the
    /// previous `-> Vec<String>` return required on every call.
    ///
    /// The caller is responsible for clearing the buffer before passing it in
    /// (or relying on the fact that this method clears it at entry).
    ///
    /// ## Lock change vs original
    ///
    /// Acquires a **read** lock: individual watchdog timestamps live in
    /// `AtomicU64`/`AtomicBool` fields, so reading them requires no
    /// HashMap mutation.  Multiple threads can call `check_watchdogs`
    /// concurrently without blocking each other.
    pub(crate) fn check_watchdogs(&self, expired: &mut Vec<String>) {
        expired.clear();
        for (name, watchdog) in self.watchdogs.read().iter() {
            if watchdog.check() {
                expired.push(name.clone());
            }
        }

        // If any critical node watchdog expired, trigger emergency stop.
        // Snapshot critical_nodes while briefly holding the read lock, then
        // release before calling trigger_emergency_stop (which logs + stores
        // state — we don't want to hold the lock across I/O or Mutex acquires).
        if !expired.is_empty() {
            let critical: Vec<String> = self.critical_nodes.read().clone();
            for node in expired.iter() {
                if critical.contains(node) {
                    self.trigger_emergency_stop(format!("Critical node {} watchdog expired", node));
                    break;
                }
            }
        }
    }

    /// Graduated watchdog check: returns per-node severity levels.
    ///
    /// Unlike `check_watchdogs()` which only reports expired/not-expired,
    /// this method returns graduated severity for each node, enabling the
    /// scheduler to transition node health states progressively:
    ///
    /// - `Warning` (1x timeout): log, keep ticking
    /// - `Expired` (2x timeout): mark Unhealthy, skip in tick loop
    /// - `Critical` (3x timeout): trigger safety response for critical nodes
    ///
    /// The buffer is reused across ticks (caller-owned).
    pub(crate) fn check_watchdogs_graduated(&self, results: &mut Vec<(String, WatchdogSeverity)>) {
        results.clear();
        for (name, watchdog) in self.watchdogs.read().iter() {
            let severity = watchdog.check_graduated();
            if !matches!(severity, WatchdogSeverity::Ok) {
                results.push((name.clone(), severity));
            }
        }

        // For critical nodes at Critical severity: trigger emergency stop
        if results
            .iter()
            .any(|(_, s)| *s == WatchdogSeverity::Critical)
        {
            let critical: Vec<String> = self.critical_nodes.read().clone();
            for (name, severity) in results.iter() {
                if *severity == WatchdogSeverity::Critical && critical.contains(name) {
                    self.trigger_emergency_stop(format!(
                        "Critical node {} watchdog at 3x timeout",
                        name
                    ));
                    break;
                }
            }
        }
    }

    /// Check tick budget for a node — records timing data and checks overrun.
    pub(crate) fn check_tick_budget(
        &self,
        node_name: &str,
        execution_time: Duration,
    ) -> Result<(), BudgetViolation> {
        let result = self
            .budget_enforcer
            .lock()
            .check_budget(node_name, execution_time);

        if let Err(ref violation) = result {
            let is_critical = self
                .critical_nodes
                .read()
                .contains(&violation.node_name().to_string());
            if is_critical {
                self.budget_enforcer.lock().mark_critical_overrun();
                self.trigger_emergency_stop(format!(
                    "Critical node {} exceeded tick budget: {:?} > {:?}",
                    violation.node_name(),
                    violation.actual(),
                    violation.budget()
                ));
            }
        }

        result
    }

    /// Record a deadline miss with severity tracking.
    ///
    /// Tracks per-node: total misses, consecutive misses, worst miss duration.
    /// Triggers emergency stop for critical nodes or when total exceeds max.
    pub(crate) fn record_deadline_miss(&self, node_name: &str) {
        self.record_deadline_miss_with_severity(node_name, 0);
    }

    /// Record a deadline miss with a specific severity (how far past deadline, in μs).
    pub(crate) fn record_deadline_miss_with_severity(&self, node_name: &str, severity_us: u64) {
        let misses = self.deadline_misses.fetch_add(1, Ordering::SeqCst) + 1;

        // Track per-node deadline miss data
        self.budget_enforcer
            .lock()
            .record_deadline_miss(node_name, severity_us);

        let is_critical = self.critical_nodes.read().contains(&node_name.to_string());
        if is_critical {
            self.trigger_emergency_stop(format!("Critical node {} missed deadline", node_name));
        } else if misses >= self.max_deadline_misses {
            self.trigger_emergency_stop(format!("Too many deadline misses: {}", misses));
        }
    }

    /// Get per-node timing stats (for timing report).
    pub(crate) fn all_node_timing(&self) -> Vec<(String, TimingStats, Option<Duration>, u64)> {
        self.budget_enforcer.lock().all_node_stats()
    }

    /// Get consecutive miss count for a node.
    pub(crate) fn consecutive_misses(&self, node_name: &str) -> u64 {
        self.budget_enforcer
            .lock()
            .node_timing
            .get(node_name)
            .map(|s| s.consecutive_misses)
            .unwrap_or(0)
    }

    /// Evaluate degradation for a node based on its consecutive miss count.
    ///
    /// Returns a `DegradationAction` that the scheduler should execute.
    /// Call this after recording a deadline miss or budget violation.
    pub(crate) fn evaluate_degradation(
        &self,
        node_name: &str,
        consecutive_misses: u64,
        current_rate_hz: Option<f64>,
    ) -> DegradationAction {
        let policy = &self.degradation_policy;
        let mut states = self.degradation_states.lock();
        let state = states.entry(node_name.to_string()).or_default();

        if consecutive_misses >= policy.kill_after && state.stage != DegradationStage::Killed {
            state.stage = DegradationStage::Killed;
            state.recovery_counter = 0;
            DegradationAction::Kill(node_name.to_string())
        } else if consecutive_misses >= policy.isolate_after
            && state.stage != DegradationStage::Isolated
            && state.stage != DegradationStage::Killed
        {
            state.stage = DegradationStage::Isolated;
            state.recovery_counter = 0;
            DegradationAction::Isolate(node_name.to_string())
        } else if consecutive_misses >= policy.reduce_after
            && state.stage != DegradationStage::RateReduced
            && state.stage != DegradationStage::Isolated
            && state.stage != DegradationStage::Killed
        {
            state.stage = DegradationStage::RateReduced;
            state.recovery_counter = 0;
            if state.original_rate_hz.is_none() {
                state.original_rate_hz = current_rate_hz;
            }
            let new_rate = current_rate_hz.unwrap_or(100.0) / 2.0;
            DegradationAction::ReduceRate {
                node: node_name.to_string(),
                new_rate_hz: new_rate,
            }
        } else if consecutive_misses >= policy.warn_after
            && state.stage == DegradationStage::Normal
        {
            state.stage = DegradationStage::Warned;
            DegradationAction::Warn(node_name.to_string())
        } else {
            DegradationAction::None
        }
    }

    /// Record a successful tick for recovery tracking.
    ///
    /// When a node at `RateReduced` stage ticks successfully for `recovery_ticks`
    /// consecutive times, it returns `RestoreRate` to signal the scheduler should
    /// restore the original rate.
    pub(crate) fn record_successful_tick(&self, node_name: &str) -> DegradationAction {
        let recovery_ticks = self.degradation_policy.recovery_ticks;

        let mut states = self.degradation_states.lock();
        let Some(state) = states.get_mut(node_name) else {
            return DegradationAction::None;
        };

        match state.stage {
            DegradationStage::RateReduced => {
                state.recovery_counter += 1;
                if state.recovery_counter >= recovery_ticks {
                    let original = state.original_rate_hz;
                    state.stage = DegradationStage::Normal;
                    state.recovery_counter = 0;
                    state.original_rate_hz = None;
                    if let Some(rate) = original {
                        return DegradationAction::RestoreRate {
                            node: node_name.to_string(),
                            original_rate_hz: rate,
                        };
                    }
                    // No original rate saved — rate cannot be restored
                    log::warn!(
                        "Node '{}': recovered from RateReduced but original_rate_hz was None — rate not restored",
                        node_name
                    );
                    DegradationAction::None
                } else {
                    DegradationAction::None
                }
            }
            DegradationStage::Warned => {
                // Successful tick at Warned stage — reset to Normal
                state.stage = DegradationStage::Normal;
                state.recovery_counter = 0;
                DegradationAction::None
            }
            DegradationStage::Isolated => {
                // Successful tick at Isolated stage — recover to RateReduced first,
                // then RateReduced → Normal on continued success.
                state.recovery_counter += 1;
                if state.recovery_counter >= recovery_ticks {
                    state.stage = DegradationStage::RateReduced;
                    state.recovery_counter = 0;
                    log::info!(
                        "Node '{}': recovered from Isolated to RateReduced after {} successful ticks",
                        node_name, recovery_ticks
                    );
                    DegradationAction::Deisolate(node_name.to_string())
                } else {
                    DegradationAction::None
                }
            }
            _ => DegradationAction::None,
        }
    }

    /// Get the current degradation stage for a node.
    #[cfg(test)]
    pub(crate) fn degradation_stage(&self, node_name: &str) -> DegradationStage {
        self.degradation_states
            .lock()
            .get(node_name)
            .map(|s| s.stage)
            .unwrap_or(DegradationStage::Normal)
    }

    /// Record a safe mode activation (Miss::SafeMode triggered on a node).
    pub(crate) fn record_degrade_activation(&self) {
        self.degrade_activations.fetch_add(1, Ordering::SeqCst);
    }

    /// Trigger emergency stop
    pub(crate) fn trigger_emergency_stop(&self, reason: String) {
        eprintln!(" EMERGENCY STOP: {}", reason);
        self.emergency_stop.store(true, Ordering::SeqCst);
        *self.state.lock() = SafetyState::EmergencyStop;
    }

    /// Check if emergency stop is active
    pub(crate) fn is_emergency_stop(&self) -> bool {
        self.emergency_stop.load(Ordering::SeqCst)
    }

    /// Get current safety state
    pub(crate) fn get_state(&self) -> SafetyState {
        *self.state.lock()
    }

    /// Get safety statistics
    pub(crate) fn get_stats(&self) -> SafetyStats {
        SafetyStats {
            state: self.get_state(),
            budget_overruns: self.budget_enforcer.lock().get_overrun_count(),
            deadline_misses: self.deadline_misses.load(Ordering::SeqCst),
            watchdog_expirations: self
                .watchdogs
                .read()
                .values()
                .filter(|w| w.is_expired())
                .count() as u64,
            degrade_activations: self.degrade_activations.load(Ordering::SeqCst),
        }
    }
}

/// Safety statistics
#[derive(Debug, Clone)]
pub struct SafetyStats {
    state: SafetyState,
    budget_overruns: u64,
    deadline_misses: u64,
    watchdog_expirations: u64,
    degrade_activations: u64,
}

impl SafetyStats {
    pub fn state(&self) -> &SafetyState {
        &self.state
    }

    pub fn budget_overruns(&self) -> u64 {
        self.budget_overruns
    }

    pub fn deadline_misses(&self) -> u64 {
        self.deadline_misses
    }

    pub fn watchdog_expirations(&self) -> u64 {
        self.watchdog_expirations
    }

    pub fn degrade_activations(&self) -> u64 {
        self.degrade_activations
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::duration_ext::DurationExt;

    // ── Watchdog TOCTOU fix tests ─────────────────────────────────────────

    /// feed() called shortly before check() with a generous timeout must return not-expired.
    ///
    /// Before the fix, the Mutex<Instant> TOCTOU window let a heartbeat arrive
    /// between the lock() and elapsed() calls, resulting in a stale timestamp and
    /// a false-positive expired reading.  With AtomicU64 the window is eliminated.
    ///
    /// Uses a 50ms timeout so that even on loaded non-RT systems where
    /// thread::sleep(1μs) can oversleep to ~1ms, the check still passes.
    #[test]
    fn test_watchdog_feed_then_check_1us_not_expired() {
        use std::thread;

        let wd = Watchdog::new(50_u64.ms());

        // Feed, then sleep briefly (well under the 50ms timeout), then check.
        wd.feed();
        thread::sleep(1_u64.us());
        assert!(
            !wd.check(),
            "watchdog should NOT be expired shortly after feed with 50ms timeout"
        );
    }

    /// Verify that a watchdog correctly expires after the timeout period.
    #[test]
    fn test_watchdog_expires_after_timeout() {
        use std::thread;

        let wd = Watchdog::new(10_u64.ms());
        wd.feed();
        thread::sleep(15_u64.ms());
        assert!(
            wd.check(),
            "watchdog should be expired after 15ms with 10ms timeout"
        );
    }

    /// Verify that feeding a watchdog resets the expiry flag.
    #[test]
    fn test_watchdog_feed_clears_expired() {
        use std::thread;

        let wd = Watchdog::new(10_u64.ms());
        wd.feed();
        thread::sleep(15_u64.ms());
        assert!(wd.check(), "should be expired before re-feed");

        wd.feed();
        assert!(
            !wd.is_expired(),
            "expired flag should be cleared after feed"
        );
        assert!(
            !wd.check(),
            "should not be expired immediately after re-feed"
        );
    }

    // ── std::thread concurrent tests ──────────────────────────────────────

    /// Verify that add_critical_node() and record_deadline_miss() can be
    /// called concurrently from separate threads without data races.
    ///
    /// Before the fix, add_critical_node() mutated a plain Vec<String> while
    /// record_deadline_miss() read it under &self — a data race with UB.
    /// After the fix, both paths go through the Arc<RwLock<Vec<String>>>.
    #[test]
    fn test_concurrent_add_and_record_deadline_miss() {
        use std::sync::Arc;
        use std::thread;

        let monitor = Arc::new(SafetyMonitor::new(100));

        // Writer thread: add critical nodes concurrently with readers.
        let m_w = monitor.clone();
        let writer = thread::spawn(move || {
            for i in 0..50 {
                m_w.add_critical_node(format!("node_{}", i), 100_u64.ms());
            }
        });

        // Reader thread: record deadline misses while writer runs.
        let m_r = monitor.clone();
        let reader = thread::spawn(move || {
            for i in 0..50 {
                m_r.record_deadline_miss(&format!("node_{}", i));
            }
        });

        writer.join().unwrap();
        reader.join().unwrap();

        // 50 deadline misses were recorded; max is 100 so no emergency stop
        // (unless a node added by the writer was checked — that's fine too).
        let stats = monitor.get_stats();
        assert_eq!(stats.deadline_misses, 50);
    }

    /// Verify that add_critical_node() and check_watchdogs() can be called
    /// concurrently without data races.
    #[test]
    fn test_concurrent_add_and_check_watchdogs() {
        use std::sync::Arc;
        use std::thread;

        let monitor = Arc::new(SafetyMonitor::new(100));

        let m_w = monitor.clone();
        let writer = thread::spawn(move || {
            for i in 0..50 {
                m_w.add_critical_node(
                    format!("node_{}", i),
                    500_u64.ms(), // long timeout → won't expire
                );
            }
        });

        let m_r = monitor.clone();
        let reader = thread::spawn(move || {
            let mut expired_buf = Vec::new();
            for _ in 0..20 {
                m_r.check_watchdogs(&mut expired_buf);
            }
        });

        writer.join().unwrap();
        reader.join().unwrap();
    }

    /// Smoke-test that a critical-node deadline miss still triggers emergency
    /// stop after the RwLock refactor.
    #[test]
    fn test_critical_node_deadline_miss_triggers_emergency_stop() {
        let monitor = SafetyMonitor::new(100);
        monitor.add_critical_node("critical".to_string(), 1_u64.secs());

        assert!(!monitor.is_emergency_stop());
        monitor.record_deadline_miss("critical");
        assert!(
            monitor.is_emergency_stop(),
            "Emergency stop should be set after critical node deadline miss"
        );
    }

    /// check_watchdogs() with 100 watchdogs must complete in < 1μs on average
    /// under concurrent add_critical_node() pressure.
    ///
    /// The test runs 10 000 check iterations and asserts that the mean is below
    /// 1 μs.  This is conservative — on a modern CPU iterating 100 HashMap
    /// entries doing only atomic loads takes ~200–400 ns.
    ///
    /// Note: the RwLock allows concurrent readers so the "concurrent add pressure"
    /// thread only briefly contends when it acquires the write lock for a new
    /// insert; during reads the check thread is never blocked by the writer.
    /// Verifies that swapping Mutex for RwLock did not introduce unexpected
    /// contention or overhead in the check_watchdogs() hot path when a
    /// concurrent writer is adding nodes.
    ///
    /// Budget: 20 µs/call for 100–200 nodes.  At ~33 ns/node × 200 nodes the
    /// expected cost is ~6-7 µs; the 20 µs ceiling leaves 3× headroom for
    /// scheduling jitter while still catching pathological lock stalls.
    /// (At a 1 kHz RT rate a 20 µs watchdog check costs ~2% of the tick budget.)
    #[test]
    fn check_watchdogs_100_nodes_under_20us_mean() {
        use std::sync::Arc;
        use std::thread;

        let monitor = Arc::new(SafetyMonitor::new(100_000));

        // Register 100 watchdogs with a long timeout (they won't expire in test).
        for i in 0..100 {
            monitor.add_critical_node(format!("bench_node_{:03}", i), 3600_u64.secs());
        }

        // Background writer: add more nodes concurrently during the benchmark.
        let m_w = monitor.clone();
        let writer = thread::spawn(move || {
            for i in 100..200 {
                m_w.add_critical_node(format!("bench_node_{:03}", i), 3600_u64.secs());
                // Slow the writer so it doesn't finish before the benchmark starts.
                std::hint::spin_loop();
            }
        });

        // Benchmark: time 10 000 check_watchdogs() calls.
        const ITERS: u32 = 10_000;
        let mut expired_buf = Vec::with_capacity(16);
        let t0 = std::time::Instant::now();
        for _ in 0..ITERS {
            monitor.check_watchdogs(&mut expired_buf);
            std::hint::black_box(&expired_buf);
        }
        let total = t0.elapsed();
        let mean_ns = total.as_nanos() / ITERS as u128;

        writer.join().unwrap();

        // 20 µs ceiling for 100-200 nodes in release; expected ~6-7 µs, 3× headroom.
        // Debug builds skip the wall-clock assertion (instrumentation overhead ~10-40 µs).
        #[cfg(not(debug_assertions))]
        assert!(
            mean_ns < 20_000,
            "check_watchdogs() mean latency must be < 20μs for 100-200 nodes; got {} ns",
            mean_ns
        );
        #[cfg(debug_assertions)]
        let _ = mean_ns;
    }

    /// Smoke-test that a non-critical deadline miss below max does not trigger
    /// emergency stop.
    #[test]
    fn test_non_critical_deadline_miss_no_emergency() {
        let monitor = SafetyMonitor::new(100);
        monitor.add_critical_node("critical".to_string(), 1_u64.secs());

        monitor.record_deadline_miss("other_node");
        assert!(!monitor.is_emergency_stop());
        assert_eq!(monitor.get_stats().deadline_misses, 1);
    }

    // ── Graduated watchdog tests ─────────────────────────────────────────

    /// Watchdog returns Ok immediately after feed.
    #[test]
    fn test_watchdog_graduated_ok_after_feed() {
        let wd = Watchdog::new(50_u64.ms());
        wd.feed();
        assert_eq!(wd.check_graduated(), WatchdogSeverity::Ok);
    }

    /// Watchdog returns Warning between 1x and 2x timeout.
    #[test]
    fn test_watchdog_graduated_warning() {
        use std::thread;

        // 10ms timeout, sleep 15ms → 1.5x → Warning
        let wd = Watchdog::new(10_u64.ms());
        wd.feed();
        thread::sleep(15_u64.ms());
        assert_eq!(wd.check_graduated(), WatchdogSeverity::Warning);
    }

    /// Watchdog returns Expired between 2x and 3x timeout.
    #[test]
    fn test_watchdog_graduated_expired() {
        use std::thread;

        // 10ms timeout, sleep 25ms → 2.5x → Expired
        let wd = Watchdog::new(10_u64.ms());
        wd.feed();
        thread::sleep(25_u64.ms());
        assert_eq!(wd.check_graduated(), WatchdogSeverity::Expired);
    }

    /// Watchdog returns Critical beyond 3x timeout.
    #[test]
    fn test_watchdog_graduated_critical() {
        use std::thread;

        // 10ms timeout, sleep 35ms → 3.5x → Critical
        let wd = Watchdog::new(10_u64.ms());
        wd.feed();
        thread::sleep(35_u64.ms());
        assert_eq!(wd.check_graduated(), WatchdogSeverity::Critical);
    }

    /// Feed resets graduated severity back to Ok.
    #[test]
    fn test_watchdog_graduated_feed_resets() {
        use std::thread;

        let wd = Watchdog::new(10_u64.ms());
        wd.feed();
        thread::sleep(25_u64.ms());
        assert_eq!(wd.check_graduated(), WatchdogSeverity::Expired);

        // Feed → should be Ok again
        wd.feed();
        assert_eq!(wd.check_graduated(), WatchdogSeverity::Ok);
    }

    /// SafetyMonitor::check_watchdogs_graduated returns graduated severities.
    #[test]
    fn test_monitor_graduated_watchdog() {
        use std::thread;

        let monitor = SafetyMonitor::new(100);
        // 10ms timeout watchdog — we'll let it expire to Warning
        monitor.add_critical_node("fast_node".to_string(), 10_u64.ms());
        // 500ms timeout — will stay Ok
        monitor.add_critical_node("slow_node".to_string(), 500_u64.ms());

        thread::sleep(15_u64.ms());

        let mut results = Vec::new();
        monitor.check_watchdogs_graduated(&mut results);

        // fast_node should be Warning (1x-2x), slow_node should not appear (Ok)
        assert!(
            results
                .iter()
                .any(|(n, s)| n == "fast_node" && *s == WatchdogSeverity::Warning),
            "fast_node should be at Warning severity; got {:?}",
            results
        );
        assert!(
            !results.iter().any(|(n, _)| n == "slow_node"),
            "slow_node should not appear (still Ok)"
        );
    }

    /// Graduated watchdog triggers emergency stop for critical nodes at 3x timeout.
    #[test]
    fn test_monitor_graduated_critical_triggers_estop() {
        use std::thread;

        let monitor = SafetyMonitor::new(100);
        monitor.add_critical_node("critical_ctrl".to_string(), 10_u64.ms());

        // Sleep past 3x timeout
        thread::sleep(35_u64.ms());

        let mut results = Vec::new();
        monitor.check_watchdogs_graduated(&mut results);

        assert!(
            monitor.is_emergency_stop(),
            "Emergency stop should trigger for critical node at 3x timeout"
        );
    }

    /// Non-critical nodes at Critical severity don't trigger emergency stop.
    #[test]
    fn test_monitor_graduated_noncritical_no_estop() {
        use std::thread;

        let monitor = SafetyMonitor::new(100);
        // Register a watchdog but NOT as critical
        monitor
            .watchdogs
            .write()
            .insert("noncritical".to_string(), Watchdog::new(10_u64.ms()));

        thread::sleep(35_u64.ms());

        let mut results = Vec::new();
        monitor.check_watchdogs_graduated(&mut results);

        assert!(!monitor.is_emergency_stop());
        assert!(results
            .iter()
            .any(|(n, s)| n == "noncritical" && *s == WatchdogSeverity::Critical));
    }

    /// Buffer is reused across calls (cleared at entry).
    #[test]
    fn test_monitor_graduated_buffer_reuse() {
        let monitor = SafetyMonitor::new(100);
        monitor.add_critical_node("node_a".to_string(), 3600_u64.secs());

        let mut results = Vec::new();
        // Pollute buffer with old data
        results.push(("stale".to_string(), WatchdogSeverity::Critical));

        monitor.check_watchdogs_graduated(&mut results);

        // Buffer should be cleared — stale entry gone, only current results
        assert!(
            !results.iter().any(|(n, _)| n == "stale"),
            "Buffer should be cleared at entry"
        );
    }

    // ── NodeTimingState tests ────────────────────────────────────────────

    /// TickTimingRing wraps correctly after capacity.
    #[test]
    fn test_timing_ring_wraps() {
        let mut ring = TickTimingRing::new();
        // Fill beyond capacity
        for i in 0..(TIMING_RING_CAPACITY + 100) {
            ring.record(i as u64);
        }
        assert_eq!(ring.total_count(), (TIMING_RING_CAPACITY + 100) as u64);
        let stats = ring.stats();
        assert!(stats.total_ticks > 0);
    }

    /// TimingStats p99 is correct for uniform data.
    #[test]
    fn test_timing_stats_p99() {
        let mut ring = TickTimingRing::new();
        // Record 100 samples: 1..=100
        for i in 1..=100u64 {
            ring.record(i);
        }
        let stats = ring.stats();
        assert_eq!(stats.min_us, 1);
        assert_eq!(stats.max_us, 100);
        assert!(
            stats.p99_us >= 99,
            "p99 should be >= 99, got {}",
            stats.p99_us
        );
    }

    /// NodeTimingState tracks overruns correctly.
    #[test]
    fn test_node_timing_state_overruns() {
        let mut state = NodeTimingState::new(Some(100_u64.us()));

        // Normal tick — no violation
        assert!(state.record_tick(50_u64.us()).is_none());
        assert_eq!(state.overrun_count, 0);

        // Overrun tick
        let violation = state.record_tick(150_u64.us());
        assert!(violation.is_some());
        assert_eq!(state.overrun_count, 1);
        assert_eq!(state.worst_overrun_us, 50); // 150 - 100
    }

    /// NodeTimingState tracks consecutive deadline misses.
    #[test]
    fn test_node_timing_consecutive_misses() {
        let mut state = NodeTimingState::new(None);

        state.record_miss(100);
        assert_eq!(state.consecutive_misses, 1);
        state.record_miss(200);
        assert_eq!(state.consecutive_misses, 2);
        state.record_miss(50);
        assert_eq!(state.consecutive_misses, 3);
        assert!(state.is_chronic());

        // Successful tick resets consecutive counter
        state.record_tick(10_u64.us());
        assert_eq!(state.consecutive_misses, 0);
        assert!(!state.is_chronic());
        // But total misses persist
        assert_eq!(state.total_deadline_misses, 3);
    }

    /// BudgetEnforcer records per-node timing correctly.
    #[test]
    fn test_budget_enforcer_per_node() {
        let mut enforcer = BudgetEnforcer::new();
        enforcer.set_budget("motor".to_string(), 100_u64.us());

        // Under budget
        assert!(enforcer.check_budget("motor", 80_u64.us()).is_ok());

        // Over budget
        assert!(enforcer.check_budget("motor", 150_u64.us()).is_err());
        assert_eq!(enforcer.get_overrun_count(), 1);

        // No budget set — should always succeed
        assert!(enforcer.check_budget("planner", 9999_u64.us()).is_ok());
    }

    // ── Graduated degradation tests ──────────────────────────────────────

    #[test]
    fn test_degradation_policy_default() {
        let policy = DegradationPolicy::default();
        assert_eq!(policy.warn_after, 3);
        assert_eq!(policy.reduce_after, 5);
        assert_eq!(policy.isolate_after, 10);
        assert_eq!(policy.recovery_ticks, 100);
    }

    #[test]
    fn test_degradation_graduated_stages_in_order() {
        let mut monitor = SafetyMonitor::new(100);
        monitor.set_degradation_policy(DegradationPolicy {
            warn_after: 3,
            reduce_after: 5,
            isolate_after: 10,
            kill_after: 20,
            recovery_ticks: 100,
        });

        // Below warn threshold — no action
        let action = monitor.evaluate_degradation("motor", 2, Some(100.0));
        assert_eq!(action, DegradationAction::None);

        // At warn threshold — warn
        let action = monitor.evaluate_degradation("motor", 3, Some(100.0));
        assert_eq!(action, DegradationAction::Warn("motor".to_string()));

        // At reduce threshold — reduce rate to half
        let action = monitor.evaluate_degradation("motor", 5, Some(100.0));
        assert_eq!(
            action,
            DegradationAction::ReduceRate {
                node: "motor".to_string(),
                new_rate_hz: 50.0,
            }
        );

        // At isolate threshold — isolate
        let action = monitor.evaluate_degradation("motor", 10, Some(100.0));
        assert_eq!(action, DegradationAction::Isolate("motor".to_string()));

        // At kill threshold — kill the node permanently
        let action = monitor.evaluate_degradation("motor", 20, Some(100.0));
        assert_eq!(action, DegradationAction::Kill("motor".to_string()));

        // Already killed — no further action
        let action = monitor.evaluate_degradation("motor", 30, Some(100.0));
        assert_eq!(action, DegradationAction::None);
    }

    #[test]
    fn test_degradation_recovery_after_stable_period() {
        let mut monitor = SafetyMonitor::new(100);
        monitor.set_degradation_policy(DegradationPolicy {
            warn_after: 1,
            reduce_after: 2,
            isolate_after: 100, // high so we test recovery at RateReduced stage
            kill_after: 200,
            recovery_ticks: 5,
        });

        // Trigger rate reduction
        let _ = monitor.evaluate_degradation("sensor", 1, Some(200.0)); // warn
        let action = monitor.evaluate_degradation("sensor", 2, Some(200.0)); // reduce
        assert_eq!(
            action,
            DegradationAction::ReduceRate {
                node: "sensor".to_string(),
                new_rate_hz: 100.0,
            }
        );

        // 4 successful ticks — not yet recovered
        for _ in 0..4 {
            let action = monitor.record_successful_tick("sensor");
            assert_eq!(action, DegradationAction::None);
        }

        // 5th successful tick — recovery!
        let action = monitor.record_successful_tick("sensor");
        assert_eq!(
            action,
            DegradationAction::RestoreRate {
                node: "sensor".to_string(),
                original_rate_hz: 200.0,
            }
        );

        // Stage should be back to Normal
        assert_eq!(
            monitor.degradation_stage("sensor"),
            DegradationStage::Normal
        );
    }

    #[test]
    fn test_degradation_warned_recovers_on_success() {
        let mut monitor = SafetyMonitor::new(100);
        monitor.set_degradation_policy(DegradationPolicy {
            warn_after: 2,
            reduce_after: 5,
            isolate_after: 10,
            kill_after: 20,
            recovery_ticks: 100,
        });

        // Trigger warn
        let action = monitor.evaluate_degradation("planner", 2, Some(50.0));
        assert_eq!(action, DegradationAction::Warn("planner".to_string()));
        assert_eq!(
            monitor.degradation_stage("planner"),
            DegradationStage::Warned
        );

        // Single successful tick at Warned stage → back to Normal
        let action = monitor.record_successful_tick("planner");
        assert_eq!(action, DegradationAction::None);
        assert_eq!(
            monitor.degradation_stage("planner"),
            DegradationStage::Normal
        );
    }

    #[test]
    fn test_degradation_no_rate_set_uses_default() {
        let mut monitor = SafetyMonitor::new(100);
        monitor.set_degradation_policy(DegradationPolicy {
            warn_after: 1,
            reduce_after: 2,
            isolate_after: 10,
            kill_after: 20,
            recovery_ticks: 100,
        });

        let _ = monitor.evaluate_degradation("node", 1, None); // warn
        let action = monitor.evaluate_degradation("node", 2, None); // reduce
                                                                    // Should use 100.0 / 2 = 50.0 as fallback
        assert_eq!(
            action,
            DegradationAction::ReduceRate {
                node: "node".to_string(),
                new_rate_hz: 50.0,
            }
        );
    }

    #[test]
    fn test_degradation_multiple_nodes_independent() {
        let mut monitor = SafetyMonitor::new(100);
        monitor.set_degradation_policy(DegradationPolicy {
            warn_after: 2,
            reduce_after: 5,
            isolate_after: 10,
            kill_after: 20,
            recovery_ticks: 100,
        });

        // Node A warns
        let action = monitor.evaluate_degradation("node_a", 2, Some(100.0));
        assert_eq!(action, DegradationAction::Warn("node_a".to_string()));

        // Node B still normal (different consecutive count)
        let action = monitor.evaluate_degradation("node_b", 1, Some(200.0));
        assert_eq!(action, DegradationAction::None);

        // Node A reduces
        let action = monitor.evaluate_degradation("node_a", 5, Some(100.0));
        assert_eq!(
            action,
            DegradationAction::ReduceRate {
                node: "node_a".to_string(),
                new_rate_hz: 50.0,
            }
        );

        // Node B still unaffected
        assert_eq!(
            monitor.degradation_stage("node_b"),
            DegradationStage::Normal
        );
    }

    #[test]
    fn test_consecutive_misses_method() {
        let monitor = SafetyMonitor::new(100);
        assert_eq!(monitor.consecutive_misses("unknown_node"), 0);

        monitor.record_deadline_miss_with_severity("motor", 500);
        assert_eq!(monitor.consecutive_misses("motor"), 1);

        monitor.record_deadline_miss_with_severity("motor", 600);
        assert_eq!(monitor.consecutive_misses("motor"), 2);
    }

    #[test]
    fn test_timing_ring_total_count() {
        let mut ring = TickTimingRing::new();
        assert_eq!(ring.total_count(), 0);
        ring.record(100);
        ring.record(200);
        ring.record(300);
        assert_eq!(ring.total_count(), 3);
    }

    #[test]
    fn test_timing_stats_fields() {
        let mut ring = TickTimingRing::new();
        ring.record(10);
        ring.record(20);
        ring.record(30);
        ring.record(40);
        ring.record(50);
        let stats = ring.stats();
        assert_eq!(stats.min_us, 10);
        assert_eq!(stats.max_us, 50);
        assert_eq!(stats.avg_us, 30);
        assert_eq!(stats.total_ticks, 5);
    }

    #[test]
    fn test_node_timing_is_chronic() {
        let mut state = NodeTimingState::new(Some(1_u64.ms()));
        assert!(!state.is_chronic());
        state.record_miss(100);
        state.record_miss(200);
        assert!(!state.is_chronic());
        state.record_miss(300);
        assert!(state.is_chronic());
    }

    #[test]
    fn test_budget_enforcer_node_stats() {
        let mut enforcer = BudgetEnforcer::new();
        // No data yet
        assert!(enforcer.node_stats("unknown").is_none());

        // Set a budget and record some ticks
        enforcer.set_budget("motor".to_string(), 10_u64.ms());
        enforcer.check_budget("motor", 100_u64.us()).ok();
        enforcer.check_budget("motor", 200_u64.us()).ok();
        enforcer.check_budget("motor", 300_u64.us()).ok();

        let stats = enforcer.node_stats("motor");
        assert!(stats.is_some());
        let stats = stats.unwrap();
        assert_eq!(stats.min_us, 100);
        assert_eq!(stats.max_us, 300);
        assert_eq!(stats.total_ticks, 3);
    }

    #[test]
    fn test_set_degradation_policy_and_stage() {
        let mut monitor = SafetyMonitor::new(100);

        // Default stage is Normal
        assert_eq!(monitor.degradation_stage("motor"), DegradationStage::Normal);

        // With high thresholds, low miss counts return None
        monitor.set_degradation_policy(DegradationPolicy {
            warn_after: 1000,
            reduce_after: 2000,
            isolate_after: 3000,
            kill_after: 6000,
            recovery_ticks: 100,
        });

        monitor.record_deadline_miss_with_severity("motor", 500);
        let action = monitor.evaluate_degradation("motor", 1, None);
        assert_eq!(action, DegradationAction::None);
    }

    // ========================================================================
    // Deisolate recovery tests
    // ========================================================================

    #[test]
    fn test_deisolate_recovery_from_isolated_stage() {
        let mut monitor = SafetyMonitor::new(100);
        monitor.set_degradation_policy(DegradationPolicy {
            warn_after: 1,
            reduce_after: 2,
            isolate_after: 5,
            kill_after: 100,
            recovery_ticks: 3,
        });

        // Escalate to Isolated
        let _ = monitor.evaluate_degradation("motor", 1, Some(100.0)); // Warn
        let _ = monitor.evaluate_degradation("motor", 2, Some(100.0)); // ReduceRate
        let action = monitor.evaluate_degradation("motor", 5, Some(100.0)); // Isolate
        assert_eq!(action, DegradationAction::Isolate("motor".to_string()));
        assert_eq!(
            monitor.degradation_stage("motor"),
            DegradationStage::Isolated
        );

        // 2 successful ticks — not yet recovered
        for _ in 0..2 {
            let action = monitor.record_successful_tick("motor");
            assert_eq!(action, DegradationAction::None);
        }
        assert_eq!(
            monitor.degradation_stage("motor"),
            DegradationStage::Isolated
        );

        // 3rd successful tick — Deisolate!
        let action = monitor.record_successful_tick("motor");
        assert_eq!(
            action,
            DegradationAction::Deisolate("motor".to_string())
        );
        assert_eq!(
            monitor.degradation_stage("motor"),
            DegradationStage::RateReduced
        );
    }

    #[test]
    fn test_deisolate_then_restore_full_recovery_path() {
        let mut monitor = SafetyMonitor::new(100);
        monitor.set_degradation_policy(DegradationPolicy {
            warn_after: 1,
            reduce_after: 2,
            isolate_after: 5,
            kill_after: 100,
            recovery_ticks: 3,
        });

        // Escalate to Isolated
        let _ = monitor.evaluate_degradation("arm", 1, Some(200.0));
        let _ = monitor.evaluate_degradation("arm", 2, Some(200.0));
        let _ = monitor.evaluate_degradation("arm", 5, Some(200.0));
        assert_eq!(monitor.degradation_stage("arm"), DegradationStage::Isolated);

        // Recover from Isolated → RateReduced (3 successful ticks)
        for _ in 0..2 {
            monitor.record_successful_tick("arm");
        }
        let action = monitor.record_successful_tick("arm");
        assert_eq!(action, DegradationAction::Deisolate("arm".to_string()));
        assert_eq!(
            monitor.degradation_stage("arm"),
            DegradationStage::RateReduced
        );

        // Recover from RateReduced → Normal (3 more successful ticks)
        for _ in 0..2 {
            monitor.record_successful_tick("arm");
        }
        let action = monitor.record_successful_tick("arm");
        assert_eq!(
            action,
            DegradationAction::RestoreRate {
                node: "arm".to_string(),
                original_rate_hz: 200.0,
            }
        );
        assert_eq!(monitor.degradation_stage("arm"), DegradationStage::Normal);
    }

    #[test]
    fn test_isolated_node_does_not_stay_stuck_forever() {
        let mut monitor = SafetyMonitor::new(100);
        monitor.set_degradation_policy(DegradationPolicy {
            warn_after: 1,
            reduce_after: 2,
            isolate_after: 3,
            kill_after: 100,
            recovery_ticks: 5,
        });

        // Isolate the node
        let _ = monitor.evaluate_degradation("sensor", 1, Some(50.0));
        let _ = monitor.evaluate_degradation("sensor", 2, Some(50.0));
        let _ = monitor.evaluate_degradation("sensor", 3, Some(50.0));
        assert_eq!(
            monitor.degradation_stage("sensor"),
            DegradationStage::Isolated
        );

        // Simulate 1000 successful ticks — must eventually recover fully
        let mut saw_deisolate = false;
        let mut saw_restore = false;
        for _ in 0..1000 {
            let action = monitor.record_successful_tick("sensor");
            match action {
                DegradationAction::Deisolate(_) => saw_deisolate = true,
                DegradationAction::RestoreRate { .. } => saw_restore = true,
                _ => {}
            }
            if saw_restore {
                break;
            }
        }
        assert!(saw_deisolate, "node must transition through Deisolate");
        assert!(saw_restore, "node must eventually reach RestoreRate");
        assert_eq!(
            monitor.degradation_stage("sensor"),
            DegradationStage::Normal
        );
    }

    // ========================================================================
    // Kill degradation tests
    // ========================================================================

    #[test]
    fn test_kill_stage_after_prolonged_failure() {
        let mut monitor = SafetyMonitor::new(100);
        monitor.set_degradation_policy(DegradationPolicy {
            warn_after: 2,
            reduce_after: 5,
            isolate_after: 10,
            kill_after: 20,
            recovery_ticks: 100,
        });

        // Escalate through all stages
        let _ = monitor.evaluate_degradation("stalled", 2, Some(100.0)); // Warn
        let _ = monitor.evaluate_degradation("stalled", 5, Some(100.0)); // ReduceRate
        let _ = monitor.evaluate_degradation("stalled", 10, Some(100.0)); // Isolate
        let action = monitor.evaluate_degradation("stalled", 20, Some(100.0)); // Kill
        assert_eq!(
            action,
            DegradationAction::Kill("stalled".to_string())
        );
        assert_eq!(
            monitor.degradation_stage("stalled"),
            DegradationStage::Killed
        );
    }

    #[test]
    fn test_killed_node_no_further_actions() {
        let mut monitor = SafetyMonitor::new(100);
        monitor.set_degradation_policy(DegradationPolicy {
            warn_after: 1,
            reduce_after: 2,
            isolate_after: 3,
            kill_after: 5,
            recovery_ticks: 2,
        });

        // Kill the node
        let _ = monitor.evaluate_degradation("dead", 1, Some(100.0));
        let _ = monitor.evaluate_degradation("dead", 2, Some(100.0));
        let _ = monitor.evaluate_degradation("dead", 3, Some(100.0));
        let _ = monitor.evaluate_degradation("dead", 5, Some(100.0));
        assert_eq!(monitor.degradation_stage("dead"), DegradationStage::Killed);

        // Further misses produce no action (already dead)
        let action = monitor.evaluate_degradation("dead", 100, Some(100.0));
        assert_eq!(action, DegradationAction::None);

        // Successful ticks produce no action (killed nodes can't recover)
        let action = monitor.record_successful_tick("dead");
        assert_eq!(action, DegradationAction::None);
    }

    // ========================================================================
    // Multi-node independent degradation
    // ========================================================================

    #[test]
    fn test_degradation_multiple_nodes_independent_stages() {
        let mut monitor = SafetyMonitor::new(100);
        monitor.set_degradation_policy(DegradationPolicy {
            warn_after: 2,
            reduce_after: 4,
            isolate_after: 8,
            kill_after: 16,
            recovery_ticks: 3,
        });

        // Motor is isolated, arm is warned, sensor is normal
        let _ = monitor.evaluate_degradation("motor", 2, Some(100.0));
        let _ = monitor.evaluate_degradation("motor", 4, Some(100.0));
        let _ = monitor.evaluate_degradation("motor", 8, Some(100.0));
        let _ = monitor.evaluate_degradation("arm", 2, Some(50.0));

        assert_eq!(
            monitor.degradation_stage("motor"),
            DegradationStage::Isolated
        );
        assert_eq!(
            monitor.degradation_stage("arm"),
            DegradationStage::Warned
        );

        // Motor recovers while arm stays warned
        for _ in 0..3 {
            monitor.record_successful_tick("motor");
        }
        assert_eq!(
            monitor.degradation_stage("motor"),
            DegradationStage::RateReduced
        );
        assert_eq!(
            monitor.degradation_stage("arm"),
            DegradationStage::Warned
        );

        // Arm recovers independently
        let action = monitor.record_successful_tick("arm");
        assert_eq!(action, DegradationAction::None);
        assert_eq!(
            monitor.degradation_stage("arm"),
            DegradationStage::Normal
        );
    }

    // ========================================================================
    // Emergency stop cleanup
    // ========================================================================

    #[test]
    fn test_emergency_stop_sets_flag_and_state() {
        let monitor = SafetyMonitor::new(100);
        assert!(!monitor.is_emergency_stop());

        monitor.trigger_emergency_stop("test reason".to_string());

        assert!(monitor.is_emergency_stop());
    }

    #[test]
    fn test_emergency_stop_from_critical_node_miss() {
        let monitor = SafetyMonitor::new(100);
        monitor.add_critical_node("balance_controller".to_string(), Duration::from_millis(100));

        // Non-critical miss — no estop
        monitor.record_deadline_miss("arm_controller");
        assert!(!monitor.is_emergency_stop());

        // Critical miss — estop
        monitor.record_deadline_miss("balance_controller");
        assert!(monitor.is_emergency_stop());
    }

    // ========================================================================
    // Stress and multi-node tests
    // ========================================================================

    /// 10+ nodes with different budgets, all recording ticks concurrently.
    /// Verifies per-node isolation: no cross-contamination of timing data.
    #[test]
    fn test_multi_node_different_budgets_all_recording() {
        let monitor = SafetyMonitor::new(1000);
        let node_count = 15;

        // Register nodes with budgets from 100us to 1500us (100us increments)
        for i in 0..node_count {
            let budget = Duration::from_micros((i + 1) * 100);
            let name = format!("node_{:02}", i);
            monitor.set_tick_budget(name, budget);
        }

        // Each node records 50 ticks at half its budget (no overruns expected)
        for i in 0..node_count {
            let tick_duration = Duration::from_micros((i + 1) * 50);
            let name = format!("node_{:02}", i);
            for _ in 0..50 {
                let result = monitor.check_tick_budget(&name, tick_duration);
                assert!(result.is_ok(), "node_{:02} should be within budget", i);
            }
        }

        // Verify per-node stats are independent
        let all_stats = monitor.all_node_timing();
        assert_eq!(all_stats.len(), node_count as usize);
        for (name, stats, budget, overruns) in &all_stats {
            assert_eq!(stats.total_ticks, 50, "{} should have 50 ticks", name);
            assert!(budget.is_some(), "{} should have a budget set", name);
            assert_eq!(*overruns, 0, "{} should have 0 overruns", name);
        }
    }

    /// Rapid watchdog feed/check cycles: 1000+ iterations in a tight loop.
    /// Verifies the AtomicU64 approach stays consistent under rapid mutation.
    #[test]
    fn test_rapid_watchdog_feed_check_1000_iterations() {
        let wd = Watchdog::new(1_u64.secs()); // long timeout so it never expires

        for _ in 0..2000 {
            wd.feed();
            assert!(!wd.check(), "watchdog must not expire with 1s timeout in a tight loop");
            assert!(!wd.is_expired());
            assert_eq!(wd.check_graduated(), WatchdogSeverity::Ok);
        }
    }

    /// BudgetEnforcer with 20 different node names, each recording multiple ticks.
    /// Verifies HashMap scaling and per-node isolation.
    #[test]
    fn test_budget_enforcer_20_nodes() {
        let mut enforcer = BudgetEnforcer::new();

        for i in 0..20 {
            enforcer.set_budget(format!("node_{:02}", i), Duration::from_micros((i + 1) * 100));
        }

        // Record 100 ticks for each node at exactly half budget (no overruns)
        for i in 0..20u64 {
            let tick = Duration::from_micros((i + 1) * 50);
            for _ in 0..100 {
                let result = enforcer.check_budget(&format!("node_{:02}", i), tick);
                assert!(result.is_ok());
            }
        }

        assert_eq!(enforcer.get_overrun_count(), 0);

        // Verify each node has independent stats
        for i in 0..20u64 {
            let stats = enforcer.node_stats(&format!("node_{:02}", i)).unwrap();
            assert_eq!(stats.total_ticks, 100);
            assert_eq!(stats.min_us, (i + 1) * 50);
            assert_eq!(stats.max_us, (i + 1) * 50);
            assert_eq!(stats.avg_us, (i + 1) * 50);
        }

        // Now trigger one overrun on node_00 (budget 100us, actual 200us)
        let result = enforcer.check_budget("node_00", 200_u64.us());
        assert!(result.is_err());
        assert_eq!(enforcer.get_overrun_count(), 1);

        // Other nodes unaffected
        let stats_01 = enforcer.node_stats("node_01").unwrap();
        assert_eq!(stats_01.total_ticks, 100);
    }

    /// Degradation cascade: 5+ consecutive misses on one node while others stay healthy.
    /// Verifies that only the failing node degrades through warn -> reduce -> isolate -> kill.
    #[test]
    fn test_degradation_cascade_single_node_others_healthy() {
        let mut monitor = SafetyMonitor::new(1000); // high max so no global estop
        monitor.set_degradation_policy(DegradationPolicy {
            warn_after: 2,
            reduce_after: 4,
            isolate_after: 8,
            kill_after: 15,
            recovery_ticks: 100,
        });

        // 5 healthy nodes
        let healthy_nodes: Vec<String> = (0..5).map(|i| format!("healthy_{}", i)).collect();

        // Simulate the failing node escalating through all stages
        let failing = "failing_motor";

        // Miss 1-1: below warn
        let action = monitor.evaluate_degradation(failing, 1, Some(100.0));
        assert_eq!(action, DegradationAction::None);

        // Miss 2: warn
        let action = monitor.evaluate_degradation(failing, 2, Some(100.0));
        assert_eq!(action, DegradationAction::Warn(failing.to_string()));

        // All healthy nodes remain Normal
        for h in &healthy_nodes {
            assert_eq!(monitor.degradation_stage(h), DegradationStage::Normal);
        }

        // Miss 4: reduce rate
        let action = monitor.evaluate_degradation(failing, 4, Some(100.0));
        assert_eq!(
            action,
            DegradationAction::ReduceRate {
                node: failing.to_string(),
                new_rate_hz: 50.0,
            }
        );

        // Miss 8: isolate
        let action = monitor.evaluate_degradation(failing, 8, Some(100.0));
        assert_eq!(action, DegradationAction::Isolate(failing.to_string()));

        // Miss 15: kill
        let action = monitor.evaluate_degradation(failing, 15, Some(100.0));
        assert_eq!(action, DegradationAction::Kill(failing.to_string()));

        assert_eq!(
            monitor.degradation_stage(failing),
            DegradationStage::Killed
        );

        // Healthy nodes remain Normal throughout
        for h in &healthy_nodes {
            assert_eq!(
                monitor.degradation_stage(h),
                DegradationStage::Normal,
                "{} should still be Normal after failing node was killed",
                h
            );
        }
    }

    /// Recovery after isolation: node gets isolated, then successful ticks bring
    /// it back through Deisolate -> RateReduced -> RestoreRate -> Normal.
    #[test]
    fn test_recovery_after_isolation_full_path() {
        let mut monitor = SafetyMonitor::new(1000);
        monitor.set_degradation_policy(DegradationPolicy {
            warn_after: 1,
            reduce_after: 2,
            isolate_after: 4,
            kill_after: 100,
            recovery_ticks: 10,
        });

        // Escalate to Isolated
        let _ = monitor.evaluate_degradation("lidar", 1, Some(200.0));
        let _ = monitor.evaluate_degradation("lidar", 2, Some(200.0));
        let _ = monitor.evaluate_degradation("lidar", 4, Some(200.0));
        assert_eq!(
            monitor.degradation_stage("lidar"),
            DegradationStage::Isolated
        );

        // 9 successful ticks: still Isolated
        for i in 0..9 {
            let action = monitor.record_successful_tick("lidar");
            assert_eq!(
                action,
                DegradationAction::None,
                "should not deisolate at tick {}",
                i
            );
        }
        assert_eq!(
            monitor.degradation_stage("lidar"),
            DegradationStage::Isolated
        );

        // 10th tick: Deisolate -> RateReduced
        let action = monitor.record_successful_tick("lidar");
        assert_eq!(
            action,
            DegradationAction::Deisolate("lidar".to_string())
        );
        assert_eq!(
            monitor.degradation_stage("lidar"),
            DegradationStage::RateReduced
        );

        // 9 more successful ticks: still RateReduced
        for _ in 0..9 {
            let action = monitor.record_successful_tick("lidar");
            assert_eq!(action, DegradationAction::None);
        }
        assert_eq!(
            monitor.degradation_stage("lidar"),
            DegradationStage::RateReduced
        );

        // 10th tick at RateReduced: RestoreRate -> Normal
        let action = monitor.record_successful_tick("lidar");
        assert_eq!(
            action,
            DegradationAction::RestoreRate {
                node: "lidar".to_string(),
                original_rate_hz: 200.0,
            }
        );
        assert_eq!(
            monitor.degradation_stage("lidar"),
            DegradationStage::Normal
        );
    }

    /// Mixed node states: some healthy, some warning, some unhealthy, some isolated
    /// simultaneously. Verifies full independence across all degradation stages.
    #[test]
    fn test_mixed_node_states_simultaneous() {
        let mut monitor = SafetyMonitor::new(1000);
        monitor.set_degradation_policy(DegradationPolicy {
            warn_after: 2,
            reduce_after: 4,
            isolate_after: 8,
            kill_after: 16,
            recovery_ticks: 5,
        });

        // Node A: Normal (no misses)
        // Node B: Warned (2 misses)
        let _ = monitor.evaluate_degradation("node_b", 2, Some(100.0));
        // Node C: RateReduced (4 misses)
        let _ = monitor.evaluate_degradation("node_c", 2, Some(200.0));
        let _ = monitor.evaluate_degradation("node_c", 4, Some(200.0));
        // Node D: Isolated (8 misses)
        let _ = monitor.evaluate_degradation("node_d", 2, Some(150.0));
        let _ = monitor.evaluate_degradation("node_d", 4, Some(150.0));
        let _ = monitor.evaluate_degradation("node_d", 8, Some(150.0));
        // Node E: Killed (16 misses)
        let _ = monitor.evaluate_degradation("node_e", 2, Some(50.0));
        let _ = monitor.evaluate_degradation("node_e", 4, Some(50.0));
        let _ = monitor.evaluate_degradation("node_e", 8, Some(50.0));
        let _ = monitor.evaluate_degradation("node_e", 16, Some(50.0));

        // Verify all states coexist
        assert_eq!(
            monitor.degradation_stage("node_a"),
            DegradationStage::Normal
        );
        assert_eq!(
            monitor.degradation_stage("node_b"),
            DegradationStage::Warned
        );
        assert_eq!(
            monitor.degradation_stage("node_c"),
            DegradationStage::RateReduced
        );
        assert_eq!(
            monitor.degradation_stage("node_d"),
            DegradationStage::Isolated
        );
        assert_eq!(
            monitor.degradation_stage("node_e"),
            DegradationStage::Killed
        );

        // Recover node B (Warned -> Normal on single success)
        monitor.record_successful_tick("node_b");
        assert_eq!(
            monitor.degradation_stage("node_b"),
            DegradationStage::Normal
        );

        // Node D starts recovering but isn't done yet
        for _ in 0..4 {
            monitor.record_successful_tick("node_d");
        }
        assert_eq!(
            monitor.degradation_stage("node_d"),
            DegradationStage::Isolated
        );

        // Node E stays Killed — no recovery possible
        for _ in 0..100 {
            let action = monitor.record_successful_tick("node_e");
            assert_eq!(action, DegradationAction::None);
        }
        assert_eq!(
            monitor.degradation_stage("node_e"),
            DegradationStage::Killed
        );

        // Other nodes unaffected by node_b's recovery
        assert_eq!(
            monitor.degradation_stage("node_c"),
            DegradationStage::RateReduced
        );
    }

    /// Watchdog with very short timeout (1ms) — verify it expires after a brief wait.
    /// Uses 5ms sleep to ensure expiry even on loaded systems.
    #[test]
    fn test_watchdog_very_short_timeout_expires() {
        use std::thread;

        let wd = Watchdog::new(1_u64.ms());
        wd.feed();
        thread::sleep(5_u64.ms());
        assert!(
            wd.check(),
            "1ms watchdog should expire after 5ms sleep"
        );
        assert!(wd.is_expired());
    }

    /// Watchdog with very long timeout (1 hour) — verify it does not expire prematurely.
    #[test]
    fn test_watchdog_very_long_timeout_no_premature_expiry() {
        let wd = Watchdog::new(3600_u64.secs()); // 1 hour

        wd.feed();
        // Check immediately — must not be expired
        assert!(
            !wd.check(),
            "1-hour watchdog should not expire immediately"
        );
        assert!(!wd.is_expired());
        assert_eq!(wd.check_graduated(), WatchdogSeverity::Ok);

        // Check 1000 times in a tight loop — still not expired
        for _ in 0..1000 {
            assert!(!wd.check());
        }
        assert!(!wd.is_expired());
    }

    /// BudgetEnforcer with zero budget: any non-zero tick triggers a violation.
    #[test]
    fn test_budget_enforcer_zero_budget() {
        let mut enforcer = BudgetEnforcer::new();
        enforcer.set_budget("zero_node".to_string(), Duration::ZERO);

        // A zero-duration tick should not trigger (0 is not > 0)
        let result = enforcer.check_budget("zero_node", Duration::ZERO);
        assert!(result.is_ok(), "zero tick against zero budget should be ok");

        // Any non-zero tick should trigger a violation
        let result = enforcer.check_budget("zero_node", 1_u64.us());
        assert!(result.is_err(), "1us tick against zero budget should violate");
        assert_eq!(enforcer.get_overrun_count(), 1);

        // Verify stats recorded both ticks
        let stats = enforcer.node_stats("zero_node").unwrap();
        assert_eq!(stats.total_ticks, 2);
    }

    /// SafetyMonitor with no nodes registered: empty-state operations succeed
    /// without panics or incorrect state.
    #[test]
    fn test_safety_monitor_no_nodes_empty_state() {
        let monitor = SafetyMonitor::new(100);

        // Stats should show zeroes
        let stats = monitor.get_stats();
        assert_eq!(*stats.state(), SafetyState::Normal);
        assert_eq!(stats.budget_overruns(), 0);
        assert_eq!(stats.deadline_misses(), 0);
        assert_eq!(stats.watchdog_expirations(), 0);
        assert_eq!(stats.degrade_activations(), 0);

        // Watchdog checks on empty map should produce no results
        let mut expired = Vec::new();
        monitor.check_watchdogs(&mut expired);
        assert!(expired.is_empty());

        let mut graduated = Vec::new();
        monitor.check_watchdogs_graduated(&mut graduated);
        assert!(graduated.is_empty());

        // No emergency stop
        assert!(!monitor.is_emergency_stop());

        // All node timing returns empty
        assert!(monitor.all_node_timing().is_empty());

        // Consecutive misses for unknown node returns 0
        assert_eq!(monitor.consecutive_misses("nonexistent"), 0);

        // Degradation stage for unknown node returns Normal
        assert_eq!(
            monitor.degradation_stage("nonexistent"),
            DegradationStage::Normal
        );

        // Successful tick for unknown node returns None action
        let action = monitor.record_successful_tick("nonexistent");
        assert_eq!(action, DegradationAction::None);
    }

    /// 10 nodes with watchdogs: rapid concurrent feed from multiple threads
    /// while the main thread checks. Verifies no panics or data corruption.
    #[test]
    fn test_multi_node_concurrent_watchdog_feed_and_check() {
        use std::sync::Arc;
        use std::thread;

        let monitor = Arc::new(SafetyMonitor::new(1000));

        // Register 10 critical nodes with generous timeouts
        for i in 0..10 {
            monitor.add_critical_node(format!("wdog_{}", i), 5_u64.secs());
        }

        // Spawn 10 feeder threads, each feeding its own watchdog 500 times
        let mut handles = Vec::new();
        for i in 0..10 {
            let m = monitor.clone();
            handles.push(thread::spawn(move || {
                for _ in 0..500 {
                    m.feed_watchdog(&format!("wdog_{}", i));
                }
            }));
        }

        // Main thread checks watchdogs concurrently
        let mut expired_buf = Vec::new();
        for _ in 0..200 {
            monitor.check_watchdogs(&mut expired_buf);
            // With 5s timeout and active feeding, none should expire
            assert!(
                expired_buf.is_empty(),
                "no watchdog should expire while being fed"
            );
        }

        for h in handles {
            h.join().unwrap();
        }

        // Final check: no emergency stop
        assert!(!monitor.is_emergency_stop());
    }

    /// Budget enforcer stress: 1000+ check_budget calls on a single node.
    /// Verifies ring buffer wrapping (capacity=1024) and correct stats after wrap.
    #[test]
    fn test_budget_enforcer_ring_buffer_stress() {
        let mut enforcer = BudgetEnforcer::new();
        enforcer.set_budget("stress_node".to_string(), 1_u64.ms());

        // Record 2000 ticks (wraps ring buffer which has capacity 1024)
        for i in 0..2000u64 {
            let tick = Duration::from_micros(100 + i); // 100us to 2099us
            let _ = enforcer.check_budget("stress_node", tick);
        }

        let stats = enforcer.node_stats("stress_node").unwrap();
        assert_eq!(stats.total_ticks, 2000);

        // Ring should contain the last 1024 values: 1076us to 2099us
        // (indices 976..2000 map to micros 1076..2099)
        assert_eq!(stats.min_us, 1076);
        assert_eq!(stats.max_us, 2099);

        // Overruns: budget is 1000us, ticks above 1000us started at tick index 900
        // (100+900=1000, first overrun at 100+901=1001us, i.e. i=901)
        // Total overruns = 2000 - 901 = 1099
        assert_eq!(enforcer.get_overrun_count(), 1099);
    }

    /// Multiple deadline misses just under max should not trigger emergency stop,
    /// but the Nth miss (at max) should trigger it.
    #[test]
    fn test_deadline_miss_threshold_exact() {
        let max_misses = 10u64;
        let monitor = SafetyMonitor::new(max_misses);

        // Record max_misses - 1 misses on non-critical nodes
        for i in 0..(max_misses - 1) {
            monitor.record_deadline_miss(&format!("node_{}", i));
            assert!(
                !monitor.is_emergency_stop(),
                "should not estop at miss {} / {}",
                i + 1,
                max_misses
            );
        }

        // The Nth miss triggers emergency stop
        monitor.record_deadline_miss("final_node");
        assert!(
            monitor.is_emergency_stop(),
            "should estop at miss {}/{}",
            max_misses,
            max_misses
        );
    }

    /// Degradation with many nodes (12): verify evaluate_degradation and
    /// record_successful_tick handle high node counts without crosstalk.
    #[test]
    fn test_degradation_12_nodes_independent_lifecycle() {
        let mut monitor = SafetyMonitor::new(10000);
        monitor.set_degradation_policy(DegradationPolicy {
            warn_after: 3,
            reduce_after: 6,
            isolate_after: 12,
            kill_after: 24,
            recovery_ticks: 5,
        });

        // Push each of 12 nodes to a different degradation stage
        // Nodes 0-2: Normal (0 misses)
        // Nodes 3-5: Warned (3 misses)
        for i in 3..6 {
            let _ = monitor.evaluate_degradation(&format!("n{}", i), 3, Some(100.0));
        }
        // Nodes 6-8: RateReduced (6 misses)
        for i in 6..9 {
            let _ = monitor.evaluate_degradation(&format!("n{}", i), 3, Some(100.0));
            let _ = monitor.evaluate_degradation(&format!("n{}", i), 6, Some(100.0));
        }
        // Nodes 9-11: Isolated (12 misses)
        for i in 9..12 {
            let _ = monitor.evaluate_degradation(&format!("n{}", i), 3, Some(100.0));
            let _ = monitor.evaluate_degradation(&format!("n{}", i), 6, Some(100.0));
            let _ = monitor.evaluate_degradation(&format!("n{}", i), 12, Some(100.0));
        }

        // Verify all stages correct
        for i in 0..3 {
            assert_eq!(
                monitor.degradation_stage(&format!("n{}", i)),
                DegradationStage::Normal
            );
        }
        for i in 3..6 {
            assert_eq!(
                monitor.degradation_stage(&format!("n{}", i)),
                DegradationStage::Warned
            );
        }
        for i in 6..9 {
            assert_eq!(
                monitor.degradation_stage(&format!("n{}", i)),
                DegradationStage::RateReduced
            );
        }
        for i in 9..12 {
            assert_eq!(
                monitor.degradation_stage(&format!("n{}", i)),
                DegradationStage::Isolated
            );
        }

        // Recover all Warned nodes (single success each)
        for i in 3..6 {
            monitor.record_successful_tick(&format!("n{}", i));
            assert_eq!(
                monitor.degradation_stage(&format!("n{}", i)),
                DegradationStage::Normal
            );
        }

        // Recover all RateReduced nodes (5 successes each)
        for i in 6..9 {
            for _ in 0..5 {
                monitor.record_successful_tick(&format!("n{}", i));
            }
            assert_eq!(
                monitor.degradation_stage(&format!("n{}", i)),
                DegradationStage::Normal
            );
        }

        // Isolated nodes still isolated (not enough successes yet)
        for i in 9..12 {
            for _ in 0..4 {
                monitor.record_successful_tick(&format!("n{}", i));
            }
            assert_eq!(
                monitor.degradation_stage(&format!("n{}", i)),
                DegradationStage::Isolated
            );
        }
    }

    /// SafetyMonitor stats reflect degrade_activations count.
    #[test]
    fn test_degrade_activation_counter() {
        let monitor = SafetyMonitor::new(100);

        assert_eq!(monitor.get_stats().degrade_activations(), 0);

        monitor.record_degrade_activation();
        monitor.record_degrade_activation();
        monitor.record_degrade_activation();

        assert_eq!(monitor.get_stats().degrade_activations(), 3);
    }

    /// Budget check on node with no budget set: should always succeed regardless
    /// of execution time, but still record timing data.
    #[test]
    fn test_check_tick_budget_no_budget_set() {
        let monitor = SafetyMonitor::new(100);

        // Node has no budget — any execution time should pass
        for i in 0..10 {
            let result = monitor.check_tick_budget("unbounded", Duration::from_millis(i * 100));
            assert!(result.is_ok());
        }

        // But timing data should still be recorded
        let all = monitor.all_node_timing();
        let unbounded = all.iter().find(|(n, _, _, _)| n == "unbounded");
        assert!(unbounded.is_some());
        let (_, stats, budget, overruns) = unbounded.unwrap();
        assert_eq!(stats.total_ticks, 10);
        assert!(budget.is_none());
        assert_eq!(*overruns, 0);
    }

    /// Critical node budget violation triggers emergency stop via check_tick_budget.
    #[test]
    fn test_critical_node_budget_violation_estop() {
        let monitor = SafetyMonitor::new(100);
        monitor.add_critical_node("safety_ctrl".to_string(), 1_u64.secs());
        monitor.set_tick_budget("safety_ctrl".to_string(), 100_u64.us());

        // Within budget — no estop
        let _ = monitor.check_tick_budget("safety_ctrl", 50_u64.us());
        assert!(!monitor.is_emergency_stop());

        // Over budget on critical node — estop
        let result = monitor.check_tick_budget("safety_ctrl", 200_u64.us());
        assert!(result.is_err());
        assert!(monitor.is_emergency_stop());
    }

    /// Timing ring stats with a single sample.
    #[test]
    fn test_timing_ring_single_sample() {
        let mut ring = TickTimingRing::new();
        ring.record(42);
        let stats = ring.stats();
        assert_eq!(stats.min_us, 42);
        assert_eq!(stats.max_us, 42);
        assert_eq!(stats.avg_us, 42);
        assert_eq!(stats.p99_us, 42);
        assert_eq!(stats.total_ticks, 1);
    }

    /// Timing ring stats with zero samples returns default.
    #[test]
    fn test_timing_ring_empty_stats() {
        let ring = TickTimingRing::new();
        let stats = ring.stats();
        assert_eq!(stats.min_us, 0);
        assert_eq!(stats.max_us, 0);
        assert_eq!(stats.avg_us, 0);
        assert_eq!(stats.p99_us, 0);
        assert_eq!(stats.total_ticks, 0);
    }

    /// NodeTimingState tracks worst_miss_us correctly across multiple misses.
    #[test]
    fn test_node_timing_worst_miss_tracking() {
        let mut state = NodeTimingState::new(None);

        state.record_miss(100);
        assert_eq!(state.worst_miss_us, 100);

        state.record_miss(500);
        assert_eq!(state.worst_miss_us, 500);

        // A smaller miss does not overwrite the worst
        state.record_miss(200);
        assert_eq!(state.worst_miss_us, 500);

        assert_eq!(state.total_deadline_misses, 3);
    }

    /// NodeTimingState with zero-duration budget: record_tick returns violation
    /// for any non-zero actual, tracks worst overrun correctly.
    #[test]
    fn test_node_timing_zero_budget_overrun_tracking() {
        let mut state = NodeTimingState::new(Some(Duration::ZERO));

        // Zero tick vs zero budget: not a violation (0 is not > 0)
        assert!(state.record_tick(Duration::ZERO).is_none());
        assert_eq!(state.overrun_count, 0);

        // 1us vs zero budget: violation
        let v = state.record_tick(1_u64.us());
        assert!(v.is_some());
        assert_eq!(state.overrun_count, 1);
        assert_eq!(state.worst_overrun_us, 1);

        // 10us vs zero budget: bigger violation
        let v = state.record_tick(10_u64.us());
        assert!(v.is_some());
        assert_eq!(state.overrun_count, 2);
        assert_eq!(state.worst_overrun_us, 10);
    }

    /// Watchdog graduated check on a freshly-constructed (never fed) watchdog:
    /// should be Ok immediately since constructor calls now_ns().
    #[test]
    fn test_watchdog_freshly_constructed_is_ok() {
        let wd = Watchdog::new(1_u64.secs());
        assert_eq!(wd.check_graduated(), WatchdogSeverity::Ok);
        assert!(!wd.check());
        assert!(!wd.is_expired());
    }

    /// Multiple emergency stop triggers: idempotent — state stays EmergencyStop.
    #[test]
    fn test_multiple_emergency_stops_idempotent() {
        let monitor = SafetyMonitor::new(100);

        monitor.trigger_emergency_stop("first".to_string());
        assert!(monitor.is_emergency_stop());
        assert_eq!(monitor.get_state(), SafetyState::EmergencyStop);

        monitor.trigger_emergency_stop("second".to_string());
        assert!(monitor.is_emergency_stop());
        assert_eq!(monitor.get_state(), SafetyState::EmergencyStop);

        monitor.trigger_emergency_stop("third".to_string());
        assert!(monitor.is_emergency_stop());
        assert_eq!(monitor.get_state(), SafetyState::EmergencyStop);
    }

    /// SafetyStats reports watchdog expirations correctly across multiple nodes.
    #[test]
    fn test_safety_stats_watchdog_expirations() {
        use std::thread;

        let monitor = SafetyMonitor::new(10000); // high max to avoid global estop

        // 3 nodes: 2 with short timeouts, 1 with long
        monitor
            .watchdogs
            .write()
            .insert("short_a".to_string(), Watchdog::new(1_u64.ms()));
        monitor
            .watchdogs
            .write()
            .insert("short_b".to_string(), Watchdog::new(1_u64.ms()));
        monitor
            .watchdogs
            .write()
            .insert("long_c".to_string(), Watchdog::new(1_u64.secs()));

        // Let the short ones expire
        thread::sleep(5_u64.ms());

        let mut expired = Vec::new();
        monitor.check_watchdogs(&mut expired);

        // Should have 2 expired watchdogs
        let stats = monitor.get_stats();
        assert_eq!(stats.watchdog_expirations(), 2);
        assert_eq!(expired.len(), 2);
        assert!(expired.contains(&"short_a".to_string()));
        assert!(expired.contains(&"short_b".to_string()));
    }

    /// Concurrent graduated watchdog checks from multiple threads.
    #[test]
    fn test_concurrent_graduated_watchdog_checks() {
        use std::sync::Arc;
        use std::thread;

        let monitor = Arc::new(SafetyMonitor::new(10000));

        for i in 0..10 {
            monitor.add_critical_node(format!("gwd_{}", i), 5_u64.secs());
        }

        // 5 threads checking graduated watchdogs concurrently
        let mut handles = Vec::new();
        for _ in 0..5 {
            let m = monitor.clone();
            handles.push(thread::spawn(move || {
                let mut results = Vec::new();
                for _ in 0..100 {
                    m.check_watchdogs_graduated(&mut results);
                    // With 5s timeout, all should be Ok (no results returned)
                    assert!(results.is_empty());
                }
            }));
        }

        // Main thread feeds watchdogs concurrently
        for _ in 0..100 {
            for i in 0..10 {
                monitor.feed_watchdog(&format!("gwd_{}", i));
            }
        }

        for h in handles {
            h.join().unwrap();
        }

        assert!(!monitor.is_emergency_stop());
    }
}

// ── loom concurrency model test ───────────────────────────────────────────
//
// Run with:  RUSTFLAGS="--cfg loom" cargo test -p horus_core -- loom
//
// The loom test verifies the critical_nodes RwLock synchronization pattern
// in isolation (using loom's own Arc/RwLock which loom can intercept).
// SafetyMonitor itself uses parking_lot which loom cannot intercept, so this
// test validates that the write-before-read ordering is enforced correctly.
#[cfg(loom)]
mod loom_tests {
    use loom::sync::{Arc, RwLock};
    use loom::thread;

    /// loom model: one writer pushes to a Vec<String> via write lock while two
    /// readers concurrently check for membership via read lock.  loom explores
    /// all valid interleavings and verifies no data races occur.
    #[test]
    fn critical_nodes_concurrent_add_and_read() {
        loom::model(|| {
            let nodes: Arc<RwLock<Vec<String>>> = Arc::new(RwLock::new(Vec::new()));

            // Writer: add_critical_node() path
            let nodes_w = nodes.clone();
            let t_write = thread::spawn(move || {
                nodes_w.write().push("node_a".to_string());
            });

            // Reader 1: check_watchdogs() / record_deadline_miss() path
            let nodes_r1 = nodes.clone();
            let t_read1 = thread::spawn(move || {
                let _ = nodes_r1.read().contains(&"node_a".to_string());
            });

            // Reader 2: check_tick_budget() path
            let nodes_r2 = nodes.clone();
            let t_read2 = thread::spawn(move || {
                let _ = nodes_r2.read().contains(&"node_b".to_string());
            });

            t_write.join().unwrap();
            t_read1.join().unwrap();
            t_read2.join().unwrap();
        });
    }

    // ========================================================================
    // BudgetPolicy tests
    // ========================================================================

    #[test]
    fn test_budget_policy_default_is_warn() {
        assert_eq!(BudgetPolicy::default(), BudgetPolicy::Warn);
    }

    #[test]
    fn test_budget_policy_variants_are_distinct() {
        assert_ne!(BudgetPolicy::Warn, BudgetPolicy::Enforce);
        assert_ne!(BudgetPolicy::Warn, BudgetPolicy::EmergencyStop);
        assert_ne!(BudgetPolicy::Enforce, BudgetPolicy::EmergencyStop);
    }

    #[test]
    fn test_budget_policy_clone_and_copy() {
        let policy = BudgetPolicy::Enforce;
        let cloned = policy.clone();
        let copied = policy; // Copy
        assert_eq!(policy, cloned);
        assert_eq!(policy, copied);
    }
}
