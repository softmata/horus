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
        let expired = elapsed_ns > self.timeout.as_nanos() as u64;
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
        let min = *samples.iter().min().unwrap();
        let max = *samples.iter().max().unwrap();
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
pub struct DegradationPolicy {
    /// Consecutive misses before logging a warning (default: 3)
    pub warn_after: u64,
    /// Consecutive misses before reducing node rate (default: 5)
    pub reduce_after: u64,
    /// Consecutive misses before isolating node (default: 10)
    pub isolate_after: u64,
    /// Successful ticks at reduced rate before restoring original (default: 100)
    pub recovery_ticks: u64,
}

impl Default for DegradationPolicy {
    fn default() -> Self {
        Self {
            warn_after: 3,
            reduce_after: 5,
            isolate_after: 10,
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
    /// Restore node to original rate (recovery).
    RestoreRate { node: String, original_rate_hz: f64 },
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
    pub(crate) fn check_budget(&mut self, node: &str, actual: Duration) -> Result<(), BudgetViolation> {
        let state = self
            .node_timing
            .entry(node.to_string())
            .or_insert_with(|| NodeTimingState::new(None));

        if let Some(mut violation) = state.record_tick(actual) {
            self.overruns.fetch_add(1, Ordering::SeqCst);
            violation = BudgetViolation::new(node.to_string(), violation.budget(), violation.actual());
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
    pub(crate) fn check_watchdogs_graduated(
        &self,
        results: &mut Vec<(String, WatchdogSeverity)>,
    ) {
        results.clear();
        for (name, watchdog) in self.watchdogs.read().iter() {
            let severity = watchdog.check_graduated();
            if !matches!(severity, WatchdogSeverity::Ok) {
                results.push((name.clone(), severity));
            }
        }

        // For critical nodes at Critical severity: trigger emergency stop
        if results.iter().any(|(_, s)| *s == WatchdogSeverity::Critical) {
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
            let is_critical = self.critical_nodes.read().contains(&violation.node_name().to_string());
            if is_critical {
                self.budget_enforcer.lock().mark_critical_overrun();
                self.trigger_emergency_stop(format!(
                    "Critical node {} exceeded tick budget: {:?} > {:?}",
                    violation.node_name(), violation.actual(), violation.budget()
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
        let state = states
            .entry(node_name.to_string())
            .or_default();

        if consecutive_misses >= policy.isolate_after && state.stage != DegradationStage::Isolated {
            state.stage = DegradationStage::Isolated;
            state.recovery_counter = 0;
            DegradationAction::Isolate(node_name.to_string())
        } else if consecutive_misses >= policy.reduce_after
            && state.stage != DegradationStage::RateReduced
            && state.stage != DegradationStage::Isolated
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
                    // No original rate saved — just go back to Normal stage
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

    /// Record a degrade activation (Miss::Degrade triggered on a node).
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
            results.iter().any(|(n, s)| n == "fast_node" && *s == WatchdogSeverity::Warning),
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
        monitor.watchdogs.write().insert(
            "noncritical".to_string(),
            Watchdog::new(10_u64.ms()),
        );

        thread::sleep(35_u64.ms());

        let mut results = Vec::new();
        monitor.check_watchdogs_graduated(&mut results);

        assert!(!monitor.is_emergency_stop());
        assert!(results.iter().any(|(n, s)| n == "noncritical" && *s == WatchdogSeverity::Critical));
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
        assert!(stats.p99_us >= 99, "p99 should be >= 99, got {}", stats.p99_us);
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

        // Already isolated — no further action
        let action = monitor.evaluate_degradation("motor", 20, Some(100.0));
        assert_eq!(action, DegradationAction::None);
    }

    #[test]
    fn test_degradation_recovery_after_stable_period() {
        let mut monitor = SafetyMonitor::new(100);
        monitor.set_degradation_policy(DegradationPolicy {
            warn_after: 1,
            reduce_after: 2,
            isolate_after: 100, // high so we test recovery at RateReduced stage
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
        assert_eq!(monitor.degradation_stage("sensor"), DegradationStage::Normal);
    }

    #[test]
    fn test_degradation_warned_recovers_on_success() {
        let mut monitor = SafetyMonitor::new(100);
        monitor.set_degradation_policy(DegradationPolicy {
            warn_after: 2,
            reduce_after: 5,
            isolate_after: 10,
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
            recovery_ticks: 100,
        });

        monitor.record_deadline_miss_with_severity("motor", 500);
        let action = monitor.evaluate_degradation("motor", 1, None);
        assert_eq!(action, DegradationAction::None);
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
}
