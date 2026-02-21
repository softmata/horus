// Safety monitor for real-time critical systems
use parking_lot::Mutex;
use std::collections::HashMap;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant};

/// Safety state of the system
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SafetyState {
    /// Normal operation
    Normal,
    /// Emergency stop triggered
    EmergencyStop,
}

/// Watchdog for monitoring node health
#[derive(Debug)]
pub(crate) struct Watchdog {
    /// Node name being monitored
    node_name: String,
    /// Timeout duration
    timeout: Duration,
    /// Last heartbeat time
    last_heartbeat: Mutex<Instant>,
    /// Is watchdog expired?
    expired: AtomicBool,
}

impl Watchdog {
    pub(crate) fn new(node_name: String, timeout: Duration) -> Self {
        Self {
            node_name,
            timeout,
            last_heartbeat: Mutex::new(Instant::now()),
            expired: AtomicBool::new(false),
        }
    }

    /// Feed the watchdog (reset timer)
    pub(crate) fn feed(&self) {
        *self.last_heartbeat.lock() = Instant::now();
        self.expired.store(false, Ordering::SeqCst);
    }

    /// Check if watchdog has expired
    pub(crate) fn check(&self) -> bool {
        let last = *self.last_heartbeat.lock();
        let expired = last.elapsed() > self.timeout;
        if expired {
            self.expired.store(true, Ordering::SeqCst);
        }
        expired
    }

    pub(crate) fn is_expired(&self) -> bool {
        self.expired.load(Ordering::SeqCst)
    }

    pub(crate) fn node_name(&self) -> &str {
        &self.node_name
    }
}

/// WCET (Worst-Case Execution Time) enforcer
#[derive(Debug)]
pub(crate) struct WCETEnforcer {
    /// WCET budgets per node
    budgets: HashMap<String, Duration>,
    /// Overrun counter
    pub(crate) overruns: AtomicU64,
    /// Critical overruns (that triggered emergency stop)
    critical_overruns: AtomicU64,
}

impl Default for WCETEnforcer {
    fn default() -> Self {
        Self {
            budgets: HashMap::new(),
            overruns: AtomicU64::new(0),
            critical_overruns: AtomicU64::new(0),
        }
    }
}

impl WCETEnforcer {
    pub(crate) fn new() -> Self {
        Self::default()
    }

    /// Set WCET budget for a node
    pub(crate) fn set_budget(&mut self, node: String, budget: Duration) {
        self.budgets.insert(node, budget);
    }

    /// Check if execution time is within budget
    pub(crate) fn check_budget(&self, node: &str, actual: Duration) -> Result<(), WCETViolation> {
        if let Some(&budget) = self.budgets.get(node) {
            if actual > budget {
                self.overruns.fetch_add(1, Ordering::SeqCst);
                let overrun = actual - budget;
                return Err(WCETViolation {
                    node_name: node.to_string(),
                    budget,
                    actual,
                    overrun,
                });
            }
        }
        Ok(())
    }

    pub(crate) fn get_overrun_count(&self) -> u64 {
        self.overruns.load(Ordering::SeqCst)
    }

    pub(crate) fn mark_critical_overrun(&self) {
        self.critical_overruns.fetch_add(1, Ordering::SeqCst);
    }
}

/// WCET violation details
#[derive(Debug, Clone)]
pub struct WCETViolation {
    pub node_name: String,
    pub budget: Duration,
    pub actual: Duration,
    pub overrun: Duration,
}

/// Safety monitor for real-time critical systems
#[derive(Debug)]
pub(crate) struct SafetyMonitor {
    /// Current safety state
    state: Arc<Mutex<SafetyState>>,
    /// Emergency stop flag
    emergency_stop: Arc<AtomicBool>,
    /// Watchdogs for critical nodes
    watchdogs: Arc<Mutex<HashMap<String, Watchdog>>>,
    /// WCET enforcer
    wcet_enforcer: Arc<Mutex<WCETEnforcer>>,
    /// Critical nodes that must never fail
    critical_nodes: Vec<String>,
    /// Deadline miss counter
    deadline_misses: AtomicU64,
    /// Maximum allowed deadline misses before emergency
    max_deadline_misses: u64,
}

impl SafetyMonitor {
    pub(crate) fn new(max_deadline_misses: u64) -> Self {
        Self {
            state: Arc::new(Mutex::new(SafetyState::Normal)),
            emergency_stop: Arc::new(AtomicBool::new(false)),
            watchdogs: Arc::new(Mutex::new(HashMap::new())),
            wcet_enforcer: Arc::new(Mutex::new(WCETEnforcer::new())),
            critical_nodes: Vec::new(),
            deadline_misses: AtomicU64::new(0),
            max_deadline_misses,
        }
    }

    /// Add a critical node that must be monitored
    pub(crate) fn add_critical_node(&mut self, node_name: String, watchdog_timeout: Duration) {
        self.critical_nodes.push(node_name.clone());
        self.watchdogs.lock().insert(
            node_name.clone(),
            Watchdog::new(node_name, watchdog_timeout),
        );
    }

    /// Set WCET budget for a node
    pub(crate) fn set_wcet_budget(&mut self, node_name: String, budget: Duration) {
        self.wcet_enforcer.lock().set_budget(node_name, budget);
    }

    /// Feed watchdog for a node
    pub(crate) fn feed_watchdog(&self, node_name: &str) {
        if let Some(watchdog) = self.watchdogs.lock().get(node_name) {
            watchdog.feed();
        }
    }

    /// Check all watchdogs and return expired ones
    pub(crate) fn check_watchdogs(&self) -> Vec<String> {
        let mut expired = Vec::new();
        for (name, watchdog) in self.watchdogs.lock().iter() {
            if watchdog.check() {
                expired.push(name.clone());
            }
        }

        // If any critical node watchdog expired, trigger emergency stop
        if !expired.is_empty() {
            for node in &expired {
                if self.critical_nodes.contains(node) {
                    self.trigger_emergency_stop(format!("Critical node {} watchdog expired", node));
                    break;
                }
            }
        }

        expired
    }

    /// Check WCET budget for a node
    pub(crate) fn check_wcet(
        &self,
        node_name: &str,
        execution_time: Duration,
    ) -> Result<(), WCETViolation> {
        let result = self
            .wcet_enforcer
            .lock()
            .check_budget(node_name, execution_time);

        if let Err(ref violation) = result {
            // Critical node WCET violation triggers emergency stop
            if self.critical_nodes.contains(&violation.node_name) {
                self.wcet_enforcer.lock().mark_critical_overrun();
                self.trigger_emergency_stop(format!(
                    "Critical node {} exceeded WCET budget: {:?} > {:?}",
                    violation.node_name, violation.actual, violation.budget
                ));
            }
        }

        result
    }

    /// Record a deadline miss
    pub(crate) fn record_deadline_miss(&self, node_name: &str) {
        let misses = self.deadline_misses.fetch_add(1, Ordering::SeqCst) + 1;

        // Critical node deadline miss or too many misses trigger emergency
        if self.critical_nodes.contains(&node_name.to_string()) {
            self.trigger_emergency_stop(format!("Critical node {} missed deadline", node_name));
        } else if misses >= self.max_deadline_misses {
            self.trigger_emergency_stop(format!("Too many deadline misses: {}", misses));
        }
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
            wcet_overruns: self.wcet_enforcer.lock().get_overrun_count(),
            deadline_misses: self.deadline_misses.load(Ordering::SeqCst),
            watchdog_expirations: self
                .watchdogs
                .lock()
                .values()
                .filter(|w| w.is_expired())
                .count() as u64,
        }
    }

    /// Reset counters (for testing or after recovery)
    pub(crate) fn reset_counters(&self) {
        self.deadline_misses.store(0, Ordering::SeqCst);
        self.wcet_enforcer
            .lock()
            .overruns
            .store(0, Ordering::SeqCst);
        *self.state.lock() = SafetyState::Normal;
        self.emergency_stop.store(false, Ordering::SeqCst);
    }
}

/// Safety statistics
#[derive(Debug, Clone)]
pub struct SafetyStats {
    pub state: SafetyState,
    pub wcet_overruns: u64,
    pub deadline_misses: u64,
    pub watchdog_expirations: u64,
}
