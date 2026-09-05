// Safety monitor for real-time critical systems
use crate::core::rt_node::BudgetViolation;
use parking_lot::{Mutex, RwLock};
use std::collections::HashMap;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::Arc;
use std::time::Duration;

// ============================================================================
// Global emergency stop hook — used by horus_net for network link-loss safety
// ============================================================================

type EmergencyStopHook = Box<dyn Fn(String) + Send + Sync>;
// RwLock (not OnceLock): the hook must be RE-SETTABLE so each scheduler `run()`
// wires it to its own monitor — the most recently started scheduler owns external
// e-stop. A OnceLock latched the very first scheduler forever, so a second scheduler
// in the same process (common in tests, and in a supervisor that restarts the graph)
// could never receive a networked e-stop (SCHED-H1).
static EMERGENCY_STOP_HOOK: RwLock<Option<EmergencyStopHook>> = RwLock::new(None);

/// Set a global hook that triggers emergency stop from external systems (e.g., horus_net).
///
/// The hook receives a reason string. Called when network heartbeat detects link loss
/// with `LinkLostAction::Stop`, or when a remote e-stop packet is received. Installed by
/// the Scheduler at startup (see `SafetyMonitor::install_emergency_stop_hook`); last set
/// wins.
pub fn set_emergency_stop_hook(hook: impl Fn(String) + Send + Sync + 'static) {
    *EMERGENCY_STOP_HOOK.write() = Some(Box::new(hook));
}

/// Trigger emergency stop from external systems (e.g., horus_net heartbeat timeout).
///
/// If the hook is set (by the Scheduler at startup), this latches the Scheduler's
/// emergency stop. If not set, it only prints to stderr.
pub fn trigger_external_emergency_stop(reason: String) {
    let guard = EMERGENCY_STOP_HOOK.read();
    if let Some(hook) = guard.as_ref() {
        hook(reason);
    } else {
        drop(guard);
        crate::terminal::print_line(&format!(
            "[horus] External emergency stop (no scheduler): {reason}"
        ));
    }
}

/// Global hook for the SAFE-STATE response, distinct from emergency stop.
///
/// `safety.on_link_lost` offers `warn`, `safe_state` and `stop`. `safe_state`
/// and `stop` both used to call `trigger_external_emergency_stop`, so an
/// operator who deliberately chose the milder option — per-node safing rather
/// than halting the robot — got the full scheduler halt anyway. The only trace
/// of their choice was the `{:?}` in the log line.
///
/// Two named options that do the same thing means one of them is a lie, and on
/// a safety knob that is the defect class this audit kept finding. This hook is
/// the milder response: ask nodes to enter their safe state and keep the
/// scheduler running, so the robot stays observable and controllable.
static SAFE_STATE_HOOK: RwLock<Option<Box<dyn Fn(String) + Send + Sync>>> = RwLock::new(None);

/// Set the global safe-state hook. Installed by the Scheduler at startup.
pub fn set_safe_state_hook(hook: impl Fn(String) + Send + Sync + 'static) {
    *SAFE_STATE_HOOK.write() = Some(Box::new(hook));
}

/// Request the SAFE-STATE response from an external system (e.g. horus_net).
///
/// Unlike [`trigger_external_emergency_stop`] this does NOT latch: nodes are
/// asked to enter a safe state, and the scheduler keeps ticking. If no hook is
/// installed it falls back to the emergency stop, because failing safe is the
/// correct direction when the milder response is unavailable — and it says so,
/// rather than silently downgrading the operator's choice.
pub fn trigger_external_safe_state(reason: String) {
    let guard = SAFE_STATE_HOOK.read();
    if let Some(hook) = guard.as_ref() {
        hook(reason);
    } else {
        drop(guard);
        crate::terminal::print_line(&format!(
            "[horus] safe-state requested but no scheduler hook is installed; \
             escalating to emergency stop: {reason}"
        ));
        trigger_external_emergency_stop(reason);
    }
}

// ============================================================================
// Pending LOCAL-origin e-stop signal — the SEND half of networked e-stop.
// ============================================================================

/// Process-global slot holding this robot's OWN, local-origin e-stop reason while it
/// awaits network broadcast. One SafetyMonitor + one replicator per process, so a
/// process-global is the correct scope. Set ONLY on the false→true rising edge of a
/// local trigger (see `SafetyMonitor::trigger_emergency_stop`); a remote e-stop arrives
/// via the hook which latches the flag directly, consuming the edge, so a received
/// e-stop is never queued for re-broadcast (the #1 anti-storm invariant).
///
/// `std::sync::Mutex` (not `parking_lot`) so poisoning is recoverable via `into_inner`
/// on the safety path — never an unwrap-panic.
static PENDING_LOCAL_ESTOP: std::sync::Mutex<Option<String>> = std::sync::Mutex::new(None);

/// Serialises the tests that latch an emergency stop.
///
/// `PENDING_LOCAL_ESTOP` is a single process-global slot and `take()` empties
/// it, but cargo runs this file's tests concurrently in one binary. Every test
/// that latches a local e-stop writes its reason there on the rising edge, so a
/// neighbour could overwrite the reason a test had just queued, or drain it
/// before that test looked. `trigger_queues_the_fleet_broadcast_on_the_rising_edge`
/// failed intermittently on `main` for exactly that reason -- nothing to do with
/// the invariant it was written to check.
///
/// Every test that reaches `trigger_emergency_stop`, by any route, must hold
/// this. It is not about thread-safety of the code under test, which is fine;
/// it is about two tests sharing one mailbox.
#[cfg(test)]
static ESTOP_QUEUE_SERIAL: std::sync::Mutex<()> = std::sync::Mutex::new(());

#[cfg(test)]
pub(crate) fn estop_queue_guard() -> std::sync::MutexGuard<'static, ()> {
    ESTOP_QUEUE_SERIAL.lock().unwrap_or_else(|e| e.into_inner())
}

/// Drain the pending LOCAL-origin e-stop reason (if any) for network broadcast.
///
/// Called by horus_net's replicator tick. Returns `Some(reason)` exactly once per
/// local e-stop episode; `None` after draining or for remote-origin e-stops.
pub fn take_pending_local_estop() -> Option<String> {
    PENDING_LOCAL_ESTOP
        .lock()
        .unwrap_or_else(|e| e.into_inner())
        .take()
}

/// Monotonic nanoseconds. The clock every watchdog timeout in this module is
/// measured against.
///
/// # Why this is not the wall clock
///
/// This was `SystemTime::now() - UNIX_EPOCH`, defended by a comment arguing
/// that NTP adjustments are microseconds against millisecond timeouts. That is
/// true of NTP *slew* and false of an NTP *step*, and a step is the ordinary
/// case, not the exotic one: `chronyd` and `systemd-timesyncd` both step
/// outright on their first sync rather than slewing an arbitrarily large
/// offset, a VM or container resume lands the guest clock forward by however
/// long it was suspended, and `date -s` is a thing operators do.
///
/// Both directions of a step are a safety failure, and they fail differently:
///
/// * **Forward.** `elapsed_ns` for every registered node jumps by the size of
///   the step in a single pass. A step larger than 3x a node's timeout makes
///   [`Watchdog::check_graduated`] return `Critical` for all of them at once,
///   and `check_watchdogs_graduated` turns the first critical node into
///   `trigger_emergency_stop` — a latched, fleet-wide e-stop on a robot whose
///   every node is ticking perfectly. Nothing about the robot changed; the
///   clock moved.
/// * **Backward.** `check_graduated_at`'s `saturating_sub` clamps the elapsed
///   time to zero, so every watchdog reads `Ok` until real time catches back
///   up with the pre-step reading. The watchdog does not false-trip, it goes
///   *blind*: a node can stop ticking for the length of the step and never be
///   reported. That is the worse of the two failures, because it is silent.
///
/// `CLOCK_MONOTONIC` has neither failure by construction. It is not settable
/// (`clock_settime` returns `EINVAL` for it), NTP can only slew its *rate*, and
/// `date -s` does not touch it. It is the clock a safety timeout has to be
/// measured on, and `rt_executor` already reached the same conclusion for its
/// diagnostic throttle.
///
/// # Epoch
///
/// These timestamps never leave the process: `last_heartbeat_ns` is only ever
/// compared against a later reading of this same function, and the one accessor
/// that exposes it ([`SafetyMonitor::watchdog_last_heartbeat_ns`]) is used only
/// to compare two readings of it. The epoch is therefore free to be arbitrary —
/// boot on unix, first call elsewhere — and no reader can be broken by the
/// change of meaning.
///
/// # Cost
///
/// Unchanged. On Linux `clock_gettime(CLOCK_MONOTONIC)` resolves through the
/// vDSO exactly as the `SystemTime` read it replaces did, so this is a tail fix
/// that costs no cycles.
#[cfg(unix)]
#[inline(always)]
fn now_ns() -> u64 {
    let mut ts = libc::timespec {
        tv_sec: 0,
        tv_nsec: 0,
    };
    // SAFETY: `ts` is a live, writable `timespec` and CLOCK_MONOTONIC is always
    // a valid clock id, so both documented failure modes (EFAULT for a bad
    // pointer, EINVAL for a bad clock id) are unreachable. A failed call would
    // leave `ts` zeroed, which reads as "no time has passed" — the blind
    // direction, not the false-trip direction — but it cannot happen here.
    unsafe {
        libc::clock_gettime(libc::CLOCK_MONOTONIC, &mut ts);
    }
    (ts.tv_sec as u64)
        .wrapping_mul(1_000_000_000)
        .wrapping_add(ts.tv_nsec as u64)
}

/// Monotonic nanoseconds for platforms without `clock_gettime` (Windows).
///
/// `Instant` is monotonic and non-settable on every platform Rust supports
/// (QPC on Windows), which is the whole requirement here — the epoch being
/// process-local costs nothing because these timestamps never leave the
/// process.
#[cfg(not(unix))]
#[inline(always)]
fn now_ns() -> u64 {
    use std::sync::OnceLock;
    static ORIGIN: OnceLock<std::time::Instant> = OnceLock::new();
    ORIGIN
        .get_or_init(std::time::Instant::now)
        .elapsed()
        .as_nanos() as u64
}

/// Safety state of the system
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SafetyState {
    /// Normal operation
    Normal,
    /// A safe-state response is in force: nodes have been asked to enter their
    /// safe state, but the scheduler is still running and still observable.
    ///
    /// Distinct from `EmergencyStop` because `safety.on_link_lost` offers both
    /// `safe_state` and `stop`; collapsing them made the milder choice a lie.
    SafeState,
    /// Emergency stop triggered
    EmergencyStop,
}

/// Policy for handling tick budget violations.
///
/// Controls what happens when a node exceeds its allocated tick budget.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum BudgetPolicy {
    /// Log the violation but take no corrective action (default).
    /// The graduated degradation path will handle it over time.
    #[default]
    Warn,
    /// Stop the node once a tick exceeds **twice** its budget.
    ///
    /// Deliberate hysteresis, and the docs previously described a 1x trigger
    /// that the code does not implement. A single 1x overrun is logged and
    /// recorded but not acted on: under real RT jitter — a page fault, an IRQ,
    /// a cache miss — an occasional 1.0-2.0x tick is normal, and killing a
    /// control node for one of them is a worse outcome than the overrun. A
    /// sustained 2x overrun is a different signal and does stop the node.
    ///
    /// The node has shutdown() called and is permanently removed. Safer than
    /// mid-tick interruption — waits for the tick to complete, then prevents
    /// all future ticks. Use `EmergencyStop` if any overrun must halt the robot.
    Enforce,
    /// Trigger emergency stop on budget violation (for critical nodes).
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
/// `last_heartbeat_ns` is stored as [`now_ns`] nanoseconds — a MONOTONIC
/// reading, never the wall clock, for the reasons set out on that function — in
/// an `AtomicU64`.  This eliminates the `Mutex<Instant>` TOCTOU window that
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
    /// The timeout as configured, before any rate-change scaling. `set_scale`
    /// is always applied to this, so scaling never compounds.
    base_timeout: Duration,
    /// Last heartbeat, as monotonic nanoseconds from [`now_ns`]. Process-local
    /// epoch: only ever compared against another reading of the same clock.
    last_heartbeat_ns: AtomicU64,
    /// Is watchdog expired?
    expired: AtomicBool,
}

impl Watchdog {
    pub(crate) fn new(timeout: Duration) -> Self {
        Self {
            timeout,
            base_timeout: timeout,
            last_heartbeat_ns: AtomicU64::new(now_ns()),
            expired: AtomicBool::new(false),
        }
    }

    /// Widen (or restore) the timeout to track a deliberate change in how
    /// often the node ticks.
    ///
    /// Always relative to the configured value, never compounding, so
    /// repeated calls are idempotent and `scale = 1.0` restores exactly.
    pub(crate) fn set_scale(&mut self, scale: f64) {
        let scaled = self.base_timeout.as_secs_f64() * scale.max(1.0);
        self.timeout = Duration::from_secs_f64(scaled);
    }

    /// Feed the watchdog (reset timer)
    pub(crate) fn feed(&self) {
        self.last_heartbeat_ns.store(now_ns(), Ordering::Release);
        self.expired.store(false, Ordering::SeqCst);
    }

    /// Check if watchdog has expired (simple boolean).
    #[cfg(test)]
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
        self.check_graduated_at(now_ns())
    }

    /// The severity decision, as a pure function of what time it is now.
    ///
    /// Split out so a test can state the elapsed time instead of sleeping for
    /// it. `thread::sleep` guarantees a minimum and never a maximum, so a test
    /// that sleeps 25ms against a 10ms timeout and asserts `Expired` is really
    /// asserting that the machine was not busy: past 30ms the same code is
    /// correctly `Critical`. Three of these tests flipped that way whenever the
    /// suite ran under load, and a suite that goes red for reasons unrelated to
    /// the change is one people stop reading — which is how a fix sat unnoticed
    /// in this repository for 244 commits.
    pub(crate) fn check_graduated_at(&self, now_ns: u64) -> WatchdogSeverity {
        let last_ns = self.last_heartbeat_ns.load(Ordering::Acquire);
        let elapsed_ns = now_ns.saturating_sub(last_ns);
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
        let min = samples.iter().copied().min().unwrap_or(0);
        let max = samples.iter().copied().max().unwrap_or(0);
        let sum: u64 = samples.iter().sum();
        let avg = sum / n as u64;

        // P99: sort on stack-allocated copy (no heap allocation)
        let mut scratch = [0u64; TIMING_RING_CAPACITY];
        scratch[..n].copy_from_slice(samples);
        scratch[..n].sort_unstable();
        let p99_idx = ((n as f64 * 0.99) as usize).min(n - 1);
        let p99 = scratch[p99_idx];

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

/// One node's row in the scheduler's shutdown timing report.
///
/// `deadline_misses` is carried here because it was tracked per node
/// (`NodeTimingState::total_deadline_misses`) but never exposed, so the report
/// printed `stats.total_ticks` under its "Misses" header — a node that shut
/// down after 249 ticks having missed nothing reported 249 misses, and a node
/// that missed exactly one deadline reported 211.
#[derive(Debug, Clone)]
pub(crate) struct NodeTimingReport {
    /// Node name.
    pub(crate) name: String,
    /// Tick-duration statistics from the node's ring buffer.
    pub(crate) stats: TimingStats,
    /// Configured tick budget, if any.
    pub(crate) budget: Option<Duration>,
    /// Ticks that exceeded the budget.
    pub(crate) overruns: u64,
    /// Deadline misses recorded for this node.
    pub(crate) deadline_misses: u64,
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

        // The consecutive-miss run is NOT cleared here. It counts deadline
        // misses, and this function knows only about the budget; clearing it
        // from the budget check pinned the counter at 1 for a node that made
        // its budget and missed its deadline every single tick, which is the
        // dominant failure mode (see `check_deadline_from_release`).
        // `SafetyMonitor::record_deadline_met` owns the reset.
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
    /// Consecutive misses before killing node — permanently removing from
    /// execution (default: 20, raised to `max_deadline_misses` when that
    /// ceiling is higher, so the ceiling stays reachable — see `reaching`)
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

impl DegradationPolicy {
    /// The default ladder, with its terminal rung moved out of the way of a
    /// configured `max_deadline_misses` ceiling.
    ///
    /// `Kill` is permanent — it sets `is_stopped`, and a stopped node is
    /// skipped forever — so a node killed at 20 consecutive misses can never
    /// tick again, and its consecutive count can never climb any higher. With
    /// `kill_after` fixed at 20 and `max_deadline_misses` defaulting to 100,
    /// the documented emergency stop was unreachable in every shipped
    /// configuration: the ladder always retired the node first. Nothing warned,
    /// because the only setter for the ladder is `#[cfg(test)]`, so no user
    /// build could raise the rung either.
    ///
    /// A hardcoded constant must not silently cap a documented safety
    /// threshold, so the rung moves instead. Where the two now coincide the
    /// e-stop wins, because both dispatch paths record the miss — which
    /// evaluates the ceiling — before evaluating the ladder.
    fn reaching(max_deadline_misses: u64) -> Self {
        let base = Self::default();
        Self {
            kill_after: base.kill_after.max(max_deadline_misses),
            ..base
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
        // `entry(node.to_string())` allocated a String on EVERY call, including
        // the hit — and this one runs per tick for every RT node with a budget.
        // It is the same defect the profiler carried, and the note there says
        // why it matters: malloc has no WCET bound (arena contention -> brk/mmap
        // -> page fault), so the check that polices timing had unbounded cost of
        // its own, on the RT thread.
        //
        // `contains_key` + `get_mut` borrows the key instead of owning it: two
        // hash lookups and no allocation in the steady state, against one lookup
        // and a malloc/free pair before. Only a node's FIRST tick allocates.
        if !self.node_timing.contains_key(node) {
            self.node_timing
                .insert(node.to_string(), NodeTimingState::new(None));
        }
        let state = self
            .node_timing
            .get_mut(node)
            .expect("inserted immediately above");

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
        // As `check_budget`: no allocation on the hit path. This runs on the RT
        // thread every time a deadline is missed, and the report of a timing
        // failure must not cost more than the failure did.
        if let Some(state) = self.node_timing.get_mut(node) {
            state.record_miss(severity_us);
            return;
        }
        let mut state = NodeTimingState::new(None);
        state.record_miss(severity_us);
        self.node_timing.insert(node.to_string(), state);
    }

    /// Record that a node MET its deadline, clearing its consecutive-miss run.
    ///
    /// No allocation and no insert on the miss of the map: a node with no entry
    /// has no run to clear, and this runs on the RT thread on every healthy
    /// tick — by far the hottest of these paths.
    pub(crate) fn record_deadline_met(&mut self, node: &str) {
        if let Some(state) = self.node_timing.get_mut(node) {
            state.consecutive_misses = 0;
        }
    }

    /// Get timing stats for a specific node.
    #[cfg(test)]
    pub(crate) fn node_stats(&self, node: &str) -> Option<TimingStats> {
        self.node_timing.get(node).map(|s| s.ring.stats())
    }

    /// Get all node timing states (for timing report).
    pub(crate) fn all_node_stats(&self) -> Vec<NodeTimingReport> {
        self.node_timing
            .iter()
            .map(|(name, state)| NodeTimingReport {
                name: name.clone(),
                stats: state.ring.stats(),
                budget: state.budget,
                overruns: state.overrun_count,
                deadline_misses: state.total_deadline_misses,
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
    /// A NEW external safe-state request the tick loop has not acted on yet.
    ///
    /// Separate from `state`, which latches `SafetyState::SafeState` as an
    /// observable CONDITION and must stay set. This is the EDGE: it is raised
    /// once per external trigger and cleared once the scheduler has driven the
    /// nodes, so one link loss produces one round of safing rather than a fresh
    /// request on every tick for the rest of the run.
    external_safe_state_pending: AtomicBool,
}

/// A cheap, cloneable handle for feeding node watchdogs from another thread.
///
/// Holds a clone of the SAME `Arc<RwLock<HashMap<String, Watchdog>>>` that the
/// owning `SafetyMonitor` iterates in `check_watchdogs()` /
/// `check_watchdogs_graduated()`, so a `feed()` through this handle is observed
/// by those checks — provably the same atomics, not a copy.
///
/// Handed to the executor threads (via `SharedMonitors`) so they can keep their
/// critical nodes' watchdogs alive. Without it, executor-run nodes (which live
/// outside `self.nodes`, the only place the main loop feeds) were never fed, so
/// a HEALTHY RT node's watchdog expired and `check_watchdogs` tripped a spurious
/// fleet-halting emergency stop (FIX #2).
#[derive(Clone, Debug)]
pub(crate) struct WatchdogFeeder {
    watchdogs: Arc<RwLock<HashMap<String, Watchdog>>>,
}

impl WatchdogFeeder {
    /// Feed the watchdog for `node_name`. No-op if the node has no registered
    /// watchdog — identical semantics to `SafetyMonitor::feed_watchdog`.
    pub(crate) fn feed(&self, node_name: &str) {
        if let Some(watchdog) = self.watchdogs.read().get(node_name) {
            watchdog.feed();
        }
    }
}

/// Handle allowing an executor thread to latch the scheduler's emergency stop.
///
/// Counterpart to [`WatchdogFeeder`]: executor threads own their nodes and have
/// no `SafetyMonitor` reference, so without this they cannot report an e-stop —
/// see [`SafetyMonitor::estop_trigger`] for what that silently cost.
#[derive(Clone, Debug)]
pub(crate) struct EstopTrigger {
    emergency_stop: Arc<AtomicBool>,
    state: Arc<Mutex<SafetyState>>,
}

impl EstopTrigger {
    /// Latch the emergency stop. Semantics are identical to
    /// `SafetyMonitor::trigger_emergency_stop`, including the rising-edge gate
    /// that queues a fleet broadcast exactly once per episode.
    pub(crate) fn trigger(&self, reason: String) {
        // Latch FIRST — the safety action must not depend on logging.
        let rising_edge = !self.emergency_stop.swap(true, Ordering::SeqCst);
        *self.state.lock() = SafetyState::EmergencyStop;
        if rising_edge {
            *PENDING_LOCAL_ESTOP
                .lock()
                .unwrap_or_else(|e| e.into_inner()) = Some(reason.clone());
        }
        use std::io::Write;
        let _ = writeln!(std::io::stderr(), " EMERGENCY STOP: {}", reason);
    }
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
            degradation_policy: DegradationPolicy::reaching(max_deadline_misses),
            degradation_states: Mutex::new(HashMap::new()),
            external_safe_state_pending: AtomicBool::new(false),
        }
    }

    /// Set the degradation policy for this safety monitor.
    #[cfg(test)]
    pub(crate) fn set_degradation_policy(&mut self, policy: DegradationPolicy) {
        self.degradation_policy = policy;
    }

    /// Add a critical node that must be monitored.
    ///
    /// Number of nodes registered as critical (i.e. actually watchdogged).
    ///
    /// Exposed so callers can assert the watchdog was really armed: an empty
    /// set means `check_watchdogs` iterates nothing and the configured timeout
    /// is inert, which is indistinguishable from a working watchdog at runtime.
    #[cfg(test)]
    pub fn critical_node_count(&self) -> usize {
        self.critical_nodes.read().len()
    }

    /// Takes `&self` (not `&mut self`) because `critical_nodes` is protected by an
    /// interior `RwLock`; this allows callers holding a shared reference to safely
    /// register new nodes at any time.
    pub(crate) fn add_critical_node(&self, node_name: String, watchdog_timeout: Duration) {
        self.critical_nodes.write().push(node_name.clone());
        self.watchdogs
            .write()
            .insert(node_name, Watchdog::new(watchdog_timeout));
    }

    /// Returns true if `node_name` was registered as a critical node.
    ///
    /// Used by the scheduler to decide how to escalate when a node's safing
    /// callback (`enter_safe_state`/`shutdown`) itself panics: a critical node
    /// that cannot reach a safe state triggers a system-wide emergency stop.
    pub(crate) fn is_critical_node(&self, node_name: &str) -> bool {
        self.critical_nodes.read().iter().any(|n| n == node_name)
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

    /// The current watchdog timeout for a node, after any rate scaling.
    #[cfg(test)]
    pub(crate) fn watchdog_timeout(&self, node_name: &str) -> Option<Duration> {
        self.watchdogs.read().get(node_name).map(|w| w.timeout)
    }

    /// Scale a node's watchdog timeout to match a deliberate change in its
    /// tick rate, and feed it so the new window starts now.
    ///
    /// The watchdog asks "has this node ticked recently", and the tick is what
    /// feeds it — so halving a node's rate halves how often it feeds. Leaving
    /// the timeout fixed turns `DegradationAction::ReduceRate`, the GENTLEST
    /// rung of the ladder, into an escalation: a node whose watchdog margin was
    /// under 2x its period would start tripping 1x, 2x, then Critical, and
    /// Critical latches a fleet-wide emergency stop. Rate-reducing a struggling
    /// node must not be a slower route to halting the robot.
    ///
    /// `scale` is a multiplier on the CONFIGURED timeout (1.0 restores it), and
    /// values below 1.0 are clamped away — this may only ever widen the window,
    /// never tighten it below what the operator asked for.
    pub(crate) fn scale_watchdog(&self, node_name: &str, scale: f64) {
        if let Some(watchdog) = self.watchdogs.write().get_mut(node_name) {
            watchdog.set_scale(scale);
            watchdog.feed();
        }
    }

    /// Produce a trigger handle that shares this monitor's e-stop latch.
    ///
    /// Executor threads own their nodes and never see the `SafetyMonitor`, so
    /// before this existed the RT executor's two emergency-stop branches could
    /// only do `running.store(false)` — a plain shutdown flag. The latch was
    /// never set, so `get_state()` kept reporting `Normal`, no blackbox
    /// `EmergencyStop` event was written, and — worst — `PENDING_LOCAL_ESTOP`
    /// was never populated, so `take_pending_local_estop()` returned `None` and
    /// horus_net never broadcast. **A real RT e-stop halted this robot silently
    /// and never told the fleet.**
    ///
    /// Shares the same `Arc`s the monitor itself uses, so a trigger through the
    /// handle is indistinguishable from one on the main thread — including the
    /// rising-edge gate that decides whether to queue a fleet broadcast.
    pub(crate) fn estop_trigger(&self) -> EstopTrigger {
        EstopTrigger {
            emergency_stop: self.emergency_stop.clone(),
            state: self.state.clone(),
        }
    }

    /// Produce a cheap feed handle that shares this monitor's watchdog map.
    ///
    /// The returned `WatchdogFeeder` clones the very `Arc<RwLock<HashMap>>` that
    /// `check_watchdogs`/`check_watchdogs_graduated` iterate, so feeds through
    /// the handle are seen by those checks. Given to executor threads so they
    /// can feed their critical nodes' watchdogs (FIX #2).
    pub(crate) fn watchdog_feeder(&self) -> WatchdogFeeder {
        WatchdogFeeder {
            watchdogs: self.watchdogs.clone(),
        }
    }

    /// Read a node's last-heartbeat timestamp (ns since the Unix epoch), for
    /// tests that assert whether a watchdog was actually fed. `None` if the
    /// node has no registered watchdog.
    #[cfg(test)]
    pub(crate) fn watchdog_last_heartbeat_ns(&self, node_name: &str) -> Option<u64> {
        self.watchdogs
            .read()
            .get(node_name)
            .map(|w| w.last_heartbeat_ns.load(Ordering::Acquire))
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
    /// Legacy 1x-timeout expiry check.
    ///
    /// No longer on the scheduler path: it triggered a system-wide emergency
    /// stop at 1x while `check_watchdogs_graduated` — which runs in the same
    /// tick — classifies 1x as `Warning` ("log, keep ticking"). Retained for
    /// tests that pin the 1x semantics directly.
    #[cfg(test)]
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
    ///
    /// This is pure accounting: it records the timing sample and reports the
    /// violation to the caller. It does NOT escalate.
    ///
    /// It used to emergency-stop the whole robot whenever the overrunning node
    /// was in `critical_nodes`. That set is populated by
    /// `Scheduler::apply_safety_config` for EVERY RT node as soon as any
    /// `.watchdog()` is configured, and `NodeRegistration::finalize` derives a
    /// tick budget of 80% of the period for every `.rate()` node — so a single
    /// microsecond of ordinary RT jitter latched a fleet-wide e-stop, silently
    /// overriding the node's own `BudgetPolicy::Warn` (the documented default:
    /// "log the violation but take no corrective action"). `critical_nodes`
    /// means "has a watchdog", not "any timing blip is fatal".
    ///
    /// Escalation belongs to the paths that actually read configuration: the
    /// `BudgetPolicy` dispatch in `rt_executor.rs` and
    /// `Scheduler::check_timing_violations`, the graduated ladder via
    /// `evaluate_degradation`, and the 3x-timeout watchdog e-stop in
    /// `check_watchdogs_graduated`.
    pub(crate) fn check_tick_budget(
        &self,
        node_name: &str,
        execution_time: Duration,
    ) -> Result<(), BudgetViolation> {
        let result = self
            .budget_enforcer
            .lock()
            .check_budget(node_name, execution_time);

        // Still counted for reporting — only the escalation is gone.
        if result.is_err() && self.critical_nodes.read().contains(&node_name.to_string()) {
            self.budget_enforcer.lock().mark_critical_overrun();
        }

        result
    }

    /// Record that a node met its deadline, clearing its consecutive-miss run.
    ///
    /// The counterpart to `record_deadline_miss`, and the ONLY thing that
    /// clears the run. Call it on every tick whose deadline check came back
    /// clean, from both dispatch paths.
    ///
    /// This used to be a side effect of `check_budget`, which is the wrong
    /// contract twice over. A budget overrun and a deadline miss are different
    /// failures — a node woken late can blow its deadline having executed in a
    /// tenth of its budget — so an in-budget tick is no evidence the deadline
    /// was met. And neither production caller ran it on a healthy tick anyway:
    /// the main loop calls `check_tick_budget` only from inside the
    /// budget-violation branch, and a node with a deadline and no budget skips
    /// that block entirely, so on those two paths nothing ever cleared the
    /// count and `max_deadline_misses` was a lifetime total wearing the name of
    /// a consecutive one.
    pub(crate) fn record_deadline_met(&self, node_name: &str) {
        self.budget_enforcer.lock().record_deadline_met(node_name);
    }

    /// Record a deadline miss with severity tracking.
    ///
    /// Tracks per-node: total misses, consecutive misses, worst miss duration.
    /// Triggers an emergency stop only when THIS node's CONSECUTIVE run reaches
    /// `max_deadline_misses`; the run is cleared by `record_deadline_met` on
    /// any tick that met its deadline. The process-wide `deadline_misses` total
    /// is kept for the timing report and triggers nothing — it used to be the
    /// e-stop trigger, and as a lifetime count across every node that was
    /// indefensible on a real robot. See the note in
    /// `record_deadline_miss_with_severity`.
    pub(crate) fn record_deadline_miss(&self, node_name: &str) {
        self.record_deadline_miss_with_severity(node_name, 0);
    }

    /// Record a deadline miss with a specific severity (how far past deadline, in μs).
    ///
    /// Like `check_tick_budget`, this used to e-stop immediately whenever the
    /// missing node was in `critical_nodes` — i.e. on the first miss of any
    /// node reachable from a `.watchdog()` call. That contradicted
    /// `Miss::Warn` (the `#[default]`, documented "log warning and continue
    /// normally") and made the whole graduated ladder dead code, since
    /// `DegradationPolicy` only starts acting at `warn_after: 3` consecutive
    /// misses. Per-node escalation is the caller's job, via the `Miss` policy
    /// dispatch and `evaluate_degradation`; only the `max_deadline_misses`
    /// ceiling — an explicitly configured number — stops the robot here.
    pub(crate) fn record_deadline_miss_with_severity(&self, node_name: &str, severity_us: u64) {
        // Process-wide total, kept for the timing report only. It used to be
        // the e-stop trigger, and as a LIFETIME count across EVERY node that
        // was indefensible on a real robot: `max_deadline_misses` defaults to
        // 100, so a 1 kHz arm that missed its deadline once every few minutes
        // and recovered immediately each time -- 0.0028% of its ticks over an
        // hour, an exceptionally healthy control loop -- accumulated its way to
        // a full emergency stop. Nothing reset it, so a long enough run halted
        // regardless of health, and one badly tuned node spent the whole
        // robot's budget.
        self.deadline_misses.fetch_add(1, Ordering::SeqCst);

        // The ceiling is now per node and CONSECUTIVE, which is the thing an
        // operator setting a "maximum deadline misses" actually means: this node
        // is not keeping up right now. A run of misses is a node in trouble; a
        // scattered few that each recover are jitter, and jitter on a stock
        // kernel is not a reason to stop a robot. `consecutive_misses` is reset
        // by `record_deadline_met` on any tick that MET its deadline, on both
        // dispatch paths and whether or not the node has a budget, so recovery
        // genuinely clears the count.
        let consecutive = {
            let mut enforcer = self.budget_enforcer.lock();
            enforcer.record_deadline_miss(node_name, severity_us);
            enforcer
                .node_timing
                .get(node_name)
                .map(|s| s.consecutive_misses)
                .unwrap_or(0)
        };

        if consecutive >= self.max_deadline_misses {
            self.trigger_emergency_stop(format!(
                "node '{}' missed its deadline {} times consecutively (limit {})",
                node_name, consecutive, self.max_deadline_misses
            ));
        }
    }

    /// Get per-node timing stats (for timing report).
    pub(crate) fn all_node_timing(&self) -> Vec<NodeTimingReport> {
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
        // As `check_budget`: the RT executor calls this after every deadline
        // miss, so the hit path must not allocate.
        if !states.contains_key(node_name) {
            states.insert(node_name.to_string(), Default::default());
        }
        let state = states
            .get_mut(node_name)
            .expect("inserted immediately above");

        // Every caller reaches this only after recording a deadline miss, so
        // the run of recovery ticks is over. `record_successful_tick` means
        // "the tick did not panic" and both callers reach it on a tick already
        // recorded as a miss in the same pass; without this, a node that missed
        // EVERY deadline still accumulated `recovery_ticks` of "success" and
        // was restored to the rate it had just proved it could not hold. The
        // field's own comment already claimed to be a consecutive run.
        state.recovery_counter = 0;

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
        } else if consecutive_misses >= policy.warn_after && state.stage == DegradationStage::Normal
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

    /// Trigger emergency stop.
    ///
    /// The safety-critical latch (flag + state) is performed BEFORE any fallible
    /// I/O. This is the single sink for every safety trigger (watchdog expiry,
    /// 3x timeout, budget overrun, deadline miss, miss ceiling, external hook), so
    /// a broken stderr — EPIPE from a restarted log supervisor, ENOSPC on a full
    /// disk, EIO on a dropped pty — must never prevent the e-stop from engaging.
    /// `eprintln!` panics on a stderr write error; latching first and logging last
    /// (non-fatally, via `let _ = writeln!`) guarantees the flag is set regardless.
    pub(crate) fn trigger_emergency_stop(&self, reason: String) {
        // Latch FIRST — the safety-critical action must not depend on logging.
        //
        // Rising-edge gate: swap returns the PRIOR value. Only the call that
        // transitions false→true is a genuine LOCAL-origin e-stop that must be
        // announced to the fleet. A remote e-stop arrives via the hook
        // (install_emergency_stop_hook) which store()s true directly — consuming the
        // edge — so when THIS robot's own watchdog later starves and calls us, swap
        // returns true (already latched) → NOT a rising edge → no PENDING set → no
        // re-broadcast (the #1 anti-storm invariant). The latch still ends `true`
        // unconditionally (swap sets true); only the PENDING queueing is gated.
        let rising_edge = !self.emergency_stop.swap(true, Ordering::SeqCst);
        *self.state.lock() = SafetyState::EmergencyStop; // unchanged, idempotent
        if rising_edge {
            *PENDING_LOCAL_ESTOP
                .lock()
                .unwrap_or_else(|e| e.into_inner()) = Some(reason.clone());
        }
        // Log LAST and non-fatally — a stderr write failure must not unwind here.
        use std::io::Write;
        let _ = writeln!(std::io::stderr(), " EMERGENCY STOP: {}", reason);
    }

    /// Check if emergency stop is active
    pub(crate) fn is_emergency_stop(&self) -> bool {
        self.emergency_stop.load(Ordering::SeqCst)
    }

    /// Wire the global external-emergency-stop hook (used by horus_net link-loss /
    /// remote e-stop) to THIS monitor's stop flag. Without it the hook is never
    /// installed, so `trigger_external_emergency_stop` only printed to stderr and a
    /// networked e-stop silently did nothing (SCHED-H1). Mirrors
    /// `trigger_emergency_stop`: latch the flag FIRST (safety-critical), log LAST and
    /// non-fatally. The captured `Arc`s point at the same flag/state the scheduler's
    /// tick loop polls, so the external trigger latches the running scheduler.
    /// Install the SAFE-STATE hook: mark the monitor degraded and let the
    /// scheduler's tick loop safe the affected nodes, without latching e-stop.
    ///
    /// Deliberately does NOT set `emergency_stop`: that flag ends the run loop,
    /// which is exactly the outcome an operator choosing `safe_state` over
    /// `stop` asked to avoid.
    pub(crate) fn install_safe_state_hook(self: &Arc<Self>) {
        let state = Arc::clone(&self.state);
        let pending = Arc::clone(self);
        set_safe_state_hook(move |reason| {
            *state.lock() = SafetyState::SafeState;
            pending
                .external_safe_state_pending
                .store(true, Ordering::Release);
            use std::io::Write;
            // The claim in this line is now true. The scheduler's tick loop
            // consumes the state via `take_external_safe_state` and drives every
            // node to `enter_safe_state()`; until it did, this message was the
            // only thing that happened.
            let _ = writeln!(
                std::io::stderr(),
                " SAFE STATE (external): {reason} — nodes safing, scheduler continues"
            );
        });
    }

    pub(crate) fn install_emergency_stop_hook(&self) {
        let emergency_stop = Arc::clone(&self.emergency_stop);
        let state = Arc::clone(&self.state);
        set_emergency_stop_hook(move |reason| {
            emergency_stop.store(true, Ordering::SeqCst);
            *state.lock() = SafetyState::EmergencyStop;
            use std::io::Write;
            let _ = writeln!(std::io::stderr(), " EMERGENCY STOP (external): {reason}");
        });
    }

    /// Consume a pending EXTERNAL safe-state request, if one was raised.
    ///
    /// `install_safe_state_hook` used to set `SafetyState::SafeState` and print
    /// "nodes safing, scheduler continues" -- and nothing anywhere read that
    /// state. The only write was the hook's own; `grep SafetyState::` outside
    /// this file returned nothing. So `safety.on_link_lost = "safe_state"` told
    /// the operator the robot was safing and safed no node: a robot that lost
    /// its link to the fleet controller kept executing its last command, with a
    /// reassuring line on stderr.
    ///
    /// The state is cleared on read so one link-loss produces one round of
    /// safing rather than a safing request on every tick forever.
    pub(crate) fn take_external_safe_state(&self) -> bool {
        self.external_safe_state_pending
            .swap(false, Ordering::AcqRel)
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
        let _estop_serial = super::estop_queue_guard();
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

    // ── Emergency-stop latch robustness ──────────────────────────────────

    /// Regression: the emergency-stop latch MUST engage even when stderr writes
    /// fail. `trigger_emergency_stop` used to `eprintln!` BEFORE latching the
    /// flag/state; `eprintln!` panics on a stderr write error (EPIPE/ENOSPC/EIO),
    /// so a broken stderr (restarted log supervisor, full disk, dropped pty)
    /// skipped the latch and the robot kept running under the fault that demanded
    /// the stop. This runs a child whose stderr is /dev/full (every write fails
    /// with ENOSPC) and asserts the e-stop still latched. Pre-fix the child aborts
    /// (double panic); post-fix it latches, ignores the write error, and exits 0.
    #[test]
    fn emergency_stop_latches_even_when_stderr_write_fails() {
        let _estop_serial = super::estop_queue_guard();
        // Child branch: parent redirected our stderr to /dev/full before exec.
        if std::env::var("HORUS_ESTOP_STDERR_CHILD").is_ok() {
            let monitor = SafetyMonitor::new(100);
            monitor.trigger_emergency_stop("stderr-broken regression".to_string());
            std::process::exit(if monitor.is_emergency_stop() { 0 } else { 3 });
        }

        // Parent branch: re-exec ourselves running only this test with stderr
        // pointed at /dev/full. Skip gracefully if /dev/full is unavailable.
        let devfull = match std::fs::OpenOptions::new().write(true).open("/dev/full") {
            Ok(f) => f,
            Err(_) => return,
        };
        let exe = std::env::current_exe().expect("current_exe");
        let status = std::process::Command::new(exe)
            .args([
                "--exact",
                "scheduling::safety_monitor::tests::emergency_stop_latches_even_when_stderr_write_fails",
                "--nocapture",
            ])
            .env("HORUS_ESTOP_STDERR_CHILD", "1")
            .stderr(std::process::Stdio::from(devfull))
            .stdout(std::process::Stdio::null())
            .status()
            .expect("spawn child");
        assert!(
            status.success(),
            "emergency stop must latch even when stderr writes fail (child exit {:?})",
            status.code()
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

    /// Inverted: this test used to assert that one deadline miss by a node in
    /// `critical_nodes` latched an emergency stop. `critical_nodes` only means
    /// "this node has a watchdog" — every RT node lands there as soon as any
    /// `.watchdog()` is configured — so that made a single tick of jitter
    /// fatal and overrode the node's own `Miss::Warn` policy. A watchdogged
    /// node is not a must-never-miss node; escalation is the caller's, via the
    /// Miss policy, the degradation ladder, or the `max_deadline_misses`
    /// ceiling.
    #[test]
    fn test_critical_node_deadline_miss_does_not_trigger_emergency_stop() {
        let _estop_serial = super::estop_queue_guard();
        let monitor = SafetyMonitor::new(100);
        monitor.add_critical_node("critical".to_string(), 1_u64.secs());

        assert!(!monitor.is_emergency_stop());
        monitor.record_deadline_miss("critical");
        assert!(
            !monitor.is_emergency_stop(),
            "A single deadline miss by a watchdogged node must not e-stop the robot"
        );
    }

    /// Regression for the same defect on the budget path: a watchdogged node
    /// that overruns its tick budget once — the common case, since
    /// `NodeRegistration::finalize` derives a budget of 80% of the period for
    /// every `.rate()` node — must be reported to the caller and must NOT
    /// escalate on its own. With `BudgetPolicy::Warn` / `Miss::Warn` the
    /// graduated ladder is what eventually reacts, and only at `warn_after`.
    #[test]
    fn test_watchdogged_node_survives_first_budget_overrun() {
        let monitor = SafetyMonitor::new(100);
        monitor.add_critical_node("motor".to_string(), 500_u64.ms());
        monitor.set_tick_budget("motor".to_string(), 800_u64.us());

        // 810µs against an 800µs budget: ordinary RT jitter.
        let result = monitor.check_tick_budget("motor", 810_u64.us());
        assert!(result.is_err(), "the overrun must still be reported");
        assert!(
            !monitor.is_emergency_stop(),
            "a single budget overrun must not e-stop the robot"
        );

        // The ladder stays quiet below warn_after (default 3) and only warns
        // when it is reached.
        assert!(matches!(
            monitor.evaluate_degradation("motor", 1, Some(1000.0)),
            DegradationAction::None
        ));
        assert!(matches!(
            monitor.evaluate_degradation("motor", 3, Some(1000.0)),
            DegradationAction::Warn(_)
        ));
        assert!(!monitor.is_emergency_stop());
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

    // ── Watchdog clock source ────────────────────────────────────────────

    /// The watchdog's clock must not be a settable one.
    ///
    /// `now_ns` used to be `SystemTime::now() - UNIX_EPOCH`. On that clock an
    /// NTP step, a VM resume or `date -s` moves `elapsed_ns` for every node at
    /// once: forward past 3x a timeout latches a fleet-wide e-stop on a healthy
    /// robot via `check_watchdogs_graduated`, and backward makes
    /// `check_graduated_at`'s `saturating_sub` clamp to zero so the watchdog
    /// stops reporting real deaths.
    ///
    /// A test cannot step the system clock, so it pins the property that makes
    /// the step harmless instead: the reading is not wall-clock time. Unix-epoch
    /// nanoseconds passed 1.7e18 in 2024 and only climb; `CLOCK_MONOTONIC`
    /// counts from boot, so reaching that value takes 54 years of uptime. Any
    /// revert to `SystemTime` fails here immediately.
    #[test]
    fn the_watchdog_clock_is_monotonic_not_wall_clock() {
        // Unix-epoch nanoseconds at 2024-01-01. Every wall-clock reading this
        // code will ever see is above it; no plausible uptime is.
        const EPOCH_NS_2024: u64 = 1_704_067_200_000_000_000;

        let t0 = now_ns();
        assert!(
            t0 < EPOCH_NS_2024,
            "now_ns() returned {t0}, which is wall-clock time since the Unix \
             epoch, not a monotonic reading. A clock step can now latch a \
             fleet-wide emergency stop on a healthy robot, or blind the \
             watchdog to a real node death."
        );

        let t1 = now_ns();
        assert!(
            t1 >= t0,
            "the watchdog clock went backwards ({t0} then {t1}); every timeout \
             in this module is a subtraction of two of these readings"
        );
    }

    /// A fed watchdog stays `Ok` when the *wall* clock is far past its timeout.
    ///
    /// The two clocks are now different quantities, and this is the assertion
    /// that says so at the level the watchdog actually operates: feed a
    /// 10 ms watchdog, then evaluate it at a `SystemTime`-derived "now". On the
    /// old clock that argument was directly comparable to `last_heartbeat_ns`
    /// and the reading is nine decimal orders past the timeout, so the node
    /// would be `Critical` — the exact shape of the spurious e-stop. On a
    /// monotonic clock the wall-clock number is simply not a reading of this
    /// timebase, and `check_graduated_at` must be given one that is.
    #[test]
    fn a_wall_clock_reading_is_not_a_watchdog_reading() {
        let wd = Watchdog::new(10_u64.ms());
        wd.feed();

        let wall_ns = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap_or_default()
            .as_nanos() as u64;
        let fed_at = wd.last_heartbeat_ns.load(Ordering::Acquire);
        assert!(
            wall_ns.saturating_sub(fed_at) > 10_000_000 * 3,
            "precondition: the wall-clock reading is more than 3x the timeout \
             away from the heartbeat, so it would read Critical if the two were \
             the same timebase"
        );

        // The watchdog's own clock, which is the only one it may be judged on.
        assert_eq!(
            wd.check_graduated(),
            WatchdogSeverity::Ok,
            "a watchdog fed microseconds ago must be Ok no matter what the wall \
             clock says"
        );
    }

    // ── Graduated watchdog tests ─────────────────────────────────────────

    /// Watchdog returns Ok immediately after feed.
    #[test]
    fn test_watchdog_graduated_ok_after_feed() {
        let wd = Watchdog::new(50_u64.ms());
        wd.feed();
        assert_eq!(wd.check_graduated(), WatchdogSeverity::Ok);
    }

    /// The graduated bands, stated rather than slept for.
    ///
    /// These four used `thread::sleep` and asserted the band they expected to
    /// land in. Sleep has a floor, not a ceiling: under load a 25ms sleep
    /// against a 10ms timeout overshoots 3x and `Expired` becomes `Critical`,
    /// which is the code being right and the test being wrong. Reproduced by
    /// running the suite against 20 spinners on a 12-core box —
    /// `test_watchdog_graduated_feed_resets` failed with `left: Critical,
    /// right: Expired`.
    ///
    /// `check_graduated_at` takes the clock as an argument, so the elapsed time
    /// is now stated exactly. The bands are the assertion; the machine's mood
    /// is not.
    #[test]
    fn the_graduated_bands_follow_the_elapsed_time() {
        let wd = Watchdog::new(10_u64.ms());
        wd.feed();
        let fed_at = wd.last_heartbeat_ns.load(Ordering::Acquire);
        let ms = |n: u64| fed_at + n * 1_000_000;

        // Boundaries are inclusive at the top of each band: `elapsed <= 1x` is
        // Ok, `<= 2x` Warning, `<= 3x` Expired, beyond that Critical.
        assert_eq!(wd.check_graduated_at(ms(0)), WatchdogSeverity::Ok);
        assert_eq!(wd.check_graduated_at(ms(10)), WatchdogSeverity::Ok);
        assert_eq!(wd.check_graduated_at(ms(15)), WatchdogSeverity::Warning);
        assert_eq!(wd.check_graduated_at(ms(20)), WatchdogSeverity::Warning);
        assert_eq!(wd.check_graduated_at(ms(25)), WatchdogSeverity::Expired);
        assert_eq!(wd.check_graduated_at(ms(30)), WatchdogSeverity::Expired);
        assert_eq!(wd.check_graduated_at(ms(35)), WatchdogSeverity::Critical);
        assert_eq!(wd.check_graduated_at(ms(1000)), WatchdogSeverity::Critical);
    }

    /// A clock that goes backwards must not read as a fresh heartbeat.
    ///
    /// `saturating_sub` makes a `now` before the last feed elapse zero, i.e.
    /// `Ok`. Worth pinning: on a machine whose clock steps back, the failure
    /// this watchdog exists to catch would otherwise be reported as health.
    #[test]
    fn a_clock_that_steps_backwards_reads_as_ok_not_as_a_miss() {
        let wd = Watchdog::new(10_u64.ms());
        wd.feed();
        let fed_at = wd.last_heartbeat_ns.load(Ordering::Acquire);
        assert_eq!(
            wd.check_graduated_at(fed_at.saturating_sub(5_000_000)),
            WatchdogSeverity::Ok
        );
    }

    /// Feed resets graduated severity back to Ok.
    #[test]
    fn test_watchdog_graduated_feed_resets() {
        let wd = Watchdog::new(10_u64.ms());
        wd.feed();
        let fed_at = wd.last_heartbeat_ns.load(Ordering::Acquire);
        assert_eq!(
            wd.check_graduated_at(fed_at + 25_000_000),
            WatchdogSeverity::Expired
        );

        wd.feed();
        let refed_at = wd.last_heartbeat_ns.load(Ordering::Acquire);
        assert_eq!(wd.check_graduated_at(refed_at), WatchdogSeverity::Ok);
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

        // A tick inside budget is NOT a reset: this state knows only about the
        // budget, and a node released late misses its deadline having executed
        // well inside it. Clearing here pinned the run at 1 for exactly that
        // node. Only a met DEADLINE ends the run, via `record_deadline_met`.
        state.record_tick(10_u64.us());
        assert_eq!(state.consecutive_misses, 3);
        assert!(state.is_chronic());

        state.consecutive_misses = 0; // what `record_deadline_met` does
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
        enforcer.check_budget("motor", 80_u64.us()).unwrap();

        // Over budget
        assert!(enforcer.check_budget("motor", 150_u64.us()).is_err());
        assert_eq!(enforcer.get_overrun_count(), 1);

        // No budget set — should always succeed
        enforcer.check_budget("planner", 9999_u64.us()).unwrap();
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
        assert_eq!(action, DegradationAction::Deisolate("motor".to_string()));
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
        assert_eq!(action, DegradationAction::Kill("stalled".to_string()));
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
        assert_eq!(monitor.degradation_stage("arm"), DegradationStage::Warned);

        // Motor recovers while arm stays warned
        for _ in 0..3 {
            monitor.record_successful_tick("motor");
        }
        assert_eq!(
            monitor.degradation_stage("motor"),
            DegradationStage::RateReduced
        );
        assert_eq!(monitor.degradation_stage("arm"), DegradationStage::Warned);

        // Arm recovers independently
        let action = monitor.record_successful_tick("arm");
        assert_eq!(action, DegradationAction::None);
        assert_eq!(monitor.degradation_stage("arm"), DegradationStage::Normal);
    }

    // ========================================================================
    // Emergency stop cleanup
    // ========================================================================

    #[test]
    fn test_emergency_stop_sets_flag_and_state() {
        let _estop_serial = super::estop_queue_guard();
        let monitor = SafetyMonitor::new(100);
        assert!(!monitor.is_emergency_stop());

        monitor.trigger_emergency_stop("test reason".to_string());

        assert!(monitor.is_emergency_stop());
    }

    /// The defect this change exists for: a healthy robot must not e-stop just
    /// for running a long time.
    ///
    /// `deadline_misses` was a process-wide count that NOTHING ever reset, and
    /// `max_deadline_misses` defaults to 100. So a 1 kHz arm that missed its
    /// deadline once every few minutes and recovered immediately -- 0.0028% of
    /// its ticks over an hour, an exceptionally healthy control loop -- ground
    /// its way to a full emergency stop, and a longer run would always get
    /// there eventually whatever its health. That is a timer wearing a safety
    /// threshold's name.
    ///
    /// The configured `max_deadline_misses` ceiling must be reachable.
    ///
    /// `DegradationAction::Kill` sets `is_stopped`, and a stopped node is
    /// skipped forever on both dispatch paths — so a node the ladder kills at
    /// 20 consecutive misses can never tick again, and its consecutive count
    /// can never climb any higher. With `kill_after` hardcoded at 20 and the
    /// ceiling defaulting to 100, the emergency stop this knob names could not
    /// fire in ANY shipped configuration, and the ladder's only setter is
    /// `#[cfg(test)]`, so no user build could raise the rung either.
    ///
    /// The assertion is on the ORDER of the two terminal events, not on the
    /// counter: at this level nothing removes the node, so the count keeps
    /// climbing and the e-stop would fire on its own. What made the ceiling
    /// dead is the `Kill` that lands first.
    #[test]
    fn the_ladder_does_not_retire_a_node_before_its_configured_ceiling() {
        let _estop_serial = super::estop_queue_guard();
        // Above the ladder's default terminal rung of 20 — as the shipped
        // default of 100 also is.
        let ceiling = 25u64;
        let monitor = SafetyMonitor::new(ceiling);

        for i in 1..=ceiling {
            monitor.record_deadline_miss("arm_controller");
            let consecutive = monitor.consecutive_misses("arm_controller");
            let action = monitor.evaluate_degradation("arm_controller", consecutive, Some(100.0));

            if !monitor.is_emergency_stop() {
                assert!(
                    !matches!(action, DegradationAction::Kill(_)),
                    "the ladder killed the node at {} consecutive misses. It can \
                     never tick again, so it can never reach the configured \
                     ceiling of {} and the emergency stop that number names is \
                     unreachable",
                    i,
                    ceiling
                );
            }
        }

        assert!(
            monitor.is_emergency_stop(),
            "{} consecutive misses is the configured ceiling — the emergency \
             stop must fire",
            ceiling
        );
    }

    /// The same thing at the value users actually ship: the documented default.
    #[test]
    fn the_default_ceiling_is_reachable() {
        let _estop_serial = super::estop_queue_guard();
        let default_ceiling = super::super::config::SchedulerConfig::default()
            .realtime
            .max_deadline_misses;
        let monitor = SafetyMonitor::new(default_ceiling);

        for i in 1..=default_ceiling {
            monitor.record_deadline_miss("arm_controller");
            let consecutive = monitor.consecutive_misses("arm_controller");
            let action = monitor.evaluate_degradation("arm_controller", consecutive, Some(100.0));
            if !monitor.is_emergency_stop() {
                assert!(
                    !matches!(action, DegradationAction::Kill(_)),
                    "the default ladder retired the node at {} of the default \
                     ceiling's {} consecutive misses",
                    i,
                    default_ceiling
                );
            }
        }

        assert!(monitor.is_emergency_stop());
    }

    /// Here one node misses ten times the ceiling, recovering between each. It
    /// must not stop the robot.
    #[test]
    fn a_node_that_recovers_between_misses_never_reaches_the_ceiling() {
        let _estop_serial = super::estop_queue_guard();
        let ceiling = 5u64;
        let monitor = SafetyMonitor::new(ceiling);
        monitor.set_tick_budget("wheel_controller".to_string(), Duration::from_millis(1));

        for _ in 0..(ceiling * 10) {
            monitor.record_deadline_miss("wheel_controller");
            // The next tick MET its deadline, which is what ends the run. This
            // used to be spelled `check_tick_budget(.., in_budget_duration)`,
            // asserting a contract the runtime does not hold: an in-budget tick
            // is not a met deadline, and neither production caller reached that
            // call on a healthy tick anyway.
            monitor.record_deadline_met("wheel_controller");
        }

        assert!(
            !monitor.is_emergency_stop(),
            "{} misses, each immediately recovered, must not e-stop a robot \
             whose ceiling is {} consecutive",
            ceiling * 10,
            ceiling
        );
    }

    /// Watchdog membership is not an escalation trigger, so neither a
    /// watchdogged nor a plain node e-stops on a single miss. The e-stop comes
    /// from the configured `max_deadline_misses` ceiling — reached by ONE node,
    /// consecutively.
    ///
    /// The earlier version of this test summed a miss on `arm_controller` with
    /// two on `balance_controller` to reach a ceiling of 3. That is the
    /// process-wide reading: one node's misses spent another node's safety
    /// budget, so a chatty logger could halt an arm.
    #[test]
    fn test_deadline_misses_estop_only_at_the_configured_ceiling() {
        let _estop_serial = super::estop_queue_guard();
        let monitor = SafetyMonitor::new(3);
        monitor.add_critical_node("balance_controller".to_string(), Duration::from_millis(100));

        monitor.record_deadline_miss("arm_controller");
        assert!(!monitor.is_emergency_stop());

        // A watchdogged node missing once is not fatal either.
        monitor.record_deadline_miss("balance_controller");
        assert!(!monitor.is_emergency_stop());

        // A second consecutive miss on it is still under the ceiling of 3 — and
        // the unrelated `arm_controller` miss does not count toward it.
        monitor.record_deadline_miss("balance_controller");
        assert!(
            !monitor.is_emergency_stop(),
            "arm_controller's miss must not count toward balance_controller's ceiling"
        );

        // Its third consecutive miss reaches the ceiling.
        monitor.record_deadline_miss("balance_controller");
        assert!(monitor.is_emergency_stop());
    }

    /// The consecutive-miss run must count DEADLINE misses, not budget overruns.
    ///
    /// This is the RT executor's exact call order for the failure mode
    /// `check_deadline_from_release` was written to catch: a node woken late
    /// that then executes well inside its budget. `rt_executor.rs` calls
    /// `check_tick_budget` unconditionally (:1273) and the deadline check
    /// afterwards (:1356), so every one of these ticks is in budget and every
    /// one misses its deadline.
    ///
    /// The reset used to live in `check_budget`, so the in-budget tick cleared
    /// the run the deadline miss had just started: the counter was pinned at 1
    /// forever. Neither `warn_after: 3` nor the `max_deadline_misses` ceiling
    /// could ever fire for the failure mode that, by `primitives.rs`'s own
    /// account, "actually dominates".
    #[test]
    fn an_in_budget_tick_does_not_clear_a_deadline_miss_run() {
        let monitor = SafetyMonitor::new(1000);
        monitor.set_tick_budget("arm_controller".to_string(), Duration::from_millis(1));

        for _ in 0..10 {
            // Well inside the 1 ms budget — the budget check sees nothing wrong.
            let _ = monitor.check_tick_budget("arm_controller", Duration::from_micros(100));
            // ...and yet the deadline, measured from the scheduled release, was missed.
            monitor.record_deadline_miss("arm_controller");
        }

        assert_eq!(
            monitor.consecutive_misses("arm_controller"),
            10,
            "ten consecutive deadline misses must read as ten; a tick that stayed \
             inside its BUDGET says nothing about whether it met its DEADLINE"
        );
    }

    /// The other half of the contract: a tick that meets its deadline clears
    /// the run, for a node that has a deadline and no budget at all.
    ///
    /// `node_builder.rs` derives a deadline from a budget and never the
    /// reverse, so this node's `tick_budget` is `None` and the main loop's
    /// whole `if let Some(tick_budget)` block — the only place that used to
    /// clear the counter — is skipped. Before `record_deadline_met` existed
    /// there was no path that cleared it, and the count was a lifetime total:
    /// a node missing once an hour reached any ceiling eventually.
    #[test]
    fn a_tick_that_meets_its_deadline_clears_the_run_without_any_budget() {
        let monitor = SafetyMonitor::new(1000);
        // Deliberately no `set_tick_budget`.

        for _ in 0..10 {
            monitor.record_deadline_miss("wheel_controller");
            monitor.record_deadline_met("wheel_controller");
        }

        assert_eq!(
            monitor.consecutive_misses("wheel_controller"),
            0,
            "a node that misses and immediately recovers, ten times over, is \
             jitter — its consecutive run is zero, not ten"
        );
    }

    /// A tick that missed its deadline is not evidence of recovery.
    ///
    /// `record_successful_tick` means "did not panic", and both callers reach
    /// it on a tick already recorded as a deadline miss in the same pass. With
    /// nothing clearing `recovery_counter` on a miss, a node at `RateReduced`
    /// that missed EVERY deadline still accumulated `recovery_ticks` of
    /// "success" and was restored to the rate it had just demonstrated it
    /// could not hold.
    #[test]
    fn a_node_that_never_meets_a_deadline_is_never_declared_recovered() {
        let mut monitor = SafetyMonitor::new(10_000);
        // isolate_after is out of reach so the node stays at RateReduced and
        // the test observes recovery, not a stage transition.
        monitor.set_degradation_policy(DegradationPolicy {
            warn_after: 3,
            reduce_after: 5,
            isolate_after: 10_000,
            kill_after: 20_000,
            recovery_ticks: 10,
        });

        let miss_tick = |m: &SafetyMonitor| {
            m.record_deadline_miss("arm_controller");
            let consecutive = m.consecutive_misses("arm_controller");
            m.evaluate_degradation("arm_controller", consecutive, Some(100.0));
            // The RT executor reaches this on any tick whose closure returned
            // `Ok` — including the one just recorded as a miss.
            m.record_successful_tick("arm_controller")
        };

        for _ in 0..5 {
            miss_tick(&monitor);
        }
        assert_eq!(
            monitor.degradation_stage("arm_controller"),
            DegradationStage::RateReduced,
            "five consecutive misses must reduce the rate"
        );

        // Keep missing, for twice the recovery window.
        for i in 0..(10 * 2) {
            let action = miss_tick(&monitor);
            assert!(
                !matches!(action, DegradationAction::RestoreRate { .. }),
                "tick {} missed its deadline like every tick before it — \
                 restoring its full rate says it recovered when it never did",
                i
            );
        }

        assert_eq!(
            monitor.degradation_stage("arm_controller"),
            DegradationStage::RateReduced,
            "a node that has not met a deadline yet must still be degraded"
        );
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
        for row in &all_stats {
            assert_eq!(
                row.stats.total_ticks, 50,
                "{} should have 50 ticks",
                row.name
            );
            assert!(
                row.budget.is_some(),
                "{} should have a budget set",
                row.name
            );
            assert_eq!(row.overruns, 0, "{} should have 0 overruns", row.name);
            assert_eq!(
                row.deadline_misses, 0,
                "{} met every deadline, so its miss count must be 0 — not its \
                 tick count",
                row.name
            );
        }
    }

    /// Rapid watchdog feed/check cycles: 1000+ iterations in a tight loop.
    /// Verifies the AtomicU64 approach stays consistent under rapid mutation.
    #[test]
    fn test_rapid_watchdog_feed_check_1000_iterations() {
        let wd = Watchdog::new(1_u64.secs()); // long timeout so it never expires

        for _ in 0..2000 {
            wd.feed();
            assert!(
                !wd.check(),
                "watchdog must not expire with 1s timeout in a tight loop"
            );
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
            enforcer.set_budget(
                format!("node_{:02}", i),
                Duration::from_micros((i + 1) * 100),
            );
        }

        // Record 100 ticks for each node at exactly half budget (no overruns)
        for i in 0..20u64 {
            let tick = Duration::from_micros((i + 1) * 50);
            for _ in 0..100 {
                let result = enforcer.check_budget(&format!("node_{:02}", i), tick);
                result.unwrap();
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

        assert_eq!(monitor.degradation_stage(failing), DegradationStage::Killed);

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
        assert_eq!(action, DegradationAction::Deisolate("lidar".to_string()));
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
        assert_eq!(monitor.degradation_stage("lidar"), DegradationStage::Normal);
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
        assert!(wd.check(), "1ms watchdog should expire after 5ms sleep");
        assert!(wd.is_expired());
    }

    /// Watchdog with very long timeout (1 hour) — verify it does not expire prematurely.
    #[test]
    fn test_watchdog_very_long_timeout_no_premature_expiry() {
        let wd = Watchdog::new(3600_u64.secs()); // 1 hour

        wd.feed();
        // Check immediately — must not be expired
        assert!(!wd.check(), "1-hour watchdog should not expire immediately");
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
        assert!(
            result.is_err(),
            "1us tick against zero budget should violate"
        );
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

    /// The ceiling is per node and consecutive, so misses spread across
    /// DIFFERENT nodes never reach it.
    ///
    /// This test used to record one miss each on ten different nodes and expect
    /// an e-stop, which is the behaviour that made `max_deadline_misses` a
    /// lifetime process-wide total: ten unrelated nodes each hiccuping once
    /// halted the robot, and nothing ever reset the count, so a long enough run
    /// halted whatever its health. Now one node has to miss `max` times in a row
    /// -- it is not keeping up, right now -- which is what the setting reads as.
    #[test]
    fn test_deadline_miss_threshold_exact() {
        let _estop_serial = super::estop_queue_guard();
        let max_misses = 10u64;
        let monitor = SafetyMonitor::new(max_misses);

        // Misses scattered across many nodes must NOT reach the ceiling, however
        // many there are in total.
        for i in 0..(max_misses * 3) {
            monitor.record_deadline_miss(&format!("node_{}", i));
        }
        assert!(
            !monitor.is_emergency_stop(),
            "{} misses spread over {} different nodes must not e-stop: none of \
             them missed twice in a row",
            max_misses * 3,
            max_misses * 3
        );

        // But one node missing max_misses - 1 times consecutively still must not.
        for i in 0..(max_misses - 1) {
            monitor.record_deadline_miss("one_bad_node");
            assert!(
                !monitor.is_emergency_stop(),
                "should not estop at consecutive miss {} / {}",
                i + 1,
                max_misses
            );
        }

        // The Nth CONSECUTIVE miss on that same node triggers emergency stop.
        monitor.record_deadline_miss("one_bad_node");
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
            result.unwrap();
        }

        // But timing data should still be recorded
        let all = monitor.all_node_timing();
        let unbounded = all.iter().find(|r| r.name == "unbounded");
        assert!(unbounded.is_some());
        let row = unbounded.unwrap();
        assert_eq!(row.stats.total_ticks, 10);
        assert!(row.budget.is_none());
        assert_eq!(row.overruns, 0);
        assert_eq!(row.deadline_misses, 0);
    }

    /// Critical node budget violation triggers emergency stop via check_tick_budget.
    #[test]
    fn test_critical_node_budget_violation_estop() {
        let monitor = SafetyMonitor::new(100);
        monitor.add_critical_node("safety_ctrl".to_string(), 1_u64.secs());
        monitor.set_tick_budget("safety_ctrl".to_string(), 100_u64.us());

        // Within budget — no violation, no estop
        let _ = monitor.check_tick_budget("safety_ctrl", 50_u64.us());
        assert!(!monitor.is_emergency_stop());

        // INVERTED. This used to assert that one budget overrun by a node in
        // `critical_nodes` latched an emergency stop. That was the defect:
        // `critical_nodes` holds every node with a watchdog, and `.rate()`
        // auto-derives a budget at 80% of the period, so a single page fault
        // or IRQ stopped the robot — overriding the node's own
        // `BudgetPolicy::Warn` default ("log the violation but take no
        // corrective action") before the caller could consult it, and making
        // the graduated-degradation ladder unreachable. `check_tick_budget` is
        // pure accounting now: it reports the violation and leaves escalation
        // to the policy dispatch, the `max_deadline_misses` ceiling and the 3x
        // watchdog expiry.
        let result = monitor.check_tick_budget("safety_ctrl", 200_u64.us());
        assert!(
            result.is_err(),
            "the overrun is still reported to the caller"
        );
        assert!(
            !monitor.is_emergency_stop(),
            "a watchdogged node is not a must-never-miss node: one overrun \
             must not stop the robot"
        );
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
        let _estop_serial = super::estop_queue_guard();
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

    // ── Networked-e-stop SEND path: rising-edge gate + anti-storm ─────────────
    // PENDING_LOCAL_ESTOP is process-global — these tests drain it at the start and
    // MUST run with `--test-threads=1` (they share the global with each other and
    // with every other trigger_emergency_stop test in this crate).

    /// Rising-edge gate: only the false→true transition queues a broadcast.
    ///
    /// The take() is INTERLEAVED (drain after each trigger) so the SECOND take is the
    /// discriminator. With the gate, the second (non-rising) trigger queues nothing →
    /// None. Without it (naive unconditional set) the second trigger overwrites PENDING
    /// to Some("second") and the final assert fails. Proven RED→GREEN by forcing the
    /// set unconditional.
    #[test]
    fn test_rising_edge_queues_pending_only_on_edge() {
        let _estop_serial = super::estop_queue_guard();
        // Global slot — drain any residue left by other tests first.
        let _ = take_pending_local_estop();

        let monitor = SafetyMonitor::new(100);

        // First trigger is the rising edge (false→true) → queues the reason.
        monitor.trigger_emergency_stop("first".to_string());
        assert_eq!(
            take_pending_local_estop(),
            Some("first".to_string()),
            "first (rising-edge) trigger must queue the local e-stop reason"
        );

        // Second trigger is NOT a rising edge (already latched) → queues nothing.
        // This final None is the gate discriminator (naive set would yield Some).
        monitor.trigger_emergency_stop("second".to_string());
        assert_eq!(
            take_pending_local_estop(),
            None,
            "second (non-rising) trigger must NOT re-queue (anti-storm)"
        );
    }

    /// ANTI-STORM (the key test): a received remote e-stop latches this monitor, then
    /// THIS robot's own watchdog starves and triggers locally — that local trigger must
    /// NOT queue a broadcast, because the remote store consumed the rising edge.
    ///
    /// Without the swap-gate this returns Some → a delayed fleet-wide e-stop storm.
    /// The remote path is simulated by `store(true)` directly on the flag — exactly what
    /// `install_emergency_stop_hook`'s closure does when a remote packet arrives.
    /// Proven RED→GREEN by forcing the set unconditional.
    #[test]
    fn test_antistorm_remote_estop_not_rebroadcast() {
        let _estop_serial = super::estop_queue_guard();
        // Global slot — drain any residue left by other tests first.
        let _ = take_pending_local_estop();

        let monitor = SafetyMonitor::new(100);

        // Simulate a RECEIVED remote e-stop: the hook store()s the flag true directly.
        monitor.emergency_stop.store(true, Ordering::SeqCst);

        // Now this robot's nodes stop ticking → watchdog starves → LOCAL trigger fires.
        monitor.trigger_emergency_stop("watchdog expired".to_string());

        assert_eq!(
            take_pending_local_estop(),
            None,
            "a remote-latched e-stop must NOT be re-broadcast when the local watchdog \
             later fires — re-broadcasting is the e-stop storm bug"
        );
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

#[cfg(test)]
mod estop_trigger_tests {
    use super::*;

    /// An executor thread must be able to latch the e-stop exactly as the main
    /// thread does. Before `EstopTrigger` existed, the RT executor could only
    /// set a shutdown flag, so a real RT emergency stop left the monitor
    /// reporting Normal and queued nothing for the fleet broadcast.
    #[test]
    fn trigger_from_a_handle_latches_state_like_the_monitor() {
        let _estop_serial = super::estop_queue_guard();
        let monitor = SafetyMonitor::new(10);
        let trigger = monitor.estop_trigger();

        assert!(!monitor.is_emergency_stop());
        assert_eq!(monitor.get_state(), SafetyState::Normal);

        trigger.trigger("rt node blew its budget".to_string());

        assert!(
            monitor.is_emergency_stop(),
            "the handle must latch the SAME AtomicBool the monitor reads"
        );
        assert_eq!(monitor.get_state(), SafetyState::EmergencyStop);
    }

    /// The fleet-broadcast queue is what horus_net drains. A local-origin e-stop
    /// raised from an executor thread must populate it, or peer robots are never
    /// told this one stopped.
    #[test]
    fn trigger_queues_the_fleet_broadcast_on_the_rising_edge() {
        let _guard = estop_queue_guard();
        let _ = take_pending_local_estop(); // clear any prior test's edge
        let monitor = SafetyMonitor::new(10);
        let trigger = monitor.estop_trigger();

        trigger.trigger("deadline miss escalated".to_string());
        let pending = take_pending_local_estop();
        assert!(
            pending.is_some_and(|r| r.contains("deadline miss escalated")),
            "a rising-edge e-stop from an executor must queue the broadcast"
        );
    }

    /// Anti-storm invariant: only the transition into e-stop announces. A second
    /// trigger while already latched must not re-queue, or a fleet already
    /// halting gets a broadcast per tick.
    #[test]
    fn a_second_trigger_does_not_requeue_a_broadcast() {
        let _guard = estop_queue_guard();
        let _ = take_pending_local_estop();
        let monitor = SafetyMonitor::new(10);
        let trigger = monitor.estop_trigger();

        trigger.trigger("first".to_string());
        let _ = take_pending_local_estop();

        trigger.trigger("second".to_string());
        assert!(
            take_pending_local_estop().is_none(),
            "already latched — not a rising edge, so no second broadcast"
        );
        assert!(monitor.is_emergency_stop(), "the latch still holds");
    }
}
