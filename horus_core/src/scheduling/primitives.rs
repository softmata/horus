//! Low-level execution primitives for the scheduler.
//!
//! These are the building blocks that the scheduler composes for different
//! execution strategies (sequential, parallel, future RT-thread, etc.).
//!
//! # Everything above `run_tick` runs on an RT thread
//!
//! `honor_safe_state_request` and `apply_degradation_action` are reached only
//! from `rt_executor` -- the scheduler keeps its own main-thread copy of the
//! degradation dispatch -- so they execute on a SCHED_FIFO thread, inside the
//! tick. They therefore report through `rt_diag`, which formats into a
//! statically allocated ring and never allocates, blocks or enters the kernel,
//! rather than through `print_line`.
//!
//! `print_line` does `isatty` + `tcgetattr`, takes the process-global stdout
//! lock, then `write` and `flush`. Pointed at a slow consumer -- a serial
//! console, a pipe nobody is reading -- that write blocks for as long as the
//! reader takes, on the thread driving an actuator. Two of these sites are the
//! `Isolate` and `Kill` rungs, which are not even verbose-gated and which safe
//! or shut down the node in the same breath: the worst possible place to wait
//! on a terminal.
//!
//! The drain half is started by `RtExecutor::start` on the caller's thread
//! before any RT thread exists, so a line queued from here always has a drainer.

use std::time::{Duration, Instant};

use super::types::{NodeKind, RegisteredNode};
use crate::core::rt_node::BudgetViolation;
use crate::core::Miss;

/// Result of a single node tick execution.
///
/// Contains the raw timing and panic information. Higher-level concerns
/// (budget checks, failure policies) are handled by the scheduler.
pub(crate) struct TickResult {
    /// The instant when the tick started (needed for deadline checks).
    pub tick_start: Instant,
    /// Wall-clock duration of the tick.
    pub duration: Duration,
    /// The raw result from `catch_unwind` — `Ok(())` on success, `Err(Box<dyn Any>)` on panic.
    pub result: std::thread::Result<()>,
}

/// Run a fault-path node callback under `catch_unwind`. Returns `true` if the
/// callback panicked.
///
/// The executor-side home of the guard the main-thread scheduler applies via
/// `Scheduler::guard_fault_callback`, which delegates here so both halves of
/// the framework isolate recovery callbacks identically. A node's recovery
/// callbacks (`on_error`/`enter_safe_state`/`shutdown`) are invoked exactly
/// when that node is already failing, and an unwind out of one of them is NOT
/// contained the way a `tick()` panic is: on the RT path it escapes past
/// `apply_failure_policy_after_panic`, so a `Fatal` node neither safes nor
/// stops the scheduler; on the compute/async/event paths nothing catches it at
/// all and the executor thread dies, taking every healthy node it owns with it.
///
/// Any future `&mut dyn Node` callback reached from an executor belongs behind
/// this guard.
#[inline]
pub(crate) fn guard_fault_callback(f: impl FnOnce()) -> bool {
    std::panic::catch_unwind(std::panic::AssertUnwindSafe(f)).is_err()
}

/// A node's safing callback panicked on an executor, so the node did NOT reach
/// a safe state. Stop ticking it, and — because a node that cannot safe itself
/// is a safety failure — escalate to a system emergency stop for critical
/// nodes.
///
/// The executor twin of `Scheduler::note_safing_failure`, which states the
/// intent as a property of the system rather than of one dispatch path. The
/// isolation half of that was ported to the executors (see
/// [`guard_fault_callback`]); the ESCALATION half was not, so the identical
/// event halted the robot on a BestEffort node and merely stopped an RT one —
/// and every RT node under a scheduler `.watchdog()` is a critical node, so the
/// class most likely to be critical was the one the escalation omitted.
pub(crate) fn note_safing_failure(
    node: &mut RegisteredNode,
    monitors: &super::types::SharedMonitors,
    action: &str,
) {
    node.is_stopped = true;
    let critical = monitors
        .safety
        .as_ref()
        .is_some_and(|m| m.is_critical_node(&node.name));
    if critical {
        super::rt_executor::rt_diag(format_args!(
            "[RT-thread] '{}' PANICKED in {} and could NOT reach a safe state — EMERGENCY STOP",
            node.name, action
        ));
        if let Some(ref estop) = monitors.estop {
            estop.trigger(format!(
                "critical node '{}' panicked in {} and could not reach a safe state",
                node.name, action
            ));
        }
    } else {
        super::rt_executor::rt_diag(format_args!(
            "[RT-thread] '{}' panicked in {} and could NOT reach a safe state — node stopped",
            node.name, action
        ));
    }
}

/// Honour a pending restart request raised by `horus node restart`, if this
/// node has one.
///
/// Same shape and same reason as [`honor_safe_state_request`]: `init()` needs
/// `&mut dyn Node`, which the executor owns, so the main thread raises a flag
/// and the owning executor consumes it here, once per raise.
///
/// Every executor must call this at the top of its per-node pass. Skipping it
/// in one executor means `horus node restart` silently does nothing for that
/// whole class of node — which is what the control-command handler alone did
/// for all four executor classes, because after the class partition the
/// scheduler's `nodes` vector holds only the main-thread group.
///
/// A restart also lifts an operator pause: a wedged node is usually paused
/// before it is restarted, and leaving it paused would make the restart a
/// no-op at the very next tick gate.
pub(crate) fn honor_restart_request(
    node: &mut RegisteredNode,
    monitors: &super::types::SharedMonitors,
) {
    if !monitors
        .node_controls
        .take_restart_request(node.name.as_ref())
    {
        return;
    }

    monitors.node_controls.set_paused(node.name.as_ref(), false);
    node.is_paused = false;

    let target = &mut node.node;
    let panicked = guard_fault_callback(|| {
        let _ = target.init();
    });

    if panicked {
        // A node that cannot initialise must not be ticked.
        node.is_stopped = true;
        super::rt_executor::rt_diag(format_args!(
            " Restart: '{}' PANICKED in init() — node stopped",
            node.name
        ));
    } else {
        node.initialized = true;
        super::rt_executor::rt_diag(format_args!(
            " Restart: '{}' re-initialised on its executor",
            node.name
        ));
    }
}

/// Honour a pending safe-state request raised by the main thread's watchdog
/// ladder, if this node has one.
///
/// The ladder runs on the main thread — the only one a hung node cannot block
/// — but `enter_safe_state()` needs `&mut dyn Node`, which the executor owns.
/// So Critical raises a flag in the shared control map and the owning executor
/// consumes it here, once per raise.
///
/// Every executor must call this at the top of its per-node pass. Skipping it
/// in one executor means nodes of that class are marked Isolated and never
/// actually safed.
pub(crate) fn honor_safe_state_request(
    node: &mut RegisteredNode,
    monitors: &super::types::SharedMonitors,
) {
    if !monitors
        .node_controls
        .take_safe_state_request(node.name.as_ref())
    {
        return;
    }

    let target = &mut node.node;
    let panicked =
        std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| target.enter_safe_state()))
            .is_err();

    if panicked {
        // It did not reach a safe state. Stop it; the e-stop was already
        // latched by the ladder when it raised the request.
        node.is_stopped = true;
        if monitors.verbose {
            super::rt_executor::rt_diag(format_args!(
                " Watchdog critical: '{}' PANICKED in enter_safe_state on its executor",
                node.name
            ));
        }
    } else if monitors.verbose {
        super::rt_executor::rt_diag(format_args!(
            " Watchdog critical: '{}' entered safe state on its executor",
            node.name
        ));
    }
}

/// Apply a `DegradationAction` to a node an executor owns.
///
/// Counterpart to `Scheduler::apply_degradation_action`, which reaches its
/// node through `self.nodes[i]` and therefore only ever covers BestEffort
/// nodes. The RT executor computed the action and merely PRINTED it, so
/// `ReduceRate`, `Isolate` and `Kill` were produced and discarded — the
/// documented graceful-degradation ladder did nothing for the RT nodes it
/// exists to protect.
///
/// Health is written to the node AND mirrored into the shared control map so
/// the main thread, the registry and `horus node status` see one consistent
/// value across all five execution groups.
pub(crate) fn apply_degradation_action(
    node: &mut RegisteredNode,
    action: super::safety_monitor::DegradationAction,
    monitors: &super::types::SharedMonitors,
) {
    use super::safety_monitor::DegradationAction;
    use super::types::NodeHealthState;

    let set_health = |node: &mut RegisteredNode, state: NodeHealthState| {
        node.health_state.store(state);
        monitors.node_controls.set_health(node.name.as_ref(), state);
    };

    match action {
        DegradationAction::None => {}
        DegradationAction::Warn(ref name) => {
            if monitors.verbose {
                super::rt_executor::rt_diag(format_args!(
                    " Degradation: '{name}' — sustained timing violations, monitoring"
                ));
            }
        }
        DegradationAction::ReduceRate {
            node: ref name,
            new_rate_hz,
        } => {
            // Widen the watchdog by the same factor the rate shrank by.
            // The tick is what feeds the watchdog, so halving the rate halves
            // the feed frequency; leaving the timeout fixed would turn the
            // GENTLEST rung of the ladder into an escalation for any node
            // whose watchdog margin was under 2x its period — 1x, 2x, then
            // Critical, which latches a fleet-wide e-stop. Rate-reducing a
            // struggling node must not be a slower route to halting the robot.
            if let (Some(monitor), Some(old_rate)) = (monitors.safety.as_ref(), node.rate_hz) {
                if new_rate_hz > 0.0 && old_rate > new_rate_hz {
                    monitor.scale_watchdog(node.name.as_ref(), old_rate / new_rate_hz);
                }
            }
            node.rate_hz = Some(new_rate_hz);
            node.last_tick = Some(Instant::now());
            if monitors.verbose {
                super::rt_executor::rt_diag(format_args!(
                    " Degradation: '{name}' — reducing rate to {new_rate_hz:.1} Hz"
                ));
            }
        }
        DegradationAction::Isolate(ref name) => {
            set_health(node, NodeHealthState::Isolated);
            let target = &mut node.node;
            let panicked = guard_fault_callback(|| target.enter_safe_state());
            super::rt_executor::rt_diag(format_args!(
                " Degradation: '{name}' — isolated, entered safe state{}",
                if panicked {
                    " FAILED (panicked) — node stopped"
                } else {
                    ""
                }
            ));
            if panicked {
                note_safing_failure(node, monitors, "enter_safe_state");
            }
            if let Some(ref monitor) = monitors.safety {
                monitor.record_degrade_activation();
            }
        }
        DegradationAction::Kill(ref name) => {
            set_health(node, NodeHealthState::Isolated);
            let target = &mut node.node;
            let panicked = guard_fault_callback(|| {
                let _ = target.shutdown();
            });
            node.is_stopped = true;
            super::rt_executor::rt_diag(format_args!(
                " KILL: '{name}' — permanently removed from execution after shutdown(){}",
                if panicked { " (shutdown panicked)" } else { "" }
            ));
            if panicked {
                note_safing_failure(node, monitors, "shutdown");
            }
            if let Some(ref monitor) = monitors.safety {
                monitor.record_degrade_activation();
            }
        }
        DegradationAction::RestoreRate {
            node: ref name,
            original_rate_hz,
        } => {
            // Back to the configured window now the node ticks at full rate.
            if let Some(monitor) = monitors.safety.as_ref() {
                monitor.scale_watchdog(node.name.as_ref(), 1.0);
            }
            node.rate_hz = Some(original_rate_hz);
            node.last_tick = Some(Instant::now());
            set_health(node, NodeHealthState::Healthy);
            if monitors.verbose {
                super::rt_executor::rt_diag(format_args!(
                    " Recovery: '{name}' — restored to {original_rate_hz:.1} Hz"
                ));
            }
        }
        DegradationAction::Deisolate(ref name) => {
            set_health(node, NodeHealthState::Warning);
            if monitors.verbose {
                super::rt_executor::rt_diag(format_args!(
                    " Recovery: '{name}' — de-isolated, resuming at reduced rate"
                ));
            }
        }
    }
}

/// Executes a single node tick with timing measurement and panic isolation.
///
/// This is the minimal execution primitive: measure time, call `tick()`, catch panics.
/// It does NOT handle rate limiting, failure policies, watchdog feeding,
/// or recording — those are scheduler-level concerns.
pub(crate) struct NodeRunner;

impl NodeRunner {
    /// Run a single tick on the given node.
    ///
    /// Measures wall-clock duration and catches any panics from the node's `tick()` method.
    /// The caller is responsible for all pre-tick checks and post-tick processing.
    #[inline]
    pub fn run_tick(node: &mut NodeKind) -> TickResult {
        let tick_start = Instant::now();
        let result = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
            node.tick();
        }));
        let duration = tick_start.elapsed();
        TickResult {
            tick_start,
            duration,
            result,
        }
    }
}

/// Install the per-tick thread-local ambient context for `node`, mirroring the
/// main loop's `Scheduler::run_node_tick` (scheduler/mod.rs:3636-3665).
///
/// The tick context (`horus::now()`/`dt()`/`elapsed()`/`rng()`/
/// `budget_remaining()`) and node context (`hlog!`) are **thread-locals**, so
/// this MUST be called on the SAME thread that will run `NodeRunner::run_tick`
/// — for the compute (crossbeam) and async (`spawn_blocking`) executors that is
/// the child worker thread, not the pool thread. Without this, executor-run
/// nodes see the inert fallbacks (`dt()`→0, `budget_remaining()`→`MAX`,
/// `rng()`→unseeded), because only the main loop was calling it (FIX #5).
///
/// Pair every call with `clear_node_tick_context()` on the same thread after
/// the tick returns.
pub(crate) fn set_node_tick_context(
    node: &RegisteredNode,
    clock: &dyn crate::core::clock::Clock,
    tick_period: Duration,
) {
    // Read the tick number the same way run_node_tick does: after start_tick()
    // (already called by the executor) and before record_tick().
    let tick_number = node
        .context
        .as_ref()
        .map(|c| c.metrics().total_ticks())
        .unwrap_or(0);
    crate::core::hlog::set_node_context(&node.name, tick_number);

    // dt = 1/rate for rate-driven nodes, else the scheduler's tick period
    // (identical to run_node_tick's `self.tick.period` fallback).
    let node_dt = node
        .rate_hz
        .map(|hz| Duration::from_secs_f64(1.0 / hz))
        .unwrap_or(tick_period);
    // One clock reading, not two. All three `Clock` impls define `now()` and
    // `elapsed()` as the same quantity (`WallClock` has both call
    // `epoch.elapsed()`; `SimClock` has both load `self.nanos`; `ReplayClock`
    // defines `elapsed()` via `now()`), so deriving one from the other is an
    // exact substitution rather than an approximation.
    //
    // Worth ~30ns per node per tick on WallClock and nothing on SimClock —
    // 0.27% of the measured 11.2us p50 jitter, so this will not move any
    // latency figure and is not offered as an RT fix. What it does remove is a
    // within-tick disagreement between `horus::now()` and `horus::elapsed()`.
    //
    // The `as u64` narrowing is exact, not a gamble. `ClockInstant` *is* a
    // `u64` nanosecond count, so every constructor narrows; this is the same
    // cast `WallClock::now()` performs internally on the line this replaces.
    // For `SimClock` and `ReplayClock` the value provably fits: both build
    // `elapsed()` out of `Duration::from_nanos(u64)`, so the round trip is
    // lossless. For `WallClock` the source is `Instant::elapsed()` since
    // process start, which needs 584 years of uptime to exceed `u64::MAX` ns.
    let sim_time = clock.elapsed();
    let tick_start_ci = crate::core::clock::ClockInstant::from_nanos(sim_time.as_nanos() as u64);
    crate::core::tick_context::set_tick_context(
        &node.name,
        tick_number,
        clock,
        node_dt,
        sim_time,
        tick_start_ci,
        node.tick_budget,
    );
}

/// Clear the per-tick thread-local context installed by `set_node_tick_context`.
///
/// Must run on the SAME thread as its paired `set_node_tick_context`, after
/// `NodeRunner::run_tick` returns (mirrors run_node_tick's
/// `clear_tick_context()` + `clear_node_context()`).
pub(crate) fn clear_node_tick_context() {
    crate::core::tick_context::clear_tick_context();
    crate::core::hlog::clear_node_context();
}

/// Maximum time an executor thread gets to exit during shutdown before it is
/// detached.
///
/// `RtExecutor::stop` documents the guarantee this constant backs: shutdown
/// always completes within `SHUTDOWN_TIMEOUT_PER_THREAD x num_threads`, so a
/// single stalled node cannot block the process from exiting.
pub(crate) const SHUTDOWN_TIMEOUT_PER_THREAD: Duration = Duration::from_secs(3);

/// Join `handle` with a bounded deadline, returning `None` if it does not
/// finish in time (the handle is dropped and the thread detached) or panicked.
///
/// Every executor must use this rather than a bare `handle.join()`. The
/// compute, event and async executors used to join unbounded, and their loops
/// only re-check `running` BETWEEN ticks — so a node blocked inside `tick()`
/// (an unplugged device read with no timeout, a deadlocked mutex, an infinite
/// loop) hung the whole shutdown. Because `run_with_filter` calls the executor
/// stops before `shutdown_filtered_nodes` and `finalize_run`, that also meant
/// no OTHER node was ever shut down or safed, and the blackbox was never
/// flushed: the process had to be SIGKILLed with actuators left at their last
/// commanded value.
pub(crate) fn join_with_timeout<T>(
    handle: std::thread::JoinHandle<T>,
    label: &str,
    timeout: Duration,
) -> Option<T> {
    let start = Instant::now();
    loop {
        if handle.is_finished() {
            return match handle.join() {
                Ok(v) => Some(v),
                Err(_) => {
                    crate::terminal::print_line(&format!("[{label}] thread panicked during stop"));
                    None
                }
            };
        }
        if start.elapsed() > timeout {
            crate::terminal::print_line(&format!(
                "[{label}] thread did not exit within {timeout:?} — detaching \
                 (possible stalled tick); its nodes are not reclaimed. The \
                 thread dies when the process exits."
            ));
            // Drop the JoinHandle without joining: the thread keeps running but
            // no longer holds shutdown hostage.
            drop(handle);
            return None;
        }
        std::thread::sleep(Duration::from_millis(10));
    }
}

/// Action to take after a budget violation is detected.
#[derive(Debug)]
pub(crate) struct BudgetViolationResult {
    pub violation: BudgetViolation,
}

/// Action to take after a deadline miss is detected.
#[derive(Debug)]
pub(crate) enum DeadlineAction {
    /// Log warning, no further action.
    Warn,
    /// Pause the node for one tick.
    Skip,
    /// Call `enter_safe_state()` on the node, continue ticking in safe mode.
    SafeMode,
    /// Trigger emergency stop — caller must stop the scheduler.
    EmergencyStop,
}

/// Result of a deadline check when a miss is detected.
#[derive(Debug)]
pub(crate) struct DeadlineMissResult {
    pub elapsed: Duration,
    pub deadline: Duration,
    pub action: DeadlineAction,
}

/// Stateless timing enforcer for budget and deadline checks.
///
/// Extracts the timing violation logic from both the scheduler and RT executor
/// into a single reusable struct. Callers provide the raw tick data and node
/// configuration; the enforcer returns structured results describing what
/// happened and what action to take.
pub(crate) struct TimingEnforcer;

impl TimingEnforcer {
    /// Check whether a tick exceeded its budget.
    ///
    /// Returns `Some(BudgetViolationResult)` if the tick duration exceeds the budget,
    /// `None` otherwise.
    #[inline]
    pub fn check_tick_budget(
        node_name: &str,
        tick_duration: Duration,
        tick_budget: Duration,
    ) -> Option<BudgetViolationResult> {
        if tick_duration > tick_budget {
            Some(BudgetViolationResult {
                violation: BudgetViolation::new(node_name.to_string(), tick_budget, tick_duration),
            })
        } else {
            None
        }
    }

    /// Check whether a tick missed its deadline.
    ///
    /// `tick_start` is the `Instant` when the tick began; the elapsed time since
    /// then is compared against `deadline`. The `miss` policy determines the action.
    ///
    /// Returns `Some(DeadlineMissResult)` if the deadline was missed, `None` otherwise.
    #[inline]
    pub fn check_deadline(
        tick_start: Instant,
        deadline: Duration,
        miss: Miss,
    ) -> Option<DeadlineMissResult> {
        Self::check_deadline_from_release(tick_start, Duration::ZERO, deadline, miss)
    }

    /// Deadline measured from the node's SCHEDULED RELEASE, not from the moment
    /// its `tick()` happened to start.
    ///
    /// `check_deadline` measures `tick_start.elapsed()` — how long the node's
    /// own code ran. So does `check_tick_budget`, against `tr.duration`. Two
    /// knobs measuring the same quantity is the defect class this codebase
    /// names elsewhere: "two named options that do the same thing means one of
    /// them is a lie". The lie here is that `deadline` sounded like a deadline.
    ///
    /// What a control loop actually promises is that the command LEAVES by a
    /// certain time after the period began. A node released 3.5 ms late that
    /// then executes in 10 us has blown a 900 us deadline by 2.6 ms — the
    /// actuator got its command three cycles late — while the old check saw
    /// 10 us against 900 us and recorded a healthy tick. The deadline-miss
    /// ladder, and every degradation and safe-state response hanging off it,
    /// therefore never fired for the dominant real-world failure mode. It fired
    /// only for the rarer one where the node's own code is slow, which
    /// `budget` already covers.
    ///
    /// `release_late` is how late the tick was released relative to its
    /// scheduled slot; the RT executor's cyclic waiter already computes it.
    /// Passing `Duration::ZERO` reproduces the old execution-time-only
    /// behaviour exactly, which is what the non-RT paths do — they have no
    /// scheduled slot to be late against.
    pub fn check_deadline_from_release(
        tick_start: Instant,
        release_late: Duration,
        deadline: Duration,
        miss: Miss,
    ) -> Option<DeadlineMissResult> {
        let elapsed = tick_start.elapsed() + release_late;
        if elapsed > deadline {
            let action = match miss {
                Miss::Warn => DeadlineAction::Warn,
                Miss::Skip => DeadlineAction::Skip,
                Miss::SafeMode => DeadlineAction::SafeMode,
                Miss::Stop => DeadlineAction::EmergencyStop,
            };
            Some(DeadlineMissResult {
                elapsed,
                deadline,
                action,
            })
        } else {
            None
        }
    }
}

/// Consecutive failed ticks before a node is marked `Unhealthy`.
///
/// A single panic can be transient — a bad message, a one-off division. A
/// sustained run of them is a different condition, and one that every health
/// surface (`horus node info`, `node list`, the registry) previously missed
/// entirely because health was driven only by the timing ladder.
pub(crate) const FAILURES_BEFORE_UNHEALTHY: u32 = 3;

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::duration_ext::DurationExt;
    use crate::core::Node;

    struct OkNode;
    impl Node for OkNode {
        fn name(&self) -> &str {
            "ok_node"
        }
        fn tick(&mut self) {}
    }

    struct PanicNode;
    impl Node for PanicNode {
        fn name(&self) -> &str {
            "panic_node"
        }
        fn tick(&mut self) {
            panic!("intentional panic");
        }
    }

    struct SlowNode {
        work_us: u64,
    }
    impl Node for SlowNode {
        fn name(&self) -> &str {
            "slow_node"
        }
        fn tick(&mut self) {
            std::thread::sleep(self.work_us.us());
        }
    }

    #[test]
    fn test_run_tick_success() {
        let mut node = NodeKind::new(Box::new(OkNode));
        let result = NodeRunner::run_tick(&mut node);
        result.result.unwrap();
    }

    #[test]
    fn test_run_tick_catches_panic() {
        let mut node = NodeKind::new(Box::new(PanicNode));
        let result = NodeRunner::run_tick(&mut node);
        assert!(result.result.is_err());
    }

    #[test]
    fn test_run_tick_measures_duration() {
        let mut node = NodeKind::new(Box::new(SlowNode { work_us: 1000 }));
        let result = NodeRunner::run_tick(&mut node);
        result.result.unwrap();
        // Should be at least ~1ms (allowing some scheduling jitter)
        assert!(result.duration.as_micros() >= 500);
    }

    // ====================================================================
    // TimingEnforcer tests
    // ====================================================================

    #[test]
    fn test_check_budget_no_violation() {
        let result = TimingEnforcer::check_tick_budget("node", 100_u64.us(), 500_u64.us());
        assert!(result.is_none());
    }

    #[test]
    fn test_check_budget_violation() {
        let result = TimingEnforcer::check_tick_budget("motor_ctrl", 800_u64.us(), 500_u64.us());
        let v = result.expect("should detect violation");
        assert_eq!(v.violation.node_name(), "motor_ctrl");
        assert_eq!(v.violation.budget(), 500_u64.us());
        assert_eq!(v.violation.actual(), 800_u64.us());
        assert_eq!(v.violation.overrun(), 300_u64.us());
    }

    #[test]
    fn test_check_budget_exact_no_violation() {
        let result = TimingEnforcer::check_tick_budget("node", 500_u64.us(), 500_u64.us());
        assert!(result.is_none(), "exact budget should not be a violation");
    }

    #[test]
    fn test_check_deadline_no_miss() {
        // tick_start is now — elapsed is ~0, well within any deadline
        let tick_start = Instant::now();
        let result = TimingEnforcer::check_deadline(tick_start, 100_u64.ms(), Miss::Warn);
        assert!(result.is_none());
    }

    #[test]
    fn test_check_deadline_miss() {
        // tick_start was 50ms ago — deadline is 10ms
        let tick_start = Instant::now() - 50_u64.ms();
        let result = TimingEnforcer::check_deadline(tick_start, 10_u64.ms(), Miss::Stop);
        let dm = result.expect("should detect deadline miss");
        assert!(dm.elapsed >= 50_u64.ms());
        assert_eq!(dm.deadline, 10_u64.ms());
        assert!(matches!(dm.action, DeadlineAction::EmergencyStop));
    }

    /// A tick released late misses its deadline even when its own code is fast.
    ///
    /// This is the case the old check could not see. On a 1 kHz loop with a
    /// 900us deadline, a node woken 3.5ms late — the max lateness actually
    /// measured on a stock kernel — and executing in microseconds has delivered
    /// its command three cycles after it was due. Judged on execution time
    /// alone it looked perfectly healthy, so the deadline-miss ladder, the
    /// degradation ladder and the safe-state response all stayed silent.
    #[test]
    fn release_lateness_alone_misses_the_deadline() {
        let tick_start = Instant::now(); // execution ~0
        let result = TimingEnforcer::check_deadline_from_release(
            tick_start,
            3_500_u64.us(),
            900_u64.us(),
            Miss::SafeMode,
        );
        let dm = result.expect(
            "a tick released 3.5ms late against a 900us deadline is a miss, \
             however fast its own code ran",
        );
        assert!(dm.elapsed >= 3_500_u64.us());
        assert_eq!(dm.deadline, 900_u64.us());
        assert!(matches!(dm.action, DeadlineAction::SafeMode));

        // And the old entry point, which cannot see the lateness, still says
        // healthy — this is precisely the gap, pinned so it cannot come back.
        assert!(
            TimingEnforcer::check_deadline(tick_start, 900_u64.us(), Miss::SafeMode).is_none(),
            "execution-time-only check should see nothing wrong; if this starts \
             failing the two entry points have converged and one is redundant"
        );
    }

    /// Zero lateness must reproduce the old behaviour exactly, since every
    /// non-RT caller passes zero — they have no scheduled slot to be late against.
    #[test]
    fn zero_release_lateness_matches_the_execution_only_check() {
        for (start_ago_ms, deadline_ms) in [(0_u64, 100_u64), (50, 10), (9, 10), (11, 10)] {
            let tick_start = Instant::now() - start_ago_ms.ms();
            let a = TimingEnforcer::check_deadline_from_release(
                tick_start,
                Duration::ZERO,
                deadline_ms.ms(),
                Miss::Warn,
            );
            let b = TimingEnforcer::check_deadline(tick_start, deadline_ms.ms(), Miss::Warn);
            assert_eq!(
                a.is_some(),
                b.is_some(),
                "zero lateness diverged from the old check at \
                 {start_ago_ms}ms/{deadline_ms}ms"
            );
        }
    }

    /// Execution time and release lateness add: neither alone would miss here,
    /// together they do. A budget check on execution alone cannot express this.
    #[test]
    fn release_lateness_and_execution_time_combine() {
        let tick_start = Instant::now() - 600_u64.us();
        let result = TimingEnforcer::check_deadline_from_release(
            tick_start,
            600_u64.us(),
            900_u64.us(),
            Miss::Warn,
        );
        assert!(
            result.is_some(),
            "600us late plus 600us of execution exceeds a 900us deadline"
        );
        assert!(
            TimingEnforcer::check_deadline(tick_start, 900_u64.us(), Miss::Warn).is_none(),
            "600us of execution alone is inside a 900us deadline"
        );
    }

    #[test]
    fn test_check_deadline_miss_policy_mapping() {
        let tick_start = Instant::now() - 50_u64.ms();
        let deadline = 10_u64.ms();

        let policies_and_expected = [
            (Miss::Warn, "Warn"),
            (Miss::Skip, "Skip"),
            (Miss::SafeMode, "SafeMode"),
            (Miss::Stop, "EmergencyStop"),
        ];

        for (miss, expected_name) in policies_and_expected {
            let result = TimingEnforcer::check_deadline(tick_start, deadline, miss);
            let dm = result.unwrap_or_else(|| panic!("should detect miss for {}", expected_name));
            let action_name = format!("{:?}", dm.action);
            assert_eq!(action_name, expected_name);
        }
    }
}
