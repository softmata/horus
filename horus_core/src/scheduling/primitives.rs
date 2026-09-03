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
            let panicked = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
                target.enter_safe_state()
            }))
            .is_err();
            if panicked {
                node.is_stopped = true;
            }
            super::rt_executor::rt_diag(format_args!(
                " Degradation: '{name}' — isolated, entered safe state{}",
                if panicked {
                    " FAILED (panicked) — node stopped"
                } else {
                    ""
                }
            ));
            if let Some(ref monitor) = monitors.safety {
                monitor.record_degrade_activation();
            }
        }
        DegradationAction::Kill(ref name) => {
            set_health(node, NodeHealthState::Isolated);
            let target = &mut node.node;
            let panicked = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
                let _ = target.shutdown();
            }))
            .is_err();
            node.is_stopped = true;
            super::rt_executor::rt_diag(format_args!(
                " KILL: '{name}' — permanently removed from execution after shutdown(){}",
                if panicked { " (shutdown panicked)" } else { "" }
            ));
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
    let sim_time = clock.elapsed();
    let tick_start_ci = clock.now();
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
