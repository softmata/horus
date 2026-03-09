//! Dedicated RT thread executor.
//!
//! Runs all RT nodes on an isolated OS thread with its own tick loop.
//! The RT thread is never blocked by compute or event nodes — it ticks
//! RT nodes sequentially in priority order at their declared rates.
//!
//! # Architecture
//!
//! ```text
//!  Main Thread (compute/event)    RT Thread (isolated)
//!  ┌──────────────────────┐       ┌──────────────────────┐
//!  │  non-RT tick loop    │       │  RT tick loop         │
//!  │  ┌────────────────┐  │       │  ┌────────────────┐   │
//!  │  │ compute nodes  │  │       │  │ RT node 0 tick │   │
//!  │  │ (parallel)     │  │       │  │ RT node 1 tick │   │
//!  │  └────────────────┘  │       │  │ RT node 2 tick │   │
//!  │                      │       │  └────────────────┘   │
//!  │  shared: running     │◄──────┤  uses: NodeRunner     │
//!  │  (AtomicBool)        │       │  SCHED_FIFO if avail  │
//!  └──────────────────────┘       └──────────────────────┘
//! ```

use std::panic::{catch_unwind, AssertUnwindSafe};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant};

use crate::terminal::print_line;

use super::primitives::{DeadlineAction, NodeRunner, TimingEnforcer};
use super::types::{RegisteredNode, SharedMonitors};

/// Dedicated RT thread executor.
///
/// Owns RT nodes and ticks them on an isolated OS thread at the rate of
/// the fastest node. Shutdown is coordinated via the shared `running` flag.
pub(crate) struct RtExecutor {
    handle: Option<std::thread::JoinHandle<Vec<RegisteredNode>>>,
}

impl RtExecutor {
    /// Start the RT executor with the given nodes on a dedicated thread.
    ///
    /// `running` is the shared scheduler running flag — when set to `false`,
    /// the RT thread finishes its current tick cycle and returns its nodes.
    ///
    /// The tick period is derived from the fastest RT node's rate. If no node
    /// has a declared rate, falls back to `fallback_period`.
    pub fn start(
        mut nodes: Vec<RegisteredNode>,
        running: Arc<AtomicBool>,
        fallback_period: Duration,
        monitors: SharedMonitors,
        rt_cpus: Vec<usize>,
    ) -> Self {
        // Sort by priority before handing off to the thread
        nodes.sort_by_key(|n| n.priority);

        // Determine tick period from the fastest RT node rate
        let max_rate_hz = nodes
            .iter()
            .filter_map(|n| n.rate_hz)
            .fold(0.0_f64, f64::max);

        let tick_period = if max_rate_hz > 0.0 {
            Duration::from_secs_f64(1.0 / max_rate_hz)
        } else {
            fallback_period
        };

        let handle = std::thread::Builder::new()
            .name("horus-rt".to_string())
            .spawn(move || Self::rt_thread_main(nodes, running, tick_period, monitors, rt_cpus))
            .expect("Failed to spawn RT thread");

        Self {
            handle: Some(handle),
        }
    }

    /// Stop the RT executor and reclaim its nodes.
    ///
    /// The caller should have already set `running` to `false` before calling this.
    /// This method blocks until the RT thread finishes and returns the nodes.
    pub fn stop(mut self) -> Vec<RegisteredNode> {
        self.handle
            .take()
            .expect("RT thread handle already consumed")
            .join()
            .expect("RT thread panicked")
    }

    /// Process a single node tick with all infrastructure (stats, profiler, WCET, deadline, failure handling).
    ///
    /// Extracted from the RT loop body so the caller can wrap this in `catch_unwind`.
    /// If this function panics, the caller marks the node as stopped.
    fn tick_node(node: &mut RegisteredNode, monitors: &SharedMonitors, running: &Arc<AtomicBool>) {
        // Update last tick time
        if node.rate_hz.is_some() {
            node.last_tick = Some(Instant::now());
        }

        // RT pre-conditions (catch_unwind: user code may panic)
        {
            let pre_ok = catch_unwind(AssertUnwindSafe(|| node.node.pre_condition()));
            match pre_ok {
                Ok(true) => {} // proceed to tick
                Ok(false) => {
                    if monitors.verbose {
                        print_line(&format!(
                            "[RT-thread] Pre-condition failed for '{}' — skipping",
                            node.name
                        ));
                    }
                    return;
                }
                Err(_) => {
                    if monitors.verbose {
                        print_line(&format!(
                            "[RT-thread] Pre-condition panicked for '{}' — skipping",
                            node.name
                        ));
                    }
                    return;
                }
            }
        }

        // Begin recording tick (before execution)
        if let Some(ref mut recorder) = node.recorder {
            recorder.begin_tick(0); // RT thread has no global tick counter
        }

        // Execute tick via NodeRunner
        let tr = NodeRunner::run_tick(&mut node.node);

        // RT post-conditions (only on success, catch_unwind: user code may panic)
        if tr.result.is_ok() {
            match catch_unwind(AssertUnwindSafe(|| node.node.post_condition())) {
                Ok(true) => {}
                Ok(false) => {
                    if monitors.verbose {
                        print_line(&format!(
                            "[RT-thread] Post-condition failed for '{}'",
                            node.name
                        ));
                    }
                }
                Err(_) => {
                    if monitors.verbose {
                        print_line(&format!(
                            "[RT-thread] Post-condition panicked for '{}'",
                            node.name
                        ));
                    }
                }
            }
        }

        // Record execution stats
        if let Some(ref mut stats) = node.rt_stats {
            stats.record_execution(tr.duration);
        }

        // Profiler recording (shared with main thread)
        // Use try_lock to avoid priority inversion — skip if contended
        if let Ok(mut profiler) = monitors.profiler.try_lock() {
            profiler.record(&node.name, tr.duration);
        }

        // Record in node recorder
        if let Some(ref mut recorder) = node.recorder {
            recorder.end_tick(tr.duration.as_nanos() as u64);
        }

        // WCET budget check via TimingEnforcer
        if let Some(wcet_budget) = node.wcet_budget {
            if let Some(wcet_result) =
                TimingEnforcer::check_wcet(&node.name, tr.duration, wcet_budget)
            {
                if monitors.verbose {
                    print_line(&format!(
                        "[RT-thread] WCET violation in '{}': {:?} > {:?}",
                        node.name, wcet_result.violation.actual, wcet_result.violation.budget
                    ));
                }
                if let Some(ref mut stats) = node.rt_stats {
                    stats.wcet_violations += 1;
                }
                node.node.on_wcet_violation(&wcet_result.violation);
                // Record to blackbox (try_lock to avoid RT priority inversion)
                if let Some(ref bb) = monitors.blackbox {
                    if let Ok(mut bb) = bb.try_lock() {
                        bb.record(super::blackbox::BlackBoxEvent::WCETViolation {
                            name: node.name.to_string(),
                            budget_us: wcet_budget.as_micros() as u64,
                            actual_us: tr.duration.as_micros() as u64,
                        });
                    }
                }
            }
        }

        // Deadline check via TimingEnforcer
        if let Some(deadline) = node.deadline {
            let policy = node.node.deadline_miss_policy();

            if let Some(dm) = TimingEnforcer::check_deadline(tr.tick_start, deadline, policy) {
                if monitors.verbose {
                    print_line(&format!(
                        "[RT-thread] Deadline miss in '{}': {:?} > {:?}",
                        node.name, dm.elapsed, dm.deadline
                    ));
                }
                if let Some(ref mut stats) = node.rt_stats {
                    stats.record_deadline_miss();
                }
                // Record to blackbox (try_lock to avoid RT priority inversion)
                if let Some(ref bb) = monitors.blackbox {
                    if let Ok(mut bb) = bb.try_lock() {
                        bb.record(super::blackbox::BlackBoxEvent::DeadlineMiss {
                            name: node.name.to_string(),
                            deadline_us: deadline.as_micros() as u64,
                            actual_us: dm.elapsed.as_micros() as u64,
                        });
                    }
                }
                // Invoke RtNode callback
                node.node.on_deadline_miss(dm.elapsed, dm.deadline);

                match dm.action {
                    DeadlineAction::Warn => {}
                    DeadlineAction::Skip => {
                        node.is_paused = true;
                    }
                    DeadlineAction::EmergencyStop => {
                        print_line(&format!(
                            "[RT-thread] Emergency stop triggered by '{}'",
                            node.name
                        ));
                        running.store(false, Ordering::SeqCst);
                    }
                    DeadlineAction::Degrade => {
                        node.priority = node.priority.saturating_add(10);
                    }
                    DeadlineAction::Fallback => {
                        let fallback = node.node.fallback_node();
                        if let Some(fallback_node) = fallback {
                            let fallback_name: Arc<str> = Arc::from(fallback_node.name());
                            if monitors.verbose {
                                print_line(&format!(
                                    "[RT-thread] Switching '{}' to fallback '{}'",
                                    node.name, fallback_name
                                ));
                            }
                            node.node = super::types::NodeKind::new(fallback_node);
                            node.name = fallback_name;
                        }
                    }
                }
            }
        }

        // Handle tick result
        match tr.result {
            Ok(_) => {
                node.failure_handler.record_success();
                if let Some(ref mut ctx) = node.context {
                    ctx.record_tick();
                }
            }
            Err(panic_err) => {
                let action = node.failure_handler.record_failure();
                // Use try_lock to avoid priority inversion — skip if contended
                if let Ok(mut profiler) = monitors.profiler.try_lock() {
                    profiler.record_node_failure(&node.name);
                }
                let error_msg = if let Some(s) = panic_err.downcast_ref::<&str>() {
                    format!("[RT-thread] Node '{}' panicked: {}", node.name, s)
                } else if let Some(s) = panic_err.downcast_ref::<String>() {
                    format!("[RT-thread] Node '{}' panicked: {}", node.name, s)
                } else {
                    format!("[RT-thread] Node '{}' panicked (unknown)", node.name)
                };
                if monitors.verbose {
                    print_line(&error_msg);
                }

                // Call on_error handler
                node.node.on_error(&error_msg);

                match action {
                    super::fault_tolerance::FailureAction::StopScheduler
                    | super::fault_tolerance::FailureAction::FatalAfterRestarts => {
                        // Always print fatal stop — this is an emergency-level message
                        print_line(&format!(
                            "[RT-thread] Fatal failure in '{}' — stopping scheduler",
                            node.name
                        ));
                        running.store(false, Ordering::SeqCst);
                    }
                    super::fault_tolerance::FailureAction::RestartNode => {
                        if let Some(ref mut ctx) = node.context {
                            match ctx.restart() {
                                Ok(_) => {
                                    node.initialized = true;
                                }
                                Err(e) => {
                                    if monitors.verbose {
                                        print_line(&format!(
                                            "[RT-thread] Restart failed for '{}': {}",
                                            node.name, e
                                        ));
                                    }
                                    node.initialized = false;
                                }
                            }
                        }
                    }
                    super::fault_tolerance::FailureAction::SkipNode => {
                        // Circuit breaker opened — will skip via should_allow()
                    }
                    super::fault_tolerance::FailureAction::Continue => {
                        // Log already printed above
                    }
                }
            }
        }
    }

    /// Main function for the RT thread.
    ///
    /// Attempts SCHED_FIFO RT priority, then runs a tight tick loop executing
    /// each RT node sequentially via `NodeRunner::run_tick()`.
    fn rt_thread_main(
        mut nodes: Vec<RegisteredNode>,
        running: Arc<AtomicBool>,
        tick_period: Duration,
        monitors: SharedMonitors,
        rt_cpus: Vec<usize>,
    ) -> Vec<RegisteredNode> {
        // Try to elevate to SCHED_FIFO (best-effort — degrades gracefully)
        match super::rt::set_realtime_priority(80) {
            Ok(()) => {}
            Err(e) => {
                if monitors.verbose {
                    print_line(&format!(
                        "[RT-thread] Could not set SCHED_FIFO: {} (continuing with normal priority)",
                        e
                    ));
                }
            }
        }

        // Pin to recommended RT CPU(s) to avoid cache thrashing and timer interrupts
        if !rt_cpus.is_empty() {
            match super::rt::set_thread_affinity(&rt_cpus) {
                Ok(()) => {
                    if monitors.verbose {
                        print_line(&format!("[RT-thread] Pinned to CPU(s) {:?}", rt_cpus));
                    }
                }
                Err(e) => {
                    if monitors.verbose {
                        print_line(&format!(
                            "[RT-thread] Could not pin to CPU(s) {:?}: {} (continuing unpinned)",
                            rt_cpus, e
                        ));
                    }
                }
            }
        }

        // Pre-fault 64KB of stack to avoid page faults during first ticks
        crate::core::rt_config::prefault_stack(64 * 1024);
        if monitors.verbose {
            print_line("[RT-thread] Pre-faulted 64KB of stack");
        }

        if monitors.verbose {
            print_line(&format!(
                "[RT-thread] Started with {} nodes, tick period {:?}",
                nodes.len(),
                tick_period
            ));
        }

        while running.load(Ordering::Relaxed) {
            let loop_start = Instant::now();

            for node in nodes.iter_mut() {
                if !node.initialized || node.is_stopped {
                    continue;
                }

                // Auto-unpause (DeadlineMissPolicy::Skip skips one tick)
                if node.is_paused {
                    node.is_paused = false;
                    continue;
                }

                // Per-node rate limiting
                if let Some(rate_hz) = node.rate_hz {
                    if let Some(last_tick) = node.last_tick {
                        let elapsed = loop_start.duration_since(last_tick).as_secs_f64();
                        if elapsed < 1.0 / rate_hz {
                            continue;
                        }
                    }
                }

                // Circuit breaker / failure handler
                if !node.failure_handler.should_allow() {
                    continue;
                }

                // Guard all infrastructure + tick processing against panics.
                // If ANY infrastructure code panics (recorder, failure_handler,
                // timing enforcer, etc.), the node is stopped but the RT thread
                // continues ticking remaining nodes.
                let infra_result = catch_unwind(AssertUnwindSafe(|| {
                    Self::tick_node(node, &monitors, &running)
                }));

                match infra_result {
                    Ok(()) => {} // normal completion
                    Err(_) => {
                        if monitors.verbose {
                            print_line(&format!(
                                "[RT-thread] Infrastructure panic for '{}' — node stopped",
                                node.name
                            ));
                        }
                        node.is_stopped = true;
                    }
                }
            }

            // Sleep until next tick period
            let elapsed = loop_start.elapsed();
            if elapsed < tick_period {
                // Use spin-wait for sub-millisecond precision on RT thread
                if tick_period - elapsed < Duration::from_millis(1) {
                    // Spin-wait for very short sleeps (better jitter than thread::sleep)
                    while loop_start.elapsed() < tick_period {
                        std::hint::spin_loop();
                    }
                } else {
                    // Sleep for bulk of the time, then spin for the remainder
                    let sleep_dur = (tick_period - elapsed) - Duration::from_micros(500);
                    if sleep_dur > Duration::ZERO {
                        std::thread::sleep(sleep_dur);
                    }
                    while loop_start.elapsed() < tick_period {
                        std::hint::spin_loop();
                    }
                }
            }
        }

        if monitors.verbose {
            print_line(&format!(
                "[RT-thread] Stopped ({} nodes returning to scheduler)",
                nodes.len()
            ));
        }

        nodes
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::Node;
    use std::sync::Mutex;

    fn test_monitors() -> SharedMonitors {
        SharedMonitors {
            profiler: Arc::new(Mutex::new(super::super::profiler::RuntimeProfiler::new())),
            blackbox: None,
            verbose: true,
        }
    }

    struct CounterNode {
        name: String,
        count: Arc<std::sync::atomic::AtomicU64>,
    }

    impl Node for CounterNode {
        fn name(&self) -> &str {
            &self.name
        }
        fn tick(&mut self) {
            self.count
                .fetch_add(1, std::sync::atomic::Ordering::Relaxed);
        }
    }

    fn make_rt_registered(name: &str, count: Arc<std::sync::atomic::AtomicU64>) -> RegisteredNode {
        use super::super::fault_tolerance::FailureHandler;
        use crate::core::NodeInfo;

        let node = CounterNode {
            name: name.to_string(),
            count,
        };
        RegisteredNode {
            node: super::super::types::NodeKind::new(Box::new(node)),
            name: Arc::from(name),
            priority: 0,
            initialized: true,
            context: Some(NodeInfo::new(name.to_string())),
            rate_hz: None,
            last_tick: None,
            failure_handler: FailureHandler::new(
                super::super::fault_tolerance::FailurePolicy::Fatal,
            ),
            is_rt_node: true,
            wcet_budget: None,
            deadline: None,
            recorder: None,
            is_stopped: false,
            is_paused: false,
            rt_stats: None,
            execution_class: super::super::types::ExecutionClass::Rt,
            has_custom_failure_policy: false,
        }
    }

    #[test]
    fn test_rt_executor_runs_nodes() {
        let count = Arc::new(std::sync::atomic::AtomicU64::new(0));
        let nodes = vec![make_rt_registered("test_rt", count.clone())];
        let running = Arc::new(AtomicBool::new(true));

        let executor = RtExecutor::start(
            nodes,
            running.clone(),
            Duration::from_millis(1),
            test_monitors(),
            Vec::new(),
        );

        // Let it run for a bit
        std::thread::sleep(Duration::from_millis(50));

        // Stop
        running.store(false, Ordering::SeqCst);
        let returned_nodes = executor.stop();

        assert_eq!(returned_nodes.len(), 1);
        assert!(count.load(std::sync::atomic::Ordering::Relaxed) > 0);
    }

    #[test]
    fn test_rt_executor_respects_running_flag() {
        let count = Arc::new(std::sync::atomic::AtomicU64::new(0));
        let nodes = vec![make_rt_registered("test_rt", count.clone())];
        let running = Arc::new(AtomicBool::new(true));

        let executor = RtExecutor::start(
            nodes,
            running.clone(),
            Duration::from_millis(1),
            test_monitors(),
            Vec::new(),
        );

        // Stop immediately
        running.store(false, Ordering::SeqCst);
        let returned_nodes = executor.stop();

        assert_eq!(returned_nodes.len(), 1);
        // May have ticked 0 or a few times — that's fine
    }

    #[test]
    fn test_rt_executor_multiple_nodes() {
        let count1 = Arc::new(std::sync::atomic::AtomicU64::new(0));
        let count2 = Arc::new(std::sync::atomic::AtomicU64::new(0));
        let nodes = vec![
            make_rt_registered("rt_node_1", count1.clone()),
            make_rt_registered("rt_node_2", count2.clone()),
        ];
        let running = Arc::new(AtomicBool::new(true));

        let executor = RtExecutor::start(
            nodes,
            running.clone(),
            Duration::from_millis(1),
            test_monitors(),
            Vec::new(),
        );

        std::thread::sleep(Duration::from_millis(50));
        running.store(false, Ordering::SeqCst);
        let returned_nodes = executor.stop();

        assert_eq!(returned_nodes.len(), 2);
        // Both nodes should have ticked
        assert!(count1.load(std::sync::atomic::Ordering::Relaxed) > 0);
        assert!(count2.load(std::sync::atomic::Ordering::Relaxed) > 0);
    }

    struct PanicNode;
    impl Node for PanicNode {
        fn name(&self) -> &str {
            "panic_rt"
        }
        fn tick(&mut self) {
            panic!("intentional RT panic");
        }
    }

    #[test]
    fn test_rt_executor_handles_panic() {
        use super::super::fault_tolerance::FailureHandler;
        use crate::core::NodeInfo;

        let node = PanicNode;
        let registered = RegisteredNode {
            node: super::super::types::NodeKind::new(Box::new(node)),
            name: Arc::from("panic_rt"),
            priority: 0,
            initialized: true,
            context: Some(NodeInfo::new("panic_rt".to_string())),
            rate_hz: None,
            last_tick: None,
            failure_handler: FailureHandler::new(
                super::super::fault_tolerance::FailurePolicy::Fatal,
            ),
            is_rt_node: true,
            wcet_budget: None,
            deadline: None,
            recorder: None,
            is_stopped: false,
            is_paused: false,
            rt_stats: None,
            execution_class: super::super::types::ExecutionClass::Rt,
            has_custom_failure_policy: false,
        };

        let running = Arc::new(AtomicBool::new(true));
        let executor = RtExecutor::start(
            vec![registered],
            running.clone(),
            Duration::from_millis(1),
            test_monitors(),
            Vec::new(),
        );

        // The panic node with Fatal policy should stop the scheduler
        std::thread::sleep(Duration::from_millis(50));

        // running should have been set to false by the fatal policy
        assert!(!running.load(Ordering::Relaxed));

        let returned = executor.stop();
        assert_eq!(returned.len(), 1);
    }

    /// RtNode whose pre_condition panics — tests catch_unwind isolation
    struct PanicPreConditionNode {
        name: String,
        count: Arc<std::sync::atomic::AtomicU64>,
    }

    impl Node for PanicPreConditionNode {
        fn name(&self) -> &str {
            &self.name
        }
        fn tick(&mut self) {
            self.count
                .fetch_add(1, std::sync::atomic::Ordering::Relaxed);
        }
        fn wcet_budget(&self) -> Option<Duration> {
            Some(Duration::from_millis(10))
        }
        fn pre_condition(&self) -> bool {
            panic!("pre_condition panic");
        }
    }

    #[test]
    fn test_rt_executor_survives_precondition_panic() {
        use super::super::fault_tolerance::FailureHandler;
        use crate::core::NodeInfo;

        let panic_count = Arc::new(std::sync::atomic::AtomicU64::new(0));
        let normal_count = Arc::new(std::sync::atomic::AtomicU64::new(0));

        // Node with panicking pre_condition (RtNode variant)
        let panic_node = PanicPreConditionNode {
            name: "panic_pre".to_string(),
            count: panic_count.clone(),
        };
        let panic_registered = RegisteredNode {
            node: super::super::types::NodeKind::new(Box::new(panic_node)),
            name: Arc::from("panic_pre"),
            priority: 0,
            initialized: true,
            context: Some(NodeInfo::new("panic_pre".to_string())),
            rate_hz: None,
            last_tick: None,
            failure_handler: FailureHandler::new(
                super::super::fault_tolerance::FailurePolicy::Ignore,
            ),
            is_rt_node: true,
            wcet_budget: None,
            deadline: None,
            recorder: None,
            is_stopped: false,
            is_paused: false,
            rt_stats: None,
            execution_class: super::super::types::ExecutionClass::Rt,
            has_custom_failure_policy: false,
        };

        // Normal node that should still tick
        let normal_registered = make_rt_registered("normal_rt", normal_count.clone());

        let running = Arc::new(AtomicBool::new(true));
        let executor = RtExecutor::start(
            vec![panic_registered, normal_registered],
            running.clone(),
            Duration::from_millis(1),
            test_monitors(),
            Vec::new(),
        );

        std::thread::sleep(Duration::from_millis(100));
        running.store(false, Ordering::SeqCst);
        let returned = executor.stop();

        // RT thread should NOT have crashed
        assert_eq!(returned.len(), 2);
        // Pre-condition panicked → tick was skipped → count should be 0
        assert_eq!(
            panic_count.load(std::sync::atomic::Ordering::Relaxed),
            0,
            "Panicking pre_condition node should never tick"
        );
        // Normal node should have ticked many times
        assert!(
            normal_count.load(std::sync::atomic::Ordering::Relaxed) > 0,
            "Normal node should still tick despite sibling's pre_condition panic"
        );
    }

    fn make_rt_with_rate(
        name: &str,
        count: Arc<std::sync::atomic::AtomicU64>,
        rate_hz: f64,
    ) -> RegisteredNode {
        let mut node = make_rt_registered(name, count);
        node.rate_hz = Some(rate_hz);
        node
    }

    #[test]
    fn test_multi_rate_timing_wheel() {
        // Simulate IMU at 1kHz and lidar at 10Hz on same RT thread
        let fast_count = Arc::new(std::sync::atomic::AtomicU64::new(0));
        let slow_count = Arc::new(std::sync::atomic::AtomicU64::new(0));

        let nodes = vec![
            make_rt_with_rate("imu_1khz", fast_count.clone(), 1000.0),
            make_rt_with_rate("lidar_10hz", slow_count.clone(), 10.0),
        ];
        let running = Arc::new(AtomicBool::new(true));

        let executor = RtExecutor::start(
            nodes,
            running.clone(),
            Duration::from_millis(10),
            test_monitors(),
            Vec::new(),
        );

        // Run for 200ms
        std::thread::sleep(Duration::from_millis(200));
        running.store(false, Ordering::SeqCst);
        let _returned = executor.stop();

        let fast_ticks = fast_count.load(std::sync::atomic::Ordering::Relaxed);
        let slow_ticks = slow_count.load(std::sync::atomic::Ordering::Relaxed);

        // 1kHz for 200ms → ~200 ticks (very wide margin for debug builds on non-RT kernel)
        assert!(
            fast_ticks >= 10,
            "1kHz node should tick at least 10 times in 200ms, got {}",
            fast_ticks
        );

        // 10Hz for 200ms → ~2 ticks
        assert!(
            slow_ticks >= 1 && slow_ticks <= 10,
            "10Hz node should tick 1-10 times in 200ms, got {}",
            slow_ticks
        );

        // Fast node should tick significantly more than slow node
        assert!(
            fast_ticks > slow_ticks * 5,
            "1kHz node ({}) should tick >5x more than 10Hz node ({})",
            fast_ticks,
            slow_ticks
        );
    }

    #[test]
    fn test_multi_rate_three_rates() {
        // Three different rates on same RT thread: 500Hz, 100Hz, 20Hz
        let count_500 = Arc::new(std::sync::atomic::AtomicU64::new(0));
        let count_100 = Arc::new(std::sync::atomic::AtomicU64::new(0));
        let count_20 = Arc::new(std::sync::atomic::AtomicU64::new(0));

        let nodes = vec![
            make_rt_with_rate("motor_500hz", count_500.clone(), 500.0),
            make_rt_with_rate("sensor_100hz", count_100.clone(), 100.0),
            make_rt_with_rate("planner_20hz", count_20.clone(), 20.0),
        ];
        let running = Arc::new(AtomicBool::new(true));

        let executor = RtExecutor::start(
            nodes,
            running.clone(),
            Duration::from_millis(10),
            test_monitors(),
            Vec::new(),
        );

        // Run for 200ms
        std::thread::sleep(Duration::from_millis(200));
        running.store(false, Ordering::SeqCst);
        let _returned = executor.stop();

        let ticks_500 = count_500.load(std::sync::atomic::Ordering::Relaxed);
        let ticks_100 = count_100.load(std::sync::atomic::Ordering::Relaxed);
        let ticks_20 = count_20.load(std::sync::atomic::Ordering::Relaxed);

        // Verify ordering: 500Hz > 100Hz > 20Hz
        assert!(
            ticks_500 > ticks_100,
            "500Hz ({}) should tick more than 100Hz ({})",
            ticks_500,
            ticks_100
        );
        assert!(
            ticks_100 > ticks_20,
            "100Hz ({}) should tick more than 20Hz ({})",
            ticks_100,
            ticks_20
        );

        // Verify approximate ratios (with wide margins for non-RT kernel)
        // 500Hz should tick ~5x more than 100Hz
        assert!(
            ticks_500 >= ticks_100 * 2,
            "500Hz ({}) should be at least 2x of 100Hz ({})",
            ticks_500,
            ticks_100
        );
    }

    // ========================================================================
    // Stress tests for production hardening (RT safety & crash resilience)
    // ========================================================================

    /// Stress test: contended profiler lock should not block RT thread.
    ///
    /// A background thread holds the profiler Mutex for 50ms. Because the RT
    /// executor uses try_lock(), it should skip profiling without blocking.
    #[test]
    fn test_stress_contended_profiler_no_deadlock() {
        let count = Arc::new(std::sync::atomic::AtomicU64::new(0));
        let monitors = test_monitors();
        let profiler_arc = monitors.profiler.clone();

        let nodes = vec![make_rt_registered("stress_profiler", count.clone())];
        let running = Arc::new(AtomicBool::new(true));

        let executor = RtExecutor::start(
            nodes,
            running.clone(),
            Duration::from_millis(1),
            monitors,
            Vec::new(),
        );

        // Background thread holds profiler lock for 50ms
        let lock_thread = std::thread::spawn(move || {
            let _guard = profiler_arc.lock().unwrap();
            std::thread::sleep(Duration::from_millis(50));
        });

        // Let RT thread run concurrently with the contended lock
        std::thread::sleep(Duration::from_millis(100));
        running.store(false, Ordering::SeqCst);
        let returned = executor.stop();
        lock_thread.join().unwrap();

        assert_eq!(returned.len(), 1);
        // RT thread should have been ticking throughout (not blocked)
        let ticks = count.load(std::sync::atomic::Ordering::Relaxed);
        assert!(
            ticks > 10,
            "RT thread should have ticked many times despite profiler contention, got {}",
            ticks
        );
    }

    /// RtNode whose post_condition panics — tests catch_unwind isolation
    struct PanicPostConditionNode {
        name: String,
        count: Arc<std::sync::atomic::AtomicU64>,
    }

    impl Node for PanicPostConditionNode {
        fn name(&self) -> &str {
            &self.name
        }
        fn tick(&mut self) {
            self.count
                .fetch_add(1, std::sync::atomic::Ordering::Relaxed);
        }
        fn wcet_budget(&self) -> Option<Duration> {
            Some(Duration::from_millis(10))
        }
        fn post_condition(&self) -> bool {
            panic!("post_condition panic");
        }
    }

    /// Stress test: panicking post_condition should not crash RT executor.
    ///
    /// The RT thread should catch the panic, log a warning, and continue
    /// ticking other nodes.
    #[test]
    fn test_stress_postcondition_panic_survives() {
        use super::super::fault_tolerance::FailureHandler;
        use crate::core::NodeInfo;

        let panic_count = Arc::new(std::sync::atomic::AtomicU64::new(0));
        let normal_count = Arc::new(std::sync::atomic::AtomicU64::new(0));

        let panic_node = PanicPostConditionNode {
            name: "panic_post".to_string(),
            count: panic_count.clone(),
        };
        let panic_registered = RegisteredNode {
            node: super::super::types::NodeKind::new(Box::new(panic_node)),
            name: Arc::from("panic_post"),
            priority: 0,
            initialized: true,
            context: Some(NodeInfo::new("panic_post".to_string())),
            rate_hz: None,
            last_tick: None,
            failure_handler: FailureHandler::new(
                super::super::fault_tolerance::FailurePolicy::Ignore,
            ),
            is_rt_node: true,
            wcet_budget: None,
            deadline: None,
            recorder: None,
            is_stopped: false,
            is_paused: false,
            rt_stats: None,
            execution_class: super::super::types::ExecutionClass::Rt,
            has_custom_failure_policy: false,
        };

        let normal_registered = make_rt_registered("normal_beside_post", normal_count.clone());

        let running = Arc::new(AtomicBool::new(true));
        let executor = RtExecutor::start(
            vec![panic_registered, normal_registered],
            running.clone(),
            Duration::from_millis(1),
            test_monitors(),
            Vec::new(),
        );

        std::thread::sleep(Duration::from_millis(100));
        running.store(false, Ordering::SeqCst);
        let returned = executor.stop();

        assert_eq!(returned.len(), 2);
        // Post-condition panics inside catch_unwind — the node's tick DID run
        // but infrastructure panic marks it stopped after first iteration
        // Normal node should keep ticking regardless
        assert!(
            normal_count.load(std::sync::atomic::Ordering::Relaxed) > 0,
            "Normal node should still tick despite sibling's post_condition panic"
        );
    }

    /// Node that panics on every tick — Fatal policy triggers scheduler stop.
    struct AlwaysPanicNode {
        name: String,
        count: Arc<std::sync::atomic::AtomicU64>,
    }

    impl Node for AlwaysPanicNode {
        fn name(&self) -> &str {
            &self.name
        }
        fn tick(&mut self) {
            self.count
                .fetch_add(1, std::sync::atomic::Ordering::Relaxed);
            panic!("simulated persistent panic");
        }
    }

    /// Stress test: tick panic with Ignore policy — node continues, sibling unaffected.
    ///
    /// The tick panic is caught by NodeRunner::run_tick's catch_unwind. With Ignore
    /// policy, the failure handler lets the node continue. The outer infrastructure
    /// catch_unwind handles panics in post-tick infrastructure (profiler, recorder,
    /// timing enforcer) — not in the tick itself.
    #[test]
    fn test_stress_tick_panic_ignore_policy_continues() {
        use super::super::fault_tolerance::FailureHandler;
        use crate::core::NodeInfo;

        let panic_count = Arc::new(std::sync::atomic::AtomicU64::new(0));
        let normal_count = Arc::new(std::sync::atomic::AtomicU64::new(0));

        let panic_node = AlwaysPanicNode {
            name: "always_panic".to_string(),
            count: panic_count.clone(),
        };
        let panic_registered = RegisteredNode {
            node: super::super::types::NodeKind::new(Box::new(panic_node)),
            name: Arc::from("always_panic"),
            priority: 0,
            initialized: true,
            context: Some(NodeInfo::new("always_panic".to_string())),
            rate_hz: None,
            last_tick: None,
            failure_handler: FailureHandler::new(
                super::super::fault_tolerance::FailurePolicy::Ignore,
            ),
            is_rt_node: true,
            wcet_budget: None,
            deadline: None,
            recorder: None,
            is_stopped: false,
            is_paused: false,
            rt_stats: None,
            execution_class: super::super::types::ExecutionClass::Rt,
            has_custom_failure_policy: false,
        };

        let normal_registered = make_rt_registered("survivor_node", normal_count.clone());

        let running = Arc::new(AtomicBool::new(true));
        let executor = RtExecutor::start(
            vec![panic_registered, normal_registered],
            running.clone(),
            Duration::from_millis(1),
            test_monitors(),
            Vec::new(),
        );

        std::thread::sleep(Duration::from_millis(100));
        running.store(false, Ordering::SeqCst);
        let returned = executor.stop();

        // RT thread should NOT have crashed
        assert_eq!(returned.len(), 2);

        // The panicking node should have attempted multiple ticks (Ignore policy)
        let panic_ticks = panic_count.load(std::sync::atomic::Ordering::Relaxed);
        assert!(
            panic_ticks > 0,
            "Panicking node should have been ticked (panic caught by NodeRunner)"
        );

        // Normal node should keep ticking regardless
        assert!(
            normal_count.load(std::sync::atomic::Ordering::Relaxed) > 10,
            "Survivor node should keep ticking despite sibling's repeated panics"
        );
    }

    /// Stress test: verbose=false suppresses RT thread output.
    ///
    /// We can't easily capture stdout in Rust tests, but we verify the executor
    /// runs correctly with verbose=false and doesn't crash.
    #[test]
    fn test_stress_verbose_false_runs_clean() {
        let count = Arc::new(std::sync::atomic::AtomicU64::new(0));
        let monitors = SharedMonitors {
            profiler: Arc::new(Mutex::new(super::super::profiler::RuntimeProfiler::new())),
            blackbox: None,
            verbose: false,
        };

        let nodes = vec![make_rt_registered("quiet_node", count.clone())];
        let running = Arc::new(AtomicBool::new(true));

        let executor = RtExecutor::start(
            nodes,
            running.clone(),
            Duration::from_millis(1),
            monitors,
            Vec::new(),
        );

        std::thread::sleep(Duration::from_millis(50));
        running.store(false, Ordering::SeqCst);
        let returned = executor.stop();

        assert_eq!(returned.len(), 1);
        assert!(
            count.load(std::sync::atomic::Ordering::Relaxed) > 0,
            "Node should still tick with verbose=false"
        );
    }

    /// Stress test: verbose=false with panicking pre_condition.
    ///
    /// Combines verbose=false with a panicking pre_condition to verify
    /// that the panic message path doesn't crash when verbose is disabled.
    #[test]
    fn test_stress_verbose_false_with_panic() {
        use super::super::fault_tolerance::FailureHandler;
        use crate::core::NodeInfo;

        let normal_count = Arc::new(std::sync::atomic::AtomicU64::new(0));
        let panic_count = Arc::new(std::sync::atomic::AtomicU64::new(0));

        let panic_node = PanicPreConditionNode {
            name: "quiet_panic".to_string(),
            count: panic_count,
        };
        let panic_registered = RegisteredNode {
            node: super::super::types::NodeKind::new(Box::new(panic_node)),
            name: Arc::from("quiet_panic"),
            priority: 0,
            initialized: true,
            context: Some(NodeInfo::new("quiet_panic".to_string())),
            rate_hz: None,
            last_tick: None,
            failure_handler: FailureHandler::new(
                super::super::fault_tolerance::FailurePolicy::Ignore,
            ),
            is_rt_node: true,
            wcet_budget: None,
            deadline: None,
            recorder: None,
            is_stopped: false,
            is_paused: false,
            rt_stats: None,
            execution_class: super::super::types::ExecutionClass::Rt,
            has_custom_failure_policy: false,
        };

        let normal_registered = make_rt_registered("quiet_normal", normal_count.clone());

        let monitors = SharedMonitors {
            profiler: Arc::new(Mutex::new(super::super::profiler::RuntimeProfiler::new())),
            blackbox: None,
            verbose: false,
        };

        let running = Arc::new(AtomicBool::new(true));
        let executor = RtExecutor::start(
            vec![panic_registered, normal_registered],
            running.clone(),
            Duration::from_millis(1),
            monitors,
            Vec::new(),
        );

        std::thread::sleep(Duration::from_millis(100));
        running.store(false, Ordering::SeqCst);
        let returned = executor.stop();

        assert_eq!(returned.len(), 2);
        assert!(
            normal_count.load(std::sync::atomic::Ordering::Relaxed) > 0,
            "Normal node should tick even with verbose=false and sibling panic"
        );
    }
}
