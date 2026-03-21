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
use crate::core::DurationExt;

/// Dedicated RT thread executor.
///
/// Owns RT nodes and ticks them on isolated OS thread(s) at the rate of
/// the fastest node. Shutdown is coordinated via the shared `running` flag.
///
/// Supports multiple RT threads for independent node chains — each chain
/// gets its own thread with optional CPU pinning and priority.
pub(crate) struct RtExecutor {
    handles: Vec<std::thread::JoinHandle<Vec<RegisteredNode>>>,
}

impl RtExecutor {
    /// Start the RT executor with the given nodes on a dedicated thread.
    ///
    /// `running` is the shared scheduler running flag — when set to `false`,
    /// the RT thread finishes its current tick cycle and returns its nodes.
    ///
    /// The tick period is derived from the fastest RT node's rate. If no node
    /// has a declared rate, falls back to `fallback_period`.
    /// Start the RT executor with multiple independent chains on separate threads.
    ///
    /// Each chain gets its own dedicated RT thread. Independent chains run in
    /// parallel — a slow node in chain 1 cannot block chain 2.
    ///
    /// Falls back to single thread when given 1 chain.
    pub fn start_pool(
        chains: Vec<Vec<RegisteredNode>>,
        running: Arc<AtomicBool>,
        fallback_period: Duration,
        monitors: SharedMonitors,
        rt_cpus: Vec<usize>,
    ) -> Self {
        let num_chains = chains.len();
        let mut handles = Vec::with_capacity(num_chains);

        for (chain_idx, mut nodes) in chains.into_iter().enumerate() {
            // Sort by priority before handing off to the thread
            nodes.sort_by_key(|n| n.priority);

            // Determine tick period from the fastest node in this chain
            let max_rate_hz = nodes
                .iter()
                .filter_map(|n| n.rate_hz)
                .fold(0.0_f64, f64::max);

            let tick_period = if max_rate_hz > 0.0 {
                max_rate_hz.hz().period()
            } else {
                fallback_period
            };

            // Assign CPU core: round-robin across available RT CPUs
            let thread_cpus = if !rt_cpus.is_empty() {
                vec![rt_cpus[chain_idx % rt_cpus.len()]]
            } else {
                vec![]
            };

            let thread_name = if num_chains == 1 {
                "horus-rt".to_string()
            } else {
                format!("horus-rt-{}", chain_idx)
            };

            let running = running.clone();
            let monitors = monitors.clone();

            let handle = std::thread::Builder::new()
                .name(thread_name)
                .spawn(move || {
                    Self::rt_thread_main(nodes, running, tick_period, monitors, thread_cpus)
                })
                .expect("Failed to spawn RT thread");

            handles.push(handle);
        }

        Self { handles }
    }

    /// Stop the RT executor and reclaim its nodes.
    ///
    /// The caller should have already set `running` to `false` before calling this.
    /// Each RT thread gets up to 3 seconds to exit cleanly. If a thread is stuck
    /// (stalled `tick()`, deadlock, infinite loop), it is detached after the timeout
    /// to prevent the entire scheduler shutdown from hanging.
    ///
    /// # Safety guarantee
    /// Shutdown always completes within `SHUTDOWN_TIMEOUT_PER_THREAD × num_threads`.
    /// A single stalled node cannot block the process from exiting.
    pub fn stop(mut self) -> Vec<RegisteredNode> {
        /// Maximum time to wait for each RT thread to exit during shutdown.
        /// If a thread doesn't exit within this time, it is detached.
        const SHUTDOWN_TIMEOUT: Duration = Duration::from_secs(3);

        let mut all_nodes = Vec::new();
        for (i, handle) in std::mem::take(&mut self.handles).into_iter().enumerate() {
            let start = Instant::now();
            loop {
                if handle.is_finished() {
                    match handle.join() {
                        Ok(nodes) => all_nodes.extend(nodes),
                        Err(_) => {
                            print_line(&format!("[RT-thread] Thread {} panicked during stop", i));
                        }
                    }
                    break;
                }
                if start.elapsed() > SHUTDOWN_TIMEOUT {
                    print_line(&format!(
                        "[RT-thread] Thread {} did not exit within {:?} — \
                         detaching (possible stalled tick). The thread will be \
                         terminated when the process exits.",
                        i, SHUTDOWN_TIMEOUT
                    ));
                    // Drop JoinHandle without joining. The thread continues
                    // running but dies when the process exits. Nodes on this
                    // thread are lost (not returned to the scheduler).
                    drop(handle);
                    break;
                }
                std::thread::sleep(Duration::from_millis(10));
            }
        }
        all_nodes
    }

    /// Process a single node tick with all infrastructure (stats, profiler, budget, deadline).
    ///
    /// Extracted from the RT loop body so the caller can wrap this in `catch_unwind`.
    /// If this function panics, the caller marks the node as stopped.
    fn tick_node(node: &mut RegisteredNode, monitors: &SharedMonitors, running: &Arc<AtomicBool>) {
        // Update last tick time
        if node.rate_hz.is_some() {
            node.last_tick = Some(Instant::now());
        }

        // Begin recording tick (before execution)
        if let Some(ref mut recorder) = node.recorder {
            recorder.begin_tick(0); // RT thread has no global tick counter
        }

        // Execute tick via NodeRunner
        let tr = NodeRunner::run_tick(&mut node.node);

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

        // tick budget check via TimingEnforcer
        if let Some(tick_budget) = node.tick_budget {
            if let Some(budget_result) =
                TimingEnforcer::check_tick_budget(&node.name, tr.duration, tick_budget)
            {
                if monitors.verbose {
                    print_line(&format!(
                        "[RT-thread] budget violation in '{}': {:?} > {:?}",
                        node.name,
                        budget_result.violation.actual(),
                        budget_result.violation.budget()
                    ));
                }
                if let Some(ref mut stats) = node.rt_stats {
                    stats.record_budget_violation();
                }
                // Record to blackbox (try_lock to avoid RT priority inversion)
                if let Some(ref bb) = monitors.blackbox {
                    if let Ok(mut bb) = bb.try_lock() {
                        bb.record(super::blackbox::BlackBoxEvent::BudgetViolation {
                            name: node.name.to_string(),
                            budget_us: tick_budget.as_micros() as u64,
                            actual_us: tr.duration.as_micros() as u64,
                        });
                    }
                }
                // Budget enforcement based on per-node policy.
                // Post-tick enforcement (safe — tick completed, no shared state issues).
                use super::safety_monitor::BudgetPolicy;
                match node.budget_policy {
                    BudgetPolicy::Warn => {
                        // Default: log only (already logged above)
                    }
                    BudgetPolicy::Enforce => {
                        // Stop node if tick exceeded 2x budget
                        if tr.duration > tick_budget * 2 {
                            print_line(&format!(
                                "[RT-thread] BUDGET ENFORCE: '{}' exceeded 2x budget ({:?} > {:?}) — node stopped",
                                node.name, tr.duration, tick_budget * 2
                            ));
                            node.node.shutdown();
                            node.is_stopped = true;
                        }
                    }
                    BudgetPolicy::EmergencyStop => {
                        // Any budget violation triggers e-stop
                        print_line(&format!(
                            "[RT-thread] BUDGET E-STOP: '{}' budget violation ({:?} > {:?})",
                            node.name, tr.duration, tick_budget
                        ));
                        node.node.shutdown();
                        node.is_stopped = true;
                        // Signal stop via running flag — RT thread will exit
                        running.store(false, Ordering::SeqCst);
                    }
                }
            }
        }

        // Deadline check via TimingEnforcer
        if let Some(deadline) = node.deadline {
            if let Some(dm) =
                TimingEnforcer::check_deadline(tr.tick_start, deadline, node.miss_policy)
            {
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
                match dm.action {
                    DeadlineAction::Warn => {}
                    DeadlineAction::Skip => {
                        node.is_paused = true;
                    }
                    DeadlineAction::SafeMode => {
                        if monitors.verbose {
                            print_line(&format!(
                                "[RT-thread] SafeMode: '{}' entering safe state after deadline miss",
                                node.name
                            ));
                        }
                        node.node.enter_safe_state();
                    }
                    DeadlineAction::EmergencyStop => {
                        print_line(&format!(
                            "[RT-thread] Emergency stop triggered by '{}'",
                            node.name
                        ));
                        running.store(false, Ordering::SeqCst);
                    }
                }
            }
        }

        // Handle tick result
        match tr.result {
            Ok(_) => {
                if let Some(ref mut ctx) = node.context {
                    ctx.record_tick();
                }
            }
            Err(panic_err) => {
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
        // Use per-node priority if any node in this chain has one, otherwise default 80
        let thread_priority = nodes
            .iter()
            .filter_map(|n| n.os_priority)
            .max()
            .unwrap_or(80);

        // Try to elevate to SCHED_FIFO (best-effort — degrades gracefully)
        match super::rt::set_realtime_priority(thread_priority) {
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

        // Per-node core override: if any node in this chain specifies .core(), use that
        let effective_cpus = {
            let node_core = nodes.iter().filter_map(|n| n.pinned_core).next();
            if let Some(core) = node_core {
                vec![core]
            } else {
                rt_cpus.clone()
            }
        };

        // Pin to recommended RT CPU(s) to avoid cache thrashing and timer interrupts
        let rt_cpus = effective_cpus;
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

                // Auto-unpause (Miss::Skip skips one tick)
                if node.is_paused {
                    node.is_paused = false;
                    continue;
                }

                // Per-node rate limiting
                if let Some(rate_hz) = node.rate_hz {
                    if let Some(last_tick) = node.last_tick {
                        let elapsed = loop_start.duration_since(last_tick).as_secs_f64();
                        if rate_hz > 0.0 && elapsed < 1.0 / rate_hz {
                            continue;
                        }
                    }
                }

                // Guard all infrastructure + tick processing against panics.
                // If ANY infrastructure code panics (recorder,
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
                if tick_period - elapsed < 1_u64.ms() {
                    // Spin-wait for very short sleeps (better jitter than thread::sleep)
                    while loop_start.elapsed() < tick_period {
                        std::hint::spin_loop();
                    }
                } else {
                    // Sleep for bulk of the time, then spin for the remainder
                    let sleep_dur = (tick_period - elapsed) - 500_u64.us();
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

impl Drop for RtExecutor {
    fn drop(&mut self) {
        for handle in self.handles.drain(..) {
            let _ = handle.join();
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::{Miss, Node};
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
            is_rt_node: true,
            tick_budget: None,
            deadline: None,
            recorder: None,
            is_stopped: false,
            is_paused: false,
            rt_stats: None,
            miss_policy: Miss::Warn,
            execution_class: super::super::types::ExecutionClass::Rt,
            health_state: super::super::types::AtomicHealthState::default(),
            os_priority: None,
            pinned_core: None,
            node_watchdog: None,
            failure_handler: None,
            budget_policy: super::super::safety_monitor::BudgetPolicy::default(),
        }
    }

    #[test]
    fn test_rt_executor_runs_nodes() {
        let count = Arc::new(std::sync::atomic::AtomicU64::new(0));
        let nodes = vec![make_rt_registered("test_rt", count.clone())];
        let running = Arc::new(AtomicBool::new(true));

        let executor = RtExecutor::start_pool(
            vec![nodes],
            running.clone(),
            1_u64.ms(),
            test_monitors(),
            Vec::new(),
        );

        // Let it run for a bit
        std::thread::sleep(50_u64.ms());

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

        let executor = RtExecutor::start_pool(
            vec![nodes],
            running.clone(),
            1_u64.ms(),
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

        let executor = RtExecutor::start_pool(
            vec![nodes],
            running.clone(),
            1_u64.ms(),
            test_monitors(),
            Vec::new(),
        );

        std::thread::sleep(50_u64.ms());
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
            is_rt_node: true,
            tick_budget: None,
            deadline: None,
            recorder: None,
            is_stopped: false,
            is_paused: false,
            rt_stats: None,
            miss_policy: Miss::Warn,
            execution_class: super::super::types::ExecutionClass::Rt,
            health_state: super::super::types::AtomicHealthState::default(),
            os_priority: None,
            pinned_core: None,
            node_watchdog: None,
            failure_handler: None,
            budget_policy: super::super::safety_monitor::BudgetPolicy::default(),
        };

        let running = Arc::new(AtomicBool::new(true));
        let executor = RtExecutor::start_pool(
            vec![vec![registered]],
            running.clone(),
            1_u64.ms(),
            test_monitors(),
            Vec::new(),
        );

        // The panic node should log errors and continue (no longer stops scheduler)
        std::thread::sleep(50_u64.ms());

        // Signal the executor to stop before calling stop() (which joins the thread)
        running.store(false, Ordering::SeqCst);
        let returned = executor.stop();
        assert_eq!(returned.len(), 1);
    }

    /// Node that just ticks normally -- used to verify siblings survive panicking nodes.
    struct SimpleCounterNode {
        name: String,
        count: Arc<std::sync::atomic::AtomicU64>,
    }

    impl Node for SimpleCounterNode {
        fn name(&self) -> &str {
            &self.name
        }
        fn tick(&mut self) {
            self.count
                .fetch_add(1, std::sync::atomic::Ordering::Relaxed);
        }
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

        let executor = RtExecutor::start_pool(
            vec![nodes],
            running.clone(),
            10_u64.ms(),
            test_monitors(),
            Vec::new(),
        );

        // Run for 200ms
        std::thread::sleep(200_u64.ms());
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
            (1..=10).contains(&slow_ticks),
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

        let executor = RtExecutor::start_pool(
            vec![nodes],
            running.clone(),
            10_u64.ms(),
            test_monitors(),
            Vec::new(),
        );

        // Run for 200ms
        std::thread::sleep(200_u64.ms());
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

        let executor = RtExecutor::start_pool(
            vec![nodes],
            running.clone(),
            1_u64.ms(),
            monitors,
            Vec::new(),
        );

        // Background thread holds profiler lock for 50ms
        let lock_thread = std::thread::spawn(move || {
            let _guard = profiler_arc.lock().unwrap();
            std::thread::sleep(50_u64.ms());
        });

        // Let RT thread run concurrently with the contended lock
        std::thread::sleep(100_u64.ms());
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

    /// Node that panics on every tick -- used for panic survival tests.
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

    /// Stress test: tick panic -- node logs error and continues, sibling unaffected.
    ///
    /// The tick panic is caught by NodeRunner::run_tick's catch_unwind. The error
    /// is logged and the node continues. The outer infrastructure catch_unwind
    /// handles panics in post-tick infrastructure (profiler, recorder,
    /// timing enforcer) -- not in the tick itself.
    #[test]
    fn test_stress_tick_panic_continues() {
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
            is_rt_node: true,
            tick_budget: None,
            deadline: None,
            recorder: None,
            is_stopped: false,
            is_paused: false,
            rt_stats: None,
            miss_policy: Miss::Warn,
            execution_class: super::super::types::ExecutionClass::Rt,
            health_state: super::super::types::AtomicHealthState::default(),
            os_priority: None,
            pinned_core: None,
            node_watchdog: None,
            failure_handler: None,
            budget_policy: super::super::safety_monitor::BudgetPolicy::default(),
        };

        let normal_registered = make_rt_registered("survivor_node", normal_count.clone());

        let running = Arc::new(AtomicBool::new(true));
        let executor = RtExecutor::start_pool(
            vec![vec![panic_registered, normal_registered]],
            running.clone(),
            1_u64.ms(),
            test_monitors(),
            Vec::new(),
        );

        std::thread::sleep(100_u64.ms());
        running.store(false, Ordering::SeqCst);
        let returned = executor.stop();

        // RT thread should NOT have crashed
        assert_eq!(returned.len(), 2);

        // The panicking node should have attempted multiple ticks
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

        let executor = RtExecutor::start_pool(
            vec![nodes],
            running.clone(),
            1_u64.ms(),
            monitors,
            Vec::new(),
        );

        std::thread::sleep(50_u64.ms());
        running.store(false, Ordering::SeqCst);
        let returned = executor.stop();

        assert_eq!(returned.len(), 1);
        assert!(
            count.load(std::sync::atomic::Ordering::Relaxed) > 0,
            "Node should still tick with verbose=false"
        );
    }

    /// Stress test: verbose=false with a panicking node.
    ///
    /// Combines verbose=false with a panicking tick to verify
    /// that the error logging path doesn't crash when verbose is disabled.
    #[test]
    fn test_stress_verbose_false_with_panic() {
        use crate::core::NodeInfo;

        let normal_count = Arc::new(std::sync::atomic::AtomicU64::new(0));
        let panic_count = Arc::new(std::sync::atomic::AtomicU64::new(0));

        let panic_node = AlwaysPanicNode {
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
            is_rt_node: true,
            tick_budget: None,
            deadline: None,
            recorder: None,
            is_stopped: false,
            is_paused: false,
            rt_stats: None,
            miss_policy: Miss::Warn,
            execution_class: super::super::types::ExecutionClass::Rt,
            health_state: super::super::types::AtomicHealthState::default(),
            os_priority: None,
            pinned_core: None,
            node_watchdog: None,
            failure_handler: None,
            budget_policy: super::super::safety_monitor::BudgetPolicy::default(),
        };

        let normal_registered = make_rt_registered("quiet_normal", normal_count.clone());

        let monitors = SharedMonitors {
            profiler: Arc::new(Mutex::new(super::super::profiler::RuntimeProfiler::new())),
            blackbox: None,
            verbose: false,
        };

        let running = Arc::new(AtomicBool::new(true));
        let executor = RtExecutor::start_pool(
            vec![vec![panic_registered, normal_registered]],
            running.clone(),
            1_u64.ms(),
            monitors,
            Vec::new(),
        );

        std::thread::sleep(100_u64.ms());
        running.store(false, Ordering::SeqCst);
        let returned = executor.stop();

        assert_eq!(returned.len(), 2);
        assert!(
            normal_count.load(std::sync::atomic::Ordering::Relaxed) > 0,
            "Normal node should tick even with verbose=false and sibling panic"
        );
    }

    // ========================================================================
    // Multi-chain parallel execution tests
    // ========================================================================

    #[test]
    fn test_multi_chain_independent_execution() {
        // Two chains on separate threads — both must tick independently
        let count_a = Arc::new(std::sync::atomic::AtomicU64::new(0));
        let count_b = Arc::new(std::sync::atomic::AtomicU64::new(0));

        let chain_0 = vec![make_rt_registered("chain0_node", count_a.clone())];
        let chain_1 = vec![make_rt_registered("chain1_node", count_b.clone())];

        let running = Arc::new(AtomicBool::new(true));

        let executor = RtExecutor::start_pool(
            vec![chain_0, chain_1],
            running.clone(),
            1_u64.ms(),
            test_monitors(),
            Vec::new(),
        );

        std::thread::sleep(Duration::from_millis(50));
        running.store(false, Ordering::SeqCst);
        let returned = executor.stop();

        let ticks_a = count_a.load(std::sync::atomic::Ordering::Relaxed);
        let ticks_b = count_b.load(std::sync::atomic::Ordering::Relaxed);

        assert!(ticks_a > 0, "chain 0 must tick: got {}", ticks_a);
        assert!(ticks_b > 0, "chain 1 must tick: got {}", ticks_b);
        assert_eq!(returned.len(), 2, "both nodes returned on stop");
    }

    #[test]
    fn test_multi_chain_panic_in_one_does_not_block_other() {
        // Chain 0 has a panicking node. Chain 1 must still tick.
        struct PanicNode;
        impl Node for PanicNode {
            fn name(&self) -> &str { "panic_node" }
            fn tick(&mut self) { panic!("intentional panic in chain 0"); }
        }

        let healthy_count = Arc::new(std::sync::atomic::AtomicU64::new(0));

        let panic_node = RegisteredNode {
            node: super::super::types::NodeKind::new(Box::new(PanicNode)),
            name: Arc::from("panic_node"),
            priority: 0,
            initialized: true,
            context: Some(crate::core::NodeInfo::new("panic_node".to_string())),
            rate_hz: None,
            last_tick: None,
            is_rt_node: true,
            tick_budget: None,
            deadline: None,
            recorder: None,
            is_stopped: false,
            is_paused: false,
            rt_stats: None,
            miss_policy: Miss::Warn,
            execution_class: super::super::types::ExecutionClass::Rt,
            health_state: super::super::types::AtomicHealthState::default(),
            os_priority: None,
            pinned_core: None,
            node_watchdog: None,
            failure_handler: None,
            budget_policy: super::super::safety_monitor::BudgetPolicy::default(),
        };

        let chain_0 = vec![panic_node];
        let chain_1 = vec![make_rt_registered("healthy", healthy_count.clone())];

        let running = Arc::new(AtomicBool::new(true));
        let executor = RtExecutor::start_pool(
            vec![chain_0, chain_1],
            running.clone(),
            1_u64.ms(),
            test_monitors(),
            Vec::new(),
        );

        std::thread::sleep(Duration::from_millis(50));
        running.store(false, Ordering::SeqCst);
        let _ = executor.stop();

        let healthy_ticks = healthy_count.load(std::sync::atomic::Ordering::Relaxed);
        assert!(
            healthy_ticks > 0,
            "healthy chain must tick despite panic in other chain: got {}",
            healthy_ticks
        );
    }

    #[test]
    fn test_multi_chain_three_chains_all_tick() {
        let counts: Vec<_> = (0..3)
            .map(|_| Arc::new(std::sync::atomic::AtomicU64::new(0)))
            .collect();

        let chains: Vec<Vec<RegisteredNode>> = counts
            .iter()
            .enumerate()
            .map(|(i, c)| vec![make_rt_registered(&format!("node_{}", i), c.clone())])
            .collect();

        let running = Arc::new(AtomicBool::new(true));
        let executor = RtExecutor::start_pool(
            chains,
            running.clone(),
            1_u64.ms(),
            test_monitors(),
            Vec::new(),
        );

        std::thread::sleep(Duration::from_millis(50));
        running.store(false, Ordering::SeqCst);
        let returned = executor.stop();

        for (i, c) in counts.iter().enumerate() {
            let ticks = c.load(std::sync::atomic::Ordering::Relaxed);
            assert!(ticks > 0, "chain {} must tick: got {}", i, ticks);
        }
        assert_eq!(returned.len(), 3);
    }

    // ========================================================================
    // Budget policy enforcement tests
    // ========================================================================

    struct SlowNode {
        name: String,
        count: Arc<std::sync::atomic::AtomicU64>,
        sleep_us: u64,
    }

    impl Node for SlowNode {
        fn name(&self) -> &str {
            &self.name
        }
        fn tick(&mut self) {
            self.count
                .fetch_add(1, std::sync::atomic::Ordering::Relaxed);
            std::thread::sleep(Duration::from_micros(self.sleep_us));
        }
    }

    fn make_slow_rt_node(
        name: &str,
        count: Arc<std::sync::atomic::AtomicU64>,
        sleep_us: u64,
        budget: Duration,
        policy: super::super::safety_monitor::BudgetPolicy,
    ) -> RegisteredNode {
        use crate::core::NodeInfo;

        let node = SlowNode {
            name: name.to_string(),
            count,
            sleep_us,
        };
        RegisteredNode {
            node: super::super::types::NodeKind::new(Box::new(node)),
            name: Arc::from(name),
            priority: 0,
            initialized: true,
            context: Some(NodeInfo::new(name.to_string())),
            rate_hz: Some(1000.0),
            last_tick: None,
            is_rt_node: true,
            tick_budget: Some(budget),
            deadline: None,
            recorder: None,
            is_stopped: false,
            is_paused: false,
            rt_stats: Some(crate::core::RtStats::default()),
            miss_policy: Miss::Warn,
            execution_class: super::super::types::ExecutionClass::Rt,
            health_state: super::super::types::AtomicHealthState::default(),
            os_priority: None,
            pinned_core: None,
            node_watchdog: None,
            failure_handler: None,
            budget_policy: policy,
        }
    }

    #[test]
    fn test_budget_policy_warn_does_not_stop_node() {
        use super::super::safety_monitor::BudgetPolicy;

        // Node sleeps 500μs with 100μs budget (5x over) but policy=Warn → keeps running
        let count = Arc::new(std::sync::atomic::AtomicU64::new(0));
        let node = make_slow_rt_node(
            "warn_node",
            count.clone(),
            500,
            Duration::from_micros(100),
            BudgetPolicy::Warn,
        );

        let running = Arc::new(AtomicBool::new(true));
        let executor = RtExecutor::start_pool(
            vec![vec![node]],
            running.clone(),
            1_u64.ms(),
            test_monitors(),
            Vec::new(),
        );

        std::thread::sleep(Duration::from_millis(50));
        running.store(false, Ordering::SeqCst);
        let returned = executor.stop();

        let ticks = count.load(std::sync::atomic::Ordering::Relaxed);
        // With Warn policy, node should tick multiple times (not stopped)
        assert!(
            ticks > 1,
            "Warn policy should NOT stop node — got {} ticks",
            ticks
        );
        // Node should still be alive (not stopped)
        assert!(
            !returned[0].is_stopped,
            "Warn policy should not set is_stopped"
        );
    }

    #[test]
    fn test_budget_policy_enforce_stops_node_on_2x_overrun() {
        use super::super::safety_monitor::BudgetPolicy;

        // Node sleeps 500μs with 100μs budget (5x > 2x threshold) and policy=Enforce → stopped
        let count = Arc::new(std::sync::atomic::AtomicU64::new(0));
        let node = make_slow_rt_node(
            "enforce_node",
            count.clone(),
            500,
            Duration::from_micros(100),
            BudgetPolicy::Enforce,
        );

        let running = Arc::new(AtomicBool::new(true));
        let executor = RtExecutor::start_pool(
            vec![vec![node]],
            running.clone(),
            1_u64.ms(),
            test_monitors(),
            Vec::new(),
        );

        std::thread::sleep(Duration::from_millis(50));
        running.store(false, Ordering::SeqCst);
        let returned = executor.stop();

        // Node should have been stopped after first tick (5x > 2x budget)
        assert!(
            returned[0].is_stopped,
            "Enforce policy should stop node that exceeds 2x budget"
        );
    }

    #[test]
    fn test_budget_policy_emergency_stop_halts_executor() {
        use super::super::safety_monitor::BudgetPolicy;

        // Node sleeps 500μs with 100μs budget and policy=EmergencyStop → halts everything
        let count = Arc::new(std::sync::atomic::AtomicU64::new(0));
        let node = make_slow_rt_node(
            "estop_node",
            count.clone(),
            500,
            Duration::from_micros(100),
            BudgetPolicy::EmergencyStop,
        );

        let running = Arc::new(AtomicBool::new(true));
        let executor = RtExecutor::start_pool(
            vec![vec![node]],
            running.clone(),
            1_u64.ms(),
            test_monitors(),
            Vec::new(),
        );

        // Give it enough time to detect and e-stop
        std::thread::sleep(Duration::from_millis(50));
        // running should be false (set by e-stop)
        assert!(
            !running.load(Ordering::SeqCst),
            "EmergencyStop policy should set running=false"
        );

        let returned = executor.stop();
        assert!(
            returned[0].is_stopped,
            "EmergencyStop policy should stop the node"
        );
    }

    // ========================================================================
    // Edge case tests
    // ========================================================================

    /// Empty executor (zero chains) — start_pool and stop complete without error.
    #[test]
    fn test_empty_executor_no_chains() {
        let running = Arc::new(AtomicBool::new(true));

        let executor = RtExecutor::start_pool(
            Vec::new(),
            running.clone(),
            1_u64.ms(),
            test_monitors(),
            Vec::new(),
        );

        running.store(false, Ordering::SeqCst);
        let returned = executor.stop();
        assert!(returned.is_empty(), "no chains → no nodes returned");
    }

    /// Single chain with a single empty chain (zero nodes in chain).
    #[test]
    fn test_single_empty_chain() {
        let running = Arc::new(AtomicBool::new(true));

        let executor = RtExecutor::start_pool(
            vec![Vec::new()],
            running.clone(),
            1_u64.ms(),
            test_monitors(),
            Vec::new(),
        );

        // Let the empty RT thread loop briefly
        std::thread::sleep(20_u64.ms());
        running.store(false, Ordering::SeqCst);
        let returned = executor.stop();
        assert!(returned.is_empty(), "empty chain → no nodes returned");
    }

    /// Single chain with exactly one node — verifies the degenerate-chain case.
    #[test]
    fn test_single_chain_single_node() {
        let count = Arc::new(std::sync::atomic::AtomicU64::new(0));
        let node = make_rt_registered("solo_node", count.clone());

        let running = Arc::new(AtomicBool::new(true));
        let executor = RtExecutor::start_pool(
            vec![vec![node]],
            running.clone(),
            1_u64.ms(),
            test_monitors(),
            Vec::new(),
        );

        std::thread::sleep(30_u64.ms());
        running.store(false, Ordering::SeqCst);
        let returned = executor.stop();

        assert_eq!(returned.len(), 1);
        assert_eq!(&*returned[0].name, "solo_node");
        assert!(
            count.load(std::sync::atomic::Ordering::Relaxed) > 0,
            "single node must tick at least once"
        );
    }

    /// Chain with many nodes (12) — all are ticked and returned.
    #[test]
    fn test_chain_with_many_nodes() {
        let counts: Vec<_> = (0..12)
            .map(|_| Arc::new(std::sync::atomic::AtomicU64::new(0)))
            .collect();

        let nodes: Vec<RegisteredNode> = counts
            .iter()
            .enumerate()
            .map(|(i, c)| make_rt_registered(&format!("node_{}", i), c.clone()))
            .collect();

        let running = Arc::new(AtomicBool::new(true));
        let executor = RtExecutor::start_pool(
            vec![nodes],
            running.clone(),
            1_u64.ms(),
            test_monitors(),
            Vec::new(),
        );

        std::thread::sleep(50_u64.ms());
        running.store(false, Ordering::SeqCst);
        let returned = executor.stop();

        assert_eq!(returned.len(), 12, "all 12 nodes returned");
        for (i, c) in counts.iter().enumerate() {
            let ticks = c.load(std::sync::atomic::Ordering::Relaxed);
            assert!(ticks > 0, "node_{} must tick at least once, got {}", i, ticks);
        }
    }

    /// Node with a sub-microsecond tick (empty body) — no issues with near-zero durations.
    struct NoOpNode(String);
    impl Node for NoOpNode {
        fn name(&self) -> &str {
            &self.0
        }
        fn tick(&mut self) {
            // Intentionally empty — sub-microsecond tick
        }
    }

    #[test]
    fn test_sub_microsecond_node_runs_without_issue() {
        use crate::core::NodeInfo;

        let node = NoOpNode("noop".to_string());
        let registered = RegisteredNode {
            node: super::super::types::NodeKind::new(Box::new(node)),
            name: Arc::from("noop"),
            priority: 0,
            initialized: true,
            context: Some(NodeInfo::new("noop".to_string())),
            rate_hz: None,
            last_tick: None,
            is_rt_node: true,
            tick_budget: None,
            deadline: None,
            recorder: None,
            is_stopped: false,
            is_paused: false,
            rt_stats: Some(crate::core::RtStats::default()),
            miss_policy: Miss::Warn,
            execution_class: super::super::types::ExecutionClass::Rt,
            health_state: super::super::types::AtomicHealthState::default(),
            os_priority: None,
            pinned_core: None,
            node_watchdog: None,
            failure_handler: None,
            budget_policy: super::super::safety_monitor::BudgetPolicy::default(),
        };

        let running = Arc::new(AtomicBool::new(true));
        let executor = RtExecutor::start_pool(
            vec![vec![registered]],
            running.clone(),
            1_u64.ms(),
            test_monitors(),
            Vec::new(),
        );

        std::thread::sleep(30_u64.ms());
        running.store(false, Ordering::SeqCst);
        let returned = executor.stop();

        assert_eq!(returned.len(), 1);
        // RtStats should have recorded ticks with non-NaN avg
        let stats = returned[0].rt_stats.as_ref().unwrap();
        assert!(
            stats.sampled_ticks() > 0,
            "sub-microsecond node should have recorded ticks"
        );
        assert!(
            stats.avg_execution_us().is_finite(),
            "avg_execution_us must be finite, not NaN/inf"
        );
    }

    /// Chain ordering: nodes are sorted by priority before execution.
    /// Nodes with lower priority values run first.
    #[test]
    fn test_chain_nodes_sorted_by_priority() {
        // Create nodes with intentionally reversed priorities to verify sorting.
        // After sorting, priority 0 runs before priority 10, which runs before priority 20.
        // We use a shared Vec to capture execution order on the first tick cycle.
        let order = Arc::new(Mutex::new(Vec::<String>::new()));

        struct OrderNode {
            name: String,
            order: Arc<Mutex<Vec<String>>>,
        }
        impl Node for OrderNode {
            fn name(&self) -> &str {
                &self.name
            }
            fn tick(&mut self) {
                let mut guard = self.order.lock().unwrap();
                // Only record during the first few ticks to keep deterministic
                if guard.len() < 30 {
                    guard.push(self.name.clone());
                }
            }
        }

        let make_order_node = |name: &str, priority: u32| -> RegisteredNode {
            use crate::core::NodeInfo;
            let node = OrderNode {
                name: name.to_string(),
                order: order.clone(),
            };
            RegisteredNode {
                node: super::super::types::NodeKind::new(Box::new(node)),
                name: Arc::from(name),
                priority,
                initialized: true,
                context: Some(NodeInfo::new(name.to_string())),
                rate_hz: None,
                last_tick: None,
                is_rt_node: true,
                tick_budget: None,
                deadline: None,
                recorder: None,
                is_stopped: false,
                is_paused: false,
                rt_stats: None,
                miss_policy: Miss::Warn,
                execution_class: super::super::types::ExecutionClass::Rt,
                health_state: super::super::types::AtomicHealthState::default(),
                os_priority: None,
                pinned_core: None,
                node_watchdog: None,
                failure_handler: None,
                budget_policy: super::super::safety_monitor::BudgetPolicy::default(),
            }
        };

        // Provide in REVERSE priority order — start_pool should sort them
        let nodes = vec![
            make_order_node("prio_20", 20),
            make_order_node("prio_0", 0),
            make_order_node("prio_10", 10),
        ];

        let running = Arc::new(AtomicBool::new(true));
        let executor = RtExecutor::start_pool(
            vec![nodes],
            running.clone(),
            1_u64.ms(),
            test_monitors(),
            Vec::new(),
        );

        std::thread::sleep(30_u64.ms());
        running.store(false, Ordering::SeqCst);
        let _returned = executor.stop();

        let recorded = order.lock().unwrap();
        // We need at least 3 entries (one full cycle) to check ordering
        assert!(
            recorded.len() >= 3,
            "need at least one full cycle of 3 nodes, got {}",
            recorded.len()
        );
        // Check that within each 3-node cycle, priority order is maintained
        // (prio_0, prio_10, prio_20) repeating
        for chunk in recorded.chunks_exact(3) {
            assert_eq!(chunk[0], "prio_0", "first in cycle should be prio_0");
            assert_eq!(chunk[1], "prio_10", "second in cycle should be prio_10");
            assert_eq!(chunk[2], "prio_20", "third in cycle should be prio_20");
        }
    }

    /// Multiple chains with multiple nodes each — all execute.
    #[test]
    fn test_multiple_chains_multiple_nodes_each() {
        let counts: Vec<_> = (0..6)
            .map(|_| Arc::new(std::sync::atomic::AtomicU64::new(0)))
            .collect();

        // 3 chains with 2 nodes each
        let chain_0 = vec![
            make_rt_registered("c0_n0", counts[0].clone()),
            make_rt_registered("c0_n1", counts[1].clone()),
        ];
        let chain_1 = vec![
            make_rt_registered("c1_n0", counts[2].clone()),
            make_rt_registered("c1_n1", counts[3].clone()),
        ];
        let chain_2 = vec![
            make_rt_registered("c2_n0", counts[4].clone()),
            make_rt_registered("c2_n1", counts[5].clone()),
        ];

        let running = Arc::new(AtomicBool::new(true));
        let executor = RtExecutor::start_pool(
            vec![chain_0, chain_1, chain_2],
            running.clone(),
            1_u64.ms(),
            test_monitors(),
            Vec::new(),
        );

        std::thread::sleep(50_u64.ms());
        running.store(false, Ordering::SeqCst);
        let returned = executor.stop();

        assert_eq!(returned.len(), 6, "all 6 nodes returned across 3 chains");
        for (i, c) in counts.iter().enumerate() {
            let ticks = c.load(std::sync::atomic::Ordering::Relaxed);
            assert!(ticks > 0, "node index {} must tick, got 0", i);
        }
    }

    /// RtStats are correctly populated after execution.
    #[test]
    fn test_rt_stats_populated_after_execution() {
        use crate::core::NodeInfo;

        let count = Arc::new(std::sync::atomic::AtomicU64::new(0));
        let node = CounterNode {
            name: "stats_node".to_string(),
            count: count.clone(),
        };

        let registered = RegisteredNode {
            node: super::super::types::NodeKind::new(Box::new(node)),
            name: Arc::from("stats_node"),
            priority: 0,
            initialized: true,
            context: Some(NodeInfo::new("stats_node".to_string())),
            rate_hz: None,
            last_tick: None,
            is_rt_node: true,
            tick_budget: None,
            deadline: None,
            recorder: None,
            is_stopped: false,
            is_paused: false,
            rt_stats: Some(crate::core::RtStats::default()),
            miss_policy: Miss::Warn,
            execution_class: super::super::types::ExecutionClass::Rt,
            health_state: super::super::types::AtomicHealthState::default(),
            os_priority: None,
            pinned_core: None,
            node_watchdog: None,
            failure_handler: None,
            budget_policy: super::super::safety_monitor::BudgetPolicy::default(),
        };

        let running = Arc::new(AtomicBool::new(true));
        let executor = RtExecutor::start_pool(
            vec![vec![registered]],
            running.clone(),
            1_u64.ms(),
            test_monitors(),
            Vec::new(),
        );

        std::thread::sleep(50_u64.ms());
        running.store(false, Ordering::SeqCst);
        let returned = executor.stop();

        let stats = returned[0].rt_stats.as_ref().unwrap();
        let tick_count = count.load(std::sync::atomic::Ordering::Relaxed);

        assert_eq!(
            stats.sampled_ticks(), tick_count,
            "sampled_ticks should match actual tick count"
        );
        assert!(stats.sampled_ticks() > 0, "must have recorded at least one tick");
        assert!(
            stats.worst_execution() >= stats.last_execution()
                || stats.sampled_ticks() == 1,
            "worst >= last (or only one sample)"
        );
        assert_eq!(stats.deadline_misses(), 0, "no deadline set → 0 misses");
        assert_eq!(stats.budget_violations(), 0, "no budget set → 0 violations");
        assert!(stats.avg_execution_us().is_finite(), "avg must be finite");
        assert!(stats.jitter_us().is_finite(), "jitter must be finite");
    }

    /// Stopped node is skipped — it never gets ticked.
    #[test]
    fn test_stopped_node_is_skipped() {
        let stopped_count = Arc::new(std::sync::atomic::AtomicU64::new(0));
        let healthy_count = Arc::new(std::sync::atomic::AtomicU64::new(0));

        let mut stopped_node = make_rt_registered("stopped", stopped_count.clone());
        stopped_node.is_stopped = true;

        let healthy_node = make_rt_registered("healthy", healthy_count.clone());

        let running = Arc::new(AtomicBool::new(true));
        let executor = RtExecutor::start_pool(
            vec![vec![stopped_node, healthy_node]],
            running.clone(),
            1_u64.ms(),
            test_monitors(),
            Vec::new(),
        );

        std::thread::sleep(50_u64.ms());
        running.store(false, Ordering::SeqCst);
        let returned = executor.stop();

        assert_eq!(returned.len(), 2);
        assert_eq!(
            stopped_count.load(std::sync::atomic::Ordering::Relaxed), 0,
            "stopped node must never tick"
        );
        assert!(
            healthy_count.load(std::sync::atomic::Ordering::Relaxed) > 0,
            "healthy sibling must still tick"
        );
    }

    /// Uninitialized node is skipped — it never gets ticked.
    #[test]
    fn test_uninitialized_node_is_skipped() {
        let uninit_count = Arc::new(std::sync::atomic::AtomicU64::new(0));
        let normal_count = Arc::new(std::sync::atomic::AtomicU64::new(0));

        let mut uninit_node = make_rt_registered("uninit", uninit_count.clone());
        uninit_node.initialized = false;

        let normal_node = make_rt_registered("normal", normal_count.clone());

        let running = Arc::new(AtomicBool::new(true));
        let executor = RtExecutor::start_pool(
            vec![vec![uninit_node, normal_node]],
            running.clone(),
            1_u64.ms(),
            test_monitors(),
            Vec::new(),
        );

        std::thread::sleep(50_u64.ms());
        running.store(false, Ordering::SeqCst);
        let returned = executor.stop();

        assert_eq!(returned.len(), 2);
        assert_eq!(
            uninit_count.load(std::sync::atomic::Ordering::Relaxed), 0,
            "uninitialized node must never tick"
        );
        assert!(
            normal_count.load(std::sync::atomic::Ordering::Relaxed) > 0,
            "initialized sibling must still tick"
        );
    }

    /// Paused node auto-unpauses after being skipped once.
    /// The RT loop sets is_paused=false and skips (continue), so the node
    /// misses exactly one tick cycle then resumes.
    #[test]
    fn test_paused_node_auto_unpauses() {
        let count = Arc::new(std::sync::atomic::AtomicU64::new(0));
        let mut node = make_rt_registered("paused_node", count.clone());
        node.is_paused = true;

        let running = Arc::new(AtomicBool::new(true));
        let executor = RtExecutor::start_pool(
            vec![vec![node]],
            running.clone(),
            1_u64.ms(),
            test_monitors(),
            Vec::new(),
        );

        // Let it run — first cycle skips (unpause), subsequent cycles tick normally
        std::thread::sleep(50_u64.ms());
        running.store(false, Ordering::SeqCst);
        let returned = executor.stop();

        assert_eq!(returned.len(), 1);
        assert!(
            !returned[0].is_paused,
            "node should be unpaused after running"
        );
        assert!(
            count.load(std::sync::atomic::Ordering::Relaxed) > 0,
            "paused node must resume ticking after auto-unpause"
        );
    }

    /// Fallback period is used when no node declares a rate.
    #[test]
    fn test_fallback_period_used_when_no_rate() {
        let count = Arc::new(std::sync::atomic::AtomicU64::new(0));
        // Node has rate_hz = None
        let node = make_rt_registered("no_rate", count.clone());

        let running = Arc::new(AtomicBool::new(true));
        // Fallback period of 10ms → ~100Hz effective tick rate
        let executor = RtExecutor::start_pool(
            vec![vec![node]],
            running.clone(),
            10_u64.ms(),
            test_monitors(),
            Vec::new(),
        );

        std::thread::sleep(100_u64.ms());
        running.store(false, Ordering::SeqCst);
        let returned = executor.stop();

        let ticks = count.load(std::sync::atomic::Ordering::Relaxed);
        // At 10ms fallback period over 100ms, expect roughly 10 ticks (wide margin)
        assert!(
            ticks >= 3 && ticks <= 50,
            "with 10ms fallback period over 100ms, expected 3-50 ticks, got {}",
            ticks
        );
        assert_eq!(returned.len(), 1);
    }

    /// Nodes without rt_stats=Some still tick without error (stats recording skipped).
    #[test]
    fn test_node_without_rt_stats() {
        let count = Arc::new(std::sync::atomic::AtomicU64::new(0));
        let mut node = make_rt_registered("no_stats", count.clone());
        // Explicitly set rt_stats to None (this is the default from make_rt_registered)
        node.rt_stats = None;

        let running = Arc::new(AtomicBool::new(true));
        let executor = RtExecutor::start_pool(
            vec![vec![node]],
            running.clone(),
            1_u64.ms(),
            test_monitors(),
            Vec::new(),
        );

        std::thread::sleep(30_u64.ms());
        running.store(false, Ordering::SeqCst);
        let returned = executor.stop();

        assert_eq!(returned.len(), 1);
        assert!(returned[0].rt_stats.is_none(), "rt_stats should remain None");
        assert!(
            count.load(std::sync::atomic::Ordering::Relaxed) > 0,
            "node without rt_stats must still tick"
        );
    }

    /// CPU affinity round-robin assignment across chains.
    /// Verifies the executor doesn't crash when rt_cpus are provided.
    #[test]
    fn test_cpu_affinity_with_multiple_chains() {
        let count_a = Arc::new(std::sync::atomic::AtomicU64::new(0));
        let count_b = Arc::new(std::sync::atomic::AtomicU64::new(0));

        let chain_0 = vec![make_rt_registered("cpu_node_0", count_a.clone())];
        let chain_1 = vec![make_rt_registered("cpu_node_1", count_b.clone())];

        let running = Arc::new(AtomicBool::new(true));
        // Provide CPU cores — may fail to pin (requires root/capabilities) but
        // must not crash. The executor logs and continues unpinned.
        let executor = RtExecutor::start_pool(
            vec![chain_0, chain_1],
            running.clone(),
            1_u64.ms(),
            test_monitors(),
            vec![0, 1],
        );

        std::thread::sleep(30_u64.ms());
        running.store(false, Ordering::SeqCst);
        let returned = executor.stop();

        assert_eq!(returned.len(), 2);
        assert!(count_a.load(std::sync::atomic::Ordering::Relaxed) > 0);
        assert!(count_b.load(std::sync::atomic::Ordering::Relaxed) > 0);
    }

    /// Immediate stop — running set false before any tick can occur.
    /// The executor must still return all nodes without hanging.
    #[test]
    fn test_immediate_stop_returns_all_nodes() {
        let counts: Vec<_> = (0..4)
            .map(|_| Arc::new(std::sync::atomic::AtomicU64::new(0)))
            .collect();

        let chain = counts
            .iter()
            .enumerate()
            .map(|(i, c)| make_rt_registered(&format!("imm_{}", i), c.clone()))
            .collect();

        let running = Arc::new(AtomicBool::new(false)); // Already false!

        let executor = RtExecutor::start_pool(
            vec![chain],
            running.clone(),
            1_u64.ms(),
            test_monitors(),
            Vec::new(),
        );

        let returned = executor.stop();
        assert_eq!(returned.len(), 4, "all nodes returned even with immediate stop");
    }

    /// Mixed stopped/uninitialized/paused/healthy in one chain — only healthy ones tick.
    #[test]
    fn test_mixed_node_states_in_chain() {
        let count_stopped = Arc::new(std::sync::atomic::AtomicU64::new(0));
        let count_uninit = Arc::new(std::sync::atomic::AtomicU64::new(0));
        let count_paused = Arc::new(std::sync::atomic::AtomicU64::new(0));
        let count_healthy = Arc::new(std::sync::atomic::AtomicU64::new(0));

        let mut stopped = make_rt_registered("stopped", count_stopped.clone());
        stopped.is_stopped = true;

        let mut uninit = make_rt_registered("uninit", count_uninit.clone());
        uninit.initialized = false;

        let mut paused = make_rt_registered("paused", count_paused.clone());
        paused.is_paused = true;

        let healthy = make_rt_registered("healthy", count_healthy.clone());

        let running = Arc::new(AtomicBool::new(true));
        let executor = RtExecutor::start_pool(
            vec![vec![stopped, uninit, paused, healthy]],
            running.clone(),
            1_u64.ms(),
            test_monitors(),
            Vec::new(),
        );

        std::thread::sleep(50_u64.ms());
        running.store(false, Ordering::SeqCst);
        let returned = executor.stop();

        assert_eq!(returned.len(), 4);
        assert_eq!(
            count_stopped.load(std::sync::atomic::Ordering::Relaxed), 0,
            "stopped node must not tick"
        );
        assert_eq!(
            count_uninit.load(std::sync::atomic::Ordering::Relaxed), 0,
            "uninitialized node must not tick"
        );
        // Paused node skips one cycle then resumes
        assert!(
            count_paused.load(std::sync::atomic::Ordering::Relaxed) > 0,
            "paused node must resume after auto-unpause"
        );
        assert!(
            count_healthy.load(std::sync::atomic::Ordering::Relaxed) > 0,
            "healthy node must tick normally"
        );
    }
}
