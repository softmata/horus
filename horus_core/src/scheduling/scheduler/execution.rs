//! Execution loop and tick methods for `Scheduler`.
//!
//! Contains the main loop (`run`, `run_for`, `run_with_filter`), single-tick
//! methods (`tick_once`, `execute_single_tick`), signal handling, control
//! commands, safety monitors, periodic monitoring, and tick timing.

use super::*;

// Re-import scheduling-level modules
use crate::scheduling::async_executor;
use crate::scheduling::blackbox;
use crate::scheduling::compute_executor;
use crate::scheduling::dependency_graph;
use crate::scheduling::event_executor;
use crate::scheduling::rt_executor;
use crate::scheduling::safety_monitor;
use crate::scheduling::types;

impl Scheduler {
    /// Tick specific nodes by name (runs continuously with the specified nodes)
    #[doc(hidden)]
    pub fn tick(&mut self, node_names: &[&str]) -> HorusResult<()> {
        // Use the same pattern as run() but with node filtering
        self.run_with_filter(Some(node_names), None)
    }

    /// Check if the scheduler is running
    pub fn is_running(&self) -> bool {
        self.running.load(Ordering::SeqCst)
    }

    /// Stop the scheduler
    pub fn stop(&self) {
        self.running.store(false, Ordering::SeqCst);
    }

    /// Get a clone of the running flag for external stop control.
    ///
    /// This allows external code (e.g., Python bindings) to stop the scheduler
    /// by setting the flag to `false` from outside the run loop.
    #[doc(hidden)]
    pub fn running_flag(&self) -> Arc<AtomicBool> {
        self.running.clone()
    }

    /// Apply deferred configuration and initialize all nodes.
    ///
    /// Called lazily on first `tick_once()` or at the start of `run()`.
    /// Idempotent — subsequent calls are no-ops.
    pub(super) fn finalize_and_init(&mut self) {
        if self.initialized {
            return;
        }
        self.finalize_config();
        self.adjust_tick_period_for_node_rates();

        // Build dependency graph for deterministic mode
        if self.pending_config.timing.deterministic_order {
            match dependency_graph::DependencyGraph::build(&self.nodes) {
                Ok(graph) => {
                    if graph.has_topic_metadata() {
                        print_line(&format!(
                            "Deterministic mode: {} execution steps from topic dependencies",
                            graph.step_count()
                        ));
                    } else {
                        print_line(&format!(
                            "Deterministic mode: {} execution steps from .order() tiers (no pub/sub metadata)",
                            graph.step_count()
                        ));
                    }
                    self.dependency_graph = Some(graph);
                }
                Err(e) => {
                    print_line(&format!("WARNING: Dependency graph error: {}. Falling back to sequential.", e));
                }
            }
        }

        self.initialize_filtered_nodes(None);

        // Create live node registry in SHM
        match super::super::registry::SchedulerRegistry::open(&self.scheduler_name) {
            Ok(reg) => {
                // Register all initialized nodes
                for registered in &self.nodes {
                    if registered.initialized {
                        let name = registered.name.as_ref();
                        let exec_class = match &registered.execution_class {
                            super::types::ExecutionClass::Rt => 0,
                            super::types::ExecutionClass::Compute => 1,
                            super::types::ExecutionClass::Event(_) => 2,
                            super::types::ExecutionClass::AsyncIo => 3,
                            super::types::ExecutionClass::BestEffort => 4,
                        };
                        let slot_idx = reg.register_node(
                            name,
                            registered.priority as u8,
                            registered.rate_hz.unwrap_or(0.0) as f64,
                            exec_class,
                        );
                        self.registry_slots.insert(name.to_string(), slot_idx);
                    }
                }
                self.registry = Some(reg);
            }
            Err(e) => {
                print_line(&format!(
                    "Warning: Failed to create scheduler registry: {}",
                    e
                ));
            }
        }

        // Create control topic for CLI-to-scheduler commands
        let ctl_name = format!("horus.ctl.{}", self.scheduler_name);
        match crate::communication::Topic::<super::super::control::ControlCommand>::new_with_kind(
            &ctl_name,
            crate::communication::TopicKind::System as u8,
        ) {
            Ok(topic) => {
                self.control_topic = Some(topic);
            }
            Err(e) => {
                print_line(&format!(
                    "Warning: Failed to create control topic: {}",
                    e
                ));
            }
        }

        self.initialized = true;
    }

    /// Execute exactly one tick cycle for all nodes, then return.
    ///
    /// Does **not** loop. Does **not** sleep. The caller controls timing.
    /// Lazily initializes on first call (applies deferred config, inits nodes).
    ///
    /// Designed for simulation and testing:
    /// ```rust,ignore
    /// let mut scheduler = Scheduler::new()
    ///     .tick_rate(100_u64.hz());
    ///
    /// scheduler.add(MyNode::new()).build()?;
    ///
    /// // Simulation loop
    /// loop {
    ///     sim.step_physics(dt);
    ///     scheduler.tick_once()?;
    ///     sim.render();
    /// }
    /// ```
    pub fn tick_once(&mut self) -> HorusResult<()> {
        self.finalize_and_init();
        self.execute_single_tick(None)
    }

    /// Execute exactly `n` ticks, then return.
    ///
    /// Equivalent to calling `tick_once()` n times. Initializes nodes lazily
    /// on first tick.
    ///
    /// # Example
    /// ```rust,ignore
    /// use horus::prelude::*;
    ///
    /// let mut scheduler = Scheduler::new();
    /// scheduler.add(MyController::new()).rate(100.hz()).build();
    /// scheduler.run_ticks(10)?;
    /// ```
    pub fn run_ticks(&mut self, n: u64) -> HorusResult<()> {
        for _ in 0..n {
            self.tick_once()?;
        }
        Ok(())
    }

    /// Run until `predicate` returns true, or `max_ticks` is reached.
    ///
    /// Returns `Ok(true)` if the predicate was satisfied, `Ok(false)` if
    /// max_ticks was hit without the predicate becoming true.
    /// The predicate is checked AFTER each tick.
    ///
    /// # Example
    /// ```rust,ignore
    /// let reached = scheduler.run_until(
    ///     || position_topic.recv().map_or(false, |p| p.x > 5.0),
    ///     10_000,
    /// )?;
    /// assert!(reached, "robot should reach target within 10k ticks");
    /// ```
    pub fn run_until<F>(&mut self, mut predicate: F, max_ticks: u64) -> HorusResult<bool>
    where
        F: FnMut() -> bool,
    {
        for _ in 0..max_ticks {
            self.tick_once()?;
            if predicate() {
                return Ok(true);
            }
        }
        Ok(false)
    }

    /// Execute exactly one tick cycle for specific named nodes only.
    ///
    /// Internal: run one tick cycle. No loop, no sleep.
    pub(super) fn execute_single_tick(&mut self, node_filter: Option<&[&str]>) -> HorusResult<()> {
        self.process_control_commands();
        self.tick.last_instant = Instant::now();
        self.reinit_pending_nodes();

        if let Some(ref graph) = self.dependency_graph {
            // Deterministic mode: execute by dependency steps.
            // Each step contains independent nodes (no data dependencies between them).
            // Steps execute sequentially (barrier between steps ensures producer-before-consumer).
            //
            // Within a step, nodes execute sequentially on the main thread in tick_once() mode.
            // In run() mode, the RT executor pool dispatches independent nodes to separate
            // threads for true parallelism. tick_once() prioritizes reproducibility over
            // wall-time performance — simulation/testing don't need real-time speed.
            let steps: Vec<Vec<usize>> = graph.steps().to_vec();
            for step in &steps {
                for &node_idx in step {
                    if self.execute_single_node(node_idx, node_filter) {
                        return Err(crate::horus_internal!(
                            "Fatal node failure during deterministic tick"
                        ));
                    }
                }
                // Advance SimClock by dt after each step (barrier point)
                if self.pending_config.timing.deterministic_order {
                    let dt = self.tick.period;
                    self.clock.advance(dt);
                }
            }
        } else {
            // Normal mode: sequential by priority (existing behavior)
            self.nodes.sort_by_key(|r| r.priority);
            let num_nodes = self.nodes.len();
            for i in 0..num_nodes {
                if self.execute_single_node(i, node_filter) {
                    return Err(crate::horus_internal!(
                        "Fatal node failure during tick_once"
                    ));
                }
            }
        }

        if self.check_safety_monitors() {
            return Err(crate::horus_internal!(
                "Emergency stop triggered during tick_once"
            ));
        }

        // Record execution order for replay fidelity
        if let Some(ref mut recording_state) = self.recording {
            let order: Vec<String> = if let Some(ref graph) = self.dependency_graph {
                // Deterministic: record dependency step order
                graph.steps().iter().flatten().map(|&idx| {
                    self.nodes.get(idx)
                        .map(|n| n.name.to_string())
                        .unwrap_or_default()
                }).collect()
            } else {
                // Normal: record priority-sorted order
                self.nodes.iter().map(|n| n.name.to_string()).collect()
            };
            recording_state.scheduler_recording.execution_order.push(order);
        }

        self.tick.current += 1;
        Ok(())
    }

    /// Main loop with automatic signal handling and cleanup.
    pub fn run(&mut self) -> HorusResult<()> {
        self.run_with_filter(None, None)
    }

    /// Run all nodes for a specified duration, then shutdown gracefully.
    pub fn run_for(&mut self, duration: Duration) -> HorusResult<()> {
        self.run_with_filter(None, Some(duration))
    }

    /// Run specific nodes for a specified duration, then shutdown gracefully.
    #[doc(hidden)]
    pub fn tick_for(&mut self, node_names: &[&str], duration: Duration) -> HorusResult<()> {
        self.run_with_filter(Some(node_names), Some(duration))
    }

    /// Internal method to run scheduler with optional node filtering and duration.
    pub(super) fn run_with_filter(
        &mut self,
        node_filter: Option<&[&str]>,
        duration: Option<Duration>,
    ) -> HorusResult<()> {
        // Use a single-threaded tokio runtime for the scheduler main loop.
        // The main loop only needs timers (tokio::time::sleep). Heavy async I/O
        // nodes run on their own dedicated runtime (see AsyncExecutor).
        let rt = tokio::runtime::Builder::new_current_thread()
            .enable_time()
            .build()
            .horus_context("creating scheduler tokio runtime")?;

        rt.block_on(async {
            let start_time = Instant::now();

            self.finalize_config();
            self.initialized = true;
            self.install_panic_hook();
            self.setup_signal_handlers();
            self.initialize_filtered_nodes(node_filter);
            Self::setup_control_directory();
            self.update_registry();
            self.adjust_tick_period_for_node_rates();

            let deterministic = self.pending_config.timing.deterministic_order;

            // In deterministic mode: all nodes stay on the main thread,
            // no executor threads are spawned. This guarantees identical
            // execution order across runs.
            let mut rt_executor = None;
            let mut compute_executor = None;
            let mut event_executor = None;
            let mut async_executor = None;

            if deterministic {
                print_line("Deterministic mode: all nodes execute sequentially on main thread");
                // All nodes stay in self.nodes — no grouping, no executors
            } else {
                // Group nodes by ExecutionClass and dispatch to dedicated executors:
                // - RT nodes → RtExecutor (dedicated high-priority thread)
                // - Compute nodes → ComputeExecutor (parallel thread pool)
                // - Event nodes → EventExecutor (per-node watcher threads)
                // - AsyncIo nodes → AsyncExecutor (tokio blocking pool)
                // - BestEffort nodes → stay in self.nodes for main-thread sequential execution
                let all_nodes = std::mem::take(&mut self.nodes);
                let groups = types::group_nodes_by_class(all_nodes);

                // BestEffort nodes remain on the main thread
                self.nodes = groups.main_nodes;

                // Shared monitors for all executor threads
                let shared_monitors = types::SharedMonitors {
                    profiler: self.monitor.profiler.clone(),
                    blackbox: self.monitor.blackbox.clone(),
                    safety: self.monitor.safety.clone(),
                    verbose: self.pending_config.monitoring.verbose,
                };

                if !groups.rt_nodes.is_empty() {
                    print_line(&format!(
                        "Starting RT executor with {} RT nodes on dedicated thread",
                        groups.rt_nodes.len()
                    ));
                    let rt_cpus = if let Some(ref cores) = self.pending_config.resources.cpu_cores {
                        cores.clone()
                    } else if let Some(ref caps) = self.rt.capabilities {
                        if !caps.recommended_rt_cpus.is_empty() {
                            caps.recommended_rt_cpus.clone()
                        } else if caps.cpu_count > 1 {
                            vec![caps.cpu_count - 1]
                        } else {
                            Vec::new()
                        }
                    } else {
                        Vec::new()
                    };

                    // Split RT nodes into isolated threads for safety.
                    // Default: one thread per node to prevent stalled nodes from
                    // blocking siblings. Dependency graph can group dependent nodes.
                    let rt_chains = if let Some(ref graph) = self.dependency_graph {
                        let rt_indices: Vec<usize> = (0..groups.rt_nodes.len()).collect();
                        let chain_groups = graph.independent_chains(&rt_indices);
                        if chain_groups.len() > 1 {
                            // Distribute nodes into chain groups
                            let mut chains: Vec<Vec<types::RegisteredNode>> =
                                chain_groups.iter().map(|_| Vec::new()).collect();
                            let mut rt_nodes = groups.rt_nodes;
                            rt_nodes.reverse();
                            for (chain_idx, group) in chain_groups.iter().enumerate() {
                                for _ in group {
                                    if let Some(node) = rt_nodes.pop() {
                                        chains[chain_idx].push(node);
                                    }
                                }
                            }
                            chains
                        } else {
                            groups.rt_nodes.into_iter().map(|n| vec![n]).collect()
                        }
                    } else {
                        // No dependency graph — isolate each RT node on its own thread
                        groups.rt_nodes.into_iter().map(|n| vec![n]).collect()
                    };

                    rt_executor = Some(rt_executor::RtExecutor::start_pool(
                        rt_chains,
                        self.running.clone(),
                        self.tick.period,
                        shared_monitors.clone(),
                        rt_cpus,
                    ));
                }

                if !groups.compute_nodes.is_empty() {
                    print_line(&format!(
                        "Starting compute executor with {} nodes on thread pool",
                        groups.compute_nodes.len()
                    ));
                    compute_executor = Some(compute_executor::ComputeExecutor::start(
                        groups.compute_nodes,
                        self.running.clone(),
                        self.tick.period,
                        shared_monitors.clone(),
                    ));
                }

                if !groups.event_nodes.is_empty() {
                    print_line(&format!(
                        "Starting event executor with {} event-triggered nodes",
                        groups.event_nodes.len()
                    ));
                    event_executor = Some(event_executor::EventExecutor::start(
                        groups.event_nodes,
                        self.running.clone(),
                        shared_monitors.clone(),
                    ));
                }

                if !groups.async_io_nodes.is_empty() {
                    print_line(&format!(
                        "Starting async I/O executor with {} nodes on tokio pool",
                        groups.async_io_nodes.len()
                    ));
                    async_executor = Some(async_executor::AsyncExecutor::start(
                        groups.async_io_nodes,
                        self.running.clone(),
                        self.tick.period,
                        shared_monitors.clone(),
                    ));
                }
            }

            // Main tick loop
            while self.is_running() {
                if self.should_stop_loop(start_time, duration) {
                    break;
                }
                self.process_control_commands();
                self.tick.last_instant = Instant::now();
                self.reinit_pending_nodes();
                self.execute_nodes(node_filter).await;
                if self.check_safety_monitors() {
                    break;
                }
                self.periodic_monitoring(start_time);
                if let Some(sleep_dur) = self.compute_tick_sleep() {
                    tokio::time::sleep(sleep_dur).await;
                }
                self.advance_tick();
            }

            // Stop executors and reclaim nodes for shutdown
            self.running.store(false, Ordering::SeqCst);
            if let Some(executor) = rt_executor {
                let rt_nodes = executor.stop();
                self.nodes.extend(rt_nodes);
            }
            if let Some(executor) = compute_executor {
                let compute_nodes = executor.stop();
                self.nodes.extend(compute_nodes);
            }
            if let Some(executor) = event_executor {
                let event_nodes = executor.stop();
                self.nodes.extend(event_nodes);
            }
            if let Some(executor) = async_executor {
                let async_nodes = executor.stop();
                self.nodes.extend(async_nodes);
            }

            self.shutdown_filtered_nodes(node_filter);
            self.finalize_run();
        });

        Ok(())
    }

    /// Set up Ctrl+C and SIGTERM signal handlers for graceful shutdown.
    /// Install a global panic hook for forensic logging.
    ///
    /// Chains with the previous hook and adds:
    /// - Blackbox flush to disk (if configured)
    /// - Crash report written to `<temp_dir>/horus_crash_<pid>.log`
    fn install_panic_hook(&self) {
        let blackbox = self.monitor.blackbox.clone();
        let scheduler_name = self.scheduler_name.clone();
        let prev_hook = std::panic::take_hook();

        std::panic::set_hook(Box::new(move |info| {
            // 1. Flush blackbox to disk
            if let Some(ref bb) = blackbox {
                if let Ok(mut bb) = bb.try_lock() {
                    bb.flush_wal();
                }
            }

            // 2. Build crash report
            let thread = std::thread::current();
            let thread_name = thread.name().unwrap_or("<unnamed>");
            let pid = std::process::id();
            let location = if let Some(loc) = info.location() {
                format!("{}:{}:{}", loc.file(), loc.line(), loc.column())
            } else {
                "unknown location".to_string()
            };
            let payload = if let Some(s) = info.payload().downcast_ref::<&str>() {
                s.to_string()
            } else if let Some(s) = info.payload().downcast_ref::<String>() {
                s.clone()
            } else {
                "unknown panic payload".to_string()
            };

            let report = format!(
                "=== HORUS Crash Report ===\n\
                 Scheduler: {}\n\
                 PID: {}\n\
                 Thread: {}\n\
                 Location: {}\n\
                 Panic: {}\n\
                 Blackbox flushed: {}\n\
                 ===========================\n",
                scheduler_name,
                pid,
                thread_name,
                location,
                payload,
                blackbox.is_some(),
            );

            // 3. Write crash report to /tmp (best-effort, no allocation beyond the format above)
            let crash_path = std::env::temp_dir().join(format!("horus_crash_{}.log", pid));
            let _ = std::fs::write(&crash_path, &report);

            // 4. Also print to stderr (visible in logs)
            eprintln!("{}", report);

            // 5. Chain to previous hook
            prev_hook(info);
        }));
    }

    fn setup_signal_handlers(&self) {
        let running = self.running.clone();
        if let Err(e) = ctrlc::set_handler(move || {
            print_line("\nCtrl+C received! Shutting down HORUS scheduler...");
            running.store(false, Ordering::SeqCst);
            std::thread::spawn(|| {
                std::thread::sleep(2_u64.secs());
                print_line("Force terminating - cleaning up session...");
                // Clean up session before forced exit to prevent stale files
                Self::cleanup_session();
                std::process::exit(0);
            });
        }) {
            print_line(&format!("Warning: Failed to set signal handler: {}", e));
        }

        // Set up SIGTERM handler for graceful termination (e.g., from `kill` or `timeout`)
        horus_sys::process::on_terminate(sigterm_handler);
    }

    /// Check if the main loop should stop (duration limit, replay stop tick, or SIGTERM).
    fn should_stop_loop(&self, start_time: Instant, duration: Option<Duration>) -> bool {
        if let Some(max_duration) = duration {
            if start_time.elapsed() >= max_duration {
                print_line(&format!(
                    "Scheduler reached time limit of {:?}",
                    max_duration
                ));
                return true;
            }
        }

        if let Some(stop_tick) = self.replay.as_ref().and_then(|r| r.stop_tick) {
            if self.tick.current >= stop_tick {
                print_line(&format!("[REPLAY] Reached stop tick {}", stop_tick));
                return true;
            }
        }

        if SIGTERM_RECEIVED.load(Ordering::SeqCst) {
            print_line("\nSIGTERM received! Shutting down HORUS scheduler...");
            return true;
        }

        false
    }

    /// Check watchdogs with graduated severity and update node health states.
    ///
    /// Uses graduated watchdog to transition nodes through health states:
    /// - Warning (1x timeout): log, node stays Healthy→Warning
    /// - Expired (2x timeout): mark Unhealthy, skip in tick loop
    /// - Critical (3x timeout): mark Isolated, call `enter_safe_state()` for critical nodes
    ///
    /// Returns `true` if the scheduler should stop (emergency stop triggered).
    pub(super) fn check_safety_monitors(&mut self) -> bool {
        if let Some(ref monitor) = self.monitor.safety {
            // Use graduated check for per-node health state transitions
            monitor
                .check_watchdogs_graduated(&mut self.monitor.watchdog_graduated_buf);

            // Apply health state transitions based on severity
            for (node_name, severity) in &self.monitor.watchdog_graduated_buf {
                use safety_monitor::WatchdogSeverity;
                use types::NodeHealthState;

                if let Some(registered) = self.nodes.iter_mut().find(|n| n.name.as_ref() == node_name) {
                    let current = registered.health_state.load();
                    match severity {
                        WatchdogSeverity::Ok => {
                            // Should not appear in results, but handle gracefully
                        }
                        WatchdogSeverity::Warning => {
                            if current == NodeHealthState::Healthy {
                                registered.health_state.store(NodeHealthState::Warning);
                                print_line(&format!(
                                    " Watchdog warning: '{}' (1x timeout) — marking Warning",
                                    node_name
                                ));
                            }
                        }
                        WatchdogSeverity::Expired => {
                            if current != NodeHealthState::Unhealthy
                                && current != NodeHealthState::Isolated
                            {
                                registered.health_state.store(NodeHealthState::Unhealthy);
                                print_line(&format!(
                                    " Watchdog expired: '{}' (2x timeout) — marking Unhealthy, skipping",
                                    node_name
                                ));
                            }
                        }
                        WatchdogSeverity::Critical => {
                            if current != NodeHealthState::Isolated {
                                registered.health_state.store(NodeHealthState::Isolated);
                                registered.node.enter_safe_state();
                                print_line(&format!(
                                    " Watchdog critical: '{}' (3x timeout) — Isolated, entered safe state",
                                    node_name
                                ));
                            }
                        }
                    }
                }
            }

            // Also run the classic expired check for backward compat logging
            monitor.check_watchdogs(&mut self.monitor.watchdog_expired_buf);

            if monitor.is_emergency_stop() {
                print_line(" Emergency stop activated - shutting down scheduler");
                if let Some(ref bb) = self.monitor.blackbox {
                    bb.lock()
                        .unwrap()
                        .record(blackbox::BlackBoxEvent::EmergencyStop {
                            reason: "Safety monitor triggered emergency stop".to_string(),
                        });
                }

                return true;
            }
        }
        false
    }

    /// Periodic registry snapshot, failure logging, blackbox tick, and telemetry export.
    fn periodic_monitoring(&mut self, start_time: Instant) {
        // Registry snapshot every 5 seconds
        if self.monitor.last_snapshot.elapsed() >= 5_u64.secs() {
            self.snapshot_state_to_registry();
            self.monitor.last_snapshot = Instant::now();

            // Log stopped nodes
            let stopped_count = self.nodes.iter().filter(|n| n.is_stopped).count();
            if stopped_count > 0 {
                if let Some(ref bb) = self.monitor.blackbox {
                    bb.lock()
                        .unwrap()
                        .record(blackbox::BlackBoxEvent::Custom {
                            category: "safety".to_string(),
                            message: format!("{} nodes stopped", stopped_count),
                        });
                }
            }
        }

        // === Runtime feature integrations ===

        // Black box tick increment
        if let Some(ref bb) = self.monitor.blackbox {
            bb.lock().unwrap().tick();
        }

        // Telemetry export (if interval elapsed)
        if let Some(ref mut tm) = self.monitor.telemetry {
            if tm.should_export() {
                let profiler = self.monitor.profiler.lock().unwrap();
                let total_ticks = profiler
                    .node_stats
                    .values()
                    .map(|s| s.count)
                    .max()
                    .unwrap_or(0) as u64;
                tm.counter("scheduler_ticks", total_ticks);
                tm.gauge("scheduler_uptime_secs", start_time.elapsed().as_secs_f64());
                tm.gauge("nodes_active", self.nodes.len() as f64);

                for registered in &self.nodes {
                    let node_name = registered.name.as_ref();
                    if let Some(stats) = profiler.node_stats.get(node_name) {
                        let mut labels = std::collections::HashMap::new();
                        labels.insert("node".to_string(), node_name.to_string());
                        tm.gauge_with_labels("node_avg_duration_us", stats.avg_us, labels.clone());
                        tm.counter_with_labels("node_tick_count", stats.count as u64, labels);
                    }
                }
                drop(profiler);

                let _ = tm.export();
            }
        }
    }

    /// Compute the tick sleep duration.
    fn compute_tick_sleep(&self) -> Option<Duration> {
        let sleep_duration = if let Some(ref replay) = self.replay {
            if replay.speed != 1.0 {
                Duration::from_nanos((self.tick.period.as_nanos() as f64 / replay.speed) as u64)
            } else {
                self.tick.period
            }
        } else {
            self.tick.period
        };

        Some(sleep_duration)
    }

    /// Increment tick counter.
    fn advance_tick(&mut self) {
        self.tick.current += 1;
    }

    /// Drain the control topic and process commands (pause/resume/shutdown).
    ///
    /// Called at the START of each tick — deterministic (commands take effect
    /// at tick boundaries, not mid-tick).
    pub(super) fn process_control_commands(&mut self) {
        use super::super::control::ControlCommand;

        let Some(ref topic) = self.control_topic else {
            return;
        };

        // Drain all pending commands
        let mut commands = Vec::new();
        while let Some(cmd) = topic.recv() {
            commands.push(cmd);
        }

        for cmd in commands {
            match cmd {
                ControlCommand::PauseNode { ref name } => {
                    if let Some(node) = self.nodes.iter_mut().find(|n| n.name.as_ref() == name.as_str()) {
                        node.is_paused = true;
                        print_line(&format!("[CONTROL] Paused node '{}'", name));
                    } else {
                        print_line(&format!("[CONTROL] Node '{}' not found", name));
                    }
                }
                ControlCommand::ResumeNode { ref name } => {
                    if let Some(node) = self.nodes.iter_mut().find(|n| n.name.as_ref() == name.as_str()) {
                        node.is_paused = false;
                        print_line(&format!("[CONTROL] Resumed node '{}'", name));
                    } else {
                        print_line(&format!("[CONTROL] Node '{}' not found", name));
                    }
                }
                ControlCommand::SetNodeRate { ref name, rate_hz } => {
                    if let Some(node) = self.nodes.iter_mut().find(|n| n.name.as_ref() == name.as_str()) {
                        node.rate_hz = Some(rate_hz);
                        print_line(&format!("[CONTROL] Set node '{}' rate to {} Hz", name, rate_hz));
                    }
                }
                ControlCommand::GracefulShutdown => {
                    self.running.store(false, Ordering::SeqCst);
                    print_line("[CONTROL] Graceful shutdown requested");
                }
                ControlCommand::RequestMetricsFlush => {
                    // Registry updates per-tick — no-op
                }
            }
        }
    }

    /// Setup control directory — no-op, replaced by control topic.
    pub(super) fn setup_control_directory() {
        // Control commands now flow via horus.ctl.{scheduler} topic.
    }

    /// Clean up session directory (no-op with flat namespace)
    ///
    /// With the simplified flat namespace model, topics are shared globally
    /// and should be cleaned up manually via `horus clean` command.
    pub(super) fn cleanup_session() {
        // Clean up this process's SHM namespace on exit.
        // On normal exit, the namespace persists until the process group dies.
        // On signal-triggered exit, we proactively clean stale namespaces
        // so a restart doesn't see orphaned SHM files.
        let cleanup = crate::memory::platform::cleanup_stale_namespaces();
        if cleanup.removed > 0 {
            log::info!(
                "Session cleanup: removed {} stale SHM namespace(s), freed {} bytes",
                cleanup.removed,
                cleanup.bytes_freed
            );
        }
    }

    /// Execute main-thread (BestEffort) nodes sequentially in priority order.
    ///
    /// RT, Compute, and Event nodes are handled by their dedicated executors.
    /// Only BestEffort nodes remain in `self.nodes` for main-thread execution.
    async fn execute_nodes(&mut self, node_filter: Option<&[&str]>) {
        // Sort by priority
        self.nodes.sort_by_key(|r| r.priority);

        let num_nodes = self.nodes.len();
        for i in 0..num_nodes {
            if self.execute_single_node(i, node_filter) {
                return; // Fatal failure — scheduler stopping
            }
        }
    }
}
