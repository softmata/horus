//! Node lifecycle and execution methods for `Scheduler`.
//!
//! Contains node registration (`add`, `add_configured`), node execution
//! (`execute_single_node`, `process_tick_result`), timing violations,
//! degradation actions, failure handling, initialization/shutdown, and
//! registry/metrics methods.

use super::*;

// Re-import scheduling-level modules
use crate::scheduling::blackbox;
use crate::scheduling::fault_tolerance;
use crate::scheduling::node_builder;
use crate::scheduling::primitives;
use crate::scheduling::safety_monitor;
use crate::scheduling::types;

impl Scheduler {
    // ========================================================================
    // NODE BUILDER API (Fluent Interface)
    // ========================================================================

    /// Add a node using the fluent builder API.
    ///
    /// This is the **recommended** way to add nodes. No `Box::new()` required!
    /// Call `.build()` on the builder to register the node.
    ///
    /// # Example
    /// ```rust,ignore
    /// use horus_core::Scheduler;
    ///
    /// let mut scheduler = Scheduler::new();
    ///
    /// // Simple - just node and order
    /// scheduler.add(MyNode::new())
    ///     .order(0)
    ///     .build();
    ///
    /// // RT node — auto-detected from .rate()
    /// scheduler.add(MotorController::new())
    ///     .order(0)
    ///     .rate(1000_u64.hz())  // 1kHz → auto-derives budget & deadline
    ///     .build()?;
    ///
    /// // Chain multiple nodes
    /// scheduler.add(SensorNode::new()).order(0).rate(100_u64.hz()).build()?;
    /// scheduler.add(ControlNode::new()).order(1).rate(500_u64.hz()).build()?;
    /// scheduler.add(LoggerNode::new()).order(100).build()?;
    /// ```
    pub fn add<N: Node + 'static>(&mut self, node: N) -> node_builder::NodeBuilder<'_> {
        node_builder::NodeBuilder::new(self, Box::new(node))
    }

    /// Add a node using a pre-built NodeRegistration.
    ///
    /// This is called internally by `NodeBuilder::done()`. You can also use it
    /// directly with a `NodeRegistration`.
    ///
    /// # Example
    /// ```rust,ignore
    /// use horus_core::scheduling::NodeRegistration;
    ///
    /// let config = NodeRegistration::new(Box::new(my_node))
    ///     .order(0)
    ///     .rate(1000_u64.hz());
    ///
    /// scheduler.add_configured(config);
    /// ```
    #[doc(hidden)]
    pub fn add_configured(&mut self, config: node_builder::NodeRegistration) -> &mut Self {
        self.add_configured_internal(config)
    }

    /// Internal method to add a fully configured node.
    fn add_configured_internal(&mut self, config: node_builder::NodeRegistration) -> &mut Self {
        let node = config.node;
        let priority = config.order;
        let custom_rate = config.rate_hz;
        let is_rt_node = config.is_rt;
        let tick_budget = config.tick_budget;
        let deadline = config.deadline;
        let miss_policy = config.miss_policy;
        let execution_class = config.execution_class;
        let failure_handler = config
            .failure_policy
            .map(fault_tolerance::FailureHandler::new);

        let node_name = node.name().to_string();

        let context = NodeInfo::new(node_name.clone());

        // Create node recorder if recording is enabled
        let recorder = if let Some(ref mut rec_state) = self.recording {
            let node_id = format!(
                "{:x}{:x}",
                std::time::SystemTime::now()
                    .duration_since(std::time::UNIX_EPOCH)
                    .map(|d| d.as_nanos() as u64)
                    .unwrap_or(0),
                self.nodes.len() as u64
            );
            let recorder = NodeRecorder::new(&node_name, &node_id, rec_state.config.clone());

            let relative_path = format!("{}@{}.horus", node_name, node_id);
            rec_state
                .scheduler_recording
                .add_node_recording(&node_id, &relative_path);

            Some(recorder)
        } else {
            None
        };

        // Rate comes from builder config only (Node trait no longer declares rate)
        let node_rate = custom_rate;

        // Allocate RtStats for RT nodes
        let rt_stats = if is_rt_node {
            Some(crate::core::RtStats::default())
        } else {
            None
        };

        self.nodes.push(RegisteredNode {
            node,
            name: Arc::from(node_name.as_str()),
            priority,
            initialized: false,
            context: Some(context),
            rate_hz: node_rate,
            last_tick: if node_rate.is_some() {
                Some(Instant::now())
            } else {
                None
            },
            is_rt_node,
            tick_budget,
            deadline,
            recorder,
            is_stopped: false,
            is_paused: false,
            rt_stats,
            miss_policy,
            execution_class,
            health_state: crate::scheduling::types::AtomicHealthState::default(),
            os_priority: config.os_priority,
            pinned_core: config.pinned_core,
            node_watchdog: config.node_watchdog,
            failure_handler,
        });

        if let Some(rate) = node_rate {
            print_line(&format!(
                "Added {} '{}' with priority {} at {:.1}Hz",
                if is_rt_node { "RT node" } else { "node" },
                node_name,
                priority,
                rate
            ));
            // Ensure tick_period is fast enough for this node's rate
            self.adjust_tick_period_for_node_rates();
        } else {
            print_line(&format!(
                "Added {} '{}' with priority {}",
                if is_rt_node { "RT node" } else { "node" },
                node_name,
                priority
            ));
        }

        self
    }

    /// Add a critical node to the safety monitor with a watchdog timeout.
    ///
    /// Critical nodes are monitored more strictly:
    /// - Watchdog expiration triggers emergency stop
    /// - budget violations trigger emergency stop
    /// - Deadline misses trigger emergency stop
    ///
    /// # Example
    /// ```rust,ignore
    /// use horus_core::Scheduler;
    /// use std::time::Duration;
    ///
    /// let mut scheduler = Scheduler::new().watchdog(500_u64.ms());
    /// // ... after run() starts, safety monitor is active
    /// ```
    ///
    /// # Errors
    /// Returns an error if the safety monitor is not enabled.
    #[doc(hidden)]
    pub fn add_critical_node(
        &mut self,
        node_name: &str,
        watchdog_timeout: std::time::Duration,
    ) -> crate::error::HorusResult<&mut Self> {
        if let Some(ref monitor) = self.monitor.safety {
            monitor.add_critical_node(node_name.to_string(), watchdog_timeout);
            Ok(self)
        } else {
            Err(crate::error::HorusError::Config(
                crate::error::ConfigError::Other(
                    "Safety monitor not enabled. Use Scheduler::new().watchdog(500.ms()) to enable."
                        .to_string(),
                ),
            ))
        }
    }

    /// Set the budget (Worst-Case Execution Time) budget for a node.
    ///
    /// When a node exceeds its tick budget:
    /// - Regular nodes: Warning logged, counter incremented
    /// - Critical nodes: Emergency stop triggered
    ///
    /// # Example
    /// ```rust,ignore
    /// use horus_core::Scheduler;
    /// use std::time::Duration;
    ///
    /// let mut scheduler = Scheduler::new().watchdog(500_u64.ms());
    /// // tick budgets are auto-derived from .rate() on the node builder
    /// ```
    ///
    /// # Errors
    /// Returns an error if the safety monitor is not enabled.
    #[doc(hidden)]
    pub fn set_tick_budget(
        &mut self,
        node_name: &str,
        budget: std::time::Duration,
    ) -> crate::error::HorusResult<&mut Self> {
        if let Some(ref monitor) = self.monitor.safety {
            monitor.set_tick_budget(node_name.to_string(), budget);
            Ok(self)
        } else {
            Err(crate::error::HorusError::Config(
                crate::error::ConfigError::Other(
                    "Safety monitor not enabled. Use Scheduler::new().watchdog(500.ms()) to enable."
                        .to_string(),
                ),
            ))
        }
    }

    /// Set per-node rate control (chainable)
    ///
    /// Allows individual nodes to run at different frequencies independent of the global scheduler rate.
    /// If a node's rate is not set, it will tick at the global scheduler frequency.
    ///
    /// # Arguments
    /// * `name` - The name of the node
    /// * `rate` - The desired rate as a `Frequency` (e.g. `100_u64.hz()`)
    ///
    /// # Example
    /// ```ignore
    /// scheduler.set_node_rate("sensor", 100_u64.hz());
    /// ```
    #[doc(hidden)]
    pub fn set_node_rate(
        &mut self,
        name: &str,
        rate: crate::core::duration_ext::Frequency,
    ) -> &mut Self {
        let rate_hz = rate.value();
        let mut found = false;
        for registered in self.nodes.iter_mut() {
            if &*registered.name == name {
                registered.rate_hz = Some(rate_hz);
                registered.last_tick = Some(Instant::now());
                print_line(&format!("Set node '{}' rate to {:.1} Hz", name, rate_hz));
                found = true;
                break;
            }
        }
        if !found {
            print_line(&format!(
                "[WARN] set_node_rate('{}') — node not found (may be running on RT/Compute/Event executor)",
                name
            ));
        }
        self
    }

    /// Adjust tick_period to be fast enough for the fastest registered node.
    ///
    /// If a node declares rate_hz(500), the scheduler must tick at >= 500Hz
    /// for that node to actually achieve its declared rate.
    pub(super) fn adjust_tick_period_for_node_rates(&mut self) {
        let current_rate = 1_000_000.0 / self.tick.period.as_micros() as f64;
        let max_node_rate = self
            .nodes
            .iter()
            .filter_map(|n| n.rate_hz)
            .fold(0.0_f64, f64::max);

        if max_node_rate > current_rate {
            let new_period = max_node_rate.hz().period();
            print_line(&format!(
                "Adjusting scheduler tick rate from {:.1} Hz to {:.1} Hz (fastest node requires it)",
                current_rate, max_node_rate
            ));
            self.tick.period = new_period;
        }
    }

    /// Initialize nodes matching the optional filter.
    pub(super) fn initialize_filtered_nodes(&mut self, node_filter: Option<&[&str]>) {
        for registered in self.nodes.iter_mut() {
            let node_name = registered.name.as_ref();
            let should_run = node_filter.is_none_or(|filter| filter.contains(&node_name));

            if should_run && !registered.initialized {
                if let Some(ref mut ctx) = registered.context {
                    // Set node context for hlog!() macro
                    set_node_context(node_name, 0);
                    let init_result =
                        std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
                            registered.node.init()
                        }));
                    clear_node_context();

                    // Convert panic to error
                    let init_result = match init_result {
                        Ok(result) => result,
                        Err(_) => Err(crate::HorusError::Node(
                            crate::error::NodeError::InitPanic {
                                node: node_name.to_string(),
                            },
                        )),
                    };

                    match init_result {
                        Ok(()) => {
                            registered.initialized = true;
                            let publishers = registered.node.publishers();
                            let subscribers = registered.node.subscribers();

                            // Write presence file for monitor detection
                            let presence = match NodePresence::new(
                                node_name,
                                Some(&self.scheduler_name),
                                publishers,
                                subscribers,
                                registered.priority,
                                registered.rate_hz,
                            ) {
                                Ok(p) => p,
                                Err(e) => {
                                    print_line(&format!(
                                        "Warning: Invalid node name '{}': {}",
                                        node_name, e
                                    ));
                                    continue;
                                }
                            };
                            if let Err(e) = presence.write() {
                                print_line(&format!(
                                    "Warning: Failed to write presence file for '{}': {}",
                                    node_name, e
                                ));
                            }

                            print_line(&format!("Initialized node '{}'", node_name));
                        }
                        Err(e) => {
                            if let Some(hint) = e.help() {
                                print_line(&format!(
                                    "Failed to initialize node '{}': {}\n  hint: {}",
                                    node_name, e, hint
                                ));
                            } else {
                                print_line(&format!(
                                    "Failed to initialize node '{}': {}",
                                    node_name, e
                                ));
                            }

                            let severity = e.severity();
                            if severity == crate::error::Severity::Fatal {
                                print_line(&format!(
                                    " FATAL: Node '{}' init failed with fatal severity — stopping",
                                    node_name
                                ));
                                ctx.transition_to_crashed(format!("Fatal init: {}", e));
                                self.stop();
                                return;
                            } else if severity == crate::error::Severity::Transient {
                                ctx.transition_to_error(format!(
                                    "Init failed (transient, will retry): {}",
                                    e
                                ));
                            } else {
                                ctx.transition_to_error(format!("Initialization failed: {}", e));
                            }
                        }
                    }
                }
            }
        }
    }

    /// Re-initialize nodes that need restart (set by control commands).
    pub(super) fn reinit_pending_nodes(&mut self) {
        for registered in self.nodes.iter_mut() {
            if !registered.is_stopped && !registered.is_paused && !registered.initialized {
                let node_name = registered.name.as_ref();
                if let Some(ref mut ctx) = registered.context {
                    // Set node context for hlog!() macro
                    set_node_context(node_name, 0);
                    let init_result =
                        std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
                            registered.node.init()
                        }));
                    clear_node_context();

                    // Convert panic to error
                    let init_result = match init_result {
                        Ok(result) => result,
                        Err(_) => Err(crate::HorusError::Node(
                            crate::error::NodeError::ReInitPanic {
                                node: node_name.to_string(),
                            },
                        )),
                    };

                    match init_result {
                        Ok(()) => {
                            registered.initialized = true;
                            print_line(&format!("[CONTROL] Node '{}' re-initialized", node_name));
                        }
                        Err(e) => {
                            if let Some(hint) = e.help() {
                                print_line(&format!(
                                    "[CONTROL] Failed to re-initialize node '{}': {}\n  hint: {}",
                                    node_name, e, hint
                                ));
                            } else {
                                print_line(&format!(
                                    "[CONTROL] Failed to re-initialize node '{}': {}",
                                    node_name, e
                                ));
                            }

                            let severity = e.severity();
                            if severity == crate::error::Severity::Fatal {
                                registered.is_stopped = true;
                                ctx.transition_to_crashed(format!("Fatal re-init: {}", e));
                            } else if severity == crate::error::Severity::Transient {
                                ctx.transition_to_error(format!(
                                    "Re-init failed (transient, will retry): {}",
                                    e
                                ));
                            } else {
                                registered.is_stopped = true;
                                ctx.transition_to_error(format!("Re-initialization failed: {}", e));
                            }
                        }
                    }
                }
            }
        }
    }

    /// Shutdown nodes matching the optional filter.
    pub(super) fn shutdown_filtered_nodes(&mut self, node_filter: Option<&[&str]>) {
        for registered in self.nodes.iter_mut() {
            let node_name = registered.name.as_ref();
            let should_run = node_filter.is_none_or(|filter| filter.contains(&node_name));

            if should_run && registered.initialized {
                if let Some(ref mut ctx) = registered.context {
                    ctx.record_shutdown();

                    // Set node context for hlog!() macro
                    set_node_context(node_name, ctx.metrics().total_ticks());
                    let shutdown_result =
                        std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
                            registered.node.shutdown()
                        }));
                    clear_node_context();

                    // Convert panic to error
                    let shutdown_result = match shutdown_result {
                        Ok(result) => result,
                        Err(_) => Err(crate::HorusError::Node(
                            crate::error::NodeError::ShutdownPanic {
                                node: node_name.to_string(),
                            },
                        )),
                    };

                    match shutdown_result {
                        Ok(()) => {
                            // Remove presence file
                            if let Err(e) = NodePresence::remove(node_name) {
                                print_line(&format!(
                                    "Warning: Failed to remove presence file for '{}': {}",
                                    node_name, e
                                ));
                            }
                            print_line(&format!("Shutdown node '{}' successfully", node_name));
                        }
                        Err(e) => {
                            // Still try to remove presence file on error
                            let _ = NodePresence::remove(node_name);
                            print_line(&format!("Error shutting down node '{}': {}", node_name, e));
                        }
                    }
                }
            }
        }
    }

    /// Record final blackbox/telemetry events and clean up registry/session files.
    pub(super) fn finalize_run(&mut self) {
        let total_ticks = self
            .monitor
            .profiler
            .lock()
            .unwrap()
            .node_stats
            .values()
            .map(|s| s.count)
            .max()
            .unwrap_or(0) as u64;

        // Record scheduler stop to blackbox and save
        if let Some(ref bb) = self.monitor.blackbox {
            let mut bb = bb.lock().unwrap();
            bb.record(blackbox::BlackBoxEvent::SchedulerStop {
                reason: "Normal shutdown".to_string(),
                total_ticks,
            });
            // Flush any buffered WAL records that haven't reached the batch
            // interval yet — prevents losing the final events on clean exit.
            bb.flush_wal();
            if let Err(e) = bb.save() {
                print_line(&format!("[BLACKBOX] Failed to save: {}", e));
            }
        }

        // Final telemetry export
        if let Some(ref mut tm) = self.monitor.telemetry {
            tm.counter("scheduler_ticks", total_ticks);
            tm.gauge("scheduler_shutdown", 1.0);
            let _ = tm.export();
        }

        // Print timing report if profiling enabled and ticks were executed
        if self.pending_config.monitoring.verbose && total_ticks > 0 {
            self.print_timing_report();
        }

        // Clean up SHM node registry
        if let Some(ref reg) = self.registry {
            if let Err(e) = reg.remove() {
                print_line(&format!("Warning: Failed to remove scheduler registry: {}", e));
            }
        }

        // Clean up registry file and session
        self.cleanup_registry();
        Self::cleanup_session();

        print_line("Scheduler shutdown complete");
    }

    /// Print a formatted per-node timing report to stderr.
    ///
    /// Uses data from: RuntimeProfiler (Welford avg/stddev/min/max) and
    /// SafetyMonitor (ring buffer p99, budget, overruns, deadline misses).
    ///
    /// Zero runtime cost — only runs on shutdown. Output goes to stderr
    /// to avoid polluting stdout piping.
    fn print_timing_report(&self) {
        let profiler = self.monitor.profiler.lock().unwrap();
        if profiler.node_stats.is_empty() {
            return;
        }

        // Gather safety monitor data (if available)
        let safety_data: HashMap<String, (safety_monitor::TimingStats, Option<Duration>, u64)> =
            if let Some(ref monitor) = self.monitor.safety {
                monitor
                    .all_node_timing()
                    .into_iter()
                    .map(|(name, stats, budget, overruns)| (name, (stats, budget, overruns)))
                    .collect()
            } else {
                HashMap::new()
            };

        let sep = "=".repeat(90);
        let thin_sep = "-".repeat(90);
        eprintln!("\n{}", sep);
        eprintln!("TIMING REPORT (per-node)");
        eprintln!("{}", sep);
        eprintln!(
            "{:<20} {:>8} {:>8} {:>8} {:>8} {:>10} {:>8} {:>8}",
            "Node", "Avg(us)", "P99(us)", "Max(us)", "Stddev", "Budget(us)", "Overruns", "Misses"
        );
        eprintln!("{}", thin_sep);

        // Sort nodes by name for consistent output
        let mut node_names: Vec<&String> = profiler.node_stats.keys().collect();
        node_names.sort();

        let mut suggestions: Vec<String> = Vec::new();

        for name in &node_names {
            let stats = &profiler.node_stats[*name];
            if stats.count == 0 {
                continue;
            }

            let (p99_str, budget_str, overruns_str, misses_str) =
                if let Some((ref ring_stats, budget, overruns)) = safety_data.get(*name) {
                    let p99 = format!("{}", ring_stats.p99_us);
                    let budget_s = budget
                        .map(|b| format!("{}", b.as_micros()))
                        .unwrap_or_else(|| "-".to_string());
                    let overruns_s = format!("{}", overruns);
                    let misses_s = format!("{}", ring_stats.total_ticks); // reuse total_ticks
                    (p99, budget_s, overruns_s, misses_s)
                } else {
                    (
                        "-".to_string(),
                        "-".to_string(),
                        "0".to_string(),
                        "-".to_string(),
                    )
                };

            // Status indicator
            let status = if let Some((_, _, overruns)) = safety_data.get(*name) {
                if *overruns > 0 {
                    "!"
                } else {
                    ""
                }
            } else {
                ""
            };

            eprintln!(
                "{:<20} {:>8.0} {:>8} {:>8.0} {:>8.1} {:>10} {:>8} {:>7}{}",
                truncate_name(name, 20),
                stats.avg_us,
                p99_str,
                stats.max_us,
                stats.stddev_us,
                budget_str,
                overruns_str,
                misses_str,
                status,
            );

            // Generate suggestions
            if let Some((ref ring_stats, budget, overruns)) = safety_data.get(*name) {
                if budget.is_none() && ring_stats.p99_us > 0 {
                    suggestions.push(format!(
                        "  {} — no budget set (min={}us avg={}us p99={}us max={}us). Use .rate() to auto-derive budget",
                        name, ring_stats.min_us, ring_stats.avg_us, ring_stats.p99_us, ring_stats.max_us
                    ));
                }
                if *overruns > 10 {
                    suggestions.push(format!(
                        "  {} — {} overruns. Consider increasing budget or reducing tick complexity",
                        name, overruns
                    ));
                }
            }
        }

        eprintln!("{}", thin_sep);
        eprintln!(
            "Total nodes: {}  |  Ticked: {}",
            self.nodes.len(),
            node_names.len()
        );

        if !suggestions.is_empty() {
            eprintln!("\nSuggestions:");
            for s in &suggestions {
                eprintln!("{}", s);
            }
        }

        eprintln!("{}\n", sep);
    }

    /// Get information about all registered nodes
    #[doc(hidden)]
    pub fn node_list(&self) -> Vec<String> {
        self.nodes
            .iter()
            .map(|registered| registered.name.to_string())
            .collect()
    }

    /// Get performance metrics for all nodes.
    ///
    /// Internal: used by `benchmark()`, `horus monitor`, and timing reports.
    /// Users should call `benchmark()` for structured timing data.
    pub(crate) fn metrics(&self) -> Vec<NodeMetrics> {
        self.nodes
            .iter()
            .map(|registered| {
                let name = registered.name.to_string();
                let order = registered.priority;

                if let Some(ref ctx) = registered.context {
                    ctx.metrics().snapshot(name, order)
                } else {
                    NodeMetrics::new(name, order)
                }
            })
            .collect()
    }

    /// Get real-time statistics for a specific node.
    ///
    /// Returns `None` if the node doesn't exist or is not an RT node.
    pub(crate) fn rt_stats(&self, node_name: &str) -> Option<&crate::core::RtStats> {
        self.nodes
            .iter()
            .find(|n| &*n.name == node_name)
            .and_then(|n| n.rt_stats.as_ref())
    }

    /// Write metadata to registry file for monitor to read
    pub(super) fn update_registry(&self) {
        if let Ok(registry_path) = Self::get_registry_path() {
            let pid = std::process::id();

            // Collect pub/sub info from each node
            let nodes_json: Vec<String> = self.nodes.iter().map(|registered| {
                let name = registered.name.as_ref();
                let priority = registered.priority;

                // Get pub/sub from Node trait (macro-declared)
                // Runtime-discovered pub/sub is now tracked by TopicRegistry, not NodeInfo
                let publishers = registered.node.publishers();
                let subscribers = registered.node.subscribers();

                // Format publishers
                let pubs_json = publishers.iter()
                    .map(|p| format!("{{\"topic\": \"{}\", \"type\": \"{}\"}}",
                        p.topic_name.replace("\"", "\\\""),
                        p.type_name.replace("\"", "\\\"")))
                    .collect::<Vec<_>>()
                    .join(", ");

                // Format subscribers
                let subs_json = subscribers.iter()
                    .map(|s| format!("{{\"topic\": \"{}\", \"type\": \"{}\"}}",
                        s.topic_name.replace("\"", "\\\""),
                        s.type_name.replace("\"", "\\\"")))
                    .collect::<Vec<_>>()
                    .join(", ");

                format!(
                    "    {{\"name\": \"{}\", \"priority\": {}, \"publishers\": [{}], \"subscribers\": [{}]}}",
                    name, priority, pubs_json, subs_json
                )
            }).collect();

            let registry_data = format!(
                "{{\n  \"pid\": {},\n  \"scheduler_name\": \"{}\",\n  \"working_dir\": \"{}\",\n  \"nodes\": [\n{}\n  ]\n}}",
                pid,
                self.scheduler_name,
                self.monitor.working_dir.to_string_lossy(),
                nodes_json.join(",\n")
            );

            let _ = fs::write(&registry_path, registry_data);
        }
    }

    /// Remove registry file when scheduler stops
    fn cleanup_registry(&self) {
        if let Ok(registry_path) = Self::get_registry_path() {
            let _ = fs::remove_file(registry_path);
        }
    }

    /// Get path to registry file
    fn get_registry_path() -> Result<PathBuf, std::io::Error> {
        let mut path = dirs::home_dir().unwrap_or_else(|| PathBuf::from("/tmp"));
        path.push(".horus_registry.json");
        Ok(path)
    }

    /// Snapshot node state to registry (for crash forensics and persistence)
    /// Called every 5 seconds to avoid I/O overhead
    pub(super) fn snapshot_state_to_registry(&self) {
        if let Ok(registry_path) = Self::get_registry_path() {
            let pid = std::process::id();
            let timestamp = std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap_or_default()
                .as_secs();

            // Collect node info including state and health
            let nodes_json: Vec<String> = self.nodes.iter().map(|registered| {
                let name = registered.name.as_ref();
                let priority = registered.priority;

                // Get pub/sub from Node trait (macro-declared)
                // Runtime-discovered pub/sub is now tracked by TopicRegistry, not NodeInfo
                let publishers = registered.node.publishers();
                let subscribers = registered.node.subscribers();

                // Get state and health from context
                let (state_str, health_str, error_count, tick_count) = if let Some(ref ctx) = registered.context {
                    let metrics = ctx.metrics();
                    // Watchdog health overrides context-based health when degraded
                    let watchdog_health = registered.health_state.load();
                    let health = if watchdog_health != types::NodeHealthState::Healthy {
                        watchdog_health.to_string()
                    } else {
                        metrics.calculate_health().as_str().to_string()
                    };
                    (
                        ctx.state().to_string(),
                        health,
                        metrics.errors_count(),
                        metrics.total_ticks(),
                    )
                } else {
                    ("Unknown".to_string(), "Unknown".to_string(), 0, 0)
                };

                // Format publishers
                let pubs_json = publishers.iter()
                    .map(|p| format!("{{\"topic\": \"{}\", \"type\": \"{}\"}}",
                        p.topic_name.replace("\"", "\\\""),
                        p.type_name.replace("\"", "\\\"")))
                    .collect::<Vec<_>>()
                    .join(", ");

                // Format subscribers
                let subs_json = subscribers.iter()
                    .map(|s| format!("{{\"topic\": \"{}\", \"type\": \"{}\"}}",
                        s.topic_name.replace("\"", "\\\""),
                        s.type_name.replace("\"", "\\\"")))
                    .collect::<Vec<_>>()
                    .join(", ");

                format!(
                    "    {{\"name\": \"{}\", \"priority\": {}, \"state\": \"{}\", \"health\": \"{}\", \"error_count\": {}, \"tick_count\": {}, \"publishers\": [{}], \"subscribers\": [{}]}}",
                    name, priority, state_str, health_str, error_count, tick_count, pubs_json, subs_json
                )
            }).collect();

            let registry_data = format!(
                "{{\n  \"pid\": {},\n  \"scheduler_name\": \"{}\",\n  \"working_dir\": \"{}\",\n  \"last_snapshot\": {},\n  \"nodes\": [\n{}\n  ]\n}}",
                pid,
                self.scheduler_name,
                self.monitor.working_dir.to_string_lossy(),
                timestamp,
                nodes_json.join(",\n")
            );

            // Atomic write: write to temp file, then rename
            if let Some(parent) = registry_path.parent() {
                let temp_path = parent.join(format!(".horus_registry.json.tmp.{}", pid));

                // Write to temp file
                if fs::write(&temp_path, &registry_data).is_ok() {
                    // Atomically rename to final path
                    let _ = fs::rename(&temp_path, &registry_path);
                }
            }
        }
    }

    /// Execute a single node: tick + profiling + budget + deadline + failure handling.
    /// Returns true if the scheduler should stop (fatal failure).
    pub(super) fn execute_single_node(&mut self, i: usize, node_filter: Option<&[&str]>) -> bool {
        // Skip stopped nodes
        if self.nodes[i].is_stopped {
            return false;
        }

        // Skip Unhealthy nodes; poll Isolated nodes for safe-state recovery
        {
            use types::NodeHealthState;
            let health = self.nodes[i].health_state.load();
            if health == NodeHealthState::Unhealthy {
                return false;
            }
            if health == NodeHealthState::Isolated {
                if self.nodes[i].node.is_safe_state() {
                    self.nodes[i].health_state.store(NodeHealthState::Healthy);
                    print_line(&format!(
                        " Node '{}' recovered from safe state — resuming normal operation",
                        self.nodes[i].name
                    ));
                    // Fall through to tick this node
                } else {
                    return false; // Still in safe mode, skip
                }
            }
        }

        // Skip nodes in failure backoff/cooldown
        if let Some(ref handler) = self.nodes[i].failure_handler {
            if !handler.should_allow() {
                return false;
            }
        }

        // Auto-unpause nodes that were paused by Miss::Skip
        // (they skip exactly one tick, then resume)
        if self.nodes[i].is_paused {
            self.nodes[i].is_paused = false;
            return false;
        }

        let (should_run, should_tick) = {
            let registered = &self.nodes[i];
            let name = registered.name.as_ref();
            let should_run = node_filter.is_none_or(|filter| filter.contains(&name));

            // Check rate limiting (skip in deterministic mode — tick_once controls timing)
            let should_tick = if self.pending_config.timing.deterministic_order {
                true // deterministic mode: always tick (SimClock handles timing)
            } else if let Some(rate_hz) = registered.rate_hz {
                let current_time = Instant::now();
                if let Some(last_tick) = registered.last_tick {
                    let elapsed_secs = (current_time - last_tick).as_secs_f64();
                    let period_secs = 1.0 / rate_hz;
                    elapsed_secs >= period_secs
                } else {
                    true
                }
            } else {
                true
            };

            (should_run, should_tick)
        };

        if !should_tick {
            return false;
        }

        // Update last tick time if rate limited
        if self.nodes[i].rate_hz.is_some() {
            self.nodes[i].last_tick = Some(Instant::now());
        }

        if should_run && self.nodes[i].initialized {
            // Feed watchdog for RT nodes
            if self.nodes[i].is_rt_node {
                if let Some(ref monitor) = self.monitor.safety {
                    monitor.feed_watchdog(&self.nodes[i].name);
                }
            }

            // Begin recording tick (before node execution)
            {
                if let Some(ref mut recorder) = self.nodes[i].recorder {
                    recorder.begin_tick(self.tick.current);
                }
            }

            // Capture inputs from subscriber topics (for recording)
            {
                let RegisteredNode {
                    ref mut node,
                    ref mut recorder,
                    ..
                } = self.nodes[i];
                if let Some(recorder) = recorder.as_mut() {
                    if recorder.is_active_tick() {
                        let subscribers = node.subscribers();
                        if !subscribers.is_empty() {
                            let topics_dir = crate::memory::platform::shm_topics_dir();
                            for sub in &subscribers {
                                let topic_path = topics_dir.join(&sub.topic_name);
                                if let Some(slot_read) =
                                    crate::communication::read_latest_slot_bytes(&topic_path, 0)
                                {
                                    recorder.record_input(&sub.topic_name, slot_read.payload);
                                }
                            }
                        }
                    }
                }
            }

            // Replay: if this node has a replayer, advance it and feed the
            // recorded (or overridden) outputs into the node's recorder so
            // that diff/export tools can compare replay results.
            //
            // Replay: advance replayer, feed outputs into recorder AND inject
            // into shared-memory topics so live subscriber nodes see replay data.
            {
                let node_name = self.nodes[i].name.clone();
                let replay_outputs: Option<Vec<(String, Vec<u8>)>> =
                    self.replay.as_mut().and_then(|replay| {
                        let replayer = replay.nodes.get_mut(node_name.as_ref())?;
                        let node_overrides = replay.overrides.get(node_name.as_ref());
                        let snapshot = replayer.current_snapshot()?;
                        let outputs: Vec<(String, Vec<u8>)> = snapshot
                            .outputs
                            .iter()
                            .map(|(topic, data)| {
                                let output_data = node_overrides
                                    .and_then(|ovr| ovr.get(topic))
                                    .unwrap_or(data)
                                    .clone();
                                (topic.clone(), output_data)
                            })
                            .collect();
                        replayer.advance();
                        Some(outputs)
                    });
                if let Some(outputs) = replay_outputs {
                    for (topic, data) in &outputs {
                        // Inject into shared-memory topic so live subscriber nodes see it
                        let shm_dir = crate::memory::platform::shm_topics_dir();
                        let topic_path = shm_dir.join(format!("horus_{}", topic));
                        if topic_path.exists() {
                            crate::communication::write_topic_slot_bytes(&topic_path, data);
                        }
                    }
                    // Also feed into recorder for diff/export tools
                    if let Some(ref mut recorder) = self.nodes[i].recorder {
                        for (topic, data) in outputs {
                            recorder.record_output(&topic, data);
                        }
                    }
                }
            }

            let (tick_start, tick_duration, tick_result) = {
                let registered = &mut self.nodes[i];
                if let Some(ref mut context) = registered.context {
                    context.start_tick();

                    // Set node context for hlog!() macro
                    let tick_number = context.metrics().total_ticks();
                    set_node_context(&registered.name, tick_number);

                    // Set tick context for horus::now(), horus::dt(), horus::rng() etc.
                    let clock_ref: &dyn crate::core::clock::Clock = &*self.clock;
                    let node_dt = registered
                        .rate_hz
                        .map(|hz| Duration::from_secs_f64(1.0 / hz))
                        .unwrap_or(self.tick.period);
                    let sim_time = self.clock.elapsed();
                    let tick_start_ci = self.clock.now();
                    set_tick_context(
                        &registered.name,
                        tick_number,
                        clock_ref,
                        node_dt,
                        sim_time,
                        tick_start_ci,
                        registered.tick_budget,
                    );

                    // Execute node tick via NodeRunner (timing + panic isolation)
                    let tr = primitives::NodeRunner::run_tick(&mut registered.node);

                    clear_tick_context();
                    clear_node_context();
                    (tr.tick_start, tr.duration, tr.result)
                } else {
                    return false;
                }
            };

            return self.process_tick_result(i, tick_start, tick_duration, tick_result);
        }
        false
    }

    /// Process the result of a node tick (profiling, budget, failure handling).
    /// Returns true if the scheduler should stop.
    fn process_tick_result(
        &mut self,
        i: usize,
        tick_start: Instant,
        tick_duration: Duration,
        tick_result: std::thread::Result<()>,
    ) -> bool {
        // Profiling and monitoring
        {
            let node_name = self.nodes[i].name.as_ref();
            let mut profiler = self.monitor.profiler.lock().unwrap();
            if tick_result.is_err() {
                profiler.record_node_failure(node_name);
                print_line(&format!("Node '{}' panicked during execution", node_name));
            }
            profiler.record(node_name, tick_duration);
        }

        // Update per-node RtStats
        if let Some(ref mut stats) = self.nodes[i].rt_stats {
            stats.record_execution(tick_duration);
        }

        // Update SHM registry with live metrics
        self.update_registry_for_node(i, tick_duration);

        // Capture outputs from publisher topics (for recording)
        {
            let RegisteredNode {
                ref mut node,
                ref mut recorder,
                ..
            } = self.nodes[i];
            if let Some(recorder) = recorder.as_mut() {
                if recorder.is_active_tick() {
                    let publishers = node.publishers();
                    if !publishers.is_empty() {
                        let topics_dir = crate::memory::platform::shm_topics_dir();
                        for pub_topic in &publishers {
                            let topic_path = topics_dir.join(&pub_topic.topic_name);
                            if let Some(slot_read) =
                                crate::communication::read_latest_slot_bytes(&topic_path, 0)
                            {
                                recorder.record_output(&pub_topic.topic_name, slot_read.payload);
                            }
                        }
                    }
                }
            }
        }

        // End recording tick
        if let Some(ref mut recorder) = self.nodes[i].recorder {
            recorder.end_tick(tick_duration.as_nanos() as u64);
        }

        // Check timing violations for RT nodes
        if self.check_timing_violations(i, tick_start, tick_duration) {
            return true; // Emergency stop triggered by deadline policy
        }

        // Handle tick result
        match tick_result {
            Ok(_) => {
                if let Some(ref mut context) = self.nodes[i].context {
                    context.record_tick();
                }

                // Record successful tick for failure handler (clears backoff state)
                if let Some(ref mut handler) = self.nodes[i].failure_handler {
                    handler.record_success();
                }

                // Recovery: successful tick transitions Warning→Healthy
                {
                    use types::NodeHealthState;
                    let health = self.nodes[i].health_state.load();
                    if health == NodeHealthState::Warning {
                        self.nodes[i].health_state.store(NodeHealthState::Healthy);
                        // Feed watchdog on recovery so the graduated check sees fresh timestamp
                        if let Some(ref monitor) = self.monitor.safety {
                            monitor.feed_watchdog(&self.nodes[i].name);
                        }
                    }
                }

                // Graduated degradation recovery: track successful ticks at reduced rate
                if let Some(ref monitor) = self.monitor.safety {
                    let action = monitor.record_successful_tick(&self.nodes[i].name);
                    self.apply_degradation_action(i, action);
                }

                false
            }
            Err(panic_err) => self.handle_tick_failure(i, panic_err),
        }
    }

    /// Update the SHM registry with live metrics for a node after its tick.
    fn update_registry_for_node(&self, i: usize, tick_duration: Duration) {
        if let Some(ref reg) = self.registry {
            let node_name = self.nodes[i].name.as_ref();
            if let Some(&slot_idx) = self.registry_slots.get(node_name) {
                let health = self.nodes[i].health_state.load() as u8;
                let (tick_count, error_count) = self.nodes[i]
                    .context
                    .as_ref()
                    .map(|ctx| {
                        let m = ctx.metrics();
                        (m.total_ticks(), m.errors_count() as u32)
                    })
                    .unwrap_or((0, 0));

                let tick_ns = tick_duration.as_nanos() as u64;

                // Get rolling average and max from profiler (microseconds → nanoseconds)
                let (avg_ns, max_ns) = self
                    .monitor
                    .profiler
                    .lock()
                    .ok()
                    .and_then(|p| {
                        p.node_stats.get(node_name).map(|s| {
                            ((s.avg_us * 1000.0) as u64, (s.max_us * 1000.0) as u64)
                        })
                    })
                    .unwrap_or((tick_ns, tick_ns));

                // Budget/deadline misses from RtStats (if available)
                let (budget_misses, deadline_misses) = self.nodes[i]
                    .rt_stats
                    .as_ref()
                    .map(|s| (s.budget_violations() as u32, s.deadline_misses() as u32))
                    .unwrap_or((0, 0));

                reg.update_node(
                    slot_idx,
                    health,
                    tick_count,
                    error_count,
                    budget_misses,
                    deadline_misses,
                    tick_ns,
                    avg_ns,
                    max_ns,
                );
            }
        }
    }

    /// Check tick budget and deadline violations for real-time nodes.
    ///
    /// Uses `TimingEnforcer` for the core budget/deadline detection, then dispatches
    /// node callbacks, stats updates, blackbox recording, and policy actions.
    ///
    /// Returns `true` if the scheduler should stop (EmergencyStop policy).
    fn check_timing_violations(
        &mut self,
        i: usize,
        tick_start: Instant,
        tick_duration: Duration,
    ) -> bool {
        use primitives::{DeadlineAction, TimingEnforcer};

        let node_name = Arc::clone(&self.nodes[i].name);

        // Check tick budget for RT nodes via TimingEnforcer
        if self.nodes[i].is_rt_node {
            if let Some(tick_budget) = self.nodes[i].tick_budget {
                if let Some(budget_result) =
                    TimingEnforcer::check_tick_budget(&node_name, tick_duration, tick_budget)
                {
                    let violation = &budget_result.violation;
                    print_line(&format!(
                        " budget violation in {}: {:?} > {:?}",
                        violation.node_name(),
                        violation.actual(),
                        violation.budget()
                    ));

                    // Update RtStats
                    if let Some(ref mut stats) = self.nodes[i].rt_stats {
                        stats.record_budget_violation();
                    }

                    // Record in blackbox
                    if let Some(ref bb) = self.monitor.blackbox {
                        bb.lock()
                            .unwrap()
                            .record(blackbox::BlackBoxEvent::BudgetViolation {
                                name: node_name.to_string(),
                                budget_us: violation.budget().as_micros() as u64,
                                actual_us: violation.actual().as_micros() as u64,
                            });
                    }

                    // Also report to safety monitor if available
                    if let Some(ref monitor) = self.monitor.safety {
                        let _ = monitor.check_tick_budget(&node_name, tick_duration);
                    }
                }
            }
        }

        // Check deadline for RT nodes via TimingEnforcer
        if self.nodes[i].is_rt_node {
            if let Some(deadline) = self.nodes[i].deadline {
                if let Some(dm) =
                    TimingEnforcer::check_deadline(tick_start, deadline, self.nodes[i].miss_policy)
                {
                    if let Some(ref monitor) = self.monitor.safety {
                        monitor.record_deadline_miss(&node_name);
                    }
                    print_line(&format!(
                        " Deadline miss in {}: {:?} > {:?}",
                        node_name, dm.elapsed, dm.deadline
                    ));

                    // Update RtStats
                    if let Some(ref mut stats) = self.nodes[i].rt_stats {
                        stats.record_deadline_miss();
                    }

                    // Record in blackbox
                    if let Some(ref bb) = self.monitor.blackbox {
                        bb.lock()
                            .unwrap()
                            .record(blackbox::BlackBoxEvent::DeadlineMiss {
                                name: node_name.to_string(),
                                deadline_us: dm.deadline.as_micros() as u64,
                                actual_us: dm.elapsed.as_micros() as u64,
                            });
                    }

                    // Graduated degradation: evaluate and apply corrective action
                    if let Some(ref monitor) = self.monitor.safety {
                        let consecutive = monitor.consecutive_misses(&node_name);
                        let action = monitor.evaluate_degradation(
                            &node_name,
                            consecutive,
                            self.nodes[i].rate_hz,
                        );
                        self.apply_degradation_action(i, action);
                    }

                    // Dispatch on deadline action
                    match dm.action {
                        DeadlineAction::Warn => {
                            // Already logged above — no further action
                        }
                        DeadlineAction::Skip => {
                            self.nodes[i].is_paused = true;
                            print_line(&format!(
                                " Deadline policy: skipping '{}' for one tick",
                                node_name
                            ));
                        }
                        DeadlineAction::SafeMode => {
                            print_line(&format!(
                                " Deadline policy: '{}' entering safe state",
                                node_name
                            ));
                            self.nodes[i].node.enter_safe_state();
                            if let Some(ref monitor) = self.monitor.safety {
                                monitor.record_degrade_activation();
                            }
                        }
                        DeadlineAction::EmergencyStop => {
                            print_line(&format!(
                                " Deadline policy: emergency stop triggered by '{}'",
                                node_name
                            ));
                            if let Some(ref monitor) = self.monitor.safety {
                                monitor.trigger_emergency_stop(format!(
                                    "Deadline miss in '{}': {:?} > {:?}",
                                    node_name, dm.elapsed, dm.deadline
                                ));
                            }
                            return true;
                        }
                    }
                }
            }
        }
        false
    }

    /// Apply a degradation action produced by the safety monitor.
    fn apply_degradation_action(&mut self, i: usize, action: safety_monitor::DegradationAction) {
        use safety_monitor::DegradationAction;
        use types::NodeHealthState;

        match action {
            DegradationAction::None => {}
            DegradationAction::Warn(ref name) => {
                print_line(&format!(
                    " Degradation: '{}' — sustained timing violations, monitoring",
                    name
                ));
            }
            DegradationAction::ReduceRate {
                ref node,
                new_rate_hz,
            } => {
                self.nodes[i].rate_hz = Some(new_rate_hz);
                self.nodes[i].last_tick = Some(Instant::now());
                print_line(&format!(
                    " Degradation: '{}' — reducing rate to {:.1} Hz",
                    node, new_rate_hz
                ));
            }
            DegradationAction::Isolate(ref name) => {
                self.nodes[i].health_state.store(NodeHealthState::Isolated);
                self.nodes[i].node.enter_safe_state();
                print_line(&format!(
                    " Degradation: '{}' — isolated, entered safe state",
                    name
                ));
                if let Some(ref monitor) = self.monitor.safety {
                    monitor.record_degrade_activation();
                }
            }
            DegradationAction::RestoreRate {
                ref node,
                original_rate_hz,
            } => {
                self.nodes[i].rate_hz = Some(original_rate_hz);
                self.nodes[i].last_tick = Some(Instant::now());
                self.nodes[i].health_state.store(NodeHealthState::Healthy);
                print_line(&format!(
                    " Recovery: '{}' — restored to {:.1} Hz",
                    node, original_rate_hz
                ));
            }
        }
    }

    /// Handle a node tick failure.
    /// Returns `true` if the scheduler should stop (fatal failure).
    fn handle_tick_failure(&mut self, i: usize, panic_err: Box<dyn std::any::Any + Send>) -> bool {
        let error_msg = if let Some(s) = panic_err.downcast_ref::<&str>() {
            format!("Node panicked: {}", s)
        } else if let Some(s) = panic_err.downcast_ref::<String>() {
            format!("Node panicked: {}", s)
        } else {
            "Node panicked with unknown error".to_string()
        };

        // Arc::clone — cheap atomic increment (only on node panic)
        let node_name = Arc::clone(&self.nodes[i].name);
        let registered = &mut self.nodes[i];
        if let Some(ref mut context) = registered.context {
            context.record_tick_failure(error_msg.clone());

            // Set context for on_error handler
            set_node_context(&node_name, context.metrics().total_ticks());
            registered.node.on_error(&error_msg);
            clear_node_context();

            print_line(&format!(
                " Node '{}' failed (continuing): {}",
                node_name, error_msg
            ));
            context.transition_to_error(error_msg);
        }

        // Consult failure handler if configured
        if let Some(ref mut handler) = self.nodes[i].failure_handler {
            match handler.record_failure() {
                fault_tolerance::FailureAction::StopScheduler
                | fault_tolerance::FailureAction::FatalAfterRestarts => {
                    print_line(&format!(
                        " [FATAL] Node '{}' failure policy triggered scheduler stop",
                        node_name
                    ));
                    self.running
                        .store(false, std::sync::atomic::Ordering::SeqCst);
                    return true;
                }
                fault_tolerance::FailureAction::RestartNode => {
                    print_line(&format!(
                        " Node '{}' will restart (failure policy)",
                        node_name
                    ));
                    self.nodes[i].initialized = false;
                    return false;
                }
                fault_tolerance::FailureAction::SkipNode => {
                    return false;
                }
                fault_tolerance::FailureAction::Continue => return false,
            }
        }
        false
    }
}
