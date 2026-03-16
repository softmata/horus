//! Builder methods and status reporting for `Scheduler`.
//!
//! Contains all chainable configuration methods (`.name()`, `.deterministic()`,
//! `.cores()`, `.prefer_rt()`, etc.) and status/query methods (`.status()`,
//! `.capabilities()`, `.degradations()`, etc.).

use super::*;

// Re-import scheduling-level modules that `super::` no longer reaches
use crate::scheduling::blackbox;
use crate::scheduling::rt;
use crate::scheduling::safety_monitor;
use crate::scheduling::types;

impl Scheduler {
    // ========================================================================
    // BUILDER METHODS — composable, chainable, explicit
    // ========================================================================

    /// Set the scheduler name (used in status reports, crash logs, and SHM registry).
    ///
    /// Default: `"Scheduler"`.
    ///
    /// # Example
    /// ```rust,ignore
    /// let scheduler = Scheduler::new()
    ///     .name("motor_control")
    ///     .tick_rate(1000_u64.hz());
    /// ```
    pub fn name(mut self, name: &str) -> Self {
        self.scheduler_name = name.to_string();
        self
    }

    /// Enable deterministic execution order.
    ///
    /// When enabled, **all** nodes run sequentially on the main thread —
    /// no executor threads are spawned. This guarantees identical execution
    /// order across runs, which is essential for simulation replay and
    /// deterministic testing with `tick_once()`.
    ///
    /// Default: `false`.
    ///
    /// # Example
    /// ```rust,ignore
    /// let mut scheduler = Scheduler::new()
    ///     .deterministic(true)
    ///     .tick_rate(100_u64.hz());
    ///
    /// scheduler.add(Physics).order(0).rate(100_u64.hz()).build()?;
    /// scheduler.add(Controller).order(1).rate(100_u64.hz()).build()?;
    ///
    /// // Every tick_once() executes Physics then Controller, always
    /// scheduler.tick_once()?;
    /// ```
    pub fn deterministic(mut self, enabled: bool) -> Self {
        self.pending_config.timing.deterministic_order = enabled;
        if enabled {
            self.clock = Arc::new(crate::core::clock::SimClock::new());
        }
        self
    }

    /// Pin the scheduler to specific CPU cores.
    ///
    /// Restricts the scheduler's main thread and executor threads to the
    /// specified cores. On NUMA systems, pinning to cores on the same node
    /// reduces cross-node memory access latency.
    ///
    /// Default: all cores (no pinning).
    ///
    /// # Example
    /// ```rust,ignore
    /// let scheduler = Scheduler::new()
    ///     .cores(&[2, 3])            // pin to cores 2 and 3
    ///     .prefer_rt()
    ///     .tick_rate(1000_u64.hz());
    /// ```
    pub fn cores(mut self, cpu_ids: &[usize]) -> Self {
        self.pending_config.resources.cpu_cores = Some(cpu_ids.to_vec());
        self
    }

    /// Try to enable OS-level RT features (mlockall, SCHED_FIFO).
    ///
    /// If the system lacks RT capabilities, logs degradation warnings and
    /// continues running with reduced capabilities. **Never panics.**
    ///
    /// Use `.require_rt()` instead if you need hard failure on non-RT systems.
    ///
    /// # Example
    /// ```rust,ignore
    /// let scheduler = Scheduler::new()
    ///     .prefer_rt()           // try RT, warn if unavailable
    ///     .watchdog(500_u64.ms())
    ///     .tick_rate(100_u64.hz());
    /// ```
    pub fn prefer_rt(mut self) -> Self {
        self.pending_config.realtime.memory_locking = true;
        self.pending_config.realtime.rt_scheduling_class = true;
        self
    }

    /// Enable OS-level RT features (mlockall, SCHED_FIFO).
    ///
    /// **Panics** if the system lacks RT capabilities. The name `require_rt`
    /// makes this explicit — you chose to fail loudly.
    ///
    /// Use `.prefer_rt()` for graceful degradation instead.
    ///
    /// # Panics
    /// Panics if the system has neither `SCHED_FIFO` nor `mlockall` support.
    ///
    /// # Example
    /// ```rust,ignore
    /// let scheduler = Scheduler::new()
    ///     .require_rt()          // panic if RT unavailable
    ///     .watchdog(500_u64.ms())
    ///     .tick_rate(1000_u64.hz());
    /// ```
    pub fn require_rt(mut self) -> Self {
        let can_rt = self
            .rt
            .capabilities
            .as_ref()
            .is_some_and(|c| c.rt_priority_available || c.mlockall_permitted);
        assert!(
            can_rt,
            "require_rt(): system lacks RT capabilities.\n\
             Use .prefer_rt() to run with graceful degradation instead."
        );
        self.pending_config.realtime.memory_locking = true;
        self.pending_config.realtime.rt_scheduling_class = true;
        self
    }

    /// Enable the watchdog — detects frozen or unresponsive nodes.
    ///
    /// When set, a safety monitor is auto-created that watches all registered
    /// nodes. If a node doesn't tick within the timeout, the watchdog triggers
    /// graduated degradation (warn → reduce rate → isolate → safe state).
    ///
    /// Budget enforcement and deadline monitoring are always active for nodes
    /// that have `.rate()` set (no flag needed).
    ///
    /// # Example
    /// ```rust,ignore
    /// let scheduler = Scheduler::new()
    ///     .watchdog(500_u64.ms())    // 500ms timeout for frozen nodes
    ///     .tick_rate(100_u64.hz());
    /// ```
    pub fn watchdog(mut self, timeout: Duration) -> Self {
        self.pending_config.realtime.watchdog_timeout_ms = timeout.as_millis() as u64;
        self
    }

    /// Enable the blackbox flight recorder for crash forensics.
    ///
    /// Records critical events (deadline misses, budget violations, fault tolerance
    /// state changes, emergency stops) in a ring buffer. After a crash, the blackbox
    /// provides a timeline of what happened.
    ///
    /// # Example
    /// ```rust,ignore
    /// let scheduler = Scheduler::new()
    ///     .blackbox(64)              // 64MB flight recorder
    ///     .tick_rate(100_u64.hz());
    /// ```
    pub fn blackbox(mut self, size_mb: usize) -> Self {
        self.pending_config.monitoring.black_box_size_mb = size_mb;
        let bb_dir = self.monitor.working_dir.join(".horus").join("blackbox");
        let bb = blackbox::BlackBox::new(size_mb).with_path(bb_dir);
        self.monitor.blackbox = Some(Arc::new(Mutex::new(bb)));
        self
    }

    /// Set the maximum number of deadline misses before emergency stop.
    ///
    /// Default: 100. Only meaningful when nodes have `.rate()` set.
    ///
    /// # Example
    /// ```rust,ignore
    /// let scheduler = Scheduler::new()
    ///     .max_deadline_misses(10)   // strict — stop after 10 misses
    ///     .tick_rate(1000_u64.hz());
    /// ```
    pub fn max_deadline_misses(mut self, n: u64) -> Self {
        self.pending_config.realtime.max_deadline_misses = n.max(1);
        self
    }

    /// Set the global tick rate.
    ///
    /// # Example
    /// ```rust,ignore
    /// use horus::prelude::*;
    ///
    /// let scheduler = Scheduler::new()
    ///     .tick_rate(1000_u64.hz()); // 1kHz control loop
    /// ```
    pub fn tick_rate(mut self, freq: crate::core::duration_ext::Frequency) -> Self {
        self.pending_config.timing.global_rate_hz = freq.value();
        self
    }

    /// Enable or disable verbose logging from executor threads.
    ///
    /// When disabled, suppresses non-emergency messages from the RT thread
    /// (budget warnings, pre/post-condition failures, startup/shutdown notices).
    /// Emergency-stop messages are always printed regardless of this setting.
    ///
    /// Default: `true` (verbose logging enabled).
    pub fn verbose(mut self, enabled: bool) -> Self {
        self.pending_config.monitoring.verbose = enabled;
        self
    }

    /// Enable recording with sensible defaults.
    ///
    /// Equivalent to applying `SchedulerConfig` with `recording: Some(RecordingConfigYaml::full())`.
    ///
    /// # Example
    /// ```rust,ignore
    /// let mut scheduler = Scheduler::new()
    ///     .tick_rate(100_u64.hz())
    ///     .with_recording();
    /// ```
    pub fn with_recording(mut self) -> Self {
        let recording_yaml = crate::scheduling::config::RecordingConfigYaml::full();
        self.pending_config.recording = Some(recording_yaml.clone());
        let recording_config = crate::scheduling::record_replay::RecordingConfig::from(recording_yaml);
        let scheduler_id = format!(
            "{:x}{:x}",
            std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .map(|d| d.as_nanos() as u64)
                .unwrap_or(0),
            std::process::id() as u64
        );
        let session_name = recording_config.session_name.clone();
        self.recording = Some(RecordingState {
            config: recording_config,
            scheduler_recording: crate::scheduling::record_replay::SchedulerRecording::new(
                &scheduler_id,
                &session_name,
            ),
        });
        self
    }

    /// Set the telemetry export endpoint.
    ///
    /// Telemetry data (node timing, deadline misses, budget violations) is
    /// exported to the specified endpoint. Supports UDP and file URIs.
    ///
    /// Can also be set via `HORUS_TELEMETRY_ENDPOINT` env var (builder takes precedence).
    ///
    /// # Example
    /// ```rust,ignore
    /// let scheduler = Scheduler::new()
    ///     .telemetry("udp://localhost:9999")
    ///     .tick_rate(100_u64.hz());
    /// ```
    pub fn telemetry(mut self, endpoint: &str) -> Self {
        self.pending_config.monitoring.telemetry_endpoint = Some(endpoint.to_string());
        self
    }

    // Profiling: always on (negligible overhead).
    // Budget enforcement + deadline monitoring: always on when nodes have .rate() set.

    /// Get the detected runtime capabilities.
    ///
    /// Returns `None` if `Scheduler::simulation()` was used (skips detection).
    ///
    /// # Example
    /// ```rust,ignore
    /// if let Some(caps) = scheduler.capabilities() {
    ///     if caps.has_hard_rt_support() {
    ///         println!("Hard RT available!");
    ///     }
    /// }
    /// ```
    #[doc(hidden)]
    pub fn capabilities(&self) -> Option<&RuntimeCapabilities> {
        self.rt.capabilities.as_ref()
    }

    /// Get the list of RT features that degraded during auto-optimization.
    ///
    /// An empty list means all detected features were successfully applied.
    ///
    /// # Example
    /// ```rust,ignore
    /// for deg in scheduler.degradations() {
    ///     match deg.severity {
    ///         DegradationSeverity::High => {
    ///             eprintln!("CRITICAL: {} - {}", deg.feature, deg.reason);
    ///         }
    ///         _ => {
    ///             println!("Note: {} - {}", deg.feature, deg.reason);
    ///         }
    ///     }
    /// }
    /// ```
    pub(crate) fn degradations(&self) -> &[RtFeatureDegradation] {
        &self.rt.degradations
    }

    /// Check if the scheduler has full RT capabilities (no high-severity degradations).
    pub(crate) fn has_full_rt(&self) -> bool {
        !self
            .rt
            .degradations
            .iter()
            .any(|d| d.severity == DegradationSeverity::High)
    }

    /// Get a reference to the BlackBox flight recorder.
    ///
    /// The BlackBox automatically records critical events for post-mortem crash analysis:
    /// - Scheduler start/stop
    /// - Node additions
    /// - Deadline misses
    /// - budget violations
    /// - Fault tolerance state changes
    /// - Emergency stops
    ///
    /// # Example
    /// ```rust,ignore
    /// use horus_core::Scheduler;
    ///
    /// let scheduler = Scheduler::new();
    ///
    /// // Access the blackbox for debugging
    /// if let Some(bb) = scheduler.blackbox() {
    ///     // Get recent events
    ///     let events = bb.recent(100);
    ///     println!("Last 100 events: {:?}", events.len());
    /// }
    /// ```
    pub(crate) fn get_blackbox(&self) -> Option<&Arc<Mutex<blackbox::BlackBox>>> {
        self.monitor.blackbox.as_ref()
    }

    /// Get safety statistics including budget overruns, deadline misses, and watchdog expirations.
    ///
    /// Returns `None` if the safety monitor is not enabled.
    ///
    /// The returned `SafetyStats` contains:
    /// - `state`: Current safety state (Normal, Degraded, EmergencyStop)
    /// - `budget_overruns`: Number of tick budget violations
    /// - `deadline_misses`: Number of deadline misses
    /// - `watchdog_expirations`: Number of watchdog timeouts
    ///
    /// # Example
    /// ```rust,ignore
    /// let scheduler = Scheduler::new();
    /// // ... run scheduler for a while ...
    ///
    /// if let Some(stats) = scheduler.safety_stats() {
    ///     println!("budget overruns: {}", stats.budget_overruns());
    ///     println!("Deadline misses: {}", stats.deadline_misses());
    /// }
    /// ```
    #[doc(hidden)]
    pub fn safety_stats(&self) -> Option<safety_monitor::SafetyStats> {
        self.monitor.safety.as_ref().map(|m| m.get_stats())
    }

    /// Get the scheduler name.
    ///
    /// Returns the name assigned via `.name()` (default: `"Scheduler"`).
    pub(crate) fn scheduler_name(&self) -> &str {
        &self.scheduler_name
    }

    /// Get a formatted status report of the scheduler's configuration and state.
    ///
    /// Returns a human-readable string showing:
    /// - Scheduler name and platform
    /// - RT features status (priority, memory lock, CPU affinity)
    /// - Determinism mode (simulation vs real-time)
    /// - Safety features (monitor, watchdog, blackbox)
    /// - Any degradations that occurred during initialization
    ///
    /// This is useful for debugging and auditing scheduler configuration.
    ///
    /// # Example
    /// ```rust,ignore
    /// let scheduler = Scheduler::new();
    /// println!("{}", scheduler.status());
    /// // Output:
    /// // ==================================================================
    /// // SCHEDULER STATUS: AutoOptimized
    /// // ==================================================================
    /// // Platform: Linux 5.15.0-generic (8 CPUs, 1 NUMA nodes)
    /// // ------------------------------------------------------------------
    /// // RT Features:
    /// //   [x] PREEMPT_RT kernel
    /// //   [ ] RT Priority (max=0)
    /// //   [x] Memory Lock (limit=64MB)
    /// //   [ ] Isolated CPUs: []
    /// // ------------------------------------------------------------------
    /// // Execution Mode:
    /// //   [x] Real-time Mode
    /// // ------------------------------------------------------------------
    /// // Safety Features:
    /// //   [ ] Safety Monitor
    /// //   [ ] BlackBox Recorder
    /// // ==================================================================
    /// ```
    pub fn status(&self) -> String {
        let sep = "==================================================================";
        let mut lines = Vec::new();

        // Header
        lines.push(sep.to_string());
        lines.push(format!("SCHEDULER STATUS: {}", self.scheduler_name));
        lines.push(sep.to_string());

        // Sections
        lines.extend(self.platform_status_lines());
        lines.extend(self.safety_status_lines());
        lines.extend(self.node_health_status_lines());

        // Footer
        lines.push(sep.to_string());
        lines.join("\n")
    }

    /// Build platform and RT capability status lines.
    fn platform_status_lines(&self) -> Vec<String> {
        let thin_sep = "------------------------------------------------------------------";
        let mut lines = Vec::new();

        if let Some(caps) = &self.rt.capabilities {
            lines.push(format!(
                "Platform: {} ({} CPUs, {} NUMA nodes)",
                caps.kernel_version, caps.cpu_count, caps.numa_node_count
            ));

            lines.push(thin_sep.to_string());
            lines.push("RT Features:".to_string());
            lines.push(format!(
                "  [{}] PREEMPT_RT kernel",
                if caps.preempt_rt { "x" } else { " " }
            ));
            lines.push(format!(
                "  [{}] RT Priority (max={})",
                if caps.rt_priority_available { "x" } else { " " },
                caps.max_rt_priority
            ));
            lines.push(format!(
                "  [{}] Memory Lock (limit={}MB)",
                if caps.mlockall_permitted { "x" } else { " " },
                caps.memlock_limit_bytes / (1024 * 1024)
            ));
            lines.push(format!(
                "  [{}] Isolated CPUs: {:?}",
                if !caps.isolated_cpus.is_empty() {
                    "x"
                } else {
                    " "
                },
                caps.isolated_cpus
            ));
            lines.push(format!(
                "  [{}] Tickless CPUs (nohz_full): {:?}",
                if !caps.nohz_full_cpus.is_empty() {
                    "x"
                } else {
                    " "
                },
                caps.nohz_full_cpus
            ));
        } else {
            lines.push("Platform: (capabilities not detected)".to_string());
        }

        lines
    }

    /// Build safety features, execution mode, and budget statistics status lines.
    fn safety_status_lines(&self) -> Vec<String> {
        let thin_sep = "------------------------------------------------------------------";
        let mut lines = Vec::new();

        // Execution mode
        lines.push(thin_sep.to_string());
        lines.push("Execution Mode:".to_string());
        lines.push("  [x] Real-time Mode".to_string());

        // Safety features
        lines.push(thin_sep.to_string());
        lines.push("Safety Features:".to_string());
        lines.push(format!(
            "  [{}] Safety Monitor",
            if self.monitor.safety.is_some() {
                "x"
            } else {
                " "
            }
        ));
        lines.push(format!(
            "  [{}] BlackBox Recorder",
            if self.monitor.blackbox.is_some() {
                "x"
            } else {
                " "
            }
        ));
        lines.push("  [x] Error Handling".to_string());
        let has_budget =
            self.monitor.safety.is_some() && self.nodes.iter().any(|n| n.tick_budget.is_some());
        lines.push(format!(
            "  [{}] budget Enforcement (RT nodes)",
            if has_budget { "x" } else { " " }
        ));

        // budget Stats
        if let Some(ref monitor) = self.monitor.safety {
            let stats = monitor.get_stats();
            if stats.budget_overruns() > 0 || stats.deadline_misses() > 0 {
                lines.push(thin_sep.to_string());
                lines.push("budget / Deadline Statistics:".to_string());
                if stats.budget_overruns() > 0 {
                    lines.push(format!(
                        "  [WARN] {} budget overruns",
                        stats.budget_overruns()
                    ));
                } else {
                    lines.push("  [OK] No budget overruns".to_string());
                }
                if stats.deadline_misses() > 0 {
                    lines.push(format!(
                        "  [WARN] {} deadline misses",
                        stats.deadline_misses()
                    ));
                } else {
                    lines.push("  [OK] No deadline misses".to_string());
                }
            }
        }

        lines
    }

    /// Build node health and RT degradation status lines.
    fn node_health_status_lines(&self) -> Vec<String> {
        use types::NodeHealthState;

        let thin_sep = "------------------------------------------------------------------";
        let mut lines = Vec::new();

        // Node Health
        if !self.nodes.is_empty() {
            lines.push(thin_sep.to_string());
            lines.push("Node Health:".to_string());

            let stopped_count = self.nodes.iter().filter(|n| n.is_stopped).count();
            let warning_count = self
                .nodes
                .iter()
                .filter(|n| n.health_state.load() == NodeHealthState::Warning)
                .count();
            let unhealthy_count = self
                .nodes
                .iter()
                .filter(|n| n.health_state.load() == NodeHealthState::Unhealthy)
                .count();
            let isolated_count = self
                .nodes
                .iter()
                .filter(|n| n.health_state.load() == NodeHealthState::Isolated)
                .count();

            let degraded = stopped_count + warning_count + unhealthy_count + isolated_count;
            if degraded == 0 {
                lines.push(format!("  [OK] All {} nodes healthy", self.nodes.len()));
            } else {
                lines.push(format!(
                    "  {} healthy, {} warning, {} unhealthy, {} isolated, {} stopped",
                    self.nodes.len() - degraded,
                    warning_count,
                    unhealthy_count,
                    isolated_count,
                    stopped_count,
                ));
                for node in &self.nodes {
                    let health = node.health_state.load();
                    if node.is_stopped || health != NodeHealthState::Healthy {
                        let label = if node.is_stopped {
                            "STOPPED".to_string()
                        } else {
                            health.to_string().to_uppercase()
                        };
                        lines.push(format!("    - {}: {}", node.name.as_ref(), label));
                    }
                }
            }
        }

        // Degradations
        if !self.rt.degradations.is_empty() {
            lines.push(thin_sep.to_string());
            lines.push("Degradations:".to_string());
            for deg in &self.rt.degradations {
                let severity_tag = match deg.severity {
                    DegradationSeverity::High => "[HIGH]",
                    DegradationSeverity::Medium => "[MED] ",
                    DegradationSeverity::Low => "[LOW] ",
                };
                lines.push(format!(
                    "  {} {:?}: {}",
                    severity_tag, deg.feature, deg.reason
                ));
            }
        }

        lines
    }

    // ============================================================================
    // OS Integration Methods (low-level, genuinely different from config)
    // ============================================================================

    /// Set OS-level scheduling priority using SCHED_FIFO (Linux RT-PREEMPT required).
    ///
    /// Delegates to [`rt::set_realtime_priority`].
    ///
    /// # Arguments
    /// * `priority` - Priority level (1-99, higher = more important)
    ///
    /// # Example
    /// ```ignore
    /// scheduler.set_os_priority(99)?;  // Highest priority
    /// ```
    #[doc(hidden)]
    pub fn set_os_priority(&self, priority: i32) -> crate::error::HorusResult<()> {
        if !(1..=99).contains(&priority) {
            return Err(crate::error::HorusError::config(
                "Priority must be between 1 and 99",
            ));
        }
        rt::set_realtime_priority(priority)?;
        print_line(&format!(
            "[OK] OS priority set to {} (SCHED_FIFO)",
            priority
        ));
        Ok(())
    }

    /// Pin scheduler to a specific CPU core (prevent context switches).
    ///
    /// Delegates to [`rt::set_thread_affinity`].
    ///
    /// # Example
    /// ```ignore
    /// scheduler.pin_to_cpu(7)?;
    /// ```
    #[doc(hidden)]
    pub fn pin_to_cpu(&self, cpu_id: usize) -> crate::error::HorusResult<()> {
        rt::set_thread_affinity(&[cpu_id])?;
        print_line(&format!("[OK] Scheduler pinned to CPU core {}", cpu_id));
        Ok(())
    }

    /// Lock all memory pages to prevent page faults (critical for <20μs latency).
    ///
    /// Delegates to [`rt::lock_all_memory`].
    ///
    /// # Example
    /// ```ignore
    /// scheduler.lock_memory()?;
    /// ```
    #[doc(hidden)]
    pub fn lock_memory(&self) -> crate::error::HorusResult<()> {
        rt::lock_all_memory()?;
        print_line("[OK] Memory locked (no page faults)");
        Ok(())
    }

    /// Pre-fault stack to prevent page faults during execution.
    ///
    /// Delegates to [`rt::prefault_stack`].
    ///
    /// # Example
    /// ```ignore
    /// scheduler.prefault_stack(8 * 1024 * 1024)?;  // 8MB stack
    /// ```
    #[doc(hidden)]
    pub fn prefault_stack(&self, stack_size: usize) -> crate::error::HorusResult<()> {
        rt::prefault_stack(stack_size)?;
        print_line(&format!(
            "[OK] Pre-faulted {} KB of stack",
            stack_size / 1024
        ));
        Ok(())
    }
}
