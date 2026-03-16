//! Configuration application methods for `Scheduler`.
//!
//! Contains methods that apply configuration during init: `apply_config`,
//! `apply_safety_config`, `apply_rt_optimizations`, `apply_monitoring_config`,
//! `apply_recording_config`, and `finalize_config`.

use super::*;

// Re-import scheduling-level modules
use crate::scheduling::blackbox;
use crate::scheduling::config as sched_config;
use crate::scheduling::telemetry;
use crate::scheduling::types;

impl Scheduler {
    /// Apply a full configuration struct. Used internally by horus_py.
    #[doc(hidden)]
    pub fn apply_config(&mut self, config: sched_config::SchedulerConfig) {
        // Global tick rate
        let rate_hz =
            if config.timing.global_rate_hz.is_finite() && config.timing.global_rate_hz > 0.0 {
                config.timing.global_rate_hz
            } else {
                100.0 // Safe fallback for invalid rate
            };
        self.tick.period = Duration::from_micros((1_000_000.0 / rate_hz) as u64);

        // RT safety and OS-level optimizations
        self.apply_safety_config(&config.realtime);
        self.apply_rt_optimizations(&config.realtime, &config.resources);

        // Resource configuration
        if let Some(ref cores) = config.resources.cpu_cores {
            print_line(&format!("CPU cores configuration: {:?}", cores));
        }

        // Monitoring (profiling, blackbox, telemetry)
        self.apply_monitoring_config(&config.monitoring);

        // Recording
        if let Some(ref recording_yaml) = config.recording {
            if recording_yaml.enabled {
                self.apply_recording_config(recording_yaml);
            }
        }
    }

    /// Configure the safety monitor for RT nodes (watchdogs, budget, deadlines).
    pub(super) fn apply_safety_config(&mut self, rt: &sched_config::RealTimeConfig) {
        let has_rt_nodes = self.nodes.iter().any(|n| n.is_rt_node);
        let watchdog_active = rt.watchdog_timeout_ms > 0;
        if watchdog_active || has_rt_nodes {
            let monitor = SafetyMonitor::new(rt.max_deadline_misses);

            let global_watchdog_timeout = rt.watchdog_timeout_ms.ms();

            for registered in self.nodes.iter() {
                // Per-node watchdog: use node-specific timeout if set, otherwise global
                let node_timeout = registered.node_watchdog.unwrap_or(global_watchdog_timeout);

                if registered.is_rt_node {
                    if watchdog_active || registered.node_watchdog.is_some() {
                        monitor.add_critical_node(registered.name.to_string(), node_timeout);
                    }

                    if let Some(budget) = registered.tick_budget {
                        monitor.set_tick_budget(registered.name.to_string(), budget);
                    }
                } else if registered.node_watchdog.is_some() {
                    // Non-RT node with explicit per-node watchdog
                    monitor.add_critical_node(registered.name.to_string(), node_timeout);
                }
            }

            self.monitor.safety = Some(Arc::new(monitor));
            print_line("Safety monitor configured for RT nodes");
        }
    }

    /// Apply OS-level RT optimizations (memory locking, scheduling, affinity, NUMA).
    pub(super) fn apply_rt_optimizations(
        &mut self,
        rt: &sched_config::RealTimeConfig,
        resources: &sched_config::ResourceConfig,
    ) {
        use crate::core::rt_config::{RtApplyResult, RtConfig, RtScheduler};

        let has_rt_features =
            rt.memory_locking || rt.rt_scheduling_class || resources.cpu_cores.is_some();

        if has_rt_features {
            let mut builder = RtConfig::builder()
                .memory_locked(rt.memory_locking)
                .warn_on_degradation(true);

            if rt.rt_scheduling_class {
                builder = builder.scheduler(RtScheduler::Fifo).priority(50);
            }

            if let Some(ref cores) = resources.cpu_cores {
                builder = builder.cpu_affinity(cores);
            }

            let rt_config = builder.build();
            match rt_config.apply() {
                Ok(RtApplyResult::FullSuccess) => {
                    if rt.memory_locking {
                        print_line("[SCHEDULER] Memory locked (mlockall)");
                    }
                    if rt.rt_scheduling_class {
                        print_line("[SCHEDULER] RT scheduling enabled (SCHED_FIFO, priority 50)");
                    }
                    if let Some(ref cores) = resources.cpu_cores {
                        print_line(&format!(
                            "[SCHEDULER] CPU affinity set to cores {:?}",
                            cores
                        ));
                    }
                }
                Ok(RtApplyResult::Degraded(degradations)) => {
                    for d in &degradations {
                        print_line(&format!("[SCHEDULER] RT degraded: {:?}", d));
                    }
                }
                Err(e) => {
                    print_line(&format!("[SCHEDULER] RT config error: {}", e));
                }
            }
        }
    }

    /// Apply monitoring configuration (profiling, blackbox, telemetry).
    pub(super) fn apply_monitoring_config(&mut self, monitoring: &sched_config::MonitoringConfig) {
        // Profiling is always on (negligible overhead: ~microsecond per tick via Welford's algorithm)
        self.monitor.profiler.lock().unwrap().enable();

        // Black box flight recorder
        if monitoring.black_box_size_mb > 0 {
            let bb_dir = self.monitor.working_dir.join(".horus").join("blackbox");
            let mut bb =
                blackbox::BlackBox::new(monitoring.black_box_size_mb).with_path(bb_dir);
            bb.record(blackbox::BlackBoxEvent::SchedulerStart {
                name: self.scheduler_name.clone(),
                node_count: self.nodes.len(),
                config: format!(
                    "rate={}Hz",
                    1_000_000.0 / self.tick.period.as_micros() as f64
                ),
            });
            self.monitor.blackbox = Some(Arc::new(Mutex::new(bb)));
            #[cfg(feature = "blackbox")]
            print_line(&format!(
                "[SCHEDULER] Black box enabled ({}MB buffer)",
                monitoring.black_box_size_mb
            ));
            #[cfg(not(feature = "blackbox"))]
            print_line(&format!(
                "[SCHEDULER] Black box configured ({}MB) but feature disabled — data will not be recorded",
                monitoring.black_box_size_mb
            ));
        }

        // Telemetry export
        if let Some(ref endpoint_str) = monitoring.telemetry_endpoint {
            let endpoint = telemetry::TelemetryEndpoint::from_string(endpoint_str);
            let interval_ms = 1000u64;
            let mut tm = telemetry::TelemetryManager::new(endpoint, interval_ms);
            tm.set_scheduler_name(&self.scheduler_name);
            self.monitor.telemetry = Some(tm);
            #[cfg(feature = "telemetry")]
            print_line(&format!(
                "[SCHEDULER] Telemetry enabled (endpoint: {})",
                endpoint_str
            ));
            #[cfg(not(feature = "telemetry"))]
            print_line(&format!(
                "[SCHEDULER] Telemetry configured ({}) but feature disabled — metrics will not be exported",
                endpoint_str
            ));
        }
    }

    /// Apply recording configuration for the record/replay system.
    pub(super) fn apply_recording_config(&mut self, recording_yaml: &sched_config::RecordingConfigYaml) {
        let recording_config = RecordingConfig::from(recording_yaml.clone());
        let session_name = recording_config.session_name.clone();

        // Generate unique scheduler ID
        let scheduler_id = format!(
            "{:x}{:x}",
            std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .map(|d| d.as_nanos() as u64)
                .unwrap_or(0),
            std::process::id() as u64
        );

        print_line(&format!(
            "[SCHEDULER] Recording enabled (session: {}, compress: {})",
            session_name, recording_yaml.compress
        ));

        self.recording = Some(RecordingState {
            config: recording_config,
            scheduler_recording: SchedulerRecording::new(&scheduler_id, &session_name),
        });
    }

    /// Apply the deferred `pending_config` before the tick loop starts.
    ///
    /// Called once from `run_with_filter()`. This is better than eagerly applying
    /// config in builders because `finalize_config()` runs AFTER all nodes are added,
    /// so `apply_safety_config` correctly sees all nodes.
    pub(super) fn finalize_config(&mut self) {
        let config = self.pending_config.clone();

        // Apply tick period from deferred config
        if config.timing.global_rate_hz.is_finite() && config.timing.global_rate_hz > 0.0 {
            self.tick.period =
                Duration::from_micros((1_000_000.0 / config.timing.global_rate_hz) as u64);
        }

        // Auto-derive: if any node's rate exceeds the global tick rate, bump it up.
        // This ensures the scheduler is always fast enough for its fastest node,
        // even when tick_hz() wasn't explicitly set high enough.
        self.adjust_tick_period_for_node_rates();

        // Safety config runs here so it sees ALL nodes (added after builder methods)
        self.apply_safety_config(&config.realtime);

        // Auto-enable mlockall if: RT nodes present, system permits it, and user didn't
        // explicitly disable it. This prevents 10-100ms page fault spikes under memory pressure.
        let has_rt_nodes = self
            .nodes
            .iter()
            .any(|n| matches!(n.execution_class, types::ExecutionClass::Rt));
        let mut rt_config = config.realtime.clone();
        if has_rt_nodes && !rt_config.memory_locking {
            if let Some(ref caps) = self.rt.capabilities {
                if caps.mlockall_permitted {
                    rt_config.memory_locking = true;
                    self.pending_config.realtime.memory_locking = true;
                }
            }
        }

        // Env var: HORUS_RT_CORES — comma-separated core IDs (e.g. "2,3")
        // Only applies if no cpu_cores already configured.
        if self.pending_config.resources.cpu_cores.is_none() {
            if let Ok(cores_str) = std::env::var("HORUS_RT_CORES") {
                let cores: Vec<usize> = cores_str
                    .split(',')
                    .filter_map(|s| s.trim().parse::<usize>().ok())
                    .collect();
                if !cores.is_empty() {
                    self.pending_config.resources.cpu_cores = Some(cores);
                }
            }
        }

        // Re-read resources after env var override
        let resources = self.pending_config.resources.clone();
        self.apply_rt_optimizations(&rt_config, &resources);

        // Env var: HORUS_TELEMETRY_ENDPOINT — UDP or file URI (e.g. "udp://localhost:9999")
        if self.pending_config.monitoring.telemetry_endpoint.is_none() {
            if let Ok(endpoint) = std::env::var("HORUS_TELEMETRY_ENDPOINT") {
                if !endpoint.is_empty() {
                    self.pending_config.monitoring.telemetry_endpoint = Some(endpoint);
                }
            }
        }

        // Re-read monitoring config after env var overrides
        let monitoring_config = self.pending_config.monitoring.clone();

        // Only apply monitoring if not already set by eager builders (with_blackbox, etc.)
        if self.monitor.blackbox.is_none() {
            self.apply_monitoring_config(&monitoring_config);
        } else {
            // Blackbox already exists (from .blackbox() builder), only apply profiling/telemetry
            if monitoring_config.verbose {
                self.monitor.profiler.lock().unwrap().enable();
            }
            if let Some(ref endpoint_str) = monitoring_config.telemetry_endpoint {
                if self.monitor.telemetry.is_none() {
                    let endpoint = telemetry::TelemetryEndpoint::from_string(endpoint_str);
                    let mut tm = telemetry::TelemetryManager::new(endpoint, 1000u64);
                    tm.set_scheduler_name(&self.scheduler_name);
                    self.monitor.telemetry = Some(tm);
                }
            }
        }

        if let Some(ref recording_yaml) = config.recording {
            if recording_yaml.enabled && self.recording.is_none() {
                self.apply_recording_config(recording_yaml);
            }
        }
    }
}
