// Scheduler configuration - preset factories and data structs

use super::deterministic::DeterministicConfig;

/// Timing configuration
#[derive(Debug, Clone)]
pub struct TimingConfig {
    /// Global tick rate in Hz (default: 60)
    pub global_rate_hz: f64,
}

/// Real-time configuration for the scheduler thread.
///
/// These are **policy flags** that control scheduler behavior. The actual OS-level
/// syscalls (mlockall, sched_setscheduler, sched_setaffinity) are delegated to
/// [`RtConfig::apply()`](crate::core::rt_config::RtConfig) internally.
///
/// For standalone thread-level RT configuration (outside the scheduler), use
/// [`RtConfig`](crate::core::rt_config::RtConfig) directly.
#[derive(Debug, Clone)]
pub struct RealTimeConfig {
    /// Enable WCET enforcement
    pub wcet_enforcement: bool,
    /// Enable deadline monitoring
    pub deadline_monitoring: bool,
    /// Enable watchdog timers
    pub watchdog_enabled: bool,
    /// Default watchdog timeout in milliseconds
    pub watchdog_timeout_ms: u64,
    /// Enable safety monitor
    pub safety_monitor: bool,
    /// Maximum deadline misses before emergency stop
    pub max_deadline_misses: u64,
    /// Memory locking (mlockall)
    pub memory_locking: bool,
    /// Use real-time scheduling class (SCHED_FIFO/RR)
    pub rt_scheduling_class: bool,
}

/// Resource management configuration
#[derive(Debug, Clone)]
pub struct ResourceConfig {
    /// CPU cores to use (None = all cores)
    pub cpu_cores: Option<Vec<usize>>,
    /// Enable NUMA awareness
    pub numa_aware: bool,
}

/// Monitoring and telemetry configuration
#[derive(Debug, Clone)]
pub struct MonitoringConfig {
    /// Enable runtime profiling
    pub profiling_enabled: bool,
    /// Metrics export interval in ms
    pub metrics_interval_ms: u64,
    /// Enable black box recording
    pub black_box_enabled: bool,
    /// Black box buffer size in MB
    pub black_box_size_mb: usize,
    /// How many WAL records to accumulate before flushing to the OS.
    ///
    /// Flushing the WAL `BufWriter` on every `record()` call causes thousands
    /// of `fsync`-equivalent syscalls per second at high scheduler frequencies,
    /// stalling the scheduler thread on disk I/O.  Batching amortises the cost:
    ///
    /// | Scheduler Hz | interval=1 (old) | interval=64 | interval=256 |
    /// |---|---|---|---|
    /// | 100 | 100 flushes/s | ≤2 flushes/s | ≤1 flush/2s |
    /// | 1000 | 1000 flushes/s | ≤16 flushes/s | ≤4 flushes/s |
    ///
    /// Default: 64 (a flush every ~640 ms at 100 Hz, ~64 ms at 1 kHz).
    /// Set to 1 to restore the legacy per-record flush behaviour.
    pub wal_flush_interval: usize,
    /// Telemetry export endpoint (e.g., "udp://localhost:9999", "file:///var/log/metrics.json")
    pub telemetry_endpoint: Option<String>,
    /// Enable verbose logging from executor threads (WCET warnings, pre/post-condition
    /// failures, startup/shutdown messages, etc.). Emergency-stop messages are always
    /// printed regardless of this setting. Default: true.
    pub verbose: bool,
}

/// Recording configuration for record/replay system (YAML-compatible)
#[derive(Debug, Clone)]
pub struct RecordingConfigYaml {
    /// Enable recording when scheduler starts
    pub enabled: bool,
    /// Session name for recordings (auto-generated if None)
    pub session_name: Option<String>,
    /// Enable zstd compression for recordings
    pub compress: bool,
    /// Recording interval in ticks (1 = every tick)
    pub interval: u32,
    /// Base directory for recordings (default: ~/.horus/recordings)
    pub output_dir: Option<String>,
    /// Maximum recording size in MB (0 = unlimited)
    pub max_size_mb: usize,
    /// Nodes to record (empty = all nodes)
    pub include_nodes: Vec<String>,
    /// Nodes to exclude from recording
    pub exclude_nodes: Vec<String>,
    /// Record input values
    pub record_inputs: bool,
    /// Record output values
    pub record_outputs: bool,
    /// Record timing information
    pub record_timing: bool,
}

impl Default for RecordingConfigYaml {
    fn default() -> Self {
        Self {
            enabled: false,
            session_name: None,
            compress: true,
            interval: 1,
            output_dir: None,
            max_size_mb: 0,
            include_nodes: vec![],
            exclude_nodes: vec![],
            record_inputs: true,
            record_outputs: true,
            record_timing: true,
        }
    }
}

impl RecordingConfigYaml {
    /// Create a recording config that records everything
    pub fn full() -> Self {
        Self {
            enabled: true,
            ..Default::default()
        }
    }

    /// Create a minimal recording config (outputs only)
    pub fn minimal() -> Self {
        Self {
            enabled: true,
            compress: true,
            interval: 10,
            max_size_mb: 50,
            record_inputs: false,
            record_timing: false,
            ..Default::default()
        }
    }
}

/// Internal scheduler configuration — use `Scheduler::new()` with builder methods instead.
///
/// ```rust,ignore
/// let scheduler = Scheduler::new()
///     .tick_hz(500.0)
///     .safety_monitor(true);
/// ```
#[derive(Debug, Clone)]
pub struct SchedulerConfig {
    /// Timing configuration
    pub timing: TimingConfig,
    /// Enable tier-based fault tolerance (circuit breaker, restart policies)
    ///
    /// When true, nodes use their tier's default FailurePolicy.
    /// When false, all nodes get FailurePolicy::Ignore.
    pub circuit_breaker: bool,
    /// Real-time configuration
    pub realtime: RealTimeConfig,
    /// Resource management
    pub resources: ResourceConfig,
    /// Monitoring and telemetry
    pub monitoring: MonitoringConfig,
    /// Recording configuration for record/replay system
    pub recording: Option<RecordingConfigYaml>,
    /// Deterministic execution configuration (virtual time, seeded RNG)
    pub deterministic: Option<DeterministicConfig>,
}

impl Default for SchedulerConfig {
    fn default() -> Self {
        Self {
            timing: TimingConfig {
                global_rate_hz: 60.0,
            },
            circuit_breaker: false,
            realtime: RealTimeConfig {
                wcet_enforcement: false,
                deadline_monitoring: false,
                watchdog_enabled: false,
                watchdog_timeout_ms: 1000,
                safety_monitor: false,
                max_deadline_misses: 100,
                memory_locking: false,
                rt_scheduling_class: false,
            },
            resources: ResourceConfig {
                cpu_cores: None,
                numa_aware: false,
            },
            monitoring: MonitoringConfig {
                profiling_enabled: false,
                metrics_interval_ms: 1000,
                black_box_enabled: false,
                black_box_size_mb: 0,
                wal_flush_interval: 64,
                telemetry_endpoint: None,
                verbose: true,
            },
            recording: None,
            deterministic: None,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use proptest::prelude::*;

    // ── Arbitrary strategies ────────────────────────────────────────────

    fn arb_timing_config() -> impl Strategy<Value = TimingConfig> {
        (0.001f64..10_000.0).prop_map(|hz| TimingConfig { global_rate_hz: hz })
    }

    fn arb_realtime_config() -> impl Strategy<Value = RealTimeConfig> {
        (
            any::<bool>(),
            any::<bool>(),
            any::<bool>(),
            1u64..60_000,
            any::<bool>(),
            1u64..10_000,
            any::<bool>(),
            any::<bool>(),
        )
            .prop_map(
                |(wcet, deadline, watchdog, timeout, safety, max_miss, memlock, rt_class)| {
                    RealTimeConfig {
                        wcet_enforcement: wcet,
                        deadline_monitoring: deadline,
                        watchdog_enabled: watchdog,
                        watchdog_timeout_ms: timeout,
                        safety_monitor: safety,
                        max_deadline_misses: max_miss,
                        memory_locking: memlock,
                        rt_scheduling_class: rt_class,
                    }
                },
            )
    }

    fn arb_monitoring_config() -> impl Strategy<Value = MonitoringConfig> {
        (
            any::<bool>(),
            1u64..30_000,
            any::<bool>(),
            0usize..1024,
            1usize..1024,
            any::<bool>(),
        )
            .prop_map(
                |(prof, interval, bb, bb_size, wal, verbose)| MonitoringConfig {
                    profiling_enabled: prof,
                    metrics_interval_ms: interval,
                    black_box_enabled: bb,
                    black_box_size_mb: bb_size,
                    wal_flush_interval: wal,
                    telemetry_endpoint: None,
                    verbose,
                },
            )
    }

    fn arb_recording_config() -> impl Strategy<Value = RecordingConfigYaml> {
        (
            any::<bool>(),
            any::<bool>(),
            1u32..1000,
            0usize..10_000,
            any::<bool>(),
            any::<bool>(),
            any::<bool>(),
        )
            .prop_map(
                |(enabled, compress, interval, max_size, inputs, outputs, timing)| {
                    RecordingConfigYaml {
                        enabled,
                        session_name: None,
                        compress,
                        interval,
                        output_dir: None,
                        max_size_mb: max_size,
                        include_nodes: vec![],
                        exclude_nodes: vec![],
                        record_inputs: inputs,
                        record_outputs: outputs,
                        record_timing: timing,
                    }
                },
            )
    }

    fn arb_scheduler_config() -> impl Strategy<Value = SchedulerConfig> {
        (
            arb_timing_config(),
            any::<bool>(),
            arb_realtime_config(),
            arb_monitoring_config(),
        )
            .prop_map(|(timing, cb, realtime, monitoring)| SchedulerConfig {
                timing,
                circuit_breaker: cb,
                realtime,
                resources: ResourceConfig {
                    cpu_cores: None,
                    numa_aware: false,
                },
                monitoring,
                recording: None,
                deterministic: None,
            })
    }

    // ── Property tests ──────────────────────────────────────────────────

    proptest! {
        #![proptest_config(ProptestConfig::with_cases(500))]

        /// SchedulerConfig: global tick rate must always be positive
        #[test]
        fn scheduler_config_tick_rate_positive(config in arb_scheduler_config()) {
            prop_assert!(config.timing.global_rate_hz > 0.0,
                "Tick rate must be positive, got {}", config.timing.global_rate_hz);
            prop_assert!(config.timing.global_rate_hz.is_finite(),
                "Tick rate must be finite, got {}", config.timing.global_rate_hz);
        }

        /// SchedulerConfig: watchdog timeout must be positive
        #[test]
        fn scheduler_config_watchdog_timeout_positive(config in arb_scheduler_config()) {
            prop_assert!(config.realtime.watchdog_timeout_ms > 0,
                "Watchdog timeout must be > 0, got {}", config.realtime.watchdog_timeout_ms);
        }

        /// SchedulerConfig: max deadline misses must be positive
        #[test]
        fn scheduler_config_max_deadline_misses_positive(config in arb_scheduler_config()) {
            prop_assert!(config.realtime.max_deadline_misses > 0,
                "Max deadline misses must be > 0, got {}", config.realtime.max_deadline_misses);
        }

        /// SchedulerConfig: metrics interval must be positive
        #[test]
        fn scheduler_config_metrics_interval_positive(config in arb_scheduler_config()) {
            prop_assert!(config.monitoring.metrics_interval_ms > 0,
                "Metrics interval must be > 0, got {}", config.monitoring.metrics_interval_ms);
        }

        /// SchedulerConfig: WAL flush interval must be positive
        #[test]
        fn scheduler_config_wal_flush_positive(config in arb_scheduler_config()) {
            prop_assert!(config.monitoring.wal_flush_interval > 0,
                "WAL flush interval must be > 0, got {}", config.monitoring.wal_flush_interval);
        }

        /// SchedulerConfig: clone produces identical config
        #[test]
        fn scheduler_config_clone_preserves_values(config in arb_scheduler_config()) {
            let cloned = config.clone();
            // Compare via Debug since no PartialEq
            prop_assert_eq!(format!("{:?}", config), format!("{:?}", cloned));
        }

        /// RecordingConfig: interval is always positive
        #[test]
        fn recording_config_interval_positive(config in arb_recording_config()) {
            prop_assert!(config.interval > 0,
                "Recording interval must be > 0, got {}", config.interval);
        }

        /// RecordingConfig: clone preserves all fields
        #[test]
        fn recording_config_clone_preserves_values(config in arb_recording_config()) {
            let cloned = config.clone();
            prop_assert_eq!(format!("{:?}", config), format!("{:?}", cloned));
        }

        /// TimingConfig: tick period derivation is consistent
        #[test]
        fn timing_config_period_consistency(hz in 0.001f64..10_000.0) {
            let config = TimingConfig { global_rate_hz: hz };
            let period_secs = 1.0 / config.global_rate_hz;
            let derived_hz = 1.0 / period_secs;
            // Allow small floating point error
            let diff = (derived_hz - hz).abs();
            prop_assert!(diff < 1e-10 * hz.abs(),
                "Hz→period→Hz roundtrip: {} → {} → {}, diff={}",
                hz, period_secs, derived_hz, diff);
        }
    }

    // ── Non-proptest unit tests for defaults and factories ──────────────

    #[test]
    fn scheduler_config_default_has_positive_rate() {
        let config = SchedulerConfig::default();
        assert!(config.timing.global_rate_hz > 0.0);
        assert_eq!(config.timing.global_rate_hz, 60.0);
    }

    #[test]
    fn scheduler_config_default_circuit_breaker_off() {
        let config = SchedulerConfig::default();
        assert!(!config.circuit_breaker);
    }

    #[test]
    fn scheduler_config_default_no_recording() {
        let config = SchedulerConfig::default();
        assert!(config.recording.is_none());
    }

    #[test]
    fn scheduler_config_default_no_deterministic() {
        let config = SchedulerConfig::default();
        assert!(config.deterministic.is_none());
    }

    #[test]
    fn recording_config_full_is_enabled() {
        let config = RecordingConfigYaml::full();
        assert!(config.enabled);
        assert!(config.record_inputs);
        assert!(config.record_outputs);
        assert!(config.record_timing);
    }

    #[test]
    fn recording_config_minimal_skips_inputs_and_timing() {
        let config = RecordingConfigYaml::minimal();
        assert!(config.enabled);
        assert!(!config.record_inputs);
        assert!(!config.record_timing);
        assert!(config.record_outputs);
        assert!(config.compress);
        assert_eq!(config.interval, 10);
        assert_eq!(config.max_size_mb, 50);
    }

    #[test]
    fn recording_config_default_not_enabled() {
        let config = RecordingConfigYaml::default();
        assert!(!config.enabled);
        assert_eq!(config.interval, 1);
    }

    #[test]
    fn monitoring_config_default_values() {
        let config = SchedulerConfig::default();
        assert_eq!(config.monitoring.metrics_interval_ms, 1000);
        assert_eq!(config.monitoring.wal_flush_interval, 64);
        assert!(config.monitoring.verbose);
    }

    #[test]
    fn realtime_config_default_all_disabled() {
        let config = SchedulerConfig::default();
        assert!(!config.realtime.wcet_enforcement);
        assert!(!config.realtime.deadline_monitoring);
        assert!(!config.realtime.watchdog_enabled);
        assert!(!config.realtime.safety_monitor);
        assert!(!config.realtime.memory_locking);
        assert!(!config.realtime.rt_scheduling_class);
    }
}
