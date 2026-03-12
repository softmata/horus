// Scheduler configuration data structs

/// Timing configuration
#[doc(hidden)]
#[derive(Debug, Clone)]
pub struct TimingConfig {
    /// Global tick rate in Hz (default: 100)
    pub global_rate_hz: f64,
    /// Enable deterministic execution order (default: false).
    ///
    /// When true, all nodes tick sequentially on the main thread in
    /// registration order — no thread pools, no watcher threads, no executors.
    /// Designed for simulation and testing with `tick_once()`.
    pub deterministic_order: bool,
}

/// Real-time configuration for the scheduler thread.
///
/// These are **policy flags** that control scheduler behavior. The actual OS-level
/// syscalls (mlockall, sched_setscheduler, sched_setaffinity) are delegated to
/// [`RtConfig::apply()`](crate::core::rt_config::RtConfig) internally.
///
/// For standalone thread-level RT configuration (outside the scheduler), use
/// [`RtConfig`](crate::core::rt_config::RtConfig) directly.
#[doc(hidden)]
#[derive(Debug, Clone)]
pub struct RealTimeConfig {
    /// Watchdog timeout in milliseconds. 0 = disabled, >0 = enabled.
    /// When enabled, safety monitor is auto-created.
    pub watchdog_timeout_ms: u64,
    /// Maximum deadline misses before emergency stop (default: 100)
    pub max_deadline_misses: u64,
    /// Memory locking (mlockall)
    pub memory_locking: bool,
    /// Use real-time scheduling class (SCHED_FIFO/RR)
    pub rt_scheduling_class: bool,
}

/// Resource management configuration
#[doc(hidden)]
#[derive(Debug, Clone)]
pub struct ResourceConfig {
    /// CPU cores to use (None = all cores)
    pub cpu_cores: Option<Vec<usize>>,
}

/// Monitoring and telemetry configuration
#[doc(hidden)]
#[derive(Debug, Clone)]
pub struct MonitoringConfig {
    /// Black box buffer size in MB. 0 = disabled, >0 = enabled.
    pub black_box_size_mb: usize,
    /// Telemetry export endpoint (e.g., "udp://localhost:9999", "file:///var/log/metrics.json")
    pub telemetry_endpoint: Option<String>,
    /// Enable verbose logging from executor threads (budget warnings, pre/post-condition
    /// failures, startup/shutdown messages, etc.). Emergency-stop messages are always
    /// printed regardless of this setting. Default: true.
    pub verbose: bool,
}

/// Recording configuration for record/replay system (YAML-compatible)
#[doc(hidden)]
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

/// Internal scheduler configuration — use `Scheduler` builder methods instead.
///
/// ```rust,ignore
/// // Production: enable watchdog + blackbox
/// let scheduler = Scheduler::new()
///     .watchdog(500_u64.ms())
///     .blackbox(64)
///     .tick_rate(500_u64.hz());
///
/// // Lightweight: no watchdog or blackbox
/// let scheduler = Scheduler::new().tick_rate(500_u64.hz());
/// ```
#[doc(hidden)]
#[derive(Debug, Clone)]
pub struct SchedulerConfig {
    /// Timing configuration
    pub timing: TimingConfig,
    /// Real-time configuration
    pub realtime: RealTimeConfig,
    /// Resource management
    pub resources: ResourceConfig,
    /// Monitoring and telemetry
    pub monitoring: MonitoringConfig,
    /// Recording configuration for record/replay system
    pub recording: Option<RecordingConfigYaml>,
}

impl Default for SchedulerConfig {
    fn default() -> Self {
        Self {
            timing: TimingConfig {
                global_rate_hz: 100.0,
                deterministic_order: false,
            },
            realtime: RealTimeConfig {
                watchdog_timeout_ms: 0,
                max_deadline_misses: 100,
                memory_locking: false,
                rt_scheduling_class: false,
            },
            resources: ResourceConfig { cpu_cores: None },
            monitoring: MonitoringConfig {
                black_box_size_mb: 0,
                telemetry_endpoint: None,
                verbose: true,
            },
            recording: None,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use proptest::prelude::*;

    // ── Arbitrary strategies ────────────────────────────────────────────

    fn arb_timing_config() -> impl Strategy<Value = TimingConfig> {
        (0.001f64..10_000.0, any::<bool>()).prop_map(|(hz, deterministic)| TimingConfig {
            global_rate_hz: hz,
            deterministic_order: deterministic,
        })
    }

    fn arb_realtime_config() -> impl Strategy<Value = RealTimeConfig> {
        (
            0u64..60_000,
            1u64..10_000,
            any::<bool>(),
            any::<bool>(),
        )
            .prop_map(
                |(timeout, max_miss, memlock, rt_class)| {
                    RealTimeConfig {
                        watchdog_timeout_ms: timeout,
                        max_deadline_misses: max_miss,
                        memory_locking: memlock,
                        rt_scheduling_class: rt_class,
                    }
                },
            )
    }

    fn arb_monitoring_config() -> impl Strategy<Value = MonitoringConfig> {
        (0usize..1024, any::<bool>()).prop_map(
            |(bb_size, verbose)| MonitoringConfig {
                black_box_size_mb: bb_size,
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
            arb_realtime_config(),
            arb_monitoring_config(),
        )
            .prop_map(|(timing, realtime, monitoring)| SchedulerConfig {
                timing,
                realtime,
                resources: ResourceConfig { cpu_cores: None },
                monitoring,
                recording: None,
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

        /// SchedulerConfig: watchdog timeout is within valid range
        #[test]
        fn scheduler_config_watchdog_timeout_in_range(config in arb_scheduler_config()) {
            prop_assert!(config.realtime.watchdog_timeout_ms < 60_000,
                "Watchdog timeout out of range, got {}", config.realtime.watchdog_timeout_ms);
        }

        /// SchedulerConfig: max deadline misses must be positive
        #[test]
        fn scheduler_config_max_deadline_misses_positive(config in arb_scheduler_config()) {
            prop_assert!(config.realtime.max_deadline_misses > 0,
                "Max deadline misses must be > 0, got {}", config.realtime.max_deadline_misses);
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
            let config = TimingConfig { global_rate_hz: hz, deterministic_order: false };
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
        assert_eq!(config.timing.global_rate_hz, 100.0);
    }

    #[test]
    fn scheduler_config_default_no_recording() {
        let config = SchedulerConfig::default();
        assert!(config.recording.is_none());
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
        assert!(config.monitoring.verbose);
    }

    #[test]
    fn realtime_config_default_all_disabled() {
        let config = SchedulerConfig::default();
        assert_eq!(config.realtime.watchdog_timeout_ms, 0);
        assert!(!config.realtime.memory_locking);
        assert!(!config.realtime.rt_scheduling_class);
        assert_eq!(config.realtime.max_deadline_misses, 100);
    }

    // ── Edge Case & Boundary Tests ──────────────────────────────────────

    #[test]
    fn timing_config_very_low_rate() {
        let config = TimingConfig {
            global_rate_hz: 0.001,
            deterministic_order: false,
        };
        let period = 1.0 / config.global_rate_hz;
        assert!(period > 999.0, "Very low rate should produce long period");
    }

    #[test]
    fn timing_config_very_high_rate() {
        let config = TimingConfig {
            global_rate_hz: 100_000.0,
            deterministic_order: false,
        };
        let period = 1.0 / config.global_rate_hz;
        assert!(period < 0.001, "Very high rate should produce tiny period");
    }

    #[test]
    fn timing_config_exactly_one_hz() {
        let config = TimingConfig {
            global_rate_hz: 1.0,
            deterministic_order: false,
        };
        let period = 1.0 / config.global_rate_hz;
        assert!((period - 1.0).abs() < f64::EPSILON);
    }

    #[test]
    fn realtime_config_all_enabled_simultaneously() {
        let rt = RealTimeConfig {
            watchdog_timeout_ms: 500,
            max_deadline_misses: 10,
            memory_locking: true,
            rt_scheduling_class: true,
        };
        let default_rt = SchedulerConfig::default().realtime;
        assert!(rt.memory_locking);
        assert!(rt.rt_scheduling_class);
        assert_eq!(rt.watchdog_timeout_ms, 500);
        assert_eq!(rt.max_deadline_misses, 10);
        // Should differ from defaults
        assert_ne!(rt.watchdog_timeout_ms, default_rt.watchdog_timeout_ms);
        assert_ne!(rt.memory_locking, default_rt.memory_locking);
    }

    #[test]
    fn realtime_config_min_watchdog_timeout() {
        let rt = RealTimeConfig {
            watchdog_timeout_ms: 1,
            ..SchedulerConfig::default().realtime
        };
        assert_eq!(rt.watchdog_timeout_ms, 1);
        let default_rt = SchedulerConfig::default().realtime;
        assert_eq!(rt.max_deadline_misses, default_rt.max_deadline_misses);
    }

    #[test]
    fn realtime_config_large_max_deadline_misses() {
        let rt = RealTimeConfig {
            max_deadline_misses: u64::MAX,
            ..SchedulerConfig::default().realtime
        };
        assert_eq!(rt.max_deadline_misses, u64::MAX);
        // Default should be much smaller
        let default_rt = SchedulerConfig::default().realtime;
        assert!(
            default_rt.max_deadline_misses < u64::MAX,
            "Default max_deadline_misses should be less than MAX"
        );
    }

    #[test]
    fn monitoring_config_zero_blackbox_means_disabled() {
        let config = SchedulerConfig::default();
        assert_eq!(config.monitoring.black_box_size_mb, 0, "Default blackbox should be disabled (0 MB)");
    }

    #[test]
    fn monitoring_config_with_telemetry_endpoint() {
        let mon = MonitoringConfig {
            telemetry_endpoint: Some("udp://localhost:9999".to_string()),
            ..SchedulerConfig::default().monitoring
        };
        assert_eq!(
            mon.telemetry_endpoint.as_deref(),
            Some("udp://localhost:9999")
        );
        // Default should have no endpoint
        let default_mon = SchedulerConfig::default().monitoring;
        assert!(
            default_mon.telemetry_endpoint.is_none(),
            "Default should have no telemetry endpoint"
        );
    }

    #[test]
    fn monitoring_config_verbose_false() {
        let explicit_false = MonitoringConfig {
            verbose: false,
            ..SchedulerConfig::default().monitoring
        };
        assert!(!explicit_false.verbose);
        // Verify clone preserves the flag
        let cloned = explicit_false.clone();
        assert_eq!(explicit_false.verbose, cloned.verbose);
    }

    #[test]
    fn resource_config_specific_cpu_cores() {
        let res = ResourceConfig {
            cpu_cores: Some(vec![0, 2, 4]),
        };
        let cores = res.cpu_cores.as_ref().unwrap();
        assert_eq!(cores.len(), 3);
        // Cores should be sorted and unique (no duplicates)
        let mut sorted = cores.clone();
        sorted.sort();
        sorted.dedup();
        assert_eq!(cores.len(), sorted.len(), "CPU cores should be unique");
        // Default should have None
        let default_res = SchedulerConfig::default().resources;
        assert!(default_res.cpu_cores.is_none(), "Default should not pin CPUs");
    }

    #[test]
    fn resource_config_empty_cpu_cores() {
        let res = ResourceConfig {
            cpu_cores: Some(vec![]),
        };
        // Some(empty) is distinct from None
        assert!(res.cpu_cores.is_some());
        assert!(res.cpu_cores.as_ref().unwrap().is_empty());
        assert_ne!(res.cpu_cores, None, "Some(empty) should differ from None");
    }

    #[test]
    fn scheduler_config_with_recording_enabled() {
        let mut config = SchedulerConfig::default();
        config.recording = Some(RecordingConfigYaml::full());
        assert!(config.recording.is_some());
        assert!(config.recording.as_ref().unwrap().enabled);
    }

    #[test]
    fn recording_config_include_and_exclude_nodes() {
        let config = RecordingConfigYaml {
            enabled: true,
            include_nodes: vec!["sensor".to_string(), "motor".to_string()],
            exclude_nodes: vec!["logger".to_string()],
            ..Default::default()
        };
        assert_eq!(config.include_nodes.len(), 2);
        assert_eq!(config.exclude_nodes.len(), 1);
        // Include and exclude should not overlap (logical consistency)
        for inc in &config.include_nodes {
            assert!(
                !config.exclude_nodes.contains(inc),
                "Node '{}' is in both include and exclude",
                inc
            );
        }
        assert!(config.include_nodes.contains(&"sensor".to_string()));
        assert!(config.include_nodes.contains(&"motor".to_string()));
        assert!(config.exclude_nodes.contains(&"logger".to_string()));
    }

    #[test]
    fn recording_config_with_session_name() {
        let config = RecordingConfigYaml {
            session_name: Some("test_session_001".to_string()),
            ..Default::default()
        };
        assert_eq!(config.session_name.as_deref(), Some("test_session_001"));
        // Default should have no session name
        let default_config = RecordingConfigYaml::default();
        assert!(default_config.session_name.is_none(), "Default should have no session name");
    }

    #[test]
    fn recording_config_max_size_zero_means_unlimited() {
        let config = RecordingConfigYaml::default();
        assert_eq!(config.max_size_mb, 0);
        // Non-default should preserve the value
        let custom = RecordingConfigYaml {
            max_size_mb: 1024,
            ..Default::default()
        };
        assert_eq!(custom.max_size_mb, 1024);
        assert_ne!(custom.max_size_mb, config.max_size_mb);
    }

    #[test]
    fn scheduler_config_clone_independence() {
        let config = SchedulerConfig::default();
        let mut cloned = config.clone();
        cloned.timing.global_rate_hz = 999.0;
        // Original should be unmodified
        assert_eq!(config.timing.global_rate_hz, 100.0);
    }

    #[test]
    fn scheduler_config_debug_format() {
        let config = SchedulerConfig::default();
        let debug = format!("{:?}", config);
        assert!(debug.contains("SchedulerConfig"));
        assert!(debug.contains("global_rate_hz"));
        assert!(debug.contains("100"));
    }

    // ── Proptest: boundary value fuzzing ────────────────────────────────

    proptest! {
        #![proptest_config(ProptestConfig::with_cases(500))]

        /// RealTimeConfig: watchdog timeout round-trips correctly and other fields are preserved
        #[test]
        fn realtime_config_watchdog_roundtrip(timeout in 1u64..u64::MAX) {
            let default_rt = SchedulerConfig::default().realtime;
            let rt = RealTimeConfig {
                watchdog_timeout_ms: timeout,
                ..default_rt.clone()
            };
            prop_assert_eq!(rt.watchdog_timeout_ms, timeout);
            prop_assert_eq!(rt.max_deadline_misses, default_rt.max_deadline_misses);
        }

        /// RecordingConfig: max_size_mb round-trips and clone is independent
        #[test]
        fn recording_config_max_size_roundtrip(size in 0usize..1_000_000) {
            let config = RecordingConfigYaml {
                max_size_mb: size,
                ..Default::default()
            };
            prop_assert_eq!(config.max_size_mb, size);
            // Clone independence
            let mut cloned = config.clone();
            cloned.max_size_mb = size.wrapping_add(1);
            prop_assert_eq!(config.max_size_mb, size);
            prop_assert_ne!(config.max_size_mb, cloned.max_size_mb);
        }
    }
}
