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
        (0u64..60_000, 1u64..10_000, any::<bool>(), any::<bool>()).prop_map(
            |(timeout, max_miss, memlock, rt_class)| RealTimeConfig {
                watchdog_timeout_ms: timeout,
                max_deadline_misses: max_miss,
                memory_locking: memlock,
                rt_scheduling_class: rt_class,
            },
        )
    }

    fn arb_monitoring_config() -> impl Strategy<Value = MonitoringConfig> {
        (0usize..1024, any::<bool>()).prop_map(|(bb_size, verbose)| MonitoringConfig {
            black_box_size_mb: bb_size,
            telemetry_endpoint: None,
            verbose,
        })
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
        assert_eq!(
            config.monitoring.black_box_size_mb, 0,
            "Default blackbox should be disabled (0 MB)"
        );
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
        assert!(
            default_res.cpu_cores.is_none(),
            "Default should not pin CPUs"
        );
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
        assert!(
            default_config.session_name.is_none(),
            "Default should have no session name"
        );
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

    // ========================================================================
    // Deterministic negative tests — edge cases and invalid inputs
    // ========================================================================

    #[test]
    fn test_timing_config_zero_rate_uses_default() {
        let config = TimingConfig {
            global_rate_hz: 0.0,
            deterministic_order: false,
        };
        // Zero rate is invalid — scheduler should fallback to safe default
        // This tests that the config struct accepts it (validation happens at scheduler level)
        assert_eq!(config.global_rate_hz, 0.0);
    }

    #[test]
    fn test_timing_config_negative_rate() {
        let config = TimingConfig {
            global_rate_hz: -100.0,
            deterministic_order: false,
        };
        assert!(config.global_rate_hz < 0.0);
    }

    #[test]
    fn test_timing_config_nan_rate() {
        let config = TimingConfig {
            global_rate_hz: f64::NAN,
            deterministic_order: false,
        };
        assert!(config.global_rate_hz.is_nan());
    }

    #[test]
    fn test_timing_config_infinity_rate() {
        let config = TimingConfig {
            global_rate_hz: f64::INFINITY,
            deterministic_order: false,
        };
        assert!(config.global_rate_hz.is_infinite());
    }

    #[test]
    fn test_realtime_config_zero_watchdog() {
        let config = RealTimeConfig {
            watchdog_timeout_ms: 0,
            max_deadline_misses: 100,
            memory_locking: false,
            rt_scheduling_class: false,
        };
        // Zero watchdog means disabled
        assert_eq!(config.watchdog_timeout_ms, 0);
    }

    #[test]
    fn test_realtime_config_zero_max_misses() {
        let config = RealTimeConfig {
            watchdog_timeout_ms: 100,
            max_deadline_misses: 0,
            memory_locking: false,
            rt_scheduling_class: false,
        };
        // Zero max misses means emergency stop on first miss
        assert_eq!(config.max_deadline_misses, 0);
    }

    #[test]
    fn test_resource_config_empty_cores() {
        let config = ResourceConfig {
            cpu_cores: Some(vec![]),
        };
        // Empty core list should be treated same as None
        assert!(config.cpu_cores.as_ref().unwrap().is_empty());
    }

    #[test]
    fn test_monitoring_config_zero_blackbox() {
        let config = MonitoringConfig {
            black_box_size_mb: 0,
            telemetry_endpoint: None,
            verbose: false,
        };
        assert_eq!(config.black_box_size_mb, 0);
    }

    #[test]
    fn test_scheduler_config_defaults_are_sane() {
        let config = SchedulerConfig::default();
        assert!(config.timing.global_rate_hz > 0.0, "default rate must be positive");
        assert!(config.timing.global_rate_hz <= 10_000.0, "default rate must be reasonable");
        assert!(!config.timing.deterministic_order, "default should not be deterministic");
        assert_eq!(config.realtime.watchdog_timeout_ms, 0, "default watchdog should be disabled");
        assert!(config.realtime.max_deadline_misses > 0, "default max misses must be positive");
    }

    #[test]
    fn test_recording_config_defaults_are_sane() {
        let config = RecordingConfigYaml::default();
        assert!(!config.enabled, "recording should be off by default");
        // max_size_mb = 0 means unlimited — valid default
        assert_eq!(config.max_size_mb, 0, "default max size should be 0 (unlimited)");
        assert!(config.compress, "compression should be on by default");
        assert_eq!(config.interval, 1, "default interval should be 1");
    }

    // ========================================================================
    // Edge case & combination tests
    // ========================================================================

    #[test]
    fn scheduler_config_all_features_enabled() {
        // Monitoring + profiling (telemetry) + blackbox + deterministic + RT all at once
        let config = SchedulerConfig {
            timing: TimingConfig {
                global_rate_hz: 1000.0,
                deterministic_order: true,
            },
            realtime: RealTimeConfig {
                watchdog_timeout_ms: 500,
                max_deadline_misses: 5,
                memory_locking: true,
                rt_scheduling_class: true,
            },
            resources: ResourceConfig {
                cpu_cores: Some(vec![0, 1]),
            },
            monitoring: MonitoringConfig {
                black_box_size_mb: 128,
                telemetry_endpoint: Some("udp://10.0.0.1:9999".to_string()),
                verbose: true,
            },
            recording: Some(RecordingConfigYaml::full()),
        };
        // All features should coexist without contradiction
        assert!(config.timing.deterministic_order);
        assert!(config.realtime.memory_locking);
        assert!(config.realtime.rt_scheduling_class);
        assert_eq!(config.monitoring.black_box_size_mb, 128);
        assert!(config.monitoring.telemetry_endpoint.is_some());
        assert!(config.recording.is_some());
        assert!(config.recording.as_ref().unwrap().enabled);
        assert_eq!(config.resources.cpu_cores.as_ref().unwrap(), &[0, 1]);
    }

    #[test]
    fn scheduler_config_all_fields_at_extreme_maximums() {
        let config = SchedulerConfig {
            timing: TimingConfig {
                global_rate_hz: f64::MAX,
                deterministic_order: true,
            },
            realtime: RealTimeConfig {
                watchdog_timeout_ms: u64::MAX,
                max_deadline_misses: u64::MAX,
                memory_locking: true,
                rt_scheduling_class: true,
            },
            resources: ResourceConfig {
                cpu_cores: Some(vec![usize::MAX]),
            },
            monitoring: MonitoringConfig {
                black_box_size_mb: usize::MAX,
                telemetry_endpoint: Some(String::new()),
                verbose: true,
            },
            recording: Some(RecordingConfigYaml {
                enabled: true,
                session_name: Some(String::new()),
                compress: true,
                interval: u32::MAX,
                output_dir: Some(String::new()),
                max_size_mb: usize::MAX,
                include_nodes: vec![],
                exclude_nodes: vec![],
                record_inputs: true,
                record_outputs: true,
                record_timing: true,
            }),
        };
        assert_eq!(config.timing.global_rate_hz, f64::MAX);
        assert_eq!(config.realtime.watchdog_timeout_ms, u64::MAX);
        assert_eq!(config.realtime.max_deadline_misses, u64::MAX);
        assert_eq!(config.monitoring.black_box_size_mb, usize::MAX);
        assert_eq!(config.recording.as_ref().unwrap().interval, u32::MAX);
        assert_eq!(config.recording.as_ref().unwrap().max_size_mb, usize::MAX);
        assert_eq!(config.resources.cpu_cores.as_ref().unwrap(), &[usize::MAX]);
    }

    #[test]
    fn scheduler_config_all_fields_at_minimums() {
        let config = SchedulerConfig {
            timing: TimingConfig {
                global_rate_hz: f64::MIN_POSITIVE,
                deterministic_order: false,
            },
            realtime: RealTimeConfig {
                watchdog_timeout_ms: 0,
                max_deadline_misses: 0,
                memory_locking: false,
                rt_scheduling_class: false,
            },
            resources: ResourceConfig { cpu_cores: None },
            monitoring: MonitoringConfig {
                black_box_size_mb: 0,
                telemetry_endpoint: None,
                verbose: false,
            },
            recording: None,
        };
        assert!(config.timing.global_rate_hz > 0.0);
        assert!(config.timing.global_rate_hz < 1e-300);
        assert_eq!(config.realtime.watchdog_timeout_ms, 0);
        assert_eq!(config.realtime.max_deadline_misses, 0);
        assert!(!config.monitoring.verbose);
        assert!(config.recording.is_none());
    }

    #[test]
    fn scheduler_config_deterministic_with_rt_flags() {
        // Deterministic mode + RT scheduling is a potentially conflicting combo:
        // deterministic runs all nodes on main thread, but RT flags still set.
        // Config should accept this — validation is scheduler-level, not config-level.
        let config = SchedulerConfig {
            timing: TimingConfig {
                global_rate_hz: 100.0,
                deterministic_order: true,
            },
            realtime: RealTimeConfig {
                watchdog_timeout_ms: 500,
                max_deadline_misses: 10,
                memory_locking: true,
                rt_scheduling_class: true,
            },
            ..SchedulerConfig::default()
        };
        assert!(config.timing.deterministic_order);
        assert!(config.realtime.rt_scheduling_class);
        assert!(config.realtime.memory_locking);
    }

    #[test]
    fn scheduler_config_watchdog_enabled_without_rt_class() {
        // Watchdog without RT scheduling — valid for soft-RT use cases
        let config = SchedulerConfig {
            realtime: RealTimeConfig {
                watchdog_timeout_ms: 1000,
                max_deadline_misses: 50,
                memory_locking: false,
                rt_scheduling_class: false,
            },
            ..SchedulerConfig::default()
        };
        assert!(config.realtime.watchdog_timeout_ms > 0);
        assert!(!config.realtime.rt_scheduling_class);
    }

    #[test]
    fn scheduler_config_rt_class_without_watchdog() {
        // RT scheduling but no watchdog — valid for trusted code
        let config = SchedulerConfig {
            realtime: RealTimeConfig {
                watchdog_timeout_ms: 0,
                max_deadline_misses: 100,
                memory_locking: true,
                rt_scheduling_class: true,
            },
            ..SchedulerConfig::default()
        };
        assert_eq!(config.realtime.watchdog_timeout_ms, 0);
        assert!(config.realtime.rt_scheduling_class);
    }

    #[test]
    fn timing_config_subnormal_rate() {
        // Subnormal float — extremely small but not zero
        let rate = 5e-324_f64; // smallest positive f64
        let config = TimingConfig {
            global_rate_hz: rate,
            deterministic_order: false,
        };
        assert!(config.global_rate_hz > 0.0);
        assert!(config.global_rate_hz.is_normal() || config.global_rate_hz.is_subnormal());
    }

    #[test]
    fn timing_config_negative_infinity_rate() {
        let config = TimingConfig {
            global_rate_hz: f64::NEG_INFINITY,
            deterministic_order: false,
        };
        assert!(config.global_rate_hz.is_infinite());
        assert!(config.global_rate_hz < 0.0);
    }

    #[test]
    fn timing_config_deterministic_clone_independence() {
        let config = TimingConfig {
            global_rate_hz: 50.0,
            deterministic_order: true,
        };
        let mut cloned = config.clone();
        cloned.deterministic_order = false;
        cloned.global_rate_hz = 200.0;
        // Original must be unchanged
        assert!(config.deterministic_order);
        assert_eq!(config.global_rate_hz, 50.0);
    }

    #[test]
    fn timing_config_debug_contains_all_fields() {
        let config = TimingConfig {
            global_rate_hz: 42.5,
            deterministic_order: true,
        };
        let debug = format!("{:?}", config);
        assert!(debug.contains("TimingConfig"));
        assert!(debug.contains("42.5"));
        assert!(debug.contains("true"));
        assert!(debug.contains("global_rate_hz"));
        assert!(debug.contains("deterministic_order"));
    }

    #[test]
    fn realtime_config_debug_contains_all_fields() {
        let rt = RealTimeConfig {
            watchdog_timeout_ms: 777,
            max_deadline_misses: 33,
            memory_locking: true,
            rt_scheduling_class: false,
        };
        let debug = format!("{:?}", rt);
        assert!(debug.contains("RealTimeConfig"));
        assert!(debug.contains("777"));
        assert!(debug.contains("33"));
        assert!(debug.contains("memory_locking"));
        assert!(debug.contains("rt_scheduling_class"));
    }

    #[test]
    fn realtime_config_clone_independence() {
        let rt = RealTimeConfig {
            watchdog_timeout_ms: 200,
            max_deadline_misses: 50,
            memory_locking: true,
            rt_scheduling_class: true,
        };
        let mut cloned = rt.clone();
        cloned.watchdog_timeout_ms = 999;
        cloned.memory_locking = false;
        // Original preserved
        assert_eq!(rt.watchdog_timeout_ms, 200);
        assert!(rt.memory_locking);
    }

    #[test]
    fn monitoring_config_debug_contains_all_fields() {
        let mon = MonitoringConfig {
            black_box_size_mb: 64,
            telemetry_endpoint: Some("file:///var/log/metrics.json".to_string()),
            verbose: false,
        };
        let debug = format!("{:?}", mon);
        assert!(debug.contains("MonitoringConfig"));
        assert!(debug.contains("64"));
        assert!(debug.contains("file:///var/log/metrics.json"));
        assert!(debug.contains("verbose"));
    }

    #[test]
    fn monitoring_config_clone_independence() {
        let mon = MonitoringConfig {
            black_box_size_mb: 32,
            telemetry_endpoint: Some("udp://host:1234".to_string()),
            verbose: true,
        };
        let mut cloned = mon.clone();
        cloned.black_box_size_mb = 0;
        cloned.telemetry_endpoint = None;
        cloned.verbose = false;
        // Original preserved
        assert_eq!(mon.black_box_size_mb, 32);
        assert!(mon.telemetry_endpoint.is_some());
        assert!(mon.verbose);
    }

    #[test]
    fn monitoring_config_empty_telemetry_endpoint() {
        // Some("") is distinct from None — an empty string endpoint
        let mon = MonitoringConfig {
            black_box_size_mb: 0,
            telemetry_endpoint: Some(String::new()),
            verbose: true,
        };
        assert!(mon.telemetry_endpoint.is_some());
        assert!(mon.telemetry_endpoint.as_ref().unwrap().is_empty());
    }

    #[test]
    fn resource_config_single_core() {
        let res = ResourceConfig {
            cpu_cores: Some(vec![0]),
        };
        assert_eq!(res.cpu_cores.as_ref().unwrap().len(), 1);
        assert_eq!(res.cpu_cores.as_ref().unwrap()[0], 0);
    }

    #[test]
    fn resource_config_duplicate_cores() {
        // Config struct doesn't enforce uniqueness — that's scheduler-level validation
        let res = ResourceConfig {
            cpu_cores: Some(vec![2, 2, 2]),
        };
        assert_eq!(res.cpu_cores.as_ref().unwrap().len(), 3);
    }

    #[test]
    fn resource_config_many_cores() {
        let cores: Vec<usize> = (0..128).collect();
        let res = ResourceConfig {
            cpu_cores: Some(cores),
        };
        assert_eq!(res.cpu_cores.as_ref().unwrap().len(), 128);
        assert_eq!(res.cpu_cores.as_ref().unwrap()[127], 127);
    }

    #[test]
    fn resource_config_clone_independence() {
        let res = ResourceConfig {
            cpu_cores: Some(vec![0, 1, 2]),
        };
        let mut cloned = res.clone();
        cloned.cpu_cores.as_mut().unwrap().push(3);
        // Original unchanged
        assert_eq!(res.cpu_cores.as_ref().unwrap().len(), 3);
    }

    #[test]
    fn resource_config_debug_format() {
        let res = ResourceConfig {
            cpu_cores: Some(vec![4, 5]),
        };
        let debug = format!("{:?}", res);
        assert!(debug.contains("ResourceConfig"));
        assert!(debug.contains("4"));
        assert!(debug.contains("5"));
    }

    #[test]
    fn recording_config_full_inherits_defaults_except_enabled() {
        let full = RecordingConfigYaml::full();
        let default = RecordingConfigYaml::default();
        // full() should only differ from default() in `enabled`
        assert!(full.enabled);
        assert!(!default.enabled);
        assert_eq!(full.compress, default.compress);
        assert_eq!(full.interval, default.interval);
        assert_eq!(full.max_size_mb, default.max_size_mb);
        assert_eq!(full.record_inputs, default.record_inputs);
        assert_eq!(full.record_outputs, default.record_outputs);
        assert_eq!(full.record_timing, default.record_timing);
        assert_eq!(full.session_name, default.session_name);
        assert_eq!(full.output_dir, default.output_dir);
        assert_eq!(full.include_nodes, default.include_nodes);
        assert_eq!(full.exclude_nodes, default.exclude_nodes);
    }

    #[test]
    fn recording_config_minimal_differs_from_full() {
        let full = RecordingConfigYaml::full();
        let minimal = RecordingConfigYaml::minimal();
        // Both enabled
        assert!(full.enabled);
        assert!(minimal.enabled);
        // Minimal skips inputs and timing
        assert!(full.record_inputs);
        assert!(!minimal.record_inputs);
        assert!(full.record_timing);
        assert!(!minimal.record_timing);
        // Minimal has larger interval (less frequent)
        assert!(minimal.interval > full.interval);
        // Minimal has a size cap
        assert!(minimal.max_size_mb > 0);
        assert_eq!(full.max_size_mb, 0);
    }

    #[test]
    fn recording_config_with_output_dir() {
        let config = RecordingConfigYaml {
            output_dir: Some("/tmp/horus_recordings".to_string()),
            ..Default::default()
        };
        assert_eq!(
            config.output_dir.as_deref(),
            Some("/tmp/horus_recordings")
        );
    }

    #[test]
    fn recording_config_all_recording_flags_false() {
        let config = RecordingConfigYaml {
            enabled: true,
            record_inputs: false,
            record_outputs: false,
            record_timing: false,
            ..Default::default()
        };
        // Valid config: enabled but recording nothing — config allows it
        assert!(config.enabled);
        assert!(!config.record_inputs);
        assert!(!config.record_outputs);
        assert!(!config.record_timing);
    }

    #[test]
    fn recording_config_overlapping_include_exclude_nodes() {
        // Config struct accepts overlapping lists — validation is at recorder level
        let config = RecordingConfigYaml {
            include_nodes: vec!["sensor".to_string(), "motor".to_string()],
            exclude_nodes: vec!["motor".to_string(), "logger".to_string()],
            ..Default::default()
        };
        // "motor" is in both — struct doesn't enforce consistency
        assert!(config.include_nodes.contains(&"motor".to_string()));
        assert!(config.exclude_nodes.contains(&"motor".to_string()));
    }

    #[test]
    fn recording_config_clone_independence() {
        let config = RecordingConfigYaml {
            enabled: true,
            session_name: Some("session_a".to_string()),
            include_nodes: vec!["node1".to_string()],
            ..Default::default()
        };
        let mut cloned = config.clone();
        cloned.session_name = Some("session_b".to_string());
        cloned.include_nodes.push("node2".to_string());
        cloned.enabled = false;
        // Original preserved
        assert!(config.enabled);
        assert_eq!(config.session_name.as_deref(), Some("session_a"));
        assert_eq!(config.include_nodes.len(), 1);
    }

    #[test]
    fn recording_config_debug_format() {
        let config = RecordingConfigYaml {
            enabled: true,
            session_name: Some("debug_test".to_string()),
            ..Default::default()
        };
        let debug = format!("{:?}", config);
        assert!(debug.contains("RecordingConfigYaml"));
        assert!(debug.contains("debug_test"));
        assert!(debug.contains("enabled"));
    }

    #[test]
    fn recording_config_interval_one_means_every_tick() {
        let config = RecordingConfigYaml::default();
        assert_eq!(config.interval, 1, "Default interval should record every tick");
        let full = RecordingConfigYaml::full();
        assert_eq!(full.interval, 1, "Full recording should record every tick");
    }

    #[test]
    fn scheduler_config_multiple_clones_are_independent() {
        let original = SchedulerConfig::default();
        let mut clone1 = original.clone();
        let mut clone2 = original.clone();
        clone1.timing.global_rate_hz = 500.0;
        clone1.monitoring.verbose = false;
        clone2.timing.global_rate_hz = 1000.0;
        clone2.realtime.watchdog_timeout_ms = 999;
        // All three are independent
        assert_eq!(original.timing.global_rate_hz, 100.0);
        assert!(original.monitoring.verbose);
        assert_eq!(original.realtime.watchdog_timeout_ms, 0);
        assert_eq!(clone1.timing.global_rate_hz, 500.0);
        assert!(!clone1.monitoring.verbose);
        assert_eq!(clone2.timing.global_rate_hz, 1000.0);
        assert_eq!(clone2.realtime.watchdog_timeout_ms, 999);
    }

    #[test]
    fn scheduler_config_debug_contains_nested_structs() {
        let config = SchedulerConfig {
            monitoring: MonitoringConfig {
                black_box_size_mb: 64,
                telemetry_endpoint: Some("udp://metrics:9999".to_string()),
                verbose: false,
            },
            recording: Some(RecordingConfigYaml {
                session_name: Some("nested_debug".to_string()),
                ..RecordingConfigYaml::full()
            }),
            ..SchedulerConfig::default()
        };
        let debug = format!("{:?}", config);
        // Should contain all nested struct names
        assert!(debug.contains("SchedulerConfig"));
        assert!(debug.contains("TimingConfig"));
        assert!(debug.contains("RealTimeConfig"));
        assert!(debug.contains("ResourceConfig"));
        assert!(debug.contains("MonitoringConfig"));
        assert!(debug.contains("RecordingConfigYaml"));
        // Should contain nested values
        assert!(debug.contains("udp://metrics:9999"));
        assert!(debug.contains("nested_debug"));
        assert!(debug.contains("64"));
    }

    #[test]
    fn scheduler_config_recording_none_vs_disabled() {
        // None = no recording config at all
        let no_recording = SchedulerConfig::default();
        assert!(no_recording.recording.is_none());

        // Some(disabled) = config exists but recording is off
        let disabled_recording = SchedulerConfig {
            recording: Some(RecordingConfigYaml::default()),
            ..SchedulerConfig::default()
        };
        assert!(disabled_recording.recording.is_some());
        assert!(!disabled_recording.recording.as_ref().unwrap().enabled);

        // These are semantically different states
        let none_debug = format!("{:?}", no_recording.recording);
        let some_debug = format!("{:?}", disabled_recording.recording);
        assert_ne!(none_debug, some_debug);
    }

    #[test]
    fn scheduler_config_clone_with_recording() {
        let config = SchedulerConfig {
            recording: Some(RecordingConfigYaml {
                enabled: true,
                session_name: Some("clone_test".to_string()),
                include_nodes: vec!["a".to_string(), "b".to_string()],
                ..Default::default()
            }),
            ..SchedulerConfig::default()
        };
        let mut cloned = config.clone();
        // Mutate the clone's recording
        cloned.recording.as_mut().unwrap().enabled = false;
        cloned
            .recording
            .as_mut()
            .unwrap()
            .include_nodes
            .push("c".to_string());
        // Original preserved
        assert!(config.recording.as_ref().unwrap().enabled);
        assert_eq!(config.recording.as_ref().unwrap().include_nodes.len(), 2);
    }

    #[test]
    fn scheduler_config_struct_update_syntax() {
        // Verify that `..SchedulerConfig::default()` fills all remaining fields correctly
        let config = SchedulerConfig {
            timing: TimingConfig {
                global_rate_hz: 250.0,
                deterministic_order: true,
            },
            ..SchedulerConfig::default()
        };
        // Overridden fields
        assert_eq!(config.timing.global_rate_hz, 250.0);
        assert!(config.timing.deterministic_order);
        // Inherited defaults
        assert_eq!(config.realtime.watchdog_timeout_ms, 0);
        assert!(!config.realtime.memory_locking);
        assert!(config.resources.cpu_cores.is_none());
        assert!(config.monitoring.verbose);
        assert!(config.recording.is_none());
    }

    #[test]
    fn recording_config_many_include_nodes() {
        let nodes: Vec<String> = (0..1000).map(|i| format!("node_{}", i)).collect();
        let config = RecordingConfigYaml {
            include_nodes: nodes,
            ..Default::default()
        };
        assert_eq!(config.include_nodes.len(), 1000);
        assert_eq!(config.include_nodes[0], "node_0");
        assert_eq!(config.include_nodes[999], "node_999");
    }

    #[test]
    fn recording_config_many_exclude_nodes() {
        let nodes: Vec<String> = (0..500).map(|i| format!("excluded_{}", i)).collect();
        let config = RecordingConfigYaml {
            exclude_nodes: nodes,
            ..Default::default()
        };
        assert_eq!(config.exclude_nodes.len(), 500);
    }

    #[test]
    fn recording_config_empty_session_name_vs_none() {
        let empty_name = RecordingConfigYaml {
            session_name: Some(String::new()),
            ..Default::default()
        };
        let no_name = RecordingConfigYaml::default();
        // Some("") and None are distinct
        assert!(empty_name.session_name.is_some());
        assert!(empty_name.session_name.as_ref().unwrap().is_empty());
        assert!(no_name.session_name.is_none());
    }

    #[test]
    fn timing_config_fractional_rate() {
        // Non-integer rates should work (e.g. 33.33 Hz for 30ms period)
        let config = TimingConfig {
            global_rate_hz: 33.333333,
            deterministic_order: false,
        };
        let period = 1.0 / config.global_rate_hz;
        assert!((period - 0.03).abs() < 0.001, "33.33 Hz should be ~30ms period");
    }

    #[test]
    fn timing_config_epsilon_rate() {
        let config = TimingConfig {
            global_rate_hz: f64::EPSILON,
            deterministic_order: false,
        };
        assert!(config.global_rate_hz > 0.0);
        let period = 1.0 / config.global_rate_hz;
        assert!(period.is_finite(), "Period of epsilon rate should be finite");
    }

    #[test]
    fn scheduler_config_blackbox_without_monitoring_endpoint() {
        // Blackbox enabled but no telemetry endpoint — valid combination
        let config = SchedulerConfig {
            monitoring: MonitoringConfig {
                black_box_size_mb: 256,
                telemetry_endpoint: None,
                verbose: false,
            },
            ..SchedulerConfig::default()
        };
        assert_eq!(config.monitoring.black_box_size_mb, 256);
        assert!(config.monitoring.telemetry_endpoint.is_none());
        assert!(!config.monitoring.verbose);
    }

    #[test]
    fn scheduler_config_telemetry_without_blackbox() {
        // Telemetry endpoint but no blackbox — valid combination
        let config = SchedulerConfig {
            monitoring: MonitoringConfig {
                black_box_size_mb: 0,
                telemetry_endpoint: Some("udp://localhost:5000".to_string()),
                verbose: true,
            },
            ..SchedulerConfig::default()
        };
        assert_eq!(config.monitoring.black_box_size_mb, 0);
        assert!(config.monitoring.telemetry_endpoint.is_some());
    }

    #[test]
    fn realtime_config_max_watchdog_with_one_miss() {
        // Extremely long watchdog but only 1 miss allowed — strict mode
        let rt = RealTimeConfig {
            watchdog_timeout_ms: 59_999,
            max_deadline_misses: 1,
            memory_locking: true,
            rt_scheduling_class: true,
        };
        assert_eq!(rt.watchdog_timeout_ms, 59_999);
        assert_eq!(rt.max_deadline_misses, 1);
    }

    #[test]
    fn realtime_config_one_ms_watchdog_with_max_misses() {
        // Tiniest watchdog but unlimited misses — lenient mode
        let rt = RealTimeConfig {
            watchdog_timeout_ms: 1,
            max_deadline_misses: u64::MAX,
            memory_locking: false,
            rt_scheduling_class: false,
        };
        assert_eq!(rt.watchdog_timeout_ms, 1);
        assert_eq!(rt.max_deadline_misses, u64::MAX);
    }

    #[test]
    fn scheduler_config_default_debug_is_stable() {
        // Two defaults should produce identical debug output
        let a = format!("{:?}", SchedulerConfig::default());
        let b = format!("{:?}", SchedulerConfig::default());
        assert_eq!(a, b, "Default config Debug output must be deterministic");
    }

    #[test]
    fn recording_config_default_debug_is_stable() {
        let a = format!("{:?}", RecordingConfigYaml::default());
        let b = format!("{:?}", RecordingConfigYaml::default());
        assert_eq!(a, b, "Default recording Debug output must be deterministic");
    }

    // ── Proptest: combination fuzzing ──────────────────────────────────

    proptest! {
        #![proptest_config(ProptestConfig::with_cases(200))]

        /// Scheduler config with recording: clone via Debug roundtrip is consistent
        #[test]
        fn scheduler_config_with_recording_clone_consistent(
            rec in arb_recording_config(),
            timing in arb_timing_config(),
            rt in arb_realtime_config(),
            mon in arb_monitoring_config(),
        ) {
            let config = SchedulerConfig {
                timing,
                realtime: rt,
                resources: ResourceConfig { cpu_cores: None },
                monitoring: mon,
                recording: Some(rec),
            };
            let cloned = config.clone();
            prop_assert_eq!(format!("{:?}", config), format!("{:?}", cloned));
        }

        /// Mutating a clone never affects the original
        #[test]
        fn scheduler_config_clone_mutation_isolation(
            rate in 0.001f64..10_000.0,
            watchdog in 0u64..60_000,
            bb_size in 0usize..1024,
        ) {
            let original = SchedulerConfig {
                timing: TimingConfig {
                    global_rate_hz: rate,
                    deterministic_order: false,
                },
                realtime: RealTimeConfig {
                    watchdog_timeout_ms: watchdog,
                    ..SchedulerConfig::default().realtime
                },
                monitoring: MonitoringConfig {
                    black_box_size_mb: bb_size,
                    ..SchedulerConfig::default().monitoring
                },
                ..SchedulerConfig::default()
            };
            let original_debug = format!("{:?}", original);
            let mut cloned = original.clone();
            cloned.timing.global_rate_hz = 99999.0;
            cloned.realtime.watchdog_timeout_ms = 0;
            cloned.monitoring.black_box_size_mb = 0;
            // Original unchanged
            prop_assert_eq!(format!("{:?}", original), original_debug);
        }

        /// RecordingConfig clone is always independent (deep copy of Vecs)
        #[test]
        fn recording_config_clone_vec_independence(
            n_include in 0usize..50,
            n_exclude in 0usize..50,
        ) {
            let config = RecordingConfigYaml {
                include_nodes: (0..n_include).map(|i| format!("inc_{}", i)).collect(),
                exclude_nodes: (0..n_exclude).map(|i| format!("exc_{}", i)).collect(),
                ..Default::default()
            };
            let mut cloned = config.clone();
            cloned.include_nodes.push("extra".to_string());
            cloned.exclude_nodes.clear();
            prop_assert_eq!(config.include_nodes.len(), n_include);
            prop_assert_eq!(config.exclude_nodes.len(), n_exclude);
        }
    }
}
