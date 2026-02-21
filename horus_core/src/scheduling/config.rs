// Scheduler configuration - preset factories and data structs

/// Execution mode for the scheduler
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ExecutionMode {
    /// Parallel execution with dependency resolution
    Parallel,
    /// Traditional sequential execution
    Sequential,
}

/// Timing configuration
#[derive(Debug, Clone)]
pub struct TimingConfig {
    /// Global tick rate in Hz (default: 60)
    pub global_rate_hz: f64,
    /// Enable per-node rate control
    pub per_node_rates: bool,
}

/// Fault tolerance configuration
#[derive(Debug, Clone)]
pub struct FaultConfig {
    /// Enable circuit breaker pattern
    pub circuit_breaker_enabled: bool,
    /// Max failures before circuit opens
    pub max_failures: u32,
    /// Success count to close circuit
    pub recovery_threshold: u32,
    /// Circuit timeout in milliseconds
    pub circuit_timeout_ms: u64,
}

/// Real-time configuration
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
    /// Telemetry export endpoint (e.g., "udp://localhost:9999", "file:///var/log/metrics.json")
    pub telemetry_endpoint: Option<String>,
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

    /// Create a recording config optimized for debugging
    pub fn debug() -> Self {
        Self {
            enabled: true,
            session_name: Some("debug".to_string()),
            compress: false,
            max_size_mb: 100,
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

/// Scheduler configuration
///
/// Use preset factories (`standard()`, `safety_critical()`, etc.) then
/// mutate fields directly for customization:
///
/// ```rust,ignore
/// let mut config = SchedulerConfig::standard();
/// config.timing.global_rate_hz = 500.0;
/// config.realtime.wcet_enforcement = true;
/// ```
#[derive(Debug, Clone)]
pub struct SchedulerConfig {
    /// Execution mode
    pub execution: ExecutionMode,
    /// Timing configuration
    pub timing: TimingConfig,
    /// Fault tolerance configuration
    pub fault: FaultConfig,
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
        Self::standard()
    }
}

impl SchedulerConfig {
    /// Minimal configuration - starts with the most basic settings.
    ///
    /// Use this when you want to configure everything explicitly.
    /// Most features are disabled by default.
    pub fn minimal() -> Self {
        Self {
            execution: ExecutionMode::Sequential,
            timing: TimingConfig {
                global_rate_hz: 60.0,
                per_node_rates: false,
            },
            fault: FaultConfig {
                circuit_breaker_enabled: false,
                max_failures: 5,
                recovery_threshold: 3,
                circuit_timeout_ms: 5000,
            },
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
                telemetry_endpoint: None,
            },
            recording: None,
        }
    }

    /// Standard configuration for most robots
    pub fn standard() -> Self {
        Self {
            execution: ExecutionMode::Sequential,
            timing: TimingConfig {
                global_rate_hz: 60.0,
                per_node_rates: true,
            },
            fault: FaultConfig {
                circuit_breaker_enabled: true,
                max_failures: 5,
                recovery_threshold: 3,
                circuit_timeout_ms: 5000,
            },
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
                profiling_enabled: true,
                metrics_interval_ms: 1000,
                black_box_enabled: false,
                black_box_size_mb: 0,
                telemetry_endpoint: None,
            },
            recording: None,
        }
    }

    /// Deploy configuration for production robots.
    ///
    /// Standard rate (60 Hz) with RT features (best-effort) and a 16MB BlackBox flight recorder.
    pub fn deploy() -> Self {
        Self {
            execution: ExecutionMode::Sequential,
            timing: TimingConfig {
                global_rate_hz: 60.0,
                per_node_rates: true,
            },
            fault: FaultConfig {
                circuit_breaker_enabled: true,
                max_failures: 5,
                recovery_threshold: 3,
                circuit_timeout_ms: 5000,
            },
            realtime: RealTimeConfig {
                wcet_enforcement: false,
                deadline_monitoring: true,
                watchdog_enabled: true,
                watchdog_timeout_ms: 1000,
                safety_monitor: false,
                max_deadline_misses: 100,
                memory_locking: true,
                rt_scheduling_class: true,
            },
            resources: ResourceConfig {
                cpu_cores: None,
                numa_aware: false,
            },
            monitoring: MonitoringConfig {
                profiling_enabled: true,
                metrics_interval_ms: 1000,
                black_box_enabled: true,
                black_box_size_mb: 16,
                telemetry_endpoint: None,
            },
            recording: None,
        }
    }

    /// Deterministic configuration for safety certification and replay
    pub fn deterministic() -> Self {
        Self {
            execution: ExecutionMode::Sequential,
            timing: TimingConfig {
                global_rate_hz: 1000.0,
                per_node_rates: false,
            },
            fault: FaultConfig {
                circuit_breaker_enabled: false,
                max_failures: 0,
                recovery_threshold: 0,
                circuit_timeout_ms: 0,
            },
            realtime: RealTimeConfig {
                wcet_enforcement: false,
                deadline_monitoring: true,
                watchdog_enabled: false,
                watchdog_timeout_ms: 1000,
                safety_monitor: false,
                max_deadline_misses: 3,
                memory_locking: false,
                rt_scheduling_class: false,
            },
            resources: ResourceConfig {
                cpu_cores: None,
                numa_aware: false,
            },
            monitoring: MonitoringConfig {
                profiling_enabled: false,
                metrics_interval_ms: 100,
                black_box_enabled: true,
                black_box_size_mb: 100,
                telemetry_endpoint: None,
            },
            recording: Some(RecordingConfigYaml::full()),
        }
    }

    /// Safety-critical configuration (medical, surgical)
    pub fn safety_critical() -> Self {
        Self {
            execution: ExecutionMode::Sequential,
            timing: TimingConfig {
                global_rate_hz: 1000.0,
                per_node_rates: false,
            },
            fault: FaultConfig {
                circuit_breaker_enabled: false,
                max_failures: 0,
                recovery_threshold: 0,
                circuit_timeout_ms: 0,
            },
            realtime: RealTimeConfig {
                wcet_enforcement: true,
                deadline_monitoring: true,
                watchdog_enabled: true,
                watchdog_timeout_ms: 100,
                safety_monitor: true,
                max_deadline_misses: 0,
                memory_locking: true,
                rt_scheduling_class: true,
            },
            resources: ResourceConfig {
                cpu_cores: Some(vec![0, 1]),
                numa_aware: true,
            },
            monitoring: MonitoringConfig {
                profiling_enabled: false,
                metrics_interval_ms: 10,
                black_box_enabled: true,
                black_box_size_mb: 1024,
                telemetry_endpoint: None,
            },
            recording: Some(RecordingConfigYaml::full()),
        }
    }

    /// High-performance configuration (racing, competition)
    pub fn high_performance() -> Self {
        Self {
            execution: ExecutionMode::Parallel,
            timing: TimingConfig {
                global_rate_hz: 10000.0,
                per_node_rates: true,
            },
            fault: FaultConfig {
                circuit_breaker_enabled: true,
                max_failures: 3,
                recovery_threshold: 1,
                circuit_timeout_ms: 100,
            },
            realtime: RealTimeConfig {
                wcet_enforcement: true,
                deadline_monitoring: true,
                watchdog_enabled: false,
                watchdog_timeout_ms: 0,
                safety_monitor: false,
                max_deadline_misses: 10,
                memory_locking: true,
                rt_scheduling_class: true,
            },
            resources: ResourceConfig {
                cpu_cores: None,
                numa_aware: true,
            },
            monitoring: MonitoringConfig {
                profiling_enabled: false,
                metrics_interval_ms: 10000,
                black_box_enabled: false,
                black_box_size_mb: 0,
                telemetry_endpoint: None,
            },
            recording: None,
        }
    }

    /// Hard real-time configuration for surgical robots, CNC machines
    pub fn hard_realtime() -> Self {
        let mut config = Self::standard();
        config.execution = ExecutionMode::Parallel;
        config.timing.global_rate_hz = 1000.0;
        config.fault.circuit_breaker_enabled = true;
        config.fault.max_failures = 3;
        config.realtime.wcet_enforcement = true;
        config.realtime.deadline_monitoring = true;
        config.realtime.watchdog_enabled = true;
        config.realtime.watchdog_timeout_ms = 10;
        config.realtime.safety_monitor = true;
        config.realtime.max_deadline_misses = 3;
        config.realtime.memory_locking = true;
        config.realtime.rt_scheduling_class = true;
        config.monitoring.profiling_enabled = false;
        config.monitoring.black_box_enabled = true;
        config.monitoring.black_box_size_mb = 100;
        config.recording = Some(RecordingConfigYaml::minimal());
        config
    }
}
