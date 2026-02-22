// Scheduler configuration - preset factories and data structs

use super::deterministic::DeterministicConfig;

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
    /// Deterministic execution configuration (virtual time, seeded RNG, trace)
    pub deterministic: Option<DeterministicConfig>,
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
                telemetry_endpoint: None,
            },
            recording: None,
            deterministic: None,
        }
    }

    /// Standard configuration for most robots
    pub fn standard() -> Self {
        let mut config = Self::minimal();
        config.circuit_breaker = true;
        config.monitoring.profiling_enabled = true;
        config
    }

    /// Deploy configuration for production robots.
    ///
    /// Standard rate (60 Hz) with RT features (best-effort) and a 16MB BlackBox flight recorder.
    pub fn deploy() -> Self {
        let mut config = Self::standard();
        config.realtime.deadline_monitoring = true;
        config.realtime.watchdog_enabled = true;
        config.realtime.memory_locking = true;
        config.realtime.rt_scheduling_class = true;
        config.monitoring.black_box_enabled = true;
        config.monitoring.black_box_size_mb = 16;
        config
    }

    /// Deterministic configuration for safety certification and replay
    pub fn deterministic() -> Self {
        let mut config = Self::minimal();
        config.timing.global_rate_hz = 1000.0;
        config.realtime.deadline_monitoring = true;
        config.realtime.max_deadline_misses = 3;
        config.monitoring.metrics_interval_ms = 100;
        config.monitoring.black_box_enabled = true;
        config.monitoring.black_box_size_mb = 100;
        config.recording = Some(RecordingConfigYaml::full());
        config.deterministic = Some(DeterministicConfig::default());
        config
    }

    /// Safety-critical configuration (medical, surgical)
    pub fn safety_critical() -> Self {
        let mut config = Self::minimal();
        config.timing.global_rate_hz = 1000.0;
        config.realtime.wcet_enforcement = true;
        config.realtime.deadline_monitoring = true;
        config.realtime.watchdog_enabled = true;
        config.realtime.watchdog_timeout_ms = 100;
        config.realtime.safety_monitor = true;
        config.realtime.max_deadline_misses = 0;
        config.realtime.memory_locking = true;
        config.realtime.rt_scheduling_class = true;
        config.resources.cpu_cores = Some(vec![0, 1]);
        config.resources.numa_aware = true;
        config.monitoring.metrics_interval_ms = 10;
        config.monitoring.black_box_enabled = true;
        config.monitoring.black_box_size_mb = 1024;
        config.recording = Some(RecordingConfigYaml::full());
        config
    }

    /// High-performance configuration (racing, competition)
    pub fn high_performance() -> Self {
        let mut config = Self::minimal();
        config.execution = ExecutionMode::Parallel;
        config.timing.global_rate_hz = 10000.0;
        config.circuit_breaker = true;
        config.realtime.wcet_enforcement = true;
        config.realtime.deadline_monitoring = true;
        config.realtime.watchdog_timeout_ms = 0;
        config.realtime.max_deadline_misses = 10;
        config.realtime.memory_locking = true;
        config.realtime.rt_scheduling_class = true;
        config.resources.numa_aware = true;
        config.monitoring.metrics_interval_ms = 10000;
        config
    }

    /// Hard real-time configuration for surgical robots, CNC machines
    pub fn hard_realtime() -> Self {
        let mut config = Self::standard();
        config.execution = ExecutionMode::Parallel;
        config.timing.global_rate_hz = 1000.0;
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
