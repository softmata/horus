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

/// Scheduler configuration — plain data bag.
///
/// Use `SchedulerConfig::minimal()` as a starting point, then mutate fields directly:
///
/// ```rust,ignore
/// let mut config = SchedulerConfig::minimal();
/// config.timing.global_rate_hz = 500.0;
/// config.realtime.wcet_enforcement = true;
/// let mut scheduler = Scheduler::from_config(config);
/// ```
///
/// For common use cases, prefer `Scheduler` presets:
/// `Scheduler::deploy()`, `Scheduler::safety_critical()`, etc.
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
    /// Deterministic execution configuration (virtual time, seeded RNG)
    pub deterministic: Option<DeterministicConfig>,
}

impl Default for SchedulerConfig {
    fn default() -> Self {
        Self::minimal()
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

}
