// Comprehensive scheduler configuration for 100% robotics coverage
use std::collections::HashMap;
use std::time::Duration;

/// Execution mode for the scheduler
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum ExecutionMode {
    /// JIT compilation for ultra-fast control loops (37ns tick time)
    JITOptimized,
    /// Parallel execution with dependency resolution
    Parallel,
    /// Async I/O for network and file operations
    AsyncIO,
    /// Traditional sequential execution
    Sequential,
    /// Mixed mode with automatic classification
    AutoAdaptive,
}

/// Timing configuration for different robot requirements
#[derive(Debug, Clone)]
pub struct TimingConfig {
    /// Global tick rate in Hz (default: 60)
    pub global_rate_hz: f64,
    /// Enable per-node rate control
    pub per_node_rates: bool,
    /// Maximum allowed jitter in microseconds
    pub max_jitter_us: u64,
    /// Deadline miss policy
    pub deadline_miss_policy: DeadlineMissPolicy,
    /// Time synchronization source
    pub time_sync_source: TimeSyncSource,
}

/// What to do when a deadline is missed
#[derive(Debug, Clone, Copy)]
pub enum DeadlineMissPolicy {
    /// Log warning and continue
    Warn,
    /// Skip the node for this tick
    Skip,
    /// Terminate the scheduler
    Panic,
    /// Downgrade priority and continue
    Degrade,
}

/// Time synchronization source for distributed systems
#[derive(Debug, Clone, Copy)]
pub enum TimeSyncSource {
    /// System monotonic clock
    Monotonic,
    /// Network Time Protocol
    NTP,
    /// GPS time
    GPS,
    /// Precision Time Protocol (IEEE 1588)
    PTP,
    /// Custom external source
    External,
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
    /// Enable automatic node restart
    pub auto_restart: bool,
    /// Redundancy factor (reserved - use RedundancyManager directly in nodes)
    pub redundancy_factor: u32,
    /// Checkpointing frequency (0 = disabled)
    pub checkpoint_interval_ms: u64,
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
    /// Enable priority inheritance protocol
    pub priority_inheritance: bool,
    /// Enable formal verification checks (debug builds only)
    pub formal_verification: bool,
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
    /// Memory limit in MB (0 = unlimited)
    pub memory_limit_mb: usize,
    /// I/O priority (0-7, 0 = highest)
    pub io_priority: u8,
    /// Enable NUMA awareness
    pub numa_aware: bool,
    /// GPU device IDs to use
    pub gpu_devices: Vec<usize>,
    /// Enable power management
    pub power_management: bool,
    /// Target power budget in watts (0 = unlimited)
    pub power_budget_watts: u32,
}

/// Monitoring and telemetry configuration
#[derive(Debug, Clone)]
pub struct MonitoringConfig {
    /// Enable runtime profiling
    pub profiling_enabled: bool,
    /// Enable distributed tracing
    pub tracing_enabled: bool,
    /// Metrics export interval in ms
    pub metrics_interval_ms: u64,
    /// Telemetry endpoint URL
    pub telemetry_endpoint: Option<String>,
    /// Enable black box recording
    pub black_box_enabled: bool,
    /// Black box buffer size in MB
    pub black_box_size_mb: usize,
}

/// Recording configuration for record/replay system
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

    /// Create a recording config optimized for debugging
    pub fn debug() -> Self {
        Self {
            enabled: true,
            session_name: Some("debug".to_string()),
            compress: false, // Faster without compression
            interval: 1,
            output_dir: None,
            max_size_mb: 100, // Limit size for debugging
            include_nodes: vec![],
            exclude_nodes: vec![],
            record_inputs: true,
            record_outputs: true,
            record_timing: true,
        }
    }

    /// Create a minimal recording config (outputs only)
    pub fn minimal() -> Self {
        Self {
            enabled: true,
            session_name: None,
            compress: true,
            interval: 10, // Every 10 ticks
            output_dir: None,
            max_size_mb: 50,
            include_nodes: vec![],
            exclude_nodes: vec![],
            record_inputs: false,
            record_outputs: true,
            record_timing: false,
        }
    }
}

/// Robot-specific presets
///
/// Note: Only presets with actual constructor functions are listed.
/// Use `SchedulerConfig::standard()`, `safety_critical()`, etc.
#[derive(Debug, Clone, Copy)]
pub enum RobotPreset {
    /// Standard industrial robot
    Standard,
    /// Safety-critical medical/surgical robot
    SafetyCritical,
    /// Hard real-time aerospace/defense
    HardRealTime,
    /// High-performance racing/competition
    HighPerformance,
    /// Space/satellite robot (with redundancy + checkpointing)
    Space,
    /// Swarm robotics (parallel execution)
    Swarm,
    /// Soft robotics (slower tick rate for soft materials)
    SoftRobotics,
    /// Quantum-assisted robotics
    Quantum,
    /// Educational/learning robots
    Educational,
    /// Mobile ground robots
    Mobile,
    /// Underwater/marine robots
    Underwater,
    /// Custom configuration
    Custom,
}

/// Deterministic execution configuration
///
/// This enables strict topology validation and deterministic execution order,
/// providing guarantees similar to compile-time scheduled systems.
#[derive(Debug, Clone)]
pub struct DeterministicConfig {
    /// Enforce strict topology - reject undeclared topics at runtime
    pub strict_topology: bool,

    /// Wait for all declared connections before first tick
    pub startup_barrier: bool,

    /// Startup barrier timeout in milliseconds (fail if not all connected)
    pub barrier_timeout_ms: u64,

    /// Deterministic RNG seed for reproducible randomness
    pub rng_seed: Option<u64>,

    /// Reject dynamic node addition after startup
    pub freeze_topology_after_start: bool,

    /// Validate all topic producers have consumers (and vice versa)
    pub require_complete_connections: bool,

    /// Static execution order (computed once at startup, never changes)
    pub static_execution_order: bool,
}

impl Default for DeterministicConfig {
    fn default() -> Self {
        Self {
            strict_topology: false,
            startup_barrier: false,
            barrier_timeout_ms: 5000,
            rng_seed: None,
            freeze_topology_after_start: false,
            require_complete_connections: false,
            static_execution_order: false,
        }
    }
}

impl DeterministicConfig {
    /// Full determinism - all guarantees enabled
    pub fn strict() -> Self {
        Self {
            strict_topology: true,
            startup_barrier: true,
            barrier_timeout_ms: 5000,
            rng_seed: Some(42),
            freeze_topology_after_start: true,
            require_complete_connections: true,
            static_execution_order: true,
        }
    }

    /// Partial determinism - execution order only, no topology validation
    pub fn execution_only() -> Self {
        Self {
            strict_topology: false,
            startup_barrier: false,
            barrier_timeout_ms: 5000,
            rng_seed: Some(42),
            freeze_topology_after_start: false,
            require_complete_connections: false,
            static_execution_order: true,
        }
    }
}

/// Complete scheduler configuration for 100% robotics coverage
#[derive(Debug, Clone)]
pub struct SchedulerConfig {
    /// Execution mode configuration
    pub execution: ExecutionMode,

    /// Timing and scheduling configuration
    pub timing: TimingConfig,

    /// Fault tolerance configuration
    pub fault: FaultConfig,

    /// Real-time configuration
    pub realtime: RealTimeConfig,

    /// Resource management
    pub resources: ResourceConfig,

    /// Monitoring and telemetry
    pub monitoring: MonitoringConfig,

    /// Robot preset (for quick setup)
    pub preset: RobotPreset,

    /// Custom configuration for edge cases
    /// This HashMap allows ANY custom configuration that might be needed
    /// for exotic robot types (quantum, biological, hybrid, etc.)
    pub custom: HashMap<String, ConfigValue>,

    /// Deterministic execution configuration
    /// When Some, enables strict topology validation and deterministic execution
    pub deterministic: Option<DeterministicConfig>,

    /// Recording configuration for record/replay system
    /// When Some and enabled, scheduler will automatically record all node execution
    pub recording: Option<RecordingConfigYaml>,
}

/// Flexible value type for custom configurations
#[derive(Debug, Clone)]
pub enum ConfigValue {
    Bool(bool),
    Integer(i64),
    Float(f64),
    String(String),
    Duration(Duration),
    List(Vec<ConfigValue>),
    Map(HashMap<String, ConfigValue>),
    Binary(Vec<u8>),
}

impl Default for SchedulerConfig {
    fn default() -> Self {
        Self::standard()
    }
}

impl SchedulerConfig {
    /// Standard configuration for most robots
    pub fn standard() -> Self {
        Self {
            execution: ExecutionMode::AutoAdaptive,
            timing: TimingConfig {
                global_rate_hz: 60.0,
                per_node_rates: true,
                max_jitter_us: 1000,
                deadline_miss_policy: DeadlineMissPolicy::Warn,
                time_sync_source: TimeSyncSource::Monotonic,
            },
            fault: FaultConfig {
                circuit_breaker_enabled: true,
                max_failures: 5,
                recovery_threshold: 3,
                circuit_timeout_ms: 5000,
                auto_restart: true,
                redundancy_factor: 1,
                checkpoint_interval_ms: 0,
            },
            realtime: RealTimeConfig {
                wcet_enforcement: false,
                deadline_monitoring: false,
                watchdog_enabled: false,
                watchdog_timeout_ms: 1000,
                safety_monitor: false,
                max_deadline_misses: 100,
                priority_inheritance: false,
                formal_verification: cfg!(debug_assertions),
                memory_locking: false,
                rt_scheduling_class: false,
            },
            resources: ResourceConfig {
                cpu_cores: None,
                memory_limit_mb: 0,
                io_priority: 4,
                numa_aware: false,
                gpu_devices: vec![],
                power_management: false,
                power_budget_watts: 0,
            },
            monitoring: MonitoringConfig {
                profiling_enabled: true,
                tracing_enabled: false,
                metrics_interval_ms: 1000,
                telemetry_endpoint: None,
                black_box_enabled: false,
                black_box_size_mb: 0,
            },
            preset: RobotPreset::Standard,
            custom: HashMap::new(),
            deterministic: None,
            recording: None,
        }
    }

    /// Deterministic configuration
    ///
    /// Enables:
    /// - Static execution order (computed once at startup)
    /// - Startup barrier (wait for all nodes to connect)
    /// - Topology validation (all topics must have producers and consumers)
    /// - Frozen topology after startup (no dynamic node addition)
    /// - Deterministic RNG seed for reproducibility
    ///
    /// Use this for safety certification, debugging, and exact replay.
    pub fn deterministic() -> Self {
        Self {
            execution: ExecutionMode::Sequential,
            timing: TimingConfig {
                global_rate_hz: 1000.0,
                per_node_rates: false,
                max_jitter_us: 10,
                deadline_miss_policy: DeadlineMissPolicy::Panic,
                time_sync_source: TimeSyncSource::Monotonic,
            },
            fault: FaultConfig {
                circuit_breaker_enabled: false,
                max_failures: 0,
                recovery_threshold: 0,
                circuit_timeout_ms: 0,
                auto_restart: false,
                redundancy_factor: 1,
                checkpoint_interval_ms: 0,
            },
            realtime: RealTimeConfig {
                wcet_enforcement: false,
                deadline_monitoring: true,
                watchdog_enabled: false,
                watchdog_timeout_ms: 1000,
                safety_monitor: false,
                max_deadline_misses: 3,
                priority_inheritance: true,
                formal_verification: true,
                memory_locking: false,
                rt_scheduling_class: false,
            },
            resources: ResourceConfig {
                cpu_cores: None,
                memory_limit_mb: 0,
                io_priority: 4,
                numa_aware: false,
                gpu_devices: vec![],
                power_management: false,
                power_budget_watts: 0,
            },
            monitoring: MonitoringConfig {
                profiling_enabled: false, // No profiling overhead
                tracing_enabled: true,    // Full audit trail
                metrics_interval_ms: 100,
                telemetry_endpoint: None,
                black_box_enabled: true,
                black_box_size_mb: 100,
            },
            preset: RobotPreset::Custom,
            custom: HashMap::new(),
            deterministic: Some(DeterministicConfig::strict()),
            recording: Some(RecordingConfigYaml::full()), // Deterministic mode should record for replay
        }
    }

    /// Safety-critical configuration (medical, surgical)
    pub fn safety_critical() -> Self {
        Self {
            execution: ExecutionMode::Sequential, // Deterministic execution
            timing: TimingConfig {
                global_rate_hz: 1000.0,                          // 1kHz for precise control
                per_node_rates: false,                           // Fixed timing for predictability
                max_jitter_us: 10,                               // Ultra-low jitter
                deadline_miss_policy: DeadlineMissPolicy::Panic, // Fail-safe
                time_sync_source: TimeSyncSource::PTP,           // Precision timing
            },
            fault: FaultConfig {
                circuit_breaker_enabled: false, // No automatic recovery
                max_failures: 0,                // Zero tolerance
                recovery_threshold: 0,
                circuit_timeout_ms: 0,
                auto_restart: false,         // Manual intervention required
                redundancy_factor: 3,        // Triple redundancy
                checkpoint_interval_ms: 100, // Frequent checkpoints
            },
            realtime: RealTimeConfig {
                wcet_enforcement: true,     // Strict WCET enforcement
                deadline_monitoring: true,  // Monitor all deadlines
                watchdog_enabled: true,     // Enable watchdogs
                watchdog_timeout_ms: 100,   // 100ms watchdog timeout
                safety_monitor: true,       // Full safety monitoring
                max_deadline_misses: 0,     // Zero tolerance for deadline misses
                priority_inheritance: true, // Priority inheritance protocol
                formal_verification: true,  // Always verify
                memory_locking: true,       // Lock all memory (mlockall)
                rt_scheduling_class: true,  // Use SCHED_FIFO
            },
            resources: ResourceConfig {
                cpu_cores: Some(vec![0, 1]), // Dedicated cores
                memory_limit_mb: 1024,       // Fixed memory
                io_priority: 0,              // Highest priority
                numa_aware: true,
                gpu_devices: vec![],
                power_management: false, // No power scaling
                power_budget_watts: 0,
            },
            monitoring: MonitoringConfig {
                profiling_enabled: false, // No runtime overhead
                tracing_enabled: true,    // Full audit trail
                metrics_interval_ms: 10,  // High-frequency monitoring
                telemetry_endpoint: Some("local".to_string()),
                black_box_enabled: true, // Always record
                black_box_size_mb: 1024, // Large buffer
            },
            preset: RobotPreset::SafetyCritical,
            custom: HashMap::new(),
            deterministic: Some(DeterministicConfig::strict()), // Safety-critical needs determinism
            recording: Some(RecordingConfigYaml::full()),       // Full recording for audit trail
        }
    }

    /// High-performance configuration (racing, competition)
    pub fn high_performance() -> Self {
        Self {
            execution: ExecutionMode::JITOptimized, // Maximum speed
            timing: TimingConfig {
                global_rate_hz: 10000.0, // 10kHz ultra-high frequency
                per_node_rates: true,
                max_jitter_us: 100,
                deadline_miss_policy: DeadlineMissPolicy::Skip,
                time_sync_source: TimeSyncSource::Monotonic,
            },
            fault: FaultConfig {
                circuit_breaker_enabled: true,
                max_failures: 3,
                recovery_threshold: 1,
                circuit_timeout_ms: 100, // Fast recovery
                auto_restart: true,
                redundancy_factor: 1,      // Speed over redundancy
                checkpoint_interval_ms: 0, // No checkpointing overhead
            },
            realtime: RealTimeConfig {
                wcet_enforcement: true,    // Enforce to prevent overruns
                deadline_monitoring: true, // Track timing
                watchdog_enabled: false,   // No watchdogs (overhead)
                watchdog_timeout_ms: 0,
                safety_monitor: false,      // No safety overhead
                max_deadline_misses: 10,    // Some tolerance
                priority_inheritance: true, // Prevent priority inversion
                formal_verification: false, // No verification overhead
                memory_locking: true,       // Lock memory for speed
                rt_scheduling_class: true,  // Real-time scheduling
            },
            resources: ResourceConfig {
                cpu_cores: None,    // Use all cores
                memory_limit_mb: 0, // Unlimited
                io_priority: 2,
                numa_aware: true,
                gpu_devices: vec![0, 1, 2, 3], // All GPUs
                power_management: false,
                power_budget_watts: 0, // Maximum power
            },
            monitoring: MonitoringConfig {
                profiling_enabled: false, // No overhead
                tracing_enabled: false,
                metrics_interval_ms: 10000, // Minimal monitoring
                telemetry_endpoint: None,
                black_box_enabled: false,
                black_box_size_mb: 0,
            },
            preset: RobotPreset::HighPerformance,
            custom: HashMap::new(),
            deterministic: None, // Performance over determinism
            recording: None,     // No recording overhead in high-performance mode
        }
    }

    /// Space robotics configuration
    pub fn space() -> Self {
        let mut config = Self::standard();
        config.preset = RobotPreset::Space;
        config.timing.time_sync_source = TimeSyncSource::GPS;
        config.fault.redundancy_factor = 2;
        config.fault.checkpoint_interval_ms = 5000;
        config.resources.power_management = true;
        config.resources.power_budget_watts = 100; // Limited power

        // Custom space-specific settings
        config
            .custom
            .insert("radiation_hardening".to_string(), ConfigValue::Bool(true));
        config
            .custom
            .insert("thermal_management".to_string(), ConfigValue::Bool(true));
        config.custom.insert(
            "communication_delay_ms".to_string(),
            ConfigValue::Integer(1000),
        );
        config
            .custom
            .insert("autonomous_mode".to_string(), ConfigValue::Bool(true));

        config
    }

    /// Swarm robotics configuration
    pub fn swarm() -> Self {
        let mut config = Self::standard();
        config.preset = RobotPreset::Swarm;
        config.execution = ExecutionMode::Parallel;
        config.timing.time_sync_source = TimeSyncSource::NTP;

        // Custom swarm settings
        config
            .custom
            .insert("swarm_id".to_string(), ConfigValue::Integer(0));
        config
            .custom
            .insert("swarm_size".to_string(), ConfigValue::Integer(100));
        config.custom.insert(
            "consensus_algorithm".to_string(),
            ConfigValue::String("raft".to_string()),
        );
        config
            .custom
            .insert("neighbor_discovery".to_string(), ConfigValue::Bool(true));
        config.custom.insert(
            "collective_behavior".to_string(),
            ConfigValue::String("flocking".to_string()),
        );

        config
    }

    /// Soft robotics configuration
    pub fn soft_robotics() -> Self {
        let mut config = Self::standard();
        config.preset = RobotPreset::SoftRobotics;
        config.timing.global_rate_hz = 100.0; // Slower for soft materials

        // Custom soft robotics settings
        config.custom.insert(
            "material_model".to_string(),
            ConfigValue::String("hyperelastic".to_string()),
        );
        config
            .custom
            .insert("pneumatic_control".to_string(), ConfigValue::Bool(true));
        config
            .custom
            .insert("deformation_limit".to_string(), ConfigValue::Float(0.5));
        config
            .custom
            .insert("pressure_sensors".to_string(), ConfigValue::Integer(32));
        config
            .custom
            .insert("adaptive_compliance".to_string(), ConfigValue::Bool(true));

        config
    }

    /// Hard real-time configuration for surgical robots, CNC machines
    ///
    /// Optimized for <20μs latency and <5μs jitter
    ///
    /// # Features
    /// - WCET enforcement and deadline monitoring
    /// - Deterministic execution (no learning phase)
    /// - Safety monitor with watchdog
    /// - JIT optimization for ultra-fast nodes
    /// - High-priority scheduling
    ///
    /// # Use with
    /// ```ignore
    /// let mut scheduler = Scheduler::new_realtime()?;  // Applies this config automatically
    /// ```
    pub fn hard_realtime() -> Self {
        let mut config = Self::standard();
        config.preset = RobotPreset::HardRealTime;

        // Execution mode: JIT optimization for minimal latency
        config.execution = ExecutionMode::JITOptimized;

        // Timing: High-frequency control with strict jitter limits
        config.timing.global_rate_hz = 1000.0; // 1 kHz default
        config.timing.max_jitter_us = 5; // 5μs max jitter
        config.timing.deadline_miss_policy = DeadlineMissPolicy::Panic; // Hard RT: panic on deadline miss

        // Fault tolerance: Fast recovery
        config.fault.circuit_breaker_enabled = true;
        config.fault.max_failures = 3; // Fail fast
        config.fault.auto_restart = false; // No auto-restart in RT (unsafe)
        config.fault.redundancy_factor = 2; // N-version programming

        // Real-time: Maximum enforcement
        config.realtime.wcet_enforcement = true;
        config.realtime.deadline_monitoring = true;
        config.realtime.watchdog_enabled = true;
        config.realtime.watchdog_timeout_ms = 10; // 10ms watchdog
        config.realtime.safety_monitor = true;
        config.realtime.max_deadline_misses = 3; // Strict: 3 strikes and emergency stop
        config.realtime.priority_inheritance = true;
        config.realtime.formal_verification = true;
        config.realtime.memory_locking = true;
        config.realtime.rt_scheduling_class = true;

        // Resources: Dedicated cores, no power management
        config.resources.io_priority = 0; // Highest I/O priority
        config.resources.power_management = false; // Disable power saving

        // Monitoring: Minimal overhead
        config.monitoring.profiling_enabled = false; // Disable profiling in production RT
        config.monitoring.black_box_enabled = true; // But enable black box for forensics
        config.monitoring.black_box_size_mb = 100;

        // Hard real-time needs deterministic execution
        config.deterministic = Some(DeterministicConfig::execution_only());

        // Enable recording for forensics and debugging
        config.recording = Some(RecordingConfigYaml::minimal());

        config
    }

    /// Helper to get custom value
    pub fn get_custom<T>(&self, key: &str) -> Option<T>
    where
        T: FromConfigValue,
    {
        self.custom.get(key).and_then(|v| T::from_config_value(v))
    }

    /// Helper to set custom value
    pub fn set_custom(&mut self, key: String, value: ConfigValue) {
        self.custom.insert(key, value);
    }

    // ============================================
    // Builder Methods
    // ============================================

    /// Set the execution mode
    pub fn with_execution_mode(mut self, mode: ExecutionMode) -> Self {
        self.execution = mode;
        self
    }

    /// Set the global tick rate in Hz
    pub fn with_tick_rate(mut self, rate_hz: f64) -> Self {
        self.timing.global_rate_hz = rate_hz;
        self
    }

    /// Set the maximum allowed jitter in microseconds
    pub fn with_max_jitter(mut self, jitter_us: u64) -> Self {
        self.timing.max_jitter_us = jitter_us;
        self
    }

    /// Set the deadline miss policy
    pub fn with_deadline_policy(mut self, policy: DeadlineMissPolicy) -> Self {
        self.timing.deadline_miss_policy = policy;
        self
    }

    /// Enable or disable the circuit breaker
    pub fn with_circuit_breaker(mut self, enabled: bool) -> Self {
        self.fault.circuit_breaker_enabled = enabled;
        self
    }

    /// Enable or disable WCET enforcement
    pub fn with_wcet_enforcement(mut self, enabled: bool) -> Self {
        self.realtime.wcet_enforcement = enabled;
        self
    }

    /// Enable or disable the watchdog
    pub fn with_watchdog(mut self, enabled: bool, timeout_ms: u64) -> Self {
        self.realtime.watchdog_enabled = enabled;
        self.realtime.watchdog_timeout_ms = timeout_ms;
        self
    }

    /// Set the CPU cores to use
    pub fn with_cpu_cores(mut self, cores: Vec<usize>) -> Self {
        self.resources.cpu_cores = Some(cores);
        self
    }

    /// Enable or disable profiling
    pub fn with_profiling(mut self, enabled: bool) -> Self {
        self.monitoring.profiling_enabled = enabled;
        self
    }

    /// Set the telemetry endpoint
    pub fn with_telemetry_endpoint(mut self, endpoint: Option<String>) -> Self {
        self.monitoring.telemetry_endpoint = endpoint;
        self
    }

    /// Enable or disable black box recording
    pub fn with_black_box(mut self, enabled: bool, size_mb: usize) -> Self {
        self.monitoring.black_box_enabled = enabled;
        self.monitoring.black_box_size_mb = size_mb;
        self
    }

    /// Set the deterministic configuration
    pub fn with_deterministic(mut self, config: Option<DeterministicConfig>) -> Self {
        self.deterministic = config;
        self
    }

    /// Set the recording configuration
    pub fn with_recording(mut self, config: Option<RecordingConfigYaml>) -> Self {
        self.recording = config;
        self
    }
}

/// Trait for converting from ConfigValue
pub trait FromConfigValue: Sized {
    fn from_config_value(value: &ConfigValue) -> Option<Self>;
}

impl FromConfigValue for bool {
    fn from_config_value(value: &ConfigValue) -> Option<Self> {
        match value {
            ConfigValue::Bool(b) => Some(*b),
            _ => None,
        }
    }
}

impl FromConfigValue for i64 {
    fn from_config_value(value: &ConfigValue) -> Option<Self> {
        match value {
            ConfigValue::Integer(i) => Some(*i),
            _ => None,
        }
    }
}

impl FromConfigValue for f64 {
    fn from_config_value(value: &ConfigValue) -> Option<Self> {
        match value {
            ConfigValue::Float(f) => Some(*f),
            _ => None,
        }
    }
}

impl FromConfigValue for String {
    fn from_config_value(value: &ConfigValue) -> Option<Self> {
        match value {
            ConfigValue::String(s) => Some(s.clone()),
            _ => None,
        }
    }
}
