use horus::scheduling::config::{
    ExecutionMode, RecordingConfigYaml, SchedulerConfig, TopologyConfig,
};
use pyo3::prelude::*;

/// Full scheduler configuration for Python, faithfully mirroring horus_core's SchedulerConfig.
///
/// All fields from the Rust config are exposed so Python users get identical behavior
/// to Rust users when using presets like `safety_critical()` or `high_performance()`.
#[pyclass(module = "horus._horus")]
#[derive(Clone, Debug)]
pub struct PySchedulerConfig {
    // --- Execution ---
    #[pyo3(get, set)]
    /// Execution mode: "sequential" or "parallel"
    pub execution_mode: String,

    // --- Timing ---
    #[pyo3(get, set)]
    /// Global tick rate in Hz
    pub tick_rate: f64,

    #[pyo3(get, set)]
    /// Enable per-node rate control
    pub per_node_rates: bool,

    // --- Fault ---
    #[pyo3(get, set)]
    /// Enable circuit breaker pattern
    pub circuit_breaker: bool,

    #[pyo3(get, set)]
    /// Max failures before circuit opens
    pub max_failures: u32,

    #[pyo3(get, set)]
    /// Success count to close circuit
    pub recovery_threshold: u32,

    #[pyo3(get, set)]
    /// Circuit timeout in milliseconds
    pub circuit_timeout_ms: u64,

    // --- Real-time ---
    #[pyo3(get, set)]
    /// Enable WCET enforcement
    pub wcet_enforcement: bool,

    #[pyo3(get, set)]
    /// Enable deadline monitoring
    pub deadline_monitoring: bool,

    #[pyo3(get, set)]
    /// Enable watchdog timers
    pub watchdog_enabled: bool,

    #[pyo3(get, set)]
    /// Watchdog timeout in milliseconds
    pub watchdog_timeout_ms: u64,

    #[pyo3(get, set)]
    /// Enable safety monitor
    pub safety_monitor: bool,

    #[pyo3(get, set)]
    /// Maximum deadline misses before emergency stop
    pub max_deadline_misses: u64,

    #[pyo3(get, set)]
    /// Enable memory locking (mlockall)
    pub memory_locking: bool,

    #[pyo3(get, set)]
    /// Use real-time scheduling class (SCHED_FIFO/RR)
    pub rt_scheduling_class: bool,

    // --- Resources ---
    #[pyo3(get, set)]
    /// CPU cores to pin to (None = all cores)
    pub cpu_cores: Option<Vec<usize>>,

    #[pyo3(get, set)]
    /// Enable NUMA awareness
    pub numa_aware: bool,

    // --- Monitoring ---
    #[pyo3(get, set)]
    /// Enable runtime profiling
    pub profiling: bool,

    #[pyo3(get, set)]
    /// Metrics export interval in ms
    pub metrics_interval_ms: u64,

    #[pyo3(get, set)]
    /// Enable black box recording
    pub black_box_enabled: bool,

    #[pyo3(get, set)]
    /// Black box buffer size in MB
    pub black_box_size_mb: usize,

    #[pyo3(get, set)]
    /// Telemetry export endpoint
    pub telemetry_endpoint: Option<String>,

    // --- Deterministic ---
    #[pyo3(get, set)]
    /// Enable deterministic execution (uses DeterministicConfig::strict())
    pub deterministic_enabled: bool,

    // --- Recording ---
    #[pyo3(get, set)]
    /// Enable recording (uses RecordingConfigYaml::full())
    pub recording_enabled: bool,

    /// Internal: config name for repr
    config_name: String,
}

#[pymethods]
impl PySchedulerConfig {
    #[new]
    pub fn new() -> Self {
        Self::standard()
    }

    /// Create standard configuration (default)
    #[staticmethod]
    pub fn standard() -> Self {
        let rust_config = SchedulerConfig::standard();
        Self::from_rust_config(rust_config, "Standard")
    }

    /// Create safety-critical configuration
    #[staticmethod]
    pub fn safety_critical() -> Self {
        let rust_config = SchedulerConfig::safety_critical();
        Self::from_rust_config(rust_config, "SafetyCritical")
    }

    /// Create high-performance configuration
    #[staticmethod]
    pub fn high_performance() -> Self {
        let rust_config = SchedulerConfig::high_performance();
        Self::from_rust_config(rust_config, "HighPerformance")
    }

    /// Create hard real-time configuration
    #[staticmethod]
    pub fn hard_realtime() -> Self {
        let rust_config = SchedulerConfig::hard_realtime();
        Self::from_rust_config(rust_config, "HardRealTime")
    }

    /// Create deterministic configuration
    #[staticmethod]
    pub fn deterministic() -> Self {
        let rust_config = SchedulerConfig::deterministic();
        Self::from_rust_config(rust_config, "Deterministic")
    }

    /// Create a minimal configuration builder.
    ///
    /// Example:
    ///     config = SchedulerConfig.builder().rate_hz(1000).build()
    #[staticmethod]
    pub fn builder() -> Self {
        let rust_config = SchedulerConfig::minimal();
        Self::from_rust_config(rust_config, "Builder")
    }

    /// Create a minimal configuration (alias for builder).
    #[staticmethod]
    pub fn minimal() -> Self {
        Self::builder()
    }

    // ========================================================================
    // FLAT SETTERS (chainable methods matching Rust API)
    // ========================================================================

    /// Set tick rate in Hz and return self for chaining.
    pub fn rate_hz(&mut self, hz: f64) -> Self {
        self.tick_rate = hz;
        self.clone()
    }

    /// Set watchdog timeout in milliseconds and enable watchdog.
    pub fn watchdog_ms(&mut self, ms: u64) -> Self {
        self.watchdog_enabled = true;
        self.watchdog_timeout_ms = ms;
        self.clone()
    }

    /// Disable watchdog.
    pub fn no_watchdog(&mut self) -> Self {
        self.watchdog_enabled = false;
        self.clone()
    }

    /// Enable circuit breaker.
    pub fn enable_circuit_breaker(&mut self) -> Self {
        self.circuit_breaker = true;
        self.clone()
    }

    /// Disable circuit breaker.
    pub fn no_circuit_breaker(&mut self) -> Self {
        self.circuit_breaker = false;
        self.clone()
    }

    /// Enable deadline monitoring.
    pub fn enable_deadline_monitoring(&mut self) -> Self {
        self.deadline_monitoring = true;
        self.clone()
    }

    /// Enable profiling.
    pub fn enable_profiling(&mut self) -> Self {
        self.profiling = true;
        self.clone()
    }

    /// Set execution mode to parallel.
    pub fn parallel(&mut self) -> Self {
        self.execution_mode = "parallel".to_string();
        self.clone()
    }

    /// Set execution mode to sequential.
    pub fn sequential(&mut self) -> Self {
        self.execution_mode = "sequential".to_string();
        self.clone()
    }

    /// Enable WCET enforcement.
    pub fn wcet(&mut self) -> Self {
        self.wcet_enforcement = true;
        self.clone()
    }

    /// Enable memory locking (mlockall).
    pub fn memory_lock(&mut self) -> Self {
        self.memory_locking = true;
        self.clone()
    }

    /// Enable real-time scheduling class (SCHED_FIFO/RR).
    pub fn rt_scheduling(&mut self) -> Self {
        self.rt_scheduling_class = true;
        self.clone()
    }

    /// Set black box buffer size in MB and enable it.
    pub fn blackbox_mb(&mut self, mb: usize) -> Self {
        self.black_box_enabled = true;
        self.black_box_size_mb = mb;
        self.clone()
    }

    /// Enable safety monitor.
    pub fn enable_safety_monitor(&mut self) -> Self {
        self.safety_monitor = true;
        self.clone()
    }

    /// Set CPU core affinity.
    pub fn cpu_affinity(&mut self, cores: Vec<usize>) -> Self {
        self.cpu_cores = Some(cores);
        self.clone()
    }

    /// Set telemetry endpoint.
    pub fn telemetry(&mut self, endpoint: String) -> Self {
        self.telemetry_endpoint = Some(endpoint);
        self.clone()
    }

    /// Enable deterministic execution (DeterministicConfig::strict()).
    pub fn enable_deterministic(&mut self) -> Self {
        self.deterministic_enabled = true;
        self.clone()
    }

    /// Enable recording (RecordingConfigYaml::full()).
    pub fn enable_recording(&mut self) -> Self {
        self.recording_enabled = true;
        self.clone()
    }

    /// Enable per-node rate control.
    pub fn enable_per_node_rates(&mut self) -> Self {
        self.per_node_rates = true;
        self.clone()
    }

    /// Set max deadline misses before emergency stop.
    pub fn max_misses(&mut self, n: u64) -> Self {
        self.max_deadline_misses = n;
        self.clone()
    }

    /// Finalize configuration (no-op, for builder pattern).
    pub fn build(&self) -> Self {
        self.clone()
    }

    fn __repr__(&self) -> String {
        format!(
            "SchedulerConfig(config={}, mode={}, tick_rate={:.1}Hz, circuit_breaker={}, \
             wcet={}, deadline_monitoring={}, safety_monitor={}, memory_locking={}, \
             deterministic={}, recording={})",
            self.config_name,
            self.execution_mode,
            self.tick_rate,
            self.circuit_breaker,
            self.wcet_enforcement,
            self.deadline_monitoring,
            self.safety_monitor,
            self.memory_locking,
            self.deterministic_enabled,
            self.recording_enabled,
        )
    }

    fn __str__(&self) -> String {
        self.__repr__()
    }
}

impl PySchedulerConfig {
    /// Convert to horus_core's SchedulerConfig, faithfully representing ALL stored fields.
    pub fn to_core_config(&self) -> SchedulerConfig {
        let mut config = SchedulerConfig::minimal();

        config.execution = match self.execution_mode.as_str() {
            "parallel" => ExecutionMode::Parallel,
            _ => ExecutionMode::Sequential,
        };

        config.timing.global_rate_hz = self.tick_rate;
        config.timing.per_node_rates = self.per_node_rates;

        config.fault.circuit_breaker_enabled = self.circuit_breaker;
        config.fault.max_failures = self.max_failures;
        config.fault.recovery_threshold = self.recovery_threshold;
        config.fault.circuit_timeout_ms = self.circuit_timeout_ms;

        config.realtime.wcet_enforcement = self.wcet_enforcement;
        config.realtime.deadline_monitoring = self.deadline_monitoring;
        config.realtime.watchdog_enabled = self.watchdog_enabled;
        config.realtime.watchdog_timeout_ms = self.watchdog_timeout_ms;
        config.realtime.safety_monitor = self.safety_monitor;
        config.realtime.max_deadline_misses = self.max_deadline_misses;
        config.realtime.memory_locking = self.memory_locking;
        config.realtime.rt_scheduling_class = self.rt_scheduling_class;

        config.resources.cpu_cores = self.cpu_cores.clone();
        config.resources.numa_aware = self.numa_aware;

        config.monitoring.profiling_enabled = self.profiling;
        config.monitoring.metrics_interval_ms = self.metrics_interval_ms;
        config.monitoring.black_box_enabled = self.black_box_enabled;
        config.monitoring.black_box_size_mb = self.black_box_size_mb;
        config.monitoring.telemetry_endpoint = self.telemetry_endpoint.clone();

        config.topology = if self.deterministic_enabled {
            Some(TopologyConfig::strict())
        } else {
            None
        };

        config.recording = if self.recording_enabled {
            Some(RecordingConfigYaml::full())
        } else {
            None
        };

        config
    }

    fn from_rust_config(rust_config: SchedulerConfig, name: &str) -> Self {
        PySchedulerConfig {
            execution_mode: match rust_config.execution {
                ExecutionMode::Parallel => "parallel".to_string(),
                ExecutionMode::Sequential => "sequential".to_string(),
            },
            tick_rate: rust_config.timing.global_rate_hz,
            per_node_rates: rust_config.timing.per_node_rates,
            circuit_breaker: rust_config.fault.circuit_breaker_enabled,
            max_failures: rust_config.fault.max_failures,
            recovery_threshold: rust_config.fault.recovery_threshold,
            circuit_timeout_ms: rust_config.fault.circuit_timeout_ms,
            wcet_enforcement: rust_config.realtime.wcet_enforcement,
            deadline_monitoring: rust_config.realtime.deadline_monitoring,
            watchdog_enabled: rust_config.realtime.watchdog_enabled,
            watchdog_timeout_ms: rust_config.realtime.watchdog_timeout_ms,
            safety_monitor: rust_config.realtime.safety_monitor,
            max_deadline_misses: rust_config.realtime.max_deadline_misses,
            memory_locking: rust_config.realtime.memory_locking,
            rt_scheduling_class: rust_config.realtime.rt_scheduling_class,
            cpu_cores: rust_config.resources.cpu_cores,
            numa_aware: rust_config.resources.numa_aware,
            profiling: rust_config.monitoring.profiling_enabled,
            metrics_interval_ms: rust_config.monitoring.metrics_interval_ms,
            black_box_enabled: rust_config.monitoring.black_box_enabled,
            black_box_size_mb: rust_config.monitoring.black_box_size_mb,
            telemetry_endpoint: rust_config.monitoring.telemetry_endpoint,
            deterministic_enabled: rust_config.topology.is_some(),
            recording_enabled: rust_config.recording.is_some(),
            config_name: name.to_string(),
        }
    }
}
