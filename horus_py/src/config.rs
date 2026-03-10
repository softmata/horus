use horus::scheduling::config::{RecordingConfigYaml, SchedulerConfig};
use pyo3::prelude::*;

/// Full scheduler configuration for Python — plain data bag.
///
/// All fields from the Rust config are exposed. For common use cases,
/// prefer `Scheduler.deploy()`, `Scheduler.safety_critical()`, etc.
#[pyclass(name = "SchedulerConfig", module = "horus._horus")]
#[derive(Clone, Debug)]
pub struct PySchedulerConfig {
    // --- Timing ---
    #[pyo3(get, set)]
    /// Global tick rate in Hz
    pub tick_rate: f64,

    // --- Real-time ---
    #[pyo3(get, set)]
    /// Enable budget enforcement
    pub budget_enforcement: bool,

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

    // --- Monitoring ---
    #[pyo3(get, set)]
    /// Enable runtime profiling
    pub profiling: bool,

    #[pyo3(get, set)]
    /// Enable black box recording
    pub black_box_enabled: bool,

    #[pyo3(get, set)]
    /// Black box buffer size in MB
    pub black_box_size_mb: usize,

    #[pyo3(get, set)]
    /// Telemetry export endpoint
    pub telemetry_endpoint: Option<String>,

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
        Self::minimal()
    }

    /// Create a minimal configuration with sensible defaults.
    ///
    /// For common use cases, prefer Scheduler presets:
    /// `Scheduler.deploy()`, `Scheduler.safety_critical()`, etc.
    #[staticmethod]
    pub fn minimal() -> Self {
        let rust_config = SchedulerConfig::default();
        Self::from_rust_config(rust_config, "Minimal")
    }

    // ========================================================================
    // PROFILE PRESETS — recommended for common use cases
    // ========================================================================

    /// Deploy profile — safety monitor, watchdog, and blackbox.
    ///
    /// Suitable for production robots. Gracefully degrades if RT not available.
    ///
    /// Example:
    ///     cfg = SchedulerConfig.deploy()
    ///     scheduler = Scheduler(config=cfg)
    #[staticmethod]
    pub fn deploy() -> Self {
        let mut cfg = Self::minimal();
        cfg.tick_rate = 100.0;
        cfg.safety_monitor = true;
        cfg.budget_enforcement = true;
        cfg.deadline_monitoring = true;
        cfg.watchdog_enabled = true;
        cfg.watchdog_timeout_ms = 500;
        cfg.black_box_enabled = true;
        cfg.black_box_size_mb = 64;
        cfg.config_name = "Deploy".to_string();
        cfg
    }

    /// Hard real-time profile — all RT features enabled.
    ///
    /// Suitable for hard real-time control loops with strict timing guarantees.
    ///
    /// Example:
    ///     cfg = SchedulerConfig.hard_rt()
    ///     scheduler = Scheduler(config=cfg)
    #[staticmethod]
    pub fn hard_rt() -> Self {
        let mut cfg = Self::deploy();
        cfg.memory_locking = true;
        cfg.rt_scheduling_class = true;
        cfg.max_deadline_misses = 10;
        cfg.config_name = "HardRT".to_string();
        cfg
    }

    /// Safety-critical profile — hard RT plus profiling and strict deadlines.
    ///
    /// Suitable for safety-critical systems that must never miss a deadline.
    ///
    /// Example:
    ///     cfg = SchedulerConfig.safety_critical()
    ///     scheduler = Scheduler(config=cfg)
    #[staticmethod]
    pub fn safety_critical() -> Self {
        let mut cfg = Self::hard_rt();
        cfg.profiling = true;
        cfg.max_deadline_misses = 3;
        cfg.config_name = "SafetyCritical".to_string();
        cfg
    }

    // ========================================================================
    // COMPOUND BUILDER METHODS (set multiple fields or improve readability)
    // ========================================================================

    /// Set tick rate in Hz and return self for chaining.
    pub fn rate(&mut self, hz: f64) -> Self {
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

    /// Set black box buffer size in MB and enable it.
    pub fn blackbox_mb(&mut self, mb: usize) -> Self {
        self.black_box_enabled = true;
        self.black_box_size_mb = mb;
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

    fn __repr__(&self) -> String {
        format!(
            "SchedulerConfig(config={}, tick_rate={:.1}Hz, \
             budget={}, deadline_monitoring={}, safety_monitor={}, memory_locking={}, \
             recording={})",
            self.config_name,
            self.tick_rate,
            self.budget_enforcement,
            self.deadline_monitoring,
            self.safety_monitor,
            self.memory_locking,
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
        let mut config = SchedulerConfig::default();

        config.timing.global_rate_hz = self.tick_rate;

        config.realtime.budget_enforcement = self.budget_enforcement;
        config.realtime.deadline_monitoring = self.deadline_monitoring;
        config.realtime.watchdog_enabled = self.watchdog_enabled;
        config.realtime.watchdog_timeout_ms = self.watchdog_timeout_ms;
        config.realtime.safety_monitor = self.safety_monitor;
        config.realtime.max_deadline_misses = self.max_deadline_misses;
        config.realtime.memory_locking = self.memory_locking;
        config.realtime.rt_scheduling_class = self.rt_scheduling_class;

        config.resources.cpu_cores = self.cpu_cores.clone();

        config.monitoring.profiling_enabled = self.profiling;
        config.monitoring.black_box_enabled = self.black_box_enabled;
        config.monitoring.black_box_size_mb = self.black_box_size_mb;
        config.monitoring.telemetry_endpoint = self.telemetry_endpoint.clone();

        config.recording = if self.recording_enabled {
            Some(RecordingConfigYaml::full())
        } else {
            None
        };

        config
    }

    fn from_rust_config(rust_config: SchedulerConfig, name: &str) -> Self {
        PySchedulerConfig {
            tick_rate: rust_config.timing.global_rate_hz,
            budget_enforcement: rust_config.realtime.budget_enforcement,
            deadline_monitoring: rust_config.realtime.deadline_monitoring,
            watchdog_enabled: rust_config.realtime.watchdog_enabled,
            watchdog_timeout_ms: rust_config.realtime.watchdog_timeout_ms,
            safety_monitor: rust_config.realtime.safety_monitor,
            max_deadline_misses: rust_config.realtime.max_deadline_misses,
            memory_locking: rust_config.realtime.memory_locking,
            rt_scheduling_class: rust_config.realtime.rt_scheduling_class,
            cpu_cores: rust_config.resources.cpu_cores,
            profiling: rust_config.monitoring.profiling_enabled,
            black_box_enabled: rust_config.monitoring.black_box_enabled,
            black_box_size_mb: rust_config.monitoring.black_box_size_mb,
            telemetry_endpoint: rust_config.monitoring.telemetry_endpoint,
            recording_enabled: rust_config.recording.is_some(),
            config_name: name.to_string(),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn minimal_defaults() {
        let cfg = PySchedulerConfig::minimal();
        assert!(!cfg.budget_enforcement);
        assert!(!cfg.safety_monitor);
        assert!(!cfg.memory_locking);
        assert!(!cfg.recording_enabled);
        assert_eq!(cfg.config_name, "Minimal");
    }

    #[test]
    fn deploy_enables_safety_features() {
        let cfg = PySchedulerConfig::deploy();
        assert_eq!(cfg.tick_rate, 100.0);
        assert!(cfg.safety_monitor);
        assert!(cfg.budget_enforcement);
        assert!(cfg.deadline_monitoring);
        assert!(cfg.watchdog_enabled);
        assert!(cfg.black_box_enabled);
        assert_eq!(cfg.config_name, "Deploy");
    }

    #[test]
    fn hard_rt_extends_deploy() {
        let cfg = PySchedulerConfig::hard_rt();
        // Inherits deploy features
        assert!(cfg.safety_monitor);
        // Adds RT features
        assert!(cfg.memory_locking);
        assert!(cfg.rt_scheduling_class);
        assert_eq!(cfg.max_deadline_misses, 10);
    }

    #[test]
    fn safety_critical_extends_hard_rt() {
        let cfg = PySchedulerConfig::safety_critical();
        assert!(cfg.memory_locking);
        assert!(cfg.profiling);
        assert_eq!(cfg.max_deadline_misses, 3);
    }

    #[test]
    fn to_core_config_round_trip() {
        let py_cfg = PySchedulerConfig::deploy();
        let core_cfg = py_cfg.to_core_config();

        assert_eq!(core_cfg.timing.global_rate_hz, 100.0);
        assert!(core_cfg.realtime.budget_enforcement);
        assert!(core_cfg.realtime.deadline_monitoring);
        assert!(core_cfg.realtime.watchdog_enabled);
        assert!(core_cfg.realtime.safety_monitor);
        assert!(core_cfg.monitoring.black_box_enabled);
    }

    #[test]
    fn builder_methods_chain() {
        let cfg = PySchedulerConfig::minimal()
            .rate(50.0)
            .watchdog_ms(200)
            .blackbox_mb(32)
            .cpu_affinity(vec![0, 1]);

        assert_eq!(cfg.tick_rate, 50.0);
        assert!(cfg.watchdog_enabled);
        assert_eq!(cfg.watchdog_timeout_ms, 200);
        assert!(cfg.black_box_enabled);
        assert_eq!(cfg.black_box_size_mb, 32);
        assert_eq!(cfg.cpu_cores, Some(vec![0, 1]));
    }

    #[test]
    fn no_watchdog_disables_it() {
        let cfg = PySchedulerConfig::deploy().no_watchdog();
        assert!(!cfg.watchdog_enabled);
    }

    #[test]
    fn recording_enabled_generates_config() {
        let mut cfg = PySchedulerConfig::minimal();
        cfg.recording_enabled = true;
        let core = cfg.to_core_config();
        assert!(core.recording.is_some());
    }

    #[test]
    fn recording_disabled_no_config() {
        let cfg = PySchedulerConfig::minimal();
        let core = cfg.to_core_config();
        assert!(core.recording.is_none());
    }
}
