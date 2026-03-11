use horus::scheduling::{RecordingConfigYaml, SchedulerConfig};
use pyo3::prelude::*;

/// Full scheduler configuration for Python — plain data bag.
///
/// All fields from the Rust config are exposed. Use builder methods
/// like `.watchdog()`, `.blackbox()` on the Scheduler, or construct
/// a config and pass it to `Scheduler(config=cfg)`.
#[pyclass(name = "SchedulerConfig", module = "horus._horus")]
#[derive(Clone, Debug)]
pub struct PySchedulerConfig {
    // --- Timing ---
    #[pyo3(get)]
    /// Global tick rate in Hz
    pub tick_rate: f64,

    // --- Real-time ---
    #[pyo3(get, set)]
    /// Watchdog timeout in milliseconds. 0 = disabled, >0 = enabled.
    pub watchdog_timeout_ms: u64,

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
    /// Black box buffer size in MB. 0 = disabled, >0 = enabled.
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

    /// Set the global tick rate in Hz.
    ///
    /// Raises `ValueError` for zero, negative, NaN, or infinite values.
    #[setter]
    pub fn set_tick_rate(&mut self, value: f64) -> PyResult<()> {
        if value <= 0.0 || !value.is_finite() {
            return Err(pyo3::exceptions::PyValueError::new_err(
                "tick_rate must be a positive finite number",
            ));
        }
        self.tick_rate = value;
        Ok(())
    }

    /// Create a minimal configuration with sensible defaults.
    #[staticmethod]
    pub fn minimal() -> Self {
        let rust_config = SchedulerConfig::default();
        Self::from_rust_config(rust_config, "Minimal")
    }

    // ========================================================================
    // CONFIG BUILDER METHODS — composable, explicit
    // ========================================================================

    /// Enable watchdog with 500ms timeout, budget enforcement, and deadline
    /// monitoring (implicit when nodes have `.rate()` set).
    ///
    /// Example:
    ///     cfg = SchedulerConfig.with_watchdog()
    ///     scheduler = Scheduler(config=cfg)
    #[staticmethod]
    pub fn with_watchdog() -> Self {
        let mut cfg = Self::minimal();
        cfg.watchdog_timeout_ms = 500;
        cfg.config_name = "Watchdog".to_string();
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

    /// Set watchdog timeout in milliseconds.
    pub fn watchdog_ms(&mut self, ms: u64) -> Self {
        self.watchdog_timeout_ms = ms;
        self.clone()
    }

    /// Disable watchdog.
    pub fn no_watchdog(&mut self) -> Self {
        self.watchdog_timeout_ms = 0;
        self.clone()
    }

    /// Set black box buffer size in MB.
    pub fn blackbox_mb(&mut self, mb: usize) -> Self {
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
             watchdog_timeout_ms={}, max_deadline_misses={}, memory_locking={}, \
             recording={})",
            self.config_name,
            self.tick_rate,
            self.watchdog_timeout_ms,
            self.max_deadline_misses,
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

        config.realtime.watchdog_timeout_ms = self.watchdog_timeout_ms;
        config.realtime.max_deadline_misses = self.max_deadline_misses;
        config.realtime.memory_locking = self.memory_locking;
        config.realtime.rt_scheduling_class = self.rt_scheduling_class;

        config.resources.cpu_cores = self.cpu_cores.clone();

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
            watchdog_timeout_ms: rust_config.realtime.watchdog_timeout_ms,
            max_deadline_misses: rust_config.realtime.max_deadline_misses,
            memory_locking: rust_config.realtime.memory_locking,
            rt_scheduling_class: rust_config.realtime.rt_scheduling_class,
            cpu_cores: rust_config.resources.cpu_cores,
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
        assert_eq!(cfg.watchdog_timeout_ms, 0);
        assert!(!cfg.memory_locking);
        assert!(!cfg.recording_enabled);
        assert_eq!(cfg.config_name, "Minimal");
    }

    #[test]
    fn with_watchdog_enables_safety_features() {
        let cfg = PySchedulerConfig::with_watchdog();
        assert_eq!(cfg.watchdog_timeout_ms, 500);
        assert_eq!(cfg.config_name, "Watchdog");
    }

    #[test]
    fn composable_rt_config() {
        let cfg = PySchedulerConfig::with_watchdog()
            .rate(100.0)
            .blackbox_mb(64);
        assert_eq!(cfg.watchdog_timeout_ms, 500);
        assert_eq!(cfg.black_box_size_mb, 64);
        assert_eq!(cfg.tick_rate, 100.0);
    }

    #[test]
    fn composable_strict_config() {
        let mut cfg = PySchedulerConfig::with_watchdog();
        cfg.memory_locking = true;
        cfg.rt_scheduling_class = true;
        cfg.max_deadline_misses = 3;
        assert!(cfg.memory_locking);
        assert_eq!(cfg.max_deadline_misses, 3);
    }

    #[test]
    fn to_core_config_round_trip() {
        let py_cfg = PySchedulerConfig::with_watchdog()
            .rate(100.0)
            .blackbox_mb(64);
        let core_cfg = py_cfg.to_core_config();

        assert_eq!(core_cfg.timing.global_rate_hz, 100.0);
        assert_eq!(core_cfg.realtime.watchdog_timeout_ms, 500);
        assert_eq!(core_cfg.monitoring.black_box_size_mb, 64);
    }

    #[test]
    fn builder_methods_chain() {
        let cfg = PySchedulerConfig::minimal()
            .rate(50.0)
            .watchdog_ms(200)
            .blackbox_mb(32)
            .cpu_affinity(vec![0, 1]);

        assert_eq!(cfg.tick_rate, 50.0);
        assert_eq!(cfg.watchdog_timeout_ms, 200);
        assert_eq!(cfg.black_box_size_mb, 32);
        assert_eq!(cfg.cpu_cores, Some(vec![0, 1]));
    }

    #[test]
    fn no_watchdog_disables_it() {
        let cfg = PySchedulerConfig::with_watchdog().no_watchdog();
        assert_eq!(cfg.watchdog_timeout_ms, 0);
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
