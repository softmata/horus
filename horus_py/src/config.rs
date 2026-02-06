use horus::scheduling::config::SchedulerConfig;
use pyo3::prelude::*;

/// Simplified scheduler configuration for Python
///
/// Provides a subset of Rust's SchedulerConfig with Python-friendly defaults.
/// Use the static constructor methods like `standard()`, `safety_critical()`, etc.
#[pyclass(module = "horus._horus")]
#[derive(Clone, Debug)]
pub struct PySchedulerConfig {
    #[pyo3(get, set)]
    /// Global tick rate in Hz (default: 60.0)
    pub tick_rate: f64,

    #[pyo3(get, set)]
    /// Enable circuit breaker pattern (default: True)
    pub circuit_breaker: bool,

    #[pyo3(get, set)]
    /// Max failures before circuit opens (default: 5)
    pub max_failures: u32,

    #[pyo3(get, set)]
    /// Enable deadline monitoring (default: False, set True for soft RT)
    pub deadline_monitoring: bool,

    #[pyo3(get, set)]
    /// Enable watchdog timers (default: False)
    pub watchdog_enabled: bool,

    #[pyo3(get, set)]
    /// Watchdog timeout in milliseconds (default: 1000)
    pub watchdog_timeout_ms: u64,

    #[pyo3(get, set)]
    /// Enable profiling (default: False)
    pub profiling: bool,

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
    ///     config = SchedulerConfig.builder().rate_hz(1000).watchdog_ms(100).build()
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

    /// Finalize configuration (no-op, for builder pattern).
    pub fn build(&self) -> Self {
        self.clone()
    }

    fn __repr__(&self) -> String {
        format!(
            "SchedulerConfig(config={}, tick_rate={:.1}Hz, circuit_breaker={})",
            self.config_name, self.tick_rate, self.circuit_breaker
        )
    }

    fn __str__(&self) -> String {
        self.__repr__()
    }
}

impl PySchedulerConfig {
    fn from_rust_config(rust_config: SchedulerConfig, name: &str) -> Self {
        PySchedulerConfig {
            tick_rate: rust_config.timing.global_rate_hz,
            circuit_breaker: rust_config.fault.circuit_breaker_enabled,
            max_failures: rust_config.fault.max_failures,
            deadline_monitoring: rust_config.realtime.deadline_monitoring,
            watchdog_enabled: rust_config.realtime.watchdog_enabled,
            watchdog_timeout_ms: rust_config.realtime.watchdog_timeout_ms,
            profiling: rust_config.monitoring.profiling_enabled,
            config_name: name.to_string(),
        }
    }
}
