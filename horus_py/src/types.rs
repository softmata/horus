use horus::NodeConfig as CoreNodeConfig;
use pyo3::exceptions::PyValueError;
use pyo3::prelude::*;
use std::collections::HashMap;

/// Python wrapper for messages
#[pyclass]
#[derive(Clone)]
pub struct PyMessage {
    #[pyo3(get, set)]
    pub data: Vec<u8>,
    #[pyo3(get, set)]
    pub topic: String,
    #[pyo3(get, set)]
    pub timestamp: f64,
    #[pyo3(get, set)]
    pub metadata: HashMap<String, String>,
}

#[pymethods]
impl PyMessage {
    #[new]
    fn new(data: Vec<u8>, topic: String) -> Self {
        PyMessage {
            data,
            topic,
            timestamp: std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap_or_default()
                .as_secs_f64(),
            metadata: HashMap::new(),
        }
    }

    fn set_metadata_item(&mut self, key: String, value: String) {
        self.metadata.insert(key, value);
    }

    fn get_metadata_item(&self, key: String) -> Option<String> {
        self.metadata.get(&key).cloned()
    }

    fn __repr__(&self) -> String {
        format!(
            "Message(topic='{}', size={} bytes, timestamp={})",
            self.topic,
            self.data.len(),
            self.timestamp
        )
    }

    fn __len__(&self) -> usize {
        self.data.len()
    }
}

/// Priority constants for Python users
///
/// Priorities are now represented as u32 values for maximum flexibility.
/// Lower numbers = higher priority. You can use any u32 value, but here
/// are some common presets:
///
/// - PRIORITY_CRITICAL = 0 (highest priority, for emergency/safety nodes)
/// - PRIORITY_HIGH = 10 (for important control loops)
/// - PRIORITY_NORMAL = 50 (default priority for most nodes)
/// - PRIORITY_LOW = 80 (for background processing)
/// - PRIORITY_BACKGROUND = 100 (lowest priority, for logging/telemetry)
///
/// You can also use any custom value between 0-255 for fine-grained control.
#[pyclass]
pub struct Priority;

#[pymethods]
impl Priority {
    #[classattr]
    const CRITICAL: u32 = 0;

    #[classattr]
    const HIGH: u32 = 10;

    #[classattr]
    const NORMAL: u32 = 50;

    #[classattr]
    const LOW: u32 = 80;

    #[classattr]
    const BACKGROUND: u32 = 100;

    /// Parse a priority string into a u32 value
    #[staticmethod]
    fn from_string(priority: String) -> PyResult<u32> {
        match priority.to_lowercase().as_str() {
            "critical" => Ok(0),
            "high" => Ok(10),
            "normal" => Ok(50),
            "low" => Ok(80),
            "background" => Ok(100),
            _ => {
                // Try parsing as number
                priority.parse::<u32>().map_err(|_| {
                    PyValueError::new_err(format!(
                        "Invalid priority '{}'. Use 'critical', 'high', 'normal', 'low', 'background', or a numeric value (0-255)",
                        priority
                    ))
                })
            }
        }
    }

    /// Convert a u32 priority to a descriptive string
    #[staticmethod]
    fn to_string(priority: u32) -> String {
        match priority {
            0 => "critical".to_string(),
            10 => "high".to_string(),
            50 => "normal".to_string(),
            80 => "low".to_string(),
            100 => "background".to_string(),
            n => format!("custom({})", n),
        }
    }
}

/// Python wrapper for NodeConfig
#[pyclass]
#[derive(Clone)]
pub struct PyNodeConfig {
    #[pyo3(get, set)]
    pub max_tick_duration_ms: Option<u64>,
    #[pyo3(get, set)]
    pub restart_on_failure: bool,
    #[pyo3(get, set)]
    pub max_restart_attempts: u32,
    #[pyo3(get, set)]
    pub restart_delay_ms: u64,
    #[pyo3(get, set)]
    pub log_level: String,
    #[pyo3(get, set)]
    pub custom_params: HashMap<String, String>,
}

#[pymethods]
impl PyNodeConfig {
    #[new]
    fn new() -> Self {
        let config = CoreNodeConfig::default();
        PyNodeConfig {
            max_tick_duration_ms: config.max_tick_duration_ms,
            restart_on_failure: config.restart_on_failure,
            max_restart_attempts: config.max_restart_attempts,
            restart_delay_ms: config.restart_delay_ms,
            log_level: config.log_level,
            custom_params: config.custom_params,
        }
    }

    fn set_param(&mut self, key: String, value: String) {
        self.custom_params.insert(key, value);
    }

    fn get_param(&self, key: String) -> Option<String> {
        self.custom_params.get(&key).cloned()
    }

    fn __repr__(&self) -> String {
        format!("NodeConfig(log_level='{}')", self.log_level)
    }
}

impl From<PyNodeConfig> for CoreNodeConfig {
    fn from(py_config: PyNodeConfig) -> Self {
        CoreNodeConfig {
            max_tick_duration_ms: py_config.max_tick_duration_ms,
            restart_on_failure: py_config.restart_on_failure,
            max_restart_attempts: py_config.max_restart_attempts,
            restart_delay_ms: py_config.restart_delay_ms,
            log_level: py_config.log_level,
            custom_params: py_config.custom_params,
            circuit_breaker: None, // Use scheduler-level circuit breaker settings
        }
    }
}
