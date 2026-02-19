use horus::{NodeInfo as CoreNodeInfo, NodeState as CoreNodeState};
use pyo3::exceptions::PyRuntimeError;
use pyo3::prelude::*;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};

/// Python wrapper for NodeState
#[pyclass(module = "horus._horus")]
#[derive(Clone, PartialEq, Eq)]
pub struct PyNodeState {
    #[pyo3(get)]
    pub name: String,
}

#[pymethods]
impl PyNodeState {
    #[new]
    fn new(name: String) -> Self {
        PyNodeState { name }
    }

    /// Uninitialized state constant
    #[classattr]
    const UNINITIALIZED: &'static str = "uninitialized";

    /// Initializing state constant
    #[classattr]
    const INITIALIZING: &'static str = "initializing";

    /// Running state constant
    #[classattr]
    const RUNNING: &'static str = "running";

    /// Paused state constant
    #[classattr]
    const PAUSED: &'static str = "paused";

    /// Stopping state constant
    #[classattr]
    const STOPPING: &'static str = "stopping";

    /// Stopped state constant
    #[classattr]
    const STOPPED: &'static str = "stopped";

    /// Error state constant
    #[classattr]
    const ERROR: &'static str = "error";

    /// Crashed state constant
    #[classattr]
    const CRASHED: &'static str = "crashed";

    fn __repr__(&self) -> String {
        format!("NodeState('{}')", self.name)
    }

    fn __str__(&self) -> String {
        self.name.clone()
    }

    fn __eq__(&self, other: &Self) -> bool {
        self.name == other.name
    }
}

impl From<&CoreNodeState> for PyNodeState {
    fn from(state: &CoreNodeState) -> Self {
        let name = match state {
            CoreNodeState::Uninitialized => "uninitialized",
            CoreNodeState::Initializing => "initializing",
            CoreNodeState::Running => "running",
            CoreNodeState::Paused => "paused",
            CoreNodeState::Stopping => "stopping",
            CoreNodeState::Stopped => "stopped",
            CoreNodeState::Error(_) => "error",
            CoreNodeState::Crashed(_) => "crashed",
        };
        PyNodeState::new(name.to_string())
    }
}

/// Python wrapper for NodeInfo
#[pyclass(module = "horus._horus")]
#[derive(Clone)]
pub struct PyNodeInfo {
    pub inner: Arc<Mutex<CoreNodeInfo>>,
    pub scheduler_running: Option<Arc<AtomicBool>>,
}

#[pymethods]
impl PyNodeInfo {
    #[new]
    fn new(name: String) -> Self {
        PyNodeInfo {
            inner: Arc::new(Mutex::new(CoreNodeInfo::new(name))),
            scheduler_running: None,
        }
    }

    #[getter]
    fn name(&self) -> PyResult<String> {
        let info = self
            .inner
            .lock()
            .map_err(|e| PyRuntimeError::new_err(format!("Failed to lock NodeInfo: {}", e)))?;
        Ok(info.name().to_string())
    }

    #[getter]
    fn state(&self) -> PyResult<PyNodeState> {
        let info = self
            .inner
            .lock()
            .map_err(|e| PyRuntimeError::new_err(format!("Failed to lock NodeInfo: {}", e)))?;
        Ok(PyNodeState::from(info.state()))
    }

    fn log_info(&self, message: String) -> PyResult<()> {
        // Use hlog!() for logging (thread-local, no NodeInfo needed)
        horus::hlog!(info, "{}", message);
        Ok(())
    }

    fn log_warning(&self, message: String) -> PyResult<()> {
        // Use hlog!() for logging and track in metrics
        horus::hlog!(warn, "{}", message);
        let mut info = self
            .inner
            .lock()
            .map_err(|e| PyRuntimeError::new_err(format!("Failed to lock NodeInfo: {}", e)))?;
        info.track_warning(&message);
        Ok(())
    }

    fn log_error(&self, message: String) -> PyResult<()> {
        // Use hlog!() for logging and track in metrics
        horus::hlog!(error, "{}", message);
        let mut info = self
            .inner
            .lock()
            .map_err(|e| PyRuntimeError::new_err(format!("Failed to lock NodeInfo: {}", e)))?;
        info.track_error(&message);
        Ok(())
    }

    fn log_debug(&self, message: String) -> PyResult<()> {
        // Use hlog!() for logging (thread-local, no NodeInfo needed)
        horus::hlog!(debug, "{}", message);
        Ok(())
    }

    fn set_custom_data(&self, key: String, value: String) -> PyResult<()> {
        let mut info = self
            .inner
            .lock()
            .map_err(|e| PyRuntimeError::new_err(format!("Failed to lock NodeInfo: {}", e)))?;
        info.set_custom_data(key, value);
        Ok(())
    }

    fn get_custom_data(&self, key: String) -> PyResult<Option<String>> {
        let info = self
            .inner
            .lock()
            .map_err(|e| PyRuntimeError::new_err(format!("Failed to lock NodeInfo: {}", e)))?;
        Ok(info.get_custom_data(&key).cloned())
    }

    /// Get total tick count
    fn tick_count(&self) -> PyResult<u64> {
        let info = self
            .inner
            .lock()
            .map_err(|e| PyRuntimeError::new_err(format!("Failed to lock NodeInfo: {}", e)))?;
        Ok(info.metrics().total_ticks)
    }

    /// Get error count
    fn error_count(&self) -> PyResult<u64> {
        let info = self
            .inner
            .lock()
            .map_err(|e| PyRuntimeError::new_err(format!("Failed to lock NodeInfo: {}", e)))?;
        Ok(info.metrics().errors_count)
    }

    /// Transition to error state
    fn transition_to_error(&self, error_msg: String) -> PyResult<()> {
        let mut info = self
            .inner
            .lock()
            .map_err(|e| PyRuntimeError::new_err(format!("Failed to lock NodeInfo: {}", e)))?;
        info.transition_to_error(error_msg);
        Ok(())
    }

    /// Log a publish operation with IPC timing
    fn log_pub(&self, topic: String, data_repr: String, ipc_ns: u64) -> PyResult<()> {
        // Take String (owned) instead of &str (borrowed) to avoid PyO3 borrow issues
        let info = self
            .inner
            .lock()
            .map_err(|e| PyRuntimeError::new_err(format!("Failed to lock NodeInfo: {}", e)))?;

        let timestamp = chrono::Local::now().format("%H:%M:%S%.3f").to_string();
        let node_name = info.name().to_string();

        // Calculate tick_us from tick_start_time
        let tick_us = info.tick_elapsed_us();

        // Write to global log buffer for monitor
        use horus::core::log_buffer::{publish_log, LogEntry, LogType};
        publish_log(LogEntry {
            timestamp,
            tick_number: 0, // Python nodes don't have deterministic tick counter
            node_name,
            log_type: LogType::Publish,
            topic: Some(topic),
            message: data_repr,
            tick_us,
            ipc_ns,
        });

        Ok(())
    }

    /// Log a subscribe operation with IPC timing
    fn log_sub(&self, topic: String, data_repr: String, ipc_ns: u64) -> PyResult<()> {
        // Take String (owned) instead of &str (borrowed) to avoid PyO3 borrow issues
        let info = self
            .inner
            .lock()
            .map_err(|e| PyRuntimeError::new_err(format!("Failed to lock NodeInfo: {}", e)))?;

        let timestamp = chrono::Local::now().format("%H:%M:%S%.3f").to_string();
        let node_name = info.name().to_string();

        // Calculate tick_us from tick_start_time
        let tick_us = info.tick_elapsed_us();

        // Write to global log buffer for monitor
        use horus::core::log_buffer::{publish_log, LogEntry, LogType};
        publish_log(LogEntry {
            timestamp,
            tick_number: 0, // Python nodes don't have deterministic tick counter
            node_name,
            log_type: LogType::Subscribe,
            topic: Some(topic),
            message: data_repr,
            tick_us,
            ipc_ns,
        });

        Ok(())
    }

    /// Get comprehensive metrics dictionary
    fn get_metrics(&self) -> PyResult<std::collections::HashMap<String, f64>> {
        let info = self
            .inner
            .lock()
            .map_err(|e| PyRuntimeError::new_err(format!("Failed to lock NodeInfo: {}", e)))?;

        let metrics = info.metrics();
        let mut result = std::collections::HashMap::new();

        result.insert("total_ticks".to_string(), metrics.total_ticks as f64);
        result.insert(
            "successful_ticks".to_string(),
            metrics.successful_ticks as f64,
        );
        result.insert("failed_ticks".to_string(), metrics.failed_ticks as f64);
        result.insert("errors_count".to_string(), metrics.errors_count as f64);
        result.insert(
            "avg_tick_duration_ms".to_string(),
            metrics.avg_tick_duration_ms,
        );
        result.insert(
            "min_tick_duration_ms".to_string(),
            metrics.min_tick_duration_ms,
        );
        result.insert(
            "max_tick_duration_ms".to_string(),
            metrics.max_tick_duration_ms,
        );
        result.insert(
            "last_tick_duration_ms".to_string(),
            metrics.last_tick_duration_ms,
        );

        Ok(result)
    }

    /// Get node uptime in seconds
    fn get_uptime(&self) -> PyResult<f64> {
        let info = self
            .inner
            .lock()
            .map_err(|e| PyRuntimeError::new_err(format!("Failed to lock NodeInfo: {}", e)))?;
        Ok(info.uptime().as_secs_f64())
    }

    /// Get average tick duration in milliseconds
    fn avg_tick_duration_ms(&self) -> PyResult<f64> {
        let info = self
            .inner
            .lock()
            .map_err(|e| PyRuntimeError::new_err(format!("Failed to lock NodeInfo: {}", e)))?;
        Ok(info.metrics().avg_tick_duration_ms)
    }

    /// Get number of failed ticks
    fn failed_ticks(&self) -> PyResult<u64> {
        let info = self
            .inner
            .lock()
            .map_err(|e| PyRuntimeError::new_err(format!("Failed to lock NodeInfo: {}", e)))?;
        Ok(info.metrics().failed_ticks)
    }

    /// Get successful ticks
    fn successful_ticks(&self) -> PyResult<u64> {
        let info = self
            .inner
            .lock()
            .map_err(|e| PyRuntimeError::new_err(format!("Failed to lock NodeInfo: {}", e)))?;
        Ok(info.metrics().successful_ticks)
    }

    /// Register a publisher topic for runtime discovery
    /// Note: Topic registration is now handled by TopicRegistry. This method is a no-op kept for API compatibility.
    fn register_publisher(&self, _topic: String, _msg_type: String) -> PyResult<()> {
        // Topic registration is now handled by TopicRegistry when Topics are created
        Ok(())
    }

    /// Register a subscriber topic for runtime discovery
    /// Note: Topic registration is now handled by TopicRegistry. This method is a no-op kept for API compatibility.
    fn register_subscriber(&self, _topic: String, _msg_type: String) -> PyResult<()> {
        // Topic registration is now handled by TopicRegistry when Topics are created
        Ok(())
    }

    /// Request the scheduler to stop
    fn request_stop(&self) -> PyResult<()> {
        if let Some(ref running_flag) = self.scheduler_running {
            running_flag.store(false, Ordering::SeqCst);
        }
        Ok(())
    }

    fn __repr__(&self) -> PyResult<String> {
        if let Ok(info) = self.inner.lock() {
            Ok(format!(
                "NodeInfo(name='{}', state='{}', ticks={}, errors={})",
                info.name(),
                info.state(),
                info.metrics().total_ticks,
                info.metrics().errors_count
            ))
        } else {
            Ok("NodeInfo(locked)".to_string())
        }
    }

    /// Pickle support: Provide constructor arguments
    fn __getnewargs__(&self) -> PyResult<(String,)> {
        let info = self
            .inner
            .lock()
            .map_err(|e| PyRuntimeError::new_err(format!("Failed to lock NodeInfo: {}", e)))?;
        Ok((info.name().to_string(),))
    }
}

/// Python wrapper for HORUS Node
///
/// This class allows Python code to implement HORUS nodes
/// by subclassing and implementing the required methods.
///
/// NOTE: PyNode no longer creates its own NodeInfo. The scheduler will provide one.
#[pyclass(module = "horus._horus", subclass)]
pub struct PyNode {
    #[pyo3(get)]
    pub name: String,
    pub py_callback: Option<Py<PyAny>>,
}

#[pymethods]
impl PyNode {
    #[new]
    pub fn new(name: String) -> PyResult<Self> {
        Ok(PyNode {
            name: name.clone(),
            py_callback: None,
        })
    }

    /// Initialize the node
    /// The scheduler passes NodeInfo, which we forward to the Python callback
    fn init(&mut self, py: Python, info: PyNodeInfo) -> PyResult<()> {
        if let Some(callback) = &self.py_callback {
            callback.call_method1(py, "init", (info,))?;
        }
        Ok(())
    }

    /// Main execution tick
    /// The scheduler passes NodeInfo, which we forward to the Python callback
    fn tick(&mut self, py: Python, info: PyNodeInfo) -> PyResult<()> {
        if let Some(callback) = &self.py_callback {
            callback.call_method1(py, "tick", (info,))?;
        }
        Ok(())
    }

    /// Shutdown the node
    /// The scheduler passes NodeInfo, which we forward to the Python callback
    fn shutdown(&mut self, py: Python, info: PyNodeInfo) -> PyResult<()> {
        if let Some(callback) = &self.py_callback {
            callback.call_method1(py, "shutdown", (info,))?;
        }
        Ok(())
    }

    /// Set the Python callback object (usually 'self' from Python subclass)
    fn set_callback(&mut self, callback: Py<PyAny>) -> PyResult<()> {
        self.py_callback = Some(callback);
        Ok(())
    }

    fn __repr__(&self) -> String {
        format!("Node(name='{}')", self.name)
    }

    /// Pickle support: Provide constructor arguments
    fn __getnewargs__(&self) -> PyResult<(String,)> {
        Ok((self.name.clone(),))
    }
}

// Bridge struct to implement the Rust Node trait
