// Type-based Hub implementation for Python bindings
//
// New API matches Rust exactly:
//   from horus import Hub, CmdVel, Pose2D
//   hub = Hub(CmdVel)  # Type determines everything
//
// Network support (NEW):
//   hub = Hub(CmdVel, endpoint="cmdvel@192.168.1.5:9000")  # Direct UDP
//   hub = Hub(CmdVel, endpoint="cmdvel@localhost")         # Unix socket
//   hub = Hub(CmdVel, endpoint="cmdvel@router")            # Via router
//   hub = Hub(CmdVel, endpoint="cmdvel@*")                 # Multicast

use horus::communication::hub::Hub;
use horus_library::messages::cmd_vel::CmdVel;
use horus_library::messages::geometry::Pose2D;
use horus_library::messages::GenericMessage;
use pyo3::prelude::*;
use std::sync::{Arc, Mutex};

/// Internal enum tracking which Rust type the Hub wraps
enum HubType {
    CmdVel(Arc<Mutex<Hub<CmdVel>>>),
    Pose2D(Arc<Mutex<Hub<Pose2D>>>),
    Generic(Arc<Mutex<Hub<GenericMessage>>>),
}

/// Python Hub - type-safe wrapper that creates the right Rust Hub<T>
///
/// Examples:
///     hub = Hub(CmdVel)       # Creates Hub<CmdVel> - zero overhead!
///     hub = Hub(Pose2D)       # Creates Hub<Pose2D>
///     hub = Hub("custom")     # Generic hub (fallback, slower)
///
/// Network examples:
///     hub = Hub(CmdVel, endpoint="cmdvel@192.168.1.5:9000")  # Direct UDP
///     hub = Hub(CmdVel, endpoint="cmdvel@localhost")         # Unix socket
///     hub = Hub(CmdVel, endpoint="cmdvel@router:7777")       # Via router
///     hub = Hub(CmdVel, endpoint="cmdvel@*")                 # Multicast
#[pyclass(name = "Hub")] // Export as "Hub" in Python, not "PyHub"
pub struct PyHub {
    hub_type: HubType,
    topic: String,
    /// Endpoint string used to create this hub (if network)
    endpoint: Option<String>,
    /// Whether this hub uses network transport
    is_network: bool,
}

#[pymethods]
impl PyHub {
    /// Create a new Hub for a specific message type
    ///
    /// Args:
    ///     msg_type: Message class (CmdVel, Pose2D) or string for generic hub
    ///     capacity: Optional buffer capacity (default: 1024 if not specified)
    ///     endpoint: Optional network endpoint string for distributed communication
    ///
    /// Endpoint formats:
    ///     "topic"                    - Local shared memory (default)
    ///     "topic@host:port"          - Direct UDP to specific host
    ///     "topic@localhost"          - Unix domain socket (Unix only)
    ///     "topic@router"             - Via HORUS router (TCP broker)
    ///     "topic@router:port"        - Via router on specific port
    ///     "topic@*"                  - Multicast discovery
    ///
    /// Examples:
    ///     hub = Hub(CmdVel)                                    # Local, default capacity
    ///     hub = Hub(Pose2D, capacity=2048)                     # Local, custom capacity
    ///     hub = Hub(CmdVel, endpoint="cmdvel@192.168.1.5:9000") # Network UDP
    ///     hub = Hub(CmdVel, endpoint="cmdvel@localhost")       # Unix socket
    ///     hub = Hub(CmdVel, endpoint="cmdvel@router")          # Via router
    ///     hub = Hub("custom")                                  # Generic hub
    #[new]
    #[pyo3(signature = (msg_type, capacity=None, endpoint=None))]
    fn new(
        py: Python,
        msg_type: PyObject,
        capacity: Option<usize>,
        endpoint: Option<String>,
    ) -> PyResult<Self> {
        // Get type name from the Python object
        let type_name = if let Ok(name) = msg_type.getattr(py, "__name__") {
            name.extract::<String>(py)?
        } else if let Ok(s) = msg_type.extract::<String>(py) {
            s // String fallback for generic hubs
        } else {
            return Err(pyo3::exceptions::PyTypeError::new_err(
                "Hub() requires a message type (CmdVel, Pose2D) or topic string",
            ));
        };

        // Get topic name from type's __topic_name__, or default to lowercase type name
        let topic = if let Ok(topic_attr) = msg_type.getattr(py, "__topic_name__") {
            topic_attr.extract::<String>(py)?
        } else {
            type_name.to_lowercase()
        };

        // Determine the effective endpoint:
        // - If endpoint is provided, use it directly
        // - Otherwise, use just the topic name (local shared memory)
        let effective_endpoint = endpoint.clone().unwrap_or_else(|| topic.clone());
        let is_network = endpoint.as_ref().is_some_and(|e| e.contains('@'));
        let cap = capacity.unwrap_or(1024);

        // Create the appropriate typed Hub
        // new_with_capacity() automatically parses the endpoint string and creates
        // network backends when needed (e.g., "topic@host:port")
        let hub_type = match type_name.as_str() {
            "CmdVel" => {
                let hub =
                    Hub::<CmdVel>::new_with_capacity(&effective_endpoint, cap).map_err(|e| {
                        pyo3::exceptions::PyRuntimeError::new_err(format!(
                            "Failed to create Hub<CmdVel>: {}",
                            e
                        ))
                    })?;
                HubType::CmdVel(Arc::new(Mutex::new(hub)))
            }
            "Pose2D" => {
                let hub =
                    Hub::<Pose2D>::new_with_capacity(&effective_endpoint, cap).map_err(|e| {
                        pyo3::exceptions::PyRuntimeError::new_err(format!(
                            "Failed to create Hub<Pose2D>: {}",
                            e
                        ))
                    })?;
                HubType::Pose2D(Arc::new(Mutex::new(hub)))
            }
            _ => {
                // Fallback to GenericMessage for unknown types
                let hub = Hub::<GenericMessage>::new_with_capacity(&effective_endpoint, cap)
                    .map_err(|e| {
                        pyo3::exceptions::PyRuntimeError::new_err(format!(
                            "Failed to create Hub<GenericMessage>: {}",
                            e
                        ))
                    })?;
                HubType::Generic(Arc::new(Mutex::new(hub)))
            }
        };

        Ok(Self {
            hub_type,
            topic,
            endpoint,
            is_network,
        })
    }

    /// Send a message (type must match Hub's type)
    ///
    /// Args:
    ///     message: Message object (CmdVel, Pose2D, etc.)
    ///     node: Optional Node for automatic logging with IPC timing
    ///
    /// Returns:
    ///     True if sent successfully, False otherwise
    ///
    /// Examples:
    ///     hub.send(CmdVel(1.5, 0.5), node)      # With logging
    ///     hub.send(Pose2D(1.0, 2.0, 0.5))       # Without logging
    #[pyo3(signature = (message, node=None))]
    fn send(&self, py: Python, message: PyObject, node: Option<PyObject>) -> PyResult<bool> {
        use std::time::Instant;
        let start = Instant::now();

        let result = match &self.hub_type {
            HubType::CmdVel(hub) => {
                // Extract fields from Python CmdVel object
                let linear: f32 = message.getattr(py, "linear")?.extract(py)?;
                let angular: f32 = message.getattr(py, "angular")?.extract(py)?;
                let stamp_nanos: u64 = message.getattr(py, "timestamp")?.extract(py)?;

                // Create Rust CmdVel - zero-copy!
                let cmd = CmdVel::with_timestamp(linear, angular, stamp_nanos);

                // Send via typed Hub<CmdVel>
                let hub = hub.lock().unwrap();
                let success = hub.send(cmd, &mut None).is_ok();

                // Log if node provided (use LogSummary trait!)
                if let Some(node_obj) = &node {
                    let ipc_ns = start.elapsed().as_nanos() as u64;
                    if let Ok(info) = node_obj.getattr(py, "info") {
                        if !info.is_none(py) {
                            // Register publisher for runtime discovery
                            let _ = info.call_method1(
                                py,
                                "register_publisher",
                                (&self.topic, "CmdVel"),
                            );
                            use horus::core::LogSummary;
                            let log_msg = cmd.log_summary();
                            let _ =
                                info.call_method1(py, "log_pub", (&self.topic, log_msg, ipc_ns));
                        }
                    }
                }

                success
            }
            HubType::Pose2D(hub) => {
                // Extract fields from Python Pose2D object
                let x: f64 = message.getattr(py, "x")?.extract(py)?;
                let y: f64 = message.getattr(py, "y")?.extract(py)?;
                let theta: f64 = message.getattr(py, "theta")?.extract(py)?;
                let timestamp: u64 = message.getattr(py, "timestamp")?.extract(py)?;

                // Create Rust Pose2D - zero-copy!
                let pose = Pose2D {
                    x,
                    y,
                    theta,
                    timestamp,
                };

                // Send via typed Hub<Pose2D>
                let hub = hub.lock().unwrap();
                let success = hub.send(pose, &mut None).is_ok();

                // Log if node provided (use LogSummary trait!)
                if let Some(node_obj) = &node {
                    let ipc_ns = start.elapsed().as_nanos() as u64;
                    if let Ok(info) = node_obj.getattr(py, "info") {
                        if !info.is_none(py) {
                            // Register publisher for runtime discovery
                            let _ = info.call_method1(
                                py,
                                "register_publisher",
                                (&self.topic, "Pose2D"),
                            );
                            use horus::core::LogSummary;
                            let log_msg = pose.log_summary();
                            let _ =
                                info.call_method1(py, "log_pub", (&self.topic, log_msg, ipc_ns));
                        }
                    }
                }

                success
            }
            HubType::Generic(hub) => {
                // Convert Python object to MessagePack via pythonize
                let bound = message.bind(py);
                let value: serde_json::Value = pythonize::depythonize(bound).map_err(|e| {
                    pyo3::exceptions::PyTypeError::new_err(format!(
                        "Failed to convert Python object: {}",
                        e
                    ))
                })?;

                // Serialize to MessagePack
                let msgpack_bytes = rmp_serde::to_vec(&value).map_err(|e| {
                    pyo3::exceptions::PyRuntimeError::new_err(format!(
                        "Failed to serialize to MessagePack: {}",
                        e
                    ))
                })?;

                // Create GenericMessage (with size validation)
                let msg = GenericMessage::new(msgpack_bytes)
                    .map_err(pyo3::exceptions::PyValueError::new_err)?;

                // Send via Hub<GenericMessage>
                let hub = hub.lock().unwrap();
                let success = hub.send(msg, &mut None).is_ok();

                // Log if node provided
                if let Some(node_obj) = &node {
                    let ipc_ns = start.elapsed().as_nanos() as u64;
                    if let Ok(info) = node_obj.getattr(py, "info") {
                        if !info.is_none(py) {
                            // Register publisher for runtime discovery
                            let _ = info.call_method1(
                                py,
                                "register_publisher",
                                (&self.topic, "GenericMessage"),
                            );
                            use horus::core::LogSummary;
                            let log_msg = msg.log_summary();
                            let _ =
                                info.call_method1(py, "log_pub", (&self.topic, log_msg, ipc_ns));
                        }
                    }
                }

                success
            }
        };

        Ok(result)
    }

    /// Receive a message (returns typed object matching Hub's type)
    ///
    /// Args:
    ///     node: Optional Node for automatic logging with IPC timing
    ///
    /// Returns:
    ///     CmdVel/Pose2D object if available, None otherwise
    ///
    /// Examples:
    ///     cmd = hub.recv(node)       # With logging
    ///     pose = hub.recv()          # Without logging
    #[pyo3(signature = (node=None))]
    fn recv(&self, py: Python, node: Option<PyObject>) -> PyResult<Option<PyObject>> {
        use std::time::Instant;
        let start = Instant::now();

        match &self.hub_type {
            HubType::CmdVel(hub) => {
                let hub = hub.lock().unwrap();
                if let Some(cmd) = hub.recv(&mut None) {
                    let ipc_ns = start.elapsed().as_nanos() as u64;

                    // Log if node provided (use LogSummary trait!)
                    if let Some(node_obj) = &node {
                        if let Ok(info) = node_obj.getattr(py, "info") {
                            if !info.is_none(py) {
                                // Register subscriber for runtime discovery
                                let _ = info.call_method1(
                                    py,
                                    "register_subscriber",
                                    (&self.topic, "CmdVel"),
                                );
                                use horus::core::LogSummary;
                                let log_msg = cmd.log_summary();
                                let _ = info.call_method1(
                                    py,
                                    "log_sub",
                                    (&self.topic, log_msg, ipc_ns),
                                );
                            }
                        }
                    }

                    // Create Python CmdVel object
                    let horus_module = py.import("horus")?;
                    let cmdvel_class = horus_module.getattr("CmdVel")?;
                    let py_cmd = cmdvel_class.call1((cmd.linear, cmd.angular, cmd.stamp_nanos))?;
                    Ok(Some(py_cmd.into()))
                } else {
                    Ok(None)
                }
            }
            HubType::Pose2D(hub) => {
                let hub = hub.lock().unwrap();
                if let Some(pose) = hub.recv(&mut None) {
                    let ipc_ns = start.elapsed().as_nanos() as u64;

                    // Log if node provided (use LogSummary trait!)
                    if let Some(node_obj) = &node {
                        if let Ok(info) = node_obj.getattr(py, "info") {
                            if !info.is_none(py) {
                                // Register subscriber for runtime discovery
                                let _ = info.call_method1(
                                    py,
                                    "register_subscriber",
                                    (&self.topic, "Pose2D"),
                                );
                                use horus::core::LogSummary;
                                let log_msg = pose.log_summary();
                                let _ = info.call_method1(
                                    py,
                                    "log_sub",
                                    (&self.topic, log_msg, ipc_ns),
                                );
                            }
                        }
                    }

                    // Create Python Pose2D object
                    let horus_module = py.import("horus")?;
                    let pose2d_class = horus_module.getattr("Pose2D")?;
                    let py_pose =
                        pose2d_class.call1((pose.x, pose.y, pose.theta, pose.timestamp))?;
                    Ok(Some(py_pose.into()))
                } else {
                    Ok(None)
                }
            }
            HubType::Generic(hub) => {
                let hub = hub.lock().unwrap();
                if let Some(msg) = hub.recv(&mut None) {
                    let ipc_ns = start.elapsed().as_nanos() as u64;

                    // Log if node provided
                    if let Some(node_obj) = &node {
                        if let Ok(info) = node_obj.getattr(py, "info") {
                            if !info.is_none(py) {
                                // Register subscriber for runtime discovery
                                let _ = info.call_method1(
                                    py,
                                    "register_subscriber",
                                    (&self.topic, "GenericMessage"),
                                );
                                use horus::core::LogSummary;
                                let log_msg = msg.log_summary();
                                let _ = info.call_method1(
                                    py,
                                    "log_sub",
                                    (&self.topic, log_msg, ipc_ns),
                                );
                            }
                        }
                    }

                    // Deserialize MessagePack to serde_json::Value
                    let data = msg.data();
                    let value: serde_json::Value = rmp_serde::from_slice(&data).map_err(|e| {
                        pyo3::exceptions::PyRuntimeError::new_err(format!(
                            "Failed to deserialize MessagePack: {}",
                            e
                        ))
                    })?;

                    // Convert serde Value to Python object
                    let py_obj = pythonize::pythonize(py, &value).map_err(|e| {
                        pyo3::exceptions::PyRuntimeError::new_err(format!(
                            "Failed to convert to Python: {}",
                            e
                        ))
                    })?;

                    Ok(Some(py_obj.into()))
                } else {
                    Ok(None)
                }
            }
        }
    }

    /// Get the topic name
    fn topic(&self) -> String {
        self.topic.clone()
    }

    /// Send raw bytes (for generic Python hubs)
    ///
    /// Args:
    ///     data: Raw bytes to send
    ///     node: Optional Node for automatic logging with IPC timing
    ///
    /// Returns:
    ///     True if sent successfully
    #[pyo3(signature = (data, node=None))]
    fn send_bytes(&self, py: Python, data: Vec<u8>, node: Option<PyObject>) -> PyResult<bool> {
        use std::time::Instant;
        let start = Instant::now();

        // Generic hubs only - wrap bytes in GenericMessage
        match &self.hub_type {
            HubType::Generic(hub) => {
                let msg =
                    GenericMessage::new(data).map_err(pyo3::exceptions::PyValueError::new_err)?;
                let hub = hub.lock().unwrap();
                let success = hub.send(msg, &mut None).is_ok();

                // Log if node provided
                if let Some(node_obj) = &node {
                    let ipc_ns = start.elapsed().as_nanos() as u64;
                    if let Ok(info) = node_obj.getattr(py, "info") {
                        if !info.is_none(py) {
                            use horus::core::LogSummary;
                            let log_msg = msg.log_summary();
                            let _ =
                                info.call_method1(py, "log_pub", (&self.topic, log_msg, ipc_ns));
                        }
                    }
                }

                Ok(success)
            }
            _ => Err(pyo3::exceptions::PyTypeError::new_err(
                "send_bytes() only supported for generic hubs",
            )),
        }
    }

    /// Send data with metadata (for generic Python hubs)
    ///
    /// Args:
    ///     data: Raw bytes to send
    ///     metadata: Metadata string (e.g., "json", "pickle", "numpy")
    ///     node: Optional Node for automatic logging with IPC timing
    ///
    /// Returns:
    ///     True if sent successfully
    #[pyo3(signature = (data, _metadata, node=None))]
    fn send_with_metadata(
        &self,
        py: Python,
        data: Vec<u8>,
        _metadata: String,
        node: Option<PyObject>,
    ) -> PyResult<bool> {
        use std::time::Instant;
        let start = Instant::now();

        // Create GenericMessage with metadata
        match &self.hub_type {
            HubType::Generic(hub) => {
                let msg = if _metadata.is_empty() {
                    GenericMessage::new(data)
                } else {
                    GenericMessage::with_metadata(data, _metadata)
                }
                .map_err(pyo3::exceptions::PyValueError::new_err)?;
                let hub = hub.lock().unwrap();
                let success = hub.send(msg, &mut None).is_ok();

                // Log if node provided
                if let Some(node_obj) = &node {
                    let ipc_ns = start.elapsed().as_nanos() as u64;
                    if let Ok(info) = node_obj.getattr(py, "info") {
                        if !info.is_none(py) {
                            use horus::core::LogSummary;
                            let log_msg = msg.log_summary();
                            let _ =
                                info.call_method1(py, "log_pub", (&self.topic, log_msg, ipc_ns));
                        }
                    }
                }

                Ok(success)
            }
            _ => Err(pyo3::exceptions::PyTypeError::new_err(
                "send_with_metadata() only supported for generic hubs",
            )),
        }
    }

    /// Send numpy array (for generic Python hubs)
    ///
    /// Args:
    ///     data: Numpy array (as bytes from Python)
    ///     node: Optional Node for automatic logging with IPC timing
    ///
    /// Returns:
    ///     True if sent successfully
    #[pyo3(signature = (data, node=None))]
    fn send_numpy(&self, py: Python, data: PyObject, node: Option<PyObject>) -> PyResult<bool> {
        use std::time::Instant;
        let start = Instant::now();

        // Extract numpy array bytes using buffer protocol
        match &self.hub_type {
            HubType::Generic(hub) => {
                // Try to get bytes from the numpy array
                let bytes: Vec<u8> = if let Ok(bytes_obj) = data.call_method0(py, "tobytes") {
                    bytes_obj.extract(py)?
                } else {
                    // Fallback: try to extract as bytes directly
                    data.extract(py)?
                };

                let msg =
                    GenericMessage::new(bytes).map_err(pyo3::exceptions::PyValueError::new_err)?;
                let hub = hub.lock().unwrap();
                let success = hub.send(msg, &mut None).is_ok();

                // Log if node provided
                if let Some(node_obj) = &node {
                    let ipc_ns = start.elapsed().as_nanos() as u64;
                    if let Ok(info) = node_obj.getattr(py, "info") {
                        if !info.is_none(py) {
                            use horus::core::LogSummary;
                            let log_msg = msg.log_summary();
                            let _ =
                                info.call_method1(py, "log_pub", (&self.topic, log_msg, ipc_ns));
                        }
                    }
                }

                Ok(success)
            }
            _ => Err(pyo3::exceptions::PyTypeError::new_err(
                "send_numpy() only supported for generic hubs",
            )),
        }
    }

    /// Receive data with metadata (for generic Python hubs)
    ///
    /// Args:
    ///     node: Optional Node for automatic logging with IPC timing
    ///
    /// Returns:
    ///     Tuple of (bytes, metadata_str, timestamp) or None
    #[pyo3(signature = (node=None))]
    fn recv_with_metadata(
        &self,
        py: Python,
        node: Option<PyObject>,
    ) -> PyResult<Option<(PyObject, String, f64)>> {
        use std::time::Instant;
        let start = Instant::now();

        match &self.hub_type {
            HubType::Generic(hub) => {
                let hub = hub.lock().unwrap();
                if let Some(msg) = hub.recv(&mut None) {
                    let ipc_ns = start.elapsed().as_nanos() as u64;

                    // Log if node provided
                    if let Some(node_obj) = &node {
                        if let Ok(info) = node_obj.getattr(py, "info") {
                            if !info.is_none(py) {
                                use horus::core::LogSummary;
                                let log_msg = msg.log_summary();
                                let _ = info.call_method1(
                                    py,
                                    "log_sub",
                                    (&self.topic, log_msg, ipc_ns),
                                );
                            }
                        }
                    }

                    // Use current time as timestamp (GenericMessage doesn't have timestamp field)
                    use std::time::{SystemTime, UNIX_EPOCH};
                    let timestamp = SystemTime::now()
                        .duration_since(UNIX_EPOCH)
                        .unwrap()
                        .as_secs_f64();

                    // Check if message has metadata, otherwise default to "json"
                    let metadata = msg.metadata().unwrap_or_else(|| "json".to_string());

                    // Convert Vec<u8> to Python bytes object
                    let data = msg.data();
                    let py_bytes = pyo3::types::PyBytes::new(py, &data).into();

                    Ok(Some((py_bytes, metadata, timestamp)))
                } else {
                    Ok(None)
                }
            }
            _ => Err(pyo3::exceptions::PyTypeError::new_err(
                "recv_with_metadata() only supported for generic hubs",
            )),
        }
    }

    /// Check if this hub is a generic hub (supports metadata methods)
    ///
    /// Returns:
    ///     True if this is a generic hub, False if it's a typed hub
    fn is_generic(&self) -> PyResult<bool> {
        Ok(matches!(self.hub_type, HubType::Generic(_)))
    }

    // === Network Information Methods ===

    /// Check if this hub uses network transport
    ///
    /// Returns:
    ///     True if the hub is communicating over the network, False for local shared memory
    ///
    /// Example:
    ///     hub = Hub(CmdVel, endpoint="cmdvel@192.168.1.5:9000")
    ///     print(hub.is_network())  # True
    #[getter]
    fn is_network_hub(&self) -> bool {
        self.is_network
    }

    /// Get the endpoint string (if network hub)
    ///
    /// Returns:
    ///     The endpoint string used to create this hub, or None for local hubs
    ///
    /// Example:
    ///     hub = Hub(CmdVel, endpoint="cmdvel@192.168.1.5:9000")
    ///     print(hub.endpoint)  # "cmdvel@192.168.1.5:9000"
    #[getter]
    fn get_endpoint(&self) -> Option<String> {
        self.endpoint.clone()
    }

    /// Get transport type information
    ///
    /// Returns:
    ///     String describing the transport: "shared_memory", "unix_socket", "udp_direct",
    ///     "batch_udp", "multicast", or "router"
    ///
    /// Example:
    ///     hub = Hub(CmdVel, endpoint="cmdvel@192.168.1.5:9000")
    ///     print(hub.transport_type)  # "udp_direct" or "batch_udp"
    #[getter]
    fn transport_type(&self) -> String {
        if !self.is_network {
            return "shared_memory".to_string();
        }

        // Check endpoint pattern to determine transport type
        if let Some(ref ep) = self.endpoint {
            if ep.contains("@localhost") {
                #[cfg(unix)]
                return "unix_socket".to_string();
                #[cfg(not(unix))]
                return "udp_direct".to_string();
            } else if ep.contains("@router") {
                return "router".to_string();
            } else if ep.contains("@*") {
                return "multicast".to_string();
            } else if ep.contains("@") {
                // Direct UDP (or batch UDP on Linux)
                #[cfg(target_os = "linux")]
                return "batch_udp".to_string();
                #[cfg(not(target_os = "linux"))]
                return "udp_direct".to_string();
            }
        }

        "shared_memory".to_string()
    }

    /// Get hub statistics as a dictionary
    ///
    /// Returns:
    ///     Dictionary with keys: messages_sent, messages_received, send_failures, recv_failures
    ///
    /// Example:
    ///     stats = hub.stats()
    ///     print(f"Sent: {stats['messages_sent']}, Received: {stats['messages_received']}")
    fn stats(&self) -> PyResult<pyo3::Py<pyo3::types::PyDict>> {
        Python::with_gil(|py| {
            let dict = pyo3::types::PyDict::new(py);

            // Get metrics from the underlying hub using get_metrics()
            let (sent, received, send_failures, recv_failures) = match &self.hub_type {
                HubType::CmdVel(hub) => {
                    let h = hub.lock().unwrap();
                    let m = h.get_metrics();
                    (
                        m.messages_sent,
                        m.messages_received,
                        m.send_failures,
                        m.recv_failures,
                    )
                }
                HubType::Pose2D(hub) => {
                    let h = hub.lock().unwrap();
                    let m = h.get_metrics();
                    (
                        m.messages_sent,
                        m.messages_received,
                        m.send_failures,
                        m.recv_failures,
                    )
                }
                HubType::Generic(hub) => {
                    let h = hub.lock().unwrap();
                    let m = h.get_metrics();
                    (
                        m.messages_sent,
                        m.messages_received,
                        m.send_failures,
                        m.recv_failures,
                    )
                }
            };

            dict.set_item("messages_sent", sent)?;
            dict.set_item("messages_received", received)?;
            dict.set_item("send_failures", send_failures)?;
            dict.set_item("recv_failures", recv_failures)?;
            dict.set_item("is_network", self.is_network)?;
            dict.set_item("transport", self.transport_type())?;

            Ok(dict.into())
        })
    }

    /// String representation
    fn __repr__(&self) -> String {
        let transport = self.transport_type();
        if self.is_network {
            format!(
                "Hub(topic='{}', endpoint='{}', transport='{}')",
                self.topic,
                self.endpoint.as_deref().unwrap_or("unknown"),
                transport
            )
        } else {
            format!("Hub(topic='{}', transport='{}')", self.topic, transport)
        }
    }
}
