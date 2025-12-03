// Python Link bindings - Point-to-point SPSC communication
//
// Link provides 1P1C (Single Producer, Single Consumer) communication pattern,
// optimized for direct point-to-point data transfer with minimal latency.
//
// Network support:
//   producer = Link.producer(CmdVel, "cmdvel@192.168.1.5:9000")  # Send to specific host
//   consumer = Link.consumer(CmdVel, "cmdvel@0.0.0.0:9000")      # Listen for connections
//
// Local shared memory:
//   producer = Link.producer(CmdVel, "sensor_data")  # Local (fast)
//   consumer = Link.consumer(CmdVel, "sensor_data")  # Local

use horus::communication::link::Link;
use horus_library::messages::cmd_vel::CmdVel;
use horus_library::messages::geometry::Pose2D;
use horus_library::messages::GenericMessage;
use pyo3::prelude::*;
use std::sync::{Arc, Mutex};

/// Link role type
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum LinkRoleType {
    Producer,
    Consumer,
}

/// Internal enum tracking which Rust type the Link wraps
enum LinkType {
    CmdVelProducer(Arc<Mutex<Link<CmdVel>>>),
    CmdVelConsumer(Arc<Mutex<Link<CmdVel>>>),
    Pose2DProducer(Arc<Mutex<Link<Pose2D>>>),
    Pose2DConsumer(Arc<Mutex<Link<Pose2D>>>),
    GenericProducer(Arc<Mutex<Link<GenericMessage>>>),
    GenericConsumer(Arc<Mutex<Link<GenericMessage>>>),
}

/// Python Link - Point-to-point SPSC communication channel
///
/// Unlike Hub (pub/sub MPMC), Link provides direct 1P1C communication:
/// - Single Producer, Single Consumer pattern
/// - Optimized for low-latency point-to-point data transfer
/// - ~30% faster than Hub for direct connections
///
/// Examples:
///     # Local shared memory
///     producer = Link.producer(CmdVel, "sensor_data")
///     consumer = Link.consumer(CmdVel, "sensor_data")
///
///     # Network communication
///     producer = Link.producer(CmdVel, "data@192.168.1.5:9000")  # Connect to consumer
///     consumer = Link.consumer(CmdVel, "data@0.0.0.0:9000")      # Listen for producer
#[pyclass(name = "Link")]
pub struct PyLink {
    link_type: LinkType,
    topic: String,
    endpoint: String,
    role: LinkRoleType,
    is_network: bool,
}

#[pymethods]
impl PyLink {
    /// Create a Link producer (sender)
    ///
    /// The producer can send messages but cannot receive.
    /// For network: connects to the specified endpoint.
    ///
    /// Args:
    ///     msg_type: Message class (CmdVel, Pose2D) or string for generic
    ///     endpoint: Topic name or network endpoint
    ///         - "topic"             - Local shared memory (fastest)
    ///         - "topic@host:port"   - Connect to remote consumer
    ///         - "topic@localhost"   - Unix socket to local consumer
    ///
    /// Returns:
    ///     Link producer instance
    ///
    /// Examples:
    ///     # Local
    ///     producer = Link.producer(CmdVel, "sensor")
    ///     producer.send(CmdVel(1.5, 0.5))
    ///
    ///     # Network
    ///     producer = Link.producer(CmdVel, "sensor@192.168.1.5:9000")
    #[staticmethod]
    fn producer(py: Python, msg_type: PyObject, endpoint: String) -> PyResult<Self> {
        Self::create(py, msg_type, endpoint, LinkRoleType::Producer)
    }

    /// Create a Link consumer (receiver)
    ///
    /// The consumer can receive messages but cannot send.
    /// For network: listens on the specified endpoint for a producer.
    ///
    /// Args:
    ///     msg_type: Message class (CmdVel, Pose2D) or string for generic
    ///     endpoint: Topic name or network endpoint
    ///         - "topic"             - Local shared memory (fastest)
    ///         - "topic@0.0.0.0:port" - Listen for remote producer
    ///         - "topic@localhost"   - Unix socket listener
    ///
    /// Returns:
    ///     Link consumer instance
    ///
    /// Examples:
    ///     # Local
    ///     consumer = Link.consumer(CmdVel, "sensor")
    ///     if msg := consumer.recv():
    ///         print(f"Received: {msg}")
    ///
    ///     # Network (listen)
    ///     consumer = Link.consumer(CmdVel, "sensor@0.0.0.0:9000")
    #[staticmethod]
    fn consumer(py: Python, msg_type: PyObject, endpoint: String) -> PyResult<Self> {
        Self::create(py, msg_type, endpoint, LinkRoleType::Consumer)
    }

    /// Send a message (producer only)
    ///
    /// Args:
    ///     message: Message to send (must match Link's type)
    ///
    /// Returns:
    ///     True if sent successfully
    ///
    /// Raises:
    ///     TypeError: If called on a consumer or wrong message type
    fn send(&self, py: Python, message: PyObject) -> PyResult<bool> {
        match &self.link_type {
            LinkType::CmdVelProducer(link) => {
                let linear: f32 = message.getattr(py, "linear")?.extract(py)?;
                let angular: f32 = message.getattr(py, "angular")?.extract(py)?;
                let stamp_nanos: u64 = message.getattr(py, "timestamp")?.extract(py)?;
                let cmd = CmdVel::with_timestamp(linear, angular, stamp_nanos);

                let link = link.lock().unwrap();
                match link.send(cmd, &mut None) {
                    Ok(()) => Ok(true),
                    Err(_) => Ok(false),
                }
            }
            LinkType::Pose2DProducer(link) => {
                let x: f64 = message.getattr(py, "x")?.extract(py)?;
                let y: f64 = message.getattr(py, "y")?.extract(py)?;
                let theta: f64 = message.getattr(py, "theta")?.extract(py)?;
                let timestamp: u64 = message.getattr(py, "timestamp")?.extract(py)?;
                let pose = Pose2D {
                    x,
                    y,
                    theta,
                    timestamp,
                };

                let link = link.lock().unwrap();
                match link.send(pose, &mut None) {
                    Ok(()) => Ok(true),
                    Err(_) => Ok(false),
                }
            }
            LinkType::GenericProducer(link) => {
                let bound = message.bind(py);
                let value: serde_json::Value = pythonize::depythonize(bound).map_err(|e| {
                    pyo3::exceptions::PyTypeError::new_err(format!(
                        "Failed to convert Python object: {}",
                        e
                    ))
                })?;
                let msgpack_bytes = rmp_serde::to_vec(&value).map_err(|e| {
                    pyo3::exceptions::PyRuntimeError::new_err(format!("Failed to serialize: {}", e))
                })?;
                let msg = GenericMessage::new(msgpack_bytes)
                    .map_err(pyo3::exceptions::PyValueError::new_err)?;

                let link = link.lock().unwrap();
                match link.send(msg, &mut None) {
                    Ok(()) => Ok(true),
                    Err(_) => Ok(false),
                }
            }
            _ => Err(pyo3::exceptions::PyTypeError::new_err(
                "send() can only be called on a producer Link",
            )),
        }
    }

    /// Receive a message (consumer only)
    ///
    /// Returns:
    ///     Message if available, None otherwise
    ///
    /// Raises:
    ///     TypeError: If called on a producer
    fn recv(&self, py: Python) -> PyResult<Option<PyObject>> {
        match &self.link_type {
            LinkType::CmdVelConsumer(link) => {
                let link = link.lock().unwrap();
                if let Some(cmd) = link.recv(&mut None) {
                    let horus_module = py.import("horus")?;
                    let cmdvel_class = horus_module.getattr("CmdVel")?;
                    let py_cmd = cmdvel_class.call1((cmd.linear, cmd.angular, cmd.stamp_nanos))?;
                    Ok(Some(py_cmd.into()))
                } else {
                    Ok(None)
                }
            }
            LinkType::Pose2DConsumer(link) => {
                let link = link.lock().unwrap();
                if let Some(pose) = link.recv(&mut None) {
                    let horus_module = py.import("horus")?;
                    let pose2d_class = horus_module.getattr("Pose2D")?;
                    let py_pose =
                        pose2d_class.call1((pose.x, pose.y, pose.theta, pose.timestamp))?;
                    Ok(Some(py_pose.into()))
                } else {
                    Ok(None)
                }
            }
            LinkType::GenericConsumer(link) => {
                let link = link.lock().unwrap();
                if let Some(msg) = link.recv(&mut None) {
                    let data = msg.data();
                    let value: serde_json::Value = rmp_serde::from_slice(&data).map_err(|e| {
                        pyo3::exceptions::PyRuntimeError::new_err(format!(
                            "Failed to deserialize: {}",
                            e
                        ))
                    })?;
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
            _ => Err(pyo3::exceptions::PyTypeError::new_err(
                "recv() can only be called on a consumer Link",
            )),
        }
    }

    /// Get the topic name
    #[getter]
    fn topic(&self) -> String {
        self.topic.clone()
    }

    /// Get the endpoint string
    #[getter]
    fn endpoint(&self) -> String {
        self.endpoint.clone()
    }

    /// Check if this is a producer
    #[getter]
    fn is_producer(&self) -> bool {
        self.role == LinkRoleType::Producer
    }

    /// Check if this is a consumer
    #[getter]
    fn is_consumer(&self) -> bool {
        self.role == LinkRoleType::Consumer
    }

    /// Check if using network transport
    #[getter]
    fn is_network(&self) -> bool {
        self.is_network
    }

    /// Get transport type
    #[getter]
    fn transport_type(&self) -> String {
        if !self.is_network {
            return "shared_memory".to_string();
        }

        if self.endpoint.contains("@localhost") {
            #[cfg(unix)]
            return "unix_socket".to_string();
            #[cfg(not(unix))]
            return "connected_udp".to_string();
        }

        "connected_udp".to_string()
    }

    /// String representation
    fn __repr__(&self) -> String {
        let role = if self.role == LinkRoleType::Producer {
            "producer"
        } else {
            "consumer"
        };
        format!(
            "Link(role='{}', endpoint='{}', transport='{}')",
            role,
            self.endpoint,
            self.transport_type()
        )
    }
}

impl PyLink {
    /// Internal helper to create a Link with a specific role
    fn create(
        py: Python,
        msg_type: PyObject,
        endpoint: String,
        role: LinkRoleType,
    ) -> PyResult<Self> {
        // Get type name
        let type_name = if let Ok(name) = msg_type.getattr(py, "__name__") {
            name.extract::<String>(py)?
        } else if let Ok(s) = msg_type.extract::<String>(py) {
            s
        } else {
            return Err(pyo3::exceptions::PyTypeError::new_err(
                "Link requires a message type (CmdVel, Pose2D) or topic string",
            ));
        };

        // Extract topic name (part before @)
        let topic = if endpoint.contains('@') {
            endpoint.split('@').next().unwrap_or(&endpoint).to_string()
        } else {
            endpoint.clone()
        };

        let is_network = endpoint.contains('@');

        // Create the appropriate typed Link
        let link_type = match (type_name.as_str(), role) {
            ("CmdVel", LinkRoleType::Producer) => {
                let link = Link::<CmdVel>::producer(&endpoint).map_err(|e| {
                    pyo3::exceptions::PyRuntimeError::new_err(format!(
                        "Failed to create CmdVel producer Link: {}",
                        e
                    ))
                })?;
                LinkType::CmdVelProducer(Arc::new(Mutex::new(link)))
            }
            ("CmdVel", LinkRoleType::Consumer) => {
                let link = Link::<CmdVel>::consumer(&endpoint).map_err(|e| {
                    pyo3::exceptions::PyRuntimeError::new_err(format!(
                        "Failed to create CmdVel consumer Link: {}",
                        e
                    ))
                })?;
                LinkType::CmdVelConsumer(Arc::new(Mutex::new(link)))
            }
            ("Pose2D", LinkRoleType::Producer) => {
                let link = Link::<Pose2D>::producer(&endpoint).map_err(|e| {
                    pyo3::exceptions::PyRuntimeError::new_err(format!(
                        "Failed to create Pose2D producer Link: {}",
                        e
                    ))
                })?;
                LinkType::Pose2DProducer(Arc::new(Mutex::new(link)))
            }
            ("Pose2D", LinkRoleType::Consumer) => {
                let link = Link::<Pose2D>::consumer(&endpoint).map_err(|e| {
                    pyo3::exceptions::PyRuntimeError::new_err(format!(
                        "Failed to create Pose2D consumer Link: {}",
                        e
                    ))
                })?;
                LinkType::Pose2DConsumer(Arc::new(Mutex::new(link)))
            }
            (_, LinkRoleType::Producer) => {
                let link = Link::<GenericMessage>::producer(&endpoint).map_err(|e| {
                    pyo3::exceptions::PyRuntimeError::new_err(format!(
                        "Failed to create generic producer Link: {}",
                        e
                    ))
                })?;
                LinkType::GenericProducer(Arc::new(Mutex::new(link)))
            }
            (_, LinkRoleType::Consumer) => {
                let link = Link::<GenericMessage>::consumer(&endpoint).map_err(|e| {
                    pyo3::exceptions::PyRuntimeError::new_err(format!(
                        "Failed to create generic consumer Link: {}",
                        e
                    ))
                })?;
                LinkType::GenericConsumer(Arc::new(Mutex::new(link)))
            }
        };

        Ok(Self {
            link_type,
            topic,
            endpoint,
            role,
            is_network,
        })
    }
}
