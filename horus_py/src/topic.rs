// Type-based Topic implementation for Python bindings
//
// Topic is the unified API replacing Hub and Link:
//   from horus import Topic, CmdVel, Pose2D
//   topic = Topic(CmdVel)  # Type determines everything
//
// Network support:
//   topic = Topic(CmdVel, endpoint="cmdvel@192.168.1.5:9000")  # Direct UDP
//   topic = Topic(CmdVel, endpoint="cmdvel@localhost")         # Unix socket
//   topic = Topic(CmdVel, endpoint="cmdvel@router")            # Via router
//   topic = Topic(CmdVel, endpoint="cmdvel@*")                 # Multicast
//
// Backend hints for fine-grained control:
//   topic = Topic(CmdVel, backend="direct")    # DirectChannel (~3ns)
//   topic = Topic(CmdVel, backend="spsc")      # SPSC intra-process (~18ns)
//   topic = Topic(CmdVel, backend="mpmc_shm")  # MPMC shared memory (~167ns)

use horus::communication::Topic;
use horus_library::messages::cmd_vel::CmdVel;
use horus_library::messages::geometry::Pose2D;
use horus_library::messages::sensor::{Imu, LaserScan, Odometry};
use horus_library::messages::GenericMessage;
use pyo3::prelude::*;
use pyo3::sync::PyOnceLock;
use pyo3::types::{PyDict, PyType};
use serde::{de::DeserializeOwned, Serialize};
use std::sync::{Arc, RwLock};

// ============================================================================
// Cached Python classes (100x faster than py.import() per call)
// ============================================================================

static CMDVEL_CLASS: PyOnceLock<Py<PyType>> = PyOnceLock::new();
static POSE2D_CLASS: PyOnceLock<Py<PyType>> = PyOnceLock::new();
static IMU_CLASS: PyOnceLock<Py<PyType>> = PyOnceLock::new();
static ODOMETRY_CLASS: PyOnceLock<Py<PyType>> = PyOnceLock::new();
static LASERSCAN_CLASS: PyOnceLock<Py<PyType>> = PyOnceLock::new();

fn get_cmdvel_class(py: Python<'_>) -> PyResult<&Bound<'_, PyType>> {
    CMDVEL_CLASS
        .get_or_try_init(py, || {
            let m = py.import("horus")?;
            Ok(m.getattr("CmdVel")?.cast::<PyType>()?.clone().unbind())
        })
        .map(|c| c.bind(py))
}

fn get_pose2d_class(py: Python<'_>) -> PyResult<&Bound<'_, PyType>> {
    POSE2D_CLASS
        .get_or_try_init(py, || {
            let m = py.import("horus")?;
            Ok(m.getattr("Pose2D")?.cast::<PyType>()?.clone().unbind())
        })
        .map(|c| c.bind(py))
}

fn get_imu_class(py: Python<'_>) -> PyResult<&Bound<'_, PyType>> {
    IMU_CLASS
        .get_or_try_init(py, || {
            let m = py.import("horus")?;
            Ok(m.getattr("Imu")?.cast::<PyType>()?.clone().unbind())
        })
        .map(|c| c.bind(py))
}

fn get_odometry_class(py: Python<'_>) -> PyResult<&Bound<'_, PyType>> {
    ODOMETRY_CLASS
        .get_or_try_init(py, || {
            let m = py.import("horus")?;
            Ok(m.getattr("Odometry")?.cast::<PyType>()?.clone().unbind())
        })
        .map(|c| c.bind(py))
}

fn get_laserscan_class(py: Python<'_>) -> PyResult<&Bound<'_, PyType>> {
    LASERSCAN_CLASS
        .get_or_try_init(py, || {
            let m = py.import("horus")?;
            Ok(m.getattr("LaserScan")?.cast::<PyType>()?.clone().unbind())
        })
        .map(|c| c.bind(py))
}

// ============================================================================
// Direct field extraction (bypasses serde, 2-5x faster)
// ============================================================================

fn extract_cmdvel(py: Python<'_>, obj: &Py<PyAny>) -> PyResult<CmdVel> {
    let linear: f32 = obj.getattr(py, "linear")?.extract(py)?;
    let angular: f32 = obj.getattr(py, "angular")?.extract(py)?;
    let timestamp_ns: u64 = obj.getattr(py, "timestamp_ns")?.extract(py).unwrap_or(0);
    Ok(CmdVel::with_timestamp(linear, angular, timestamp_ns))
}

fn extract_pose2d(py: Python<'_>, obj: &Py<PyAny>) -> PyResult<Pose2D> {
    let x: f64 = obj.getattr(py, "x")?.extract(py)?;
    let y: f64 = obj.getattr(py, "y")?.extract(py)?;
    let theta: f64 = obj.getattr(py, "theta")?.extract(py)?;
    let timestamp_ns: u64 = obj.getattr(py, "timestamp_ns")?.extract(py).unwrap_or(0);
    Ok(Pose2D {
        x,
        y,
        theta,
        timestamp_ns,
    })
}

fn extract_imu(py: Python<'_>, obj: &Py<PyAny>) -> PyResult<Imu> {
    let ax: f64 = obj.getattr(py, "accel_x")?.extract(py)?;
    let ay: f64 = obj.getattr(py, "accel_y")?.extract(py)?;
    let az: f64 = obj.getattr(py, "accel_z")?.extract(py)?;
    let gx: f64 = obj.getattr(py, "gyro_x")?.extract(py)?;
    let gy: f64 = obj.getattr(py, "gyro_y")?.extract(py)?;
    let gz: f64 = obj.getattr(py, "gyro_z")?.extract(py)?;
    let timestamp_ns: u64 = obj.getattr(py, "timestamp_ns")?.extract(py).unwrap_or(0);
    let mut imu = Imu::new();
    imu.linear_acceleration = [ax, ay, az];
    imu.angular_velocity = [gx, gy, gz];
    imu.timestamp_ns = timestamp_ns;
    Ok(imu)
}

fn extract_odometry(py: Python<'_>, obj: &Py<PyAny>) -> PyResult<Odometry> {
    let x: f64 = obj.getattr(py, "x")?.extract(py)?;
    let y: f64 = obj.getattr(py, "y")?.extract(py)?;
    let theta: f64 = obj.getattr(py, "theta")?.extract(py)?;
    let linear_velocity: f64 = obj
        .getattr(py, "linear_velocity")?
        .extract(py)
        .unwrap_or(0.0);
    let angular_velocity: f64 = obj
        .getattr(py, "angular_velocity")?
        .extract(py)
        .unwrap_or(0.0);
    let timestamp_ns: u64 = obj.getattr(py, "timestamp_ns")?.extract(py).unwrap_or(0);
    let mut odom = Odometry::new();
    odom.pose.x = x;
    odom.pose.y = y;
    odom.pose.theta = theta;
    odom.twist.linear[0] = linear_velocity;
    odom.twist.angular[2] = angular_velocity;
    odom.timestamp_ns = timestamp_ns;
    Ok(odom)
}

// ============================================================================
// Python object creation (direct field access, no serde)
// ============================================================================

fn cmdvel_to_python(py: Python<'_>, cmd: &CmdVel) -> PyResult<Py<PyAny>> {
    let cls = get_cmdvel_class(py)?;
    Ok(cls
        .call1((cmd.linear, cmd.angular, cmd.stamp_nanos))?
        .into())
}

fn pose2d_to_python(py: Python<'_>, pose: &Pose2D) -> PyResult<Py<PyAny>> {
    let cls = get_pose2d_class(py)?;
    Ok(cls
        .call1((pose.x, pose.y, pose.theta, pose.timestamp_ns))?
        .into())
}

fn imu_to_python(py: Python<'_>, imu: &Imu) -> PyResult<Py<PyAny>> {
    let cls = get_imu_class(py)?;
    Ok(cls
        .call1((
            imu.linear_acceleration[0],
            imu.linear_acceleration[1],
            imu.linear_acceleration[2],
            imu.angular_velocity[0],
            imu.angular_velocity[1],
            imu.angular_velocity[2],
            imu.timestamp_ns,
        ))?
        .into())
}

fn odometry_to_python(py: Python<'_>, odom: &Odometry) -> PyResult<Py<PyAny>> {
    let cls = get_odometry_class(py)?;
    let dict = PyDict::new(py);
    dict.set_item("x", odom.pose.x)?;
    dict.set_item("y", odom.pose.y)?;
    dict.set_item("theta", odom.pose.theta)?;
    dict.set_item("linear_velocity", odom.twist.linear[0])?;
    dict.set_item("angular_velocity", odom.twist.angular[2])?;
    dict.set_item("timestamp_ns", odom.timestamp_ns)?;
    Ok(cls.call((), Some(&dict))?.into())
}

fn laserscan_to_python(py: Python<'_>, scan: &LaserScan) -> PyResult<Py<PyAny>> {
    let cls = get_laserscan_class(py)?;
    let dict = PyDict::new(py);
    dict.set_item("angle_min", scan.angle_min)?;
    dict.set_item("angle_max", scan.angle_max)?;
    dict.set_item("angle_increment", scan.angle_increment)?;
    dict.set_item("range_min", scan.range_min)?;
    dict.set_item("range_max", scan.range_max)?;
    dict.set_item("ranges", scan.ranges.as_slice())?;
    dict.set_item("timestamp_ns", scan.timestamp_ns)?;
    Ok(cls.call((), Some(&dict))?.into())
}

fn from_python<T: DeserializeOwned>(py: Python, obj: &Py<PyAny>) -> PyResult<T> {
    pythonize::depythonize(obj.bind(py)).map_err(|e| {
        pyo3::exceptions::PyTypeError::new_err(format!("Failed to convert from Python: {}", e))
    })
}

fn to_python<T: Serialize>(py: Python, value: &T) -> PyResult<Py<PyAny>> {
    pythonize::pythonize(py, value)
        .map(|o| o.into())
        .map_err(|e| {
            pyo3::exceptions::PyRuntimeError::new_err(format!("Failed to convert to Python: {}", e))
        })
}

// ============================================================================
// TopicType enum - tracks which Rust type the Topic wraps
// ============================================================================

enum TopicType {
    CmdVel(Arc<RwLock<Topic<CmdVel>>>),
    Pose2D(Arc<RwLock<Topic<Pose2D>>>),
    Imu(Arc<RwLock<Topic<Imu>>>),
    Odometry(Arc<RwLock<Topic<Odometry>>>),
    LaserScan(Arc<RwLock<Topic<LaserScan>>>),
    Generic(Arc<RwLock<Topic<GenericMessage>>>),
}

// ============================================================================
// PyTopic - Unified Python API for HORUS communication
// ============================================================================

/// Python Topic - unified type-safe wrapper for HORUS communication
///
/// Topic is the primary API for HORUS communication in Python.
/// It automatically selects the optimal backend based on configuration.
///
/// Examples:
///     # Local shared memory (fastest for same-machine)
///     topic = Topic(CmdVel)
///
///     # With custom capacity
///     topic = Topic(Pose2D, capacity=2048)
///
///     # Network communication
///     topic = Topic(CmdVel, endpoint="cmdvel@192.168.1.5:9000")  # Direct UDP
///     topic = Topic(CmdVel, endpoint="cmdvel@localhost")         # Unix socket
///     topic = Topic(CmdVel, endpoint="cmdvel@router")            # Via router
///
///     # Backend hints for fine-grained control
///     topic = Topic(CmdVel, backend="direct")    # DirectChannel (~3ns)
///     topic = Topic(CmdVel, backend="spsc")      # SPSC intra-process (~18ns)
///     topic = Topic(CmdVel, backend="mpmc_shm")  # MPMC shared memory (~167ns)
#[pyclass(name = "Topic")]
pub struct PyTopic {
    topic_type: TopicType,
    name: String,
    endpoint: Option<String>,
    is_network: bool,
}

#[pymethods]
impl PyTopic {
    /// Create a new Topic for a specific message type
    ///
    /// Args:
    ///     msg_type: Message class (CmdVel, Pose2D) or string for generic topic
    ///     capacity: Optional buffer capacity (default: 1024)
    ///     endpoint: Optional network endpoint string
    ///     backend: Optional backend hint ("direct", "spsc", "mpmc", "mpmc_shm", etc.)
    ///
    /// Endpoint formats:
    ///     "topic"                    - Local shared memory (default)
    ///     "topic@host:port"          - Direct UDP to specific host
    ///     "topic@localhost"          - Unix domain socket (Unix only)
    ///     "topic@router"             - Via HORUS router (TCP broker)
    ///     "topic@*"                  - Multicast discovery
    ///
    /// Backend hints:
    ///     "direct"      - DirectChannel (~3ns) - same-thread callbacks
    ///     "spsc"        - SPSC intra-process (~18ns) - single producer/consumer
    ///     "mpsc"        - MPSC intra-process (~26ns) - multiple producers
    ///     "mpmc"        - MPMC intra-process (~36ns) - multiple producers/consumers
    ///     "spsc_shm"    - SPSC shared memory (~85ns) - cross-process SPSC
    ///     "mpmc_shm"    - MPMC shared memory (~167ns) - cross-process MPMC
    ///     "network"     - Network transport (requires endpoint)
    ///
    /// Examples:
    ///     topic = Topic(CmdVel)                                      # Local, default
    ///     topic = Topic(Pose2D, capacity=2048)                       # Custom capacity
    ///     topic = Topic(CmdVel, endpoint="cmdvel@192.168.1.5:9000")  # Network
    ///     topic = Topic(CmdVel, backend="direct")                    # Fastest intra-thread
    #[new]
    #[pyo3(signature = (msg_type, capacity=None, endpoint=None, backend=None))]
    fn new(
        py: Python,
        msg_type: Py<PyAny>,
        capacity: Option<usize>,
        endpoint: Option<String>,
        backend: Option<String>,
    ) -> PyResult<Self> {
        // Get type name from the Python object
        let type_name = if let Ok(name) = msg_type.getattr(py, "__name__") {
            name.extract::<String>(py)?
        } else if let Ok(s) = msg_type.extract::<String>(py) {
            s
        } else {
            return Err(pyo3::exceptions::PyTypeError::new_err(
                "Topic() requires a message type (CmdVel, Pose2D) or topic string",
            ));
        };

        // Get topic name from type's __topic_name__, or default to lowercase type name
        let topic_name = if let Ok(topic_attr) = msg_type.getattr(py, "__topic_name__") {
            topic_attr.extract::<String>(py)?
        } else {
            type_name.to_lowercase()
        };

        let effective_endpoint = endpoint.clone().unwrap_or_else(|| topic_name.clone());
        let is_network = endpoint.as_ref().is_some_and(|e| e.contains('@'));
        let cap = capacity.unwrap_or(1024);

        // Create the appropriate typed Topic based on backend hint and message type
        let topic_type = match type_name.as_str() {
            "CmdVel" => {
                let topic = create_topic::<CmdVel>(&effective_endpoint, cap, backend.as_deref())?;
                TopicType::CmdVel(Arc::new(RwLock::new(topic)))
            }
            "Pose2D" => {
                let topic = create_topic::<Pose2D>(&effective_endpoint, cap, backend.as_deref())?;
                TopicType::Pose2D(Arc::new(RwLock::new(topic)))
            }
            "Imu" => {
                let topic = create_topic::<Imu>(&effective_endpoint, cap, backend.as_deref())?;
                TopicType::Imu(Arc::new(RwLock::new(topic)))
            }
            "Odometry" => {
                let topic = create_topic::<Odometry>(&effective_endpoint, cap, backend.as_deref())?;
                TopicType::Odometry(Arc::new(RwLock::new(topic)))
            }
            "LaserScan" => {
                let topic =
                    create_topic::<LaserScan>(&effective_endpoint, cap, backend.as_deref())?;
                TopicType::LaserScan(Arc::new(RwLock::new(topic)))
            }
            _ => {
                let topic =
                    create_topic::<GenericMessage>(&effective_endpoint, cap, backend.as_deref())?;
                TopicType::Generic(Arc::new(RwLock::new(topic)))
            }
        };

        Ok(Self {
            topic_type,
            name: topic_name,
            endpoint,
            is_network,
        })
    }

    /// Send a message (type must match Topic's type)
    ///
    /// Args:
    ///     message: Message object (CmdVel, Pose2D, etc.)
    ///     node: Optional Node for automatic logging with IPC timing
    ///
    /// Returns:
    ///     Always True (send is infallible fire-and-forget)
    ///
    /// Examples:
    ///     topic.send(CmdVel(1.5, 0.5), node)  # With logging
    ///     topic.send(Pose2D(1.0, 2.0, 0.5))   # Without logging
    #[pyo3(signature = (message, node=None))]
    fn send(&self, py: Python, message: Py<PyAny>, node: Option<Py<PyAny>>) -> PyResult<bool> {
        use std::time::Instant;
        let start = Instant::now();

        let result = match &self.topic_type {
            TopicType::CmdVel(topic) => {
                let cmd = extract_cmdvel(py, &message)?;
                let topic_ref = topic.clone();
                let success = py.detach(|| {
                    let topic = topic_ref.write().unwrap();
                    topic.send(cmd);
                    true
                });

                if let Some(node_obj) = &node {
                    let ipc_ns = start.elapsed().as_nanos() as u64;
                    if let Ok(info) = node_obj.getattr(py, "info") {
                        if !info.is_none(py) {
                            let _ =
                                info.call_method1(py, "register_publisher", (&self.name, "CmdVel"));
                            use horus::core::LogSummary;
                            let _ = info.call_method1(
                                py,
                                "log_pub",
                                (&self.name, cmd.log_summary(), ipc_ns),
                            );
                        }
                    }
                }
                success
            }
            TopicType::Pose2D(topic) => {
                let pose = extract_pose2d(py, &message)?;
                let topic_ref = topic.clone();
                let success = py.detach(|| {
                    let topic = topic_ref.write().unwrap();
                    topic.send(pose);
                    true
                });

                if let Some(node_obj) = &node {
                    let ipc_ns = start.elapsed().as_nanos() as u64;
                    if let Ok(info) = node_obj.getattr(py, "info") {
                        if !info.is_none(py) {
                            let _ =
                                info.call_method1(py, "register_publisher", (&self.name, "Pose2D"));
                            use horus::core::LogSummary;
                            let _ = info.call_method1(
                                py,
                                "log_pub",
                                (&self.name, pose.log_summary(), ipc_ns),
                            );
                        }
                    }
                }
                success
            }
            TopicType::Imu(topic) => {
                let imu = extract_imu(py, &message)?;
                let topic_ref = topic.clone();
                let success = py.detach(|| {
                    let topic = topic_ref.write().unwrap();
                    topic.send(imu);
                    true
                });

                if let Some(node_obj) = &node {
                    let ipc_ns = start.elapsed().as_nanos() as u64;
                    if let Ok(info) = node_obj.getattr(py, "info") {
                        if !info.is_none(py) {
                            let _ =
                                info.call_method1(py, "register_publisher", (&self.name, "Imu"));
                            use horus::core::LogSummary;
                            let _ = info.call_method1(
                                py,
                                "log_pub",
                                (&self.name, imu.log_summary(), ipc_ns),
                            );
                        }
                    }
                }
                success
            }
            TopicType::Odometry(topic) => {
                let odom = extract_odometry(py, &message)?;
                let topic_ref = topic.clone();
                let success = py.detach(|| {
                    let topic = topic_ref.write().unwrap();
                    topic.send(odom);
                    true
                });

                if let Some(node_obj) = &node {
                    let ipc_ns = start.elapsed().as_nanos() as u64;
                    if let Ok(info) = node_obj.getattr(py, "info") {
                        if !info.is_none(py) {
                            let _ = info.call_method1(
                                py,
                                "register_publisher",
                                (&self.name, "Odometry"),
                            );
                            use horus::core::LogSummary;
                            let _ = info.call_method1(
                                py,
                                "log_pub",
                                (&self.name, odom.log_summary(), ipc_ns),
                            );
                        }
                    }
                }
                success
            }
            TopicType::LaserScan(topic) => {
                let scan: LaserScan = from_python(py, &message)?;
                use horus::core::LogSummary;
                let log_summary = scan.log_summary();

                let topic_ref = topic.clone();
                let success = py.detach(|| {
                    let topic = topic_ref.write().unwrap();
                    topic.send(scan);
                    true
                });

                if let Some(node_obj) = &node {
                    let ipc_ns = start.elapsed().as_nanos() as u64;
                    if let Ok(info) = node_obj.getattr(py, "info") {
                        if !info.is_none(py) {
                            let _ = info.call_method1(
                                py,
                                "register_publisher",
                                (&self.name, "LaserScan"),
                            );
                            let _ =
                                info.call_method1(py, "log_pub", (&self.name, log_summary, ipc_ns));
                        }
                    }
                }
                success
            }
            TopicType::Generic(topic) => {
                let bound = message.bind(py);
                let value: serde_json::Value = pythonize::depythonize(bound).map_err(|e| {
                    pyo3::exceptions::PyTypeError::new_err(format!(
                        "Failed to convert Python object: {}",
                        e
                    ))
                })?;

                let msgpack_bytes = rmp_serde::to_vec(&value).map_err(|e| {
                    pyo3::exceptions::PyRuntimeError::new_err(format!(
                        "Failed to serialize to MessagePack: {}",
                        e
                    ))
                })?;

                let msg = GenericMessage::new(msgpack_bytes)
                    .map_err(pyo3::exceptions::PyValueError::new_err)?;

                use horus::core::LogSummary;
                let log_summary = msg.log_summary();

                let topic_ref = topic.clone();
                let success = py.detach(|| {
                    let topic = topic_ref.write().unwrap();
                    topic.send(msg);
                    true
                });

                if let Some(node_obj) = &node {
                    let ipc_ns = start.elapsed().as_nanos() as u64;
                    if let Ok(info) = node_obj.getattr(py, "info") {
                        if !info.is_none(py) {
                            let _ = info.call_method1(
                                py,
                                "register_publisher",
                                (&self.name, "GenericMessage"),
                            );
                            let _ =
                                info.call_method1(py, "log_pub", (&self.name, log_summary, ipc_ns));
                        }
                    }
                }
                success
            }
        };

        Ok(result)
    }

    /// Receive a message (returns typed object matching Topic's type)
    ///
    /// Args:
    ///     node: Optional Node for automatic logging with IPC timing
    ///
    /// Returns:
    ///     CmdVel/Pose2D object if available, None otherwise
    ///
    /// Examples:
    ///     cmd = topic.recv(node)  # With logging
    ///     pose = topic.recv()     # Without logging
    #[pyo3(signature = (node=None))]
    fn recv(&self, py: Python, node: Option<Py<PyAny>>) -> PyResult<Option<Py<PyAny>>> {
        use std::time::Instant;
        let start = Instant::now();

        match &self.topic_type {
            TopicType::CmdVel(topic) => {
                let topic_ref = topic.clone();
                let msg_opt = py.detach(|| {
                    let topic = topic_ref.read().unwrap();
                    topic.recv()
                });
                if let Some(cmd) = msg_opt {
                    let ipc_ns = start.elapsed().as_nanos() as u64;
                    if let Some(node_obj) = &node {
                        if let Ok(info) = node_obj.getattr(py, "info") {
                            if !info.is_none(py) {
                                let _ = info.call_method1(
                                    py,
                                    "register_subscriber",
                                    (&self.name, "CmdVel"),
                                );
                                use horus::core::LogSummary;
                                let _ = info.call_method1(
                                    py,
                                    "log_sub",
                                    (&self.name, cmd.log_summary(), ipc_ns),
                                );
                            }
                        }
                    }
                    Ok(Some(cmdvel_to_python(py, &cmd)?))
                } else {
                    Ok(None)
                }
            }
            TopicType::Pose2D(topic) => {
                let topic_ref = topic.clone();
                let msg_opt = py.detach(|| {
                    let topic = topic_ref.read().unwrap();
                    topic.recv()
                });
                if let Some(pose) = msg_opt {
                    let ipc_ns = start.elapsed().as_nanos() as u64;
                    if let Some(node_obj) = &node {
                        if let Ok(info) = node_obj.getattr(py, "info") {
                            if !info.is_none(py) {
                                let _ = info.call_method1(
                                    py,
                                    "register_subscriber",
                                    (&self.name, "Pose2D"),
                                );
                                use horus::core::LogSummary;
                                let _ = info.call_method1(
                                    py,
                                    "log_sub",
                                    (&self.name, pose.log_summary(), ipc_ns),
                                );
                            }
                        }
                    }
                    Ok(Some(pose2d_to_python(py, &pose)?))
                } else {
                    Ok(None)
                }
            }
            TopicType::Imu(topic) => {
                let topic_ref = topic.clone();
                let msg_opt = py.detach(|| {
                    let topic = topic_ref.read().unwrap();
                    topic.recv()
                });
                if let Some(imu) = msg_opt {
                    let ipc_ns = start.elapsed().as_nanos() as u64;
                    if let Some(node_obj) = &node {
                        if let Ok(info) = node_obj.getattr(py, "info") {
                            if !info.is_none(py) {
                                let _ = info.call_method1(
                                    py,
                                    "register_subscriber",
                                    (&self.name, "Imu"),
                                );
                                use horus::core::LogSummary;
                                let _ = info.call_method1(
                                    py,
                                    "log_sub",
                                    (&self.name, imu.log_summary(), ipc_ns),
                                );
                            }
                        }
                    }
                    Ok(Some(imu_to_python(py, &imu)?))
                } else {
                    Ok(None)
                }
            }
            TopicType::Odometry(topic) => {
                let topic_ref = topic.clone();
                let msg_opt = py.detach(|| {
                    let topic = topic_ref.read().unwrap();
                    topic.recv()
                });
                if let Some(odom) = msg_opt {
                    let ipc_ns = start.elapsed().as_nanos() as u64;
                    if let Some(node_obj) = &node {
                        if let Ok(info) = node_obj.getattr(py, "info") {
                            if !info.is_none(py) {
                                let _ = info.call_method1(
                                    py,
                                    "register_subscriber",
                                    (&self.name, "Odometry"),
                                );
                                use horus::core::LogSummary;
                                let _ = info.call_method1(
                                    py,
                                    "log_sub",
                                    (&self.name, odom.log_summary(), ipc_ns),
                                );
                            }
                        }
                    }
                    Ok(Some(odometry_to_python(py, &odom)?))
                } else {
                    Ok(None)
                }
            }
            TopicType::LaserScan(topic) => {
                let topic_ref = topic.clone();
                let msg_opt = py.detach(|| {
                    let topic = topic_ref.read().unwrap();
                    topic.recv()
                });
                if let Some(scan) = msg_opt {
                    let ipc_ns = start.elapsed().as_nanos() as u64;
                    if let Some(node_obj) = &node {
                        if let Ok(info) = node_obj.getattr(py, "info") {
                            if !info.is_none(py) {
                                let _ = info.call_method1(
                                    py,
                                    "register_subscriber",
                                    (&self.name, "LaserScan"),
                                );
                                use horus::core::LogSummary;
                                let _ = info.call_method1(
                                    py,
                                    "log_sub",
                                    (&self.name, scan.log_summary(), ipc_ns),
                                );
                            }
                        }
                    }
                    Ok(Some(laserscan_to_python(py, &scan)?))
                } else {
                    Ok(None)
                }
            }
            TopicType::Generic(topic) => {
                let topic_ref = topic.clone();
                let msg_opt = py.detach(|| {
                    let topic = topic_ref.read().unwrap();
                    topic.recv()
                });
                if let Some(msg) = msg_opt {
                    let ipc_ns = start.elapsed().as_nanos() as u64;

                    if let Some(node_obj) = &node {
                        if let Ok(info) = node_obj.getattr(py, "info") {
                            if !info.is_none(py) {
                                let _ = info.call_method1(
                                    py,
                                    "register_subscriber",
                                    (&self.name, "GenericMessage"),
                                );
                                use horus::core::LogSummary;
                                let log_msg = msg.log_summary();
                                let _ =
                                    info.call_method1(py, "log_sub", (&self.name, log_msg, ipc_ns));
                            }
                        }
                    }

                    let data = msg.data();
                    let value: serde_json::Value = rmp_serde::from_slice(&data).map_err(|e| {
                        pyo3::exceptions::PyRuntimeError::new_err(format!(
                            "Failed to deserialize MessagePack: {}",
                            e
                        ))
                    })?;

                    Ok(Some(to_python(py, &value)?))
                } else {
                    Ok(None)
                }
            }
        }
    }

    /// Get the topic name
    #[getter]
    fn name(&self) -> String {
        self.name.clone()
    }

    /// Check if this topic uses network transport
    #[getter]
    fn is_network_topic(&self) -> bool {
        self.is_network
    }

    /// Get the endpoint string (if network topic)
    #[getter]
    fn endpoint(&self) -> Option<String> {
        self.endpoint.clone()
    }

    /// Get the backend type name
    #[getter]
    fn backend_type(&self) -> String {
        match &self.topic_type {
            TopicType::CmdVel(t) => t.read().unwrap().backend_name().to_string(),
            TopicType::Pose2D(t) => t.read().unwrap().backend_name().to_string(),
            TopicType::Imu(t) => t.read().unwrap().backend_name().to_string(),
            TopicType::Odometry(t) => t.read().unwrap().backend_name().to_string(),
            TopicType::LaserScan(t) => t.read().unwrap().backend_name().to_string(),
            TopicType::Generic(t) => t.read().unwrap().backend_name().to_string(),
        }
    }

    /// Get topic statistics as a dictionary
    ///
    /// Returns:
    ///     Dictionary with keys: messages_sent, messages_received, send_failures, recv_failures
    fn stats(&self) -> PyResult<pyo3::Py<pyo3::types::PyDict>> {
        Python::attach(|py| {
            let dict = pyo3::types::PyDict::new(py);

            let metrics = match &self.topic_type {
                TopicType::CmdVel(t) => t.read().unwrap().metrics(),
                TopicType::Pose2D(t) => t.read().unwrap().metrics(),
                TopicType::Imu(t) => t.read().unwrap().metrics(),
                TopicType::Odometry(t) => t.read().unwrap().metrics(),
                TopicType::LaserScan(t) => t.read().unwrap().metrics(),
                TopicType::Generic(t) => t.read().unwrap().metrics(),
            };

            dict.set_item("messages_sent", metrics.messages_sent)?;
            dict.set_item("messages_received", metrics.messages_received)?;
            dict.set_item("send_failures", metrics.send_failures)?;
            dict.set_item("recv_failures", metrics.recv_failures)?;
            dict.set_item("is_network", self.is_network)?;
            dict.set_item("backend", self.backend_type())?;

            Ok(dict.into())
        })
    }

    /// Check if this is a generic topic (supports metadata methods)
    fn is_generic(&self) -> bool {
        matches!(self.topic_type, TopicType::Generic(_))
    }

    /// String representation
    fn __repr__(&self) -> String {
        let backend = self.backend_type();
        if self.is_network {
            format!(
                "Topic(name='{}', endpoint='{}', backend='{}')",
                self.name,
                self.endpoint.as_deref().unwrap_or("unknown"),
                backend
            )
        } else {
            format!("Topic(name='{}', backend='{}')", self.name, backend)
        }
    }
}

// ============================================================================
// Helper functions
// ============================================================================

fn create_topic<T>(endpoint: &str, capacity: usize, backend: Option<&str>) -> PyResult<Topic<T>>
where
    T: Clone
        + Send
        + Sync
        + 'static
        + serde::Serialize
        + serde::de::DeserializeOwned
        + std::fmt::Debug,
{
    use horus::communication::{BackendHint, TopicConfig};

    // Check if this is a network endpoint
    if endpoint.contains('@') {
        let topic_name = endpoint
            .split('@')
            .next()
            .unwrap_or(endpoint);
        return Topic::with_capacity(topic_name, capacity as u32, None).map_err(|e| {
            pyo3::exceptions::PyRuntimeError::new_err(format!(
                "Failed to create network Topic: {}",
                e
            ))
        });
    }

    // Determine backend hint
    let hint = match backend {
        Some("direct") => BackendHint::DirectChannel,
        Some("spsc") => BackendHint::SpscIntra,
        Some("mpsc") => BackendHint::MpscIntra,
        Some("spmc") => BackendHint::SpmcIntra,
        Some("mpmc") => BackendHint::MpmcIntra,
        Some("spsc_shm") => BackendHint::SpscShm,
        Some("mpsc_shm") => BackendHint::MpscShm,
        Some("spmc_shm") => BackendHint::SpmcShm,
        Some("mpmc_shm") => BackendHint::MpmcShm,
        Some(other) => {
            return Err(pyo3::exceptions::PyValueError::new_err(format!(
                "Unknown backend hint: '{}'. Valid options: direct, spsc, mpsc, spmc, mpmc, \
                 spsc_shm, mpsc_shm, spmc_shm, mpmc_shm",
                other
            )));
        }
        None => BackendHint::Auto,
    };

    // Create topic configuration
    let config = TopicConfig::new(endpoint)
        .with_capacity(capacity as u32)
        .with_backend(hint);

    // Create topic from config
    Topic::from_config(config).map_err(|e| {
        pyo3::exceptions::PyRuntimeError::new_err(format!("Failed to create Topic: {}", e))
    })
}
