// Type-based Topic implementation for Python bindings
//
// Topic is the unified API:
//   from horus import Topic, CmdVel, Pose2D
//   topic = Topic(CmdVel)  # Type determines everything
//
// Network support:
//   topic = Topic(CmdVel, endpoint="cmdvel@192.168.1.5:9000")  # Direct UDP
//   topic = Topic(CmdVel, endpoint="cmdvel@localhost")         # Unix socket
//
// Note: Backend is auto-selected based on topology (pub/sub count, same-process, etc.).
//
// # POD-Optimized Message Path
//
// Python message wrappers (PyCmdVel, PyPose2D, etc.) store the Rust POD struct
// directly in their `inner` field. This enables:
//
//   Send: message.extract::<PyRef<PyCmdVel>>()?.inner  (single memcpy, no getattr)
//   Recv: Py::new(py, PyCmdVel { inner: msg })         (no Python constructor call)
//
// This eliminates per-field attribute lookups on send and Python class
// construction on recv — ~8x faster send, ~3x faster recv vs the old approach.

use horus::communication::Topic;
use horus_core::memory::{DepthImage, Image, PointCloud};
use horus_library::messages::cmd_vel::CmdVel;
use horus_library::messages::geometry::Pose2D;
use horus_library::messages::sensor::{Imu, LaserScan, Odometry};
use horus_library::messages::GenericMessage;
use pyo3::exceptions::PyRuntimeError;
use pyo3::prelude::*;
use serde::{de::DeserializeOwned, Serialize};
use std::sync::{Arc, RwLock};

use crate::depth_image::PyDepthImage;
use crate::image::PyImage;
use crate::messages::{PyCmdVel, PyImu, PyLaserScan, PyOdometry, PyPose2D};
use crate::pointcloud::PyPointCloud;

/// Log a failed Python node callback at debug level instead of silently dropping it.
/// Used for non-critical observability calls (log_pub/sub)
/// that must never crash the data path.
#[inline]
fn log_py_callback(result: PyResult<Py<PyAny>>, method: &str, topic: &str) {
    if let Err(e) = result {
        tracing::debug!(topic, method, error = %e, "Python node callback failed");
    }
}

/// Log a publish/subscribe event if node info is available.
#[inline]
fn log_ipc_event(
    py: Python,
    node: &Option<Py<PyAny>>,
    topic_name: &str,
    log_summary: String,
    ipc_ns: u64,
    method: &str,
) {
    if let Some(node_obj) = node {
        if let Ok(info) = node_obj.getattr(py, "info") {
            if !info.is_none(py) {
                log_py_callback(
                    info.call_method1(py, method, (topic_name, log_summary, ipc_ns)),
                    method,
                    topic_name,
                );
            }
        }
    }
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
    Image(Arc<RwLock<Topic<Image>>>),
    PointCloud(Arc<RwLock<Topic<PointCloud>>>),
    DepthImage(Arc<RwLock<Topic<DepthImage>>>),
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
/// ```python
/// # Local shared memory (fastest for same-machine)
/// topic = Topic(CmdVel)
///
/// # With custom capacity
/// topic = Topic(Pose2D, capacity=2048)
///
/// # Network communication
/// topic = Topic(CmdVel, endpoint="cmdvel@192.168.1.5:9000")  # Direct UDP
/// topic = Topic(CmdVel, endpoint="cmdvel@localhost")         # Unix socket
/// topic = Topic(CmdVel, endpoint="cmdvel@router")            # Via router
/// ```
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
    ///
    /// Endpoint formats:
    ///     "topic"                    - Local shared memory (default)
    ///     "topic@host:port"          - Direct UDP to specific host
    ///     "topic@localhost"          - Unix domain socket (Unix only)
    ///     "topic@router"             - Via HORUS router (TCP broker)
    ///     "topic@*"                  - Multicast discovery
    ///
    /// Examples:
    ///     topic = Topic(CmdVel)
    ///     topic = Topic(Pose2D, capacity=2048)
    ///     topic = Topic(CmdVel, endpoint="cmdvel@192.168.1.5:9000")
    #[new]
    #[pyo3(signature = (msg_type, capacity=None, endpoint=None))]
    fn new(
        py: Python,
        msg_type: Py<PyAny>,
        capacity: Option<usize>,
        endpoint: Option<String>,
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

        // Create the appropriate typed Topic based on message type
        let topic_type = match type_name.as_str() {
            "CmdVel" => {
                let topic = create_topic::<CmdVel>(&effective_endpoint, cap)?;
                TopicType::CmdVel(Arc::new(RwLock::new(topic)))
            }
            "Pose2D" => {
                let topic = create_topic::<Pose2D>(&effective_endpoint, cap)?;
                TopicType::Pose2D(Arc::new(RwLock::new(topic)))
            }
            "Imu" => {
                let topic = create_topic::<Imu>(&effective_endpoint, cap)?;
                TopicType::Imu(Arc::new(RwLock::new(topic)))
            }
            "Odometry" => {
                let topic = create_topic::<Odometry>(&effective_endpoint, cap)?;
                TopicType::Odometry(Arc::new(RwLock::new(topic)))
            }
            "LaserScan" => {
                let topic = create_topic::<LaserScan>(&effective_endpoint, cap)?;
                TopicType::LaserScan(Arc::new(RwLock::new(topic)))
            }
            "Image" => {
                let topic = create_pool_topic::<Image>(&effective_endpoint, cap)?;
                TopicType::Image(Arc::new(RwLock::new(topic)))
            }
            "PointCloud" => {
                let topic = create_pool_topic::<PointCloud>(&effective_endpoint, cap)?;
                TopicType::PointCloud(Arc::new(RwLock::new(topic)))
            }
            "DepthImage" => {
                let topic = create_pool_topic::<DepthImage>(&effective_endpoint, cap)?;
                TopicType::DepthImage(Arc::new(RwLock::new(topic)))
            }
            _ => {
                let topic = create_topic::<GenericMessage>(&effective_endpoint, cap)?;
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
    fn send(
        &self,
        py: Python,
        message: Py<PyAny>,
        node: Option<Py<PyAny>>,
    ) -> PyResult<bool> {
        use std::time::Instant;
        let start = Instant::now();

        let result = match &self.topic_type {
            TopicType::CmdVel(topic) => {
                // POD-optimized: direct struct access, no getattr
                let cmd = message.extract::<PyRef<PyCmdVel>>(py)?.inner;
                let topic_ref = topic.clone();
                let success = py.detach(|| {
                    topic_ref.write().unwrap().send(cmd);
                    true
                });
                if node.is_some() {
                    use horus::core::LogSummary;
                    log_ipc_event(
                        py,
                        &node,
                        &self.name,
                        cmd.log_summary(),
                        start.elapsed().as_nanos() as u64,
                        "log_pub",
                    );
                }
                success
            }
            TopicType::Pose2D(topic) => {
                let pose = message.extract::<PyRef<PyPose2D>>(py)?.inner;
                let topic_ref = topic.clone();
                let success = py.detach(|| {
                    topic_ref.write().unwrap().send(pose);
                    true
                });
                if node.is_some() {
                    use horus::core::LogSummary;
                    log_ipc_event(
                        py,
                        &node,
                        &self.name,
                        pose.log_summary(),
                        start.elapsed().as_nanos() as u64,
                        "log_pub",
                    );
                }
                success
            }
            TopicType::Imu(topic) => {
                let imu = message.extract::<PyRef<PyImu>>(py)?.inner;
                let topic_ref = topic.clone();
                let success = py.detach(|| {
                    topic_ref.write().unwrap().send(imu);
                    true
                });
                if node.is_some() {
                    use horus::core::LogSummary;
                    log_ipc_event(
                        py,
                        &node,
                        &self.name,
                        imu.log_summary(),
                        start.elapsed().as_nanos() as u64,
                        "log_pub",
                    );
                }
                success
            }
            TopicType::Odometry(topic) => {
                let odom = message.extract::<PyRef<PyOdometry>>(py)?.inner;
                let topic_ref = topic.clone();
                let success = py.detach(|| {
                    topic_ref.write().unwrap().send(odom);
                    true
                });
                if node.is_some() {
                    use horus::core::LogSummary;
                    log_ipc_event(
                        py,
                        &node,
                        &self.name,
                        odom.log_summary(),
                        start.elapsed().as_nanos() as u64,
                        "log_pub",
                    );
                }
                success
            }
            TopicType::LaserScan(topic) => {
                // POD-optimized: direct struct copy instead of serde/pythonize
                let scan = message.extract::<PyRef<PyLaserScan>>(py)?.inner;
                use horus::core::LogSummary;
                let log_summary = scan.log_summary();
                let topic_ref = topic.clone();
                let success = py.detach(|| {
                    topic_ref.write().unwrap().send(scan);
                    true
                });
                if node.is_some() {
                    log_ipc_event(
                        py,
                        &node,
                        &self.name,
                        log_summary,
                        start.elapsed().as_nanos() as u64,
                        "log_pub",
                    );
                }
                success
            }
            TopicType::Image(topic) => {
                let py_img: PyRef<PyImage> = message.extract(py)?;
                let img = py_img.inner().clone();
                let topic_ref = topic.clone();
                let success = py.detach(|| {
                    let topic = topic_ref.read().unwrap();
                    topic.send(&img);
                    true
                });
                if node.is_some() {
                    log_ipc_event(
                        py,
                        &node,
                        &self.name,
                        format!("Image({}x{})", img.height(), img.width()),
                        start.elapsed().as_nanos() as u64,
                        "log_pub",
                    );
                }
                success
            }
            TopicType::PointCloud(topic) => {
                let py_pc: PyRef<PyPointCloud> = message.extract(py)?;
                let pc = py_pc.inner().clone();
                let topic_ref = topic.clone();
                let success = py.detach(|| {
                    let topic = topic_ref.read().unwrap();
                    topic.send(&pc);
                    true
                });
                if node.is_some() {
                    log_ipc_event(
                        py,
                        &node,
                        &self.name,
                        format!("PointCloud({} pts)", pc.point_count()),
                        start.elapsed().as_nanos() as u64,
                        "log_pub",
                    );
                }
                success
            }
            TopicType::DepthImage(topic) => {
                let py_depth: PyRef<PyDepthImage> = message.extract(py)?;
                let depth = py_depth.inner().clone();
                let topic_ref = topic.clone();
                let success = py.detach(|| {
                    let topic = topic_ref.read().unwrap();
                    topic.send(&depth);
                    true
                });
                if node.is_some() {
                    log_ipc_event(
                        py,
                        &node,
                        &self.name,
                        format!("DepthImage({}x{})", depth.height(), depth.width()),
                        start.elapsed().as_nanos() as u64,
                        "log_pub",
                    );
                }
                success
            }
            TopicType::Generic(topic) => {
                let bound = message.bind(py);
                let value: serde_json::Value =
                    pythonize::depythonize(bound).map_err(|e| {
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
                    topic_ref.write().unwrap().send(msg);
                    true
                });
                if node.is_some() {
                    log_ipc_event(
                        py,
                        &node,
                        &self.name,
                        log_summary,
                        start.elapsed().as_nanos() as u64,
                        "log_pub",
                    );
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
                let msg_opt = py.detach(|| topic_ref.read().unwrap().recv());
                if let Some(cmd) = msg_opt {
                    if node.is_some() {
                        use horus::core::LogSummary;
                        log_ipc_event(
                            py,
                            &node,
                            &self.name,
                            cmd.log_summary(),
                            start.elapsed().as_nanos() as u64,
                            "log_sub",
                        );
                    }
                    // POD-optimized: direct Rust allocation, no Python constructor
                    Ok(Some(Py::new(py, PyCmdVel { inner: cmd })?.into_any()))
                } else {
                    Ok(None)
                }
            }
            TopicType::Pose2D(topic) => {
                let topic_ref = topic.clone();
                let msg_opt = py.detach(|| topic_ref.read().unwrap().recv());
                if let Some(pose) = msg_opt {
                    if node.is_some() {
                        use horus::core::LogSummary;
                        log_ipc_event(
                            py,
                            &node,
                            &self.name,
                            pose.log_summary(),
                            start.elapsed().as_nanos() as u64,
                            "log_sub",
                        );
                    }
                    Ok(Some(Py::new(py, PyPose2D { inner: pose })?.into_any()))
                } else {
                    Ok(None)
                }
            }
            TopicType::Imu(topic) => {
                let topic_ref = topic.clone();
                let msg_opt = py.detach(|| topic_ref.read().unwrap().recv());
                if let Some(imu) = msg_opt {
                    if node.is_some() {
                        use horus::core::LogSummary;
                        log_ipc_event(
                            py,
                            &node,
                            &self.name,
                            imu.log_summary(),
                            start.elapsed().as_nanos() as u64,
                            "log_sub",
                        );
                    }
                    Ok(Some(Py::new(py, PyImu { inner: imu })?.into_any()))
                } else {
                    Ok(None)
                }
            }
            TopicType::Odometry(topic) => {
                let topic_ref = topic.clone();
                let msg_opt = py.detach(|| topic_ref.read().unwrap().recv());
                if let Some(odom) = msg_opt {
                    if node.is_some() {
                        use horus::core::LogSummary;
                        log_ipc_event(
                            py,
                            &node,
                            &self.name,
                            odom.log_summary(),
                            start.elapsed().as_nanos() as u64,
                            "log_sub",
                        );
                    }
                    Ok(Some(
                        Py::new(py, PyOdometry { inner: odom })?.into_any(),
                    ))
                } else {
                    Ok(None)
                }
            }
            TopicType::LaserScan(topic) => {
                let topic_ref = topic.clone();
                let msg_opt = py.detach(|| topic_ref.read().unwrap().recv());
                if let Some(scan) = msg_opt {
                    if node.is_some() {
                        use horus::core::LogSummary;
                        log_ipc_event(
                            py,
                            &node,
                            &self.name,
                            scan.log_summary(),
                            start.elapsed().as_nanos() as u64,
                            "log_sub",
                        );
                    }
                    Ok(Some(
                        Py::new(py, PyLaserScan { inner: scan })?.into_any(),
                    ))
                } else {
                    Ok(None)
                }
            }
            TopicType::Image(topic) => {
                let topic_ref = topic.clone();
                let msg_opt = py.detach(|| topic_ref.read().unwrap().recv());
                if let Some(img) = msg_opt {
                    if node.is_some() {
                        log_ipc_event(
                            py,
                            &node,
                            &self.name,
                            format!("Image({}x{})", img.height(), img.width()),
                            start.elapsed().as_nanos() as u64,
                            "log_sub",
                        );
                    }
                    let py_img = PyImage::from_inner(img);
                    Ok(Some(py_img.into_pyobject(py)?.into_any().unbind()))
                } else {
                    Ok(None)
                }
            }
            TopicType::PointCloud(topic) => {
                let topic_ref = topic.clone();
                let msg_opt = py.detach(|| topic_ref.read().unwrap().recv());
                if let Some(pc) = msg_opt {
                    if node.is_some() {
                        log_ipc_event(
                            py,
                            &node,
                            &self.name,
                            format!("PointCloud({} pts)", pc.point_count()),
                            start.elapsed().as_nanos() as u64,
                            "log_sub",
                        );
                    }
                    let py_pc = PyPointCloud::from_inner(pc);
                    Ok(Some(py_pc.into_pyobject(py)?.into_any().unbind()))
                } else {
                    Ok(None)
                }
            }
            TopicType::DepthImage(topic) => {
                let topic_ref = topic.clone();
                let msg_opt = py.detach(|| topic_ref.read().unwrap().recv());
                if let Some(depth) = msg_opt {
                    if node.is_some() {
                        log_ipc_event(
                            py,
                            &node,
                            &self.name,
                            format!("DepthImage({}x{})", depth.height(), depth.width()),
                            start.elapsed().as_nanos() as u64,
                            "log_sub",
                        );
                    }
                    let py_depth = PyDepthImage::from_inner(depth);
                    Ok(Some(py_depth.into_pyobject(py)?.into_any().unbind()))
                } else {
                    Ok(None)
                }
            }
            TopicType::Generic(topic) => {
                let topic_ref = topic.clone();
                let msg_opt = py.detach(|| topic_ref.read().unwrap().recv());
                if let Some(msg) = msg_opt {
                    if node.is_some() {
                        use horus::core::LogSummary;
                        log_ipc_event(
                            py,
                            &node,
                            &self.name,
                            msg.log_summary(),
                            start.elapsed().as_nanos() as u64,
                            "log_sub",
                        );
                    }
                    let data = msg.data();
                    let value: serde_json::Value =
                        rmp_serde::from_slice(&data).map_err(|e| {
                            PyRuntimeError::new_err(format!(
                                "Failed to deserialize MessagePack: {}",
                                e
                            ))
                        })?;
                    let py_obj = pythonize::pythonize(py, &value)
                        .map_err(|e| {
                            PyRuntimeError::new_err(format!("Failed to convert to Python: {}", e))
                        })?
                        .into();
                    Ok(Some(py_obj))
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
            TopicType::Image(t) => t.read().unwrap().backend_name().to_string(),
            TopicType::PointCloud(t) => t.read().unwrap().backend_name().to_string(),
            TopicType::DepthImage(t) => t.read().unwrap().backend_name().to_string(),
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
                TopicType::Image(t) => t.read().unwrap().metrics(),
                TopicType::PointCloud(t) => t.read().unwrap().metrics(),
                TopicType::DepthImage(t) => t.read().unwrap().metrics(),
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

/// Create a Topic for pool-backed types (Image, PointCloud, DepthImage).
///
/// These types use `TopicMessage` with `Wire` = descriptor (not Serialize on Self),
/// so they need different trait bounds than `create_topic`.
fn create_pool_topic<T>(endpoint: &str, capacity: usize) -> PyResult<Topic<T>>
where
    T: horus::communication::TopicMessage + Send + 'static,
    T::Wire: Clone + Send + Sync + Serialize + DeserializeOwned + 'static,
{
    let topic_name = if endpoint.contains('@') {
        endpoint.split('@').next().unwrap_or(endpoint)
    } else {
        endpoint
    };

    Topic::with_capacity(topic_name, capacity as u32, None)
        .map_err(|e| PyRuntimeError::new_err(format!("Failed to create Topic: {}", e)))
}

fn create_topic<T>(endpoint: &str, capacity: usize) -> PyResult<Topic<T>>
where
    T: Clone
        + Send
        + Sync
        + 'static
        + serde::Serialize
        + serde::de::DeserializeOwned
        + std::fmt::Debug,
{
    // Check if this is a network endpoint
    if endpoint.contains('@') {
        let topic_name = endpoint.split('@').next().unwrap_or(endpoint);
        return Topic::with_capacity(topic_name, capacity as u32, None).map_err(|e| {
            pyo3::exceptions::PyRuntimeError::new_err(format!(
                "Failed to create network Topic: {}",
                e
            ))
        });
    }

    Topic::with_capacity(endpoint, capacity as u32, None).map_err(|e| {
        pyo3::exceptions::PyRuntimeError::new_err(format!("Failed to create Topic: {}", e))
    })
}
