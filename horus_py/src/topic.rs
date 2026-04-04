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
use horus::memory::TensorHandle;
use horus_core::memory::{DepthImage, Image, PointCloud};
use horus_types::{
    Accel, AccelStamped, Clock, DiagnosticReport, DiagnosticStatus, DiagnosticValue, EmergencyStop,
    GenericMessage, Heartbeat, NodeHeartbeat, Point3, Pose2D, Pose3D, PoseStamped,
    PoseWithCovariance, Quaternion, ResourceUsage, SafetyStatus, TimeReference, TransformStamped,
    Twist, TwistWithCovariance, Vector3,
};
use horus_robotics::CmdVel;
use horus_robotics::messages::audio::AudioFrame;
use horus_robotics::messages::control::{
    DifferentialDriveCommand, JointCommand, MotorCommand, PidConfig, ServoCommand, TrajectoryPoint,
};
use horus_robotics::messages::detection::{BoundingBox2D, BoundingBox3D, Detection, Detection3D};
use horus_robotics::messages::force::{
    ContactInfo, ForceCommand, HapticFeedback, ImpedanceParameters, TactileArray, WrenchStamped,
};
use horus_robotics::messages::joystick_msg::JoystickInput;
use horus_robotics::messages::keyboard_input_msg::KeyboardInput;
use horus_robotics::messages::landmark::{Landmark, Landmark3D, LandmarkArray};
use horus_robotics::messages::navigation::{
    CostMap, GoalResult, NavGoal, NavPath, OccupancyGrid, PathPlan, VelocityObstacle,
    VelocityObstacles, Waypoint,
};
use horus_robotics::messages::perception::{PlaneArray, PlaneDetection, PointField};
use horus_robotics::messages::segmentation::SegmentationMask;
use horus_robotics::messages::sensor::{
    BatteryState, FluidPressure, Illuminance, Imu, JointState, LaserScan, MagneticField, NavSatFix,
    Odometry, RangeSensor, Temperature,
};
use horus_robotics::messages::tracking::{TrackedObject, TrackingHeader};
use horus_robotics::messages::vision::{CameraInfo, CompressedImage, RegionOfInterest, StereoInfo};
use pyo3::exceptions::PyRuntimeError;
use pyo3::prelude::*;
use serde::{de::DeserializeOwned, Serialize};
use std::sync::{Arc, RwLock};

use crate::depth_image::PyDepthImage;
use crate::image::PyImage;
use crate::messages::{
    PyAccel, PyAccelStamped, PyAudioFrame, PyBatteryState, PyBoundingBox2DMsg, PyBoundingBox3D,
    PyCameraInfo, PyClock, PyCmdVel, PyCompressedImage, PyContactInfo, PyCostMap, PyDetection3D,
    PyDetectionMsg, PyDiagnosticReport, PyDiagnosticStatus, PyDiagnosticValue,
    PyDifferentialDriveCommand, PyEmergencyStop, PyFluidPressure, PyForceCommand, PyGoalResult,
    PyHapticFeedback, PyHeartbeat, PyIlluminance, PyImpedanceParameters, PyImu, PyJointCommand,
    PyJointState, PyJoystickInput, PyKeyboardInput, PyLandmark3D, PyLandmarkArray, PyLandmarkMsg,
    PyLaserScan, PyMagneticField, PyMotorCommand, PyNavGoal, PyNavPath, PyNavSatFix,
    PyNodeHeartbeat, PyOccupancyGrid, PyOdometry, PyPathPlan, PyPidConfig, PyPlaneArray,
    PyPlaneDetection, PyPoint3, PyPointField, PyPose2D, PyPose3D, PyPoseStamped,
    PyPoseWithCovariance, PyQuaternion, PyRangeSensor, PyRegionOfInterest, PyResourceUsage,
    PySafetyStatus, PySegmentationMask, PyServoCommand, PyStereoInfo, PyTactileArray,
    PyTemperature, PyTimeReference, PyTrackedObjectMsg, PyTrackingHeader, PyTrajectoryPoint,
    PyTransformStamped, PyTwist, PyTwistWithCovariance, PyVector3, PyVelocityObstacle,
    PyVelocityObstacles, PyWaypoint, PyWrenchStamped,
};
use crate::pointcloud::PyPointCloud;
use crate::tensor::PyTensorHandle;

/// Acquire a read lock, converting a poisoned lock into a PyRuntimeError.
#[inline]
fn read_lock<T>(lock: &RwLock<T>) -> PyResult<std::sync::RwLockReadGuard<'_, T>> {
    lock.read()
        .map_err(|e| PyRuntimeError::new_err(format!("Topic lock poisoned: {e}")))
}

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
// TopicType enum + dispatch — generated by pod_topic_types! macro
// ============================================================================
//
// Adding a new POD message type? Just add (RustType, PyType) to the macro
// invocation below. The macro generates the enum, topic_dispatch!, and
// create_pod_topic() helper.

/// Generate TopicType enum, topic_dispatch! macro, and POD helper methods from
/// a list of (RustType, PyType) pairs. Pool-backed types and Generic are appended
/// manually since they have different send/recv/creation patterns.
macro_rules! pod_topic_types {
    ( $( ($rust_ty:ident, $py_ty:ident) ),* $(,)? ) => {
        enum TopicType {
            $( $rust_ty(Arc<RwLock<Topic<$rust_ty>>>), )*
            Image(Arc<RwLock<Topic<Image>>>),
            PointCloud(Arc<RwLock<Topic<PointCloud>>>),
            DepthImage(Arc<RwLock<Topic<DepthImage>>>),
            Tensor(Arc<RwLock<Topic<horus_core::types::Tensor>>>),
            Generic(Arc<RwLock<Topic<GenericMessage>>>),
        }

        macro_rules! topic_dispatch {
            ($topic_type_expr:expr, $t:ident, $body:expr) => {
                match $topic_type_expr {
                    $( TopicType::$rust_ty($t) => $body, )*
                    TopicType::Image($t) => $body,
                    TopicType::PointCloud($t) => $body,
                    TopicType::DepthImage($t) => $body,
                    TopicType::Tensor($t) => $body,
                    TopicType::Generic($t) => $body,
                }
            };
        }

        impl PyTopic {
            /// Create a TopicType from a type name string (POD types only).
            /// Returns None for special types handled by the caller.
            fn create_pod_topic(type_name: &str, endpoint: &str, cap: usize) -> PyResult<Option<TopicType>> {
                let tt = match type_name {
                    $( stringify!($rust_ty) => {
                        let topic = create_topic::<$rust_ty>(endpoint, cap)?;
                        TopicType::$rust_ty(Arc::new(RwLock::new(topic)))
                    }, )*
                    _ => return Ok(None),
                };
                Ok(Some(tt))
            }

            /// Send a POD message. Returns Some(true) on success, None if not a POD type.
            fn send_pod(
                &self, py: Python, message: &Py<PyAny>, node: &Option<Py<PyAny>>,
                start: std::time::Instant,
            ) -> PyResult<Option<bool>> {
                match &self.topic_type {
                    $(
                        TopicType::$rust_ty(topic) => {
                            let pyref = message.extract::<PyRef<$py_ty>>(py)?;
                            let val = pyref.inner.clone();
                            let summary = if node.is_some() {
                                use horus::core::LogSummary;
                                Some(val.log_summary())
                            } else { None };
                            let topic_ref = topic.clone();
                            py.detach(|| { topic_ref.write().expect("lock").send(val); });
                            if let Some(s) = summary {
                                log_ipc_event(py, node, &self.name, s,
                                    start.elapsed().as_nanos() as u64, "log_pub");
                            }
                            Ok(Some(true))
                        }
                    )*
                    _ => Ok(None), // Special types — caller handles
                }
            }

            /// Receive a POD message. Returns Ok(None) if no message, Err(None) if not POD.
            /// Uses a nested Option: outer = "is this a POD type?", inner = "is there a message?"
            fn recv_pod(
                &self, py: Python, node: &Option<Py<PyAny>>, start: std::time::Instant,
            ) -> PyResult<Option<Option<Py<PyAny>>>> {
                match &self.topic_type {
                    $(
                        TopicType::$rust_ty(topic) => {
                            let topic_ref = topic.clone();
                            let msg_opt = py.detach(|| {
                                topic_ref.read().expect("lock").recv()
                            });
                            if let Some(val) = msg_opt {
                                if node.is_some() {
                                    use horus::core::LogSummary;
                                    log_ipc_event(py, node, &self.name, val.log_summary(),
                                        start.elapsed().as_nanos() as u64, "log_sub");
                                }
                                Ok(Some(Some(Py::new(py, $py_ty { inner: val })?.into_any())))
                            } else {
                                Ok(Some(None))
                            }
                        }
                    )*
                    _ => Ok(None), // Special types — caller handles
                }
            }

            /// Get the Rust type name for this topic type (POD types only).
            #[allow(dead_code)]
            fn pod_type_name(&self) -> Option<&'static str> {
                match &self.topic_type {
                    $( TopicType::$rust_ty(_) => Some(stringify!($rust_ty)), )*
                    _ => None,
                }
            }
        }
    };
}

pod_topic_types!(
    (CmdVel, PyCmdVel),
    (Pose2D, PyPose2D),
    (Pose3D, PyPose3D),
    (Imu, PyImu),
    (Odometry, PyOdometry),
    (LaserScan, PyLaserScan),
    (JointState, PyJointState),
    (Clock, PyClock),
    (TimeReference, PyTimeReference),
    (Twist, PyTwist),
    (Vector3, PyVector3),
    (Point3, PyPoint3),
    (Quaternion, PyQuaternion),
    (TransformStamped, PyTransformStamped),
    (PoseStamped, PyPoseStamped),
    (PoseWithCovariance, PyPoseWithCovariance),
    (TwistWithCovariance, PyTwistWithCovariance),
    (Accel, PyAccel),
    (AccelStamped, PyAccelStamped),
    (MotorCommand, PyMotorCommand),
    (ServoCommand, PyServoCommand),
    (DifferentialDriveCommand, PyDifferentialDriveCommand),
    (PidConfig, PyPidConfig),
    (TrajectoryPoint, PyTrajectoryPoint),
    (JointCommand, PyJointCommand),
    (RangeSensor, PyRangeSensor),
    (BatteryState, PyBatteryState),
    (NavSatFix, PyNavSatFix),
    (MagneticField, PyMagneticField),
    (Temperature, PyTemperature),
    (FluidPressure, PyFluidPressure),
    (Illuminance, PyIlluminance),
    (Heartbeat, PyHeartbeat),
    (DiagnosticStatus, PyDiagnosticStatus),
    (EmergencyStop, PyEmergencyStop),
    (ResourceUsage, PyResourceUsage),
    (WrenchStamped, PyWrenchStamped),
    (ForceCommand, PyForceCommand),
    (ContactInfo, PyContactInfo),
    (NavGoal, PyNavGoal),
    (GoalResult, PyGoalResult),
    (PathPlan, PyPathPlan),
    (JoystickInput, PyJoystickInput),
    (KeyboardInput, PyKeyboardInput),
    (BoundingBox2D, PyBoundingBox2DMsg),
    (BoundingBox3D, PyBoundingBox3D),
    (Detection, PyDetectionMsg),
    (Detection3D, PyDetection3D),
    (SegmentationMask, PySegmentationMask),
    (TrackedObject, PyTrackedObjectMsg),
    (TrackingHeader, PyTrackingHeader),
    (Landmark, PyLandmarkMsg),
    (Landmark3D, PyLandmark3D),
    (LandmarkArray, PyLandmarkArray),
    (PointField, PyPointField),
    (PlaneDetection, PyPlaneDetection),
    (PlaneArray, PyPlaneArray),
    (CompressedImage, PyCompressedImage),
    (CameraInfo, PyCameraInfo),
    (RegionOfInterest, PyRegionOfInterest),
    (StereoInfo, PyStereoInfo),
    (ImpedanceParameters, PyImpedanceParameters),
    (HapticFeedback, PyHapticFeedback),
    (TactileArray, PyTactileArray),
    (DiagnosticValue, PyDiagnosticValue),
    (DiagnosticReport, PyDiagnosticReport),
    (NodeHeartbeat, PyNodeHeartbeat),
    (SafetyStatus, PySafetyStatus),
    (Waypoint, PyWaypoint),
    (NavPath, PyNavPath),
    (VelocityObstacle, PyVelocityObstacle),
    (VelocityObstacles, PyVelocityObstacles),
    (OccupancyGrid, PyOccupancyGrid),
    (CostMap, PyCostMap),
    (AudioFrame, PyAudioFrame),
);

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
                "Topic() requires a message type or topic string.\n\
                 Common types: CmdVel, Pose2D, Pose3D, Twist, Imu, Odometry, LaserScan, \
                 JointState, Image, PointCloud, DepthImage, NavGoal, Heartbeat, \
                 MotorCommand, ServoCommand, Detection, WrenchStamped, BatteryState.\n\
                 Usage: Topic(CmdVel) or Topic(CmdVel, capacity=2048) or Topic(\"my_topic\")",
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

        // Create typed Topic. POD types are handled by macro-generated create_pod_topic().
        let topic_type = if let Some(tt) =
            Self::create_pod_topic(&type_name, &effective_endpoint, cap)?
        {
            tt
        } else {
            // Special types: pool-backed + generic
            match type_name.as_str() {
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
                "Tensor" | "TensorHandle" => {
                    let topic =
                        create_pool_topic::<horus_core::types::Tensor>(&effective_endpoint, cap)?;
                    TopicType::Tensor(Arc::new(RwLock::new(topic)))
                }
                _ => {
                    let topic = create_topic::<GenericMessage>(&effective_endpoint, cap)?;
                    TopicType::Generic(Arc::new(RwLock::new(topic)))
                }
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

        // Fast path: POD types (generated by pod_topic_types! macro)
        if let Some(success) = self.send_pod(py, &message, &node, start)? {
            return Ok(success);
        }

        // Special types: pool-backed + generic
        let result = match &self.topic_type {
            TopicType::Image(topic) => {
                let py_img: PyRef<PyImage> = message.extract(py)?;
                let img = py_img.inner().clone();
                let topic_ref = topic.clone();
                let success = py.detach(|| {
                    topic_ref.read().expect("topic lock poisoned").send(&img);
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
                    topic_ref.read().expect("topic lock poisoned").send(&pc);
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
                    topic_ref.read().expect("topic lock poisoned").send(&depth);
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
            TopicType::Tensor(topic) => {
                let py_tensor: PyRef<PyTensorHandle> = message.extract(py)?;
                let handle = py_tensor
                    .handle
                    .as_ref()
                    .ok_or_else(|| PyRuntimeError::new_err("Tensor has been released"))?;
                handle.pool().retain(handle.tensor());
                let descriptor = *handle.tensor();
                let log_msg = format!(
                    "Tensor(shape={:?}, dtype={})",
                    handle.shape(),
                    handle.dtype()
                );
                let topic_ref = topic.clone();
                let success = py.detach(|| {
                    topic_ref
                        .write()
                        .expect("topic lock poisoned")
                        .send(descriptor);
                    true
                });
                if node.is_some() {
                    log_ipc_event(
                        py,
                        &node,
                        &self.name,
                        log_msg,
                        start.elapsed().as_nanos() as u64,
                        "log_pub",
                    );
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
                    .map_err(|e| pyo3::exceptions::PyValueError::new_err(e.to_string()))?;
                use horus::core::LogSummary;
                let log_summary = msg.log_summary();
                let topic_ref = topic.clone();
                let success = py.detach(|| {
                    topic_ref.write().expect("topic lock poisoned").send(msg);
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
            _ => unreachable!("send_pod handles all POD types"),
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

        // Fast path: POD types (generated by pod_topic_types! macro)
        if let Some(result) = self.recv_pod(py, &node, start)? {
            return Ok(result);
        }

        // Special types: pool-backed + generic
        match &self.topic_type {
            TopicType::Image(topic) => {
                let topic_ref = topic.clone();
                let msg_opt = py.detach(|| topic_ref.read().expect("topic lock poisoned").recv());
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
                let msg_opt = py.detach(|| topic_ref.read().expect("topic lock poisoned").recv());
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
                let msg_opt = py.detach(|| topic_ref.read().expect("topic lock poisoned").recv());
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
            TopicType::Tensor(topic) => {
                let topic_ref = topic.clone();
                let msg_opt = py.detach(|| topic_ref.read().expect("topic lock poisoned").recv());
                if let Some(descriptor) = msg_opt {
                    let pool = { topic_ref.read().expect("topic lock").pool() };
                    let handle = TensorHandle::new(descriptor, pool);
                    if node.is_some() {
                        log_ipc_event(
                            py,
                            &node,
                            &self.name,
                            format!("Tensor(shape={:?})", handle.shape()),
                            start.elapsed().as_nanos() as u64,
                            "log_sub",
                        );
                    }
                    Ok(Some(
                        Py::new(
                            py,
                            PyTensorHandle {
                                handle: Some(handle),
                            },
                        )?
                        .into_any(),
                    ))
                } else {
                    Ok(None)
                }
            }
            TopicType::Generic(topic) => {
                let topic_ref = topic.clone();
                let msg_opt = py.detach(|| topic_ref.read().expect("topic lock poisoned").recv());
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
                    let value: serde_json::Value = rmp_serde::from_slice(&data).map_err(|e| {
                        PyRuntimeError::new_err(format!("Failed to deserialize MessagePack: {}", e))
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
            _ => unreachable!("recv_pod handles all POD types"),
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
            TopicType::CmdVel(t) => read_lock(t)
                .map(|g| g.backend_name().to_string())
                .unwrap_or_default(),
            TopicType::Pose2D(t) => read_lock(t)
                .map(|g| g.backend_name().to_string())
                .unwrap_or_default(),
            TopicType::Pose3D(t) => read_lock(t)
                .map(|g| g.backend_name().to_string())
                .unwrap_or_default(),
            TopicType::Imu(t) => read_lock(t)
                .map(|g| g.backend_name().to_string())
                .unwrap_or_default(),
            TopicType::Odometry(t) => read_lock(t)
                .map(|g| g.backend_name().to_string())
                .unwrap_or_default(),
            TopicType::LaserScan(t) => read_lock(t)
                .map(|g| g.backend_name().to_string())
                .unwrap_or_default(),
            TopicType::JointState(t) => read_lock(t)
                .map(|g| g.backend_name().to_string())
                .unwrap_or_default(),
            TopicType::Clock(t) => read_lock(t)
                .map(|g| g.backend_name().to_string())
                .unwrap_or_default(),
            TopicType::TimeReference(t) => read_lock(t)
                .map(|g| g.backend_name().to_string())
                .unwrap_or_default(),
            TopicType::Image(t) => read_lock(t)
                .map(|g| g.backend_name().to_string())
                .unwrap_or_default(),
            TopicType::PointCloud(t) => read_lock(t)
                .map(|g| g.backend_name().to_string())
                .unwrap_or_default(),
            TopicType::DepthImage(t) => read_lock(t)
                .map(|g| g.backend_name().to_string())
                .unwrap_or_default(),
            // Geometry types
            TopicType::Twist(t) => read_lock(t)
                .map(|g| g.backend_name().to_string())
                .unwrap_or_default(),
            TopicType::Vector3(t) => read_lock(t)
                .map(|g| g.backend_name().to_string())
                .unwrap_or_default(),
            TopicType::Point3(t) => read_lock(t)
                .map(|g| g.backend_name().to_string())
                .unwrap_or_default(),
            TopicType::Quaternion(t) => read_lock(t)
                .map(|g| g.backend_name().to_string())
                .unwrap_or_default(),
            TopicType::TransformStamped(t) => read_lock(t)
                .map(|g| g.backend_name().to_string())
                .unwrap_or_default(),
            TopicType::PoseStamped(t) => read_lock(t)
                .map(|g| g.backend_name().to_string())
                .unwrap_or_default(),
            TopicType::PoseWithCovariance(t) => read_lock(t)
                .map(|g| g.backend_name().to_string())
                .unwrap_or_default(),
            TopicType::TwistWithCovariance(t) => read_lock(t)
                .map(|g| g.backend_name().to_string())
                .unwrap_or_default(),
            TopicType::Accel(t) => read_lock(t)
                .map(|g| g.backend_name().to_string())
                .unwrap_or_default(),
            TopicType::AccelStamped(t) => read_lock(t)
                .map(|g| g.backend_name().to_string())
                .unwrap_or_default(),
            // Control types
            TopicType::MotorCommand(t) => read_lock(t)
                .map(|g| g.backend_name().to_string())
                .unwrap_or_default(),
            TopicType::ServoCommand(t) => read_lock(t)
                .map(|g| g.backend_name().to_string())
                .unwrap_or_default(),
            TopicType::DifferentialDriveCommand(t) => read_lock(t)
                .map(|g| g.backend_name().to_string())
                .unwrap_or_default(),
            TopicType::PidConfig(t) => read_lock(t)
                .map(|g| g.backend_name().to_string())
                .unwrap_or_default(),
            TopicType::TrajectoryPoint(t) => read_lock(t)
                .map(|g| g.backend_name().to_string())
                .unwrap_or_default(),
            TopicType::JointCommand(t) => read_lock(t)
                .map(|g| g.backend_name().to_string())
                .unwrap_or_default(),
            // Sensor types
            TopicType::RangeSensor(t) => read_lock(t)
                .map(|g| g.backend_name().to_string())
                .unwrap_or_default(),
            TopicType::BatteryState(t) => read_lock(t)
                .map(|g| g.backend_name().to_string())
                .unwrap_or_default(),
            TopicType::NavSatFix(t) => read_lock(t)
                .map(|g| g.backend_name().to_string())
                .unwrap_or_default(),
            TopicType::MagneticField(t) => read_lock(t)
                .map(|g| g.backend_name().to_string())
                .unwrap_or_default(),
            TopicType::Temperature(t) => read_lock(t)
                .map(|g| g.backend_name().to_string())
                .unwrap_or_default(),
            TopicType::FluidPressure(t) => read_lock(t)
                .map(|g| g.backend_name().to_string())
                .unwrap_or_default(),
            TopicType::Illuminance(t) => read_lock(t)
                .map(|g| g.backend_name().to_string())
                .unwrap_or_default(),
            // Diagnostics types
            TopicType::Heartbeat(t) => read_lock(t)
                .map(|g| g.backend_name().to_string())
                .unwrap_or_default(),
            TopicType::DiagnosticStatus(t) => read_lock(t)
                .map(|g| g.backend_name().to_string())
                .unwrap_or_default(),
            TopicType::EmergencyStop(t) => read_lock(t)
                .map(|g| g.backend_name().to_string())
                .unwrap_or_default(),
            TopicType::ResourceUsage(t) => read_lock(t)
                .map(|g| g.backend_name().to_string())
                .unwrap_or_default(),
            // Force types
            TopicType::WrenchStamped(t) => read_lock(t)
                .map(|g| g.backend_name().to_string())
                .unwrap_or_default(),
            TopicType::ForceCommand(t) => read_lock(t)
                .map(|g| g.backend_name().to_string())
                .unwrap_or_default(),
            TopicType::ContactInfo(t) => read_lock(t)
                .map(|g| g.backend_name().to_string())
                .unwrap_or_default(),
            // Navigation types
            TopicType::NavGoal(t) => read_lock(t)
                .map(|g| g.backend_name().to_string())
                .unwrap_or_default(),
            TopicType::GoalResult(t) => read_lock(t)
                .map(|g| g.backend_name().to_string())
                .unwrap_or_default(),
            TopicType::PathPlan(t) => read_lock(t)
                .map(|g| g.backend_name().to_string())
                .unwrap_or_default(),
            // Input types
            TopicType::JoystickInput(t) => read_lock(t)
                .map(|g| g.backend_name().to_string())
                .unwrap_or_default(),
            TopicType::KeyboardInput(t) => read_lock(t)
                .map(|g| g.backend_name().to_string())
                .unwrap_or_default(),
            // Detection/Perception types
            TopicType::BoundingBox2D(t) => read_lock(t)
                .map(|g| g.backend_name().to_string())
                .unwrap_or_default(),
            TopicType::BoundingBox3D(t) => read_lock(t)
                .map(|g| g.backend_name().to_string())
                .unwrap_or_default(),
            TopicType::Detection(t) => read_lock(t)
                .map(|g| g.backend_name().to_string())
                .unwrap_or_default(),
            TopicType::Detection3D(t) => read_lock(t)
                .map(|g| g.backend_name().to_string())
                .unwrap_or_default(),
            TopicType::SegmentationMask(t) => read_lock(t)
                .map(|g| g.backend_name().to_string())
                .unwrap_or_default(),
            // Tracking types
            TopicType::TrackedObject(t) => read_lock(t)
                .map(|g| g.backend_name().to_string())
                .unwrap_or_default(),
            TopicType::TrackingHeader(t) => read_lock(t)
                .map(|g| g.backend_name().to_string())
                .unwrap_or_default(),
            // Landmark types
            TopicType::Landmark(t) => read_lock(t)
                .map(|g| g.backend_name().to_string())
                .unwrap_or_default(),
            TopicType::Landmark3D(t) => read_lock(t)
                .map(|g| g.backend_name().to_string())
                .unwrap_or_default(),
            TopicType::LandmarkArray(t) => read_lock(t)
                .map(|g| g.backend_name().to_string())
                .unwrap_or_default(),
            // Perception helper types
            TopicType::PointField(t) => read_lock(t)
                .map(|g| g.backend_name().to_string())
                .unwrap_or_default(),
            TopicType::PlaneDetection(t) => read_lock(t)
                .map(|g| g.backend_name().to_string())
                .unwrap_or_default(),
            TopicType::PlaneArray(t) => read_lock(t)
                .map(|g| g.backend_name().to_string())
                .unwrap_or_default(),
            // Vision types
            TopicType::CompressedImage(t) => read_lock(t)
                .map(|g| g.backend_name().to_string())
                .unwrap_or_default(),
            TopicType::CameraInfo(t) => read_lock(t)
                .map(|g| g.backend_name().to_string())
                .unwrap_or_default(),
            TopicType::RegionOfInterest(t) => read_lock(t)
                .map(|g| g.backend_name().to_string())
                .unwrap_or_default(),
            TopicType::StereoInfo(t) => read_lock(t)
                .map(|g| g.backend_name().to_string())
                .unwrap_or_default(),
            // Force types (additional)
            TopicType::ImpedanceParameters(t) => read_lock(t)
                .map(|g| g.backend_name().to_string())
                .unwrap_or_default(),
            TopicType::HapticFeedback(t) => read_lock(t)
                .map(|g| g.backend_name().to_string())
                .unwrap_or_default(),
            TopicType::TactileArray(t) => read_lock(t)
                .map(|g| g.backend_name().to_string())
                .unwrap_or_default(),
            // Diagnostics types (additional)
            TopicType::DiagnosticValue(t) => read_lock(t)
                .map(|g| g.backend_name().to_string())
                .unwrap_or_default(),
            TopicType::DiagnosticReport(t) => read_lock(t)
                .map(|g| g.backend_name().to_string())
                .unwrap_or_default(),
            TopicType::NodeHeartbeat(t) => read_lock(t)
                .map(|g| g.backend_name().to_string())
                .unwrap_or_default(),
            TopicType::SafetyStatus(t) => read_lock(t)
                .map(|g| g.backend_name().to_string())
                .unwrap_or_default(),
            // Navigation types (additional)
            TopicType::Waypoint(t) => read_lock(t)
                .map(|g| g.backend_name().to_string())
                .unwrap_or_default(),
            TopicType::NavPath(t) => read_lock(t)
                .map(|g| g.backend_name().to_string())
                .unwrap_or_default(),
            TopicType::VelocityObstacle(t) => read_lock(t)
                .map(|g| g.backend_name().to_string())
                .unwrap_or_default(),
            TopicType::VelocityObstacles(t) => read_lock(t)
                .map(|g| g.backend_name().to_string())
                .unwrap_or_default(),
            TopicType::OccupancyGrid(t) => read_lock(t)
                .map(|g| g.backend_name().to_string())
                .unwrap_or_default(),
            TopicType::CostMap(t) => read_lock(t)
                .map(|g| g.backend_name().to_string())
                .unwrap_or_default(),
            TopicType::AudioFrame(t) => read_lock(t)
                .map(|g| g.backend_name().to_string())
                .unwrap_or_default(),
            TopicType::Tensor(t) => read_lock(t)
                .map(|g| g.backend_name().to_string())
                .unwrap_or_default(),
            TopicType::Generic(t) => read_lock(t)
                .map(|g| g.backend_name().to_string())
                .unwrap_or_default(),
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
                TopicType::CmdVel(t) => read_lock(t)?.metrics(),
                TopicType::Pose2D(t) => read_lock(t)?.metrics(),
                TopicType::Pose3D(t) => read_lock(t)?.metrics(),
                TopicType::Imu(t) => read_lock(t)?.metrics(),
                TopicType::Odometry(t) => read_lock(t)?.metrics(),
                TopicType::LaserScan(t) => read_lock(t)?.metrics(),
                TopicType::JointState(t) => read_lock(t)?.metrics(),
                TopicType::Clock(t) => read_lock(t)?.metrics(),
                TopicType::TimeReference(t) => read_lock(t)?.metrics(),
                TopicType::Image(t) => read_lock(t)?.metrics(),
                TopicType::PointCloud(t) => read_lock(t)?.metrics(),
                TopicType::DepthImage(t) => read_lock(t)?.metrics(),
                // Geometry types
                TopicType::Twist(t) => read_lock(t)?.metrics(),
                TopicType::Vector3(t) => read_lock(t)?.metrics(),
                TopicType::Point3(t) => read_lock(t)?.metrics(),
                TopicType::Quaternion(t) => read_lock(t)?.metrics(),
                TopicType::TransformStamped(t) => read_lock(t)?.metrics(),
                TopicType::PoseStamped(t) => read_lock(t)?.metrics(),
                TopicType::PoseWithCovariance(t) => read_lock(t)?.metrics(),
                TopicType::TwistWithCovariance(t) => read_lock(t)?.metrics(),
                TopicType::Accel(t) => read_lock(t)?.metrics(),
                TopicType::AccelStamped(t) => read_lock(t)?.metrics(),
                // Control types
                TopicType::MotorCommand(t) => read_lock(t)?.metrics(),
                TopicType::ServoCommand(t) => read_lock(t)?.metrics(),
                TopicType::DifferentialDriveCommand(t) => read_lock(t)?.metrics(),
                TopicType::PidConfig(t) => read_lock(t)?.metrics(),
                TopicType::TrajectoryPoint(t) => read_lock(t)?.metrics(),
                TopicType::JointCommand(t) => read_lock(t)?.metrics(),
                // Sensor types
                TopicType::RangeSensor(t) => read_lock(t)?.metrics(),
                TopicType::BatteryState(t) => read_lock(t)?.metrics(),
                TopicType::NavSatFix(t) => read_lock(t)?.metrics(),
                TopicType::MagneticField(t) => read_lock(t)?.metrics(),
                TopicType::Temperature(t) => read_lock(t)?.metrics(),
                TopicType::FluidPressure(t) => read_lock(t)?.metrics(),
                TopicType::Illuminance(t) => read_lock(t)?.metrics(),
                // Diagnostics types
                TopicType::Heartbeat(t) => read_lock(t)?.metrics(),
                TopicType::DiagnosticStatus(t) => read_lock(t)?.metrics(),
                TopicType::EmergencyStop(t) => read_lock(t)?.metrics(),
                TopicType::ResourceUsage(t) => read_lock(t)?.metrics(),
                // Force types
                TopicType::WrenchStamped(t) => read_lock(t)?.metrics(),
                TopicType::ForceCommand(t) => read_lock(t)?.metrics(),
                TopicType::ContactInfo(t) => read_lock(t)?.metrics(),
                // Navigation types
                TopicType::NavGoal(t) => read_lock(t)?.metrics(),
                TopicType::GoalResult(t) => read_lock(t)?.metrics(),
                TopicType::PathPlan(t) => read_lock(t)?.metrics(),
                // Input types
                TopicType::JoystickInput(t) => read_lock(t)?.metrics(),
                TopicType::KeyboardInput(t) => read_lock(t)?.metrics(),
                // Detection/Perception types
                TopicType::BoundingBox2D(t) => read_lock(t)?.metrics(),
                TopicType::BoundingBox3D(t) => read_lock(t)?.metrics(),
                TopicType::Detection(t) => read_lock(t)?.metrics(),
                TopicType::Detection3D(t) => read_lock(t)?.metrics(),
                TopicType::SegmentationMask(t) => read_lock(t)?.metrics(),
                // Tracking types
                TopicType::TrackedObject(t) => read_lock(t)?.metrics(),
                TopicType::TrackingHeader(t) => read_lock(t)?.metrics(),
                // Landmark types
                TopicType::Landmark(t) => read_lock(t)?.metrics(),
                TopicType::Landmark3D(t) => read_lock(t)?.metrics(),
                TopicType::LandmarkArray(t) => read_lock(t)?.metrics(),
                // Perception helper types
                TopicType::PointField(t) => read_lock(t)?.metrics(),
                TopicType::PlaneDetection(t) => read_lock(t)?.metrics(),
                TopicType::PlaneArray(t) => read_lock(t)?.metrics(),
                // Vision types
                TopicType::CompressedImage(t) => read_lock(t)?.metrics(),
                TopicType::CameraInfo(t) => read_lock(t)?.metrics(),
                TopicType::RegionOfInterest(t) => read_lock(t)?.metrics(),
                TopicType::StereoInfo(t) => read_lock(t)?.metrics(),
                // Force types (additional)
                TopicType::ImpedanceParameters(t) => read_lock(t)?.metrics(),
                TopicType::HapticFeedback(t) => read_lock(t)?.metrics(),
                TopicType::TactileArray(t) => read_lock(t)?.metrics(),
                // Diagnostics types (additional)
                TopicType::DiagnosticValue(t) => read_lock(t)?.metrics(),
                TopicType::DiagnosticReport(t) => read_lock(t)?.metrics(),
                TopicType::NodeHeartbeat(t) => read_lock(t)?.metrics(),
                TopicType::SafetyStatus(t) => read_lock(t)?.metrics(),
                // Navigation types (additional)
                TopicType::Waypoint(t) => read_lock(t)?.metrics(),
                TopicType::NavPath(t) => read_lock(t)?.metrics(),
                TopicType::VelocityObstacle(t) => read_lock(t)?.metrics(),
                TopicType::VelocityObstacles(t) => read_lock(t)?.metrics(),
                TopicType::OccupancyGrid(t) => read_lock(t)?.metrics(),
                TopicType::CostMap(t) => read_lock(t)?.metrics(),
                TopicType::AudioFrame(t) => read_lock(t)?.metrics(),
                TopicType::Tensor(t) => read_lock(t)?.metrics(),
                TopicType::Generic(t) => read_lock(t)?.metrics(),
            };

            dict.set_item("messages_sent", metrics.messages_sent())?;
            dict.set_item("messages_received", metrics.messages_received())?;
            dict.set_item("send_failures", metrics.send_failures())?;
            dict.set_item("recv_failures", metrics.recv_failures())?;
            dict.set_item("is_network", self.is_network)?;
            dict.set_item("backend", self.backend_type())?;

            Ok(dict.into())
        })
    }

    /// Check if this is a generic topic (supports metadata methods)
    fn is_generic(&self) -> bool {
        matches!(self.topic_type, TopicType::Generic(_))
    }

    /// Non-blocking receive — returns message or None without logging overhead.
    ///
    /// Equivalent to recv() with no node parameter. Use this in hot sensor
    /// polling loops where you want minimal overhead.
    ///
    /// Returns:
    ///     Message object or None if no message is available
    fn try_recv(&self, py: Python) -> PyResult<Option<Py<PyAny>>> {
        self.recv(py, None)
    }

    /// Number of unconsumed messages in the ring buffer.
    ///
    /// Returns:
    ///     Number of pending messages (u64)
    fn pending_count(&self) -> u64 {
        topic_dispatch!(
            &self.topic_type,
            t,
            read_lock(t).map(|g| g.pending_count()).unwrap_or(0)
        )
    }

    /// Number of active publishers on this topic.
    ///
    /// Returns:
    ///     Publisher count (u32)
    fn pub_count(&self) -> u32 {
        topic_dispatch!(
            &self.topic_type,
            t,
            read_lock(t).map(|g| g.pub_count()).unwrap_or(0)
        )
    }

    /// Number of active subscribers on this topic.
    ///
    /// Returns:
    ///     Subscriber count (u32)
    fn sub_count(&self) -> u32 {
        topic_dispatch!(
            &self.topic_type,
            t,
            read_lock(t).map(|g| g.sub_count()).unwrap_or(0)
        )
    }

    /// Read the most recent message without consuming it.
    ///
    /// Unlike recv(), this does not advance the read pointer — the same
    /// message can be read again. Returns the newest published message,
    /// not the oldest unread. Ideal for "always use the freshest sensor
    /// reading" patterns.
    ///
    /// Only available for Copy (POD) message types. Returns an error for
    /// types with dynamic allocations (ML types, CompressedImage, etc.).
    ///
    /// Returns:
    ///     Latest message or None if no messages have been published
    fn read_latest(&self, py: Python) -> PyResult<Option<Py<PyAny>>> {
        macro_rules! rl {
            ($t:expr, $py:expr, $PyT:ident) => {{
                let tr = $t.clone();
                let msg_opt = $py.detach(|| tr.read().expect("topic lock poisoned").read_latest());
                match msg_opt {
                    Some(msg) => Ok(Some(Py::new($py, $PyT { inner: msg })?.into_any())),
                    None => Ok(None),
                }
            }};
        }
        match &self.topic_type {
            // Core POD types
            TopicType::CmdVel(t) => rl!(t, py, PyCmdVel),
            TopicType::Pose2D(t) => rl!(t, py, PyPose2D),
            TopicType::Pose3D(t) => rl!(t, py, PyPose3D),
            TopicType::Imu(t) => rl!(t, py, PyImu),
            TopicType::Odometry(t) => rl!(t, py, PyOdometry),
            TopicType::LaserScan(t) => rl!(t, py, PyLaserScan),
            TopicType::JointState(t) => rl!(t, py, PyJointState),
            TopicType::Clock(t) => rl!(t, py, PyClock),
            TopicType::TimeReference(t) => rl!(t, py, PyTimeReference),
            // Geometry types
            TopicType::Twist(t) => rl!(t, py, PyTwist),
            TopicType::Vector3(t) => rl!(t, py, PyVector3),
            TopicType::Point3(t) => rl!(t, py, PyPoint3),
            TopicType::Quaternion(t) => rl!(t, py, PyQuaternion),
            TopicType::TransformStamped(t) => rl!(t, py, PyTransformStamped),
            TopicType::PoseStamped(t) => rl!(t, py, PyPoseStamped),
            TopicType::PoseWithCovariance(t) => rl!(t, py, PyPoseWithCovariance),
            TopicType::TwistWithCovariance(t) => rl!(t, py, PyTwistWithCovariance),
            TopicType::Accel(t) => rl!(t, py, PyAccel),
            TopicType::AccelStamped(t) => rl!(t, py, PyAccelStamped),
            // Control types
            TopicType::MotorCommand(t) => rl!(t, py, PyMotorCommand),
            TopicType::ServoCommand(t) => rl!(t, py, PyServoCommand),
            TopicType::DifferentialDriveCommand(t) => rl!(t, py, PyDifferentialDriveCommand),
            TopicType::PidConfig(t) => rl!(t, py, PyPidConfig),
            TopicType::TrajectoryPoint(t) => rl!(t, py, PyTrajectoryPoint),
            TopicType::JointCommand(t) => rl!(t, py, PyJointCommand),
            // Sensor types
            TopicType::RangeSensor(t) => rl!(t, py, PyRangeSensor),
            TopicType::BatteryState(t) => rl!(t, py, PyBatteryState),
            TopicType::NavSatFix(t) => rl!(t, py, PyNavSatFix),
            TopicType::MagneticField(t) => rl!(t, py, PyMagneticField),
            TopicType::Temperature(t) => rl!(t, py, PyTemperature),
            TopicType::FluidPressure(t) => rl!(t, py, PyFluidPressure),
            TopicType::Illuminance(t) => rl!(t, py, PyIlluminance),
            // Diagnostics types
            TopicType::Heartbeat(t) => rl!(t, py, PyHeartbeat),
            TopicType::DiagnosticStatus(t) => rl!(t, py, PyDiagnosticStatus),
            TopicType::EmergencyStop(t) => rl!(t, py, PyEmergencyStop),
            TopicType::ResourceUsage(t) => rl!(t, py, PyResourceUsage),
            TopicType::DiagnosticValue(t) => rl!(t, py, PyDiagnosticValue),
            TopicType::DiagnosticReport(t) => rl!(t, py, PyDiagnosticReport),
            TopicType::NodeHeartbeat(t) => rl!(t, py, PyNodeHeartbeat),
            TopicType::SafetyStatus(t) => rl!(t, py, PySafetyStatus),
            // Force types
            TopicType::WrenchStamped(t) => rl!(t, py, PyWrenchStamped),
            TopicType::ForceCommand(t) => rl!(t, py, PyForceCommand),
            TopicType::ContactInfo(t) => rl!(t, py, PyContactInfo),
            TopicType::ImpedanceParameters(t) => rl!(t, py, PyImpedanceParameters),
            TopicType::HapticFeedback(t) => rl!(t, py, PyHapticFeedback),
            // Navigation types (Copy)
            TopicType::NavGoal(t) => rl!(t, py, PyNavGoal),
            TopicType::GoalResult(t) => rl!(t, py, PyGoalResult),
            TopicType::PathPlan(t) => rl!(t, py, PyPathPlan),
            TopicType::Waypoint(t) => rl!(t, py, PyWaypoint),
            TopicType::NavPath(t) => rl!(t, py, PyNavPath),
            TopicType::VelocityObstacle(t) => rl!(t, py, PyVelocityObstacle),
            TopicType::VelocityObstacles(t) => rl!(t, py, PyVelocityObstacles),
            // Input types
            TopicType::JoystickInput(t) => rl!(t, py, PyJoystickInput),
            TopicType::KeyboardInput(t) => rl!(t, py, PyKeyboardInput),
            // Detection/Perception types
            TopicType::BoundingBox2D(t) => rl!(t, py, PyBoundingBox2DMsg),
            TopicType::BoundingBox3D(t) => rl!(t, py, PyBoundingBox3D),
            TopicType::Detection(t) => rl!(t, py, PyDetectionMsg),
            TopicType::Detection3D(t) => rl!(t, py, PyDetection3D),
            TopicType::SegmentationMask(t) => rl!(t, py, PySegmentationMask),
            // Tracking types
            TopicType::TrackedObject(t) => rl!(t, py, PyTrackedObjectMsg),
            TopicType::TrackingHeader(t) => rl!(t, py, PyTrackingHeader),
            // Landmark types
            TopicType::Landmark(t) => rl!(t, py, PyLandmarkMsg),
            TopicType::Landmark3D(t) => rl!(t, py, PyLandmark3D),
            TopicType::LandmarkArray(t) => rl!(t, py, PyLandmarkArray),
            // Perception helper types
            TopicType::PointField(t) => rl!(t, py, PyPointField),
            TopicType::PlaneDetection(t) => rl!(t, py, PyPlaneDetection),
            // Vision types (Copy)
            TopicType::CameraInfo(t) => rl!(t, py, PyCameraInfo),
            TopicType::RegionOfInterest(t) => rl!(t, py, PyRegionOfInterest),
            TopicType::StereoInfo(t) => rl!(t, py, PyStereoInfo),
            // Audio types (Copy)
            TopicType::AudioFrame(t) => rl!(t, py, PyAudioFrame),
            // Non-Copy types: pool-backed, dynamic
            TopicType::Image(_) | TopicType::PointCloud(_) | TopicType::DepthImage(_)
            | TopicType::OccupancyGrid(_) | TopicType::CostMap(_)
            | TopicType::Tensor(_)
            | TopicType::CompressedImage(_) | TopicType::PlaneArray(_)
            | TopicType::TactileArray(_) | TopicType::Generic(_) => {
                Err(PyRuntimeError::new_err(
                    "read_latest() not supported for this type (requires fixed-size POD). Use recv() instead."
                ))
            }
        }
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

    Topic::with_capacity(topic_name, capacity as u32, None).map_err(|e| {
        PyRuntimeError::new_err(format!(
            "Failed to create Topic '{}': {}. Common causes: invalid topic name \
             (allowed: a-z, 0-9, _, -, ., /), SHM permission denied, \
             or insufficient shared memory space. Run: horus clean --shm",
            topic_name, e
        ))
    })
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
        pyo3::exceptions::PyRuntimeError::new_err(format!(
            "Failed to create Topic '{}': {}. Common causes: invalid topic name \
             (allowed: a-z, 0-9, _, -, ., /), SHM permission denied, \
             or insufficient shared memory space. Run: horus clean --shm",
            endpoint, e
        ))
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    /// read_lock succeeds on a healthy (non-poisoned) lock.
    /// PyResult can be constructed without a Python runtime.
    #[test]
    fn read_lock_succeeds_on_healthy_lock() {
        let lock = RwLock::new(42u32);
        let guard = read_lock(&lock).expect("read_lock should succeed on healthy lock");
        assert_eq!(*guard, 42);
    }

    /// read_lock returns Err on a poisoned lock.
    #[test]
    fn read_lock_returns_error_on_poisoned_lock() {
        let lock = Arc::new(RwLock::new(42u32));
        let lock2 = lock.clone();

        // Poison the lock by panicking while holding a write guard
        let _ = std::thread::spawn(move || {
            let _guard = lock2.write().unwrap();
            panic!("intentional poison");
        })
        .join();

        let result = read_lock(&lock);
        assert!(result.is_err(), "read_lock should fail on poisoned lock");
    }

    /// log_py_callback does not panic on Err (it logs and swallows).
    #[test]
    fn log_py_callback_does_not_panic_on_err() {
        let err_result: PyResult<Py<PyAny>> = Err(PyRuntimeError::new_err("test error"));
        log_py_callback(err_result, "test_method", "test_topic");
    }

    /// Multiple concurrent readers can hold read_lock simultaneously.
    #[test]
    fn read_lock_allows_concurrent_readers() {
        let lock = Arc::new(RwLock::new(99u32));

        let handles: Vec<_> = (0..4)
            .map(|_| {
                let lock = lock.clone();
                std::thread::spawn(move || {
                    let guard = read_lock(&lock).unwrap();
                    assert_eq!(*guard, 99);
                })
            })
            .collect();

        for h in handles {
            h.join().unwrap();
        }
    }
}
