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
use horus_robotics::CmdVel;
use horus_types::{
    Accel, AccelStamped, Clock, DiagnosticReport, DiagnosticStatus, DiagnosticValue, EmergencyStop,
    GenericMessage, Heartbeat, NodeHeartbeat, Point3, Pose2D, Pose3D, PoseStamped,
    PoseWithCovariance, Quaternion, ResourceUsage, SafetyStatus, TimeReference, TransformStamped,
    Twist, TwistWithCovariance, Vector3,
};
use pyo3::exceptions::PyRuntimeError;
use pyo3::prelude::*;
use serde::{de::DeserializeOwned, Serialize};
use std::sync::{Arc, Mutex};

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

/// Acquire the topic's lock, converting a poisoned lock into a PyRuntimeError.
///
/// This was a *read* lock on an `RwLock`, and every recv path plus every
/// pool-backed send path used it. That handed out N simultaneous `&Topic<T>` to
/// N Python threads, while `horus_core` requires exclusive access: every
/// `Topic` method takes `&self` and then mutates unsynchronised `UnsafeCell`
/// cursor state and `Cell` keep-alive state. Two threads calling `recv()` on
/// one `horus.Topic` raced on `local_tail` (duplicated or lost messages, and a
/// data race outright); two calling `send()` on an image topic could release
/// the same pool slot twice. `Topic` is now `!Sync` in horus_core, and the lock
/// here is a `Mutex` so no shared-borrow path can be reintroduced by accident.
#[inline]
fn topic_lock<T>(lock: &Mutex<T>) -> PyResult<std::sync::MutexGuard<'_, T>> {
    lock.lock()
        .map_err(|e| PyRuntimeError::new_err(format!("Topic lock poisoned: {e}")))
}

/// Encode an arbitrary Python object as a `GenericMessage` (MessagePack payload).
///
/// Shared by `send`, `try_send`, and `send_blocking` so an untyped
/// `Topic("name")` reports the *same* error for the same unencodable object no
/// matter which of the three the caller reached for.
fn generic_message_from_py(py: Python, message: &Py<PyAny>) -> PyResult<GenericMessage> {
    let bound = message.bind(py);
    let value: serde_json::Value = pythonize::depythonize(bound).map_err(|e| {
        pyo3::exceptions::PyTypeError::new_err(format!("Failed to convert Python object: {}", e))
    })?;
    let msgpack_bytes = rmp_serde::to_vec(&value).map_err(|e| {
        pyo3::exceptions::PyRuntimeError::new_err(format!(
            "Failed to serialize to MessagePack: {}",
            e
        ))
    })?;
    GenericMessage::new(msgpack_bytes)
        .map_err(|e| pyo3::exceptions::PyValueError::new_err(e.to_string()))
}

/// Validate a Python `timeout_s` argument and convert it to a `Duration`.
///
/// `Duration::from_secs_f64` *panics* on NaN, negatives, and values past
/// `Duration::MAX` — and a panic across the FFI boundary aborts the interpreter
/// rather than raising something a robot's supervisor can catch. A bad timeout
/// is a caller mistake, so it becomes a `ValueError`.
fn parse_timeout(timeout_s: f64) -> PyResult<std::time::Duration> {
    if !timeout_s.is_finite()
        || timeout_s < 0.0
        || timeout_s > std::time::Duration::MAX.as_secs_f64()
    {
        return Err(pyo3::exceptions::PyValueError::new_err(format!(
            "timeout_s must be a finite, non-negative number of seconds (got {timeout_s})"
        )));
    }
    Ok(std::time::Duration::from_secs_f64(timeout_s))
}

/// The exception raised when `send_blocking` runs out of time.
///
/// Routed through `errors::to_py_err` so it arrives as `HorusTimeoutError` —
/// the same type every other blocking HORUS call raises — carrying the topic
/// name and both durations, because "the send timed out" without saying which
/// topic is useless on a robot publishing to thirty of them.
fn send_timeout_err(
    topic: &str,
    elapsed: std::time::Duration,
    deadline: std::time::Duration,
) -> PyErr {
    crate::errors::to_py_err(send_timeout_error(topic, elapsed, deadline))
}

/// The `HorusError` behind [`send_timeout_err`], split out so its wording can be
/// asserted on without a Python interpreter to unwrap a `PyErr` with.
fn send_timeout_error(
    topic: &str,
    elapsed: std::time::Duration,
    deadline: std::time::Duration,
) -> horus_core::error::HorusError {
    horus_core::error::HorusError::Timeout(horus_core::error::TimeoutError {
        resource: format!("send_blocking on topic {topic}"),
        elapsed,
        deadline: Some(deadline),
    })
}

/// The exception raised when a pool-backed topic is asked for a send mode it
/// does not have.
///
/// `Image`/`PointCloud`/`DepthImage`/`Tensor` travel as pool descriptors, and
/// `horus_core` exposes no `send_blocking` for them at all. Saying so — and
/// naming the type and the call that does work — beats a bare `TypeError` from
/// a failed downcast.
fn unsupported_send_mode(call: &str, type_name: &str) -> PyErr {
    pyo3::exceptions::PyNotImplementedError::new_err(unsupported_send_mode_msg(call, type_name))
}

/// The wording of [`unsupported_send_mode`], split out so it can be asserted on
/// without a Python interpreter to unwrap a `PyErr` with.
fn unsupported_send_mode_msg(call: &str, type_name: &str) -> String {
    format!(
        "{call}() is not available for {type_name} topics: pool-backed messages \
         (Image, PointCloud, DepthImage, Tensor) are transported as descriptors \
         and horus_core provides no backpressure-aware send for them. Use send(), \
         which is drop-oldest, and watch stats()['missed_count'] on the subscriber."
    )
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
            $( $rust_ty(Arc<Mutex<Topic<$rust_ty>>>), )*
            Image(Arc<Mutex<Topic<Image>>>),
            PointCloud(Arc<Mutex<Topic<PointCloud>>>),
            DepthImage(Arc<Mutex<Topic<DepthImage>>>),
            Tensor(Arc<Mutex<Topic<horus_core::types::Tensor>>>),
            Generic(Arc<Mutex<Topic<GenericMessage>>>),
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
                        TopicType::$rust_ty(Arc::new(Mutex::new(topic)))
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
                            py.detach(|| { topic_ref.lock().expect("lock").send(val); });
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
                                topic_ref.lock().expect("lock").recv()
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

            /// Try to send a POD message without blocking.
            ///
            /// `Ok(Some(true))` sent, `Ok(Some(false))` refused because the ring
            /// was full, `Ok(None)` not a POD type (caller handles it).
            fn try_send_pod(
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
                            let sent = py.detach(|| {
                                topic_ref.lock().expect("lock").try_send(val).is_ok()
                            });
                            if sent {
                                if let Some(s) = summary {
                                    log_ipc_event(py, node, &self.name, s,
                                        start.elapsed().as_nanos() as u64, "log_pub");
                                }
                            }
                            Ok(Some(sent))
                        }
                    )*
                    _ => Ok(None), // Special types — caller handles
                }
            }

            /// Send a POD message, waiting up to `timeout` for ring space.
            ///
            /// `Ok(Some(true))` sent, `Ok(Some(false))` the ring stayed full for
            /// the whole timeout, `Ok(None)` not a POD type (caller handles it).
            fn send_blocking_pod(
                &self, py: Python, message: &Py<PyAny>, node: &Option<Py<PyAny>>,
                timeout: std::time::Duration, start: std::time::Instant,
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
                            // The whole point of this call is to park until the
                            // consumer drains, so the GIL cannot be held across it:
                            // a 10 ms wait here would stop every other Python thread
                            // in the process for 10 ms.
                            let sent = py.detach(|| {
                                topic_ref.lock().expect("lock").send_blocking(val, timeout).is_ok()
                            });
                            if sent {
                                if let Some(s) = summary {
                                    log_ipc_event(py, node, &self.name, s,
                                        start.elapsed().as_nanos() as u64, "log_pub");
                                }
                            }
                            Ok(Some(sent))
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

        // An `@host` in the endpoint is DISCARDED, so say so.
        //
        // `create_topic`/`create_pool_topic` both split on '@', keep the name and
        // throw the host away, then build an ordinary local SHM topic -- while
        // the constructor's own docstring advertises
        // `"topic@host:port"  - Direct UDP to specific host`. `create_topic`
        // even comments "Check if this is a network endpoint" immediately before
        // doing it, and reports failures as "Failed to create network Topic".
        //
        // So a user who wrote `Topic(CmdVel, endpoint="cmdvel@192.168.1.5:9000")`
        // to drive a second machine got a topic that published to shared memory
        // on THIS one and dropped every cross-machine message in silence -- and
        // `stats()["is_network"]` agreed it was networked, because that flag was
        // derived from the string containing '@' rather than from anything being
        // connected. On a robot that is a command stream that appears configured
        // and goes nowhere.
        //
        // Warned rather than raised, matching the `net=True` path in
        // `horus/__init__.py`, which has the same shape and the same cause: LAN
        // replication is a Rust-only feature and is not wired into these
        // bindings. Raising would break callers who are already (unknowingly)
        // running local-only.
        let host_requested = endpoint.as_ref().and_then(|e| e.split_once('@'));
        if let Some((_, host)) = host_requested {
            let _ = pyo3::PyErr::warn(
                py,
                &py.get_type::<pyo3::exceptions::PyRuntimeWarning>(),
                &std::ffi::CString::new(format!(
                    "Topic endpoint '{}' names host '{}', but the host is IGNORED: this \
                     topic is local shared memory only and cross-machine messages are \
                     silently dropped. LAN replication is currently a Rust-only feature \
                     (build horus with `--features net`). Remove the '@{}' to make the \
                     local-only behaviour explicit.",
                    endpoint.as_deref().unwrap_or(""),
                    host,
                    host
                ))
                .unwrap_or_else(|_| std::ffi::CString::new("topic endpoint host ignored").unwrap()),
                1,
            );
        }
        // NOT `endpoint.contains('@')`. This flag is read back by
        // `is_network_topic()` and `stats()["is_network"]`, and answering true
        // for a topic that is local shared memory is the lie above, reported by
        // the API itself.
        let is_network = false;
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
                    TopicType::Image(Arc::new(Mutex::new(topic)))
                }
                "PointCloud" => {
                    let topic = create_pool_topic::<PointCloud>(&effective_endpoint, cap)?;
                    TopicType::PointCloud(Arc::new(Mutex::new(topic)))
                }
                "DepthImage" => {
                    let topic = create_pool_topic::<DepthImage>(&effective_endpoint, cap)?;
                    TopicType::DepthImage(Arc::new(Mutex::new(topic)))
                }
                "Tensor" | "TensorHandle" => {
                    let topic =
                        create_pool_topic::<horus_core::types::Tensor>(&effective_endpoint, cap)?;
                    TopicType::Tensor(Arc::new(Mutex::new(topic)))
                }
                _ => {
                    let topic = create_topic::<GenericMessage>(&effective_endpoint, cap)?;
                    TopicType::Generic(Arc::new(Mutex::new(topic)))
                }
            }
        };

        Ok(Self {
            topic_type,
            // The effective endpoint, not the type-derived `topic_name`, is the
            // key the shared-memory topic was actually created under above.
            // Storing the type name here made `topic.name` report `imu` for a
            // `Topic(Imu, endpoint="imu.data")` — contradicting its own
            // documentation ("the topic name string") and misattributing every
            // `log_ipc_event` below to a topic that does not exist. The two are
            // identical when no `endpoint=` is passed.
            name: effective_endpoint,
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
                    topic_ref.lock().expect("topic lock poisoned").send(&img);
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
                    topic_ref.lock().expect("topic lock poisoned").send(&pc);
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
                    topic_ref.lock().expect("topic lock poisoned").send(&depth);
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
                let log_msg = format!(
                    "Tensor(shape={:?}, dtype={})",
                    handle.shape(),
                    handle.dtype()
                );
                // The tensor was allocated in a process-local pool (from_numpy uses
                // a scratch pool), but the receiver reads from THIS topic's shared
                // pool. Sending the source descriptor as-is pointed the receiver at
                // a slot in the wrong pool → data_ptr() null on recv. Allocate in
                // the topic's pool and copy the bytes in, so the descriptor refers
                // to the pool the receiver actually reads — the same pattern
                // Topic<Tensor>::send_handle expects and Image already follows.
                let topic_ref = topic.clone();
                let shape = handle.shape().to_vec();
                let dtype = handle.dtype();
                let device = handle.device();
                let src_bytes = handle
                    .data_slice()
                    .map_err(|e| PyRuntimeError::new_err(format!("tensor read failed: {e}")))?;
                let success = py.detach(|| -> PyResult<bool> {
                    let t = topic_ref.lock().expect("topic lock poisoned");
                    let dst = t
                        .alloc_tensor(&shape, dtype, device)
                        .map_err(|e| PyRuntimeError::new_err(format!("pool alloc failed: {e}")))?;
                    {
                        let dst_bytes = dst.data_slice_mut().map_err(|e| {
                            PyRuntimeError::new_err(format!("tensor write failed: {e}"))
                        })?;
                        let n = src_bytes.len().min(dst_bytes.len());
                        dst_bytes[..n].copy_from_slice(&src_bytes[..n]);
                    }
                    t.send_handle(&dst);
                    Ok(true)
                })?;
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
                let msg = generic_message_from_py(py, &message)?;
                use horus::core::LogSummary;
                let log_summary = msg.log_summary();
                let topic_ref = topic.clone();
                let success = py.detach(|| {
                    topic_ref.lock().expect("topic lock poisoned").send(msg);
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

    /// Try to send a message, reporting whether it was actually published.
    ///
    /// `send()` is fire-and-forget: when the ring buffer is full it overwrites
    /// the oldest unread slot and still answers True. `try_send()` does not —
    /// it returns False and leaves the buffer untouched, which is the only way
    /// a Python publisher can find out that a message did not get through.
    ///
    /// Args:
    ///     message: Message object (CmdVel, Pose2D, etc.)
    ///     node: Optional Node for automatic logging with IPC timing
    ///
    /// Returns:
    ///     True if the message was published, False if the buffer was full.
    ///
    /// Raises:
    ///     NotImplementedError: Tensor topics have no non-blocking send path.
    ///
    /// Examples:
    ///     if not topic.try_send(CmdVel(1.5, 0.5)):
    ///         node.log_warning("cmd_vel full - command not delivered")
    #[pyo3(signature = (message, node=None))]
    fn try_send(&self, py: Python, message: Py<PyAny>, node: Option<Py<PyAny>>) -> PyResult<bool> {
        use std::time::Instant;
        let start = Instant::now();

        // Fast path: POD types (generated by pod_topic_types! macro)
        if let Some(sent) = self.try_send_pod(py, &message, &node, start)? {
            return Ok(sent);
        }

        // Special types: pool-backed + generic
        match &self.topic_type {
            TopicType::Image(topic) => {
                let py_img: PyRef<PyImage> = message.extract(py)?;
                let img = py_img.inner().clone();
                let log_msg = format!("Image({}x{})", img.height(), img.width());
                let topic_ref = topic.clone();
                let sent = py.detach(|| {
                    topic_ref
                        .lock()
                        .expect("topic lock poisoned")
                        .try_send(img)
                        .is_ok()
                });
                if sent && node.is_some() {
                    log_ipc_event(
                        py,
                        &node,
                        &self.name,
                        log_msg,
                        start.elapsed().as_nanos() as u64,
                        "log_pub",
                    );
                }
                Ok(sent)
            }
            TopicType::PointCloud(topic) => {
                let py_pc: PyRef<PyPointCloud> = message.extract(py)?;
                let pc = py_pc.inner().clone();
                let log_msg = format!("PointCloud({} pts)", pc.point_count());
                let topic_ref = topic.clone();
                let sent = py.detach(|| {
                    topic_ref
                        .lock()
                        .expect("topic lock poisoned")
                        .try_send(pc)
                        .is_ok()
                });
                if sent && node.is_some() {
                    log_ipc_event(
                        py,
                        &node,
                        &self.name,
                        log_msg,
                        start.elapsed().as_nanos() as u64,
                        "log_pub",
                    );
                }
                Ok(sent)
            }
            TopicType::DepthImage(topic) => {
                let py_depth: PyRef<PyDepthImage> = message.extract(py)?;
                let depth = py_depth.inner().clone();
                let log_msg = format!("DepthImage({}x{})", depth.height(), depth.width());
                let topic_ref = topic.clone();
                let sent = py.detach(|| {
                    topic_ref
                        .lock()
                        .expect("topic lock poisoned")
                        .try_send(depth)
                        .is_ok()
                });
                if sent && node.is_some() {
                    log_ipc_event(
                        py,
                        &node,
                        &self.name,
                        log_msg,
                        start.elapsed().as_nanos() as u64,
                        "log_pub",
                    );
                }
                Ok(sent)
            }
            TopicType::Generic(topic) => {
                let msg = generic_message_from_py(py, &message)?;
                use horus::core::LogSummary;
                let log_summary = msg.log_summary();
                let topic_ref = topic.clone();
                let sent = py.detach(|| {
                    topic_ref
                        .lock()
                        .expect("topic lock poisoned")
                        .try_send(msg)
                        .is_ok()
                });
                if sent && node.is_some() {
                    log_ipc_event(
                        py,
                        &node,
                        &self.name,
                        log_summary,
                        start.elapsed().as_nanos() as u64,
                        "log_pub",
                    );
                }
                Ok(sent)
            }
            TopicType::Tensor(_) => Err(unsupported_send_mode("try_send", "Tensor")),
            _ => unreachable!("try_send_pod handles all POD types"),
        }
    }

    /// Send a message, waiting up to `timeout_s` for room in the ring buffer.
    ///
    /// For topics where losing a message is not acceptable — emergency stop,
    /// motor setpoints, a goal handoff. Applies backpressure instead of
    /// overwriting: it parks until the subscriber drains a slot, and raises if
    /// it never does. The GIL is released while waiting, so other Python
    /// threads keep running.
    ///
    /// It does hold THIS Topic object's internal lock for the whole wait, so a
    /// thread sharing the same Topic instance cannot recv() until the wait ends
    /// — and on a full ring that means it cannot be the one to free the slot.
    /// Give the consumer its own Topic handle on the same name.
    ///
    /// Args:
    ///     message: Message object (CmdVel, Pose2D, etc.)
    ///     timeout_s: Seconds to wait for space. 0.0 means "try once".
    ///     node: Optional Node for automatic logging with IPC timing
    ///
    /// Raises:
    ///     HorusTimeoutError: the buffer stayed full for the whole timeout;
    ///         the message was NOT published.
    ///     ValueError: timeout_s is negative, NaN, or infinite.
    ///     NotImplementedError: pool-backed topics (Image, PointCloud,
    ///         DepthImage, Tensor) have no blocking send path.
    ///
    /// Examples:
    ///     try:
    ///         estop.send_blocking(EmergencyStop(True), 0.05)
    ///     except horus.HorusTimeoutError:
    ///         node.log_error("e-stop not delivered")
    #[pyo3(signature = (message, timeout_s, node=None))]
    fn send_blocking(
        &self,
        py: Python,
        message: Py<PyAny>,
        timeout_s: f64,
        node: Option<Py<PyAny>>,
    ) -> PyResult<()> {
        use std::time::Instant;
        let start = Instant::now();
        let timeout = parse_timeout(timeout_s)?;

        // Fast path: POD types (generated by pod_topic_types! macro)
        if let Some(sent) = self.send_blocking_pod(py, &message, &node, timeout, start)? {
            return if sent {
                Ok(())
            } else {
                Err(send_timeout_err(&self.name, start.elapsed(), timeout))
            };
        }

        // Special types: generic is supported; pool-backed transports are not.
        match &self.topic_type {
            TopicType::Generic(topic) => {
                let msg = generic_message_from_py(py, &message)?;
                use horus::core::LogSummary;
                let log_summary = msg.log_summary();
                let topic_ref = topic.clone();
                let sent = py.detach(|| {
                    topic_ref
                        .lock()
                        .expect("topic lock poisoned")
                        .send_blocking(msg, timeout)
                        .is_ok()
                });
                if !sent {
                    return Err(send_timeout_err(&self.name, start.elapsed(), timeout));
                }
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
                Ok(())
            }
            TopicType::Image(_) => Err(unsupported_send_mode("send_blocking", "Image")),
            TopicType::PointCloud(_) => Err(unsupported_send_mode("send_blocking", "PointCloud")),
            TopicType::DepthImage(_) => Err(unsupported_send_mode("send_blocking", "DepthImage")),
            TopicType::Tensor(_) => Err(unsupported_send_mode("send_blocking", "Tensor")),
            _ => unreachable!("send_blocking_pod handles all POD types"),
        }
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
                let msg_opt = py.detach(|| topic_ref.lock().expect("topic lock poisoned").recv());
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
                let msg_opt = py.detach(|| topic_ref.lock().expect("topic lock poisoned").recv());
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
                let msg_opt = py.detach(|| topic_ref.lock().expect("topic lock poisoned").recv());
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
                // recv_handle wraps TensorHandle::from_owned: the sender already
                // bumped the refcount (via send_handle) and from_owned validates
                // the descriptor's pool_id matches this topic's pool — guarding
                // against a cross-pool descriptor instead of silently handing back
                // a null data pointer.
                let msg_opt =
                    py.detach(|| topic_ref.lock().expect("topic lock poisoned").recv_handle());
                if let Some(handle) = msg_opt {
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
                                view_keepalive: None,
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
                let msg_opt = py.detach(|| topic_ref.lock().expect("topic lock poisoned").recv());
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
            TopicType::CmdVel(t) => topic_lock(t)
                .map(|g| g.backend_name().to_string())
                .unwrap_or_default(),
            TopicType::Pose2D(t) => topic_lock(t)
                .map(|g| g.backend_name().to_string())
                .unwrap_or_default(),
            TopicType::Pose3D(t) => topic_lock(t)
                .map(|g| g.backend_name().to_string())
                .unwrap_or_default(),
            TopicType::Imu(t) => topic_lock(t)
                .map(|g| g.backend_name().to_string())
                .unwrap_or_default(),
            TopicType::Odometry(t) => topic_lock(t)
                .map(|g| g.backend_name().to_string())
                .unwrap_or_default(),
            TopicType::LaserScan(t) => topic_lock(t)
                .map(|g| g.backend_name().to_string())
                .unwrap_or_default(),
            TopicType::JointState(t) => topic_lock(t)
                .map(|g| g.backend_name().to_string())
                .unwrap_or_default(),
            TopicType::Clock(t) => topic_lock(t)
                .map(|g| g.backend_name().to_string())
                .unwrap_or_default(),
            TopicType::TimeReference(t) => topic_lock(t)
                .map(|g| g.backend_name().to_string())
                .unwrap_or_default(),
            TopicType::Image(t) => topic_lock(t)
                .map(|g| g.backend_name().to_string())
                .unwrap_or_default(),
            TopicType::PointCloud(t) => topic_lock(t)
                .map(|g| g.backend_name().to_string())
                .unwrap_or_default(),
            TopicType::DepthImage(t) => topic_lock(t)
                .map(|g| g.backend_name().to_string())
                .unwrap_or_default(),
            // Geometry types
            TopicType::Twist(t) => topic_lock(t)
                .map(|g| g.backend_name().to_string())
                .unwrap_or_default(),
            TopicType::Vector3(t) => topic_lock(t)
                .map(|g| g.backend_name().to_string())
                .unwrap_or_default(),
            TopicType::Point3(t) => topic_lock(t)
                .map(|g| g.backend_name().to_string())
                .unwrap_or_default(),
            TopicType::Quaternion(t) => topic_lock(t)
                .map(|g| g.backend_name().to_string())
                .unwrap_or_default(),
            TopicType::TransformStamped(t) => topic_lock(t)
                .map(|g| g.backend_name().to_string())
                .unwrap_or_default(),
            TopicType::PoseStamped(t) => topic_lock(t)
                .map(|g| g.backend_name().to_string())
                .unwrap_or_default(),
            TopicType::PoseWithCovariance(t) => topic_lock(t)
                .map(|g| g.backend_name().to_string())
                .unwrap_or_default(),
            TopicType::TwistWithCovariance(t) => topic_lock(t)
                .map(|g| g.backend_name().to_string())
                .unwrap_or_default(),
            TopicType::Accel(t) => topic_lock(t)
                .map(|g| g.backend_name().to_string())
                .unwrap_or_default(),
            TopicType::AccelStamped(t) => topic_lock(t)
                .map(|g| g.backend_name().to_string())
                .unwrap_or_default(),
            // Control types
            TopicType::MotorCommand(t) => topic_lock(t)
                .map(|g| g.backend_name().to_string())
                .unwrap_or_default(),
            TopicType::ServoCommand(t) => topic_lock(t)
                .map(|g| g.backend_name().to_string())
                .unwrap_or_default(),
            TopicType::DifferentialDriveCommand(t) => topic_lock(t)
                .map(|g| g.backend_name().to_string())
                .unwrap_or_default(),
            TopicType::PidConfig(t) => topic_lock(t)
                .map(|g| g.backend_name().to_string())
                .unwrap_or_default(),
            TopicType::TrajectoryPoint(t) => topic_lock(t)
                .map(|g| g.backend_name().to_string())
                .unwrap_or_default(),
            TopicType::JointCommand(t) => topic_lock(t)
                .map(|g| g.backend_name().to_string())
                .unwrap_or_default(),
            // Sensor types
            TopicType::RangeSensor(t) => topic_lock(t)
                .map(|g| g.backend_name().to_string())
                .unwrap_or_default(),
            TopicType::BatteryState(t) => topic_lock(t)
                .map(|g| g.backend_name().to_string())
                .unwrap_or_default(),
            TopicType::NavSatFix(t) => topic_lock(t)
                .map(|g| g.backend_name().to_string())
                .unwrap_or_default(),
            TopicType::MagneticField(t) => topic_lock(t)
                .map(|g| g.backend_name().to_string())
                .unwrap_or_default(),
            TopicType::Temperature(t) => topic_lock(t)
                .map(|g| g.backend_name().to_string())
                .unwrap_or_default(),
            TopicType::FluidPressure(t) => topic_lock(t)
                .map(|g| g.backend_name().to_string())
                .unwrap_or_default(),
            TopicType::Illuminance(t) => topic_lock(t)
                .map(|g| g.backend_name().to_string())
                .unwrap_or_default(),
            // Diagnostics types
            TopicType::Heartbeat(t) => topic_lock(t)
                .map(|g| g.backend_name().to_string())
                .unwrap_or_default(),
            TopicType::DiagnosticStatus(t) => topic_lock(t)
                .map(|g| g.backend_name().to_string())
                .unwrap_or_default(),
            TopicType::EmergencyStop(t) => topic_lock(t)
                .map(|g| g.backend_name().to_string())
                .unwrap_or_default(),
            TopicType::ResourceUsage(t) => topic_lock(t)
                .map(|g| g.backend_name().to_string())
                .unwrap_or_default(),
            // Force types
            TopicType::WrenchStamped(t) => topic_lock(t)
                .map(|g| g.backend_name().to_string())
                .unwrap_or_default(),
            TopicType::ForceCommand(t) => topic_lock(t)
                .map(|g| g.backend_name().to_string())
                .unwrap_or_default(),
            TopicType::ContactInfo(t) => topic_lock(t)
                .map(|g| g.backend_name().to_string())
                .unwrap_or_default(),
            // Navigation types
            TopicType::NavGoal(t) => topic_lock(t)
                .map(|g| g.backend_name().to_string())
                .unwrap_or_default(),
            TopicType::GoalResult(t) => topic_lock(t)
                .map(|g| g.backend_name().to_string())
                .unwrap_or_default(),
            TopicType::PathPlan(t) => topic_lock(t)
                .map(|g| g.backend_name().to_string())
                .unwrap_or_default(),
            // Input types
            TopicType::JoystickInput(t) => topic_lock(t)
                .map(|g| g.backend_name().to_string())
                .unwrap_or_default(),
            TopicType::KeyboardInput(t) => topic_lock(t)
                .map(|g| g.backend_name().to_string())
                .unwrap_or_default(),
            // Detection/Perception types
            TopicType::BoundingBox2D(t) => topic_lock(t)
                .map(|g| g.backend_name().to_string())
                .unwrap_or_default(),
            TopicType::BoundingBox3D(t) => topic_lock(t)
                .map(|g| g.backend_name().to_string())
                .unwrap_or_default(),
            TopicType::Detection(t) => topic_lock(t)
                .map(|g| g.backend_name().to_string())
                .unwrap_or_default(),
            TopicType::Detection3D(t) => topic_lock(t)
                .map(|g| g.backend_name().to_string())
                .unwrap_or_default(),
            TopicType::SegmentationMask(t) => topic_lock(t)
                .map(|g| g.backend_name().to_string())
                .unwrap_or_default(),
            // Tracking types
            TopicType::TrackedObject(t) => topic_lock(t)
                .map(|g| g.backend_name().to_string())
                .unwrap_or_default(),
            TopicType::TrackingHeader(t) => topic_lock(t)
                .map(|g| g.backend_name().to_string())
                .unwrap_or_default(),
            // Landmark types
            TopicType::Landmark(t) => topic_lock(t)
                .map(|g| g.backend_name().to_string())
                .unwrap_or_default(),
            TopicType::Landmark3D(t) => topic_lock(t)
                .map(|g| g.backend_name().to_string())
                .unwrap_or_default(),
            TopicType::LandmarkArray(t) => topic_lock(t)
                .map(|g| g.backend_name().to_string())
                .unwrap_or_default(),
            // Perception helper types
            TopicType::PointField(t) => topic_lock(t)
                .map(|g| g.backend_name().to_string())
                .unwrap_or_default(),
            TopicType::PlaneDetection(t) => topic_lock(t)
                .map(|g| g.backend_name().to_string())
                .unwrap_or_default(),
            TopicType::PlaneArray(t) => topic_lock(t)
                .map(|g| g.backend_name().to_string())
                .unwrap_or_default(),
            // Vision types
            TopicType::CompressedImage(t) => topic_lock(t)
                .map(|g| g.backend_name().to_string())
                .unwrap_or_default(),
            TopicType::CameraInfo(t) => topic_lock(t)
                .map(|g| g.backend_name().to_string())
                .unwrap_or_default(),
            TopicType::RegionOfInterest(t) => topic_lock(t)
                .map(|g| g.backend_name().to_string())
                .unwrap_or_default(),
            TopicType::StereoInfo(t) => topic_lock(t)
                .map(|g| g.backend_name().to_string())
                .unwrap_or_default(),
            // Force types (additional)
            TopicType::ImpedanceParameters(t) => topic_lock(t)
                .map(|g| g.backend_name().to_string())
                .unwrap_or_default(),
            TopicType::HapticFeedback(t) => topic_lock(t)
                .map(|g| g.backend_name().to_string())
                .unwrap_or_default(),
            TopicType::TactileArray(t) => topic_lock(t)
                .map(|g| g.backend_name().to_string())
                .unwrap_or_default(),
            // Diagnostics types (additional)
            TopicType::DiagnosticValue(t) => topic_lock(t)
                .map(|g| g.backend_name().to_string())
                .unwrap_or_default(),
            TopicType::DiagnosticReport(t) => topic_lock(t)
                .map(|g| g.backend_name().to_string())
                .unwrap_or_default(),
            TopicType::NodeHeartbeat(t) => topic_lock(t)
                .map(|g| g.backend_name().to_string())
                .unwrap_or_default(),
            TopicType::SafetyStatus(t) => topic_lock(t)
                .map(|g| g.backend_name().to_string())
                .unwrap_or_default(),
            // Navigation types (additional)
            TopicType::Waypoint(t) => topic_lock(t)
                .map(|g| g.backend_name().to_string())
                .unwrap_or_default(),
            TopicType::NavPath(t) => topic_lock(t)
                .map(|g| g.backend_name().to_string())
                .unwrap_or_default(),
            TopicType::VelocityObstacle(t) => topic_lock(t)
                .map(|g| g.backend_name().to_string())
                .unwrap_or_default(),
            TopicType::VelocityObstacles(t) => topic_lock(t)
                .map(|g| g.backend_name().to_string())
                .unwrap_or_default(),
            TopicType::OccupancyGrid(t) => topic_lock(t)
                .map(|g| g.backend_name().to_string())
                .unwrap_or_default(),
            TopicType::CostMap(t) => topic_lock(t)
                .map(|g| g.backend_name().to_string())
                .unwrap_or_default(),
            TopicType::AudioFrame(t) => topic_lock(t)
                .map(|g| g.backend_name().to_string())
                .unwrap_or_default(),
            TopicType::Tensor(t) => topic_lock(t)
                .map(|g| g.backend_name().to_string())
                .unwrap_or_default(),
            TopicType::Generic(t) => topic_lock(t)
                .map(|g| g.backend_name().to_string())
                .unwrap_or_default(),
        }
    }

    /// Get topic statistics as a dictionary
    ///
    /// `send_failures` counts sends this publisher could not place;
    /// `missed_count` counts messages this subscriber lost to a publisher that
    /// outran it. Those are different failures at opposite ends of the link,
    /// and nothing else in this dict reports the second one.
    ///
    /// Returns:
    ///     Dictionary with keys: messages_sent, messages_received, send_failures,
    ///     recv_failures, missed_count, is_network, backend.
    fn stats(&self) -> PyResult<pyo3::Py<pyo3::types::PyDict>> {
        Python::attach(|py| {
            let dict = pyo3::types::PyDict::new(py);

            let metrics = match &self.topic_type {
                TopicType::CmdVel(t) => topic_lock(t)?.metrics(),
                TopicType::Pose2D(t) => topic_lock(t)?.metrics(),
                TopicType::Pose3D(t) => topic_lock(t)?.metrics(),
                TopicType::Imu(t) => topic_lock(t)?.metrics(),
                TopicType::Odometry(t) => topic_lock(t)?.metrics(),
                TopicType::LaserScan(t) => topic_lock(t)?.metrics(),
                TopicType::JointState(t) => topic_lock(t)?.metrics(),
                TopicType::Clock(t) => topic_lock(t)?.metrics(),
                TopicType::TimeReference(t) => topic_lock(t)?.metrics(),
                TopicType::Image(t) => topic_lock(t)?.metrics(),
                TopicType::PointCloud(t) => topic_lock(t)?.metrics(),
                TopicType::DepthImage(t) => topic_lock(t)?.metrics(),
                // Geometry types
                TopicType::Twist(t) => topic_lock(t)?.metrics(),
                TopicType::Vector3(t) => topic_lock(t)?.metrics(),
                TopicType::Point3(t) => topic_lock(t)?.metrics(),
                TopicType::Quaternion(t) => topic_lock(t)?.metrics(),
                TopicType::TransformStamped(t) => topic_lock(t)?.metrics(),
                TopicType::PoseStamped(t) => topic_lock(t)?.metrics(),
                TopicType::PoseWithCovariance(t) => topic_lock(t)?.metrics(),
                TopicType::TwistWithCovariance(t) => topic_lock(t)?.metrics(),
                TopicType::Accel(t) => topic_lock(t)?.metrics(),
                TopicType::AccelStamped(t) => topic_lock(t)?.metrics(),
                // Control types
                TopicType::MotorCommand(t) => topic_lock(t)?.metrics(),
                TopicType::ServoCommand(t) => topic_lock(t)?.metrics(),
                TopicType::DifferentialDriveCommand(t) => topic_lock(t)?.metrics(),
                TopicType::PidConfig(t) => topic_lock(t)?.metrics(),
                TopicType::TrajectoryPoint(t) => topic_lock(t)?.metrics(),
                TopicType::JointCommand(t) => topic_lock(t)?.metrics(),
                // Sensor types
                TopicType::RangeSensor(t) => topic_lock(t)?.metrics(),
                TopicType::BatteryState(t) => topic_lock(t)?.metrics(),
                TopicType::NavSatFix(t) => topic_lock(t)?.metrics(),
                TopicType::MagneticField(t) => topic_lock(t)?.metrics(),
                TopicType::Temperature(t) => topic_lock(t)?.metrics(),
                TopicType::FluidPressure(t) => topic_lock(t)?.metrics(),
                TopicType::Illuminance(t) => topic_lock(t)?.metrics(),
                // Diagnostics types
                TopicType::Heartbeat(t) => topic_lock(t)?.metrics(),
                TopicType::DiagnosticStatus(t) => topic_lock(t)?.metrics(),
                TopicType::EmergencyStop(t) => topic_lock(t)?.metrics(),
                TopicType::ResourceUsage(t) => topic_lock(t)?.metrics(),
                // Force types
                TopicType::WrenchStamped(t) => topic_lock(t)?.metrics(),
                TopicType::ForceCommand(t) => topic_lock(t)?.metrics(),
                TopicType::ContactInfo(t) => topic_lock(t)?.metrics(),
                // Navigation types
                TopicType::NavGoal(t) => topic_lock(t)?.metrics(),
                TopicType::GoalResult(t) => topic_lock(t)?.metrics(),
                TopicType::PathPlan(t) => topic_lock(t)?.metrics(),
                // Input types
                TopicType::JoystickInput(t) => topic_lock(t)?.metrics(),
                TopicType::KeyboardInput(t) => topic_lock(t)?.metrics(),
                // Detection/Perception types
                TopicType::BoundingBox2D(t) => topic_lock(t)?.metrics(),
                TopicType::BoundingBox3D(t) => topic_lock(t)?.metrics(),
                TopicType::Detection(t) => topic_lock(t)?.metrics(),
                TopicType::Detection3D(t) => topic_lock(t)?.metrics(),
                TopicType::SegmentationMask(t) => topic_lock(t)?.metrics(),
                // Tracking types
                TopicType::TrackedObject(t) => topic_lock(t)?.metrics(),
                TopicType::TrackingHeader(t) => topic_lock(t)?.metrics(),
                // Landmark types
                TopicType::Landmark(t) => topic_lock(t)?.metrics(),
                TopicType::Landmark3D(t) => topic_lock(t)?.metrics(),
                TopicType::LandmarkArray(t) => topic_lock(t)?.metrics(),
                // Perception helper types
                TopicType::PointField(t) => topic_lock(t)?.metrics(),
                TopicType::PlaneDetection(t) => topic_lock(t)?.metrics(),
                TopicType::PlaneArray(t) => topic_lock(t)?.metrics(),
                // Vision types
                TopicType::CompressedImage(t) => topic_lock(t)?.metrics(),
                TopicType::CameraInfo(t) => topic_lock(t)?.metrics(),
                TopicType::RegionOfInterest(t) => topic_lock(t)?.metrics(),
                TopicType::StereoInfo(t) => topic_lock(t)?.metrics(),
                // Force types (additional)
                TopicType::ImpedanceParameters(t) => topic_lock(t)?.metrics(),
                TopicType::HapticFeedback(t) => topic_lock(t)?.metrics(),
                TopicType::TactileArray(t) => topic_lock(t)?.metrics(),
                // Diagnostics types (additional)
                TopicType::DiagnosticValue(t) => topic_lock(t)?.metrics(),
                TopicType::DiagnosticReport(t) => topic_lock(t)?.metrics(),
                TopicType::NodeHeartbeat(t) => topic_lock(t)?.metrics(),
                TopicType::SafetyStatus(t) => topic_lock(t)?.metrics(),
                // Navigation types (additional)
                TopicType::Waypoint(t) => topic_lock(t)?.metrics(),
                TopicType::NavPath(t) => topic_lock(t)?.metrics(),
                TopicType::VelocityObstacle(t) => topic_lock(t)?.metrics(),
                TopicType::VelocityObstacles(t) => topic_lock(t)?.metrics(),
                TopicType::OccupancyGrid(t) => topic_lock(t)?.metrics(),
                TopicType::CostMap(t) => topic_lock(t)?.metrics(),
                TopicType::AudioFrame(t) => topic_lock(t)?.metrics(),
                TopicType::Tensor(t) => topic_lock(t)?.metrics(),
                TopicType::Generic(t) => topic_lock(t)?.metrics(),
            };

            dict.set_item("messages_sent", metrics.messages_sent())?;
            dict.set_item("messages_received", metrics.messages_received())?;
            dict.set_item("send_failures", metrics.send_failures())?;
            dict.set_item("recv_failures", metrics.recv_failures())?;
            // Deliberately NOT folded into recv_failures: a recv failure is a
            // read this subscriber attempted and did not get, while a miss is a
            // message it was never offered. Summing them would hide the one the
            // subscriber cannot otherwise detect.
            dict.set_item("missed_count", self.missed_count()?)?;
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
            topic_lock(t).map(|g| g.pending_count()).unwrap_or(0)
        )
    }

    /// Messages this subscriber never saw because the publisher lapped it.
    ///
    /// The ring buffer is drop-oldest. When a publisher outruns this
    /// subscriber it overwrites unread slots, and the next recv() quietly
    /// resumes further along the ring — every message it hands back is
    /// well-formed and recent, so nothing about the data says a gap happened.
    /// This counter is the only record that one did: a 30 Hz consumer of a
    /// 500 Hz stream reads valid samples forever while dropping 15 of every 16,
    /// and without this it looks exactly like a consumer dropping none.
    ///
    /// Counts THIS handle's losses only — it is per-subscriber local state, not
    /// a topic-wide figure — and is monotonic for the life of the handle.
    ///
    /// Returns:
    ///     Number of missed messages (int)
    ///
    /// Examples:
    ///     before = topic.missed_count()
    ///     msg = topic.recv()
    ///     if topic.missed_count() > before:
    ///         node.log_warning("state stream lapped us - sample is not continuous")
    fn missed_count(&self) -> PyResult<u64> {
        // Raises rather than reporting 0 on a poisoned lock. This whole method
        // exists so a Python caller can tell "I lost samples" from "I did not",
        // and unwrap_or(0) spends exactly that distinction to avoid an error
        // path -- the failure would arrive disguised as the good news.
        topic_dispatch!(&self.topic_type, t, Ok(topic_lock(t)?.missed_count()))
    }

    /// Messages this publisher could not place and gave up on.
    ///
    /// The publisher-side counterpart of `missed_count`. `send()` retries
    /// briefly and then drops rather than blocking; each drop increments this.
    /// A rising value means the ring is saturated and downstream is too slow.
    ///
    /// The count is per-handle and process-local.
    ///
    /// Returns:
    ///     Messages dropped by this publisher (u64)
    ///
    /// Raises:
    ///     RuntimeError: the topic lock is poisoned, so the count is unknown.
    ///     It is not reported as 0 -- that is a real reading.
    fn dropped_count(&self) -> PyResult<u64> {
        // Raises rather than reporting 0; see `missed_count`.
        topic_dispatch!(&self.topic_type, t, Ok(topic_lock(t)?.dropped_count()))
    }

    /// Number of active publishers on this topic.
    ///
    /// Returns:
    ///     Publisher count (u32)
    fn pub_count(&self) -> u32 {
        topic_dispatch!(
            &self.topic_type,
            t,
            topic_lock(t).map(|g| g.pub_count()).unwrap_or(0)
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
            topic_lock(t).map(|g| g.sub_count()).unwrap_or(0)
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
                let msg_opt = $py.detach(|| tr.lock().expect("topic lock poisoned").read_latest());
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

    /// topic_lock succeeds on a healthy (non-poisoned) lock.
    /// PyResult can be constructed without a Python runtime.
    #[test]
    fn topic_lock_succeeds_on_healthy_lock() {
        let lock = Mutex::new(42u32);
        let guard = topic_lock(&lock).expect("topic_lock should succeed on healthy lock");
        assert_eq!(*guard, 42);
    }

    /// topic_lock returns Err on a poisoned lock.
    #[test]
    fn topic_lock_returns_error_on_poisoned_lock() {
        let lock = Arc::new(Mutex::new(42u32));
        let lock2 = lock.clone();

        // Poison the lock by panicking while holding the guard
        let _ = std::thread::spawn(move || {
            let _guard = lock2.lock().unwrap();
            panic!("intentional poison");
        })
        .join();

        let result = topic_lock(&lock);
        assert!(result.is_err(), "topic_lock should fail on poisoned lock");
    }

    /// `send_blocking(msg, -1.0)` must raise, not abort the interpreter.
    ///
    /// `Duration::from_secs_f64` panics on a negative, and a panic unwinding
    /// through the pyo3 boundary takes the whole process with it — a robot loses
    /// its controller because someone typo'd a timeout.
    #[test]
    fn parse_timeout_rejects_negative() {
        assert!(
            parse_timeout(-1.0).is_err(),
            "a negative timeout must be an error, not a panic"
        );
    }

    /// NaN and infinity reach `from_secs_f64` from Python as plain floats.
    #[test]
    fn parse_timeout_rejects_nan_and_infinity() {
        assert!(
            parse_timeout(f64::NAN).is_err(),
            "NaN timeout must be rejected"
        );
        assert!(
            parse_timeout(f64::INFINITY).is_err(),
            "infinite timeout must be rejected"
        );
        assert!(
            parse_timeout(f64::NEG_INFINITY).is_err(),
            "negative-infinite timeout must be rejected"
        );
    }

    /// A timeout past `Duration::MAX` also panics `from_secs_f64`.
    #[test]
    fn parse_timeout_rejects_overflow() {
        let too_big = std::time::Duration::MAX.as_secs_f64() * 2.0;
        assert!(
            parse_timeout(too_big).is_err(),
            "an out-of-range timeout must be rejected"
        );
    }

    /// Ordinary timeouts convert, including the "try once" zero.
    #[test]
    fn parse_timeout_accepts_valid_seconds() {
        assert_eq!(parse_timeout(0.0).unwrap(), std::time::Duration::ZERO);
        assert_eq!(
            parse_timeout(0.05).unwrap(),
            std::time::Duration::from_millis(50)
        );
        assert_eq!(
            parse_timeout(2.0).unwrap(),
            std::time::Duration::from_secs(2)
        );
    }

    /// The timeout error must name the topic and route to HorusTimeoutError.
    ///
    /// "send timed out" with no topic name is not actionable on a robot
    /// publishing to thirty of them.
    #[test]
    fn send_timeout_error_names_the_topic_and_the_deadline() {
        let err = send_timeout_error(
            "cmd_vel",
            std::time::Duration::from_millis(52),
            std::time::Duration::from_millis(50),
        );
        let msg = err.to_string();
        assert!(
            msg.contains("cmd_vel"),
            "timeout message must name the topic: {msg}"
        );
        assert!(
            msg.contains("send_blocking"),
            "timeout message must name the call: {msg}"
        );
        assert!(
            msg.contains("50ms"),
            "timeout message must carry the deadline: {msg}"
        );
        assert!(
            matches!(err, horus_core::error::HorusError::Timeout(_)),
            "must be the Timeout variant so to_py_err maps it to HorusTimeoutError"
        );
    }

    /// The pool-backed refusal must say which call and which type, and point at
    /// the counter that makes the loss visible instead.
    #[test]
    fn unsupported_send_mode_msg_is_actionable() {
        let msg = unsupported_send_mode_msg("send_blocking", "Image");
        assert!(msg.contains("send_blocking()"), "must name the call: {msg}");
        assert!(msg.contains("Image"), "must name the message type: {msg}");
        assert!(
            msg.contains("missed_count"),
            "must point at the visible alternative: {msg}"
        );
    }

    /// log_py_callback does not panic on Err (it logs and swallows).
    #[test]
    fn log_py_callback_does_not_panic_on_err() {
        let err_result: PyResult<Py<PyAny>> = Err(PyRuntimeError::new_err("test error"));
        log_py_callback(err_result, "test_method", "test_topic");
    }

    /// INVERTED: this test used to assert that four threads could hold the
    /// topic guard *simultaneously* — which is precisely the bug. A `Topic`
    /// method mutates unsynchronised local state through `&self`, so the guard
    /// must be exclusive. Four threads incrementing under it must produce
    /// exactly four increments; with the old shared read guard the
    /// read-modify-write could lose updates (and was UB besides).
    #[test]
    fn topic_lock_is_exclusive_not_shared() {
        let lock = Arc::new(Mutex::new(0u32));

        let handles: Vec<_> = (0..4)
            .map(|_| {
                let lock = lock.clone();
                std::thread::spawn(move || {
                    for _ in 0..1000 {
                        let mut guard = topic_lock(&lock).unwrap();
                        *guard += 1;
                    }
                })
            })
            .collect();

        for h in handles {
            h.join().unwrap();
        }
        assert_eq!(
            *topic_lock(&lock).unwrap(),
            4000,
            "the topic guard must serialise all access, not just writers"
        );
    }
}
