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
use horus_library::messages::clock::{Clock, TimeReference};
use horus_library::messages::cmd_vel::CmdVel;
use horus_library::messages::control::{
    DifferentialDriveCommand, JointCommand, MotorCommand, PidConfig, PwmCommand, ServoCommand,
    StepperCommand, TrajectoryPoint,
};
use horus_library::messages::detection::{BoundingBox2D, BoundingBox3D, Detection, Detection3D};
use horus_library::messages::diagnostics::{
    DiagnosticReport, DiagnosticStatus, DiagnosticValue, EmergencyStop, Heartbeat, NodeHeartbeat,
    ResourceUsage, SafetyStatus,
};
use horus_library::messages::force::{
    ContactInfo, ForceCommand, HapticFeedback, ImpedanceParameters, TactileArray, WrenchStamped,
};
use horus_library::messages::geometry::{
    Accel, AccelStamped, Point3, Pose2D, Pose3D, PoseStamped, PoseWithCovariance, Quaternion,
    TransformStamped, Twist, TwistWithCovariance, Vector3,
};
use horus_library::messages::joystick_msg::JoystickInput;
use horus_library::messages::keyboard_input_msg::KeyboardInput;
use horus_library::messages::landmark::{Landmark, Landmark3D, LandmarkArray};
use horus_library::messages::ml::{
    ChatMessage, Classification, DeploymentConfig, FeatureVector, InferenceMetrics, LLMRequest,
    LLMResponse, MlTrajectoryPoint, ModelInfo, Predictions, TensorData, TrainingMetrics,
};
use horus_library::messages::navigation::{
    CostMap, GoalResult, NavGoal, NavPath, OccupancyGrid, PathPlan, VelocityObstacle,
    VelocityObstacles, Waypoint,
};
use horus_library::messages::perception::{PlaneArray, PlaneDetection, PointField};
use horus_library::messages::segmentation::SegmentationMask;
use horus_library::messages::sensor::{
    BatteryState, FluidPressure, Illuminance, Imu, JointState, LaserScan, MagneticField, NavSatFix,
    Odometry, RangeSensor, Temperature,
};
use horus_library::messages::tracking::{TrackedObject, TrackingHeader};
use horus_library::messages::vision::{CameraInfo, CompressedImage, RegionOfInterest, StereoInfo};
use horus_library::messages::GenericMessage;
use pyo3::exceptions::PyRuntimeError;
use pyo3::prelude::*;
use serde::{de::DeserializeOwned, Serialize};
use std::sync::{Arc, RwLock};

use crate::depth_image::PyDepthImage;
use crate::image::PyImage;
use crate::messages::{
    PyAccel, PyAccelStamped, PyBatteryState, PyBoundingBox2DMsg, PyBoundingBox3D, PyCameraInfo,
    PyChatMessage, PyClassification, PyClock, PyCmdVel, PyCompressedImage, PyContactInfo,
    PyCostMap, PyDeploymentConfig, PyDetection3D, PyDetectionMsg, PyDiagnosticReport,
    PyDiagnosticStatus, PyDiagnosticValue, PyDifferentialDriveCommand, PyEmergencyStop,
    PyFeatureVector, PyFluidPressure, PyForceCommand, PyGoalResult, PyHapticFeedback, PyHeartbeat,
    PyIlluminance, PyImpedanceParameters, PyImu, PyInferenceMetrics, PyJointCommand, PyJointState,
    PyJoystickInput, PyKeyboardInput, PyLLMRequest, PyLLMResponse, PyLandmark3D, PyLandmarkArray,
    PyLandmarkMsg, PyLaserScan, PyMagneticField, PyMlTrajectoryPoint, PyModelInfo, PyMotorCommand,
    PyNavGoal, PyNavPath, PyNavSatFix, PyNodeHeartbeat, PyOccupancyGrid, PyOdometry, PyPathPlan,
    PyPidConfig, PyPlaneArray, PyPlaneDetection, PyPoint3, PyPointField, PyPose2D, PyPose3D,
    PyPoseStamped, PyPoseWithCovariance, PyPredictions, PyPwmCommand, PyQuaternion, PyRangeSensor,
    PyRegionOfInterest, PyResourceUsage, PySafetyStatus, PySegmentationMask, PyServoCommand,
    PyStepperCommand, PyStereoInfo, PyTactileArray, PyTemperature, PyTensorData, PyTimeReference,
    PyTrackedObjectMsg, PyTrackingHeader, PyTrainingMetrics, PyTrajectoryPoint, PyTransformStamped,
    PyTwist, PyTwistWithCovariance, PyVector3, PyVelocityObstacle, PyVelocityObstacles, PyWaypoint,
    PyWrenchStamped,
};
use crate::pointcloud::PyPointCloud;

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
// TopicType enum - tracks which Rust type the Topic wraps
// ============================================================================

enum TopicType {
    CmdVel(Arc<RwLock<Topic<CmdVel>>>),
    Pose2D(Arc<RwLock<Topic<Pose2D>>>),
    Pose3D(Arc<RwLock<Topic<Pose3D>>>),
    Imu(Arc<RwLock<Topic<Imu>>>),
    Odometry(Arc<RwLock<Topic<Odometry>>>),
    LaserScan(Arc<RwLock<Topic<LaserScan>>>),
    JointState(Arc<RwLock<Topic<JointState>>>),
    Clock(Arc<RwLock<Topic<Clock>>>),
    TimeReference(Arc<RwLock<Topic<TimeReference>>>),
    Image(Arc<RwLock<Topic<Image>>>),
    PointCloud(Arc<RwLock<Topic<PointCloud>>>),
    DepthImage(Arc<RwLock<Topic<DepthImage>>>),
    // Geometry types
    Twist(Arc<RwLock<Topic<Twist>>>),
    Vector3(Arc<RwLock<Topic<Vector3>>>),
    Point3(Arc<RwLock<Topic<Point3>>>),
    Quaternion(Arc<RwLock<Topic<Quaternion>>>),
    TransformStamped(Arc<RwLock<Topic<TransformStamped>>>),
    PoseStamped(Arc<RwLock<Topic<PoseStamped>>>),
    PoseWithCovariance(Arc<RwLock<Topic<PoseWithCovariance>>>),
    TwistWithCovariance(Arc<RwLock<Topic<TwistWithCovariance>>>),
    Accel(Arc<RwLock<Topic<Accel>>>),
    AccelStamped(Arc<RwLock<Topic<AccelStamped>>>),
    // Control types
    MotorCommand(Arc<RwLock<Topic<MotorCommand>>>),
    ServoCommand(Arc<RwLock<Topic<ServoCommand>>>),
    DifferentialDriveCommand(Arc<RwLock<Topic<DifferentialDriveCommand>>>),
    PidConfig(Arc<RwLock<Topic<PidConfig>>>),
    TrajectoryPoint(Arc<RwLock<Topic<TrajectoryPoint>>>),
    JointCommand(Arc<RwLock<Topic<JointCommand>>>),
    PwmCommand(Arc<RwLock<Topic<PwmCommand>>>),
    StepperCommand(Arc<RwLock<Topic<StepperCommand>>>),
    // Sensor types
    RangeSensor(Arc<RwLock<Topic<RangeSensor>>>),
    BatteryState(Arc<RwLock<Topic<BatteryState>>>),
    NavSatFix(Arc<RwLock<Topic<NavSatFix>>>),
    MagneticField(Arc<RwLock<Topic<MagneticField>>>),
    Temperature(Arc<RwLock<Topic<Temperature>>>),
    FluidPressure(Arc<RwLock<Topic<FluidPressure>>>),
    Illuminance(Arc<RwLock<Topic<Illuminance>>>),
    // Diagnostics types
    Heartbeat(Arc<RwLock<Topic<Heartbeat>>>),
    DiagnosticStatus(Arc<RwLock<Topic<DiagnosticStatus>>>),
    EmergencyStop(Arc<RwLock<Topic<EmergencyStop>>>),
    ResourceUsage(Arc<RwLock<Topic<ResourceUsage>>>),
    // Force types
    WrenchStamped(Arc<RwLock<Topic<WrenchStamped>>>),
    ForceCommand(Arc<RwLock<Topic<ForceCommand>>>),
    ContactInfo(Arc<RwLock<Topic<ContactInfo>>>),
    // Navigation types
    NavGoal(Arc<RwLock<Topic<NavGoal>>>),
    GoalResult(Arc<RwLock<Topic<GoalResult>>>),
    PathPlan(Arc<RwLock<Topic<PathPlan>>>),
    // Input types
    JoystickInput(Arc<RwLock<Topic<JoystickInput>>>),
    KeyboardInput(Arc<RwLock<Topic<KeyboardInput>>>),
    // Detection/Perception types
    BoundingBox2D(Arc<RwLock<Topic<BoundingBox2D>>>),
    BoundingBox3D(Arc<RwLock<Topic<BoundingBox3D>>>),
    Detection(Arc<RwLock<Topic<Detection>>>),
    Detection3D(Arc<RwLock<Topic<Detection3D>>>),
    SegmentationMask(Arc<RwLock<Topic<SegmentationMask>>>),
    // Tracking types
    TrackedObject(Arc<RwLock<Topic<TrackedObject>>>),
    TrackingHeader(Arc<RwLock<Topic<TrackingHeader>>>),
    // Landmark types
    Landmark(Arc<RwLock<Topic<Landmark>>>),
    Landmark3D(Arc<RwLock<Topic<Landmark3D>>>),
    LandmarkArray(Arc<RwLock<Topic<LandmarkArray>>>),
    // Perception helper types (serde-based)
    PointField(Arc<RwLock<Topic<PointField>>>),
    PlaneDetection(Arc<RwLock<Topic<PlaneDetection>>>),
    PlaneArray(Arc<RwLock<Topic<PlaneArray>>>),
    // ML types (serde-based)
    TensorData(Arc<RwLock<Topic<TensorData>>>),
    Predictions(Arc<RwLock<Topic<Predictions>>>),
    InferenceMetrics(Arc<RwLock<Topic<InferenceMetrics>>>),
    ModelInfo(Arc<RwLock<Topic<ModelInfo>>>),
    FeatureVector(Arc<RwLock<Topic<FeatureVector>>>),
    Classification(Arc<RwLock<Topic<Classification>>>),
    ChatMessage(Arc<RwLock<Topic<ChatMessage>>>),
    LLMRequest(Arc<RwLock<Topic<LLMRequest>>>),
    LLMResponse(Arc<RwLock<Topic<LLMResponse>>>),
    TrainingMetrics(Arc<RwLock<Topic<TrainingMetrics>>>),
    MlTrajectoryPoint(Arc<RwLock<Topic<MlTrajectoryPoint>>>),
    DeploymentConfig(Arc<RwLock<Topic<DeploymentConfig>>>),
    // Vision types (serde-based)
    CompressedImage(Arc<RwLock<Topic<CompressedImage>>>),
    CameraInfo(Arc<RwLock<Topic<CameraInfo>>>),
    RegionOfInterest(Arc<RwLock<Topic<RegionOfInterest>>>),
    StereoInfo(Arc<RwLock<Topic<StereoInfo>>>),
    // Force types (additional Pod)
    TactileArray(Arc<RwLock<Topic<TactileArray>>>),
    ImpedanceParameters(Arc<RwLock<Topic<ImpedanceParameters>>>),
    HapticFeedback(Arc<RwLock<Topic<HapticFeedback>>>),
    // Diagnostics types (additional Pod)
    DiagnosticValue(Arc<RwLock<Topic<DiagnosticValue>>>),
    DiagnosticReport(Arc<RwLock<Topic<DiagnosticReport>>>),
    NodeHeartbeat(Arc<RwLock<Topic<NodeHeartbeat>>>),
    SafetyStatus(Arc<RwLock<Topic<SafetyStatus>>>),
    // Navigation types (additional)
    Waypoint(Arc<RwLock<Topic<Waypoint>>>),
    NavPath(Arc<RwLock<Topic<NavPath>>>),
    VelocityObstacle(Arc<RwLock<Topic<VelocityObstacle>>>),
    VelocityObstacles(Arc<RwLock<Topic<VelocityObstacles>>>),
    OccupancyGrid(Arc<RwLock<Topic<OccupancyGrid>>>),
    CostMap(Arc<RwLock<Topic<CostMap>>>),
    Generic(Arc<RwLock<Topic<GenericMessage>>>),
}

/// Dispatch a method call to the inner Topic across all TopicType variants.
/// The bound variable `$t` is an `Arc<RwLock<Topic<T>>>` in each arm.
/// All arms evaluate the same expression `$body`.
macro_rules! topic_dispatch {
    ($topic_type:expr, $t:ident, $body:expr) => {
        match $topic_type {
            TopicType::CmdVel($t) => $body,
            TopicType::Pose2D($t) => $body,
            TopicType::Pose3D($t) => $body,
            TopicType::Imu($t) => $body,
            TopicType::Odometry($t) => $body,
            TopicType::LaserScan($t) => $body,
            TopicType::JointState($t) => $body,
            TopicType::Clock($t) => $body,
            TopicType::TimeReference($t) => $body,
            TopicType::Image($t) => $body,
            TopicType::PointCloud($t) => $body,
            TopicType::DepthImage($t) => $body,
            TopicType::Twist($t) => $body,
            TopicType::Vector3($t) => $body,
            TopicType::Point3($t) => $body,
            TopicType::Quaternion($t) => $body,
            TopicType::TransformStamped($t) => $body,
            TopicType::PoseStamped($t) => $body,
            TopicType::PoseWithCovariance($t) => $body,
            TopicType::TwistWithCovariance($t) => $body,
            TopicType::Accel($t) => $body,
            TopicType::AccelStamped($t) => $body,
            TopicType::MotorCommand($t) => $body,
            TopicType::ServoCommand($t) => $body,
            TopicType::DifferentialDriveCommand($t) => $body,
            TopicType::PidConfig($t) => $body,
            TopicType::TrajectoryPoint($t) => $body,
            TopicType::JointCommand($t) => $body,
            TopicType::PwmCommand($t) => $body,
            TopicType::StepperCommand($t) => $body,
            TopicType::RangeSensor($t) => $body,
            TopicType::BatteryState($t) => $body,
            TopicType::NavSatFix($t) => $body,
            TopicType::MagneticField($t) => $body,
            TopicType::Temperature($t) => $body,
            TopicType::FluidPressure($t) => $body,
            TopicType::Illuminance($t) => $body,
            TopicType::Heartbeat($t) => $body,
            TopicType::DiagnosticStatus($t) => $body,
            TopicType::EmergencyStop($t) => $body,
            TopicType::ResourceUsage($t) => $body,
            TopicType::WrenchStamped($t) => $body,
            TopicType::ForceCommand($t) => $body,
            TopicType::ContactInfo($t) => $body,
            TopicType::NavGoal($t) => $body,
            TopicType::GoalResult($t) => $body,
            TopicType::PathPlan($t) => $body,
            TopicType::JoystickInput($t) => $body,
            TopicType::KeyboardInput($t) => $body,
            TopicType::BoundingBox2D($t) => $body,
            TopicType::BoundingBox3D($t) => $body,
            TopicType::Detection($t) => $body,
            TopicType::Detection3D($t) => $body,
            TopicType::SegmentationMask($t) => $body,
            TopicType::TrackedObject($t) => $body,
            TopicType::TrackingHeader($t) => $body,
            TopicType::Landmark($t) => $body,
            TopicType::Landmark3D($t) => $body,
            TopicType::LandmarkArray($t) => $body,
            TopicType::PointField($t) => $body,
            TopicType::PlaneDetection($t) => $body,
            TopicType::PlaneArray($t) => $body,
            TopicType::TensorData($t) => $body,
            TopicType::Predictions($t) => $body,
            TopicType::InferenceMetrics($t) => $body,
            TopicType::ModelInfo($t) => $body,
            TopicType::FeatureVector($t) => $body,
            TopicType::Classification($t) => $body,
            TopicType::ChatMessage($t) => $body,
            TopicType::LLMRequest($t) => $body,
            TopicType::LLMResponse($t) => $body,
            TopicType::TrainingMetrics($t) => $body,
            TopicType::MlTrajectoryPoint($t) => $body,
            TopicType::DeploymentConfig($t) => $body,
            TopicType::CompressedImage($t) => $body,
            TopicType::CameraInfo($t) => $body,
            TopicType::RegionOfInterest($t) => $body,
            TopicType::StereoInfo($t) => $body,
            TopicType::TactileArray($t) => $body,
            TopicType::ImpedanceParameters($t) => $body,
            TopicType::HapticFeedback($t) => $body,
            TopicType::DiagnosticValue($t) => $body,
            TopicType::DiagnosticReport($t) => $body,
            TopicType::NodeHeartbeat($t) => $body,
            TopicType::SafetyStatus($t) => $body,
            TopicType::Waypoint($t) => $body,
            TopicType::NavPath($t) => $body,
            TopicType::VelocityObstacle($t) => $body,
            TopicType::VelocityObstacles($t) => $body,
            TopicType::OccupancyGrid($t) => $body,
            TopicType::CostMap($t) => $body,
            TopicType::Generic($t) => $body,
        }
    };
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
            "Pose3D" => {
                let topic = create_topic::<Pose3D>(&effective_endpoint, cap)?;
                TopicType::Pose3D(Arc::new(RwLock::new(topic)))
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
            "JointState" => {
                let topic = create_topic::<JointState>(&effective_endpoint, cap)?;
                TopicType::JointState(Arc::new(RwLock::new(topic)))
            }
            "Clock" => {
                let topic = create_topic::<Clock>(&effective_endpoint, cap)?;
                TopicType::Clock(Arc::new(RwLock::new(topic)))
            }
            "TimeReference" => {
                let topic = create_topic::<TimeReference>(&effective_endpoint, cap)?;
                TopicType::TimeReference(Arc::new(RwLock::new(topic)))
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
            // Geometry types
            "Twist" => {
                let topic = create_topic::<Twist>(&effective_endpoint, cap)?;
                TopicType::Twist(Arc::new(RwLock::new(topic)))
            }
            "Vector3" => {
                let topic = create_topic::<Vector3>(&effective_endpoint, cap)?;
                TopicType::Vector3(Arc::new(RwLock::new(topic)))
            }
            "Point3" => {
                let topic = create_topic::<Point3>(&effective_endpoint, cap)?;
                TopicType::Point3(Arc::new(RwLock::new(topic)))
            }
            "Quaternion" => {
                let topic = create_topic::<Quaternion>(&effective_endpoint, cap)?;
                TopicType::Quaternion(Arc::new(RwLock::new(topic)))
            }
            "TransformStamped" => {
                let topic = create_topic::<TransformStamped>(&effective_endpoint, cap)?;
                TopicType::TransformStamped(Arc::new(RwLock::new(topic)))
            }
            "PoseStamped" => {
                let topic = create_topic::<PoseStamped>(&effective_endpoint, cap)?;
                TopicType::PoseStamped(Arc::new(RwLock::new(topic)))
            }
            "PoseWithCovariance" => {
                let topic = create_topic::<PoseWithCovariance>(&effective_endpoint, cap)?;
                TopicType::PoseWithCovariance(Arc::new(RwLock::new(topic)))
            }
            "TwistWithCovariance" => {
                let topic = create_topic::<TwistWithCovariance>(&effective_endpoint, cap)?;
                TopicType::TwistWithCovariance(Arc::new(RwLock::new(topic)))
            }
            "Accel" => {
                let topic = create_topic::<Accel>(&effective_endpoint, cap)?;
                TopicType::Accel(Arc::new(RwLock::new(topic)))
            }
            "AccelStamped" => {
                let topic = create_topic::<AccelStamped>(&effective_endpoint, cap)?;
                TopicType::AccelStamped(Arc::new(RwLock::new(topic)))
            }
            // Control types
            "MotorCommand" => {
                let topic = create_topic::<MotorCommand>(&effective_endpoint, cap)?;
                TopicType::MotorCommand(Arc::new(RwLock::new(topic)))
            }
            "ServoCommand" => {
                let topic = create_topic::<ServoCommand>(&effective_endpoint, cap)?;
                TopicType::ServoCommand(Arc::new(RwLock::new(topic)))
            }
            "DifferentialDriveCommand" => {
                let topic = create_topic::<DifferentialDriveCommand>(&effective_endpoint, cap)?;
                TopicType::DifferentialDriveCommand(Arc::new(RwLock::new(topic)))
            }
            "PidConfig" => {
                let topic = create_topic::<PidConfig>(&effective_endpoint, cap)?;
                TopicType::PidConfig(Arc::new(RwLock::new(topic)))
            }
            "TrajectoryPoint" => {
                let topic = create_topic::<TrajectoryPoint>(&effective_endpoint, cap)?;
                TopicType::TrajectoryPoint(Arc::new(RwLock::new(topic)))
            }
            "JointCommand" => {
                let topic = create_topic::<JointCommand>(&effective_endpoint, cap)?;
                TopicType::JointCommand(Arc::new(RwLock::new(topic)))
            }
            "PwmCommand" => {
                let topic = create_topic::<PwmCommand>(&effective_endpoint, cap)?;
                TopicType::PwmCommand(Arc::new(RwLock::new(topic)))
            }
            "StepperCommand" => {
                let topic = create_topic::<StepperCommand>(&effective_endpoint, cap)?;
                TopicType::StepperCommand(Arc::new(RwLock::new(topic)))
            }
            // Sensor types
            "RangeSensor" => {
                let topic = create_topic::<RangeSensor>(&effective_endpoint, cap)?;
                TopicType::RangeSensor(Arc::new(RwLock::new(topic)))
            }
            "BatteryState" => {
                let topic = create_topic::<BatteryState>(&effective_endpoint, cap)?;
                TopicType::BatteryState(Arc::new(RwLock::new(topic)))
            }
            "NavSatFix" => {
                let topic = create_topic::<NavSatFix>(&effective_endpoint, cap)?;
                TopicType::NavSatFix(Arc::new(RwLock::new(topic)))
            }
            "MagneticField" => {
                let topic = create_topic::<MagneticField>(&effective_endpoint, cap)?;
                TopicType::MagneticField(Arc::new(RwLock::new(topic)))
            }
            "Temperature" => {
                let topic = create_topic::<Temperature>(&effective_endpoint, cap)?;
                TopicType::Temperature(Arc::new(RwLock::new(topic)))
            }
            "FluidPressure" => {
                let topic = create_topic::<FluidPressure>(&effective_endpoint, cap)?;
                TopicType::FluidPressure(Arc::new(RwLock::new(topic)))
            }
            "Illuminance" => {
                let topic = create_topic::<Illuminance>(&effective_endpoint, cap)?;
                TopicType::Illuminance(Arc::new(RwLock::new(topic)))
            }
            // Diagnostics types
            "Heartbeat" => {
                let topic = create_topic::<Heartbeat>(&effective_endpoint, cap)?;
                TopicType::Heartbeat(Arc::new(RwLock::new(topic)))
            }
            "DiagnosticStatus" => {
                let topic = create_topic::<DiagnosticStatus>(&effective_endpoint, cap)?;
                TopicType::DiagnosticStatus(Arc::new(RwLock::new(topic)))
            }
            "EmergencyStop" => {
                let topic = create_topic::<EmergencyStop>(&effective_endpoint, cap)?;
                TopicType::EmergencyStop(Arc::new(RwLock::new(topic)))
            }
            "ResourceUsage" => {
                let topic = create_topic::<ResourceUsage>(&effective_endpoint, cap)?;
                TopicType::ResourceUsage(Arc::new(RwLock::new(topic)))
            }
            // Force types
            "WrenchStamped" => {
                let topic = create_topic::<WrenchStamped>(&effective_endpoint, cap)?;
                TopicType::WrenchStamped(Arc::new(RwLock::new(topic)))
            }
            "ForceCommand" => {
                let topic = create_topic::<ForceCommand>(&effective_endpoint, cap)?;
                TopicType::ForceCommand(Arc::new(RwLock::new(topic)))
            }
            "ContactInfo" => {
                let topic = create_topic::<ContactInfo>(&effective_endpoint, cap)?;
                TopicType::ContactInfo(Arc::new(RwLock::new(topic)))
            }
            // Navigation types
            "NavGoal" => {
                let topic = create_topic::<NavGoal>(&effective_endpoint, cap)?;
                TopicType::NavGoal(Arc::new(RwLock::new(topic)))
            }
            "GoalResult" => {
                let topic = create_topic::<GoalResult>(&effective_endpoint, cap)?;
                TopicType::GoalResult(Arc::new(RwLock::new(topic)))
            }
            "PathPlan" => {
                let topic = create_topic::<PathPlan>(&effective_endpoint, cap)?;
                TopicType::PathPlan(Arc::new(RwLock::new(topic)))
            }
            // Input types
            "JoystickInput" => {
                let topic = create_topic::<JoystickInput>(&effective_endpoint, cap)?;
                TopicType::JoystickInput(Arc::new(RwLock::new(topic)))
            }
            "KeyboardInput" => {
                let topic = create_topic::<KeyboardInput>(&effective_endpoint, cap)?;
                TopicType::KeyboardInput(Arc::new(RwLock::new(topic)))
            }
            // Detection/Perception types
            "BoundingBox2D" => {
                let topic = create_topic::<BoundingBox2D>(&effective_endpoint, cap)?;
                TopicType::BoundingBox2D(Arc::new(RwLock::new(topic)))
            }
            "BoundingBox3D" => {
                let topic = create_topic::<BoundingBox3D>(&effective_endpoint, cap)?;
                TopicType::BoundingBox3D(Arc::new(RwLock::new(topic)))
            }
            "Detection" => {
                let topic = create_topic::<Detection>(&effective_endpoint, cap)?;
                TopicType::Detection(Arc::new(RwLock::new(topic)))
            }
            "Detection3D" => {
                let topic = create_topic::<Detection3D>(&effective_endpoint, cap)?;
                TopicType::Detection3D(Arc::new(RwLock::new(topic)))
            }
            "SegmentationMask" => {
                let topic = create_topic::<SegmentationMask>(&effective_endpoint, cap)?;
                TopicType::SegmentationMask(Arc::new(RwLock::new(topic)))
            }
            // Tracking types
            "TrackedObject" => {
                let topic = create_topic::<TrackedObject>(&effective_endpoint, cap)?;
                TopicType::TrackedObject(Arc::new(RwLock::new(topic)))
            }
            "TrackingHeader" => {
                let topic = create_topic::<TrackingHeader>(&effective_endpoint, cap)?;
                TopicType::TrackingHeader(Arc::new(RwLock::new(topic)))
            }
            // Landmark types
            "Landmark" => {
                let topic = create_topic::<Landmark>(&effective_endpoint, cap)?;
                TopicType::Landmark(Arc::new(RwLock::new(topic)))
            }
            "Landmark3D" => {
                let topic = create_topic::<Landmark3D>(&effective_endpoint, cap)?;
                TopicType::Landmark3D(Arc::new(RwLock::new(topic)))
            }
            "LandmarkArray" => {
                let topic = create_topic::<LandmarkArray>(&effective_endpoint, cap)?;
                TopicType::LandmarkArray(Arc::new(RwLock::new(topic)))
            }
            // Perception helper types (serde-based)
            "PointField" => {
                let topic = create_topic::<PointField>(&effective_endpoint, cap)?;
                TopicType::PointField(Arc::new(RwLock::new(topic)))
            }
            "PlaneDetection" => {
                let topic = create_topic::<PlaneDetection>(&effective_endpoint, cap)?;
                TopicType::PlaneDetection(Arc::new(RwLock::new(topic)))
            }
            "PlaneArray" => {
                let topic = create_topic::<PlaneArray>(&effective_endpoint, cap)?;
                TopicType::PlaneArray(Arc::new(RwLock::new(topic)))
            }
            // ML types (serde-based)
            "TensorData" => {
                let topic = create_topic::<TensorData>(&effective_endpoint, cap)?;
                TopicType::TensorData(Arc::new(RwLock::new(topic)))
            }
            "Predictions" => {
                let topic = create_topic::<Predictions>(&effective_endpoint, cap)?;
                TopicType::Predictions(Arc::new(RwLock::new(topic)))
            }
            "InferenceMetrics" => {
                let topic = create_topic::<InferenceMetrics>(&effective_endpoint, cap)?;
                TopicType::InferenceMetrics(Arc::new(RwLock::new(topic)))
            }
            "ModelInfo" => {
                let topic = create_topic::<ModelInfo>(&effective_endpoint, cap)?;
                TopicType::ModelInfo(Arc::new(RwLock::new(topic)))
            }
            "FeatureVector" => {
                let topic = create_topic::<FeatureVector>(&effective_endpoint, cap)?;
                TopicType::FeatureVector(Arc::new(RwLock::new(topic)))
            }
            "Classification" => {
                let topic = create_topic::<Classification>(&effective_endpoint, cap)?;
                TopicType::Classification(Arc::new(RwLock::new(topic)))
            }
            "ChatMessage" => {
                let topic = create_topic::<ChatMessage>(&effective_endpoint, cap)?;
                TopicType::ChatMessage(Arc::new(RwLock::new(topic)))
            }
            "LLMRequest" => {
                let topic = create_topic::<LLMRequest>(&effective_endpoint, cap)?;
                TopicType::LLMRequest(Arc::new(RwLock::new(topic)))
            }
            "LLMResponse" => {
                let topic = create_topic::<LLMResponse>(&effective_endpoint, cap)?;
                TopicType::LLMResponse(Arc::new(RwLock::new(topic)))
            }
            "TrainingMetrics" => {
                let topic = create_topic::<TrainingMetrics>(&effective_endpoint, cap)?;
                TopicType::TrainingMetrics(Arc::new(RwLock::new(topic)))
            }
            "MlTrajectoryPoint" => {
                let topic = create_topic::<MlTrajectoryPoint>(&effective_endpoint, cap)?;
                TopicType::MlTrajectoryPoint(Arc::new(RwLock::new(topic)))
            }
            "DeploymentConfig" => {
                let topic = create_topic::<DeploymentConfig>(&effective_endpoint, cap)?;
                TopicType::DeploymentConfig(Arc::new(RwLock::new(topic)))
            }
            // Vision types (serde-based)
            "CompressedImage" => {
                let topic = create_topic::<CompressedImage>(&effective_endpoint, cap)?;
                TopicType::CompressedImage(Arc::new(RwLock::new(topic)))
            }
            "CameraInfo" => {
                let topic = create_topic::<CameraInfo>(&effective_endpoint, cap)?;
                TopicType::CameraInfo(Arc::new(RwLock::new(topic)))
            }
            "RegionOfInterest" => {
                let topic = create_topic::<RegionOfInterest>(&effective_endpoint, cap)?;
                TopicType::RegionOfInterest(Arc::new(RwLock::new(topic)))
            }
            "StereoInfo" => {
                let topic = create_topic::<StereoInfo>(&effective_endpoint, cap)?;
                TopicType::StereoInfo(Arc::new(RwLock::new(topic)))
            }
            // Force types (additional)
            "TactileArray" => {
                let topic = create_topic::<TactileArray>(&effective_endpoint, cap)?;
                TopicType::TactileArray(Arc::new(RwLock::new(topic)))
            }
            "ImpedanceParameters" => {
                let topic = create_topic::<ImpedanceParameters>(&effective_endpoint, cap)?;
                TopicType::ImpedanceParameters(Arc::new(RwLock::new(topic)))
            }
            "HapticFeedback" => {
                let topic = create_topic::<HapticFeedback>(&effective_endpoint, cap)?;
                TopicType::HapticFeedback(Arc::new(RwLock::new(topic)))
            }
            // Diagnostics types (additional)
            "DiagnosticValue" => {
                let topic = create_topic::<DiagnosticValue>(&effective_endpoint, cap)?;
                TopicType::DiagnosticValue(Arc::new(RwLock::new(topic)))
            }
            "DiagnosticReport" => {
                let topic = create_topic::<DiagnosticReport>(&effective_endpoint, cap)?;
                TopicType::DiagnosticReport(Arc::new(RwLock::new(topic)))
            }
            "NodeHeartbeat" => {
                let topic = create_topic::<NodeHeartbeat>(&effective_endpoint, cap)?;
                TopicType::NodeHeartbeat(Arc::new(RwLock::new(topic)))
            }
            "SafetyStatus" => {
                let topic = create_topic::<SafetyStatus>(&effective_endpoint, cap)?;
                TopicType::SafetyStatus(Arc::new(RwLock::new(topic)))
            }
            // Navigation types (additional)
            "Waypoint" => {
                let topic = create_topic::<Waypoint>(&effective_endpoint, cap)?;
                TopicType::Waypoint(Arc::new(RwLock::new(topic)))
            }
            "NavPath" => {
                let topic = create_topic::<NavPath>(&effective_endpoint, cap)?;
                TopicType::NavPath(Arc::new(RwLock::new(topic)))
            }
            "VelocityObstacle" => {
                let topic = create_topic::<VelocityObstacle>(&effective_endpoint, cap)?;
                TopicType::VelocityObstacle(Arc::new(RwLock::new(topic)))
            }
            "VelocityObstacles" => {
                let topic = create_topic::<VelocityObstacles>(&effective_endpoint, cap)?;
                TopicType::VelocityObstacles(Arc::new(RwLock::new(topic)))
            }
            "OccupancyGrid" => {
                let topic = create_topic::<OccupancyGrid>(&effective_endpoint, cap)?;
                TopicType::OccupancyGrid(Arc::new(RwLock::new(topic)))
            }
            "CostMap" => {
                let topic = create_topic::<CostMap>(&effective_endpoint, cap)?;
                TopicType::CostMap(Arc::new(RwLock::new(topic)))
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
    fn send(&self, py: Python, message: Py<PyAny>, node: Option<Py<PyAny>>) -> PyResult<bool> {
        use std::time::Instant;
        let start = Instant::now();

        let result = match &self.topic_type {
            TopicType::CmdVel(topic) => {
                // POD-optimized: direct struct access, no getattr
                let cmd = message.extract::<PyRef<PyCmdVel>>(py)?.inner;
                let topic_ref = topic.clone();
                let success = py.detach(|| {
                    topic_ref.write().expect("topic lock poisoned").send(cmd);
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
                    topic_ref.write().expect("topic lock poisoned").send(pose);
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
                    topic_ref.write().expect("topic lock poisoned").send(imu);
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
                    topic_ref.write().expect("topic lock poisoned").send(odom);
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
                    topic_ref.write().expect("topic lock poisoned").send(scan);
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
            TopicType::Pose3D(topic) => {
                let pose = message.extract::<PyRef<PyPose3D>>(py)?.inner;
                let topic_ref = topic.clone();
                let success = py.detach(|| {
                    topic_ref.write().expect("topic lock poisoned").send(pose);
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
            TopicType::JointState(topic) => {
                let js = message.extract::<PyRef<PyJointState>>(py)?.inner;
                let topic_ref = topic.clone();
                let success = py.detach(|| {
                    topic_ref.write().expect("topic lock poisoned").send(js);
                    true
                });
                if node.is_some() {
                    use horus::core::LogSummary;
                    log_ipc_event(
                        py,
                        &node,
                        &self.name,
                        js.log_summary(),
                        start.elapsed().as_nanos() as u64,
                        "log_pub",
                    );
                }
                success
            }
            TopicType::Clock(topic) => {
                let clk = message.extract::<PyRef<PyClock>>(py)?.inner;
                let topic_ref = topic.clone();
                let success = py.detach(|| {
                    topic_ref.write().expect("topic lock poisoned").send(clk);
                    true
                });
                if node.is_some() {
                    use horus::core::LogSummary;
                    log_ipc_event(
                        py,
                        &node,
                        &self.name,
                        clk.log_summary(),
                        start.elapsed().as_nanos() as u64,
                        "log_pub",
                    );
                }
                success
            }
            TopicType::TimeReference(topic) => {
                let tr = message.extract::<PyRef<PyTimeReference>>(py)?.inner;
                let topic_ref = topic.clone();
                let success = py.detach(|| {
                    topic_ref.write().expect("topic lock poisoned").send(tr);
                    true
                });
                if node.is_some() {
                    use horus::core::LogSummary;
                    log_ipc_event(
                        py,
                        &node,
                        &self.name,
                        tr.log_summary(),
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
                    let topic = topic_ref.read().expect("topic lock poisoned");
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
                    let topic = topic_ref.read().expect("topic lock poisoned");
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
                    let topic = topic_ref.read().expect("topic lock poisoned");
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
            // Geometry types — POD-optimized send
            TopicType::Twist(topic) => {
                let msg = message.extract::<PyRef<PyTwist>>(py)?.inner;
                let topic_ref = topic.clone();
                let success = py.detach(|| {
                    topic_ref.write().expect("topic lock poisoned").send(msg);
                    true
                });
                if node.is_some() {
                    use horus::core::LogSummary;
                    log_ipc_event(
                        py,
                        &node,
                        &self.name,
                        msg.log_summary(),
                        start.elapsed().as_nanos() as u64,
                        "log_pub",
                    );
                }
                success
            }
            TopicType::Vector3(topic) => {
                let msg = message.extract::<PyRef<PyVector3>>(py)?.inner;
                let topic_ref = topic.clone();
                let success = py.detach(|| {
                    topic_ref.write().expect("topic lock poisoned").send(msg);
                    true
                });
                if node.is_some() {
                    use horus::core::LogSummary;
                    log_ipc_event(
                        py,
                        &node,
                        &self.name,
                        msg.log_summary(),
                        start.elapsed().as_nanos() as u64,
                        "log_pub",
                    );
                }
                success
            }
            TopicType::Point3(topic) => {
                let msg = message.extract::<PyRef<PyPoint3>>(py)?.inner;
                let topic_ref = topic.clone();
                let success = py.detach(|| {
                    topic_ref.write().expect("topic lock poisoned").send(msg);
                    true
                });
                if node.is_some() {
                    use horus::core::LogSummary;
                    log_ipc_event(
                        py,
                        &node,
                        &self.name,
                        msg.log_summary(),
                        start.elapsed().as_nanos() as u64,
                        "log_pub",
                    );
                }
                success
            }
            TopicType::Quaternion(topic) => {
                let msg = message.extract::<PyRef<PyQuaternion>>(py)?.inner;
                let topic_ref = topic.clone();
                let success = py.detach(|| {
                    topic_ref.write().expect("topic lock poisoned").send(msg);
                    true
                });
                if node.is_some() {
                    use horus::core::LogSummary;
                    log_ipc_event(
                        py,
                        &node,
                        &self.name,
                        msg.log_summary(),
                        start.elapsed().as_nanos() as u64,
                        "log_pub",
                    );
                }
                success
            }
            TopicType::TransformStamped(topic) => {
                let msg = message.extract::<PyRef<PyTransformStamped>>(py)?.inner;
                let topic_ref = topic.clone();
                let success = py.detach(|| {
                    topic_ref.write().expect("topic lock poisoned").send(msg);
                    true
                });
                if node.is_some() {
                    use horus::core::LogSummary;
                    log_ipc_event(
                        py,
                        &node,
                        &self.name,
                        msg.log_summary(),
                        start.elapsed().as_nanos() as u64,
                        "log_pub",
                    );
                }
                success
            }
            TopicType::PoseStamped(topic) => {
                let msg = message.extract::<PyRef<PyPoseStamped>>(py)?.inner;
                let topic_ref = topic.clone();
                let success = py.detach(|| {
                    topic_ref.write().expect("topic lock poisoned").send(msg);
                    true
                });
                if node.is_some() {
                    use horus::core::LogSummary;
                    log_ipc_event(
                        py,
                        &node,
                        &self.name,
                        msg.log_summary(),
                        start.elapsed().as_nanos() as u64,
                        "log_pub",
                    );
                }
                success
            }
            TopicType::PoseWithCovariance(topic) => {
                let msg = message.extract::<PyRef<PyPoseWithCovariance>>(py)?.inner;
                let topic_ref = topic.clone();
                let success = py.detach(|| {
                    topic_ref.write().expect("topic lock poisoned").send(msg);
                    true
                });
                if node.is_some() {
                    use horus::core::LogSummary;
                    log_ipc_event(
                        py,
                        &node,
                        &self.name,
                        msg.log_summary(),
                        start.elapsed().as_nanos() as u64,
                        "log_pub",
                    );
                }
                success
            }
            TopicType::TwistWithCovariance(topic) => {
                let msg = message.extract::<PyRef<PyTwistWithCovariance>>(py)?.inner;
                let topic_ref = topic.clone();
                let success = py.detach(|| {
                    topic_ref.write().expect("topic lock poisoned").send(msg);
                    true
                });
                if node.is_some() {
                    use horus::core::LogSummary;
                    log_ipc_event(
                        py,
                        &node,
                        &self.name,
                        msg.log_summary(),
                        start.elapsed().as_nanos() as u64,
                        "log_pub",
                    );
                }
                success
            }
            TopicType::Accel(topic) => {
                let msg = message.extract::<PyRef<PyAccel>>(py)?.inner;
                let topic_ref = topic.clone();
                let success = py.detach(|| {
                    topic_ref.write().expect("topic lock poisoned").send(msg);
                    true
                });
                if node.is_some() {
                    use horus::core::LogSummary;
                    log_ipc_event(
                        py,
                        &node,
                        &self.name,
                        msg.log_summary(),
                        start.elapsed().as_nanos() as u64,
                        "log_pub",
                    );
                }
                success
            }
            TopicType::AccelStamped(topic) => {
                let msg = message.extract::<PyRef<PyAccelStamped>>(py)?.inner;
                let topic_ref = topic.clone();
                let success = py.detach(|| {
                    topic_ref.write().expect("topic lock poisoned").send(msg);
                    true
                });
                if node.is_some() {
                    use horus::core::LogSummary;
                    log_ipc_event(
                        py,
                        &node,
                        &self.name,
                        msg.log_summary(),
                        start.elapsed().as_nanos() as u64,
                        "log_pub",
                    );
                }
                success
            }
            // Control types — POD-optimized send
            TopicType::MotorCommand(topic) => {
                let msg = message.extract::<PyRef<PyMotorCommand>>(py)?.inner;
                let topic_ref = topic.clone();
                let success = py.detach(|| {
                    topic_ref.write().expect("topic lock poisoned").send(msg);
                    true
                });
                if node.is_some() {
                    use horus::core::LogSummary;
                    log_ipc_event(
                        py,
                        &node,
                        &self.name,
                        msg.log_summary(),
                        start.elapsed().as_nanos() as u64,
                        "log_pub",
                    );
                }
                success
            }
            TopicType::ServoCommand(topic) => {
                let msg = message.extract::<PyRef<PyServoCommand>>(py)?.inner;
                let topic_ref = topic.clone();
                let success = py.detach(|| {
                    topic_ref.write().expect("topic lock poisoned").send(msg);
                    true
                });
                if node.is_some() {
                    use horus::core::LogSummary;
                    log_ipc_event(
                        py,
                        &node,
                        &self.name,
                        msg.log_summary(),
                        start.elapsed().as_nanos() as u64,
                        "log_pub",
                    );
                }
                success
            }
            TopicType::DifferentialDriveCommand(topic) => {
                let msg = message
                    .extract::<PyRef<PyDifferentialDriveCommand>>(py)?
                    .inner;
                let topic_ref = topic.clone();
                let success = py.detach(|| {
                    topic_ref.write().expect("topic lock poisoned").send(msg);
                    true
                });
                if node.is_some() {
                    use horus::core::LogSummary;
                    log_ipc_event(
                        py,
                        &node,
                        &self.name,
                        msg.log_summary(),
                        start.elapsed().as_nanos() as u64,
                        "log_pub",
                    );
                }
                success
            }
            TopicType::PidConfig(topic) => {
                let msg = message.extract::<PyRef<PyPidConfig>>(py)?.inner;
                let topic_ref = topic.clone();
                let success = py.detach(|| {
                    topic_ref.write().expect("topic lock poisoned").send(msg);
                    true
                });
                if node.is_some() {
                    use horus::core::LogSummary;
                    log_ipc_event(
                        py,
                        &node,
                        &self.name,
                        msg.log_summary(),
                        start.elapsed().as_nanos() as u64,
                        "log_pub",
                    );
                }
                success
            }
            TopicType::TrajectoryPoint(topic) => {
                let msg = message.extract::<PyRef<PyTrajectoryPoint>>(py)?.inner;
                let topic_ref = topic.clone();
                let success = py.detach(|| {
                    topic_ref.write().expect("topic lock poisoned").send(msg);
                    true
                });
                if node.is_some() {
                    use horus::core::LogSummary;
                    log_ipc_event(
                        py,
                        &node,
                        &self.name,
                        msg.log_summary(),
                        start.elapsed().as_nanos() as u64,
                        "log_pub",
                    );
                }
                success
            }
            TopicType::JointCommand(topic) => {
                let msg = message.extract::<PyRef<PyJointCommand>>(py)?.inner;
                let topic_ref = topic.clone();
                let success = py.detach(|| {
                    topic_ref.write().expect("topic lock poisoned").send(msg);
                    true
                });
                if node.is_some() {
                    use horus::core::LogSummary;
                    log_ipc_event(
                        py,
                        &node,
                        &self.name,
                        msg.log_summary(),
                        start.elapsed().as_nanos() as u64,
                        "log_pub",
                    );
                }
                success
            }
            TopicType::PwmCommand(topic) => {
                let msg = message.extract::<PyRef<PyPwmCommand>>(py)?.inner;
                let topic_ref = topic.clone();
                let success = py.detach(|| {
                    topic_ref.write().expect("topic lock poisoned").send(msg);
                    true
                });
                if node.is_some() {
                    use horus::core::LogSummary;
                    log_ipc_event(
                        py,
                        &node,
                        &self.name,
                        msg.log_summary(),
                        start.elapsed().as_nanos() as u64,
                        "log_pub",
                    );
                }
                success
            }
            TopicType::StepperCommand(topic) => {
                let msg = message.extract::<PyRef<PyStepperCommand>>(py)?.inner;
                let topic_ref = topic.clone();
                let success = py.detach(|| {
                    topic_ref.write().expect("topic lock poisoned").send(msg);
                    true
                });
                if node.is_some() {
                    use horus::core::LogSummary;
                    log_ipc_event(
                        py,
                        &node,
                        &self.name,
                        msg.log_summary(),
                        start.elapsed().as_nanos() as u64,
                        "log_pub",
                    );
                }
                success
            }
            // Sensor types — POD-optimized send
            TopicType::RangeSensor(topic) => {
                let msg = message.extract::<PyRef<PyRangeSensor>>(py)?.inner;
                let topic_ref = topic.clone();
                let success = py.detach(|| {
                    topic_ref.write().expect("topic lock poisoned").send(msg);
                    true
                });
                if node.is_some() {
                    use horus::core::LogSummary;
                    log_ipc_event(
                        py,
                        &node,
                        &self.name,
                        msg.log_summary(),
                        start.elapsed().as_nanos() as u64,
                        "log_pub",
                    );
                }
                success
            }
            TopicType::BatteryState(topic) => {
                let msg = message.extract::<PyRef<PyBatteryState>>(py)?.inner;
                let topic_ref = topic.clone();
                let success = py.detach(|| {
                    topic_ref.write().expect("topic lock poisoned").send(msg);
                    true
                });
                if node.is_some() {
                    use horus::core::LogSummary;
                    log_ipc_event(
                        py,
                        &node,
                        &self.name,
                        msg.log_summary(),
                        start.elapsed().as_nanos() as u64,
                        "log_pub",
                    );
                }
                success
            }
            TopicType::NavSatFix(topic) => {
                let msg = message.extract::<PyRef<PyNavSatFix>>(py)?.inner;
                let topic_ref = topic.clone();
                let success = py.detach(|| {
                    topic_ref.write().expect("topic lock poisoned").send(msg);
                    true
                });
                if node.is_some() {
                    use horus::core::LogSummary;
                    log_ipc_event(
                        py,
                        &node,
                        &self.name,
                        msg.log_summary(),
                        start.elapsed().as_nanos() as u64,
                        "log_pub",
                    );
                }
                success
            }
            TopicType::MagneticField(topic) => {
                let msg = message.extract::<PyRef<PyMagneticField>>(py)?.inner;
                let topic_ref = topic.clone();
                let success = py.detach(|| {
                    topic_ref.write().expect("topic lock poisoned").send(msg);
                    true
                });
                if node.is_some() {
                    use horus::core::LogSummary;
                    log_ipc_event(
                        py,
                        &node,
                        &self.name,
                        msg.log_summary(),
                        start.elapsed().as_nanos() as u64,
                        "log_pub",
                    );
                }
                success
            }
            TopicType::Temperature(topic) => {
                let msg = message.extract::<PyRef<PyTemperature>>(py)?.inner;
                let topic_ref = topic.clone();
                let success = py.detach(|| {
                    topic_ref.write().expect("topic lock poisoned").send(msg);
                    true
                });
                if node.is_some() {
                    use horus::core::LogSummary;
                    log_ipc_event(
                        py,
                        &node,
                        &self.name,
                        msg.log_summary(),
                        start.elapsed().as_nanos() as u64,
                        "log_pub",
                    );
                }
                success
            }
            TopicType::FluidPressure(topic) => {
                let msg = message.extract::<PyRef<PyFluidPressure>>(py)?.inner;
                let topic_ref = topic.clone();
                let success = py.detach(|| {
                    topic_ref.write().expect("topic lock poisoned").send(msg);
                    true
                });
                if node.is_some() {
                    use horus::core::LogSummary;
                    log_ipc_event(
                        py,
                        &node,
                        &self.name,
                        msg.log_summary(),
                        start.elapsed().as_nanos() as u64,
                        "log_pub",
                    );
                }
                success
            }
            TopicType::Illuminance(topic) => {
                let msg = message.extract::<PyRef<PyIlluminance>>(py)?.inner;
                let topic_ref = topic.clone();
                let success = py.detach(|| {
                    topic_ref.write().expect("topic lock poisoned").send(msg);
                    true
                });
                if node.is_some() {
                    use horus::core::LogSummary;
                    log_ipc_event(
                        py,
                        &node,
                        &self.name,
                        msg.log_summary(),
                        start.elapsed().as_nanos() as u64,
                        "log_pub",
                    );
                }
                success
            }
            // Diagnostics types — POD-optimized send
            TopicType::Heartbeat(topic) => {
                let msg = message.extract::<PyRef<PyHeartbeat>>(py)?.inner;
                let topic_ref = topic.clone();
                let success = py.detach(|| {
                    topic_ref.write().expect("topic lock poisoned").send(msg);
                    true
                });
                if node.is_some() {
                    use horus::core::LogSummary;
                    log_ipc_event(
                        py,
                        &node,
                        &self.name,
                        msg.log_summary(),
                        start.elapsed().as_nanos() as u64,
                        "log_pub",
                    );
                }
                success
            }
            TopicType::DiagnosticStatus(topic) => {
                let msg = message.extract::<PyRef<PyDiagnosticStatus>>(py)?.inner;
                let topic_ref = topic.clone();
                let success = py.detach(|| {
                    topic_ref.write().expect("topic lock poisoned").send(msg);
                    true
                });
                if node.is_some() {
                    use horus::core::LogSummary;
                    log_ipc_event(
                        py,
                        &node,
                        &self.name,
                        msg.log_summary(),
                        start.elapsed().as_nanos() as u64,
                        "log_pub",
                    );
                }
                success
            }
            TopicType::EmergencyStop(topic) => {
                let msg = message.extract::<PyRef<PyEmergencyStop>>(py)?.inner;
                let topic_ref = topic.clone();
                let success = py.detach(|| {
                    topic_ref.write().expect("topic lock poisoned").send(msg);
                    true
                });
                if node.is_some() {
                    use horus::core::LogSummary;
                    log_ipc_event(
                        py,
                        &node,
                        &self.name,
                        msg.log_summary(),
                        start.elapsed().as_nanos() as u64,
                        "log_pub",
                    );
                }
                success
            }
            TopicType::ResourceUsage(topic) => {
                let msg = message.extract::<PyRef<PyResourceUsage>>(py)?.inner;
                let topic_ref = topic.clone();
                let success = py.detach(|| {
                    topic_ref.write().expect("topic lock poisoned").send(msg);
                    true
                });
                if node.is_some() {
                    use horus::core::LogSummary;
                    log_ipc_event(
                        py,
                        &node,
                        &self.name,
                        msg.log_summary(),
                        start.elapsed().as_nanos() as u64,
                        "log_pub",
                    );
                }
                success
            }
            // Force types — POD-optimized send
            TopicType::WrenchStamped(topic) => {
                let msg = message.extract::<PyRef<PyWrenchStamped>>(py)?.inner;
                let topic_ref = topic.clone();
                let success = py.detach(|| {
                    topic_ref.write().expect("topic lock poisoned").send(msg);
                    true
                });
                if node.is_some() {
                    use horus::core::LogSummary;
                    log_ipc_event(
                        py,
                        &node,
                        &self.name,
                        msg.log_summary(),
                        start.elapsed().as_nanos() as u64,
                        "log_pub",
                    );
                }
                success
            }
            TopicType::ForceCommand(topic) => {
                let msg = message.extract::<PyRef<PyForceCommand>>(py)?.inner;
                let topic_ref = topic.clone();
                let success = py.detach(|| {
                    topic_ref.write().expect("topic lock poisoned").send(msg);
                    true
                });
                if node.is_some() {
                    use horus::core::LogSummary;
                    log_ipc_event(
                        py,
                        &node,
                        &self.name,
                        msg.log_summary(),
                        start.elapsed().as_nanos() as u64,
                        "log_pub",
                    );
                }
                success
            }
            TopicType::ContactInfo(topic) => {
                let msg = message.extract::<PyRef<PyContactInfo>>(py)?.inner;
                let topic_ref = topic.clone();
                let success = py.detach(|| {
                    topic_ref.write().expect("topic lock poisoned").send(msg);
                    true
                });
                if node.is_some() {
                    use horus::core::LogSummary;
                    log_ipc_event(
                        py,
                        &node,
                        &self.name,
                        msg.log_summary(),
                        start.elapsed().as_nanos() as u64,
                        "log_pub",
                    );
                }
                success
            }
            // Navigation types — POD-optimized send
            TopicType::NavGoal(topic) => {
                let msg = message.extract::<PyRef<PyNavGoal>>(py)?.inner;
                let topic_ref = topic.clone();
                let success = py.detach(|| {
                    topic_ref.write().expect("topic lock poisoned").send(msg);
                    true
                });
                if node.is_some() {
                    use horus::core::LogSummary;
                    log_ipc_event(
                        py,
                        &node,
                        &self.name,
                        msg.log_summary(),
                        start.elapsed().as_nanos() as u64,
                        "log_pub",
                    );
                }
                success
            }
            TopicType::GoalResult(topic) => {
                let msg = message.extract::<PyRef<PyGoalResult>>(py)?.inner;
                let topic_ref = topic.clone();
                let success = py.detach(|| {
                    topic_ref.write().expect("topic lock poisoned").send(msg);
                    true
                });
                if node.is_some() {
                    use horus::core::LogSummary;
                    log_ipc_event(
                        py,
                        &node,
                        &self.name,
                        msg.log_summary(),
                        start.elapsed().as_nanos() as u64,
                        "log_pub",
                    );
                }
                success
            }
            TopicType::PathPlan(topic) => {
                let msg = message.extract::<PyRef<PyPathPlan>>(py)?.inner;
                let topic_ref = topic.clone();
                let success = py.detach(|| {
                    topic_ref.write().expect("topic lock poisoned").send(msg);
                    true
                });
                if node.is_some() {
                    use horus::core::LogSummary;
                    log_ipc_event(
                        py,
                        &node,
                        &self.name,
                        msg.log_summary(),
                        start.elapsed().as_nanos() as u64,
                        "log_pub",
                    );
                }
                success
            }
            // Input types — POD-optimized send
            TopicType::JoystickInput(topic) => {
                let msg = message.extract::<PyRef<PyJoystickInput>>(py)?.inner;
                let topic_ref = topic.clone();
                let success = py.detach(|| {
                    topic_ref.write().expect("topic lock poisoned").send(msg);
                    true
                });
                if node.is_some() {
                    use horus::core::LogSummary;
                    log_ipc_event(
                        py,
                        &node,
                        &self.name,
                        msg.log_summary(),
                        start.elapsed().as_nanos() as u64,
                        "log_pub",
                    );
                }
                success
            }
            TopicType::KeyboardInput(topic) => {
                let msg = message.extract::<PyRef<PyKeyboardInput>>(py)?.inner;
                let topic_ref = topic.clone();
                let success = py.detach(|| {
                    topic_ref.write().expect("topic lock poisoned").send(msg);
                    true
                });
                if node.is_some() {
                    use horus::core::LogSummary;
                    log_ipc_event(
                        py,
                        &node,
                        &self.name,
                        msg.log_summary(),
                        start.elapsed().as_nanos() as u64,
                        "log_pub",
                    );
                }
                success
            }
            // Detection/Perception types — POD-optimized send
            TopicType::BoundingBox2D(topic) => {
                let msg = message.extract::<PyRef<PyBoundingBox2DMsg>>(py)?.inner;
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
                        format!("{:?}", msg),
                        start.elapsed().as_nanos() as u64,
                        "log_pub",
                    );
                }
                success
            }
            TopicType::BoundingBox3D(topic) => {
                let msg = message.extract::<PyRef<PyBoundingBox3D>>(py)?.inner;
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
                        format!("{:?}", msg),
                        start.elapsed().as_nanos() as u64,
                        "log_pub",
                    );
                }
                success
            }
            TopicType::Detection(topic) => {
                let msg = message.extract::<PyRef<PyDetectionMsg>>(py)?.inner;
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
                        format!("{:?}", msg),
                        start.elapsed().as_nanos() as u64,
                        "log_pub",
                    );
                }
                success
            }
            TopicType::Detection3D(topic) => {
                let msg = message.extract::<PyRef<PyDetection3D>>(py)?.inner;
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
                        format!("{:?}", msg),
                        start.elapsed().as_nanos() as u64,
                        "log_pub",
                    );
                }
                success
            }
            TopicType::SegmentationMask(topic) => {
                let msg = message.extract::<PyRef<PySegmentationMask>>(py)?.inner;
                let topic_ref = topic.clone();
                let success = py.detach(|| {
                    topic_ref.write().expect("topic lock poisoned").send(msg);
                    true
                });
                if node.is_some() {
                    use horus::core::LogSummary;
                    log_ipc_event(
                        py,
                        &node,
                        &self.name,
                        msg.log_summary(),
                        start.elapsed().as_nanos() as u64,
                        "log_pub",
                    );
                }
                success
            }
            // Tracking types — POD-optimized send
            TopicType::TrackedObject(topic) => {
                let msg = message.extract::<PyRef<PyTrackedObjectMsg>>(py)?.inner;
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
                        format!("{:?}", msg),
                        start.elapsed().as_nanos() as u64,
                        "log_pub",
                    );
                }
                success
            }
            TopicType::TrackingHeader(topic) => {
                let msg = message.extract::<PyRef<PyTrackingHeader>>(py)?.inner;
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
                        format!("{:?}", msg),
                        start.elapsed().as_nanos() as u64,
                        "log_pub",
                    );
                }
                success
            }
            // Landmark types — POD-optimized send
            TopicType::Landmark(topic) => {
                let msg = message.extract::<PyRef<PyLandmarkMsg>>(py)?.inner;
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
                        format!("{:?}", msg),
                        start.elapsed().as_nanos() as u64,
                        "log_pub",
                    );
                }
                success
            }
            TopicType::Landmark3D(topic) => {
                let msg = message.extract::<PyRef<PyLandmark3D>>(py)?.inner;
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
                        format!("{:?}", msg),
                        start.elapsed().as_nanos() as u64,
                        "log_pub",
                    );
                }
                success
            }
            TopicType::LandmarkArray(topic) => {
                let msg = message.extract::<PyRef<PyLandmarkArray>>(py)?.inner;
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
                        format!("{:?}", msg),
                        start.elapsed().as_nanos() as u64,
                        "log_pub",
                    );
                }
                success
            }
            // Perception helper types — serde-based send
            TopicType::PointField(topic) => {
                let msg = message.extract::<PyRef<PyPointField>>(py)?.inner;
                let topic_ref = topic.clone();
                let success = py.detach(|| {
                    topic_ref.write().expect("topic lock poisoned").send(msg);
                    true
                });
                if node.is_some() {
                    use horus::core::LogSummary;
                    log_ipc_event(
                        py,
                        &node,
                        &self.name,
                        msg.log_summary(),
                        start.elapsed().as_nanos() as u64,
                        "log_pub",
                    );
                }
                success
            }
            TopicType::PlaneDetection(topic) => {
                let msg = message.extract::<PyRef<PyPlaneDetection>>(py)?.inner;
                let topic_ref = topic.clone();
                let success = py.detach(|| {
                    topic_ref.write().expect("topic lock poisoned").send(msg);
                    true
                });
                if node.is_some() {
                    use horus::core::LogSummary;
                    log_ipc_event(
                        py,
                        &node,
                        &self.name,
                        msg.log_summary(),
                        start.elapsed().as_nanos() as u64,
                        "log_pub",
                    );
                }
                success
            }
            TopicType::PlaneArray(topic) => {
                let msg = message.extract::<PyRef<PyPlaneArray>>(py)?.inner.clone();
                let topic_ref = topic.clone();
                let log_msg = {
                    use horus::core::LogSummary;
                    msg.log_summary()
                };
                let success = py.detach(|| {
                    topic_ref.write().expect("topic lock poisoned").send(msg);
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
            // ML types (serde-based, all use .clone())
            TopicType::TensorData(topic) => {
                let msg = message.extract::<PyRef<PyTensorData>>(py)?.inner.clone();
                let topic_ref = topic.clone();
                let log_msg = {
                    use horus::core::LogSummary;
                    msg.log_summary()
                };
                let success = py.detach(|| {
                    topic_ref.write().expect("topic lock poisoned").send(msg);
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
            TopicType::Predictions(topic) => {
                let msg = message.extract::<PyRef<PyPredictions>>(py)?.inner.clone();
                let topic_ref = topic.clone();
                let log_msg = {
                    use horus::core::LogSummary;
                    msg.log_summary()
                };
                let success = py.detach(|| {
                    topic_ref.write().expect("topic lock poisoned").send(msg);
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
            TopicType::InferenceMetrics(topic) => {
                let msg = message
                    .extract::<PyRef<PyInferenceMetrics>>(py)?
                    .inner
                    .clone();
                let topic_ref = topic.clone();
                let log_msg = {
                    use horus::core::LogSummary;
                    msg.log_summary()
                };
                let success = py.detach(|| {
                    topic_ref.write().expect("topic lock poisoned").send(msg);
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
            TopicType::ModelInfo(topic) => {
                let msg = message.extract::<PyRef<PyModelInfo>>(py)?.inner.clone();
                let topic_ref = topic.clone();
                let log_msg = format!("{:?}", msg);
                let success = py.detach(|| {
                    topic_ref.write().expect("topic lock poisoned").send(msg);
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
            TopicType::FeatureVector(topic) => {
                let msg = message.extract::<PyRef<PyFeatureVector>>(py)?.inner.clone();
                let topic_ref = topic.clone();
                let log_msg = format!("{:?}", msg);
                let success = py.detach(|| {
                    topic_ref.write().expect("topic lock poisoned").send(msg);
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
            TopicType::Classification(topic) => {
                let msg = message
                    .extract::<PyRef<PyClassification>>(py)?
                    .inner
                    .clone();
                let topic_ref = topic.clone();
                let log_msg = format!("{:?}", msg);
                let success = py.detach(|| {
                    topic_ref.write().expect("topic lock poisoned").send(msg);
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
            TopicType::ChatMessage(topic) => {
                let msg = message.extract::<PyRef<PyChatMessage>>(py)?.inner.clone();
                let topic_ref = topic.clone();
                let log_msg = {
                    use horus::core::LogSummary;
                    msg.log_summary()
                };
                let success = py.detach(|| {
                    topic_ref.write().expect("topic lock poisoned").send(msg);
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
            TopicType::LLMRequest(topic) => {
                let msg = message.extract::<PyRef<PyLLMRequest>>(py)?.inner.clone();
                let topic_ref = topic.clone();
                let log_msg = {
                    use horus::core::LogSummary;
                    msg.log_summary()
                };
                let success = py.detach(|| {
                    topic_ref.write().expect("topic lock poisoned").send(msg);
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
            TopicType::LLMResponse(topic) => {
                let msg = message.extract::<PyRef<PyLLMResponse>>(py)?.inner.clone();
                let topic_ref = topic.clone();
                let log_msg = {
                    use horus::core::LogSummary;
                    msg.log_summary()
                };
                let success = py.detach(|| {
                    topic_ref.write().expect("topic lock poisoned").send(msg);
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
            TopicType::TrainingMetrics(topic) => {
                let msg = message
                    .extract::<PyRef<PyTrainingMetrics>>(py)?
                    .inner
                    .clone();
                let topic_ref = topic.clone();
                let log_msg = format!("{:?}", msg);
                let success = py.detach(|| {
                    topic_ref.write().expect("topic lock poisoned").send(msg);
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
            TopicType::MlTrajectoryPoint(topic) => {
                let msg = message
                    .extract::<PyRef<PyMlTrajectoryPoint>>(py)?
                    .inner
                    .clone();
                let topic_ref = topic.clone();
                let log_msg = format!("{:?}", msg);
                let success = py.detach(|| {
                    topic_ref.write().expect("topic lock poisoned").send(msg);
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
            TopicType::DeploymentConfig(topic) => {
                let msg = message
                    .extract::<PyRef<PyDeploymentConfig>>(py)?
                    .inner
                    .clone();
                let topic_ref = topic.clone();
                let log_msg = format!("{:?}", msg);
                let success = py.detach(|| {
                    topic_ref.write().expect("topic lock poisoned").send(msg);
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
            // Vision types (serde-based, all use .clone())
            TopicType::CompressedImage(topic) => {
                let msg = message
                    .extract::<PyRef<PyCompressedImage>>(py)?
                    .inner
                    .clone();
                let topic_ref = topic.clone();
                let log_msg = {
                    use horus::core::LogSummary;
                    msg.log_summary()
                };
                let success = py.detach(|| {
                    topic_ref.write().expect("topic lock poisoned").send(msg);
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
            TopicType::CameraInfo(topic) => {
                let msg = message.extract::<PyRef<PyCameraInfo>>(py)?.inner;
                let topic_ref = topic.clone();
                let log_msg = {
                    use horus::core::LogSummary;
                    msg.log_summary()
                };
                let success = py.detach(|| {
                    topic_ref.write().expect("topic lock poisoned").send(msg);
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
            TopicType::RegionOfInterest(topic) => {
                let msg = message.extract::<PyRef<PyRegionOfInterest>>(py)?.inner;
                let topic_ref = topic.clone();
                let log_msg = {
                    use horus::core::LogSummary;
                    msg.log_summary()
                };
                let success = py.detach(|| {
                    topic_ref.write().expect("topic lock poisoned").send(msg);
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
            TopicType::StereoInfo(topic) => {
                let msg = message.extract::<PyRef<PyStereoInfo>>(py)?.inner;
                let topic_ref = topic.clone();
                let log_msg = {
                    use horus::core::LogSummary;
                    msg.log_summary()
                };
                let success = py.detach(|| {
                    topic_ref.write().expect("topic lock poisoned").send(msg);
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
            // Force types (additional Pod)
            TopicType::TactileArray(topic) => {
                let msg = message.extract::<PyRef<PyTactileArray>>(py)?.inner;
                let topic_ref = topic.clone();
                let log_msg = {
                    use horus::core::LogSummary;
                    msg.log_summary()
                };
                let success = py.detach(|| {
                    topic_ref.write().expect("topic lock poisoned").send(msg);
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
            TopicType::ImpedanceParameters(topic) => {
                let msg = message.extract::<PyRef<PyImpedanceParameters>>(py)?.inner;
                let topic_ref = topic.clone();
                let log_msg = {
                    use horus::core::LogSummary;
                    msg.log_summary()
                };
                let success = py.detach(|| {
                    topic_ref.write().expect("topic lock poisoned").send(msg);
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
            TopicType::HapticFeedback(topic) => {
                let msg = message.extract::<PyRef<PyHapticFeedback>>(py)?.inner;
                let topic_ref = topic.clone();
                let log_msg = {
                    use horus::core::LogSummary;
                    msg.log_summary()
                };
                let success = py.detach(|| {
                    topic_ref.write().expect("topic lock poisoned").send(msg);
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
            // Diagnostics types (additional Pod)
            TopicType::DiagnosticValue(topic) => {
                let msg = message.extract::<PyRef<PyDiagnosticValue>>(py)?.inner;
                let topic_ref = topic.clone();
                let log_msg = {
                    use horus::core::LogSummary;
                    msg.log_summary()
                };
                let success = py.detach(|| {
                    topic_ref.write().expect("topic lock poisoned").send(msg);
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
            TopicType::DiagnosticReport(topic) => {
                let msg = message.extract::<PyRef<PyDiagnosticReport>>(py)?.inner;
                let topic_ref = topic.clone();
                let log_msg = {
                    use horus::core::LogSummary;
                    msg.log_summary()
                };
                let success = py.detach(|| {
                    topic_ref.write().expect("topic lock poisoned").send(msg);
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
            TopicType::NodeHeartbeat(topic) => {
                let msg = message.extract::<PyRef<PyNodeHeartbeat>>(py)?.inner;
                let topic_ref = topic.clone();
                let log_msg = {
                    use horus::core::LogSummary;
                    msg.log_summary()
                };
                let success = py.detach(|| {
                    topic_ref.write().expect("topic lock poisoned").send(msg);
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
            TopicType::SafetyStatus(topic) => {
                let msg = message.extract::<PyRef<PySafetyStatus>>(py)?.inner;
                let topic_ref = topic.clone();
                let log_msg = {
                    use horus::core::LogSummary;
                    msg.log_summary()
                };
                let success = py.detach(|| {
                    topic_ref.write().expect("topic lock poisoned").send(msg);
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
            // Navigation types (additional)
            TopicType::Waypoint(topic) => {
                let msg = message.extract::<PyRef<PyWaypoint>>(py)?.inner;
                let topic_ref = topic.clone();
                let log_msg = {
                    use horus::core::LogSummary;
                    msg.log_summary()
                };
                let success = py.detach(|| {
                    topic_ref.write().expect("topic lock poisoned").send(msg);
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
            TopicType::NavPath(topic) => {
                let msg = message.extract::<PyRef<PyNavPath>>(py)?.inner;
                let topic_ref = topic.clone();
                let log_msg = {
                    use horus::core::LogSummary;
                    msg.log_summary()
                };
                let success = py.detach(|| {
                    topic_ref.write().expect("topic lock poisoned").send(msg);
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
            TopicType::VelocityObstacle(topic) => {
                let msg = message.extract::<PyRef<PyVelocityObstacle>>(py)?.inner;
                let topic_ref = topic.clone();
                let log_msg = {
                    use horus::core::LogSummary;
                    msg.log_summary()
                };
                let success = py.detach(|| {
                    topic_ref.write().expect("topic lock poisoned").send(msg);
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
            TopicType::VelocityObstacles(topic) => {
                let msg = message.extract::<PyRef<PyVelocityObstacles>>(py)?.inner;
                let topic_ref = topic.clone();
                let log_msg = {
                    use horus::core::LogSummary;
                    msg.log_summary()
                };
                let success = py.detach(|| {
                    topic_ref.write().expect("topic lock poisoned").send(msg);
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
            TopicType::OccupancyGrid(topic) => {
                let msg = message.extract::<PyRef<PyOccupancyGrid>>(py)?.inner.clone();
                let topic_ref = topic.clone();
                let log_msg = {
                    use horus::core::LogSummary;
                    msg.log_summary()
                };
                let success = py.detach(|| {
                    topic_ref.write().expect("topic lock poisoned").send(msg);
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
            TopicType::CostMap(topic) => {
                let msg = message.extract::<PyRef<PyCostMap>>(py)?.inner.clone();
                let topic_ref = topic.clone();
                let log_msg = {
                    use horus::core::LogSummary;
                    msg.log_summary()
                };
                let success = py.detach(|| {
                    topic_ref.write().expect("topic lock poisoned").send(msg);
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
                let msg_opt = py.detach(|| topic_ref.read().expect("topic lock poisoned").recv());
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
                let msg_opt = py.detach(|| topic_ref.read().expect("topic lock poisoned").recv());
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
                let msg_opt = py.detach(|| topic_ref.read().expect("topic lock poisoned").recv());
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
                let msg_opt = py.detach(|| topic_ref.read().expect("topic lock poisoned").recv());
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
                    Ok(Some(Py::new(py, PyOdometry { inner: odom })?.into_any()))
                } else {
                    Ok(None)
                }
            }
            TopicType::LaserScan(topic) => {
                let topic_ref = topic.clone();
                let msg_opt = py.detach(|| topic_ref.read().expect("topic lock poisoned").recv());
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
                    Ok(Some(Py::new(py, PyLaserScan { inner: scan })?.into_any()))
                } else {
                    Ok(None)
                }
            }
            TopicType::Pose3D(topic) => {
                let topic_ref = topic.clone();
                let msg_opt = py.detach(|| topic_ref.read().expect("topic lock poisoned").recv());
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
                    Ok(Some(Py::new(py, PyPose3D { inner: pose })?.into_any()))
                } else {
                    Ok(None)
                }
            }
            TopicType::JointState(topic) => {
                let topic_ref = topic.clone();
                let msg_opt = py.detach(|| topic_ref.read().expect("topic lock poisoned").recv());
                if let Some(js) = msg_opt {
                    if node.is_some() {
                        use horus::core::LogSummary;
                        log_ipc_event(
                            py,
                            &node,
                            &self.name,
                            js.log_summary(),
                            start.elapsed().as_nanos() as u64,
                            "log_sub",
                        );
                    }
                    Ok(Some(Py::new(py, PyJointState { inner: js })?.into_any()))
                } else {
                    Ok(None)
                }
            }
            TopicType::Clock(topic) => {
                let topic_ref = topic.clone();
                let msg_opt = py.detach(|| topic_ref.read().expect("topic lock poisoned").recv());
                if let Some(clk) = msg_opt {
                    if node.is_some() {
                        use horus::core::LogSummary;
                        log_ipc_event(
                            py,
                            &node,
                            &self.name,
                            clk.log_summary(),
                            start.elapsed().as_nanos() as u64,
                            "log_sub",
                        );
                    }
                    Ok(Some(Py::new(py, PyClock { inner: clk })?.into_any()))
                } else {
                    Ok(None)
                }
            }
            TopicType::TimeReference(topic) => {
                let topic_ref = topic.clone();
                let msg_opt = py.detach(|| topic_ref.read().expect("topic lock poisoned").recv());
                if let Some(tr) = msg_opt {
                    if node.is_some() {
                        use horus::core::LogSummary;
                        log_ipc_event(
                            py,
                            &node,
                            &self.name,
                            tr.log_summary(),
                            start.elapsed().as_nanos() as u64,
                            "log_sub",
                        );
                    }
                    Ok(Some(Py::new(py, PyTimeReference { inner: tr })?.into_any()))
                } else {
                    Ok(None)
                }
            }
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
            // Geometry types — POD-optimized recv
            TopicType::Twist(topic) => {
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
                    Ok(Some(Py::new(py, PyTwist { inner: msg })?.into_any()))
                } else {
                    Ok(None)
                }
            }
            TopicType::Vector3(topic) => {
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
                    Ok(Some(Py::new(py, PyVector3 { inner: msg })?.into_any()))
                } else {
                    Ok(None)
                }
            }
            TopicType::Point3(topic) => {
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
                    Ok(Some(Py::new(py, PyPoint3 { inner: msg })?.into_any()))
                } else {
                    Ok(None)
                }
            }
            TopicType::Quaternion(topic) => {
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
                    Ok(Some(Py::new(py, PyQuaternion { inner: msg })?.into_any()))
                } else {
                    Ok(None)
                }
            }
            TopicType::TransformStamped(topic) => {
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
                    Ok(Some(
                        Py::new(py, PyTransformStamped { inner: msg })?.into_any(),
                    ))
                } else {
                    Ok(None)
                }
            }
            TopicType::PoseStamped(topic) => {
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
                    Ok(Some(Py::new(py, PyPoseStamped { inner: msg })?.into_any()))
                } else {
                    Ok(None)
                }
            }
            TopicType::PoseWithCovariance(topic) => {
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
                    Ok(Some(
                        Py::new(py, PyPoseWithCovariance { inner: msg })?.into_any(),
                    ))
                } else {
                    Ok(None)
                }
            }
            TopicType::TwistWithCovariance(topic) => {
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
                    Ok(Some(
                        Py::new(py, PyTwistWithCovariance { inner: msg })?.into_any(),
                    ))
                } else {
                    Ok(None)
                }
            }
            TopicType::Accel(topic) => {
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
                    Ok(Some(Py::new(py, PyAccel { inner: msg })?.into_any()))
                } else {
                    Ok(None)
                }
            }
            TopicType::AccelStamped(topic) => {
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
                    Ok(Some(Py::new(py, PyAccelStamped { inner: msg })?.into_any()))
                } else {
                    Ok(None)
                }
            }
            // Control types — POD-optimized recv
            TopicType::MotorCommand(topic) => {
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
                    Ok(Some(Py::new(py, PyMotorCommand { inner: msg })?.into_any()))
                } else {
                    Ok(None)
                }
            }
            TopicType::ServoCommand(topic) => {
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
                    Ok(Some(Py::new(py, PyServoCommand { inner: msg })?.into_any()))
                } else {
                    Ok(None)
                }
            }
            TopicType::DifferentialDriveCommand(topic) => {
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
                    Ok(Some(
                        Py::new(py, PyDifferentialDriveCommand { inner: msg })?.into_any(),
                    ))
                } else {
                    Ok(None)
                }
            }
            TopicType::PidConfig(topic) => {
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
                    Ok(Some(Py::new(py, PyPidConfig { inner: msg })?.into_any()))
                } else {
                    Ok(None)
                }
            }
            TopicType::TrajectoryPoint(topic) => {
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
                    Ok(Some(
                        Py::new(py, PyTrajectoryPoint { inner: msg })?.into_any(),
                    ))
                } else {
                    Ok(None)
                }
            }
            TopicType::JointCommand(topic) => {
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
                    Ok(Some(Py::new(py, PyJointCommand { inner: msg })?.into_any()))
                } else {
                    Ok(None)
                }
            }
            TopicType::PwmCommand(topic) => {
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
                    Ok(Some(Py::new(py, PyPwmCommand { inner: msg })?.into_any()))
                } else {
                    Ok(None)
                }
            }
            TopicType::StepperCommand(topic) => {
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
                    Ok(Some(
                        Py::new(py, PyStepperCommand { inner: msg })?.into_any(),
                    ))
                } else {
                    Ok(None)
                }
            }
            // Sensor types — POD-optimized recv
            TopicType::RangeSensor(topic) => {
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
                    Ok(Some(Py::new(py, PyRangeSensor { inner: msg })?.into_any()))
                } else {
                    Ok(None)
                }
            }
            TopicType::BatteryState(topic) => {
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
                    Ok(Some(Py::new(py, PyBatteryState { inner: msg })?.into_any()))
                } else {
                    Ok(None)
                }
            }
            TopicType::NavSatFix(topic) => {
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
                    Ok(Some(Py::new(py, PyNavSatFix { inner: msg })?.into_any()))
                } else {
                    Ok(None)
                }
            }
            TopicType::MagneticField(topic) => {
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
                    Ok(Some(
                        Py::new(py, PyMagneticField { inner: msg })?.into_any(),
                    ))
                } else {
                    Ok(None)
                }
            }
            TopicType::Temperature(topic) => {
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
                    Ok(Some(Py::new(py, PyTemperature { inner: msg })?.into_any()))
                } else {
                    Ok(None)
                }
            }
            TopicType::FluidPressure(topic) => {
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
                    Ok(Some(
                        Py::new(py, PyFluidPressure { inner: msg })?.into_any(),
                    ))
                } else {
                    Ok(None)
                }
            }
            TopicType::Illuminance(topic) => {
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
                    Ok(Some(Py::new(py, PyIlluminance { inner: msg })?.into_any()))
                } else {
                    Ok(None)
                }
            }
            // Diagnostics types — POD-optimized recv
            TopicType::Heartbeat(topic) => {
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
                    Ok(Some(Py::new(py, PyHeartbeat { inner: msg })?.into_any()))
                } else {
                    Ok(None)
                }
            }
            TopicType::DiagnosticStatus(topic) => {
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
                    Ok(Some(
                        Py::new(py, PyDiagnosticStatus { inner: msg })?.into_any(),
                    ))
                } else {
                    Ok(None)
                }
            }
            TopicType::EmergencyStop(topic) => {
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
                    Ok(Some(
                        Py::new(py, PyEmergencyStop { inner: msg })?.into_any(),
                    ))
                } else {
                    Ok(None)
                }
            }
            TopicType::ResourceUsage(topic) => {
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
                    Ok(Some(
                        Py::new(py, PyResourceUsage { inner: msg })?.into_any(),
                    ))
                } else {
                    Ok(None)
                }
            }
            // Force types — POD-optimized recv
            TopicType::WrenchStamped(topic) => {
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
                    Ok(Some(
                        Py::new(py, PyWrenchStamped { inner: msg })?.into_any(),
                    ))
                } else {
                    Ok(None)
                }
            }
            TopicType::ForceCommand(topic) => {
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
                    Ok(Some(Py::new(py, PyForceCommand { inner: msg })?.into_any()))
                } else {
                    Ok(None)
                }
            }
            TopicType::ContactInfo(topic) => {
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
                    Ok(Some(Py::new(py, PyContactInfo { inner: msg })?.into_any()))
                } else {
                    Ok(None)
                }
            }
            // Navigation types — POD-optimized recv
            TopicType::NavGoal(topic) => {
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
                    Ok(Some(Py::new(py, PyNavGoal { inner: msg })?.into_any()))
                } else {
                    Ok(None)
                }
            }
            TopicType::GoalResult(topic) => {
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
                    Ok(Some(Py::new(py, PyGoalResult { inner: msg })?.into_any()))
                } else {
                    Ok(None)
                }
            }
            TopicType::PathPlan(topic) => {
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
                    Ok(Some(Py::new(py, PyPathPlan { inner: msg })?.into_any()))
                } else {
                    Ok(None)
                }
            }
            // Input types — POD-optimized recv
            TopicType::JoystickInput(topic) => {
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
                    Ok(Some(
                        Py::new(py, PyJoystickInput { inner: msg })?.into_any(),
                    ))
                } else {
                    Ok(None)
                }
            }
            TopicType::KeyboardInput(topic) => {
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
                    Ok(Some(
                        Py::new(py, PyKeyboardInput { inner: msg })?.into_any(),
                    ))
                } else {
                    Ok(None)
                }
            }
            // Detection/Perception types — POD-optimized recv
            TopicType::BoundingBox2D(topic) => {
                let topic_ref = topic.clone();
                let msg_opt = py.detach(|| topic_ref.read().expect("topic lock poisoned").recv());
                if let Some(msg) = msg_opt {
                    if node.is_some() {
                        log_ipc_event(
                            py,
                            &node,
                            &self.name,
                            format!("{:?}", msg),
                            start.elapsed().as_nanos() as u64,
                            "log_sub",
                        );
                    }
                    Ok(Some(
                        Py::new(py, PyBoundingBox2DMsg { inner: msg })?.into_any(),
                    ))
                } else {
                    Ok(None)
                }
            }
            TopicType::BoundingBox3D(topic) => {
                let topic_ref = topic.clone();
                let msg_opt = py.detach(|| topic_ref.read().expect("topic lock poisoned").recv());
                if let Some(msg) = msg_opt {
                    if node.is_some() {
                        log_ipc_event(
                            py,
                            &node,
                            &self.name,
                            format!("{:?}", msg),
                            start.elapsed().as_nanos() as u64,
                            "log_sub",
                        );
                    }
                    Ok(Some(
                        Py::new(py, PyBoundingBox3D { inner: msg })?.into_any(),
                    ))
                } else {
                    Ok(None)
                }
            }
            TopicType::Detection(topic) => {
                let topic_ref = topic.clone();
                let msg_opt = py.detach(|| topic_ref.read().expect("topic lock poisoned").recv());
                if let Some(msg) = msg_opt {
                    if node.is_some() {
                        log_ipc_event(
                            py,
                            &node,
                            &self.name,
                            format!("{:?}", msg),
                            start.elapsed().as_nanos() as u64,
                            "log_sub",
                        );
                    }
                    Ok(Some(Py::new(py, PyDetectionMsg { inner: msg })?.into_any()))
                } else {
                    Ok(None)
                }
            }
            TopicType::Detection3D(topic) => {
                let topic_ref = topic.clone();
                let msg_opt = py.detach(|| topic_ref.read().expect("topic lock poisoned").recv());
                if let Some(msg) = msg_opt {
                    if node.is_some() {
                        log_ipc_event(
                            py,
                            &node,
                            &self.name,
                            format!("{:?}", msg),
                            start.elapsed().as_nanos() as u64,
                            "log_sub",
                        );
                    }
                    Ok(Some(Py::new(py, PyDetection3D { inner: msg })?.into_any()))
                } else {
                    Ok(None)
                }
            }
            TopicType::SegmentationMask(topic) => {
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
                    Ok(Some(
                        Py::new(py, PySegmentationMask { inner: msg })?.into_any(),
                    ))
                } else {
                    Ok(None)
                }
            }
            // Tracking types — POD-optimized recv
            TopicType::TrackedObject(topic) => {
                let topic_ref = topic.clone();
                let msg_opt = py.detach(|| topic_ref.read().expect("topic lock poisoned").recv());
                if let Some(msg) = msg_opt {
                    if node.is_some() {
                        log_ipc_event(
                            py,
                            &node,
                            &self.name,
                            format!("{:?}", msg),
                            start.elapsed().as_nanos() as u64,
                            "log_sub",
                        );
                    }
                    Ok(Some(
                        Py::new(py, PyTrackedObjectMsg { inner: msg })?.into_any(),
                    ))
                } else {
                    Ok(None)
                }
            }
            TopicType::TrackingHeader(topic) => {
                let topic_ref = topic.clone();
                let msg_opt = py.detach(|| topic_ref.read().expect("topic lock poisoned").recv());
                if let Some(msg) = msg_opt {
                    if node.is_some() {
                        log_ipc_event(
                            py,
                            &node,
                            &self.name,
                            format!("{:?}", msg),
                            start.elapsed().as_nanos() as u64,
                            "log_sub",
                        );
                    }
                    Ok(Some(
                        Py::new(py, PyTrackingHeader { inner: msg })?.into_any(),
                    ))
                } else {
                    Ok(None)
                }
            }
            // Landmark types — POD-optimized recv
            TopicType::Landmark(topic) => {
                let topic_ref = topic.clone();
                let msg_opt = py.detach(|| topic_ref.read().expect("topic lock poisoned").recv());
                if let Some(msg) = msg_opt {
                    if node.is_some() {
                        log_ipc_event(
                            py,
                            &node,
                            &self.name,
                            format!("{:?}", msg),
                            start.elapsed().as_nanos() as u64,
                            "log_sub",
                        );
                    }
                    Ok(Some(Py::new(py, PyLandmarkMsg { inner: msg })?.into_any()))
                } else {
                    Ok(None)
                }
            }
            TopicType::Landmark3D(topic) => {
                let topic_ref = topic.clone();
                let msg_opt = py.detach(|| topic_ref.read().expect("topic lock poisoned").recv());
                if let Some(msg) = msg_opt {
                    if node.is_some() {
                        log_ipc_event(
                            py,
                            &node,
                            &self.name,
                            format!("{:?}", msg),
                            start.elapsed().as_nanos() as u64,
                            "log_sub",
                        );
                    }
                    Ok(Some(Py::new(py, PyLandmark3D { inner: msg })?.into_any()))
                } else {
                    Ok(None)
                }
            }
            TopicType::LandmarkArray(topic) => {
                let topic_ref = topic.clone();
                let msg_opt = py.detach(|| topic_ref.read().expect("topic lock poisoned").recv());
                if let Some(msg) = msg_opt {
                    if node.is_some() {
                        log_ipc_event(
                            py,
                            &node,
                            &self.name,
                            format!("{:?}", msg),
                            start.elapsed().as_nanos() as u64,
                            "log_sub",
                        );
                    }
                    Ok(Some(
                        Py::new(py, PyLandmarkArray { inner: msg })?.into_any(),
                    ))
                } else {
                    Ok(None)
                }
            }
            // Perception helper types — serde-based recv
            TopicType::PointField(topic) => {
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
                    Ok(Some(Py::new(py, PyPointField { inner: msg })?.into_any()))
                } else {
                    Ok(None)
                }
            }
            TopicType::PlaneDetection(topic) => {
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
                    Ok(Some(
                        Py::new(py, PyPlaneDetection { inner: msg })?.into_any(),
                    ))
                } else {
                    Ok(None)
                }
            }
            TopicType::PlaneArray(topic) => {
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
                    Ok(Some(Py::new(py, PyPlaneArray { inner: msg })?.into_any()))
                } else {
                    Ok(None)
                }
            }
            // ML types (serde-based)
            TopicType::TensorData(topic) => {
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
                    Ok(Some(Py::new(py, PyTensorData { inner: msg })?.into_any()))
                } else {
                    Ok(None)
                }
            }
            TopicType::Predictions(topic) => {
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
                    Ok(Some(Py::new(py, PyPredictions { inner: msg })?.into_any()))
                } else {
                    Ok(None)
                }
            }
            TopicType::InferenceMetrics(topic) => {
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
                    Ok(Some(
                        Py::new(py, PyInferenceMetrics { inner: msg })?.into_any(),
                    ))
                } else {
                    Ok(None)
                }
            }
            TopicType::ModelInfo(topic) => {
                let topic_ref = topic.clone();
                let msg_opt = py.detach(|| topic_ref.read().expect("topic lock poisoned").recv());
                if let Some(msg) = msg_opt {
                    if node.is_some() {
                        log_ipc_event(
                            py,
                            &node,
                            &self.name,
                            format!("{:?}", msg),
                            start.elapsed().as_nanos() as u64,
                            "log_sub",
                        );
                    }
                    Ok(Some(Py::new(py, PyModelInfo { inner: msg })?.into_any()))
                } else {
                    Ok(None)
                }
            }
            TopicType::FeatureVector(topic) => {
                let topic_ref = topic.clone();
                let msg_opt = py.detach(|| topic_ref.read().expect("topic lock poisoned").recv());
                if let Some(msg) = msg_opt {
                    if node.is_some() {
                        log_ipc_event(
                            py,
                            &node,
                            &self.name,
                            format!("{:?}", msg),
                            start.elapsed().as_nanos() as u64,
                            "log_sub",
                        );
                    }
                    Ok(Some(
                        Py::new(py, PyFeatureVector { inner: msg })?.into_any(),
                    ))
                } else {
                    Ok(None)
                }
            }
            TopicType::Classification(topic) => {
                let topic_ref = topic.clone();
                let msg_opt = py.detach(|| topic_ref.read().expect("topic lock poisoned").recv());
                if let Some(msg) = msg_opt {
                    if node.is_some() {
                        log_ipc_event(
                            py,
                            &node,
                            &self.name,
                            format!("{:?}", msg),
                            start.elapsed().as_nanos() as u64,
                            "log_sub",
                        );
                    }
                    Ok(Some(
                        Py::new(py, PyClassification { inner: msg })?.into_any(),
                    ))
                } else {
                    Ok(None)
                }
            }
            TopicType::ChatMessage(topic) => {
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
                    Ok(Some(Py::new(py, PyChatMessage { inner: msg })?.into_any()))
                } else {
                    Ok(None)
                }
            }
            TopicType::LLMRequest(topic) => {
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
                    Ok(Some(Py::new(py, PyLLMRequest { inner: msg })?.into_any()))
                } else {
                    Ok(None)
                }
            }
            TopicType::LLMResponse(topic) => {
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
                    Ok(Some(Py::new(py, PyLLMResponse { inner: msg })?.into_any()))
                } else {
                    Ok(None)
                }
            }
            TopicType::TrainingMetrics(topic) => {
                let topic_ref = topic.clone();
                let msg_opt = py.detach(|| topic_ref.read().expect("topic lock poisoned").recv());
                if let Some(msg) = msg_opt {
                    if node.is_some() {
                        log_ipc_event(
                            py,
                            &node,
                            &self.name,
                            format!("{:?}", msg),
                            start.elapsed().as_nanos() as u64,
                            "log_sub",
                        );
                    }
                    Ok(Some(
                        Py::new(py, PyTrainingMetrics { inner: msg })?.into_any(),
                    ))
                } else {
                    Ok(None)
                }
            }
            TopicType::MlTrajectoryPoint(topic) => {
                let topic_ref = topic.clone();
                let msg_opt = py.detach(|| topic_ref.read().expect("topic lock poisoned").recv());
                if let Some(msg) = msg_opt {
                    if node.is_some() {
                        log_ipc_event(
                            py,
                            &node,
                            &self.name,
                            format!("{:?}", msg),
                            start.elapsed().as_nanos() as u64,
                            "log_sub",
                        );
                    }
                    Ok(Some(
                        Py::new(py, PyMlTrajectoryPoint { inner: msg })?.into_any(),
                    ))
                } else {
                    Ok(None)
                }
            }
            TopicType::DeploymentConfig(topic) => {
                let topic_ref = topic.clone();
                let msg_opt = py.detach(|| topic_ref.read().expect("topic lock poisoned").recv());
                if let Some(msg) = msg_opt {
                    if node.is_some() {
                        log_ipc_event(
                            py,
                            &node,
                            &self.name,
                            format!("{:?}", msg),
                            start.elapsed().as_nanos() as u64,
                            "log_sub",
                        );
                    }
                    Ok(Some(
                        Py::new(py, PyDeploymentConfig { inner: msg })?.into_any(),
                    ))
                } else {
                    Ok(None)
                }
            }
            // Vision types (serde-based)
            TopicType::CompressedImage(topic) => {
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
                    Ok(Some(
                        Py::new(py, PyCompressedImage { inner: msg })?.into_any(),
                    ))
                } else {
                    Ok(None)
                }
            }
            TopicType::CameraInfo(topic) => {
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
                    Ok(Some(Py::new(py, PyCameraInfo { inner: msg })?.into_any()))
                } else {
                    Ok(None)
                }
            }
            TopicType::RegionOfInterest(topic) => {
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
                    Ok(Some(
                        Py::new(py, PyRegionOfInterest { inner: msg })?.into_any(),
                    ))
                } else {
                    Ok(None)
                }
            }
            TopicType::StereoInfo(topic) => {
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
                    Ok(Some(Py::new(py, PyStereoInfo { inner: msg })?.into_any()))
                } else {
                    Ok(None)
                }
            }
            // Force types (additional Pod)
            TopicType::TactileArray(topic) => {
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
                    Ok(Some(Py::new(py, PyTactileArray { inner: msg })?.into_any()))
                } else {
                    Ok(None)
                }
            }
            TopicType::ImpedanceParameters(topic) => {
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
                    Ok(Some(
                        Py::new(py, PyImpedanceParameters { inner: msg })?.into_any(),
                    ))
                } else {
                    Ok(None)
                }
            }
            TopicType::HapticFeedback(topic) => {
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
                    Ok(Some(
                        Py::new(py, PyHapticFeedback { inner: msg })?.into_any(),
                    ))
                } else {
                    Ok(None)
                }
            }
            // Diagnostics types (additional Pod)
            TopicType::DiagnosticValue(topic) => {
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
                    Ok(Some(
                        Py::new(py, PyDiagnosticValue { inner: msg })?.into_any(),
                    ))
                } else {
                    Ok(None)
                }
            }
            TopicType::DiagnosticReport(topic) => {
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
                    Ok(Some(
                        Py::new(py, PyDiagnosticReport { inner: msg })?.into_any(),
                    ))
                } else {
                    Ok(None)
                }
            }
            TopicType::NodeHeartbeat(topic) => {
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
                    Ok(Some(
                        Py::new(py, PyNodeHeartbeat { inner: msg })?.into_any(),
                    ))
                } else {
                    Ok(None)
                }
            }
            TopicType::SafetyStatus(topic) => {
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
                    Ok(Some(Py::new(py, PySafetyStatus { inner: msg })?.into_any()))
                } else {
                    Ok(None)
                }
            }
            // Navigation types (additional)
            TopicType::Waypoint(topic) => {
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
                    Ok(Some(Py::new(py, PyWaypoint { inner: msg })?.into_any()))
                } else {
                    Ok(None)
                }
            }
            TopicType::NavPath(topic) => {
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
                    Ok(Some(Py::new(py, PyNavPath { inner: msg })?.into_any()))
                } else {
                    Ok(None)
                }
            }
            TopicType::VelocityObstacle(topic) => {
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
                    Ok(Some(
                        Py::new(py, PyVelocityObstacle { inner: msg })?.into_any(),
                    ))
                } else {
                    Ok(None)
                }
            }
            TopicType::VelocityObstacles(topic) => {
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
                    Ok(Some(
                        Py::new(py, PyVelocityObstacles { inner: msg })?.into_any(),
                    ))
                } else {
                    Ok(None)
                }
            }
            TopicType::OccupancyGrid(topic) => {
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
                    Ok(Some(
                        Py::new(py, PyOccupancyGrid { inner: msg })?.into_any(),
                    ))
                } else {
                    Ok(None)
                }
            }
            TopicType::CostMap(topic) => {
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
                    Ok(Some(Py::new(py, PyCostMap { inner: msg })?.into_any()))
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
            TopicType::PwmCommand(t) => read_lock(t)
                .map(|g| g.backend_name().to_string())
                .unwrap_or_default(),
            TopicType::StepperCommand(t) => read_lock(t)
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
            // ML types
            TopicType::TensorData(t) => read_lock(t)
                .map(|g| g.backend_name().to_string())
                .unwrap_or_default(),
            TopicType::Predictions(t) => read_lock(t)
                .map(|g| g.backend_name().to_string())
                .unwrap_or_default(),
            TopicType::InferenceMetrics(t) => read_lock(t)
                .map(|g| g.backend_name().to_string())
                .unwrap_or_default(),
            TopicType::ModelInfo(t) => read_lock(t)
                .map(|g| g.backend_name().to_string())
                .unwrap_or_default(),
            TopicType::FeatureVector(t) => read_lock(t)
                .map(|g| g.backend_name().to_string())
                .unwrap_or_default(),
            TopicType::Classification(t) => read_lock(t)
                .map(|g| g.backend_name().to_string())
                .unwrap_or_default(),
            TopicType::ChatMessage(t) => read_lock(t)
                .map(|g| g.backend_name().to_string())
                .unwrap_or_default(),
            TopicType::LLMRequest(t) => read_lock(t)
                .map(|g| g.backend_name().to_string())
                .unwrap_or_default(),
            TopicType::LLMResponse(t) => read_lock(t)
                .map(|g| g.backend_name().to_string())
                .unwrap_or_default(),
            TopicType::TrainingMetrics(t) => read_lock(t)
                .map(|g| g.backend_name().to_string())
                .unwrap_or_default(),
            TopicType::MlTrajectoryPoint(t) => read_lock(t)
                .map(|g| g.backend_name().to_string())
                .unwrap_or_default(),
            TopicType::DeploymentConfig(t) => read_lock(t)
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
            TopicType::TactileArray(t) => read_lock(t)
                .map(|g| g.backend_name().to_string())
                .unwrap_or_default(),
            TopicType::ImpedanceParameters(t) => read_lock(t)
                .map(|g| g.backend_name().to_string())
                .unwrap_or_default(),
            TopicType::HapticFeedback(t) => read_lock(t)
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
                TopicType::PwmCommand(t) => read_lock(t)?.metrics(),
                TopicType::StepperCommand(t) => read_lock(t)?.metrics(),
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
                // ML types
                TopicType::TensorData(t) => read_lock(t)?.metrics(),
                TopicType::Predictions(t) => read_lock(t)?.metrics(),
                TopicType::InferenceMetrics(t) => read_lock(t)?.metrics(),
                TopicType::ModelInfo(t) => read_lock(t)?.metrics(),
                TopicType::FeatureVector(t) => read_lock(t)?.metrics(),
                TopicType::Classification(t) => read_lock(t)?.metrics(),
                TopicType::ChatMessage(t) => read_lock(t)?.metrics(),
                TopicType::LLMRequest(t) => read_lock(t)?.metrics(),
                TopicType::LLMResponse(t) => read_lock(t)?.metrics(),
                TopicType::TrainingMetrics(t) => read_lock(t)?.metrics(),
                TopicType::MlTrajectoryPoint(t) => read_lock(t)?.metrics(),
                TopicType::DeploymentConfig(t) => read_lock(t)?.metrics(),
                // Vision types
                TopicType::CompressedImage(t) => read_lock(t)?.metrics(),
                TopicType::CameraInfo(t) => read_lock(t)?.metrics(),
                TopicType::RegionOfInterest(t) => read_lock(t)?.metrics(),
                TopicType::StereoInfo(t) => read_lock(t)?.metrics(),
                // Force types (additional)
                TopicType::TactileArray(t) => read_lock(t)?.metrics(),
                TopicType::ImpedanceParameters(t) => read_lock(t)?.metrics(),
                TopicType::HapticFeedback(t) => read_lock(t)?.metrics(),
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
                TopicType::Generic(t) => read_lock(t)?.metrics(),
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
            TopicType::PwmCommand(t) => rl!(t, py, PyPwmCommand),
            TopicType::StepperCommand(t) => rl!(t, py, PyStepperCommand),
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
            TopicType::TactileArray(t) => rl!(t, py, PyTactileArray),
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
            // Non-Copy types: ML, pool-backed, dynamic
            TopicType::Image(_) | TopicType::PointCloud(_) | TopicType::DepthImage(_)
            | TopicType::TensorData(_) | TopicType::Predictions(_)
            | TopicType::InferenceMetrics(_) | TopicType::ModelInfo(_)
            | TopicType::FeatureVector(_) | TopicType::Classification(_)
            | TopicType::ChatMessage(_) | TopicType::LLMRequest(_)
            | TopicType::LLMResponse(_) | TopicType::TrainingMetrics(_)
            | TopicType::MlTrajectoryPoint(_) | TopicType::DeploymentConfig(_)
            | TopicType::OccupancyGrid(_) | TopicType::CostMap(_)
            | TopicType::CompressedImage(_) | TopicType::PlaneArray(_)
            | TopicType::Generic(_) => {
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
