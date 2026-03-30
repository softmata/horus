//! Python wrapper classes for HORUS message types — POD-optimized
//!
//! Each Python class stores the Rust POD struct directly (`inner` field),
//! enabling zero-extraction send and zero-construction recv in Topic.
//!
// PyO3 `#[new]` constructors expose struct fields as Python keyword arguments.
// Python conventions expect `__init__` to accept all fields directly, so these
// constructors inherently exceed Clippy's 7-argument threshold.
#![allow(clippy::too_many_arguments)]
//!
//! ```python
//! from horus import Topic, CmdVel, Pose2D
//!
//! topic = Topic(CmdVel)
//! topic.send(CmdVel(1.5, 0.3))
//! msg = topic.recv()  # Returns CmdVel backed by Rust POD
//! ```

use horus_library::messages::clock::{Clock, TimeReference};
use horus_library::messages::cmd_vel::CmdVel;
use horus_library::messages::control::{
    DifferentialDriveCommand, JointCommand, MotorCommand, PidConfig, ServoCommand, TrajectoryPoint,
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
use horus_library::messages::navigation::{
    CostMap, GoalResult, GoalStatus, NavGoal, NavPath, OccupancyGrid, PathPlan, VelocityObstacle,
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
use pyo3::prelude::*;

// ============================================================================
// CmdVel — 16 bytes POD
// ============================================================================

/// Velocity command message for differential drive robots
///
/// Attributes:
///     linear: Forward/backward velocity in m/s
///     angular: Rotational velocity in rad/s
///     timestamp_ns: Timestamp in nanoseconds (default: 0)
///
/// Examples:
///     cmd = CmdVel(1.5, 0.3)
///     cmd = CmdVel(linear=0.5, angular=0.0)
#[pyclass(name = "CmdVel")]
#[derive(Clone)]
pub struct PyCmdVel {
    pub(crate) inner: CmdVel,
}

#[pymethods]
impl PyCmdVel {
    #[new]
    #[pyo3(signature = (linear, angular, timestamp_ns=0))]
    fn new(linear: f32, angular: f32, timestamp_ns: u64) -> Self {
        Self {
            inner: CmdVel::with_timestamp(linear, angular, timestamp_ns),
        }
    }

    #[getter]
    fn linear(&self) -> f32 {
        self.inner.linear
    }
    #[setter]
    fn set_linear(&mut self, v: f32) {
        self.inner.linear = v;
    }

    #[getter]
    fn angular(&self) -> f32 {
        self.inner.angular
    }
    #[setter]
    fn set_angular(&mut self, v: f32) {
        self.inner.angular = v;
    }

    #[getter]
    fn timestamp_ns(&self) -> u64 {
        self.inner.timestamp_ns
    }
    #[setter]
    fn set_timestamp_ns(&mut self, v: u64) {
        self.inner.timestamp_ns = v;
    }

    /// Stop command: zero linear and angular velocity.
    #[staticmethod]
    fn zero() -> Self {
        Self {
            inner: CmdVel::zero(),
        }
    }

    #[classattr]
    fn __topic_name__() -> &'static str {
        "cmd_vel"
    }

    fn __repr__(&self) -> String {
        format!(
            "CmdVel(linear={:.3}, angular={:.3}, timestamp_ns={})",
            self.inner.linear, self.inner.angular, self.inner.timestamp_ns
        )
    }

    fn __eq__(&self, other: &Self) -> bool {
        (self.inner.linear - other.inner.linear).abs() < 1e-6
            && (self.inner.angular - other.inner.angular).abs() < 1e-6
    }
}

// ============================================================================
// Pose2D — 32 bytes POD
// ============================================================================

/// 2D pose message (position and orientation)
///
/// Attributes:
///     x: X position in meters
///     y: Y position in meters
///     theta: Orientation in radians
///     timestamp_ns: Timestamp in nanoseconds (default: 0)
///
/// Examples:
///     pose = Pose2D(1.0, 2.0, 0.5)
///     pose = Pose2D(x=0, y=0, theta=3.14)
#[pyclass(name = "Pose2D")]
#[derive(Clone)]
pub struct PyPose2D {
    pub(crate) inner: Pose2D,
}

#[pymethods]
impl PyPose2D {
    #[new]
    #[pyo3(signature = (x, y, theta, timestamp_ns=0))]
    fn new(x: f64, y: f64, theta: f64, timestamp_ns: u64) -> Self {
        Self {
            inner: Pose2D {
                x,
                y,
                theta,
                timestamp_ns,
            },
        }
    }

    #[getter]
    fn x(&self) -> f64 {
        self.inner.x
    }
    #[setter]
    fn set_x(&mut self, v: f64) {
        self.inner.x = v;
    }

    #[getter]
    fn y(&self) -> f64 {
        self.inner.y
    }
    #[setter]
    fn set_y(&mut self, v: f64) {
        self.inner.y = v;
    }

    #[getter]
    fn theta(&self) -> f64 {
        self.inner.theta
    }
    #[setter]
    fn set_theta(&mut self, v: f64) {
        self.inner.theta = v;
    }

    #[getter]
    fn timestamp_ns(&self) -> u64 {
        self.inner.timestamp_ns
    }
    #[setter]
    fn set_timestamp_ns(&mut self, v: u64) {
        self.inner.timestamp_ns = v;
    }

    /// The origin pose (0, 0, 0).
    #[staticmethod]
    fn origin() -> Self {
        Self {
            inner: Pose2D::origin(),
        }
    }

    /// Euclidean distance to another pose (ignores theta).
    fn distance_to(&self, other: &PyPose2D) -> f64 {
        self.inner.distance_to(&other.inner)
    }

    /// Normalize theta to [-pi, pi] in-place.
    fn normalize_angle(&mut self) {
        self.inner.normalize_angle();
    }

    /// Check if all fields are finite (not NaN/Inf).
    fn is_valid(&self) -> bool {
        self.inner.is_valid()
    }

    #[classattr]
    fn __topic_name__() -> &'static str {
        "pose"
    }

    fn __repr__(&self) -> String {
        format!(
            "Pose2D(x={:.3}, y={:.3}, theta={:.3}, timestamp_ns={})",
            self.inner.x, self.inner.y, self.inner.theta, self.inner.timestamp_ns
        )
    }

    fn __eq__(&self, other: &Self) -> bool {
        (self.inner.x - other.inner.x).abs() < 1e-9
            && (self.inner.y - other.inner.y).abs() < 1e-9
            && (self.inner.theta - other.inner.theta).abs() < 1e-9
    }
}

// ============================================================================
// Imu — 304 bytes POD
// ============================================================================

/// IMU (Inertial Measurement Unit) sensor message
///
/// Attributes:
///     accel_x, accel_y, accel_z: Linear acceleration in m/s²
///     gyro_x, gyro_y, gyro_z: Angular velocity in rad/s
///     timestamp_ns: Timestamp in nanoseconds (default: 0)
///
/// Examples:
///     imu = Imu(0.0, 0.0, 9.81, 0.0, 0.0, 0.0)
///     imu = Imu(accel_x=1.0, accel_y=0.0, accel_z=9.81,
///               gyro_x=0.0, gyro_y=0.0, gyro_z=0.1)
#[pyclass(name = "Imu")]
#[derive(Clone)]
pub struct PyImu {
    pub(crate) inner: Imu,
}

#[pymethods]
impl PyImu {
    #[new]
    #[pyo3(signature = (accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z, timestamp_ns=0))]
    fn new(
        accel_x: f64,
        accel_y: f64,
        accel_z: f64,
        gyro_x: f64,
        gyro_y: f64,
        gyro_z: f64,
        timestamp_ns: u64,
    ) -> Self {
        let mut imu = Imu::new();
        imu.linear_acceleration = [accel_x, accel_y, accel_z];
        imu.angular_velocity = [gyro_x, gyro_y, gyro_z];
        imu.timestamp_ns = timestamp_ns;
        Self { inner: imu }
    }

    #[getter]
    fn accel_x(&self) -> f64 {
        self.inner.linear_acceleration[0]
    }
    #[setter]
    fn set_accel_x(&mut self, v: f64) {
        self.inner.linear_acceleration[0] = v;
    }

    #[getter]
    fn accel_y(&self) -> f64 {
        self.inner.linear_acceleration[1]
    }
    #[setter]
    fn set_accel_y(&mut self, v: f64) {
        self.inner.linear_acceleration[1] = v;
    }

    #[getter]
    fn accel_z(&self) -> f64 {
        self.inner.linear_acceleration[2]
    }
    #[setter]
    fn set_accel_z(&mut self, v: f64) {
        self.inner.linear_acceleration[2] = v;
    }

    #[getter]
    fn gyro_x(&self) -> f64 {
        self.inner.angular_velocity[0]
    }
    #[setter]
    fn set_gyro_x(&mut self, v: f64) {
        self.inner.angular_velocity[0] = v;
    }

    #[getter]
    fn gyro_y(&self) -> f64 {
        self.inner.angular_velocity[1]
    }
    #[setter]
    fn set_gyro_y(&mut self, v: f64) {
        self.inner.angular_velocity[1] = v;
    }

    #[getter]
    fn gyro_z(&self) -> f64 {
        self.inner.angular_velocity[2]
    }
    #[setter]
    fn set_gyro_z(&mut self, v: f64) {
        self.inner.angular_velocity[2] = v;
    }

    #[getter]
    fn timestamp_ns(&self) -> u64 {
        self.inner.timestamp_ns
    }
    #[setter]
    fn set_timestamp_ns(&mut self, v: u64) {
        self.inner.timestamp_ns = v;
    }

    #[classattr]
    fn __topic_name__() -> &'static str {
        "imu"
    }

    /// Set orientation from Euler angles (roll, pitch, yaw) in radians.
    fn set_orientation_from_euler(&mut self, roll: f64, pitch: f64, yaw: f64) {
        self.inner.set_orientation_from_euler(roll, pitch, yaw);
    }

    /// Check if orientation data is present (not identity quaternion).
    fn has_orientation(&self) -> bool {
        self.inner.has_orientation()
    }

    /// Check if all fields are finite (not NaN/Inf).
    fn is_valid(&self) -> bool {
        self.inner.is_valid()
    }

    /// Get angular velocity as a Vector3.
    fn angular_velocity_vec(&self) -> PyVector3 {
        PyVector3 {
            inner: self.inner.angular_velocity_vec(),
        }
    }

    /// Get linear acceleration as a Vector3.
    fn linear_acceleration_vec(&self) -> PyVector3 {
        PyVector3 {
            inner: self.inner.linear_acceleration_vec(),
        }
    }

    fn __repr__(&self) -> String {
        format!(
            "Imu(accel=[{:.3}, {:.3}, {:.3}], gyro=[{:.3}, {:.3}, {:.3}], timestamp_ns={})",
            self.inner.linear_acceleration[0],
            self.inner.linear_acceleration[1],
            self.inner.linear_acceleration[2],
            self.inner.angular_velocity[0],
            self.inner.angular_velocity[1],
            self.inner.angular_velocity[2],
            self.inner.timestamp_ns
        )
    }
}

// ============================================================================
// Odometry — 748 bytes POD
// ============================================================================

/// Odometry message (pose + velocity)
///
/// Attributes:
///     x, y, theta: Position and orientation
///     linear_velocity: Forward velocity in m/s
///     angular_velocity: Rotational velocity in rad/s
///     timestamp_ns: Timestamp in nanoseconds (default: 0)
///
/// Examples:
///     odom = Odometry(x=1.0, y=2.0, theta=0.5)
///     odom = Odometry(x=1.0, y=2.0, theta=0.5,
///                     linear_velocity=0.5, angular_velocity=0.1)
#[pyclass(name = "Odometry")]
#[derive(Clone)]
pub struct PyOdometry {
    pub(crate) inner: Odometry,
}

#[pymethods]
impl PyOdometry {
    #[new]
    #[pyo3(signature = (x=0.0, y=0.0, theta=0.0, linear_velocity=0.0, angular_velocity=0.0, timestamp_ns=0))]
    fn new(
        x: f64,
        y: f64,
        theta: f64,
        linear_velocity: f64,
        angular_velocity: f64,
        timestamp_ns: u64,
    ) -> Self {
        let mut odom = Odometry::new();
        odom.pose.x = x;
        odom.pose.y = y;
        odom.pose.theta = theta;
        odom.twist.linear[0] = linear_velocity;
        odom.twist.angular[2] = angular_velocity;
        odom.timestamp_ns = timestamp_ns;
        Self { inner: odom }
    }

    #[getter]
    fn x(&self) -> f64 {
        self.inner.pose.x
    }
    #[setter]
    fn set_x(&mut self, v: f64) {
        self.inner.pose.x = v;
    }

    #[getter]
    fn y(&self) -> f64 {
        self.inner.pose.y
    }
    #[setter]
    fn set_y(&mut self, v: f64) {
        self.inner.pose.y = v;
    }

    #[getter]
    fn theta(&self) -> f64 {
        self.inner.pose.theta
    }
    #[setter]
    fn set_theta(&mut self, v: f64) {
        self.inner.pose.theta = v;
    }

    #[getter]
    fn linear_velocity(&self) -> f64 {
        self.inner.twist.linear[0]
    }
    #[setter]
    fn set_linear_velocity(&mut self, v: f64) {
        self.inner.twist.linear[0] = v;
    }

    #[getter]
    fn angular_velocity(&self) -> f64 {
        self.inner.twist.angular[2]
    }
    #[setter]
    fn set_angular_velocity(&mut self, v: f64) {
        self.inner.twist.angular[2] = v;
    }

    #[getter]
    fn timestamp_ns(&self) -> u64 {
        self.inner.timestamp_ns
    }
    #[setter]
    fn set_timestamp_ns(&mut self, v: u64) {
        self.inner.timestamp_ns = v;
    }

    #[classattr]
    fn __topic_name__() -> &'static str {
        "odom"
    }

    /// Set the coordinate frame names for this odometry message.
    fn set_frames(&mut self, frame: &str, child_frame: &str) {
        self.inner.set_frames(frame, child_frame);
    }

    /// Update pose and twist from new measurements.
    fn update(&mut self, pose: &PyPose2D, twist: &PyTwist) {
        self.inner.update(pose.inner, twist.inner);
    }

    /// Check if all fields are finite.
    fn is_valid(&self) -> bool {
        self.inner.is_valid()
    }

    fn __repr__(&self) -> String {
        format!(
            "Odometry(x={:.3}, y={:.3}, theta={:.3}, v_lin={:.3}, v_ang={:.3}, timestamp_ns={})",
            self.inner.pose.x,
            self.inner.pose.y,
            self.inner.pose.theta,
            self.inner.twist.linear[0],
            self.inner.twist.angular[2],
            self.inner.timestamp_ns
        )
    }
}

// ============================================================================
// LaserScan — 1480 bytes POD (fixed [f32; 360])
// ============================================================================

/// Laser scan message (range measurements from a LiDAR)
///
/// Attributes:
///     angle_min: Start angle in radians
///     angle_max: End angle in radians
///     angle_increment: Angular distance between measurements
///     range_min: Minimum valid range in meters
///     range_max: Maximum valid range in meters
///     ranges: List of range measurements (max 360)
///     timestamp_ns: Timestamp in nanoseconds (default: 0)
///
/// Examples:
///     scan = LaserScan(
///         angle_min=-1.57,
///         angle_max=1.57,
///         angle_increment=0.01,
///         range_min=0.1,
///         range_max=10.0,
///         ranges=[1.0, 1.1, 1.2, ...]
///     )
#[pyclass(name = "LaserScan")]
#[derive(Clone)]
pub struct PyLaserScan {
    pub(crate) inner: LaserScan,
}

#[pymethods]
impl PyLaserScan {
    #[new]
    #[pyo3(signature = (angle_min=0.0, angle_max=0.0, angle_increment=0.0, range_min=0.0, range_max=0.0, ranges=None, timestamp_ns=0))]
    fn new(
        angle_min: f32,
        angle_max: f32,
        angle_increment: f32,
        range_min: f32,
        range_max: f32,
        ranges: Option<Vec<f32>>,
        timestamp_ns: u64,
    ) -> Self {
        let mut scan = LaserScan::new();
        scan.angle_min = angle_min;
        scan.angle_max = angle_max;
        scan.angle_increment = angle_increment;
        scan.range_min = range_min;
        scan.range_max = range_max;
        scan.timestamp_ns = timestamp_ns;
        if let Some(r) = ranges {
            let len = r.len().min(360);
            scan.ranges[..len].copy_from_slice(&r[..len]);
        }
        Self { inner: scan }
    }

    #[getter]
    fn angle_min(&self) -> f32 {
        self.inner.angle_min
    }
    #[setter]
    fn set_angle_min(&mut self, v: f32) {
        self.inner.angle_min = v;
    }

    #[getter]
    fn angle_max(&self) -> f32 {
        self.inner.angle_max
    }
    #[setter]
    fn set_angle_max(&mut self, v: f32) {
        self.inner.angle_max = v;
    }

    #[getter]
    fn angle_increment(&self) -> f32 {
        self.inner.angle_increment
    }
    #[setter]
    fn set_angle_increment(&mut self, v: f32) {
        self.inner.angle_increment = v;
    }

    #[getter]
    fn range_min(&self) -> f32 {
        self.inner.range_min
    }
    #[setter]
    fn set_range_min(&mut self, v: f32) {
        self.inner.range_min = v;
    }

    #[getter]
    fn range_max(&self) -> f32 {
        self.inner.range_max
    }
    #[setter]
    fn set_range_max(&mut self, v: f32) {
        self.inner.range_max = v;
    }

    #[getter]
    fn ranges(&self) -> Vec<f32> {
        self.inner.ranges.to_vec()
    }
    #[setter]
    fn set_ranges(&mut self, ranges: Vec<f32>) {
        self.inner.ranges = [0.0; 360];
        let len = ranges.len().min(360);
        self.inner.ranges[..len].copy_from_slice(&ranges[..len]);
    }

    #[getter]
    fn timestamp_ns(&self) -> u64 {
        self.inner.timestamp_ns
    }
    #[setter]
    fn set_timestamp_ns(&mut self, v: u64) {
        self.inner.timestamp_ns = v;
    }

    /// Compute the angle (radians) for a given range index.
    fn angle_at(&self, index: usize) -> f32 {
        self.inner.angle_at(index)
    }

    /// Check if the range reading at the given index is within [range_min, range_max].
    fn is_range_valid(&self, index: usize) -> bool {
        self.inner.is_range_valid(index)
    }

    /// Return the minimum valid range value, or None if no valid ranges.
    fn min_range(&self) -> Option<f32> {
        self.inner.min_range()
    }

    #[classattr]
    fn __topic_name__() -> &'static str {
        "scan"
    }

    fn __len__(&self) -> usize {
        self.inner.valid_count()
    }

    fn __repr__(&self) -> String {
        format!(
            "LaserScan(angle=[{:.2}, {:.2}], range=[{:.2}, {:.2}], {} points, timestamp_ns={})",
            self.inner.angle_min,
            self.inner.angle_max,
            self.inner.range_min,
            self.inner.range_max,
            self.inner.valid_count(),
            self.inner.timestamp_ns
        )
    }
}

// ============================================================================
// Pose3D — 64 bytes POD
// ============================================================================

/// 3D pose (position + orientation) — POD-optimized
///
/// Wraps the Rust `Pose3D` POD struct directly for zero-copy send/recv.
///
/// Attributes:
///     x, y, z: Position in meters
///     qx, qy, qz, qw: Orientation as quaternion (w-last convention)
///     timestamp_ns: Timestamp in nanoseconds (default: 0)
///
/// Examples:
///     pose = Pose3D(1.0, 2.0, 3.0)
///     pose = Pose3D(1.0, 2.0, 3.0, 0, 0, 0.707, 0.707)
#[pyclass(name = "Pose3D")]
#[derive(Clone)]
pub struct PyPose3D {
    pub(crate) inner: Pose3D,
}

#[pymethods]
impl PyPose3D {
    #[new]
    #[pyo3(signature = (x=0.0, y=0.0, z=0.0, qx=0.0, qy=0.0, qz=0.0, qw=1.0, timestamp_ns=0))]

    fn new(x: f64, y: f64, z: f64, qx: f64, qy: f64, qz: f64, qw: f64, timestamp_ns: u64) -> Self {
        Self {
            inner: Pose3D {
                position: Point3::new(x, y, z),
                orientation: Quaternion::new(qx, qy, qz, qw),
                timestamp_ns,
            },
        }
    }

    #[getter]
    fn x(&self) -> f64 {
        self.inner.position.x
    }
    #[setter]
    fn set_x(&mut self, v: f64) {
        self.inner.position.x = v;
    }

    #[getter]
    fn y(&self) -> f64 {
        self.inner.position.y
    }
    #[setter]
    fn set_y(&mut self, v: f64) {
        self.inner.position.y = v;
    }

    #[getter]
    fn z(&self) -> f64 {
        self.inner.position.z
    }
    #[setter]
    fn set_z(&mut self, v: f64) {
        self.inner.position.z = v;
    }

    #[getter]
    fn qx(&self) -> f64 {
        self.inner.orientation.x
    }
    #[setter]
    fn set_qx(&mut self, v: f64) {
        self.inner.orientation.x = v;
    }

    #[getter]
    fn qy(&self) -> f64 {
        self.inner.orientation.y
    }
    #[setter]
    fn set_qy(&mut self, v: f64) {
        self.inner.orientation.y = v;
    }

    #[getter]
    fn qz(&self) -> f64 {
        self.inner.orientation.z
    }
    #[setter]
    fn set_qz(&mut self, v: f64) {
        self.inner.orientation.z = v;
    }

    #[getter]
    fn qw(&self) -> f64 {
        self.inner.orientation.w
    }
    #[setter]
    fn set_qw(&mut self, v: f64) {
        self.inner.orientation.w = v;
    }

    #[getter]
    fn timestamp_ns(&self) -> u64 {
        self.inner.timestamp_ns
    }
    #[setter]
    fn set_timestamp_ns(&mut self, v: u64) {
        self.inner.timestamp_ns = v;
    }

    /// Identity pose: position (0,0,0), orientation identity quaternion.
    #[staticmethod]
    fn identity() -> Self {
        Self {
            inner: Pose3D::identity(),
        }
    }

    /// Create a 3D pose from a 2D pose (x,y → x,y,0; theta → yaw quaternion).
    #[staticmethod]
    fn from_pose_2d(pose: &PyPose2D) -> Self {
        Self {
            inner: Pose3D::from_pose_2d(&pose.inner),
        }
    }

    /// Euclidean distance to another 3D pose (position only).
    fn distance_to(&self, other: &PyPose3D) -> f64 {
        self.inner.distance_to(&other.inner)
    }

    /// Check if all fields are finite.
    fn is_valid(&self) -> bool {
        self.inner.is_valid()
    }

    #[classattr]
    fn __topic_name__() -> &'static str {
        "pose3d"
    }

    fn __repr__(&self) -> String {
        format!(
            "Pose3D(pos=[{:.3}, {:.3}, {:.3}], quat=[{:.3}, {:.3}, {:.3}, {:.3}], timestamp_ns={})",
            self.inner.position.x,
            self.inner.position.y,
            self.inner.position.z,
            self.inner.orientation.x,
            self.inner.orientation.y,
            self.inner.orientation.z,
            self.inner.orientation.w,
            self.inner.timestamp_ns
        )
    }

    fn __eq__(&self, other: &Self) -> bool {
        (self.inner.position.x - other.inner.position.x).abs() < 1e-9
            && (self.inner.position.y - other.inner.position.y).abs() < 1e-9
            && (self.inner.position.z - other.inner.position.z).abs() < 1e-9
            && (self.inner.orientation.x - other.inner.orientation.x).abs() < 1e-9
            && (self.inner.orientation.y - other.inner.orientation.y).abs() < 1e-9
            && (self.inner.orientation.z - other.inner.orientation.z).abs() < 1e-9
            && (self.inner.orientation.w - other.inner.orientation.w).abs() < 1e-9
    }
}

// ============================================================================
// JointState — ~913 bytes POD (fixed [f64; 16] arrays + [[u8; 32]; 16] names)
// ============================================================================

/// Joint state feedback message
///
/// Reports positions, velocities, and efforts for up to 16 joints.
///
/// Attributes:
///     names: List of joint names (strings)
///     positions: List of joint positions (radians or meters)
///     velocities: List of joint velocities
///     efforts: List of joint efforts (torques or forces)
///     timestamp_ns: Timestamp in nanoseconds (default: 0)
///
/// Examples:
///     js = JointState(names=["shoulder", "elbow"], positions=[0.5, 1.2])
#[pyclass(name = "JointState")]
#[derive(Clone)]
pub struct PyJointState {
    pub(crate) inner: JointState,
}

#[pymethods]
impl PyJointState {
    #[new]
    #[pyo3(signature = (names=None, positions=None, velocities=None, efforts=None, timestamp_ns=0))]
    fn new(
        names: Option<Vec<String>>,
        positions: Option<Vec<f64>>,
        velocities: Option<Vec<f64>>,
        efforts: Option<Vec<f64>>,
        timestamp_ns: u64,
    ) -> Self {
        let mut js = JointState::new();
        js.timestamp_ns = timestamp_ns;
        let names = names.unwrap_or_default();
        let positions = positions.unwrap_or_else(|| vec![0.0; names.len()]);
        let velocities = velocities.unwrap_or_else(|| vec![0.0; names.len()]);
        let efforts = efforts.unwrap_or_else(|| vec![0.0; names.len()]);
        for (i, name) in names.iter().take(16).enumerate() {
            let pos = positions.get(i).copied().unwrap_or(0.0);
            let vel = velocities.get(i).copied().unwrap_or(0.0);
            let eff = efforts.get(i).copied().unwrap_or(0.0);
            let _ = js.add_joint(name, pos, vel, eff);
        }
        Self { inner: js }
    }

    #[getter]
    fn names(&self) -> Vec<String> {
        (0..self.inner.joint_count as usize)
            .filter_map(|i| self.inner.joint_name(i).map(|s| s.to_string()))
            .collect()
    }

    #[getter]
    fn positions(&self) -> Vec<f64> {
        self.inner.positions[..self.inner.joint_count as usize].to_vec()
    }

    #[getter]
    fn velocities(&self) -> Vec<f64> {
        self.inner.velocities[..self.inner.joint_count as usize].to_vec()
    }

    #[getter]
    fn efforts(&self) -> Vec<f64> {
        self.inner.efforts[..self.inner.joint_count as usize].to_vec()
    }

    #[getter]
    fn timestamp_ns(&self) -> u64 {
        self.inner.timestamp_ns
    }
    #[setter]
    fn set_timestamp_ns(&mut self, v: u64) {
        self.inner.timestamp_ns = v;
    }

    /// Look up a joint's position by name. Returns None if not found.
    fn position(&self, name: &str) -> Option<f64> {
        self.inner.position(name)
    }

    /// Look up a joint's velocity by name. Returns None if not found.
    fn velocity(&self, name: &str) -> Option<f64> {
        self.inner.velocity(name)
    }

    /// Look up a joint's effort by name. Returns None if not found.
    fn effort(&self, name: &str) -> Option<f64> {
        self.inner.effort(name)
    }

    #[classattr]
    fn __topic_name__() -> &'static str {
        "joint_states"
    }

    fn __len__(&self) -> usize {
        self.inner.joint_count as usize
    }

    fn __repr__(&self) -> String {
        let names: Vec<String> = self.names();
        format!(
            "JointState({} joints: [{}], timestamp_ns={})",
            names.len(),
            names.join(", "),
            self.inner.timestamp_ns
        )
    }
}

// ============================================================================
// Clock — 48 bytes POD
// ============================================================================

/// Clock / time synchronization message
///
/// Attributes:
///     clock_ns: Current simulation/replay time in nanoseconds
///     realtime_ns: Wall clock time for comparison
///     sim_speed: Playback speed multiplier (1.0 = real-time)
///     paused: Whether the clock is paused
///     source: Clock source (0=wall, 1=sim, 2=replay)
///     timestamp_ns: Timestamp in nanoseconds (default: 0)
///
/// Examples:
///     clk = Clock(clock_ns=1000000000, sim_speed=2.0, source=1)
#[pyclass(name = "Clock")]
#[derive(Clone)]
pub struct PyClock {
    pub(crate) inner: Clock,
}

#[pymethods]
impl PyClock {
    #[new]
    #[pyo3(signature = (clock_ns=0, realtime_ns=0, sim_speed=1.0, paused=false, source=0, timestamp_ns=0))]
    fn new(
        clock_ns: u64,
        realtime_ns: u64,
        sim_speed: f64,
        paused: bool,
        source: u8,
        timestamp_ns: u64,
    ) -> Self {
        let mut clk = Clock::wall_clock();
        clk.clock_ns = clock_ns;
        clk.realtime_ns = realtime_ns;
        clk.sim_speed = sim_speed;
        clk.paused = paused as u8;
        clk.source = source;
        clk.timestamp_ns = timestamp_ns;
        Self { inner: clk }
    }

    #[getter]
    fn clock_ns(&self) -> u64 {
        self.inner.clock_ns
    }
    #[setter]
    fn set_clock_ns(&mut self, v: u64) {
        self.inner.clock_ns = v;
    }

    #[getter]
    fn realtime_ns(&self) -> u64 {
        self.inner.realtime_ns
    }
    #[setter]
    fn set_realtime_ns(&mut self, v: u64) {
        self.inner.realtime_ns = v;
    }

    #[getter]
    fn sim_speed(&self) -> f64 {
        self.inner.sim_speed
    }
    #[setter]
    fn set_sim_speed(&mut self, v: f64) {
        self.inner.sim_speed = v;
    }

    #[getter]
    fn paused(&self) -> bool {
        self.inner.paused != 0
    }
    #[setter]
    fn set_paused(&mut self, v: bool) {
        self.inner.paused = v as u8;
    }

    #[getter]
    fn source(&self) -> u8 {
        self.inner.source
    }
    #[setter]
    fn set_source(&mut self, v: u8) {
        self.inner.source = v;
    }

    #[getter]
    fn timestamp_ns(&self) -> u64 {
        self.inner.timestamp_ns
    }
    #[setter]
    fn set_timestamp_ns(&mut self, v: u64) {
        self.inner.timestamp_ns = v;
    }

    /// Create a wall clock (real time).
    #[staticmethod]
    fn wall_clock() -> Self {
        Self {
            inner: Clock::wall_clock(),
        }
    }

    /// Create a simulation clock with a given time and speed multiplier.
    #[staticmethod]
    fn sim_time(sim_ns: u64, speed: f64) -> Self {
        Self {
            inner: Clock::sim_time(sim_ns, speed),
        }
    }

    /// Create a replay clock with a given time and speed multiplier.
    #[staticmethod]
    fn replay_time(replay_ns: u64, speed: f64) -> Self {
        Self {
            inner: Clock::replay_time(replay_ns, speed),
        }
    }

    /// Elapsed time in nanoseconds since another clock reading.
    fn elapsed_since(&self, earlier: &PyClock) -> u64 {
        self.inner.elapsed_since(&earlier.inner)
    }

    /// Check if the clock is paused.
    fn is_paused(&self) -> bool {
        self.inner.is_paused()
    }

    #[classattr]
    fn __topic_name__() -> &'static str {
        "clock"
    }

    fn __repr__(&self) -> String {
        let source_str = match self.inner.source {
            0 => "wall",
            1 => "sim",
            2 => "replay",
            _ => "unknown",
        };
        format!(
            "Clock(clock_ns={}, speed={:.1}x, paused={}, source={}, timestamp_ns={})",
            self.inner.clock_ns,
            self.inner.sim_speed,
            self.inner.paused != 0,
            source_str,
            self.inner.timestamp_ns
        )
    }
}

// ============================================================================
// TimeReference — 56 bytes POD
// ============================================================================

/// External time reference for synchronization (GPS, NTP, PTP)
///
/// Attributes:
///     time_ref_ns: External reference time in nanoseconds
///     source: Source name string (e.g. "gps", "ntp", "ptp")
///     offset_ns: Signed offset (local_time - reference_time) in nanoseconds
///     timestamp_ns: Timestamp in nanoseconds (default: 0)
///
/// Examples:
///     tref = TimeReference(time_ref_ns=1000000000, source="gps", offset_ns=-500)
#[pyclass(name = "TimeReference")]
#[derive(Clone)]
pub struct PyTimeReference {
    pub(crate) inner: TimeReference,
}

#[pymethods]
impl PyTimeReference {
    #[new]
    #[pyo3(signature = (time_ref_ns=0, source="", offset_ns=0, timestamp_ns=0))]
    fn new(time_ref_ns: u64, source: &str, offset_ns: i64, timestamp_ns: u64) -> Self {
        let mut tr = TimeReference::new(time_ref_ns, source, offset_ns);
        tr.timestamp_ns = timestamp_ns;
        Self { inner: tr }
    }

    #[getter]
    fn time_ref_ns(&self) -> u64 {
        self.inner.time_ref_ns
    }
    #[setter]
    fn set_time_ref_ns(&mut self, v: u64) {
        self.inner.time_ref_ns = v;
    }

    #[getter]
    fn source(&self) -> &str {
        self.inner.source_name()
    }
    #[setter]
    fn set_source(&mut self, v: &str) {
        let bytes = v.as_bytes();
        let len = bytes.len().min(31);
        self.inner.source = [0u8; 32];
        self.inner.source[..len].copy_from_slice(&bytes[..len]);
    }

    #[getter]
    fn offset_ns(&self) -> i64 {
        self.inner.offset_ns
    }
    #[setter]
    fn set_offset_ns(&mut self, v: i64) {
        self.inner.offset_ns = v;
    }

    #[getter]
    fn timestamp_ns(&self) -> u64 {
        self.inner.timestamp_ns
    }
    #[setter]
    fn set_timestamp_ns(&mut self, v: u64) {
        self.inner.timestamp_ns = v;
    }

    /// Correct a local timestamp using this reference's offset
    fn correct_timestamp(&self, local_ns: u64) -> u64 {
        self.inner.correct_timestamp(local_ns)
    }

    #[classattr]
    fn __topic_name__() -> &'static str {
        "time_reference"
    }

    fn __repr__(&self) -> String {
        format!(
            "TimeReference(time_ref_ns={}, source=\"{}\", offset_ns={}, timestamp_ns={})",
            self.inner.time_ref_ns,
            self.inner.source_name(),
            self.inner.offset_ns,
            self.inner.timestamp_ns
        )
    }

    fn __eq__(&self, other: &Self) -> bool {
        self.inner.time_ref_ns == other.inner.time_ref_ns
            && self.inner.offset_ns == other.inner.offset_ns
            && self.inner.source_name() == other.inner.source_name()
    }
}

// ============================================================================
// Twist — 56 bytes POD
// ============================================================================

/// Full 6-DOF velocity (linear + angular) for 3D robots
///
/// Attributes:
///     linear_x, linear_y, linear_z: Linear velocity in m/s
///     angular_x, angular_y, angular_z: Angular velocity in rad/s
///     timestamp_ns: Timestamp in nanoseconds (default: 0)
///
/// Examples:
///     twist = Twist(linear_x=1.0, angular_z=0.5)
///     twist = Twist(1.0, 0.0, 0.0, 0.0, 0.0, 0.5)
#[pyclass(name = "Twist")]
#[derive(Clone)]
pub struct PyTwist {
    pub(crate) inner: Twist,
}

#[pymethods]
impl PyTwist {
    #[new]
    #[pyo3(signature = (linear_x=0.0, linear_y=0.0, linear_z=0.0, angular_x=0.0, angular_y=0.0, angular_z=0.0, timestamp_ns=0))]

    fn new(
        linear_x: f64,
        linear_y: f64,
        linear_z: f64,
        angular_x: f64,
        angular_y: f64,
        angular_z: f64,
        timestamp_ns: u64,
    ) -> Self {
        let mut t = Twist::new(
            [linear_x, linear_y, linear_z],
            [angular_x, angular_y, angular_z],
        );
        t.timestamp_ns = timestamp_ns;
        Self { inner: t }
    }

    #[getter]
    fn linear_x(&self) -> f64 {
        self.inner.linear[0]
    }
    #[setter]
    fn set_linear_x(&mut self, v: f64) {
        self.inner.linear[0] = v;
    }
    #[getter]
    fn linear_y(&self) -> f64 {
        self.inner.linear[1]
    }
    #[setter]
    fn set_linear_y(&mut self, v: f64) {
        self.inner.linear[1] = v;
    }
    #[getter]
    fn linear_z(&self) -> f64 {
        self.inner.linear[2]
    }
    #[setter]
    fn set_linear_z(&mut self, v: f64) {
        self.inner.linear[2] = v;
    }
    #[getter]
    fn angular_x(&self) -> f64 {
        self.inner.angular[0]
    }
    #[setter]
    fn set_angular_x(&mut self, v: f64) {
        self.inner.angular[0] = v;
    }
    #[getter]
    fn angular_y(&self) -> f64 {
        self.inner.angular[1]
    }
    #[setter]
    fn set_angular_y(&mut self, v: f64) {
        self.inner.angular[1] = v;
    }
    #[getter]
    fn angular_z(&self) -> f64 {
        self.inner.angular[2]
    }
    #[setter]
    fn set_angular_z(&mut self, v: f64) {
        self.inner.angular[2] = v;
    }
    #[getter]
    fn timestamp_ns(&self) -> u64 {
        self.inner.timestamp_ns
    }
    #[setter]
    fn set_timestamp_ns(&mut self, v: u64) {
        self.inner.timestamp_ns = v;
    }

    /// Zero velocity command (all zeros) — emergency stop.
    #[staticmethod]
    fn stop() -> Self {
        Self {
            inner: Twist::stop(),
        }
    }

    /// Create a 2D twist (linear_x + angular_z only, rest zero).
    #[staticmethod]
    #[pyo3(signature = (linear_x=0.0, angular_z=0.0))]
    fn new_2d(linear_x: f64, angular_z: f64) -> Self {
        Self {
            inner: Twist::new_2d(linear_x, angular_z),
        }
    }

    /// Check if all fields are finite (not NaN/Inf).
    fn is_valid(&self) -> bool {
        self.inner.is_valid()
    }

    #[classattr]
    fn __topic_name__() -> &'static str {
        "twist"
    }

    fn __repr__(&self) -> String {
        format!(
            "Twist(lin=[{:.3}, {:.3}, {:.3}], ang=[{:.3}, {:.3}, {:.3}], timestamp_ns={})",
            self.inner.linear[0],
            self.inner.linear[1],
            self.inner.linear[2],
            self.inner.angular[0],
            self.inner.angular[1],
            self.inner.angular[2],
            self.inner.timestamp_ns
        )
    }

    fn __eq__(&self, other: &Self) -> bool {
        self.inner
            .linear
            .iter()
            .zip(other.inner.linear.iter())
            .chain(self.inner.angular.iter().zip(other.inner.angular.iter()))
            .all(|(a, b)| (a - b).abs() < 1e-9)
    }
}

// ============================================================================
// Vector3 — 24 bytes
// ============================================================================

/// 3D vector
///
/// Attributes:
///     x, y, z: Components (f64)
///
/// Examples:
///     v = Vector3(1.0, 2.0, 3.0)
#[pyclass(name = "Vector3")]
#[derive(Clone)]
pub struct PyVector3 {
    pub(crate) inner: Vector3,
}

#[pymethods]
impl PyVector3 {
    #[new]
    #[pyo3(signature = (x=0.0, y=0.0, z=0.0))]
    fn new(x: f64, y: f64, z: f64) -> Self {
        Self {
            inner: Vector3::new(x, y, z),
        }
    }

    #[getter]
    fn x(&self) -> f64 {
        self.inner.x
    }
    #[setter]
    fn set_x(&mut self, v: f64) {
        self.inner.x = v;
    }
    #[getter]
    fn y(&self) -> f64 {
        self.inner.y
    }
    #[setter]
    fn set_y(&mut self, v: f64) {
        self.inner.y = v;
    }
    #[getter]
    fn z(&self) -> f64 {
        self.inner.z
    }
    #[setter]
    fn set_z(&mut self, v: f64) {
        self.inner.z = v;
    }

    /// Create a zero vector (0, 0, 0).
    #[staticmethod]
    fn zero() -> Self {
        Self {
            inner: Vector3::zero(),
        }
    }

    /// Euclidean magnitude (length) of this vector.
    fn magnitude(&self) -> f64 {
        self.inner.magnitude()
    }

    /// Normalize this vector in-place to unit length.
    fn normalize(&mut self) {
        self.inner.normalize();
    }

    /// Return a new unit-length copy of this vector.
    fn normalized(&self) -> Self {
        Self {
            inner: self.inner.normalized(),
        }
    }

    /// Dot product with another vector.
    fn dot(&self, other: &PyVector3) -> f64 {
        self.inner.dot(&other.inner)
    }

    /// Cross product with another vector.
    fn cross(&self, other: &PyVector3) -> PyVector3 {
        PyVector3 {
            inner: self.inner.cross(&other.inner),
        }
    }

    fn __repr__(&self) -> String {
        format!(
            "Vector3({:.3}, {:.3}, {:.3})",
            self.inner.x, self.inner.y, self.inner.z
        )
    }

    fn __eq__(&self, other: &Self) -> bool {
        (self.inner.x - other.inner.x).abs() < 1e-9
            && (self.inner.y - other.inner.y).abs() < 1e-9
            && (self.inner.z - other.inner.z).abs() < 1e-9
    }
}

// ============================================================================
// Point3 — 24 bytes
// ============================================================================

/// 3D point
///
/// Attributes:
///     x, y, z: Coordinates in meters (f64)
///
/// Examples:
///     p = Point3(1.0, 2.0, 3.0)
#[pyclass(name = "Point3")]
#[derive(Clone)]
pub struct PyPoint3 {
    pub(crate) inner: Point3,
}

#[pymethods]
impl PyPoint3 {
    #[new]
    #[pyo3(signature = (x=0.0, y=0.0, z=0.0))]
    fn new(x: f64, y: f64, z: f64) -> Self {
        Self {
            inner: Point3::new(x, y, z),
        }
    }

    #[getter]
    fn x(&self) -> f64 {
        self.inner.x
    }
    #[setter]
    fn set_x(&mut self, v: f64) {
        self.inner.x = v;
    }
    #[getter]
    fn y(&self) -> f64 {
        self.inner.y
    }
    #[setter]
    fn set_y(&mut self, v: f64) {
        self.inner.y = v;
    }
    #[getter]
    fn z(&self) -> f64 {
        self.inner.z
    }
    #[setter]
    fn set_z(&mut self, v: f64) {
        self.inner.z = v;
    }

    /// The origin point (0, 0, 0).
    #[staticmethod]
    fn origin() -> Self {
        Self {
            inner: Point3::origin(),
        }
    }

    /// Euclidean distance to another point.
    fn distance_to(&self, other: &PyPoint3) -> f64 {
        self.inner.distance_to(&other.inner)
    }

    fn __repr__(&self) -> String {
        format!(
            "Point3({:.3}, {:.3}, {:.3})",
            self.inner.x, self.inner.y, self.inner.z
        )
    }

    fn __eq__(&self, other: &Self) -> bool {
        (self.inner.x - other.inner.x).abs() < 1e-9
            && (self.inner.y - other.inner.y).abs() < 1e-9
            && (self.inner.z - other.inner.z).abs() < 1e-9
    }
}

// ============================================================================
// Quaternion — 32 bytes
// ============================================================================

/// Quaternion orientation (x, y, z, w)
///
/// Attributes:
///     x, y, z, w: Quaternion components (f64)
///
/// Examples:
///     q = Quaternion()          # Identity (0, 0, 0, 1)
///     q = Quaternion(0, 0, 0.707, 0.707)
#[pyclass(name = "Quaternion")]
#[derive(Clone)]
pub struct PyQuaternion {
    pub(crate) inner: Quaternion,
}

#[pymethods]
impl PyQuaternion {
    #[new]
    #[pyo3(signature = (x=0.0, y=0.0, z=0.0, w=1.0))]
    fn new(x: f64, y: f64, z: f64, w: f64) -> Self {
        Self {
            inner: Quaternion::new(x, y, z, w),
        }
    }

    #[getter]
    fn x(&self) -> f64 {
        self.inner.x
    }
    #[setter]
    fn set_x(&mut self, v: f64) {
        self.inner.x = v;
    }
    #[getter]
    fn y(&self) -> f64 {
        self.inner.y
    }
    #[setter]
    fn set_y(&mut self, v: f64) {
        self.inner.y = v;
    }
    #[getter]
    fn z(&self) -> f64 {
        self.inner.z
    }
    #[setter]
    fn set_z(&mut self, v: f64) {
        self.inner.z = v;
    }
    #[getter]
    fn w(&self) -> f64 {
        self.inner.w
    }
    #[setter]
    fn set_w(&mut self, v: f64) {
        self.inner.w = v;
    }

    /// Identity quaternion (no rotation): (0, 0, 0, 1).
    #[staticmethod]
    fn identity() -> Self {
        Self {
            inner: Quaternion::identity(),
        }
    }

    /// Create a quaternion from Euler angles (roll, pitch, yaw) in radians.
    #[staticmethod]
    #[pyo3(signature = (roll=0.0, pitch=0.0, yaw=0.0))]
    fn from_euler(roll: f64, pitch: f64, yaw: f64) -> Self {
        Self {
            inner: Quaternion::from_euler(roll, pitch, yaw),
        }
    }

    /// Normalize this quaternion in-place to unit length.
    fn normalize(&mut self) {
        self.inner.normalize();
    }

    /// Check if this quaternion has finite, non-zero components.
    fn is_valid(&self) -> bool {
        self.inner.is_valid()
    }

    fn __repr__(&self) -> String {
        format!(
            "Quaternion({:.3}, {:.3}, {:.3}, {:.3})",
            self.inner.x, self.inner.y, self.inner.z, self.inner.w
        )
    }

    fn __eq__(&self, other: &Self) -> bool {
        (self.inner.x - other.inner.x).abs() < 1e-9
            && (self.inner.y - other.inner.y).abs() < 1e-9
            && (self.inner.z - other.inner.z).abs() < 1e-9
            && (self.inner.w - other.inner.w).abs() < 1e-9
    }
}

// ============================================================================
// TransformStamped — 64 bytes POD
// ============================================================================

/// 3D transform (translation + rotation)
///
/// Attributes:
///     tx, ty, tz: Translation in meters
///     rx, ry, rz, rw: Rotation as quaternion
///     timestamp_ns: Timestamp in nanoseconds
///
/// Examples:
///     tf = TransformStamped(tx=1.0, ty=2.0, tz=0.0)
#[pyclass(name = "TransformStamped")]
#[derive(Clone)]
pub struct PyTransformStamped {
    pub(crate) inner: TransformStamped,
}

#[pymethods]
impl PyTransformStamped {
    #[new]
    #[pyo3(signature = (tx=0.0, ty=0.0, tz=0.0, rx=0.0, ry=0.0, rz=0.0, rw=1.0, timestamp_ns=0))]

    fn new(
        tx: f64,
        ty: f64,
        tz: f64,
        rx: f64,
        ry: f64,
        rz: f64,
        rw: f64,
        timestamp_ns: u64,
    ) -> Self {
        let mut t = TransformStamped::new([tx, ty, tz], [rx, ry, rz, rw]);
        t.timestamp_ns = timestamp_ns;
        Self { inner: t }
    }

    #[getter]
    fn tx(&self) -> f64 {
        self.inner.translation[0]
    }
    #[setter]
    fn set_tx(&mut self, v: f64) {
        self.inner.translation[0] = v;
    }
    #[getter]
    fn ty(&self) -> f64 {
        self.inner.translation[1]
    }
    #[setter]
    fn set_ty(&mut self, v: f64) {
        self.inner.translation[1] = v;
    }
    #[getter]
    fn tz(&self) -> f64 {
        self.inner.translation[2]
    }
    #[setter]
    fn set_tz(&mut self, v: f64) {
        self.inner.translation[2] = v;
    }
    #[getter]
    fn rx(&self) -> f64 {
        self.inner.rotation[0]
    }
    #[setter]
    fn set_rx(&mut self, v: f64) {
        self.inner.rotation[0] = v;
    }
    #[getter]
    fn ry(&self) -> f64 {
        self.inner.rotation[1]
    }
    #[setter]
    fn set_ry(&mut self, v: f64) {
        self.inner.rotation[1] = v;
    }
    #[getter]
    fn rz(&self) -> f64 {
        self.inner.rotation[2]
    }
    #[setter]
    fn set_rz(&mut self, v: f64) {
        self.inner.rotation[2] = v;
    }
    #[getter]
    fn rw(&self) -> f64 {
        self.inner.rotation[3]
    }
    #[setter]
    fn set_rw(&mut self, v: f64) {
        self.inner.rotation[3] = v;
    }
    #[getter]
    fn timestamp_ns(&self) -> u64 {
        self.inner.timestamp_ns
    }
    #[setter]
    fn set_timestamp_ns(&mut self, v: u64) {
        self.inner.timestamp_ns = v;
    }

    /// Identity transform: zero translation, identity rotation.
    #[staticmethod]
    fn identity() -> Self {
        Self {
            inner: TransformStamped::identity(),
        }
    }

    /// Create a 3D transform from a 2D pose.
    #[staticmethod]
    fn from_pose_2d(pose: &PyPose2D) -> Self {
        Self {
            inner: TransformStamped::from_pose_2d(&pose.inner),
        }
    }

    /// Check if all fields are finite.
    fn is_valid(&self) -> bool {
        self.inner.is_valid()
    }

    /// Normalize the rotation quaternion in-place to unit length.
    fn normalize_rotation(&mut self) {
        self.inner.normalize_rotation();
    }

    #[classattr]
    fn __topic_name__() -> &'static str {
        "transform"
    }

    fn __repr__(&self) -> String {
        format!(
            "TransformStamped(t=[{:.3}, {:.3}, {:.3}], r=[{:.3}, {:.3}, {:.3}, {:.3}], timestamp_ns={})",
            self.inner.translation[0], self.inner.translation[1], self.inner.translation[2],
            self.inner.rotation[0], self.inner.rotation[1], self.inner.rotation[2], self.inner.rotation[3],
            self.inner.timestamp_ns
        )
    }
}

// ============================================================================
// PoseStamped — Pose3D + frame_id
// ============================================================================

/// 3D pose with coordinate frame
///
/// Attributes:
///     x, y, z: Position in meters
///     qx, qy, qz, qw: Orientation as quaternion
///     frame_id: Coordinate frame name (string, max 31 chars)
///     timestamp_ns: Timestamp in nanoseconds
///
/// Examples:
///     ps = PoseStamped(x=1.0, y=2.0, frame_id="map")
#[pyclass(name = "PoseStamped")]
#[derive(Clone)]
pub struct PyPoseStamped {
    pub(crate) inner: PoseStamped,
}

#[pymethods]
impl PyPoseStamped {
    #[new]
    #[pyo3(signature = (x=0.0, y=0.0, z=0.0, qx=0.0, qy=0.0, qz=0.0, qw=1.0, frame_id="", timestamp_ns=0))]

    fn new(
        x: f64,
        y: f64,
        z: f64,
        qx: f64,
        qy: f64,
        qz: f64,
        qw: f64,
        frame_id: &str,
        timestamp_ns: u64,
    ) -> Self {
        let pose = Pose3D {
            position: Point3::new(x, y, z),
            orientation: Quaternion::new(qx, qy, qz, qw),
            timestamp_ns: 0,
        };
        let mut ps = PoseStamped::new(pose);
        let bytes = frame_id.as_bytes();
        let len = bytes.len().min(31);
        ps.frame_id[..len].copy_from_slice(&bytes[..len]);
        ps.timestamp_ns = timestamp_ns;
        Self { inner: ps }
    }

    #[getter]
    fn x(&self) -> f64 {
        self.inner.pose.position.x
    }
    #[setter]
    fn set_x(&mut self, v: f64) {
        self.inner.pose.position.x = v;
    }
    #[getter]
    fn y(&self) -> f64 {
        self.inner.pose.position.y
    }
    #[setter]
    fn set_y(&mut self, v: f64) {
        self.inner.pose.position.y = v;
    }
    #[getter]
    fn z(&self) -> f64 {
        self.inner.pose.position.z
    }
    #[setter]
    fn set_z(&mut self, v: f64) {
        self.inner.pose.position.z = v;
    }
    #[getter]
    fn qx(&self) -> f64 {
        self.inner.pose.orientation.x
    }
    #[setter]
    fn set_qx(&mut self, v: f64) {
        self.inner.pose.orientation.x = v;
    }
    #[getter]
    fn qy(&self) -> f64 {
        self.inner.pose.orientation.y
    }
    #[setter]
    fn set_qy(&mut self, v: f64) {
        self.inner.pose.orientation.y = v;
    }
    #[getter]
    fn qz(&self) -> f64 {
        self.inner.pose.orientation.z
    }
    #[setter]
    fn set_qz(&mut self, v: f64) {
        self.inner.pose.orientation.z = v;
    }
    #[getter]
    fn qw(&self) -> f64 {
        self.inner.pose.orientation.w
    }
    #[setter]
    fn set_qw(&mut self, v: f64) {
        self.inner.pose.orientation.w = v;
    }
    #[getter]
    fn frame_id(&self) -> String {
        let end = self
            .inner
            .frame_id
            .iter()
            .position(|&b| b == 0)
            .unwrap_or(32);
        String::from_utf8_lossy(&self.inner.frame_id[..end]).to_string()
    }
    #[setter]
    fn set_frame_id(&mut self, v: &str) {
        self.inner.frame_id = [0; 32];
        let bytes = v.as_bytes();
        let len = bytes.len().min(31);
        self.inner.frame_id[..len].copy_from_slice(&bytes[..len]);
    }
    #[getter]
    fn timestamp_ns(&self) -> u64 {
        self.inner.timestamp_ns
    }
    #[setter]
    fn set_timestamp_ns(&mut self, v: u64) {
        self.inner.timestamp_ns = v;
    }

    #[classattr]
    fn __topic_name__() -> &'static str {
        "pose_stamped"
    }

    fn __repr__(&self) -> String {
        format!(
            "PoseStamped(pos=[{:.3}, {:.3}, {:.3}], frame='{}', timestamp_ns={})",
            self.inner.pose.position.x,
            self.inner.pose.position.y,
            self.inner.pose.position.z,
            self.frame_id(),
            self.inner.timestamp_ns
        )
    }
}

// ============================================================================
// PoseWithCovariance — Pose3D + 6x6 covariance
// ============================================================================

/// 3D pose with 6x6 covariance matrix
///
/// Attributes:
///     x, y, z, qx, qy, qz, qw: Pose components
///     covariance: 36-element list (row-major 6x6)
///     frame_id: Coordinate frame name
///     timestamp_ns: Timestamp in nanoseconds
#[pyclass(name = "PoseWithCovariance")]
#[derive(Clone)]
pub struct PyPoseWithCovariance {
    pub(crate) inner: PoseWithCovariance,
}

#[pymethods]
impl PyPoseWithCovariance {
    #[new]
    #[pyo3(signature = (x=0.0, y=0.0, z=0.0, qx=0.0, qy=0.0, qz=0.0, qw=1.0, frame_id="", timestamp_ns=0))]

    fn new(
        x: f64,
        y: f64,
        z: f64,
        qx: f64,
        qy: f64,
        qz: f64,
        qw: f64,
        frame_id: &str,
        timestamp_ns: u64,
    ) -> Self {
        let pose = Pose3D {
            position: Point3::new(x, y, z),
            orientation: Quaternion::new(qx, qy, qz, qw),
            timestamp_ns: 0,
        };
        let mut pwc = PoseWithCovariance::new(pose);
        let bytes = frame_id.as_bytes();
        let len = bytes.len().min(31);
        pwc.frame_id[..len].copy_from_slice(&bytes[..len]);
        pwc.timestamp_ns = timestamp_ns;
        Self { inner: pwc }
    }

    #[getter]
    fn x(&self) -> f64 {
        self.inner.pose.position.x
    }
    #[setter]
    fn set_x(&mut self, v: f64) {
        self.inner.pose.position.x = v;
    }
    #[getter]
    fn y(&self) -> f64 {
        self.inner.pose.position.y
    }
    #[setter]
    fn set_y(&mut self, v: f64) {
        self.inner.pose.position.y = v;
    }
    #[getter]
    fn z(&self) -> f64 {
        self.inner.pose.position.z
    }
    #[setter]
    fn set_z(&mut self, v: f64) {
        self.inner.pose.position.z = v;
    }
    #[getter]
    fn qx(&self) -> f64 {
        self.inner.pose.orientation.x
    }
    #[setter]
    fn set_qx(&mut self, v: f64) {
        self.inner.pose.orientation.x = v;
    }
    #[getter]
    fn qy(&self) -> f64 {
        self.inner.pose.orientation.y
    }
    #[setter]
    fn set_qy(&mut self, v: f64) {
        self.inner.pose.orientation.y = v;
    }
    #[getter]
    fn qz(&self) -> f64 {
        self.inner.pose.orientation.z
    }
    #[setter]
    fn set_qz(&mut self, v: f64) {
        self.inner.pose.orientation.z = v;
    }
    #[getter]
    fn qw(&self) -> f64 {
        self.inner.pose.orientation.w
    }
    #[setter]
    fn set_qw(&mut self, v: f64) {
        self.inner.pose.orientation.w = v;
    }
    #[getter]
    fn covariance(&self) -> Vec<f64> {
        self.inner.covariance.to_vec()
    }
    #[setter]
    fn set_covariance(&mut self, cov: Vec<f64>) {
        let len = cov.len().min(36);
        self.inner.covariance[..len].copy_from_slice(&cov[..len]);
    }
    #[getter]
    fn frame_id(&self) -> String {
        let end = self
            .inner
            .frame_id
            .iter()
            .position(|&b| b == 0)
            .unwrap_or(32);
        String::from_utf8_lossy(&self.inner.frame_id[..end]).to_string()
    }
    #[setter]
    fn set_frame_id(&mut self, v: &str) {
        self.inner.frame_id = [0; 32];
        let bytes = v.as_bytes();
        let len = bytes.len().min(31);
        self.inner.frame_id[..len].copy_from_slice(&bytes[..len]);
    }
    #[getter]
    fn timestamp_ns(&self) -> u64 {
        self.inner.timestamp_ns
    }
    #[setter]
    fn set_timestamp_ns(&mut self, v: u64) {
        self.inner.timestamp_ns = v;
    }

    /// Extract position variance [var_x, var_y, var_z] from the covariance diagonal.
    fn position_variance(&self) -> [f64; 3] {
        self.inner.position_variance()
    }

    /// Extract orientation variance [var_roll, var_pitch, var_yaw] from the covariance diagonal.
    fn orientation_variance(&self) -> [f64; 3] {
        self.inner.orientation_variance()
    }

    #[classattr]
    fn __topic_name__() -> &'static str {
        "pose_cov"
    }

    fn __repr__(&self) -> String {
        format!(
            "PoseWithCovariance(pos=[{:.3}, {:.3}, {:.3}], frame='{}', timestamp_ns={})",
            self.inner.pose.position.x,
            self.inner.pose.position.y,
            self.inner.pose.position.z,
            self.frame_id(),
            self.inner.timestamp_ns
        )
    }
}

// ============================================================================
// TwistWithCovariance — Twist + 6x6 covariance
// ============================================================================

/// 3D twist with 6x6 covariance matrix
///
/// Attributes:
///     linear_x/y/z, angular_x/y/z: Velocity components
///     covariance: 36-element list (row-major 6x6)
///     frame_id: Coordinate frame name
///     timestamp_ns: Timestamp in nanoseconds
#[pyclass(name = "TwistWithCovariance")]
#[derive(Clone)]
pub struct PyTwistWithCovariance {
    pub(crate) inner: TwistWithCovariance,
}

#[pymethods]
impl PyTwistWithCovariance {
    #[new]
    #[pyo3(signature = (linear_x=0.0, linear_y=0.0, linear_z=0.0, angular_x=0.0, angular_y=0.0, angular_z=0.0, frame_id="", timestamp_ns=0))]

    fn new(
        linear_x: f64,
        linear_y: f64,
        linear_z: f64,
        angular_x: f64,
        angular_y: f64,
        angular_z: f64,
        frame_id: &str,
        timestamp_ns: u64,
    ) -> Self {
        let twist = Twist::new(
            [linear_x, linear_y, linear_z],
            [angular_x, angular_y, angular_z],
        );
        let mut twc = TwistWithCovariance::new(twist);
        let bytes = frame_id.as_bytes();
        let len = bytes.len().min(31);
        twc.frame_id[..len].copy_from_slice(&bytes[..len]);
        twc.timestamp_ns = timestamp_ns;
        Self { inner: twc }
    }

    #[getter]
    fn linear_x(&self) -> f64 {
        self.inner.twist.linear[0]
    }
    #[setter]
    fn set_linear_x(&mut self, v: f64) {
        self.inner.twist.linear[0] = v;
    }
    #[getter]
    fn linear_y(&self) -> f64 {
        self.inner.twist.linear[1]
    }
    #[setter]
    fn set_linear_y(&mut self, v: f64) {
        self.inner.twist.linear[1] = v;
    }
    #[getter]
    fn linear_z(&self) -> f64 {
        self.inner.twist.linear[2]
    }
    #[setter]
    fn set_linear_z(&mut self, v: f64) {
        self.inner.twist.linear[2] = v;
    }
    #[getter]
    fn angular_x(&self) -> f64 {
        self.inner.twist.angular[0]
    }
    #[setter]
    fn set_angular_x(&mut self, v: f64) {
        self.inner.twist.angular[0] = v;
    }
    #[getter]
    fn angular_y(&self) -> f64 {
        self.inner.twist.angular[1]
    }
    #[setter]
    fn set_angular_y(&mut self, v: f64) {
        self.inner.twist.angular[1] = v;
    }
    #[getter]
    fn angular_z(&self) -> f64 {
        self.inner.twist.angular[2]
    }
    #[setter]
    fn set_angular_z(&mut self, v: f64) {
        self.inner.twist.angular[2] = v;
    }
    #[getter]
    fn covariance(&self) -> Vec<f64> {
        self.inner.covariance.to_vec()
    }
    #[setter]
    fn set_covariance(&mut self, cov: Vec<f64>) {
        let len = cov.len().min(36);
        self.inner.covariance[..len].copy_from_slice(&cov[..len]);
    }
    #[getter]
    fn frame_id(&self) -> String {
        let end = self
            .inner
            .frame_id
            .iter()
            .position(|&b| b == 0)
            .unwrap_or(32);
        String::from_utf8_lossy(&self.inner.frame_id[..end]).to_string()
    }
    #[setter]
    fn set_frame_id(&mut self, v: &str) {
        self.inner.frame_id = [0; 32];
        let bytes = v.as_bytes();
        let len = bytes.len().min(31);
        self.inner.frame_id[..len].copy_from_slice(&bytes[..len]);
    }
    #[getter]
    fn timestamp_ns(&self) -> u64 {
        self.inner.timestamp_ns
    }
    #[setter]
    fn set_timestamp_ns(&mut self, v: u64) {
        self.inner.timestamp_ns = v;
    }

    /// Extract linear velocity variance [var_x, var_y, var_z] from the covariance diagonal.
    fn linear_variance(&self) -> [f64; 3] {
        self.inner.linear_variance()
    }

    /// Extract angular velocity variance [var_x, var_y, var_z] from the covariance diagonal.
    fn angular_variance(&self) -> [f64; 3] {
        self.inner.angular_variance()
    }

    #[classattr]
    fn __topic_name__() -> &'static str {
        "twist_cov"
    }

    fn __repr__(&self) -> String {
        format!(
            "TwistWithCovariance(lin=[{:.3}, {:.3}, {:.3}], ang=[{:.3}, {:.3}, {:.3}], timestamp_ns={})",
            self.inner.twist.linear[0], self.inner.twist.linear[1], self.inner.twist.linear[2],
            self.inner.twist.angular[0], self.inner.twist.angular[1], self.inner.twist.angular[2],
            self.inner.timestamp_ns
        )
    }
}

// ============================================================================
// Accel — 56 bytes POD
// ============================================================================

/// 3D acceleration (linear + angular)
///
/// Attributes:
///     linear_x/y/z: Linear acceleration in m/s²
///     angular_x/y/z: Angular acceleration in rad/s²
///     timestamp_ns: Timestamp in nanoseconds
#[pyclass(name = "Accel")]
#[derive(Clone)]
pub struct PyAccel {
    pub(crate) inner: Accel,
}

#[pymethods]
impl PyAccel {
    #[new]
    #[pyo3(signature = (linear_x=0.0, linear_y=0.0, linear_z=0.0, angular_x=0.0, angular_y=0.0, angular_z=0.0, timestamp_ns=0))]

    fn new(
        linear_x: f64,
        linear_y: f64,
        linear_z: f64,
        angular_x: f64,
        angular_y: f64,
        angular_z: f64,
        timestamp_ns: u64,
    ) -> Self {
        let mut a = Accel::new(
            [linear_x, linear_y, linear_z],
            [angular_x, angular_y, angular_z],
        );
        a.timestamp_ns = timestamp_ns;
        Self { inner: a }
    }

    #[getter]
    fn linear_x(&self) -> f64 {
        self.inner.linear[0]
    }
    #[setter]
    fn set_linear_x(&mut self, v: f64) {
        self.inner.linear[0] = v;
    }
    #[getter]
    fn linear_y(&self) -> f64 {
        self.inner.linear[1]
    }
    #[setter]
    fn set_linear_y(&mut self, v: f64) {
        self.inner.linear[1] = v;
    }
    #[getter]
    fn linear_z(&self) -> f64 {
        self.inner.linear[2]
    }
    #[setter]
    fn set_linear_z(&mut self, v: f64) {
        self.inner.linear[2] = v;
    }
    #[getter]
    fn angular_x(&self) -> f64 {
        self.inner.angular[0]
    }
    #[setter]
    fn set_angular_x(&mut self, v: f64) {
        self.inner.angular[0] = v;
    }
    #[getter]
    fn angular_y(&self) -> f64 {
        self.inner.angular[1]
    }
    #[setter]
    fn set_angular_y(&mut self, v: f64) {
        self.inner.angular[1] = v;
    }
    #[getter]
    fn angular_z(&self) -> f64 {
        self.inner.angular[2]
    }
    #[setter]
    fn set_angular_z(&mut self, v: f64) {
        self.inner.angular[2] = v;
    }
    #[getter]
    fn timestamp_ns(&self) -> u64 {
        self.inner.timestamp_ns
    }
    #[setter]
    fn set_timestamp_ns(&mut self, v: u64) {
        self.inner.timestamp_ns = v;
    }

    #[classattr]
    fn __topic_name__() -> &'static str {
        "accel"
    }

    /// Check if all fields are finite (not NaN/Inf).
    fn is_valid(&self) -> bool {
        self.inner.is_valid()
    }

    fn __repr__(&self) -> String {
        format!(
            "Accel(lin=[{:.3}, {:.3}, {:.3}], ang=[{:.3}, {:.3}, {:.3}], timestamp_ns={})",
            self.inner.linear[0],
            self.inner.linear[1],
            self.inner.linear[2],
            self.inner.angular[0],
            self.inner.angular[1],
            self.inner.angular[2],
            self.inner.timestamp_ns
        )
    }
}

// ============================================================================
// AccelStamped — Accel + frame_id
// ============================================================================

/// 3D acceleration with coordinate frame
#[pyclass(name = "AccelStamped")]
#[derive(Clone)]
pub struct PyAccelStamped {
    pub(crate) inner: AccelStamped,
}

#[pymethods]
impl PyAccelStamped {
    #[new]
    #[pyo3(signature = (linear_x=0.0, linear_y=0.0, linear_z=0.0, angular_x=0.0, angular_y=0.0, angular_z=0.0, frame_id="", timestamp_ns=0))]

    fn new(
        linear_x: f64,
        linear_y: f64,
        linear_z: f64,
        angular_x: f64,
        angular_y: f64,
        angular_z: f64,
        frame_id: &str,
        timestamp_ns: u64,
    ) -> Self {
        let accel = Accel::new(
            [linear_x, linear_y, linear_z],
            [angular_x, angular_y, angular_z],
        );
        let mut as_ = AccelStamped {
            accel,
            frame_id: [0; 32],
            timestamp_ns,
        };
        let bytes = frame_id.as_bytes();
        let len = bytes.len().min(31);
        as_.frame_id[..len].copy_from_slice(&bytes[..len]);
        Self { inner: as_ }
    }

    #[getter]
    fn linear_x(&self) -> f64 {
        self.inner.accel.linear[0]
    }
    #[setter]
    fn set_linear_x(&mut self, v: f64) {
        self.inner.accel.linear[0] = v;
    }
    #[getter]
    fn linear_y(&self) -> f64 {
        self.inner.accel.linear[1]
    }
    #[setter]
    fn set_linear_y(&mut self, v: f64) {
        self.inner.accel.linear[1] = v;
    }
    #[getter]
    fn linear_z(&self) -> f64 {
        self.inner.accel.linear[2]
    }
    #[setter]
    fn set_linear_z(&mut self, v: f64) {
        self.inner.accel.linear[2] = v;
    }
    #[getter]
    fn angular_x(&self) -> f64 {
        self.inner.accel.angular[0]
    }
    #[setter]
    fn set_angular_x(&mut self, v: f64) {
        self.inner.accel.angular[0] = v;
    }
    #[getter]
    fn angular_y(&self) -> f64 {
        self.inner.accel.angular[1]
    }
    #[setter]
    fn set_angular_y(&mut self, v: f64) {
        self.inner.accel.angular[1] = v;
    }
    #[getter]
    fn angular_z(&self) -> f64 {
        self.inner.accel.angular[2]
    }
    #[setter]
    fn set_angular_z(&mut self, v: f64) {
        self.inner.accel.angular[2] = v;
    }
    #[getter]
    fn frame_id(&self) -> String {
        let end = self
            .inner
            .frame_id
            .iter()
            .position(|&b| b == 0)
            .unwrap_or(32);
        String::from_utf8_lossy(&self.inner.frame_id[..end]).to_string()
    }
    #[setter]
    fn set_frame_id(&mut self, v: &str) {
        self.inner.frame_id = [0; 32];
        let bytes = v.as_bytes();
        let len = bytes.len().min(31);
        self.inner.frame_id[..len].copy_from_slice(&bytes[..len]);
    }
    #[getter]
    fn timestamp_ns(&self) -> u64 {
        self.inner.timestamp_ns
    }
    #[setter]
    fn set_timestamp_ns(&mut self, v: u64) {
        self.inner.timestamp_ns = v;
    }

    #[classattr]
    fn __topic_name__() -> &'static str {
        "accel_stamped"
    }

    fn __repr__(&self) -> String {
        format!(
            "AccelStamped(lin=[{:.3}, {:.3}, {:.3}], frame='{}', timestamp_ns={})",
            self.inner.accel.linear[0],
            self.inner.accel.linear[1],
            self.inner.accel.linear[2],
            self.frame_id(),
            self.inner.timestamp_ns
        )
    }
}

// ============================================================================
// MotorCommand — POD
// ============================================================================

/// Motor command for direct motor control
///
/// Attributes:
///     motor_id: Motor ID (u8)
///     mode: Control mode (0=velocity, 1=position, 2=torque, 3=voltage)
///     target: Target value (units depend on mode)
///     max_velocity: Maximum velocity (for position mode)
///     max_acceleration: Maximum acceleration
///     feed_forward: Feed-forward term
///     enable: Enable motor (bool)
///     timestamp_ns: Timestamp in nanoseconds
///
/// Examples:
///     cmd = MotorCommand(motor_id=0, target=1.5)
///     cmd = MotorCommand(motor_id=0, mode=1, target=3.14)
#[pyclass(name = "MotorCommand")]
#[derive(Clone)]
pub struct PyMotorCommand {
    pub(crate) inner: MotorCommand,
}

#[pymethods]
impl PyMotorCommand {
    #[new]
    #[pyo3(signature = (motor_id=0, mode=0, target=0.0, max_velocity=f64::INFINITY, max_acceleration=f64::INFINITY, feed_forward=0.0, enable=true, timestamp_ns=0))]

    fn new(
        motor_id: u8,
        mode: u8,
        target: f64,
        max_velocity: f64,
        max_acceleration: f64,
        feed_forward: f64,
        enable: bool,
        timestamp_ns: u64,
    ) -> Self {
        Self {
            inner: MotorCommand {
                motor_id,
                mode,
                target,
                max_velocity,
                max_acceleration,
                feed_forward,
                enable: enable as u8,
                timestamp_ns,
            },
        }
    }

    #[getter]
    fn motor_id(&self) -> u8 {
        self.inner.motor_id
    }
    #[setter]
    fn set_motor_id(&mut self, v: u8) {
        self.inner.motor_id = v;
    }
    #[getter]
    fn mode(&self) -> u8 {
        self.inner.mode
    }
    #[setter]
    fn set_mode(&mut self, v: u8) {
        self.inner.mode = v;
    }
    #[getter]
    fn target(&self) -> f64 {
        self.inner.target
    }
    #[setter]
    fn set_target(&mut self, v: f64) {
        self.inner.target = v;
    }
    #[getter]
    fn max_velocity(&self) -> f64 {
        self.inner.max_velocity
    }
    #[setter]
    fn set_max_velocity(&mut self, v: f64) {
        self.inner.max_velocity = v;
    }
    #[getter]
    fn max_acceleration(&self) -> f64 {
        self.inner.max_acceleration
    }
    #[setter]
    fn set_max_acceleration(&mut self, v: f64) {
        self.inner.max_acceleration = v;
    }
    #[getter]
    fn feed_forward(&self) -> f64 {
        self.inner.feed_forward
    }
    #[setter]
    fn set_feed_forward(&mut self, v: f64) {
        self.inner.feed_forward = v;
    }
    #[getter]
    fn enable(&self) -> bool {
        self.inner.enable != 0
    }
    #[setter]
    fn set_enable(&mut self, v: bool) {
        self.inner.enable = v as u8;
    }
    #[getter]
    fn timestamp_ns(&self) -> u64 {
        self.inner.timestamp_ns
    }
    #[setter]
    fn set_timestamp_ns(&mut self, v: u64) {
        self.inner.timestamp_ns = v;
    }

    /// Create a velocity command for a motor.
    #[staticmethod]
    fn velocity(motor_id: u8, velocity: f64) -> Self {
        Self {
            inner: MotorCommand::velocity(motor_id, velocity),
        }
    }

    /// Create a position command for a motor.
    #[staticmethod]
    fn position(motor_id: u8, position: f64, max_velocity: f64) -> Self {
        Self {
            inner: MotorCommand::position(motor_id, position, max_velocity),
        }
    }

    /// Create a stop command for a motor.
    #[staticmethod]
    fn stop(motor_id: u8) -> Self {
        Self {
            inner: MotorCommand::stop(motor_id),
        }
    }

    /// Check if command fields are valid.
    fn is_valid(&self) -> bool {
        self.inner.is_valid()
    }

    #[classattr]
    fn __topic_name__() -> &'static str {
        "motor_cmd"
    }

    fn __repr__(&self) -> String {
        format!(
            "MotorCommand(id={}, mode={}, target={:.3}, enable={}, timestamp_ns={})",
            self.inner.motor_id,
            self.inner.mode,
            self.inner.target,
            self.inner.enable != 0,
            self.inner.timestamp_ns
        )
    }
}

// ============================================================================
// ServoCommand — POD
// ============================================================================

/// Servo command for position-controlled servos
///
/// Attributes:
///     servo_id: Servo ID (u8)
///     position: Target position in radians (f32)
///     speed: Movement speed 0-1 (f32)
///     enable: Torque enable (bool)
///     timestamp_ns: Timestamp in nanoseconds
#[pyclass(name = "ServoCommand")]
#[derive(Clone)]
pub struct PyServoCommand {
    pub(crate) inner: ServoCommand,
}

#[pymethods]
impl PyServoCommand {
    #[new]
    #[pyo3(signature = (servo_id=0, position=0.0, speed=0.5, enable=true, timestamp_ns=0))]
    fn new(servo_id: u8, position: f32, speed: f32, enable: bool, timestamp_ns: u64) -> Self {
        Self {
            inner: ServoCommand {
                servo_id,
                position,
                speed: speed.clamp(0.0, 1.0),
                enable: enable as u8,
                timestamp_ns,
            },
        }
    }

    #[getter]
    fn servo_id(&self) -> u8 {
        self.inner.servo_id
    }
    #[setter]
    fn set_servo_id(&mut self, v: u8) {
        self.inner.servo_id = v;
    }
    #[getter]
    fn position(&self) -> f32 {
        self.inner.position
    }
    #[setter]
    fn set_position(&mut self, v: f32) {
        self.inner.position = v;
    }
    #[getter]
    fn speed(&self) -> f32 {
        self.inner.speed
    }
    #[setter]
    fn set_speed(&mut self, v: f32) {
        self.inner.speed = v.clamp(0.0, 1.0);
    }
    #[getter]
    fn enable(&self) -> bool {
        self.inner.enable != 0
    }
    #[setter]
    fn set_enable(&mut self, v: bool) {
        self.inner.enable = v as u8;
    }
    #[getter]
    fn timestamp_ns(&self) -> u64 {
        self.inner.timestamp_ns
    }
    #[setter]
    fn set_timestamp_ns(&mut self, v: u64) {
        self.inner.timestamp_ns = v;
    }

    /// Create a servo command with speed limit.
    #[staticmethod]
    fn with_speed(servo_id: u8, position: f32, speed: f32) -> Self {
        Self {
            inner: ServoCommand::with_speed(servo_id, position, speed),
        }
    }

    /// Create a disable (power-off) command for a servo.
    #[staticmethod]
    fn disable(servo_id: u8) -> Self {
        Self {
            inner: ServoCommand::disable(servo_id),
        }
    }

    /// Create a servo command from angle in degrees (converts to radians).
    #[staticmethod]
    fn from_degrees(servo_id: u8, degrees: f32) -> Self {
        Self {
            inner: ServoCommand::from_degrees(servo_id, degrees),
        }
    }

    /// Check if command fields are valid.
    fn is_valid(&self) -> bool {
        self.inner.is_valid()
    }

    #[classattr]
    fn __topic_name__() -> &'static str {
        "servo_cmd"
    }

    fn __repr__(&self) -> String {
        format!(
            "ServoCommand(id={}, pos={:.3}rad, speed={:.2}, enable={}, timestamp_ns={})",
            self.inner.servo_id,
            self.inner.position,
            self.inner.speed,
            self.inner.enable != 0,
            self.inner.timestamp_ns
        )
    }
}

// ============================================================================
// DifferentialDriveCommand — POD
// ============================================================================

/// Differential drive motor commands
///
/// Attributes:
///     left_velocity: Left wheel velocity in rad/s
///     right_velocity: Right wheel velocity in rad/s
///     max_acceleration: Maximum acceleration in rad/s²
///     enable: Enable motors (bool)
///     timestamp_ns: Timestamp in nanoseconds
#[pyclass(name = "DifferentialDriveCommand")]
#[derive(Clone)]
pub struct PyDifferentialDriveCommand {
    pub(crate) inner: DifferentialDriveCommand,
}

#[pymethods]
impl PyDifferentialDriveCommand {
    #[new]
    #[pyo3(signature = (left_velocity=0.0, right_velocity=0.0, max_acceleration=f64::INFINITY, enable=true, timestamp_ns=0))]
    fn new(
        left_velocity: f64,
        right_velocity: f64,
        max_acceleration: f64,
        enable: bool,
        timestamp_ns: u64,
    ) -> Self {
        Self {
            inner: DifferentialDriveCommand {
                left_velocity,
                right_velocity,
                max_acceleration,
                enable: enable as u8,
                timestamp_ns,
            },
        }
    }

    #[getter]
    fn left_velocity(&self) -> f64 {
        self.inner.left_velocity
    }
    #[setter]
    fn set_left_velocity(&mut self, v: f64) {
        self.inner.left_velocity = v;
    }
    #[getter]
    fn right_velocity(&self) -> f64 {
        self.inner.right_velocity
    }
    #[setter]
    fn set_right_velocity(&mut self, v: f64) {
        self.inner.right_velocity = v;
    }
    #[getter]
    fn max_acceleration(&self) -> f64 {
        self.inner.max_acceleration
    }
    #[setter]
    fn set_max_acceleration(&mut self, v: f64) {
        self.inner.max_acceleration = v;
    }
    #[getter]
    fn enable(&self) -> bool {
        self.inner.enable != 0
    }
    #[setter]
    fn set_enable(&mut self, v: bool) {
        self.inner.enable = v as u8;
    }
    #[getter]
    fn timestamp_ns(&self) -> u64 {
        self.inner.timestamp_ns
    }
    #[setter]
    fn set_timestamp_ns(&mut self, v: u64) {
        self.inner.timestamp_ns = v;
    }

    /// Stop both wheels.
    #[staticmethod]
    fn stop() -> Self {
        Self {
            inner: DifferentialDriveCommand::stop(),
        }
    }

    /// Convert a Twist (linear + angular velocity) to wheel speeds using kinematics.
    #[staticmethod]
    fn from_twist(linear: f64, angular: f64, wheel_base: f64, wheel_radius: f64) -> Self {
        Self {
            inner: DifferentialDriveCommand::from_twist(linear, angular, wheel_base, wheel_radius),
        }
    }

    /// Check if command fields are valid.
    fn is_valid(&self) -> bool {
        self.inner.is_valid()
    }

    #[classattr]
    fn __topic_name__() -> &'static str {
        "diff_drive_cmd"
    }

    fn __repr__(&self) -> String {
        format!(
            "DifferentialDriveCommand(left={:.3}, right={:.3}, enable={}, timestamp_ns={})",
            self.inner.left_velocity,
            self.inner.right_velocity,
            self.inner.enable != 0,
            self.inner.timestamp_ns
        )
    }
}

// ============================================================================
// PidConfig — POD
// ============================================================================

/// PID gains configuration
///
/// Attributes:
///     controller_id: Controller ID (u8)
///     kp, ki, kd: PID gains
///     integral_limit, output_limit: Limits
///     anti_windup: Enable anti-windup (bool)
///     timestamp_ns: Timestamp in nanoseconds
#[pyclass(name = "PidConfig")]
#[derive(Clone)]
pub struct PyPidConfig {
    pub(crate) inner: PidConfig,
}

#[pymethods]
impl PyPidConfig {
    #[new]
    #[pyo3(signature = (kp=1.0, ki=0.0, kd=0.0, controller_id=0, integral_limit=f64::INFINITY, output_limit=f64::INFINITY, anti_windup=true, timestamp_ns=0))]

    fn new(
        kp: f64,
        ki: f64,
        kd: f64,
        controller_id: u8,
        integral_limit: f64,
        output_limit: f64,
        anti_windup: bool,
        timestamp_ns: u64,
    ) -> Self {
        Self {
            inner: PidConfig {
                controller_id,
                kp,
                ki,
                kd,
                integral_limit,
                output_limit,
                anti_windup: anti_windup as u8,
                timestamp_ns,
            },
        }
    }

    #[getter]
    fn controller_id(&self) -> u8 {
        self.inner.controller_id
    }
    #[setter]
    fn set_controller_id(&mut self, v: u8) {
        self.inner.controller_id = v;
    }
    #[getter]
    fn kp(&self) -> f64 {
        self.inner.kp
    }
    #[setter]
    fn set_kp(&mut self, v: f64) {
        self.inner.kp = v;
    }
    #[getter]
    fn ki(&self) -> f64 {
        self.inner.ki
    }
    #[setter]
    fn set_ki(&mut self, v: f64) {
        self.inner.ki = v;
    }
    #[getter]
    fn kd(&self) -> f64 {
        self.inner.kd
    }
    #[setter]
    fn set_kd(&mut self, v: f64) {
        self.inner.kd = v;
    }
    #[getter]
    fn integral_limit(&self) -> f64 {
        self.inner.integral_limit
    }
    #[setter]
    fn set_integral_limit(&mut self, v: f64) {
        self.inner.integral_limit = v;
    }
    #[getter]
    fn output_limit(&self) -> f64 {
        self.inner.output_limit
    }
    #[setter]
    fn set_output_limit(&mut self, v: f64) {
        self.inner.output_limit = v;
    }
    #[getter]
    fn anti_windup(&self) -> bool {
        self.inner.anti_windup != 0
    }
    #[setter]
    fn set_anti_windup(&mut self, v: bool) {
        self.inner.anti_windup = v as u8;
    }
    #[getter]
    fn timestamp_ns(&self) -> u64 {
        self.inner.timestamp_ns
    }
    #[setter]
    fn set_timestamp_ns(&mut self, v: u64) {
        self.inner.timestamp_ns = v;
    }

    /// Proportional-only controller.
    #[staticmethod]
    fn proportional(kp: f64) -> Self {
        Self {
            inner: PidConfig::proportional(kp),
        }
    }

    /// PI controller (proportional + integral).
    #[staticmethod]
    fn pi(kp: f64, ki: f64) -> Self {
        Self {
            inner: PidConfig::pi(kp, ki),
        }
    }

    /// PD controller (proportional + derivative).
    #[staticmethod]
    fn pd(kp: f64, kd: f64) -> Self {
        Self {
            inner: PidConfig::pd(kp, kd),
        }
    }

    /// Set integral and output limits. Returns a new PidConfig with limits applied.
    fn with_limits(&self, integral_limit: f64, output_limit: f64) -> Self {
        Self {
            inner: self.inner.with_limits(integral_limit, output_limit),
        }
    }

    /// Check if gains are valid (finite, non-negative kp).
    fn is_valid(&self) -> bool {
        self.inner.is_valid()
    }

    #[classattr]
    fn __topic_name__() -> &'static str {
        "pid_config"
    }

    fn __repr__(&self) -> String {
        format!(
            "PidConfig(kp={:.3}, ki={:.3}, kd={:.3}, timestamp_ns={})",
            self.inner.kp, self.inner.ki, self.inner.kd, self.inner.timestamp_ns
        )
    }
}

// ============================================================================
// TrajectoryPoint — POD
// ============================================================================

/// Trajectory point for path following
///
/// Attributes:
///     position: [x, y, z] list
///     velocity: [vx, vy, vz] list
///     acceleration: [ax, ay, az] list
///     orientation: [qx, qy, qz, qw] list
///     angular_velocity: [wx, wy, wz] list
///     time_from_start: Time in seconds
#[pyclass(name = "TrajectoryPoint")]
#[derive(Clone)]
pub struct PyTrajectoryPoint {
    pub(crate) inner: TrajectoryPoint,
}

#[pymethods]
impl PyTrajectoryPoint {
    #[new]
    #[pyo3(signature = (position=None, velocity=None, acceleration=None, orientation=None, angular_velocity=None, time_from_start=0.0))]
    fn new(
        position: Option<Vec<f64>>,
        velocity: Option<Vec<f64>>,
        acceleration: Option<Vec<f64>>,
        orientation: Option<Vec<f64>>,
        angular_velocity: Option<Vec<f64>>,
        time_from_start: f64,
    ) -> Self {
        let to_3 = |v: Option<Vec<f64>>| -> [f64; 3] {
            match v {
                Some(v) => [
                    *v.first().unwrap_or(&0.0),
                    *v.get(1).unwrap_or(&0.0),
                    *v.get(2).unwrap_or(&0.0),
                ],
                None => [0.0; 3],
            }
        };
        let ori = match orientation {
            Some(v) => [
                *v.first().unwrap_or(&0.0),
                *v.get(1).unwrap_or(&0.0),
                *v.get(2).unwrap_or(&0.0),
                *v.get(3).unwrap_or(&1.0),
            ],
            None => [0.0, 0.0, 0.0, 1.0],
        };
        Self {
            inner: TrajectoryPoint {
                position: to_3(position),
                velocity: to_3(velocity),
                acceleration: to_3(acceleration),
                orientation: ori,
                angular_velocity: to_3(angular_velocity),
                time_from_start,
            },
        }
    }

    #[getter]
    fn position(&self) -> Vec<f64> {
        self.inner.position.to_vec()
    }
    #[setter]
    fn set_position(&mut self, v: Vec<f64>) {
        let len = v.len().min(3);
        self.inner.position[..len].copy_from_slice(&v[..len]);
    }
    #[getter]
    fn velocity(&self) -> Vec<f64> {
        self.inner.velocity.to_vec()
    }
    #[setter]
    fn set_velocity(&mut self, v: Vec<f64>) {
        let len = v.len().min(3);
        self.inner.velocity[..len].copy_from_slice(&v[..len]);
    }
    #[getter]
    fn acceleration(&self) -> Vec<f64> {
        self.inner.acceleration.to_vec()
    }
    #[setter]
    fn set_acceleration(&mut self, v: Vec<f64>) {
        let len = v.len().min(3);
        self.inner.acceleration[..len].copy_from_slice(&v[..len]);
    }
    #[getter]
    fn orientation(&self) -> Vec<f64> {
        self.inner.orientation.to_vec()
    }
    #[setter]
    fn set_orientation(&mut self, v: Vec<f64>) {
        let len = v.len().min(4);
        self.inner.orientation[..len].copy_from_slice(&v[..len]);
    }
    #[getter]
    fn angular_velocity(&self) -> Vec<f64> {
        self.inner.angular_velocity.to_vec()
    }
    #[setter]
    fn set_angular_velocity(&mut self, v: Vec<f64>) {
        let len = v.len().min(3);
        self.inner.angular_velocity[..len].copy_from_slice(&v[..len]);
    }
    #[getter]
    fn time_from_start(&self) -> f64 {
        self.inner.time_from_start
    }
    #[setter]
    fn set_time_from_start(&mut self, v: f64) {
        self.inner.time_from_start = v;
    }

    /// Create a 2D trajectory point with position and velocity.
    #[staticmethod]
    fn new_2d(x: f64, y: f64, vx: f64, vy: f64, time: f64) -> Self {
        Self {
            inner: TrajectoryPoint::new_2d(x, y, vx, vy, time),
        }
    }

    /// Create a stationary point (zero velocity) at a 3D position.
    #[staticmethod]
    fn stationary(x: f64, y: f64, z: f64) -> Self {
        Self {
            inner: TrajectoryPoint::stationary(x, y, z),
        }
    }

    #[classattr]
    fn __topic_name__() -> &'static str {
        "trajectory_point"
    }

    fn __repr__(&self) -> String {
        format!(
            "TrajectoryPoint(pos=[{:.3}, {:.3}, {:.3}], t={:.3}s)",
            self.inner.position[0],
            self.inner.position[1],
            self.inner.position[2],
            self.inner.time_from_start
        )
    }
}

// ============================================================================
// JointCommand — POD (fixed arrays, max 16 joints)
// ============================================================================

/// Joint command for multi-DOF systems
///
/// Attributes:
///     names: List of joint names (max 16)
///     positions: List of position commands (radians)
///     velocities: List of velocity commands (rad/s)
///     efforts: List of effort/torque commands (Nm)
///     modes: List of control modes (0=position, 1=velocity, 2=effort)
///     timestamp_ns: Timestamp in nanoseconds
#[pyclass(name = "JointCommand")]
#[derive(Clone)]
pub struct PyJointCommand {
    pub(crate) inner: JointCommand,
}

#[pymethods]
impl PyJointCommand {
    #[new]
    #[pyo3(signature = (names=None, positions=None, velocities=None, efforts=None, modes=None, timestamp_ns=0))]
    fn new(
        names: Option<Vec<String>>,
        positions: Option<Vec<f64>>,
        velocities: Option<Vec<f64>>,
        efforts: Option<Vec<f64>>,
        modes: Option<Vec<u8>>,
        timestamp_ns: u64,
    ) -> Self {
        let mut cmd = JointCommand::new();
        cmd.timestamp_ns = timestamp_ns;
        let names = names.unwrap_or_default();
        let positions = positions.unwrap_or_default();
        let velocities = velocities.unwrap_or_default();
        let efforts = efforts.unwrap_or_default();
        let modes = modes.unwrap_or_default();
        for (i, name) in names.iter().take(16).enumerate() {
            let _ = cmd.add_position(name, positions.get(i).copied().unwrap_or(0.0));
            if let Some(&vel) = velocities.get(i) {
                cmd.velocities[i] = vel;
            }
            if let Some(&eff) = efforts.get(i) {
                cmd.efforts[i] = eff;
            }
            if let Some(&mode) = modes.get(i) {
                cmd.modes[i] = mode;
            }
        }
        Self { inner: cmd }
    }

    #[getter]
    fn names(&self) -> Vec<String> {
        (0..self.inner.joint_count as usize)
            .map(|i| {
                let end = self.inner.joint_names[i]
                    .iter()
                    .position(|&b| b == 0)
                    .unwrap_or(32);
                String::from_utf8_lossy(&self.inner.joint_names[i][..end]).to_string()
            })
            .collect()
    }
    #[getter]
    fn positions(&self) -> Vec<f64> {
        self.inner.positions[..self.inner.joint_count as usize].to_vec()
    }
    #[getter]
    fn velocities(&self) -> Vec<f64> {
        self.inner.velocities[..self.inner.joint_count as usize].to_vec()
    }
    #[getter]
    fn efforts(&self) -> Vec<f64> {
        self.inner.efforts[..self.inner.joint_count as usize].to_vec()
    }
    #[getter]
    fn modes(&self) -> Vec<u8> {
        self.inner.modes[..self.inner.joint_count as usize].to_vec()
    }
    #[getter]
    fn timestamp_ns(&self) -> u64 {
        self.inner.timestamp_ns
    }
    #[setter]
    fn set_timestamp_ns(&mut self, v: u64) {
        self.inner.timestamp_ns = v;
    }

    fn __len__(&self) -> usize {
        self.inner.joint_count as usize
    }

    /// Add a position command for a named joint.
    fn add_position(&mut self, name: &str, position: f64) -> pyo3::PyResult<()> {
        self.inner
            .add_position(name, position)
            .map_err(pyo3::exceptions::PyValueError::new_err)
    }

    /// Add a velocity command for a named joint.
    fn add_velocity(&mut self, name: &str, velocity: f64) -> pyo3::PyResult<()> {
        self.inner
            .add_velocity(name, velocity)
            .map_err(pyo3::exceptions::PyValueError::new_err)
    }

    /// Check if command fields are valid.
    fn is_valid(&self) -> bool {
        self.inner.is_valid()
    }

    #[classattr]
    fn __topic_name__() -> &'static str {
        "joint_cmd"
    }

    fn __repr__(&self) -> String {
        format!(
            "JointCommand({} joints, timestamp_ns={})",
            self.inner.joint_count, self.inner.timestamp_ns
        )
    }
}

// ============================================================================
// RangeSensor — POD
// ============================================================================

/// Range sensor reading (ultrasonic, infrared, etc.)
#[pyclass(name = "RangeSensor")]
#[derive(Clone)]
pub struct PyRangeSensor {
    pub(crate) inner: RangeSensor,
}

#[pymethods]
impl PyRangeSensor {
    #[new]
    #[pyo3(signature = (range=0.0, sensor_type=0, field_of_view=0.1, min_range=0.02, max_range=4.0, timestamp_ns=0))]
    fn new(
        range: f32,
        sensor_type: u8,
        field_of_view: f32,
        min_range: f32,
        max_range: f32,
        timestamp_ns: u64,
    ) -> Self {
        Self {
            inner: RangeSensor {
                sensor_type,
                field_of_view,
                min_range,
                max_range,
                range,
                timestamp_ns,
            },
        }
    }

    #[getter]
    fn range(&self) -> f32 {
        self.inner.range
    }
    #[setter]
    fn set_range(&mut self, v: f32) {
        self.inner.range = v;
    }
    #[getter]
    fn sensor_type(&self) -> u8 {
        self.inner.sensor_type
    }
    #[setter]
    fn set_sensor_type(&mut self, v: u8) {
        self.inner.sensor_type = v;
    }
    #[getter]
    fn field_of_view(&self) -> f32 {
        self.inner.field_of_view
    }
    #[setter]
    fn set_field_of_view(&mut self, v: f32) {
        self.inner.field_of_view = v;
    }
    #[getter]
    fn min_range(&self) -> f32 {
        self.inner.min_range
    }
    #[setter]
    fn set_min_range(&mut self, v: f32) {
        self.inner.min_range = v;
    }
    #[getter]
    fn max_range(&self) -> f32 {
        self.inner.max_range
    }
    #[setter]
    fn set_max_range(&mut self, v: f32) {
        self.inner.max_range = v;
    }
    #[getter]
    fn timestamp_ns(&self) -> u64 {
        self.inner.timestamp_ns
    }
    #[setter]
    fn set_timestamp_ns(&mut self, v: u64) {
        self.inner.timestamp_ns = v;
    }

    fn is_valid(&self) -> bool {
        self.inner.is_valid()
    }

    #[classattr]
    fn __topic_name__() -> &'static str {
        "range"
    }

    fn __repr__(&self) -> String {
        format!(
            "RangeSensor(range={:.3}, type={}, timestamp_ns={})",
            self.inner.range, self.inner.sensor_type, self.inner.timestamp_ns
        )
    }
}

// ============================================================================
// BatteryState — POD
// ============================================================================

/// Battery status message
#[pyclass(name = "BatteryState")]
#[derive(Clone)]
pub struct PyBatteryState {
    pub(crate) inner: BatteryState,
}

#[pymethods]
impl PyBatteryState {
    #[new]
    #[pyo3(signature = (voltage=0.0, percentage=0.0, current=0.0, temperature=25.0, power_supply_status=0, timestamp_ns=0))]
    fn new(
        voltage: f32,
        percentage: f32,
        current: f32,
        temperature: f32,
        power_supply_status: u8,
        timestamp_ns: u64,
    ) -> Self {
        let mut bs = BatteryState::new(voltage, percentage);
        bs.current = current;
        bs.temperature = temperature;
        bs.power_supply_status = power_supply_status;
        bs.timestamp_ns = timestamp_ns;
        Self { inner: bs }
    }

    #[getter]
    fn voltage(&self) -> f32 {
        self.inner.voltage
    }
    #[setter]
    fn set_voltage(&mut self, v: f32) {
        self.inner.voltage = v;
    }
    #[getter]
    fn current(&self) -> f32 {
        self.inner.current
    }
    #[setter]
    fn set_current(&mut self, v: f32) {
        self.inner.current = v;
    }
    #[getter]
    fn percentage(&self) -> f32 {
        self.inner.percentage
    }
    #[setter]
    fn set_percentage(&mut self, v: f32) {
        self.inner.percentage = v;
    }
    #[getter]
    fn temperature(&self) -> f32 {
        self.inner.temperature
    }
    #[setter]
    fn set_temperature(&mut self, v: f32) {
        self.inner.temperature = v;
    }
    #[getter]
    fn power_supply_status(&self) -> u8 {
        self.inner.power_supply_status
    }
    #[setter]
    fn set_power_supply_status(&mut self, v: u8) {
        self.inner.power_supply_status = v;
    }
    #[getter]
    fn timestamp_ns(&self) -> u64 {
        self.inner.timestamp_ns
    }
    #[setter]
    fn set_timestamp_ns(&mut self, v: u64) {
        self.inner.timestamp_ns = v;
    }

    fn is_low(&self, threshold: f32) -> bool {
        self.inner.is_low(threshold)
    }
    fn is_critical(&self) -> bool {
        self.inner.is_critical()
    }

    /// Estimated time remaining in seconds, or None if not discharging.
    fn time_remaining(&self) -> Option<f32> {
        self.inner.time_remaining()
    }

    #[classattr]
    fn __topic_name__() -> &'static str {
        "battery"
    }

    fn __repr__(&self) -> String {
        format!(
            "BatteryState(voltage={:.2}V, {:.1}%, status={}, timestamp_ns={})",
            self.inner.voltage,
            self.inner.percentage,
            self.inner.power_supply_status,
            self.inner.timestamp_ns
        )
    }
}

// ============================================================================
// NavSatFix — POD (GPS)
// ============================================================================

/// GPS/GNSS position data
#[pyclass(name = "NavSatFix")]
#[derive(Clone)]
pub struct PyNavSatFix {
    pub(crate) inner: NavSatFix,
}

#[pymethods]
impl PyNavSatFix {
    #[new]
    #[pyo3(signature = (latitude=0.0, longitude=0.0, altitude=0.0, timestamp_ns=0))]
    fn new(latitude: f64, longitude: f64, altitude: f64, timestamp_ns: u64) -> Self {
        let mut fix = NavSatFix::from_coordinates(latitude, longitude, altitude);
        fix.timestamp_ns = timestamp_ns;
        Self { inner: fix }
    }

    #[getter]
    fn latitude(&self) -> f64 {
        self.inner.latitude
    }
    #[setter]
    fn set_latitude(&mut self, v: f64) {
        self.inner.latitude = v;
    }
    #[getter]
    fn longitude(&self) -> f64 {
        self.inner.longitude
    }
    #[setter]
    fn set_longitude(&mut self, v: f64) {
        self.inner.longitude = v;
    }
    #[getter]
    fn altitude(&self) -> f64 {
        self.inner.altitude
    }
    #[setter]
    fn set_altitude(&mut self, v: f64) {
        self.inner.altitude = v;
    }
    #[getter]
    fn status(&self) -> u8 {
        self.inner.status
    }
    #[setter]
    fn set_status(&mut self, v: u8) {
        self.inner.status = v;
    }
    #[getter]
    fn satellites_visible(&self) -> u16 {
        self.inner.satellites_visible
    }
    #[setter]
    fn set_satellites_visible(&mut self, v: u16) {
        self.inner.satellites_visible = v;
    }
    #[getter]
    fn hdop(&self) -> f32 {
        self.inner.hdop
    }
    #[setter]
    fn set_hdop(&mut self, v: f32) {
        self.inner.hdop = v;
    }
    #[getter]
    fn speed(&self) -> f32 {
        self.inner.speed
    }
    #[setter]
    fn set_speed(&mut self, v: f32) {
        self.inner.speed = v;
    }
    #[getter]
    fn heading(&self) -> f32 {
        self.inner.heading
    }
    #[setter]
    fn set_heading(&mut self, v: f32) {
        self.inner.heading = v;
    }
    #[getter]
    fn timestamp_ns(&self) -> u64 {
        self.inner.timestamp_ns
    }
    #[setter]
    fn set_timestamp_ns(&mut self, v: u64) {
        self.inner.timestamp_ns = v;
    }

    fn has_fix(&self) -> bool {
        self.inner.has_fix()
    }
    fn is_valid(&self) -> bool {
        self.inner.is_valid()
    }
    fn horizontal_accuracy(&self) -> f32 {
        self.inner.horizontal_accuracy()
    }

    /// Create from GPS coordinates.
    #[staticmethod]
    fn from_coordinates(lat: f64, lon: f64, alt: f64) -> Self {
        Self {
            inner: NavSatFix::from_coordinates(lat, lon, alt),
        }
    }

    /// Great-circle distance to another GPS position in meters.
    fn distance_to(&self, other: &PyNavSatFix) -> f64 {
        self.inner.distance_to(&other.inner)
    }

    #[classattr]
    fn __topic_name__() -> &'static str {
        "gps"
    }

    fn __repr__(&self) -> String {
        format!(
            "NavSatFix(lat={:.6}, lon={:.6}, alt={:.1}, sats={}, timestamp_ns={})",
            self.inner.latitude,
            self.inner.longitude,
            self.inner.altitude,
            self.inner.satellites_visible,
            self.inner.timestamp_ns
        )
    }
}

// ============================================================================
// MagneticField — POD
// ============================================================================

/// Magnetometer data (3-axis magnetic field)
#[pyclass(name = "MagneticField")]
#[derive(Clone)]
pub struct PyMagneticField {
    pub(crate) inner: MagneticField,
}

#[pymethods]
impl PyMagneticField {
    #[new]
    #[pyo3(signature = (x=0.0, y=0.0, z=0.0, timestamp_ns=0))]
    fn new(x: f64, y: f64, z: f64, timestamp_ns: u64) -> Self {
        let mut m = MagneticField::new([x, y, z]);
        m.timestamp_ns = timestamp_ns;
        Self { inner: m }
    }

    #[getter]
    fn x(&self) -> f64 {
        self.inner.magnetic_field[0]
    }
    #[setter]
    fn set_x(&mut self, v: f64) {
        self.inner.magnetic_field[0] = v;
    }
    #[getter]
    fn y(&self) -> f64 {
        self.inner.magnetic_field[1]
    }
    #[setter]
    fn set_y(&mut self, v: f64) {
        self.inner.magnetic_field[1] = v;
    }
    #[getter]
    fn z(&self) -> f64 {
        self.inner.magnetic_field[2]
    }
    #[setter]
    fn set_z(&mut self, v: f64) {
        self.inner.magnetic_field[2] = v;
    }
    #[getter]
    fn timestamp_ns(&self) -> u64 {
        self.inner.timestamp_ns
    }
    #[setter]
    fn set_timestamp_ns(&mut self, v: u64) {
        self.inner.timestamp_ns = v;
    }

    #[classattr]
    fn __topic_name__() -> &'static str {
        "mag"
    }

    fn __repr__(&self) -> String {
        format!(
            "MagneticField(x={:.6}, y={:.6}, z={:.6}, timestamp_ns={})",
            self.inner.magnetic_field[0],
            self.inner.magnetic_field[1],
            self.inner.magnetic_field[2],
            self.inner.timestamp_ns
        )
    }
}

// ============================================================================
// Temperature — POD
// ============================================================================

/// Temperature sensor reading
#[pyclass(name = "Temperature")]
#[derive(Clone)]
pub struct PyTemperature {
    pub(crate) inner: Temperature,
}

#[pymethods]
impl PyTemperature {
    #[new]
    #[pyo3(signature = (temperature=0.0, variance=0.0, timestamp_ns=0))]
    fn new(temperature: f64, variance: f64, timestamp_ns: u64) -> Self {
        let mut t = Temperature::new(temperature);
        t.variance = variance;
        t.timestamp_ns = timestamp_ns;
        Self { inner: t }
    }

    #[getter]
    fn temperature(&self) -> f64 {
        self.inner.temperature
    }
    #[setter]
    fn set_temperature(&mut self, v: f64) {
        self.inner.temperature = v;
    }
    #[getter]
    fn variance(&self) -> f64 {
        self.inner.variance
    }
    #[setter]
    fn set_variance(&mut self, v: f64) {
        self.inner.variance = v;
    }
    #[getter]
    fn timestamp_ns(&self) -> u64 {
        self.inner.timestamp_ns
    }
    #[setter]
    fn set_timestamp_ns(&mut self, v: u64) {
        self.inner.timestamp_ns = v;
    }

    #[classattr]
    fn __topic_name__() -> &'static str {
        "temperature"
    }

    fn __repr__(&self) -> String {
        format!(
            "Temperature({:.2}°C, timestamp_ns={})",
            self.inner.temperature, self.inner.timestamp_ns
        )
    }
}

// ============================================================================
// FluidPressure — POD
// ============================================================================

/// Fluid pressure sensor reading (barometer, etc.)
#[pyclass(name = "FluidPressure")]
#[derive(Clone)]
pub struct PyFluidPressure {
    pub(crate) inner: FluidPressure,
}

#[pymethods]
impl PyFluidPressure {
    #[new]
    #[pyo3(signature = (pressure=0.0, variance=0.0, timestamp_ns=0))]
    fn new(pressure: f64, variance: f64, timestamp_ns: u64) -> Self {
        let mut p = FluidPressure::new(pressure);
        p.variance = variance;
        p.timestamp_ns = timestamp_ns;
        Self { inner: p }
    }

    #[getter]
    fn pressure(&self) -> f64 {
        self.inner.fluid_pressure
    }
    #[setter]
    fn set_pressure(&mut self, v: f64) {
        self.inner.fluid_pressure = v;
    }
    #[getter]
    fn variance(&self) -> f64 {
        self.inner.variance
    }
    #[setter]
    fn set_variance(&mut self, v: f64) {
        self.inner.variance = v;
    }
    #[getter]
    fn timestamp_ns(&self) -> u64 {
        self.inner.timestamp_ns
    }
    #[setter]
    fn set_timestamp_ns(&mut self, v: u64) {
        self.inner.timestamp_ns = v;
    }

    #[classattr]
    fn __topic_name__() -> &'static str {
        "pressure"
    }

    fn __repr__(&self) -> String {
        format!(
            "FluidPressure({:.1} Pa, timestamp_ns={})",
            self.inner.fluid_pressure, self.inner.timestamp_ns
        )
    }
}

// ============================================================================
// Illuminance — POD
// ============================================================================

/// Illuminance sensor reading (light level in Lux)
#[pyclass(name = "Illuminance")]
#[derive(Clone)]
pub struct PyIlluminance {
    pub(crate) inner: Illuminance,
}

#[pymethods]
impl PyIlluminance {
    #[new]
    #[pyo3(signature = (illuminance=0.0, variance=0.0, timestamp_ns=0))]
    fn new(illuminance: f64, variance: f64, timestamp_ns: u64) -> Self {
        let mut i = Illuminance::new(illuminance);
        i.variance = variance;
        i.timestamp_ns = timestamp_ns;
        Self { inner: i }
    }

    #[getter]
    fn illuminance(&self) -> f64 {
        self.inner.illuminance
    }
    #[setter]
    fn set_illuminance(&mut self, v: f64) {
        self.inner.illuminance = v;
    }
    #[getter]
    fn variance(&self) -> f64 {
        self.inner.variance
    }
    #[setter]
    fn set_variance(&mut self, v: f64) {
        self.inner.variance = v;
    }
    #[getter]
    fn timestamp_ns(&self) -> u64 {
        self.inner.timestamp_ns
    }
    #[setter]
    fn set_timestamp_ns(&mut self, v: u64) {
        self.inner.timestamp_ns = v;
    }

    #[classattr]
    fn __topic_name__() -> &'static str {
        "illuminance"
    }

    fn __repr__(&self) -> String {
        format!(
            "Illuminance({:.1} lux, timestamp_ns={})",
            self.inner.illuminance, self.inner.timestamp_ns
        )
    }
}

// ============================================================================
// Heartbeat — POD (Diagnostics)
// ============================================================================

/// System heartbeat message
#[pyclass(name = "Heartbeat")]
#[derive(Clone)]
pub struct PyHeartbeat {
    pub(crate) inner: Heartbeat,
}

#[pymethods]
impl PyHeartbeat {
    #[new]
    #[pyo3(signature = (node_name="", node_id=0, timestamp_ns=0))]
    fn new(node_name: &str, node_id: u32, timestamp_ns: u64) -> Self {
        let mut hb = Heartbeat::new(node_name, node_id);
        if timestamp_ns != 0 {
            hb.timestamp_ns = timestamp_ns;
        }
        Self { inner: hb }
    }

    #[getter]
    fn node_name(&self) -> String {
        self.inner.name()
    }
    #[getter]
    fn node_id(&self) -> u32 {
        self.inner.node_id
    }
    #[getter]
    fn sequence(&self) -> u64 {
        self.inner.sequence
    }
    #[getter]
    fn alive(&self) -> bool {
        self.inner.alive != 0
    }
    #[getter]
    fn uptime(&self) -> f64 {
        self.inner.uptime
    }
    #[getter]
    fn timestamp_ns(&self) -> u64 {
        self.inner.timestamp_ns
    }
    #[setter]
    fn set_timestamp_ns(&mut self, v: u64) {
        self.inner.timestamp_ns = v;
    }

    /// Increment sequence number and update uptime.
    fn update(&mut self, uptime: f64) {
        self.inner.update(uptime);
    }

    #[classattr]
    fn __topic_name__() -> &'static str {
        "heartbeat"
    }

    fn __repr__(&self) -> String {
        format!(
            "Heartbeat(node='{}', id={}, seq={}, alive={}, uptime={:.1}s)",
            self.inner.name(),
            self.inner.node_id,
            self.inner.sequence,
            self.inner.alive != 0,
            self.inner.uptime
        )
    }
}

// ============================================================================
// DiagnosticStatus — POD (Diagnostics)
// ============================================================================

/// Diagnostic status message
#[pyclass(name = "DiagnosticStatus")]
#[derive(Clone)]
pub struct PyDiagnosticStatus {
    pub(crate) inner: DiagnosticStatus,
}

#[pymethods]
impl PyDiagnosticStatus {
    #[new]
    #[pyo3(signature = (level=0, code=0, message="", component="", timestamp_ns=0))]
    fn new(level: u8, code: u32, message: &str, component: &str, timestamp_ns: u64) -> Self {
        let status_level = match level {
            1 => horus_library::messages::diagnostics::StatusLevel::Warn,
            2 => horus_library::messages::diagnostics::StatusLevel::Error,
            3 => horus_library::messages::diagnostics::StatusLevel::Fatal,
            _ => horus_library::messages::diagnostics::StatusLevel::Ok,
        };
        let mut ds = DiagnosticStatus::new(status_level, code, message).with_component(component);
        if timestamp_ns != 0 {
            ds.timestamp_ns = timestamp_ns;
        }
        Self { inner: ds }
    }

    #[getter]
    fn level(&self) -> u8 {
        self.inner.level
    }
    #[setter]
    fn set_level(&mut self, v: u8) {
        self.inner.level = v;
    }
    #[getter]
    fn code(&self) -> u32 {
        self.inner.code
    }
    #[setter]
    fn set_code(&mut self, v: u32) {
        self.inner.code = v;
    }
    #[getter]
    fn message(&self) -> String {
        self.inner.message_str()
    }
    #[getter]
    fn component(&self) -> String {
        self.inner.component_str()
    }
    #[getter]
    fn timestamp_ns(&self) -> u64 {
        self.inner.timestamp_ns
    }
    #[setter]
    fn set_timestamp_ns(&mut self, v: u64) {
        self.inner.timestamp_ns = v;
    }

    /// Create an OK status.
    #[staticmethod]
    fn ok(message: &str) -> Self {
        Self {
            inner: DiagnosticStatus::ok(message),
        }
    }

    /// Create a WARN status.
    #[staticmethod]
    fn warn(code: u32, message: &str) -> Self {
        Self {
            inner: DiagnosticStatus::warn(code, message),
        }
    }

    /// Create an ERROR status.
    #[staticmethod]
    fn error(code: u32, message: &str) -> Self {
        Self {
            inner: DiagnosticStatus::error(code, message),
        }
    }

    /// Create a FATAL status.
    #[staticmethod]
    fn fatal(code: u32, message: &str) -> Self {
        Self {
            inner: DiagnosticStatus::fatal(code, message),
        }
    }

    /// Set the component name. Returns a new DiagnosticStatus.
    fn with_component(&self, component: &str) -> Self {
        Self {
            inner: self.inner.with_component(component),
        }
    }

    /// Get the message as a string.
    fn message_str(&self) -> String {
        self.inner.message_str()
    }

    /// Get the component as a string.
    fn component_str(&self) -> String {
        self.inner.component_str()
    }

    #[classattr]
    fn __topic_name__() -> &'static str {
        "diagnostics"
    }

    fn __repr__(&self) -> String {
        let level_str = match self.inner.level {
            0 => "OK",
            1 => "WARN",
            2 => "ERROR",
            3 => "FATAL",
            _ => "?",
        };
        format!(
            "DiagnosticStatus(level={}, code={}, msg='{}', component='{}')",
            level_str,
            self.inner.code,
            self.inner.message_str(),
            self.inner.component_str()
        )
    }
}

// ============================================================================
// EmergencyStop — POD (Diagnostics)
// ============================================================================

/// Emergency stop message — critical safety signal
#[pyclass(name = "EmergencyStop")]
#[derive(Clone)]
pub struct PyEmergencyStop {
    pub(crate) inner: EmergencyStop,
}

#[pymethods]
impl PyEmergencyStop {
    #[new]
    #[pyo3(signature = (engaged=true, reason="", timestamp_ns=0))]
    fn new(engaged: bool, reason: &str, timestamp_ns: u64) -> Self {
        let es = if engaged {
            EmergencyStop::engage(reason)
        } else {
            EmergencyStop::release()
        };
        let mut s = Self { inner: es };
        if timestamp_ns != 0 {
            s.inner.timestamp_ns = timestamp_ns;
        }
        s
    }

    #[getter]
    fn engaged(&self) -> bool {
        self.inner.engaged != 0
    }
    #[setter]
    fn set_engaged(&mut self, v: bool) {
        self.inner.engaged = v as u8;
    }
    #[getter]
    fn reason(&self) -> String {
        self.inner.reason_str()
    }
    #[getter]
    fn auto_reset(&self) -> bool {
        self.inner.auto_reset != 0
    }
    #[getter]
    fn timestamp_ns(&self) -> u64 {
        self.inner.timestamp_ns
    }
    #[setter]
    fn set_timestamp_ns(&mut self, v: u64) {
        self.inner.timestamp_ns = v;
    }

    /// Create an engaged emergency stop.
    #[staticmethod]
    fn engage(reason: &str) -> Self {
        Self {
            inner: EmergencyStop::engage(reason),
        }
    }

    /// Create a release (disengage) command.
    #[staticmethod]
    fn release() -> Self {
        Self {
            inner: EmergencyStop::release(),
        }
    }

    /// Set the source identifier. Returns a new EmergencyStop.
    fn with_source(&self, source: &str) -> Self {
        Self {
            inner: self.inner.with_source(source),
        }
    }

    /// Get the reason as a string.
    fn reason_str(&self) -> String {
        self.inner.reason_str()
    }

    #[classattr]
    fn __topic_name__() -> &'static str {
        "estop"
    }

    fn __repr__(&self) -> String {
        format!(
            "EmergencyStop(engaged={}, reason='{}', timestamp_ns={})",
            self.inner.engaged != 0,
            self.inner.reason_str(),
            self.inner.timestamp_ns
        )
    }
}

// ============================================================================
// ResourceUsage — POD (Diagnostics)
// ============================================================================

/// System resource usage message
#[pyclass(name = "ResourceUsage")]
#[derive(Clone)]
pub struct PyResourceUsage {
    pub(crate) inner: ResourceUsage,
}

#[pymethods]
impl PyResourceUsage {
    #[new]
    #[pyo3(signature = (cpu_percent=0.0, memory_bytes=0, memory_percent=0.0, timestamp_ns=0))]
    fn new(cpu_percent: f32, memory_bytes: u64, memory_percent: f32, timestamp_ns: u64) -> Self {
        let mut ru = ResourceUsage::new();
        ru.cpu_percent = cpu_percent;
        ru.memory_bytes = memory_bytes;
        ru.memory_percent = memory_percent;
        if timestamp_ns != 0 {
            ru.timestamp_ns = timestamp_ns;
        }
        Self { inner: ru }
    }

    #[getter]
    fn cpu_percent(&self) -> f32 {
        self.inner.cpu_percent
    }
    #[setter]
    fn set_cpu_percent(&mut self, v: f32) {
        self.inner.cpu_percent = v;
    }
    #[getter]
    fn memory_bytes(&self) -> u64 {
        self.inner.memory_bytes
    }
    #[setter]
    fn set_memory_bytes(&mut self, v: u64) {
        self.inner.memory_bytes = v;
    }
    #[getter]
    fn memory_percent(&self) -> f32 {
        self.inner.memory_percent
    }
    #[setter]
    fn set_memory_percent(&mut self, v: f32) {
        self.inner.memory_percent = v;
    }
    #[getter]
    fn disk_bytes(&self) -> u64 {
        self.inner.disk_bytes
    }
    #[setter]
    fn set_disk_bytes(&mut self, v: u64) {
        self.inner.disk_bytes = v;
    }
    #[getter]
    fn disk_percent(&self) -> f32 {
        self.inner.disk_percent
    }
    #[setter]
    fn set_disk_percent(&mut self, v: f32) {
        self.inner.disk_percent = v;
    }
    #[getter]
    fn network_tx_bytes(&self) -> u64 {
        self.inner.network_tx_bytes
    }
    #[setter]
    fn set_network_tx_bytes(&mut self, v: u64) {
        self.inner.network_tx_bytes = v;
    }
    #[getter]
    fn network_rx_bytes(&self) -> u64 {
        self.inner.network_rx_bytes
    }
    #[setter]
    fn set_network_rx_bytes(&mut self, v: u64) {
        self.inner.network_rx_bytes = v;
    }
    #[getter]
    fn temperature(&self) -> f32 {
        self.inner.temperature
    }
    #[setter]
    fn set_temperature(&mut self, v: f32) {
        self.inner.temperature = v;
    }
    #[getter]
    fn thread_count(&self) -> u32 {
        self.inner.thread_count
    }
    #[setter]
    fn set_thread_count(&mut self, v: u32) {
        self.inner.thread_count = v;
    }
    #[getter]
    fn timestamp_ns(&self) -> u64 {
        self.inner.timestamp_ns
    }
    #[setter]
    fn set_timestamp_ns(&mut self, v: u64) {
        self.inner.timestamp_ns = v;
    }

    /// Check if CPU usage exceeds the given threshold percentage.
    fn is_cpu_high(&self, threshold: f32) -> bool {
        self.inner.is_cpu_high(threshold)
    }

    /// Check if memory usage exceeds the given threshold percentage.
    fn is_memory_high(&self, threshold: f32) -> bool {
        self.inner.is_memory_high(threshold)
    }

    /// Check if temperature exceeds the given threshold.
    fn is_temperature_high(&self, threshold: f32) -> bool {
        self.inner.is_temperature_high(threshold)
    }

    #[classattr]
    fn __topic_name__() -> &'static str {
        "resources"
    }

    fn __repr__(&self) -> String {
        format!(
            "ResourceUsage(cpu={:.1}%, mem={:.1}%, threads={}, timestamp_ns={})",
            self.inner.cpu_percent,
            self.inner.memory_percent,
            self.inner.thread_count,
            self.inner.timestamp_ns
        )
    }
}

// ============================================================================
// WrenchStamped — POD (Force/Torque)
// ============================================================================

/// Force/torque sensor reading (wrench with reference frame)
#[pyclass(name = "WrenchStamped")]
#[derive(Clone)]
pub struct PyWrenchStamped {
    pub(crate) inner: WrenchStamped,
}

#[pymethods]
impl PyWrenchStamped {
    #[new]
    #[pyo3(signature = (fx=0.0, fy=0.0, fz=0.0, tx=0.0, ty=0.0, tz=0.0, timestamp_ns=0))]

    fn new(fx: f64, fy: f64, fz: f64, tx: f64, ty: f64, tz: f64, timestamp_ns: u64) -> Self {
        let mut w = WrenchStamped::new(Vector3::new(fx, fy, fz), Vector3::new(tx, ty, tz));
        w.timestamp_ns = timestamp_ns;
        Self { inner: w }
    }

    #[getter]
    fn fx(&self) -> f64 {
        self.inner.force.x
    }
    #[setter]
    fn set_fx(&mut self, v: f64) {
        self.inner.force.x = v;
    }
    #[getter]
    fn fy(&self) -> f64 {
        self.inner.force.y
    }
    #[setter]
    fn set_fy(&mut self, v: f64) {
        self.inner.force.y = v;
    }
    #[getter]
    fn fz(&self) -> f64 {
        self.inner.force.z
    }
    #[setter]
    fn set_fz(&mut self, v: f64) {
        self.inner.force.z = v;
    }
    #[getter]
    fn tx(&self) -> f64 {
        self.inner.torque.x
    }
    #[setter]
    fn set_tx(&mut self, v: f64) {
        self.inner.torque.x = v;
    }
    #[getter]
    fn ty(&self) -> f64 {
        self.inner.torque.y
    }
    #[setter]
    fn set_ty(&mut self, v: f64) {
        self.inner.torque.y = v;
    }
    #[getter]
    fn tz(&self) -> f64 {
        self.inner.torque.z
    }
    #[setter]
    fn set_tz(&mut self, v: f64) {
        self.inner.torque.z = v;
    }
    #[getter]
    fn timestamp_ns(&self) -> u64 {
        self.inner.timestamp_ns
    }
    #[setter]
    fn set_timestamp_ns(&mut self, v: u64) {
        self.inner.timestamp_ns = v;
    }

    /// Create a force-only wrench (zero torque).
    #[staticmethod]
    fn force_only(force: &PyVector3) -> Self {
        Self {
            inner: WrenchStamped::force_only(force.inner),
        }
    }

    /// Create a torque-only wrench (zero force).
    #[staticmethod]
    fn torque_only(torque: &PyVector3) -> Self {
        Self {
            inner: WrenchStamped::torque_only(torque.inner),
        }
    }

    /// Euclidean magnitude of the force vector.
    fn force_magnitude(&self) -> f64 {
        self.inner.force_magnitude()
    }

    /// Euclidean magnitude of the torque vector.
    fn torque_magnitude(&self) -> f64 {
        self.inner.torque_magnitude()
    }

    /// Check if force or torque exceed safety limits.
    fn exceeds_limits(&self, max_force: f64, max_torque: f64) -> bool {
        self.inner.exceeds_limits(max_force, max_torque)
    }

    /// Apply exponential moving average filter with previous reading.
    fn filter(&mut self, prev: &PyWrenchStamped, alpha: f64) {
        self.inner.filter(&prev.inner, alpha);
    }

    #[classattr]
    fn __topic_name__() -> &'static str {
        "wrench"
    }

    fn __repr__(&self) -> String {
        format!(
            "WrenchStamped(force=[{:.3},{:.3},{:.3}], torque=[{:.3},{:.3},{:.3}])",
            self.inner.force.x,
            self.inner.force.y,
            self.inner.force.z,
            self.inner.torque.x,
            self.inner.torque.y,
            self.inner.torque.z
        )
    }
}

// ============================================================================
// ForceCommand — POD
// ============================================================================

/// Force control command
#[pyclass(name = "ForceCommand")]
#[derive(Clone)]
pub struct PyForceCommand {
    pub(crate) inner: ForceCommand,
}

#[pymethods]
impl PyForceCommand {
    #[new]
    #[pyo3(signature = (fx=0.0, fy=0.0, fz=0.0, tx=0.0, ty=0.0, tz=0.0, timeout=0.0, timestamp_ns=0))]

    fn new(
        fx: f64,
        fy: f64,
        fz: f64,
        tx: f64,
        ty: f64,
        tz: f64,
        timeout: f64,
        timestamp_ns: u64,
    ) -> Self {
        let mut cmd = ForceCommand::force_only(Vector3::new(fx, fy, fz));
        cmd.target_torque = Vector3::new(tx, ty, tz);
        cmd.timeout_seconds = timeout;
        cmd.timestamp_ns = timestamp_ns;
        Self { inner: cmd }
    }

    #[getter]
    fn fx(&self) -> f64 {
        self.inner.target_force.x
    }
    #[setter]
    fn set_fx(&mut self, v: f64) {
        self.inner.target_force.x = v;
    }
    #[getter]
    fn fy(&self) -> f64 {
        self.inner.target_force.y
    }
    #[setter]
    fn set_fy(&mut self, v: f64) {
        self.inner.target_force.y = v;
    }
    #[getter]
    fn fz(&self) -> f64 {
        self.inner.target_force.z
    }
    #[setter]
    fn set_fz(&mut self, v: f64) {
        self.inner.target_force.z = v;
    }
    #[getter]
    fn tx(&self) -> f64 {
        self.inner.target_torque.x
    }
    #[setter]
    fn set_tx(&mut self, v: f64) {
        self.inner.target_torque.x = v;
    }
    #[getter]
    fn ty(&self) -> f64 {
        self.inner.target_torque.y
    }
    #[setter]
    fn set_ty(&mut self, v: f64) {
        self.inner.target_torque.y = v;
    }
    #[getter]
    fn tz(&self) -> f64 {
        self.inner.target_torque.z
    }
    #[setter]
    fn set_tz(&mut self, v: f64) {
        self.inner.target_torque.z = v;
    }
    #[getter]
    fn timeout_seconds(&self) -> f64 {
        self.inner.timeout_seconds
    }
    #[setter]
    fn set_timeout_seconds(&mut self, v: f64) {
        self.inner.timeout_seconds = v;
    }
    #[getter]
    fn timestamp_ns(&self) -> u64 {
        self.inner.timestamp_ns
    }
    #[setter]
    fn set_timestamp_ns(&mut self, v: u64) {
        self.inner.timestamp_ns = v;
    }

    /// Create a force-only command (zero torque).
    #[staticmethod]
    fn force_only(target: &PyVector3) -> Self {
        Self {
            inner: ForceCommand::force_only(target.inner),
        }
    }

    /// Create a surface contact command along a normal direction.
    #[staticmethod]
    fn surface_contact(normal_force: f64, normal: &PyVector3) -> Self {
        Self {
            inner: ForceCommand::surface_contact(normal_force, normal.inner),
        }
    }

    /// Set a timeout. Returns a new ForceCommand.
    fn with_timeout(&self, timeout_seconds: f64) -> Self {
        Self {
            inner: self.inner.with_timeout(timeout_seconds),
        }
    }

    #[classattr]
    fn __topic_name__() -> &'static str {
        "force_cmd"
    }

    fn __repr__(&self) -> String {
        format!(
            "ForceCommand(force=[{:.3},{:.3},{:.3}], torque=[{:.3},{:.3},{:.3}])",
            self.inner.target_force.x,
            self.inner.target_force.y,
            self.inner.target_force.z,
            self.inner.target_torque.x,
            self.inner.target_torque.y,
            self.inner.target_torque.z
        )
    }
}

// ============================================================================
// ContactInfo — POD
// ============================================================================

/// Contact detection information
#[pyclass(name = "ContactInfo")]
#[derive(Clone)]
pub struct PyContactInfo {
    pub(crate) inner: ContactInfo,
}

#[pymethods]
impl PyContactInfo {
    #[new]
    #[pyo3(signature = (state=0, contact_force=0.0, confidence=0.0, timestamp_ns=0))]
    fn new(state: u8, contact_force: f64, confidence: f32, timestamp_ns: u64) -> Self {
        let mut ci = ContactInfo::new(
            horus_library::messages::force::ContactState::NoContact,
            contact_force,
        );
        ci.state = state;
        ci.confidence = confidence;
        ci.timestamp_ns = timestamp_ns;
        Self { inner: ci }
    }

    #[getter]
    fn state(&self) -> u8 {
        self.inner.state
    }
    #[setter]
    fn set_state(&mut self, v: u8) {
        self.inner.state = v;
    }
    #[getter]
    fn contact_force(&self) -> f64 {
        self.inner.contact_force
    }
    #[setter]
    fn set_contact_force(&mut self, v: f64) {
        self.inner.contact_force = v;
    }
    #[getter]
    fn confidence(&self) -> f32 {
        self.inner.confidence
    }
    #[setter]
    fn set_confidence(&mut self, v: f32) {
        self.inner.confidence = v;
    }
    #[getter]
    fn stiffness(&self) -> f64 {
        self.inner.stiffness
    }
    #[setter]
    fn set_stiffness(&mut self, v: f64) {
        self.inner.stiffness = v;
    }
    #[getter]
    fn damping(&self) -> f64 {
        self.inner.damping
    }
    #[setter]
    fn set_damping(&mut self, v: f64) {
        self.inner.damping = v;
    }
    #[getter]
    fn timestamp_ns(&self) -> u64 {
        self.inner.timestamp_ns
    }
    #[setter]
    fn set_timestamp_ns(&mut self, v: u64) {
        self.inner.timestamp_ns = v;
    }

    fn is_in_contact(&self) -> bool {
        self.inner.is_in_contact()
    }

    /// Duration of current contact in seconds.
    fn contact_duration_seconds(&self) -> f64 {
        self.inner.contact_duration_seconds()
    }

    #[classattr]
    fn __topic_name__() -> &'static str {
        "contact"
    }

    fn __repr__(&self) -> String {
        format!(
            "ContactInfo(state={}, force={:.3}N, confidence={:.2})",
            self.inner.state, self.inner.contact_force, self.inner.confidence
        )
    }
}

// ============================================================================
// NavGoal — POD (Navigation)
// ============================================================================

/// Navigation goal (target pose with tolerances)
#[pyclass(name = "NavGoal")]
#[derive(Clone)]
pub struct PyNavGoal {
    pub(crate) inner: NavGoal,
}

#[pymethods]
impl PyNavGoal {
    #[new]
    #[pyo3(signature = (x=0.0, y=0.0, theta=0.0, position_tolerance=0.1, angle_tolerance=0.1, timeout=30.0, timestamp_ns=0))]

    fn new(
        x: f64,
        y: f64,
        theta: f64,
        position_tolerance: f64,
        angle_tolerance: f64,
        timeout: f64,
        timestamp_ns: u64,
    ) -> Self {
        let mut goal = NavGoal::new(
            Pose2D::new(x, y, theta),
            position_tolerance,
            angle_tolerance,
        );
        goal.timeout_seconds = timeout;
        goal.timestamp_ns = timestamp_ns;
        Self { inner: goal }
    }

    #[getter]
    fn x(&self) -> f64 {
        self.inner.target_pose.x
    }
    #[setter]
    fn set_x(&mut self, v: f64) {
        self.inner.target_pose.x = v;
    }
    #[getter]
    fn y(&self) -> f64 {
        self.inner.target_pose.y
    }
    #[setter]
    fn set_y(&mut self, v: f64) {
        self.inner.target_pose.y = v;
    }
    #[getter]
    fn theta(&self) -> f64 {
        self.inner.target_pose.theta
    }
    #[setter]
    fn set_theta(&mut self, v: f64) {
        self.inner.target_pose.theta = v;
    }
    #[getter]
    fn tolerance_position(&self) -> f64 {
        self.inner.tolerance_position
    }
    #[setter]
    fn set_tolerance_position(&mut self, v: f64) {
        self.inner.tolerance_position = v;
    }
    #[getter]
    fn tolerance_angle(&self) -> f64 {
        self.inner.tolerance_angle
    }
    #[setter]
    fn set_tolerance_angle(&mut self, v: f64) {
        self.inner.tolerance_angle = v;
    }
    #[getter]
    fn timeout_seconds(&self) -> f64 {
        self.inner.timeout_seconds
    }
    #[setter]
    fn set_timeout_seconds(&mut self, v: f64) {
        self.inner.timeout_seconds = v;
    }
    #[getter]
    fn goal_id(&self) -> u32 {
        self.inner.goal_id
    }
    #[setter]
    fn set_goal_id(&mut self, v: u32) {
        self.inner.goal_id = v;
    }
    #[getter]
    fn priority(&self) -> u8 {
        self.inner.priority
    }
    #[setter]
    fn set_priority(&mut self, v: u8) {
        self.inner.priority = v;
    }
    #[getter]
    fn timestamp_ns(&self) -> u64 {
        self.inner.timestamp_ns
    }
    #[setter]
    fn set_timestamp_ns(&mut self, v: u64) {
        self.inner.timestamp_ns = v;
    }

    /// Set a timeout in seconds. Returns a new NavGoal with the timeout applied.
    fn with_timeout(&self, timeout_seconds: f64) -> Self {
        Self {
            inner: self.inner.with_timeout(timeout_seconds),
        }
    }

    /// Set a priority level. Returns a new NavGoal with the priority applied.
    fn with_priority(&self, priority: u8) -> Self {
        Self {
            inner: self.inner.with_priority(priority),
        }
    }

    /// Check if the position tolerance is satisfied.
    fn is_position_reached(&self, current: &PyPose2D) -> bool {
        self.inner.is_position_reached(&current.inner)
    }

    /// Check if the orientation tolerance is satisfied.
    fn is_orientation_reached(&self, current: &PyPose2D) -> bool {
        self.inner.is_orientation_reached(&current.inner)
    }

    /// Check if both position and orientation tolerances are satisfied.
    fn is_reached(&self, current: &PyPose2D) -> bool {
        self.inner.is_reached(&current.inner)
    }

    #[classattr]
    fn __topic_name__() -> &'static str {
        "nav_goal"
    }

    fn __repr__(&self) -> String {
        format!(
            "NavGoal(x={:.3}, y={:.3}, theta={:.3}, timeout={:.1}s)",
            self.inner.target_pose.x,
            self.inner.target_pose.y,
            self.inner.target_pose.theta,
            self.inner.timeout_seconds
        )
    }
}

// ============================================================================
// GoalResult — POD (Navigation)
// ============================================================================

/// Navigation goal result/feedback
#[pyclass(name = "GoalResult")]
#[derive(Clone)]
pub struct PyGoalResult {
    pub(crate) inner: GoalResult,
}

#[pymethods]
impl PyGoalResult {
    #[new]
    #[pyo3(signature = (goal_id=0, status=0, progress=0.0, timestamp_ns=0))]
    fn new(goal_id: u32, status: u8, progress: f32, timestamp_ns: u64) -> Self {
        let goal_status = match status {
            1 => GoalStatus::Active,
            2 => GoalStatus::Succeeded,
            3 => GoalStatus::Aborted,
            4 => GoalStatus::Cancelled,
            5 => GoalStatus::Preempted,
            6 => GoalStatus::TimedOut,
            _ => GoalStatus::Pending,
        };
        let mut gr = GoalResult::new(goal_id, goal_status);
        gr.progress = progress;
        gr.timestamp_ns = timestamp_ns;
        Self { inner: gr }
    }

    #[getter]
    fn goal_id(&self) -> u32 {
        self.inner.goal_id
    }
    #[getter]
    fn status(&self) -> u8 {
        self.inner.status
    }
    #[setter]
    fn set_status(&mut self, v: u8) {
        self.inner.status = v;
    }
    #[getter]
    fn distance_to_goal(&self) -> f64 {
        self.inner.distance_to_goal
    }
    #[setter]
    fn set_distance_to_goal(&mut self, v: f64) {
        self.inner.distance_to_goal = v;
    }
    #[getter]
    fn eta_seconds(&self) -> f64 {
        self.inner.eta_seconds
    }
    #[setter]
    fn set_eta_seconds(&mut self, v: f64) {
        self.inner.eta_seconds = v;
    }
    #[getter]
    fn progress(&self) -> f32 {
        self.inner.progress
    }
    #[setter]
    fn set_progress(&mut self, v: f32) {
        self.inner.progress = v;
    }
    #[getter]
    fn timestamp_ns(&self) -> u64 {
        self.inner.timestamp_ns
    }
    #[setter]
    fn set_timestamp_ns(&mut self, v: u64) {
        self.inner.timestamp_ns = v;
    }

    /// Attach an error message. Returns a new GoalResult with the message set.
    fn with_error(&self, message: &str) -> Self {
        Self {
            inner: self.inner.with_error(message),
        }
    }

    #[classattr]
    fn __topic_name__() -> &'static str {
        "nav_result"
    }

    fn __repr__(&self) -> String {
        let status_str = match self.inner.status {
            0 => "Pending",
            1 => "Active",
            2 => "Succeeded",
            3 => "Aborted",
            4 => "Cancelled",
            _ => "?",
        };
        format!(
            "GoalResult(id={}, status={}, progress={:.1}%)",
            self.inner.goal_id,
            status_str,
            self.inner.progress * 100.0
        )
    }
}

// ============================================================================
// PathPlan — POD (Navigation)
// ============================================================================

/// Path plan with waypoints
#[pyclass(name = "PathPlan")]
#[derive(Clone)]
pub struct PyPathPlan {
    pub(crate) inner: PathPlan,
}

#[pymethods]
impl PyPathPlan {
    #[new]
    #[pyo3(signature = (goal_x=0.0, goal_y=0.0, goal_theta=0.0, timestamp_ns=0))]
    fn new(goal_x: f32, goal_y: f32, goal_theta: f32, timestamp_ns: u64) -> Self {
        let mut pp = PathPlan::new();
        pp.goal_pose = [goal_x, goal_y, goal_theta];
        pp.timestamp_ns = timestamp_ns;
        Self { inner: pp }
    }

    fn add_waypoint(&mut self, x: f32, y: f32, theta: f32) -> bool {
        self.inner.add_waypoint(x, y, theta)
    }

    /// Get waypoint at index as (x, y, theta), or None if out of bounds.
    fn waypoint(&self, index: u16) -> Option<(f32, f32, f32)> {
        self.inner.waypoint(index).map(|w| (w[0], w[1], w[2]))
    }

    /// Create a PathPlan from a list of [x, y, theta] waypoints and a goal.
    #[staticmethod]
    #[pyo3(signature = (waypoints, goal_x=0.0, goal_y=0.0, goal_theta=0.0))]
    fn from_waypoints(waypoints: Vec<[f32; 3]>, goal_x: f32, goal_y: f32, goal_theta: f32) -> Self {
        Self {
            inner: PathPlan::from_waypoints(&waypoints, [goal_x, goal_y, goal_theta]),
        }
    }

    #[getter]
    fn waypoint_count(&self) -> u16 {
        self.inner.waypoint_count
    }
    #[getter]
    fn goal_pose(&self) -> (f32, f32, f32) {
        (
            self.inner.goal_pose[0],
            self.inner.goal_pose[1],
            self.inner.goal_pose[2],
        )
    }
    #[getter]
    fn timestamp_ns(&self) -> u64 {
        self.inner.timestamp_ns
    }
    #[setter]
    fn set_timestamp_ns(&mut self, v: u64) {
        self.inner.timestamp_ns = v;
    }

    fn is_empty(&self) -> bool {
        self.inner.is_empty()
    }

    #[classattr]
    fn __topic_name__() -> &'static str {
        "path_plan"
    }

    fn __repr__(&self) -> String {
        format!(
            "PathPlan({} waypoints, goal=[{:.2},{:.2},{:.2}])",
            self.inner.waypoint_count,
            self.inner.goal_pose[0],
            self.inner.goal_pose[1],
            self.inner.goal_pose[2]
        )
    }
}

// ============================================================================
// JoystickInput — POD (Input)
// ============================================================================

/// Joystick input event
#[pyclass(name = "JoystickInput")]
#[derive(Clone)]
pub struct PyJoystickInput {
    pub(crate) inner: JoystickInput,
}

#[pymethods]
impl PyJoystickInput {
    #[new]
    #[pyo3(signature = (joystick_id=0, element_id=0, value=0.0, pressed=false))]
    fn new(joystick_id: u32, element_id: u32, value: f32, pressed: bool) -> Self {
        let mut ji = JoystickInput::new_axis(joystick_id, element_id, String::new(), value);
        ji.pressed = pressed as u8;
        Self { inner: ji }
    }

    #[getter]
    fn joystick_id(&self) -> u32 {
        self.inner.joystick_id
    }
    #[getter]
    fn element_id(&self) -> u32 {
        self.inner.element_id
    }
    #[getter]
    fn value(&self) -> f32 {
        self.inner.value
    }
    #[setter]
    fn set_value(&mut self, v: f32) {
        self.inner.value = v;
    }
    #[getter]
    fn pressed(&self) -> bool {
        self.inner.is_pressed()
    }
    #[getter]
    fn event_type(&self) -> String {
        self.inner.event_type().to_string()
    }
    #[getter]
    fn element_name(&self) -> String {
        self.inner.element_name().to_string()
    }
    #[getter]
    fn timestamp_ms(&self) -> u64 {
        self.inner.timestamp_ms
    }

    fn is_button(&self) -> bool {
        self.inner.is_button()
    }
    fn is_axis(&self) -> bool {
        self.inner.is_axis()
    }

    /// Check if this is a hat/D-pad event.
    fn is_hat(&self) -> bool {
        self.inner.is_hat()
    }

    /// Check if this is a connection/disconnection event.
    fn is_connection_event(&self) -> bool {
        self.inner.is_connection_event()
    }

    /// Check if the joystick is connected (for connection events).
    fn is_connected(&self) -> bool {
        self.inner.is_connected()
    }

    /// Create a button press/release event.
    #[staticmethod]
    fn new_button(joystick_id: u32, button_id: u32, name: String, pressed: bool) -> Self {
        Self {
            inner: JoystickInput::new_button(joystick_id, button_id, name, pressed),
        }
    }

    /// Create an axis movement event.
    #[staticmethod]
    fn new_axis(joystick_id: u32, axis_id: u32, name: String, value: f32) -> Self {
        Self {
            inner: JoystickInput::new_axis(joystick_id, axis_id, name, value),
        }
    }

    /// Create a hat/D-pad event.
    #[staticmethod]
    fn new_hat(joystick_id: u32, hat_id: u32, name: String, value: f32) -> Self {
        Self {
            inner: JoystickInput::new_hat(joystick_id, hat_id, name, value),
        }
    }

    /// Create a connection/disconnection event.
    #[staticmethod]
    fn new_connection(joystick_id: u32, connected: bool) -> Self {
        Self {
            inner: JoystickInput::new_connection(joystick_id, connected),
        }
    }

    #[classattr]
    fn __topic_name__() -> &'static str {
        "joystick"
    }

    fn __repr__(&self) -> String {
        format!(
            "JoystickInput(id={}, elem={}, value={:.3}, pressed={})",
            self.inner.joystick_id,
            self.inner.element_id,
            self.inner.value,
            self.inner.is_pressed()
        )
    }
}

// ============================================================================
// KeyboardInput — POD (Input)
// ============================================================================

/// Keyboard input event
#[pyclass(name = "KeyboardInput")]
#[derive(Clone)]
pub struct PyKeyboardInput {
    pub(crate) inner: KeyboardInput,
}

#[pymethods]
impl PyKeyboardInput {
    #[new]
    #[pyo3(signature = (key_name="", code=0, pressed=true, modifiers=0))]
    fn new(key_name: &str, code: u32, pressed: bool, modifiers: u32) -> Self {
        let mut ki = KeyboardInput::new(key_name.to_string(), code, vec![], pressed);
        ki.modifier_flags = modifiers;
        Self { inner: ki }
    }

    #[getter]
    fn key_name(&self) -> String {
        self.inner.key_name().to_string()
    }
    #[getter]
    fn code(&self) -> u32 {
        self.inner.code
    }
    #[getter]
    fn pressed(&self) -> bool {
        self.inner.is_pressed()
    }
    #[getter]
    fn modifier_flags(&self) -> u32 {
        self.inner.modifier_flags
    }
    #[getter]
    fn timestamp_ms(&self) -> u64 {
        self.inner.timestamp_ms
    }

    fn is_ctrl(&self) -> bool {
        self.inner.is_ctrl()
    }
    fn is_shift(&self) -> bool {
        self.inner.is_shift()
    }
    fn is_alt(&self) -> bool {
        self.inner.is_alt()
    }

    #[classattr]
    fn __topic_name__() -> &'static str {
        "keyboard"
    }

    fn __repr__(&self) -> String {
        let mods = self.inner.modifiers().join("+");
        if mods.is_empty() {
            format!(
                "KeyboardInput(key='{}', code={}, pressed={})",
                self.inner.key_name(),
                self.inner.code,
                self.inner.is_pressed()
            )
        } else {
            format!(
                "KeyboardInput(key='{}', code={}, mods={}, pressed={})",
                self.inner.key_name(),
                self.inner.code,
                mods,
                self.inner.is_pressed()
            )
        }
    }
}

// ============================================================================
// BoundingBox2D — 16 bytes POD (Detection)
// ============================================================================

/// 2D bounding box for object detection
#[pyclass(name = "BoundingBox2D")]
#[derive(Clone)]
pub struct PyBoundingBox2DMsg {
    pub(crate) inner: BoundingBox2D,
}

#[pymethods]
impl PyBoundingBox2DMsg {
    #[new]
    #[pyo3(signature = (x=0.0, y=0.0, width=0.0, height=0.0))]
    fn new(x: f32, y: f32, width: f32, height: f32) -> Self {
        Self {
            inner: BoundingBox2D::new(x, y, width, height),
        }
    }

    #[staticmethod]
    fn from_center(cx: f32, cy: f32, width: f32, height: f32) -> Self {
        Self {
            inner: BoundingBox2D::from_center(cx, cy, width, height),
        }
    }

    #[getter]
    fn x(&self) -> f32 {
        self.inner.x
    }
    #[setter]
    fn set_x(&mut self, v: f32) {
        self.inner.x = v;
    }
    #[getter]
    fn y(&self) -> f32 {
        self.inner.y
    }
    #[setter]
    fn set_y(&mut self, v: f32) {
        self.inner.y = v;
    }
    #[getter]
    fn width(&self) -> f32 {
        self.inner.width
    }
    #[setter]
    fn set_width(&mut self, v: f32) {
        self.inner.width = v;
    }
    #[getter]
    fn height(&self) -> f32 {
        self.inner.height
    }
    #[setter]
    fn set_height(&mut self, v: f32) {
        self.inner.height = v;
    }
    #[getter]
    fn center_x(&self) -> f32 {
        self.inner.center_x()
    }
    #[getter]
    fn center_y(&self) -> f32 {
        self.inner.center_y()
    }
    #[getter]
    fn area(&self) -> f32 {
        self.inner.area()
    }

    fn iou(&self, other: &PyBoundingBox2DMsg) -> f32 {
        self.inner.iou(&other.inner)
    }

    fn as_tuple(&self) -> (f32, f32, f32, f32) {
        (
            self.inner.x,
            self.inner.y,
            self.inner.width,
            self.inner.height,
        )
    }

    fn as_xyxy(&self) -> (f32, f32, f32, f32) {
        (
            self.inner.x,
            self.inner.y,
            self.inner.x + self.inner.width,
            self.inner.y + self.inner.height,
        )
    }

    #[classattr]
    fn __topic_name__() -> &'static str {
        "bbox2d"
    }

    fn __repr__(&self) -> String {
        format!(
            "BoundingBox2D(x={:.1}, y={:.1}, w={:.1}, h={:.1})",
            self.inner.x, self.inner.y, self.inner.width, self.inner.height
        )
    }
}

// ============================================================================
// BoundingBox3D — 48 bytes POD (Detection)
// ============================================================================

/// 3D bounding box for 3D object detection
#[pyclass(name = "BoundingBox3D")]
#[derive(Clone)]
pub struct PyBoundingBox3D {
    pub(crate) inner: BoundingBox3D,
}

#[pymethods]
impl PyBoundingBox3D {
    #[new]
    #[pyo3(signature = (cx=0.0, cy=0.0, cz=0.0, length=0.0, width=0.0, height=0.0, yaw=0.0))]
    fn new(cx: f32, cy: f32, cz: f32, length: f32, width: f32, height: f32, yaw: f32) -> Self {
        Self {
            inner: BoundingBox3D::new(cx, cy, cz, length, width, height, yaw),
        }
    }

    #[staticmethod]
    #[pyo3(signature = (cx, cy, cz, length, width, height, roll, pitch, yaw))]
    fn with_rotation(
        cx: f32,
        cy: f32,
        cz: f32,
        length: f32,
        width: f32,
        height: f32,
        roll: f32,
        pitch: f32,
        yaw: f32,
    ) -> Self {
        Self {
            inner: BoundingBox3D::with_rotation(
                [cx, cy, cz],
                [length, width, height],
                [roll, pitch, yaw],
            ),
        }
    }

    #[getter]
    fn cx(&self) -> f32 {
        self.inner.cx
    }
    #[setter]
    fn set_cx(&mut self, v: f32) {
        self.inner.cx = v;
    }
    #[getter]
    fn cy(&self) -> f32 {
        self.inner.cy
    }
    #[setter]
    fn set_cy(&mut self, v: f32) {
        self.inner.cy = v;
    }
    #[getter]
    fn cz(&self) -> f32 {
        self.inner.cz
    }
    #[setter]
    fn set_cz(&mut self, v: f32) {
        self.inner.cz = v;
    }
    #[getter]
    fn length(&self) -> f32 {
        self.inner.length
    }
    #[setter]
    fn set_length(&mut self, v: f32) {
        self.inner.length = v;
    }
    #[getter]
    fn width(&self) -> f32 {
        self.inner.width
    }
    #[setter]
    fn set_width(&mut self, v: f32) {
        self.inner.width = v;
    }
    #[getter]
    fn height(&self) -> f32 {
        self.inner.height
    }
    #[setter]
    fn set_height(&mut self, v: f32) {
        self.inner.height = v;
    }
    #[getter]
    fn roll(&self) -> f32 {
        self.inner.roll
    }
    #[setter]
    fn set_roll(&mut self, v: f32) {
        self.inner.roll = v;
    }
    #[getter]
    fn pitch(&self) -> f32 {
        self.inner.pitch
    }
    #[setter]
    fn set_pitch(&mut self, v: f32) {
        self.inner.pitch = v;
    }
    #[getter]
    fn yaw(&self) -> f32 {
        self.inner.yaw
    }
    #[setter]
    fn set_yaw(&mut self, v: f32) {
        self.inner.yaw = v;
    }
    #[getter]
    fn volume(&self) -> f32 {
        self.inner.volume()
    }

    #[classattr]
    fn __topic_name__() -> &'static str {
        "bbox3d"
    }

    fn __repr__(&self) -> String {
        format!(
            "BoundingBox3D(cx={:.2}, cy={:.2}, cz={:.2}, l={:.2}, w={:.2}, h={:.2}, yaw={:.2})",
            self.inner.cx,
            self.inner.cy,
            self.inner.cz,
            self.inner.length,
            self.inner.width,
            self.inner.height,
            self.inner.yaw
        )
    }
}

// ============================================================================
// Detection — 72 bytes POD (Detection)
// ============================================================================

/// 2D object detection result
#[pyclass(name = "Detection")]
#[derive(Clone)]
pub struct PyDetectionMsg {
    pub(crate) inner: Detection,
}

#[pymethods]
impl PyDetectionMsg {
    #[new]
    #[pyo3(signature = (class_name="", confidence=0.0, x=0.0, y=0.0, width=0.0, height=0.0, class_id=0, instance_id=0))]

    fn new(
        class_name: &str,
        confidence: f32,
        x: f32,
        y: f32,
        width: f32,
        height: f32,
        class_id: u32,
        instance_id: u32,
    ) -> Self {
        let mut det = Detection::new(class_name, confidence, x, y, width, height);
        det.class_id = class_id;
        det.instance_id = instance_id;
        Self { inner: det }
    }

    #[getter]
    fn bbox(&self) -> PyBoundingBox2DMsg {
        PyBoundingBox2DMsg {
            inner: self.inner.bbox,
        }
    }
    #[setter]
    fn set_bbox(&mut self, bbox: PyBoundingBox2DMsg) {
        self.inner.bbox = bbox.inner;
    }
    #[getter]
    fn confidence(&self) -> f32 {
        self.inner.confidence
    }
    #[setter]
    fn set_confidence(&mut self, v: f32) {
        self.inner.confidence = v;
    }
    #[getter]
    fn class_id(&self) -> u32 {
        self.inner.class_id
    }
    #[setter]
    fn set_class_id(&mut self, v: u32) {
        self.inner.class_id = v;
    }
    #[getter]
    fn class_name(&self) -> String {
        self.inner.class_name().to_string()
    }
    #[setter]
    fn set_class_name(&mut self, name: &str) {
        self.inner.set_class_name(name);
    }
    #[getter]
    fn instance_id(&self) -> u32 {
        self.inner.instance_id
    }
    #[setter]
    fn set_instance_id(&mut self, v: u32) {
        self.inner.instance_id = v;
    }

    fn is_confident(&self, threshold: f32) -> bool {
        self.inner.confidence >= threshold
    }

    /// Set the class ID. Returns a new Detection.
    fn with_class_id(&self, class_id: u32) -> Self {
        let mut inner = self.inner;
        inner.class_id = class_id;
        Self { inner }
    }

    #[classattr]
    fn __topic_name__() -> &'static str {
        "detection"
    }

    fn __repr__(&self) -> String {
        format!(
            "Detection('{}', conf={:.2}, bbox=({:.0}, {:.0}, {:.0}, {:.0}))",
            self.inner.class_name(),
            self.inner.confidence,
            self.inner.bbox.x,
            self.inner.bbox.y,
            self.inner.bbox.width,
            self.inner.bbox.height
        )
    }
}

// ============================================================================
// Detection3D — 104 bytes POD (Detection)
// ============================================================================

/// 3D object detection result
#[pyclass(name = "Detection3D")]
#[derive(Clone)]
pub struct PyDetection3D {
    pub(crate) inner: Detection3D,
}

#[pymethods]
impl PyDetection3D {
    #[new]
    #[pyo3(signature = (class_name="", confidence=0.0, cx=0.0, cy=0.0, cz=0.0, length=0.0, width=0.0, height=0.0, yaw=0.0))]

    fn new(
        class_name: &str,
        confidence: f32,
        cx: f32,
        cy: f32,
        cz: f32,
        length: f32,
        width: f32,
        height: f32,
        yaw: f32,
    ) -> Self {
        let bbox = BoundingBox3D::new(cx, cy, cz, length, width, height, yaw);
        Self {
            inner: Detection3D::new(class_name, confidence, bbox),
        }
    }

    #[getter]
    fn bbox(&self) -> PyBoundingBox3D {
        PyBoundingBox3D {
            inner: self.inner.bbox,
        }
    }
    #[setter]
    fn set_bbox(&mut self, bbox: PyBoundingBox3D) {
        self.inner.bbox = bbox.inner;
    }
    #[getter]
    fn confidence(&self) -> f32 {
        self.inner.confidence
    }
    #[setter]
    fn set_confidence(&mut self, v: f32) {
        self.inner.confidence = v;
    }
    #[getter]
    fn class_id(&self) -> u32 {
        self.inner.class_id
    }
    #[setter]
    fn set_class_id(&mut self, v: u32) {
        self.inner.class_id = v;
    }
    #[getter]
    fn class_name(&self) -> String {
        self.inner.class_name().to_string()
    }
    #[setter]
    fn set_class_name(&mut self, name: &str) {
        self.inner.set_class_name(name);
    }
    #[getter]
    fn velocity_x(&self) -> f32 {
        self.inner.velocity_x
    }
    #[setter]
    fn set_velocity_x(&mut self, v: f32) {
        self.inner.velocity_x = v;
    }
    #[getter]
    fn velocity_y(&self) -> f32 {
        self.inner.velocity_y
    }
    #[setter]
    fn set_velocity_y(&mut self, v: f32) {
        self.inner.velocity_y = v;
    }
    #[getter]
    fn velocity_z(&self) -> f32 {
        self.inner.velocity_z
    }
    #[setter]
    fn set_velocity_z(&mut self, v: f32) {
        self.inner.velocity_z = v;
    }
    #[getter]
    fn instance_id(&self) -> u32 {
        self.inner.instance_id
    }
    #[setter]
    fn set_instance_id(&mut self, v: u32) {
        self.inner.instance_id = v;
    }

    /// Set velocity components. Returns a new Detection3D.
    fn with_velocity(&self, vx: f32, vy: f32, vz: f32) -> Self {
        let mut inner = self.inner;
        inner.velocity_x = vx;
        inner.velocity_y = vy;
        inner.velocity_z = vz;
        Self { inner }
    }

    #[classattr]
    fn __topic_name__() -> &'static str {
        "detection3d"
    }

    fn __repr__(&self) -> String {
        format!(
            "Detection3D('{}', conf={:.2}, pos=({:.2}, {:.2}, {:.2}))",
            self.inner.class_name(),
            self.inner.confidence,
            self.inner.bbox.cx,
            self.inner.bbox.cy,
            self.inner.bbox.cz
        )
    }
}

// ============================================================================
// SegmentationMask — 64 bytes POD (Segmentation)
// ============================================================================

/// Segmentation mask header (semantic, instance, or panoptic)
#[pyclass(name = "SegmentationMask")]
#[derive(Clone)]
pub struct PySegmentationMask {
    pub(crate) inner: SegmentationMask,
}

#[pymethods]
impl PySegmentationMask {
    #[new]
    #[pyo3(signature = (width=0, height=0, mask_type=0, num_classes=0))]
    fn new(width: u32, height: u32, mask_type: u32, num_classes: u32) -> Self {
        let inner = match mask_type {
            1 => SegmentationMask::instance(width, height),
            2 => SegmentationMask::panoptic(width, height, num_classes),
            _ => SegmentationMask::semantic(width, height, num_classes),
        };
        Self { inner }
    }

    #[staticmethod]
    fn semantic(width: u32, height: u32, num_classes: u32) -> Self {
        Self {
            inner: SegmentationMask::semantic(width, height, num_classes),
        }
    }

    #[staticmethod]
    fn instance(width: u32, height: u32) -> Self {
        Self {
            inner: SegmentationMask::instance(width, height),
        }
    }

    #[staticmethod]
    fn panoptic(width: u32, height: u32, num_classes: u32) -> Self {
        Self {
            inner: SegmentationMask::panoptic(width, height, num_classes),
        }
    }

    #[getter]
    fn width(&self) -> u32 {
        self.inner.width
    }
    #[setter]
    fn set_width(&mut self, v: u32) {
        self.inner.width = v;
    }
    #[getter]
    fn height(&self) -> u32 {
        self.inner.height
    }
    #[setter]
    fn set_height(&mut self, v: u32) {
        self.inner.height = v;
    }
    #[getter]
    fn num_classes(&self) -> u32 {
        self.inner.num_classes
    }
    #[setter]
    fn set_num_classes(&mut self, v: u32) {
        self.inner.num_classes = v;
    }
    #[getter]
    fn mask_type(&self) -> u32 {
        self.inner.mask_type
    }
    #[getter]
    fn timestamp_ns(&self) -> u64 {
        self.inner.timestamp_ns
    }
    #[setter]
    fn set_timestamp_ns(&mut self, v: u64) {
        self.inner.timestamp_ns = v;
    }
    #[getter]
    fn seq(&self) -> u64 {
        self.inner.seq
    }
    #[setter]
    fn set_seq(&mut self, v: u64) {
        self.inner.seq = v;
    }
    #[getter]
    fn frame_id(&self) -> String {
        self.inner.frame_id().to_string()
    }

    fn is_semantic(&self) -> bool {
        self.inner.is_semantic()
    }
    fn is_instance(&self) -> bool {
        self.inner.is_instance()
    }
    fn is_panoptic(&self) -> bool {
        self.inner.is_panoptic()
    }
    fn data_size(&self) -> usize {
        self.inner.data_size()
    }

    /// Size of the u16 data array (for instance/panoptic masks).
    fn data_size_u16(&self) -> usize {
        self.inner.data_size_u16()
    }

    #[classattr]
    fn __topic_name__() -> &'static str {
        "segmentation_mask"
    }

    fn __repr__(&self) -> String {
        let kind = match self.inner.mask_type {
            0 => "semantic",
            1 => "instance",
            2 => "panoptic",
            _ => "unknown",
        };
        format!(
            "SegmentationMask({}x{}, {}, {} classes)",
            self.inner.width, self.inner.height, kind, self.inner.num_classes
        )
    }
}

// ============================================================================
// TrackedObject — 96 bytes POD (Tracking)
// ============================================================================

/// Tracked object with ID, velocity, and state
#[pyclass(name = "TrackedObject")]
#[derive(Clone)]
pub struct PyTrackedObjectMsg {
    pub(crate) inner: TrackedObject,
}

#[pymethods]
impl PyTrackedObjectMsg {
    #[new]
    #[pyo3(signature = (track_id=0, x=0.0, y=0.0, width=0.0, height=0.0, class_id=0, confidence=0.0))]
    fn new(
        track_id: u64,
        x: f32,
        y: f32,
        width: f32,
        height: f32,
        class_id: u32,
        confidence: f32,
    ) -> Self {
        let bbox = BoundingBox2D::new(x, y, width, height);
        Self {
            inner: TrackedObject::new(track_id, bbox, class_id, confidence),
        }
    }

    #[getter]
    fn bbox(&self) -> PyBoundingBox2DMsg {
        PyBoundingBox2DMsg {
            inner: self.inner.bbox,
        }
    }
    #[getter]
    fn predicted_bbox(&self) -> PyBoundingBox2DMsg {
        PyBoundingBox2DMsg {
            inner: self.inner.predicted_bbox,
        }
    }
    #[getter]
    fn track_id(&self) -> u64 {
        self.inner.track_id
    }
    #[getter]
    fn confidence(&self) -> f32 {
        self.inner.confidence
    }
    #[getter]
    fn class_id(&self) -> u32 {
        self.inner.class_id
    }
    #[getter]
    fn class_name(&self) -> String {
        self.inner.class_name().to_string()
    }
    #[setter]
    fn set_class_name(&mut self, name: &str) {
        self.inner.set_class_name(name);
    }
    #[getter]
    fn velocity_x(&self) -> f32 {
        self.inner.velocity_x
    }
    #[getter]
    fn velocity_y(&self) -> f32 {
        self.inner.velocity_y
    }
    #[getter]
    fn velocity(&self) -> (f32, f32) {
        (self.inner.velocity_x, self.inner.velocity_y)
    }
    #[getter]
    fn accel_x(&self) -> f32 {
        self.inner.accel_x
    }
    #[getter]
    fn accel_y(&self) -> f32 {
        self.inner.accel_y
    }
    #[getter]
    fn age(&self) -> u32 {
        self.inner.age
    }
    #[getter]
    fn hits(&self) -> u32 {
        self.inner.hits
    }
    #[getter]
    fn time_since_update(&self) -> u32 {
        self.inner.time_since_update
    }
    #[getter]
    fn state(&self) -> u32 {
        self.inner.state
    }

    fn speed(&self) -> f32 {
        self.inner.speed()
    }
    fn heading(&self) -> f32 {
        self.inner.heading()
    }
    fn is_tentative(&self) -> bool {
        self.inner.is_tentative()
    }
    fn is_confirmed(&self) -> bool {
        self.inner.is_confirmed()
    }
    fn is_deleted(&self) -> bool {
        self.inner.is_deleted()
    }

    fn confirm(&mut self) {
        self.inner.confirm();
    }
    fn delete(&mut self) {
        self.inner.delete();
    }
    fn mark_missed(&mut self) {
        self.inner.mark_missed();
    }

    fn update(&mut self, bbox: &PyBoundingBox2DMsg, confidence: f32) {
        self.inner.update(bbox.inner, confidence);
    }

    #[classattr]
    fn __topic_name__() -> &'static str {
        "tracked_object"
    }

    fn __repr__(&self) -> String {
        format!(
            "TrackedObject(id={}, '{}', conf={:.2}, age={})",
            self.inner.track_id,
            self.inner.class_name(),
            self.inner.confidence,
            self.inner.age
        )
    }
}

// ============================================================================
// TrackingHeader — 32 bytes POD (Tracking)
// ============================================================================

/// Header for tracking frame data
#[pyclass(name = "TrackingHeader")]
#[derive(Clone)]
pub struct PyTrackingHeader {
    pub(crate) inner: TrackingHeader,
}

#[pymethods]
impl PyTrackingHeader {
    #[new]
    #[pyo3(signature = (num_tracks=0, frame_id=0))]
    fn new(num_tracks: u32, frame_id: u32) -> Self {
        Self {
            inner: TrackingHeader::new(num_tracks, frame_id),
        }
    }

    #[getter]
    fn num_tracks(&self) -> u32 {
        self.inner.num_tracks
    }
    #[setter]
    fn set_num_tracks(&mut self, v: u32) {
        self.inner.num_tracks = v;
    }
    #[getter]
    fn frame_id(&self) -> u32 {
        self.inner.frame_id
    }
    #[setter]
    fn set_frame_id(&mut self, v: u32) {
        self.inner.frame_id = v;
    }
    #[getter]
    fn timestamp_ns(&self) -> u64 {
        self.inner.timestamp_ns
    }
    #[setter]
    fn set_timestamp_ns(&mut self, v: u64) {
        self.inner.timestamp_ns = v;
    }
    #[getter]
    fn total_tracks(&self) -> u64 {
        self.inner.total_tracks
    }
    #[setter]
    fn set_total_tracks(&mut self, v: u64) {
        self.inner.total_tracks = v;
    }
    #[getter]
    fn active_tracks(&self) -> u32 {
        self.inner.active_tracks
    }
    #[setter]
    fn set_active_tracks(&mut self, v: u32) {
        self.inner.active_tracks = v;
    }

    fn data_size(&self) -> usize {
        self.inner.data_size()
    }

    #[classattr]
    fn __topic_name__() -> &'static str {
        "tracking_header"
    }

    fn __repr__(&self) -> String {
        format!(
            "TrackingHeader(tracks={}, frame={}, active={})",
            self.inner.num_tracks, self.inner.frame_id, self.inner.active_tracks
        )
    }
}

// ============================================================================
// Landmark — 16 bytes POD (Landmark)
// ============================================================================

/// 2D landmark/keypoint for pose estimation
#[pyclass(name = "Landmark")]
#[derive(Clone)]
pub struct PyLandmarkMsg {
    pub(crate) inner: Landmark,
}

#[pymethods]
impl PyLandmarkMsg {
    #[new]
    #[pyo3(signature = (x=0.0, y=0.0, visibility=1.0, index=0))]
    fn new(x: f32, y: f32, visibility: f32, index: u32) -> Self {
        Self {
            inner: Landmark::new(x, y, visibility, index),
        }
    }

    #[staticmethod]
    fn visible(x: f32, y: f32, index: u32) -> Self {
        Self {
            inner: Landmark::visible(x, y, index),
        }
    }

    #[getter]
    fn x(&self) -> f32 {
        self.inner.x
    }
    #[setter]
    fn set_x(&mut self, v: f32) {
        self.inner.x = v;
    }
    #[getter]
    fn y(&self) -> f32 {
        self.inner.y
    }
    #[setter]
    fn set_y(&mut self, v: f32) {
        self.inner.y = v;
    }
    #[getter]
    fn visibility(&self) -> f32 {
        self.inner.visibility
    }
    #[setter]
    fn set_visibility(&mut self, v: f32) {
        self.inner.visibility = v;
    }
    #[getter]
    fn index(&self) -> u32 {
        self.inner.index
    }
    #[setter]
    fn set_index(&mut self, v: u32) {
        self.inner.index = v;
    }

    fn is_visible(&self, threshold: f32) -> bool {
        self.inner.is_visible(threshold)
    }
    fn distance_to(&self, other: &PyLandmarkMsg) -> f32 {
        self.inner.distance_to(&other.inner)
    }

    #[classattr]
    fn __topic_name__() -> &'static str {
        "landmark"
    }

    fn __repr__(&self) -> String {
        format!(
            "Landmark({:.1}, {:.1}, vis={:.2}, idx={})",
            self.inner.x, self.inner.y, self.inner.visibility, self.inner.index
        )
    }
}

// ============================================================================
// Landmark3D — 20 bytes POD (Landmark)
// ============================================================================

/// 3D landmark/keypoint for pose estimation
#[pyclass(name = "Landmark3D")]
#[derive(Clone)]
pub struct PyLandmark3D {
    pub(crate) inner: Landmark3D,
}

#[pymethods]
impl PyLandmark3D {
    #[new]
    #[pyo3(signature = (x=0.0, y=0.0, z=0.0, visibility=1.0, index=0))]
    fn new(x: f32, y: f32, z: f32, visibility: f32, index: u32) -> Self {
        Self {
            inner: Landmark3D::new(x, y, z, visibility, index),
        }
    }

    #[staticmethod]
    fn visible(x: f32, y: f32, z: f32, index: u32) -> Self {
        Self {
            inner: Landmark3D::visible(x, y, z, index),
        }
    }

    // Landmark3D is #[repr(C, packed)] — must copy fields to locals to avoid unaligned refs
    #[getter]
    fn x(&self) -> f32 {
        self.inner.x
    }
    #[setter]
    fn set_x(&mut self, v: f32) {
        self.inner.x = v;
    }
    #[getter]
    fn y(&self) -> f32 {
        self.inner.y
    }
    #[setter]
    fn set_y(&mut self, v: f32) {
        self.inner.y = v;
    }
    #[getter]
    fn z(&self) -> f32 {
        self.inner.z
    }
    #[setter]
    fn set_z(&mut self, v: f32) {
        self.inner.z = v;
    }
    #[getter]
    fn visibility(&self) -> f32 {
        self.inner.visibility
    }
    #[setter]
    fn set_visibility(&mut self, v: f32) {
        self.inner.visibility = v;
    }
    #[getter]
    fn index(&self) -> u32 {
        self.inner.index
    }
    #[setter]
    fn set_index(&mut self, v: u32) {
        self.inner.index = v;
    }

    fn is_visible(&self, threshold: f32) -> bool {
        self.inner.is_visible(threshold)
    }
    fn distance_to(&self, other: &PyLandmark3D) -> f32 {
        self.inner.distance_to(&other.inner)
    }
    fn to_2d(&self) -> PyLandmarkMsg {
        PyLandmarkMsg {
            inner: self.inner.to_2d(),
        }
    }

    #[classattr]
    fn __topic_name__() -> &'static str {
        "landmark3d"
    }

    fn __repr__(&self) -> String {
        let (x, y, z, vis, idx) = (
            self.inner.x,
            self.inner.y,
            self.inner.z,
            self.inner.visibility,
            self.inner.index,
        );
        format!(
            "Landmark3D({:.1}, {:.1}, {:.1}, vis={:.2}, idx={})",
            x, y, z, vis, idx
        )
    }
}

// ============================================================================
// LandmarkArray — 40 bytes POD (Landmark)
// ============================================================================

/// Landmark array header for pose estimation (COCO, MediaPipe, etc.)
#[pyclass(name = "LandmarkArray")]
#[derive(Clone)]
pub struct PyLandmarkArray {
    pub(crate) inner: LandmarkArray,
}

#[pymethods]
impl PyLandmarkArray {
    #[new]
    #[pyo3(signature = (num_landmarks=17, dimension=2))]
    fn new(num_landmarks: u32, dimension: u32) -> Self {
        let inner = if dimension == 3 {
            LandmarkArray::new_3d(num_landmarks)
        } else {
            LandmarkArray::new_2d(num_landmarks)
        };
        Self { inner }
    }

    #[staticmethod]
    fn coco_pose() -> Self {
        Self {
            inner: LandmarkArray::coco_pose(),
        }
    }
    #[staticmethod]
    fn mediapipe_pose() -> Self {
        Self {
            inner: LandmarkArray::mediapipe_pose(),
        }
    }
    #[staticmethod]
    fn mediapipe_hand() -> Self {
        Self {
            inner: LandmarkArray::mediapipe_hand(),
        }
    }
    #[staticmethod]
    fn mediapipe_face() -> Self {
        Self {
            inner: LandmarkArray::mediapipe_face(),
        }
    }

    #[getter]
    fn num_landmarks(&self) -> u32 {
        self.inner.num_landmarks
    }
    #[getter]
    fn dimension(&self) -> u32 {
        self.inner.dimension
    }
    #[getter]
    fn instance_id(&self) -> u32 {
        self.inner.instance_id
    }
    #[setter]
    fn set_instance_id(&mut self, v: u32) {
        self.inner.instance_id = v;
    }
    #[getter]
    fn confidence(&self) -> f32 {
        self.inner.confidence
    }
    #[setter]
    fn set_confidence(&mut self, v: f32) {
        self.inner.confidence = v;
    }
    #[getter]
    fn timestamp_ns(&self) -> u64 {
        self.inner.timestamp_ns
    }
    #[setter]
    fn set_timestamp_ns(&mut self, v: u64) {
        self.inner.timestamp_ns = v;
    }
    #[getter]
    fn bbox_x(&self) -> f32 {
        self.inner.bbox_x
    }
    #[getter]
    fn bbox_y(&self) -> f32 {
        self.inner.bbox_y
    }
    #[getter]
    fn bbox_width(&self) -> f32 {
        self.inner.bbox_width
    }
    #[getter]
    fn bbox_height(&self) -> f32 {
        self.inner.bbox_height
    }

    fn data_size(&self) -> usize {
        self.inner.data_size()
    }

    #[classattr]
    fn __topic_name__() -> &'static str {
        "landmark_array"
    }

    fn __repr__(&self) -> String {
        format!(
            "LandmarkArray(n={}, dim={}, conf={:.2})",
            self.inner.num_landmarks, self.inner.dimension, self.inner.confidence
        )
    }
}

// ============================================================================
// PointField — Perception helper (serde-based)
// ============================================================================

/// Point cloud field descriptor
#[pyclass(name = "PointField")]
#[derive(Clone)]
pub struct PyPointField {
    pub(crate) inner: PointField,
}

#[pymethods]
impl PyPointField {
    #[new]
    #[pyo3(signature = (name="", offset=0, datatype=0, count=1))]
    fn new(name: &str, offset: u32, datatype: u8, count: u32) -> Self {
        let dtype = horus_core::types::TensorDtype::parse(match datatype {
            0 => "f32",
            1 => "f64",
            2 => "f16",
            3 => "bf16",
            4 => "i8",
            5 => "i16",
            6 => "i32",
            7 => "i64",
            8 => "u8",
            9 => "u16",
            10 => "u32",
            _ => "f32",
        })
        .unwrap_or_default();
        Self {
            inner: PointField::new(name, offset, dtype, count),
        }
    }

    #[getter]
    fn name(&self) -> String {
        self.inner.name_str().to_string()
    }
    #[getter]
    fn offset(&self) -> u32 {
        self.inner.offset
    }
    #[setter]
    fn set_offset(&mut self, v: u32) {
        self.inner.offset = v;
    }
    #[getter]
    fn datatype(&self) -> u8 {
        self.inner.datatype as u8
    }
    #[getter]
    fn count(&self) -> u32 {
        self.inner.count
    }
    #[setter]
    fn set_count(&mut self, v: u32) {
        self.inner.count = v;
    }

    fn field_size(&self) -> u32 {
        self.inner.field_size()
    }

    #[classattr]
    fn __topic_name__() -> &'static str {
        "point_field"
    }

    fn __repr__(&self) -> String {
        format!(
            "PointField('{}', offset={}, dtype={}, count={})",
            self.inner.name_str(),
            self.inner.offset,
            self.inner.datatype as u8,
            self.inner.count
        )
    }
}

// ============================================================================
// PlaneDetection — Perception (serde-based)
// ============================================================================

/// Detected plane in 3D space
#[pyclass(name = "PlaneDetection")]
#[derive(Clone)]
pub struct PyPlaneDetection {
    pub(crate) inner: PlaneDetection,
}

#[pymethods]
impl PyPlaneDetection {
    #[new]
    #[pyo3(signature = (a=0.0, b=0.0, c=1.0, d=0.0, cx=0.0, cy=0.0, cz=0.0, nx=0.0, ny=0.0, nz=1.0))]

    fn new(
        a: f64,
        b: f64,
        c: f64,
        d: f64,
        cx: f64,
        cy: f64,
        cz: f64,
        nx: f64,
        ny: f64,
        nz: f64,
    ) -> Self {
        use horus_library::messages::geometry::{Point3, Vector3};
        Self {
            inner: PlaneDetection::new(
                [a, b, c, d],
                Point3::new(cx, cy, cz),
                Vector3::new(nx, ny, nz),
            ),
        }
    }

    #[getter]
    fn coefficients(&self) -> (f64, f64, f64, f64) {
        let c = self.inner.coefficients;
        (c[0], c[1], c[2], c[3])
    }
    #[getter]
    fn center(&self) -> PyPoint3 {
        PyPoint3 {
            inner: self.inner.center,
        }
    }
    #[getter]
    fn normal(&self) -> PyVector3 {
        PyVector3 {
            inner: self.inner.normal,
        }
    }
    #[getter]
    fn inlier_count(&self) -> u32 {
        self.inner.inlier_count
    }
    #[setter]
    fn set_inlier_count(&mut self, v: u32) {
        self.inner.inlier_count = v;
    }
    #[getter]
    fn confidence(&self) -> f32 {
        self.inner.confidence
    }
    #[setter]
    fn set_confidence(&mut self, v: f32) {
        self.inner.confidence = v;
    }
    #[getter]
    fn plane_type(&self) -> String {
        self.inner.plane_type_str().to_string()
    }
    #[getter]
    fn timestamp_ns(&self) -> u64 {
        self.inner.timestamp_ns
    }
    #[setter]
    fn set_timestamp_ns(&mut self, v: u64) {
        self.inner.timestamp_ns = v;
    }

    /// Signed distance from a point to this plane.
    fn distance_to_point(&self, px: f64, py: f64, pz: f64) -> f64 {
        self.inner.distance_to_point(&Point3::new(px, py, pz))
    }

    /// Check if a point lies on or near this plane within tolerance.
    fn contains_point(&self, px: f64, py: f64, pz: f64, tolerance: f64) -> bool {
        self.inner
            .contains_point(&Point3::new(px, py, pz), tolerance)
    }

    #[classattr]
    fn __topic_name__() -> &'static str {
        "plane_detection"
    }

    fn __repr__(&self) -> String {
        format!(
            "PlaneDetection(type='{}', conf={:.2}, inliers={})",
            self.inner.plane_type_str(),
            self.inner.confidence,
            self.inner.inlier_count
        )
    }
}

// ============================================================================
// PlaneArray — Perception (serde-based, variable size)
// ============================================================================

/// Array of up to 16 detected planes
#[pyclass(name = "PlaneArray")]
#[derive(Clone)]
pub struct PyPlaneArray {
    pub(crate) inner: PlaneArray,
}

#[pymethods]
impl PyPlaneArray {
    #[new]
    fn new() -> Self {
        Self {
            inner: PlaneArray::default(),
        }
    }

    #[getter]
    fn count(&self) -> u8 {
        self.inner.count
    }
    #[getter]
    fn frame_id(&self) -> String {
        let bytes = &self.inner.frame_id;
        let end = bytes.iter().position(|&b| b == 0).unwrap_or(bytes.len());
        String::from_utf8_lossy(&bytes[..end]).to_string()
    }
    #[getter]
    fn timestamp_ns(&self) -> u64 {
        self.inner.timestamp_ns
    }
    #[setter]
    fn set_timestamp_ns(&mut self, v: u64) {
        self.inner.timestamp_ns = v;
    }

    fn __len__(&self) -> usize {
        self.inner.count as usize
    }

    fn __getitem__(&self, index: usize) -> PyResult<PyPlaneDetection> {
        if index >= self.inner.count as usize {
            return Err(pyo3::exceptions::PyIndexError::new_err(
                "Index out of range",
            ));
        }
        Ok(PyPlaneDetection {
            inner: self.inner.planes[index],
        })
    }

    fn add_plane(&mut self, plane: &PyPlaneDetection) -> PyResult<()> {
        if self.inner.count >= 16 {
            return Err(pyo3::exceptions::PyValueError::new_err(
                "PlaneArray is full (max 16)",
            ));
        }
        self.inner.planes[self.inner.count as usize] = plane.inner;
        self.inner.count += 1;
        Ok(())
    }

    #[classattr]
    fn __topic_name__() -> &'static str {
        "plane_array"
    }

    fn __repr__(&self) -> String {
        format!("PlaneArray({} planes)", self.inner.count)
    }
}

// ============================================================================
// CompressedImage — Vision compressed image
// ============================================================================

#[pyclass(name = "CompressedImage")]
#[derive(Clone)]
pub struct PyCompressedImage {
    pub(crate) inner: CompressedImage,
}

#[pymethods]
impl PyCompressedImage {
    #[new]
    #[pyo3(signature = (format="jpeg", data=vec![], width=0, height=0))]
    fn new(format: &str, data: Vec<u8>, width: u32, height: u32) -> Self {
        let mut img = CompressedImage::new(format, data);
        img.width = width;
        img.height = height;
        Self { inner: img }
    }
    #[getter]
    fn format(&self) -> String {
        self.inner.format_str()
    }
    #[getter]
    fn data(&self) -> Vec<u8> {
        self.inner.data.clone()
    }
    #[setter]
    fn set_data(&mut self, v: Vec<u8>) {
        self.inner.data = v;
    }
    #[getter]
    fn width(&self) -> u32 {
        self.inner.width
    }
    #[setter]
    fn set_width(&mut self, v: u32) {
        self.inner.width = v;
    }
    #[getter]
    fn height(&self) -> u32 {
        self.inner.height
    }
    #[setter]
    fn set_height(&mut self, v: u32) {
        self.inner.height = v;
    }
    #[getter]
    fn timestamp_ns(&self) -> u64 {
        self.inner.timestamp_ns
    }
    #[setter]
    fn set_timestamp_ns(&mut self, v: u64) {
        self.inner.timestamp_ns = v;
    }
    fn data_len(&self) -> usize {
        self.inner.data.len()
    }
    #[classattr]
    fn __topic_name__() -> &'static str {
        "compressed_image"
    }
    fn __repr__(&self) -> String {
        format!(
            "CompressedImage({}, {}x{}, {} bytes)",
            self.inner.format_str(),
            self.inner.width,
            self.inner.height,
            self.inner.data.len()
        )
    }
}

// ============================================================================
// CameraInfo — Camera calibration
// ============================================================================

#[pyclass(name = "CameraInfo")]
#[derive(Clone)]
pub struct PyCameraInfo {
    pub(crate) inner: CameraInfo,
}

#[pymethods]
impl PyCameraInfo {
    #[new]
    #[pyo3(signature = (width=0, height=0, fx=0.0, fy=0.0, cx=0.0, cy=0.0))]
    fn new(width: u32, height: u32, fx: f64, fy: f64, cx: f64, cy: f64) -> Self {
        Self {
            inner: CameraInfo::new(width, height, fx, fy, cx, cy),
        }
    }
    #[getter]
    fn width(&self) -> u32 {
        self.inner.width
    }
    #[setter]
    fn set_width(&mut self, v: u32) {
        self.inner.width = v;
    }
    #[getter]
    fn height(&self) -> u32 {
        self.inner.height
    }
    #[setter]
    fn set_height(&mut self, v: u32) {
        self.inner.height = v;
    }
    #[getter]
    fn camera_matrix(&self) -> Vec<f64> {
        self.inner.camera_matrix.to_vec()
    }
    #[getter]
    fn distortion_coefficients(&self) -> Vec<f64> {
        self.inner.distortion_coefficients.to_vec()
    }
    #[getter]
    fn rectification_matrix(&self) -> Vec<f64> {
        self.inner.rectification_matrix.to_vec()
    }
    #[getter]
    fn projection_matrix(&self) -> Vec<f64> {
        self.inner.projection_matrix.to_vec()
    }
    fn focal_lengths(&self) -> (f64, f64) {
        self.inner.focal_lengths()
    }
    fn principal_point(&self) -> (f64, f64) {
        self.inner.principal_point()
    }
    #[getter]
    fn timestamp_ns(&self) -> u64 {
        self.inner.timestamp_ns
    }
    #[setter]
    fn set_timestamp_ns(&mut self, v: u64) {
        self.inner.timestamp_ns = v;
    }
    /// Set the distortion model name. Returns a new CameraInfo.
    fn with_distortion_model(&self, model: &str) -> Self {
        Self {
            inner: self.inner.with_distortion_model(model),
        }
    }

    #[classattr]
    fn __topic_name__() -> &'static str {
        "camera_info"
    }
    fn __repr__(&self) -> String {
        let (fx, fy) = self.inner.focal_lengths();
        format!(
            "CameraInfo({}x{}, f={:.1}/{:.1})",
            self.inner.width, self.inner.height, fx, fy
        )
    }
}

// ============================================================================
// RegionOfInterest — Vision ROI
// ============================================================================

#[pyclass(name = "RegionOfInterest")]
#[derive(Clone)]
pub struct PyRegionOfInterest {
    pub(crate) inner: RegionOfInterest,
}

#[pymethods]
impl PyRegionOfInterest {
    #[new]
    #[pyo3(signature = (x=0, y=0, width=0, height=0, do_rectify=false))]
    fn new(x: u32, y: u32, width: u32, height: u32, do_rectify: bool) -> Self {
        let mut roi = RegionOfInterest::new(x, y, width, height);
        roi.do_rectify = do_rectify;
        Self { inner: roi }
    }
    #[getter]
    fn x_offset(&self) -> u32 {
        self.inner.x_offset
    }
    #[setter]
    fn set_x_offset(&mut self, v: u32) {
        self.inner.x_offset = v;
    }
    #[getter]
    fn y_offset(&self) -> u32 {
        self.inner.y_offset
    }
    #[setter]
    fn set_y_offset(&mut self, v: u32) {
        self.inner.y_offset = v;
    }
    #[getter]
    fn width(&self) -> u32 {
        self.inner.width
    }
    #[setter]
    fn set_width(&mut self, v: u32) {
        self.inner.width = v;
    }
    #[getter]
    fn height(&self) -> u32 {
        self.inner.height
    }
    #[setter]
    fn set_height(&mut self, v: u32) {
        self.inner.height = v;
    }
    #[getter]
    fn do_rectify(&self) -> bool {
        self.inner.do_rectify
    }
    #[setter]
    fn set_do_rectify(&mut self, v: bool) {
        self.inner.do_rectify = v;
    }
    fn contains(&self, x: u32, y: u32) -> bool {
        self.inner.contains(x, y)
    }
    fn area(&self) -> u32 {
        self.inner.area()
    }
    #[classattr]
    fn __topic_name__() -> &'static str {
        "region_of_interest"
    }
    fn __repr__(&self) -> String {
        format!(
            "RegionOfInterest(x={}, y={}, {}x{})",
            self.inner.x_offset, self.inner.y_offset, self.inner.width, self.inner.height
        )
    }
}

// ============================================================================
// StereoInfo — Stereo camera pair
// ============================================================================

#[pyclass(name = "StereoInfo")]
#[derive(Clone)]
pub struct PyStereoInfo {
    pub(crate) inner: StereoInfo,
}

#[pymethods]
impl PyStereoInfo {
    #[new]
    #[pyo3(signature = (left_camera=PyCameraInfo::new(0, 0, 0.0, 0.0, 0.0, 0.0), right_camera=PyCameraInfo::new(0, 0, 0.0, 0.0, 0.0, 0.0), baseline=0.0, depth_scale=1.0))]
    fn new(
        left_camera: PyCameraInfo,
        right_camera: PyCameraInfo,
        baseline: f64,
        depth_scale: f64,
    ) -> Self {
        Self {
            inner: StereoInfo {
                left_camera: left_camera.inner,
                right_camera: right_camera.inner,
                baseline,
                depth_scale,
            },
        }
    }
    #[getter]
    fn left_camera(&self) -> PyCameraInfo {
        PyCameraInfo {
            inner: self.inner.left_camera,
        }
    }
    #[setter]
    fn set_left_camera(&mut self, v: PyCameraInfo) {
        self.inner.left_camera = v.inner;
    }
    #[getter]
    fn right_camera(&self) -> PyCameraInfo {
        PyCameraInfo {
            inner: self.inner.right_camera,
        }
    }
    #[setter]
    fn set_right_camera(&mut self, v: PyCameraInfo) {
        self.inner.right_camera = v.inner;
    }
    #[getter]
    fn baseline(&self) -> f64 {
        self.inner.baseline
    }
    #[setter]
    fn set_baseline(&mut self, v: f64) {
        self.inner.baseline = v;
    }
    #[getter]
    fn depth_scale(&self) -> f64 {
        self.inner.depth_scale
    }
    #[setter]
    fn set_depth_scale(&mut self, v: f64) {
        self.inner.depth_scale = v;
    }
    fn depth_from_disparity(&self, disparity: f32) -> f32 {
        self.inner.depth_from_disparity(disparity)
    }
    fn disparity_from_depth(&self, depth: f32) -> f32 {
        self.inner.disparity_from_depth(depth)
    }
    #[classattr]
    fn __topic_name__() -> &'static str {
        "stereo_info"
    }
    fn __repr__(&self) -> String {
        format!(
            "StereoInfo({}x{}, baseline={:.3}m)",
            self.inner.left_camera.width, self.inner.left_camera.height, self.inner.baseline
        )
    }
}

// ============================================================================
// ImpedanceParameters — Impedance control (Pod)
// ============================================================================

#[pyclass(name = "ImpedanceParameters")]
#[derive(Clone)]
pub struct PyImpedanceParameters {
    pub(crate) inner: ImpedanceParameters,
}

#[pymethods]
impl PyImpedanceParameters {
    #[new]
    fn new() -> Self {
        Self {
            inner: ImpedanceParameters::new(),
        }
    }
    #[getter]
    fn stiffness(&self) -> Vec<f64> {
        self.inner.stiffness.to_vec()
    }
    #[setter]
    fn set_stiffness(&mut self, v: Vec<f64>) {
        if v.len() == 6 {
            self.inner.stiffness.copy_from_slice(&v);
        }
    }
    #[getter]
    fn damping(&self) -> Vec<f64> {
        self.inner.damping.to_vec()
    }
    #[setter]
    fn set_damping(&mut self, v: Vec<f64>) {
        if v.len() == 6 {
            self.inner.damping.copy_from_slice(&v);
        }
    }
    #[getter]
    fn inertia(&self) -> Vec<f64> {
        self.inner.inertia.to_vec()
    }
    #[setter]
    fn set_inertia(&mut self, v: Vec<f64>) {
        if v.len() == 6 {
            self.inner.inertia.copy_from_slice(&v);
        }
    }
    #[getter]
    fn force_limits(&self) -> Vec<f64> {
        self.inner.force_limits.to_vec()
    }
    #[setter]
    fn set_force_limits(&mut self, v: Vec<f64>) {
        if v.len() == 6 {
            self.inner.force_limits.copy_from_slice(&v);
        }
    }
    #[getter]
    fn enabled(&self) -> bool {
        self.inner.enabled != 0
    }
    #[setter]
    fn set_enabled(&mut self, v: bool) {
        self.inner.enabled = v as u8;
    }
    #[getter]
    fn timestamp_ns(&self) -> u64 {
        self.inner.timestamp_ns
    }
    #[setter]
    fn set_timestamp_ns(&mut self, v: u64) {
        self.inner.timestamp_ns = v;
    }
    #[staticmethod]
    fn compliant() -> Self {
        Self {
            inner: ImpedanceParameters::compliant(),
        }
    }
    #[staticmethod]
    fn stiff() -> Self {
        Self {
            inner: ImpedanceParameters::stiff(),
        }
    }

    /// Enable impedance control.
    fn enable(&mut self) {
        self.inner.enable();
    }

    /// Disable impedance control.
    fn disable(&mut self) {
        self.inner.disable();
    }

    #[classattr]
    fn __topic_name__() -> &'static str {
        "impedance_parameters"
    }
    fn __repr__(&self) -> String {
        format!("ImpedanceParameters(enabled={})", self.inner.enabled != 0)
    }
}

// ============================================================================
// HapticFeedback — Haptic feedback command (Pod)
// ============================================================================

#[pyclass(name = "HapticFeedback")]
#[derive(Clone)]
pub struct PyHapticFeedback {
    pub(crate) inner: HapticFeedback,
}

#[pymethods]
impl PyHapticFeedback {
    #[new]
    #[pyo3(signature = (vibration_intensity=0.0, vibration_frequency=0.0, duration_seconds=0.0, pattern_type=0))]
    fn new(
        vibration_intensity: f32,
        vibration_frequency: f32,
        duration_seconds: f32,
        pattern_type: u8,
    ) -> Self {
        Self {
            inner: HapticFeedback {
                vibration_intensity,
                vibration_frequency,
                duration_seconds,
                force_feedback: Vector3::zero(),
                pattern_type,
                enabled: 1,
                timestamp_ns: 0,
            },
        }
    }
    #[getter]
    fn vibration_intensity(&self) -> f32 {
        self.inner.vibration_intensity
    }
    #[setter]
    fn set_vibration_intensity(&mut self, v: f32) {
        self.inner.vibration_intensity = v;
    }
    #[getter]
    fn vibration_frequency(&self) -> f32 {
        self.inner.vibration_frequency
    }
    #[setter]
    fn set_vibration_frequency(&mut self, v: f32) {
        self.inner.vibration_frequency = v;
    }
    #[getter]
    fn duration_seconds(&self) -> f32 {
        self.inner.duration_seconds
    }
    #[setter]
    fn set_duration_seconds(&mut self, v: f32) {
        self.inner.duration_seconds = v;
    }
    #[getter]
    fn pattern_type(&self) -> u8 {
        self.inner.pattern_type
    }
    #[setter]
    fn set_pattern_type(&mut self, v: u8) {
        self.inner.pattern_type = v;
    }
    #[getter]
    fn enabled(&self) -> bool {
        self.inner.enabled != 0
    }
    #[setter]
    fn set_enabled(&mut self, v: bool) {
        self.inner.enabled = v as u8;
    }
    #[getter]
    fn timestamp_ns(&self) -> u64 {
        self.inner.timestamp_ns
    }
    #[setter]
    fn set_timestamp_ns(&mut self, v: u64) {
        self.inner.timestamp_ns = v;
    }
    /// Create a vibration feedback pattern.
    #[staticmethod]
    fn vibration(intensity: f32, frequency: f32, duration: f32) -> Self {
        Self {
            inner: HapticFeedback::vibration(intensity, frequency, duration),
        }
    }

    /// Create a force feedback pattern.
    #[staticmethod]
    fn force(force: &PyVector3, duration: f32) -> Self {
        Self {
            inner: HapticFeedback::force(force.inner, duration),
        }
    }

    /// Create a pulse feedback pattern.
    #[staticmethod]
    fn pulse(intensity: f32, frequency: f32, duration: f32) -> Self {
        Self {
            inner: HapticFeedback::pulse(intensity, frequency, duration),
        }
    }

    #[classattr]
    fn __topic_name__() -> &'static str {
        "haptic_feedback"
    }
    fn __repr__(&self) -> String {
        format!(
            "HapticFeedback(intensity={:.2}, freq={:.1}Hz, dur={:.2}s)",
            self.inner.vibration_intensity,
            self.inner.vibration_frequency,
            self.inner.duration_seconds
        )
    }
}

// ============================================================================
// DiagnosticValue — Diagnostic key-value pair (Pod)
// ============================================================================

#[pyclass(name = "DiagnosticValue")]
#[derive(Clone)]
pub struct PyDiagnosticValue {
    pub(crate) inner: DiagnosticValue,
}

#[pymethods]
impl PyDiagnosticValue {
    #[new]
    #[pyo3(signature = (key="", value=""))]
    fn new(key: &str, value: &str) -> Self {
        Self {
            inner: DiagnosticValue::string(key, value),
        }
    }
    #[getter]
    fn key(&self) -> String {
        let end = self.inner.key.iter().position(|&b| b == 0).unwrap_or(32);
        String::from_utf8_lossy(&self.inner.key[..end]).into_owned()
    }
    #[getter]
    fn value(&self) -> String {
        let end = self.inner.value.iter().position(|&b| b == 0).unwrap_or(64);
        String::from_utf8_lossy(&self.inner.value[..end]).into_owned()
    }
    #[getter]
    fn value_type(&self) -> u8 {
        self.inner.value_type
    }
    #[staticmethod]
    fn int(key: &str, value: i64) -> Self {
        Self {
            inner: DiagnosticValue::int(key, value),
        }
    }
    #[staticmethod]
    fn float(key: &str, value: f64) -> Self {
        Self {
            inner: DiagnosticValue::float(key, value),
        }
    }
    #[staticmethod]
    fn boolean(key: &str, value: bool) -> Self {
        Self {
            inner: DiagnosticValue::bool(key, value),
        }
    }
    #[classattr]
    fn __topic_name__() -> &'static str {
        "diagnostic_value"
    }
    fn __repr__(&self) -> String {
        let key = self.key();
        let val = self.value();
        format!("DiagnosticValue({}: \"{}\")", key, val)
    }
}

// ============================================================================
// DiagnosticReport — Diagnostic report (Pod)
// ============================================================================

#[pyclass(name = "DiagnosticReport")]
#[derive(Clone)]
pub struct PyDiagnosticReport {
    pub(crate) inner: DiagnosticReport,
}

#[pymethods]
impl PyDiagnosticReport {
    #[new]
    #[pyo3(signature = (component="", level=0))]
    fn new(component: &str, level: u8) -> Self {
        let mut inner = DiagnosticReport::default();
        let bytes = component.as_bytes();
        let len = bytes.len().min(31);
        inner.component[..len].copy_from_slice(&bytes[..len]);
        inner.level = level;
        Self { inner }
    }
    #[getter]
    fn component(&self) -> String {
        let end = self
            .inner
            .component
            .iter()
            .position(|&b| b == 0)
            .unwrap_or(32);
        String::from_utf8_lossy(&self.inner.component[..end]).into_owned()
    }
    #[getter]
    fn value_count(&self) -> u8 {
        self.inner.value_count
    }
    #[getter]
    fn level(&self) -> u8 {
        self.inner.level
    }
    #[setter]
    fn set_level(&mut self, v: u8) {
        self.inner.level = v;
    }
    #[getter]
    fn timestamp_ns(&self) -> u64 {
        self.inner.timestamp_ns
    }
    #[setter]
    fn set_timestamp_ns(&mut self, v: u64) {
        self.inner.timestamp_ns = v;
    }
    fn add_value(&mut self, val: PyDiagnosticValue) -> PyResult<()> {
        let idx = self.inner.value_count as usize;
        if idx < 16 {
            self.inner.values[idx] = val.inner;
            self.inner.value_count += 1;
            Ok(())
        } else {
            Err(pyo3::exceptions::PyOverflowError::new_err(
                "max 16 diagnostic values",
            ))
        }
    }
    fn get_values(&self) -> Vec<PyDiagnosticValue> {
        (0..self.inner.value_count as usize)
            .map(|i| PyDiagnosticValue {
                inner: self.inner.values[i],
            })
            .collect()
    }
    /// Add a string key-value pair to the report.
    fn add_string(&mut self, key: &str, value: &str) -> pyo3::PyResult<()> {
        self.inner
            .add_string(key, value)
            .map_err(pyo3::exceptions::PyValueError::new_err)
    }

    /// Add an integer key-value pair to the report.
    fn add_int(&mut self, key: &str, value: i64) -> pyo3::PyResult<()> {
        self.inner
            .add_int(key, value)
            .map_err(pyo3::exceptions::PyValueError::new_err)
    }

    /// Add a float key-value pair to the report.
    fn add_float(&mut self, key: &str, value: f64) -> pyo3::PyResult<()> {
        self.inner
            .add_float(key, value)
            .map_err(pyo3::exceptions::PyValueError::new_err)
    }

    /// Add a boolean key-value pair to the report.
    fn add_bool(&mut self, key: &str, value: bool) -> pyo3::PyResult<()> {
        self.inner
            .add_bool(key, value)
            .map_err(pyo3::exceptions::PyValueError::new_err)
    }

    #[classattr]
    fn __topic_name__() -> &'static str {
        "diagnostic_report"
    }
    fn __repr__(&self) -> String {
        format!(
            "DiagnosticReport({}, {} values, level={})",
            self.component(),
            self.inner.value_count,
            self.inner.level
        )
    }
}

// ============================================================================
// NodeHeartbeat — Node health heartbeat (Pod)
// ============================================================================

#[pyclass(name = "NodeHeartbeat")]
#[derive(Clone)]
pub struct PyNodeHeartbeat {
    pub(crate) inner: NodeHeartbeat,
}

#[pymethods]
impl PyNodeHeartbeat {
    #[new]
    #[pyo3(signature = (state=0, health=0))]
    fn new(state: u8, health: u8) -> Self {
        Self {
            inner: NodeHeartbeat {
                state,
                health,
                ..NodeHeartbeat::default()
            },
        }
    }
    #[getter]
    fn state(&self) -> u8 {
        self.inner.state
    }
    #[setter]
    fn set_state(&mut self, v: u8) {
        self.inner.state = v;
    }
    #[getter]
    fn health(&self) -> u8 {
        self.inner.health
    }
    #[setter]
    fn set_health(&mut self, v: u8) {
        self.inner.health = v;
    }
    #[getter]
    fn tick_count(&self) -> u64 {
        self.inner.tick_count
    }
    #[setter]
    fn set_tick_count(&mut self, v: u64) {
        self.inner.tick_count = v;
    }
    #[getter]
    fn target_rate_hz(&self) -> u32 {
        self.inner.target_rate_hz
    }
    #[setter]
    fn set_target_rate_hz(&mut self, v: u32) {
        self.inner.target_rate_hz = v;
    }
    #[getter]
    fn actual_rate_hz(&self) -> u32 {
        self.inner.actual_rate_hz
    }
    #[setter]
    fn set_actual_rate_hz(&mut self, v: u32) {
        self.inner.actual_rate_hz = v;
    }
    #[getter]
    fn error_count(&self) -> u32 {
        self.inner.error_count
    }
    #[setter]
    fn set_error_count(&mut self, v: u32) {
        self.inner.error_count = v;
    }
    #[getter]
    fn last_tick_timestamp(&self) -> u64 {
        self.inner.last_tick_timestamp
    }
    #[getter]
    fn heartbeat_timestamp(&self) -> u64 {
        self.inner.heartbeat_timestamp
    }
    fn is_fresh(&self, max_age_secs: u64) -> bool {
        self.inner.is_fresh(max_age_secs)
    }
    /// Update the timestamp to current time.
    fn update_timestamp(&mut self) {
        self.inner.update_timestamp();
    }

    #[classattr]
    fn __topic_name__() -> &'static str {
        "node_heartbeat"
    }
    fn __repr__(&self) -> String {
        format!(
            "NodeHeartbeat(state={}, health={}, ticks={})",
            self.inner.state, self.inner.health, self.inner.tick_count
        )
    }
}

// ============================================================================
// SafetyStatus — Safety system status (Pod)
// ============================================================================

#[pyclass(name = "SafetyStatus")]
#[derive(Clone)]
pub struct PySafetyStatus {
    pub(crate) inner: SafetyStatus,
}

#[pymethods]
impl PySafetyStatus {
    #[new]
    fn new() -> Self {
        Self {
            inner: SafetyStatus::new(),
        }
    }
    #[getter]
    fn enabled(&self) -> bool {
        self.inner.enabled != 0
    }
    #[setter]
    fn set_enabled(&mut self, v: bool) {
        self.inner.enabled = v as u8;
    }
    #[getter]
    fn estop_engaged(&self) -> bool {
        self.inner.estop_engaged != 0
    }
    #[setter]
    fn set_estop_engaged(&mut self, v: bool) {
        self.inner.estop_engaged = v as u8;
    }
    #[getter]
    fn watchdog_ok(&self) -> bool {
        self.inner.watchdog_ok != 0
    }
    #[setter]
    fn set_watchdog_ok(&mut self, v: bool) {
        self.inner.watchdog_ok = v as u8;
    }
    #[getter]
    fn limits_ok(&self) -> bool {
        self.inner.limits_ok != 0
    }
    #[setter]
    fn set_limits_ok(&mut self, v: bool) {
        self.inner.limits_ok = v as u8;
    }
    #[getter]
    fn comms_ok(&self) -> bool {
        self.inner.comms_ok != 0
    }
    #[setter]
    fn set_comms_ok(&mut self, v: bool) {
        self.inner.comms_ok = v as u8;
    }
    #[getter]
    fn mode(&self) -> u8 {
        self.inner.mode
    }
    #[setter]
    fn set_mode(&mut self, v: u8) {
        self.inner.mode = v;
    }
    #[getter]
    fn fault_code(&self) -> u32 {
        self.inner.fault_code
    }
    #[setter]
    fn set_fault_code(&mut self, v: u32) {
        self.inner.fault_code = v;
    }
    #[getter]
    fn timestamp_ns(&self) -> u64 {
        self.inner.timestamp_ns
    }
    #[setter]
    fn set_timestamp_ns(&mut self, v: u64) {
        self.inner.timestamp_ns = v;
    }
    /// Check if all safety conditions are met.
    fn is_safe(&self) -> bool {
        self.inner.is_safe()
    }

    /// Set a fault code.
    fn set_fault(&mut self, code: u32) {
        self.inner.set_fault(code);
    }

    /// Clear all fault codes.
    fn clear_faults(&mut self) {
        self.inner.clear_faults();
    }

    #[classattr]
    fn __topic_name__() -> &'static str {
        "safety_status"
    }
    fn __repr__(&self) -> String {
        format!(
            "SafetyStatus(enabled={}, estop={}, mode={})",
            self.inner.enabled != 0,
            self.inner.estop_engaged != 0,
            self.inner.mode
        )
    }
}

// ============================================================================
// Waypoint — Navigation waypoint (Pod)
// ============================================================================

#[pyclass(name = "Waypoint")]
#[derive(Clone)]
pub struct PyWaypoint {
    pub(crate) inner: Waypoint,
}

#[pymethods]
impl PyWaypoint {
    #[new]
    #[pyo3(signature = (x=0.0, y=0.0, theta=0.0))]
    fn new(x: f64, y: f64, theta: f64) -> Self {
        Self {
            inner: Waypoint::new(Pose2D {
                x,
                y,
                theta,
                timestamp_ns: 0,
            }),
        }
    }
    #[getter]
    fn pose(&self) -> PyPose2D {
        PyPose2D {
            inner: self.inner.pose,
        }
    }
    #[setter]
    fn set_pose(&mut self, v: PyPose2D) {
        self.inner.pose = v.inner;
    }
    #[getter]
    fn velocity(&self) -> PyTwist {
        PyTwist {
            inner: self.inner.velocity,
        }
    }
    #[setter]
    fn set_velocity(&mut self, v: PyTwist) {
        self.inner.velocity = v.inner;
    }
    #[getter]
    fn time_from_start(&self) -> f64 {
        self.inner.time_from_start
    }
    #[setter]
    fn set_time_from_start(&mut self, v: f64) {
        self.inner.time_from_start = v;
    }
    #[getter]
    fn curvature(&self) -> f32 {
        self.inner.curvature
    }
    #[setter]
    fn set_curvature(&mut self, v: f32) {
        self.inner.curvature = v;
    }
    #[getter]
    fn stop_required(&self) -> bool {
        self.inner.stop_required != 0
    }
    #[setter]
    fn set_stop_required(&mut self, v: bool) {
        self.inner.stop_required = v as u8;
    }
    /// Set a velocity constraint at this waypoint. Returns a new Waypoint.
    fn with_velocity(&self, velocity: &PyTwist) -> Self {
        Self {
            inner: self.inner.with_velocity(velocity.inner),
        }
    }

    /// Mark this waypoint as requiring a full stop. Returns a new Waypoint.
    fn with_stop(&self) -> Self {
        Self {
            inner: self.inner.with_stop(),
        }
    }

    #[classattr]
    fn __topic_name__() -> &'static str {
        "waypoint"
    }
    fn __repr__(&self) -> String {
        format!(
            "Waypoint({:.2}, {:.2}, {:.2})",
            self.inner.pose.x, self.inner.pose.y, self.inner.pose.theta
        )
    }
}

// ============================================================================
// NavPath — Navigation path (Pod)
// ============================================================================

#[pyclass(name = "NavPath")]
#[derive(Clone)]
pub struct PyNavPath {
    pub(crate) inner: NavPath,
}

#[pymethods]
impl PyNavPath {
    #[new]
    fn new() -> Self {
        Self {
            inner: NavPath::default(),
        }
    }
    #[getter]
    fn waypoint_count(&self) -> u16 {
        self.inner.waypoint_count
    }
    #[getter]
    fn total_length(&self) -> f64 {
        self.inner.total_length
    }
    #[setter]
    fn set_total_length(&mut self, v: f64) {
        self.inner.total_length = v;
    }
    #[getter]
    fn duration_seconds(&self) -> f64 {
        self.inner.duration_seconds
    }
    #[setter]
    fn set_duration_seconds(&mut self, v: f64) {
        self.inner.duration_seconds = v;
    }
    #[getter]
    fn timestamp_ns(&self) -> u64 {
        self.inner.timestamp_ns
    }
    #[setter]
    fn set_timestamp_ns(&mut self, v: u64) {
        self.inner.timestamp_ns = v;
    }
    fn add_waypoint(&mut self, wp: PyWaypoint) -> PyResult<()> {
        let idx = self.inner.waypoint_count as usize;
        if idx < 256 {
            self.inner.waypoints[idx] = wp.inner;
            self.inner.waypoint_count += 1;
            Ok(())
        } else {
            Err(pyo3::exceptions::PyOverflowError::new_err(
                "max 256 waypoints",
            ))
        }
    }
    fn get_waypoints(&self) -> Vec<PyWaypoint> {
        (0..self.inner.waypoint_count as usize)
            .map(|i| PyWaypoint {
                inner: self.inner.waypoints[i],
            })
            .collect()
    }
    /// Find the index of the closest waypoint to the current pose.
    fn closest_waypoint_index(&self, current: &PyPose2D) -> Option<usize> {
        self.inner.closest_waypoint_index(&current.inner)
    }

    /// Calculate progress along the path as a fraction [0.0, 1.0].
    fn calculate_progress(&self, current: &PyPose2D) -> f32 {
        self.inner.calculate_progress(&current.inner)
    }

    #[classattr]
    fn __topic_name__() -> &'static str {
        "nav_path"
    }
    fn __repr__(&self) -> String {
        format!(
            "NavPath({} waypoints, {:.1}m)",
            self.inner.waypoint_count, self.inner.total_length
        )
    }
}

// ============================================================================
// VelocityObstacle — Dynamic obstacle (Pod)
// ============================================================================

#[pyclass(name = "VelocityObstacle")]
#[derive(Clone)]
pub struct PyVelocityObstacle {
    pub(crate) inner: VelocityObstacle,
}

#[pymethods]
impl PyVelocityObstacle {
    #[new]
    #[pyo3(signature = (px=0.0, py_=0.0, vx=0.0, vy=0.0, radius=0.0, time_horizon=1.0, obstacle_id=0))]
    fn new(
        px: f64,
        py_: f64,
        vx: f64,
        vy: f64,
        radius: f32,
        time_horizon: f32,
        obstacle_id: u32,
    ) -> Self {
        Self {
            inner: VelocityObstacle {
                position: [px, py_],
                velocity: [vx, vy],
                radius,
                time_horizon,
                obstacle_id,
            },
        }
    }
    #[getter]
    fn position(&self) -> (f64, f64) {
        (self.inner.position[0], self.inner.position[1])
    }
    #[getter]
    fn velocity(&self) -> (f64, f64) {
        (self.inner.velocity[0], self.inner.velocity[1])
    }
    #[getter]
    fn radius(&self) -> f32 {
        self.inner.radius
    }
    #[setter]
    fn set_radius(&mut self, v: f32) {
        self.inner.radius = v;
    }
    #[getter]
    fn time_horizon(&self) -> f32 {
        self.inner.time_horizon
    }
    #[setter]
    fn set_time_horizon(&mut self, v: f32) {
        self.inner.time_horizon = v;
    }
    #[getter]
    fn obstacle_id(&self) -> u32 {
        self.inner.obstacle_id
    }
    #[setter]
    fn set_obstacle_id(&mut self, v: u32) {
        self.inner.obstacle_id = v;
    }
    #[classattr]
    fn __topic_name__() -> &'static str {
        "velocity_obstacle"
    }
    fn __repr__(&self) -> String {
        format!(
            "VelocityObstacle(pos=({:.1}, {:.1}), r={:.2}, id={})",
            self.inner.position[0],
            self.inner.position[1],
            self.inner.radius,
            self.inner.obstacle_id
        )
    }
}

// ============================================================================
// VelocityObstacles — Array of velocity obstacles (Pod)
// ============================================================================

#[pyclass(name = "VelocityObstacles")]
#[derive(Clone)]
pub struct PyVelocityObstacles {
    pub(crate) inner: VelocityObstacles,
}

#[pymethods]
impl PyVelocityObstacles {
    #[new]
    fn new() -> Self {
        Self {
            inner: VelocityObstacles::default(),
        }
    }
    #[getter]
    fn count(&self) -> u8 {
        self.inner.count
    }
    #[getter]
    fn timestamp_ns(&self) -> u64 {
        self.inner.timestamp_ns
    }
    #[setter]
    fn set_timestamp_ns(&mut self, v: u64) {
        self.inner.timestamp_ns = v;
    }
    fn add_obstacle(&mut self, obs: PyVelocityObstacle) -> PyResult<()> {
        let idx = self.inner.count as usize;
        if idx < 32 {
            self.inner.obstacles[idx] = obs.inner;
            self.inner.count += 1;
            Ok(())
        } else {
            Err(pyo3::exceptions::PyOverflowError::new_err(
                "max 32 velocity obstacles",
            ))
        }
    }
    fn get_obstacles(&self) -> Vec<PyVelocityObstacle> {
        (0..self.inner.count as usize)
            .map(|i| PyVelocityObstacle {
                inner: self.inner.obstacles[i],
            })
            .collect()
    }
    #[classattr]
    fn __topic_name__() -> &'static str {
        "velocity_obstacles"
    }
    fn __repr__(&self) -> String {
        format!("VelocityObstacles({} obstacles)", self.inner.count)
    }
}

// ============================================================================
// OccupancyGrid — Navigation occupancy grid (serde-based)
// ============================================================================

#[pyclass(name = "OccupancyGrid")]
#[derive(Clone)]
pub struct PyOccupancyGrid {
    pub(crate) inner: OccupancyGrid,
}

#[pymethods]
impl PyOccupancyGrid {
    #[new]
    #[pyo3(signature = (width=0, height=0, resolution=0.05))]
    fn new(width: u32, height: u32, resolution: f32) -> Self {
        Self {
            inner: OccupancyGrid::new(width, height, resolution, Pose2D::origin()),
        }
    }
    #[getter]
    fn width(&self) -> u32 {
        self.inner.width
    }
    #[getter]
    fn height(&self) -> u32 {
        self.inner.height
    }
    #[getter]
    fn resolution(&self) -> f32 {
        self.inner.resolution
    }
    #[setter]
    fn set_resolution(&mut self, v: f32) {
        self.inner.resolution = v;
    }
    #[getter]
    fn origin(&self) -> PyPose2D {
        PyPose2D {
            inner: self.inner.origin,
        }
    }
    #[setter]
    fn set_origin(&mut self, v: PyPose2D) {
        self.inner.origin = v.inner;
    }
    #[getter]
    fn data(&self) -> Vec<i8> {
        self.inner.data.clone()
    }
    #[setter]
    fn set_data(&mut self, v: Vec<i8>) {
        self.inner.data = v;
    }
    #[getter]
    fn timestamp_ns(&self) -> u64 {
        self.inner.timestamp_ns
    }
    #[setter]
    fn set_timestamp_ns(&mut self, v: u64) {
        self.inner.timestamp_ns = v;
    }
    fn occupancy(&self, grid_x: u32, grid_y: u32) -> Option<i8> {
        self.inner.occupancy(grid_x, grid_y)
    }
    fn set_occupancy(&mut self, grid_x: u32, grid_y: u32, value: i8) -> bool {
        self.inner.set_occupancy(grid_x, grid_y, value)
    }
    fn is_free(&self, x: f64, y: f64) -> bool {
        self.inner.is_free(x, y)
    }
    fn is_occupied(&self, x: f64, y: f64) -> bool {
        self.inner.is_occupied(x, y)
    }
    fn world_to_grid(&self, x: f64, y: f64) -> Option<(u32, u32)> {
        self.inner.world_to_grid(x, y)
    }
    fn grid_to_world(&self, grid_x: u32, grid_y: u32) -> Option<(f64, f64)> {
        self.inner.grid_to_world(grid_x, grid_y)
    }
    #[classattr]
    fn __topic_name__() -> &'static str {
        "occupancy_grid"
    }
    fn __repr__(&self) -> String {
        format!(
            "OccupancyGrid({}x{}, res={:.3}m)",
            self.inner.width, self.inner.height, self.inner.resolution
        )
    }
}

// ============================================================================
// CostMap — Navigation cost map (serde-based)
// ============================================================================

#[pyclass(name = "CostMap")]
#[derive(Clone)]
pub struct PyCostMap {
    pub(crate) inner: CostMap,
}

#[pymethods]
impl PyCostMap {
    #[new]
    #[pyo3(signature = (grid=PyOccupancyGrid::new(0, 0, 0.05), inflation_radius=0.55))]
    fn new(grid: PyOccupancyGrid, inflation_radius: f32) -> Self {
        Self {
            inner: CostMap::from_occupancy_grid(grid.inner, inflation_radius),
        }
    }
    #[getter]
    fn occupancy_grid(&self) -> PyOccupancyGrid {
        PyOccupancyGrid {
            inner: self.inner.occupancy_grid.clone(),
        }
    }
    #[getter]
    fn costs(&self) -> Vec<u8> {
        self.inner.costs.clone()
    }
    #[getter]
    fn inflation_radius(&self) -> f32 {
        self.inner.inflation_radius
    }
    #[setter]
    fn set_inflation_radius(&mut self, v: f32) {
        self.inner.inflation_radius = v;
    }
    #[getter]
    fn cost_scaling_factor(&self) -> f32 {
        self.inner.cost_scaling_factor
    }
    #[setter]
    fn set_cost_scaling_factor(&mut self, v: f32) {
        self.inner.cost_scaling_factor = v;
    }
    #[getter]
    fn lethal_cost(&self) -> u8 {
        self.inner.lethal_cost
    }
    #[setter]
    fn set_lethal_cost(&mut self, v: u8) {
        self.inner.lethal_cost = v;
    }
    fn cost(&self, x: f64, y: f64) -> Option<u8> {
        self.inner.cost(x, y)
    }
    fn compute_costs(&mut self) {
        self.inner.compute_costs();
    }
    #[classattr]
    fn __topic_name__() -> &'static str {
        "cost_map"
    }
    fn __repr__(&self) -> String {
        format!(
            "CostMap({}x{}, inflation={:.2}m)",
            self.inner.occupancy_grid.width,
            self.inner.occupancy_grid.height,
            self.inner.inflation_radius
        )
    }
}

// ============================================================================
// Module registration
// ============================================================================

pub fn register_message_classes(m: &Bound<'_, PyModule>) -> PyResult<()> {
    // Original types
    m.add_class::<PyCmdVel>()?;
    m.add_class::<PyPose2D>()?;
    m.add_class::<PyImu>()?;
    m.add_class::<PyOdometry>()?;
    m.add_class::<PyLaserScan>()?;
    m.add_class::<PyPose3D>()?;
    m.add_class::<PyJointState>()?;
    m.add_class::<PyClock>()?;
    m.add_class::<PyTimeReference>()?;
    // Geometry types
    m.add_class::<PyTwist>()?;
    m.add_class::<PyVector3>()?;
    m.add_class::<PyPoint3>()?;
    m.add_class::<PyQuaternion>()?;
    m.add_class::<PyTransformStamped>()?;
    m.add_class::<PyPoseStamped>()?;
    m.add_class::<PyPoseWithCovariance>()?;
    m.add_class::<PyTwistWithCovariance>()?;
    m.add_class::<PyAccel>()?;
    m.add_class::<PyAccelStamped>()?;
    // Control types
    m.add_class::<PyMotorCommand>()?;
    m.add_class::<PyServoCommand>()?;
    m.add_class::<PyDifferentialDriveCommand>()?;
    m.add_class::<PyPidConfig>()?;
    m.add_class::<PyTrajectoryPoint>()?;
    m.add_class::<PyJointCommand>()?;
    // Sensor types
    m.add_class::<PyRangeSensor>()?;
    m.add_class::<PyBatteryState>()?;
    m.add_class::<PyNavSatFix>()?;
    m.add_class::<PyMagneticField>()?;
    m.add_class::<PyTemperature>()?;
    m.add_class::<PyFluidPressure>()?;
    m.add_class::<PyIlluminance>()?;
    // Diagnostics types
    m.add_class::<PyHeartbeat>()?;
    m.add_class::<PyDiagnosticStatus>()?;
    m.add_class::<PyEmergencyStop>()?;
    m.add_class::<PyResourceUsage>()?;
    // Force types
    m.add_class::<PyWrenchStamped>()?;
    m.add_class::<PyForceCommand>()?;
    m.add_class::<PyContactInfo>()?;
    // Navigation types
    m.add_class::<PyNavGoal>()?;
    m.add_class::<PyGoalResult>()?;
    m.add_class::<PyPathPlan>()?;
    // Input types
    m.add_class::<PyJoystickInput>()?;
    m.add_class::<PyKeyboardInput>()?;
    // Detection/Perception types
    m.add_class::<PyBoundingBox2DMsg>()?;
    m.add_class::<PyBoundingBox3D>()?;
    m.add_class::<PyDetectionMsg>()?;
    m.add_class::<PyDetection3D>()?;
    m.add_class::<PySegmentationMask>()?;
    // Tracking types
    m.add_class::<PyTrackedObjectMsg>()?;
    m.add_class::<PyTrackingHeader>()?;
    // Landmark types
    m.add_class::<PyLandmarkMsg>()?;
    m.add_class::<PyLandmark3D>()?;
    m.add_class::<PyLandmarkArray>()?;
    // Perception helper types
    m.add_class::<PyPointField>()?;
    m.add_class::<PyPlaneDetection>()?;
    m.add_class::<PyPlaneArray>()?;
    // Vision types
    m.add_class::<PyCompressedImage>()?;
    m.add_class::<PyCameraInfo>()?;
    m.add_class::<PyRegionOfInterest>()?;
    m.add_class::<PyStereoInfo>()?;
    // Force types (additional)
    m.add_class::<PyImpedanceParameters>()?;
    m.add_class::<PyHapticFeedback>()?;
    // Diagnostics types (additional)
    m.add_class::<PyDiagnosticValue>()?;
    m.add_class::<PyDiagnosticReport>()?;
    m.add_class::<PyNodeHeartbeat>()?;
    m.add_class::<PySafetyStatus>()?;
    // Navigation types (additional)
    m.add_class::<PyWaypoint>()?;
    m.add_class::<PyNavPath>()?;
    m.add_class::<PyVelocityObstacle>()?;
    m.add_class::<PyVelocityObstacles>()?;
    m.add_class::<PyOccupancyGrid>()?;
    m.add_class::<PyCostMap>()?;
    // Audio
    m.add_class::<PyAudioFrame>()?;
    // Tactile
    m.add_class::<PyTactileArray>()?;
    Ok(())
}

// ============================================================================
// Audio Types
// ============================================================================

use horus_library::messages::AudioFrame;

/// AudioFrame — audio data from a microphone or audio source.
///
/// Args:
///     sample_rate: Sample rate in Hz (default: 16000)
///     channels: Number of channels, 1=mono, 2=stereo (default: 1)
///     samples: List of float samples (default: empty)
///     timestamp_ns: Timestamp in nanoseconds (default: 0)
///     frame_id: Sensor identifier string (default: "")
///
/// Examples:
///     # Mono microphone at 16kHz
///     frame = AudioFrame(sample_rate=16000, samples=[0.1, 0.2, 0.3])
///
///     # Stereo at 48kHz
///     frame = AudioFrame(sample_rate=48000, channels=2, samples=interleaved_data)
///
///     # Access data
///     samples = frame.samples    # list of valid samples
///     duration = frame.duration_ms
#[pyclass(name = "AudioFrame")]
#[derive(Clone)]
pub struct PyAudioFrame {
    pub(crate) inner: AudioFrame,
}

#[pymethods]
impl PyAudioFrame {
    #[new]
    #[pyo3(signature = (sample_rate=16000, channels=1, samples=None, timestamp_ns=0, frame_id=""))]
    fn new(
        sample_rate: u32,
        channels: u8,
        samples: Option<Vec<f32>>,
        timestamp_ns: u64,
        frame_id: &str,
    ) -> Self {
        let mut frame = if let Some(ref s) = samples {
            match channels {
                1 => AudioFrame::mono(sample_rate, s),
                2 => AudioFrame::stereo(sample_rate, s),
                n => AudioFrame::multi_channel(sample_rate, n, s),
            }
        } else {
            AudioFrame {
                sample_rate,
                channels,
                ..AudioFrame::default()
            }
        };
        frame.timestamp_ns = timestamp_ns;
        if !frame_id.is_empty() {
            frame = frame.with_frame_id(frame_id);
        }
        Self { inner: frame }
    }

    /// Valid samples as a Python list.
    #[getter]
    fn samples(&self) -> Vec<f32> {
        self.inner.valid_samples().to_vec()
    }

    /// Number of valid samples.
    #[getter]
    fn num_samples(&self) -> u32 {
        self.inner.num_samples
    }

    /// Sample rate in Hz.
    #[getter]
    fn sample_rate(&self) -> u32 {
        self.inner.sample_rate
    }

    /// Number of audio channels.
    #[getter]
    fn channels(&self) -> u8 {
        self.inner.channels
    }

    /// Duration of this frame in milliseconds.
    #[getter]
    fn duration_ms(&self) -> f64 {
        self.inner.duration_ms()
    }

    /// Number of audio frames (samples per channel).
    #[getter]
    fn frame_count(&self) -> u32 {
        self.inner.frame_count()
    }

    /// Timestamp in nanoseconds.
    #[getter]
    fn timestamp_ns(&self) -> u64 {
        self.inner.timestamp_ns
    }

    #[setter]
    fn set_timestamp_ns(&mut self, value: u64) {
        self.inner.timestamp_ns = value;
    }

    /// Frame/sensor identifier.
    #[getter]
    fn frame_id(&self) -> String {
        self.inner.frame_id_str().to_string()
    }

    /// Create a mono audio frame.
    #[staticmethod]
    fn mono(sample_rate: u32, samples: Vec<f32>) -> Self {
        Self {
            inner: AudioFrame::mono(sample_rate, &samples),
        }
    }

    /// Create a stereo audio frame (interleaved L/R samples).
    #[staticmethod]
    fn stereo(sample_rate: u32, samples: Vec<f32>) -> Self {
        Self {
            inner: AudioFrame::stereo(sample_rate, &samples),
        }
    }

    /// Create a multi-channel audio frame.
    #[staticmethod]
    fn multi_channel(sample_rate: u32, channels: u8, samples: Vec<f32>) -> Self {
        Self {
            inner: AudioFrame::multi_channel(sample_rate, channels, &samples),
        }
    }

    fn __repr__(&self) -> String {
        format!(
            "AudioFrame({}ch, {}Hz, {} samples, {:.1}ms)",
            self.inner.channels,
            self.inner.sample_rate,
            self.inner.num_samples,
            self.inner.duration_ms()
        )
    }
}

// ============================================================================
// Tactile Types
// ============================================================================

/// TactileArray — tactile sensor array reading for contact-rich manipulation.
///
/// Args:
///     rows: Number of taxel rows (default: 0)
///     cols: Number of taxel columns (default: 0)
///
/// Example::
///
///     tactile = TactileArray(rows=4, cols=4)
///     tactile.set_force(1, 2, 3.5)
///     print(tactile.get_force(1, 2))
#[pyclass(name = "TactileArray")]
#[derive(Clone)]
pub struct PyTactileArray {
    pub(crate) inner: TactileArray,
}

#[pymethods]
impl PyTactileArray {
    #[new]
    #[pyo3(signature = (rows=0, cols=0))]
    fn new(rows: u32, cols: u32) -> Self {
        Self {
            inner: TactileArray::new(rows, cols),
        }
    }
    #[getter]
    fn rows(&self) -> u32 {
        self.inner.rows
    }
    #[setter]
    fn set_rows(&mut self, v: u32) {
        self.inner.rows = v;
    }
    #[getter]
    fn cols(&self) -> u32 {
        self.inner.cols
    }
    #[setter]
    fn set_cols(&mut self, v: u32) {
        self.inner.cols = v;
    }
    #[getter]
    fn forces(&self) -> Vec<f32> {
        self.inner.forces.clone()
    }
    #[setter]
    fn set_forces(&mut self, v: Vec<f32>) {
        self.inner.forces = v;
    }
    #[getter]
    fn total_force(&self) -> [f32; 3] {
        self.inner.total_force
    }
    #[setter]
    fn set_total_force(&mut self, v: [f32; 3]) {
        self.inner.total_force = v;
    }
    #[getter]
    fn center_of_pressure(&self) -> [f32; 2] {
        self.inner.center_of_pressure
    }
    #[setter]
    fn set_center_of_pressure(&mut self, v: [f32; 2]) {
        self.inner.center_of_pressure = v;
    }
    #[getter]
    fn in_contact(&self) -> bool {
        self.inner.in_contact
    }
    #[setter]
    fn set_in_contact(&mut self, v: bool) {
        self.inner.in_contact = v;
    }
    #[getter]
    fn physical_size(&self) -> [f32; 2] {
        self.inner.physical_size
    }
    #[setter]
    fn set_physical_size(&mut self, v: [f32; 2]) {
        self.inner.physical_size = v;
    }
    #[getter]
    fn timestamp_ns(&self) -> u64 {
        self.inner.timestamp_ns
    }
    #[setter]
    fn set_timestamp_ns(&mut self, v: u64) {
        self.inner.timestamp_ns = v;
    }
    fn get_force(&self, row: u32, col: u32) -> Option<f32> {
        self.inner.get_force(row, col)
    }
    fn set_force(&mut self, row: u32, col: u32, force: f32) {
        self.inner.set_force(row, col, force);
    }
    #[classattr]
    fn __topic_name__() -> &'static str {
        "tactile_array"
    }
    fn __repr__(&self) -> String {
        use horus::core::LogSummary;
        self.inner.log_summary()
    }
}
