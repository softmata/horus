//! Python wrapper classes for HORUS message types — POD-optimized
//!
//! Each Python class stores the Rust POD struct directly (`inner` field),
//! enabling zero-extraction send and zero-construction recv in Topic.
//!
//! ```python
//! from horus import Topic, CmdVel, Pose2D
//!
//! topic = Topic(CmdVel)
//! topic.send(CmdVel(1.5, 0.3))
//! msg = topic.recv()  # Returns CmdVel backed by Rust POD
//! ```

use horus_library::messages::clock::Clock;
use horus_library::messages::cmd_vel::CmdVel;
use horus_library::messages::geometry::Pose2D;
use horus_library::messages::sensor::{Imu, JointState, LaserScan, Odometry};
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

/// 3D pose (position + orientation)
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
#[derive(Clone, Debug)]
pub struct PyPose3D {
    #[pyo3(get, set)]
    pub x: f64,
    #[pyo3(get, set)]
    pub y: f64,
    #[pyo3(get, set)]
    pub z: f64,
    #[pyo3(get, set)]
    pub qx: f64,
    #[pyo3(get, set)]
    pub qy: f64,
    #[pyo3(get, set)]
    pub qz: f64,
    #[pyo3(get, set)]
    pub qw: f64,
    #[pyo3(get, set)]
    pub timestamp_ns: u64,
}

#[pymethods]
impl PyPose3D {
    #[new]
    #[pyo3(signature = (x=0.0, y=0.0, z=0.0, qx=0.0, qy=0.0, qz=0.0, qw=1.0, timestamp_ns=0))]
    #[allow(clippy::too_many_arguments)]
    fn new(
        x: f64,
        y: f64,
        z: f64,
        qx: f64,
        qy: f64,
        qz: f64,
        qw: f64,
        timestamp_ns: u64,
    ) -> Self {
        Self {
            x,
            y,
            z,
            qx,
            qy,
            qz,
            qw,
            timestamp_ns,
        }
    }

    #[classattr]
    fn __topic_name__() -> &'static str {
        "pose3d"
    }

    fn __repr__(&self) -> String {
        format!(
            "Pose3D(pos=[{:.3}, {:.3}, {:.3}], quat=[{:.3}, {:.3}, {:.3}, {:.3}], timestamp_ns={})",
            self.x, self.y, self.z, self.qx, self.qy, self.qz, self.qw, self.timestamp_ns
        )
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
// Module registration
// ============================================================================

pub fn register_message_classes(m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add_class::<PyCmdVel>()?;
    m.add_class::<PyPose2D>()?;
    m.add_class::<PyImu>()?;
    m.add_class::<PyOdometry>()?;
    m.add_class::<PyLaserScan>()?;
    m.add_class::<PyPose3D>()?;
    m.add_class::<PyJointState>()?;
    m.add_class::<PyClock>()?;
    Ok(())
}
