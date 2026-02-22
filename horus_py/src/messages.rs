//! Python wrapper classes for HORUS message types
//!
//! These Python classes mirror the Rust message types from horus_library,
//! enabling typed Topic communication in Python:
//!
//! ```python
//! from horus import Topic, CmdVel, Pose2D
//!
//! topic = Topic(CmdVel)
//! topic.send(CmdVel(1.5, 0.3))
//! msg = topic.recv()  # Returns CmdVel instance
//! ```

use pyo3::prelude::*;

/// Velocity command message for differential drive robots
///
/// Attributes:
///     linear: Forward/backward velocity in m/s
///     angular: Rotational velocity in rad/s
///     timestamp_ns: Timestamp in nanoseconds (default: 0)
///
/// Examples:
///     cmd = CmdVel(1.5, 0.3)           # Forward at 1.5 m/s, turning at 0.3 rad/s
///     cmd = CmdVel(linear=0.5, angular=0.0)  # Straight forward
///     cmd = CmdVel(0.0, 1.0, timestamp_ns=12345)  # With explicit timestamp
#[pyclass(name = "CmdVel")]
#[derive(Clone, Debug)]
pub struct PyCmdVel {
    #[pyo3(get, set)]
    pub linear: f32,
    #[pyo3(get, set)]
    pub angular: f32,
    #[pyo3(get, set)]
    pub timestamp_ns: u64,
}

#[pymethods]
impl PyCmdVel {
    /// Create a new CmdVel message
    ///
    /// Args:
    ///     linear: Forward velocity in m/s
    ///     angular: Angular velocity in rad/s
    ///     timestamp_ns: Optional timestamp in nanoseconds
    #[new]
    #[pyo3(signature = (linear, angular, timestamp_ns=0))]
    fn new(linear: f32, angular: f32, timestamp_ns: u64) -> Self {
        Self {
            linear,
            angular,
            timestamp_ns,
        }
    }

    /// Topic name for this message type (used for auto-discovery)
    #[classattr]
    fn __topic_name__() -> &'static str {
        "cmd_vel"
    }

    fn __repr__(&self) -> String {
        format!(
            "CmdVel(linear={:.3}, angular={:.3}, timestamp_ns={})",
            self.linear, self.angular, self.timestamp_ns
        )
    }

    fn __eq__(&self, other: &Self) -> bool {
        (self.linear - other.linear).abs() < 1e-6 && (self.angular - other.angular).abs() < 1e-6
    }
}

/// 2D pose message (position and orientation)
///
/// Attributes:
///     x: X position in meters
///     y: Y position in meters
///     theta: Orientation in radians
///     timestamp_ns: Timestamp in nanoseconds (default: 0)
///
/// Examples:
///     pose = Pose2D(1.0, 2.0, 0.5)     # Position (1,2) facing 0.5 rad
///     pose = Pose2D(x=0, y=0, theta=3.14)  # At origin, facing backward
#[pyclass(name = "Pose2D")]
#[derive(Clone, Debug)]
pub struct PyPose2D {
    #[pyo3(get, set)]
    pub x: f64,
    #[pyo3(get, set)]
    pub y: f64,
    #[pyo3(get, set)]
    pub theta: f64,
    #[pyo3(get, set)]
    pub timestamp_ns: u64,
}

#[pymethods]
impl PyPose2D {
    /// Create a new Pose2D message
    ///
    /// Args:
    ///     x: X position in meters
    ///     y: Y position in meters
    ///     theta: Orientation in radians
    ///     timestamp_ns: Optional timestamp in nanoseconds
    #[new]
    #[pyo3(signature = (x, y, theta, timestamp_ns=0))]
    fn new(x: f64, y: f64, theta: f64, timestamp_ns: u64) -> Self {
        Self {
            x,
            y,
            theta,
            timestamp_ns,
        }
    }

    /// Topic name for this message type
    #[classattr]
    fn __topic_name__() -> &'static str {
        "pose"
    }

    fn __repr__(&self) -> String {
        format!(
            "Pose2D(x={:.3}, y={:.3}, theta={:.3}, timestamp_ns={})",
            self.x, self.y, self.theta, self.timestamp_ns
        )
    }

    fn __eq__(&self, other: &Self) -> bool {
        (self.x - other.x).abs() < 1e-9
            && (self.y - other.y).abs() < 1e-9
            && (self.theta - other.theta).abs() < 1e-9
    }
}

/// IMU (Inertial Measurement Unit) sensor message
///
/// Attributes:
///     accel_x, accel_y, accel_z: Linear acceleration in m/s²
///     gyro_x, gyro_y, gyro_z: Angular velocity in rad/s
///     timestamp_ns: Timestamp in nanoseconds (default: 0)
///
/// Examples:
///     imu = Imu(0.0, 0.0, 9.81, 0.0, 0.0, 0.0)  # At rest (gravity on Z)
///     imu = Imu(accel_x=1.0, accel_y=0.0, accel_z=9.81,
///               gyro_x=0.0, gyro_y=0.0, gyro_z=0.1)  # Accelerating + rotating
#[pyclass(name = "Imu")]
#[derive(Clone, Debug)]
pub struct PyImu {
    #[pyo3(get, set)]
    pub accel_x: f64,
    #[pyo3(get, set)]
    pub accel_y: f64,
    #[pyo3(get, set)]
    pub accel_z: f64,
    #[pyo3(get, set)]
    pub gyro_x: f64,
    #[pyo3(get, set)]
    pub gyro_y: f64,
    #[pyo3(get, set)]
    pub gyro_z: f64,
    #[pyo3(get, set)]
    pub timestamp_ns: u64,
}

#[pymethods]
impl PyImu {
    /// Create a new Imu message
    ///
    /// Args:
    ///     accel_x, accel_y, accel_z: Linear acceleration (m/s²)
    ///     gyro_x, gyro_y, gyro_z: Angular velocity (rad/s)
    ///     timestamp_ns: Optional timestamp in nanoseconds
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
        Self {
            accel_x,
            accel_y,
            accel_z,
            gyro_x,
            gyro_y,
            gyro_z,
            timestamp_ns,
        }
    }

    /// Topic name for this message type
    #[classattr]
    fn __topic_name__() -> &'static str {
        "imu"
    }

    fn __repr__(&self) -> String {
        format!(
            "Imu(accel=[{:.3}, {:.3}, {:.3}], gyro=[{:.3}, {:.3}, {:.3}], timestamp_ns={})",
            self.accel_x,
            self.accel_y,
            self.accel_z,
            self.gyro_x,
            self.gyro_y,
            self.gyro_z,
            self.timestamp_ns
        )
    }
}

/// Odometry message (pose + velocity)
///
/// Attributes:
///     x, y, theta: Position and orientation
///     linear_velocity: Forward velocity in m/s
///     angular_velocity: Rotational velocity in rad/s
///     timestamp_ns: Timestamp in nanoseconds (default: 0)
///
/// Examples:
///     odom = Odometry(x=1.0, y=2.0, theta=0.5)  # Position only
///     odom = Odometry(x=1.0, y=2.0, theta=0.5,
///                     linear_velocity=0.5, angular_velocity=0.1)  # With velocity
#[pyclass(name = "Odometry")]
#[derive(Clone, Debug)]
pub struct PyOdometry {
    #[pyo3(get, set)]
    pub x: f64,
    #[pyo3(get, set)]
    pub y: f64,
    #[pyo3(get, set)]
    pub theta: f64,
    #[pyo3(get, set)]
    pub linear_velocity: f64,
    #[pyo3(get, set)]
    pub angular_velocity: f64,
    #[pyo3(get, set)]
    pub timestamp_ns: u64,
}

#[pymethods]
impl PyOdometry {
    /// Create a new Odometry message
    ///
    /// Args:
    ///     x, y, theta: Position and orientation
    ///     linear_velocity: Forward velocity (m/s)
    ///     angular_velocity: Rotational velocity (rad/s)
    ///     timestamp_ns: Optional timestamp in nanoseconds
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
        Self {
            x,
            y,
            theta,
            linear_velocity,
            angular_velocity,
            timestamp_ns,
        }
    }

    /// Topic name for this message type
    #[classattr]
    fn __topic_name__() -> &'static str {
        "odom"
    }

    fn __repr__(&self) -> String {
        format!(
            "Odometry(x={:.3}, y={:.3}, theta={:.3}, v_lin={:.3}, v_ang={:.3}, timestamp_ns={})",
            self.x,
            self.y,
            self.theta,
            self.linear_velocity,
            self.angular_velocity,
            self.timestamp_ns
        )
    }
}

/// Laser scan message (range measurements from a LiDAR)
///
/// Attributes:
///     angle_min: Start angle in radians
///     angle_max: End angle in radians
///     angle_increment: Angular distance between measurements
///     range_min: Minimum valid range in meters
///     range_max: Maximum valid range in meters
///     ranges: List of range measurements
///     timestamp_ns: Timestamp in nanoseconds (default: 0)
///
/// Examples:
///     scan = LaserScan(
///         angle_min=-1.57,    # -90 degrees
///         angle_max=1.57,     # +90 degrees
///         angle_increment=0.01,
///         range_min=0.1,
///         range_max=10.0,
///         ranges=[1.0, 1.1, 1.2, ...]  # Distance measurements
///     )
#[pyclass(name = "LaserScan")]
#[derive(Clone, Debug)]
pub struct PyLaserScan {
    #[pyo3(get, set)]
    pub angle_min: f32,
    #[pyo3(get, set)]
    pub angle_max: f32,
    #[pyo3(get, set)]
    pub angle_increment: f32,
    #[pyo3(get, set)]
    pub range_min: f32,
    #[pyo3(get, set)]
    pub range_max: f32,
    #[pyo3(get, set)]
    pub ranges: Vec<f32>,
    #[pyo3(get, set)]
    pub timestamp_ns: u64,
}

#[pymethods]
impl PyLaserScan {
    /// Create a new LaserScan message
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
        Self {
            angle_min,
            angle_max,
            angle_increment,
            range_min,
            range_max,
            ranges: ranges.unwrap_or_default(),
            timestamp_ns,
        }
    }

    /// Topic name for this message type
    #[classattr]
    fn __topic_name__() -> &'static str {
        "scan"
    }

    /// Number of range measurements
    fn __len__(&self) -> usize {
        self.ranges.len()
    }

    fn __repr__(&self) -> String {
        format!(
            "LaserScan(angle=[{:.2}, {:.2}], range=[{:.2}, {:.2}], {} points, timestamp_ns={})",
            self.angle_min,
            self.angle_max,
            self.range_min,
            self.range_max,
            self.ranges.len(),
            self.timestamp_ns
        )
    }
}

/// 3D pose (position + orientation)
///
/// Attributes:
///     x, y, z: Position in meters
///     qx, qy, qz, qw: Orientation as quaternion (w-last convention)
///     timestamp_ns: Timestamp in nanoseconds (default: 0)
///
/// Examples:
///     pose = Pose3D(1.0, 2.0, 3.0)                    # Position only (identity rotation)
///     pose = Pose3D(1.0, 2.0, 3.0, 0, 0, 0.707, 0.707)  # With rotation
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
    /// Create a new Pose3D message
    ///
    /// Args:
    ///     x, y, z: Position in meters
    ///     qx, qy, qz, qw: Quaternion orientation (default: identity)
    ///     timestamp_ns: Optional timestamp in nanoseconds
    #[new]
    #[pyo3(signature = (x=0.0, y=0.0, z=0.0, qx=0.0, qy=0.0, qz=0.0, qw=1.0, timestamp_ns=0))]
    #[allow(clippy::too_many_arguments)]
    fn new(x: f64, y: f64, z: f64, qx: f64, qy: f64, qz: f64, qw: f64, timestamp_ns: u64) -> Self {
        Self { x, y, z, qx, qy, qz, qw, timestamp_ns }
    }

    /// Topic name for this message type
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
#[derive(Clone, Debug)]
pub struct PyJointState {
    #[pyo3(get, set)]
    pub names: Vec<String>,
    #[pyo3(get, set)]
    pub positions: Vec<f64>,
    #[pyo3(get, set)]
    pub velocities: Vec<f64>,
    #[pyo3(get, set)]
    pub efforts: Vec<f64>,
    #[pyo3(get, set)]
    pub timestamp_ns: u64,
}

#[pymethods]
impl PyJointState {
    /// Create a new JointState message
    ///
    /// Args:
    ///     names: Joint names (max 16)
    ///     positions: Joint positions (radians or meters)
    ///     velocities: Joint velocities (default: zeros)
    ///     efforts: Joint efforts/torques (default: zeros)
    ///     timestamp_ns: Optional timestamp in nanoseconds
    #[new]
    #[pyo3(signature = (names=None, positions=None, velocities=None, efforts=None, timestamp_ns=0))]
    fn new(
        names: Option<Vec<String>>,
        positions: Option<Vec<f64>>,
        velocities: Option<Vec<f64>>,
        efforts: Option<Vec<f64>>,
        timestamp_ns: u64,
    ) -> Self {
        let names = names.unwrap_or_default();
        let count = names.len();
        Self {
            names,
            positions: positions.unwrap_or_else(|| vec![0.0; count]),
            velocities: velocities.unwrap_or_else(|| vec![0.0; count]),
            efforts: efforts.unwrap_or_else(|| vec![0.0; count]),
            timestamp_ns,
        }
    }

    /// Topic name for this message type
    #[classattr]
    fn __topic_name__() -> &'static str {
        "joint_states"
    }

    /// Number of joints
    fn __len__(&self) -> usize {
        self.names.len()
    }

    fn __repr__(&self) -> String {
        format!(
            "JointState({} joints: [{}], timestamp_ns={})",
            self.names.len(),
            self.names.join(", "),
            self.timestamp_ns
        )
    }
}

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
///     clk = Clock(clock_ns=1000000000, sim_speed=2.0, source=1)  # 1s sim time at 2x
#[pyclass(name = "Clock")]
#[derive(Clone, Debug)]
pub struct PyClock {
    #[pyo3(get, set)]
    pub clock_ns: u64,
    #[pyo3(get, set)]
    pub realtime_ns: u64,
    #[pyo3(get, set)]
    pub sim_speed: f64,
    #[pyo3(get, set)]
    pub paused: bool,
    #[pyo3(get, set)]
    pub source: u8,
    #[pyo3(get, set)]
    pub timestamp_ns: u64,
}

#[pymethods]
impl PyClock {
    /// Create a new Clock message
    ///
    /// Args:
    ///     clock_ns: Simulation/replay time in nanoseconds
    ///     realtime_ns: Wall clock time in nanoseconds
    ///     sim_speed: Playback speed (1.0 = real-time)
    ///     paused: Whether the clock is paused
    ///     source: 0=wall, 1=sim, 2=replay
    ///     timestamp_ns: Optional timestamp in nanoseconds
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
        Self { clock_ns, realtime_ns, sim_speed, paused, source, timestamp_ns }
    }

    /// Topic name for this message type
    #[classattr]
    fn __topic_name__() -> &'static str {
        "clock"
    }

    fn __repr__(&self) -> String {
        let source_str = match self.source {
            0 => "wall",
            1 => "sim",
            2 => "replay",
            _ => "unknown",
        };
        format!(
            "Clock(clock_ns={}, speed={:.1}x, paused={}, source={}, timestamp_ns={})",
            self.clock_ns, self.sim_speed, self.paused, source_str, self.timestamp_ns
        )
    }
}

/// Register all message classes with the Python module
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
