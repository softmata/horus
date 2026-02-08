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

/// Register all message classes with the Python module
pub fn register_message_classes(m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add_class::<PyCmdVel>()?;
    m.add_class::<PyPose2D>()?;
    m.add_class::<PyImu>()?;
    m.add_class::<PyOdometry>()?;
    m.add_class::<PyLaserScan>()?;
    Ok(())
}
