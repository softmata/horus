// Python bindings - allow common clippy warnings
#![allow(clippy::all)]
#![allow(deprecated)]

use horus_library::messages::{cmd_vel, geometry, sensor};
use numpy::PyArray1;
use pyo3::prelude::*;
use pyo3::types::{PyDict, PyTuple};

mod control_messages;
mod coordination_messages;
mod diagnostics_messages;
mod force_messages;
mod input_messages;
mod io_messages;
mod navigation_messages;
mod perception_messages;
mod sensor_messages;
mod vision_messages;

use control_messages::*;
pub use coordination_messages::*;
use diagnostics_messages::*;
pub use force_messages::*;
use input_messages::*;
use io_messages::*;
pub use navigation_messages::*;
pub use perception_messages::*;
pub use vision_messages::*;

/// Python wrapper for Pose2D
#[pyclass(module = "horus.library._library", name = "Pose2D")]
#[derive(Clone)]
pub struct PyPose2D {
    inner: geometry::Pose2D,
}

#[pymethods]
impl PyPose2D {
    #[new]
    #[pyo3(signature = (x, y, theta, timestamp=None))]
    fn new(x: f64, y: f64, theta: f64, timestamp: Option<u64>) -> Self {
        let mut pose = geometry::Pose2D::new(x, y, theta);
        if let Some(ts) = timestamp {
            pose.timestamp = ts;
        }
        Self { inner: pose }
    }

    /// Create pose at origin
    #[staticmethod]
    fn origin() -> Self {
        Self {
            inner: geometry::Pose2D::origin(),
        }
    }

    #[getter]
    fn x(&self) -> f64 {
        self.inner.x
    }

    #[setter]
    fn set_x(&mut self, value: f64) {
        self.inner.x = value;
    }

    #[getter]
    fn y(&self) -> f64 {
        self.inner.y
    }

    #[setter]
    fn set_y(&mut self, value: f64) {
        self.inner.y = value;
    }

    #[getter]
    fn theta(&self) -> f64 {
        self.inner.theta
    }

    #[setter]
    fn set_theta(&mut self, value: f64) {
        self.inner.theta = value;
    }

    #[getter]
    fn timestamp(&self) -> u64 {
        self.inner.timestamp
    }

    /// Calculate euclidean distance to another pose
    fn distance_to(&self, other: &PyPose2D) -> f64 {
        self.inner.distance_to(&other.inner)
    }

    /// Normalize theta to [-pi, pi]
    fn normalize_angle(&mut self) {
        self.inner.normalize_angle();
    }

    /// Check if values are finite
    fn is_valid(&self) -> bool {
        self.inner.is_valid()
    }

    fn __repr__(&self) -> String {
        format!(
            "Pose2D(x={:.3}, y={:.3}, theta={:.3})",
            self.inner.x, self.inner.y, self.inner.theta
        )
    }

    fn __str__(&self) -> String {
        self.__repr__()
    }

    /// Pickle support: Provide constructor arguments
    fn __getnewargs__(&self) -> (f64, f64, f64) {
        (self.inner.x, self.inner.y, self.inner.theta)
    }
}

/// Python wrapper for Twist
#[pyclass(module = "horus.library._library", name = "Twist")]
#[derive(Clone)]
pub struct PyTwist {
    inner: geometry::Twist,
}

#[pymethods]
impl PyTwist {
    #[new]
    #[pyo3(signature = (linear, angular))]
    fn new(linear: [f64; 3], angular: [f64; 3]) -> Self {
        Self {
            inner: geometry::Twist::new(linear, angular),
        }
    }

    /// Create a 2D twist (forward velocity and rotation)
    #[staticmethod]
    fn new_2d(linear_x: f64, angular_z: f64) -> Self {
        Self {
            inner: geometry::Twist::new_2d(linear_x, angular_z),
        }
    }

    /// Stop command (all zeros)
    #[staticmethod]
    fn stop() -> Self {
        Self {
            inner: geometry::Twist::stop(),
        }
    }

    #[getter]
    fn linear(&self) -> [f64; 3] {
        self.inner.linear
    }

    #[setter]
    fn set_linear(&mut self, value: [f64; 3]) {
        self.inner.linear = value;
    }

    #[getter]
    fn angular(&self) -> [f64; 3] {
        self.inner.angular
    }

    #[setter]
    fn set_angular(&mut self, value: [f64; 3]) {
        self.inner.angular = value;
    }

    #[getter]
    fn timestamp(&self) -> u64 {
        self.inner.timestamp
    }

    /// Check if all values are finite
    fn is_valid(&self) -> bool {
        self.inner.is_valid()
    }

    fn __repr__(&self) -> String {
        format!(
            "Twist(linear=[{:.2}, {:.2}, {:.2}], angular=[{:.2}, {:.2}, {:.2}])",
            self.inner.linear[0],
            self.inner.linear[1],
            self.inner.linear[2],
            self.inner.angular[0],
            self.inner.angular[1],
            self.inner.angular[2]
        )
    }

    fn __str__(&self) -> String {
        self.__repr__()
    }

    /// Pickle support: Provide constructor arguments
    fn __getnewargs__(&self) -> ([f64; 3], [f64; 3]) {
        (self.inner.linear, self.inner.angular)
    }
}

/// Python wrapper for Transform
#[pyclass(module = "horus.library._library", name = "Transform")]
#[derive(Clone)]
pub struct PyTransform {
    inner: geometry::Transform,
}

#[pymethods]
impl PyTransform {
    #[new]
    #[pyo3(signature = (translation, rotation))]
    fn new(translation: [f64; 3], rotation: [f64; 4]) -> Self {
        Self {
            inner: geometry::Transform::new(translation, rotation),
        }
    }

    /// Identity transform (no translation or rotation)
    #[staticmethod]
    fn identity() -> Self {
        Self {
            inner: geometry::Transform::identity(),
        }
    }

    /// Create from 2D pose
    #[staticmethod]
    fn from_pose_2d(pose: &PyPose2D) -> Self {
        Self {
            inner: geometry::Transform::from_pose_2d(&pose.inner),
        }
    }

    #[getter]
    fn translation(&self) -> [f64; 3] {
        self.inner.translation
    }

    #[setter]
    fn set_translation(&mut self, value: [f64; 3]) {
        self.inner.translation = value;
    }

    #[getter]
    fn rotation(&self) -> [f64; 4] {
        self.inner.rotation
    }

    #[setter]
    fn set_rotation(&mut self, value: [f64; 4]) {
        self.inner.rotation = value;
    }

    #[getter]
    fn timestamp(&self) -> u64 {
        self.inner.timestamp
    }

    /// Check if quaternion is normalized and values are finite
    fn is_valid(&self) -> bool {
        self.inner.is_valid()
    }

    /// Normalize the quaternion component
    fn normalize_rotation(&mut self) {
        self.inner.normalize_rotation();
    }

    fn __repr__(&self) -> String {
        format!(
            "Transform(translation=[{:.2}, {:.2}, {:.2}], rotation=[{:.2}, {:.2}, {:.2}, {:.2}])",
            self.inner.translation[0],
            self.inner.translation[1],
            self.inner.translation[2],
            self.inner.rotation[0],
            self.inner.rotation[1],
            self.inner.rotation[2],
            self.inner.rotation[3]
        )
    }

    fn __str__(&self) -> String {
        self.__repr__()
    }

    /// Pickle support: Provide constructor arguments
    fn __getnewargs__(&self) -> ([f64; 3], [f64; 4]) {
        (self.inner.translation, self.inner.rotation)
    }
}

/// Python wrapper for Point3
#[pyclass(module = "horus.library._library", name = "Point3")]
#[derive(Clone)]
pub struct PyPoint3 {
    inner: geometry::Point3,
}

#[pymethods]
impl PyPoint3 {
    #[new]
    #[pyo3(signature = (x, y, z))]
    fn new(x: f64, y: f64, z: f64) -> Self {
        Self {
            inner: geometry::Point3::new(x, y, z),
        }
    }

    #[staticmethod]
    fn origin() -> Self {
        Self {
            inner: geometry::Point3::origin(),
        }
    }

    #[getter]
    fn x(&self) -> f64 {
        self.inner.x
    }

    #[setter]
    fn set_x(&mut self, value: f64) {
        self.inner.x = value;
    }

    #[getter]
    fn y(&self) -> f64 {
        self.inner.y
    }

    #[setter]
    fn set_y(&mut self, value: f64) {
        self.inner.y = value;
    }

    #[getter]
    fn z(&self) -> f64 {
        self.inner.z
    }

    #[setter]
    fn set_z(&mut self, value: f64) {
        self.inner.z = value;
    }

    fn distance_to(&self, other: &PyPoint3) -> f64 {
        self.inner.distance_to(&other.inner)
    }

    fn __repr__(&self) -> String {
        format!(
            "Point3(x={:.3}, y={:.3}, z={:.3})",
            self.inner.x, self.inner.y, self.inner.z
        )
    }

    fn __str__(&self) -> String {
        self.__repr__()
    }

    /// Pickle support: Provide constructor arguments
    fn __getnewargs__(&self) -> (f64, f64, f64) {
        (self.inner.x, self.inner.y, self.inner.z)
    }
}

/// Python wrapper for Vector3
#[pyclass(module = "horus.library._library", name = "Vector3")]
#[derive(Clone)]
pub struct PyVector3 {
    inner: geometry::Vector3,
}

#[pymethods]
impl PyVector3 {
    #[new]
    #[pyo3(signature = (x, y, z))]
    fn new(x: f64, y: f64, z: f64) -> Self {
        Self {
            inner: geometry::Vector3::new(x, y, z),
        }
    }

    #[staticmethod]
    fn zero() -> Self {
        Self {
            inner: geometry::Vector3::zero(),
        }
    }

    #[getter]
    fn x(&self) -> f64 {
        self.inner.x
    }

    #[setter]
    fn set_x(&mut self, value: f64) {
        self.inner.x = value;
    }

    #[getter]
    fn y(&self) -> f64 {
        self.inner.y
    }

    #[setter]
    fn set_y(&mut self, value: f64) {
        self.inner.y = value;
    }

    #[getter]
    fn z(&self) -> f64 {
        self.inner.z
    }

    #[setter]
    fn set_z(&mut self, value: f64) {
        self.inner.z = value;
    }

    fn magnitude(&self) -> f64 {
        self.inner.magnitude()
    }

    fn normalize(&mut self) {
        self.inner.normalize();
    }

    fn dot(&self, other: &PyVector3) -> f64 {
        self.inner.dot(&other.inner)
    }

    fn cross(&self, other: &PyVector3) -> PyVector3 {
        PyVector3 {
            inner: self.inner.cross(&other.inner),
        }
    }

    fn __repr__(&self) -> String {
        format!(
            "Vector3(x={:.3}, y={:.3}, z={:.3})",
            self.inner.x, self.inner.y, self.inner.z
        )
    }

    fn __str__(&self) -> String {
        self.__repr__()
    }

    /// Pickle support: Provide constructor arguments
    fn __getnewargs__(&self) -> (f64, f64, f64) {
        (self.inner.x, self.inner.y, self.inner.z)
    }
}

/// Python wrapper for Quaternion
#[pyclass(module = "horus.library._library", name = "Quaternion")]
#[derive(Clone)]
pub struct PyQuaternion {
    inner: geometry::Quaternion,
}

#[pymethods]
impl PyQuaternion {
    #[new]
    #[pyo3(signature = (x, y, z, w))]
    fn new(x: f64, y: f64, z: f64, w: f64) -> Self {
        Self {
            inner: geometry::Quaternion::new(x, y, z, w),
        }
    }

    #[staticmethod]
    fn identity() -> Self {
        Self {
            inner: geometry::Quaternion::identity(),
        }
    }

    #[staticmethod]
    fn from_euler(roll: f64, pitch: f64, yaw: f64) -> Self {
        Self {
            inner: geometry::Quaternion::from_euler(roll, pitch, yaw),
        }
    }

    #[getter]
    fn x(&self) -> f64 {
        self.inner.x
    }

    #[setter]
    fn set_x(&mut self, value: f64) {
        self.inner.x = value;
    }

    #[getter]
    fn y(&self) -> f64 {
        self.inner.y
    }

    #[setter]
    fn set_y(&mut self, value: f64) {
        self.inner.y = value;
    }

    #[getter]
    fn z(&self) -> f64 {
        self.inner.z
    }

    #[setter]
    fn set_z(&mut self, value: f64) {
        self.inner.z = value;
    }

    #[getter]
    fn w(&self) -> f64 {
        self.inner.w
    }

    #[setter]
    fn set_w(&mut self, value: f64) {
        self.inner.w = value;
    }

    fn normalize(&mut self) {
        self.inner.normalize();
    }

    fn is_valid(&self) -> bool {
        self.inner.is_valid()
    }

    fn __repr__(&self) -> String {
        format!(
            "Quaternion(x={:.3}, y={:.3}, z={:.3}, w={:.3})",
            self.inner.x, self.inner.y, self.inner.z, self.inner.w
        )
    }

    fn __str__(&self) -> String {
        self.__repr__()
    }

    /// Pickle support: Provide constructor arguments
    fn __getnewargs__(&self) -> (f64, f64, f64, f64) {
        (self.inner.x, self.inner.y, self.inner.z, self.inner.w)
    }
}

/// Python wrapper for CmdVel (2D velocity command)
#[pyclass(module = "horus.library._library", name = "CmdVel")]
#[derive(Clone)]
pub struct PyCmdVel {
    inner: cmd_vel::CmdVel,
}

#[pymethods]
impl PyCmdVel {
    #[new]
    #[pyo3(signature = (linear, angular, timestamp=None))]
    fn new(linear: f32, angular: f32, timestamp: Option<u64>) -> Self {
        Self {
            inner: match timestamp {
                Some(ts) => cmd_vel::CmdVel::with_timestamp(linear, angular, ts),
                None => cmd_vel::CmdVel::new(linear, angular),
            },
        }
    }

    /// Create a zero velocity command (stop)
    #[staticmethod]
    fn zero() -> Self {
        Self {
            inner: cmd_vel::CmdVel::zero(),
        }
    }

    #[getter]
    fn linear(&self) -> f32 {
        self.inner.linear
    }

    #[setter]
    fn set_linear(&mut self, value: f32) {
        self.inner.linear = value;
    }

    #[getter]
    fn angular(&self) -> f32 {
        self.inner.angular
    }

    #[setter]
    fn set_angular(&mut self, value: f32) {
        self.inner.angular = value;
    }

    #[getter]
    fn timestamp(&self) -> u64 {
        self.inner.stamp_nanos
    }

    fn __repr__(&self) -> String {
        format!(
            "CmdVel(linear={:.2}, angular={:.2})",
            self.inner.linear, self.inner.angular
        )
    }

    fn __str__(&self) -> String {
        self.__repr__()
    }

    /// Pickle support: Provide constructor arguments
    fn __getnewargs__(&self) -> (f32, f32) {
        (self.inner.linear, self.inner.angular)
    }
}

/// Python wrapper for LaserScan (2D lidar data)
#[pyclass(module = "horus.library._library", name = "LaserScan")]
#[derive(Clone)]
pub struct PyLaserScan {
    inner: sensor::LaserScan,
}

#[pymethods]
impl PyLaserScan {
    #[new]
    fn new() -> Self {
        Self {
            inner: sensor::LaserScan::new(),
        }
    }

    /// Get ranges as NumPy array (zero-copy view)
    #[getter]
    fn ranges<'py>(&self, py: Python<'py>) -> Bound<'py, PyArray1<f32>> {
        PyArray1::from_slice(py, &self.inner.ranges)
    }

    /// Set ranges from Python list or NumPy array
    #[setter]
    fn set_ranges(&mut self, _py: Python, value: &Bound<'_, PyAny>) -> PyResult<()> {
        // Try extracting as Vec<f32> - works for both NumPy arrays and lists
        if let Ok(vec) = value.extract::<Vec<f32>>() {
            if vec.len() == 360 {
                self.inner.ranges.copy_from_slice(&vec);
                return Ok(());
            } else {
                return Err(pyo3::exceptions::PyValueError::new_err(format!(
                    "Input must have exactly 360 elements, got {}",
                    vec.len()
                )));
            }
        }

        Err(pyo3::exceptions::PyTypeError::new_err(
            "ranges must be a NumPy array or Python list of floats",
        ))
    }

    #[getter]
    fn angle_min(&self) -> f32 {
        self.inner.angle_min
    }

    #[setter]
    fn set_angle_min(&mut self, value: f32) {
        self.inner.angle_min = value;
    }

    #[getter]
    fn angle_max(&self) -> f32 {
        self.inner.angle_max
    }

    #[setter]
    fn set_angle_max(&mut self, value: f32) {
        self.inner.angle_max = value;
    }

    #[getter]
    fn range_min(&self) -> f32 {
        self.inner.range_min
    }

    #[setter]
    fn set_range_min(&mut self, value: f32) {
        self.inner.range_min = value;
    }

    #[getter]
    fn range_max(&self) -> f32 {
        self.inner.range_max
    }

    #[setter]
    fn set_range_max(&mut self, value: f32) {
        self.inner.range_max = value;
    }

    #[getter]
    fn angle_increment(&self) -> f32 {
        self.inner.angle_increment
    }

    #[setter]
    fn set_angle_increment(&mut self, value: f32) {
        self.inner.angle_increment = value;
    }

    #[getter]
    fn timestamp(&self) -> u64 {
        self.inner.timestamp
    }

    /// Get the angle for a specific range index
    fn angle_at(&self, index: usize) -> f32 {
        self.inner.angle_at(index)
    }

    /// Check if a range reading is valid
    fn is_range_valid(&self, index: usize) -> bool {
        self.inner.is_range_valid(index)
    }

    /// Count valid range readings
    fn valid_count(&self) -> usize {
        self.inner.valid_count()
    }

    /// Get minimum valid range reading
    fn min_range(&self) -> Option<f32> {
        self.inner.min_range()
    }

    fn __repr__(&self) -> String {
        format!(
            "LaserScan(ranges={}, valid={}, min={:.2}m)",
            self.inner.ranges.len(),
            self.inner.valid_count(),
            self.inner.min_range().unwrap_or(0.0)
        )
    }

    fn __str__(&self) -> String {
        self.__repr__()
    }

    fn __len__(&self) -> usize {
        360
    }

    /// Pickle support: Return empty args since LaserScan() has no constructor args,
    /// but we preserve state via __getstate__/__setstate__ pattern
    fn __getnewargs__<'py>(&self, py: Python<'py>) -> Bound<'py, PyTuple> {
        PyTuple::empty(py)
    }

    /// Pickle support: Return the full state for unpickling
    fn __getstate__(&self, py: Python) -> PyResult<PyObject> {
        let state = PyDict::new(py);
        state.set_item("ranges", self.inner.ranges.to_vec())?;
        state.set_item("angle_min", self.inner.angle_min)?;
        state.set_item("angle_max", self.inner.angle_max)?;
        state.set_item("range_min", self.inner.range_min)?;
        state.set_item("range_max", self.inner.range_max)?;
        state.set_item("angle_increment", self.inner.angle_increment)?;
        state.set_item("timestamp", self.inner.timestamp)?;
        Ok(state.into())
    }

    /// Pickle support: Restore the full state from unpickling
    fn __setstate__(&mut self, state: &Bound<'_, PyDict>) -> PyResult<()> {
        let ranges: Vec<f32> = state.get_item("ranges")?.unwrap().extract()?;
        let angle_min: f32 = state.get_item("angle_min")?.unwrap().extract()?;
        let angle_max: f32 = state.get_item("angle_max")?.unwrap().extract()?;
        let range_min: f32 = state.get_item("range_min")?.unwrap().extract()?;
        let range_max: f32 = state.get_item("range_max")?.unwrap().extract()?;
        let angle_increment: f32 = state.get_item("angle_increment")?.unwrap().extract()?;
        let timestamp: u64 = state.get_item("timestamp")?.unwrap().extract()?;

        self.inner.ranges.copy_from_slice(&ranges);
        self.inner.angle_min = angle_min;
        self.inner.angle_max = angle_max;
        self.inner.range_min = range_min;
        self.inner.range_max = range_max;
        self.inner.angle_increment = angle_increment;
        self.inner.timestamp = timestamp;

        Ok(())
    }
}

/// Python wrapper for IMU sensor data
#[pyclass(module = "horus.library._library", name = "Imu")]
#[derive(Clone)]
pub struct PyImu {
    inner: sensor::Imu,
}

#[pymethods]
impl PyImu {
    #[new]
    fn new() -> Self {
        Self {
            inner: sensor::Imu::new(),
        }
    }

    #[getter]
    fn orientation(&self) -> [f64; 4] {
        self.inner.orientation
    }

    #[setter]
    fn set_orientation(&mut self, value: [f64; 4]) {
        self.inner.orientation = value;
    }

    #[getter]
    fn angular_velocity(&self) -> [f64; 3] {
        self.inner.angular_velocity
    }

    #[setter]
    fn set_angular_velocity(&mut self, value: [f64; 3]) {
        self.inner.angular_velocity = value;
    }

    #[getter]
    fn linear_acceleration(&self) -> [f64; 3] {
        self.inner.linear_acceleration
    }

    #[setter]
    fn set_linear_acceleration(&mut self, value: [f64; 3]) {
        self.inner.linear_acceleration = value;
    }

    #[getter]
    fn orientation_covariance(&self) -> [f64; 9] {
        self.inner.orientation_covariance
    }

    #[setter]
    fn set_orientation_covariance(&mut self, value: [f64; 9]) {
        self.inner.orientation_covariance = value;
    }

    #[getter]
    fn angular_velocity_covariance(&self) -> [f64; 9] {
        self.inner.angular_velocity_covariance
    }

    #[setter]
    fn set_angular_velocity_covariance(&mut self, value: [f64; 9]) {
        self.inner.angular_velocity_covariance = value;
    }

    #[getter]
    fn linear_acceleration_covariance(&self) -> [f64; 9] {
        self.inner.linear_acceleration_covariance
    }

    #[setter]
    fn set_linear_acceleration_covariance(&mut self, value: [f64; 9]) {
        self.inner.linear_acceleration_covariance = value;
    }

    #[getter]
    fn timestamp(&self) -> u64 {
        self.inner.timestamp
    }

    fn set_orientation_from_euler(&mut self, roll: f64, pitch: f64, yaw: f64) {
        self.inner.set_orientation_from_euler(roll, pitch, yaw);
    }

    fn has_orientation(&self) -> bool {
        self.inner.has_orientation()
    }

    fn is_valid(&self) -> bool {
        self.inner.is_valid()
    }

    fn __repr__(&self) -> String {
        format!(
            "Imu(angular_vel=[{:.2}, {:.2}, {:.2}], linear_accel=[{:.2}, {:.2}, {:.2}])",
            self.inner.angular_velocity[0],
            self.inner.angular_velocity[1],
            self.inner.angular_velocity[2],
            self.inner.linear_acceleration[0],
            self.inner.linear_acceleration[1],
            self.inner.linear_acceleration[2]
        )
    }

    fn __str__(&self) -> String {
        self.__repr__()
    }
}

/// Python wrapper for Battery State
#[pyclass(module = "horus.library._library", name = "BatteryState")]
#[derive(Clone)]
pub struct PyBatteryState {
    inner: sensor::BatteryState,
}

#[pymethods]
impl PyBatteryState {
    #[new]
    #[pyo3(signature = (voltage=0.0, percentage=0.0))]
    fn new(voltage: f32, percentage: f32) -> Self {
        Self {
            inner: sensor::BatteryState::new(voltage, percentage),
        }
    }

    #[getter]
    fn voltage(&self) -> f32 {
        self.inner.voltage
    }

    #[setter]
    fn set_voltage(&mut self, value: f32) {
        self.inner.voltage = value;
    }

    #[getter]
    fn current(&self) -> f32 {
        self.inner.current
    }

    #[setter]
    fn set_current(&mut self, value: f32) {
        self.inner.current = value;
    }

    #[getter]
    fn charge(&self) -> f32 {
        self.inner.charge
    }

    #[setter]
    fn set_charge(&mut self, value: f32) {
        self.inner.charge = value;
    }

    #[getter]
    fn capacity(&self) -> f32 {
        self.inner.capacity
    }

    #[setter]
    fn set_capacity(&mut self, value: f32) {
        self.inner.capacity = value;
    }

    #[getter]
    fn percentage(&self) -> f32 {
        self.inner.percentage
    }

    #[setter]
    fn set_percentage(&mut self, value: f32) {
        self.inner.percentage = value;
    }

    #[getter]
    fn power_supply_status(&self) -> u8 {
        self.inner.power_supply_status
    }

    #[setter]
    fn set_power_supply_status(&mut self, value: u8) {
        self.inner.power_supply_status = value;
    }

    #[getter]
    fn temperature(&self) -> f32 {
        self.inner.temperature
    }

    #[setter]
    fn set_temperature(&mut self, value: f32) {
        self.inner.temperature = value;
    }

    #[getter]
    fn timestamp(&self) -> u64 {
        self.inner.timestamp
    }

    fn is_low(&self, threshold: f32) -> bool {
        self.inner.is_low(threshold)
    }

    fn is_critical(&self) -> bool {
        self.inner.is_critical()
    }

    fn time_remaining(&self) -> Option<f32> {
        self.inner.time_remaining()
    }

    fn __repr__(&self) -> String {
        format!(
            "BatteryState(voltage={:.2}V, {}%, status={})",
            self.inner.voltage, self.inner.percentage, self.inner.power_supply_status
        )
    }

    fn __str__(&self) -> String {
        self.__repr__()
    }
}

/// Python wrapper for NavSatFix (GPS data)
#[pyclass(module = "horus.library._library", name = "NavSatFix")]
#[derive(Clone)]
pub struct PyNavSatFix {
    inner: sensor::NavSatFix,
}

#[pymethods]
impl PyNavSatFix {
    #[new]
    #[pyo3(signature = (latitude=0.0, longitude=0.0, altitude=0.0))]
    fn new(latitude: f64, longitude: f64, altitude: f64) -> Self {
        Self {
            inner: if latitude == 0.0 && longitude == 0.0 && altitude == 0.0 {
                sensor::NavSatFix::new()
            } else {
                sensor::NavSatFix::from_coordinates(latitude, longitude, altitude)
            },
        }
    }

    #[getter]
    fn latitude(&self) -> f64 {
        self.inner.latitude
    }

    #[setter]
    fn set_latitude(&mut self, value: f64) {
        self.inner.latitude = value;
    }

    #[getter]
    fn longitude(&self) -> f64 {
        self.inner.longitude
    }

    #[setter]
    fn set_longitude(&mut self, value: f64) {
        self.inner.longitude = value;
    }

    #[getter]
    fn altitude(&self) -> f64 {
        self.inner.altitude
    }

    #[setter]
    fn set_altitude(&mut self, value: f64) {
        self.inner.altitude = value;
    }

    #[getter]
    fn status(&self) -> u8 {
        self.inner.status
    }

    #[setter]
    fn set_status(&mut self, value: u8) {
        self.inner.status = value;
    }

    #[getter]
    fn satellites_visible(&self) -> u16 {
        self.inner.satellites_visible
    }

    #[setter]
    fn set_satellites_visible(&mut self, value: u16) {
        self.inner.satellites_visible = value;
    }

    #[getter]
    fn hdop(&self) -> f32 {
        self.inner.hdop
    }

    #[setter]
    fn set_hdop(&mut self, value: f32) {
        self.inner.hdop = value;
    }

    #[getter]
    fn vdop(&self) -> f32 {
        self.inner.vdop
    }

    #[setter]
    fn set_vdop(&mut self, value: f32) {
        self.inner.vdop = value;
    }

    #[getter]
    fn speed(&self) -> f32 {
        self.inner.speed
    }

    #[setter]
    fn set_speed(&mut self, value: f32) {
        self.inner.speed = value;
    }

    #[getter]
    fn heading(&self) -> f32 {
        self.inner.heading
    }

    #[setter]
    fn set_heading(&mut self, value: f32) {
        self.inner.heading = value;
    }

    #[getter]
    fn timestamp(&self) -> u64 {
        self.inner.timestamp
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

    fn distance_to(&self, other: &PyNavSatFix) -> f64 {
        self.inner.distance_to(&other.inner)
    }

    fn __repr__(&self) -> String {
        format!(
            "NavSatFix(lat={:.6}, lon={:.6}, alt={:.1}m, sats={})",
            self.inner.latitude,
            self.inner.longitude,
            self.inner.altitude,
            self.inner.satellites_visible
        )
    }

    fn __str__(&self) -> String {
        self.__repr__()
    }
}

/// Python wrapper for Odometry
#[pyclass(module = "horus.library._library", name = "Odometry")]
#[derive(Clone)]
pub struct PyOdometry {
    inner: sensor::Odometry,
}

#[pymethods]
impl PyOdometry {
    #[new]
    fn new() -> Self {
        Self {
            inner: sensor::Odometry::new(),
        }
    }

    #[getter]
    fn pose(&self) -> PyPose2D {
        PyPose2D {
            inner: self.inner.pose,
        }
    }

    #[setter]
    fn set_pose(&mut self, value: &PyPose2D) {
        self.inner.pose = value.inner;
    }

    #[getter]
    fn twist(&self) -> PyTwist {
        PyTwist {
            inner: self.inner.twist,
        }
    }

    #[setter]
    fn set_twist(&mut self, value: &PyTwist) {
        self.inner.twist = value.inner;
    }

    #[getter]
    fn timestamp(&self) -> u64 {
        self.inner.timestamp
    }

    fn update(&mut self, pose: &PyPose2D, twist: &PyTwist) {
        self.inner.update(pose.inner, twist.inner);
    }

    fn is_valid(&self) -> bool {
        self.inner.is_valid()
    }

    fn __repr__(&self) -> String {
        format!(
            "Odometry(pose=Pose2D({:.2}, {:.2}, {:.2}), twist=[{:.2}, {:.2}])",
            self.inner.pose.x,
            self.inner.pose.y,
            self.inner.pose.theta,
            self.inner.twist.linear[0],
            self.inner.twist.angular[2]
        )
    }

    fn __str__(&self) -> String {
        self.__repr__()
    }
}

/// Python wrapper for Range sensor
#[pyclass(module = "horus.library._library", name = "Range")]
#[derive(Clone)]
pub struct PyRange {
    inner: sensor::Range,
}

#[pymethods]
impl PyRange {
    #[new]
    #[pyo3(signature = (sensor_type=0, range=0.0))]
    fn new(sensor_type: u8, range: f32) -> Self {
        Self {
            inner: sensor::Range::new(sensor_type, range),
        }
    }

    #[classattr]
    const ULTRASONIC: u8 = sensor::Range::ULTRASONIC;

    #[classattr]
    const INFRARED: u8 = sensor::Range::INFRARED;

    #[getter]
    fn sensor_type(&self) -> u8 {
        self.inner.sensor_type
    }

    #[setter]
    fn set_sensor_type(&mut self, value: u8) {
        self.inner.sensor_type = value;
    }

    #[getter]
    fn field_of_view(&self) -> f32 {
        self.inner.field_of_view
    }

    #[setter]
    fn set_field_of_view(&mut self, value: f32) {
        self.inner.field_of_view = value;
    }

    #[getter]
    fn min_range(&self) -> f32 {
        self.inner.min_range
    }

    #[setter]
    fn set_min_range(&mut self, value: f32) {
        self.inner.min_range = value;
    }

    #[getter]
    fn max_range(&self) -> f32 {
        self.inner.max_range
    }

    #[setter]
    fn set_max_range(&mut self, value: f32) {
        self.inner.max_range = value;
    }

    #[getter]
    fn range(&self) -> f32 {
        self.inner.range
    }

    #[setter]
    fn set_range(&mut self, value: f32) {
        self.inner.range = value;
    }

    #[getter]
    fn timestamp(&self) -> u64 {
        self.inner.timestamp
    }

    fn is_valid(&self) -> bool {
        self.inner.is_valid()
    }

    fn __repr__(&self) -> String {
        format!(
            "Range(type={}, range={:.2}m)",
            self.inner.sensor_type, self.inner.range
        )
    }

    fn __str__(&self) -> String {
        self.__repr__()
    }
}

/// Register sim2d submodule
fn register_sim2d(py: Python, parent: &Bound<'_, PyModule>) -> PyResult<()> {
    use sim2d::python_api::{RobotConfigPy, Sim2D, WorldConfigPy};

    let sim2d_module = PyModule::new(py, "sim2d")?;
    sim2d_module.add_class::<Sim2D>()?;
    sim2d_module.add_class::<RobotConfigPy>()?;
    sim2d_module.add_class::<WorldConfigPy>()?;
    parent.add_submodule(&sim2d_module)?;
    Ok(())
}

/// Register sim3d submodule (feature-gated)
#[cfg(feature = "sim3d")]
fn register_sim3d(py: Python, parent: &Bound<'_, PyModule>) -> PyResult<()> {
    use sim3d::rl::python::{make_env, make_vec_env, PySim3DEnv, PyVecSim3DEnv};

    let sim3d_module = PyModule::new(py, "sim3d")?;
    sim3d_module.add_class::<PySim3DEnv>()?;
    sim3d_module.add_class::<PyVecSim3DEnv>()?;
    sim3d_module.add_function(wrap_pyfunction!(make_env, &sim3d_module)?)?;
    sim3d_module.add_function(wrap_pyfunction!(make_vec_env, &sim3d_module)?)?;
    parent.add_submodule(&sim3d_module)?;
    Ok(())
}

/// HORUS Library Python Module
#[pymodule]
fn _library(py: Python, m: &Bound<'_, PyModule>) -> PyResult<()> {
    // Geometry messages
    m.add_class::<PyPose2D>()?;
    m.add_class::<PyTwist>()?;
    m.add_class::<PyTransform>()?;
    m.add_class::<PyPoint3>()?;
    m.add_class::<PyVector3>()?;
    m.add_class::<PyQuaternion>()?;

    // Control messages
    m.add_class::<PyCmdVel>()?;
    m.add_class::<PyMotorCommand>()?;
    m.add_class::<PyDifferentialDriveCommand>()?;
    m.add_class::<PyServoCommand>()?;
    m.add_class::<PyPwmCommand>()?;
    m.add_class::<PyStepperCommand>()?;
    m.add_class::<PyPidConfig>()?;

    // Sensor messages
    m.add_class::<PyLaserScan>()?;
    m.add_class::<PyImu>()?;
    m.add_class::<PyBatteryState>()?;
    m.add_class::<PyNavSatFix>()?;
    m.add_class::<PyOdometry>()?;
    m.add_class::<PyRange>()?;

    // Diagnostics messages
    m.add_class::<PyStatus>()?;
    m.add_class::<PyEmergencyStop>()?;
    m.add_class::<PyHeartbeat>()?;
    m.add_class::<PyResourceUsage>()?;

    // Input messages
    m.add_class::<PyJoystickInput>()?;
    m.add_class::<PyKeyboardInput>()?;

    // I/O messages
    m.add_class::<PyDigitalIO>()?;
    m.add_class::<PyAnalogIO>()?;

    // Vision messages
    m.add_class::<PyImageEncoding>()?;
    m.add_class::<PyImage>()?;
    m.add_class::<PyCompressedImage>()?;
    m.add_class::<PyCameraInfo>()?;
    m.add_class::<PyRegionOfInterest>()?;
    m.add_class::<PyDetection>()?;
    m.add_class::<PyDetectionArray>()?;
    m.add_class::<PyStereoInfo>()?;

    // Navigation messages
    m.add_class::<PyGoalStatus>()?;
    m.add_class::<PyGoal>()?;
    m.add_class::<PyGoalResult>()?;
    m.add_class::<PyWaypoint>()?;
    m.add_class::<PyPath>()?;
    m.add_class::<PyOccupancyGrid>()?;
    m.add_class::<PyCostMap>()?;
    m.add_class::<PyVelocityObstacle>()?;
    m.add_class::<PyVelocityObstacles>()?;
    m.add_class::<PyPathPlan>()?;

    // Perception messages
    m.add_class::<PyPointFieldType>()?;
    m.add_class::<PyPointField>()?;
    m.add_class::<PyPointCloud>()?;
    m.add_class::<PyBoundingBox3D>()?;
    m.add_class::<PyBoundingBoxArray3D>()?;
    m.add_class::<PyDepthImage>()?;
    m.add_class::<PyPlaneDetection>()?;
    m.add_class::<PyPlaneArray>()?;

    // Force/Tactile messages
    m.add_class::<PyContactState>()?;
    m.add_class::<PyWrenchStamped>()?;
    m.add_class::<PyTactileArray>()?;
    m.add_class::<PyImpedanceParameters>()?;
    m.add_class::<PyForceCommand>()?;
    m.add_class::<PyContactInfo>()?;
    m.add_class::<PyHapticFeedback>()?;

    // Coordination messages
    m.add_class::<PyRobotType>()?;
    m.add_class::<PyCoordinationMode>()?;
    m.add_class::<PyTaskType>()?;
    m.add_class::<PyTaskStatus>()?;
    m.add_class::<PyFormationType>()?;
    m.add_class::<PyBidStatus>()?;
    m.add_class::<PyRobotState>()?;
    m.add_class::<PyFleetStatus>()?;
    m.add_class::<PyTaskAssignment>()?;
    m.add_class::<PyFormationControl>()?;
    m.add_class::<PyAuctionBid>()?;

    // Register simulation submodules
    register_sim2d(py, m)?;

    #[cfg(feature = "sim3d")]
    register_sim3d(py, m)?;

    Ok(())
}
