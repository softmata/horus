// Python wrappers for force and tactile messages
use horus_library::messages::force;
use pyo3::prelude::*;

use crate::{PyPoint3, PyVector3};

/// Contact state enum
#[pyclass(module = "horus.library._library", name = "ContactState")]
#[derive(Clone, Copy)]
pub struct PyContactState {
    pub(crate) inner: force::ContactState,
}

#[pymethods]
impl PyContactState {
    #[classattr]
    const NO_CONTACT: u8 = 0;
    #[classattr]
    const INITIAL_CONTACT: u8 = 1;
    #[classattr]
    const STABLE_CONTACT: u8 = 2;
    #[classattr]
    const CONTACT_LOSS: u8 = 3;
    #[classattr]
    const SLIDING: u8 = 4;
    #[classattr]
    const IMPACT: u8 = 5;

    #[new]
    #[pyo3(signature = (state=0))]
    fn new(state: u8) -> Self {
        let inner = match state {
            0 => force::ContactState::NoContact,
            1 => force::ContactState::InitialContact,
            2 => force::ContactState::StableContact,
            3 => force::ContactState::ContactLoss,
            4 => force::ContactState::Sliding,
            5 => force::ContactState::Impact,
            _ => force::ContactState::NoContact,
        };
        Self { inner }
    }

    fn __repr__(&self) -> String {
        format!("{:?}", self.inner)
    }
}

/// Python wrapper for WrenchStamped
#[pyclass(module = "horus.library._library", name = "WrenchStamped")]
#[derive(Clone)]
pub struct PyWrenchStamped {
    pub(crate) inner: force::WrenchStamped,
}

#[pymethods]
impl PyWrenchStamped {
    #[new]
    #[pyo3(signature = (force=None, torque=None))]
    fn new(force: Option<&PyVector3>, torque: Option<&PyVector3>) -> Self {
        let f = force
            .map(|v| {
                horus_library::messages::geometry::Vector3::new(v.inner.x, v.inner.y, v.inner.z)
            })
            .unwrap_or_default();
        let t = torque
            .map(|v| {
                horus_library::messages::geometry::Vector3::new(v.inner.x, v.inner.y, v.inner.z)
            })
            .unwrap_or_default();
        Self {
            inner: force::WrenchStamped::new(f, t),
        }
    }

    #[staticmethod]
    fn force_only(force: &PyVector3) -> Self {
        let f = horus_library::messages::geometry::Vector3::new(
            force.inner.x,
            force.inner.y,
            force.inner.z,
        );
        Self {
            inner: force::WrenchStamped::force_only(f),
        }
    }

    #[staticmethod]
    fn torque_only(torque: &PyVector3) -> Self {
        let t = horus_library::messages::geometry::Vector3::new(
            torque.inner.x,
            torque.inner.y,
            torque.inner.z,
        );
        Self {
            inner: force::WrenchStamped::torque_only(t),
        }
    }

    #[getter]
    fn force(&self) -> PyVector3 {
        PyVector3 {
            inner: horus_library::messages::geometry::Vector3::new(
                self.inner.force.x,
                self.inner.force.y,
                self.inner.force.z,
            ),
        }
    }

    #[getter]
    fn torque(&self) -> PyVector3 {
        PyVector3 {
            inner: horus_library::messages::geometry::Vector3::new(
                self.inner.torque.x,
                self.inner.torque.y,
                self.inner.torque.z,
            ),
        }
    }

    #[getter]
    fn point_of_application(&self) -> PyPoint3 {
        PyPoint3 {
            inner: horus_library::messages::geometry::Point3::new(
                self.inner.point_of_application.x,
                self.inner.point_of_application.y,
                self.inner.point_of_application.z,
            ),
        }
    }

    #[getter]
    fn timestamp(&self) -> u64 {
        self.inner.timestamp
    }

    fn force_magnitude(&self) -> f64 {
        self.inner.force_magnitude()
    }

    fn torque_magnitude(&self) -> f64 {
        self.inner.torque_magnitude()
    }

    fn exceeds_limits(&self, max_force: f64, max_torque: f64) -> bool {
        self.inner.exceeds_limits(max_force, max_torque)
    }

    fn __repr__(&self) -> String {
        format!(
            "WrenchStamped(force=[{:.2}, {:.2}, {:.2}]N, torque=[{:.2}, {:.2}, {:.2}]Nm)",
            self.inner.force.x,
            self.inner.force.y,
            self.inner.force.z,
            self.inner.torque.x,
            self.inner.torque.y,
            self.inner.torque.z
        )
    }
}

/// Python wrapper for TactileArray
#[pyclass(module = "horus.library._library", name = "TactileArray")]
#[derive(Clone)]
pub struct PyTactileArray {
    pub(crate) inner: force::TactileArray,
}

#[pymethods]
impl PyTactileArray {
    #[classattr]
    const ARRANGEMENT_GRID: u8 = 0;
    #[classattr]
    const ARRANGEMENT_LINEAR: u8 = 1;
    #[classattr]
    const ARRANGEMENT_CIRCULAR: u8 = 2;

    #[new]
    #[pyo3(signature = (sensor_count=0, arrangement=0))]
    fn new(sensor_count: u8, arrangement: u8) -> Self {
        Self {
            inner: force::TactileArray::new(sensor_count, arrangement),
        }
    }

    #[getter]
    fn sensor_count(&self) -> u8 {
        self.inner.sensor_count
    }

    #[getter]
    fn arrangement(&self) -> u8 {
        self.inner.arrangement
    }

    #[getter]
    fn grid_width(&self) -> u8 {
        self.inner.grid_width
    }

    #[setter]
    fn set_grid_width(&mut self, value: u8) {
        self.inner.grid_width = value;
    }

    #[getter]
    fn grid_height(&self) -> u8 {
        self.inner.grid_height
    }

    #[setter]
    fn set_grid_height(&mut self, value: u8) {
        self.inner.grid_height = value;
    }

    #[getter]
    fn sensor_spacing(&self) -> f32 {
        self.inner.sensor_spacing
    }

    #[setter]
    fn set_sensor_spacing(&mut self, value: f32) {
        self.inner.sensor_spacing = value;
    }

    #[getter]
    fn sensitivity(&self) -> f32 {
        self.inner.sensitivity
    }

    #[setter]
    fn set_sensitivity(&mut self, value: f32) {
        self.inner.sensitivity = value;
    }

    #[getter]
    fn timestamp(&self) -> u64 {
        self.inner.timestamp
    }

    fn get_active_sensors(&self) -> Vec<f32> {
        self.inner.get_active_sensors().to_vec()
    }

    fn set_sensor(&mut self, index: u8, value: f32) -> bool {
        self.inner.set_sensor(index, value)
    }

    fn get_sensor(&self, index: u8) -> Option<f32> {
        self.inner.get_sensor(index)
    }

    fn total_force(&self) -> f32 {
        self.inner.total_force()
    }

    fn center_of_pressure(&self) -> Option<(f32, f32)> {
        self.inner.center_of_pressure()
    }

    fn detect_contact(&self, threshold: f32) -> bool {
        self.inner.detect_contact(threshold)
    }

    fn contact_pattern(&self, threshold: f32) -> Vec<bool> {
        self.inner.contact_pattern(threshold)
    }

    fn __len__(&self) -> usize {
        self.inner.sensor_count as usize
    }

    fn __repr__(&self) -> String {
        format!(
            "TactileArray({} sensors, arrangement={}, total_force={:.2})",
            self.inner.sensor_count,
            self.inner.arrangement,
            self.inner.total_force()
        )
    }
}

/// Python wrapper for ImpedanceParameters
#[pyclass(module = "horus.library._library", name = "ImpedanceParameters")]
#[derive(Clone)]
pub struct PyImpedanceParameters {
    pub(crate) inner: force::ImpedanceParameters,
}

#[pymethods]
impl PyImpedanceParameters {
    #[new]
    fn new() -> Self {
        Self {
            inner: force::ImpedanceParameters::new(),
        }
    }

    #[staticmethod]
    fn compliant() -> Self {
        Self {
            inner: force::ImpedanceParameters::compliant(),
        }
    }

    #[staticmethod]
    fn stiff() -> Self {
        Self {
            inner: force::ImpedanceParameters::stiff(),
        }
    }

    #[getter]
    fn stiffness(&self) -> [f64; 6] {
        self.inner.stiffness
    }

    #[setter]
    fn set_stiffness(&mut self, value: [f64; 6]) {
        self.inner.stiffness = value;
    }

    #[getter]
    fn damping(&self) -> [f64; 6] {
        self.inner.damping
    }

    #[setter]
    fn set_damping(&mut self, value: [f64; 6]) {
        self.inner.damping = value;
    }

    #[getter]
    fn inertia(&self) -> [f64; 6] {
        self.inner.inertia
    }

    #[setter]
    fn set_inertia(&mut self, value: [f64; 6]) {
        self.inner.inertia = value;
    }

    #[getter]
    fn force_limits(&self) -> [f64; 6] {
        self.inner.force_limits
    }

    #[setter]
    fn set_force_limits(&mut self, value: [f64; 6]) {
        self.inner.force_limits = value;
    }

    #[getter]
    fn enabled(&self) -> bool {
        self.inner.enabled
    }

    #[getter]
    fn timestamp(&self) -> u64 {
        self.inner.timestamp
    }

    fn enable(&mut self) {
        self.inner.enable();
    }

    fn disable(&mut self) {
        self.inner.disable();
    }

    fn __repr__(&self) -> String {
        format!(
            "ImpedanceParameters(enabled={}, stiffness=[{:.0}, {:.0}, {:.0}, ...])",
            self.inner.enabled,
            self.inner.stiffness[0],
            self.inner.stiffness[1],
            self.inner.stiffness[2]
        )
    }
}

/// Python wrapper for ForceCommand
#[pyclass(module = "horus.library._library", name = "ForceCommand")]
#[derive(Clone)]
pub struct PyForceCommand {
    pub(crate) inner: force::ForceCommand,
}

#[pymethods]
impl PyForceCommand {
    #[new]
    fn new() -> Self {
        Self {
            inner: force::ForceCommand::default(),
        }
    }

    #[staticmethod]
    fn force_only(target_force: &PyVector3) -> Self {
        let f = horus_library::messages::geometry::Vector3::new(
            target_force.inner.x,
            target_force.inner.y,
            target_force.inner.z,
        );
        Self {
            inner: force::ForceCommand::force_only(f),
        }
    }

    #[getter]
    fn target_force(&self) -> PyVector3 {
        PyVector3 {
            inner: horus_library::messages::geometry::Vector3::new(
                self.inner.target_force.x,
                self.inner.target_force.y,
                self.inner.target_force.z,
            ),
        }
    }

    #[getter]
    fn target_torque(&self) -> PyVector3 {
        PyVector3 {
            inner: horus_library::messages::geometry::Vector3::new(
                self.inner.target_torque.x,
                self.inner.target_torque.y,
                self.inner.target_torque.z,
            ),
        }
    }

    #[getter]
    fn force_mode(&self) -> [bool; 6] {
        self.inner.force_mode
    }

    #[setter]
    fn set_force_mode(&mut self, value: [bool; 6]) {
        self.inner.force_mode = value;
    }

    #[getter]
    fn gains(&self) -> [f64; 6] {
        self.inner.gains
    }

    #[setter]
    fn set_gains(&mut self, value: [f64; 6]) {
        self.inner.gains = value;
    }

    #[getter]
    fn timeout_seconds(&self) -> f64 {
        self.inner.timeout_seconds
    }

    #[setter]
    fn set_timeout_seconds(&mut self, value: f64) {
        self.inner.timeout_seconds = value;
    }

    #[getter]
    fn timestamp(&self) -> u64 {
        self.inner.timestamp
    }

    fn __repr__(&self) -> String {
        format!(
            "ForceCommand(target_force=[{:.2}, {:.2}, {:.2}]N)",
            self.inner.target_force.x, self.inner.target_force.y, self.inner.target_force.z
        )
    }
}

/// Python wrapper for ContactInfo
#[pyclass(module = "horus.library._library", name = "ContactInfo")]
#[derive(Clone)]
pub struct PyContactInfo {
    pub(crate) inner: force::ContactInfo,
}

#[pymethods]
impl PyContactInfo {
    #[new]
    #[pyo3(signature = (state=None, force_magnitude=0.0))]
    fn new(state: Option<&PyContactState>, force_magnitude: f64) -> Self {
        let s = state
            .map(|s| s.inner)
            .unwrap_or(force::ContactState::NoContact);
        Self {
            inner: force::ContactInfo::new(s, force_magnitude),
        }
    }

    #[getter]
    fn state(&self) -> PyContactState {
        PyContactState {
            inner: self.inner.state,
        }
    }

    #[getter]
    fn contact_force(&self) -> f64 {
        self.inner.contact_force
    }

    #[setter]
    fn set_contact_force(&mut self, value: f64) {
        self.inner.contact_force = value;
    }

    #[getter]
    fn contact_normal(&self) -> PyVector3 {
        PyVector3 {
            inner: horus_library::messages::geometry::Vector3::new(
                self.inner.contact_normal.x,
                self.inner.contact_normal.y,
                self.inner.contact_normal.z,
            ),
        }
    }

    #[getter]
    fn contact_point(&self) -> PyPoint3 {
        PyPoint3 {
            inner: horus_library::messages::geometry::Point3::new(
                self.inner.contact_point.x,
                self.inner.contact_point.y,
                self.inner.contact_point.z,
            ),
        }
    }

    #[getter]
    fn stiffness(&self) -> f64 {
        self.inner.stiffness
    }

    #[setter]
    fn set_stiffness(&mut self, value: f64) {
        self.inner.stiffness = value;
    }

    #[getter]
    fn damping(&self) -> f64 {
        self.inner.damping
    }

    #[setter]
    fn set_damping(&mut self, value: f64) {
        self.inner.damping = value;
    }

    #[getter]
    fn confidence(&self) -> f32 {
        self.inner.confidence
    }

    #[setter]
    fn set_confidence(&mut self, value: f32) {
        self.inner.confidence = value;
    }

    #[getter]
    fn timestamp(&self) -> u64 {
        self.inner.timestamp
    }

    fn is_in_contact(&self) -> bool {
        self.inner.is_in_contact()
    }

    fn contact_duration_seconds(&self) -> f64 {
        self.inner.contact_duration_seconds()
    }

    fn __repr__(&self) -> String {
        format!(
            "ContactInfo({:?}, force={:.2}N, conf={:.2})",
            self.inner.state, self.inner.contact_force, self.inner.confidence
        )
    }
}

/// Python wrapper for HapticFeedback
#[pyclass(module = "horus.library._library", name = "HapticFeedback")]
#[derive(Clone)]
pub struct PyHapticFeedback {
    pub(crate) inner: force::HapticFeedback,
}

#[pymethods]
impl PyHapticFeedback {
    #[classattr]
    const PATTERN_CONSTANT: u8 = 0;
    #[classattr]
    const PATTERN_PULSE: u8 = 1;
    #[classattr]
    const PATTERN_RAMP: u8 = 2;

    #[new]
    fn new() -> Self {
        Self {
            inner: force::HapticFeedback::default(),
        }
    }

    #[staticmethod]
    fn vibration(intensity: f32, frequency: f32, duration: f32) -> Self {
        Self {
            inner: force::HapticFeedback::vibration(intensity, frequency, duration),
        }
    }

    #[staticmethod]
    fn force(force: &PyVector3, duration: f32) -> Self {
        let f = horus_library::messages::geometry::Vector3::new(
            force.inner.x,
            force.inner.y,
            force.inner.z,
        );
        Self {
            inner: force::HapticFeedback::force(f, duration),
        }
    }

    #[staticmethod]
    fn pulse(intensity: f32, frequency: f32, duration: f32) -> Self {
        Self {
            inner: force::HapticFeedback::pulse(intensity, frequency, duration),
        }
    }

    #[getter]
    fn vibration_intensity(&self) -> f32 {
        self.inner.vibration_intensity
    }

    #[setter]
    fn set_vibration_intensity(&mut self, value: f32) {
        self.inner.vibration_intensity = value;
    }

    #[getter]
    fn vibration_frequency(&self) -> f32 {
        self.inner.vibration_frequency
    }

    #[setter]
    fn set_vibration_frequency(&mut self, value: f32) {
        self.inner.vibration_frequency = value;
    }

    #[getter]
    fn duration_seconds(&self) -> f32 {
        self.inner.duration_seconds
    }

    #[setter]
    fn set_duration_seconds(&mut self, value: f32) {
        self.inner.duration_seconds = value;
    }

    #[getter]
    fn force_feedback(&self) -> PyVector3 {
        PyVector3 {
            inner: horus_library::messages::geometry::Vector3::new(
                self.inner.force_feedback.x,
                self.inner.force_feedback.y,
                self.inner.force_feedback.z,
            ),
        }
    }

    #[getter]
    fn pattern_type(&self) -> u8 {
        self.inner.pattern_type
    }

    #[setter]
    fn set_pattern_type(&mut self, value: u8) {
        self.inner.pattern_type = value;
    }

    #[getter]
    fn enabled(&self) -> bool {
        self.inner.enabled
    }

    #[setter]
    fn set_enabled(&mut self, value: bool) {
        self.inner.enabled = value;
    }

    #[getter]
    fn timestamp(&self) -> u64 {
        self.inner.timestamp
    }

    fn __repr__(&self) -> String {
        format!(
            "HapticFeedback(intensity={:.2}, freq={:.1}Hz, duration={:.2}s)",
            self.inner.vibration_intensity,
            self.inner.vibration_frequency,
            self.inner.duration_seconds
        )
    }
}
