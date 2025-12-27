// Python wrappers for coordination and fleet management messages
use horus_library::messages::coordination;
use pyo3::prelude::*;

use crate::{PyPose2D, PyTwist};

/// Robot type enum
#[pyclass(module = "horus.library._library", name = "RobotType")]
#[derive(Clone, Copy)]
pub struct PyRobotType {
    pub(crate) inner: coordination::RobotType,
}

#[pymethods]
impl PyRobotType {
    #[classattr]
    const MOBILE: u8 = 0;
    #[classattr]
    const MANIPULATOR: u8 = 1;
    #[classattr]
    const AERIAL: u8 = 2;
    #[classattr]
    const MARINE: u8 = 3;
    #[classattr]
    const STATIONARY: u8 = 4;
    #[classattr]
    const TOOL: u8 = 5;
    #[classattr]
    const TRANSPORT: u8 = 6;
    #[classattr]
    const SERVICE: u8 = 7;

    #[new]
    #[pyo3(signature = (robot_type=0))]
    fn new(robot_type: u8) -> Self {
        let inner = match robot_type {
            0 => coordination::RobotType::Mobile,
            1 => coordination::RobotType::Manipulator,
            2 => coordination::RobotType::Aerial,
            3 => coordination::RobotType::Marine,
            4 => coordination::RobotType::Stationary,
            5 => coordination::RobotType::Tool,
            6 => coordination::RobotType::Transport,
            7 => coordination::RobotType::Service,
            _ => coordination::RobotType::Mobile,
        };
        Self { inner }
    }

    fn __repr__(&self) -> String {
        format!("{:?}", self.inner)
    }
}

/// Coordination mode enum
#[pyclass(module = "horus.library._library", name = "CoordinationMode")]
#[derive(Clone, Copy)]
pub struct PyCoordinationMode {
    pub(crate) inner: coordination::CoordinationMode,
}

#[pymethods]
impl PyCoordinationMode {
    #[classattr]
    const DECENTRALIZED: u8 = 0;
    #[classattr]
    const CENTRALIZED: u8 = 1;
    #[classattr]
    const HIERARCHICAL: u8 = 2;
    #[classattr]
    const MARKET_BASED: u8 = 3;
    #[classattr]
    const SWARM: u8 = 4;

    #[new]
    #[pyo3(signature = (mode=0))]
    fn new(mode: u8) -> Self {
        let inner = match mode {
            0 => coordination::CoordinationMode::Decentralized,
            1 => coordination::CoordinationMode::Centralized,
            2 => coordination::CoordinationMode::Hierarchical,
            3 => coordination::CoordinationMode::MarketBased,
            4 => coordination::CoordinationMode::Swarm,
            _ => coordination::CoordinationMode::Decentralized,
        };
        Self { inner }
    }

    fn __repr__(&self) -> String {
        format!("{:?}", self.inner)
    }
}

/// Task type enum
#[pyclass(module = "horus.library._library", name = "TaskType")]
#[derive(Clone, Copy)]
pub struct PyTaskType {
    pub(crate) inner: coordination::TaskType,
}

#[pymethods]
impl PyTaskType {
    #[classattr]
    const NAVIGATION: u8 = 0;
    #[classattr]
    const MANIPULATION: u8 = 1;
    #[classattr]
    const TRANSPORT: u8 = 2;
    #[classattr]
    const INSPECTION: u8 = 3;
    #[classattr]
    const CLEANING: u8 = 4;
    #[classattr]
    const SURVEILLANCE: u8 = 5;
    #[classattr]
    const EMERGENCY: u8 = 6;
    #[classattr]
    const MAINTENANCE: u8 = 7;
    #[classattr]
    const DATA_COLLECTION: u8 = 8;
    #[classattr]
    const COMMUNICATION: u8 = 9;
    #[classattr]
    const FORMATION: u8 = 10;
    #[classattr]
    const CUSTOM: u8 = 255;

    #[new]
    #[pyo3(signature = (task_type=0))]
    fn new(task_type: u8) -> Self {
        let inner = match task_type {
            0 => coordination::TaskType::Navigation,
            1 => coordination::TaskType::Manipulation,
            2 => coordination::TaskType::Transport,
            3 => coordination::TaskType::Inspection,
            4 => coordination::TaskType::Cleaning,
            5 => coordination::TaskType::Surveillance,
            6 => coordination::TaskType::Emergency,
            7 => coordination::TaskType::Maintenance,
            8 => coordination::TaskType::DataCollection,
            9 => coordination::TaskType::Communication,
            10 => coordination::TaskType::Formation,
            255 => coordination::TaskType::Custom,
            _ => coordination::TaskType::Navigation,
        };
        Self { inner }
    }

    fn __repr__(&self) -> String {
        format!("{:?}", self.inner)
    }
}

/// Task status enum
#[pyclass(module = "horus.library._library", name = "TaskStatus")]
#[derive(Clone, Copy)]
pub struct PyTaskStatus {
    pub(crate) inner: coordination::TaskStatus,
}

#[pymethods]
impl PyTaskStatus {
    #[classattr]
    const ASSIGNED: u8 = 0;
    #[classattr]
    const IN_PROGRESS: u8 = 1;
    #[classattr]
    const COMPLETED: u8 = 2;
    #[classattr]
    const FAILED: u8 = 3;
    #[classattr]
    const CANCELLED: u8 = 4;
    #[classattr]
    const PAUSED: u8 = 5;
    #[classattr]
    const ABORTED: u8 = 6;

    #[new]
    #[pyo3(signature = (status=0))]
    fn new(status: u8) -> Self {
        let inner = match status {
            0 => coordination::TaskStatus::Assigned,
            1 => coordination::TaskStatus::InProgress,
            2 => coordination::TaskStatus::Completed,
            3 => coordination::TaskStatus::Failed,
            4 => coordination::TaskStatus::Cancelled,
            5 => coordination::TaskStatus::Paused,
            6 => coordination::TaskStatus::Aborted,
            _ => coordination::TaskStatus::Assigned,
        };
        Self { inner }
    }

    fn __repr__(&self) -> String {
        format!("{:?}", self.inner)
    }
}

/// Formation type enum
#[pyclass(module = "horus.library._library", name = "FormationType")]
#[derive(Clone, Copy)]
pub struct PyFormationType {
    pub(crate) inner: coordination::FormationType,
}

#[pymethods]
impl PyFormationType {
    #[classattr]
    const LINE: u8 = 0;
    #[classattr]
    const COLUMN: u8 = 1;
    #[classattr]
    const WEDGE: u8 = 2;
    #[classattr]
    const CIRCLE: u8 = 3;
    #[classattr]
    const GRID: u8 = 4;
    #[classattr]
    const DIAMOND: u8 = 5;
    #[classattr]
    const LEADER_FOLLOWER: u8 = 6;
    #[classattr]
    const CUSTOM: u8 = 255;

    #[new]
    #[pyo3(signature = (formation_type=0))]
    fn new(formation_type: u8) -> Self {
        let inner = match formation_type {
            0 => coordination::FormationType::Line,
            1 => coordination::FormationType::Column,
            2 => coordination::FormationType::Wedge,
            3 => coordination::FormationType::Circle,
            4 => coordination::FormationType::Grid,
            5 => coordination::FormationType::Diamond,
            6 => coordination::FormationType::LeaderFollower,
            255 => coordination::FormationType::Custom,
            _ => coordination::FormationType::Line,
        };
        Self { inner }
    }

    fn __repr__(&self) -> String {
        format!("{:?}", self.inner)
    }
}

/// Bid status enum
#[pyclass(module = "horus.library._library", name = "BidStatus")]
#[derive(Clone, Copy)]
pub struct PyBidStatus {
    pub(crate) inner: coordination::BidStatus,
}

#[pymethods]
impl PyBidStatus {
    #[classattr]
    const ACTIVE: u8 = 0;
    #[classattr]
    const WON: u8 = 1;
    #[classattr]
    const LOST: u8 = 2;
    #[classattr]
    const WITHDRAWN: u8 = 3;
    #[classattr]
    const EXPIRED: u8 = 4;

    #[new]
    #[pyo3(signature = (status=0))]
    fn new(status: u8) -> Self {
        let inner = match status {
            0 => coordination::BidStatus::Active,
            1 => coordination::BidStatus::Won,
            2 => coordination::BidStatus::Lost,
            3 => coordination::BidStatus::Withdrawn,
            4 => coordination::BidStatus::Expired,
            _ => coordination::BidStatus::Active,
        };
        Self { inner }
    }

    fn __repr__(&self) -> String {
        format!("{:?}", self.inner)
    }
}

/// Python wrapper for RobotState
#[pyclass(module = "horus.library._library", name = "RobotState")]
#[derive(Clone)]
pub struct PyRobotState {
    pub(crate) inner: coordination::RobotState,
}

#[pymethods]
impl PyRobotState {
    #[new]
    #[pyo3(signature = (robot_id="", robot_type=None))]
    fn new(robot_id: &str, robot_type: Option<&PyRobotType>) -> Self {
        let rt = robot_type
            .map(|t| t.inner)
            .unwrap_or(coordination::RobotType::Mobile);
        Self {
            inner: coordination::RobotState::new(robot_id, rt),
        }
    }

    fn robot_id_str(&self) -> String {
        self.inner.robot_id_str()
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
    fn velocity(&self) -> PyTwist {
        PyTwist {
            inner: self.inner.velocity,
        }
    }

    #[setter]
    fn set_velocity(&mut self, value: &PyTwist) {
        self.inner.velocity = value.inner;
    }

    #[getter]
    fn battery_level(&self) -> f32 {
        self.inner.battery_level
    }

    #[setter]
    fn set_battery_level(&mut self, value: f32) {
        self.inner.battery_level = value;
    }

    #[getter]
    fn current_task_id(&self) -> u32 {
        self.inner.current_task_id
    }

    #[setter]
    fn set_current_task_id(&mut self, value: u32) {
        self.inner.current_task_id = value;
    }

    #[getter]
    fn capabilities(&self) -> u32 {
        self.inner.capabilities
    }

    #[setter]
    fn set_capabilities(&mut self, value: u32) {
        self.inner.capabilities = value;
    }

    #[getter]
    fn load_factor(&self) -> f32 {
        self.inner.load_factor
    }

    #[setter]
    fn set_load_factor(&mut self, value: f32) {
        self.inner.load_factor = value;
    }

    #[getter]
    fn priority(&self) -> u8 {
        self.inner.priority
    }

    #[setter]
    fn set_priority(&mut self, value: u8) {
        self.inner.priority = value;
    }

    #[getter]
    fn comm_quality(&self) -> f32 {
        self.inner.comm_quality
    }

    #[setter]
    fn set_comm_quality(&mut self, value: f32) {
        self.inner.comm_quality = value;
    }

    #[getter]
    fn timestamp(&self) -> u64 {
        self.inner.timestamp
    }

    fn is_available(&self) -> bool {
        self.inner.is_available()
    }

    fn needs_maintenance(&self) -> bool {
        self.inner.needs_maintenance()
    }

    fn update_motion(&mut self, pose: &PyPose2D, velocity: &PyTwist) {
        self.inner.update_motion(pose.inner, velocity.inner);
    }

    fn __repr__(&self) -> String {
        format!(
            "RobotState('{}', {:?}, battery={:.1}%, available={})",
            self.inner.robot_id_str(),
            self.inner.robot_type,
            self.inner.battery_level,
            self.inner.is_available()
        )
    }
}

/// Python wrapper for FleetStatus
#[pyclass(module = "horus.library._library", name = "FleetStatus")]
#[derive(Clone)]
pub struct PyFleetStatus {
    pub(crate) inner: coordination::FleetStatus,
}

#[pymethods]
impl PyFleetStatus {
    #[new]
    #[pyo3(signature = (fleet_id=""))]
    fn new(fleet_id: &str) -> Self {
        Self {
            inner: coordination::FleetStatus::new(fleet_id),
        }
    }

    #[getter]
    fn robot_count(&self) -> u8 {
        self.inner.robot_count
    }

    #[getter]
    fn active_tasks(&self) -> u32 {
        self.inner.active_tasks
    }

    #[setter]
    fn set_active_tasks(&mut self, value: u32) {
        self.inner.active_tasks = value;
    }

    #[getter]
    fn coordination_mode(&self) -> PyCoordinationMode {
        PyCoordinationMode {
            inner: self.inner.coordination_mode,
        }
    }

    #[getter]
    fn emergency_active(&self) -> bool {
        self.inner.emergency_active
    }

    #[getter]
    fn average_battery(&self) -> f32 {
        self.inner.average_battery
    }

    #[getter]
    fn comm_health(&self) -> f32 {
        self.inner.comm_health
    }

    #[getter]
    fn timestamp(&self) -> u64 {
        self.inner.timestamp
    }

    fn update_robot(&mut self, robot_state: &PyRobotState) -> PyResult<()> {
        self.inner
            .update_robot(robot_state.inner)
            .map_err(|e| pyo3::exceptions::PyValueError::new_err(e))
    }

    fn remove_robot(&mut self, robot_id: &str) -> bool {
        self.inner.remove_robot(robot_id)
    }

    fn get_robots(&self) -> Vec<PyRobotState> {
        self.inner
            .get_robots()
            .iter()
            .map(|r| PyRobotState { inner: *r })
            .collect()
    }

    fn available_robots(&self) -> Vec<PyRobotState> {
        self.inner
            .available_robots()
            .into_iter()
            .map(|r| PyRobotState { inner: *r })
            .collect()
    }

    fn maintenance_needed(&self) -> Vec<PyRobotState> {
        self.inner
            .maintenance_needed()
            .into_iter()
            .map(|r| PyRobotState { inner: *r })
            .collect()
    }

    fn __len__(&self) -> usize {
        self.inner.robot_count as usize
    }

    fn __repr__(&self) -> String {
        format!(
            "FleetStatus({} robots, {:?}, avg_battery={:.1}%)",
            self.inner.robot_count, self.inner.coordination_mode, self.inner.average_battery
        )
    }
}

/// Python wrapper for TaskAssignment
#[pyclass(module = "horus.library._library", name = "TaskAssignment")]
#[derive(Clone)]
pub struct PyTaskAssignment {
    pub(crate) inner: coordination::TaskAssignment,
}

#[pymethods]
impl PyTaskAssignment {
    #[new]
    #[pyo3(signature = (task_id=0, robot_id="", task_type=None))]
    fn new(task_id: u32, robot_id: &str, task_type: Option<&PyTaskType>) -> Self {
        let tt = task_type
            .map(|t| t.inner)
            .unwrap_or(coordination::TaskType::Navigation);
        Self {
            inner: coordination::TaskAssignment::new(task_id, robot_id, tt),
        }
    }

    #[getter]
    fn task_id(&self) -> u32 {
        self.inner.task_id
    }

    fn robot_id_str(&self) -> String {
        self.inner.robot_id_str()
    }

    #[getter]
    fn task_type(&self) -> PyTaskType {
        PyTaskType {
            inner: self.inner.task_type,
        }
    }

    #[getter]
    fn priority(&self) -> u8 {
        self.inner.priority
    }

    #[setter]
    fn set_priority(&mut self, value: u8) {
        self.inner.priority = value;
    }

    #[getter]
    fn deadline(&self) -> u64 {
        self.inner.deadline
    }

    #[setter]
    fn set_deadline(&mut self, value: u64) {
        self.inner.deadline = value;
    }

    #[getter]
    fn estimated_duration(&self) -> f64 {
        self.inner.estimated_duration
    }

    #[setter]
    fn set_estimated_duration(&mut self, value: f64) {
        self.inner.estimated_duration = value;
    }

    #[getter]
    fn required_capabilities(&self) -> u32 {
        self.inner.required_capabilities
    }

    #[setter]
    fn set_required_capabilities(&mut self, value: u32) {
        self.inner.required_capabilities = value;
    }

    #[getter]
    fn status(&self) -> PyTaskStatus {
        PyTaskStatus {
            inner: self.inner.status,
        }
    }

    #[getter]
    fn assigned_time(&self) -> u64 {
        self.inner.assigned_time
    }

    #[getter]
    fn expected_completion(&self) -> u64 {
        self.inner.expected_completion
    }

    #[getter]
    fn timestamp(&self) -> u64 {
        self.inner.timestamp
    }

    fn update_status(&mut self, status: &PyTaskStatus) {
        self.inner.update_status(status.inner);
    }

    fn is_overdue(&self) -> bool {
        self.inner.is_overdue()
    }

    fn __repr__(&self) -> String {
        format!(
            "TaskAssignment(id={}, robot='{}', {:?}, {:?})",
            self.inner.task_id,
            self.inner.robot_id_str(),
            self.inner.task_type,
            self.inner.status
        )
    }
}

/// Python wrapper for FormationControl
#[pyclass(module = "horus.library._library", name = "FormationControl")]
#[derive(Clone)]
pub struct PyFormationControl {
    pub(crate) inner: coordination::FormationControl,
}

#[pymethods]
impl PyFormationControl {
    #[new]
    fn new() -> Self {
        Self {
            inner: coordination::FormationControl::default(),
        }
    }

    #[staticmethod]
    fn leader_follower(leader_id: &str, relative_pos: [f64; 2]) -> Self {
        Self {
            inner: coordination::FormationControl::leader_follower(leader_id, relative_pos),
        }
    }

    #[staticmethod]
    fn circle(index: u8, total_robots: u8, radius: f64) -> Self {
        Self {
            inner: coordination::FormationControl::circle(index, total_robots, radius),
        }
    }

    #[getter]
    fn formation_type(&self) -> PyFormationType {
        PyFormationType {
            inner: self.inner.formation_type,
        }
    }

    #[getter]
    fn formation_index(&self) -> u8 {
        self.inner.formation_index
    }

    #[setter]
    fn set_formation_index(&mut self, value: u8) {
        self.inner.formation_index = value;
    }

    #[getter]
    fn relative_position(&self) -> [f64; 2] {
        self.inner.relative_position
    }

    #[setter]
    fn set_relative_position(&mut self, value: [f64; 2]) {
        self.inner.relative_position = value;
    }

    #[getter]
    fn relative_orientation(&self) -> f64 {
        self.inner.relative_orientation
    }

    #[setter]
    fn set_relative_orientation(&mut self, value: f64) {
        self.inner.relative_orientation = value;
    }

    #[getter]
    fn scale(&self) -> f64 {
        self.inner.scale
    }

    #[setter]
    fn set_scale(&mut self, value: f64) {
        self.inner.scale = value;
    }

    #[getter]
    fn spacing(&self) -> f64 {
        self.inner.spacing
    }

    #[setter]
    fn set_spacing(&mut self, value: f64) {
        self.inner.spacing = value;
    }

    #[getter]
    fn stiffness(&self) -> f32 {
        self.inner.stiffness
    }

    #[setter]
    fn set_stiffness(&mut self, value: f32) {
        self.inner.stiffness = value;
    }

    #[getter]
    fn enabled(&self) -> bool {
        self.inner.enabled
    }

    #[getter]
    fn timestamp(&self) -> u64 {
        self.inner.timestamp
    }

    fn set_enabled(&mut self, enabled: bool) {
        self.inner.set_enabled(enabled);
    }

    fn __repr__(&self) -> String {
        format!(
            "FormationControl({:?}, index={}, enabled={})",
            self.inner.formation_type, self.inner.formation_index, self.inner.enabled
        )
    }
}

/// Python wrapper for AuctionBid
#[pyclass(module = "horus.library._library", name = "AuctionBid")]
#[derive(Clone)]
pub struct PyAuctionBid {
    pub(crate) inner: coordination::AuctionBid,
}

#[pymethods]
impl PyAuctionBid {
    #[new]
    #[pyo3(signature = (task_id=0, robot_id="", bid_value=0.0))]
    fn new(task_id: u32, robot_id: &str, bid_value: f64) -> Self {
        Self {
            inner: coordination::AuctionBid::new(task_id, robot_id, bid_value),
        }
    }

    #[getter]
    fn task_id(&self) -> u32 {
        self.inner.task_id
    }

    #[getter]
    fn bid_value(&self) -> f64 {
        self.inner.bid_value
    }

    #[setter]
    fn set_bid_value(&mut self, value: f64) {
        self.inner.bid_value = value;
    }

    #[getter]
    fn estimated_time(&self) -> f64 {
        self.inner.estimated_time
    }

    #[setter]
    fn set_estimated_time(&mut self, value: f64) {
        self.inner.estimated_time = value;
    }

    #[getter]
    fn capability_score(&self) -> f32 {
        self.inner.capability_score
    }

    #[setter]
    fn set_capability_score(&mut self, value: f32) {
        self.inner.capability_score = value;
    }

    #[getter]
    fn availability(&self) -> f32 {
        self.inner.availability
    }

    #[setter]
    fn set_availability(&mut self, value: f32) {
        self.inner.availability = value;
    }

    #[getter]
    fn bid_time(&self) -> u64 {
        self.inner.bid_time
    }

    #[getter]
    fn expiration_time(&self) -> u64 {
        self.inner.expiration_time
    }

    #[setter]
    fn set_expiration_time(&mut self, value: u64) {
        self.inner.expiration_time = value;
    }

    #[getter]
    fn status(&self) -> PyBidStatus {
        PyBidStatus {
            inner: self.inner.status,
        }
    }

    #[getter]
    fn timestamp(&self) -> u64 {
        self.inner.timestamp
    }

    fn is_valid(&self) -> bool {
        self.inner.is_valid()
    }

    fn total_score(&self) -> f64 {
        self.inner.total_score()
    }

    fn __repr__(&self) -> String {
        format!(
            "AuctionBid(task={}, bid={:.2}, {:?})",
            self.inner.task_id, self.inner.bid_value, self.inner.status
        )
    }
}
