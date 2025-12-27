// Python wrappers for navigation messages
use horus_library::messages::navigation;
use pyo3::prelude::*;

use crate::PyPose2D;

/// Goal status enum
#[pyclass(module = "horus.library._library", name = "GoalStatus")]
#[derive(Clone, Copy)]
pub struct PyGoalStatus {
    pub(crate) inner: navigation::GoalStatus,
}

#[pymethods]
impl PyGoalStatus {
    #[classattr]
    const PENDING: u8 = 0;
    #[classattr]
    const ACTIVE: u8 = 1;
    #[classattr]
    const SUCCEEDED: u8 = 2;
    #[classattr]
    const ABORTED: u8 = 3;
    #[classattr]
    const CANCELLED: u8 = 4;
    #[classattr]
    const PREEMPTED: u8 = 5;
    #[classattr]
    const TIMED_OUT: u8 = 6;

    #[new]
    #[pyo3(signature = (status=0))]
    fn new(status: u8) -> Self {
        let inner = match status {
            0 => navigation::GoalStatus::Pending,
            1 => navigation::GoalStatus::Active,
            2 => navigation::GoalStatus::Succeeded,
            3 => navigation::GoalStatus::Aborted,
            4 => navigation::GoalStatus::Cancelled,
            5 => navigation::GoalStatus::Preempted,
            6 => navigation::GoalStatus::TimedOut,
            _ => navigation::GoalStatus::Pending,
        };
        Self { inner }
    }

    fn __repr__(&self) -> String {
        format!("{:?}", self.inner)
    }
}

/// Python wrapper for Goal
#[pyclass(module = "horus.library._library", name = "Goal")]
#[derive(Clone)]
pub struct PyGoal {
    pub(crate) inner: navigation::Goal,
}

#[pymethods]
impl PyGoal {
    #[new]
    #[pyo3(signature = (target_pose=None, position_tolerance=0.1, angle_tolerance=0.1))]
    fn new(target_pose: Option<&PyPose2D>, position_tolerance: f64, angle_tolerance: f64) -> Self {
        let pose = target_pose.map(|p| p.inner).unwrap_or_default();
        Self {
            inner: navigation::Goal::new(pose, position_tolerance, angle_tolerance),
        }
    }

    #[getter]
    fn target_pose(&self) -> PyPose2D {
        PyPose2D {
            inner: self.inner.target_pose,
        }
    }

    #[setter]
    fn set_target_pose(&mut self, value: &PyPose2D) {
        self.inner.target_pose = value.inner;
    }

    #[getter]
    fn tolerance_position(&self) -> f64 {
        self.inner.tolerance_position
    }

    #[setter]
    fn set_tolerance_position(&mut self, value: f64) {
        self.inner.tolerance_position = value;
    }

    #[getter]
    fn tolerance_angle(&self) -> f64 {
        self.inner.tolerance_angle
    }

    #[setter]
    fn set_tolerance_angle(&mut self, value: f64) {
        self.inner.tolerance_angle = value;
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
    fn priority(&self) -> u8 {
        self.inner.priority
    }

    #[setter]
    fn set_priority(&mut self, value: u8) {
        self.inner.priority = value;
    }

    #[getter]
    fn goal_id(&self) -> u32 {
        self.inner.goal_id
    }

    #[setter]
    fn set_goal_id(&mut self, value: u32) {
        self.inner.goal_id = value;
    }

    #[getter]
    fn timestamp(&self) -> u64 {
        self.inner.timestamp
    }

    fn is_position_reached(&self, current_pose: &PyPose2D) -> bool {
        self.inner.is_position_reached(&current_pose.inner)
    }

    fn is_orientation_reached(&self, current_pose: &PyPose2D) -> bool {
        self.inner.is_orientation_reached(&current_pose.inner)
    }

    fn is_reached(&self, current_pose: &PyPose2D) -> bool {
        self.inner.is_reached(&current_pose.inner)
    }

    fn __repr__(&self) -> String {
        format!(
            "Goal(target=({:.2}, {:.2}, {:.2}), tol_pos={:.2}, tol_angle={:.2})",
            self.inner.target_pose.x,
            self.inner.target_pose.y,
            self.inner.target_pose.theta,
            self.inner.tolerance_position,
            self.inner.tolerance_angle
        )
    }
}

/// Python wrapper for GoalResult
#[pyclass(module = "horus.library._library", name = "GoalResult")]
#[derive(Clone)]
pub struct PyGoalResult {
    pub(crate) inner: navigation::GoalResult,
}

#[pymethods]
impl PyGoalResult {
    #[new]
    #[pyo3(signature = (goal_id=0, status=None))]
    fn new(goal_id: u32, status: Option<&PyGoalStatus>) -> Self {
        let goal_status = status
            .map(|s| s.inner)
            .unwrap_or(navigation::GoalStatus::Pending);
        Self {
            inner: navigation::GoalResult::new(goal_id, goal_status),
        }
    }

    #[getter]
    fn goal_id(&self) -> u32 {
        self.inner.goal_id
    }

    #[getter]
    fn status(&self) -> PyGoalStatus {
        PyGoalStatus {
            inner: self.inner.status,
        }
    }

    #[getter]
    fn distance_to_goal(&self) -> f64 {
        self.inner.distance_to_goal
    }

    #[setter]
    fn set_distance_to_goal(&mut self, value: f64) {
        self.inner.distance_to_goal = value;
    }

    #[getter]
    fn eta_seconds(&self) -> f64 {
        self.inner.eta_seconds
    }

    #[setter]
    fn set_eta_seconds(&mut self, value: f64) {
        self.inner.eta_seconds = value;
    }

    #[getter]
    fn progress(&self) -> f32 {
        self.inner.progress
    }

    #[setter]
    fn set_progress(&mut self, value: f32) {
        self.inner.progress = value;
    }

    #[getter]
    fn timestamp(&self) -> u64 {
        self.inner.timestamp
    }

    fn __repr__(&self) -> String {
        format!(
            "GoalResult(id={}, status={:?}, progress={:.1}%)",
            self.inner.goal_id,
            self.inner.status,
            self.inner.progress * 100.0
        )
    }
}

/// Python wrapper for Waypoint
#[pyclass(module = "horus.library._library", name = "Waypoint")]
#[derive(Clone)]
pub struct PyWaypoint {
    pub(crate) inner: navigation::Waypoint,
}

#[pymethods]
impl PyWaypoint {
    #[new]
    #[pyo3(signature = (pose=None))]
    fn new(pose: Option<&PyPose2D>) -> Self {
        let p = pose.map(|p| p.inner).unwrap_or_default();
        Self {
            inner: navigation::Waypoint::new(p),
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
    fn time_from_start(&self) -> f64 {
        self.inner.time_from_start
    }

    #[setter]
    fn set_time_from_start(&mut self, value: f64) {
        self.inner.time_from_start = value;
    }

    #[getter]
    fn curvature(&self) -> f32 {
        self.inner.curvature
    }

    #[setter]
    fn set_curvature(&mut self, value: f32) {
        self.inner.curvature = value;
    }

    #[getter]
    fn stop_required(&self) -> bool {
        self.inner.stop_required
    }

    #[setter]
    fn set_stop_required(&mut self, value: bool) {
        self.inner.stop_required = value;
    }

    fn __repr__(&self) -> String {
        format!(
            "Waypoint(pose=({:.2}, {:.2}, {:.2}), stop={})",
            self.inner.pose.x, self.inner.pose.y, self.inner.pose.theta, self.inner.stop_required
        )
    }
}

/// Python wrapper for Path
#[pyclass(module = "horus.library._library", name = "Path")]
#[derive(Clone)]
pub struct PyPath {
    pub(crate) inner: navigation::Path,
}

#[pymethods]
impl PyPath {
    #[new]
    fn new() -> Self {
        Self {
            inner: navigation::Path::new(),
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

    #[getter]
    fn duration_seconds(&self) -> f64 {
        self.inner.duration_seconds
    }

    #[setter]
    fn set_duration_seconds(&mut self, value: f64) {
        self.inner.duration_seconds = value;
    }

    #[getter]
    fn timestamp(&self) -> u64 {
        self.inner.timestamp
    }

    fn add_waypoint(&mut self, waypoint: &PyWaypoint) -> PyResult<()> {
        self.inner
            .add_waypoint(waypoint.inner)
            .map_err(|e| pyo3::exceptions::PyValueError::new_err(e))
    }

    fn get_waypoints(&self) -> Vec<PyWaypoint> {
        self.inner
            .get_waypoints()
            .iter()
            .map(|w| PyWaypoint { inner: *w })
            .collect()
    }

    fn closest_waypoint_index(&self, current_pose: &PyPose2D) -> Option<usize> {
        self.inner.closest_waypoint_index(&current_pose.inner)
    }

    fn calculate_progress(&self, current_pose: &PyPose2D) -> f32 {
        self.inner.calculate_progress(&current_pose.inner)
    }

    fn __len__(&self) -> usize {
        self.inner.waypoint_count as usize
    }

    fn __repr__(&self) -> String {
        format!(
            "Path({} waypoints, length={:.2}m, duration={:.1}s)",
            self.inner.waypoint_count, self.inner.total_length, self.inner.duration_seconds
        )
    }
}

/// Python wrapper for OccupancyGrid
#[pyclass(module = "horus.library._library", name = "OccupancyGrid")]
#[derive(Clone)]
pub struct PyOccupancyGrid {
    pub(crate) inner: navigation::OccupancyGrid,
}

#[pymethods]
impl PyOccupancyGrid {
    #[new]
    #[pyo3(signature = (width=100, height=100, resolution=0.05, origin=None))]
    fn new(width: u32, height: u32, resolution: f32, origin: Option<&PyPose2D>) -> Self {
        let org = origin.map(|p| p.inner).unwrap_or_default();
        Self {
            inner: navigation::OccupancyGrid::new(width, height, resolution, org),
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

    #[getter]
    fn origin(&self) -> PyPose2D {
        PyPose2D {
            inner: self.inner.origin,
        }
    }

    #[getter]
    fn data(&self) -> Vec<i8> {
        self.inner.data.clone()
    }

    #[setter]
    fn set_data(&mut self, value: Vec<i8>) {
        self.inner.data = value;
    }

    #[getter]
    fn timestamp(&self) -> u64 {
        self.inner.timestamp
    }

    fn world_to_grid(&self, x: f64, y: f64) -> Option<(u32, u32)> {
        self.inner.world_to_grid(x, y)
    }

    fn grid_to_world(&self, grid_x: u32, grid_y: u32) -> Option<(f64, f64)> {
        self.inner.grid_to_world(grid_x, grid_y)
    }

    fn get_occupancy(&self, grid_x: u32, grid_y: u32) -> Option<i8> {
        self.inner.get_occupancy(grid_x, grid_y)
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

    fn __repr__(&self) -> String {
        format!(
            "OccupancyGrid({}x{}, resolution={:.3}m)",
            self.inner.width, self.inner.height, self.inner.resolution
        )
    }
}

/// Python wrapper for CostMap
#[pyclass(module = "horus.library._library", name = "CostMap")]
#[derive(Clone)]
pub struct PyCostMap {
    pub(crate) inner: navigation::CostMap,
}

#[pymethods]
impl PyCostMap {
    #[new]
    fn new() -> Self {
        Self {
            inner: navigation::CostMap::default(),
        }
    }

    #[staticmethod]
    fn from_occupancy_grid(grid: &PyOccupancyGrid, inflation_radius: f32) -> Self {
        Self {
            inner: navigation::CostMap::from_occupancy_grid(grid.inner.clone(), inflation_radius),
        }
    }

    #[getter]
    fn inflation_radius(&self) -> f32 {
        self.inner.inflation_radius
    }

    #[setter]
    fn set_inflation_radius(&mut self, value: f32) {
        self.inner.inflation_radius = value;
    }

    #[getter]
    fn cost_scaling_factor(&self) -> f32 {
        self.inner.cost_scaling_factor
    }

    #[setter]
    fn set_cost_scaling_factor(&mut self, value: f32) {
        self.inner.cost_scaling_factor = value;
    }

    #[getter]
    fn lethal_cost(&self) -> u8 {
        self.inner.lethal_cost
    }

    #[getter]
    fn costs(&self) -> Vec<u8> {
        self.inner.costs.clone()
    }

    fn get_cost(&self, x: f64, y: f64) -> Option<u8> {
        self.inner.get_cost(x, y)
    }

    fn compute_costs(&mut self) {
        self.inner.compute_costs();
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

/// Python wrapper for VelocityObstacle
#[pyclass(module = "horus.library._library", name = "VelocityObstacle")]
#[derive(Clone, Copy)]
pub struct PyVelocityObstacle {
    pub(crate) inner: navigation::VelocityObstacle,
}

#[pymethods]
impl PyVelocityObstacle {
    #[new]
    fn new() -> Self {
        Self {
            inner: navigation::VelocityObstacle::default(),
        }
    }

    #[getter]
    fn position(&self) -> [f64; 2] {
        self.inner.position
    }

    #[setter]
    fn set_position(&mut self, value: [f64; 2]) {
        self.inner.position = value;
    }

    #[getter]
    fn velocity(&self) -> [f64; 2] {
        self.inner.velocity
    }

    #[setter]
    fn set_velocity(&mut self, value: [f64; 2]) {
        self.inner.velocity = value;
    }

    #[getter]
    fn radius(&self) -> f32 {
        self.inner.radius
    }

    #[setter]
    fn set_radius(&mut self, value: f32) {
        self.inner.radius = value;
    }

    #[getter]
    fn time_horizon(&self) -> f32 {
        self.inner.time_horizon
    }

    #[setter]
    fn set_time_horizon(&mut self, value: f32) {
        self.inner.time_horizon = value;
    }

    #[getter]
    fn obstacle_id(&self) -> u32 {
        self.inner.obstacle_id
    }

    #[setter]
    fn set_obstacle_id(&mut self, value: u32) {
        self.inner.obstacle_id = value;
    }

    fn __repr__(&self) -> String {
        format!(
            "VelocityObstacle(pos=[{:.2}, {:.2}], vel=[{:.2}, {:.2}], r={:.2})",
            self.inner.position[0],
            self.inner.position[1],
            self.inner.velocity[0],
            self.inner.velocity[1],
            self.inner.radius
        )
    }
}

/// Python wrapper for VelocityObstacles
#[pyclass(module = "horus.library._library", name = "VelocityObstacles")]
#[derive(Clone)]
pub struct PyVelocityObstacles {
    pub(crate) inner: navigation::VelocityObstacles,
}

#[pymethods]
impl PyVelocityObstacles {
    #[new]
    fn new() -> Self {
        Self {
            inner: navigation::VelocityObstacles::default(),
        }
    }

    #[getter]
    fn count(&self) -> u8 {
        self.inner.count
    }

    #[getter]
    fn timestamp(&self) -> u64 {
        self.inner.timestamp
    }

    fn get_obstacles(&self) -> Vec<PyVelocityObstacle> {
        self.inner.obstacles[..self.inner.count as usize]
            .iter()
            .map(|o| PyVelocityObstacle { inner: *o })
            .collect()
    }

    fn __len__(&self) -> usize {
        self.inner.count as usize
    }

    fn __repr__(&self) -> String {
        format!("VelocityObstacles({} obstacles)", self.inner.count)
    }
}

/// Python wrapper for PathPlan
#[pyclass(module = "horus.library._library", name = "PathPlan")]
#[derive(Clone)]
pub struct PyPathPlan {
    pub(crate) inner: navigation::PathPlan,
}

#[pymethods]
impl PyPathPlan {
    #[new]
    fn new() -> Self {
        Self {
            inner: navigation::PathPlan::new(),
        }
    }

    #[staticmethod]
    fn with_waypoints(waypoints: Vec<[f32; 3]>, goal: [f32; 3]) -> Self {
        Self {
            inner: navigation::PathPlan::with_waypoints(waypoints, goal),
        }
    }

    #[getter]
    fn waypoints(&self) -> Vec<[f32; 3]> {
        self.inner.waypoints.clone()
    }

    #[setter]
    fn set_waypoints(&mut self, value: Vec<[f32; 3]>) {
        self.inner.waypoints = value;
        self.inner.path_length = self.inner.waypoints.len() as u32;
    }

    #[getter]
    fn goal_pose(&self) -> [f32; 3] {
        self.inner.goal_pose
    }

    #[setter]
    fn set_goal_pose(&mut self, value: [f32; 3]) {
        self.inner.goal_pose = value;
    }

    #[getter]
    fn path_length(&self) -> u32 {
        self.inner.path_length
    }

    #[getter]
    fn timestamp(&self) -> u64 {
        self.inner.timestamp
    }

    fn add_waypoint(&mut self, x: f32, y: f32, theta: f32) {
        self.inner.add_waypoint(x, y, theta);
    }

    fn is_empty(&self) -> bool {
        self.inner.is_empty()
    }

    fn __len__(&self) -> usize {
        self.inner.waypoints.len()
    }

    fn __repr__(&self) -> String {
        format!(
            "PathPlan({} waypoints, goal=[{:.2}, {:.2}, {:.2}])",
            self.inner.path_length,
            self.inner.goal_pose[0],
            self.inner.goal_pose[1],
            self.inner.goal_pose[2]
        )
    }
}
