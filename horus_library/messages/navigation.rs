use horus_core::core::LogSummary;
// Navigation and path planning message types for robotics
//
// This module provides messages for autonomous navigation, path planning,
// mapping, and localization systems.

use crate::messages::geometry::{Pose2D, Twist};
use serde::{Deserialize, Serialize};
use serde_arrays;

/// Navigation goal specification
#[derive(Debug, Clone, Copy, Serialize, Deserialize, Default)]
pub struct Goal {
    /// Target pose to reach
    pub target_pose: Pose2D,
    /// Position tolerance in meters
    pub tolerance_position: f64,
    /// Orientation tolerance in radians
    pub tolerance_angle: f64,
    /// Maximum time to reach goal (0 = no limit)
    pub timeout_seconds: f64,
    /// Goal priority (0 = highest)
    pub priority: u8,
    /// Unique goal identifier
    pub goal_id: u32,
    /// Timestamp in nanoseconds since epoch
    pub timestamp_ns: u64,
}

impl Goal {
    /// Create a new navigation goal
    pub fn new(target_pose: Pose2D, position_tolerance: f64, angle_tolerance: f64) -> Self {
        Self {
            target_pose,
            tolerance_position: position_tolerance,
            tolerance_angle: angle_tolerance,
            timeout_seconds: 0.0,
            priority: 1,
            goal_id: 0,
            timestamp_ns: std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap_or_default()
                .as_nanos() as u64,
        }
    }

    /// Create a goal with timeout
    pub fn with_timeout(mut self, timeout_seconds: f64) -> Self {
        self.timeout_seconds = timeout_seconds;
        self
    }

    /// Create a goal with priority
    pub fn with_priority(mut self, priority: u8) -> Self {
        self.priority = priority;
        self
    }

    /// Check if position is within tolerance
    pub fn is_position_reached(&self, current_pose: &Pose2D) -> bool {
        self.target_pose.distance_to(current_pose) <= self.tolerance_position
    }

    /// Check if orientation is within tolerance
    pub fn is_orientation_reached(&self, current_pose: &Pose2D) -> bool {
        let angle_diff = (self.target_pose.theta - current_pose.theta).abs();
        let normalized_diff = if angle_diff > std::f64::consts::PI {
            2.0 * std::f64::consts::PI - angle_diff
        } else {
            angle_diff
        };
        normalized_diff <= self.tolerance_angle
    }

    /// Check if goal is fully reached
    pub fn is_reached(&self, current_pose: &Pose2D) -> bool {
        self.is_position_reached(current_pose) && self.is_orientation_reached(current_pose)
    }
}

/// Goal status enumeration
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[repr(u8)]
#[derive(Default)]
pub enum GoalStatus {
    /// Goal is pending execution
    #[default]
    Pending = 0,
    /// Goal is actively being pursued
    Active = 1,
    /// Goal was successfully reached
    Succeeded = 2,
    /// Goal was aborted due to error
    Aborted = 3,
    /// Goal was cancelled by user
    Cancelled = 4,
    /// Goal was preempted by higher priority goal
    Preempted = 5,
    /// Goal timed out
    TimedOut = 6,
}

/// Goal status feedback
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct GoalResult {
    /// Goal identifier
    pub goal_id: u32,
    /// Current status
    pub status: GoalStatus,
    /// Distance to goal in meters
    pub distance_to_goal: f64,
    /// Estimated time to reach goal in seconds
    pub eta_seconds: f64,
    /// Progress percentage (0.0 to 1.0)
    pub progress: f32,
    /// Error message if failed
    #[serde(with = "serde_arrays")]
    pub error_message: [u8; 64],
    /// Timestamp in nanoseconds since epoch
    pub timestamp_ns: u64,
}

impl Default for GoalResult {
    fn default() -> Self {
        Self {
            goal_id: 0,
            status: GoalStatus::Pending,
            distance_to_goal: 0.0,
            eta_seconds: 0.0,
            progress: 0.0,
            error_message: [0; 64],
            timestamp_ns: 0,
        }
    }
}

impl GoalResult {
    /// Create a new goal result
    pub fn new(goal_id: u32, status: GoalStatus) -> Self {
        Self {
            goal_id,
            status,
            distance_to_goal: 0.0,
            eta_seconds: 0.0,
            progress: 0.0,
            error_message: [0; 64],
            timestamp_ns: std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap_or_default()
                .as_nanos() as u64,
        }
    }

    /// Set error message
    pub fn with_error(mut self, message: &str) -> Self {
        let msg_bytes = message.as_bytes();
        let len = msg_bytes.len().min(63);
        self.error_message[..len].copy_from_slice(&msg_bytes[..len]);
        self.error_message[len] = 0;
        self
    }
}

/// Waypoint in a path
#[derive(Debug, Clone, Copy, Serialize, Deserialize, Default)]
pub struct Waypoint {
    /// Pose at this waypoint
    pub pose: Pose2D,
    /// Desired velocity at this point
    pub velocity: Twist,
    /// Time to reach this waypoint from start
    pub time_from_start: f64,
    /// Curvature at this point (1/radius)
    pub curvature: f32,
    /// Whether to stop at this waypoint
    pub stop_required: bool,
}

impl Waypoint {
    /// Create a new waypoint
    pub fn new(pose: Pose2D) -> Self {
        Self {
            pose,
            velocity: Twist::default(),
            time_from_start: 0.0,
            curvature: 0.0,
            stop_required: false,
        }
    }

    /// Create waypoint with velocity
    pub fn with_velocity(mut self, velocity: Twist) -> Self {
        self.velocity = velocity;
        self
    }

    /// Create waypoint requiring stop
    pub fn with_stop(mut self) -> Self {
        self.stop_required = true;
        self.velocity = Twist::stop();
        self
    }
}

/// Navigation path message
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Path {
    /// Array of waypoints (max 256)
    #[serde(with = "serde_arrays")]
    pub waypoints: [Waypoint; 256],
    /// Number of valid waypoints
    pub waypoint_count: u16,
    /// Total path length in meters
    pub total_length: f64,
    /// Estimated time to complete path
    pub duration_seconds: f64,
    /// Path coordinate frame
    pub frame_id: [u8; 32],
    /// Path generation algorithm used
    pub algorithm: [u8; 32],
    /// Timestamp in nanoseconds since epoch
    pub timestamp_ns: u64,
}

impl Default for Path {
    fn default() -> Self {
        Self {
            waypoints: [Waypoint::default(); 256],
            waypoint_count: 0,
            total_length: 0.0,
            duration_seconds: 0.0,
            frame_id: [0; 32],
            algorithm: [0; 32],
            timestamp_ns: 0,
        }
    }
}

impl Path {
    /// Create a new empty path
    pub fn new() -> Self {
        Self {
            timestamp_ns: std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap_or_default()
                .as_nanos() as u64,
            ..Default::default()
        }
    }

    /// Add a waypoint to the path
    pub fn add_waypoint(&mut self, waypoint: Waypoint) -> Result<(), &'static str> {
        if self.waypoint_count >= 256 {
            return Err("Maximum 256 waypoints supported");
        }

        self.waypoints[self.waypoint_count as usize] = waypoint;
        self.waypoint_count += 1;

        // Update total length
        if self.waypoint_count > 1 {
            let prev = &self.waypoints[(self.waypoint_count - 2) as usize];
            let current = &self.waypoints[(self.waypoint_count - 1) as usize];
            self.total_length += prev.pose.distance_to(&current.pose);
        }

        Ok(())
    }

    /// Get valid waypoints
    pub fn get_waypoints(&self) -> &[Waypoint] {
        &self.waypoints[..self.waypoint_count as usize]
    }

    /// Find closest waypoint to current position
    pub fn closest_waypoint_index(&self, current_pose: &Pose2D) -> Option<usize> {
        if self.waypoint_count == 0 {
            return None;
        }

        let mut min_distance = f64::INFINITY;
        let mut closest_index = 0;

        for (i, waypoint) in self.get_waypoints().iter().enumerate() {
            let distance = current_pose.distance_to(&waypoint.pose);
            if distance < min_distance {
                min_distance = distance;
                closest_index = i;
            }
        }

        Some(closest_index)
    }

    /// Calculate progress along path (0.0 to 1.0)
    pub fn calculate_progress(&self, current_pose: &Pose2D) -> f32 {
        if let Some(index) = self.closest_waypoint_index(current_pose) {
            (index as f32) / (self.waypoint_count as f32).max(1.0)
        } else {
            0.0
        }
    }

    /// Set frame ID
    pub fn with_frame_id(mut self, frame_id: &str) -> Self {
        let frame_bytes = frame_id.as_bytes();
        let len = frame_bytes.len().min(31);
        self.frame_id[..len].copy_from_slice(&frame_bytes[..len]);
        self.frame_id[len] = 0;
        self
    }
}

/// Occupancy grid map
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct OccupancyGrid {
    /// Map resolution (meters per pixel)
    pub resolution: f32,
    /// Map width in pixels
    pub width: u32,
    /// Map height in pixels
    pub height: u32,
    /// Map origin pose (bottom-left corner)
    pub origin: Pose2D,
    /// Map data (-1=unknown, 0=free, 100=occupied)
    pub data: Vec<i8>,
    /// Frame ID for map coordinates
    pub frame_id: [u8; 32],
    /// Map metadata
    #[serde(with = "serde_arrays")]
    pub metadata: [u8; 64],
    /// Timestamp when map was created
    pub timestamp_ns: u64,
}

impl Default for OccupancyGrid {
    fn default() -> Self {
        Self {
            resolution: 0.05, // 5cm default
            width: 0,
            height: 0,
            origin: Pose2D::origin(),
            data: Vec::new(),
            frame_id: [0; 32],
            metadata: [0; 64],
            timestamp_ns: 0,
        }
    }
}

impl OccupancyGrid {
    /// Create a new occupancy grid
    pub fn new(width: u32, height: u32, resolution: f32, origin: Pose2D) -> Self {
        let data_size = (width as usize) * (height as usize);
        Self {
            resolution,
            width,
            height,
            origin,
            data: vec![-1; data_size], // Initialize as unknown
            frame_id: [0; 32],
            metadata: [0; 64],
            timestamp_ns: std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap_or_default()
                .as_nanos() as u64,
        }
    }

    /// Convert world coordinates to grid indices
    pub fn world_to_grid(&self, x: f64, y: f64) -> Option<(u32, u32)> {
        // Add small epsilon to handle floating point precision at cell boundaries
        // This ensures that cell boundary coordinates (like 0.5 with resolution 0.1)
        // map to the correct cell even when FP division gives values like 4.999999
        const EPSILON: f64 = 1e-6;
        let grid_x = ((x - self.origin.x) / self.resolution as f64 + EPSILON).floor() as i32;
        let grid_y = ((y - self.origin.y) / self.resolution as f64 + EPSILON).floor() as i32;

        if grid_x >= 0 && grid_x < self.width as i32 && grid_y >= 0 && grid_y < self.height as i32 {
            Some((grid_x as u32, grid_y as u32))
        } else {
            None
        }
    }

    /// Convert grid indices to world coordinates
    pub fn grid_to_world(&self, grid_x: u32, grid_y: u32) -> Option<(f64, f64)> {
        if grid_x < self.width && grid_y < self.height {
            let x = self.origin.x + (grid_x as f64 + 0.5) * self.resolution as f64;
            let y = self.origin.y + (grid_y as f64 + 0.5) * self.resolution as f64;
            Some((x, y))
        } else {
            None
        }
    }

    /// Get occupancy value at grid coordinates
    pub fn get_occupancy(&self, grid_x: u32, grid_y: u32) -> Option<i8> {
        if grid_x < self.width && grid_y < self.height {
            let index = (grid_y * self.width + grid_x) as usize;
            self.data.get(index).copied()
        } else {
            None
        }
    }

    /// Set occupancy value at grid coordinates
    pub fn set_occupancy(&mut self, grid_x: u32, grid_y: u32, value: i8) -> bool {
        if grid_x < self.width && grid_y < self.height {
            let index = (grid_y * self.width + grid_x) as usize;
            if index < self.data.len() {
                self.data[index] = value.clamp(-1, 100);
                return true;
            }
        }
        false
    }

    /// Check if a point is free (< 50% occupancy)
    pub fn is_free(&self, x: f64, y: f64) -> bool {
        if let Some((gx, gy)) = self.world_to_grid(x, y) {
            if let Some(occupancy) = self.get_occupancy(gx, gy) {
                return (0..50).contains(&occupancy);
            }
        }
        false
    }

    /// Check if a point is occupied (>= 50% occupancy)
    pub fn is_occupied(&self, x: f64, y: f64) -> bool {
        if let Some((gx, gy)) = self.world_to_grid(x, y) {
            if let Some(occupancy) = self.get_occupancy(gx, gy) {
                return occupancy >= 50;
            }
        }
        false
    }
}

/// Cost map for navigation planning
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CostMap {
    /// Base occupancy grid
    pub occupancy_grid: OccupancyGrid,
    /// Cost values (0-255, 255=lethal)
    pub costs: Vec<u8>,
    /// Inflation radius in meters
    pub inflation_radius: f32,
    /// Cost scaling factor
    pub cost_scaling_factor: f32,
    /// Lethal cost threshold
    pub lethal_cost: u8,
}

impl Default for CostMap {
    fn default() -> Self {
        Self {
            occupancy_grid: OccupancyGrid::default(),
            costs: Vec::new(),
            inflation_radius: 0.55,
            cost_scaling_factor: 10.0,
            lethal_cost: 253,
        }
    }
}

impl CostMap {
    /// Create costmap from occupancy grid
    pub fn from_occupancy_grid(grid: OccupancyGrid, inflation_radius: f32) -> Self {
        let data_size = (grid.width * grid.height) as usize;
        let mut costmap = Self {
            occupancy_grid: grid,
            costs: vec![0; data_size],
            inflation_radius,
            ..Default::default()
        };

        costmap.compute_costs();
        costmap
    }

    /// Compute cost values from occupancy data
    pub fn compute_costs(&mut self) {
        let data_size = (self.occupancy_grid.width * self.occupancy_grid.height) as usize;
        self.costs = vec![0; data_size];

        // Convert occupancy to basic costs
        for (i, &occupancy) in self.occupancy_grid.data.iter().enumerate() {
            self.costs[i] = match occupancy {
                -1 => 255,                            // Unknown = lethal
                occ if occ >= 65 => self.lethal_cost, // Occupied = lethal
                occ => (occ * 2).max(0) as u8,        // Free space = low cost
            };
        }

        // Apply obstacle inflation
        self.inflate_obstacles();
    }

    /// Inflate obstacles in the costmap
    /// Uses distance transform to propagate costs around obstacles
    fn inflate_obstacles(&mut self) {
        if self.inflation_radius <= 0.0 {
            return; // No inflation needed
        }

        let width = self.occupancy_grid.width as usize;
        let height = self.occupancy_grid.height as usize;
        let resolution = self.occupancy_grid.resolution;

        // Calculate inflation radius in cells
        let inflation_cells = (self.inflation_radius / resolution).ceil() as i32;

        // Create a copy of current costs to avoid modifying while iterating
        let original_costs = self.costs.clone();

        // For each cell, check if it should be inflated
        for y in 0..height {
            for x in 0..width {
                let center_idx = y * width + x;

                // Skip if already at or above lethal cost (it's an obstacle)
                if original_costs[center_idx] >= self.lethal_cost {
                    continue;
                }

                // Check neighborhood for obstacles
                let mut min_distance = f32::MAX;
                let mut found_obstacle = false;

                // Search within inflation radius
                for dy in -inflation_cells..=inflation_cells {
                    for dx in -inflation_cells..=inflation_cells {
                        let nx = x as i32 + dx;
                        let ny = y as i32 + dy;

                        // Check bounds
                        if nx < 0 || ny < 0 || nx >= width as i32 || ny >= height as i32 {
                            continue;
                        }

                        let neighbor_idx = (ny as usize) * width + (nx as usize);

                        // Check if neighbor is an obstacle
                        if original_costs[neighbor_idx] >= self.lethal_cost {
                            // Calculate Euclidean distance in meters
                            let dist = ((dx * dx + dy * dy) as f32).sqrt() * resolution;

                            if dist < min_distance {
                                min_distance = dist;
                                found_obstacle = true;
                            }
                        }
                    }
                }

                // Apply inflation cost if obstacle found within radius
                if found_obstacle && min_distance <= self.inflation_radius {
                    // Calculate inflated cost using exponential decay
                    // Cost decreases from lethal_cost at obstacle to free space cost at inflation_radius
                    let factor = 1.0 - (min_distance / self.inflation_radius);
                    let inflation_cost = ((self.lethal_cost as f32 - 1.0)
                        * factor.powf(self.cost_scaling_factor))
                    .min(self.lethal_cost as f32 - 1.0);

                    // Keep the higher of the two costs
                    self.costs[center_idx] = self.costs[center_idx].max(inflation_cost as u8);
                }
            }
        }
    }

    /// Get cost at world coordinates
    pub fn get_cost(&self, x: f64, y: f64) -> Option<u8> {
        if let Some((gx, gy)) = self.occupancy_grid.world_to_grid(x, y) {
            let index = (gy * self.occupancy_grid.width + gx) as usize;
            self.costs.get(index).copied()
        } else {
            Some(self.lethal_cost) // Outside map = lethal
        }
    }
}

/// Velocity obstacle for dynamic obstacle avoidance
#[derive(Debug, Clone, Copy, Serialize, Deserialize, Default)]
pub struct VelocityObstacle {
    /// Obstacle position
    pub position: [f64; 2],
    /// Obstacle velocity
    pub velocity: [f64; 2],
    /// Obstacle radius
    pub radius: f32,
    /// Time horizon for collision prediction
    pub time_horizon: f32,
    /// Obstacle ID for tracking
    pub obstacle_id: u32,
}

/// Array of velocity obstacles
#[derive(Debug, Clone, Serialize, Deserialize, Default)]
pub struct VelocityObstacles {
    /// Array of obstacles (max 32)
    #[serde(with = "serde_arrays")]
    pub obstacles: [VelocityObstacle; 32],
    /// Number of valid obstacles
    pub count: u8,
    /// Timestamp in nanoseconds since epoch
    pub timestamp_ns: u64,
}

impl LogSummary for Goal {
    fn log_summary(&self) -> String {
        format!("{:?}", self)
    }
}

impl LogSummary for GoalResult {
    fn log_summary(&self) -> String {
        format!("{:?}", self)
    }
}

impl LogSummary for Waypoint {
    fn log_summary(&self) -> String {
        format!("{:?}", self)
    }
}

impl LogSummary for Path {
    fn log_summary(&self) -> String {
        format!("{:?}", self)
    }
}

impl LogSummary for OccupancyGrid {
    fn log_summary(&self) -> String {
        format!("{:?}", self)
    }
}

impl LogSummary for CostMap {
    fn log_summary(&self) -> String {
        format!("{:?}", self)
    }
}

impl LogSummary for VelocityObstacle {
    fn log_summary(&self) -> String {
        format!("{:?}", self)
    }
}

impl LogSummary for VelocityObstacles {
    fn log_summary(&self) -> String {
        format!("{:?}", self)
    }
}

impl LogSummary for PathPlan {
    fn log_summary(&self) -> String {
        format!("{:?}", self)
    }
}

impl LogSummary for GoalStatus {
    fn log_summary(&self) -> String {
        format!("{:?}", self)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_goal_creation() {
        let target = Pose2D::new(5.0, 3.0, 1.57);
        let goal = Goal::new(target, 0.1, 0.1);

        assert_eq!(goal.target_pose.x, 5.0);
        assert_eq!(goal.tolerance_position, 0.1);
    }

    #[test]
    fn test_goal_reached() {
        let target = Pose2D::new(5.0, 3.0, 0.0);
        let goal = Goal::new(target, 0.2, 0.1);

        let close_pose = Pose2D::new(5.1, 3.05, 0.05);
        assert!(goal.is_reached(&close_pose));

        let far_pose = Pose2D::new(6.0, 3.0, 0.0);
        assert!(!goal.is_reached(&far_pose));
    }

    #[test]
    fn test_path_operations() {
        let mut path = Path::new();
        let wp1 = Waypoint::new(Pose2D::new(0.0, 0.0, 0.0));
        let wp2 = Waypoint::new(Pose2D::new(1.0, 0.0, 0.0));

        path.add_waypoint(wp1).unwrap();
        path.add_waypoint(wp2).unwrap();

        assert_eq!(path.waypoint_count, 2);
        assert_eq!(path.total_length, 1.0);
    }

    #[test]
    fn test_occupancy_grid() {
        let mut grid = OccupancyGrid::new(100, 100, 0.1, Pose2D::origin());

        // Test coordinate conversion
        let (gx, gy) = grid.world_to_grid(0.5, 0.5).unwrap();
        assert_eq!(gx, 5);
        assert_eq!(gy, 5);

        // Test occupancy setting
        assert!(grid.set_occupancy(10, 10, 100));
        assert_eq!(grid.get_occupancy(10, 10), Some(100));

        let (x, y) = grid.grid_to_world(10, 10).unwrap();
        assert!(grid.is_occupied(x, y));
    }
}

/// Simplified path plan message for basic navigation
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PathPlan {
    /// Array of waypoints as [x, y, theta] coordinates
    pub waypoints: Vec<[f32; 3]>,
    /// Goal pose [x, y, theta]
    pub goal_pose: [f32; 3],
    /// Number of waypoints in path
    pub path_length: u32,
    /// Timestamp in nanoseconds since epoch
    pub timestamp_ns: u64,
}

impl Default for PathPlan {
    fn default() -> Self {
        Self {
            waypoints: Vec::new(),
            goal_pose: [0.0, 0.0, 0.0],
            path_length: 0,
            timestamp_ns: 0,
        }
    }
}

impl PathPlan {
    /// Create a new path plan
    pub fn new() -> Self {
        Self {
            timestamp_ns: std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap_or_default()
                .as_nanos() as u64,
            ..Default::default()
        }
    }

    /// Create path plan with waypoints
    pub fn with_waypoints(waypoints: Vec<[f32; 3]>, goal: [f32; 3]) -> Self {
        Self {
            path_length: waypoints.len() as u32,
            waypoints,
            goal_pose: goal,
            timestamp_ns: std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap_or_default()
                .as_nanos() as u64,
        }
    }

    /// Add waypoint to path
    pub fn add_waypoint(&mut self, x: f32, y: f32, theta: f32) {
        self.waypoints.push([x, y, theta]);
        self.path_length = self.waypoints.len() as u32;
    }

    /// Check if path is empty
    pub fn is_empty(&self) -> bool {
        self.waypoints.is_empty()
    }
}
