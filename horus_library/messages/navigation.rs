use horus_macros::LogSummary;
// Navigation and path planning message types for robotics
//
// This module provides messages for autonomous navigation, path planning,
// mapping, and localization systems.

use crate::messages::geometry::{Pose2D, Twist};
use serde::{Deserialize, Serialize};
use serde_arrays;

/// Navigation goal specification
#[repr(C)]
#[derive(Debug, Clone, Copy, Serialize, Deserialize, Default, LogSummary)]
pub struct NavGoal {
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

impl NavGoal {
    /// Create a new navigation goal
    pub fn new(target_pose: Pose2D, position_tolerance: f64, angle_tolerance: f64) -> Self {
        Self {
            target_pose,
            tolerance_position: position_tolerance,
            tolerance_angle: angle_tolerance,
            timeout_seconds: 0.0,
            priority: 1,
            goal_id: 0,
            timestamp_ns: crate::transform_frame::timestamp_now(),
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
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize, LogSummary)]
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
#[repr(C)]
#[derive(Debug, Clone, Copy, Serialize, Deserialize, LogSummary)]
pub struct GoalResult {
    /// Goal identifier
    pub goal_id: u32,
    /// Current status
    pub status: u8,
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
            status: GoalStatus::Pending as u8,
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
            status: status as u8,
            distance_to_goal: 0.0,
            eta_seconds: 0.0,
            progress: 0.0,
            error_message: [0; 64],
            timestamp_ns: crate::transform_frame::timestamp_now(),
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
#[repr(C)]
#[derive(Debug, Clone, Copy, Serialize, Deserialize, Default, LogSummary)]
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
    pub stop_required: u8,
}

impl Waypoint {
    /// Create a new waypoint
    pub fn new(pose: Pose2D) -> Self {
        Self {
            pose,
            velocity: Twist::default(),
            time_from_start: 0.0,
            curvature: 0.0,
            stop_required: 0,
        }
    }

    /// Create waypoint with velocity
    pub fn with_velocity(mut self, velocity: Twist) -> Self {
        self.velocity = velocity;
        self
    }

    /// Create waypoint requiring stop
    pub fn with_stop(mut self) -> Self {
        self.stop_required = 1;
        self.velocity = Twist::stop();
        self
    }
}

/// Navigation path message
#[repr(C)]
#[derive(Debug, Clone, Copy, Serialize, Deserialize, LogSummary)]
pub struct NavPath {
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

impl Default for NavPath {
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

impl NavPath {
    /// Create a new empty path
    pub fn new() -> Self {
        Self {
            timestamp_ns: crate::transform_frame::timestamp_now(),
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
    pub fn waypoints(&self) -> &[Waypoint] {
        &self.waypoints[..self.waypoint_count as usize]
    }

    /// Find closest waypoint to current position
    pub fn closest_waypoint_index(&self, current_pose: &Pose2D) -> Option<usize> {
        if self.waypoint_count == 0 {
            return None;
        }

        let mut min_distance = f64::INFINITY;
        let mut closest_index = 0;

        for (i, waypoint) in self.waypoints().iter().enumerate() {
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

    crate::messages::impl_with_frame_id!();
}

/// Occupancy grid map
#[derive(Debug, Clone, Serialize, Deserialize, LogSummary)]
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
            timestamp_ns: crate::transform_frame::timestamp_now(),
        }
    }

    /// Convert world coordinates to grid indices
    pub fn world_to_grid(&self, x: f64, y: f64) -> Option<(u32, u32)> {
        // Add small epsilon to handle floating point precision at cell boundaries
        // This ensures that cell boundary coordinates (like 0.5 with resolution 0.1)
        // map to the correct cell even when FP division gives values like 4.999999
        const EPSILON: f64 = 1e-6;
        let res = (self.resolution as f64).max(1e-9);
        let grid_x = ((x - self.origin.x) / res + EPSILON).floor() as i32;
        let grid_y = ((y - self.origin.y) / res + EPSILON).floor() as i32;

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
    pub fn occupancy(&self, grid_x: u32, grid_y: u32) -> Option<i8> {
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
            if let Some(occupancy) = self.occupancy(gx, gy) {
                return (0..50).contains(&occupancy);
            }
        }
        false
    }

    /// Check if a point is occupied (>= 50% occupancy)
    pub fn is_occupied(&self, x: f64, y: f64) -> bool {
        if let Some((gx, gy)) = self.world_to_grid(x, y) {
            if let Some(occupancy) = self.occupancy(gx, gy) {
                return occupancy >= 50;
            }
        }
        false
    }
}

/// Cost map for navigation planning
#[derive(Debug, Clone, Serialize, Deserialize, LogSummary)]
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
        let safe_resolution = resolution.max(1e-9);
        let inflation_cells = (self.inflation_radius / safe_resolution).ceil() as i32;

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
                    let factor = 1.0 - (min_distance / self.inflation_radius.max(1e-9));
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
    pub fn cost(&self, x: f64, y: f64) -> Option<u8> {
        if let Some((gx, gy)) = self.occupancy_grid.world_to_grid(x, y) {
            let index = (gy * self.occupancy_grid.width + gx) as usize;
            self.costs.get(index).copied()
        } else {
            Some(self.lethal_cost) // Outside map = lethal
        }
    }
}

/// Velocity obstacle for dynamic obstacle avoidance
#[repr(C)]
#[derive(Debug, Clone, Copy, Serialize, Deserialize, Default, LogSummary)]
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
#[repr(C)]
#[derive(Debug, Clone, Copy, Serialize, Deserialize, Default, LogSummary)]
pub struct VelocityObstacles {
    /// Array of obstacles (max 32)
    #[serde(with = "serde_arrays")]
    pub obstacles: [VelocityObstacle; 32],
    /// Number of valid obstacles
    pub count: u8,
    /// Timestamp in nanoseconds since epoch
    pub timestamp_ns: u64,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_goal_creation() {
        let target = Pose2D::new(5.0, 3.0, 1.57);
        let goal = NavGoal::new(target, 0.1, 0.1);

        assert_eq!(goal.target_pose.x, 5.0);
        assert_eq!(goal.tolerance_position, 0.1);
    }

    #[test]
    fn test_goal_reached() {
        let target = Pose2D::new(5.0, 3.0, 0.0);
        let goal = NavGoal::new(target, 0.2, 0.1);

        let close_pose = Pose2D::new(5.1, 3.05, 0.05);
        assert!(goal.is_reached(&close_pose));

        let far_pose = Pose2D::new(6.0, 3.0, 0.0);
        assert!(!goal.is_reached(&far_pose));
    }

    #[test]
    fn test_path_operations() {
        let mut path = NavPath::new();
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
        assert_eq!(grid.occupancy(10, 10), Some(100));

        let (x, y) = grid.grid_to_world(10, 10).unwrap();
        assert!(grid.is_occupied(x, y));
    }

    // ========================================================================
    // Navigation message edge case tests
    // ========================================================================

    #[test]
    fn test_occupancy_grid_zero_size() {
        let grid = OccupancyGrid::new(0, 0, 0.1, Pose2D::origin());
        assert_eq!(grid.data.len(), 0);
        assert_eq!(grid.occupancy(0, 0), None);
    }

    #[test]
    fn test_occupancy_grid_out_of_bounds() {
        let grid = OccupancyGrid::new(10, 10, 0.1, Pose2D::origin());
        assert_eq!(grid.occupancy(10, 10), None);
        assert_eq!(grid.occupancy(100, 100), None);
    }

    #[test]
    fn test_occupancy_grid_world_to_grid_outside() {
        let grid = OccupancyGrid::new(10, 10, 0.1, Pose2D::origin());
        assert_eq!(grid.world_to_grid(-1.0, -1.0), None);
        assert_eq!(grid.world_to_grid(100.0, 100.0), None);
    }

    #[test]
    fn test_occupancy_grid_clamp_values() {
        let mut grid = OccupancyGrid::new(5, 5, 0.1, Pose2D::origin());
        // Set occupancy should clamp to [-1, 100]
        assert!(grid.set_occupancy(0, 0, 127));
        assert_eq!(grid.occupancy(0, 0), Some(100)); // Clamped to 100
        assert!(grid.set_occupancy(0, 0, -2));
        assert_eq!(grid.occupancy(0, 0), Some(-1)); // Clamped to -1
    }

    #[test]
    fn test_path_plan_empty() {
        let plan = PathPlan::new();
        assert!(plan.is_empty());
        assert_eq!(plan.waypoint_count, 0);
        assert_eq!(plan.waypoint(0), None);
    }

    #[test]
    fn test_path_plan_add_max_waypoints() {
        let mut plan = PathPlan::new();
        // Add 256 waypoints (max)
        for i in 0..256 {
            assert!(plan.add_waypoint(i as f32, 0.0, 0.0));
        }
        // 257th should fail
        assert!(!plan.add_waypoint(256.0, 0.0, 0.0));
        assert_eq!(plan.waypoint_count, 256);
    }

    #[test]
    fn test_nav_goal_reached() {
        let target = Pose2D::new(5.0, 3.0, 0.0);
        let goal = NavGoal::new(target, 0.2, 0.1);
        let close = Pose2D::new(5.1, 3.05, 0.05);
        assert!(goal.is_reached(&close));
        let far = Pose2D::new(6.0, 3.0, 0.0);
        assert!(!goal.is_reached(&far));
    }

    #[test]
    fn test_costmap_default_not_empty() {
        let costmap = CostMap::default();
        assert_eq!(costmap.inflation_radius, 0.55);
        assert_eq!(costmap.lethal_cost, 253);
    }

    // ============================================================================
    // NavPath (many waypoints) Tests
    // ============================================================================

    #[test]
    fn test_nav_path_fill_256_waypoints() {
        let mut path = NavPath::new();
        for i in 0..256 {
            let wp = Waypoint::new(Pose2D::new(i as f64, 0.0, 0.0));
            path.add_waypoint(wp).unwrap();
        }
        assert_eq!(path.waypoint_count, 256);
        assert_eq!(path.waypoints().len(), 256);
    }

    #[test]
    fn test_nav_path_257th_waypoint_fails() {
        let mut path = NavPath::new();
        for i in 0..256 {
            path.add_waypoint(Waypoint::new(Pose2D::new(i as f64, 0.0, 0.0)))
                .unwrap();
        }
        let result = path.add_waypoint(Waypoint::new(Pose2D::new(256.0, 0.0, 0.0)));
        assert!(result.is_err());
        assert_eq!(result.unwrap_err(), "Maximum 256 waypoints supported");
    }

    #[test]
    fn test_nav_path_total_length_accumulates() {
        let mut path = NavPath::new();
        // Three waypoints forming a right angle: (0,0) -> (3,0) -> (3,4)
        path.add_waypoint(Waypoint::new(Pose2D::new(0.0, 0.0, 0.0)))
            .unwrap();
        path.add_waypoint(Waypoint::new(Pose2D::new(3.0, 0.0, 0.0)))
            .unwrap();
        path.add_waypoint(Waypoint::new(Pose2D::new(3.0, 4.0, 0.0)))
            .unwrap();
        // Total should be 3 + 4 = 7
        assert!((path.total_length - 7.0).abs() < 1e-10);
    }

    #[test]
    fn test_nav_path_single_waypoint_zero_length() {
        let mut path = NavPath::new();
        path.add_waypoint(Waypoint::new(Pose2D::new(5.0, 5.0, 0.0)))
            .unwrap();
        assert_eq!(path.waypoint_count, 1);
        assert!((path.total_length - 0.0).abs() < 1e-10);
    }

    #[test]
    fn test_nav_path_closest_waypoint_empty() {
        let path = NavPath::new();
        let pose = Pose2D::new(1.0, 1.0, 0.0);
        assert!(path.closest_waypoint_index(&pose).is_none());
    }

    #[test]
    fn test_nav_path_closest_waypoint_finds_nearest() {
        let mut path = NavPath::new();
        path.add_waypoint(Waypoint::new(Pose2D::new(0.0, 0.0, 0.0)))
            .unwrap();
        path.add_waypoint(Waypoint::new(Pose2D::new(10.0, 0.0, 0.0)))
            .unwrap();
        path.add_waypoint(Waypoint::new(Pose2D::new(20.0, 0.0, 0.0)))
            .unwrap();

        // Closest to (9, 0) should be waypoint 1 at (10, 0)
        let idx = path.closest_waypoint_index(&Pose2D::new(9.0, 0.0, 0.0));
        assert_eq!(idx, Some(1));

        // Closest to (21, 0) should be waypoint 2 at (20, 0)
        let idx2 = path.closest_waypoint_index(&Pose2D::new(21.0, 0.0, 0.0));
        assert_eq!(idx2, Some(2));
    }

    #[test]
    fn test_nav_path_calculate_progress() {
        let mut path = NavPath::new();
        for i in 0..10 {
            path.add_waypoint(Waypoint::new(Pose2D::new(i as f64 * 10.0, 0.0, 0.0)))
                .unwrap();
        }
        // Standing at roughly the 5th waypoint (50, 0)
        let progress = path.calculate_progress(&Pose2D::new(50.0, 0.0, 0.0));
        // Closest is index 5, progress = 5/10 = 0.5
        assert!((progress - 0.5).abs() < 1e-6);
    }

    #[test]
    fn test_nav_path_calculate_progress_empty() {
        let path = NavPath::new();
        let progress = path.calculate_progress(&Pose2D::new(0.0, 0.0, 0.0));
        assert!((progress - 0.0).abs() < 1e-6);
    }

    #[test]
    fn test_nav_path_with_frame_id() {
        let path = NavPath::new().with_frame_id("map");
        let frame_str = std::str::from_utf8(
            &path.frame_id[..path.frame_id.iter().position(|&b| b == 0).unwrap_or(32)],
        )
        .unwrap();
        assert_eq!(frame_str, "map");
    }

    // ============================================================================
    // Waypoint Edge Cases
    // ============================================================================

    #[test]
    fn test_waypoint_with_stop() {
        let wp = Waypoint::new(Pose2D::new(1.0, 2.0, 0.0)).with_stop();
        assert_eq!(wp.stop_required, 1);
        assert_eq!(wp.velocity.linear, [0.0; 3]);
        assert_eq!(wp.velocity.angular, [0.0; 3]);
    }

    #[test]
    fn test_waypoint_with_velocity() {
        let vel = Twist::new_2d(1.5, 0.0);
        let wp = Waypoint::new(Pose2D::new(0.0, 0.0, 0.0)).with_velocity(vel);
        assert!((wp.velocity.linear[0] - 1.5).abs() < 1e-10);
    }

    #[test]
    fn test_waypoint_default_curvature() {
        let wp = Waypoint::new(Pose2D::new(0.0, 0.0, 0.0));
        assert_eq!(wp.curvature, 0.0);
        assert_eq!(wp.stop_required, 0);
        assert_eq!(wp.time_from_start, 0.0);
    }

    // ============================================================================
    // NavGoal Edge Cases
    // ============================================================================

    #[test]
    fn test_nav_goal_with_timeout() {
        let target = Pose2D::new(5.0, 3.0, 0.0);
        let goal = NavGoal::new(target, 0.1, 0.1).with_timeout(30.0);
        assert!((goal.timeout_seconds - 30.0).abs() < 1e-10);
    }

    #[test]
    fn test_nav_goal_with_priority() {
        let target = Pose2D::new(5.0, 3.0, 0.0);
        let goal = NavGoal::new(target, 0.1, 0.1).with_priority(0);
        assert_eq!(goal.priority, 0);
    }

    #[test]
    fn test_nav_goal_orientation_wrapping() {
        // Test orientation check across the +-PI boundary
        let target = Pose2D::new(0.0, 0.0, 3.0); // ~171 degrees
        let goal = NavGoal::new(target, 1.0, 0.5);
        // Current pose at -3.0 (~-171 degrees), angle diff should be ~0.28 radians
        let current = Pose2D::new(0.0, 0.0, -3.0);
        // Raw diff = |3.0 - (-3.0)| = 6.0 > PI, normalized = 2*PI - 6.0 ≈ 0.28
        assert!(goal.is_orientation_reached(&current));
    }

    #[test]
    fn test_nav_goal_exact_position() {
        let target = Pose2D::new(5.0, 3.0, 0.0);
        let goal = NavGoal::new(target, 0.0, 0.0);
        // Exactly at goal
        let at_goal = Pose2D::new(5.0, 3.0, 0.0);
        assert!(goal.is_reached(&at_goal));
    }

    #[test]
    fn test_nav_goal_serialization() {
        let target = Pose2D::new(5.0, 3.0, 1.57);
        let goal = NavGoal::new(target, 0.1, 0.05)
            .with_timeout(60.0)
            .with_priority(2);
        let json = serde_json::to_string(&goal).unwrap();
        let recovered: NavGoal = serde_json::from_str(&json).unwrap();
        assert!((recovered.target_pose.x - 5.0).abs() < 1e-10);
        assert!((recovered.tolerance_position - 0.1).abs() < 1e-10);
        assert!((recovered.timeout_seconds - 60.0).abs() < 1e-10);
        assert_eq!(recovered.priority, 2);
    }

    // ============================================================================
    // GoalResult Tests
    // ============================================================================

    #[test]
    fn test_goal_result_with_error() {
        let result = GoalResult::new(42, GoalStatus::Aborted).with_error("Obstacle detected");
        assert_eq!(result.goal_id, 42);
        assert_eq!(result.status, GoalStatus::Aborted as u8);
        let end = result
            .error_message
            .iter()
            .position(|&b| b == 0)
            .unwrap_or(64);
        let msg = std::str::from_utf8(&result.error_message[..end]).unwrap();
        assert_eq!(msg, "Obstacle detected");
    }

    #[test]
    fn test_goal_result_long_error_truncated() {
        let long_msg = "x".repeat(100);
        let result = GoalResult::new(1, GoalStatus::Aborted).with_error(&long_msg);
        let end = result
            .error_message
            .iter()
            .position(|&b| b == 0)
            .unwrap_or(64);
        assert_eq!(end, 63); // truncated to 63 chars + null terminator
    }

    #[test]
    fn test_goal_result_default_pending() {
        let result = GoalResult::default();
        assert_eq!(result.status, GoalStatus::Pending as u8);
        assert_eq!(result.goal_id, 0);
    }

    // ============================================================================
    // OccupancyGrid Additional Edge Cases
    // ============================================================================

    #[test]
    fn test_occupancy_grid_1x1() {
        let mut grid = OccupancyGrid::new(1, 1, 1.0, Pose2D::origin());
        assert_eq!(grid.data.len(), 1);
        assert!(grid.set_occupancy(0, 0, 50));
        assert_eq!(grid.occupancy(0, 0), Some(50));
        assert!(!grid.set_occupancy(1, 0, 0)); // out of bounds
    }

    #[test]
    fn test_occupancy_grid_default_unknown() {
        let grid = OccupancyGrid::new(5, 5, 0.1, Pose2D::origin());
        // All cells should initialize to -1 (unknown)
        for x in 0..5 {
            for y in 0..5 {
                assert_eq!(grid.occupancy(x, y), Some(-1));
            }
        }
    }

    #[test]
    fn test_occupancy_grid_is_free_unknown() {
        let grid = OccupancyGrid::new(5, 5, 0.1, Pose2D::origin());
        // Unknown cells (-1) should NOT be considered free
        let (wx, wy) = grid.grid_to_world(2, 2).unwrap();
        assert!(!grid.is_free(wx, wy));
    }

    #[test]
    fn test_occupancy_grid_is_occupied_threshold() {
        let mut grid = OccupancyGrid::new(10, 10, 0.1, Pose2D::origin());
        grid.set_occupancy(5, 5, 49); // just below threshold
        let (x1, y1) = grid.grid_to_world(5, 5).unwrap();
        assert!(!grid.is_occupied(x1, y1));
        assert!(grid.is_free(x1, y1));

        grid.set_occupancy(5, 5, 50); // at threshold
        let (x2, y2) = grid.grid_to_world(5, 5).unwrap();
        assert!(grid.is_occupied(x2, y2));
        assert!(!grid.is_free(x2, y2));
    }

    #[test]
    fn test_occupancy_grid_large_map() {
        let grid = OccupancyGrid::new(1000, 1000, 0.05, Pose2D::origin());
        assert_eq!(grid.data.len(), 1_000_000);
        // Test boundary coordinates
        assert!(grid.world_to_grid(0.0, 0.0).is_some());
        assert!(grid.world_to_grid(49.9, 49.9).is_some());
        // world_to_grid uses a 1e-6 epsilon, so go clearly outside the boundary
        assert!(grid.world_to_grid(50.1, 50.1).is_none());
    }

    #[test]
    fn test_occupancy_grid_negative_origin() {
        let grid = OccupancyGrid::new(10, 10, 0.1, Pose2D::new(-0.5, -0.5, 0.0));
        // Grid origin is at (-0.5, -0.5), so world (0,0) maps to cell (5,5)
        let (gx, gy) = grid.world_to_grid(0.0, 0.0).unwrap();
        assert_eq!(gx, 5);
        assert_eq!(gy, 5);
    }

    #[test]
    fn test_occupancy_grid_grid_to_world_out_of_bounds() {
        let grid = OccupancyGrid::new(10, 10, 0.1, Pose2D::origin());
        assert!(grid.grid_to_world(10, 10).is_none());
        assert!(grid.grid_to_world(0, 0).is_some());
        assert!(grid.grid_to_world(9, 9).is_some());
    }

    // ============================================================================
    // PathPlan Additional Edge Cases
    // ============================================================================

    #[test]
    fn test_path_plan_from_waypoints() {
        let waypoints = vec![[0.0_f32, 0.0, 0.0], [1.0, 0.0, 0.0], [2.0, 1.0, 0.5]];
        let goal = [5.0_f32, 5.0, 1.57];
        let plan = PathPlan::from_waypoints(&waypoints, goal);
        assert_eq!(plan.waypoint_count, 3);
        assert_eq!(plan.goal_pose, [5.0, 5.0, 1.57]);
        assert_eq!(plan.waypoint(0), Some([0.0, 0.0, 0.0]));
        assert_eq!(plan.waypoint(2), Some([2.0, 1.0, 0.5]));
        assert_eq!(plan.waypoint(3), None);
    }

    #[test]
    fn test_path_plan_from_waypoints_over_256() {
        let waypoints: Vec<[f32; 3]> = (0..300).map(|i| [i as f32, 0.0, 0.0]).collect();
        let plan = PathPlan::from_waypoints(&waypoints, [0.0; 3]);
        // Should clamp to 256
        assert_eq!(plan.waypoint_count, 256);
    }

    #[test]
    fn test_path_plan_waypoint_data_layout() {
        let mut plan = PathPlan::new();
        plan.add_waypoint(1.0, 2.0, 3.0);
        plan.add_waypoint(4.0, 5.0, 6.0);
        // Verify raw data layout: [x0, y0, theta0, x1, y1, theta1, ...]
        assert_eq!(plan.waypoint_data[0], 1.0);
        assert_eq!(plan.waypoint_data[1], 2.0);
        assert_eq!(plan.waypoint_data[2], 3.0);
        assert_eq!(plan.waypoint_data[3], 4.0);
        assert_eq!(plan.waypoint_data[4], 5.0);
        assert_eq!(plan.waypoint_data[5], 6.0);
    }

    // ============================================================================
    // CostMap Tests
    // ============================================================================

    #[test]
    fn test_costmap_from_occupancy_grid() {
        let mut grid = OccupancyGrid::new(10, 10, 0.1, Pose2D::origin());
        // Set one cell as occupied
        grid.set_occupancy(5, 5, 100);
        let costmap = CostMap::from_occupancy_grid(grid, 0.2);
        assert_eq!(costmap.costs.len(), 100);
        // The occupied cell should have lethal cost
        let idx = 5 * 10 + 5;
        assert_eq!(costmap.costs[idx], costmap.lethal_cost);
    }

    #[test]
    fn test_costmap_cost_outside_map() {
        let grid = OccupancyGrid::new(10, 10, 0.1, Pose2D::origin());
        let costmap = CostMap::from_occupancy_grid(grid, 0.0);
        // Outside map should return lethal cost
        let cost = costmap.cost(-1.0, -1.0);
        assert_eq!(cost, Some(costmap.lethal_cost));
    }

    // ============================================================================
    // VelocityObstacle Tests
    // ============================================================================

    #[test]
    fn test_velocity_obstacles_default() {
        let vobs = VelocityObstacles::default();
        assert_eq!(vobs.count, 0);
        assert_eq!(vobs.timestamp_ns, 0);
    }

    #[test]
    fn test_velocity_obstacle_default() {
        let vo = VelocityObstacle::default();
        assert_eq!(vo.position, [0.0, 0.0]);
        assert_eq!(vo.velocity, [0.0, 0.0]);
        assert_eq!(vo.radius, 0.0);
        assert_eq!(vo.obstacle_id, 0);
    }

    // ============================================================================
    // GoalStatus Tests
    // ============================================================================

    #[test]
    fn test_goal_status_default_is_pending() {
        let status = GoalStatus::default();
        assert_eq!(status, GoalStatus::Pending);
    }

    #[test]
    fn test_goal_status_values() {
        assert_eq!(GoalStatus::Pending as u8, 0);
        assert_eq!(GoalStatus::Active as u8, 1);
        assert_eq!(GoalStatus::Succeeded as u8, 2);
        assert_eq!(GoalStatus::Aborted as u8, 3);
        assert_eq!(GoalStatus::Cancelled as u8, 4);
        assert_eq!(GoalStatus::Preempted as u8, 5);
        assert_eq!(GoalStatus::TimedOut as u8, 6);
    }
}

/// Path plan message for navigation
///
/// Fixed-size path plan for zero-copy IPC transfer.
/// Stores up to 256 waypoints as [x, y, theta] coordinates.
#[repr(C)]
#[derive(Debug, Clone, Copy, Serialize, Deserialize, LogSummary)]
pub struct PathPlan {
    /// Waypoint data: 256 waypoints × 3 floats (x, y, theta)
    #[serde(with = "serde_arrays")]
    pub waypoint_data: [f32; 768],
    /// Goal pose [x, y, theta]
    pub goal_pose: [f32; 3],
    /// Number of valid waypoints
    pub waypoint_count: u16,
    /// Timestamp in nanoseconds since epoch
    pub timestamp_ns: u64,
}

impl Default for PathPlan {
    fn default() -> Self {
        Self {
            waypoint_data: [0.0; 768],
            goal_pose: [0.0; 3],
            waypoint_count: 0,
            timestamp_ns: 0,
        }
    }
}

impl PathPlan {
    /// Create a new empty path plan
    pub fn new() -> Self {
        Self {
            timestamp_ns: crate::transform_frame::timestamp_now(),
            ..Default::default()
        }
    }

    /// Add a waypoint
    pub fn add_waypoint(&mut self, x: f32, y: f32, theta: f32) -> bool {
        if self.waypoint_count >= 256 {
            return false;
        }
        let idx = self.waypoint_count as usize * 3;
        self.waypoint_data[idx] = x;
        self.waypoint_data[idx + 1] = y;
        self.waypoint_data[idx + 2] = theta;
        self.waypoint_count += 1;
        true
    }

    /// Get waypoint at index as [x, y, theta]
    pub fn waypoint(&self, index: u16) -> Option<[f32; 3]> {
        if index >= self.waypoint_count {
            return None;
        }
        let idx = index as usize * 3;
        Some([
            self.waypoint_data[idx],
            self.waypoint_data[idx + 1],
            self.waypoint_data[idx + 2],
        ])
    }

    /// Check if path is empty
    pub fn is_empty(&self) -> bool {
        self.waypoint_count == 0
    }

    /// Create a path plan from a slice of waypoints
    pub fn from_waypoints(waypoints: &[[f32; 3]], goal: [f32; 3]) -> Self {
        let mut plan = Self::new();
        plan.goal_pose = goal;
        let count = waypoints.len().min(256);
        for (i, wp) in waypoints.iter().take(count).enumerate() {
            plan.waypoint_data[i * 3] = wp[0];
            plan.waypoint_data[i * 3 + 1] = wp[1];
            plan.waypoint_data[i * 3 + 2] = wp[2];
        }
        plan.waypoint_count = count as u16;
        plan
    }
}

// =============================================================================
// POD (Plain Old Data) Message Support
// =============================================================================

crate::messages::impl_pod_message!(
    NavGoal,
    GoalResult,
    Waypoint,
    NavPath,
    VelocityObstacle,
    VelocityObstacles,
    PathPlan,
);
