#![allow(clippy::needless_range_loop)] // Grid indexing patterns are clearer with explicit indices

use crate::{LaserScan, Odometry, PathPlan};
use horus_core::error::HorusResult;

// Import algorithms from horus_library/algorithms
use crate::algorithms::astar::AStar;
use crate::algorithms::occupancy_grid::OccupancyGrid;
use crate::algorithms::rrt::RRT;

// Type alias for cleaner signatures
type Result<T> = HorusResult<T>;
use horus_core::{Node, NodeInfo, Topic};
use std::time::{SystemTime, UNIX_EPOCH};

// Processor imports for hybrid pattern
use crate::nodes::processor::{
    ClosureProcessor, FilterProcessor, PassThrough, Pipeline, Processor,
};

/// Path Planner Node - A* and RRT path planning for autonomous navigation
///
/// Plans collision-free paths from current position to goal using A* algorithm
/// for grid-based environments and RRT for complex obstacle spaces.
///
/// This node is a thin wrapper around the pure algorithms in horus_library/algorithms.
///
/// # Hybrid Pattern
///
/// This node supports the hybrid pattern for custom processing:
///
/// ```rust,ignore
/// // Default mode - just plans paths
/// let node = PathPlannerNode::new()?;
///
/// // With custom processing - smooth paths or add waypoints
/// let node = PathPlannerNode::builder()
///     .with_closure(|mut plan| {
///         // Post-process path (e.g., smooth it)
///         plan
///     })
///     .build()?;
/// ```
pub struct PathPlannerNode<P = PassThrough<PathPlan>>
where
    P: Processor<PathPlan>,
{
    plan_publisher: Topic<PathPlan>,
    odometry_subscriber: Topic<Odometry>,
    lidar_subscriber: Topic<LaserScan>,
    goal_subscriber: Topic<PathPlan>, // Receives goal positions

    // Current state
    current_pose: (f64, f64, f64), // (x, y, theta)
    goal_pose: (f64, f64, f64),

    // Algorithm instances
    astar: AStar,
    rrt: RRT,
    occupancy_grid: OccupancyGrid,

    // Configuration
    grid_resolution: f64,
    robot_radius: f64,
    planning_algorithm: PlanningAlgorithm,

    // Path planning state
    current_path: Vec<(f64, f64)>,
    path_valid: bool,
    replanning_threshold: f64, // replan if deviation > threshold

    // Processor for hybrid pattern
    processor: P,
}

#[derive(Clone, Copy)]
enum PlanningAlgorithm {
    AStar,
    Rrt,
}

impl PathPlannerNode {
    /// Create a new path planner node with default topics
    pub fn new() -> Result<Self> {
        Self::new_with_topics("path_plan", "odom", "lidar_scan", "goal")
    }

    /// Create a new path planner node with custom topics
    pub fn new_with_topics(
        plan_topic: &str,
        odom_topic: &str,
        lidar_topic: &str,
        goal_topic: &str,
    ) -> Result<Self> {
        let grid_width = 200;
        let grid_height = 200;
        let grid_resolution = 0.1;

        // Create algorithm instances
        let astar = AStar::new(grid_width, grid_height);

        let rrt = RRT::new((0.0, 0.0), (0.0, 0.0), (-10.0, -10.0), (10.0, 10.0));

        let mut occupancy_grid = OccupancyGrid::new(grid_width, grid_height, grid_resolution);
        occupancy_grid.set_origin(-10.0, -10.0);

        Ok(Self {
            plan_publisher: Topic::new(plan_topic)?,
            odometry_subscriber: Topic::new(odom_topic)?,
            lidar_subscriber: Topic::new(lidar_topic)?,
            goal_subscriber: Topic::new(goal_topic)?,

            current_pose: (0.0, 0.0, 0.0),
            goal_pose: (0.0, 0.0, 0.0),

            astar,
            rrt,
            occupancy_grid,

            grid_resolution,
            robot_radius: 0.3, // 30cm robot radius
            planning_algorithm: PlanningAlgorithm::AStar,

            current_path: Vec::new(),
            path_valid: false,
            replanning_threshold: 0.5, // 50cm deviation
            processor: PassThrough::new(),
        })
    }

    /// Create a builder for configuring the node with custom processing
    pub fn builder() -> PathPlannerNodeBuilder<PassThrough<PathPlan>> {
        PathPlannerNodeBuilder::new()
    }
}

impl<P> PathPlannerNode<P>
where
    P: Processor<PathPlan>,
{
    /// Set grid map parameters
    pub fn set_grid_config(&mut self, resolution: f64, width: usize, height: usize) {
        self.grid_resolution = resolution;
        self.astar = AStar::new(width, height);
        self.occupancy_grid = OccupancyGrid::new(width, height, resolution);
    }

    /// Set robot radius for collision checking
    pub fn set_robot_radius(&mut self, radius: f64) {
        self.robot_radius = radius;
    }

    /// Set planning algorithm
    pub fn set_algorithm(&mut self, use_rrt: bool) {
        self.planning_algorithm = if use_rrt {
            PlanningAlgorithm::Rrt
        } else {
            PlanningAlgorithm::AStar
        };
    }

    /// Set goal position
    pub fn set_goal(&mut self, x: f64, y: f64, theta: f64) {
        self.goal_pose = (x, y, theta);
        self.path_valid = false; // Invalidate current path
    }

    /// Get current path
    pub fn get_path(&self) -> &Vec<(f64, f64)> {
        &self.current_path
    }

    /// Check if path is valid
    pub fn is_path_valid(&self) -> bool {
        self.path_valid
    }

    fn update_occupancy_grid(&mut self, lidar_data: &LaserScan) {
        // Clear previous obstacles
        self.occupancy_grid.clear();

        let (robot_x, robot_y, robot_theta) = self.current_pose;

        // Process lidar points using ray tracing
        for (i, &range) in lidar_data.ranges.iter().enumerate() {
            if range > 0.1 && range < lidar_data.range_max {
                let angle = lidar_data.angle_min as f64
                    + i as f64 * lidar_data.angle_increment as f64
                    + robot_theta;

                let obstacle_x = robot_x + range as f64 * angle.cos();
                let obstacle_y = robot_y + range as f64 * angle.sin();

                // Use occupancy grid's ray tracing
                self.occupancy_grid.ray_trace(
                    (robot_x, robot_y),
                    (obstacle_x, obstacle_y),
                    true, // Mark free cells along ray
                );

                // Inflate obstacles by robot radius
                let (grid_x, grid_y) = self.occupancy_grid.world_to_grid(obstacle_x, obstacle_y);
                let inflation_cells = (self.robot_radius / self.grid_resolution).ceil() as i32;

                for dy in -inflation_cells..=inflation_cells {
                    for dx in -inflation_cells..=inflation_cells {
                        let nx = grid_x + dx;
                        let ny = grid_y + dy;
                        let dist = ((dx * dx + dy * dy) as f64).sqrt() * self.grid_resolution;

                        if dist <= self.robot_radius && nx >= 0 && ny >= 0 {
                            self.occupancy_grid.set_occupied(nx as usize, ny as usize);
                        }
                    }
                }
            }
        }
    }

    fn plan_path_astar(&mut self) -> Vec<(f64, f64)> {
        let (start_x, start_y, _) = self.current_pose;
        let (goal_x, goal_y, _) = self.goal_pose;

        // Convert occupancy grid to A* grid format
        let (width, height) = self.occupancy_grid.get_dimensions();
        let mut grid = vec![vec![false; width]; height];

        for y in 0..height {
            for x in 0..width {
                grid[y][x] = self.occupancy_grid.is_occupied(x, y);
            }
        }

        // Set grid in A* planner
        self.astar.set_grid(grid);

        // Convert world coordinates to grid coordinates
        let (start_grid_x, start_grid_y) = self.occupancy_grid.world_to_grid(start_x, start_y);
        let (goal_grid_x, goal_grid_y) = self.occupancy_grid.world_to_grid(goal_x, goal_y);

        // Set start and goal in A*
        self.astar.set_start(start_grid_x, start_grid_y);
        self.astar.set_goal(goal_grid_x, goal_grid_y);

        // Plan using A*
        if let Some(grid_path) = self.astar.plan() {
            // Convert grid coordinates back to world coordinates
            grid_path
                .iter()
                .map(|(gx, gy)| self.occupancy_grid.grid_to_world(*gx, *gy))
                .collect()
        } else {
            Vec::new()
        }
    }

    fn plan_path_rrt(&mut self) -> Vec<(f64, f64)> {
        let (start_x, start_y, _) = self.current_pose;
        let (goal_x, goal_y, _) = self.goal_pose;

        // Reset RRT with new start/goal
        let (width, height) = self.occupancy_grid.get_dimensions();
        let bounds_size = width as f64 * self.grid_resolution;

        self.rrt = RRT::new(
            (start_x, start_y),
            (goal_x, goal_y),
            (-bounds_size / 2.0, -bounds_size / 2.0),
            (bounds_size / 2.0, bounds_size / 2.0),
        );

        // Add obstacles from occupancy grid
        // Convert grid obstacles to circular obstacles for RRT
        for y in 0..height {
            for x in 0..width {
                if self.occupancy_grid.is_occupied(x, y) {
                    let (world_x, world_y) = self.occupancy_grid.grid_to_world(x as i32, y as i32);
                    self.rrt
                        .add_obstacle((world_x, world_y), self.grid_resolution);
                }
            }
        }

        // Plan using RRT
        self.rrt.plan().unwrap_or_default()
    }

    fn euclidean_distance(&self, x1: f64, y1: f64, x2: f64, y2: f64) -> f64 {
        ((x2 - x1).powi(2) + (y2 - y1).powi(2)).sqrt()
    }

    fn check_path_deviation(&self) -> bool {
        if self.current_path.is_empty() {
            return false;
        }

        let (current_x, current_y, _) = self.current_pose;

        // Find closest point on path
        let mut min_distance = f64::INFINITY;
        for &(path_x, path_y) in &self.current_path {
            let distance = self.euclidean_distance(current_x, current_y, path_x, path_y);
            min_distance = min_distance.min(distance);
        }

        min_distance > self.replanning_threshold
    }

    fn build_and_publish_path(&mut self) {
        if self.current_path.is_empty() {
            return;
        }

        let current_time = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .unwrap()
            .as_nanos() as u64;

        let path_plan = PathPlan {
            waypoints: self
                .current_path
                .iter()
                .map(|&(x, y)| [x as f32, y as f32, 0.0])
                .collect(),
            goal_pose: [
                self.goal_pose.0 as f32,
                self.goal_pose.1 as f32,
                self.goal_pose.2 as f32,
            ],
            path_length: self.current_path.len() as u32,
            timestamp: current_time,
        };

        // Process through the processor pipeline
        if let Some(processed) = self.processor.process(path_plan) {
            let _ = self.plan_publisher.send(processed, &mut None);
        }
    }
}

impl<P> Node for PathPlannerNode<P>
where
    P: Processor<PathPlan>,
{
    fn name(&self) -> &'static str {
        "PathPlannerNode"
    }

    fn init(&mut self, ctx: &mut NodeInfo) -> Result<()> {
        // Call processor hook
        self.processor.on_start();
        ctx.log_info("PathPlannerNode initialized with hybrid processor pattern");
        Ok(())
    }

    fn shutdown(&mut self, ctx: &mut NodeInfo) -> Result<()> {
        ctx.log_info("PathPlannerNode shutting down");
        self.processor.on_shutdown();
        Ok(())
    }

    fn tick(&mut self, _ctx: Option<&mut NodeInfo>) {
        // Call processor tick hook
        self.processor.on_tick();

        // Update current pose from odometry
        if let Some(odom) = self.odometry_subscriber.recv(&mut None) {
            self.current_pose = (odom.pose.x, odom.pose.y, odom.pose.theta);
        }

        // Handle new goal commands
        if let Some(goal) = self.goal_subscriber.recv(&mut None) {
            if !goal.waypoints.is_empty() {
                let goal_point = goal.waypoints.last().unwrap();
                self.set_goal(
                    goal_point[0] as f64,
                    goal_point[1] as f64,
                    goal.goal_pose[2] as f64,
                );
            }
        }

        // Update occupancy grid from lidar
        if let Some(lidar) = self.lidar_subscriber.recv(&mut None) {
            self.update_occupancy_grid(&lidar);
        }

        // Check if we need to replan
        let should_replan =
            !self.path_valid || self.current_path.is_empty() || self.check_path_deviation();

        if should_replan {
            self.current_path = match self.planning_algorithm {
                PlanningAlgorithm::AStar => self.plan_path_astar(),
                PlanningAlgorithm::Rrt => self.plan_path_rrt(),
            };

            self.path_valid = !self.current_path.is_empty();

            // Publish new path (through processor pipeline)
            if self.path_valid {
                self.build_and_publish_path();
            }
        }
    }
}

/// Builder for PathPlannerNode with custom processor
pub struct PathPlannerNodeBuilder<P>
where
    P: Processor<PathPlan>,
{
    plan_topic: String,
    odom_topic: String,
    lidar_topic: String,
    goal_topic: String,
    processor: P,
}

impl PathPlannerNodeBuilder<PassThrough<PathPlan>> {
    /// Create a new builder with default settings
    pub fn new() -> Self {
        Self {
            plan_topic: "path_plan".to_string(),
            odom_topic: "odom".to_string(),
            lidar_topic: "lidar_scan".to_string(),
            goal_topic: "goal".to_string(),
            processor: PassThrough::new(),
        }
    }
}

impl Default for PathPlannerNodeBuilder<PassThrough<PathPlan>> {
    fn default() -> Self {
        Self::new()
    }
}

impl<P> PathPlannerNodeBuilder<P>
where
    P: Processor<PathPlan>,
{
    /// Set the plan output topic
    pub fn with_plan_topic(mut self, topic: &str) -> Self {
        self.plan_topic = topic.to_string();
        self
    }

    /// Set the odometry input topic
    pub fn with_odom_topic(mut self, topic: &str) -> Self {
        self.odom_topic = topic.to_string();
        self
    }

    /// Set the lidar input topic
    pub fn with_lidar_topic(mut self, topic: &str) -> Self {
        self.lidar_topic = topic.to_string();
        self
    }

    /// Set the goal input topic
    pub fn with_goal_topic(mut self, topic: &str) -> Self {
        self.goal_topic = topic.to_string();
        self
    }

    /// Set a custom processor
    pub fn with_processor<P2>(self, processor: P2) -> PathPlannerNodeBuilder<P2>
    where
        P2: Processor<PathPlan>,
    {
        PathPlannerNodeBuilder {
            plan_topic: self.plan_topic,
            odom_topic: self.odom_topic,
            lidar_topic: self.lidar_topic,
            goal_topic: self.goal_topic,
            processor,
        }
    }

    /// Add a closure-based processor
    #[allow(clippy::type_complexity)]
    pub fn with_closure<F>(
        self,
        f: F,
    ) -> PathPlannerNodeBuilder<
        Pipeline<PathPlan, PathPlan, PathPlan, P, ClosureProcessor<PathPlan, PathPlan, F>>,
    >
    where
        F: FnMut(PathPlan) -> PathPlan + Send + 'static,
    {
        PathPlannerNodeBuilder {
            plan_topic: self.plan_topic,
            odom_topic: self.odom_topic,
            lidar_topic: self.lidar_topic,
            goal_topic: self.goal_topic,
            processor: Pipeline::new(self.processor, ClosureProcessor::new(f)),
        }
    }

    /// Add a filter processor
    #[allow(clippy::type_complexity)]
    pub fn with_filter<F>(
        self,
        f: F,
    ) -> PathPlannerNodeBuilder<
        Pipeline<PathPlan, PathPlan, PathPlan, P, FilterProcessor<PathPlan, PathPlan, F>>,
    >
    where
        F: FnMut(PathPlan) -> Option<PathPlan> + Send + 'static,
    {
        PathPlannerNodeBuilder {
            plan_topic: self.plan_topic,
            odom_topic: self.odom_topic,
            lidar_topic: self.lidar_topic,
            goal_topic: self.goal_topic,
            processor: Pipeline::new(self.processor, FilterProcessor::new(f)),
        }
    }

    /// Pipe to another processor (output must remain PathPlan)
    pub fn pipe<P2>(
        self,
        next: P2,
    ) -> PathPlannerNodeBuilder<Pipeline<PathPlan, PathPlan, PathPlan, P, P2>>
    where
        P2: Processor<PathPlan, PathPlan>,
    {
        PathPlannerNodeBuilder {
            plan_topic: self.plan_topic,
            odom_topic: self.odom_topic,
            lidar_topic: self.lidar_topic,
            goal_topic: self.goal_topic,
            processor: Pipeline::new(self.processor, next),
        }
    }

    /// Build the node
    pub fn build(self) -> Result<PathPlannerNode<P>> {
        let grid_width = 200;
        let grid_height = 200;
        let grid_resolution = 0.1;

        // Create algorithm instances
        let astar = AStar::new(grid_width, grid_height);
        let rrt = RRT::new((0.0, 0.0), (0.0, 0.0), (-10.0, -10.0), (10.0, 10.0));

        let mut occupancy_grid = OccupancyGrid::new(grid_width, grid_height, grid_resolution);
        occupancy_grid.set_origin(-10.0, -10.0);

        Ok(PathPlannerNode {
            plan_publisher: Topic::new(&self.plan_topic)?,
            odometry_subscriber: Topic::new(&self.odom_topic)?,
            lidar_subscriber: Topic::new(&self.lidar_topic)?,
            goal_subscriber: Topic::new(&self.goal_topic)?,

            current_pose: (0.0, 0.0, 0.0),
            goal_pose: (0.0, 0.0, 0.0),

            astar,
            rrt,
            occupancy_grid,

            grid_resolution,
            robot_radius: 0.3,
            planning_algorithm: PlanningAlgorithm::AStar,

            current_path: Vec::new(),
            path_valid: false,
            replanning_threshold: 0.5,
            processor: self.processor,
        })
    }
}
