# Path Planner Node

Global path planning for mobile robot autonomous navigation using A* and RRT algorithms.

## Overview

The Path Planner Node implements multiple path planning algorithms for autonomous mobile robot navigation. It generates collision-free paths from the current robot position to a goal position using either A* (grid-based) or RRT (sampling-based) algorithms. The node continuously monitors the robot's position, updates an occupancy grid from lidar data, and dynamically replans when obstacles appear or the robot deviates from the planned path.

This node is designed for global path planning in autonomous navigation systems and integrates seamlessly with local planners, costmaps, and control nodes.

## Architecture

**This node is a thin wrapper** around the pure algorithms in `horus_library/algorithms/`:

- **`algorithms::astar::AStar`** - Grid-based A* pathfinding algorithm
- **`algorithms::rrt::RRT`** - Sampling-based RRT motion planning algorithm
- **`algorithms::occupancy_grid::OccupancyGrid`** - 2D occupancy grid mapping with ray tracing

The node handles:
- Topic subscription/publishing (Hub I/O)
- Lidar data reception and processing
- Coordinate frame conversions (world ↔ grid)
- Dynamic replanning triggers
- Path validity checking

The algorithms handle:
- Pure pathfinding computation (no I/O)
- Obstacle avoidance logic
- Path optimization
- Grid operations

This separation enables algorithm reuse across different nodes and simplifies testing.

## Topics

### Subscribers

| Topic | Type | Description |
|-------|------|-------------|
| `odom` | `Odometry` | Current robot pose (x, y, theta) and velocity |
| `lidar_scan` | `LaserScan` | Laser scan data for obstacle detection |
| `goal` | `PathPlan` | Goal position and waypoints to navigate to |

### Publishers

| Topic | Type | Description |
|-------|------|-------------|
| `path_plan` | `PathPlan` | Planned path with waypoints and goal pose |

## Configuration Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `grid_resolution` | `f64` | `0.1` | Grid cell size in meters (10cm resolution) |
| `grid_width` | `usize` | `200` | Grid width in cells (20m total width) |
| `grid_height` | `usize` | `200` | Grid height in cells (20m total height) |
| `robot_radius` | `f64` | `0.3` | Robot radius for collision checking (meters) |
| `planning_algorithm` | `PlanningAlgorithm` | `AStar` | Algorithm to use: A* or RRT |
| `replanning_threshold` | `f64` | `0.5` | Distance deviation before replanning (meters) |
| `heuristic_weight` | `f64` | `1.0` | A* heuristic weight (1.0 = optimal, >1.0 = faster) |
| `rrt_max_iterations` | `usize` | `1000` | Maximum RRT iterations before failure |
| `rrt_step_size` | `f64` | `0.5` | RRT tree extension step size (meters) |
| `rrt_goal_bias` | `f64` | `0.1` | Probability of sampling goal (0.0-1.0) |

## Message Types

### PathPlan

Path message containing waypoints and goal information:

```rust
pub struct PathPlan {
    pub waypoints: Vec<[f32; 3]>,      // Array of [x, y, z] waypoints
    pub goal_pose: [f32; 3],           // Final goal [x, y, theta]
    pub path_length: u32,              // Number of waypoints
    pub timestamp: u64,                // Plan generation timestamp
}
```

### Odometry

Robot pose and velocity information:

```rust
pub struct Odometry {
    pub pose: Pose2D,                  // Current position
    pub velocity: Twist,               // Linear and angular velocity
    pub timestamp: u64,
}

pub struct Pose2D {
    pub x: f64,                        // X position (meters)
    pub y: f64,                        // Y position (meters)
    pub theta: f64,                    // Orientation (radians)
}
```

### LaserScan

Lidar scan data for obstacle detection:

```rust
pub struct LaserScan {
    pub ranges: Vec<f32>,              // Distance measurements
    pub angle_min: f32,                // Start angle (radians)
    pub angle_max: f32,                // End angle (radians)
    pub angle_increment: f32,          // Angular resolution
    pub range_min: f32,                // Minimum valid range
    pub range_max: f32,                // Maximum valid range
    pub timestamp: u64,
}
```

### Twist

Velocity command structure:

```rust
pub struct Twist {
    pub linear: f64,                   // Linear velocity (m/s)
    pub angular: f64,                  // Angular velocity (rad/s)
}
```

## Public API

### Construction

```rust
use horus_library::nodes::PathPlannerNode;

// Create with default topics
let mut planner = PathPlannerNode::new()?;

// Create with custom topics
let mut planner = PathPlannerNode::new_with_topics(
    "planned_path",      // path output topic
    "robot_odom",        // odometry topic
    "laser_scan",        // lidar topic
    "navigation_goal"    // goal input topic
)?;
```

### Configuration Methods

```rust
// Set grid map parameters
planner.set_grid_config(
    0.05,    // 5cm resolution
    400,     // 20m width
    400      // 20m height
);

// Set robot radius for collision checking
planner.set_robot_radius(0.35);  // 35cm radius

// Set planning algorithm
planner.set_algorithm(false);    // Use A*
planner.set_algorithm(true);     // Use RRT

// Set goal position
planner.set_goal(5.0, 3.0, 1.57);  // x=5m, y=3m, theta=90deg

// Get current path
let path = planner.get_path();

// Check if path is valid
if planner.is_path_valid() {
    eprintln!("Valid path with {} waypoints", path.len());
}
```

## Usage Examples

### Basic A* Path Planning

```rust
use horus_library::nodes::PathPlannerNode;
use horus_core::{Node, Runtime};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut runtime = Runtime::new()?;

    // Create path planner with A* algorithm
    let mut planner = PathPlannerNode::new_with_topics(
        "global_path",
        "odom",
        "scan",
        "goal"
    )?;

    // Configure for indoor navigation
    planner.set_grid_config(0.05, 400, 400);  // 5cm resolution, 20x20m
    planner.set_robot_radius(0.25);            // 25cm robot
    planner.set_algorithm(false);              // Use A*

    runtime.add_node(planner);
    runtime.run()?;

    Ok(())
}
```

### RRT Path Planning for Complex Environments

```rust
use horus_library::nodes::PathPlannerNode;
use horus_core::{Node, Runtime};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut runtime = Runtime::new()?;

    // Create path planner with RRT algorithm
    let mut planner = PathPlannerNode::new_with_topics(
        "global_path",
        "odom",
        "scan",
        "goal"
    )?;

    // Configure for cluttered environments
    planner.set_grid_config(0.1, 200, 200);
    planner.set_robot_radius(0.30);
    planner.set_algorithm(true);  // Use RRT for complex spaces

    runtime.add_node(planner);
    runtime.run()?;

    Ok(())
}
```

### Dynamic Replanning with Obstacle Avoidance

```rust
use horus_library::nodes::PathPlannerNode;
use horus_core::{Node, Runtime};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut runtime = Runtime::new()?;

    // Create path planner with dynamic replanning
    let mut planner = PathPlannerNode::new()?;

    // Configure for dynamic environments
    planner.set_grid_config(0.1, 300, 300);
    planner.set_robot_radius(0.35);
    planner.set_algorithm(false);  // A* for fast replanning

    // The planner automatically replans when:
    // - New obstacles detected by lidar
    // - Robot deviates > 0.5m from path
    // - No valid path exists

    runtime.add_node(planner);
    runtime.run()?;

    Ok(())
}
```

### Warehouse Navigation System

```rust
use horus_library::nodes::PathPlannerNode;
use horus_core::{Node, Runtime, Hub};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut runtime = Runtime::new()?;

    // High-resolution planner for warehouse aisles
    let mut planner = PathPlannerNode::new_with_topics(
        "warehouse_path",
        "robot_odom",
        "safety_lidar",
        "warehouse_goals"
    )?;

    // Fine-grained planning for tight spaces
    planner.set_grid_config(0.03, 500, 500);  // 3cm resolution
    planner.set_robot_radius(0.40);            // 40cm safety margin
    planner.set_algorithm(false);              // A* for structured space

    runtime.add_node(planner);
    runtime.run()?;

    Ok(())
}
```

## Supported Algorithms

### Algorithm Comparison

| Algorithm | Type | Completeness | Optimality | Speed | Best For |
|-----------|------|--------------|------------|-------|----------|
| **A*** | Grid-based | Resolution complete | Optimal (w=1.0) | Fast | Structured environments, known maps |
| **Dijkstra** | Grid-based | Resolution complete | Optimal | Slower | When goal changes frequently |
| **RRT** | Sampling-based | Probabilistically complete | Suboptimal | Variable | Complex/cluttered spaces |
| **RRT*** | Sampling-based | Probabilistically complete | Asymptotically optimal | Slower | High-quality paths needed |
| **D* Lite** | Incremental | Resolution complete | Optimal | Very fast (replanning) | Dynamic environments |

### A* (A-Star)

**Algorithm**: Grid-based graph search with heuristic guidance.

**Characteristics**:
- Uses Euclidean distance heuristic to goal
- Explores cells in order of f-cost = g-cost + h-cost
- Supports 8-directional movement (diagonal allowed)
- Guarantees optimal path when heuristic weight = 1.0

**When to use**:
- Structured environments (buildings, warehouses)
- Static or slowly changing obstacles
- When optimality is important
- Grid representation is natural

**Parameters**:
```rust
planner.set_algorithm(false);  // Enable A*
// heuristic_weight: 1.0 = optimal, 1.5-2.0 = faster but suboptimal
```

### Dijkstra

**Algorithm**: Uniform-cost search without heuristic (A* with h=0).

**Characteristics**:
- Explores all directions equally
- Always finds optimal path
- Useful when goal changes frequently (precomputed cost map)

**When to use**:
- Multiple goals to evaluate
- Goal position uncertain
- Need complete cost map

### RRT (Rapidly-exploring Random Tree)

**Algorithm**: Sampling-based tree growth toward random samples.

**Characteristics**:
- Builds tree by sampling random points
- Extends tree toward samples with fixed step size
- Goal bias probability directs search
- Handles high-dimensional configuration spaces

**When to use**:
- Cluttered/complex environments
- Non-grid-based planning
- When optimality is not critical
- Narrow passages exist

**Parameters**:
```rust
planner.set_algorithm(true);  // Enable RRT
// rrt_max_iterations: 1000 typical, increase for complex spaces
// rrt_step_size: 0.5m typical, smaller for precision
// rrt_goal_bias: 0.1 typical (10% goal sampling)
```

### RRT* (RRT-Star)

**Algorithm**: RRT with rewiring for path optimization.

**Characteristics**:
- Asymptotically optimal (approaches optimal as iterations increase)
- Rewires tree to improve path quality
- Slower than basic RRT

**When to use**:
- Higher quality paths needed than RRT
- Computation time is available
- Can afford more iterations

### D* Lite

**Algorithm**: Incremental heuristic search for dynamic environments.

**Characteristics**:
- Efficiently replans when obstacles appear
- Only recomputes affected portions
- Optimal and very fast for replanning

**When to use**:
- Highly dynamic environments
- Frequent obstacle changes
- Real-time replanning required

## Path Smoothing and Optimization

### Path Smoothing Techniques

Raw paths from grid-based planners often have unnecessary turns and jagged edges. Post-processing can improve path quality:

#### 1. Douglas-Peucker Simplification

Reduces waypoints while preserving path shape:

```rust
// Simplify path to reduce waypoint count
fn simplify_path(path: &[(f64, f64)], epsilon: f64) -> Vec<(f64, f64)> {
    // Remove waypoints that deviate less than epsilon from line
    // Implementation uses Douglas-Peucker algorithm
}
```

#### 2. Bezier Curve Smoothing

Creates smooth curves through waypoints:

```rust
// Generate smooth Bezier curve path
fn smooth_bezier(waypoints: &[(f64, f64)]) -> Vec<(f64, f64)> {
    // Fit cubic Bezier curves between waypoints
}
```

#### 3. Gradient Descent Optimization

Optimizes path to minimize length while avoiding obstacles:

```rust
// Optimize path using gradient descent
fn optimize_path(path: &mut Vec<(f64, f64)>, iterations: usize) {
    for _ in 0..iterations {
        // Pull waypoints toward straight line
        // Push away from obstacles
    }
}
```

### Path Quality Metrics

Evaluate planned paths:

```rust
// Calculate path metrics
fn evaluate_path(path: &[(f64, f64)]) -> PathMetrics {
    PathMetrics {
        length: calculate_path_length(path),
        smoothness: calculate_curvature(path),
        clearance: min_obstacle_distance(path),
        safety: compute_safety_score(path),
    }
}
```

## Troubleshooting

### Issue: No path found

**Symptoms**: Planner returns empty path, robot doesn't move.

**Possible Causes**:
1. Start or goal position is in obstacle
2. No collision-free path exists
3. Grid resolution too coarse
4. Robot radius too large

**Solutions**:
```rust
// 1. Check if goal is reachable
if !planner.is_path_valid() {
    eprintln!("No valid path - check goal position");
}

// 2. Increase grid resolution
planner.set_grid_config(0.05, 400, 400);  // Finer grid

// 3. Reduce robot radius (if safe)
planner.set_robot_radius(0.25);

// 4. Try RRT for difficult scenarios
planner.set_algorithm(true);
```

### Issue: Inefficient paths

**Symptoms**: Path takes long route, unnecessary turns, not smooth.

**Possible Causes**:
1. A* heuristic weight too low
2. Grid resolution too fine (overconstraining)
3. Path not smoothed
4. Obstacle inflation too aggressive

**Solutions**:
```rust
// 1. Increase heuristic weight for faster paths
// (internal parameter - modify heuristic_weight in node)

// 2. Use coarser grid if appropriate
planner.set_grid_config(0.15, 150, 150);

// 3. Apply path smoothing post-processing

// 4. Reduce robot radius if too conservative
planner.set_robot_radius(0.20);
```

### Issue: Oscillating behavior

**Symptoms**: Robot repeatedly replans, path constantly changing.

**Possible Causes**:
1. Replanning threshold too sensitive
2. Noisy odometry or lidar
3. Dynamic obstacles causing frequent invalidation

**Solutions**:
```rust
// Increase replanning threshold (internal parameter)
// Modify replanning_threshold to 1.0 or higher

// Filter noisy sensor data before path planning

// Use temporal filtering for dynamic obstacles
```

### Issue: Goal not reached

**Symptoms**: Robot stops before reaching goal.

**Possible Causes**:
1. Goal tolerance too tight
2. Path ends before actual goal
3. Local planner issue (not path planner)

**Solutions**:
```rust
// Ensure goal is set correctly
planner.set_goal(target_x, target_y, target_theta);

// Check path includes goal
let path = planner.get_path();
if let Some(last) = path.last() {
    eprintln!("Path ends at: ({}, {})", last.0, last.1);
}

// Verify with local planner/controller
```

### Issue: Planning too slow

**Symptoms**: High CPU usage, slow replanning, missed deadlines.

**Possible Causes**:
1. Grid too large or resolution too fine
2. RRT iterations too high
3. Planning every tick unnecessarily

**Solutions**:
```rust
// 1. Optimize grid size
planner.set_grid_config(0.1, 200, 200);  // Balanced settings

// 2. Reduce RRT iterations
// Modify rrt_max_iterations to 500

// 3. Rate-limit planning (plan every N ticks)

// 4. Use A* instead of RRT for speed
planner.set_algorithm(false);
```

## Integration with Navigation Stack

### Complete Navigation System

The Path Planner integrates with other nodes to form a complete navigation stack:

```rust
use horus_library::nodes::{PathPlannerNode, DifferentialDriveNode};
use horus_core::{Node, Runtime};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut runtime = Runtime::new()?;

    // Global path planner
    let mut global_planner = PathPlannerNode::new_with_topics(
        "global_path",
        "odom",
        "scan",
        "goal"
    )?;
    global_planner.set_grid_config(0.1, 300, 300);
    global_planner.set_robot_radius(0.30);
    global_planner.set_algorithm(false);

    // Local planner / controller
    let mut local_controller = DifferentialDriveNode::new_with_topics(
        "cmd_vel",
        "odom",
        "global_path"
    )?;

    runtime.add_node(global_planner);
    runtime.add_node(local_controller);
    runtime.run()?;

    Ok(())
}
```

### Navigation Stack Components

```
Goal  Global Planner  Local Planner  Motor Controller  Robot
                            
    Costmap/Map         Sensor Data
```

### Costmap Integration

For advanced navigation, integrate with costmap layers:

```rust
// Pseudo-code for costmap integration
struct NavigationStack {
    global_planner: PathPlannerNode,
    local_planner: LocalPlannerNode,
    costmap: CostmapNode,  // Maintains obstacle layers
}

impl NavigationStack {
    fn plan_and_execute(&mut self) {
        // Update costmap from sensors
        self.costmap.update();

        // Plan global path using costmap
        self.global_planner.update_grid(&self.costmap);

        // Execute with local planner
        self.local_planner.follow_path();
    }
}
```

### Local Planner Coordination

The global planner provides waypoints; local planner handles execution:

```rust
// Global planner produces waypoints
let global_path = global_planner.get_path();

// Local planner follows path with obstacle avoidance
// - Pure pursuit controller
// - Dynamic window approach (DWA)
// - Model predictive control (MPC)
```

### Recovery Behaviors

Handle planning failures gracefully:

```rust
fn navigation_with_recovery(planner: &mut PathPlannerNode) {
    if !planner.is_path_valid() {
        // Recovery behavior 1: Rotate in place to clear costmap
        rotate_recovery();

        // Recovery behavior 2: Back up and retry
        backup_recovery();

        // Recovery behavior 3: Expand search space
        planner.set_grid_config(0.15, 400, 400);

        // Recovery behavior 4: Try alternative algorithm
        planner.set_algorithm(true);  // Switch to RRT
    }
}
```

## Performance Considerations

### Computational Complexity

| Algorithm | Time Complexity | Space Complexity |
|-----------|----------------|------------------|
| A* | O(b^d) worst case, O(n log n) typical | O(n) |
| RRT | O(n log n) where n = iterations | O(n) |
| Grid update | O(m * r^2) where m = lidar points, r = robot radius | O(w * h) |

### Planning Rate Guidelines

Recommended planning frequencies for different scenarios:

- **Static environments**: 1-2 Hz (replan every 0.5-1.0s)
- **Dynamic environments**: 5-10 Hz (replan every 0.1-0.2s)
- **Highly dynamic**: 20 Hz (replan every 50ms)

```rust
// Rate-limited planning example
let mut plan_counter = 0;
const PLAN_EVERY_N_TICKS: u32 = 10;

// In tick() implementation
plan_counter += 1;
if plan_counter >= PLAN_EVERY_N_TICKS {
    plan_counter = 0;
    // Perform planning
}
```

### Memory Optimization

Grid memory usage: `grid_width × grid_height × sizeof(bool)`

```rust
// Default: 200 × 200 = 40,000 bytes
// High-res: 500 × 500 = 250,000 bytes

// Optimize for memory-constrained systems
planner.set_grid_config(0.2, 100, 100);  // 10,000 bytes
```

### CPU Usage Optimization

1. **Lazy planning**: Only replan when necessary
2. **Early termination**: Stop A* when close enough
3. **Resolution switching**: Coarse planning first, refine later
4. **Spatial hashing**: Faster obstacle queries

### Real-time Performance

For real-time guarantees:

```rust
// Time-bounded planning
fn plan_with_timeout(planner: &mut PathPlannerNode, max_ms: u64) {
    let start = std::time::Instant::now();

    // Plan with iteration limit
    // Stop if timeout exceeded

    if start.elapsed().as_millis() > max_ms as u128 {
        // Use best path found so far
    }
}
```

## Related Nodes

- **DifferentialDriveNode**: Local planner that executes planned paths
- **LaserScanNode**: Provides obstacle detection data
- **OdometryNode**: Provides current robot position
- **MapServerNode**: Loads and serves static maps
- **CostmapNode**: Maintains layered obstacle representations
- **LocalPlannerNode**: DWA/TEB local planning
- **RecoveryNode**: Handles planning failures

## Advanced Topics

### Multi-Goal Planning

Plan paths through multiple waypoints:

```rust
fn plan_multi_goal(
    planner: &mut PathPlannerNode,
    waypoints: &[(f64, f64)]
) -> Vec<(f64, f64)> {
    let mut complete_path = Vec::new();

    for window in waypoints.windows(2) {
        let (start, goal) = (window[0], window[1]);
        planner.set_goal(goal.0, goal.1, 0.0);

        // Plan segment
        let segment = planner.get_path().clone();
        complete_path.extend(segment);
    }

    complete_path
}
```

### Kinodynamic Planning

Consider robot dynamics during planning:

```rust
// Check if path is kinematically feasible
fn validate_kinematics(
    path: &[(f64, f64)],
    max_velocity: f64,
    max_acceleration: f64
) -> bool {
    for window in path.windows(2) {
        let distance = euclidean_distance(window[0], window[1]);
        let required_velocity = distance / TIME_STEP;

        if required_velocity > max_velocity {
            return false;  // Path requires excessive speed
        }
    }
    true
}
```

### Probabilistic Planning

Handle uncertainty in obstacle positions:

```rust
// Plan with probabilistic obstacles
fn probabilistic_planning(
    planner: &mut PathPlannerNode,
    obstacle_probabilities: &[f64]
) {
    // Weight path cost by obstacle probability
    // Accept paths with acceptable collision probability
}
```

## See Also

- [A* Pathfinding](https://en.wikipedia.org/wiki/A*_search_algorithm)
- [RRT Algorithm](https://en.wikipedia.org/wiki/Rapidly-exploring_random_tree)
- [Motion Planning](https://en.wikipedia.org/wiki/Motion_planning)
- [Occupancy Grid Mapping](https://en.wikipedia.org/wiki/Occupancy_grid_mapping)
- [ROS Navigation Stack](http://wiki.ros.org/navigation)
