# Collision Detector Node

Safety system for obstacle avoidance and collision prevention using lidar and safety sensors.

## Overview

The Collision Detector Node monitors lidar scans and safety sensors to detect potential collisions and trigger emergency stops or warnings. It implements multiple safety zones with velocity-dependent thresholds, obstacle tracking, and integration with digital safety sensors for comprehensive collision avoidance.

## Architecture

**This node is a thin wrapper** around the pure algorithm in `horus_library/algorithms/`:

- **`algorithms::aabb::AABB`** - Axis-aligned bounding box collision detection

The node handles:
- Topic subscription/publishing (Hub I/O)
- Lidar data reception and processing
- Obstacle position tracking
- Emergency stop triggering
- Safety sensor monitoring
- Collision history filtering

The algorithm handles:
- AABB intersection tests (robot vs obstacles)
- Box creation and expansion
- Point containment checks
- Pure collision detection logic

This separation enables AABB algorithm reuse in different collision detection contexts.

## Topics

### Subscribers

| Topic | Type | Description |
|-------|------|-------------|
| `lidar_scan` | `LaserScan` | Lidar sensor data for obstacle detection |
| `odom` | `Odometry` | Robot pose and velocity for dynamic safety zones |
| `digital_input` | `DigitalIO` | Safety sensor inputs (bumpers, proximity sensors) |

### Publishers

| Topic | Type | Description |
|-------|------|-------------|
| `emergency_stop` | `EmergencyStop` | Emergency stop commands when collision imminent |

## Configuration Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `critical_zone` | `f64` | `0.3` | Immediate stop zone distance (meters) |
| `warning_zone` | `f64` | `0.8` | Slow down zone distance (meters) |
| `monitoring_zone` | `f64` | `2.0` | Obstacle tracking zone distance (meters) |
| `robot_width` | `f64` | `0.6` | Robot width for collision envelope (meters) |
| `robot_length` | `f64` | `0.8` | Robot length for collision envelope (meters) |
| `safety_margin` | `f64` | `0.1` | Additional safety margin (meters) |
| `velocity_dependent_zones` | `bool` | `true` | Enable dynamic safety zones based on velocity |
| `min_stopping_distance` | `f64` | `0.2` | Minimum stopping distance (meters) |
| `max_deceleration` | `f64` | `2.0` | Maximum braking deceleration (m/s²) |
| `history_length` | `usize` | `5` | Number of readings for collision filtering |
| `safety_sensor_pins` | `Vec<u8>` | `[0,1,2,3]` | Digital input pins for safety sensors |
| `emergency_cooldown` | `u64` | `500` | Cooldown between emergency stops (milliseconds) |

## Message Types

### LaserScan

Lidar scan data message:

```rust
pub struct LaserScan {
    pub timestamp: u64,
    pub angle_min: f32,
    pub angle_max: f32,
    pub angle_increment: f32,
    pub range_min: f32,
    pub range_max: f32,
    pub ranges: Vec<f32>,
}
```

### EmergencyStop

Emergency stop command message:

```rust
pub struct EmergencyStop {
    pub engaged: bool,
    pub reason: String,
    pub timestamp: u64,
}

impl EmergencyStop {
    pub fn engage(reason: &str) -> Self;
    pub fn release() -> Self;
}
```

### Odometry

Robot pose and velocity message:

```rust
pub struct Odometry {
    pub timestamp: u64,
    pub pose: Pose2D,
    pub twist: Twist2D,
}

pub struct Pose2D {
    pub x: f64,
    pub y: f64,
    pub theta: f64,
}

pub struct Twist2D {
    pub linear: [f64; 2],  // [vx, vy]
    pub angular: [f64; 3], // [0, 0, omega]
}
```

### DigitalIO

Digital input/output message:

```rust
pub struct DigitalIO {
    pub timestamp: u64,
    pub pin_count: u8,
    pub pins: Vec<bool>,  // Pin states (true = triggered)
}
```

## Public API

### Construction

```rust
use horus_library::nodes::CollisionDetectorNode;

// Create with default topics
let mut detector = CollisionDetectorNode::new()?;

// Create with custom topics
let mut detector = CollisionDetectorNode::new_with_topics(
    "emergency_stop",    // emergency stop topic
    "lidar_scan",        // lidar topic
    "odom",              // odometry topic
    "digital_input"      // safety sensor topic
)?;
```

### Configuration Methods

```rust
// Configure safety zones (critical, warning, monitoring)
detector.set_safety_zones(0.5, 1.2, 3.0);

// Configure robot geometry (width, length, margin)
detector.set_robot_geometry(0.7, 0.9, 0.15);

// Configure safety sensor pins
detector.set_safety_sensor_pins(vec![0, 1, 2, 3]);

// Enable/disable velocity-dependent safety zones
detector.set_velocity_dependent_zones(true);

// Get collision status
let is_critical = detector.is_collision_imminent();
let is_warning = detector.is_warning_active();

// Get detected obstacles (world coordinates)
let obstacles = detector.get_obstacles();

// Manual emergency stop
detector.trigger_manual_emergency();

// Reset collision state (after manual intervention)
detector.reset_collision_state();

// Get detection statistics (count, minimum distance)
let (obstacle_count, min_distance) = detector.get_detection_stats();
```

## Usage Examples

### Basic Collision Detection

```rust
use horus_library::nodes::CollisionDetectorNode;
use horus_core::{Node, Runtime};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut runtime = Runtime::new()?;

    // Create collision detector with default settings
    let mut detector = CollisionDetectorNode::new()?;

    // Configure for typical indoor robot
    detector.set_safety_zones(0.3, 0.8, 2.0);
    detector.set_robot_geometry(0.6, 0.8, 0.1);

    runtime.add_node(detector);
    runtime.run()?;

    Ok(())
}
```

### Warehouse AGV with Large Safety Zones

```rust
use horus_library::nodes::CollisionDetectorNode;
use horus_core::{Node, Runtime};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut runtime = Runtime::new()?;

    // Create collision detector for high-speed AGV
    let mut detector = CollisionDetectorNode::new_with_topics(
        "agv_emergency",
        "front_lidar",
        "robot_odom",
        "safety_io"
    )?;

    // Configure for larger, faster vehicle
    detector.set_safety_zones(
        1.0,  // 1m critical zone
        2.5,  // 2.5m warning zone
        5.0   // 5m monitoring zone
    );
    detector.set_robot_geometry(1.2, 1.5, 0.2);
    detector.set_velocity_dependent_zones(true);

    runtime.add_node(detector);
    runtime.run()?;

    Ok(())
}
```

### Multi-Sensor Safety System

```rust
use horus_library::nodes::CollisionDetectorNode;
use horus_core::{Node, Runtime};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut runtime = Runtime::new()?;

    // Create collision detector with multiple safety sensors
    let mut detector = CollisionDetectorNode::new()?;

    // Configure safety zones
    detector.set_safety_zones(0.4, 1.0, 2.5);
    detector.set_robot_geometry(0.8, 1.0, 0.15);

    // Configure digital safety sensors
    // Pin 0: Front bumper
    // Pin 1: Rear bumper
    // Pin 2: Left proximity sensor
    // Pin 3: Right proximity sensor
    detector.set_safety_sensor_pins(vec![0, 1, 2, 3]);

    runtime.add_node(detector);
    runtime.run()?;

    Ok(())
}
```

### Dynamic Obstacle Monitoring

```rust
use horus_library::nodes::CollisionDetectorNode;
use horus_core::{Node, Runtime};
use std::thread;
use std::time::Duration;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut runtime = Runtime::new()?;

    // Create collision detector
    let mut detector = CollisionDetectorNode::new()?;
    detector.set_safety_zones(0.5, 1.2, 3.0);
    detector.set_robot_geometry(0.6, 0.8, 0.1);

    runtime.add_node(detector);

    // Run in background
    let handle = thread::spawn(move || {
        runtime.run()
    });

    // Monitor collision status
    loop {
        thread::sleep(Duration::from_millis(100));

        // In practice, you would access detector through shared state
        // This is simplified for demonstration

        // Check collision status
        // if detector.is_collision_imminent() {
        //     println!("CRITICAL: Collision imminent!");
        //     let (count, min_dist) = detector.get_detection_stats();
        //     println!("Obstacles: {}, Min distance: {:.2}m", count, min_dist);
        // } else if detector.is_warning_active() {
        //     println!("WARNING: Obstacles detected in warning zone");
        // }
    }

    handle.join().unwrap()
}
```

### Static Safety Zones (No Velocity Dependence)

```rust
use horus_library::nodes::CollisionDetectorNode;
use horus_core::{Node, Runtime};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut runtime = Runtime::new()?;

    // Create collision detector with fixed safety zones
    let mut detector = CollisionDetectorNode::new()?;

    // Configure static safety zones
    detector.set_safety_zones(0.4, 1.0, 2.0);
    detector.set_robot_geometry(0.5, 0.7, 0.1);

    // Disable velocity-dependent zones for predictable behavior
    detector.set_velocity_dependent_zones(false);

    runtime.add_node(detector);
    runtime.run()?;

    Ok(())
}
```

## Detection Zones

The collision detector implements three concentric safety zones around the robot:

### Critical Zone (Stop Zone)

**Distance**: Configurable (default 0.3m)

**Behavior**: Immediate emergency stop triggered

**Use case**: Prevents imminent collisions, last line of defense

**Dynamic calculation** (when velocity-dependent zones enabled):
```
stopping_distance = v² / (2 * max_deceleration) + min_stopping_distance
critical_zone = max(stopping_distance, configured_critical_zone)
```

### Warning Zone (Slow Zone)

**Distance**: Configurable (default 0.8m)

**Behavior**: Warning flag activated (no automatic action)

**Use case**: Alert higher-level planners to slow down or adjust path

**Dynamic calculation**:
```
warning_zone = max(2 * stopping_distance, configured_warning_zone)
```

### Monitoring Zone (Tracking Zone)

**Distance**: Configurable (default 2.0m)

**Behavior**: Obstacles tracked and logged

**Use case**: Long-range obstacle awareness for path planning

**Dynamic calculation**:
```
monitoring_zone = max(3 * stopping_distance, configured_monitoring_zone)
```

### Zone Selection Guidelines

| Robot Type | Critical | Warning | Monitoring | Notes |
|------------|----------|---------|------------|-------|
| Small indoor robot | 0.3m | 0.8m | 2.0m | Default settings |
| AGV/warehouse | 1.0m | 2.5m | 5.0m | High speed, large vehicle |
| Outdoor robot | 0.5m | 1.5m | 4.0m | Rough terrain, variable speed |
| Collaborative robot | 0.2m | 0.5m | 1.5m | Close human interaction |
| Cleaning robot | 0.25m | 0.6m | 1.5m | Frequent obstacles |

## Response Strategies

### Emergency Stop (Critical Zone)

When an obstacle enters the critical zone:

1. **Collision detection**: Lidar range < critical_zone
2. **Filtering**: Majority consensus over last 5 readings (reduces false positives)
3. **Emergency trigger**: EmergencyStop message published
4. **Cooldown**: 500ms cooldown prevents message spam
5. **State update**: `collision_imminent` flag set to true

**Integration pattern**:
```rust
// In motion controller
if let Some(emergency) = emergency_subscriber.recv(None) {
    if emergency.engaged {
        // Stop all motors immediately
        motor_controller.stop_all();
        eprintln!("Emergency stop: {}", emergency.reason);
    }
}
```

### Warning Response (Warning Zone)

When an obstacle enters the warning zone:

1. **Warning flag**: `warning_active` set to true
2. **No automatic action**: Higher-level planner must respond
3. **Obstacle tracking**: Obstacle positions available via `get_obstacles()`

**Integration pattern**:
```rust
// In path planner
if detector.is_warning_active() {
    let obstacles = detector.get_obstacles();

    // Reduce velocity
    target_velocity *= 0.5;

    // Or trigger avoidance behavior
    planner.avoid_obstacles(obstacles);
}
```

### Avoidance Strategy (Monitoring Zone)

When an obstacle enters the monitoring zone:

1. **Passive tracking**: Obstacles logged and tracked
2. **Predictive planning**: Path planner can use obstacle data
3. **No immediate action**: Normal operation continues

**Integration pattern**:
```rust
// In predictive path planner
let (obstacle_count, min_distance) = detector.get_detection_stats();

if obstacle_count > 0 {
    let obstacles = detector.get_obstacles();

    // Update cost map
    costmap.update_obstacles(obstacles);

    // Replan if necessary
    if min_distance < 3.0 {
        planner.replan_path();
    }
}
```

## Safety Sensor Integration

The collision detector supports digital safety sensors (bumpers, proximity sensors):

### Hardware Configuration

```rust
// Configure safety sensor pins
detector.set_safety_sensor_pins(vec![
    0,  // Front bumper
    1,  // Rear bumper
    2,  // Left proximity sensor
    3,  // Right proximity sensor
]);
```

### Sensor Logic

- **Normally Open (NO)**: Sensors trigger when closed (pin state = true)
- **Immediate response**: Any triggered sensor causes emergency stop
- **No filtering**: Direct hardware triggers bypass software filtering
- **Priority**: Safety sensors override all other detection logic

### Physical Sensor Examples

| Sensor Type | Typical Range | Use Case |
|-------------|---------------|----------|
| Mechanical bumper | Contact (0mm) | Last resort physical contact |
| IR proximity | 5-30cm | Close-range pre-contact detection |
| Ultrasonic | 2-400cm | Medium-range supplementary detection |
| Capacitive | 1-10cm | Non-contact human detection |

## Collision Envelope

The collision detector uses a rectangular collision envelope based on robot geometry:

### Envelope Calculation

```rust
envelope_width = robot_width + 2 * safety_margin
envelope_length = robot_length + 2 * safety_margin
```

### Coordinate Transformation

Obstacles are checked against the robot's local coordinate frame:

1. **World to robot frame**: Rotate obstacle coordinates by robot heading
2. **Bounding box check**: Test if obstacle falls within envelope
3. **Forward cone check**: Additional 60° forward cone for angular obstacles

### Example Configuration

```rust
// For a 60cm x 80cm robot with 10cm margin
detector.set_robot_geometry(
    0.6,  // width
    0.8,  // length
    0.1   // margin
);

// Results in 70cm x 90cm collision envelope
```

## Troubleshooting

### Issue: Frequent false positives

**Cause**: Sensor noise, reflections, or overly sensitive zones

**Solution**:
```rust
// Increase critical zone threshold
detector.set_safety_zones(0.5, 1.0, 2.0);  // Was 0.3, 0.8, 2.0

// Increase filter history (more smoothing)
// Note: history_length is set in constructor, requires rebuild

// Reduce safety margin
detector.set_robot_geometry(0.6, 0.8, 0.05);  // Reduced margin
```

### Issue: Missed detections at high speed

**Cause**: Velocity-dependent zones disabled or insufficient max_deceleration

**Solution**:
```rust
// Enable velocity-dependent zones
detector.set_velocity_dependent_zones(true);

// Ensure max_deceleration matches robot capabilities
// Note: max_deceleration is set in constructor
// Typical values: 1.5-3.0 m/s² for wheeled robots

// Increase base safety zones
detector.set_safety_zones(0.5, 1.2, 3.0);
```

### Issue: Emergency stops too sensitive

**Cause**: Critical zone too large or filtering insufficient

**Solution**:
```rust
// Reduce critical zone (with caution!)
detector.set_safety_zones(0.25, 0.8, 2.0);

// Increase emergency cooldown
// Note: emergency_cooldown is set in constructor

// Check for lidar noise or misalignment
// Verify lidar mounting and calibration
```

### Issue: Collisions detected when none exist

**Cause**: Lidar detecting robot's own body or mounting hardware

**Solution**:
```rust
// Adjust robot geometry to match actual dimensions
detector.set_robot_geometry(0.55, 0.75, 0.1);  // Slightly smaller

// Check lidar mounting position and angle
// Ensure lidar doesn't see robot body parts

// Filter out specific angle ranges in lidar data (upstream processing)
```

### Issue: Obstacles not tracked accurately

**Cause**: Odometry drift or incorrect coordinate transformation

**Solution**:
```rust
// Verify odometry data is publishing correctly
// Check robot pose estimation accuracy

// Ensure lidar and odometry coordinate frames aligned

// Consider implementing obstacle filtering/clustering
// (upstream lidar processing)
```

### Issue: Safety sensors not triggering

**Cause**: Incorrect pin configuration or sensor wiring

**Solution**:
```rust
// Verify pin assignments match hardware
detector.set_safety_sensor_pins(vec![0, 1, 2, 3]);

// Test digital inputs independently
// Use DigitalIO node to verify sensor signals

// Check sensor power and wiring
// Verify normally-open (NO) sensor configuration
```

## Integration with Motion Planning

### Architecture Overview

```
Lidar  CollisionDetectorNode  EmergencyStop  MotorController
                                 
      Odometry              WarningFlags  PathPlanner
                                            
                                      MotionCommands
```

### Layer 1: Emergency Response (Hardware Safety)

**Priority**: Highest (hardware-level)

**Components**: Safety sensors  CollisionDetectorNode  EmergencyStop

**Response time**: < 50ms

```rust
// Emergency stop handler (runs in motor controller)
if let Some(emergency) = emergency_subscriber.recv(None) {
    if emergency.engaged {
        // Immediate hardware stop
        hardware.stop_all_motors();
        state.emergency_active = true;
    }
}
```

### Layer 2: Reactive Avoidance (Software Safety)

**Priority**: High (software-level)

**Components**: Critical zone detection  Velocity reduction

**Response time**: < 200ms

```rust
// In motion controller or velocity smoother
if detector.is_collision_imminent() {
    // Decelerate rapidly
    cmd_vel.linear *= 0.1;  // 10% speed
    cmd_vel.angular *= 0.5;  // 50% rotation
} else if detector.is_warning_active() {
    // Slow down proportionally
    cmd_vel.linear *= 0.5;  // 50% speed
}
```

### Layer 3: Predictive Planning (Path Planning)

**Priority**: Medium (planning-level)

**Components**: Monitoring zone  Cost map update  Path replanning

**Response time**: < 1s

```rust
// In global path planner
let obstacles = detector.get_obstacles();

// Update costmap with detected obstacles
for (x, y) in obstacles {
    costmap.add_obstacle(x, y, inflation_radius);
}

// Replan if path blocked
if costmap.is_path_blocked(&current_path) {
    current_path = planner.replan(start, goal, &costmap);
}
```

### Complete Integration Example

```rust
use horus_library::nodes::{CollisionDetectorNode, DifferentialDriveNode};
use horus_core::{Node, Runtime, Hub};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut runtime = Runtime::new()?;

    // Create collision detector
    let mut detector = CollisionDetectorNode::new_with_topics(
        "emergency_stop",
        "lidar_scan",
        "odom",
        "digital_input"
    )?;
    detector.set_safety_zones(0.4, 1.0, 2.5);
    detector.set_robot_geometry(0.6, 0.8, 0.1);
    detector.set_velocity_dependent_zones(true);

    // Create motion controller (subscribes to emergency_stop)
    let mut drive = DifferentialDriveNode::new_with_topics(
        "cmd_vel",
        "odom",
        "motor_commands",
        Some("emergency_stop")  // Subscribe to emergency stop
    )?;

    // Add nodes to runtime
    runtime.add_node(detector);
    runtime.add_node(drive);

    // Run system
    runtime.run()?;

    Ok(())
}
```

### Best Practices

1. **Layered Safety**:
   - Layer 1 (hardware): Safety sensors + emergency stop
   - Layer 2 (reactive): Collision detector + velocity limiting
   - Layer 3 (predictive): Path planning + obstacle avoidance

2. **Priority Ordering**:
   - Emergency stop (highest priority, no override)
   - Critical zone detection (high priority, can override commands)
   - Warning zone (medium priority, influence planning)
   - Monitoring zone (low priority, informational)

3. **Fail-Safe Design**:
   - Default to safe state (stopped) on loss of sensor data
   - Hardware e-stop independent of software
   - Redundant sensors (lidar + safety sensors)

4. **Performance Optimization**:
   - Run collision detector at lidar rate (10-40Hz typical)
   - Use non-blocking message reads
   - Filter obstacles before passing to planner
   - Cache dynamic zone calculations

5. **Testing Strategy**:
   - Unit test: Individual zone calculations
   - Integration test: Full sensor  emergency pipeline
   - Safety test: Verify emergency stop response time
   - Field test: Real-world obstacle scenarios

## Performance Considerations

### Update Rate

The collision detector runs on every tick and processes:
- Lidar scans (typically 10-40Hz)
- Odometry updates (50-200Hz)
- Safety sensor readings (polled every tick)

**Recommended tick rates**:
- Indoor robot: 20-30Hz (30-50ms tick)
- Warehouse AGV: 30-50Hz (20-30ms tick)
- High-speed vehicle: 50-100Hz (10-20ms tick)

### CPU Usage

**Typical load per tick**:
- Lidar processing: O(n) where n = number of lidar points (typically 360-1080)
- Collision filtering: O(m) where m = history_length (default 5)
- Safety sensor check: O(k) where k = number of sensor pins (default 4)

**Total**: Lightweight, typically < 1ms per tick on modern hardware

### Memory Usage

**Fixed allocations**:
- Node structure: ~200 bytes
- Collision history: ~40 bytes (5 x 8 bytes)
- Safety sensor pins: ~32 bytes (4 x 8 bytes)

**Dynamic allocations**:
- Obstacles vector: ~16 bytes per obstacle (typical: 0-20 obstacles)

**Total**: Small footprint, < 1KB typical

### Optimization Tips

```rust
// For high-frequency lidar (> 1000 points), downsample
// Upstream in lidar processing node

// For multiple collision detectors, share odometry subscription
// Use single odometry node with multiple subscribers

// Reduce history length if CPU-constrained
// Trade-off: less filtering, more false positives

// Disable velocity-dependent zones if not needed
detector.set_velocity_dependent_zones(false);
```

## Related Nodes

- **LaserScanNode**: Provides lidar data for obstacle detection
- **OdometryNode**: Provides robot pose and velocity
- **DifferentialDriveNode**: Consumes emergency stop messages
- **PathPlannerNode**: Uses obstacle data for navigation
- **DigitalIONode**: Provides safety sensor inputs

## See Also

- [Collision Avoidance Algorithms](https://en.wikipedia.org/wiki/Collision_avoidance)
- [Dynamic Window Approach](https://en.wikipedia.org/wiki/Dynamic_window_approach)
- [Robot Safety Standards (ISO 13849)](https://www.iso.org/standard/69883.html)
- [Lidar-based Navigation](https://en.wikipedia.org/wiki/Lidar)
