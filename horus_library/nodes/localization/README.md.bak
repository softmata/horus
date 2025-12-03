# Localization Node

Robot position estimation using Extended Kalman Filter (EKF) sensor fusion with odometry, IMU, and lidar data.

## Overview

The Localization Node implements an Extended Kalman Filter (EKF) for accurate robot pose estimation by fusing multiple sensor sources. It combines odometry data, IMU measurements, and lidar-based landmark observations to maintain a real-time estimate of the robot's position (x, y), orientation (theta), and velocities (vx, vy, omega) within a known coordinate frame.

The node uses a 6-dimensional state vector [x, y, theta, vx, vy, omega] and maintains uncertainty estimates through a state covariance matrix, providing both pose estimates and confidence metrics.

## Architecture

**This node is a thin wrapper** around the pure algorithms in `horus_library/algorithms/`:

- **`algorithms::ekf::EKF`** - Extended Kalman Filter for 2D robot localization
- **`algorithms::sensor_fusion::SensorFusion`** - Multi-sensor fusion with variance weighting

The node handles:
- Topic subscription/publishing (Hub I/O)
- Sensor data reception (odometry, IMU, lidar)
- Landmark detection and association
- Pose publishing with covariances
- Frame ID management

The algorithms handle:
- EKF prediction and update steps
- State covariance propagation
- Variance-weighted sensor fusion (odometry + IMU angular velocity)
- Pure state estimation logic

This separation enables algorithm reuse and independent testing of estimation logic.

## Topics

### Subscribers

| Topic | Type | Description |
|-------|------|-------------|
| `odom` | `Odometry` | Wheel odometry measurements for position/velocity estimation |
| `imu` | `Imu` | Inertial measurement data (accelerations, angular velocities) |
| `lidar_scan` | `LaserScan` | Laser range data for landmark-based correction |

### Publishers

| Topic | Type | Description |
|-------|------|-------------|
| `pose` | `Odometry` | Localized robot pose with covariance estimates |

## Configuration Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| **Process Noise** |  |  | Motion model uncertainty |
| `process_noise[0][0]` | `f64` | `0.1` | Position x process noise (m²) |
| `process_noise[1][1]` | `f64` | `0.1` | Position y process noise (m²) |
| `process_noise[2][2]` | `f64` | `0.05` | Orientation process noise (rad²) |
| `process_noise[3][3]` | `f64` | `0.2` | Velocity x process noise (m²/s²) |
| `process_noise[4][4]` | `f64` | `0.2` | Velocity y process noise (m²/s²) |
| `process_noise[5][5]` | `f64` | `0.1` | Angular velocity process noise (rad²/s²) |
| **Odometry Noise** |  |  | Odometry measurement uncertainty |
| `odometry_noise[0][0]` | `f64` | `0.05` | Position x measurement noise (m²) |
| `odometry_noise[1][1]` | `f64` | `0.05` | Position y measurement noise (m²) |
| `odometry_noise[2][2]` | `f64` | `0.02` | Orientation measurement noise (rad²) |
| **IMU Noise** |  |  | IMU measurement uncertainty |
| `imu_noise[0][0]` | `f64` | `0.1` | Linear acceleration x noise (m²/s⁴) |
| `imu_noise[1][1]` | `f64` | `0.1` | Linear acceleration y noise (m²/s⁴) |
| `imu_noise[2][2]` | `f64` | `0.05` | Angular velocity noise (rad²/s²) |
| **Frame Configuration** |  |  | Coordinate frame settings |
| `frame_id` | `String` | `"map"` | Reference coordinate frame |
| `child_frame_id` | `String` | `"base_link"` | Robot body frame |
| **Landmark Settings** |  |  | Map-based localization |
| `landmark_detection_range` | `f64` | `10.0` | Maximum landmark detection distance (m) |

## Message Types

### Odometry

Robot pose and velocity message:

```rust
pub struct Odometry {
    pub timestamp: u64,
    pub frame_id: [u8; 32],
    pub child_frame_id: [u8; 32],
    pub pose: Pose2D,
    pub twist: Twist2D,
    pub pose_covariance: [f64; 36],    // 6x6 covariance matrix
    pub twist_covariance: [f64; 36],   // 6x6 covariance matrix
}
```

### Pose2D

2D pose representation:

```rust
pub struct Pose2D {
    pub x: f64,        // Position x (m)
    pub y: f64,        // Position y (m)
    pub theta: f64,    // Orientation (rad)
}
```

### Twist2D

2D velocity representation:

```rust
pub struct Twist2D {
    pub linear: [f64; 3],     // [vx, vy, vz] (m/s)
    pub angular: [f64; 3],    // [wx, wy, wz] (rad/s)
}
```

### Imu

Inertial measurement data:

```rust
pub struct Imu {
    pub timestamp: u64,
    pub linear_acceleration: [f64; 3],  // [ax, ay, az] (m/s²)
    pub angular_velocity: [f64; 3],     // [wx, wy, wz] (rad/s)
    pub orientation: [f64; 4],          // Quaternion [x, y, z, w]
}
```

### LaserScan

Laser range finder data:

```rust
pub struct LaserScan {
    pub timestamp: u64,
    pub frame_id: [u8; 32],
    pub angle_min: f32,           // Start angle (rad)
    pub angle_max: f32,           // End angle (rad)
    pub angle_increment: f32,     // Angular resolution (rad)
    pub range_min: f32,           // Minimum valid range (m)
    pub range_max: f32,           // Maximum valid range (m)
    pub ranges: Vec<f32>,         // Range measurements (m)
    pub intensities: Vec<f32>,    // Intensity values
}
```

## Public API

### Construction

```rust
use horus_library::nodes::LocalizationNode;

// Create with default topics
let mut localizer = LocalizationNode::new()?;

// Create with custom topics
let mut localizer = LocalizationNode::new_with_topics(
    "robot_pose",       // pose output topic
    "wheel_odom",       // odometry input topic
    "imu_data",         // IMU input topic
    "scan"              // lidar input topic
)?;
```

### Configuration Methods

```rust
// Set initial robot pose
localizer.set_initial_pose(2.0, 3.0, 0.785);  // x=2m, y=3m, theta=45°

// Configure coordinate frames
localizer.set_frame_ids("map", "base_footprint");

// Add known landmarks for map-based correction
localizer.add_landmark(5.0, 5.0);    // Landmark at (5, 5)
localizer.add_landmark(10.0, 2.0);   // Landmark at (10, 2)

// Reset localization (for relocalization)
localizer.reset();
```

### State Query Methods

```rust
// Get current pose estimate
let (x, y, theta) = localizer.get_pose();

// Get current velocity estimate
let (vx, vy, omega) = localizer.get_velocity();

// Get position uncertainty (standard deviation)
let uncertainty = localizer.get_position_uncertainty();  // meters

// Check if localization has converged
let is_converged = localizer.is_converged();  // true if uncertainty < 0.3m
```

## Usage Examples

### Basic EKF Localization

```rust
use horus_library::nodes::LocalizationNode;
use horus_core::{Node, Runtime};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut runtime = Runtime::new()?;

    // Create localization node with default EKF configuration
    let mut localizer = LocalizationNode::new()?;

    // Set initial pose (required for localization)
    localizer.set_initial_pose(0.0, 0.0, 0.0);

    // Configure frames
    localizer.set_frame_ids("map", "base_link");

    runtime.add_node(localizer);
    runtime.run()?;

    Ok(())
}
```

### Warehouse Robot with Known Landmarks

```rust
use horus_library::nodes::LocalizationNode;
use horus_core::{Node, Runtime};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut runtime = Runtime::new()?;

    // Create localization node
    let mut localizer = LocalizationNode::new_with_topics(
        "global_pose",
        "wheel_odom",
        "imu",
        "front_lidar"
    )?;

    // Set starting position at loading dock
    localizer.set_initial_pose(1.0, 1.0, 0.0);

    // Add warehouse landmarks (corner reflectors)
    localizer.add_landmark(0.0, 0.0);      // Southwest corner
    localizer.add_landmark(20.0, 0.0);     // Southeast corner
    localizer.add_landmark(20.0, 15.0);    // Northeast corner
    localizer.add_landmark(0.0, 15.0);     // Northwest corner
    localizer.add_landmark(10.0, 7.5);     // Central pillar

    runtime.add_node(localizer);
    runtime.run()?;

    Ok(())
}
```

### Multi-Sensor Fusion with High-Rate IMU

```rust
use horus_library::nodes::LocalizationNode;
use horus_core::{Node, Runtime};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut runtime = Runtime::new()?;

    // Create localization node with all sensors
    let mut localizer = LocalizationNode::new_with_topics(
        "fused_pose",
        "odom",
        "imu/data",
        "scan"
    )?;

    // Initialize at known starting location
    localizer.set_initial_pose(5.0, 5.0, 1.57);  // 90° orientation

    // Configure for indoor navigation
    localizer.set_frame_ids("map", "base_footprint");

    runtime.add_node(localizer);
    runtime.run()?;

    Ok(())
}
```

### AMCL-Style Particle Filter Integration

```rust
use horus_library::nodes::LocalizationNode;
use horus_core::{Node, Runtime};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut runtime = Runtime::new()?;

    // EKF localizer for smooth pose estimation
    let mut localizer = LocalizationNode::new()?;

    // Don't set initial pose - will be initialized from AMCL
    // AMCL particle filter runs separately and provides initial guess

    // Add map landmarks for correction
    // These would typically be extracted from a map file
    for landmark in load_map_landmarks()? {
        localizer.add_landmark(landmark.0, landmark.1);
    }

    runtime.add_node(localizer);
    runtime.run()?;

    Ok(())
}

fn load_map_landmarks() -> Result<Vec<(f64, f64)>, Box<dyn std::error::Error>> {
    // Load landmarks from map file
    Ok(vec![(1.0, 1.0), (5.0, 5.0), (10.0, 3.0)])
}
```

### Relocalization After Kidnapped Robot

```rust
use horus_library::nodes::LocalizationNode;
use horus_core::{Node, Runtime};
use std::thread;
use std::time::Duration;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut runtime = Runtime::new()?;

    let mut localizer = LocalizationNode::new()?;
    localizer.set_initial_pose(0.0, 0.0, 0.0);

    // Monitor localization quality
    thread::spawn(move || {
        loop {
            thread::sleep(Duration::from_secs(1));

            let uncertainty = localizer.get_position_uncertainty();

            // Detect kidnapped robot scenario
            if uncertainty > 2.0 {  // More than 2m uncertainty
                eprintln!("Localization lost! Triggering relocalization...");
                localizer.reset();

                // Trigger global localization or wait for initial pose estimate
                // from external system (e.g., AMCL, manual initialization)
            }

            if localizer.is_converged() {
                eprintln!("Localization converged!");
            }
        }
    });

    runtime.run()?;
    Ok(())
}
```

## Sensor Fusion Methods

### EKF Prediction Step

The prediction step uses a kinematic motion model:

```
x_{k+1} = x_k + vx * dt
y_{k+1} = y_k + vy * dt
theta_{k+1} = theta_k + omega * dt

P_{k+1} = P_k + Q * dt
```

Where:
- `P` is the state covariance matrix
- `Q` is the process noise matrix
- `dt` is the time step

### Odometry Update

Odometry provides direct measurements of pose:

```
Innovation = z_odom - h(x)
where z_odom = [x_odom, y_odom, theta_odom]

State update:
x = x + K * Innovation
P = (1 - K) * P
```

The Kalman gain `K` determines the trust between prediction and measurement.

### IMU Fusion

IMU data refines angular velocity and validates linear accelerations:

```
omega_fused = (1 - w) * omega_odom + w * omega_imu
where w = 0.3 (IMU weight)

Velocity correction:
vx += ax * dt * correction_factor
vy += ay * dt * correction_factor
```

### Landmark-Based Correction

Lidar scans are matched to known landmarks:

```
For each detected feature:
  1. Project feature to global frame
  2. Associate with nearest landmark (within 1m threshold)
  3. Compute position error
  4. Apply weighted correction to robot pose
```

This provides absolute position corrections to prevent drift.

## Map-Based Localization

### Landmark Map Format

Landmarks are stored as (x, y) coordinates in the global frame:

```rust
// Manually add landmarks
localizer.add_landmark(5.0, 5.0);
localizer.add_landmark(10.0, 2.0);

// Or load from a map file
fn load_landmarks_from_map(map_file: &str) -> Vec<(f64, f64)> {
    // Parse map file and extract landmark positions
    // Could be corners, doorways, reflectors, etc.
    vec![]
}

for (x, y) in load_landmarks_from_map("warehouse.map") {
    localizer.add_landmark(x, y);
}
```

### Landmark Detection

The node detects landmarks from lidar scans:

1. **Range filtering**: Only consider readings between 0.5m and detection_range
2. **Feature extraction**: Convert polar to Cartesian coordinates
3. **Data association**: Match detected features to known landmarks
4. **Correction**: Apply weighted position correction

### Map Integration with SLAM

For SLAM integration:

```rust
// EKF localizer uses landmarks from SLAM map
let mut localizer = LocalizationNode::new()?;

// SLAM system publishes updated landmarks
// Subscribe to landmark updates and add to localizer
for landmark in slam_landmarks {
    localizer.add_landmark(landmark.x, landmark.y);
}
```

### Grid Map Localization

For occupancy grid maps, extract landmarks at:
- Corners and edges
- Doorways and openings
- Distinctive features (pillars, furniture)

```rust
fn extract_landmarks_from_grid(grid: &OccupancyGrid) -> Vec<(f64, f64)> {
    let mut landmarks = Vec::new();

    // Find corners using corner detection
    // Find edges using Canny or similar
    // Convert to global coordinates

    landmarks
}
```

## Troubleshooting

### Issue: Poor Localization Accuracy

**Symptoms**: Robot pose drifts over time, large uncertainty values

**Causes**:
- Wheel slippage affecting odometry
- Incorrect noise parameters
- Insufficient landmarks
- Sensor calibration issues

**Solutions**:

```rust
// Increase odometry noise if wheels slip frequently
localizer.odometry_noise[0][0] = 0.1;  // Increase from 0.05
localizer.odometry_noise[1][1] = 0.1;

// Decrease process noise for more trust in motion model
localizer.process_noise[0][0] = 0.05;  // Decrease from 0.1

// Add more landmarks in the environment
localizer.add_landmark(x, y);

// Verify sensor calibration
// - Check IMU axes alignment
// - Verify lidar mounting position
// - Calibrate wheel encoders
```

### Issue: Localization Diverges

**Symptoms**: Pose estimate jumps erratically, never converges

**Causes**:
- Incorrect initial pose
- Bad landmark associations
- Timestamp synchronization issues
- Conflicting sensor data

**Solutions**:

```rust
// Reset and set accurate initial pose
localizer.reset();
localizer.set_initial_pose(known_x, known_y, known_theta);

// Check sensor timestamps are synchronized
// Verify all sensors use same clock source

// Increase measurement noise to reduce sensor trust
localizer.odometry_noise[0][0] = 0.1;
localizer.imu_noise[2][2] = 0.1;

// Remove or correct bad landmarks
localizer = LocalizationNode::new()?;
// Re-add only verified landmarks
```

### Issue: Kidnapped Robot Problem

**Symptoms**: Robot is physically moved, localization doesn't recover

**Causes**:
- EKF cannot handle multi-modal distributions
- No global relocalization mechanism

**Solutions**:

```rust
// Implement uncertainty monitoring
if localizer.get_position_uncertainty() > 2.0 {
    // Trigger global relocalization
    localizer.reset();

    // Use particle filter (AMCL) for global localization
    // Once converged, reinitialize EKF
    let (x, y, theta) = amcl_global_localization();
    localizer.set_initial_pose(x, y, theta);
}

// Or implement periodic convergence checks
fn monitor_localization(localizer: &LocalizationNode) -> bool {
    localizer.is_converged() &&
    localizer.get_position_uncertainty() < 0.5
}
```

### Issue: High Computational Load

**Symptoms**: Node tick takes too long, delays in pose updates

**Causes**:
- Too many landmarks
- High-rate sensor data
- Large covariance matrices

**Solutions**:

```rust
// Reduce landmark count (keep only distinctive features)
// Use spatial indexing for landmark lookup

// Reduce sensor update rates
// - Odometry: 50-100 Hz sufficient
// - IMU: 100 Hz sufficient
// - Lidar: 10-20 Hz sufficient

// Run localization node at lower rate
runtime.set_node_rate("LocalizationNode", 50.0)?;  // 50 Hz
```

### Issue: Jumps in Pose Estimate

**Symptoms**: Sudden discontinuous changes in estimated pose

**Causes**:
- Bad landmark associations
- Outlier sensor readings
- Incorrect coordinate frame transforms

**Solutions**:

```rust
// Tighten landmark association threshold
// Modify in source: distance < 0.5  (from 1.0)

// Implement outlier rejection
// Check innovation magnitude before update

// Verify coordinate frame conventions match
localizer.set_frame_ids("map", "base_link");
// Ensure all sensors publish in correct frames
```

## Integration with SLAM and Navigation

### SLAM Integration

The localization node can work alongside SLAM systems:

```rust
// SLAM builds map and publishes landmarks
// Localization uses landmarks for pose estimation

let mut slam_node = SlamNode::new()?;
let mut localizer = LocalizationNode::new()?;

// SLAM publishes to /landmarks
// Localization subscribes and adds landmarks
// Localization publishes to /pose
// SLAM uses /pose for loop closure
```

**Workflow**:
1. SLAM explores environment and builds map
2. SLAM extracts and publishes landmarks
3. Localization subscribes to landmarks and updates internal map
4. Localization publishes refined pose back to SLAM
5. SLAM uses refined pose for improved mapping

### Navigation Stack Integration

```rust
// Navigation stack architecture:
//
// Sensors (Lidar, IMU, Odom)
//     |
//     v
// LocalizationNode -> Pose estimate
//     |
//     v
// PathPlannerNode -> Planned path
//     |
//     v
// TrajectoryControllerNode -> Velocity commands
//     |
//     v
// Motors

let mut localizer = LocalizationNode::new_with_topics(
    "localized_pose",
    "odom",
    "imu",
    "scan"
)?;

let mut planner = PathPlannerNode::new_with_topics(
    "localized_pose",    // Subscribes to localization
    "goal",
    "path"
)?;

let mut controller = TrajectoryControllerNode::new_with_topics(
    "path",
    "localized_pose",
    "cmd_vel"
)?;

runtime.add_node(localizer);
runtime.add_node(planner);
runtime.add_node(controller);
```

### Map Server Integration

```rust
// Load static map and extract landmarks
let map = load_occupancy_grid("warehouse.pgm")?;
let landmarks = extract_landmarks_from_map(&map);

let mut localizer = LocalizationNode::new()?;

// Add all extracted landmarks
for (x, y) in landmarks {
    localizer.add_landmark(x, y);
}

// Publish map for visualization and planning
let map_publisher = Hub::new("map")?;
map_publisher.send(map, None)?;
```

### AMCL Hybrid Approach

Combine particle filter (AMCL) for global localization with EKF for tracking:

```rust
// Particle filter for global localization
let mut amcl = AmclNode::new()?;

// EKF for precise tracking once localized
let mut ekf = LocalizationNode::new()?;

// State machine
enum LocalizationMode {
    Global,      // Using AMCL
    Tracking,    // Using EKF
}

let mut mode = LocalizationMode::Global;

loop {
    match mode {
        LocalizationMode::Global => {
            // Run AMCL until converged
            amcl.tick(None);

            if amcl.is_converged() {
                let (x, y, theta) = amcl.get_pose();
                ekf.set_initial_pose(x, y, theta);
                mode = LocalizationMode::Tracking;
            }
        }
        LocalizationMode::Tracking => {
            // Run EKF for smooth tracking
            ekf.tick(None);

            // Check if kidnapped (high uncertainty)
            if ekf.get_position_uncertainty() > 2.0 {
                ekf.reset();
                mode = LocalizationMode::Global;
            }
        }
    }
}
```

## Performance Considerations

### Update Rates

Recommended sensor rates for optimal performance:

| Sensor | Rate | Notes |
|--------|------|-------|
| Odometry | 50-100 Hz | Primary pose source |
| IMU | 100-200 Hz | High-rate for orientation |
| Lidar | 10-20 Hz | Landmark detection |
| Localization output | 50 Hz | Sufficient for navigation |

### CPU Usage

- **Prediction step**: ~10 µs (matrix operations)
- **Odometry update**: ~20 µs (Kalman update)
- **IMU update**: ~10 µs (weighted fusion)
- **Landmark update**: ~100 µs per scan (feature matching)

Total: ~140 µs per tick (50 Hz = 0.7% CPU on modern processor)

### Memory Usage

- **State vector**: 48 bytes (6 × f64)
- **Covariance matrix**: 288 bytes (6×6 × f64)
- **Noise matrices**: 576 bytes (3 matrices)
- **Landmarks**: 16 bytes per landmark
- **Hubs**: ~1 KB

Total: ~2 KB + 16 bytes per landmark

### Latency

- **Sensor to pose output**: 1-2 ticks (~20-40 ms at 50 Hz)
- **End-to-end localization**: ~50 ms including sensor acquisition

## Algorithm Details

### Extended Kalman Filter (EKF)

The EKF operates in two phases:

**Prediction Phase**:
```
State prediction:
  x̂⁻ = f(x̂, u, Δt)  [Motion model]

Covariance prediction:
  P⁻ = F·P·Fᵀ + Q
```

**Update Phase**:
```
Innovation:
  y = z - h(x̂⁻)  [Measurement residual]

Kalman gain:
  K = P⁻·Hᵀ·(H·P⁻·Hᵀ + R)⁻¹

State update:
  x̂ = x̂⁻ + K·y

Covariance update:
  P = (I - K·H)·P⁻
```

### State Vector

The 6D state vector represents:
```
x = [x, y, θ, vₓ, vᵧ, ω]ᵀ

where:
  x, y    : Position in global frame (m)
  θ       : Orientation (rad)
  vₓ, vᵧ  : Linear velocities (m/s)
  ω       : Angular velocity (rad/s)
```

### Motion Model

Simple kinematic model:
```
x(k+1) = x(k) + vₓ·Δt
y(k+1) = y(k) + vᵧ·Δt
θ(k+1) = θ(k) + ω·Δt
vₓ(k+1) = vₓ(k)
vᵧ(k+1) = vᵧ(k)
ω(k+1) = ω(k)
```

## Related Nodes

- **OdometryNode**: Provides wheel odometry measurements
- **ImuNode**: Provides inertial measurements
- **LidarNode**: Provides laser scan data
- **SlamNode**: Builds maps using localization output
- **PathPlannerNode**: Uses pose for navigation planning
- **DifferentialDriveNode**: Provides odometry, consumes velocity commands

## See Also

- [Extended Kalman Filter](https://en.wikipedia.org/wiki/Extended_Kalman_filter)
- [Sensor Fusion](https://en.wikipedia.org/wiki/Sensor_fusion)
- [AMCL - Adaptive Monte Carlo Localization](http://wiki.ros.org/amcl)
- [Probabilistic Robotics (Thrun, Burgard, Fox)](http://www.probabilistic-robotics.org/)
- [Robot Localization Package](http://docs.ros.org/en/melodic/api/robot_localization/html/index.html)
