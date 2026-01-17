# Odometry Node

Dead reckoning position tracking from wheel encoders and velocity commands for autonomous navigation and localization.

## Overview

The Odometry Node computes robot pose (x, y, theta) and velocity through integration of wheel encoder feedback or velocity commands. It supports multiple kinematic models for different robot platforms and provides essential state estimation for navigation, path following, and localization systems.

Supports differential drive, mecanum, ackermann steering, and skid steer platforms.

Key features:
- Multiple kinematic models (differential, mecanum, ackermann, skid steer)
- Encoder-based dead reckoning with configurable resolution
- Velocity integration for dead reckoning fallback
- Real-time pose and velocity estimation
- Covariance tracking for uncertainty estimation
- Configurable coordinate frames (odom, base_link)
- Reset and calibration support
- High-frequency updates (up to 100 Hz)

## Topics

### Subscribers

| Topic | Type | Description |
|-------|------|-------------|
| `encoder.left` | `i64` | Left wheel encoder tick count |
| `encoder.right` | `i64` | Right wheel encoder tick count |
| `cmd_vel` | `Twist` | Velocity commands for dead reckoning fallback |

### Publishers

| Topic | Type | Description |
|-------|------|-------------|
| `odom` | `Odometry` | Robot pose and velocity with covariance estimates |

## Configuration Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `wheel_base` | `f64` | `0.5` | Distance between left and right wheels (m) |
| `wheel_radius` | `f64` | `0.05` | Wheel radius (m) |
| `track_width` | `f64` | `wheel_base` | Distance between left and right tracks (m) |
| `encoder_resolution` | `u32` | `4096` | Encoder ticks per wheel revolution |
| `update_rate` | `f64` | `50.0` | Odometry publishing frequency in Hz (0.1-1000) |
| `position_variance` | `f64` | `0.01` | Position covariance (m^2) - default 1cm std dev |
| `orientation_variance` | `f64` | `0.001` | Orientation covariance (rad^2) - default ~1.8 degree std dev |
| `frame_id` | `String` | `"odom"` | Reference frame for pose measurements |
| `child_frame_id` | `String` | `"base_link"` | Robot body frame for velocity measurements |
| `use_encoder_input` | `bool` | `true` | Enable encoder-based odometry |
| `use_velocity_input` | `bool` | `false` | Enable velocity-based odometry |

## Message Types

### Odometry

Comprehensive odometry message with pose, velocity, and covariance:

```rust
pub struct Odometry {
    pub pose: Pose2D,                     // Current position and orientation
    pub twist: Twist,                     // Current velocity (linear and angular)
    pub pose_covariance: [f64; 36],       // 6x6 pose uncertainty matrix
    pub twist_covariance: [f64; 36],      // 6x6 velocity uncertainty matrix
    pub frame_id: [u8; 32],               // Reference frame (e.g., "odom")
    pub child_frame_id: [u8; 32],         // Body frame (e.g., "base_link")
    pub timestamp: u64,                   // Measurement time (ns since epoch)
}
```

**Helper Methods**:
- `Odometry::new()` - Create new odometry message
- `odometry.set_frames(frame, child_frame)` - Set coordinate frames
- `odometry.update(pose, twist)` - Update pose and velocity
- `odometry.is_valid()` - Check if values are finite

### Pose2D

2D position and orientation:

```rust
pub struct Pose2D {
    pub x: f64,           // X position in meters
    pub y: f64,           // Y position in meters
    pub theta: f64,       // Orientation in radians
    pub timestamp: u64,   // Timestamp (ns since epoch)
}
```

**Helper Methods**:
- `Pose2D::new(x, y, theta)` - Create pose
- `Pose2D::origin()` - Create pose at (0, 0, 0)
- `pose.distance_to(&other)` - Euclidean distance
- `pose.normalize_angle()` - Normalize theta to [-pi, pi]

### Twist

Velocity command with linear and angular components:

```rust
pub struct Twist {
    pub linear: [f64; 3],    // Linear velocity [x, y, z] in m/s
    pub angular: [f64; 3],   // Angular velocity [roll, pitch, yaw] in rad/s
    pub timestamp: u64,      // Timestamp (ns since epoch)
}
```

**Helper Methods**:
- `Twist::new(linear, angular)` - Create twist
- `Twist::new_2d(linear_x, angular_z)` - Create 2D twist
- `Twist::stop()` - Zero velocity command

## Public API

### Construction

```rust
use horus_library::nodes::OdometryNode;

// Create differential drive odometry (two-wheeled robot)
let mut odom = OdometryNode::new_differential_drive(0.5, 0.05)?;  // wheelbase, radius

// Create mecanum drive odometry (omni-directional)
let mut odom = OdometryNode::new_mecanum_drive(0.4, 0.35, 0.05)?;  // base, width, radius

// Create ackermann steering odometry (car-like)
let mut odom = OdometryNode::new_ackermann(0.5, 0.4, 0.08)?;  // wheelbase, track, radius
```

### Configuration Methods

```rust
// Set encoder resolution (ticks per revolution)
odom.set_encoder_resolution(4096);  // Typical quadrature encoder

// Set wheel parameters
odom.set_wheel_parameters(0.5, 0.05);  // wheelbase(m), radius(m)

// Set update rate in Hz
odom.set_update_rate(50.0);  // 50 Hz publishing

// Set covariance parameters
odom.set_covariance(0.01, 0.001);  // position_var(m^2), orientation_var(rad^2)

// Set coordinate frame IDs
odom.set_frames("odom", "base_link");

// Enable/disable encoder input
odom.enable_encoder_input(true);

// Enable/disable velocity input (fallback or additional source)
odom.enable_velocity_input(false);
```

### State Management Methods

```rust
// Reset odometry to origin
odom.reset();

// Set current pose (for calibration or initialization)
odom.set_pose(1.5, 2.0, 1.57);  // x(m), y(m), theta(rad)

// Get current pose
let (x, y, theta) = odom.get_pose();
println!("Position: ({:.3}, {:.3})m, Heading: {:.2}°", x, y, theta.to_degrees());

// Get current velocity
let (vx, vy, vtheta) = odom.get_velocity();
println!("Linear: ({:.2}, {:.2})m/s, Angular: {:.2}rad/s", vx, vy, vtheta);
```

## Usage Examples

### Differential Drive Robot

```rust
use horus_library::prelude::*;

fn main() -> Result<()> {
    let mut scheduler = Scheduler::new();

    // Create odometry node for differential drive
    let mut odom = OdometryNode::new_differential_drive(0.3, 0.05)?;

    // Configure encoder parameters
    odom.set_encoder_resolution(600);  // 600 ticks per revolution
    odom.set_update_rate(50.0);  // 50 Hz updates

    // Set covariance estimates
    odom.set_covariance(0.005, 0.0005);  // 7cm position, 1.3° orientation std dev

    scheduler.add(Box::new(odom), 1, Some(true));
    scheduler.run()?;
    Ok(())
}
```

### Reading Odometry Data

```rust
use horus_library::prelude::*;

fn main() -> Result<()> {
    let mut scheduler = Scheduler::new();

    // Odometry node
    let mut odom = OdometryNode::new_differential_drive(0.4, 0.06)?;
    odom.set_encoder_resolution(2048);
    odom.set_update_rate(50.0);
    scheduler.add(Box::new(odom), 1, Some(true));

    // Odometry consumer
    let odom_reader = node! {
        name: "odom_reader",
        tick: |ctx| {
            let hub = Hub::<Odometry>::new("odom")?;

            while let Some(odom_msg) = hub.recv(None) {
                ctx.log_info(&format!(
                    "Pose: x={:.3}m, y={:.3}m, theta={:.1}°",
                    odom_msg.pose.x,
                    odom_msg.pose.y,
                    odom_msg.pose.theta.to_degrees()
                ));

                ctx.log_info(&format!(
                    "Velocity: vx={:.2}m/s, omega={:.2}rad/s",
                    odom_msg.twist.linear[0],
                    odom_msg.twist.angular[2]
                ));
            }
            Ok(())
        }
    };

    scheduler.add(Box::new(odom_reader), 2, Some(true));
    scheduler.run()?;
    Ok(())
}
```

### Mecanum Drive with Encoder Inputs

```rust
use horus_library::prelude::*;

fn main() -> Result<()> {
    let mut scheduler = Scheduler::new();

    // Mecanum odometry (omni-directional)
    let mut odom = OdometryNode::new_mecanum_drive(0.35, 0.30, 0.05)?;
    odom.set_encoder_resolution(4096);
    odom.set_update_rate(100.0);  // 100 Hz for responsive control
    odom.enable_encoder_input(true);
    scheduler.add(Box::new(odom), 1, Some(true));

    // Encoder publisher (simulated)
    let encoder_sim = node! {
        name: "encoder_simulator",
        tick: |ctx| {
            let left_hub = Hub::<i64>::new("encoder.left")?;
            let right_hub = Hub::<i64>::new("encoder.right")?;

            static mut LEFT_COUNT: i64 = 0;
            static mut RIGHT_COUNT: i64 = 0;

            unsafe {
                // Simulate forward motion: 10 ticks per update
                LEFT_COUNT += 10;
                RIGHT_COUNT += 10;

                left_hub.send(LEFT_COUNT, &mut None)?;
                right_hub.send(RIGHT_COUNT, &mut None)?;
            }

            Ok(())
        }
    };

    scheduler.add(Box::new(encoder_sim), 1, Some(true));
    scheduler.run()?;
    Ok(())
}
```

### Velocity-Based Odometry (Dead Reckoning)

```rust
use horus_library::prelude::*;

fn main() -> Result<()> {
    let mut scheduler = Scheduler::new();

    // Odometry using velocity commands when encoders unavailable
    let mut odom = OdometryNode::new_differential_drive(0.4, 0.05)?;
    odom.enable_encoder_input(false);  // Disable encoder input
    odom.enable_velocity_input(true);   // Enable velocity integration
    odom.set_update_rate(30.0);
    scheduler.add(Box::new(odom), 1, Some(true));

    // Velocity command publisher
    let cmd_publisher = node! {
        name: "velocity_commander",
        tick: |ctx| {
            let cmd_hub = Hub::<Twist>::new("cmd_vel")?;

            // Send forward velocity command: 0.5 m/s, 0.1 rad/s
            let twist = Twist::new_2d(0.5, 0.1);
            cmd_hub.send(twist, &mut None)?;

            Ok(())
        }
    };

    scheduler.add(Box::new(cmd_publisher), 1, Some(true));
    scheduler.run()?;
    Ok(())
}
```

### Reset and Calibration

```rust
use horus_library::prelude::*;

fn main() -> Result<()> {
    let mut scheduler = Scheduler::new();

    let mut odom = OdometryNode::new_differential_drive(0.4, 0.06)?;
    odom.set_encoder_resolution(1024);
    odom.set_update_rate(50.0);

    // Initialize at known position (e.g., from localization)
    odom.set_pose(2.5, 1.0, 0.785);  // x=2.5m, y=1.0m, theta=45°

    scheduler.add(Box::new(odom), 1, Some(true));
    scheduler.run()?;
    Ok(())
}
```

### Navigation with Odometry Feedback

```rust
use horus_library::prelude::*;
use std::f64::consts::PI;

fn main() -> Result<()> {
    let mut scheduler = Scheduler::new();

    // Odometry node
    let mut odom = OdometryNode::new_differential_drive(0.4, 0.06)?;
    odom.set_encoder_resolution(2048);
    odom.set_update_rate(50.0);
    scheduler.add(Box::new(odom), 1, Some(true));

    // Simple goal-based controller
    let controller = node! {
        name: "position_controller",
        tick: |ctx| {
            let odom_hub = Hub::<Odometry>::new("odom")?;
            let cmd_hub = Hub::<Twist>::new("cmd_vel")?;

            // Goal position
            let goal_x = 5.0;
            let goal_y = 3.0;
            let tolerance = 0.1;  // 10cm tolerance

            if let Some(odom_msg) = odom_hub.recv(None) {
                let dx = goal_x - odom_msg.pose.x;
                let dy = goal_y - odom_msg.pose.y;
                let distance = (dx * dx + dy * dy).sqrt();

                if distance > tolerance {
                    // Calculate heading to goal
                    let desired_theta = dy.atan2(dx);
                    let theta_error = desired_theta - odom_msg.pose.theta;

                    // Normalize angle error to [-pi, pi]
                    let theta_error = if theta_error > PI {
                        theta_error - 2.0 * PI
                    } else if theta_error < -PI {
                        theta_error + 2.0 * PI
                    } else {
                        theta_error
                    };

                    // Simple proportional controller
                    let linear_vel = (0.3 * distance).min(0.5);  // Max 0.5 m/s
                    let angular_vel = 1.0 * theta_error;  // Proportional to error

                    let twist = Twist::new_2d(linear_vel, angular_vel);
                    cmd_hub.send(twist, &mut None)?;

                    ctx.log_info(&format!("Distance to goal: {:.2}m", distance));
                } else {
                    // Stop at goal
                    cmd_hub.send(Twist::stop(), &mut None)?;
                    ctx.log_info("Goal reached!");
                }
            }
            Ok(())
        }
    };

    scheduler.add(Box::new(controller), 2, Some(true));
    scheduler.run()?;
    Ok(())
}
```

## Kinematic Models

### Differential Drive

Two independently driven wheels with a caster or ball support:

```rust
let odom = OdometryNode::new_differential_drive(wheel_base, wheel_radius)?;
```

**Equations**:
```
dc = (dleft + dright) / 2              // Center displacement
dtheta = (dright - dleft) / wheelbase  // Change in orientation

// Arc motion:
x += radius * sin(theta + dtheta) - radius * sin(theta)
y += -radius * cos(theta + dtheta) + radius * cos(theta)
theta += dtheta

// Straight line (when dtheta ≈ 0):
x += dc * cos(theta)
y += dc * sin(theta)
```

**Use cases**: Two-wheeled robots, mobile platforms

### Mecanum Drive

Four mecanum wheels for omni-directional motion:

```rust
let odom = OdometryNode::new_mecanum_drive(wheel_base, track_width, wheel_radius)?;
```

**Note**: Current implementation uses simplified 2-wheel kinematics. Full mecanum requires 4 encoders for lateral motion estimation.

**Use cases**: Omni-directional robots, warehouse AGVs

### Ackermann Steering

Car-like steering with front wheel steering:

```rust
let odom = OdometryNode::new_ackermann(wheel_base, track_width, wheel_radius)?;
```

**Note**: Full ackermann kinematics require steering angle input. Current implementation approximates using differential wheel speeds.

**Use cases**: Car-like robots, outdoor vehicles

### Skid Steer

Tank-like motion with independent track control:

```rust
let odom = OdometryNode::new_differential_drive(wheel_base, wheel_radius)?;
// Skid steer uses same kinematics as differential drive
```

**Use cases**: Tracked vehicles, rough-terrain robots

## Encoder Configuration

### Resolution Calculation

Encoder resolution depends on encoder type:

```
Quadrature encoder: ticks_per_rev = CPR × 4
Single-channel encoder: ticks_per_rev = CPR × 1
```

**Example**:
- 600 CPR quadrature encoder: `set_encoder_resolution(2400)`
- 1024 CPR quadrature encoder: `set_encoder_resolution(4096)`

### Common Encoders

| Encoder Model | Type | CPR | Ticks/Rev | Resolution |
|--------------|------|-----|-----------|------------|
| E4P-100 | Quadrature | 100 | 400 | `400` |
| HEDS-5540 | Quadrature | 500 | 2000 | `2000` |
| AMT103 | Quadrature | 2048 | 8192 | `8192` |
| AS5048A | Magnetic | 4096 | 16384 | `16384` |

### Measurement Accuracy

Distance per tick depends on wheel radius and encoder resolution:

```
distance_per_tick = (2π × wheel_radius) / encoder_resolution

Examples:
- wheel_radius = 0.05m, resolution = 2048:
  distance_per_tick = 0.314 / 2048 = 0.153 mm

- wheel_radius = 0.08m, resolution = 600:
  distance_per_tick = 0.502 / 600 = 0.837 mm
```

## Covariance and Uncertainty

### Understanding Covariance

Covariance matrices (6x6) represent uncertainty in pose and velocity:

```
Pose covariance: [x, y, z, roll, pitch, yaw]
Twist covariance: [vx, vy, vz, vroll, vpitch, vyaw]
```

For 2D robots, only x, y, and yaw are used. Other values are set to 1e9 (effectively infinite uncertainty).

### Setting Covariance

```rust
// position_variance: expected squared error in position (m^2)
// orientation_variance: expected squared error in angle (rad^2)
odom.set_covariance(0.01, 0.001);

// Convert from standard deviation:
let pos_std_dev = 0.05;  // 5cm
let angle_std_dev = 0.02;  // ~1.15 degrees
odom.set_covariance(
    pos_std_dev * pos_std_dev,        // 0.0025
    angle_std_dev * angle_std_dev     // 0.0004
);
```

### Typical Values

| Robot Type | Position Variance | Orientation Variance | Notes |
|-----------|------------------|---------------------|-------|
| High-precision encoders | 0.001 (3cm std) | 0.0001 (0.6° std) | Lab robots, smooth floors |
| Standard encoders | 0.01 (10cm std) | 0.001 (1.8° std) | Most mobile robots |
| Low-cost encoders | 0.04 (20cm std) | 0.005 (4° std) | Budget platforms |
| Velocity-only | 0.1 (30cm std) | 0.01 (5.7° std) | Dead reckoning fallback |

### Error Accumulation

Odometry error grows unbounded over time due to:
- Wheel slip on smooth or uneven surfaces
- Encoder quantization and noise
- Wheel radius uncertainty
- Uneven tire wear
- Floor irregularities

**Mitigation strategies**:
1. Fuse with absolute sensors (GPS, SLAM, visual landmarks)
2. Periodic reset from known positions
3. Use for short-term navigation only
4. Increase covariance over distance traveled

## Frame Conventions

### Coordinate Frames

```
odom frame (frame_id):
- World-fixed reference frame
- Origin at odometry reset point
- Does not drift over time relative to starting point
- Used for short-term navigation

base_link frame (child_frame_id):
- Robot body-fixed frame
- Origin typically at robot center
- Moves with the robot
- Used for velocity measurements
```

### Transform Hierarchy

```
map → odom → base_link

map: Global reference frame (from SLAM/localization)
odom: Local reference frame (from odometry)
base_link: Robot body frame
```

### Custom Frame IDs

```rust
// Default frames
odom.set_frames("odom", "base_link");

// Custom frames for multi-robot systems
odom.set_frames("robot1/odom", "robot1/base_link");
odom.set_frames("robot2/odom", "robot2/base_link");
```

## Best Practices

1. **Choose appropriate update rate**:
   ```rust
   // Balance between CPU usage and responsiveness
   odom.set_update_rate(50.0);   // Good for most applications
   odom.set_update_rate(100.0);  // High-speed navigation
   odom.set_update_rate(20.0);   // Low-power applications
   ```

2. **Accurate wheel parameters**:
   ```rust
   // Measure actual wheel radius under load
   // Account for tire compression
   let loaded_radius = 0.048;  // vs 0.050 unloaded
   odom.set_wheel_parameters(0.40, loaded_radius);
   ```

3. **Encoder resolution matching**:
   ```rust
   // Use actual encoder ticks per revolution
   // Account for gear ratios if encoder on motor shaft
   let motor_cpr = 600;
   let gear_ratio = 30.0;
   let effective_resolution = (motor_cpr * 4.0 * gear_ratio) as u32;
   odom.set_encoder_resolution(effective_resolution);  // 72000
   ```

4. **Reset periodically from absolute localization**:
   ```rust
   // When receiving GPS or SLAM pose update
   if let Some(absolute_pose) = localization.get_pose() {
       odom.set_pose(absolute_pose.x, absolute_pose.y, absolute_pose.theta);
   }
   ```

5. **Use velocity fallback for robustness**:
   ```rust
   // Enable both encoder and velocity inputs
   odom.enable_encoder_input(true);
   odom.enable_velocity_input(true);  // Fallback when encoders fail
   ```

6. **Tune covariance to match actual performance**:
   ```rust
   // Test by driving known distances and measuring error
   // Increase covariance if consistently underestimating error
   // Decrease if overestimating (but conservative is safer)
   ```

7. **Handle encoder overflow**:
   ```rust
   // The node handles i64 tick counts automatically
   // Overflow occurs after ~9.2 quintillion ticks
   // For 4096 ticks/rev: ~2.2 trillion revolutions
   // Not a practical concern for robotics applications
   ```

## Troubleshooting

### Robot position drifts even when stationary

**Causes**:
- Encoder noise or electrical interference
- Motor jitter or vibration
- Loose encoder coupling

**Solutions**:
1. Add deadband to encoder readings
2. Check encoder wiring and shielding
3. Ensure solid encoder mounting
4. Verify motor isn't oscillating due to controller gains

### Position estimate way off from actual position

**Causes**:
- Incorrect wheel radius or wheelbase
- Wrong encoder resolution
- Encoder gear ratio not accounted for
- Wheel slip

**Solutions**:
1. Measure actual wheel parameters under load
   ```rust
   // Drive 10 meters straight, measure encoder ticks
   let actual_distance = 10.0;  // meters
   let total_ticks = 20000;      // measured
   let wheel_circumference = (actual_distance * encoder_resolution as f64)
                            / total_ticks as f64;
   let corrected_radius = wheel_circumference / (2.0 * PI);
   odom.set_wheel_parameters(wheelbase, corrected_radius);
   ```

2. Verify encoder resolution matches physical hardware
3. Increase position covariance to reflect uncertainty

### Heading (theta) drifts during straight motion

**Causes**:
- Wheelbase parameter incorrect
- Wheel diameters not identical
- Encoder mounting or calibration mismatch

**Solutions**:
1. Calibrate wheelbase by rotating robot 360 degrees
   ```rust
   // Rotate robot exactly 360° (2π radians)
   // Measure encoder difference
   let left_ticks = -31400;
   let right_ticks = 31400;
   let tick_diff = (right_ticks - left_ticks).abs();

   // Calculate actual wheelbase
   let meters_per_tick = (2.0 * PI * wheel_radius) / encoder_resolution as f64;
   let arc_length = tick_diff as f64 * meters_per_tick;
   let actual_wheelbase = arc_length / (2.0 * PI);

   odom.set_wheel_parameters(actual_wheelbase, wheel_radius);
   ```

2. Check for mechanical play in drivetrain
3. Verify wheels are same size and properly inflated

### No odometry messages published

**Causes**:
- No encoder data being received
- Encoder topics misconfigured
- Node not running

**Solutions**:
1. Verify encoder publishers are running
2. Check topic names match:
   ```rust
   // Ensure encoder publishers use correct topics
   let left_pub = Hub::<i64>::new("encoder.left")?;
   let right_pub = Hub::<i64>::new("encoder.right")?;
   ```
3. Enable velocity input as fallback:
   ```rust
   odom.enable_velocity_input(true);
   ```

### Odometry updates too slow or too fast

**Causes**:
- Update rate not matching application needs
- Encoder update rate insufficient

**Solutions**:
1. Adjust update rate:
   ```rust
   odom.set_update_rate(50.0);  // Match control loop rate
   ```

2. Ensure encoder publishers run at sufficient frequency
3. Check scheduler priorities and tick rates

### Position jumps or discontinuities

**Causes**:
- Encoder counter overflow (unlikely with i64)
- Time delta calculation issues
- Rapid encoder changes

**Solutions**:
1. Check for encoder data corruption
2. Verify encoders aren't skipping due to loose coupling
3. Review logs for large velocity spikes
4. Add velocity limiting if needed

### Cannot move sideways with mecanum wheels

**Expected behavior**: Current mecanum implementation only supports 2-encoder mode (forward/backward and rotation). Full omni-directional motion requires 4 encoders.

**Workaround**: Use velocity-based odometry with 4-wheel mecanum kinematics implemented in velocity publisher.

## Integration with Other Nodes

### With EncoderNode

```rust
use horus_library::prelude::*;

let mut scheduler = Scheduler::new();

// Encoder nodes publish to encoder.left and encoder.right
let left_encoder = EncoderNode::new("encoder.left", 23, 24)?;
let right_encoder = EncoderNode::new("encoder.right", 25, 26)?;

scheduler.add(Box::new(left_encoder), 1, Some(true));
scheduler.add(Box::new(right_encoder), 1, Some(true));

// Odometry node subscribes to encoder topics
let mut odom = OdometryNode::new_differential_drive(0.4, 0.06)?;
odom.set_encoder_resolution(2400);
odom.set_update_rate(50.0);

scheduler.add(Box::new(odom), 2, Some(true));
scheduler.run()?;
```

### With DifferentialDriveNode

```rust
use horus_library::prelude::*;

let mut scheduler = Scheduler::new();

// Differential drive publishes velocity commands
let mut drive = DifferentialDriveNode::new(0.4, 0.06)?;
scheduler.add(Box::new(drive), 1, Some(true));

// Odometry can use velocity for dead reckoning
let mut odom = OdometryNode::new_differential_drive(0.4, 0.06)?;
odom.enable_velocity_input(true);
odom.set_update_rate(30.0);

scheduler.add(Box::new(odom), 2, Some(true));
scheduler.run()?;
```

### With LocalizationNode (Sensor Fusion)

```rust
use horus_library::prelude::*;

let mut scheduler = Scheduler::new();

// Odometry provides motion model
let mut odom = OdometryNode::new_differential_drive(0.4, 0.06)?;
odom.set_covariance(0.01, 0.001);
scheduler.add(Box::new(odom), 1, Some(true));

// Localization fuses odometry with GPS/SLAM
let localization = node! {
    name: "localization",
    tick: |ctx| {
        let odom_hub = Hub::<Odometry>::new("odom")?;

        if let Some(odom) = odom_hub.recv(None) {
            // Use odometry covariance for Kalman filter
            // Fuse with GPS or visual odometry
            // Publish corrected pose estimate
        }
        Ok(())
    }
};

scheduler.add(Box::new(localization), 2, Some(true));
scheduler.run()?;
```

## Performance Considerations

### CPU Usage

- Update rate directly impacts CPU usage
- Trigonometric calculations in motion model
- Covariance matrix operations

**Optimization**:
```rust
// Reduce update rate if CPU-constrained
odom.set_update_rate(20.0);  // 20 Hz instead of 50 Hz

// Disable velocity input if not needed
odom.enable_velocity_input(false);
```

### Memory Usage

- Minimal: ~2 KB per odometry node
- Static size regardless of update rate
- No dynamic allocations in tick loop

### Latency

- Sub-millisecond processing time
- Limited by encoder update rate
- Typical end-to-end latency: 1-5ms

## See Also

- [EncoderNode](../encoder/) - Wheel encoder reading
- [DifferentialDriveNode](../differential_drive/) - Two-wheeled robot control
- [MecanumDriveNode](../mecanum_drive/) - Omni-directional robot control
- [LocalizationNode](../localization/) - Sensor fusion for pose estimation
- [PathPlannerNode](../path_planner/) - Navigation path planning
- [PIDControllerNode](../pid_controller/) - Position and velocity control
