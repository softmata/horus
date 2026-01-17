# Differential Drive Node

Mobile robot base controller that converts Twist velocity commands to differential drive motor commands and publishes odometry.

## Overview

The Differential Drive Node is designed for mobile robots with differential drive kinematics (two independently driven wheels). It converts high-level Twist velocity commands (linear and angular) into left/right wheel speeds and maintains odometry by integrating wheel movements.

## Architecture

**This node is a thin wrapper** around the pure algorithm in `horus_library/algorithms/`:

- **`algorithms::differential_drive::DifferentialDrive`** - Differential drive kinematics (forward/inverse) and odometry

The node handles:
- Topic subscription/publishing (Hub I/O)
- Twist command reception
- Velocity clamping and validation
- Drive command and odometry publishing
- Frame ID management

The algorithm handles:
- Inverse kinematics (twist → wheel speeds)
- Forward kinematics (wheel speeds → twist)
- Odometry updates (pose integration)
- Pure kinematic computations

This separation enables kinematics algorithm reuse across different robot platforms.

## Topics

### Subscribers

| Topic | Type | Description |
|-------|------|-------------|
| `cmd_vel` | `Twist` | Velocity commands (linear x, angular z) |

### Publishers

| Topic | Type | Description |
|-------|------|-------------|
| `drive_command` | `DifferentialDriveCommand` | Left and right wheel speeds |
| `odom` | `Odometry` | Robot odometry (position, orientation, velocities) |

## Configuration Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `wheel_base` | `f32` | `0.5` | Distance between left and right wheels (meters) |
| `wheel_radius` | `f32` | `0.1` | Radius of the wheels (meters) |
| `max_linear_vel` | `f32` | `2.0` | Maximum linear velocity (m/s) |
| `max_angular_vel` | `f32` | `3.14` | Maximum angular velocity (rad/s) |

## Message Types

### Twist (Input)

Velocity command message:

```rust
pub struct Twist {
    pub linear: [f64; 3],   // [x, y, z] velocities in m/s
    pub angular: [f64; 3],  // [roll, pitch, yaw] velocities in rad/s
}
```

For differential drive, only `linear[0]` (forward/backward) and `angular[2]` (rotation) are used.

### DifferentialDriveCommand (Output)

Wheel speed command:

```rust
pub struct DifferentialDriveCommand {
    pub left_speed: f64,   // Left wheel speed (m/s)
    pub right_speed: f64,  // Right wheel speed (m/s)
}
```

### Odometry (Output)

Robot pose and velocity:

```rust
pub struct Odometry {
    pub pose: Pose2D,           // Position (x, y, theta)
    pub twist: Twist,           // Current velocities
    pub frame_id: [u8; 32],     // Reference frame ("odom")
    pub child_frame_id: [u8; 32], // Child frame ("base_link")
    pub timestamp: u64,
}

pub struct Pose2D {
    pub x: f64,         // X position in meters
    pub y: f64,         // Y position in meters
    pub theta: f64,     // Orientation in radians
}
```

## Public API

### Construction

```rust
use horus_library::nodes::DifferentialDriveNode;

// Create with default topics
let mut drive = DifferentialDriveNode::new()?;

// Create with custom topics
let mut drive = DifferentialDriveNode::new_with_topics(
    "cmd_vel",          // velocity command topic
    "drive_command",    // wheel speed output topic
    "odom"              // odometry topic
)?;
```

### Configuration Methods

```rust
// Set wheel base (distance between wheels in meters)
drive.set_wheel_base(0.45);

// Set wheel radius (in meters)
drive.set_wheel_radius(0.075);

// Set maximum velocities
drive.set_velocity_limits(
    1.5,  // max linear velocity (m/s)
    2.0   // max angular velocity (rad/s)
);

// Reset odometry to origin
drive.reset_odometry();

// Get current position (x, y, theta)
let (x, y, theta) = drive.get_position();
```

## Usage Examples

### Basic Mobile Robot

```rust
use horus_library::nodes::DifferentialDriveNode;
use horus_core::{Node, Runtime};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut runtime = Runtime::new()?;

    // Create differential drive controller
    let mut drive = DifferentialDriveNode::new()?;

    // Configure for specific robot
    drive.set_wheel_base(0.45);      // 45cm wheelbase
    drive.set_wheel_radius(0.075);   // 7.5cm wheels
    drive.set_velocity_limits(1.0, 2.0);

    runtime.add_node(drive);
    runtime.run()?;

    Ok(())
}
```

### With Teleoperation

```rust
use horus_library::nodes::{DifferentialDriveNode, KeyboardInputNode};
use horus_core::{Node, Runtime};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut runtime = Runtime::new()?;

    // Create keyboard teleop node
    let mut keyboard = KeyboardInputNode::new()?;
    keyboard.set_linear_speed(0.5);   // 0.5 m/s
    keyboard.set_angular_speed(1.0);  // 1.0 rad/s

    // Create differential drive controller
    let mut drive = DifferentialDriveNode::new()?;
    drive.set_wheel_base(0.5);
    drive.set_wheel_radius(0.1);

    runtime.add_node(keyboard);
    runtime.add_node(drive);
    runtime.run()?;

    Ok(())
}
```

### With Custom Topics

```rust
use horus_library::nodes::DifferentialDriveNode;
use horus_core::{Node, Runtime};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut runtime = Runtime::new()?;

    // Create drive controller with custom topics
    let mut drive = DifferentialDriveNode::new_with_topics(
        "robot/cmd_vel",
        "motors/wheel_speeds",
        "robot/odometry"
    )?;

    // Configure robot parameters
    drive.set_wheel_base(0.6);
    drive.set_wheel_radius(0.08);
    drive.set_velocity_limits(2.0, 3.0);

    runtime.add_node(drive);
    runtime.run()?;

    Ok(())
}
```

### Position Tracking

```rust
use horus_library::nodes::DifferentialDriveNode;
use horus_core::{Node, Runtime, Hub};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut runtime = Runtime::new()?;

    // Create drive controller
    let mut drive = DifferentialDriveNode::new()?;
    drive.set_wheel_base(0.5);
    drive.set_wheel_radius(0.1);

    // Subscribe to odometry in another node
    let odom_sub = Hub::new("odom")?;

    runtime.add_node(drive);

    // In your application loop
    if let Some(odom) = odom_sub.recv(None) {
        eprintln!("Robot position: ({:.2}, {:.2}, {:.2})",
            odom.pose.x, odom.pose.y, odom.pose.theta);
    }

    Ok(())
}
```

## Differential Drive Kinematics

### Forward Kinematics

Converts Twist command to wheel speeds:

```
left_speed = linear_vel - (angular_vel * wheel_base / 2)
right_speed = linear_vel + (angular_vel * wheel_base / 2)
```

### Examples

**Moving forward** (1 m/s):
```
Twist { linear: [1.0, 0, 0], angular: [0, 0, 0] }
 left_speed = 1.0, right_speed = 1.0
```

**Rotating in place** (1 rad/s, wheel_base = 0.5m):
```
Twist { linear: [0, 0, 0], angular: [0, 0, 1.0] }
 left_speed = -0.25, right_speed = 0.25
```

**Arc motion** (0.5 m/s forward, 0.5 rad/s turn):
```
Twist { linear: [0.5, 0, 0], angular: [0, 0, 0.5] }
 left_speed = 0.375, right_speed = 0.625
```

### Inverse Kinematics

Calculates robot velocity from wheel speeds:

```
linear_vel = (left_speed + right_speed) / 2
angular_vel = (right_speed - left_speed) / wheel_base
```

## Odometry Calculation

The node integrates wheel velocities to estimate robot position:

```
dx = linear_vel * cos(theta) * dt
dy = linear_vel * sin(theta) * dt
dtheta = angular_vel * dt

x += dx
y += dy
theta += dtheta
```

**Note**: Odometry drifts over time due to wheel slippage and integration errors. For accurate localization, combine with external sensors (GPS, SLAM, etc.).

## Robot Configuration Guide

### Measuring Wheel Base

Measure the distance between the centers of the left and right wheels:

```
    Left Wheel         Right Wheel
        |                  |
        |<--- wheel_base ->|
        |                  |
```

### Measuring Wheel Radius

Measure the diameter and divide by 2, or:

1. Mark a point on the wheel
2. Roll the wheel one complete rotation
3. Measure distance traveled
4. radius = distance / (2 * π)

### Determining Maximum Velocities

**Linear Velocity**:
```rust
// Based on motor max RPM and wheel radius
let max_rpm = 120.0;  // motor specification
let wheel_radius = 0.075;  // meters
let max_linear_vel = (max_rpm / 60.0) * 2.0 * PI * wheel_radius;
```

**Angular Velocity**:
```rust
// Based on wheel base and max linear velocity
let max_angular_vel = max_linear_vel / (wheel_base / 2.0);
```

## Common Robot Configurations

### Small Indoor Robot

```rust
drive.set_wheel_base(0.3);      // 30cm wheelbase
drive.set_wheel_radius(0.05);   // 5cm wheels
drive.set_velocity_limits(0.5, 2.0);  // Slow, agile
```

### Medium Service Robot

```rust
drive.set_wheel_base(0.5);      // 50cm wheelbase
drive.set_wheel_radius(0.1);    // 10cm wheels
drive.set_velocity_limits(1.0, 2.0);  // Moderate speed
```

### Large Outdoor Robot

```rust
drive.set_wheel_base(0.8);      // 80cm wheelbase
drive.set_wheel_radius(0.15);   // 15cm wheels
drive.set_velocity_limits(2.0, 1.5);  // Fast, stable
```

### Warehouse AGV

```rust
drive.set_wheel_base(0.6);      // 60cm wheelbase
drive.set_wheel_radius(0.125);  // 12.5cm wheels
drive.set_velocity_limits(1.5, 1.0);  // Fast forward, slow turns
```

## Coordinate Frames

The node uses standard ROS coordinate conventions:

- **odom frame**: Fixed world frame (odometry origin)
- **base_link frame**: Robot's local frame (center between wheels)

**Coordinate System**:
- X-axis: Forward
- Y-axis: Left
- Z-axis: Up
- Theta: Counter-clockwise rotation (right-hand rule)

## Performance Considerations

### Update Rate

- Recommended: 20-100 Hz (10-50ms tick rate)
- Higher rates improve odometry accuracy
- Lower rates reduce CPU usage

### Odometry Accuracy

Factors affecting accuracy:
1. Wheel slippage (on smooth/uneven surfaces)
2. Wheel diameter differences
3. Measurement errors in wheel_base
4. Encoder resolution
5. Update rate

**Typical drift**: 1-5% of distance traveled

### CPU Usage

Minimal - simple trigonometric calculations per tick.

### Memory Usage

Small fixed footprint (~200 bytes).

## Troubleshooting

### Issue: Robot doesn't move

**Check**:
1. Velocity commands are being published to `cmd_vel`
2. Velocity limits aren't too restrictive
3. Motor controller is receiving wheel speed commands

**Debug**:
```rust
// Monitor wheel speeds
let drive_sub = Hub::new("drive_command")?;
if let Some(cmd) = drive_sub.recv(None) {
    eprintln!("Left: {:.2}, Right: {:.2}", cmd.left_speed, cmd.right_speed);
}
```

### Issue: Robot moves in wrong direction

**Cause**: Motors wired incorrectly or wheel_base sign wrong

**Solution**:
- Swap left/right motor connections
- Or negate angular velocity in motor controller

### Issue: Robot drifts to one side

**Cause**: Wheels have different effective radii

**Solution**:
- Calibrate wheel radius for each wheel
- Add trim factor in motor controller
- Check for mechanical issues (wheel wear, alignment)

### Issue: Odometry drifts significantly

**Causes**:
1. Incorrect wheel_base or wheel_radius
2. Wheel slippage
3. Low update rate

**Solutions**:
```rust
// Verify robot dimensions
drive.set_wheel_base(measured_wheel_base);
drive.set_wheel_radius(measured_wheel_radius);

// Increase update rate (in Runtime config)
runtime.set_tick_rate(50.0);  // 50 Hz

// For critical applications, use sensor fusion
// (combine odometry with IMU, visual odometry, etc.)
```

### Issue: Robot can't turn in place

**Cause**: Insufficient angular velocity limit

**Solution**:
```rust
// Increase angular velocity limit
drive.set_velocity_limits(1.0, 3.0);  // Allow faster rotation
```

## Integration with Other Nodes

### With Path Planner

```rust
// Path planner publishes cmd_vel
// DifferentialDrive subscribes and executes
let mut planner = PathPlannerNode::new()?;
let mut drive = DifferentialDriveNode::new()?;

runtime.add_node(planner);
runtime.add_node(drive);
```

### With Localization

```rust
// Odometry feeds into localization
let mut drive = DifferentialDriveNode::new()?;
let mut localization = LocalizationNode::new()?;

// Localization subscribes to "odom"
runtime.add_node(drive);
runtime.add_node(localization);
```

### With Safety Monitor

```rust
// Safety monitor checks velocities from odometry
let mut drive = DifferentialDriveNode::new()?;
let mut safety = SafetyMonitorNode::new()?;

runtime.add_node(drive);
runtime.add_node(safety);
```

## Advanced Topics

### Slip Detection

Monitor odometry vs. expected motion:

```rust
// Compare commanded velocity to actual velocity from encoders
let commanded_vel = twist.linear[0];
let actual_vel = odom.twist.linear[0];
let slip_ratio = (commanded_vel - actual_vel) / commanded_vel;

if slip_ratio > 0.2 {
    eprintln!("Warning: Wheel slip detected!");
}
```

### Acceleration Limiting

Add acceleration ramp in your velocity command publisher:

```rust
let max_accel = 0.5;  // m/s^2
let target_vel = 1.0;
let current_vel = 0.0;

// Ramp velocity
let delta_vel = max_accel * dt;
let commanded_vel = current_vel + delta_vel.min(target_vel - current_vel);
```

## Related Nodes

- **KeyboardInputNode**: Provides cmd_vel for teleoperation
- **PathPlannerNode**: Generates cmd_vel for autonomous navigation
- **EncoderNode**: Can provide wheel encoder feedback for closed-loop control
- **LocalizationNode**: Uses odometry for robot localization

## See Also

- [Differential Drive Kinematics](https://en.wikipedia.org/wiki/Differential_wheeled_robot)
- [Mobile Robot Odometry](https://en.wikipedia.org/wiki/Odometry)
- [ROS Navigation Tuning Guide](http://wiki.ros.org/navigation/Tutorials)
