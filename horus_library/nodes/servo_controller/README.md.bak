# Servo Controller Node

Multi-servo controller for robotic arms, pan-tilt mechanisms, and coordinated actuator systems.

## Overview

The Servo Controller Node manages multiple servo motors with position, velocity, and torque control. It provides coordinated multi-servo control for applications like robotic arms, pan-tilt camera systems, and other multi-DOF mechanisms. The node supports individual servo commands and synchronized joint control with position limits, velocity limits, and smooth interpolation.

## Topics

### Subscribers

| Topic | Type | Description |
|-------|------|-------------|
| `servo_command` | `ServoCommand` | Individual servo position commands |
| `joint_command` | `JointCommand` | Multi-joint coordinated commands |

### Publishers

| Topic | Type | Description |
|-------|------|-------------|
| `joint_states` | `JointCommand` | Current positions, velocities, and efforts for all servos |

## Configuration Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `servo_count` | `u8` | `6` | Number of servos to control (typical 6-DOF robot arm) |
| `position_limits` | `HashMap<u8, (f64, f64)>` | `(-3.14, 3.14)` | Min/max position limits per servo in radians |
| `velocity_limits` | `HashMap<u8, f64>` | `2.0` | Maximum velocity per servo in rad/s |
| `torque_limits` | `HashMap<u8, f64>` | `10.0` | Maximum torque per servo in Nm |
| `position_tolerance` | `f64` | `0.01` | Position error tolerance in radians (~0.57 degrees) |
| `default_velocity` | `f64` | `1.0` | Default movement velocity in rad/s |
| `interpolation_enabled` | `bool` | `true` | Enable smooth position interpolation |

## Message Types

### ServoCommand

Individual servo position command:

```rust
pub struct ServoCommand {
    pub servo_id: u8,        // Servo identifier (0-255)
    pub position: f32,       // Target position in radians
    pub speed: f32,          // Movement speed (0-1, where 0=max speed)
    pub enable: bool,        // Torque enable/disable
    pub timestamp: u64,      // Nanoseconds since epoch
}
```

**Helper Methods:**
```rust
// Create basic servo command
ServoCommand::new(servo_id: u8, position: f32)

// Create with specific speed
ServoCommand::with_speed(servo_id: u8, position: f32, speed: f32)

// Disable servo (remove torque)
ServoCommand::disable(servo_id: u8)

// Convert from degrees
ServoCommand::from_degrees(servo_id: u8, degrees: f32)
```

### JointCommand

Multi-joint coordinated control:

```rust
pub struct JointCommand {
    pub joint_names: [[u8; 32]; 16],  // Joint name strings (max 16 joints)
    pub joint_count: u8,               // Number of active joints
    pub positions: [f64; 16],          // Position commands in radians
    pub velocities: [f64; 16],         // Velocity commands in rad/s
    pub efforts: [f64; 16],            // Effort/torque commands in Nm
    pub modes: [u8; 16],               // Control mode per joint (0=pos, 1=vel, 2=effort)
    pub timestamp: u64,                // Nanoseconds since epoch
}
```

**Control Modes:**
- `MODE_POSITION = 0` - Position control mode
- `MODE_VELOCITY = 1` - Velocity control mode
- `MODE_EFFORT = 2` - Effort/torque control mode

## Public API

### Construction

```rust
use horus_library::nodes::ServoControllerNode;

// Create with default topics
let mut servo_ctrl = ServoControllerNode::new()?;

// Create with custom topics
let mut servo_ctrl = ServoControllerNode::new_with_topics(
    "robot/servo_cmd",     // servo command topic
    "robot/joint_cmd",     // joint command topic
    "robot/joint_states"   // joint states topic
)?;
```

### Configuration Methods

```rust
// Set number of servos
servo_ctrl.set_servo_count(6);

// Set position limits for a specific servo
servo_ctrl.set_position_limits(0, -1.57, 1.57);  // ±90 degrees

// Set velocity limit for a servo
servo_ctrl.set_velocity_limit(0, 3.0);  // 3 rad/s max

// Set torque limit for a servo
servo_ctrl.set_torque_limit(0, 5.0);  // 5 Nm max

// Enable/disable position interpolation
servo_ctrl.set_interpolation(true);  // smooth motion

// Get current position of a servo
if let Some(pos) = servo_ctrl.get_position(0) {
    eprintln!("Servo 0 position: {} rad", pos);
}

// Get all current positions
let positions = servo_ctrl.get_all_positions();

// Move all servos to home position (0.0)
servo_ctrl.move_to_home();

// Stop all servo motion
servo_ctrl.stop_all();
```

## Usage Examples

### Robotic Arm Control

```rust
use horus_library::nodes::ServoControllerNode;
use horus_core::{Node, Runtime, Hub};
use horus_library::JointCommand;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut runtime = Runtime::new()?;

    // Create 6-DOF robot arm controller
    let mut arm = ServoControllerNode::new_with_topics(
        "arm/servo_cmd",
        "arm/joint_cmd",
        "arm/joint_states"
    )?;

    // Configure 6-DOF arm
    arm.set_servo_count(6);

    // Set limits for each joint
    // Base rotation (unlimited)
    arm.set_position_limits(0, -3.14, 3.14);
    arm.set_velocity_limit(0, 2.0);

    // Shoulder (limited range)
    arm.set_position_limits(1, -1.57, 1.57);
    arm.set_velocity_limit(1, 1.5);

    // Elbow
    arm.set_position_limits(2, 0.0, 2.36);  // 0-135 degrees
    arm.set_velocity_limit(2, 2.0);

    // Wrist joints (faster, limited range)
    for i in 3..6 {
        arm.set_position_limits(i, -1.57, 1.57);
        arm.set_velocity_limit(i, 3.0);
        arm.set_torque_limit(i, 3.0);  // Lower torque for wrist
    }

    // Enable smooth interpolation for fluid motion
    arm.set_interpolation(true);

    runtime.add_node(arm);
    runtime.run()?;

    Ok(())
}
```

### Pan-Tilt Camera System

```rust
use horus_library::nodes::ServoControllerNode;
use horus_library::ServoCommand;
use horus_core::{Node, Runtime, Hub};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut runtime = Runtime::new()?;

    // Create pan-tilt controller
    let mut camera = ServoControllerNode::new_with_topics(
        "camera/servo_cmd",
        "camera/joint_cmd",
        "camera/joint_states"
    )?;

    // Configure 2-DOF pan-tilt
    camera.set_servo_count(2);

    // Pan servo (horizontal rotation)
    camera.set_position_limits(0, -3.14, 3.14);  // Full rotation
    camera.set_velocity_limit(0, 4.0);  // Fast panning

    // Tilt servo (vertical)
    camera.set_position_limits(1, -0.785, 0.785);  // ±45 degrees
    camera.set_velocity_limit(1, 3.0);

    // Enable smooth tracking
    camera.set_interpolation(true);

    runtime.add_node(camera);

    // Example: Send pan-tilt commands
    let servo_pub = Hub::<ServoCommand>::new("camera/servo_cmd")?;

    // Pan to 45 degrees
    let pan_cmd = ServoCommand::from_degrees(0, 45.0);
    servo_pub.send(pan_cmd, None)?;

    // Tilt to 30 degrees with slow speed
    let tilt_cmd = ServoCommand::with_speed(1, 0.523, 0.3);
    servo_pub.send(tilt_cmd, None)?;

    runtime.run()?;

    Ok(())
}
```

### Multi-Servo Coordination

```rust
use horus_library::nodes::ServoControllerNode;
use horus_library::JointCommand;
use horus_core::{Node, Runtime, Hub};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut runtime = Runtime::new()?;

    // Create multi-servo controller
    let mut servos = ServoControllerNode::new()?;
    servos.set_servo_count(4);

    // Configure all servos with same limits
    for i in 0..4 {
        servos.set_position_limits(i, -2.0, 2.0);
        servos.set_velocity_limit(i, 2.5);
    }

    runtime.add_node(servos);

    // Coordinated joint command publisher
    let joint_pub = Hub::<JointCommand>::new("joint_command")?;

    // Create coordinated movement
    let mut cmd = JointCommand::new();
    cmd.add_position("joint_0", 1.0)?;
    cmd.add_position("joint_1", -0.5)?;
    cmd.add_position("joint_2", 0.8)?;
    cmd.add_position("joint_3", -0.3)?;

    // All servos move simultaneously
    joint_pub.send(cmd, None)?;

    runtime.run()?;

    Ok(())
}
```

### Gripper Control

```rust
use horus_library::nodes::ServoControllerNode;
use horus_library::ServoCommand;
use horus_core::{Node, Runtime, Hub};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut runtime = Runtime::new()?;

    // Create gripper controller
    let mut gripper = ServoControllerNode::new_with_topics(
        "gripper/servo_cmd",
        "gripper/joint_cmd",
        "gripper/joint_states"
    )?;

    // Single servo gripper
    gripper.set_servo_count(1);
    gripper.set_position_limits(0, 0.0, 1.57);  // 0-90 degrees (closed-open)
    gripper.set_velocity_limit(0, 2.0);
    gripper.set_torque_limit(0, 8.0);  // Strong grip

    runtime.add_node(gripper);

    let servo_pub = Hub::<ServoCommand>::new("gripper/servo_cmd")?;

    // Open gripper
    let open_cmd = ServoCommand::new(0, 1.57);
    servo_pub.send(open_cmd, None)?;

    // Close gripper with moderate speed
    let close_cmd = ServoCommand::with_speed(0, 0.0, 0.5);
    servo_pub.send(close_cmd, None)?;

    runtime.run()?;

    Ok(())
}
```

## Servo Calibration Procedures

### Manual Calibration

1. **Physical Setup**
   - Mount servos in their zero position
   - Ensure mechanical stops are not engaged
   - Verify power supply is adequate for all servos

2. **Find Center Position**
   ```rust
   // Move servo to center
   let center_cmd = ServoCommand::new(servo_id, 0.0);
   servo_pub.send(center_cmd, None)?;

   // Verify actual position matches expected
   if let Some(actual_pos) = servo_ctrl.get_position(servo_id) {
       eprintln!("Servo {} at position: {}", servo_id, actual_pos);
   }
   ```

3. **Determine Limits**
   ```rust
   // Slowly move to positive limit
   for pos in (0..157).step_by(10) {
       let angle = (pos as f32) / 100.0;
       let cmd = ServoCommand::with_speed(servo_id, angle, 0.2);
       servo_pub.send(cmd, None)?;
       std::thread::sleep(std::time::Duration::from_millis(500));
   }

   // Record maximum safe position
   let max_pos = 1.50;  // Just before physical limit
   servo_ctrl.set_position_limits(servo_id, -max_pos, max_pos);
   ```

4. **Test Movement Range**
   ```rust
   // Test full range motion
   let test_positions = [-1.0, 0.0, 1.0, 0.0];
   for pos in test_positions {
       let cmd = ServoCommand::with_speed(servo_id, pos, 0.3);
       servo_pub.send(cmd, None)?;
       std::thread::sleep(std::time::Duration::from_secs(2));
   }
   ```

### Automated Calibration

```rust
fn calibrate_servo(
    servo_id: u8,
    servo_pub: &Hub<ServoCommand>,
    servo_ctrl: &ServoControllerNode,
) -> Result<(f64, f64), Box<dyn std::error::Error>> {
    eprintln!("Calibrating servo {}...", servo_id);

    // Move to center
    let center = ServoCommand::new(servo_id, 0.0);
    servo_pub.send(center, None)?;
    std::thread::sleep(std::time::Duration::from_secs(1));

    // Find positive limit
    let mut pos_limit = 0.0;
    for i in 0..100 {
        let angle = (i as f32) * 0.05;  // 0.05 rad steps
        let cmd = ServoCommand::with_speed(servo_id, angle, 0.1);
        servo_pub.send(cmd, None)?;
        std::thread::sleep(std::time::Duration::from_millis(200));

        // Check if servo reached target
        if let Some(actual) = servo_ctrl.get_position(servo_id) {
            if (actual - angle as f64).abs() > 0.1 {
                // Servo can't reach target - limit found
                pos_limit = actual - 0.1;  // Safety margin
                break;
            }
        }
    }

    // Return to center
    servo_pub.send(center, None)?;
    std::thread::sleep(std::time::Duration::from_secs(1));

    // Find negative limit (similar process)
    let mut neg_limit = 0.0;
    for i in 0..100 {
        let angle = -(i as f32) * 0.05;
        let cmd = ServoCommand::with_speed(servo_id, angle, 0.1);
        servo_pub.send(cmd, None)?;
        std::thread::sleep(std::time::Duration::from_millis(200));

        if let Some(actual) = servo_ctrl.get_position(servo_id) {
            if (actual - angle as f64).abs() > 0.1 {
                neg_limit = actual + 0.1;
                break;
            }
        }
    }

    // Return to center
    servo_pub.send(center, None)?;

    eprintln!("Servo {} limits: ({}, {})", servo_id, neg_limit, pos_limit);
    Ok((neg_limit, pos_limit))
}
```

## Position and Speed Control

### Position Control Modes

**Direct Position Control** (Interpolation Disabled):
```rust
// Servo jumps directly to target position
servo_ctrl.set_interpolation(false);

let cmd = ServoCommand::new(0, 1.57);
servo_pub.send(cmd, None)?;
// Servo moves at maximum speed to target
```

**Interpolated Position Control** (Interpolation Enabled):
```rust
// Servo moves smoothly respecting velocity limits
servo_ctrl.set_interpolation(true);
servo_ctrl.set_velocity_limit(0, 1.0);  // 1 rad/s max

let cmd = ServoCommand::new(0, 1.57);
servo_pub.send(cmd, None)?;
// Servo takes ~1.57 seconds to reach target
```

### Speed Control

```rust
// Speed parameter in ServoCommand (0.0 to 1.0)
// 0.0 = maximum speed (respects velocity_limit)
// 1.0 = slowest speed

// Fast movement
let fast_cmd = ServoCommand::with_speed(0, 1.0, 0.0);

// Medium speed
let medium_cmd = ServoCommand::with_speed(0, 1.0, 0.5);

// Slow, precise movement
let slow_cmd = ServoCommand::with_speed(0, 1.0, 0.9);
```

### Trajectory Following

```rust
fn execute_trajectory(
    servo_id: u8,
    waypoints: &[f32],
    duration_per_segment: f32,
    servo_pub: &Hub<ServoCommand>,
) -> Result<(), Box<dyn std::error::Error>> {
    for waypoint in waypoints {
        // Calculate speed based on distance and time
        let cmd = ServoCommand::new(servo_id, *waypoint);
        servo_pub.send(cmd, None)?;

        let sleep_time = std::time::Duration::from_secs_f32(duration_per_segment);
        std::thread::sleep(sleep_time);
    }
    Ok(())
}

// Example usage
let trajectory = [0.0, 0.5, 1.0, 1.5, 1.0, 0.0];
execute_trajectory(0, &trajectory, 1.0, &servo_pub)?;
```

## Integration with Control Systems

### PID Position Control Integration

```rust
use horus_library::nodes::{ServoControllerNode, PidControllerNode};
use horus_core::{Node, Runtime};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut runtime = Runtime::new()?;

    // Servo controller provides position feedback
    let mut servos = ServoControllerNode::new_with_topics(
        "servo_cmd",
        "joint_cmd",
        "joint_states"
    )?;
    servos.set_servo_count(1);

    // PID controller for higher-level control
    let mut pid = PidControllerNode::new_with_topics(
        "target_position",      // Setpoint
        "joint_states",         // Feedback from servo controller
        "servo_cmd",            // Output to servo controller
        "pid_config"
    )?;

    pid.set_gains(5.0, 0.5, 0.1);
    pid.set_output_limits(-3.14, 3.14);

    runtime.add_node(servos);
    runtime.add_node(pid);
    runtime.run()?;

    Ok(())
}
```

### Inverse Kinematics Integration

```rust
use horus_library::{ServoControllerNode, JointCommand};
use horus_core::{Node, Runtime, Hub};

// Simple 2-DOF planar arm inverse kinematics
fn inverse_kinematics_2dof(
    target_x: f64,
    target_y: f64,
    link1_length: f64,
    link2_length: f64,
) -> Option<(f64, f64)> {
    let distance = (target_x * target_x + target_y * target_y).sqrt();

    // Check if target is reachable
    if distance > link1_length + link2_length {
        return None;
    }

    // Calculate joint angles using law of cosines
    let cos_theta2 = (distance * distance - link1_length * link1_length - link2_length * link2_length)
        / (2.0 * link1_length * link2_length);

    if cos_theta2.abs() > 1.0 {
        return None;
    }

    let theta2 = cos_theta2.acos();
    let theta1 = target_y.atan2(target_x)
        - ((link2_length * theta2.sin()).atan2(link1_length + link2_length * theta2.cos()));

    Some((theta1, theta2))
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut runtime = Runtime::new()?;

    let mut arm = ServoControllerNode::new()?;
    arm.set_servo_count(2);

    runtime.add_node(arm);

    let joint_pub = Hub::<JointCommand>::new("joint_command")?;

    // Move end effector to target position
    if let Some((j1, j2)) = inverse_kinematics_2dof(0.3, 0.2, 0.25, 0.25) {
        let mut cmd = JointCommand::new();
        cmd.add_position("shoulder", j1)?;
        cmd.add_position("elbow", j2)?;
        joint_pub.send(cmd, None)?;
    }

    runtime.run()?;

    Ok(())
}
```

### Path Planning Integration

```rust
use horus_library::{ServoControllerNode, JointCommand};
use horus_core::{Node, Runtime, Hub};

fn interpolate_path(
    start: &[f64],
    end: &[f64],
    steps: usize,
) -> Vec<Vec<f64>> {
    let mut path = Vec::new();

    for i in 0..=steps {
        let t = i as f64 / steps as f64;
        let point: Vec<f64> = start.iter()
            .zip(end.iter())
            .map(|(s, e)| s + (e - s) * t)
            .collect();
        path.push(point);
    }

    path
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut runtime = Runtime::new()?;

    let mut arm = ServoControllerNode::new()?;
    arm.set_servo_count(6);
    arm.set_interpolation(true);

    runtime.add_node(arm);

    let joint_pub = Hub::<JointCommand>::new("joint_command")?;

    // Define start and goal configurations
    let start = vec![0.0, 0.5, 1.0, 0.0, 0.5, 0.0];
    let goal = vec![1.57, -0.5, 1.5, 0.8, -0.3, 1.0];

    // Generate interpolated path
    let path = interpolate_path(&start, &goal, 50);

    // Execute path
    for waypoint in path {
        let mut cmd = JointCommand::new();
        for (i, pos) in waypoint.iter().enumerate() {
            cmd.add_position(&format!("joint_{}", i), *pos)?;
        }
        joint_pub.send(cmd, None)?;
        std::thread::sleep(std::time::Duration::from_millis(100));
    }

    runtime.run()?;

    Ok(())
}
```

## Troubleshooting

### Issue: Servo Jitter or Oscillation

**Symptoms**: Servo vibrates or oscillates around target position

**Possible Causes**:
1. Position tolerance too tight
2. Velocity limit too high
3. Mechanical binding or friction
4. Power supply noise

**Solutions**:

```rust
// Increase position tolerance
servo_ctrl.set_interpolation(true);
// Internal tolerance is 0.01 rad by default

// Reduce velocity limit
servo_ctrl.set_velocity_limit(servo_id, 0.5);  // Slower motion

// Disable servo when at target to prevent hunting
let cmd = ServoCommand::disable(servo_id);
servo_pub.send(cmd, None)?;

// Add deadband by checking if close enough
if let Some(current) = servo_ctrl.get_position(servo_id) {
    if (current - target).abs() < 0.05 {
        // Close enough, disable servo
        let disable = ServoCommand::disable(servo_id);
        servo_pub.send(disable, None)?;
    }
}
```

### Issue: Servo Not Responding

**Symptoms**: Commands sent but servo doesn't move

**Possible Causes**:
1. Servo ID mismatch
2. Position outside limits
3. Servo disabled
4. Communication error

**Solutions**:

```rust
// Verify servo ID is within range
assert!(servo_id < servo_ctrl.get_servo_count());

// Check position limits
let (min, max) = servo_ctrl.get_position_limits(servo_id);
eprintln!("Limits for servo {}: ({}, {})", servo_id, min, max);

// Ensure servo is enabled
let cmd = ServoCommand::new(servo_id, target);
assert!(cmd.enable);

// Test with home position
servo_ctrl.move_to_home();

// Check if position is being clamped
let requested = 5.0;  // Outside normal range
let clamped = requested.clamp(-3.14, 3.14);
eprintln!("Position clamped from {} to {}", requested, clamped);
```

### Issue: Erratic or Jerky Movement

**Symptoms**: Servo moves in stutters or jumps

**Possible Causes**:
1. Interpolation disabled
2. Commands sent too infrequently
3. Velocity limits inconsistent
4. Tick rate too slow

**Solutions**:

```rust
// Enable smooth interpolation
servo_ctrl.set_interpolation(true);

// Set consistent velocity limits
for i in 0..servo_count {
    servo_ctrl.set_velocity_limit(i, 1.5);
}

// Send commands at regular intervals
use std::thread;
use std::time::Duration;

loop {
    let cmd = calculate_next_position();
    servo_pub.send(cmd, None)?;
    thread::sleep(Duration::from_millis(20));  // 50 Hz update rate
}

// Ensure runtime tick rate is adequate
// Servo controller should tick at minimum 50Hz (20ms)
```

### Issue: Servo Overshoots Target

**Symptoms**: Servo moves past target position

**Possible Causes**:
1. Velocity too high
2. No deceleration ramp
3. Mechanical inertia
4. Position tolerance too large

**Solutions**:

```rust
// Reduce maximum velocity
servo_ctrl.set_velocity_limit(servo_id, 1.0);  // Slower max speed

// Use speed parameter for controlled motion
let cmd = ServoCommand::with_speed(servo_id, target, 0.7);  // 70% speed

// Implement deceleration zone
fn calculate_speed(current: f64, target: f64, decel_zone: f64) -> f32 {
    let error = (target - current).abs();
    if error < decel_zone {
        // Slow down when close to target
        (error / decel_zone).max(0.1) as f32
    } else {
        1.0  // Full speed
    }
}

let speed = calculate_speed(current_pos, target_pos, 0.3);
let cmd = ServoCommand::with_speed(servo_id, target, speed);
```

### Issue: Communication Errors

**Symptoms**: Timeouts, missed commands, or servo state not updating

**Possible Causes**:
1. Topic name mismatch
2. Message rate too high
3. Node not running
4. Hub connection issues

**Solutions**:

```rust
// Verify topic names match
let servo_ctrl = ServoControllerNode::new_with_topics(
    "servo_cmd",    // Must match publisher
    "joint_cmd",    // Must match publisher
    "joint_states"  // Must match subscriber
)?;

// Limit command rate
use std::time::Instant;

let mut last_send = Instant::now();
let min_interval = Duration::from_millis(50);

if last_send.elapsed() >= min_interval {
    servo_pub.send(cmd, None)?;
    last_send = Instant::now();
}

// Check node is added to runtime
runtime.add_node(servo_ctrl);

// Verify hub creation
let servo_pub = Hub::<ServoCommand>::new("servo_cmd")?;
eprintln!("Publisher created on topic: servo_cmd");
```

### Issue: Servos Move at Different Speeds

**Symptoms**: In multi-servo systems, servos don't reach targets simultaneously

**Possible Causes**:
1. Different velocity limits
2. Different distances to target
3. Non-synchronized commands

**Solutions**:

```rust
// Set uniform velocity limits
for i in 0..servo_count {
    servo_ctrl.set_velocity_limit(i, 2.0);
}

// Use JointCommand for synchronized motion
let mut joint_cmd = JointCommand::new();
for i in 0..6 {
    joint_cmd.add_position(&format!("joint_{}", i), targets[i])?;
}
joint_pub.send(joint_cmd, None)?;

// Calculate time-synchronized velocities
fn calculate_synchronized_velocities(
    current: &[f64],
    target: &[f64],
) -> Vec<f64> {
    // Find longest movement time needed
    let max_distance = current.iter()
        .zip(target.iter())
        .map(|(c, t)| (t - c).abs())
        .max_by(|a, b| a.partial_cmp(b).unwrap())
        .unwrap();

    // Calculate time for slowest servo
    let max_velocity = 2.0;
    let move_time = max_distance / max_velocity;

    // Set velocities so all servos finish together
    current.iter()
        .zip(target.iter())
        .map(|(c, t)| (t - c).abs() / move_time)
        .collect()
}
```

## Performance Considerations

### Update Rate

The servo controller processes commands and updates positions on every tick:

- **Recommended tick rate**: 50-100 Hz (10-20ms intervals)
- **Minimum tick rate**: 20 Hz (50ms intervals)
- **Maximum useful rate**: 200 Hz (5ms intervals)

Lower rates cause jerky motion; higher rates provide minimal improvement.

### Command Rate

- **Individual servos**: Up to 50 Hz per servo
- **Multi-servo coordination**: 20-50 Hz for JointCommand
- **Trajectory execution**: 10-30 Hz waypoint updates

### CPU Usage

- Minimal per servo: ~0.1% CPU per servo at 100 Hz
- 6-servo system: ~0.6% CPU
- Scales linearly with servo count
- Interpolation adds negligible overhead

### Memory Usage

Per-servo memory footprint:
- Position data: ~40 bytes per servo
- State tracking: ~32 bytes per servo
- Limits configuration: ~48 bytes per servo
- **Total**: ~120 bytes per servo

6-DOF system: ~720 bytes total

### Latency

- Command to motion: 10-50ms depending on tick rate
- Interpolated motion: Adds 0-20ms smoothing
- Network latency: Minimal (<1ms for local topics)

## Related Nodes

- **PidControllerNode**: Closed-loop position/velocity control
- **DifferentialDriveNode**: Mobile robot wheel control
- **ImuNode**: Orientation feedback for stabilization
- **PathPlannerNode**: Trajectory generation for arm motion

## See Also

- [Servo Motor Basics](https://en.wikipedia.org/wiki/Servomotor)
- [Robot Kinematics](https://en.wikipedia.org/wiki/Robot_kinematics)
- [Inverse Kinematics](https://en.wikipedia.org/wiki/Inverse_kinematics)
- [Motion Planning](https://en.wikipedia.org/wiki/Motion_planning)
- [JointCommand Message Type](../messages/control.rs)
- [ServoCommand Message Type](../messages/control.rs)
