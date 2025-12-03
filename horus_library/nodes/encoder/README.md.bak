# Encoder Node

Wheel and joint position feedback for odometry and motor control.

## Overview

The Encoder Node reads encoder data from wheels or joints and publishes position, velocity, and odometry information for robot navigation and control feedback. It supports both hardware encoders (in production) and simulation mode (for testing). The node calculates velocity from position changes, tracks total distance traveled, and publishes odometry messages for navigation systems.

This node is essential for closed-loop motor control, providing real-time feedback to PID controllers and other control systems. It can be used for wheel odometry in mobile robots, joint position feedback in manipulators, or any application requiring accurate position and velocity measurement.

## Topics

### Publishers

| Topic | Type | Description |
|-------|------|-------------|
| `odom` (default) | `Odometry` | Combined position, velocity, and odometry data |

## Configuration Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `encoder_resolution` | `f64` | `1024.0` | Pulses per revolution (PPR) of the encoder |
| `wheel_radius` | `f64` | `0.1` | Wheel radius in meters (0.1m = 10cm) |
| `gear_ratio` | `f64` | `1.0` | Gear ratio (motor rotations / wheel rotations) |
| `frame_id` | `String` | `"odom"` | Reference frame for odometry data |
| `child_frame_id` | `String` | `"base_link"` | Child frame identifier |

### Encoder Resolution

Common encoder resolutions:
- **Low resolution**: 48-128 PPR (suitable for low-speed applications)
- **Medium resolution**: 256-1024 PPR (general purpose robotics)
- **High resolution**: 2048-4096 PPR (precision positioning)

### Sample Rate

The encoder is read on every node tick. Recommended tick rates:
- **Wheel odometry**: 50-100 Hz (10-20ms)
- **Motor control feedback**: 100-500 Hz (2-10ms)
- **Precision positioning**: 500-1000 Hz (1-2ms)

## Message Types

### Odometry

The primary output message containing position, velocity, and odometry data:

```rust
pub struct Odometry {
    pub pose: Pose2D,              // Current pose estimate
    pub twist: Twist,              // Current velocity estimate
    pub pose_covariance: [f64; 36],   // 6x6 pose covariance matrix
    pub twist_covariance: [f64; 36],  // 6x6 twist covariance matrix
    pub frame_id: [u8; 32],        // Reference frame ("odom")
    pub child_frame_id: [u8; 32],  // Child frame ("base_link")
    pub timestamp: u64,            // Nanoseconds since epoch
}
```

### Pose2D

2D position and orientation:

```rust
pub struct Pose2D {
    pub x: f64,        // X position in meters
    pub y: f64,        // Y position in meters
    pub theta: f64,    // Orientation in radians
}
```

### Twist

Linear and angular velocities:

```rust
pub struct Twist {
    pub linear: [f64; 3],   // [x, y, z] linear velocity (m/s)
    pub angular: [f64; 3],  // [roll, pitch, yaw] angular velocity (rad/s)
}
```

## Public API

### Construction

```rust
use horus_library::nodes::EncoderNode;

// Create with default topic "odom"
let mut encoder = EncoderNode::new()?;

// Create with custom topic
let mut encoder = EncoderNode::new_with_topic("wheel_encoder")?;
```

### Configuration Methods

```rust
// Set encoder configuration
encoder.set_encoder_config(
    1024.0,  // encoder_resolution (PPR)
    0.05,    // wheel_radius (meters)
    1.0      // gear_ratio
);

// Set coordinate frame IDs
encoder.set_frame_ids("odom", "base_link");

// Reset encoder position and distance
encoder.reset();

// Set simulation velocity (for testing without hardware)
encoder.set_simulation_velocity(
    0.5,   // linear velocity (m/s)
    0.1    // angular velocity (rad/s)
);
```

### State Query Methods

```rust
// Get current velocity (m/s)
let velocity = encoder.get_velocity();

// Get total distance traveled (meters)
let distance = encoder.get_total_distance();
```

## Usage Examples

### Basic Motor Feedback

```rust
use horus_library::nodes::EncoderNode;
use horus_core::{Node, Runtime};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut runtime = Runtime::new()?;

    // Create encoder node for motor feedback
    let mut encoder = EncoderNode::new_with_topic("motor_encoder")?;

    // Configure for a 1024 PPR encoder on 6cm wheel
    encoder.set_encoder_config(
        1024.0,  // 1024 pulses per revolution
        0.06,    // 6cm wheel radius
        1.0      // direct drive (no gearing)
    );

    encoder.set_frame_ids("odom", "base_link");

    runtime.add_node(encoder);
    runtime.run()?;

    Ok(())
}
```

### Wheel Odometry for Mobile Robot

```rust
use horus_library::nodes::EncoderNode;
use horus_core::{Node, Runtime};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut runtime = Runtime::new()?;

    // Left wheel encoder
    let mut left_encoder = EncoderNode::new_with_topic("left_wheel_odom")?;
    left_encoder.set_encoder_config(
        2048.0,  // High-resolution encoder
        0.075,   // 7.5cm wheel radius
        1.0      // Direct drive
    );
    left_encoder.set_frame_ids("odom", "left_wheel");

    // Right wheel encoder
    let mut right_encoder = EncoderNode::new_with_topic("right_wheel_odom")?;
    right_encoder.set_encoder_config(
        2048.0,  // High-resolution encoder
        0.075,   // 7.5cm wheel radius
        1.0      // Direct drive
    );
    right_encoder.set_frame_ids("odom", "right_wheel");

    runtime.add_node(left_encoder);
    runtime.add_node(right_encoder);
    runtime.run()?;

    Ok(())
}
```

### Geared Motor System

```rust
use horus_library::nodes::EncoderNode;
use horus_core::{Node, Runtime};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut runtime = Runtime::new()?;

    // Encoder on motor shaft with 10:1 gearbox
    let mut encoder = EncoderNode::new_with_topic("geared_motor")?;

    encoder.set_encoder_config(
        512.0,   // 512 PPR encoder on motor shaft
        0.05,    // 5cm output wheel
        10.0     // 10:1 gear reduction (motor spins 10x faster)
    );

    encoder.set_frame_ids("odom", "base_link");

    runtime.add_node(encoder);
    runtime.run()?;

    Ok(())
}
```

### Encoder with PID Control

```rust
use horus_library::nodes::{EncoderNode, PidControllerNode};
use horus_core::{Node, Runtime};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut runtime = Runtime::new()?;

    // Encoder provides feedback
    let mut encoder = EncoderNode::new_with_topic("encoder_velocity")?;
    encoder.set_encoder_config(1024.0, 0.05, 1.0);

    // PID controller uses encoder feedback
    let mut pid = PidControllerNode::new_with_topics(
        "target_velocity",
        "encoder_velocity",  // Subscribe to encoder output
        "motor_command",
        "pid_config"
    )?;
    pid.set_gains(1.5, 0.2, 0.05);

    runtime.add_node(encoder);
    runtime.add_node(pid);
    runtime.run()?;

    Ok(())
}
```

### Simulation Mode Testing

```rust
use horus_library::nodes::EncoderNode;
use horus_core::{Node, Runtime};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut runtime = Runtime::new()?;

    // Create encoder for simulation
    let mut encoder = EncoderNode::new()?;
    encoder.set_encoder_config(1024.0, 0.1, 1.0);

    // Set simulated velocity for testing
    encoder.set_simulation_velocity(
        0.5,   // 0.5 m/s linear
        0.0    // 0 rad/s angular
    );

    runtime.add_node(encoder);
    runtime.run()?;

    Ok(())
}
```

## Encoder Types

### Quadrature Encoders

Quadrature encoders provide two channels (A and B) that are 90 degrees out of phase, enabling:

- **Direction detection**: By comparing phase relationship of A and B channels
- **Higher resolution**: 4x multiplication (count on rising and falling edges of both channels)
- **Noise immunity**: Differential signaling reduces electrical noise

**Effective Resolution**:
```
Effective PPR = Nominal PPR × 4 (with quadrature decoding)
Example: 256 PPR encoder  1024 counts per revolution
```

**Configuration for quadrature encoder**:
```rust
encoder.set_encoder_config(
    1024.0,  // Use effective PPR (nominal × 4)
    0.05,
    1.0
);
```

### Single Channel Encoders

Single channel encoders provide only one pulse train:

- **Direction**: Requires separate direction signal or assumes unidirectional motion
- **Resolution**: Count only rising edges (or both edges for 2x)
- **Simpler hardware**: Lower cost, fewer pins

**Configuration for single channel**:
```rust
encoder.set_encoder_config(
    256.0,   // Nominal PPR (no multiplication)
    0.05,
    1.0
);
```

### Absolute vs Incremental

**Incremental Encoders** (this node):
- Measure relative position change
- Require homing/zeroing procedure
- Lower cost, higher resolution
- Position lost on power cycle

**Absolute Encoders** (not covered by this node):
- Know absolute position at power-on
- No homing required
- Higher cost
- Typically lower resolution

## Velocity Calculation

The encoder node calculates velocity using the finite difference method:

```
velocity = (current_position - last_position) / dt

where:
  current_position: Encoder reading converted to linear distance
  last_position: Previous encoder reading
  dt: Time delta between readings
```

### Distance Conversion

Encoder pulses are converted to linear distance:

```
distance = (pulse_count / encoder_resolution) × (2 × π × wheel_radius) / gear_ratio

where:
  pulse_count: Number of encoder pulses
  encoder_resolution: Pulses per revolution (PPR)
  wheel_radius: Radius in meters
  gear_ratio: Motor rotations per wheel rotation
```

### Velocity Smoothing

For noisy encoder signals, consider:

1. **Low-pass filtering**: Add external filtering node
2. **Higher sample rate**: Reduce quantization error
3. **Higher resolution encoder**: Reduce position quantization
4. **Moving average**: Average over multiple samples

## Troubleshooting

### Issue: Missed Counts

**Symptoms**:
- Velocity readings lower than expected
- Distance traveled less than actual
- Intermittent zero readings

**Causes**:
- Sample rate too low for encoder speed
- Electrical noise on encoder signals
- Loose encoder connections
- Encoder resolution too high for CPU

**Solutions**:
```rust
// Increase sample rate (decrease tick interval)
// In your runtime configuration, increase update frequency

// Reduce effective resolution if necessary
encoder.set_encoder_config(
    512.0,   // Lower resolution
    0.05,
    1.0
);

// Add hardware filtering (pull-up resistors, shielded cables)
// Check encoder wiring and connections
```

**Rule of thumb**: Sample rate should be at least 10x the maximum encoder frequency:
```
max_encoder_freq = (max_wheel_speed / wheel_circumference) × encoder_resolution
min_sample_rate = max_encoder_freq × 10
```

### Issue: Incorrect Direction

**Symptoms**:
- Encoder counts backward when motor goes forward
- Velocity has wrong sign

**Causes**:
- Encoder wiring reversed
- A/B channels swapped (quadrature encoder)

**Solutions**:
```rust
// Software fix: Invert velocity reading
let corrected_velocity = -encoder.get_velocity();

// Hardware fix: Swap A and B encoder channels
// Or: Reverse encoder mounting direction
```

### Issue: Noisy Velocity Readings

**Symptoms**:
- Velocity fluctuates rapidly
- Jittery velocity values at constant speed

**Causes**:
- Low encoder resolution
- Quantization error at low speeds
- Electrical noise
- Variable sample rate

**Solutions**:
```rust
// Use higher resolution encoder
encoder.set_encoder_config(
    2048.0,  // Higher resolution reduces noise
    0.05,
    1.0
);

// Implement velocity filtering in separate node
// Ensure consistent sample timing
// Add hardware debouncing/filtering
```

### Issue: Velocity Always Zero

**Symptoms**:
- Encoder distance increases but velocity reads zero
- No velocity output

**Causes**:
- Time delta calculation error
- Sample rate too high (dt  0)
- Integer overflow in position calculation

**Solutions**:
```rust
// Check tick rate is reasonable (not too fast)
// Verify encoder is actually moving
// Reset encoder state
encoder.reset();

// Check for correct velocity reading after reset
```

### Issue: Distance Drifts Over Time

**Symptoms**:
- Cumulative distance error
- Position drifts even when stationary

**Causes**:
- Encoder slippage
- Wheel slippage
- Vibration-induced false counts
- Temperature effects on encoder

**Solutions**:
```rust
// Periodically reset encoder when at known position
encoder.reset();

// Use external absolute position reference (GPS, visual markers)
// Implement sensor fusion with IMU
// Add mechanical coupling to prevent slippage
```

## Integration with Motor Control

### Closed-Loop Velocity Control

```rust
use horus_library::nodes::{EncoderNode, PidControllerNode};
use horus_core::{Node, Runtime, Hub};

fn setup_velocity_control() -> Result<(), Box<dyn std::error::Error>> {
    let mut runtime = Runtime::new()?;

    // Encoder measures actual velocity
    let mut encoder = EncoderNode::new_with_topic("measured_velocity")?;
    encoder.set_encoder_config(1024.0, 0.05, 1.0);

    // PID controls motor to match target velocity
    let mut pid = PidControllerNode::new_with_topics(
        "target_velocity",
        "measured_velocity",
        "motor_pwm",
        "pid_config"
    )?;
    pid.set_gains(1.5, 0.2, 0.05);
    pid.set_output_limits(-255.0, 255.0);

    runtime.add_node(encoder);
    runtime.add_node(pid);
    runtime.run()?;

    Ok(())
}
```

### Position Control

```rust
use horus_library::nodes::{EncoderNode, PidControllerNode};
use horus_core::{Node, Runtime};

fn setup_position_control() -> Result<(), Box<dyn std::error::Error>> {
    let mut runtime = Runtime::new()?;

    // Encoder measures position
    let mut encoder = EncoderNode::new_with_topic("current_position")?;
    encoder.set_encoder_config(2048.0, 0.05, 1.0);

    // PID controls to reach target position
    let mut pid = PidControllerNode::new_with_topics(
        "target_position",
        "current_position",
        "motor_command",
        "pid_config"
    )?;
    pid.set_gains(2.0, 0.1, 0.3);
    pid.set_output_limits(-100.0, 100.0);

    runtime.add_node(encoder);
    runtime.add_node(pid);
    runtime.run()?;

    Ok(())
}
```

### Differential Drive Odometry

```rust
use horus_library::nodes::EncoderNode;
use horus_core::{Node, Runtime};

fn setup_differential_odometry() -> Result<(), Box<dyn std::error::Error>> {
    let mut runtime = Runtime::new()?;

    // Left wheel encoder
    let mut left_encoder = EncoderNode::new_with_topic("left_odom")?;
    left_encoder.set_encoder_config(1024.0, 0.05, 1.0);

    // Right wheel encoder
    let mut right_encoder = EncoderNode::new_with_topic("right_odom")?;
    right_encoder.set_encoder_config(1024.0, 0.05, 1.0);

    // DifferentialDriveNode can consume these odometry messages
    // to calculate robot pose and publish combined odometry

    runtime.add_node(left_encoder);
    runtime.add_node(right_encoder);
    runtime.run()?;

    Ok(())
}
```

## Performance Considerations

### CPU Usage

Minimal CPU usage per encoder:
- Position read: ~1-5 microseconds
- Velocity calculation: ~1-2 microseconds
- Message publishing: ~10-50 microseconds

**Total per tick**: ~15-60 microseconds per encoder

### Memory Usage

Fixed memory footprint per encoder node:
- Configuration: ~64 bytes
- State variables: ~48 bytes
- Publisher: ~32 bytes

**Total**: ~150 bytes per encoder node

### Recommended Tick Rates by Application

| Application | Tick Rate | Notes |
|-------------|-----------|-------|
| Wheel odometry | 50-100 Hz | Balance accuracy and CPU usage |
| Velocity feedback | 100-500 Hz | Higher rate for responsive control |
| Position control | 100-200 Hz | Moderate rate for stable control |
| High-speed motors | 500-1000 Hz | Fast motors need fast feedback |
| Low-speed applications | 10-50 Hz | Save CPU for slow-moving systems |

### Multi-Encoder Systems

For robots with multiple encoders (e.g., 4-wheel drive):

```rust
// All encoders run at same tick rate for synchronized readings
// Memory: 150 bytes × 4 = 600 bytes
// CPU: 60 μs × 4 = 240 μs per tick @ 100 Hz = 2.4% CPU usage
```

## Calibration

### Encoder Resolution Calibration

To verify encoder resolution:

1. **Mark a reference point** on the wheel
2. **Rotate exactly one revolution**
3. **Count pulses** received
4. **Compare to nominal specification**

```rust
// If actual count differs from specification:
encoder.set_encoder_config(
    actual_pulse_count,  // Use measured value
    0.05,
    1.0
);
```

### Wheel Radius Calibration

To calibrate wheel radius for accurate distance:

1. **Mark starting position**
2. **Drive robot a known distance** (e.g., 10 meters)
3. **Read encoder distance**
4. **Calculate actual radius**:

```
actual_radius = (measured_distance / 10.0) × nominal_radius
```

5. **Update configuration**:
```rust
encoder.set_encoder_config(
    1024.0,
    actual_radius,  // Calibrated radius
    1.0
);
```

### Gear Ratio Calibration

For geared systems:

1. **Count motor shaft rotations** for one wheel rotation
2. **Calculate gear ratio**:

```
gear_ratio = motor_rotations / wheel_rotations
```

3. **Update configuration**:
```rust
encoder.set_encoder_config(
    1024.0,
    0.05,
    measured_gear_ratio  // Actual gear ratio
);
```

## Implementation Details

### Hardware Interface

In production systems, the `read_encoder_position()` method would interface with actual encoder hardware:

```rust
// Example hardware interface (not in current simulation)
fn read_encoder_position(&self) -> f64 {
    // Read from hardware encoder via GPIO, SPI, or dedicated encoder IC
    // Convert pulse count to position in meters
    let pulse_count = hardware::read_encoder_pulses();
    let rotations = pulse_count as f64 / self.encoder_resolution;
    let wheel_rotations = rotations / self.gear_ratio;
    let distance = wheel_rotations * 2.0 * PI * self.wheel_radius;
    distance
}
```

### Simulation Mode

The current implementation includes simulation mode for testing:

```rust
// Synthetic encoder data generation
fn read_encoder_position(&self) -> f64 {
    let current_time = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap()
        .as_millis() as f64 / 1000.0;

    current_time * self.sim_velocity
}
```

This allows testing control systems without physical hardware.

## Related Nodes

- **PidControllerNode**: Uses encoder feedback for closed-loop control
- **DifferentialDriveNode**: Combines encoder data from two wheels for robot odometry
- **MotorControlNode**: Receives encoder feedback for commutation and control
- **LocalizationNode**: Fuses encoder odometry with other sensors

## See Also

- [Quadrature Encoder Tutorial](https://en.wikipedia.org/wiki/Rotary_encoder)
- [Odometry for Mobile Robots](https://en.wikipedia.org/wiki/Odometry)
- [PID Control with Encoder Feedback](https://en.wikipedia.org/wiki/PID_controller)
- [Encoder Signal Conditioning](https://www.motioncontroltips.com/encoder-signal-conditioning/)
