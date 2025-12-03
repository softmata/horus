# PID Controller Node

Generic PID controller for position, velocity, or any feedback control application.

## Overview

The PID Controller Node implements a standard PID (Proportional-Integral-Derivative) control algorithm. It subscribes to setpoint and feedback values, calculates the control output, and publishes motor commands.

## Architecture

**This node is a thin wrapper** around the pure algorithm in `horus_library/algorithms/`:

- **`algorithms::pid::PID`** - PID feedback control algorithm with anti-windup and deadband

The node handles:
- Topic subscription/publishing (Hub I/O)
- Setpoint and feedback reception
- Motor command publishing
- Configuration updates
- Motor ID management

The algorithm handles:
- PID computation (P, I, D terms)
- Anti-windup (integral limiting)
- Output limiting
- Deadband application
- Error tracking

This separation enables PID algorithm reuse in different control contexts.

## Topics

### Subscribers

| Topic | Type | Description |
|-------|------|-------------|
| `setpoint` | `f32` | Target value for the controller |
| `feedback` | `f32` | Current measurement from sensor |
| `pid_config` | `PidConfig` | Runtime PID parameter updates |

### Publishers

| Topic | Type | Description |
|-------|------|-------------|
| `pid_output` | `MotorCommand` | Control output command |

## Configuration Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `kp` | `f32` | `1.0` | Proportional gain |
| `ki` | `f32` | `0.1` | Integral gain |
| `kd` | `f32` | `0.05` | Derivative gain |
| `output_min` | `f32` | `-100.0` | Minimum output value |
| `output_max` | `f32` | `100.0` | Maximum output value |
| `integral_min` | `f32` | `-50.0` | Minimum integral value (anti-windup) |
| `integral_max` | `f32` | `50.0` | Maximum integral value (anti-windup) |
| `deadband` | `f32` | `0.01` | Error threshold below which output is zero |
| `motor_id` | `u8` | `0` | Motor identifier for output commands |

## Message Types

### PidConfig

Runtime configuration message for updating PID parameters:

```rust
pub struct PidConfig {
    pub kp: f64,
    pub ki: f64,
    pub kd: f64,
}
```

### MotorCommand

Output command message:

```rust
pub enum MotorCommand {
    Velocity { motor_id: u8, value: f64 },
    Position { motor_id: u8, value: f64 },
    Torque { motor_id: u8, value: f64 },
}
```

## Public API

### Construction

```rust
use horus_library::nodes::PidControllerNode;

// Create with default topics
let mut pid = PidControllerNode::new()?;

// Create with custom topics
let mut pid = PidControllerNode::new_with_topics(
    "target_position",      // setpoint topic
    "encoder_position",     // feedback topic
    "motor_command",        // output topic
    "pid_params"            // config topic
)?;
```

### Configuration Methods

```rust
// Set PID gains
pid.set_gains(2.0, 0.5, 0.1);

// Set output limits
pid.set_output_limits(-255.0, 255.0);

// Set integral limits (anti-windup)
pid.set_integral_limits(-100.0, 100.0);

// Set error deadband
pid.set_deadband(0.05);

// Set motor ID for output commands
pid.set_motor_id(1);

// Reset PID controller state
pid.reset();

// Get current state (setpoint, feedback, error, integral)
let (setpoint, feedback, error, integral) = pid.get_state();
```

## Usage Examples

### Basic Position Control

```rust
use horus_library::nodes::PidControllerNode;
use horus_core::{Node, Runtime};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut runtime = Runtime::new()?;

    // Create PID controller for motor position control
    let mut pid = PidControllerNode::new_with_topics(
        "target_position",
        "encoder_position",
        "motor_command",
        "pid_config"
    )?;

    // Configure for position control
    pid.set_gains(2.0, 0.5, 0.1);
    pid.set_output_limits(-100.0, 100.0);
    pid.set_motor_id(0);

    runtime.add_node(pid);
    runtime.run()?;

    Ok(())
}
```

### Velocity Control

```rust
use horus_library::nodes::PidControllerNode;
use horus_core::{Node, Runtime};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut runtime = Runtime::new()?;

    // Create PID controller for velocity control
    let mut pid = PidControllerNode::new_with_topics(
        "target_velocity",
        "measured_velocity",
        "motor_pwm",
        "pid_config"
    )?;

    // Configure for velocity control (typically needs less integral gain)
    pid.set_gains(1.5, 0.1, 0.05);
    pid.set_output_limits(-255.0, 255.0);
    pid.set_integral_limits(-50.0, 50.0);
    pid.set_motor_id(0);

    runtime.add_node(pid);
    runtime.run()?;

    Ok(())
}
```

### Temperature Control

```rust
use horus_library::nodes::PidControllerNode;
use horus_core::{Node, Runtime};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut runtime = Runtime::new()?;

    // Create PID controller for temperature control
    let mut pid = PidControllerNode::new_with_topics(
        "target_temperature",
        "current_temperature",
        "heater_power",
        "pid_config"
    )?;

    // Configure for temperature control (slower system, more integral)
    pid.set_gains(5.0, 1.0, 0.5);
    pid.set_output_limits(0.0, 100.0);  // 0-100% heater power
    pid.set_integral_limits(-20.0, 20.0);
    pid.set_deadband(0.5);  // 0.5 degree deadband

    runtime.add_node(pid);
    runtime.run()?;

    Ok(())
}
```

### Multiple PID Controllers

```rust
use horus_library::nodes::PidControllerNode;
use horus_core::{Node, Runtime};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut runtime = Runtime::new()?;

    // Left wheel velocity controller
    let mut left_pid = PidControllerNode::new_with_topics(
        "left_target_vel",
        "left_measured_vel",
        "left_motor_cmd",
        "left_pid_config"
    )?;
    left_pid.set_gains(1.5, 0.2, 0.05);
    left_pid.set_motor_id(0);

    // Right wheel velocity controller
    let mut right_pid = PidControllerNode::new_with_topics(
        "right_target_vel",
        "right_measured_vel",
        "right_motor_cmd",
        "right_pid_config"
    )?;
    right_pid.set_gains(1.5, 0.2, 0.05);
    right_pid.set_motor_id(1);

    runtime.add_node(left_pid);
    runtime.add_node(right_pid);
    runtime.run()?;

    Ok(())
}
```

## PID Tuning Guide

### Understanding PID Terms

- **Proportional (Kp)**: Responds to current error. Higher values = faster response, but can cause oscillation.
- **Integral (Ki)**: Eliminates steady-state error. Higher values = faster convergence, but can cause overshoot.
- **Derivative (Kd)**: Dampens oscillations. Higher values = less overshoot, but sensitive to noise.

### Tuning Process (Ziegler-Nichols Method)

1. **Set Ki and Kd to zero**
   ```rust
   pid.set_gains(1.0, 0.0, 0.0);
   ```

2. **Increase Kp until oscillation**
   - Gradually increase Kp until system oscillates
   - Record the Kp value (Ku) and oscillation period (Tu)

3. **Calculate initial gains**
   ```
   Kp = 0.6 * Ku
   Ki = 2.0 * Kp / Tu
   Kd = Kp * Tu / 8.0
   ```

4. **Fine-tune**
   - Too much overshoot? Decrease Kp, increase Kd
   - Slow convergence? Increase Ki
   - Oscillation? Decrease Kp, increase Kd

### Common Applications and Typical Gains

| Application | Kp | Ki | Kd | Notes |
|-------------|----|----|-----|-------|
| Motor Position | 2.0-5.0 | 0.1-1.0 | 0.05-0.5 | Fast response needed |
| Motor Velocity | 1.0-3.0 | 0.1-0.5 | 0.01-0.1 | Less integral gain |
| Temperature | 5.0-20.0 | 1.0-5.0 | 0.5-2.0 | Slow system, more integral |
| Pressure | 3.0-10.0 | 0.5-2.0 | 0.2-1.0 | Medium response |

## Anti-Windup

The integral term can accumulate ("wind up") when the output is saturated, causing overshoot. This node implements anti-windup by clamping the integral value:

```rust
// Set integral limits to prevent windup
pid.set_integral_limits(-50.0, 50.0);
```

**Guidelines**:
- Set integral limits to about 50% of output limits
- For systems with frequent saturation, use tighter integral limits
- Monitor integral value using `get_state()` during tuning

## Deadband

The deadband prevents output jitter when error is very small:

```rust
// Ignore errors smaller than 0.05
pid.set_deadband(0.05);
```

**When to use**:
- Noisy feedback signals
- Systems where small errors are acceptable
- To reduce actuator wear

## Performance Considerations

### Update Rate

The PID controller calculates output on every tick. For best performance:

- Position control: 50-200 Hz (5-20ms tick rate)
- Velocity control: 100-500 Hz (2-10ms tick rate)
- Temperature control: 1-10 Hz (100-1000ms tick rate)

### CPU Usage

Minimal CPU usage - single multiplication and addition per term.

### Memory Usage

Small fixed memory footprint (~100 bytes).

## Troubleshooting

### Issue: Output oscillates

**Cause**: Kp too high or Kd too low

**Solution**:
```rust
// Reduce Kp by 20-50%
pid.set_gains(kp * 0.7, ki, kd);

// Or increase Kd
pid.set_gains(kp, ki, kd * 1.5);
```

### Issue: Steady-state error

**Cause**: Ki too low or integral limits too tight

**Solution**:
```rust
// Increase Ki
pid.set_gains(kp, ki * 2.0, kd);

// Or relax integral limits
pid.set_integral_limits(-100.0, 100.0);
```

### Issue: Slow response

**Cause**: Kp too low

**Solution**:
```rust
// Increase Kp by 20-50%
pid.set_gains(kp * 1.3, ki, kd);
```

### Issue: Output saturates

**Cause**: Limits too restrictive or gains too high

**Solution**:
```rust
// Increase output limits
pid.set_output_limits(-200.0, 200.0);

// Or reduce gains
pid.set_gains(kp * 0.8, ki * 0.8, kd);
```

## Implementation Details

### Control Loop Calculation

The PID output is calculated as:

```
error = setpoint - feedback
output = Kp * error + Ki * integral + Kd * derivative

where:
  integral = sum(error * dt)  [clamped to integral_min/max]
  derivative = (error - last_error) / dt
```

### Deadband Application

```
effective_error = (abs(error) < deadband) ? 0 : error
```

### Output Limiting

```
output = clamp(output, output_min, output_max)
```

## Related Nodes

- **EncoderNode**: Provides position/velocity feedback
- **DifferentialDriveNode**: Uses PID controllers internally
- **ServoControllerNode**: Alternative for servo motors

## See Also

- [PID Control Theory](https://en.wikipedia.org/wiki/PID_controller)
- [Ziegler-Nichols Tuning](https://en.wikipedia.org/wiki/Ziegler%E2%80%93Nichols_method)
- [Anti-windup Techniques](https://en.wikipedia.org/wiki/Integral_windup)
