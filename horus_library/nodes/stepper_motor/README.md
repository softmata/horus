# Stepper Motor Node

Step/direction stepper motor controller with trapezoidal motion profiling, multi-motor support, and microstepping for precision positioning systems.

## Overview

The Stepper Motor Node provides precise position and velocity control for stepper motors using step/direction interface. It supports up to 8 motors simultaneously with independent control, microstepping, gear ratios, acceleration ramping, and homing sequences with automatic hardware/simulation fallback.

Compatible with A4988, DRV8825, TMC2208, TMC2209, TB6600, and other step/direction stepper drivers.

Key features:
- Multi-motor support (up to 8 motors)
- Multiple control modes: steps, position, velocity, homing
- Trapezoidal motion profiling with acceleration/deceleration ramping
- Microstepping support (1x to 256x)
- Gear ratio compensation
- Position tracking and feedback
- Current limiting (driver-dependent)
- Emergency stop handling
- Simulation fallback when hardware unavailable
- Hardware GPIO control via sysfs

## Topics

### Subscribers

| Topic | Type | Description |
|-------|------|-------------|
| `stepper_cmd` | `StepperCommand` | Stepper motor control commands |

### Publishers

| Topic | Type | Description |
|-------|------|-------------|
| `stepper_cmd_feedback` | `StepperCommand` | Command echo for monitoring |
| `stepper_cmd_position` | `MotorCommand` | Position and velocity feedback |

## Configuration Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `num_motors` | `u8` | `1` | Number of stepper motors (1-8) |
| `steps_per_rev` | `u32` | `200` | Full steps per revolution (per motor) |
| `microsteps` | `u16` | `16` | Microstepping divisor (1, 2, 4, 8, 16, 32, etc.) |
| `gear_ratio` | `f64` | `1.0` | Gear reduction ratio output/input (per motor) |
| `max_velocity` | `f64` | `1000.0` | Maximum velocity in steps/sec (per motor) |
| `max_acceleration` | `f64` | `500.0` | Maximum acceleration in steps/sec^2 (per motor) |
| `current_limit` | `u16` | `0` | Current limit in milliamps (0=driver default) |
| `invert_direction` | `bool` | `false` | Invert direction pin logic (per motor) |
| `enable_feedback` | `bool` | `true` | Enable position feedback publishing |
| `command_timeout_ms` | `u64` | `5000` | Command timeout in milliseconds (0=disable) |
| `step_pulse_duration_us` | `u64` | `5` | Step pulse width in microseconds (1-20) |

### Hardware GPIO Pins

Configure GPIO pins for each motor (Raspberry Pi BCM numbering):

```rust
stepper.set_gpio_pins(motor_id, step_pin, dir_pin, enable_pin);
```

Example pin mapping:
- Motor 0: GPIO 17 (step), GPIO 18 (dir), GPIO 27 (enable)
- Motor 1: GPIO 22 (step), GPIO 23 (dir), GPIO 24 (enable)
- Motor 2: GPIO 5 (step), GPIO 6 (dir), GPIO 13 (enable)
- Motor 3: GPIO 19 (step), GPIO 26 (dir), GPIO 21 (enable)

**IMPORTANT**: Most stepper drivers use 3.3V or 5V logic. Verify your driver's logic voltage before wiring.

## Message Types

### StepperCommand

Stepper motor control message:

```rust
pub struct StepperCommand {
    pub motor_id: u8,          // Motor ID (0-7)
    pub mode: u8,              // Control mode
    pub target: f64,           // Target value (mode-dependent)
    pub max_velocity: f64,     // Maximum velocity in steps/sec (0=use default)
    pub acceleration: f64,     // Acceleration in steps/sec^2 (0=use default)
    pub enable: bool,          // Enable motor
    pub microsteps: u16,       // Microstepping divisor (1, 2, 4, 8, 16, 32, 64, 128, 256)
    pub current_limit: u16,    // Current limit in mA (0=use default)
    pub timestamp: u64,        // Command timestamp (ns since epoch)
}
```

**Control Modes**:
- `StepperCommand::MODE_STEPS = 0` - Relative steps movement
- `StepperCommand::MODE_POSITION = 1` - Absolute position in radians
- `StepperCommand::MODE_VELOCITY = 2` - Continuous velocity in steps/sec
- `StepperCommand::MODE_HOMING = 3` - Homing sequence

### MotorCommand (Feedback)

Position and velocity feedback message:

```rust
pub struct MotorCommand {
    pub motor_id: u8,          // Motor ID
    pub mode: u8,              // MODE_POSITION = 1
    pub target: f64,           // Current position in radians
    pub max_velocity: f64,     // Current velocity in steps/sec
    pub max_acceleration: f64, // Current acceleration setting
    pub feed_forward: f64,     // Always 0.0 for stepper
    pub enable: bool,          // Motor enable state
    pub timestamp: u64,        // Feedback timestamp (ns since epoch)
}
```

## Public API

### Construction

```rust
use horus_library::nodes::StepperMotorNode;

// Create with default topic "stepper_cmd"
let mut stepper = StepperMotorNode::new()?;

// Create with custom topic
let mut stepper = StepperMotorNode::new_with_topic("robot.steppers")?;
```

### Configuration Methods

```rust
// Set number of motors (1-8)
stepper.set_num_motors(4);

// Configure motor parameters
stepper.set_steps_per_revolution(0, 200, 16);  // motor_id, steps, microsteps

// Set gear ratio (output/input)
stepper.set_gear_ratio(0, 5.0);  // 5:1 reduction

// Set maximum velocity in steps/sec
stepper.set_max_velocity(0, 2000.0);

// Set acceleration in steps/sec^2
stepper.set_acceleration(0, 5000.0);

// Set current limit in milliamps (driver-dependent)
stepper.set_current_limit(0, 1500);  // 1.5A

// Invert direction pin logic
stepper.set_direction_inverted(0, true);

// Configure GPIO pins for a motor
stepper.set_gpio_pins(0, 17, 18, 27);  // motor_id, step_pin, dir_pin, enable_pin

// Set step pulse duration in microseconds
stepper.set_step_pulse_duration(5);  // 5us (1-20us range)

// Set command timeout in milliseconds (0=disable)
stepper.set_command_timeout(5000);  // 5 seconds

// Enable/disable position feedback publishing
stepper.set_feedback_enabled(true);
```

### Query Methods

```rust
// Get current position in microsteps
if let Some(position) = stepper.get_position(0) {
    println!("Position: {} microsteps", position);
}

// Get current position in radians
if let Some(position_rad) = stepper.get_position_radians(0) {
    println!("Position: {:.3} rad", position_rad);
}

// Get current velocity in steps/sec
if let Some(velocity) = stepper.get_velocity(0) {
    println!("Velocity: {:.1} steps/sec", velocity);
}

// Check if motor is enabled
if stepper.is_enabled(0) {
    println!("Motor 0 is enabled");
}

// Check if motor has been homed
if stepper.is_homed(0) {
    println!("Motor 0 is homed");
}
```

### Command Methods

```rust
// Set home position to current position
stepper.set_home_position(0);

// Zero the current position
stepper.zero_position(0);

// Emergency stop all motors
stepper.emergency_stop();
```

## Usage Examples

### Single Stepper Motor

```rust
use horus_library::prelude::*;

fn main() -> Result<()> {
    let mut scheduler = Scheduler::new();

    // Create stepper motor node
    let mut stepper = StepperMotorNode::new()?;

    // Configure motor 0 (NEMA 17 with A4988 driver)
    stepper.set_num_motors(1);
    stepper.set_steps_per_revolution(0, 200, 16);  // 200 steps, 1/16 microstepping
    stepper.set_gpio_pins(0, 17, 18, 27);  // step=GPIO17, dir=GPIO18, enable=GPIO27
    stepper.set_max_velocity(0, 2000.0);   // 2000 steps/sec
    stepper.set_acceleration(0, 5000.0);   // 5000 steps/sec^2
    stepper.set_current_limit(0, 1200);    // 1.2A

    scheduler.add(Box::new(stepper), 1, Some(true));
    scheduler.run()?;
    Ok(())
}
```

### Relative Movement (Steps)

```rust
use horus_library::prelude::*;

fn main() -> Result<()> {
    let mut scheduler = Scheduler::new();

    let mut stepper = StepperMotorNode::new()?;
    stepper.set_num_motors(1);
    stepper.set_steps_per_revolution(0, 200, 16);
    stepper.set_gpio_pins(0, 17, 18, 27);

    scheduler.add(Box::new(stepper), 1, Some(true));

    // Control node to send movement commands
    let controller = node! {
        name: "stepper_controller",
        tick: |ctx| {
            let hub = Hub::<StepperCommand>::new("stepper_cmd")?;

            // Move 1000 steps forward
            let cmd = StepperCommand::steps(0, 1000);
            hub.send(cmd, &mut None)?;

            std::thread::sleep(std::time::Duration::from_secs(2));

            // Move 1000 steps backward
            let cmd = StepperCommand::steps(0, -1000);
            hub.send(cmd, &mut None)?;

            Ok(())
        }
    };

    scheduler.add(Box::new(controller), 2, Some(false));
    scheduler.run()?;
    Ok(())
}
```

### Absolute Position Control

```rust
use horus_library::prelude::*;
use std::f64::consts::TAU;

fn main() -> Result<()> {
    let mut scheduler = Scheduler::new();

    let mut stepper = StepperMotorNode::new()?;
    stepper.set_num_motors(1);
    stepper.set_steps_per_revolution(0, 200, 16);
    stepper.set_gpio_pins(0, 17, 18, 27);
    stepper.set_max_velocity(0, 3000.0);
    stepper.set_acceleration(0, 10000.0);

    scheduler.add(Box::new(stepper), 1, Some(true));

    let controller = node! {
        name: "position_controller",
        tick: |ctx| {
            let hub = Hub::<StepperCommand>::new("stepper_cmd")?;

            // Move to 5 revolutions (10*PI radians)
            let cmd = StepperCommand::position(0, 5.0 * TAU, 2000.0);
            hub.send(cmd, &mut None)?;

            std::thread::sleep(std::time::Duration::from_secs(3));

            // Return to zero position
            let cmd = StepperCommand::position(0, 0.0, 2000.0);
            hub.send(cmd, &mut None)?;

            Ok(())
        }
    };

    scheduler.add(Box::new(controller), 2, Some(false));
    scheduler.run()?;
    Ok(())
}
```

### Continuous Velocity Mode

```rust
use horus_library::prelude::*;

fn main() -> Result<()> {
    let mut scheduler = Scheduler::new();

    let mut stepper = StepperMotorNode::new()?;
    stepper.set_num_motors(1);
    stepper.set_steps_per_revolution(0, 200, 16);
    stepper.set_gpio_pins(0, 17, 18, 27);
    stepper.set_max_velocity(0, 3000.0);
    stepper.set_acceleration(0, 5000.0);

    scheduler.add(Box::new(stepper), 1, Some(true));

    let controller = node! {
        name: "velocity_controller",
        tick: |ctx| {
            let hub = Hub::<StepperCommand>::new("stepper_cmd")?;

            // Spin at 1000 steps/sec
            let cmd = StepperCommand::velocity(0, 1000.0);
            hub.send(cmd, &mut None)?;

            std::thread::sleep(std::time::Duration::from_secs(5));

            // Reverse at 500 steps/sec
            let cmd = StepperCommand::velocity(0, -500.0);
            hub.send(cmd, &mut None)?;

            std::thread::sleep(std::time::Duration::from_secs(5));

            // Stop
            let cmd = StepperCommand::velocity(0, 0.0);
            hub.send(cmd, &mut None)?;

            Ok(())
        }
    };

    scheduler.add(Box::new(controller), 2, Some(false));
    scheduler.run()?;
    Ok(())
}
```

### Homing Sequence

```rust
use horus_library::prelude::*;

fn main() -> Result<()> {
    let mut scheduler = Scheduler::new();

    let mut stepper = StepperMotorNode::new()?;
    stepper.set_num_motors(1);
    stepper.set_steps_per_revolution(0, 200, 16);
    stepper.set_gpio_pins(0, 17, 18, 27);
    stepper.set_max_velocity(0, 1000.0);
    stepper.set_acceleration(0, 2000.0);

    scheduler.add(Box::new(stepper), 1, Some(true));

    let controller = node! {
        name: "homing_controller",
        tick: |ctx| {
            let hub = Hub::<StepperCommand>::new("stepper_cmd")?;

            // Start homing (move backward at 500 steps/sec until limit switch)
            let cmd = StepperCommand::home(0, -500.0);
            hub.send(cmd, &mut None)?;

            ctx.log_info("Homing motor 0...");

            // In real implementation, wait for homing to complete
            // by monitoring is_homed() or limit switch GPIO

            Ok(())
        }
    };

    scheduler.add(Box::new(controller), 2, Some(false));
    scheduler.run()?;
    Ok(())
}
```

### Multi-Motor CNC System

```rust
use horus_library::prelude::*;

fn main() -> Result<()> {
    let mut scheduler = Scheduler::new();

    // Configure 3-axis CNC (X, Y, Z)
    let mut stepper = StepperMotorNode::new()?;
    stepper.set_num_motors(3);

    // X-axis (motor 0) - 5mm lead screw
    stepper.set_steps_per_revolution(0, 200, 16);
    stepper.set_gear_ratio(0, 1.0);
    stepper.set_max_velocity(0, 3000.0);
    stepper.set_acceleration(0, 10000.0);
    stepper.set_gpio_pins(0, 17, 18, 27);

    // Y-axis (motor 1) - 5mm lead screw
    stepper.set_steps_per_revolution(1, 200, 16);
    stepper.set_gear_ratio(1, 1.0);
    stepper.set_max_velocity(1, 3000.0);
    stepper.set_acceleration(1, 10000.0);
    stepper.set_gpio_pins(1, 22, 23, 24);

    // Z-axis (motor 2) - 2mm lead screw (slower)
    stepper.set_steps_per_revolution(2, 200, 16);
    stepper.set_gear_ratio(2, 1.0);
    stepper.set_max_velocity(2, 1500.0);
    stepper.set_acceleration(2, 5000.0);
    stepper.set_gpio_pins(2, 5, 6, 13);

    stepper.set_feedback_enabled(true);
    stepper.set_command_timeout(5000);

    scheduler.add(Box::new(stepper), 1, Some(true));

    // G-code interpreter
    let gcode = node! {
        name: "gcode_interpreter",
        tick: |ctx| {
            let hub = Hub::<StepperCommand>::new("stepper_cmd")?;

            // Example: Move all axes to home
            hub.send(StepperCommand::position(0, 0.0, 2000.0), None)?;  // X
            hub.send(StepperCommand::position(1, 0.0, 2000.0), None)?;  // Y
            hub.send(StepperCommand::position(2, 0.0, 1000.0), None)?;  // Z

            Ok(())
        }
    };

    scheduler.add(Box::new(gcode), 2, Some(false));
    scheduler.run()?;
    Ok(())
}
```

### Position Feedback Monitoring

```rust
use horus_library::prelude::*;

fn main() -> Result<()> {
    let mut scheduler = Scheduler::new();

    let mut stepper = StepperMotorNode::new()?;
    stepper.set_num_motors(2);
    stepper.set_steps_per_revolution(0, 200, 16);
    stepper.set_steps_per_revolution(1, 200, 16);
    stepper.set_gpio_pins(0, 17, 18, 27);
    stepper.set_gpio_pins(1, 22, 23, 24);
    stepper.set_feedback_enabled(true);

    scheduler.add(Box::new(stepper), 1, Some(true));

    // Monitor position feedback
    let monitor = node! {
        name: "position_monitor",
        tick: |ctx| {
            let hub = Hub::<MotorCommand>::new("stepper_cmd_position")?;

            while let Some(feedback) = hub.recv(None) {
                ctx.log_info(&format!(
                    "Motor {}: pos={:.3} rad, vel={:.1} steps/s, enabled={}",
                    feedback.motor_id,
                    feedback.target,
                    feedback.max_velocity,
                    feedback.enable
                ));
            }
            Ok(())
        }
    };

    scheduler.add(Box::new(monitor), 2, Some(true));
    scheduler.run()?;
    Ok(())
}
```

## Hardware Setup

### Wiring Diagram (A4988 Driver)

```
Raspberry Pi          A4988          Stepper Motor (NEMA 17)
GPIO 17       --->    STEP
GPIO 18       --->    DIR
GPIO 27       --->    ENABLE
3.3V          --->    VDD
GND           --->    GND
                      VMOT   <---    12V Power Supply (+)
                      GND    <---    12V Power Supply (-)
                      1A     <---    Coil A (Black)
                      1B     <---    Coil A (Green)
                      2A     <---    Coil B (Red)
                      2B     <---    Coil B (Blue)

Microstepping pins (MS1, MS2, MS3):
For 1/16 microstepping: MS1=HIGH, MS2=HIGH, MS3=HIGH
```

### DRV8825 Wiring

```
Raspberry Pi          DRV8825        Stepper Motor
GPIO 17       --->    STEP
GPIO 18       --->    DIR
GPIO 27       --->    ENABLE
3.3V          --->    VDD
GND           --->    GND
                      VMOT   <---    12-24V Power (+)
                      GND    <---    Power (-)
                      A1     <---    Coil A
                      A2     <---    Coil A
                      B1     <---    Coil B
                      B2     <---    Coil B

Microstepping (MODE0, MODE1, MODE2):
For 1/32 microstepping: M0=HIGH, M1=HIGH, M2=HIGH
```

### TMC2208 Wiring (UART Mode)

```
Raspberry Pi          TMC2208        Stepper Motor
GPIO 17       --->    STEP
GPIO 18       --->    DIR
GPIO 27       --->    EN
GPIO 14 (TX)  --->    PDN_UART
GPIO 15 (RX)  <---    PDN_UART
3.3V          --->    VIO
GND           --->    GND
                      VM     <---    12-24V Power (+)
                      GND    <---    Power (-)
                      B2     <---    Coil B
                      B1     <---    Coil B
                      A1     <---    Coil A
                      A2     <---    Coil A
```

### Current Limiting (A4988/DRV8825)

Adjust the potentiometer on the driver board:

```
Vref = I_max × 8 × R_sense

For A4988 (R_sense = 0.05Ω):
  Vref = I_max × 0.4

For DRV8825 (R_sense = 0.1Ω):
  Vref = I_max × 0.8

Example: 1.5A current limit
  A4988: Vref = 1.5 × 0.4 = 0.6V
  DRV8825: Vref = 1.5 × 0.8 = 1.2V
```

Measure Vref between GND and the potentiometer wiper using a multimeter.

### System Requirements

```bash
# Install GPIO library
sudo apt install libraspberrypi-dev

# Enable GPIO interface
sudo raspi-config
# Select: Interface Options -> GPIO -> Enable
```

### Enable in Project

```toml
[dependencies]
horus_library = { version = "0.1", features = ["gpio-hardware"] }
```

```bash
cargo build --features="gpio-hardware"
```

## Motion Profiling

### Trapezoidal Velocity Profile

The node automatically generates smooth trapezoidal velocity profiles:

```
Velocity
    ^
    |      /----------\
    |     /            \
    |    /              \
    |   /                \
    |  /                  \
    | /                    \
    |/______________________\___> Time
     Accel   Cruise   Decel
```

**Phases**:
1. **Acceleration**: Ramps up from current velocity to target velocity
2. **Cruise**: Maintains constant velocity (if distance allows)
3. **Deceleration**: Ramps down to stop at target position

**Timing Calculation**:
```
Acceleration time: t_accel = (v_max - v_0) / a
Acceleration distance: d_accel = v_0 * t + 0.5 * a * t^2
Deceleration distance: d_decel = v_max^2 / (2 * a)

If total distance < (d_accel + d_decel):
  No cruise phase - triangular profile
Else:
  Cruise distance = total - d_accel - d_decel
```

### Motion State Machine

```
                    +---------+
        Start  -->  |  Idle   |
                    +---------+
                         |
                    Target set
                         |
                         v
                  +-------------+
                  | Accelerating|
                  +-------------+
                         |
                  Reached target
                   velocity
                         |
                         v
                   +---------+
                   | Cruising|
                   +---------+
                         |
                   Need to stop/
                   change velocity
                         |
                         v
                  +-------------+
                  | Decelerating|
                  +-------------+
                         |
                  Reached target
                   position/velocity
                         |
                         v
                    +---------+
                    |  Idle   |
                    +---------+
```

## Microstepping Guide

### Common Microstepping Settings

| Mode | Steps/Rev (200 step motor) | Resolution | Torque | Smoothness | Noise |
|------|---------------------------|------------|--------|------------|-------|
| Full step | 200 | 1.8° | High | Low | High |
| Half step | 400 | 0.9° | Medium | Medium | Medium |
| 1/4 step | 800 | 0.45° | Medium | Good | Low |
| 1/8 step | 1600 | 0.225° | Medium-Low | Good | Low |
| 1/16 step | 3200 | 0.1125° | Low | Excellent | Very Low |
| 1/32 step | 6400 | 0.05625° | Low | Excellent | Very Low |

### Choosing Microstepping

```rust
// High torque, faster motion (rough)
stepper.set_steps_per_revolution(0, 200, 1);  // Full step

// Balanced performance (common default)
stepper.set_steps_per_revolution(0, 200, 16);  // 1/16 step

// Ultra-smooth, quiet (lower torque)
stepper.set_steps_per_revolution(0, 200, 32);  // 1/32 step

// Maximum resolution (TMC drivers)
stepper.set_steps_per_revolution(0, 200, 256);  // 1/256 step
```

### Driver Microstepping Configuration

**A4988** (up to 1/16):
- MS1=LOW, MS2=LOW, MS3=LOW: Full step
- MS1=HIGH, MS2=LOW, MS3=LOW: Half step
- MS1=LOW, MS2=HIGH, MS3=LOW: 1/4 step
- MS1=HIGH, MS2=HIGH, MS3=LOW: 1/8 step
- MS1=HIGH, MS2=HIGH, MS3=HIGH: 1/16 step

**DRV8825** (up to 1/32):
- M0=LOW, M1=LOW, M2=LOW: Full step
- M0=HIGH, M1=LOW, M2=LOW: Half step
- M0=LOW, M1=HIGH, M2=LOW: 1/4 step
- M0=HIGH, M1=HIGH, M2=LOW: 1/8 step
- M0=LOW, M1=LOW, M2=HIGH: 1/16 step
- M0=HIGH, M1=LOW, M2=HIGH: 1/32 step

**TMC2208/TMC2209** (up to 1/256):
Configure via UART or hardwired pins for advanced microstepping.

## Best Practices

1. **Always set realistic velocities**:
   ```rust
   // Start conservative, tune upward
   stepper.set_max_velocity(0, 1000.0);  // Start here
   stepper.set_max_velocity(0, 3000.0);  // After testing

   // Too fast = missed steps, motor stalling
   ```

2. **Use appropriate acceleration**:
   ```rust
   // Higher acceleration for light loads
   stepper.set_acceleration(0, 10000.0);

   // Lower acceleration for heavy loads
   stepper.set_acceleration(0, 2000.0);
   ```

3. **Enable position feedback for critical applications**:
   ```rust
   stepper.set_feedback_enabled(true);
   // Monitor position drift, verify motion completion
   ```

4. **Implement homing on startup**:
   ```rust
   // Find reference position using limit switch
   let cmd = StepperCommand::home(0, -500.0);
   hub.send(cmd, &mut None)?;
   ```

5. **Set current limits to prevent overheating**:
   ```rust
   // Match motor rating (typically 1.0A - 2.0A for NEMA 17)
   stepper.set_current_limit(0, 1500);  // 1.5A
   ```

6. **Use command timeouts for safety**:
   ```rust
   // Stop motors if commands stop coming
   stepper.set_command_timeout(5000);  // 5 seconds
   ```

7. **Balance microstepping vs performance**:
   ```rust
   // 1/16 is a good default for most applications
   stepper.set_steps_per_revolution(0, 200, 16);

   // Higher microstepping = smoother but slower max speed
   ```

8. **Account for gear ratios**:
   ```rust
   // 5:1 gear reduction
   stepper.set_gear_ratio(0, 5.0);

   // Position commands now account for gearing
   let cmd = StepperCommand::position(0, 6.28, 1000.0);  // 1 output revolution
   ```

## Troubleshooting

### "Hardware unavailable - using SIMULATION mode"

```
[WARN] StepperMotorNode motor 0: Hardware unavailable - using SIMULATION mode
[WARN]   Tried GPIO pins: step=17, dir=18, enable=27
[WARN]   Error: Permission denied
```

**Solutions:**
1. Check GPIO permissions: `ls -l /sys/class/gpio`
2. Install libraspberrypi-dev: `sudo apt install libraspberrypi-dev`
3. Enable GPIO: `sudo raspi-config -> Interface Options -> GPIO`
4. Verify wiring with multimeter
5. Rebuild with: `cargo build --features="gpio-hardware"`

### Motor vibrates but doesn't move

**Causes:**
- Incorrect wiring (coils swapped)
- Insufficient current
- Driver not enabled
- Wrong microstepping pins

**Solutions:**
1. Verify coil wiring (swap A+/A- or B+/B- pairs if needed)
2. Increase current limit via potentiometer
3. Check ENABLE pin is LOW (or connected correctly)
4. Verify microstepping pin configuration on driver
5. Try full step mode first: `stepper.set_steps_per_revolution(0, 200, 1)`

### Motor skips steps

**Causes:**
- Velocity too high
- Acceleration too aggressive
- Load too heavy
- Current too low
- Mechanical binding

**Solutions:**
1. Reduce max velocity:
   ```rust
   stepper.set_max_velocity(0, 1500.0);  // Lower from 3000
   ```
2. Reduce acceleration:
   ```rust
   stepper.set_acceleration(0, 3000.0);  // Lower from 10000
   ```
3. Increase current limit (adjust Vref potentiometer)
4. Check for mechanical issues (binding, misalignment)
5. Reduce load or use larger motor

### Motor gets hot

**Causes:**
- Current limit too high
- Motor running continuously
- Poor ventilation
- Driver heatsink inadequate

**Solutions:**
1. Reduce current limit:
   ```rust
   stepper.set_current_limit(0, 1000);  // Reduce from 2000mA
   ```
2. Disable motor when not in use:
   ```rust
   let cmd = StepperCommand::disable(0);
   hub.send(cmd, &mut None)?;
   ```
3. Add heatsinks to drivers and motors
4. Improve airflow around motors
5. Use TMC drivers with StealthChop for cooler operation

### Position drift over time

**Causes:**
- Missed steps
- Mechanical slippage
- Vibration/resonance
- No closed-loop feedback

**Solutions:**
1. Enable position feedback and monitor:
   ```rust
   stepper.set_feedback_enabled(true);
   // Log position periodically
   ```
2. Reduce speed/acceleration
3. Increase current
4. Add encoder for closed-loop control
5. Home periodically to reset position
6. Use dampers to reduce vibration

### Motor makes noise

**Causes:**
- Resonance at certain speeds
- Low microstepping
- Mechanical vibration
- Driver chopper frequency

**Solutions:**
1. Increase microstepping:
   ```rust
   stepper.set_steps_per_revolution(0, 200, 32);  // From 16 to 32
   ```
2. Use TMC drivers with StealthChop mode
3. Avoid resonant speeds (typically 100-200 Hz)
4. Add dampers to motor mounts
5. Adjust acceleration to pass through resonant frequencies quickly

### Emergency stop not working

**Solutions:**
1. Call emergency_stop() method:
   ```rust
   stepper.emergency_stop();  // Stops all motors immediately
   ```
2. Send disable command to all motors:
   ```rust
   for motor_id in 0..num_motors {
       let cmd = StepperCommand::disable(motor_id);
       hub.send(cmd, &mut None)?;
   }
   ```
3. Implement hardware E-STOP switch that cuts motor power

## Simulation Mode

When hardware is unavailable, the node operates in simulation mode:

```
[INFO] StepperMotorNode motor 0: (SIM) position 1234 steps, velocity 1000 steps/s
```

Simulated behavior:
- Tracks position and velocity mathematically
- No actual GPIO output
- Follows motion profile accurately
- Useful for algorithm testing without hardware
- Command processing identical to hardware mode

## Common Motor Specifications

### NEMA 17 Motors

| Parameter | Typical Value |
|-----------|---------------|
| Steps/rev | 200 (1.8° per step) |
| Voltage | 12V |
| Current | 1.0A - 2.0A per phase |
| Holding torque | 40-60 Nm·cm |
| Max speed | 1000-3000 RPM |

### NEMA 23 Motors

| Parameter | Typical Value |
|-----------|---------------|
| Steps/rev | 200 (1.8° per step) |
| Voltage | 24V |
| Current | 2.0A - 4.0A per phase |
| Holding torque | 120-200 Nm·cm |
| Max speed | 1000-2000 RPM |

### Configuration Example

```rust
// NEMA 17 (1.8°, 200 steps/rev)
stepper.set_steps_per_revolution(0, 200, 16);  // 1/16 microstepping = 3200 steps/rev
stepper.set_max_velocity(0, 2000.0);           // ~37.5 RPM at max
stepper.set_acceleration(0, 5000.0);
stepper.set_current_limit(0, 1200);            // 1.2A

// NEMA 23 (1.8°, 200 steps/rev, higher torque)
stepper.set_steps_per_revolution(1, 200, 8);   // 1/8 microstepping = 1600 steps/rev
stepper.set_max_velocity(1, 3000.0);           // ~112.5 RPM at max
stepper.set_acceleration(1, 10000.0);
stepper.set_current_limit(1, 2800);            // 2.8A
```

## See Also

- [BldcMotorNode](../bldc_motor/) - Brushless DC motor control
- [DynamixelNode](../dynamixel/) - Smart servo motors
- [EncoderNode](../encoder/) - Closed-loop position feedback
- [DifferentialDriveNode](../differential_drive/) - Mobile robot drive
- [PwmNode](../pwm/) - DC motor PWM control
