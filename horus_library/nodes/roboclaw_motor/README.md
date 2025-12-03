# Roboclaw Motor Controller Node

BasicMicro/Ion Motion Control Roboclaw dual-channel DC motor controller with encoder feedback, PID control, and current monitoring for high-performance mobile robotics.

## Overview

The Roboclaw Motor Controller Node provides comprehensive interface for BasicMicro Roboclaw motor controllers. It supports dual-channel DC motor control with quadrature encoder feedback, velocity and position control modes, PID tuning, current sensing, battery monitoring, and safety features. Supports hardware serial communication with automatic simulation fallback.

Supports all Roboclaw models: 2x7A, 2x15A, 2x30A, 2x45A, 2x60A, 2x160A, Solo, and ST series variants.

Key features:
- Dual motor control per controller (M1 and M2)
- Multiple control modes (duty cycle, velocity, position)
- Quadrature encoder feedback (up to 4M counts)
- PID velocity and position control
- Current sensing and limiting
- Battery voltage monitoring
- Temperature monitoring
- Hardware serial (UART/USB) interface
- Simulation fallback when hardware unavailable
- Support for up to 8 controllers on one bus (0x80-0x87)

## Topics

### Subscribers

| Topic | Type | Description |
|-------|------|-------------|
| `roboclaw/motor1/cmd` | `MotorCommand` | Motor 1 control commands |
| `roboclaw/motor2/cmd` | `MotorCommand` | Motor 2 control commands |
| `{serial_port}/rx` | `SerialData` | Serial receive data from Roboclaw |

### Publishers

| Topic | Type | Description |
|-------|------|-------------|
| `roboclaw/motor1/feedback` | `RoboclawFeedback` | Motor 1 encoder, velocity, current feedback |
| `roboclaw/motor2/feedback` | `RoboclawFeedback` | Motor 2 encoder, velocity, current feedback |
| `roboclaw/diagnostics` | `RoboclawDiagnostics` | Battery voltage, currents, temperatures, errors |
| `{serial_port}/tx` | `SerialData` | Serial transmit data to Roboclaw |

## Configuration Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `serial_port` | `String` | `/dev/ttyUSB0` | Serial port device path |
| `device_address` | `u8` | `0x80` | Roboclaw address (0x80-0x87 for up to 8 units) |
| `baud_rate` | `u32` | `38400` | Serial baud rate (2400-460800) |
| `timeout_ms` | `u32` | `100` | Serial communication timeout in milliseconds |
| `encoder_resolution` | `[u32; 2]` | `[1024, 1024]` | Encoder pulses per revolution for each motor |
| `gear_ratio` | `[f64; 2]` | `[1.0, 1.0]` | Gear ratio (motor:wheel) for each motor |
| `wheel_radius` | `[f64; 2]` | `[0.05, 0.05]` | Wheel radius in meters for each motor |
| `max_current` | `f64` | `30.0` | Maximum current limit in amperes |
| `max_velocity` | `[i32; 2]` | `[44000, 44000]` | Maximum velocity in QPPS (quad pulses per second) |
| `max_acceleration` | `[u32; 2]` | `[10000, 10000]` | Maximum acceleration in QPPS/second |

### PID Parameters

Configure PID gains for velocity and position control:

```rust
// Velocity PID
roboclaw.set_velocity_pid(motor_id, p, i, d, qpps);

// Position PID
roboclaw.set_position_pid(motor_id, p, i, d, qpps);
```

Default PID values:
- P: 1.0, I: 0.5, D: 0.25, QPPS: 44000

## Message Types

### MotorCommand

Motor control command message:

```rust
pub struct MotorCommand {
    pub motor_id: u8,           // Motor ID (1 or 2)
    pub mode: u8,               // Control mode
    pub target: f64,            // Target value (units depend on mode)
    pub max_velocity: f64,      // Maximum velocity (for position mode)
    pub max_acceleration: f64,  // Maximum acceleration
    pub feed_forward: f64,      // Feed-forward term
    pub enable: bool,           // Enable motor
    pub timestamp: u64,         // Timestamp (ns since epoch)
}
```

**Control Modes**:
- `MotorCommand::MODE_VELOCITY = 0`: Velocity control (m/s)
- `MotorCommand::MODE_POSITION = 1`: Position control (meters or radians)
- `MotorCommand::MODE_TORQUE = 2`: Torque control (Nm) - not fully implemented
- `MotorCommand::MODE_VOLTAGE = 3`: Duty cycle control (-1.0 to 1.0)

**Helper Methods**:
```rust
MotorCommand::velocity(motor_id, velocity_m_s)
MotorCommand::position(motor_id, position, max_velocity)
MotorCommand::stop(motor_id)
```

### RoboclawFeedback

Motor feedback message with encoder and current data:

```rust
pub struct RoboclawFeedback {
    pub motor_id: u8,           // 1 or 2
    pub encoder_count: i32,     // Absolute encoder count
    pub velocity: i32,          // Velocity in QPPS
    pub duty_cycle: i16,        // PWM duty cycle (-10000 to +10000)
    pub current: f64,           // Motor current in amperes
    pub position: f64,          // Calculated position (meters or radians)
    pub linear_velocity: f64,   // Calculated velocity (m/s or rad/s)
    pub timestamp: u64,         // Timestamp (ns since epoch)
}
```

### RoboclawDiagnostics

System diagnostics and health monitoring:

```rust
pub struct RoboclawDiagnostics {
    pub battery_voltage: f64,   // Battery voltage in volts
    pub main_current: f64,      // Total current draw in amperes
    pub motor1_current: f64,    // Motor 1 current in amperes
    pub motor2_current: f64,    // Motor 2 current in amperes
    pub temperature1: f64,      // Temperature 1 in Celsius
    pub temperature2: f64,      // Temperature 2 in Celsius
    pub error_status: u16,      // Error status flags
    pub warning_flags: u8,      // Warning flags
    pub timestamp: u64,         // Timestamp (ns since epoch)
}
```

## Public API

### Construction

```rust
use horus_library::nodes::RoboclawMotorNode;

// Create with serial port and address
let mut roboclaw = RoboclawMotorNode::new("/dev/ttyUSB0", 0x80)?;

// Multiple controllers on same bus
let mut roboclaw1 = RoboclawMotorNode::new("/dev/ttyUSB0", 0x80)?; // Controller 1
let mut roboclaw2 = RoboclawMotorNode::new("/dev/ttyUSB0", 0x81)?; // Controller 2
```

### Communication Configuration

```rust
// Set baud rate (must match Roboclaw settings)
roboclaw.set_baud_rate(38400);   // Default
roboclaw.set_baud_rate(115200);  // High-speed mode
```

### Encoder Configuration

```rust
// Set encoder resolution (pulses per revolution)
roboclaw.set_encoder_resolution(1, 1024);  // Motor 1: 1024 PPR
roboclaw.set_encoder_resolution(2, 1024);  // Motor 2: 1024 PPR

// Set gear ratio (motor:output)
roboclaw.set_gear_ratio(1, 60.0);  // 60:1 gearbox on motor 1
roboclaw.set_gear_ratio(2, 60.0);  // 60:1 gearbox on motor 2

// Set wheel radius for velocity calculations
roboclaw.set_wheel_radius(1, 0.05);  // 5cm radius wheels
roboclaw.set_wheel_radius(2, 0.05);
```

### PID Tuning

```rust
// Velocity PID parameters: motor_id, P, I, D, max_QPPS
roboclaw.set_velocity_pid(1, 1.0, 0.5, 0.25, 44000);
roboclaw.set_velocity_pid(2, 1.0, 0.5, 0.25, 44000);

// Position PID parameters: motor_id, P, I, D, max_QPPS
roboclaw.set_position_pid(1, 5.0, 0.1, 1.0, 20000);
roboclaw.set_position_pid(2, 5.0, 0.1, 1.0, 20000);
```

### Limits and Safety

```rust
// Set maximum current limit (amperes)
roboclaw.set_max_current(30.0);  // 30A limit

// Set maximum velocity for each motor (QPPS)
roboclaw.set_max_velocity(1, 50000);
roboclaw.set_max_velocity(2, 50000);

// Set maximum acceleration (QPPS/second)
roboclaw.set_max_acceleration(1, 20000);
roboclaw.set_max_acceleration(2, 20000);
```

### Query Methods

```rust
// Get encoder count
if let Some(count) = roboclaw.get_encoder(1) {
    println!("Motor 1 encoder: {}", count);
}

// Get velocity in QPPS
if let Some(velocity) = roboclaw.get_velocity(1) {
    println!("Motor 1 velocity: {} QPPS", velocity);
}

// Get battery voltage
let voltage = roboclaw.get_battery_voltage();
println!("Battery: {:.1}V", voltage);

// Get motor current
if let Some(current) = roboclaw.get_motor_current(1) {
    println!("Motor 1 current: {:.2}A", current);
}

// Reset encoder to zero
roboclaw.reset_encoder(1);
roboclaw.reset_encoder(2);
```

### Preset Configurations

```rust
// Configure for differential drive robot
roboclaw.configure_differential_drive(
    0.3,     // wheel_base (meters)
    0.05,    // wheel_radius (meters)
    1024,    // encoder_ppr
    60.0     // gear_ratio
);

// Configure for high-power application
roboclaw.configure_high_power();

// Configure for precision positioning
roboclaw.configure_precision();
```

## Usage Examples

### Basic Motor Control

```rust
use horus_library::prelude::*;

fn main() -> Result<()> {
    let mut scheduler = Scheduler::new();

    // Create Roboclaw node
    let mut roboclaw = RoboclawMotorNode::new("/dev/ttyUSB0", 0x80)?;
    roboclaw.set_baud_rate(38400);
    roboclaw.set_encoder_resolution(1, 1024);
    roboclaw.set_encoder_resolution(2, 1024);

    scheduler.add(Box::new(roboclaw), 1, Some(true));

    // Motor control node
    let motor_ctrl = node! {
        name: "motor_control",
        tick: |ctx| {
            let hub = Hub::<MotorCommand>::new("roboclaw/motor1/cmd")?;

            // Drive motor 1 at 50% forward
            let cmd = MotorCommand {
                motor_id: 1,
                mode: MotorCommand::MODE_VOLTAGE,
                target: 0.5,  // 50% duty cycle
                enable: true,
                ..Default::default()
            };
            hub.send(cmd, &mut None)?;

            Ok(())
        }
    };

    scheduler.add(Box::new(motor_ctrl), 2, Some(true));
    scheduler.run()?;
    Ok(())
}
```

### Velocity Control

```rust
use horus_library::prelude::*;

fn main() -> Result<()> {
    let mut scheduler = Scheduler::new();

    // Configure Roboclaw
    let mut roboclaw = RoboclawMotorNode::new("/dev/ttyUSB0", 0x80)?;
    roboclaw.set_baud_rate(38400);
    roboclaw.set_encoder_resolution(1, 1024);
    roboclaw.set_wheel_radius(1, 0.05);
    roboclaw.set_gear_ratio(1, 60.0);
    roboclaw.set_velocity_pid(1, 1.0, 0.5, 0.25, 44000);

    scheduler.add(Box::new(roboclaw), 1, Some(true));

    // Velocity control node
    let velocity_ctrl = node! {
        name: "velocity_control",
        tick: |ctx| {
            let hub = Hub::<MotorCommand>::new("roboclaw/motor1/cmd")?;

            // Command 0.5 m/s forward
            let cmd = MotorCommand::velocity(1, 0.5);
            hub.send(cmd, &mut None)?;

            // Monitor feedback
            let feedback_hub = Hub::<RoboclawFeedback>::new("roboclaw/motor1/feedback")?;
            if let Some(feedback) = feedback_hub.recv(None) {
                ctx.log_info(&format!(
                    "Motor 1: pos={:.3}m, vel={:.2}m/s, current={:.2}A",
                    feedback.position,
                    feedback.linear_velocity,
                    feedback.current
                ));
            }

            Ok(())
        }
    };

    scheduler.add(Box::new(velocity_ctrl), 2, Some(true));
    scheduler.run()?;
    Ok(())
}
```

### Differential Drive Robot

```rust
use horus_library::prelude::*;

fn main() -> Result<()> {
    let mut scheduler = Scheduler::new();

    // Configure Roboclaw for differential drive
    let mut roboclaw = RoboclawMotorNode::new("/dev/ttyUSB0", 0x80)?;
    roboclaw.set_baud_rate(115200);
    roboclaw.configure_differential_drive(
        0.3,     // 30cm wheel base
        0.05,    // 5cm wheel radius
        1440,    // 1440 CPR encoders
        60.0     // 60:1 gearbox
    );

    scheduler.add(Box::new(roboclaw), 1, Some(true));

    // Differential drive controller
    let drive_node = node! {
        name: "diff_drive",
        tick: |ctx| {
            let motor1_hub = Hub::<MotorCommand>::new("roboclaw/motor1/cmd")?;
            let motor2_hub = Hub::<MotorCommand>::new("roboclaw/motor2/cmd")?;

            // Drive forward at 0.5 m/s
            motor1_hub.send(MotorCommand::velocity(1, 0.5), None)?;
            motor2_hub.send(MotorCommand::velocity(2, 0.5), None)?;

            // Monitor diagnostics
            let diag_hub = Hub::<RoboclawDiagnostics>::new("roboclaw.diagnostics")?;
            if let Some(diag) = diag_hub.recv(None) {
                ctx.log_info(&format!(
                    "Battery: {:.1}V, M1: {:.2}A, M2: {:.2}A, Temp: {:.1}C",
                    diag.battery_voltage,
                    diag.motor1_current,
                    diag.motor2_current,
                    diag.temperature1
                ));

                // Safety checks
                if diag.battery_voltage < 11.0 {
                    ctx.log_warning("Low battery voltage!");
                }

                if diag.motor1_current > 25.0 || diag.motor2_current > 25.0 {
                    ctx.log_warning("High motor current detected!");
                }

                if diag.temperature1 > 70.0 {
                    ctx.log_warning("High temperature warning!");
                }
            }

            Ok(())
        }
    };

    scheduler.add(Box::new(drive_node), 2, Some(true));
    scheduler.run()?;
    Ok(())
}
```

### Position Control

```rust
use horus_library::prelude::*;

fn main() -> Result<()> {
    let mut scheduler = Scheduler::new();

    // Configure for precision positioning
    let mut roboclaw = RoboclawMotorNode::new("/dev/ttyUSB0", 0x80)?;
    roboclaw.set_baud_rate(38400);
    roboclaw.configure_precision();

    scheduler.add(Box::new(roboclaw), 1, Some(true));

    // Position control node
    let position_ctrl = node! {
        name: "position_control",
        tick: |ctx| {
            let hub = Hub::<MotorCommand>::new("roboclaw/motor1/cmd")?;

            // Move to 1.0 meter position at 0.2 m/s max
            let cmd = MotorCommand::position(1, 1.0, 0.2);
            hub.send(cmd, &mut None)?;

            // Monitor position
            let feedback_hub = Hub::<RoboclawFeedback>::new("roboclaw/motor1/feedback")?;
            if let Some(feedback) = feedback_hub.recv(None) {
                let error = 1.0 - feedback.position;
                ctx.log_info(&format!(
                    "Position: {:.3}m, Error: {:.3}m, Velocity: {:.2}m/s",
                    feedback.position, error, feedback.linear_velocity
                ));

                if error.abs() < 0.01 {
                    ctx.log_info("Position reached!");
                }
            }

            Ok(())
        }
    };

    scheduler.add(Box::new(position_ctrl), 2, Some(true));
    scheduler.run()?;
    Ok(())
}
```

### Multiple Controllers

```rust
use horus_library::prelude::*;

fn main() -> Result<()> {
    let mut scheduler = Scheduler::new();

    // Controller 1 - Drive motors (address 0x80)
    let mut drive_controller = RoboclawMotorNode::new("/dev/ttyUSB0", 0x80)?;
    drive_controller.set_baud_rate(115200);
    drive_controller.configure_differential_drive(0.3, 0.05, 1440, 60.0);

    // Controller 2 - Arm motors (address 0x81)
    let mut arm_controller = RoboclawMotorNode::new("/dev/ttyUSB0", 0x81)?;
    arm_controller.set_baud_rate(115200);
    arm_controller.configure_precision();

    scheduler.add(Box::new(drive_controller), 1, Some(true));
    scheduler.add(Box::new(arm_controller), 2, Some(true));

    scheduler.run()?;
    Ok(())
}
```

### Safety Monitoring

```rust
use horus_library::prelude::*;

fn main() -> Result<()> {
    let mut scheduler = Scheduler::new();

    let mut roboclaw = RoboclawMotorNode::new("/dev/ttyUSB0", 0x80)?;
    roboclaw.set_baud_rate(38400);
    roboclaw.set_max_current(30.0);

    scheduler.add(Box::new(roboclaw), 1, Some(true));

    // Safety monitor node
    let safety_monitor = node! {
        name: "safety_monitor",
        tick: |ctx| {
            let diag_hub = Hub::<RoboclawDiagnostics>::new("roboclaw.diagnostics")?;

            if let Some(diag) = diag_hub.recv(None) {
                // Check error status
                if diag.error_status != 0 {
                    ctx.log_error(&format!(
                        "Roboclaw error status: 0x{:04X}",
                        diag.error_status
                    ));

                    // Send stop commands on error
                    let motor1_hub = Hub::<MotorCommand>::new("roboclaw/motor1/cmd")?;
                    let motor2_hub = Hub::<MotorCommand>::new("roboclaw/motor2/cmd")?;
                    motor1_hub.send(MotorCommand::stop(1), &mut None)?;
                    motor2_hub.send(MotorCommand::stop(2), &mut None)?;
                }

                // Battery check
                if diag.battery_voltage < 10.5 {
                    ctx.log_error("Critical battery voltage! Stopping motors.");
                    // Emergency stop
                }

                // Overcurrent check
                if diag.motor1_current > 30.0 {
                    ctx.log_warning("Motor 1 overcurrent!");
                }
                if diag.motor2_current > 30.0 {
                    ctx.log_warning("Motor 2 overcurrent!");
                }

                // Temperature check
                if diag.temperature1 > 75.0 || diag.temperature2 > 75.0 {
                    ctx.log_warning("High temperature detected!");
                }
            }

            Ok(())
        }
    };

    scheduler.add(Box::new(safety_monitor), 3, Some(true));
    scheduler.run()?;
    Ok(())
}
```

## Hardware Setup

### Wiring Diagram

```
Raspberry Pi/Computer      Roboclaw           Motors/Encoders
=====================      ========           ===============
USB Port           <-->   USB (or)

GPIO TX (3.3V)     --->   S1 (RX)
GPIO RX (3.3V)     <---   S2 (TX)
GND                ---    GND

                          M1A/M1B    <--->   Motor 1
                          M2A/M2B    <--->   Motor 2

                          ENC1A      <---    Encoder 1 A
                          ENC1B      <---    Encoder 1 B
                          ENC2A      <---    Encoder 2 A
                          ENC2B      <---    Encoder 2 B

Battery (+)        --->   B+ (or +)
Battery (-)        --->   B- (or -)  ---    GND
```

### Roboclaw Models and Specifications

| Model | Voltage Range | Current (Continuous) | Current (Peak) | Max Power |
|-------|---------------|---------------------|----------------|-----------|
| 2x7A | 7.2V - 34V | 7A per channel | 15A | 238W |
| 2x15A | 7.2V - 34V | 15A per channel | 30A | 510W |
| 2x30A | 7.2V - 34V | 30A per channel | 60A | 1020W |
| 2x45A | 7.2V - 34V | 45A per channel | 90A | 1530W |
| 2x60A | 7.2V - 34V | 60A per channel | 120A | 2040W |
| 2x160A | 12V - 48V | 160A per channel | 320A | 7680W |

### Serial Connection Options

**Option 1: USB Connection**
- Use USB cable to connect Roboclaw USB port to computer
- Device appears as `/dev/ttyACM0` or `/dev/ttyUSB0`
- Easiest option, provides power and data

**Option 2: TTL Serial (3.3V or 5V)**
- Connect GPIO TX to Roboclaw S1 (RX)
- Connect GPIO RX to Roboclaw S2 (TX)
- Connect GND to Roboclaw GND
- Use 3.3V logic level (Raspberry Pi compatible)

**Option 3: RS-232 Serial**
- Requires RS-232 level shifter
- Connect to S1/S2 pins
- Used for longer cable runs

### Encoder Wiring

```
Encoder Type          Roboclaw Pins       Encoder Pins
============          =============       ============
Quadrature (A/B)      ENC1A, ENC1B       Channel A, B
                      ENC2A, ENC2B       Channel A, B
                      GND                GND
                      +5V (optional)     VCC (if needed)

Absolute Encoder      Use external       N/A
                      microcontroller
```

### System Requirements

```bash
# Install serial port tools
sudo apt install setserial cu

# Add user to dialout group for serial access
sudo usermod -a -G dialout $USER

# Logout and login for group to take effect

# Verify serial ports
ls -l /dev/ttyUSB* /dev/ttyACM*

# Test serial connection
sudo cu -l /dev/ttyUSB0 -s 38400
```

### Enable in Project

```toml
[dependencies]
horus_library = { version = "0.1", features = ["serial-hardware"] }
```

```bash
cargo build --features="serial-hardware"
```

## Roboclaw Configuration

### Baud Rate Settings

The Roboclaw must be configured to match your baud rate:

- **DIP Switches** (older models): Set physical switches
- **Motion Studio** (recommended): Configure via software
- **Default**: 38400 baud

Common baud rates:
- 2400, 9600, 19200, 38400 (default), 57600, 115200, 230400, 460800

### Address Settings

Set Roboclaw address for multi-controller setups:

- **Address Range**: 0x80 to 0x87 (8 controllers max on one bus)
- **Default**: 0x80
- **Configure**: Use Motion Studio or DIP switches

Example configuration:
```rust
let mut roboclaw1 = RoboclawMotorNode::new("/dev/ttyUSB0", 0x80)?;
let mut roboclaw2 = RoboclawMotorNode::new("/dev/ttyUSB0", 0x81)?;
let mut roboclaw3 = RoboclawMotorNode::new("/dev/ttyUSB0", 0x82)?;
```

### Encoder Settings

Configure encoder mode and CPR (Counts Per Revolution):

1. **Quadrature mode** (most common): 4x counting
2. **Absolute encoders**: Not directly supported
3. **CPR**: Must match your encoder specification

Common encoder CPR values:
- 360, 512, 1024, 1440, 2048, 4096

```rust
roboclaw.set_encoder_resolution(1, 1024);  // 1024 PPR encoder
```

## PID Tuning Guide

### Understanding QPPS

QPPS (Quadrature Pulses Per Second) is the fundamental unit for Roboclaw velocity:

```
QPPS = Encoder_CPR × 4 × RPS

Where:
- Encoder_CPR = Counts per revolution (e.g., 1024)
- 4 = Quadrature multiplier
- RPS = Revolutions per second
```

Example:
```
Encoder: 1024 CPR
Motor speed: 60 RPM = 1 RPS
QPPS = 1024 × 4 × 1 = 4096 QPPS
```

### Velocity PID Tuning

**Step 1: Set all gains to zero**
```rust
roboclaw.set_velocity_pid(1, 0.0, 0.0, 0.0, 44000);
```

**Step 2: Increase P gain until oscillation**
```rust
roboclaw.set_velocity_pid(1, 1.0, 0.0, 0.0, 44000);
// Gradually increase P: 1.0, 2.0, 3.0, ...
```

**Step 3: Add I gain to eliminate steady-state error**
```rust
roboclaw.set_velocity_pid(1, 2.0, 0.5, 0.0, 44000);
```

**Step 4: Add D gain to reduce overshoot (if needed)**
```rust
roboclaw.set_velocity_pid(1, 2.0, 0.5, 0.25, 44000);
```

**Step 5: Fine-tune for performance**
```rust
// Typical values:
// P: 1.0 - 5.0
// I: 0.1 - 2.0
// D: 0.0 - 1.0
// QPPS: Motor-dependent (usually 20000 - 100000)
```

### Position PID Tuning

Position PID is typically easier to tune:

**Step 1: Start with P only**
```rust
roboclaw.set_position_pid(1, 5.0, 0.0, 0.0, 20000);
```

**Step 2: Add D to reduce overshoot**
```rust
roboclaw.set_position_pid(1, 5.0, 0.0, 1.0, 20000);
```

**Step 3: Add I only if position holding needs improvement**
```rust
roboclaw.set_position_pid(1, 5.0, 0.1, 1.0, 20000);
```

### Using Motion Studio

BasicMicro's Motion Studio provides:
- Auto-tuning wizard
- Real-time graphing
- PID parameter adjustment
- Velocity profiling
- Firmware updates

Download from: `www.basicmicro.com`

## Best Practices

1. **Always use encoder feedback for closed-loop control**:
   ```rust
   roboclaw.set_encoder_resolution(1, 1024);
   // Velocity and position modes require encoders
   ```

2. **Set appropriate current limits**:
   ```rust
   roboclaw.set_max_current(30.0);  // 30A max (adjust for your motors)
   // Protects both motors and controller
   ```

3. **Monitor battery voltage continuously**:
   ```rust
   if diag.battery_voltage < 11.0 {
       // Stop or slow down
       motor_hub.send(MotorCommand::stop(1), &mut None)?;
   }
   ```

4. **Use appropriate PWM frequency**:
   - Default: 20kHz (good for most motors)
   - Lower frequencies: More torque at low speeds
   - Higher frequencies: Quieter operation

5. **Implement timeout safety**:
   ```rust
   // If no commands received for 1 second, stop motors
   if last_command_age > 1.0 {
       motor_hub.send(MotorCommand::stop(1), &mut None)?;
   }
   ```

6. **Check error status regularly**:
   ```rust
   if diag.error_status != 0 {
       ctx.log_error(&format!("Error: 0x{:04X}", diag.error_status));
       // Take corrective action
   }
   ```

7. **Use simulation mode for development**:
   - Test logic without hardware
   - Simulation provides realistic feedback
   - Automatic fallback if hardware unavailable

8. **Proper grounding is critical**:
   - Connect all GND pins together
   - Use star grounding topology
   - Minimize ground loops

9. **Heat management**:
   - Monitor temperatures
   - Add heatsinks for high-current applications
   - Ensure adequate ventilation

10. **Calibrate your system**:
    ```rust
    // Measure actual wheel circumference
    // Calculate gear ratio precisely
    // Verify encoder CPR specification
    ```

## Timing and Performance

### Update Rates

The Roboclaw node operates at:
- **Feedback publishing**: 50Hz (20ms period)
- **Diagnostics publishing**: 10Hz (100ms period)
- **Command processing**: Every tick cycle

### Encoder Resolution

Encoder counts for full 360° rotation:
```
Total Counts = Encoder_CPR × 4 × Gear_Ratio

Example:
- Encoder: 1024 CPR
- Gear ratio: 60:1
- Total counts: 1024 × 4 × 60 = 245,760 counts per wheel revolution
```

### Velocity Calculation

Linear velocity from QPPS:
```
velocity (m/s) = (QPPS / (Encoder_CPR × 4)) × (1 / Gear_Ratio) × (2π × Wheel_Radius)

Example:
- QPPS: 4096
- Encoder: 1024 CPR
- Gear ratio: 60:1
- Wheel radius: 0.05m
- Velocity: (4096 / 4096) × (1/60) × (2π × 0.05) = 0.00523 m/s
```

## Troubleshooting

### "Hardware unavailable - using SIMULATION mode"

```
[WARN] RoboclawMotorNode: Hardware unavailable - using SIMULATION mode
[WARN]   Tried: /dev/ttyUSB0
[WARN]   Error: Permission denied
```

**Solutions:**
1. Check serial port exists: `ls -l /dev/ttyUSB*`
2. Add user to dialout group: `sudo usermod -a -G dialout $USER`
3. Logout and login for group membership to take effect
4. Verify Roboclaw power (LED should be on)
5. Check USB cable connection
6. Try different USB port
7. Verify baud rate matches Roboclaw settings
8. Check device address (default 0x80)
9. Rebuild with: `cargo build --features="serial-hardware"`

### Motor runs but wrong direction

**Solutions:**
1. Swap motor wires: M1A <-> M1B or M2A <-> M2B
2. Invert encoder if feedback shows negative velocity
3. Reverse in software by negating command values

### Encoder counts not changing

**Solutions:**
1. Check encoder wiring (A, B, GND, VCC)
2. Verify encoder power supply (usually 5V)
3. Test encoder with Motion Studio
4. Check CPR setting matches encoder specification
5. Ensure encoder shaft coupling is secure (no slipping)
6. Verify encoder channels are not swapped

### Motor runs but velocity control unstable

**Solutions:**
1. Re-tune PID gains (start with lower values)
2. Reduce P gain if oscillating
3. Increase D gain to dampen oscillations
4. Check for mechanical binding or friction
5. Verify encoder is reading correctly
6. Check gear ratio and wheel radius settings
7. Ensure adequate power supply (voltage sag causes issues)

### Intermittent serial communication errors

**Solutions:**
1. Check cable length (keep under 10 feet for TTL)
2. Use shielded cable for longer runs
3. Add decoupling capacitors near Roboclaw
4. Check for electrical noise sources
5. Reduce baud rate to 38400 or lower
6. Verify ground connections are solid

### Overcurrent errors

**Solutions:**
1. Reduce current limit if set too high
2. Check for stalled motor or mechanical binding
3. Verify motor specifications match controller rating
4. Check for short circuits in motor wiring
5. Ensure adequate power supply current capacity
6. Add current monitoring to detect issues early

### Position mode not reaching target

**Solutions:**
1. Tune position PID gains
2. Increase max velocity parameter
3. Check for mechanical resistance
4. Verify encoder feedback is accurate
5. Check target position is within reasonable range

### Battery voltage reading incorrect

**Solutions:**
1. Verify battery is properly connected
2. Check B+ and B- connections
3. Use Motion Studio to verify voltage reading
4. Calibrate voltage reading if necessary
5. Check for voltage drop under load

### High temperature warnings

**Solutions:**
1. Reduce motor current limit
2. Add heatsink to Roboclaw controller
3. Improve airflow around controller
4. Reduce duty cycle or speed
5. Check motor efficiency (inefficient motors generate heat)
6. Verify motor isn't stalling or binding

## Simulation Mode

When hardware is unavailable, the node operates in simulation mode:

```
[INFO] RoboclawMotorNode: Hardware unavailable - using SIMULATION mode
[DEBUG] Roboclaw 0x80 M1 (SIM): Duty = 5000 (50.0%)
```

Simulated behavior:
- Accepts all commands
- Updates encoder counts based on commanded velocity
- Simulates current draw proportional to velocity
- Provides realistic feedback for testing
- Battery voltage starts at 24V and drops under load
- Temperature increases with current draw
- Useful for algorithm development without hardware

To force simulation mode:
```rust
// Don't build with serial-hardware feature
cargo build  // Without --features="serial-hardware"
```

## Error Status Codes

The `error_status` field in `RoboclawDiagnostics` contains error flags:

| Bit | Value | Description |
|-----|-------|-------------|
| 0 | 0x0001 | Emergency stop active |
| 1 | 0x0002 | Temperature error |
| 2 | 0x0004 | Main battery voltage high |
| 3 | 0x0008 | Logic battery voltage high |
| 4 | 0x0010 | Logic battery voltage low |
| 5 | 0x0020 | M1 driver fault |
| 6 | 0x0040 | M2 driver fault |
| 7 | 0x0080 | M1 over current |
| 8 | 0x0100 | M2 over current |
| 9 | 0x0200 | M1 over current warning |
| 10 | 0x0400 | M2 over current warning |
| 11 | 0x0800 | Main battery voltage high warning |
| 12 | 0x1000 | Main battery voltage low warning |
| 13 | 0x2000 | Temperature warning |
| 14 | 0x4000 | S4 signal triggered |
| 15 | 0x8000 | S5 signal triggered |

Check for errors:
```rust
if diag.error_status & 0x0001 != 0 {
    ctx.log_error("Emergency stop active!");
}
if diag.error_status & 0x0020 != 0 {
    ctx.log_error("Motor 1 driver fault!");
}
if diag.error_status & 0x0080 != 0 {
    ctx.log_error("Motor 1 overcurrent!");
}
```

## Advanced Features

### Mixed Mode Control

Control both motors simultaneously with forward/turn commands:

```rust
// Not directly implemented, but can achieve with differential drive:
let linear_vel = 0.5;   // m/s forward
let angular_vel = 0.3;  // rad/s turning

let wheel_base = 0.3;   // meters between wheels
let left_vel = linear_vel - (angular_vel * wheel_base / 2.0);
let right_vel = linear_vel + (angular_vel * wheel_base / 2.0);

motor1_hub.send(MotorCommand::velocity(1, left_vel), None)?;
motor2_hub.send(MotorCommand::velocity(2, right_vel), None)?;
```

### Acceleration Limiting

Set maximum acceleration to smooth motion:

```rust
let mut cmd = MotorCommand::velocity(1, 1.0);
cmd.max_acceleration = 0.5;  // 0.5 m/s² max acceleration
motor_hub.send(cmd, &mut None)?;
```

### Feed-Forward Control

Improve tracking with feed-forward term:

```rust
let mut cmd = MotorCommand::velocity(1, 0.5);
cmd.feed_forward = 0.1;  // Add 10% feed-forward
motor_hub.send(cmd, &mut None)?;
```

## See Also

- [DcMotorNode](../dc_motor/) - Basic DC motor control without encoders
- [ServoNode](../servo/) - Position-controlled servo motors
- [EncoderNode](../encoder/) - Standalone encoder reading
- [PidControllerNode](../pid_controller/) - Custom PID control loops
- [DifferentialDriveNode](../differential_drive/) - High-level robot navigation
- [OdometryNode](../odometry/) - Position tracking from wheel encoders
