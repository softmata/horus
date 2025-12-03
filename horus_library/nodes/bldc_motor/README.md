# BLDC Motor Controller Node

Brushless DC motor and Electronic Speed Controller (ESC) interface supporting 10+ protocols for drones, robots, and high-performance applications.

## Overview

The BLDC Motor Node provides comprehensive control of brushless DC motors through Electronic Speed Controllers (ESCs). It supports multiple communication protocols including PWM, OneShot125/42, MultiShot, DShot150/300/600/1200, ProShot, and CAN bus. Designed for applications requiring precise, high-performance motor control with up to 8 motors simultaneously.

Supports hobby ESCs, industrial motor controllers (ODrive, VESC, SimpleFOC), and digital protocols with bidirectional telemetry.

Key features:
- Multi-motor support (up to 8 motors)
- 10+ ESC protocols (PWM, OneShot, DShot, CAN)
- Velocity and position control modes
- Motor arming/disarming safety
- Telemetry feedback (RPM, voltage, current, temperature)
- Direction reversal support
- Acceleration ramping/limiting
- Failsafe timeout handling
- Hardware PWM via GPIO or simulation fallback

## Topics

### Subscribers

| Topic | Type | Description |
|-------|------|-------------|
| `bldc_cmd` | `MotorCommand` | Motor velocity/position/torque commands |

### Publishers

| Topic | Type | Description |
|-------|------|-------------|
| `bldc_cmd_feedback` | `MotorCommand` | Echo of received commands for monitoring |
| `bldc_cmd_telemetry` | `BldcTelemetry` | Motor telemetry data (voltage, current, RPM, temperature) |

## Configuration Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `num_motors` | `u8` | `1` | Number of BLDC motors (1-8) |
| `protocol` | `BldcProtocol` | `DShot600` | ESC communication protocol |
| `pwm_frequency_hz` | `f64` | `50.0` | PWM frequency in Hz (50 Hz for standard ESC, higher for digital protocols) |
| `command_timeout_ms` | `u64` | `1000` | Command timeout in milliseconds (0 to disable) |
| `enable_telemetry` | `bool` | `true` | Enable telemetry publishing |
| `armed` | `bool` | `false` | Global arm state (safety) |

### Per-Motor Configuration

Configure individually for each motor (motor_id 0-7):

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `pwm_min` | `u16` | `1000` | Minimum PWM value in microseconds |
| `pwm_max` | `u16` | `2000` | Maximum PWM value in microseconds |
| `pwm_neutral` | `u16` | `1500` | Neutral/zero throttle in microseconds |
| `min_velocity` | `f64` | `0.0` | Minimum RPM |
| `max_velocity` | `f64` | `10000.0` | Maximum RPM |
| `acceleration_limit` | `f64` | `5000.0` | Maximum acceleration in RPM per second |
| `invert_direction` | `bool` | `false` | Reverse motor direction |
| `pwm_gpio_pin` | `u8` | `0` | GPIO pin for PWM output (12, 13, 18, or 19 for Raspberry Pi) |

## ESC Protocols

### BldcProtocol Enum

```rust
pub enum BldcProtocol {
    StandardPwm,      // 1000-2000μs, 50Hz update rate
    OneShot125,       // 125-250μs, up to 4kHz
    OneShot42,        // 42-84μs, up to 12kHz
    MultiShot,        // 5-25μs, up to 32kHz
    DShot150,         // 150kbit/s digital
    DShot300,         // 300kbit/s digital
    DShot600,         // 600kbit/s digital (recommended)
    DShot1200,        // 1200kbit/s digital
    ProShot,          // High-speed bidirectional
    Can,              // CAN bus control
}
```

**Protocol Comparison:**

| Protocol | Update Rate | Latency | Telemetry | Digital | Calibration | Use Case |
|----------|-------------|---------|-----------|---------|-------------|----------|
| StandardPwm | 50 Hz | 20ms | No | No | Required | Basic/legacy |
| OneShot125 | 4 kHz | 0.25ms | No | No | Required | Fast RC |
| OneShot42 | 12 kHz | 0.08ms | No | No | Required | Racing |
| MultiShot | 32 kHz | 0.03ms | No | No | Required | High-performance |
| DShot600 | Variable | ~0.1ms | Yes | Yes | Not required | Recommended |
| DShot1200 | Variable | ~0.05ms | Yes | Yes | Not required | Max performance |
| ProShot | Variable | ~0.03ms | Yes | Yes | Not required | Pro racing |
| Can | 1 kHz | 1ms | Yes | Yes | Not required | Industrial |

## Message Types

### MotorCommand

Motor control command message:

```rust
pub struct MotorCommand {
    pub motor_id: u8,           // Motor ID (0-7)
    pub mode: u8,               // Control mode
    pub target: f64,            // Target value (RPM, position, etc.)
    pub max_velocity: f64,      // Maximum velocity for position mode
    pub max_acceleration: f64,  // Maximum acceleration
    pub feed_forward: f64,      // Feed-forward term
    pub enable: bool,           // Enable motor
    pub timestamp: u64,         // Timestamp in nanoseconds
}
```

**Control Modes:**
- `MotorCommand::MODE_VELOCITY = 0` - Velocity control (RPM)
- `MotorCommand::MODE_POSITION = 1` - Position control (not supported for BLDC)
- `MotorCommand::MODE_TORQUE = 2` - Torque control (requires current sensing ESC)
- `MotorCommand::MODE_VOLTAGE = 3` - Direct voltage control

**Helper Methods:**
```rust
// Create velocity command (most common for BLDC)
MotorCommand::velocity(motor_id, rpm)

// Create position command
MotorCommand::position(motor_id, position, max_velocity)

// Create stop command
MotorCommand::stop(motor_id)

// Validate command
command.is_valid()
```

### BldcTelemetry

Telemetry feedback message (from ESC, simulation, or measured):

```rust
pub struct BldcTelemetry {
    pub motor_id: u8,       // Motor ID
    pub voltage: f32,       // Motor/battery voltage (V)
    pub current: f32,       // Motor current draw (A)
    pub temperature: f32,   // ESC/motor temperature (C)
    pub rpm: f64,           // Measured RPM
    pub throttle: f64,      // Current throttle (0.0-1.0)
    pub error_count: u32,   // Communication error count
    pub timestamp: u64,     // Timestamp in nanoseconds
}
```

## Public API

### Construction

```rust
use horus_library::nodes::BldcMotorNode;

// Create with default topic "bldc_cmd"
let mut bldc = BldcMotorNode::new()?;

// Create with custom topic
let mut bldc = BldcMotorNode::new_with_topic("motors.quadcopter")?;
```

### Configuration Methods

```rust
// Set number of motors (1-8)
bldc.set_num_motors(4);

// Set ESC protocol
bldc.set_protocol(BldcProtocol::DShot600);

// Configure PWM range for a motor (in microseconds)
bldc.set_pwm_range(0, 1000, 2000, 1500);  // motor_id, min, max, neutral

// Configure GPIO pin for PWM output
bldc.set_pwm_pin(0, 18);  // motor_id, gpio_pin (12, 13, 18, or 19 for RPi)

// Set PWM frequency in Hz
bldc.set_pwm_frequency(50.0);  // 50 Hz for standard ESC

// Set velocity limits for a motor (in RPM)
bldc.set_velocity_limits(0, 0.0, 10000.0);  // motor_id, min_rpm, max_rpm

// Set acceleration limit (RPM per second)
bldc.set_acceleration_limit(0, 5000.0);  // motor_id, rpm_per_sec

// Invert motor direction
bldc.set_direction_inverted(0, true);  // motor_id, inverted

// Arm all motors (safety requirement)
bldc.arm_motors();

// Disarm all motors
bldc.disarm_motors();

// Arm a specific motor
bldc.arm_motor(0);  // motor_id

// Emergency stop all motors
bldc.emergency_stop();
```

### Preset Configurations

```rust
// Configure for racing quadcopter (high-performance)
bldc.configure_racing_quad();
// Sets: DShot600, 0-40000 RPM, 50000 RPM/s acceleration

// Configure for camera gimbal (smooth, precise)
bldc.configure_gimbal();
// Sets: DShot300, 0-5000 RPM, 1000 RPM/s acceleration

// Configure for industrial application
bldc.configure_industrial();
// Sets: CAN protocol, 0-15000 RPM, 5000 RPM/s acceleration
```

### Query Methods

```rust
// Check if motors are armed
if bldc.is_armed() {
    println!("Motors armed and ready");
}

// Get current velocity for a motor (RPM)
if let Some(velocity) = bldc.get_velocity(0) {
    println!("Motor 0 velocity: {:.0} RPM", velocity);
}

// Get current throttle for a motor (0.0-1.0)
if let Some(throttle) = bldc.get_throttle(0) {
    println!("Motor 0 throttle: {:.1}%", throttle * 100.0);
}

// Get telemetry for a motor
if let Some(telem) = bldc.get_telemetry(0) {
    println!("Motor 0: {:.1}V, {:.1}A, {:.0}C, {:.0} RPM",
        telem.voltage, telem.current, telem.temperature, telem.rpm);
}
```

## Usage Examples

### Single Motor Control

```rust
use horus_library::prelude::*;

fn main() -> Result<()> {
    let mut scheduler = Scheduler::new();

    // Create BLDC node
    let mut bldc = BldcMotorNode::new()?;
    bldc.set_num_motors(1);
    bldc.set_protocol(BldcProtocol::DShot600);
    bldc.set_pwm_pin(0, 18);  // GPIO 18
    bldc.set_velocity_limits(0, 0.0, 5000.0);  // 0-5000 RPM
    bldc.arm_motors();

    scheduler.add(Box::new(bldc), 1, Some(true));

    // Motor control node
    let controller = node! {
        name: "motor_controller",
        tick: |ctx| {
            let hub = Hub::<MotorCommand>::new("bldc_cmd")?;

            // Send velocity command: 2000 RPM
            let cmd = MotorCommand::velocity(0, 2000.0);
            hub.send(cmd, &mut None)?;

            Ok(())
        }
    };

    scheduler.add(Box::new(controller), 2, Some(true));
    scheduler.run()?;
    Ok(())
}
```

### Quadcopter (4 Motors)

```rust
use horus_library::prelude::*;

fn main() -> Result<()> {
    let mut scheduler = Scheduler::new();

    // Configure 4 motors for quadcopter
    let mut bldc = BldcMotorNode::new()?;
    bldc.set_num_motors(4);
    bldc.set_protocol(BldcProtocol::DShot600);

    // Motor layout (X configuration):
    //   0   1
    //    \ /
    //     X
    //    / \
    //   2   3

    // Configure each motor
    for motor_id in 0..4 {
        bldc.set_pwm_pin(motor_id, 18 + motor_id);  // GPIO 18-21
        bldc.set_velocity_limits(motor_id, 0.0, 10000.0);
        bldc.set_acceleration_limit(motor_id, 10000.0);  // Fast response
    }

    // Reverse motors 1 and 3 for proper rotation
    bldc.set_direction_inverted(1, true);
    bldc.set_direction_inverted(3, true);

    bldc.arm_motors();

    scheduler.add(Box::new(bldc), 1, Some(true));

    // Flight controller
    let flight_ctrl = node! {
        name: "flight_controller",
        tick: |ctx| {
            let hub = Hub::<MotorCommand>::new("bldc_cmd")?;

            // Example: Hover throttle at 50%
            let hover_rpm = 5000.0;

            for motor_id in 0..4 {
                let cmd = MotorCommand::velocity(motor_id, hover_rpm);
                hub.send(cmd, &mut None)?;
            }

            Ok(())
        }
    };

    scheduler.add(Box::new(flight_ctrl), 2, Some(true));
    scheduler.run()?;
    Ok(())
}
```

### Telemetry Monitoring

```rust
use horus_library::prelude::*;

fn main() -> Result<()> {
    let mut scheduler = Scheduler::new();

    // Create BLDC node with telemetry enabled
    let mut bldc = BldcMotorNode::new()?;
    bldc.set_num_motors(4);
    bldc.set_protocol(BldcProtocol::DShot600);

    for motor_id in 0..4 {
        bldc.set_pwm_pin(motor_id, 18 + motor_id);
        bldc.set_velocity_limits(motor_id, 0.0, 10000.0);
    }

    bldc.arm_motors();
    scheduler.add(Box::new(bldc), 1, Some(true));

    // Telemetry monitor
    let monitor = node! {
        name: "telemetry_monitor",
        tick: |ctx| {
            let telem_hub = Hub::<BldcTelemetry>::new("bldc_cmd_telemetry")?;

            while let Some(telem) = telem_hub.recv(None) {
                // Check for overheating
                if telem.temperature > 80.0 {
                    ctx.log_warning(&format!(
                        "Motor {} overheating: {:.1}C - reduce throttle!",
                        telem.motor_id, telem.temperature
                    ));
                }

                // Check for low voltage
                if telem.voltage < 10.8 {  // 3S LiPo low voltage
                    ctx.log_warning(&format!(
                        "Low battery: {:.2}V - land immediately!",
                        telem.voltage
                    ));
                }

                // Check for excessive current
                if telem.current > 25.0 {
                    ctx.log_warning(&format!(
                        "Motor {} high current: {:.1}A",
                        telem.motor_id, telem.current
                    ));
                }

                // Log normal operation
                ctx.log_debug(&format!(
                    "Motor {}: {:.0} RPM, {:.1}V, {:.1}A, {:.1}C",
                    telem.motor_id, telem.rpm, telem.voltage,
                    telem.current, telem.temperature
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

### Acceleration Limiting

```rust
use horus_library::prelude::*;

fn main() -> Result<()> {
    let mut scheduler = Scheduler::new();

    let mut bldc = BldcMotorNode::new()?;
    bldc.set_num_motors(1);
    bldc.set_protocol(BldcProtocol::DShot600);
    bldc.set_pwm_pin(0, 18);
    bldc.set_velocity_limits(0, 0.0, 5000.0);

    // Set smooth acceleration: 1000 RPM per second
    bldc.set_acceleration_limit(0, 1000.0);

    bldc.arm_motors();
    scheduler.add(Box::new(bldc), 1, Some(true));

    // Smooth startup controller
    let controller = node! {
        name: "smooth_controller",
        init: |ctx| {
            let hub = Hub::<MotorCommand>::new("bldc_cmd")?;

            // Command 3000 RPM - will ramp up smoothly
            let cmd = MotorCommand::velocity(0, 3000.0);
            hub.send(cmd, &mut None)?;

            ctx.log_info("Ramping motor to 3000 RPM (1000 RPM/s)");
            Ok(())
        },
        tick: |_ctx| {
            // Acceleration limiting handled automatically by BldcMotorNode
            Ok(())
        }
    };

    scheduler.add(Box::new(controller), 2, Some(true));
    scheduler.run()?;
    Ok(())
}
```

### Direction Reversal

```rust
use horus_library::prelude::*;

fn main() -> Result<()> {
    let mut scheduler = Scheduler::new();

    let mut bldc = BldcMotorNode::new()?;
    bldc.set_num_motors(2);
    bldc.set_protocol(BldcProtocol::DShot600);

    // Motor 0: Normal direction
    bldc.set_pwm_pin(0, 18);
    bldc.set_velocity_limits(0, 0.0, 5000.0);
    bldc.set_direction_inverted(0, false);

    // Motor 1: Reversed direction
    bldc.set_pwm_pin(1, 19);
    bldc.set_velocity_limits(1, 0.0, 5000.0);
    bldc.set_direction_inverted(1, true);  // Reversed

    bldc.arm_motors();
    scheduler.add(Box::new(bldc), 1, Some(true));

    // Controller
    let controller = node! {
        name: "direction_test",
        tick: |ctx| {
            let hub = Hub::<MotorCommand>::new("bldc_cmd")?;

            // Both motors receive same positive RPM command
            let cmd0 = MotorCommand::velocity(0, 2000.0);
            let cmd1 = MotorCommand::velocity(1, 2000.0);

            hub.send(cmd0, &mut None)?;
            hub.send(cmd1, &mut None)?;

            // Motor 0 spins forward, Motor 1 spins backward
            // (due to direction inversion)

            Ok(())
        }
    };

    scheduler.add(Box::new(controller), 2, Some(true));
    scheduler.run()?;
    Ok(())
}
```

## Hardware Setup

### Wiring Diagram (Single Motor)

```
Raspberry Pi          ESC               Motor
GPIO 18 (PWM) ----->  Signal     ----->  Phase A
                      GND        <-----   Phase B
                      VCC (5V)           Phase C
5V (optional) ----->  BEC (if needed)
GND           ------> GND (REQUIRED)

Power Supply:
LiPo Battery  ----->  ESC Power Input
                      (DO NOT connect to Pi!)
```

### Multi-Motor Setup (Quadcopter)

```
Raspberry Pi         ESCs              Motors
GPIO 18 (PWM) ----->  ESC 0 Signal ---> Motor 0
GPIO 19 (PWM) ----->  ESC 1 Signal ---> Motor 1
GPIO 20 (PWM) ----->  ESC 2 Signal ---> Motor 2
GPIO 21 (PWM) ----->  ESC 3 Signal ---> Motor 3
GND           ------> Common Ground (ALL ESCs)

Power Distribution:
LiPo Battery ----> Power Distribution Board
                   |
                   +---> ESC 0
                   +---> ESC 1
                   +---> ESC 2
                   +---> ESC 3
```

### Important Wiring Notes

1. **Common Ground is CRITICAL**: Always connect GND between Raspberry Pi and all ESCs
2. **Separate Power**: NEVER power motors from Pi's 5V rail - use dedicated LiPo battery
3. **GPIO Pins**: Raspberry Pi hardware PWM pins are GPIO 12, 13, 18, and 19
4. **Signal Voltage**: Most ESCs accept 3.3V signal (Pi output) - no level shifting needed
5. **BEC Power**: Some ESCs provide 5V BEC output - can power Pi but check current rating

### System Requirements

```bash
# Install GPIO library for hardware PWM
sudo apt install libraspberrypi-dev

# Enable GPIO and PWM interfaces
sudo raspi-config
# Select: Interface Options -> GPIO -> Enable

# Add user to gpio group (for permissions)
sudo usermod -a -G gpio $USER
```

### Enable in Project

```toml
[dependencies]
horus_library = { version = "0.1", features = ["gpio-hardware"] }
```

```bash
cargo build --features="gpio-hardware"
```

## Protocol Details

### Standard PWM (50 Hz)

```rust
bldc.set_protocol(BldcProtocol::StandardPwm);
bldc.set_pwm_frequency(50.0);

// Pulse width range: 1000-2000 μs
// 1000 μs = 0% throttle (stopped)
// 1500 μs = 50% throttle
// 2000 μs = 100% throttle (full speed)

// Pros: Universal compatibility
// Cons: Slow update rate (~50 Hz), requires calibration
```

### OneShot125

```rust
bldc.set_protocol(BldcProtocol::OneShot125);

// Pulse width range: 125-250 μs
// Update rate: Up to 4 kHz
// 8x faster response than standard PWM

// Pros: Fast response, widely supported
// Cons: Still analog, requires calibration
```

### DShot600 (Recommended)

```rust
bldc.set_protocol(BldcProtocol::DShot600);

// Bit rate: 600 kbit/s
// Digital protocol with CRC error checking
// 11-bit throttle value: 0-2047
// No calibration required
// Supports bidirectional telemetry

// Pros: Digital (no calibration), fast, telemetry, CRC
// Cons: Requires compatible ESC (most modern ESCs support it)
```

### CAN Bus

```rust
bldc.set_protocol(BldcProtocol::Can);

// For industrial motor controllers
// Supports: ODrive, VESC, SimpleFOC with CAN
// Bidirectional communication
// Telemetry and configuration over CAN

// Pros: Reliable, long distances, full telemetry
// Cons: Requires CAN bus hardware
```

## Best Practices

1. **Use DShot600 for modern ESCs**:
   ```rust
   bldc.set_protocol(BldcProtocol::DShot600);
   // No calibration needed, CRC protection, telemetry support
   ```

2. **Always implement arming sequence**:
   ```rust
   // Never arm motors automatically on startup
   // Require explicit arming command or button press
   bldc.arm_motors();
   ```

3. **Set appropriate velocity limits**:
   ```rust
   // Know your motor's KV rating and battery voltage
   // Max RPM = KV × Voltage
   // Example: 2300 KV motor on 4S (16.8V) = 38,640 RPM max
   bldc.set_velocity_limits(0, 0.0, 38000.0);
   ```

4. **Use acceleration limiting for smooth operation**:
   ```rust
   // Prevents jerky movements and reduces mechanical stress
   bldc.set_acceleration_limit(0, 5000.0);  // 5000 RPM/s
   ```

5. **Monitor telemetry for safety**:
   ```rust
   // Check temperature, voltage, and current
   if telem.temperature > 80.0 {
       // Reduce throttle or emergency stop
   }
   if telem.voltage < 10.5 {  // 3S LiPo low voltage
       // Land immediately
   }
   ```

6. **Common ground is non-negotiable**:
   ```
   ALWAYS connect GND between Pi and ESC
   Without common ground, PWM signals will be unreliable
   ```

7. **Test individual motors first**:
   ```rust
   // Test one motor at a time
   // Verify correct rotation direction
   // Check for unusual vibrations or sounds
   // Ensure propellers are REMOVED during testing
   ```

8. **Handle command timeouts**:
   ```rust
   // ESCs should stop if no command received
   bldc.command_timeout_ms = 500;  // 500ms timeout
   ```

## ESC Calibration (PWM/OneShot only)

DShot does NOT require calibration. For PWM and OneShot protocols:

### Manual Calibration Procedure

1. Disconnect motor power (unplug battery)
2. Send maximum throttle (2000 μs):
   ```rust
   let cmd = MotorCommand::velocity(0, 10000.0);  // Max velocity
   hub.send(cmd, &mut None)?;
   ```
3. Connect motor power (battery) - ESC will beep
4. Wait for confirmation beeps (2-3 seconds)
5. Send minimum throttle (1000 μs):
   ```rust
   let cmd = MotorCommand::velocity(0, 0.0);  // Min velocity
   hub.send(cmd, &mut None)?;
   ```
6. ESC will beep to confirm calibration complete
7. Repeat for all motors

### Calibration Notes

- Only needed for PWM, OneShot, MultiShot protocols
- DShot/ProShot/CAN do NOT require calibration
- Calibration maps PWM range to ESC's throttle range
- Must recalibrate if changing PWM range or ESC

## Timing and Performance

### Update Rates by Protocol

| Protocol | Theoretical Max | Practical Rate | Use Case |
|----------|----------------|----------------|----------|
| StandardPwm | 50 Hz | 50 Hz | Basic hobby |
| OneShot125 | 4 kHz | 1-2 kHz | Sport flying |
| OneShot42 | 12 kHz | 4-8 kHz | Racing |
| MultiShot | 32 kHz | 8-16 kHz | Pro racing |
| DShot600 | Variable | 2-4 kHz | Recommended |
| DShot1200 | Variable | 4-8 kHz | Max performance |

### Latency Considerations

```
Total latency = Command generation + Protocol transmission + ESC processing

Example (DShot600):
- Command generation: 1ms (from control loop)
- DShot transmission: ~0.1ms
- ESC processing: 1-2ms
- Total: 2-3ms loop time

For optimal control:
- Use 500-1000 Hz control loop
- Use DShot600 or faster
- Minimize ESC processing delay (use modern ESCs)
```

## Troubleshooting

### "Hardware unavailable - using SIMULATION mode"

```
[WARN] BldcMotorNode motor 0: Hardware unavailable - using SIMULATION mode
[WARN]   Tried GPIO pin: 18
[WARN]   Error: Permission denied
```

**Solutions:**
1. Check GPIO permissions: `sudo usermod -a -G gpio $USER` (logout/login required)
2. Install libraspberrypi-dev: `sudo apt install libraspberrypi-dev`
3. Enable GPIO in raspi-config: `sudo raspi-config` -> Interface Options -> GPIO
4. Verify wiring with multimeter: check for 3.3V on GPIO pin when idle
5. Rebuild with hardware feature: `cargo build --features="gpio-hardware"`
6. Use hardware PWM pins only: GPIO 12, 13, 18, or 19

### Motor beeps but won't spin

**Solutions:**
1. Not armed: Call `bldc.arm_motors()` before sending commands
2. Throttle too low: Increase velocity above minimum
3. ESC not calibrated: Perform calibration (PWM/OneShot only)
4. Wrong protocol: Verify ESC supports selected protocol
5. Enable flag false: Ensure `MotorCommand.enable = true`

### Motor stutters or jitters

**Solutions:**
1. Missing common ground: Connect GND between Pi and ESC
2. Signal wire too long: Keep signal wires under 30cm
3. Power supply insufficient: Check battery voltage and current capacity
4. Wrong protocol: Try DShot instead of PWM
5. Electrical noise: Add decoupling capacitor near ESC (100μF)
6. Loose connections: Check all wiring connections

### Motor spins wrong direction

**Solutions:**
1. Software reversal (easiest): `bldc.set_direction_inverted(motor_id, true)`
2. Hardware reversal: Swap any two motor phase wires (not signal wire!)
3. For DShot: Some ESCs support direction commands

### No telemetry data

**Solutions:**
1. Telemetry not supported: ESC must support bidirectional DShot
2. Not using DShot: Switch to `BldcProtocol::DShot600`
3. Telemetry disabled: Ensure `enable_telemetry = true`
4. ESC firmware: Update ESC to firmware with bidirectional DShot support
5. GPIO limitations: Bidirectional requires special GPIO handling

### Motor runs but wrong speed

**Solutions:**
1. Velocity limits incorrect: Check `min_velocity` and `max_velocity`
2. PWM range wrong: Verify `pwm_min` and `pwm_max` match ESC
3. Needs calibration: Calibrate ESC (PWM/OneShot protocols)
4. Battery voltage low: Check battery voltage and charge level

### All motors respond to single command

**Solutions:**
1. Check motor_id: Ensure unique motor_id for each motor (0-7)
2. GPIO pin conflict: Each motor needs separate GPIO pin
3. Wiring error: Verify each ESC connected to different GPIO pin

## Safety Guidelines

**CRITICAL SAFETY WARNINGS:**

1. **Remove propellers during all testing and development**
2. **Secure robot/drone before motor tests** (use test stand)
3. **Use current-limited power supply initially** (before battery)
4. **Always implement arming sequence** (prevent accidental startup)
5. **Configure command timeout** (motors stop if signal lost)
6. **Monitor battery voltage** (land before low voltage)
7. **Check motor direction** (ensure correct rotation for your application)
8. **Keep clear of rotating parts** (motors can start unexpectedly)
9. **Use appropriate battery** (check ESC voltage rating)
10. **Never bypass safety features** (don't disable arming or timeouts)

### Recommended Safety Implementation

```rust
// 1. Require explicit arming
bldc.set_arming_required(true);

// 2. Implement command timeout
bldc.command_timeout_ms = 500;  // Stop after 500ms no command

// 3. Monitor telemetry
if telem.voltage < LOW_VOLTAGE_THRESHOLD {
    emergency_land();
}
if telem.temperature > MAX_TEMPERATURE {
    reduce_throttle();
}

// 4. Implement emergency stop
if emergency_button_pressed() {
    bldc.emergency_stop();
}

// 5. Limit acceleration
bldc.set_acceleration_limit(motor_id, SAFE_ACCEL_LIMIT);
```

## Application Examples

### Racing Quadcopter

```rust
let mut bldc = BldcMotorNode::new()?;
bldc.configure_racing_quad();  // DShot600, 0-40000 RPM, fast response
bldc.set_num_motors(4);

for i in 0..4 {
    bldc.set_pwm_pin(i, 18 + i);
}

bldc.arm_motors();
```

### Camera Gimbal

```rust
let mut bldc = BldcMotorNode::new()?;
bldc.configure_gimbal();  // DShot300, 0-5000 RPM, smooth motion
bldc.set_num_motors(3);  // Roll, pitch, yaw

for i in 0..3 {
    bldc.set_pwm_pin(i, 18 + i);
    bldc.set_acceleration_limit(i, 500.0);  // Very smooth
}

bldc.arm_motors();
```

### Fixed-Wing Aircraft

```rust
let mut bldc = BldcMotorNode::new()?;
bldc.set_num_motors(1);  // Single motor
bldc.set_protocol(BldcProtocol::OneShot125);
bldc.set_pwm_pin(0, 18);
bldc.set_velocity_limits(0, 0.0, 8000.0);
bldc.set_acceleration_limit(0, 2000.0);  // Gradual throttle

bldc.arm_motors();
```

## Simulation Mode

When hardware is unavailable (no GPIO access, wrong pins, or feature disabled), the node operates in simulation mode:

```
[INFO] BldcMotorNode motor 0 (SIM): DShot600 1250μs (25.0%)
```

Simulated behavior:
- Accepts all commands normally
- Generates realistic telemetry
- Simulates battery voltage sag
- Simulates motor heating
- Updates at correct rates
- Useful for testing control logic without hardware

Telemetry simulation:
- Voltage: 14.8V - (throttle × 0.5V) (battery sag)
- Current: throttle × 30A
- Temperature: 25C + (throttle × 50C) (heating)
- RPM: matches commanded velocity

## See Also

- [StepperMotorNode](../stepper_motor/) - Precise stepper motor control
- [DcMotorNode](../dc_motor/) - Basic DC motor control with PWM
- [ServoNode](../servo/) - Hobby servo control
- [DifferentialDriveNode](../differential_drive/) - Two-wheel drive systems
- [DynamixelNode](../dynamixel/) - Smart servo actuators
