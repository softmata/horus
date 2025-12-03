# DC Motor Control Node

PWM-based DC motor control for brushed DC motors with multi-channel support and comprehensive speed/direction control.

## Quick Start

```rust
use horus_library::nodes::{DcMotorNode, MotorDriver};
use horus_library::PwmCommand;
use horus_core::{Scheduler, Hub};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut scheduler = Scheduler::new();

    // Create DC motor controller
    let mut motor = DcMotorNode::new()?;
    motor.set_driver(MotorDriver::L298N);
    motor.set_num_channels(2);
    motor.set_gpio_pins(0, 18, 23, 24);  // Motor A: PWM=GPIO18, DIR1=GPIO23, DIR2=GPIO24
    motor.set_gpio_pins(1, 13, 25, 8);   // Motor B: PWM=GPIO13, DIR1=GPIO25, DIR2=GPIO8
    motor.set_pwm_frequency(20000);      // 20kHz (silent)

    scheduler.add(Box::new(motor), 50, Some(true));
    scheduler.run()?;
    Ok(())
}

// Send motor commands from another node:
let cmd_hub = Hub::<PwmCommand>::new("motor_cmd")?;
cmd_hub.send(PwmCommand::forward(0, 0.5), None);  // Motor 0 at 50% forward
cmd_hub.send(PwmCommand::reverse(1, 0.3), None);  // Motor 1 at 30% reverse
cmd_hub.send(PwmCommand::brake(0), None);         // Stop motor 0
```

**Subscribes to:** `motor_cmd` topic for motor speed commands.
**Publishes to:** `motor_cmd_feedback` for command confirmation.

## Overview

The DC Motor Node provides PWM-based speed control and direction switching for brushed DC motors using common H-bridge drivers. It supports up to 8 independent motor channels with configurable duty cycle limits, channel inversion, command timeout safety, and motor feedback publishing.

Compatible with L298N, TB6612FNG, DRV8833, and other H-bridge motor drivers commonly used in robotics applications.

Key features:
- Multi-channel support (up to 8 motors)
- PWM duty cycle control (-100% to +100%)
- Configurable PWM frequency (1kHz-20kHz)
- Channel inversion for motor wiring flexibility
- Duty cycle limits for safety and motor protection
- Command timeout with automatic motor stop
- Motor feedback publishing for monitoring
- Brake and coast modes
- Hardware PWM with simulation fallback

## Topics

### Subscribers

| Topic | Type | Description |
|-------|------|-------------|
| `motor_cmd` | `PwmCommand` | Motor speed and direction commands |

### Publishers

| Topic | Type | Description |
|-------|------|-------------|
| `motor_cmd_feedback` | `PwmCommand` | Echo of executed commands for monitoring |

## Configuration Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `num_channels` | `u8` | `2` | Number of motor channels (1-8) |
| `max_duty_cycle` | `f32` | `1.0` | Maximum duty cycle limit (0.0-1.0) |
| `min_duty_cycle` | `f32` | `0.0` | Minimum duty cycle / dead zone compensation (0.0-1.0) |
| `pwm_frequency` | `u32` | `20000` | PWM frequency in Hz (1000-20000) |
| `invert_channels` | `u8` | `0` | Bitfield for channel direction inversion |
| `enable_feedback` | `bool` | `true` | Enable motor command feedback publishing |
| `command_timeout_ms` | `u64` | `1000` | Command timeout in milliseconds (0=disable) |

### Motor Driver Setup

The DC Motor Node works with standard H-bridge motor drivers that use PWM for speed control and direction pins for forward/reverse:

**L298N Configuration:**
```
Raspberry Pi    L298N       Motor
GPIO 18 (PWM)   ENA         Motor A Speed
GPIO 23         IN1         Motor A Dir+
GPIO 24         IN2         Motor A Dir-
5V              +5V
GND             GND
12V Supply      +12V        Motor Power
```

**TB6612FNG Configuration:**
```
Raspberry Pi    TB6612      Motor
GPIO 18 (PWM)   PWMA        Motor A Speed
GPIO 23         AIN1        Motor A Dir+
GPIO 24         AIN2        Motor A Dir-
3.3V            VCC
GND             GND
12V Supply      VM          Motor Power
```

## Message Types

### PwmCommand

PWM control command for DC motors:

```rust
pub struct PwmCommand {
    pub channel_id: u8,       // Motor channel (0-15)
    pub duty_cycle: f32,      // PWM duty cycle (-1.0 to 1.0, negative=reverse)
    pub frequency: u32,       // PWM frequency in Hz (1000-20000)
    pub enable: bool,         // Enable motor output
    pub brake_mode: bool,     // Brake mode (true=brake, false=coast when disabled)
    pub current_limit: f32,   // Current limit in amperes (0=no limit)
    pub timestamp: u64,       // Command timestamp (ns since epoch)
}
```

**Helper Functions:**
```rust
// Create forward command
PwmCommand::forward(channel, speed);       // speed: 0.0-1.0

// Create reverse command
PwmCommand::reverse(channel, speed);       // speed: 0.0-1.0

// Create coast stop (motor free-spins)
PwmCommand::coast(channel);

// Create brake stop (motor actively braked)
PwmCommand::brake(channel);

// Create custom command
PwmCommand::new(channel, duty_cycle);      // duty_cycle: -1.0 to 1.0

// Add frequency
cmd.with_frequency(20000);                 // 20kHz PWM

// Add current limit
cmd.with_current_limit(2.0);               // 2A limit
```

**Validation:**
```rust
cmd.is_valid();         // Check if duty cycle, frequency, and limits are valid
cmd.speed();            // Get absolute speed (0-1)
cmd.is_forward();       // Check direction (true=forward, false=reverse)
```

## Public API

### Construction

```rust
use horus_library::nodes::DcMotorNode;

// Create with default topic "motor_cmd"
let mut dc_motor = DcMotorNode::new()?;

// Create with custom topic
let mut dc_motor = DcMotorNode::new_with_topic("robot.motors")?;
```

### Configuration Methods

```rust
// Set number of motor channels (1-8)
dc_motor.set_num_channels(4);

// Set duty cycle limits (min, max)
dc_motor.set_duty_cycle_limits(0.1, 0.8);  // 10% min, 80% max

// Set PWM frequency in Hz
dc_motor.set_pwm_frequency(20000);         // 20kHz (ultrasonic, silent)

// Invert a motor channel (swap forward/reverse)
dc_motor.set_channel_inverted(0, true);    // Invert channel 0

// Set command timeout in milliseconds (0=disable)
dc_motor.set_command_timeout(1000);        // 1 second timeout

// Enable/disable feedback publishing
dc_motor.set_feedback_enabled(true);

// Emergency stop all motors
dc_motor.emergency_stop();
```

### Query Methods

```rust
// Get current duty cycle for a channel
if let Some(duty) = dc_motor.get_duty_cycle(0) {
    println!("Motor 0 duty cycle: {:.1}%", duty * 100.0);
}

// Check if a channel is enabled
if dc_motor.is_channel_enabled(0) {
    println!("Motor 0 is running");
}
```

## Usage Examples

### Single Motor Control

```rust
use horus_library::prelude::*;
use std::time::Duration;

fn main() -> Result<()> {
    let mut scheduler = Scheduler::new();

    // Create DC motor node
    let mut dc_motor = DcMotorNode::new()?;
    dc_motor.set_num_channels(1);
    dc_motor.set_pwm_frequency(20000);  // 20kHz PWM
    dc_motor.set_duty_cycle_limits(0.0, 1.0);

    scheduler.add(Box::new(dc_motor), 1, Some(true));

    // Motor control node
    let control_node = node! {
        name: "motor_control",
        tick: |ctx| {
            let hub = Hub::<PwmCommand>::new("motor_cmd")?;

            // Run forward at 70%
            ctx.log_info("Forward 70%");
            hub.send(PwmCommand::forward(0, 0.7), None)?;
            std::thread::sleep(Duration::from_secs(2));

            // Run reverse at 50%
            ctx.log_info("Reverse 50%");
            hub.send(PwmCommand::reverse(0, 0.5), None)?;
            std::thread::sleep(Duration::from_secs(2));

            // Brake stop
            ctx.log_info("Brake");
            hub.send(PwmCommand::brake(0), &mut None)?;

            Ok(())
        }
    };

    scheduler.add(Box::new(control_node), 2, Some(false));
    scheduler.run()?;
    Ok(())
}
```

### Differential Drive Robot

```rust
use horus_library::prelude::*;

fn main() -> Result<()> {
    let mut scheduler = Scheduler::new();

    // Configure 2-motor differential drive
    let mut dc_motor = DcMotorNode::new()?;
    dc_motor.set_num_channels(2);
    dc_motor.set_pwm_frequency(20000);
    dc_motor.set_command_timeout(500);  // 500ms timeout for safety

    // Invert right motor (motors face opposite directions)
    dc_motor.set_channel_inverted(1, true);

    scheduler.add(Box::new(dc_motor), 1, Some(true));

    // Drive control node
    let drive_node = node! {
        name: "diff_drive",
        tick: |ctx| {
            let hub = Hub::<PwmCommand>::new("motor_cmd")?;

            // Forward
            ctx.log_info("Drive forward");
            hub.send(PwmCommand::forward(0, 0.6), None)?;  // Left motor
            hub.send(PwmCommand::forward(1, 0.6), None)?;  // Right motor
            std::thread::sleep(Duration::from_secs(2));

            // Turn right (left motor faster)
            ctx.log_info("Turn right");
            hub.send(PwmCommand::forward(0, 0.8), None)?;  // Left faster
            hub.send(PwmCommand::forward(1, 0.4), None)?;  // Right slower
            std::thread::sleep(Duration::from_secs(1));

            // Spin in place (tank turn)
            ctx.log_info("Spin left");
            hub.send(PwmCommand::reverse(0, 0.5), None)?;  // Left reverse
            hub.send(PwmCommand::forward(1, 0.5), None)?;  // Right forward
            std::thread::sleep(Duration::from_secs(1));

            // Stop with brake
            ctx.log_info("Stop");
            hub.send(PwmCommand::brake(0), &mut None)?;
            hub.send(PwmCommand::brake(1), &mut None)?;

            Ok(())
        }
    };

    scheduler.add(Box::new(drive_node), 2, Some(false));
    scheduler.run()?;
    Ok(())
}
```

### 4-Motor Mecanum Drive

```rust
use horus_library::prelude::*;

fn main() -> Result<()> {
    let mut scheduler = Scheduler::new();

    // Configure 4 motors for mecanum drive
    let mut dc_motor = DcMotorNode::new()?;
    dc_motor.set_num_channels(4);
    dc_motor.set_pwm_frequency(20000);
    dc_motor.set_duty_cycle_limits(0.15, 0.85);  // Dead zone and safety limit

    // Invert right-side motors
    dc_motor.set_channel_inverted(1, true);  // Front-right
    dc_motor.set_channel_inverted(3, true);  // Rear-right

    scheduler.add(Box::new(dc_motor), 1, Some(true));

    // Mecanum drive control
    let mecanum_node = node! {
        name: "mecanum_control",
        tick: |ctx| {
            let hub = Hub::<PwmCommand>::new("motor_cmd")?;

            // Forward
            ctx.log_info("Forward");
            for motor in 0..4 {
                hub.send(PwmCommand::forward(motor, 0.6), None)?;
            }
            std::thread::sleep(Duration::from_secs(2));

            // Strafe right
            ctx.log_info("Strafe right");
            hub.send(PwmCommand::forward(0, 0.6), None)?;   // FL
            hub.send(PwmCommand::reverse(1, 0.6), None)?;   // FR
            hub.send(PwmCommand::reverse(2, 0.6), None)?;   // RL
            hub.send(PwmCommand::forward(3, 0.6), None)?;   // RR
            std::thread::sleep(Duration::from_secs(2));

            // Rotate clockwise
            ctx.log_info("Rotate CW");
            hub.send(PwmCommand::forward(0, 0.5), None)?;   // FL
            hub.send(PwmCommand::reverse(1, 0.5), None)?;   // FR
            hub.send(PwmCommand::forward(2, 0.5), None)?;   // RL
            hub.send(PwmCommand::reverse(3, 0.5), None)?;   // RR
            std::thread::sleep(Duration::from_secs(2));

            // Stop all
            ctx.log_info("Stop");
            for motor in 0..4 {
                hub.send(PwmCommand::brake(motor), &mut None)?;
            }

            Ok(())
        }
    };

    scheduler.add(Box::new(mecanum_node), 2, Some(false));
    scheduler.run()?;
    Ok(())
}
```

### Velocity Control with Dead Zone

```rust
use horus_library::prelude::*;

fn main() -> Result<()> {
    let mut scheduler = Scheduler::new();

    // Configure motor with dead zone compensation
    let mut dc_motor = DcMotorNode::new()?;
    dc_motor.set_num_channels(1);
    dc_motor.set_pwm_frequency(20000);

    // Set min duty cycle to overcome motor static friction
    dc_motor.set_duty_cycle_limits(0.2, 1.0);  // Motor won't move below 20%

    scheduler.add(Box::new(dc_motor), 1, Some(true));

    // Gradual acceleration
    let accel_node = node! {
        name: "acceleration",
        tick: |ctx| {
            let hub = Hub::<PwmCommand>::new("motor_cmd")?;

            ctx.log_info("Gradual acceleration");
            for speed in (0..=10).map(|i| i as f32 / 10.0) {
                ctx.log_debug(&format!("Speed: {:.1}%", speed * 100.0));
                hub.send(PwmCommand::forward(0, speed), None)?;
                std::thread::sleep(Duration::from_millis(200));
            }

            std::thread::sleep(Duration::from_secs(1));

            ctx.log_info("Gradual deceleration");
            for speed in (0..=10).rev().map(|i| i as f32 / 10.0) {
                ctx.log_debug(&format!("Speed: {:.1}%", speed * 100.0));
                hub.send(PwmCommand::forward(0, speed), None)?;
                std::thread::sleep(Duration::from_millis(200));
            }

            hub.send(PwmCommand::coast(0), &mut None)?;

            Ok(())
        }
    };

    scheduler.add(Box::new(accel_node), 2, Some(false));
    scheduler.run()?;
    Ok(())
}
```

### Command Timeout Safety

```rust
use horus_library::prelude::*;

fn main() -> Result<()> {
    let mut scheduler = Scheduler::new();

    // Configure with 1 second command timeout
    let mut dc_motor = DcMotorNode::new()?;
    dc_motor.set_num_channels(1);
    dc_motor.set_command_timeout(1000);  // Auto-stop after 1 second
    dc_motor.set_feedback_enabled(true);

    scheduler.add(Box::new(dc_motor), 1, Some(true));

    // Send single command - will auto-stop after timeout
    let timeout_node = node! {
        name: "timeout_test",
        tick: |ctx| {
            let hub = Hub::<PwmCommand>::new("motor_cmd")?;

            ctx.log_info("Starting motor - will auto-stop after 1 second");
            hub.send(PwmCommand::forward(0, 0.7), None)?;

            // Don't send more commands - motor will timeout and stop
            std::thread::sleep(Duration::from_secs(3));

            ctx.log_info("Motor should have stopped by now");

            Ok(())
        }
    };

    scheduler.add(Box::new(timeout_node), 2, Some(false));
    scheduler.run()?;
    Ok(())
}
```

### Monitoring with Feedback

```rust
use horus_library::prelude::*;

fn main() -> Result<()> {
    let mut scheduler = Scheduler::new();

    // Enable feedback publishing
    let mut dc_motor = DcMotorNode::new()?;
    dc_motor.set_num_channels(2);
    dc_motor.set_feedback_enabled(true);

    scheduler.add(Box::new(dc_motor), 1, Some(true));

    // Control node
    let control_node = node! {
        name: "controller",
        tick: |ctx| {
            let hub = Hub::<PwmCommand>::new("motor_cmd")?;

            // Send commands
            hub.send(PwmCommand::forward(0, 0.6), None)?;
            hub.send(PwmCommand::forward(1, 0.8), None)?;

            std::thread::sleep(Duration::from_secs(2));

            hub.send(PwmCommand::brake(0), &mut None)?;
            hub.send(PwmCommand::brake(1), &mut None)?;

            Ok(())
        }
    };

    // Monitoring node
    let monitor_node = node! {
        name: "monitor",
        tick: |ctx| {
            let hub = Hub::<PwmCommand>::new("motor_cmd_feedback")?;

            // Read all feedback messages
            while let Some(feedback) = hub.recv(None) {
                ctx.log_info(&format!(
                    "Motor {} feedback: {:.1}% @ {}Hz, enabled={}",
                    feedback.channel_id,
                    feedback.duty_cycle * 100.0,
                    feedback.frequency,
                    feedback.enable
                ));
            }

            Ok(())
        }
    };

    scheduler.add(Box::new(control_node), 2, Some(false));
    scheduler.add(Box::new(monitor_node), 3, Some(true));
    scheduler.run()?;
    Ok(())
}
```

## Hardware Setup

### System Requirements

```bash
# Install GPIO library
sudo apt install libraspberrypi-dev

# Enable GPIO and PWM interfaces
sudo raspi-config
# Select: Interface Options -> GPIO -> Enable
# Select: Interface Options -> SPI -> Enable (for hardware PWM)

# Add user to GPIO group
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

### L298N H-Bridge Wiring

```
Raspberry Pi          L298N           Motors
GPIO 18 (PWM0)   -->  ENA             Motor A Speed Control
GPIO 23          -->  IN1             Motor A Direction+
GPIO 24          -->  IN2             Motor A Direction-
GPIO 13 (PWM1)   -->  ENB             Motor B Speed Control
GPIO 25          -->  IN3             Motor B Direction+
GPIO 8           -->  IN4             Motor B Direction-
5V               -->  +5V             Logic Power
GND              ---  GND             Common Ground
                      +12V       -->  Motor Power Supply
                      Motor A+   -->  Left Motor+
                      Motor A-   -->  Left Motor-
                      Motor B+   -->  Right Motor+
                      Motor B-   -->  Right Motor-
```

**L298N Specifications:**
- Logic voltage: 5V
- Motor voltage: 5V-35V
- Max current: 2A per channel (continuous)
- Peak current: 3A per channel
- Dual H-bridge
- Built-in flyback diodes
- Thermal protection

### TB6612FNG Wiring

```
Raspberry Pi          TB6612          Motors
GPIO 18 (PWM0)   -->  PWMA            Motor A Speed
GPIO 23          -->  AIN1            Motor A Direction+
GPIO 24          -->  AIN2            Motor A Direction-
GPIO 13 (PWM1)   -->  PWMB            Motor B Speed
GPIO 25          -->  BIN1            Motor B Direction+
GPIO 8           -->  BIN2            Motor B Direction-
3.3V             -->  VCC             Logic Power
                      STBY       -->  3.3V (always enabled)
GND              ---  GND             Common Ground
                      VM         -->  Motor Power (4.5-13.5V)
                      A01/A02    -->  Motor A
                      B01/B02    -->  Motor B
```

**TB6612FNG Specifications:**
- Logic voltage: 3.3V or 5V compatible
- Motor voltage: 4.5V-13.5V
- Max current: 1.2A per channel (continuous)
- Peak current: 3.2A per channel (< 10ms)
- Dual H-bridge
- More efficient than L298N
- Lower voltage drop (~0.5V vs 2V)
- Built-in flyback diodes

### DRV8833 Wiring

```
Raspberry Pi          DRV8833         Motors
GPIO 18 (PWM0)   -->  AIN1            Motor A PWM+
GPIO 23          -->  AIN2            Motor A PWM-
GPIO 13 (PWM1)   -->  BIN1            Motor B PWM+
GPIO 25          -->  BIN2            Motor B PWM-
3.3V             -->  VCC             Logic Power
GND              ---  GND             Common Ground
                      VM         -->  Motor Power (2.7-10.8V)
                      AOUT1/2    -->  Motor A
                      BOUT1/2    -->  Motor B
```

**DRV8833 Specifications:**
- Logic voltage: 3.3V or 5V compatible
- Motor voltage: 2.7V-10.8V
- Max current: 1.5A per channel (continuous)
- Peak current: 2A per channel
- Dual H-bridge
- Very low voltage drop
- Built-in flyback diodes
- Sleep mode support

### PWM Pin Selection (Raspberry Pi)

**Hardware PWM Pins (Recommended):**
- GPIO 12 (PWM0)
- GPIO 13 (PWM1)
- GPIO 18 (PWM0)
- GPIO 19 (PWM1)

Hardware PWM provides precise, jitter-free PWM signals critical for smooth motor control.

**Software PWM (Fallback):**
Any GPIO pin can be used for software PWM, but may have timing jitter.

## PWM Frequency Guidelines

### Recommended Frequencies

```rust
// 20kHz - Ultrasonic (silent, preferred for most applications)
dc_motor.set_pwm_frequency(20000);

// 10kHz - High frequency (minimal audible noise)
dc_motor.set_pwm_frequency(10000);

// 5kHz - Medium frequency (slight audible whine)
dc_motor.set_pwm_frequency(5000);

// 1kHz - Low frequency (audible buzz, use only if required)
dc_motor.set_pwm_frequency(1000);
```

### Frequency Considerations

| Frequency | Advantages | Disadvantages |
|-----------|-----------|---------------|
| 20-25kHz | Silent operation, smooth control | Higher switching losses |
| 10-15kHz | Good balance, minimal noise | Some switching losses |
| 5-10kHz | Lower switching losses | Audible whine |
| 1-5kHz | Very low switching losses | Loud audible noise |

**Best Practice:** Use 20kHz for most applications to avoid audible noise.

## Motor Control Modes

### Brake vs Coast

**Brake Mode:**
```rust
// Active braking - both motor leads shorted
// Motor stops quickly with regenerative braking
hub.send(PwmCommand::brake(0), &mut None)?;
```

**Coast Mode:**
```rust
// Free-spinning - motor leads disconnected
// Motor coasts to a stop naturally
hub.send(PwmCommand::coast(0), &mut None)?;
```

**When to Use:**
- **Brake:** Precision positioning, quick stops, holding position
- **Coast:** Gentle stops, reducing mechanical stress, energy efficiency

### Direction Control

**Forward/Reverse:**
```rust
// Positive duty cycle = forward
hub.send(PwmCommand::forward(0, 0.7), None)?;

// Negative duty cycle = reverse
hub.send(PwmCommand::reverse(0, 0.7), None)?;

// Or use direct duty cycle
hub.send(PwmCommand::new(0, 0.7), None)?;   // Forward
hub.send(PwmCommand::new(0, -0.7), None)?;  // Reverse
```

**Channel Inversion:**
```rust
// For motors wired backwards or mirrored configurations
dc_motor.set_channel_inverted(1, true);  // Swap direction for channel 1
```

## Safety Features

### Command Timeout

Automatically stops motors if no command received within timeout period:

```rust
// Enable 1 second timeout
dc_motor.set_command_timeout(1000);

// Disable timeout (use with caution!)
dc_motor.set_command_timeout(0);
```

**How it works:**
1. Each motor command updates last command timestamp
2. Node checks for timeout on every tick
3. If elapsed time > timeout, motor automatically stops
4. Warning logged and stop command published

### Duty Cycle Limits

Protect motors and provide dead zone compensation:

```rust
// Limit maximum speed to 80%
dc_motor.set_duty_cycle_limits(0.0, 0.8);

// Set minimum threshold (dead zone compensation)
dc_motor.set_duty_cycle_limits(0.2, 1.0);

// Combine both
dc_motor.set_duty_cycle_limits(0.15, 0.85);
```

**Use cases:**
- **Max limit:** Protect weak motors, reduce current draw, safety margin
- **Min limit:** Overcome static friction, ensure motor actually moves

### Emergency Stop

Immediately stop all motors:

```rust
// Stops all configured motors with brake mode
dc_motor.emergency_stop();
```

## Best Practices

1. **Use appropriate PWM frequency:**
   ```rust
   dc_motor.set_pwm_frequency(20000);  // 20kHz - silent operation
   ```

2. **Enable command timeout for safety:**
   ```rust
   dc_motor.set_command_timeout(1000);  // 1 second timeout
   ```

3. **Set duty cycle limits to protect motors:**
   ```rust
   dc_motor.set_duty_cycle_limits(0.15, 0.85);  // Safe operating range
   ```

4. **Use brake mode for precision stops:**
   ```rust
   hub.send(PwmCommand::brake(0), &mut None)?;  // Active braking
   ```

5. **Invert channels instead of rewiring:**
   ```rust
   dc_motor.set_channel_inverted(1, true);  // Fix motor direction in software
   ```

6. **Monitor feedback for debugging:**
   ```rust
   dc_motor.set_feedback_enabled(true);
   // Subscribe to "motor_cmd_feedback" topic
   ```

7. **Separate motor power from logic power:**
   - Use separate power supplies for motors (high current)
   - Use regulated 5V/3.3V for Raspberry Pi and logic
   - Connect all grounds together (common ground)

8. **Add capacitors for noise reduction:**
   - 100uF electrolytic across motor power supply
   - 0.1uF ceramic across each motor terminal
   - Reduces electrical noise and protects driver

## Troubleshooting

### "Hardware unavailable - using SIMULATION mode"

```
[WARN] DcMotorNode: Hardware unavailable - using SIMULATION mode
```

**Solutions:**
1. Check GPIO permissions: `sudo usermod -a -G gpio $USER`
2. Install GPIO library: `sudo apt install libraspberrypi-dev`
3. Enable GPIO in raspi-config: `sudo raspi-config`
4. Rebuild with hardware feature: `cargo build --features="gpio-hardware"`
5. Verify wiring and power connections

### Motor doesn't spin

**Solutions:**
1. Check motor power supply (VM/VCC on driver)
2. Verify motor connections to driver outputs
3. Increase duty cycle (may be below motor stall threshold)
4. Check if max duty cycle limit too low
5. Test motor directly with battery to verify it works
6. Verify enable pin is HIGH (if driver has separate enable)

### Motor runs at full speed always

**Solutions:**
1. PWM pin not connected or not configured correctly
2. Enable pin (ENA/ENB) not connected to PWM
3. Check if using hardware PWM pins (GPIO 12, 13, 18, 19)
4. Verify PWM frequency not too low
5. Check driver enable jumper (L298N has removable jumpers)

### Motor direction doesn't change

**Solutions:**
1. Direction pins (IN1/IN2) not connected properly
2. Check wiring between GPIO and driver direction pins
3. Verify common ground between Pi and driver
4. Test with `set_channel_inverted()` to rule out software issue
5. Check if motor wires crossed (swap motor wires)

### Motor stutters or jerks

**Solutions:**
1. Increase PWM frequency: `set_pwm_frequency(20000)`
2. Add capacitors across motor terminals (0.1uF)
3. Check for loose connections
4. Verify adequate power supply current rating
5. Enable dead zone compensation: `set_duty_cycle_limits(0.2, 1.0)`
6. Check motor isn't mechanically binding

### Motor whines or buzzes

**Solutions:**
1. Increase PWM frequency to ultrasonic range (20kHz+)
2. Add 0.1uF capacitor across motor terminals
3. Check for mechanical resonance in mounting
4. Verify motor isn't overloaded

### Motors work individually but not together

**Solutions:**
1. Insufficient power supply current capacity
2. Add larger capacitor to power supply (1000uF+)
3. Use separate power supply for motors vs logic
4. Check voltage drops under load
5. Reduce max duty cycle limits to lower current draw

### Command timeout stops motors unexpectedly

**Solutions:**
1. Increase timeout: `set_command_timeout(2000)`
2. Ensure control loop sends commands regularly
3. Check for timing issues in control node
4. Disable timeout if using manual control: `set_command_timeout(0)`

### Raspberry Pi resets when motor starts

**Solutions:**
1. Separate power supplies (never power motors from Pi)
2. Add large capacitor (1000uF+) to motor power supply
3. Reduce inrush current with duty cycle limits
4. Use external 5V regulator for Raspberry Pi
5. Check power supply current rating adequate

## H-Bridge Truth Tables

### L298N / Generic H-Bridge

| IN1 | IN2 | ENA | Motor Action |
|-----|-----|-----|--------------|
| LOW | LOW | PWM | Brake (short) |
| LOW | HIGH | PWM | Reverse |
| HIGH | LOW | PWM | Forward |
| HIGH | HIGH | PWM | Brake (short) |
| X | X | LOW | Coast (disable) |

**PWM on ENA/ENB controls speed, IN1/IN2 control direction.**

### TB6612FNG / DRV8833

| AIN1 | AIN2 | Motor Action |
|------|------|--------------|
| LOW | LOW | Coast/Standby |
| LOW | HIGH (PWM) | Reverse |
| HIGH (PWM) | LOW | Forward |
| HIGH | HIGH | Brake (short) |

**PWM on direction pins, or separate PWMA/PWMB pin.**

## Performance Characteristics

### Timing

- **Command latency:** < 1ms (depends on node scheduling)
- **PWM update rate:** Determined by PWM frequency
- **Timeout check rate:** Every node tick

### Resource Usage

- **Memory:** ~2KB per node instance
- **CPU:** Minimal (< 1% on Raspberry Pi 4)
- **GPIO pins:** 2-3 per motor (PWM + 2 direction pins)

### Limitations

- Maximum 8 channels per node instance
- PWM frequency limited by hardware capabilities
- Software PWM may have jitter (use hardware PWM pins)
- Command timeout minimum resolution: 1ms

## Simulation Mode

When hardware is unavailable, the node operates in simulation mode:

```
[INFO] DcMotorNode: Motor 0: duty=75.0%, enable=true, freq=10000Hz
```

Simulated behavior:
- Accepts all commands normally
- Logs motor state changes
- Publishes feedback messages
- No actual hardware control
- Useful for logic testing and development without hardware

## See Also

- [StepperMotorNode](../stepper_motor/) - Precise position control with stepper motors
- [ServoNode](../servo/) - Position-controlled servo motors
- [DifferentialDriveNode](../differential_drive/) - High-level differential drive control
- [OdometryNode](../odometry/) - Wheel odometry for position tracking
- [PidControllerNode](../pid_controller/) - Closed-loop motor control
