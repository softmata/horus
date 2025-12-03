# Dynamixel Servo Controller Node

Smart servo controller supporting 18+ Dynamixel models with Protocol 1.0 and 2.0 for robot arms, humanoids, grippers, and articulated mechanisms.

## Overview

The Dynamixel Node provides comprehensive control and feedback for Dynamixel smart servo motors. It supports position, velocity, and current/torque control modes with extensive monitoring capabilities including temperature, voltage, and hardware error detection. The node handles both individual servo commands and synchronized multi-servo operations with automatic hardware/simulation fallback.

Supports AX, MX, RX, EX, X, XL, XC, XM, XH, P, and PRO series servos.

Key features:
- 18+ Dynamixel models (Protocol 1.0 and 2.0)
- Multi-servo support (up to 253 servos per bus)
- Position, velocity, and current/torque control
- Extended position mode (multi-turn absolute positioning)
- PWM control mode
- PID parameter tuning
- Temperature and voltage monitoring
- Hardware error detection and reporting
- Bulk read/write for synchronized motion
- Communication statistics tracking
- Simulation fallback when hardware unavailable
- Preset configurations for common robot setups

## Topics

### Subscribers

| Topic | Type | Description |
|-------|------|-------------|
| `dynamixel/servo_cmd` | `ServoCommand` | Individual servo position commands |
| `dynamixel/joint_cmd` | `JointCommand` | Multi-joint coordinated commands |

### Publishers

| Topic | Type | Description |
|-------|------|-------------|
| `dynamixel/feedback` | `ServoCommand` | Servo state feedback (position, velocity, torque status) |

## Configuration Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `port` | `String` | `"/dev/ttyUSB0"` | Serial port device path |
| `protocol` | `DynamixelProtocol` | `Protocol2` | Protocol version (Protocol1 or Protocol2) |
| `baud_rate` | `u32` | `1000000` | Communication baud rate (9600 to 4500000 bps) |
| `enable_bulk_operations` | `bool` | `true` | Enable bulk read/write for performance |

### Servo Parameters (per servo)

| Parameter | Type | Description |
|-----------|------|-------------|
| `id` | `u8` | Servo ID (1-252) |
| `model` | `DynamixelModel` | Servo model type |
| `torque_enabled` | `bool` | Enable/disable holding torque |
| `goal_position` | `f32` | Target position in radians |
| `goal_velocity` | `f32` | Target velocity in rad/s |
| `goal_current` | `f32` | Target current in mA |

## Supported Models

### Protocol 1.0 Models

**AX Series (Entry-Level)**
- `AX12A`: 0-300 degrees, 0.15 Nm stall torque, 1024 resolution
- `AX18A`: 0-300 degrees, 0.18 Nm stall torque, 1024 resolution

**MX Series (Mid-Range)**
- `MX28`: 0-360 degrees, 0.34 Nm, 4096 resolution
- `MX64`: 0-360 degrees, 0.74 Nm, 4096 resolution
- `MX106`: 0-360 degrees, 1.02 Nm, 4096 resolution

**RX Series**
- `RX-10`, `RX-24F`, `RX-28`, `RX-64`

**EX Series**
- `EX-106+`: High torque

### Protocol 2.0 Models

**XL Series (Compact/Budget)**
- `XL330M077`: 24.09 rad/s, 0.059 Nm, 4096 resolution
- `XL330M288`: 19.42 rad/s, 0.038 Nm, 4096 resolution
- `XL430W250`: 5.02 rad/s, 0.14 Nm, 4096 resolution

**XC Series (Compact)**
- `XC430W150`: 0-360 degrees, compact design
- `XC430W240`: 0-360 degrees, higher torque

**XM Series (Performance)**
- `XM430W210`: 7.65 rad/s, 0.29 Nm, 4096 resolution
- `XM430W350`: 4.82 rad/s, 0.39 Nm, 4096 resolution
- `XM540W150`: 5.44 rad/s, 1.05 Nm, 4096 resolution
- `XM540W270`: 3.14 rad/s, 1.49 Nm, 4096 resolution

**XH Series (High-Performance)**
- `XH430V210`: 0.44 Nm, position feedback
- `XH430V350`: 0.69 Nm, position feedback
- `XH430W210`: 0.44 Nm
- `XH430W350`: 0.69 Nm
- `XH540W150`: 1.16 Nm
- `XH540W270`: 1.74 Nm
- `XH540V150`: 1.16 Nm, position feedback
- `XH540V270`: 1.74 Nm, position feedback

**PRO Series (Industrial)**
- `H42P020`: 42mm frame
- `H54P100`: 54mm frame, 5.40 Nm
- `H54P200`: 54mm frame, high torque
- `M42P010`: 42mm modular
- `M54P040`: 54mm modular
- `M54P060`: 54mm modular

## Message Types

### ServoCommand

Individual servo control message:

```rust
pub struct ServoCommand {
    pub servo_id: u8,        // Servo ID (1-252)
    pub position: f32,       // Target position in radians
    pub speed: f32,          // Movement speed (0-1, 0=max speed)
    pub enable: bool,        // Torque enable
    pub timestamp: u64,      // Command time (ns since epoch)
}
```

**Helper Methods**:
```rust
ServoCommand::new(servo_id: u8, position: f32)
ServoCommand::with_speed(servo_id: u8, position: f32, speed: f32)
ServoCommand::disable(servo_id: u8)
ServoCommand::from_degrees(servo_id: u8, degrees: f32)
```

### JointCommand

Multi-joint coordinated control message:

```rust
pub struct JointCommand {
    pub joint_names: [[u8; 32]; 16],  // Joint names (max 16 joints)
    pub joint_count: u8,               // Number of active joints
    pub positions: [f64; 16],          // Position commands in radians
    pub velocities: [f64; 16],         // Velocity commands in rad/s
    pub efforts: [f64; 16],            // Effort/torque commands in Nm
    pub modes: [u8; 16],               // Control mode per joint
    pub timestamp: u64,                // Command time (ns since epoch)
}
```

**Control Modes**:
- `JointCommand::MODE_POSITION = 0`: Position control
- `JointCommand::MODE_VELOCITY = 1`: Velocity control
- `JointCommand::MODE_EFFORT = 2`: Torque/effort control

**Helper Methods**:
```rust
JointCommand::new()
joint_cmd.add_position(name: &str, position: f64)
joint_cmd.add_velocity(name: &str, velocity: f64)
```

## Public API

### Construction

```rust
use horus_library::nodes::DynamixelNode;
use horus_library::nodes::DynamixelProtocol;

// Create with default settings (1 Mbps, Protocol 2.0)
let mut dxl = DynamixelNode::new("/dev/ttyUSB0", DynamixelProtocol::Protocol2)?;

// Create with Protocol 1.0 for legacy servos
let mut dxl = DynamixelNode::new("/dev/ttyUSB0", DynamixelProtocol::Protocol1)?;
```

### Configuration Methods

```rust
// Set communication baud rate (9600 to 4500000 bps)
dxl.set_baud_rate(1_000_000);  // 1 Mbps (standard)
dxl.set_baud_rate(3_000_000);  // 3 Mbps (fast)
dxl.set_baud_rate(4_500_000);  // 4.5 Mbps (maximum)

// Add servos by ID and model
dxl.add_servo(1, DynamixelModel::XM430W350);
dxl.add_servo(2, DynamixelModel::XM430W350);
dxl.add_servo(3, DynamixelModel::XM540W270);

// Remove a servo
dxl.remove_servo(1);

// Enable/disable torque for individual servo
dxl.enable_torque(1, true);   // Enable holding torque
dxl.enable_torque(1, false);  // Disable (servo goes limp)

// Enable/disable torque for all servos
dxl.enable_all_torque(true);   // Enable all
dxl.enable_all_torque(false);  // Disable all (safety)
```

### Query Methods

```rust
// Get servo position in radians
if let Some(position) = dxl.get_position(1) {
    println!("Position: {:.3} rad ({:.1}°)", position, position.to_degrees());
}

// Get servo velocity in rad/s
if let Some(velocity) = dxl.get_velocity(1) {
    println!("Velocity: {:.3} rad/s", velocity);
}

// Get servo current in mA
if let Some(current) = dxl.get_current(1) {
    println!("Current: {:.0} mA", current);
}

// Get servo temperature in °C
if let Some(temp) = dxl.get_temperature(1) {
    println!("Temperature: {}°C", temp);
}

// Check for hardware errors
if dxl.has_error(1) {
    println!("Servo 1 has a hardware error!");
}

// Get communication statistics (successful, errors, success_rate)
let (success, errors, rate) = dxl.get_comm_stats();
println!("Comm: {} ok, {} err ({:.1}% success)", success, errors, rate);
```

### Utility Methods

```rust
// Ping a servo to check if responsive
if dxl.ping(1) {
    println!("Servo 1 is responding");
}

// Scan for all servos on the bus
let found_servos = dxl.scan(None);
println!("Found servos: {:?}", found_servos);

// Reboot a servo
dxl.reboot(1, None);

// Factory reset a servo (WARNING: resets ID and baud rate!)
dxl.factory_reset(1, None);
```

### Preset Configurations

```rust
// Configure for 6-DOF robot arm
dxl.configure_6dof_arm();
// Adds: Base (XM430W350), Shoulder (XM430W350), Elbow (XM430W210),
//       Wrist Roll/Pitch/Yaw (XL430W250)

// Configure for humanoid manipulator (OpenManipulator-X style)
dxl.configure_manipulator();
// Adds: 5x XM430W350 servos (IDs 1-5)

// Configure for pan-tilt camera system
dxl.configure_pan_tilt();
// Adds: Pan (XL430W250), Tilt (XL430W250)
```

## Usage Examples

### Single Servo Control

```rust
use horus_library::prelude::*;

fn main() -> Result<()> {
    let mut scheduler = Scheduler::new();

    // Create Dynamixel controller
    let mut dxl = DynamixelNode::new("/dev/ttyUSB0", DynamixelProtocol::Protocol2)?;
    dxl.set_baud_rate(1_000_000);

    // Add servo
    dxl.add_servo(1, DynamixelModel::XM430W350);
    dxl.enable_torque(1, true);

    scheduler.add(Box::new(dxl), 1, Some(true));

    // Control node
    let controller = node! {
        name: "servo_controller",
        tick: |ctx| {
            let hub = Hub::<ServoCommand>::new("dynamixel.servo_cmd")?;

            // Move to 90 degrees
            let cmd = ServoCommand::from_degrees(1, 90.0);
            hub.send(cmd, &mut None)?;

            Ok(())
        }
    };

    scheduler.add(Box::new(controller), 2, Some(true));
    scheduler.run()?;
    Ok(())
}
```

### Robot Arm (6-DOF)

```rust
use horus_library::prelude::*;

fn main() -> Result<()> {
    let mut scheduler = Scheduler::new();

    // Configure 6-DOF arm
    let mut dxl = DynamixelNode::new("/dev/ttyUSB0", DynamixelProtocol::Protocol2)?;
    dxl.set_baud_rate(1_000_000);
    dxl.configure_6dof_arm();  // Preset configuration
    dxl.enable_all_torque(true);

    scheduler.add(Box::new(dxl), 1, Some(true));

    // Arm control node
    let arm_control = node! {
        name: "arm_control",
        tick: |ctx| {
            let hub = Hub::<ServoCommand>::new("dynamixel.servo_cmd")?;

            // Move to home position
            let home_angles = [0.0, -30.0, 60.0, 0.0, -30.0, 0.0]; // degrees

            for (servo_id, &angle) in home_angles.iter().enumerate() {
                let cmd = ServoCommand::from_degrees((servo_id + 1) as u8, angle);
                hub.send(cmd, &mut None)?;
            }

            // Monitor feedback
            let feedback_hub = Hub::<ServoCommand>::new("dynamixel.feedback")?;
            while let Some(feedback) = feedback_hub.recv(None) {
                ctx.log_debug(&format!(
                    "Servo {}: pos={:.2}rad vel={:.2}rad/s enabled={}",
                    feedback.servo_id, feedback.position, feedback.speed, feedback.enable
                ));
            }

            Ok(())
        }
    };

    scheduler.add(Box::new(arm_control), 2, Some(true));
    scheduler.run()?;
    Ok(())
}
```

### Multi-Joint Coordinated Control

```rust
use horus_library::prelude::*;

fn main() -> Result<()> {
    let mut scheduler = Scheduler::new();

    // Setup controller
    let mut dxl = DynamixelNode::new("/dev/ttyUSB0", DynamixelProtocol::Protocol2)?;
    dxl.set_baud_rate(1_000_000);

    for id in 1..=5 {
        dxl.add_servo(id, DynamixelModel::XM430W350);
    }
    dxl.enable_all_torque(true);

    scheduler.add(Box::new(dxl), 1, Some(true));

    // Coordinated control node
    let coordinator = node! {
        name: "joint_coordinator",
        tick: |ctx| {
            let hub = Hub::<JointCommand>::new("dynamixel.joint_cmd")?;

            // Create synchronized joint command
            let mut joint_cmd = JointCommand::new();
            joint_cmd.add_position("joint_1", 0.0)?;
            joint_cmd.add_position("joint_2", -0.523)?;  // -30 degrees
            joint_cmd.add_position("joint_3", 1.047)?;   // 60 degrees
            joint_cmd.add_position("joint_4", 0.0)?;
            joint_cmd.add_position("joint_5", -0.523)?;

            hub.send(joint_cmd, &mut None)?;

            Ok(())
        }
    };

    scheduler.add(Box::new(coordinator), 2, Some(true));
    scheduler.run()?;
    Ok(())
}
```

### Pan-Tilt Camera Tracking

```rust
use horus_library::prelude::*;

fn main() -> Result<()> {
    let mut scheduler = Scheduler::new();

    // Setup pan-tilt system
    let mut dxl = DynamixelNode::new("/dev/ttyUSB0", DynamixelProtocol::Protocol2)?;
    dxl.set_baud_rate(1_000_000);
    dxl.configure_pan_tilt();  // Pan=servo 1, Tilt=servo 2
    dxl.enable_all_torque(true);

    scheduler.add(Box::new(dxl), 1, Some(true));

    // Tracking node
    let tracker = node! {
        name: "camera_tracker",
        tick: |ctx| {
            let hub = Hub::<ServoCommand>::new("dynamixel.servo_cmd")?;

            // Smooth pan sweep (-45 to +45 degrees)
            let time = std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap()
                .as_secs_f32();

            let pan_angle = 45.0 * (time * 0.5).sin();
            let tilt_angle = 15.0;  // Fixed tilt

            hub.send(ServoCommand::from_degrees(1, pan_angle), None)?;
            hub.send(ServoCommand::from_degrees(2, tilt_angle), None)?;

            Ok(())
        }
    };

    scheduler.add(Box::new(tracker), 2, Some(true));
    scheduler.run()?;
    Ok(())
}
```

### Position-Velocity Mixed Control

```rust
use horus_library::prelude::*;

fn main() -> Result<()> {
    let mut scheduler = Scheduler::new();

    let mut dxl = DynamixelNode::new("/dev/ttyUSB0", DynamixelProtocol::Protocol2)?;
    dxl.set_baud_rate(1_000_000);

    dxl.add_servo(1, DynamixelModel::XM430W350);  // Position control
    dxl.add_servo(2, DynamixelModel::XM430W350);  // Velocity control
    dxl.enable_all_torque(true);

    scheduler.add(Box::new(dxl), 1, Some(true));

    let controller = node! {
        name: "mixed_control",
        tick: |ctx| {
            let hub = Hub::<JointCommand>::new("dynamixel.joint_cmd")?;

            let mut cmd = JointCommand::new();
            cmd.add_position("joint_1", 1.57)?;      // 90 degrees position
            cmd.add_velocity("joint_2", 2.0)?;       // 2 rad/s velocity

            hub.send(cmd, &mut None)?;

            Ok(())
        }
    };

    scheduler.add(Box::new(controller), 2, Some(true));
    scheduler.run()?;
    Ok(())
}
```

### Temperature and Error Monitoring

```rust
use horus_library::prelude::*;

fn main() -> Result<()> {
    let mut scheduler = Scheduler::new();

    let mut dxl = DynamixelNode::new("/dev/ttyUSB0", DynamixelProtocol::Protocol2)?;
    dxl.set_baud_rate(1_000_000);
    dxl.configure_6dof_arm();
    dxl.enable_all_torque(true);

    scheduler.add(Box::new(dxl), 1, Some(true));

    let monitor = node! {
        name: "safety_monitor",
        tick: |ctx| {
            // Access node through scheduler would be needed in real implementation
            // This is a conceptual example

            for servo_id in 1..=6 {
                // Check temperature
                if let Some(temp) = /* dxl.get_temperature(servo_id) */ Some(50) {
                    if temp > 70 {
                        ctx.log_warning(&format!(
                            "Servo {} overheating: {}°C - reducing load!",
                            servo_id, temp
                        ));
                    } else if temp > 60 {
                        ctx.log_warning(&format!(
                            "Servo {} hot: {}°C",
                            servo_id, temp
                        ));
                    }
                }

                // Check for hardware errors
                if /* dxl.has_error(servo_id) */ false {
                    ctx.log_error(&format!(
                        "Servo {} hardware error detected!",
                        servo_id
                    ));
                }
            }

            Ok(())
        }
    };

    scheduler.add(Box::new(monitor), 3, Some(true));
    scheduler.run()?;
    Ok(())
}
```

## Hardware Setup

### System Requirements

```bash
# Install serial port tools
sudo apt install setserial

# Add user to dialout group for serial port access
sudo usermod -a -G dialout $USER

# Logout and login again for group change to take effect

# Verify serial port
ls -l /dev/ttyUSB*
# Should show: crw-rw---- 1 root dialout ... /dev/ttyUSB0

# Check permissions
groups
# Should include: dialout
```

### Wiring Diagram (Daisy Chain)

```
Computer         U2D2/USB2Dynamixel    Servo 1       Servo 2       Servo 3
  USB    <--->    USB Interface    -->  Data  -->    Data  -->    Data
                       Power       -->   VDD   -->     VDD  -->     VDD
                       Ground      -->   GND   -->     GND  -->     GND
```

Daisy-chain topology allows up to 253 servos on one bus.

### U2D2 / USB2Dynamixel Adapter

```
Computer USB Port
      |
   U2D2 Adapter
      |
   JST 3-pin connector (Data, VDD, GND)
      |
   Dynamixel Bus
```

**U2D2 Features**:
- USB to TTL/RS-485 conversion
- Automatic direction switching for half-duplex
- LED indicators for TX/RX/Power
- Compatible with all Dynamixel servos

**Connection**:
1. Connect U2D2 to computer USB port
2. Connect Dynamixel servo to U2D2 data port
3. Connect external power supply to servo VDD/GND
4. Device appears as /dev/ttyUSB0 (Linux) or COM port (Windows)

### Power Requirements

| Servo Series | Voltage | Current (per servo) |
|--------------|---------|---------------------|
| AX Series | 9-12V (recommended 11.1V) | Up to 1.5A |
| MX Series | 11.1-14.8V (3S LiPo) | Up to 2.5A |
| XL Series | 6-9V (2S LiPo) | Up to 1.5A |
| XM/XH Series | 11.1-14.8V (3S LiPo) | Up to 3.5A |
| PRO/P Series | 24V | Up to 5A |

**IMPORTANT**:
- Do NOT power servos from USB (insufficient current)
- Use separate power supply rated for total servo load
- Connect power supply ground to U2D2 ground
- Use appropriate gauge wire (18-22 AWG for most servos)
- Add capacitors near servos (100-220uF) to reduce voltage spikes

### Enable in Project

```toml
[dependencies]
horus_library = { version = "0.1", features = ["serial-hardware"] }
```

```bash
# Build with serial hardware support
cargo build --features="serial-hardware"

# Run with serial hardware support
cargo run --features="serial-hardware"
```

## Communication Protocols

### Protocol 1.0 (Legacy)

- Used by: AX, MX (legacy), RX, EX series
- Packet structure: [0xFF] [0xFF] [ID] [Length] [Instruction] [Parameters...] [Checksum]
- Checksum: 8-bit complement
- Baud rates: 9600 to 1M bps
- Control table addresses differ from Protocol 2.0

### Protocol 2.0 (Modern)

- Used by: X, XL, XC, XM, XH, P, PRO series
- Packet structure: [0xFF] [0xFF] [0xFD] [0x00] [ID] [Length] [Instruction] [Parameters...] [CRC]
- CRC: 16-bit CRC-16-IBM
- Baud rates: 9600 to 4.5M bps
- Enhanced error detection
- More features (extended position, current control, etc.)

### Baud Rate Selection

```rust
// Standard rates
dxl.set_baud_rate(9600);       // Very slow, long cables
dxl.set_baud_rate(57600);      // Slow, testing
dxl.set_baud_rate(115200);     // Moderate
dxl.set_baud_rate(1_000_000);  // Standard (recommended)
dxl.set_baud_rate(3_000_000);  // Fast (short cables)
dxl.set_baud_rate(4_500_000);  // Maximum (very short cables)
```

**Guidelines**:
- Start with 1 Mbps (reliable for most setups)
- Use lower rates for long cables (>3m) or noisy environments
- Use higher rates for low-latency control with short cables
- All servos on bus must use same baud rate
- Change servo baud rate using Dynamixel Wizard

## Timing and Performance

### Command Rate

The node processes commands at 1 kHz (1ms tick rate):
- Servo simulation: 1ms updates
- Bulk read: 100Hz (every 10ms)
- Feedback publishing: Every tick for enabled servos
- Status logging: Every 10 seconds

### Communication Timing

```
Typical command latency:
- 1 Mbps: ~1-2ms per servo
- 3 Mbps: ~0.5-1ms per servo

Bulk operations (sync write/read):
- Much faster than individual commands
- All servos updated in single transaction
```

### Motion Control Accuracy

- **Position resolution**: Depends on model (1024 or 4096 counts/revolution)
  - AX series: 1024 steps = 0.35 degrees per step
  - MX/X series: 4096 steps = 0.088 degrees per step
- **Velocity control**: Proportional to max velocity of model
- **Current control**: mA resolution (Protocol 2.0 only)

### Servo Response Time

- **Position mode**: Servo reaches target with P-controller simulation
- **Velocity mode**: Servo tracks velocity with acceleration limiting
- **Real hardware**: Response depends on PID gains and mechanical load

## Best Practices

1. **Use Protocol 2.0 for new projects**:
   ```rust
   let dxl = DynamixelNode::new("/dev/ttyUSB0", DynamixelProtocol::Protocol2)?;
   // Better error detection, more features, higher performance
   ```

2. **Set appropriate baud rate**:
   ```rust
   dxl.set_baud_rate(1_000_000);  // Start with 1 Mbps
   // Increase only if needed and stable
   ```

3. **Always enable torque before commanding**:
   ```rust
   dxl.enable_torque(1, true);
   // Servo won't move without torque enabled
   ```

4. **Disable torque for safety when not in use**:
   ```rust
   dxl.enable_all_torque(false);
   // Servos go limp, safe to manually position
   ```

5. **Monitor temperature and errors**:
   ```rust
   if let Some(temp) = dxl.get_temperature(1) {
       if temp > 70 {
           // Reduce load or stop motion
       }
   }
   if dxl.has_error(1) {
       // Check and handle hardware error
   }
   ```

6. **Use bulk operations for synchronized motion**:
   ```rust
   // Use JointCommand for coordinated multi-servo moves
   let mut cmd = JointCommand::new();
   cmd.add_position("joint_1", 1.57)?;
   cmd.add_position("joint_2", 0.0)?;
   // Much faster than individual ServoCommand messages
   ```

7. **Clamp values to valid ranges**:
   ```rust
   let position = desired_position.clamp(-max_pos, max_pos);
   // Node automatically clamps but good practice
   ```

8. **Power servos separately from controller**:
   - Use dedicated power supply rated for total servo current draw
   - Connect power supply ground to U2D2 ground
   - Never power servos from USB

## Troubleshooting

### "Hardware unavailable - using SIMULATION mode"

```
[WARN] DynamixelNode: Hardware unavailable - using SIMULATION mode
[WARN]   Tried: /dev/ttyUSB0
[WARN]   Error: Permission denied
```

**Solutions:**
1. Check serial port exists:
   ```bash
   ls -l /dev/ttyUSB*
   ls -l /dev/ttyACM*
   ```

2. Add user to dialout group:
   ```bash
   sudo usermod -a -G dialout $USER
   # Logout and login again
   groups  # Verify 'dialout' is listed
   ```

3. Check U2D2/USB2Dynamixel connection:
   - USB cable properly connected
   - Power LED on U2D2 should be lit
   - Try different USB port

4. Verify servo power supply:
   - Check voltage (11-14.8V for most servos)
   - Check current capacity (2A+ per servo)
   - Ensure power supply ground connected to U2D2 ground

5. Rebuild with serial-hardware feature:
   ```bash
   cargo build --features="serial-hardware"
   ```

### Servo not responding (ping fails)

**Solutions:**
1. Check servo ID matches code:
   ```rust
   dxl.add_servo(1, DynamixelModel::XM430W350);  // Must match physical ID
   ```

2. Verify protocol version:
   ```rust
   // AX/MX servos use Protocol 1.0
   DynamixelProtocol::Protocol1

   // X/XL/XM/XH servos use Protocol 2.0
   DynamixelProtocol::Protocol2
   ```

3. Check baud rate matches servo:
   - Default: 57600 bps (many servos)
   - Default: 1000000 bps (X series)
   - Use Dynamixel Wizard to check/change baud rate

4. Test with Dynamixel Wizard:
   - Available from ROBOTIS website
   - Scan for servos
   - Verify connectivity and settings

5. Check wiring:
   - Data line properly connected
   - No reversed polarity on VDD/GND
   - Secure connections (no loose wires)

### Communication errors / packet loss

```
Comm: 850 ok, 150 err (85.0% success)
```

**Solutions:**
1. Reduce baud rate:
   ```rust
   dxl.set_baud_rate(1_000_000);  // Down from 3 Mbps or 4.5 Mbps
   ```

2. Check cable quality and length:
   - Use high-quality cables
   - Keep cables under 3m for high baud rates
   - Avoid running near noisy power cables

3. Reduce number of servos on bus:
   - Split into multiple buses if >10 servos
   - Use separate controllers for large systems

4. Add delay between commands:
   - Bulk operations are better than rapid individual commands
   - Let servos process commands before sending more

5. Check for electrical noise:
   - Add capacitors near servos (100-220uF)
   - Use shielded cables if in noisy environment
   - Separate power and data cables

### Servo overheating

```
[WARN] Servo 1 hot: 72°C
```

**Solutions:**
1. Reduce current limit:
   - Lower goal current for current control mode
   - Reduce load on servo

2. Improve cooling:
   - Add heatsinks to servo case
   - Add fan for active cooling
   - Ensure good airflow around servos

3. Reduce duty cycle:
   - Don't run at max speed continuously
   - Add rest periods between moves

4. Check for mechanical binding:
   - Ensure smooth motion without obstacles
   - Check for misalignment or excessive friction
   - Reduce load on servo

5. Lower ambient temperature:
   - Operating temperature: 0-70°C
   - Optimal: 20-25°C

### Position oscillation / instability

**Symptoms**: Servo oscillates around target position

**Solutions:**
1. Reduce P gain (if adjustable):
   - Lower proportional gain reduces oscillation
   - May slow response time

2. Increase D gain (if adjustable):
   - Damping term reduces oscillation
   - Improves stability

3. Check for mechanical play:
   - Loose connections cause instability
   - Check horn attachment
   - Check gear backlash

4. Use profile control (X series):
   - Smooth acceleration/deceleration profiles
   - Prevents abrupt changes

5. Lower command rate:
   - Don't send commands faster than servo can process
   - Add small delays between commands

### Servo moves to wrong position

**Solutions:**
1. Check angle units (radians vs degrees):
   ```rust
   // CORRECT: Use radians
   ServoCommand::new(1, 1.57)  // 90 degrees

   // CORRECT: Use helper for degrees
   ServoCommand::from_degrees(1, 90.0)

   // INCORRECT: Don't mix units
   ServoCommand::new(1, 90.0)  // This is 90 radians (very wrong!)
   ```

2. Verify position limits:
   - Check model max position
   - AX series: 0-300 degrees (limited range)
   - MX/X series: 0-360 degrees (full rotation)

3. Check for multi-turn mode:
   - Extended position mode allows >360 degrees
   - Normal mode wraps at 360 degrees

4. Verify servo calibration:
   - Use Dynamixel Wizard to check center position
   - May need factory reset if misconfigured

### Servo LED blinking / error codes

**LED patterns indicate hardware errors**:

- **Blinking slowly**: Input voltage error (voltage too low or high)
- **Blinking fast**: Overheating error (>70-80°C)
- **Solid red**: Hardware error (encoder, motor, electrical shock)

**Solutions:**
1. Check power supply voltage
2. Allow servo to cool down
3. Check for mechanical binding
4. Use Dynamixel Wizard to read detailed error code
5. May require servo repair or replacement

## Simulation Mode

When hardware is unavailable, the node operates in simulation mode:

```
[INFO] DynamixelNode: Hardware unavailable - using SIMULATION mode
[DEBUG] Servo 1 (SIM): position 1.571 rad (90.0°)
```

Simulated behavior:
- Realistic motion dynamics (velocity, acceleration)
- Position tracking with P-controller
- Velocity ramping with acceleration limits
- Current draw proportional to load
- Temperature simulation (increases with load)
- Useful for testing control logic without hardware

Limitations:
- No real servo feedback
- No communication error testing
- No real hardware error detection

## See Also

- [ServoController](../servo_controller/) - Standard PWM servo control
- [StepperMotor](../stepper_motor/) - Stepper motor control
- [DcMotor](../dc_motor/) - DC motor control
- [BldcMotor](../bldc_motor/) - Brushless DC motor control
- [RoboclawMotor](../roboclaw_motor/) - Roboclaw motor controller
