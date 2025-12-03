# Force/Torque Sensor Node

6-axis force/torque sensor interface for robot manipulation, compliance control, and safety monitoring with support for ATI, Robotiq, OnRobot, and other industrial sensors.

## Overview

The Force/Torque Sensor Node provides 6-DOF (degrees of freedom) force and torque measurements from industrial F/T sensors commonly used in robotic manipulation, assembly, and contact-rich tasks. It supports multiple sensor models, automatic bias removal (taring), gravity compensation, filtering, and overload detection.

Supports ATI Industrial Automation (Novanta), Robotiq FT-300, OnRobot HEX-E, Weiss Robotics KMS, Optoforce, JR3, and generic strain gauge based 6-axis sensors.

Key features:
- 6-axis measurement (Fx, Fy, Fz, Tx, Ty, Tz)
- Multiple sensor models with predefined ranges
- Automatic bias removal (taring/zeroing)
- Tool mass and gravity compensation
- Temperature compensation
- Low-pass filtering for noise reduction
- Overload protection monitoring
- Configurable sampling rates (1 Hz to 10 kHz)
- Multiple communication interfaces (Ethernet, CANopen, Serial, USB, EtherCAT)
- Calibration matrix support

## Topics

### Publishers

| Topic | Type | Description |
|-------|------|-------------|
| `force_torque/wrench` | `WrenchStamped` | 6-axis force and torque measurements |
| `force_torque/calibration` | `CalibrationStatus` | Sensor calibration and status information |

## Configuration Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `device_address` | `String` | Required | IP address, serial port, or device identifier |
| `sensor_model` | `SensorModel` | `Generic` | Sensor model (sets predefined ranges) |
| `connection_type` | `ConnectionType` | `Ethernet` | Communication interface type |
| `sample_rate` | `u32` | `1000` | Sampling frequency in Hz (1-10000) |
| `frame_id` | `String` | `"ft_sensor"` | Frame ID for measurements |
| `force_range` | `[f64; 3]` | `[100, 100, 100]` | Maximum force per axis in Newtons |
| `torque_range` | `[f64; 3]` | `[10, 10, 10]` | Maximum torque per axis in Nm |
| `use_lowpass_filter` | `bool` | `true` | Enable low-pass filtering |
| `filter_cutoff_hz` | `f64` | `100.0` | Filter cutoff frequency in Hz |
| `overload_threshold` | `f64` | `0.95` | Overload threshold (0.5-1.0 fraction of max) |
| `temperature_compensation_enabled` | `bool` | `true` | Enable temperature compensation |
| `tool_mass` | `f64` | `0.0` | Tool mass for gravity compensation (kg) |
| `tool_com` | `[f64; 3]` | `[0, 0, 0]` | Tool center of mass offset in meters |
| `gravity_vector` | `[f64; 3]` | `[0, 0, -9.81]` | Gravity direction in sensor frame |

### Sensor Models

Predefined sensor models with factory ranges:

| Model | Force Range (N) | Torque Range (Nm) | Application |
|-------|-----------------|-------------------|-------------|
| `ATI_Nano17` | 12 / 12 / 17 | 0.12 / 0.12 / 0.12 | Precision assembly |
| `ATI_Mini40` | 240 / 240 / 240 | 6 / 6 / 6 | Small robots, UR arms |
| `ATI_Mini45` | 290 / 290 / 580 | 10 / 10 / 10 | Collaborative robots |
| `ATI_Gamma` | 660 / 660 / 1980 | 60 / 60 / 60 | Industrial cobots |
| `ATI_Delta` | 3300 / 3300 / 9900 | 330 / 330 / 330 | Industrial robots |
| `ATI_Theta` | 6600 / 6600 / 19800 | 660 / 660 / 660 | Heavy-duty industrial |
| `Robotiq_FT300` | 300 / 300 / 500 | 30 / 30 / 30 | UR/cobot applications |
| `OnRobot_HexE` | 400 / 400 / 650 | 20 / 20 / 20 | Collaborative robots |
| `Weiss_KMS40` | 400 / 400 / 1000 | 15 / 15 / 15 | Assembly tasks |
| `Optoforce_HEX` | 200 / 200 / 500 | 4 / 4 / 4 | Optical force sensing |
| `Generic` | User-defined | User-defined | Custom sensors |

### Connection Types

| Type | Description |
|------|-------------|
| `Ethernet` | TCP/IP network communication (most common) |
| `EtherCAT` | Real-time industrial Ethernet protocol |
| `CANopen` | Industrial fieldbus protocol |
| `Serial` | RS-232/RS-485 serial communication |
| `USB` | Direct USB connection |

## Message Types

### WrenchStamped

6-axis force and torque measurement:

```rust
pub struct WrenchStamped {
    pub force: Vector3,                 // Force vector [fx, fy, fz] in Newtons
    pub torque: Vector3,                // Torque vector [tx, ty, tz] in Newton-meters
    pub point_of_application: Point3,   // Point of application relative to sensor frame
    pub frame_id: [u8; 32],            // Frame ID for measurement reference
    pub timestamp: u64,                 // Measurement time (ns since epoch)
}
```

**Helper methods**:
- `WrenchStamped::new(force, torque)` - Create new wrench
- `WrenchStamped::force_only(force)` - Force-only measurement
- `WrenchStamped::torque_only(torque)` - Torque-only measurement
- `with_frame_id(frame_id)` - Set frame ID
- `force_magnitude()` - Get force magnitude
- `torque_magnitude()` - Get torque magnitude
- `exceeds_limits(max_force, max_torque)` - Check safety limits
- `filter(prev_wrench, alpha)` - Apply low-pass filter

### CalibrationStatus

Sensor calibration and status information:

```rust
pub struct CalibrationStatus {
    pub is_calibrated: bool,        // Sensor calibrated
    pub is_zeroed: bool,            // Sensor tared/zeroed
    pub temperature: f64,           // Sensor temperature in Celsius
    pub overload_detected: bool,    // Overload condition detected
    pub overload_count: u32,        // Number of overload events
    pub measurement_count: u64,     // Total measurements taken
    pub max_force: f64,             // Maximum force recorded (N)
    pub max_torque: f64,            // Maximum torque recorded (Nm)
    pub timestamp: u64,             // Status timestamp (ns since epoch)
}
```

### Vector3

3D vector for force/torque components:

```rust
pub struct Vector3 {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}
```

## Public API

### Construction

```rust
use horus_library::nodes::ForceTorqueSensorNode;

// Create with device address (IP or serial port)
let mut ft_sensor = ForceTorqueSensorNode::new("192.168.1.100")?;

// Create with serial port
let mut ft_sensor = ForceTorqueSensorNode::new("/dev/ttyUSB0")?;
```

### Configuration Methods

```rust
// Set sensor model (applies predefined ranges)
ft_sensor.set_sensor_model(SensorModel::ATI_Mini40);

// Set connection type
ft_sensor.set_connection_type(ConnectionType::Ethernet);

// Set sample rate in Hz (1-10000)
ft_sensor.set_sample_rate(1000);  // 1 kHz

// Set frame ID for measurements
ft_sensor.set_frame_id("gripper_ft_sensor");

// Set custom force and torque ranges
ft_sensor.set_ranges(
    [250.0, 250.0, 500.0],  // Force range [Fx, Fy, Fz] in N
    [10.0, 10.0, 20.0]      // Torque range [Tx, Ty, Tz] in Nm
);

// Zero the sensor (tare) - removes current bias
ft_sensor.zero_sensor();

// Reset bias to zero
ft_sensor.reset_bias();

// Set tool mass and center of mass for gravity compensation
ft_sensor.set_tool_mass(
    0.5,    // mass in kg
    0.0,    // COM x-offset in meters
    0.0,    // COM y-offset in meters
    0.05    // COM z-offset in meters (5cm from sensor)
);

// Set gravity vector (for non-standard orientations)
ft_sensor.set_gravity_vector(0.0, 0.0, -9.81);  // Z-up frame

// Enable/disable low-pass filter
ft_sensor.set_lowpass_filter(true, 100.0);  // enable, 100 Hz cutoff

// Set overload threshold (0.5-1.0 fraction of max range)
ft_sensor.set_overload_threshold(0.9);  // 90% of max range

// Set calibration matrix (6x6 matrix from factory calibration)
let cal_matrix: [[f64; 6]; 6] = load_calibration_from_file();
ft_sensor.set_calibration_matrix(cal_matrix);
```

### Query Methods

```rust
// Get current force reading (compensated and filtered)
let force = ft_sensor.get_force();  // [Fx, Fy, Fz]
println!("Force: [{:.2}, {:.2}, {:.2}] N", force[0], force[1], force[2]);

// Get current torque reading (compensated and filtered)
let torque = ft_sensor.get_torque();  // [Tx, Ty, Tz]
println!("Torque: [{:.2}, {:.2}, {:.2}] Nm", torque[0], torque[1], torque[2]);

// Check if sensor is overloaded
if ft_sensor.is_overloaded() {
    println!("WARNING: Sensor overload detected!");
}
```

### Preset Configurations

```rust
// Configure for ATI Mini40 (common on UR robots)
ft_sensor.configure_ati_mini40();
// Sets: ATI_Mini40 model, 1kHz sample rate, 100Hz filter

// Configure for collaborative robot applications
ft_sensor.configure_cobot();
// Sets: Robotiq_FT300 model, 500Hz sample rate, 50Hz filter, 80% threshold

// Configure for precision assembly
ft_sensor.configure_precision();
// Sets: ATI_Nano17 model, 2kHz sample rate, 200Hz filter

// Configure for heavy-duty industrial use
ft_sensor.configure_heavy_duty();
// Sets: ATI_Theta model, 1kHz sample rate, 100Hz filter, 95% threshold
```

## Usage Examples

### Basic Force/Torque Monitoring

```rust
use horus_library::prelude::*;

fn main() -> Result<()> {
    let mut scheduler = Scheduler::new();

    // Create and configure F/T sensor
    let mut ft_sensor = ForceTorqueSensorNode::new("192.168.1.100")?;
    ft_sensor.set_sensor_model(SensorModel::ATI_Mini40);
    ft_sensor.set_sample_rate(1000);  // 1 kHz
    ft_sensor.zero_sensor();  // Tare the sensor

    scheduler.add(Box::new(ft_sensor), 1, Some(true));
    scheduler.run()?;
    Ok(())
}
```

### Contact Detection

```rust
use horus_library::prelude::*;

fn main() -> Result<()> {
    let mut scheduler = Scheduler::new();

    let mut ft_sensor = ForceTorqueSensorNode::new("192.168.1.100")?;
    ft_sensor.configure_cobot();
    ft_sensor.zero_sensor();

    scheduler.add(Box::new(ft_sensor), 1, Some(true));

    // Contact detection node
    let detector = node! {
        name: "contact_detector",
        tick: |ctx| {
            let hub = Hub::<WrenchStamped>::new("force_torque.wrench")?;

            while let Some(wrench) = hub.recv(None) {
                let force_mag = wrench.force_magnitude();
                let torque_mag = wrench.torque_magnitude();

                if force_mag > 5.0 {
                    ctx.log_warning(&format!(
                        "Contact detected! Force: {:.1}N, Torque: {:.1}Nm",
                        force_mag, torque_mag
                    ));
                } else {
                    ctx.log_info(&format!(
                        "No contact - Force: {:.2}N",
                        force_mag
                    ));
                }
            }
            Ok(())
        }
    };

    scheduler.add(Box::new(detector), 2, Some(true));
    scheduler.run()?;
    Ok(())
}
```

### Gravity Compensation for Tool

```rust
use horus_library::prelude::*;

fn main() -> Result<()> {
    let mut scheduler = Scheduler::new();

    let mut ft_sensor = ForceTorqueSensorNode::new("192.168.1.100")?;
    ft_sensor.set_sensor_model(SensorModel::ATI_Mini40);
    ft_sensor.set_sample_rate(1000);

    // Configure tool compensation for a 0.5kg gripper
    // with center of mass 5cm below the sensor
    ft_sensor.set_tool_mass(0.5, 0.0, 0.0, 0.05);
    ft_sensor.set_gravity_vector(0.0, 0.0, -9.81);

    // Zero sensor after mounting tool
    ft_sensor.zero_sensor();

    scheduler.add(Box::new(ft_sensor), 1, Some(true));
    scheduler.run()?;
    Ok(())
}
```

### Force-Controlled Insertion

```rust
use horus_library::prelude::*;

fn main() -> Result<()> {
    let mut scheduler = Scheduler::new();

    let mut ft_sensor = ForceTorqueSensorNode::new("192.168.1.100")?;
    ft_sensor.configure_precision();  // Use high-precision sensor
    ft_sensor.zero_sensor();

    scheduler.add(Box::new(ft_sensor), 1, Some(true));

    // Insertion control node
    let insertion = node! {
        name: "insertion_controller",
        tick: |ctx| {
            let hub = Hub::<WrenchStamped>::new("force_torque.wrench")?;

            while let Some(wrench) = hub.recv(None) {
                let z_force = wrench.force.z;

                // Monitor insertion force
                if z_force > 2.0 && z_force < 5.0 {
                    ctx.log_info("Normal insertion force");
                } else if z_force > 5.0 {
                    ctx.log_error("Excessive force - stopping!");
                    // Emergency stop logic here
                } else if z_force > 10.0 {
                    ctx.log_error("CRITICAL: Force overload!");
                    // Hardware emergency stop
                }

                // Check for misalignment via torque
                let torque_mag = wrench.torque_magnitude();
                if torque_mag > 0.5 {
                    ctx.log_warning("High torque - possible misalignment");
                }
            }
            Ok(())
        }
    };

    scheduler.add(Box::new(insertion), 2, Some(true));
    scheduler.run()?;
    Ok(())
}
```

### Multi-Sensor Safety Monitor

```rust
use horus_library::prelude::*;

fn main() -> Result<()> {
    let mut scheduler = Scheduler::new();

    let mut ft_sensor = ForceTorqueSensorNode::new("192.168.1.100")?;
    ft_sensor.configure_cobot();
    ft_sensor.set_overload_threshold(0.80);  // Lower threshold for safety
    ft_sensor.zero_sensor();

    scheduler.add(Box::new(ft_sensor), 1, Some(true));

    // Safety monitor
    let safety = node! {
        name: "safety_monitor",
        tick: |ctx| {
            let wrench_hub = Hub::<WrenchStamped>::new("force_torque.wrench")?;
            let status_hub = Hub::<CalibrationStatus>::new("force_torque.calibration")?;

            // Check wrench data
            while let Some(wrench) = wrench_hub.recv(None) {
                if wrench.exceeds_limits(100.0, 10.0) {
                    ctx.log_error("SAFETY LIMIT EXCEEDED!");
                }
            }

            // Check sensor status
            while let Some(status) = status_hub.recv(None) {
                if status.overload_detected {
                    ctx.log_error(&format!(
                        "Sensor overload detected! Count: {}",
                        status.overload_count
                    ));
                }

                if !status.is_zeroed {
                    ctx.log_warning("Sensor not zeroed - readings may include bias");
                }

                // Periodic status report
                if status.measurement_count % 10000 == 0 {
                    ctx.log_info(&format!(
                        "Status: Cal={} Zero={} T={:.1}C MaxF={:.1}N MaxT={:.1}Nm",
                        status.is_calibrated,
                        status.is_zeroed,
                        status.temperature,
                        status.max_force,
                        status.max_torque
                    ));
                }
            }
            Ok(())
        }
    };

    scheduler.add(Box::new(safety), 2, Some(true));
    scheduler.run()?;
    Ok(())
}
```

### Surface Following with Force Control

```rust
use horus_library::prelude::*;

fn main() -> Result<()> {
    let mut scheduler = Scheduler::new();

    let mut ft_sensor = ForceTorqueSensorNode::new("192.168.1.100")?;
    ft_sensor.set_sensor_model(SensorModel::ATI_Gamma);
    ft_sensor.set_sample_rate(500);
    ft_sensor.set_lowpass_filter(true, 50.0);  // Lower cutoff for smooth control
    ft_sensor.zero_sensor();

    scheduler.add(Box::new(ft_sensor), 1, Some(true));

    // Surface following controller
    let controller = node! {
        name: "surface_follower",
        tick: |ctx| {
            let hub = Hub::<WrenchStamped>::new("force_torque.wrench")?;

            while let Some(wrench) = hub.recv(None) {
                let normal_force = wrench.force.z;  // Assuming Z is surface normal
                let target_force = 10.0;  // 10N contact force

                let error = target_force - normal_force;

                if error.abs() < 1.0 {
                    ctx.log_info(&format!("Good contact: {:.1}N", normal_force));
                } else if error > 0.0 {
                    ctx.log_info(&format!("Too light: {:.1}N (move closer)", normal_force));
                } else {
                    ctx.log_warning(&format!("Too hard: {:.1}N (back off)", normal_force));
                }

                // Check for lateral forces (sliding/misalignment)
                let lateral_force = (wrench.force.x.powi(2) + wrench.force.y.powi(2)).sqrt();
                if lateral_force > 5.0 {
                    ctx.log_warning(&format!("High lateral force: {:.1}N", lateral_force));
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

## Hardware Setup

### ATI Industrial Automation Sensors

#### Network Configuration

```bash
# Configure sensor IP address (usually 192.168.1.1)
# Set your PC to same subnet: 192.168.1.100

# Test connection
ping 192.168.1.1

# Verify sensor responds on HTTP interface
curl http://192.168.1.1
```

#### Mounting

1. Mount sensor between robot flange and tool
2. Ensure proper alignment (X-forward, Y-left, Z-up typical)
3. Torque mounting bolts to specification (typically 4-8 Nm)
4. Connect Ethernet cable with strain relief
5. Connect power if required (some sensors are passive)

#### Calibration

ATI sensors include factory calibration files:

```rust
// Load calibration matrix from sensor's EEPROM or file
let cal_matrix = load_ati_calibration("FT12345.cal")?;
ft_sensor.set_calibration_matrix(cal_matrix);
```

### Robotiq FT-300

```rust
// Configure for Robotiq FT-300 on UR robot
let mut ft_sensor = ForceTorqueSensorNode::new("192.168.1.11")?;
ft_sensor.set_sensor_model(SensorModel::Robotiq_FT300);
ft_sensor.set_connection_type(ConnectionType::Ethernet);
ft_sensor.set_sample_rate(500);
```

### OnRobot HEX-E

```rust
// Configure for OnRobot HEX-E
let mut ft_sensor = ForceTorqueSensorNode::new("192.168.1.12")?;
ft_sensor.set_sensor_model(SensorModel::OnRobot_HexE);
ft_sensor.set_connection_type(ConnectionType::Ethernet);
```

### Serial Connection

```rust
// Configure for serial-connected sensor
let mut ft_sensor = ForceTorqueSensorNode::new("/dev/ttyUSB0")?;
ft_sensor.set_connection_type(ConnectionType::Serial);
ft_sensor.set_sample_rate(100);  // Lower rate for serial
```

## Coordinate Frames

### Sensor Frame Convention

Standard right-hand coordinate system:

```
      Z (up/normal)
      |
      |
      +---- Y (left)
     /
    /
   X (forward)
```

Force components:
- Fx: Force along X-axis (forward/backward)
- Fy: Force along Y-axis (left/right)
- Fz: Force along Z-axis (up/down, typically normal to mounting surface)

Torque components:
- Tx: Torque about X-axis (pitch)
- Ty: Torque about Y-axis (roll)
- Tz: Torque about Z-axis (yaw)

### Frame Transformations

When sensor is mounted at non-standard orientation:

```rust
// Sensor mounted with X pointing down (90-degree rotation)
ft_sensor.set_gravity_vector(-9.81, 0.0, 0.0);  // Gravity now along -X
```

## Calibration and Zeroing

### Bias Removal (Taring)

Remove constant offsets from measurements:

```rust
// Zero sensor to remove current readings as bias
ft_sensor.zero_sensor();

// Recommended: Zero with robot in representative pose
// and tool attached but not in contact
```

### When to Zero

1. After sensor power-up or reset
2. After mounting/unmounting tools
3. When changing robot orientation significantly
4. Periodically during long operations (sensor drift)

### Factory Calibration

ATI and other manufacturers provide calibration matrices:

```rust
// Apply factory calibration matrix
let cal_matrix = [
    [1.00123, 0.00045, -0.00012, 0.00234, 0.00456, -0.00089],
    // ... 5 more rows
];
ft_sensor.set_calibration_matrix(cal_matrix);
```

## Gravity Compensation

Compensate for tool weight to measure only interaction forces:

```rust
// Measure tool weight first
// 1. Mount tool on sensor
// 2. Read force with robot static: tool_weight = Fz
// 3. Measure tool COM offset from sensor center

// Configure compensation
ft_sensor.set_tool_mass(0.8, 0.02, 0.01, 0.06);  // 800g tool, 2cm X, 1cm Y, 6cm Z

// Gravity compensation is applied automatically
// Readings will now show only interaction forces
```

### Multi-Orientation Compensation

For robots that move through different orientations:

```rust
// Update gravity vector based on robot orientation
// (requires robot pose feedback)

let robot_rotation = get_robot_orientation();
let gravity_sensor_frame = robot_rotation.transform_vector([0.0, 0.0, -9.81]);
ft_sensor.set_gravity_vector(
    gravity_sensor_frame[0],
    gravity_sensor_frame[1],
    gravity_sensor_frame[2]
);
```

## Filtering and Signal Processing

### Low-Pass Filter

Exponential moving average filter for noise reduction:

```rust
// Enable filter with cutoff frequency
ft_sensor.set_lowpass_filter(true, 100.0);  // 100 Hz cutoff

// Filter characteristics:
// - Cutoff frequency: frequency at -3dB attenuation
// - Higher cutoff = less filtering, faster response
// - Lower cutoff = more filtering, slower response

// Typical settings:
ft_sensor.set_lowpass_filter(true, 200.0);  // Fast response (high-speed assembly)
ft_sensor.set_lowpass_filter(true, 50.0);   // Smooth control (surface following)
ft_sensor.set_lowpass_filter(true, 10.0);   // Heavy filtering (noisy environment)
```

### Filter Parameter Selection

```
Sample Rate = 1000 Hz
Cutoff = 100 Hz
=> Filter alpha = 0.386
=> Time constant = 1.6 ms
=> Settling time ~ 8 ms (5 tau)
```

Guidelines:
- Cutoff should be 2-5x lower than sample rate
- Higher sample rate allows higher cutoff
- Consider control loop bandwidth

## Overload Protection

### Threshold Detection

```rust
// Set threshold as fraction of maximum range
ft_sensor.set_overload_threshold(0.90);  // Warn at 90% of max

// Sensor will log warnings when exceeded
// Check programmatically:
if ft_sensor.is_overloaded() {
    emergency_stop();
}
```

### Monitoring Overload Events

```rust
let status_hub = Hub::<CalibrationStatus>::new("force_torque.calibration")?;

while let Some(status) = status_hub.recv(None) {
    if status.overload_detected {
        println!("Overload! Count: {}", status.overload_count);
        // Take corrective action
    }
}
```

### Safety Limits

```rust
// Check against absolute limits
let wrench_hub = Hub::<WrenchStamped>::new("force_torque.wrench")?;

while let Some(wrench) = wrench_hub.recv(None) {
    if wrench.exceeds_limits(100.0, 10.0) {  // 100N force, 10Nm torque
        emergency_stop();
    }
}
```

## Best Practices

1. **Always zero sensor before operation**:
   ```rust
   ft_sensor.zero_sensor();  // Tare with robot static and tool mounted
   ```

2. **Use gravity compensation for accuracy**:
   ```rust
   // Measure and configure tool mass
   ft_sensor.set_tool_mass(0.5, 0.0, 0.0, 0.05);
   ```

3. **Apply appropriate filtering**:
   ```rust
   // Balance noise reduction vs. response time
   ft_sensor.set_lowpass_filter(true, 100.0);  // 100 Hz for most applications
   ```

4. **Set conservative overload thresholds**:
   ```rust
   ft_sensor.set_overload_threshold(0.80);  // 80% for safety-critical apps
   ```

5. **Monitor calibration status**:
   ```rust
   // Subscribe to calibration topic for continuous monitoring
   let status_hub = Hub::<CalibrationStatus>::new("force_torque.calibration")?;
   ```

6. **Handle sensor failures gracefully**:
   ```rust
   // Check for valid measurements
   if wrench.force_magnitude() < 0.0 || wrench.force_magnitude() > max_expected {
       log_error("Sensor reading out of expected range");
   }
   ```

7. **Protect sensor from impacts**:
   - Use overload stops on robot
   - Implement software force limits
   - Add mechanical protection if needed

8. **Regular recalibration**:
   - Re-zero periodically (every few hours)
   - Annual factory recalibration for precision work
   - Check calibration after impacts

## Troubleshooting

### No sensor data / Connection failed

```
[ERROR] Failed to connect to F/T sensor at 192.168.1.100
```

**Solutions:**
1. Check network connection
   ```bash
   ping 192.168.1.100
   ```
2. Verify sensor IP address in sensor documentation
3. Check firewall settings
4. Ensure sensor is powered on
5. Try different connection type (Ethernet vs Serial)
6. Check cable connections

### Readings show constant offset

```
[INFO] Force: [0.0, 0.0, -4.9]N (expected ~0)
```

**Solutions:**
1. Zero the sensor:
   ```rust
   ft_sensor.zero_sensor();
   ```
2. Configure gravity compensation:
   ```rust
   ft_sensor.set_tool_mass(0.5, 0.0, 0.0, 0.05);
   ```
3. Check sensor mounting orientation
4. Verify gravity vector setting

### Noisy readings / Erratic data

```
[WARN] Force jumping between 5.0N and 15.0N
```

**Solutions:**
1. Enable or increase filtering:
   ```rust
   ft_sensor.set_lowpass_filter(true, 50.0);  // Lower cutoff = more filtering
   ```
2. Check electrical connections
3. Look for electromagnetic interference sources
4. Reduce sample rate if communication is unreliable
5. Check sensor grounding
6. Verify cable shielding

### Overload warnings during normal operation

```
[WARN] F/T Sensor OVERLOAD: F=[245, 12, 38]N
```

**Solutions:**
1. Increase overload threshold:
   ```rust
   ft_sensor.set_overload_threshold(0.95);  // 95% instead of 90%
   ```
2. Check if sensor model/ranges are correct:
   ```rust
   ft_sensor.set_sensor_model(SensorModel::ATI_Gamma);  // Higher capacity
   ```
3. Verify actual forces aren't exceeding sensor capacity
4. Consider using higher-capacity sensor model

### Gravity compensation not working

```
[INFO] Expected 0N, reading -4.9N vertical
```

**Solutions:**
1. Verify gravity vector matches sensor orientation:
   ```rust
   ft_sensor.set_gravity_vector(0.0, 0.0, -9.81);  // Z-up
   ```
2. Check tool mass and COM:
   ```rust
   // Measure actual tool weight
   let measured_weight = 0.5;  // kg
   ft_sensor.set_tool_mass(measured_weight, 0.0, 0.0, 0.05);
   ```
3. Ensure sensor is zeroed before setting compensation
4. Update gravity vector if robot orientation changes

### Temperature drift

```
[WARN] Readings drifting over time
```

**Solutions:**
1. Allow sensor warm-up period (15-30 minutes)
2. Enable temperature compensation:
   ```rust
   ft_sensor.temperature_compensation_enabled = true;
   ```
3. Re-zero sensor after warm-up
4. Control ambient temperature if possible
5. Consider sensor with better thermal stability

### Communication timeout / Slow response

```
[ERROR] Timeout waiting for sensor data
```

**Solutions:**
1. Reduce sample rate:
   ```rust
   ft_sensor.set_sample_rate(500);  // Lower from 1000 Hz
   ```
2. Check network latency
3. Use wired Ethernet instead of WiFi
4. Verify sensor isn't overloaded with requests
5. Check for network congestion

## Simulation Mode

When hardware is unavailable, the node operates in simulation mode:

```
[INFO] F/T Sensor: F=[0.02, -0.03, 0.01]N T=[0.001, -0.002, 0.000]Nm (SIM)
```

Simulated behavior:
- Returns small random forces/torques (noise simulation)
- Includes gravity effects if tool mass is configured
- Applies filtering as configured
- Useful for:
  - Algorithm development without hardware
  - Testing safety logic
  - Integration testing
  - Student learning

Enable simulation explicitly:
```rust
// Simulation happens automatically when hardware connection fails
// No special configuration needed
```

## Performance Considerations

### Sample Rate Selection

| Application | Recommended Rate | Notes |
|-------------|------------------|-------|
| Static force measurement | 10-100 Hz | Low rate sufficient |
| Contact detection | 100-500 Hz | Balance response/bandwidth |
| Force control | 500-1000 Hz | Match control loop rate |
| Impact detection | 1000-7000 Hz | High rate for fast transients |
| Assembly tasks | 250-500 Hz | Typical range |

### Computational Load

- Raw measurement processing: <0.1 ms per sample
- Gravity compensation: ~0.05 ms per sample
- Filtering: <0.01 ms per sample
- Total overhead: Negligible for rates <10 kHz

### Network Bandwidth

Ethernet bandwidth usage:
- 1000 Hz @ 48 bytes per sample = 48 KB/s
- Negligible compared to 100 Mbps Ethernet
- EtherCAT provides deterministic real-time performance

## Advanced Features

### Custom Calibration Matrix

Load sensor-specific calibration:

```rust
// 6x6 calibration matrix from factory calibration file
let cal_matrix: [[f64; 6]; 6] = [
    [1.00123, 0.00045, -0.00012, 0.00234, 0.00456, -0.00089],
    [0.00034, 0.99987, 0.00056, -0.00123, 0.00234, 0.00167],
    [-0.00023, 0.00012, 1.00045, 0.00345, -0.00234, 0.00123],
    [0.00145, -0.00078, 0.00234, 0.99956, 0.00123, -0.00056],
    [0.00089, 0.00156, -0.00167, 0.00078, 1.00012, 0.00234],
    [-0.00067, 0.00234, 0.00123, -0.00045, 0.00089, 0.99978],
];

ft_sensor.set_calibration_matrix(cal_matrix);
```

### Multi-Sensor Systems

Multiple F/T sensors on one robot:

```rust
// Wrist sensor
let mut wrist_ft = ForceTorqueSensorNode::new("192.168.1.100")?;
wrist_ft.set_sensor_model(SensorModel::ATI_Mini40);
wrist_ft.set_frame_id("wrist_ft");

// Gripper sensor
let mut gripper_ft = ForceTorqueSensorNode::new("192.168.1.101")?;
gripper_ft.set_sensor_model(SensorModel::ATI_Nano17);
gripper_ft.set_frame_id("gripper_ft");

// Different topics automatically via frame_id
// Subscribe to specific sensors:
let wrist_hub = Hub::<WrenchStamped>::new("force_torque.wrench")?;
// Filter by frame_id in your node
```

## See Also

- [DynamixelNode](../dynamixel/) - Robot arm servos for manipulation
- [GripperNode](../gripper/) - Gripper control for grasping
- [IMUNode](../imu/) - Inertial measurements for gravity vector
- [SafetyMonitorNode](../safety_monitor/) - Safety systems integration
- [JointStateNode](../joint_state/) - Robot joint positions
