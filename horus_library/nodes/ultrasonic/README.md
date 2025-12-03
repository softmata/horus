# Ultrasonic Distance Sensor Node

HC-SR04 ultrasonic distance sensor for obstacle detection, proximity sensing, and rangefinding with multi-sensor support.

## Quick Start

```rust
use horus_library::nodes::UltrasonicNode;
use horus_library::Range;
use horus_core::{Scheduler, Hub};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut scheduler = Scheduler::new();

    // Create ultrasonic sensor node
    let mut ultrasonic = UltrasonicNode::new()?;
    ultrasonic.set_num_sensors(1);
    ultrasonic.set_gpio_pins(0, 23, 24);     // Sensor 0: Trigger=GPIO23, Echo=GPIO24
    ultrasonic.set_measurement_rate(10.0);   // 10 Hz
    ultrasonic.enable_median_filter(true);   // Reduce noise

    scheduler.add(Box::new(ultrasonic), 50, Some(true));
    scheduler.run()?;
    Ok(())
}

// Subscribe to range data in another node:
let range_hub = Hub::<Range>::new("ultrasonic.range")?;
if let Some(range) = range_hub.recv_latest() {
    if range.range > 0.0 && range.range < range.max_range {
        println!("Distance: {:.2} m", range.range);
    } else {
        println!("Out of range");
    }
}
```

**Publishes to:** `ultrasonic.range` topic with distance measurements.

**Hardware Note:** HC-SR04 Echo outputs 5V - use a voltage divider for 3.3V GPIO!

## Overview

The Ultrasonic Node provides distance measurements using ultrasonic sensors via echo/trigger interface. It supports multiple sensors simultaneously for 360-degree coverage or sensor arrays, with median filtering, temperature compensation, and automatic hardware/simulation fallback.

Supports HC-SR04, HC-SR04+, US-100, JSN-SR04T, Maxbotix MB1xxx series, and DYP-A01 sensors.

Key features:
- Multi-sensor support (up to 16 sensors)
- Range: 2cm to 4m (sensor dependent)
- Configurable measurement rate (1-100 Hz)
- Median filtering for noise reduction
- Temperature compensation for accuracy
- Simulation fallback when hardware unavailable
- Hardware GPIO control via sysfs

## Topics

### Publishers

| Topic | Type | Description |
|-------|------|-------------|
| `ultrasonic.range` | `Range` | Distance measurements from ultrasonic sensors |

## Configuration Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `num_sensors` | `u8` | `1` | Number of ultrasonic sensors (1-16) |
| `measurement_rate` | `f32` | `10.0` | Measurement frequency in Hz (1-100) |
| `temperature_celsius` | `f32` | `20.0` | Ambient temperature for speed of sound compensation |
| `min_range` | `f32` | `0.02` | Minimum valid range in meters (per sensor) |
| `max_range` | `f32` | `4.0` | Maximum valid range in meters (per sensor) |
| `field_of_view` | `f32` | `0.26` | Beam angle in radians (~15 degrees) |
| `enable_median_filter` | `bool` | `true` | Enable median filtering for noise reduction |
| `median_filter_size` | `usize` | `5` | Median filter window size (3, 5, or 7) |
| `trigger_duration_ns` | `u64` | `10000` | Trigger pulse duration in nanoseconds (10μs) |
| `max_echo_time_ns` | `u64` | `38000000` | Maximum echo wait time in nanoseconds (38ms) |

### Hardware GPIO Pins

Configure GPIO pins for each sensor (Raspberry Pi BCM numbering):

```rust
ultrasonic.set_gpio_pins(sensor_id, trigger_pin, echo_pin);
```

Example pin mapping:
- Sensor 0: GPIO 23 (trigger), GPIO 24 (echo)
- Sensor 1: GPIO 25 (trigger), GPIO 8 (echo)
- Sensor 2: GPIO 7 (trigger), GPIO 1 (echo)
- Sensor 3: GPIO 12 (trigger), GPIO 16 (echo)

**IMPORTANT**: HC-SR04 Echo pin outputs 5V but Raspberry Pi GPIOs are 3.3V tolerant. Use a voltage divider:

```
Echo (5V) ---[R1: 1kΩ]---+--- GPIO (3.3V)
                         |
                      [R2: 2kΩ]
                         |
                        GND
```

## Message Types

### Range

Distance measurement message:

```rust
pub struct Range {
    pub sensor_type: u8,      // Sensor type (0=ultrasonic, 1=infrared)
    pub field_of_view: f32,   // Beam angle in radians
    pub min_range: f32,       // Minimum range in meters
    pub max_range: f32,       // Maximum range in meters
    pub range: f32,           // Distance reading in meters
    pub timestamp: u64,       // Measurement time (ns since epoch)
}
```

**Constants**:
- `Range::ULTRASONIC = 0`
- `Range::INFRARED = 1`

## Public API

### Construction

```rust
use horus_library::nodes::UltrasonicNode;

// Create with default topic "ultrasonic.range"
let mut ultrasonic = UltrasonicNode::new()?;

// Create with custom topic
let mut ultrasonic = UltrasonicNode::new_with_topic("sensors.distance")?;
```

### Configuration Methods

```rust
// Set number of sensors (1-16)
ultrasonic.set_num_sensors(4);

// Set sensor name for identification
ultrasonic.set_sensor_name(0, "front_sensor");

// Configure GPIO pins for a sensor
ultrasonic.set_gpio_pins(0, 23, 24);  // sensor_id, trigger_pin, echo_pin

// Set measurement rate in Hz (1-100)
ultrasonic.set_measurement_rate(20.0);  // 20 measurements per second

// Set ambient temperature for speed of sound compensation
ultrasonic.set_temperature(25.0);  // 25°C

// Set range limits for a sensor
ultrasonic.set_range_limits(0, 0.02, 4.0);  // sensor_id, min_m, max_m

// Set field of view (beam angle) in degrees
ultrasonic.set_field_of_view_degrees(0, 15.0);

// Enable/disable median filtering
ultrasonic.enable_median_filter(true);

// Set median filter size (3, 5, or 7 samples)
ultrasonic.set_median_filter_size(5);
```

### Query Methods

```rust
// Get current range reading for a sensor
if let Some(distance) = ultrasonic.get_range(0) {
    println!("Distance: {:.3} m", distance);
}

// Get measurement statistics (total, errors, error_rate)
if let Some((total, errors, error_rate)) = ultrasonic.get_statistics(0) {
    println!("Total: {}, Errors: {}, Error Rate: {:.1}%", total, errors, error_rate);
}
```

## Usage Examples

### Single Sensor

```rust
use horus_library::prelude::*;

fn main() -> Result<()> {
    let mut scheduler = Scheduler::new();

    // Create ultrasonic node
    let mut ultrasonic = UltrasonicNode::new()?;

    // Configure sensor
    ultrasonic.set_num_sensors(1);
    ultrasonic.set_gpio_pins(0, 23, 24);  // sensor 0: trigger=GPIO23, echo=GPIO24
    ultrasonic.set_measurement_rate(10.0);  // 10 Hz
    ultrasonic.enable_median_filter(true);
    ultrasonic.set_median_filter_size(5);

    scheduler.add(Box::new(ultrasonic), 1, Some(true));
    scheduler.run()?;
    Ok(())
}
```

### Multiple Sensors (360° Coverage)

```rust
use horus_library::prelude::*;

fn main() -> Result<()> {
    let mut scheduler = Scheduler::new();

    // Configure 4 sensors around robot
    let mut ultrasonic = UltrasonicNode::new()?;
    ultrasonic.set_num_sensors(4);

    // Front, Right, Back, Left sensors
    ultrasonic.set_gpio_pins(0, 23, 24);   // Front
    ultrasonic.set_gpio_pins(1, 25, 8);    // Right
    ultrasonic.set_gpio_pins(2, 7, 1);     // Back
    ultrasonic.set_gpio_pins(3, 12, 16);   // Left

    ultrasonic.set_sensor_name(0, "front");
    ultrasonic.set_sensor_name(1, "right");
    ultrasonic.set_sensor_name(2, "back");
    ultrasonic.set_sensor_name(3, "left");

    ultrasonic.set_measurement_rate(10.0);
    ultrasonic.enable_median_filter(true);
    ultrasonic.set_median_filter_size(3);

    scheduler.add(Box::new(ultrasonic), 1, Some(true));
    scheduler.run()?;
    Ok(())
}
```

### Obstacle Detection

```rust
use horus_library::prelude::*;

fn main() -> Result<()> {
    let mut scheduler = Scheduler::new();

    let mut ultrasonic = UltrasonicNode::new()?;
    ultrasonic.set_num_sensors(1);
    ultrasonic.set_gpio_pins(0, 23, 24);
    ultrasonic.set_measurement_rate(10.0);

    scheduler.add(Box::new(ultrasonic), 1, Some(true));

    // Obstacle detection node
    let detector = node! {
        name: "obstacle_detector",
        tick: |ctx| {
            let hub = Hub::<Range>::new("ultrasonic.range")?;

            while let Some(range) = hub.recv(None) {
                if range.range >= range.min_range && range.range <= range.max_range {
                    if range.range < 0.3 {
                        ctx.log_warning(&format!("Obstacle detected at {:.2}m!", range.range));
                    } else {
                        ctx.log_info(&format!("Clear: {:.2}m", range.range));
                    }
                } else {
                    ctx.log_warning("Out of range");
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

### Temperature Compensation

```rust
use horus_library::prelude::*;

fn main() -> Result<()> {
    let mut scheduler = Scheduler::new();

    let mut ultrasonic = UltrasonicNode::new()?;
    ultrasonic.set_num_sensors(1);
    ultrasonic.set_gpio_pins(0, 23, 24);

    // Set temperature for accurate speed of sound
    // Speed of sound: 331.3 + 0.606 * temperature_celsius
    ultrasonic.set_temperature(25.0);  // 25°C = 346.45 m/s

    ultrasonic.set_measurement_rate(20.0);

    scheduler.add(Box::new(ultrasonic), 1, Some(true));
    scheduler.run()?;
    Ok(())
}
```

## Hardware Setup

### Wiring Diagram

```
Raspberry Pi          HC-SR04
GPIO 23 (output) ---> Trig
GPIO 24 (input)  <--- Echo (via voltage divider!)
5V               ---> VCC
GND              ---> GND
```

### Voltage Divider Circuit

```
HC-SR04 Echo Pin (5V)
         |
        [1kΩ resistor]
         |
         +----------> Raspberry Pi GPIO 24 (3.3V input)
         |
        [2kΩ resistor]
         |
        GND
```

This creates: 3.3V = 5V × (2kΩ / (1kΩ + 2kΩ))

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

## Timing and Accuracy

### Measurement Process

1. **Trigger**: Send 10μs HIGH pulse on trigger pin
2. **Echo wait**: Wait for echo pin to go HIGH (up to 38ms)
3. **Echo measurement**: Measure echo pin HIGH duration
4. **Distance calculation**: distance = (echo_duration × speed_of_sound) / 2

### Timing Constraints

```
Maximum range: 4m
Speed of sound: 343 m/s (at 20°C)
Echo time: (4m × 2) / 343 m/s = 23.3ms
Safe timeout: 38ms (includes margin)
```

### Accuracy

- **Resolution**: ~3mm (based on timing resolution)
- **Accuracy**: ±3cm typical
- **Affected by**:
  - Temperature (speed of sound changes)
  - Surface angle (best at 90°)
  - Surface material (soft materials absorb sound)
  - Air currents

### Speed of Sound vs Temperature

- 15°C: 340.0 m/s
- 20°C: 343.0 m/s
- 25°C: 346.5 m/s
- 30°C: 349.0 m/s

## Best Practices

1. **Use voltage divider on Echo pin**:
   Always protect 3.3V GPIO from 5V Echo signal

2. **Space sensors apart**:
   Minimum 10cm spacing recommended to avoid crosstalk

3. **Use filtering for noisy environments**:
   ```rust
   ultrasonic.enable_median_filter(true);
   ultrasonic.set_median_filter_size(5);
   ```

4. **Set appropriate measurement rate**:
   ```rust
   // Balance between update rate and max range
   ultrasonic.set_measurement_rate(10.0);  // 10Hz for 4m range
   ultrasonic.set_measurement_rate(20.0);  // 20Hz for 2m range
   ```

5. **Compensate for temperature**:
   ```rust
   // Read ambient temperature from IMU or separate sensor
   ultrasonic.set_temperature(measured_temp);
   ```

6. **Handle out-of-range measurements**:
   ```rust
   if range.range >= range.min_range && range.range <= range.max_range {
       // Valid measurement
   } else {
       // Out of range - object too close or too far
   }
   ```

## Troubleshooting

### "Hardware unavailable - using SIMULATION mode"

```
[WARN] UltrasonicNode sensor 0: Hardware unavailable - using SIMULATION mode
[WARN]   Tried GPIO pins: trigger=23, echo=24
[WARN]   Error: Permission denied
```

**Solutions:**
1. Check GPIO permissions
2. Verify wiring and voltage divider
3. Install libraspberrypi-dev
4. Enable GPIO in raspi-config
5. Rebuild with `--features="gpio-hardware"`

### Always reading maximum distance

**Solutions:**
1. Check VCC and GND connections
2. Verify voltage divider on Echo pin
3. Check for obstacles in range
4. Ensure sensor faces forward (not angled down/up)

### Erratic readings

**Solutions:**
1. Enable filtering: `ultrasonic.set_median_filter_size(5)`
2. Check for electrical noise sources
3. Ensure solid connections (no loose wires)
4. Add capacitor near sensor (100μF)
5. Lower measurement rate

### No readings below 20cm

This is normal - HC-SR04 has 2cm minimum range (blind zone)

## Simulation Mode

When hardware is unavailable, the node operates in simulation mode:

```
[INFO] UltrasonicNode sensor 0: (SIM) distance 1.234 m
```

Simulated behavior:
- Returns random distances between 0.3m and 3.0m
- Includes realistic noise
- Occasional invalid readings for testing error handling
- Useful for logic testing without hardware

## See Also

- [LidarNode](../lidar/) - 360° laser rangefinding
- [DepthCameraNode](../depth_camera/) - Dense 3D depth maps
- [CollisionDetectorNode](../collision_detector/) - Collision avoidance
- [SafetyMonitorNode](../safety_monitor/) - Safety systems
