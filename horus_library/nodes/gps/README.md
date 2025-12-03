# GPS/GNSS Navigation Node

GPS/GNSS satellite navigation receiver for positioning, navigation, and time synchronization with multi-constellation support.

## Quick Start

```rust
use horus_library::nodes::{GpsNode, GpsBackend};
use horus_core::Scheduler;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut scheduler = Scheduler::new();

    // Option 1: Simulation mode (no hardware needed)
    let gps = GpsNode::new()?;

    // Option 2: With real GPS hardware
    let mut gps = GpsNode::new_with_backend("gps.fix", GpsBackend::NmeaSerial)?;
    gps.set_serial_config("/dev/ttyUSB0", 9600);
    gps.set_update_rate(1.0);  // 1 Hz

    scheduler.add(Box::new(gps), 50, Some(true));
    scheduler.run()?;
    Ok(())
}
```

**Publishes to:** `gps.fix` topic with latitude, longitude, altitude, and accuracy data.

## Overview

The GPS Node provides position, velocity, and time data from GPS/GNSS satellite receivers via NMEA 0183 serial protocol. It supports GPS, GLONASS, Galileo, and BeiDou constellations for global positioning with automatic hardware/simulation fallback.

Supports u-blox NEO/ZED series, MediaTek MTK3339, GlobalSat BU-353, Adafruit Ultimate GPS, SparkFun GPS modules, and any NMEA-compatible GPS receiver.

Key features:
- NMEA 0183 protocol support
- Multi-constellation (GPS, GLONASS, Galileo, BeiDou)
- Position (latitude/longitude), altitude, speed
- Course/heading, satellite count, DOP metrics
- Fix quality validation (satellite count, HDOP)
- Configurable baud rates (4800-115200)
- Simulation fallback when hardware unavailable
- Serial port auto-detection

## Topics

### Publishers

| Topic | Type | Description |
|-------|------|-------------|
| `gps.fix` | `NavSatFix` | GPS position, velocity, and accuracy data |

## Configuration Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `backend` | `GpsBackend` | `Simulation` | GPS backend (Simulation, NmeaSerial) |
| `update_rate_hz` | `f32` | `1.0` | Update frequency in Hz (0.1-20 Hz, typical 1-10) |
| `min_satellites` | `u16` | `4` | Minimum satellites required for valid fix |
| `max_hdop` | `f32` | `20.0` | Maximum acceptable HDOP for valid fix |
| `frame_id` | `String` | `"gps"` | Coordinate frame identifier |
| `serial_port` | `String` | `"/dev/ttyUSB0"` | Serial port device path |
| `baud_rate` | `u32` | `9600` | Serial communication baud rate |

### Serial Port Configuration

Common serial ports:
- `/dev/ttyUSB0` - USB GPS modules
- `/dev/ttyAMA0` - Raspberry Pi hardware UART
- `/dev/ttyS0` - Standard serial port
- `/dev/serial0` - Raspberry Pi primary UART symlink

Common baud rates:
- 4800 - Older GPS modules
- 9600 - Standard GPS baud rate (most common)
- 19200 - Some high-performance modules
- 38400 - u-blox high-speed mode
- 57600 - Advanced configurations
- 115200 - Maximum speed for some modules

## Message Types

### NavSatFix

GPS/GNSS position fix message:

```rust
pub struct NavSatFix {
    pub latitude: f64,              // Degrees (positive=North, negative=South)
    pub longitude: f64,             // Degrees (positive=East, negative=West)
    pub altitude: f64,              // Meters above WGS84 ellipsoid
    pub position_covariance: [f64; 9],  // 3x3 covariance matrix [lat, lon, alt]
    pub position_covariance_type: u8,   // Covariance type (0-3)
    pub status: u8,                 // Fix status (0=no_fix, 1=fix, 2=sbas_fix, 3=gbas_fix)
    pub satellites_visible: u16,    // Number of satellites in view
    pub hdop: f32,                  // Horizontal dilution of precision
    pub vdop: f32,                  // Vertical dilution of precision
    pub speed: f32,                 // Ground speed in m/s
    pub heading: f32,               // Course/heading in degrees (0-360)
    pub timestamp: u64,             // Time in nanoseconds since epoch
}
```

**Status Constants**:
- `NavSatFix::STATUS_NO_FIX = 0` - No GPS fix
- `NavSatFix::STATUS_FIX = 1` - Standard GPS fix
- `NavSatFix::STATUS_SBAS_FIX = 2` - SBAS augmented fix (WAAS/EGNOS/MSAS)
- `NavSatFix::STATUS_GBAS_FIX = 3` - GBAS augmented fix

**Covariance Type Constants**:
- `NavSatFix::COVARIANCE_TYPE_UNKNOWN = 0` - Covariance unknown
- `NavSatFix::COVARIANCE_TYPE_APPROXIMATED = 1` - Approximated from HDOP
- `NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN = 2` - Diagonal elements known
- `NavSatFix::COVARIANCE_TYPE_KNOWN = 3` - Full covariance matrix known

**Helper Methods**:
```rust
// Check if we have a valid GPS fix
fix.has_fix() -> bool

// Check if coordinates are valid
fix.is_valid() -> bool

// Get horizontal accuracy estimate in meters
fix.horizontal_accuracy() -> f32

// Calculate distance to another GPS position (Haversine formula)
fix.distance_to(&other_fix) -> f64
```

## Public API

### Construction

```rust
use horus_library::nodes::GpsNode;

// Create with default topic "gps.fix" in simulation mode
let mut gps = GpsNode::new()?;

// Create with custom topic in simulation mode
let mut gps = GpsNode::new_with_topic("navigation.gps")?;

// Create with specific backend
let mut gps = GpsNode::new_with_backend("gps.fix", GpsBackend::NmeaSerial)?;
```

### Configuration Methods

```rust
// Set GPS backend (Simulation or NmeaSerial)
gps.set_backend(GpsBackend::NmeaSerial);

// Configure serial port and baud rate
gps.set_serial_config("/dev/ttyUSB0", 9600);

// Set update rate in Hz (0.1-20 Hz, typical 1-10 Hz)
gps.set_update_rate(1.0);  // 1 Hz (standard)
gps.set_update_rate(5.0);  // 5 Hz (high performance)
gps.set_update_rate(10.0); // 10 Hz (maximum for most modules)

// Set minimum satellites required for valid fix
gps.set_min_satellites(4);  // Minimum for 3D fix
gps.set_min_satellites(6);  // Better accuracy

// Set maximum acceptable HDOP
gps.set_max_hdop(5.0);   // Excellent to good accuracy
gps.set_max_hdop(10.0);  // Moderate accuracy
gps.set_max_hdop(20.0);  // Poor accuracy (default)

// Set coordinate frame ID
gps.set_frame_id("gps_link");

// Set simulation position (simulation mode only)
gps.set_simulation_position(37.7749, -122.4194, 10.0);  // San Francisco
```

### Query Methods

```rust
// Get last GPS fix
let fix = gps.get_last_fix();
println!("Position: {:.6}, {:.6}", fix.latitude, fix.longitude);

// Get total number of fixes received
let count = gps.get_fix_count();
println!("Fixes received: {}", count);

// Check if we have a valid GPS fix
if gps.has_valid_fix() {
    println!("GPS fix valid");
} else {
    println!("Waiting for GPS fix...");
}
```

## Usage Examples

### Basic GPS Reading

```rust
use horus_library::prelude::*;

fn main() -> Result<()> {
    let mut scheduler = Scheduler::new();

    // Create GPS node with NMEA serial backend
    let mut gps = GpsNode::new_with_backend("gps.fix", GpsBackend::NmeaSerial)?;
    gps.set_serial_config("/dev/ttyUSB0", 9600);
    gps.set_update_rate(1.0);  // 1 Hz standard update rate
    gps.set_min_satellites(4);
    gps.set_max_hdop(10.0);

    scheduler.add(Box::new(gps), 1, Some(true));
    scheduler.run()?;
    Ok(())
}
```

### Position Monitoring

```rust
use horus_library::prelude::*;

fn main() -> Result<()> {
    let mut scheduler = Scheduler::new();

    // GPS node
    let mut gps = GpsNode::new_with_backend("gps.fix", GpsBackend::NmeaSerial)?;
    gps.set_serial_config("/dev/ttyUSB0", 9600);
    gps.set_update_rate(1.0);

    scheduler.add(Box::new(gps), 1, Some(true));

    // Position monitor node
    let monitor = node! {
        name: "position_monitor",
        tick: |ctx| {
            let hub = Hub::<NavSatFix>::new("gps.fix")?;

            while let Some(fix) = hub.recv(None) {
                if fix.has_fix() && fix.is_valid() {
                    ctx.log_info(&format!(
                        "Position: {:.6}°, {:.6}°, Alt: {:.1}m",
                        fix.latitude, fix.longitude, fix.altitude
                    ));
                    ctx.log_info(&format!(
                        "Speed: {:.1} m/s, Heading: {:.0}°",
                        fix.speed, fix.heading
                    ));
                    ctx.log_info(&format!(
                        "Satellites: {}, HDOP: {:.1}, Accuracy: {:.1}m",
                        fix.satellites_visible, fix.hdop, fix.horizontal_accuracy()
                    ));
                } else {
                    ctx.log_warning("No valid GPS fix");
                }
            }
            Ok(())
        }
    };

    scheduler.add(Box::new(monitor), 2, Some(true));
    scheduler.run()?;
    Ok(())
}
```

### Geo-Fencing

```rust
use horus_library::prelude::*;

fn main() -> Result<()> {
    let mut scheduler = Scheduler::new();

    // GPS node
    let mut gps = GpsNode::new_with_backend("gps.fix", GpsBackend::NmeaSerial)?;
    gps.set_serial_config("/dev/ttyUSB0", 9600);
    gps.set_update_rate(1.0);

    scheduler.add(Box::new(gps), 1, Some(true));

    // Geo-fence monitor
    let monitor = node! {
        name: "geofence_monitor",
        tick: |ctx| {
            let hub = Hub::<NavSatFix>::new("gps.fix")?;

            // Define geo-fence center (home position)
            let home_lat = 37.7749;
            let home_lon = -122.4194;
            let radius_meters = 100.0;

            while let Some(fix) = hub.recv(None) {
                if fix.has_fix() && fix.is_valid() {
                    // Create home position fix for distance calculation
                    let home = NavSatFix::from_coordinates(home_lat, home_lon, 0.0);
                    let distance = fix.distance_to(&home);

                    if distance > radius_meters {
                        ctx.log_warning(&format!(
                            "OUTSIDE GEO-FENCE! Distance: {:.1}m from home",
                            distance
                        ));
                    } else {
                        ctx.log_info(&format!(
                            "Inside geo-fence: {:.1}m from home",
                            distance
                        ));
                    }
                }
            }
            Ok(())
        }
    };

    scheduler.add(Box::new(monitor), 2, Some(true));
    scheduler.run()?;
    Ok(())
}
```

### Waypoint Navigation

```rust
use horus_library::prelude::*;

fn main() -> Result<()> {
    let mut scheduler = Scheduler::new();

    // GPS node
    let mut gps = GpsNode::new_with_backend("gps.fix", GpsBackend::NmeaSerial)?;
    gps.set_serial_config("/dev/ttyUSB0", 9600);
    gps.set_update_rate(1.0);
    gps.set_min_satellites(6);  // Higher accuracy for navigation
    gps.set_max_hdop(5.0);

    scheduler.add(Box::new(gps), 1, Some(true));

    // Waypoint navigator
    let navigator = node! {
        name: "waypoint_navigator",
        tick: |ctx| {
            let hub = Hub::<NavSatFix>::new("gps.fix")?;

            // Define waypoint
            let waypoint_lat = 37.7750;
            let waypoint_lon = -122.4195;
            let arrival_threshold = 5.0;  // 5 meters

            while let Some(fix) = hub.recv(None) {
                if fix.has_fix() && fix.is_valid() && fix.hdop <= 5.0 {
                    let waypoint = NavSatFix::from_coordinates(waypoint_lat, waypoint_lon, 0.0);
                    let distance = fix.distance_to(&waypoint);

                    if distance <= arrival_threshold {
                        ctx.log_info("WAYPOINT REACHED!");
                    } else {
                        ctx.log_info(&format!(
                            "Distance to waypoint: {:.1}m, Accuracy: {:.1}m",
                            distance, fix.horizontal_accuracy()
                        ));
                    }
                }
            }
            Ok(())
        }
    };

    scheduler.add(Box::new(navigator), 2, Some(true));
    scheduler.run()?;
    Ok(())
}
```

### Speed and Heading Tracking

```rust
use horus_library::prelude::*;

fn main() -> Result<()> {
    let mut scheduler = Scheduler::new();

    // GPS node
    let mut gps = GpsNode::new_with_backend("gps.fix", GpsBackend::NmeaSerial)?;
    gps.set_serial_config("/dev/ttyUSB0", 9600);
    gps.set_update_rate(5.0);  // 5 Hz for better speed/heading updates

    scheduler.add(Box::new(gps), 1, Some(true));

    // Velocity tracker
    let tracker = node! {
        name: "velocity_tracker",
        tick: |ctx| {
            let hub = Hub::<NavSatFix>::new("gps.fix")?;

            while let Some(fix) = hub.recv(None) {
                if fix.has_fix() && fix.is_valid() {
                    // Convert m/s to km/h
                    let speed_kmh = fix.speed * 3.6;

                    if fix.speed > 0.5 {  // Moving (threshold to avoid noise)
                        ctx.log_info(&format!(
                            "Moving: {:.1} km/h, Heading: {:.0}°",
                            speed_kmh, fix.heading
                        ));
                    } else {
                        ctx.log_info("Stationary");
                    }
                }
            }
            Ok(())
        }
    };

    scheduler.add(Box::new(tracker), 2, Some(true));
    scheduler.run()?;
    Ok(())
}
```

### Simulation Mode Testing

```rust
use horus_library::prelude::*;

fn main() -> Result<()> {
    let mut scheduler = Scheduler::new();

    // Create GPS node in simulation mode
    let mut gps = GpsNode::new()?;  // Default is simulation mode

    // Set custom simulation position
    gps.set_simulation_position(
        37.7749,   // Latitude (San Francisco)
        -122.4194, // Longitude
        10.0       // Altitude (meters)
    );

    gps.set_update_rate(1.0);

    scheduler.add(Box::new(gps), 1, Some(true));

    // Test consumer
    let consumer = node! {
        name: "gps_consumer",
        tick: |ctx| {
            let hub = Hub::<NavSatFix>::new("gps.fix")?;

            while let Some(fix) = hub.recv(None) {
                ctx.log_info(&format!(
                    "SIM GPS: {:.6}, {:.6}, {:.1}m - {} sats, HDOP {:.1}",
                    fix.latitude, fix.longitude, fix.altitude,
                    fix.satellites_visible, fix.hdop
                ));
            }
            Ok(())
        }
    };

    scheduler.add(Box::new(consumer), 2, Some(true));
    scheduler.run()?;
    Ok(())
}
```

## Hardware Setup

### Wiring Diagram

```
Raspberry Pi          GPS Module
GPIO TX (14)    --->  RX
GPIO RX (15)    <---  TX
3.3V or 5V      --->  VCC
GND             ---   GND
```

Note: Most GPS modules work with 3.3V or 5V. Check your module's datasheet.

### USB GPS Modules

USB GPS modules appear as `/dev/ttyUSB0` or `/dev/ttyACM0`:

```bash
# List USB serial devices
ls -l /dev/ttyUSB* /dev/ttyACM*

# Check dmesg for connection
dmesg | grep tty
```

### Raspberry Pi UART

For hardware UART on Raspberry Pi:

1. Disable serial console:
```bash
sudo raspi-config
# Interface Options -> Serial Port
# "Would you like a login shell accessible over serial?" -> No
# "Would you like serial port hardware enabled?" -> Yes
```

2. Edit `/boot/config.txt`:
```
# Disable Bluetooth to free up UART
dtoverlay=disable-bt

# Or use miniUART for GPIO
dtoverlay=miniuart-bt
```

3. Reboot:
```bash
sudo reboot
```

4. Use `/dev/ttyAMA0` or `/dev/serial0` as serial port

### System Requirements

```bash
# Check for GPS device
ls -l /dev/ttyUSB* /dev/ttyAMA* /dev/serial*

# Check user permissions (add user to dialout group)
sudo usermod -a -G dialout $USER

# Install serial port tools (optional, for debugging)
sudo apt install minicom cu screen

# Test GPS output
sudo cat /dev/ttyUSB0

# Or use minicom
sudo minicom -D /dev/ttyUSB0 -b 9600
```

### Enable in Project

```toml
[dependencies]
horus_library = { version = "0.1", features = ["nmea-gps"] }
```

```bash
cargo build --features="nmea-gps"
```

## GPS Accuracy and HDOP

### HDOP Values

HDOP (Horizontal Dilution of Precision) indicates GPS accuracy:

| HDOP | Rating | Accuracy | Use Case |
|------|--------|----------|----------|
| < 1 | Ideal | < 5m | Surveying, high-precision |
| 1-2 | Excellent | 5-10m | Navigation, autonomous vehicles |
| 2-5 | Good | 10-25m | General robotics, waypoint following |
| 5-10 | Moderate | 25-50m | Coarse positioning |
| 10-20 | Fair | 50-100m | Poor satellite geometry |
| > 20 | Poor | > 100m | Unreliable, should reject |

### Satellite Count

Minimum satellites for position fix:
- 3 satellites: 2D fix (lat/lon only, no altitude)
- 4 satellites: 3D fix (lat/lon/alt)
- 6+ satellites: Better accuracy and reliability
- 8+ satellites: Optimal accuracy

### Factors Affecting Accuracy

- Satellite geometry (HDOP/VDOP)
- Number of satellites in view
- Atmospheric conditions
- Multipath errors (signal reflections)
- Urban canyons (tall buildings)
- Tree canopy, tunnels
- Electronic interference

### Typical Accuracy

- Consumer GPS: 3-10 meters horizontal
- SBAS (WAAS/EGNOS): 1-3 meters horizontal
- DGPS: < 1 meter horizontal
- RTK GPS: 1-2 cm horizontal (requires base station)

## NMEA Sentences

The GPS node parses standard NMEA 0183 sentences:

- **GPGGA**: Position, altitude, fix quality, satellites
- **GPRMC**: Position, speed, course, date/time
- **GPGSA**: DOP values, fix type, satellite IDs
- **GPGSV**: Satellites in view, elevation, azimuth
- **GPVTG**: Course and speed

The node supports multi-constellation NMEA:
- `GP` - GPS (USA)
- `GL` - GLONASS (Russia)
- `GA` - Galileo (EU)
- `GB` - BeiDou (China)
- `GN` - Combined GNSS

## Best Practices

1. **Wait for valid fix before navigation**:
   ```rust
   if gps.has_valid_fix() {
       // Safe to navigate
   }
   ```

2. **Check HDOP for accuracy requirements**:
   ```rust
   if fix.hdop <= 2.0 {
       // Excellent accuracy for precise navigation
   } else if fix.hdop <= 5.0 {
       // Good accuracy for general robotics
   } else {
       // Poor accuracy, use with caution
   }
   ```

3. **Use outdoor with clear sky view**:
   - GPS requires line-of-sight to satellites
   - Avoid indoor use, dense tree cover, urban canyons
   - Best performance in open areas

4. **Allow time for initial fix (cold start)**:
   - 30-60 seconds for cold start (no almanac data)
   - 1-5 seconds for hot start (recent almanac)
   - Warm start: 10-30 seconds

5. **Set appropriate minimum satellites**:
   ```rust
   gps.set_min_satellites(4);  // Minimum for 3D fix
   gps.set_min_satellites(6);  // Better for navigation
   ```

6. **Use appropriate update rates**:
   ```rust
   gps.set_update_rate(1.0);   // 1 Hz - standard, battery efficient
   gps.set_update_rate(5.0);   // 5 Hz - better for speed/heading
   gps.set_update_rate(10.0);  // 10 Hz - high performance (if supported)
   ```

7. **Validate coordinates**:
   ```rust
   if fix.is_valid() && fix.has_fix() {
       // Coordinates are valid and we have satellite fix
   }
   ```

8. **Consider position covariance**:
   ```rust
   let accuracy = fix.horizontal_accuracy();  // Estimated accuracy in meters
   if accuracy <= 10.0 {
       // Position is sufficiently accurate
   }
   ```

9. **Handle moving vs stationary**:
   ```rust
   if fix.speed > 0.5 {  // Threshold to filter GPS noise
       // Vehicle is moving
   } else {
       // Vehicle is stationary
   }
   ```

10. **Monitor satellite count trends**:
    ```rust
    if fix.satellites_visible < 4 {
        ctx.log_warning("Low satellite count");
    }
    ```

## Troubleshooting

### "Failed to open GPS serial port"

```
[ERROR] Failed to open GPS serial port /dev/ttyUSB0: Permission denied
```

**Solutions:**
1. Check device exists: `ls -l /dev/ttyUSB*`
2. Add user to dialout group: `sudo usermod -a -G dialout $USER`
3. Logout and login again
4. Check permissions: `sudo chmod 666 /dev/ttyUSB0` (temporary)
5. Verify GPS module is connected and powered

### "Serial port not found"

```
[ERROR] Serial port /dev/ttyUSB0 not found
Available ports:
  - /dev/ttyAMA0
```

**Solutions:**
1. List available ports: `ls -l /dev/tty*`
2. Check dmesg: `dmesg | grep tty`
3. Try alternative ports: `/dev/ttyAMA0`, `/dev/serial0`
4. Check USB connection
5. Update `serial_port` parameter to correct device

### No GPS fix / "Waiting for GPS fix"

```
[WARN] Insufficient satellites: 2 < 4
[WARN] No valid GPS fix
```

**Solutions:**
1. Move to outdoor location with clear sky view
2. Wait 30-60 seconds for cold start
3. Check GPS antenna connection
4. Verify module LED blinks (indicates satellite search)
5. Lower minimum satellites temporarily: `gps.set_min_satellites(3)`
6. Check baud rate matches module: Try 4800, 9600, 115200
7. Test with minicom: `sudo minicom -D /dev/ttyUSB0 -b 9600`

### Poor accuracy / High HDOP

```
[WARN] Poor GPS accuracy: HDOP 15.2 > 5.0
```

**Solutions:**
1. Move away from tall buildings (urban canyon effect)
2. Avoid tree canopy, tunnels, parking garages
3. Wait for more satellites to be visible
4. Check for electronic interference sources
5. Increase `max_hdop` threshold if needed
6. Use external active GPS antenna

### Incorrect baud rate

```
[ERROR] GPS NMEA: Timeout reading from serial port
```

**Solutions:**
1. Check module documentation for correct baud rate
2. Try common rates: 4800, 9600, 19200, 38400, 57600, 115200
3. Some modules auto-detect baud rate
4. Configure module baud rate via NMEA command (module-specific)

### UART conflicts on Raspberry Pi

```
[ERROR] Failed to open /dev/ttyAMA0
```

**Solutions:**
1. Disable serial console in `raspi-config`
2. Edit `/boot/config.txt`: Add `dtoverlay=disable-bt`
3. Use `/dev/serial0` instead of `/dev/ttyAMA0`
4. Reboot after configuration changes

### Stationary drift

GPS position slowly drifts when stationary:

**Solutions:**
1. This is normal GPS behavior (multipath, atmospheric effects)
2. Implement stationary detection using speed threshold
3. Average multiple fixes when stationary
4. Use Kalman filter for sensor fusion with IMU
5. Consider RTK GPS for centimeter accuracy

### Hardware unavailable - using SIMULATION mode

```
[INFO] GPS simulation mode enabled
```

This is expected when:
- No GPS hardware connected
- Serial port not configured
- Testing without hardware
- `features = ["nmea-gps"]` not enabled in Cargo.toml

**Solutions:**
1. Enable feature: `cargo build --features="nmea-gps"`
2. Configure serial port: `gps.set_serial_config("/dev/ttyUSB0", 9600)`
3. Set backend: `gps.set_backend(GpsBackend::NmeaSerial)`
4. Use simulation mode for testing: This is expected behavior

## Simulation Mode

When hardware is unavailable, the node operates in simulation mode:

```
[INFO] GPS simulation mode enabled
[DEBUG] GPS: 37.774900, -122.419400, alt=10.0m, sats=8
```

Simulated behavior:
- Returns fixed position (configurable with `set_simulation_position()`)
- Realistic GPS characteristics (8 satellites, HDOP 1.2)
- Position covariance ~3m (typical consumer GPS)
- Zero velocity (stationary)
- Useful for testing navigation logic without hardware

Configure simulation position:
```rust
gps.set_simulation_position(37.7749, -122.4194, 10.0);  // San Francisco
gps.set_simulation_position(51.5074, -0.1278, 50.0);    // London
gps.set_simulation_position(35.6762, 139.6503, 40.0);   // Tokyo
```

## Common GPS Modules

### u-blox NEO-6M/NEO-7M/NEO-8M

- Baud rate: 9600 (default), configurable up to 115200
- Update rate: 1-10 Hz
- Accuracy: 2.5m CEP
- Cold start: 29s, Hot start: 1s

### u-blox ZED-F9P (RTK)

- Baud rate: 38400 (default)
- Update rate: 1-20 Hz
- Accuracy: 0.01m (RTK), 1.5m (standalone)
- Multi-band GNSS

### Adafruit Ultimate GPS (MTK3339)

- Baud rate: 9600 (default)
- Update rate: 1-10 Hz
- Patch antenna or external
- SBAS support (WAAS/EGNOS)

### GlobalSat BU-353

- Baud rate: 4800 (default)
- Update rate: 1 Hz
- USB interface
- SiRF Star III chipset

### SparkFun GPS Modules

- Various chipsets (u-blox, Mediatek)
- Check specific model documentation
- Common baud: 9600 or 38400

## See Also

- [ImuNode](../imu/) - Inertial measurement for sensor fusion
- [OdometryNode](../odometry/) - Dead reckoning navigation
- [LocalizationNode](../localization/) - Multi-sensor fusion
- [PathPlannerNode](../path_planner/) - Waypoint navigation
