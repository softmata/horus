# LiDAR Node

Generic LiDAR interface for obstacle detection and mapping with support for multiple sensor backends.

## Overview

The LiDAR Node provides a unified interface for capturing laser scan data from various LiDAR sensors. It publishes LaserScan messages containing distance measurements in a 360-degree field of view, enabling obstacle detection, mapping, and navigation applications. The node supports configurable scan parameters, multiple sensor backends (RPLidar, YDLIDAR, etc.), and integration with SLAM and path planning systems.

## Topics

### Publishers

| Topic | Type | Description |
|-------|------|-------------|
| `scan` | `LaserScan` | Laser scan data with range measurements and metadata |

## Configuration Parameters

| Parameter | Type | Default | Range | Description |
|-----------|------|---------|-------|-------------|
| `frame_id` | `String` | `"laser_frame"` | - | Coordinate frame identifier for TF transforms |
| `scan_frequency` | `f32` | `10.0` | `0.1-100.0` Hz | Target scan rate in scans per second |
| `min_range` | `f32` | `0.1` | `>=0.0` m | Minimum valid range measurement |
| `max_range` | `f32` | `30.0` | `>min_range` m | Maximum valid range measurement |
| `angle_increment` | `f32` | `π/180` | `0.001-0.1` rad | Angular resolution (default: 1 degree) |

## Message Types

### LaserScan

Laser scan message containing distance measurements and scan metadata:

```rust
pub struct LaserScan {
    /// Range measurements in meters (0 = invalid reading)
    pub ranges: [f32; 360],

    /// Start angle of the scan in radians
    pub angle_min: f32,

    /// End angle of the scan in radians
    pub angle_max: f32,

    /// Minimum valid range in meters
    pub range_min: f32,

    /// Maximum valid range in meters
    pub range_max: f32,

    /// Angular resolution in radians
    pub angle_increment: f32,

    /// Time between measurements in seconds
    pub time_increment: f32,

    /// Time to complete full scan in seconds
    pub scan_time: f32,

    /// Timestamp in nanoseconds since epoch
    pub timestamp: u64,
}
```

### LaserScan Helper Methods

```rust
// Get angle for a specific range index
let angle = scan.angle_at(index);

// Check if a range reading is valid
if scan.is_range_valid(index) {
    // Process valid range
}

// Count all valid range readings
let valid_count = scan.valid_count();

// Find closest obstacle
if let Some(min_dist) = scan.min_range() {
    eprintln!("Closest obstacle at {} meters", min_dist);
}
```

## Public API

### Construction

```rust
use horus_library::nodes::LidarNode;

// Create with default topic "scan"
let mut lidar = LidarNode::new()?;

// Create with custom topic name
let mut lidar = LidarNode::new_with_topic("front_lidar")?;
```

### Configuration Methods

```rust
// Set coordinate frame ID for transforms
lidar.set_frame_id("base_laser");

// Set scan frequency (Hz)
lidar.set_scan_frequency(15.0);  // 15 Hz scan rate

// Set range limits (min, max in meters)
lidar.set_range_limits(0.15, 12.0);  // 15cm - 12m range

// Set angular resolution (radians)
lidar.set_angle_increment(std::f32::consts::PI / 360.0);  // 0.5 degrees

// Get actual scan rate (diagnostics)
let actual_rate = lidar.get_actual_scan_rate();
eprintln!("Actual scan rate: {} Hz", actual_rate);
```

## Usage Examples

### Basic Obstacle Detection

```rust
use horus_library::nodes::LidarNode;
use horus_library::LaserScan;
use horus_core::{Node, Runtime, Receiver};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut runtime = Runtime::new()?;

    // Create LiDAR node
    let mut lidar = LidarNode::new()?;
    lidar.set_scan_frequency(10.0);
    lidar.set_range_limits(0.1, 30.0);

    // Create subscriber to process scan data
    let scan_sub = Receiver::<LaserScan>::new("scan")?;

    runtime.add_node(lidar);
    runtime.spawn(move || {
        loop {
            if let Ok(scan) = scan_sub.recv() {
                // Find closest obstacle
                if let Some(min_dist) = scan.min_range() {
                    if min_dist < 0.5 {
                        eprintln!("Warning: Obstacle at {} meters!", min_dist);
                    }
                }
            }
        }
    });

    runtime.run()?;
    Ok(())
}
```

### RPLidar A1 Configuration

```rust
use horus_library::nodes::LidarNode;
use horus_core::{Node, Runtime};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut runtime = Runtime::new()?;

    // Configure for RPLidar A1 specifications
    let mut lidar = LidarNode::new_with_topic("rplidar_scan")?;
    lidar.set_frame_id("rplidar_frame");
    lidar.set_scan_frequency(10.0);       // 5.5-10 Hz typical
    lidar.set_range_limits(0.15, 12.0);   // 15cm - 12m range
    lidar.set_angle_increment(std::f32::consts::PI / 180.0);  // 1 degree

    runtime.add_node(lidar);
    runtime.run()?;
    Ok(())
}
```

### YDLIDAR X4 Configuration

```rust
use horus_library::nodes::LidarNode;
use horus_core::{Node, Runtime};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut runtime = Runtime::new()?;

    // Configure for YDLIDAR X4 specifications
    let mut lidar = LidarNode::new_with_topic("ydlidar_scan")?;
    lidar.set_frame_id("ydlidar_frame");
    lidar.set_scan_frequency(7.0);        // 7 Hz typical
    lidar.set_range_limits(0.12, 10.0);   // 12cm - 10m range
    lidar.set_angle_increment(std::f32::consts::PI / 180.0);

    runtime.add_node(lidar);
    runtime.run()?;
    Ok(())
}
```

### Multi-LiDAR Setup (Front and Rear)

```rust
use horus_library::nodes::LidarNode;
use horus_library::LaserScan;
use horus_core::{Node, Runtime, Receiver};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut runtime = Runtime::new()?;

    // Front LiDAR
    let mut front_lidar = LidarNode::new_with_topic("front_scan")?;
    front_lidar.set_frame_id("front_laser");
    front_lidar.set_scan_frequency(10.0);
    front_lidar.set_range_limits(0.15, 12.0);

    // Rear LiDAR
    let mut rear_lidar = LidarNode::new_with_topic("rear_scan")?;
    rear_lidar.set_frame_id("rear_laser");
    rear_lidar.set_scan_frequency(10.0);
    rear_lidar.set_range_limits(0.15, 12.0);

    // Subscribe to both lidars
    let front_sub = Receiver::<LaserScan>::new("front_scan")?;
    let rear_sub = Receiver::<LaserScan>::new("rear_scan")?;

    runtime.add_node(front_lidar);
    runtime.add_node(rear_lidar);

    // Process front scan
    runtime.spawn(move || {
        loop {
            if let Ok(scan) = front_sub.recv() {
                eprintln!("Front LiDAR: {} valid points", scan.valid_count());
            }
        }
    });

    // Process rear scan
    runtime.spawn(move || {
        loop {
            if let Ok(scan) = rear_sub.recv() {
                eprintln!("Rear LiDAR: {} valid points", scan.valid_count());
            }
        }
    });

    runtime.run()?;
    Ok(())
}
```

### Collision Detection Zones

```rust
use horus_library::nodes::LidarNode;
use horus_library::LaserScan;
use horus_core::{Node, Runtime, Receiver};

fn check_collision_zones(scan: &LaserScan) {
    let num_zones = 8;
    let zone_angle = 2.0 * std::f32::consts::PI / num_zones as f32;

    for zone in 0..num_zones {
        let mut min_dist_in_zone = f32::MAX;
        let zone_start = zone as f32 * zone_angle;
        let zone_end = (zone + 1) as f32 * zone_angle;

        // Check all points in this zone
        for i in 0..360 {
            if !scan.is_range_valid(i) {
                continue;
            }

            let angle = scan.angle_at(i);
            if angle >= zone_start && angle < zone_end {
                min_dist_in_zone = min_dist_in_zone.min(scan.ranges[i]);
            }
        }

        // Alert if obstacle too close in this zone
        if min_dist_in_zone < 0.5 {
            eprintln!("Zone {}: COLLISION RISK at {:.2}m", zone, min_dist_in_zone);
        }
    }
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut runtime = Runtime::new()?;
    let mut lidar = LidarNode::new()?;
    let scan_sub = Receiver::<LaserScan>::new("scan")?;

    runtime.add_node(lidar);
    runtime.spawn(move || {
        loop {
            if let Ok(scan) = scan_sub.recv() {
                check_collision_zones(&scan);
            }
        }
    });

    runtime.run()?;
    Ok(())
}
```

## Supported LiDAR Models

### RPLidar Series

| Model | Range | Scan Rate | Angular Resolution | Notes |
|-------|-------|-----------|-------------------|-------|
| **RPLidar A1** | 0.15-12m | 5.5-10 Hz | 1 degree | Budget-friendly, widely used |
| **RPLidar A2** | 0.15-18m | 5-15 Hz | 0.9 degrees | Better range and speed |
| **RPLidar A3** | 0.15-25m | 5-20 Hz | 0.225-0.45 degrees | High performance |
| **RPLidar S1** | 0.15-40m | 10 Hz | 0.39 degrees | Outdoor capable |

**Configuration Example:**
```rust
lidar.set_range_limits(0.15, 12.0);  // A1
lidar.set_scan_frequency(10.0);
lidar.set_angle_increment(std::f32::consts::PI / 180.0);
```

### YDLIDAR Series

| Model | Range | Scan Rate | Angular Resolution | Notes |
|-------|-------|-----------|-------------------|-------|
| **YDLIDAR X4** | 0.12-10m | 7 Hz | 1 degree | Compact design |
| **YDLIDAR X2/X2L** | 0.12-8m | 8 Hz | 1 degree | Ultra-compact |
| **YDLIDAR G4** | 0.12-16m | 9 Hz | 0.48 degrees | High precision |
| **YDLIDAR TG15** | 0.05-30m | 10 Hz | 0.13 degrees | Time-of-flight |

**Configuration Example:**
```rust
lidar.set_range_limits(0.12, 10.0);  // X4
lidar.set_scan_frequency(7.0);
lidar.set_angle_increment(std::f32::consts::PI / 180.0);
```

### SLAMTEC Mapper Series

| Model | Range | Scan Rate | Angular Resolution | Notes |
|-------|-------|-----------|-------------------|-------|
| **Mapper M1M1** | 0.15-40m | 10 Hz | 0.39 degrees | Professional grade |
| **Mapper M2M1** | 0.1-64m | 15 Hz | 0.225 degrees | Long-range outdoor |

**Configuration Example:**
```rust
lidar.set_range_limits(0.15, 40.0);  // M1M1
lidar.set_scan_frequency(10.0);
lidar.set_angle_increment(0.39 * std::f32::consts::PI / 180.0);
```

### Hokuto/UTM Series

| Model | Range | Scan Rate | Angular Resolution | Notes |
|-------|-------|-----------|-------------------|-------|
| **UTM-30LX** | 0.1-30m | 40 Hz | 0.25 degrees | Industrial grade |
| **URG-04LX** | 0.02-5.6m | 10 Hz | 0.36 degrees | Short-range precision |

**Configuration Example:**
```rust
lidar.set_range_limits(0.1, 30.0);  // UTM-30LX
lidar.set_scan_frequency(40.0);
lidar.set_angle_increment(0.25 * std::f32::consts::PI / 180.0);
```

## Scan Parameters Explanation

### Angular Resolution (angle_increment)

The angular spacing between consecutive range measurements:

```rust
// 1 degree resolution (360 points per scan)
lidar.set_angle_increment(std::f32::consts::PI / 180.0);

// 0.5 degree resolution (720 points per scan)
lidar.set_angle_increment(std::f32::consts::PI / 360.0);

// Calculate number of points in scan
let num_points = (2.0 * std::f32::consts::PI / angle_increment) as usize;
```

**Trade-offs:**
- **Higher resolution** (smaller increment): More detail, slower processing, more data
- **Lower resolution** (larger increment): Faster processing, less detail

### Range Limits

Define the valid measurement range for the sensor:

```rust
// Indoor navigation (short-range, high precision)
lidar.set_range_limits(0.1, 5.0);

// Outdoor navigation (long-range)
lidar.set_range_limits(0.2, 30.0);

// Obstacle avoidance (medium-range)
lidar.set_range_limits(0.15, 12.0);
```

**Purpose:**
- `min_range`: Filter out invalid close readings (reflections, sensor housing)
- `max_range`: Filter out noise at maximum distance
- Readings outside limits are marked as invalid

### Scan Frequency

Number of complete 360-degree scans per second:

```rust
// Low frequency for mapping (5 Hz)
lidar.set_scan_frequency(5.0);

// Medium frequency for navigation (10 Hz)
lidar.set_scan_frequency(10.0);

// High frequency for dynamic obstacle avoidance (20 Hz)
lidar.set_scan_frequency(20.0);
```

**Considerations:**
- Hardware limitations (check sensor specs)
- Processing capacity
- Application requirements (static vs dynamic environments)

### Frame ID

Coordinate frame identifier for TF transforms:

```rust
// Single LiDAR setup
lidar.set_frame_id("base_laser");

// Multi-LiDAR setup with unique frames
front_lidar.set_frame_id("front_laser");
rear_lidar.set_frame_id("rear_laser");
left_lidar.set_frame_id("left_laser");
```

**Usage in transforms:**
- Defines the origin of scan measurements
- Required for sensor fusion and mapping
- Must match your robot's TF tree

## Interpreting Scan Data

### Understanding the Ranges Array

The `ranges` array contains 360 distance measurements:

```rust
// Get scan data
let scan = scan_sub.recv()?;

// Iterate through all measurements
for i in 0..360 {
    if scan.is_range_valid(i) {
        let angle = scan.angle_at(i);
        let distance = scan.ranges[i];

        // Convert to Cartesian coordinates
        let x = distance * angle.cos();
        let y = distance * angle.sin();

        eprintln!("Point {}: angle={:.2}°, dist={:.2}m, pos=({:.2}, {:.2})",
                 i, angle.to_degrees(), distance, x, y);
    }
}
```

### Coordinate System

```
        0° (forward)
             |
             |
   270° -----+----- 90°
  (left)     |    (right)
             |
          180° (rear)
```

### Finding Obstacles in Specific Directions

```rust
fn check_direction(scan: &LaserScan, direction_deg: f32, tolerance_deg: f32) -> Option<f32> {
    let target_angle = direction_deg.to_radians();
    let tolerance = tolerance_deg.to_radians();
    let mut min_dist = f32::MAX;

    for i in 0..360 {
        if !scan.is_range_valid(i) {
            continue;
        }

        let angle = scan.angle_at(i);
        let angle_diff = (angle - target_angle).abs();

        if angle_diff <= tolerance {
            min_dist = min_dist.min(scan.ranges[i]);
        }
    }

    if min_dist < f32::MAX {
        Some(min_dist)
    } else {
        None
    }
}

// Usage
if let Some(dist) = check_direction(&scan, 0.0, 10.0) {
    eprintln!("Obstacle ahead at {} meters", dist);
}
```

## Integration with SLAM/Navigation

### GMapping SLAM Integration

```rust
use horus_library::nodes::LidarNode;
use horus_library::LaserScan;
use horus_core::{Node, Runtime, Receiver};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut runtime = Runtime::new()?;

    // Configure LiDAR for SLAM
    let mut lidar = LidarNode::new()?;
    lidar.set_frame_id("base_laser");
    lidar.set_scan_frequency(10.0);  // 10 Hz for stable mapping
    lidar.set_range_limits(0.15, 12.0);
    lidar.set_angle_increment(std::f32::consts::PI / 180.0);

    // Subscribe to scans for SLAM processing
    let scan_sub = Receiver::<LaserScan>::new("scan")?;

    runtime.add_node(lidar);
    runtime.spawn(move || {
        loop {
            if let Ok(scan) = scan_sub.recv() {
                // Filter out invalid readings for better SLAM results
                let valid_ratio = scan.valid_count() as f32 / 360.0;
                if valid_ratio > 0.8 {  // At least 80% valid points
                    // Pass to SLAM system
                    process_slam(&scan);
                }
            }
        }
    });

    runtime.run()?;
    Ok(())
}

fn process_slam(scan: &LaserScan) {
    // SLAM processing implementation
}
```

### Navigation Stack Integration

```rust
use horus_library::nodes::{LidarNode, PathPlannerNode, CollisionDetectorNode};
use horus_library::LaserScan;
use horus_core::{Node, Runtime};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut runtime = Runtime::new()?;

    // LiDAR provides scan data
    let mut lidar = LidarNode::new()?;
    lidar.set_frame_id("base_laser");
    lidar.set_scan_frequency(15.0);  // 15 Hz for responsive navigation

    // Collision detector uses scan data
    let collision_detector = CollisionDetectorNode::new(
        "scan",           // Subscribe to LiDAR
        "collision",      // Publish collision events
        0.3,              // 30cm safety margin
    )?;

    // Path planner considers obstacles
    let path_planner = PathPlannerNode::new_with_topics(
        "goal",
        "scan",           // Uses LiDAR for obstacle avoidance
        "path",
    )?;

    runtime.add_node(lidar);
    runtime.add_node(collision_detector);
    runtime.add_node(path_planner);
    runtime.run()?;

    Ok(())
}
```

### Costmap Generation

```rust
use horus_library::LaserScan;

const GRID_SIZE: usize = 200;  // 200x200 grid
const GRID_RESOLUTION: f32 = 0.05;  // 5cm per cell

fn generate_costmap(scan: &LaserScan) -> [[u8; GRID_SIZE]; GRID_SIZE] {
    let mut costmap = [[0u8; GRID_SIZE]; GRID_SIZE];
    let center = GRID_SIZE / 2;

    for i in 0..360 {
        if !scan.is_range_valid(i) {
            continue;
        }

        let angle = scan.angle_at(i);
        let distance = scan.ranges[i];

        // Mark obstacle cell
        let x = center + (distance * angle.cos() / GRID_RESOLUTION) as usize;
        let y = center + (distance * angle.sin() / GRID_RESOLUTION) as usize;

        if x < GRID_SIZE && y < GRID_SIZE {
            costmap[y][x] = 255;  // Obstacle

            // Inflate obstacle (safety margin)
            let inflation = 3;  // 3 cells = 15cm
            for dy in -(inflation as i32)..=(inflation as i32) {
                for dx in -(inflation as i32)..=(inflation as i32) {
                    let nx = (x as i32 + dx) as usize;
                    let ny = (y as i32 + dy) as usize;
                    if nx < GRID_SIZE && ny < GRID_SIZE && costmap[ny][nx] == 0 {
                        let dist = ((dx * dx + dy * dy) as f32).sqrt();
                        costmap[ny][nx] = ((255.0 * (1.0 - dist / inflation as f32)) as u8).max(50);
                    }
                }
            }
        }
    }

    costmap
}
```

### Localization with Particle Filter

```rust
use horus_library::nodes::LidarNode;
use horus_library::LaserScan;
use horus_core::{Node, Runtime, Receiver};

struct Particle {
    x: f32,
    y: f32,
    theta: f32,
    weight: f32,
}

fn match_scan_to_map(scan: &LaserScan, particle: &Particle, map: &[u8]) -> f32 {
    let mut score = 0.0;
    let mut count = 0;

    for i in 0..360 {
        if !scan.is_range_valid(i) {
            continue;
        }

        let angle = scan.angle_at(i) + particle.theta;
        let distance = scan.ranges[i];

        let x = particle.x + distance * angle.cos();
        let y = particle.y + distance * angle.sin();

        // Check if this point matches known obstacles in map
        // (Simplified - real implementation would use proper map lookup)
        score += 1.0;
        count += 1;
    }

    if count > 0 {
        score / count as f32
    } else {
        0.0
    }
}
```

## Troubleshooting

### Issue: No scan data published

**Symptoms:**
- No messages on `scan` topic
- Receiver blocks indefinitely

**Possible causes and solutions:**

1. **LiDAR not initialized**
   ```rust
   // Check initialization in logs
   // Ensure hardware is connected
   ```

2. **Node not added to runtime**
   ```rust
   // Make sure node is added
   runtime.add_node(lidar);
   ```

3. **Wrong topic name**
   ```rust
   // Verify topic names match
   let lidar = LidarNode::new_with_topic("scan")?;
   let sub = Receiver::<LaserScan>::new("scan")?;  // Must match
   ```

### Issue: Serial port permission denied

**Symptoms:**
- Error: "Permission denied" when opening serial port
- Cannot connect to LiDAR hardware

**Solution:**

```bash
# Add user to dialout group (Linux)
sudo usermod -a -G dialout $USER

# Set permissions on device
sudo chmod 666 /dev/ttyUSB0

# Verify device exists
ls -l /dev/ttyUSB*
```

**Permanent fix:**
```bash
# Create udev rule
sudo nano /etc/udev/rules.d/99-lidar.rules

# Add rule for RPLidar
KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", MODE="0666"

# Reload rules
sudo udevadm control --reload-rules
sudo udevadm trigger
```

### Issue: Inconsistent scan rate

**Symptoms:**
- Actual scan rate differs from configured rate
- Variable timing between scans

**Diagnosis:**
```rust
// Monitor actual scan rate
let actual_rate = lidar.get_actual_scan_rate();
eprintln!("Target: {} Hz, Actual: {} Hz", scan_frequency, actual_rate);
```

**Solutions:**

1. **Hardware limitation**
   ```rust
   // Reduce scan frequency to match hardware capability
   lidar.set_scan_frequency(7.0);  // Lower than max
   ```

2. **Processing bottleneck**
   ```rust
   // Optimize scan processing
   // Use parallel processing for multiple subscribers
   ```

3. **Serial port buffer**
   ```rust
   // Increase buffer size (implementation-specific)
   // Flush old data before reading
   ```

### Issue: Noisy or invalid readings

**Symptoms:**
- Many invalid readings (out of range)
- Erratic distance measurements
- `valid_count()` is low

**Solutions:**

1. **Adjust range limits**
   ```rust
   // Filter out noise at extremes
   lidar.set_range_limits(0.2, 10.0);  // Tighter limits
   ```

2. **Filter in post-processing**
   ```rust
   fn filter_scan(scan: &LaserScan) -> Vec<f32> {
       let mut filtered = Vec::new();
       let mut prev_range = 0.0f32;

       for i in 0..360 {
           if !scan.is_range_valid(i) {
               continue;
           }

           let range = scan.ranges[i];

           // Remove sudden jumps (likely noise)
           if prev_range > 0.0 && (range - prev_range).abs() > 2.0 {
               continue;  // Skip this reading
           }

           filtered.push(range);
           prev_range = range;
       }

       filtered
   }
   ```

3. **Check hardware**
   - Clean LiDAR lens/window
   - Ensure stable mounting (no vibration)
   - Check for reflective surfaces
   - Verify power supply stability

### Issue: USB device disconnects

**Symptoms:**
- LiDAR stops publishing mid-operation
- "Device not found" errors

**Solutions:**

1. **USB power management**
   ```bash
   # Disable USB autosuspend (Linux)
   echo -1 | sudo tee /sys/module/usbcore/parameters/autosuspend
   ```

2. **Use powered USB hub**
   - Some LiDAR models require more power than USB port provides
   - Use hub with external power supply

3. **Check USB cable quality**
   - Use high-quality, shielded USB cable
   - Keep cable length under 3 meters

### Issue: Scans misaligned with robot motion

**Symptoms:**
- Mapping produces distorted results
- Objects appear stretched or compressed

**Possible causes:**

1. **Incorrect frame_id**
   ```rust
   // Ensure frame_id matches TF tree
   lidar.set_frame_id("base_laser");  // Must match your TF configuration
   ```

2. **Timestamp synchronization**
   ```rust
   // Check timestamp alignment with other sensors
   eprintln!("Scan timestamp: {}", scan.timestamp);
   ```

3. **Motion blur**
   - Increase scan frequency for faster moving robots
   - Implement motion compensation in SLAM

### Issue: Low performance with multiple LiDARs

**Symptoms:**
- System slows down with multiple LiDAR nodes
- High CPU usage

**Solutions:**

1. **Distribute processing**
   ```rust
   // Use separate threads for each LiDAR
   runtime.spawn(move || {
       // Process front_lidar in dedicated thread
   });

   runtime.spawn(move || {
       // Process rear_lidar in dedicated thread
   });
   ```

2. **Reduce scan frequency**
   ```rust
   // Not all LiDARs need high frequency
   front_lidar.set_scan_frequency(15.0);  // High for navigation
   rear_lidar.set_scan_frequency(5.0);    // Low for monitoring
   ```

3. **Optimize subscribers**
   ```rust
   // Use single subscriber with zone processing
   // Instead of multiple subscribers processing full scan
   ```

## Performance Considerations

### CPU Usage

Typical CPU usage per LiDAR node:
- **Scan generation**: <1% CPU (synthetic data)
- **Data publishing**: ~2-5% CPU (10 Hz, 360 points)
- **Subscriber processing**: Depends on application

### Memory Usage

Fixed memory footprint:
- **LaserScan message**: ~1.5 KB (360 floats + metadata)
- **Node state**: <1 KB
- **Total per node**: ~2-3 KB

### Bandwidth

Data rate calculation:
```
Rate = scan_frequency × scan_size
     = 10 Hz × 1.5 KB
     = 15 KB/s per LiDAR
```

For multiple LiDARs:
```
4 LiDARs × 15 KB/s = 60 KB/s
```

### Optimization Tips

1. **Reduce angular resolution for distant LiDARs**
   ```rust
   rear_lidar.set_angle_increment(2.0 * std::f32::consts::PI / 180.0);  // 2 degrees
   ```

2. **Use different scan frequencies for different purposes**
   ```rust
   navigation_lidar.set_scan_frequency(20.0);  // High frequency
   mapping_lidar.set_scan_frequency(5.0);      // Low frequency
   ```

3. **Process scans in parallel**
   ```rust
   use rayon::prelude::*;

   // Process scan points in parallel
   let obstacles: Vec<_> = (0..360)
       .into_par_iter()
       .filter(|&i| scan.is_range_valid(i))
       .filter(|&i| scan.ranges[i] < threshold)
       .collect();
   ```

## Related Nodes

- **CollisionDetectorNode**: Uses LaserScan for obstacle detection
- **PathPlannerNode**: Integrates scan data for path planning
- **LocalizationNode**: Uses scans for pose estimation
- **MapBuilderNode**: Creates occupancy grids from scans

## See Also

- [LaserScan Message Format](../messages/sensor.rs)
- [RPLidar SDK Documentation](https://github.com/Slamtec/rplidar_sdk)
- [ROS LaserScan Message](http://docs.ros.org/en/api/sensor_msgs/html/msg/LaserScan.html)
- [SLAM Algorithms Overview](https://en.wikipedia.org/wiki/Simultaneous_localization_and_mapping)
