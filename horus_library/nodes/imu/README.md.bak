# IMU Node

Inertial Measurement Unit for orientation sensing and motion tracking.

## Overview

The IMU Node reads accelerometer, gyroscope, and magnetometer data from IMU sensors and publishes Imu messages with orientation and motion information. It provides essential data for robot localization, orientation tracking, and sensor fusion applications. The node supports configurable sample rates and frame IDs for integration with coordinate frame systems.

## Topics

### Publishers

| Topic | Type | Description |
|-------|------|-------------|
| `imu` | `Imu` | IMU sensor data including orientation, angular velocity, and linear acceleration |

## Configuration Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `frame_id` | `String` | `"imu_link"` | Coordinate frame identifier for the IMU |
| `sample_rate` | `f32` | `100.0` | Target sample rate in Hz (clamped 1.0-1000.0) |
| `topic` | `String` | `"imu"` | Topic name for publishing IMU data |

## Message Types

### Imu

IMU sensor data message containing orientation, angular velocity, and linear acceleration:

```rust
pub struct Imu {
    /// Orientation as quaternion [x, y, z, w]
    pub orientation: [f64; 4],

    /// Orientation covariance matrix (row-major, -1 = no data)
    pub orientation_covariance: [f64; 9],

    /// Angular velocity [x, y, z] in rad/s
    pub angular_velocity: [f64; 3],

    /// Angular velocity covariance matrix
    pub angular_velocity_covariance: [f64; 9],

    /// Linear acceleration [x, y, z] in m/s²
    pub linear_acceleration: [f64; 3],

    /// Linear acceleration covariance matrix
    pub linear_acceleration_covariance: [f64; 9],

    /// Timestamp in nanoseconds since epoch
    pub timestamp: u64,
}
```

### Imu Helper Methods

```rust
impl Imu {
    /// Create a new IMU message with identity quaternion
    pub fn new() -> Self;

    /// Set orientation from Euler angles (roll, pitch, yaw)
    pub fn set_orientation_from_euler(&mut self, roll: f64, pitch: f64, yaw: f64);

    /// Check if orientation data is available
    pub fn has_orientation(&self) -> bool;

    /// Check if all values are finite and valid
    pub fn is_valid(&self) -> bool;

    /// Get angular velocity as Vector3
    pub fn angular_velocity_vec(&self) -> Vector3;

    /// Get linear acceleration as Vector3
    pub fn linear_acceleration_vec(&self) -> Vector3;
}
```

## Public API

### Construction

```rust
use horus_library::nodes::ImuNode;

// Create with default topic "imu"
let mut imu = ImuNode::new()?;

// Create with custom topic
let mut imu = ImuNode::new_with_topic("sensors/imu")?;
```

### Configuration Methods

```rust
// Set coordinate frame ID
imu.set_frame_id("base_imu");

// Set sample rate (Hz, clamped to 1.0-1000.0)
imu.set_sample_rate(200.0);

// Get actual measured sample rate
let actual_rate = imu.get_actual_sample_rate();
```

## Usage Examples

### Basic Orientation Tracking

```rust
use horus_library::nodes::ImuNode;
use horus_library::Imu;
use horus_core::{Node, Runtime, Hub};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut runtime = Runtime::new()?;

    // Create IMU node
    let mut imu_node = ImuNode::new()?;
    imu_node.set_frame_id("robot_imu");
    imu_node.set_sample_rate(100.0);  // 100 Hz

    // Subscribe to IMU data
    let imu_sub = Hub::<Imu>::subscribe("imu")?;

    runtime.add_node(imu_node);
    runtime.run()?;

    Ok(())
}
```

### Orientation Monitoring with Euler Angles

```rust
use horus_library::nodes::ImuNode;
use horus_library::{Imu, Quaternion};
use horus_core::{Node, Runtime, Hub};
use std::f64::consts::PI;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut runtime = Runtime::new()?;

    // Create IMU node with high sample rate
    let mut imu_node = ImuNode::new()?;
    imu_node.set_sample_rate(200.0);

    // Subscribe and process IMU data
    let imu_sub = Hub::<Imu>::subscribe("imu")?;

    runtime.add_node_fn(move || {
        if let Ok(imu_data) = imu_sub.try_recv() {
            if imu_data.has_orientation() {
                // Convert quaternion to Euler angles
                let q = Quaternion::new(
                    imu_data.orientation[0],
                    imu_data.orientation[1],
                    imu_data.orientation[2],
                    imu_data.orientation[3]
                );
                let (roll, pitch, yaw) = q.to_euler();

                eprintln!("Orientation - Roll: {:.2}°, Pitch: {:.2}°, Yaw: {:.2}°",
                    roll * 180.0 / PI,
                    pitch * 180.0 / PI,
                    yaw * 180.0 / PI
                );
            }
        }
    });

    runtime.add_node(imu_node);
    runtime.run()?;

    Ok(())
}
```

### Motion Detection

```rust
use horus_library::nodes::ImuNode;
use horus_library::Imu;
use horus_core::{Node, Runtime, Hub};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut runtime = Runtime::new()?;

    // Create IMU node
    let mut imu_node = ImuNode::new()?;
    imu_node.set_sample_rate(100.0);

    // Motion detection subscriber
    let imu_sub = Hub::<Imu>::subscribe("imu")?;

    runtime.add_node_fn(move || {
        if let Ok(imu_data) = imu_sub.try_recv() {
            // Calculate angular velocity magnitude
            let angular_mag = (
                imu_data.angular_velocity[0].powi(2) +
                imu_data.angular_velocity[1].powi(2) +
                imu_data.angular_velocity[2].powi(2)
            ).sqrt();

            // Calculate linear acceleration magnitude (excluding gravity)
            let accel_mag = (
                imu_data.linear_acceleration[0].powi(2) +
                imu_data.linear_acceleration[1].powi(2) +
                (imu_data.linear_acceleration[2] + 9.81).powi(2)
            ).sqrt();

            // Detect significant motion
            if angular_mag > 0.5 || accel_mag > 2.0 {
                eprintln!("Motion detected! Angular: {:.2} rad/s, Accel: {:.2} m/s²",
                    angular_mag, accel_mag);
            }
        }
    });

    runtime.add_node(imu_node);
    runtime.run()?;

    Ok(())
}
```

### Sensor Fusion with Odometry

```rust
use horus_library::nodes::ImuNode;
use horus_library::{Imu, Odometry};
use horus_core::{Node, Runtime, Hub};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut runtime = Runtime::new()?;

    // Create IMU node
    let mut imu_node = ImuNode::new()?;
    imu_node.set_sample_rate(100.0);

    // Subscribe to both IMU and odometry
    let imu_sub = Hub::<Imu>::subscribe("imu")?;
    let odom_sub = Hub::<Odometry>::subscribe("odom")?;
    let fused_pub = Hub::<Odometry>::new("fused_odom")?;

    runtime.add_node_fn(move || {
        // Simple complementary filter
        if let (Ok(imu_data), Ok(mut odom_data)) =
            (imu_sub.try_recv(), odom_sub.try_recv()) {

            // Use IMU orientation for more accurate heading
            if imu_data.has_orientation() {
                // Extract yaw from quaternion
                let q = imu_data.orientation;
                let yaw = (2.0 * (q[3] * q[2] + q[0] * q[1]))
                    .atan2(1.0 - 2.0 * (q[1].powi(2) + q[2].powi(2)));

                // Update odometry with IMU yaw
                odom_data.pose.theta = yaw;

                let _ = fused_pub.send(odom_data, None);
            }
        }
    });

    runtime.add_node(imu_node);
    runtime.run()?;

    Ok(())
}
```

### Multiple IMU Sensors

```rust
use horus_library::nodes::ImuNode;
use horus_core::{Node, Runtime};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut runtime = Runtime::new()?;

    // Primary IMU for robot base
    let mut base_imu = ImuNode::new_with_topic("imu/base")?;
    base_imu.set_frame_id("base_imu");
    base_imu.set_sample_rate(100.0);

    // Secondary IMU for arm or turret
    let mut arm_imu = ImuNode::new_with_topic("imu/arm")?;
    arm_imu.set_frame_id("arm_imu");
    arm_imu.set_sample_rate(50.0);

    runtime.add_node(base_imu);
    runtime.add_node(arm_imu);
    runtime.run()?;

    Ok(())
}
```

## Calibration Procedures

### Accelerometer Calibration

The accelerometer measures linear acceleration including gravity. Proper calibration is essential for accurate measurements.

#### Six-Point Calibration

1. **Collect data in six orientations** (each axis aligned with gravity):
   ```rust
   // Place IMU with each axis pointing up/down
   // Collect 100 samples per orientation
   let mut samples = Vec::new();
   for _ in 0..100 {
       let imu_data = imu_sub.recv()?;
       samples.push(imu_data.linear_acceleration);
   }
   ```

2. **Calculate bias and scale**:
   ```rust
   // Expected: ±9.81 m/s² on axis aligned with gravity, 0 on others
   // Calculate offset (bias) and scale factor for each axis
   let x_bias = (x_up + x_down) / 2.0;
   let x_scale = 9.81 / ((x_up - x_down) / 2.0);
   ```

3. **Apply calibration**:
   ```rust
   let calibrated_x = (raw_x - x_bias) * x_scale;
   let calibrated_y = (raw_y - y_bias) * y_scale;
   let calibrated_z = (raw_z - z_bias) * z_scale;
   ```

### Gyroscope Calibration

The gyroscope measures angular velocity. It typically needs bias (zero-offset) calibration.

#### Static Calibration

1. **Keep IMU stationary**:
   ```rust
   // Collect 1000 samples while IMU is perfectly still
   let mut gyro_samples = Vec::new();
   for _ in 0..1000 {
       let imu_data = imu_sub.recv()?;
       gyro_samples.push(imu_data.angular_velocity);
   }
   ```

2. **Calculate bias**:
   ```rust
   let gyro_x_bias = gyro_samples.iter()
       .map(|s| s[0]).sum::<f64>() / gyro_samples.len() as f64;
   let gyro_y_bias = gyro_samples.iter()
       .map(|s| s[1]).sum::<f64>() / gyro_samples.len() as f64;
   let gyro_z_bias = gyro_samples.iter()
       .map(|s| s[2]).sum::<f64>() / gyro_samples.len() as f64;
   ```

3. **Apply calibration**:
   ```rust
   let calibrated_gyro_x = raw_gyro_x - gyro_x_bias;
   let calibrated_gyro_y = raw_gyro_y - gyro_y_bias;
   let calibrated_gyro_z = raw_gyro_z - gyro_z_bias;
   ```

### Magnetometer Calibration

Magnetometers are sensitive to hard-iron (permanent magnets) and soft-iron (ferromagnetic materials) distortions.

#### Hard-Iron Calibration

1. **Rotate IMU through full 360° in all axes**:
   ```rust
   // Collect data while rotating IMU slowly
   let mut mag_samples = Vec::new();
   // Rotate for 30-60 seconds, collecting samples
   ```

2. **Calculate offset** (center of sphere):
   ```rust
   let mag_x_offset = (mag_x_max + mag_x_min) / 2.0;
   let mag_y_offset = (mag_y_max + mag_y_min) / 2.0;
   let mag_z_offset = (mag_z_max + mag_z_min) / 2.0;
   ```

3. **Apply calibration**:
   ```rust
   let calibrated_mag_x = raw_mag_x - mag_x_offset;
   let calibrated_mag_y = raw_mag_y - mag_y_offset;
   let calibrated_mag_z = raw_mag_z - mag_z_offset;
   ```

### Calibration Storage

Store calibration parameters for reuse:

```rust
use serde::{Serialize, Deserialize};
use std::fs;

#[derive(Serialize, Deserialize)]
struct ImuCalibration {
    accel_bias: [f64; 3],
    accel_scale: [f64; 3],
    gyro_bias: [f64; 3],
    mag_offset: [f64; 3],
}

impl ImuCalibration {
    fn save(&self, path: &str) -> std::io::Result<()> {
        let data = serde_json::to_string_pretty(self)?;
        fs::write(path, data)
    }

    fn load(path: &str) -> std::io::Result<Self> {
        let data = fs::read_to_string(path)?;
        serde_json::from_str(&data)
            .map_err(|e| std::io::Error::new(std::io::ErrorKind::InvalidData, e))
    }
}
```

## Coordinate Frames

### IMU Frame Convention

The IMU uses a right-handed coordinate system:

- **X-axis**: Forward (robot front)
- **Y-axis**: Left (robot left side)
- **Z-axis**: Up (opposite to gravity)

### Rotation Convention

Rotations follow the right-hand rule:
- **Roll**: Rotation around X-axis (positive = lean right)
- **Pitch**: Rotation around Y-axis (positive = nose up)
- **Yaw**: Rotation around Z-axis (positive = turn left)

### Quaternion Orientation

Quaternions represent orientation as `[x, y, z, w]`:
- **Identity** (no rotation): `[0, 0, 0, 1]`
- **90° yaw** (left turn): `[0, 0, 0.707, 0.707]`

```rust
// Convert quaternion to Euler angles
let q = Quaternion::new(
    imu_data.orientation[0],
    imu_data.orientation[1],
    imu_data.orientation[2],
    imu_data.orientation[3]
);
let (roll, pitch, yaw) = q.to_euler();
```

### Gravity Vector

When the IMU is level:
- Acceleration: `[0, 0, -9.81]` m/s²
- The negative Z indicates gravity pulls down

### Frame Transformations

Transform IMU data to different frames:

```rust
use horus_library::Transform;

// Transform IMU from sensor frame to base frame
let imu_to_base = Transform::new(0.1, 0.0, 0.05, 0.0); // x, y, z, rotation

// Apply transform to acceleration
let accel_base = imu_to_base.transform_vector(
    imu_data.linear_acceleration_vec()
);
```

## Troubleshooting

### Issue: No IMU data published

**Cause**: IMU initialization failed or hardware connection issue

**Solution**:
```rust
// Check IMU initialization
let actual_rate = imu.get_actual_sample_rate();
if actual_rate == 0.0 {
    eprintln!("IMU not publishing data - check hardware connection");
}

// For I2C IMUs, verify:
// 1. I2C bus is enabled
// 2. Correct I2C address
// 3. Pull-up resistors present
// 4. Correct power supply voltage
```

### Issue: I2C communication errors

**Cause**: Bus contention, timing issues, or electrical problems

**Solution**:
```bash
# Check I2C devices
i2cdetect -y 1

# Verify IMU address (common: 0x68, 0x69)
# Check for address conflicts

# Adjust I2C bus speed if needed (in /boot/config.txt)
dtparam=i2c_arm_baudrate=100000  # Reduce to 100 kHz

# Verify pull-up resistors (1.8kΩ - 4.7kΩ)
```

### Issue: Noisy accelerometer data

**Cause**: Vibration, electromagnetic interference, or sensor noise

**Solution**:
```rust
// Apply low-pass filter
struct LowPassFilter {
    alpha: f64,  // 0.0-1.0, lower = more filtering
    prev: [f64; 3],
}

impl LowPassFilter {
    fn filter(&mut self, input: [f64; 3]) -> [f64; 3] {
        for i in 0..3 {
            self.prev[i] = self.alpha * input[i] + (1.0 - self.alpha) * self.prev[i];
        }
        self.prev
    }
}

// Use alpha = 0.1 to 0.3 for smoothing
let mut filter = LowPassFilter { alpha: 0.2, prev: [0.0; 3] };
let filtered = filter.filter(imu_data.linear_acceleration);
```

**Hardware solutions**:
- Mount IMU on vibration-dampening material
- Move IMU away from motors and power lines
- Add capacitors near IMU power supply
- Shield IMU from electromagnetic interference

### Issue: Gyroscope drift

**Cause**: Temperature changes, bias accumulation, or integration error

**Solution**:
```rust
// Periodic gyro bias recalibration
struct GyroDriftCompensation {
    bias: [f64; 3],
    stationary_threshold: f64,
    calibration_samples: Vec<[f64; 3]>,
}

impl GyroDriftCompensation {
    fn update(&mut self, gyro: [f64; 3], accel: [f64; 3]) {
        // Detect stationary state (low acceleration variation)
        let accel_mag = (accel[0].powi(2) + accel[1].powi(2) +
                        accel[2].powi(2)).sqrt();

        if (accel_mag - 9.81).abs() < self.stationary_threshold {
            // Robot is stationary, collect samples
            self.calibration_samples.push(gyro);

            if self.calibration_samples.len() >= 100 {
                // Update bias
                for i in 0..3 {
                    self.bias[i] = self.calibration_samples.iter()
                        .map(|s| s[i]).sum::<f64>() / 100.0;
                }
                self.calibration_samples.clear();
            }
        }
    }

    fn compensate(&self, gyro: [f64; 3]) -> [f64; 3] {
        [
            gyro[0] - self.bias[0],
            gyro[1] - self.bias[1],
            gyro[2] - self.bias[2],
        ]
    }
}
```

### Issue: Incorrect orientation

**Cause**: Wrong mounting orientation or coordinate frame mismatch

**Solution**:
```rust
// Remap axes based on physical mounting
fn remap_imu_axes(imu: &mut Imu, mounting: &str) {
    match mounting {
        "rotated_90_z" => {
            // IMU rotated 90° around Z axis
            let temp = imu.linear_acceleration[0];
            imu.linear_acceleration[0] = -imu.linear_acceleration[1];
            imu.linear_acceleration[1] = temp;

            let temp = imu.angular_velocity[0];
            imu.angular_velocity[0] = -imu.angular_velocity[1];
            imu.angular_velocity[1] = temp;
        },
        "upside_down" => {
            // IMU mounted upside down
            imu.linear_acceleration[0] = -imu.linear_acceleration[0];
            imu.linear_acceleration[1] = -imu.linear_acceleration[1];
            imu.angular_velocity[2] = -imu.angular_velocity[2];
        },
        _ => {}
    }
}
```

### Issue: Temperature-dependent drift

**Cause**: IMU sensor temperature coefficient

**Solution**:
```rust
// Temperature compensation (if IMU provides temperature)
struct TemperatureCompensation {
    reference_temp: f64,
    temp_coefficient: [f64; 3],
}

impl TemperatureCompensation {
    fn compensate(&self, gyro: [f64; 3], temp: f64) -> [f64; 3] {
        let temp_delta = temp - self.reference_temp;
        [
            gyro[0] - self.temp_coefficient[0] * temp_delta,
            gyro[1] - self.temp_coefficient[1] * temp_delta,
            gyro[2] - self.temp_coefficient[2] * temp_delta,
        ]
    }
}

// Or use IMUs with built-in temperature compensation (DMP)
```

### Issue: Magnetic interference

**Cause**: Nearby ferromagnetic materials or electromagnetic fields

**Solution**:
```rust
// Detect magnetic interference
fn detect_magnetic_anomaly(mag: [f64; 3], expected_field_strength: f64) -> bool {
    let magnitude = (mag[0].powi(2) + mag[1].powi(2) + mag[2].powi(2)).sqrt();
    let deviation = (magnitude - expected_field_strength).abs();

    // If deviation > 20% of expected, likely interference
    deviation > 0.2 * expected_field_strength
}

// Fallback to gyro-only orientation when interference detected
if detect_magnetic_anomaly(mag_data, 50.0) {
    eprintln!("Magnetic interference detected - using gyro-only mode");
    // Use complementary filter with accel + gyro only
}
```

**Hardware solutions**:
- Mount magnetometer away from motors, batteries, and metal
- Use external magnetometer on extension cable
- Shield magnetometer from interference sources
- Calibrate in final mounting location

## Integration with Localization

### Complementary Filter

Fuse IMU with other sensors for robust orientation:

```rust
use horus_library::{Imu, Odometry};

struct ComplementaryFilter {
    alpha: f64,  // Weight for gyro (0.95-0.98 typical)
    orientation: [f64; 4],  // Current orientation quaternion
}

impl ComplementaryFilter {
    fn update(&mut self, imu: &Imu, dt: f64) {
        // High-pass: Integrate gyro for short-term orientation
        let gyro_quat = self.integrate_gyro(imu.angular_velocity, dt);

        // Low-pass: Use accelerometer for long-term correction
        let accel_roll_pitch = self.accel_to_quaternion(imu.linear_acceleration);

        // Fuse with complementary filter
        self.orientation = self.quaternion_slerp(
            gyro_quat,
            accel_roll_pitch,
            1.0 - self.alpha
        );
    }

    fn integrate_gyro(&self, gyro: [f64; 3], dt: f64) -> [f64; 4] {
        // Quaternion integration from angular velocity
        let omega = (gyro[0].powi(2) + gyro[1].powi(2) + gyro[2].powi(2)).sqrt();

        if omega < 1e-6 {
            return self.orientation;
        }

        let half_omega = omega * dt / 2.0;
        let s = half_omega.sin() / omega;

        [
            self.orientation[0] + s * gyro[0],
            self.orientation[1] + s * gyro[1],
            self.orientation[2] + s * gyro[2],
            self.orientation[3] + half_omega.cos(),
        ]
    }

    fn accel_to_quaternion(&self, accel: [f64; 3]) -> [f64; 4] {
        // Convert acceleration to roll/pitch quaternion
        let roll = accel[1].atan2(accel[2]);
        let pitch = (-accel[0]).atan2(
            (accel[1].powi(2) + accel[2].powi(2)).sqrt()
        );

        let cr = (roll / 2.0).cos();
        let sr = (roll / 2.0).sin();
        let cp = (pitch / 2.0).cos();
        let sp = (pitch / 2.0).sin();

        [sr * cp, cr * sp, 0.0, cr * cp]
    }

    fn quaternion_slerp(&self, q1: [f64; 4], q2: [f64; 4], t: f64) -> [f64; 4] {
        // Spherical linear interpolation
        let dot = q1[0] * q2[0] + q1[1] * q2[1] + q1[2] * q2[2] + q1[3] * q2[3];
        let theta = dot.acos();
        let sin_theta = theta.sin();

        if sin_theta < 1e-6 {
            return q1;
        }

        let a = ((1.0 - t) * theta).sin() / sin_theta;
        let b = (t * theta).sin() / sin_theta;

        [
            a * q1[0] + b * q2[0],
            a * q1[1] + b * q2[1],
            a * q1[2] + b * q2[2],
            a * q1[3] + b * q2[3],
        ]
    }
}
```

### Extended Kalman Filter (EKF)

Advanced sensor fusion using EKF:

```rust
struct EkfImuFusion {
    // State: [position, velocity, orientation, gyro_bias, accel_bias]
    state: [f64; 16],
    // Covariance matrix
    covariance: [[f64; 16]; 16],
    // Process noise
    process_noise: [[f64; 16]; 16],
}

impl EkfImuFusion {
    fn predict(&mut self, imu: &Imu, dt: f64) {
        // Prediction step: propagate state using IMU
        // Update position from velocity
        self.state[0] += self.state[3] * dt;
        self.state[1] += self.state[4] * dt;
        self.state[2] += self.state[5] * dt;

        // Update velocity from acceleration (minus bias)
        let accel = [
            imu.linear_acceleration[0] - self.state[13],
            imu.linear_acceleration[1] - self.state[14],
            imu.linear_acceleration[2] - self.state[15],
        ];

        self.state[3] += accel[0] * dt;
        self.state[4] += accel[1] * dt;
        self.state[5] += accel[2] * dt;

        // Update orientation from gyro (minus bias)
        let gyro = [
            imu.angular_velocity[0] - self.state[10],
            imu.angular_velocity[1] - self.state[11],
            imu.angular_velocity[2] - self.state[12],
        ];

        // Integrate gyro to update orientation quaternion
        // ... (quaternion integration)

        // Update covariance
        // P = F * P * F^T + Q
        // ... (covariance propagation)
    }

    fn update(&mut self, measurement: &Odometry) {
        // Correction step: fuse with odometry or other sensors
        // K = P * H^T * (H * P * H^T + R)^-1
        // state = state + K * (z - h(state))
        // P = (I - K * H) * P
        // ... (measurement update)
    }
}
```

### Dead Reckoning with IMU

Use IMU for position estimation when other sensors unavailable:

```rust
struct ImuDeadReckoning {
    position: [f64; 3],
    velocity: [f64; 3],
    orientation: [f64; 4],
    last_update: u64,
}

impl ImuDeadReckoning {
    fn update(&mut self, imu: &Imu) {
        let dt = (imu.timestamp - self.last_update) as f64 / 1e9;

        // Rotate acceleration to world frame
        let world_accel = self.rotate_by_quaternion(
            imu.linear_acceleration,
            self.orientation
        );

        // Remove gravity
        let accel_no_gravity = [
            world_accel[0],
            world_accel[1],
            world_accel[2] - 9.81,
        ];

        // Integrate acceleration to velocity
        self.velocity[0] += accel_no_gravity[0] * dt;
        self.velocity[1] += accel_no_gravity[1] * dt;
        self.velocity[2] += accel_no_gravity[2] * dt;

        // Integrate velocity to position
        self.position[0] += self.velocity[0] * dt;
        self.position[1] += self.velocity[1] * dt;
        self.position[2] += self.velocity[2] * dt;

        // Update orientation from gyro
        self.orientation = self.integrate_gyro(imu.angular_velocity, dt);

        self.last_update = imu.timestamp;
    }

    fn rotate_by_quaternion(&self, v: [f64; 3], q: [f64; 4]) -> [f64; 3] {
        // Rotate vector by quaternion: v' = q * v * q^-1
        let qx = q[0];
        let qy = q[1];
        let qz = q[2];
        let qw = q[3];

        let x = v[0];
        let y = v[1];
        let z = v[2];

        // Optimized quaternion-vector multiplication
        let ix =  qw * x + qy * z - qz * y;
        let iy =  qw * y + qz * x - qx * z;
        let iz =  qw * z + qx * y - qy * x;
        let iw = -qx * x - qy * y - qz * z;

        [
            ix * qw + iw * -qx + iy * -qz - iz * -qy,
            iy * qw + iw * -qy + iz * -qx - ix * -qz,
            iz * qw + iw * -qz + ix * -qy - iy * -qx,
        ]
    }

    fn integrate_gyro(&self, gyro: [f64; 3], dt: f64) -> [f64; 4] {
        // Similar to complementary filter integration
        // ... (quaternion integration from gyro)
        self.orientation  // Simplified
    }
}

// Note: IMU dead reckoning accumulates error rapidly
// Use only for short durations between GPS or other absolute position updates
```

### Sensor Fusion Pipeline

Complete example integrating IMU with multiple sensors:

```rust
use horus_library::{Imu, Odometry, Pose2D};
use horus_core::Hub;

struct SensorFusionNode {
    imu_sub: Hub<Imu>,
    odom_sub: Hub<Odometry>,
    pose_pub: Hub<Pose2D>,

    complementary_filter: ComplementaryFilter,
    last_imu_time: u64,
}

impl SensorFusionNode {
    fn new() -> Result<Self, Box<dyn std::error::Error>> {
        Ok(Self {
            imu_sub: Hub::subscribe("imu")?,
            odom_sub: Hub::subscribe("odom")?,
            pose_pub: Hub::new("fused_pose")?,
            complementary_filter: ComplementaryFilter {
                alpha: 0.98,
                orientation: [0.0, 0.0, 0.0, 1.0],
            },
            last_imu_time: 0,
        })
    }

    fn tick(&mut self) {
        // Get latest IMU data
        if let Ok(imu) = self.imu_sub.try_recv() {
            let dt = if self.last_imu_time > 0 {
                (imu.timestamp - self.last_imu_time) as f64 / 1e9
            } else {
                0.01  // Default 10ms
            };

            // Update orientation estimate
            self.complementary_filter.update(&imu, dt);
            self.last_imu_time = imu.timestamp;
        }

        // Get odometry and fuse with IMU orientation
        if let Ok(mut odom) = self.odom_sub.try_recv() {
            // Extract yaw from fused orientation
            let q = self.complementary_filter.orientation;
            let yaw = (2.0 * (q[3] * q[2] + q[0] * q[1]))
                .atan2(1.0 - 2.0 * (q[1].powi(2) + q[2].powi(2)));

            // Create fused pose
            let fused_pose = Pose2D::new(odom.pose.x, odom.pose.y, yaw);

            let _ = self.pose_pub.send(fused_pose, None);
        }
    }
}
```

## Performance Considerations

### Sample Rate Selection

Choose sample rate based on application:

- **Orientation tracking**: 50-100 Hz sufficient
- **High-speed motion**: 200-500 Hz recommended
- **Vibration analysis**: 500-1000 Hz required
- **Low-power applications**: 10-50 Hz acceptable

```rust
// Adjust sample rate for different modes
imu.set_sample_rate(100.0);  // Normal operation
imu.set_sample_rate(10.0);   // Power saving mode
imu.set_sample_rate(500.0);  // High-speed tracking
```

### CPU Usage

IMU node CPU usage is minimal:
- Reading sensor: ~0.1ms per sample
- Publishing message: ~0.05ms
- At 100 Hz: ~1.5% CPU on typical embedded system

### Memory Usage

Fixed memory footprint:
- Node state: ~200 bytes
- Message: 200 bytes
- Total: <1 KB

### Latency

End-to-end latency (sensor to subscriber):
- I2C read: 0.1-1ms
- Message publish: <0.1ms
- **Total**: 0.2-1.2ms typical

## Related Nodes

- **LocalizationNode**: Uses IMU data for pose estimation
- **DifferentialDriveNode**: Subscribes to IMU for heading correction
- **ComplementaryFilterNode**: Fuses IMU with other sensors
- **KalmanFilterNode**: Advanced sensor fusion with IMU

## See Also

- [IMU Sensor Technology](https://en.wikipedia.org/wiki/Inertial_measurement_unit)
- [Quaternion Kinematics](https://en.wikipedia.org/wiki/Quaternion)
- [Complementary Filter](https://en.wikipedia.org/wiki/Complementary_filter)
- [Extended Kalman Filter](https://en.wikipedia.org/wiki/Extended_Kalman_filter)
- [Sensor Fusion Techniques](https://en.wikipedia.org/wiki/Sensor_fusion)
