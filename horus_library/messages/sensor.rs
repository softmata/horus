use horus_core::core::LogSummary;
use horus_macros::LogSummary;
// Sensor data message types for robotics
//
// This module provides standard sensor data formats for common
// robotics sensors including lidar, IMU, cameras, and odometry.

use crate::messages::geometry::{Pose2D, Quaternion, Twist, Vector3};
use serde::{Deserialize, Serialize};
use serde_arrays;

/// Laser scan data from a 2D lidar sensor
///
/// Fixed-size array for shared memory safety. Supports up to 360-degree
/// scanning with 1-degree resolution.
#[derive(Debug, Clone, Serialize, Deserialize, LogSummary)]
pub struct LaserScan {
    /// Range measurements in meters (0 = invalid reading)
    #[serde(with = "serde_arrays")]
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
    pub timestamp_ns: u64,
}

impl Default for LaserScan {
    fn default() -> Self {
        Self {
            ranges: [0.0; 360],
            angle_min: -std::f32::consts::PI,
            angle_max: std::f32::consts::PI,
            range_min: 0.1,
            range_max: 30.0,
            angle_increment: std::f32::consts::PI / 180.0,
            time_increment: 0.0,
            scan_time: 0.1,
            timestamp_ns: 0,
        }
    }
}

impl LaserScan {
    /// Create a new laser scan with default parameters
    pub fn new() -> Self {
        Self {
            timestamp_ns: std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap_or_default()
                .as_nanos() as u64,
            ..Self::default()
        }
    }

    /// Get the angle for a specific range index
    pub fn angle_at(&self, index: usize) -> f32 {
        if index >= 360 {
            return 0.0;
        }
        self.angle_min + (index as f32) * self.angle_increment
    }

    /// Check if a range reading is valid
    pub fn is_range_valid(&self, index: usize) -> bool {
        if index >= 360 {
            return false;
        }
        let range = self.ranges[index];
        range >= self.range_min && range <= self.range_max && range.is_finite()
    }

    /// Count valid range readings
    pub fn valid_count(&self) -> usize {
        self.ranges
            .iter()
            .filter(|&&r| r >= self.range_min && r <= self.range_max && r.is_finite())
            .count()
    }

    /// Get minimum valid range reading
    pub fn min_range(&self) -> Option<f32> {
        self.ranges
            .iter()
            .filter(|&&r| r >= self.range_min && r <= self.range_max && r.is_finite())
            .min_by(|a, b| a.partial_cmp(b).unwrap())
            .copied()
    }
}

/// IMU (Inertial Measurement Unit) sensor data
///
/// Provides orientation, angular velocity, and linear acceleration
/// measurements from an IMU sensor.
#[derive(Debug, Clone, Copy, Serialize, Deserialize, Default, LogSummary)]
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
    pub timestamp_ns: u64,
}

impl Imu {
    /// Create a new IMU message
    pub fn new() -> Self {
        Self {
            orientation: [0.0, 0.0, 0.0, 1.0], // Identity quaternion
            orientation_covariance: [-1.0; 9], // No orientation data
            angular_velocity: [0.0; 3],
            angular_velocity_covariance: [0.0; 9],
            linear_acceleration: [0.0; 3],
            linear_acceleration_covariance: [0.0; 9],
            timestamp_ns: std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap_or_default()
                .as_nanos() as u64,
        }
    }

    /// Set orientation from Euler angles
    pub fn set_orientation_from_euler(&mut self, roll: f64, pitch: f64, yaw: f64) {
        let q = Quaternion::from_euler(roll, pitch, yaw);
        self.orientation = [q.x, q.y, q.z, q.w];
    }

    /// Check if orientation data is available
    pub fn has_orientation(&self) -> bool {
        self.orientation_covariance[0] >= 0.0
    }

    /// Check if all values are finite
    pub fn is_valid(&self) -> bool {
        self.orientation.iter().all(|v| v.is_finite())
            && self.angular_velocity.iter().all(|v| v.is_finite())
            && self.linear_acceleration.iter().all(|v| v.is_finite())
    }

    /// Get angular velocity as Vector3
    pub fn angular_velocity_vec(&self) -> Vector3 {
        Vector3::new(
            self.angular_velocity[0],
            self.angular_velocity[1],
            self.angular_velocity[2],
        )
    }

    /// Get linear acceleration as Vector3
    pub fn linear_acceleration_vec(&self) -> Vector3 {
        Vector3::new(
            self.linear_acceleration[0],
            self.linear_acceleration[1],
            self.linear_acceleration[2],
        )
    }
}

/// Odometry data combining pose and velocity
///
/// Typically computed from wheel encoders or visual odometry,
/// provides the robot's estimated position and velocity.
#[derive(Debug, Clone, Copy, Serialize, Deserialize, LogSummary)]
pub struct Odometry {
    /// Current pose estimate
    pub pose: Pose2D,
    /// Current velocity estimate
    pub twist: Twist,
    /// Pose covariance matrix (6x6 row-major)
    #[serde(with = "serde_arrays")]
    pub pose_covariance: [f64; 36],
    /// Twist covariance matrix (6x6 row-major)
    #[serde(with = "serde_arrays")]
    pub twist_covariance: [f64; 36],
    /// Frame ID for the pose (e.g., "odom", "map")
    pub frame_id: [u8; 32],
    /// Frame ID for the twist (e.g., "base_link")
    pub child_frame_id: [u8; 32],
    /// Timestamp in nanoseconds since epoch
    pub timestamp_ns: u64,
}

impl Default for Odometry {
    fn default() -> Self {
        Self {
            pose: Pose2D::default(),
            twist: Twist::default(),
            pose_covariance: [0.0; 36],
            twist_covariance: [0.0; 36],
            frame_id: [0; 32],
            child_frame_id: [0; 32],
            timestamp_ns: 0,
        }
    }
}

impl Odometry {
    /// Create a new odometry message
    pub fn new() -> Self {
        Self {
            pose: Pose2D::origin(),
            twist: Twist::stop(),
            pose_covariance: [0.0; 36],
            twist_covariance: [0.0; 36],
            frame_id: [0; 32],
            child_frame_id: [0; 32],
            timestamp_ns: std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap_or_default()
                .as_nanos() as u64,
        }
    }

    /// Set frame IDs from strings
    pub fn set_frames(&mut self, frame: &str, child_frame: &str) {
        // Copy frame_id string
        let frame_bytes = frame.as_bytes();
        let len = frame_bytes.len().min(31);
        self.frame_id[..len].copy_from_slice(&frame_bytes[..len]);
        self.frame_id[len] = 0; // Null terminator

        // Copy child_frame_id string
        let child_bytes = child_frame.as_bytes();
        let len = child_bytes.len().min(31);
        self.child_frame_id[..len].copy_from_slice(&child_bytes[..len]);
        self.child_frame_id[len] = 0; // Null terminator
    }

    /// Update pose and velocity
    pub fn update(&mut self, pose: Pose2D, twist: Twist) {
        self.pose = pose;
        self.twist = twist;
        self.timestamp_ns = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap_or_default()
            .as_nanos() as u64;
    }

    /// Check if values are valid
    pub fn is_valid(&self) -> bool {
        self.pose.is_valid() && self.twist.is_valid()
    }
}

/// Range sensor data (ultrasonic, infrared, etc.)
///
/// Single-point distance measurement from sensors like
/// ultrasonic or infrared rangers.
#[derive(Debug, Clone, Copy, Serialize, Deserialize, Default, LogSummary)]
pub struct Range {
    /// Sensor type (0=ultrasonic, 1=infrared)
    pub sensor_type: u8,
    /// Field of view in radians
    pub field_of_view: f32,
    /// Minimum range in meters
    pub min_range: f32,
    /// Maximum range in meters
    pub max_range: f32,
    /// Range reading in meters
    pub range: f32,
    /// Timestamp in nanoseconds since epoch
    pub timestamp_ns: u64,
}

impl Range {
    pub const ULTRASONIC: u8 = 0;
    pub const INFRARED: u8 = 1;

    /// Create a new range message
    pub fn new(sensor_type: u8, range: f32) -> Self {
        Self {
            sensor_type,
            field_of_view: 0.1, // ~6 degrees
            min_range: 0.02,
            max_range: 4.0,
            range,
            timestamp_ns: std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap_or_default()
                .as_nanos() as u64,
        }
    }

    /// Check if the range reading is valid
    pub fn is_valid(&self) -> bool {
        self.range >= self.min_range && self.range <= self.max_range && self.range.is_finite()
    }
}

/// Battery status message
#[derive(Debug, Clone, Copy, Serialize, Deserialize, LogSummary)]
pub struct BatteryState {
    /// Voltage in volts
    pub voltage: f32,
    /// Current in amperes (negative = discharging)
    pub current: f32,
    /// Charge in amp-hours (NaN if unknown)
    pub charge: f32,
    /// Capacity in amp-hours (NaN if unknown)
    pub capacity: f32,
    /// Percentage charge (0-100)
    pub percentage: f32,
    /// Power supply status (0=unknown, 1=charging, 2=discharging, 3=full)
    pub power_supply_status: u8,
    /// Temperature in celsius
    pub temperature: f32,
    /// Cell voltages if available
    #[serde(with = "serde_arrays")]
    pub cell_voltages: [f32; 16],
    /// Number of valid cell voltage readings
    pub cell_count: u8,
    /// Timestamp in nanoseconds since epoch
    pub timestamp_ns: u64,
}

impl Default for BatteryState {
    fn default() -> Self {
        Self {
            voltage: 0.0,
            current: 0.0,
            charge: f32::NAN,
            capacity: f32::NAN,
            percentage: 0.0,
            power_supply_status: Self::STATUS_UNKNOWN,
            temperature: 25.0,
            cell_voltages: [0.0; 16],
            cell_count: 0,
            timestamp_ns: 0,
        }
    }
}

impl BatteryState {
    pub const STATUS_UNKNOWN: u8 = 0;
    pub const STATUS_CHARGING: u8 = 1;
    pub const STATUS_DISCHARGING: u8 = 2;
    pub const STATUS_FULL: u8 = 3;

    /// Create a new battery state message
    pub fn new(voltage: f32, percentage: f32) -> Self {
        Self {
            voltage,
            current: 0.0,
            charge: f32::NAN,
            capacity: f32::NAN,
            percentage,
            power_supply_status: Self::STATUS_UNKNOWN,
            temperature: 25.0,
            cell_voltages: [0.0; 16],
            cell_count: 0,
            timestamp_ns: std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap_or_default()
                .as_nanos() as u64,
        }
    }

    /// Check if battery is low (below threshold)
    pub fn is_low(&self, threshold: f32) -> bool {
        self.percentage < threshold
    }

    /// Check if battery is critical (below 10%)
    pub fn is_critical(&self) -> bool {
        self.percentage < 10.0
    }

    /// Estimate remaining time in seconds (negative current only)
    pub fn time_remaining(&self) -> Option<f32> {
        if self.current < 0.0 && !self.charge.is_nan() {
            Some((self.charge / -self.current) * 3600.0)
        } else {
            None
        }
    }
}

/// GPS/GNSS Position Data
///
/// Standard GNSS position data from GPS, GLONASS, Galileo, or other
/// satellite navigation systems. Provides latitude, longitude, altitude,
/// and accuracy information.
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct NavSatFix {
    /// Latitude in degrees (positive = North, negative = South)
    pub latitude: f64,
    /// Longitude in degrees (positive = East, negative = West)
    pub longitude: f64,
    /// Altitude in meters above WGS84 ellipsoid
    pub altitude: f64,
    /// Position covariance matrix [lat, lon, alt] diagonal
    pub position_covariance: [f64; 9],
    /// Covariance type (0=unknown, 1=approximated, 2=diagonal_known, 3=known)
    pub position_covariance_type: u8,
    /// Satellite fix status (0=no_fix, 1=fix, 2=sbas_fix, 3=gbas_fix)
    pub status: u8,
    /// Number of satellites visible
    pub satellites_visible: u16,
    /// HDOP (Horizontal dilution of precision)
    pub hdop: f32,
    /// VDOP (Vertical dilution of precision)
    pub vdop: f32,
    /// Ground speed in m/s
    pub speed: f32,
    /// Course/heading in degrees
    pub heading: f32,
    /// Timestamp in nanoseconds since epoch
    pub timestamp_ns: u64,
}

impl Default for NavSatFix {
    fn default() -> Self {
        Self {
            latitude: 0.0,
            longitude: 0.0,
            altitude: 0.0,
            position_covariance: [0.0; 9],
            position_covariance_type: Self::COVARIANCE_TYPE_UNKNOWN,
            status: Self::STATUS_NO_FIX,
            satellites_visible: 0,
            hdop: 99.9,
            vdop: 99.9,
            speed: 0.0,
            heading: 0.0,
            timestamp_ns: 0,
        }
    }
}

impl NavSatFix {
    // Status constants
    pub const STATUS_NO_FIX: u8 = 0;
    pub const STATUS_FIX: u8 = 1;
    pub const STATUS_SBAS_FIX: u8 = 2;
    pub const STATUS_GBAS_FIX: u8 = 3;

    // Covariance type constants
    pub const COVARIANCE_TYPE_UNKNOWN: u8 = 0;
    pub const COVARIANCE_TYPE_APPROXIMATED: u8 = 1;
    pub const COVARIANCE_TYPE_DIAGONAL_KNOWN: u8 = 2;
    pub const COVARIANCE_TYPE_KNOWN: u8 = 3;

    /// Create a new GPS fix message
    pub fn new() -> Self {
        Self {
            timestamp_ns: std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap_or_default()
                .as_nanos() as u64,
            ..Self::default()
        }
    }

    /// Create from lat/lon/alt
    pub fn from_coordinates(lat: f64, lon: f64, alt: f64) -> Self {
        let mut fix = Self::new();
        fix.latitude = lat;
        fix.longitude = lon;
        fix.altitude = alt;
        fix.status = Self::STATUS_FIX;
        fix
    }

    /// Check if we have a valid GPS fix
    pub fn has_fix(&self) -> bool {
        self.status >= Self::STATUS_FIX
    }

    /// Check if coordinates are valid
    pub fn is_valid(&self) -> bool {
        self.latitude >= -90.0
            && self.latitude <= 90.0
            && self.longitude >= -180.0
            && self.longitude <= 180.0
            && self.latitude.is_finite()
            && self.longitude.is_finite()
            && self.altitude.is_finite()
    }

    /// Calculate approximate accuracy in meters from HDOP
    pub fn horizontal_accuracy(&self) -> f32 {
        // Rough approximation: accuracy ≈ HDOP * 5m
        self.hdop * 5.0
    }

    /// Calculate distance to another GPS position in meters (Haversine formula)
    pub fn distance_to(&self, other: &NavSatFix) -> f64 {
        const EARTH_RADIUS: f64 = 6371000.0; // meters

        let lat1 = self.latitude.to_radians();
        let lat2 = other.latitude.to_radians();
        let delta_lat = (other.latitude - self.latitude).to_radians();
        let delta_lon = (other.longitude - self.longitude).to_radians();

        let a = (delta_lat / 2.0).sin().powi(2)
            + lat1.cos() * lat2.cos() * (delta_lon / 2.0).sin().powi(2);
        let c = 2.0 * a.sqrt().atan2((1.0 - a).sqrt());

        EARTH_RADIUS * c
    }
}

impl LogSummary for NavSatFix {
    fn log_summary(&self) -> String {
        format!(
            "GPS: lat={:.6}, lon={:.6}, alt={:.1}m, sats={}, fix={}",
            self.latitude, self.longitude, self.altitude, self.satellites_visible, self.status
        )
    }
}

