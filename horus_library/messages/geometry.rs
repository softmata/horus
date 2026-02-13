use horus_macros::LogSummary;
// Geometric and spatial message types for robotics
//
// This module provides fundamental geometric primitives used throughout
// robotics applications for representing position, orientation, and motion.

use serde::{Deserialize, Serialize};

/// 3D velocity command with linear and angular components
///
/// Used for commanding robot motion in 3D space. For 2D robots,
/// only x (forward) and yaw (rotation) are typically used.
///
/// Implements `PodMessage` for ultra-fast zero-serialization transfer (~50ns).
#[repr(C)]
#[derive(Debug, Clone, Copy, Serialize, Deserialize, Default, LogSummary)]
pub struct Twist {
    /// Linear velocity [x, y, z] in m/s
    pub linear: [f64; 3],
    /// Angular velocity [roll, pitch, yaw] in rad/s
    pub angular: [f64; 3],
    /// Timestamp in nanoseconds since epoch
    pub timestamp_ns: u64,
}

impl Twist {
    /// Create a new Twist message
    pub fn new(linear: [f64; 3], angular: [f64; 3]) -> Self {
        Self {
            linear,
            angular,
            timestamp_ns: std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap_or_default()
                .as_nanos() as u64,
        }
    }

    /// Create a 2D twist (forward velocity and rotation)
    pub fn new_2d(linear_x: f64, angular_z: f64) -> Self {
        Self::new([linear_x, 0.0, 0.0], [0.0, 0.0, angular_z])
    }

    /// Stop command (all zeros)
    pub fn stop() -> Self {
        Self::new([0.0; 3], [0.0; 3])
    }

    /// Check if all values are finite
    pub fn is_valid(&self) -> bool {
        self.linear.iter().all(|v| v.is_finite()) && self.angular.iter().all(|v| v.is_finite())
    }
}

/// 2D pose representation (position and orientation)
///
/// Commonly used for mobile robots operating in planar environments.
///
/// Implements `PodMessage` for ultra-fast zero-serialization transfer (~50ns).
#[repr(C)]
#[derive(Debug, Clone, Copy, Serialize, Deserialize, Default, LogSummary)]
pub struct Pose2D {
    /// X position in meters
    pub x: f64,
    /// Y position in meters
    pub y: f64,
    /// Orientation angle in radians
    pub theta: f64,
    /// Timestamp in nanoseconds since epoch
    pub timestamp_ns: u64,
}

impl Pose2D {
    /// Create a new 2D pose
    pub fn new(x: f64, y: f64, theta: f64) -> Self {
        Self {
            x,
            y,
            theta,
            timestamp_ns: std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap_or_default()
                .as_nanos() as u64,
        }
    }

    /// Create pose at origin
    pub fn origin() -> Self {
        Self::new(0.0, 0.0, 0.0)
    }

    /// Calculate euclidean distance to another pose
    pub fn distance_to(&self, other: &Pose2D) -> f64 {
        let dx = self.x - other.x;
        let dy = self.y - other.y;
        (dx * dx + dy * dy).sqrt()
    }

    /// Normalize theta to [-pi, pi]
    pub fn normalize_angle(&mut self) {
        while self.theta > std::f64::consts::PI {
            self.theta -= 2.0 * std::f64::consts::PI;
        }
        while self.theta < -std::f64::consts::PI {
            self.theta += 2.0 * std::f64::consts::PI;
        }
    }

    /// Check if values are finite
    pub fn is_valid(&self) -> bool {
        self.x.is_finite() && self.y.is_finite() && self.theta.is_finite()
    }
}

/// 3D transformation (translation and rotation)
///
/// Represents a full 3D transformation using translation vector and
/// quaternion rotation. Used for coordinate frame transformations.
///
/// Implements `PodMessage` for ultra-fast zero-serialization transfer (~50ns).
#[repr(C)]
#[derive(Debug, Clone, Copy, Serialize, Deserialize, Default, LogSummary)]
pub struct TransformStamped {
    /// Translation [x, y, z] in meters
    pub translation: [f64; 3],
    /// Rotation as quaternion [x, y, z, w]
    pub rotation: [f64; 4],
    /// Timestamp in nanoseconds since epoch
    pub timestamp_ns: u64,
}

impl TransformStamped {
    /// Create a new transform
    pub fn new(translation: [f64; 3], rotation: [f64; 4]) -> Self {
        Self {
            translation,
            rotation,
            timestamp_ns: std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap_or_default()
                .as_nanos() as u64,
        }
    }

    /// Identity transform (no translation or rotation)
    pub fn identity() -> Self {
        Self::new([0.0; 3], [0.0, 0.0, 0.0, 1.0])
    }

    /// Create from 2D pose (z=0, only yaw rotation)
    pub fn from_pose_2d(pose: &Pose2D) -> Self {
        let half_theta = pose.theta / 2.0;
        Self::new(
            [pose.x, pose.y, 0.0],
            [0.0, 0.0, half_theta.sin(), half_theta.cos()],
        )
    }

    /// Check if quaternion is normalized and values are finite
    pub fn is_valid(&self) -> bool {
        // Check finite values
        if !self.translation.iter().all(|v| v.is_finite())
            || !self.rotation.iter().all(|v| v.is_finite())
        {
            return false;
        }

        // Check quaternion normalization
        let norm = self.rotation.iter().map(|v| v * v).sum::<f64>().sqrt();
        (norm - 1.0).abs() < 0.01
    }

    /// Normalize the quaternion component
    pub fn normalize_rotation(&mut self) {
        let norm = self.rotation.iter().map(|v| v * v).sum::<f64>().sqrt();
        if norm > 0.0 {
            for v in &mut self.rotation {
                *v /= norm;
            }
        }
    }
}

/// 3D point representation
///
/// Implements `PodMessage` for ultra-fast zero-serialization transfer (~50ns).
#[repr(C)]
#[derive(Debug, Clone, Copy, Serialize, Deserialize, Default, LogSummary)]
pub struct Point3 {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

impl Point3 {
    pub fn new(x: f64, y: f64, z: f64) -> Self {
        Self { x, y, z }
    }

    pub fn origin() -> Self {
        Self::new(0.0, 0.0, 0.0)
    }

    pub fn distance_to(&self, other: &Point3) -> f64 {
        let dx = self.x - other.x;
        let dy = self.y - other.y;
        let dz = self.z - other.z;
        (dx * dx + dy * dy + dz * dz).sqrt()
    }
}

/// 3D vector representation
///
/// Implements `PodMessage` for ultra-fast zero-serialization transfer (~50ns).
#[repr(C)]
#[derive(Debug, Clone, Copy, Serialize, Deserialize, Default, LogSummary)]
pub struct Vector3 {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

impl Vector3 {
    pub fn new(x: f64, y: f64, z: f64) -> Self {
        Self { x, y, z }
    }

    pub fn zero() -> Self {
        Self::new(0.0, 0.0, 0.0)
    }

    pub fn magnitude(&self) -> f64 {
        (self.x * self.x + self.y * self.y + self.z * self.z).sqrt()
    }

    pub fn normalize(&mut self) {
        let mag = self.magnitude();
        if mag > 0.0 {
            self.x /= mag;
            self.y /= mag;
            self.z /= mag;
        }
    }

    pub fn dot(&self, other: &Vector3) -> f64 {
        self.x * other.x + self.y * other.y + self.z * other.z
    }

    pub fn cross(&self, other: &Vector3) -> Vector3 {
        Vector3::new(
            self.y * other.z - self.z * other.y,
            self.z * other.x - self.x * other.z,
            self.x * other.y - self.y * other.x,
        )
    }
}

/// Quaternion for 3D rotation representation
///
/// Implements `PodMessage` for ultra-fast zero-serialization transfer (~50ns).
#[repr(C)]
#[derive(Debug, Clone, Copy, Serialize, Deserialize, LogSummary)]
pub struct Quaternion {
    pub x: f64,
    pub y: f64,
    pub z: f64,
    pub w: f64,
}

impl Default for Quaternion {
    fn default() -> Self {
        Self::identity()
    }
}

impl Quaternion {
    pub fn new(x: f64, y: f64, z: f64, w: f64) -> Self {
        Self { x, y, z, w }
    }

    pub fn identity() -> Self {
        Self::new(0.0, 0.0, 0.0, 1.0)
    }

    pub fn from_euler(roll: f64, pitch: f64, yaw: f64) -> Self {
        let cr = (roll / 2.0).cos();
        let sr = (roll / 2.0).sin();
        let cp = (pitch / 2.0).cos();
        let sp = (pitch / 2.0).sin();
        let cy = (yaw / 2.0).cos();
        let sy = (yaw / 2.0).sin();

        Self {
            x: sr * cp * cy - cr * sp * sy,
            y: cr * sp * cy + sr * cp * sy,
            z: cr * cp * sy - sr * sp * cy,
            w: cr * cp * cy + sr * sp * sy,
        }
    }

    pub fn normalize(&mut self) {
        let norm = (self.x * self.x + self.y * self.y + self.z * self.z + self.w * self.w).sqrt();
        if norm > 0.0 {
            self.x /= norm;
            self.y /= norm;
            self.z /= norm;
            self.w /= norm;
        }
    }

    pub fn is_valid(&self) -> bool {
        self.x.is_finite() && self.y.is_finite() && self.z.is_finite() && self.w.is_finite()
    }
}

// =============================================================================
// POD (Plain Old Data) Message Support
// =============================================================================
// These implementations enable ultra-fast zero-serialization transfer (~50ns)
// for real-time robotics control loops. Topic automatically uses POD backend.

// Bytemuck implementations for safe byte casting
unsafe impl horus_core::bytemuck::Pod for Twist {}
unsafe impl horus_core::bytemuck::Zeroable for Twist {}
unsafe impl horus_core::communication::PodMessage for Twist {}

unsafe impl horus_core::bytemuck::Pod for Pose2D {}
unsafe impl horus_core::bytemuck::Zeroable for Pose2D {}
unsafe impl horus_core::communication::PodMessage for Pose2D {}

unsafe impl horus_core::bytemuck::Pod for TransformStamped {}
unsafe impl horus_core::bytemuck::Zeroable for TransformStamped {}
unsafe impl horus_core::communication::PodMessage for TransformStamped {}

unsafe impl horus_core::bytemuck::Pod for Point3 {}
unsafe impl horus_core::bytemuck::Zeroable for Point3 {}
unsafe impl horus_core::communication::PodMessage for Point3 {}

unsafe impl horus_core::bytemuck::Pod for Vector3 {}
unsafe impl horus_core::bytemuck::Zeroable for Vector3 {}
unsafe impl horus_core::communication::PodMessage for Vector3 {}

unsafe impl horus_core::bytemuck::Pod for Quaternion {}
unsafe impl horus_core::bytemuck::Zeroable for Quaternion {}
unsafe impl horus_core::communication::PodMessage for Quaternion {}

// Note: POD types are now auto-detected via needs_drop, no registration needed

#[cfg(test)]
mod tests {
    use super::*;
    use horus_core::core::LogSummary;
    use std::f64::consts::PI;

    // ============================================================================
    // Twist Tests
    // ============================================================================

    #[test]
    fn test_twist_new() {
        let twist = Twist::new([1.0, 2.0, 3.0], [0.1, 0.2, 0.3]);
        assert_eq!(twist.linear[0], 1.0);
        assert_eq!(twist.linear[1], 2.0);
        assert_eq!(twist.linear[2], 3.0);
        assert_eq!(twist.angular[0], 0.1);
        assert_eq!(twist.angular[1], 0.2);
        assert_eq!(twist.angular[2], 0.3);
        assert!(twist.timestamp_ns > 0);
    }

    #[test]
    fn test_twist_new_2d() {
        let twist = Twist::new_2d(1.5, 0.5);
        assert_eq!(twist.linear[0], 1.5);
        assert_eq!(twist.linear[1], 0.0);
        assert_eq!(twist.linear[2], 0.0);
        assert_eq!(twist.angular[0], 0.0);
        assert_eq!(twist.angular[1], 0.0);
        assert_eq!(twist.angular[2], 0.5);
    }

    #[test]
    fn test_twist_stop() {
        let twist = Twist::stop();
        assert_eq!(twist.linear, [0.0, 0.0, 0.0]);
        assert_eq!(twist.angular, [0.0, 0.0, 0.0]);
    }

    #[test]
    fn test_twist_is_valid() {
        let valid = Twist::new([1.0, 2.0, 3.0], [0.1, 0.2, 0.3]);
        assert!(valid.is_valid());

        let invalid = Twist::new([f64::INFINITY, 0.0, 0.0], [0.0; 3]);
        assert!(!invalid.is_valid());

        let nan = Twist::new([f64::NAN, 0.0, 0.0], [0.0; 3]);
        assert!(!nan.is_valid());
    }

    #[test]
    fn test_twist_serialization() {
        let twist = Twist::new([1.0, 2.0, 3.0], [0.1, 0.2, 0.3]);
        let serialized = serde_json::to_string(&twist).unwrap();
        let deserialized: Twist = serde_json::from_str(&serialized).unwrap();
        assert_eq!(twist.linear, deserialized.linear);
        assert_eq!(twist.angular, deserialized.angular);
    }

    // ============================================================================
    // Pose2D Tests
    // ============================================================================

    #[test]
    fn test_pose2d_new() {
        let pose = Pose2D::new(1.0, 2.0, 0.5);
        assert_eq!(pose.x, 1.0);
        assert_eq!(pose.y, 2.0);
        assert_eq!(pose.theta, 0.5);
        assert!(pose.timestamp_ns > 0);
    }

    #[test]
    fn test_pose2d_origin() {
        let pose = Pose2D::origin();
        assert_eq!(pose.x, 0.0);
        assert_eq!(pose.y, 0.0);
        assert_eq!(pose.theta, 0.0);
    }

    #[test]
    fn test_pose2d_distance_to() {
        let p1 = Pose2D::new(0.0, 0.0, 0.0);
        let p2 = Pose2D::new(3.0, 4.0, 0.0);
        assert!((p1.distance_to(&p2) - 5.0).abs() < 1e-10);
    }

    #[test]
    fn test_pose2d_normalize_angle() {
        let mut pose = Pose2D::new(0.0, 0.0, 3.0 * PI);
        pose.normalize_angle();
        assert!(pose.theta >= -PI && pose.theta <= PI);

        let mut pose2 = Pose2D::new(0.0, 0.0, -3.0 * PI);
        pose2.normalize_angle();
        assert!(pose2.theta >= -PI && pose2.theta <= PI);
    }

    #[test]
    fn test_pose2d_is_valid() {
        let valid = Pose2D::new(1.0, 2.0, 0.5);
        assert!(valid.is_valid());

        let invalid = Pose2D::new(f64::INFINITY, 0.0, 0.0);
        assert!(!invalid.is_valid());
    }

    #[test]
    fn test_pose2d_serialization() {
        let pose = Pose2D::new(1.0, 2.0, 0.5);
        let serialized = serde_json::to_string(&pose).unwrap();
        let deserialized: Pose2D = serde_json::from_str(&serialized).unwrap();
        assert_eq!(pose.x, deserialized.x);
        assert_eq!(pose.y, deserialized.y);
        assert_eq!(pose.theta, deserialized.theta);
    }

    // ============================================================================
    // Transform Tests
    // ============================================================================

    #[test]
    fn test_transform_new() {
        let tf = TransformStamped::new([1.0, 2.0, 3.0], [0.0, 0.0, 0.0, 1.0]);
        assert_eq!(tf.translation, [1.0, 2.0, 3.0]);
        assert_eq!(tf.rotation, [0.0, 0.0, 0.0, 1.0]);
        assert!(tf.timestamp_ns > 0);
    }

    #[test]
    fn test_transform_identity() {
        let tf = TransformStamped::identity();
        assert_eq!(tf.translation, [0.0, 0.0, 0.0]);
        assert_eq!(tf.rotation, [0.0, 0.0, 0.0, 1.0]);
    }

    #[test]
    fn test_transform_from_pose_2d() {
        let pose = Pose2D::new(1.0, 2.0, 0.0);
        let tf = TransformStamped::from_pose_2d(&pose);
        assert_eq!(tf.translation[0], 1.0);
        assert_eq!(tf.translation[1], 2.0);
        assert_eq!(tf.translation[2], 0.0);
        // Identity quaternion for theta=0
        assert!((tf.rotation[3] - 1.0).abs() < 1e-10);
    }

    #[test]
    fn test_transform_is_valid() {
        let valid = TransformStamped::identity();
        assert!(valid.is_valid());

        let invalid_translation =
            TransformStamped::new([f64::INFINITY, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]);
        assert!(!invalid_translation.is_valid());

        let unnormalized = TransformStamped::new([0.0; 3], [1.0, 1.0, 1.0, 1.0]);
        assert!(!unnormalized.is_valid()); // Quaternion not normalized
    }

    #[test]
    fn test_transform_normalize_rotation() {
        let mut tf = TransformStamped::new([0.0; 3], [1.0, 1.0, 1.0, 1.0]);
        tf.normalize_rotation();
        let norm = tf.rotation.iter().map(|v| v * v).sum::<f64>().sqrt();
        assert!((norm - 1.0).abs() < 1e-10);
    }

    #[test]
    fn test_transform_serialization() {
        let tf = TransformStamped::identity();
        let serialized = serde_json::to_string(&tf).unwrap();
        let deserialized: TransformStamped = serde_json::from_str(&serialized).unwrap();
        assert_eq!(tf.translation, deserialized.translation);
        assert_eq!(tf.rotation, deserialized.rotation);
    }

    // ============================================================================
    // Point3 Tests
    // ============================================================================

    #[test]
    fn test_point3_new() {
        let p = Point3::new(1.0, 2.0, 3.0);
        assert_eq!(p.x, 1.0);
        assert_eq!(p.y, 2.0);
        assert_eq!(p.z, 3.0);
    }

    #[test]
    fn test_point3_origin() {
        let p = Point3::origin();
        assert_eq!(p.x, 0.0);
        assert_eq!(p.y, 0.0);
        assert_eq!(p.z, 0.0);
    }

    #[test]
    fn test_point3_distance_to() {
        let p1 = Point3::origin();
        let p2 = Point3::new(1.0, 2.0, 2.0);
        assert!((p1.distance_to(&p2) - 3.0).abs() < 1e-10);
    }

    #[test]
    fn test_point3_serialization() {
        let p = Point3::new(1.0, 2.0, 3.0);
        let serialized = serde_json::to_string(&p).unwrap();
        let deserialized: Point3 = serde_json::from_str(&serialized).unwrap();
        assert_eq!(p.x, deserialized.x);
        assert_eq!(p.y, deserialized.y);
        assert_eq!(p.z, deserialized.z);
    }

    // ============================================================================
    // Vector3 Tests
    // ============================================================================

    #[test]
    fn test_vector3_new() {
        let v = Vector3::new(1.0, 2.0, 3.0);
        assert_eq!(v.x, 1.0);
        assert_eq!(v.y, 2.0);
        assert_eq!(v.z, 3.0);
    }

    #[test]
    fn test_vector3_zero() {
        let v = Vector3::zero();
        assert_eq!(v.x, 0.0);
        assert_eq!(v.y, 0.0);
        assert_eq!(v.z, 0.0);
    }

    #[test]
    fn test_vector3_magnitude() {
        let v = Vector3::new(3.0, 4.0, 0.0);
        assert!((v.magnitude() - 5.0).abs() < 1e-10);
    }

    #[test]
    fn test_vector3_normalize() {
        let mut v = Vector3::new(3.0, 4.0, 0.0);
        v.normalize();
        assert!((v.magnitude() - 1.0).abs() < 1e-10);
        assert!((v.x - 0.6).abs() < 1e-10);
        assert!((v.y - 0.8).abs() < 1e-10);
    }

    #[test]
    fn test_vector3_dot() {
        let v1 = Vector3::new(1.0, 2.0, 3.0);
        let v2 = Vector3::new(4.0, 5.0, 6.0);
        assert!((v1.dot(&v2) - 32.0).abs() < 1e-10); // 1*4 + 2*5 + 3*6 = 32
    }

    #[test]
    fn test_vector3_cross() {
        let i = Vector3::new(1.0, 0.0, 0.0);
        let j = Vector3::new(0.0, 1.0, 0.0);
        let k = i.cross(&j);
        assert!((k.x - 0.0).abs() < 1e-10);
        assert!((k.y - 0.0).abs() < 1e-10);
        assert!((k.z - 1.0).abs() < 1e-10);
    }

    #[test]
    fn test_vector3_serialization() {
        let v = Vector3::new(1.0, 2.0, 3.0);
        let serialized = serde_json::to_string(&v).unwrap();
        let deserialized: Vector3 = serde_json::from_str(&serialized).unwrap();
        assert_eq!(v.x, deserialized.x);
        assert_eq!(v.y, deserialized.y);
        assert_eq!(v.z, deserialized.z);
    }

    // ============================================================================
    // Quaternion Tests
    // ============================================================================

    #[test]
    fn test_quaternion_new() {
        let q = Quaternion::new(0.0, 0.0, 0.0, 1.0);
        assert_eq!(q.x, 0.0);
        assert_eq!(q.y, 0.0);
        assert_eq!(q.z, 0.0);
        assert_eq!(q.w, 1.0);
    }

    #[test]
    fn test_quaternion_identity() {
        let q = Quaternion::identity();
        assert_eq!(q.x, 0.0);
        assert_eq!(q.y, 0.0);
        assert_eq!(q.z, 0.0);
        assert_eq!(q.w, 1.0);
    }

    #[test]
    fn test_quaternion_default() {
        let q = Quaternion::default();
        assert_eq!(q.w, 1.0);
    }

    #[test]
    fn test_quaternion_from_euler_identity() {
        let q = Quaternion::from_euler(0.0, 0.0, 0.0);
        assert!((q.x - 0.0).abs() < 1e-10);
        assert!((q.y - 0.0).abs() < 1e-10);
        assert!((q.z - 0.0).abs() < 1e-10);
        assert!((q.w - 1.0).abs() < 1e-10);
    }

    #[test]
    fn test_quaternion_from_euler_yaw_90() {
        let q = Quaternion::from_euler(0.0, 0.0, PI / 2.0);
        // 90 degree yaw: qz ≈ sin(45°) ≈ 0.707, qw ≈ cos(45°) ≈ 0.707
        assert!((q.z - std::f64::consts::FRAC_1_SQRT_2).abs() < 1e-6);
        assert!((q.w - std::f64::consts::FRAC_1_SQRT_2).abs() < 1e-6);
    }

    #[test]
    fn test_quaternion_normalize() {
        let mut q = Quaternion::new(1.0, 1.0, 1.0, 1.0);
        q.normalize();
        let norm = (q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w).sqrt();
        assert!((norm - 1.0).abs() < 1e-10);
    }

    #[test]
    fn test_quaternion_is_valid() {
        let valid = Quaternion::identity();
        assert!(valid.is_valid());

        let invalid = Quaternion::new(f64::INFINITY, 0.0, 0.0, 1.0);
        assert!(!invalid.is_valid());
    }

    #[test]
    fn test_quaternion_serialization() {
        let q = Quaternion::identity();
        let serialized = serde_json::to_string(&q).unwrap();
        let deserialized: Quaternion = serde_json::from_str(&serialized).unwrap();
        assert_eq!(q.x, deserialized.x);
        assert_eq!(q.y, deserialized.y);
        assert_eq!(q.z, deserialized.z);
        assert_eq!(q.w, deserialized.w);
    }

    // ============================================================================
    // LogSummary Tests
    // ============================================================================

    #[test]
    fn test_twist_log_summary() {
        let twist = Twist::new_2d(1.0, 0.5);
        let summary = twist.log_summary();
        assert!(!summary.is_empty());
        assert!(summary.contains("linear") || summary.contains("1.0"));
    }

    #[test]
    fn test_pose2d_log_summary() {
        let pose = Pose2D::new(1.0, 2.0, 0.5);
        let summary = pose.log_summary();
        assert!(!summary.is_empty());
    }

    // ============================================================================
    // Pod Message (bytemuck) Tests
    // ============================================================================

    #[test]
    fn test_twist_pod_cast() {
        let twist = Twist::new_2d(1.5, 0.3);
        let bytes: &[u8] = horus_core::bytemuck::bytes_of(&twist);
        let reconstructed: &Twist = horus_core::bytemuck::from_bytes(bytes);
        assert_eq!(twist.linear, reconstructed.linear);
        assert_eq!(twist.angular, reconstructed.angular);
    }

    #[test]
    fn test_pose2d_pod_cast() {
        let pose = Pose2D::new(1.0, 2.0, 0.5);
        let bytes: &[u8] = horus_core::bytemuck::bytes_of(&pose);
        let reconstructed: &Pose2D = horus_core::bytemuck::from_bytes(bytes);
        assert_eq!(pose.x, reconstructed.x);
        assert_eq!(pose.y, reconstructed.y);
        assert_eq!(pose.theta, reconstructed.theta);
    }

    #[test]
    fn test_vector3_pod_cast() {
        let v = Vector3::new(1.0, 2.0, 3.0);
        let bytes: &[u8] = horus_core::bytemuck::bytes_of(&v);
        let reconstructed: &Vector3 = horus_core::bytemuck::from_bytes(bytes);
        assert_eq!(v.x, reconstructed.x);
        assert_eq!(v.y, reconstructed.y);
        assert_eq!(v.z, reconstructed.z);
    }
}
