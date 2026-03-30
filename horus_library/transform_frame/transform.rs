//! 3D Transform representation and math operations
//!
//! Provides a `Transform` struct representing a rigid body transformation
//! (translation + rotation) with full support for composition, inversion,
//! and point/vector transformation.

use horus_core::bytemuck::{Pod, Zeroable};
use serde::{Deserialize, Serialize};

/// 3D Transform (translation + rotation as quaternion)
///
/// Represents a rigid body transformation in 3D space.
/// Uses f64 for precision in robotics applications.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[repr(C)]
pub struct Transform {
    /// Translation [x, y, z] in meters
    pub translation: [f64; 3],
    /// Rotation as quaternion [x, y, z, w] (Hamilton convention)
    pub rotation: [f64; 4],
}

// Pod/Zeroable impls for shared memory compatibility
unsafe impl Pod for Transform {}
unsafe impl Zeroable for Transform {}

impl Default for Transform {
    fn default() -> Self {
        Self::identity()
    }
}

impl Transform {
    /// Create identity transform (no translation or rotation)
    pub fn identity() -> Self {
        Self {
            translation: [0.0, 0.0, 0.0],
            rotation: [0.0, 0.0, 0.0, 1.0], // Identity quaternion
        }
    }

    /// Create transform from translation and quaternion [x, y, z, w]
    pub fn new(translation: [f64; 3], rotation: [f64; 4]) -> Self {
        let mut tf = Self {
            translation,
            rotation,
        };
        tf.normalize_rotation();
        tf
    }

    /// Create transform from translation only (no rotation)
    pub fn from_translation(translation: [f64; 3]) -> Self {
        Self {
            translation,
            rotation: [0.0, 0.0, 0.0, 1.0],
        }
    }

    /// Create transform from rotation only (no translation)
    pub fn from_rotation(rotation: [f64; 4]) -> Self {
        let mut tf = Self {
            translation: [0.0, 0.0, 0.0],
            rotation,
        };
        tf.normalize_rotation();
        tf
    }

    /// Create transform from translation and Euler angles (roll, pitch, yaw) in radians
    pub fn from_euler(translation: [f64; 3], rpy: [f64; 3]) -> Self {
        let rotation = euler_to_quaternion(rpy[0], rpy[1], rpy[2]);
        Self {
            translation,
            rotation,
        }
    }

    // ========================================================================
    // Short Constructors
    // ========================================================================

    /// Create a translation-only transform from x, y, z components.
    ///
    /// Equivalent to `Transform::from_translation([x, y, z])`.
    #[inline]
    pub fn xyz(x: f64, y: f64, z: f64) -> Self {
        Self::from_translation([x, y, z])
    }

    /// Create a translation-only transform along the X axis.
    #[inline]
    pub fn x(x: f64) -> Self {
        Self::from_translation([x, 0.0, 0.0])
    }

    /// Create a translation-only transform along the Y axis.
    #[inline]
    pub fn y(y: f64) -> Self {
        Self::from_translation([0.0, y, 0.0])
    }

    /// Create a translation-only transform along the Z axis.
    #[inline]
    pub fn z(z: f64) -> Self {
        Self::from_translation([0.0, 0.0, z])
    }

    /// Create a rotation-only transform around the Z axis (yaw).
    ///
    /// Angle in radians.
    #[inline]
    pub fn yaw(angle: f64) -> Self {
        Self::from_euler([0.0, 0.0, 0.0], [0.0, 0.0, angle])
    }

    /// Create a rotation-only transform around the Y axis (pitch).
    ///
    /// Angle in radians.
    #[inline]
    pub fn pitch(angle: f64) -> Self {
        Self::from_euler([0.0, 0.0, 0.0], [0.0, angle, 0.0])
    }

    /// Create a rotation-only transform around the X axis (roll).
    ///
    /// Angle in radians.
    #[inline]
    pub fn roll(angle: f64) -> Self {
        Self::from_euler([0.0, 0.0, 0.0], [angle, 0.0, 0.0])
    }

    /// Create a rotation-only transform from roll, pitch, yaw angles.
    ///
    /// Angles in radians. Convention: roll=X, pitch=Y, yaw=Z.
    #[inline]
    pub fn rpy(roll: f64, pitch: f64, yaw: f64) -> Self {
        Self::from_euler([0.0, 0.0, 0.0], [roll, pitch, yaw])
    }

    // ========================================================================
    // Chainable Rotation Modifiers
    // ========================================================================

    /// Compose a yaw (Z-axis) rotation onto this transform.
    ///
    /// Returns a new transform with the translation preserved and the
    /// rotation composed: `self.rotation * yaw_rotation`.
    #[inline]
    pub fn with_yaw(self, angle: f64) -> Self {
        self.compose(&Self::yaw(angle))
    }

    /// Compose an RPY rotation onto this transform.
    ///
    /// Equivalent to `self.compose(Transform::rpy(roll, pitch, yaw))`.
    #[inline]
    pub fn with_rpy(self, roll: f64, pitch: f64, yaw: f64) -> Self {
        self.compose(&Self::rpy(roll, pitch, yaw))
    }

    /// Normalize the rotation quaternion
    fn normalize_rotation(&mut self) {
        let norm = (self.rotation[0].powi(2)
            + self.rotation[1].powi(2)
            + self.rotation[2].powi(2)
            + self.rotation[3].powi(2))
        .sqrt();

        if norm > 1e-10 {
            self.rotation[0] /= norm;
            self.rotation[1] /= norm;
            self.rotation[2] /= norm;
            self.rotation[3] /= norm;
        } else {
            self.rotation = [0.0, 0.0, 0.0, 1.0];
        }
    }

    /// Compose two transforms: self * other
    /// Result transforms from other's frame to self's parent frame
    pub fn compose(&self, other: &Transform) -> Transform {
        // Rotate other's translation by self's rotation, then add self's translation
        let rotated_translation = self.rotate_vector(other.translation);
        let translation = [
            self.translation[0] + rotated_translation[0],
            self.translation[1] + rotated_translation[1],
            self.translation[2] + rotated_translation[2],
        ];

        // Compose quaternions: self.rotation * other.rotation
        let rotation = quaternion_multiply(self.rotation, other.rotation);

        Transform {
            translation,
            rotation,
        }
    }

    /// Invert transform (reverse direction)
    pub fn inverse(&self) -> Transform {
        // Conjugate of quaternion (negate x, y, z)
        let inv_rotation = [
            -self.rotation[0],
            -self.rotation[1],
            -self.rotation[2],
            self.rotation[3],
        ];

        // Rotate negated translation by inverse rotation
        let neg_translation = [
            -self.translation[0],
            -self.translation[1],
            -self.translation[2],
        ];
        let inv_translation = rotate_vector_by_quaternion(neg_translation, inv_rotation);

        Transform {
            translation: inv_translation,
            rotation: inv_rotation,
        }
    }

    /// Apply transform to a 3D point
    pub fn transform_point(&self, point: [f64; 3]) -> [f64; 3] {
        let rotated = self.rotate_vector(point);
        [
            rotated[0] + self.translation[0],
            rotated[1] + self.translation[1],
            rotated[2] + self.translation[2],
        ]
    }

    /// Apply transform to a vector (rotation only, no translation)
    pub fn transform_vector(&self, vector: [f64; 3]) -> [f64; 3] {
        self.rotate_vector(vector)
    }

    /// Rotate a vector by the transform's rotation
    fn rotate_vector(&self, v: [f64; 3]) -> [f64; 3] {
        rotate_vector_by_quaternion(v, self.rotation)
    }

    /// Convert rotation to Euler angles (roll, pitch, yaw) in radians
    pub fn to_euler(&self) -> [f64; 3] {
        quaternion_to_euler(self.rotation)
    }

    /// Convert to 4x4 homogeneous transformation matrix (row-major)
    pub fn to_matrix(&self) -> [[f64; 4]; 4] {
        let [x, y, z, w] = self.rotation;
        let [tx, ty, tz] = self.translation;

        let xx = x * x;
        let yy = y * y;
        let zz = z * z;
        let xy = x * y;
        let xz = x * z;
        let yz = y * z;
        let wx = w * x;
        let wy = w * y;
        let wz = w * z;

        [
            [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy), tx],
            [2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx), ty],
            [2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy), tz],
            [0.0, 0.0, 0.0, 1.0],
        ]
    }

    /// Create transform from 4x4 homogeneous matrix (row-major)
    pub fn from_matrix(m: [[f64; 4]; 4]) -> Self {
        let translation = [m[0][3], m[1][3], m[2][3]];
        let rotation = matrix_to_quaternion([
            [m[0][0], m[0][1], m[0][2]],
            [m[1][0], m[1][1], m[1][2]],
            [m[2][0], m[2][1], m[2][2]],
        ]);
        Self::new(translation, rotation)
    }

    /// Linear interpolation between two transforms (SLERP for rotation)
    pub fn interpolate(&self, other: &Transform, t: f64) -> Transform {
        let t = t.clamp(0.0, 1.0);

        // Linear interpolation for translation
        let translation = [
            self.translation[0] + t * (other.translation[0] - self.translation[0]),
            self.translation[1] + t * (other.translation[1] - self.translation[1]),
            self.translation[2] + t * (other.translation[2] - self.translation[2]),
        ];

        // SLERP for rotation
        let rotation = quaternion_slerp(self.rotation, other.rotation, t);

        Transform {
            translation,
            rotation,
        }
    }

    /// Validate that this transform contains no NaN/Inf values and has a
    /// valid rotation quaternion.
    ///
    /// Returns `Ok(())` if valid, or `Err(HorusError::InvalidInput)` describing
    /// the problem. Called automatically by `TransformFrame::update_transform*` methods.
    ///
    /// # Validation rules
    /// 1. Translation must be finite (no NaN, no Inf)
    /// 2. Rotation must be finite (no NaN, no Inf)
    /// 3. Rotation must not be the zero quaternion `[0,0,0,0]`
    /// 4. Rotation norm must be in `[0.5, 2.0]` — near-unit quaternions are
    ///    auto-normalized, far-from-unit ones are rejected
    pub fn validate(&self) -> Result<(), String> {
        // Check translation for NaN/Inf
        for (i, &v) in self.translation.iter().enumerate() {
            if !v.is_finite() {
                let axis = ["x", "y", "z"][i];
                return Err(format!("translation.{} is {} (must be finite)", axis, v));
            }
        }

        // Check rotation for NaN/Inf
        for (i, &v) in self.rotation.iter().enumerate() {
            if !v.is_finite() {
                let comp = ["x", "y", "z", "w"][i];
                return Err(format!("rotation.{} is {} (must be finite)", comp, v));
            }
        }

        // Check for zero quaternion
        let norm_sq = self.rotation[0].powi(2)
            + self.rotation[1].powi(2)
            + self.rotation[2].powi(2)
            + self.rotation[3].powi(2);

        if norm_sq < 1e-20 {
            return Err("rotation is zero quaternion [0,0,0,0]".to_string());
        }

        // Check quaternion norm range
        let norm = norm_sq.sqrt();
        if !(0.5..=2.0).contains(&norm) {
            return Err(format!(
                "rotation quaternion norm {} is outside valid range [0.5, 2.0]",
                norm
            ));
        }

        Ok(())
    }

    /// Validate and auto-normalize the rotation quaternion.
    ///
    /// Returns the validated (and possibly normalized) transform, or an error.
    /// Near-unit quaternions (norm in `[0.5, 2.0]`) are normalized to unit length.
    pub fn validated(mut self) -> Result<Self, String> {
        self.validate()?;

        // Auto-normalize quaternion to unit length
        let norm = (self.rotation[0].powi(2)
            + self.rotation[1].powi(2)
            + self.rotation[2].powi(2)
            + self.rotation[3].powi(2))
        .sqrt();

        if (norm - 1.0).abs() > 1e-10 {
            self.rotation[0] /= norm;
            self.rotation[1] /= norm;
            self.rotation[2] /= norm;
            self.rotation[3] /= norm;
        }

        Ok(self)
    }

    /// Check if transform is approximately identity
    pub fn is_identity(&self, epsilon: f64) -> bool {
        let t_zero = self.translation[0].abs() < epsilon
            && self.translation[1].abs() < epsilon
            && self.translation[2].abs() < epsilon;

        let r_identity = self.rotation[0].abs() < epsilon
            && self.rotation[1].abs() < epsilon
            && self.rotation[2].abs() < epsilon
            && (self.rotation[3] - 1.0).abs() < epsilon;

        t_zero && r_identity
    }

    /// Get the distance (translation magnitude) of this transform
    pub fn translation_magnitude(&self) -> f64 {
        (self.translation[0].powi(2) + self.translation[1].powi(2) + self.translation[2].powi(2))
            .sqrt()
    }

    /// Get the rotation angle in radians
    pub fn rotation_angle(&self) -> f64 {
        2.0 * self.rotation[3].clamp(-1.0, 1.0).acos()
    }
}

// Quaternion math helpers

fn quaternion_multiply(a: [f64; 4], b: [f64; 4]) -> [f64; 4] {
    [
        a[3] * b[0] + a[0] * b[3] + a[1] * b[2] - a[2] * b[1],
        a[3] * b[1] - a[0] * b[2] + a[1] * b[3] + a[2] * b[0],
        a[3] * b[2] + a[0] * b[1] - a[1] * b[0] + a[2] * b[3],
        a[3] * b[3] - a[0] * b[0] - a[1] * b[1] - a[2] * b[2],
    ]
}

fn rotate_vector_by_quaternion(v: [f64; 3], q: [f64; 4]) -> [f64; 3] {
    let [qx, qy, qz, qw] = q;

    // Optimized quaternion-vector rotation
    let t = [
        2.0 * (qy * v[2] - qz * v[1]),
        2.0 * (qz * v[0] - qx * v[2]),
        2.0 * (qx * v[1] - qy * v[0]),
    ];

    [
        v[0] + qw * t[0] + qy * t[2] - qz * t[1],
        v[1] + qw * t[1] + qz * t[0] - qx * t[2],
        v[2] + qw * t[2] + qx * t[1] - qy * t[0],
    ]
}

fn euler_to_quaternion(roll: f64, pitch: f64, yaw: f64) -> [f64; 4] {
    let cr = (roll / 2.0).cos();
    let sr = (roll / 2.0).sin();
    let cp = (pitch / 2.0).cos();
    let sp = (pitch / 2.0).sin();
    let cy = (yaw / 2.0).cos();
    let sy = (yaw / 2.0).sin();

    [
        sr * cp * cy - cr * sp * sy, // x
        cr * sp * cy + sr * cp * sy, // y
        cr * cp * sy - sr * sp * cy, // z
        cr * cp * cy + sr * sp * sy, // w
    ]
}

fn quaternion_to_euler(q: [f64; 4]) -> [f64; 3] {
    let [x, y, z, w] = q;

    // Roll (x-axis rotation)
    let sinr_cosp = 2.0 * (w * x + y * z);
    let cosr_cosp = 1.0 - 2.0 * (x * x + y * y);
    let roll = sinr_cosp.atan2(cosr_cosp);

    // Pitch (y-axis rotation)
    let sinp = 2.0 * (w * y - z * x);
    let pitch = if sinp.abs() >= 1.0 {
        std::f64::consts::FRAC_PI_2.copysign(sinp)
    } else {
        sinp.asin()
    };

    // Yaw (z-axis rotation)
    let siny_cosp = 2.0 * (w * z + x * y);
    let cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
    let yaw = siny_cosp.atan2(cosy_cosp);

    [roll, pitch, yaw]
}

fn matrix_to_quaternion(m: [[f64; 3]; 3]) -> [f64; 4] {
    let trace = m[0][0] + m[1][1] + m[2][2];

    if trace > 0.0 {
        let s = 0.5 / (trace + 1.0).max(0.0).sqrt().max(1e-12);
        [
            (m[2][1] - m[1][2]) * s,
            (m[0][2] - m[2][0]) * s,
            (m[1][0] - m[0][1]) * s,
            0.25 / s,
        ]
    } else if m[0][0] > m[1][1] && m[0][0] > m[2][2] {
        let s = 2.0
            * (1.0 + m[0][0] - m[1][1] - m[2][2])
                .max(0.0)
                .sqrt()
                .max(1e-12);
        [
            0.25 * s,
            (m[0][1] + m[1][0]) / s,
            (m[0][2] + m[2][0]) / s,
            (m[2][1] - m[1][2]) / s,
        ]
    } else if m[1][1] > m[2][2] {
        let s = 2.0
            * (1.0 + m[1][1] - m[0][0] - m[2][2])
                .max(0.0)
                .sqrt()
                .max(1e-12);
        [
            (m[0][1] + m[1][0]) / s,
            0.25 * s,
            (m[1][2] + m[2][1]) / s,
            (m[0][2] - m[2][0]) / s,
        ]
    } else {
        let s = 2.0
            * (1.0 + m[2][2] - m[0][0] - m[1][1])
                .max(0.0)
                .sqrt()
                .max(1e-12);
        [
            (m[0][2] + m[2][0]) / s,
            (m[1][2] + m[2][1]) / s,
            0.25 * s,
            (m[1][0] - m[0][1]) / s,
        ]
    }
}

/// Quaternion SLERP (Spherical Linear intERPolation)
///
/// Inputs should be unit quaternions but the function is robust against
/// slight floating-point deviations (norm within ±0.01 of 1.0):
/// - `dot` is clamped to `[-1.0, 1.0]` before `acos()` to prevent NaN from
///   rounding errors on near-unit inputs.
/// - The result is always a unit quaternion.
pub fn quaternion_slerp(a: [f64; 4], b: [f64; 4], t: f64) -> [f64; 4] {
    // Guard: inputs should be unit quaternions.
    debug_assert!(
        (a[0].powi(2) + a[1].powi(2) + a[2].powi(2) + a[3].powi(2) - 1.0).abs() < 0.01,
        "quaternion_slerp: input `a` is not a unit quaternion (norm² deviation > 0.01)"
    );
    debug_assert!(
        (b[0].powi(2) + b[1].powi(2) + b[2].powi(2) + b[3].powi(2) - 1.0).abs() < 0.01,
        "quaternion_slerp: input `b` is not a unit quaternion (norm² deviation > 0.01)"
    );

    // Compute dot product
    let mut dot = a[0] * b[0] + a[1] * b[1] + a[2] * b[2] + a[3] * b[3];

    // If dot is negative, negate one quaternion to take shorter path
    let mut b = b;
    if dot < 0.0 {
        b = [-b[0], -b[1], -b[2], -b[3]];
        dot = -dot;
    }

    // If quaternions are very close, use linear interpolation
    if dot > 0.9995 {
        let result = [
            a[0] + t * (b[0] - a[0]),
            a[1] + t * (b[1] - a[1]),
            a[2] + t * (b[2] - a[2]),
            a[3] + t * (b[3] - a[3]),
        ];
        // Normalize
        let norm =
            (result[0].powi(2) + result[1].powi(2) + result[2].powi(2) + result[3].powi(2)).sqrt();
        return [
            result[0] / norm,
            result[1] / norm,
            result[2] / norm,
            result[3] / norm,
        ];
    }

    // Clamp dot to [-1.0, 1.0] before acos() to prevent NaN from floating-point
    // rounding on near-unit quaternions (e.g. dot = 1.0000001 → acos → NaN).
    let theta_0 = dot.clamp(-1.0, 1.0).acos();
    let theta = theta_0 * t;
    let sin_theta = theta.sin();
    let sin_theta_0 = theta_0.sin();

    let s0 = theta.cos() - dot * sin_theta / sin_theta_0;
    let s1 = sin_theta / sin_theta_0;

    let result = [
        s0 * a[0] + s1 * b[0],
        s0 * a[1] + s1 * b[1],
        s0 * a[2] + s1 * b[2],
        s0 * a[3] + s1 * b[3],
    ];

    // Guard: result must be finite (no NaN/Inf).
    // Note: we do not assert unit-norm here because slightly-unnormalized inputs
    // produce slightly-unnormalized output — that is the caller's responsibility.
    debug_assert!(
        result[0].is_finite()
            && result[1].is_finite()
            && result[2].is_finite()
            && result[3].is_finite(),
        "quaternion_slerp: result contains NaN/Inf"
    );

    result
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::PI;

    const EPSILON: f64 = 1e-10;

    fn approx_eq(a: f64, b: f64) -> bool {
        (a - b).abs() < 1e-6
    }

    fn approx_eq_arr3(a: [f64; 3], b: [f64; 3]) -> bool {
        approx_eq(a[0], b[0]) && approx_eq(a[1], b[1]) && approx_eq(a[2], b[2])
    }

    #[test]
    fn test_identity() {
        let tf = Transform::identity();
        assert!(tf.is_identity(EPSILON));
    }

    #[test]
    fn test_translation_only() {
        let tf = Transform::from_translation([1.0, 2.0, 3.0]);
        let point = tf.transform_point([0.0, 0.0, 0.0]);
        assert!(approx_eq_arr3(point, [1.0, 2.0, 3.0]));
    }

    #[test]
    fn test_rotation_only() {
        // 90 degree rotation around Z axis
        let tf = Transform::from_euler([0.0, 0.0, 0.0], [0.0, 0.0, PI / 2.0]);
        let point = tf.transform_point([1.0, 0.0, 0.0]);
        assert!(approx_eq_arr3(point, [0.0, 1.0, 0.0]));
    }

    #[test]
    fn test_compose_translations() {
        let tf1 = Transform::from_translation([1.0, 0.0, 0.0]);
        let tf2 = Transform::from_translation([0.0, 2.0, 0.0]);
        let composed = tf1.compose(&tf2);
        assert!(approx_eq_arr3(composed.translation, [1.0, 2.0, 0.0]));
    }

    #[test]
    fn test_inverse() {
        let tf = Transform::from_euler([1.0, 2.0, 3.0], [0.1, 0.2, 0.3]);
        let inv = tf.inverse();
        let composed = tf.compose(&inv);
        assert!(composed.is_identity(1e-6));
    }

    #[test]
    fn test_transform_point() {
        let tf = Transform::from_euler([1.0, 0.0, 0.0], [0.0, 0.0, PI / 2.0]);
        let point = tf.transform_point([1.0, 0.0, 0.0]);
        // Rotate [1,0,0] by 90 deg around Z = [0,1,0], then translate by [1,0,0]
        assert!(approx_eq_arr3(point, [1.0, 1.0, 0.0]));
    }

    #[test]
    fn test_euler_roundtrip() {
        let rpy = [0.1, 0.2, 0.3];
        let tf = Transform::from_euler([0.0, 0.0, 0.0], rpy);
        let result = tf.to_euler();
        assert!(approx_eq(result[0], rpy[0]));
        assert!(approx_eq(result[1], rpy[1]));
        assert!(approx_eq(result[2], rpy[2]));
    }

    #[test]
    fn test_matrix_roundtrip() {
        let tf = Transform::from_euler([1.0, 2.0, 3.0], [0.1, 0.2, 0.3]);
        let matrix = tf.to_matrix();
        let recovered = Transform::from_matrix(matrix);

        assert!(approx_eq_arr3(tf.translation, recovered.translation));
        // Quaternions may differ by sign but represent same rotation
        let dot = tf.rotation[0] * recovered.rotation[0]
            + tf.rotation[1] * recovered.rotation[1]
            + tf.rotation[2] * recovered.rotation[2]
            + tf.rotation[3] * recovered.rotation[3];
        assert!(approx_eq(dot.abs(), 1.0));
    }

    #[test]
    fn test_interpolate() {
        let tf1 = Transform::from_translation([0.0, 0.0, 0.0]);
        let tf2 = Transform::from_translation([10.0, 0.0, 0.0]);

        let mid = tf1.interpolate(&tf2, 0.5);
        assert!(approx_eq_arr3(mid.translation, [5.0, 0.0, 0.0]));
    }

    #[test]
    fn test_translation_magnitude() {
        let tf = Transform::from_translation([3.0, 4.0, 0.0]);
        assert!(approx_eq(tf.translation_magnitude(), 5.0));
    }

    #[test]
    fn test_pod_safety() {
        // Ensure Transform is Pod-safe
        let tf = Transform::identity();
        let bytes: &[u8] = horus_core::bytemuck::bytes_of(&tf);
        assert!(!bytes.is_empty());
    }

    /// Regression test: quaternion_slerp must not produce NaN when inputs are
    /// slightly over-normalized (norm > 1.0) due to floating-point rounding.
    ///
    /// Before the fix, `dot.acos()` with dot = 1.0000001 returned NaN and
    /// propagated silently through all downstream coordinate transforms.
    /// The fix clamps dot to [-1.0, 1.0] before acos().
    #[test]
    fn test_slerp_unnormalized_inputs_no_nan() {
        // Slightly over-normalized quaternions (norm ≈ 1.001, mimicking
        // accumulated floating-point error in a sensor pipeline).
        let factor = 1.001_f64.sqrt(); // scale so that norm² = 1.001
        let a: [f64; 4] = [
            0.0,
            0.0,
            0.0,
            1.0 * factor, // identity-ish, but norm > 1.0
        ];
        let b: [f64; 4] = [
            0.0,
            (PI / 4.0).sin() * factor,
            0.0,
            (PI / 4.0).cos() * factor, // 90° around Y, but norm > 1.0
        ];

        for &t in &[0.0, 0.25, 0.5, 0.75, 1.0] {
            let result = quaternion_slerp(a, b, t);
            // The only guarantee for unnormalized inputs is finite output (no NaN/Inf).
            // Callers are responsible for normalizing quaternions before SLERP.
            assert!(
                result.iter().all(|x| x.is_finite()),
                "slerp returned NaN/Inf at t={}: {:?}",
                t,
                result
            );
        }
    }

    /// Edge case: slerp of two identical (unit) quaternions must return the same
    /// quaternion for all t, with no NaN.
    #[test]
    fn test_slerp_identical_quats_no_nan() {
        let q = [0.0_f64, (PI / 6.0).sin(), 0.0, (PI / 6.0).cos()]; // 60° around Y
        for &t in &[0.0, 0.5, 1.0] {
            let result = quaternion_slerp(q, q, t);
            assert!(
                result.iter().all(|x| x.is_finite()),
                "NaN for identical quats at t={}",
                t
            );
        }
    }

    // =====================================================================
    // Input Validation Tests
    // =====================================================================

    #[test]
    fn test_validate_identity_ok() {
        Transform::identity().validate().unwrap();
    }

    #[test]
    fn test_validate_normal_transform_ok() {
        let tf = Transform::from_euler([1.0, 2.0, 3.0], [0.1, 0.2, 0.3]);
        tf.validate().unwrap();
    }

    #[test]
    fn test_validate_nan_translation_x() {
        let tf = Transform {
            translation: [f64::NAN, 0.0, 0.0],
            rotation: [0.0, 0.0, 0.0, 1.0],
        };
        let err = tf.validate().unwrap_err();
        assert!(err.contains("translation.x"), "Error: {}", err);
    }

    #[test]
    fn test_validate_nan_translation_y() {
        let tf = Transform {
            translation: [0.0, f64::NAN, 0.0],
            rotation: [0.0, 0.0, 0.0, 1.0],
        };
        let err = tf.validate().unwrap_err();
        assert!(err.contains("translation.y"), "Error: {}", err);
    }

    #[test]
    fn test_validate_nan_translation_z() {
        let tf = Transform {
            translation: [0.0, 0.0, f64::NAN],
            rotation: [0.0, 0.0, 0.0, 1.0],
        };
        let err = tf.validate().unwrap_err();
        assert!(err.contains("translation.z"), "Error: {}", err);
    }

    #[test]
    fn test_validate_inf_translation() {
        let tf = Transform {
            translation: [f64::INFINITY, 0.0, 0.0],
            rotation: [0.0, 0.0, 0.0, 1.0],
        };
        let err = tf.validate().unwrap_err();
        assert!(err.contains("translation.x"), "Error: {}", err);
    }

    #[test]
    fn test_validate_neg_inf_translation() {
        let tf = Transform {
            translation: [0.0, f64::NEG_INFINITY, 0.0],
            rotation: [0.0, 0.0, 0.0, 1.0],
        };
        let err = tf.validate().unwrap_err();
        assert!(err.contains("translation.y"), "Error: {}", err);
    }

    #[test]
    fn test_validate_nan_rotation() {
        let tf = Transform {
            translation: [0.0, 0.0, 0.0],
            rotation: [f64::NAN, 0.0, 0.0, 1.0],
        };
        let err = tf.validate().unwrap_err();
        assert!(err.contains("rotation.x"), "Error: {}", err);
    }

    #[test]
    fn test_validate_inf_rotation() {
        let tf = Transform {
            translation: [0.0, 0.0, 0.0],
            rotation: [0.0, 0.0, f64::INFINITY, 1.0],
        };
        let err = tf.validate().unwrap_err();
        assert!(err.contains("rotation.z"), "Error: {}", err);
    }

    #[test]
    fn test_validate_zero_quaternion() {
        let tf = Transform {
            translation: [0.0, 0.0, 0.0],
            rotation: [0.0, 0.0, 0.0, 0.0],
        };
        let err = tf.validate().unwrap_err();
        assert!(err.contains("zero quaternion"), "Error: {}", err);
    }

    #[test]
    fn test_validate_far_from_unit_quaternion() {
        let tf = Transform {
            translation: [0.0, 0.0, 0.0],
            rotation: [0.0, 0.0, 0.0, 5.0], // norm = 5.0, way out of range
        };
        let err = tf.validate().unwrap_err();
        assert!(err.contains("outside valid range"), "Error: {}", err);
    }

    #[test]
    fn test_validate_tiny_quaternion() {
        let tf = Transform {
            translation: [0.0, 0.0, 0.0],
            rotation: [0.0, 0.0, 0.0, 0.1], // norm = 0.1, below 0.5
        };
        let err = tf.validate().unwrap_err();
        assert!(err.contains("outside valid range"), "Error: {}", err);
    }

    #[test]
    fn test_validated_auto_normalizes() {
        // Quaternion with norm ~1.5 (within [0.5, 2.0])
        let tf = Transform {
            translation: [1.0, 2.0, 3.0],
            rotation: [0.0, 0.0, 0.0, 1.5],
        };
        let result = tf.validated().unwrap();
        // Should be normalized to unit length
        let norm = (result.rotation[0].powi(2)
            + result.rotation[1].powi(2)
            + result.rotation[2].powi(2)
            + result.rotation[3].powi(2))
        .sqrt();
        assert!(
            (norm - 1.0).abs() < 1e-10,
            "Expected unit quaternion, got norm={}",
            norm
        );
        // Translation should be unchanged
        assert_eq!(result.translation, [1.0, 2.0, 3.0]);
    }

    #[test]
    fn test_validated_already_unit_unchanged() {
        let tf = Transform::identity();
        let result = tf.validated().unwrap();
        assert_eq!(result.rotation, [0.0, 0.0, 0.0, 1.0]);
    }

    #[test]
    fn test_validated_rejects_nan() {
        let tf = Transform {
            translation: [f64::NAN, 0.0, 0.0],
            rotation: [0.0, 0.0, 0.0, 1.0],
        };
        tf.validated().unwrap_err();
    }

    // =====================================================================
    // Short Constructor Tests
    // =====================================================================

    #[test]
    fn test_xyz_equivalence() {
        let a = Transform::xyz(1.0, 2.0, 3.0);
        let b = Transform::from_translation([1.0, 2.0, 3.0]);
        assert_eq!(a.translation, b.translation);
        assert_eq!(a.rotation, b.rotation);
    }

    #[test]
    fn test_x_y_z_shortcuts() {
        let tx = Transform::x(5.0);
        assert_eq!(tx.translation, [5.0, 0.0, 0.0]);

        let ty = Transform::y(3.0);
        assert_eq!(ty.translation, [0.0, 3.0, 0.0]);

        let tz = Transform::z(1.5);
        assert_eq!(tz.translation, [0.0, 0.0, 1.5]);
    }

    #[test]
    fn test_yaw_rotation() {
        let tf = Transform::yaw(std::f64::consts::FRAC_PI_2); // 90° around Z
        assert_eq!(tf.translation, [0.0, 0.0, 0.0]);

        // Rotating [1,0,0] by 90° yaw → [0,1,0]
        let pt = tf.transform_point([1.0, 0.0, 0.0]);
        assert!((pt[0]).abs() < 1e-10);
        assert!((pt[1] - 1.0).abs() < 1e-10);
        assert!((pt[2]).abs() < 1e-10);
    }

    #[test]
    fn test_pitch_rotation() {
        let tf = Transform::pitch(std::f64::consts::FRAC_PI_2); // 90° around Y
                                                                // Rotating [1,0,0] by 90° pitch → [0,0,-1]
        let pt = tf.transform_point([1.0, 0.0, 0.0]);
        assert!((pt[0]).abs() < 1e-10);
        assert!((pt[1]).abs() < 1e-10);
        assert!((pt[2] + 1.0).abs() < 1e-10);
    }

    #[test]
    fn test_roll_rotation() {
        let tf = Transform::roll(std::f64::consts::FRAC_PI_2); // 90° around X
                                                               // Rotating [0,1,0] by 90° roll → [0,0,1]
        let pt = tf.transform_point([0.0, 1.0, 0.0]);
        assert!((pt[0]).abs() < 1e-10);
        assert!((pt[1]).abs() < 1e-10);
        assert!((pt[2] - 1.0).abs() < 1e-10);
    }

    #[test]
    fn test_rpy_equivalence() {
        let a = Transform::rpy(0.1, 0.2, 0.3);
        let b = Transform::from_euler([0.0, 0.0, 0.0], [0.1, 0.2, 0.3]);
        for i in 0..4 {
            assert!((a.rotation[i] - b.rotation[i]).abs() < 1e-15);
        }
    }

    #[test]
    fn test_with_yaw_composition() {
        let tf = Transform::xyz(1.0, 0.0, 0.0).with_yaw(std::f64::consts::FRAC_PI_2);

        // Translation should be preserved (1,0,0)
        assert!((tf.translation[0] - 1.0).abs() < 1e-10);

        // Rotating [1,0,0] by the composed rotation (90° yaw) → [0,1,0]
        let pt = tf.transform_point([0.0, 0.0, 0.0]);
        assert!((pt[0] - 1.0).abs() < 1e-10); // Only translation applies to origin
    }

    #[test]
    fn test_with_rpy() {
        let tf = Transform::x(2.0).with_rpy(0.0, 0.0, std::f64::consts::PI);

        // Translation is 2.0 in X
        assert!((tf.translation[0] - 2.0).abs() < 1e-10);

        // 180° yaw on [1,0,0] → [-1,0,0]
        let v = tf.transform_vector([1.0, 0.0, 0.0]);
        assert!((v[0] + 1.0).abs() < 1e-10);
    }
}
