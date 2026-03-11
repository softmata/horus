//! Pod point element types for zero-copy point cloud IPC
//!
//! Fixed-size point types (`PointXYZ`, `PointXYZRGB`, `PointXYZI`) suitable for
//! shared memory transport. These are the elements stored in a `PointCloud`'s
//! tensor data — they live in `horus_core::types` as leaf types with no HORUS
//! dependencies beyond `bytemuck`.

use bytemuck::{Pod, Zeroable};

/// Basic 3D point (XYZ only)
///
/// Size: 12 bytes (packed, no padding)
#[repr(C, packed)]
#[derive(Debug, Clone, Copy, Default, PartialEq, Pod, Zeroable)]
pub struct PointXYZ {
    /// X coordinate (meters)
    pub x: f32,
    /// Y coordinate (meters)
    pub y: f32,
    /// Z coordinate (meters)
    pub z: f32,
}

impl PointXYZ {
    /// Create a new point
    pub fn new(x: f32, y: f32, z: f32) -> Self {
        Self { x, y, z }
    }

    /// Calculate distance from origin
    pub fn distance(&self) -> f32 {
        (self.x * self.x + self.y * self.y + self.z * self.z).sqrt()
    }

    /// Calculate distance to another point
    pub fn distance_to(&self, other: &PointXYZ) -> f32 {
        let dx = self.x - other.x;
        let dy = self.y - other.y;
        let dz = self.z - other.z;
        (dx * dx + dy * dy + dz * dz).sqrt()
    }
}

/// 3D point with RGB color
///
/// Common format for RGB-D cameras like Intel RealSense.
///
/// Size: 16 bytes (aligned)
#[repr(C)]
#[derive(Debug, Clone, Copy, Default, PartialEq, Pod, Zeroable)]
pub struct PointXYZRGB {
    /// X coordinate (meters)
    pub x: f32,
    /// Y coordinate (meters)
    pub y: f32,
    /// Z coordinate (meters)
    pub z: f32,
    /// Red component (0-255)
    pub r: u8,
    /// Green component (0-255)
    pub g: u8,
    /// Blue component (0-255)
    pub b: u8,
    /// Alpha/padding
    pub a: u8,
}

impl PointXYZRGB {
    /// Create a new colored point
    pub fn new(x: f32, y: f32, z: f32, r: u8, g: u8, b: u8) -> Self {
        Self {
            x,
            y,
            z,
            r,
            g,
            b,
            a: 255,
        }
    }

    /// Create from XYZ with white color
    pub fn from_xyz(xyz: PointXYZ) -> Self {
        Self {
            x: xyz.x,
            y: xyz.y,
            z: xyz.z,
            r: 255,
            g: 255,
            b: 255,
            a: 255,
        }
    }

    /// Get RGB as packed u32 (0xRRGGBBAA)
    pub fn rgb_packed(&self) -> u32 {
        ((self.r as u32) << 24) | ((self.g as u32) << 16) | ((self.b as u32) << 8) | (self.a as u32)
    }

    /// Get as PointXYZ
    pub fn xyz(&self) -> PointXYZ {
        PointXYZ {
            x: self.x,
            y: self.y,
            z: self.z,
        }
    }
}

/// 3D point with intensity (LiDAR format)
///
/// Common format for LiDAR sensors like Velodyne, Ouster, Livox.
///
/// Size: 16 bytes (aligned)
#[repr(C)]
#[derive(Debug, Clone, Copy, Default, PartialEq, Pod, Zeroable)]
pub struct PointXYZI {
    /// X coordinate (meters)
    pub x: f32,
    /// Y coordinate (meters)
    pub y: f32,
    /// Z coordinate (meters)
    pub z: f32,
    /// Intensity (reflectance, typically 0-255 or 0-65535)
    pub intensity: f32,
}

impl PointXYZI {
    /// Create a new point with intensity
    pub fn new(x: f32, y: f32, z: f32, intensity: f32) -> Self {
        Self { x, y, z, intensity }
    }

    /// Create from XYZ with zero intensity
    pub fn from_xyz(xyz: PointXYZ) -> Self {
        Self {
            x: xyz.x,
            y: xyz.y,
            z: xyz.z,
            intensity: 0.0,
        }
    }

    /// Get as PointXYZ
    pub fn xyz(&self) -> PointXYZ {
        PointXYZ {
            x: self.x,
            y: self.y,
            z: self.z,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_point_xyz_size_and_layout() {
        assert_eq!(std::mem::size_of::<PointXYZ>(), 12);
        // packed: alignment must be 1 (no padding)
        assert_eq!(std::mem::align_of::<PointXYZ>(), 1);
        // Exactly 3 * f32 with no gaps
        assert_eq!(std::mem::size_of::<PointXYZ>(), 3 * std::mem::size_of::<f32>());
    }

    #[test]
    fn test_point_xyzrgb_size_and_layout() {
        assert_eq!(std::mem::size_of::<PointXYZRGB>(), 16);
        // repr(C) with 4-byte alignment from the f32 fields
        assert_eq!(std::mem::align_of::<PointXYZRGB>(), 4);
        // Must be a power of 2 for efficient GPU transfer
        assert!(std::mem::size_of::<PointXYZRGB>().is_power_of_two());
    }

    #[test]
    fn test_point_xyzi_size_and_layout() {
        assert_eq!(std::mem::size_of::<PointXYZI>(), 16);
        assert_eq!(std::mem::align_of::<PointXYZI>(), 4);
        // Exactly 4 * f32 with no padding
        assert_eq!(std::mem::size_of::<PointXYZI>(), 4 * std::mem::size_of::<f32>());
        assert!(std::mem::size_of::<PointXYZI>().is_power_of_two());
    }

    #[test]
    fn test_point_distance() {
        let p = PointXYZ::new(3.0, 4.0, 0.0);
        assert!((p.distance() - 5.0).abs() < 0.001);
    }

    #[test]
    fn test_point_distance_to() {
        let a = PointXYZ::new(0.0, 0.0, 0.0);
        let b = PointXYZ::new(1.0, 2.0, 2.0);
        assert!((a.distance_to(&b) - 3.0).abs() < 0.001);
    }

    #[test]
    fn test_rgb_packed() {
        let p = PointXYZRGB::new(0.0, 0.0, 0.0, 255, 128, 64);
        let packed = p.rgb_packed();
        assert_eq!(packed >> 24, 255);
        assert_eq!((packed >> 16) & 0xFF, 128);
        assert_eq!((packed >> 8) & 0xFF, 64);
    }

    #[test]
    fn test_from_xyz_conversions() {
        let xyz = PointXYZ::new(1.0, 2.0, 3.0);

        // XYZ -> XYZRGB preserves coordinates, sets white color
        let rgb = PointXYZRGB::from_xyz(xyz);
        assert_eq!(rgb.x, 1.0);
        assert_eq!(rgb.y, 2.0);
        assert_eq!(rgb.z, 3.0);
        assert_eq!(rgb.r, 255);
        assert_eq!(rgb.g, 255);
        assert_eq!(rgb.b, 255);
        assert_eq!(rgb.a, 255);

        // XYZ -> XYZI preserves coordinates, sets zero intensity
        let intensity = PointXYZI::from_xyz(xyz);
        assert_eq!(intensity.x, 1.0);
        assert_eq!(intensity.y, 2.0);
        assert_eq!(intensity.z, 3.0);
        assert_eq!(intensity.intensity, 0.0);

        // Round-trip: XYZRGB -> xyz() -> from_xyz should preserve coordinates
        let back = PointXYZRGB::from_xyz(rgb.xyz());
        assert_eq!(back.x, rgb.x);
        assert_eq!(back.y, rgb.y);
        assert_eq!(back.z, rgb.z);

        // Round-trip: XYZI -> xyz() -> from_xyz should preserve coordinates
        let back_i = PointXYZI::from_xyz(intensity.xyz());
        assert_eq!(back_i.x, intensity.x);
        assert_eq!(back_i.y, intensity.y);
        assert_eq!(back_i.z, intensity.z);
    }

    #[test]
    fn test_point_pod_soundness() {
        // PointXYZ roundtrip through bytes
        let p = PointXYZ::new(1.0, 2.0, 3.0);
        let bytes: &[u8] = bytemuck::bytes_of(&p);
        assert_eq!(bytes.len(), 12);
        let recovered: PointXYZ = *bytemuck::from_bytes::<PointXYZ>(bytes);
        // Copy fields to avoid unaligned references on packed structs
        let (rx, ry, rz) = (recovered.x, recovered.y, recovered.z);
        assert_eq!(rx, 1.0);
        assert_eq!(ry, 2.0);
        assert_eq!(rz, 3.0);

        // PointXYZRGB roundtrip through bytes
        let rgb = PointXYZRGB::new(4.0, 5.0, 6.0, 128, 64, 32);
        let rgb_bytes = bytemuck::bytes_of(&rgb);
        assert_eq!(rgb_bytes.len(), 16);
        let rgb_rec: PointXYZRGB = *bytemuck::from_bytes::<PointXYZRGB>(rgb_bytes);
        let (rr_x, rr_r, rr_g, rr_b, rr_a) = (rgb_rec.x, rgb_rec.r, rgb_rec.g, rgb_rec.b, rgb_rec.a);
        assert_eq!(rr_x, 4.0);
        assert_eq!(rr_r, 128);
        assert_eq!(rr_g, 64);
        assert_eq!(rr_b, 32);
        assert_eq!(rr_a, 255);

        // PointXYZI roundtrip through bytes
        let pi = PointXYZI::new(7.0, 8.0, 9.0, 100.5);
        let pi_bytes = bytemuck::bytes_of(&pi);
        assert_eq!(pi_bytes.len(), 16);
        let pi_rec: PointXYZI = *bytemuck::from_bytes::<PointXYZI>(pi_bytes);
        let (pr_x, pr_i) = (pi_rec.x, pi_rec.intensity);
        assert_eq!(pr_x, 7.0);
        assert_eq!(pr_i, 100.5);
    }

    #[test]
    fn test_point_default_is_zeroed() {
        let xyz = PointXYZ::default();
        let (dx, dy, dz) = (xyz.x, xyz.y, xyz.z);
        assert_eq!(dx, 0.0);
        assert_eq!(dy, 0.0);
        assert_eq!(dz, 0.0);
        // Default must differ from a non-zero point
        assert_ne!(xyz, PointXYZ::new(1.0, 0.0, 0.0));

        let rgb = PointXYZRGB::default();
        let (rgb_dx, rgb_dr) = (rgb.x, rgb.r);
        assert_eq!(rgb_dx, 0.0);
        assert_eq!(rgb_dr, 0);

        let xyzi = PointXYZI::default();
        let di = xyzi.intensity;
        assert_eq!(di, 0.0);
    }

    #[test]
    fn test_point_clone_independence() {
        let mut a = PointXYZ::new(1.0, 2.0, 3.0);
        let b = a;
        a.x = 99.0;
        // Copy fields to locals to avoid unaligned references on packed struct
        let bx = { b.x };
        let ax = { a.x };
        assert_eq!(bx, 1.0, "copy must be independent of original");
        assert_eq!(ax, 99.0);
    }
}
