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
    fn test_point_xyz_size() {
        assert_eq!(std::mem::size_of::<PointXYZ>(), 12);
    }

    #[test]
    fn test_point_xyzrgb_size() {
        assert_eq!(std::mem::size_of::<PointXYZRGB>(), 16);
    }

    #[test]
    fn test_point_xyzi_size() {
        assert_eq!(std::mem::size_of::<PointXYZI>(), 16);
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
        let rgb = PointXYZRGB::from_xyz(xyz);
        assert_eq!(rgb.x, 1.0);
        assert_eq!(rgb.r, 255); // white default

        let intensity = PointXYZI::from_xyz(xyz);
        assert_eq!(intensity.z, 3.0);
        assert_eq!(intensity.intensity, 0.0); // zero default
    }

    #[test]
    fn test_point_pod_soundness() {
        let p = PointXYZ::new(1.0, 2.0, 3.0);
        let bytes: &[u8] = bytemuck::bytes_of(&p);
        assert_eq!(bytes.len(), 12);
        let recovered: PointXYZ = *bytemuck::from_bytes::<PointXYZ>(bytes);
        let x = recovered.x;
        assert_eq!(x, 1.0);
    }
}
