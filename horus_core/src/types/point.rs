//! Pod point element types for zero-copy point cloud IPC
//!
//! Re-exports canonical point types from softmata-core.
//! HORUS-specific extension methods are added via trait impls.

pub use softmata_core::sensor::{PointXYZ, PointXYZI, PointXYZRGB};

/// HORUS-specific extensions for PointXYZ.
pub trait PointXYZExt {
    fn distance(&self) -> f32;
    fn distance_to(&self, other: &PointXYZ) -> f32;
}

impl PointXYZExt for PointXYZ {
    fn distance(&self) -> f32 {
        (self.x * self.x + self.y * self.y + self.z * self.z).sqrt()
    }

    fn distance_to(&self, other: &PointXYZ) -> f32 {
        let dx = self.x - other.x;
        let dy = self.y - other.y;
        let dz = self.z - other.z;
        (dx * dx + dy * dy + dz * dz).sqrt()
    }
}

/// HORUS-specific extensions for PointXYZRGB.
pub trait PointXYZRGBExt {
    fn from_xyz(xyz: PointXYZ) -> PointXYZRGB;
    fn rgb_packed(&self) -> u32;
    fn xyz(&self) -> PointXYZ;
}

impl PointXYZRGBExt for PointXYZRGB {
    fn from_xyz(xyz: PointXYZ) -> PointXYZRGB {
        PointXYZRGB::new(xyz.x, xyz.y, xyz.z, 255, 255, 255)
    }

    fn rgb_packed(&self) -> u32 {
        ((self.r as u32) << 24) | ((self.g as u32) << 16) | ((self.b as u32) << 8) | (self.a as u32)
    }

    fn xyz(&self) -> PointXYZ {
        PointXYZ::new(self.x, self.y, self.z)
    }
}

/// HORUS-specific extensions for PointXYZI.
pub trait PointXYZIExt {
    fn from_xyz(xyz: PointXYZ) -> PointXYZI;
    fn xyz(&self) -> PointXYZ;
}

impl PointXYZIExt for PointXYZI {
    fn from_xyz(xyz: PointXYZ) -> PointXYZI {
        PointXYZI::new(xyz.x, xyz.y, xyz.z, 0.0)
    }

    fn xyz(&self) -> PointXYZ {
        PointXYZ::new(self.x, self.y, self.z)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_point_xyz_size() {
        assert_eq!(std::mem::size_of::<PointXYZ>(), 12);
        assert_eq!(
            std::mem::size_of::<PointXYZ>(),
            3 * std::mem::size_of::<f32>()
        );
    }

    #[test]
    fn test_point_xyz_rgb_size() {
        assert_eq!(std::mem::size_of::<PointXYZRGB>(), 16);
    }

    #[test]
    fn test_point_xyzi_size() {
        assert_eq!(std::mem::size_of::<PointXYZI>(), 16);
    }

    #[test]
    fn test_distance() {
        let p = PointXYZ::new(3.0, 4.0, 0.0);
        assert!((p.distance() - 5.0).abs() < 1e-5);
    }

    #[test]
    fn test_distance_to() {
        let a = PointXYZ::new(0.0, 0.0, 0.0);
        let b = PointXYZ::new(1.0, 0.0, 0.0);
        assert!((a.distance_to(&b) - 1.0).abs() < 1e-5);
    }

    #[test]
    fn test_rgb_packed() {
        let p = PointXYZRGB::new(0.0, 0.0, 0.0, 255, 128, 64);
        let packed = p.rgb_packed();
        assert_eq!((packed >> 24) & 0xFF, 255);
        assert_eq!((packed >> 16) & 0xFF, 128);
        assert_eq!((packed >> 8) & 0xFF, 64);
    }

    #[test]
    fn test_xyz_conversion() {
        let xyz = PointXYZ::new(1.0, 2.0, 3.0);
        let rgb = <PointXYZRGB as PointXYZRGBExt>::from_xyz(xyz);
        assert_eq!(rgb.x, 1.0);
        assert_eq!(rgb.r, 255);
        let back = rgb.xyz();
        assert_eq!(back.x, 1.0);
    }
}
