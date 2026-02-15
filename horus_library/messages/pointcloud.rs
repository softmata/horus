//! Point cloud types for zero-copy IPC
//!
//! Pod/Zeroable fixed-size point types for 3D point cloud data from LiDAR,
//! depth cameras, etc. These are suitable for shared memory transport.
//!
//! For the Serde-based `PointCloud` with flexible PointField descriptors,
//! see `perception.rs`.

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

/// Point cloud header for array transmission
///
/// This header is sent before the point data array via IPC.
///
/// Size: 64 bytes
#[repr(C)]
#[derive(Debug, Clone, Copy, Default, Pod, Zeroable)]
pub struct PointCloudHeader {
    /// Number of points in the cloud
    pub num_points: u64,
    /// Point type: 0=XYZ, 1=XYZRGB, 2=XYZI
    pub point_type: u32,
    /// Bytes per point
    pub point_stride: u32,
    /// Timestamp (nanoseconds since epoch)
    pub timestamp_ns: u64,
    /// Sequence number
    pub seq: u64,
    /// Frame ID (sensor/coordinate frame)
    pub frame_id: [u8; 32],
}

impl PointCloudHeader {
    /// Create header for PointXYZ cloud
    pub fn xyz(num_points: u64) -> Self {
        Self {
            num_points,
            point_type: 0,
            point_stride: std::mem::size_of::<PointXYZ>() as u32,
            timestamp_ns: 0,
            seq: 0,
            frame_id: [0u8; 32],
        }
    }

    /// Create header for PointXYZRGB cloud
    pub fn xyzrgb(num_points: u64) -> Self {
        Self {
            num_points,
            point_type: 1,
            point_stride: std::mem::size_of::<PointXYZRGB>() as u32,
            timestamp_ns: 0,
            seq: 0,
            frame_id: [0u8; 32],
        }
    }

    /// Create header for PointXYZI cloud
    pub fn xyzi(num_points: u64) -> Self {
        Self {
            num_points,
            point_type: 2,
            point_stride: std::mem::size_of::<PointXYZI>() as u32,
            timestamp_ns: 0,
            seq: 0,
            frame_id: [0u8; 32],
        }
    }

    /// Set frame ID
    pub fn with_frame_id(mut self, frame_id: &str) -> Self {
        let bytes = frame_id.as_bytes();
        let len = bytes.len().min(31);
        self.frame_id[..len].copy_from_slice(&bytes[..len]);
        self.frame_id[len..].fill(0);
        self
    }

    /// Set timestamp
    pub fn with_timestamp(mut self, timestamp_ns: u64) -> Self {
        self.timestamp_ns = timestamp_ns;
        self
    }

    /// Get frame ID as string
    pub fn get_frame_id(&self) -> &str {
        let end = self.frame_id.iter().position(|&b| b == 0).unwrap_or(32);
        std::str::from_utf8(&self.frame_id[..end]).unwrap_or("")
    }

    /// Calculate total data size
    pub fn data_size(&self) -> usize {
        (self.num_points as usize) * (self.point_stride as usize)
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
    fn test_header_size() {
        assert_eq!(std::mem::size_of::<PointCloudHeader>(), 64);
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
    fn test_header_data_size() {
        let header = PointCloudHeader::xyz(1000);
        assert_eq!(header.data_size(), 1000 * 12);

        let header_rgb = PointCloudHeader::xyzrgb(500);
        assert_eq!(header_rgb.data_size(), 500 * 16);
    }

    #[test]
    fn test_header_frame_id() {
        let header = PointCloudHeader::xyz(100).with_frame_id("lidar_front");
        assert_eq!(header.get_frame_id(), "lidar_front");
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
}
