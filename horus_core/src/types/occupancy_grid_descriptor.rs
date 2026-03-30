//! Pod occupancy grid descriptor for zero-copy ring buffer transport
//!
//! `OccupancyGridDescriptor` is a fixed-size (320 byte) `repr(C)` descriptor
//! that flows through the ring buffer via the ~50ns Pod path. Actual grid data
//! (i8 occupancy cells) lives in a `TensorPool`.
//!
//! Users should use `OccupancyGrid` from `horus_library` which wraps this with
//! data access and coordinate transform methods.

use bytemuck::{Pod, Zeroable};
use serde::{Deserialize, Serialize};

use super::tensor::Tensor;

fn default_metadata() -> [u8; 64] {
    [0; 64]
}

fn serialize_metadata<S: serde::Serializer>(data: &[u8; 64], s: S) -> Result<S::Ok, S::Error> {
    s.serialize_bytes(data)
}

/// Occupancy grid descriptor — Pod, 320 bytes.
///
/// Contains a `Tensor` (shape `[H, W]`, dtype I8) plus map metadata
/// (resolution, origin, frame_id, timestamp). This is what flows through
/// the ring buffer; grid cell data stays in shared memory.
///
/// # Layout (320 bytes, repr(C))
///
/// ```text
/// inner:         Tensor       (168 bytes)
/// timestamp_ns:  u64          (8 bytes)
/// resolution:    f32          (4 bytes)  — meters per cell
/// width:         u32          (4 bytes)  — grid width in cells
/// height:        u32          (4 bytes)  — grid height in cells
/// _pad1:         u32          (4 bytes)  — alignment padding
/// origin_x:      f64          (8 bytes)  — map origin X (meters)
/// origin_y:      f64          (8 bytes)  — map origin Y (meters)
/// origin_theta:  f64          (8 bytes)  — map origin orientation (radians)
/// frame_id:      [u8; 32]     (32 bytes) — coordinate frame, null-terminated
/// metadata:      [u8; 64]     (64 bytes) — user metadata
/// _reserved:     [u8; 8]      (8 bytes)  — future use
/// Total:                       320 bytes
/// ```
#[repr(C)]
#[derive(Clone, Copy, Debug, Serialize, Deserialize)]
pub struct OccupancyGridDescriptor {
    /// Inner tensor: shape [H, W], dtype I8, data in pool
    inner: Tensor,
    /// Timestamp in nanoseconds since epoch
    timestamp_ns: u64,
    /// Map resolution in meters per cell
    resolution: f32,
    /// Grid width in cells
    width: u32,
    /// Grid height in cells
    height: u32,
    /// Alignment padding
    #[serde(skip)]
    _pad1: u32,
    /// Map origin X position in meters
    origin_x: f64,
    /// Map origin Y position in meters
    origin_y: f64,
    /// Map origin orientation in radians
    origin_theta: f64,
    /// Coordinate frame reference (null-terminated)
    frame_id: [u8; 32],
    /// User-defined metadata
    #[serde(
        skip_deserializing,
        serialize_with = "serialize_metadata",
        default = "default_metadata"
    )]
    metadata: [u8; 64],
    /// Reserved for future use
    #[serde(skip)]
    _reserved: [u8; 8],
}

// Safety: OccupancyGridDescriptor is repr(C), all fields are Pod, no implicit padding.
// 168 + 8 + 4 + 4 + 4 + 4 + 8 + 8 + 8 + 32 + 64 + 8 = 320 bytes, 320 % 8 = 0.
unsafe impl Zeroable for OccupancyGridDescriptor {}
unsafe impl Pod for OccupancyGridDescriptor {}

impl Default for OccupancyGridDescriptor {
    fn default() -> Self {
        Self {
            inner: Tensor::default(),
            timestamp_ns: 0,
            resolution: 0.05, // 5cm default
            width: 0,
            height: 0,
            _pad1: 0,
            origin_x: 0.0,
            origin_y: 0.0,
            origin_theta: 0.0,
            frame_id: [0; 32],
            metadata: [0; 64],
            _reserved: [0; 8],
        }
    }
}

impl OccupancyGridDescriptor {
    /// Sanitize a descriptor read from untrusted bytes (SHM, network, file).
    ///
    /// Clamps inner tensor fields to valid ranges and validates resolution.
    #[inline]
    pub fn sanitize_from_shm(&mut self) {
        self.inner.sanitize_from_shm();
        // Clamp resolution to a sane range (1um to 1km per cell)
        if !self.resolution.is_finite() || self.resolution <= 0.0 {
            self.resolution = 0.05;
        }
    }

    /// Create a new occupancy grid descriptor from a pre-built tensor + metadata.
    ///
    /// The tensor should have shape `[height, width]` with dtype `I8`.
    pub fn new(tensor: Tensor, resolution: f32, width: u32, height: u32) -> Self {
        Self {
            inner: tensor,
            timestamp_ns: 0,
            resolution,
            width,
            height,
            _pad1: 0,
            origin_x: 0.0,
            origin_y: 0.0,
            origin_theta: 0.0,
            frame_id: [0; 32],
            metadata: [0; 64],
            _reserved: [0; 8],
        }
    }

    // === Grid metadata accessors ===

    /// Map resolution in meters per cell.
    #[inline]
    pub fn resolution(&self) -> f32 {
        self.resolution
    }

    /// Grid width in cells.
    #[inline]
    pub fn width(&self) -> u32 {
        self.width
    }

    /// Grid height in cells.
    #[inline]
    pub fn height(&self) -> u32 {
        self.height
    }

    /// Total number of cells.
    #[inline]
    pub fn cell_count(&self) -> usize {
        self.width as usize * self.height as usize
    }

    // === Origin accessors ===

    /// Map origin X position in meters.
    #[inline]
    pub fn origin_x(&self) -> f64 {
        self.origin_x
    }

    /// Map origin Y position in meters.
    #[inline]
    pub fn origin_y(&self) -> f64 {
        self.origin_y
    }

    /// Map origin orientation in radians.
    #[inline]
    pub fn origin_theta(&self) -> f64 {
        self.origin_theta
    }

    /// Set map origin (X, Y, theta).
    pub fn set_origin(&mut self, x: f64, y: f64, theta: f64) {
        self.origin_x = x;
        self.origin_y = y;
        self.origin_theta = theta;
    }

    // === Metadata ===

    /// Get user metadata bytes.
    #[inline]
    pub fn metadata(&self) -> &[u8; 64] {
        &self.metadata
    }

    /// Set user metadata bytes.
    pub fn set_metadata(&mut self, data: &[u8]) {
        let len = data.len().min(64);
        self.metadata = [0; 64];
        self.metadata[..len].copy_from_slice(&data[..len]);
    }

    crate::impl_tensor_accessors!();
    crate::impl_timestamp_field!();
    crate::impl_frame_id_field!();
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::types::{Device, TensorDtype};

    #[test]
    fn test_occupancy_grid_size_and_alignment() {
        let size = std::mem::size_of::<OccupancyGridDescriptor>();
        let align = std::mem::align_of::<OccupancyGridDescriptor>();
        assert_eq!(
            size, 320,
            "OccupancyGridDescriptor must be exactly 320 bytes"
        );
        assert_eq!(align, 8, "alignment must be 8 from u64/f64 fields");
        assert_eq!(size % align, 0, "size must be a multiple of alignment");
        assert_eq!(size % 32, 0, "320 = 32 * 10, SIMD-friendly");
    }

    #[test]
    fn test_occupancy_grid_pod() {
        let tensor = Tensor::new(1, 42, 99, 0, &[1000, 1000], TensorDtype::I8, Device::cpu());
        let mut grid = OccupancyGridDescriptor::new(tensor, 0.05, 1000, 1000);
        grid.set_frame_id("map");
        grid.set_timestamp_ns(1_000_000_000);
        grid.set_origin(1.5, 2.5, 0.785);

        let bytes: &[u8] = bytemuck::bytes_of(&grid);
        assert_eq!(bytes.len(), 320);
        let recovered: &OccupancyGridDescriptor = bytemuck::from_bytes(bytes);
        assert_eq!(recovered.width(), 1000);
        assert_eq!(recovered.height(), 1000);
        assert_eq!(recovered.resolution(), 0.05);
        assert_eq!(recovered.frame_id(), "map");
        assert_eq!(recovered.timestamp_ns(), 1_000_000_000);
        assert_eq!(recovered.origin_x(), 1.5);
        assert_eq!(recovered.origin_y(), 2.5);
        assert_eq!(recovered.origin_theta(), 0.785);
        assert_eq!(recovered.cell_count(), 1_000_000);
    }

    #[test]
    fn test_occupancy_grid_default() {
        let grid = OccupancyGridDescriptor::default();
        assert_eq!(grid.width(), 0);
        assert_eq!(grid.height(), 0);
        assert_eq!(grid.resolution(), 0.05);
        assert_eq!(grid.origin_x(), 0.0);
        assert_eq!(grid.origin_y(), 0.0);
        assert_eq!(grid.origin_theta(), 0.0);
        assert_eq!(grid.frame_id(), "");
        assert_eq!(grid.timestamp_ns(), 0);
    }

    #[test]
    fn test_occupancy_grid_frame_id() {
        let mut grid = OccupancyGridDescriptor::default();
        grid.set_frame_id("odom");
        assert_eq!(grid.frame_id(), "odom");
    }

    #[test]
    fn test_occupancy_grid_metadata() {
        let mut grid = OccupancyGridDescriptor::default();
        let meta = b"layer=costmap;source=lidar";
        grid.set_metadata(meta);
        assert_eq!(&grid.metadata()[..meta.len()], meta);
    }

    #[test]
    fn test_occupancy_grid_sanitize() {
        let mut grid = OccupancyGridDescriptor::default();
        grid.resolution = f32::NAN;
        grid.sanitize_from_shm();
        assert_eq!(
            grid.resolution(),
            0.05,
            "NaN resolution should reset to default"
        );

        grid.resolution = -1.0;
        grid.sanitize_from_shm();
        assert_eq!(
            grid.resolution(),
            0.05,
            "negative resolution should reset to default"
        );
    }

    #[test]
    fn test_occupancy_grid_serde_roundtrip() {
        let tensor = Tensor::new(1, 42, 1, 0, &[500, 500], TensorDtype::I8, Device::cpu());
        let mut grid = OccupancyGridDescriptor::new(tensor, 0.1, 500, 500);
        grid.set_frame_id("map");
        grid.set_timestamp_ns(123456789);
        grid.set_origin(10.0, 20.0, 1.57);

        let json = serde_json::to_string(&grid).unwrap();
        let recovered: OccupancyGridDescriptor = serde_json::from_str(&json).unwrap();
        assert_eq!(recovered.width(), 500);
        assert_eq!(recovered.height(), 500);
        assert_eq!(recovered.resolution(), 0.1);
        assert_eq!(recovered.frame_id(), "map");
        assert_eq!(recovered.timestamp_ns(), 123456789);
        assert_eq!(recovered.origin_x(), 10.0);
        assert_eq!(recovered.origin_y(), 20.0);
    }
}
