//! Unified Pod depth image descriptor for zero-copy depth sensor pipelines
//!
//! `DepthImage` is a fixed-size (224 byte) `repr(C)` descriptor that flows
//! through `Topic<DepthImage>` via the ~50ns Pod path. Actual depth data
//! lives in a `TensorPool`.
//!
//! For data access (get_depth, depth_statistics), use `DepthImage` from `horus_core`.
//! in `horus_core`.

use bytemuck::{Pod, Zeroable};
use serde::{Deserialize, Serialize};

use super::dtype::TensorDtype;
use super::tensor::Tensor;

/// Unified depth image descriptor — Pod, 224 bytes.
///
/// Contains a `Tensor` (shape `[H, W]`) plus depth metadata.
/// Dtype indicates units: F32 = meters, U16 = millimeters.
///
/// # Layout (224 bytes, repr(C))
///
/// ```text
/// inner:        Tensor  (168 bytes)
/// timestamp_ns: u64          (8 bytes)
/// depth_scale:  f32          (4 bytes)
/// min_depth:    u16          (2 bytes)
/// max_depth:    u16          (2 bytes)
/// frame_id:     [u8; 32]     (32 bytes)
/// _reserved:    [u8; 8]      (8 bytes)
/// Total:                      224 bytes
/// ```
#[repr(C)]
#[derive(Clone, Copy, Debug, Serialize, Deserialize)]
pub struct DepthImageDescriptor {
    /// Inner tensor: shape [H, W], data in pool
    inner: Tensor,
    /// Timestamp in nanoseconds since epoch
    timestamp_ns: u64,
    /// Depth scale (mm per unit for U16, usually 1.0)
    depth_scale: f32,
    /// Minimum reliable depth value
    min_depth: u16,
    /// Maximum reliable depth value
    max_depth: u16,
    /// Frame ID (camera identifier, null-terminated)
    frame_id: [u8; 32],
    #[serde(skip)]
    _reserved: [u8; 8],
}

// Safety: DepthImage is repr(C), all fields are Pod, no implicit padding.
// 168 + 8 + 4 + 2 + 2 + 32 + 8 = 224 bytes, 224 % 8 = 0.
unsafe impl Zeroable for DepthImageDescriptor {}
unsafe impl Pod for DepthImageDescriptor {}

impl Default for DepthImageDescriptor {
    fn default() -> Self {
        Self {
            inner: Tensor::default(),
            timestamp_ns: 0,
            depth_scale: 1.0,
            min_depth: 200,   // 20cm
            max_depth: 10000, // 10m
            frame_id: [0; 32],
            _reserved: [0; 8],
        }
    }
}

impl DepthImageDescriptor {
    /// Create a depth image descriptor from a tensor.
    ///
    /// Tensor shape should be `[H, W]` with dtype F32 (meters) or U16 (mm).
    pub fn new(tensor: Tensor) -> Self {
        Self {
            inner: tensor,
            ..Default::default()
        }
    }

    /// Create with explicit depth range.
    pub fn with_range(mut self, min_depth: u16, max_depth: u16) -> Self {
        self.min_depth = min_depth;
        self.max_depth = max_depth;
        self
    }

    // === Accessors ===

    /// Image height in pixels (tensor dimension 0).
    #[inline]
    pub fn height(&self) -> u32 {
        if self.inner.ndim >= 1 {
            self.inner.shape[0] as u32
        } else {
            0
        }
    }

    /// Image width in pixels (tensor dimension 1).
    #[inline]
    pub fn width(&self) -> u32 {
        if self.inner.ndim >= 2 {
            self.inner.shape[1] as u32
        } else {
            0
        }
    }

    /// Whether depth values are in meters (F32).
    #[inline]
    pub fn is_meters(&self) -> bool {
        self.inner.dtype == TensorDtype::F32
    }

    /// Whether depth values are in millimeters (U16).
    #[inline]
    pub fn is_millimeters(&self) -> bool {
        self.inner.dtype == TensorDtype::U16
    }

    /// Depth scale (mm per unit).
    #[inline]
    pub fn depth_scale(&self) -> f32 {
        self.depth_scale
    }

    /// Minimum reliable depth value.
    #[inline]
    pub fn min_depth(&self) -> u16 {
        self.min_depth
    }

    /// Maximum reliable depth value.
    #[inline]
    pub fn max_depth(&self) -> u16 {
        self.max_depth
    }

    crate::impl_tensor_accessors!();
    crate::impl_timestamp_field!();
    crate::impl_frame_id_field!();
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::types::Device;

    #[test]
    fn test_depth_image_size_and_alignment() {
        let size = std::mem::size_of::<DepthImageDescriptor>();
        let align = std::mem::align_of::<DepthImageDescriptor>();
        assert_eq!(size, 224, "DepthImageDescriptor must be exactly 224 bytes");
        assert_eq!(align, 8, "alignment must be 8 from u64 fields");
        assert_eq!(size % align, 0, "size must be a multiple of alignment");
        // Same size as ImageDescriptor (both 224 bytes) for uniform ring buffer slots
        assert_eq!(
            size,
            std::mem::size_of::<super::super::image_descriptor::ImageDescriptor>()
        );
    }

    #[test]
    fn test_depth_image_pod() {
        // Roundtrip a configured descriptor through bytes
        let tensor = Tensor::new(1, 0, 0, 0, &[480, 640], TensorDtype::F32, Device::cpu());
        let mut di = DepthImageDescriptor::new(tensor).with_range(200, 10000);
        di.set_frame_id("depth0");
        di.set_timestamp_ns(42);

        let bytes: &[u8] = bytemuck::bytes_of(&di);
        assert_eq!(bytes.len(), 224);
        let recovered: &DepthImageDescriptor = bytemuck::from_bytes(bytes);
        assert_eq!(recovered.height(), 480);
        assert_eq!(recovered.width(), 640);
        assert_eq!(recovered.min_depth(), 200);
        assert_eq!(recovered.max_depth(), 10000);
        assert_eq!(recovered.frame_id(), "depth0");
        assert_eq!(recovered.timestamp_ns(), 42);
    }

    #[test]
    fn test_depth_image_default_has_sensible_range() {
        let di = DepthImageDescriptor::default();
        // Default range: 20cm to 10m (200mm to 10000mm)
        assert_eq!(di.min_depth(), 200);
        assert_eq!(di.max_depth(), 10000);
        assert!(
            di.min_depth() < di.max_depth(),
            "min_depth must be less than max_depth"
        );
        assert_eq!(di.depth_scale(), 1.0);
    }

    #[test]
    fn test_depth_image_f32() {
        let tensor = Tensor::new(1, 0, 0, 0, &[480, 640], TensorDtype::F32, Device::cpu());
        let di = DepthImageDescriptor::new(tensor);
        assert_eq!(di.height(), 480);
        assert_eq!(di.width(), 640);
        assert!(di.is_meters());
        assert!(!di.is_millimeters());
    }

    #[test]
    fn test_depth_image_u16() {
        let tensor = Tensor::new(1, 0, 0, 0, &[720, 1280], TensorDtype::U16, Device::cpu());
        let di = DepthImageDescriptor::new(tensor);
        assert_eq!(di.height(), 720);
        assert_eq!(di.width(), 1280);
        assert!(di.is_millimeters());
        assert!(!di.is_meters());
    }

    #[test]
    fn test_depth_image_with_range() {
        let tensor = Tensor::new(1, 0, 0, 0, &[100, 100], TensorDtype::U16, Device::cpu());
        let di = DepthImageDescriptor::new(tensor).with_range(100, 5000);
        assert_eq!(di.min_depth(), 100);
        assert_eq!(di.max_depth(), 5000);
    }

    #[test]
    fn test_depth_image_frame_id() {
        let mut di = DepthImageDescriptor::default();
        di.set_frame_id("depth_camera");
        assert_eq!(di.frame_id(), "depth_camera");
    }
}
