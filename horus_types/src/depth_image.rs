//! Unified Pod depth image descriptor for zero-copy depth sensor pipelines
//!
//! `DepthImage` is a fixed-size (288 byte) `repr(C)` descriptor that flows
//! through `Topic<DepthImage>` via the ~50ns Pod path. Actual depth data
//! lives in a `TensorPool`.
//!
//! For data access (get_depth, depth_statistics), use `DepthImage` from `horus_core`.
//! in `horus_core`.

use bytemuck::{Pod, Zeroable};
use serde::{Deserialize, Serialize};

use crate::tensor::HorusTensor;
use crate::TensorDtype;

/// Unified depth image descriptor — Pod, 288 bytes.
///
/// Contains a `HorusTensor` (shape `[H, W]`) plus depth metadata.
/// Dtype indicates units: F32 = meters, U16 = millimeters.
///
/// # Layout (288 bytes, repr(C))
///
/// ```text
/// inner:        HorusTensor  (232 bytes)
/// timestamp_ns: u64          (8 bytes)
/// depth_scale:  f32          (4 bytes)
/// min_depth:    u16          (2 bytes)
/// max_depth:    u16          (2 bytes)
/// frame_id:     [u8; 32]     (32 bytes)
/// _reserved:    [u8; 8]      (8 bytes)
/// Total:                      288 bytes
/// ```
#[repr(C)]
#[derive(Clone, Copy, Debug, Serialize, Deserialize)]
pub struct DepthImageDescriptor {
    /// Inner tensor: shape [H, W], data in pool
    inner: HorusTensor,
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
// 232 + 8 + 4 + 2 + 2 + 32 + 8 = 288 bytes, 288 % 8 = 0.
unsafe impl Zeroable for DepthImageDescriptor {}
unsafe impl Pod for DepthImageDescriptor {}

impl Default for DepthImageDescriptor {
    fn default() -> Self {
        Self {
            inner: HorusTensor::default(),
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
    pub fn new(tensor: HorusTensor) -> Self {
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

    /// Create with explicit depth scale.
    pub fn with_scale(mut self, scale: f32) -> Self {
        self.depth_scale = scale;
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

    /// Data type of depth values.
    #[inline]
    pub fn dtype(&self) -> TensorDtype {
        self.inner.dtype
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

    /// Total number of pixels (height * width).
    #[inline]
    pub fn pixel_count(&self) -> u64 {
        self.height() as u64 * self.width() as u64
    }

    /// Total bytes of depth data.
    #[inline]
    pub fn nbytes(&self) -> u64 {
        self.inner.nbytes()
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

    /// Whether tensor data is on CPU.
    #[inline]
    pub fn is_cpu(&self) -> bool {
        self.inner.is_cpu()
    }

    /// Whether tensor data is on CUDA.
    #[inline]
    pub fn is_cuda(&self) -> bool {
        self.inner.is_cuda()
    }

    /// Timestamp in nanoseconds since epoch.
    #[inline]
    pub fn timestamp_ns(&self) -> u64 {
        self.timestamp_ns
    }

    /// Set timestamp.
    #[inline]
    pub fn set_timestamp_ns(&mut self, ts: u64) {
        self.timestamp_ns = ts;
    }

    /// Get frame ID as string.
    pub fn frame_id(&self) -> &str {
        let end = self.frame_id.iter().position(|&b| b == 0).unwrap_or(32);
        std::str::from_utf8(&self.frame_id[..end]).unwrap_or("")
    }

    /// Set frame ID from string.
    pub fn set_frame_id(&mut self, id: &str) {
        let bytes = id.as_bytes();
        let len = bytes.len().min(31);
        self.frame_id = [0; 32];
        self.frame_id[..len].copy_from_slice(&bytes[..len]);
    }

    /// Get the inner tensor descriptor.
    #[inline]
    pub fn tensor(&self) -> &HorusTensor {
        &self.inner
    }

    /// Get a mutable reference to the inner tensor descriptor.
    #[inline]
    pub fn tensor_mut(&mut self) -> &mut HorusTensor {
        &mut self.inner
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::Device;

    #[test]
    fn test_depth_image_size() {
        assert_eq!(
            std::mem::size_of::<DepthImageDescriptor>(),
            288,
            "DepthImageDescriptor must be exactly 288 bytes"
        );
    }

    #[test]
    fn test_depth_image_pod() {
        let di = DepthImageDescriptor::default();
        let bytes: &[u8] = bytemuck::bytes_of(&di);
        assert_eq!(bytes.len(), 288);
        let _recovered: &DepthImageDescriptor = bytemuck::from_bytes(bytes);
    }

    #[test]
    fn test_depth_image_f32() {
        let tensor = HorusTensor::new(1, 0, 0, 0, &[480, 640], TensorDtype::F32, Device::cpu());
        let di = DepthImageDescriptor::new(tensor);
        assert_eq!(di.height(), 480);
        assert_eq!(di.width(), 640);
        assert!(di.is_meters());
        assert!(!di.is_millimeters());
    }

    #[test]
    fn test_depth_image_u16() {
        let tensor = HorusTensor::new(1, 0, 0, 0, &[720, 1280], TensorDtype::U16, Device::cpu());
        let di = DepthImageDescriptor::new(tensor);
        assert_eq!(di.height(), 720);
        assert_eq!(di.width(), 1280);
        assert!(di.is_millimeters());
        assert!(!di.is_meters());
    }

    #[test]
    fn test_depth_image_with_range() {
        let tensor = HorusTensor::new(1, 0, 0, 0, &[100, 100], TensorDtype::U16, Device::cpu());
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
