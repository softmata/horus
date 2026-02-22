//! Unified Pod point cloud descriptor for zero-copy lidar/depth pipelines
//!
//! `PointCloud` is a fixed-size (336 byte) `repr(C)` descriptor that flows
//! through `Topic<PointCloud>` via the ~50ns Pod path. Actual point data
//! lives in a `TensorPool`.
//!
//! For data access (point_at, extract_xyz), use `PointCloud` from `horus_core`.

use bytemuck::{Pod, Zeroable};
use serde::{Deserialize, Serialize};

use crate::tensor::HorusTensor;
use crate::TensorDtype;

/// Unified point cloud descriptor — Pod, 336 bytes.
///
/// Contains a `HorusTensor` (shape `[N, K]`) plus field metadata.
/// The tensor holds N points with K fields each (3 for XYZ, 4 for XYZI, etc.).
///
/// # Layout (336 bytes, repr(C))
///
/// ```text
/// inner:         HorusTensor  (232 bytes)
/// timestamp_ns:  u64          (8 bytes)
/// field_names:   [[u8; 4]; 8] (32 bytes)  — compact field names ("x\0\0\0", etc.)
/// field_offsets:  [u16; 8]    (16 bytes)
/// field_dtypes:   [u8; 8]     (8 bytes)   — TensorDtype as u8 per field
/// field_count:    u8          (1 byte)
/// is_dense:       u8          (1 byte)
/// _pad:           [u8; 2]     (2 bytes)
/// frame_id:       [u8; 32]    (32 bytes)
/// _reserved:      [u8; 4]     (4 bytes)
/// Total:                       336 bytes
/// ```
#[repr(C)]
#[derive(Clone, Copy, Debug, Serialize, Deserialize, Default)]
pub struct PointCloudDescriptor {
    /// Inner tensor: shape [N, K], data in pool
    inner: HorusTensor,
    /// Timestamp in nanoseconds since epoch
    timestamp_ns: u64,
    /// Compact field names (4 bytes each, null-terminated)
    field_names: [[u8; 4]; 8],
    /// Byte offset of each field within a point
    field_offsets: [u16; 8],
    /// TensorDtype as u8 for each field
    field_dtypes: [u8; 8],
    /// Number of defined fields (0-8)
    field_count: u8,
    /// Whether the cloud is dense (no invalid points)
    is_dense: u8,
    #[serde(skip)]
    _pad: [u8; 2],
    /// Coordinate frame reference (null-terminated)
    frame_id: [u8; 32],
    #[serde(skip)]
    _reserved: [u8; 4],
}

// Safety: PointCloud is repr(C), all fields are Pod, no implicit padding.
// 232 + 8 + 32 + 16 + 8 + 1 + 1 + 2 + 32 + 4 = 336, 336 % 8 = 0.
unsafe impl Zeroable for PointCloudDescriptor {}
unsafe impl Pod for PointCloudDescriptor {}

impl PointCloudDescriptor {
    /// Create an XYZ point cloud descriptor.
    ///
    /// Tensor shape should be `[N, 3]` with dtype F32.
    pub fn xyz(tensor: HorusTensor) -> Self {
        let mut pc = Self::from_tensor(tensor);
        pc.set_field(0, b"x\0\0\0", 0, TensorDtype::F32);
        pc.set_field(1, b"y\0\0\0", 4, TensorDtype::F32);
        pc.set_field(2, b"z\0\0\0", 8, TensorDtype::F32);
        pc.field_count = 3;
        pc.is_dense = 1;
        pc
    }

    /// Create an XYZI point cloud descriptor.
    ///
    /// Tensor shape should be `[N, 4]` with dtype F32.
    pub fn xyzi(tensor: HorusTensor) -> Self {
        let mut pc = Self::xyz(tensor);
        pc.set_field(3, b"i\0\0\0", 12, TensorDtype::F32);
        pc.field_count = 4;
        pc
    }

    /// Wrap a `HorusTensor` as a `PointCloud` with no field metadata.
    pub fn from_tensor(tensor: HorusTensor) -> Self {
        Self {
            inner: tensor,
            ..Default::default()
        }
    }

    /// Set field metadata at index.
    fn set_field(&mut self, idx: usize, name: &[u8; 4], offset: u16, dtype: TensorDtype) {
        if idx < 8 {
            self.field_names[idx] = *name;
            self.field_offsets[idx] = offset;
            self.field_dtypes[idx] = dtype as u8;
        }
    }

    // === Accessors ===

    /// Number of points (tensor dimension 0).
    #[inline]
    pub fn point_count(&self) -> u64 {
        if self.inner.ndim >= 1 {
            self.inner.shape[0]
        } else {
            0
        }
    }

    /// Fields per point (tensor dimension 1).
    #[inline]
    pub fn fields_per_point(&self) -> u32 {
        if self.inner.ndim >= 2 {
            self.inner.shape[1] as u32
        } else {
            0
        }
    }

    /// Number of defined field descriptors.
    #[inline]
    pub fn field_count(&self) -> u8 {
        self.field_count
    }

    /// Whether this is a plain XYZ cloud (3 fields).
    #[inline]
    pub fn is_xyz(&self) -> bool {
        self.fields_per_point() == 3
    }

    /// Whether this cloud has intensity (>= 4 fields).
    #[inline]
    pub fn has_intensity(&self) -> bool {
        self.fields_per_point() >= 4
    }

    /// Whether this cloud has color (>= 6 fields).
    #[inline]
    pub fn has_color(&self) -> bool {
        self.fields_per_point() >= 6
    }

    /// Data type of point components.
    #[inline]
    pub fn dtype(&self) -> TensorDtype {
        self.inner.dtype
    }

    /// Total bytes of point data.
    #[inline]
    pub fn nbytes(&self) -> u64 {
        self.inner.nbytes()
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

    /// Whether the cloud is dense (no invalid points).
    #[inline]
    pub fn is_dense(&self) -> bool {
        self.is_dense != 0
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
    fn test_pointcloud_size() {
        assert_eq!(
            std::mem::size_of::<PointCloudDescriptor>(),
            336,
            "PointCloudDescriptor must be exactly 336 bytes"
        );
    }

    #[test]
    fn test_pointcloud_pod() {
        let pc = PointCloudDescriptor::default();
        let bytes: &[u8] = bytemuck::bytes_of(&pc);
        assert_eq!(bytes.len(), 336);
        let _recovered: &PointCloudDescriptor = bytemuck::from_bytes(bytes);
    }

    #[test]
    fn test_pointcloud_xyz() {
        let tensor = HorusTensor::new(1, 0, 0, 0, &[10000, 3], TensorDtype::F32, Device::cpu());
        let pc = PointCloudDescriptor::xyz(tensor);
        assert_eq!(pc.point_count(), 10000);
        assert_eq!(pc.fields_per_point(), 3);
        assert!(pc.is_xyz());
        assert!(!pc.has_intensity());
        assert!(pc.is_dense());
    }

    #[test]
    fn test_pointcloud_xyzi() {
        let tensor = HorusTensor::new(1, 0, 0, 0, &[5000, 4], TensorDtype::F32, Device::cpu());
        let pc = PointCloudDescriptor::xyzi(tensor);
        assert_eq!(pc.point_count(), 5000);
        assert!(pc.has_intensity());
        assert_eq!(pc.field_count(), 4);
    }

    #[test]
    fn test_pointcloud_frame_id() {
        let mut pc = PointCloudDescriptor::default();
        pc.set_frame_id("lidar_front");
        assert_eq!(pc.frame_id(), "lidar_front");
    }
}
