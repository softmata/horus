//! Tensor-backed point cloud for zero-copy lidar/depth pipelines
//!
//! `TensorPointCloud` wraps a `HorusTensor` with point-cloud-specific accessors.
//! Because it's Pod, `Topic<TensorPointCloud>` uses the zero-copy shared memory
//! path (~50ns) instead of serde serialization.
//!
//! # Layout
//!
//! The tensor shape is `[N, K]` where:
//! - `N` = number of points
//! - `K` = fields per point (3 for XYZ, 4 for XYZI, 6 for XYZRGB, etc.)
//!
//! Typical configs:
//! - `[N, 3]` F32 — XYZ point cloud
//! - `[N, 4]` F32 — XYZI (XYZ + intensity)
//! - `[N, 6]` F32 — XYZRGB (XYZ + RGB as floats)

use bytemuck::{Pod, Zeroable};
use horus_core::communication::PodMessage;
use horus_core::core::LogSummary;
use horus_types::{HorusTensor, TensorDtype};
use serde::{Deserialize, Serialize};

/// A point cloud backed by a shared-memory tensor.
///
/// Zero-overhead wrapper around `HorusTensor`. The tensor shape encodes
/// `[num_points, fields_per_point]`. Because this is Pod,
/// `Topic<TensorPointCloud>` routes through the ~50ns zero-copy path.
#[repr(C)]
#[derive(Clone, Copy, Debug, Serialize, Deserialize)]
pub struct TensorPointCloud {
    inner: HorusTensor,
}

// Safety: TensorPointCloud is repr(C) and contains only Pod fields.
unsafe impl Zeroable for TensorPointCloud {}
unsafe impl Pod for TensorPointCloud {}
unsafe impl PodMessage for TensorPointCloud {}

impl TensorPointCloud {
    /// Wrap an existing `HorusTensor` as a `TensorPointCloud`.
    ///
    /// The tensor should have shape `[N, K]` where N = points, K = fields.
    pub fn from_tensor(tensor: HorusTensor) -> Self {
        Self { inner: tensor }
    }

    /// Get the inner tensor descriptor.
    pub fn tensor(&self) -> &HorusTensor {
        &self.inner
    }

    /// Get a mutable reference to the inner tensor descriptor.
    pub fn tensor_mut(&mut self) -> &mut HorusTensor {
        &mut self.inner
    }

    /// Number of points in the cloud (shape dimension 0).
    pub fn point_count(&self) -> u64 {
        let shape = self.inner.shape();
        if shape.is_empty() {
            0
        } else {
            shape[0]
        }
    }

    /// Fields per point (shape dimension 1).
    ///
    /// Common values: 3 (XYZ), 4 (XYZI), 6 (XYZRGB).
    pub fn fields_per_point(&self) -> u32 {
        let shape = self.inner.shape();
        if shape.len() < 2 {
            0
        } else {
            shape[1] as u32
        }
    }

    /// Whether this is a plain XYZ cloud (3 fields).
    pub fn is_xyz(&self) -> bool {
        self.fields_per_point() == 3
    }

    /// Whether this cloud has intensity (4 fields: XYZI).
    pub fn has_intensity(&self) -> bool {
        self.fields_per_point() >= 4
    }

    /// Whether this cloud has color (6 fields: XYZRGB).
    pub fn has_color(&self) -> bool {
        self.fields_per_point() >= 6
    }

    /// Data type of point components.
    pub fn dtype(&self) -> TensorDtype {
        self.inner.dtype
    }

    /// Total bytes of point data.
    pub fn nbytes(&self) -> u64 {
        self.inner.nbytes()
    }

    /// Whether the tensor data lives on CPU.
    pub fn is_cpu(&self) -> bool {
        self.inner.is_cpu()
    }

    /// Whether the tensor data lives on a CUDA GPU.
    pub fn is_cuda(&self) -> bool {
        self.inner.is_cuda()
    }
}

impl LogSummary for TensorPointCloud {
    fn log_summary(&self) -> String {
        let kind = match self.fields_per_point() {
            3 => "XYZ",
            4 => "XYZI",
            6 => "XYZRGB",
            k => return format!("TensorPointCloud({} pts, {} fields)", self.point_count(), k),
        };
        format!(
            "TensorPointCloud({} pts, {}, {:?}, {})",
            self.point_count(),
            kind,
            self.dtype(),
            self.inner.device(),
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_tensor_pointcloud_size() {
        assert_eq!(
            std::mem::size_of::<TensorPointCloud>(),
            std::mem::size_of::<HorusTensor>()
        );
    }

    #[test]
    fn test_tensor_pointcloud_xyz() {
        let mut tensor = <HorusTensor as Zeroable>::zeroed();
        tensor.ndim = 2;
        tensor.dtype = TensorDtype::F32;
        tensor.shape[0] = 10000;
        tensor.shape[1] = 3;

        let cloud = TensorPointCloud::from_tensor(tensor);
        assert_eq!(cloud.point_count(), 10000);
        assert_eq!(cloud.fields_per_point(), 3);
        assert!(cloud.is_xyz());
        assert!(!cloud.has_intensity());
        assert!(!cloud.has_color());
    }

    #[test]
    fn test_tensor_pointcloud_xyzrgb() {
        let mut tensor = <HorusTensor as Zeroable>::zeroed();
        tensor.ndim = 2;
        tensor.dtype = TensorDtype::F32;
        tensor.shape[0] = 5000;
        tensor.shape[1] = 6;

        let cloud = TensorPointCloud::from_tensor(tensor);
        assert_eq!(cloud.point_count(), 5000);
        assert!(cloud.has_color());
        assert!(cloud.has_intensity());
    }
}
