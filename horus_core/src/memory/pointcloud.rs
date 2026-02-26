//! User-facing PointCloud type with zero-copy shared memory backing
//!
//! `PointCloud` is the primary type for 3D point data in HORUS. It wraps a Pod
//! descriptor + `Arc<TensorPool>` and provides a rich API for point access.

use std::sync::Arc;

use crate::types::{Device, PointCloudDescriptor, TensorDtype};

use super::tensor_pool::TensorPool;
use crate::communication::topic::pool_registry::global_pool;
use crate::error::HorusResult;

/// Point cloud with zero-copy shared memory backing.
///
/// Create with `PointCloud::new()`, access points with `data()` / `data_mut()`,
/// and send through topics with `topic.send(&pc)`.
pub struct PointCloud {
    descriptor: PointCloudDescriptor,
    pool: Arc<TensorPool>,
}

// Shared methods: data access, lifecycle, metadata delegation
crate::impl_tensor_backed!(PointCloud, PointCloudDescriptor, "point cloud");

impl PointCloud {
    /// Create a new point cloud with the given number of points and fields.
    ///
    /// - `num_points`: number of 3D points
    /// - `fields_per_point`: floats per point (3=XYZ, 4=XYZI, 6=XYZRGB)
    /// - `dtype`: data type (typically `TensorDtype::F32`)
    ///
    /// # Example
    ///
    /// ```rust,ignore
    /// let pc = PointCloud::new(10000, 3, TensorDtype::F32)?;
    /// assert_eq!(pc.point_count(), 10000);
    /// assert!(pc.is_xyz());
    /// ```
    pub fn new(num_points: u32, fields_per_point: u32, dtype: TensorDtype) -> HorusResult<Self> {
        let pool = global_pool();
        let shape = [num_points as u64, fields_per_point as u64];
        let tensor = pool.alloc(&shape, dtype, Device::cpu())?;

        let descriptor = if fields_per_point == 3 {
            PointCloudDescriptor::xyz(tensor)
        } else if fields_per_point == 4 {
            PointCloudDescriptor::xyzi(tensor)
        } else {
            PointCloudDescriptor::from_tensor(tensor)
        };

        Ok(Self { descriptor, pool })
    }

    // === Point-specific methods ===

    /// Get the i-th point as a byte slice.
    pub fn point_at(&self, idx: u64) -> Option<&[u8]> {
        if idx >= self.point_count() {
            return None;
        }
        let fpp = self.fields_per_point() as usize;
        let elem_size = self.dtype().element_size();
        let point_bytes = fpp * elem_size;
        let offset = idx as usize * point_bytes;
        let data = self.data();
        if offset + point_bytes <= data.len() {
            Some(&data[offset..offset + point_bytes])
        } else {
            None
        }
    }

    /// Extract all XYZ coordinates as `Vec<[f32; 3]>`.
    ///
    /// Only valid for F32 clouds with at least 3 fields per point.
    pub fn extract_xyz(&self) -> Option<Vec<[f32; 3]>> {
        if self.dtype() != TensorDtype::F32 || self.fields_per_point() < 3 {
            return None;
        }

        let fpp = self.fields_per_point() as usize;
        let count = self.point_count() as usize;
        let floats = unsafe { self.data_as::<f32>() };

        let mut points = Vec::with_capacity(count);
        for i in 0..count {
            let base = i * fpp;
            if base + 3 <= floats.len() {
                points.push([floats[base], floats[base + 1], floats[base + 2]]);
            }
        }
        Some(points)
    }

    // === PointCloud-specific metadata accessors ===

    /// Number of points.
    #[inline]
    pub fn point_count(&self) -> u64 {
        self.descriptor.point_count()
    }

    /// Fields per point.
    #[inline]
    pub fn fields_per_point(&self) -> u32 {
        self.descriptor.fields_per_point()
    }

    /// Whether this is a plain XYZ cloud.
    #[inline]
    pub fn is_xyz(&self) -> bool {
        self.descriptor.is_xyz()
    }

    /// Whether this cloud has intensity.
    #[inline]
    pub fn has_intensity(&self) -> bool {
        self.descriptor.has_intensity()
    }

    /// Whether this cloud has color.
    #[inline]
    pub fn has_color(&self) -> bool {
        self.descriptor.has_color()
    }
}

impl std::fmt::Debug for PointCloud {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("PointCloud")
            .field("point_count", &self.point_count())
            .field("fields_per_point", &self.fields_per_point())
            .field("dtype", &self.dtype())
            .finish()
    }
}
