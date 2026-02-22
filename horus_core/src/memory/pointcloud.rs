//! User-facing PointCloud type with zero-copy shared memory backing
//!
//! `PointCloud` is the primary type for 3D point data in HORUS. It wraps a Pod
//! descriptor + `Arc<TensorPool>` and provides a rich API for point access.

use std::sync::Arc;

use horus_types::{Device, PointCloudDescriptor, TensorDtype};

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

    /// Create a PointCloud from a descriptor and pool.
    pub fn from_owned(descriptor: PointCloudDescriptor, pool: Arc<TensorPool>) -> Self {
        Self { descriptor, pool }
    }

    // === Point data access ===

    /// Get raw point data as a byte slice (zero-copy).
    #[inline]
    pub fn data(&self) -> &[u8] {
        self.pool.data_slice(self.descriptor.tensor())
    }

    /// Get raw point data as a mutable byte slice (zero-copy).
    #[inline]
    #[allow(clippy::mut_from_ref)]
    pub fn data_mut(&self) -> &mut [u8] {
        self.pool.data_slice_mut(self.descriptor.tensor())
    }

    /// Copy point data from a buffer.
    ///
    /// # Panics
    ///
    /// Panics if `src` length doesn't match `nbytes()`.
    pub fn copy_from(&mut self, src: &[u8]) -> &mut Self {
        let data = self.data_mut();
        assert_eq!(
            src.len(),
            data.len(),
            "source buffer size ({}) doesn't match point cloud size ({})",
            src.len(),
            data.len()
        );
        data.copy_from_slice(src);
        self
    }

    /// Get point data as typed slice (e.g., `&[f32]` for F32 clouds).
    ///
    /// # Safety
    /// Caller must ensure T matches the tensor dtype.
    #[inline]
    pub unsafe fn data_as<T: Copy>(&self) -> &[T] {
        let bytes = self.data();
        let ptr = bytes.as_ptr() as *const T;
        let len = bytes.len() / std::mem::size_of::<T>();
        std::slice::from_raw_parts(ptr, len)
    }

    /// Get point data as mutable typed slice.
    ///
    /// # Safety
    /// Caller must ensure T matches the tensor dtype.
    #[inline]
    #[allow(clippy::mut_from_ref)]
    pub unsafe fn data_as_mut<T: Copy>(&self) -> &mut [T] {
        let bytes = self.data_mut();
        let ptr = bytes.as_mut_ptr() as *mut T;
        let len = bytes.len() / std::mem::size_of::<T>();
        std::slice::from_raw_parts_mut(ptr, len)
    }

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

    /// Set the frame ID.
    pub fn set_frame_id(&mut self, id: &str) -> &mut Self {
        self.descriptor.set_frame_id(id);
        self
    }

    /// Set the timestamp in nanoseconds.
    pub fn set_timestamp_ns(&mut self, ts: u64) -> &mut Self {
        self.descriptor.set_timestamp_ns(ts);
        self
    }

    // === Metadata accessors ===

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

    /// Data type of point components.
    #[inline]
    pub fn dtype(&self) -> TensorDtype {
        self.descriptor.dtype()
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

    /// Total bytes of point data.
    #[inline]
    pub fn nbytes(&self) -> u64 {
        self.descriptor.nbytes()
    }

    /// Whether data is on CPU.
    #[inline]
    pub fn is_cpu(&self) -> bool {
        self.descriptor.is_cpu()
    }

    /// Whether data is on CUDA.
    #[inline]
    pub fn is_cuda(&self) -> bool {
        self.descriptor.is_cuda()
    }

    /// Timestamp in nanoseconds.
    #[inline]
    pub fn timestamp_ns(&self) -> u64 {
        self.descriptor.timestamp_ns()
    }

    /// Frame ID.
    #[inline]
    pub fn frame_id(&self) -> &str {
        self.descriptor.frame_id()
    }

    // === Descriptor / pool accessors ===

    /// Get the underlying descriptor.
    #[inline]
    pub fn descriptor(&self) -> &PointCloudDescriptor {
        &self.descriptor
    }

    /// Get the pool reference.
    #[inline]
    pub fn pool(&self) -> &Arc<TensorPool> {
        &self.pool
    }

}

impl Clone for PointCloud {
    fn clone(&self) -> Self {
        self.pool.retain(self.descriptor.tensor());
        Self {
            descriptor: self.descriptor,
            pool: Arc::clone(&self.pool),
        }
    }
}

impl Drop for PointCloud {
    fn drop(&mut self) {
        self.pool.release(self.descriptor.tensor());
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

unsafe impl Send for PointCloud {}
unsafe impl Sync for PointCloud {}
