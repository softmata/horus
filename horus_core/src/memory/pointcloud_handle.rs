//! RAII handle for PointCloud tensor data access
//!
//! `PointCloudHandle` wraps a `PointCloud` descriptor + `Arc<TensorPool>` and
//! provides ergonomic point access methods.

use std::sync::Arc;

use horus_types::{PointCloud, TensorDtype};

use super::tensor_pool::TensorPool;

/// RAII handle providing data access for a `PointCloud` descriptor.
pub struct PointCloudHandle {
    descriptor: PointCloud,
    pool: Arc<TensorPool>,
}

impl PointCloudHandle {
    /// Create a handle from a descriptor that already has a refcount of 1.
    pub(crate) fn from_owned(descriptor: PointCloud, pool: Arc<TensorPool>) -> Self {
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
    ///
    /// For an F32 XYZ cloud, each point is `fields_per_point * 4` bytes.
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
        // SAFETY: We've verified dtype is F32 above
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

    /// Get the underlying descriptor.
    #[inline]
    pub fn descriptor(&self) -> &PointCloud {
        &self.descriptor
    }

    /// Get a mutable reference to the descriptor.
    #[inline]
    pub fn descriptor_mut(&mut self) -> &mut PointCloud {
        &mut self.descriptor
    }

    /// Get the pool.
    #[inline]
    pub fn pool(&self) -> &Arc<TensorPool> {
        &self.pool
    }

    /// Current reference count.
    #[inline]
    pub fn refcount(&self) -> u32 {
        self.pool.refcount(self.descriptor.tensor())
    }
}

impl Clone for PointCloudHandle {
    fn clone(&self) -> Self {
        self.pool.retain(self.descriptor.tensor());
        Self {
            descriptor: self.descriptor,
            pool: Arc::clone(&self.pool),
        }
    }
}

impl Drop for PointCloudHandle {
    fn drop(&mut self) {
        self.pool.release(self.descriptor.tensor());
    }
}

impl std::fmt::Debug for PointCloudHandle {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("PointCloudHandle")
            .field("point_count", &self.point_count())
            .field("fields_per_point", &self.fields_per_point())
            .field("dtype", &self.dtype())
            .field("refcount", &self.refcount())
            .finish()
    }
}

unsafe impl Send for PointCloudHandle {}
unsafe impl Sync for PointCloudHandle {}
