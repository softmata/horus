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
    /// Returns `None` if dtype is wrong, the tensor claims more points than
    /// the backing allocation contains, or the data pointer is not 4-byte aligned.
    pub fn extract_xyz(&self) -> Option<Vec<[f32; 3]>> {
        if self.dtype() != TensorDtype::F32 || self.fields_per_point() < 3 {
            return None;
        }

        let fpp = self.fields_per_point() as usize;
        let count = self.point_count() as usize;

        // Validate bounds and alignment BEFORE the unsafe reinterpretation.
        // A corrupted descriptor may report more points than the slot contains;
        // checking after the slice is created is too late to prevent UB.
        let bytes = self.data();
        let required_bytes = count
            .checked_mul(fpp)
            .and_then(|n| n.checked_mul(std::mem::size_of::<f32>()))
            .unwrap_or(usize::MAX);
        if required_bytes > bytes.len() {
            return None;
        }
        // f32 requires 4-byte alignment; verify before reinterpretation.
        if !(bytes.as_ptr() as usize).is_multiple_of(std::mem::align_of::<f32>()) {
            return None;
        }

        // SAFETY:
        // - dtype is F32 (checked above).
        // - The pointer is 4-byte aligned (verified above).
        // - `floats.len() >= count * fpp` (bounds-validated above), so every
        //   in-loop access `floats[base..base+3]` is guaranteed in-bounds.
        let floats = unsafe { self.data_as::<f32>() };

        let mut points = Vec::with_capacity(count);
        for i in 0..count {
            let base = i * fpp;
            // No per-iteration bounds check needed — pre-validated above.
            points.push([floats[base], floats[base + 1], floats[base + 2]]);
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

#[cfg(test)]
mod tests {
    use super::*;
    use crate::types::TensorDtype;

    #[test]
    fn test_extract_xyz_truncated_data_returns_none() {
        // Allocate a real PointCloud with 10 points (120 bytes of storage).
        let real = PointCloud::new(10, 3, TensorDtype::F32).unwrap();
        let pool = real.pool().clone();

        // Copy the descriptor and inflate shape[0] so it claims 100 points
        // (1200 bytes), far more than the 120-byte slot contains.
        // This simulates a corrupted tensor descriptor received cross-process.
        let mut bad_desc = *real.descriptor();
        bad_desc.tensor_mut().shape[0] = 100;

        let bad_pc = PointCloud::from_owned(bad_desc, pool);

        // extract_xyz must detect the mismatch and return None, not access
        // out-of-bounds memory or cause undefined behaviour.
        assert!(
            bad_pc.extract_xyz().is_none(),
            "extract_xyz should return None when descriptor claims more data than is allocated"
        );

        // Keep `real` alive to prevent slot reuse during the test.
        drop(real);
    }

    #[test]
    fn test_extract_xyz_valid_cloud() {
        let mut pc = PointCloud::new(3, 3, TensorDtype::F32).unwrap();
        let floats: &mut [f32] = bytemuck::cast_slice_mut(pc.data_mut());
        floats[0..9].copy_from_slice(&[1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0]);

        let xyz = pc.extract_xyz().expect("valid XYZ cloud should extract");
        assert_eq!(xyz.len(), 3);
        assert_eq!(xyz[0], [1.0, 2.0, 3.0]);
        assert_eq!(xyz[1], [4.0, 5.0, 6.0]);
        assert_eq!(xyz[2], [7.0, 8.0, 9.0]);
    }

    #[test]
    fn test_extract_xyz_wrong_dtype_returns_none() {
        let pc = PointCloud::new(10, 3, TensorDtype::U16).unwrap();
        assert!(
            pc.extract_xyz().is_none(),
            "extract_xyz should return None for non-F32 dtype"
        );
    }

    #[test]
    fn test_extract_xyz_too_few_fields_returns_none() {
        // A 1-field-per-point cloud cannot produce XYZ triples.
        let pc = PointCloud::new(10, 2, TensorDtype::F32).unwrap();
        assert!(
            pc.extract_xyz().is_none(),
            "extract_xyz should return None for < 3 fields per point"
        );
    }
}
