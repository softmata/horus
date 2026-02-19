//! Topic extension for `Topic<PointCloud>` — zero-copy point cloud pipeline
//!
//! Provides `create()`, `publish()`, `next()` methods.

use std::sync::Arc;

use super::pool_registry::get_or_create_pool;
use super::Topic;
use crate::error::HorusResult;
use crate::memory::pointcloud_handle::PointCloudHandle;
use crate::memory::TensorPool;
use horus_types::{Device, PointCloud, TensorDtype};

impl Topic<PointCloud> {
    /// Get or create the auto-managed tensor pool for this topic.
    pub fn pool(&self) -> Arc<TensorPool> {
        get_or_create_pool(self.name())
    }

    /// Create a new point cloud in shared memory.
    ///
    /// - `num_points`: number of points
    /// - `fields_per_point`: floats per point (3=XYZ, 4=XYZI, 6=XYZRGB)
    /// - `dtype`: data type (typically `TensorDtype::F32`)
    pub fn create(
        &self,
        num_points: u32,
        fields_per_point: u32,
        dtype: TensorDtype,
    ) -> HorusResult<PointCloudHandle> {
        let pool = self.pool();
        let shape = [num_points as u64, fields_per_point as u64];
        let tensor = pool.alloc(&shape, dtype, Device::cpu())?;

        let descriptor = if fields_per_point == 3 {
            PointCloud::xyz(tensor)
        } else if fields_per_point == 4 {
            PointCloud::xyzi(tensor)
        } else {
            PointCloud::from_tensor(tensor)
        };

        Ok(PointCloudHandle::from_owned(descriptor, pool))
    }

    /// Publish a point cloud (zero-copy).
    pub fn publish(&self, handle: &PointCloudHandle) {
        handle.pool().retain(handle.descriptor().tensor());
        self.send(*handle.descriptor());
    }

    /// Receive the next point cloud.
    pub fn next(&self) -> Option<PointCloudHandle> {
        let descriptor = self.recv()?;
        let pool = self.pool();
        Some(PointCloudHandle::from_owned(descriptor, pool))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::communication::pod::is_pod;

    #[test]
    fn test_pointcloud_is_pod() {
        assert!(is_pod::<PointCloud>());
    }

    #[test]
    fn test_pointcloud_size() {
        assert_eq!(std::mem::size_of::<PointCloud>(), 336);
    }

    #[test]
    fn test_topic_pointcloud_roundtrip() {
        let topic: Topic<PointCloud> = Topic::new("test/pc_ext_roundtrip").unwrap();

        let handle = topic.create(4, 3, TensorDtype::F32).unwrap();

        assert_eq!(handle.point_count(), 4);
        assert_eq!(handle.fields_per_point(), 3);

        // Write XYZ data: 4 points * 3 floats = 12 floats
        let data = handle.data_mut();
        let floats: &mut [f32] =
            bytemuck::cast_slice_mut(&mut data[..48]);
        for i in 0..12 {
            floats[i] = (i as f32) * 0.1;
        }

        topic.publish(&handle);

        let recv_handle = topic.next().expect("should receive pointcloud");
        assert_eq!(recv_handle.point_count(), 4);
        assert_eq!(recv_handle.fields_per_point(), 3);

        let recv_data = recv_handle.data();
        let recv_floats: &[f32] = bytemuck::cast_slice(&recv_data[..48]);
        for i in 0..12 {
            assert_eq!(recv_floats[i], (i as f32) * 0.1, "mismatch at {}", i);
        }
    }
}
