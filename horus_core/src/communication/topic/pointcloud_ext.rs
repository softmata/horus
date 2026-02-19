//! Topic extension for `Topic<PointCloud>` — zero-copy point cloud pipeline
//!
//! Provides `alloc_pointcloud()`, `send_pointcloud()`, `recv_pointcloud()` methods.

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

    /// Allocate a point cloud from the topic's pool.
    ///
    /// Creates a tensor with shape `[num_points, fields_per_point]`.
    /// For XYZ clouds, use `fields=3`. For XYZI, `fields=4`, etc.
    pub fn alloc_pointcloud(
        &self,
        num_points: u64,
        fields_per_point: u32,
        dtype: TensorDtype,
    ) -> HorusResult<PointCloudHandle> {
        let pool = self.pool();
        let shape = [num_points, fields_per_point as u64];
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

    /// Send a point cloud handle through this topic (zero-copy).
    pub fn send_pointcloud(&self, handle: &PointCloudHandle) {
        handle.pool().retain(handle.descriptor().tensor());
        self.send(*handle.descriptor());
    }

    /// Receive a point cloud and wrap in a `PointCloudHandle`.
    pub fn recv_pointcloud(&self) -> Option<PointCloudHandle> {
        let descriptor = self.recv()?;
        let pool = self.pool();
        Some(PointCloudHandle::from_owned(descriptor, pool))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::communication::is_pod;

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

        // Allocate a 4-point XYZ cloud (4 * 3 * 4 = 48 bytes of data)
        let handle = topic
            .alloc_pointcloud(4, 3, TensorDtype::F32)
            .unwrap();

        assert_eq!(handle.point_count(), 4);
        assert_eq!(handle.fields_per_point(), 3);

        // Write XYZ data: 4 points * 3 floats = 12 floats
        let data = handle.data_mut();
        let floats: &mut [f32] =
            bytemuck::cast_slice_mut(&mut data[..48]);
        for i in 0..12 {
            floats[i] = (i as f32) * 0.1;
        }

        // Send
        topic.send_pointcloud(&handle);

        // Receive
        let recv_handle = topic.recv_pointcloud().expect("should receive pointcloud");
        assert_eq!(recv_handle.point_count(), 4);
        assert_eq!(recv_handle.fields_per_point(), 3);

        let recv_data = recv_handle.data();
        let recv_floats: &[f32] = bytemuck::cast_slice(&recv_data[..48]);
        for i in 0..12 {
            assert_eq!(recv_floats[i], (i as f32) * 0.1, "mismatch at {}", i);
        }
    }
}
