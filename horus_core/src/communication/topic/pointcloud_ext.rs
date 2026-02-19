//! `PointCloudTopic` — zero-copy point cloud transport with `new()`, `send()`, `recv()`

use super::pool_registry::global_pool;
use super::Topic;
use crate::error::HorusResult;
use crate::memory::pointcloud::PointCloud;
use horus_types::PointCloudDescriptor;

/// Topic for sending and receiving `PointCloud` via zero-copy shared memory.
///
/// # Example
///
/// ```rust,ignore
/// use horus::prelude::*;
///
/// let topic = PointCloudTopic::new("lidar/points")?;
///
/// let mut pc = PointCloud::new(1000, 3, TensorDtype::F32)?;
/// // Fill point data...
/// topic.send(&pc);
///
/// if let Some(pc) = topic.recv() {
///     let xyz = pc.extract_xyz();
/// }
/// ```
pub struct PointCloudTopic {
    inner: Topic<PointCloudDescriptor>,
}

impl PointCloudTopic {
    /// Create a new point cloud topic.
    pub fn new(name: impl Into<String>) -> HorusResult<Self> {
        Ok(Self {
            inner: Topic::new(name)?,
        })
    }

    /// Send a point cloud (zero-copy).
    pub fn send(&self, pc: &PointCloud) {
        pc.pool().retain(pc.descriptor().tensor());
        self.inner.send(*pc.descriptor());
    }

    /// Receive the next point cloud.
    pub fn recv(&self) -> Option<PointCloud> {
        let descriptor = self.inner.recv()?;
        let pool = global_pool();
        Some(PointCloud::from_owned(descriptor, pool))
    }

    /// Topic name.
    pub fn name(&self) -> &str {
        self.inner.name()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::communication::pod::is_pod;
    use horus_types::TensorDtype;

    #[test]
    fn test_pointcloud_descriptor_is_pod() {
        assert!(is_pod::<PointCloudDescriptor>());
    }

    #[test]
    fn test_pointcloud_descriptor_size() {
        assert_eq!(std::mem::size_of::<PointCloudDescriptor>(), 336);
    }

    #[test]
    fn test_pointcloud_topic_roundtrip() {
        let topic = PointCloudTopic::new("test/pc_topic_roundtrip").unwrap();

        let pc = PointCloud::new(4, 3, TensorDtype::F32).unwrap();

        assert_eq!(pc.point_count(), 4);
        assert_eq!(pc.fields_per_point(), 3);

        // Write XYZ data: 4 points * 3 floats = 12 floats
        let data = pc.data_mut();
        let floats: &mut [f32] = bytemuck::cast_slice_mut(&mut data[..48]);
        for i in 0..12 {
            floats[i] = (i as f32) * 0.1;
        }

        topic.send(&pc);

        let recv_pc = topic.recv().expect("should receive pointcloud");
        assert_eq!(recv_pc.point_count(), 4);
        assert_eq!(recv_pc.fields_per_point(), 3);

        let recv_data = recv_pc.data();
        let recv_floats: &[f32] = bytemuck::cast_slice(&recv_data[..48]);
        for i in 0..12 {
            assert_eq!(recv_floats[i], (i as f32) * 0.1, "mismatch at {}", i);
        }
    }
}
