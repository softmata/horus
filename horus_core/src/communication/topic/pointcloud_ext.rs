//! PointCloud topic tests — validates Topic<PointCloud> zero-copy roundtrip

#[cfg(test)]
mod tests {
    use crate::communication::pod::is_pod;
    use crate::communication::topic::Topic;
    use crate::memory::pointcloud::PointCloud;
    use crate::types::{PointCloudDescriptor, TensorDtype};

    #[test]
    fn test_pointcloud_descriptor_is_pod() {
        assert!(is_pod::<PointCloudDescriptor>());
    }

    #[test]
    fn test_pointcloud_descriptor_size() {
        assert_eq!(std::mem::size_of::<PointCloudDescriptor>(), 272);
    }

    #[test]
    fn test_pointcloud_topic_roundtrip() {
        let topic: Topic<PointCloud> = Topic::new("test/pc_topic_roundtrip_unified").unwrap();

        let pc = PointCloud::new(4, 3, TensorDtype::F32).unwrap();

        assert_eq!(pc.point_count(), 4);
        assert_eq!(pc.fields_per_point(), 3);

        // Write XYZ data: 4 points * 3 floats = 12 floats
        let data = pc.data_mut();
        let floats: &mut [f32] = bytemuck::cast_slice_mut(&mut data[..48]);
        for (i, val) in floats.iter_mut().enumerate() {
            *val = (i as f32) * 0.1;
        }

        topic.send(&pc);

        let recv_pc = topic.recv().expect("should receive pointcloud");
        assert_eq!(recv_pc.point_count(), 4);
        assert_eq!(recv_pc.fields_per_point(), 3);

        let recv_data = recv_pc.data();
        let recv_floats: &[f32] = bytemuck::cast_slice(&recv_data[..48]);
        for (i, val) in recv_floats.iter().enumerate() {
            assert_eq!(*val, (i as f32) * 0.1, "mismatch at {}", i);
        }
    }
}
