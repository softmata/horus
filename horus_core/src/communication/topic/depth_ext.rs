//! DepthImage topic tests — validates Topic<DepthImage> zero-copy roundtrip

#[cfg(test)]
mod tests {
    use crate::communication::pod::is_pod;
    use crate::communication::topic::Topic;
    use crate::memory::depth_image::DepthImage;
    use crate::types::{DepthImageDescriptor, TensorDtype};

    #[test]
    fn test_depth_descriptor_is_pod() {
        assert!(is_pod::<DepthImageDescriptor>());
    }

    #[test]
    fn test_depth_descriptor_size() {
        assert_eq!(std::mem::size_of::<DepthImageDescriptor>(), 224);
    }

    #[test]
    fn test_depth_topic_roundtrip() {
        let topic: Topic<DepthImage> = Topic::new("test/depth_topic_roundtrip_unified").unwrap();

        let mut depth = DepthImage::new(2, 3, TensorDtype::F32).unwrap();

        assert_eq!(depth.height(), 2);
        assert_eq!(depth.width(), 3);
        assert!(depth.is_meters());

        depth.set_depth(0, 0, 1.5);
        depth.set_depth(1, 0, 2.0);
        depth.set_depth(2, 0, 0.5);

        topic.send(&depth);

        let recv_depth = topic.recv().expect("should receive depth");
        assert_eq!(recv_depth.height(), 2);
        assert_eq!(recv_depth.width(), 3);
        assert!(recv_depth.is_meters());

        assert_eq!(recv_depth.get_depth(0, 0), Some(1.5));
        assert_eq!(recv_depth.get_depth(1, 0), Some(2.0));
        assert_eq!(recv_depth.get_depth(2, 0), Some(0.5));
    }
}
