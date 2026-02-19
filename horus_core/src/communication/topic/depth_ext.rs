//! `DepthTopic` — zero-copy depth image transport with `new()`, `send()`, `recv()`

use super::pool_registry::global_pool;
use super::Topic;
use crate::error::HorusResult;
use crate::memory::depth_image::DepthImage;
use horus_types::DepthImageDescriptor;

/// Topic for sending and receiving `DepthImage` via zero-copy shared memory.
///
/// # Example
///
/// ```rust,ignore
/// use horus::prelude::*;
///
/// let topic = DepthTopic::new("depth/camera")?;
///
/// let mut depth = DepthImage::new(480, 640, TensorDtype::F32)?;
/// depth.set_depth(100, 200, 1.5);
/// topic.send(&depth);
///
/// if let Some(depth) = topic.recv() {
///     let d = depth.get_depth(100, 200);
/// }
/// ```
pub struct DepthTopic {
    inner: Topic<DepthImageDescriptor>,
}

impl DepthTopic {
    /// Create a new depth image topic.
    pub fn new(name: impl Into<String>) -> HorusResult<Self> {
        Ok(Self {
            inner: Topic::new(name)?,
        })
    }

    /// Send a depth image (zero-copy).
    pub fn send(&self, depth: &DepthImage) {
        depth.pool().retain(depth.descriptor().tensor());
        self.inner.send(*depth.descriptor());
    }

    /// Receive the next depth image.
    pub fn recv(&self) -> Option<DepthImage> {
        let descriptor = self.inner.recv()?;
        let pool = global_pool();
        Some(DepthImage::from_owned(descriptor, pool))
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
    fn test_depth_descriptor_is_pod() {
        assert!(is_pod::<DepthImageDescriptor>());
    }

    #[test]
    fn test_depth_descriptor_size() {
        assert_eq!(std::mem::size_of::<DepthImageDescriptor>(), 288);
    }

    #[test]
    fn test_depth_topic_roundtrip() {
        let topic = DepthTopic::new("test/depth_topic_roundtrip").unwrap();

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
