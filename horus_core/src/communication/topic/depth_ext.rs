//! Topic extension for `Topic<DepthImage>` — zero-copy depth pipeline
//!
//! Provides `create()`, `publish()`, `next()` methods.

use std::sync::Arc;

use super::pool_registry::get_or_create_pool;
use super::Topic;
use crate::error::HorusResult;
use crate::memory::depth_handle::DepthImageHandle;
use crate::memory::TensorPool;
use horus_types::{DepthImage, Device, TensorDtype};

impl Topic<DepthImage> {
    /// Get or create the auto-managed tensor pool for this topic.
    pub fn pool(&self) -> Arc<TensorPool> {
        get_or_create_pool(self.name())
    }

    /// Create a new depth image in shared memory.
    ///
    /// Use `TensorDtype::F32` for meters or `TensorDtype::U16` for millimeters.
    pub fn create(
        &self,
        height: u32,
        width: u32,
        dtype: TensorDtype,
    ) -> HorusResult<DepthImageHandle> {
        let pool = self.pool();
        let shape = [height as u64, width as u64];
        let tensor = pool.alloc(&shape, dtype, Device::cpu())?;
        let descriptor = DepthImage::new(tensor);
        Ok(DepthImageHandle::from_owned(descriptor, pool))
    }

    /// Publish a depth image (zero-copy).
    pub fn publish(&self, handle: &DepthImageHandle) {
        handle.pool().retain(handle.descriptor().tensor());
        self.send(*handle.descriptor());
    }

    /// Receive the next depth image.
    pub fn next(&self) -> Option<DepthImageHandle> {
        let descriptor = self.recv()?;
        let pool = self.pool();
        Some(DepthImageHandle::from_owned(descriptor, pool))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::communication::pod::is_pod;

    #[test]
    fn test_depth_image_is_pod() {
        assert!(is_pod::<DepthImage>());
    }

    #[test]
    fn test_depth_image_size() {
        assert_eq!(std::mem::size_of::<DepthImage>(), 288);
    }

    #[test]
    fn test_topic_depth_roundtrip() {
        let topic: Topic<DepthImage> = Topic::new("test/depth_ext_roundtrip").unwrap();

        let handle = topic.create(2, 3, TensorDtype::F32).unwrap();

        assert_eq!(handle.height(), 2);
        assert_eq!(handle.width(), 3);
        assert!(handle.is_meters());

        handle.set_depth_f32(0, 0, 1.5);
        handle.set_depth_f32(1, 0, 2.0);
        handle.set_depth_f32(2, 0, 0.5);

        topic.publish(&handle);

        let recv_handle = topic.next().expect("should receive depth");
        assert_eq!(recv_handle.height(), 2);
        assert_eq!(recv_handle.width(), 3);
        assert!(recv_handle.is_meters());

        assert_eq!(recv_handle.get_depth(0, 0), Some(1.5));
        assert_eq!(recv_handle.get_depth(1, 0), Some(2.0));
        assert_eq!(recv_handle.get_depth(2, 0), Some(0.5));
    }
}
