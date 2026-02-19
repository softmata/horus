//! Topic extension for `Topic<DepthImage>` — zero-copy depth pipeline
//!
//! Provides `alloc_depth()`, `send_depth()`, `recv_depth()` methods.

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

    /// Allocate a depth image from the topic's pool.
    ///
    /// Creates a tensor with shape `[height, width]`.
    /// Use `TensorDtype::F32` for meters or `TensorDtype::U16` for millimeters.
    pub fn alloc_depth(
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

    /// Send a depth image handle through this topic (zero-copy).
    pub fn send_depth(&self, handle: &DepthImageHandle) {
        handle.pool().retain(handle.descriptor().tensor());
        self.send(*handle.descriptor());
    }

    /// Receive a depth image and wrap in a `DepthImageHandle`.
    pub fn recv_depth(&self) -> Option<DepthImageHandle> {
        let descriptor = self.recv()?;
        let pool = self.pool();
        Some(DepthImageHandle::from_owned(descriptor, pool))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::communication::is_pod;

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

        // Allocate a 2x3 F32 depth image (6 pixels * 4 bytes = 24 bytes)
        let handle = topic
            .alloc_depth(2, 3, TensorDtype::F32)
            .unwrap();

        assert_eq!(handle.height(), 2);
        assert_eq!(handle.width(), 3);
        assert!(handle.is_meters());

        // Write depth data
        handle.set_depth_f32(0, 0, 1.5);
        handle.set_depth_f32(1, 0, 2.0);
        handle.set_depth_f32(2, 0, 0.5);

        // Send
        topic.send_depth(&handle);

        // Receive
        let recv_handle = topic.recv_depth().expect("should receive depth");
        assert_eq!(recv_handle.height(), 2);
        assert_eq!(recv_handle.width(), 3);
        assert!(recv_handle.is_meters());

        // Verify depth values
        assert_eq!(recv_handle.get_depth(0, 0), Some(1.5));
        assert_eq!(recv_handle.get_depth(1, 0), Some(2.0));
        assert_eq!(recv_handle.get_depth(2, 0), Some(0.5));
    }
}
