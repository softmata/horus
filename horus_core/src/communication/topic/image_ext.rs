//! Topic extension for `Topic<Image>` — zero-copy image pipeline
//!
//! Provides `alloc_image()`, `send_image()`, `recv_image()` methods on
//! `Topic<Image>` that manage the tensor pool automatically.

use std::sync::Arc;

use super::pool_registry::get_or_create_pool;
use super::Topic;
use crate::error::HorusResult;
use crate::memory::image_handle::ImageHandle;
use crate::memory::TensorPool;
use horus_types::{Device, Image, ImageEncoding};

impl Topic<Image> {
    /// Get or create the auto-managed tensor pool for this image topic.
    pub fn pool(&self) -> Arc<TensorPool> {
        get_or_create_pool(self.name())
    }

    /// Allocate an image from the topic's auto-managed pool.
    ///
    /// Creates a tensor with shape `[height, width, channels]` and the dtype
    /// inferred from the encoding. Returns an `ImageHandle` with RAII refcounting.
    ///
    /// # Example
    ///
    /// ```rust,ignore
    /// let topic: Topic<Image> = Topic::new("camera/rgb")?;
    /// let mut img = topic.alloc_image(1080, 1920, ImageEncoding::Rgb8)?;
    /// // Fill pixel data...
    /// camera.capture_into(img.pixels_mut());
    /// topic.send_image(&img);
    /// ```
    pub fn alloc_image(
        &self,
        height: u32,
        width: u32,
        encoding: ImageEncoding,
    ) -> HorusResult<ImageHandle> {
        let pool = self.pool();
        let channels = encoding.channels();
        let dtype = encoding.tensor_dtype();

        let shape = if channels == 1 {
            vec![height as u64, width as u64]
        } else {
            vec![height as u64, width as u64, channels as u64]
        };

        let tensor = pool.alloc(&shape, dtype, Device::cpu())?;
        let descriptor = Image::new(tensor, encoding);
        Ok(ImageHandle::from_owned(descriptor, pool))
    }

    /// Send an image handle through this topic (zero-copy).
    ///
    /// Increments the tensor's refcount so it stays alive for the receiver,
    /// then sends the 288-byte Image descriptor through the ring buffer.
    /// Actual pixel data remains in shared memory.
    pub fn send_image(&self, handle: &ImageHandle) {
        handle.pool().retain(handle.descriptor().tensor());
        self.send(*handle.descriptor());
    }

    /// Receive an image and wrap it in an `ImageHandle` for safe data access.
    ///
    /// Returns `None` if no message is available.
    pub fn recv_image(&self) -> Option<ImageHandle> {
        let descriptor = self.recv()?;
        let pool = self.pool();
        Some(ImageHandle::from_owned(descriptor, pool))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::communication::is_pod;

    #[test]
    fn test_image_is_pod() {
        assert!(is_pod::<Image>());
    }

    #[test]
    fn test_image_size() {
        assert_eq!(std::mem::size_of::<Image>(), 288);
    }

    #[test]
    fn test_topic_image_roundtrip() {
        let topic: Topic<Image> = Topic::new("test/image_ext_roundtrip").unwrap();

        // Allocate a small 2x3 RGB image
        let handle = topic
            .alloc_image(2, 3, ImageEncoding::Rgb8)
            .unwrap();

        assert_eq!(handle.height(), 2);
        assert_eq!(handle.width(), 3);
        assert_eq!(handle.channels(), 3);
        assert_eq!(handle.encoding(), ImageEncoding::Rgb8);

        // Write pixel data (2x3 RGB = 18 bytes)
        let pixels = handle.pixels_mut();
        assert!(pixels.len() >= 18);
        for i in 0..18 {
            pixels[i] = i as u8;
        }

        // Send (zero-copy — only descriptor through ring)
        topic.send_image(&handle);

        // Receive
        let recv_handle = topic.recv_image().expect("should receive image");
        assert_eq!(recv_handle.height(), 2);
        assert_eq!(recv_handle.width(), 3);
        assert_eq!(recv_handle.channels(), 3);
        assert_eq!(recv_handle.encoding(), ImageEncoding::Rgb8);

        // Verify pixel data (zero-copy — same shared memory)
        let recv_pixels = recv_handle.pixels();
        for i in 0..18 {
            assert_eq!(recv_pixels[i], i as u8, "pixel mismatch at index {}", i);
        }
    }

    #[test]
    fn test_topic_image_pool_shared() {
        let topic: Topic<Image> = Topic::new("test/image_ext_shared_pool").unwrap();
        let pool1 = topic.pool();
        let pool2 = topic.pool();
        assert_eq!(pool1.pool_id(), pool2.pool_id());
    }
}
