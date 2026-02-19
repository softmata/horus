//! Topic extension for `Topic<Image>` — zero-copy image pipeline
//!
//! Provides `create()`, `publish()`, `next()` methods on `Topic<Image>`
//! that manage the tensor pool automatically.

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

    /// Create a new image in shared memory.
    ///
    /// Allocates a tensor with shape `[height, width, channels]` from the
    /// topic's auto-managed pool. The dtype is inferred from the encoding.
    ///
    /// # Example
    ///
    /// ```rust,ignore
    /// let topic: Topic<Image> = Topic::new("camera/rgb")?;
    /// let img = topic.create(480, 640, ImageEncoding::Rgb8)?;
    /// img.pixels_mut()[0..3].copy_from_slice(&[255, 0, 0]);
    /// topic.publish(&img);
    /// ```
    pub fn create(
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

    /// Publish an image (zero-copy).
    ///
    /// Retains the tensor so it stays alive for receivers, then sends the
    /// 288-byte descriptor through the ring buffer. Pixel data stays in
    /// shared memory — no copy.
    pub fn publish(&self, handle: &ImageHandle) {
        handle.pool().retain(handle.descriptor().tensor());
        self.send(*handle.descriptor());
    }

    /// Receive the next image.
    ///
    /// Returns `None` if no message is available. The returned handle
    /// provides zero-copy access to the pixel data.
    pub fn next(&self) -> Option<ImageHandle> {
        let descriptor = self.recv()?;
        let pool = self.pool();
        Some(ImageHandle::from_owned(descriptor, pool))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::communication::pod::is_pod;

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

        let handle = topic.create(2, 3, ImageEncoding::Rgb8).unwrap();

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

        // Publish (zero-copy — only descriptor through ring)
        topic.publish(&handle);

        // Receive
        let recv_handle = topic.next().expect("should receive image");
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
