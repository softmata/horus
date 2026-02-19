//! `ImageTopic` — zero-copy image transport with `new()`, `send()`, `recv()`
//!
//! Wraps `Topic<ImageDescriptor>` internally. Users only see `Image`.

use super::pool_registry::global_pool;
use super::Topic;
use crate::error::HorusResult;
use crate::memory::image::Image;
use horus_types::ImageDescriptor;

/// Topic for sending and receiving `Image` via zero-copy shared memory.
///
/// Internally transports a 288-byte Pod descriptor through the ring buffer.
/// Pixel data stays in shared memory — no copy.
///
/// # Example
///
/// ```rust,ignore
/// use horus::prelude::*;
///
/// let topic = ImageTopic::new("camera/rgb")?;
///
/// let mut img = Image::new(480, 640, ImageEncoding::Rgb8)?;
/// img.set_pixel(100, 200, &[255, 0, 0]);
/// topic.send(&img);
///
/// if let Some(img) = topic.recv() {
///     let data = img.data();
/// }
/// ```
pub struct ImageTopic {
    inner: Topic<ImageDescriptor>,
}

impl ImageTopic {
    /// Create a new image topic.
    pub fn new(name: impl Into<String>) -> HorusResult<Self> {
        Ok(Self {
            inner: Topic::new(name)?,
        })
    }

    /// Send an image (zero-copy).
    ///
    /// Retains the tensor so it stays alive for receivers, then sends the
    /// 288-byte descriptor through the ring buffer. Pixel data stays in
    /// shared memory.
    pub fn send(&self, img: &Image) {
        img.pool().retain(img.descriptor().tensor());
        self.inner.send(*img.descriptor());
    }

    /// Receive the next image.
    ///
    /// Returns `None` if no message is available. The returned `Image`
    /// provides zero-copy access to the pixel data.
    pub fn recv(&self) -> Option<Image> {
        let descriptor = self.inner.recv()?;
        let pool = global_pool();
        Some(Image::from_owned(descriptor, pool))
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
    use horus_types::ImageEncoding;

    #[test]
    fn test_image_descriptor_is_pod() {
        assert!(is_pod::<ImageDescriptor>());
    }

    #[test]
    fn test_image_descriptor_size() {
        assert_eq!(std::mem::size_of::<ImageDescriptor>(), 288);
    }

    #[test]
    fn test_image_topic_roundtrip() {
        let topic = ImageTopic::new("test/image_topic_roundtrip").unwrap();

        let img = Image::new(2, 3, ImageEncoding::Rgb8).unwrap();

        assert_eq!(img.height(), 2);
        assert_eq!(img.width(), 3);
        assert_eq!(img.channels(), 3);
        assert_eq!(img.encoding(), ImageEncoding::Rgb8);

        // Write pixel data (2x3 RGB = 18 bytes)
        let pixels = img.data_mut();
        assert!(pixels.len() >= 18);
        for i in 0..18 {
            pixels[i] = i as u8;
        }

        topic.send(&img);

        let recv_img = topic.recv().expect("should receive image");
        assert_eq!(recv_img.height(), 2);
        assert_eq!(recv_img.width(), 3);
        assert_eq!(recv_img.channels(), 3);
        assert_eq!(recv_img.encoding(), ImageEncoding::Rgb8);

        // Verify pixel data (zero-copy — same shared memory)
        let recv_pixels = recv_img.data();
        for i in 0..18 {
            assert_eq!(recv_pixels[i], i as u8, "pixel mismatch at index {}", i);
        }
    }
}
