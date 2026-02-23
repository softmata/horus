//! Image topic tests — validates Topic<Image> zero-copy roundtrip

#[cfg(test)]
mod tests {
    use crate::communication::pod::is_pod;
    use crate::communication::topic::Topic;
    use crate::memory::image::Image;
    use horus_types::{ImageDescriptor, ImageEncoding};

    #[test]
    fn test_image_descriptor_is_pod() {
        assert!(is_pod::<ImageDescriptor>());
    }

    #[test]
    fn test_image_descriptor_size() {
        assert_eq!(std::mem::size_of::<ImageDescriptor>(), 224);
    }

    #[test]
    fn test_image_topic_roundtrip() {
        let topic: Topic<Image> = Topic::new("test/image_topic_roundtrip_unified").unwrap();

        let img = Image::new(2, 3, ImageEncoding::Rgb8).unwrap();

        assert_eq!(img.height(), 2);
        assert_eq!(img.width(), 3);
        assert_eq!(img.channels(), 3);
        assert_eq!(img.encoding(), ImageEncoding::Rgb8);

        // Write pixel data (2x3 RGB = 18 bytes)
        let pixels = img.data_mut();
        assert!(pixels.len() >= 18);
        for (i, pixel) in pixels[..18].iter_mut().enumerate() {
            *pixel = i as u8;
        }

        topic.send(&img);

        let recv_img = topic.recv().expect("should receive image");
        assert_eq!(recv_img.height(), 2);
        assert_eq!(recv_img.width(), 3);
        assert_eq!(recv_img.channels(), 3);
        assert_eq!(recv_img.encoding(), ImageEncoding::Rgb8);

        // Verify pixel data (zero-copy — same shared memory)
        let recv_pixels = recv_img.data();
        for (i, pixel) in recv_pixels[..18].iter().enumerate() {
            assert_eq!(*pixel, i as u8, "pixel mismatch at index {}", i);
        }
    }
}
