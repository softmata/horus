//! Image encoding formats for HORUS vision types
//!
//! Defines the pixel format of image data. Lives in `horus_core::types`
//! so both `horus_core` (for `Topic<Image>`) and `horus_library` (for
//! `CompressedImage`, `CameraInfo`) can reference it.

use bytemuck::{Pod, Zeroable};
use serde::{Deserialize, Serialize};
use std::fmt;

use super::dtype::TensorDtype;

/// Image encoding formats
///
/// Represents the pixel format and data layout of image data.
/// `repr(u8)` for Pod safety — stored inline in fixed-size descriptors.
#[repr(u8)]
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum ImageEncoding {
    /// 8-bit monochrome (1 channel)
    Mono8 = 0,
    /// 16-bit monochrome (1 channel)
    Mono16 = 1,
    /// 8-bit RGB (3 channels)
    #[default]
    Rgb8 = 2,
    /// 8-bit BGR (3 channels, OpenCV format)
    Bgr8 = 3,
    /// 8-bit RGBA (4 channels)
    Rgba8 = 4,
    /// 8-bit BGRA (4 channels)
    Bgra8 = 5,
    /// YUV 4:2:2 format (2 bytes per pixel)
    Yuv422 = 6,
    /// 32-bit float monochrome
    Mono32F = 7,
    /// 32-bit float RGB
    Rgb32F = 8,
    /// Bayer pattern (raw sensor data)
    BayerRggb8 = 9,
    /// 16-bit depth image (millimeters)
    Depth16 = 10,
}

// Safety: ImageEncoding is repr(u8) with valid values 0-10.
unsafe impl Pod for ImageEncoding {}
unsafe impl Zeroable for ImageEncoding {}

impl ImageEncoding {
    /// Bytes per pixel for this encoding
    #[inline]
    pub const fn bytes_per_pixel(&self) -> u32 {
        match self {
            ImageEncoding::Mono8 | ImageEncoding::BayerRggb8 => 1,
            ImageEncoding::Mono16 | ImageEncoding::Yuv422 | ImageEncoding::Depth16 => 2,
            ImageEncoding::Rgb8 | ImageEncoding::Bgr8 => 3,
            ImageEncoding::Rgba8 | ImageEncoding::Bgra8 | ImageEncoding::Mono32F => 4,
            ImageEncoding::Rgb32F => 12,
        }
    }

    /// Number of channels for this encoding
    #[inline]
    pub const fn channels(&self) -> u32 {
        match self {
            ImageEncoding::Mono8
            | ImageEncoding::Mono16
            | ImageEncoding::Mono32F
            | ImageEncoding::BayerRggb8
            | ImageEncoding::Depth16 => 1,
            ImageEncoding::Yuv422 => 2,
            ImageEncoding::Rgb8 | ImageEncoding::Bgr8 | ImageEncoding::Rgb32F => 3,
            ImageEncoding::Rgba8 | ImageEncoding::Bgra8 => 4,
        }
    }

    /// Tensor element dtype corresponding to this encoding
    #[inline]
    pub const fn tensor_dtype(&self) -> TensorDtype {
        match self {
            ImageEncoding::Mono8
            | ImageEncoding::Rgb8
            | ImageEncoding::Bgr8
            | ImageEncoding::Rgba8
            | ImageEncoding::Bgra8
            | ImageEncoding::Yuv422
            | ImageEncoding::BayerRggb8 => TensorDtype::U8,
            ImageEncoding::Mono16 | ImageEncoding::Depth16 => TensorDtype::U16,
            ImageEncoding::Mono32F | ImageEncoding::Rgb32F => TensorDtype::F32,
        }
    }

    /// Infer encoding from tensor dtype and channel count
    #[inline]
    pub const fn from_dtype_channels(dtype: TensorDtype, channels: u32) -> Self {
        match (dtype as u8, channels) {
            (8, 1) => ImageEncoding::Mono8,   // U8 = 8
            (9, 1) => ImageEncoding::Mono16,  // U16 = 9
            (0, 1) => ImageEncoding::Mono32F, // F32 = 0
            (8, 3) => ImageEncoding::Rgb8,
            (0, 3) => ImageEncoding::Rgb32F,
            (8, 4) => ImageEncoding::Rgba8,
            _ => ImageEncoding::Rgb8,
        }
    }
}

impl fmt::Display for ImageEncoding {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let name = match self {
            ImageEncoding::Mono8 => "mono8",
            ImageEncoding::Mono16 => "mono16",
            ImageEncoding::Rgb8 => "rgb8",
            ImageEncoding::Bgr8 => "bgr8",
            ImageEncoding::Rgba8 => "rgba8",
            ImageEncoding::Bgra8 => "bgra8",
            ImageEncoding::Yuv422 => "yuv422",
            ImageEncoding::Mono32F => "mono32f",
            ImageEncoding::Rgb32F => "rgb32f",
            ImageEncoding::BayerRggb8 => "bayer_rggb8",
            ImageEncoding::Depth16 => "depth16",
        };
        write!(f, "{}", name)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_encoding_bytes_per_pixel() {
        assert_eq!(ImageEncoding::Mono8.bytes_per_pixel(), 1);
        assert_eq!(ImageEncoding::Mono16.bytes_per_pixel(), 2);
        assert_eq!(ImageEncoding::Rgb8.bytes_per_pixel(), 3);
        assert_eq!(ImageEncoding::Rgba8.bytes_per_pixel(), 4);
        assert_eq!(ImageEncoding::Rgb32F.bytes_per_pixel(), 12);
    }

    #[test]
    fn test_encoding_channels() {
        assert_eq!(ImageEncoding::Mono8.channels(), 1);
        assert_eq!(ImageEncoding::Yuv422.channels(), 2);
        assert_eq!(ImageEncoding::Rgb8.channels(), 3);
        assert_eq!(ImageEncoding::Rgba8.channels(), 4);
    }

    #[test]
    fn test_encoding_tensor_dtype() {
        assert_eq!(ImageEncoding::Rgb8.tensor_dtype(), TensorDtype::U8);
        assert_eq!(ImageEncoding::Mono16.tensor_dtype(), TensorDtype::U16);
        assert_eq!(ImageEncoding::Mono32F.tensor_dtype(), TensorDtype::F32);
        assert_eq!(ImageEncoding::Depth16.tensor_dtype(), TensorDtype::U16);
    }

    #[test]
    fn test_encoding_from_dtype_channels() {
        assert_eq!(
            ImageEncoding::from_dtype_channels(TensorDtype::U8, 3),
            ImageEncoding::Rgb8
        );
        assert_eq!(
            ImageEncoding::from_dtype_channels(TensorDtype::U8, 1),
            ImageEncoding::Mono8
        );
        assert_eq!(
            ImageEncoding::from_dtype_channels(TensorDtype::F32, 1),
            ImageEncoding::Mono32F
        );
    }

    #[test]
    fn test_encoding_pod_soundness() {
        let enc = ImageEncoding::Rgb8;
        let bytes = bytemuck::bytes_of(&enc);
        assert_eq!(bytes.len(), 1);
        assert_eq!(bytes[0], 2); // Rgb8 = 2
    }

    #[test]
    fn test_encoding_serde_roundtrip() {
        for enc in [
            ImageEncoding::Mono8,
            ImageEncoding::Rgb8,
            ImageEncoding::Rgba8,
            ImageEncoding::Depth16,
        ] {
            let json = serde_json::to_string(&enc).unwrap();
            let recovered: ImageEncoding = serde_json::from_str(&json).unwrap();
            assert_eq!(recovered, enc);
        }
    }
}
