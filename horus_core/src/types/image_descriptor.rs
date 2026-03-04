//! Internal Pod image descriptor for zero-copy ring buffer transport
//!
//! `ImageDescriptor` is a fixed-size (224 byte) `repr(C)` descriptor that flows
//! through the ring buffer via the ~50ns Pod path. Actual pixel data lives in a
//! `TensorPool` — only the descriptor is copied.
//!
//! Users should use `Image` from `horus_core` which wraps this with data access.

use bytemuck::{Pod, Zeroable};
use serde::{Deserialize, Serialize};

use super::image_encoding::ImageEncoding;
use super::tensor::Tensor;

/// Unified image descriptor — Pod, 224 bytes.
///
/// Contains a `Tensor` (shape `[H, W, C]`) plus domain metadata
/// (encoding, step, frame_id, timestamp). This is what flows through
/// the ring buffer; pixel data stays in shared memory.
///
/// # Layout (224 bytes, repr(C))
///
/// ```text
/// inner:        Tensor  (168 bytes)
/// timestamp_ns: u64          (8 bytes)
/// step:         u32          (4 bytes)
/// encoding:     ImageEncoding(1 byte, repr(u8))
/// _pad:         [u8; 3]      (3 bytes)
/// frame_id:     [u8; 32]     (32 bytes)
/// _reserved:    [u8; 8]      (8 bytes)
/// Total:                      224 bytes
/// ```
#[repr(C)]
#[derive(Clone, Copy, Debug, Serialize, Deserialize)]
pub struct ImageDescriptor {
    /// Inner tensor: shape [H, W, C], data in pool
    inner: Tensor,
    /// Timestamp in nanoseconds since epoch
    timestamp_ns: u64,
    /// Bytes per row (may include padding for alignment)
    step: u32,
    /// Pixel encoding format
    encoding: ImageEncoding,
    #[serde(skip)]
    _pad: [u8; 3],
    /// Frame ID (camera identifier, null-terminated)
    frame_id: [u8; 32],
    #[serde(skip)]
    _reserved: [u8; 8],
}

// Safety: Image is repr(C), all fields are Pod, no implicit padding.
// 168 + 8 + 4 + 1 + 3 + 32 + 8 = 224 bytes, 224 % 8 = 0.
unsafe impl Zeroable for ImageDescriptor {}
unsafe impl Pod for ImageDescriptor {}

impl Default for ImageDescriptor {
    fn default() -> Self {
        Self {
            inner: Tensor::default(),
            timestamp_ns: 0,
            step: 0,
            encoding: ImageEncoding::Rgb8,
            _pad: [0; 3],
            frame_id: [0; 32],
            _reserved: [0; 8],
        }
    }
}

impl ImageDescriptor {
    /// Create a new image descriptor from pre-built tensor + metadata.
    ///
    /// The `step` (bytes per row) is taken from `tensor.strides[0]`, which
    /// reflects the actual row stride including any alignment padding.  This
    /// is correct for tensors allocated with row-padding (e.g., SIMD-aligned
    /// images).  Falls back to `width * bytes_per_pixel` only when the stride
    /// is zero (default/zeroed tensors).
    pub fn new(tensor: Tensor, encoding: ImageEncoding) -> Self {
        // Use the tensor's actual row stride (strides[0]) so that padded
        // images — where the row stride exceeds width * bpp — have their
        // step set correctly.  pixel() / set_pixel() / roi() all depend on
        // step being accurate; a wrong step causes them to read or write the
        // wrong bytes in adjacent rows.
        let step = if tensor.ndim >= 1 && tensor.strides[0] > 0 {
            tensor.strides[0] as u32
        } else {
            // Fallback for zeroed/default tensors (no meaningful stride).
            let width = if tensor.ndim >= 2 {
                tensor.shape[1] as u32
            } else {
                0
            };
            width * encoding.bytes_per_pixel()
        };

        Self {
            inner: tensor,
            timestamp_ns: 0,
            step,
            encoding,
            _pad: [0; 3],
            frame_id: [0; 32],
            _reserved: [0; 8],
        }
    }

    /// Wrap an existing `Tensor` as an `Image`, inferring encoding.
    pub fn from_tensor(tensor: Tensor) -> Self {
        let channels = if tensor.ndim >= 3 {
            tensor.shape[2] as u32
        } else {
            1
        };
        let encoding = ImageEncoding::from_dtype_channels(tensor.dtype, channels);
        Self::new(tensor, encoding)
    }

    // === Metadata accessors ===

    /// Image height in pixels (tensor dimension 0).
    #[inline]
    pub fn height(&self) -> u32 {
        if self.inner.ndim >= 1 {
            self.inner.shape[0] as u32
        } else {
            0
        }
    }

    /// Image width in pixels (tensor dimension 1).
    #[inline]
    pub fn width(&self) -> u32 {
        if self.inner.ndim >= 2 {
            self.inner.shape[1] as u32
        } else {
            0
        }
    }

    /// Number of channels (tensor dimension 2, defaults to 1).
    #[inline]
    pub fn channels(&self) -> u32 {
        if self.inner.ndim >= 3 {
            self.inner.shape[2] as u32
        } else {
            1
        }
    }

    /// Pixel encoding format.
    #[inline]
    pub fn encoding(&self) -> ImageEncoding {
        self.encoding
    }

    /// Bytes per row.
    #[inline]
    pub fn step(&self) -> u32 {
        self.step
    }

    crate::impl_tensor_accessors!();
    crate::impl_timestamp_field!();
    crate::impl_frame_id_field!();
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::types::{Device, TensorDtype};

    #[test]
    fn test_image_size() {
        assert_eq!(
            std::mem::size_of::<ImageDescriptor>(),
            224,
            "ImageDescriptor must be exactly 224 bytes"
        );
    }

    #[test]
    fn test_image_pod() {
        // Verify Pod by roundtripping through bytes
        let img = ImageDescriptor::default();
        let bytes: &[u8] = bytemuck::bytes_of(&img);
        assert_eq!(bytes.len(), 224);
        let _recovered: &ImageDescriptor = bytemuck::from_bytes(bytes);
    }

    #[test]
    fn test_image_from_tensor() {
        let tensor = Tensor::new(1, 0, 0, 0, &[480, 640, 3], TensorDtype::U8, Device::cpu());
        let img = ImageDescriptor::from_tensor(tensor);
        assert_eq!(img.height(), 480);
        assert_eq!(img.width(), 640);
        assert_eq!(img.channels(), 3);
        assert_eq!(img.encoding(), ImageEncoding::Rgb8);
        assert_eq!(img.step(), 640 * 3);
    }

    #[test]
    fn test_image_mono() {
        let tensor = Tensor::new(1, 0, 0, 0, &[100, 200], TensorDtype::U8, Device::cpu());
        let img = ImageDescriptor::from_tensor(tensor);
        assert_eq!(img.channels(), 1);
        assert_eq!(img.encoding(), ImageEncoding::Mono8);
    }

    #[test]
    fn test_image_frame_id() {
        let mut img = ImageDescriptor::default();
        img.set_frame_id("camera_left");
        assert_eq!(img.frame_id(), "camera_left");
    }

    #[test]
    fn test_image_step_uses_tensor_stride() {
        // Create a tensor with a padded row stride (e.g., SIMD-aligned rows).
        // strides[0] = width * channels + 16 bytes of padding per row.
        let mut tensor = Tensor::new(1, 0, 0, 0, &[480, 640, 3], TensorDtype::U8, Device::cpu());
        let padded_stride = 640u64 * 3 + 16;
        tensor.strides[0] = padded_stride;

        let img = ImageDescriptor::new(tensor, ImageEncoding::Rgb8);

        // step must reflect the padded stride, not width * bpp.
        assert_eq!(
            img.step(),
            padded_stride as u32,
            "step should equal tensor.strides[0] for padded images"
        );
        assert_ne!(
            img.step(),
            640 * 3,
            "step must NOT equal width*bpp when there is row padding"
        );
    }

    #[test]
    fn test_image_step_fallback_for_zeroed_tensor() {
        // A zeroed (default) tensor has strides[0] == 0 — fallback to width*bpp.
        let tensor = Tensor::default(); // all zeros, ndim == 0
        let img = ImageDescriptor::new(tensor, ImageEncoding::Rgb8);
        // Both width and step should be 0 for an empty descriptor.
        assert_eq!(img.step(), 0);
    }

    #[test]
    fn test_image_serde_roundtrip() {
        let tensor = Tensor::new(
            1,
            42,
            1,
            0,
            &[1080, 1920, 3],
            TensorDtype::U8,
            Device::cpu(),
        );
        let mut img = ImageDescriptor::new(tensor, ImageEncoding::Rgb8);
        img.set_frame_id("cam0");
        img.set_timestamp_ns(123456789);

        let json = serde_json::to_string(&img).unwrap();
        let recovered: ImageDescriptor = serde_json::from_str(&json).unwrap();
        assert_eq!(recovered.height(), 1080);
        assert_eq!(recovered.width(), 1920);
        assert_eq!(recovered.encoding(), ImageEncoding::Rgb8);
        assert_eq!(recovered.frame_id(), "cam0");
        assert_eq!(recovered.timestamp_ns(), 123456789);
    }
}
