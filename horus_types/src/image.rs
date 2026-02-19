//! Internal Pod image descriptor for zero-copy ring buffer transport
//!
//! `ImageDescriptor` is a fixed-size (288 byte) `repr(C)` descriptor that flows
//! through the ring buffer via the ~50ns Pod path. Actual pixel data lives in a
//! `TensorPool` — only the descriptor is copied.
//!
//! Users should use `Image` from `horus_core` which wraps this with data access.

use bytemuck::{Pod, Zeroable};
use serde::{Deserialize, Serialize};

use crate::image_encoding::ImageEncoding;
use crate::tensor::HorusTensor;
use crate::TensorDtype;

/// Unified image descriptor — Pod, 288 bytes.
///
/// Contains a `HorusTensor` (shape `[H, W, C]`) plus domain metadata
/// (encoding, step, frame_id, timestamp). This is what flows through
/// the ring buffer; pixel data stays in shared memory.
///
/// # Layout (288 bytes, repr(C))
///
/// ```text
/// inner:        HorusTensor  (232 bytes)
/// timestamp_ns: u64          (8 bytes)
/// step:         u32          (4 bytes)
/// encoding:     ImageEncoding(1 byte, repr(u8))
/// _pad:         [u8; 3]      (3 bytes)
/// frame_id:     [u8; 32]     (32 bytes)
/// _reserved:    [u8; 8]      (8 bytes)
/// Total:                      288 bytes
/// ```
#[repr(C)]
#[derive(Clone, Copy, Debug, Serialize, Deserialize)]
pub struct ImageDescriptor {
    /// Inner tensor: shape [H, W, C], data in pool
    inner: HorusTensor,
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
// 232 + 8 + 4 + 1 + 3 + 32 + 8 = 288 bytes, 288 % 8 = 0.
unsafe impl Zeroable for ImageDescriptor {}
unsafe impl Pod for ImageDescriptor {}

impl Default for ImageDescriptor {
    fn default() -> Self {
        Self {
            inner: HorusTensor::default(),
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
    pub fn new(tensor: HorusTensor, encoding: ImageEncoding) -> Self {
        let width = if tensor.ndim >= 2 {
            tensor.shape[1] as u32
        } else {
            0
        };
        let step = width * encoding.bytes_per_pixel();

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

    /// Wrap an existing `HorusTensor` as an `Image`, inferring encoding.
    pub fn from_tensor(tensor: HorusTensor) -> Self {
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

    /// Element data type.
    #[inline]
    pub fn dtype(&self) -> TensorDtype {
        self.inner.dtype
    }

    /// Total pixel count (height * width).
    #[inline]
    pub fn pixel_count(&self) -> u64 {
        self.height() as u64 * self.width() as u64
    }

    /// Total bytes of image data.
    #[inline]
    pub fn nbytes(&self) -> u64 {
        self.inner.nbytes()
    }

    /// Bytes per row.
    #[inline]
    pub fn step(&self) -> u32 {
        self.step
    }

    /// Whether tensor data is on CPU.
    #[inline]
    pub fn is_cpu(&self) -> bool {
        self.inner.is_cpu()
    }

    /// Whether tensor data is on a CUDA GPU.
    #[inline]
    pub fn is_cuda(&self) -> bool {
        self.inner.is_cuda()
    }

    /// Timestamp in nanoseconds since epoch.
    #[inline]
    pub fn timestamp_ns(&self) -> u64 {
        self.timestamp_ns
    }

    /// Set the timestamp.
    #[inline]
    pub fn set_timestamp_ns(&mut self, ts: u64) {
        self.timestamp_ns = ts;
    }

    /// Get frame ID as string.
    pub fn frame_id(&self) -> &str {
        let end = self.frame_id.iter().position(|&b| b == 0).unwrap_or(32);
        std::str::from_utf8(&self.frame_id[..end]).unwrap_or("")
    }

    /// Set frame ID from string.
    pub fn set_frame_id(&mut self, id: &str) {
        let bytes = id.as_bytes();
        let len = bytes.len().min(31);
        self.frame_id = [0; 32];
        self.frame_id[..len].copy_from_slice(&bytes[..len]);
    }

    // === Tensor access (for pool operations) ===

    /// Get the inner tensor descriptor.
    #[inline]
    pub fn tensor(&self) -> &HorusTensor {
        &self.inner
    }

    /// Get a mutable reference to the inner tensor descriptor.
    #[inline]
    pub fn tensor_mut(&mut self) -> &mut HorusTensor {
        &mut self.inner
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::Device;

    #[test]
    fn test_image_size() {
        assert_eq!(
            std::mem::size_of::<ImageDescriptor>(),
            288,
            "ImageDescriptor must be exactly 288 bytes"
        );
    }

    #[test]
    fn test_image_pod() {
        // Verify Pod by roundtripping through bytes
        let img = ImageDescriptor::default();
        let bytes: &[u8] = bytemuck::bytes_of(&img);
        assert_eq!(bytes.len(), 288);
        let _recovered: &ImageDescriptor = bytemuck::from_bytes(bytes);
    }

    #[test]
    fn test_image_from_tensor() {
        let tensor = HorusTensor::new(1, 0, 0, 0, &[480, 640, 3], TensorDtype::U8, Device::cpu());
        let img = ImageDescriptor::from_tensor(tensor);
        assert_eq!(img.height(), 480);
        assert_eq!(img.width(), 640);
        assert_eq!(img.channels(), 3);
        assert_eq!(img.encoding(), ImageEncoding::Rgb8);
        assert_eq!(img.step(), 640 * 3);
    }

    #[test]
    fn test_image_mono() {
        let tensor = HorusTensor::new(1, 0, 0, 0, &[100, 200], TensorDtype::U8, Device::cpu());
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
    fn test_image_serde_roundtrip() {
        let tensor =
            HorusTensor::new(1, 42, 1, 0, &[1080, 1920, 3], TensorDtype::U8, Device::cpu());
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
