//! Tensor-backed image for zero-copy camera pipelines
//!
//! `TensorImage` wraps a `HorusTensor` with image-specific accessors.
//! Because it's Pod, `Topic<TensorImage>` uses the zero-copy shared memory
//! path (~50ns) instead of serde serialization (~167ns).
//!
//! # When to use
//!
//! - **`TensorImage`**: High-throughput pipelines (1080p @ 30fps, ML inference).
//!   Data lives in a shared-memory `TensorPool` — only the 232-byte descriptor
//!   is sent through the ring buffer.
//! - **`Image`** (vision module): General-purpose image messages with rich API
//!   (pixel access, ROI, etc.) that go through the serde path.
//!
//! # Example
//!
//! ```rust,ignore
//! use horus::prelude::*;
//!
//! let topic: Topic<HorusTensor> = Topic::new("camera/rgb")?;
//!
//! // Allocate a 1080p RGB image from the topic's pool
//! let handle = topic.alloc_tensor(&[1080, 1920, 3], TensorDtype::U8, Device::cpu())?;
//! // ... fill pixel data via handle.data_slice_mut() ...
//! topic.send_handle(&handle)?;
//!
//! // Receiver side — wrap in TensorImage for domain-specific accessors
//! let recv = topic.recv_handle().unwrap();
//! let img = TensorImage::from_tensor(*recv.tensor());
//! assert_eq!(img.width(), 1920);
//! assert_eq!(img.height(), 1080);
//! ```

use bytemuck::{Pod, Zeroable};
use horus_core::communication::PodMessage;
use horus_core::core::LogSummary;
use horus_types::{HorusTensor, TensorDtype};
use serde::{Deserialize, Serialize};

use super::vision::ImageEncoding;

/// A camera image backed by a shared-memory tensor.
///
/// Zero-overhead wrapper around `HorusTensor`. The tensor shape encodes
/// `[height, width, channels]` (row-major). Because this is Pod,
/// `Topic<TensorImage>` routes through the ~50ns zero-copy path.
#[repr(C)]
#[derive(Clone, Copy, Debug, Serialize, Deserialize)]
pub struct TensorImage {
    inner: HorusTensor,
}

// Safety: TensorImage is repr(C) and contains only Pod fields (HorusTensor is Pod).
unsafe impl Zeroable for TensorImage {}
unsafe impl Pod for TensorImage {}
unsafe impl PodMessage for TensorImage {}

impl TensorImage {
    /// Wrap an existing `HorusTensor` as a `TensorImage`.
    ///
    /// The tensor should have shape `[height, width, channels]` or `[height, width]`
    /// for single-channel images.
    pub fn from_tensor(tensor: HorusTensor) -> Self {
        Self { inner: tensor }
    }

    /// Get the inner tensor descriptor.
    pub fn tensor(&self) -> &HorusTensor {
        &self.inner
    }

    /// Get a mutable reference to the inner tensor descriptor.
    pub fn tensor_mut(&mut self) -> &mut HorusTensor {
        &mut self.inner
    }

    /// Image height in pixels (shape dimension 0).
    pub fn height(&self) -> u32 {
        let shape = self.inner.shape();
        if shape.is_empty() {
            0
        } else {
            shape[0] as u32
        }
    }

    /// Image width in pixels (shape dimension 1).
    pub fn width(&self) -> u32 {
        let shape = self.inner.shape();
        if shape.len() < 2 {
            0
        } else {
            shape[1] as u32
        }
    }

    /// Number of channels (shape dimension 2, defaults to 1 for 2D tensors).
    pub fn channels(&self) -> u32 {
        let shape = self.inner.shape();
        if shape.len() < 3 {
            1
        } else {
            shape[2] as u32
        }
    }

    /// Data type of each pixel component.
    pub fn dtype(&self) -> TensorDtype {
        self.inner.dtype
    }

    /// Infer the image encoding from dtype and channel count.
    ///
    /// Returns the most likely encoding; users can override if needed.
    pub fn inferred_encoding(&self) -> ImageEncoding {
        match (self.channels(), self.dtype()) {
            (1, TensorDtype::U8) => ImageEncoding::Mono8,
            (1, TensorDtype::U16) => ImageEncoding::Mono16,
            (1, TensorDtype::F32) => ImageEncoding::Mono32F,
            (3, TensorDtype::U8) => ImageEncoding::Rgb8,
            (3, TensorDtype::F32) => ImageEncoding::Rgb32F,
            (4, TensorDtype::U8) => ImageEncoding::Rgba8,
            _ => ImageEncoding::Rgb8, // default fallback
        }
    }

    /// Total number of pixels (height * width).
    pub fn pixel_count(&self) -> u64 {
        self.height() as u64 * self.width() as u64
    }

    /// Total bytes of image data.
    pub fn nbytes(&self) -> u64 {
        self.inner.nbytes()
    }

    /// Whether the tensor data lives on CPU.
    pub fn is_cpu(&self) -> bool {
        self.inner.is_cpu()
    }

    /// Whether the tensor data lives on a CUDA GPU.
    pub fn is_cuda(&self) -> bool {
        self.inner.is_cuda()
    }
}

impl LogSummary for TensorImage {
    fn log_summary(&self) -> String {
        format!(
            "TensorImage({}x{}, ch={}, {:?}, {})",
            self.width(),
            self.height(),
            self.channels(),
            self.dtype(),
            self.inner.device(),
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_tensor_image_size() {
        assert_eq!(
            std::mem::size_of::<TensorImage>(),
            std::mem::size_of::<HorusTensor>()
        );
    }

    #[test]
    fn test_tensor_image_accessors() {
        let mut tensor = <HorusTensor as Zeroable>::zeroed();
        tensor.ndim = 3;
        tensor.dtype = TensorDtype::U8;
        tensor.shape[0] = 480;
        tensor.shape[1] = 640;
        tensor.shape[2] = 3;

        let img = TensorImage::from_tensor(tensor);
        assert_eq!(img.height(), 480);
        assert_eq!(img.width(), 640);
        assert_eq!(img.channels(), 3);
        assert_eq!(img.dtype(), TensorDtype::U8);
        assert_eq!(img.inferred_encoding(), ImageEncoding::Rgb8);
        assert_eq!(img.pixel_count(), 480 * 640);
    }

    #[test]
    fn test_tensor_image_mono() {
        let mut tensor = <HorusTensor as Zeroable>::zeroed();
        tensor.ndim = 2;
        tensor.dtype = TensorDtype::U8;
        tensor.shape[0] = 100;
        tensor.shape[1] = 200;

        let img = TensorImage::from_tensor(tensor);
        assert_eq!(img.channels(), 1);
        assert_eq!(img.inferred_encoding(), ImageEncoding::Mono8);
    }
}
