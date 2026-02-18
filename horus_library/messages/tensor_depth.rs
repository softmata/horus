//! Tensor-backed depth image for zero-copy depth sensor pipelines
//!
//! `TensorDepthImage` wraps a `HorusTensor` with depth-image-specific accessors.
//! Because it's Pod, `Topic<TensorDepthImage>` uses the zero-copy shared memory
//! path (~50ns) instead of serde serialization.
//!
//! # Layout
//!
//! The tensor shape is `[height, width]` with a single-channel dtype:
//! - `F32` — depth in meters (most common for ML pipelines)
//! - `U16` — depth in millimeters (common for depth sensors like RealSense)

use bytemuck::{Pod, Zeroable};
use horus_core::communication::PodMessage;
use horus_core::core::LogSummary;
use horus_types::{HorusTensor, TensorDtype};
use serde::{Deserialize, Serialize};

/// A depth image backed by a shared-memory tensor.
///
/// Zero-overhead wrapper around `HorusTensor`. The tensor shape encodes
/// `[height, width]`. Because this is Pod, `Topic<TensorDepthImage>` routes
/// through the ~50ns zero-copy path.
#[repr(C)]
#[derive(Clone, Copy, Debug, Serialize, Deserialize)]
pub struct TensorDepthImage {
    inner: HorusTensor,
}

// Safety: TensorDepthImage is repr(C) and contains only Pod fields.
unsafe impl Zeroable for TensorDepthImage {}
unsafe impl Pod for TensorDepthImage {}
unsafe impl PodMessage for TensorDepthImage {}

impl TensorDepthImage {
    /// Wrap an existing `HorusTensor` as a `TensorDepthImage`.
    ///
    /// The tensor should have shape `[height, width]` with dtype F32 or U16.
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

    /// Data type of depth values.
    pub fn dtype(&self) -> TensorDtype {
        self.inner.dtype
    }

    /// Whether depth values are in meters (F32).
    pub fn is_meters(&self) -> bool {
        self.inner.dtype == TensorDtype::F32
    }

    /// Whether depth values are in millimeters (U16).
    pub fn is_millimeters(&self) -> bool {
        self.inner.dtype == TensorDtype::U16
    }

    /// Total number of pixels (height * width).
    pub fn pixel_count(&self) -> u64 {
        self.height() as u64 * self.width() as u64
    }

    /// Total bytes of depth data.
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

impl LogSummary for TensorDepthImage {
    fn log_summary(&self) -> String {
        let unit = if self.is_meters() {
            "m"
        } else if self.is_millimeters() {
            "mm"
        } else {
            "?"
        };
        format!(
            "TensorDepthImage({}x{}, {}, {})",
            self.width(),
            self.height(),
            unit,
            self.inner.device(),
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_tensor_depth_size() {
        assert_eq!(
            std::mem::size_of::<TensorDepthImage>(),
            std::mem::size_of::<HorusTensor>()
        );
    }

    #[test]
    fn test_tensor_depth_f32() {
        let mut tensor = <HorusTensor as Zeroable>::zeroed();
        tensor.ndim = 2;
        tensor.dtype = TensorDtype::F32;
        tensor.shape[0] = 480;
        tensor.shape[1] = 640;

        let depth = TensorDepthImage::from_tensor(tensor);
        assert_eq!(depth.height(), 480);
        assert_eq!(depth.width(), 640);
        assert!(depth.is_meters());
        assert!(!depth.is_millimeters());
    }

    #[test]
    fn test_tensor_depth_u16() {
        let mut tensor = <HorusTensor as Zeroable>::zeroed();
        tensor.ndim = 2;
        tensor.dtype = TensorDtype::U16;
        tensor.shape[0] = 720;
        tensor.shape[1] = 1280;

        let depth = TensorDepthImage::from_tensor(tensor);
        assert_eq!(depth.height(), 720);
        assert_eq!(depth.width(), 1280);
        assert!(!depth.is_meters());
        assert!(depth.is_millimeters());
    }
}
