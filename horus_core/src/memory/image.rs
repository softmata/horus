//! User-facing Image type with zero-copy shared memory backing
//!
//! `Image` is the primary type for image data in HORUS. It wraps a Pod
//! descriptor + `Arc<TensorPool>` and provides a rich API for pixel access.
//! Clone increments the refcount; Drop releases it.
//!
//! # Example
//!
//! ```rust,ignore
//! use horus::prelude::*;
//!
//! let mut img = Image::new(480, 640, ImageEncoding::Rgb8)?;
//! img.set_pixel(100, 200, &[255, 0, 0]);
//! topic.send(&img);
//! ```

use std::sync::Arc;

use horus_types::{Device, ImageDescriptor, ImageEncoding, TensorDtype};

use super::tensor_pool::TensorPool;
use crate::communication::topic::pool_registry::global_pool;
use crate::error::HorusResult;

/// Image with zero-copy shared memory backing.
///
/// Create with `Image::new()`, access pixels with `data()` / `data_mut()`,
/// and send through topics with `topic.send(&img)`.
///
/// When cloned, the tensor's refcount is incremented. When dropped,
/// it is decremented. When the count reaches zero, the memory slot
/// is returned to the pool.
pub struct Image {
    descriptor: ImageDescriptor,
    pool: Arc<TensorPool>,
}

impl Image {
    /// Create a new image with the given dimensions and encoding.
    ///
    /// Allocates shared memory from a global pool. Pixel data is
    /// zero-initialized.
    ///
    /// # Example
    ///
    /// ```rust,ignore
    /// let img = Image::new(480, 640, ImageEncoding::Rgb8)?;
    /// assert_eq!(img.width(), 640);
    /// assert_eq!(img.height(), 480);
    /// ```
    pub fn new(height: u32, width: u32, encoding: ImageEncoding) -> HorusResult<Self> {
        let pool = global_pool();
        let channels = encoding.channels();
        let dtype = encoding.tensor_dtype();

        let shape = if channels == 1 {
            vec![height as u64, width as u64]
        } else {
            vec![height as u64, width as u64, channels as u64]
        };

        let tensor = pool.alloc(&shape, dtype, Device::cpu())?;
        let descriptor = ImageDescriptor::new(tensor, encoding);
        Ok(Self { descriptor, pool })
    }

    /// Create an Image from a descriptor and pool (internal).
    ///
    /// Used by Topic extensions after allocation or receive.
    pub(crate) fn from_owned(descriptor: ImageDescriptor, pool: Arc<TensorPool>) -> Self {
        Self { descriptor, pool }
    }

    // === Pixel data access ===

    /// Get raw pixel data as a byte slice (zero-copy).
    #[inline]
    pub fn data(&self) -> &[u8] {
        self.pool.data_slice(self.descriptor.tensor())
    }

    /// Get raw pixel data as a mutable byte slice (zero-copy).
    #[inline]
    #[allow(clippy::mut_from_ref)]
    pub fn data_mut(&self) -> &mut [u8] {
        self.pool.data_slice_mut(self.descriptor.tensor())
    }

    /// Copy pixel data from a buffer into this image.
    ///
    /// Returns `&mut Self` for method chaining.
    ///
    /// # Panics
    ///
    /// Panics if `src` length doesn't match `nbytes()`.
    pub fn copy_from(&mut self, src: &[u8]) -> &mut Self {
        let data = self.data_mut();
        assert_eq!(
            src.len(),
            data.len(),
            "source buffer size ({}) doesn't match image size ({})",
            src.len(),
            data.len()
        );
        data.copy_from_slice(src);
        self
    }

    /// Get a single pixel at (x, y) as a byte slice.
    ///
    /// Returns `None` if coordinates are out of bounds.
    pub fn pixel(&self, x: u32, y: u32) -> Option<&[u8]> {
        if x >= self.width() || y >= self.height() {
            return None;
        }
        let bpp = self.encoding().bytes_per_pixel() as usize;
        let offset = (y * self.step() + x * self.encoding().bytes_per_pixel()) as usize;
        let data = self.data();
        if offset + bpp <= data.len() {
            Some(&data[offset..offset + bpp])
        } else {
            None
        }
    }

    /// Set a single pixel at (x, y) from a byte slice.
    ///
    /// Returns `&mut Self` for method chaining. Does nothing if out of bounds
    /// or if `value` length doesn't match bytes-per-pixel.
    pub fn set_pixel(&mut self, x: u32, y: u32, value: &[u8]) -> &mut Self {
        if x >= self.width() || y >= self.height() {
            return self;
        }
        let bpp = self.encoding().bytes_per_pixel() as usize;
        if value.len() != bpp {
            return self;
        }
        let offset = (y * self.step() + x * self.encoding().bytes_per_pixel()) as usize;
        let data = self.data_mut();
        if offset + bpp <= data.len() {
            data[offset..offset + bpp].copy_from_slice(value);
        }
        self
    }

    /// Fill the entire image with a single color value.
    ///
    /// `value` must match the bytes-per-pixel of the encoding.
    /// Returns `&mut Self` for method chaining.
    pub fn fill(&mut self, value: &[u8]) -> &mut Self {
        let bpp = self.encoding().bytes_per_pixel() as usize;
        if value.len() != bpp {
            return self;
        }
        let data = self.data_mut();
        for chunk in data.chunks_exact_mut(bpp) {
            chunk.copy_from_slice(value);
        }
        self
    }

    /// Extract a region of interest as a new `Vec<u8>`.
    ///
    /// Returns `None` if the ROI extends beyond image bounds.
    pub fn roi(&self, x: u32, y: u32, w: u32, h: u32) -> Option<Vec<u8>> {
        if x + w > self.width() || y + h > self.height() {
            return None;
        }
        let bpp = self.encoding().bytes_per_pixel() as usize;
        let data = self.data();
        let step = self.step() as usize;
        let row_bytes = w as usize * bpp;
        let mut result = Vec::with_capacity(row_bytes * h as usize);

        for row in y..y + h {
            let start = row as usize * step + x as usize * bpp;
            let end = start + row_bytes;
            if end <= data.len() {
                result.extend_from_slice(&data[start..end]);
            }
        }

        Some(result)
    }

    /// Set the frame ID (camera identifier).
    ///
    /// Returns `&mut Self` for method chaining.
    pub fn set_frame_id(&mut self, id: &str) -> &mut Self {
        self.descriptor.set_frame_id(id);
        self
    }

    /// Set the timestamp in nanoseconds.
    ///
    /// Returns `&mut Self` for method chaining.
    pub fn set_timestamp_ns(&mut self, ts: u64) -> &mut Self {
        self.descriptor.set_timestamp_ns(ts);
        self
    }

    /// Get pixel data as a typed slice.
    ///
    /// # Safety
    /// Caller must ensure the dtype matches the requested type T.
    #[inline]
    pub unsafe fn data_as<T: Copy>(&self) -> &[T] {
        let bytes = self.data();
        let ptr = bytes.as_ptr() as *const T;
        let len = bytes.len() / std::mem::size_of::<T>();
        std::slice::from_raw_parts(ptr, len)
    }

    /// Get pixel data as a mutable typed slice.
    ///
    /// # Safety
    /// Caller must ensure the dtype matches the requested type T.
    #[inline]
    #[allow(clippy::mut_from_ref)]
    pub unsafe fn data_as_mut<T: Copy>(&self) -> &mut [T] {
        let bytes = self.data_mut();
        let ptr = bytes.as_mut_ptr() as *mut T;
        let len = bytes.len() / std::mem::size_of::<T>();
        std::slice::from_raw_parts_mut(ptr, len)
    }

    // === Metadata accessors ===

    /// Image height in pixels.
    #[inline]
    pub fn height(&self) -> u32 {
        self.descriptor.height()
    }

    /// Image width in pixels.
    #[inline]
    pub fn width(&self) -> u32 {
        self.descriptor.width()
    }

    /// Number of channels.
    #[inline]
    pub fn channels(&self) -> u32 {
        self.descriptor.channels()
    }

    /// Pixel encoding format.
    #[inline]
    pub fn encoding(&self) -> ImageEncoding {
        self.descriptor.encoding()
    }

    /// Element data type.
    #[inline]
    pub fn dtype(&self) -> TensorDtype {
        self.descriptor.dtype()
    }

    /// Bytes per row.
    #[inline]
    pub fn step(&self) -> u32 {
        self.descriptor.step()
    }

    /// Total bytes of image data.
    #[inline]
    pub fn nbytes(&self) -> u64 {
        self.descriptor.nbytes()
    }

    /// Whether tensor data is on CPU.
    #[inline]
    pub fn is_cpu(&self) -> bool {
        self.descriptor.is_cpu()
    }

    /// Whether tensor data is on CUDA.
    #[inline]
    pub fn is_cuda(&self) -> bool {
        self.descriptor.is_cuda()
    }

    /// Timestamp in nanoseconds.
    #[inline]
    pub fn timestamp_ns(&self) -> u64 {
        self.descriptor.timestamp_ns()
    }

    /// Frame ID.
    #[inline]
    pub fn frame_id(&self) -> &str {
        self.descriptor.frame_id()
    }

    // === Internal accessors (for Topic extensions) ===

    /// Get the underlying descriptor (internal).
    #[inline]
    pub(crate) fn descriptor(&self) -> &ImageDescriptor {
        &self.descriptor
    }

    /// Get the pool (internal).

    #[inline]
    pub(crate) fn pool(&self) -> &Arc<TensorPool> {
        &self.pool
    }

}

impl Clone for Image {
    fn clone(&self) -> Self {
        self.pool.retain(self.descriptor.tensor());
        Self {
            descriptor: self.descriptor,
            pool: Arc::clone(&self.pool),
        }
    }
}

impl Drop for Image {
    fn drop(&mut self) {
        self.pool.release(self.descriptor.tensor());
    }
}

impl std::fmt::Debug for Image {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("Image")
            .field("width", &self.width())
            .field("height", &self.height())
            .field("encoding", &self.encoding())
            .finish()
    }
}

// Safety: Image can be sent between threads.
// The underlying pool uses atomic operations for all shared state.
unsafe impl Send for Image {}
unsafe impl Sync for Image {}
