//! RAII handle for Image tensor data access
//!
//! `ImageHandle` wraps an `Image` descriptor + `Arc<TensorPool>` and provides
//! ergonomic pixel access methods. Clone increments the refcount; Drop releases it.

use std::sync::Arc;

use horus_types::{Image, ImageEncoding, TensorDtype};

use super::tensor_pool::TensorPool;

/// RAII handle providing data access for an `Image` descriptor.
///
/// When cloned, the tensor's refcount is incremented. When dropped,
/// it is decremented. When the count reaches zero, the memory slot
/// is returned to the pool.
pub struct ImageHandle {
    descriptor: Image,
    pool: Arc<TensorPool>,
}

impl ImageHandle {
    /// Create a handle from a descriptor that already has a refcount of 1.
    ///
    /// Used internally after allocation or after send bumps the refcount.
    pub(crate) fn from_owned(descriptor: Image, pool: Arc<TensorPool>) -> Self {
        Self { descriptor, pool }
    }

    // === Pixel data access ===

    /// Get pixel data as a byte slice (zero-copy).
    #[inline]
    pub fn pixels(&self) -> &[u8] {
        self.pool.data_slice(self.descriptor.tensor())
    }

    /// Get pixel data as a mutable byte slice (zero-copy).
    #[inline]
    #[allow(clippy::mut_from_ref)]
    pub fn pixels_mut(&self) -> &mut [u8] {
        self.pool.data_slice_mut(self.descriptor.tensor())
    }

    /// Get a single pixel at (x, y) as a byte slice.
    ///
    /// Returns `None` if coordinates are out of bounds.
    pub fn get_pixel(&self, x: u32, y: u32) -> Option<&[u8]> {
        if x >= self.width() || y >= self.height() {
            return None;
        }
        let bpp = self.encoding().bytes_per_pixel() as usize;
        let offset = (y * self.step() + x * self.encoding().bytes_per_pixel()) as usize;
        let data = self.pixels();
        if offset + bpp <= data.len() {
            Some(&data[offset..offset + bpp])
        } else {
            None
        }
    }

    /// Extract a region of interest as a new `Vec<u8>`.
    ///
    /// Returns `None` if the ROI extends beyond image bounds.
    pub fn roi(&self, x: u32, y: u32, w: u32, h: u32) -> Option<Vec<u8>> {
        if x + w > self.width() || y + h > self.height() {
            return None;
        }
        let bpp = self.encoding().bytes_per_pixel() as usize;
        let data = self.pixels();
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

    /// Get pixel data as a typed slice.
    ///
    /// # Safety
    /// Caller must ensure the dtype matches the requested type T.
    #[inline]
    pub unsafe fn pixels_as<T: Copy>(&self) -> &[T] {
        let bytes = self.pixels();
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
    pub unsafe fn pixels_as_mut<T: Copy>(&self) -> &mut [T] {
        let bytes = self.pixels_mut();
        let ptr = bytes.as_mut_ptr() as *mut T;
        let len = bytes.len() / std::mem::size_of::<T>();
        std::slice::from_raw_parts_mut(ptr, len)
    }

    // === Metadata accessors (delegate to descriptor) ===

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

    /// Get the underlying descriptor.
    #[inline]
    pub fn descriptor(&self) -> &Image {
        &self.descriptor
    }

    /// Get a mutable reference to the descriptor.
    #[inline]
    pub fn descriptor_mut(&mut self) -> &mut Image {
        &mut self.descriptor
    }

    /// Get the pool.
    #[inline]
    pub fn pool(&self) -> &Arc<TensorPool> {
        &self.pool
    }

    /// Current reference count.
    #[inline]
    pub fn refcount(&self) -> u32 {
        self.pool.refcount(self.descriptor.tensor())
    }
}

impl Clone for ImageHandle {
    fn clone(&self) -> Self {
        self.pool.retain(self.descriptor.tensor());
        Self {
            descriptor: self.descriptor,
            pool: Arc::clone(&self.pool),
        }
    }
}

impl Drop for ImageHandle {
    fn drop(&mut self) {
        self.pool.release(self.descriptor.tensor());
    }
}

impl std::fmt::Debug for ImageHandle {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("ImageHandle")
            .field("width", &self.width())
            .field("height", &self.height())
            .field("encoding", &self.encoding())
            .field("refcount", &self.refcount())
            .finish()
    }
}

// Safety: ImageHandle can be sent between threads.
// The underlying pool uses atomic operations for all shared state.
unsafe impl Send for ImageHandle {}
unsafe impl Sync for ImageHandle {}
