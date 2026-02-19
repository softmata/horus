//! RAII handle for DepthImage tensor data access
//!
//! `DepthImageHandle` wraps a `DepthImage` descriptor + `Arc<TensorPool>` and
//! provides ergonomic depth value access methods.

use std::sync::Arc;

use horus_types::{DepthImage, TensorDtype};

use super::tensor_pool::TensorPool;

/// RAII handle providing data access for a `DepthImage` descriptor.
pub struct DepthImageHandle {
    descriptor: DepthImage,
    pool: Arc<TensorPool>,
}

impl DepthImageHandle {
    /// Create a handle from a descriptor that already has a refcount of 1.
    pub(crate) fn from_owned(descriptor: DepthImage, pool: Arc<TensorPool>) -> Self {
        Self { descriptor, pool }
    }

    // === Depth data access ===

    /// Get raw depth data as a byte slice (zero-copy).
    #[inline]
    pub fn data(&self) -> &[u8] {
        self.pool.data_slice(self.descriptor.tensor())
    }

    /// Get raw depth data as a mutable byte slice (zero-copy).
    #[inline]
    #[allow(clippy::mut_from_ref)]
    pub fn data_mut(&self) -> &mut [u8] {
        self.pool.data_slice_mut(self.descriptor.tensor())
    }

    /// Get depth at pixel (x, y) as f32 meters.
    ///
    /// For U16 data, converts from millimeters using depth_scale.
    /// Returns `None` if out of bounds.
    pub fn get_depth(&self, x: u32, y: u32) -> Option<f32> {
        if x >= self.width() || y >= self.height() {
            return None;
        }
        let idx = (y * self.width() + x) as usize;

        if self.descriptor.is_meters() {
            let data = self.data();
            let offset = idx * 4;
            if offset + 4 <= data.len() {
                let bytes = [data[offset], data[offset + 1], data[offset + 2], data[offset + 3]];
                Some(f32::from_le_bytes(bytes))
            } else {
                None
            }
        } else if self.descriptor.is_millimeters() {
            let data = self.data();
            let offset = idx * 2;
            if offset + 2 <= data.len() {
                let mm = u16::from_le_bytes([data[offset], data[offset + 1]]);
                Some(mm as f32 * self.descriptor.depth_scale() / 1000.0)
            } else {
                None
            }
        } else {
            None
        }
    }

    /// Get raw depth at pixel (x, y) as u16 (millimeters).
    ///
    /// Only valid for U16 dtype. Returns `None` for F32 data or out of bounds.
    pub fn get_depth_u16(&self, x: u32, y: u32) -> Option<u16> {
        if x >= self.width() || y >= self.height() || !self.descriptor.is_millimeters() {
            return None;
        }
        let idx = (y * self.width() + x) as usize;
        let data = self.data();
        let offset = idx * 2;
        if offset + 2 <= data.len() {
            Some(u16::from_le_bytes([data[offset], data[offset + 1]]))
        } else {
            None
        }
    }

    /// Set depth at pixel (x, y) in the raw format.
    ///
    /// For F32: value is in meters. For U16: value is in raw units.
    pub fn set_depth_f32(&self, x: u32, y: u32, value: f32) -> bool {
        if x >= self.width() || y >= self.height() {
            return false;
        }
        let idx = (y * self.width() + x) as usize;

        if self.descriptor.is_meters() {
            let data = self.data_mut();
            let offset = idx * 4;
            if offset + 4 <= data.len() {
                let bytes = value.to_le_bytes();
                data[offset..offset + 4].copy_from_slice(&bytes);
                return true;
            }
        } else if self.descriptor.is_millimeters() {
            let mm = (value * 1000.0 / self.descriptor.depth_scale()) as u16;
            let data = self.data_mut();
            let offset = idx * 2;
            if offset + 2 <= data.len() {
                data[offset..offset + 2].copy_from_slice(&mm.to_le_bytes());
                return true;
            }
        }
        false
    }

    /// Calculate depth statistics: (min, max, mean) in meters.
    ///
    /// Skips invalid depths (0 for U16, NaN/Inf for F32).
    pub fn depth_statistics(&self) -> (f32, f32, f32) {
        let w = self.width();
        let h = self.height();
        let mut min = f32::MAX;
        let mut max = f32::MIN;
        let mut sum = 0.0f64;
        let mut count = 0u64;

        for y in 0..h {
            for x in 0..w {
                if let Some(d) = self.get_depth(x, y) {
                    if d.is_finite() && d > 0.0 {
                        if d < min { min = d; }
                        if d > max { max = d; }
                        sum += d as f64;
                        count += 1;
                    }
                }
            }
        }

        if count == 0 {
            return (0.0, 0.0, 0.0);
        }
        (min, max, (sum / count as f64) as f32)
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

    /// Data type of depth values.
    #[inline]
    pub fn dtype(&self) -> TensorDtype {
        self.descriptor.dtype()
    }

    /// Whether depth values are in meters (F32).
    #[inline]
    pub fn is_meters(&self) -> bool {
        self.descriptor.is_meters()
    }

    /// Whether depth values are in millimeters (U16).
    #[inline]
    pub fn is_millimeters(&self) -> bool {
        self.descriptor.is_millimeters()
    }

    /// Depth scale.
    #[inline]
    pub fn depth_scale(&self) -> f32 {
        self.descriptor.depth_scale()
    }

    /// Total bytes of depth data.
    #[inline]
    pub fn nbytes(&self) -> u64 {
        self.descriptor.nbytes()
    }

    /// Whether data is on CPU.
    #[inline]
    pub fn is_cpu(&self) -> bool {
        self.descriptor.is_cpu()
    }

    /// Whether data is on CUDA.
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
    pub fn descriptor(&self) -> &DepthImage {
        &self.descriptor
    }

    /// Get a mutable reference to the descriptor.
    #[inline]
    pub fn descriptor_mut(&mut self) -> &mut DepthImage {
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

impl Clone for DepthImageHandle {
    fn clone(&self) -> Self {
        self.pool.retain(self.descriptor.tensor());
        Self {
            descriptor: self.descriptor,
            pool: Arc::clone(&self.pool),
        }
    }
}

impl Drop for DepthImageHandle {
    fn drop(&mut self) {
        self.pool.release(self.descriptor.tensor());
    }
}

impl std::fmt::Debug for DepthImageHandle {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let unit = if self.is_meters() { "m" } else if self.is_millimeters() { "mm" } else { "?" };
        f.debug_struct("DepthImageHandle")
            .field("width", &self.width())
            .field("height", &self.height())
            .field("unit", &unit)
            .field("refcount", &self.refcount())
            .finish()
    }
}

unsafe impl Send for DepthImageHandle {}
unsafe impl Sync for DepthImageHandle {}
