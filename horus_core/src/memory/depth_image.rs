//! User-facing DepthImage type with zero-copy shared memory backing
//!
//! `DepthImage` is the primary type for depth sensor data in HORUS.
//! Supports both F32 (meters) and U16 (millimeters) formats.

use std::sync::Arc;

use horus_types::{DepthImageDescriptor, Device, TensorDtype};

use super::tensor_pool::TensorPool;
use crate::communication::topic::pool_registry::global_pool;
use crate::error::HorusResult;

/// Depth image with zero-copy shared memory backing.
///
/// Create with `DepthImage::new()`, access depth values with `get_depth()`,
/// and send through topics with `topic.send(&depth)`.
pub struct DepthImage {
    descriptor: DepthImageDescriptor,
    pool: Arc<TensorPool>,
}

impl DepthImage {
    /// Create a new depth image with the given dimensions.
    ///
    /// Use `TensorDtype::F32` for meters or `TensorDtype::U16` for millimeters.
    ///
    /// # Example
    ///
    /// ```rust,ignore
    /// let depth = DepthImage::new(480, 640, TensorDtype::F32)?;
    /// assert!(depth.is_meters());
    /// ```
    pub fn new(height: u32, width: u32, dtype: TensorDtype) -> HorusResult<Self> {
        let pool = global_pool();
        let shape = [height as u64, width as u64];
        let tensor = pool.alloc(&shape, dtype, Device::cpu())?;
        let descriptor = DepthImageDescriptor::new(tensor);
        Ok(Self { descriptor, pool })
    }

    /// Create a DepthImage from a descriptor and pool (internal).
    pub(crate) fn from_owned(descriptor: DepthImageDescriptor, pool: Arc<TensorPool>) -> Self {
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

    /// Set depth at pixel (x, y).
    ///
    /// For F32: value is in meters. For U16: converts from meters to raw units.
    /// Returns `&mut Self` for method chaining.
    pub fn set_depth(&mut self, x: u32, y: u32, value: f32) -> &mut Self {
        if x >= self.width() || y >= self.height() {
            return self;
        }
        let idx = (y * self.width() + x) as usize;

        if self.descriptor.is_meters() {
            let data = self.data_mut();
            let offset = idx * 4;
            if offset + 4 <= data.len() {
                let bytes = value.to_le_bytes();
                data[offset..offset + 4].copy_from_slice(&bytes);
            }
        } else if self.descriptor.is_millimeters() {
            let mm = (value * 1000.0 / self.descriptor.depth_scale()) as u16;
            let data = self.data_mut();
            let offset = idx * 2;
            if offset + 2 <= data.len() {
                data[offset..offset + 2].copy_from_slice(&mm.to_le_bytes());
            }
        }
        self
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
                        if d < min {
                            min = d;
                        }
                        if d > max {
                            max = d;
                        }
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

    /// Set the frame ID.
    pub fn set_frame_id(&mut self, id: &str) -> &mut Self {
        self.descriptor.set_frame_id(id);
        self
    }

    /// Set the timestamp in nanoseconds.
    pub fn set_timestamp_ns(&mut self, ts: u64) -> &mut Self {
        self.descriptor.set_timestamp_ns(ts);
        self
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

    // === Internal accessors ===

    #[inline]
    pub(crate) fn descriptor(&self) -> &DepthImageDescriptor {
        &self.descriptor
    }

    #[inline]
    pub(crate) fn pool(&self) -> &Arc<TensorPool> {
        &self.pool
    }

}

impl Clone for DepthImage {
    fn clone(&self) -> Self {
        self.pool.retain(self.descriptor.tensor());
        Self {
            descriptor: self.descriptor,
            pool: Arc::clone(&self.pool),
        }
    }
}

impl Drop for DepthImage {
    fn drop(&mut self) {
        self.pool.release(self.descriptor.tensor());
    }
}

impl std::fmt::Debug for DepthImage {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let unit = if self.is_meters() {
            "m"
        } else if self.is_millimeters() {
            "mm"
        } else {
            "?"
        };
        f.debug_struct("DepthImage")
            .field("width", &self.width())
            .field("height", &self.height())
            .field("unit", &unit)
            .finish()
    }
}

unsafe impl Send for DepthImage {}
unsafe impl Sync for DepthImage {}
