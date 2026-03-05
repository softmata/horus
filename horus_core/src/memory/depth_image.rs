//! User-facing DepthImage type with zero-copy shared memory backing
//!
//! `DepthImage` is the primary type for depth sensor data in HORUS.
//! Supports both F32 (meters) and U16 (millimeters) formats.
//!
//! # Example
//!
//! ```rust,ignore
//! use horus::prelude::*;
//!
//! // Create a 480x640 depth image (float32 meters)
//! let mut depth = DepthImage::new(480, 640, TensorDtype::F32)?;
//! depth.set_depth(100, 200, 2.5)?; // 2.5 meters
//!
//! // Send via topic (zero-copy)
//! let topic: Topic<DepthImage> = Topic::new("depth/raw")?;
//! topic.send(&depth);
//!
//! // Receive and query
//! if let Some(received) = topic.recv() {
//!     let d = received.get_depth(100, 200); // Some(2.5)
//!     let stats = received.depth_statistics(); // (min, max, mean)
//! }
//! ```

use std::sync::Arc;

use crate::types::{DepthImageDescriptor, Device, TensorDtype};

use super::tensor_pool::TensorPool;
use crate::communication::topic::pool_registry::global_pool;
use crate::error::{HorusError, HorusResult};

/// Depth image with zero-copy shared memory backing.
///
/// Create with `DepthImage::new()`, access depth values with `get_depth()`,
/// and send through topics with `topic.send(&depth)`.
pub struct DepthImage {
    descriptor: DepthImageDescriptor,
    pool: Arc<TensorPool>,
}

// Shared methods: data access, lifecycle, metadata delegation
crate::impl_tensor_backed!(DepthImage, DepthImageDescriptor, "depth image");

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

    // === Depth-specific methods ===

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
                let bytes = [
                    data[offset],
                    data[offset + 1],
                    data[offset + 2],
                    data[offset + 3],
                ];
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
    /// Returns `Ok(&mut Self)` for method chaining on success.
    ///
    /// # Errors
    ///
    /// - [`HorusError::OutOfRange`] — for U16 images, if the converted
    ///   millimeter value does not fit in `u16` (e.g., value > 65.535m at
    ///   default scale 1.0). Callers must handle this instead of silently
    ///   storing the saturated `u16::MAX`.
    pub fn set_depth(&mut self, x: u32, y: u32, value: f32) -> HorusResult<&mut Self> {
        if x >= self.width() || y >= self.height() {
            return Ok(self);
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
            // Convert meters → raw millimeter units, then range-check before cast.
            // `as u16` would silently saturate; we must reject out-of-range values
            // so callers can distinguish a valid 65535mm reading from an overflow.
            let mm_f32 = value * 1000.0 / self.descriptor.depth_scale();
            if !mm_f32.is_finite() || mm_f32 < 0.0 || mm_f32 > u16::MAX as f32 {
                return Err(HorusError::OutOfRange(format!(
                    "Depth {:.4}m maps to {:.1}mm, which is outside u16 range [0, 65535]",
                    value, mm_f32
                )));
            }
            let mm = mm_f32 as u16;
            let data = self.data_mut();
            let offset = idx * 2;
            if offset + 2 <= data.len() {
                data[offset..offset + 2].copy_from_slice(&mm.to_le_bytes());
            }
        }
        Ok(self)
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

    /// Calculate depth statistics: `Some((min, max, mean))` in meters.
    ///
    /// Returns `None` when the image contains no valid depth pixels (all
    /// pixels are 0 for U16, or NaN/Inf/0 for F32).  This distinguishes
    /// "no data" from a real all-zero depth reading, avoiding the previous
    /// ambiguity of returning `(0.0, 0.0, 0.0)` for both cases.
    pub fn depth_statistics(&self) -> Option<(f32, f32, f32)> {
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
            return None;
        }
        Some((min, max, (sum / count as f64) as f32))
    }

    // === DepthImage-specific metadata accessors ===

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

#[cfg(test)]
mod tests {
    use super::*;
    use crate::error::HorusError;
    use crate::types::TensorDtype;

    #[test]
    fn test_set_depth_u16_overflow_returns_err() {
        // 70 meters → 70_000mm, which exceeds u16::MAX (65535).
        // Before this fix, the cast `as u16` would silently wrap to ~4464.
        let mut depth = DepthImage::new(4, 4, TensorDtype::U16).unwrap();
        let result = depth.set_depth(0, 0, 70.0);
        assert!(
            matches!(result, Err(HorusError::OutOfRange(_))),
            "Expected OutOfRange for 70m on U16 depth image, got {:?}",
            result
        );
    }

    #[test]
    fn test_set_depth_u16_in_range_succeeds() {
        // 10 meters → 10_000mm, well within u16 range.
        let mut depth = DepthImage::new(4, 4, TensorDtype::U16).unwrap();
        depth.set_depth(0, 0, 10.0).unwrap();
        // Round-trip: get_depth reads back from raw mm.
        let v = depth.get_depth(0, 0).expect("should have value");
        assert!((v - 10.0).abs() < 0.001, "Expected ~10.0m, got {}", v);
    }

    #[test]
    fn test_set_depth_f32_no_overflow_check() {
        // F32 images accept any finite value — no u16 overflow check applies.
        let mut depth = DepthImage::new(4, 4, TensorDtype::F32).unwrap();
        depth.set_depth(0, 0, 1000.0).unwrap();
        let v = depth.get_depth(0, 0).expect("should have value");
        assert!((v - 1000.0).abs() < 0.001, "Expected 1000.0m, got {}", v);
    }

    #[test]
    fn test_set_depth_out_of_bounds_returns_ok() {
        // OOB is a silent no-op (returns Ok but doesn't write).
        let mut depth = DepthImage::new(2, 2, TensorDtype::U16).unwrap();
        let result = depth.set_depth(99, 99, 1.0);
        assert!(result.is_ok(), "OOB should return Ok, not Err");
    }

    #[test]
    fn test_set_depth_u16_negative_returns_err() {
        let mut depth = DepthImage::new(2, 2, TensorDtype::U16).unwrap();
        let result = depth.set_depth(0, 0, -1.0);
        assert!(
            matches!(result, Err(HorusError::OutOfRange(_))),
            "Negative depth should return OutOfRange, got {:?}",
            result
        );
    }

    /// depth_statistics() must return None when no valid pixels exist.
    ///
    /// Before this change the function returned (0.0, 0.0, 0.0) for all-zero
    /// images, which was ambiguous with a real all-zero depth reading.
    #[test]
    fn test_depth_statistics_none_for_empty() {
        // New zero-initialized image: all pixels are 0 (invalid/no-data).
        let depth = DepthImage::new(4, 4, TensorDtype::F32).unwrap();
        assert_eq!(
            depth.depth_statistics(),
            None,
            "all-zero F32 image must return None, not Some((0,0,0))"
        );

        let depth_u16 = DepthImage::new(4, 4, TensorDtype::U16).unwrap();
        assert_eq!(
            depth_u16.depth_statistics(),
            None,
            "all-zero U16 image must return None"
        );
    }

    /// depth_statistics() returns Some with correct (min, max, mean) for valid data.
    #[test]
    fn test_depth_statistics_some_with_data() {
        let mut depth = DepthImage::new(4, 4, TensorDtype::F32).unwrap();
        depth.set_depth(0, 0, 1.0).unwrap();
        depth.set_depth(1, 0, 2.0).unwrap();
        depth.set_depth(2, 0, 3.0).unwrap();

        let stats = depth.depth_statistics();
        assert!(stats.is_some(), "should have statistics with valid pixels");
        let (min, max, mean) = stats.unwrap();
        assert!((min - 1.0).abs() < 1e-4, "min={}", min);
        assert!((max - 3.0).abs() < 1e-4, "max={}", max);
        assert!((mean - 2.0).abs() < 1e-4, "mean={}", mean);
    }
}
