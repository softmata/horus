//! Pod cost map descriptor for zero-copy ring buffer transport
//!
//! `CostMapDescriptor` is a fixed-size (440 byte) `repr(C)` descriptor that
//! flows through the ring buffer via the ~50ns Pod path. It contains TWO
//! tensor references: one for occupancy grid data (i8) and one for cost
//! layer data (u8). Both data arrays live in a `TensorPool`.
//!
//! Users should use `CostMap` from `horus_core` which wraps this with
//! data access and navigation methods.

use bytemuck::{Pod, Zeroable};
use serde::{Deserialize, Serialize};

use super::tensor::Tensor;

/// Cost map descriptor — Pod, 440 bytes.
///
/// Contains TWO `Tensor` references plus navigation metadata. The grid tensor
/// holds occupancy cells (i8, shape `[H, W]`) and the cost tensor holds
/// computed costs (u8, shape `[H, W]`). Both data arrays stay in shared memory;
/// only this descriptor flows through the ring buffer.
///
/// # Layout (440 bytes, repr(C))
///
/// ```text
/// grid_tensor:         Tensor  (168 bytes) — occupancy data (i8)
/// cost_tensor:         Tensor  (168 bytes) — cost layer data (u8)
/// timestamp_ns:        u64          (8 bytes)
/// resolution:          f32          (4 bytes) — meters per cell
/// width:               u32          (4 bytes)
/// height:              u32          (4 bytes)
/// _pad1:               u32          (4 bytes) — alignment
/// origin_x:            f64          (8 bytes)
/// origin_y:            f64          (8 bytes)
/// origin_theta:        f64          (8 bytes)
/// inflation_radius:    f32          (4 bytes) — meters
/// cost_scaling_factor: f32          (4 bytes)
/// lethal_cost:         u8           (1 byte)
/// _pad2:               [u8; 3]      (3 bytes)
/// frame_id:            [u8; 32]     (32 bytes)
/// _reserved:           [u8; 8]      (8 bytes)
/// Total:                             440 bytes
/// ```
#[repr(C)]
#[derive(Clone, Copy, Debug, Serialize, Deserialize)]
pub struct CostMapDescriptor {
    /// Occupancy grid tensor: shape [H, W], dtype I8, data in pool
    grid_tensor: Tensor,
    /// Cost layer tensor: shape [H, W], dtype U8, data in pool
    cost_tensor: Tensor,
    /// Timestamp in nanoseconds since epoch
    timestamp_ns: u64,
    /// Map resolution in meters per cell
    resolution: f32,
    /// Grid width in cells
    width: u32,
    /// Grid height in cells
    height: u32,
    /// Alignment padding
    #[serde(skip)]
    _pad1: u32,
    /// Map origin X position in meters
    origin_x: f64,
    /// Map origin Y position in meters
    origin_y: f64,
    /// Map origin orientation in radians
    origin_theta: f64,
    /// Inflation radius in meters
    inflation_radius: f32,
    /// Cost scaling factor for inflation
    cost_scaling_factor: f32,
    /// Lethal cost threshold (cells at or above this are impassable)
    lethal_cost: u8,
    /// Alignment padding
    #[serde(skip)]
    _pad2: [u8; 3],
    /// Coordinate frame reference (null-terminated)
    frame_id: [u8; 32],
    /// Reserved for future use
    #[serde(skip)]
    _reserved: [u8; 8],
}

// Safety: CostMapDescriptor is repr(C), all fields are Pod, no implicit padding.
// 168 + 168 + 8 + 4 + 4 + 4 + 4 + 8 + 8 + 8 + 4 + 4 + 1 + 3 + 32 + 8 = 440 bytes.
// 440 % 8 = 0.
unsafe impl Zeroable for CostMapDescriptor {}
unsafe impl Pod for CostMapDescriptor {}

impl Default for CostMapDescriptor {
    fn default() -> Self {
        Self {
            grid_tensor: Tensor::default(),
            cost_tensor: Tensor::default(),
            timestamp_ns: 0,
            resolution: 0.05,
            width: 0,
            height: 0,
            _pad1: 0,
            origin_x: 0.0,
            origin_y: 0.0,
            origin_theta: 0.0,
            inflation_radius: 0.3,
            cost_scaling_factor: 10.0,
            lethal_cost: 254,
            _pad2: [0; 3],
            frame_id: [0; 32],
            _reserved: [0; 8],
        }
    }
}

impl CostMapDescriptor {
    /// Sanitize a descriptor read from untrusted bytes (SHM, network, file).
    #[inline]
    pub fn sanitize_from_shm(&mut self) {
        self.grid_tensor.sanitize_from_shm();
        self.cost_tensor.sanitize_from_shm();
        if !self.resolution.is_finite() || self.resolution <= 0.0 {
            self.resolution = 0.05;
        }
        if !self.inflation_radius.is_finite() || self.inflation_radius < 0.0 {
            self.inflation_radius = 0.3;
        }
        if !self.cost_scaling_factor.is_finite() || self.cost_scaling_factor <= 0.0 {
            self.cost_scaling_factor = 10.0;
        }
    }

    /// Create a new cost map descriptor from pre-built tensors + metadata.
    pub fn new(
        grid_tensor: Tensor,
        cost_tensor: Tensor,
        resolution: f32,
        width: u32,
        height: u32,
    ) -> Self {
        Self {
            grid_tensor,
            cost_tensor,
            timestamp_ns: 0,
            resolution,
            width,
            height,
            _pad1: 0,
            origin_x: 0.0,
            origin_y: 0.0,
            origin_theta: 0.0,
            inflation_radius: 0.3,
            cost_scaling_factor: 10.0,
            lethal_cost: 254,
            _pad2: [0; 3],
            frame_id: [0; 32],
            _reserved: [0; 8],
        }
    }

    // === Tensor accessors ===

    /// Get the occupancy grid tensor descriptor.
    #[inline]
    pub fn grid_tensor(&self) -> &Tensor {
        &self.grid_tensor
    }

    /// Get the cost layer tensor descriptor.
    #[inline]
    pub fn cost_tensor(&self) -> &Tensor {
        &self.cost_tensor
    }

    // === Grid metadata ===

    /// Map resolution in meters per cell.
    #[inline]
    pub fn resolution(&self) -> f32 {
        self.resolution
    }

    /// Grid width in cells.
    #[inline]
    pub fn width(&self) -> u32 {
        self.width
    }

    /// Grid height in cells.
    #[inline]
    pub fn height(&self) -> u32 {
        self.height
    }

    /// Total number of cells.
    #[inline]
    pub fn cell_count(&self) -> usize {
        self.width as usize * self.height as usize
    }

    // === Origin ===

    /// Map origin X position in meters.
    #[inline]
    pub fn origin_x(&self) -> f64 {
        self.origin_x
    }

    /// Map origin Y position in meters.
    #[inline]
    pub fn origin_y(&self) -> f64 {
        self.origin_y
    }

    /// Map origin orientation in radians.
    #[inline]
    pub fn origin_theta(&self) -> f64 {
        self.origin_theta
    }

    /// Set map origin.
    pub fn set_origin(&mut self, x: f64, y: f64, theta: f64) {
        self.origin_x = x;
        self.origin_y = y;
        self.origin_theta = theta;
    }

    // === Cost map parameters ===

    /// Inflation radius in meters.
    #[inline]
    pub fn inflation_radius(&self) -> f32 {
        self.inflation_radius
    }

    /// Set inflation radius.
    pub fn set_inflation_radius(&mut self, radius: f32) {
        self.inflation_radius = radius;
    }

    /// Cost scaling factor.
    #[inline]
    pub fn cost_scaling_factor(&self) -> f32 {
        self.cost_scaling_factor
    }

    /// Set cost scaling factor.
    pub fn set_cost_scaling_factor(&mut self, factor: f32) {
        self.cost_scaling_factor = factor;
    }

    /// Lethal cost threshold.
    #[inline]
    pub fn lethal_cost(&self) -> u8 {
        self.lethal_cost
    }

    /// Set lethal cost threshold.
    pub fn set_lethal_cost(&mut self, cost: u8) {
        self.lethal_cost = cost;
    }

    // === Timestamp & frame_id (manual, since we don't have a single `inner` tensor) ===

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
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::types::{Device, TensorDtype};

    #[test]
    fn test_costmap_size_and_alignment() {
        let size = std::mem::size_of::<CostMapDescriptor>();
        let align = std::mem::align_of::<CostMapDescriptor>();
        assert_eq!(size, 440, "CostMapDescriptor must be exactly 440 bytes");
        assert_eq!(align, 8, "alignment must be 8 from u64/f64 fields");
        assert_eq!(size % align, 0, "size must be a multiple of alignment");
    }

    #[test]
    fn test_costmap_pod() {
        let grid_t = Tensor::new(1, 10, 1, 0, &[500, 500], TensorDtype::I8, Device::cpu());
        let cost_t = Tensor::new(1, 11, 2, 0, &[500, 500], TensorDtype::U8, Device::cpu());
        let mut cm = CostMapDescriptor::new(grid_t, cost_t, 0.05, 500, 500);
        cm.set_frame_id("map");
        cm.set_timestamp_ns(999);
        cm.set_origin(1.0, 2.0, 0.5);

        let bytes: &[u8] = bytemuck::bytes_of(&cm);
        assert_eq!(bytes.len(), 440);
        let recovered: &CostMapDescriptor = bytemuck::from_bytes(bytes);
        assert_eq!(recovered.width(), 500);
        assert_eq!(recovered.height(), 500);
        assert_eq!(recovered.resolution(), 0.05);
        assert_eq!(recovered.frame_id(), "map");
        assert_eq!(recovered.timestamp_ns(), 999);
        assert_eq!(recovered.origin_x(), 1.0);
        assert_eq!(recovered.origin_y(), 2.0);
        assert_eq!(recovered.inflation_radius(), 0.3);
        assert_eq!(recovered.lethal_cost(), 254);
        assert_eq!(recovered.grid_tensor().slot_id, 10);
        assert_eq!(recovered.cost_tensor().slot_id, 11);
    }

    #[test]
    fn test_costmap_default() {
        let cm = CostMapDescriptor::default();
        assert_eq!(cm.width(), 0);
        assert_eq!(cm.height(), 0);
        assert_eq!(cm.resolution(), 0.05);
        assert_eq!(cm.inflation_radius(), 0.3);
        assert_eq!(cm.cost_scaling_factor(), 10.0);
        assert_eq!(cm.lethal_cost(), 254);
    }

    #[test]
    fn test_costmap_parameters() {
        let mut cm = CostMapDescriptor::default();
        cm.set_inflation_radius(0.5);
        cm.set_cost_scaling_factor(5.0);
        cm.set_lethal_cost(200);
        assert_eq!(cm.inflation_radius(), 0.5);
        assert_eq!(cm.cost_scaling_factor(), 5.0);
        assert_eq!(cm.lethal_cost(), 200);
    }

    #[test]
    fn test_costmap_sanitize() {
        let mut cm = CostMapDescriptor::default();
        cm.resolution = f32::NAN;
        cm.inflation_radius = -1.0;
        cm.cost_scaling_factor = 0.0;
        cm.sanitize_from_shm();
        assert_eq!(cm.resolution(), 0.05);
        assert_eq!(cm.inflation_radius(), 0.3);
        assert_eq!(cm.cost_scaling_factor(), 10.0);
    }

    #[test]
    fn test_costmap_dual_tensor() {
        let grid_t = Tensor::new(1, 0, 0, 0, &[100, 100], TensorDtype::I8, Device::cpu());
        let cost_t = Tensor::new(1, 1, 0, 0, &[100, 100], TensorDtype::U8, Device::cpu());
        let cm = CostMapDescriptor::new(grid_t, cost_t, 0.1, 100, 100);

        // Grid tensor is I8
        assert_eq!(cm.grid_tensor().dtype, TensorDtype::I8);
        assert_eq!(cm.grid_tensor().shape(), &[100, 100]);

        // Cost tensor is U8
        assert_eq!(cm.cost_tensor().dtype, TensorDtype::U8);
        assert_eq!(cm.cost_tensor().shape(), &[100, 100]);

        // They have different slot IDs
        assert_ne!(cm.grid_tensor().slot_id, cm.cost_tensor().slot_id);
    }

    #[test]
    fn test_costmap_serde_roundtrip() {
        let grid_t = Tensor::new(1, 5, 1, 0, &[200, 200], TensorDtype::I8, Device::cpu());
        let cost_t = Tensor::new(1, 6, 2, 0, &[200, 200], TensorDtype::U8, Device::cpu());
        let mut cm = CostMapDescriptor::new(grid_t, cost_t, 0.1, 200, 200);
        cm.set_frame_id("odom");
        cm.set_timestamp_ns(42);

        let json = serde_json::to_string(&cm).unwrap();
        let recovered: CostMapDescriptor = serde_json::from_str(&json).unwrap();
        assert_eq!(recovered.width(), 200);
        assert_eq!(recovered.height(), 200);
        assert_eq!(recovered.frame_id(), "odom");
        assert_eq!(recovered.timestamp_ns(), 42);
    }
}
