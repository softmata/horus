//! User-facing CostMap type with zero-copy shared memory backing
//!
//! `CostMap` wraps a dual-tensor Pod descriptor + `Arc<TensorPool>` and provides
//! a rich API for cost-based navigation. It contains TWO shared memory tensors:
//! one for occupancy data (i8) and one for cost layer data (u8).
//!
//! Clone increments both tensors' refcounts; Drop releases both.
//!
//! # Example
//!
//! ```rust,ignore
//! use horus::prelude::*;
//!
//! let mut costmap = CostMap::new(500, 500, 0.05, 0.3)?;
//! costmap.set_occupancy(100, 100, 100);  // Mark occupied
//! costmap.set_cost(100, 100, 254);       // Lethal cost
//!
//! // Send via topic (zero-copy — only the 440B descriptor travels)
//! let topic: Topic<CostMap> = Topic::new("costmap")?;
//! topic.send(&costmap);
//! ```

use std::sync::Arc;

use crate::types::{CostMapDescriptor, TensorDtype};

use super::tensor_pool::TensorPool;
use crate::communication::topic::pool_registry::global_pool;
use crate::error::HorusResult;

/// Cost map with zero-copy shared memory backing.
///
/// Contains two data layers:
/// - **Occupancy grid** (i8): -1 = unknown, 0 = free, 100 = occupied
/// - **Cost layer** (u8): 0 = free, 254 = lethal, 255 = unknown
///
/// When cloned, both tensors' refcounts are incremented. When dropped,
/// both are decremented. When counts reach zero, memory slots are freed.
pub struct CostMap {
    descriptor: CostMapDescriptor,
    pool: Arc<TensorPool>,
}

// Manual Clone — must retain BOTH tensors
impl Clone for CostMap {
    fn clone(&self) -> Self {
        self.pool.retain(self.descriptor.grid_tensor());
        self.pool.retain(self.descriptor.cost_tensor());
        Self {
            descriptor: self.descriptor,
            pool: Arc::clone(&self.pool),
        }
    }
}

// Manual Drop — must release BOTH tensors
impl Drop for CostMap {
    fn drop(&mut self) {
        self.pool.release(self.descriptor.grid_tensor());
        self.pool.release(self.descriptor.cost_tensor());
    }
}

// Safety: The underlying pool uses atomic operations for all shared state.
unsafe impl Send for CostMap {}
unsafe impl Sync for CostMap {}

impl CostMap {
    /// Create a new cost map with the given dimensions and parameters.
    ///
    /// Allocates TWO shared memory tensors from the global pool:
    /// - Occupancy grid (i8, initialized to -1 = unknown)
    /// - Cost layer (u8, initialized to 0 = free)
    pub fn new(
        width: u32,
        height: u32,
        resolution: f32,
        inflation_radius: f32,
    ) -> HorusResult<Self> {
        Self::new_on(width, height, resolution, inflation_radius, global_pool())
    }

    /// Create a cost map backed by a specific pool.
    pub fn new_on(
        width: u32,
        height: u32,
        resolution: f32,
        inflation_radius: f32,
        pool: Arc<TensorPool>,
    ) -> HorusResult<Self> {
        let device = pool.backend_device();
        let shape = vec![height as u64, width as u64];

        let grid_tensor = pool.alloc(&shape, TensorDtype::I8, device)?;
        let cost_tensor = pool.alloc(&shape, TensorDtype::U8, device)?;

        let mut descriptor =
            CostMapDescriptor::new(grid_tensor, cost_tensor, resolution, width, height);
        descriptor.set_inflation_radius(inflation_radius);

        let costmap = Self { descriptor, pool };

        // Initialize occupancy to -1 (unknown)
        costmap.grid_data_mut().fill(0xFF); // -1 as u8
        // Initialize costs to 0 (free)
        costmap.cost_data_mut().fill(0);

        Ok(costmap)
    }

    /// Create from a descriptor and pool (used by Topic recv).
    #[doc(hidden)]
    pub fn from_owned(descriptor: CostMapDescriptor, pool: Arc<TensorPool>) -> Self {
        Self { descriptor, pool }
    }

    // === Descriptor & pool access ===

    /// Get the underlying descriptor.
    #[doc(hidden)]
    #[inline]
    pub fn descriptor(&self) -> &CostMapDescriptor {
        &self.descriptor
    }

    /// Get the pool reference.
    #[doc(hidden)]
    #[inline]
    pub fn pool(&self) -> &Arc<TensorPool> {
        &self.pool
    }

    // === Raw data access (zero-copy from shared memory) ===

    /// Get occupancy grid data as raw bytes (zero-copy).
    #[inline]
    pub fn grid_data(&self) -> &[u8] {
        self.pool
            .data_slice(self.descriptor.grid_tensor())
            .expect("grid tensor descriptor must be valid")
    }

    /// Get occupancy grid data as mutable raw bytes.
    #[inline]
    #[allow(clippy::mut_from_ref)]
    pub fn grid_data_mut(&self) -> &mut [u8] {
        self.pool
            .data_slice_mut(self.descriptor.grid_tensor())
            .expect("grid tensor descriptor must be valid")
    }

    /// Get cost layer data as raw bytes (zero-copy).
    #[inline]
    pub fn cost_data(&self) -> &[u8] {
        self.pool
            .data_slice(self.descriptor.cost_tensor())
            .expect("cost tensor descriptor must be valid")
    }

    /// Get cost layer data as mutable raw bytes.
    #[inline]
    #[allow(clippy::mut_from_ref)]
    pub fn cost_data_mut(&self) -> &mut [u8] {
        self.pool
            .data_slice_mut(self.descriptor.cost_tensor())
            .expect("cost tensor descriptor must be valid")
    }

    // === Typed data access ===

    /// Get occupancy cells as an i8 slice (zero-copy).
    #[inline]
    pub fn occupancy_cells(&self) -> &[i8] {
        let bytes = self.grid_data();
        // SAFETY: i8 has alignment 1, same size as u8
        unsafe { std::slice::from_raw_parts(bytes.as_ptr() as *const i8, bytes.len()) }
    }

    /// Get occupancy cells as a mutable i8 slice.
    // SAFETY: &self receiver is intentional — the mutable slice is obtained from
    // shared-memory pointer arithmetic, not from &mut self. Interior mutability
    // via raw pointers into SHM is the design contract.
    #[allow(clippy::mut_from_ref)]
    #[inline]
    pub fn occupancy_cells_mut(&self) -> &mut [i8] {
        let bytes = self.grid_data_mut();
        // SAFETY: i8 has alignment 1, same size as u8
        unsafe { std::slice::from_raw_parts_mut(bytes.as_mut_ptr() as *mut i8, bytes.len()) }
    }

    /// Get cost layer as a u8 slice (zero-copy).
    #[inline]
    pub fn cost_cells(&self) -> &[u8] {
        self.cost_data()
    }

    /// Get cost layer as a mutable u8 slice.
    #[inline]
    pub fn cost_cells_mut(&self) -> &mut [u8] {
        self.cost_data_mut()
    }

    // === Grid metadata ===

    /// Map resolution in meters per cell.
    #[inline]
    pub fn resolution(&self) -> f32 {
        self.descriptor.resolution()
    }

    /// Grid width in cells.
    #[inline]
    pub fn width(&self) -> u32 {
        self.descriptor.width()
    }

    /// Grid height in cells.
    #[inline]
    pub fn height(&self) -> u32 {
        self.descriptor.height()
    }

    /// Total number of cells.
    #[inline]
    pub fn cell_count(&self) -> usize {
        self.descriptor.cell_count()
    }

    // === Origin ===

    /// Map origin X position in meters.
    #[inline]
    pub fn origin_x(&self) -> f64 {
        self.descriptor.origin_x()
    }

    /// Map origin Y position in meters.
    #[inline]
    pub fn origin_y(&self) -> f64 {
        self.descriptor.origin_y()
    }

    /// Map origin orientation in radians.
    #[inline]
    pub fn origin_theta(&self) -> f64 {
        self.descriptor.origin_theta()
    }

    /// Set map origin.
    pub fn set_origin(&mut self, x: f64, y: f64, theta: f64) -> &mut Self {
        self.descriptor.set_origin(x, y, theta);
        self
    }

    // === Cost map parameters ===

    /// Inflation radius in meters.
    #[inline]
    pub fn inflation_radius(&self) -> f32 {
        self.descriptor.inflation_radius()
    }

    /// Cost scaling factor.
    #[inline]
    pub fn cost_scaling_factor(&self) -> f32 {
        self.descriptor.cost_scaling_factor()
    }

    /// Lethal cost threshold.
    #[inline]
    pub fn lethal_cost(&self) -> u8 {
        self.descriptor.lethal_cost()
    }

    // === Timestamp & frame_id ===

    /// Timestamp in nanoseconds.
    #[inline]
    pub fn timestamp_ns(&self) -> u64 {
        self.descriptor.timestamp_ns()
    }

    /// Set timestamp.
    pub fn set_timestamp_ns(&mut self, ts: u64) -> &mut Self {
        self.descriptor.set_timestamp_ns(ts);
        self
    }

    /// Frame ID.
    #[inline]
    pub fn frame_id(&self) -> &str {
        self.descriptor.frame_id()
    }

    /// Set frame ID.
    pub fn set_frame_id(&mut self, id: &str) -> &mut Self {
        self.descriptor.set_frame_id(id);
        self
    }

    // === Cell access ===

    /// Get occupancy value at grid coordinates.
    pub fn get_occupancy(&self, grid_x: u32, grid_y: u32) -> Option<i8> {
        if grid_x < self.width() && grid_y < self.height() {
            let index = (grid_y * self.width() + grid_x) as usize;
            self.occupancy_cells().get(index).copied()
        } else {
            None
        }
    }

    /// Set occupancy value at grid coordinates (clamped to [-1, 100]).
    pub fn set_occupancy(&mut self, grid_x: u32, grid_y: u32, value: i8) -> bool {
        if grid_x < self.width() && grid_y < self.height() {
            let index = (grid_y * self.width() + grid_x) as usize;
            let cells = self.occupancy_cells_mut();
            if index < cells.len() {
                cells[index] = value.clamp(-1, 100);
                return true;
            }
        }
        false
    }

    /// Get cost value at grid coordinates.
    pub fn get_cost(&self, grid_x: u32, grid_y: u32) -> Option<u8> {
        if grid_x < self.width() && grid_y < self.height() {
            let index = (grid_y * self.width() + grid_x) as usize;
            self.cost_cells().get(index).copied()
        } else {
            None
        }
    }

    /// Set cost value at grid coordinates.
    pub fn set_cost(&mut self, grid_x: u32, grid_y: u32, value: u8) -> bool {
        if grid_x < self.width() && grid_y < self.height() {
            let index = (grid_y * self.width() + grid_x) as usize;
            let cells = self.cost_cells_mut();
            if index < cells.len() {
                cells[index] = value;
                return true;
            }
        }
        false
    }

    // === Convenience queries ===

    /// Check if a cell is lethal (cost >= lethal_cost threshold).
    pub fn is_lethal(&self, grid_x: u32, grid_y: u32) -> bool {
        self.get_cost(grid_x, grid_y)
            .is_some_and(|c| c >= self.lethal_cost())
    }

    /// Check if a cell is traversable (cost < lethal_cost and occupancy known).
    pub fn is_traversable(&self, grid_x: u32, grid_y: u32) -> bool {
        let cost_ok = self
            .get_cost(grid_x, grid_y)
            .is_some_and(|c| c < self.lethal_cost());
        let occ_ok = self.get_occupancy(grid_x, grid_y).is_some_and(|o| o >= 0);
        cost_ok && occ_ok
    }

    // === Coordinate transforms ===

    /// Convert world coordinates to grid indices.
    pub fn world_to_grid(&self, x: f64, y: f64) -> Option<(u32, u32)> {
        const EPSILON: f64 = 1e-6;
        let res = (self.resolution() as f64).max(1e-9);
        let grid_x = ((x - self.origin_x()) / res + EPSILON).floor() as i32;
        let grid_y = ((y - self.origin_y()) / res + EPSILON).floor() as i32;

        if grid_x >= 0
            && grid_x < self.width() as i32
            && grid_y >= 0
            && grid_y < self.height() as i32
        {
            Some((grid_x as u32, grid_y as u32))
        } else {
            None
        }
    }

    /// Convert grid indices to world coordinates (cell center).
    pub fn grid_to_world(&self, grid_x: u32, grid_y: u32) -> Option<(f64, f64)> {
        if grid_x < self.width() && grid_y < self.height() {
            let x = self.origin_x() + (grid_x as f64 + 0.5) * self.resolution() as f64;
            let y = self.origin_y() + (grid_y as f64 + 0.5) * self.resolution() as f64;
            Some((x, y))
        } else {
            None
        }
    }

    // === Bulk operations ===

    /// Clear the cost layer (set all costs to 0 = free).
    pub fn clear_costs(&mut self) -> &mut Self {
        self.cost_cells_mut().fill(0);
        self
    }

    /// Reset occupancy grid (set all cells to -1 = unknown).
    pub fn reset_occupancy(&mut self) -> &mut Self {
        self.grid_data_mut().fill(0xFF); // -1 as u8
        self
    }

    /// Copy occupied cells from occupancy grid to cost layer.
    ///
    /// Cells with occupancy >= 50 get lethal cost, others get 0.
    pub fn apply_from_occupancy(&mut self) -> &mut Self {
        let lethal = self.lethal_cost();
        let occ = self.occupancy_cells();
        let len = occ.len();
        // Read occupancy into temp buffer to avoid aliasing
        let occ_copy: Vec<i8> = occ[..len].to_vec();
        let costs = self.cost_cells_mut();
        for i in 0..len.min(costs.len()) {
            costs[i] = if occ_copy[i] >= 50 { lethal } else { 0 };
        }
        self
    }
}

impl std::fmt::Debug for CostMap {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("CostMap")
            .field("width", &self.width())
            .field("height", &self.height())
            .field("resolution", &self.resolution())
            .field("inflation_radius", &self.inflation_radius())
            .field("lethal_cost", &self.lethal_cost())
            .finish()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_costmap_new() {
        let cm = CostMap::new(100, 200, 0.05, 0.3).expect("alloc");
        assert_eq!(cm.width(), 100);
        assert_eq!(cm.height(), 200);
        assert_eq!(cm.resolution(), 0.05);
        assert_eq!(cm.inflation_radius(), 0.3);
        assert_eq!(cm.cell_count(), 20_000);
        // Occupancy initialized to -1 (unknown)
        assert_eq!(cm.get_occupancy(0, 0), Some(-1));
        // Cost initialized to 0 (free)
        assert_eq!(cm.get_cost(0, 0), Some(0));
    }

    #[test]
    fn test_costmap_set_get() {
        let mut cm = CostMap::new(10, 10, 0.1, 0.3).expect("alloc");
        assert!(cm.set_occupancy(5, 5, 100));
        assert_eq!(cm.get_occupancy(5, 5), Some(100));

        assert!(cm.set_cost(5, 5, 254));
        assert_eq!(cm.get_cost(5, 5), Some(254));

        // Out of bounds
        assert_eq!(cm.get_occupancy(10, 10), None);
        assert_eq!(cm.get_cost(10, 10), None);
        assert!(!cm.set_occupancy(10, 10, 50));
        assert!(!cm.set_cost(10, 10, 50));
    }

    #[test]
    fn test_costmap_dual_tensor_independence() {
        let mut cm = CostMap::new(5, 5, 0.1, 0.3).expect("alloc");
        // Set occupancy at (2,2)
        cm.set_occupancy(2, 2, 100);
        // Cost at (2,2) should still be 0
        assert_eq!(cm.get_cost(2, 2), Some(0));

        // Set cost at (3,3)
        cm.set_cost(3, 3, 200);
        // Occupancy at (3,3) should still be -1
        assert_eq!(cm.get_occupancy(3, 3), Some(-1));
    }

    #[test]
    fn test_costmap_clone_drop() {
        let mut cm = CostMap::new(10, 10, 0.1, 0.3).expect("alloc");
        cm.set_occupancy(0, 0, 50);
        cm.set_cost(0, 0, 128);

        let clone = cm.clone();
        assert_eq!(clone.get_occupancy(0, 0), Some(50));
        assert_eq!(clone.get_cost(0, 0), Some(128));
        drop(clone);

        // Original still valid after clone dropped
        assert_eq!(cm.get_occupancy(0, 0), Some(50));
        assert_eq!(cm.get_cost(0, 0), Some(128));
    }

    #[test]
    fn test_costmap_is_lethal() {
        let mut cm = CostMap::new(10, 10, 0.1, 0.3).expect("alloc");
        cm.set_cost(5, 5, 254); // lethal (default threshold = 254)
        cm.set_cost(6, 6, 100); // not lethal

        assert!(cm.is_lethal(5, 5));
        assert!(!cm.is_lethal(6, 6));
    }

    #[test]
    fn test_costmap_is_traversable() {
        let mut cm = CostMap::new(10, 10, 0.1, 0.3).expect("alloc");
        // Cell (0,0): occupancy=-1 (unknown), cost=0 → NOT traversable (unknown occ)
        assert!(!cm.is_traversable(0, 0));

        // Cell (1,1): occupancy=0 (free), cost=0 → traversable
        cm.set_occupancy(1, 1, 0);
        assert!(cm.is_traversable(1, 1));

        // Cell (2,2): occupancy=0 (free), cost=254 (lethal) → NOT traversable
        cm.set_occupancy(2, 2, 0);
        cm.set_cost(2, 2, 254);
        assert!(!cm.is_traversable(2, 2));
    }

    #[test]
    fn test_costmap_apply_from_occupancy() {
        let mut cm = CostMap::new(5, 5, 0.1, 0.3).expect("alloc");
        cm.set_occupancy(0, 0, 0); // free
        cm.set_occupancy(1, 0, 100); // occupied
        cm.set_occupancy(2, 0, 50); // threshold

        cm.apply_from_occupancy();

        assert_eq!(cm.get_cost(0, 0), Some(0)); // free → 0
        assert_eq!(cm.get_cost(1, 0), Some(254)); // occupied → lethal
        assert_eq!(cm.get_cost(2, 0), Some(254)); // 50 >= 50 → lethal
    }

    #[test]
    fn test_costmap_clear_reset() {
        let mut cm = CostMap::new(5, 5, 0.1, 0.3).expect("alloc");
        cm.set_cost(2, 2, 200);
        cm.set_occupancy(2, 2, 100);

        cm.clear_costs();
        assert_eq!(cm.get_cost(2, 2), Some(0));
        assert_eq!(cm.get_occupancy(2, 2), Some(100)); // unchanged

        cm.reset_occupancy();
        assert_eq!(cm.get_occupancy(2, 2), Some(-1));
    }

    #[test]
    fn test_costmap_world_to_grid() {
        let mut cm = CostMap::new(10, 10, 0.1, 0.3).expect("alloc");
        cm.set_origin(0.0, 0.0, 0.0);

        assert_eq!(cm.world_to_grid(0.05, 0.05), Some((0, 0)));
        assert_eq!(cm.world_to_grid(0.95, 0.95), Some((9, 9)));
        assert_eq!(cm.world_to_grid(-0.1, 0.0), None);
    }

    #[test]
    fn test_costmap_frame_id_timestamp() {
        let mut cm = CostMap::new(5, 5, 0.1, 0.3).expect("alloc");
        cm.set_frame_id("map");
        cm.set_timestamp_ns(999);
        assert_eq!(cm.frame_id(), "map");
        assert_eq!(cm.timestamp_ns(), 999);
    }

    #[test]
    fn test_costmap_debug() {
        let cm = CostMap::new(100, 200, 0.05, 0.3).expect("alloc");
        let dbg = format!("{:?}", cm);
        assert!(dbg.contains("CostMap"));
        assert!(dbg.contains("100"));
    }
}
