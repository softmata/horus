//! User-facing OccupancyGrid type with zero-copy shared memory backing
//!
//! `OccupancyGrid` wraps a Pod descriptor + `Arc<TensorPool>` and provides
//! a rich API for grid cell access and coordinate transforms. Clone increments
//! the refcount; Drop releases it.
//!
//! Grid cells are stored as `i8` values: -1 = unknown, 0 = free, 100 = occupied.
//!
//! # Example
//!
//! ```rust,ignore
//! use horus::prelude::*;
//!
//! // Create a 1000x1000 grid at 5cm resolution (shared memory backed)
//! let mut grid = OccupancyGrid::new(1000, 1000, 0.05)?;
//! grid.set_cell(500, 500, 100);  // Mark as occupied
//!
//! // Send via topic (zero-copy — only the 320B descriptor travels)
//! let topic: Topic<OccupancyGrid> = Topic::new("map.occupancy")?;
//! topic.send(&grid);
//! ```

use std::sync::Arc;

use crate::types::{OccupancyGridDescriptor, TensorDtype};

use super::tensor_pool::TensorPool;
use crate::communication::topic::pool_registry::global_pool;
use crate::error::HorusResult;

/// Occupancy grid with zero-copy shared memory backing.
///
/// Grid cells are `i8` values: -1 = unknown, 0 = free, 100 = occupied.
///
/// When cloned, the tensor's refcount is incremented. When dropped,
/// it is decremented. When the count reaches zero, the memory slot
/// is returned to the pool.
pub struct OccupancyGrid {
    descriptor: OccupancyGridDescriptor,
    pool: Arc<TensorPool>,
}

// Shared methods: data access, lifecycle, metadata delegation
crate::impl_tensor_backed!(OccupancyGrid, OccupancyGridDescriptor, "occupancy grid");

impl OccupancyGrid {
    /// Create a new occupancy grid with the given dimensions and resolution.
    ///
    /// Allocates shared memory from a global pool. Cells are initialized to -1 (unknown).
    pub fn new(width: u32, height: u32, resolution: f32) -> HorusResult<Self> {
        Self::new_on(width, height, resolution, global_pool())
    }

    /// Create an occupancy grid backed by a specific pool.
    pub fn new_on(
        width: u32,
        height: u32,
        resolution: f32,
        pool: Arc<TensorPool>,
    ) -> HorusResult<Self> {
        let device = pool.backend_device();
        let shape = vec![height as u64, width as u64];
        let tensor = pool.alloc(&shape, TensorDtype::I8, device)?;
        let descriptor = OccupancyGridDescriptor::new(tensor, resolution, width, height);

        let grid = Self { descriptor, pool };

        // Initialize all cells to -1 (unknown)
        let data = grid.data_mut();
        // SAFETY: i8 and u8 have the same size, -1i8 == 0xFF as u8
        data.fill(0xFF);

        Ok(grid)
    }

    // === Grid metadata accessors ===

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

    // === Origin accessors ===

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

    /// Set map origin (X, Y, theta).
    pub fn set_origin(&mut self, x: f64, y: f64, theta: f64) -> &mut Self {
        self.descriptor.set_origin(x, y, theta);
        self
    }

    // === Metadata ===

    /// Get user metadata bytes.
    #[inline]
    pub fn metadata(&self) -> &[u8; 64] {
        self.descriptor.metadata()
    }

    /// Set user metadata bytes.
    pub fn set_metadata(&mut self, data: &[u8]) -> &mut Self {
        self.descriptor.set_metadata(data);
        self
    }

    // === Cell access ===

    /// Get the raw grid cells as an i8 slice (zero-copy from shared memory).
    #[inline]
    pub fn cells(&self) -> &[i8] {
        // SAFETY: dtype is I8, alignment is always valid for i8 (align=1)
        unsafe { self.data_as::<i8>() }
    }

    /// Get the raw grid cells as a mutable i8 slice.
    // SAFETY: &self receiver is intentional — the mutable slice is obtained from
    // shared-memory pointer arithmetic, not from &mut self. Interior mutability
    // via raw pointers into SHM is the design contract.
    #[allow(clippy::mut_from_ref)]
    #[inline]
    pub fn cells_mut(&self) -> &mut [i8] {
        // SAFETY: dtype is I8, alignment is always valid for i8 (align=1)
        unsafe { self.data_as_mut::<i8>() }
    }

    /// Get occupancy value at grid coordinates.
    ///
    /// Returns `None` if coordinates are out of bounds.
    pub fn get_cell(&self, grid_x: u32, grid_y: u32) -> Option<i8> {
        if grid_x < self.width() && grid_y < self.height() {
            let index = (grid_y * self.width() + grid_x) as usize;
            self.cells().get(index).copied()
        } else {
            None
        }
    }

    /// Set occupancy value at grid coordinates.
    ///
    /// Values are clamped to the valid range [-1, 100].
    /// Returns `true` if the cell was set, `false` if out of bounds.
    pub fn set_cell(&mut self, grid_x: u32, grid_y: u32, value: i8) -> bool {
        if grid_x < self.width() && grid_y < self.height() {
            let index = (grid_y * self.width() + grid_x) as usize;
            let cells = self.cells_mut();
            if index < cells.len() {
                cells[index] = value.clamp(-1, 100);
                return true;
            }
        }
        false
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

    // === Convenience queries ===

    /// Check if a world-coordinate point is free (occupancy 0..50).
    pub fn is_free(&self, x: f64, y: f64) -> bool {
        if let Some((gx, gy)) = self.world_to_grid(x, y) {
            if let Some(occ) = self.get_cell(gx, gy) {
                return (0..50).contains(&occ);
            }
        }
        false
    }

    /// Check if a world-coordinate point is occupied (occupancy >= 50).
    pub fn is_occupied(&self, x: f64, y: f64) -> bool {
        if let Some((gx, gy)) = self.world_to_grid(x, y) {
            if let Some(occ) = self.get_cell(gx, gy) {
                return occ >= 50;
            }
        }
        false
    }

    /// Check if a cell is unknown (value == -1).
    pub fn is_unknown(&self, grid_x: u32, grid_y: u32) -> bool {
        self.get_cell(grid_x, grid_y) == Some(-1)
    }

    /// Fill the entire grid with a single value.
    pub fn fill(&mut self, value: i8) -> &mut Self {
        let cells = self.cells_mut();
        cells.fill(value);
        self
    }

    /// Clear the grid (set all cells to 0 = free).
    pub fn clear(&mut self) -> &mut Self {
        self.fill(0)
    }

    /// Reset the grid (set all cells to -1 = unknown).
    pub fn reset(&mut self) -> &mut Self {
        self.fill(-1)
    }

    /// Get one row of the grid as a slice (zero-copy).
    pub fn row(&self, y: u32) -> Option<&[i8]> {
        if y < self.height() {
            let w = self.width() as usize;
            let start = y as usize * w;
            let cells = self.cells();
            cells.get(start..start + w)
        } else {
            None
        }
    }
}

impl std::fmt::Debug for OccupancyGrid {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("OccupancyGrid")
            .field("width", &self.width())
            .field("height", &self.height())
            .field("resolution", &self.resolution())
            .finish()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_occupancy_grid_new() {
        let grid = OccupancyGrid::new(100, 200, 0.05).expect("alloc");
        assert_eq!(grid.width(), 100);
        assert_eq!(grid.height(), 200);
        assert_eq!(grid.resolution(), 0.05);
        assert_eq!(grid.cell_count(), 20_000);
        // All cells initialized to -1 (unknown)
        assert_eq!(grid.get_cell(0, 0), Some(-1));
        assert_eq!(grid.get_cell(99, 199), Some(-1));
    }

    #[test]
    fn test_occupancy_grid_set_get_cell() {
        let mut grid = OccupancyGrid::new(10, 10, 0.1).expect("alloc");
        assert!(grid.set_cell(5, 5, 100));
        assert_eq!(grid.get_cell(5, 5), Some(100));

        // Out of bounds
        assert_eq!(grid.get_cell(10, 10), None);
        assert!(!grid.set_cell(10, 10, 50));

        // Clamp to valid range
        assert!(grid.set_cell(0, 0, 127));
        assert_eq!(grid.get_cell(0, 0), Some(100)); // clamped to 100
    }

    #[test]
    fn test_occupancy_grid_world_to_grid() {
        let mut grid = OccupancyGrid::new(10, 10, 0.1).expect("alloc");
        grid.set_origin(0.0, 0.0, 0.0);

        assert_eq!(grid.world_to_grid(0.05, 0.05), Some((0, 0)));
        assert_eq!(grid.world_to_grid(0.95, 0.95), Some((9, 9)));
        assert_eq!(grid.world_to_grid(-0.1, 0.0), None); // out of bounds
        assert_eq!(grid.world_to_grid(1.0, 0.0), None); // just past edge
    }

    #[test]
    fn test_occupancy_grid_grid_to_world() {
        let grid = OccupancyGrid::new(10, 10, 0.1).expect("alloc");
        let (x, y) = grid.grid_to_world(0, 0).unwrap();
        assert!((x - 0.05).abs() < 1e-9);
        assert!((y - 0.05).abs() < 1e-9);
        assert_eq!(grid.grid_to_world(10, 10), None);
    }

    #[test]
    fn test_occupancy_grid_clone_drops() {
        let grid = OccupancyGrid::new(10, 10, 0.1).expect("alloc");
        let clone = grid.clone();
        assert_eq!(clone.width(), 10);
        assert_eq!(clone.get_cell(0, 0), Some(-1));
        drop(clone);
        // Original still valid
        assert_eq!(grid.get_cell(0, 0), Some(-1));
    }

    #[test]
    fn test_occupancy_grid_fill_clear_reset() {
        let mut grid = OccupancyGrid::new(5, 5, 0.1).expect("alloc");
        grid.fill(50);
        assert_eq!(grid.get_cell(2, 2), Some(50));

        grid.clear();
        assert_eq!(grid.get_cell(2, 2), Some(0));

        grid.reset();
        assert_eq!(grid.get_cell(2, 2), Some(-1));
    }

    #[test]
    fn test_occupancy_grid_row() {
        let mut grid = OccupancyGrid::new(5, 3, 0.1).expect("alloc");
        grid.clear();
        grid.set_cell(2, 1, 100);
        let row = grid.row(1).unwrap();
        assert_eq!(row.len(), 5);
        assert_eq!(row[2], 100);
        assert_eq!(row[0], 0);
        assert_eq!(grid.row(3), None);
    }

    #[test]
    fn test_occupancy_grid_is_queries() {
        let mut grid = OccupancyGrid::new(10, 10, 0.1).expect("alloc");
        grid.set_origin(0.0, 0.0, 0.0);
        grid.set_cell(0, 0, 0); // free
        grid.set_cell(1, 0, 100); // occupied
                                  // Cell (2, 0) is -1 (unknown)

        assert!(grid.is_free(0.05, 0.05));
        assert!(grid.is_occupied(0.15, 0.05));
        assert!(!grid.is_free(0.25, 0.05)); // unknown is not free
        assert!(grid.is_unknown(2, 0));
    }

    #[test]
    fn test_occupancy_grid_origin() {
        let mut grid = OccupancyGrid::new(10, 10, 0.1).expect("alloc");
        grid.set_origin(5.0, 10.0, 1.57);
        assert_eq!(grid.origin_x(), 5.0);
        assert_eq!(grid.origin_y(), 10.0);
        assert_eq!(grid.origin_theta(), 1.57);
    }

    #[test]
    fn test_occupancy_grid_metadata() {
        let mut grid = OccupancyGrid::new(10, 10, 0.1).expect("alloc");
        grid.set_metadata(b"test_meta");
        assert_eq!(&grid.metadata()[..9], b"test_meta");
    }

    #[test]
    fn test_occupancy_grid_frame_id() {
        let mut grid = OccupancyGrid::new(10, 10, 0.1).expect("alloc");
        grid.set_frame_id("map");
        assert_eq!(grid.frame_id(), "map");
    }

    #[test]
    fn test_occupancy_grid_debug() {
        let grid = OccupancyGrid::new(100, 200, 0.05).expect("alloc");
        let dbg = format!("{:?}", grid);
        assert!(dbg.contains("OccupancyGrid"));
        assert!(dbg.contains("100"));
        assert!(dbg.contains("200"));
    }
}
