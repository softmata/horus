//! Integration tests for OccupancyGrid and CostMap tensor-backed operations.
//!
//! Run with:
//!   cargo test --no-default-features -p horus_core --test grid_tensor_ops

mod common;
use common::cleanup_stale_shm;

use horus_core::memory::{CostMap, OccupancyGrid};

// ============================================================
// OccupancyGrid tests
// ============================================================

#[test]
fn test_occupancy_grid_create_and_dimensions() {
    cleanup_stale_shm();

    let grid = OccupancyGrid::new(200, 150, 0.05).expect("alloc");
    assert_eq!(grid.width(), 200);
    assert_eq!(grid.height(), 150);
    assert_eq!(grid.resolution(), 0.05);
    assert_eq!(grid.cell_count(), 200 * 150);

    // All cells start as -1 (unknown)
    for y in 0..150 {
        for x in 0..200 {
            assert_eq!(
                grid.get_cell(x, y),
                Some(-1),
                "cell ({x},{y}) should be -1 (unknown) after creation"
            );
        }
    }
}

#[test]
fn test_occupancy_grid_set_get_cell() {
    cleanup_stale_shm();

    let mut grid = OccupancyGrid::new(50, 40, 0.1).expect("alloc");

    // Set various cells and read back
    assert!(grid.set_cell(10, 20, 75));
    assert_eq!(grid.get_cell(10, 20), Some(75));

    assert!(grid.set_cell(0, 0, 0));
    assert_eq!(grid.get_cell(0, 0), Some(0));

    // Boundary cell: (width-1, height-1)
    assert!(grid.set_cell(49, 39, 100));
    assert_eq!(grid.get_cell(49, 39), Some(100));

    // Another boundary: (0, height-1)
    assert!(grid.set_cell(0, 39, 42));
    assert_eq!(grid.get_cell(0, 39), Some(42));

    // Boundary: (width-1, 0)
    assert!(grid.set_cell(49, 0, 33));
    assert_eq!(grid.get_cell(49, 0), Some(33));

    // Out-of-bounds reads return None
    assert_eq!(grid.get_cell(50, 0), None);
    assert_eq!(grid.get_cell(0, 40), None);
    assert_eq!(grid.get_cell(50, 40), None);

    // Out-of-bounds writes return false
    assert!(!grid.set_cell(50, 0, 10));
    assert!(!grid.set_cell(0, 40, 10));

    // Clamping: values above 100 are clamped to 100
    assert!(grid.set_cell(5, 5, 127));
    assert_eq!(grid.get_cell(5, 5), Some(100));

    // Clamping: values below -1 are clamped to -1
    assert!(grid.set_cell(6, 6, -128));
    assert_eq!(grid.get_cell(6, 6), Some(-1));
}

#[test]
fn test_occupancy_grid_world_to_grid_roundtrip() {
    cleanup_stale_shm();

    let mut grid = OccupancyGrid::new(100, 100, 0.1).expect("alloc");
    // Set origin at (5.0, 10.0)
    grid.set_origin(5.0, 10.0, 0.0);

    // Mark cell (25, 30) as occupied
    grid.set_cell(25, 30, 100);

    // The world coordinates of cell (25, 30) center should be:
    //   x = 5.0 + (25 + 0.5) * 0.1 = 5.0 + 2.55 = 7.55
    //   y = 10.0 + (30 + 0.5) * 0.1 = 10.0 + 3.05 = 13.05
    let (wx, wy) = grid.grid_to_world(25, 30).expect("valid cell");
    assert!((wx - 7.55).abs() < 1e-6, "world x: expected ~7.55, got {wx}");
    assert!(
        (wy - 13.05).abs() < 1e-6,
        "world y: expected ~13.05, got {wy}"
    );

    // Convert those world coords back to grid coords
    let (gx, gy) = grid.world_to_grid(wx, wy).expect("should map back");
    assert_eq!(gx, 25, "roundtrip grid_x");
    assert_eq!(gy, 30, "roundtrip grid_y");

    // Verify the cell value at the roundtripped coordinates
    assert_eq!(grid.get_cell(gx, gy), Some(100));

    // Out-of-bounds world coordinates return None
    assert_eq!(grid.world_to_grid(4.0, 10.0), None, "below origin_x");
    assert_eq!(grid.world_to_grid(5.0, 9.0), None, "below origin_y");
    assert_eq!(
        grid.world_to_grid(15.1, 10.0),
        None,
        "past grid width (5.0 + 100*0.1 = 15.0)"
    );
}

#[test]
fn test_occupancy_grid_free_occupied_unknown() {
    cleanup_stale_shm();

    let mut grid = OccupancyGrid::new(10, 10, 0.1).expect("alloc");
    grid.set_origin(0.0, 0.0, 0.0);

    // Set cell (0,0) = -1 (unknown) — already default, but explicit
    grid.set_cell(0, 0, -1);
    // Set cell (1,0) = 0 (free)
    grid.set_cell(1, 0, 0);
    // Set cell (2,0) = 100 (fully occupied)
    grid.set_cell(2, 0, 100);
    // Set cell (3,0) = 49 (below occupied threshold, still free)
    grid.set_cell(3, 0, 49);
    // Set cell (4,0) = 50 (at occupied threshold)
    grid.set_cell(4, 0, 50);

    // is_unknown checks by grid coordinates
    assert!(grid.is_unknown(0, 0), "cell -1 should be unknown");
    assert!(!grid.is_unknown(1, 0), "cell 0 should not be unknown");
    assert!(!grid.is_unknown(2, 0), "cell 100 should not be unknown");

    // is_free and is_occupied check by world coordinates
    // Cell (0,0) center = (0.05, 0.05), cell (1,0) center = (0.15, 0.05), etc.

    // Unknown cell: neither free nor occupied
    assert!(
        !grid.is_free(0.05, 0.05),
        "unknown cell should not be free"
    );
    assert!(
        !grid.is_occupied(0.05, 0.05),
        "unknown cell (-1) should not be occupied (threshold is >= 50)"
    );

    // Free cell (value 0): is_free range is 0..50
    assert!(grid.is_free(0.15, 0.05), "cell 0 should be free");
    assert!(
        !grid.is_occupied(0.15, 0.05),
        "cell 0 should not be occupied"
    );

    // Occupied cell (value 100): is_occupied threshold is >= 50
    assert!(
        !grid.is_free(0.25, 0.05),
        "cell 100 should not be free"
    );
    assert!(
        grid.is_occupied(0.25, 0.05),
        "cell 100 should be occupied"
    );

    // Cell value 49: within free range (0..50)
    assert!(grid.is_free(0.35, 0.05), "cell 49 should be free");
    assert!(
        !grid.is_occupied(0.35, 0.05),
        "cell 49 should not be occupied"
    );

    // Cell value 50: at occupied threshold (>= 50)
    assert!(
        !grid.is_free(0.45, 0.05),
        "cell 50 should not be free (free range is 0..50)"
    );
    assert!(
        grid.is_occupied(0.45, 0.05),
        "cell 50 should be occupied"
    );
}

#[test]
fn test_occupancy_grid_fill_and_reset() {
    cleanup_stale_shm();

    let mut grid = OccupancyGrid::new(20, 15, 0.1).expect("alloc");

    // Fill with value 50
    grid.fill(50);
    for y in 0..15 {
        for x in 0..20 {
            assert_eq!(
                grid.get_cell(x, y),
                Some(50),
                "after fill(50), cell ({x},{y}) should be 50"
            );
        }
    }

    // Reset sets all cells to -1 (unknown)
    grid.reset();
    for y in 0..15 {
        for x in 0..20 {
            assert_eq!(
                grid.get_cell(x, y),
                Some(-1),
                "after reset(), cell ({x},{y}) should be -1"
            );
        }
    }

    // Clear sets all cells to 0 (free)
    grid.clear();
    for y in 0..15 {
        for x in 0..20 {
            assert_eq!(
                grid.get_cell(x, y),
                Some(0),
                "after clear(), cell ({x},{y}) should be 0"
            );
        }
    }
}

// ============================================================
// CostMap tests
// ============================================================

#[test]
fn test_costmap_create_and_dual_layers() {
    cleanup_stale_shm();

    let mut costmap = CostMap::new(30, 25, 0.05, 0.3).expect("alloc");
    assert_eq!(costmap.width(), 30);
    assert_eq!(costmap.height(), 25);
    assert_eq!(costmap.resolution(), 0.05);
    assert_eq!(costmap.inflation_radius(), 0.3);
    assert_eq!(costmap.cell_count(), 30 * 25);

    // Default state: occupancy = -1, cost = 0
    assert_eq!(costmap.get_occupancy(0, 0), Some(-1));
    assert_eq!(costmap.get_cost(0, 0), Some(0));

    // Set occupancy at (5, 10) — cost should remain unaffected
    assert!(costmap.set_occupancy(5, 10, 100));
    assert_eq!(costmap.get_occupancy(5, 10), Some(100));
    assert_eq!(
        costmap.get_cost(5, 10),
        Some(0),
        "setting occupancy must not alter cost layer"
    );

    // Set cost at (15, 20) — occupancy should remain unaffected
    assert!(costmap.set_cost(15, 20, 254));
    assert_eq!(costmap.get_cost(15, 20), Some(254));
    assert_eq!(
        costmap.get_occupancy(15, 20),
        Some(-1),
        "setting cost must not alter occupancy layer"
    );

    // Set both layers at the same cell
    assert!(costmap.set_occupancy(3, 3, 50));
    assert!(costmap.set_cost(3, 3, 128));
    assert_eq!(costmap.get_occupancy(3, 3), Some(50));
    assert_eq!(costmap.get_cost(3, 3), Some(128));

    // Verify other cells are untouched
    assert_eq!(costmap.get_occupancy(0, 0), Some(-1));
    assert_eq!(costmap.get_cost(0, 0), Some(0));
}

#[test]
fn test_costmap_clone_drops_correctly() {
    cleanup_stale_shm();

    let mut costmap = CostMap::new(10, 10, 0.1, 0.5).expect("alloc");
    costmap.set_occupancy(3, 4, 80);
    costmap.set_cost(3, 4, 200);
    costmap.set_occupancy(7, 8, 0);
    costmap.set_cost(7, 8, 42);

    // Clone retains both tensors
    let mut clone = costmap.clone();
    assert_eq!(clone.width(), 10);
    assert_eq!(clone.height(), 10);
    assert_eq!(clone.resolution(), 0.1);
    assert_eq!(clone.inflation_radius(), 0.5);
    assert_eq!(clone.get_occupancy(3, 4), Some(80));
    assert_eq!(clone.get_cost(3, 4), Some(200));
    assert_eq!(clone.get_occupancy(7, 8), Some(0));
    assert_eq!(clone.get_cost(7, 8), Some(42));

    // Drop the original
    drop(costmap);

    // Clone must still have valid data (dual-tensor retain/release works)
    assert_eq!(
        clone.get_occupancy(3, 4),
        Some(80),
        "occupancy must survive after original dropped"
    );
    assert_eq!(
        clone.get_cost(3, 4),
        Some(200),
        "cost must survive after original dropped"
    );
    assert_eq!(clone.get_occupancy(7, 8), Some(0));
    assert_eq!(clone.get_cost(7, 8), Some(42));

    // Can still write to the clone after original is dropped
    assert!(clone.set_occupancy(0, 0, 10));
    assert_eq!(clone.get_occupancy(0, 0), Some(10));
    assert!(clone.set_cost(0, 0, 55));
    assert_eq!(clone.get_cost(0, 0), Some(55));
}

#[test]
fn test_occupancy_grid_clone_data_independent() {
    cleanup_stale_shm();

    let mut grid = OccupancyGrid::new(10, 10, 0.1).expect("alloc");
    grid.set_cell(5, 5, 75);

    // Clone the grid — clone shares the same underlying shared memory
    // (OccupancyGrid uses refcounted shared memory, not deep copy)
    let mut clone = grid.clone();

    // Clone sees the same data as the original
    assert_eq!(clone.get_cell(5, 5), Some(75));
    assert_eq!(clone.width(), grid.width());
    assert_eq!(clone.height(), grid.height());
    assert_eq!(clone.resolution(), grid.resolution());

    // Drop the original — clone must survive
    drop(grid);

    // Clone is still valid and readable after original is dropped
    assert_eq!(
        clone.get_cell(5, 5),
        Some(75),
        "clone data must survive after original is dropped"
    );

    // Clone can be written to after original is dropped
    assert!(clone.set_cell(0, 0, 42));
    assert_eq!(clone.get_cell(0, 0), Some(42));

    // All other cells should still be -1 (unknown)
    assert_eq!(clone.get_cell(1, 1), Some(-1));
    assert_eq!(clone.get_cell(9, 9), Some(-1));
}
