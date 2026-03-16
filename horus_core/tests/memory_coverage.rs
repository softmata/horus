//! Coverage tests for memory and types functions that lack dedicated tests.
//!
//! Run with:
//!   cargo test --no-default-features -p horus_core --test memory_coverage

use horus_core::memory::tensor_pool::{TensorPool, TensorPoolConfig, TensorPoolStats};
use horus_core::memory::{DepthImage, Image, PointCloud};
use horus_core::types::{ImageEncoding, TensorDtype};

// ============================================================
// Image tests
// ============================================================

#[test]
fn image_pixel_returns_correct_bytes() {
    let mut img = Image::new(4, 4, ImageEncoding::Rgb8).expect("alloc");
    // Write a known pixel at (2, 1): red = [255, 0, 0]
    img.set_pixel(2, 1, &[255, 0, 0]);
    let px = img.pixel(2, 1).expect("in-bounds pixel should return Some");
    assert_eq!(px, &[255, 0, 0]);
}

#[test]
fn image_pixel_out_of_bounds_returns_none() {
    let img = Image::new(4, 4, ImageEncoding::Rgb8).expect("alloc");
    assert!(img.pixel(4, 0).is_none(), "x == width is OOB");
    assert!(img.pixel(0, 4).is_none(), "y == height is OOB");
    assert!(img.pixel(100, 100).is_none(), "far OOB");
}

#[test]
fn image_set_pixel_and_readback() {
    let mut img = Image::new(8, 8, ImageEncoding::Rgb8).expect("alloc");
    img.set_pixel(0, 0, &[10, 20, 30]);
    img.set_pixel(7, 7, &[40, 50, 60]);

    assert_eq!(img.pixel(0, 0).unwrap(), &[10, 20, 30]);
    assert_eq!(img.pixel(7, 7).unwrap(), &[40, 50, 60]);
}

#[test]
fn image_set_pixel_wrong_bpp_is_noop() {
    let mut img = Image::new(4, 4, ImageEncoding::Rgb8).expect("alloc");
    // Rgb8 has bpp=3; passing 2 bytes should be a silent no-op.
    img.set_pixel(0, 0, &[255, 255]);
    let px = img.pixel(0, 0).unwrap();
    // Data is zero-initialized, so pixel should still be [0, 0, 0].
    assert_eq!(px, &[0, 0, 0]);
}

#[test]
fn image_fill_sets_all_pixels() {
    let mut img = Image::new(4, 3, ImageEncoding::Rgb8).expect("alloc");
    img.fill(&[100, 150, 200]);

    for y in 0..3 {
        for x in 0..4 {
            let px = img.pixel(x, y).expect("pixel should be Some");
            assert_eq!(px, &[100, 150, 200], "mismatch at ({}, {})", x, y);
        }
    }
}

#[test]
fn image_fill_wrong_bpp_is_noop() {
    let mut img = Image::new(2, 2, ImageEncoding::Rgb8).expect("alloc");
    img.fill(&[1, 2]); // Wrong length — should be no-op.
    let px = img.pixel(0, 0).unwrap();
    assert_eq!(px, &[0, 0, 0], "fill with wrong bpp should leave data unchanged");
}

// ============================================================
// PointCloud tests
// ============================================================

#[test]
fn pointcloud_point_at_returns_correct_data() {
    let pc = PointCloud::new(5, 3, TensorDtype::F32).expect("alloc");
    // Write known values to point 2
    let floats: &mut [f32] = bytemuck::cast_slice_mut(pc.data_mut());
    // Point 2 is at indices 6, 7, 8
    floats[6] = 1.0;
    floats[7] = 2.0;
    floats[8] = 3.0;

    let raw = pc.point_at(2).expect("valid index");
    let point: &[f32] = bytemuck::cast_slice(raw);
    assert_eq!(point, &[1.0, 2.0, 3.0]);
}

#[test]
fn pointcloud_point_at_out_of_bounds_returns_none() {
    let pc = PointCloud::new(5, 3, TensorDtype::F32).expect("alloc");
    assert!(pc.point_at(5).is_none(), "idx == count is OOB");
    assert!(pc.point_at(100).is_none(), "far OOB");
}

#[test]
fn pointcloud_is_xyz_for_3_fields() {
    let pc = PointCloud::new(10, 3, TensorDtype::F32).expect("alloc");
    assert!(pc.is_xyz(), "3-field cloud should be XYZ");
}

#[test]
fn pointcloud_is_xyz_false_for_4_fields() {
    let pc = PointCloud::new(10, 4, TensorDtype::F32).expect("alloc");
    assert!(!pc.is_xyz(), "4-field cloud should not be plain XYZ");
}

#[test]
fn pointcloud_has_intensity_for_4_fields() {
    let pc = PointCloud::new(10, 4, TensorDtype::F32).expect("alloc");
    assert!(pc.has_intensity(), "4-field cloud should have intensity");
}

#[test]
fn pointcloud_has_intensity_false_for_3_fields() {
    let pc = PointCloud::new(10, 3, TensorDtype::F32).expect("alloc");
    assert!(
        !pc.has_intensity(),
        "3-field cloud should not have intensity"
    );
}

#[test]
fn pointcloud_has_color_for_6_fields() {
    let pc = PointCloud::new(10, 6, TensorDtype::F32).expect("alloc");
    assert!(pc.has_color(), "6-field cloud should have color");
}

#[test]
fn pointcloud_has_color_false_for_3_fields() {
    let pc = PointCloud::new(10, 3, TensorDtype::F32).expect("alloc");
    assert!(!pc.has_color(), "3-field cloud should not have color");
}

// ============================================================
// DepthImage tests
// ============================================================

#[test]
fn depth_image_get_depth_u16_roundtrip() {
    let mut depth = DepthImage::new(8, 8, TensorDtype::U16).expect("alloc");
    // set_depth takes meters; 5.0m → 5000mm (u16 raw value 5000)
    depth.set_depth(3, 4, 5.0).expect("in-range");
    let raw = depth.get_depth_u16(3, 4).expect("valid U16 pixel");
    assert_eq!(raw, 5000, "5.0m should map to 5000mm raw value");
}

#[test]
fn depth_image_get_depth_u16_zero_initialized() {
    let depth = DepthImage::new(4, 4, TensorDtype::U16).expect("alloc");
    let raw = depth.get_depth_u16(0, 0).expect("valid U16 pixel");
    assert_eq!(raw, 0, "zero-initialized depth should be 0");
}

#[test]
fn depth_image_get_depth_u16_returns_none_for_f32() {
    let depth = DepthImage::new(4, 4, TensorDtype::F32).expect("alloc");
    assert!(
        depth.get_depth_u16(0, 0).is_none(),
        "get_depth_u16 should return None for F32 images"
    );
}

#[test]
fn depth_image_get_depth_u16_oob_returns_none() {
    let depth = DepthImage::new(4, 4, TensorDtype::U16).expect("alloc");
    assert!(depth.get_depth_u16(4, 0).is_none());
    assert!(depth.get_depth_u16(0, 4).is_none());
}

// ============================================================
// Tensor::sanitize_from_shm tests
// ============================================================

#[test]
fn tensor_sanitize_clamps_invalid_ndim() {
    use horus_core::types::Tensor;

    let mut t = Tensor::default();
    t.ndim = 255; // Invalid: max is 8
    t.sanitize_from_shm();
    assert_eq!(t.ndim, 8, "ndim should be clamped to MAX_TENSOR_DIMS (8)");
}

#[test]
fn tensor_sanitize_preserves_valid_ndim() {
    use horus_core::types::Tensor;

    let mut t = Tensor::default();
    t.ndim = 3;
    t.sanitize_from_shm();
    assert_eq!(t.ndim, 3, "valid ndim should be unchanged");
}

#[test]
fn tensor_sanitize_clamps_invalid_dtype() {
    use horus_core::types::Tensor;

    let mut t = Tensor::default();
    // Set dtype to an invalid raw value by writing directly to the byte
    let dtype_ptr = &mut t.dtype as *mut TensorDtype as *mut u8;
    unsafe { *dtype_ptr = 200 }; // Invalid discriminant
    t.sanitize_from_shm();
    // from_raw falls back to F32 for invalid discriminants
    assert_eq!(t.dtype, TensorDtype::F32, "invalid dtype should fall back to F32");
}

#[test]
fn tensor_sanitize_preserves_valid_dtype() {
    use horus_core::types::Tensor;

    let mut t = Tensor::default();
    t.dtype = TensorDtype::U8;
    t.sanitize_from_shm();
    assert_eq!(t.dtype, TensorDtype::U8, "valid dtype should be preserved");
}

// ============================================================
// TensorPoolStats tests
// ============================================================

#[test]
fn pool_stats_slot_utilization_empty() {
    let stats = TensorPoolStats {
        pool_id: 1,
        pool_size: 1024,
        max_slots: 100,
        allocated_slots: 0,
        total_refcount: 0,
        used_bytes: 0,
        free_bytes: 1024,
    };
    assert!((stats.slot_utilization() - 0.0).abs() < 1e-9);
}

#[test]
fn pool_stats_slot_utilization_half() {
    let stats = TensorPoolStats {
        pool_id: 1,
        pool_size: 1024,
        max_slots: 100,
        allocated_slots: 50,
        total_refcount: 50,
        used_bytes: 512,
        free_bytes: 512,
    };
    assert!((stats.slot_utilization() - 0.5).abs() < 1e-9);
}

#[test]
fn pool_stats_slot_utilization_zero_max_slots() {
    let stats = TensorPoolStats {
        pool_id: 1,
        pool_size: 1024,
        max_slots: 0,
        allocated_slots: 0,
        total_refcount: 0,
        used_bytes: 0,
        free_bytes: 1024,
    };
    assert!((stats.slot_utilization() - 0.0).abs() < 1e-9);
}

#[test]
fn pool_stats_data_utilization() {
    let stats = TensorPoolStats {
        pool_id: 1,
        pool_size: 1000,
        max_slots: 10,
        allocated_slots: 5,
        total_refcount: 5,
        used_bytes: 250,
        free_bytes: 750,
    };
    assert!((stats.data_utilization() - 0.25).abs() < 1e-9);
}

#[test]
fn pool_stats_data_utilization_zero_pool_size() {
    let stats = TensorPoolStats {
        pool_id: 1,
        pool_size: 0,
        max_slots: 10,
        allocated_slots: 0,
        total_refcount: 0,
        used_bytes: 0,
        free_bytes: 0,
    };
    assert!((stats.data_utilization() - 0.0).abs() < 1e-9);
}

#[test]
fn pool_stats_is_under_pressure_false_when_low() {
    let stats = TensorPoolStats {
        pool_id: 1,
        pool_size: 1000,
        max_slots: 100,
        allocated_slots: 10,
        total_refcount: 10,
        used_bytes: 100,
        free_bytes: 900,
    };
    assert!(!stats.is_under_pressure());
}

#[test]
fn pool_stats_is_under_pressure_true_when_slots_high() {
    let stats = TensorPoolStats {
        pool_id: 1,
        pool_size: 1000,
        max_slots: 100,
        allocated_slots: 85, // 85% > 80%
        total_refcount: 85,
        used_bytes: 100,
        free_bytes: 900,
    };
    assert!(stats.is_under_pressure());
}

#[test]
fn pool_stats_is_under_pressure_true_when_data_high() {
    let stats = TensorPoolStats {
        pool_id: 1,
        pool_size: 1000,
        max_slots: 100,
        allocated_slots: 10,
        total_refcount: 10,
        used_bytes: 850, // 85% > 80%
        free_bytes: 150,
    };
    assert!(stats.is_under_pressure());
}

#[test]
fn pool_stats_summary_contains_key_info() {
    let stats = TensorPoolStats {
        pool_id: 42,
        pool_size: 1024 * 1024,
        max_slots: 64,
        allocated_slots: 16,
        total_refcount: 32,
        used_bytes: 512 * 1024,
        free_bytes: 512 * 1024,
    };
    let s = stats.summary();
    assert!(s.contains("42"), "summary should contain pool_id");
    assert!(s.contains("16"), "summary should contain allocated_slots");
    assert!(s.contains("64"), "summary should contain max_slots");
}

// ============================================================
// TensorPool::open test
// ============================================================

#[test]
fn tensor_pool_open_existing() {
    use horus_core::types::Device;

    let pool_id = 99990; // Use a unique ID to avoid collisions
    let config = TensorPoolConfig {
        pool_size: 64 * 1024,
        max_slots: 8,
        slot_alignment: 64,
        ..Default::default()
    };

    // Create the pool first
    let pool = TensorPool::new(pool_id, config).expect("create pool");

    // Allocate a tensor and write data
    let tensor = pool
        .alloc(&[4, 4], TensorDtype::F32, Device::cpu())
        .expect("alloc");
    let data = pool.data_slice_mut(&tensor).expect("data access");
    data[0..4].copy_from_slice(&42.0f32.to_le_bytes());

    // Open the same pool from another handle
    let pool2 = TensorPool::open(pool_id).expect("open existing pool");
    let data2 = pool2.data_slice(&tensor).expect("data access via opened pool");
    let val = f32::from_le_bytes([data2[0], data2[1], data2[2], data2[3]]);
    assert!(
        (val - 42.0).abs() < 1e-9,
        "opened pool should see data written by creator"
    );

    // Cleanup: remove the SHM file
    let shm_path = pool.shm_path().to_path_buf();
    drop(pool2);
    drop(pool);
    let _ = std::fs::remove_file(&shm_path);
}

#[test]
fn tensor_pool_open_nonexistent_fails() {
    let result = TensorPool::open(99999);
    assert!(result.is_err(), "opening a nonexistent pool should fail");
}
