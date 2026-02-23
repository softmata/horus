// Integration tests for GPU detection on CPU-only systems.
//
// These tests verify that GPU auto-detection works correctly on machines
// without CUDA GPUs — the common dev/CI case. All tests pass without
// GPU hardware.

use horus_core::memory::{gpu_capability, GpuCapability, TensorPool, TensorPoolConfig};
use horus_core::scheduling::SchedulerConfig;

// =============================================================================
// GPU detection basics
// =============================================================================

#[test]
fn test_gpu_capability_returns_none_without_cuda() {
    // On CI and most dev machines, no CUDA driver is installed.
    // gpu_capability() should return GpuCapability::None.
    let cap = gpu_capability();
    // We can't assert None on a machine WITH a GPU, so we just verify
    // the result is valid and deterministic.
    let cap2 = gpu_capability();
    assert_eq!(cap, cap2, "gpu_capability() must be deterministic");
}

#[test]
fn test_gpu_capability_none_methods() {
    let none = GpuCapability::None;
    assert!(!none.has_gpu());
    assert!(!none.is_unified());
    assert!(!none.needs_coherency_sync());
    assert_eq!(none.device_id(), None);
}

#[test]
fn test_gpu_capability_is_copy_and_debug() {
    let cap = gpu_capability();
    let cap2 = cap; // Copy
    assert_eq!(cap, cap2);
    let _ = format!("{:?}", cap); // Debug
}

// =============================================================================
// CPU-only pool allocation
// =============================================================================

#[test]
fn test_tensor_pool_allocates_on_cpu_only() {
    use horus_types::{Device, TensorDtype};
    use std::sync::atomic::{AtomicU32, Ordering};

    static POOL_ID: AtomicU32 = AtomicU32::new(97000);
    let pool_id = POOL_ID.fetch_add(1, Ordering::Relaxed);

    let config = TensorPoolConfig {
        pool_size: 1024 * 1024,
        max_slots: 8,
        slot_alignment: 64,
        allocator: Default::default(), // Mmap
    };

    let pool = TensorPool::new(pool_id, config).expect("pool creation should succeed on CPU");

    // Allocate a tensor — should work on CPU-only
    let tensor = pool
        .alloc(&[10, 20], TensorDtype::F32, Device::cpu())
        .expect("alloc should succeed");

    // Verify data is accessible
    let data = pool.data_slice(&tensor);
    assert_eq!(data.len(), 10 * 20 * 4); // 800 bytes

    pool.release(&tensor);
    std::fs::remove_file(pool.shm_path()).ok();
}

// =============================================================================
// Image/Topic round-trip on CPU-only
// =============================================================================

#[test]
fn test_image_roundtrip_cpu_only() {
    use horus_core::communication::Topic;
    use horus_core::memory::Image;
    use horus_types::ImageEncoding;

    let topic: Topic<Image> = Topic::new("test/gpu_det_img_rt").unwrap();

    // Create a small RGB image
    let mut img = Image::new(4, 8, ImageEncoding::Rgb8).unwrap();
    let pixel_data = vec![42u8; 4 * 8 * 3];
    img.copy_from(&pixel_data);
    img.set_frame_id("camera");

    // Send and receive
    topic.send(&img);
    let recv = topic.recv().expect("should receive image");

    assert_eq!(recv.height(), 4);
    assert_eq!(recv.width(), 8);
    assert_eq!(recv.encoding(), ImageEncoding::Rgb8);
    assert_eq!(recv.frame_id(), "camera");
    assert_eq!(recv.data(), &pixel_data[..]);
}

#[test]
fn test_pointcloud_roundtrip_cpu_only() {
    use horus_core::communication::Topic;
    use horus_core::memory::PointCloud;
    use horus_types::TensorDtype;

    let topic: Topic<PointCloud> = Topic::new("test/gpu_det_pc_rt").unwrap();

    let mut pc = PointCloud::new(100, 3, TensorDtype::F32).unwrap();
    pc.set_frame_id("lidar");
    // Fill with some data
    let data = vec![1u8; 100 * 3 * 4];
    pc.copy_from(&data);

    topic.send(&pc);
    let recv = topic.recv().expect("should receive pointcloud");

    assert_eq!(recv.point_count(), 100);
    assert_eq!(recv.frame_id(), "lidar");
}

#[test]
fn test_depth_image_roundtrip_cpu_only() {
    use horus_core::communication::Topic;
    use horus_core::memory::DepthImage;
    use horus_types::TensorDtype;

    let topic: Topic<DepthImage> = Topic::new("test/gpu_det_depth_rt").unwrap();

    let mut depth = DepthImage::new(4, 8, TensorDtype::F32).unwrap();
    depth.set_frame_id("depth_cam");

    topic.send(&depth);
    let recv = topic.recv().expect("should receive depth image");

    assert_eq!(recv.height(), 4);
    assert_eq!(recv.width(), 8);
    assert_eq!(recv.frame_id(), "depth_cam");
}

// =============================================================================
// SchedulerConfig GPU fields
// =============================================================================

#[test]
fn test_scheduler_config_gpu_defaults() {
    let config = SchedulerConfig::minimal();
    assert!(!config.resources.force_cpu_only);
    assert!(config.resources.gpu_memory_budget_mb.is_none());
}

#[test]
fn test_all_presets_have_auto_detect_defaults() {
    // Every preset should default to auto-detect (no forced CPU, no memory limit)
    let presets = vec![
        SchedulerConfig::minimal(),
        SchedulerConfig::standard(),
        SchedulerConfig::deploy(),
        SchedulerConfig::deterministic(),
        SchedulerConfig::safety_critical(),
        SchedulerConfig::high_performance(),
        SchedulerConfig::hard_realtime(),
    ];

    for (i, config) in presets.iter().enumerate() {
        assert!(
            !config.resources.force_cpu_only,
            "preset {} should not force CPU-only",
            i
        );
        assert!(
            config.resources.gpu_memory_budget_mb.is_none(),
            "preset {} should not limit GPU memory",
            i
        );
    }
}

#[test]
fn test_force_cpu_only_override() {
    let mut config = SchedulerConfig::standard();
    config.resources.force_cpu_only = true;
    assert!(config.resources.force_cpu_only);
    // Memory budget can be set independently
    config.resources.gpu_memory_budget_mb = Some(512);
    assert_eq!(config.resources.gpu_memory_budget_mb, Some(512));
}

// =============================================================================
// cuda_available / cuda_device_count stubs
// =============================================================================

#[test]
fn test_cuda_available_stub() {
    // Without cuda feature, this always returns false
    let available = horus_core::memory::cuda_available();
    // On CPU-only: false. On GPU machine: could be true.
    // Just verify it doesn't panic and returns a bool.
    let _ = available;
}

#[test]
fn test_cuda_device_count_stub() {
    let count = horus_core::memory::cuda_device_count();
    // On CPU-only: 0. On GPU machine: >= 1.
    // Just verify it doesn't panic.
    let _ = count;
}
