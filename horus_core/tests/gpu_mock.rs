//! Mock GPU tests — run on every CI machine, no GPU required.
//!
//! Uses only PUBLIC API. No internal modules accessed.

use horus_core::gpu::{cuda_available, cuda_device_count, gpu_platform};
use horus_core::memory::{TensorPool, TensorPoolConfig};
use horus_core::types::{Device, TensorDtype};

/// Detection API doesn't panic regardless of GPU presence.
#[test]
fn test_detection_no_panic() {
    let _ = cuda_available();
    let _ = cuda_device_count();
    let _ = gpu_platform();
}

/// cuda_device_count returns reasonable value.
#[test]
fn test_device_count_reasonable() {
    let count = cuda_device_count();
    assert!(count <= 128, "unreasonable device count: {}", count);
}

/// gpu_platform doesn't panic.
#[test]
fn test_gpu_platform_no_panic() {
    let platform = gpu_platform();
    if let Some(p) = platform {
        assert!(!p.name().is_empty());
    }
}

/// MmapBackend is still default for CPU pools.
#[test]
fn test_mmap_backend_still_default() {
    let config = TensorPoolConfig {
        pool_size: 1024 * 1024,
        max_slots: 16,
        ..Default::default()
    };
    let pool = TensorPool::new(99990, config).unwrap();
    assert_eq!(pool.backend_name(), "mmap");
    assert_eq!(pool.backend_device(), Device::cpu());

    let tensor = pool.alloc(&[100], TensorDtype::F32, Device::cpu()).unwrap();
    let data = pool.data_slice(&tensor).unwrap();
    assert_eq!(data.len(), 400);

    pool.release(&tensor);
    std::fs::remove_file(pool.shm_path()).ok();
}

/// Device mismatch rejected (soundness gap closed).
#[test]
fn test_device_mismatch_rejected() {
    let config = TensorPoolConfig {
        pool_size: 1024 * 1024,
        max_slots: 16,
        ..Default::default()
    };
    let pool = TensorPool::new(99991, config).unwrap();

    let result = pool.alloc(&[100], TensorDtype::F32, Device::cuda(0));
    assert!(result.is_err());

    let err_msg = format!("{}", result.unwrap_err());
    assert!(err_msg.contains("device mismatch"), "error: {}", err_msg);

    std::fs::remove_file(pool.shm_path()).ok();
}

/// CPU Image unaffected by GPU changes.
#[test]
fn test_cpu_image_works() {
    use horus_core::memory::Image;
    use horus_core::types::ImageEncoding;

    let img = Image::new(32, 32, ImageEncoding::Rgb8).unwrap();
    assert!(img.is_cpu());
    assert!(!img.is_cuda());
    assert_eq!(img.data().len(), 32 * 32 * 3);
}
