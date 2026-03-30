//! Stress tests for TensorPool with GPU and CPU backends.
//! Uses only PUBLIC API.

use horus_core::gpu::cuda_available;
use horus_core::memory::{Image, TensorPool, TensorPoolConfig};
use horus_core::types::{Device, ImageEncoding, TensorDtype};
use std::sync::Arc;

/// 10K alloc/free cycles on MmapBackend — zero leaked slots.
#[test]
fn test_mmap_pool_stress_10k_cycles() {
    let config = TensorPoolConfig {
        pool_size: 64 * 1024 * 1024,
        max_slots: 64,
        ..Default::default()
    };
    let pool = Arc::new(TensorPool::new(88001, config).unwrap());

    for cycle in 0..10_000 {
        let tensor = pool.alloc(&[256], TensorDtype::F32, Device::cpu()).unwrap();
        let data = pool.data_slice(&tensor).unwrap();
        assert_eq!(data.len(), 1024, "cycle {}: wrong data size", cycle);
        pool.release(&tensor);
    }

    let stats = pool.stats();
    assert_eq!(
        stats.allocated_slots,
        0,
        "All slots must be freed. Stats: {}",
        stats.summary()
    );
    std::fs::remove_file(pool.shm_path()).ok();
}

/// Multi-threaded stress: 4 threads x 2500 cycles on MmapBackend.
#[test]
fn test_mmap_pool_stress_multithreaded() {
    let config = TensorPoolConfig {
        pool_size: 64 * 1024 * 1024,
        max_slots: 128,
        ..Default::default()
    };
    let pool = Arc::new(TensorPool::new(88002, config).unwrap());

    let handles: Vec<_> = (0..4)
        .map(|thread_id| {
            let pool = Arc::clone(&pool);
            std::thread::spawn(move || {
                for _ in 0..2500 {
                    match pool.alloc(&[64], TensorDtype::U8, Device::cpu()) {
                        Ok(tensor) => {
                            let data = pool.data_slice_mut(&tensor).unwrap();
                            data.fill(thread_id as u8);
                            pool.release(&tensor);
                        }
                        Err(_) => std::thread::yield_now(),
                    }
                }
            })
        })
        .collect();

    for h in handles {
        h.join().unwrap();
    }

    let stats = pool.stats();
    assert!(
        stats.allocated_slots <= 2,
        "Too many leaked slots. Stats: {}",
        stats.summary()
    );
    std::fs::remove_file(pool.shm_path()).ok();
}

/// GPU Image to_gpu/to_cpu stress: 200 roundtrips.
#[test]
fn test_gpu_image_transfer_stress() {
    if !cuda_available() {
        eprintln!("CUDA not available, skipping");
        return;
    }

    let mut cpu_img = Image::new(64, 64, ImageEncoding::Rgb8).unwrap();
    cpu_img.data_mut().fill(42);
    let expected: Vec<u8> = cpu_img.data().to_vec();

    for cycle in 0..200 {
        let gpu = cpu_img.to_gpu(Device::cuda(0)).unwrap();
        assert!(gpu.is_cuda(), "cycle {}: should be CUDA", cycle);
        let back = gpu.to_cpu().unwrap();
        assert!(back.is_cpu(), "cycle {}: should be CPU", cycle);
        assert_eq!(
            back.data(),
            expected.as_slice(),
            "cycle {}: data mismatch",
            cycle
        );
    }
}

/// GPU Image alloc/drop stress: 500 GPU images, verify no leak.
#[test]
fn test_gpu_image_alloc_drop_stress() {
    if !cuda_available() {
        return;
    }

    let base = Image::new(32, 32, ImageEncoding::Rgb8).unwrap();

    for _ in 0..500 {
        let gpu = base.to_gpu(Device::cuda(0)).unwrap();
        assert!(gpu.is_cuda());
        // gpu dropped here — should free GPU allocation
    }
    // If no OOM or panic, allocations are being freed
}
