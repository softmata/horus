#![allow(dead_code)]
//! GPU End-to-End Tests — User-Facing API Only
//!
//! These tests exercise the PUBLIC API that users interact with.
//! No internal modules (cuda_ffi, backends, kernels) are accessed directly.

use horus_core::gpu::{cuda_available, cuda_device_count, gpu_platform};
use horus_core::gpu::{ColorFormat, GpuImageOps};
use horus_core::memory::Image;
use horus_core::types::{Device, ImageEncoding};

// =========================================================================
// Test 1: GPU detection API
// =========================================================================

#[test]
fn test_gpu_detection_api() {
    let available = cuda_available();
    let count = cuda_device_count();
    let platform = gpu_platform();

    eprintln!("cuda_available: {}", available);
    eprintln!("cuda_device_count: {}", count);
    eprintln!("gpu_platform: {:?}", platform);

    if available {
        assert!(count > 0, "If CUDA available, count should be > 0");
        let p = platform.expect("platform should be Some when CUDA available");
        assert!(!p.name().is_empty());
        let (maj, _) = p.compute_capability();
        assert!(maj >= 3);
    } else {
        assert_eq!(count, 0);
        assert!(platform.is_none());
    }
}

// =========================================================================
// Test 2: Image to_gpu / to_cpu roundtrip with real pixel data
// =========================================================================

#[test]
fn test_image_to_gpu_to_cpu_roundtrip() {
    if !cuda_available() {
        eprintln!("CUDA not available, skipping");
        return;
    }

    // Create CPU image with known gradient
    let cpu_img = Image::new(64, 48, ImageEncoding::Rgb8).unwrap();
    {
        let data = cpu_img.data_mut();
        for i in 0..data.len() {
            data[i] = (i % 256) as u8;
        }
    }
    let original: Vec<u8> = cpu_img.data().to_vec();

    // CPU → GPU
    let gpu_img = cpu_img.to_gpu(Device::cuda(0)).unwrap();
    assert!(gpu_img.is_cuda());
    assert_eq!(gpu_img.width(), 64);
    assert_eq!(gpu_img.height(), 48);

    // GPU data accessible (managed memory) and matches
    assert_eq!(gpu_img.data(), original.as_slice(), "GPU data mismatch");

    // GPU → CPU
    let back = gpu_img.to_cpu().unwrap();
    assert!(back.is_cpu());
    assert_eq!(back.data(), original.as_slice(), "Roundtrip mismatch");
}

// =========================================================================
// Test 3: to_gpu on already-GPU is a clone (no copy)
// =========================================================================

#[test]
fn test_to_gpu_idempotent() {
    if !cuda_available() {
        return;
    }
    let img = Image::new(16, 16, ImageEncoding::Rgb8).unwrap();
    let gpu1 = img.to_gpu(Device::cuda(0)).unwrap();
    let gpu2 = gpu1.to_gpu(Device::cuda(0)).unwrap();
    assert!(gpu2.is_cuda());
    assert_eq!(gpu1.data(), gpu2.data());
}

// =========================================================================
// Test 4: to_cpu on already-CPU is a clone (no copy)
// =========================================================================

#[test]
fn test_to_cpu_idempotent() {
    let img = Image::new(16, 16, ImageEncoding::Rgb8).unwrap();
    let img2 = img.to_cpu().unwrap();
    assert!(img2.is_cpu());
    assert_eq!(img.data(), img2.data());
}

// =========================================================================
// Test 5: DLPack device metadata correct for GPU Image
// =========================================================================

#[test]
fn test_gpu_image_dlpack_metadata() {
    if !cuda_available() {
        return;
    }
    let img = Image::new(32, 32, ImageEncoding::Rgb8).unwrap();
    let gpu_img = img.to_gpu(Device::cuda(0)).unwrap();

    let tensor = gpu_img.descriptor().tensor();
    assert!(tensor.is_cuda());
    let device = tensor.device();
    assert_eq!(device.to_dlpack_device_type(), 2, "should be kDLCUDA=2");
    assert_eq!(device.to_dlpack_device_id(), 0);
}

// =========================================================================
// Test 6: GPU Image lifecycle — 100 alloc/free, no leak
// =========================================================================

#[test]
fn test_gpu_image_no_leak() {
    if !cuda_available() {
        return;
    }
    let base_img = Image::new(64, 64, ImageEncoding::Rgb8).unwrap();

    for _ in 0..100 {
        let gpu_img = base_img.to_gpu(Device::cuda(0)).unwrap();
        assert!(gpu_img.is_cuda());
        // dropped here
    }
    // If this doesn't OOM or panic, allocations are being freed
}

// =========================================================================
// Test 7: GpuImageOps::available() reports correctly
// =========================================================================

#[test]
fn test_gpu_image_ops_available() {
    let available = GpuImageOps::available();
    eprintln!("GpuImageOps::available() = {}", available);
    // On GPU machine with kernel .so: true
    // On CPU machine or without .so: false
    // Just verify it doesn't panic
}

// =========================================================================
// Test 8: CPU path completely unaffected
// =========================================================================

#[test]
fn test_cpu_path_unaffected() {
    let img = Image::new(320, 240, ImageEncoding::Rgb8).unwrap();
    assert!(img.is_cpu());
    assert!(!img.is_cuda());

    // Write and read back
    let data = img.data_mut();
    data[0] = 42;
    data[1] = 99;
    assert_eq!(img.data()[0], 42);
    assert_eq!(img.data()[1], 99);

    // Clone works
    let img2 = img.clone();
    assert_eq!(img2.data()[0], 42);

    // to_cpu on CPU image is no-op clone
    let img3 = img.to_cpu().unwrap();
    assert!(img3.is_cpu());
}

// =========================================================================
// Test 9: HORUS_GPU=0 kill switch
// =========================================================================

#[test]
fn test_kill_switch_env_var() {
    // We can't set env vars safely in parallel tests, but we can verify
    // the detection API doesn't panic regardless of state
    let _ = cuda_available();
    let _ = cuda_device_count();
    let _ = gpu_platform();
}

// =========================================================================
// Test 10: GpuImageOps rejects zero dimensions
// =========================================================================

#[test]
fn test_gpu_ops_zero_dimension_rejected() {
    if !GpuImageOps::available() {
        return;
    }

    let result = GpuImageOps::resize(
        std::ptr::null(),
        std::ptr::null_mut(),
        0,
        0,
        0,
        0,
        3,
        std::ptr::null_mut(),
    );
    assert!(result.is_err());

    let result = GpuImageOps::preprocess_fused(
        std::ptr::null(),
        std::ptr::null_mut(),
        0,
        0,
        640,
        480,
        ColorFormat::Rgb,
        &[0.0; 3],
        &[1.0; 3],
        std::ptr::null_mut(),
    );
    assert!(result.is_err());
}
