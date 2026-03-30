#![allow(dead_code)]
//! Comprehensive kernel correctness tests — PUBLIC API ONLY.
//!
//! Tests each kernel via GpuImageOps (public) with known inputs and expected outputs.
//! For kernels that need raw pointers, we use Image::to_gpu() + .data() on managed memory.

use horus_core::gpu::{cuda_available, gpu_synchronize, ColorFormat, GpuImageOps};
use horus_core::memory::Image;
use horus_core::types::{Device, ImageEncoding};

// =========================================================================
// color_convert: RGB→BGR→RGB roundtrip via GpuImageOps
// =========================================================================

#[test]
fn test_color_convert_rgb_bgr_roundtrip() {
    if !cuda_available() || !GpuImageOps::available() {
        return;
    }

    let img = Image::new(16, 16, ImageEncoding::Rgb8).unwrap();
    {
        let data = img.data_mut();
        for i in 0..(16 * 16) {
            data[i * 3] = 10; // R
            data[i * 3 + 1] = 20; // G
            data[i * 3 + 2] = 30; // B
        }
    }

    let gpu_img = img.to_gpu(Device::cuda(0)).unwrap();
    let src = gpu_img.data().as_ptr();
    let _size = 16 * 16 * 3;

    // Allocate GPU output buffers using to_gpu on a blank image
    let mid_img = Image::new(16, 16, ImageEncoding::Rgb8).unwrap();
    let mid_gpu = mid_img.to_gpu(Device::cuda(0)).unwrap();
    let mid = mid_gpu.data().as_ptr() as *mut u8;

    let dst_img = Image::new(16, 16, ImageEncoding::Rgb8).unwrap();
    let dst_gpu = dst_img.to_gpu(Device::cuda(0)).unwrap();
    let dst = dst_gpu.data().as_ptr() as *mut u8;

    // RGB → BGR
    GpuImageOps::color_convert(
        src,
        mid,
        16,
        16,
        ColorFormat::Rgb,
        ColorFormat::Bgr,
        std::ptr::null_mut(),
    )
    .unwrap();
    gpu_synchronize().unwrap();

    // BGR → RGB
    GpuImageOps::color_convert(
        mid as *const u8,
        dst,
        16,
        16,
        ColorFormat::Bgr,
        ColorFormat::Rgb,
        std::ptr::null_mut(),
    )
    .unwrap();
    gpu_synchronize().unwrap();

    // Verify roundtrip
    let result = dst_gpu.data();
    assert_eq!(result[0], 10, "R roundtrip");
    assert_eq!(result[1], 20, "G roundtrip");
    assert_eq!(result[2], 30, "B roundtrip");

    // Verify mid has swapped: [30, 20, 10]
    let mid_data = mid_gpu.data();
    assert_eq!(mid_data[0], 30, "BGR.B");
    assert_eq!(mid_data[1], 20, "BGR.G");
    assert_eq!(mid_data[2], 10, "BGR.R");
}

// =========================================================================
// resize: uniform input → uniform output
// =========================================================================

#[test]
fn test_resize_uniform_preserved() {
    if !cuda_available() || !GpuImageOps::available() {
        return;
    }

    let img = Image::new(32, 32, ImageEncoding::Rgb8).unwrap();
    img.data_mut().fill(128);
    let gpu_img = img.to_gpu(Device::cuda(0)).unwrap();

    let out = Image::new(16, 16, ImageEncoding::Rgb8).unwrap();
    let out_gpu = out.to_gpu(Device::cuda(0)).unwrap();

    GpuImageOps::resize(
        gpu_img.data().as_ptr(),
        out_gpu.data().as_ptr() as *mut u8,
        32,
        32,
        16,
        16,
        3,
        std::ptr::null_mut(),
    )
    .unwrap();
    gpu_synchronize().unwrap();

    let result = out_gpu.data();
    for (i, &val) in result.iter().enumerate() {
        assert!(
            (val as i32 - 128).unsigned_abs() <= 1,
            "byte {}: expected ~128, got {}",
            i,
            val
        );
    }
}

// =========================================================================
// normalize: known values with ImageNet mean/std
// =========================================================================

#[test]
fn test_normalize_known_values() {
    if !cuda_available() || !GpuImageOps::available() {
        return;
    }

    // 2x2 RGB: [0, 128, 255] per pixel
    let img = Image::new(2, 2, ImageEncoding::Rgb8).unwrap();
    {
        let data = img.data_mut();
        for i in 0..4 {
            data[i * 3] = 0; // R=0
            data[i * 3 + 1] = 128; // G=128
            data[i * 3 + 2] = 255; // B=255
        }
    }
    let gpu_img = img.to_gpu(Device::cuda(0)).unwrap();

    let _out_size = 2 * 2 * 3 * 4; // f32 HWC
                                   // Use a raw managed alloc for f32 output since Image is u8 only
                                   // Just test via the fused kernel which outputs f32 CHW
                                   // (individual normalize kernel needs raw f32 pointer — tested via fused)

    // Test via preprocess_fused with 2x2→2x2 (no resize effect)
    let mean = [0.485f32, 0.456, 0.406];
    let std_dev = [0.229f32, 0.224, 0.225];

    // Allocate f32 output as managed memory via a tensor
    use horus_core::memory::TensorPool;
    let pool = std::sync::Arc::new(TensorPool::new(77099, Default::default()).unwrap());
    let tensor = pool
        .alloc(
            &[3, 2, 2], // CHW
            horus_core::types::TensorDtype::F32,
            Device::cpu(),
        )
        .unwrap();

    // Use a temporary managed GPU buffer for output
    // Actually, the fused kernel needs GPU pointers. Let's use the GPU image data directly.
    let src_ptr = gpu_img.data().as_ptr();

    // For the f32 output, we need a GPU buffer. Use a second GPU image as raw storage.
    let _out_img = Image::new(6, 2, ImageEncoding::Rgb8).unwrap(); // 6*2*3=36 bytes ≥ 3*2*2*4=48... not enough
                                                                   // Just use a bigger buffer
    let out_img = Image::new(16, 16, ImageEncoding::Rgb8).unwrap(); // 768 bytes >> 48
    let out_gpu = out_img.to_gpu(Device::cuda(0)).unwrap();

    GpuImageOps::preprocess_fused(
        src_ptr,
        out_gpu.data().as_ptr() as *mut f32,
        2,
        2,
        2,
        2, // no resize
        ColorFormat::Rgb,
        &mean,
        &std_dev,
        std::ptr::null_mut(),
    )
    .unwrap();
    gpu_synchronize().unwrap();

    // Read f32 from the output buffer (reinterpret u8 slice as f32)
    let out_bytes = out_gpu.data();
    let out_f32: &[f32] =
        unsafe { std::slice::from_raw_parts(out_bytes.as_ptr() as *const f32, 3 * 2 * 2) };

    // CHW layout: channel 0 = R values
    // R: 0/255=0 → (0-0.485)/0.229 = -2.118
    // G: 128/255≈0.502 → (0.502-0.456)/0.224 = 0.205
    // B: 255/255=1.0 → (1.0-0.406)/0.225 = 2.640
    let r = out_f32[0]; // CHW[0][0][0]
    let g = out_f32[4]; // CHW[1][0][0]
    let b = out_f32[8]; // CHW[2][0][0]

    eprintln!("Normalize via fused: R={:.3} G={:.3} B={:.3}", r, g, b);
    assert!((r - (-2.118)).abs() < 0.05, "R: expected -2.118, got {}", r);
    assert!((g - 0.205).abs() < 0.05, "G: expected 0.205, got {}", g);
    assert!((b - 2.640).abs() < 0.05, "B: expected 2.640, got {}", b);

    pool.release(&tensor);
    std::fs::remove_file(pool.shm_path()).ok();
}

// =========================================================================
// fused: BGR input swaps channels correctly
// =========================================================================

#[test]
fn test_fused_bgr_swaps_channels() {
    if !cuda_available() || !GpuImageOps::available() {
        return;
    }

    // BGR input: B=50, G=100, R=200
    let img = Image::new(8, 8, ImageEncoding::Bgr8).unwrap();
    {
        let data = img.data_mut();
        for i in 0..(8 * 8) {
            data[i * 3] = 50; // B
            data[i * 3 + 1] = 100; // G
            data[i * 3 + 2] = 200; // R
        }
    }
    let gpu_img = img.to_gpu(Device::cuda(0)).unwrap();

    // Output buffer (4x4 CHW f32)
    let out = Image::new(16, 16, ImageEncoding::Rgb8).unwrap(); // big enough for f32 data
    let out_gpu = out.to_gpu(Device::cuda(0)).unwrap();

    let mean = [0.485f32, 0.456, 0.406];
    let std_dev = [0.229f32, 0.224, 0.225];

    GpuImageOps::preprocess_fused(
        gpu_img.data().as_ptr(),
        out_gpu.data().as_ptr() as *mut f32,
        8,
        8,
        4,
        4,
        ColorFormat::Bgr,
        &mean,
        &std_dev,
        std::ptr::null_mut(),
    )
    .unwrap();
    gpu_synchronize().unwrap();

    let out_bytes = out_gpu.data();
    let out_f32: &[f32] =
        unsafe { std::slice::from_raw_parts(out_bytes.as_ptr() as *const f32, 3 * 4 * 4) };

    // After BGR→RGB swap in fused kernel: R=200, G=100, B=50
    // R: 200/255≈0.784 → (0.784-0.485)/0.229 ≈ 1.308
    // B: 50/255≈0.196 → (0.196-0.406)/0.225 ≈ -0.933
    let r = out_f32[0];
    let b = out_f32[2 * 16]; // channel 2, first pixel

    eprintln!("BGR fused: R={:.3} B={:.3}", r, b);
    assert!(
        (r - 1.308).abs() < 0.05,
        "R from BGR: expected ~1.308, got {}",
        r
    );
    assert!(
        (b - (-0.933)).abs() < 0.05,
        "B from BGR: expected ~-0.933, got {}",
        b
    );
}

// =========================================================================
// crop_pad: correct region extracted
// =========================================================================

#[test]
fn test_crop_extracts_correct_region() {
    if !cuda_available() || !GpuImageOps::available() {
        return;
    }

    // 8x8 grayscale: value = y*8 + x
    let src = Image::new(8, 8, ImageEncoding::Mono8).unwrap();
    {
        let data = src.data_mut();
        for y in 0..8u32 {
            for x in 0..8u32 {
                data[(y * 8 + x) as usize] = (y * 8 + x) as u8;
            }
        }
    }
    let src_gpu = src.to_gpu(Device::cuda(0)).unwrap();

    let dst = Image::new(4, 4, ImageEncoding::Mono8).unwrap();
    let dst_gpu = dst.to_gpu(Device::cuda(0)).unwrap();

    GpuImageOps::crop_pad(
        src_gpu.data().as_ptr(),
        dst_gpu.data().as_ptr() as *mut u8,
        8,
        8,
        2,
        2,
        4,
        4, // crop (2,2) size 4x4
        4,
        4,
        1,
        114,
        std::ptr::null_mut(),
    )
    .unwrap();
    gpu_synchronize().unwrap();

    let result = dst_gpu.data();
    // dst[0][0] = src[2][2] = 2*8+2 = 18
    assert_eq!(result[0], 18, "crop origin: expected 18, got {}", result[0]);
    // dst[1][1] = src[3][3] = 27
    assert_eq!(result[5], 27, "crop [1][1]: expected 27, got {}", result[5]);
}

// =========================================================================
// GpuImageOps::available matches cuda_available + kernels
// =========================================================================

#[test]
fn test_ops_available_consistent() {
    let ops = GpuImageOps::available();
    let cuda = cuda_available();
    // If CUDA available and kernels loaded, ops should be true
    // If no CUDA, ops must be false
    if !cuda {
        assert!(!ops, "GpuImageOps should not be available without CUDA");
    }
    eprintln!("cuda_available={}, GpuImageOps::available={}", cuda, ops);
}
