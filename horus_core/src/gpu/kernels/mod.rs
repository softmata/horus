//! CUDA preprocessing kernel loader.
//!
//! Loads `libhorus_kernels.so` via dlopen at runtime and exposes safe
//! Rust wrappers around each kernel function.

use std::ffi::c_void;
use std::sync::OnceLock;

use super::cuda_ffi::{CudaError, CudaResult};

type KernelResult = i32;

/// All kernel function pointers loaded from libhorus_kernels.so.
struct KernelApi {
    color_convert: unsafe extern "C" fn(*const u8, *mut u8, i32, i32, i32, i32, *mut c_void) -> KernelResult,
    resize: unsafe extern "C" fn(*const u8, *mut u8, i32, i32, i32, i32, i32, *mut c_void) -> KernelResult,
    normalize: unsafe extern "C" fn(*const u8, *mut f32, i32, i32, i32, *const f32, *const f32, *mut c_void) -> KernelResult,
    transpose_hwc_to_chw: unsafe extern "C" fn(*const c_void, *mut c_void, i32, i32, i32, i32, *mut c_void) -> KernelResult,
    transpose_chw_to_hwc: unsafe extern "C" fn(*const c_void, *mut c_void, i32, i32, i32, i32, *mut c_void) -> KernelResult,
    undistort: unsafe extern "C" fn(*const u8, *mut u8, i32, i32, i32, *const f32, *const f32, *mut c_void) -> KernelResult,
    crop_pad: unsafe extern "C" fn(*const u8, *mut u8, i32, i32, i32, i32, i32, i32, i32, i32, i32, u8, *mut c_void) -> KernelResult,
    preprocess_fused: unsafe extern "C" fn(*const u8, *mut f32, i32, i32, i32, i32, i32, *const f32, *const f32, *mut c_void) -> KernelResult,
}

unsafe impl Send for KernelApi {}
unsafe impl Sync for KernelApi {}

static KERNEL_API: OnceLock<Result<KernelApi, String>> = OnceLock::new();

fn load_kernel_api() -> Result<KernelApi, String> {
    // Search paths for the .so
    let search_paths = [
        // Build directory (during development)
        concat!(env!("CARGO_MANIFEST_DIR"), "/src/gpu/kernels/libhorus_kernels.so"),
        // Installed location
        "/usr/local/lib/libhorus_kernels.so",
        // Relative to binary
        "./libhorus_kernels.so",
    ];

    let mut handle: *mut c_void = std::ptr::null_mut();
    let mut last_err = String::new();

    for path in &search_paths {
        let cpath = std::ffi::CString::new(*path).unwrap();
        unsafe { libc::dlerror() }; // clear
        let h = unsafe { libc::dlopen(cpath.as_ptr(), libc::RTLD_NOW) };
        if !h.is_null() {
            handle = h;
            break;
        }
        let err = unsafe { libc::dlerror() };
        if !err.is_null() {
            last_err = unsafe { std::ffi::CStr::from_ptr(err).to_string_lossy().to_string() };
        }
    }

    // Also try without path (system library search)
    if handle.is_null() {
        let cname = std::ffi::CString::new("libhorus_kernels.so").unwrap();
        handle = unsafe { libc::dlopen(cname.as_ptr(), libc::RTLD_NOW) };
    }

    if handle.is_null() {
        return Err(format!(
            "Failed to load libhorus_kernels.so (searched {} paths): {}",
            search_paths.len(),
            last_err
        ));
    }

    unsafe {
        Ok(KernelApi {
            color_convert: load_sym(handle, "horus_color_convert")?,
            resize: load_sym(handle, "horus_resize")?,
            normalize: load_sym(handle, "horus_normalize")?,
            transpose_hwc_to_chw: load_sym(handle, "horus_transpose_hwc_to_chw")?,
            transpose_chw_to_hwc: load_sym(handle, "horus_transpose_chw_to_hwc")?,
            undistort: load_sym(handle, "horus_undistort")?,
            crop_pad: load_sym(handle, "horus_crop_pad")?,
            preprocess_fused: load_sym(handle, "horus_preprocess_fused")?,
        })
    }
}

unsafe fn load_sym<T>(handle: *mut c_void, name: &str) -> Result<T, String> {
    let cname = std::ffi::CString::new(name).unwrap();
    let ptr = libc::dlsym(handle, cname.as_ptr());
    if ptr.is_null() {
        return Err(format!("kernel symbol '{}' not found", name));
    }
    Ok(std::mem::transmute_copy(&ptr))
}

fn api() -> CudaResult<&'static KernelApi> {
    KERNEL_API
        .get_or_init(load_kernel_api)
        .as_ref()
        .map_err(|msg| CudaError {
            code: -1,
            message: msg.clone(),
        })
}

fn check_kernel(code: KernelResult, name: &str) -> CudaResult<()> {
    if code == 0 {
        Ok(())
    } else {
        Err(CudaError {
            code,
            message: format!("kernel '{}' failed (cuda error {})", name, code),
        })
    }
}

/// Check if the kernel library is loaded and available.
pub fn kernels_available() -> bool {
    api().is_ok()
}

// ---------------------------------------------------------------------------
// Public safe kernel wrappers
// ---------------------------------------------------------------------------

/// Color space conversion.
/// src_fmt/dst_fmt: 0=RGB, 1=BGR, 2=GRAY
pub fn color_convert(
    src: *const u8, dst: *mut u8,
    width: i32, height: i32,
    src_fmt: i32, dst_fmt: i32,
    stream: *mut c_void,
) -> CudaResult<()> {
    let k = api()?;
    check_kernel(unsafe { (k.color_convert)(src, dst, width, height, src_fmt, dst_fmt, stream) }, "color_convert")
}

/// Bilinear resize.
pub fn resize(
    src: *const u8, dst: *mut u8,
    src_w: i32, src_h: i32,
    dst_w: i32, dst_h: i32,
    channels: i32,
    stream: *mut c_void,
) -> CudaResult<()> {
    let k = api()?;
    check_kernel(unsafe { (k.resize)(src, dst, src_w, src_h, dst_w, dst_h, channels, stream) }, "resize")
}

/// Per-channel normalize: u8 HWC -> f32 HWC.
pub fn normalize(
    src: *const u8, dst: *mut f32,
    width: i32, height: i32, channels: i32,
    mean: &[f32; 4], stddev: &[f32; 4],
    stream: *mut c_void,
) -> CudaResult<()> {
    let k = api()?;
    check_kernel(unsafe { (k.normalize)(src, dst, width, height, channels, mean.as_ptr(), stddev.as_ptr(), stream) }, "normalize")
}

/// HWC -> CHW transpose.
pub fn transpose_hwc_to_chw(
    src: *const c_void, dst: *mut c_void,
    width: i32, height: i32, channels: i32, elem_size: i32,
    stream: *mut c_void,
) -> CudaResult<()> {
    let k = api()?;
    check_kernel(unsafe { (k.transpose_hwc_to_chw)(src, dst, width, height, channels, elem_size, stream) }, "transpose_hwc_to_chw")
}

/// CHW -> HWC transpose.
pub fn transpose_chw_to_hwc(
    src: *const c_void, dst: *mut c_void,
    width: i32, height: i32, channels: i32, elem_size: i32,
    stream: *mut c_void,
) -> CudaResult<()> {
    let k = api()?;
    check_kernel(unsafe { (k.transpose_chw_to_hwc)(src, dst, width, height, channels, elem_size, stream) }, "transpose_chw_to_hwc")
}

/// Lens undistortion.
pub fn undistort(
    src: *const u8, dst: *mut u8,
    width: i32, height: i32, channels: i32,
    k_intrinsics: &[f32; 4], dist: &[f32; 5],
    stream: *mut c_void,
) -> CudaResult<()> {
    let k = api()?;
    check_kernel(unsafe { (k.undistort)(src, dst, width, height, channels, k_intrinsics.as_ptr(), dist.as_ptr(), stream) }, "undistort")
}

/// ROI crop with padding.
pub fn crop_pad(
    src: *const u8, dst: *mut u8,
    src_w: i32, src_h: i32,
    crop_x: i32, crop_y: i32, crop_w: i32, crop_h: i32,
    dst_w: i32, dst_h: i32,
    channels: i32, pad_value: u8,
    stream: *mut c_void,
) -> CudaResult<()> {
    let k = api()?;
    check_kernel(unsafe { (k.crop_pad)(src, dst, src_w, src_h, crop_x, crop_y, crop_w, crop_h, dst_w, dst_h, channels, pad_value, stream) }, "crop_pad")
}

/// Fused preprocessing: color_convert + resize + normalize + HWC->CHW.
/// Input: u8 HWC, Output: f32 CHW normalized.
pub fn preprocess_fused(
    src: *const u8, dst: *mut f32,
    src_w: i32, src_h: i32,
    dst_w: i32, dst_h: i32,
    src_fmt: i32,
    mean: &[f32; 3], stddev: &[f32; 3],
    stream: *mut c_void,
) -> CudaResult<()> {
    let k = api()?;
    check_kernel(unsafe { (k.preprocess_fused)(src, dst, src_w, src_h, dst_w, dst_h, src_fmt, mean.as_ptr(), stddev.as_ptr(), stream) }, "preprocess_fused")
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::gpu::cuda_ffi;

    #[test]
    fn test_kernels_load() {
        if !cuda_ffi::is_available() {
            eprintln!("CUDA not available, skipping");
            return;
        }
        assert!(kernels_available(), "libhorus_kernels.so should be loadable");
    }

    #[test]
    fn test_resize_kernel() {
        if !cuda_ffi::is_available() || !kernels_available() {
            return;
        }
        let _ctx = cuda_ffi::CudaContext::new(0).unwrap();

        // Create a 4x4 RGB image on GPU
        let src_size = 4 * 4 * 3;
        let dst_size = 2 * 2 * 3;
        let src_dptr = cuda_ffi::mem_alloc_managed(src_size).unwrap();
        let dst_dptr = cuda_ffi::mem_alloc_managed(dst_size).unwrap();

        // Fill source with a pattern
        let src_ptr = src_dptr as *mut u8;
        unsafe {
            for i in 0..src_size {
                *src_ptr.add(i) = (i % 256) as u8;
            }
        }

        // Resize 4x4 -> 2x2
        resize(
            src_ptr as *const u8, dst_dptr as *mut u8,
            4, 4, 2, 2, 3,
            std::ptr::null_mut(), // default stream
        ).unwrap();

        // Synchronize and check output exists (non-zero)
        _ctx.synchronize().unwrap();
        let dst_ptr = dst_dptr as *const u8;
        let mut has_nonzero = false;
        unsafe {
            for i in 0..dst_size {
                if *dst_ptr.add(i) != 0 { has_nonzero = true; }
            }
        }
        assert!(has_nonzero, "resize output should have non-zero values");

        cuda_ffi::mem_free(src_dptr).unwrap();
        cuda_ffi::mem_free(dst_dptr).unwrap();
    }

    #[test]
    fn test_fused_preprocess() {
        if !cuda_ffi::is_available() || !kernels_available() {
            return;
        }
        let _ctx = cuda_ffi::CudaContext::new(0).unwrap();

        let src_w = 8;
        let src_h = 8;
        let dst_w = 4;
        let dst_h = 4;
        let src_size = src_w * src_h * 3;
        let dst_size = 3 * dst_w * dst_h * 4; // f32 CHW

        let src_dptr = cuda_ffi::mem_alloc_managed(src_size).unwrap();
        let dst_dptr = cuda_ffi::mem_alloc_managed(dst_size).unwrap();

        // Fill source with 128 (mid-gray)
        let src_ptr = src_dptr as *mut u8;
        unsafe { std::ptr::write_bytes(src_ptr, 128, src_size); }

        let mean = [0.485f32, 0.456, 0.406];
        let std = [0.229f32, 0.224, 0.225];

        preprocess_fused(
            src_ptr as *const u8, dst_dptr as *mut f32,
            src_w as i32, src_h as i32,
            dst_w as i32, dst_h as i32,
            0, // RGB
            &mean, &std,
            std::ptr::null_mut(),
        ).unwrap();

        _ctx.synchronize().unwrap();

        // Check output: 128/255 ≈ 0.502
        // Channel 0: (0.502 - 0.485) / 0.229 ≈ 0.074
        let dst_f32 = dst_dptr as *const f32;
        let val = unsafe { *dst_f32 };
        eprintln!("Fused output[0] = {} (expected ~0.074)", val);
        assert!((val - 0.074).abs() < 0.1, "fused normalize output wrong: {}", val);

        cuda_ffi::mem_free(src_dptr).unwrap();
        cuda_ffi::mem_free(dst_dptr).unwrap();
    }
}
