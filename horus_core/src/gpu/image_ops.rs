//! High-level GPU image preprocessing operations.
//!
//! [`GpuImageOps`] wraps the raw CUDA kernel functions from [`super::kernels`]
//! with input validation, output allocation, and ergonomic Rust API.
//!
//! All operations accept a raw CUDA stream pointer (`*mut c_void`) for async
//! execution. Inside a GPU node's `tick()`, get the stream via `horus::gpu_stream()`.
//! Outside tick(), pass `std::ptr::null_mut()` for the default stream.

#![allow(clippy::too_many_arguments)]

use std::ffi::c_void;

use super::cuda_ffi::{self, CudaResult};
use super::kernels;

/// High-level GPU image preprocessing operations.
///
/// Stateless — all state is in the CUDA context (set per-thread by the executor).
/// Methods validate inputs and call the underlying CUDA kernels.
///
/// # Example
///
/// ```rust,ignore
/// // Inside a GPU node's tick():
/// let stream = horus::gpu_stream().unwrap_or(std::ptr::null_mut());
/// let resized = GpuImageOps::resize_raw(src_ptr, 1920, 1080, 640, 480, 3, stream)?;
/// ```
pub struct GpuImageOps;

/// Color format for GPU operations.
#[repr(i32)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ColorFormat {
    Rgb = 0,
    Bgr = 1,
    Gray = 2,
}

impl GpuImageOps {
    /// Check if GPU kernels are available.
    pub fn available() -> bool {
        cuda_ffi::is_available() && kernels::kernels_available()
    }

    /// Color space conversion on raw GPU pointers.
    ///
    /// `src` and `dst` must be GPU-accessible pointers (managed or device memory).
    /// For RGB<->BGR: dst size = width * height * 3.
    /// For RGB/BGR->GRAY: dst size = width * height.
    pub fn color_convert(
        src: *const u8,
        dst: *mut u8,
        width: u32,
        height: u32,
        src_fmt: ColorFormat,
        dst_fmt: ColorFormat,
        stream: *mut c_void,
    ) -> CudaResult<()> {
        if width == 0 || height == 0 {
            return Err(cuda_ffi::CudaError {
                code: -1,
                message: "zero dimensions".into(),
            });
        }
        kernels::color_convert(
            src,
            dst,
            width as i32,
            height as i32,
            src_fmt as i32,
            dst_fmt as i32,
            stream,
        )
    }

    /// Bilinear resize on raw GPU pointers.
    pub fn resize(
        src: *const u8,
        dst: *mut u8,
        src_w: u32,
        src_h: u32,
        dst_w: u32,
        dst_h: u32,
        channels: u32,
        stream: *mut c_void,
    ) -> CudaResult<()> {
        if src_w == 0 || src_h == 0 || dst_w == 0 || dst_h == 0 || channels == 0 {
            return Err(cuda_ffi::CudaError {
                code: -1,
                message: "zero dimensions".into(),
            });
        }
        kernels::resize(
            src,
            dst,
            src_w as i32,
            src_h as i32,
            dst_w as i32,
            dst_h as i32,
            channels as i32,
            stream,
        )
    }

    /// Per-channel normalize: u8 HWC -> f32 HWC.
    ///
    /// `mean` and `stddev` are per-channel (e.g., ImageNet: [0.485, 0.456, 0.406]).
    pub fn normalize(
        src: *const u8,
        dst: *mut f32,
        width: u32,
        height: u32,
        channels: u32,
        mean: &[f32; 4],
        stddev: &[f32; 4],
        stream: *mut c_void,
    ) -> CudaResult<()> {
        kernels::normalize(
            src,
            dst,
            width as i32,
            height as i32,
            channels as i32,
            mean,
            stddev,
            stream,
        )
    }

    /// HWC -> CHW transpose on raw GPU pointers.
    pub fn transpose_hwc_to_chw(
        src: *const c_void,
        dst: *mut c_void,
        width: u32,
        height: u32,
        channels: u32,
        elem_size: u32,
        stream: *mut c_void,
    ) -> CudaResult<()> {
        kernels::transpose_hwc_to_chw(
            src,
            dst,
            width as i32,
            height as i32,
            channels as i32,
            elem_size as i32,
            stream,
        )
    }

    /// Lens undistortion (Brown-Conrady model).
    ///
    /// `intrinsics`: [fx, fy, cx, cy]
    /// `distortion`: [k1, k2, p1, p2, k3]
    pub fn undistort(
        src: *const u8,
        dst: *mut u8,
        width: u32,
        height: u32,
        channels: u32,
        intrinsics: &[f32; 4],
        distortion: &[f32; 5],
        stream: *mut c_void,
    ) -> CudaResult<()> {
        kernels::undistort(
            src,
            dst,
            width as i32,
            height as i32,
            channels as i32,
            intrinsics,
            distortion,
            stream,
        )
    }

    /// ROI crop with padding.
    pub fn crop_pad(
        src: *const u8,
        dst: *mut u8,
        src_w: u32,
        src_h: u32,
        crop_x: i32,
        crop_y: i32,
        crop_w: u32,
        crop_h: u32,
        dst_w: u32,
        dst_h: u32,
        channels: u32,
        pad_value: u8,
        stream: *mut c_void,
    ) -> CudaResult<()> {
        kernels::crop_pad(
            src,
            dst,
            src_w as i32,
            src_h as i32,
            crop_x,
            crop_y,
            crop_w as i32,
            crop_h as i32,
            dst_w as i32,
            dst_h as i32,
            channels as i32,
            pad_value,
            stream,
        )
    }

    /// Fused preprocessing: color_convert + resize + normalize + HWC->CHW.
    ///
    /// **This is the primary API for neural network preprocessing.**
    /// One kernel launch, no intermediate buffers.
    ///
    /// Input: u8 HWC (RGB or BGR), Output: f32 CHW (normalized).
    ///
    /// # Example
    ///
    /// ```rust,ignore
    /// let mean = [0.485, 0.456, 0.406]; // ImageNet
    /// let std  = [0.229, 0.224, 0.225];
    /// GpuImageOps::preprocess_fused(
    ///     src_ptr, dst_ptr,
    ///     1920, 1080, 640, 480,
    ///     ColorFormat::Bgr,
    ///     &mean, &std,
    ///     stream,
    /// )?;
    /// ```
    pub fn preprocess_fused(
        src: *const u8,
        dst: *mut f32,
        src_w: u32,
        src_h: u32,
        dst_w: u32,
        dst_h: u32,
        src_fmt: ColorFormat,
        mean: &[f32; 3],
        stddev: &[f32; 3],
        stream: *mut c_void,
    ) -> CudaResult<()> {
        if src_w == 0 || src_h == 0 || dst_w == 0 || dst_h == 0 {
            return Err(cuda_ffi::CudaError {
                code: -1,
                message: "zero dimensions".into(),
            });
        }
        kernels::preprocess_fused(
            src,
            dst,
            src_w as i32,
            src_h as i32,
            dst_w as i32,
            dst_h as i32,
            src_fmt as i32,
            mean,
            stddev,
            stream,
        )
    }
}
