/*
 * HORUS GPU Preprocessing Kernels — C API
 *
 * All functions accept a CUDA stream for async execution.
 * Stream parameter is `void*` (actually CUstream) for C ABI compatibility.
 * Return 0 on success, non-zero CUDA error code on failure.
 */

#ifndef HORUS_KERNELS_H
#define HORUS_KERNELS_H

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Color space conversion.
 * src_fmt/dst_fmt: 0=RGB, 1=BGR, 2=GRAY, 3=NV12
 */
int horus_color_convert(
    const uint8_t* src, uint8_t* dst,
    int width, int height,
    int src_fmt, int dst_fmt,
    void* stream);

/* Bilinear resize.
 * channels: 1, 3, or 4
 */
int horus_resize(
    const uint8_t* src, uint8_t* dst,
    int src_w, int src_h,
    int dst_w, int dst_h,
    int channels,
    void* stream);

/* Per-channel normalize: out[c] = (in[c]/255.0 - mean[c]) / std[c]
 * Input: u8 HWC, Output: f32 HWC
 */
int horus_normalize(
    const uint8_t* src, float* dst,
    int width, int height, int channels,
    const float mean[4], const float stddev[4],
    void* stream);

/* HWC <-> CHW transpose.
 * elem_size: 1 for u8, 4 for f32
 */
int horus_transpose_hwc_to_chw(
    const void* src, void* dst,
    int width, int height, int channels, int elem_size,
    void* stream);

int horus_transpose_chw_to_hwc(
    const void* src, void* dst,
    int width, int height, int channels, int elem_size,
    void* stream);

/* Lens undistortion (Brown-Conrady model).
 * K: [fx, fy, cx, cy] (camera intrinsics)
 * dist: [k1, k2, p1, p2, k3] (distortion coefficients)
 */
int horus_undistort(
    const uint8_t* src, uint8_t* dst,
    int width, int height, int channels,
    const float K[4], const float dist[5],
    void* stream);

/* ROI crop with optional padding (letterbox).
 * If crop region extends past source, filled with pad_value.
 */
int horus_crop_pad(
    const uint8_t* src, uint8_t* dst,
    int src_w, int src_h,
    int crop_x, int crop_y, int crop_w, int crop_h,
    int dst_w, int dst_h,
    int channels, uint8_t pad_value,
    void* stream);

/* Fused preprocessing: color_convert + resize + normalize + HWC->CHW
 * Input: u8 HWC (src_fmt encoding), Output: f32 CHW normalized
 * One kernel launch, no intermediate buffers.
 * src_fmt: 0=RGB, 1=BGR
 */
int horus_preprocess_fused(
    const uint8_t* src, float* dst,
    int src_w, int src_h,
    int dst_w, int dst_h,
    int src_fmt,
    const float mean[3], const float stddev[3],
    void* stream);

#ifdef __cplusplus
}
#endif

#endif /* HORUS_KERNELS_H */
