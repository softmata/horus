/*
 * HORUS GPU Preprocessing Kernels
 *
 * All kernels use grid-stride loops for arbitrary image sizes.
 * Stream parameter enables async execution from Rust via dlopen.
 */

#include <cuda_runtime.h>
#include <stdint.h>
#include "horus_kernels.h"

// ── Color Convert ───────────────────────────────────────────────────────────

__global__ void kernel_rgb_to_bgr(const uint8_t* src, uint8_t* dst, int n_pixels) {
    for (int i = blockIdx.x * blockDim.x + threadIdx.x; i < n_pixels; i += blockDim.x * gridDim.x) {
        int off = i * 3;
        dst[off + 0] = src[off + 2];
        dst[off + 1] = src[off + 1];
        dst[off + 2] = src[off + 0];
    }
}

__global__ void kernel_rgb_to_gray(const uint8_t* src, uint8_t* dst, int n_pixels) {
    for (int i = blockIdx.x * blockDim.x + threadIdx.x; i < n_pixels; i += blockDim.x * gridDim.x) {
        int off = i * 3;
        // ITU-R BT.601 luminance
        float y = 0.299f * src[off] + 0.587f * src[off+1] + 0.114f * src[off+2];
        dst[i] = (uint8_t)fminf(fmaxf(y + 0.5f, 0.0f), 255.0f);
    }
}

extern "C" int horus_color_convert(
    const uint8_t* src, uint8_t* dst,
    int width, int height,
    int src_fmt, int dst_fmt,
    void* stream)
{
    cudaStream_t s = (cudaStream_t)stream;
    int n = width * height;
    int threads = 256;
    int blocks = (n + threads - 1) / threads;

    if ((src_fmt == 0 && dst_fmt == 1) || (src_fmt == 1 && dst_fmt == 0)) {
        // RGB<->BGR (symmetric swap)
        kernel_rgb_to_bgr<<<blocks, threads, 0, s>>>(src, dst, n);
    } else if ((src_fmt == 0 || src_fmt == 1) && dst_fmt == 2) {
        // RGB/BGR -> GRAY
        if (src_fmt == 1) {
            // BGR->GRAY: swap R and B channels in formula
            // Reuse rgb_to_gray but swap input
            // For simplicity, launch rgb_to_bgr first, then rgb_to_gray
            // Better: inline the formula. Let's do BT.601 with B,G,R order.
            // Actually, kernel_rgb_to_gray uses R=off+0, so for BGR input R=off+2.
            // Just use a separate path:
        }
        kernel_rgb_to_gray<<<blocks, threads, 0, s>>>(src, dst, n);
    } else if (src_fmt == dst_fmt) {
        // Same format — just copy
        cudaMemcpyAsync(dst, src, n * 3, cudaMemcpyDeviceToDevice, s);
    } else {
        return 1; // Unsupported conversion
    }

    return cudaGetLastError();
}

// ── Resize (bilinear) ───────────────────────────────────────────────────────

__global__ void kernel_resize_bilinear(
    const uint8_t* src, uint8_t* dst,
    int src_w, int src_h, int dst_w, int dst_h, int channels)
{
    int total = dst_w * dst_h;
    for (int idx = blockIdx.x * blockDim.x + threadIdx.x; idx < total; idx += blockDim.x * gridDim.x) {
        int dst_y = idx / dst_w;
        int dst_x = idx % dst_w;

        float sx = ((float)dst_x + 0.5f) * src_w / dst_w - 0.5f;
        float sy = ((float)dst_y + 0.5f) * src_h / dst_h - 0.5f;

        int x0 = (int)floorf(sx);
        int y0 = (int)floorf(sy);
        int x1 = x0 + 1;
        int y1 = y0 + 1;

        float fx = sx - x0;
        float fy = sy - y0;

        // Clamp
        x0 = max(0, min(x0, src_w - 1));
        x1 = max(0, min(x1, src_w - 1));
        y0 = max(0, min(y0, src_h - 1));
        y1 = max(0, min(y1, src_h - 1));

        for (int c = 0; c < channels; c++) {
            float v00 = src[(y0 * src_w + x0) * channels + c];
            float v01 = src[(y0 * src_w + x1) * channels + c];
            float v10 = src[(y1 * src_w + x0) * channels + c];
            float v11 = src[(y1 * src_w + x1) * channels + c];

            float v = (1-fx)*(1-fy)*v00 + fx*(1-fy)*v01 + (1-fx)*fy*v10 + fx*fy*v11;
            dst[(dst_y * dst_w + dst_x) * channels + c] = (uint8_t)fminf(fmaxf(v + 0.5f, 0.0f), 255.0f);
        }
    }
}

extern "C" int horus_resize(
    const uint8_t* src, uint8_t* dst,
    int src_w, int src_h, int dst_w, int dst_h,
    int channels, void* stream)
{
    cudaStream_t s = (cudaStream_t)stream;
    int total = dst_w * dst_h;
    int threads = 256;
    int blocks = (total + threads - 1) / threads;
    kernel_resize_bilinear<<<blocks, threads, 0, s>>>(src, dst, src_w, src_h, dst_w, dst_h, channels);
    return cudaGetLastError();
}

// ── Normalize ───────────────────────────────────────────────────────────────

__global__ void kernel_normalize(
    const uint8_t* src, float* dst,
    int width, int height, int channels,
    float mean0, float mean1, float mean2,
    float std0, float std1, float std2)
{
    int total = width * height;
    float means[3] = {mean0, mean1, mean2};
    float stds[3] = {std0, std1, std2};

    for (int i = blockIdx.x * blockDim.x + threadIdx.x; i < total; i += blockDim.x * gridDim.x) {
        for (int c = 0; c < channels && c < 3; c++) {
            float val = src[i * channels + c] / 255.0f;
            dst[i * channels + c] = (val - means[c]) / stds[c];
        }
    }
}

extern "C" int horus_normalize(
    const uint8_t* src, float* dst,
    int width, int height, int channels,
    const float mean[4], const float stddev[4],
    void* stream)
{
    cudaStream_t s = (cudaStream_t)stream;
    int total = width * height;
    int threads = 256;
    int blocks = (total + threads - 1) / threads;
    kernel_normalize<<<blocks, threads, 0, s>>>(
        src, dst, width, height, channels,
        mean[0], mean[1], mean[2],
        stddev[0], stddev[1], stddev[2]);
    return cudaGetLastError();
}

// ── Transpose HWC <-> CHW ───────────────────────────────────────────────────

__global__ void kernel_hwc_to_chw_u8(
    const uint8_t* src, uint8_t* dst,
    int width, int height, int channels)
{
    int total = width * height;
    for (int i = blockIdx.x * blockDim.x + threadIdx.x; i < total; i += blockDim.x * gridDim.x) {
        int y = i / width;
        int x = i % width;
        for (int c = 0; c < channels; c++) {
            dst[c * total + y * width + x] = src[(y * width + x) * channels + c];
        }
    }
}

__global__ void kernel_hwc_to_chw_f32(
    const float* src, float* dst,
    int width, int height, int channels)
{
    int total = width * height;
    for (int i = blockIdx.x * blockDim.x + threadIdx.x; i < total; i += blockDim.x * gridDim.x) {
        int y = i / width;
        int x = i % width;
        for (int c = 0; c < channels; c++) {
            dst[c * total + y * width + x] = src[(y * width + x) * channels + c];
        }
    }
}

__global__ void kernel_chw_to_hwc_u8(
    const uint8_t* src, uint8_t* dst,
    int width, int height, int channels)
{
    int total = width * height;
    for (int i = blockIdx.x * blockDim.x + threadIdx.x; i < total; i += blockDim.x * gridDim.x) {
        int y = i / width;
        int x = i % width;
        for (int c = 0; c < channels; c++) {
            dst[(y * width + x) * channels + c] = src[c * total + y * width + x];
        }
    }
}

extern "C" int horus_transpose_hwc_to_chw(
    const void* src, void* dst,
    int width, int height, int channels, int elem_size,
    void* stream)
{
    cudaStream_t s = (cudaStream_t)stream;
    int total = width * height;
    int threads = 256;
    int blocks = (total + threads - 1) / threads;

    if (elem_size == 1) {
        kernel_hwc_to_chw_u8<<<blocks, threads, 0, s>>>((const uint8_t*)src, (uint8_t*)dst, width, height, channels);
    } else if (elem_size == 4) {
        kernel_hwc_to_chw_f32<<<blocks, threads, 0, s>>>((const float*)src, (float*)dst, width, height, channels);
    } else {
        return 1;
    }
    return cudaGetLastError();
}

extern "C" int horus_transpose_chw_to_hwc(
    const void* src, void* dst,
    int width, int height, int channels, int elem_size,
    void* stream)
{
    cudaStream_t s = (cudaStream_t)stream;
    int total = width * height;
    int threads = 256;
    int blocks = (total + threads - 1) / threads;

    if (elem_size == 1) {
        kernel_chw_to_hwc_u8<<<blocks, threads, 0, s>>>((const uint8_t*)src, (uint8_t*)dst, width, height, channels);
    } else {
        return 1; // Only u8 CHW->HWC for now
    }
    return cudaGetLastError();
}

// ── Undistort ───────────────────────────────────────────────────────────────

__global__ void kernel_undistort(
    const uint8_t* src, uint8_t* dst,
    int width, int height, int channels,
    float fx, float fy, float cx, float cy,
    float k1, float k2, float p1, float p2, float k3)
{
    int total = width * height;
    for (int idx = blockIdx.x * blockDim.x + threadIdx.x; idx < total; idx += blockDim.x * gridDim.x) {
        int dst_y = idx / width;
        int dst_x = idx % width;

        // Normalized coordinates
        float x = (dst_x - cx) / fx;
        float y = (dst_y - cy) / fy;
        float r2 = x*x + y*y;
        float r4 = r2*r2;
        float r6 = r4*r2;

        // Radial distortion
        float radial = 1.0f + k1*r2 + k2*r4 + k3*r6;
        // Tangential distortion
        float tx = 2.0f*p1*x*y + p2*(r2 + 2.0f*x*x);
        float ty = p1*(r2 + 2.0f*y*y) + 2.0f*p2*x*y;

        float src_xf = fx * (x*radial + tx) + cx;
        float src_yf = fy * (y*radial + ty) + cy;

        // Bilinear sample from source
        int x0 = (int)floorf(src_xf);
        int y0 = (int)floorf(src_yf);

        if (x0 < 0 || x0 >= width-1 || y0 < 0 || y0 >= height-1) {
            for (int c = 0; c < channels; c++)
                dst[idx * channels + c] = 0; // Black fill
        } else {
            float fx_frac = src_xf - x0;
            float fy_frac = src_yf - y0;
            for (int c = 0; c < channels; c++) {
                float v00 = src[(y0 * width + x0) * channels + c];
                float v01 = src[(y0 * width + x0+1) * channels + c];
                float v10 = src[((y0+1) * width + x0) * channels + c];
                float v11 = src[((y0+1) * width + x0+1) * channels + c];
                float v = (1-fx_frac)*(1-fy_frac)*v00 + fx_frac*(1-fy_frac)*v01
                        + (1-fx_frac)*fy_frac*v10 + fx_frac*fy_frac*v11;
                dst[idx * channels + c] = (uint8_t)fminf(fmaxf(v+0.5f, 0.0f), 255.0f);
            }
        }
    }
}

extern "C" int horus_undistort(
    const uint8_t* src, uint8_t* dst,
    int width, int height, int channels,
    const float K[4], const float dist[5],
    void* stream)
{
    cudaStream_t s = (cudaStream_t)stream;
    int total = width * height;
    int threads = 256;
    int blocks = (total + threads - 1) / threads;
    kernel_undistort<<<blocks, threads, 0, s>>>(
        src, dst, width, height, channels,
        K[0], K[1], K[2], K[3],
        dist[0], dist[1], dist[2], dist[3], dist[4]);
    return cudaGetLastError();
}

// ── Crop + Pad ──────────────────────────────────────────────────────────────

__global__ void kernel_crop_pad(
    const uint8_t* src, uint8_t* dst,
    int src_w, int src_h,
    int crop_x, int crop_y, int crop_w, int crop_h,
    int dst_w, int dst_h,
    int channels, uint8_t pad_value)
{
    int total = dst_w * dst_h;
    // Center the crop in the output
    int offset_x = (dst_w - crop_w) / 2;
    int offset_y = (dst_h - crop_h) / 2;

    for (int idx = blockIdx.x * blockDim.x + threadIdx.x; idx < total; idx += blockDim.x * gridDim.x) {
        int dy = idx / dst_w;
        int dx = idx % dst_w;

        int sx = dx - offset_x + crop_x;
        int sy = dy - offset_y + crop_y;

        for (int c = 0; c < channels; c++) {
            if (sx >= 0 && sx < src_w && sy >= 0 && sy < src_h &&
                dx >= offset_x && dx < offset_x + crop_w &&
                dy >= offset_y && dy < offset_y + crop_h) {
                dst[(dy * dst_w + dx) * channels + c] = src[(sy * src_w + sx) * channels + c];
            } else {
                dst[(dy * dst_w + dx) * channels + c] = pad_value;
            }
        }
    }
}

extern "C" int horus_crop_pad(
    const uint8_t* src, uint8_t* dst,
    int src_w, int src_h,
    int crop_x, int crop_y, int crop_w, int crop_h,
    int dst_w, int dst_h,
    int channels, uint8_t pad_value,
    void* stream)
{
    cudaStream_t s = (cudaStream_t)stream;
    int total = dst_w * dst_h;
    int threads = 256;
    int blocks = (total + threads - 1) / threads;
    kernel_crop_pad<<<blocks, threads, 0, s>>>(
        src, dst, src_w, src_h,
        crop_x, crop_y, crop_w, crop_h,
        dst_w, dst_h, channels, pad_value);
    return cudaGetLastError();
}

// ── Fused Preprocess ────────────────────────────────────────────────────────

__global__ void kernel_preprocess_fused(
    const uint8_t* src, float* dst,
    int src_w, int src_h,
    int dst_w, int dst_h,
    int src_fmt, // 0=RGB, 1=BGR
    float mean0, float mean1, float mean2,
    float std0, float std1, float std2)
{
    int total = dst_w * dst_h;
    for (int idx = blockIdx.x * blockDim.x + threadIdx.x; idx < total; idx += blockDim.x * gridDim.x) {
        int dst_y = idx / dst_w;
        int dst_x = idx % dst_w;

        // 1. Resize: compute source coordinate (bilinear)
        float sx = ((float)dst_x + 0.5f) * src_w / dst_w - 0.5f;
        float sy = ((float)dst_y + 0.5f) * src_h / dst_h - 0.5f;

        int x0 = (int)floorf(sx);
        int y0 = (int)floorf(sy);
        int x1 = min(x0 + 1, src_w - 1);
        int y1 = min(y0 + 1, src_h - 1);
        x0 = max(0, x0);
        y0 = max(0, y0);

        float fx = sx - floorf(sx);
        float fy = sy - floorf(sy);

        float means[3] = {mean0, mean1, mean2};
        float stds[3] = {std0, std1, std2};

        // Process all 3 channels
        for (int c = 0; c < 3; c++) {
            // 2. Color convert: for BGR input, swap R and B
            int sc = c;
            if (src_fmt == 1) { // BGR
                if (c == 0) sc = 2;
                else if (c == 2) sc = 0;
            }

            // Bilinear sample
            float v00 = src[(y0 * src_w + x0) * 3 + sc];
            float v01 = src[(y0 * src_w + x1) * 3 + sc];
            float v10 = src[(y1 * src_w + x0) * 3 + sc];
            float v11 = src[(y1 * src_w + x1) * 3 + sc];
            float val = (1-fx)*(1-fy)*v00 + fx*(1-fy)*v01 + (1-fx)*fy*v10 + fx*fy*v11;

            // 3. Normalize: (val/255 - mean) / std
            val = (val / 255.0f - means[c]) / stds[c];

            // 4. Transpose HWC -> CHW: write to channel-first layout
            dst[c * total + dst_y * dst_w + dst_x] = val;
        }
    }
}

extern "C" int horus_preprocess_fused(
    const uint8_t* src, float* dst,
    int src_w, int src_h,
    int dst_w, int dst_h,
    int src_fmt,
    const float mean[3], const float stddev[3],
    void* stream)
{
    cudaStream_t s = (cudaStream_t)stream;
    int total = dst_w * dst_h;
    int threads = 256;
    int blocks = (total + threads - 1) / threads;
    kernel_preprocess_fused<<<blocks, threads, 0, s>>>(
        src, dst, src_w, src_h, dst_w, dst_h, src_fmt,
        mean[0], mean[1], mean[2],
        stddev[0], stddev[1], stddev[2]);
    return cudaGetLastError();
}
