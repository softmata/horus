// TensorPool, Tensor, Image, PointCloud — zero-copy pool-backed types.
//
// These wrap SHM-backed memory pools for efficient large-data handling
// (camera images, lidar point clouds, neural network tensors).
//
// Usage:
//   auto pool = horus::TensorPool(1, 16 * 1024 * 1024, 64);
//   auto img  = horus::Image(pool, 640, 480, horus::Encoding::Rgb8);
//   auto pc   = horus::PointCloud(pool, 1000, 3);  // 1000 XYZ points
#pragma once

#include "horus_c.h"
#include <cstdint>
#include <utility>

namespace horus {

// Forward declarations
class Tensor;
class Image;
class PointCloud;

// ── Dtype ──────────────────────────────────────────────────────────────────

enum class Dtype : uint8_t {
    F32 = HORUS_DTYPE_F32,
    F64 = HORUS_DTYPE_F64,
    U8  = HORUS_DTYPE_U8,
    I32 = HORUS_DTYPE_I32,
};

// ── Encoding ───────────────────────────────────────────────────────────────

enum class Encoding : uint8_t {
    Rgb8  = HORUS_ENC_RGB8,
    Rgba8 = HORUS_ENC_RGBA8,
    Gray8 = HORUS_ENC_GRAY8,
    Bgr8  = HORUS_ENC_BGR8,
};

// ── TensorPool ─────────────────────────────────────────────────────────────

class TensorPool {
public:
    TensorPool(uint32_t pool_id, size_t pool_size_bytes, size_t max_slots)
        : handle_(horus_tensor_pool_new(pool_id, pool_size_bytes, max_slots)) {}

    ~TensorPool() { if (handle_) horus_tensor_pool_destroy(handle_); }

    // Move only
    TensorPool(TensorPool&& o) noexcept : handle_(o.handle_) { o.handle_ = nullptr; }
    TensorPool& operator=(TensorPool&& o) noexcept {
        if (this != &o) { if (handle_) horus_tensor_pool_destroy(handle_); handle_ = o.handle_; o.handle_ = nullptr; }
        return *this;
    }
    TensorPool(const TensorPool&) = delete;
    TensorPool& operator=(const TensorPool&) = delete;

    explicit operator bool() const { return handle_ != nullptr; }

    struct Stats { size_t allocated; size_t used_bytes; size_t free_bytes; };
    Stats stats() const {
        Stats s{};
        if (handle_) horus_tensor_pool_stats(handle_, &s.allocated, &s.used_bytes, &s.free_bytes);
        return s;
    }

    HorusTensorPool* raw() const { return handle_; }

private:
    HorusTensorPool* handle_;
};

// ── Tensor ─────────────────────────────────────────────────────────────────

class Tensor {
public:
    Tensor(const TensorPool& pool, const uint64_t* shape, size_t ndim, Dtype dtype)
        : handle_(horus_tensor_alloc(pool.raw(), shape, ndim, static_cast<uint8_t>(dtype)))
        , pool_raw_(pool.raw()) {}

    ~Tensor() { if (handle_) horus_tensor_destroy(handle_); }

    // Move only
    Tensor(Tensor&& o) noexcept : handle_(o.handle_), pool_raw_(o.pool_raw_) { o.handle_ = nullptr; }
    Tensor& operator=(Tensor&& o) noexcept {
        if (this != &o) { if (handle_) horus_tensor_destroy(handle_); handle_ = o.handle_; pool_raw_ = o.pool_raw_; o.handle_ = nullptr; }
        return *this;
    }
    Tensor(const Tensor&) = delete;
    Tensor& operator=(const Tensor&) = delete;

    explicit operator bool() const { return handle_ != nullptr; }

    uint8_t* data() const { return horus_tensor_data_ptr(pool_raw_, handle_); }
    uint64_t nbytes() const { return handle_ ? horus_tensor_nbytes(handle_) : 0; }

    void release() {
        if (handle_ && pool_raw_) horus_tensor_release(pool_raw_, handle_);
    }

private:
    HorusTensor* handle_;
    HorusTensorPool* pool_raw_;
};

// ── Image ──────────────────────────────────────────────────────────────────

class Image {
public:
    Image(const TensorPool& pool, uint32_t width, uint32_t height, Encoding enc = Encoding::Rgb8)
        : handle_(horus_image_new(pool.raw(), width, height, static_cast<uint8_t>(enc))) {}

    ~Image() { if (handle_) horus_image_destroy(handle_); }

    // Move only
    Image(Image&& o) noexcept : handle_(o.handle_) { o.handle_ = nullptr; }
    Image& operator=(Image&& o) noexcept {
        if (this != &o) { if (handle_) horus_image_destroy(handle_); handle_ = o.handle_; o.handle_ = nullptr; }
        return *this;
    }
    Image(const Image&) = delete;
    Image& operator=(const Image&) = delete;

    explicit operator bool() const { return handle_ != nullptr; }

    uint32_t width() const { return handle_ ? horus_image_width(handle_) : 0; }
    uint32_t height() const { return handle_ ? horus_image_height(handle_) : 0; }
    size_t data_size() const { return handle_ ? horus_image_data_size(handle_) : 0; }

private:
    HorusImage* handle_;
};

// ── PointCloud ─────────────────────────────────────────────────────────────

class PointCloud {
public:
    /// fields_per_point: 3=XYZ, 4=XYZI, 6=XYZRGB
    PointCloud(const TensorPool& pool, uint32_t num_points, uint32_t fields_per_point)
        : handle_(horus_pointcloud_new(pool.raw(), num_points, fields_per_point)) {}

    ~PointCloud() { if (handle_) horus_pointcloud_destroy(handle_); }

    // Move only
    PointCloud(PointCloud&& o) noexcept : handle_(o.handle_) { o.handle_ = nullptr; }
    PointCloud& operator=(PointCloud&& o) noexcept {
        if (this != &o) { if (handle_) horus_pointcloud_destroy(handle_); handle_ = o.handle_; o.handle_ = nullptr; }
        return *this;
    }
    PointCloud(const PointCloud&) = delete;
    PointCloud& operator=(const PointCloud&) = delete;

    explicit operator bool() const { return handle_ != nullptr; }

    uint64_t num_points() const { return handle_ ? horus_pointcloud_num_points(handle_) : 0; }
    uint32_t fields_per_point() const { return handle_ ? horus_pointcloud_fields(handle_) : 0; }

private:
    HorusPointCloud* handle_;
};

} // namespace horus
