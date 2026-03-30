//! FFI wrappers for TensorPool and pool-backed types (Image, PointCloud).

use std::sync::Arc;

use horus_core::memory::{TensorPool, TensorPoolConfig, Image, PointCloud};
use horus_core::types::{Tensor, TensorDtype, Device};

/// Opaque TensorPool handle for C.
pub struct FfiTensorPool {
    inner: Arc<TensorPool>,
}

/// Opaque Tensor handle for C.
pub struct FfiTensor {
    inner: Tensor,
    pool: Arc<TensorPool>,
}

/// Opaque Image handle for C.
pub struct FfiImage {
    inner: Image,
}

// ─── TensorPool ──────────────────────────────────────────────────────────────

/// Create a new TensorPool. Returns null on error.
/// pool_id: unique identifier. pool_size_bytes: total data region. max_slots: max concurrent tensors.
pub fn tensor_pool_new(pool_id: u32, pool_size_bytes: usize, max_slots: usize) -> Option<Box<FfiTensorPool>> {
    let config = TensorPoolConfig {
        pool_size: pool_size_bytes,
        max_slots,
        ..TensorPoolConfig::default()
    };
    TensorPool::new(pool_id, config).ok().map(|pool| {
        Box::new(FfiTensorPool { inner: Arc::new(pool) })
    })
}

/// Destroy a TensorPool.
pub fn tensor_pool_destroy(_pool: Box<FfiTensorPool>) {
    // Drop releases the Arc
}

/// Get pool statistics: (allocated_slots, total_refcount, used_bytes, free_bytes).
pub fn tensor_pool_stats(pool: &FfiTensorPool) -> (usize, u64, usize, usize) {
    let s = pool.inner.stats();
    (s.allocated_slots, s.total_refcount, s.used_bytes, s.free_bytes)
}

// ─── Tensor ──────────────────────────────────────────────────────────────────

/// Allocate a tensor from the pool. shape is [dim0, dim1, ...], ndim elements.
/// dtype: 0=f32, 1=f64, 2=u8, 3=i32. Returns null on error.
pub fn tensor_alloc(
    pool: &FfiTensorPool,
    shape: &[u64],
    dtype: u8,
) -> Option<Box<FfiTensor>> {
    let dt = match dtype {
        0 => TensorDtype::F32,
        1 => TensorDtype::F64,
        2 => TensorDtype::U8,
        3 => TensorDtype::I32,
        _ => return None,
    };
    pool.inner.alloc(shape, dt, Device::cpu()).ok().map(|tensor| {
        Box::new(FfiTensor { inner: tensor, pool: pool.inner.clone() })
    })
}

/// Get raw data pointer for a tensor.
pub fn tensor_data_ptr(pool: &FfiTensorPool, tensor: &FfiTensor) -> *mut u8 {
    pool.inner.data_ptr(&tensor.inner)
}

/// Get tensor size in bytes.
pub fn tensor_nbytes(tensor: &FfiTensor) -> u64 {
    tensor.inner.nbytes()
}

/// Release a tensor back to the pool.
pub fn tensor_release(pool: &FfiTensorPool, tensor: &FfiTensor) {
    pool.inner.release(&tensor.inner);
}

// ─── Image ───────────────────────────────────────────────────────────────────

/// Allocate an Image from the pool.
/// encoding: 0=RGB8, 1=RGBA8, 2=GRAY8, 3=BGR8.
pub fn image_new(
    pool: &FfiTensorPool,
    width: u32,
    height: u32,
    encoding: u8,
) -> Option<Box<FfiImage>> {
    use horus_core::types::ImageEncoding;
    let enc = match encoding {
        0 => ImageEncoding::Rgb8,
        1 => ImageEncoding::Rgba8,
        2 => ImageEncoding::Mono8,
        3 => ImageEncoding::Bgr8,
        _ => return None,
    };
    Image::new_on(width, height, enc, pool.inner.clone()).ok().map(|img| {
        Box::new(FfiImage { inner: img })
    })
}

/// Get image width * height * channels in bytes.
pub fn image_data_size(img: &FfiImage) -> usize {
    (img.inner.width() * img.inner.height() * img.inner.channels()) as usize
}

/// Get image width.
pub fn image_width(img: &FfiImage) -> u32 {
    img.inner.width()
}

/// Get image height.
pub fn image_height(img: &FfiImage) -> u32 {
    img.inner.height()
}

/// Destroy an image (releases pool slot).
pub fn image_destroy(_img: Box<FfiImage>) {
    // Drop releases
}

// ─── Tests ───────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    use std::sync::atomic::{AtomicU32, Ordering};
    static POOL_ID: AtomicU32 = AtomicU32::new(0);
    fn next_pool_id() -> u32 {
        // PID-based to avoid SHM collisions between test runs
        let base = (std::process::id() % 10000) * 100;
        base + POOL_ID.fetch_add(1, Ordering::Relaxed)
    }

    #[test]
    fn tensor_pool_create_destroy() {
        let pool = tensor_pool_new(next_pool_id(), 1024 * 1024, 64);
        assert!(pool.is_some());
    }

    #[test]
    fn tensor_alloc_release() {
        let pool = tensor_pool_new(next_pool_id(), 1024 * 1024, 64).unwrap();
        let tensor = tensor_alloc(&pool, &[640, 480, 3], 2); // u8
        assert!(tensor.is_some());
        let t = tensor.unwrap();
        assert_eq!(tensor_nbytes(&t), 640 * 480 * 3);

        let ptr = tensor_data_ptr(&pool, &t);
        assert!(!ptr.is_null());

        tensor_release(&pool, &t);
    }

    #[test]
    fn image_create_access() {
        let pool = tensor_pool_new(next_pool_id(), 4 * 1024 * 1024, 64).unwrap();
        let img = image_new(&pool, 320, 240, 0); // RGB8
        assert!(img.is_some());
        let img = img.unwrap();
        assert_eq!(image_width(&img), 320);
        assert_eq!(image_height(&img), 240);
        assert_eq!(image_data_size(&img), 320 * 240 * 3);
    }

    #[test]
    fn pool_stats() {
        let pool = tensor_pool_new(next_pool_id(), 1024 * 1024, 64).unwrap();
        let (alloc, _refcount, _used, free) = tensor_pool_stats(&pool);
        assert_eq!(alloc, 0);
        assert!(free > 0);

        let _t = tensor_alloc(&pool, &[100], 0).unwrap(); // f32
        let (alloc2, _, _, _) = tensor_pool_stats(&pool);
        assert!(alloc2 >= 1);
    }
}
