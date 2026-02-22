//! # SIMD-accelerated memory operations for HORUS
//!
//! This module provides high-performance memory copy operations using AVX2
//! SIMD instructions for large data transfers in shared memory.
//!
//! ## Performance Benefits
//!
//! - **Streaming stores**: Non-temporal stores bypass the CPU cache,
//!   ideal for write-once patterns (camera frames, LiDAR scans)
//! - **Wide operations**: Copy 256 bits (32 bytes) per instruction
//! - **Prefetching**: Hardware prefetch hints on reads for better bandwidth
//!
//! ## Where SIMD is Used
//!
//! Perception types that handle large buffers (images, point clouds, depth maps):
//! - `Image::copy_from()` — e.g. 640×480 RGB8 = 921KB at 30fps
//! - `PointCloud::copy_from()` — e.g. 100K points = 1.2MB at 10Hz
//! - `DepthImage::copy_from()` — e.g. 640×480 F32 = 1.2MB at 30fps
//! - Topic send/recv for messages larger than `SIMD_COPY_THRESHOLD`
//!
//! ## Runtime Detection
//!
//! The module automatically detects CPU capabilities at runtime and selects
//! the best available implementation:
//! - AVX2: 256-bit operations (most modern x86-64 CPUs)
//! - Fallback: Standard library copy (always works)

use std::sync::OnceLock;

// ============================================================================
// SIMD THRESHOLDS AND CONSTANTS
// ============================================================================

/// Threshold in bytes above which SIMD copy is beneficial.
/// Below this size, the setup overhead exceeds the benefit.
pub const SIMD_COPY_THRESHOLD: usize = 4096; // 4KB

/// Alignment requirement for SIMD operations (AVX2 = 32 bytes)
const SIMD_ALIGNMENT: usize = 32;

// Runtime feature detection — initialized exactly once, race-free
static AVX2_AVAILABLE: OnceLock<bool> = OnceLock::new();

/// Check if AVX2 is available on the current CPU
#[inline]
fn is_avx2_available() -> bool {
    *AVX2_AVAILABLE.get_or_init(|| {
        #[cfg(target_arch = "x86_64")]
        {
            is_x86_feature_detected!("avx2")
        }
        #[cfg(not(target_arch = "x86_64"))]
        {
            false
        }
    })
}

/// Copy data TO shared memory using SIMD streaming stores.
///
/// This function uses non-temporal stores (_mm256_stream_si256) which bypass
/// the CPU cache, making it ideal for write-once scenarios where the data
/// won't be read again by the writer.
///
/// # Arguments
/// * `src` - Source pointer (should be readable for `len` bytes)
/// * `dst` - Destination pointer (should be writable for `len` bytes)
/// * `len` - Number of bytes to copy
///
/// # Safety
/// - Both `src` and `dst` must be valid for reads/writes of `len` bytes
/// - `src` and `dst` must not overlap
/// - Caller must ensure proper lifetime of memory regions
///
/// # Performance
/// - For buffers < 4KB: Falls back to standard copy (setup overhead)
/// - For buffers >= 4KB: Uses AVX2 streaming stores
/// - Automatically detects CPU capabilities at runtime
#[inline]
pub unsafe fn simd_copy_to_shm(src: *const u8, dst: *mut u8, len: usize) {
    if len < SIMD_COPY_THRESHOLD {
        std::ptr::copy_nonoverlapping(src, dst, len);
        return;
    }

    #[cfg(target_arch = "x86_64")]
    {
        if is_avx2_available() {
            simd_copy_to_shm_avx2(src, dst, len);
            return;
        }
    }

    // Fallback: standard copy
    std::ptr::copy_nonoverlapping(src, dst, len);
}

/// Copy data FROM shared memory using SIMD with prefetching.
///
/// Uses regular SIMD loads with software prefetching (512 bytes ahead)
/// to hide memory latency when reading from cross-process shared memory.
///
/// # Arguments
/// * `src` - Source pointer in shared memory
/// * `dst` - Destination pointer (local memory)
/// * `len` - Number of bytes to copy
///
/// # Safety
/// Same as `simd_copy_to_shm`
#[inline]
pub unsafe fn simd_copy_from_shm(src: *const u8, dst: *mut u8, len: usize) {
    if len < SIMD_COPY_THRESHOLD {
        std::ptr::copy_nonoverlapping(src, dst, len);
        return;
    }

    #[cfg(target_arch = "x86_64")]
    {
        if is_avx2_available() {
            simd_copy_from_shm_avx2(src, dst, len);
            return;
        }
    }

    // Fallback: standard copy
    std::ptr::copy_nonoverlapping(src, dst, len);
}

// ============================================================================
// SAFE SLICE-BASED WRAPPERS
// ============================================================================

/// Copy `src` into `dst` using SIMD-accelerated streaming stores when beneficial.
///
/// Safe wrapper around `simd_copy_to_shm`. For buffers >= 4KB on x86-64 with
/// AVX2, uses non-temporal stores that bypass the CPU cache — ideal for
/// write-once patterns like loading camera frames into shared memory.
///
/// # Panics
///
/// Panics if `src.len() != dst.len()`.
#[inline]
pub fn fast_copy_to_shm(src: &[u8], dst: &mut [u8]) {
    assert_eq!(
        src.len(),
        dst.len(),
        "fast_copy_to_shm: src ({}) and dst ({}) must be same length",
        src.len(),
        dst.len()
    );
    // SAFETY: src and dst are valid, non-overlapping slices of equal length.
    // &[u8] and &mut [u8] guarantee non-overlap by Rust's borrow rules.
    unsafe {
        simd_copy_to_shm(src.as_ptr(), dst.as_mut_ptr(), src.len());
    }
}

// ============================================================================
// AVX2 IMPLEMENTATIONS
// ============================================================================

/// AVX2 implementation of streaming copy TO shared memory.
///
/// Uses _mm256_stream_si256 for non-temporal stores that bypass cache.
#[cfg(target_arch = "x86_64")]
#[target_feature(enable = "avx2")]
unsafe fn simd_copy_to_shm_avx2(src: *const u8, dst: *mut u8, len: usize) {
    use std::arch::x86_64::*;

    let mut src_ptr = src;
    let mut dst_ptr = dst;
    let mut remaining = len;

    // Handle unaligned prefix (copy bytes until dst is 32-byte aligned)
    let dst_addr = dst_ptr as usize;
    let unaligned_prefix = (SIMD_ALIGNMENT - (dst_addr % SIMD_ALIGNMENT)) % SIMD_ALIGNMENT;

    if unaligned_prefix > 0 && unaligned_prefix <= remaining {
        std::ptr::copy_nonoverlapping(src_ptr, dst_ptr, unaligned_prefix);
        src_ptr = src_ptr.add(unaligned_prefix);
        dst_ptr = dst_ptr.add(unaligned_prefix);
        remaining -= unaligned_prefix;
    }

    // Main SIMD loop: 256 bits (32 bytes) per iteration
    // Process 4 vectors (128 bytes) per loop for better instruction-level parallelism
    const VECTOR_SIZE: usize = 32;
    const UNROLL_FACTOR: usize = 4;
    const CHUNK_SIZE: usize = VECTOR_SIZE * UNROLL_FACTOR;

    while remaining >= CHUNK_SIZE {
        // Load 4 vectors from source (unaligned load is fine for reads)
        let v0 = _mm256_loadu_si256(src_ptr as *const __m256i);
        let v1 = _mm256_loadu_si256(src_ptr.add(VECTOR_SIZE) as *const __m256i);
        let v2 = _mm256_loadu_si256(src_ptr.add(VECTOR_SIZE * 2) as *const __m256i);
        let v3 = _mm256_loadu_si256(src_ptr.add(VECTOR_SIZE * 3) as *const __m256i);

        // Store using streaming (non-temporal) stores to bypass cache
        _mm256_stream_si256(dst_ptr as *mut __m256i, v0);
        _mm256_stream_si256(dst_ptr.add(VECTOR_SIZE) as *mut __m256i, v1);
        _mm256_stream_si256(dst_ptr.add(VECTOR_SIZE * 2) as *mut __m256i, v2);
        _mm256_stream_si256(dst_ptr.add(VECTOR_SIZE * 3) as *mut __m256i, v3);

        src_ptr = src_ptr.add(CHUNK_SIZE);
        dst_ptr = dst_ptr.add(CHUNK_SIZE);
        remaining -= CHUNK_SIZE;
    }

    // Handle remaining full vectors
    while remaining >= VECTOR_SIZE {
        let v = _mm256_loadu_si256(src_ptr as *const __m256i);
        _mm256_stream_si256(dst_ptr as *mut __m256i, v);

        src_ptr = src_ptr.add(VECTOR_SIZE);
        dst_ptr = dst_ptr.add(VECTOR_SIZE);
        remaining -= VECTOR_SIZE;
    }

    // Memory fence to ensure streaming stores are visible
    _mm_sfence();

    // Handle remaining bytes (< 32)
    if remaining > 0 {
        std::ptr::copy_nonoverlapping(src_ptr, dst_ptr, remaining);
    }
}

/// AVX2 implementation of copy FROM shared memory.
///
/// Uses regular loads (with prefetching) since data may be used multiple times.
#[cfg(target_arch = "x86_64")]
#[target_feature(enable = "avx2")]
unsafe fn simd_copy_from_shm_avx2(src: *const u8, dst: *mut u8, len: usize) {
    use std::arch::x86_64::*;

    let mut src_ptr = src;
    let mut dst_ptr = dst;
    let mut remaining = len;

    // Handle unaligned prefix
    let dst_addr = dst_ptr as usize;
    let unaligned_prefix = (SIMD_ALIGNMENT - (dst_addr % SIMD_ALIGNMENT)) % SIMD_ALIGNMENT;

    if unaligned_prefix > 0 && unaligned_prefix <= remaining {
        std::ptr::copy_nonoverlapping(src_ptr, dst_ptr, unaligned_prefix);
        src_ptr = src_ptr.add(unaligned_prefix);
        dst_ptr = dst_ptr.add(unaligned_prefix);
        remaining -= unaligned_prefix;
    }

    const VECTOR_SIZE: usize = 32;
    const UNROLL_FACTOR: usize = 4;
    const CHUNK_SIZE: usize = VECTOR_SIZE * UNROLL_FACTOR;
    const PREFETCH_DISTANCE: usize = 512; // Prefetch 512 bytes ahead

    while remaining >= CHUNK_SIZE {
        // Prefetch future data
        if remaining > PREFETCH_DISTANCE {
            _mm_prefetch(src_ptr.add(PREFETCH_DISTANCE) as *const i8, _MM_HINT_T0);
        }

        // Load 4 vectors from shared memory
        let v0 = _mm256_loadu_si256(src_ptr as *const __m256i);
        let v1 = _mm256_loadu_si256(src_ptr.add(VECTOR_SIZE) as *const __m256i);
        let v2 = _mm256_loadu_si256(src_ptr.add(VECTOR_SIZE * 2) as *const __m256i);
        let v3 = _mm256_loadu_si256(src_ptr.add(VECTOR_SIZE * 3) as *const __m256i);

        // Store to local memory
        _mm256_storeu_si256(dst_ptr as *mut __m256i, v0);
        _mm256_storeu_si256(dst_ptr.add(VECTOR_SIZE) as *mut __m256i, v1);
        _mm256_storeu_si256(dst_ptr.add(VECTOR_SIZE * 2) as *mut __m256i, v2);
        _mm256_storeu_si256(dst_ptr.add(VECTOR_SIZE * 3) as *mut __m256i, v3);

        src_ptr = src_ptr.add(CHUNK_SIZE);
        dst_ptr = dst_ptr.add(CHUNK_SIZE);
        remaining -= CHUNK_SIZE;
    }

    // Handle remaining full vectors
    while remaining >= VECTOR_SIZE {
        let v = _mm256_loadu_si256(src_ptr as *const __m256i);
        _mm256_storeu_si256(dst_ptr as *mut __m256i, v);

        src_ptr = src_ptr.add(VECTOR_SIZE);
        dst_ptr = dst_ptr.add(VECTOR_SIZE);
        remaining -= VECTOR_SIZE;
    }

    // Handle remaining bytes
    if remaining > 0 {
        std::ptr::copy_nonoverlapping(src_ptr, dst_ptr, remaining);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_avx2_detection() {
        // Call multiple times to test caching
        let first = is_avx2_available();
        let second = is_avx2_available();
        assert_eq!(first, second);
    }

    #[test]
    fn test_simd_copy_small() {
        // Small copy should use standard path
        let src = vec![42u8; 100];
        let mut dst = vec![0u8; 100];

        // SAFETY: src and dst are valid, non-overlapping Vec buffers of equal length.
        unsafe {
            simd_copy_to_shm(src.as_ptr(), dst.as_mut_ptr(), 100);
        }

        assert_eq!(src, dst);
    }

    #[test]
    fn test_simd_copy_large() {
        // Large copy (> 4KB) should use SIMD path if available
        let size = 64 * 1024; // 64KB
        let src: Vec<u8> = (0..size).map(|i| (i % 256) as u8).collect();
        let mut dst = vec![0u8; size];

        // SAFETY: src and dst are valid, non-overlapping Vec buffers of equal length.
        unsafe {
            simd_copy_to_shm(src.as_ptr(), dst.as_mut_ptr(), size);
        }

        assert_eq!(src, dst);
    }

    #[test]
    fn test_simd_copy_from_shm() {
        let size = 32 * 1024; // 32KB
        let src: Vec<u8> = (0..size).map(|i| (i % 256) as u8).collect();
        let mut dst = vec![0u8; size];

        // SAFETY: src and dst are valid, non-overlapping Vec buffers of equal length.
        unsafe {
            simd_copy_from_shm(src.as_ptr(), dst.as_mut_ptr(), size);
        }

        assert_eq!(src, dst);
    }

    #[test]
    fn test_simd_copy_unaligned() {
        // Test with unaligned pointers
        let size = 8 * 1024 + 17; // Non-round size
        let src: Vec<u8> = (0..size).map(|i| (i % 256) as u8).collect();
        let mut dst = vec![0u8; size];

        // Copy starting at offset 3 (unaligned)
        let offset = 3;
        // SAFETY: src and dst are valid, non-overlapping, and offset range is within bounds.
        unsafe {
            simd_copy_to_shm(
                src.as_ptr().add(offset),
                dst.as_mut_ptr().add(offset),
                size - offset,
            );
        }

        assert_eq!(&src[offset..], &dst[offset..]);
    }

    #[test]
    fn test_fast_copy_to_shm_small() {
        let src = vec![99u8; 256];
        let mut dst = vec![0u8; 256];
        fast_copy_to_shm(&src, &mut dst);
        assert_eq!(src, dst);
    }

    #[test]
    fn test_fast_copy_to_shm_large() {
        // Simulate a 640x480 grayscale image (307,200 bytes)
        let size = 640 * 480;
        let src: Vec<u8> = (0..size).map(|i| (i % 256) as u8).collect();
        let mut dst = vec![0u8; size];
        fast_copy_to_shm(&src, &mut dst);
        assert_eq!(src, dst);
    }

    #[test]
    #[should_panic(expected = "must be same length")]
    fn test_fast_copy_to_shm_mismatched_panics() {
        let src = vec![0u8; 100];
        let mut dst = vec![0u8; 200];
        fast_copy_to_shm(&src, &mut dst);
    }

    #[test]
    fn test_simd_copy_exact_chunk_size() {
        // Test with size that's exactly a multiple of chunk size (128 bytes)
        let size = 128 * 100; // 12,800 bytes
        let src: Vec<u8> = (0..size).map(|i| (i % 256) as u8).collect();
        let mut dst = vec![0u8; size];

        // SAFETY: src and dst are valid, non-overlapping Vec buffers of equal length.
        unsafe {
            simd_copy_to_shm(src.as_ptr(), dst.as_mut_ptr(), size);
        }

        assert_eq!(src, dst);
    }
}
