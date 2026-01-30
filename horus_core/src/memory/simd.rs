//! # SIMD-accelerated memory operations for HORUS
//!
//! This module provides high-performance memory copy operations using AVX2/AVX512
//! SIMD instructions for large message transfers in shared memory.
//!
//! ## Performance Benefits
//!
//! - **Streaming stores**: Use non-temporal stores to avoid cache pollution
//!   for large buffers that won't be read again soon
//! - **Wide operations**: Copy 256/512 bits per instruction vs 64 bits
//! - **Prefetching**: Hardware prefetch hints for better memory bandwidth
//!
//! ## Usage
//!
//! ```rust,ignore
//! use horus_core::memory::simd::{simd_copy_to_shm, simd_copy_from_shm};
//!
//! let src = vec![0u8; 64 * 1024]; // 64KB buffer
//! let dst = vec![0u8; 64 * 1024];
//!
//! // Copy to shared memory (uses streaming stores)
//! unsafe { simd_copy_to_shm(src.as_ptr(), dst.as_mut_ptr(), 64 * 1024); }
//! ```
//!
//! ## Runtime Detection
//!
//! The module automatically detects CPU capabilities at runtime and selects
//! the best available implementation:
//! - AVX512F: 512-bit operations (if available)
//! - AVX2: 256-bit operations (most modern x86-64 CPUs)
//! - Fallback: Standard library copy (always works)

use std::sync::atomic::{AtomicBool, Ordering};

// ============================================================================
// PREFETCH HINTS FOR NON-CONTIGUOUS ACCESS PATTERNS
// ============================================================================
//
// These prefetch functions are designed for ring buffer scenarios where:
// - Producer and consumer positions are far apart
// - Wrap-around access patterns require non-sequential prefetching
// - Data access patterns are known ahead of time
//
// NOTE: Do NOT use these for sequential access - the CPU hardware prefetcher
// handles sequential patterns very efficiently (and often better than manual hints).
//

/// Prefetch locality hint - determines where data is placed in cache hierarchy.
///
/// Use these hints based on expected data reuse:
/// - `T0`: Keep in all cache levels (L1, L2, L3) - for hot data accessed soon
/// - `T1`: Keep in L2 and L3 only - for data accessed moderately soon
/// - `T2`: Keep in L3 only - for data accessed later
/// - `Nta`: Non-temporal - data used once, don't pollute caches
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(i32)]
#[derive(Default)]
pub enum PrefetchHint {
    /// Prefetch to all cache levels (L1, L2, L3) - for immediate reuse
    #[default]
    T0 = 3,
    /// Prefetch to L2 and L3 - for moderate reuse
    T1 = 2,
    /// Prefetch to L3 only - for delayed reuse
    T2 = 1,
    /// Non-temporal prefetch - for streaming/single-use data
    Nta = 0,
}

/// Prefetch a single cache line from the specified address.
///
/// This is a low-level primitive for manual prefetching. For most use cases,
/// prefer the higher-level `prefetch_ring_segment` or `prefetch_non_contiguous`.
///
/// # Arguments
/// * `addr` - Address to prefetch (will be rounded to cache line boundary)
/// * `hint` - Cache level hint (see [`PrefetchHint`])
///
/// # When to Use
/// - Non-contiguous access patterns (e.g., jumping around in a buffer)
/// - Ring buffer wrap-around points
/// - Known future access points in sparse data structures
///
/// # When NOT to Use
/// - Sequential access (CPU hardware prefetcher handles this better)
/// - Small buffers (< 4KB - prefetch overhead exceeds benefit)
/// - Unpredictable access patterns
///
/// # Safety
/// The address must be valid for reading, though reading from the address
/// after prefetch is safe. Invalid addresses result in a no-op.
#[inline]
pub fn prefetch(addr: *const u8, hint: PrefetchHint) {
    #[cfg(target_arch = "x86_64")]
    {
        if is_avx2_available() || is_x86_feature_detected!("sse") {
            // SAFETY: _mm_prefetch is safe to call with any address.
            // Invalid addresses simply result in a no-op.
            unsafe {
                use std::arch::x86_64::*;
                match hint {
                    PrefetchHint::T0 => _mm_prefetch(addr as *const i8, _MM_HINT_T0),
                    PrefetchHint::T1 => _mm_prefetch(addr as *const i8, _MM_HINT_T1),
                    PrefetchHint::T2 => _mm_prefetch(addr as *const i8, _MM_HINT_T2),
                    PrefetchHint::Nta => _mm_prefetch(addr as *const i8, _MM_HINT_NTA),
                }
            }
        }
    }

    #[cfg(not(target_arch = "x86_64"))]
    {
        // No-op on non-x86 architectures
        let _ = (addr, hint);
    }
}

/// Prefetch multiple cache lines covering a memory range.
///
/// Efficiently prefetches multiple cache lines to cover the specified range.
/// Useful when you know you'll need a chunk of memory soon.
///
/// # Arguments
/// * `addr` - Starting address
/// * `len` - Number of bytes to prefetch
/// * `hint` - Cache level hint
///
/// # Safety
/// - `addr` must be valid for reads of `len` bytes
/// - The memory range `[addr, addr + len)` must be within a single allocated object
///
/// # Example
/// ```rust,no_run
/// use horus_core::memory::simd::{prefetch_range, PrefetchHint};
///
/// let buffer = vec![0u8; 4096];
/// // Prefetch the first 1KB for immediate use
/// // SAFETY: buffer is valid for 4096 bytes, we're prefetching 1024
/// unsafe { prefetch_range(buffer.as_ptr(), 1024, PrefetchHint::T0); }
/// ```
#[inline]
pub unsafe fn prefetch_range(addr: *const u8, len: usize, hint: PrefetchHint) {
    const CACHE_LINE_SIZE: usize = 64;

    // Don't prefetch tiny ranges - not worth the instruction overhead
    if len < CACHE_LINE_SIZE {
        return;
    }

    let num_lines = len.div_ceil(CACHE_LINE_SIZE);

    // Limit prefetch count to avoid instruction cache pressure
    let max_prefetches = 16;
    let lines_to_prefetch = num_lines.min(max_prefetches);

    for i in 0..lines_to_prefetch {
        // SAFETY: We're computing addresses within the specified range
        let line_addr = unsafe { addr.add(i * CACHE_LINE_SIZE) };
        prefetch(line_addr, hint);
    }
}

/// Prefetch a segment of a ring buffer with wrap-around handling.
///
/// This function handles the common ring buffer pattern where data may wrap
/// around from the end of the buffer to the beginning. It prefetches both
/// the contiguous part and the wrapped part if necessary.
///
/// # Arguments
/// * `buffer_base` - Start of the ring buffer
/// * `buffer_size` - Total size of the ring buffer in bytes
/// * `offset` - Current position in the ring buffer
/// * `length` - Number of bytes to prefetch
/// * `hint` - Cache level hint
///
/// # Safety
/// - `buffer_base` must be valid for reads of `buffer_size` bytes
/// - The memory range `[buffer_base, buffer_base + buffer_size)` must be within a single allocated object
///
/// # Example
/// ```rust,no_run
/// use horus_core::memory::simd::{prefetch_ring_segment, PrefetchHint};
///
/// let ring_buffer = vec![0u8; 64 * 1024]; // 64KB ring buffer
/// let buffer_size = ring_buffer.len();
/// let current_pos = 60 * 1024; // Near the end
/// let prefetch_len = 8 * 1024; // Need 8KB (will wrap around)
///
/// // SAFETY: ring_buffer is valid for buffer_size bytes
/// unsafe {
///     prefetch_ring_segment(
///         ring_buffer.as_ptr(),
///         buffer_size,
///         current_pos,
///         prefetch_len,
///         PrefetchHint::T0,
///     );
/// }
/// ```
///
/// # Performance Notes
/// - Only use for non-contiguous access patterns where producer/consumer are far apart
/// - The CPU's hardware prefetcher handles sequential ring buffer access well
/// - Best for cases where you're jumping to a known position (e.g., after a sequence gap)
#[inline]
pub unsafe fn prefetch_ring_segment(
    buffer_base: *const u8,
    buffer_size: usize,
    offset: usize,
    length: usize,
    hint: PrefetchHint,
) {
    if buffer_size == 0 || length == 0 {
        return;
    }

    // Normalize offset to be within buffer bounds
    let offset = offset % buffer_size;

    // Calculate how much data is in the contiguous part (before wrap)
    let contiguous_len = (buffer_size - offset).min(length);

    // Prefetch the contiguous part (offset is within buffer_size)
    let contiguous_addr = buffer_base.add(offset);
    prefetch_range(contiguous_addr, contiguous_len, hint);

    // If we need to wrap around, prefetch from the beginning
    if length > contiguous_len {
        let wrap_len = length - contiguous_len;
        prefetch_range(buffer_base, wrap_len, hint);
    }
}

/// Prefetch non-contiguous memory regions (scatter pattern).
///
/// Prefetches multiple non-contiguous memory locations. Useful for:
/// - Sparse data structure traversal
/// - Jump tables or indexed access patterns
/// - Any pattern where you know multiple future access points
///
/// # Arguments
/// * `addresses` - Slice of addresses to prefetch
/// * `hint` - Cache level hint
///
/// # Example
/// ```rust,no_run
/// use horus_core::memory::simd::{prefetch_scatter, PrefetchHint};
///
/// let buffer = vec![0u8; 1024 * 1024]; // 1MB buffer
/// let indices = [100, 50000, 200000, 800000]; // Known future access points
///
/// let addrs: Vec<*const u8> = indices.iter()
///     .map(|&i| unsafe { buffer.as_ptr().add(i) })
///     .collect();
///
/// prefetch_scatter(&addrs, PrefetchHint::T0);
/// ```
#[inline]
pub fn prefetch_scatter(addresses: &[*const u8], hint: PrefetchHint) {
    // Limit to reasonable number of prefetches
    let max_prefetches = 32;
    let count = addresses.len().min(max_prefetches);

    for addr in addresses.iter().take(count) {
        prefetch(*addr, hint);
    }
}

/// Prefetch stride pattern for predictable non-sequential access.
///
/// Prefetches memory at regular intervals (stride), useful for:
/// - Column access in row-major matrices
/// - Accessing every Nth element
/// - Any pattern with regular spacing
///
/// # Arguments
/// * `base` - Starting address
/// * `stride` - Distance between accesses in bytes
/// * `count` - Number of elements to prefetch
/// * `hint` - Cache level hint
///
/// # Safety
/// - `base` must be valid for reads up to `base + (count - 1) * stride` bytes
/// - The entire memory range being prefetched must be within a single allocated object
///
/// # Example
/// ```rust,no_run
/// use horus_core::memory::simd::{prefetch_stride, PrefetchHint};
///
/// let matrix = vec![0u8; 1024 * 1024]; // Row-major matrix
/// let row_stride = 1024; // Columns per row
///
/// // Prefetch every row's first element (column access pattern)
/// // SAFETY: matrix is valid for 1MB, we're prefetching 16 elements at 1024-byte intervals
/// unsafe { prefetch_stride(matrix.as_ptr(), row_stride, 16, PrefetchHint::T0); }
/// ```
#[inline]
pub unsafe fn prefetch_stride(base: *const u8, stride: usize, count: usize, hint: PrefetchHint) {
    if stride == 0 {
        return;
    }

    // Limit prefetch count
    let max_prefetches = 16;
    let actual_count = count.min(max_prefetches);

    for i in 0..actual_count {
        let addr = base.add(i * stride);
        prefetch(addr, hint);
    }
}

// ============================================================================
// SIMD THRESHOLDS AND CONSTANTS
// ============================================================================

/// Threshold in bytes above which SIMD copy is beneficial
/// Below this size, the setup overhead exceeds the benefit
pub const SIMD_COPY_THRESHOLD: usize = 4096; // 4KB

/// Alignment requirement for SIMD operations (AVX2 = 32 bytes)
pub const SIMD_ALIGNMENT: usize = 32;

// Runtime feature detection cache
static AVX2_CHECKED: AtomicBool = AtomicBool::new(false);
static AVX2_AVAILABLE: AtomicBool = AtomicBool::new(false);
static AVX512_CHECKED: AtomicBool = AtomicBool::new(false);
static AVX512_AVAILABLE: AtomicBool = AtomicBool::new(false);

/// Check if AVX2 is available on the current CPU
#[inline]
pub fn is_avx2_available() -> bool {
    // Use cached result if already checked
    if AVX2_CHECKED.load(Ordering::Relaxed) {
        return AVX2_AVAILABLE.load(Ordering::Relaxed);
    }

    #[cfg(target_arch = "x86_64")]
    {
        let available = is_x86_feature_detected!("avx2");
        AVX2_AVAILABLE.store(available, Ordering::Relaxed);
        AVX2_CHECKED.store(true, Ordering::Release);
        available
    }

    #[cfg(not(target_arch = "x86_64"))]
    {
        AVX2_CHECKED.store(true, Ordering::Release);
        false
    }
}

/// Check if AVX512F is available on the current CPU
#[inline]
pub fn is_avx512_available() -> bool {
    // Use cached result if already checked
    if AVX512_CHECKED.load(Ordering::Relaxed) {
        return AVX512_AVAILABLE.load(Ordering::Relaxed);
    }

    #[cfg(target_arch = "x86_64")]
    {
        let available = is_x86_feature_detected!("avx512f");
        AVX512_AVAILABLE.store(available, Ordering::Relaxed);
        AVX512_CHECKED.store(true, Ordering::Release);
        available
    }

    #[cfg(not(target_arch = "x86_64"))]
    {
        AVX512_CHECKED.store(true, Ordering::Release);
        false
    }
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
/// - For buffers >= 4KB: Uses AVX2/AVX512 streaming stores
/// - Automatically detects CPU capabilities at runtime
#[inline]
pub unsafe fn simd_copy_to_shm(src: *const u8, dst: *mut u8, len: usize) {
    if len < SIMD_COPY_THRESHOLD {
        // Small copy: use standard library
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

/// Copy data FROM shared memory using SIMD.
///
/// This function uses regular SIMD loads (not streaming) to read from
/// shared memory, which works well when the data might be read multiple
/// times or is already in cache.
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
        // Small copy: use standard library
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
        // This is optimal when writing to shared memory that another process will read
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

        // Store to local memory (aligned store when possible)
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

/// Zero-initialize a buffer using SIMD.
///
/// Useful for pre-initializing shared memory regions.
#[inline]
pub unsafe fn simd_zero_memory(dst: *mut u8, len: usize) {
    if len < SIMD_COPY_THRESHOLD {
        std::ptr::write_bytes(dst, 0, len);
        return;
    }

    #[cfg(target_arch = "x86_64")]
    {
        if is_avx2_available() {
            simd_zero_memory_avx2(dst, len);
            return;
        }
    }

    std::ptr::write_bytes(dst, 0, len);
}

/// AVX2 implementation of memory zeroing with streaming stores.
#[cfg(target_arch = "x86_64")]
#[target_feature(enable = "avx2")]
unsafe fn simd_zero_memory_avx2(dst: *mut u8, len: usize) {
    use std::arch::x86_64::*;

    let zero = _mm256_setzero_si256();
    let mut dst_ptr = dst;
    let mut remaining = len;

    // Handle unaligned prefix
    let dst_addr = dst_ptr as usize;
    let unaligned_prefix = (SIMD_ALIGNMENT - (dst_addr % SIMD_ALIGNMENT)) % SIMD_ALIGNMENT;

    if unaligned_prefix > 0 && unaligned_prefix <= remaining {
        std::ptr::write_bytes(dst_ptr, 0, unaligned_prefix);
        dst_ptr = dst_ptr.add(unaligned_prefix);
        remaining -= unaligned_prefix;
    }

    const VECTOR_SIZE: usize = 32;
    const UNROLL_FACTOR: usize = 4;
    const CHUNK_SIZE: usize = VECTOR_SIZE * UNROLL_FACTOR;

    while remaining >= CHUNK_SIZE {
        _mm256_stream_si256(dst_ptr as *mut __m256i, zero);
        _mm256_stream_si256(dst_ptr.add(VECTOR_SIZE) as *mut __m256i, zero);
        _mm256_stream_si256(dst_ptr.add(VECTOR_SIZE * 2) as *mut __m256i, zero);
        _mm256_stream_si256(dst_ptr.add(VECTOR_SIZE * 3) as *mut __m256i, zero);

        dst_ptr = dst_ptr.add(CHUNK_SIZE);
        remaining -= CHUNK_SIZE;
    }

    while remaining >= VECTOR_SIZE {
        _mm256_stream_si256(dst_ptr as *mut __m256i, zero);
        dst_ptr = dst_ptr.add(VECTOR_SIZE);
        remaining -= VECTOR_SIZE;
    }

    _mm_sfence();

    if remaining > 0 {
        std::ptr::write_bytes(dst_ptr, 0, remaining);
    }
}

/// Get information about SIMD capabilities for debugging/logging.
#[derive(Debug, Clone)]
pub struct SimdCapabilities {
    pub avx2: bool,
    pub avx512f: bool,
    pub recommended_copy_threshold: usize,
    pub vector_size: usize,
}

impl SimdCapabilities {
    /// Detect current CPU's SIMD capabilities.
    pub fn detect() -> Self {
        let avx2 = is_avx2_available();
        let avx512f = is_avx512_available();

        let vector_size = if avx512f {
            64
        } else if avx2 {
            32
        } else {
            8 // Default word size
        };

        Self {
            avx2,
            avx512f,
            recommended_copy_threshold: SIMD_COPY_THRESHOLD,
            vector_size,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_simd_capabilities_detect() {
        let caps = SimdCapabilities::detect();
        // Should not panic
        println!("SIMD capabilities: {:?}", caps);
    }

    #[test]
    fn test_avx2_detection() {
        // Call multiple times to test caching
        let first = is_avx2_available();
        let second = is_avx2_available();
        assert_eq!(first, second);
    }

    #[test]
    fn test_simd_copy_small() {
        // Small copy should work
        let src = vec![42u8; 100];
        let mut dst = vec![0u8; 100];

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

        unsafe {
            simd_copy_from_shm(src.as_ptr(), dst.as_mut_ptr(), size);
        }

        assert_eq!(src, dst);
    }

    #[test]
    fn test_simd_zero_memory() {
        let size = 16 * 1024; // 16KB
        let mut buffer = vec![0xFFu8; size];

        unsafe {
            simd_zero_memory(buffer.as_mut_ptr(), size);
        }

        assert!(buffer.iter().all(|&b| b == 0));
    }

    #[test]
    fn test_simd_copy_unaligned() {
        // Test with unaligned pointers
        let size = 8 * 1024 + 17; // Non-round size
        let src: Vec<u8> = (0..size).map(|i| (i % 256) as u8).collect();
        let mut dst = vec![0u8; size];

        // Copy starting at offset 3 (unaligned)
        let offset = 3;
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
    fn test_simd_copy_exact_chunk_size() {
        // Test with size that's exactly a multiple of chunk size (128 bytes)
        let size = 128 * 100; // 12,800 bytes
        let src: Vec<u8> = (0..size).map(|i| (i % 256) as u8).collect();
        let mut dst = vec![0u8; size];

        unsafe {
            simd_copy_to_shm(src.as_ptr(), dst.as_mut_ptr(), size);
        }

        assert_eq!(src, dst);
    }

    // ========================================================================
    // Prefetch Tests
    // ========================================================================

    #[test]
    fn test_prefetch_single() {
        // Basic prefetch should not panic
        let buffer = vec![0u8; 1024];
        prefetch(buffer.as_ptr(), PrefetchHint::T0);
        prefetch(buffer.as_ptr(), PrefetchHint::T1);
        prefetch(buffer.as_ptr(), PrefetchHint::T2);
        prefetch(buffer.as_ptr(), PrefetchHint::Nta);
    }

    #[test]
    fn test_prefetch_hint_default() {
        let hint = PrefetchHint::default();
        assert_eq!(hint, PrefetchHint::T0);
    }

    #[test]
    fn test_prefetch_range_normal() {
        let buffer = vec![0u8; 4096];
        // Should not panic
        // SAFETY: buffer is valid and large enough for the specified length
        unsafe {
            prefetch_range(buffer.as_ptr(), 1024, PrefetchHint::T0);
            prefetch_range(buffer.as_ptr(), 4096, PrefetchHint::T1);
        }
    }

    #[test]
    fn test_prefetch_range_small() {
        // Small range (< cache line) should be a no-op but not panic
        let buffer = vec![0u8; 32];
        // SAFETY: buffer is valid for the specified length
        unsafe {
            prefetch_range(buffer.as_ptr(), 32, PrefetchHint::T0);
        }
    }

    #[test]
    fn test_prefetch_range_large() {
        // Large range should be limited to max prefetches
        let buffer = vec![0u8; 1024 * 1024]; // 1MB
                                             // SAFETY: buffer is valid for the specified length
        unsafe {
            prefetch_range(buffer.as_ptr(), 1024 * 1024, PrefetchHint::T0);
        }
    }

    #[test]
    fn test_prefetch_ring_segment_no_wrap() {
        let buffer = vec![0u8; 64 * 1024]; // 64KB ring buffer
        let buffer_size = buffer.len();

        // SAFETY: buffer is valid for buffer_size bytes
        unsafe {
            // Access at beginning - no wrap
            prefetch_ring_segment(buffer.as_ptr(), buffer_size, 0, 4096, PrefetchHint::T0);

            // Access in middle - no wrap
            prefetch_ring_segment(
                buffer.as_ptr(),
                buffer_size,
                32 * 1024,
                8192,
                PrefetchHint::T0,
            );
        }
    }

    #[test]
    fn test_prefetch_ring_segment_with_wrap() {
        let buffer = vec![0u8; 64 * 1024]; // 64KB ring buffer
        let buffer_size = buffer.len();

        // SAFETY: buffer is valid for buffer_size bytes
        unsafe {
            // Access near end that wraps around
            prefetch_ring_segment(
                buffer.as_ptr(),
                buffer_size,
                60 * 1024, // Start near end
                8192,      // Need 8KB, which wraps around
                PrefetchHint::T0,
            );
        }
    }

    #[test]
    fn test_prefetch_ring_segment_edge_cases() {
        let buffer = vec![0u8; 1024];

        // SAFETY: buffer is valid for 1024 bytes
        unsafe {
            // Zero length
            prefetch_ring_segment(buffer.as_ptr(), 1024, 0, 0, PrefetchHint::T0);

            // Zero buffer size
            prefetch_ring_segment(buffer.as_ptr(), 0, 0, 100, PrefetchHint::T0);

            // Offset larger than buffer (should wrap)
            prefetch_ring_segment(buffer.as_ptr(), 1024, 2048, 100, PrefetchHint::T0);
        }
    }

    #[test]
    fn test_prefetch_scatter() {
        let buffer = vec![0u8; 1024 * 1024]; // 1MB buffer

        // Create scattered addresses
        let addresses: Vec<*const u8> = vec![
            unsafe { buffer.as_ptr().add(100) },
            unsafe { buffer.as_ptr().add(50000) },
            unsafe { buffer.as_ptr().add(200000) },
            unsafe { buffer.as_ptr().add(800000) },
        ];

        prefetch_scatter(&addresses, PrefetchHint::T0);
    }

    #[test]
    fn test_prefetch_scatter_empty() {
        // Empty address list should not panic
        let addresses: Vec<*const u8> = vec![];
        prefetch_scatter(&addresses, PrefetchHint::T0);
    }

    #[test]
    fn test_prefetch_stride() {
        let buffer = vec![0u8; 1024 * 1024]; // 1MB buffer

        // SAFETY: buffer is large enough for stride * count accesses
        unsafe {
            // Stride access (like column access in a matrix)
            prefetch_stride(buffer.as_ptr(), 1024, 16, PrefetchHint::T0);
        }
    }

    #[test]
    fn test_prefetch_stride_zero() {
        let buffer = vec![0u8; 1024];

        // SAFETY: buffer is valid, zero stride is handled gracefully
        unsafe {
            // Zero stride should be a no-op
            prefetch_stride(buffer.as_ptr(), 0, 16, PrefetchHint::T0);
        }
    }

    #[test]
    fn test_prefetch_stride_large_count() {
        let buffer = vec![0u8; 64 * 1024];

        // SAFETY: buffer is large enough for stride * (limited) count
        unsafe {
            // Large count should be limited to max prefetches
            prefetch_stride(buffer.as_ptr(), 64, 1000, PrefetchHint::T0);
        }
    }
}
