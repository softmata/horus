//! # Bulk memory copies into and out of shared memory
//!
//! These are the copies a topic makes for a message too large to move as a
//! plain value, and the ones the perception types make for whole frames:
//! `Image::copy_from` (640x480 RGB8 = 921 KB at 30 fps), `PointCloud::copy_from`
//! (100K points = 1.2 MB at 10 Hz), `DepthImage::copy_from`.
//!
//! ## Why there is no hand-written SIMD here any more
//!
//! This module used to implement both directions with AVX2: the write side
//! with `_mm256_stream_si256` non-temporal stores "ideal for write-once
//! patterns", the read side with 256-bit loads and software prefetch 512 bytes
//! ahead. Both lost to `std::ptr::copy_nonoverlapping`, which lowers to the
//! platform `memcpy`.
//!
//! Measured against iceoryx2 on an i7-10750H, same-thread 4 KB messages,
//! median / p99 / max in ns:
//!
//! | write side        | read side         | HORUS               |
//! |-------------------|-------------------|---------------------|
//! | AVX2 streaming    | AVX2 + prefetch   | 1018 / 1670 / 147731 |
//! | memcpy            | AVX2 + prefetch   |  698 /  785 /  18733 |
//! | memcpy            | memcpy            |  572 /  649 /  24975 |
//!
//! The streaming stores were the larger mistake, and the reason is structural
//! rather than a tuning miss: a non-temporal store bypasses the cache, so the
//! producer pushes the payload to DRAM and the consumer — which is about to
//! read it, immediately, that being the entire point of a topic — takes a full
//! memory round trip to get it back. "Write-once" described how the producer
//! touches the buffer, not how the system does.
//!
//! glibc's `memcpy` already makes this decision per microarchitecture: it
//! switches to non-temporal stores above `__x86_shared_non_temporal_threshold`,
//! tuned to a fraction of L3, and uses AVX or AVX-512 below it. A fixed 4 KB
//! rule compiled into this crate cannot know any of that, and on this CPU it
//! was wrong for every size a topic carries — `MAX_SLOT_SIZE` is 1 MB, well
//! inside a 12 MB L3.
//!
//! The functions are kept as named seams: they mark the places where a bulk
//! copy crosses into shared memory, and that is worth being able to find.

// ============================================================================
// SIMD THRESHOLDS AND CONSTANTS
// ============================================================================

/// Threshold in bytes above which SIMD copy is beneficial.
/// Below this size, the setup overhead exceeds the benefit.
pub const SIMD_COPY_THRESHOLD: usize = 4096; // 4KB

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

#[cfg(test)]
mod tests {
    use super::*;

    /// Both directions are byte-exact across the sizes where this module used
    /// to change strategy.
    ///
    /// This replaces a test that called `is_avx2_available()` twice and
    /// asserted the two results matched — it verified that a `OnceLock` caches,
    /// which `OnceLock` already guarantees, and would have passed on a build
    /// where every copy produced garbage.
    ///
    /// The sizes bracket the old `SIMD_COPY_THRESHOLD` and the old 32-byte
    /// vector and 128-byte unrolled chunk, because those boundaries are where a
    /// hand-written copy drops or duplicates its tail. Nothing dispatches on
    /// them now, which is exactly why they are worth keeping: a future
    /// reintroduction of a size-dependent path has to survive them.
    #[test]
    fn copies_are_byte_exact_across_the_old_strategy_boundaries() {
        for len in [
            0, 1, 31, 32, 33, 127, 128, 129, 4095, 4096, 4097, 8192, 12_289,
        ] {
            let src: Vec<u8> = (0..len).map(|i| (i % 251) as u8).collect();
            let mut dst = vec![0u8; len];
            // SAFETY: distinct Vec buffers of equal length.
            unsafe { simd_copy_to_shm(src.as_ptr(), dst.as_mut_ptr(), len) };
            assert_eq!(src, dst, "write path differs at len {len}");

            let mut back = vec![0u8; len];
            // SAFETY: distinct Vec buffers of equal length.
            unsafe { simd_copy_from_shm(dst.as_ptr(), back.as_mut_ptr(), len) };
            assert_eq!(src, back, "read path differs at len {len}");
        }
    }

    /// An unaligned destination copies correctly.
    ///
    /// The streaming-store path needed a 32-byte-aligned destination and hand
    /// wrote a prefix loop to reach one; `_mm256_stream_si256` on a misaligned
    /// address is a SIGBUS. Plain `memcpy` has no such requirement, so this
    /// guards the property rather than the implementation.
    #[test]
    fn an_unaligned_destination_copies_correctly() {
        let len = 4096 + 96;
        let src: Vec<u8> = (0..len).map(|i| (i % 253) as u8).collect();
        let mut backing = vec![0u8; len + 32];
        for off in [1usize, 7, 15, 31] {
            let dst = &mut backing[off..off + len];
            // SAFETY: `src` and the `backing` subslice are distinct allocations
            // of at least `len` bytes.
            unsafe { simd_copy_to_shm(src.as_ptr(), dst.as_mut_ptr(), len) };
            assert_eq!(&src[..], &dst[..], "unaligned by {off}");
        }
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

    /// Verify that simd_copy_to_shm and simd_copy_from_shm produce correct output
    /// for every possible unaligned offset 0..32 into the destination buffer.
    ///
    /// This exercises the unaligned-prefix logic in the AVX2 path (when AVX2 is
    /// available) and confirms that the debug_assert alignment invariant holds.
    /// A buffer of 8 KiB ensures the SIMD threshold is exceeded for all offsets.
    #[test]
    fn test_simd_copy_all_dst_offsets() {
        const BUF_SIZE: usize = 8 * 1024;
        // Over-allocate so we can start at offsets 0..32 and still copy BUF_SIZE bytes.
        const EXTRA: usize = 32;
        let src: Vec<u8> = (0..BUF_SIZE).map(|i| (i % 251) as u8).collect();

        for offset in 0..32usize {
            // dst_storage is aligned to at least 8 bytes by Vec; starting at
            // `offset` gives us every alignment 0..31 relative to 32.
            let mut dst_storage = vec![0u8; BUF_SIZE + EXTRA];
            // SAFETY: offset < 32 and dst_storage has BUF_SIZE + 32 bytes, so add(offset) is in bounds.
            let dst_ptr = unsafe { dst_storage.as_mut_ptr().add(offset) };

            // SAFETY: src is BUF_SIZE bytes; dst_ptr points into dst_storage with
            // BUF_SIZE bytes available from `offset`; regions don't overlap.
            unsafe {
                simd_copy_to_shm(src.as_ptr(), dst_ptr, BUF_SIZE);
            }

            assert_eq!(
                &src[..],
                &dst_storage[offset..offset + BUF_SIZE],
                "simd_copy_to_shm wrong at dst offset {}",
                offset,
            );

            // Reset and test simd_copy_from_shm with the same offset.
            let mut dst_storage2 = vec![0u8; BUF_SIZE + EXTRA];
            // SAFETY: offset < 32 and dst_storage2 has BUF_SIZE + 32 bytes, so add(offset) is in bounds.
            let dst_ptr2 = unsafe { dst_storage2.as_mut_ptr().add(offset) };
            // SAFETY: src is BUF_SIZE bytes; dst_ptr2 points into dst_storage2 with
            // BUF_SIZE bytes available from `offset`; regions don't overlap.
            unsafe {
                simd_copy_from_shm(src.as_ptr(), dst_ptr2, BUF_SIZE);
            }

            assert_eq!(
                &src[..],
                &dst_storage2[offset..offset + BUF_SIZE],
                "simd_copy_from_shm wrong at dst offset {}",
                offset,
            );
        }
    }
}
