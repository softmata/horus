//! Pool backend abstraction for tensor data allocation.
//!
//! [`PoolBackend`] is the open trait that controls how the tensor pool's
//! **data region** is allocated. Metadata (header + slot headers) always
//! lives in mmap for cross-process discovery; only the actual tensor bytes
//! go through the backend.
//!
//! # Built-in backends
//!
//! - [`MmapBackend`] — standard `/dev/shm` mmap (default, always available)
//!
//! # Future backends (feature-gated)
//!
//! - `CudaManagedBackend` — `cudaMallocManaged` (Jetson unified memory)
//! - `CudaPinnedBackend` — `cudaMallocHost` (discrete GPU, DMA-capable)
//! - `CudaDeviceBackend` — `cudaMalloc` (GPU-only, highest bandwidth)
//!
//! # Extensibility
//!
//! Users can implement [`PoolBackend`] for custom memory backends (Vulkan
//! compute, FPGA, RDMA, etc.) without modifying horus_core.

use crate::types::Device;
use std::fmt;
use std::sync::atomic::{AtomicU64, Ordering};

/// Describes a single allocation made by a [`PoolBackend`].
///
/// The backend fills in the fields that apply to its memory type.
/// For CPU-only backends, `device_ptr` is null. For GPU-only backends,
/// `cpu_ptr` is null. For unified memory, both may be the same address.
#[derive(Debug)]
pub struct BackendAllocation {
    /// CPU-accessible pointer (null for device-only memory).
    pub cpu_ptr: *mut u8,
    /// Device-accessible pointer (null for CPU-only memory).
    /// For unified memory backends (Jetson), this equals `cpu_ptr`.
    pub device_ptr: *mut u8,
    /// Size of the allocation in bytes.
    pub size: usize,
}

// SAFETY: BackendAllocation is a bag of raw pointers + size.
// The backend that created it is responsible for ensuring the pointers
// remain valid. Sending the allocation across threads is safe because
// the underlying memory (mmap, CUDA managed, pinned) is thread-safe.
unsafe impl Send for BackendAllocation {}
unsafe impl Sync for BackendAllocation {}

/// Memory backend for the tensor pool's data region.
///
/// This trait controls **where tensor bytes live**. The tensor pool's
/// metadata (PoolHeader, SlotHeaders, Treiber free stack) always stays
/// in mmap for cross-process atomics. Only the data region is delegated
/// to the backend.
///
/// # Object Safety
///
/// This trait is object-safe and intended to be used as `Box<dyn PoolBackend>`.
///
/// # Thread Safety
///
/// Backends must be `Send + Sync` — the tensor pool is shared across
/// threads and the data region may be accessed concurrently.
///
/// # Implementing a Custom Backend
///
/// ```rust,ignore
/// use horus_core::memory::backend::{PoolBackend, BackendAllocation};
/// use horus_core::types::Device;
///
/// struct MyFpgaBackend { device_id: u32 }
///
/// impl PoolBackend for MyFpgaBackend {
///     fn alloc(&self, size: usize) -> Result<BackendAllocation, String> {
///         let ptr = my_fpga_malloc(size)?;
///         Ok(BackendAllocation {
///             cpu_ptr: std::ptr::null_mut(), // FPGA memory, not CPU-accessible
///             device_ptr: ptr,
///             size,
///         })
///     }
///     fn free(&self, alloc: &BackendAllocation) { my_fpga_free(alloc.device_ptr); }
///     fn device(&self) -> Device { Device::cuda(self.device_id) } // or a future Device variant
///     fn is_shared(&self) -> bool { false }
///     fn name(&self) -> &str { "fpga" }
/// }
/// ```
pub trait PoolBackend: Send + Sync + fmt::Debug {
    /// Allocate `size` bytes of memory.
    ///
    /// Returns a [`BackendAllocation`] describing where the memory lives.
    /// The backend decides which pointers are valid:
    /// - CPU backend: `cpu_ptr` is valid, `device_ptr` is null
    /// - Unified memory: both are valid (same address on Jetson)
    /// - Pinned memory: `cpu_ptr` is valid (DMA-capable), `device_ptr` is null
    /// - Device-only: `cpu_ptr` is null, `device_ptr` is valid
    ///
    /// # Errors
    ///
    /// Returns an error string if allocation fails (out of memory,
    /// device not available, etc.).
    fn alloc(&self, size: usize) -> Result<BackendAllocation, String>;

    /// Free a previously allocated region.
    ///
    /// The `alloc` parameter must have been returned by a previous call
    /// to [`alloc`](PoolBackend::alloc) on the same backend instance.
    /// Passing an allocation from a different backend is undefined behavior.
    fn free(&self, alloc: &BackendAllocation);

    /// What device does this backend target?
    ///
    /// Used by the tensor pool to set `device_type` and `device_id`
    /// on allocated tensor descriptors, ensuring the descriptor
    /// truthfully reports where the data lives.
    fn device(&self) -> Device;

    /// Is the memory cross-process shareable?
    ///
    /// `true` for mmap-backed shared memory (another process can open
    /// the same pool and access the data). `false` for process-local
    /// allocations (CUDA managed, pinned, device memory).
    fn is_shared(&self) -> bool;

    /// Human-readable backend name for diagnostics.
    ///
    /// Used by `horus doctor`, `horus topic list --verbose`, and
    /// observability APIs. Examples: `"mmap"`, `"cuda_managed"`,
    /// `"cuda_pinned"`, `"cuda_device"`.
    fn name(&self) -> &str;

    /// Zero out a previously allocated region (security: prevent data leaks).
    ///
    /// Called by `return_slot()` before marking a slot as free, so a later
    /// tenant of the same slot — possibly in another process — cannot read the
    /// previous tenant's bytes.
    ///
    /// The default implementation is [`scrub_bytes`]: a `write_bytes` kept
    /// alive by an inline-assembly barrier, so it cannot be optimised away and
    /// is still free to vectorise. See that function's docs for why the
    /// guarantee matters here and what the volatile version it replaced cost.
    ///
    /// GPU backends may override to use `cudaMemset` or skip zeroing
    /// entirely if cross-process data leaks are not a concern.
    fn zero(&self, alloc: &BackendAllocation) {
        if alloc.cpu_ptr.is_null() || alloc.size == 0 {
            return;
        }
        // SAFETY: cpu_ptr is valid and writable for alloc.size bytes — that is
        // the contract of `alloc()`, and `TensorPool::return_slot` additionally
        // bounds-checks a shared-backend pointer against the mapping before
        // building the `BackendAllocation` handed here.
        unsafe { scrub_bytes(alloc.cpu_ptr, alloc.size) };
        // Ensure the zero writes are ordered before the slot is marked free.
        core::sync::atomic::compiler_fence(core::sync::atomic::Ordering::SeqCst);
    }
}

// ---------------------------------------------------------------------------
// Scrub primitive
// ---------------------------------------------------------------------------

/// Overwrite `len` bytes at `ptr` with zeros, guaranteed not to be optimised
/// away.
///
/// # Why this needs a guarantee at all
///
/// This is the slot scrub that stands between two tenants of the same pool
/// slot, possibly in different processes. If the compiler removes it, the next
/// tenant reads the previous one's bytes out of shared memory. The scrub is
/// therefore not an optimisation detail — it is the whole security property,
/// and "the optimiser probably will not remove it" is not the standard to hold
/// it to.
///
/// # Why `memcpy`-family stores rather than `write_volatile`
///
/// This used to be a hand-unrolled loop of eight `write_volatile::<usize>` per
/// cache line. Volatile did give the guarantee, but it also forbids
/// vectorisation and forbids the platform `memset` from choosing non-temporal
/// stores above its shared-cache threshold, and that costs most of the
/// throughput. Measured on an i7-10750H:
///
/// | region  | volatile words | `write_bytes` + barrier |
/// |---------|----------------|-------------------------|
/// | 4 KB    | 127 ns         |  56 ns                  |
/// | 300 KB  | 11.2 us        | 6.3 us                  |
/// | 2 MB    | 84.2 us        | 45.3 us                 |
///
/// Widening the volatile store does not recover it — `write_volatile::<u128>`
/// measured within noise of the `usize` version (25 vs 25 GB/s at 2 MB),
/// because what costs the bandwidth is that each volatile store must be
/// emitted individually, not how wide it is.
///
/// A 2 MB frame is a 1920x1080 Mono8 image, and this runs on the drop path, so
/// the difference is ~39 us per frame returned to the pool.
///
/// # How the guarantee survives
///
/// `asm!` without `options(nomem)` must be assumed by the compiler to read and
/// write every byte of memory it could reach. The stores above are therefore
/// observable and cannot be dead-code eliminated — the same guarantee volatile
/// gives, obtained without constraining how the stores are emitted.
///
/// On a target where inline assembly is unavailable this falls back to the
/// volatile loop: slower, and still correct.
///
/// # Safety
///
/// `ptr..ptr + len` must be valid for writes and exclusively owned by the
/// caller for the duration of the call.
#[inline]
pub unsafe fn scrub_bytes(ptr: *mut u8, len: usize) {
    if len == 0 {
        return;
    }

    #[cfg(any(
        target_arch = "x86_64",
        target_arch = "x86",
        target_arch = "aarch64",
        target_arch = "arm",
        target_arch = "riscv32",
        target_arch = "riscv64",
    ))]
    {
        // SAFETY: the caller guarantees the region is valid for writes.
        unsafe { core::ptr::write_bytes(ptr, 0, len) };
        // SAFETY: an empty asm block. No `nomem`, so the compiler must treat it
        // as reading the region written above and cannot elide those stores.
        unsafe {
            core::arch::asm!("/* scrub {0} */", in(reg) ptr, options(nostack, preserves_flags));
        }
    }

    #[cfg(not(any(
        target_arch = "x86_64",
        target_arch = "x86",
        target_arch = "aarch64",
        target_arch = "arm",
        target_arch = "riscv32",
        target_arch = "riscv64",
    )))]
    {
        const WORD: usize = core::mem::size_of::<usize>();
        let mut offset = 0usize;
        let misalign = ptr as usize & (WORD - 1);
        let head = if misalign == 0 { 0 } else { WORD - misalign };
        let head = head.min(len);
        while offset < head {
            // SAFETY: offset < head <= len.
            unsafe { ptr.add(offset).write_volatile(0u8) };
            offset += 1;
        }
        while len - offset >= WORD {
            // SAFETY: at least WORD bytes remain and offset is word-aligned.
            unsafe { (ptr.add(offset) as *mut usize).write_volatile(0) };
            offset += WORD;
        }
        while offset < len {
            // SAFETY: offset < len.
            unsafe { ptr.add(offset).write_volatile(0u8) };
            offset += 1;
        }
    }
}

// ---------------------------------------------------------------------------
// MmapBackend — default backend using the pool's shared mmap data region
// ---------------------------------------------------------------------------

/// Mmap-backed data allocation within the tensor pool's shared memory file.
///
/// This is the default backend. The data region is part of the same mmap
/// file as the pool metadata (PoolHeader + SlotHeaders). Allocation is a
/// lock-free bump from the shared `next_alloc_offset` atomic in the header.
///
/// # Ownership
///
/// `MmapBackend` does NOT own the mmap — [`TensorPool`](super::tensor_pool::TensorPool)
/// owns it. MmapBackend holds raw pointers into the mmap region, valid for
/// the pool's lifetime. This is enforced by construction: MmapBackend is
/// only created inside TensorPool and never outlives it.
///
/// # Free semantics
///
/// The bump allocator never reclaims data bytes. `free()` is a no-op for
/// the data region. Slot reuse is handled by the Treiber free stack in
/// the pool metadata — a reused slot may point to a different offset than
/// its previous allocation (if the old data was abandoned).
pub struct MmapBackend {
    /// Pointer to the start of the data region within the mmap.
    data_base: *mut u8,
    /// Pointer to the shared `next_alloc_offset` atomic in the PoolHeader.
    /// Multiple processes coordinate bump allocation through this atomic.
    next_alloc_offset: *const AtomicU64,
    /// Total size of the data region in bytes.
    pool_size: usize,
    /// Alignment for each allocation (default 64, cache-line).
    alignment: usize,
}

impl fmt::Debug for MmapBackend {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("MmapBackend")
            .field("pool_size", &self.pool_size)
            .field("alignment", &self.alignment)
            .finish()
    }
}

// SAFETY: The mmap region backing these pointers is shared memory that is
// inherently thread-safe (accessed via atomics for coordination). The raw
// pointers are stable for the lifetime of the TensorPool that created them.
unsafe impl Send for MmapBackend {}
unsafe impl Sync for MmapBackend {}

impl MmapBackend {
    /// Create a new MmapBackend from raw pointers into the pool's mmap.
    ///
    /// # Safety
    ///
    /// - `data_base` must point to the start of a valid data region of `pool_size` bytes.
    /// - `next_alloc_offset` must point to a valid `AtomicU64` in the pool header.
    /// - Both pointers must remain valid for the lifetime of this backend.
    /// - The caller (TensorPool) must ensure these invariants by keeping the mmap alive.
    pub(crate) unsafe fn new(
        data_base: *mut u8,
        next_alloc_offset: *const AtomicU64,
        pool_size: usize,
        alignment: usize,
    ) -> Self {
        Self {
            data_base,
            next_alloc_offset,
            pool_size,
            alignment,
        }
    }

    /// Round `value` up to the next multiple of `alignment`.
    ///
    /// Returns `None` rather than wrapping. The old body was
    /// `value.wrapping_add(alignment - 1) & !(alignment - 1)`, and both halves
    /// were unsound against the value it is actually fed: `value` is
    /// `next_alloc_offset`, an atomic in the pool's shared header that every
    /// process on the pool can write.
    ///
    /// * `wrapping_add` turned a large `value` into a SMALL aligned result —
    ///   `usize::MAX` with a 64-byte alignment aligned to 0 — which then passed
    ///   the `new_offset > pool_size` bounds check and won the CAS, *rewinding*
    ///   the shared bump cursor to the front of the data region. Every
    ///   subsequent allocation then handed out memory already owned by a live
    ///   tensor: two publishers writing the same bytes, with no error anywhere.
    ///   That is silent cross-process data corruption, not a crash.
    /// * `!(alignment - 1)` is only a valid mask for a power-of-two alignment.
    ///   `alignment` comes from `TensorPoolConfig::slot_alignment`; at 0 it
    ///   underflows to `usize::MAX`, whose mask is 0, so every allocation
    ///   returns offset 0 — the same aliasing, reached from a config typo.
    #[inline]
    fn align_up(value: usize, alignment: usize) -> Option<usize> {
        // `is_power_of_two` is false for 0, which also rejects the underflow.
        if !alignment.is_power_of_two() {
            return None;
        }
        value
            .checked_add(alignment - 1)
            .map(|v| v & !(alignment - 1))
    }
}

impl PoolBackend for MmapBackend {
    fn alloc(&self, size: usize) -> Result<BackendAllocation, String> {
        // SAFETY: next_alloc_offset was validated at construction time;
        // the PoolHeader (and its mmap backing) outlives this backend.
        let offset_atomic = unsafe { &*self.next_alloc_offset };

        loop {
            let current_raw = offset_atomic.load(Ordering::Acquire);
            // A bump cursor past the end of the data region cannot have been
            // produced by this code (every store is bounds-checked below), so it
            // is corruption in the shared header — a peer process, a stale
            // region, or a torn write. Refuse rather than fold it back into
            // range: `align_up` used to wrap such a value to a small offset and
            // hand out memory that a live tensor already owns.
            if current_raw > self.pool_size as u64 {
                return Err(format!(
                    "mmap pool bump cursor is {} but the data region is only {} bytes — \
                     the shared PoolHeader is corrupt; refusing to allocate",
                    current_raw, self.pool_size
                ));
            }
            let current = current_raw as usize;
            let aligned_current = Self::align_up(current, self.alignment).ok_or_else(|| {
                format!(
                    "cannot align bump cursor {} to {} bytes (alignment must be a power of two)",
                    current, self.alignment
                )
            })?;

            let new_offset = aligned_current
                .checked_add(size)
                .ok_or_else(|| "allocation offset overflow".to_string())?;

            if new_offset > self.pool_size {
                return Err(format!(
                    "mmap pool out of memory: need {} bytes, only {} available",
                    size,
                    self.pool_size.saturating_sub(current)
                ));
            }

            if offset_atomic
                .compare_exchange_weak(
                    current as u64,
                    new_offset as u64,
                    Ordering::AcqRel,
                    Ordering::Relaxed,
                )
                .is_ok()
            {
                // SAFETY: aligned_current is within the data region (bounds checked above).
                // data_base is valid for pool_size bytes (guaranteed by constructor).
                let ptr = unsafe { self.data_base.add(aligned_current) };
                return Ok(BackendAllocation {
                    cpu_ptr: ptr,
                    device_ptr: std::ptr::null_mut(),
                    size,
                });
            }
            // CAS failed — another thread/process won the race; retry.
        }
    }

    fn free(&self, _alloc: &BackendAllocation) {
        // Bump allocator never reclaims. The data region space is "leaked" by design.
        // Slot reuse is handled at the TensorPool level (Treiber free stack reuses
        // slots, but the data offset may point to abandoned space if the slot was
        // originally allocated with a different size).
    }

    fn device(&self) -> Device {
        Device::cpu()
    }

    fn is_shared(&self) -> bool {
        true
    }

    fn name(&self) -> &str {
        "mmap"
    }

    // zero() uses the default implementation from the PoolBackend trait
    // (word-sized volatile stores + compiler fence). The compiler fence keeps
    // the scrub ahead of `return_slot`'s `flags.store(SLOT_FREE, Release)`,
    // which is what publishes the slot to other allocators; on a weakly ordered
    // target that Release store is also what orders the scrub in *hardware*.
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    /// Scrub `len` bytes starting `start` bytes into a `0xAB`-filled buffer and
    /// assert that exactly `start..start + len` came back zero and every byte
    /// outside it is untouched.
    ///
    /// Both halves matter. A short write is a data leak (the whole point of the
    /// scrub); a long write is memory corruption of the neighbouring slot.
    fn scrub_exactly(buf: &mut [u8], start: usize, len: usize) {
        buf.fill(0xAB);
        // SAFETY: start + len <= buf.len() is checked by the caller's slicing.
        unsafe { scrub_bytes(buf.as_mut_ptr().add(start), len) };

        let short: Vec<usize> = buf[start..start + len]
            .iter()
            .enumerate()
            .filter(|(_, &b)| b != 0)
            .map(|(i, _)| i)
            .collect();
        assert!(
            short.is_empty(),
            "start={start} len={len}: unscrubbed bytes (LEAK) at offsets {short:?}"
        );

        let over_before: Vec<usize> = buf[..start]
            .iter()
            .enumerate()
            .filter(|(_, &b)| b != 0xAB)
            .map(|(i, _)| i)
            .collect();
        let over_after: Vec<usize> = buf[start + len..]
            .iter()
            .enumerate()
            .filter(|(_, &b)| b != 0xAB)
            .map(|(i, _)| i + start + len)
            .collect();
        assert!(
            over_before.is_empty() && over_after.is_empty(),
            "start={start} len={len}: wrote outside the region (CORRUPTION) \
             before={over_before:?} after={over_after:?}"
        );
    }

    #[test]
    fn scrub_covers_every_alignment_and_length() {
        // The head/body/tail split means the interesting cases are every
        // combination of start misalignment and length remainder around the
        // 8-word block. 0..=40 crosses two full blocks on a 64-bit target and
        // four on a 32-bit one, with every misalignment.
        let mut buf = vec![0u8; 128];
        for start in 0..=16usize {
            for len in 0..=40usize {
                scrub_exactly(&mut buf, start, len);
            }
        }
    }

    #[test]
    fn scrub_handles_a_multi_block_region() {
        // A length well past the unrolled body, at an odd start and an odd
        // length, so head, body, the whole-word remainder and the tail all run.
        let mut buf = vec![0u8; 4096];
        scrub_exactly(&mut buf, 3, 4001);
        scrub_exactly(&mut buf, 0, 4096);
        scrub_exactly(&mut buf, 7, 1);
    }

    #[test]
    fn scrub_of_zero_length_writes_nothing() {
        let mut buf = vec![0xABu8; 64];
        // SAFETY: a zero-length write through a valid pointer.
        unsafe { scrub_bytes(buf.as_mut_ptr(), 0) };
        assert!(buf.iter().all(|&b| b == 0xAB), "len=0 must write nothing");
    }

    /// Minimal backend used to exercise the `PoolBackend::zero` default body
    /// (null/zero-size guards and the delegation to `volatile_zero_bytes`)
    /// without standing up a whole `TensorPool`.
    #[derive(Debug)]
    struct HeapBackend;

    impl PoolBackend for HeapBackend {
        fn alloc(&self, _size: usize) -> Result<BackendAllocation, String> {
            Err("test backend does not allocate".to_string())
        }
        fn free(&self, _alloc: &BackendAllocation) {}
        fn device(&self) -> Device {
            Device::cpu()
        }
        fn is_shared(&self) -> bool {
            false
        }
        fn name(&self) -> &str {
            "test-heap"
        }
    }

    #[test]
    fn pool_backend_zero_scrubs_the_whole_allocation() {
        let mut buf = vec![0xCDu8; 300];
        let alloc = BackendAllocation {
            cpu_ptr: buf.as_mut_ptr(),
            device_ptr: std::ptr::null_mut(),
            size: buf.len(),
        };
        HeapBackend.zero(&alloc);
        assert!(
            buf.iter().all(|&b| b == 0),
            "PoolBackend::zero must scrub the whole allocation"
        );
    }

    /// `align_up` is fed `next_alloc_offset`, an atomic in the pool's shared
    /// header that any process on the pool can write. It must never turn a
    /// hostile or corrupt value into a small, plausible-looking offset.
    #[test]
    fn align_up_refuses_to_wrap_a_corrupt_bump_cursor() {
        // The ordinary cases still work.
        assert_eq!(MmapBackend::align_up(0, 64), Some(0));
        assert_eq!(MmapBackend::align_up(1, 64), Some(64));
        assert_eq!(MmapBackend::align_up(64, 64), Some(64));
        assert_eq!(MmapBackend::align_up(65, 64), Some(128));

        // The old `wrapping_add` body aligned this to 0 — the front of the data
        // region — and the caller then CAS'd the shared cursor back to a small
        // value and handed out memory a live tensor already owns.
        assert_eq!(
            MmapBackend::align_up(usize::MAX, 64),
            None,
            "a bump cursor near usize::MAX must be refused, not wrapped to 0"
        );
        assert_eq!(MmapBackend::align_up(usize::MAX - 62, 64), None);

        // A non-power-of-two alignment has no valid `!(n - 1)` mask; 0 also
        // underflowed to usize::MAX, whose mask is 0 (every offset becomes 0).
        assert_eq!(MmapBackend::align_up(4096, 0), None);
        assert_eq!(MmapBackend::align_up(4096, 48), None);
    }

    #[test]
    fn pool_backend_zero_ignores_null_and_empty_allocations() {
        // A GPU-only allocation has a null cpu_ptr; a zero-size slot has
        // nothing to scrub. Neither may be dereferenced.
        HeapBackend.zero(&BackendAllocation {
            cpu_ptr: std::ptr::null_mut(),
            device_ptr: std::ptr::null_mut(),
            size: 4096,
        });

        let mut buf = vec![0xCDu8; 16];
        HeapBackend.zero(&BackendAllocation {
            cpu_ptr: buf.as_mut_ptr(),
            device_ptr: std::ptr::null_mut(),
            size: 0,
        });
        assert!(buf.iter().all(|&b| b == 0xCD), "size=0 must write nothing");
    }
}
