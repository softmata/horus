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
    /// The default implementation writes the region with *word*-sized volatile
    /// stores (see [`volatile_zero_bytes`]). Volatile is what makes the write
    /// unelidable; the word width is what makes it affordable. See that
    /// function's docs for why the width matters and why it is still a
    /// guarantee rather than an argument.
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
        unsafe { volatile_zero_bytes(alloc.cpu_ptr, alloc.size) };
        // Ensure the zero writes are ordered before the slot is marked free.
        core::sync::atomic::compiler_fence(core::sync::atomic::Ordering::SeqCst);
    }
}

// ---------------------------------------------------------------------------
// Scrub primitive
// ---------------------------------------------------------------------------

/// Number of words written per unrolled iteration of the scrub body.
///
/// 8 × `usize` is one 64-byte cache line on a 64-bit target, so the body loop
/// closes exactly one line per branch.
const SCRUB_UNROLL: usize = 8;

/// Overwrite `len` bytes at `ptr` with zeros using volatile word stores.
///
/// # Why volatile, and why *not* byte-at-a-time
///
/// This is the slot scrub that stands between two tenants of the same pool
/// slot, so the bytes have to actually reach memory: the compiler must not be
/// allowed to decide the stores are dead. Volatile gives that as a *language
/// guarantee* — a volatile store is an observable side effect and LLVM may
/// neither remove it nor merge it with its neighbours.
///
/// That last clause is also the reason the old byte-at-a-time form was so
/// expensive. Because volatile stores cannot be merged or vectorised, one
/// `write_volatile::<u8>` is one store *instruction*, so scrubbing a 1080p RGB
/// frame (~6 MB) cost ~6 million stores — and on the default `MmapBackend` that
/// ran synchronously inside `Topic<Image>::send`, on the publisher's thread.
///
/// Widening the unit to `usize` keeps the guarantee exactly (each store is
/// still volatile, still unelidable) while cutting the instruction count by
/// `size_of::<usize>()`. What is left is the memory system: a multi-megabyte
/// region has to be written once no matter how, so large scrubs bottom out on
/// write bandwidth rather than on store issue.
///
/// # Why not `ptr::write_bytes` / `memset`
///
/// A plain `memset` would be somewhat faster still (glibc can reach for
/// non-temporal stores above its shared-cache threshold, which also avoids
/// evicting the caller's working set). It was deliberately not used here: a
/// non-volatile store is only kept alive by the argument that LLVM cannot prove
/// the region unread, and on this particular function that argument is doing
/// security work. Volatile makes it a guarantee instead. See the module docs of
/// the tensor pool for the trade if it is ever revisited.
///
/// # Alignment
///
/// `write_volatile::<usize>` requires a properly aligned pointer — misaligned
/// is UB in Rust even on ISAs that tolerate it — so the head is scrubbed a byte
/// at a time until the cursor is word-aligned, and the tail likewise. Both are
/// bounded by `size_of::<usize>() - 1` bytes.
///
/// # Safety
///
/// `ptr..ptr + len` must be valid for writes and exclusively owned by the
/// caller for the duration of the call.
#[inline]
pub unsafe fn volatile_zero_bytes(ptr: *mut u8, len: usize) {
    const WORD: usize = core::mem::size_of::<usize>();
    const BLOCK: usize = WORD * SCRUB_UNROLL;
    // The body below is unrolled by hand; keep the constant and the code in step.
    const _: () = assert!(SCRUB_UNROLL == 8);

    if len == 0 {
        return;
    }

    let mut offset = 0usize;

    // ── Head: byte stores until the cursor is word-aligned ──────────────────
    let misalign = ptr as usize & (WORD - 1);
    let head = if misalign == 0 { 0 } else { WORD - misalign };
    let head = head.min(len);
    while offset < head {
        // SAFETY: offset < head <= len, so ptr.add(offset) is inside the region.
        unsafe { ptr.add(offset).write_volatile(0u8) };
        offset += 1;
    }

    // ── Body: SCRUB_UNROLL word stores per branch ───────────────────────────
    // `offset` is word-aligned from here on, so every `*mut usize` below is too.
    let body_end = head + ((len - head) / BLOCK) * BLOCK;
    while offset < body_end {
        // SAFETY: offset + BLOCK <= body_end <= len, and offset is word-aligned.
        unsafe {
            let p = ptr.add(offset) as *mut usize;
            p.write_volatile(0);
            p.add(1).write_volatile(0);
            p.add(2).write_volatile(0);
            p.add(3).write_volatile(0);
            p.add(4).write_volatile(0);
            p.add(5).write_volatile(0);
            p.add(6).write_volatile(0);
            p.add(7).write_volatile(0);
        }
        offset += BLOCK;
    }

    // ── Remaining whole words ───────────────────────────────────────────────
    while len - offset >= WORD {
        // SAFETY: at least WORD bytes remain and offset is word-aligned.
        unsafe { (ptr.add(offset) as *mut usize).write_volatile(0) };
        offset += WORD;
    }

    // ── Tail: fewer than WORD bytes left ────────────────────────────────────
    while offset < len {
        // SAFETY: offset < len, so ptr.add(offset) is inside the region.
        unsafe { ptr.add(offset).write_volatile(0u8) };
        offset += 1;
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

    #[inline]
    fn align_up(value: usize, alignment: usize) -> usize {
        value.wrapping_add(alignment - 1) & !(alignment - 1)
    }
}

impl PoolBackend for MmapBackend {
    fn alloc(&self, size: usize) -> Result<BackendAllocation, String> {
        // SAFETY: next_alloc_offset was validated at construction time;
        // the PoolHeader (and its mmap backing) outlives this backend.
        let offset_atomic = unsafe { &*self.next_alloc_offset };

        loop {
            let current = offset_atomic.load(Ordering::Acquire) as usize;
            let aligned_current = Self::align_up(current, self.alignment);

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
        unsafe { volatile_zero_bytes(buf.as_mut_ptr().add(start), len) };

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
    fn volatile_zero_bytes_covers_every_alignment_and_length() {
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
    fn volatile_zero_bytes_handles_a_multi_block_region() {
        // A length well past the unrolled body, at an odd start and an odd
        // length, so head, body, the whole-word remainder and the tail all run.
        let mut buf = vec![0u8; 4096];
        scrub_exactly(&mut buf, 3, 4001);
        scrub_exactly(&mut buf, 0, 4096);
        scrub_exactly(&mut buf, 7, 1);
    }

    #[test]
    fn volatile_zero_bytes_of_zero_length_writes_nothing() {
        let mut buf = vec![0xABu8; 64];
        // SAFETY: a zero-length write through a valid pointer.
        unsafe { volatile_zero_bytes(buf.as_mut_ptr(), 0) };
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
