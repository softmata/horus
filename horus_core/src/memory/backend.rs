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
    /// Called by `return_slot()` before marking a slot as free.
    /// The default implementation uses volatile byte writes for CPU-accessible
    /// memory, which cannot be elided by the compiler.
    ///
    /// GPU backends may override to use `cudaMemset` or skip zeroing
    /// entirely if cross-process data leaks are not a concern.
    fn zero(&self, alloc: &BackendAllocation) {
        if alloc.cpu_ptr.is_null() || alloc.size == 0 {
            return;
        }
        // Volatile writes prevent the compiler from eliding the zeroing.
        for i in 0..alloc.size {
            // SAFETY: cpu_ptr is valid for alloc.size bytes (guaranteed by alloc()).
            unsafe { alloc.cpu_ptr.add(i).write_volatile(0u8) };
        }
        // Ensure the zero writes are ordered before the slot is marked free.
        core::sync::atomic::compiler_fence(core::sync::atomic::Ordering::SeqCst);
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

    // zero() uses the default implementation from PoolBackend trait
    // (volatile byte writes + compiler fence), which matches the existing
    // TensorPool::volatile_zero + compiler_fence pattern exactly.
}

