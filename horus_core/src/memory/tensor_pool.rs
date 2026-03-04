//! Shared memory tensor pool for zero-copy tensor communication
//!
//! This module provides a high-performance memory pool for allocating tensors
//! that can be shared across processes with zero-copy semantics.
//!
//! # Architecture
//!
//! ```text
//! ┌────────────────────────────────────────────────────────────┐
//! │                    TensorPool Layout                        │
//! ├────────────────────────────────────────────────────────────┤
//! │  PoolHeader (64 bytes)                                     │
//! │  ├── magic: u64                                            │
//! │  ├── version: u32                                          │
//! │  ├── pool_id: u32                                          │
//! │  ├── pool_size: u64                                        │
//! │  ├── max_slots: u32                                        │
//! │  ├── slot_alignment: u32                                   │
//! │  └── next_alloc_offset: AtomicU64                          │
//! ├────────────────────────────────────────────────────────────┤
//! │  SlotHeaders[max_slots] (40 bytes each)                    │
//! │  ├── refcount: AtomicU32                                   │
//! │  ├── flags: AtomicU32                                      │
//! │  ├── generation: AtomicU64  (upgraded from U32 in v2)      │
//! │  ├── offset: u64                                           │
//! │  ├── size: u64                                             │
//! │  └── next_free: AtomicU32                                  │
//! ├────────────────────────────────────────────────────────────┤
//! │  Free Stack (lock-free)                                    │
//! │  └── free_stack_head: AtomicU64                            │
//! ├────────────────────────────────────────────────────────────┤
//! │                                                            │
//! │  Data Region (remaining space)                             │
//! │  └── Tensor data aligned to slot_alignment                 │
//! │                                                            │
//! └────────────────────────────────────────────────────────────┘
//! ```
//!
//! # Example
//!
//! ```rust,ignore
//! use horus_core::memory::TensorPool;
//!
//! // Create or open a tensor pool
//! let pool = TensorPool::new(1, TensorPoolConfig::default())?;
//!
//! // Allocate a tensor
//! let tensor = pool.alloc(&[1080, 1920, 3], TensorDtype::U8)?;
//!
//! // Get data pointer for writing
//! let data = pool.data_slice_mut(&tensor);
//! // ... write tensor data ...
//!
//! // Send tensor through Hub (only descriptor is copied)
//! hub.send(tensor);
//!
//! // Reference counting handles cleanup automatically
//! ```

use crate::error::{HorusError, HorusResult};
use crate::memory::platform::shm_base_dir;
use memmap2::{MmapMut, MmapOptions};
use std::fs::{File, OpenOptions};
use std::path::PathBuf;
use std::sync::atomic::{AtomicU32, AtomicU64, Ordering};

/// Magic number for pool validation
const POOL_MAGIC: u64 = 0x484F5255535F5450; // "HORUS_TP" in hex

/// Current pool version.
///
/// Version history:
///   1 — Initial format; `SlotHeader.generation` was `AtomicU32` (32-bit).
///   2 — `SlotHeader.generation` upgraded to `AtomicU64` (64-bit).
///       `SlotHeader` field order changed; total size grew from 36 to 40 bytes.
///       Pools created by v1 will fail `validate()` with a version mismatch.
const POOL_VERSION: u32 = 2;

/// Slot flags
const SLOT_FREE: u32 = 0;
const SLOT_ALLOCATED: u32 = 1;

/// Invalid slot index (sentinel for free list)
const INVALID_SLOT: u32 = u32::MAX;

/// Pack a generation counter and slot index into a single u64 for ABA-safe CAS.
/// Upper 32 bits: generation counter, lower 32 bits: slot index.
fn pack_tagged_head(generation: u32, slot_id: u32) -> u64 {
    ((generation as u64) << 32) | slot_id as u64
}

/// Unpack a tagged head into (generation, slot_id).
fn unpack_tagged_head(tagged: u64) -> (u32, u32) {
    let generation = (tagged >> 32) as u32;
    let slot_id = (tagged & 0xFFFF_FFFF) as u32;
    (generation, slot_id)
}

/// Pool header stored at the start of shared memory
#[repr(C)]
struct PoolHeader {
    magic: u64,
    version: u32,
    pool_id: u32,
    pool_size: u64,
    max_slots: u32,
    slot_alignment: u32,
    next_alloc_offset: AtomicU64,
    free_stack_head: AtomicU64,
    _padding: [u8; 16],
}

/// Slot metadata stored in shared memory
///
/// Layout (40 bytes, repr(C)):
///   refcount(4) + flags(4) + generation(8) + offset(8) + size(8)
///   + next_free(4) + _padding(4)
///
/// Field ordering ensures `AtomicU64`/`u64` fields land on 8-byte boundaries
/// without implicit padding.  Changed in POOL_VERSION 2 (from v1's 36-byte layout).
#[repr(C)]
struct SlotHeader {
    /// Reference count for this slot.
    refcount: AtomicU32,
    /// Slot flags: SLOT_FREE or SLOT_ALLOCATED.
    flags: AtomicU32,
    /// 64-bit generation counter for ABA prevention.
    ///
    /// Upgraded from `AtomicU32` (POOL_VERSION 1) to `AtomicU64` (POOL_VERSION 2).
    /// Incremented each time the slot is allocated.  At u64::MAX ≈ 1.8×10¹⁹
    /// allocations, practical wraparound is impossible.
    generation: AtomicU64,
    /// Byte offset from the data region base to this slot's tensor data.
    offset: u64,
    /// Allocation size in bytes.
    size: u64,
    /// Next free slot index for the Treiber free-stack.
    next_free: AtomicU32,
    /// Padding to bring struct size to 40 bytes (multiple of 8-byte alignment).
    _padding: u32,
}

/// Memory allocator backend for pool data region.
///
/// Controls how the data region (where tensor bytes live) is allocated.
/// Metadata (header + slots) always uses mmap for cross-process discovery.
#[derive(Clone, Debug, Default)]
pub enum PoolAllocator {
    /// Standard mmap-backed shared memory (default).
    /// Works everywhere; data lives in a file-backed mmap.
    #[default]
    Mmap,
}

/// Configuration for tensor pool
#[derive(Clone, Debug)]
pub struct TensorPoolConfig {
    /// Total pool size in bytes (default: 1GB)
    pub pool_size: usize,
    /// Maximum number of concurrent tensors (default: 1024)
    pub max_slots: usize,
    /// Memory alignment for tensor data (default: 64 bytes, cache-line)
    pub slot_alignment: usize,
    /// Allocator backend for the data region (default: Mmap)
    pub allocator: PoolAllocator,
}

impl Default for TensorPoolConfig {
    fn default() -> Self {
        Self {
            pool_size: 1024 * 1024 * 1024, // 1GB
            max_slots: 1024,
            slot_alignment: 64,
            allocator: PoolAllocator::default(),
        }
    }
}

/// Shared memory tensor pool
///
/// Manages a region of shared memory for tensor allocation with reference counting.
/// Multiple processes can attach to the same pool and share tensors with zero-copy.
///
/// Data is backed by mmap-based shared memory.
pub struct TensorPool {
    config: TensorPoolConfig,
    pool_id: u32,
    shm_path: PathBuf,
    mmap: MmapMut,
    _file: File,
    is_owner: bool,
    slots_offset: usize,
    data_offset: usize,

    // Process-local backpressure for `alloc_with_timeout()`.
    //
    // `slot_return_count` is incremented (Release) by `return_slot()` before it
    // acquires `slot_mutex` and calls `notify_all()`.  `alloc_with_timeout()`
    // snapshots the counter *before* each allocation attempt; on failure it
    // re-reads the counter *while holding* `slot_mutex` to decide whether to
    // sleep or retry immediately.  This prevents missed wakeups without
    // serializing the fast allocation path through a mutex.
    slot_available: std::sync::Condvar,
    slot_mutex: std::sync::Mutex<()>,
    slot_return_count: AtomicU64,
}

impl TensorPool {
    /// Create or open a tensor pool
    ///
    /// If the pool already exists, it will be opened. Otherwise, a new pool is created.
    pub fn new(pool_id: u32, config: TensorPoolConfig) -> HorusResult<Self> {
        let shm_dir = shm_base_dir().join("tensors");
        std::fs::create_dir_all(&shm_dir)?;

        let shm_path = shm_dir.join(format!("tensor_pool_{}", pool_id));

        // Calculate layout
        let header_size = std::mem::size_of::<PoolHeader>();
        let slots_size = config.max_slots * std::mem::size_of::<SlotHeader>();
        let metadata_size = header_size + slots_size;
        let data_offset = Self::align_up(metadata_size, config.slot_alignment);

        let mmap_size = data_offset + config.pool_size;

        // Try to open existing or create new
        let (file, is_owner) = if shm_path.exists() {
            let file = OpenOptions::new().read(true).write(true).open(&shm_path)?;
            let actual_size = file.metadata()?.len();
            if actual_size < mmap_size as u64 {
                file.set_len(mmap_size as u64)?;
            }
            (file, false)
        } else {
            let file = OpenOptions::new()
                .read(true)
                .write(true)
                .create(true)
                .truncate(true)
                .open(&shm_path)?;
            file.set_len(mmap_size as u64)?;
            (file, true)
        };

        // SAFETY: file is a valid open file descriptor with sufficient size for the mapping.
        let mmap = unsafe { MmapOptions::new().len(mmap_size).map_mut(&file)? };

        let mut pool = Self {
            config: config.clone(),
            pool_id,
            shm_path,
            mmap,
            _file: file,
            is_owner,
            slots_offset: header_size,
            data_offset,
            slot_available: std::sync::Condvar::new(),
            slot_mutex: std::sync::Mutex::new(()),
            slot_return_count: AtomicU64::new(0),
        };

        if is_owner {
            pool.initialize()?;
        } else {
            pool.validate()?;
        }

        Ok(pool)
    }

    /// Open an existing tensor pool (fails if pool doesn't exist)
    pub fn open(pool_id: u32) -> HorusResult<Self> {
        let shm_dir = shm_base_dir().join("tensors");
        let shm_path = shm_dir.join(format!("tensor_pool_{}", pool_id));

        if !shm_path.exists() {
            return Err(HorusError::Config(format!(
                "Tensor pool {} does not exist",
                pool_id
            )));
        }

        let file = OpenOptions::new().read(true).write(true).open(&shm_path)?;

        let metadata = file.metadata()?;
        let total_size = metadata.len() as usize;

        // SAFETY: file is a valid open file descriptor with sufficient size for the mapping.
        let mmap = unsafe { MmapOptions::new().len(total_size).map_mut(&file)? };

        // Read header to get config
        // SAFETY: mmap region is properly sized and aligned for PoolHeader; initialized at creation.
        let header = unsafe { &*(mmap.as_ptr() as *const PoolHeader) };

        if header.magic != POOL_MAGIC {
            return Err(HorusError::Config("Invalid tensor pool magic".to_string()));
        }

        let config = TensorPoolConfig {
            pool_size: header.pool_size as usize,
            max_slots: header.max_slots as usize,
            slot_alignment: header.slot_alignment as usize,
            allocator: PoolAllocator::default(), // Opened pools use mmap (data in the file)
        };

        let header_size = std::mem::size_of::<PoolHeader>();
        let slots_size = config.max_slots * std::mem::size_of::<SlotHeader>();
        let metadata_size = header_size + slots_size;
        let data_offset = Self::align_up(metadata_size, config.slot_alignment);

        Ok(Self {
            config,
            pool_id,
            shm_path,
            mmap,
            _file: file,
            is_owner: false,
            slots_offset: header_size,
            data_offset,
            slot_available: std::sync::Condvar::new(),
            slot_mutex: std::sync::Mutex::new(()),
            slot_return_count: AtomicU64::new(0),
        })
    }

    /// Initialize a newly created pool
    fn initialize(&mut self) -> HorusResult<()> {
        // Zero the entire region
        self.mmap.fill(0);

        // Copy config values to avoid borrow issues
        let pool_id = self.pool_id;
        let pool_size = self.config.pool_size as u64;
        let max_slots = self.config.max_slots as u32;
        let slot_alignment = self.config.slot_alignment as u32;

        // Initialize header
        let header = self.header_mut();
        header.magic = POOL_MAGIC;
        header.version = POOL_VERSION;
        header.pool_id = pool_id;
        header.pool_size = pool_size;
        header.max_slots = max_slots;
        header.slot_alignment = slot_alignment;
        header.next_alloc_offset.store(0, Ordering::Release);
        header
            .free_stack_head
            .store(INVALID_SLOT as u64, Ordering::Release);

        // Initialize all slots as free
        let max_slots_usize = self.config.max_slots;
        for i in 0..max_slots_usize {
            let slot = self.slot_mut(i as u32);
            slot.refcount.store(0, Ordering::Release);
            slot.flags.store(SLOT_FREE, Ordering::Release);
            slot.generation.store(0, Ordering::Release);
            slot.offset = 0;
            slot.size = 0;
            slot.next_free.store(INVALID_SLOT, Ordering::Release);
        }

        // Flush to ensure visibility
        self.mmap.flush()?;

        Ok(())
    }

    /// Validate an existing pool
    fn validate(&self) -> HorusResult<()> {
        let header = self.header();

        if header.magic != POOL_MAGIC {
            return Err(HorusError::Config("Invalid tensor pool magic".to_string()));
        }

        if header.version != POOL_VERSION {
            return Err(HorusError::Config(format!(
                "Tensor pool version mismatch: expected {}, got {}",
                POOL_VERSION, header.version
            )));
        }

        if header.pool_id != self.pool_id {
            return Err(HorusError::Config(format!(
                "Tensor pool ID mismatch: expected {}, got {}",
                self.pool_id, header.pool_id
            )));
        }

        Ok(())
    }

    /// Allocate a tensor slot
    ///
    /// Returns a Tensor descriptor pointing to the allocated memory.
    /// The device field on the descriptor is set automatically when the pool
    /// uses a managed memory allocator, otherwise it uses the caller-supplied device.
    pub fn alloc(&self, shape: &[u64], dtype: TensorDtype, device: Device) -> HorusResult<Tensor> {
        // Calculate required size — use checked arithmetic to prevent overflow.
        // A crafted shape like [u32::MAX, u32::MAX] would overflow u64 without this check,
        // causing the pool to allocate a near-zero-size region and subsequent writes to
        // corrupt memory beyond the allocation.
        let num_elements: u64 = shape.iter().copied().try_fold(1u64, |acc, x| {
            acc.checked_mul(x).ok_or(HorusError::Memory(
                "Tensor shape too large: element count overflows u64".to_string(),
            ))
        })?;
        let element_size = dtype.element_size() as u64;
        let size = num_elements.checked_mul(element_size).ok_or_else(|| {
            HorusError::Memory("Tensor too large: shape * element_size overflows u64".to_string())
        })?;
        let aligned_size = Self::align_up(size as usize, self.config.slot_alignment);

        // Find a free slot
        let slot_id = self.find_free_slot()?;
        let slot = self.slot_mut(slot_id);

        // Allocate from data region
        let offset = self.allocate_data(aligned_size)?;

        // Initialize slot — bump 64-bit generation counter to prevent ABA.
        // The returned u64 is split into low/high 32-bit halves in Tensor.
        let generation_u64 = slot.generation.fetch_add(1, Ordering::AcqRel) + 1;
        slot.offset = offset as u64;
        slot.size = size;
        slot.refcount.store(1, Ordering::Release);
        slot.flags.store(SLOT_ALLOCATED, Ordering::Release);

        // Create tensor descriptor
        Ok(Tensor::new(
            self.pool_id,
            slot_id,
            generation_u64,
            offset as u64,
            shape,
            dtype,
            device,
        ))
    }

    /// Allocate a tensor slot, blocking until one becomes free or the timeout expires.
    ///
    /// When the pool has no free slots, this method sleeps the calling thread
    /// (releasing CPU) until another thread calls `release()`/`try_release()`,
    /// which signals an internal `Condvar`.  The caller is woken and the
    /// allocation is retried.  If no slot becomes free within `timeout`, returns
    /// [`HorusError::Timeout`].
    ///
    /// The immediate [`alloc`] method is unchanged — it still returns
    /// [`HorusError::Memory`] immediately on a full pool.
    ///
    /// # Missed-wakeup safety
    ///
    /// `return_slot()` increments an atomic counter before acquiring the condvar
    /// mutex.  `alloc_with_timeout()` snapshots the counter *before* each
    /// allocation attempt and re-reads it *while holding the mutex* after a
    /// failure.  If the counter changed, the method retries immediately instead
    /// of sleeping, preventing indefinite stalls due to missed wakeups.
    ///
    /// # Errors
    ///
    /// - [`HorusError::Timeout`]  – no free slot within `timeout`
    /// - [`HorusError::Memory`]   – data region exhausted (permanent OOM, retry won't help)
    /// - Other allocation errors propagated from [`alloc`]
    pub fn alloc_with_timeout(
        &self,
        shape: &[u64],
        dtype: TensorDtype,
        device: Device,
        timeout: std::time::Duration,
    ) -> HorusResult<Tensor> {
        use std::time::Instant;

        let deadline = Instant::now() + timeout;

        loop {
            // Snapshot the return counter before the allocation attempt.
            // Using Acquire to synchronize with the Release store in return_slot().
            let count_before = self.slot_return_count.load(Ordering::Acquire);

            match self.alloc(shape, dtype, device) {
                Ok(tensor) => return Ok(tensor),
                Err(HorusError::Memory(ref msg)) if msg.contains("No free tensor slots") => {
                    // Transient exhaustion — wait for a slot to be returned.
                    let remaining = deadline.saturating_duration_since(Instant::now());
                    if remaining.is_zero() {
                        return Err(HorusError::Timeout(
                            "Tensor pool exhausted: timed out waiting for a free slot".to_string(),
                        ));
                    }

                    // Take the mutex before re-reading the counter.  If return_slot()
                    // ran between our snapshot above and here, it will have:
                    //   (a) incremented the counter (Release), then
                    //   (b) acquired the mutex (Acquire).
                    // Because (b) happens-after (a), any load of the counter while
                    // holding the mutex sees the updated value.
                    let guard = self.slot_mutex.lock().unwrap_or_else(|e| e.into_inner());

                    if self.slot_return_count.load(Ordering::Acquire) != count_before {
                        // A slot was freed since our last allocation attempt — retry
                        // without sleeping (guard drops here, releasing the mutex).
                        continue;
                    }

                    // No slot was freed; sleep until notified or timeout.
                    // `wait_timeout` atomically releases the mutex and sleeps.
                    let _ = self
                        .slot_available
                        .wait_timeout(guard, remaining)
                        .unwrap_or_else(|e| e.into_inner());
                    // Loop: retry alloc() after wakeup.
                }
                Err(e) => return Err(e),
            }
        }
    }

    /// Increment reference count for a tensor.
    ///
    /// Silently ignores mismatched pool IDs or stale generation counters.
    /// Use [`try_retain`] to get an explicit error on mismatch.
    #[inline]
    pub fn retain(&self, tensor: &Tensor) {
        if tensor.pool_id != self.pool_id {
            return;
        }

        let slot = self.slot(tensor.slot_id);

        // Verify full 64-bit generation matches (ABA prevention).
        if slot.generation.load(Ordering::Acquire) != tensor.generation_full() {
            return;
        }

        slot.refcount.fetch_add(1, Ordering::AcqRel);
    }

    /// Increment reference count, returning an error on pool-ID or generation mismatch.
    ///
    /// Unlike [`retain`] which silently ignores stale descriptors, this method
    /// returns a [`HorusError::Memory`] if the slot has been freed and
    /// reallocated since the descriptor was issued.
    pub fn try_retain(&self, tensor: &Tensor) -> HorusResult<()> {
        if tensor.pool_id != self.pool_id {
            return Err(HorusError::Memory(format!(
                "Pool ID mismatch: tensor belongs to pool {}, this pool is {}",
                tensor.pool_id, self.pool_id
            )));
        }
        let slot = self.slot(tensor.slot_id);
        let slot_gen = slot.generation.load(Ordering::Acquire);
        if slot_gen != tensor.generation_full() {
            return Err(HorusError::Memory(format!(
                "Generation mismatch on retain: tensor generation {} != slot generation {} \
                 (slot has been freed and reallocated — potential use-after-free)",
                tensor.generation_full(),
                slot_gen
            )));
        }
        slot.refcount.fetch_add(1, Ordering::AcqRel);
        Ok(())
    }

    /// Decrement reference count for a tensor.
    ///
    /// If the count reaches zero, the slot is returned to the free list.
    /// Silently ignores mismatched pool IDs or stale generation counters.
    /// Use [`try_release`] to get an explicit error on mismatch.
    #[inline]
    pub fn release(&self, tensor: &Tensor) {
        if tensor.pool_id != self.pool_id {
            return;
        }

        let slot = self.slot_mut(tensor.slot_id);

        // Verify full 64-bit generation matches (ABA prevention).
        if slot.generation.load(Ordering::Acquire) != tensor.generation_full() {
            return;
        }

        let prev = slot.refcount.fetch_sub(1, Ordering::AcqRel);
        if prev == 1 {
            // Last reference — return slot to free list.
            self.return_slot(tensor.slot_id);
        }
    }

    /// Decrement reference count, returning an error on pool-ID or generation mismatch.
    ///
    /// Unlike [`release`] which silently ignores stale descriptors, this method
    /// returns a [`HorusError::Memory`] if the slot has been freed and
    /// reallocated since the descriptor was issued.  If the count reaches zero
    /// the slot is returned to the free list.
    pub fn try_release(&self, tensor: &Tensor) -> HorusResult<()> {
        if tensor.pool_id != self.pool_id {
            return Err(HorusError::Memory(format!(
                "Pool ID mismatch: tensor belongs to pool {}, this pool is {}",
                tensor.pool_id, self.pool_id
            )));
        }
        let slot = self.slot_mut(tensor.slot_id);
        let slot_gen = slot.generation.load(Ordering::Acquire);
        if slot_gen != tensor.generation_full() {
            return Err(HorusError::Memory(format!(
                "Generation mismatch on release: tensor generation {} != slot generation {} \
                 (slot has been freed and reallocated — potential use-after-free)",
                tensor.generation_full(),
                slot_gen
            )));
        }
        let prev = slot.refcount.fetch_sub(1, Ordering::AcqRel);
        if prev == 1 {
            self.return_slot(tensor.slot_id);
        }
        Ok(())
    }

    /// Get the base data pointer.
    #[inline]
    fn data_base_ptr(&self) -> *const u8 {
        // SAFETY: data_offset is within bounds of the mmap region.
        unsafe { self.mmap.as_ptr().add(self.data_offset) }
    }

    /// Get the effective data region size for bounds checking.
    #[inline]
    fn data_region_size(&self) -> usize {
        self.mmap.len().saturating_sub(self.data_offset)
    }

    /// Get raw pointer to tensor data
    #[inline]
    pub fn data_ptr(&self, tensor: &Tensor) -> *mut u8 {
        if tensor.pool_id != self.pool_id {
            return std::ptr::null_mut();
        }

        // SAFETY: tensor.offset is within the data region (managed or mmap).
        unsafe { self.data_base_ptr().add(tensor.offset as usize) as *mut u8 }
    }

    /// Get data as a byte slice.
    ///
    /// # Errors
    ///
    /// Returns [`HorusError::Memory`] if:
    /// - `tensor.pool_id` does not match this pool's ID (descriptor from a different pool)
    /// - `tensor.offset + tensor.size` exceeds the pool's data region (out-of-bounds access)
    ///
    /// Callers that hold a properly initialized tensor descriptor should never see
    /// these errors during normal operation.  They indicate either a programming
    /// error (wrong pool) or descriptor corruption / tampering from another process.
    pub fn data_slice(&self, tensor: &Tensor) -> HorusResult<&[u8]> {
        if tensor.pool_id != self.pool_id {
            return Err(HorusError::Memory(format!(
                "pool_id mismatch in data_slice: tensor belongs to pool {}, this pool is {}",
                tensor.pool_id, self.pool_id
            )));
        }

        let offset = tensor.offset as usize;
        let size = tensor.size as usize;
        let region_size = self.data_region_size();
        if offset.checked_add(size).is_none_or(|end| end > region_size) {
            return Err(HorusError::Memory(format!(
                "out-of-bounds data access in data_slice: offset={} size={} region_size={}",
                offset, size, region_size
            )));
        }

        // SAFETY: base + offset + size is within the data region (bounds-checked above).
        Ok(unsafe {
            let ptr = self.data_base_ptr().add(offset);
            std::slice::from_raw_parts(ptr, size)
        })
    }

    /// Get data as a mutable byte slice.
    ///
    /// # Errors
    ///
    /// Returns [`HorusError::Memory`] if:
    /// - `tensor.pool_id` does not match this pool's ID (descriptor from a different pool)
    /// - `tensor.offset + tensor.size` exceeds the pool's data region (out-of-bounds access)
    #[allow(clippy::mut_from_ref)]
    pub fn data_slice_mut(&self, tensor: &Tensor) -> HorusResult<&mut [u8]> {
        if tensor.pool_id != self.pool_id {
            return Err(HorusError::Memory(format!(
                "pool_id mismatch in data_slice_mut: tensor belongs to pool {}, this pool is {}",
                tensor.pool_id, self.pool_id
            )));
        }

        let offset = tensor.offset as usize;
        let size = tensor.size as usize;
        let region_size = self.data_region_size();
        if offset.checked_add(size).is_none_or(|end| end > region_size) {
            return Err(HorusError::Memory(format!(
                "out-of-bounds data access in data_slice_mut: offset={} size={} region_size={}",
                offset, size, region_size
            )));
        }

        // SAFETY: base + offset + size is within the data region (bounds-checked above).
        Ok(unsafe {
            let ptr = self.data_base_ptr().add(offset) as *mut u8;
            std::slice::from_raw_parts_mut(ptr, size)
        })
    }

    /// Validate a `Tensor` descriptor received from another process.
    ///
    /// Performs all structural integrity checks before any data access:
    ///
    /// | Check | Prevents |
    /// |-------|---------|
    /// | `pool_id` match | Descriptor targeting a different pool |
    /// | `slot_id` in range | Out-of-bounds slot index |
    /// | slot `ALLOCATED` flag | Accessing a free/uninitialized slot |
    /// | generation match | ABA / use-after-free after slot recycle |
    /// | `offset` / `size` match slot's records | Tampered data location fields |
    /// | `offset + size ≤ region_size` | Out-of-bounds data access |
    ///
    /// Returns `Ok(())` only when all checks pass.
    /// Returns [`HorusError::InvalidDescriptor`] on any violation.
    ///
    /// Callers **must** call this before invoking [`data_slice`] or
    /// [`data_slice_mut`] on descriptors received from untrusted processes.
    pub fn validate_descriptor(&self, tensor: &Tensor) -> HorusResult<()> {
        // 1. Pool ID
        if tensor.pool_id != self.pool_id {
            return Err(HorusError::InvalidDescriptor(format!(
                "pool_id mismatch: descriptor has {}, this pool is {}",
                tensor.pool_id, self.pool_id
            )));
        }

        // 2. Slot index in range
        let max_slots = self.config.max_slots as u32;
        if tensor.slot_id >= max_slots {
            return Err(HorusError::InvalidDescriptor(format!(
                "slot_id {} is out of range (pool has {} slots)",
                tensor.slot_id, max_slots
            )));
        }

        let slot = self.slot(tensor.slot_id);

        // 3. Slot must be ALLOCATED
        if slot.flags.load(Ordering::Acquire) != SLOT_ALLOCATED {
            return Err(HorusError::InvalidDescriptor(format!(
                "slot {} is not in ALLOCATED state (may have been freed)",
                tensor.slot_id
            )));
        }

        // 4. Generation must match (prevents use-after-free / ABA)
        let slot_gen = slot.generation.load(Ordering::Acquire);
        if slot_gen != tensor.generation_full() {
            return Err(HorusError::InvalidDescriptor(format!(
                "generation mismatch for slot {}: descriptor has {}, slot has {} \
                 (slot was freed and reallocated since descriptor was issued)",
                tensor.slot_id,
                tensor.generation_full(),
                slot_gen
            )));
        }

        // 5. Offset must match slot's recorded offset
        if tensor.offset != slot.offset {
            return Err(HorusError::InvalidDescriptor(format!(
                "offset mismatch for slot {}: descriptor has {}, slot records {}",
                tensor.slot_id, tensor.offset, slot.offset
            )));
        }

        // 6. Size must match slot's recorded size
        if tensor.size != slot.size {
            return Err(HorusError::InvalidDescriptor(format!(
                "size mismatch for slot {}: descriptor has {}, slot records {}",
                tensor.slot_id, tensor.size, slot.size
            )));
        }

        // 7. offset + size must be within the data region
        let region_size = self.data_region_size() as u64;
        match tensor.offset.checked_add(tensor.size) {
            None => {
                return Err(HorusError::InvalidDescriptor(format!(
                    "descriptor offset ({}) + size ({}) overflows u64",
                    tensor.offset, tensor.size
                )));
            }
            Some(end) if end > region_size => {
                return Err(HorusError::InvalidDescriptor(format!(
                    "descriptor data range {}..{} exceeds pool data region size {}",
                    tensor.offset, end, region_size
                )));
            }
            _ => {}
        }

        Ok(())
    }

    /// Get reference count for a tensor.
    ///
    /// Returns 0 if the pool ID doesn't match or the generation is stale.
    pub fn refcount(&self, tensor: &Tensor) -> u32 {
        if tensor.pool_id != self.pool_id {
            return 0;
        }

        let slot = self.slot(tensor.slot_id);
        if slot.generation.load(Ordering::Acquire) != tensor.generation_full() {
            return 0;
        }

        slot.refcount.load(Ordering::Acquire)
    }

    /// Get pool statistics
    pub fn stats(&self) -> TensorPoolStats {
        let header = self.header();
        let mut allocated_slots = 0;
        let mut total_refcount = 0;

        for i in 0..self.config.max_slots {
            let slot = self.slot(i as u32);
            let flags = slot.flags.load(Ordering::Relaxed);
            if flags != SLOT_FREE {
                allocated_slots += 1;
                total_refcount += slot.refcount.load(Ordering::Relaxed);
            }
        }

        let used_bytes = header.next_alloc_offset.load(Ordering::Relaxed) as usize;

        TensorPoolStats {
            pool_id: self.pool_id,
            pool_size: self.config.pool_size,
            max_slots: self.config.max_slots,
            allocated_slots,
            total_refcount,
            used_bytes,
            free_bytes: self.config.pool_size.saturating_sub(used_bytes),
        }
    }

    /// Get pool ID
    #[inline]
    pub fn pool_id(&self) -> u32 {
        self.pool_id
    }

    /// Get path to the shared memory file
    pub fn shm_path(&self) -> &std::path::Path {
        &self.shm_path
    }

    /// Check if this instance created the pool
    pub fn is_owner(&self) -> bool {
        self.is_owner
    }

    // === Private helpers ===

    fn header(&self) -> &PoolHeader {
        // SAFETY: mmap region is properly sized and aligned for PoolHeader; initialized at creation.
        unsafe { &*(self.mmap.as_ptr() as *const PoolHeader) }
    }

    fn header_mut(&mut self) -> &mut PoolHeader {
        // SAFETY: exclusive access via &mut self; mmap is properly sized and aligned for PoolHeader.
        unsafe { &mut *(self.mmap.as_mut_ptr() as *mut PoolHeader) }
    }

    fn slot(&self, index: u32) -> &SlotHeader {
        let offset = self.slots_offset + (index as usize) * std::mem::size_of::<SlotHeader>();
        // SAFETY: offset is within bounds of mmap region; properly aligned for SlotHeader.
        unsafe { &*(self.mmap.as_ptr().add(offset) as *const SlotHeader) }
    }

    #[allow(clippy::mut_from_ref)]
    fn slot_mut(&self, index: u32) -> &mut SlotHeader {
        let offset = self.slots_offset + (index as usize) * std::mem::size_of::<SlotHeader>();
        // SAFETY: offset is within bounds of mmap region; properly aligned for SlotHeader.
        unsafe { &mut *(self.mmap.as_ptr().add(offset) as *mut SlotHeader) }
    }

    fn find_free_slot(&self) -> HorusResult<u32> {
        // Try to pop from free stack first (Treiber stack with ABA prevention
        // via generation counter in upper 32 bits of tagged head)
        let header = self.header();

        loop {
            let tagged_head = header.free_stack_head.load(Ordering::Acquire);
            let (generation, slot_id) = unpack_tagged_head(tagged_head);
            if slot_id != INVALID_SLOT {
                let slot = self.slot(slot_id);
                let next = slot.next_free.load(Ordering::Acquire);

                let new_tagged = pack_tagged_head(generation.wrapping_add(1), next);
                if header
                    .free_stack_head
                    .compare_exchange_weak(
                        tagged_head,
                        new_tagged,
                        Ordering::AcqRel,
                        Ordering::Relaxed,
                    )
                    .is_ok()
                {
                    return Ok(slot_id);
                }
                continue;
            }
            break;
        }

        // No free slots in stack, search linearly
        for i in 0..self.config.max_slots {
            let slot = self.slot(i as u32);
            let flags = slot.flags.load(Ordering::Acquire);

            if flags == SLOT_FREE {
                // Try to claim this slot
                if slot
                    .flags
                    .compare_exchange(
                        SLOT_FREE,
                        SLOT_ALLOCATED,
                        Ordering::AcqRel,
                        Ordering::Relaxed,
                    )
                    .is_ok()
                {
                    return Ok(i as u32);
                }
            }
        }

        Err(HorusError::Memory(
            "No free tensor slots available".to_string(),
        ))
    }

    fn return_slot(&self, slot_id: u32) {
        let header = self.header();
        let slot = self.slot_mut(slot_id);

        // ── Security: zero the data region before marking the slot free ──────
        // Without this, Process B can alloc the same slot and read Process A's
        // data (sensor readings, camera frames, key material) left behind.
        //
        // We use volatile writes so the compiler cannot treat the stores as
        // dead writes and elide them — even in optimized builds where the
        // compiler can see the memory is not read before it's reallocated.
        //
        // Ordering:
        //   1. volatile_zero()  — scrub all data bytes
        //   2. compiler_fence(SeqCst) — prevent reordering with flag store
        //   3. slot.flags = SLOT_FREE  — now visible to other allocators
        //   4. push to free stack  — slot becomes allocatable
        let data_offset = slot.offset as usize;
        let data_size = slot.size as usize;
        if data_size > 0 {
            // SAFETY: slot.offset and slot.size describe this slot's reserved
            // region within the mmap.  We still hold the only reference (refcount
            // just reached 0) so the region is exclusively ours until the flag
            // store below.
            let data_ptr = unsafe { self.data_base_ptr().add(data_offset) as *mut u8 };
            Self::volatile_zero(data_ptr, data_size);
            core::sync::atomic::compiler_fence(Ordering::SeqCst);
        }

        // Mark as free
        slot.flags.store(SLOT_FREE, Ordering::Release);

        // Push to free stack (ABA-safe via generation counter)
        loop {
            let tagged_head = header.free_stack_head.load(Ordering::Acquire);
            let (generation, current_head) = unpack_tagged_head(tagged_head);
            slot.next_free.store(current_head, Ordering::Release);

            let new_tagged = pack_tagged_head(generation.wrapping_add(1), slot_id);
            if header
                .free_stack_head
                .compare_exchange_weak(tagged_head, new_tagged, Ordering::AcqRel, Ordering::Relaxed)
                .is_ok()
            {
                break;
            }
        }

        // Notify any in-process threads waiting in `alloc_with_timeout()`.
        //
        // We increment `slot_return_count` with Release ordering *before* taking
        // `slot_mutex`, so any waiter that reads the count while holding the mutex
        // sees the updated value and can skip the wait.  This prevents missed
        // wakeups without requiring the waiter to hold the mutex during allocation.
        self.slot_return_count.fetch_add(1, Ordering::Release);
        let _guard = self.slot_mutex.lock().unwrap_or_else(|e| e.into_inner());
        self.slot_available.notify_all();
    }

    /// Zero a memory region using per-byte volatile writes.
    ///
    /// Volatile semantics prevent the optimizer from treating the stores as
    /// dead writes and removing them, which could otherwise happen when the
    /// compiler can prove the buffer is not read again before reallocation.
    ///
    /// # Safety
    ///
    /// `ptr..ptr+len` must be a valid, exclusively-owned, writable region.
    #[inline]
    fn volatile_zero(ptr: *mut u8, len: usize) {
        // SAFETY: caller guarantees ptr..ptr+len is valid and exclusively owned.
        for i in 0..len {
            unsafe { ptr.add(i).write_volatile(0u8) };
        }
    }

    fn allocate_data(&self, size: usize) -> HorusResult<usize> {
        let header = self.header();

        loop {
            let current = header.next_alloc_offset.load(Ordering::Acquire) as usize;
            let aligned_current = Self::align_up(current, self.config.slot_alignment);
            // Checked add prevents offset overflow on pathological concurrent allocations.
            let new_offset = aligned_current.checked_add(size).ok_or_else(|| {
                HorusError::Memory("Tensor pool: allocation offset overflow".to_string())
            })?;

            if new_offset > self.config.pool_size {
                return Err(HorusError::Memory(format!(
                    "Tensor pool out of memory: need {} bytes, only {} available",
                    size,
                    self.config.pool_size.saturating_sub(current)
                )));
            }

            if header
                .next_alloc_offset
                .compare_exchange_weak(
                    current as u64,
                    new_offset as u64,
                    Ordering::AcqRel,
                    Ordering::Relaxed,
                )
                .is_ok()
            {
                return Ok(aligned_current);
            }
        }
    }

    #[inline]
    fn align_up(value: usize, alignment: usize) -> usize {
        // Use wrapping_add to avoid overflow in the standard bit-manipulation trick.
        // The mask ensures the result is always aligned regardless of wrap.
        value.wrapping_add(alignment - 1) & !(alignment - 1)
    }
}

impl Drop for TensorPool {
    fn drop(&mut self) {
        // Don't delete the shm file - other processes may still be using it.
        // The file can be cleaned up manually or by a cleanup routine.
    }
}

// Thread safety
unsafe impl Send for TensorPool {}
unsafe impl Sync for TensorPool {}

/// Statistics for a tensor pool
#[derive(Clone, Debug)]
pub struct TensorPoolStats {
    pub pool_id: u32,
    pub pool_size: usize,
    pub max_slots: usize,
    pub allocated_slots: usize,
    pub total_refcount: u32,
    pub used_bytes: usize,
    pub free_bytes: usize,
}

// Re-export tensor types from types module
pub use crate::types::{Device, Tensor, TensorDtype};

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_pool_creation() {
        let config = TensorPoolConfig {
            pool_size: 1024 * 1024, // 1MB for testing
            max_slots: 16,
            slot_alignment: 64,
            allocator: Default::default(),
        };

        let pool = TensorPool::new(9999, config).expect("Failed to create pool");
        assert_eq!(pool.pool_id(), 9999);

        let stats = pool.stats();
        assert_eq!(stats.allocated_slots, 0);
        assert_eq!(stats.max_slots, 16);

        // Clean up
        std::fs::remove_file(&pool.shm_path).ok();
    }

    #[test]
    fn test_alloc_and_release() {
        let config = TensorPoolConfig {
            pool_size: 1024 * 1024,
            max_slots: 16,
            slot_alignment: 64,
            allocator: Default::default(),
        };

        let pool = TensorPool::new(9998, config).expect("Failed to create pool");

        // Allocate a tensor
        let tensor = pool
            .alloc(&[100, 100], TensorDtype::F32, Device::cpu())
            .expect("Failed to allocate tensor");

        assert_eq!(tensor.pool_id, 9998);
        assert_eq!(pool.refcount(&tensor), 1);

        // Retain
        pool.retain(&tensor);
        assert_eq!(pool.refcount(&tensor), 2);

        // Release
        pool.release(&tensor);
        assert_eq!(pool.refcount(&tensor), 1);

        pool.release(&tensor);
        assert_eq!(pool.refcount(&tensor), 0);

        // Clean up
        std::fs::remove_file(&pool.shm_path).ok();
    }

    #[test]
    fn test_data_access() {
        let config = TensorPoolConfig {
            pool_size: 1024 * 1024,
            max_slots: 16,
            slot_alignment: 64,
            allocator: Default::default(),
        };

        let pool = TensorPool::new(9997, config).expect("Failed to create pool");

        let tensor = pool
            .alloc(&[10], TensorDtype::U8, Device::cpu())
            .expect("Failed to allocate tensor");

        // Write data
        let data = pool.data_slice_mut(&tensor).unwrap();
        for (i, byte) in data.iter_mut().enumerate() {
            *byte = i as u8;
        }

        // Read data
        let data = pool.data_slice(&tensor).unwrap();
        for (i, &byte) in data.iter().enumerate() {
            assert_eq!(byte, i as u8);
        }

        pool.release(&tensor);
        std::fs::remove_file(&pool.shm_path).ok();
    }

    #[test]
    fn test_alloc_overflow_shape_returns_err() {
        // Regression test: shape products that overflow u64 must return Err
        // rather than wrapping and allocating a near-zero-size region.
        //
        // Note: (u32::MAX)^2 = 2^64 - 2^33 + 1, which fits in u64 (just a huge valid
        // number), so it triggers "out of memory" not an overflow error. We need
        // values whose product actually wraps past 2^64.
        let config = TensorPoolConfig {
            pool_size: 1024 * 1024,
            max_slots: 16,
            slot_alignment: 64,
            allocator: Default::default(),
        };
        let pool = TensorPool::new(9996, config).expect("Failed to create pool");

        // (u64::MAX/2 + 1) * 2 = u64::MAX + 1 → true u64 overflow.
        let half_max_plus_one = u64::MAX / 2 + 1;
        let result = pool.alloc(&[half_max_plus_one, 2], TensorDtype::U8, Device::cpu());
        assert!(
            result.is_err(),
            "Expected Err for overflowing shape product, got Ok"
        );
        match result.unwrap_err() {
            HorusError::Memory(msg) => {
                assert!(
                    msg.contains("overflows") || msg.contains("too large"),
                    "Unexpected error message: {msg}"
                );
            }
            other => panic!("Expected HorusError::Memory, got {other:?}"),
        }

        // element_size overflow: F64 element_size=8; (u64::MAX/8 + 1) * 8 = u64::MAX+1.
        let overflow_dim = u64::MAX / 8 + 1;
        let result2 = pool.alloc(&[overflow_dim], TensorDtype::F64, Device::cpu());
        assert!(result2.is_err(), "Expected Err for size overflow, got Ok");
        match result2.unwrap_err() {
            HorusError::Memory(msg) => {
                assert!(
                    msg.contains("overflows") || msg.contains("too large"),
                    "Unexpected error message: {msg}"
                );
            }
            other => panic!("Expected HorusError::Memory, got {other:?}"),
        }

        std::fs::remove_file(&pool.shm_path).ok();
    }

    // ── validate_descriptor tests ──────────────────────────────────────────

    fn make_test_pool(id: u32) -> TensorPool {
        let config = TensorPoolConfig {
            pool_size: 1024 * 1024,
            max_slots: 8,
            slot_alignment: 64,
            allocator: Default::default(),
        };
        TensorPool::new(id, config).expect("Failed to create test pool")
    }

    #[test]
    fn test_validate_descriptor_ok_on_live_tensor() {
        let pool = make_test_pool(9990);
        let tensor = pool
            .alloc(&[32], TensorDtype::U8, Device::cpu())
            .expect("alloc failed");
        assert!(
            pool.validate_descriptor(&tensor).is_ok(),
            "validate must succeed for a freshly allocated tensor"
        );
        pool.release(&tensor);
        std::fs::remove_file(&pool.shm_path).ok();
    }

    #[test]
    fn test_validate_descriptor_rejects_wrong_pool_id() {
        let pool = make_test_pool(9989);
        let tensor = pool
            .alloc(&[16], TensorDtype::U8, Device::cpu())
            .expect("alloc failed");
        // Craft a descriptor with the wrong pool_id.
        let mut bad = tensor;
        bad.pool_id = 9999;
        match pool.validate_descriptor(&bad).unwrap_err() {
            HorusError::InvalidDescriptor(msg) => {
                assert!(
                    msg.contains("pool_id"),
                    "error should mention pool_id: {msg}"
                );
            }
            other => panic!("expected InvalidDescriptor, got {other:?}"),
        }
        pool.release(&tensor);
        std::fs::remove_file(&pool.shm_path).ok();
    }

    #[test]
    fn test_validate_descriptor_rejects_out_of_range_slot_id() {
        let pool = make_test_pool(9988);
        let tensor = pool
            .alloc(&[16], TensorDtype::U8, Device::cpu())
            .expect("alloc failed");
        let mut bad = tensor;
        bad.slot_id = u32::MAX; // way out of range
        match pool.validate_descriptor(&bad).unwrap_err() {
            HorusError::InvalidDescriptor(msg) => {
                assert!(
                    msg.contains("slot_id") || msg.contains("out of range"),
                    "error should mention slot_id: {msg}"
                );
            }
            other => panic!("expected InvalidDescriptor, got {other:?}"),
        }
        pool.release(&tensor);
        std::fs::remove_file(&pool.shm_path).ok();
    }

    #[test]
    fn test_validate_descriptor_rejects_freed_slot() {
        let pool = make_test_pool(9987);
        let tensor = pool
            .alloc(&[16], TensorDtype::U8, Device::cpu())
            .expect("alloc failed");
        pool.release(&tensor); // slot is now free
                               // The descriptor still refers to the (now-freed) slot.
        match pool.validate_descriptor(&tensor).unwrap_err() {
            HorusError::InvalidDescriptor(msg) => {
                // Either the ALLOCATED check or the generation check fires —
                // either is a valid security signal.
                assert!(
                    msg.contains("ALLOCATED") || msg.contains("generation") || msg.contains("free"),
                    "error should mention freed state: {msg}"
                );
            }
            other => panic!("expected InvalidDescriptor, got {other:?}"),
        }
        std::fs::remove_file(&pool.shm_path).ok();
    }

    #[test]
    fn test_validate_descriptor_rejects_tampered_offset() {
        let pool = make_test_pool(9986);
        let tensor = pool
            .alloc(&[16], TensorDtype::U8, Device::cpu())
            .expect("alloc failed");
        let mut bad = tensor;
        bad.offset = tensor.offset.wrapping_add(128); // tamper
        match pool.validate_descriptor(&bad).unwrap_err() {
            HorusError::InvalidDescriptor(msg) => {
                assert!(msg.contains("offset"), "error should mention offset: {msg}");
            }
            other => panic!("expected InvalidDescriptor, got {other:?}"),
        }
        pool.release(&tensor);
        std::fs::remove_file(&pool.shm_path).ok();
    }

    #[test]
    fn test_validate_descriptor_rejects_tampered_size() {
        let pool = make_test_pool(9985);
        let tensor = pool
            .alloc(&[16], TensorDtype::U8, Device::cpu())
            .expect("alloc failed");
        let mut bad = tensor;
        bad.size = u64::MAX; // overflow-inducing crafted size
        match pool.validate_descriptor(&bad).unwrap_err() {
            HorusError::InvalidDescriptor(msg) => {
                assert!(msg.contains("size"), "error should mention size: {msg}");
            }
            other => panic!("expected InvalidDescriptor, got {other:?}"),
        }
        pool.release(&tensor);
        std::fs::remove_file(&pool.shm_path).ok();
    }

    #[test]
    fn test_slot_data_zeroed_on_release() {
        // Security regression: freed slot data must be zeroed before the slot
        // is returned to the free list.  Without zeroing, a subsequent process
        // that re-allocates the same slot can read the previous tenant's data.
        let config = TensorPoolConfig {
            pool_size: 1024 * 1024,
            max_slots: 2,
            slot_alignment: 64,
            allocator: Default::default(),
        };
        let pool = TensorPool::new(9991, config).expect("Failed to create pool");

        // Allocate a slot and fill it with a known non-zero pattern.
        let tensor = pool
            .alloc(&[128], TensorDtype::U8, Device::cpu())
            .expect("alloc failed");
        pool.data_slice_mut(&tensor).unwrap().fill(0xAB);
        assert!(
            pool.data_slice(&tensor).unwrap().iter().all(|&b| b == 0xAB),
            "setup: bytes should all be 0xAB before release"
        );

        // Release: triggers return_slot → volatile_zero.
        pool.release(&tensor);

        // Read the same memory location through the (now stale) descriptor.
        // data_slice() performs only bounds checks, not liveness checks, so
        // it gives us a view of the original data region regardless of whether
        // the slot is still allocated.
        let scrubbed = pool.data_slice(&tensor).unwrap();
        let non_zero_offsets: Vec<usize> = scrubbed
            .iter()
            .enumerate()
            .filter(|(_, &b)| b != 0)
            .map(|(i, _)| i)
            .collect();
        assert!(
            non_zero_offsets.is_empty(),
            "slot data must be zeroed after release; non-zero bytes at offsets: {non_zero_offsets:?}"
        );

        std::fs::remove_file(&pool.shm_path).ok();
    }

    #[test]
    fn test_generation_increments_on_reuse() {
        // Each alloc+release cycle should increment the 64-bit generation counter.
        let config = TensorPoolConfig {
            pool_size: 1024 * 1024,
            max_slots: 2,
            slot_alignment: 64,
            allocator: Default::default(),
        };
        let pool = TensorPool::new(9994, config).expect("Failed to create pool");

        // Allocate and release the same slot N times; verify generation grows.
        const CYCLES: u64 = 10;
        let mut last_gen = 0u64;
        for _ in 0..CYCLES {
            let tensor = pool
                .alloc(&[4], TensorDtype::U8, Device::cpu())
                .expect("alloc failed");
            let gen = tensor.generation_full();
            assert!(
                gen > last_gen,
                "generation_full should monotonically increase; prev={last_gen} curr={gen}"
            );
            last_gen = gen;
            pool.release(&tensor);
        }

        std::fs::remove_file(&pool.shm_path).ok();
    }

    #[test]
    fn test_stale_descriptor_rejected_by_retain_release() {
        // Allocating a new tensor into a reused slot must make the old descriptor stale.
        let config = TensorPoolConfig {
            pool_size: 1024 * 1024,
            max_slots: 1, // Force slot 0 to always be reused
            slot_alignment: 64,
            allocator: Default::default(),
        };
        let pool = TensorPool::new(9993, config).expect("Failed to create pool");

        // First allocation — capture a stale copy of the descriptor.
        let first = pool
            .alloc(&[8], TensorDtype::U8, Device::cpu())
            .expect("first alloc failed");
        let stale = first; // stale descriptor
        pool.release(&first); // slot returned to free list

        // Second allocation reuses the same slot with a bumped generation.
        let second = pool
            .alloc(&[8], TensorDtype::U8, Device::cpu())
            .expect("second alloc failed");
        assert_eq!(second.slot_id, stale.slot_id, "same slot should be reused");
        assert!(
            second.generation_full() > stale.generation_full(),
            "new generation should exceed old"
        );

        // Stale descriptor: retain and release should silently do nothing.
        pool.retain(&stale);
        assert_eq!(
            pool.refcount(&second),
            1,
            "stale retain must not change refcount of live descriptor"
        );
        pool.release(&stale);
        assert_eq!(
            pool.refcount(&second),
            1,
            "stale release must not change refcount of live descriptor"
        );

        // try_retain and try_release should return errors for the stale descriptor.
        assert!(
            pool.try_retain(&stale).is_err(),
            "try_retain with stale descriptor must return Err"
        );
        assert!(
            pool.try_release(&stale).is_err(),
            "try_release with stale descriptor must return Err"
        );

        pool.release(&second);
        std::fs::remove_file(&pool.shm_path).ok();
    }

    #[test]
    fn test_try_retain_release_succeed_on_live_descriptor() {
        let config = TensorPoolConfig {
            pool_size: 1024 * 1024,
            max_slots: 4,
            slot_alignment: 64,
            allocator: Default::default(),
        };
        let pool = TensorPool::new(9992, config).expect("Failed to create pool");

        let tensor = pool
            .alloc(&[16], TensorDtype::F32, Device::cpu())
            .expect("alloc failed");

        assert!(
            pool.try_retain(&tensor).is_ok(),
            "try_retain on live descriptor should succeed"
        );
        assert_eq!(pool.refcount(&tensor), 2);

        assert!(
            pool.try_release(&tensor).is_ok(),
            "try_release on live descriptor should succeed"
        );
        assert_eq!(pool.refcount(&tensor), 1);

        pool.release(&tensor);
        std::fs::remove_file(&pool.shm_path).ok();
    }

    // ── alloc_with_timeout tests ───────────────────────────────────────────

    /// alloc_with_timeout returns Ok when a slot is freed within the deadline.
    ///
    /// Scenario: pool has 1 slot.  Main thread allocates it, spawns a helper
    /// that releases it after 50 ms, then calls alloc_with_timeout(200 ms).
    /// Expect the timed allocation to succeed and return well under the deadline.
    #[test]
    fn test_alloc_with_timeout_succeeds_when_slot_freed() {
        use std::sync::Arc;
        use std::time::{Duration, Instant};

        let config = TensorPoolConfig {
            pool_size: 1024 * 1024,
            max_slots: 1, // Only one slot — easy to exhaust
            slot_alignment: 64,
            allocator: Default::default(),
        };
        let pool = Arc::new(TensorPool::new(9980, config).expect("Failed to create pool"));

        // Exhaust the pool's single slot.
        let tensor = pool
            .alloc(&[64], TensorDtype::U8, Device::cpu())
            .expect("initial alloc failed");
        assert_eq!(pool.stats().allocated_slots, 1);

        // Spawn a thread that releases the slot after 50 ms.
        let pool2 = Arc::clone(&pool);
        let tensor_for_thread = tensor;
        let handle = std::thread::spawn(move || {
            std::thread::sleep(Duration::from_millis(50));
            pool2.release(&tensor_for_thread);
        });

        // alloc_with_timeout should block until the helper releases the slot.
        let t0 = Instant::now();
        let result = pool.alloc_with_timeout(
            &[64],
            TensorDtype::U8,
            Device::cpu(),
            Duration::from_millis(200),
        );
        let elapsed = t0.elapsed();

        assert!(
            result.is_ok(),
            "alloc_with_timeout should succeed once a slot is freed: {:?}",
            result.err()
        );
        // We should have blocked at least ~50 ms (until the helper released the slot)
        // but well under the 200 ms timeout.
        assert!(
            elapsed >= Duration::from_millis(30),
            "should have blocked while waiting for slot (elapsed={elapsed:?})"
        );
        assert!(
            elapsed < Duration::from_millis(180),
            "should have been woken promptly by the condvar (elapsed={elapsed:?})"
        );

        // Clean up
        pool.release(&result.unwrap());
        handle.join().expect("helper thread panicked");
        std::fs::remove_file(&pool.shm_path).ok();
    }

    /// alloc_with_timeout returns Timeout when no slot is freed within the deadline.
    #[test]
    fn test_alloc_with_timeout_returns_timeout_when_pool_stays_full() {
        use std::time::Duration;

        let config = TensorPoolConfig {
            pool_size: 1024 * 1024,
            max_slots: 1,
            slot_alignment: 64,
            allocator: Default::default(),
        };
        let pool = TensorPool::new(9979, config).expect("Failed to create pool");

        // Exhaust the pool's single slot and hold it.
        let _tensor = pool
            .alloc(&[64], TensorDtype::U8, Device::cpu())
            .expect("initial alloc failed");

        // Nobody releases the slot — expect Timeout.
        let result = pool.alloc_with_timeout(
            &[64],
            TensorDtype::U8,
            Device::cpu(),
            Duration::from_millis(80),
        );

        assert!(
            matches!(result, Err(HorusError::Timeout(_))),
            "expected HorusError::Timeout, got {:?}",
            result
        );

        // Intentionally do NOT release `_tensor` — it's dropped here, which calls
        // release() implicitly via TensorHandle; but since we're testing the raw
        // pool API, we release manually.
        pool.release(&_tensor);
        std::fs::remove_file(&pool.shm_path).ok();
    }
}
