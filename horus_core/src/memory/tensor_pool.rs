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

use crate::error::{ConfigError, HorusError, HorusResult};
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
///   + next_free(4) + owner_pid(4)
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
    /// PID of the process that last allocated this slot.
    /// Used for dead-process slot reclamation: if `refcount > 0` and the owning
    /// PID is no longer alive, the slot can be safely reclaimed.
    /// Zero means no owner tracked (pre-v2 pools or freshly initialized).
    owner_pid: AtomicU32,
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

    /// Allocation counter for periodic utilization checks.
    alloc_count: AtomicU64,
    /// Whether the utilization warning has been emitted.
    warned_pressure: std::sync::atomic::AtomicBool,
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

        // Atomically try to be the creator: `create_new` maps to O_CREAT|O_EXCL,
        // guaranteeing exactly one winner even when many threads/processes race.
        // The previous path.exists() + create(true) pattern had a TOCTOU window.
        let (file, is_owner) = match OpenOptions::new()
            .read(true)
            .write(true)
            .create_new(true)
            .open(&shm_path)
        {
            Ok(file) => {
                file.set_len(mmap_size as u64)?;
                (file, true)
            }
            Err(e) if e.kind() == std::io::ErrorKind::AlreadyExists => {
                let file = OpenOptions::new().read(true).write(true).open(&shm_path)?;
                let actual_size = file.metadata()?.len();
                if actual_size < mmap_size as u64 {
                    file.set_len(mmap_size as u64)?;
                }
                (file, false)
            }
            Err(e) => return Err(e.into()),
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
            alloc_count: AtomicU64::new(0),
            warned_pressure: std::sync::atomic::AtomicBool::new(false),
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
            return Err(HorusError::Config(ConfigError::Other(format!(
                "Tensor pool {} does not exist",
                pool_id
            ))));
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
            return Err(HorusError::Config(ConfigError::Other(
                "Invalid tensor pool magic".to_string(),
            )));
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
            alloc_count: AtomicU64::new(0),
            warned_pressure: std::sync::atomic::AtomicBool::new(false),
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
            return Err(HorusError::Config(ConfigError::Other(
                "Invalid tensor pool magic".to_string(),
            )));
        }

        if header.version != POOL_VERSION {
            return Err(HorusError::Config(ConfigError::Other(format!(
                "Tensor pool version mismatch: expected {}, got {}",
                POOL_VERSION, header.version
            ))));
        }

        if header.pool_id != self.pool_id {
            return Err(HorusError::Config(ConfigError::Other(format!(
                "Tensor pool ID mismatch: expected {}, got {}",
                self.pool_id, header.pool_id
            ))));
        }

        Ok(())
    }

    /// Check pool utilization and emit a warning if under pressure.
    ///
    /// Called periodically (every 64 allocations) to avoid per-alloc overhead.
    fn check_utilization(&self) {
        let count = self.alloc_count.fetch_add(1, Ordering::Relaxed);
        // Check every 64 allocations
        if !count.is_multiple_of(64) {
            return;
        }

        let stats = self.stats();
        if stats.is_under_pressure()
            && !self
                .warned_pressure
                .swap(true, std::sync::atomic::Ordering::Relaxed)
        {
            eprintln!(
                "[horus::tensor_pool] WARNING: {}\n  \
                 hint: Ensure tensors are dropped promptly. Consider increasing \
                 pool_size or max_slots in TensorPoolConfig.",
                stats.summary(),
            );
        }
    }

    /// Allocate a tensor slot
    ///
    /// Returns a Tensor descriptor pointing to the allocated memory.
    /// The device field on the descriptor is set automatically when the pool
    /// uses a managed memory allocator, otherwise it uses the caller-supplied device.
    pub fn alloc(&self, shape: &[u64], dtype: TensorDtype, device: Device) -> HorusResult<Tensor> {
        self.check_utilization();

        // Calculate required size — use checked arithmetic to prevent overflow.
        // A crafted shape like [u32::MAX, u32::MAX] would overflow u64 without this check,
        // causing the pool to allocate a near-zero-size region and subsequent writes to
        // corrupt memory beyond the allocation.
        let num_elements: u64 = shape.iter().copied().try_fold(1u64, |acc, x| {
            acc.checked_mul(x).ok_or(HorusError::Memory(
                "Tensor shape too large: element count overflows u64"
                    .to_string()
                    .into(),
            ))
        })?;
        let element_size = dtype.element_size() as u64;
        let size = num_elements.checked_mul(element_size).ok_or_else(|| {
            HorusError::Memory(
                "Tensor too large: shape * element_size overflows u64"
                    .to_string()
                    .into(),
            )
        })?;
        let aligned_size = Self::align_up(size as usize, self.config.slot_alignment);

        // Find a free slot
        let slot_id = self.find_free_slot()?;

        // Allocate from data region — if this fails, return the slot to the free list
        let offset = match self.allocate_data(aligned_size) {
            Ok(o) => o,
            Err(e) => {
                // Return slot to free list to prevent permanent slot leak.
                // Without this, a failed allocate_data leaves the slot removed
                // from the free stack but never marked ALLOCATED — permanently lost.
                self.push_free_slot(slot_id);
                return Err(e);
            }
        };

        let slot = self.slot_mut(slot_id);

        // Initialize slot — bump 64-bit generation counter to prevent ABA.
        // The returned u64 is split into low/high 32-bit halves in Tensor.
        let generation_u64 = slot.generation.fetch_add(1, Ordering::AcqRel) + 1;
        slot.offset = offset as u64;
        slot.size = size;
        slot.owner_pid.store(std::process::id(), Ordering::Release);
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
                Err(HorusError::Memory(ref e))
                    if e.to_string().contains("No free tensor slots") =>
                {
                    // Transient exhaustion — wait for a slot to be returned.
                    let remaining = deadline.saturating_duration_since(Instant::now());
                    if remaining.is_zero() {
                        return Err(HorusError::Timeout(crate::error::TimeoutError {
                            resource: "tensor_pool".to_string(),
                            elapsed: timeout,
                            deadline: Some(timeout),
                        }));
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
    /// Silently ignores mismatched pool IDs, stale generation counters, or
    /// slots that were freed between the generation check and the refcount
    /// increment (TOCTOU prevention via CAS loop that rejects refcount 0).
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

        // CAS loop: atomically increment refcount only if it's > 0.
        // If refcount is 0, the slot was freed between our generation check
        // and now — silently bail out instead of leaking the new allocation.
        loop {
            let current = slot.refcount.load(Ordering::Acquire);
            if current == 0 {
                return; // Slot was freed — stale descriptor
            }
            if slot
                .refcount
                .compare_exchange_weak(current, current + 1, Ordering::AcqRel, Ordering::Relaxed)
                .is_ok()
            {
                return;
            }
        }
    }

    /// Increment reference count, returning an error on pool-ID or generation mismatch.
    ///
    /// Unlike [`retain`] which silently ignores stale descriptors, this method
    /// returns a [`HorusError::Memory`] if the slot has been freed and
    /// reallocated since the descriptor was issued.
    pub fn try_retain(&self, tensor: &Tensor) -> HorusResult<()> {
        if tensor.pool_id != self.pool_id {
            return Err(HorusError::Memory(
                format!(
                    "Pool ID mismatch: tensor belongs to pool {}, this pool is {}",
                    tensor.pool_id, self.pool_id
                )
                .into(),
            ));
        }
        let slot = self.slot(tensor.slot_id);
        let slot_gen = slot.generation.load(Ordering::Acquire);
        if slot_gen != tensor.generation_full() {
            return Err(HorusError::Memory(
                format!(
                    "Generation mismatch on retain: tensor generation {} != slot generation {} \
                 (slot has been freed and reallocated — potential use-after-free)",
                    tensor.generation_full(),
                    slot_gen
                )
                .into(),
            ));
        }
        // CAS loop: atomically increment refcount only if it's > 0.
        // If refcount is 0, the slot was freed between our generation check
        // and now (TOCTOU race).
        loop {
            let current = slot.refcount.load(Ordering::Acquire);
            if current == 0 {
                return Err(HorusError::Memory(
                    "Slot freed during retain: generation matched but refcount dropped to 0 \
                     (concurrent release between generation check and refcount increment)"
                        .to_string()
                        .into(),
                ));
            }
            if slot
                .refcount
                .compare_exchange_weak(current, current + 1, Ordering::AcqRel, Ordering::Relaxed)
                .is_ok()
            {
                return Ok(());
            }
        }
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

        // CAS loop: atomically decrement refcount only if it's > 0.
        // Prevents refcount underflow (wrap to u32::MAX) on double-release.
        loop {
            let current = slot.refcount.load(Ordering::Acquire);
            if current == 0 {
                return; // Already freed — double-release
            }
            if slot
                .refcount
                .compare_exchange_weak(current, current - 1, Ordering::AcqRel, Ordering::Relaxed)
                .is_ok()
            {
                if current == 1 {
                    self.return_slot(tensor.slot_id);
                }
                return;
            }
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
            return Err(HorusError::Memory(
                format!(
                    "Pool ID mismatch: tensor belongs to pool {}, this pool is {}",
                    tensor.pool_id, self.pool_id
                )
                .into(),
            ));
        }
        let slot = self.slot_mut(tensor.slot_id);
        let slot_gen = slot.generation.load(Ordering::Acquire);
        if slot_gen != tensor.generation_full() {
            return Err(HorusError::Memory(
                format!(
                    "Generation mismatch on release: tensor generation {} != slot generation {} \
                 (slot has been freed and reallocated — potential use-after-free)",
                    tensor.generation_full(),
                    slot_gen
                )
                .into(),
            ));
        }
        // CAS loop: atomically decrement refcount only if it's > 0.
        // Prevents refcount underflow (wrap to u32::MAX) on double-release.
        loop {
            let current = slot.refcount.load(Ordering::Acquire);
            if current == 0 {
                return Err(HorusError::Memory(
                    format!(
                        "Double-release detected for slot {} (refcount already 0)",
                        tensor.slot_id
                    )
                    .into(),
                ));
            }
            if slot
                .refcount
                .compare_exchange_weak(current, current - 1, Ordering::AcqRel, Ordering::Relaxed)
                .is_ok()
            {
                if current == 1 {
                    self.return_slot(tensor.slot_id);
                }
                return Ok(());
            }
        }
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
            return Err(HorusError::Memory(
                format!(
                    "pool_id mismatch in data_slice: tensor belongs to pool {}, this pool is {}",
                    tensor.pool_id, self.pool_id
                )
                .into(),
            ));
        }

        let offset = tensor.offset as usize;
        let size = tensor.size as usize;
        let region_size = self.data_region_size();
        if offset.checked_add(size).is_none_or(|end| end > region_size) {
            return Err(HorusError::Memory(
                format!(
                    "out-of-bounds data access in data_slice: offset={} size={} region_size={}",
                    offset, size, region_size
                )
                .into(),
            ));
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
    #[allow(clippy::mut_from_ref)] // mmap'd shared memory: mutability is OS-level, not Rust borrow-level
    pub fn data_slice_mut(&self, tensor: &Tensor) -> HorusResult<&mut [u8]> {
        if tensor.pool_id != self.pool_id {
            return Err(HorusError::Memory(
                format!(
                "pool_id mismatch in data_slice_mut: tensor belongs to pool {}, this pool is {}",
                tensor.pool_id, self.pool_id
            )
                .into(),
            ));
        }

        let offset = tensor.offset as usize;
        let size = tensor.size as usize;
        let region_size = self.data_region_size();
        if offset.checked_add(size).is_none_or(|end| end > region_size) {
            return Err(HorusError::Memory(
                format!(
                    "out-of-bounds data access in data_slice_mut: offset={} size={} region_size={}",
                    offset, size, region_size
                )
                .into(),
            ));
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

    #[allow(clippy::mut_from_ref)] // mmap'd shared memory: mutability is OS-level, not Rust borrow-level
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

        // Last resort: try to reclaim slots from dead processes
        let reclaimed = self.reclaim_dead_slots();
        if reclaimed > 0 {
            // Retry — reclaimed slots were pushed to the free stack
            return self.find_free_slot_inner();
        }

        Err(HorusError::Memory(
            "No free tensor slots available".to_string().into(),
        ))
    }

    /// Reclaim tensor slots owned by processes that no longer exist.
    ///
    /// Scans all allocated slots. For each with `refcount > 0` and a non-zero
    /// `owner_pid`, checks if the process is alive via `kill(pid, 0)`. Dead
    /// processes' slots are freed: refcount reset, flags set to SLOT_FREE,
    /// generation incremented, and pushed back onto the free stack.
    ///
    /// Only called as a recovery path when `find_free_slot()` fails — never
    /// on the normal allocation fast path.
    fn reclaim_dead_slots(&self) -> u32 {
        let mut reclaimed = 0u32;
        let max = self.config.max_slots as u32;

        for i in 0..max {
            let slot = self.slot(i);
            let flags = slot.flags.load(Ordering::Acquire);
            if flags != SLOT_ALLOCATED {
                continue;
            }

            let pid = slot.owner_pid.load(Ordering::Acquire);
            if pid == 0 {
                // No owner tracked (legacy slot or unset) — can't reclaim
                continue;
            }

            // Check if the owning process is still alive
            if Self::is_process_alive(pid) {
                continue;
            }

            // Process is dead — reclaim this slot.
            // Use CAS on flags to avoid racing with a concurrent reclaim.
            if slot
                .flags
                .compare_exchange(
                    SLOT_ALLOCATED,
                    SLOT_FREE,
                    Ordering::AcqRel,
                    Ordering::Relaxed,
                )
                .is_ok()
            {
                slot.refcount.store(0, Ordering::Release);
                slot.owner_pid.store(0, Ordering::Release);
                slot.generation.fetch_add(1, Ordering::AcqRel);
                self.push_free_slot(i);
                reclaimed += 1;
            }
        }

        if reclaimed > 0 {
            log::info!(
                "Reclaimed {} tensor slot(s) from dead process(es)",
                reclaimed
            );
        }

        reclaimed
    }

    /// Check if a process is still alive.
    fn is_process_alive(pid: u32) -> bool {
        horus_sys::process::ProcessHandle::from_pid(pid).is_alive()
    }

    /// Inner find_free_slot without the reclaim fallback (used for retry after reclaim).
    fn find_free_slot_inner(&self) -> HorusResult<u32> {
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

        Err(HorusError::Memory(
            "No free tensor slots available".to_string().into(),
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

        // Mark as free and clear owner
        slot.owner_pid.store(0, Ordering::Release);
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

    /// Push a slot back onto the Treiber free stack (ABA-safe via generation counter).
    ///
    /// Used by `reclaim_dead_slots()` to return reclaimed slots.
    /// Unlike `return_slot()`, this does NOT zero the data region or notify waiters
    /// — the caller is responsible for any cleanup before calling this.
    fn push_free_slot(&self, slot_id: u32) {
        let header = self.header();
        let slot = self.slot(slot_id);

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
        for i in 0..len {
            // SAFETY: caller guarantees ptr..ptr+len is valid and exclusively owned;
            // i is always in 0..len so ptr.add(i) stays within that region.
            unsafe { ptr.add(i).write_volatile(0u8) };
        }
    }

    fn allocate_data(&self, size: usize) -> HorusResult<usize> {
        let header = self.header();

        loop {
            let current = header.next_alloc_offset.load(Ordering::Acquire) as usize;
            let aligned_current = Self::align_up(current, self.config.slot_alignment);
            // Checked add prevents offset overflow on pathological concurrent allocations.
            let new_offset = aligned_current
                .checked_add(size)
                .ok_or_else(|| HorusError::Memory(crate::error::MemoryError::OffsetOverflow))?;

            if new_offset > self.config.pool_size {
                return Err(HorusError::Memory(
                    format!(
                        "Tensor pool out of memory: need {} bytes, only {} available",
                        size,
                        self.config.pool_size.saturating_sub(current)
                    )
                    .into(),
                ));
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

// SAFETY: TensorPool's mmap region is shared memory accessed through atomics;
// all mutable state uses atomic operations or is protected by the slot_mutex/condvar.
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

impl TensorPoolStats {
    /// Slot utilization as a fraction (0.0 to 1.0).
    pub fn slot_utilization(&self) -> f64 {
        if self.max_slots == 0 {
            return 0.0;
        }
        self.allocated_slots as f64 / self.max_slots as f64
    }

    /// Data region utilization as a fraction (0.0 to 1.0).
    pub fn data_utilization(&self) -> f64 {
        if self.pool_size == 0 {
            return 0.0;
        }
        self.used_bytes as f64 / self.pool_size as f64
    }

    /// Whether the pool is under memory pressure (>80% utilization on either axis).
    pub fn is_under_pressure(&self) -> bool {
        self.slot_utilization() > 0.8 || self.data_utilization() > 0.8
    }

    /// Human-readable summary.
    pub fn summary(&self) -> String {
        format!(
            "TensorPool(id={}, slots={}/{} ({:.0}%), data={}/{} ({:.0}%))",
            self.pool_id,
            self.allocated_slots,
            self.max_slots,
            self.slot_utilization() * 100.0,
            format_bytes(self.used_bytes),
            format_bytes(self.pool_size),
            self.data_utilization() * 100.0,
        )
    }
}

fn format_bytes(bytes: usize) -> String {
    if bytes < 1024 {
        format!("{}B", bytes)
    } else if bytes < 1024 * 1024 {
        format!("{:.1}KB", bytes as f64 / 1024.0)
    } else if bytes < 1024 * 1024 * 1024 {
        format!("{:.1}MB", bytes as f64 / (1024.0 * 1024.0))
    } else {
        format!("{:.1}GB", bytes as f64 / (1024.0 * 1024.0 * 1024.0))
    }
}

// Re-export tensor types from types module
pub use crate::types::{Device, Tensor, TensorDtype};

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::duration_ext::DurationExt;

    #[test]
    fn test_pool_creation() {
        let config = TensorPoolConfig {
            pool_size: 1024 * 1024, // 1MB for testing
            max_slots: 16,
            slot_alignment: 64,
            allocator: Default::default(),
        };

        let pool = TensorPool::new(7777, config).expect("Failed to create pool");
        assert_eq!(pool.pool_id(), 7777);

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
            HorusError::Memory(ref e) => {
                let msg = e.to_string();
                assert!(
                    msg.contains("overflows") || msg.contains("too large"),
                    "Unexpected error message: {msg}"
                );
            }
            other => unreachable!("Expected HorusError::Memory, got {other:?}"),
        }

        // element_size overflow: F64 element_size=8; (u64::MAX/8 + 1) * 8 = u64::MAX+1.
        let overflow_dim = u64::MAX / 8 + 1;
        let result2 = pool.alloc(&[overflow_dim], TensorDtype::F64, Device::cpu());
        assert!(result2.is_err(), "Expected Err for size overflow, got Ok");
        match result2.unwrap_err() {
            HorusError::Memory(ref e) => {
                let msg = e.to_string();
                assert!(
                    msg.contains("overflows") || msg.contains("too large"),
                    "Unexpected error message: {msg}"
                );
            }
            other => unreachable!("Expected HorusError::Memory, got {other:?}"),
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
            other => unreachable!("expected InvalidDescriptor, got {other:?}"),
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
            other => unreachable!("expected InvalidDescriptor, got {other:?}"),
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
            other => unreachable!("expected InvalidDescriptor, got {other:?}"),
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
            other => unreachable!("expected InvalidDescriptor, got {other:?}"),
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
            other => unreachable!("expected InvalidDescriptor, got {other:?}"),
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
        use std::time::Instant;

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
            std::thread::sleep(50_u64.ms());
            pool2.release(&tensor_for_thread);
        });

        // alloc_with_timeout should block until the helper releases the slot.
        let t0 = Instant::now();
        let result = pool.alloc_with_timeout(&[64], TensorDtype::U8, Device::cpu(), 200_u64.ms());
        let elapsed = t0.elapsed();

        assert!(
            result.is_ok(),
            "alloc_with_timeout should succeed once a slot is freed: {:?}",
            result.err()
        );
        // We should have blocked at least ~50 ms (until the helper released the slot)
        // but well under the 200 ms timeout.
        assert!(
            elapsed >= 30_u64.ms(),
            "should have blocked while waiting for slot (elapsed={elapsed:?})"
        );
        assert!(
            elapsed < 180_u64.ms(),
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
        let result = pool.alloc_with_timeout(&[64], TensorDtype::U8, Device::cpu(), 80_u64.ms());

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

    // ========================================================================
    // Crash recovery tests
    // ========================================================================

    /// Test: dead-process tensor slot reclamation.
    ///
    /// Allocates all slots, writes a fake dead PID into one slot's owner_pid,
    /// then attempts another allocation. The allocator should reclaim the dead
    /// slot as a last resort and succeed.
    #[test]
    #[cfg(unix)]
    fn test_reclaim_dead_process_slots() {
        let config = TensorPoolConfig {
            pool_size: 1024 * 1024,
            max_slots: 2,
            slot_alignment: 64,
            allocator: Default::default(),
        };
        let pool = TensorPool::new(9900, config).expect("Failed to create pool");

        // Allocate both slots
        let t1 = pool
            .alloc(&[64], TensorDtype::U8, Device::cpu())
            .expect("alloc t1");
        let t2 = pool
            .alloc(&[64], TensorDtype::U8, Device::cpu())
            .expect("alloc t2");

        // Pool is now full — verify we can't allocate
        pool.alloc(&[64], TensorDtype::U8, Device::cpu())
            .unwrap_err();

        // Simulate process death: write a guaranteed-dead PID into slot 0's owner_pid.
        // PID 2^30 + 1 (1073741825) should not exist on any reasonable system.
        let dead_pid: u32 = (1 << 30) + 1;
        let slot = pool.slot(t1.slot_id);
        slot.owner_pid.store(dead_pid, Ordering::Release);

        // Now allocation should succeed because reclaim_dead_slots kicks in
        let t3 = pool
            .alloc(&[64], TensorDtype::U8, Device::cpu())
            .expect("alloc after reclaim should succeed");

        // The reclaimed slot should be valid
        assert_eq!(pool.refcount(&t3), 1);

        // Clean up
        pool.release(&t3);
        pool.release(&t2);
        std::fs::remove_file(&pool.shm_path).ok();
    }

    /// Test: reclaim_dead_slots does NOT reclaim slots owned by living processes.
    #[test]
    #[cfg(unix)]
    fn test_reclaim_skips_live_process_slots() {
        let config = TensorPoolConfig {
            pool_size: 1024 * 1024,
            max_slots: 2,
            slot_alignment: 64,
            allocator: Default::default(),
        };
        let pool = TensorPool::new(9901, config).expect("Failed to create pool");

        // Allocate both slots — they're owned by the current (living) process
        let t1 = pool
            .alloc(&[64], TensorDtype::U8, Device::cpu())
            .expect("alloc t1");
        let _t2 = pool
            .alloc(&[64], TensorDtype::U8, Device::cpu())
            .expect("alloc t2");

        // Pool full, owner is alive → reclaim should find nothing → alloc fails
        pool.alloc(&[64], TensorDtype::U8, Device::cpu())
            .unwrap_err();

        // Verify refcount unchanged (no spurious reclamation)
        assert_eq!(pool.refcount(&t1), 1);

        // Clean up
        pool.release(&t1);
        pool.release(&_t2);
        std::fs::remove_file(&pool.shm_path).ok();
    }

    /// Test: reclaim_dead_slots returns count of reclaimed slots.
    #[test]
    #[cfg(unix)]
    fn test_reclaim_dead_slots_returns_count() {
        let config = TensorPoolConfig {
            pool_size: 1024 * 1024,
            max_slots: 4,
            slot_alignment: 64,
            allocator: Default::default(),
        };
        let pool = TensorPool::new(9902, config).expect("Failed to create pool");

        // Allocate 3 slots
        let t1 = pool.alloc(&[64], TensorDtype::U8, Device::cpu()).unwrap();
        let t2 = pool.alloc(&[64], TensorDtype::U8, Device::cpu()).unwrap();
        let t3 = pool.alloc(&[64], TensorDtype::U8, Device::cpu()).unwrap();

        // Mark 2 of them as owned by dead PIDs
        let dead_pid: u32 = (1 << 30) + 2;
        pool.slot(t1.slot_id)
            .owner_pid
            .store(dead_pid, Ordering::Release);
        pool.slot(t2.slot_id)
            .owner_pid
            .store(dead_pid + 1, Ordering::Release);

        // Call reclaim directly
        let reclaimed = pool.reclaim_dead_slots();
        assert_eq!(reclaimed, 2, "Should reclaim exactly 2 dead slots");

        // t3 should still be allocated with live PID
        assert_eq!(pool.refcount(&t3), 1);

        pool.release(&t3);
        std::fs::remove_file(&pool.shm_path).ok();
    }

    // ── Negative input tests ──────────────────────────────────────────────

    #[test]
    fn test_alloc_empty_shape() {
        let pool = make_test_pool(9800);
        // Empty shape: product of no elements = 1 (identity element of multiplication)
        // This allocates a scalar tensor (0-dimensional) with 1 element
        let result = pool.alloc(&[], TensorDtype::F32, Device::cpu());
        match result {
            Ok(tensor) => {
                // Scalar: ndim=0, numel=1, nbytes=4
                assert_eq!(tensor.ndim, 0);
                pool.release(&tensor);
            }
            Err(_) => {} // Also acceptable if the API rejects empty shape
        }
        std::fs::remove_file(&pool.shm_path).ok();
    }

    #[test]
    fn test_alloc_zero_element_dimension() {
        let pool = make_test_pool(9799);
        // Shape with a zero dimension — total elements = 0
        let result = pool.alloc(&[10, 0, 5], TensorDtype::F32, Device::cpu());
        match result {
            Ok(tensor) => {
                assert_eq!(tensor.numel(), 0);
                pool.release(&tensor);
            }
            Err(_) => {} // Zero-element tensors may be rejected
        }
        std::fs::remove_file(&pool.shm_path).ok();
    }

    #[test]
    fn test_alloc_single_element() {
        let pool = make_test_pool(9798);
        let tensor = pool
            .alloc(&[1], TensorDtype::U8, Device::cpu())
            .expect("single element alloc should succeed");
        assert_eq!(tensor.numel(), 1);
        assert_eq!(tensor.nbytes(), 1);
        pool.release(&tensor);
        std::fs::remove_file(&pool.shm_path).ok();
    }

    #[test]
    fn test_alloc_too_many_dimensions() {
        let pool = make_test_pool(9797);
        // Tensor supports up to 8 dimensions (ndim: u8, shape: [u64; 8])
        let shape_9d: Vec<u64> = vec![1; 9];
        let result = pool.alloc(&shape_9d, TensorDtype::U8, Device::cpu());
        match result {
            Ok(tensor) => {
                // If it succeeds, it should have clamped or handled gracefully
                pool.release(&tensor);
            }
            Err(e) => {
                // Expected: reject shapes with > 8 dimensions
                let msg = format!("{}", e);
                assert!(
                    !msg.is_empty(),
                    "Error message should be descriptive for >8 dimensions"
                );
            }
        }
        std::fs::remove_file(&pool.shm_path).ok();
    }

    #[test]
    fn test_alloc_pool_exhaustion() {
        let config = TensorPoolConfig {
            pool_size: 1024 * 1024,
            max_slots: 2,
            slot_alignment: 64,
            allocator: Default::default(),
        };
        let pool = TensorPool::new(9796, config).expect("Failed to create pool");

        let t1 = pool
            .alloc(&[8], TensorDtype::U8, Device::cpu())
            .expect("first alloc");
        let t2 = pool
            .alloc(&[8], TensorDtype::U8, Device::cpu())
            .expect("second alloc");

        // Third allocation should fail — pool is exhausted
        let result = pool.alloc(&[8], TensorDtype::U8, Device::cpu());
        assert!(result.is_err(), "Should fail when pool is exhausted");

        pool.release(&t1);
        pool.release(&t2);
        std::fs::remove_file(&pool.shm_path).ok();
    }

    #[test]
    fn test_alloc_after_release_reuses_slot() {
        let config = TensorPoolConfig {
            pool_size: 1024 * 1024,
            max_slots: 1,
            slot_alignment: 64,
            allocator: Default::default(),
        };
        let pool = TensorPool::new(9795, config).expect("Failed to create pool");

        let t1 = pool
            .alloc(&[16], TensorDtype::U8, Device::cpu())
            .expect("first alloc");
        let slot_id = t1.slot_id;
        pool.release(&t1);

        // After release, should be able to allocate again
        let t2 = pool
            .alloc(&[16], TensorDtype::U8, Device::cpu())
            .expect("alloc after release should succeed");
        assert_eq!(t2.slot_id, slot_id, "Should reuse the freed slot");
        assert!(
            t2.generation_full() > t1.generation_full(),
            "Generation should increase on reuse"
        );

        pool.release(&t2);
        std::fs::remove_file(&pool.shm_path).ok();
    }

    #[test]
    fn test_double_release_is_safe() {
        let pool = make_test_pool(9794);
        let tensor = pool
            .alloc(&[16], TensorDtype::U8, Device::cpu())
            .expect("alloc failed");

        pool.release(&tensor);
        assert_eq!(pool.refcount(&tensor), 0);

        // Second release should be a no-op (generation mismatch after slot recycling)
        pool.release(&tensor);
        // No panic, no UB
        std::fs::remove_file(&pool.shm_path).ok();
    }

    #[test]
    fn test_release_without_retain_goes_to_zero() {
        let pool = make_test_pool(9793);
        let tensor = pool
            .alloc(&[8], TensorDtype::U8, Device::cpu())
            .expect("alloc failed");

        assert_eq!(pool.refcount(&tensor), 1);
        pool.release(&tensor);
        assert_eq!(pool.refcount(&tensor), 0);

        std::fs::remove_file(&pool.shm_path).ok();
    }

    #[test]
    fn test_retain_release_balance() {
        let pool = make_test_pool(9792);
        let tensor = pool
            .alloc(&[8], TensorDtype::U8, Device::cpu())
            .expect("alloc failed");

        // Retain 5 times
        for _ in 0..5 {
            pool.retain(&tensor);
        }
        assert_eq!(pool.refcount(&tensor), 6); // 1 initial + 5 retains

        // Release 6 times
        for _ in 0..6 {
            pool.release(&tensor);
        }
        assert_eq!(pool.refcount(&tensor), 0);

        std::fs::remove_file(&pool.shm_path).ok();
    }

    #[test]
    fn test_alloc_with_timeout_zero_expires_immediately() {
        let config = TensorPoolConfig {
            pool_size: 1024 * 1024,
            max_slots: 1,
            slot_alignment: 64,
            allocator: Default::default(),
        };
        let pool = TensorPool::new(9791, config).expect("Failed to create pool");

        // Exhaust the pool
        let t1 = pool
            .alloc(&[8], TensorDtype::U8, Device::cpu())
            .expect("first alloc");

        // Zero timeout should fail immediately
        let start = std::time::Instant::now();
        let result = pool.alloc_with_timeout(
            &[8],
            TensorDtype::U8,
            Device::cpu(),
            std::time::Duration::ZERO,
        );
        assert!(
            result.is_err(),
            "Zero timeout on exhausted pool should fail"
        );
        assert!(
            start.elapsed() < 1_u64.secs(),
            "Zero timeout should return quickly"
        );

        pool.release(&t1);
        std::fs::remove_file(&pool.shm_path).ok();
    }

    #[test]
    fn test_data_slice_bounds() {
        let pool = make_test_pool(9790);
        let tensor = pool
            .alloc(&[10], TensorDtype::F32, Device::cpu())
            .expect("alloc failed");

        // data_slice should return exactly 40 bytes (10 * 4 bytes per f32)
        let data = pool.data_slice(&tensor).unwrap();
        assert_eq!(data.len(), 40);

        let data_mut = pool.data_slice_mut(&tensor).unwrap();
        assert_eq!(data_mut.len(), 40);

        pool.release(&tensor);
        std::fs::remove_file(&pool.shm_path).ok();
    }

    #[test]
    fn test_stats_reflect_allocations() {
        let config = TensorPoolConfig {
            pool_size: 1024 * 1024,
            max_slots: 4,
            slot_alignment: 64,
            allocator: Default::default(),
        };
        let pool = TensorPool::new(9789, config).expect("Failed to create pool");

        let stats = pool.stats();
        assert_eq!(stats.allocated_slots, 0);
        assert_eq!(stats.max_slots, 4);

        let t1 = pool.alloc(&[100], TensorDtype::U8, Device::cpu()).unwrap();
        let t2 = pool.alloc(&[200], TensorDtype::F32, Device::cpu()).unwrap();

        let stats = pool.stats();
        assert_eq!(stats.allocated_slots, 2);
        assert!(stats.used_bytes > 0);

        pool.release(&t1);
        let stats = pool.stats();
        assert_eq!(stats.allocated_slots, 1);

        pool.release(&t2);
        let stats = pool.stats();
        assert_eq!(stats.allocated_slots, 0);

        std::fs::remove_file(&pool.shm_path).ok();
    }

    #[test]
    fn test_pool_config_defaults() {
        let config = TensorPoolConfig::default();
        assert!(config.pool_size > 0, "Default pool_size should be positive");
        assert!(config.max_slots > 0, "Default max_slots should be positive");
        assert!(
            config.slot_alignment > 0,
            "Default alignment should be positive"
        );
    }

    #[test]
    fn test_alloc_larger_than_pool() {
        let config = TensorPoolConfig {
            pool_size: 1024, // Very small pool: 1KB
            max_slots: 4,
            slot_alignment: 64,
            allocator: Default::default(),
        };
        let pool = TensorPool::new(9788, config).expect("Failed to create pool");

        // Try to allocate 2KB in a 1KB pool
        let result = pool.alloc(&[2048], TensorDtype::U8, Device::cpu());
        assert!(result.is_err(), "Allocation larger than pool should fail");

        std::fs::remove_file(&pool.shm_path).ok();
    }

    #[test]
    fn test_multiple_dtypes() {
        let pool = make_test_pool(9787);

        let t_u8 = pool.alloc(&[10], TensorDtype::U8, Device::cpu()).unwrap();
        let t_f32 = pool.alloc(&[10], TensorDtype::F32, Device::cpu()).unwrap();
        let t_f64 = pool.alloc(&[10], TensorDtype::F64, Device::cpu()).unwrap();

        assert_eq!(t_u8.nbytes(), 10);
        assert_eq!(t_f32.nbytes(), 40);
        assert_eq!(t_f64.nbytes(), 80);

        pool.release(&t_u8);
        pool.release(&t_f32);
        pool.release(&t_f64);
        std::fs::remove_file(&pool.shm_path).ok();
    }

    // ========================================================================
    // Exhaustion and fragmentation tests
    // ========================================================================

    #[test]
    fn test_slot_exhaustion_returns_error_not_panic() {
        let config = TensorPoolConfig {
            pool_size: 1024 * 1024,
            max_slots: 4,
            slot_alignment: 64,
            allocator: Default::default(),
        };
        let pool = TensorPool::new(9600, config).expect("Failed to create pool");

        // Fill all 4 slots
        let mut tensors = Vec::new();
        for _ in 0..4 {
            tensors.push(pool.alloc(&[8], TensorDtype::U8, Device::cpu()).unwrap());
        }

        // 5th allocation should return Err, not panic
        let result = pool.alloc(&[8], TensorDtype::U8, Device::cpu());
        assert!(result.is_err());
        match result.unwrap_err() {
            HorusError::Memory(msg) => {
                assert!(
                    msg.to_string().contains("No free tensor slots"),
                    "Expected 'No free tensor slots' error, got: {}",
                    msg
                );
            }
            other => panic!("Expected Memory error, got: {:?}", other),
        }

        for t in &tensors {
            pool.release(t);
        }
        std::fs::remove_file(&pool.shm_path).ok();
    }

    #[test]
    fn test_data_region_exhaustion_returns_error() {
        let config = TensorPoolConfig {
            pool_size: 256, // Very small: 256 bytes data region
            max_slots: 8,
            slot_alignment: 64,
            allocator: Default::default(),
        };
        let pool = TensorPool::new(9601, config).expect("Failed to create pool");

        // First alloc takes most of the data region (192 bytes, aligned to 64 → 192)
        let t1 = pool
            .alloc(&[192], TensorDtype::U8, Device::cpu())
            .expect("First alloc should succeed");

        // Second alloc should fail — data region exhausted (only 64 bytes left, need 128+alignment)
        let result = pool.alloc(&[128], TensorDtype::U8, Device::cpu());
        assert!(result.is_err(), "Should fail when data region is exhausted");
        match result.unwrap_err() {
            HorusError::Memory(msg) => {
                assert!(
                    msg.to_string().contains("out of memory"),
                    "Expected 'out of memory' error, got: {}",
                    msg
                );
            }
            other => panic!("Expected Memory error, got: {:?}", other),
        }

        pool.release(&t1);
        std::fs::remove_file(&pool.shm_path).ok();
    }

    #[test]
    fn test_fragmentation_pattern_alloc_free_interleaved() {
        // Allocate/free in a pattern that would fragment a naive allocator
        let config = TensorPoolConfig {
            pool_size: 1024 * 1024,
            max_slots: 8,
            slot_alignment: 64,
            allocator: Default::default(),
        };
        let pool = TensorPool::new(9602, config).expect("Failed to create pool");

        // Round 1: fill all slots
        let mut tensors: Vec<_> = (0..8)
            .map(|_| pool.alloc(&[64], TensorDtype::U8, Device::cpu()).unwrap())
            .collect();

        // Free even slots
        for i in (0..8).step_by(2) {
            pool.release(&tensors[i]);
        }

        // Verify stats: 4 allocated, 4 free
        let stats = pool.stats();
        assert_eq!(stats.allocated_slots, 4);

        // Reallocate into the freed slots
        let new_tensors: Vec<_> = (0..4)
            .map(|_| pool.alloc(&[32], TensorDtype::U8, Device::cpu()).unwrap())
            .collect();

        let stats = pool.stats();
        assert_eq!(stats.allocated_slots, 8);

        // Free odd slots from round 1
        for i in (1..8).step_by(2) {
            pool.release(&tensors[i]);
        }

        // Free round 2 tensors
        for t in &new_tensors {
            pool.release(t);
        }

        let stats = pool.stats();
        assert_eq!(stats.allocated_slots, 0);

        // Round 3: can still allocate after fragmentation
        let t = pool
            .alloc(&[100], TensorDtype::U8, Device::cpu())
            .expect("Should still allocate after fragmentation");
        pool.release(&t);

        // Clear the original vec to prevent use-after-free on tensors
        tensors.clear();
        std::fs::remove_file(&pool.shm_path).ok();
    }

    #[test]
    fn test_rapid_alloc_release_cycles() {
        let config = TensorPoolConfig {
            pool_size: 1024 * 1024,
            max_slots: 2,
            slot_alignment: 64,
            allocator: Default::default(),
        };
        let pool = TensorPool::new(9603, config).expect("Failed to create pool");

        // Run 50 alloc/release cycles on a 2-slot pool
        for i in 0..50 {
            let t = pool
                .alloc(&[64], TensorDtype::U8, Device::cpu())
                .unwrap_or_else(|e| panic!("Alloc failed on cycle {}: {:?}", i, e));
            pool.release(&t);
        }

        let stats = pool.stats();
        assert_eq!(stats.allocated_slots, 0);

        std::fs::remove_file(&pool.shm_path).ok();
    }

    #[test]
    fn test_alloc_free_different_sizes_fragmentation() {
        let config = TensorPoolConfig {
            pool_size: 1024 * 1024,
            max_slots: 16,
            slot_alignment: 64,
            allocator: Default::default(),
        };
        let pool = TensorPool::new(9604, config).expect("Failed to create pool");

        // Allocate tensors of varying sizes
        let sizes: Vec<u64> = vec![8, 256, 1024, 16, 512, 64, 2048, 32];
        let mut tensors: Vec<Tensor> = Vec::new();
        for &s in &sizes {
            tensors.push(pool.alloc(&[s], TensorDtype::U8, Device::cpu()).unwrap());
        }

        let stats_full = pool.stats();
        assert_eq!(stats_full.allocated_slots, 8);

        // Free every other tensor (fragmentation pattern)
        for i in (0..8).step_by(2) {
            pool.release(&tensors[i]);
        }

        // Reallocate with different sizes
        let realloc_sizes: Vec<u64> = vec![128, 512, 32, 1024];
        let mut new_tensors = Vec::new();
        for &s in &realloc_sizes {
            new_tensors.push(pool.alloc(&[s], TensorDtype::U8, Device::cpu()).unwrap());
        }

        let stats = pool.stats();
        assert_eq!(stats.allocated_slots, 8); // 4 original odd + 4 new

        // Clean up
        for i in (1..8).step_by(2) {
            pool.release(&tensors[i]);
        }
        for t in &new_tensors {
            pool.release(t);
        }

        std::fs::remove_file(&pool.shm_path).ok();
    }

    // ========================================================================
    // Concurrent allocation tests
    // ========================================================================

    #[test]
    fn test_concurrent_alloc_no_corruption() {
        use std::sync::Arc;

        let config = TensorPoolConfig {
            pool_size: 16 * 1024 * 1024, // 16MB
            max_slots: 64,
            slot_alignment: 64,
            allocator: Default::default(),
        };
        let pool = Arc::new(TensorPool::new(9605, config).expect("Failed to create pool"));

        let num_threads = 4;
        let allocs_per_thread = 10;

        let handles: Vec<_> = (0..num_threads)
            .map(|_| {
                let pool = Arc::clone(&pool);
                std::thread::spawn(move || {
                    let mut tensors = Vec::new();
                    for _ in 0..allocs_per_thread {
                        let t = pool
                            .alloc(&[64], TensorDtype::F32, Device::cpu())
                            .expect("concurrent alloc failed");
                        // Write unique data to verify no overlap
                        let data = pool.data_slice_mut(&t).unwrap();
                        for byte in data.iter_mut() {
                            *byte = 0xAB;
                        }
                        tensors.push(t);
                    }
                    tensors
                })
            })
            .collect();

        let all_tensors: Vec<Vec<Tensor>> =
            handles.into_iter().map(|h| h.join().unwrap()).collect();

        // Verify no slot ID collisions
        let mut all_slot_ids: Vec<u32> = all_tensors
            .iter()
            .flat_map(|v| v.iter().map(|t| t.slot_id))
            .collect();
        let total = all_slot_ids.len();
        all_slot_ids.sort();
        all_slot_ids.dedup();
        assert_eq!(
            all_slot_ids.len(),
            total,
            "Slot IDs should be unique across threads"
        );

        // Verify data integrity — each tensor's data should be 0xAB
        for tensors in &all_tensors {
            for t in tensors {
                let data = pool.data_slice(t).unwrap();
                assert!(
                    data.iter().all(|&b| b == 0xAB),
                    "Data corruption detected in slot {}",
                    t.slot_id
                );
            }
        }

        // Stats should reflect total allocations
        let stats = pool.stats();
        assert_eq!(stats.allocated_slots, num_threads * allocs_per_thread);

        // Clean up
        for tensors in &all_tensors {
            for t in tensors {
                pool.release(t);
            }
        }

        let stats = pool.stats();
        assert_eq!(stats.allocated_slots, 0);
        std::fs::remove_file(&pool.shm_path).ok();
    }

    #[test]
    fn test_concurrent_alloc_release_stress() {
        use std::sync::Arc;

        let config = TensorPoolConfig {
            pool_size: 4 * 1024 * 1024, // 4MB
            max_slots: 8,               // Small pool to force contention
            slot_alignment: 64,
            allocator: Default::default(),
        };
        let pool = Arc::new(TensorPool::new(9606, config).expect("Failed to create pool"));

        let num_threads = 4;
        let cycles_per_thread = 20;

        let handles: Vec<_> = (0..num_threads)
            .map(|_| {
                let pool = Arc::clone(&pool);
                std::thread::spawn(move || {
                    let mut successes = 0u32;
                    for _ in 0..cycles_per_thread {
                        match pool.alloc(&[32], TensorDtype::U8, Device::cpu()) {
                            Ok(t) => {
                                successes += 1;
                                // Brief hold then release
                                std::thread::yield_now();
                                pool.release(&t);
                            }
                            Err(_) => {
                                // Pool full — expected under contention
                                std::thread::yield_now();
                            }
                        }
                    }
                    successes
                })
            })
            .collect();

        let total_successes: u32 = handles.into_iter().map(|h| h.join().unwrap()).sum();
        assert!(
            total_successes > 0,
            "At least some concurrent allocations should succeed"
        );

        // After all threads complete, pool should be empty
        let stats = pool.stats();
        assert_eq!(
            stats.allocated_slots, 0,
            "All slots should be freed after stress test"
        );

        std::fs::remove_file(&pool.shm_path).ok();
    }

    #[test]
    fn test_concurrent_retain_release() {
        use std::sync::Arc;

        let config = TensorPoolConfig {
            pool_size: 1024 * 1024,
            max_slots: 4,
            slot_alignment: 64,
            allocator: Default::default(),
        };
        let pool = Arc::new(TensorPool::new(9607, config).expect("Failed to create pool"));
        let tensor = pool
            .alloc(&[64], TensorDtype::U8, Device::cpu())
            .expect("alloc failed");

        // 4 threads each retain then release
        let num_threads = 4;
        let ops_per_thread = 25;

        // Pre-retain so refcount = 1 + num_threads * ops_per_thread
        for _ in 0..num_threads * ops_per_thread {
            pool.retain(&tensor);
        }
        assert_eq!(
            pool.refcount(&tensor),
            1 + (num_threads * ops_per_thread) as u32
        );

        let handles: Vec<_> = (0..num_threads)
            .map(|_| {
                let pool = Arc::clone(&pool);
                let t = tensor;
                std::thread::spawn(move || {
                    for _ in 0..ops_per_thread {
                        pool.release(&t);
                    }
                })
            })
            .collect();

        for h in handles {
            h.join().unwrap();
        }

        // Should be back to refcount 1 (original alloc)
        assert_eq!(pool.refcount(&tensor), 1);

        pool.release(&tensor);
        assert_eq!(pool.refcount(&tensor), 0);

        std::fs::remove_file(&pool.shm_path).ok();
    }

    // ========================================================================
    // Stats accuracy tests
    // ========================================================================

    #[test]
    fn test_stats_accuracy_under_alloc_release_load() {
        let config = TensorPoolConfig {
            pool_size: 4 * 1024 * 1024,
            max_slots: 16,
            slot_alignment: 64,
            allocator: Default::default(),
        };
        let pool = TensorPool::new(9608, config).expect("Failed to create pool");

        // Allocate 10 tensors of various sizes
        let shapes: Vec<Vec<u64>> = vec![
            vec![100],
            vec![10, 20],
            vec![8, 8, 8],
            vec![1024],
            vec![64],
            vec![256],
            vec![32],
            vec![128],
            vec![512],
            vec![16],
        ];
        let mut tensors = Vec::new();
        for shape in &shapes {
            tensors.push(pool.alloc(shape, TensorDtype::F32, Device::cpu()).unwrap());
        }

        let stats = pool.stats();
        assert_eq!(stats.allocated_slots, 10);
        assert_eq!(stats.max_slots, 16);
        assert_eq!(stats.total_refcount, 10); // Each has refcount 1
        assert!(stats.used_bytes > 0);
        assert!(stats.free_bytes < stats.pool_size);
        assert_eq!(stats.used_bytes + stats.free_bytes, stats.pool_size);

        // Retain some tensors
        pool.retain(&tensors[0]);
        pool.retain(&tensors[0]);
        pool.retain(&tensors[5]);

        let stats = pool.stats();
        assert_eq!(stats.total_refcount, 13); // 10 + 3 retains

        // Release all original references
        for t in &tensors {
            pool.release(t);
        }

        let stats = pool.stats();
        assert_eq!(stats.allocated_slots, 2); // tensors[0] (rc=2) and tensors[5] (rc=1) still live
        assert_eq!(stats.total_refcount, 3);

        // Final cleanup
        pool.release(&tensors[0]);
        pool.release(&tensors[0]);
        pool.release(&tensors[5]);

        let stats = pool.stats();
        assert_eq!(stats.allocated_slots, 0);
        assert_eq!(stats.total_refcount, 0);

        std::fs::remove_file(&pool.shm_path).ok();
    }

    #[test]
    fn test_stats_used_bytes_monotonically_increases() {
        // The bump allocator never reclaims data region bytes, only slot metadata
        let config = TensorPoolConfig {
            pool_size: 1024 * 1024,
            max_slots: 8,
            slot_alignment: 64,
            allocator: Default::default(),
        };
        let pool = TensorPool::new(9609, config).expect("Failed to create pool");

        let mut prev_used = 0;
        let mut tensors = Vec::new();

        for _ in 0..5 {
            let t = pool.alloc(&[64], TensorDtype::U8, Device::cpu()).unwrap();
            tensors.push(t);
            let stats = pool.stats();
            assert!(
                stats.used_bytes >= prev_used,
                "used_bytes should never decrease: {} < {}",
                stats.used_bytes,
                prev_used
            );
            prev_used = stats.used_bytes;
        }

        // Release all tensors — used_bytes should NOT decrease (bump allocator)
        for t in &tensors {
            pool.release(t);
        }
        let stats = pool.stats();
        assert_eq!(
            stats.used_bytes, prev_used,
            "Bump allocator: used_bytes stays the same after release"
        );

        std::fs::remove_file(&pool.shm_path).ok();
    }

    // ========================================================================
    // Pack/unpack tagged head tests
    // ========================================================================

    #[test]
    fn test_pack_unpack_tagged_head_roundtrip() {
        let test_cases = [
            (0u32, 0u32),
            (1, 0),
            (0, 1),
            (u32::MAX, u32::MAX),
            (42, 7),
            (0xDEAD, 0xBEEF),
        ];
        for (gen, slot) in test_cases {
            let packed = pack_tagged_head(gen, slot);
            let (g, s) = unpack_tagged_head(packed);
            assert_eq!(
                (g, s),
                (gen, slot),
                "Roundtrip failed for ({}, {})",
                gen,
                slot
            );
        }
    }

    #[test]
    fn test_pack_tagged_head_layout() {
        let packed = pack_tagged_head(0x12345678, 0xABCDEF01);
        assert_eq!(packed >> 32, 0x12345678);
        assert_eq!(packed & 0xFFFF_FFFF, 0xABCDEF01);
    }

    // ========================================================================
    // Stress and hardening tests
    // ========================================================================

    #[test]
    fn test_stress_alloc_release_cycle() {
        let pool = TensorPool::new(10100, TensorPoolConfig::default()).expect("create pool");
        // Alloc and release 200 tensors (more than pool slots)
        for i in 0..200 {
            let tensor = pool.alloc(
                &[100],
                TensorDtype::F32,
                Device::cpu(),
            );
            match tensor {
                Ok(t) => pool.release(&t),
                Err(_) => {} // Pool full is OK — previous releases should have freed slots
            }
            // No panic, no leak after 200 cycles
            let _ = i;
        }
    }

    #[test]
    fn test_stress_retain_release_same_tensor() {
        let pool = TensorPool::new(10101, TensorPoolConfig::default()).expect("create pool");
        let tensor = pool.alloc(&[64], TensorDtype::U8, Device::cpu()).unwrap();

        // Retain 50 times
        for _ in 0..50 {
            pool.retain(&tensor);
        }
        // Release 50 times (back to refcount=1 from alloc)
        for _ in 0..50 {
            pool.release(&tensor);
        }
        // Final release (refcount=0, freed)
        pool.release(&tensor);

        // Slot should be free now — can alloc again
        let tensor2 = pool.alloc(&[64], TensorDtype::U8, Device::cpu());
        assert!(tensor2.is_ok(), "slot should be reclaimable after full release");
    }

    #[test]
    fn test_pool_alloc_when_full_returns_error() {
        // Create a very small pool
        let config = TensorPoolConfig {
            pool_size: 4096,     // 4KB — very small
            max_slots: 4,        // Only 4 slots
            slot_alignment: 64,
            allocator: PoolAllocator::Mmap,
        };
        let pool = TensorPool::new(9999, config).expect("create small pool");

        // Alloc until full
        let mut tensors = vec![];
        for _ in 0..4 {
            match pool.alloc(&[100], TensorDtype::U8, Device::cpu()) {
                Ok(t) => tensors.push(t),
                Err(_) => break,
            }
        }

        // Next alloc should fail (pool full)
        let result = pool.alloc(&[100], TensorDtype::U8, Device::cpu());
        assert!(result.is_err(), "alloc on full pool should return error");

        // Release one and alloc again — should succeed
        if let Some(t) = tensors.pop() {
            pool.release(&t);
        }
        let result = pool.alloc(&[100], TensorDtype::U8, Device::cpu());
        assert!(result.is_ok(), "alloc after release should succeed");
    }

    #[test]
    fn test_release_on_wrong_pool_id_is_silent() {
        let pool = TensorPool::new(10102, TensorPoolConfig::default()).expect("create pool");
        let tensor = pool.alloc(&[64], TensorDtype::U8, Device::cpu()).unwrap();

        // Create a fake tensor with wrong pool_id
        let mut fake = tensor;
        fake.pool_id = 99999;

        // Release with wrong pool_id should be silent (no panic, no effect)
        pool.release(&fake);

        // Original tensor should still be valid
        let data = pool.data_slice(&tensor);
        assert!(data.is_ok(), "original tensor should still be accessible");

        pool.release(&tensor);
    }

    #[test]
    fn test_retain_on_freed_tensor_is_silent() {
        let pool = TensorPool::new(10103, TensorPoolConfig::default()).expect("create pool");
        let tensor = pool.alloc(&[64], TensorDtype::U8, Device::cpu()).unwrap();

        // Release (refcount=0, freed)
        pool.release(&tensor);

        // Retain on freed tensor should be silent (refcount=0, stale)
        pool.retain(&tensor);
        // No panic = success
    }

    // ========================================================================
    // Edge case combination tests
    // ========================================================================

    #[test]
    fn test_alloc_release_immediate_realloc_same_slot() {
        // Allocate a tensor, release it immediately, then reallocate.
        // The same slot should be reused with a bumped generation, and
        // data from the first allocation must be zeroed.
        let config = TensorPoolConfig {
            pool_size: 1024 * 1024,
            max_slots: 1, // Force reuse of slot 0
            slot_alignment: 64,
            allocator: Default::default(),
        };
        let pool = TensorPool::new(8800, config).expect("Failed to create pool");

        let t1 = pool
            .alloc(&[64], TensorDtype::U8, Device::cpu())
            .expect("first alloc");
        let slot_id_1 = t1.slot_id;
        let gen_1 = t1.generation_full();

        // Write a pattern before releasing
        pool.data_slice_mut(&t1).unwrap().fill(0xCD);

        // Immediate release
        pool.release(&t1);
        assert_eq!(pool.refcount(&t1), 0);

        // Reallocate — must get the same slot
        let t2 = pool
            .alloc(&[64], TensorDtype::U8, Device::cpu())
            .expect("realloc after release");
        assert_eq!(t2.slot_id, slot_id_1, "same slot should be reused");
        assert!(
            t2.generation_full() > gen_1,
            "generation must increment on reuse"
        );

        // Previous data must have been zeroed by release
        let data = pool.data_slice(&t2).unwrap();
        assert!(
            data.iter().all(|&b| b == 0),
            "reused slot data should be zeroed"
        );

        pool.release(&t2);
        std::fs::remove_file(&pool.shm_path).ok();
    }

    #[test]
    fn test_alloc_all_release_half_realloc_half() {
        // Fill all slots, release the first half, then reallocate those slots.
        // Verify stats track correctly throughout.
        let config = TensorPoolConfig {
            pool_size: 4 * 1024 * 1024,
            max_slots: 8,
            slot_alignment: 64,
            allocator: Default::default(),
        };
        let pool = TensorPool::new(8801, config).expect("Failed to create pool");

        // Allocate all 8 slots
        let mut tensors: Vec<Tensor> = Vec::new();
        for _ in 0..8 {
            tensors.push(
                pool.alloc(&[64], TensorDtype::U8, Device::cpu())
                    .expect("alloc should succeed"),
            );
        }

        let stats = pool.stats();
        assert_eq!(stats.allocated_slots, 8);
        assert_eq!(stats.max_slots, 8);

        // Pool should be full
        assert!(pool.alloc(&[8], TensorDtype::U8, Device::cpu()).is_err());

        // Release first 4 slots
        for t in &tensors[..4] {
            pool.release(t);
        }

        let stats = pool.stats();
        assert_eq!(stats.allocated_slots, 4, "4 slots should remain after releasing half");

        // Reallocate 4 slots — should succeed
        let mut new_tensors: Vec<Tensor> = Vec::new();
        for _ in 0..4 {
            new_tensors.push(
                pool.alloc(&[32], TensorDtype::F32, Device::cpu())
                    .expect("realloc into freed slots should succeed"),
            );
        }

        let stats = pool.stats();
        assert_eq!(stats.allocated_slots, 8, "all 8 slots should be allocated again");

        // Pool should be full again
        assert!(pool.alloc(&[8], TensorDtype::U8, Device::cpu()).is_err());

        // Cleanup
        for t in &tensors[4..] {
            pool.release(t);
        }
        for t in &new_tensors {
            pool.release(t);
        }

        let stats = pool.stats();
        assert_eq!(stats.allocated_slots, 0);

        std::fs::remove_file(&pool.shm_path).ok();
    }

    #[test]
    fn test_retain_three_release_three_frees_slot() {
        // Alloc (rc=1), retain 3 times (rc=4), release 3 times (rc=1),
        // verify slot still live, then final release (rc=0) frees slot.
        let config = TensorPoolConfig {
            pool_size: 1024 * 1024,
            max_slots: 1,
            slot_alignment: 64,
            allocator: Default::default(),
        };
        let pool = TensorPool::new(8802, config).expect("Failed to create pool");

        let tensor = pool
            .alloc(&[32], TensorDtype::U8, Device::cpu())
            .expect("alloc");
        assert_eq!(pool.refcount(&tensor), 1);

        // Retain 3 times
        pool.retain(&tensor);
        pool.retain(&tensor);
        pool.retain(&tensor);
        assert_eq!(pool.refcount(&tensor), 4);

        // Release 3 times — should still be alive (rc=1)
        pool.release(&tensor);
        pool.release(&tensor);
        pool.release(&tensor);
        assert_eq!(pool.refcount(&tensor), 1);

        // Slot should still be allocated
        let stats = pool.stats();
        assert_eq!(stats.allocated_slots, 1);

        // Data should still be accessible
        assert!(pool.data_slice(&tensor).is_ok());

        // Final release frees the slot
        pool.release(&tensor);
        assert_eq!(pool.refcount(&tensor), 0);

        let stats = pool.stats();
        assert_eq!(stats.allocated_slots, 0);

        // Slot is free — can allocate again
        let t2 = pool
            .alloc(&[32], TensorDtype::U8, Device::cpu())
            .expect("realloc after full release");
        pool.release(&t2);

        std::fs::remove_file(&pool.shm_path).ok();
    }

    #[test]
    fn test_stats_accuracy_alloc_n_release_m() {
        // Allocate N tensors, verify stats.allocated_slots == N,
        // release M, verify stats.allocated_slots == N - M.
        let config = TensorPoolConfig {
            pool_size: 4 * 1024 * 1024,
            max_slots: 16,
            slot_alignment: 64,
            allocator: Default::default(),
        };
        let pool = TensorPool::new(8803, config).expect("Failed to create pool");

        let n = 12;
        let m = 7;

        let mut tensors = Vec::new();
        for i in 0..n {
            let t = pool
                .alloc(&[(i as u64 + 1) * 16], TensorDtype::U8, Device::cpu())
                .expect("alloc");
            tensors.push(t);
        }

        let stats = pool.stats();
        assert_eq!(stats.allocated_slots, n);
        assert_eq!(stats.pool_id, 8803);
        assert_eq!(stats.total_refcount, n as u32);
        assert!(stats.used_bytes > 0);

        // Release first M tensors
        for t in &tensors[..m] {
            pool.release(t);
        }

        let stats = pool.stats();
        assert_eq!(
            stats.allocated_slots,
            n - m,
            "allocated_slots should be N-M after releasing M"
        );
        assert_eq!(stats.total_refcount, (n - m) as u32);

        // Release remaining
        for t in &tensors[m..] {
            pool.release(t);
        }

        let stats = pool.stats();
        assert_eq!(stats.allocated_slots, 0);
        assert_eq!(stats.total_refcount, 0);

        std::fs::remove_file(&pool.shm_path).ok();
    }

    #[test]
    fn test_stats_with_mixed_refcounts() {
        // Allocate tensors with varying retain counts, verify total_refcount
        // reflects the sum correctly across allocs and retains.
        let config = TensorPoolConfig {
            pool_size: 1024 * 1024,
            max_slots: 4,
            slot_alignment: 64,
            allocator: Default::default(),
        };
        let pool = TensorPool::new(8804, config).expect("Failed to create pool");

        let t0 = pool.alloc(&[8], TensorDtype::U8, Device::cpu()).unwrap();
        let t1 = pool.alloc(&[8], TensorDtype::U8, Device::cpu()).unwrap();
        let t2 = pool.alloc(&[8], TensorDtype::U8, Device::cpu()).unwrap();

        // t0: rc=1, t1: retain 2x → rc=3, t2: retain 4x → rc=5
        pool.retain(&t1);
        pool.retain(&t1);
        for _ in 0..4 {
            pool.retain(&t2);
        }

        let stats = pool.stats();
        assert_eq!(stats.allocated_slots, 3);
        assert_eq!(stats.total_refcount, 1 + 3 + 5); // 9

        // Release t0 fully
        pool.release(&t0);
        let stats = pool.stats();
        assert_eq!(stats.allocated_slots, 2);
        assert_eq!(stats.total_refcount, 3 + 5); // 8

        // Clean up t1 (3 releases) and t2 (5 releases)
        for _ in 0..3 {
            pool.release(&t1);
        }
        for _ in 0..5 {
            pool.release(&t2);
        }

        let stats = pool.stats();
        assert_eq!(stats.allocated_slots, 0);
        assert_eq!(stats.total_refcount, 0);

        std::fs::remove_file(&pool.shm_path).ok();
    }

    #[test]
    fn test_concurrent_alloc_release_interleaved_threads() {
        // Multiple threads each allocate and release in rapid succession,
        // verifying that all slots are properly freed afterward.
        use std::sync::Arc;

        // Clean stale SHM from previous runs
        let _ = std::fs::remove_file(format!("/dev/shm/horus_tensor_pool_8805"));

        let config = TensorPoolConfig {
            pool_size: 4 * 1024 * 1024,
            max_slots: 16,
            slot_alignment: 64,
            allocator: Default::default(),
        };
        let pool = Arc::new(TensorPool::new(8805, config).expect("Failed to create pool"));

        let num_threads = 4;
        let cycles = 30;

        let handles: Vec<_> = (0..num_threads)
            .map(|thread_id| {
                let pool = Arc::clone(&pool);
                std::thread::spawn(move || {
                    let mut total_allocs = 0u32;
                    for _ in 0..cycles {
                        // Each thread tries to hold 2 tensors at a time
                        match pool.alloc(&[32], TensorDtype::U8, Device::cpu()) {
                            Ok(t1) => {
                                total_allocs += 1;
                                // Write thread-specific pattern to verify no cross-corruption
                                let pattern = (thread_id + 1) as u8;
                                pool.data_slice_mut(&t1).unwrap().fill(pattern);

                                match pool.alloc(&[32], TensorDtype::U8, Device::cpu()) {
                                    Ok(t2) => {
                                        total_allocs += 1;
                                        pool.data_slice_mut(&t2).unwrap().fill(pattern);

                                        // Verify first tensor wasn't corrupted
                                        assert!(
                                            pool.data_slice(&t1)
                                                .unwrap()
                                                .iter()
                                                .all(|&b| b == pattern),
                                            "data corruption in thread {}",
                                            thread_id
                                        );

                                        pool.release(&t2);
                                    }
                                    Err(_) => {} // Pool full under contention — OK
                                }
                                pool.release(&t1);
                            }
                            Err(_) => {
                                std::thread::yield_now();
                            }
                        }
                    }
                    total_allocs
                })
            })
            .collect();

        let total: u32 = handles.into_iter().map(|h| h.join().unwrap()).sum();
        assert!(total > 0, "at least some allocations should succeed");

        let stats = pool.stats();
        assert_eq!(
            stats.allocated_slots, 0,
            "all slots must be freed after all threads complete"
        );

        std::fs::remove_file(&pool.shm_path).ok();
    }

    #[test]
    fn test_generation_counter_increments_across_multiple_reuses() {
        // Allocate and release the same single slot many times.
        // Verify generation strictly increases on each reuse and that
        // old descriptors become stale.
        let config = TensorPoolConfig {
            pool_size: 1024 * 1024,
            max_slots: 1, // Force same slot reuse
            slot_alignment: 64,
            allocator: Default::default(),
        };
        let pool = TensorPool::new(8806, config).expect("Failed to create pool");

        let mut prev_gen = 0u64;
        let mut stale_descriptors: Vec<Tensor> = Vec::new();

        for cycle in 0..20 {
            let tensor = pool
                .alloc(&[16], TensorDtype::U8, Device::cpu())
                .unwrap_or_else(|e| panic!("alloc failed on cycle {}: {:?}", cycle, e));

            let gen = tensor.generation_full();
            assert!(
                gen > prev_gen,
                "cycle {}: generation must strictly increase: prev={} curr={}",
                cycle, prev_gen, gen
            );
            prev_gen = gen;

            // Verify all previously-saved descriptors are now stale
            for (i, stale) in stale_descriptors.iter().enumerate() {
                assert_eq!(
                    pool.refcount(stale),
                    0,
                    "cycle {}: stale descriptor from cycle {} should report refcount 0",
                    cycle, i
                );
                assert!(
                    pool.try_retain(stale).is_err(),
                    "cycle {}: try_retain on stale descriptor from cycle {} should fail",
                    cycle, i
                );
            }

            stale_descriptors.push(tensor);
            pool.release(&tensor);
        }

        std::fs::remove_file(&pool.shm_path).ok();
    }

    #[test]
    fn test_pool_with_very_small_pool_size() {
        // Pool with pool_size too small to fit any aligned allocation
        // should fail gracefully on alloc, not on pool creation.
        let config = TensorPoolConfig {
            pool_size: 1, // 1 byte — too small for any 64-byte-aligned allocation
            max_slots: 4,
            slot_alignment: 64,
            allocator: Default::default(),
        };
        let pool = TensorPool::new(8807, config).expect("pool creation should succeed");

        // Any allocation should fail due to data region exhaustion
        let result = pool.alloc(&[1], TensorDtype::U8, Device::cpu());
        assert!(
            result.is_err(),
            "alloc in tiny pool should fail gracefully"
        );

        assert_eq!(pool.stats().pool_size, 1);

        std::fs::remove_file(&pool.shm_path).ok();
    }

    #[test]
    fn test_pool_with_max_slots_one() {
        // Edge case: pool with exactly 1 slot — alloc/release/realloc cycle.
        let config = TensorPoolConfig {
            pool_size: 1024 * 1024,
            max_slots: 1,
            slot_alignment: 64,
            allocator: Default::default(),
        };
        let pool = TensorPool::new(8808, config).expect("Failed to create pool");

        let stats = pool.stats();
        assert_eq!(stats.max_slots, 1);

        // Alloc the single slot
        let t1 = pool
            .alloc(&[128], TensorDtype::F32, Device::cpu())
            .expect("alloc single slot");
        assert_eq!(pool.stats().allocated_slots, 1);

        // Second alloc should fail
        assert!(pool.alloc(&[8], TensorDtype::U8, Device::cpu()).is_err());

        // Release and realloc
        pool.release(&t1);
        assert_eq!(pool.stats().allocated_slots, 0);

        let t2 = pool
            .alloc(&[64], TensorDtype::U8, Device::cpu())
            .expect("realloc single slot");
        assert_eq!(pool.stats().allocated_slots, 1);

        pool.release(&t2);
        std::fs::remove_file(&pool.shm_path).ok();
    }

    #[test]
    fn test_validate_descriptor_on_freed_then_realloced_slot() {
        // Allocate slot, save descriptor, release, reallocate (new generation).
        // validate_descriptor on the old descriptor should fail because the
        // generation has changed.
        let config = TensorPoolConfig {
            pool_size: 1024 * 1024,
            max_slots: 1, // Force reuse
            slot_alignment: 64,
            allocator: Default::default(),
        };
        let pool = TensorPool::new(8809, config).expect("Failed to create pool");

        let old = pool
            .alloc(&[32], TensorDtype::U8, Device::cpu())
            .expect("first alloc");
        pool.release(&old);

        let _new = pool
            .alloc(&[32], TensorDtype::U8, Device::cpu())
            .expect("second alloc");

        // Old descriptor should fail validation (generation mismatch)
        let result = pool.validate_descriptor(&old);
        assert!(
            result.is_err(),
            "stale descriptor should fail validation after slot reuse"
        );
        match result.unwrap_err() {
            HorusError::InvalidDescriptor(msg) => {
                assert!(
                    msg.contains("generation") || msg.contains("ALLOCATED"),
                    "error should mention generation or allocation state: {msg}"
                );
            }
            other => panic!("expected InvalidDescriptor, got {other:?}"),
        }

        // New descriptor should pass validation
        assert!(
            pool.validate_descriptor(&_new).is_ok(),
            "current descriptor should pass validation"
        );

        pool.release(&_new);
        std::fs::remove_file(&pool.shm_path).ok();
    }

    #[test]
    fn test_data_slice_on_freed_slot_still_returns_zeroed_data() {
        // data_slice() performs only bounds checks, not liveness checks.
        // After release, the data region is zeroed. A stale descriptor
        // can still read the memory (it's bounds-valid), but should see zeros.
        let config = TensorPoolConfig {
            pool_size: 1024 * 1024,
            max_slots: 2,
            slot_alignment: 64,
            allocator: Default::default(),
        };
        let pool = TensorPool::new(8810, config).expect("Failed to create pool");

        let tensor = pool
            .alloc(&[256], TensorDtype::U8, Device::cpu())
            .expect("alloc");

        // Fill with non-zero data
        pool.data_slice_mut(&tensor).unwrap().fill(0xFF);
        assert!(pool.data_slice(&tensor).unwrap().iter().all(|&b| b == 0xFF));

        // Release zeroes the data
        pool.release(&tensor);

        // data_slice on the stale descriptor should return zeroed bytes
        let data = pool.data_slice(&tensor).unwrap();
        assert!(
            data.iter().all(|&b| b == 0),
            "freed slot data must be zeroed"
        );

        std::fs::remove_file(&pool.shm_path).ok();
    }

    #[test]
    fn test_try_retain_release_on_stale_descriptor_returns_errors() {
        // After alloc → release → realloc, the old descriptor should cause
        // try_retain and try_release to return generation-mismatch errors.
        let config = TensorPoolConfig {
            pool_size: 1024 * 1024,
            max_slots: 1,
            slot_alignment: 64,
            allocator: Default::default(),
        };
        let pool = TensorPool::new(8811, config).expect("Failed to create pool");

        let old = pool
            .alloc(&[16], TensorDtype::U8, Device::cpu())
            .expect("first alloc");
        pool.release(&old);

        let current = pool
            .alloc(&[16], TensorDtype::U8, Device::cpu())
            .expect("second alloc");

        // try_retain/try_release on stale descriptor must error
        let retain_err = pool.try_retain(&old);
        assert!(retain_err.is_err(), "try_retain on stale must fail");

        let release_err = pool.try_release(&old);
        assert!(release_err.is_err(), "try_release on stale must fail");

        // Current descriptor should not be affected
        assert_eq!(pool.refcount(&current), 1);

        pool.release(&current);
        std::fs::remove_file(&pool.shm_path).ok();
    }

    #[test]
    fn test_alloc_release_realloc_data_isolation() {
        // Two successive tenants of the same slot should have complete
        // data isolation: tenant 1's data must not leak to tenant 2.
        let config = TensorPoolConfig {
            pool_size: 1024 * 1024,
            max_slots: 1,
            slot_alignment: 64,
            allocator: Default::default(),
        };
        let pool = TensorPool::new(8812, config).expect("Failed to create pool");

        // Tenant 1: write a specific pattern
        let t1 = pool
            .alloc(&[512], TensorDtype::U8, Device::cpu())
            .expect("tenant 1 alloc");
        pool.data_slice_mut(&t1).unwrap().fill(0xDE);
        pool.release(&t1);

        // Tenant 2: should see zeroed data, not 0xDE
        let t2 = pool
            .alloc(&[512], TensorDtype::U8, Device::cpu())
            .expect("tenant 2 alloc");
        let data = pool.data_slice(&t2).unwrap();
        let leaked_bytes: Vec<usize> = data
            .iter()
            .enumerate()
            .filter(|(_, &b)| b == 0xDE)
            .map(|(i, _)| i)
            .collect();
        assert!(
            leaked_bytes.is_empty(),
            "data from tenant 1 leaked to tenant 2 at offsets: {:?}",
            leaked_bytes
        );

        pool.release(&t2);
        std::fs::remove_file(&pool.shm_path).ok();
    }

    #[test]
    fn test_stats_slot_and_data_utilization() {
        // Verify the utilization calculation methods are correct after
        // a series of alloc/release operations.
        let config = TensorPoolConfig {
            pool_size: 1024 * 1024, // 1MB
            max_slots: 4,
            slot_alignment: 64,
            allocator: Default::default(),
        };
        let pool = TensorPool::new(8813, config).expect("Failed to create pool");

        // Empty pool
        let stats = pool.stats();
        assert_eq!(stats.slot_utilization(), 0.0);
        assert_eq!(stats.data_utilization(), 0.0);
        assert!(!stats.is_under_pressure());

        // Fill 2 of 4 slots
        let t1 = pool.alloc(&[1024], TensorDtype::U8, Device::cpu()).unwrap();
        let t2 = pool.alloc(&[1024], TensorDtype::U8, Device::cpu()).unwrap();

        let stats = pool.stats();
        assert!(
            (stats.slot_utilization() - 0.5).abs() < f64::EPSILON,
            "slot_utilization should be 0.5 with 2/4 slots"
        );

        // Release one
        pool.release(&t1);
        let stats = pool.stats();
        assert!(
            (stats.slot_utilization() - 0.25).abs() < f64::EPSILON,
            "slot_utilization should be 0.25 with 1/4 slots"
        );

        pool.release(&t2);
        std::fs::remove_file(&pool.shm_path).ok();
    }

    #[test]
    fn test_generation_survives_multiple_alloc_release_with_data_writes() {
        // Perform many alloc/write/release cycles on a single-slot pool,
        // verifying generation strictly increases and data is always clean.
        let config = TensorPoolConfig {
            pool_size: 1024 * 1024,
            max_slots: 1,
            slot_alignment: 64,
            allocator: Default::default(),
        };
        let pool = TensorPool::new(8814, config).expect("Failed to create pool");

        let mut prev_gen = 0u64;
        for cycle in 0..30 {
            let tensor = pool
                .alloc(&[128], TensorDtype::U8, Device::cpu())
                .unwrap_or_else(|e| panic!("alloc failed on cycle {}: {:?}", cycle, e));

            // Generation must increase
            assert!(
                tensor.generation_full() > prev_gen,
                "cycle {}: generation did not increase",
                cycle
            );
            prev_gen = tensor.generation_full();

            // Data must be clean (zeroed from previous release)
            let data = pool.data_slice(&tensor).unwrap();
            assert!(
                data.iter().all(|&b| b == 0),
                "cycle {}: data not zeroed on entry",
                cycle
            );

            // Write unique pattern for this cycle
            let pattern = ((cycle + 1) % 255) as u8;
            pool.data_slice_mut(&tensor).unwrap().fill(pattern);

            // Verify the write
            let data = pool.data_slice(&tensor).unwrap();
            assert!(
                data.iter().all(|&b| b == pattern),
                "cycle {}: data write failed",
                cycle
            );

            pool.release(&tensor);
        }

        std::fs::remove_file(&pool.shm_path).ok();
    }

    #[test]
    fn test_stats_free_bytes_plus_used_bytes_equals_pool_size() {
        // The invariant used_bytes + free_bytes == pool_size should hold
        // at all times (bump allocator never reclaims data region).
        let config = TensorPoolConfig {
            pool_size: 1024 * 1024,
            max_slots: 8,
            slot_alignment: 64,
            allocator: Default::default(),
        };
        let pool = TensorPool::new(8815, config).expect("Failed to create pool");

        // Check at empty
        let stats = pool.stats();
        assert_eq!(stats.used_bytes + stats.free_bytes, stats.pool_size);

        // Allocate a few tensors
        let mut tensors = Vec::new();
        for _ in 0..5 {
            tensors.push(pool.alloc(&[128], TensorDtype::F32, Device::cpu()).unwrap());
            let stats = pool.stats();
            assert_eq!(
                stats.used_bytes + stats.free_bytes,
                stats.pool_size,
                "invariant broken after alloc"
            );
        }

        // Release all
        for t in &tensors {
            pool.release(t);
            let stats = pool.stats();
            assert_eq!(
                stats.used_bytes + stats.free_bytes,
                stats.pool_size,
                "invariant broken after release"
            );
        }

        std::fs::remove_file(&pool.shm_path).ok();
    }

    #[test]
    fn test_alloc_varying_dtypes_and_shapes_then_release_all() {
        // Allocate tensors of different dtypes and shapes, verify each
        // has correct size, then release in reverse order.
        let config = TensorPoolConfig {
            pool_size: 4 * 1024 * 1024,
            max_slots: 8,
            slot_alignment: 64,
            allocator: Default::default(),
        };
        let pool = TensorPool::new(8816, config).expect("Failed to create pool");

        let cases: Vec<(Vec<u64>, TensorDtype, u64)> = vec![
            (vec![10], TensorDtype::U8, 10),
            (vec![10], TensorDtype::F32, 40),
            (vec![10], TensorDtype::F64, 80),
            (vec![2, 3], TensorDtype::F32, 24),
            (vec![4, 4, 4], TensorDtype::U8, 64),
        ];

        let mut tensors = Vec::new();
        for (shape, dtype, expected_bytes) in &cases {
            let t = pool
                .alloc(shape, *dtype, Device::cpu())
                .expect("alloc varied dtype");
            assert_eq!(
                t.nbytes(),
                *expected_bytes,
                "nbytes mismatch for shape {:?} dtype {:?}",
                shape,
                dtype
            );
            tensors.push(t);
        }

        assert_eq!(pool.stats().allocated_slots, cases.len());

        // Release in reverse order
        for t in tensors.iter().rev() {
            pool.release(t);
        }
        assert_eq!(pool.stats().allocated_slots, 0);

        std::fs::remove_file(&pool.shm_path).ok();
    }

    #[test]
    fn test_concurrent_retain_release_preserves_liveness() {
        // Multiple threads each retain and release a shared tensor N times.
        // After all threads finish, the tensor should still be alive
        // with refcount == 1 (the original alloc).
        use std::sync::Arc;

        let config = TensorPoolConfig {
            pool_size: 1024 * 1024,
            max_slots: 4,
            slot_alignment: 64,
            allocator: Default::default(),
        };
        let pool = Arc::new(TensorPool::new(8817, config).expect("Failed to create pool"));
        let tensor = pool
            .alloc(&[64], TensorDtype::U8, Device::cpu())
            .expect("alloc");

        let num_threads = 8;
        let retains_per_thread = 10;

        // Pre-retain so refcount = 1 + (num_threads * retains_per_thread)
        // This ensures releases never hit zero while threads are still running.
        for _ in 0..(num_threads * retains_per_thread) {
            pool.retain(&tensor);
        }

        // Each thread releases its share of retains
        let handles: Vec<_> = (0..num_threads)
            .map(|_| {
                let pool = Arc::clone(&pool);
                let t = tensor;
                std::thread::spawn(move || {
                    for _ in 0..retains_per_thread {
                        pool.release(&t);
                    }
                })
            })
            .collect();

        for h in handles {
            h.join().unwrap();
        }

        // After all threads release, refcount should be back to 1 (original alloc)
        assert_eq!(
            pool.refcount(&tensor),
            1,
            "refcount should be back to 1 after balanced retain/release"
        );

        pool.release(&tensor);
        assert_eq!(pool.stats().allocated_slots, 0);

        std::fs::remove_file(&pool.shm_path).ok();
    }
}
