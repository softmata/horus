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
//! │  SlotHeaders[max_slots] (32 bytes each)                    │
//! │  ├── refcount: AtomicU32                                   │
//! │  ├── generation: AtomicU32                                 │
//! │  ├── offset: u64                                           │
//! │  ├── size: u64                                             │
//! │  └── flags: AtomicU32                                      │
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

/// Current pool version
const POOL_VERSION: u32 = 1;

/// Slot flags
const SLOT_FREE: u32 = 0;
const SLOT_ALLOCATED: u32 = 1;
const SLOT_CUDA: u32 = 2;

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
#[repr(C)]
struct SlotHeader {
    refcount: AtomicU32,
    generation: AtomicU32,
    offset: u64,
    size: u64,
    flags: AtomicU32,
    next_free: AtomicU32,
    _padding: [u8; 4],
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
}

impl Default for TensorPoolConfig {
    fn default() -> Self {
        Self {
            pool_size: 1024 * 1024 * 1024, // 1GB
            max_slots: 1024,
            slot_alignment: 64,
        }
    }
}

impl TensorPoolConfig {
    /// Create a smaller pool for testing
    pub fn small() -> Self {
        Self {
            pool_size: 64 * 1024 * 1024, // 64MB
            max_slots: 256,
            slot_alignment: 64,
        }
    }

    /// Create a larger pool for production ML workloads
    pub fn large() -> Self {
        Self {
            pool_size: 4 * 1024 * 1024 * 1024, // 4GB
            max_slots: 4096,
            slot_alignment: 64,
        }
    }
}

/// Shared memory tensor pool
///
/// Manages a region of shared memory for tensor allocation with reference counting.
/// Multiple processes can attach to the same pool and share tensors with zero-copy.
pub struct TensorPool {
    config: TensorPoolConfig,
    pool_id: u32,
    shm_path: PathBuf,
    mmap: MmapMut,
    _file: File,
    is_owner: bool,
    slots_offset: usize,
    data_offset: usize,
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
        let total_size = data_offset + config.pool_size;

        // Try to open existing or create new
        let (file, is_owner) = if shm_path.exists() {
            let file = OpenOptions::new().read(true).write(true).open(&shm_path)?;
            let actual_size = file.metadata()?.len();
            if actual_size < total_size as u64 {
                file.set_len(total_size as u64)?;
            }
            (file, false)
        } else {
            let file = OpenOptions::new()
                .read(true)
                .write(true)
                .create(true)
                .truncate(true)
                .open(&shm_path)?;
            file.set_len(total_size as u64)?;
            (file, true)
        };

        // SAFETY: file is a valid open file descriptor with sufficient size for the mapping.
        let mmap = unsafe { MmapOptions::new().len(total_size).map_mut(&file)? };

        let mut pool = Self {
            config: config.clone(),
            pool_id,
            shm_path,
            mmap,
            _file: file,
            is_owner,
            slots_offset: header_size,
            data_offset,
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
            slot.generation.store(0, Ordering::Release);
            slot.offset = 0;
            slot.size = 0;
            slot.flags.store(SLOT_FREE, Ordering::Release);
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
    /// Returns a HorusTensor descriptor pointing to the allocated memory.
    pub fn alloc(
        &self,
        shape: &[u64],
        dtype: TensorDtype,
        device: Device,
    ) -> HorusResult<HorusTensor> {
        // Calculate required size
        let num_elements: u64 = shape.iter().product();
        let element_size = dtype.element_size() as u64;
        let size = num_elements * element_size;
        let aligned_size = Self::align_up(size as usize, self.config.slot_alignment);

        // Find a free slot
        let slot_id = self.find_free_slot()?;
        let slot = self.slot_mut(slot_id);

        // Allocate from data region
        let offset = self.allocate_data(aligned_size)?;

        // Initialize slot
        let generation = slot.generation.fetch_add(1, Ordering::AcqRel) + 1;
        slot.offset = offset as u64;
        slot.size = size;
        slot.refcount.store(1, Ordering::Release);
        slot.flags.store(
            if device.is_cuda() {
                SLOT_CUDA
            } else {
                SLOT_ALLOCATED
            },
            Ordering::Release,
        );

        // Create tensor descriptor
        Ok(HorusTensor::new(
            self.pool_id,
            slot_id,
            generation,
            offset as u64,
            shape,
            dtype,
            device,
        ))
    }

    /// Increment reference count for a tensor
    #[inline]
    pub fn retain(&self, tensor: &HorusTensor) {
        if tensor.pool_id != self.pool_id {
            return;
        }

        let slot = self.slot(tensor.slot_id);

        // Verify generation matches (ABA prevention)
        if slot.generation.load(Ordering::Acquire) != tensor.generation {
            return;
        }

        slot.refcount.fetch_add(1, Ordering::AcqRel);
    }

    /// Decrement reference count for a tensor
    ///
    /// If the count reaches zero, the slot is returned to the free list.
    #[inline]
    pub fn release(&self, tensor: &HorusTensor) {
        if tensor.pool_id != self.pool_id {
            return;
        }

        let slot = self.slot_mut(tensor.slot_id);

        // Verify generation matches
        if slot.generation.load(Ordering::Acquire) != tensor.generation {
            return;
        }

        let prev = slot.refcount.fetch_sub(1, Ordering::AcqRel);
        if prev == 1 {
            // Last reference, return slot to free list
            self.return_slot(tensor.slot_id);
        }
    }

    /// Get raw pointer to tensor data
    #[inline]
    pub fn data_ptr(&self, tensor: &HorusTensor) -> *mut u8 {
        if tensor.pool_id != self.pool_id {
            return std::ptr::null_mut();
        }

        // SAFETY: data_offset + tensor.offset is within bounds of the mmap region.
        unsafe {
            self.mmap
                .as_ptr()
                .add(self.data_offset + tensor.offset as usize) as *mut u8
        }
    }

    /// Get data as slice
    pub fn data_slice(&self, tensor: &HorusTensor) -> &[u8] {
        if tensor.pool_id != self.pool_id {
            return &[];
        }

        // Bounds check: ensure offset + size fits within the mmap'd data region
        let offset = tensor.offset as usize;
        let size = tensor.size as usize;
        let data_region_size = self.mmap.len().saturating_sub(self.data_offset);
        if offset
            .checked_add(size)
            .is_none_or(|end| end > data_region_size)
        {
            return &[];
        }

        // SAFETY: data_offset + tensor.offset + tensor.size is within mmap bounds (checked above).
        unsafe {
            let ptr = self.mmap.as_ptr().add(self.data_offset + offset);
            std::slice::from_raw_parts(ptr, size)
        }
    }

    /// Get data as mutable slice
    #[allow(clippy::mut_from_ref)]
    pub fn data_slice_mut(&self, tensor: &HorusTensor) -> &mut [u8] {
        if tensor.pool_id != self.pool_id {
            return &mut [];
        }

        // Bounds check: ensure offset + size fits within the mmap'd data region
        let offset = tensor.offset as usize;
        let size = tensor.size as usize;
        let data_region_size = self.mmap.len().saturating_sub(self.data_offset);
        if offset
            .checked_add(size)
            .is_none_or(|end| end > data_region_size)
        {
            return &mut [];
        }

        // SAFETY: data_offset + tensor.offset + tensor.size is within mmap bounds (checked above).
        unsafe {
            let ptr = self.mmap.as_ptr().add(self.data_offset + offset) as *mut u8;
            std::slice::from_raw_parts_mut(ptr, size)
        }
    }

    /// Get reference count for a tensor
    pub fn refcount(&self, tensor: &HorusTensor) -> u32 {
        if tensor.pool_id != self.pool_id {
            return 0;
        }

        let slot = self.slot(tensor.slot_id);
        if slot.generation.load(Ordering::Acquire) != tensor.generation {
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
    }

    fn allocate_data(&self, size: usize) -> HorusResult<usize> {
        let header = self.header();

        loop {
            let current = header.next_alloc_offset.load(Ordering::Acquire) as usize;
            let aligned_current = Self::align_up(current, self.config.slot_alignment);
            let new_offset = aligned_current + size;

            if new_offset > self.config.pool_size {
                return Err(HorusError::Memory(format!(
                    "Tensor pool out of memory: need {} bytes, only {} available",
                    size,
                    self.config.pool_size - current
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
        (value + alignment - 1) & !(alignment - 1)
    }
}

impl Drop for TensorPool {
    fn drop(&mut self) {
        // Don't delete the file - other processes may still be using it
        // The file can be cleaned up manually or by a cleanup routine
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

// Re-export tensor types from horus_types (canonical source)
pub use horus_types::{Device, HorusTensor, TensorDtype, MAX_TENSOR_DIMS};

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_pool_creation() {
        let config = TensorPoolConfig {
            pool_size: 1024 * 1024, // 1MB for testing
            max_slots: 16,
            slot_alignment: 64,
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
        };

        let pool = TensorPool::new(9997, config).expect("Failed to create pool");

        let tensor = pool
            .alloc(&[10], TensorDtype::U8, Device::cpu())
            .expect("Failed to allocate tensor");

        // Write data
        let data = pool.data_slice_mut(&tensor);
        for (i, byte) in data.iter_mut().enumerate() {
            *byte = i as u8;
        }

        // Read data
        let data = pool.data_slice(&tensor);
        for (i, &byte) in data.iter().enumerate() {
            assert_eq!(byte, i as u8);
        }

        pool.release(&tensor);
        std::fs::remove_file(&pool.shm_path).ok();
    }
}
