//! CUDA Tensor Pool for GPU memory sharing across processes
//!
//! This module provides a GPU memory pool that supports inter-process communication
//! via CUDA IPC handles. It mirrors the CPU TensorPool design but manages GPU memory.
//!
//! # Architecture
//!
//! ```text
//! ┌─────────────────────────────────────────────────────────────────┐
//! │                    CudaTensorPool Design                         │
//! ├─────────────────────────────────────────────────────────────────┤
//! │  Process A (Owner)              Process B (Consumer)            │
//! │  ┌─────────────────┐            ┌─────────────────┐             │
//! │  │ cudaMalloc      │            │                 │             │
//! │  │    ↓            │            │                 │             │
//! │  │ GPU Memory      │ ═══════════│ GPU Memory      │             │
//! │  │ (same physical) │  IPC Handle│ (same physical) │             │
//! │  │    ↓            │   64 bytes │    ↓            │             │
//! │  │ IpcGetHandle    │────────────│ IpcOpenHandle   │             │
//! │  └─────────────────┘            └─────────────────┘             │
//! │                                                                  │
//! │  Shared Memory (CPU): Stores IPC handles + metadata              │
//! │  ┌──────────────────────────────────────────────────┐           │
//! │  │ slot[0]: {ipc_handle, size, refcount, device_id} │           │
//! │  │ slot[1]: {ipc_handle, size, refcount, device_id} │           │
//! │  │ ...                                               │           │
//! │  └──────────────────────────────────────────────────┘           │
//! └─────────────────────────────────────────────────────────────────┘
//! ```
//!
//! # Usage
//!
//! ```rust,ignore
//! // Process A: Allocate GPU memory
//! let pool = CudaTensorPool::new(1, 0)?;  // pool_id=1, device=0
//! let tensor = pool.alloc(&[1080, 1920, 3], TensorDtype::F32)?;
//!
//! // Get IPC handle (64 bytes) to share with other processes
//! let handle = tensor.ipc_handle();
//!
//! // Process B: Open shared GPU memory
//! let pool = CudaTensorPool::open(1, 0)?;
//! let tensor = pool.from_ipc_handle(handle, &[1080, 1920, 3], TensorDtype::F32)?;
//! // Now tensor points to the SAME GPU memory as Process A
//! ```

use crate::error::{HorusError, HorusResult};
use crate::memory::platform::shm_base_dir;
use crate::memory::tensor_pool::{Device, TensorDtype, MAX_TENSOR_DIMS};
use memmap2::{MmapMut, MmapOptions};
use std::ffi::c_void;
use std::fs::{File, OpenOptions};
use std::sync::atomic::{AtomicU32, AtomicU64, Ordering};

use super::cuda_ffi::{self, CUDA_IPC_HANDLE_SIZE};

/// Magic number for CUDA pool validation
const CUDA_POOL_MAGIC: u64 = 0x484F52555343554; // "HORUS_CU" in hex

/// Pool version
const CUDA_POOL_VERSION: u32 = 1;

/// Maximum slots per pool
const MAX_CUDA_SLOTS: usize = 256;

/// Slot states
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

/// CUDA pool header stored in shared memory
#[repr(C)]
struct CudaPoolHeader {
    magic: u64,
    version: u32,
    pool_id: u32,
    device_id: u32,
    max_slots: u32,
    allocated_count: AtomicU32,
    /// Head of free slot stack (for O(1) allocation)
    free_stack_head: AtomicU64,
    _padding: [u8; 32], // Pad to 64 bytes
}

/// Per-slot metadata in shared memory
#[repr(C)]
struct CudaSlotHeader {
    /// IPC handle (64 bytes)
    ipc_handle: [u8; CUDA_IPC_HANDLE_SIZE],
    /// Device pointer (stored for owner process, reconstructed for others)
    device_ptr: AtomicU64,
    /// Allocation size in bytes
    size: u64,
    /// Number of elements
    numel: u64,
    /// Shape dimensions
    shape: [u64; MAX_TENSOR_DIMS],
    /// Number of dimensions
    ndim: u8,
    /// Data type
    dtype: u8,
    /// Slot state
    state: AtomicU32,
    /// Reference count
    refcount: AtomicU32,
    /// Generation counter (ABA prevention)
    generation: AtomicU32,
    /// Next free slot in free stack (for O(1) allocation)
    next_free: AtomicU32,
    _padding: [u8; 14], // Align to 256 bytes total
}

/// Configuration for CUDA tensor pool
#[derive(Clone, Debug)]
pub struct CudaTensorPoolConfig {
    /// Maximum number of tensor slots
    pub max_slots: usize,
}

impl Default for CudaTensorPoolConfig {
    fn default() -> Self {
        Self {
            max_slots: MAX_CUDA_SLOTS,
        }
    }
}

/// A GPU tensor descriptor with IPC support
#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct CudaTensor {
    /// Pool ID this tensor belongs to
    pub pool_id: u32,
    /// Slot index in the pool
    pub slot_id: u32,
    /// Generation counter for ABA prevention
    pub generation: u32,
    /// Device ID
    pub device_id: u32,
    /// Size in bytes
    pub size: u64,
    /// Number of elements
    pub numel: u64,
    /// Byte offset into the underlying buffer (for views/slices)
    pub offset: u64,
    /// Data type
    pub dtype: TensorDtype,
    /// Number of dimensions
    pub ndim: u8,
    pub _pad: [u8; 6],
    /// Shape
    pub shape: [u64; MAX_TENSOR_DIMS],
    /// Strides in elements (for views/slices)
    pub strides: [u64; MAX_TENSOR_DIMS],
    /// IPC handle for cross-process sharing
    pub ipc_handle: [u8; CUDA_IPC_HANDLE_SIZE],
}

impl CudaTensor {
    /// Get the IPC handle bytes for sharing
    pub fn ipc_handle_bytes(&self) -> &[u8] {
        &self.ipc_handle
    }

    /// Get device
    pub fn device(&self) -> Device {
        Device::cuda(self.device_id)
    }

    /// Check if tensor is contiguous in memory
    pub fn is_contiguous(&self) -> bool {
        if self.ndim == 0 {
            return true;
        }

        // Check strides match contiguous layout (row-major)
        let mut expected_stride = 1u64;
        for i in (0..self.ndim as usize).rev() {
            if self.strides[i] != expected_stride {
                return false;
            }
            expected_stride *= self.shape[i];
        }
        true
    }

    /// Create a view of this tensor with different shape
    /// Only works for contiguous tensors with matching element count
    pub fn view(&self, new_shape: &[u64]) -> Option<Self> {
        let old_numel: u64 = self.shape[..self.ndim as usize].iter().product();
        let new_numel: u64 = new_shape.iter().product();

        if old_numel != new_numel || !self.is_contiguous() {
            return None;
        }

        if new_shape.len() > MAX_TENSOR_DIMS {
            return None;
        }

        let mut result = *self;
        result.ndim = new_shape.len() as u8;

        // Set new shape
        result.shape = [0; MAX_TENSOR_DIMS];
        for (i, &dim) in new_shape.iter().enumerate() {
            result.shape[i] = dim;
        }

        // Compute contiguous strides for new shape
        result.strides = [0; MAX_TENSOR_DIMS];
        let mut stride = 1u64;
        for i in (0..new_shape.len()).rev() {
            result.strides[i] = stride;
            stride *= new_shape[i];
        }

        Some(result)
    }

    /// Create a slice/view of this tensor along the first dimension
    /// Returns a view into [start, end) of the first dimension
    pub fn slice_first_dim(&self, start: u64, end: u64) -> Option<Self> {
        if self.ndim == 0 || start >= end || end > self.shape[0] {
            return None;
        }

        let mut result = *self;
        result.shape[0] = end - start;
        result.offset += start * self.strides[0] * self.dtype.element_size() as u64;

        // Recompute numel and size
        let new_numel: u64 = result.shape[..result.ndim as usize].iter().product();
        result.numel = new_numel;
        result.size = new_numel * self.dtype.element_size() as u64;

        Some(result)
    }

    /// Create a slice along any dimension
    /// Returns a view into [start, end) of the specified dimension
    pub fn slice_dim(&self, dim: usize, start: u64, end: u64) -> Option<Self> {
        if dim >= self.ndim as usize || start >= end || end > self.shape[dim] {
            return None;
        }

        let mut result = *self;
        result.offset += start * self.strides[dim] * self.dtype.element_size() as u64;
        result.shape[dim] = end - start;

        // Recompute numel and size
        let new_numel: u64 = result.shape[..result.ndim as usize].iter().product();
        result.numel = new_numel;
        result.size = new_numel * self.dtype.element_size() as u64;

        Some(result)
    }

    /// Transpose two dimensions (creates a non-contiguous view)
    pub fn transpose(&self, dim0: usize, dim1: usize) -> Option<Self> {
        if dim0 >= self.ndim as usize || dim1 >= self.ndim as usize {
            return None;
        }

        if dim0 == dim1 {
            return Some(*self);
        }

        let mut result = *self;
        result.shape.swap(dim0, dim1);
        result.strides.swap(dim0, dim1);

        Some(result)
    }

    /// Squeeze dimension (remove dimension of size 1)
    pub fn squeeze(&self, dim: usize) -> Option<Self> {
        if dim >= self.ndim as usize || self.shape[dim] != 1 {
            return None;
        }

        if self.ndim == 1 {
            // Can't squeeze scalar
            return None;
        }

        let mut result = *self;
        result.ndim -= 1;

        // Shift dimensions down
        for i in dim..result.ndim as usize {
            result.shape[i] = result.shape[i + 1];
            result.strides[i] = result.strides[i + 1];
        }
        result.shape[result.ndim as usize] = 0;
        result.strides[result.ndim as usize] = 0;

        Some(result)
    }

    /// Unsqueeze (add dimension of size 1 at position)
    pub fn unsqueeze(&self, dim: usize) -> Option<Self> {
        if dim > self.ndim as usize || self.ndim as usize >= MAX_TENSOR_DIMS {
            return None;
        }

        let mut result = *self;
        result.ndim += 1;

        // Shift dimensions up
        for i in (dim + 1..result.ndim as usize).rev() {
            result.shape[i] = result.shape[i - 1];
            result.strides[i] = result.strides[i - 1];
        }
        result.shape[dim] = 1;
        // Stride doesn't matter for size-1 dimension
        result.strides[dim] = if dim + 1 < result.ndim as usize {
            result.shape[dim + 1] * result.strides[dim + 1]
        } else {
            1
        };

        Some(result)
    }

    /// Get the effective device pointer (base + offset)
    pub fn effective_offset(&self) -> u64 {
        self.offset
    }
}

/// CUDA Tensor Pool for GPU memory with IPC support
pub struct CudaTensorPool {
    pool_id: u32,
    device_id: i32,
    mmap: MmapMut,
    _file: File,
    is_owner: bool,
}

// SAFETY: CudaTensorPool can be sent between threads. The pool's shared state
// (allocated_count, free_stack_head, slot state/refcount/generation) is accessed
// exclusively through atomics with appropriate orderings. The mmap region is
// process-shared memory backed by a file; concurrent access to slots is
// coordinated via atomic CAS (free stack) or atomic state transitions.
// CUDA IPC handles are process-safe by design (64-byte opaque tokens).
unsafe impl Send for CudaTensorPool {}
unsafe impl Sync for CudaTensorPool {}

impl CudaTensorPool {
    /// Create a new CUDA tensor pool
    ///
    /// # Arguments
    /// * `pool_id` - Unique identifier for this pool
    /// * `device_id` - CUDA device index (0, 1, 2, ...)
    /// * `config` - Pool configuration
    pub fn new(pool_id: u32, device_id: i32, config: CudaTensorPoolConfig) -> HorusResult<Self> {
        // Verify CUDA is available
        if !cuda_ffi::cuda_available() {
            return Err(HorusError::Config("CUDA not available".into()));
        }

        // Set device
        cuda_ffi::set_device(device_id)
            .map_err(|e| HorusError::Config(format!("Failed to set CUDA device: {}", e)))?;

        let shm_dir = shm_base_dir().join("cuda");
        std::fs::create_dir_all(&shm_dir)?;

        let shm_path = shm_dir.join(format!("cuda_pool_{}_{}", pool_id, device_id));

        // Calculate layout
        let header_size = std::mem::size_of::<CudaPoolHeader>();
        let slots_size = config.max_slots * std::mem::size_of::<CudaSlotHeader>();
        let total_size = header_size + slots_size;

        // Create shared memory file
        let file = OpenOptions::new()
            .read(true)
            .write(true)
            .create(true)
            .truncate(true)
            .open(&shm_path)?;
        file.set_len(total_size as u64)?;

        // SAFETY: file is a valid open file descriptor with sufficient size for the mapping.
        let mmap = unsafe { MmapOptions::new().len(total_size).map_mut(&file)? };

        let mut pool = Self {
            pool_id,
            device_id,
            mmap,
            _file: file,
            is_owner: true,
        };

        pool.initialize(config.max_slots)?;

        Ok(pool)
    }

    /// Open an existing CUDA tensor pool
    pub fn open(pool_id: u32, device_id: i32) -> HorusResult<Self> {
        // Verify CUDA is available
        if !cuda_ffi::cuda_available() {
            return Err(HorusError::Config("CUDA not available".into()));
        }

        cuda_ffi::set_device(device_id)
            .map_err(|e| HorusError::Config(format!("Failed to set CUDA device: {}", e)))?;

        let shm_dir = shm_base_dir().join("cuda");
        let shm_path = shm_dir.join(format!("cuda_pool_{}_{}", pool_id, device_id));

        if !shm_path.exists() {
            return Err(HorusError::Config(format!(
                "CUDA pool {} does not exist",
                pool_id
            )));
        }

        let file = OpenOptions::new().read(true).write(true).open(&shm_path)?;

        let metadata = file.metadata()?;
        let total_size = metadata.len() as usize;

        // SAFETY: file is a valid open file descriptor with sufficient size for the mapping.
        let mmap = unsafe { MmapOptions::new().len(total_size).map_mut(&file)? };

        let pool = Self {
            pool_id,
            device_id,
            mmap,
            _file: file,
            is_owner: false,
        };

        pool.validate()?;

        Ok(pool)
    }

    /// Initialize pool header and slots
    fn initialize(&mut self, max_slots: usize) -> HorusResult<()> {
        self.mmap.fill(0);

        // Copy values before mutable borrow
        let pool_id = self.pool_id;
        let device_id = self.device_id as u32;

        let header = self.header_mut();
        header.magic = CUDA_POOL_MAGIC;
        header.version = CUDA_POOL_VERSION;
        header.pool_id = pool_id;
        header.device_id = device_id;
        header.max_slots = max_slots as u32;
        header.allocated_count.store(0, Ordering::Release);
        header
            .free_stack_head
            .store(INVALID_SLOT as u64, Ordering::Release);

        // Initialize all slots as free
        for i in 0..max_slots {
            let slot = self.slot_mut(i as u32);
            slot.state.store(SLOT_FREE, Ordering::Release);
            slot.refcount.store(0, Ordering::Release);
            slot.generation.store(0, Ordering::Release);
            slot.next_free.store(INVALID_SLOT, Ordering::Release);
        }

        self.mmap.flush()?;
        Ok(())
    }

    /// Validate pool header
    fn validate(&self) -> HorusResult<()> {
        let header = self.header();

        if header.magic != CUDA_POOL_MAGIC {
            return Err(HorusError::Config("Invalid CUDA pool magic".into()));
        }

        if header.version != CUDA_POOL_VERSION {
            return Err(HorusError::Config(format!(
                "CUDA pool version mismatch: expected {}, got {}",
                CUDA_POOL_VERSION, header.version
            )));
        }

        Ok(())
    }

    /// Allocate a GPU tensor
    pub fn alloc(&self, shape: &[u64], dtype: TensorDtype) -> HorusResult<CudaTensor> {
        let numel: u64 = shape.iter().product();
        let size = numel * dtype.element_size() as u64;

        // Find a free slot
        let slot_id = self.find_free_slot()?;

        // Allocate GPU memory
        let dev_ptr = cuda_ffi::malloc(size as usize)
            .map_err(|e| HorusError::Memory(format!("CUDA malloc failed: {}", e)))?;

        // Get IPC handle
        let ipc_handle = cuda_ffi::ipc_get_mem_handle(dev_ptr)
            .map_err(|e| HorusError::Memory(format!("Failed to get IPC handle: {}", e)))?;

        // Update slot metadata
        let slot = self.slot_mut(slot_id);
        slot.ipc_handle.copy_from_slice(&ipc_handle.reserved);
        slot.device_ptr.store(dev_ptr as u64, Ordering::Release);
        slot.size = size;
        slot.numel = numel;
        slot.ndim = shape.len().min(MAX_TENSOR_DIMS) as u8;
        slot.dtype = dtype as u8;

        for (i, &dim) in shape.iter().take(MAX_TENSOR_DIMS).enumerate() {
            slot.shape[i] = dim;
        }

        let generation = slot.generation.fetch_add(1, Ordering::AcqRel) + 1;
        slot.refcount.store(1, Ordering::Release);
        slot.state.store(SLOT_ALLOCATED, Ordering::Release);

        // Update pool count
        self.header().allocated_count.fetch_add(1, Ordering::AcqRel);

        // Build tensor descriptor
        let mut tensor = CudaTensor {
            pool_id: self.pool_id,
            slot_id,
            generation,
            device_id: self.device_id as u32,
            size,
            numel,
            offset: 0, // New allocations start at offset 0
            dtype,
            ndim: shape.len().min(MAX_TENSOR_DIMS) as u8,
            _pad: [0; 6],
            shape: [0; MAX_TENSOR_DIMS],
            strides: [0; MAX_TENSOR_DIMS],
            ipc_handle: [0; CUDA_IPC_HANDLE_SIZE],
        };

        // Set shape and compute contiguous strides
        let ndim = shape.len().min(MAX_TENSOR_DIMS);
        for (i, &dim) in shape.iter().take(ndim).enumerate() {
            tensor.shape[i] = dim;
        }

        // Compute row-major contiguous strides
        let mut stride = 1u64;
        for i in (0..ndim).rev() {
            tensor.strides[i] = stride;
            stride *= tensor.shape[i];
        }

        tensor.ipc_handle.copy_from_slice(&ipc_handle.reserved);

        Ok(tensor)
    }

    /// Import a tensor from an IPC handle (for cross-process sharing)
    pub fn import_ipc(
        &self,
        ipc_handle_bytes: &[u8],
        shape: &[u64],
        dtype: TensorDtype,
    ) -> HorusResult<(*mut c_void, CudaTensor)> {
        if ipc_handle_bytes.len() != CUDA_IPC_HANDLE_SIZE {
            return Err(HorusError::Config(format!(
                "Invalid IPC handle size: expected {}, got {}",
                CUDA_IPC_HANDLE_SIZE,
                ipc_handle_bytes.len()
            )));
        }

        // Reconstruct IPC handle
        let mut handle = cuda_ffi::CudaIpcMemHandle::default();
        handle.reserved.copy_from_slice(ipc_handle_bytes);

        // Open the shared GPU memory
        let dev_ptr = cuda_ffi::ipc_open_mem_handle(handle)
            .map_err(|e| HorusError::Memory(format!("Failed to open IPC handle: {}", e)))?;

        let numel: u64 = shape.iter().product();
        let size = numel * dtype.element_size() as u64;

        // Build tensor descriptor
        let mut tensor = CudaTensor {
            pool_id: self.pool_id,
            slot_id: u32::MAX, // Imported tensors don't have a slot
            generation: 0,
            device_id: self.device_id as u32,
            size,
            numel,
            offset: 0,
            dtype,
            ndim: shape.len().min(MAX_TENSOR_DIMS) as u8,
            _pad: [0; 6],
            shape: [0; MAX_TENSOR_DIMS],
            strides: [0; MAX_TENSOR_DIMS],
            ipc_handle: [0; CUDA_IPC_HANDLE_SIZE],
        };

        // Set shape and compute contiguous strides
        let ndim = shape.len().min(MAX_TENSOR_DIMS);
        for (i, &dim) in shape.iter().take(ndim).enumerate() {
            tensor.shape[i] = dim;
        }

        // Compute row-major contiguous strides
        let mut stride = 1u64;
        for i in (0..ndim).rev() {
            tensor.strides[i] = stride;
            stride *= tensor.shape[i];
        }

        tensor.ipc_handle.copy_from_slice(ipc_handle_bytes);

        Ok((dev_ptr, tensor))
    }

    /// Close an imported IPC handle
    pub fn close_ipc(&self, dev_ptr: *mut c_void) -> HorusResult<()> {
        cuda_ffi::ipc_close_mem_handle(dev_ptr)
            .map_err(|e| HorusError::Memory(format!("Failed to close IPC handle: {}", e)))
    }

    /// Release a tensor (decrement refcount, free if zero)
    pub fn release(&self, tensor: &CudaTensor) -> HorusResult<()> {
        if tensor.pool_id != self.pool_id {
            return Ok(()); // Not our tensor
        }

        if tensor.slot_id == u32::MAX {
            return Ok(()); // Imported tensor, no slot
        }

        let slot = self.slot_mut(tensor.slot_id);

        // Verify generation
        if slot.generation.load(Ordering::Acquire) != tensor.generation {
            return Ok(()); // Stale reference
        }

        let prev_refcount = slot.refcount.fetch_sub(1, Ordering::AcqRel);
        if prev_refcount == 1 {
            // Last reference, free GPU memory
            let dev_ptr = slot.device_ptr.load(Ordering::Acquire) as *mut c_void;
            if !dev_ptr.is_null() {
                cuda_ffi::free(dev_ptr)
                    .map_err(|e| HorusError::Memory(format!("CUDA free failed: {}", e)))?;
            }

            // Return slot to free stack for O(1) reallocation
            self.return_slot(tensor.slot_id);
            self.header().allocated_count.fetch_sub(1, Ordering::AcqRel);
        }

        Ok(())
    }

    /// Retain a tensor (increment refcount)
    pub fn retain(&self, tensor: &CudaTensor) {
        if tensor.pool_id != self.pool_id || tensor.slot_id == u32::MAX {
            return;
        }

        let slot = self.slot(tensor.slot_id);
        if slot.generation.load(Ordering::Acquire) == tensor.generation {
            slot.refcount.fetch_add(1, Ordering::AcqRel);
        }
    }

    /// Get base device pointer for a tensor (without offset)
    pub fn base_device_ptr(&self, tensor: &CudaTensor) -> *mut c_void {
        if tensor.pool_id != self.pool_id || tensor.slot_id == u32::MAX {
            return std::ptr::null_mut();
        }

        let slot = self.slot(tensor.slot_id);
        if slot.generation.load(Ordering::Acquire) != tensor.generation {
            return std::ptr::null_mut();
        }

        slot.device_ptr.load(Ordering::Acquire) as *mut c_void
    }

    /// Get device pointer for a tensor (includes offset for views/slices)
    pub fn device_ptr(&self, tensor: &CudaTensor) -> *mut c_void {
        let base = self.base_device_ptr(tensor);
        if base.is_null() {
            return base;
        }

        // Add offset for views/slices
        if tensor.offset > 0 {
            // SAFETY: offset is within bounds of the allocated GPU memory region.
            unsafe { base.add(tensor.offset as usize) }
        } else {
            base
        }
    }

    /// Get pool statistics
    pub fn stats(&self) -> CudaPoolStats {
        let header = self.header();
        CudaPoolStats {
            pool_id: self.pool_id,
            device_id: self.device_id,
            max_slots: header.max_slots as usize,
            allocated_slots: header.allocated_count.load(Ordering::Relaxed) as usize,
        }
    }

    /// Get pool ID
    pub fn pool_id(&self) -> u32 {
        self.pool_id
    }

    /// Get device ID
    pub fn device_id(&self) -> i32 {
        self.device_id
    }

    /// Check if this instance owns the pool
    pub fn is_owner(&self) -> bool {
        self.is_owner
    }

    // === Private helpers ===

    fn header(&self) -> &CudaPoolHeader {
        // SAFETY: mmap region is properly sized and aligned for CudaPoolHeader; initialized at creation.
        unsafe { &*(self.mmap.as_ptr() as *const CudaPoolHeader) }
    }

    fn header_mut(&mut self) -> &mut CudaPoolHeader {
        // SAFETY: exclusive access via &mut self; mmap is properly sized and aligned for CudaPoolHeader.
        unsafe { &mut *(self.mmap.as_mut_ptr() as *mut CudaPoolHeader) }
    }

    fn slot(&self, index: u32) -> &CudaSlotHeader {
        let offset = std::mem::size_of::<CudaPoolHeader>()
            + (index as usize) * std::mem::size_of::<CudaSlotHeader>();
        // SAFETY: offset is within bounds of mmap region; properly aligned for CudaSlotHeader.
        unsafe { &*(self.mmap.as_ptr().add(offset) as *const CudaSlotHeader) }
    }

    #[allow(clippy::mut_from_ref)]
    fn slot_mut(&self, index: u32) -> &mut CudaSlotHeader {
        let offset = std::mem::size_of::<CudaPoolHeader>()
            + (index as usize) * std::mem::size_of::<CudaSlotHeader>();
        // SAFETY: offset is within bounds of mmap region; properly aligned for CudaSlotHeader.
        unsafe { &mut *(self.mmap.as_ptr().add(offset) as *mut CudaSlotHeader) }
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
                    // Successfully popped, now mark as allocated
                    slot.state.store(SLOT_ALLOCATED, Ordering::Release);
                    return Ok(slot_id);
                }
                // CAS failed, retry
                continue;
            }
            break;
        }

        // No free slots in stack, fall back to linear search (O(n) slow path)
        let max_slots = header.max_slots as usize;
        for i in 0..max_slots {
            let slot = self.slot(i as u32);
            if slot
                .state
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

        Err(HorusError::Memory("No free CUDA tensor slots".into()))
    }

    /// Return a slot to the free stack (O(1))
    fn return_slot(&self, slot_id: u32) {
        let header = self.header();
        let slot = self.slot_mut(slot_id);

        // Mark as free
        slot.state.store(SLOT_FREE, Ordering::Release);

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
}

impl Drop for CudaTensorPool {
    fn drop(&mut self) {
        // If we're the owner, free all allocated GPU memory
        if self.is_owner {
            let header = self.header();
            let max_slots = header.max_slots as usize;

            for i in 0..max_slots {
                let slot = self.slot(i as u32);
                if slot.state.load(Ordering::Acquire) == SLOT_ALLOCATED {
                    let dev_ptr = slot.device_ptr.load(Ordering::Acquire) as *mut c_void;
                    if !dev_ptr.is_null() {
                        let _ = cuda_ffi::free(dev_ptr);
                    }
                }
            }
        }
    }
}

/// Statistics for CUDA tensor pool
#[derive(Clone, Debug)]
pub struct CudaPoolStats {
    pub pool_id: u32,
    pub device_id: i32,
    pub max_slots: usize,
    pub allocated_slots: usize,
}

// =============================================================================
// Multi-GPU Peer-to-Peer (P2P) Support
// =============================================================================

/// Information about P2P access between devices
#[derive(Clone, Debug)]
pub struct P2PAccessInfo {
    /// Device ID of the source
    pub device: i32,
    /// Device ID of the peer
    pub peer_device: i32,
    /// Whether P2P access is supported between these devices
    pub can_access: bool,
    /// Whether P2P access is currently enabled
    pub is_enabled: bool,
}

/// Multi-GPU P2P manager for cross-GPU tensor transfers
pub struct P2PManager {
    /// Enabled P2P connections (source_device, peer_device)
    enabled_connections: std::sync::Mutex<Vec<(i32, i32)>>,
}

impl P2PManager {
    /// Create a new P2P manager
    pub fn new() -> Self {
        Self {
            enabled_connections: std::sync::Mutex::new(Vec::new()),
        }
    }

    /// Check if P2P access is possible between two devices
    pub fn can_access_peer(device: i32, peer_device: i32) -> HorusResult<bool> {
        if device == peer_device {
            return Ok(true);
        }

        cuda_ffi::device_can_access_peer(device, peer_device)
            .map_err(|e| HorusError::Memory(format!("Failed to check P2P access: {}", e)))
    }

    /// Enable P2P access between two devices (allows direct GPU-to-GPU transfers)
    pub fn enable_peer_access(&self, device: i32, peer_device: i32) -> HorusResult<()> {
        if device == peer_device {
            return Ok(());
        }

        // Check if already enabled
        {
            let connections = self.enabled_connections.lock().unwrap();
            if connections.contains(&(device, peer_device)) {
                return Ok(());
            }
        }

        // Check if P2P is possible
        if !Self::can_access_peer(device, peer_device)? {
            return Err(HorusError::Config(format!(
                "P2P access not supported between devices {} and {}",
                device, peer_device
            )));
        }

        // Set current device and enable peer access
        cuda_ffi::set_device(device)
            .map_err(|e| HorusError::Memory(format!("Failed to set device: {}", e)))?;

        cuda_ffi::device_enable_peer_access(peer_device)
            .map_err(|e| HorusError::Memory(format!("Failed to enable P2P access: {}", e)))?;

        // Record the enabled connection
        {
            let mut connections = self.enabled_connections.lock().unwrap();
            connections.push((device, peer_device));
        }

        Ok(())
    }

    /// Disable P2P access between two devices
    pub fn disable_peer_access(&self, device: i32, peer_device: i32) -> HorusResult<()> {
        if device == peer_device {
            return Ok(());
        }

        // Check if enabled
        {
            let connections = self.enabled_connections.lock().unwrap();
            if !connections.contains(&(device, peer_device)) {
                return Ok(()); // Already disabled
            }
        }

        // Set current device and disable peer access
        cuda_ffi::set_device(device)
            .map_err(|e| HorusError::Memory(format!("Failed to set device: {}", e)))?;

        cuda_ffi::device_disable_peer_access(peer_device)
            .map_err(|e| HorusError::Memory(format!("Failed to disable P2P access: {}", e)))?;

        // Remove the connection
        {
            let mut connections = self.enabled_connections.lock().unwrap();
            connections.retain(|&(d, p)| !(d == device && p == peer_device));
        }

        Ok(())
    }

    /// Enable bidirectional P2P access between two devices
    pub fn enable_bidirectional(&self, device1: i32, device2: i32) -> HorusResult<()> {
        self.enable_peer_access(device1, device2)?;
        self.enable_peer_access(device2, device1)?;
        Ok(())
    }

    /// Get all P2P access information for all GPU pairs
    pub fn get_p2p_topology() -> HorusResult<Vec<P2PAccessInfo>> {
        let device_count = cuda_ffi::get_device_count().unwrap_or(0) as usize;
        let mut topology = Vec::new();

        let connections: Vec<(i32, i32)> = Vec::new(); // No manager context, assume none enabled

        for d in 0..device_count as i32 {
            for p in 0..device_count as i32 {
                if d != p {
                    let can_access = Self::can_access_peer(d, p).unwrap_or(false);
                    topology.push(P2PAccessInfo {
                        device: d,
                        peer_device: p,
                        can_access,
                        is_enabled: connections.contains(&(d, p)),
                    });
                }
            }
        }

        Ok(topology)
    }

    /// Get enabled P2P connections
    pub fn enabled_connections(&self) -> Vec<(i32, i32)> {
        self.enabled_connections.lock().unwrap().clone()
    }

    /// Copy tensor between GPUs using P2P (requires P2P enabled)
    /// Returns the destination tensor
    pub fn copy_p2p(
        &self,
        src_pool: &CudaTensorPool,
        src_tensor: &CudaTensor,
        dst_pool: &CudaTensorPool,
    ) -> HorusResult<CudaTensor> {
        let src_device = src_pool.device_id();
        let dst_device = dst_pool.device_id();

        // Check P2P is enabled
        {
            let connections = self.enabled_connections.lock().unwrap();
            if src_device != dst_device && !connections.contains(&(dst_device, src_device)) {
                return Err(HorusError::Config(format!(
                    "P2P access not enabled from device {} to device {}",
                    dst_device, src_device
                )));
            }
        }

        // Allocate destination tensor
        let dst_tensor = dst_pool.alloc(
            &src_tensor.shape[..src_tensor.ndim as usize],
            src_tensor.dtype,
        )?;

        // Get pointers
        let src_ptr = src_pool.device_ptr(src_tensor);
        let dst_ptr = dst_pool.device_ptr(&dst_tensor);

        if src_ptr.is_null() || dst_ptr.is_null() {
            return Err(HorusError::Memory("Invalid tensor pointers".into()));
        }

        // Use cudaMemcpyPeer semantics (device to device)
        // cudaMemcpy with cudaMemcpyDeviceToDevice handles P2P if enabled
        cuda_ffi::memcpy(
            dst_ptr,
            src_ptr,
            src_tensor.size as usize,
            cuda_ffi::CudaMemcpyKind::DeviceToDevice,
        )
        .map_err(|e| HorusError::Memory(format!("P2P memcpy failed: {}", e)))?;

        Ok(dst_tensor)
    }

    /// Async P2P copy using CUDA streams
    pub fn copy_p2p_async(
        &self,
        src_pool: &CudaTensorPool,
        src_tensor: &CudaTensor,
        dst_pool: &CudaTensorPool,
        dst_tensor: &CudaTensor,
        stream: cuda_ffi::CudaStream,
    ) -> HorusResult<()> {
        let src_device = src_pool.device_id();
        let dst_device = dst_pool.device_id();

        // Check P2P is enabled
        {
            let connections = self.enabled_connections.lock().unwrap();
            if src_device != dst_device && !connections.contains(&(dst_device, src_device)) {
                return Err(HorusError::Config(format!(
                    "P2P access not enabled from device {} to device {}",
                    dst_device, src_device
                )));
            }
        }

        // Get pointers
        let src_ptr = src_pool.device_ptr(src_tensor);
        let dst_ptr = dst_pool.device_ptr(dst_tensor);

        if src_ptr.is_null() || dst_ptr.is_null() {
            return Err(HorusError::Memory("Invalid tensor pointers".into()));
        }

        // Async P2P copy
        cuda_ffi::memcpy_async(
            dst_ptr,
            src_ptr,
            src_tensor.size as usize,
            cuda_ffi::CudaMemcpyKind::DeviceToDevice,
            stream,
        )
        .map_err(|e| HorusError::Memory(format!("Async P2P memcpy failed: {}", e)))?;

        Ok(())
    }
}

impl Default for P2PManager {
    fn default() -> Self {
        Self::new()
    }
}

impl Drop for P2PManager {
    fn drop(&mut self) {
        // Disable all P2P connections on cleanup
        let connections = self.enabled_connections.lock().unwrap().clone();
        for (device, peer_device) in connections {
            if cuda_ffi::set_device(device).is_ok() {
                let _ = cuda_ffi::device_disable_peer_access(peer_device);
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    // ==========================================================================
    // Structure and Layout Tests
    // ==========================================================================

    #[test]
    fn test_cuda_tensor_size() {
        // Ensure CudaTensor is a reasonable size for IPC
        let size = std::mem::size_of::<CudaTensor>();
        println!("CudaTensor size: {} bytes", size);
        assert!(size < 512, "CudaTensor too large: {} bytes", size);
    }

    #[test]
    fn test_slot_header_alignment() {
        // Ensure slot headers are properly sized
        let size = std::mem::size_of::<CudaSlotHeader>();
        println!("CudaSlotHeader size: {} bytes", size);
        // Should be reasonably sized for cache efficiency
        assert!(size <= 512, "CudaSlotHeader too large: {} bytes", size);
    }

    #[test]
    fn test_pool_header_size() {
        let size = std::mem::size_of::<CudaPoolHeader>();
        println!("CudaPoolHeader size: {} bytes", size);
        // Should fit in a cache line or two
        assert!(size <= 128, "CudaPoolHeader too large: {} bytes", size);
    }

    #[test]
    fn test_p2p_access_info_clone() {
        let info = P2PAccessInfo {
            device: 0,
            peer_device: 1,
            can_access: true,
            is_enabled: false,
        };
        let cloned = info.clone();
        assert_eq!(cloned.device, 0);
        assert_eq!(cloned.peer_device, 1);
        assert!(cloned.can_access);
        assert!(!cloned.is_enabled);
    }

    // ==========================================================================
    // CudaTensor View/Slice API Tests
    // ==========================================================================

    fn create_test_tensor(shape: &[u64], dtype: TensorDtype) -> CudaTensor {
        let numel: u64 = shape.iter().product();
        let size = numel * dtype.element_size() as u64;

        let mut tensor = CudaTensor {
            pool_id: 1,
            slot_id: 0,
            generation: 1,
            device_id: 0,
            size,
            numel,
            offset: 0,
            dtype,
            ndim: shape.len().min(MAX_TENSOR_DIMS) as u8,
            _pad: [0; 6],
            shape: [0; MAX_TENSOR_DIMS],
            strides: [0; MAX_TENSOR_DIMS],
            ipc_handle: [0; CUDA_IPC_HANDLE_SIZE],
        };

        // Set shape (clamped to MAX_TENSOR_DIMS)
        for (i, &dim) in shape.iter().take(MAX_TENSOR_DIMS).enumerate() {
            tensor.shape[i] = dim;
        }

        // Compute contiguous strides
        let mut stride = 1u64;
        for i in (0..shape.len().min(MAX_TENSOR_DIMS)).rev() {
            tensor.strides[i] = stride;
            stride *= shape[i];
        }

        tensor
    }

    #[test]
    fn test_tensor_is_contiguous() {
        // Contiguous tensor
        let tensor = create_test_tensor(&[2, 3, 4], TensorDtype::F32);
        assert!(tensor.is_contiguous());

        // After transpose, should not be contiguous
        let transposed = tensor.transpose(0, 1).unwrap();
        assert!(!transposed.is_contiguous());
    }

    #[test]
    fn test_tensor_view_same_numel() {
        let tensor = create_test_tensor(&[2, 3, 4], TensorDtype::F32);
        assert_eq!(tensor.numel, 24);

        // Reshape to different shape with same numel
        let viewed = tensor.view(&[4, 6]).unwrap();
        assert_eq!(viewed.ndim, 2);
        assert_eq!(viewed.shape[0], 4);
        assert_eq!(viewed.shape[1], 6);
        assert_eq!(viewed.numel, 24);
        assert!(viewed.is_contiguous());
    }

    #[test]
    fn test_tensor_view_different_numel_fails() {
        let tensor = create_test_tensor(&[2, 3, 4], TensorDtype::F32);

        // Different numel should fail
        let result = tensor.view(&[5, 5]);
        assert!(result.is_none());
    }

    #[test]
    fn test_tensor_view_non_contiguous_fails() {
        let tensor = create_test_tensor(&[2, 3, 4], TensorDtype::F32);
        let transposed = tensor.transpose(0, 1).unwrap();

        // Non-contiguous tensor can't be reshaped
        let result = transposed.view(&[24]);
        assert!(result.is_none());
    }

    #[test]
    fn test_tensor_slice_first_dim() {
        let tensor = create_test_tensor(&[10, 5, 3], TensorDtype::F32);

        // Slice first 3 elements
        let sliced = tensor.slice_first_dim(0, 3).unwrap();
        assert_eq!(sliced.shape[0], 3);
        assert_eq!(sliced.shape[1], 5);
        assert_eq!(sliced.shape[2], 3);
        assert_eq!(sliced.numel, 45); // 3 * 5 * 3
        assert_eq!(sliced.offset, 0);

        // Slice middle
        let sliced2 = tensor.slice_first_dim(2, 7).unwrap();
        assert_eq!(sliced2.shape[0], 5);
        assert_eq!(sliced2.numel, 75); // 5 * 5 * 3
                                       // Offset should be 2 * stride[0] * element_size
        let expected_offset = 2 * tensor.strides[0] * tensor.dtype.element_size() as u64;
        assert_eq!(sliced2.offset, expected_offset);
    }

    #[test]
    fn test_tensor_slice_first_dim_invalid() {
        let tensor = create_test_tensor(&[10, 5], TensorDtype::F32);

        // Invalid ranges
        assert!(tensor.slice_first_dim(5, 3).is_none()); // start >= end
        assert!(tensor.slice_first_dim(0, 11).is_none()); // end > dim
        assert!(tensor.slice_first_dim(10, 11).is_none()); // start >= dim
    }

    #[test]
    fn test_tensor_slice_any_dim() {
        let tensor = create_test_tensor(&[4, 6, 8], TensorDtype::F32);

        // Slice along dim 1
        let sliced = tensor.slice_dim(1, 2, 5).unwrap();
        assert_eq!(sliced.shape[0], 4);
        assert_eq!(sliced.shape[1], 3); // 5 - 2 = 3
        assert_eq!(sliced.shape[2], 8);
        assert_eq!(sliced.numel, 96); // 4 * 3 * 8

        // Slice along last dim
        let sliced2 = tensor.slice_dim(2, 0, 4).unwrap();
        assert_eq!(sliced2.shape[2], 4);
        assert_eq!(sliced2.numel, 96); // 4 * 6 * 4
    }

    #[test]
    fn test_tensor_transpose() {
        let tensor = create_test_tensor(&[2, 3, 4], TensorDtype::F32);

        let transposed = tensor.transpose(0, 2).unwrap();
        assert_eq!(transposed.shape[0], 4);
        assert_eq!(transposed.shape[1], 3);
        assert_eq!(transposed.shape[2], 2);
        assert_eq!(transposed.strides[0], tensor.strides[2]);
        assert_eq!(transposed.strides[2], tensor.strides[0]);

        // Same dim transpose returns copy
        let same = tensor.transpose(1, 1).unwrap();
        assert_eq!(same.shape[0], tensor.shape[0]);
    }

    #[test]
    fn test_tensor_transpose_invalid() {
        let tensor = create_test_tensor(&[2, 3], TensorDtype::F32);

        // Invalid dimension
        assert!(tensor.transpose(0, 3).is_none());
        assert!(tensor.transpose(5, 0).is_none());
    }

    #[test]
    fn test_tensor_squeeze() {
        let tensor = create_test_tensor(&[1, 3, 1, 4], TensorDtype::F32);

        // Squeeze first dimension
        let squeezed = tensor.squeeze(0).unwrap();
        assert_eq!(squeezed.ndim, 3);
        assert_eq!(squeezed.shape[0], 3);
        assert_eq!(squeezed.shape[1], 1);
        assert_eq!(squeezed.shape[2], 4);

        // Squeeze middle dimension
        let squeezed2 = tensor.squeeze(2).unwrap();
        assert_eq!(squeezed2.ndim, 3);
        assert_eq!(squeezed2.shape[0], 1);
        assert_eq!(squeezed2.shape[1], 3);
        assert_eq!(squeezed2.shape[2], 4);
    }

    #[test]
    fn test_tensor_squeeze_invalid() {
        let tensor = create_test_tensor(&[2, 3, 4], TensorDtype::F32);

        // Can't squeeze non-1 dimension
        assert!(tensor.squeeze(0).is_none());
        assert!(tensor.squeeze(1).is_none());

        // Can't squeeze scalar
        let scalar = create_test_tensor(&[1], TensorDtype::F32);
        assert!(scalar.squeeze(0).is_none());
    }

    #[test]
    fn test_tensor_unsqueeze() {
        let tensor = create_test_tensor(&[3, 4], TensorDtype::F32);

        // Unsqueeze at beginning
        let unsqueezed = tensor.unsqueeze(0).unwrap();
        assert_eq!(unsqueezed.ndim, 3);
        assert_eq!(unsqueezed.shape[0], 1);
        assert_eq!(unsqueezed.shape[1], 3);
        assert_eq!(unsqueezed.shape[2], 4);

        // Unsqueeze at end
        let unsqueezed2 = tensor.unsqueeze(2).unwrap();
        assert_eq!(unsqueezed2.ndim, 3);
        assert_eq!(unsqueezed2.shape[0], 3);
        assert_eq!(unsqueezed2.shape[1], 4);
        assert_eq!(unsqueezed2.shape[2], 1);

        // Unsqueeze in middle
        let unsqueezed3 = tensor.unsqueeze(1).unwrap();
        assert_eq!(unsqueezed3.shape[0], 3);
        assert_eq!(unsqueezed3.shape[1], 1);
        assert_eq!(unsqueezed3.shape[2], 4);
    }

    #[test]
    fn test_tensor_unsqueeze_invalid() {
        let tensor = create_test_tensor(&[2, 3], TensorDtype::F32);

        // Invalid dimension (> ndim)
        assert!(tensor.unsqueeze(5).is_none());

        // Can't unsqueeze beyond MAX_TENSOR_DIMS
        let mut big_tensor = tensor;
        big_tensor.ndim = MAX_TENSOR_DIMS as u8;
        assert!(big_tensor.unsqueeze(0).is_none());
    }

    #[test]
    fn test_tensor_effective_offset() {
        let mut tensor = create_test_tensor(&[10, 5], TensorDtype::F32);
        assert_eq!(tensor.effective_offset(), 0);

        tensor.offset = 100;
        assert_eq!(tensor.effective_offset(), 100);
    }

    #[test]
    fn test_tensor_device() {
        let mut tensor = create_test_tensor(&[2, 3], TensorDtype::F32);

        tensor.device_id = 0;
        assert_eq!(tensor.device(), Device::cuda(0));

        tensor.device_id = 1;
        assert_eq!(tensor.device(), Device::cuda(1));

        tensor.device_id = 2;
        assert_eq!(tensor.device(), Device::cuda(2));

        tensor.device_id = 3;
        assert_eq!(tensor.device(), Device::cuda(3));

        tensor.device_id = 99;
        assert_eq!(tensor.device(), Device::cuda(99)); // No longer limited to 4 GPUs
    }

    #[test]
    fn test_tensor_ipc_handle_bytes() {
        let mut tensor = create_test_tensor(&[2, 3], TensorDtype::F32);
        tensor.ipc_handle[0] = 0xAB;
        tensor.ipc_handle[63] = 0xCD;

        let bytes = tensor.ipc_handle_bytes();
        assert_eq!(bytes.len(), CUDA_IPC_HANDLE_SIZE);
        assert_eq!(bytes[0], 0xAB);
        assert_eq!(bytes[63], 0xCD);
    }

    // ==========================================================================
    // Chained Operations Tests
    // ==========================================================================

    #[test]
    fn test_tensor_chained_operations() {
        let tensor = create_test_tensor(&[2, 4, 6, 8], TensorDtype::F32);

        // Chain: slice -> transpose -> squeeze
        let result = tensor
            .slice_dim(0, 0, 1) // [1, 4, 6, 8]
            .and_then(|t| t.squeeze(0)) // [4, 6, 8]
            .and_then(|t| t.transpose(0, 2)); // [8, 6, 4]

        let result = result.unwrap();
        assert_eq!(result.ndim, 3);
        assert_eq!(result.shape[0], 8);
        assert_eq!(result.shape[1], 6);
        assert_eq!(result.shape[2], 4);
    }

    #[test]
    fn test_tensor_view_to_1d() {
        let tensor = create_test_tensor(&[2, 3, 4], TensorDtype::F32);

        // Flatten to 1D
        let flat = tensor.view(&[24]).unwrap();
        assert_eq!(flat.ndim, 1);
        assert_eq!(flat.shape[0], 24);
        assert!(flat.is_contiguous());
    }

    // ==========================================================================
    // P2PManager Tests
    // ==========================================================================

    #[test]
    fn test_p2p_manager_creation() {
        let manager = P2PManager::new();
        assert!(manager.enabled_connections().is_empty());
    }

    #[test]
    fn test_p2p_manager_default() {
        let manager = P2PManager::default();
        assert!(manager.enabled_connections().is_empty());
    }

    // ==========================================================================
    // Data Type Tests
    // ==========================================================================

    #[test]
    fn test_tensor_different_dtypes() {
        let f32_tensor = create_test_tensor(&[10, 10], TensorDtype::F32);
        assert_eq!(f32_tensor.size, 400); // 100 * 4 bytes

        let f64_tensor = create_test_tensor(&[10, 10], TensorDtype::F64);
        assert_eq!(f64_tensor.size, 800); // 100 * 8 bytes

        let i32_tensor = create_test_tensor(&[10, 10], TensorDtype::I32);
        assert_eq!(i32_tensor.size, 400); // 100 * 4 bytes

        let u8_tensor = create_test_tensor(&[10, 10], TensorDtype::U8);
        assert_eq!(u8_tensor.size, 100); // 100 * 1 byte
    }

    #[test]
    fn test_tensor_slice_preserves_dtype() {
        let tensor = create_test_tensor(&[10, 5], TensorDtype::F64);
        let sliced = tensor.slice_first_dim(0, 5).unwrap();
        assert!(matches!(sliced.dtype, TensorDtype::F64));
    }

    // ==========================================================================
    // Edge Cases
    // ==========================================================================

    #[test]
    fn test_tensor_1d_operations() {
        let tensor = create_test_tensor(&[100], TensorDtype::F32);

        // Slice
        let sliced = tensor.slice_first_dim(10, 50).unwrap();
        assert_eq!(sliced.shape[0], 40);
        assert_eq!(sliced.numel, 40);

        // Can't squeeze 1D tensor
        assert!(tensor.squeeze(0).is_none());

        // Unsqueeze works
        let unsqueezed = tensor.unsqueeze(0).unwrap();
        assert_eq!(unsqueezed.ndim, 2);
        assert_eq!(unsqueezed.shape[0], 1);
        assert_eq!(unsqueezed.shape[1], 100);
    }

    #[test]
    fn test_tensor_scalar_like() {
        // Single element tensor
        let tensor = create_test_tensor(&[1, 1, 1], TensorDtype::F32);
        assert_eq!(tensor.numel, 1);
        assert!(tensor.is_contiguous());

        // Can squeeze to 2D
        let squeezed = tensor.squeeze(0).unwrap();
        assert_eq!(squeezed.ndim, 2);
    }

    #[test]
    fn test_tensor_large_shape() {
        // Large tensor
        let tensor = create_test_tensor(&[1000, 1000, 10], TensorDtype::F32);
        assert_eq!(tensor.numel, 10_000_000);
        assert_eq!(tensor.size, 40_000_000); // 10M * 4 bytes

        // Slice should still work
        let sliced = tensor.slice_first_dim(500, 600).unwrap();
        assert_eq!(sliced.numel, 1_000_000); // 100 * 1000 * 10
    }

    #[test]
    fn test_tensor_strides_calculation() {
        let tensor = create_test_tensor(&[2, 3, 4], TensorDtype::F32);

        // Row-major strides: [12, 4, 1]
        assert_eq!(tensor.strides[0], 12); // 3 * 4
        assert_eq!(tensor.strides[1], 4); // 4
        assert_eq!(tensor.strides[2], 1); // 1
    }

    #[test]
    fn test_invalid_slot_constant() {
        assert_eq!(INVALID_SLOT, u32::MAX);
    }

    // ==========================================================================
    // Pool Configuration Tests
    // ==========================================================================

    #[test]
    fn test_pool_config_default() {
        let config = CudaTensorPoolConfig::default();
        assert_eq!(config.max_slots, MAX_CUDA_SLOTS);
    }

    #[test]
    fn test_pool_stats_structure() {
        let stats = CudaPoolStats {
            pool_id: 42,
            device_id: 0,
            max_slots: 256,
            allocated_slots: 10,
        };

        assert_eq!(stats.pool_id, 42);
        assert_eq!(stats.device_id, 0);
        assert_eq!(stats.max_slots, 256);
        assert_eq!(stats.allocated_slots, 10);
    }
}
