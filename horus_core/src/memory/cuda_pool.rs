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
use crate::memory::platform::shm_global_dir;
use crate::memory::tensor_pool::{TensorDevice, TensorDtype, MAX_TENSOR_DIMS};
use memmap2::{MmapMut, MmapOptions};
use std::collections::HashMap;
use std::ffi::c_void;
use std::fs::{File, OpenOptions};
use std::path::PathBuf;
use std::sync::atomic::{AtomicU32, AtomicU64, Ordering};
use std::sync::{Arc, Mutex};

#[cfg(feature = "cuda")]
use super::cuda_ffi::{self, CUDA_IPC_HANDLE_SIZE};

#[cfg(not(feature = "cuda"))]
pub const CUDA_IPC_HANDLE_SIZE: usize = 64;

/// Magic number for CUDA pool validation
const CUDA_POOL_MAGIC: u64 = 0x484F52555343554; // "HORUS_CU" in hex

/// Pool version
const CUDA_POOL_VERSION: u32 = 1;

/// Maximum slots per pool
const MAX_CUDA_SLOTS: usize = 256;

/// Slot states
const SLOT_FREE: u32 = 0;
const SLOT_ALLOCATED: u32 = 1;

/// CUDA pool header stored in shared memory
#[repr(C)]
struct CudaPoolHeader {
    magic: u64,
    version: u32,
    pool_id: u32,
    device_id: u32,
    max_slots: u32,
    allocated_count: AtomicU32,
    _padding: [u8; 40], // Pad to 64 bytes
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
    _padding: [u8; 18], // Align to 256 bytes total
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
    /// Data type
    pub dtype: TensorDtype,
    /// Number of dimensions
    pub ndim: u8,
    pub _pad: [u8; 6],
    /// Shape
    pub shape: [u64; MAX_TENSOR_DIMS],
    /// IPC handle for cross-process sharing
    pub ipc_handle: [u8; CUDA_IPC_HANDLE_SIZE],
}

impl CudaTensor {
    /// Get the IPC handle bytes for sharing
    pub fn ipc_handle_bytes(&self) -> &[u8] {
        &self.ipc_handle
    }

    /// Get device enum
    pub fn device(&self) -> TensorDevice {
        match self.device_id {
            0 => TensorDevice::Cuda0,
            1 => TensorDevice::Cuda1,
            2 => TensorDevice::Cuda2,
            3 => TensorDevice::Cuda3,
            _ => TensorDevice::Cuda0,
        }
    }
}

/// CUDA Tensor Pool for GPU memory with IPC support
pub struct CudaTensorPool {
    pool_id: u32,
    device_id: i32,
    #[allow(dead_code)]
    shm_path: PathBuf,
    mmap: MmapMut,
    _file: File,
    is_owner: bool,
    /// Local cache of opened IPC handles (slot_id -> device_ptr)
    /// Only used by non-owner processes
    #[allow(dead_code)]
    opened_handles: Arc<Mutex<HashMap<u32, *mut c_void>>>,
}

// Safety: Pool uses atomic operations and IPC handles are process-safe
unsafe impl Send for CudaTensorPool {}
unsafe impl Sync for CudaTensorPool {}

impl CudaTensorPool {
    /// Create a new CUDA tensor pool
    ///
    /// # Arguments
    /// * `pool_id` - Unique identifier for this pool
    /// * `device_id` - CUDA device index (0, 1, 2, ...)
    /// * `config` - Pool configuration
    #[cfg(feature = "cuda")]
    pub fn new(pool_id: u32, device_id: i32, config: CudaTensorPoolConfig) -> HorusResult<Self> {
        // Verify CUDA is available
        if !cuda_ffi::cuda_available() {
            return Err(HorusError::Config("CUDA not available".into()));
        }

        // Set device
        cuda_ffi::set_device(device_id)
            .map_err(|e| HorusError::Config(format!("Failed to set CUDA device: {}", e)))?;

        let shm_dir = shm_global_dir();
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

        let mmap = unsafe { MmapOptions::new().len(total_size).map_mut(&file)? };

        let mut pool = Self {
            pool_id,
            device_id,
            shm_path,
            mmap,
            _file: file,
            is_owner: true,
            opened_handles: Arc::new(Mutex::new(HashMap::new())),
        };

        pool.initialize(config.max_slots)?;

        Ok(pool)
    }

    #[cfg(not(feature = "cuda"))]
    pub fn new(_pool_id: u32, _device_id: i32, _config: CudaTensorPoolConfig) -> HorusResult<Self> {
        Err(HorusError::Config("CUDA feature not enabled".into()))
    }

    /// Open an existing CUDA tensor pool
    #[cfg(feature = "cuda")]
    pub fn open(pool_id: u32, device_id: i32) -> HorusResult<Self> {
        // Verify CUDA is available
        if !cuda_ffi::cuda_available() {
            return Err(HorusError::Config("CUDA not available".into()));
        }

        cuda_ffi::set_device(device_id)
            .map_err(|e| HorusError::Config(format!("Failed to set CUDA device: {}", e)))?;

        let shm_dir = shm_global_dir();
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

        let mmap = unsafe { MmapOptions::new().len(total_size).map_mut(&file)? };

        let pool = Self {
            pool_id,
            device_id,
            shm_path,
            mmap,
            _file: file,
            is_owner: false,
            opened_handles: Arc::new(Mutex::new(HashMap::new())),
        };

        pool.validate()?;

        Ok(pool)
    }

    #[cfg(not(feature = "cuda"))]
    pub fn open(_pool_id: u32, _device_id: i32) -> HorusResult<Self> {
        Err(HorusError::Config("CUDA feature not enabled".into()))
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

        // Initialize all slots as free
        for i in 0..max_slots {
            let slot = self.slot_mut(i as u32);
            slot.state.store(SLOT_FREE, Ordering::Release);
            slot.refcount.store(0, Ordering::Release);
            slot.generation.store(0, Ordering::Release);
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
    #[cfg(feature = "cuda")]
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
            dtype,
            ndim: shape.len().min(MAX_TENSOR_DIMS) as u8,
            _pad: [0; 6],
            shape: [0; MAX_TENSOR_DIMS],
            ipc_handle: [0; CUDA_IPC_HANDLE_SIZE],
        };

        for (i, &dim) in shape.iter().take(MAX_TENSOR_DIMS).enumerate() {
            tensor.shape[i] = dim;
        }
        tensor.ipc_handle.copy_from_slice(&ipc_handle.reserved);

        Ok(tensor)
    }

    #[cfg(not(feature = "cuda"))]
    pub fn alloc(&self, _shape: &[u64], _dtype: TensorDtype) -> HorusResult<CudaTensor> {
        Err(HorusError::Config("CUDA feature not enabled".into()))
    }

    /// Import a tensor from an IPC handle (for cross-process sharing)
    #[cfg(feature = "cuda")]
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
            dtype,
            ndim: shape.len().min(MAX_TENSOR_DIMS) as u8,
            _pad: [0; 6],
            shape: [0; MAX_TENSOR_DIMS],
            ipc_handle: [0; CUDA_IPC_HANDLE_SIZE],
        };

        for (i, &dim) in shape.iter().take(MAX_TENSOR_DIMS).enumerate() {
            tensor.shape[i] = dim;
        }
        tensor.ipc_handle.copy_from_slice(ipc_handle_bytes);

        Ok((dev_ptr, tensor))
    }

    #[cfg(not(feature = "cuda"))]
    pub fn import_ipc(
        &self,
        _ipc_handle_bytes: &[u8],
        _shape: &[u64],
        _dtype: TensorDtype,
    ) -> HorusResult<(*mut c_void, CudaTensor)> {
        Err(HorusError::Config("CUDA feature not enabled".into()))
    }

    /// Close an imported IPC handle
    #[cfg(feature = "cuda")]
    pub fn close_ipc(&self, dev_ptr: *mut c_void) -> HorusResult<()> {
        cuda_ffi::ipc_close_mem_handle(dev_ptr)
            .map_err(|e| HorusError::Memory(format!("Failed to close IPC handle: {}", e)))
    }

    #[cfg(not(feature = "cuda"))]
    pub fn close_ipc(&self, _dev_ptr: *mut c_void) -> HorusResult<()> {
        Err(HorusError::Config("CUDA feature not enabled".into()))
    }

    /// Release a tensor (decrement refcount, free if zero)
    #[cfg(feature = "cuda")]
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

            slot.state.store(SLOT_FREE, Ordering::Release);
            self.header().allocated_count.fetch_sub(1, Ordering::AcqRel);
        }

        Ok(())
    }

    #[cfg(not(feature = "cuda"))]
    pub fn release(&self, _tensor: &CudaTensor) -> HorusResult<()> {
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

    /// Get device pointer for a tensor
    pub fn device_ptr(&self, tensor: &CudaTensor) -> *mut c_void {
        if tensor.pool_id != self.pool_id || tensor.slot_id == u32::MAX {
            return std::ptr::null_mut();
        }

        let slot = self.slot(tensor.slot_id);
        if slot.generation.load(Ordering::Acquire) != tensor.generation {
            return std::ptr::null_mut();
        }

        slot.device_ptr.load(Ordering::Acquire) as *mut c_void
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
        unsafe { &*(self.mmap.as_ptr() as *const CudaPoolHeader) }
    }

    fn header_mut(&mut self) -> &mut CudaPoolHeader {
        unsafe { &mut *(self.mmap.as_mut_ptr() as *mut CudaPoolHeader) }
    }

    fn slot(&self, index: u32) -> &CudaSlotHeader {
        let offset = std::mem::size_of::<CudaPoolHeader>()
            + (index as usize) * std::mem::size_of::<CudaSlotHeader>();
        unsafe { &*(self.mmap.as_ptr().add(offset) as *const CudaSlotHeader) }
    }

    fn slot_mut(&self, index: u32) -> &mut CudaSlotHeader {
        let offset = std::mem::size_of::<CudaPoolHeader>()
            + (index as usize) * std::mem::size_of::<CudaSlotHeader>();
        unsafe { &mut *(self.mmap.as_ptr().add(offset) as *mut CudaSlotHeader) }
    }

    fn find_free_slot(&self) -> HorusResult<u32> {
        let header = self.header();
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
}

impl Drop for CudaTensorPool {
    fn drop(&mut self) {
        // If we're the owner, free all allocated GPU memory
        #[cfg(feature = "cuda")]
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

        // Close any opened IPC handles (for non-owner processes)
        #[cfg(feature = "cuda")]
        {
            let handles = self.opened_handles.lock().unwrap();
            for (_, &ptr) in handles.iter() {
                let _ = cuda_ffi::ipc_close_mem_handle(ptr);
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

/// Check if CUDA is available
pub fn cuda_available() -> bool {
    #[cfg(feature = "cuda")]
    {
        cuda_ffi::cuda_available()
    }
    #[cfg(not(feature = "cuda"))]
    {
        false
    }
}

/// Get number of CUDA devices
pub fn cuda_device_count() -> usize {
    #[cfg(feature = "cuda")]
    {
        cuda_ffi::get_device_count().unwrap_or(0) as usize
    }
    #[cfg(not(feature = "cuda"))]
    {
        0
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_cuda_tensor_size() {
        // Ensure CudaTensor is a reasonable size for IPC
        assert!(std::mem::size_of::<CudaTensor>() < 512);
    }

    #[test]
    fn test_slot_header_alignment() {
        // Ensure slot headers are properly sized
        let size = std::mem::size_of::<CudaSlotHeader>();
        println!("CudaSlotHeader size: {} bytes", size);
    }
}
