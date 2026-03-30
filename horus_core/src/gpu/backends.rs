//! CUDA memory backends implementing [`PoolBackend`].
//!
//! Three backends for different GPU memory types:
//!
//! - [`CudaManagedBackend`] — `cuMemAllocManaged` (Jetson unified memory)
//! - [`CudaPinnedBackend`] — `cuMemAllocHost` (discrete GPU, DMA-capable)
//! - [`CudaDeviceBackend`] — `cuMemAlloc` (GPU-only, highest bandwidth)
//!
//! All use the CUDA Driver API via dlopen (no link-time CUDA dependency).
//! Each backend creates a CUDA context on construction that lives for the
//! pool's lifetime.

use super::cuda_ffi::{self, CudaContext, CUdeviceptr};
use crate::memory::backend::{BackendAllocation, PoolBackend};
use crate::types::Device;
use std::sync::Mutex;

// ---------------------------------------------------------------------------
// CudaManagedBackend — unified memory (Jetson optimal)
// ---------------------------------------------------------------------------

/// CUDA managed (unified) memory backend.
///
/// Uses `cuMemAllocManaged` which returns memory accessible from both
/// CPU and GPU. On Jetson (iGPU), this is true zero-copy — the CPU
/// and GPU access the same physical memory. On discrete GPUs, the
/// driver migrates pages on demand.
///
/// # Cross-Process
///
/// NOT cross-process shareable. Each process gets its own managed
/// allocation. Cross-process tensor sharing requires copying through
/// mmap or using `horus_net` for network replication.
pub struct CudaManagedBackend {
    device_id: u32,
    _ctx: CudaContext,
    /// Track all live allocations so we can free them. Mutex because
    /// alloc/free can be called from any thread (pool is Send+Sync).
    allocations: Mutex<Vec<CUdeviceptr>>,
}

impl std::fmt::Debug for CudaManagedBackend {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("CudaManagedBackend")
            .field("device_id", &self.device_id)
            .finish()
    }
}

// SAFETY: CudaManagedBackend's mutable state is protected by Mutex.
// CudaContext is Send+Sync (CUDA serializes internally).
// CUdeviceptr values are u64 handles, inherently thread-safe.
unsafe impl Send for CudaManagedBackend {}
unsafe impl Sync for CudaManagedBackend {}

impl CudaManagedBackend {
    /// Create a new managed memory backend on the given CUDA device.
    pub fn new(device_id: u32) -> Result<Self, String> {
        cuda_ffi::init().map_err(|e| e.to_string())?;
        let dev = cuda_ffi::device_get(device_id as i32).map_err(|e| e.to_string())?;
        let ctx = CudaContext::new(dev).map_err(|e| e.to_string())?;
        ctx.set_current().map_err(|e| e.to_string())?;
        Ok(Self {
            device_id,
            _ctx: ctx,
            allocations: Mutex::new(Vec::new()),
        })
    }
}

impl PoolBackend for CudaManagedBackend {
    fn alloc(&self, size: usize) -> Result<BackendAllocation, String> {
        self._ctx.set_current().map_err(|e| e.to_string())?;
        let dptr = cuda_ffi::mem_alloc_managed(size).map_err(|e| e.to_string())?;
        let ptr = dptr as *mut u8;

        self.allocations
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .push(dptr);

        Ok(BackendAllocation {
            cpu_ptr: ptr,     // Managed memory is CPU-accessible
            device_ptr: ptr,  // Same pointer works on GPU
            size,
        })
    }

    fn free(&self, alloc: &BackendAllocation) {
        if alloc.cpu_ptr.is_null() {
            return;
        }
        let dptr = alloc.cpu_ptr as CUdeviceptr;
        let _ = self._ctx.set_current();
        let _ = cuda_ffi::mem_free(dptr);

        // Remove from tracking
        let mut allocs = self.allocations.lock().unwrap_or_else(|e| e.into_inner());
        allocs.retain(|&a| a != dptr);
    }

    fn device(&self) -> Device {
        Device::cuda(self.device_id)
    }

    fn is_shared(&self) -> bool {
        false // Managed memory is process-local
    }

    fn name(&self) -> &str {
        "cuda_managed"
    }

    fn zero(&self, alloc: &BackendAllocation) {
        if alloc.cpu_ptr.is_null() || alloc.size == 0 {
            return;
        }
        // For managed memory, we can use either CPU writes or cuMemsetD8.
        // cuMemsetD8 is faster for large allocations and offloads to GPU.
        let dptr = alloc.cpu_ptr as CUdeviceptr;
        let _ = self._ctx.set_current();
        if cuda_ffi::memset_d8(dptr, 0, alloc.size).is_err() {
            // Fallback to CPU volatile zero if GPU memset fails
            for i in 0..alloc.size {
                unsafe { alloc.cpu_ptr.add(i).write_volatile(0u8) };
            }
            core::sync::atomic::compiler_fence(core::sync::atomic::Ordering::SeqCst);
        }
    }
}

impl Drop for CudaManagedBackend {
    fn drop(&mut self) {
        let _ = self._ctx.set_current();
        let allocs = self.allocations.lock().unwrap_or_else(|e| e.into_inner());
        for &dptr in allocs.iter() {
            let _ = cuda_ffi::mem_free(dptr);
        }
    }
}

// ---------------------------------------------------------------------------
// CudaPinnedBackend — page-locked host memory (discrete GPU optimal)
// ---------------------------------------------------------------------------

/// CUDA pinned (page-locked) host memory backend.
///
/// Uses `cuMemAllocHost` which allocates CPU memory that is page-locked,
/// enabling fast DMA transfers to/from GPU via `cuMemcpyHtoD`/`DtoH`.
///
/// Best for discrete GPUs where data is primarily accessed on CPU but
/// needs fast GPU transfers for preprocessing/inference.
pub struct CudaPinnedBackend {
    device_id: u32,
    _ctx: CudaContext,
    allocations: Mutex<Vec<*mut u8>>,
}

impl std::fmt::Debug for CudaPinnedBackend {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("CudaPinnedBackend")
            .field("device_id", &self.device_id)
            .finish()
    }
}

// SAFETY: Pinned host memory is CPU memory (page-locked). The pointers
// are stable and thread-safe (same as regular heap allocations).
unsafe impl Send for CudaPinnedBackend {}
unsafe impl Sync for CudaPinnedBackend {}

impl CudaPinnedBackend {
    /// Create a new pinned memory backend on the given CUDA device.
    pub fn new(device_id: u32) -> Result<Self, String> {
        cuda_ffi::init().map_err(|e| e.to_string())?;
        let dev = cuda_ffi::device_get(device_id as i32).map_err(|e| e.to_string())?;
        let ctx = CudaContext::new(dev).map_err(|e| e.to_string())?;
        ctx.set_current().map_err(|e| e.to_string())?;
        Ok(Self {
            device_id,
            _ctx: ctx,
            allocations: Mutex::new(Vec::new()),
        })
    }
}

impl PoolBackend for CudaPinnedBackend {
    fn alloc(&self, size: usize) -> Result<BackendAllocation, String> {
        self._ctx.set_current().map_err(|e| e.to_string())?;
        let ptr = cuda_ffi::mem_alloc_host(size).map_err(|e| e.to_string())?;

        self.allocations
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .push(ptr);

        Ok(BackendAllocation {
            cpu_ptr: ptr,                       // Pinned host memory is CPU-accessible
            device_ptr: std::ptr::null_mut(),    // Not directly GPU-accessible (needs memcpy)
            size,
        })
    }

    fn free(&self, alloc: &BackendAllocation) {
        if alloc.cpu_ptr.is_null() {
            return;
        }
        let _ = self._ctx.set_current();
        let _ = cuda_ffi::mem_free_host(alloc.cpu_ptr);

        let mut allocs = self.allocations.lock().unwrap_or_else(|e| e.into_inner());
        allocs.retain(|&a| a != alloc.cpu_ptr);
    }

    fn device(&self) -> Device {
        // Pinned memory lives on CPU but is optimized for GPU transfers.
        // The device is CPU because `data_slice()` returns a valid CPU pointer.
        Device::cpu()
    }

    fn is_shared(&self) -> bool {
        false // Pinned memory is process-local
    }

    fn name(&self) -> &str {
        "cuda_pinned"
    }

    // Uses default zero() — volatile CPU writes (pinned memory is CPU-accessible)
}

impl Drop for CudaPinnedBackend {
    fn drop(&mut self) {
        let _ = self._ctx.set_current();
        let allocs = self.allocations.lock().unwrap_or_else(|e| e.into_inner());
        for &ptr in allocs.iter() {
            let _ = cuda_ffi::mem_free_host(ptr);
        }
    }
}

// ---------------------------------------------------------------------------
// CudaDeviceBackend — GPU-only memory (highest bandwidth)
// ---------------------------------------------------------------------------

/// CUDA device-only memory backend.
///
/// Uses `cuMemAlloc` which allocates memory on the GPU. NOT CPU-accessible.
/// `cpu_ptr()` returns null — callers must use `device_ptr()` and explicit
/// `cuMemcpy` for CPU access.
///
/// Best for pure GPU pipelines (preprocess → inference → postprocess)
/// where data never touches the CPU.
pub struct CudaDeviceBackend {
    device_id: u32,
    _ctx: CudaContext,
    allocations: Mutex<Vec<CUdeviceptr>>,
}

impl std::fmt::Debug for CudaDeviceBackend {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("CudaDeviceBackend")
            .field("device_id", &self.device_id)
            .finish()
    }
}

// SAFETY: Device memory pointers are opaque handles (CUdeviceptr = u64).
// Thread-safe because CUDA serializes all device operations per context.
unsafe impl Send for CudaDeviceBackend {}
unsafe impl Sync for CudaDeviceBackend {}

impl CudaDeviceBackend {
    /// Create a new device memory backend on the given CUDA device.
    pub fn new(device_id: u32) -> Result<Self, String> {
        cuda_ffi::init().map_err(|e| e.to_string())?;
        let dev = cuda_ffi::device_get(device_id as i32).map_err(|e| e.to_string())?;
        let ctx = CudaContext::new(dev).map_err(|e| e.to_string())?;
        ctx.set_current().map_err(|e| e.to_string())?;
        Ok(Self {
            device_id,
            _ctx: ctx,
            allocations: Mutex::new(Vec::new()),
        })
    }
}

impl PoolBackend for CudaDeviceBackend {
    fn alloc(&self, size: usize) -> Result<BackendAllocation, String> {
        self._ctx.set_current().map_err(|e| e.to_string())?;
        let dptr = cuda_ffi::mem_alloc(size).map_err(|e| e.to_string())?;

        self.allocations
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .push(dptr);

        Ok(BackendAllocation {
            cpu_ptr: std::ptr::null_mut(),   // NOT CPU-accessible
            device_ptr: dptr as *mut u8,     // GPU device pointer
            size,
        })
    }

    fn free(&self, alloc: &BackendAllocation) {
        if alloc.device_ptr.is_null() {
            return;
        }
        let dptr = alloc.device_ptr as CUdeviceptr;
        let _ = self._ctx.set_current();
        let _ = cuda_ffi::mem_free(dptr);

        let mut allocs = self.allocations.lock().unwrap_or_else(|e| e.into_inner());
        allocs.retain(|&a| a != dptr);
    }

    fn device(&self) -> Device {
        Device::cuda(self.device_id)
    }

    fn is_shared(&self) -> bool {
        false
    }

    fn name(&self) -> &str {
        "cuda_device"
    }

    fn zero(&self, alloc: &BackendAllocation) {
        if alloc.device_ptr.is_null() || alloc.size == 0 {
            return;
        }
        let dptr = alloc.device_ptr as CUdeviceptr;
        let _ = self._ctx.set_current();
        let _ = cuda_ffi::memset_d8(dptr, 0, alloc.size);
    }
}

impl Drop for CudaDeviceBackend {
    fn drop(&mut self) {
        let _ = self._ctx.set_current();
        let allocs = self.allocations.lock().unwrap_or_else(|e| e.into_inner());
        for &dptr in allocs.iter() {
            let _ = cuda_ffi::mem_free(dptr);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_cuda_managed_backend_alloc_free() {
        if !cuda_ffi::is_available() {
            eprintln!("CUDA not available, skipping");
            return;
        }
        let backend = CudaManagedBackend::new(0).unwrap();
        assert_eq!(backend.device(), Device::cuda(0));
        assert!(!backend.is_shared());
        assert_eq!(backend.name(), "cuda_managed");

        let alloc = backend.alloc(4096).unwrap();
        assert!(!alloc.cpu_ptr.is_null());
        assert!(!alloc.device_ptr.is_null());
        assert_eq!(alloc.cpu_ptr, alloc.device_ptr); // Managed: same pointer
        assert_eq!(alloc.size, 4096);

        // CPU write + read (managed memory)
        unsafe {
            std::ptr::write_bytes(alloc.cpu_ptr, 0xAB, 4096);
            assert_eq!(*alloc.cpu_ptr, 0xAB);
        }

        backend.zero(&alloc);
        unsafe { assert_eq!(*alloc.cpu_ptr, 0x00) };

        backend.free(&alloc);
    }

    #[test]
    fn test_cuda_pinned_backend_alloc_free() {
        if !cuda_ffi::is_available() {
            eprintln!("CUDA not available, skipping");
            return;
        }
        let backend = CudaPinnedBackend::new(0).unwrap();
        assert_eq!(backend.device(), Device::cpu()); // Pinned is CPU-accessible
        assert!(!backend.is_shared());
        assert_eq!(backend.name(), "cuda_pinned");

        let alloc = backend.alloc(4096).unwrap();
        assert!(!alloc.cpu_ptr.is_null());
        assert!(alloc.device_ptr.is_null()); // No device pointer for pinned
        assert_eq!(alloc.size, 4096);

        // CPU write + read
        unsafe {
            std::ptr::write_bytes(alloc.cpu_ptr, 0xCD, 4096);
            assert_eq!(*alloc.cpu_ptr, 0xCD);
        }

        backend.zero(&alloc);
        unsafe { assert_eq!(*alloc.cpu_ptr, 0x00) };

        backend.free(&alloc);
    }

    #[test]
    fn test_cuda_device_backend_alloc_free() {
        if !cuda_ffi::is_available() {
            eprintln!("CUDA not available, skipping");
            return;
        }
        let backend = CudaDeviceBackend::new(0).unwrap();
        assert_eq!(backend.device(), Device::cuda(0));
        assert!(!backend.is_shared());
        assert_eq!(backend.name(), "cuda_device");

        let alloc = backend.alloc(4096).unwrap();
        assert!(alloc.cpu_ptr.is_null());     // Device-only: no CPU pointer
        assert!(!alloc.device_ptr.is_null());
        assert_eq!(alloc.size, 4096);

        // Zero via GPU memset
        backend.zero(&alloc);

        // Verify by copying to host
        let mut host = vec![0xFFu8; 4096];
        cuda_ffi::memcpy_dtoh(
            host.as_mut_ptr(),
            alloc.device_ptr as CUdeviceptr,
            4096,
        )
        .unwrap();
        assert!(host.iter().all(|&b| b == 0));

        backend.free(&alloc);
    }

    #[test]
    fn test_managed_data_integrity_roundtrip() {
        if !cuda_ffi::is_available() {
            return;
        }
        let backend = CudaManagedBackend::new(0).unwrap();
        let alloc = backend.alloc(256).unwrap();

        // Write pattern from CPU
        let pattern: Vec<u8> = (0..=255).collect();
        unsafe {
            std::ptr::copy_nonoverlapping(pattern.as_ptr(), alloc.cpu_ptr, 256);
        }

        // Read back and verify
        let mut readback = vec![0u8; 256];
        unsafe {
            std::ptr::copy_nonoverlapping(alloc.cpu_ptr, readback.as_mut_ptr(), 256);
        }
        assert_eq!(pattern, readback);

        backend.free(&alloc);
    }

    #[test]
    fn test_device_htod_dtoh_roundtrip() {
        if !cuda_ffi::is_available() {
            return;
        }
        let backend = CudaDeviceBackend::new(0).unwrap();
        let alloc = backend.alloc(256).unwrap();

        // Host → Device
        let data: Vec<u8> = (0..=255).collect();
        cuda_ffi::memcpy_htod(
            alloc.device_ptr as CUdeviceptr,
            data.as_ptr(),
            256,
        )
        .unwrap();

        // Device → Host
        let mut result = vec![0u8; 256];
        cuda_ffi::memcpy_dtoh(
            result.as_mut_ptr(),
            alloc.device_ptr as CUdeviceptr,
            256,
        )
        .unwrap();

        assert_eq!(data, result);
        backend.free(&alloc);
    }

    #[test]
    fn test_drop_frees_all_allocations() {
        if !cuda_ffi::is_available() {
            return;
        }
        // Allocate many buffers, then drop backend — should free all
        let backend = CudaManagedBackend::new(0).unwrap();
        for _ in 0..100 {
            let _ = backend.alloc(1024).unwrap();
        }
        // Drop backend — should free all 100 allocations without leak
        drop(backend);
        // If this doesn't crash or hang, allocations were freed
    }
}
