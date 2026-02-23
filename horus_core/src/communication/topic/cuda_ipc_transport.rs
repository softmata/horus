//! CUDA IPC transport for cross-process GPU tensor sharing.
//!
//! When a tensor lives in GPU memory (device.is_cuda()), cross-process transport
//! uses CUDA IPC handles instead of copying data through the SHM ring buffer.
//!
//! The 64-byte IPC handle is embedded in the `HorusTensor.cuda_ipc_handle` field,
//! which is already part of the descriptor that flows through the ring.
//!
//! # Send path
//!
//! 1. Caller has a GPU-backed tensor (managed or device memory)
//! 2. `stamp_ipc_handle()` calls `cudaIpcGetMemHandle` on the data pointer
//! 3. Handle is written into `tensor.cuda_ipc_handle[0..64]`
//! 4. Descriptor goes through the SHM ring buffer (existing path)
//!
//! # Recv path
//!
//! 1. Descriptor arrives with `device.is_cuda()` and non-zero IPC handle
//! 2. `open_ipc_handle()` calls `cudaIpcOpenMemHandle` to get a local GPU pointer
//! 3. Receiver uses this pointer to access the same GPU memory
//! 4. Handle is cached per (pool_id, slot_id) to avoid repeated opens
//! 5. Cache entries are cleaned up when the tensor handle is dropped

#[cfg(feature = "cuda")]
use crate::memory::cuda_ffi;
#[cfg(feature = "cuda")]
use std::collections::HashMap;

use horus_types::HorusTensor;

/// Stamp the CUDA IPC handle into a tensor descriptor before sending.
///
/// Given a tensor whose data lives in GPU memory (via the pool's managed/device pointer),
/// this retrieves the IPC handle and writes it into `tensor.cuda_ipc_handle`.
///
/// # Unified Memory Fast-Path (Jetson)
///
/// On Jetson and other integrated GPUs with unified memory, CPU and GPU share the
/// same physical memory through `cudaMallocManaged`. Both processes access data
/// through the shared pool's managed pointer — no CUDA IPC handles needed.
/// This avoids the overhead of `cudaIpcGetMemHandle()` on every send.
///
/// A lightweight memory fence ensures all CPU data writes are committed before the
/// descriptor is published through the ring buffer.
///
/// # Arguments
/// * `tensor` — Mutable reference to the tensor descriptor to stamp
/// * `data_ptr` — The GPU device pointer to the tensor's data region
///
/// # Returns
/// * `true` if the handle was successfully stamped (discrete GPU path)
/// * `false` if on unified memory (no handle needed), CUDA IPC is unavailable,
///   or the operation failed
#[cfg(feature = "cuda")]
pub(crate) fn stamp_ipc_handle(tensor: &mut HorusTensor, data_ptr: *mut u8) -> bool {
    if data_ptr.is_null() || !tensor.device().is_cuda() {
        return false;
    }

    // Unified memory (Jetson): CPU and GPU share physical memory via cudaMallocManaged.
    // The existing SHM descriptor path is sufficient — both processes access data through
    // the same managed pointer. Skip IPC handle stamping for zero overhead.
    //
    // Cache coherency: the ring buffer's acquire/release ordering ensures CPU-to-CPU
    // visibility. For CPU→GPU scenarios, Xavier+ (ConcurrentManagedAccess) provides
    // hardware coherency; older Jetson needs application-level cudaDeviceSynchronize
    // before GPU kernel launch.
    if crate::memory::gpu_detect::detect_gpu().is_unified() {
        // Memory fence: ensures all prior data writes to managed memory are ordered
        // before the ring buffer's Release store (belt-and-suspenders with ring ordering).
        std::sync::atomic::fence(std::sync::atomic::Ordering::Release);
        return false;
    }

    // Discrete GPU: stamp IPC handle for cross-process GPU memory sharing
    match cuda_ffi::ipc_get_mem_handle(data_ptr as *mut std::ffi::c_void) {
        Ok(handle) => {
            tensor.cuda_ipc_handle = handle.reserved;
            true
        }
        Err(_) => false,
    }
}

/// Stub when CUDA is not available.
#[cfg(not(feature = "cuda"))]
pub(crate) fn stamp_ipc_handle(_tensor: &mut HorusTensor, _data_ptr: *mut u8) -> bool {
    false
}

/// Check if a tensor descriptor carries a valid CUDA IPC handle.
///
/// A non-zero `cuda_ipc_handle` on a CUDA-device tensor means the sender
/// stamped an IPC handle that the receiver can open.
#[cfg(test)]
pub(crate) fn has_ipc_handle(tensor: &HorusTensor) -> bool {
    tensor.device().is_cuda() && tensor.cuda_ipc_handle.iter().any(|&b| b != 0)
}

// =============================================================================
// IPC handle cache — one per thread, keyed by (pool_id, slot_id, generation)
// =============================================================================

/// Key for the IPC handle cache.
///
/// We include generation to avoid stale mappings after a slot is freed and reused.
#[cfg(feature = "cuda")]
#[derive(Clone, Copy, PartialEq, Eq, Hash)]
struct IpcCacheKey {
    pool_id: u32,
    slot_id: u32,
    generation: u32,
}

/// A cached IPC memory mapping.
#[cfg(feature = "cuda")]
struct IpcMapping {
    /// Local GPU pointer obtained from cudaIpcOpenMemHandle
    dev_ptr: *mut std::ffi::c_void,
}

#[cfg(feature = "cuda")]
impl Drop for IpcMapping {
    fn drop(&mut self) {
        let _ = cuda_ffi::ipc_close_mem_handle(self.dev_ptr);
    }
}

/// Thread-local cache of opened IPC handles.
///
/// Each receiver thread maintains its own cache. Entries are evicted when
/// the tensor generation changes (slot reuse) or when the cache is dropped.
#[cfg(feature = "cuda")]
pub(crate) struct IpcHandleCache {
    entries: HashMap<IpcCacheKey, IpcMapping>,
}

#[cfg(feature = "cuda")]
impl IpcHandleCache {
    pub(crate) fn new() -> Self {
        Self {
            entries: HashMap::new(),
        }
    }

    /// Open or retrieve a cached IPC handle mapping.
    ///
    /// Returns the local GPU device pointer for the tensor's data region,
    /// or `None` if the IPC handle could not be opened.
    ///
    /// # Arguments
    /// * `tensor` — The received tensor descriptor with a valid IPC handle
    pub(crate) fn open_or_get(&mut self, tensor: &HorusTensor) -> Option<*mut u8> {
        let key = IpcCacheKey {
            pool_id: tensor.pool_id,
            slot_id: tensor.slot_id,
            generation: tensor.generation,
        };

        // Check cache first
        if let Some(mapping) = self.entries.get(&key) {
            // Return base pointer + offset
            return Some(unsafe { (mapping.dev_ptr as *mut u8).add(tensor.offset as usize) });
        }

        // Open the IPC handle
        let handle = cuda_ffi::CudaIpcMemHandle {
            reserved: tensor.cuda_ipc_handle,
        };

        match cuda_ffi::ipc_open_mem_handle(handle) {
            Ok(dev_ptr) => {
                let data_ptr = unsafe { (dev_ptr as *mut u8).add(tensor.offset as usize) };
                self.entries.insert(key, IpcMapping { dev_ptr });
                Some(data_ptr)
            }
            Err(_) => None,
        }
    }

    /// Evict stale entries for a specific pool/slot that have an older generation.
    ///
    /// Called when a slot is freed and reallocated (generation bumped).
    pub(crate) fn evict_stale(&mut self, pool_id: u32, slot_id: u32, current_gen: u32) {
        self.entries.retain(|k, _| {
            !(k.pool_id == pool_id && k.slot_id == slot_id && k.generation != current_gen)
        });
    }

    /// Number of cached entries (for diagnostics).
    pub(crate) fn len(&self) -> usize {
        self.entries.len()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_has_ipc_handle_cpu_tensor() {
        let tensor = HorusTensor::default(); // CPU, all zeros
        assert!(!has_ipc_handle(&tensor));
    }

    #[test]
    fn test_has_ipc_handle_cuda_no_handle() {
        let mut tensor = HorusTensor::default();
        tensor.device_type = 1; // CUDA
        tensor.device_id = 0;
        // handle is all zeros
        assert!(!has_ipc_handle(&tensor));
    }

    #[test]
    fn test_has_ipc_handle_cuda_with_handle() {
        let mut tensor = HorusTensor::default();
        tensor.device_type = 1; // CUDA
        tensor.device_id = 0;
        tensor.cuda_ipc_handle[0] = 0xAB; // Non-zero = valid handle
        assert!(has_ipc_handle(&tensor));
    }

    #[test]
    fn test_stamp_ipc_handle_null_ptr() {
        let mut tensor = HorusTensor::default();
        tensor.device_type = 1;
        assert!(!stamp_ipc_handle(&mut tensor, std::ptr::null_mut()));
    }

    #[test]
    fn test_stamp_ipc_handle_cpu_device() {
        let mut tensor = HorusTensor::default(); // CPU device
        let fake_ptr = 0x1000 as *mut u8;
        assert!(!stamp_ipc_handle(&mut tensor, fake_ptr));
    }

    #[cfg(feature = "cuda")]
    #[test]
    fn test_ipc_cache_evict_stale() {
        let mut cache = IpcHandleCache::new();
        // Can't actually open real IPC handles without a GPU, but we can test eviction logic
        assert_eq!(cache.len(), 0);
        cache.evict_stale(1, 0, 5); // No-op on empty cache
        assert_eq!(cache.len(), 0);
    }
}
