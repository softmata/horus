//! # Shared Memory utilities for HORUS
//!
//! This module provides core shared memory functionality for robotics applications:
//!
//! - **ShmRegion**: Cross-process memory regions using HORUS absolute paths
//!
//! ## Performance Features
//!
//! HORUS shared memory is designed for low-latency robotics systems:
//! - **True shared memory**: Cross-process memory sharing via memory-mapped files
//! - **Zero-copy access**: Direct memory access without serialization overhead
//!
//! ## Memory Safety
//!
//! All memory operations maintain Rust's safety guarantees through careful
//! use of lifetime management and atomic operations.
//!
//! ## Note
//!
//! The unified `Topic<T>` API (see `communication::Topic`) is the recommended
//! interface for all IPC. It uses `ShmRegion` internally for cross-process paths.

pub(crate) mod platform;
pub(crate) mod shm_region;
pub(crate) mod simd;
pub(crate) mod tensor_descriptor;

// Re-export platform functions needed by horus_manager
pub use platform::{has_native_shm, shm_base_dir, shm_control_dir, shm_network_dir, shm_topics_dir};
pub mod tensor_handle;
pub mod tensor_pool;

/// Generate shared methods and trait impls for tensor-backed RAII types.
///
/// All three domain types (Image, PointCloud, DepthImage) share the same
/// `descriptor: D` + `pool: Arc<TensorPool>` structure and identical
/// data access, lifecycle, and metadata delegation code.
#[macro_export]
macro_rules! impl_tensor_backed {
    ($type_name:ident, $desc_type:ty, $label:expr) => {
        impl $type_name {
            /// Create from a descriptor and pool (used by Topic recv and Python bindings).
            pub fn from_owned(
                descriptor: $desc_type,
                pool: std::sync::Arc<$crate::memory::tensor_pool::TensorPool>,
            ) -> Self {
                Self { descriptor, pool }
            }

            /// Get raw data as a byte slice (zero-copy from shared memory).
            #[inline]
            pub fn data(&self) -> &[u8] {
                self.pool.data_slice(self.descriptor.tensor())
            }

            /// Get raw data as a mutable byte slice (zero-copy from shared memory).
            #[inline]
            #[allow(clippy::mut_from_ref)]
            pub fn data_mut(&self) -> &mut [u8] {
                self.pool.data_slice_mut(self.descriptor.tensor())
            }

            /// Copy data from a buffer into shared memory.
            ///
            /// Returns `&mut Self` for method chaining.
            ///
            /// # Panics
            ///
            /// Panics if `src` length doesn't match `nbytes()`.
            pub fn copy_from(&mut self, src: &[u8]) -> &mut Self {
                let data = self.data_mut();
                assert_eq!(
                    src.len(),
                    data.len(),
                    "source buffer size ({}) doesn't match {} size ({})",
                    src.len(),
                    $label,
                    data.len()
                );
                $crate::memory::simd::fast_copy_to_shm(src, data);
                self
            }

            /// Get data as a typed slice.
            ///
            /// # Safety
            /// Caller must ensure the dtype matches the requested type T.
            #[inline]
            pub unsafe fn data_as<T: Copy>(&self) -> &[T] {
                let bytes = self.data();
                let ptr = bytes.as_ptr() as *const T;
                let len = bytes.len() / std::mem::size_of::<T>();
                std::slice::from_raw_parts(ptr, len)
            }

            /// Get data as a mutable typed slice.
            ///
            /// # Safety
            /// Caller must ensure the dtype matches the requested type T.
            #[inline]
            #[allow(clippy::mut_from_ref)]
            pub unsafe fn data_as_mut<T: Copy>(&self) -> &mut [T] {
                let bytes = self.data_mut();
                let ptr = bytes.as_mut_ptr() as *mut T;
                let len = bytes.len() / std::mem::size_of::<T>();
                std::slice::from_raw_parts_mut(ptr, len)
            }

            /// Set the frame ID.
            pub fn set_frame_id(&mut self, id: &str) -> &mut Self {
                self.descriptor.set_frame_id(id);
                self
            }

            /// Set the timestamp in nanoseconds.
            pub fn set_timestamp_ns(&mut self, ts: u64) -> &mut Self {
                self.descriptor.set_timestamp_ns(ts);
                self
            }

            /// Element data type.
            #[inline]
            pub fn dtype(&self) -> horus_types::TensorDtype {
                self.descriptor.dtype()
            }

            /// Total bytes of data.
            #[inline]
            pub fn nbytes(&self) -> u64 {
                self.descriptor.nbytes()
            }

            /// Whether tensor data is on CPU.
            #[inline]
            pub fn is_cpu(&self) -> bool {
                self.descriptor.is_cpu()
            }

            /// Whether tensor data is on CUDA.
            #[inline]
            pub fn is_cuda(&self) -> bool {
                self.descriptor.is_cuda()
            }

            /// Timestamp in nanoseconds.
            #[inline]
            pub fn timestamp_ns(&self) -> u64 {
                self.descriptor.timestamp_ns()
            }

            /// Frame ID.
            #[inline]
            pub fn frame_id(&self) -> &str {
                self.descriptor.frame_id()
            }

            /// Get the underlying descriptor.
            #[inline]
            pub fn descriptor(&self) -> &$desc_type {
                &self.descriptor
            }

            /// Get the pool reference.
            #[inline]
            pub fn pool(&self) -> &std::sync::Arc<$crate::memory::tensor_pool::TensorPool> {
                &self.pool
            }
        }

        impl Clone for $type_name {
            fn clone(&self) -> Self {
                self.pool.retain(self.descriptor.tensor());
                Self {
                    descriptor: self.descriptor,
                    pool: std::sync::Arc::clone(&self.pool),
                }
            }
        }

        impl Drop for $type_name {
            fn drop(&mut self) {
                self.pool.release(self.descriptor.tensor());
            }
        }

        // Safety: The underlying pool uses atomic operations for all shared state.
        unsafe impl Send for $type_name {}
        unsafe impl Sync for $type_name {}
    };
}

// Domain-specific types (RAII wrappers with rich API for data access)
pub mod depth_image;
pub mod image;
pub mod pointcloud;

// CUDA IPC support (optional feature) — internal modules, use pub re-exports below
#[cfg(feature = "cuda")]
pub(crate) mod cuda_ffi;
#[cfg(feature = "cuda")]
pub(crate) mod cuda_pool;
// GPU hardware detection (feature-gated)
#[cfg(feature = "cuda")]
pub(crate) mod gpu_detect;

pub use tensor_handle::TensorHandle;
pub use tensor_pool::{PoolAllocator, TensorPool, TensorPoolConfig};

pub use depth_image::DepthImage;
pub use image::Image;
pub use pointcloud::PointCloud;

// CUDA pool types — internal only. Users interact via Topic<HorusTensor>.
#[cfg(feature = "cuda")]
pub(crate) use cuda_pool::{
    CudaPoolStats, CudaTensorPool, CudaTensorPoolConfig, P2PAccessInfo, P2PManager,
};

// CUDA FFI types — only CudaMemcpyKind is user-facing (for horus_py DLPack bridge).
// Everything else stays crate-internal.
#[cfg(feature = "cuda")]
pub use cuda_ffi::CudaMemcpyKind;
#[cfg(feature = "cuda")]
pub(crate) use cuda_ffi::{
    CudaError, CudaEvent, CudaIpcMemHandle, CudaResult, CudaStream, CUDA_IPC_HANDLE_SIZE,
};

// GPU capability detection — public types and query function
#[cfg(feature = "cuda")]
pub use gpu_detect::GpuCapability;

/// GPU capability level detected at runtime (stub for non-CUDA builds).
///
/// Without CUDA support, no GPU is ever detected.
#[cfg(not(feature = "cuda"))]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum GpuCapability {
    /// No CUDA-capable GPU detected — pure CPU allocation via mmap.
    None,
}

#[cfg(not(feature = "cuda"))]
impl GpuCapability {
    /// Returns true if any GPU is available. Always false without CUDA.
    pub fn has_gpu(&self) -> bool {
        false
    }
    /// Returns true if this is a unified memory device. Always false without CUDA.
    pub fn is_unified(&self) -> bool {
        false
    }
    /// Returns true if explicit coherency sync is needed. Always false without CUDA.
    pub fn needs_coherency_sync(&self) -> bool {
        false
    }
    /// Returns the primary device ID. Always None without CUDA.
    pub fn device_id(&self) -> Option<i32> {
        None
    }
}

/// Query the GPU capability detected at startup.
///
/// First call probes hardware (CUDA runtime). All subsequent calls return the
/// cached result with zero overhead (atomic load).
///
/// Returns [`GpuCapability::None`] if no GPU is found or CUDA is not available.
#[cfg(feature = "cuda")]
pub fn gpu_capability() -> GpuCapability {
    gpu_detect::detect_gpu()
}

/// Query the GPU capability (stub for non-CUDA builds — always None).
#[cfg(not(feature = "cuda"))]
pub fn gpu_capability() -> GpuCapability {
    GpuCapability::None
}

// =============================================================================
// CUDA operations — public surface is minimal (query + horus_py bridge ops)
// =============================================================================

/// Check if CUDA is available at runtime.
#[cfg(feature = "cuda")]
pub fn cuda_available() -> bool {
    cuda_ffi::cuda_available()
}

/// Get number of CUDA devices (returns 0 if CUDA unavailable).
#[cfg(feature = "cuda")]
pub fn cuda_device_count() -> usize {
    cuda_ffi::get_device_count().unwrap_or(0) as usize
}

/// Set the active CUDA device (used by horus_py for `.cuda()` transfers).
#[cfg(feature = "cuda")]
pub fn cuda_set_device(device: i32) -> cuda_ffi::CudaResult<()> {
    cuda_ffi::set_device(device)
}

/// Allocate GPU device memory (used by horus_py for `.cuda()` transfers).
#[cfg(feature = "cuda")]
pub fn cuda_malloc(size: usize) -> cuda_ffi::CudaResult<*mut std::ffi::c_void> {
    cuda_ffi::malloc(size)
}

/// Free GPU device memory (used by horus_py for cleanup).
#[cfg(feature = "cuda")]
pub fn cuda_free(ptr: *mut std::ffi::c_void) -> cuda_ffi::CudaResult<()> {
    cuda_ffi::free(ptr)
}

/// Copy memory between host and device (used by horus_py DLPack bridge).
#[cfg(feature = "cuda")]
pub fn cuda_memcpy(
    dst: *mut std::ffi::c_void,
    src: *const std::ffi::c_void,
    size: usize,
    kind: cuda_ffi::CudaMemcpyKind,
) -> cuda_ffi::CudaResult<()> {
    cuda_ffi::memcpy(dst, src, size, kind)
}

/// Get an IPC memory handle for cross-process GPU memory sharing
/// (used by horus_py `.cuda()` to stamp IPC handles on new GPU tensors).
#[cfg(feature = "cuda")]
pub fn cuda_ipc_get_mem_handle(
    dev_ptr: *mut std::ffi::c_void,
) -> cuda_ffi::CudaResult<cuda_ffi::CudaIpcMemHandle> {
    cuda_ffi::ipc_get_mem_handle(dev_ptr)
}

// --- Internal CUDA operations (used by Topic dispatch, tensor pool, tests) ---

/// Get the currently active CUDA device.
#[cfg(feature = "cuda")]
pub(crate) fn cuda_get_device() -> cuda_ffi::CudaResult<i32> {
    cuda_ffi::get_device()
}

/// Synchronize the current CUDA device (wait for all pending work).
#[cfg(feature = "cuda")]
pub(crate) fn cuda_device_synchronize() -> cuda_ffi::CudaResult<()> {
    cuda_ffi::device_synchronize()
}

/// Open an IPC memory handle from another process to access shared GPU memory.
#[cfg(feature = "cuda")]
pub(crate) fn cuda_ipc_open_mem_handle(
    handle: cuda_ffi::CudaIpcMemHandle,
) -> cuda_ffi::CudaResult<*mut std::ffi::c_void> {
    cuda_ffi::ipc_open_mem_handle(handle)
}

/// Close a previously opened IPC memory handle.
#[cfg(feature = "cuda")]
pub(crate) fn cuda_ipc_close_mem_handle(
    dev_ptr: *mut std::ffi::c_void,
) -> cuda_ffi::CudaResult<()> {
    cuda_ffi::ipc_close_mem_handle(dev_ptr)
}

// Stub functions when CUDA is not enabled
#[cfg(not(feature = "cuda"))]
pub fn cuda_available() -> bool {
    false
}
#[cfg(not(feature = "cuda"))]
pub fn cuda_device_count() -> usize {
    0
}

// Tests are in the tests/ directory
