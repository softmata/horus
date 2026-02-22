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

pub mod platform;
pub mod shm_region;
pub mod simd;
pub mod tensor_descriptor;
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
            pub fn from_owned(descriptor: $desc_type, pool: std::sync::Arc<$crate::memory::tensor_pool::TensorPool>) -> Self {
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
            pub fn dtype(&self) -> horus_types::TensorDtype { self.descriptor.dtype() }

            /// Total bytes of data.
            #[inline]
            pub fn nbytes(&self) -> u64 { self.descriptor.nbytes() }

            /// Whether tensor data is on CPU.
            #[inline]
            pub fn is_cpu(&self) -> bool { self.descriptor.is_cpu() }

            /// Whether tensor data is on CUDA.
            #[inline]
            pub fn is_cuda(&self) -> bool { self.descriptor.is_cuda() }

            /// Timestamp in nanoseconds.
            #[inline]
            pub fn timestamp_ns(&self) -> u64 { self.descriptor.timestamp_ns() }

            /// Frame ID.
            #[inline]
            pub fn frame_id(&self) -> &str { self.descriptor.frame_id() }

            /// Get the underlying descriptor.
            #[inline]
            pub fn descriptor(&self) -> &$desc_type { &self.descriptor }

            /// Get the pool reference.
            #[inline]
            pub fn pool(&self) -> &std::sync::Arc<$crate::memory::tensor_pool::TensorPool> { &self.pool }
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

// CUDA IPC support (optional feature)
#[cfg(feature = "cuda")]
pub mod cuda_ffi;
#[cfg(feature = "cuda")]
pub mod cuda_pool;

pub use platform::*;
pub use tensor_handle::TensorHandle;
pub use tensor_pool::{TensorPool, TensorPoolConfig};

pub use depth_image::DepthImage;
pub use image::Image;
pub use pointcloud::PointCloud;

// CUDA exports
#[cfg(feature = "cuda")]
pub use cuda_pool::{
    CudaPoolStats, CudaTensor, CudaTensorPool, CudaTensorPoolConfig, P2PAccessInfo, P2PManager,
};

/// Check if CUDA is available at runtime
#[cfg(feature = "cuda")]
pub fn cuda_available() -> bool {
    cuda_ffi::cuda_available()
}

/// Get number of CUDA devices (returns 0 if CUDA unavailable)
#[cfg(feature = "cuda")]
pub fn cuda_device_count() -> usize {
    cuda_ffi::get_device_count().unwrap_or(0) as usize
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
