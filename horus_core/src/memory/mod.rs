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
pub mod tensor_handle;
pub mod tensor_pool;

// Domain-specific handles (RAII wrappers with rich API)
pub mod image_handle;
pub mod pointcloud_handle;
pub mod depth_handle;

// CUDA IPC support (optional feature)
#[cfg(feature = "cuda")]
pub mod cuda_ffi;
#[cfg(feature = "cuda")]
pub mod cuda_pool;

pub use platform::*;
pub use shm_region::ShmRegion;
pub use simd::{simd_copy_from_shm, simd_copy_to_shm, SIMD_COPY_THRESHOLD};
pub use tensor_handle::TensorHandle;
pub use tensor_pool::{
    Device, HorusTensor, TensorDevice, TensorDtype, TensorPool, TensorPoolConfig, TensorPoolStats,
    MAX_TENSOR_DIMS,
};

pub use image_handle::ImageHandle;
pub use pointcloud_handle::PointCloudHandle;
pub use depth_handle::DepthImageHandle;

// CUDA exports
#[cfg(feature = "cuda")]
pub use cuda_ffi::{CudaIpcMemHandle, CUDA_IPC_HANDLE_SIZE};
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
