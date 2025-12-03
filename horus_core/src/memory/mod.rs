//! # Shared Memory utilities for HORUS
//!
//! This module provides core shared memory functionality for robotics applications:
//!
//! - **ShmRegion**: Cross-process memory regions using HORUS absolute paths
//! - **ShmTopic**: Lock-free ring buffers in shared memory for high-performance messaging
//!
//! ## Performance Features
//!
//! HORUS shared memory is designed for low-latency robotics systems:
//! - **True shared memory**: Cross-process memory sharing via memory-mapped files
//! - **Lock-free ring buffers**: Atomic operations for high-concurrency scenarios
//! - **Zero-copy access**: Direct memory access without serialization overhead
//!
//! ## Memory Safety
//!
//! All memory operations maintain Rust's safety guarantees through careful
//! use of lifetime management and atomic operations.

pub mod platform;
pub mod shm_region;
pub mod shm_topic;
pub mod tensor_handle;
pub mod tensor_pool;

// CUDA IPC support (optional feature)
#[cfg(feature = "cuda")]
pub mod cuda_ffi;
#[cfg(feature = "cuda")]
pub mod cuda_pool;

pub use platform::*;
pub use shm_region::ShmRegion;
pub use shm_topic::ShmTopic;
pub use tensor_handle::TensorHandle;
pub use tensor_pool::{
    HorusTensor, TensorDevice, TensorDtype, TensorPool, TensorPoolConfig, TensorPoolStats,
    MAX_TENSOR_DIMS,
};

// CUDA exports
#[cfg(feature = "cuda")]
pub use cuda_ffi::{CudaIpcMemHandle, CUDA_IPC_HANDLE_SIZE};
#[cfg(feature = "cuda")]
pub use cuda_pool::{CudaTensor, CudaTensorPool, CudaTensorPoolConfig};

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
