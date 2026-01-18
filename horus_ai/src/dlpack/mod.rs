//! DLPack tensor exchange protocol
//!
//! This module implements the DLPack standard for zero-copy tensor exchange
//! between deep learning frameworks (PyTorch, JAX, TensorFlow, CuPy, etc.).
//!
//! # DLPack Overview
//!
//! DLPack is a minimal, stable ABI for tensor exchange. It defines:
//! - `DLTensor`: Core tensor metadata (data ptr, shape, strides, dtype, device)
//! - `DLManagedTensor`: Owned tensor with deleter callback
//! - `DLDevice`: Device type and index
//! - `DLDataType`: Element type (int/uint/float, bits, lanes)
//!
//! # Usage
//!
//! ```rust,ignore
//! use horus_ai::dlpack::{to_dlpack, from_dlpack};
//!
//! // Export HORUS tensor to DLPack
//! let managed = to_dlpack(&horus_tensor);
//!
//! // Import DLPack tensor to HORUS
//! let horus_tensor = from_dlpack(managed)?;
//! ```
//!
//! # Python Integration
//!
//! In Python, implement `__dlpack__` and `__dlpack_device__` methods:
//!
//! ```python
//! class Tensor:
//!     def __dlpack__(self, stream=None):
//!         return self._to_dlpack_capsule(stream)
//!
//!     def __dlpack_device__(self):
//!         return (device_type, device_id)
//! ```

mod ffi;
mod export;
mod import;

pub use ffi::{DLDataType, DLDevice, DLManagedTensor, DLTensor};
pub use export::to_dlpack;
pub use import::from_dlpack;

/// DLPack version supported by this implementation
pub const DLPACK_VERSION: u32 = 0x80; // Version 0.8

/// DLPack device type codes
pub mod device_type {
    /// CPU device
    pub const CPU: i32 = 1;
    /// CUDA GPU
    pub const CUDA: i32 = 2;
    /// CUDA managed/unified memory
    pub const CUDA_MANAGED: i32 = 13;
    /// ROCm GPU
    pub const ROCM: i32 = 10;
    /// Vulkan GPU
    pub const VULKAN: i32 = 7;
    /// Metal GPU
    pub const METAL: i32 = 8;
    /// OpenCL
    pub const OPENCL: i32 = 4;
}
