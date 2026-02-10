//! HORUS AI - Machine Learning and AI tensor system
//!
//! This crate provides high-performance tensor primitives for ML/AI workloads:
//! - Zero-copy tensor sharing via shared memory
//! - DLPack interop for PyTorch, JAX, TensorFlow, CuPy
//! - CUDA tensor support with IPC handles
//! - Memory pooling for efficient allocation
//! - ML model registry and loader with caching
//!
//! # Example
//!
//! ```rust,ignore
//! use horus_ai::{TensorPool, TensorHandle, TensorDtype, Device};
//!
//! // Create a tensor pool
//! let pool = TensorPool::new(1, Default::default())?;
//!
//! // Allocate a tensor
//! let tensor = TensorHandle::alloc(&pool, &[1080, 1920, 3], TensorDtype::U8, Device::Cpu)?;
//!
//! // Get raw data pointer for zero-copy access
//! let ptr = tensor.data_ptr();
//! ```

pub mod device;
pub mod dlpack;
pub mod ml;
pub mod tensor;

// Re-export main types at crate root
pub use device::Device;
pub use dlpack::{DLDataType, DLDevice, DLManagedTensor, DLTensor};
pub use ml::{ModelEntry, ModelLoader, ModelRegistry};
pub use tensor::{TensorDescriptor, TensorDtype};

// Re-export from horus_core for convenience (will be moved here later)
pub use horus_core::memory::tensor_handle::TensorHandle;
pub use horus_core::memory::tensor_pool::{TensorPool, TensorPoolConfig, TensorPoolStats};
