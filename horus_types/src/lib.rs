//! # HORUS Types - Core tensor types with zero HORUS dependencies
//!
//! This is a leaf crate providing the canonical definitions of:
//! - [`TensorDtype`] - Element data types (f32, f16, u8, etc.)
//! - [`Device`] - Device location (CPU or CUDA with unlimited GPU index)
//! - [`HorusTensor`] - Zero-copy tensor descriptor (232 bytes, Pod-safe)
//!
//! All other HORUS crates depend on this crate for tensor types,
//! eliminating the previous 3x duplication across horus_core, horus_library, and horus_ai.

pub mod device;
pub mod dtype;
pub mod tensor;

pub use device::Device;
pub use dtype::{dlpack_codes, TensorDtype};
pub use tensor::{HorusTensor, CUDA_IPC_HANDLE_SIZE, MAX_TENSOR_DIMS};
