//! # HORUS Types - Core types with zero HORUS dependencies
//!
//! This is a leaf crate providing the canonical definitions of:
//! - [`TensorDtype`] - Element data types (f32, f16, u8, etc.)
//! - [`Device`] - Device location (CPU or CUDA with unlimited GPU index)
//! - [`HorusTensor`] - Zero-copy tensor descriptor (232 bytes, Pod-safe)
//! - [`ImageEncoding`] - Pixel format (mono8, rgb8, rgba8, etc.)
//! - [`Image`] - Pod image descriptor (288 bytes)
//! - [`PointCloud`] - Pod point cloud descriptor (336 bytes)
//! - [`DepthImage`] - Pod depth image descriptor (288 bytes)
//!
//! All other HORUS crates depend on this crate for these types,
//! eliminating duplication across horus_core, horus_library, and horus_ai.

pub mod device;
pub mod dtype;
pub mod tensor;

// Vision/perception domain types
pub mod image_encoding;
pub mod image;
pub mod pointcloud;
pub mod depth_image;

pub use device::Device;
pub use dtype::{dlpack_codes, TensorDtype};
pub use tensor::{HorusTensor, CUDA_IPC_HANDLE_SIZE, MAX_TENSOR_DIMS};

pub use image_encoding::ImageEncoding;
pub use image::Image;
pub use pointcloud::PointCloud;
pub use depth_image::DepthImage;
