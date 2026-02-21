//! # HORUS Types - Core types with zero HORUS dependencies
//!
//! This is a leaf crate providing the canonical definitions of:
//! - [`TensorDtype`] - Element data types (f32, f16, u8, etc.)
//! - [`Device`] - Device location (CPU or CUDA with unlimited GPU index)
//! - [`HorusTensor`] - Zero-copy tensor descriptor (232 bytes, Pod-safe)
//! - [`ImageEncoding`] - Pixel format (mono8, rgb8, rgba8, etc.)
//! - [`ImageDescriptor`] - Pod image descriptor (288 bytes)
//! - [`PointCloudDescriptor`] - Pod point cloud descriptor (336 bytes)
//! - [`DepthImageDescriptor`] - Pod depth image descriptor (288 bytes)
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

// Point element types (Pod, zero-copy)
pub mod point;

pub use device::Device;
pub use dtype::{dlpack_codes, TensorDtype};
pub use tensor::{HorusTensor, CUDA_IPC_HANDLE_SIZE, MAX_TENSOR_DIMS};

pub use image_encoding::ImageEncoding;
pub use image::ImageDescriptor;
pub use pointcloud::PointCloudDescriptor;
pub use depth_image::DepthImageDescriptor;

pub use point::{PointXYZ, PointXYZI, PointXYZRGB};
