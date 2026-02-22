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

/// Generate `frame_id()` and `set_frame_id()` inherent methods for types
/// with a `frame_id: [u8; 32]` field.
///
/// Eliminates duplication across ImageDescriptor, PointCloudDescriptor,
/// DepthImageDescriptor, and horus_library message types.
#[macro_export]
macro_rules! impl_frame_id_field {
    () => {
        /// Get frame ID as string.
        pub fn frame_id(&self) -> &str {
            let end = self.frame_id.iter().position(|&b| b == 0).unwrap_or(32);
            std::str::from_utf8(&self.frame_id[..end]).unwrap_or("")
        }

        /// Set frame ID from string.
        pub fn set_frame_id(&mut self, id: &str) {
            let bytes = id.as_bytes();
            let len = bytes.len().min(31);
            self.frame_id = [0; 32];
            self.frame_id[..len].copy_from_slice(&bytes[..len]);
        }
    };
}

/// Generate `timestamp_ns()` and `set_timestamp_ns()` inherent methods for
/// types with a `timestamp_ns: u64` field.
#[macro_export]
macro_rules! impl_timestamp_field {
    () => {
        /// Timestamp in nanoseconds since epoch.
        #[inline]
        pub fn timestamp_ns(&self) -> u64 {
            self.timestamp_ns
        }

        /// Set the timestamp.
        #[inline]
        pub fn set_timestamp_ns(&mut self, ts: u64) {
            self.timestamp_ns = ts;
        }
    };
}

/// Generate shared tensor accessor methods for descriptor types with an
/// `inner: HorusTensor` field.
#[macro_export]
macro_rules! impl_tensor_accessors {
    () => {
        /// Element data type.
        #[inline]
        pub fn dtype(&self) -> $crate::TensorDtype {
            self.inner.dtype
        }

        /// Total bytes of data.
        #[inline]
        pub fn nbytes(&self) -> u64 {
            self.inner.nbytes()
        }

        /// Whether tensor data is on CPU.
        #[inline]
        pub fn is_cpu(&self) -> bool {
            self.inner.is_cpu()
        }

        /// Whether tensor data is on a CUDA GPU.
        #[inline]
        pub fn is_cuda(&self) -> bool {
            self.inner.is_cuda()
        }

        /// Get the inner tensor descriptor.
        #[inline]
        pub fn tensor(&self) -> &$crate::HorusTensor {
            &self.inner
        }

        /// Get a mutable reference to the inner tensor descriptor.
        #[inline]
        pub fn tensor_mut(&mut self) -> &mut $crate::HorusTensor {
            &mut self.inner
        }
    };
}

// Vision/perception domain types
pub mod depth_image;
pub mod image;
pub mod image_encoding;
pub mod pointcloud;

// Point element types (Pod, zero-copy)
pub mod point;

pub use device::Device;
pub use dtype::{dlpack_codes, TensorDtype};
pub use tensor::{HorusTensor, CUDA_IPC_HANDLE_SIZE, MAX_TENSOR_DIMS};

pub use depth_image::DepthImageDescriptor;
pub use image::ImageDescriptor;
pub use image_encoding::ImageEncoding;
pub use pointcloud::PointCloudDescriptor;

pub use point::{PointXYZ, PointXYZI, PointXYZRGB};
