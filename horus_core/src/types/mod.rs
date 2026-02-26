//! Core tensor types for HORUS - zero-copy descriptors and element types.

pub mod device;
pub mod dtype;
pub mod tensor;

pub mod image_descriptor;
pub mod image_encoding;
pub mod depth_image_descriptor;
pub mod pointcloud_descriptor;
pub mod point;

/// Generate `frame_id()` and `set_frame_id()` inherent methods for types
/// with a `frame_id: [u8; 32]` field.
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
        pub fn dtype(&self) -> $crate::types::TensorDtype {
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
        pub fn tensor(&self) -> &$crate::types::HorusTensor {
            &self.inner
        }

        /// Get a mutable reference to the inner tensor descriptor.
        #[inline]
        pub fn tensor_mut(&mut self) -> &mut $crate::types::HorusTensor {
            &mut self.inner
        }
    };
}

// User-facing types (re-exported via horus::prelude)
pub use device::Device;
pub use image_encoding::ImageEncoding;
pub use point::{PointXYZ, PointXYZI, PointXYZRGB};
pub use dtype::TensorDtype;

// Internal types
#[doc(hidden)]
pub use dtype::dlpack_codes;
#[doc(hidden)]
pub use tensor::{HorusTensor, MAX_TENSOR_DIMS};
#[doc(hidden)]
pub use depth_image_descriptor::DepthImageDescriptor;
#[doc(hidden)]
pub use image_descriptor::ImageDescriptor;
#[doc(hidden)]
pub use pointcloud_descriptor::PointCloudDescriptor;
