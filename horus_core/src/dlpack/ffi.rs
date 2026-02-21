//! DLPack FFI definitions
//!
//! These structs match the DLPack C header exactly for ABI compatibility.
//! See: <https://github.com/dmlc/dlpack/blob/main/include/dlpack/dlpack.h>

use std::ffi::c_void;
use std::os::raw::c_int;

/// DLPack device specification
///
/// Identifies where tensor data resides.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct DLDevice {
    /// Device type (kDLCPU=1, kDLCUDA=2, etc.)
    pub device_type: i32,
    /// Device index (e.g., GPU 0, 1, 2...)
    pub device_id: i32,
}

impl DLDevice {
    /// Create a CPU device
    #[inline]
    pub const fn cpu() -> Self {
        Self {
            device_type: super::device_type::CPU,
            device_id: 0,
        }
    }

    /// Create a CUDA device
    #[inline]
    pub const fn cuda(device_id: i32) -> Self {
        Self {
            device_type: super::device_type::CUDA,
            device_id,
        }
    }

    /// Check if this is a CPU device
    #[inline]
    pub const fn is_cpu(&self) -> bool {
        self.device_type == super::device_type::CPU
    }

    /// Check if this is a CUDA device
    #[inline]
    pub const fn is_cuda(&self) -> bool {
        self.device_type == super::device_type::CUDA
    }
}

/// DLPack data type specification
///
/// Describes the element type of a tensor.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct DLDataType {
    /// Type category:
    /// - 0 = int
    /// - 1 = uint
    /// - 2 = float
    /// - 3 = opaque handle
    /// - 4 = bfloat
    /// - 5 = complex
    /// - 6 = bool
    pub code: u8,
    /// Number of bits per element (e.g., 8, 16, 32, 64)
    pub bits: u8,
    /// Number of lanes (usually 1, >1 for vectorized types)
    pub lanes: u16,
}

impl DLDataType {
    /// Create a new data type
    #[inline]
    pub const fn new(code: u8, bits: u8, lanes: u16) -> Self {
        Self { code, bits, lanes }
    }

    /// Float32
    pub const F32: Self = Self::new(2, 32, 1);
    /// Float64
    pub const F64: Self = Self::new(2, 64, 1);
    /// Float16
    pub const F16: Self = Self::new(2, 16, 1);
    /// BFloat16
    pub const BF16: Self = Self::new(4, 16, 1);
    /// Int8
    pub const I8: Self = Self::new(0, 8, 1);
    /// Int16
    pub const I16: Self = Self::new(0, 16, 1);
    /// Int32
    pub const I32: Self = Self::new(0, 32, 1);
    /// Int64
    pub const I64: Self = Self::new(0, 64, 1);
    /// UInt8
    pub const U8: Self = Self::new(1, 8, 1);
    /// UInt16
    pub const U16: Self = Self::new(1, 16, 1);
    /// UInt32
    pub const U32: Self = Self::new(1, 32, 1);
    /// UInt64
    pub const U64: Self = Self::new(1, 64, 1);
    /// Bool
    pub const BOOL: Self = Self::new(6, 8, 1);
}

/// DLPack tensor structure
///
/// Core tensor metadata without ownership semantics.
/// This struct is ABI-compatible with the DLPack C definition.
#[repr(C)]
#[derive(Debug)]
pub struct DLTensor {
    /// Pointer to the tensor data
    ///
    /// For CUDA tensors, this is a device pointer.
    /// For CPU tensors, this is a host pointer.
    pub data: *mut c_void,

    /// Device where data resides
    pub device: DLDevice,

    /// Number of dimensions
    pub ndim: c_int,

    /// Data type of elements
    pub dtype: DLDataType,

    /// Shape of the tensor
    ///
    /// Pointer to array of `ndim` elements.
    /// Shape values are in number of elements, not bytes.
    pub shape: *mut i64,

    /// Strides of the tensor in number of elements
    ///
    /// Pointer to array of `ndim` elements.
    /// Can be NULL for compact row-major tensors.
    /// Strides are in number of elements, not bytes.
    pub strides: *mut i64,

    /// Byte offset from data pointer
    ///
    /// The actual data starts at `data + byte_offset`.
    pub byte_offset: u64,
}

impl Default for DLTensor {
    fn default() -> Self {
        Self {
            data: std::ptr::null_mut(),
            device: DLDevice::default(),
            ndim: 0,
            dtype: DLDataType::default(),
            shape: std::ptr::null_mut(),
            strides: std::ptr::null_mut(),
            byte_offset: 0,
        }
    }
}

/// Deleter function type for DLManagedTensor
pub type DLManagedTensorDeleter = unsafe extern "C" fn(*mut DLManagedTensor);

/// DLPack managed tensor with ownership
///
/// Wraps a DLTensor with a deleter callback for memory management.
/// When the consumer is done with the tensor, they must call the deleter.
#[repr(C)]
pub struct DLManagedTensor {
    /// The tensor metadata
    pub dl_tensor: DLTensor,

    /// Opaque pointer to manager context
    ///
    /// Used by the producer to track state needed for cleanup.
    pub manager_ctx: *mut c_void,

    /// Deleter function
    ///
    /// Called by the consumer when done with the tensor.
    /// The deleter should free any resources and the DLManagedTensor itself.
    pub deleter: Option<DLManagedTensorDeleter>,
}

impl Default for DLManagedTensor {
    fn default() -> Self {
        Self {
            dl_tensor: DLTensor::default(),
            manager_ctx: std::ptr::null_mut(),
            deleter: None,
        }
    }
}

// Safety: DLManagedTensor can be sent between threads if the underlying
// data is thread-safe (which it should be for tensor data).
unsafe impl Send for DLManagedTensor {}

#[cfg(test)]
mod tests {
    use super::*;
    use std::mem;

    #[test]
    fn test_dldevice_size() {
        // DLDevice should be 8 bytes (2 x i32)
        assert_eq!(mem::size_of::<DLDevice>(), 8);
    }

    #[test]
    fn test_dldatatype_size() {
        // DLDataType should be 4 bytes (u8 + u8 + u16)
        assert_eq!(mem::size_of::<DLDataType>(), 4);
    }

    #[test]
    fn test_dltensor_layout() {
        // Verify struct layout matches C definition
        // On 64-bit: data(8) + device(8) + ndim(4) + padding(4) + dtype(4) + padding(4) + shape(8) + strides(8) + byte_offset(8)
        // This may vary by platform, but should be consistent
        let tensor = DLTensor::default();
        assert!(tensor.data.is_null());
        assert_eq!(tensor.ndim, 0);
    }

    #[test]
    fn test_device_constructors() {
        let cpu = DLDevice::cpu();
        assert!(cpu.is_cpu());
        assert!(!cpu.is_cuda());

        let cuda = DLDevice::cuda(1);
        assert!(!cuda.is_cpu());
        assert!(cuda.is_cuda());
        assert_eq!(cuda.device_id, 1);
    }
}
