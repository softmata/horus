//! DLPack import - convert DLPack tensors to HORUS format
//!
//! This module provides functionality to import DLPack managed tensors
//! from other frameworks (PyTorch, JAX, TensorFlow, etc.) into HORUS.

use super::ffi::{DLDevice, DLManagedTensor};
use crate::memory::tensor_descriptor::TensorDescriptor;
use crate::types::{Device, TensorDtype};
use thiserror::Error;

/// Errors that can occur when importing DLPack tensors
#[derive(Debug, Error)]
pub enum DLPackImportError {
    /// Null pointer provided
    #[error("DLManagedTensor pointer is null")]
    NullPointer,

    /// Unsupported device type
    #[error("Unsupported device type: {0}")]
    UnsupportedDevice(i32),

    /// Unsupported data type
    #[error("Unsupported data type: code={code}, bits={bits}, lanes={lanes}")]
    UnsupportedDtype { code: u8, bits: u8, lanes: u16 },

    /// Invalid tensor (null data, shape, etc.)
    #[error("Invalid tensor: {0}")]
    InvalidTensor(String),
}

impl From<DLPackImportError> for crate::error::HorusError {
    fn from(err: DLPackImportError) -> Self {
        crate::error::HorusError::Memory(crate::error::MemoryError::DLPackImportFailed {
            reason: err.to_string(),
        })
    }
}

/// Import a DLPack tensor to HORUS
///
/// Extracts tensor metadata from a DLManagedTensor and returns a TensorDescriptor.
/// The caller is responsible for managing the DLManagedTensor lifecycle.
///
/// # Arguments
///
/// * `managed` - Pointer to DLManagedTensor from another framework
///
/// # Safety
///
/// The caller must ensure:
/// - `managed` points to a valid DLManagedTensor
/// - The DLManagedTensor remains valid for the lifetime of the returned descriptor
/// - The deleter is called when done with the tensor
///
/// # Returns
///
/// A TensorDescriptor describing the tensor, or an error if the tensor
/// format is not supported.
pub unsafe fn from_dlpack(
    managed: *const DLManagedTensor,
) -> Result<TensorDescriptor, DLPackImportError> {
    if managed.is_null() {
        return Err(DLPackImportError::NullPointer);
    }

    // SAFETY: null check above. Caller guarantees `managed` points to a valid
    // DLManagedTensor per this function's documented safety requirements.
    let managed = &*managed;
    let tensor = &managed.dl_tensor;

    // Validate data pointer
    if tensor.data.is_null() {
        return Err(DLPackImportError::InvalidTensor(
            "data pointer is null".into(),
        ));
    }

    // Convert device
    let device = device_from_dlpack(&tensor.device)?;

    // Convert dtype
    let dtype = dtype_from_dlpack(tensor.dtype.code, tensor.dtype.bits, tensor.dtype.lanes)?;

    // Extract shape
    if tensor.shape.is_null() && tensor.ndim > 0 {
        return Err(DLPackImportError::InvalidTensor(
            "shape pointer is null".into(),
        ));
    }

    // Validate ndim is non-negative and within reasonable bounds before casting to usize
    if tensor.ndim < 0 || tensor.ndim > 64 {
        return Err(DLPackImportError::InvalidTensor(format!(
            "invalid ndim: {} (must be 0..=64)",
            tensor.ndim
        )));
    }
    let ndim = tensor.ndim as usize;

    let shape: Vec<u64> = if ndim > 0 {
        // SAFETY: shape is non-null (checked above for ndim > 0), and per DLPack
        // spec it points to `ndim` contiguous i64 values owned by the producer.
        let raw_shape = std::slice::from_raw_parts(tensor.shape, ndim);
        let mut shape = Vec::with_capacity(ndim);
        for (i, &x) in raw_shape.iter().enumerate() {
            if x < 0 {
                return Err(DLPackImportError::InvalidTensor(format!(
                    "negative shape[{}] = {}",
                    i, x
                )));
            }
            shape.push(x as u64);
        }
        shape
    } else {
        vec![]
    };

    // Extract strides (convert from elements to bytes if present)
    let strides: Vec<u64> = if tensor.strides.is_null() {
        // Compute default row-major strides; may return Err on overflow.
        compute_contiguous_strides(&shape, dtype.size_bytes())?
    } else {
        // Convert element strides to byte strides
        let elem_size = dtype.size_bytes() as u64;
        // SAFETY: strides is non-null (else branch), and per DLPack spec it
        // points to `ndim` contiguous i64 values owned by the producer.
        let raw_strides = std::slice::from_raw_parts(tensor.strides, ndim);
        let mut strides = Vec::with_capacity(ndim);
        for (i, &x) in raw_strides.iter().enumerate() {
            if x < 0 {
                return Err(DLPackImportError::InvalidTensor(format!(
                    "negative stride[{}] = {}",
                    i, x
                )));
            }
            strides.push((x as u64).checked_mul(elem_size).ok_or_else(|| {
                DLPackImportError::InvalidTensor(format!(
                    "stride[{}] byte conversion overflows u64", i
                ))
            })?);
        }
        strides
    };

    // Compute total size (checked to detect overflow from adversarial shapes)
    let num_elements: u64 = shape
        .iter()
        .try_fold(1u64, |acc, &dim| acc.checked_mul(dim))
        .ok_or_else(|| {
            DLPackImportError::InvalidTensor("shape product overflows u64".into())
        })?;
    let size_bytes = num_elements.checked_mul(dtype.size_bytes() as u64).ok_or_else(|| {
        DLPackImportError::InvalidTensor("size_bytes overflows u64".into())
    })?;

    // Compute actual data pointer (checked to detect wraparound from adversarial byte_offset)
    let data_ptr = (tensor.data as usize)
        .checked_add(tensor.byte_offset as usize)
        .ok_or_else(|| {
            DLPackImportError::InvalidTensor("data + byte_offset overflows pointer".into())
        })? as u64;

    Ok(TensorDescriptor {
        shape,
        strides,
        dtype,
        device,
        size_bytes,
        data_ptr,
    })
}

/// Convert DLPack device to HORUS Device
fn device_from_dlpack(dl_device: &DLDevice) -> Result<Device, DLPackImportError> {
    Device::from_dlpack(dl_device.device_type, dl_device.device_id)
        .ok_or(DLPackImportError::UnsupportedDevice(dl_device.device_type))
}

/// Convert DLPack dtype to HORUS TensorDtype
fn dtype_from_dlpack(code: u8, bits: u8, lanes: u16) -> Result<TensorDtype, DLPackImportError> {
    TensorDtype::from_dlpack(code, bits, lanes).ok_or(DLPackImportError::UnsupportedDtype {
        code,
        bits,
        lanes,
    })
}

/// Compute contiguous (row-major) strides in bytes.
///
/// Returns `Err(InvalidTensor)` if any stride multiplication overflows `u64`.
/// This prevents silent garbage stride values that would direct numpy/torch
/// to wrong memory addresses on very large shapes.
///
/// For scalar tensors (ndim == 0), returns `[0]` as a single-element sentinel.
fn compute_contiguous_strides(
    shape: &[u64],
    elem_size: usize,
) -> Result<Vec<u64>, DLPackImportError> {
    let ndim = shape.len();
    if ndim == 0 {
        // Scalar tensor: one zero stride (no real dimension to stride over).
        return Ok(vec![0]);
    }

    let mut strides = vec![0u64; ndim];
    strides[ndim - 1] = elem_size as u64;
    for i in (0..ndim - 1).rev() {
        strides[i] = strides[i + 1].checked_mul(shape[i + 1]).ok_or_else(|| {
            DLPackImportError::InvalidTensor(format!(
                "Shape too large: stride overflow at dimension {} \
                     (strides[{}]={} * shape[{}]={})",
                i,
                i + 1,
                strides[i + 1],
                i + 1,
                shape[i + 1]
            ))
        })?;
    }
    Ok(strides)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::dlpack::export::to_dlpack;
    use std::ffi::c_void;

    #[test]
    fn test_dlpack_roundtrip() {
        // Create a tensor and export to DLPack
        let data: Vec<f32> = vec![1.0, 2.0, 3.0, 4.0, 5.0, 6.0];
        let shape = vec![2i64, 3];
        let strides = vec![3i64, 1]; // Element strides

        let managed = to_dlpack(
            data.as_ptr() as *mut c_void,
            &shape,
            &strides,
            TensorDtype::F32,
            Device::cpu(),
        );

        // Import back
        // SAFETY: managed was just created by to_dlpack with valid data, shape, and strides.
        let descriptor = unsafe { from_dlpack(Box::into_raw(managed)).unwrap() };

        assert_eq!(descriptor.shape, vec![2, 3]);
        assert_eq!(descriptor.dtype, TensorDtype::F32);
        assert_eq!(descriptor.device, Device::cpu());
        assert_eq!(descriptor.size_bytes, 24); // 6 * 4 bytes

        // Strides should be in bytes: [12, 4] (3*4, 1*4)
        assert_eq!(descriptor.strides, vec![12, 4]);
    }

    #[test]
    fn test_dlpack_roundtrip_cuda_device() {
        // Verify that CUDA device info survives the export→import roundtrip.
        // Uses a CPU-hosted buffer but sets Device::cuda(2) to test device propagation.
        let data: Vec<f32> = vec![1.0, 2.0, 3.0, 4.0];
        let shape = vec![4i64];
        let strides = vec![1i64];

        let managed = to_dlpack(
            data.as_ptr() as *mut c_void,
            &shape,
            &strides,
            TensorDtype::F32,
            Device::cuda(2),
        );

        // SAFETY: managed was just created by to_dlpack with valid data, shape, and strides.
        let descriptor = unsafe { from_dlpack(Box::into_raw(managed)).unwrap() };

        assert_eq!(descriptor.device, Device::cuda(2));
        assert_eq!(descriptor.dtype, TensorDtype::F32);
        assert_eq!(descriptor.shape, vec![4]);
        assert_eq!(descriptor.size_bytes, 16); // 4 * 4 bytes
    }

    #[test]
    fn test_contiguous_strides() {
        let shape = vec![2u64, 3, 4];
        let strides = compute_contiguous_strides(&shape, 4).unwrap(); // f32

        // For [2, 3, 4] with f32:
        // stride[2] = 4 (bytes)
        // stride[1] = 4 * 4 = 16
        // stride[0] = 16 * 3 = 48
        assert_eq!(strides, vec![48, 16, 4]);
    }

    #[test]
    fn test_contiguous_strides_overflow_returns_err() {
        // shape[1] * elem_size * 2 overflows u64:
        //   strides[1] = elem_size = 2
        //   strides[0] = strides[1] * shape[1] = 2 * (u64::MAX/2 + 1) = overflow
        let shape = vec![2u64, u64::MAX / 2 + 1];
        let result = compute_contiguous_strides(&shape, 2);
        assert!(
            result.is_err(),
            "expected Err on stride overflow, got {:?}",
            result
        );
        let err_msg = result.unwrap_err().to_string();
        assert!(
            err_msg.contains("overflow"),
            "error should mention overflow: {}",
            err_msg
        );
    }

    #[test]
    fn test_contiguous_strides_scalar_returns_zero_stride() {
        // A 0-dim (scalar) tensor returns a single-element zero-stride sentinel.
        let strides = compute_contiguous_strides(&[], 4).unwrap();
        assert_eq!(strides, vec![0], "scalar tensor should have stride [0]");
    }
}
