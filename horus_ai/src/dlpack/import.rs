//! DLPack import - convert DLPack tensors to HORUS format
//!
//! This module provides functionality to import DLPack managed tensors
//! from other frameworks (PyTorch, JAX, TensorFlow, etc.) into HORUS.

use super::ffi::{DLDevice, DLManagedTensor};
use crate::device::Device;
use crate::tensor::{TensorDescriptor, TensorDtype};
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

    let shape: Vec<u64> = if tensor.ndim > 0 {
        std::slice::from_raw_parts(tensor.shape, tensor.ndim as usize)
            .iter()
            .map(|&x| x as u64)
            .collect()
    } else {
        vec![]
    };

    // Extract strides (convert from elements to bytes if present)
    let strides: Vec<u64> = if tensor.strides.is_null() {
        // Compute default row-major strides
        compute_contiguous_strides(&shape, dtype.size_bytes())
    } else {
        // Convert element strides to byte strides
        let elem_size = dtype.size_bytes() as u64;
        std::slice::from_raw_parts(tensor.strides, tensor.ndim as usize)
            .iter()
            .map(|&x| (x as u64) * elem_size)
            .collect()
    };

    // Compute total size
    let num_elements: u64 = shape.iter().product();
    let size_bytes = num_elements * dtype.size_bytes() as u64;

    // Compute actual data pointer (accounting for byte_offset)
    let data_ptr = (tensor.data as usize + tensor.byte_offset as usize) as u64;

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

/// Compute contiguous (row-major) strides in bytes
fn compute_contiguous_strides(shape: &[u64], elem_size: usize) -> Vec<u64> {
    let ndim = shape.len();
    if ndim == 0 {
        return vec![];
    }

    let mut strides = vec![0u64; ndim];
    strides[ndim - 1] = elem_size as u64;
    for i in (0..ndim - 1).rev() {
        strides[i] = strides[i + 1] * shape[i + 1];
    }
    strides
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
            Device::Cpu,
        );

        // Import back
        // SAFETY: managed was just created by to_dlpack with valid data, shape, and strides.
        let descriptor = unsafe { from_dlpack(Box::into_raw(managed)).unwrap() };

        assert_eq!(descriptor.shape, vec![2, 3]);
        assert_eq!(descriptor.dtype, TensorDtype::F32);
        assert_eq!(descriptor.device, Device::Cpu);
        assert_eq!(descriptor.size_bytes, 24); // 6 * 4 bytes

        // Strides should be in bytes: [12, 4] (3*4, 1*4)
        assert_eq!(descriptor.strides, vec![12, 4]);
    }

    #[test]
    fn test_contiguous_strides() {
        let shape = vec![2u64, 3, 4];
        let strides = compute_contiguous_strides(&shape, 4); // f32

        // For [2, 3, 4] with f32:
        // stride[2] = 4 (bytes)
        // stride[1] = 4 * 4 = 16
        // stride[0] = 16 * 3 = 48
        assert_eq!(strides, vec![48, 16, 4]);
    }
}
