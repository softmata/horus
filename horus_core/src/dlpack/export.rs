//! DLPack export - convert HORUS tensors to DLPack format
//!
//! This module provides functionality to export HORUS tensors as DLPack
//! managed tensors for zero-copy sharing with other frameworks.

use std::ffi::c_void;

use super::ffi::{DLDataType, DLDevice, DLManagedTensor, DLTensor};
use crate::types::{Device, TensorDtype};

/// Context stored in DLManagedTensor::manager_ctx
///
/// Holds references to keep the underlying data alive until the consumer
/// calls the deleter.
struct DLPackContext {
    /// Shape array (owned, will be freed in deleter)
    shape: Vec<i64>,
    /// Strides array (owned, will be freed in deleter)
    strides: Vec<i64>,
}

/// Deleter callback for DLManagedTensor
///
/// This is called by the consumer when they're done with the tensor.
/// It frees the context and the DLManagedTensor itself.
unsafe extern "C" fn dlpack_deleter(managed: *mut DLManagedTensor) {
    if managed.is_null() {
        return;
    }

    // SAFETY: null check above. The pointer was created by `Box::into_raw` in
    // `to_dlpack` (or by the Python capsule path), so it is valid, aligned,
    // and was allocated via the global allocator.
    let managed = &mut *managed;

    // Free the context
    if !managed.manager_ctx.is_null() {
        // SAFETY: manager_ctx was created by `Box::into_raw(context)` in
        // `to_dlpack`, so reconstructing the Box reclaims ownership.
        let _ = Box::from_raw(managed.manager_ctx as *mut DLPackContext);
    }

    // SAFETY: `managed` was created by `Box::into_raw` in the export path.
    // Reconstructing the Box drops it and frees the allocation.
    let _ = Box::from_raw(managed as *mut DLManagedTensor);
}

/// Convert HORUS tensor information to DLPack format
///
/// Creates a DLManagedTensor that can be passed to other frameworks.
/// The returned tensor borrows the data - ensure the source data remains
/// valid until the consumer calls the deleter.
///
/// # Arguments
///
/// * `data_ptr` - Pointer to tensor data
/// * `shape` - Shape of the tensor
/// * `strides` - Strides in number of elements (not bytes)
/// * `dtype` - Data type
/// * `device` - Device where data resides
///
/// # Safety
///
/// The caller must ensure `data_ptr` remains valid until the returned
/// DLManagedTensor's deleter is called.
///
/// # Returns
///
/// A boxed DLManagedTensor that can be converted to a PyCapsule for Python.
pub fn to_dlpack(
    data_ptr: *mut c_void,
    shape: &[i64],
    strides_elements: &[i64],
    dtype: TensorDtype,
    device: Device,
) -> Box<DLManagedTensor> {
    // Convert dtype to DLPack format
    let (code, bits, lanes) = dtype.to_dlpack();
    let dl_dtype = DLDataType::new(code, bits, lanes);

    // Convert device to DLPack format
    let dl_device = DLDevice {
        device_type: device.to_dlpack_device_type(),
        device_id: device.to_dlpack_device_id(),
    };

    // Create owned copies of shape and strides
    let shape_vec: Vec<i64> = shape.to_vec();
    let strides_vec: Vec<i64> = strides_elements.to_vec();

    // Create context to hold owned data
    let context = Box::new(DLPackContext {
        shape: shape_vec,
        strides: strides_vec,
    });

    // Get raw pointers from context (context keeps them alive)
    let shape_ptr = context.shape.as_ptr() as *mut i64;
    let strides_ptr = if context.strides.is_empty() {
        std::ptr::null_mut()
    } else {
        context.strides.as_ptr() as *mut i64
    };

    // Create DLTensor
    let dl_tensor = DLTensor {
        data: data_ptr,
        device: dl_device,
        ndim: shape.len() as i32,
        dtype: dl_dtype,
        shape: shape_ptr,
        strides: strides_ptr,
        byte_offset: 0,
    };

    // Create managed tensor
    let context_ptr = Box::into_raw(context) as *mut c_void;

    Box::new(DLManagedTensor {
        dl_tensor,
        manager_ctx: context_ptr,
        deleter: Some(dlpack_deleter),
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_to_dlpack_cpu() {
        let data: Vec<f32> = vec![1.0, 2.0, 3.0, 4.0, 5.0, 6.0];
        let shape = vec![2i64, 3];
        let strides = vec![3i64, 1]; // Row-major strides in elements

        let managed = to_dlpack(
            data.as_ptr() as *mut c_void,
            &shape,
            &strides,
            TensorDtype::F32,
            Device::cpu(),
        );

        // Verify tensor properties
        assert_eq!(managed.dl_tensor.ndim, 2);
        assert!(managed.dl_tensor.device.is_cpu());
        assert_eq!(managed.dl_tensor.dtype.code, 2); // float
        assert_eq!(managed.dl_tensor.dtype.bits, 32);

        // Verify shape
        // SAFETY: shape pointer is valid for ndim (2) elements, allocated by to_dlpack.
        unsafe {
            assert_eq!(*managed.dl_tensor.shape, 2);
            assert_eq!(*managed.dl_tensor.shape.add(1), 3);
        }

        // Call deleter to clean up
        if let Some(deleter) = managed.deleter {
            // SAFETY: managed was created by to_dlpack and has not been freed yet.
            unsafe { deleter(Box::into_raw(managed)) };
        }
    }

    #[test]
    fn test_to_dlpack_cuda() {
        let data: Vec<f32> = vec![1.0, 2.0, 3.0, 4.0];
        let shape = vec![4i64];
        let strides = vec![1i64];

        let managed = to_dlpack(
            data.as_ptr() as *mut c_void,
            &shape,
            &strides,
            TensorDtype::F32,
            Device::cuda(1),
        );

        assert!(managed.dl_tensor.device.is_cuda());
        assert_eq!(managed.dl_tensor.device.device_id, 1);

        if let Some(deleter) = managed.deleter {
            // SAFETY: managed was created by to_dlpack and has not been freed yet.
            unsafe { deleter(Box::into_raw(managed)) };
        }
    }
}
