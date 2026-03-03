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
/// This is called by the consumer when they're done with the tensor, or by the
/// PyCapsule destructor for unclaimed tensors.
///
/// It frees the context and the DLManagedTensor itself.
///
/// # Double-free safety
///
/// `manager_ctx` is nulled out immediately before the context `Box` is dropped.
/// If this function is somehow called a second time (caller bug or reference
/// cycle), the null-check on `manager_ctx` prevents a double-free of the
/// context.  The `DLManagedTensor` itself is still freed once; the primary
/// protection against double-free of `managed` is [`dlpack_capsule_destructor`]
/// nulling the capsule pointer before invoking this deleter.
unsafe extern "C" fn dlpack_deleter(managed: *mut DLManagedTensor) {
    if managed.is_null() {
        return;
    }

    // SAFETY: null check above. The pointer was created by `Box::into_raw` in
    // `to_dlpack` (or by the Python capsule path), so it is valid, aligned,
    // and was allocated via the global allocator.
    let managed_ref = &mut *managed;

    // Free the context — null it out first so a hypothetical second invocation
    // skips this block instead of reconstructing a dangling Box.
    if !managed_ref.manager_ctx.is_null() {
        let ctx_ptr = managed_ref.manager_ctx as *mut DLPackContext;
        // Null before drop: any second call sees null and skips.
        managed_ref.manager_ctx = std::ptr::null_mut();
        // SAFETY: ctx_ptr was created by `Box::into_raw(context)` in `to_dlpack`.
        let _ = Box::from_raw(ctx_ptr);
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

    /// dlpack_deleter must not crash when manager_ctx is already null.
    ///
    /// This simulates the state after a first cleanup call has already freed
    /// and nulled the context: a second invocation (caller bug or GC anomaly)
    /// should return safely without a double-free.
    ///
    /// The primary guard is `dlpack_capsule_destructor` nulling the capsule
    /// pointer; this test verifies the secondary defense inside `dlpack_deleter`.
    #[test]
    fn test_dlpack_deleter_null_context_is_safe() {
        let data: Vec<f32> = vec![1.0, 2.0, 3.0, 4.0];
        let managed = to_dlpack(
            data.as_ptr() as *mut c_void,
            &[4i64],
            &[1i64],
            TensorDtype::F32,
            Device::cpu(),
        );
        let managed_ptr = Box::into_raw(managed);

        // Simulate state after a first cleanup: free the context via the deleter
        // mechanism but leave manager_ctx as null (the deleter nulls it before
        // freeing, so on a second call it would see null).
        //
        // We achieve this by calling the deleter once — which frees both the
        // context and the managed tensor — but we verify the null-ctx guard
        // separately below with a fresh allocation.
        //
        // Fresh allocation to test the null-manager_ctx guard path directly:
        let managed2 = to_dlpack(
            data.as_ptr() as *mut c_void,
            &[4i64],
            &[1i64],
            TensorDtype::F32,
            Device::cpu(),
        );
        let managed2_ptr = Box::into_raw(managed2);

        unsafe {
            // Manually free the context (simulate what the first deleter call does)
            // and null manager_ctx to represent "already cleaned up".
            let ctx_ptr = (*managed2_ptr).manager_ctx as *mut DLPackContext;
            (*managed2_ptr).manager_ctx = std::ptr::null_mut();
            let _ = Box::from_raw(ctx_ptr); // free context

            // Call deleter with manager_ctx=null — must not crash or double-free.
            if let Some(deleter) = (*managed2_ptr).deleter {
                deleter(managed2_ptr); // frees managed2_ptr; context skip is the guard
            }
            // Reaching here without a crash or MIRI error means the guard works.
        }

        // Clean up the first allocation normally.
        unsafe {
            if let Some(deleter) = (*managed_ptr).deleter {
                deleter(managed_ptr);
            }
        }
    }

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
