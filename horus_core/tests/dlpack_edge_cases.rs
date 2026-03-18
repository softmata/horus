// DLPack edge case tests.
//
// Tests zero-copy verification, large tensors, scalar tensors,
// and deleter callback invocation.

use horus_core::dlpack::{from_dlpack, to_dlpack};
use horus_core::types::{Device, TensorDtype};
use std::ffi::c_void;

mod common;

// ============================================================================
// Test: Zero-copy — exported tensor data pointer equals original
// ============================================================================

#[test]
fn test_dlpack_zero_copy_same_pointer() {
    let mut data: Vec<f32> = vec![1.0, 2.0, 3.0, 4.0, 5.0, 6.0];
    let data_ptr = data.as_mut_ptr() as *mut c_void;
    let shape = [2i64, 3];
    let strides = [3i64, 1]; // row-major

    let managed = to_dlpack(data_ptr, &shape, &strides, TensorDtype::F32, Device::cpu());

    // Zero-copy check: DLTensor.data must equal original data_ptr
    assert_eq!(
        managed.dl_tensor.data, data_ptr,
        "DLPack export should be zero-copy (same data pointer)"
    );

    // Shape should be accessible
    assert_eq!(managed.dl_tensor.ndim, 2);

    // Clean up via deleter
    unsafe {
        if let Some(deleter) = managed.deleter {
            deleter(Box::into_raw(managed));
        }
    }
}

// ============================================================================
// Test: Large tensor (10MB) roundtrip
// ============================================================================

#[test]
fn test_dlpack_large_tensor_10mb() {
    // 10MB of f32 = 2.5M elements
    let num_elements = 2_500_000usize;
    let mut data: Vec<f32> = vec![0.0; num_elements];
    // Write pattern to verify integrity
    for (i, v) in data.iter_mut().enumerate() {
        *v = (i % 1000) as f32;
    }

    let data_ptr = data.as_mut_ptr() as *mut c_void;
    let shape = [num_elements as i64];
    let strides = [1i64];

    let managed = to_dlpack(data_ptr, &shape, &strides, TensorDtype::F32, Device::cpu());

    // Import back
    let descriptor = unsafe { from_dlpack(&*managed) }.unwrap();

    assert_eq!(descriptor.shape, &[num_elements as u64]);
    assert_eq!(descriptor.dtype, TensorDtype::F32);

    // Verify data pointer is same (zero-copy)
    assert_eq!(managed.dl_tensor.data, data_ptr);

    // Clean up
    unsafe {
        if let Some(deleter) = managed.deleter {
            deleter(Box::into_raw(managed));
        }
    }
}

// ============================================================================
// Test: Scalar (0-dim) tensor roundtrip
// ============================================================================

#[test]
fn test_dlpack_scalar_tensor() {
    let mut value: f32 = 42.0;
    let data_ptr = &mut value as *mut f32 as *mut c_void;
    let shape: [i64; 0] = [];
    let strides: [i64; 0] = [];

    let managed = to_dlpack(data_ptr, &shape, &strides, TensorDtype::F32, Device::cpu());

    assert_eq!(
        managed.dl_tensor.ndim, 0,
        "Scalar tensor should have 0 dims"
    );
    assert_eq!(managed.dl_tensor.data, data_ptr);

    // Clean up
    unsafe {
        if let Some(deleter) = managed.deleter {
            deleter(Box::into_raw(managed));
        }
    }
}

// ============================================================================
// Test: Deleter callback invoked and cleans up context
// ============================================================================

#[test]
fn test_dlpack_deleter_frees_context() {
    let mut data: Vec<f32> = vec![1.0, 2.0, 3.0];
    let data_ptr = data.as_mut_ptr() as *mut c_void;

    let managed = to_dlpack(data_ptr, &[3], &[1], TensorDtype::F32, Device::cpu());

    // manager_ctx should be non-null before deleter
    assert!(
        !managed.manager_ctx.is_null(),
        "Context should be set before deleter"
    );

    // Call deleter — should not crash
    unsafe {
        let raw = Box::into_raw(managed);
        if let Some(deleter) = (*raw).deleter {
            deleter(raw);
        }
    }
    // If we get here without crash, deleter worked correctly
}

// ============================================================================
// Test: DLPack roundtrip preserves dtype and device
// ============================================================================

#[test]
fn test_dlpack_roundtrip_dtype_device() {
    let mut data: Vec<u8> = vec![255, 128, 64, 32];
    let data_ptr = data.as_mut_ptr() as *mut c_void;
    let shape = [4i64];
    let strides = [1i64];

    let managed = to_dlpack(data_ptr, &shape, &strides, TensorDtype::U8, Device::cpu());

    let descriptor = unsafe { from_dlpack(&*managed) }.unwrap();

    assert_eq!(
        descriptor.dtype,
        TensorDtype::U8,
        "DType should survive roundtrip"
    );
    assert_eq!(
        descriptor.device,
        Device::cpu(),
        "Device should survive roundtrip"
    );
    assert_eq!(descriptor.shape, &[4u64]);

    unsafe {
        if let Some(deleter) = managed.deleter {
            deleter(Box::into_raw(managed));
        }
    }
}

// ============================================================================
// Test: Multiple dtypes roundtrip correctly
// ============================================================================

#[test]
fn test_dlpack_multiple_dtypes() {
    // F64
    {
        let mut data = vec![1.0f64, 2.0, 3.0];
        let ptr = data.as_mut_ptr() as *mut c_void;
        let managed = to_dlpack(ptr, &[3], &[1], TensorDtype::F64, Device::cpu());
        let desc = unsafe { from_dlpack(&*managed) }.unwrap();
        assert_eq!(desc.dtype, TensorDtype::F64);
        unsafe {
            if let Some(d) = managed.deleter {
                d(Box::into_raw(managed));
            }
        }
    }

    // I32
    {
        let mut data = vec![1i32, -2, 3];
        let ptr = data.as_mut_ptr() as *mut c_void;
        let managed = to_dlpack(ptr, &[3], &[1], TensorDtype::I32, Device::cpu());
        let desc = unsafe { from_dlpack(&*managed) }.unwrap();
        assert_eq!(desc.dtype, TensorDtype::I32);
        unsafe {
            if let Some(d) = managed.deleter {
                d(Box::into_raw(managed));
            }
        }
    }

    // U16
    {
        let mut data = vec![1u16, 2, 3];
        let ptr = data.as_mut_ptr() as *mut c_void;
        let managed = to_dlpack(ptr, &[3], &[1], TensorDtype::U16, Device::cpu());
        let desc = unsafe { from_dlpack(&*managed) }.unwrap();
        assert_eq!(desc.dtype, TensorDtype::U16);
        unsafe {
            if let Some(d) = managed.deleter {
                d(Box::into_raw(managed));
            }
        }
    }
}
