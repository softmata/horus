#![allow(dead_code)]
//! DLPack coverage tests — comprehensive export/import/error path testing.
//!
//! Covers all untested paths in the DLPack module:
//! - Multi-dtype roundtrips (I8, U8, I32, F64, etc.)
//! - High-dimensional tensors (3D, 4D, 5D)
//! - Scalar (0-dim) tensor roundtrip
//! - Error paths: null pointer, null data, null shape, negative shape/strides
//! - Unsupported device/dtype errors
//! - Byte offset handling
//! - Overflow detection (adversarial shapes/strides)

use horus_core::dlpack::{from_dlpack, to_dlpack, DLDataType, DLDevice, DLManagedTensor, DLTensor};
use horus_core::types::{Device, TensorDtype};
use std::ffi::c_void;

// ============================================================================
// Multi-dtype roundtrips
// ============================================================================

fn roundtrip_dtype(
    data: &[u8],
    shape: &[i64],
    strides: &[i64],
    dtype: TensorDtype,
    elem_size: usize,
) {
    let managed = to_dlpack(
        data.as_ptr() as *mut c_void,
        shape,
        strides,
        dtype,
        Device::cpu(),
    );
    let desc = unsafe { from_dlpack(Box::into_raw(managed)).unwrap() };
    assert_eq!(desc.dtype, dtype);
    let expected_elements: u64 = shape.iter().map(|&s| s as u64).product();
    assert_eq!(desc.size_bytes, expected_elements * elem_size as u64);
}

#[test]
fn roundtrip_f32() {
    let data: Vec<f32> = vec![1.0, 2.0, 3.0, 4.0];
    roundtrip_dtype(
        unsafe { std::slice::from_raw_parts(data.as_ptr() as *const u8, 16) },
        &[4],
        &[1],
        TensorDtype::F32,
        4,
    );
}

#[test]
fn roundtrip_f64() {
    let data: Vec<f64> = vec![1.0, 2.0, 3.0];
    roundtrip_dtype(
        unsafe { std::slice::from_raw_parts(data.as_ptr() as *const u8, 24) },
        &[3],
        &[1],
        TensorDtype::F64,
        8,
    );
}

#[test]
fn roundtrip_i32() {
    let data: Vec<i32> = vec![10, 20, 30, 40, 50, 60];
    roundtrip_dtype(
        unsafe { std::slice::from_raw_parts(data.as_ptr() as *const u8, 24) },
        &[2, 3],
        &[3, 1],
        TensorDtype::I32,
        4,
    );
}

#[test]
fn roundtrip_u8() {
    let data: Vec<u8> = vec![0, 128, 255, 1, 2, 3, 4, 5];
    roundtrip_dtype(&data, &[2, 4], &[4, 1], TensorDtype::U8, 1);
}

#[test]
fn roundtrip_i8() {
    let data: Vec<i8> = vec![-128, 0, 127, 1];
    roundtrip_dtype(
        unsafe { std::slice::from_raw_parts(data.as_ptr() as *const u8, 4) },
        &[4],
        &[1],
        TensorDtype::I8,
        1,
    );
}

#[test]
fn roundtrip_i64() {
    let data: Vec<i64> = vec![i64::MIN, 0, i64::MAX];
    roundtrip_dtype(
        unsafe { std::slice::from_raw_parts(data.as_ptr() as *const u8, 24) },
        &[3],
        &[1],
        TensorDtype::I64,
        8,
    );
}

// ============================================================================
// High-dimensional tensors
// ============================================================================

#[test]
fn roundtrip_3d_tensor() {
    let data: Vec<f32> = vec![0.0; 2 * 3 * 4];
    let managed = to_dlpack(
        data.as_ptr() as *mut c_void,
        &[2, 3, 4],
        &[12, 4, 1],
        TensorDtype::F32,
        Device::cpu(),
    );
    let desc = unsafe { from_dlpack(Box::into_raw(managed)).unwrap() };
    assert_eq!(desc.shape, vec![2, 3, 4]);
    assert_eq!(desc.size_bytes, 2 * 3 * 4 * 4); // 96 bytes
}

#[test]
fn roundtrip_4d_tensor_batch_chw() {
    // Batch x Channel x Height x Width — common in vision
    let data: Vec<f32> = vec![0.0; 2 * 3 * 8 * 8];
    let managed = to_dlpack(
        data.as_ptr() as *mut c_void,
        &[2, 3, 8, 8],
        &[192, 64, 8, 1],
        TensorDtype::F32,
        Device::cpu(),
    );
    let desc = unsafe { from_dlpack(Box::into_raw(managed)).unwrap() };
    assert_eq!(desc.shape, vec![2, 3, 8, 8]);
    assert_eq!(desc.size_bytes, 2 * 3 * 8 * 8 * 4);
}

#[test]
fn roundtrip_5d_tensor() {
    let data: Vec<f32> = vec![0.0; 2 * 3 * 4 * 5 * 6];
    let managed = to_dlpack(
        data.as_ptr() as *mut c_void,
        &[2, 3, 4, 5, 6],
        &[360, 120, 30, 6, 1],
        TensorDtype::F32,
        Device::cpu(),
    );
    let desc = unsafe { from_dlpack(Box::into_raw(managed)).unwrap() };
    assert_eq!(desc.shape, vec![2, 3, 4, 5, 6]);
}

// ============================================================================
// Scalar (0-dim) tensor
// ============================================================================

#[test]
fn roundtrip_scalar_tensor() {
    let data: Vec<f32> = vec![42.0];
    let managed = to_dlpack(
        data.as_ptr() as *mut c_void,
        &[], // 0-dim
        &[], // no strides
        TensorDtype::F32,
        Device::cpu(),
    );
    let desc = unsafe { from_dlpack(Box::into_raw(managed)).unwrap() };
    assert_eq!(desc.shape, Vec::<u64>::new());
    assert_eq!(desc.size_bytes, 4); // 1 element * 4 bytes
}

// ============================================================================
// 1-element tensor (different from scalar)
// ============================================================================

#[test]
fn roundtrip_single_element_tensor() {
    let data: Vec<f32> = vec![2.75];
    let managed = to_dlpack(
        data.as_ptr() as *mut c_void,
        &[1],
        &[1],
        TensorDtype::F32,
        Device::cpu(),
    );
    let desc = unsafe { from_dlpack(Box::into_raw(managed)).unwrap() };
    assert_eq!(desc.shape, vec![1]);
    assert_eq!(desc.size_bytes, 4);
}

// ============================================================================
// Error paths: from_dlpack
// ============================================================================

#[test]
fn from_dlpack_null_pointer_returns_error() {
    let result = unsafe { from_dlpack(std::ptr::null()) };
    assert!(result.is_err());
    let err = result.unwrap_err().to_string();
    assert!(err.contains("null"), "Error should mention null: {}", err);
}

#[test]
fn from_dlpack_null_data_returns_error() {
    let mut shape_val: i64 = 4;
    let mut stride_val: i64 = 1;
    let managed = DLManagedTensor {
        dl_tensor: DLTensor {
            data: std::ptr::null_mut(), // null data
            device: DLDevice::cpu(),
            ndim: 1,
            dtype: DLDataType::F32,
            shape: &mut shape_val as *mut i64,
            strides: &mut stride_val as *mut i64,
            byte_offset: 0,
        },
        manager_ctx: std::ptr::null_mut(),
        deleter: None,
    };
    let result = unsafe { from_dlpack(&managed) };
    assert!(result.is_err());
    assert!(result.unwrap_err().to_string().contains("null"));
}

#[test]
fn from_dlpack_null_shape_with_ndim_returns_error() {
    let data: Vec<f32> = vec![1.0];
    let managed = DLManagedTensor {
        dl_tensor: DLTensor {
            data: data.as_ptr() as *mut c_void,
            device: DLDevice::cpu(),
            ndim: 2, // ndim > 0 but shape is null
            dtype: DLDataType::F32,
            shape: std::ptr::null_mut(),
            strides: std::ptr::null_mut(),
            byte_offset: 0,
        },
        manager_ctx: std::ptr::null_mut(),
        deleter: None,
    };
    let result = unsafe { from_dlpack(&managed) };
    result.unwrap_err();
}

#[test]
fn from_dlpack_negative_ndim_returns_error() {
    let data: Vec<f32> = vec![1.0];
    let managed = DLManagedTensor {
        dl_tensor: DLTensor {
            data: data.as_ptr() as *mut c_void,
            device: DLDevice::cpu(),
            ndim: -1,
            dtype: DLDataType::F32,
            shape: std::ptr::null_mut(),
            strides: std::ptr::null_mut(),
            byte_offset: 0,
        },
        manager_ctx: std::ptr::null_mut(),
        deleter: None,
    };
    let result = unsafe { from_dlpack(&managed) };
    assert!(result.is_err());
    assert!(result.unwrap_err().to_string().contains("ndim"));
}

#[test]
fn from_dlpack_excessive_ndim_returns_error() {
    let data: Vec<f32> = vec![1.0];
    let managed = DLManagedTensor {
        dl_tensor: DLTensor {
            data: data.as_ptr() as *mut c_void,
            device: DLDevice::cpu(),
            ndim: 65, // > 64 limit
            dtype: DLDataType::F32,
            shape: std::ptr::null_mut(),
            strides: std::ptr::null_mut(),
            byte_offset: 0,
        },
        manager_ctx: std::ptr::null_mut(),
        deleter: None,
    };
    let result = unsafe { from_dlpack(&managed) };
    result.unwrap_err();
}

#[test]
fn from_dlpack_negative_shape_returns_error() {
    let data: Vec<f32> = vec![1.0, 2.0];
    let mut shape: Vec<i64> = vec![2, -3]; // negative shape
    let managed = DLManagedTensor {
        dl_tensor: DLTensor {
            data: data.as_ptr() as *mut c_void,
            device: DLDevice::cpu(),
            ndim: 2,
            dtype: DLDataType::F32,
            shape: shape.as_mut_ptr(),
            strides: std::ptr::null_mut(),
            byte_offset: 0,
        },
        manager_ctx: std::ptr::null_mut(),
        deleter: None,
    };
    let result = unsafe { from_dlpack(&managed) };
    assert!(result.is_err());
    assert!(result.unwrap_err().to_string().contains("negative"));
}

#[test]
fn from_dlpack_negative_stride_returns_error() {
    let data: Vec<f32> = vec![1.0, 2.0, 3.0, 4.0];
    let mut shape: Vec<i64> = vec![2, 2];
    let mut strides: Vec<i64> = vec![2, -1]; // negative stride
    let managed = DLManagedTensor {
        dl_tensor: DLTensor {
            data: data.as_ptr() as *mut c_void,
            device: DLDevice::cpu(),
            ndim: 2,
            dtype: DLDataType::F32,
            shape: shape.as_mut_ptr(),
            strides: strides.as_mut_ptr(),
            byte_offset: 0,
        },
        manager_ctx: std::ptr::null_mut(),
        deleter: None,
    };
    let result = unsafe { from_dlpack(&managed) };
    assert!(result.is_err());
    assert!(result.unwrap_err().to_string().contains("negative"));
}

#[test]
fn from_dlpack_unsupported_device_returns_error() {
    let data: Vec<f32> = vec![1.0];
    let mut shape: i64 = 1;
    let managed = DLManagedTensor {
        dl_tensor: DLTensor {
            data: data.as_ptr() as *mut c_void,
            device: DLDevice {
                device_type: 999,
                device_id: 0,
            }, // unknown device
            ndim: 1,
            dtype: DLDataType::F32,
            shape: &mut shape,
            strides: std::ptr::null_mut(),
            byte_offset: 0,
        },
        manager_ctx: std::ptr::null_mut(),
        deleter: None,
    };
    let result = unsafe { from_dlpack(&managed) };
    assert!(result.is_err());
    let err_msg = format!("{}", result.unwrap_err());
    assert!(
        err_msg.contains("device") || err_msg.contains("Unsupported"),
        "Error: {}",
        err_msg
    );
}

#[test]
fn from_dlpack_unsupported_dtype_returns_error() {
    let data: Vec<u8> = vec![0];
    let mut shape: i64 = 1;
    let managed = DLManagedTensor {
        dl_tensor: DLTensor {
            data: data.as_ptr() as *mut c_void,
            device: DLDevice::cpu(),
            ndim: 1,
            dtype: DLDataType::new(7, 128, 1), // unknown dtype code
            shape: &mut shape,
            strides: std::ptr::null_mut(),
            byte_offset: 0,
        },
        manager_ctx: std::ptr::null_mut(),
        deleter: None,
    };
    let result = unsafe { from_dlpack(&managed) };
    result.unwrap_err();
}

#[test]
fn from_dlpack_shape_overflow_returns_error() {
    let data: Vec<f32> = vec![1.0];
    let mut shape: Vec<i64> = vec![i64::MAX / 2, i64::MAX / 2]; // product overflows u64
    let managed = DLManagedTensor {
        dl_tensor: DLTensor {
            data: data.as_ptr() as *mut c_void,
            device: DLDevice::cpu(),
            ndim: 2,
            dtype: DLDataType::F32,
            shape: shape.as_mut_ptr(),
            strides: std::ptr::null_mut(),
            byte_offset: 0,
        },
        manager_ctx: std::ptr::null_mut(),
        deleter: None,
    };
    let result = unsafe { from_dlpack(&managed) };
    assert!(result.is_err());
    assert!(result.unwrap_err().to_string().contains("overflow"));
}

// ============================================================================
// Null strides (contiguous) path
// ============================================================================

#[test]
fn from_dlpack_null_strides_computes_contiguous() {
    let data: Vec<f32> = vec![0.0; 6];
    let mut shape: Vec<i64> = vec![2, 3];
    let managed = DLManagedTensor {
        dl_tensor: DLTensor {
            data: data.as_ptr() as *mut c_void,
            device: DLDevice::cpu(),
            ndim: 2,
            dtype: DLDataType::F32,
            shape: shape.as_mut_ptr(),
            strides: std::ptr::null_mut(), // null → contiguous
            byte_offset: 0,
        },
        manager_ctx: std::ptr::null_mut(),
        deleter: None,
    };
    let desc = unsafe { from_dlpack(&managed).unwrap() };
    // Row-major byte strides: [3*4=12, 1*4=4]
    assert_eq!(desc.strides, vec![12, 4]);
}

// ============================================================================
// Byte offset handling
// ============================================================================

#[test]
fn from_dlpack_nonzero_byte_offset() {
    let data: Vec<f32> = vec![0.0; 10];
    let mut shape: i64 = 8;
    let mut stride: i64 = 1;
    let base_ptr = data.as_ptr() as usize;
    let managed = DLManagedTensor {
        dl_tensor: DLTensor {
            data: data.as_ptr() as *mut c_void,
            device: DLDevice::cpu(),
            ndim: 1,
            dtype: DLDataType::F32,
            shape: &mut shape,
            strides: &mut stride,
            byte_offset: 8, // skip first 2 floats
        },
        manager_ctx: std::ptr::null_mut(),
        deleter: None,
    };
    let desc = unsafe { from_dlpack(&managed).unwrap() };
    assert_eq!(desc.data_ptr, (base_ptr + 8) as u64);
}

// ============================================================================
// DLDataType constants
// ============================================================================

#[test]
fn dl_data_type_constants_correct() {
    assert_eq!(DLDataType::F32, DLDataType::new(2, 32, 1));
    assert_eq!(DLDataType::F64, DLDataType::new(2, 64, 1));
    assert_eq!(DLDataType::I8, DLDataType::new(0, 8, 1));
    assert_eq!(DLDataType::I32, DLDataType::new(0, 32, 1));
    assert_eq!(DLDataType::I64, DLDataType::new(0, 64, 1));
    assert_eq!(DLDataType::U8, DLDataType::new(1, 8, 1));
    assert_eq!(DLDataType::U16, DLDataType::new(1, 16, 1));
    assert_eq!(DLDataType::BOOL, DLDataType::new(6, 8, 1));
}

// ============================================================================
// DLDevice equality and Debug
// ============================================================================

#[test]
fn dl_device_equality() {
    assert_eq!(DLDevice::cpu(), DLDevice::cpu());
    assert_ne!(DLDevice::cpu(), DLDevice::cuda(0));
    assert_eq!(DLDevice::cuda(1), DLDevice::cuda(1));
    assert_ne!(DLDevice::cuda(0), DLDevice::cuda(1));
}

#[test]
fn dl_device_debug_format() {
    let debug = format!("{:?}", DLDevice::cpu());
    assert!(debug.contains("device_type"));
}

// ============================================================================
// Deleter safety
// ============================================================================

#[test]
fn deleter_null_managed_is_noop() {
    // Calling the deleter with null should not crash
    let data: Vec<f32> = vec![1.0];
    let managed = to_dlpack(
        data.as_ptr() as *mut c_void,
        &[1],
        &[1],
        TensorDtype::F32,
        Device::cpu(),
    );
    // Extract deleter, then properly clean up managed
    let deleter = managed.deleter.unwrap();
    let ptr = Box::into_raw(managed);
    // Call on the real pointer (proper cleanup)
    unsafe { deleter(ptr) };
    // Call on null (should be a no-op)
    unsafe { deleter(std::ptr::null_mut()) };
}
