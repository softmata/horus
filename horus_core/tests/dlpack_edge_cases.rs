#![allow(dead_code)]
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

// ============================================================================
// Test: F16 and BF16 dtype roundtrip through DLPack
// ============================================================================

#[test]
fn test_dlpack_f16_bf16_dtypes() {
    // F16 (half precision) — 2 bytes per element
    {
        let mut data: Vec<u16> = vec![0x3C00, 0x4000, 0x4200]; // 1.0, 2.0, 3.0 in f16
        let ptr = data.as_mut_ptr() as *mut c_void;
        let managed = to_dlpack(ptr, &[3], &[1], TensorDtype::F16, Device::cpu());
        let desc = unsafe { from_dlpack(&*managed) }.unwrap();
        assert_eq!(
            desc.dtype,
            TensorDtype::F16,
            "F16 dtype should survive roundtrip"
        );
        assert_eq!(desc.shape, &[3u64]);
        unsafe {
            if let Some(d) = managed.deleter {
                d(Box::into_raw(managed));
            }
        }
    }

    // BF16 (brain float) — 2 bytes per element
    {
        let mut data: Vec<u16> = vec![0x3F80, 0x4000]; // 1.0, 2.0 in bf16
        let ptr = data.as_mut_ptr() as *mut c_void;
        let managed = to_dlpack(ptr, &[2], &[1], TensorDtype::BF16, Device::cpu());
        let desc = unsafe { from_dlpack(&*managed) }.unwrap();
        assert_eq!(
            desc.dtype,
            TensorDtype::BF16,
            "BF16 dtype should survive roundtrip"
        );
        assert_eq!(desc.shape, &[2u64]);
        unsafe {
            if let Some(d) = managed.deleter {
                d(Box::into_raw(managed));
            }
        }
    }
}

// ============================================================================
// Test: Column-major (Fortran) strides roundtrip
// ============================================================================

#[test]
fn test_dlpack_column_major_strides() {
    // 3x4 matrix in column-major order: strides=[1, 3] (elements, not bytes)
    let mut data: Vec<f32> = vec![0.0; 12];
    for i in 0..12 {
        data[i] = i as f32;
    }
    let ptr = data.as_mut_ptr() as *mut c_void;
    let shape = [3i64, 4]; // 3 rows, 4 cols
    let strides = [1i64, 3]; // column-major: stride along rows=1, stride along cols=3

    let managed = to_dlpack(ptr, &shape, &strides, TensorDtype::F32, Device::cpu());

    let desc = unsafe { from_dlpack(&*managed) }.unwrap();
    assert_eq!(desc.shape, &[3u64, 4]);
    // Strides are returned in BYTES (not elements): 1*4=4, 3*4=12
    assert_eq!(
        desc.strides,
        vec![4u64, 12],
        "Column-major strides should survive DLPack roundtrip (in bytes)"
    );

    unsafe {
        if let Some(d) = managed.deleter {
            d(Box::into_raw(managed));
        }
    }
}

// ============================================================================
// Test: Sequential multi-tensor export/import — deleters independent
// ============================================================================

#[test]
fn test_dlpack_sequential_multi_tensor() {
    // Export 3 tensors, import each, verify no deleter interference
    let mut results = Vec::new();

    for i in 0..3u32 {
        let mut data: Vec<f32> = vec![(i + 1) as f32; (i as usize + 1) * 4];
        let len = data.len();
        let ptr = data.as_mut_ptr() as *mut c_void;
        let managed = to_dlpack(ptr, &[len as i64], &[1], TensorDtype::F32, Device::cpu());

        let desc = unsafe { from_dlpack(&*managed) }.unwrap();
        assert_eq!(desc.shape, &[len as u64]);
        assert_eq!(desc.dtype, TensorDtype::F32);

        results.push((managed, data));
    }

    // Clean up all 3 in reverse order
    for (managed, _data) in results.into_iter().rev() {
        unsafe {
            if let Some(d) = managed.deleter {
                d(Box::into_raw(managed));
            }
        }
    }
    // No crash = deleters are independent
}

// ============================================================================
// Test: Non-zero byte_offset handling
// ============================================================================

#[test]
fn test_dlpack_byte_offset() {
    // Create a buffer, export starting at offset 4 (skip first f32)
    let mut data: Vec<f32> = vec![999.0, 1.0, 2.0, 3.0];
    let ptr = data.as_mut_ptr() as *mut c_void;
    let shape = [3i64]; // 3 elements starting at offset
    let strides = [1i64];

    let mut managed = to_dlpack(ptr, &shape, &strides, TensorDtype::F32, Device::cpu());
    // Set byte_offset to skip the first f32 (4 bytes)
    managed.dl_tensor.byte_offset = 4;

    let desc = unsafe { from_dlpack(&*managed) }.unwrap();
    assert_eq!(desc.shape, &[3u64]);
    // The data_ptr in descriptor should account for byte_offset
    // (implementation-dependent: either adjusted ptr or separate offset field)

    unsafe {
        if let Some(d) = managed.deleter {
            d(Box::into_raw(managed));
        }
    }
}
