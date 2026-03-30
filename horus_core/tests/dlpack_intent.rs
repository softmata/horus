#![allow(dead_code)]
//! Level 7 Intent Tests — DLPack Tensor Exchange
//!
//! These tests verify **behavioral intent** of the DLPack zero-copy tensor
//! exchange protocol, not implementation details. Each test documents WHY
//! a behavior matters and what user-visible guarantee it protects.
//!
//! Covers:
//! - Export/import roundtrip preserves exact data
//! - Shape and dtype preserved across roundtrip
//! - CPU device metadata correct
//! - Zero-copy verified (data pointer unchanged)

use horus_core::dlpack::{from_dlpack, to_dlpack};
use horus_core::types::{Device, TensorDtype};
use std::ffi::c_void;

// ============================================================================
// Test 1: Export -> import preserves exact data
// ============================================================================

/// INTENT: "Export -> import preserves exact data."
///
/// ML/robotics guarantee: when a perception pipeline exports a tensor
/// (e.g., camera image, point cloud) to a DLPack consumer (PyTorch, JAX),
/// the consumer must receive the exact same bytes. Any data corruption
/// during the exchange means the model receives wrong inputs, producing
/// dangerous control decisions.
#[test]
fn test_dlpack_intent_roundtrip_preserves_data() {
    let original_data: Vec<f32> = vec![1.0, 2.5, -2.75, 42.0, 0.0, 99.9];
    let shape = [2i64, 3];
    let strides = [3i64, 1]; // Row-major element strides

    let managed = to_dlpack(
        original_data.as_ptr() as *mut c_void,
        &shape,
        &strides,
        TensorDtype::F32,
        Device::cpu(),
    );

    // Import back and verify the data pointer points to the same memory
    let managed_ptr = Box::into_raw(managed);
    // SAFETY: managed_ptr was just created by to_dlpack with valid data.
    let descriptor = unsafe { from_dlpack(managed_ptr).unwrap() };

    // The descriptor's data_ptr should point to the original data.
    // Read back through the pointer and verify byte-for-byte equality.
    let data_ptr = descriptor.data_ptr as *const f32;
    // SAFETY: data_ptr points to original_data which is still alive.
    let roundtripped: &[f32] = unsafe { std::slice::from_raw_parts(data_ptr, 6) };

    assert_eq!(
        roundtripped,
        &original_data[..],
        "Roundtripped data must be byte-for-byte identical to original"
    );

    // Clean up via deleter
    // SAFETY: managed_ptr was created by Box::into_raw and has not been freed.
    unsafe {
        if let Some(deleter) = (*managed_ptr).deleter {
            deleter(managed_ptr);
        }
    }
}

// ============================================================================
// Test 2: Shape and dtype preserved across roundtrip
// ============================================================================

/// INTENT: "Export -> import preserves shape and dtype."
///
/// ML/robotics guarantee: a 3x4 f32 tensor (e.g., a 3-row, 4-column feature
/// matrix) must arrive at the consumer with the same dimensions. If shape is
/// garbled, the consumer will reshape incorrectly, causing silent data
/// misalignment — the most dangerous class of ML bug.
#[test]
fn test_dlpack_intent_shape_preserved() {
    // 3x4 f32 tensor = 12 elements
    let data: Vec<f32> = (0..12).map(|i| i as f32).collect();
    let shape = [3i64, 4];
    let strides = [4i64, 1]; // Row-major

    let managed = to_dlpack(
        data.as_ptr() as *mut c_void,
        &shape,
        &strides,
        TensorDtype::F32,
        Device::cpu(),
    );

    let managed_ptr = Box::into_raw(managed);
    // SAFETY: managed_ptr was just created by to_dlpack with valid data.
    let descriptor = unsafe { from_dlpack(managed_ptr).unwrap() };

    // Shape must be [3, 4]
    assert_eq!(
        descriptor.shape,
        vec![3u64, 4],
        "Shape should be [3, 4], got {:?}",
        descriptor.shape
    );

    // Dtype must be F32
    assert_eq!(
        descriptor.dtype,
        TensorDtype::F32,
        "Dtype should be F32, got {:?}",
        descriptor.dtype
    );

    // Size should be 12 elements * 4 bytes = 48 bytes
    assert_eq!(
        descriptor.size_bytes, 48,
        "Size should be 48 bytes for 3x4 f32 tensor, got {}",
        descriptor.size_bytes
    );

    // Clean up
    // SAFETY: managed_ptr was created by Box::into_raw and has not been freed.
    unsafe {
        if let Some(deleter) = (*managed_ptr).deleter {
            deleter(managed_ptr);
        }
    }
}

// ============================================================================
// Test 3: CPU device metadata correct
// ============================================================================

/// INTENT: "CPU tensors export with device=CPU(0)."
///
/// ML/robotics guarantee: frameworks use device metadata to decide whether
/// to copy data or access it directly. If a CPU tensor is incorrectly
/// tagged as CUDA, the consumer will try to dereference it as a GPU pointer,
/// causing a segfault. The device type and ID must be exactly correct.
#[test]
fn test_dlpack_intent_cpu_device_correct() {
    let data: Vec<f32> = vec![1.0, 2.0, 3.0, 4.0];
    let shape = [4i64];
    let strides = [1i64];

    let managed = to_dlpack(
        data.as_ptr() as *mut c_void,
        &shape,
        &strides,
        TensorDtype::F32,
        Device::cpu(),
    );

    // Check DLTensor device fields directly before import
    assert!(
        managed.dl_tensor.device.is_cpu(),
        "Device type should be CPU, got device_type={}",
        managed.dl_tensor.device.device_type
    );
    assert_eq!(
        managed.dl_tensor.device.device_id, 0,
        "CPU device_id should be 0, got {}",
        managed.dl_tensor.device.device_id
    );

    // Also verify through the import roundtrip
    let managed_ptr = Box::into_raw(managed);
    // SAFETY: managed_ptr was just created by to_dlpack with valid data.
    let descriptor = unsafe { from_dlpack(managed_ptr).unwrap() };

    assert_eq!(
        descriptor.device,
        Device::cpu(),
        "Imported device should be CPU, got {:?}",
        descriptor.device
    );

    // Clean up
    // SAFETY: managed_ptr was created by Box::into_raw and has not been freed.
    unsafe {
        if let Some(deleter) = (*managed_ptr).deleter {
            deleter(managed_ptr);
        }
    }
}

// ============================================================================
// Test 4: Zero-copy verified — data pointer unchanged
// ============================================================================

/// INTENT: "DLPack export is zero-copy — data pointer unchanged."
///
/// Performance guarantee: DLPack's entire purpose is zero-copy tensor
/// exchange between frameworks. If the export copies data, a 10MB point
/// cloud transfer that should take 0ns takes 3ms — blowing the real-time
/// budget. The data pointer in the DLTensor must be the exact same address
/// as the source tensor's data.
#[test]
fn test_dlpack_intent_zero_copy_verified() {
    let data: Vec<f32> = vec![1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0];
    let original_ptr = data.as_ptr() as *mut c_void;
    let shape = [2i64, 4];
    let strides = [4i64, 1];

    let managed = to_dlpack(
        original_ptr,
        &shape,
        &strides,
        TensorDtype::F32,
        Device::cpu(),
    );

    // The DLTensor's data pointer must be exactly the original pointer
    let dl_data_ptr = managed.dl_tensor.data;
    assert_eq!(
        dl_data_ptr, original_ptr,
        "DLTensor data pointer ({:?}) must match original ({:?}) — zero-copy violated",
        dl_data_ptr, original_ptr
    );

    // Byte offset should be 0 (no offset applied)
    assert_eq!(
        managed.dl_tensor.byte_offset, 0,
        "Byte offset should be 0 for a direct export, got {}",
        managed.dl_tensor.byte_offset
    );

    // After import, the descriptor's data_ptr should also match
    let managed_ptr = Box::into_raw(managed);
    // SAFETY: managed_ptr was just created by to_dlpack with valid data.
    let descriptor = unsafe { from_dlpack(managed_ptr).unwrap() };

    assert_eq!(
        descriptor.data_ptr, original_ptr as u64,
        "Imported data_ptr ({}) must match original ({}) — zero-copy violated",
        descriptor.data_ptr, original_ptr as u64
    );

    // Clean up
    // SAFETY: managed_ptr was created by Box::into_raw and has not been freed.
    unsafe {
        if let Some(deleter) = (*managed_ptr).deleter {
            deleter(managed_ptr);
        }
    }
}
