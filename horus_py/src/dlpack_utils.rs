//! Shared DLPack and dtype utilities for all Python binding types.
//!
//! Centralizes code that was duplicated across tensor.rs, image.rs,
//! pointcloud.rs, and depth_image.rs.

use std::ffi::{c_void, CStr, CString};

use horus_types::{Device, TensorDtype};
use pyo3::exceptions::{PyRuntimeError, PyValueError};
use pyo3::prelude::*;

// === Dtype helpers ===

/// Parse a user-facing dtype string into `TensorDtype`.
pub fn parse_dtype(s: &str) -> PyResult<TensorDtype> {
    match s.to_lowercase().as_str() {
        "float32" | "f32" | "float" => Ok(TensorDtype::F32),
        "float64" | "f64" | "double" => Ok(TensorDtype::F64),
        "float16" | "f16" | "half" => Ok(TensorDtype::F16),
        "bfloat16" | "bf16" => Ok(TensorDtype::BF16),
        "int8" | "i8" => Ok(TensorDtype::I8),
        "int16" | "i16" => Ok(TensorDtype::I16),
        "int32" | "i32" | "int" => Ok(TensorDtype::I32),
        "int64" | "i64" | "long" => Ok(TensorDtype::I64),
        "uint8" | "u8" | "byte" => Ok(TensorDtype::U8),
        "uint16" | "u16" => Ok(TensorDtype::U16),
        "uint32" | "u32" => Ok(TensorDtype::U32),
        "uint64" | "u64" => Ok(TensorDtype::U64),
        "bool" => Ok(TensorDtype::Bool),
        _ => Err(PyValueError::new_err(format!("Unknown dtype: {}", s))),
    }
}

/// Convert `TensorDtype` to a user-facing string.
pub fn dtype_to_str(dtype: TensorDtype) -> &'static str {
    match dtype {
        TensorDtype::F32 => "float32",
        TensorDtype::F64 => "float64",
        TensorDtype::F16 => "float16",
        TensorDtype::BF16 => "bfloat16",
        TensorDtype::I8 => "int8",
        TensorDtype::I16 => "int16",
        TensorDtype::I32 => "int32",
        TensorDtype::I64 => "int64",
        TensorDtype::U8 => "uint8",
        TensorDtype::U16 => "uint16",
        TensorDtype::U32 => "uint32",
        TensorDtype::U64 => "uint64",
        TensorDtype::Bool => "bool",
    }
}

/// Parse a device string ("cpu", "cuda:0") into `Device`.
pub fn parse_device(s: &str) -> PyResult<Device> {
    Device::parse(s).ok_or_else(|| PyValueError::new_err(format!("Unknown device: {}", s)))
}

/// Format a `Device` as a string.
pub fn device_to_string(device: Device) -> String {
    device.to_string()
}

// === DLPack helpers ===

/// Create a DLPack PyCapsule from a HORUS tensor's raw components.
///
/// Shared implementation for `__dlpack__` across Image, PointCloud, DepthImage,
/// and TensorHandle.
pub fn make_dlpack_capsule(
    py: Python<'_>,
    data_ptr: *mut c_void,
    shape: &[i64],
    strides_elements: &[i64],
    dtype: TensorDtype,
    device: Device,
) -> PyResult<Py<PyAny>> {
    let managed = horus_core::dlpack::to_dlpack(data_ptr, shape, strides_elements, dtype, device);
    let managed_ptr = Box::into_raw(managed);

    let capsule_name = CString::new("dltensor").unwrap();
    unsafe {
        let capsule = pyo3::ffi::PyCapsule_New(
            managed_ptr as *mut c_void,
            capsule_name.as_ptr(),
            Some(dlpack_capsule_destructor),
        );
        std::mem::forget(capsule_name);
        if capsule.is_null() {
            let managed = &mut *managed_ptr;
            if let Some(deleter) = managed.deleter {
                deleter(managed_ptr);
            }
            return Err(PyRuntimeError::new_err("Failed to create DLPack capsule"));
        }
        Ok(Py::from_owned_ptr(py, capsule))
    }
}

/// PyCapsule destructor for unclaimed DLPack tensors.
///
/// Per DLPack spec: if capsule name is still "dltensor" at destruction time,
/// the consumer never claimed it, so we must call the deleter.
pub unsafe extern "C" fn dlpack_capsule_destructor(capsule: *mut pyo3::ffi::PyObject) {
    let name_ptr = pyo3::ffi::PyCapsule_GetName(capsule);
    if name_ptr.is_null() {
        return;
    }
    let name = CStr::from_ptr(name_ptr);
    if name.to_bytes() == b"dltensor" {
        let ptr = pyo3::ffi::PyCapsule_GetPointer(capsule, name_ptr);
        if !ptr.is_null() {
            let managed = ptr as *mut horus_core::dlpack::DLManagedTensor;
            if let Some(deleter) = (*managed).deleter {
                deleter(managed);
            }
        }
    }
}
