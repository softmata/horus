//! Shared dtype/device/numpy conversion utilities.
//!
//! Centralizes code that was duplicated across tensor.rs, image.rs,
//! pointcloud.rs, and depth_image.rs.

use horus_core::types::{Device, TensorDtype};
use pyo3::exceptions::{PyTypeError, PyValueError};
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
        _ => Err(PyValueError::new_err(format!(
            "Unknown dtype: '{}'. Valid dtypes: float32, float64, float16, bfloat16, \
             int8, int16, int32, int64, uint8, uint16, uint32, uint64, bool \
             (aliases: f32, f64, f16, bf16, i8, i16, i32, i64, u8, u16, u32, u64, float, double, half, int, long, byte)",
            s
        ))),
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
    Device::parse(s).ok_or_else(|| {
        PyValueError::new_err(format!(
            "Unknown device: '{}'. Valid devices: cpu, cuda, cuda:0, cuda:1, ..., metal, vulkan",
            s
        ))
    })
}

/// Format a `Device` as a string.
pub fn device_to_string(device: Device) -> String {
    device.to_string()
}

// === Numpy conversion ===

/// Shared `to_numpy` implementation: calls `np.asarray(obj)` after CPU check.
pub fn to_numpy_impl<'py>(
    slf: &Bound<'py, PyAny>,
    py: Python<'py>,
    is_cuda: bool,
    type_name: &str,
) -> PyResult<Bound<'py, PyAny>> {
    if is_cuda {
        return Err(PyTypeError::new_err(format!(
            "Cannot create a numpy array from a non-CPU {}.",
            type_name,
        )));
    }
    let np = py.import("numpy")?;
    np.call_method1("asarray", (slf,))
}
