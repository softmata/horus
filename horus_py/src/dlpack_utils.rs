//! Shared DLPack and dtype utilities for all Python binding types.
//!
//! Centralizes code that was duplicated across tensor.rs, image.rs,
//! pointcloud.rs, and depth_image.rs.

use std::ffi::{c_void, CStr, CString};

use horus_types::{Device, TensorDtype};
use pyo3::exceptions::{PyRuntimeError, PyTypeError, PyValueError};
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

// === ML framework conversion helpers ===

/// Shared `to_torch` implementation: calls `torch.from_dlpack(obj)`.
pub fn to_torch_impl<'py>(slf: &Bound<'py, PyAny>, py: Python<'py>) -> PyResult<Bound<'py, PyAny>> {
    let torch = py.import("torch")?;
    torch.call_method1("from_dlpack", (slf,))
}

/// Shared `to_jax` implementation: calls `jax.dlpack.from_dlpack(obj)`.
pub fn to_jax_impl<'py>(slf: &Bound<'py, PyAny>, py: Python<'py>) -> PyResult<Bound<'py, PyAny>> {
    let jax_dlpack = py.import("jax.dlpack")?;
    jax_dlpack.call_method1("from_dlpack", (slf,))
}

/// Shared `to_numpy` implementation: calls `np.asarray(obj)` after CUDA check.
pub fn to_numpy_impl<'py>(
    slf: &Bound<'py, PyAny>,
    py: Python<'py>,
    is_cuda: bool,
    type_name: &str,
) -> PyResult<Bound<'py, PyAny>> {
    if is_cuda {
        return Err(PyTypeError::new_err(format!(
            "Cannot create numpy array from CUDA {}. Use .to_torch() for GPU data.",
            type_name,
        )));
    }
    let np = py.import("numpy")?;
    np.call_method1("asarray", (slf,))
}

/// Shared `from_torch` helper: verifies CPU, calls `.numpy()`, returns the numpy array.
pub fn torch_to_numpy<'py>(tensor: &Bound<'py, PyAny>) -> PyResult<Bound<'py, PyAny>> {
    let device = tensor.getattr("device")?;
    let device_type: String = device.getattr("type")?.extract()?;
    if device_type != "cpu" {
        return Err(PyValueError::new_err(
            "Tensor must be on CPU. Call tensor.cpu() first.",
        ));
    }
    tensor.call_method0("numpy")
}

// === DLPack helpers ===

/// Prepare DLPack arguments from a HorusTensor + TensorPool.
///
/// Returns (data_ptr, shape, strides_in_elements, dtype, device) — everything
/// needed by `make_dlpack_capsule`.
pub fn prepare_dlpack_args(
    tensor: &horus_types::HorusTensor,
    pool: &horus_core::memory::TensorPool,
) -> (*mut c_void, Vec<i64>, Vec<i64>, TensorDtype, Device) {
    let shape: Vec<i64> = tensor.shape().iter().map(|&x| x as i64).collect();
    let elem_size = tensor.dtype.element_size() as i64;
    let strides: Vec<i64> = tensor
        .strides()
        .iter()
        .map(|&x| (x as i64) / elem_size)
        .collect();

    let data_ptr = pool.data_ptr(tensor) as *mut c_void;

    (data_ptr, shape, strides, tensor.dtype, tensor.device())
}

/// Shared `__dlpack_device__` implementation for types wrapping a HorusTensor.
pub fn dlpack_device_tuple(tensor: &horus_types::HorusTensor) -> (i32, i32) {
    let device = tensor.device();
    (device.to_dlpack_device_type(), device.to_dlpack_device_id())
}

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
    // SAFETY: managed_ptr is a valid heap allocation from Box::into_raw above.
    // PyCapsule_New takes ownership of the pointer and will call
    // dlpack_capsule_destructor when the capsule is garbage collected.
    // capsule_name is forgotten to prevent double-free (PyCapsule holds the C string).
    // If PyCapsule_New fails (returns null), we manually call the deleter to
    // prevent a memory leak.
    unsafe {
        let capsule = pyo3::ffi::PyCapsule_New(
            managed_ptr as *mut c_void,
            capsule_name.as_ptr(),
            Some(dlpack_capsule_destructor),
        );
        std::mem::forget(capsule_name);
        if capsule.is_null() {
            // SAFETY: managed_ptr is still valid (PyCapsule_New failed, so
            // it did not take ownership). We call the deleter to clean up.
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
///
/// # Safety
///
/// Called by Python's PyCapsule machinery. `capsule` must be a valid PyCapsule
/// created by `make_dlpack_capsule` containing a `DLManagedTensor` pointer.
pub unsafe extern "C" fn dlpack_capsule_destructor(capsule: *mut pyo3::ffi::PyObject) {
    // SAFETY: capsule is a valid PyCapsule object provided by Python runtime.
    let name_ptr = pyo3::ffi::PyCapsule_GetName(capsule);
    if name_ptr.is_null() {
        return;
    }
    // SAFETY: name_ptr is a valid null-terminated C string set by PyCapsule_New.
    let name = CStr::from_ptr(name_ptr);
    if name.to_bytes() == b"dltensor" {
        // SAFETY: capsule contains a DLManagedTensor pointer set by PyCapsule_New.
        let ptr = pyo3::ffi::PyCapsule_GetPointer(capsule, name_ptr);
        if !ptr.is_null() {
            // SAFETY: ptr is the DLManagedTensor* we stored in the capsule.
            // Calling the deleter transfers ownership back and frees resources.
            let managed = ptr as *mut horus_core::dlpack::DLManagedTensor;
            if let Some(deleter) = (*managed).deleter {
                deleter(managed);
            }
        }
    }
}
