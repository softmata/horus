//! Python bindings for HORUS DepthImage type

use std::ffi::c_void;

use horus_core::memory::DepthImage;
use horus_types::TensorDtype;
use pyo3::exceptions::{PyRuntimeError, PyTypeError, PyValueError};
use pyo3::prelude::*;
use pyo3::types::{PyBytes, PyDict, PyTuple};

use crate::dlpack_utils::{dtype_to_str, make_dlpack_capsule};

/// Parse a depth dtype string.
fn parse_depth_dtype(s: &str) -> PyResult<TensorDtype> {
    match s.to_lowercase().as_str() {
        "float32" | "f32" | "float" | "meters" => Ok(TensorDtype::F32),
        "uint16" | "u16" | "millimeters" | "mm" => Ok(TensorDtype::U16),
        _ => Err(PyValueError::new_err(format!(
            "Unknown depth dtype: '{}'. Valid: float32 (meters), uint16 (millimeters)", s
        ))),
    }
}

/// Infer depth dtype from numpy dtype string.
fn infer_depth_dtype(dtype_name: &str) -> PyResult<TensorDtype> {
    match dtype_name {
        "float32" => Ok(TensorDtype::F32),
        "uint16" => Ok(TensorDtype::U16),
        "float64" => Ok(TensorDtype::F32), // will convert
        _ => Err(PyValueError::new_err(format!(
            "Cannot infer depth dtype from '{}'. Expected float32 or uint16.", dtype_name,
        ))),
    }
}

/// HORUS DepthImage — zero-copy shared memory depth image with ML framework interop.
///
/// Examples:
///     depth = DepthImage(480, 640)               # float32 meters
///     depth = DepthImage(480, 640, "uint16")      # uint16 millimeters
///     depth = DepthImage.from_numpy(arr)
///     d = depth.get_depth(320, 240)               # meters
///     t = depth.to_torch()                        # zero-copy
#[pyclass(name = "DepthImage")]
pub struct PyDepthImage {
    inner: DepthImage,
}

#[pymethods]
impl PyDepthImage {
    /// Create a new zero-initialized depth image.
    #[new]
    #[pyo3(signature = (height, width, dtype="float32"))]
    fn new(height: u32, width: u32, dtype: &str) -> PyResult<Self> {
        let dt = parse_depth_dtype(dtype)?;
        let depth = DepthImage::new(height, width, dt)
            .map_err(|e| PyRuntimeError::new_err(format!("Failed to create depth image: {}", e)))?;
        Ok(Self { inner: depth })
    }

    /// Create a DepthImage from a numpy array with shape (H, W).
    #[staticmethod]
    fn from_numpy(py: Python<'_>, array: &Bound<'_, PyAny>) -> PyResult<Self> {
        let shape_obj = array.getattr("shape")?;
        let shape_tuple: Vec<u64> = shape_obj.extract()?;
        let dtype_obj = array.getattr("dtype")?;
        let dtype_name: String = dtype_obj.getattr("name")?.extract()?;

        if shape_tuple.len() != 2 {
            return Err(PyValueError::new_err(format!(
                "Expected 2D array (H, W), got shape {:?}", shape_tuple
            )));
        }

        let height = shape_tuple[0] as u32;
        let width = shape_tuple[1] as u32;
        let np = py.import("numpy")?;

        // Handle float64 by converting to float32
        let (dt, array_to_use) = if dtype_name == "float64" {
            let f32_arr = array.call_method1("astype", ("float32",))?;
            (TensorDtype::F32, f32_arr)
        } else {
            let dt = infer_depth_dtype(&dtype_name)?;
            let contiguous = np.call_method1("ascontiguousarray", (array,))?;
            (dt, contiguous)
        };

        let depth = DepthImage::new(height, width, dt)
            .map_err(|e| PyRuntimeError::new_err(format!("Failed to create depth image: {}", e)))?;

        let contiguous = np.call_method1("ascontiguousarray", (&array_to_use,))?;
        let bytes_obj = contiguous.call_method0("tobytes")?;
        let bytes: &[u8] = bytes_obj.cast::<PyBytes>()?.as_bytes();

        let expected = depth.nbytes() as usize;
        if bytes.len() != expected {
            return Err(PyValueError::new_err(format!(
                "Array data size ({}) doesn't match depth image size ({})",
                bytes.len(), expected,
            )));
        }

        depth.data_mut().copy_from_slice(bytes);
        Ok(Self { inner: depth })
    }

    /// Create a DepthImage from a PyTorch tensor (CPU only).
    #[staticmethod]
    fn from_torch(py: Python<'_>, tensor: &Bound<'_, PyAny>) -> PyResult<Self> {
        let device = tensor.getattr("device")?;
        let device_type: String = device.getattr("type")?.extract()?;
        if device_type != "cpu" {
            return Err(PyValueError::new_err(
                "Tensor must be on CPU. Call tensor.cpu() first.",
            ));
        }
        let arr = tensor.call_method0("numpy")?;
        Self::from_numpy(py, &arr)
    }

    // === ML Framework Conversions ===

    fn to_numpy<'py>(slf: &Bound<'py, Self>, py: Python<'py>) -> PyResult<Bound<'py, PyAny>> {
        let inner = slf.borrow();
        if inner.inner.is_cuda() {
            return Err(PyTypeError::new_err(
                "Cannot create numpy array from CUDA depth image.",
            ));
        }
        let np = py.import("numpy")?;
        np.call_method1("asarray", (slf,))
    }

    fn to_torch<'py>(slf: &Bound<'py, Self>, py: Python<'py>) -> PyResult<Bound<'py, PyAny>> {
        let torch = py.import("torch")?;
        torch.call_method1("from_dlpack", (slf,))
    }

    fn to_jax<'py>(slf: &Bound<'py, Self>, py: Python<'py>) -> PyResult<Bound<'py, PyAny>> {
        let jax_dlpack = py.import("jax.dlpack")?;
        jax_dlpack.call_method1("from_dlpack", (slf,))
    }

    // === DLPack Protocol ===

    #[pyo3(signature = (stream=None))]
    fn __dlpack__(&self, py: Python<'_>, stream: Option<i64>) -> PyResult<Py<PyAny>> {
        let _ = stream;
        let tensor = self.inner.descriptor().tensor();

        let shape: Vec<i64> = tensor.shape().iter().map(|&x| x as i64).collect();
        let elem_size = tensor.dtype.element_size() as i64;
        let strides: Vec<i64> = tensor.strides().iter().map(|&x| (x as i64) / elem_size).collect();

        let data_ptr = if self.inner.is_cuda() {
            u64::from_le_bytes(tensor.cuda_ipc_handle[..8].try_into().unwrap()) as *mut c_void
        } else {
            self.inner.pool().data_ptr(tensor) as *mut c_void
        };

        make_dlpack_capsule(py, data_ptr, &shape, &strides, tensor.dtype, tensor.device())
    }

    fn __dlpack_device__(&self) -> (i32, i32) {
        let device = self.inner.descriptor().tensor().device();
        (device.to_dlpack_device_type(), device.to_dlpack_device_id())
    }

    // === Numpy Array Interface ===

    #[getter]
    fn __array_interface__(&self, py: Python<'_>) -> PyResult<Py<PyAny>> {
        if self.inner.is_cuda() {
            return Err(PyTypeError::new_err(
                "Cannot create numpy array from CUDA depth image.",
            ));
        }

        let tensor = self.inner.descriptor().tensor();
        let dict = PyDict::new(py);

        let shape = vec![self.inner.height() as i64, self.inner.width() as i64];
        dict.set_item("shape", PyTuple::new(py, &shape)?)?;
        dict.set_item("typestr", tensor.dtype.numpy_typestr())?;

        let ptr = self.inner.pool().data_ptr(tensor) as usize;
        dict.set_item("data", (ptr, false))?;

        let strides: Vec<i64> = tensor.strides().iter().map(|&x| x as i64).collect();
        dict.set_item("strides", PyTuple::new(py, &strides)?)?;
        dict.set_item("version", 3)?;

        Ok(dict.into())
    }

    // === Depth Access ===

    /// Get depth at pixel (x, y) in meters.
    fn get_depth(&self, x: u32, y: u32) -> PyResult<f32> {
        self.inner.get_depth(x, y).ok_or_else(|| {
            PyValueError::new_err(format!("Pixel ({}, {}) out of bounds", x, y))
        })
    }

    /// Set depth at pixel (x, y) in meters.
    fn set_depth(&mut self, x: u32, y: u32, value: f32) -> PyResult<()> {
        if x >= self.inner.width() || y >= self.inner.height() {
            return Err(PyValueError::new_err(format!("Pixel ({}, {}) out of bounds", x, y)));
        }
        self.inner.set_depth(x, y, value);
        Ok(())
    }

    /// Calculate depth statistics: (min, max, mean) in meters.
    fn depth_statistics(&self) -> (f32, f32, f32) {
        self.inner.depth_statistics()
    }

    // === Properties ===

    #[getter]
    fn height(&self) -> u32 { self.inner.height() }
    #[getter]
    fn width(&self) -> u32 { self.inner.width() }
    #[getter]
    fn dtype(&self) -> &'static str { dtype_to_str(self.inner.dtype()) }
    #[getter]
    fn nbytes(&self) -> u64 { self.inner.nbytes() }
    #[getter]
    fn frame_id(&self) -> &str { self.inner.frame_id() }
    #[getter]
    fn timestamp_ns(&self) -> u64 { self.inner.timestamp_ns() }
    #[getter]
    fn depth_scale(&self) -> f32 { self.inner.depth_scale() }

    fn set_frame_id(&mut self, id: &str) { self.inner.set_frame_id(id); }
    fn set_timestamp_ns(&mut self, ts: u64) { self.inner.set_timestamp_ns(ts); }
    fn is_meters(&self) -> bool { self.inner.is_meters() }
    fn is_millimeters(&self) -> bool { self.inner.is_millimeters() }
    fn is_cpu(&self) -> bool { self.inner.is_cpu() }
    fn is_cuda(&self) -> bool { self.inner.is_cuda() }

    fn __repr__(&self) -> String {
        let kind = if self.inner.is_meters() { "meters" } else { "millimeters" };
        format!(
            "DepthImage(height={}, width={}, dtype='{}', format={})",
            self.inner.height(), self.inner.width(),
            dtype_to_str(self.inner.dtype()), kind,
        )
    }

    fn __str__(&self) -> String { self.__repr__() }
}

impl PyDepthImage {
    pub fn from_inner(inner: DepthImage) -> Self { Self { inner } }
    pub fn inner(&self) -> &DepthImage { &self.inner }
}
