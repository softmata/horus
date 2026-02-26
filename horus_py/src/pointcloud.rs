//! Python bindings for HORUS PointCloud type

use horus_core::memory::PointCloud;
use horus_core::types::TensorDtype;
use pyo3::exceptions::{PyRuntimeError, PyTypeError, PyValueError};
use pyo3::prelude::*;
use pyo3::types::{PyBytes, PyDict, PyTuple};

use crate::dlpack_utils;

/// HORUS PointCloud — zero-copy shared memory point cloud with ML framework interop.
///
/// Examples:
///     pc = PointCloud(1000)              # 1000 XYZ points (float32)
///     pc = PointCloud(1000, fields=4)    # 1000 XYZI points
///     pc = PointCloud.from_numpy(arr)    # from (N, 3) or (N, 4) array
///     t = pc.to_torch()                  # zero-copy
#[pyclass(name = "PointCloud")]
pub struct PyPointCloud {
    inner: PointCloud,
}

#[pymethods]
impl PyPointCloud {
    /// Create a new zero-initialized point cloud.
    #[new]
    #[pyo3(signature = (num_points, fields=3, dtype="float32"))]
    fn new(num_points: u32, fields: u32, dtype: &str) -> PyResult<Self> {
        let dt = dlpack_utils::parse_dtype(dtype)?;
        let pc = PointCloud::new(num_points, fields, dt)
            .map_err(|e| PyRuntimeError::new_err(format!("Failed to create point cloud: {}", e)))?;
        Ok(Self { inner: pc })
    }

    /// Create a PointCloud from a numpy array with shape (N, F).
    #[staticmethod]
    fn from_numpy(py: Python<'_>, array: &Bound<'_, PyAny>) -> PyResult<Self> {
        let shape_obj = array.getattr("shape")?;
        let shape_tuple: Vec<u64> = shape_obj.extract()?;
        let dtype_obj = array.getattr("dtype")?;
        let dtype_name: String = dtype_obj.getattr("name")?.extract()?;

        if shape_tuple.len() != 2 {
            return Err(PyValueError::new_err(format!(
                "Expected 2D array (N, fields), got shape {:?}",
                shape_tuple
            )));
        }

        let num_points = shape_tuple[0] as u32;
        let fields = shape_tuple[1] as u32;
        let dt = dlpack_utils::parse_dtype(&dtype_name)?;

        let mut pc = PointCloud::new(num_points, fields, dt)
            .map_err(|e| PyRuntimeError::new_err(format!("Failed to create point cloud: {}", e)))?;

        let np = py.import("numpy")?;
        let contiguous = np.call_method1("ascontiguousarray", (array,))?;
        let bytes_obj = contiguous.call_method0("tobytes")?;
        let bytes: &[u8] = bytes_obj.cast::<PyBytes>()?.as_bytes();

        let expected = pc.nbytes() as usize;
        if bytes.len() != expected {
            return Err(PyValueError::new_err(format!(
                "Array data size ({}) doesn't match point cloud size ({})",
                bytes.len(),
                expected,
            )));
        }

        pc.copy_from(bytes);
        Ok(Self { inner: pc })
    }

    /// Create a PointCloud from a PyTorch tensor (CPU only).
    #[staticmethod]
    fn from_torch(py: Python<'_>, tensor: &Bound<'_, PyAny>) -> PyResult<Self> {
        let arr = dlpack_utils::torch_to_numpy(tensor)?;
        Self::from_numpy(py, &arr)
    }

    // === ML Framework Conversions ===

    fn to_numpy<'py>(slf: &Bound<'py, Self>, py: Python<'py>) -> PyResult<Bound<'py, PyAny>> {
        let is_cuda = slf.borrow().inner.is_cuda();
        dlpack_utils::to_numpy_impl(slf.as_any(), py, is_cuda, "point cloud")
    }

    fn to_torch<'py>(slf: &Bound<'py, Self>, py: Python<'py>) -> PyResult<Bound<'py, PyAny>> {
        dlpack_utils::to_torch_impl(slf.as_any(), py)
    }

    fn to_jax<'py>(slf: &Bound<'py, Self>, py: Python<'py>) -> PyResult<Bound<'py, PyAny>> {
        dlpack_utils::to_jax_impl(slf.as_any(), py)
    }

    // === DLPack Protocol ===

    #[pyo3(signature = (stream=None))]
    fn __dlpack__(&self, py: Python<'_>, stream: Option<i64>) -> PyResult<Py<PyAny>> {
        let _ = stream;
        let tensor = self.inner.descriptor().tensor();
        let (data_ptr, shape, strides, dtype, device) =
            dlpack_utils::prepare_dlpack_args(tensor, self.inner.pool());
        dlpack_utils::make_dlpack_capsule(py, data_ptr, &shape, &strides, dtype, device)
    }

    fn __dlpack_device__(&self) -> (i32, i32) {
        dlpack_utils::dlpack_device_tuple(self.inner.descriptor().tensor())
    }

    // === Numpy Array Interface ===

    #[getter]
    fn __array_interface__(&self, py: Python<'_>) -> PyResult<Py<PyAny>> {
        if self.inner.is_cuda() {
            return Err(PyTypeError::new_err(
                "Cannot create numpy array from CUDA point cloud.",
            ));
        }

        let tensor = self.inner.descriptor().tensor();
        let dict = PyDict::new(py);

        let shape = vec![
            self.inner.point_count() as i64,
            self.inner.fields_per_point() as i64,
        ];
        dict.set_item("shape", PyTuple::new(py, &shape)?)?;
        dict.set_item("typestr", tensor.dtype.numpy_typestr())?;

        let ptr = self.inner.pool().data_ptr(tensor) as usize;
        dict.set_item("data", (ptr, false))?;

        let strides: Vec<i64> = tensor.strides().iter().map(|&x| x as i64).collect();
        dict.set_item("strides", PyTuple::new(py, &strides)?)?;
        dict.set_item("version", 3)?;

        Ok(dict.into())
    }

    // === Point Access ===

    /// Get the i-th point as a list of float values (float32 only).
    fn point_at(&self, idx: u64) -> PyResult<Vec<f32>> {
        let raw = self
            .inner
            .point_at(idx)
            .ok_or_else(|| PyValueError::new_err(format!("Point index {} out of bounds", idx)))?;

        if self.inner.dtype() != TensorDtype::F32 {
            return Err(PyTypeError::new_err(
                "point_at() as floats only supported for float32 point clouds",
            ));
        }

        let fpp = self.inner.fields_per_point() as usize;
        let mut values = Vec::with_capacity(fpp);
        for i in 0..fpp {
            let offset = i * 4;
            if offset + 4 <= raw.len() {
                let bytes = [
                    raw[offset],
                    raw[offset + 1],
                    raw[offset + 2],
                    raw[offset + 3],
                ];
                values.push(f32::from_le_bytes(bytes));
            }
        }
        Ok(values)
    }

    // === Properties ===

    #[getter]
    fn point_count(&self) -> u64 {
        self.inner.point_count()
    }
    #[getter]
    fn fields_per_point(&self) -> u32 {
        self.inner.fields_per_point()
    }
    #[getter]
    fn dtype(&self) -> &'static str {
        dlpack_utils::dtype_to_str(self.inner.dtype())
    }
    #[getter]
    fn nbytes(&self) -> u64 {
        self.inner.nbytes()
    }
    #[getter]
    fn frame_id(&self) -> &str {
        self.inner.frame_id()
    }
    #[getter]
    fn timestamp_ns(&self) -> u64 {
        self.inner.timestamp_ns()
    }

    fn set_frame_id(&mut self, id: &str) {
        self.inner.set_frame_id(id);
    }
    fn set_timestamp_ns(&mut self, ts: u64) {
        self.inner.set_timestamp_ns(ts);
    }
    fn is_xyz(&self) -> bool {
        self.inner.is_xyz()
    }
    fn has_intensity(&self) -> bool {
        self.inner.has_intensity()
    }
    fn has_color(&self) -> bool {
        self.inner.has_color()
    }
    fn is_cpu(&self) -> bool {
        self.inner.is_cpu()
    }
    fn is_cuda(&self) -> bool {
        self.inner.is_cuda()
    }

    fn __repr__(&self) -> String {
        format!(
            "PointCloud(points={}, fields={}, dtype='{}')",
            self.inner.point_count(),
            self.inner.fields_per_point(),
            dlpack_utils::dtype_to_str(self.inner.dtype()),
        )
    }

    fn __str__(&self) -> String {
        self.__repr__()
    }
}

impl PyPointCloud {
    pub fn from_inner(inner: PointCloud) -> Self {
        Self { inner }
    }
    pub fn inner(&self) -> &PointCloud {
        &self.inner
    }
}
