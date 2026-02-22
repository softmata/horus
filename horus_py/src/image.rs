//! Python bindings for HORUS Image type
//!
//! Provides `Image` class that hides DLPack, TensorPool, and shared memory
//! internals. Users interact with numpy/torch directly:
//!
//! ```python
//! img = Image.from_numpy(camera_frame)
//! t = img.to_torch()  # zero-copy, no DLPack visible
//! ```

use horus_core::memory::Image;
use horus_types::ImageEncoding;
use pyo3::exceptions::{PyRuntimeError, PyTypeError, PyValueError};
use pyo3::prelude::*;
use pyo3::types::{PyBytes, PyDict, PyTuple};

use crate::dlpack_utils;

/// Parse a user-facing encoding string into `ImageEncoding`.
fn parse_encoding(s: &str) -> PyResult<ImageEncoding> {
    match s.to_lowercase().as_str() {
        "mono8" | "gray" | "grey" | "l" => Ok(ImageEncoding::Mono8),
        "mono16" | "gray16" => Ok(ImageEncoding::Mono16),
        "rgb8" | "rgb" => Ok(ImageEncoding::Rgb8),
        "bgr8" | "bgr" => Ok(ImageEncoding::Bgr8),
        "rgba8" | "rgba" => Ok(ImageEncoding::Rgba8),
        "bgra8" | "bgra" => Ok(ImageEncoding::Bgra8),
        "yuv422" | "yuyv" => Ok(ImageEncoding::Yuv422),
        "mono32f" | "gray32f" | "float" => Ok(ImageEncoding::Mono32F),
        "rgb32f" => Ok(ImageEncoding::Rgb32F),
        "bayer_rggb8" | "bayer" => Ok(ImageEncoding::BayerRggb8),
        "depth16" => Ok(ImageEncoding::Depth16),
        _ => Err(PyValueError::new_err(format!(
            "Unknown encoding: '{}'. Valid: rgb8, bgr8, rgba8, bgra8, mono8, mono16, \
             yuv422, mono32f, rgb32f, bayer_rggb8, depth16",
            s
        ))),
    }
}

/// Convert `ImageEncoding` to a user-friendly string.
fn encoding_to_str(enc: ImageEncoding) -> &'static str {
    match enc {
        ImageEncoding::Mono8 => "mono8",
        ImageEncoding::Mono16 => "mono16",
        ImageEncoding::Rgb8 => "rgb8",
        ImageEncoding::Bgr8 => "bgr8",
        ImageEncoding::Rgba8 => "rgba8",
        ImageEncoding::Bgra8 => "bgra8",
        ImageEncoding::Yuv422 => "yuv422",
        ImageEncoding::Mono32F => "mono32f",
        ImageEncoding::Rgb32F => "rgb32f",
        ImageEncoding::BayerRggb8 => "bayer_rggb8",
        ImageEncoding::Depth16 => "depth16",
    }
}

/// Infer encoding from a numpy array shape and dtype.
fn infer_encoding(shape: &[u64], dtype_str: &str) -> PyResult<ImageEncoding> {
    let channels = if shape.len() == 2 { 1u64 } else { shape[2] };

    match (channels, dtype_str) {
        (1, "uint8") => Ok(ImageEncoding::Mono8),
        (1, "uint16") => Ok(ImageEncoding::Mono16),
        (1, "float32") => Ok(ImageEncoding::Mono32F),
        (3, "uint8") => Ok(ImageEncoding::Rgb8),
        (3, "float32") => Ok(ImageEncoding::Rgb32F),
        (4, "uint8") => Ok(ImageEncoding::Rgba8),
        _ => Err(PyValueError::new_err(format!(
            "Cannot infer encoding from shape {:?} with dtype '{}'. \
             Pass encoding explicitly, e.g. Image.from_numpy(arr, 'bgr8')",
            shape, dtype_str,
        ))),
    }
}

/// HORUS Image — zero-copy shared memory image with ML framework interop.
///
/// Examples:
///     img = Image(480, 640, "rgb8")
///     img = Image.from_numpy(arr)
///     t = img.to_torch()   # zero-copy
///     arr = img.to_numpy()  # zero-copy
#[pyclass(name = "Image")]
pub struct PyImage {
    inner: Image,
}

#[pymethods]
impl PyImage {
    /// Create a new zero-initialized image.
    #[new]
    #[pyo3(signature = (height, width, encoding="rgb8"))]
    fn new(height: u32, width: u32, encoding: &str) -> PyResult<Self> {
        let enc = parse_encoding(encoding)?;
        let img = Image::new(height, width, enc)
            .map_err(|e| PyRuntimeError::new_err(format!("Failed to create image: {}", e)))?;
        Ok(Self { inner: img })
    }

    /// Create an Image from a numpy array.
    #[staticmethod]
    #[pyo3(signature = (array, encoding=None))]
    fn from_numpy(
        py: Python<'_>,
        array: &Bound<'_, PyAny>,
        encoding: Option<&str>,
    ) -> PyResult<Self> {
        let shape_obj = array.getattr("shape")?;
        let shape_tuple: Vec<u64> = shape_obj.extract()?;
        let dtype_obj = array.getattr("dtype")?;
        let dtype_name: String = dtype_obj.getattr("name")?.extract()?;

        if shape_tuple.len() < 2 || shape_tuple.len() > 3 {
            return Err(PyValueError::new_err(format!(
                "Expected array with 2 or 3 dimensions (H, W) or (H, W, C), got shape {:?}",
                shape_tuple
            )));
        }

        let height = shape_tuple[0] as u32;
        let width = shape_tuple[1] as u32;

        let enc = if let Some(enc_str) = encoding {
            parse_encoding(enc_str)?
        } else {
            infer_encoding(&shape_tuple, &dtype_name)?
        };

        let mut img = Image::new(height, width, enc)
            .map_err(|e| PyRuntimeError::new_err(format!("Failed to create image: {}", e)))?;

        let np = py.import("numpy")?;
        let contiguous = np.call_method1("ascontiguousarray", (array,))?;
        let bytes_obj = contiguous.call_method0("tobytes")?;
        let bytes: &[u8] = bytes_obj.cast::<PyBytes>()?.as_bytes();

        let expected = img.nbytes() as usize;
        if bytes.len() != expected {
            return Err(PyValueError::new_err(format!(
                "Array data size ({}) doesn't match image size ({} = {}x{}x{}x{})",
                bytes.len(),
                expected,
                height,
                width,
                enc.channels(),
                enc.tensor_dtype().element_size()
            )));
        }

        img.copy_from(bytes);
        Ok(Self { inner: img })
    }

    /// Create an Image from a PyTorch tensor (CPU only).
    #[staticmethod]
    #[pyo3(signature = (tensor, encoding=None))]
    fn from_torch(
        py: Python<'_>,
        tensor: &Bound<'_, PyAny>,
        encoding: Option<&str>,
    ) -> PyResult<Self> {
        let arr = dlpack_utils::torch_to_numpy(tensor)?;
        Self::from_numpy(py, &arr, encoding)
    }

    /// Create an Image from raw bytes.
    #[staticmethod]
    #[pyo3(signature = (data, height, width, encoding="rgb8"))]
    fn from_bytes(data: &[u8], height: u32, width: u32, encoding: &str) -> PyResult<Self> {
        let enc = parse_encoding(encoding)?;
        let mut img = Image::new(height, width, enc)
            .map_err(|e| PyRuntimeError::new_err(format!("Failed to create image: {}", e)))?;

        let expected = img.nbytes() as usize;
        if data.len() != expected {
            return Err(PyValueError::new_err(format!(
                "Data size ({}) doesn't match image size ({})",
                data.len(),
                expected,
            )));
        }

        img.copy_from(data);
        Ok(Self { inner: img })
    }

    // === ML Framework Conversions ===

    /// Convert to numpy array (zero-copy).
    fn to_numpy<'py>(slf: &Bound<'py, Self>, py: Python<'py>) -> PyResult<Bound<'py, PyAny>> {
        let is_cuda = slf.borrow().inner.is_cuda();
        dlpack_utils::to_numpy_impl(slf.as_any(), py, is_cuda, "image")
    }

    /// Convert to torch tensor (zero-copy via DLPack).
    fn to_torch<'py>(slf: &Bound<'py, Self>, py: Python<'py>) -> PyResult<Bound<'py, PyAny>> {
        dlpack_utils::to_torch_impl(slf.as_any(), py)
    }

    /// Convert to JAX array (zero-copy via DLPack).
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
                "Cannot create numpy array from CUDA image. Use __cuda_array_interface__.",
            ));
        }

        let tensor = self.inner.descriptor().tensor();
        let dict = PyDict::new(py);

        let channels = self.inner.channels();
        let shape: Vec<i64> = if channels == 1 {
            vec![self.inner.height() as i64, self.inner.width() as i64]
        } else {
            vec![
                self.inner.height() as i64,
                self.inner.width() as i64,
                channels as i64,
            ]
        };
        dict.set_item("shape", PyTuple::new(py, &shape)?)?;
        dict.set_item("typestr", tensor.dtype.numpy_typestr())?;

        let ptr = self.inner.pool().data_ptr(tensor) as usize;
        dict.set_item("data", (ptr, false))?;

        let strides: Vec<i64> = tensor.strides().iter().map(|&x| x as i64).collect();
        dict.set_item("strides", PyTuple::new(py, &strides)?)?;
        dict.set_item("version", 3)?;

        Ok(dict.into())
    }

    // === Pixel Access ===

    fn pixel(&self, x: u32, y: u32) -> PyResult<Vec<u8>> {
        self.inner
            .pixel(x, y)
            .map(|p| p.to_vec())
            .ok_or_else(|| PyValueError::new_err(format!("Pixel ({}, {}) out of bounds", x, y)))
    }

    fn set_pixel(&mut self, x: u32, y: u32, value: Vec<u8>) -> PyResult<()> {
        if x >= self.inner.width() || y >= self.inner.height() {
            return Err(PyValueError::new_err(format!(
                "Pixel ({}, {}) out of bounds",
                x, y
            )));
        }
        self.inner.set_pixel(x, y, &value);
        Ok(())
    }

    fn fill(&mut self, value: Vec<u8>) -> PyResult<()> {
        self.inner.fill(&value);
        Ok(())
    }

    fn copy_from(&mut self, data: &[u8]) -> PyResult<()> {
        let expected = self.inner.nbytes() as usize;
        if data.len() != expected {
            return Err(PyValueError::new_err(format!(
                "Data size ({}) doesn't match image size ({})",
                data.len(),
                expected,
            )));
        }
        self.inner.copy_from(data);
        Ok(())
    }

    fn roi(&self, x: u32, y: u32, w: u32, h: u32) -> PyResult<Vec<u8>> {
        self.inner
            .roi(x, y, w, h)
            .ok_or_else(|| PyValueError::new_err("ROI extends beyond image bounds"))
    }

    // === Properties ===

    #[getter]
    fn height(&self) -> u32 {
        self.inner.height()
    }
    #[getter]
    fn width(&self) -> u32 {
        self.inner.width()
    }
    #[getter]
    fn channels(&self) -> u32 {
        self.inner.channels()
    }
    #[getter]
    fn encoding(&self) -> &'static str {
        encoding_to_str(self.inner.encoding())
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
    fn step(&self) -> u32 {
        self.inner.step()
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
    fn is_cpu(&self) -> bool {
        self.inner.is_cpu()
    }
    fn is_cuda(&self) -> bool {
        self.inner.is_cuda()
    }

    fn __repr__(&self) -> String {
        format!(
            "Image(height={}, width={}, encoding='{}', dtype='{}')",
            self.inner.height(),
            self.inner.width(),
            encoding_to_str(self.inner.encoding()),
            dlpack_utils::dtype_to_str(self.inner.dtype()),
        )
    }

    fn __str__(&self) -> String {
        self.__repr__()
    }
}

impl PyImage {
    pub fn from_inner(inner: Image) -> Self {
        Self { inner }
    }
    pub fn inner(&self) -> &Image {
        &self.inner
    }
}
