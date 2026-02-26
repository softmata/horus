//! Python bindings for HORUS tensor system
//!
//! Provides zero-copy tensor access from Python via numpy's `__array_interface__`
//! and the buffer protocol.

use horus::memory::{TensorHandle, TensorPool, TensorPoolConfig};
use horus_core::types::{HorusTensor, TensorDtype};
use parking_lot::RwLock;
use pyo3::exceptions::{PyRuntimeError, PyTypeError, PyValueError};
use pyo3::prelude::*;
use pyo3::types::{PyDict, PyTuple};
use std::collections::HashMap;
use std::ffi::{c_void, CString};
use std::sync::Arc;

use crate::dlpack_utils;

// Global pool registry - allows multiple pools to be accessed by ID
lazy_static::lazy_static! {
    static ref POOL_REGISTRY: RwLock<HashMap<u32, Arc<TensorPool>>> = RwLock::new(HashMap::new());
}

/// Auto-detect the optimal pool allocator based on GPU hardware.
///
/// Returns the pool allocator — currently always mmap.
fn auto_allocator() -> horus::memory::PoolAllocator {
    horus::memory::PoolAllocator::Mmap
}

/// Get or create a tensor pool by ID.
///
/// When `config` is `None`, auto-detects the optimal allocator based on GPU
/// hardware — users get the best backend without opt-in.
fn get_or_create_pool(pool_id: u32, config: Option<TensorPoolConfig>) -> PyResult<Arc<TensorPool>> {
    // Check if pool already exists
    {
        let registry = POOL_REGISTRY.read();
        if let Some(pool) = registry.get(&pool_id) {
            return Ok(Arc::clone(pool));
        }
    }

    // Create new pool with auto-detected allocator when no config specified
    let config = config.unwrap_or_else(|| TensorPoolConfig {
        allocator: auto_allocator(),
        ..TensorPoolConfig::default()
    });
    let pool = TensorPool::new(pool_id, config)
        .map_err(|e| PyRuntimeError::new_err(format!("Failed to create pool: {}", e)))?;
    let pool = Arc::new(pool);

    // Register pool
    {
        let mut registry = POOL_REGISTRY.write();
        registry.insert(pool_id, Arc::clone(&pool));
    }

    Ok(pool)
}

/// Python wrapper for TensorPool
#[pyclass(name = "TensorPool")]
pub struct PyTensorPool {
    pool: Arc<TensorPool>,
}

#[pymethods]
impl PyTensorPool {
    /// Create or open a tensor pool
    ///
    /// The pool automatically selects the optimal memory backend based on
    /// detected GPU hardware:
    ///   - Jetson: cudaMallocManaged (unified CPU+GPU memory)
    ///   - Discrete GPU: cudaMallocHost (pinned, fast DMA)
    ///   - No GPU: mmap (standard shared memory)
    ///
    /// Args:
    ///     pool_id: Unique identifier for the pool (default: 1)
    ///     size_mb: Pool size in megabytes (default: 1024 = 1GB)
    ///     max_slots: Maximum concurrent tensors (default: 1024)
    ///
    /// Returns:
    ///     TensorPool instance
    #[new]
    #[pyo3(signature = (pool_id=1, size_mb=1024, max_slots=1024))]
    fn new(pool_id: u32, size_mb: usize, max_slots: usize) -> PyResult<Self> {
        let config = TensorPoolConfig {
            pool_size: size_mb * 1024 * 1024,
            max_slots,
            slot_alignment: 64,
            allocator: auto_allocator(),
        };
        let pool = get_or_create_pool(pool_id, Some(config))?;
        Ok(Self { pool })
    }

    /// Allocate a new tensor
    ///
    /// Args:
    ///     shape: Tuple of dimensions (e.g., (1080, 1920, 3))
    ///     dtype: Data type string ("float32", "uint8", etc.)
    ///     device: Device string ("cpu" or "cuda:N")
    ///
    /// Returns:
    ///     TensorHandle for the allocated tensor
    #[pyo3(signature = (shape, dtype="float32", device="cpu"))]
    fn alloc(&self, shape: Vec<u64>, dtype: &str, device: &str) -> PyResult<PyTensorHandle> {
        let dtype = parse_dtype(dtype)?;
        let device = parse_device(device)?;

        let handle = TensorHandle::alloc(Arc::clone(&self.pool), &shape, dtype, device)
            .map_err(|e| PyRuntimeError::new_err(format!("Allocation failed: {}", e)))?;

        Ok(PyTensorHandle {
            handle: Some(handle),
        })
    }

    /// Get pool statistics
    fn stats(&self) -> PyResult<Py<PyAny>> {
        Python::attach(|py| {
            let stats = self.pool.stats();
            let dict = PyDict::new(py);
            dict.set_item("pool_id", stats.pool_id)?;
            dict.set_item("pool_size", stats.pool_size)?;
            dict.set_item("max_slots", stats.max_slots)?;
            dict.set_item("allocated_slots", stats.allocated_slots)?;
            dict.set_item("total_refcount", stats.total_refcount)?;
            dict.set_item("used_bytes", stats.used_bytes)?;
            dict.set_item("free_bytes", stats.free_bytes)?;
            Ok(dict.into())
        })
    }

    /// Get pool ID
    #[getter]
    fn pool_id(&self) -> u32 {
        self.pool.pool_id()
    }
}

/// Python wrapper for TensorHandle with numpy interop
#[pyclass(name = "TensorHandle")]
pub struct PyTensorHandle {
    handle: Option<TensorHandle>,
}

#[pymethods]
impl PyTensorHandle {
    /// Create a TensorHandle from a raw tensor descriptor
    ///
    /// This is used when receiving tensors from Topic.
    #[staticmethod]
    fn from_descriptor(pool_id: u32, descriptor_bytes: &[u8]) -> PyResult<Self> {
        if descriptor_bytes.len() != std::mem::size_of::<HorusTensor>() {
            return Err(PyValueError::new_err(format!(
                "Invalid descriptor size: expected {}, got {}",
                std::mem::size_of::<HorusTensor>(),
                descriptor_bytes.len()
            )));
        }

        // SAFETY: descriptor_bytes length is validated above to match size_of::<HorusTensor>().
        // HorusTensor is a POD type (repr(C), all primitive fields).
        // Use read_unaligned since Python byte arrays may not meet HorusTensor alignment requirements.
        let tensor: HorusTensor =
            unsafe { std::ptr::read_unaligned(descriptor_bytes.as_ptr() as *const HorusTensor) };

        let pool = get_or_create_pool(pool_id, None)?;
        let handle = TensorHandle::new(tensor, pool);

        Ok(Self {
            handle: Some(handle),
        })
    }

    /// Get the raw descriptor bytes (for sending through Topic)
    fn to_descriptor(&self) -> PyResult<Vec<u8>> {
        let handle = self
            .handle
            .as_ref()
            .ok_or_else(|| PyRuntimeError::new_err("TensorHandle has been released"))?;

        let tensor = handle.tensor();
        // SAFETY: HorusTensor is repr(C) with only POD fields. Reading its bytes is safe.
        let bytes = unsafe {
            std::slice::from_raw_parts(
                tensor as *const HorusTensor as *const u8,
                std::mem::size_of::<HorusTensor>(),
            )
        };
        Ok(bytes.to_vec())
    }

    /// Numpy array interface for zero-copy access
    ///
    /// This enables: np.asarray(tensor_handle)
    #[getter]
    fn __array_interface__(&self, py: Python<'_>) -> PyResult<Py<PyAny>> {
        let handle = self
            .handle
            .as_ref()
            .ok_or_else(|| PyRuntimeError::new_err("TensorHandle has been released"))?;

        if handle.is_cuda() {
            return Err(PyTypeError::new_err(
                "Cannot create numpy array from CUDA tensor. Use __cuda_array_interface__ instead.",
            ));
        }

        let tensor = handle.tensor();
        let dict = PyDict::new(py);

        // Shape
        let shape: Vec<i64> = handle.shape().iter().map(|&x| x as i64).collect();
        dict.set_item("shape", PyTuple::new(py, &shape)?)?;

        // Type string
        let typestr = dtype_numpy_typestr(tensor.dtype);
        dict.set_item("typestr", typestr)?;

        // Data pointer and read-only flag
        let ptr = handle.data_ptr() as usize;
        dict.set_item("data", (ptr, false))?; // (ptr, readonly=False)

        // Strides
        let strides: Vec<i64> = handle.strides().iter().map(|&x| x as i64).collect();
        dict.set_item("strides", PyTuple::new(py, &strides)?)?;

        // Version
        dict.set_item("version", 3)?;

        Ok(dict.into())
    }

    /// CUDA array interface for GPU tensor access
    ///
    /// This enables: torch.as_tensor(tensor_handle) for GPU tensors.
    /// Only valid for tensors in unified memory (Jetson) where `data_ptr()` returns
    /// a GPU-accessible pointer. CPU/pinned tensors cannot expose __cuda_array_interface__.
    #[getter]
    fn __cuda_array_interface__(&self, py: Python<'_>) -> PyResult<Py<PyAny>> {
        let handle = self
            .handle
            .as_ref()
            .ok_or_else(|| PyRuntimeError::new_err("TensorHandle has been released"))?;

        if !handle.is_cuda() {
            return Err(PyTypeError::new_err(
                "Tensor is not on CUDA device. Use __array_interface__ for CPU tensors.",
            ));
        }

        let tensor = handle.tensor();
        let dict = PyDict::new(py);

        // Shape
        let shape: Vec<i64> = handle.shape().iter().map(|&x| x as i64).collect();
        dict.set_item("shape", PyTuple::new(py, &shape)?)?;

        // Type string
        let typestr = dtype_numpy_typestr(tensor.dtype);
        dict.set_item("typestr", typestr)?;

        // Use handle.data_ptr() which goes through pool.data_ptr().
        // For unified memory (Jetson), this returns the cudaMallocManaged pointer
        // which IS a valid GPU pointer. For CPU/pinned tensors that somehow have
        // device=cuda set, we still return pool.data_ptr() — the caller marked it
        // as CUDA, so the pointer should be GPU-accessible.
        let gpu_ptr = handle.data_ptr() as usize;
        dict.set_item("data", (gpu_ptr, false))?;

        // Strides
        let strides: Vec<i64> = handle.strides().iter().map(|&x| x as i64).collect();
        dict.set_item("strides", PyTuple::new(py, &strides)?)?;

        // Stream (null = default stream)
        dict.set_item("stream", py.None())?;

        // Version
        dict.set_item("version", 3)?;

        Ok(dict.into())
    }

    /// DLPack export - returns a PyCapsule containing DLManagedTensor
    ///
    /// This enables: torch.from_dlpack(tensor_handle), np.from_dlpack(tensor_handle)
    ///
    /// Creates a real PyCapsule wrapping a C-ABI DLManagedTensor via horus_core::dlpack.
    /// No numpy/torch dependency required for export.
    ///
    /// Args:
    ///     stream: Optional CUDA stream for synchronization (ignored for CPU tensors)
    ///
    /// Returns:
    ///     PyCapsule containing DLManagedTensor
    #[pyo3(signature = (stream=None))]
    fn __dlpack__(&self, py: Python<'_>, stream: Option<i64>) -> PyResult<Py<PyAny>> {
        let handle = self
            .handle
            .as_ref()
            .ok_or_else(|| PyRuntimeError::new_err("TensorHandle has been released"))?;

        let tensor = handle.tensor();
        let _ = stream;

        let shape: Vec<i64> = handle.shape().iter().map(|&x| x as i64).collect();
        let elem_size = tensor.dtype.element_size() as i64;
        let strides: Vec<i64> = handle
            .strides()
            .iter()
            .map(|&x| (x as i64) / elem_size)
            .collect();

        // Use handle.data_ptr() for all backends — pool.data_ptr() returns the
        // correct pointer for mmap (CPU), cudaMallocManaged (unified/Jetson),
        // and cudaMallocHost (pinned). No special CUDA branch needed.
        let data_ptr = handle.data_ptr() as *mut c_void;

        dlpack_utils::make_dlpack_capsule(
            py,
            data_ptr,
            &shape,
            &strides,
            tensor.dtype,
            tensor.device(),
        )
    }

    /// DLPack device info - returns (device_type, device_id)
    ///
    /// This enables frameworks to check device compatibility before calling __dlpack__.
    ///
    /// Returns:
    ///     Tuple of (device_type: int, device_id: int)
    ///     device_type: 1=CPU, 2=CUDA
    fn __dlpack_device__(&self) -> PyResult<(i32, i32)> {
        let handle = self
            .handle
            .as_ref()
            .ok_or_else(|| PyRuntimeError::new_err("TensorHandle has been released"))?;

        let device = handle.tensor().device();
        Ok((device.to_dlpack_device_type(), device.to_dlpack_device_id()))
    }

    /// Import a DLPack tensor
    ///
    /// Creates a TensorHandle from any object that implements __dlpack__.
    /// Parses the PyCapsule directly via horus_core::dlpack — no numpy/torch required.
    ///
    /// Args:
    ///     obj: Object with __dlpack__ method (PyTorch tensor, JAX array, etc.)
    ///
    /// Returns:
    ///     TensorHandle wrapping a copy of the DLPack tensor data
    #[staticmethod]
    fn from_dlpack(py: Python<'_>, obj: Py<PyAny>) -> PyResult<Self> {
        // Get capsule from __dlpack__(stream=None)
        let capsule_obj = obj.call_method1(py, "__dlpack__", (py.None(),))?;

        // Extract DLManagedTensor* from PyCapsule
        let capsule_name = CString::new("dltensor").unwrap();
        let managed_ptr =
            unsafe { pyo3::ffi::PyCapsule_GetPointer(capsule_obj.as_ptr(), capsule_name.as_ptr()) };
        if managed_ptr.is_null() {
            return Err(PyRuntimeError::new_err(
                "Invalid DLPack capsule (not a dltensor or already consumed)",
            ));
        }

        // Parse tensor metadata via horus_core
        let descriptor = unsafe {
            horus_core::dlpack::from_dlpack(
                managed_ptr as *const horus_core::dlpack::DLManagedTensor,
            )
        }
        .map_err(|e| PyRuntimeError::new_err(format!("DLPack import failed: {}", e)))?;

        // Rename capsule to "used_dltensor" (DLPack spec: prevents double-consume)
        let used_name = CString::new("used_dltensor").unwrap();
        unsafe {
            pyo3::ffi::PyCapsule_SetName(capsule_obj.as_ptr(), used_name.as_ptr());
        }
        std::mem::forget(used_name);

        // Allocate HORUS TensorHandle and copy data from source.
        // GPU sources are handled based on hardware capability:
        //   - Jetson (unified): allocate with Device::cuda — managed memory is CPU+GPU accessible
        //   - Discrete GPU: allocate with Device::cpu in pinned pool — fast DMA staging
        //   - No GPU: allocate with Device::cpu in mmap pool
        let is_cuda_source = descriptor.device.is_cuda();
        let src_ptr = descriptor.data_ptr as *const u8;
        let copy_len = descriptor.size_bytes as usize;

        if is_cuda_source {
            return Err(PyRuntimeError::new_err(
                "Cannot import CUDA DLPack tensor: CUDA support is not available",
            ));
        }

        // CPU source — direct memory copy into default pool
        let pool = get_or_create_pool(1, None)?;
        let handle = TensorHandle::alloc(
            Arc::clone(&pool),
            &descriptor.shape,
            descriptor.dtype,
            horus_core::types::Device::cpu(),
        )
        .map_err(|e| PyRuntimeError::new_err(format!("Allocation failed: {}", e)))?;

        let dst_ptr = handle.data_ptr();
        unsafe {
            std::ptr::copy_nonoverlapping(src_ptr, dst_ptr, copy_len);
        }

        // Call deleter on the original DLManagedTensor (we're done with the data)
        unsafe {
            let managed = managed_ptr as *mut horus_core::dlpack::DLManagedTensor;
            if let Some(deleter) = (*managed).deleter {
                deleter(managed);
            }
        }

        Ok(Self {
            handle: Some(handle),
        })
    }

    /// Get tensor shape
    #[getter]
    fn shape(&self) -> PyResult<Vec<u64>> {
        let handle = self
            .handle
            .as_ref()
            .ok_or_else(|| PyRuntimeError::new_err("TensorHandle has been released"))?;
        Ok(handle.shape().to_vec())
    }

    /// Get tensor dtype as string
    #[getter]
    fn dtype(&self) -> PyResult<&'static str> {
        let handle = self
            .handle
            .as_ref()
            .ok_or_else(|| PyRuntimeError::new_err("TensorHandle has been released"))?;
        Ok(dtype_to_str(handle.dtype()))
    }

    /// Get tensor device as string
    #[getter]
    fn device(&self) -> PyResult<String> {
        let handle = self
            .handle
            .as_ref()
            .ok_or_else(|| PyRuntimeError::new_err("TensorHandle has been released"))?;
        Ok(device_to_string(handle.device()))
    }

    /// Get total number of elements
    #[getter]
    fn numel(&self) -> PyResult<u64> {
        let handle = self
            .handle
            .as_ref()
            .ok_or_else(|| PyRuntimeError::new_err("TensorHandle has been released"))?;
        Ok(handle.numel())
    }

    /// Get total size in bytes
    #[getter]
    fn nbytes(&self) -> PyResult<u64> {
        let handle = self
            .handle
            .as_ref()
            .ok_or_else(|| PyRuntimeError::new_err("TensorHandle has been released"))?;
        Ok(handle.nbytes())
    }

    /// Get reference count
    #[getter]
    fn refcount(&self) -> PyResult<u32> {
        let handle = self
            .handle
            .as_ref()
            .ok_or_else(|| PyRuntimeError::new_err("TensorHandle has been released"))?;
        Ok(handle.refcount())
    }

    /// Check if tensor is on CPU
    fn is_cpu(&self) -> PyResult<bool> {
        let handle = self
            .handle
            .as_ref()
            .ok_or_else(|| PyRuntimeError::new_err("TensorHandle has been released"))?;
        Ok(handle.is_cpu())
    }

    /// Check if tensor is on CUDA
    fn is_cuda(&self) -> PyResult<bool> {
        let handle = self
            .handle
            .as_ref()
            .ok_or_else(|| PyRuntimeError::new_err("TensorHandle has been released"))?;
        Ok(handle.is_cuda())
    }

    /// Check if tensor is contiguous
    fn is_contiguous(&self) -> PyResult<bool> {
        let handle = self
            .handle
            .as_ref()
            .ok_or_else(|| PyRuntimeError::new_err("TensorHandle has been released"))?;
        Ok(handle.is_contiguous())
    }

    /// Convert to numpy array (zero-copy if possible)
    ///
    /// Returns a numpy array that shares memory with this tensor.
    /// Changes to the numpy array will be visible in the tensor and vice versa.
    fn numpy<'py>(slf: &Bound<'py, Self>, py: Python<'py>) -> PyResult<Bound<'py, PyAny>> {
        let inner = slf.borrow();
        let handle = inner
            .handle
            .as_ref()
            .ok_or_else(|| PyRuntimeError::new_err("TensorHandle has been released"))?;

        if handle.is_cuda() {
            return Err(PyTypeError::new_err(
                "Cannot convert CUDA tensor to numpy. Use .cpu() first or access via torch.",
            ));
        }

        // Use numpy.asarray with our __array_interface__
        let np = py.import("numpy")?;
        np.call_method1("asarray", (slf,))
    }

    /// Convert to torch tensor (zero-copy if possible)
    ///
    /// Returns a PyTorch tensor that shares memory with this tensor.
    fn torch<'py>(slf: &Bound<'py, Self>, py: Python<'py>) -> PyResult<Bound<'py, PyAny>> {
        let inner = slf.borrow();
        let handle = inner
            .handle
            .as_ref()
            .ok_or_else(|| PyRuntimeError::new_err("TensorHandle has been released"))?;

        let torch = py.import("torch")?;

        if handle.is_cuda() {
            // Use __cuda_array_interface__
            torch.call_method1("as_tensor", (slf,))
        } else {
            // Use __array_interface__
            torch.call_method1("as_tensor", (slf,))
        }
    }

    /// Create a slice of the first dimension
    #[pyo3(signature = (start, end))]
    fn slice(&self, start: u64, end: u64) -> PyResult<Self> {
        let handle = self
            .handle
            .as_ref()
            .ok_or_else(|| PyRuntimeError::new_err("TensorHandle has been released"))?;

        let sliced = handle
            .slice_first_dim(start, end)
            .ok_or_else(|| PyValueError::new_err("Invalid slice range"))?;

        Ok(Self {
            handle: Some(sliced),
        })
    }

    /// Reshape the tensor (must be contiguous, same total elements)
    fn view(&self, new_shape: Vec<u64>) -> PyResult<Self> {
        let handle = self
            .handle
            .as_ref()
            .ok_or_else(|| PyRuntimeError::new_err("TensorHandle has been released"))?;

        let viewed = handle.view(&new_shape)
            .ok_or_else(|| PyValueError::new_err(
                "Cannot reshape: tensor must be contiguous and new shape must have same number of elements"
            ))?;

        Ok(Self {
            handle: Some(viewed),
        })
    }

    /// Copy tensor to CPU (returns new tensor)
    ///
    /// If already on CPU, returns a reference to self (no copy).
    /// If on CUDA with unified memory (Jetson), returns clone — managed memory
    /// is already CPU-accessible.
    /// If on CUDA with discrete GPU, copies data to CPU via cudaMemcpy(D2H).
    ///
    /// Note: Requires CUDA support to be enabled at compile time.
    fn cpu(&self) -> PyResult<Self> {
        let handle = self
            .handle
            .as_ref()
            .ok_or_else(|| PyRuntimeError::new_err("TensorHandle has been released"))?;

        if handle.is_cpu() {
            return Ok(Self {
                handle: Some(handle.clone()),
            });
        }

        Err(PyRuntimeError::new_err(
            "CUDA support not compiled. Rebuild with --features cuda or use torch.as_tensor(handle).cpu()"
        ))
    }

    /// Copy tensor to CUDA device (returns new tensor)
    ///
    /// Currently returns an error — CUDA support is not available.
    #[pyo3(signature = (device="cuda:0"))]
    fn cuda(&self, device: &str) -> PyResult<Self> {
        let handle = self
            .handle
            .as_ref()
            .ok_or_else(|| PyRuntimeError::new_err("TensorHandle has been released"))?;

        let target_device = parse_device(device)?;

        if handle.is_cuda() && handle.device() == target_device {
            return Ok(Self {
                handle: Some(handle.clone()),
            });
        }

        Err(PyRuntimeError::new_err(
            "CUDA support not compiled. Rebuild with --features cuda or use torch.as_tensor(handle).cuda()"
        ))
    }

    /// Release the tensor handle explicitly
    ///
    /// Normally handles are released when garbage collected, but this allows
    /// explicit early release.
    fn release(&mut self) {
        self.handle = None;
    }

    fn __repr__(&self) -> String {
        match &self.handle {
            Some(h) => format!(
                "TensorHandle(shape={:?}, dtype={}, device={}, refcount={})",
                h.shape(),
                dtype_to_str(h.dtype()),
                device_to_string(h.device()),
                h.refcount()
            ),
            None => "TensorHandle(released)".to_string(),
        }
    }

    fn __str__(&self) -> String {
        self.__repr__()
    }
}

// === Helper functions ===

// All helper functions (parse_dtype, dtype_to_str, parse_device, device_to_string,
// dlpack_capsule_destructor) are in crate::dlpack_utils to avoid duplication.
use dlpack_utils::{device_to_string, dtype_to_str, parse_device, parse_dtype};

fn dtype_numpy_typestr(dtype: TensorDtype) -> &'static str {
    dtype.numpy_typestr()
}

/// Check if CUDA is available
///
/// Returns True if the HORUS library was compiled with CUDA support
/// and at least one CUDA device is available.
#[pyfunction]
pub fn cuda_is_available() -> bool {
    horus::memory::cuda_available()
}

/// Get the number of available CUDA devices
///
/// Returns 0 if CUDA is not available or no devices are found.
#[pyfunction]
pub fn cuda_device_count() -> usize {
    horus::memory::cuda_device_count()
}

