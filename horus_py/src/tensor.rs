//! Python bindings for HORUS tensor system
//!
//! Provides zero-copy tensor access from Python via numpy's `__array_interface__`
//! and the buffer protocol.

use horus::memory::tensor_pool::{HorusTensor, TensorDevice, TensorDtype};
use horus::memory::{TensorHandle, TensorPool, TensorPoolConfig};
use parking_lot::RwLock;
use pyo3::exceptions::{PyRuntimeError, PyTypeError, PyValueError};
use pyo3::prelude::*;
use pyo3::types::{PyDict, PyTuple};
use std::collections::HashMap;
use std::sync::Arc;

// Global pool registry - allows multiple pools to be accessed by ID
lazy_static::lazy_static! {
    static ref POOL_REGISTRY: RwLock<HashMap<u32, Arc<TensorPool>>> = RwLock::new(HashMap::new());
}

/// Get or create a tensor pool by ID
fn get_or_create_pool(pool_id: u32, config: Option<TensorPoolConfig>) -> PyResult<Arc<TensorPool>> {
    // Check if pool already exists
    {
        let registry = POOL_REGISTRY.read();
        if let Some(pool) = registry.get(&pool_id) {
            return Ok(Arc::clone(pool));
        }
    }

    // Create new pool
    let config = config.unwrap_or_default();
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

        let tensor: HorusTensor =
            unsafe { std::ptr::read(descriptor_bytes.as_ptr() as *const HorusTensor) };

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
    /// This enables: torch.as_tensor(tensor_handle) for GPU tensors
    /// The GPU pointer is extracted from the cuda_ipc_handle field.
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

        // Data pointer (GPU address from IPC handle)
        // The cuda_ipc_handle stores the GPU pointer in the first 8 bytes (little-endian)
        let gpu_ptr = u64::from_le_bytes(tensor.cuda_ipc_handle[..8].try_into().unwrap());
        dict.set_item("data", (gpu_ptr as usize, false))?;

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
    /// This enables: torch.from_dlpack(tensor_handle)
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
        let _ = stream; // Stream synchronization not implemented yet

        // Convert shape to i64
        let shape: Vec<i64> = handle.shape().iter().map(|&x| x as i64).collect();

        // Convert strides from bytes to elements
        let elem_size = tensor.dtype.element_size() as i64;
        let strides: Vec<i64> = handle
            .strides()
            .iter()
            .map(|&x| (x as i64) / elem_size)
            .collect();

        // Get dtype in DLPack format (code, bits, lanes)
        let (dl_code, dl_bits, dl_lanes) = dtype_to_dlpack(tensor.dtype);

        // Get device type and id
        let (device_type, device_id) = device_to_dlpack(tensor.device);

        // Get data pointer
        let data_ptr = if handle.is_cuda() {
            // For CUDA, extract pointer from IPC handle
            u64::from_le_bytes(tensor.cuda_ipc_handle[..8].try_into().unwrap()) as usize
        } else {
            handle.data_ptr() as usize
        };

        // Create PyCapsule via Python ctypes
        // We'll create the DLManagedTensor structure in Python for simplicity
        let _ctypes = py.import("ctypes")?;

        // Create the capsule using a helper function
        let _dlpack_module = py.import("builtins")?;

        // Build DLManagedTensor dict representation for Python
        let dl_tensor = PyDict::new(py);
        dl_tensor.set_item("data", data_ptr)?;
        dl_tensor.set_item("device_type", device_type)?;
        dl_tensor.set_item("device_id", device_id)?;
        dl_tensor.set_item("ndim", shape.len())?;
        dl_tensor.set_item("dtype_code", dl_code)?;
        dl_tensor.set_item("dtype_bits", dl_bits)?;
        dl_tensor.set_item("dtype_lanes", dl_lanes)?;
        dl_tensor.set_item("shape", shape)?;
        dl_tensor.set_item("strides", strides)?;
        dl_tensor.set_item("byte_offset", 0u64)?;

        // Use numpy's DLPack support if available, otherwise return dict
        // This is a simplified implementation - full implementation would use PyCapsule
        match py.import("numpy") {
            Ok(np) => {
                // Try to use numpy's __dlpack__ path via as_tensor
                if handle.is_cuda() {
                    // For CUDA, use torch if available
                    match py.import("torch") {
                        Ok(torch) => {
                            // Create tensor from cuda array interface, then get dlpack
                            let cuda_iface = self.__cuda_array_interface__(py)?;
                            let t = torch.call_method1("as_tensor", (cuda_iface,))?;
                            Ok(t.call_method1("__dlpack__", (stream,))?.unbind())
                        }
                        Err(_) => Err(PyRuntimeError::new_err(
                            "CUDA DLPack export requires PyTorch. Install torch or use __cuda_array_interface__."
                        ))
                    }
                } else {
                    // For CPU, create numpy array and get dlpack from it
                    let arr = np.call_method1("asarray", (self.__array_interface__(py)?,))?;
                    Ok(arr.call_method1("__dlpack__", ())?.unbind())
                }
            }
            Err(_) => Err(PyRuntimeError::new_err(
                "DLPack export requires numpy. Install numpy.",
            )),
        }
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

        let tensor = handle.tensor();
        Ok(device_to_dlpack(tensor.device))
    }

    /// Import a DLPack tensor
    ///
    /// Creates a TensorHandle from any object that implements __dlpack__.
    ///
    /// Args:
    ///     obj: Object with __dlpack__ method (PyTorch tensor, JAX array, etc.)
    ///
    /// Returns:
    ///     TensorHandle wrapping the DLPack tensor data
    #[staticmethod]
    fn from_dlpack(py: Python<'_>, obj: Py<PyAny>) -> PyResult<Self> {
        // Try to call __dlpack__ on the object to verify it supports DLPack
        let _capsule = obj.call_method1(py, "__dlpack__", ())?;

        // Get device info
        let device_info: (i32, i32) = obj.call_method0(py, "__dlpack_device__")?.extract(py)?;
        let (device_type, device_id) = device_info;

        // For now, convert via numpy/torch for simplicity
        // Full implementation would parse the PyCapsule directly
        if device_type == 1 {
            // CPU - use numpy
            let np = py.import("numpy")?;
            let arr = np.call_method1("from_dlpack", (obj,))?;

            // Get array info
            let shape: Vec<u64> = arr
                .getattr("shape")?
                .extract::<Vec<i64>>()?
                .iter()
                .map(|&x| x as u64)
                .collect();
            let dtype_str: String = arr.getattr("dtype")?.call_method0("__str__")?.extract()?;

            // Create a pool and allocate tensor
            let pool = get_or_create_pool(1, None)?;
            let dtype = parse_dtype(&dtype_str)?;
            let handle =
                TensorHandle::alloc(Arc::clone(&pool), &shape, dtype, TensorDevice::Cpu)
                    .map_err(|e| PyRuntimeError::new_err(format!("Allocation failed: {}", e)))?;

            // Copy data
            let np_copy = np.getattr("copyto")?;
            let dest_arr = np.call_method1(
                "asarray",
                (PyTensorHandle {
                    handle: Some(handle.clone()),
                },),
            )?;
            np_copy.call1((dest_arr, arr))?;

            Ok(Self {
                handle: Some(handle),
            })
        } else if device_type == 2 {
            // CUDA - use torch
            let torch = py.import("torch")?;
            let t = torch.call_method1("from_dlpack", (obj,))?;

            // Get tensor info
            let shape: Vec<u64> = t
                .getattr("shape")?
                .extract::<Vec<i64>>()?
                .iter()
                .map(|&x| x as u64)
                .collect();
            let dtype_str: String = t.getattr("dtype")?.call_method0("__str__")?.extract()?;
            let dtype_str = dtype_str.replace("torch.", "");

            // Map torch dtype to our dtype
            let dtype = parse_dtype(&dtype_str)?;
            let device = match device_id {
                0 => TensorDevice::Cuda0,
                1 => TensorDevice::Cuda1,
                2 => TensorDevice::Cuda2,
                3 => TensorDevice::Cuda3,
                _ => {
                    return Err(PyValueError::new_err(format!(
                        "Unsupported CUDA device: {}",
                        device_id
                    )))
                }
            };

            // Create pool and allocate
            let pool = get_or_create_pool(1, None)?;
            let handle = TensorHandle::alloc(Arc::clone(&pool), &shape, dtype, device)
                .map_err(|e| PyRuntimeError::new_err(format!("Allocation failed: {}", e)))?;

            // Copy via torch
            let dest_t = torch.call_method1(
                "as_tensor",
                (PyTensorHandle {
                    handle: Some(handle.clone()),
                },),
            )?;
            dest_t.call_method1("copy_", (t,))?;

            Ok(Self {
                handle: Some(handle),
            })
        } else {
            Err(PyValueError::new_err(format!(
                "Unsupported device type: {}",
                device_type
            )))
        }
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
    fn device(&self) -> PyResult<&'static str> {
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
    /// If on CUDA, copies data to CPU memory.
    ///
    /// Note: Requires CUDA support to be enabled at compile time.
    #[cfg(feature = "cuda")]
    fn cpu(&self) -> PyResult<Self> {
        use horus_core::memory::cuda_ffi::{memcpy, CudaMemcpyKind};

        let handle = self
            .handle
            .as_ref()
            .ok_or_else(|| PyRuntimeError::new_err("TensorHandle has been released"))?;

        if handle.is_cpu() {
            // Already on CPU - return clone (increments refcount)
            return Ok(Self {
                handle: Some(handle.clone()),
            });
        }

        // CUDA -> CPU transfer
        let tensor = handle.tensor();
        let shape: Vec<u64> = handle.shape().to_vec();
        let pool = handle.pool();

        // Allocate CPU tensor
        let cpu_handle = TensorHandle::alloc(pool.clone(), &shape, tensor.dtype, TensorDevice::Cpu)
            .map_err(|e| PyRuntimeError::new_err(format!("CPU allocation failed: {}", e)))?;

        // Get GPU pointer from IPC handle (first 8 bytes store the pointer)
        let gpu_ptr = u64::from_le_bytes(tensor.cuda_ipc_handle[..8].try_into().unwrap())
            as *const std::ffi::c_void;
        let cpu_ptr = cpu_handle.data_ptr() as *mut std::ffi::c_void;

        // Copy GPU -> CPU
        memcpy(
            cpu_ptr,
            gpu_ptr,
            handle.nbytes() as usize,
            CudaMemcpyKind::DeviceToHost,
        )
        .map_err(|e| PyRuntimeError::new_err(format!("CUDA memcpy failed: {}", e)))?;

        Ok(Self {
            handle: Some(cpu_handle),
        })
    }

    #[cfg(not(feature = "cuda"))]
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
    /// If already on CUDA, returns a reference to self (no copy).
    /// If on CPU, copies data to GPU memory.
    ///
    /// Args:
    ///     device: Target device string (default: "cuda:0")
    ///
    /// Note: Requires CUDA support to be enabled at compile time.
    #[cfg(feature = "cuda")]
    #[pyo3(signature = (device="cuda:0"))]
    fn cuda(&self, device: &str) -> PyResult<Self> {
        use horus_core::memory::cuda_ffi::{
            ipc_get_mem_handle, malloc, memcpy, set_device, CudaMemcpyKind,
        };

        let handle = self
            .handle
            .as_ref()
            .ok_or_else(|| PyRuntimeError::new_err("TensorHandle has been released"))?;

        let target_device = parse_device(device)?;

        if handle.is_cuda() && handle.device() == target_device {
            // Already on target CUDA device - return clone
            return Ok(Self {
                handle: Some(handle.clone()),
            });
        }

        // Get device ID from target
        let device_id = match target_device {
            TensorDevice::Cuda0 => 0,
            TensorDevice::Cuda1 => 1,
            TensorDevice::Cuda2 => 2,
            TensorDevice::Cuda3 => 3,
            _ => return Err(PyValueError::new_err("Invalid CUDA device")),
        };

        // Set CUDA device
        set_device(device_id)
            .map_err(|e| PyRuntimeError::new_err(format!("Failed to set CUDA device: {}", e)))?;

        let tensor = handle.tensor();
        let shape: Vec<u64> = handle.shape().to_vec();
        let pool = handle.pool();
        let nbytes = handle.nbytes() as usize;

        // Allocate GPU memory
        let gpu_ptr = malloc(nbytes)
            .map_err(|e| PyRuntimeError::new_err(format!("CUDA malloc failed: {}", e)))?;

        // Copy data to GPU
        if handle.is_cpu() {
            // CPU -> GPU
            let cpu_ptr = handle.data_ptr() as *const std::ffi::c_void;
            memcpy(gpu_ptr, cpu_ptr, nbytes, CudaMemcpyKind::HostToDevice)
                .map_err(|e| PyRuntimeError::new_err(format!("CUDA memcpy failed: {}", e)))?;
        } else {
            // GPU -> GPU (different device)
            let src_gpu_ptr = u64::from_le_bytes(tensor.cuda_ipc_handle[..8].try_into().unwrap())
                as *const std::ffi::c_void;
            memcpy(gpu_ptr, src_gpu_ptr, nbytes, CudaMemcpyKind::DeviceToDevice)
                .map_err(|e| PyRuntimeError::new_err(format!("CUDA memcpy failed: {}", e)))?;
        }

        // Get IPC handle for the new GPU memory
        let ipc_handle = ipc_get_mem_handle(gpu_ptr)
            .map_err(|e| PyRuntimeError::new_err(format!("Failed to get IPC handle: {}", e)))?;

        // Allocate a tensor descriptor in the pool (for metadata tracking)
        let mut gpu_handle = TensorHandle::alloc(pool.clone(), &shape, tensor.dtype, target_device)
            .map_err(|e| PyRuntimeError::new_err(format!("Tensor allocation failed: {}", e)))?;

        // Store the IPC handle and GPU pointer in the tensor
        // Note: This modifies the tensor's cuda_ipc_handle field
        {
            let gpu_tensor = gpu_handle.tensor_mut();
            gpu_tensor
                .cuda_ipc_handle
                .copy_from_slice(&ipc_handle.reserved);
            // Store GPU pointer in first 8 bytes for quick access
            gpu_tensor.cuda_ipc_handle[..8].copy_from_slice(&(gpu_ptr as u64).to_le_bytes());
        }

        Ok(Self {
            handle: Some(gpu_handle),
        })
    }

    #[cfg(not(feature = "cuda"))]
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

    /// Get the cuda_ipc_handle bytes (for debugging/advanced use)
    ///
    /// Returns the 64-byte IPC handle that enables cross-process GPU memory sharing.
    fn get_cuda_ipc_handle(&self) -> PyResult<Vec<u8>> {
        let handle = self
            .handle
            .as_ref()
            .ok_or_else(|| PyRuntimeError::new_err("TensorHandle has been released"))?;

        if !handle.is_cuda() {
            return Err(PyTypeError::new_err("Tensor is not on CUDA device"));
        }

        Ok(handle.tensor().cuda_ipc_handle.to_vec())
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

fn parse_dtype(s: &str) -> PyResult<TensorDtype> {
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

fn dtype_to_str(dtype: TensorDtype) -> &'static str {
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

fn parse_device(s: &str) -> PyResult<TensorDevice> {
    match s.to_lowercase().as_str() {
        "cpu" => Ok(TensorDevice::Cpu),
        "cuda" | "cuda:0" | "gpu" | "gpu:0" => Ok(TensorDevice::Cuda0),
        "cuda:1" | "gpu:1" => Ok(TensorDevice::Cuda1),
        "cuda:2" | "gpu:2" => Ok(TensorDevice::Cuda2),
        "cuda:3" | "gpu:3" => Ok(TensorDevice::Cuda3),
        _ => Err(PyValueError::new_err(format!("Unknown device: {}", s))),
    }
}

fn dtype_numpy_typestr(dtype: TensorDtype) -> &'static str {
    match dtype {
        TensorDtype::F32 => "<f4",
        TensorDtype::F64 => "<f8",
        TensorDtype::F16 => "<f2",
        TensorDtype::BF16 => "<V2", // bfloat16 not directly supported in numpy
        TensorDtype::I8 => "|i1",
        TensorDtype::I16 => "<i2",
        TensorDtype::I32 => "<i4",
        TensorDtype::I64 => "<i8",
        TensorDtype::U8 => "|u1",
        TensorDtype::U16 => "<u2",
        TensorDtype::U32 => "<u4",
        TensorDtype::U64 => "<u8",
        TensorDtype::Bool => "|b1",
    }
}

fn device_to_string(device: TensorDevice) -> &'static str {
    match device {
        TensorDevice::Cpu => "cpu",
        TensorDevice::Cuda0 => "cuda:0",
        TensorDevice::Cuda1 => "cuda:1",
        TensorDevice::Cuda2 => "cuda:2",
        TensorDevice::Cuda3 => "cuda:3",
    }
}

/// Convert TensorDtype to DLPack format (code, bits, lanes)
///
/// DLPack dtype codes:
/// - 0 = kDLInt (signed integer)
/// - 1 = kDLUInt (unsigned integer)
/// - 2 = kDLFloat (IEEE floating point)
/// - 4 = kDLBfloat (bfloat16)
fn dtype_to_dlpack(dtype: TensorDtype) -> (u8, u8, u16) {
    match dtype {
        // Float types (code=2)
        TensorDtype::F16 => (2, 16, 1),
        TensorDtype::F32 => (2, 32, 1),
        TensorDtype::F64 => (2, 64, 1),
        // BFloat16 (code=4)
        TensorDtype::BF16 => (4, 16, 1),
        // Signed integers (code=0)
        TensorDtype::I8 => (0, 8, 1),
        TensorDtype::I16 => (0, 16, 1),
        TensorDtype::I32 => (0, 32, 1),
        TensorDtype::I64 => (0, 64, 1),
        // Unsigned integers (code=1)
        TensorDtype::U8 => (1, 8, 1),
        TensorDtype::U16 => (1, 16, 1),
        TensorDtype::U32 => (1, 32, 1),
        TensorDtype::U64 => (1, 64, 1),
        // Bool (unsigned 8-bit)
        TensorDtype::Bool => (1, 8, 1),
    }
}

/// Convert TensorDevice to DLPack format (device_type, device_id)
///
/// DLPack device types:
/// - 1 = kDLCPU
/// - 2 = kDLCUDA
fn device_to_dlpack(device: TensorDevice) -> (i32, i32) {
    match device {
        TensorDevice::Cpu => (1, 0),   // kDLCPU = 1
        TensorDevice::Cuda0 => (2, 0), // kDLCUDA = 2, device 0
        TensorDevice::Cuda1 => (2, 1), // kDLCUDA = 2, device 1
        TensorDevice::Cuda2 => (2, 2), // kDLCUDA = 2, device 2
        TensorDevice::Cuda3 => (2, 3), // kDLCUDA = 2, device 3
    }
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

/// Register tensor classes with the Python module
pub fn register_tensor_classes(m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add_class::<PyTensorPool>()?;
    m.add_class::<PyTensorHandle>()?;
    m.add_function(wrap_pyfunction!(cuda_is_available, m)?)?;
    m.add_function(wrap_pyfunction!(cuda_device_count, m)?)?;
    Ok(())
}
