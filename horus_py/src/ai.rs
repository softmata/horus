//! horus.ai - AI/ML tensor utilities for HORUS
//!
//! This module provides a Python-friendly API for working with tensors
//! and AI/ML frameworks like PyTorch, JAX, TensorFlow, and NumPy.
//!
//! The main features are:
//! - Zero-copy tensor interoperability via DLPack
//! - Device management (CPU, CUDA)
//! - Dtype utilities
//!
//! Example:
//! ```python
//! import horus
//! import torch
//!
//! # Create a HORUS tensor from PyTorch
//! pt_tensor = torch.randn(3, 224, 224, device='cuda')
//! horus_tensor = horus.TensorHandle.from_dlpack(pt_tensor)
//!
//! # Send via HORUS topic (zero-copy)
//! topic.publish(horus_tensor)
//!
//! # Convert back to PyTorch
//! received = topic.subscribe()
//! pt_result = torch.from_dlpack(received)
//! ```

use pyo3::prelude::*;
use pyo3::types::PyDict;

/// Supported tensor data types
#[pyclass(name = "dtype")]
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PyDtype {
    code: u8,
    bits: u8,
}

#[pymethods]
impl PyDtype {
    /// Float16 (half precision)
    #[classattr]
    fn float16() -> Self {
        Self { code: 2, bits: 16 }
    }

    /// Float32 (single precision)
    #[classattr]
    fn float32() -> Self {
        Self { code: 2, bits: 32 }
    }

    /// Float64 (double precision)
    #[classattr]
    fn float64() -> Self {
        Self { code: 2, bits: 64 }
    }

    /// BFloat16 (brain floating point)
    #[classattr]
    fn bfloat16() -> Self {
        Self { code: 4, bits: 16 }
    }

    /// Int8
    #[classattr]
    fn int8() -> Self {
        Self { code: 0, bits: 8 }
    }

    /// Int16
    #[classattr]
    fn int16() -> Self {
        Self { code: 0, bits: 16 }
    }

    /// Int32
    #[classattr]
    fn int32() -> Self {
        Self { code: 0, bits: 32 }
    }

    /// Int64
    #[classattr]
    fn int64() -> Self {
        Self { code: 0, bits: 64 }
    }

    /// Uint8
    #[classattr]
    fn uint8() -> Self {
        Self { code: 1, bits: 8 }
    }

    /// Uint16
    #[classattr]
    fn uint16() -> Self {
        Self { code: 1, bits: 16 }
    }

    /// Uint32
    #[classattr]
    fn uint32() -> Self {
        Self { code: 1, bits: 32 }
    }

    /// Uint64
    #[classattr]
    fn uint64() -> Self {
        Self { code: 1, bits: 64 }
    }

    /// Bool (stored as uint8)
    #[classattr]
    fn bool_() -> Self {
        Self { code: 1, bits: 8 }
    }

    /// Get DLPack type code
    #[getter]
    fn code(&self) -> u8 {
        self.code
    }

    /// Get bits per element
    #[getter]
    fn bits(&self) -> u8 {
        self.bits
    }

    /// Get bytes per element
    #[getter]
    fn itemsize(&self) -> usize {
        (self.bits / 8) as usize
    }

    fn __repr__(&self) -> String {
        match (self.code, self.bits) {
            (2, 16) => "dtype.float16".to_string(),
            (2, 32) => "dtype.float32".to_string(),
            (2, 64) => "dtype.float64".to_string(),
            (4, 16) => "dtype.bfloat16".to_string(),
            (0, 8) => "dtype.int8".to_string(),
            (0, 16) => "dtype.int16".to_string(),
            (0, 32) => "dtype.int32".to_string(),
            (0, 64) => "dtype.int64".to_string(),
            (1, 8) => "dtype.uint8".to_string(),
            (1, 16) => "dtype.uint16".to_string(),
            (1, 32) => "dtype.uint32".to_string(),
            (1, 64) => "dtype.uint64".to_string(),
            _ => format!("dtype(code={}, bits={})", self.code, self.bits),
        }
    }
}

/// Device type for tensor placement
#[pyclass(name = "Device")]
#[derive(Debug, Clone)]
pub struct PyDevice {
    device_type: String,
    device_id: i32,
}

#[pymethods]
impl PyDevice {
    /// Create a CPU device
    #[staticmethod]
    fn cpu() -> Self {
        Self {
            device_type: "cpu".to_string(),
            device_id: 0,
        }
    }

    /// Create a CUDA device
    #[staticmethod]
    #[pyo3(signature = (device_id=0))]
    fn cuda(device_id: i32) -> Self {
        Self {
            device_type: "cuda".to_string(),
            device_id,
        }
    }

    /// Parse device from string ("cpu", "cuda:0", etc.)
    #[staticmethod]
    fn parse(s: &str) -> PyResult<Self> {
        let s = s.to_lowercase();
        if s == "cpu" {
            Ok(Self::cpu())
        } else if s.starts_with("cuda") {
            let id = if s.contains(':') {
                s.split(':').nth(1)
                    .and_then(|id| id.parse().ok())
                    .unwrap_or(0)
            } else {
                0
            };
            Ok(Self::cuda(id))
        } else {
            Err(pyo3::exceptions::PyValueError::new_err(format!(
                "Unknown device: {}. Use 'cpu' or 'cuda:N'", s
            )))
        }
    }

    /// Check if device is CPU
    fn is_cpu(&self) -> bool {
        self.device_type == "cpu"
    }

    /// Check if device is CUDA
    fn is_cuda(&self) -> bool {
        self.device_type == "cuda"
    }

    /// Get device type string
    #[getter]
    fn type_(&self) -> &str {
        &self.device_type
    }

    /// Get device index
    #[getter]
    fn index(&self) -> i32 {
        self.device_id
    }

    fn __repr__(&self) -> String {
        if self.device_type == "cpu" {
            "Device('cpu')".to_string()
        } else {
            format!("Device('{}:{}')", self.device_type, self.device_id)
        }
    }

    fn __str__(&self) -> String {
        if self.device_type == "cpu" {
            "cpu".to_string()
        } else {
            format!("{}:{}", self.device_type, self.device_id)
        }
    }
}

/// Get DLPack version info
#[pyfunction]
fn dlpack_version() -> PyResult<PyObject> {
    Python::with_gil(|py| {
        let dict = PyDict::new(py);
        dict.set_item("version", "0.8")?;
        dict.set_item("features", vec!["cpu", "cuda", "dlpack"])?;
        Ok(dict.into())
    })
}

/// Check if DLPack is supported
#[pyfunction]
fn dlpack_supported() -> bool {
    true
}

/// Register AI module classes
pub fn register_ai_module(parent: &Bound<'_, PyModule>) -> PyResult<()> {
    // Create ai submodule
    let ai = PyModule::new(parent.py(), "ai")?;

    ai.add_class::<PyDtype>()?;
    ai.add_class::<PyDevice>()?;
    ai.add_function(wrap_pyfunction!(dlpack_version, &ai)?)?;
    ai.add_function(wrap_pyfunction!(dlpack_supported, &ai)?)?;

    // Add docstring
    ai.setattr("__doc__", "HORUS AI/ML tensor utilities with DLPack support")?;

    // Add to parent module
    parent.add_submodule(&ai)?;

    Ok(())
}
