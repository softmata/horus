//! Python bindings for HORUS RuntimeParams
//!
//! Exposes the Rust RuntimeParams key-value store to Python for
//! dynamic runtime configuration (PID tuning, sensor rates, etc.)

use horus_core::params::RuntimeParams;
use pyo3::exceptions::PyRuntimeError;
use pyo3::prelude::*;
use pyo3::types::PyDict;
use serde_json::Value;
use std::path::PathBuf;

/// Runtime parameter store for dynamic configuration.
///
/// Provides a typed key-value store with validation, persistence, and
/// concurrent access support. Parameters can be loaded from YAML files
/// and saved back.
///
/// Examples:
///     params = RuntimeParams()
///     params.set("max_speed", 1.5)
///     speed = params.get("max_speed")     # Returns 1.5
///     speed = params.get_or("max_speed", 2.0)  # With default
///
///     params.save()                        # Save to .horus/config/params.yaml
///     params.load("config/custom.yaml")    # Load from file
#[pyclass(name = "RuntimeParams")]
pub struct PyRuntimeParams {
    inner: RuntimeParams,
}

#[pymethods]
impl PyRuntimeParams {
    /// Create a new RuntimeParams store.
    ///
    /// Loads from `.horus/config/params.yaml` if present, otherwise uses defaults.
    #[new]
    fn new() -> PyResult<Self> {
        let inner = RuntimeParams::new()
            .map_err(|e| PyRuntimeError::new_err(format!("Failed to init params: {}", e)))?;
        Ok(Self { inner })
    }

    /// Get a parameter value, returns None if not found.
    ///
    /// Automatically converts to the appropriate Python type (int, float, str, bool, list, dict).
    ///
    /// Args:
    ///     key: Parameter name
    ///
    /// Returns:
    ///     Parameter value or None
    fn get(&self, py: Python, key: &str) -> PyResult<PyObject> {
        match self.inner.get::<Value>(key) {
            Some(value) => json_value_to_python(py, &value),
            None => Ok(py.None()),
        }
    }

    /// Get a parameter value with a default.
    ///
    /// Args:
    ///     key: Parameter name
    ///     default: Value to return if key doesn't exist
    ///
    /// Returns:
    ///     Parameter value or default
    fn get_or(&self, py: Python, key: &str, default: PyObject) -> PyResult<PyObject> {
        match self.inner.get::<Value>(key) {
            Some(value) => json_value_to_python(py, &value),
            None => Ok(default),
        }
    }

    /// Set a parameter value.
    ///
    /// Accepts int, float, str, bool, list, and dict values.
    ///
    /// Args:
    ///     key: Parameter name
    ///     value: Parameter value
    ///
    /// Raises:
    ///     RuntimeError: If validation fails or parameter is read-only
    fn set(&self, py: Python, key: &str, value: PyObject) -> PyResult<()> {
        let json_value = python_to_json_value(py, &value)?;
        self.inner
            .set(key, json_value)
            .map_err(|e| PyRuntimeError::new_err(e.to_string()))
    }

    /// Check if a parameter exists.
    fn has(&self, key: &str) -> bool {
        self.inner.has(key)
    }

    /// Remove a parameter.
    ///
    /// Returns True if the parameter existed, False otherwise.
    fn remove(&self, key: &str) -> bool {
        self.inner.remove(key).is_some()
    }

    /// List all parameter keys (sorted).
    fn keys(&self) -> Vec<String> {
        self.inner.list_keys()
    }

    /// Get all parameters as a dict.
    fn get_all(&self, py: Python) -> PyResult<PyObject> {
        let all = self.inner.get_all();
        let dict = PyDict::new(py);
        for (key, value) in &all {
            dict.set_item(key, json_value_to_python(py, value)?)?;
        }
        Ok(dict.into())
    }

    /// Reset all parameters to defaults.
    fn reset(&self) -> PyResult<()> {
        self.inner
            .reset()
            .map_err(|e| PyRuntimeError::new_err(e.to_string()))
    }

    /// Save parameters to disk (.horus/config/params.yaml).
    fn save(&self) -> PyResult<()> {
        self.inner
            .save_to_disk()
            .map_err(|e| PyRuntimeError::new_err(format!("Failed to save: {}", e)))
    }

    /// Load parameters from a YAML file.
    ///
    /// Args:
    ///     path: Path to YAML file
    fn load(&self, path: &str) -> PyResult<()> {
        self.inner
            .load_from_disk(&PathBuf::from(path))
            .map_err(|e| PyRuntimeError::new_err(format!("Failed to load: {}", e)))
    }

    /// Get the version number for a parameter (for optimistic locking).
    fn get_version(&self, key: &str) -> u64 {
        self.inner.get_version(key)
    }

    /// Set a parameter only if its version matches (optimistic locking).
    ///
    /// Args:
    ///     key: Parameter name
    ///     value: New value
    ///     expected_version: Version number that must match current version
    ///
    /// Raises:
    ///     RuntimeError: If version mismatch or validation fails
    fn set_with_version(
        &self,
        py: Python,
        key: &str,
        value: PyObject,
        expected_version: u64,
    ) -> PyResult<()> {
        let json_value = python_to_json_value(py, &value)?;
        self.inner
            .set_with_version(key, json_value, expected_version)
            .map_err(|e| PyRuntimeError::new_err(e.to_string()))
    }

    fn __repr__(&self) -> String {
        let keys = self.inner.list_keys();
        format!("RuntimeParams({} parameters)", keys.len())
    }

    fn __len__(&self) -> usize {
        self.inner.list_keys().len()
    }

    fn __contains__(&self, key: &str) -> bool {
        self.inner.has(key)
    }
}

/// Convert a serde_json::Value to a Python object.
fn json_value_to_python(py: Python, value: &Value) -> PyResult<PyObject> {
    match value {
        Value::Null => Ok(py.None()),
        Value::Bool(b) => Ok(b.into_pyobject(py)?.to_owned().into_any().unbind()),
        Value::Number(n) => {
            if let Some(i) = n.as_i64() {
                Ok(i.into_pyobject(py)?.into_any().unbind())
            } else if let Some(f) = n.as_f64() {
                Ok(f.into_pyobject(py)?.into_any().unbind())
            } else {
                Ok(py.None())
            }
        }
        Value::String(s) => Ok(s.into_pyobject(py)?.into_any().unbind()),
        Value::Array(arr) => {
            let items: Vec<PyObject> = arr
                .iter()
                .map(|v| json_value_to_python(py, v))
                .collect::<PyResult<_>>()?;
            Ok(items.into_pyobject(py)?.into_any().unbind())
        }
        Value::Object(map) => {
            let dict = PyDict::new(py);
            for (k, v) in map {
                dict.set_item(k, json_value_to_python(py, v)?)?;
            }
            Ok(dict.into_pyobject(py)?.into_any().unbind())
        }
    }
}

/// Convert a Python object to a serde_json::Value.
fn python_to_json_value(py: Python, obj: &PyObject) -> PyResult<Value> {
    let bound = obj.bind(py);

    if bound.is_none() {
        return Ok(Value::Null);
    }
    if let Ok(b) = bound.extract::<bool>() {
        return Ok(Value::Bool(b));
    }
    if let Ok(i) = bound.extract::<i64>() {
        return Ok(Value::Number(i.into()));
    }
    if let Ok(f) = bound.extract::<f64>() {
        return Ok(serde_json::Number::from_f64(f)
            .map(Value::Number)
            .unwrap_or(Value::Null));
    }
    if let Ok(s) = bound.extract::<String>() {
        return Ok(Value::String(s));
    }
    if let Ok(list) = bound.extract::<Vec<PyObject>>() {
        let items: Vec<Value> = list
            .iter()
            .map(|item| python_to_json_value(py, item))
            .collect::<PyResult<_>>()?;
        return Ok(Value::Array(items));
    }
    // Try dict
    if let Ok(dict) = bound.downcast::<PyDict>() {
        let mut map = serde_json::Map::new();
        for (k, v) in dict.iter() {
            let key: String = k.extract()?;
            let val = python_to_json_value(py, &v.into())?;
            map.insert(key, val);
        }
        return Ok(Value::Object(map));
    }

    Err(PyRuntimeError::new_err(format!(
        "Cannot convert {} to parameter value",
        bound.get_type().name()?
    )))
}
