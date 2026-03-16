//! Python bindings for [`DriverParams`] — typed access to driver config values.

use pyo3::exceptions::{PyKeyError, PyRuntimeError};
use pyo3::prelude::*;
use pyo3::IntoPyObjectExt;

use horus_core::drivers::DriverParams;

/// Typed access to driver config params from horus.toml.
///
/// Wraps the params from a `[drivers.NAME]` config table.  Values are
/// auto-converted from TOML types to Python types (str, int, float, bool, list).
#[pyclass(name = "DriverParams")]
pub struct PyDriverParams {
    pub(crate) inner: DriverParams,
}

#[pymethods]
impl PyDriverParams {
    /// Get a param value by key.
    ///
    /// Returns the value converted to the appropriate Python type.
    /// Raises ``KeyError`` if the key is not found.
    fn get(&self, py: Python<'_>, key: &str) -> PyResult<Py<PyAny>> {
        if !self.inner.has(key) {
            return Err(PyKeyError::new_err(format!("'{}'", key)));
        }
        toml_to_python(&self.inner, py, key)
    }

    /// Get a param value, returning *default* if not found.
    fn get_or(&self, py: Python<'_>, key: &str, default: Py<PyAny>) -> PyResult<Py<PyAny>> {
        if !self.inner.has(key) {
            return Ok(default);
        }
        toml_to_python(&self.inner, py, key).or(Ok(default))
    }

    /// Check if a param key exists.
    fn has(&self, key: &str) -> bool {
        self.inner.has(key)
    }

    /// Return all param keys.
    fn keys(&self) -> Vec<String> {
        self.inner.keys().map(|s| s.to_string()).collect()
    }

    fn __len__(&self) -> usize {
        self.inner.len()
    }

    fn __getitem__(&self, py: Python<'_>, key: &str) -> PyResult<Py<PyAny>> {
        self.get(py, key)
    }

    fn __contains__(&self, key: &str) -> bool {
        self.inner.has(key)
    }

    fn __repr__(&self) -> String {
        let mut keys: Vec<&str> = self.inner.keys().collect();
        keys.sort();
        format!("DriverParams({{{}}})", keys.join(", "))
    }
}

/// Convert a [`DriverParams`] value to a Python object by probing TOML types.
///
/// Order matters: bool before integer (distinct in TOML), integer before float
/// (since `f64::from_toml` accepts integers too).
fn toml_to_python(params: &DriverParams, py: Python<'_>, key: &str) -> PyResult<Py<PyAny>> {
    // Scalars
    if let Ok(v) = params.get::<bool>(key) {
        return v.into_py_any(py);
    }
    if let Ok(v) = params.get::<i64>(key) {
        return v.into_py_any(py);
    }
    if let Ok(v) = params.get::<f64>(key) {
        return v.into_py_any(py);
    }
    if let Ok(v) = params.get::<String>(key) {
        return v.into_py_any(py);
    }
    // Arrays (homogeneous, per TOML spec)
    if let Ok(v) = params.get::<Vec<i64>>(key) {
        return v.into_py_any(py);
    }
    if let Ok(v) = params.get::<Vec<f64>>(key) {
        return v.into_py_any(py);
    }
    if let Ok(v) = params.get::<Vec<String>>(key) {
        return v.into_py_any(py);
    }
    if let Ok(v) = params.get::<Vec<bool>>(key) {
        return v.into_py_any(py);
    }
    Err(PyRuntimeError::new_err(format!(
        "driver param '{}': unsupported TOML type",
        key
    )))
}
