//! Python bindings for [`RuntimeParams`] — dynamic runtime configuration.

use pyo3::exceptions::{PyKeyError, PyRuntimeError};
use pyo3::prelude::*;

use horus_core::params::RuntimeParams;

use crate::errors::to_py_err;

/// Runtime parameter store with dict-like access.
///
/// Loads defaults from ``.horus/config/params.yaml`` or built-in defaults.
/// Values are auto-converted between Python and JSON types.
///
/// Example::
///
///     p = Params()
///     kp = p["pid_kp"]          # KeyError if missing
///     kp = p.get("pid_kp", 1.0) # default if missing
///     p["pid_kp"] = 2.5
///     p.save()
#[pyclass(name = "Params")]
pub struct PyParams {
    inner: RuntimeParams,
}

#[pymethods]
impl PyParams {
    /// Create a parameter store.
    ///
    /// With no arguments, loads from ``.horus/config/params.yaml`` or defaults.
    /// With a path, loads from that YAML file.
    #[new]
    #[pyo3(signature = (path=None))]
    fn new(path: Option<&str>) -> PyResult<Self> {
        let params = RuntimeParams::new().map_err(to_py_err)?;
        if let Some(p) = path {
            params
                .load_from_disk(std::path::Path::new(p))
                .map_err(to_py_err)?;
        }
        Ok(PyParams { inner: params })
    }

    /// Get a parameter value, returning *default* if not found.
    ///
    /// Returns ``None`` if key is missing and no default is given.
    #[pyo3(signature = (key, default=None))]
    fn get(&self, py: Python<'_>, key: &str, default: Option<Py<PyAny>>) -> PyResult<Py<PyAny>> {
        let all = self.inner.get_all();
        match all.get(key) {
            Some(val) => pythonize::pythonize(py, val)
                .map(|b| b.unbind())
                .map_err(|e| PyRuntimeError::new_err(e.to_string())),
            None => Ok(default.unwrap_or_else(|| py.None())),
        }
    }

    /// Get a parameter value by key. Raises ``KeyError`` if missing.
    fn __getitem__(&self, py: Python<'_>, key: &str) -> PyResult<Py<PyAny>> {
        let all = self.inner.get_all();
        match all.get(key) {
            Some(val) => pythonize::pythonize(py, val)
                .map(|b| b.unbind())
                .map_err(|e| PyRuntimeError::new_err(e.to_string())),
            None => Err(PyKeyError::new_err(format!("'{}'", key))),
        }
    }

    /// Set a parameter value.
    fn __setitem__(&self, py: Python<'_>, key: &str, value: Py<PyAny>) -> PyResult<()> {
        let json_val: serde_json::Value = pythonize::depythonize(value.bind(py))
            .map_err(|e| PyRuntimeError::new_err(e.to_string()))?;
        self.inner.set(key, json_val).map_err(to_py_err)
    }

    fn __contains__(&self, key: &str) -> bool {
        self.inner.has(key)
    }

    fn __len__(&self) -> usize {
        self.inner.list_keys().len()
    }

    /// Check if a parameter exists.
    fn has(&self, key: &str) -> bool {
        self.inner.has(key)
    }

    /// Return all parameter keys (sorted).
    fn keys(&self) -> Vec<String> {
        self.inner.list_keys()
    }

    /// Persist parameters to disk (``.horus/config/params.yaml``).
    fn save(&self) -> PyResult<()> {
        self.inner.save_to_disk().map_err(to_py_err)
    }

    /// Remove a parameter. Returns ``True`` if it existed.
    fn remove(&self, key: &str) -> bool {
        self.inner.remove(key).is_some()
    }

    /// Reset all parameters to built-in defaults.
    fn reset(&self) -> PyResult<()> {
        self.inner.reset().map_err(to_py_err)
    }

    fn __repr__(&self) -> String {
        format!("Params({} keys)", self.inner.list_keys().len())
    }

    fn __iter__(&self) -> PyParamsKeyIter {
        PyParamsKeyIter {
            keys: self.inner.list_keys(),
            index: 0,
        }
    }
}

/// Iterator over parameter keys (for ``for key in params``).
#[pyclass]
struct PyParamsKeyIter {
    keys: Vec<String>,
    index: usize,
}

#[pymethods]
impl PyParamsKeyIter {
    fn __iter__(slf: PyRef<'_, Self>) -> PyRef<'_, Self> {
        slf
    }

    fn __next__(&mut self) -> Option<String> {
        if self.index < self.keys.len() {
            let key = self.keys[self.index].clone();
            self.index += 1;
            Some(key)
        } else {
            None
        }
    }
}
