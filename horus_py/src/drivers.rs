//! Python bindings for the hardware module.
//!
//! Provides `hardware.load()`, `hardware.register()`, and `NodeParams`
//! for Python-side hardware configuration.

use std::collections::HashMap;
use std::sync::{LazyLock, Mutex};

use pyo3::exceptions::PyRuntimeError;
use pyo3::prelude::*;

use crate::driver_params::PyDriverParams;

// ── Python driver class registry ───────────────────────────────────────

static PY_DRIVERS: LazyLock<Mutex<HashMap<String, Py<PyAny>>>> =
    LazyLock::new(|| Mutex::new(HashMap::new()));

// ── Module functions ───────────────────────────────────────────────────

/// Load hardware config from ``horus.toml`` ``[hardware]`` section.
///
/// Returns a list of ``(name, obj)`` tuples where ``obj`` is either:
/// - A Python object (if a matching class was registered via ``register()``)
/// - A ``NodeParams`` dict-like (if no Python class was registered)
///
/// For Rust-only node types, the NodeParams lets Python code inspect
/// the config without instantiating the node.
#[pyfunction]
pub fn load(py: Python<'_>) -> PyResult<Vec<(String, Py<PyAny>)>> {
    let path =
        horus_core::drivers::find_manifest().map_err(|e| PyRuntimeError::new_err(e.to_string()))?;
    let path_str = path.to_str().ok_or_else(|| {
        PyRuntimeError::new_err(format!(
            "horus.toml path contains non-UTF8 characters: {:?}",
            path
        ))
    })?;
    load_from(py, path_str)
}

/// Load hardware config from a specific config file path.
#[pyfunction]
pub fn load_from(py: Python<'_>, path: &str) -> PyResult<Vec<(String, Py<PyAny>)>> {
    // Use horus_core to parse the config and get (name, NodeParams) pairs
    let entries = horus_core::drivers::load_config_entries(path)
        .map_err(|e| PyRuntimeError::new_err(e.to_string()))?;

    let mut result = Vec::new();

    for (name, use_name, params) in entries {
        // Try to instantiate from Python registry
        let cls = PY_DRIVERS
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .get(&use_name)
            .map(|c| c.clone_ref(py));

        if let Some(cls) = cls {
            let py_params = Py::new(py, PyDriverParams { inner: params })?;
            let node = cls.call1(py, (py_params,))?;
            result.push((name, node));
        } else {
            // No Python class registered — return NodeParams
            let py_params = Py::new(py, PyDriverParams { inner: params })?;
            result.push((name, py_params.into_any()));
        }
    }

    Ok(result)
}

/// Register a Python node class by name.
///
/// The class will be instantiated with a ``NodeParams`` argument
/// when ``hardware.load()`` finds a matching ``use`` field.
#[pyfunction]
pub fn register_driver(name: String, cls: Py<PyAny>) {
    PY_DRIVERS
        .lock()
        .unwrap_or_else(|p| p.into_inner())
        .insert(name, cls);
}

// ── Submodule registration ─────────────────────────────────────────────

pub fn register_drivers_module(parent: &Bound<'_, PyModule>) -> PyResult<()> {
    let hardware = PyModule::new(parent.py(), "hardware")?;

    hardware.add_function(wrap_pyfunction!(load, &hardware)?)?;
    hardware.add_function(wrap_pyfunction!(load_from, &hardware)?)?;
    hardware.add_function(wrap_pyfunction!(register_driver, &hardware)?)?;
    hardware.add_class::<PyDriverParams>()?;

    hardware.setattr(
        "__doc__",
        "HORUS hardware configuration and node loading support.",
    )?;

    parent.add_submodule(&hardware)?;

    // Register as both hardware and drivers (backward compat)
    let sys_modules = parent.py().import("sys")?.getattr("modules")?;
    sys_modules.set_item("horus._horus.hardware", &hardware)?;
    sys_modules.set_item("horus._horus.drivers", &hardware)?;

    Ok(())
}
