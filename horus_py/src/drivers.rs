//! Python bindings for the drivers module.
//!
//! Provides `drivers.load()`, `PyHardwareSet`, and `drivers.register_driver()`
//! for Python-side driver configuration and hardware instantiation.

use std::collections::HashMap;
use std::sync::{LazyLock, Mutex};

use pyo3::exceptions::{PyKeyError, PyRuntimeError};
use pyo3::prelude::*;

use horus_core::drivers::{self, DriverHandle, DriverType, HardwareSet};
use horus_core::error::HorusError;

use crate::driver_params::PyDriverParams;
use crate::errors::to_py_err;

// ── Python driver class registry ───────────────────────────────────────

static PY_DRIVERS: LazyLock<Mutex<HashMap<String, Py<PyAny>>>> =
    LazyLock::new(|| Mutex::new(HashMap::new()));

// ── Module functions ───────────────────────────────────────────────────

/// Load hardware connections from ``horus.toml`` ``[drivers]`` section.
#[pyfunction]
pub fn load() -> PyResult<PyHardwareSet> {
    let hw = drivers::load().map_err(to_py_err)?;
    Ok(PyHardwareSet { inner: hw })
}

/// Load hardware connections from a specific config file path.
#[pyfunction]
pub fn load_from(path: &str) -> PyResult<PyHardwareSet> {
    let hw = drivers::load_from(path).map_err(to_py_err)?;
    Ok(PyHardwareSet { inner: hw })
}

/// Register a Python driver class by name.
///
/// The class will be instantiated with a ``DriverParams`` argument
/// when ``hw.local("name")`` is called.
#[pyfunction]
pub fn register_driver(name: String, cls: Py<PyAny>) {
    PY_DRIVERS
        .lock()
        .unwrap_or_else(|p| p.into_inner())
        .insert(name, cls);
}

// ── PyHardwareSet ──────────────────────────────────────────────────────

/// Pre-configured hardware connections loaded from ``horus.toml`` ``[drivers]``.
#[pyclass(name = "HardwareSet")]
pub struct PyHardwareSet {
    inner: HardwareSet,
}

#[pymethods]
impl PyHardwareSet {
    // ── Terra typed accessors ────────────────────────────────────────

    fn dynamixel(&mut self, name: &str) -> PyResult<PyDriverParams> {
        terra_handle(self.inner.dynamixel(name))
    }

    fn rplidar(&mut self, name: &str) -> PyResult<PyDriverParams> {
        terra_handle(self.inner.rplidar(name))
    }

    fn realsense(&mut self, name: &str) -> PyResult<PyDriverParams> {
        terra_handle(self.inner.realsense(name))
    }

    fn i2c(&mut self, name: &str) -> PyResult<PyDriverParams> {
        terra_handle(self.inner.i2c(name))
    }

    fn serial(&mut self, name: &str) -> PyResult<PyDriverParams> {
        terra_handle(self.inner.serial(name))
    }

    fn can(&mut self, name: &str) -> PyResult<PyDriverParams> {
        terra_handle(self.inner.can(name))
    }

    fn gpio(&mut self, name: &str) -> PyResult<PyDriverParams> {
        terra_handle(self.inner.gpio(name))
    }

    fn pwm(&mut self, name: &str) -> PyResult<PyDriverParams> {
        terra_handle(self.inner.pwm(name))
    }

    fn usb(&mut self, name: &str) -> PyResult<PyDriverParams> {
        terra_handle(self.inner.usb(name))
    }

    fn webcam(&mut self, name: &str) -> PyResult<PyDriverParams> {
        terra_handle(self.inner.webcam(name))
    }

    fn input(&mut self, name: &str) -> PyResult<PyDriverParams> {
        terra_handle(self.inner.input(name))
    }

    fn bluetooth(&mut self, name: &str) -> PyResult<PyDriverParams> {
        terra_handle(self.inner.bluetooth(name))
    }

    fn net(&mut self, name: &str) -> PyResult<PyDriverParams> {
        terra_handle(self.inner.net(name))
    }

    fn ethercat(&mut self, name: &str) -> PyResult<PyDriverParams> {
        terra_handle(self.inner.ethercat(name))
    }

    fn spi(&mut self, name: &str) -> PyResult<PyDriverParams> {
        terra_handle(self.inner.spi(name))
    }

    fn adc(&mut self, name: &str) -> PyResult<PyDriverParams> {
        terra_handle(self.inner.adc(name))
    }

    fn raw(&mut self, name: &str) -> PyResult<PyDriverParams> {
        terra_handle(self.inner.raw(name))
    }

    // ── Factory accessors ────────────────────────────────────────────

    /// Instantiate a local Python driver registered via ``register_driver()``.
    fn local(&mut self, py: Python<'_>, name: &str) -> PyResult<Py<PyAny>> {
        let params = self
            .inner
            .params(name)
            .ok_or_else(|| {
                PyKeyError::new_err(format!(
                    "driver '{}' not found in [drivers] config",
                    name
                ))
            })?
            .clone();

        let node_name = match self.inner.driver_type(name) {
            Some(DriverType::Local(n)) => n.clone(),
            _ => {
                return Err(PyRuntimeError::new_err(format!(
                    "driver '{}': not a local driver (expected node = ...)",
                    name
                )))
            }
        };

        // Clone Py<PyAny> out of lock before calling Python code.
        let cls = PY_DRIVERS
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .get(&node_name)
            .map(|c| c.clone_ref(py));

        if let Some(cls) = cls {
            let py_params = Py::new(py, PyDriverParams { inner: params })?;
            return cls.call1(py, (py_params,));
        }

        Err(PyRuntimeError::new_err(format!(
            "driver '{}': local node '{}' not registered in Python. \
             Use drivers.register_driver('{}', YourClass) first.",
            name, node_name, node_name
        )))
    }

    /// Get a driver as a node regardless of source type.
    fn node(&mut self, py: Python<'_>, name: &str) -> PyResult<Py<PyAny>> {
        let driver_type = self
            .inner
            .driver_type(name)
            .ok_or_else(|| {
                PyKeyError::new_err(format!("driver '{}' not found in [drivers] config", name))
            })?
            .clone();

        match driver_type {
            DriverType::Package(_) => self.package(py, name),
            DriverType::Local(_) => self.local(py, name),
            DriverType::Terra(ref t) => Err(PyRuntimeError::new_err(format!(
                "driver '{}': terra '{}' cannot be auto-wrapped as a Python node. \
                 Use the typed accessor (e.g., hw.dynamixel('{}')) and build your own Node class.",
                name, t, name
            ))),
            DriverType::Legacy => Err(PyRuntimeError::new_err(format!(
                "driver '{}': legacy driver cannot be instantiated as a node",
                name
            ))),
        }
    }

    /// Instantiate a driver from a registry package.
    fn package(&mut self, py: Python<'_>, name: &str) -> PyResult<Py<PyAny>> {
        let params = self
            .inner
            .params(name)
            .ok_or_else(|| {
                PyKeyError::new_err(format!(
                    "driver '{}' not found in [drivers] config",
                    name
                ))
            })?
            .clone();

        let pkg_name = match self.inner.driver_type(name) {
            Some(DriverType::Package(p)) => p.clone(),
            _ => {
                return Err(PyRuntimeError::new_err(format!(
                    "driver '{}': not a package driver (expected package = ...)",
                    name
                )))
            }
        };

        let cls = PY_DRIVERS
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .get(&pkg_name)
            .map(|c| c.clone_ref(py));

        if let Some(cls) = cls {
            let py_params = Py::new(py, PyDriverParams { inner: params })?;
            return cls.call1(py, (py_params,));
        }

        Err(PyRuntimeError::new_err(format!(
            "driver '{}': package '{}' not registered in Python. \
             Use drivers.register_driver('{}', YourClass) first, \
             or ensure the package is installed via `horus add {}`.",
            name, pkg_name, pkg_name, pkg_name
        )))
    }

    // ── Introspection ────────────────────────────────────────────────

    /// List all configured driver names (sorted).
    fn list(&self) -> Vec<String> {
        self.inner
            .list()
            .into_iter()
            .map(|s| s.to_string())
            .collect()
    }

    /// Check if a driver is configured.
    fn has(&self, name: &str) -> bool {
        self.inner.has(name)
    }

    /// Get config params for a driver by name.
    fn params(&self, name: &str) -> PyResult<PyDriverParams> {
        let params = self
            .inner
            .params(name)
            .ok_or_else(|| PyKeyError::new_err(format!("driver '{}' not found", name)))?;
        Ok(PyDriverParams {
            inner: params.clone(),
        })
    }

    fn __len__(&self) -> usize {
        self.inner.len()
    }

    fn __contains__(&self, name: &str) -> bool {
        self.inner.has(name)
    }

    fn __repr__(&self) -> String {
        let names = self.inner.list();
        format!("HardwareSet([{}])", names.join(", "))
    }
}

// ── Helpers ────────────────────────────────────────────────────────────

fn terra_handle(result: Result<DriverHandle, HorusError>) -> PyResult<PyDriverParams> {
    let handle = result.map_err(to_py_err)?;
    Ok(PyDriverParams {
        inner: handle.params().clone(),
    })
}

// ── Submodule registration ─────────────────────────────────────────────

pub fn register_drivers_module(parent: &Bound<'_, PyModule>) -> PyResult<()> {
    let drivers = PyModule::new(parent.py(), "drivers")?;

    drivers.add_function(wrap_pyfunction!(load, &drivers)?)?;
    drivers.add_function(wrap_pyfunction!(load_from, &drivers)?)?;
    drivers.add_function(wrap_pyfunction!(register_driver, &drivers)?)?;
    drivers.add_class::<PyHardwareSet>()?;
    drivers.add_class::<PyDriverParams>()?;

    drivers.setattr(
        "__doc__",
        "HORUS driver configuration and hardware connection support.",
    )?;

    parent.add_submodule(&drivers)?;

    parent
        .py()
        .import("sys")?
        .getattr("modules")?
        .set_item("horus._horus.drivers", &drivers)?;

    Ok(())
}
