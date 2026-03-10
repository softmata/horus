//! Custom Python exception types for structured error handling.
//!
//! Maps horus_core error types to catchable Python exceptions:
//! - `HorusNotFoundError` — missing topics, frames, nodes
//! - `HorusTransformError` — transform extrapolation, stale data
//! - `HorusTimeoutError` — blocking operation timeouts

use horus_core::error::HorusError;
use pyo3::create_exception;
use pyo3::exceptions::{PyException, PyRuntimeError};
use pyo3::PyErr;

create_exception!(
    _horus,
    HorusNotFoundError,
    PyException,
    "Raised when a topic, frame, or node is not found."
);
create_exception!(
    _horus,
    HorusTransformError,
    PyException,
    "Raised when a coordinate transform fails (extrapolation, stale data)."
);
create_exception!(
    _horus,
    HorusTimeoutError,
    PyException,
    "Raised when a blocking operation times out."
);

/// Convert a `HorusError` into the most specific Python exception.
pub fn to_py_err(err: HorusError) -> PyErr {
    match &err {
        HorusError::NotFound(_) => HorusNotFoundError::new_err(err.to_string()),
        HorusError::Transform(_) => HorusTransformError::new_err(err.to_string()),
        HorusError::Timeout(_) => HorusTimeoutError::new_err(err.to_string()),
        _ => PyRuntimeError::new_err(err.to_string()),
    }
}
