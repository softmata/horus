//! Custom Python exception types for structured error handling.
//!
//! Maps horus_core error types to catchable Python exceptions:
//! - `HorusNotFoundError` — missing topics, frames, nodes
//! - `HorusTransformError` — transform extrapolation, stale data
//! - `HorusTimeoutError` — blocking operation timeouts

use horus_core::error::HorusError;
use pyo3::create_exception;
use pyo3::exceptions::{
    PyException, PyIOError, PyMemoryError, PyRuntimeError, PyTypeError, PyValueError,
};
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
        HorusError::Io(_) => PyIOError::new_err(err.to_string()),
        HorusError::Memory(_) => PyMemoryError::new_err(err.to_string()),
        HorusError::InvalidInput(_) => PyValueError::new_err(err.to_string()),
        HorusError::InvalidDescriptor(_) => PyValueError::new_err(err.to_string()),
        HorusError::Parse(_) => PyValueError::new_err(err.to_string()),
        HorusError::Serialization(_) => PyTypeError::new_err(err.to_string()),
        HorusError::Config(_) => PyValueError::new_err(err.to_string()),
        _ => PyRuntimeError::new_err(err.to_string()),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use horus_core::error::{NotFoundError, TransformError};

    /// Verify to_py_err routes each HorusError variant to the correct exception.
    /// These tests avoid Python::attach since the test binary doesn't link libpython.
    /// Instead, they verify the conversion doesn't panic and the error message is preserved.

    #[test]
    fn not_found_topic_converts() {
        let err = HorusError::NotFound(NotFoundError::Topic {
            name: "cmd_vel".into(),
        });
        assert!(err.to_string().contains("cmd_vel"));
        let _py_err = to_py_err(err);
    }

    #[test]
    fn not_found_frame_converts() {
        let err = HorusError::NotFound(NotFoundError::Frame {
            name: "base_link".into(),
        });
        assert!(err.to_string().contains("base_link"));
        let _py_err = to_py_err(err);
    }

    #[test]
    fn not_found_node_converts() {
        let err = HorusError::NotFound(NotFoundError::Node {
            name: "motor_ctrl".into(),
        });
        assert!(err.to_string().contains("motor_ctrl"));
        let _py_err = to_py_err(err);
    }

    #[test]
    fn not_found_parent_frame_converts() {
        let err = HorusError::NotFound(NotFoundError::ParentFrame {
            name: "world".into(),
        });
        let _py_err = to_py_err(err);
    }

    #[test]
    fn transform_extrapolation_converts() {
        let err = HorusError::Transform(TransformError::Extrapolation {
            frame: "lidar".into(),
            requested_ns: 100,
            oldest_ns: 200,
            newest_ns: 300,
        });
        assert!(err.to_string().contains("lidar"));
        let _py_err = to_py_err(err);
    }

    #[test]
    fn transform_stale_converts() {
        let err = HorusError::Transform(TransformError::Stale {
            frame: "odom".into(),
            age: std::time::Duration::from_secs(5),
            threshold: std::time::Duration::from_secs(1),
        });
        assert!(err.to_string().contains("odom"));
        let _py_err = to_py_err(err);
    }

    #[test]
    fn timeout_converts() {
        let err = HorusError::Timeout(horus_core::error::TimeoutError {
            resource: "imu_topic".into(),
            elapsed: std::time::Duration::from_millis(100),
            deadline: Some(std::time::Duration::from_millis(50)),
        });
        assert!(err.to_string().contains("imu_topic"));
        let _py_err = to_py_err(err);
    }

    #[test]
    fn io_error_falls_through() {
        let err = HorusError::Io(std::io::Error::new(
            std::io::ErrorKind::NotFound,
            "file missing",
        ));
        let _py_err = to_py_err(err);
    }

    #[test]
    fn internal_error_falls_through() {
        let err = HorusError::Internal {
            message: "something broke".into(),
            file: file!(),
            line: line!(),
        };
        assert!(err.to_string().contains("something broke"));
        let _py_err = to_py_err(err);
    }

    #[test]
    fn invalid_descriptor_falls_through() {
        let err = HorusError::InvalidDescriptor("bad tensor".into());
        let _py_err = to_py_err(err);
    }
}
