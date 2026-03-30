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

/// Build the error message, appending actionable `.help()` text when available.
///
/// This ensures Python users see the same remediation hints that Rust callers get
/// (e.g., "Run: horus topic list" or "Increase ring buffer capacity").
fn format_with_help(err: &HorusError) -> String {
    let msg = err.to_string();
    match err.help() {
        Some(hint) => format!("{}\n  hint: {}", msg, hint),
        None => msg,
    }
}

/// Convert a `HorusError` into the most specific Python exception.
///
/// The exception message includes the original error AND any actionable hint
/// from [`HorusError::help()`], so Python users and AI agents can self-fix.
pub fn to_py_err(err: HorusError) -> PyErr {
    let msg = format_with_help(&err);
    match &err {
        HorusError::NotFound(_) => HorusNotFoundError::new_err(msg),
        HorusError::Transform(_) => HorusTransformError::new_err(msg),
        HorusError::Timeout(_) => HorusTimeoutError::new_err(msg),
        HorusError::Io(_) => PyIOError::new_err(msg),
        HorusError::Memory(_) => PyMemoryError::new_err(msg),
        HorusError::InvalidInput(_) => PyValueError::new_err(msg),
        HorusError::InvalidDescriptor(_) => PyValueError::new_err(msg),
        HorusError::Parse(_) => PyValueError::new_err(msg),
        HorusError::Serialization(_) => PyTypeError::new_err(msg),
        HorusError::Config(_) => PyValueError::new_err(msg),
        _ => PyRuntimeError::new_err(msg),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use horus_core::core::DurationExt;
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
        // Verify help text is included in the formatted message
        let formatted = format_with_help(&err);
        assert!(formatted.contains("cmd_vel"), "should contain topic name");
        assert!(formatted.contains("hint:"), "should contain hint prefix");
        assert!(
            formatted.contains("horus topic list"),
            "should contain diagnostic command"
        );
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
        assert!(err.to_string().contains("world"));
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
        // Verify help text includes tf_at() suggestion
        let formatted = format_with_help(&err);
        assert!(formatted.contains("hint:"), "should contain hint prefix");
        assert!(
            formatted.contains("tf_at()"),
            "should suggest tf_at() for clamped lookup"
        );
        let _py_err = to_py_err(err);
    }

    #[test]
    fn transform_stale_converts() {
        let err = HorusError::Transform(TransformError::Stale {
            frame: "odom".into(),
            age: 5_u64.secs(),
            threshold: 1_u64.secs(),
        });
        assert!(err.to_string().contains("odom"));
        let _py_err = to_py_err(err);
    }

    #[test]
    fn timeout_converts() {
        let err = HorusError::Timeout(horus_core::error::TimeoutError {
            resource: "imu_topic".into(),
            elapsed: 100_u64.ms(),
            deadline: Some(50_u64.ms()),
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
        assert!(err.to_string().contains("file missing"));
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
        assert!(err.to_string().contains("bad tensor"));
        let _py_err = to_py_err(err);
    }

    #[test]
    fn topic_full_includes_help() {
        let err = HorusError::Communication(horus_core::error::CommunicationError::TopicFull {
            topic: "cmd_vel".into(),
        });
        let formatted = format_with_help(&err);
        assert!(formatted.contains("cmd_vel"), "should contain topic name");
        assert!(formatted.contains("hint:"), "should contain hint prefix");
        assert!(
            formatted.contains("ring buffer capacity"),
            "should suggest increasing ring buffer capacity"
        );
    }

    #[test]
    fn format_with_help_no_hint_returns_plain_message() {
        // Io errors have no specific help() text
        let err = HorusError::Io(std::io::Error::other("generic io"));
        let formatted = format_with_help(&err);
        assert!(
            !formatted.contains("hint:"),
            "should NOT contain hint for Io errors"
        );
        assert!(
            formatted.contains("generic io"),
            "should contain original message"
        );
    }

    #[test]
    fn topic_not_found_includes_help() {
        let err = HorusError::NotFound(NotFoundError::Topic {
            name: "odom".into(),
        });
        let formatted = format_with_help(&err);
        assert!(formatted.contains("odom"));
        assert!(formatted.contains("hint:"));
        assert!(formatted.contains("horus topic list"));
    }

    #[test]
    fn stale_transform_includes_help() {
        let err = HorusError::Transform(TransformError::Stale {
            frame: "camera".into(),
            age: 5_u64.secs(),
            threshold: 1_u64.secs(),
        });
        let formatted = format_with_help(&err);
        assert!(formatted.contains("camera"));
        assert!(formatted.contains("hint:"));
        assert!(formatted.contains("sensor driver"));
    }
}
