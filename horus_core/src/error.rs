//! Unified error handling for HORUS
//!
//! This module provides a centralized error type for the entire HORUS system,
//! ensuring consistent error handling across all components.

use thiserror::Error;

/// Main error type for HORUS operations
#[derive(Debug, Error)]
pub enum HorusError {
    /// I/O related errors
    #[error("IO error: {0}")]
    Io(#[from] std::io::Error),

    /// Configuration parsing or validation errors
    #[error("Configuration error: {0}")]
    Config(String),

    /// Communication layer errors
    #[error("Communication error: {0}")]
    Communication(String),

    /// Node-related errors
    #[error("Node '{node}' error: {message}")]
    Node { node: String, message: String },

    /// Memory management errors
    #[error("Memory error: {0}")]
    Memory(String),

    /// Serialization/Deserialization errors
    #[error("Serialization error: {0}")]
    Serialization(String),

    /// Resource not found errors
    #[error("Resource not found: {0}")]
    NotFound(String),

    /// Permission/Access errors
    #[error("Permission denied: {0}")]
    PermissionDenied(String),

    /// Invalid input/argument errors
    #[error("Invalid input: {0}")]
    InvalidInput(String),

    /// Already exists errors (for creation operations)
    #[error("Already exists: {0}")]
    AlreadyExists(String),

    /// Parse errors
    #[error("Parse error: {0}")]
    Parse(String),

    /// Operation not supported on this platform
    #[error("Unsupported: {0}")]
    Unsupported(String),

    /// Internal errors with source location for debugging.
    /// Use the `horus_internal!()` macro to create these — it captures file/line automatically.
    #[error("Internal error: {message} (at {file}:{line})")]
    Internal {
        message: String,
        file: &'static str,
        line: u32,
    },

    /// Error with preserved source chain for context propagation.
    /// Use `.horus_context()` on Results to create these.
    #[error("{message}\n  Caused by: {source}")]
    Contextual {
        message: String,
        #[source]
        source: Box<dyn std::error::Error + Send + Sync>,
    },
}

/// Create an internal error with automatic file/line capture.
///
/// ```rust,ignore
/// use horus_core::horus_internal;
/// return Err(horus_internal!("Unexpected state: {:?}", state));
/// // Expands to: HorusError::Internal { message: "...", file: "src/foo.rs", line: 42 }
/// ```
#[macro_export]
macro_rules! horus_internal {
    ($($arg:tt)*) => {
        $crate::error::HorusError::Internal {
            message: format!($($arg)*),
            file: file!(),
            line: line!(),
        }
    };
}

/// Convenience type alias for Results using HorusError
pub type HorusResult<T> = std::result::Result<T, HorusError>;

/// Short alias — `use horus::prelude::*` brings this into scope
pub type Result<T> = HorusResult<T>;

// ============================================
// From implementations for common error types
// ============================================

impl From<serde_json::Error> for HorusError {
    fn from(err: serde_json::Error) -> Self {
        HorusError::Serialization(err.to_string())
    }
}

impl From<toml::de::Error> for HorusError {
    fn from(err: toml::de::Error) -> Self {
        HorusError::Config(format!("TOML parse error: {}", err))
    }
}

impl From<toml::ser::Error> for HorusError {
    fn from(err: toml::ser::Error) -> Self {
        HorusError::Serialization(format!("TOML serialization error: {}", err))
    }
}

impl From<serde_yaml::Error> for HorusError {
    fn from(err: serde_yaml::Error) -> Self {
        HorusError::Serialization(format!("YAML error: {}", err))
    }
}

impl From<std::num::ParseIntError> for HorusError {
    fn from(err: std::num::ParseIntError) -> Self {
        HorusError::Parse(format!("Integer parse error: {}", err))
    }
}

impl From<std::num::ParseFloatError> for HorusError {
    fn from(err: std::num::ParseFloatError) -> Self {
        HorusError::Parse(format!("Float parse error: {}", err))
    }
}

impl From<std::str::ParseBoolError> for HorusError {
    fn from(err: std::str::ParseBoolError) -> Self {
        HorusError::Parse(format!("Boolean parse error: {}", err))
    }
}

impl From<uuid::Error> for HorusError {
    fn from(err: uuid::Error) -> Self {
        HorusError::Internal {
            message: format!("UUID error: {}", err),
            file: file!(),
            line: line!(),
        }
    }
}

impl<T> From<std::sync::PoisonError<T>> for HorusError {
    fn from(_: std::sync::PoisonError<T>) -> Self {
        HorusError::Internal {
            message: "Lock poisoned".to_string(),
            file: file!(),
            line: line!(),
        }
    }
}

impl From<Box<dyn std::error::Error>> for HorusError {
    fn from(err: Box<dyn std::error::Error>) -> Self {
        HorusError::Internal {
            message: err.to_string(),
            file: file!(),
            line: line!(),
        }
    }
}

impl From<Box<dyn std::error::Error + Send + Sync>> for HorusError {
    fn from(err: Box<dyn std::error::Error + Send + Sync>) -> Self {
        let message = err.to_string();
        HorusError::Contextual {
            message,
            source: err,
        }
    }
}

impl From<anyhow::Error> for HorusError {
    fn from(err: anyhow::Error) -> Self {
        HorusError::Internal {
            message: err.to_string(),
            file: file!(),
            line: line!(),
        }
    }
}

// NOTE: From<String> and From<&str> intentionally removed.
// Use specific error variants instead:
//   HorusError::config("msg")       — for config errors
//   HorusError::internal("msg")     — for internal errors (or horus_internal! macro)
//   HorusError::InvalidInput("msg") — for input errors
// This prevents accidental untyped errors via "string".into()

// Helper methods
impl HorusError {
    /// Create a configuration error with a custom message
    pub fn config<S: Into<String>>(msg: S) -> Self {
        HorusError::Config(msg.into())
    }

    /// Create a node error with node name and message
    pub fn node<S: Into<String>, T: Into<String>>(node: S, message: T) -> Self {
        HorusError::Node {
            node: node.into(),
            message: message.into(),
        }
    }

    /// Create a communication error
    pub fn communication<S: Into<String>>(msg: S) -> Self {
        HorusError::Communication(msg.into())
    }

    /// Create a memory error
    pub fn memory<S: Into<String>>(msg: S) -> Self {
        HorusError::Memory(msg.into())
    }

    /// Create an internal error (without file/line — prefer horus_internal! macro)
    pub fn internal<S: Into<String>>(msg: S) -> Self {
        HorusError::Internal {
            message: msg.into(),
            file: "unknown",
            line: 0,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::error::Error as StdError;

    // =========================================================================
    // Section 1: Every variant constructs and displays correctly
    // =========================================================================

    /// Io variant wraps std::io::Error and preserves the message.
    /// Robotics: "Failed to open /dev/ttyUSB0: Permission denied"
    #[test]
    fn variant_io() {
        let io_err = std::io::Error::new(
            std::io::ErrorKind::PermissionDenied,
            "cannot open /dev/ttyUSB0",
        );
        let err = HorusError::Io(io_err);
        let msg = format!("{}", err);
        assert!(msg.contains("IO error"), "Display: {}", msg);
        assert!(
            msg.contains("/dev/ttyUSB0"),
            "Should contain device path: {}",
            msg
        );
    }

    /// Config variant with descriptive message.
    /// Robotics: "Missing 'control_frequency' in robot.toml"
    #[test]
    fn variant_config() {
        let err = HorusError::Config("Missing 'control_frequency' in robot.toml".to_string());
        let msg = format!("{}", err);
        assert!(msg.contains("Configuration error"), "Display: {}", msg);
        assert!(
            msg.contains("control_frequency"),
            "Should contain field name: {}",
            msg
        );
    }

    /// Communication variant for IPC/topic failures.
    #[test]
    fn variant_communication() {
        let err = HorusError::Communication("Topic 'cmd_vel' has no subscribers".to_string());
        let msg = format!("{}", err);
        assert!(msg.contains("Communication error"), "Display: {}", msg);
        assert!(
            msg.contains("cmd_vel"),
            "Should contain topic name: {}",
            msg
        );
    }

    /// Node variant includes BOTH node name AND message.
    /// Robotics: crucial for identifying which of 20+ nodes failed.
    #[test]
    fn variant_node_includes_name_and_message() {
        let err = HorusError::Node {
            node: "imu_driver".to_string(),
            message: "I2C read timeout after 50ms".to_string(),
        };
        let msg = format!("{}", err);
        assert!(
            msg.contains("imu_driver"),
            "Must include node name: {}",
            msg
        );
        assert!(
            msg.contains("I2C read timeout"),
            "Must include message: {}",
            msg
        );
    }

    /// Memory variant for SHM/allocation failures.
    #[test]
    fn variant_memory() {
        let err =
            HorusError::Memory("Failed to mmap 4096 bytes for topic 'lidar_scan'".to_string());
        let msg = format!("{}", err);
        assert!(msg.contains("Memory error"), "Display: {}", msg);
        assert!(msg.contains("lidar_scan"), "Should contain topic: {}", msg);
    }

    /// Serialization variant for serde failures.
    #[test]
    fn variant_serialization() {
        let err = HorusError::Serialization("unexpected end of input at byte 42".to_string());
        let msg = format!("{}", err);
        assert!(msg.contains("Serialization error"), "Display: {}", msg);
    }

    /// NotFound variant for missing resources.
    #[test]
    fn variant_not_found() {
        let err = HorusError::NotFound("Node 'camera_rgb' not registered".to_string());
        let msg = format!("{}", err);
        assert!(msg.contains("not found"), "Display: {}", msg);
        assert!(
            msg.contains("camera_rgb"),
            "Should contain resource: {}",
            msg
        );
    }

    /// PermissionDenied variant.
    #[test]
    fn variant_permission_denied() {
        let err = HorusError::PermissionDenied("Cannot write to /dev/i2c-1".to_string());
        let msg = format!("{}", err);
        assert!(msg.contains("Permission denied"), "Display: {}", msg);
    }

    /// InvalidInput variant.
    #[test]
    fn variant_invalid_input() {
        let err = HorusError::InvalidInput("Frequency must be > 0, got -10".to_string());
        let msg = format!("{}", err);
        assert!(msg.contains("Invalid input"), "Display: {}", msg);
        assert!(msg.contains("-10"), "Should contain the bad value: {}", msg);
    }

    /// AlreadyExists variant.
    #[test]
    fn variant_already_exists() {
        let err = HorusError::AlreadyExists("Topic 'cmd_vel' already created".to_string());
        let msg = format!("{}", err);
        assert!(msg.contains("Already exists"), "Display: {}", msg);
    }

    /// Parse variant.
    #[test]
    fn variant_parse() {
        let err = HorusError::Parse("Expected float, got 'abc'".to_string());
        let msg = format!("{}", err);
        assert!(msg.contains("Parse error"), "Display: {}", msg);
    }

    /// Unsupported variant.
    #[test]
    fn variant_unsupported() {
        let err =
            HorusError::Unsupported("CUDA tensors not available on this platform".to_string());
        let msg = format!("{}", err);
        assert!(msg.contains("Unsupported"), "Display: {}", msg);
    }

    /// Internal variant includes file and line for debugging.
    #[test]
    fn variant_internal_includes_location() {
        let err = HorusError::Internal {
            message: "Unexpected dispatch state".to_string(),
            file: "src/topic/dispatch.rs",
            line: 42,
        };
        let msg = format!("{}", err);
        assert!(msg.contains("Internal error"), "Display: {}", msg);
        assert!(
            msg.contains("src/topic/dispatch.rs"),
            "Must include file: {}",
            msg
        );
        assert!(msg.contains("42"), "Must include line: {}", msg);
    }

    /// Contextual variant chains inner error.
    /// Robotics: "Failed to initialize IMU" caused by "Permission denied on /dev/i2c-1"
    #[test]
    fn variant_contextual_chains_errors() {
        let inner = std::io::Error::new(
            std::io::ErrorKind::PermissionDenied,
            "Permission denied on /dev/i2c-1",
        );
        let err = HorusError::Contextual {
            message: "Failed to initialize IMU".to_string(),
            source: Box::new(inner),
        };
        let msg = format!("{}", err);
        assert!(msg.contains("Failed to initialize IMU"), "Display: {}", msg);
        assert!(
            msg.contains("Permission denied on /dev/i2c-1"),
            "Must chain inner error: {}",
            msg
        );
        assert!(msg.contains("Caused by"), "Should show causation: {}", msg);

        // source() should return the inner error
        let src = err.source();
        assert!(src.is_some(), "Contextual must expose source()");
        assert!(src.unwrap().to_string().contains("Permission denied"));
    }

    // =========================================================================
    // Section 2: From implementations
    // =========================================================================

    /// From<std::io::Error> → HorusError::Io
    #[test]
    fn from_io_error() {
        let io_err = std::io::Error::new(std::io::ErrorKind::NotFound, "file not found");
        let err: HorusError = io_err.into();
        assert!(matches!(err, HorusError::Io(_)));
        assert!(format!("{}", err).contains("file not found"));
    }

    /// From<serde_json::Error> → HorusError::Serialization
    #[test]
    fn from_serde_json_error() {
        let json_err = serde_json::from_str::<serde_json::Value>("{{bad json").unwrap_err();
        let original_msg = json_err.to_string();
        let err: HorusError = json_err.into();
        assert!(matches!(err, HorusError::Serialization(_)));
        let msg = format!("{}", err);
        assert!(msg.contains("Serialization error"), "Display: {}", msg);
        // The original error info should be preserved somewhere
        assert!(
            msg.len() > "Serialization error: ".len(),
            "Should contain original error details"
        );
        // Verify original message is preserved
        if let HorusError::Serialization(s) = &err {
            assert_eq!(*s, original_msg);
        }
    }

    /// From<toml::de::Error> → HorusError::Config
    #[test]
    fn from_toml_de_error() {
        let toml_err: toml::de::Error = toml::from_str::<toml::Value>("{{bad").unwrap_err();
        let err: HorusError = toml_err.into();
        assert!(matches!(err, HorusError::Config(_)));
        let msg = format!("{}", err);
        assert!(msg.contains("TOML parse error"), "Display: {}", msg);
    }

    /// From<toml::ser::Error> → HorusError::Serialization
    #[test]
    fn from_toml_ser_error() {
        // TOML can't serialize a bare string as a top-level document (must be a table)
        let toml_err = toml::to_string("bare string").unwrap_err();
        let err: HorusError = toml_err.into();
        assert!(matches!(err, HorusError::Serialization(_)));
        let msg = format!("{}", err);
        assert!(msg.contains("TOML serialization error"), "Display: {}", msg);
    }

    /// From<serde_yaml::Error> → HorusError::Serialization
    #[test]
    fn from_yaml_error() {
        let yaml_err = serde_yaml::from_str::<serde_yaml::Value>(":\n  :\n    :bad").unwrap_err();
        let err: HorusError = yaml_err.into();
        assert!(matches!(err, HorusError::Serialization(_)));
        let msg = format!("{}", err);
        assert!(msg.contains("YAML error"), "Display: {}", msg);
    }

    /// From<ParseIntError> → HorusError::Parse
    #[test]
    fn from_parse_int_error() {
        let parse_err = "not_a_number".parse::<i32>().unwrap_err();
        let err: HorusError = parse_err.into();
        assert!(matches!(err, HorusError::Parse(_)));
        let msg = format!("{}", err);
        assert!(msg.contains("Integer parse error"), "Display: {}", msg);
    }

    /// From<ParseFloatError> → HorusError::Parse
    #[test]
    fn from_parse_float_error() {
        let parse_err = "not_a_float".parse::<f64>().unwrap_err();
        let err: HorusError = parse_err.into();
        assert!(matches!(err, HorusError::Parse(_)));
        let msg = format!("{}", err);
        assert!(msg.contains("Float parse error"), "Display: {}", msg);
    }

    /// From<ParseBoolError> → HorusError::Parse
    #[test]
    fn from_parse_bool_error() {
        let parse_err = "maybe".parse::<bool>().unwrap_err();
        let err: HorusError = parse_err.into();
        assert!(matches!(err, HorusError::Parse(_)));
        let msg = format!("{}", err);
        assert!(msg.contains("Boolean parse error"), "Display: {}", msg);
    }

    /// From<uuid::Error> → HorusError::Internal
    #[test]
    fn from_uuid_error() {
        let uuid_err = uuid::Uuid::parse_str("not-a-uuid").unwrap_err();
        let err: HorusError = uuid_err.into();
        assert!(matches!(err, HorusError::Internal { .. }));
        let msg = format!("{}", err);
        assert!(msg.contains("UUID error"), "Display: {}", msg);
    }

    /// From<PoisonError> → HorusError::Internal with "Lock poisoned"
    #[test]
    fn from_poison_error() {
        let mutex = std::sync::Mutex::new(42);
        // Poison the mutex by panicking inside a lock
        let _ = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
            let _guard = mutex.lock().unwrap();
            panic!("intentional poison");
        }));
        let poison_err = mutex.lock().unwrap_err();
        let err: HorusError = poison_err.into();
        assert!(matches!(err, HorusError::Internal { .. }));
        let msg = format!("{}", err);
        assert!(msg.contains("Lock poisoned"), "Display: {}", msg);
    }

    /// From<Box<dyn Error>> → HorusError::Internal
    #[test]
    fn from_boxed_error() {
        let boxed: Box<dyn std::error::Error> = Box::new(std::io::Error::new(
            std::io::ErrorKind::Other,
            "boxed io error",
        ));
        let err: HorusError = boxed.into();
        assert!(matches!(err, HorusError::Internal { .. }));
        let msg = format!("{}", err);
        assert!(msg.contains("boxed io error"), "Display: {}", msg);
    }

    /// From<Box<dyn Error + Send + Sync>> → HorusError::Contextual
    #[test]
    fn from_boxed_send_sync_error() {
        let boxed: Box<dyn std::error::Error + Send + Sync> = Box::new(std::io::Error::new(
            std::io::ErrorKind::Other,
            "send+sync io error",
        ));
        let err: HorusError = boxed.into();
        assert!(matches!(err, HorusError::Contextual { .. }));
        let msg = format!("{}", err);
        assert!(msg.contains("send+sync io error"), "Display: {}", msg);
    }

    /// From<anyhow::Error> → HorusError::Internal
    #[test]
    fn from_anyhow_error() {
        let anyhow_err = anyhow::anyhow!("anyhow test error");
        let err: HorusError = anyhow_err.into();
        assert!(matches!(err, HorusError::Internal { .. }));
        let msg = format!("{}", err);
        assert!(msg.contains("anyhow test error"), "Display: {}", msg);
    }

    // =========================================================================
    // Section 3: horus_internal! macro
    // =========================================================================

    /// horus_internal! captures the correct file and line of the call site.
    #[test]
    fn horus_internal_macro_captures_call_site() {
        let err = horus_internal!("Unexpected state: {}", "broken");
        let call_line = line!() - 1; // The macro was on the previous line
        match &err {
            HorusError::Internal {
                message,
                file,
                line,
            } => {
                assert!(
                    message.contains("Unexpected state: broken"),
                    "Message: {}",
                    message
                );
                assert!(
                    file.contains("error.rs"),
                    "File should be this test file: {}",
                    file
                );
                assert_eq!(*line, call_line, "Line should match call site");
            }
            _ => panic!("Expected Internal variant, got {:?}", err),
        }
    }

    /// horus_internal! with format arguments.
    #[test]
    fn horus_internal_macro_format_args() {
        let node_name = "motor_ctrl";
        let err = horus_internal!("Node {} failed with code {}", node_name, 42);
        let msg = format!("{}", err);
        assert!(
            msg.contains("Node motor_ctrl failed with code 42"),
            "Display: {}",
            msg
        );
    }

    // =========================================================================
    // Section 4: Helper methods
    // =========================================================================

    /// HorusError::config() helper.
    #[test]
    fn helper_config() {
        let err = HorusError::config("bad config");
        assert!(matches!(err, HorusError::Config(ref s) if s == "bad config"));
    }

    /// HorusError::node() helper includes both name and message.
    #[test]
    fn helper_node() {
        let err = HorusError::node("lidar", "timeout");
        match &err {
            HorusError::Node { node, message } => {
                assert_eq!(node, "lidar");
                assert_eq!(message, "timeout");
            }
            _ => panic!("Expected Node variant"),
        }
    }

    /// HorusError::communication() helper.
    #[test]
    fn helper_communication() {
        let err = HorusError::communication("topic full");
        assert!(matches!(err, HorusError::Communication(ref s) if s == "topic full"));
    }

    /// HorusError::memory() helper.
    #[test]
    fn helper_memory() {
        let err = HorusError::memory("mmap failed");
        assert!(matches!(err, HorusError::Memory(ref s) if s == "mmap failed"));
    }

    /// HorusError::internal() without macro uses "unknown" file and line 0.
    #[test]
    fn helper_internal_unknown_location() {
        let err = HorusError::internal("something broke");
        match &err {
            HorusError::Internal {
                message,
                file,
                line,
            } => {
                assert_eq!(message, "something broke");
                assert_eq!(*file, "unknown");
                assert_eq!(*line, 0);
            }
            _ => panic!("Expected Internal variant"),
        }
    }

    // =========================================================================
    // Section 5: HorusResult and ? operator
    // =========================================================================

    /// ? operator converts io::Error into HorusError::Io.
    #[test]
    fn question_mark_io_conversion() {
        fn might_fail() -> HorusResult<()> {
            let _ = std::fs::File::open("/nonexistent/path/that/should/not/exist")?;
            Ok(())
        }
        let result = might_fail();
        assert!(result.is_err());
        assert!(matches!(result.unwrap_err(), HorusError::Io(_)));
    }

    /// ? operator converts ParseIntError into HorusError::Parse.
    #[test]
    fn question_mark_parse_conversion() {
        fn parse_config() -> HorusResult<i32> {
            let val: i32 = "not_a_number".parse()?;
            Ok(val)
        }
        let result = parse_config();
        assert!(result.is_err());
        assert!(matches!(result.unwrap_err(), HorusError::Parse(_)));
    }
}
