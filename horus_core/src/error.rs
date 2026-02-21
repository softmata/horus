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

    /// Backend-specific errors
    #[error("Backend '{backend}' error: {message}")]
    Backend { backend: String, message: String },

    /// Communication layer errors
    #[error("Communication error: {0}")]
    Communication(String),

    /// Node-related errors
    #[error("Node '{node}' error: {message}")]
    Node { node: String, message: String },

    /// Driver-related errors
    #[error("Driver error: {0}")]
    Driver(String),

    /// Scheduling errors
    #[error("Scheduling error: {0}")]
    Scheduling(String),

    /// Memory management errors
    #[error("Memory error: {0}")]
    Memory(String),

    /// Parameter management errors
    #[error("Parameter error: {0}")]
    Parameter(String),

    /// Serialization/Deserialization errors
    #[error("Serialization error: {0}")]
    Serialization(String),

    /// Timeout errors
    #[error("Operation timed out: {0}")]
    Timeout(String),

    /// Resource not found errors
    #[error("Resource not found: {0}")]
    NotFound(String),

    /// Permission/Access errors
    #[error("Permission denied: {0}")]
    PermissionDenied(String),

    /// Invalid input/argument errors
    #[error("Invalid input: {0}")]
    InvalidInput(String),

    /// Initialization errors
    #[error("Initialization failed: {0}")]
    InitializationFailed(String),

    /// Already exists errors (for creation operations)
    #[error("Already exists: {0}")]
    AlreadyExists(String),

    /// Parse errors
    #[error("Parse error: {0}")]
    Parse(String),

    /// External command execution errors
    #[error("Command execution failed: {0}")]
    CommandFailed(String),

    /// Feature not available errors
    #[error("Feature not available: {0}")]
    FeatureNotAvailable(String),

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

/// Short alias — `Result<T>` is equivalent to `HorusResult<T>`
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

    /// Create a backend error with backend name and message
    pub fn backend<S: Into<String>, T: Into<String>>(backend: S, message: T) -> Self {
        HorusError::Backend {
            backend: backend.into(),
            message: message.into(),
        }
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

    /// Create a driver error
    pub fn driver<S: Into<String>>(msg: S) -> Self {
        HorusError::Driver(msg.into())
    }

    /// Create a memory error
    pub fn memory<S: Into<String>>(msg: S) -> Self {
        HorusError::Memory(msg.into())
    }

    /// Create an invalid input error
    pub fn invalid_input<S: Into<String>>(msg: S) -> Self {
        HorusError::InvalidInput(msg.into())
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
