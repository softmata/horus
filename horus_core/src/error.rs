//! Unified error handling for HORUS.
//!
//! Every error in HORUS is structured, matchable, and preserves its source chain.
//!
//! # Architecture
//!
//! [`HorusError`] is the umbrella enum. Each variant wraps a **domain sub-error**
//! that carries structured fields instead of opaque strings:
//!
//! | Variant | Sub-error type | Domain |
//! |---------|---------------|--------|
//! | `Config(…)` | [`ConfigError`] | Configuration parsing/validation |
//! | `Communication(…)` | [`CommunicationError`] | IPC, topics, mDNS |
//! | `Node(…)` | [`NodeError`] | Node lifecycle (init, tick, shutdown) |
//! | `Memory(…)` | [`MemoryError`] | SHM, mmap, tensor pools |
//! | `Serialization(…)` | [`SerializationError`] | JSON, YAML, TOML, binary |
//! | `NotFound(…)` | [`NotFoundError`] | Missing frames, topics, nodes |
//! | `Resource(…)` | [`ResourceError`] | Already exists, permission denied, unsupported |
//! | `InvalidInput(…)` | [`ValidationError`] | Out-of-range, invalid format, constraints |
//! | `Parse(…)` | [`ParseError`] | Integer, float, boolean parsing |
//! | `Transform(…)` | [`TransformError`] | Extrapolation, stale data |
//! | `Timeout(…)` | [`TimeoutError`] | Resource/operation timeouts |
//!
//! # Pattern matching
//!
//! ```rust,ignore
//! use horus::prelude::*;
//!
//! match err {
//!     HorusError::Config(ConfigError::MissingField { field, .. }) => {
//!         eprintln!("Add '{}' to your config file", field);
//!     }
//!     HorusError::Resource(ResourceError::AlreadyExists { name, .. }) => {
//!         // Idempotent — skip
//!     }
//!     HorusError::Transform(TransformError::Extrapolation { frame, .. }) => {
//!         eprintln!("Increase buffer size for frame '{}'", frame);
//!     }
//!     other => {
//!         if let Some(hint) = other.help() {
//!             eprintln!("  hint: {}", hint);
//!         }
//!     }
//! }
//! ```
//!
//! # Error context and chaining
//!
//! Use [`HorusContext`] to wrap foreign errors with descriptive context:
//!
//! ```rust,ignore
//! use horus::prelude::*;
//!
//! fn load_config(path: &str) -> HorusResult<Config> {
//!     let bytes = std::fs::read(path)
//!         .horus_context_with(|| format!("reading config '{}'", path))?;
//!     // ...
//! }
//! ```
//!
//! # Severity
//!
//! Every error has a [`Severity`] classification used by the scheduler for
//! automatic recovery: `Transient` (retry), `Permanent` (skip), `Fatal` (stop).

use std::time::Duration;
use thiserror::Error;

/// Structured communication layer errors.
///
/// Enables pattern matching on specific failure conditions:
/// ```rust,ignore
/// match err {
///     HorusError::Communication(CommunicationError::TopicFull { topic }) => {
///         // Back-pressure: wait and retry
///     }
///     HorusError::Communication(CommunicationError::TopicNotFound { topic }) => {
///         // Create the topic or bail
///     }
///     _ => {}
/// }
/// ```
#[non_exhaustive]
#[derive(Debug, Error)]
pub enum CommunicationError {
    /// Ring buffer is full — publisher is faster than subscriber.
    #[error("Topic '{topic}' is full")]
    TopicFull { topic: String },

    /// Named topic does not exist.
    #[error("Topic '{topic}' not found")]
    TopicNotFound { topic: String },

    /// Topic creation failed (e.g., SHM or ring buffer setup).
    #[error("Failed to create topic '{topic}': {reason}")]
    TopicCreationFailed { topic: String, reason: String },

    /// mDNS service discovery operation failed.
    #[error("mDNS {operation} failed: {reason}")]
    MdnsFailed { operation: String, reason: String },

    /// Network fault (peer unreachable, DNS failure, corrupt response).
    #[error("Network fault for '{peer}': {reason}")]
    NetworkFault { peer: String, reason: String },

    /// Serialization/deserialization failure in the communication layer.
    #[error("Communication serialization failed: {reason}")]
    SerializationFailed { reason: String },

    /// Action system error (goal rejected, execution failed, etc.).
    #[error("Action failed: {reason}")]
    ActionFailed { reason: String },
}

impl From<String> for CommunicationError {
    fn from(reason: String) -> Self {
        CommunicationError::SerializationFailed { reason }
    }
}

/// Structured "not found" errors.
///
/// Enables pattern matching on which resource type is missing:
/// ```rust,ignore
/// match err {
///     HorusError::NotFound(NotFoundError::Frame { name }) => {
///         // Check frame registration or spelling
///     }
///     HorusError::NotFound(NotFoundError::Topic { name }) => {
///         // Create the topic first
///     }
///     _ => {}
/// }
/// ```
#[non_exhaustive]
#[derive(Debug, Error)]
pub enum NotFoundError {
    /// A coordinate frame was not found in the registry.
    #[error("Frame '{name}' not found")]
    Frame { name: String },

    /// A parent frame was not found during registration.
    #[error("Parent frame '{name}' not found")]
    ParentFrame { name: String },

    /// A topic was not found.
    #[error("Topic '{name}' not found")]
    Topic { name: String },

    /// A node was not found.
    #[error("Node '{name}' not found")]
    Node { name: String },

    /// A service was not found.
    #[error("Service '{name}' not found")]
    Service { name: String },

    /// An action was not found.
    #[error("Action '{name}' not found")]
    Action { name: String },

    /// A parameter was not found.
    #[error("Parameter '{name}' not found")]
    Parameter { name: String },

    /// A generic resource was not found (for resources not covered above).
    #[error("{kind} '{name}' not found")]
    Other { kind: String, name: String },
}

/// Structured node lifecycle errors.
///
/// Enables pattern matching on specific node failure modes:
/// ```rust,ignore
/// match err {
///     HorusError::Node(NodeError::InitPanic { node }) => {
///         // Node panicked during initialization
///     }
///     HorusError::Node(NodeError::InitFailed { node, reason }) => {
///         // Node init returned an error
///     }
///     _ => {}
/// }
/// ```
#[non_exhaustive]
#[derive(Debug, Error)]
pub enum NodeError {
    /// Node panicked during `init()`.
    #[error("Node '{node}' panicked during init")]
    InitPanic { node: String },

    /// Node panicked during `tick()` re-initialization.
    #[error("Node '{node}' panicked during re-init")]
    ReInitPanic { node: String },

    /// Node panicked during `shutdown()`.
    #[error("Node '{node}' panicked during shutdown")]
    ShutdownPanic { node: String },

    /// Node `init()` returned an error.
    #[error("Node '{node}' init failed: {reason}")]
    InitFailed { node: String, reason: String },

    /// Node `tick()` returned an error.
    #[error("Node '{node}' tick failed: {reason}")]
    TickFailed { node: String, reason: String },

    /// Catch-all for node errors not covered above.
    #[error("Node '{node}' error: {message}")]
    Other { node: String, message: String },
}

/// Structured memory management errors.
///
/// Enables pattern matching on specific failure conditions:
/// ```rust,ignore
/// match err {
///     HorusError::Memory(MemoryError::PoolExhausted { .. }) => {
///         // Wait for slots to free up, or grow the pool
///     }
///     HorusError::Memory(MemoryError::ShmCreateFailed { path, .. }) => {
///         // Check permissions on /dev/shm
///     }
///     _ => {}
/// }
/// ```
#[non_exhaustive]
#[derive(Debug, Error)]
pub enum MemoryError {
    /// Tensor pool has no free slots.
    #[error("Pool exhausted: {reason}")]
    PoolExhausted { reason: String },

    /// Allocation request failed (overflow, too large, etc.).
    #[error("Allocation failed: {reason}")]
    AllocationFailed { reason: String },

    /// Shared memory region could not be created or opened.
    #[error("SHM failed for '{path}': {reason}")]
    ShmCreateFailed { path: String, reason: String },

    /// Memory-mapping (mmap) operation failed.
    #[error("mmap failed: {reason}")]
    MmapFailed { reason: String },

    /// DLPack tensor import validation failed.
    #[error("DLPack import failed: {reason}")]
    DLPackImportFailed { reason: String },

    /// Tensor pool allocation offset would overflow.
    #[error("Tensor pool allocation offset overflow")]
    OffsetOverflow,
}

impl From<String> for MemoryError {
    fn from(reason: String) -> Self {
        MemoryError::AllocationFailed { reason }
    }
}

/// Structured configuration errors.
///
/// Replaces the old `HorusError::Config(String)` with matchable variants:
/// ```rust,ignore
/// match err {
///     HorusError::Config(ConfigError::MissingField { field, .. }) => {
///         // Prompt user for the missing field
///     }
///     HorusError::Config(ConfigError::ValidationFailed { field, expected, .. }) => {
///         // Show valid range
///     }
///     _ => {}
/// }
/// ```
#[non_exhaustive]
#[derive(Debug, Error)]
pub enum ConfigError {
    /// Config file or string could not be parsed.
    #[error("{format} parse error: {reason}")]
    ParseFailed {
        format: &'static str,
        reason: String,
        #[source]
        source: Option<Box<dyn std::error::Error + Send + Sync>>,
    },

    /// A required configuration field is missing.
    #[error("Missing required field '{field}'{}", context.as_ref().map(|c| format!(" in {}", c)).unwrap_or_default())]
    MissingField {
        field: String,
        context: Option<String>,
    },

    /// A configuration value failed validation.
    #[error("Validation failed for '{field}': expected {expected}, got '{actual}'")]
    ValidationFailed {
        field: String,
        expected: String,
        actual: String,
    },

    /// A configuration key has an invalid value.
    #[error("Invalid config value for '{key}': {reason}")]
    InvalidValue { key: String, reason: String },

    /// Catch-all for config errors during gradual migration.
    #[error("{0}")]
    Other(String),
}

impl ConfigError {
    /// Create a catch-all config error (for gradual migration from string variant).
    pub fn other(msg: impl Into<String>) -> Self {
        ConfigError::Other(msg.into())
    }
}

/// Structured input validation errors.
///
/// Replaces `HorusError::InvalidInput(String)` and `HorusError::OutOfRange(String)`:
/// ```rust,ignore
/// match err {
///     HorusError::InvalidInput(ValidationError::OutOfRange { field, min, max, .. }) => {
///         println!("'{}' must be between {} and {}", field, min, max);
///     }
///     _ => {}
/// }
/// ```
#[non_exhaustive]
#[derive(Debug, Error)]
pub enum ValidationError {
    /// A value is outside the valid range.
    #[error("'{field}' out of range: expected [{min}..{max}], got {actual}")]
    OutOfRange {
        field: String,
        min: String,
        max: String,
        actual: String,
    },

    /// A value has an invalid format.
    #[error("Invalid format for '{field}': expected {expected_format}, got '{actual}'")]
    InvalidFormat {
        field: String,
        expected_format: String,
        actual: String,
    },

    /// A value is not one of the allowed options.
    #[error("Invalid value for '{field}': must be one of [{valid_options}], got '{actual}'")]
    InvalidEnum {
        field: String,
        valid_options: String,
        actual: String,
    },

    /// A required field is missing.
    #[error("Missing required field '{field}'")]
    MissingRequired { field: String },

    /// A generic constraint was violated.
    #[error("Constraint violated for '{field}': {constraint}")]
    ConstraintViolation { field: String, constraint: String },

    /// A value is invalid (wrong type, NaN, negative when positive required, etc.).
    #[error("Invalid value for '{field}': '{value}' — {reason}")]
    InvalidValue {
        field: String,
        value: String,
        reason: String,
    },

    /// Two configuration options conflict with each other.
    #[error("Conflicting configuration: '{field_a}' vs '{field_b}' — {reason}")]
    Conflict {
        field_a: String,
        field_b: String,
        reason: String,
    },

    /// Catch-all for validation errors during gradual migration.
    #[error("{0}")]
    Other(String),
}

impl ValidationError {
    /// Create a catch-all validation error (for gradual migration).
    pub fn other(msg: impl Into<String>) -> Self {
        ValidationError::Other(msg.into())
    }
}

/// Structured timeout errors with resource and timing metadata.
///
/// Replaces `HorusError::Timeout(String)` with inspectable fields:
/// ```rust,ignore
/// match err {
///     HorusError::Timeout(ref t) => {
///         if t.elapsed > Duration::from_secs(5) {
///             // System is severely overloaded
///         }
///     }
///     _ => {}
/// }
/// ```
#[derive(Debug, Error)]
#[error("Timeout waiting for '{resource}' after {elapsed:?}{}", deadline.map(|d| format!(" (deadline: {:?})", d)).unwrap_or_default())]
pub struct TimeoutError {
    /// The resource or operation that timed out.
    pub resource: String,
    /// How long the operation waited before timing out.
    pub elapsed: Duration,
    /// The original deadline, if specified.
    pub deadline: Option<Duration>,
}

/// Structured serialization errors preserving source type info.
///
/// Replaces `HorusError::Serialization(String)`:
/// ```rust,ignore
/// match err {
///     HorusError::Serialization(SerializationError::Json { .. }) => {
///         // Handle JSON-specific issue
///     }
///     _ => {}
/// }
/// ```
#[non_exhaustive]
#[derive(Debug, Error)]
pub enum SerializationError {
    /// JSON serialization/deserialization failed.
    #[error("JSON error: {source}")]
    Json {
        #[source]
        source: serde_json::Error,
    },

    /// YAML serialization/deserialization failed.
    #[error("YAML error: {source}")]
    Yaml {
        #[source]
        source: serde_yaml::Error,
    },

    /// TOML serialization failed.
    #[error("TOML serialization error: {source}")]
    Toml {
        #[source]
        source: toml::ser::Error,
    },

    /// Other serialization format or generic failure.
    #[error("{format} serialization error: {reason}")]
    Other { format: String, reason: String },
}

/// Structured transform / coordinate frame errors.
///
/// Replaces `HorusError::Extrapolation(String)` and `HorusError::Stale(String)`:
/// ```rust,ignore
/// match err {
///     HorusError::Transform(TransformError::Extrapolation { frame, gap_ns, .. }) => {
///         // Increase buffer size or use clamped lookup
///     }
///     HorusError::Transform(TransformError::Stale { frame, age, threshold }) => {
///         // Alert: sensor driver for 'frame' may have stopped
///     }
///     _ => {}
/// }
/// ```
#[non_exhaustive]
#[derive(Debug, Error)]
pub enum TransformError {
    /// Requested timestamp is outside the frame's buffered time range.
    #[error("Extrapolation on frame '{frame}': requested {requested_ns}ns, buffer range [{oldest_ns}ns..{newest_ns}ns]")]
    Extrapolation {
        frame: String,
        requested_ns: u64,
        oldest_ns: u64,
        newest_ns: u64,
    },

    /// Transform data is stale — last update exceeded the acceptable age.
    #[error("Frame '{frame}' is stale: last update {age:?} ago, threshold {threshold:?}")]
    Stale {
        frame: String,
        age: Duration,
        threshold: Duration,
    },
}

/// Structured resource lifecycle errors.
///
/// Replaces `HorusError::AlreadyExists(String)`, `HorusError::PermissionDenied(String)`,
/// and `HorusError::Unsupported(String)`:
/// ```rust,ignore
/// match err {
///     HorusError::Resource(ResourceError::AlreadyExists { resource_type, name }) => {
///         if resource_type == "frame" { /* skip, idempotent */ }
///     }
///     HorusError::Resource(ResourceError::PermissionDenied { resource, .. }) => {
///         // Escalate privilege or bail
///     }
///     _ => {}
/// }
/// ```
#[non_exhaustive]
#[derive(Debug, Error)]
pub enum ResourceError {
    /// A resource with the given name already exists.
    #[error("{resource_type} '{name}' already exists")]
    AlreadyExists { resource_type: String, name: String },

    /// Permission denied for a resource operation.
    #[error("Permission denied on '{resource}': requires {required_permission}")]
    PermissionDenied {
        resource: String,
        required_permission: String,
    },

    /// Feature or operation not supported on this platform.
    #[error("Unsupported: {feature} ({reason})")]
    Unsupported { feature: String, reason: String },
}

/// Structured parse errors preserving std source errors.
///
/// Replaces `HorusError::Parse(String)`:
/// ```rust,ignore
/// match err {
///     HorusError::Parse(ParseError::Int { input, .. }) => {
///         println!("'{}' is not a valid integer", input);
///     }
///     _ => {}
/// }
/// ```
#[non_exhaustive]
#[derive(Debug, Error)]
pub enum ParseError {
    /// Integer parse failure.
    #[error("Failed to parse '{input}' as integer: {source}")]
    Int {
        input: String,
        #[source]
        source: std::num::ParseIntError,
    },

    /// Float parse failure.
    #[error("Failed to parse '{input}' as float: {source}")]
    Float {
        input: String,
        #[source]
        source: std::num::ParseFloatError,
    },

    /// Boolean parse failure.
    #[error("Failed to parse '{input}' as boolean: {source}")]
    Bool {
        input: String,
        #[source]
        source: std::str::ParseBoolError,
    },

    /// Custom type parse failure.
    #[error("Failed to parse '{input}' as {type_name}: {reason}")]
    Custom {
        type_name: String,
        input: String,
        reason: String,
    },
}

/// Main error type for HORUS operations
#[non_exhaustive]
#[derive(Debug, Error)]
pub enum HorusError {
    /// I/O related errors
    #[error("IO error: {0}")]
    Io(#[from] std::io::Error),

    /// Configuration parsing or validation errors (structured — see [`ConfigError`]).
    ///
    /// ```rust,ignore
    /// match err {
    ///     HorusError::Config(ConfigError::MissingField { field, .. }) => { /* ... */ }
    ///     HorusError::Config(ConfigError::ParseFailed { format, .. }) => { /* ... */ }
    ///     _ => {}
    /// }
    /// ```
    #[error("Configuration error: {0}")]
    Config(#[from] ConfigError),

    /// Communication layer errors (structured — see [`CommunicationError`]).
    #[error("Communication error: {0}")]
    Communication(#[from] CommunicationError),

    /// Node lifecycle errors (structured — see [`NodeError`]).
    #[error("Node error: {0}")]
    Node(#[from] NodeError),

    /// Memory management errors (structured — see [`MemoryError`]).
    #[error("Memory error: {0}")]
    Memory(#[from] MemoryError),

    /// Serialization/Deserialization errors (structured — see [`SerializationError`]).
    #[error("Serialization error: {0}")]
    Serialization(#[from] SerializationError),

    /// Resource not found errors (structured — see [`NotFoundError`]).
    #[error("Not found: {0}")]
    NotFound(#[from] NotFoundError),

    /// Resource lifecycle errors — already exists, permission denied, unsupported
    /// (structured — see [`ResourceError`]).
    ///
    /// ```rust,ignore
    /// match err {
    ///     HorusError::Resource(ResourceError::AlreadyExists { resource_type, name }) => {
    ///         if resource_type == "frame" { /* skip, idempotent */ }
    ///     }
    ///     HorusError::Resource(ResourceError::PermissionDenied { resource, .. }) => {
    ///         // Escalate privilege or bail
    ///     }
    ///     HorusError::Resource(ResourceError::Unsupported { feature, .. }) => {
    ///         // Fall back to alternative
    ///     }
    ///     _ => {}
    /// }
    /// ```
    #[error("Resource error: {0}")]
    Resource(#[from] ResourceError),

    /// Invalid input/argument errors (structured — see [`ValidationError`]).
    ///
    /// ```rust,ignore
    /// match err {
    ///     HorusError::InvalidInput(ValidationError::OutOfRange { field, min, max, .. }) => { /* ... */ }
    ///     HorusError::InvalidInput(ValidationError::InvalidEnum { field, .. }) => { /* ... */ }
    ///     _ => {}
    /// }
    /// ```
    #[error("Invalid input: {0}")]
    InvalidInput(#[from] ValidationError),

    /// Parse errors (structured — see [`ParseError`]).
    #[error("Parse error: {0}")]
    Parse(#[from] ParseError),

    /// Cross-process tensor descriptor validation failure.
    ///
    /// Returned when a `Tensor` received from another process fails the
    /// structural integrity check: wrong pool ID, out-of-range slot index,
    /// freed or reused slot, or mismatched offset/size fields.
    ///
    /// Callers MUST treat this as an indication of data corruption or a
    /// malicious sender and abort the receive operation.
    #[error("Invalid tensor descriptor: {0}")]
    InvalidDescriptor(String),

    /// Transform errors — extrapolation and stale data (structured — see [`TransformError`]).
    #[error("Transform error: {0}")]
    Transform(#[from] TransformError),

    /// Operation timed out waiting for a resource (structured — see [`TimeoutError`]).
    #[error("Timeout: {0}")]
    Timeout(#[from] TimeoutError),

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

/// Short alias for HorusError — `use horus::prelude::*` brings this into scope
pub type Error = HorusError;

// ============================================
// From implementations for common error types
// ============================================

impl From<serde_json::Error> for HorusError {
    fn from(err: serde_json::Error) -> Self {
        HorusError::Serialization(SerializationError::Json { source: err })
    }
}

impl From<toml::de::Error> for HorusError {
    fn from(err: toml::de::Error) -> Self {
        HorusError::Config(ConfigError::ParseFailed {
            format: "TOML",
            reason: err.to_string(),
            source: Some(Box::new(err)),
        })
    }
}

impl From<toml::ser::Error> for HorusError {
    fn from(err: toml::ser::Error) -> Self {
        HorusError::Serialization(SerializationError::Toml { source: err })
    }
}

impl From<serde_yaml::Error> for HorusError {
    fn from(err: serde_yaml::Error) -> Self {
        HorusError::Serialization(SerializationError::Yaml { source: err })
    }
}

impl From<std::num::ParseIntError> for HorusError {
    fn from(err: std::num::ParseIntError) -> Self {
        HorusError::Parse(ParseError::Int {
            input: String::new(),
            source: err,
        })
    }
}

impl From<std::num::ParseFloatError> for HorusError {
    fn from(err: std::num::ParseFloatError) -> Self {
        HorusError::Parse(ParseError::Float {
            input: String::new(),
            source: err,
        })
    }
}

impl From<std::str::ParseBoolError> for HorusError {
    fn from(err: std::str::ParseBoolError) -> Self {
        HorusError::Parse(ParseError::Bool {
            input: String::new(),
            source: err,
        })
    }
}

// ============================================
// From implementations for new structured errors
// ============================================
// These enable using the new structured types with ? operator:
//   let err: HorusError = ConfigError::MissingField { ... }.into();

// From<ResourceError> is now auto-generated by #[from] on HorusError::Resource

impl From<uuid::Error> for HorusError {
    fn from(err: uuid::Error) -> Self {
        HorusError::Contextual {
            message: format!("UUID error: {}", err),
            source: Box::new(err),
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
        HorusError::Contextual {
            message: err.to_string(),
            source: err.into(),
        }
    }
}

// NOTE: From<String> and From<&str> intentionally removed.
// Use specific error variants instead:
//   HorusError::config("msg")       — for config errors
//   horus_internal!("msg")          — for internal errors (captures file/line)
//   HorusError::InvalidInput(ValidationError::Other("msg".into())) — for input errors
// This prevents accidental untyped errors via "string".into()

/// Error severity classification for automatic recovery decisions.
///
/// The scheduler uses this to decide how to handle node errors:
/// - **Transient**: Retry — the error may resolve on its own (back-pressure, timeouts).
/// - **Permanent**: Skip — this operation won't succeed but the node can continue.
/// - **Fatal**: Stop — data integrity is compromised or the node cannot recover.
///
/// ```rust,ignore
/// match err.severity() {
///     Severity::Transient => { retries += 1; continue; }
///     Severity::Permanent => { log::warn!("{}", err); }
///     Severity::Fatal     => { break; }
/// }
/// ```
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, serde::Serialize, serde::Deserialize)]
pub enum Severity {
    /// Transient failure — retry may succeed (e.g., TopicFull, Timeout, PoolExhausted).
    Transient,
    /// Permanent failure — this operation won't succeed, but the system can continue.
    Permanent,
    /// Fatal failure — data integrity violation or unrecoverable state. Stop immediately.
    Fatal,
}

/// Extension trait for adding HORUS context to any `Result`.
///
/// Converts foreign errors into `HorusError::Contextual` with a descriptive
/// message, preserving the original error as the `source()` chain.
///
/// # Example
///
/// ```rust,ignore
/// use horus::prelude::*;
///
/// fn load_config(path: &str) -> HorusResult<String> {
///     std::fs::read_to_string(path)
///         .horus_context(format!("reading config '{}'", path))?;
///     Ok(config)
/// }
///
/// // Lazy context (avoids formatting when Ok):
/// std::fs::read_to_string(path)
///     .horus_context_with(|| format!("reading config '{}'", path))?;
/// ```
pub trait HorusContext<T> {
    /// Wrap the error with a static or pre-built context message.
    fn horus_context<M: Into<String>>(self, msg: M) -> HorusResult<T>;

    /// Wrap the error with a lazily-evaluated context message.
    ///
    /// The closure is only called on `Err`, avoiding allocation on the happy path.
    fn horus_context_with<F: FnOnce() -> String>(self, f: F) -> HorusResult<T>;
}

impl<T, E> HorusContext<T> for std::result::Result<T, E>
where
    E: std::error::Error + Send + Sync + 'static,
{
    #[inline]
    fn horus_context<M: Into<String>>(self, msg: M) -> HorusResult<T> {
        self.map_err(|e| HorusError::Contextual {
            message: msg.into(),
            source: Box::new(e),
        })
    }

    #[inline]
    fn horus_context_with<F: FnOnce() -> String>(self, f: F) -> HorusResult<T> {
        self.map_err(|e| HorusError::Contextual {
            message: f(),
            source: Box::new(e),
        })
    }
}

// Helper methods
impl HorusError {
    /// Create a configuration error with a custom message.
    pub fn config<S: Into<String>>(msg: S) -> Self {
        HorusError::Config(ConfigError::Other(msg.into()))
    }

    /// Create a node error with node name and message (convenience catch-all).
    pub fn node<S: Into<String>, T: Into<String>>(node: S, message: T) -> Self {
        HorusError::Node(NodeError::Other {
            node: node.into(),
            message: message.into(),
        })
    }

    /// Create a network fault communication error.
    pub fn network_fault<S: Into<String>, T: Into<String>>(peer: S, reason: T) -> Self {
        HorusError::Communication(CommunicationError::NetworkFault {
            peer: peer.into(),
            reason: reason.into(),
        })
    }

    /// Create an mDNS communication error.
    pub fn mdns_failed<S: Into<String>, T: Into<String>>(operation: S, reason: T) -> Self {
        HorusError::Communication(CommunicationError::MdnsFailed {
            operation: operation.into(),
            reason: reason.into(),
        })
    }

    /// Returns an actionable remediation hint for this error, if available.
    ///
    /// Help text tells the user **what to do**, not just what went wrong.
    /// Returns `None` for errors where no specific guidance applies.
    /// All strings are `&'static str` — zero allocation.
    ///
    /// ```rust,ignore
    /// if let Err(e) = result {
    ///     eprintln!("error: {}", e);
    ///     if let Some(hint) = e.help() {
    ///         eprintln!("  hint: {}", hint);
    ///     }
    /// }
    /// ```
    pub fn help(&self) -> Option<&'static str> {
        match self {
            // === Communication ===
            Self::Communication(CommunicationError::TopicFull { .. }) =>
                Some("Subscriber is slower than publisher. Increase ring buffer capacity or reduce publish rate."),
            Self::Communication(CommunicationError::TopicNotFound { .. }) =>
                Some("Create the topic before subscribing, or check for typos in the topic name. Run: horus topic list"),
            Self::Communication(CommunicationError::TopicCreationFailed { .. }) =>
                Some("Check shared memory permissions and available space. Run: ls -la /dev/shm/horus_*"),
            Self::Communication(CommunicationError::MdnsFailed { .. }) =>
                Some("Ensure mDNS/Avahi is running. Check firewall rules for UDP port 5353. Run: horus discover"),
            Self::Communication(CommunicationError::NetworkFault { .. }) =>
                Some("Check network connectivity to the peer. Verify the peer node is running. Run: horus node list"),
            Self::Communication(CommunicationError::ActionFailed { .. }) =>
                Some("Check that the action server is running and accepting goals. Run: horus action list"),

            // === Memory ===
            Self::Memory(MemoryError::PoolExhausted { .. }) =>
                Some("All tensor pool slots are in use. Ensure consumers drop() tensors promptly, or increase pool capacity in config."),
            Self::Memory(MemoryError::ShmCreateFailed { .. }) =>
                Some("Check permissions on /dev/shm and available disk space. Run: ls -la /dev/shm/ && df -h /dev/shm"),
            Self::Memory(MemoryError::MmapFailed { .. }) =>
                Some("Memory mapping failed. Check available system memory and /dev/shm space. On macOS, check with: vm_stat"),
            Self::Memory(MemoryError::AllocationFailed { .. }) =>
                Some("Memory allocation failed. The system may be low on memory, or the requested size exceeds limits."),
            Self::Memory(MemoryError::OffsetOverflow) =>
                Some("Tensor pool allocation overflowed. This indicates a pool sizing issue — reduce concurrent allocations or increase pool size."),
            Self::Memory(MemoryError::DLPackImportFailed { .. }) =>
                Some("The DLPack tensor has an invalid structure. Verify the tensor shape, dtype, and device match expectations."),

            // === Node ===
            Self::Node(NodeError::InitPanic { .. }) =>
                Some("A node panicked during init(). Check the node's init() implementation for unwrap() calls or index out of bounds."),
            Self::Node(NodeError::ReInitPanic { .. }) =>
                Some("A node panicked during re-initialization after failure. Check the node's init() for state that isn't properly reset."),
            Self::Node(NodeError::ShutdownPanic { .. }) =>
                Some("A node panicked during shutdown(). Check for resources that may already be freed or locks that are poisoned."),
            Self::Node(NodeError::InitFailed { .. }) =>
                Some("Node initialization returned an error. Check node logs for details. Run: horus log -n <node_name>"),
            Self::Node(NodeError::TickFailed { .. }) =>
                Some("Node tick() returned an error. Check node logs for details. Run: horus log -n <node_name>"),

            // === NotFound ===
            Self::NotFound(NotFoundError::Frame { .. } | NotFoundError::ParentFrame { .. }) =>
                Some("Frame not found in the registry. Check spelling and ensure the frame is registered before use. Run: horus frame list"),
            Self::NotFound(NotFoundError::Topic { .. }) =>
                Some("Topic not found. Ensure a publisher has created it. Run: horus topic list"),
            Self::NotFound(NotFoundError::Node { .. }) =>
                Some("Node not found. Ensure it is registered with the scheduler. Run: horus node list"),
            Self::NotFound(NotFoundError::Service { .. }) =>
                Some("Service not found. Ensure the server is running. Run: horus service list"),

            // === Transform ===
            Self::Transform(TransformError::Stale { .. }) =>
                Some("Transform data is stale — a sensor driver may have stopped publishing. Run: horus node list && horus frame monitor-rates"),
            Self::Transform(TransformError::Extrapolation { .. }) =>
                Some("Requested timestamp is outside the frame buffer range. Use tf_at() for clamped lookup, or increase the frame buffer size."),

            // === Safety ===
            Self::InvalidDescriptor(_) =>
                Some("CRITICAL: Received a corrupt IPC tensor descriptor. This indicates data corruption or a malicious sender. Do NOT use the data. Investigate the sending process."),

            // === Config ===
            Self::Config(ConfigError::ParseFailed { format: _, .. }) =>
                Some("Check the config file syntax. Run a linter for the format (e.g., `taplo check` for TOML, `yamllint` for YAML)."),
            Self::Config(ConfigError::MissingField { .. }) =>
                Some("A required field is missing from the config file. Check the documentation for the expected schema."),
            Self::Config(ConfigError::ValidationFailed { .. }) =>
                Some("A config value failed validation. Check the field's allowed values in the documentation."),

            // === Validation ===
            Self::InvalidInput(ValidationError::OutOfRange { .. }) =>
                Some("A value is outside the allowed range. Check the parameter constraints in the documentation."),

            // === Resource ===
            Self::Resource(ResourceError::AlreadyExists { .. }) =>
                Some("A resource with this name already exists. Use a unique name or check if the existing resource can be reused."),
            Self::Resource(ResourceError::PermissionDenied { .. }) =>
                Some("Permission denied. Check file/device permissions and ensure the process has the required access level."),
            Self::Resource(ResourceError::Unsupported { .. }) =>
                Some("This operation is not supported on the current platform or configuration."),

            // === Timeout ===
            Self::Timeout(_) =>
                Some("Operation timed out. The resource may be contended or the system under heavy load. Consider increasing the timeout or checking system load."),

            // No specific hint for generic variants
            _ => None,
        }
    }

    /// Returns the severity classification for this error.
    ///
    /// Used by the scheduler and recovery logic to decide whether to retry,
    /// skip, or stop a node after an error.
    pub fn severity(&self) -> Severity {
        match self {
            // === Transient: retry may succeed ===
            Self::Communication(CommunicationError::TopicFull { .. }) => Severity::Transient,
            Self::Communication(CommunicationError::NetworkFault { .. }) => Severity::Transient,
            Self::Communication(CommunicationError::MdnsFailed { .. }) => Severity::Transient,
            Self::Memory(MemoryError::PoolExhausted { .. }) => Severity::Transient,
            Self::Timeout(_) => Severity::Transient,
            Self::Transform(TransformError::Stale { .. }) => Severity::Transient,

            // === Fatal: data integrity or unrecoverable ===
            Self::InvalidDescriptor(_) => Severity::Fatal,
            Self::Memory(MemoryError::ShmCreateFailed { .. }) => Severity::Fatal,
            Self::Memory(MemoryError::MmapFailed { .. }) => Severity::Fatal,
            Self::Node(NodeError::InitPanic { .. }) => Severity::Fatal,
            Self::Node(NodeError::ReInitPanic { .. }) => Severity::Fatal,
            Self::Node(NodeError::ShutdownPanic { .. }) => Severity::Fatal,

            // === Permanent: won't succeed but system continues ===
            _ => Severity::Permanent,
        }
    }
}

// ============================================
// Retry support for transient errors
// ============================================

/// Configuration for automatic retry of transient errors.
///
/// Used with [`retry_transient`] and `ServiceClient::call_resilient()`.
///
/// # Example
///
/// ```rust,ignore
/// use horus::prelude::*;
///
/// let config = RetryConfig::new(3, Duration::from_millis(10));
/// let result = retry_transient(&config, || tf.tf("camera", "base"));
/// ```
#[derive(Debug, Clone)]
pub struct RetryConfig {
    /// Maximum number of retry attempts (not counting the initial attempt).
    max_retries: u32,
    /// Initial backoff duration before the first retry.
    initial_backoff: Duration,
    /// Maximum backoff duration (caps exponential growth).
    max_backoff: Duration,
    /// Multiplier applied to backoff after each retry (default: 2.0).
    backoff_multiplier: f64,
}

impl RetryConfig {
    /// Create a retry config with the given max retries and initial backoff.
    ///
    /// Uses exponential backoff (2x multiplier) capped at 1 second.
    pub fn new(max_retries: u32, initial_backoff: Duration) -> Self {
        Self {
            max_retries,
            initial_backoff,
            max_backoff: Duration::from_secs(1),
            backoff_multiplier: 2.0,
        }
    }

    /// Set the maximum backoff duration.
    pub fn with_max_backoff(mut self, max: Duration) -> Self {
        self.max_backoff = max;
        self
    }

    /// Set the backoff multiplier. Must be a positive finite number.
    ///
    /// # Panics
    ///
    /// Panics if `multiplier` is not positive or not finite.
    pub fn with_multiplier(mut self, multiplier: f64) -> Self {
        assert!(
            multiplier > 0.0 && multiplier.is_finite(),
            "backoff multiplier must be positive and finite, got {multiplier}"
        );
        self.backoff_multiplier = multiplier;
        self
    }

    /// Maximum number of retry attempts.
    pub fn max_retries(&self) -> u32 {
        self.max_retries
    }

    /// Initial backoff duration before the first retry.
    pub fn initial_backoff(&self) -> Duration {
        self.initial_backoff
    }

    /// Maximum backoff duration (caps exponential growth).
    pub fn max_backoff(&self) -> Duration {
        self.max_backoff
    }

    /// Multiplier applied to backoff after each retry.
    pub fn backoff_multiplier(&self) -> f64 {
        self.backoff_multiplier
    }
}

impl Default for RetryConfig {
    /// Default: 3 retries, 10ms initial backoff, 2x multiplier, 1s cap.
    fn default() -> Self {
        Self::new(3, Duration::from_millis(10))
    }
}

/// Retry a fallible operation, retrying only on transient errors.
///
/// Calls `f` up to `config.max_retries + 1` times total. On each failure,
/// checks [`HorusError::severity()`] — only [`Severity::Transient`] errors
/// trigger a retry. Permanent and fatal errors propagate immediately.
///
/// Uses exponential backoff between retries.
///
/// # Example
///
/// ```rust,ignore
/// use horus::prelude::*;
///
/// // Retry stale transform lookups with default config
/// let tf = retry_transient(&RetryConfig::default(), || {
///     tf.tf("camera", "base")
/// })?;
///
/// // Custom retry: 5 attempts, 50ms initial backoff
/// let data = retry_transient(
///     &RetryConfig::new(5, Duration::from_millis(50)),
///     || expensive_operation(),
/// )?;
/// ```
pub fn retry_transient<T, F>(config: &RetryConfig, mut f: F) -> HorusResult<T>
where
    F: FnMut() -> HorusResult<T>,
{
    let mut backoff = config.initial_backoff();

    for attempt in 0..=config.max_retries() {
        match f() {
            Ok(val) => return Ok(val),
            Err(e) => {
                if attempt == config.max_retries() || e.severity() != Severity::Transient {
                    return Err(e);
                }
                std::thread::sleep(backoff);
                backoff = Duration::from_secs_f64(
                    (backoff.as_secs_f64() * config.backoff_multiplier())
                        .min(config.max_backoff().as_secs_f64()),
                );
            }
        }
    }

    // Unreachable: the loop always returns
    unreachable!()
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
        let err = HorusError::Config(ConfigError::Other(
            "Missing 'control_frequency' in robot.toml".to_string(),
        ));
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
        let err = HorusError::Communication(CommunicationError::TopicNotFound {
            topic: "cmd_vel".to_string(),
        });
        let msg = format!("{}", err);
        assert!(msg.contains("Communication error"), "Display: {}", msg);
        assert!(
            msg.contains("cmd_vel"),
            "Should contain topic name: {}",
            msg
        );
    }

    /// Structured CommunicationError::TopicFull can be pattern-matched.
    #[test]
    fn variant_communication_topic_full() {
        let err = HorusError::Communication(CommunicationError::TopicFull {
            topic: "cmd_vel".into(),
        });
        assert!(
            matches!(&err, HorusError::Communication(CommunicationError::TopicFull { topic }) if topic == "cmd_vel"),
            "Expected TopicFull, got {:?}",
            err
        );
        let msg = format!("{}", err);
        assert!(msg.contains("cmd_vel"), "Display: {}", msg);
    }

    /// Structured CommunicationError::TopicNotFound can be pattern-matched.
    #[test]
    fn variant_communication_topic_not_found() {
        let err = HorusError::Communication(CommunicationError::TopicNotFound {
            topic: "lidar_scan".into(),
        });
        assert!(
            matches!(&err, HorusError::Communication(CommunicationError::TopicNotFound { topic }) if topic == "lidar_scan"),
            "Expected TopicNotFound, got {:?}",
            err
        );
    }

    /// NodeError::Other includes BOTH node name AND message.
    /// Robotics: crucial for identifying which of 20+ nodes failed.
    #[test]
    fn variant_node_includes_name_and_message() {
        let err = HorusError::Node(NodeError::Other {
            node: "imu_driver".to_string(),
            message: "I2C read timeout after 50ms".to_string(),
        });
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

    /// NodeError::InitPanic can be pattern-matched.
    #[test]
    fn variant_node_init_panic() {
        let err = HorusError::Node(NodeError::InitPanic {
            node: "motor_ctrl".to_string(),
        });
        assert!(
            matches!(&err, HorusError::Node(NodeError::InitPanic { node }) if node == "motor_ctrl"),
            "Expected InitPanic, got {:?}",
            err
        );
        let msg = format!("{}", err);
        assert!(msg.contains("motor_ctrl"), "Display: {}", msg);
        assert!(msg.contains("init"), "Display: {}", msg);
    }

    /// NodeError::TickFailed can be pattern-matched.
    #[test]
    fn variant_node_tick_failed() {
        let err = HorusError::Node(NodeError::TickFailed {
            node: "lidar".to_string(),
            reason: "sensor timeout".to_string(),
        });
        match &err {
            HorusError::Node(NodeError::TickFailed { node, reason }) => {
                assert_eq!(node, "lidar");
                assert!(reason.contains("timeout"));
            }
            other => unreachable!("Expected TickFailed, got {:?}", other),
        }
    }

    /// Memory variant for mmap failures.
    #[test]
    fn variant_memory_mmap() {
        let err = HorusError::Memory(MemoryError::MmapFailed {
            reason: "Failed to mmap 4096 bytes for topic 'lidar_scan'".to_string(),
        });
        let msg = format!("{}", err);
        assert!(msg.contains("Memory error"), "Display: {}", msg);
        assert!(msg.contains("lidar_scan"), "Should contain topic: {}", msg);
    }

    /// MemoryError::OffsetOverflow can be pattern-matched.
    #[test]
    fn variant_memory_offset_overflow() {
        let err = HorusError::Memory(MemoryError::OffsetOverflow);
        assert!(matches!(
            &err,
            HorusError::Memory(MemoryError::OffsetOverflow)
        ));
        let msg = format!("{}", err);
        assert!(msg.contains("overflow"), "Display: {}", msg);
    }

    /// Structured MemoryError::PoolExhausted can be pattern-matched.
    #[test]
    fn variant_memory_pool_exhausted() {
        let err = HorusError::Memory(MemoryError::PoolExhausted {
            reason: "No free tensor slots in pool 42".into(),
        });
        assert!(
            matches!(&err, HorusError::Memory(MemoryError::PoolExhausted { reason }) if reason.contains("42")),
            "Expected PoolExhausted, got {:?}",
            err
        );
    }

    /// Structured MemoryError::ShmCreateFailed can be pattern-matched.
    #[test]
    fn variant_memory_shm_create_failed() {
        let err = HorusError::Memory(MemoryError::ShmCreateFailed {
            path: "/dev/shm/horus_pool_0".into(),
            reason: "Permission denied".into(),
        });
        match &err {
            HorusError::Memory(MemoryError::ShmCreateFailed { path, reason }) => {
                assert!(path.contains("horus_pool_0"));
                assert!(reason.contains("Permission denied"));
            }
            other => unreachable!("Expected ShmCreateFailed, got {:?}", other),
        }
    }

    /// Serialization variant for serde failures.
    #[test]
    fn variant_serialization() {
        let err = HorusError::Serialization(SerializationError::Other {
            format: "binary".into(),
            reason: "unexpected end of input at byte 42".into(),
        });
        let msg = format!("{}", err);
        assert!(msg.contains("Serialization error"), "Display: {}", msg);
    }

    /// NotFoundError::Frame can be pattern-matched.
    #[test]
    fn variant_not_found_frame() {
        let err = HorusError::NotFound(NotFoundError::Frame {
            name: "camera_rgb".to_string(),
        });
        assert!(
            matches!(&err, HorusError::NotFound(NotFoundError::Frame { name }) if name == "camera_rgb"),
            "Expected Frame, got {:?}",
            err
        );
        let msg = format!("{}", err);
        assert!(msg.contains("camera_rgb"), "Display: {}", msg);
        assert!(msg.contains("not found"), "Display: {}", msg);
    }

    /// NotFoundError::Topic can be pattern-matched.
    #[test]
    fn variant_not_found_topic() {
        let err = HorusError::NotFound(NotFoundError::Topic {
            name: "cmd_vel".to_string(),
        });
        assert!(
            matches!(&err, HorusError::NotFound(NotFoundError::Topic { name }) if name == "cmd_vel"),
            "Expected Topic, got {:?}",
            err
        );
    }

    /// NotFoundError::ParentFrame can be pattern-matched.
    #[test]
    fn variant_not_found_parent_frame() {
        let err = HorusError::NotFound(NotFoundError::ParentFrame {
            name: "world".to_string(),
        });
        assert!(
            matches!(&err, HorusError::NotFound(NotFoundError::ParentFrame { name }) if name == "world"),
            "Expected ParentFrame, got {:?}",
            err
        );
    }

    /// Resource::PermissionDenied variant.
    #[test]
    fn variant_permission_denied() {
        let err = HorusError::Resource(ResourceError::PermissionDenied {
            resource: "/dev/i2c-1".to_string(),
            required_permission: "rw".to_string(),
        });
        let msg = format!("{}", err);
        assert!(msg.contains("Permission denied"), "Display: {}", msg);
        assert!(msg.contains("/dev/i2c-1"), "Display: {}", msg);
    }

    /// InvalidInput variant.
    #[test]
    fn variant_invalid_input() {
        let err = HorusError::InvalidInput(ValidationError::Other(
            "Frequency must be > 0, got -10".to_string(),
        ));
        let msg = format!("{}", err);
        assert!(msg.contains("Invalid input"), "Display: {}", msg);
        assert!(msg.contains("-10"), "Should contain the bad value: {}", msg);
    }

    /// Resource::AlreadyExists variant.
    #[test]
    fn variant_already_exists() {
        let err = HorusError::Resource(ResourceError::AlreadyExists {
            resource_type: "topic".to_string(),
            name: "cmd_vel".to_string(),
        });
        let msg = format!("{}", err);
        assert!(msg.contains("already exists"), "Display: {}", msg);
        assert!(msg.contains("cmd_vel"), "Display: {}", msg);
    }

    /// Parse variant.
    #[test]
    fn variant_parse() {
        let err = HorusError::Parse(ParseError::Custom {
            type_name: "f32".into(),
            input: "abc".into(),
            reason: "not a valid float".into(),
        });
        let msg = format!("{}", err);
        assert!(msg.contains("Parse error"), "Display: {}", msg);
        assert!(msg.contains("abc"), "Display: {}", msg);
    }

    /// Transform::Extrapolation for TransformFrame time-range violations.
    #[test]
    fn variant_extrapolation() {
        let err = HorusError::Transform(TransformError::Extrapolation {
            frame: "lidar".into(),
            requested_ns: 1000,
            oldest_ns: 5000,
            newest_ns: 10000,
        });
        let msg = format!("{}", err);
        assert!(msg.contains("lidar"), "Should contain frame name: {}", msg);
        assert!(
            msg.contains("1000"),
            "Should contain requested timestamp: {}",
            msg
        );
    }

    /// Transform::Stale for expired transform data.
    #[test]
    fn variant_stale() {
        let err = HorusError::Transform(TransformError::Stale {
            frame: "imu".into(),
            age: Duration::from_millis(2500),
            threshold: Duration::from_millis(500),
        });
        let msg = format!("{}", err);
        assert!(msg.contains("imu"), "Should contain frame name: {}", msg);
        assert!(msg.contains("stale"), "Should say stale: {}", msg);
    }

    /// Resource::Unsupported variant.
    #[test]
    fn variant_unsupported() {
        let err = HorusError::Resource(ResourceError::Unsupported {
            feature: "GPU acceleration".to_string(),
            reason: "not available on this platform".to_string(),
        });
        let msg = format!("{}", err);
        assert!(msg.contains("Unsupported"), "Display: {}", msg);
        assert!(msg.contains("GPU acceleration"), "Display: {}", msg);
    }

    // =========================================================================
    // Section 1b: New structured sub-error types with From conversions
    // =========================================================================

    /// ConfigError converts to HorusError::Config via From, preserving structure.
    #[test]
    fn from_config_error() {
        let err: HorusError = ConfigError::MissingField {
            field: "hz".to_string(),
            context: Some("robot.toml".to_string()),
        }
        .into();
        assert!(
            matches!(err, HorusError::Config(ConfigError::MissingField { ref field, .. }) if field == "hz"),
            "Expected Config(MissingField), got {:?}",
            err
        );
        let msg = format!("{}", err);
        assert!(msg.contains("hz"), "Display: {}", msg);
    }

    /// ValidationError converts to HorusError::InvalidInput via From, preserving structure.
    #[test]
    fn from_validation_error() {
        let err: HorusError = ValidationError::OutOfRange {
            field: "frequency".to_string(),
            min: "0".to_string(),
            max: "1000".to_string(),
            actual: "-10".to_string(),
        }
        .into();
        assert!(
            matches!(err, HorusError::InvalidInput(ValidationError::OutOfRange { ref actual, .. }) if actual == "-10"),
            "Expected InvalidInput(OutOfRange), got {:?}",
            err
        );
        let msg = format!("{}", err);
        assert!(msg.contains("-10"), "Display: {}", msg);
    }

    /// TransformError::Extrapolation converts to HorusError::Transform via From.
    #[test]
    fn from_transform_error_extrapolation() {
        let err: HorusError = TransformError::Extrapolation {
            frame: "lidar".to_string(),
            requested_ns: 1000,
            oldest_ns: 5000,
            newest_ns: 10000,
        }
        .into();
        assert!(
            matches!(err, HorusError::Transform(TransformError::Extrapolation { ref frame, .. }) if frame == "lidar"),
            "Expected Transform(Extrapolation), got {:?}",
            err
        );
    }

    /// TransformError::Stale converts to HorusError::Transform via From.
    #[test]
    fn from_transform_error_stale() {
        let err: HorusError = TransformError::Stale {
            frame: "imu".to_string(),
            age: Duration::from_millis(2500),
            threshold: Duration::from_millis(500),
        }
        .into();
        assert!(
            matches!(err, HorusError::Transform(TransformError::Stale { ref frame, .. }) if frame == "imu"),
            "Expected Transform(Stale), got {:?}",
            err
        );
    }

    /// TimeoutError converts to HorusError::Timeout via From, preserving structure.
    #[test]
    fn from_timeout_error() {
        let err: HorusError = TimeoutError {
            resource: "tensor_pool".to_string(),
            elapsed: Duration::from_millis(500),
            deadline: Some(Duration::from_millis(100)),
        }
        .into();
        assert!(
            matches!(err, HorusError::Timeout(ref t) if t.resource == "tensor_pool"),
            "Expected Timeout, got {:?}",
            err
        );
    }

    /// ResourceError::AlreadyExists converts to HorusError::Resource via From.
    #[test]
    fn from_resource_error_already_exists() {
        let err: HorusError = ResourceError::AlreadyExists {
            resource_type: "frame".to_string(),
            name: "base_link".to_string(),
        }
        .into();
        assert!(
            matches!(err, HorusError::Resource(ResourceError::AlreadyExists { ref name, .. }) if name == "base_link"),
            "Expected Resource(AlreadyExists), got {:?}",
            err
        );
    }

    /// ResourceError::PermissionDenied converts to HorusError::Resource via From.
    #[test]
    fn from_resource_error_permission_denied() {
        let err: HorusError = ResourceError::PermissionDenied {
            resource: "/dev/i2c-1".to_string(),
            required_permission: "rw".to_string(),
        }
        .into();
        assert!(
            matches!(
                err,
                HorusError::Resource(ResourceError::PermissionDenied { .. })
            ),
            "Expected Resource(PermissionDenied), got {:?}",
            err
        );
    }

    /// SerializationError::Json converts to HorusError::Serialization via From.
    #[test]
    fn from_serialization_error_json() {
        let json_err = serde_json::from_str::<serde_json::Value>("{{bad").unwrap_err();
        let err: HorusError = SerializationError::Json { source: json_err }.into();
        assert!(matches!(err, HorusError::Serialization(_)));
    }

    /// ParseError converts to HorusError::Parse via From.
    #[test]
    fn from_parse_error() {
        let parse_err = "abc".parse::<i32>().unwrap_err();
        let err: HorusError = ParseError::Int {
            input: "abc".to_string(),
            source: parse_err,
        }
        .into();
        assert!(matches!(err, HorusError::Parse(_)));
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

    /// From<serde_json::Error> → HorusError::Serialization(SerializationError::Json)
    #[test]
    fn from_serde_json_error() {
        let json_err = serde_json::from_str::<serde_json::Value>("{{bad json").unwrap_err();
        let err: HorusError = json_err.into();
        assert!(
            matches!(
                err,
                HorusError::Serialization(SerializationError::Json { .. })
            ),
            "Expected Serialization(Json), got {:?}",
            err
        );
        let msg = format!("{}", err);
        assert!(msg.contains("Serialization error"), "Display: {}", msg);
        assert!(msg.contains("JSON"), "Should contain JSON: {}", msg);
    }

    /// From<toml::de::Error> → HorusError::Config(ConfigError::ParseFailed)
    #[test]
    fn from_toml_de_error() {
        let toml_err: toml::de::Error = toml::from_str::<toml::Value>("{{bad").unwrap_err();
        let err: HorusError = toml_err.into();
        assert!(
            matches!(
                err,
                HorusError::Config(ConfigError::ParseFailed { format: "TOML", .. })
            ),
            "Expected Config(ParseFailed), got {:?}",
            err
        );
        let msg = format!("{}", err);
        assert!(msg.contains("TOML"), "Display: {}", msg);
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

    /// From<ParseIntError> → HorusError::Parse(ParseError::Int)
    #[test]
    fn from_parse_int_error() {
        let parse_err = "not_a_number".parse::<i32>().unwrap_err();
        let err: HorusError = parse_err.into();
        assert!(matches!(err, HorusError::Parse(ParseError::Int { .. })));
        let msg = format!("{}", err);
        assert!(msg.contains("integer"), "Display: {}", msg);
    }

    /// From<ParseFloatError> → HorusError::Parse(ParseError::Float)
    #[test]
    fn from_parse_float_error() {
        let parse_err = "not_a_float".parse::<f64>().unwrap_err();
        let err: HorusError = parse_err.into();
        assert!(matches!(err, HorusError::Parse(ParseError::Float { .. })));
        let msg = format!("{}", err);
        assert!(msg.contains("float"), "Display: {}", msg);
    }

    /// From<ParseBoolError> → HorusError::Parse(ParseError::Bool)
    #[test]
    fn from_parse_bool_error() {
        let parse_err = "maybe".parse::<bool>().unwrap_err();
        let err: HorusError = parse_err.into();
        assert!(matches!(err, HorusError::Parse(ParseError::Bool { .. })));
        let msg = format!("{}", err);
        assert!(msg.contains("boolean"), "Display: {}", msg);
    }

    /// From<uuid::Error> → HorusError::Contextual (preserves source chain)
    #[test]
    fn from_uuid_error() {
        let uuid_err = uuid::Uuid::parse_str("not-a-uuid").unwrap_err();
        let err: HorusError = uuid_err.into();
        assert!(matches!(err, HorusError::Contextual { .. }));
        let msg = format!("{}", err);
        assert!(msg.contains("UUID error"), "Display: {}", msg);
        // Source chain is preserved
        let src = err.source();
        assert!(src.is_some(), "uuid error should preserve source chain");
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
        let boxed: Box<dyn std::error::Error> = Box::new(std::io::Error::other("boxed io error"));
        let err: HorusError = boxed.into();
        assert!(matches!(err, HorusError::Internal { .. }));
        let msg = format!("{}", err);
        assert!(msg.contains("boxed io error"), "Display: {}", msg);
    }

    /// From<Box<dyn Error + Send + Sync>> → HorusError::Contextual
    #[test]
    fn from_boxed_send_sync_error() {
        let boxed: Box<dyn std::error::Error + Send + Sync> =
            Box::new(std::io::Error::other("send+sync io error"));
        let err: HorusError = boxed.into();
        assert!(matches!(err, HorusError::Contextual { .. }));
        let msg = format!("{}", err);
        assert!(msg.contains("send+sync io error"), "Display: {}", msg);
    }

    /// From<anyhow::Error> → HorusError::Contextual (preserves source chain)
    #[test]
    fn from_anyhow_error() {
        let anyhow_err = anyhow::anyhow!("anyhow test error");
        let err: HorusError = anyhow_err.into();
        assert!(matches!(err, HorusError::Contextual { .. }));
        let msg = format!("{}", err);
        assert!(msg.contains("anyhow test error"), "Display: {}", msg);
        // Source chain is preserved
        let src = err.source();
        assert!(src.is_some(), "anyhow error should preserve source chain");
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
            other => unreachable!("Expected Internal variant, got {:?}", other),
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

    /// HorusError::config() helper wraps in ConfigError::Other.
    #[test]
    fn helper_config() {
        let err = HorusError::config("bad config");
        assert!(
            matches!(err, HorusError::Config(ConfigError::Other(ref s)) if s == "bad config"),
            "Expected Config(Other), got {:?}",
            err
        );
    }

    /// HorusError::node() helper includes both name and message.
    #[test]
    fn helper_node() {
        let err = HorusError::node("lidar", "timeout");
        match &err {
            HorusError::Node(NodeError::Other { node, message }) => {
                assert_eq!(node, "lidar");
                assert_eq!(message, "timeout");
            }
            other => unreachable!("Expected Node(Other) variant, got {:?}", other),
        }
    }

    /// HorusError::network_fault() helper.
    #[test]
    fn helper_network_fault() {
        let err = HorusError::network_fault("192.168.1.10", "connection refused");
        match &err {
            HorusError::Communication(CommunicationError::NetworkFault { peer, reason }) => {
                assert_eq!(peer, "192.168.1.10");
                assert_eq!(reason, "connection refused");
            }
            other => unreachable!("Expected NetworkFault, got {:?}", other),
        }
    }

    /// HorusError::mdns_failed() helper.
    #[test]
    fn helper_mdns_failed() {
        let err = HorusError::mdns_failed("browse", "daemon not running");
        assert!(matches!(
            err,
            HorusError::Communication(CommunicationError::MdnsFailed { .. })
        ));
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

    // =========================================================================
    // Section 6: help() remediation hints
    // =========================================================================

    /// Communication errors return help text.
    #[test]
    fn help_topic_full() {
        let err = HorusError::Communication(CommunicationError::TopicFull {
            topic: "cmd_vel".into(),
        });
        let help = err.help();
        assert!(help.is_some(), "TopicFull should have help text");
        assert!(help.unwrap().contains("buffer"), "Help: {}", help.unwrap());
    }

    /// Memory errors return help text.
    #[test]
    fn help_pool_exhausted() {
        let err = HorusError::Memory(MemoryError::PoolExhausted {
            reason: "no slots".into(),
        });
        let help = err.help();
        assert!(help.is_some(), "PoolExhausted should have help text");
        assert!(help.unwrap().contains("drop()"), "Help: {}", help.unwrap());
    }

    /// Node errors return help text.
    #[test]
    fn help_init_panic() {
        let err = HorusError::Node(NodeError::InitPanic {
            node: "motor".into(),
        });
        let help = err.help();
        assert!(help.is_some(), "InitPanic should have help text");
        assert!(help.unwrap().contains("init()"), "Help: {}", help.unwrap());
    }

    /// Stale transform returns help text.
    #[test]
    fn help_stale() {
        let err = HorusError::Transform(TransformError::Stale {
            frame: "imu".into(),
            age: Duration::from_millis(2500),
            threshold: Duration::from_millis(500),
        });
        let help = err.help();
        assert!(help.is_some(), "Stale should have help text");
        assert!(
            help.unwrap().contains("sensor driver"),
            "Help: {}",
            help.unwrap()
        );
    }

    /// InvalidDescriptor returns CRITICAL help text.
    #[test]
    fn help_invalid_descriptor() {
        let err = HorusError::InvalidDescriptor("bad magic".into());
        let help = err.help();
        assert!(help.is_some(), "InvalidDescriptor should have help text");
        assert!(
            help.unwrap().contains("CRITICAL"),
            "Help: {}",
            help.unwrap()
        );
    }

    /// Generic errors return None for help.
    #[test]
    fn help_none_for_generic() {
        let err = HorusError::Config(ConfigError::Other("bad config".into()));
        assert!(err.help().is_none(), "Config should not have help text");

        let err = HorusError::Io(std::io::Error::other("io"));
        assert!(err.help().is_none(), "Io should not have help text");
    }

    // =========================================================================
    // Section 7: severity() classification
    // =========================================================================

    /// TopicFull is transient — retry may succeed.
    #[test]
    fn severity_transient() {
        let err = HorusError::Communication(CommunicationError::TopicFull {
            topic: "cmd_vel".into(),
        });
        assert_eq!(err.severity(), Severity::Transient);

        let err = HorusError::Timeout(TimeoutError {
            resource: "pool alloc".into(),
            elapsed: Duration::from_millis(100),
            deadline: None,
        });
        assert_eq!(err.severity(), Severity::Transient);

        let err = HorusError::Transform(TransformError::Stale {
            frame: "imu".into(),
            age: Duration::from_secs(1),
            threshold: Duration::from_millis(500),
        });
        assert_eq!(err.severity(), Severity::Transient);

        let err = HorusError::Memory(MemoryError::PoolExhausted {
            reason: "no slots".into(),
        });
        assert_eq!(err.severity(), Severity::Transient);
    }

    /// InvalidDescriptor is fatal — data corruption.
    #[test]
    fn severity_fatal() {
        let err = HorusError::InvalidDescriptor("bad magic".into());
        assert_eq!(err.severity(), Severity::Fatal);

        let err = HorusError::Memory(MemoryError::ShmCreateFailed {
            path: "/dev/shm/x".into(),
            reason: "EACCES".into(),
        });
        assert_eq!(err.severity(), Severity::Fatal);

        let err = HorusError::Node(NodeError::InitPanic {
            node: "motor".into(),
        });
        assert_eq!(err.severity(), Severity::Fatal);
    }

    /// Config/Parse/NotFound are permanent — won't succeed on retry.
    #[test]
    fn severity_permanent() {
        let err = HorusError::Config(ConfigError::Other("bad".into()));
        assert_eq!(err.severity(), Severity::Permanent);

        let err = HorusError::NotFound(NotFoundError::Frame {
            name: "world".into(),
        });
        assert_eq!(err.severity(), Severity::Permanent);

        let err = HorusError::InvalidInput(ValidationError::Other("negative frequency".into()));
        assert_eq!(err.severity(), Severity::Permanent);
    }

    /// Severity is Copy and Eq.
    #[test]
    fn severity_traits() {
        let s = Severity::Transient;
        let s2 = s; // Copy
        assert_eq!(s, s2); // Eq
    }

    // =========================================================================
    // Section 8: HorusContext extension trait
    // =========================================================================

    /// horus_context wraps a foreign error into Contextual with message.
    #[test]
    fn horus_context_wraps_io_error() {
        let result: std::result::Result<(), std::io::Error> =
            Err(std::io::Error::new(std::io::ErrorKind::NotFound, "gone"));

        let err = result.horus_context("reading sensor config").unwrap_err();
        assert!(matches!(err, HorusError::Contextual { .. }));
        let msg = err.to_string();
        assert!(msg.contains("reading sensor config"), "got: {}", msg);
        assert!(msg.contains("gone"), "should preserve source: {}", msg);
    }

    /// horus_context_with only evaluates closure on Err.
    #[test]
    fn horus_context_with_lazy_evaluation() {
        let ok: std::result::Result<i32, std::io::Error> = Ok(42);
        let mut called = false;
        let result = ok.horus_context_with(|| {
            called = true;
            "should not be called".to_string()
        });
        result.unwrap();
        assert!(!called, "closure should not be called on Ok");
    }

    /// horus_context preserves the error source chain.
    #[test]
    fn horus_context_preserves_source() {
        let result: std::result::Result<(), std::io::Error> = Err(std::io::Error::new(
            std::io::ErrorKind::PermissionDenied,
            "no access",
        ));

        let err = result.horus_context("opening /dev/ttyUSB0").unwrap_err();
        let src = err.source();
        assert!(src.is_some(), "Contextual must expose source()");
        assert!(src.unwrap().to_string().contains("no access"));
    }

    // =========================================================================
    // Section 9: Error chain preservation through structured types
    // =========================================================================

    /// SerializationError::Json preserves serde_json source chain.
    #[test]
    fn chain_serialization_json_preserves_source() {
        let json_err = serde_json::from_str::<serde_json::Value>("{{bad").unwrap_err();
        let original_msg = json_err.to_string();

        let err: HorusError = json_err.into();
        // Level 1: HorusError::Serialization → source is SerializationError::Json
        let src1 = err
            .source()
            .expect("HorusError should expose SerializationError as source");
        // Level 2: SerializationError::Json → source is serde_json::Error
        let src2 = src1
            .source()
            .expect("SerializationError::Json should expose serde_json::Error as source");
        assert_eq!(
            src2.to_string(),
            original_msg,
            "original error message must be preserved"
        );
    }

    /// ConfigError::ParseFailed preserves original parse error source chain.
    #[test]
    fn chain_config_parse_preserves_source() {
        let toml_err = toml::from_str::<toml::Value>("{{bad").unwrap_err();
        let original_msg = toml_err.to_string();

        let err: HorusError = toml_err.into();
        // Level 1: HorusError::Config → source is ConfigError
        let src1 = err
            .source()
            .expect("HorusError should expose ConfigError as source");
        // Level 2: ConfigError::ParseFailed → source is Option<Box<dyn Error>>
        let src2 = src1
            .source()
            .expect("ConfigError::ParseFailed should expose original error as source");
        assert_eq!(
            src2.to_string(),
            original_msg,
            "original TOML error must be preserved"
        );
    }

    /// ParseError::Int preserves std::num::ParseIntError source chain.
    #[test]
    fn chain_parse_int_preserves_source() {
        let parse_err = "xyz".parse::<i32>().unwrap_err();
        let original_msg = parse_err.to_string();

        let err: HorusError = parse_err.into();
        // Level 1: HorusError::Parse → source is ParseError
        let src1 = err
            .source()
            .expect("HorusError should expose ParseError as source");
        // Level 2: ParseError::Int → source is ParseIntError
        let src2 = src1
            .source()
            .expect("ParseError::Int should expose ParseIntError as source");
        assert_eq!(
            src2.to_string(),
            original_msg,
            "original ParseIntError must be preserved"
        );
    }

    /// Full 3-level chain: HorusError → SerializationError → serde_yaml::Error
    #[test]
    fn chain_three_level_yaml() {
        let yaml_err = serde_yaml::from_str::<serde_yaml::Value>(":\n  :\n    :bad").unwrap_err();
        let err: HorusError = yaml_err.into();

        // Walk the full chain
        let mut depth = 0;
        let mut current: &dyn StdError = &err;
        while let Some(next) = current.source() {
            depth += 1;
            current = next;
        }
        assert!(
            depth >= 2,
            "YAML error chain should be at least 2 levels deep, got {}",
            depth
        );
    }

    /// horus_context chains stack: Contextual wrapping another Contextual.
    #[test]
    fn chain_nested_horus_context() {
        let io_err = std::io::Error::new(std::io::ErrorKind::NotFound, "sensor.yaml not found");
        let result: std::result::Result<(), std::io::Error> = Err(io_err);

        // First context layer
        let err1 = result.horus_context("reading sensor config").unwrap_err();
        // Second context layer by wrapping again
        let result2: std::result::Result<(), HorusError> = Err(err1);
        let err2 = result2
            .horus_context("initializing sensor node")
            .unwrap_err();

        // Walk the chain: Contextual → Contextual → io::Error
        let src1 = err2.source().expect("outer Contextual should have source");
        assert!(
            src1.to_string().contains("reading sensor config"),
            "first context: {}",
            src1
        );
        let src2 = src1.source().expect("inner Contextual should have source");
        assert!(
            src2.to_string().contains("sensor.yaml not found"),
            "original: {}",
            src2
        );
    }

    /// From<anyhow::Error> preserves chain via Contextual.
    #[test]
    fn chain_anyhow_preserves_source() {
        let inner = std::io::Error::other("disk full");
        let anyhow_err = anyhow::Error::new(inner);
        let err: HorusError = anyhow_err.into();

        assert!(matches!(err, HorusError::Contextual { .. }));
        let src = err
            .source()
            .expect("anyhow conversion should preserve source");
        // anyhow wraps the error, so source should eventually reach "disk full"
        assert!(src.to_string().contains("disk full"), "source: {}", src);
    }

    /// From<uuid::Error> preserves chain via Contextual.
    #[test]
    fn chain_uuid_preserves_source() {
        let uuid_err = uuid::Uuid::parse_str("not-valid").unwrap_err();
        let err: HorusError = uuid_err.into();

        assert!(matches!(err, HorusError::Contextual { .. }));
        let src = err
            .source()
            .expect("uuid conversion should preserve source");
        // The source should be the original uuid::Error
        assert!(!src.to_string().is_empty(), "source should have a message");
    }

    /// Io variant exposes std::io::Error as source.
    #[test]
    fn chain_io_exposes_source() {
        let io_err = std::io::Error::new(std::io::ErrorKind::BrokenPipe, "pipe broken");
        let err = HorusError::Io(io_err);
        let src = err.source().expect("Io variant should expose source");
        assert!(src.to_string().contains("pipe broken"));
    }

    /// TransformError variants expose themselves as source through HorusError.
    #[test]
    fn chain_transform_exposes_source() {
        let err = HorusError::Transform(TransformError::Extrapolation {
            frame: "lidar".into(),
            requested_ns: 100,
            oldest_ns: 200,
            newest_ns: 300,
        });
        let src = err
            .source()
            .expect("Transform variant should expose TransformError as source");
        assert!(src.to_string().contains("lidar"), "source: {}", src);
    }

    /// TimeoutError exposes itself as source through HorusError.
    #[test]
    fn chain_timeout_exposes_source() {
        let err = HorusError::Timeout(TimeoutError {
            resource: "sensor_pool".into(),
            elapsed: Duration::from_secs(5),
            deadline: Some(Duration::from_secs(3)),
        });
        let src = err
            .source()
            .expect("Timeout variant should expose TimeoutError as source");
        assert!(src.to_string().contains("sensor_pool"), "source: {}", src);
    }

    /// Walking the full error chain with std::error::Error trait.
    #[test]
    fn chain_walk_full_chain() {
        // Build: HorusError::Contextual → io::Error
        let io_err =
            std::io::Error::new(std::io::ErrorKind::PermissionDenied, "/dev/shm/horus_pool");
        let err = HorusError::Contextual {
            message: "creating tensor pool".to_string(),
            source: Box::new(io_err),
        };

        // Collect full chain
        let mut chain = Vec::new();
        let mut current: &dyn StdError = &err;
        chain.push(current.to_string());
        while let Some(next) = current.source() {
            chain.push(next.to_string());
            current = next;
        }

        assert_eq!(chain.len(), 2, "chain should be 2 levels: {:?}", chain);
        assert!(
            chain[0].contains("creating tensor pool"),
            "level 0: {}",
            chain[0]
        );
        assert!(
            chain[1].contains("/dev/shm/horus_pool"),
            "level 1: {}",
            chain[1]
        );
    }

    // =========================================================================
    // Retry Tests
    // =========================================================================

    #[test]
    fn retry_succeeds_immediately() {
        let config = RetryConfig::new(3, Duration::from_millis(1));
        let result = retry_transient(&config, || Ok::<_, HorusError>(42));
        assert_eq!(result.unwrap(), 42);
    }

    #[test]
    fn retry_succeeds_after_transient_failures() {
        let config = RetryConfig::new(3, Duration::from_millis(1));
        let mut attempt = 0;
        let result = retry_transient(&config, || {
            attempt += 1;
            if attempt < 3 {
                Err(HorusError::Communication(CommunicationError::TopicFull {
                    topic: "test".to_string(),
                }))
            } else {
                Ok(42)
            }
        });
        assert_eq!(result.unwrap(), 42);
        assert_eq!(attempt, 3);
    }

    #[test]
    fn retry_propagates_permanent_immediately() {
        let config = RetryConfig::new(5, Duration::from_millis(1));
        let mut attempt = 0;
        let result: HorusResult<i32> = retry_transient(&config, || {
            attempt += 1;
            Err(HorusError::NotFound(NotFoundError::Frame {
                name: "missing".to_string(),
            }))
        });
        result.unwrap_err();
        // Permanent error: should not retry
        assert_eq!(attempt, 1);
    }

    #[test]
    fn retry_propagates_fatal_immediately() {
        let config = RetryConfig::new(5, Duration::from_millis(1));
        let mut attempt = 0;
        let result: HorusResult<i32> = retry_transient(&config, || {
            attempt += 1;
            Err(HorusError::InvalidDescriptor("corrupt".to_string()))
        });
        result.unwrap_err();
        assert_eq!(attempt, 1);
    }

    #[test]
    fn retry_exhausts_max_retries() {
        let config = RetryConfig::new(2, Duration::from_millis(1));
        let mut attempt = 0;
        let result: HorusResult<i32> = retry_transient(&config, || {
            attempt += 1;
            Err(HorusError::Timeout(TimeoutError {
                resource: "test".to_string(),
                elapsed: Duration::from_secs(1),
                deadline: Some(Duration::from_secs(1)),
            }))
        });
        result.unwrap_err();
        // Initial attempt + 2 retries = 3 total
        assert_eq!(attempt, 3);
    }

    #[test]
    fn retry_config_defaults() {
        let config = RetryConfig::default();
        assert_eq!(config.max_retries(), 3);
        assert_eq!(config.initial_backoff(), Duration::from_millis(10));
        assert_eq!(config.max_backoff(), Duration::from_secs(1));
        assert_eq!(config.backoff_multiplier(), 2.0);
    }
}
