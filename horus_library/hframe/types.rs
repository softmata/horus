//! Core types for HFrame system

use thiserror::Error;

/// Frame identifier type
///
/// Using u32 allows up to 4 billion frames while keeping memory efficient.
/// The upper bits can encode metadata (static vs dynamic, generation, etc.)
pub type FrameId = u32;

/// Sentinel value indicating no parent (root frame)
pub const NO_PARENT: FrameId = FrameId::MAX;

/// Sentinel value indicating invalid/unallocated frame
pub const INVALID_FRAME: FrameId = FrameId::MAX - 1;

/// Maximum supported frames (can be configured lower)
pub const MAX_SUPPORTED_FRAMES: usize = 65536;

/// HFrame error types
#[derive(Debug, Error, Clone)]
pub enum HFrameError {
    #[error("Frame '{0}' not found")]
    FrameNotFound(String),

    #[error("Frame '{0}' already exists")]
    FrameAlreadyExists(String),

    #[error("Parent frame '{0}' not found")]
    ParentNotFound(String),

    #[error("No transform path between '{0}' and '{1}'")]
    NoPath(String, String),

    #[error("Maximum frame limit ({0}) reached")]
    MaxFramesReached(usize),

    #[error("Transform not available at timestamp {0}")]
    TransformNotAvailable(u64),

    #[error("Cycle detected in frame tree")]
    CycleDetected,

    #[error("Cannot unregister static frame '{0}'")]
    CannotUnregisterStatic(String),

    #[error("Invalid frame ID: {0}")]
    InvalidFrameId(FrameId),

    #[error("Configuration error: {0}")]
    ConfigError(String),
}

impl From<HFrameError> for horus_core::error::HorusError {
    fn from(err: HFrameError) -> Self {
        match err {
            HFrameError::FrameNotFound(name) => {
                horus_core::error::HorusError::NotFound(format!("Frame '{}'", name))
            }
            HFrameError::FrameAlreadyExists(name) => {
                horus_core::error::HorusError::AlreadyExists(format!("Frame '{}'", name))
            }
            HFrameError::ConfigError(msg) => horus_core::error::HorusError::Config(msg),
            other => horus_core::error::HorusError::Communication(other.to_string()),
        }
    }
}

/// Result type for HFrame operations
pub type HFrameResult<T> = Result<T, HFrameError>;

/// Frame type indicator
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum FrameType {
    /// Unallocated slot
    Unallocated = 0,
    /// Static frame (transform never changes, no history buffer)
    Static = 1,
    /// Dynamic frame (transform changes over time, has history buffer)
    Dynamic = 2,
}

impl From<u8> for FrameType {
    fn from(value: u8) -> Self {
        match value {
            1 => FrameType::Static,
            2 => FrameType::Dynamic,
            _ => FrameType::Unallocated,
        }
    }
}

impl From<FrameType> for u8 {
    fn from(value: FrameType) -> Self {
        value as u8
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_frame_type_conversion() {
        assert_eq!(FrameType::from(0), FrameType::Unallocated);
        assert_eq!(FrameType::from(1), FrameType::Static);
        assert_eq!(FrameType::from(2), FrameType::Dynamic);
        assert_eq!(FrameType::from(99), FrameType::Unallocated);

        assert_eq!(u8::from(FrameType::Static), 1);
        assert_eq!(u8::from(FrameType::Dynamic), 2);
    }

    #[test]
    fn test_error_display() {
        let err = HFrameError::FrameNotFound("camera".to_string());
        assert!(err.to_string().contains("camera"));
    }
}
