//! Core types for HFrame system

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
    fn test_constants() {
        assert_eq!(NO_PARENT, FrameId::MAX);
        assert_eq!(INVALID_FRAME, FrameId::MAX - 1);
        assert_ne!(MAX_SUPPORTED_FRAMES, 0);
    }
}
