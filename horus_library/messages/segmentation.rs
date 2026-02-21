//! Segmentation types for zero-copy IPC
//!
//! Fixed-size types for semantic and instance segmentation masks.
//! The mask pixel data follows the header as a raw byte array.

use bytemuck::{Pod, Zeroable};

/// Segmentation mask header
///
/// The mask data follows this header as a raw byte/uint8 array.
/// Each pixel contains either:
/// - Semantic segmentation: class ID (0-255)
/// - Instance segmentation: instance ID (0-255)
///
/// Size: 64 bytes
#[repr(C)]
#[derive(Debug, Clone, Copy, Default, Pod, Zeroable)]
pub struct SegmentationMask {
    /// Image width
    pub width: u32,
    /// Image height
    pub height: u32,
    /// Number of classes (for semantic segmentation)
    pub num_classes: u32,
    /// Mask type: 0=semantic, 1=instance, 2=panoptic
    pub mask_type: u32,
    /// Timestamp (nanoseconds since epoch)
    pub timestamp_ns: u64,
    /// Sequence number
    pub seq: u64,
    /// Frame ID (camera/coordinate frame)
    pub frame_id: [u8; 32],
}

impl SegmentationMask {
    /// Create a semantic segmentation mask header
    pub fn semantic(width: u32, height: u32, num_classes: u32) -> Self {
        Self {
            width,
            height,
            num_classes,
            mask_type: 0,
            timestamp_ns: 0,
            seq: 0,
            frame_id: [0u8; 32],
        }
    }

    /// Create an instance segmentation mask header
    pub fn instance(width: u32, height: u32) -> Self {
        Self {
            width,
            height,
            num_classes: 0,
            mask_type: 1,
            timestamp_ns: 0,
            seq: 0,
            frame_id: [0u8; 32],
        }
    }

    /// Create a panoptic segmentation mask header
    pub fn panoptic(width: u32, height: u32, num_classes: u32) -> Self {
        Self {
            width,
            height,
            num_classes,
            mask_type: 2,
            timestamp_ns: 0,
            seq: 0,
            frame_id: [0u8; 32],
        }
    }

    /// Set frame ID
    pub fn with_frame_id(mut self, frame_id: &str) -> Self {
        let bytes = frame_id.as_bytes();
        let len = bytes.len().min(31);
        self.frame_id[..len].copy_from_slice(&bytes[..len]);
        self.frame_id[len..].fill(0);
        self
    }

    /// Set timestamp
    pub fn with_timestamp(mut self, timestamp_ns: u64) -> Self {
        self.timestamp_ns = timestamp_ns;
        self
    }

    /// Get frame ID as string
    pub fn frame_id(&self) -> &str {
        let end = self.frame_id.iter().position(|&b| b == 0).unwrap_or(32);
        std::str::from_utf8(&self.frame_id[..end]).unwrap_or("")
    }

    /// Calculate mask data size (1 byte per pixel for u8 masks)
    pub fn data_size(&self) -> usize {
        (self.width as usize) * (self.height as usize)
    }

    /// Calculate mask data size for u16 masks (panoptic with >256 instances)
    pub fn data_size_u16(&self) -> usize {
        (self.width as usize) * (self.height as usize) * 2
    }

    /// Check if this is semantic segmentation
    pub fn is_semantic(&self) -> bool {
        self.mask_type == 0
    }

    /// Check if this is instance segmentation
    pub fn is_instance(&self) -> bool {
        self.mask_type == 1
    }

    /// Check if this is panoptic segmentation
    pub fn is_panoptic(&self) -> bool {
        self.mask_type == 2
    }
}

impl horus_core::core::LogSummary for SegmentationMask {
    fn log_summary(&self) -> String {
        let kind = match self.mask_type {
            0 => "semantic",
            1 => "instance",
            2 => "panoptic",
            _ => "unknown",
        };
        format!(
            "SegmentationMask {{ {}x{}, {}, {} classes }}",
            self.width, self.height, kind, self.num_classes
        )
    }
}

/// Common segmentation class IDs (COCO-style)
pub mod classes {
    pub const BACKGROUND: u8 = 0;
    pub const PERSON: u8 = 1;
    pub const BICYCLE: u8 = 2;
    pub const CAR: u8 = 3;
    pub const MOTORCYCLE: u8 = 4;
    pub const AIRPLANE: u8 = 5;
    pub const BUS: u8 = 6;
    pub const TRAIN: u8 = 7;
    pub const TRUCK: u8 = 8;
    pub const BOAT: u8 = 9;
    pub const TRAFFIC_LIGHT: u8 = 10;
    pub const FIRE_HYDRANT: u8 = 11;
    pub const STOP_SIGN: u8 = 13;
    pub const PARKING_METER: u8 = 14;
    pub const BENCH: u8 = 15;
    pub const BIRD: u8 = 16;
    pub const CAT: u8 = 17;
    pub const DOG: u8 = 18;
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_segmentation_mask_pod_size() {
        assert_eq!(std::mem::size_of::<SegmentationMask>(), 64);
    }

    #[test]
    fn test_data_size() {
        let mask = SegmentationMask::semantic(1920, 1080, 80);
        assert_eq!(mask.data_size(), 1920 * 1080);
    }

    #[test]
    fn test_mask_type() {
        let semantic = SegmentationMask::semantic(100, 100, 20);
        assert!(semantic.is_semantic());
        assert!(!semantic.is_instance());

        let instance = SegmentationMask::instance(100, 100);
        assert!(!instance.is_semantic());
        assert!(instance.is_instance());

        let panoptic = SegmentationMask::panoptic(100, 100, 80);
        assert!(panoptic.is_panoptic());
    }

    #[test]
    fn test_frame_id() {
        let mask = SegmentationMask::semantic(640, 480, 20)
            .with_frame_id("camera_front");
        assert_eq!(mask.frame_id(), "camera_front");
    }
}
