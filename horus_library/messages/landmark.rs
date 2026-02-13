//! Landmark/keypoint types for zero-copy IPC
//!
//! Pod/Zeroable types for pose estimation, facial landmarks, hand tracking.
//! These are fixed-size types suitable for shared memory transport.

use bytemuck::{Pod, Zeroable};

/// 2D landmark/keypoint
///
/// Used for human pose estimation, facial landmarks, hand tracking.
///
/// Size: 16 bytes
#[repr(C)]
#[derive(Debug, Clone, Copy, Default, PartialEq, Pod, Zeroable)]
pub struct Landmark {
    /// X coordinate (pixels or normalized 0-1)
    pub x: f32,
    /// Y coordinate (pixels or normalized 0-1)
    pub y: f32,
    /// Visibility/confidence (0.0 - 1.0)
    pub visibility: f32,
    /// Landmark index (joint ID, e.g., 0=nose, 1=left_eye, etc.)
    pub index: u32,
}

impl Landmark {
    /// Create a new landmark
    pub fn new(x: f32, y: f32, visibility: f32, index: u32) -> Self {
        Self {
            x,
            y,
            visibility,
            index,
        }
    }

    /// Create without visibility (assumed visible)
    pub fn visible(x: f32, y: f32, index: u32) -> Self {
        Self {
            x,
            y,
            visibility: 1.0,
            index,
        }
    }

    /// Check if landmark is visible
    pub fn is_visible(&self, threshold: f32) -> bool {
        self.visibility >= threshold
    }

    /// Distance to another landmark
    pub fn distance_to(&self, other: &Landmark) -> f32 {
        let dx = self.x - other.x;
        let dy = self.y - other.y;
        (dx * dx + dy * dy).sqrt()
    }
}

/// 3D landmark/keypoint
///
/// Used for 3D pose estimation, MediaPipe-style landmarks.
///
/// Size: 20 bytes (packed)
#[repr(C, packed)]
#[derive(Debug, Clone, Copy, Default, PartialEq, Pod, Zeroable)]
pub struct Landmark3D {
    /// X coordinate (meters or normalized)
    pub x: f32,
    /// Y coordinate (meters or normalized)
    pub y: f32,
    /// Z coordinate (meters or normalized, depth)
    pub z: f32,
    /// Visibility/confidence (0.0 - 1.0)
    pub visibility: f32,
    /// Landmark index
    pub index: u32,
}

impl Landmark3D {
    /// Create a new 3D landmark
    pub fn new(x: f32, y: f32, z: f32, visibility: f32, index: u32) -> Self {
        Self {
            x,
            y,
            z,
            visibility,
            index,
        }
    }

    /// Create without visibility (assumed visible)
    pub fn visible(x: f32, y: f32, z: f32, index: u32) -> Self {
        Self {
            x,
            y,
            z,
            visibility: 1.0,
            index,
        }
    }

    /// Check if landmark is visible
    pub fn is_visible(&self, threshold: f32) -> bool {
        self.visibility >= threshold
    }

    /// Project to 2D (drop Z)
    pub fn to_2d(&self) -> Landmark {
        Landmark {
            x: self.x,
            y: self.y,
            visibility: self.visibility,
            index: self.index,
        }
    }

    /// Distance to another landmark
    pub fn distance_to(&self, other: &Landmark3D) -> f32 {
        let dx = self.x - other.x;
        let dy = self.y - other.y;
        let dz = self.z - other.z;
        (dx * dx + dy * dy + dz * dz).sqrt()
    }
}

/// Landmark array header for fixed-size landmark sets
///
/// Common configurations:
/// - COCO pose: 17 landmarks
/// - MediaPipe pose: 33 landmarks
/// - MediaPipe hands: 21 landmarks per hand
/// - MediaPipe face mesh: 468 landmarks
///
/// Size: 40 bytes
#[repr(C)]
#[derive(Debug, Clone, Copy, Default, Pod, Zeroable)]
pub struct LandmarkArray {
    /// Number of landmarks in this array
    pub num_landmarks: u32,
    /// Landmark dimension: 2 for 2D, 3 for 3D
    pub dimension: u32,
    /// Detection/person ID (when multiple people detected)
    pub instance_id: u32,
    /// Confidence score for the entire pose/face (0.0 - 1.0)
    pub confidence: f32,
    /// Timestamp (nanoseconds since epoch)
    pub timestamp_ns: u64,
    /// Bounding box of landmarks (x, y, width, height in pixels)
    pub bbox_x: f32,
    pub bbox_y: f32,
    pub bbox_width: f32,
    pub bbox_height: f32,
}

impl LandmarkArray {
    /// Create header for 2D landmarks
    pub fn new_2d(num_landmarks: u32) -> Self {
        Self {
            num_landmarks,
            dimension: 2,
            ..Default::default()
        }
    }

    /// Create header for 3D landmarks
    pub fn new_3d(num_landmarks: u32) -> Self {
        Self {
            num_landmarks,
            dimension: 3,
            ..Default::default()
        }
    }

    /// Create header for COCO pose (17 landmarks)
    pub fn coco_pose() -> Self {
        Self::new_2d(17)
    }

    /// Create header for MediaPipe pose (33 landmarks)
    pub fn mediapipe_pose() -> Self {
        Self::new_3d(33)
    }

    /// Create header for MediaPipe hands (21 landmarks)
    pub fn mediapipe_hand() -> Self {
        Self::new_3d(21)
    }

    /// Create header for MediaPipe face mesh (468 landmarks)
    pub fn mediapipe_face() -> Self {
        Self::new_3d(468)
    }

    /// Set confidence
    pub fn with_confidence(mut self, confidence: f32) -> Self {
        self.confidence = confidence;
        self
    }

    /// Set bounding box
    pub fn with_bbox(mut self, x: f32, y: f32, width: f32, height: f32) -> Self {
        self.bbox_x = x;
        self.bbox_y = y;
        self.bbox_width = width;
        self.bbox_height = height;
        self
    }

    /// Calculate data size for landmark array
    pub fn data_size(&self) -> usize {
        let landmark_size = if self.dimension == 2 {
            std::mem::size_of::<Landmark>()
        } else {
            std::mem::size_of::<Landmark3D>()
        };
        (self.num_landmarks as usize) * landmark_size
    }
}

/// Standard landmark indices for COCO pose format
pub mod coco {
    pub const NOSE: u32 = 0;
    pub const LEFT_EYE: u32 = 1;
    pub const RIGHT_EYE: u32 = 2;
    pub const LEFT_EAR: u32 = 3;
    pub const RIGHT_EAR: u32 = 4;
    pub const LEFT_SHOULDER: u32 = 5;
    pub const RIGHT_SHOULDER: u32 = 6;
    pub const LEFT_ELBOW: u32 = 7;
    pub const RIGHT_ELBOW: u32 = 8;
    pub const LEFT_WRIST: u32 = 9;
    pub const RIGHT_WRIST: u32 = 10;
    pub const LEFT_HIP: u32 = 11;
    pub const RIGHT_HIP: u32 = 12;
    pub const LEFT_KNEE: u32 = 13;
    pub const RIGHT_KNEE: u32 = 14;
    pub const LEFT_ANKLE: u32 = 15;
    pub const RIGHT_ANKLE: u32 = 16;
    pub const NUM_LANDMARKS: u32 = 17;
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_landmark_size() {
        assert_eq!(std::mem::size_of::<Landmark>(), 16);
    }

    #[test]
    fn test_landmark3d_size() {
        assert_eq!(std::mem::size_of::<Landmark3D>(), 20);
    }

    #[test]
    fn test_landmark_array_size() {
        assert_eq!(std::mem::size_of::<LandmarkArray>(), 40);
    }

    #[test]
    fn test_landmark_distance() {
        let a = Landmark::visible(0.0, 0.0, 0);
        let b = Landmark::visible(3.0, 4.0, 1);
        assert!((a.distance_to(&b) - 5.0).abs() < 0.001);
    }

    #[test]
    fn test_landmark3d_to_2d() {
        let lm3d = Landmark3D::new(1.0, 2.0, 3.0, 0.9, 5);
        let lm2d = lm3d.to_2d();
        assert_eq!(lm2d.x, 1.0);
        assert_eq!(lm2d.y, 2.0);
        assert_eq!(lm2d.visibility, 0.9);
        assert_eq!(lm2d.index, 5);
    }

    #[test]
    fn test_landmark_visibility() {
        let lm = Landmark::new(0.0, 0.0, 0.5, 0);
        assert!(lm.is_visible(0.3));
        assert!(!lm.is_visible(0.8));
    }

    #[test]
    fn test_landmark_array_data_size() {
        let arr = LandmarkArray::coco_pose();
        assert_eq!(arr.data_size(), 17 * 16); // 17 landmarks * 16 bytes each (2D)

        let arr3d = LandmarkArray::mediapipe_pose();
        assert_eq!(arr3d.data_size(), 33 * 20); // 33 landmarks * 20 bytes each (3D)
    }
}
