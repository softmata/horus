//! Object detection types for zero-copy IPC
//!
//! Fixed-size types for 2D and 3D object detection results from models
//! like YOLO, SSD, etc. Suitable for shared memory transport.

use bytemuck::{Pod, Zeroable};

/// 2D bounding box (x, y, width, height in pixels)///
/// Size: 16 bytes
#[repr(C)]
#[derive(Debug, Clone, Copy, Default, PartialEq, Pod, Zeroable)]
pub struct BoundingBox2D {
    /// X coordinate of top-left corner (pixels)
    pub x: f32,
    /// Y coordinate of top-left corner (pixels)
    pub y: f32,
    /// Width of bounding box (pixels)
    pub width: f32,
    /// Height of bounding box (pixels)
    pub height: f32,
}

impl BoundingBox2D {
    /// Create a new 2D bounding box
    pub fn new(x: f32, y: f32, width: f32, height: f32) -> Self {
        Self {
            x,
            y,
            width,
            height,
        }
    }

    /// Create from center coordinates (cx, cy, w, h) — YOLO format
    pub fn from_center(cx: f32, cy: f32, width: f32, height: f32) -> Self {
        Self {
            x: cx - width / 2.0,
            y: cy - height / 2.0,
            width,
            height,
        }
    }

    /// Get center x coordinate
    pub fn center_x(&self) -> f32 {
        self.x + self.width / 2.0
    }

    /// Get center y coordinate
    pub fn center_y(&self) -> f32 {
        self.y + self.height / 2.0
    }

    /// Get area
    pub fn area(&self) -> f32 {
        self.width * self.height
    }

    /// Calculate Intersection over Union (IoU) with another box
    pub fn iou(&self, other: &BoundingBox2D) -> f32 {
        let x1 = self.x.max(other.x);
        let y1 = self.y.max(other.y);
        let x2 = (self.x + self.width).min(other.x + other.width);
        let y2 = (self.y + self.height).min(other.y + other.height);

        let intersection = (x2 - x1).max(0.0) * (y2 - y1).max(0.0);
        let union = self.area() + other.area() - intersection;

        if union > 0.0 {
            intersection / union
        } else {
            0.0
        }
    }
}

/// 3D bounding box (center + dimensions + rotation)///
/// Uses Euler angles (roll/pitch/yaw) instead of Quaternion for Pod compatibility.
///
/// Size: 48 bytes
#[repr(C)]
#[derive(Debug, Clone, Copy, Default, PartialEq, Pod, Zeroable)]
pub struct BoundingBox3D {
    /// Center X coordinate (meters)
    pub cx: f32,
    /// Center Y coordinate (meters)
    pub cy: f32,
    /// Center Z coordinate (meters)
    pub cz: f32,
    /// Length along X axis (meters)
    pub length: f32,
    /// Width along Y axis (meters)
    pub width: f32,
    /// Height along Z axis (meters)
    pub height: f32,
    /// Rotation around X axis (radians)
    pub roll: f32,
    /// Rotation around Y axis (radians)
    pub pitch: f32,
    /// Rotation around Z axis (radians, most common)
    pub yaw: f32,
    /// Padding for alignment
    _pad: [f32; 3],
}

impl BoundingBox3D {
    /// Create a new 3D bounding box with yaw rotation only
    pub fn new(cx: f32, cy: f32, cz: f32, length: f32, width: f32, height: f32, yaw: f32) -> Self {
        Self {
            cx,
            cy,
            cz,
            length,
            width,
            height,
            roll: 0.0,
            pitch: 0.0,
            yaw,
            _pad: [0.0; 3],
        }
    }

    /// Create with full rotation
    #[allow(clippy::too_many_arguments)]
    pub fn with_rotation(
        cx: f32,
        cy: f32,
        cz: f32,
        length: f32,
        width: f32,
        height: f32,
        roll: f32,
        pitch: f32,
        yaw: f32,
    ) -> Self {
        Self {
            cx,
            cy,
            cz,
            length,
            width,
            height,
            roll,
            pitch,
            yaw,
            _pad: [0.0; 3],
        }
    }

    /// Get volume
    pub fn volume(&self) -> f32 {
        self.length * self.width * self.height
    }
}

/// 2D object detection result///
/// Fixed-size struct suitable for zero-copy IPC. The class is stored as a
/// 32-byte UTF-8 string (truncated if longer).
///
/// Size: 72 bytes
#[repr(C)]
#[derive(Debug, Clone, Copy, Pod, Zeroable)]
pub struct Detection {
    /// Bounding box (x, y, width, height)
    pub bbox: BoundingBox2D,
    /// Confidence score (0.0 - 1.0)
    pub confidence: f32,
    /// Class ID (numeric identifier)
    pub class_id: u32,
    /// Class name (UTF-8, null-padded, max 31 chars + null)
    pub class_name: [u8; 32],
    /// Instance ID (for instance segmentation)
    pub instance_id: u32,
    /// Padding for alignment
    _pad: [u8; 12],
}

impl Default for Detection {
    fn default() -> Self {
        Self {
            bbox: BoundingBox2D::default(),
            confidence: 0.0,
            class_id: 0,
            class_name: [0u8; 32],
            instance_id: 0,
            _pad: [0u8; 12],
        }
    }
}

impl Detection {
    /// Create a new detection
    pub fn new(class_name: &str, confidence: f32, x: f32, y: f32, width: f32, height: f32) -> Self {
        let mut det = Self {
            bbox: BoundingBox2D::new(x, y, width, height),
            confidence,
            class_id: 0,
            class_name: [0u8; 32],
            instance_id: 0,
            _pad: [0u8; 12],
        };
        det.set_class_name(class_name);
        det
    }

    /// Create with class ID
    pub fn with_class_id(class_id: u32, confidence: f32, bbox: BoundingBox2D) -> Self {
        Self {
            bbox,
            confidence,
            class_id,
            class_name: [0u8; 32],
            instance_id: 0,
            _pad: [0u8; 12],
        }
    }

    /// Set the class name (truncates to 31 chars)
    pub fn set_class_name(&mut self, name: &str) {
        let bytes = name.as_bytes();
        let len = bytes.len().min(31);
        self.class_name[..len].copy_from_slice(&bytes[..len]);
        self.class_name[len..].fill(0);
    }

    /// Get the class name as a string
    pub fn get_class_name(&self) -> &str {
        let end = self.class_name.iter().position(|&b| b == 0).unwrap_or(32);
        std::str::from_utf8(&self.class_name[..end]).unwrap_or("")
    }

    /// Check if detection passes confidence threshold
    pub fn is_confident(&self, threshold: f32) -> bool {
        self.confidence >= threshold
    }
}

/// 3D object detection result///
/// For 3D object detection from point clouds or depth-aware models.
///
/// Size: 104 bytes
#[repr(C)]
#[derive(Debug, Clone, Copy, Pod, Zeroable)]
pub struct Detection3D {
    /// 3D bounding box
    pub bbox: BoundingBox3D,
    /// Confidence score (0.0 - 1.0)
    pub confidence: f32,
    /// Class ID (numeric identifier)
    pub class_id: u32,
    /// Class name (UTF-8, null-padded, max 31 chars + null)
    pub class_name: [u8; 32],
    /// Velocity X (m/s, for tracking-enabled detectors)
    pub velocity_x: f32,
    /// Velocity Y (m/s)
    pub velocity_y: f32,
    /// Velocity Z (m/s)
    pub velocity_z: f32,
    /// Instance/tracking ID
    pub instance_id: u32,
}

impl Default for Detection3D {
    fn default() -> Self {
        Self {
            bbox: BoundingBox3D::default(),
            confidence: 0.0,
            class_id: 0,
            class_name: [0u8; 32],
            velocity_x: 0.0,
            velocity_y: 0.0,
            velocity_z: 0.0,
            instance_id: 0,
        }
    }
}

impl Detection3D {
    /// Create a new 3D detection
    pub fn new(class_name: &str, confidence: f32, bbox: BoundingBox3D) -> Self {
        let mut det = Self {
            bbox,
            confidence,
            ..Default::default()
        };
        det.set_class_name(class_name);
        det
    }

    /// Set the class name (truncates to 31 chars)
    pub fn set_class_name(&mut self, name: &str) {
        let bytes = name.as_bytes();
        let len = bytes.len().min(31);
        self.class_name[..len].copy_from_slice(&bytes[..len]);
        self.class_name[len..].fill(0);
    }

    /// Get the class name as a string
    pub fn get_class_name(&self) -> &str {
        let end = self.class_name.iter().position(|&b| b == 0).unwrap_or(32);
        std::str::from_utf8(&self.class_name[..end]).unwrap_or("")
    }

    /// Set velocity
    pub fn with_velocity(mut self, vx: f32, vy: f32, vz: f32) -> Self {
        self.velocity_x = vx;
        self.velocity_y = vy;
        self.velocity_z = vz;
        self
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_bbox2d_size() {
        assert_eq!(std::mem::size_of::<BoundingBox2D>(), 16);
    }

    #[test]
    fn test_bbox3d_size() {
        assert_eq!(std::mem::size_of::<BoundingBox3D>(), 48);
    }

    #[test]
    fn test_detection_pod_size() {
        assert_eq!(std::mem::size_of::<Detection>(), 72);
    }

    #[test]
    fn test_detection3d_pod_size() {
        assert_eq!(std::mem::size_of::<Detection3D>(), 104);
    }

    #[test]
    fn test_detection_class_name() {
        let mut det = Detection::default();
        det.set_class_name("person");
        assert_eq!(det.get_class_name(), "person");

        // Test truncation
        det.set_class_name("this_is_a_very_long_class_name_that_should_be_truncated");
        assert_eq!(det.get_class_name().len(), 31);
    }

    #[test]
    fn test_bbox_iou() {
        let a = BoundingBox2D::new(0.0, 0.0, 100.0, 100.0);
        let b = BoundingBox2D::new(50.0, 50.0, 100.0, 100.0);

        // Intersection is 50x50 = 2500
        // Union is 100x100 + 100x100 - 2500 = 17500
        let iou = a.iou(&b);
        assert!((iou - 2500.0 / 17500.0).abs() < 0.001);
    }

    #[test]
    fn test_bbox_from_center() {
        let bbox = BoundingBox2D::from_center(100.0, 100.0, 50.0, 30.0);
        assert_eq!(bbox.x, 75.0);
        assert_eq!(bbox.y, 85.0);
        assert_eq!(bbox.width, 50.0);
        assert_eq!(bbox.height, 30.0);
    }

    #[test]
    fn test_bbox3d_volume() {
        let bbox = BoundingBox3D::new(0.0, 0.0, 0.0, 2.0, 3.0, 4.0, 0.0);
        assert!((bbox.volume() - 24.0).abs() < 0.001);
    }

    #[test]
    fn test_detection3d_velocity() {
        let det = Detection3D::new(
            "car",
            0.9,
            BoundingBox3D::new(1.0, 2.0, 0.5, 4.5, 2.0, 1.5, 0.1),
        )
        .with_velocity(10.0, 5.0, 0.0);
        assert_eq!(det.velocity_x, 10.0);
        assert_eq!(det.velocity_y, 5.0);
        assert_eq!(det.get_class_name(), "car");
    }

    #[test]
    fn test_detection_confident() {
        let det = Detection::new("person", 0.85, 10.0, 20.0, 50.0, 100.0);
        assert!(det.is_confident(0.5));
        assert!(!det.is_confident(0.9));
    }
}
