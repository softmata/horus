// 3D perception and point cloud message types for robotics
//
// This module provides messages for 3D sensors, point clouds,
// object detection, and spatial understanding systems.
//
// ## Unified Types
//
// The primary `PointCloud` and `DepthImage` types are zero-copy Pod
// descriptors from `horus_core::types`. Use `Topic<PointCloud>` with
// `create()` / `publish()` / `next()`.

use crate::messages::geometry::{Point3, Vector3};
use horus_core::core::LogSummary;
use horus_core::types::TensorDtype;
use serde::{Deserialize, Serialize};
use serde_arrays;

/// Point field descriptor
#[derive(Debug, Clone, Copy, Serialize, Deserialize, Default)]
pub struct PointField {
    /// Field name ("x", "y", "z", "rgb", "intensity", etc.)
    pub name: [u8; 16],
    /// Byte offset in point data structure
    pub offset: u32,
    /// Data type of this field (uses canonical TensorDtype from horus_core::types)
    pub datatype: TensorDtype,
    /// Number of elements (1 for scalar, >1 for vector/array)
    pub count: u32,
}

impl PointField {
    /// Create a new point field
    pub fn new(name: &str, offset: u32, datatype: TensorDtype, count: u32) -> Self {
        let mut field = Self {
            offset,
            datatype,
            count,
            name: [0; 16],
        };

        let name_bytes = name.as_bytes();
        let len = name_bytes.len().min(15);
        field.name[..len].copy_from_slice(&name_bytes[..len]);
        field.name[len] = 0;

        field
    }

    /// Get field name as string
    pub fn name_str(&self) -> String {
        let end = self.name.iter().position(|&b| b == 0).unwrap_or(16);
        String::from_utf8_lossy(&self.name[..end]).into_owned()
    }

    /// Get size in bytes of this field
    pub fn field_size(&self) -> u32 {
        self.datatype.element_size() as u32 * self.count
    }
}

/// Planar surface detection result
#[derive(Debug, Clone, Copy, Serialize, Deserialize, Default)]
pub struct PlaneDetection {
    /// Plane equation coefficients [a, b, c, d] where ax + by + cz + d = 0
    pub coefficients: [f64; 4],
    /// Center point of the plane
    pub center: Point3,
    /// Normal vector of the plane
    pub normal: Vector3,
    /// Plane size (width, height) if bounded
    pub size: [f64; 2],
    /// Number of inlier points
    pub inlier_count: u32,
    /// Confidence in detection (0.0 to 1.0)
    pub confidence: f32,
    /// Plane type label ("floor", "wall", "table", etc.)
    pub plane_type: [u8; 16],
    /// Timestamp in nanoseconds since epoch
    pub timestamp_ns: u64,
}

impl PlaneDetection {
    /// Create a new plane detection
    pub fn new(coefficients: [f64; 4], center: Point3, normal: Vector3) -> Self {
        Self {
            coefficients,
            center,
            normal,
            size: [0.0, 0.0],
            inlier_count: 0,
            confidence: 0.5,
            plane_type: [0; 16],
            timestamp_ns: crate::hframe::timestamp_now(),
        }
    }

    /// Calculate distance from point to plane
    pub fn distance_to_point(&self, point: &Point3) -> f64 {
        let [a, b, c, d] = self.coefficients;
        (a * point.x + b * point.y + c * point.z + d).abs() / (a * a + b * b + c * c).sqrt()
    }

    /// Check if point is on the plane (within tolerance)
    pub fn contains_point(&self, point: &Point3, tolerance: f64) -> bool {
        self.distance_to_point(point) <= tolerance
    }

    /// Set plane type label
    pub fn with_type(mut self, plane_type: &str) -> Self {
        let type_bytes = plane_type.as_bytes();
        let len = type_bytes.len().min(15);
        self.plane_type[..len].copy_from_slice(&type_bytes[..len]);
        self.plane_type[len] = 0;
        self
    }

    /// Get plane type as string
    pub fn plane_type_str(&self) -> String {
        let end = self.plane_type.iter().position(|&b| b == 0).unwrap_or(16);
        String::from_utf8_lossy(&self.plane_type[..end]).into_owned()
    }
}

/// Array of plane detections
#[derive(Debug, Clone, Serialize, Deserialize, Default)]
pub struct PlaneArray {
    /// Array of plane detections (max 16)
    #[serde(with = "serde_arrays")]
    pub planes: [PlaneDetection; 16],
    /// Number of valid planes
    pub count: u8,
    /// Source sensor frame
    pub frame_id: [u8; 32],
    /// Detection algorithm used
    pub algorithm: [u8; 32],
    /// Timestamp in nanoseconds since epoch
    pub timestamp_ns: u64,
}

// ============================================================================
// LogSummary Implementations - Zero-copy logging support
// ============================================================================

impl LogSummary for PointField {
    fn log_summary(&self) -> String {
        format!("PointField('{}', {})", self.name_str(), self.datatype)
    }
}

impl LogSummary for PlaneDetection {
    fn log_summary(&self) -> String {
        format!(
            "Plane('{}', {} inliers, conf={:.2})",
            self.plane_type_str(),
            self.inlier_count,
            self.confidence
        )
    }
}

impl LogSummary for PlaneArray {
    fn log_summary(&self) -> String {
        format!("PlaneArray({} planes)", self.count)
    }
}
