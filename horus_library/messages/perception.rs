// 3D perception and point cloud message types for robotics
//
// This module provides messages for 3D sensors, point clouds,
// object detection, and spatial understanding systems.

use crate::messages::geometry::{Point3, Quaternion, Vector3};
use horus_core::core::LogSummary;
use serde::{Deserialize, Serialize};
use serde_arrays;

/// Point field description for flexible point cloud data
#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq, Eq)]
#[repr(u8)]
#[derive(Default)]
pub enum PointFieldType {
    /// 8-bit integer
    Int8 = 1,
    /// 8-bit unsigned integer
    UInt8 = 2,
    /// 16-bit integer
    Int16 = 3,
    /// 16-bit unsigned integer
    UInt16 = 4,
    /// 32-bit integer
    Int32 = 5,
    /// 32-bit unsigned integer
    UInt32 = 6,
    /// 32-bit float
    #[default]
    Float32 = 7,
    /// 64-bit float
    Float64 = 8,
}

/// Point field descriptor
#[derive(Debug, Clone, Copy, Serialize, Deserialize, Default)]
pub struct PointField {
    /// Field name ("x", "y", "z", "rgb", "intensity", etc.)
    pub name: [u8; 16],
    /// Byte offset in point data structure
    pub offset: u32,
    /// Data type of this field
    pub datatype: PointFieldType,
    /// Number of elements (1 for scalar, >1 for vector/array)
    pub count: u32,
}

impl PointField {
    /// Create a new point field
    pub fn new(name: &str, offset: u32, datatype: PointFieldType, count: u32) -> Self {
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
        let type_size = match self.datatype {
            PointFieldType::Int8 | PointFieldType::UInt8 => 1,
            PointFieldType::Int16 | PointFieldType::UInt16 => 2,
            PointFieldType::Int32 | PointFieldType::UInt32 | PointFieldType::Float32 => 4,
            PointFieldType::Float64 => 8,
        };
        type_size * self.count
    }
}

/// 3D point cloud message
///
/// Represents a collection of 3D points with optional color, intensity,
/// and other attributes. Uses flexible field structure like ROS PointCloud2.
#[derive(Debug, Clone, Serialize, Deserialize, Default)]
pub struct PointCloud {
    /// Point cloud dimensions
    pub width: u32,
    pub height: u32,
    /// Field descriptions
    pub fields: [PointField; 16],
    /// Number of valid fields
    pub field_count: u8,
    /// Is data organized as image (true) or unorganized (false)
    pub is_dense: bool,
    /// Size of each point in bytes
    pub point_step: u32,
    /// Size of each row in bytes
    pub row_step: u32,
    /// Point data (binary blob)
    pub data: Vec<u8>,
    /// Coordinate frame reference
    pub frame_id: [u8; 32],
    /// Timestamp in nanoseconds since epoch
    pub timestamp_ns: u64,
}

impl PointCloud {
    /// Create a new empty point cloud
    pub fn new() -> Self {
        Self {
            timestamp_ns: std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap_or_default()
                .as_nanos() as u64,
            ..Default::default()
        }
    }

    /// Create a basic XYZ point cloud
    pub fn xyz(points: &[Point3]) -> Self {
        let mut cloud = Self::new();
        cloud.width = points.len() as u32;
        cloud.height = 1;
        cloud.is_dense = true;

        // Define XYZ fields
        cloud.fields[0] = PointField::new("x", 0, PointFieldType::Float32, 1);
        cloud.fields[1] = PointField::new("y", 4, PointFieldType::Float32, 1);
        cloud.fields[2] = PointField::new("z", 8, PointFieldType::Float32, 1);
        cloud.field_count = 3;

        cloud.point_step = 12; // 3 * 4 bytes
        cloud.row_step = cloud.point_step * cloud.width;

        // Serialize points to binary data
        let mut data = Vec::with_capacity((cloud.point_step * cloud.width) as usize);
        for point in points {
            data.extend_from_slice(&(point.x as f32).to_le_bytes());
            data.extend_from_slice(&(point.y as f32).to_le_bytes());
            data.extend_from_slice(&(point.z as f32).to_le_bytes());
        }
        cloud.data = data;

        cloud
    }

    /// Create an XYZRGB point cloud with color
    pub fn xyzrgb(points: &[(Point3, [u8; 3])]) -> Self {
        let mut cloud = Self::new();
        cloud.width = points.len() as u32;
        cloud.height = 1;
        cloud.is_dense = true;

        // Define XYZRGB fields
        cloud.fields[0] = PointField::new("x", 0, PointFieldType::Float32, 1);
        cloud.fields[1] = PointField::new("y", 4, PointFieldType::Float32, 1);
        cloud.fields[2] = PointField::new("z", 8, PointFieldType::Float32, 1);
        cloud.fields[3] = PointField::new("rgb", 12, PointFieldType::UInt32, 1);
        cloud.field_count = 4;

        cloud.point_step = 16; // 3*4 + 4 bytes
        cloud.row_step = cloud.point_step * cloud.width;

        // Serialize points to binary data
        let mut data = Vec::with_capacity((cloud.point_step * cloud.width) as usize);
        for (point, color) in points {
            data.extend_from_slice(&(point.x as f32).to_le_bytes());
            data.extend_from_slice(&(point.y as f32).to_le_bytes());
            data.extend_from_slice(&(point.z as f32).to_le_bytes());
            // Pack RGB into single u32 (0x00RRGGBB)
            let rgb_packed =
                ((color[0] as u32) << 16) | ((color[1] as u32) << 8) | (color[2] as u32);
            data.extend_from_slice(&rgb_packed.to_le_bytes());
        }
        cloud.data = data;

        cloud
    }

    /// Add a field to the point cloud
    pub fn add_field(&mut self, field: PointField) -> Result<(), &'static str> {
        if self.field_count >= 16 {
            return Err("Maximum 16 fields supported");
        }

        self.fields[self.field_count as usize] = field;
        self.field_count += 1;
        Ok(())
    }

    /// Get total number of points
    pub fn point_count(&self) -> u32 {
        self.width * self.height
    }

    /// Check if point cloud is valid
    pub fn is_valid(&self) -> bool {
        self.width > 0
            && self.height > 0
            && self.field_count > 0
            && self.point_step > 0
            && self.data.len() >= (self.point_step * self.point_count()) as usize
    }

    /// Extract XYZ coordinates (if available)
    pub fn extract_xyz(&self) -> Option<Vec<Point3>> {
        // Find X, Y, Z fields
        let mut x_field = None;
        let mut y_field = None;
        let mut z_field = None;

        for i in 0..self.field_count {
            let field = &self.fields[i as usize];
            match field.name_str().as_str() {
                "x" => x_field = Some(field),
                "y" => y_field = Some(field),
                "z" => z_field = Some(field),
                _ => {}
            }
        }

        if let (Some(x), Some(y), Some(z)) = (x_field, y_field, z_field) {
            // Verify all are float32
            if x.datatype != PointFieldType::Float32
                || y.datatype != PointFieldType::Float32
                || z.datatype != PointFieldType::Float32
            {
                return None;
            }

            let mut points = Vec::new();
            let point_count = self.point_count() as usize;

            for i in 0..point_count {
                let point_offset = i * self.point_step as usize;

                if point_offset + 12 <= self.data.len() {
                    let x_bytes = &self.data
                        [point_offset + x.offset as usize..point_offset + x.offset as usize + 4];
                    let y_bytes = &self.data
                        [point_offset + y.offset as usize..point_offset + y.offset as usize + 4];
                    let z_bytes = &self.data
                        [point_offset + z.offset as usize..point_offset + z.offset as usize + 4];

                    let x_val =
                        f32::from_le_bytes([x_bytes[0], x_bytes[1], x_bytes[2], x_bytes[3]]) as f64;
                    let y_val =
                        f32::from_le_bytes([y_bytes[0], y_bytes[1], y_bytes[2], y_bytes[3]]) as f64;
                    let z_val =
                        f32::from_le_bytes([z_bytes[0], z_bytes[1], z_bytes[2], z_bytes[3]]) as f64;

                    points.push(Point3::new(x_val, y_val, z_val));
                }
            }

            Some(points)
        } else {
            None
        }
    }

    /// Set frame ID
    pub fn with_frame_id(mut self, frame_id: &str) -> Self {
        let frame_bytes = frame_id.as_bytes();
        let len = frame_bytes.len().min(31);
        self.frame_id[..len].copy_from_slice(&frame_bytes[..len]);
        self.frame_id[len] = 0;
        self
    }
}

/// 3D bounding box
#[derive(Debug, Clone, Copy, Serialize, Deserialize, Default)]
pub struct BoundingBox3D {
    /// Center of the bounding box
    pub center: Point3,
    /// Size [width, height, depth]
    pub size: Vector3,
    /// Orientation of the box
    pub orientation: Quaternion,
    /// Object class label
    pub label: [u8; 32],
    /// Detection confidence (0.0 to 1.0)
    pub confidence: f32,
    /// Tracking ID
    pub track_id: u32,
    /// Timestamp in nanoseconds since epoch
    pub timestamp_ns: u64,
}

impl BoundingBox3D {
    /// Create a new 3D bounding box
    pub fn new(center: Point3, size: Vector3) -> Self {
        Self {
            center,
            size,
            orientation: Quaternion::identity(),
            label: [0; 32],
            confidence: 1.0,
            track_id: 0,
            timestamp_ns: std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap_or_default()
                .as_nanos() as u64,
        }
    }

    /// Set label
    pub fn with_label(mut self, label: &str) -> Self {
        let label_bytes = label.as_bytes();
        let len = label_bytes.len().min(31);
        self.label[..len].copy_from_slice(&label_bytes[..len]);
        self.label[len] = 0;
        self
    }

    /// Get label as string
    pub fn label_str(&self) -> String {
        let end = self.label.iter().position(|&b| b == 0).unwrap_or(32);
        String::from_utf8_lossy(&self.label[..end]).into_owned()
    }

    /// Check if point is inside bounding box
    pub fn contains_point(&self, point: &Point3) -> bool {
        // For simplicity, assume axis-aligned box (ignore orientation)
        let dx = (point.x - self.center.x).abs();
        let dy = (point.y - self.center.y).abs();
        let dz = (point.z - self.center.z).abs();

        dx <= self.size.x / 2.0 && dy <= self.size.y / 2.0 && dz <= self.size.z / 2.0
    }

    /// Get volume of the bounding box
    pub fn volume(&self) -> f64 {
        self.size.x * self.size.y * self.size.z
    }

    /// Get 8 corner points of the bounding box (axis-aligned)
    pub fn corners(&self) -> [Point3; 8] {
        let hx = self.size.x / 2.0;
        let hy = self.size.y / 2.0;
        let hz = self.size.z / 2.0;

        [
            Point3::new(self.center.x - hx, self.center.y - hy, self.center.z - hz),
            Point3::new(self.center.x + hx, self.center.y - hy, self.center.z - hz),
            Point3::new(self.center.x - hx, self.center.y + hy, self.center.z - hz),
            Point3::new(self.center.x + hx, self.center.y + hy, self.center.z - hz),
            Point3::new(self.center.x - hx, self.center.y - hy, self.center.z + hz),
            Point3::new(self.center.x + hx, self.center.y - hy, self.center.z + hz),
            Point3::new(self.center.x - hx, self.center.y + hy, self.center.z + hz),
            Point3::new(self.center.x + hx, self.center.y + hy, self.center.z + hz),
        ]
    }
}

/// Array of 3D bounding boxes
#[derive(Debug, Clone, Serialize, Deserialize, Default)]
pub struct BoundingBoxArray3D {
    /// Array of bounding boxes (max 32)
    #[serde(with = "serde_arrays")]
    pub boxes: [BoundingBox3D; 32],
    /// Number of valid boxes
    pub count: u8,
    /// Source sensor frame
    pub frame_id: [u8; 32],
    /// Timestamp in nanoseconds since epoch
    pub timestamp_ns: u64,
}

impl BoundingBoxArray3D {
    /// Create a new bounding box array
    pub fn new() -> Self {
        Self {
            timestamp_ns: std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap_or_default()
                .as_nanos() as u64,
            ..Default::default()
        }
    }

    /// Add a bounding box
    pub fn add_box(&mut self, bbox: BoundingBox3D) -> Result<(), &'static str> {
        if self.count >= 32 {
            return Err("Maximum 32 bounding boxes supported");
        }

        self.boxes[self.count as usize] = bbox;
        self.count += 1;
        Ok(())
    }

    /// Get valid bounding boxes
    pub fn get_boxes(&self) -> &[BoundingBox3D] {
        &self.boxes[..self.count as usize]
    }

    /// Filter boxes by confidence threshold
    pub fn filter_by_confidence(&self, threshold: f32) -> Vec<BoundingBox3D> {
        self.get_boxes()
            .iter()
            .filter(|bbox| bbox.confidence >= threshold)
            .cloned()
            .collect()
    }

    /// Filter boxes by label
    pub fn filter_by_label(&self, label: &str) -> Vec<BoundingBox3D> {
        self.get_boxes()
            .iter()
            .filter(|bbox| bbox.label_str() == label)
            .cloned()
            .collect()
    }
}

/// Depth image message
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DepthImage {
    /// Image width in pixels
    pub width: u32,
    /// Image height in pixels
    pub height: u32,
    /// Depth values in millimeters (0 = invalid/no measurement)
    pub depths: Vec<u16>,
    /// Minimum reliable depth value
    pub min_depth: u16,
    /// Maximum reliable depth value
    pub max_depth: u16,
    /// Depth scale (mm per unit)
    pub depth_scale: f32,
    /// Frame ID for camera reference
    pub frame_id: [u8; 32],
    /// Timestamp in nanoseconds since epoch
    pub timestamp_ns: u64,
}

impl Default for DepthImage {
    fn default() -> Self {
        Self {
            width: 0,
            height: 0,
            depths: Vec::new(),
            min_depth: 200,   // 20cm minimum
            max_depth: 10000, // 10m maximum
            depth_scale: 1.0, // 1mm per unit
            frame_id: [0; 32],
            timestamp_ns: 0,
        }
    }
}

impl DepthImage {
    /// Create a new depth image
    pub fn new(width: u32, height: u32, depths: Vec<u16>) -> Self {
        Self {
            width,
            height,
            depths,
            timestamp_ns: std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap_or_default()
                .as_nanos() as u64,
            ..Default::default()
        }
    }

    /// Get depth at pixel coordinates
    pub fn get_depth(&self, x: u32, y: u32) -> Option<u16> {
        if x < self.width && y < self.height {
            let index = (y * self.width + x) as usize;
            self.depths.get(index).copied()
        } else {
            None
        }
    }

    /// Set depth at pixel coordinates
    pub fn set_depth(&mut self, x: u32, y: u32, depth: u16) -> bool {
        if x < self.width && y < self.height {
            let index = (y * self.width + x) as usize;
            if index < self.depths.len() {
                self.depths[index] = depth;
                return true;
            }
        }
        false
    }

    /// Check if depth value is valid
    pub fn is_valid_depth(&self, depth: u16) -> bool {
        depth > 0 && depth >= self.min_depth && depth <= self.max_depth
    }

    /// Convert to point cloud using camera intrinsics
    pub fn to_point_cloud(&self, fx: f64, fy: f64, cx: f64, cy: f64) -> PointCloud {
        let mut points = Vec::new();

        for y in 0..self.height {
            for x in 0..self.width {
                if let Some(depth) = self.get_depth(x, y) {
                    if self.is_valid_depth(depth) {
                        let depth_m = (depth as f64 * self.depth_scale as f64) / 1000.0; // Convert to meters

                        // Back-project to 3D
                        let x_3d = (x as f64 - cx) * depth_m / fx;
                        let y_3d = (y as f64 - cy) * depth_m / fy;
                        let z_3d = depth_m;

                        points.push(Point3::new(x_3d, y_3d, z_3d));
                    }
                }
            }
        }

        PointCloud::xyz(&points)
    }

    /// Calculate statistics
    pub fn depth_statistics(&self) -> (f32, f32, f32) {
        // min, max, mean
        let valid_depths: Vec<u16> = self
            .depths
            .iter()
            .filter(|&&d| self.is_valid_depth(d))
            .cloned()
            .collect();

        if valid_depths.is_empty() {
            return (0.0, 0.0, 0.0);
        }

        let min = *valid_depths.iter().min().unwrap() as f32;
        let max = *valid_depths.iter().max().unwrap() as f32;
        let mean = valid_depths.iter().sum::<u16>() as f32 / valid_depths.len() as f32;

        (min, max, mean)
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
            timestamp_ns: std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap_or_default()
                .as_nanos() as u64,
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

impl LogSummary for PointCloud {
    fn log_summary(&self) -> String {
        format!(
            "PointCloud({} points, {} fields, {} bytes)",
            self.point_count(),
            self.field_count,
            self.data.len()
        )
    }
}

impl LogSummary for PointField {
    fn log_summary(&self) -> String {
        format!("PointField('{}', {:?})", self.name_str(), self.datatype)
    }
}

impl LogSummary for PointFieldType {
    fn log_summary(&self) -> String {
        format!("{:?}", self)
    }
}

impl LogSummary for BoundingBox3D {
    fn log_summary(&self) -> String {
        format!(
            "BBox3D('{}', conf={:.2}, vol={:.2})",
            self.label_str(),
            self.confidence,
            self.volume()
        )
    }
}

impl LogSummary for BoundingBoxArray3D {
    fn log_summary(&self) -> String {
        format!("BBoxArray3D({} boxes)", self.count)
    }
}

impl LogSummary for DepthImage {
    fn log_summary(&self) -> String {
        format!(
            "DepthImage({}x{}, {} bytes)",
            self.width,
            self.height,
            self.depths.len() * 2
        )
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
