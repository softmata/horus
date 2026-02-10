// Vision and camera message types for robotics
//
// This module provides comprehensive vision processing messages for
// cameras, images, and visual perception systems.

use horus_core::core::LogSummary;
use serde::{Deserialize, Serialize};
use serde_arrays;

/// Image encoding formats
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[repr(u8)]
#[derive(Default)]
pub enum ImageEncoding {
    /// 8-bit monochrome
    Mono8 = 0,
    /// 16-bit monochrome
    Mono16 = 1,
    /// 8-bit RGB (3 channels)
    #[default]
    Rgb8 = 2,
    /// 8-bit BGR (3 channels, OpenCV format)
    Bgr8 = 3,
    /// 8-bit RGBA (4 channels)
    Rgba8 = 4,
    /// 8-bit BGRA (4 channels)
    Bgra8 = 5,
    /// YUV 4:2:2 format
    Yuv422 = 6,
    /// 32-bit float monochrome
    Mono32F = 7,
    /// 32-bit float RGB
    Rgb32F = 8,
    /// Bayer pattern (raw sensor data)
    BayerRggb8 = 9,
    /// 16-bit depth image (millimeters)
    Depth16 = 10,
}

/// Raw image data message
///
/// For performance-critical applications, consider using shared memory
/// for large image data instead of copying through message channels.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Image {
    /// Image width in pixels
    pub width: u32,
    /// Image height in pixels
    pub height: u32,
    /// Pixel encoding format
    pub encoding: ImageEncoding,
    /// Bytes per row (may include padding)
    pub step: u32,
    /// Image data (row-major order)
    pub data: Vec<u8>,
    /// Frame ID (camera identifier)
    pub frame_id: [u8; 32],
    /// Timestamp in nanoseconds since epoch
    pub timestamp_ns: u64,
}

impl Default for Image {
    fn default() -> Self {
        Self {
            width: 0,
            height: 0,
            encoding: ImageEncoding::Rgb8,
            step: 0,
            data: Vec::new(),
            frame_id: [0; 32],
            timestamp_ns: 0,
        }
    }
}

impl Image {
    /// Create a new image message
    pub fn new(width: u32, height: u32, encoding: ImageEncoding, data: Vec<u8>) -> Self {
        let bytes_per_pixel = encoding.bytes_per_pixel();
        Self {
            width,
            height,
            encoding,
            step: width * bytes_per_pixel,
            data,
            frame_id: [0; 32],
            timestamp_ns: std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap_or_default()
                .as_nanos() as u64,
        }
    }

    /// Set the frame ID from string
    pub fn with_frame_id(mut self, frame_id: &str) -> Self {
        let frame_bytes = frame_id.as_bytes();
        let len = frame_bytes.len().min(31);
        self.frame_id[..len].copy_from_slice(&frame_bytes[..len]);
        self.frame_id[len] = 0;
        self
    }

    /// Get expected data size in bytes
    pub fn expected_size(&self) -> usize {
        (self.step * self.height) as usize
    }

    /// Validate image data consistency
    pub fn is_valid(&self) -> bool {
        self.width > 0
            && self.height > 0
            && self.step >= self.width * self.encoding.bytes_per_pixel()
            && self.data.len() >= self.expected_size()
    }

    /// Get pixel at coordinates (x, y)
    pub fn get_pixel(&self, x: u32, y: u32) -> Option<&[u8]> {
        if x >= self.width || y >= self.height {
            return None;
        }

        let bytes_per_pixel = self.encoding.bytes_per_pixel() as usize;
        let offset = (y * self.step + x * self.encoding.bytes_per_pixel()) as usize;

        if offset + bytes_per_pixel <= self.data.len() {
            Some(&self.data[offset..offset + bytes_per_pixel])
        } else {
            None
        }
    }

    /// Create a region of interest (crop)
    pub fn roi(&self, x: u32, y: u32, width: u32, height: u32) -> Option<Image> {
        if x + width > self.width || y + height > self.height {
            return None;
        }

        let bytes_per_pixel = self.encoding.bytes_per_pixel() as usize;
        let mut roi_data = Vec::with_capacity((width * height) as usize * bytes_per_pixel);

        for row in y..y + height {
            let start = (row * self.step + x * self.encoding.bytes_per_pixel()) as usize;
            let end = start + (width * self.encoding.bytes_per_pixel()) as usize;
            roi_data.extend_from_slice(&self.data[start..end]);
        }

        Some(Image::new(width, height, self.encoding, roi_data))
    }
}

impl ImageEncoding {
    /// Get bytes per pixel for this encoding
    pub fn bytes_per_pixel(&self) -> u32 {
        match self {
            ImageEncoding::Mono8 => 1,
            ImageEncoding::Mono16 => 2,
            ImageEncoding::Rgb8 | ImageEncoding::Bgr8 => 3,
            ImageEncoding::Rgba8 | ImageEncoding::Bgra8 => 4,
            ImageEncoding::Yuv422 => 2,
            ImageEncoding::Mono32F => 4,
            ImageEncoding::Rgb32F => 12,
            ImageEncoding::BayerRggb8 => 1,
            ImageEncoding::Depth16 => 2,
        }
    }

    /// Check if encoding has color information
    pub fn is_color(&self) -> bool {
        matches!(
            self,
            ImageEncoding::Rgb8
                | ImageEncoding::Bgr8
                | ImageEncoding::Rgba8
                | ImageEncoding::Bgra8
                | ImageEncoding::Yuv422
                | ImageEncoding::Rgb32F
                | ImageEncoding::BayerRggb8
        )
    }
}

/// Compressed image data (JPEG, PNG, etc.)
#[derive(Debug, Clone, Serialize, Deserialize, Default)]
pub struct CompressedImage {
    /// Compression format ("jpeg", "png", "webp")
    pub format: [u8; 8],
    /// Compressed image data
    pub data: Vec<u8>,
    /// Original image width (if known)
    pub width: u32,
    /// Original image height (if known)
    pub height: u32,
    /// Frame ID (camera identifier)
    pub frame_id: [u8; 32],
    /// Timestamp in nanoseconds since epoch
    pub timestamp_ns: u64,
}

impl CompressedImage {
    /// Create a new compressed image
    pub fn new(format: &str, data: Vec<u8>) -> Self {
        let mut format_bytes = [0; 8];
        let format_str = format.as_bytes();
        let len = format_str.len().min(7);
        format_bytes[..len].copy_from_slice(&format_str[..len]);

        Self {
            format: format_bytes,
            data,
            width: 0,
            height: 0,
            frame_id: [0; 32],
            timestamp_ns: std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap_or_default()
                .as_nanos() as u64,
        }
    }

    /// Get format as string
    pub fn format_str(&self) -> String {
        let end = self.format.iter().position(|&b| b == 0).unwrap_or(8);
        String::from_utf8_lossy(&self.format[..end]).into_owned()
    }
}

/// Camera calibration information
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct CameraInfo {
    /// Image width in pixels
    pub width: u32,
    /// Image height in pixels
    pub height: u32,
    /// Distortion model ("plumb_bob", "rational_polynomial")
    pub distortion_model: [u8; 16],
    /// Distortion coefficients [k1, k2, p1, p2, k3, k4, k5, k6]
    #[serde(with = "serde_arrays")]
    pub distortion_coefficients: [f64; 8],
    /// Camera intrinsic matrix (3x3, row-major)
    /// [fx, 0,  cx]
    /// [0,  fy, cy]
    /// [0,  0,  1 ]
    pub camera_matrix: [f64; 9],
    /// Rectification matrix (3x3, row-major)
    pub rectification_matrix: [f64; 9],
    /// Projection matrix (3x4, row-major)
    /// [fx', 0,   cx', Tx]
    /// [0,   fy', cy', Ty]
    /// [0,   0,   1,   0 ]
    #[serde(with = "serde_arrays")]
    pub projection_matrix: [f64; 12],
    /// Frame ID (camera identifier)
    pub frame_id: [u8; 32],
    /// Timestamp in nanoseconds since epoch
    pub timestamp_ns: u64,
}

impl Default for CameraInfo {
    fn default() -> Self {
        Self {
            width: 0,
            height: 0,
            distortion_model: [0; 16],
            distortion_coefficients: [0.0; 8],
            camera_matrix: [0.0; 9],
            rectification_matrix: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0], // Identity
            projection_matrix: [0.0; 12],
            frame_id: [0; 32],
            timestamp_ns: 0,
        }
    }
}

impl CameraInfo {
    /// Create camera info with basic parameters
    pub fn new(width: u32, height: u32, fx: f64, fy: f64, cx: f64, cy: f64) -> Self {
        Self {
            width,
            height,
            camera_matrix: [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0],
            projection_matrix: [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0],
            timestamp_ns: std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap_or_default()
                .as_nanos() as u64,
            ..Default::default()
        }
    }

    /// Get focal lengths
    pub fn focal_lengths(&self) -> (f64, f64) {
        (self.camera_matrix[0], self.camera_matrix[4])
    }

    /// Get principal point
    pub fn principal_point(&self) -> (f64, f64) {
        (self.camera_matrix[2], self.camera_matrix[5])
    }

    /// Set distortion model
    pub fn with_distortion_model(mut self, model: &str) -> Self {
        let model_bytes = model.as_bytes();
        let len = model_bytes.len().min(15);
        self.distortion_model[..len].copy_from_slice(&model_bytes[..len]);
        self.distortion_model[len] = 0;
        self
    }
}

/// Region of Interest in an image
#[derive(Debug, Clone, Copy, Serialize, Deserialize, Default)]
pub struct RegionOfInterest {
    /// X offset of the region
    pub x_offset: u32,
    /// Y offset of the region
    pub y_offset: u32,
    /// Width of the region
    pub width: u32,
    /// Height of the region
    pub height: u32,
    /// Whether the region is actively being used
    pub do_rectify: bool,
}

impl RegionOfInterest {
    /// Create a new ROI
    pub fn new(x: u32, y: u32, width: u32, height: u32) -> Self {
        Self {
            x_offset: x,
            y_offset: y,
            width,
            height,
            do_rectify: false,
        }
    }

    /// Check if point is inside ROI
    pub fn contains(&self, x: u32, y: u32) -> bool {
        x >= self.x_offset
            && x < self.x_offset + self.width
            && y >= self.y_offset
            && y < self.y_offset + self.height
    }

    /// Get area of ROI
    pub fn area(&self) -> u32 {
        self.width * self.height
    }
}

/// Visual detection/recognition result
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct Detection {
    /// Object class name
    pub class_name: [u8; 32],
    /// Detection confidence (0.0 to 1.0)
    pub confidence: f32,
    /// Bounding box
    pub bbox: RegionOfInterest,
    /// 3D pose if available
    pub pose: Option<crate::messages::geometry::TransformStamped>,
    /// Object ID for tracking
    pub track_id: u32,
    /// Timestamp in nanoseconds since epoch
    pub timestamp_ns: u64,
}

impl Default for Detection {
    fn default() -> Self {
        Self {
            class_name: [0; 32],
            confidence: 0.0,
            bbox: RegionOfInterest::default(),
            pose: None,
            track_id: 0,
            timestamp_ns: 0,
        }
    }
}

impl Detection {
    /// Create a new detection
    pub fn new(class_name: &str, confidence: f32, bbox: RegionOfInterest) -> Self {
        let mut name_bytes = [0; 32];
        let class_bytes = class_name.as_bytes();
        let len = class_bytes.len().min(31);
        name_bytes[..len].copy_from_slice(&class_bytes[..len]);

        Self {
            class_name: name_bytes,
            confidence,
            bbox,
            pose: None,
            track_id: 0,
            timestamp_ns: std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap_or_default()
                .as_nanos() as u64,
        }
    }

    /// Get class name as string
    pub fn class_str(&self) -> String {
        let end = self.class_name.iter().position(|&b| b == 0).unwrap_or(32);
        String::from_utf8_lossy(&self.class_name[..end]).into_owned()
    }
}

/// Array of visual detections
#[derive(Debug, Clone, Serialize, Deserialize, Default)]
pub struct DetectionArray {
    /// Array of detections (max 32)
    #[serde(with = "serde_arrays")]
    pub detections: [Detection; 32],
    /// Number of valid detections
    pub count: u8,
    /// Source image info
    pub image_width: u32,
    pub image_height: u32,
    /// Frame ID
    pub frame_id: [u8; 32],
    /// Timestamp in nanoseconds since epoch
    pub timestamp_ns: u64,
}

impl DetectionArray {
    /// Create a new detection array
    pub fn new() -> Self {
        Self {
            timestamp_ns: std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap_or_default()
                .as_nanos() as u64,
            ..Default::default()
        }
    }

    /// Add a detection
    pub fn add_detection(&mut self, detection: Detection) -> Result<(), &'static str> {
        if self.count >= 32 {
            return Err("Maximum 32 detections supported");
        }

        self.detections[self.count as usize] = detection;
        self.count += 1;
        Ok(())
    }

    /// Get valid detections
    pub fn get_detections(&self) -> &[Detection] {
        &self.detections[..self.count as usize]
    }

    /// Filter detections by confidence threshold
    pub fn filter_by_confidence(&self, threshold: f32) -> Vec<Detection> {
        self.get_detections()
            .iter()
            .filter(|d| d.confidence >= threshold)
            .cloned()
            .collect()
    }
}

/// Stereo camera pair information
#[derive(Debug, Clone, Copy, Serialize, Deserialize, Default)]
pub struct StereoInfo {
    /// Left camera info
    pub left_camera: CameraInfo,
    /// Right camera info
    pub right_camera: CameraInfo,
    /// Baseline (distance between cameras) in meters
    pub baseline: f64,
    /// Disparity-to-depth conversion factor
    pub depth_scale: f64,
}

impl StereoInfo {
    /// Calculate depth from disparity
    pub fn depth_from_disparity(&self, disparity: f32) -> f32 {
        if disparity <= 0.0 {
            return f32::INFINITY;
        }
        (self.baseline * self.left_camera.focal_lengths().0) as f32 / disparity
    }

    /// Calculate disparity from depth
    pub fn disparity_from_depth(&self, depth: f32) -> f32 {
        if depth <= 0.0 {
            return 0.0;
        }
        (self.baseline * self.left_camera.focal_lengths().0) as f32 / depth
    }
}

// ============================================================================
// LogSummary Implementations - Zero-copy logging support
// ============================================================================

impl LogSummary for Image {
    fn log_summary(&self) -> String {
        format!(
            "Image({}x{}, {:?}, {} bytes)",
            self.width,
            self.height,
            self.encoding,
            self.data.len()
        )
    }
}

impl LogSummary for CompressedImage {
    fn log_summary(&self) -> String {
        format!(
            "CompressedImage({}, {}x{}, {} bytes)",
            self.format_str(),
            self.width,
            self.height,
            self.data.len()
        )
    }
}

impl LogSummary for CameraInfo {
    fn log_summary(&self) -> String {
        let (fx, fy) = self.focal_lengths();
        format!(
            "CameraInfo({}x{}, f={:.1}/{:.1})",
            self.width, self.height, fx, fy
        )
    }
}

impl LogSummary for RegionOfInterest {
    fn log_summary(&self) -> String {
        format!(
            "ROI(x={}, y={}, {}x{})",
            self.x_offset, self.y_offset, self.width, self.height
        )
    }
}

impl LogSummary for Detection {
    fn log_summary(&self) -> String {
        format!(
            "Detection('{}', conf={:.2}, bbox={}x{})",
            self.class_str(),
            self.confidence,
            self.bbox.width,
            self.bbox.height
        )
    }
}

impl LogSummary for DetectionArray {
    fn log_summary(&self) -> String {
        format!("DetectionArray({} detections)", self.count)
    }
}

impl LogSummary for StereoInfo {
    fn log_summary(&self) -> String {
        format!(
            "StereoInfo({}x{}, baseline={:.3}m)",
            self.left_camera.width, self.left_camera.height, self.baseline
        )
    }
}

impl LogSummary for ImageEncoding {
    fn log_summary(&self) -> String {
        format!("{:?}", self)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_image_creation() {
        let data = vec![255, 0, 0, 0, 255, 0, 0, 0, 255]; // 3 RGB pixels
        let image = Image::new(3, 1, ImageEncoding::Rgb8, data);

        assert_eq!(image.width, 3);
        assert_eq!(image.height, 1);
        assert_eq!(image.encoding, ImageEncoding::Rgb8);
        assert_eq!(image.step, 9); // 3 pixels * 3 bytes
        assert!(image.is_valid());
    }

    #[test]
    fn test_encoding_properties() {
        assert_eq!(ImageEncoding::Rgb8.bytes_per_pixel(), 3);
        assert_eq!(ImageEncoding::Mono16.bytes_per_pixel(), 2);
        assert!(ImageEncoding::Rgb8.is_color());
        assert!(!ImageEncoding::Mono8.is_color());
    }

    #[test]
    fn test_camera_info() {
        let info = CameraInfo::new(640, 480, 525.0, 525.0, 320.0, 240.0);
        assert_eq!(info.focal_lengths(), (525.0, 525.0));
        assert_eq!(info.principal_point(), (320.0, 240.0));
    }

    #[test]
    fn test_detection_array() {
        let mut array = DetectionArray::new();
        let detection = Detection::new("person", 0.95, RegionOfInterest::new(10, 20, 100, 150));

        array.add_detection(detection).unwrap();
        assert_eq!(array.count, 1);

        let filtered = array.filter_by_confidence(0.9);
        assert_eq!(filtered.len(), 1);

        let filtered_high = array.filter_by_confidence(0.99);
        assert_eq!(filtered_high.len(), 0);
    }

    #[test]
    fn test_roi_operations() {
        let roi = RegionOfInterest::new(10, 20, 100, 80);
        assert!(roi.contains(50, 50));
        assert!(!roi.contains(5, 50));
        assert_eq!(roi.area(), 8000);
    }
}
