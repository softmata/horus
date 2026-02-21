// Vision and camera message types for robotics
//
// This module provides comprehensive vision processing messages for
// cameras, images, and visual perception systems.
//
// ## Unified Image Type
//
// The primary `Image` type is a zero-copy Pod descriptor from `horus_types`.
// Use `Topic<Image>` with `create()` / `publish()` / `next()`
// for high-performance image pipelines.

use horus_core::core::LogSummary;
use serde::{Deserialize, Serialize};
use serde_arrays;

// Re-export ImageEncoding from horus_types (canonical location)
pub use horus_types::ImageEncoding;

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
            timestamp_ns: crate::hframe::timestamp_now(),
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
            timestamp_ns: crate::hframe::timestamp_now(),
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

impl LogSummary for StereoInfo {
    fn log_summary(&self) -> String {
        format!(
            "StereoInfo({}x{}, baseline={:.3}m)",
            self.left_camera.width, self.left_camera.height, self.baseline
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;

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
    fn test_roi_operations() {
        let roi = RegionOfInterest::new(10, 20, 100, 80);
        assert!(roi.contains(50, 50));
        assert!(!roi.contains(5, 50));
        assert_eq!(roi.area(), 8000);
    }
}
