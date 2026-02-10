//! Object tracking types
//!
//! Types for multi-object tracking (MOT) and tracked object state.

use super::detection::BoundingBox2D;
use bytemuck::{Pod, Zeroable};

/// Tracked object state
///
/// Combines detection with tracking information like ID, velocity, and age.
///
/// Size: 96 bytes
#[repr(C)]
#[derive(Debug, Clone, Copy, Pod, Zeroable)]
pub struct TrackedObject {
    /// Bounding box (x, y, width, height)
    pub bbox: BoundingBox2D,
    /// Predicted bounding box (Kalman filter output)
    pub predicted_bbox: BoundingBox2D,
    /// Unique tracking ID (persistent across frames)
    pub track_id: u64,
    /// Detection confidence (0.0 - 1.0)
    pub confidence: f32,
    /// Class ID
    pub class_id: u32,
    /// Velocity X (pixels/frame or m/s)
    pub velocity_x: f32,
    /// Velocity Y (pixels/frame or m/s)
    pub velocity_y: f32,
    /// Acceleration X
    pub accel_x: f32,
    /// Acceleration Y
    pub accel_y: f32,
    /// Track age (frames since first detection)
    pub age: u32,
    /// Hits (number of frames with detection)
    pub hits: u32,
    /// Consecutive misses (frames without detection)
    pub time_since_update: u32,
    /// Track state: 0=tentative, 1=confirmed, 2=deleted
    pub state: u32,
    /// Class name (UTF-8, max 15 chars + null)
    pub class_name: [u8; 16],
}

impl Default for TrackedObject {
    fn default() -> Self {
        Self {
            bbox: BoundingBox2D::default(),
            predicted_bbox: BoundingBox2D::default(),
            track_id: 0,
            confidence: 0.0,
            class_id: 0,
            velocity_x: 0.0,
            velocity_y: 0.0,
            accel_x: 0.0,
            accel_y: 0.0,
            age: 0,
            hits: 0,
            time_since_update: 0,
            state: 0,
            class_name: [0u8; 16],
        }
    }
}

impl TrackedObject {
    /// Create a new tracked object from a detection
    pub fn new(track_id: u64, bbox: BoundingBox2D, class_id: u32, confidence: f32) -> Self {
        Self {
            bbox,
            predicted_bbox: bbox,
            track_id,
            confidence,
            class_id,
            hits: 1,
            state: 0, // tentative
            ..Default::default()
        }
    }

    /// Set class name
    pub fn set_class_name(&mut self, name: &str) {
        let bytes = name.as_bytes();
        let len = bytes.len().min(15);
        self.class_name[..len].copy_from_slice(&bytes[..len]);
        self.class_name[len..].fill(0);
    }

    /// Get class name
    pub fn get_class_name(&self) -> &str {
        let end = self.class_name.iter().position(|&b| b == 0).unwrap_or(16);
        std::str::from_utf8(&self.class_name[..end]).unwrap_or("")
    }

    /// Check if track is tentative (not yet confirmed)
    pub fn is_tentative(&self) -> bool {
        self.state == 0
    }

    /// Check if track is confirmed
    pub fn is_confirmed(&self) -> bool {
        self.state == 1
    }

    /// Check if track is deleted/lost
    pub fn is_deleted(&self) -> bool {
        self.state == 2
    }

    /// Confirm the track
    pub fn confirm(&mut self) {
        self.state = 1;
    }

    /// Mark track as deleted
    pub fn delete(&mut self) {
        self.state = 2;
    }

    /// Update with new detection
    pub fn update(&mut self, bbox: BoundingBox2D, confidence: f32) {
        // Calculate velocity from bbox movement
        self.velocity_x = bbox.center_x() - self.bbox.center_x();
        self.velocity_y = bbox.center_y() - self.bbox.center_y();

        self.bbox = bbox;
        self.confidence = confidence;
        self.hits += 1;
        self.time_since_update = 0;
        self.age += 1;
    }

    /// Mark frame with no detection
    pub fn mark_missed(&mut self) {
        self.time_since_update += 1;
        self.age += 1;

        // Predict next position based on velocity
        self.predicted_bbox = BoundingBox2D {
            x: self.bbox.x + self.velocity_x,
            y: self.bbox.y + self.velocity_y,
            width: self.bbox.width,
            height: self.bbox.height,
        };
    }

    /// Calculate speed magnitude
    pub fn speed(&self) -> f32 {
        (self.velocity_x * self.velocity_x + self.velocity_y * self.velocity_y).sqrt()
    }

    /// Get heading angle in radians
    pub fn heading(&self) -> f32 {
        self.velocity_y.atan2(self.velocity_x)
    }
}

/// Track state constants
pub mod state {
    pub const TENTATIVE: u32 = 0;
    pub const CONFIRMED: u32 = 1;
    pub const DELETED: u32 = 2;
}

/// Tracking array header
///
/// Header for an array of tracked objects.
///
/// Size: 32 bytes
#[repr(C)]
#[derive(Debug, Clone, Copy, Default, Pod, Zeroable)]
pub struct TrackingHeader {
    /// Number of tracked objects
    pub num_tracks: u32,
    /// Frame number
    pub frame_id: u32,
    /// Timestamp (nanoseconds since epoch)
    pub timestamp_ns: u64,
    /// Total tracks created (for ID generation)
    pub total_tracks: u64,
    /// Active confirmed tracks
    pub active_tracks: u32,
    /// Padding
    _pad: u32,
}

impl TrackingHeader {
    /// Create a new tracking header
    pub fn new(num_tracks: u32, frame_id: u32) -> Self {
        Self {
            num_tracks,
            frame_id,
            ..Default::default()
        }
    }

    /// Set timestamp
    pub fn with_timestamp(mut self, timestamp_ns: u64) -> Self {
        self.timestamp_ns = timestamp_ns;
        self
    }

    /// Calculate data size
    pub fn data_size(&self) -> usize {
        (self.num_tracks as usize) * std::mem::size_of::<TrackedObject>()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_tracked_object_size() {
        assert_eq!(std::mem::size_of::<TrackedObject>(), 96);
    }

    #[test]
    fn test_tracking_header_size() {
        assert_eq!(std::mem::size_of::<TrackingHeader>(), 32);
    }

    #[test]
    fn test_track_lifecycle() {
        let mut track =
            TrackedObject::new(1, BoundingBox2D::new(100.0, 100.0, 50.0, 50.0), 0, 0.95);

        assert!(track.is_tentative());

        track.confirm();
        assert!(track.is_confirmed());

        // Update with new detection
        track.update(BoundingBox2D::new(110.0, 105.0, 50.0, 50.0), 0.93);
        assert_eq!(track.hits, 2);
        assert!((track.velocity_x - 10.0).abs() < 0.01);

        // Miss a frame
        track.mark_missed();
        assert_eq!(track.time_since_update, 1);
        assert!((track.predicted_bbox.x - 120.0).abs() < 0.01);
    }

    #[test]
    fn test_speed_calculation() {
        let mut track = TrackedObject::default();
        track.velocity_x = 3.0;
        track.velocity_y = 4.0;
        assert!((track.speed() - 5.0).abs() < 0.001);
    }
}
