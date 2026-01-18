//! HORUS Perception Types
//!
//! This crate provides AI-friendly, zero-copy perception types for robotics.
//! All types implement `Pod` and `Zeroable` for efficient IPC via shared memory.
//!
//! # Types
//!
//! - [`Detection`] - 2D object detection (bounding box + class + confidence)
//! - [`Detection3D`] - 3D object detection (3D bounding box + class + confidence)
//! - [`PointXYZ`] - Basic 3D point
//! - [`PointXYZRGB`] - 3D point with color
//! - [`PointXYZI`] - 3D point with intensity (LiDAR)
//! - [`Landmark`] - 2D keypoint/landmark
//! - [`Landmark3D`] - 3D keypoint/landmark
//! - [`SegmentationMask`] - Per-pixel segmentation header
//! - [`TrackedObject`] - Object with tracking ID
//!
//! # Example
//!
//! ```rust,ignore
//! use horus_perception::{Detection, DetectionArray};
//! use horus::memory::TensorPool;
//!
//! // Allocate detection array via tensor pool
//! let pool = TensorPool::new(1, Default::default())?;
//! let detections = DetectionArray::alloc(&pool, 100)?;
//!
//! // Write detections
//! detections.push(Detection::new("person", 0.95, 100, 200, 50, 120));
//! detections.push(Detection::new("car", 0.87, 300, 150, 200, 100));
//!
//! // Send via HORUS topic (zero-copy)
//! topic.publish(detections.as_bytes());
//! ```

pub mod types;

pub use types::detection::{Detection, Detection3D, BoundingBox2D, BoundingBox3D};
pub use types::pointcloud::{PointXYZ, PointXYZRGB, PointXYZI, PointCloudHeader};
pub use types::landmark::{Landmark, Landmark3D, LandmarkArray};
pub use types::segmentation::SegmentationMask;
pub use types::tracking::TrackedObject;
