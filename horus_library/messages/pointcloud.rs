//! Point cloud element types for zero-copy IPC
//!
//! Pod/Zeroable fixed-size point types for 3D point cloud data from LiDAR,
//! depth cameras, etc. These are suitable for shared memory transport.
//!
//! Canonical definitions live in `horus_types::point`. This module re-exports
//! them for backwards compatibility.

pub use horus_types::{PointXYZ, PointXYZI, PointXYZRGB};
