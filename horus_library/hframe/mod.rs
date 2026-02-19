//! # HFrame - High-Performance Transform System for HORUS
//!
//! HFrame is a hybrid, lock-free coordinate frame management system designed
//! to replace traditional TF implementations with better real-time performance.
//!
//! ## Key Features
//!
//! - **Lock-free reads**: Uses seqlock (version-dance) protocol for concurrent access
//! - **Configurable capacity**: Supports 256 to 65535 frames via compile-time or runtime config
//! - **Hybrid design**: Fast static frames + flexible dynamic frames
//! - **Time-travel queries**: Ring buffer history with SLERP interpolation
//! - **Dual interface**: Integer IDs for hot path, string names for user API
//! - **f64 precision**: Full robotics-grade precision (not game engine f32)
//!
//! ## Architecture
//!
//! ```text
//! ┌─────────────────────────────────────────────────────────────┐
//! │                     HFrame System                            │
//! ├─────────────────────────────────────────────────────────────┤
//! │  ┌──────────────────┐   ┌────────────────────────────────┐ │
//! │  │  FrameRegistry   │   │         HFrameCore             │ │
//! │  │  (String ↔ ID)   │   │   (Lock-free Transform Store)  │ │
//! │  │  - name_to_id    │   │   - slots: Vec<FrameSlot>      │ │
//! │  │  - id_to_name    │   │   - parents: Vec<FrameId>      │ │
//! │  └──────────────────┘   └────────────────────────────────┘ │
//! │                                                              │
//! │  ┌──────────────────────────────────────────────────────┐  │
//! │  │  User API: hf.tf("camera", "base_link")              │  │
//! │  │            hf.tf_at("lidar", "map", timestamp)        │  │
//! │  └──────────────────────────────────────────────────────┘  │
//! └─────────────────────────────────────────────────────────────┘
//! ```
//!
//! ## Usage
//!
//! ```rust,ignore
//! use horus_library::hframe::{HFrame, HFrameConfig};
//!
//! // Create with default config (256 frames, 32 history entries)
//! let hf = HFrame::new();
//!
//! // Or customize for large simulations
//! let hf = HFrame::with_config(HFrameConfig {
//!     max_frames: 1024,
//!     max_static_frames: 512,
//!     history_len: 64,
//!     ..Default::default()
//! });
//!
//! // Register frames
//! let world = hf.register_frame("world", None)?;
//! let base = hf.register_frame("base_link", Some("world"))?;
//! let camera = hf.register_frame("camera_frame", Some("base_link"))?;
//!
//! // Update transforms (lock-free writes)
//! hf.update_transform(camera, &transform, timestamp_ns);
//!
//! // Query transforms (lock-free reads)
//! let tf = hf.tf("camera_frame", "world")?;
//! let point_world = tf.transform_point([1.0, 0.0, 0.0]);
//!
//! // Time-travel query with interpolation
//! let tf_old = hf.tf_at("camera_frame", "world", past_timestamp)?;
//! ```
//!
//! ## Performance Comparison
//!
//! | Operation | HFrame | ROS2 TF2 |
//! |-----------|--------|----------|
//! | Lookup by ID | ~50ns | N/A |
//! | Lookup by name | ~200ns | ~2μs |
//! | Chain resolution (depth 3) | ~150ns | ~5μs |
//! | Transform update | ~100ns | ~1μs |
//! | Real-time safe | Yes | No |

#[cfg(test)]
mod bench;
mod config;
mod core;
mod messages;
mod registry;
mod slot;
mod transform;
mod types;

// Re-export public API
pub use config::HFrameConfig;
pub use core::HFrameCore;
pub use registry::FrameRegistry;
pub use slot::{FrameSlot, TransformEntry};
pub use types::{FrameId, HFrameError, HFrameResult, INVALID_FRAME, NO_PARENT};

// Re-export Transform and message types
pub use messages::{
    frame_id_to_string, string_to_frame_id, HFMessage, StaticTransformStamped, TFMessage,
    TransformStamped,
    FRAME_ID_SIZE, MAX_TRANSFORMS_PER_MESSAGE,
};
pub use transform::Transform;

use std::sync::Arc;

/// Main HFrame interface combining core storage and name registry
///
/// This is the primary type users interact with. It provides both
/// high-performance ID-based access and convenient string-based access.
#[derive(Clone)]
pub struct HFrame {
    /// Lock-free transform storage
    pub core: Arc<HFrameCore>,
    /// String ↔ ID mapping
    pub registry: Arc<FrameRegistry>,
    /// Configuration
    pub config: HFrameConfig,
}

impl HFrame {
    /// Create a new HFrame with default configuration
    ///
    /// Default: 256 max frames, 128 static + 128 dynamic, 32 history entries
    pub fn new() -> Self {
        Self::with_config(HFrameConfig::default())
    }

    /// Create with custom configuration
    pub fn with_config(config: HFrameConfig) -> Self {
        let core = Arc::new(HFrameCore::new(&config));
        let registry = Arc::new(FrameRegistry::new(core.clone(), config.max_frames));

        Self {
            core,
            registry,
            config,
        }
    }

    /// Create with preset for small robots (256 frames)
    pub fn small() -> Self {
        Self::with_config(HFrameConfig::small())
    }

    /// Create with preset for medium robots (1024 frames)
    pub fn medium() -> Self {
        Self::with_config(HFrameConfig::medium())
    }

    /// Create with preset for large simulations (4096 frames)
    pub fn large() -> Self {
        Self::with_config(HFrameConfig::large())
    }

    /// Create with preset for massive simulations (16384 frames)
    pub fn massive() -> Self {
        Self::with_config(HFrameConfig::massive())
    }

    // ========================================================================
    // Frame Registration
    // ========================================================================

    /// Register a new frame with a name
    ///
    /// # Arguments
    /// * `name` - Unique frame name (e.g., "base_link", "camera_frame")
    /// * `parent` - Optional parent frame name (None for root frames)
    ///
    /// # Returns
    /// * `Ok(FrameId)` - The assigned frame ID for fast lookups
    /// * `Err(HFrameError)` - If frame already exists or parent not found
    pub fn register_frame(&self, name: &str, parent: Option<&str>) -> HFrameResult<FrameId> {
        self.registry.register(name, parent)
    }

    /// Register a static frame (transform never changes)
    ///
    /// Static frames use less memory and have faster lookups.
    pub fn register_static_frame(
        &self,
        name: &str,
        parent: Option<&str>,
        transform: &Transform,
    ) -> HFrameResult<FrameId> {
        let id = self.registry.register_static(name, parent)?;
        self.core.set_static_transform(id, transform);
        Ok(id)
    }

    /// Unregister a dynamic frame
    ///
    /// Only dynamic frames can be unregistered. Static frames are permanent.
    pub fn unregister_frame(&self, name: &str) -> HFrameResult<()> {
        self.registry.unregister(name)
    }

    /// Get frame ID by name (cache this for hot paths!)
    pub fn frame_id(&self, name: &str) -> Option<FrameId> {
        self.registry.lookup(name)
    }

    /// Get frame name by ID
    pub fn frame_name(&self, id: FrameId) -> Option<String> {
        self.registry.lookup_name(id)
    }

    /// Check if a frame exists
    pub fn has_frame(&self, name: &str) -> bool {
        self.registry.lookup(name).is_some()
    }

    /// Get all registered frame names
    pub fn all_frames(&self) -> Vec<String> {
        self.registry.all_names()
    }

    /// Get number of registered frames
    pub fn frame_count(&self) -> usize {
        self.core.frame_count()
    }

    // ========================================================================
    // Transform Updates
    // ========================================================================

    /// Update a frame's transform (by ID - fastest)
    ///
    /// This is lock-free and safe to call from any thread.
    ///
    /// # Arguments
    /// * `frame_id` - The frame to update
    /// * `transform` - Transform from parent frame to this frame
    /// * `timestamp_ns` - Timestamp in nanoseconds
    pub fn update_transform_by_id(
        &self,
        frame_id: FrameId,
        transform: &Transform,
        timestamp_ns: u64,
    ) {
        self.core.update(frame_id, transform, timestamp_ns);
    }

    /// Update a frame's transform (by name)
    pub fn update_transform(
        &self,
        name: &str,
        transform: &Transform,
        timestamp_ns: u64,
    ) -> HFrameResult<()> {
        let id = self
            .registry
            .lookup(name)
            .ok_or_else(|| HFrameError::FrameNotFound(name.to_string()))?;
        self.core.update(id, transform, timestamp_ns);
        Ok(())
    }

    /// Set a static transform (for frames that never change)
    pub fn set_static_transform(&self, name: &str, transform: &Transform) -> HFrameResult<()> {
        let id = self
            .registry
            .lookup(name)
            .ok_or_else(|| HFrameError::FrameNotFound(name.to_string()))?;
        self.core.set_static_transform(id, transform);
        Ok(())
    }

    // ========================================================================
    // Transform Queries
    // ========================================================================

    /// Get latest transform between two frames (by name)
    ///
    /// Returns the transform that converts points from `src` frame to `dst` frame.
    ///
    /// # Example
    /// ```rust,ignore
    /// // Get transform from camera to base
    /// let tf = hf.tf("camera_frame", "base_link")?;
    /// let point_in_base = tf.transform_point(point_in_camera);
    /// ```
    pub fn tf(&self, src: &str, dst: &str) -> HFrameResult<Transform> {
        let src_id = self
            .registry
            .lookup(src)
            .ok_or_else(|| HFrameError::FrameNotFound(src.to_string()))?;
        let dst_id = self
            .registry
            .lookup(dst)
            .ok_or_else(|| HFrameError::FrameNotFound(dst.to_string()))?;

        self.core
            .resolve(src_id, dst_id)
            .ok_or(HFrameError::NoPath(src.to_string(), dst.to_string()))
    }

    /// Get transform at specific timestamp with interpolation (by name)
    ///
    /// If the exact timestamp isn't available, interpolates between
    /// the two nearest samples using linear interpolation for translation
    /// and SLERP for rotation.
    pub fn tf_at(&self, src: &str, dst: &str, timestamp_ns: u64) -> HFrameResult<Transform> {
        let src_id = self
            .registry
            .lookup(src)
            .ok_or_else(|| HFrameError::FrameNotFound(src.to_string()))?;
        let dst_id = self
            .registry
            .lookup(dst)
            .ok_or_else(|| HFrameError::FrameNotFound(dst.to_string()))?;

        self.core
            .resolve_at(src_id, dst_id, timestamp_ns)
            .ok_or(HFrameError::NoPath(src.to_string(), dst.to_string()))
    }

    /// Get latest transform between two frames (by ID - fastest)
    ///
    /// Use this in hot paths where you've cached the frame IDs.
    #[inline]
    pub fn tf_by_id(&self, src: FrameId, dst: FrameId) -> Option<Transform> {
        self.core.resolve(src, dst)
    }

    /// Get transform at timestamp (by ID - fastest)
    #[inline]
    pub fn tf_at_by_id(&self, src: FrameId, dst: FrameId, timestamp_ns: u64) -> Option<Transform> {
        self.core.resolve_at(src, dst, timestamp_ns)
    }

    /// Check if a transform path exists between two frames
    pub fn can_transform(&self, src: &str, dst: &str) -> bool {
        match (self.registry.lookup(src), self.registry.lookup(dst)) {
            (Some(src_id), Some(dst_id)) => self.core.can_transform(src_id, dst_id),
            _ => false,
        }
    }

    // ========================================================================
    // Convenience Methods
    // ========================================================================

    /// Transform a point from one frame to another
    pub fn transform_point(&self, src: &str, dst: &str, point: [f64; 3]) -> HFrameResult<[f64; 3]> {
        let tf = self.tf(src, dst)?;
        Ok(tf.transform_point(point))
    }

    /// Transform a vector from one frame to another (rotation only)
    pub fn transform_vector(
        &self,
        src: &str,
        dst: &str,
        vector: [f64; 3],
    ) -> HFrameResult<[f64; 3]> {
        let tf = self.tf(src, dst)?;
        Ok(tf.transform_vector(vector))
    }

    /// Get the parent frame of a given frame
    pub fn parent(&self, name: &str) -> Option<String> {
        let id = self.registry.lookup(name)?;
        let parent_id = self.core.parent(id)?;
        self.registry.lookup_name(parent_id)
    }

    /// Get all children of a frame
    pub fn children(&self, name: &str) -> Vec<String> {
        let Some(id) = self.registry.lookup(name) else {
            return Vec::new();
        };

        self.core
            .children(id)
            .iter()
            .filter_map(|&child_id| self.registry.lookup_name(child_id))
            .collect()
    }

    /// Get the frame chain from src to dst
    pub fn frame_chain(&self, src: &str, dst: &str) -> HFrameResult<Vec<String>> {
        let src_id = self
            .registry
            .lookup(src)
            .ok_or_else(|| HFrameError::FrameNotFound(src.to_string()))?;
        let dst_id = self
            .registry
            .lookup(dst)
            .ok_or_else(|| HFrameError::FrameNotFound(dst.to_string()))?;

        let chain = self
            .core
            .frame_chain(src_id, dst_id)
            .ok_or(HFrameError::NoPath(src.to_string(), dst.to_string()))?;

        Ok(chain
            .iter()
            .filter_map(|&id| self.registry.lookup_name(id))
            .collect())
    }

    // ========================================================================
    // Diagnostics
    // ========================================================================

    /// Get statistics about HFrame usage
    pub fn stats(&self) -> HFrameStats {
        HFrameStats {
            total_frames: self.core.frame_count(),
            static_frames: self.core.static_frame_count(),
            dynamic_frames: self.core.dynamic_frame_count(),
            max_frames: self.config.max_frames,
            history_len: self.config.history_len,
        }
    }

    /// Validate the frame tree structure
    pub fn validate(&self) -> HFrameResult<()> {
        self.core.validate()
    }
}

impl Default for HFrame {
    fn default() -> Self {
        Self::new()
    }
}

// Thread-safe: HFrame uses Arc internally and all operations are thread-safe
unsafe impl Send for HFrame {}
unsafe impl Sync for HFrame {}

/// Statistics about HFrame usage
#[derive(Debug, Clone)]
pub struct HFrameStats {
    pub total_frames: usize,
    pub static_frames: usize,
    pub dynamic_frames: usize,
    pub max_frames: usize,
    pub history_len: usize,
}

impl std::fmt::Display for HFrameStats {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "HFrame: {}/{} frames ({} static, {} dynamic), {} history entries",
            self.total_frames,
            self.max_frames,
            self.static_frames,
            self.dynamic_frames,
            self.history_len
        )
    }
}

/// Get current timestamp in nanoseconds
pub fn timestamp_now() -> u64 {
    std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map(|d| d.as_nanos() as u64)
        .unwrap_or(0)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_basic_usage() {
        let hf = HFrame::new();

        // Register frames
        let world = hf.register_frame("world", None).unwrap();
        let base = hf.register_frame("base_link", Some("world")).unwrap();
        let camera = hf.register_frame("camera", Some("base_link")).unwrap();

        assert_eq!(world, 0);
        assert_eq!(base, 1);
        assert_eq!(camera, 2);

        // Update transforms
        let tf_base = Transform::from_translation([1.0, 0.0, 0.0]);
        let tf_camera = Transform::from_translation([0.0, 0.0, 0.5]);

        hf.update_transform_by_id(base, &tf_base, 1000);
        hf.update_transform_by_id(camera, &tf_camera, 1000);

        // Query
        let tf = hf.tf("camera", "world").unwrap();
        assert!((tf.translation[0] - 1.0).abs() < 1e-10);
        assert!((tf.translation[2] - 0.5).abs() < 1e-10);
    }

    #[test]
    fn test_frame_lookup() {
        let hf = HFrame::new();
        hf.register_frame("world", None).unwrap();

        assert!(hf.has_frame("world"));
        assert!(!hf.has_frame("nonexistent"));

        assert_eq!(hf.frame_id("world"), Some(0));
        assert_eq!(hf.frame_name(0), Some("world".to_string()));
    }

    #[test]
    fn test_config_presets() {
        let small = HFrame::small();
        assert_eq!(small.config.max_frames, 256);

        let large = HFrame::large();
        assert_eq!(large.config.max_frames, 4096);
    }
}
