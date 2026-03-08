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
//! | Input validation (NaN/Inf) | Yes | No (silent corruption) |
//! | Extrapolation detection | Yes | Yes |
//! | Time tolerance queries | Yes | Yes |
//! | Staleness detection | Yes | Manual |
//! | Blocking wait (condvar) | `wait` feature | Built-in |
//! | Async wait (tokio) | `async-wait` feature | rclcpp only |
//! | Tree visualization (DOT/YAML) | Built-in | `tf2_tools` |
//! | Chain failure diagnostics | Built-in | Generic errors |
//!
//! ## TF2 Migration Guide
//!
//! ### Convention Mapping
//!
//! **Argument order is reversed**: HFrame uses `(source, destination)`,
//! TF2 uses `(target, source)`. Both return a transform that maps
//! points from source to destination.
//!
//! | TF2 (C++) | HFrame |
//! |-----------|--------|
//! | `lookupTransform("dst", "src")` | `hf.tf("src", "dst")` |
//! | `lookupTransform("dst", "src", time)` | `hf.tf_at("src", "dst", time_ns)` |
//! | `canTransform("dst", "src")` | `hf.can_transform("src", "dst")` |
//! | `canTransform("dst", "src", time)` | `hf.can_transform_at("src", "dst", time_ns)` |
//! | `sendTransform(msg)` | `hf.update_transform("child", &tf, ts)` |
//! | `StaticTransformBroadcaster` | `hf.register_static_frame("name", parent, &tf)` |
//! | `waitForTransform(...)` | `hf.wait_for_transform(src, dst, timeout)` (feature `wait`) |
//!
//! ### Builder API (new in TF2 parity)
//!
//! ```rust,ignore
//! // Register frames fluently
//! hf.add_frame("world").build()?;
//! hf.add_frame("base_link").parent("world")?;
//! hf.add_static("camera").parent("base_link")
//!     .transform(&Transform::xyz(0.1, 0.0, 0.5))?;
//!
//! // Query transforms fluently
//! let tf = hf.query("camera").to("world").lookup()?;
//! let pt = hf.query("lidar").to("map").point([1.0, 0.0, 0.0])?;
//! let ok = hf.query("sensor").to("world").can_at(timestamp);
//! ```
//!
//! ### Short Transform Constructors
//!
//! ```rust,ignore
//! Transform::xyz(1.0, 2.0, 3.0)       // translation only
//! Transform::yaw(PI / 4.0)             // rotation only
//! Transform::xyz(1.0, 0.0, 0.0).with_yaw(PI / 2.0)  // chainable
//! ```
//!
//! ### Feature Flags
//!
//! | Feature | Description | Dependency |
//! |---------|-------------|------------|
//! | (default) | Core transform system | None |
//! | `wait` | Blocking `wait_for_transform()` via condvar | None |
//! | `async-wait` | Async `wait_for_transform_async()` via tokio | `tokio` |
//!
//! ### Key Differences from TF2
//!
//! 1. **Lock-free**: All reads/writes are wait-free; no mutex contention
//! 2. **Input validation**: NaN/Inf transforms are rejected at write time
//! 3. **Extrapolation is explicit**: Use `tf_at_strict()` for TF2-style errors
//! 4. **No global singleton**: Each `HFrame` is independent and `Send + Sync`
//! 5. **Error types**: Uses `HorusError::Extrapolation`, `::NotFound`, `::InvalidInput`

#[cfg(test)]
mod bench;
mod builder;
mod config;
mod core;
mod messages;
mod query;
mod registry;
mod slot;
mod transform;
mod types;

// Re-export public API
pub use builder::{FrameBuilder, StaticFrameBuilder, StaticFrameBuilderWithParent};
pub use config::HFrameConfig;
pub use core::HFrameCore;
pub use query::{TransformQuery, TransformQueryFrom};
pub use registry::FrameRegistry;
pub use slot::{FrameSlot, TransformEntry};
pub use types::{FrameId, INVALID_FRAME, NO_PARENT};

// Re-export Transform and message types
pub use messages::{
    frame_id_to_string, string_to_frame_id, HFMessage, StaticTransformStamped, TFMessage,
    TransformStamped, FRAME_ID_SIZE, MAX_TRANSFORMS_PER_MESSAGE,
};
pub use transform::Transform;

use horus_core::error::HorusError;
use horus_core::HorusResult;
use std::sync::Arc;

/// Condvar-based notification for `wait_for_transform()`.
///
/// Only constructed when the `wait` feature is enabled. The condvar is
/// notified after every `update_transform*` and `register_frame` call,
/// waking any threads blocked in `wait_for_transform`.
#[cfg(feature = "wait")]
struct TransformNotifier {
    condvar: std::sync::Condvar,
    mutex: std::sync::Mutex<()>,
}

#[cfg(feature = "wait")]
impl TransformNotifier {
    fn new() -> Self {
        Self {
            condvar: std::sync::Condvar::new(),
            mutex: std::sync::Mutex::new(()),
        }
    }

    fn notify(&self) {
        // Wake all waiters — cheap no-op if nobody is waiting
        self.condvar.notify_all();
    }

    fn wait_timeout(&self, timeout: std::time::Duration) -> bool {
        let guard = self.mutex.lock().unwrap_or_else(|e| e.into_inner());
        let result = self.condvar.wait_timeout(guard, timeout);
        match result {
            Ok((_, timeout_result)) => !timeout_result.timed_out(),
            Err(_) => false, // Poisoned mutex — treat as timeout
        }
    }
}

/// Async notification for `wait_for_transform_async()`.
///
/// Only constructed when the `async-wait` feature is enabled. Uses
/// `tokio::sync::Notify` for zero-cost async wakeup.
#[cfg(feature = "async-wait")]
struct AsyncTransformNotifier {
    notify: tokio::sync::Notify,
}

#[cfg(feature = "async-wait")]
impl AsyncTransformNotifier {
    fn new() -> Self {
        Self {
            notify: tokio::sync::Notify::new(),
        }
    }

    fn notify(&self) {
        self.notify.notify_waiters();
    }

    async fn notified(&self) {
        self.notify.notified().await;
    }
}

/// Main HFrame interface combining core storage and name registry
///
/// This is the primary type users interact with. It provides both
/// high-performance ID-based access and convenient string-based access.
#[derive(Clone)]
pub struct HFrame {
    /// Lock-free transform storage
    core: Arc<HFrameCore>,
    /// String ↔ ID mapping
    registry: Arc<FrameRegistry>,
    /// Configuration
    config: HFrameConfig,
    /// Optional notification for wait_for_transform (only with `wait` feature)
    #[cfg(feature = "wait")]
    notifier: Arc<TransformNotifier>,
    /// Optional async notification for wait_for_transform_async (only with `async-wait` feature)
    #[cfg(feature = "async-wait")]
    async_notifier: Arc<AsyncTransformNotifier>,
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
            #[cfg(feature = "wait")]
            notifier: Arc::new(TransformNotifier::new()),
            #[cfg(feature = "async-wait")]
            async_notifier: Arc::new(AsyncTransformNotifier::new()),
        }
    }

    /// Access the underlying lock-free transform storage.
    ///
    /// For advanced use only — prefer the high-level `HFrame` methods.
    pub fn core(&self) -> &Arc<HFrameCore> {
        &self.core
    }

    /// Access the string ↔ ID name registry.
    ///
    /// For advanced use only — prefer the high-level `HFrame` methods.
    pub fn registry(&self) -> &Arc<FrameRegistry> {
        &self.registry
    }

    /// Get the configuration this HFrame was created with.
    pub fn config(&self) -> &HFrameConfig {
        &self.config
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
    /// * `Err(HorusError)` - If frame already exists or parent not found
    pub fn register_frame(&self, name: &str, parent: Option<&str>) -> HorusResult<FrameId> {
        let id = self.registry.register(name, parent)?;
        #[cfg(feature = "wait")]
        self.notifier.notify();
        #[cfg(feature = "async-wait")]
        self.async_notifier.notify();
        Ok(id)
    }

    /// Register a static frame (transform never changes)
    ///
    /// Static frames use less memory and have faster lookups.
    /// Validates the transform before storing.
    pub fn register_static_frame(
        &self,
        name: &str,
        parent: Option<&str>,
        transform: &Transform,
    ) -> HorusResult<FrameId> {
        let id = self.registry.register_static(name, parent)?;
        self.core.set_static_transform(id, transform)?;
        Ok(id)
    }

    /// Start building a dynamic frame registration.
    ///
    /// ```rust,ignore
    /// hf.add_frame("world").build()?;               // root frame
    /// hf.add_frame("base_link").parent("world")?;   // child frame
    /// ```
    #[inline]
    pub fn add_frame<'a>(&'a self, name: &'a str) -> FrameBuilder<'a> {
        FrameBuilder::new(self, name)
    }

    /// Start building a static frame registration.
    ///
    /// Static frames require a parent and a transform:
    /// ```rust,ignore
    /// hf.add_static("camera")
    ///     .parent("base_link")
    ///     .transform(&Transform::from_translation([0.1, 0.0, 0.5]))?;
    /// ```
    #[inline]
    pub fn add_static<'a>(&'a self, name: &'a str) -> StaticFrameBuilder<'a> {
        StaticFrameBuilder::new(self, name)
    }

    /// Unregister a dynamic frame
    ///
    /// Only dynamic frames can be unregistered. Static frames are permanent.
    pub fn unregister_frame(&self, name: &str) -> HorusResult<()> {
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
    // Query Builder
    // ========================================================================

    /// Start building a transform query from the given source frame.
    ///
    /// Returns a builder that you chain with `.to()` and then a lookup method:
    ///
    /// ```rust,ignore
    /// let tf = hf.query("camera").to("world").lookup()?;
    /// let pt = hf.query("lidar").to("base_link").point([1.0, 0.0, 0.0])?;
    /// let ok = hf.query("imu").to("world").can_at(timestamp);
    /// ```
    ///
    /// This is zero-overhead sugar — all methods inline to the same code
    /// as calling `hf.tf()`, `hf.tf_at()`, etc. directly.
    #[inline]
    pub fn query<'a>(&'a self, src: &'a str) -> TransformQueryFrom<'a> {
        TransformQueryFrom::new(self, src)
    }

    // ========================================================================
    // Transform Updates
    // ========================================================================

    /// Update a frame's transform (by ID - fastest)
    ///
    /// This is lock-free and safe to call from any thread.
    /// Validates the transform before storing: rejects NaN/Inf values and
    /// zero quaternions, auto-normalizes near-unit quaternions.
    ///
    /// # Arguments
    /// * `frame_id` - The frame to update
    /// * `transform` - Transform from parent frame to this frame
    /// * `timestamp_ns` - Timestamp in nanoseconds
    ///
    /// # Errors
    /// Returns `HorusError::InvalidInput` if the transform contains NaN/Inf
    /// or an invalid quaternion.
    pub fn update_transform_by_id(
        &self,
        frame_id: FrameId,
        transform: &Transform,
        timestamp_ns: u64,
    ) -> HorusResult<()> {
        self.core.update(frame_id, transform, timestamp_ns)?;
        #[cfg(feature = "wait")]
        self.notifier.notify();
        #[cfg(feature = "async-wait")]
        self.async_notifier.notify();
        Ok(())
    }

    /// Update a frame's transform (by name)
    ///
    /// Validates the transform before storing: rejects NaN/Inf values and
    /// zero quaternions, auto-normalizes near-unit quaternions.
    ///
    /// # Errors
    /// Returns `HorusError::NotFound` if the frame doesn't exist, or
    /// `HorusError::InvalidInput` if the transform is invalid.
    pub fn update_transform(
        &self,
        name: &str,
        transform: &Transform,
        timestamp_ns: u64,
    ) -> HorusResult<()> {
        let id = self
            .registry
            .lookup(name)
            .ok_or_else(|| HorusError::NotFound(format!("Frame '{}'", name)))?;
        self.core.update(id, transform, timestamp_ns)?;
        #[cfg(feature = "wait")]
        self.notifier.notify();
        #[cfg(feature = "async-wait")]
        self.async_notifier.notify();
        Ok(())
    }

    /// Set a static transform (for frames that never change)
    ///
    /// Validates the transform before storing.
    pub fn set_static_transform(&self, name: &str, transform: &Transform) -> HorusResult<()> {
        let id = self
            .registry
            .lookup(name)
            .ok_or_else(|| HorusError::NotFound(format!("Frame '{}'", name)))?;
        self.core.set_static_transform(id, transform)
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
    pub fn tf(&self, src: &str, dst: &str) -> HorusResult<Transform> {
        let src_id = self
            .registry
            .lookup(src)
            .ok_or_else(|| HorusError::NotFound(format!("Frame '{}' not registered", src)))?;
        let dst_id = self
            .registry
            .lookup(dst)
            .ok_or_else(|| HorusError::NotFound(format!("Frame '{}' not registered", dst)))?;

        self.core.resolve(src_id, dst_id).ok_or_else(|| {
            HorusError::Communication(
                self.diagnose_chain_failure_named(src, src_id, dst, dst_id)
                    .into(),
            )
        })
    }

    /// Get transform at specific timestamp with interpolation (by name)
    ///
    /// If the exact timestamp isn't available, interpolates between
    /// the two nearest samples using linear interpolation for translation
    /// and SLERP for rotation.
    pub fn tf_at(&self, src: &str, dst: &str, timestamp_ns: u64) -> HorusResult<Transform> {
        let src_id = self
            .registry
            .lookup(src)
            .ok_or_else(|| HorusError::NotFound(format!("Frame '{}' not registered", src)))?;
        let dst_id = self
            .registry
            .lookup(dst)
            .ok_or_else(|| HorusError::NotFound(format!("Frame '{}' not registered", dst)))?;

        self.core
            .resolve_at(src_id, dst_id, timestamp_ns)
            .ok_or_else(|| {
                HorusError::Communication(
                    self.diagnose_chain_failure_named(src, src_id, dst, dst_id)
                        .into(),
                )
            })
    }

    /// Get transform at specific timestamp with strict time-range checking.
    ///
    /// Unlike `tf_at()` which silently clamps to edge values when the
    /// requested timestamp falls outside the buffer window, this method
    /// returns `Err(HorusError::Extrapolation)` if any frame in the
    /// chain would need to extrapolate.
    ///
    /// Use this when you need TF2-style extrapolation detection:
    /// ```rust,ignore
    /// match hf.tf_at_strict("camera", "world", timestamp) {
    ///     Ok(tf) => use_transform(tf),
    ///     Err(HorusError::Extrapolation(msg)) => log::warn!("Stale data: {}", msg),
    ///     Err(e) => return Err(e),
    /// }
    /// ```
    pub fn tf_at_strict(&self, src: &str, dst: &str, timestamp_ns: u64) -> HorusResult<Transform> {
        let src_id = self
            .registry
            .lookup(src)
            .ok_or_else(|| HorusError::NotFound(format!("Frame '{}'", src)))?;
        let dst_id = self
            .registry
            .lookup(dst)
            .ok_or_else(|| HorusError::NotFound(format!("Frame '{}'", dst)))?;

        self.core.resolve_at_strict(src_id, dst_id, timestamp_ns)
    }

    /// Get transform at timestamp with time tolerance.
    ///
    /// Like `tf_at()` but returns `Err(HorusError::Extrapolation)` if the
    /// gap between the requested timestamp and the nearest buffered entry
    /// exceeds `tolerance_ns` for any frame in the chain.
    ///
    /// Use `tolerance_ns = u64::MAX` for unlimited tolerance (same as `tf_at()`).
    ///
    /// ```rust,ignore
    /// // Accept transforms within 50ms of the requested timestamp
    /// let tf = hf.tf_at_with_tolerance("camera", "world", ts, 50_000_000)?;
    /// ```
    pub fn tf_at_with_tolerance(
        &self,
        src: &str,
        dst: &str,
        timestamp_ns: u64,
        tolerance_ns: u64,
    ) -> HorusResult<Transform> {
        let src_id = self
            .registry
            .lookup(src)
            .ok_or_else(|| HorusError::NotFound(format!("Frame '{}'", src)))?;
        let dst_id = self
            .registry
            .lookup(dst)
            .ok_or_else(|| HorusError::NotFound(format!("Frame '{}'", dst)))?;

        self.core
            .resolve_at_with_tolerance(src_id, dst_id, timestamp_ns, tolerance_ns)
    }

    /// Get transform at timestamp with tolerance (by ID - fastest)
    pub fn tf_at_with_tolerance_by_id(
        &self,
        src: FrameId,
        dst: FrameId,
        timestamp_ns: u64,
        tolerance_ns: u64,
    ) -> HorusResult<Transform> {
        self.core
            .resolve_at_with_tolerance(src, dst, timestamp_ns, tolerance_ns)
    }

    /// Get transform at timestamp with strict checking (by ID - fastest)
    ///
    /// Returns `Err(HorusError::Extrapolation)` if outside the buffer window.
    pub fn tf_at_strict_by_id(
        &self,
        src: FrameId,
        dst: FrameId,
        timestamp_ns: u64,
    ) -> HorusResult<Transform> {
        self.core.resolve_at_strict(src, dst, timestamp_ns)
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

    /// Check if a transform can be resolved at a specific timestamp.
    ///
    /// Unlike `can_transform()` which only checks structural connectivity,
    /// this also verifies that all frames in the chain have data covering
    /// the requested timestamp (no extrapolation would occur).
    ///
    /// Equivalent to TF2's `canTransform(src, dst, time)`.
    pub fn can_transform_at(&self, src: &str, dst: &str, timestamp_ns: u64) -> bool {
        self.tf_at_strict(src, dst, timestamp_ns).is_ok()
    }

    /// Check if a transform can be resolved at a timestamp within tolerance.
    ///
    /// Returns `true` if all frames in the chain have data within
    /// `tolerance_ns` of the requested timestamp.
    pub fn can_transform_at_with_tolerance(
        &self,
        src: &str,
        dst: &str,
        timestamp_ns: u64,
        tolerance_ns: u64,
    ) -> bool {
        self.tf_at_with_tolerance(src, dst, timestamp_ns, tolerance_ns)
            .is_ok()
    }

    // ========================================================================
    // Convenience Methods
    // ========================================================================

    /// Transform a point from one frame to another
    pub fn transform_point(&self, src: &str, dst: &str, point: [f64; 3]) -> HorusResult<[f64; 3]> {
        let tf = self.tf(src, dst)?;
        Ok(tf.transform_point(point))
    }

    /// Transform a vector from one frame to another (rotation only)
    pub fn transform_vector(
        &self,
        src: &str,
        dst: &str,
        vector: [f64; 3],
    ) -> HorusResult<[f64; 3]> {
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
    pub fn frame_chain(&self, src: &str, dst: &str) -> HorusResult<Vec<String>> {
        let src_id = self
            .registry
            .lookup(src)
            .ok_or_else(|| HorusError::NotFound(format!("Frame '{}'", src)))?;
        let dst_id = self
            .registry
            .lookup(dst)
            .ok_or_else(|| HorusError::NotFound(format!("Frame '{}'", dst)))?;

        let chain = self
            .core
            .frame_chain(src_id, dst_id)
            .ok_or(HorusError::Communication(
                format!("No transform path between '{}' and '{}'", src, dst).into(),
            ))?;

        Ok(chain
            .iter()
            .filter_map(|&id| self.registry.lookup_name(id))
            .collect())
    }

    /// Get the time range of buffered transforms for a frame.
    ///
    /// Returns `Some((oldest_ns, newest_ns))` if the frame has data,
    /// or `None` if the frame has never been updated. Static frames
    /// return `Some((0, u64::MAX))`.
    pub fn time_range(&self, name: &str) -> Option<(u64, u64)> {
        let id = self.registry.lookup(name)?;
        self.core.time_range(id)
    }

    // ========================================================================
    // Transform Waiting (requires `wait` feature)
    // ========================================================================

    /// Block until a transform between `src` and `dst` becomes available,
    /// or the timeout expires.
    ///
    /// This is the HFrame equivalent of TF2's `waitForTransform`. Use it
    /// during node startup to wait for sensor drivers to publish their
    /// first transforms.
    ///
    /// Requires the `wait` feature flag to be enabled.
    ///
    /// ```rust,ignore
    /// // Wait up to 5 seconds for the camera transform
    /// let tf = hf.wait_for_transform("camera", "base_link", Duration::from_secs(5))?;
    /// ```
    #[cfg(feature = "wait")]
    pub fn wait_for_transform(
        &self,
        src: &str,
        dst: &str,
        timeout: std::time::Duration,
    ) -> HorusResult<Transform> {
        // Fast path: already available
        if let Ok(tf) = self.tf(src, dst) {
            return Ok(tf);
        }

        let deadline = std::time::Instant::now() + timeout;

        loop {
            let remaining = deadline.saturating_duration_since(std::time::Instant::now());
            if remaining.is_zero() {
                return Err(HorusError::Timeout(format!(
                    "Timed out waiting for transform '{}' -> '{}' after {:?}",
                    src, dst, timeout
                )));
            }

            // Wait for a notification (update or registration)
            self.notifier.wait_timeout(remaining);

            // Retry
            if let Ok(tf) = self.tf(src, dst) {
                return Ok(tf);
            }
        }
    }

    /// Block until a transform at a specific timestamp becomes available,
    /// or the timeout expires.
    ///
    /// Uses strict time-range checking — returns only when all frames in the
    /// chain have data covering the requested timestamp.
    ///
    /// Requires the `wait` feature flag to be enabled.
    #[cfg(feature = "wait")]
    pub fn wait_for_transform_at(
        &self,
        src: &str,
        dst: &str,
        timestamp_ns: u64,
        timeout: std::time::Duration,
    ) -> HorusResult<Transform> {
        // Fast path
        if let Ok(tf) = self.tf_at_strict(src, dst, timestamp_ns) {
            return Ok(tf);
        }

        let deadline = std::time::Instant::now() + timeout;

        loop {
            let remaining = deadline.saturating_duration_since(std::time::Instant::now());
            if remaining.is_zero() {
                return Err(HorusError::Timeout(format!(
                    "Timed out waiting for transform '{}' -> '{}' at ts={}ns after {:?}",
                    src, dst, timestamp_ns, timeout
                )));
            }

            self.notifier.wait_timeout(remaining);

            if let Ok(tf) = self.tf_at_strict(src, dst, timestamp_ns) {
                return Ok(tf);
            }
        }
    }

    // ========================================================================
    // Async Transform Waiting (feature = "async-wait")
    // ========================================================================

    /// Asynchronously wait until a transform between two frames becomes available,
    /// or the timeout expires.
    ///
    /// This is the async equivalent of `wait_for_transform()`. Use this in
    /// tokio-based applications to avoid blocking an executor thread.
    ///
    /// Requires the `async-wait` feature flag to be enabled.
    ///
    /// ```rust,ignore
    /// // In a tokio task:
    /// let tf = hf.wait_for_transform_async("camera", "base_link", Duration::from_secs(5)).await?;
    /// ```
    #[cfg(feature = "async-wait")]
    pub async fn wait_for_transform_async(
        &self,
        src: &str,
        dst: &str,
        timeout: std::time::Duration,
    ) -> HorusResult<Transform> {
        // Fast path: already available
        if let Ok(tf) = self.tf(src, dst) {
            return Ok(tf);
        }

        let sleep = tokio::time::sleep(timeout);
        tokio::pin!(sleep);

        loop {
            tokio::select! {
                _ = &mut sleep => {
                    return Err(HorusError::Timeout(format!(
                        "Timed out waiting for transform '{}' -> '{}' after {:?}",
                        src, dst, timeout
                    )));
                }
                _ = self.async_notifier.notified() => {
                    if let Ok(tf) = self.tf(src, dst) {
                        return Ok(tf);
                    }
                }
            }
        }
    }

    /// Asynchronously wait until a transform at a specific timestamp becomes
    /// available, or the timeout expires.
    ///
    /// Uses strict time-range checking — resolves only when all frames in the
    /// chain have data covering the requested timestamp.
    ///
    /// Requires the `async-wait` feature flag to be enabled.
    #[cfg(feature = "async-wait")]
    pub async fn wait_for_transform_at_async(
        &self,
        src: &str,
        dst: &str,
        timestamp_ns: u64,
        timeout: std::time::Duration,
    ) -> HorusResult<Transform> {
        // Fast path
        if let Ok(tf) = self.tf_at_strict(src, dst, timestamp_ns) {
            return Ok(tf);
        }

        let sleep = tokio::time::sleep(timeout);
        tokio::pin!(sleep);

        loop {
            tokio::select! {
                _ = &mut sleep => {
                    return Err(HorusError::Timeout(format!(
                        "Timed out waiting for transform '{}' -> '{}' at ts={}ns after {:?}",
                        src, dst, timestamp_ns, timeout
                    )));
                }
                _ = self.async_notifier.notified() => {
                    if let Ok(tf) = self.tf_at_strict(src, dst, timestamp_ns) {
                        return Ok(tf);
                    }
                }
            }
        }
    }

    // ========================================================================
    // Staleness Detection
    // ========================================================================

    /// Check if a frame's transform data is stale.
    ///
    /// Returns `true` if the most recent transform for `name` is older than
    /// `max_age_ns` nanoseconds before `now_ns`. Static frames are never stale.
    ///
    /// The `now_ns` parameter allows simulation users to pass simulated time
    /// instead of wall-clock time (critical for Gazebo/Isaac Sim compatibility).
    ///
    /// Returns `true` if the frame has no data (never updated).
    ///
    /// ```rust,ignore
    /// // Real robot: use timestamp_now()
    /// if hf.is_stale("imu", 500_000_000, timestamp_now()) {
    ///     log::warn!("IMU data is >0.5s old!");
    /// }
    ///
    /// // Simulation: use sim time
    /// if hf.is_stale("imu", 500_000_000, sim_time_ns) {
    ///     log::warn!("IMU data is stale in sim time");
    /// }
    /// ```
    pub fn is_stale(&self, name: &str, max_age_ns: u64, now_ns: u64) -> bool {
        let Some(id) = self.registry.lookup(name) else {
            return true; // Unknown frame is considered stale
        };
        match self.core.time_range(id) {
            Some((_, newest)) if newest == u64::MAX => false, // Static frame
            Some((_, newest)) => now_ns.saturating_sub(newest) > max_age_ns,
            None => true, // Never updated = stale
        }
    }

    /// Convenience: check staleness against wall-clock time.
    ///
    /// Equivalent to `is_stale(name, max_age_ns, timestamp_now())`.
    /// Do NOT use this in simulation — pass sim time to `is_stale()` instead.
    pub fn is_stale_now(&self, name: &str, max_age_ns: u64) -> bool {
        self.is_stale(name, max_age_ns, timestamp_now())
    }

    /// Get nanoseconds since the most recent transform update for a frame.
    ///
    /// The `now_ns` parameter allows simulation users to pass simulated time.
    /// Returns `None` if the frame has never been updated or doesn't exist.
    /// Static frames return `Some(0)` (always fresh).
    pub fn time_since_last_update(&self, name: &str, now_ns: u64) -> Option<u64> {
        let id = self.registry.lookup(name)?;
        let (_, newest) = self.core.time_range(id)?;
        if newest == u64::MAX {
            Some(0) // Static frame: always fresh
        } else {
            Some(now_ns.saturating_sub(newest))
        }
    }

    /// Convenience: get time since last update using wall-clock time.
    ///
    /// Do NOT use this in simulation — pass sim time to
    /// `time_since_last_update()` instead.
    pub fn time_since_last_update_now(&self, name: &str) -> Option<u64> {
        self.time_since_last_update(name, timestamp_now())
    }

    // ========================================================================
    // Diagnostics
    // ========================================================================

    /// Get statistics about HFrame usage
    pub fn stats(&self) -> HFrameStats {
        let names = self.all_frames();
        let mut root_count = 0;
        let mut max_depth = 0;

        for name in &names {
            if let Some(id) = self.registry.lookup(name) {
                if self.core.parent(id).is_none() {
                    root_count += 1;
                }
                let depth = self.core.path_to_root_ids(id).len().saturating_sub(1);
                if depth > max_depth {
                    max_depth = depth;
                }
            }
        }

        HFrameStats {
            total_frames: self.core.frame_count(),
            static_frames: self.core.static_frame_count(),
            dynamic_frames: self.core.dynamic_frame_count(),
            max_frames: self.config.max_frames,
            history_len: self.config.history_len,
            tree_depth: max_depth,
            root_count,
        }
    }

    /// Get metadata for a single frame.
    ///
    /// Returns `None` if the frame is not registered.
    pub fn frame_info(&self, name: &str) -> Option<FrameInfo> {
        let id = self.registry.lookup(name)?;
        let parent = self
            .core
            .parent(id)
            .and_then(|pid| self.registry.lookup_name(pid));
        let is_static = self.core.is_static(id);
        let time_range = self.core.time_range(id).and_then(|(oldest, newest)| {
            if newest == u64::MAX || (oldest == 0 && newest == 0) {
                None // Static frames or no data
            } else {
                Some((oldest, newest))
            }
        });
        let children_count = self.core.children(id).len();
        let depth = self.core.path_to_root_ids(id).len().saturating_sub(1);

        Some(FrameInfo {
            name: name.to_string(),
            id,
            parent,
            is_static,
            time_range,
            children_count,
            depth,
        })
    }

    /// Get metadata for all registered frames.
    pub fn frame_info_all(&self) -> Vec<FrameInfo> {
        self.all_frames()
            .iter()
            .filter_map(|name| self.frame_info(name))
            .collect()
    }

    /// Validate the frame tree structure
    pub fn validate(&self) -> HorusResult<()> {
        self.core.validate()
    }

    /// Produce a detailed diagnostic message for chain resolution failures,
    /// using frame names instead of IDs for user-friendly output.
    ///
    /// Only called on error paths — never on successful lookups.
    fn diagnose_chain_failure_named(
        &self,
        src_name: &str,
        src_id: FrameId,
        dst_name: &str,
        dst_id: FrameId,
    ) -> String {
        // Helper to resolve an ID to a name, falling back to the ID
        let name_of = |id: FrameId| -> String {
            self.registry
                .lookup_name(id)
                .unwrap_or_else(|| format!("#{}", id))
        };

        let src_path: Vec<String> = self
            .core
            .path_to_root_ids(src_id)
            .iter()
            .map(|&id| name_of(id))
            .collect();
        let dst_path: Vec<String> = self
            .core
            .path_to_root_ids(dst_id)
            .iter()
            .map(|&id| name_of(id))
            .collect();

        if src_path.is_empty() {
            return format!("Frame '{}' is not initialized", src_name);
        }
        if dst_path.is_empty() {
            return format!("Frame '{}' is not initialized", dst_name);
        }

        let src_root = src_path.last().unwrap();
        let dst_root = dst_path.last().unwrap();

        if src_root != dst_root {
            return format!(
                "Frames '{}' and '{}' are in disconnected trees \
                 (root '{}' vs root '{}'). \
                 Chain: {} → ... → {}, {} → ... → {}",
                src_name, dst_name, src_root, dst_root, src_name, src_root, dst_name, dst_root,
            );
        }

        // Check for frames with no data
        let src_path_ids = self.core.path_to_root_ids(src_id);
        let dst_path_ids = self.core.path_to_root_ids(dst_id);
        let mut issues = Vec::new();
        for &fid in src_path_ids.iter().chain(dst_path_ids.iter()) {
            if self.core.is_static(fid) {
                continue;
            }
            if self.core.read_latest(fid).is_none() {
                issues.push(format!("  '{}': no transform data published", name_of(fid)));
            }
        }

        if issues.is_empty() {
            format!(
                "No transform path between '{}' and '{}' \
                 (both share root '{}', topology may have changed concurrently)",
                src_name, dst_name, src_root
            )
        } else {
            format!(
                "No transform path between '{}' and '{}':\n{}",
                src_name,
                dst_name,
                issues.join("\n")
            )
        }
    }

    // ========================================================================
    // Tree Export
    // ========================================================================

    /// Export the frame tree as a Graphviz DOT string.
    ///
    /// Produces a directed graph where each frame is a node and parent
    /// relationships are edges. Static frames are drawn with a double border,
    /// dynamic frames as plain boxes. Edge labels show the last-known
    /// translation magnitude.
    ///
    /// Paste the output into <https://graphviz.org> for instant visualization.
    ///
    /// ```rust,ignore
    /// println!("{}", hf.frames_as_dot());
    /// ```
    pub fn frames_as_dot(&self) -> String {
        let mut dot = String::from("digraph hframe {\n");
        dot.push_str("  rankdir=TB;\n");
        dot.push_str("  node [shape=box, fontname=\"monospace\"];\n\n");

        let names = self.all_frames();
        for name in &names {
            let Some(id) = self.registry.lookup(name) else {
                continue;
            };
            let is_static = self.core.is_static(id);
            let time_info = match self.core.time_range(id) {
                Some((oldest, newest)) if newest == u64::MAX => "static".to_string(),
                Some((oldest, newest)) => format!("t=[{}..{}]ns", oldest, newest),
                None => "no data".to_string(),
            };

            let shape = if is_static { "doubleoctagon" } else { "box" };
            let label = format!("{}\\n{}", name, time_info);
            dot.push_str(&format!(
                "  \"{}\" [label=\"{}\", shape={}];\n",
                name, label, shape
            ));

            if let Some(parent_id) = self.core.parent(id) {
                if let Some(parent_name) = self.registry.lookup_name(parent_id) {
                    // Edge label: translation magnitude if available
                    let edge_label = self
                        .core
                        .read_latest(id)
                        .map(|entry| {
                            let t = entry.transform.translation;
                            let mag = (t[0] * t[0] + t[1] * t[1] + t[2] * t[2]).sqrt();
                            format!("{:.3}m", mag)
                        })
                        .unwrap_or_default();

                    if edge_label.is_empty() {
                        dot.push_str(&format!("  \"{}\" -> \"{}\";\n", parent_name, name));
                    } else {
                        dot.push_str(&format!(
                            "  \"{}\" -> \"{}\" [label=\"{}\"];\n",
                            parent_name, name, edge_label
                        ));
                    }
                }
            }
        }

        dot.push_str("}\n");
        dot
    }

    /// Export the frame tree as a YAML string.
    ///
    /// Produces a YAML document compatible with TF2's `allFramesAsYAML()`
    /// output format. Each frame entry includes parent, type, last update
    /// timestamp, and buffer time range.
    ///
    /// ```rust,ignore
    /// println!("{}", hf.frames_as_yaml());
    /// ```
    pub fn frames_as_yaml(&self) -> String {
        let mut yaml = String::from("# HFrame tree export\n");
        let names = self.all_frames();

        for name in &names {
            let Some(id) = self.registry.lookup(name) else {
                continue;
            };
            let is_static = self.core.is_static(id);
            let parent_name = self
                .core
                .parent(id)
                .and_then(|pid| self.registry.lookup_name(pid));

            yaml.push_str(&format!("{}:\n", name));
            yaml.push_str(&format!(
                "  parent: {}\n",
                parent_name.as_deref().unwrap_or("(root)")
            ));
            yaml.push_str(&format!(
                "  type: {}\n",
                if is_static { "static" } else { "dynamic" }
            ));

            match self.core.time_range(id) {
                Some((oldest, newest)) if newest == u64::MAX => {
                    yaml.push_str("  last_update: static\n");
                    yaml.push_str("  buffer_range: [0, inf]\n");
                }
                Some((oldest, newest)) => {
                    yaml.push_str(&format!("  last_update: {}ns\n", newest));
                    yaml.push_str(&format!("  buffer_range: [{}ns, {}ns]\n", oldest, newest));
                }
                None => {
                    yaml.push_str("  last_update: never\n");
                    yaml.push_str("  buffer_range: []\n");
                }
            }
        }

        yaml
    }

    /// Pretty-print the frame tree to stderr for quick debugging.
    ///
    /// Shows a tree-like hierarchy with indentation, similar to the
    /// `tree` command. Each frame shows its type and latest timestamp.
    ///
    /// ```rust,ignore
    /// hf.print_tree(); // Prints to stderr
    /// ```
    pub fn print_tree(&self) {
        eprintln!("{}", self.format_tree());
    }

    /// Format the frame tree as a human-readable string.
    ///
    /// Returns the same output as `print_tree()` but as a `String` instead
    /// of printing to stderr.
    pub fn format_tree(&self) -> String {
        let mut output = String::from("HFrame Tree:\n");

        // Find root frames (no parent)
        let names = self.all_frames();
        let mut roots = Vec::new();
        for name in &names {
            if let Some(id) = self.registry.lookup(name) {
                if self.core.parent(id).is_none() {
                    roots.push(name.clone());
                }
            }
        }

        for root in &roots {
            self.format_tree_recursive(root, &mut output, "", true, true);
        }

        output
    }

    fn format_tree_recursive(
        &self,
        name: &str,
        output: &mut String,
        prefix: &str,
        is_last: bool,
        is_root: bool,
    ) {
        let connector = if is_root {
            ""
        } else if is_last {
            "└── "
        } else {
            "├── "
        };

        let id = self.registry.lookup(name);
        let type_tag = match id {
            Some(id) if self.core.is_static(id) => "[S]",
            Some(_) => "[D]",
            None => "[?]",
        };

        let time_tag = id
            .and_then(|id| self.core.time_range(id))
            .map(|(_, newest)| {
                if newest == u64::MAX {
                    "static".to_string()
                } else {
                    format!("t={}ns", newest)
                }
            })
            .unwrap_or_else(|| "no data".to_string());

        output.push_str(&format!(
            "{}{}{} {} ({})\n",
            prefix, connector, name, type_tag, time_tag
        ));

        let children = self.children(name);
        let child_prefix = if is_root {
            String::new()
        } else if is_last {
            format!("{}    ", prefix)
        } else {
            format!("{}│   ", prefix)
        };

        for (i, child) in children.iter().enumerate() {
            let child_is_last = i == children.len() - 1;
            self.format_tree_recursive(child, output, &child_prefix, child_is_last, false);
        }
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
    /// Maximum depth of the frame tree
    pub tree_depth: usize,
    /// Number of root frames (frames with no parent)
    pub root_count: usize,
}

impl std::fmt::Display for HFrameStats {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "HFrame: {}/{} frames ({} static, {} dynamic), {} history entries, \
             depth {}, {} root(s)",
            self.total_frames,
            self.max_frames,
            self.static_frames,
            self.dynamic_frames,
            self.history_len,
            self.tree_depth,
            self.root_count,
        )
    }
}

/// Per-frame metadata for monitoring and debugging
#[derive(Debug, Clone)]
pub struct FrameInfo {
    /// Frame name
    pub name: String,
    /// Frame ID (for hot-path access)
    pub id: FrameId,
    /// Parent frame name, or None for root frames
    pub parent: Option<String>,
    /// Whether this is a static (never-changing) frame
    pub is_static: bool,
    /// Time range of buffered transforms `(oldest_ns, newest_ns)`,
    /// or None if no data has been published
    pub time_range: Option<(u64, u64)>,
    /// Number of direct children
    pub children_count: usize,
    /// Depth in the frame tree (root = 0)
    pub depth: usize,
}

/// Get current timestamp in nanoseconds.
///
/// # Panics
///
/// Panics if system time is before the UNIX epoch (should never happen
/// on any sane system).
pub fn timestamp_now() -> u64 {
    std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .expect("system clock before UNIX epoch")
        .as_nanos() as u64
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

        hf.update_transform_by_id(base, &tf_base, 1000).unwrap();
        hf.update_transform_by_id(camera, &tf_camera, 1000).unwrap();

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

    #[test]
    fn test_stats_tree_depth_and_root_count() {
        let hf = HFrame::new();
        hf.register_frame("world", None).unwrap();
        hf.register_frame("base", Some("world")).unwrap();
        hf.register_frame("arm", Some("base")).unwrap();
        hf.register_frame("gripper", Some("arm")).unwrap();
        // Second tree
        hf.register_frame("map", None).unwrap();

        let stats = hf.stats();
        assert_eq!(stats.total_frames, 5);
        assert_eq!(stats.root_count, 2); // world and map
        assert_eq!(stats.tree_depth, 3); // world -> base -> arm -> gripper
    }

    #[test]
    fn test_frame_info_dynamic() {
        let hf = HFrame::new();
        hf.register_frame("world", None).unwrap();
        hf.register_frame("sensor", Some("world")).unwrap();
        hf.update_transform("sensor", &Transform::identity(), 5000)
            .unwrap();

        let info = hf.frame_info("sensor").unwrap();
        assert_eq!(info.name, "sensor");
        assert_eq!(info.parent, Some("world".to_string()));
        assert!(!info.is_static);
        assert_eq!(info.time_range, Some((5000, 5000)));
        assert_eq!(info.children_count, 0);
        assert_eq!(info.depth, 1);
    }

    #[test]
    fn test_frame_info_static() {
        let hf = HFrame::new();
        hf.register_frame("world", None).unwrap();
        hf.register_static_frame("fixed", Some("world"), &Transform::identity())
            .unwrap();

        let info = hf.frame_info("fixed").unwrap();
        assert!(info.is_static);
        assert_eq!(info.time_range, None); // Static frames have no time range
        assert_eq!(info.depth, 1);
    }

    #[test]
    fn test_frame_info_all() {
        let hf = HFrame::new();
        hf.register_frame("world", None).unwrap();
        hf.register_frame("a", Some("world")).unwrap();
        hf.register_frame("b", Some("world")).unwrap();

        let all = hf.frame_info_all();
        assert_eq!(all.len(), 3);
    }

    #[test]
    fn test_frame_info_nonexistent() {
        let hf = HFrame::new();
        assert!(hf.frame_info("ghost").is_none());
    }

    // =====================================================================
    // Query Builder Tests
    // =====================================================================

    #[test]
    fn test_query_lookup() {
        let hf = HFrame::new();
        hf.register_frame("world", None).unwrap();
        hf.register_frame("cam", Some("world")).unwrap();
        hf.update_transform("cam", &Transform::from_translation([1.0, 2.0, 3.0]), 1000)
            .unwrap();

        // Builder should produce same result as tf()
        let tf_direct = hf.tf("cam", "world").unwrap();
        let tf_query = hf.query("cam").to("world").lookup().unwrap();
        assert_eq!(tf_direct.translation, tf_query.translation);
        assert_eq!(tf_direct.rotation, tf_query.rotation);
    }

    #[test]
    fn test_query_at() {
        let hf = HFrame::new();
        hf.register_frame("world", None).unwrap();
        hf.register_frame("a", Some("world")).unwrap();
        hf.update_transform("a", &Transform::from_translation([1.0, 0.0, 0.0]), 1000)
            .unwrap();
        hf.update_transform("a", &Transform::from_translation([3.0, 0.0, 0.0]), 3000)
            .unwrap();

        let tf = hf.query("a").to("world").at(2000).unwrap();
        assert!((tf.translation[0] - 2.0).abs() < 1e-6);
    }

    #[test]
    fn test_query_point() {
        let hf = HFrame::new();
        hf.register_frame("world", None).unwrap();
        hf.register_frame("a", Some("world")).unwrap();
        hf.update_transform("a", &Transform::from_translation([10.0, 0.0, 0.0]), 1000)
            .unwrap();

        let pt = hf.query("a").to("world").point([1.0, 0.0, 0.0]).unwrap();
        assert!((pt[0] - 11.0).abs() < 1e-10);
    }

    #[test]
    fn test_query_can_at() {
        let hf = HFrame::new();
        hf.register_frame("world", None).unwrap();
        hf.register_frame("a", Some("world")).unwrap();
        hf.update_transform("a", &Transform::from_translation([1.0, 0.0, 0.0]), 1000)
            .unwrap();
        hf.update_transform("a", &Transform::from_translation([2.0, 0.0, 0.0]), 5000)
            .unwrap();

        assert!(hf.query("a").to("world").can_at(3000));
        assert!(!hf.query("a").to("world").can_at(99999));
    }

    #[test]
    fn test_query_chain() {
        let hf = HFrame::new();
        hf.register_frame("world", None).unwrap();
        hf.register_frame("a", Some("world")).unwrap();
        hf.register_frame("b", Some("a")).unwrap();

        let chain = hf.query("b").to("world").chain().unwrap();
        assert!(chain.len() >= 2);
    }

    // =====================================================================
    // Frame Builder Tests
    // =====================================================================

    #[test]
    fn test_add_frame_root() {
        let hf = HFrame::new();
        let id = hf.add_frame("world").build().unwrap();
        assert_eq!(id, 0);
        assert!(hf.has_frame("world"));
    }

    #[test]
    fn test_add_frame_with_parent() {
        let hf = HFrame::new();
        hf.add_frame("world").build().unwrap();
        let id = hf.add_frame("base").parent("world").unwrap();
        assert_eq!(id, 1);
        assert_eq!(hf.parent("base"), Some("world".to_string()));
    }

    #[test]
    fn test_add_static_frame() {
        let hf = HFrame::new();
        hf.add_frame("world").build().unwrap();
        let id = hf
            .add_static("camera")
            .parent("world")
            .transform(&Transform::from_translation([0.1, 0.0, 0.5]))
            .unwrap();

        assert!(hf.has_frame("camera"));
        let info = hf.frame_info("camera").unwrap();
        assert!(info.is_static);

        // Verify the transform is set
        let tf = hf.tf("camera", "world").unwrap();
        assert!((tf.translation[0] - 0.1).abs() < 1e-10);
        assert!((tf.translation[2] - 0.5).abs() < 1e-10);

        assert!(id > 0);
    }

    #[test]
    fn test_add_frame_equivalence() {
        // Builder should produce identical results to register_frame
        let hf1 = HFrame::new();
        hf1.register_frame("world", None).unwrap();
        hf1.register_frame("arm", Some("world")).unwrap();

        let hf2 = HFrame::new();
        hf2.add_frame("world").build().unwrap();
        hf2.add_frame("arm").parent("world").unwrap();

        assert_eq!(hf1.all_frames().len(), hf2.all_frames().len());
        assert_eq!(hf1.parent("arm"), hf2.parent("arm"));
    }

    #[test]
    fn test_tf_at_strict_in_range_ok() {
        let hf = HFrame::new();
        hf.register_frame("world", None).unwrap();
        hf.register_frame("a", Some("world")).unwrap();

        hf.update_transform("a", &Transform::from_translation([1.0, 0.0, 0.0]), 1000)
            .unwrap();
        hf.update_transform("a", &Transform::from_translation([2.0, 0.0, 0.0]), 2000)
            .unwrap();

        // Query within range should succeed
        let tf = hf.tf_at_strict("a", "world", 1500).unwrap();
        assert!((tf.translation[0] - 1.5).abs() < 1e-6);
    }

    #[test]
    fn test_tf_at_strict_extrapolation_past() {
        let hf = HFrame::new();
        hf.register_frame("world", None).unwrap();
        hf.register_frame("a", Some("world")).unwrap();

        hf.update_transform("a", &Transform::from_translation([1.0, 0.0, 0.0]), 5000)
            .unwrap();

        // Query before oldest buffered timestamp
        let result = hf.tf_at_strict("a", "world", 1000);
        assert!(result.is_err());
        let err = result.unwrap_err();
        assert!(
            matches!(err, HorusError::Extrapolation(_)),
            "Expected Extrapolation, got: {:?}",
            err
        );
    }

    #[test]
    fn test_tf_at_strict_extrapolation_future() {
        let hf = HFrame::new();
        hf.register_frame("world", None).unwrap();
        hf.register_frame("a", Some("world")).unwrap();

        hf.update_transform("a", &Transform::from_translation([1.0, 0.0, 0.0]), 1000)
            .unwrap();

        // Query after newest buffered timestamp
        let result = hf.tf_at_strict("a", "world", 99999);
        assert!(result.is_err());
        let err = result.unwrap_err();
        assert!(
            matches!(err, HorusError::Extrapolation(_)),
            "Expected Extrapolation, got: {:?}",
            err
        );
    }

    #[test]
    fn test_tf_at_strict_static_always_ok() {
        let hf = HFrame::new();
        hf.register_frame("world", None).unwrap();
        hf.register_static_frame(
            "fixed",
            Some("world"),
            &Transform::from_translation([1.0, 0.0, 0.0]),
        )
        .unwrap();

        // Static frames should never extrapolate
        let tf = hf.tf_at_strict("fixed", "world", 0).unwrap();
        assert!((tf.translation[0] - 1.0).abs() < 1e-10);
        let tf = hf.tf_at_strict("fixed", "world", u64::MAX).unwrap();
        assert!((tf.translation[0] - 1.0).abs() < 1e-10);
    }

    #[test]
    fn test_tf_at_strict_chain_any_hop_extrapolates() {
        let hf = HFrame::new();
        hf.register_frame("world", None).unwrap();
        hf.register_frame("a", Some("world")).unwrap();
        hf.register_frame("b", Some("a")).unwrap();

        // "a" has data at 1000-2000, "b" has data at 1000-5000
        hf.update_transform("a", &Transform::from_translation([1.0, 0.0, 0.0]), 1000)
            .unwrap();
        hf.update_transform("a", &Transform::from_translation([2.0, 0.0, 0.0]), 2000)
            .unwrap();
        hf.update_transform("b", &Transform::from_translation([0.5, 0.0, 0.0]), 1000)
            .unwrap();
        hf.update_transform("b", &Transform::from_translation([1.5, 0.0, 0.0]), 5000)
            .unwrap();

        // ts=3000 is within b's range but outside a's range → Extrapolation
        let result = hf.tf_at_strict("b", "world", 3000);
        assert!(
            matches!(result, Err(HorusError::Extrapolation(_))),
            "Expected Extrapolation because frame 'a' can't reach ts=3000, got: {:?}",
            result
        );
    }

    #[test]
    fn test_time_range() {
        let hf = HFrame::new();
        hf.register_frame("world", None).unwrap();
        hf.register_frame("a", Some("world")).unwrap();

        // No data yet
        assert!(hf.time_range("a").is_none());

        hf.update_transform("a", &Transform::from_translation([1.0, 0.0, 0.0]), 1000)
            .unwrap();
        hf.update_transform("a", &Transform::from_translation([2.0, 0.0, 0.0]), 5000)
            .unwrap();

        let (oldest, newest) = hf.time_range("a").unwrap();
        assert_eq!(oldest, 1000);
        assert_eq!(newest, 5000);
    }

    #[test]
    fn test_update_transform_rejects_nan() {
        let hf = HFrame::new();
        hf.register_frame("world", None).unwrap();
        hf.register_frame("a", Some("world")).unwrap();

        let bad_tf = Transform {
            translation: [f64::NAN, 0.0, 0.0],
            rotation: [0.0, 0.0, 0.0, 1.0],
        };
        let result = hf.update_transform("a", &bad_tf, 1000);
        assert!(
            matches!(result, Err(HorusError::InvalidInput(_))),
            "Expected InvalidInput, got: {:?}",
            result
        );
    }

    #[test]
    fn test_update_transform_by_id_rejects_inf() {
        let hf = HFrame::new();
        hf.register_frame("world", None).unwrap();
        let a = hf.register_frame("a", Some("world")).unwrap();

        let bad_tf = Transform {
            translation: [0.0, 0.0, 0.0],
            rotation: [0.0, 0.0, f64::INFINITY, 1.0],
        };
        let result = hf.update_transform_by_id(a, &bad_tf, 1000);
        assert!(
            matches!(result, Err(HorusError::InvalidInput(_))),
            "Expected InvalidInput, got: {:?}",
            result
        );
    }

    // =====================================================================
    // Staleness API Tests
    // =====================================================================

    #[test]
    fn test_is_stale_fresh_data() {
        let hf = HFrame::new();
        hf.register_frame("world", None).unwrap();
        hf.register_frame("a", Some("world")).unwrap();

        hf.update_transform("a", &Transform::identity(), 10_000)
            .unwrap();

        // Data at ts=10000, now=10500, max_age=1000 → not stale
        assert!(!hf.is_stale("a", 1000, 10_500));
    }

    #[test]
    fn test_is_stale_old_data() {
        let hf = HFrame::new();
        hf.register_frame("world", None).unwrap();
        hf.register_frame("a", Some("world")).unwrap();

        hf.update_transform("a", &Transform::identity(), 10_000)
            .unwrap();

        // Data at ts=10000, now=20000, max_age=5000 → stale (10000 > 5000)
        assert!(hf.is_stale("a", 5000, 20_000));
    }

    #[test]
    fn test_is_stale_never_updated() {
        let hf = HFrame::new();
        hf.register_frame("world", None).unwrap();
        hf.register_frame("a", Some("world")).unwrap();

        // Never updated → always stale
        assert!(hf.is_stale("a", 1000, 10_000));
    }

    #[test]
    fn test_is_stale_unknown_frame() {
        let hf = HFrame::new();
        // Unknown frame → stale
        assert!(hf.is_stale("nonexistent", 1000, 10_000));
    }

    #[test]
    fn test_is_stale_static_frame_never_stale() {
        let hf = HFrame::new();
        hf.register_frame("world", None).unwrap();
        hf.register_static_frame("fixed", Some("world"), &Transform::identity())
            .unwrap();

        // Static frames are never stale regardless of time
        assert!(!hf.is_stale("fixed", 0, u64::MAX));
    }

    #[test]
    fn test_time_since_last_update_basic() {
        let hf = HFrame::new();
        hf.register_frame("world", None).unwrap();
        hf.register_frame("a", Some("world")).unwrap();

        hf.update_transform("a", &Transform::identity(), 10_000)
            .unwrap();

        let age = hf.time_since_last_update("a", 15_000).unwrap();
        assert_eq!(age, 5000);
    }

    #[test]
    fn test_time_since_last_update_never_updated() {
        let hf = HFrame::new();
        hf.register_frame("world", None).unwrap();
        hf.register_frame("a", Some("world")).unwrap();

        assert!(hf.time_since_last_update("a", 10_000).is_none());
    }

    #[test]
    fn test_time_since_last_update_static_always_zero() {
        let hf = HFrame::new();
        hf.register_frame("world", None).unwrap();
        hf.register_static_frame("fixed", Some("world"), &Transform::identity())
            .unwrap();

        let age = hf.time_since_last_update("fixed", 999_999).unwrap();
        assert_eq!(age, 0);
    }

    // =====================================================================
    // Time Tolerance Tests
    // =====================================================================

    #[test]
    fn test_tf_at_with_tolerance_within() {
        let hf = HFrame::new();
        hf.register_frame("world", None).unwrap();
        hf.register_frame("a", Some("world")).unwrap();

        hf.update_transform("a", &Transform::from_translation([1.0, 0.0, 0.0]), 1000)
            .unwrap();

        // Query at ts=1500, data at 1000, gap=500, tolerance=1000 → ok
        let result = hf.tf_at_with_tolerance("a", "world", 1500, 1000);
        assert!(result.is_ok());
    }

    #[test]
    fn test_tf_at_with_tolerance_exceeded() {
        let hf = HFrame::new();
        hf.register_frame("world", None).unwrap();
        hf.register_frame("a", Some("world")).unwrap();

        hf.update_transform("a", &Transform::from_translation([1.0, 0.0, 0.0]), 1000)
            .unwrap();

        // Query at ts=5000, data at 1000, gap=4000, tolerance=1000 → Extrapolation
        let result = hf.tf_at_with_tolerance("a", "world", 5000, 1000);
        assert!(
            matches!(result, Err(HorusError::Extrapolation(_))),
            "Expected Extrapolation, got: {:?}",
            result
        );
    }

    #[test]
    fn test_tf_at_with_tolerance_max_is_unlimited() {
        let hf = HFrame::new();
        hf.register_frame("world", None).unwrap();
        hf.register_frame("a", Some("world")).unwrap();

        hf.update_transform("a", &Transform::from_translation([1.0, 0.0, 0.0]), 1000)
            .unwrap();

        // u64::MAX tolerance = no limit (same as tf_at)
        let result = hf.tf_at_with_tolerance("a", "world", 999_999, u64::MAX);
        assert!(result.is_ok());
    }

    #[test]
    fn test_tf_at_with_tolerance_past_direction() {
        let hf = HFrame::new();
        hf.register_frame("world", None).unwrap();
        hf.register_frame("a", Some("world")).unwrap();

        hf.update_transform("a", &Transform::from_translation([1.0, 0.0, 0.0]), 5000)
            .unwrap();

        // Query at ts=1000, data starts at 5000, gap=4000, tolerance=1000
        let result = hf.tf_at_with_tolerance("a", "world", 1000, 1000);
        assert!(matches!(result, Err(HorusError::Extrapolation(_))));

        // Same but tolerance=5000 → ok
        let result = hf.tf_at_with_tolerance("a", "world", 1000, 5000);
        assert!(result.is_ok());
    }

    // =====================================================================
    // can_transform_at Tests
    // =====================================================================

    #[test]
    fn test_can_transform_at_in_range() {
        let hf = HFrame::new();
        hf.register_frame("world", None).unwrap();
        hf.register_frame("a", Some("world")).unwrap();
        hf.update_transform("a", &Transform::from_translation([1.0, 0.0, 0.0]), 1000)
            .unwrap();
        hf.update_transform("a", &Transform::from_translation([2.0, 0.0, 0.0]), 5000)
            .unwrap();

        assert!(hf.can_transform_at("a", "world", 3000));
    }

    #[test]
    fn test_can_transform_at_out_of_range() {
        let hf = HFrame::new();
        hf.register_frame("world", None).unwrap();
        hf.register_frame("a", Some("world")).unwrap();
        hf.update_transform("a", &Transform::from_translation([1.0, 0.0, 0.0]), 1000)
            .unwrap();

        assert!(!hf.can_transform_at("a", "world", 99999));
    }

    #[test]
    fn test_can_transform_at_no_path() {
        let hf = HFrame::new();
        hf.register_frame("world", None).unwrap();
        hf.register_frame("isolated", None).unwrap();
        hf.update_transform("isolated", &Transform::identity(), 1000)
            .unwrap();

        assert!(!hf.can_transform_at("isolated", "world", 1000));
    }

    #[test]
    fn test_can_transform_at_with_tolerance_ok() {
        let hf = HFrame::new();
        hf.register_frame("world", None).unwrap();
        hf.register_frame("a", Some("world")).unwrap();
        hf.update_transform("a", &Transform::from_translation([1.0, 0.0, 0.0]), 1000)
            .unwrap();

        // ts=2000, data at 1000, gap=1000, tolerance=2000 → ok
        assert!(hf.can_transform_at_with_tolerance("a", "world", 2000, 2000));
        // ts=5000, data at 1000, gap=4000, tolerance=2000 → false
        assert!(!hf.can_transform_at_with_tolerance("a", "world", 5000, 2000));
    }

    #[test]
    fn test_is_stale_exact_boundary() {
        let hf = HFrame::new();
        hf.register_frame("world", None).unwrap();
        hf.register_frame("a", Some("world")).unwrap();

        hf.update_transform("a", &Transform::identity(), 10_000)
            .unwrap();

        // Exactly at boundary (age == max_age) → not stale (uses >)
        assert!(!hf.is_stale("a", 5000, 15_000));
        // One nanosecond past → stale
        assert!(hf.is_stale("a", 5000, 15_001));
    }

    // =====================================================================
    // Tree Export Tests
    // =====================================================================

    #[test]
    fn test_frames_as_dot_basic() {
        let hf = HFrame::new();
        hf.register_frame("world", None).unwrap();
        hf.register_frame("base_link", Some("world")).unwrap();
        hf.register_static_frame(
            "camera",
            Some("base_link"),
            &Transform::from_translation([0.0, 0.0, 0.5]),
        )
        .unwrap();
        hf.update_transform(
            "base_link",
            &Transform::from_translation([1.0, 0.0, 0.0]),
            1000,
        )
        .unwrap();

        let dot = hf.frames_as_dot();
        assert!(dot.starts_with("digraph hframe {"));
        assert!(dot.contains("world"));
        assert!(dot.contains("base_link"));
        assert!(dot.contains("camera"));
        assert!(dot.contains("doubleoctagon")); // static frame
        assert!(dot.ends_with("}\n"));
    }

    #[test]
    fn test_frames_as_yaml_basic() {
        let hf = HFrame::new();
        hf.register_frame("world", None).unwrap();
        hf.register_frame("arm", Some("world")).unwrap();
        hf.update_transform("arm", &Transform::identity(), 5000)
            .unwrap();

        let yaml = hf.frames_as_yaml();
        assert!(yaml.contains("world:"));
        assert!(yaml.contains("arm:"));
        assert!(yaml.contains("parent: (root)"));
        assert!(yaml.contains("parent: world"));
        assert!(yaml.contains("type: dynamic"));
        assert!(yaml.contains("last_update: 5000ns"));
    }

    #[test]
    fn test_format_tree_basic() {
        let hf = HFrame::new();
        hf.register_frame("world", None).unwrap();
        hf.register_frame("base", Some("world")).unwrap();
        hf.register_frame("left_arm", Some("base")).unwrap();
        hf.register_frame("right_arm", Some("base")).unwrap();
        hf.update_transform("base", &Transform::identity(), 1000)
            .unwrap();

        let tree = hf.format_tree();
        assert!(tree.contains("world"));
        assert!(tree.contains("base"));
        assert!(tree.contains("left_arm"));
        assert!(tree.contains("right_arm"));
        assert!(tree.contains("[D]")); // dynamic tag
        assert!(tree.contains("├── ") || tree.contains("└── ")); // tree connectors
    }

    #[test]
    fn test_frames_as_yaml_static_frame() {
        let hf = HFrame::new();
        hf.register_frame("world", None).unwrap();
        hf.register_static_frame("fixed", Some("world"), &Transform::identity())
            .unwrap();

        let yaml = hf.frames_as_yaml();
        assert!(yaml.contains("type: static"));
        assert!(yaml.contains("last_update: static"));
    }

    // =====================================================================
    // Chain Failure Diagnostics Tests
    // =====================================================================

    #[test]
    fn test_tf_error_frame_not_registered() {
        let hf = HFrame::new();
        hf.register_frame("world", None).unwrap();

        let err = hf.tf("nonexistent", "world").unwrap_err();
        match err {
            HorusError::NotFound(msg) => {
                assert!(
                    msg.contains("nonexistent"),
                    "Error should name the missing frame: {}",
                    msg
                );
                assert!(
                    msg.contains("not registered"),
                    "Error should say 'not registered': {}",
                    msg
                );
            }
            other => unreachable!("Expected NotFound, got: {:?}", other),
        }
    }

    #[test]
    fn test_tf_error_disconnected_trees() {
        let hf = HFrame::new();
        hf.register_frame("world", None).unwrap();
        hf.register_frame("map", None).unwrap();
        hf.register_frame("robot", Some("world")).unwrap();
        hf.register_frame("landmark", Some("map")).unwrap();

        hf.update_transform("robot", &Transform::identity(), 1000)
            .unwrap();
        hf.update_transform("landmark", &Transform::identity(), 1000)
            .unwrap();

        let err = hf.tf("robot", "landmark").unwrap_err();
        match err {
            HorusError::Communication(ref e) => {
                let msg = e.to_string();
                assert!(
                    msg.contains("disconnected"),
                    "Error should mention 'disconnected': {}",
                    msg
                );
            }
            other => unreachable!("Expected Communication, got: {:?}", other),
        }
    }

    #[test]
    fn test_tf_error_no_data_published() {
        let hf = HFrame::new();
        hf.register_frame("world", None).unwrap();
        hf.register_frame("sensor", Some("world")).unwrap();
        // sensor registered but no transform published

        // tf() still resolves because the chain exists even without data
        // (resolve_at returns identity for slots with no data)
        // The diagnostic only fires when resolve() returns None,
        // which happens when there's no path (disconnected trees).
        // This test verifies that NotFound messages are clear.
        let err = hf.tf("sensor", "ghost").unwrap_err();
        match err {
            HorusError::NotFound(msg) => {
                assert!(
                    msg.contains("ghost"),
                    "Error should name the missing frame: {}",
                    msg
                );
            }
            other => unreachable!("Expected NotFound, got: {:?}", other),
        }
    }

    // =====================================================================
    // wait_for_transform Tests (feature = "wait")
    // =====================================================================

    #[cfg(feature = "wait")]
    #[test]
    fn test_wait_for_transform_already_available() {
        // If the transform is already available, wait returns immediately
        let hf = HFrame::new();
        hf.register_frame("world", None).unwrap();
        hf.register_frame("a", Some("world")).unwrap();
        hf.update_transform("a", &Transform::from_translation([1.0, 0.0, 0.0]), 1000)
            .unwrap();

        let tf = hf
            .wait_for_transform("a", "world", std::time::Duration::from_secs(1))
            .unwrap();
        assert!((tf.translation[0] - 1.0).abs() < 1e-10);
    }

    #[cfg(feature = "wait")]
    #[test]
    fn test_wait_for_transform_timeout() {
        // If the transform never arrives, wait should return Timeout error
        let hf = HFrame::new();
        hf.register_frame("world", None).unwrap();
        // Frame "a" is registered but never has a path to "world" via parents
        hf.register_frame("a", None).unwrap();

        let result = hf.wait_for_transform("a", "world", std::time::Duration::from_millis(50));
        assert!(
            matches!(result, Err(HorusError::Timeout(_))),
            "Expected Timeout, got: {:?}",
            result
        );
    }

    #[cfg(feature = "wait")]
    #[test]
    fn test_wait_for_transform_wakes_on_update() {
        use std::sync::Arc;
        use std::thread;
        use std::time::Duration;

        let hf = Arc::new(HFrame::new());
        hf.register_frame("world", None).unwrap();
        hf.register_frame("sensor", Some("world")).unwrap();

        // Spawn a thread that waits for the transform
        let hf_waiter = hf.clone();
        let handle = thread::spawn(move || {
            hf_waiter
                .wait_for_transform("sensor", "world", Duration::from_secs(5))
                .unwrap()
        });

        // Give the waiter a moment to enter the wait state
        thread::sleep(Duration::from_millis(20));

        // Publish the transform — this should wake the waiter
        hf.update_transform(
            "sensor",
            &Transform::from_translation([3.0, 0.0, 0.0]),
            1000,
        )
        .unwrap();

        // Waiter should return quickly with the transform
        let tf = handle.join().unwrap();
        assert!((tf.translation[0] - 3.0).abs() < 1e-10);
    }

    #[cfg(feature = "wait")]
    #[test]
    fn test_wait_for_transform_wakes_on_register() {
        use std::sync::Arc;
        use std::thread;
        use std::time::Duration;

        let hf = Arc::new(HFrame::new());
        hf.register_frame("world", None).unwrap();

        // Spawn a thread that waits — frame "sensor" doesn't exist yet
        let hf_waiter = hf.clone();
        let handle = thread::spawn(move || {
            hf_waiter.wait_for_transform("sensor", "world", Duration::from_secs(5))
        });

        thread::sleep(Duration::from_millis(20));

        // Register the frame and publish a transform
        hf.register_frame("sensor", Some("world")).unwrap();
        hf.update_transform(
            "sensor",
            &Transform::from_translation([2.0, 0.0, 0.0]),
            1000,
        )
        .unwrap();

        let result = handle.join().unwrap();
        assert!(result.is_ok());
        assert!((result.unwrap().translation[0] - 2.0).abs() < 1e-10);
    }

    #[cfg(feature = "wait")]
    #[test]
    fn test_wait_for_transform_at_wakes_when_timestamp_covered() {
        use std::sync::Arc;
        use std::thread;
        use std::time::Duration;

        let hf = Arc::new(HFrame::new());
        hf.register_frame("world", None).unwrap();
        hf.register_frame("a", Some("world")).unwrap();

        // Only have data at ts=1000 — strict query for ts=1500 needs [1000,1500] coverage
        hf.update_transform("a", &Transform::from_translation([1.0, 0.0, 0.0]), 1000)
            .unwrap();

        let hf_waiter = hf.clone();
        let handle = thread::spawn(move || {
            hf_waiter
                .wait_for_transform_at("a", "world", 1500, Duration::from_secs(5))
                .unwrap()
        });

        thread::sleep(Duration::from_millis(20));

        // Add data at ts=2000 so ts=1500 is within [1000, 2000]
        hf.update_transform("a", &Transform::from_translation([2.0, 0.0, 0.0]), 2000)
            .unwrap();

        let tf = handle.join().unwrap();
        // Interpolation between [1.0,0,0]@1000 and [2.0,0,0]@2000 at ts=1500 → 1.5
        assert!((tf.translation[0] - 1.5).abs() < 1e-6);
    }

    #[cfg(feature = "wait")]
    #[test]
    fn test_wait_for_transform_at_timeout() {
        let hf = HFrame::new();
        hf.register_frame("world", None).unwrap();
        hf.register_frame("a", Some("world")).unwrap();

        // Only have data at ts=1000, querying ts=5000 strict
        hf.update_transform("a", &Transform::from_translation([1.0, 0.0, 0.0]), 1000)
            .unwrap();

        let result =
            hf.wait_for_transform_at("a", "world", 5000, std::time::Duration::from_millis(50));
        assert!(
            matches!(result, Err(HorusError::Timeout(_))),
            "Expected Timeout, got: {:?}",
            result
        );
    }

    // =====================================================================
    // Async wait_for_transform Tests (feature = "async-wait")
    // =====================================================================

    #[cfg(feature = "async-wait")]
    #[tokio::test]
    async fn test_async_wait_already_available() {
        let hf = HFrame::new();
        hf.register_frame("world", None).unwrap();
        hf.register_frame("a", Some("world")).unwrap();
        hf.update_transform("a", &Transform::from_translation([1.0, 0.0, 0.0]), 1000)
            .unwrap();

        let tf = hf
            .wait_for_transform_async("a", "world", std::time::Duration::from_secs(1))
            .await
            .unwrap();
        assert!((tf.translation[0] - 1.0).abs() < 1e-10);
    }

    #[cfg(feature = "async-wait")]
    #[tokio::test]
    async fn test_async_wait_timeout() {
        let hf = HFrame::new();
        hf.register_frame("world", None).unwrap();
        hf.register_frame("a", None).unwrap(); // No path to world

        let result = hf
            .wait_for_transform_async("a", "world", std::time::Duration::from_millis(50))
            .await;
        assert!(
            matches!(result, Err(HorusError::Timeout(_))),
            "Expected Timeout, got: {:?}",
            result
        );
    }

    #[cfg(feature = "async-wait")]
    #[tokio::test]
    async fn test_async_wait_wakes_on_update() {
        use std::sync::Arc;
        use std::time::Duration;

        let hf = Arc::new(HFrame::new());
        hf.register_frame("world", None).unwrap();
        hf.register_frame("sensor", Some("world")).unwrap();

        let hf_waiter = hf.clone();
        let waiter = tokio::spawn(async move {
            hf_waiter
                .wait_for_transform_async("sensor", "world", Duration::from_secs(5))
                .await
                .unwrap()
        });

        // Give the waiter time to enter the wait state
        tokio::time::sleep(Duration::from_millis(20)).await;

        // Publish transform — should wake the async waiter
        hf.update_transform(
            "sensor",
            &Transform::from_translation([4.0, 0.0, 0.0]),
            1000,
        )
        .unwrap();

        let tf = waiter.await.unwrap();
        assert!((tf.translation[0] - 4.0).abs() < 1e-10);
    }

    #[cfg(feature = "async-wait")]
    #[tokio::test]
    async fn test_async_wait_at_wakes_when_timestamp_covered() {
        use std::sync::Arc;
        use std::time::Duration;

        let hf = Arc::new(HFrame::new());
        hf.register_frame("world", None).unwrap();
        hf.register_frame("a", Some("world")).unwrap();

        // Data at ts=1000 only — strict query for ts=1500 needs coverage
        hf.update_transform("a", &Transform::from_translation([1.0, 0.0, 0.0]), 1000)
            .unwrap();

        let hf_waiter = hf.clone();
        let waiter = tokio::spawn(async move {
            hf_waiter
                .wait_for_transform_at_async("a", "world", 1500, Duration::from_secs(5))
                .await
                .unwrap()
        });

        tokio::time::sleep(Duration::from_millis(20)).await;

        // Add data at ts=2000 to cover ts=1500
        hf.update_transform("a", &Transform::from_translation([2.0, 0.0, 0.0]), 2000)
            .unwrap();

        let tf = waiter.await.unwrap();
        assert!((tf.translation[0] - 1.5).abs() < 1e-6);
    }

    #[cfg(feature = "async-wait")]
    #[tokio::test]
    async fn test_async_wait_at_timeout() {
        let hf = HFrame::new();
        hf.register_frame("world", None).unwrap();
        hf.register_frame("a", Some("world")).unwrap();

        hf.update_transform("a", &Transform::from_translation([1.0, 0.0, 0.0]), 1000)
            .unwrap();

        let result = hf
            .wait_for_transform_at_async("a", "world", 5000, std::time::Duration::from_millis(50))
            .await;
        assert!(
            matches!(result, Err(HorusError::Timeout(_))),
            "Expected Timeout, got: {:?}",
            result
        );
    }
}
