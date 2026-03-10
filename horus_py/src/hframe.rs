//! Python bindings for HFrame - High-Performance Transform System
//!
//! Provides Python access to HFrame's lock-free transform management system.

use crate::errors::to_py_err;
use horus_library::hframe::{timestamp_now, HFrame, HFrameConfig, Transform};
use pyo3::prelude::*;

/// Python wrapper for Transform
#[pyclass(name = "Transform")]
#[derive(Clone)]
pub struct PyTransform {
    inner: Transform,
}

#[pymethods]
impl PyTransform {
    /// Create a new transform from translation and quaternion
    ///
    /// Args:
    ///     translation: [x, y, z] translation in meters
    ///     rotation: [x, y, z, w] quaternion (default: identity [0, 0, 0, 1])
    #[new]
    #[pyo3(signature = (translation=None, rotation=None))]
    fn new(translation: Option<[f64; 3]>, rotation: Option<[f64; 4]>) -> Self {
        let translation = translation.unwrap_or([0.0, 0.0, 0.0]);
        let rotation = rotation.unwrap_or([0.0, 0.0, 0.0, 1.0]);
        PyTransform {
            inner: Transform::new(translation, rotation),
        }
    }

    /// Create identity transform (no translation or rotation)
    #[staticmethod]
    fn identity() -> Self {
        PyTransform {
            inner: Transform::identity(),
        }
    }

    /// Create transform from translation only
    #[staticmethod]
    fn from_translation(translation: [f64; 3]) -> Self {
        PyTransform {
            inner: Transform::from_translation(translation),
        }
    }

    /// Create transform from Euler angles (roll, pitch, yaw) in radians
    #[staticmethod]
    fn from_euler(translation: [f64; 3], rpy: [f64; 3]) -> Self {
        PyTransform {
            inner: Transform::from_euler(translation, rpy),
        }
    }

    /// Get translation [x, y, z]
    #[getter]
    fn translation(&self) -> [f64; 3] {
        self.inner.translation
    }

    /// Set translation [x, y, z]
    #[setter]
    fn set_translation(&mut self, value: [f64; 3]) {
        self.inner.translation = value;
    }

    /// Get rotation quaternion [x, y, z, w]
    #[getter]
    fn rotation(&self) -> [f64; 4] {
        self.inner.rotation
    }

    /// Set rotation quaternion [x, y, z, w]
    #[setter]
    fn set_rotation(&mut self, value: [f64; 4]) {
        self.inner.rotation = value;
    }

    /// Convert rotation to Euler angles (roll, pitch, yaw) in radians
    fn to_euler(&self) -> [f64; 3] {
        self.inner.to_euler()
    }

    /// Compose this transform with another (self * other)
    fn compose(&self, other: &PyTransform) -> PyTransform {
        PyTransform {
            inner: self.inner.compose(&other.inner),
        }
    }

    /// Get the inverse of this transform
    fn inverse(&self) -> PyTransform {
        PyTransform {
            inner: self.inner.inverse(),
        }
    }

    /// Transform a 3D point
    fn transform_point(&self, point: [f64; 3]) -> [f64; 3] {
        self.inner.transform_point(point)
    }

    /// Transform a 3D vector (rotation only, no translation)
    fn transform_vector(&self, vector: [f64; 3]) -> [f64; 3] {
        self.inner.transform_vector(vector)
    }

    /// Linear interpolation between two transforms (SLERP for rotation)
    fn interpolate(&self, other: &PyTransform, t: f64) -> PyTransform {
        PyTransform {
            inner: self.inner.interpolate(&other.inner, t),
        }
    }

    /// Get the translation magnitude (distance)
    fn translation_magnitude(&self) -> f64 {
        self.inner.translation_magnitude()
    }

    /// Get the rotation angle in radians
    fn rotation_angle(&self) -> f64 {
        self.inner.rotation_angle()
    }

    /// Convert to 4x4 homogeneous transformation matrix (row-major)
    fn to_matrix(&self) -> [[f64; 4]; 4] {
        self.inner.to_matrix()
    }

    /// Create transform from 4x4 homogeneous matrix
    #[staticmethod]
    fn from_matrix(matrix: [[f64; 4]; 4]) -> Self {
        PyTransform {
            inner: Transform::from_matrix(matrix),
        }
    }

    fn __repr__(&self) -> String {
        format!(
            "Transform(translation={:?}, rotation={:?})",
            self.inner.translation, self.inner.rotation
        )
    }

    fn __str__(&self) -> String {
        let euler = self.inner.to_euler();
        format!(
            "Transform(xyz=[{:.3}, {:.3}, {:.3}], rpy=[{:.3}, {:.3}, {:.3}])",
            self.inner.translation[0],
            self.inner.translation[1],
            self.inner.translation[2],
            euler[0],
            euler[1],
            euler[2]
        )
    }
}

/// Python wrapper for HFrameConfig
#[pyclass(name = "HFrameConfig")]
#[derive(Clone)]
pub struct PyHFrameConfig {
    inner: HFrameConfig,
}

#[pymethods]
impl PyHFrameConfig {
    /// Create configuration with specified parameters
    #[new]
    #[pyo3(signature = (max_frames=256, history_len=32))]
    fn new(max_frames: usize, history_len: usize) -> Self {
        PyHFrameConfig {
            inner: HFrameConfig {
                max_frames,
                max_static_frames: max_frames / 2,
                history_len,
                enable_overflow: false,
                chain_cache_size: 64,
            },
        }
    }

    /// Small robot preset (256 frames, ~550KB memory)
    #[staticmethod]
    fn small() -> Self {
        PyHFrameConfig {
            inner: HFrameConfig::small(),
        }
    }

    /// Medium robot preset (1024 frames, ~2.2MB memory)
    #[staticmethod]
    fn medium() -> Self {
        PyHFrameConfig {
            inner: HFrameConfig::medium(),
        }
    }

    /// Large simulation preset (4096 frames, ~9MB memory)
    #[staticmethod]
    fn large() -> Self {
        PyHFrameConfig {
            inner: HFrameConfig::large(),
        }
    }

    /// Massive simulation preset (16384 frames, ~35MB memory)
    #[staticmethod]
    fn massive() -> Self {
        PyHFrameConfig {
            inner: HFrameConfig::massive(),
        }
    }

    #[getter]
    fn max_frames(&self) -> usize {
        self.inner.max_frames
    }

    #[getter]
    fn history_len(&self) -> usize {
        self.inner.history_len
    }

    /// Get human-readable memory estimate
    fn memory_estimate(&self) -> String {
        self.inner.memory_estimate()
    }

    fn __repr__(&self) -> String {
        format!(
            "HFrameConfig(max_frames={}, history_len={}, memory={})",
            self.inner.max_frames,
            self.inner.history_len,
            self.inner.memory_estimate()
        )
    }
}

/// Python wrapper for HFrame - High-Performance Transform System
///
/// HFrame provides lock-free coordinate frame management for robotics.
/// It's designed as a high-performance alternative to ROS2 TF2.
///
/// Example:
///     >>> from horus import HFrame, Transform
///     >>> hf = HFrame()
///     >>> hf.register_frame("world", None)
///     >>> hf.register_frame("base_link", "world")
///     >>> hf.update_transform("base_link", Transform.from_translation([1.0, 0.0, 0.0]))
///     >>> tf = hf.tf("base_link", "world")
///     >>> print(tf.translation)  # [1.0, 0.0, 0.0]
#[pyclass(name = "HFrame")]
pub struct PyHFrame {
    inner: HFrame,
}

#[pymethods]
impl PyHFrame {
    /// Create a new HFrame with default configuration (256 frames)
    #[new]
    #[pyo3(signature = (config=None))]
    fn new(config: Option<PyHFrameConfig>) -> Self {
        let inner = match config {
            Some(cfg) => HFrame::with_config(cfg.inner),
            None => HFrame::new(),
        };
        PyHFrame { inner }
    }

    /// Create with small robot preset (256 frames)
    #[staticmethod]
    fn small() -> Self {
        PyHFrame {
            inner: HFrame::small(),
        }
    }

    /// Create with medium robot preset (1024 frames)
    #[staticmethod]
    fn medium() -> Self {
        PyHFrame {
            inner: HFrame::medium(),
        }
    }

    /// Create with large simulation preset (4096 frames)
    #[staticmethod]
    fn large() -> Self {
        PyHFrame {
            inner: HFrame::large(),
        }
    }

    /// Create with massive simulation preset (16384 frames)
    #[staticmethod]
    fn massive() -> Self {
        PyHFrame {
            inner: HFrame::massive(),
        }
    }

    /// Register a new frame
    ///
    /// Args:
    ///     name: Frame name (e.g., "base_link", "camera_frame")
    ///     parent: Parent frame name, or None for root frames
    ///
    /// Returns:
    ///     Frame ID (integer) for fast lookups
    #[pyo3(signature = (name, parent=None))]
    fn register_frame(&self, name: &str, parent: Option<&str>) -> PyResult<u32> {
        self.inner.register_frame(name, parent).map_err(to_py_err)
    }

    /// Register a static frame (transform never changes)
    ///
    /// Args:
    ///     name: Frame name
    ///     transform: The static transform
    ///     parent: Parent frame name, or None for root frames
    ///
    /// Returns:
    ///     Frame ID
    #[pyo3(signature = (name, transform, parent=None))]
    fn register_static_frame(
        &self,
        name: &str,
        transform: &PyTransform,
        parent: Option<&str>,
    ) -> PyResult<u32> {
        self.inner
            .register_static_frame(name, parent, &transform.inner)
            .map_err(to_py_err)
    }

    /// Unregister a dynamic frame
    fn unregister_frame(&self, name: &str) -> PyResult<()> {
        self.inner.unregister_frame(name).map_err(to_py_err)
    }

    /// Get frame ID by name
    fn frame_id(&self, name: &str) -> Option<u32> {
        self.inner.frame_id(name)
    }

    /// Get frame name by ID
    fn frame_name(&self, id: u32) -> Option<String> {
        self.inner.frame_name(id)
    }

    /// Check if a frame exists
    fn has_frame(&self, name: &str) -> bool {
        self.inner.has_frame(name)
    }

    /// Get all registered frame names
    fn all_frames(&self) -> Vec<String> {
        self.inner.all_frames()
    }

    /// Get number of registered frames
    fn frame_count(&self) -> usize {
        self.inner.frame_count()
    }

    /// Update a frame's transform
    ///
    /// Args:
    ///     name: Frame name
    ///     transform: New transform from parent to this frame
    ///     timestamp_ns: Timestamp in nanoseconds (default: now)
    #[pyo3(signature = (name, transform, timestamp_ns=None))]
    fn update_transform(
        &self,
        name: &str,
        transform: &PyTransform,
        timestamp_ns: Option<u64>,
    ) -> PyResult<()> {
        let ts = timestamp_ns.unwrap_or_else(timestamp_now);
        self.inner
            .update_transform(name, &transform.inner, ts)
            .map_err(to_py_err)
    }

    /// Update a frame's transform by ID (faster)
    ///
    /// Raises ValueError if the transform contains NaN/Inf or an invalid quaternion.
    #[pyo3(signature = (frame_id, transform, timestamp_ns=None))]
    fn update_transform_by_id(
        &self,
        frame_id: u32,
        transform: &PyTransform,
        timestamp_ns: Option<u64>,
    ) -> PyResult<()> {
        let ts = timestamp_ns.unwrap_or_else(timestamp_now);
        self.inner
            .update_transform_by_id(frame_id, &transform.inner, ts)
            .map_err(to_py_err)
    }

    /// Get transform from src frame to dst frame
    ///
    /// Args:
    ///     src: Source frame name
    ///     dst: Destination frame name
    ///
    /// Returns:
    ///     Transform that converts points from src to dst frame
    fn tf(&self, src: &str, dst: &str) -> PyResult<PyTransform> {
        self.inner
            .tf(src, dst)
            .map(|t| PyTransform { inner: t })
            .map_err(to_py_err)
    }

    /// Get transform at specific timestamp with interpolation
    ///
    /// Args:
    ///     src: Source frame name
    ///     dst: Destination frame name
    ///     timestamp_ns: Timestamp in nanoseconds
    ///
    /// Returns:
    ///     Interpolated transform at the given timestamp
    fn tf_at(&self, src: &str, dst: &str, timestamp_ns: u64) -> PyResult<PyTransform> {
        self.inner
            .tf_at(src, dst, timestamp_ns)
            .map(|t| PyTransform { inner: t })
            .map_err(to_py_err)
    }

    /// Get transform by frame IDs (fastest)
    fn tf_by_id(&self, src: u32, dst: u32) -> Option<PyTransform> {
        self.inner
            .tf_by_id(src, dst)
            .map(|t| PyTransform { inner: t })
    }

    /// Check if a transform path exists between two frames
    fn can_transform(&self, src: &str, dst: &str) -> bool {
        self.inner.can_transform(src, dst)
    }

    /// Transform a point from one frame to another
    fn transform_point(&self, src: &str, dst: &str, point: [f64; 3]) -> PyResult<[f64; 3]> {
        self.inner
            .transform_point(src, dst, point)
            .map_err(to_py_err)
    }

    /// Get the parent frame of a given frame
    fn parent(&self, name: &str) -> Option<String> {
        self.inner.parent(name)
    }

    /// Get all children of a frame
    fn children(&self, name: &str) -> Vec<String> {
        self.inner.children(name)
    }

    /// Get the frame chain from src to dst
    fn frame_chain(&self, src: &str, dst: &str) -> PyResult<Vec<String>> {
        self.inner.frame_chain(src, dst).map_err(to_py_err)
    }

    // ════════════════════════════════════════════════════════════
    // Phase 2: Waits & Staleness
    // ════════════════════════════════════════════════════════════

    /// Block until a transform between src and dst becomes available.
    ///
    /// Args:
    ///     src: Source frame name
    ///     dst: Destination frame name
    ///     timeout_sec: Maximum wait time in seconds (default: 5.0)
    ///
    /// Returns:
    ///     Transform once available
    ///
    /// Raises:
    ///     ValueError: If timeout expires before transform is available
    #[pyo3(signature = (src, dst, timeout_sec=5.0))]
    fn wait_for_transform(
        &self,
        py: Python,
        src: &str,
        dst: &str,
        timeout_sec: f64,
    ) -> PyResult<PyTransform> {
        let timeout = std::time::Duration::from_secs_f64(timeout_sec);
        let inner = &self.inner;
        let src_owned = src.to_string();
        let dst_owned = dst.to_string();
        // Release the GIL while waiting
        py.detach(|| {
            inner
                .wait_for_transform(&src_owned, &dst_owned, timeout)
                .map(|t| PyTransform { inner: t })
                .map_err(to_py_err)
        })
    }

    /// Get transform at timestamp with time tolerance for interpolation.
    ///
    /// Args:
    ///     src: Source frame name
    ///     dst: Destination frame name
    ///     timestamp_ns: Target timestamp in nanoseconds
    ///     tolerance_ns: Tolerance window in nanoseconds (default: 100ms)
    ///
    /// Returns:
    ///     Interpolated transform within tolerance window
    #[pyo3(signature = (src, dst, timestamp_ns, tolerance_ns=100_000_000))]
    fn tf_at_with_tolerance(
        &self,
        src: &str,
        dst: &str,
        timestamp_ns: u64,
        tolerance_ns: u64,
    ) -> PyResult<PyTransform> {
        self.inner
            .tf_at_with_tolerance(src, dst, timestamp_ns, tolerance_ns)
            .map(|t| PyTransform { inner: t })
            .map_err(to_py_err)
    }

    /// Check if a frame's transform data is stale.
    ///
    /// Args:
    ///     name: Frame name
    ///     max_age_sec: Maximum acceptable age in seconds (default: 1.0)
    ///
    /// Returns:
    ///     True if the frame hasn't been updated within max_age_sec
    #[pyo3(signature = (name, max_age_sec=1.0))]
    fn is_stale(&self, name: &str, max_age_sec: f64) -> bool {
        let max_age_ns = (max_age_sec * 1_000_000_000.0) as u64;
        self.inner.is_stale_now(name, max_age_ns)
    }

    /// Get seconds since a frame was last updated.
    ///
    /// Args:
    ///     name: Frame name
    ///
    /// Returns:
    ///     Seconds since last update, or None if frame has never been updated
    fn time_since_last_update(&self, name: &str) -> Option<f64> {
        self.inner
            .time_since_last_update_now(name)
            .map(|ns| ns as f64 / 1_000_000_000.0)
    }

    /// Set a static transform (registered once, never expires).
    ///
    /// Use for fixed relationships like camera_link → base_link.
    ///
    /// Args:
    ///     name: Frame name (must already be registered)
    ///     transform: The static transform from parent to this frame
    fn set_static_transform(&self, name: &str, transform: &PyTransform) -> PyResult<()> {
        self.inner
            .set_static_transform(name, &transform.inner)
            .map_err(to_py_err)
    }

    /// Transform a 3D vector from one frame to another (rotation only).
    ///
    /// Unlike transform_point(), this only applies rotation without
    /// translation. Use for directions, velocities, angular rates.
    ///
    /// Args:
    ///     src: Source frame name
    ///     dst: Destination frame name
    ///     vector: [x, y, z] vector to transform
    ///
    /// Returns:
    ///     Rotated [x, y, z] vector
    fn transform_vector(&self, src: &str, dst: &str, vector: [f64; 3]) -> PyResult<[f64; 3]> {
        self.inner
            .transform_vector(src, dst, vector)
            .map_err(to_py_err)
    }

    /// Check if a transform is available at a specific timestamp.
    ///
    /// Non-throwing alternative to tf_at() — returns bool instead of
    /// raising an error.
    ///
    /// Args:
    ///     src: Source frame name
    ///     dst: Destination frame name
    ///     timestamp_ns: Timestamp to check
    ///
    /// Returns:
    ///     True if transform data exists at the given time
    fn can_transform_at(&self, src: &str, dst: &str, timestamp_ns: u64) -> bool {
        self.inner.can_transform_at(src, dst, timestamp_ns)
    }

    // ════════════════════════════════════════════════════════════
    // Phase 3: Debug & Diagnostics
    // ════════════════════════════════════════════════════════════

    /// Get frame tree statistics.
    ///
    /// Returns:
    ///     Dict with total_frames, static_frames, dynamic_frames,
    ///     max_frames, history_len, tree_depth, root_count
    fn stats(&self) -> PyResult<pyo3::Py<pyo3::types::PyDict>> {
        Python::attach(|py| {
            let s = self.inner.stats();
            let dict = pyo3::types::PyDict::new(py);
            dict.set_item("total_frames", s.total_frames)?;
            dict.set_item("static_frames", s.static_frames)?;
            dict.set_item("dynamic_frames", s.dynamic_frames)?;
            dict.set_item("max_frames", s.max_frames)?;
            dict.set_item("history_len", s.history_len)?;
            dict.set_item("tree_depth", s.tree_depth)?;
            dict.set_item("root_count", s.root_count)?;
            Ok(dict.into())
        })
    }

    /// Validate the frame tree for consistency.
    ///
    /// Checks for cycles, orphan frames, and other structural issues.
    ///
    /// Returns:
    ///     None if valid
    ///
    /// Raises:
    ///     ValueError: Description of the consistency issue found
    fn validate(&self) -> PyResult<()> {
        self.inner.validate().map_err(to_py_err)
    }

    /// Get detailed info about a specific frame.
    ///
    /// Args:
    ///     name: Frame name
    ///
    /// Returns:
    ///     Dict with name, id, parent, is_static, children_count, depth,
    ///     time_range (oldest_ns, newest_ns), or None if frame not found
    fn frame_info(&self, name: &str) -> PyResult<Option<pyo3::Py<pyo3::types::PyDict>>> {
        let info = match self.inner.frame_info(name) {
            Some(i) => i,
            None => return Ok(None),
        };
        Python::attach(|py| {
            let dict = pyo3::types::PyDict::new(py);
            dict.set_item("name", &info.name)?;
            dict.set_item("id", info.id)?;
            dict.set_item("parent", &info.parent)?;
            dict.set_item("is_static", info.is_static)?;
            dict.set_item("children_count", info.children_count)?;
            dict.set_item("depth", info.depth)?;
            if let Some((oldest, newest)) = info.time_range {
                let range = pyo3::types::PyTuple::new(py, [oldest, newest])?;
                dict.set_item("time_range", range)?;
            } else {
                dict.set_item("time_range", py.None())?;
            }
            Ok(Some(dict.into()))
        })
    }

    fn __repr__(&self) -> String {
        let stats = self.inner.stats();
        format!(
            "HFrame({} frames: {} static, {} dynamic)",
            stats.total_frames, stats.static_frames, stats.dynamic_frames
        )
    }
}

/// Get current timestamp in nanoseconds
#[pyfunction]
pub fn get_timestamp_ns() -> u64 {
    timestamp_now()
}
