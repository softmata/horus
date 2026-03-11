//! Builder-pattern query API for TransformFrame transforms.
//!
//! Provides a fluent, direction-unambiguous API for transform lookups:
//!
//! ```rust,ignore
//! // Instead of: tf.tf("camera", "world")?
//! tf.query("camera").to("world").lookup()?;
//! tf.query("camera").to("world").at(timestamp)?;
//! tf.query("camera").to("world").point([1.0, 0.0, 0.0])?;
//! ```
//!
//! All methods are zero-overhead wrappers that inline to the same code
//! as calling `tf.tf()` directly.

use super::TransformFrame;
use crate::transform_frame::transform::Transform;
use horus_core::HorusResult;

/// Intermediate builder holding the source frame.
///
/// Created by [`TransformFrame::query`]. Call `.to()` to complete the query.
pub struct TransformQueryFrom<'a> {
    frame: &'a TransformFrame,
    src: &'a str,
}

impl<'a> TransformQueryFrom<'a> {
    pub(crate) fn new(frame: &'a TransformFrame, src: &'a str) -> Self {
        Self { frame, src }
    }

    /// Set the destination (target) frame, completing the query builder.
    #[inline]
    pub fn to(self, dst: &'a str) -> TransformQuery<'a> {
        TransformQuery {
            frame: self.frame,
            src: self.src,
            dst,
        }
    }
}

/// A fully-specified transform query between two frames.
///
/// Created by [`TransformQueryFrom::to`]. All methods delegate to
/// the corresponding [`TransformFrame`] methods with zero overhead.
pub struct TransformQuery<'a> {
    frame: &'a TransformFrame,
    src: &'a str,
    dst: &'a str,
}

impl<'a> TransformQuery<'a> {
    /// Look up the latest transform from `src` to `dst`.
    ///
    /// Equivalent to `tf.tf(src, dst)`.
    #[inline]
    pub fn lookup(&self) -> HorusResult<Transform> {
        self.frame.tf(self.src, self.dst)
    }

    /// Look up the transform at a specific timestamp with interpolation.
    ///
    /// Equivalent to `tf.tf_at(src, dst, timestamp_ns)`.
    #[inline]
    pub fn at(&self, timestamp_ns: u64) -> HorusResult<Transform> {
        self.frame.tf_at(self.src, self.dst, timestamp_ns)
    }

    /// Look up the transform with strict time-range checking.
    ///
    /// Returns `Err(HorusError::Extrapolation)` if any frame in the chain
    /// would need to extrapolate beyond its buffer window.
    ///
    /// Equivalent to `tf.tf_at_strict(src, dst, timestamp_ns)`.
    #[inline]
    pub fn at_strict(&self, timestamp_ns: u64) -> HorusResult<Transform> {
        self.frame.tf_at_strict(self.src, self.dst, timestamp_ns)
    }

    /// Look up the transform with a time tolerance.
    ///
    /// Equivalent to `tf.tf_at_with_tolerance(src, dst, timestamp_ns, tolerance_ns)`.
    #[inline]
    pub fn at_with_tolerance(
        &self,
        timestamp_ns: u64,
        tolerance_ns: u64,
    ) -> HorusResult<Transform> {
        self.frame
            .tf_at_with_tolerance(self.src, self.dst, timestamp_ns, tolerance_ns)
    }

    /// Transform a 3D point from `src` frame to `dst` frame.
    ///
    /// Equivalent to `tf.transform_point(src, dst, point)`.
    #[inline]
    pub fn point(&self, point: [f64; 3]) -> HorusResult<[f64; 3]> {
        self.frame.transform_point(self.src, self.dst, point)
    }

    /// Transform a 3D vector from `src` frame to `dst` frame.
    ///
    /// Vectors are rotation-only (translation not applied).
    /// Equivalent to `tf.transform_vector(src, dst, vector)`.
    #[inline]
    pub fn vector(&self, vector: [f64; 3]) -> HorusResult<[f64; 3]> {
        self.frame.transform_vector(self.src, self.dst, vector)
    }

    /// Check if a transform is available at the given timestamp (strict).
    ///
    /// Equivalent to `tf.can_transform_at(src, dst, timestamp_ns)`.
    #[inline]
    pub fn can_at(&self, timestamp_ns: u64) -> bool {
        self.frame.can_transform_at(self.src, self.dst, timestamp_ns)
    }

    /// Check if a transform is available at the given timestamp with tolerance.
    ///
    /// Equivalent to `tf.can_transform_at_with_tolerance(src, dst, timestamp_ns, tolerance_ns)`.
    #[inline]
    pub fn can_at_with_tolerance(&self, timestamp_ns: u64, tolerance_ns: u64) -> bool {
        self.frame
            .can_transform_at_with_tolerance(self.src, self.dst, timestamp_ns, tolerance_ns)
    }

    /// Get the frame chain from `src` to `dst`.
    ///
    /// Equivalent to `tf.frame_chain(src, dst)`.
    #[inline]
    pub fn chain(&self) -> HorusResult<Vec<String>> {
        self.frame.frame_chain(self.src, self.dst)
    }

    /// Block until the transform becomes available, or timeout expires.
    ///
    /// Requires the `wait` feature flag.
    /// Equivalent to `tf.wait_for_transform(src, dst, timeout)`.
    #[cfg(feature = "wait")]
    #[inline]
    pub fn wait(&self, timeout: std::time::Duration) -> HorusResult<Transform> {
        self.frame.wait_for_transform(self.src, self.dst, timeout)
    }

    /// Block until the transform at a specific timestamp becomes available.
    ///
    /// Requires the `wait` feature flag.
    /// Equivalent to `tf.wait_for_transform_at(src, dst, timestamp_ns, timeout)`.
    #[cfg(feature = "wait")]
    #[inline]
    pub fn wait_at(
        &self,
        timestamp_ns: u64,
        timeout: std::time::Duration,
    ) -> HorusResult<Transform> {
        self.frame
            .wait_for_transform_at(self.src, self.dst, timestamp_ns, timeout)
    }

    /// Asynchronously wait until the transform becomes available.
    ///
    /// Requires the `async-wait` feature flag.
    /// Equivalent to `tf.wait_for_transform_async(src, dst, timeout)`.
    #[cfg(feature = "async-wait")]
    #[inline]
    pub async fn wait_async(&self, timeout: std::time::Duration) -> HorusResult<Transform> {
        self.frame
            .wait_for_transform_async(self.src, self.dst, timeout)
            .await
    }

    /// Asynchronously wait until the transform at a specific timestamp is available.
    ///
    /// Requires the `async-wait` feature flag.
    /// Equivalent to `tf.wait_for_transform_at_async(src, dst, timestamp_ns, timeout)`.
    #[cfg(feature = "async-wait")]
    #[inline]
    pub async fn wait_at_async(
        &self,
        timestamp_ns: u64,
        timeout: std::time::Duration,
    ) -> HorusResult<Transform> {
        self.frame
            .wait_for_transform_at_async(self.src, self.dst, timestamp_ns, timeout)
            .await
    }
}
