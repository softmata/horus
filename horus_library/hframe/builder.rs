//! Builder-pattern frame registration API.
//!
//! Provides a fluent API for registering frames:
//!
//! ```rust,ignore
//! hf.add_frame("world").build()?;                   // root frame
//! hf.add_frame("base_link").parent("world")?;       // child frame
//!
//! hf.add_static("camera")
//!     .parent("base_link")
//!     .transform(&Transform::from_translation([0.1, 0.0, 0.5]))?;
//! ```

use super::HFrame;
use crate::hframe::transform::Transform;
use crate::hframe::types::FrameId;
use horus_core::HorusResult;

/// Builder for registering a dynamic frame.
///
/// Created by [`HFrame::add_frame`].
pub struct FrameBuilder<'a> {
    hf: &'a HFrame,
    name: &'a str,
}

impl<'a> FrameBuilder<'a> {
    pub(crate) fn new(hf: &'a HFrame, name: &'a str) -> Self {
        Self { hf, name }
    }

    /// Set the parent frame and register this frame.
    ///
    /// This is a terminal method — it registers the frame and returns the ID.
    #[inline]
    pub fn parent(self, parent: &str) -> HorusResult<FrameId> {
        self.hf.register_frame(self.name, Some(parent))
    }

    /// Register this frame as a root (no parent).
    ///
    /// This is a terminal method — it registers the frame and returns the ID.
    #[inline]
    pub fn build(self) -> HorusResult<FrameId> {
        self.hf.register_frame(self.name, None)
    }
}

/// Builder for registering a static frame.
///
/// Created by [`HFrame::add_static`]. Static frames require both a parent
/// and a transform to be set.
pub struct StaticFrameBuilder<'a> {
    hf: &'a HFrame,
    name: &'a str,
}

/// Intermediate builder after parent is set on a static frame.
pub struct StaticFrameBuilderWithParent<'a> {
    hf: &'a HFrame,
    name: &'a str,
    parent: &'a str,
}

impl<'a> StaticFrameBuilder<'a> {
    pub(crate) fn new(hf: &'a HFrame, name: &'a str) -> Self {
        Self { hf, name }
    }

    /// Set the parent frame for this static frame.
    #[inline]
    pub fn parent(self, parent: &'a str) -> StaticFrameBuilderWithParent<'a> {
        StaticFrameBuilderWithParent {
            hf: self.hf,
            name: self.name,
            parent,
        }
    }
}

impl<'a> StaticFrameBuilderWithParent<'a> {
    /// Set the transform and register this static frame.
    ///
    /// This is a terminal method — it registers the frame and returns the ID.
    #[inline]
    pub fn transform(self, tf: &Transform) -> HorusResult<FrameId> {
        self.hf
            .register_static_frame(self.name, Some(self.parent), tf)
    }
}
