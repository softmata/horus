//! Builder-pattern frame registration API.
//!
//! Provides a fluent API for registering frames:
//!
//! ```rust,ignore
//! hf.add_frame("world").build()?;                              // root frame
//! hf.add_frame("base_link").parent("world").build()?;          // child frame
//!
//! hf.add_frame("camera")
//!     .parent("base_link")
//!     .static_transform(&Transform::from_translation([0.1, 0.0, 0.5]))
//!     .build()?;
//! ```

use super::HFrame;
use crate::hframe::transform::Transform;
use crate::hframe::types::FrameId;
use horus_core::HorusResult;

/// Builder for registering frames (both dynamic and static).
///
/// Created by [`HFrame::add_frame`].
///
/// # Examples
///
/// ```rust,ignore
/// // Root frame (no parent)
/// hf.add_frame("world").build()?;
///
/// // Dynamic child frame
/// hf.add_frame("base_link").parent("world").build()?;
///
/// // Static frame with fixed transform (never changes, more efficient)
/// hf.add_frame("camera")
///     .parent("base_link")
///     .static_transform(&Transform::xyz(0.1, 0.0, 0.5))
///     .build()?;
/// ```
pub struct FrameBuilder<'a> {
    hf: &'a HFrame,
    name: &'a str,
    parent: Option<&'a str>,
    static_tf: Option<Transform>,
}

impl<'a> FrameBuilder<'a> {
    pub(crate) fn new(hf: &'a HFrame, name: &'a str) -> Self {
        Self {
            hf,
            name,
            parent: None,
            static_tf: None,
        }
    }

    /// Set the parent frame.
    #[inline]
    pub fn parent(mut self, parent: &'a str) -> Self {
        self.parent = Some(parent);
        self
    }

    /// Mark this frame as static with a fixed transform.
    ///
    /// Static frames cannot be updated after registration and use less memory.
    /// Requires a parent (static root frames are not supported).
    #[inline]
    pub fn static_transform(mut self, tf: &Transform) -> Self {
        self.static_tf = Some(tf.clone());
        self
    }

    /// Register this frame and return its ID.
    #[inline]
    pub fn build(self) -> HorusResult<FrameId> {
        if let Some(tf) = &self.static_tf {
            self.hf.register_static_frame(self.name, self.parent, tf)
        } else {
            self.hf.register_frame(self.name, self.parent)
        }
    }
}

// ── Deprecated types (kept for backward compatibility) ───────────────────────

/// Deprecated: use [`FrameBuilder`] with `.static_transform()` instead.
#[deprecated(
    since = "0.2.0",
    note = "Use hf.add_frame(name).parent(p).static_transform(&tf).build()? instead"
)]
pub struct StaticFrameBuilder<'a> {
    hf: &'a HFrame,
    name: &'a str,
}

/// Deprecated: use [`FrameBuilder`] with `.static_transform()` instead.
#[deprecated(
    since = "0.2.0",
    note = "Use hf.add_frame(name).parent(p).static_transform(&tf).build()? instead"
)]
pub struct StaticFrameBuilderWithParent<'a> {
    hf: &'a HFrame,
    name: &'a str,
    parent: &'a str,
}

#[allow(deprecated)]
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

#[allow(deprecated)]
impl<'a> StaticFrameBuilderWithParent<'a> {
    /// Set the transform and register this static frame.
    #[inline]
    pub fn transform(self, tf: &Transform) -> HorusResult<FrameId> {
        self.hf
            .register_static_frame(self.name, Some(self.parent), tf)
    }
}
