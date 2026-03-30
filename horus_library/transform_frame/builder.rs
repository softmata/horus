//! Builder-pattern frame registration API.
//!
//! Provides a fluent API for registering frames:
//!
//! ```rust,ignore
//! tf.add_frame("world").build()?;                              // root frame
//! tf.add_frame("base_link").parent("world").build()?;          // child frame
//!
//! tf.add_frame("camera")
//!     .parent("base_link")
//!     .static_transform(&Transform::from_translation([0.1, 0.0, 0.5]))
//!     .build()?;
//! ```

use super::TransformFrame;
use crate::transform_frame::transform::Transform;
use crate::transform_frame::types::FrameId;
use horus_core::HorusResult;

/// Builder for registering frames (both dynamic and static).
///
/// Created by [`TransformFrame::add_frame`].
///
/// # Examples
///
/// ```rust,ignore
/// // Root frame (no parent)
/// tf.add_frame("world").build()?;
///
/// // Dynamic child frame
/// tf.add_frame("base_link").parent("world").build()?;
///
/// // Static frame with fixed transform (never changes, more efficient)
/// tf.add_frame("camera")
///     .parent("base_link")
///     .static_transform(&Transform::xyz(0.1, 0.0, 0.5))
///     .build()?;
/// ```
pub struct FrameBuilder<'a> {
    frame: &'a TransformFrame,
    name: &'a str,
    parent: Option<&'a str>,
    static_tf: Option<Transform>,
}

impl<'a> FrameBuilder<'a> {
    pub(crate) fn new(frame: &'a TransformFrame, name: &'a str) -> Self {
        Self {
            frame,
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
        self.static_tf = Some(*tf);
        self
    }

    /// Register this frame and return its ID.
    #[inline]
    pub fn build(self) -> HorusResult<FrameId> {
        if let Some(tf) = &self.static_tf {
            self.frame.register_static_frame(self.name, self.parent, tf)
        } else {
            self.frame.register_frame(self.name, self.parent)
        }
    }
}
