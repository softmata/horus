//! TopicMessage trait — unified wire protocol for Topic<T>
//!
//! This trait bridges the gap between user-facing types and the ring buffer's
//! wire format. Two categories:
//!
//! 1. **Direct types** (`CmdVel`, `Imu`, `i32`, `String`, ...):
//!    `Wire = Self`. Covered by a blanket impl for `T: Serialize + DeserializeOwned + ...`.
//!
//! 2. **Pool-backed types** (`Image`, `PointCloud`, `DepthImage`):
//!    `Wire = XxxDescriptor` (288-336 byte Pod). Explicit impls handle
//!    pool retain/release during send/recv.
//!
//! The blanket impl and explicit impls never overlap because Image/PointCloud/
//! DepthImage contain `Arc<TensorPool>` and thus don't implement `Serialize`.

use std::sync::Arc;

use serde::{de::DeserializeOwned, Serialize};

use crate::memory::depth_image::DepthImage;
use crate::memory::image::Image;
use crate::memory::pointcloud::PointCloud;
use crate::memory::TensorPool;
use horus_types::{DepthImageDescriptor, ImageDescriptor, PointCloudDescriptor};

use super::pool_registry::global_pool;

/// Defines how a type is transported through Topic's ring buffer.
///
/// `Wire` is the actual type stored in the ring buffer. For most types,
/// `Wire = Self` (no conversion). For pool-backed types like `Image`,
/// `Wire` is a lightweight Pod descriptor.
pub trait TopicMessage: Sized + Send + 'static {
    /// The wire type that flows through the ring buffer.
    type Wire: Clone + Send + Sync + Serialize + DeserializeOwned + 'static;

    /// Convert from user type to wire type for sending.
    fn to_wire(&self, pool: &Option<Arc<TensorPool>>) -> Self::Wire;

    /// Convert from wire type back to user type after receiving.
    fn from_wire(wire: Self::Wire, pool: &Option<Arc<TensorPool>>) -> Self;

    /// Whether this type needs a TensorPool (pool-backed types return true).
    fn needs_pool() -> bool;
}

// ============================================================================
// Blanket impl: all serializable types (CmdVel, Imu, i32, String, HorusTensor, ...)
// ============================================================================

impl<T> TopicMessage for T
where
    T: Clone + Send + Sync + Serialize + DeserializeOwned + 'static,
{
    type Wire = T;

    #[inline(always)]
    fn to_wire(&self, _pool: &Option<Arc<TensorPool>>) -> Self {
        self.clone()
    }

    #[inline(always)]
    fn from_wire(wire: Self, _pool: &Option<Arc<TensorPool>>) -> Self {
        wire
    }

    #[inline(always)]
    fn needs_pool() -> bool {
        false
    }
}

// ============================================================================
// Image: Wire = ImageDescriptor (288 bytes, Pod)
// ============================================================================

impl TopicMessage for Image {
    type Wire = ImageDescriptor;

    #[inline]
    fn to_wire(&self, _pool: &Option<Arc<TensorPool>>) -> ImageDescriptor {
        // Retain the tensor slot so receiver can access pixel data
        self.pool().retain(self.descriptor().tensor());
        *self.descriptor()
    }

    #[inline]
    fn from_wire(wire: ImageDescriptor, pool: &Option<Arc<TensorPool>>) -> Self {
        let p = pool.as_ref().cloned().unwrap_or_else(global_pool);
        Image::from_owned(wire, p)
    }

    #[inline(always)]
    fn needs_pool() -> bool {
        true
    }
}

// ============================================================================
// PointCloud: Wire = PointCloudDescriptor (336 bytes, Pod)
// ============================================================================

impl TopicMessage for PointCloud {
    type Wire = PointCloudDescriptor;

    #[inline]
    fn to_wire(&self, _pool: &Option<Arc<TensorPool>>) -> PointCloudDescriptor {
        self.pool().retain(self.descriptor().tensor());
        *self.descriptor()
    }

    #[inline]
    fn from_wire(wire: PointCloudDescriptor, pool: &Option<Arc<TensorPool>>) -> Self {
        let p = pool.as_ref().cloned().unwrap_or_else(global_pool);
        PointCloud::from_owned(wire, p)
    }

    #[inline(always)]
    fn needs_pool() -> bool {
        true
    }
}

// ============================================================================
// DepthImage: Wire = DepthImageDescriptor (288 bytes, Pod)
// ============================================================================

impl TopicMessage for DepthImage {
    type Wire = DepthImageDescriptor;

    #[inline]
    fn to_wire(&self, _pool: &Option<Arc<TensorPool>>) -> DepthImageDescriptor {
        self.pool().retain(self.descriptor().tensor());
        *self.descriptor()
    }

    #[inline]
    fn from_wire(wire: DepthImageDescriptor, pool: &Option<Arc<TensorPool>>) -> Self {
        let p = pool.as_ref().cloned().unwrap_or_else(global_pool);
        DepthImage::from_owned(wire, p)
    }

    #[inline(always)]
    fn needs_pool() -> bool {
        true
    }
}
