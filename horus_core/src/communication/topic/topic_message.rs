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
use crate::types::{DepthImageDescriptor, ImageDescriptor, PointCloudDescriptor};

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
// Pool-backed types: Image, PointCloud, DepthImage
// All share the same to_wire/from_wire pattern via descriptor + pool.
// ============================================================================

/// Implement TopicMessage for a pool-backed tensor type.
///
/// Requires the type to have: `pool()`, `descriptor()`, `from_owned(wire, pool)`.
macro_rules! impl_pool_backed_topic_message {
    ($($Type:ty => $Descriptor:ty),+ $(,)?) => {
        $(
            impl TopicMessage for $Type {
                type Wire = $Descriptor;

                #[inline]
                fn to_wire(&self, _pool: &Option<Arc<TensorPool>>) -> $Descriptor {
                    self.pool().retain(self.descriptor().tensor());
                    *self.descriptor()
                }

                #[inline]
                fn from_wire(wire: $Descriptor, pool: &Option<Arc<TensorPool>>) -> Self {
                    let p = pool.as_ref().cloned().unwrap_or_else(global_pool);
                    <$Type>::from_owned(wire, p)
                }

                #[inline(always)]
                fn needs_pool() -> bool {
                    true
                }
            }
        )+
    };
}

impl_pool_backed_topic_message!(
    Image      => ImageDescriptor,
    PointCloud => PointCloudDescriptor,
    DepthImage => DepthImageDescriptor,
);
