//! TopicMessage trait — unified wire protocol for `Topic<T>`
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

use crate::memory::costmap::CostMap;
use crate::memory::depth_image::DepthImage;
use crate::memory::image::Image;
use crate::memory::occupancy_grid::OccupancyGrid;
use crate::memory::pointcloud::PointCloud;
use crate::memory::TensorHandle;
use crate::memory::TensorPool;
use crate::types::{
    CostMapDescriptor, DepthImageDescriptor, ImageDescriptor, OccupancyGridDescriptor,
    PointCloudDescriptor, Tensor,
};

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

    /// Convert from wire type back to user type on the receive path, taking a
    /// generation-guarded reference to the backing pool slot.
    ///
    /// Returns `None` when the pool slot was freed or reallocated since the
    /// message was published — i.e. a message that was superseded (drop-oldest)
    /// before this subscriber read it. The caller treats `None` as "no message",
    /// never materializing a stale handle. For plain (non-pool-backed) types this
    /// always returns `Some`.
    ///
    /// Pool-backed types override this to `try_retain` the slot so that **each**
    /// subscriber holds its own reference: a co-subscriber dropping its handle can
    /// no longer free the slot out from under the others (the multi-subscriber
    /// use-after-free). The producer-side keep-alive is released on the next send
    /// (see `Topic::send` for pool-backed types), so this does not leak.
    fn try_from_wire(wire: Self::Wire, pool: &Option<Arc<TensorPool>>) -> Option<Self> {
        Some(Self::from_wire(wire, pool))
    }

    /// Whether this type needs a TensorPool (pool-backed types return true).
    fn needs_pool() -> bool;
}

// ============================================================================
// Blanket impl: all serializable types (CmdVel, Imu, i32, String, Tensor, ...)
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

                #[inline]
                fn try_from_wire(wire: $Descriptor, pool: &Option<Arc<TensorPool>>) -> Option<Self> {
                    let p = pool.as_ref().cloned().unwrap_or_else(global_pool);
                    // Generation-guarded retain so each subscriber owns its own
                    // reference. `Err` => the slot was freed/reallocated since the
                    // message was published (superseded under drop-oldest) =>
                    // missed message. `from_owned` does not retain, so the
                    // `try_retain` above is exactly this handle's reference.
                    if p.try_retain(wire.tensor()).is_err() {
                        return None;
                    }
                    Some(<$Type>::from_owned(wire, p))
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
    Image         => ImageDescriptor,
    PointCloud    => PointCloudDescriptor,
    DepthImage    => DepthImageDescriptor,
    OccupancyGrid => OccupancyGridDescriptor,
);

// ============================================================================
// CostMap: Custom TopicMessage — dual-tensor (grid + cost)
// The macro above assumes one tensor per type. CostMap has two.
// ============================================================================

impl TopicMessage for CostMap {
    type Wire = CostMapDescriptor;

    #[inline]
    fn to_wire(&self, _pool: &Option<Arc<TensorPool>>) -> CostMapDescriptor {
        // Retain BOTH tensors so they survive until all receivers drop
        self.pool().retain(self.descriptor().grid_tensor());
        self.pool().retain(self.descriptor().cost_tensor());
        *self.descriptor()
    }

    #[inline]
    fn from_wire(wire: CostMapDescriptor, pool: &Option<Arc<TensorPool>>) -> Self {
        let p = pool.as_ref().cloned().unwrap_or_else(global_pool);
        CostMap::from_owned(wire, p)
    }

    #[inline]
    fn try_from_wire(wire: CostMapDescriptor, pool: &Option<Arc<TensorPool>>) -> Option<Self> {
        let p = pool.as_ref().cloned().unwrap_or_else(global_pool);
        // Dual-tensor: retain BOTH. If the second is already stale, unwind the
        // first so we don't leak it, and report the message as missed.
        if p.try_retain(wire.grid_tensor()).is_err() {
            return None;
        }
        if p.try_retain(wire.cost_tensor()).is_err() {
            p.release(wire.grid_tensor());
            return None;
        }
        Some(CostMap::from_owned(wire, p))
    }

    #[inline(always)]
    fn needs_pool() -> bool {
        true
    }
}

// ============================================================================
// TensorHandle: Generic user tensor — Wire = Tensor descriptor (168B)
// ============================================================================

impl TopicMessage for TensorHandle {
    type Wire = Tensor;

    #[inline]
    fn to_wire(&self, _pool: &Option<Arc<TensorPool>>) -> Tensor {
        self.pool().retain(self.tensor());
        *self.tensor()
    }

    #[inline]
    fn from_wire(wire: Tensor, pool: &Option<Arc<TensorPool>>) -> Self {
        let p = pool.as_ref().cloned().unwrap_or_else(global_pool);
        TensorHandle::new(wire, p)
    }

    #[inline]
    fn try_from_wire(wire: Tensor, pool: &Option<Arc<TensorPool>>) -> Option<Self> {
        let p = pool.as_ref().cloned().unwrap_or_else(global_pool);
        if p.try_retain(&wire).is_err() {
            return None;
        }
        // `from_owned` does not retain (the `try_retain` above is our reference)
        // and only errors on a pool-id mismatch, which `try_retain` already
        // rejected — so this is always `Some`.
        TensorHandle::from_owned(wire, p).ok()
    }

    #[inline(always)]
    fn needs_pool() -> bool {
        true
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::memory::image::Image;
    use crate::types::ImageEncoding;

    // Regression gate for the multi-subscriber zero-copy use-after-free
    // (deep-research audit COMM-H2, 2026-07-13): a pool-backed `Topic<Image>`
    // with N subscribers must keep the slot alive until EVERY subscriber drops.
    // The send path (`to_wire`) retains the slot exactly once, but each receiver
    // releases on drop — so the first receiver to drop frees the slot out from
    // under the others. Asserts on refcount only (never dereferences the freed
    // slot), so on the buggy code it fails with a clean assertion, not a crash.
    // Regression gate for COMM-H2 (deep-research audit 2026-07-13): a pool-backed
    // Topic<Image> with N subscribers keeps the slot alive until EVERY subscriber
    // drops. Before the fix this failed "refcount = 1 < 2"; the recv path now takes
    // a per-subscriber generation-guarded reference via `try_from_wire`.
    #[test]
    fn image_slot_survives_until_all_subscribers_drop() {
        let img = Image::new(8, 8, ImageEncoding::Rgb8).expect("alloc image");
        let pool = img.pool().clone();
        let slot = *img.descriptor().tensor();

        // Producer sends (retain once), then its own handle drops (release once).
        let wire = img.to_wire(&Some(pool.clone()));
        drop(img);

        // Two subscribers reconstruct the message via the recv path.
        let r1 = Image::try_from_wire(wire, &Some(pool.clone())).expect("slot live");
        let r2 = Image::try_from_wire(wire, &Some(pool.clone())).expect("slot live");

        // INVARIANT: N live receivers => slot refcount >= N. Today it is 1 (BUG),
        // because `to_wire` retains once while each receiver releases on drop.
        let rc = pool.refcount(&slot);
        assert!(
            rc >= 2,
            "two live subscribers but slot refcount = {rc} (< 2): the send path \
             retains once while each receiver releases on drop"
        );

        // Dropping one subscriber must not free the slot the other still holds.
        drop(r1);
        let rc_after = pool.refcount(r2.descriptor().tensor());
        assert!(
            rc_after >= 1,
            "surviving subscriber holds a freed slot (refcount = {rc_after}): use-after-free"
        );
        drop(r2);
    }

    // Send-side companion to the UAF gate: the producer's keep-alive for a
    // superseded (unread) message is released on the next send, so a stream of
    // sends does not leak pool slots. Before the fix, `to_wire`'s retain was
    // never balanced for multi-subscriber topics and each send leaked a slot.
    /// A topic name no other run, process or test shares.
    ///
    /// These two tests used fixed names. A SHM topic outlives the process that
    /// created it, so a fixed name means a previous run's region — possibly
    /// with an incompatible geometry — is what the next run attaches to. That
    /// is not hypothetical in this repo: a topic name shared between two
    /// message types is what produces `signal: 7, SIGBUS` in #144, and a
    /// leftover region of the wrong shape produces the "shared header declares
    /// slot_size N" refusal. Neither failure names the test that caused it.
    fn unique_topic(suffix: &str) -> String {
        use std::sync::atomic::{AtomicU64, Ordering};
        static N: AtomicU64 = AtomicU64::new(0);
        format!(
            "test.comm_h2.keepalive.{suffix}.{}.{}",
            std::process::id(),
            N.fetch_add(1, Ordering::Relaxed)
        )
    }

    #[test]
    fn image_topic_holds_keepalives_for_the_whole_ring_then_releases() {
        use crate::communication::topic::Topic;
        const CAP: u32 = 4;
        let name = unique_topic("bound");
        let topic = Topic::<Image>::with_capacity(&name, CAP, None).expect("topic");

        let a = Image::new(8, 8, ImageEncoding::Rgb8).expect("alloc a");
        let pool = a.pool().clone();
        let slot_a = *a.descriptor().tensor();
        topic.send(a);

        // The next send must NOT release A. A is still sitting unread in the
        // ring, and freeing its backing is what made `recv()` return None on a
        // non-empty ring: a subscriber polling slower than the publisher got
        // one frame in five hundred, with every counter reading zero.
        //
        // This assertion used to be the opposite (`refcount == 0` right here),
        // which encoded the depth-1 keep-alive as a requirement. The concern
        // behind it — that the producer must not leak pool slots — is real and
        // is what the rest of this test now covers.
        topic.send(Image::new(8, 8, ImageEncoding::Rgb8).expect("alloc b"));
        assert!(
            pool.refcount(&slot_a) >= 1,
            "A is still readable in the ring, so its backing must stay alive \
             (refcount {})",
            pool.refcount(&slot_a)
        );

        // Past the ring's depth A can no longer be reached, so holding it would
        // be a leak. Retention is bounded by capacity, not unbounded.
        for _ in 0..CAP {
            topic.send(Image::new(8, 8, ImageEncoding::Rgb8).expect("alloc filler"));
        }
        assert_eq!(
            pool.refcount(&slot_a),
            0,
            "once the ring has lapped past A the producer must release it, or \
             every send leaks a slot"
        );
    }

    /// The bug this file's keep-alive machinery caused: frames sent while the
    /// subscriber was not draining became unreadable, and `recv()` returned
    /// None on a non-empty ring.
    #[test]
    fn image_frames_survive_until_the_subscriber_drains_them() {
        use crate::communication::topic::Topic;
        const CAP: u32 = 8;
        const SENT: usize = 5;

        // ONE name for both handles -- they must meet on the same topic -- but a
        // name no other run shares.
        let name = unique_topic("drain");
        let tx = Topic::<Image>::with_capacity(&name, CAP, None).expect("tx");
        let rx = Topic::<Image>::with_capacity(&name, CAP, None).expect("rx");

        for i in 0..SENT {
            let img = Image::new(8, 8, ImageEncoding::Rgb8).expect("alloc");
            img.data_mut()[0] = i as u8;
            tx.send(img);
        }

        let mut got = 0;
        while rx.recv().is_some() {
            got += 1;
        }

        assert_eq!(
            got,
            SENT,
            "{SENT} frames were sent into a {CAP}-slot ring and never lapped, so \
             all {SENT} must be readable. Got {got}, missed={}, dropped={}. A \
             depth-1 keep-alive freed each frame's backing on the next send, so \
             recv() returned None while the ring was non-empty — and because \
             the ring dropped nothing and lapped nobody, no counter could see it.",
            rx.missed_count(),
            tx.dropped_count()
        );
    }
}
