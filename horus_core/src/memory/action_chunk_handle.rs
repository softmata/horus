//! Pool-backed ownership for [`ActionChunk`].
//!
//! # The problem this exists for
//!
//! [`ActionChunk`] is `Pod`, so it routes to the zero-copy topic path and a
//! `send()` puts its 256-byte descriptor on the wire. The descriptor NAMES a
//! pool slot; it does not own one. Nothing on that path retains the slot, so a
//! chunk published with a bare `send()` can have its action data recycled
//! under a subscriber that is still holding the descriptor — and the failure
//! is silent, because the slot is valid memory holding somebody else's numbers.
//!
//! On the seam this type is for — a Python policy publishing at 10-50 Hz into
//! a servo loop reading at 500-1000 Hz — the subscriber holds each chunk for
//! tens of its own periods. That is exactly the window where the producer
//! publishes again and recycles the slot.
//!
//! `ActionChunkHandle` closes it the same way [`TensorHandle`] does for
//! tensors: the handle owns a reference on the slot, the reference is
//! transferred to the receiver by the send, and dropping the handle releases
//! it. The refcounting itself is delegated to `TensorHandle` rather than
//! reimplemented — there is one refcount discipline in this crate and this is
//! not a second one.

use crate::error::HorusResult;
use crate::memory::{TensorHandle, TensorPool};
use crate::types::{ActionAt, ActionChunk, Device, TensorDtype};
use std::sync::Arc;

/// An [`ActionChunk`] together with a live reference on the pool slot holding
/// its actions.
///
/// Deref-like accessors are deliberately absent: the interesting operations
/// ([`locate`](Self::locate), [`sample_into`](Self::sample_into)) need the
/// action data as well as the descriptor, and exposing a bare `&ActionChunk`
/// invites callers to `send()` it and lose the reference this type holds.
#[derive(Debug)]
pub struct ActionChunkHandle {
    chunk: ActionChunk,
    tensor: TensorHandle,
}

impl ActionChunkHandle {
    /// Wrap a chunk whose slot reference this handle is taking ownership of.
    ///
    /// The caller must already hold a reference on `chunk.tensor()` — either
    /// from an allocation or from a send that transferred one. This does not
    /// take its own.
    pub fn from_owned(chunk: ActionChunk, tensor: TensorHandle) -> HorusResult<Self> {
        if tensor.tensor().pool_id != chunk.tensor().pool_id {
            return Err(crate::error::Error::Internal {
                message: format!(
                    "ActionChunkHandle::from_owned: chunk names pool {} but the handle holds \
                     a reference in pool {}; releasing this handle would decrement the wrong \
                     refcount",
                    chunk.tensor().pool_id,
                    tensor.tensor().pool_id
                ),
                file: file!(),
                line: line!(),
            });
        }
        Ok(Self { chunk, tensor })
    }

    /// Allocate a `[horizon, action_dim]` chunk from `pool`.
    ///
    /// `t0_ns` is when the first action applies and `dt_ns` the spacing
    /// between them; both are what [`locate`](Self::locate) reads to decide
    /// whether a query lands before, inside, or past the chunk.
    pub fn alloc(
        pool: Arc<TensorPool>,
        horizon: u32,
        action_dim: u32,
        dtype: TensorDtype,
        device: Device,
        t0_ns: u64,
        dt_ns: u64,
    ) -> HorusResult<Self> {
        let tensor =
            TensorHandle::alloc(pool, &[horizon as u64, action_dim as u64], dtype, device)?;
        let chunk = ActionChunk::new(*tensor.tensor(), t0_ns, dt_ns);
        Ok(Self { chunk, tensor })
    }

    /// The chunk descriptor.
    pub fn chunk(&self) -> &ActionChunk {
        &self.chunk
    }

    /// The chunk descriptor, mutably — for re-stamping `t0_ns`/`seq` before a
    /// send without reallocating the actions.
    pub fn chunk_mut(&mut self) -> &mut ActionChunk {
        &mut self.chunk
    }

    /// The handle holding this chunk's slot reference.
    pub fn tensor(&self) -> &TensorHandle {
        &self.tensor
    }

    /// The pool the actions live in.
    pub fn pool(&self) -> &Arc<TensorPool> {
        self.tensor.pool()
    }

    /// Where `query_ns` falls relative to this chunk.
    ///
    /// Touches only the descriptor, never the pool — safe to call at servo
    /// rate on an RT thread.
    pub fn locate(&self, query_ns: u64) -> ActionAt {
        self.chunk.locate(query_ns)
    }

    /// Whether this chunk's actions are actually stored as `f32`.
    ///
    /// Checked explicitly because `bytemuck` will not catch it. A cast from
    /// `&[u8]` to `&[f32]` only fails on alignment or a length that is not a
    /// multiple of four — an `f64` buffer satisfies both, so the cast SUCCEEDS
    /// and reinterprets each pair of `f64` halves as two `f32`s. The numbers
    /// that come out are garbage, and they are actuator commands.
    fn is_f32(&self) -> bool {
        self.tensor.dtype() == TensorDtype::F32
    }

    /// Interpolate the action at `query_ns` into `out`.
    ///
    /// `out` is written if and only if [`ActionAt::Inside`] is returned — a
    /// stale chunk never extrapolates, which is the property the whole type
    /// exists to preserve. Returns [`ActionAt::Malformed`] if the chunk is not
    /// `f32` or the pooled data cannot be read.
    pub fn sample_into(&self, query_ns: u64, out: &mut [f32]) -> ActionAt {
        if !self.is_f32() {
            return ActionAt::Malformed;
        }
        let Ok(bytes) = self.tensor.data_slice() else {
            return ActionAt::Malformed;
        };
        let Ok(actions) = bytemuck::try_cast_slice::<u8, f32>(bytes) else {
            return ActionAt::Malformed;
        };
        self.chunk.sample_into(query_ns, actions, out)
    }

    /// The chunk's actions as `f32`, or `None` if this chunk is not `f32` or
    /// the pooled data cannot be read.
    pub fn actions_f32(&self) -> Option<&[f32]> {
        if !self.is_f32() {
            return None;
        }
        let bytes = self.tensor.data_slice().ok()?;
        bytemuck::try_cast_slice::<u8, f32>(bytes).ok()
    }

    /// The chunk's actions as mutable `f32`, for filling after an allocation.
    // `mut_from_ref`: the pool slot is shared memory with interior mutability,
    // so a `&self` receiver handing out `&mut` is the established shape here —
    // `OccupancyGrid::cells_mut` and the other pool-backed types do the same.
    // The handle owns a reference on the slot for as long as it lives, and the
    // caller is the only writer during an allocation-then-fill.
    #[allow(clippy::mut_from_ref)]
    pub fn actions_f32_mut(&self) -> Option<&mut [f32]> {
        if !self.is_f32() {
            return None;
        }
        let bytes = self.tensor.data_slice_mut().ok()?;
        bytemuck::try_cast_slice_mut::<u8, f32>(bytes).ok()
    }
}
