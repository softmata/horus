//! Pod action-chunk descriptor — a policy-rate trajectory read at servo rate.
//!
//! A learned policy (VLA, diffusion policy, ACT) does not emit one setpoint per
//! control tick.  It emits a *chunk*: `horizon` future actions spaced `dt_ns`
//! apart, published at 5–30 Hz.  The servo loop runs at 500–1000 Hz, so between
//! two chunks it must interpolate — and, when the policy is late or has stopped
//! publishing, it must find out that the chunk it is holding has run out.
//!
//! Both halves live here.  [`ActionChunk`] is the fixed-size (256 byte)
//! `repr(C)` descriptor that flows through the ring buffer on the ~50 ns Pod
//! path, carrying a [`Tensor`] handle to the `[horizon, action_dim]` array in a
//! `TensorPool`.  [`ActionChunk::locate`] is the consumer: given a query time it
//! reports where that time falls, in O(1) with no allocation.
//!
//! # The case this exists for
//!
//! ```text
//! t0            t0+dt         t0+2dt        t0+3dt = end
//! |-------------|-------------|-------------|- - - - - - - ->
//!    Inside         Inside        Inside     ^ Stale from here on
//! ```
//!
//! A query past `end` is [`ActionAt::Stale`], never an extrapolated action.
//! Extrapolating is how an arm keeps tracking an expired trajectory after the
//! policy has died: the setpoints keep arriving, they are invented, and nothing
//! upstream is still closing the loop.  This module refuses to produce them, and
//! writes nothing into the caller's output buffer unless the query genuinely
//! lands inside the chunk.
//!
//! ```rust,ignore
//! // 1 kHz servo tick
//! let mut action = [0.0f32; 7];
//! match chunk.sample_into(now_ns, joints, &mut action) {
//!     ActionAt::Inside { .. } => arm.command(&action),
//!     ActionAt::Stale { by_ns } => arm.hold_and_fault(by_ns), // policy is late
//!     ActionAt::Before { .. } => {}                           // not started yet
//!     ActionAt::Malformed => arm.estop(),
//! }
//! ```

use bytemuck::{Pod, Zeroable};
use serde::{Deserialize, Serialize};

use super::tensor::Tensor;

/// Where a query time falls relative to an [`ActionChunk`].
///
/// Returned by [`ActionChunk::locate`] and by the `sample_*` methods.  The
/// contract the servo loop depends on: **the output buffer is written if and
/// only if `Inside` is returned.**  Every other variant leaves it untouched, so
/// a caller that mishandles one cannot end up commanding an invented value — it
/// commands whatever it had, which is its own last decision.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum ActionAt {
    /// The query is before `t0`: the chunk has not started yet.
    Before {
        /// Nanoseconds until the chunk starts.
        by_ns: u64,
    },

    /// The query falls inside the chunk and can be interpolated.
    ///
    /// `lo` and `hi` are row indices into the `[horizon, action_dim]` array and
    /// `alpha` is the blend in `[0, 1)`: `action = row[lo] + (row[hi] -
    /// row[lo]) * alpha`.  Exactly on a knot — and on the final action, where
    /// `hi == lo` because there is no next row — `alpha` is `0.0`, so the
    /// stored value is reproduced bit for bit rather than approached.
    Inside {
        /// Index of the action at or before the query.
        lo: u32,
        /// Index of the action after the query, or `lo` at the chunk's end.
        hi: u32,
        /// Blend factor in `[0, 1)` between `lo` and `hi`.
        alpha: f32,
    },

    /// **STALE** — the query is past the last action in the chunk.
    ///
    /// The safety-relevant case.  The policy has not published in time, and
    /// nothing in this chunk describes what the robot should do now.  `by_ns`
    /// is how far past the end the query is, which is the number a watchdog
    /// wants: a few hundred microseconds is jitter, tens of milliseconds is a
    /// dead policy.
    Stale {
        /// Nanoseconds past the chunk's last action.
        by_ns: u64,
    },

    /// The descriptor does not describe a usable chunk, or the buffers handed
    /// to a `sample_*` call are too small for the shape it declares.
    ///
    /// Specifically: `horizon == 0`; a time span `(horizon - 1) * dt_ns` that
    /// overflows `u64`; `action_dim == 0`; an action buffer shorter than
    /// `horizon * action_dim`; or an output buffer shorter than `action_dim`.
    /// A descriptor read out of peer-writable shared memory can be any of
    /// these, so this is a state to handle, not an assertion to make.
    Malformed,
}

impl ActionAt {
    /// Whether the query ran past the end of the chunk.
    #[inline]
    pub const fn is_stale(&self) -> bool {
        matches!(self, ActionAt::Stale { .. })
    }

    /// Whether the query landed inside the chunk — equivalently, whether a
    /// `sample_*` call wrote an action.
    #[inline]
    pub const fn is_inside(&self) -> bool {
        matches!(self, ActionAt::Inside { .. })
    }
}

/// Action chunk descriptor — Pod, 256 bytes.
///
/// Contains a [`Tensor`] (shape `[horizon, action_dim]`, data in a `TensorPool`)
/// plus the timing that turns those rows into a trajectory: `t0_ns` when the
/// first action applies, `dt_ns` between consecutive actions, and `seq` so a
/// consumer can tell a re-published chunk from a new one.
///
/// The chunk is valid over `[t0_ns, t0_ns + (horizon - 1) * dt_ns]` — it ends at
/// the *last action*, not one `dt` after it.  With `horizon == 1` that is a
/// single instant, because one point cannot be interpolated; treating it as
/// valid for a further `dt` would be a zero-order hold, which is extrapolation
/// wearing a hat.
///
/// # Layout (256 bytes, repr(C))
///
/// ```text
/// inner:       Tensor       (168 bytes)
/// t0_ns:       u64            (8 bytes)
/// dt_ns:       u64            (8 bytes)
/// seq:         u64            (8 bytes)
/// horizon:     u32            (4 bytes)
/// action_dim:  u32            (4 bytes)
/// frame_id:    [u8; 32]      (32 bytes)
/// _reserved:   [u8; 24]      (24 bytes)
/// Total:                     256 bytes
/// ```
#[repr(C)]
#[derive(Clone, Copy, Debug, Serialize, Deserialize, Default)]
pub struct ActionChunk {
    /// Inner tensor: shape `[horizon, action_dim]`, data in pool
    inner: Tensor,
    /// Time the first action applies, nanoseconds since epoch
    t0_ns: u64,
    /// Spacing between consecutive actions, nanoseconds
    dt_ns: u64,
    /// Monotonic chunk counter from the publisher
    seq: u64,
    /// Number of actions in the chunk (rows of `inner`)
    horizon: u32,
    /// Elements per action (columns of `inner`)
    action_dim: u32,
    /// Frame or controller identifier (null-terminated)
    frame_id: [u8; 32],
    #[serde(skip)]
    _reserved: [u8; 24],
}

// Safety: ActionChunk is repr(C); every field is a plain integer, an array of
// them, or `Tensor` (itself Pod), so every one of the 256 possible values of
// every byte is a valid value — which is what `Pod` requires of a struct read
// byte-for-byte out of peer-writable shared memory.  There is no implicit
// padding: 168 + 8 + 8 + 8 + 4 + 4 + 32 + 24 = 256, and 256 % 8 == 0.
unsafe impl Zeroable for ActionChunk {}
unsafe impl Pod for ActionChunk {}

/// Narrow a tensor dimension to the `u32` the descriptor stores, saturating
/// rather than truncating — a shape of `2^32 + 1` must not become `1`.
#[inline]
const fn dim_as_u32(dim: u64) -> u32 {
    if dim > u32::MAX as u64 {
        u32::MAX
    } else {
        dim as u32
    }
}

impl ActionChunk {
    /// Create a chunk descriptor over a `[horizon, action_dim]` tensor.
    ///
    /// `horizon` and `action_dim` are taken from the tensor's shape: a 2-D
    /// tensor gives both, a 1-D tensor is read as `horizon` actions of one
    /// element each, and a 0-D (default/zeroed) tensor gives a chunk that every
    /// query reports [`ActionAt::Malformed`] for.
    pub fn new(tensor: Tensor, t0_ns: u64, dt_ns: u64) -> Self {
        let (horizon, action_dim) = Self::shape_of(&tensor);
        Self {
            inner: tensor,
            t0_ns,
            dt_ns,
            seq: 0,
            horizon,
            action_dim,
            frame_id: [0; 32],
            _reserved: [0; 24],
        }
    }

    /// `(horizon, action_dim)` implied by a tensor's shape.
    #[inline]
    const fn shape_of(tensor: &Tensor) -> (u32, u32) {
        match tensor.ndim {
            0 => (0, 0),
            1 => (dim_as_u32(tensor.shape[0]), 1),
            _ => (dim_as_u32(tensor.shape[0]), dim_as_u32(tensor.shape[1])),
        }
    }

    /// Sanitize a chunk read from untrusted bytes (SHM, network, file).
    ///
    /// Clamps the inner tensor's fields, then clamps `horizon` and `action_dim`
    /// so the descriptor cannot claim more rows or wider actions than its own
    /// tensor describes.  This is a consistency clamp only — the shape it
    /// clamps against came off the same wire.  The check that actually bounds a
    /// read is in the `sample_*` methods, which measure the caller's real
    /// buffer.
    #[inline]
    pub fn sanitize_from_shm(&mut self) {
        self.inner.sanitize_from_shm();
        let (horizon, action_dim) = Self::shape_of(&self.inner);
        if self.horizon > horizon {
            self.horizon = horizon;
        }
        if self.action_dim > action_dim {
            self.action_dim = action_dim;
        }
    }

    // === Metadata accessors ===

    /// Time the first action applies, nanoseconds since epoch.
    #[inline]
    pub const fn t0_ns(&self) -> u64 {
        self.t0_ns
    }

    /// Re-stamp the chunk's start time.
    #[inline]
    pub fn set_t0_ns(&mut self, t0_ns: u64) {
        self.t0_ns = t0_ns;
    }

    /// Spacing between consecutive actions, nanoseconds.
    #[inline]
    pub const fn dt_ns(&self) -> u64 {
        self.dt_ns
    }

    /// Number of actions in the chunk.
    #[inline]
    pub const fn horizon(&self) -> u32 {
        self.horizon
    }

    /// Elements per action.
    #[inline]
    pub const fn action_dim(&self) -> u32 {
        self.action_dim
    }

    /// Publisher's chunk counter.
    #[inline]
    pub const fn seq(&self) -> u64 {
        self.seq
    }

    /// Set the publisher's chunk counter.
    #[inline]
    pub fn set_seq(&mut self, seq: u64) {
        self.seq = seq;
    }

    /// Time of the chunk's **last** action, or `None` if the chunk is unusable
    /// (see [`ActionAt::Malformed`]).
    ///
    /// This is the instant the chunk goes stale, not one `dt` later.
    #[inline]
    pub const fn end_ns(&self) -> Option<u64> {
        if self.horizon == 0 {
            return None;
        }
        let span = match ((self.horizon - 1) as u64).checked_mul(self.dt_ns) {
            Some(span) => span,
            None => return None,
        };
        self.t0_ns.checked_add(span)
    }

    /// The `[start, end)` element range of `row` inside a flat
    /// `horizon * action_dim` buffer, or `None` if the row is out of range or
    /// the offsets overflow `usize`.
    ///
    /// Exposed so a caller that wants a specific action — the last one, to hold
    /// position after a chunk goes stale — does not have to re-derive the index
    /// arithmetic this module exists to own.
    #[inline]
    pub const fn row_range(&self, row: u32) -> Option<core::ops::Range<usize>> {
        if row >= self.horizon || self.action_dim == 0 {
            return None;
        }
        let dim = self.action_dim as usize;
        let start = match (row as usize).checked_mul(dim) {
            Some(start) => start,
            None => return None,
        };
        match start.checked_add(dim) {
            Some(end) => Some(start..end),
            None => None,
        }
    }

    // === Consumer: the servo-rate query path ===

    /// Where `query_ns` falls in this chunk.
    ///
    /// O(1), branch-only, allocation-free — this is the 1 kHz path.  It touches
    /// only the descriptor, never the pooled action data, so a servo loop can
    /// decide whether it is still being commanded before it resolves a pointer
    /// into shared memory.
    ///
    /// Past the last action the answer is [`ActionAt::Stale`].  It is never an
    /// extrapolated index.
    #[inline]
    pub const fn locate(&self, query_ns: u64) -> ActionAt {
        if self.horizon == 0 {
            return ActionAt::Malformed;
        }
        let last = (self.horizon - 1) as u64;

        // An overflowing span is a corrupt descriptor, and the safe reading of
        // one is "unusable", not "valid until the end of time".
        let span = match last.checked_mul(self.dt_ns) {
            Some(span) => span,
            None => return ActionAt::Malformed,
        };
        let end_ns = match self.t0_ns.checked_add(span) {
            Some(end_ns) => end_ns,
            None => return ActionAt::Malformed,
        };

        if query_ns < self.t0_ns {
            return ActionAt::Before {
                by_ns: self.t0_ns - query_ns,
            };
        }
        if query_ns > end_ns {
            return ActionAt::Stale {
                by_ns: query_ns - end_ns,
            };
        }

        let since = query_ns - self.t0_ns;
        if self.dt_ns == 0 {
            // Degenerate: every action is stamped at t0, so t0 is the only
            // instant inside the chunk and row 0 is as good an answer as any.
            return ActionAt::Inside {
                lo: 0,
                hi: 0,
                alpha: 0.0,
            };
        }

        let lo = since / self.dt_ns;
        if lo >= last {
            // Exactly on the final action.  There is no row after it to
            // interpolate towards, so the chunk ends here rather than opening
            // another interval.
            return ActionAt::Inside {
                lo: last as u32,
                hi: last as u32,
                alpha: 0.0,
            };
        }
        let rem = since % self.dt_ns;
        ActionAt::Inside {
            lo: lo as u32,
            hi: (lo + 1) as u32,
            alpha: rem as f32 / self.dt_ns as f32,
        }
    }

    /// Interpolate the action at `query_ns` into `out`.
    ///
    /// `actions` is the flat `horizon * action_dim` array the descriptor's
    /// tensor points at, resolved from the pool once by the caller — the query
    /// path itself never touches the pool.  `out` receives `action_dim`
    /// elements, and **only** when the return value is [`ActionAt::Inside`].
    /// Stale, early and malformed queries leave `out` exactly as it was: an
    /// expired chunk must not silently write a setpoint.
    ///
    /// O(`action_dim`), allocation-free, no panicking path — a short or
    /// mismatched buffer returns [`ActionAt::Malformed`] rather than indexing
    /// past its end.
    #[inline]
    pub fn sample_into(&self, query_ns: u64, actions: &[f32], out: &mut [f32]) -> ActionAt {
        let at = self.locate(query_ns);
        let (lo, hi, alpha) = match at {
            ActionAt::Inside { lo, hi, alpha } => (lo, hi, alpha),
            other => return other,
        };
        let (Some(lo_range), Some(hi_range)) = (self.row_range(lo), self.row_range(hi)) else {
            return ActionAt::Malformed;
        };
        if out.len() < self.action_dim as usize {
            return ActionAt::Malformed;
        }
        let (Some(a), Some(b)) = (actions.get(lo_range), actions.get(hi_range)) else {
            return ActionAt::Malformed;
        };
        for (slot, (&a, &b)) in out.iter_mut().zip(a.iter().zip(b.iter())) {
            // a + (b - a) * alpha, not (1 - alpha) * a + alpha * b: at alpha 0
            // this reproduces `a` bit for bit, which is what makes the knots
            // and the chunk's final action exact.
            *slot = a + (b - a) * alpha;
        }
        at
    }

    /// [`sample_into`](Self::sample_into) for `f64` action data.
    ///
    /// Same contract, same guarantees.  The blend factor is computed in `f32`
    /// (better than a nanosecond of timing resolution across a 10 ms interval),
    /// which is far below the jitter of any real control loop.
    #[inline]
    pub fn sample_f64_into(&self, query_ns: u64, actions: &[f64], out: &mut [f64]) -> ActionAt {
        let at = self.locate(query_ns);
        let (lo, hi, alpha) = match at {
            ActionAt::Inside { lo, hi, alpha } => (lo, hi, alpha as f64),
            other => return other,
        };
        let (Some(lo_range), Some(hi_range)) = (self.row_range(lo), self.row_range(hi)) else {
            return ActionAt::Malformed;
        };
        if out.len() < self.action_dim as usize {
            return ActionAt::Malformed;
        }
        let (Some(a), Some(b)) = (actions.get(lo_range), actions.get(hi_range)) else {
            return ActionAt::Malformed;
        };
        for (slot, (&a, &b)) in out.iter_mut().zip(a.iter().zip(b.iter())) {
            *slot = a + (b - a) * alpha;
        }
        at
    }

    crate::impl_tensor_accessors!();
    crate::impl_frame_id_field!();
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::types::{Device, TensorDtype};

    const T0: u64 = 1_000_000_000;
    const DT: u64 = 20_000_000; // 50 Hz policy
    const HORIZON: u64 = 4;
    const DIM: u64 = 3;

    /// A 4 x 3 chunk whose rows are `(10k, 20k, 30k)` for `k` in `0..4`, so any
    /// interpolation error is visible as a plain number.
    fn chunk() -> (ActionChunk, Vec<f32>) {
        let tensor = Tensor::new(1, 0, 0, 0, &[HORIZON, DIM], TensorDtype::F32, Device::cpu());
        let actions: Vec<f32> = (0..HORIZON)
            .flat_map(|k| [10.0 * k as f32, 20.0 * k as f32, 30.0 * k as f32])
            .collect();
        (ActionChunk::new(tensor, T0, DT), actions)
    }

    #[test]
    fn action_chunk_is_two_hundred_fifty_six_bytes() {
        let size = std::mem::size_of::<ActionChunk>();
        let align = std::mem::align_of::<ActionChunk>();
        assert_eq!(size, 256, "ActionChunk must be exactly 256 bytes");
        assert_eq!(
            align, 8,
            "alignment comes from the u64s in the inner Tensor"
        );
        assert_eq!(size % align, 0, "no trailing padding");
        assert_eq!(size % 64, 0, "a whole number of cache lines");
    }

    /// The reason this module exists.  A servo loop asking for an action after
    /// the chunk's last one must be told the chunk is spent, not handed an
    /// invented setpoint — an arm tracking an extrapolated trajectory is
    /// running open-loop on numbers no policy ever produced.
    #[test]
    fn a_query_past_the_last_action_is_stale_and_writes_nothing() {
        let (chunk, actions) = chunk();
        let end = T0 + (HORIZON - 1) * DT;

        assert_eq!(
            chunk.locate(end + 1),
            ActionAt::Stale { by_ns: 1 },
            "one nanosecond past the last action is already outside the chunk"
        );
        assert_eq!(
            chunk.locate(end + 500_000_000),
            ActionAt::Stale { by_ns: 500_000_000 },
            "half a second past the end must report how far past, not clamp"
        );

        // And nothing is written: the caller keeps whatever it last commanded.
        let sentinel = [-1.0f32, -2.0, -3.0];
        let mut out = sentinel;
        let at = chunk.sample_into(end + 1, &actions, &mut out);
        assert!(at.is_stale(), "expected Stale, got {at:?}");
        assert_eq!(
            out, sentinel,
            "a stale query must leave the output buffer untouched"
        );
    }

    /// Every action in the chunk must come back bit for bit at its own
    /// timestamp.  An interpolator that only approximates its own knots drifts
    /// the trajectory it was given.
    #[test]
    fn every_knot_reproduces_its_stored_action_exactly() {
        let (chunk, actions) = chunk();
        for k in 0..HORIZON {
            let mut out = [0.0f32; DIM as usize];
            let at = chunk.sample_into(T0 + k * DT, &actions, &mut out);
            assert!(at.is_inside(), "knot {k} landed outside the chunk: {at:?}");
            assert_eq!(
                out,
                [10.0 * k as f32, 20.0 * k as f32, 30.0 * k as f32],
                "knot {k} was not reproduced exactly"
            );
        }
    }

    #[test]
    fn a_midpoint_query_blends_its_two_neighbours() {
        let (chunk, actions) = chunk();
        let mut out = [0.0f32; DIM as usize];

        let at = chunk.sample_into(T0 + DT / 2, &actions, &mut out);
        assert_eq!(
            at,
            ActionAt::Inside {
                lo: 0,
                hi: 1,
                alpha: 0.5
            }
        );
        assert_eq!(out, [5.0, 10.0, 15.0]);

        // A quarter of the way into the third interval.
        let at = chunk.sample_into(T0 + 2 * DT + DT / 4, &actions, &mut out);
        assert_eq!(
            at,
            ActionAt::Inside {
                lo: 2,
                hi: 3,
                alpha: 0.25
            }
        );
        assert_eq!(out, [22.5, 45.0, 67.5]);
    }

    /// The chunk ends *at* its last action, not one dt after it.  If the final
    /// knot opened another interval, `hi` would index row `horizon`, which is
    /// past the end of the pooled array.
    #[test]
    fn the_final_action_closes_the_chunk_rather_than_opening_an_interval() {
        let (chunk, actions) = chunk();
        let end = T0 + (HORIZON - 1) * DT;

        assert_eq!(
            chunk.locate(end),
            ActionAt::Inside {
                lo: 3,
                hi: 3,
                alpha: 0.0
            },
            "the last action must interpolate against itself, not against row 4"
        );
        assert_eq!(chunk.end_ns(), Some(end));

        let mut out = [0.0f32; DIM as usize];
        chunk.sample_into(end, &actions, &mut out);
        assert_eq!(out, [30.0, 60.0, 90.0]);
    }

    #[test]
    fn a_query_before_the_chunk_starts_is_reported_and_writes_nothing() {
        let (chunk, actions) = chunk();
        assert_eq!(chunk.locate(T0 - 5), ActionAt::Before { by_ns: 5 });

        let mut out = [7.0f32; DIM as usize];
        let at = chunk.sample_into(T0 - 5, &actions, &mut out);
        assert_eq!(at, ActionAt::Before { by_ns: 5 });
        assert_eq!(
            out, [7.0; DIM as usize],
            "an unstarted chunk must not write a setpoint either"
        );
    }

    /// A single action cannot be interpolated, so it is valid at exactly one
    /// instant.  Holding it for a further dt would be a zero-order hold, which
    /// is extrapolation by another name.
    #[test]
    fn a_one_action_chunk_is_valid_only_at_its_own_instant() {
        let tensor = Tensor::new(1, 0, 0, 0, &[1, DIM], TensorDtype::F32, Device::cpu());
        let chunk = ActionChunk::new(tensor, T0, DT);
        assert_eq!(
            chunk.locate(T0),
            ActionAt::Inside {
                lo: 0,
                hi: 0,
                alpha: 0.0
            }
        );
        assert_eq!(chunk.locate(T0 + 1), ActionAt::Stale { by_ns: 1 });
    }

    /// Descriptors arrive from peer-writable shared memory, so every one of
    /// these is reachable without a bug anywhere in HORUS.
    #[test]
    fn a_corrupt_descriptor_is_malformed_rather_than_believed() {
        // No actions at all.
        let empty = ActionChunk::new(Tensor::default(), T0, DT);
        assert_eq!(empty.horizon(), 0);
        assert_eq!(empty.locate(T0), ActionAt::Malformed);
        assert_eq!(empty.end_ns(), None);

        // A span that overflows u64.  The unsafe reading of this is "valid
        // forever"; the safe one is "unusable".
        let tensor = Tensor::new(1, 0, 0, 0, &[4, DIM], TensorDtype::F32, Device::cpu());
        let huge = ActionChunk::new(tensor, T0, u64::MAX / 2);
        assert_eq!(huge.locate(u64::MAX), ActionAt::Malformed);
        assert_eq!(huge.end_ns(), None);

        // horizon claiming more rows than the tensor holds is clamped back.
        let tensor = Tensor::new(1, 0, 0, 0, &[4, DIM], TensorDtype::F32, Device::cpu());
        let mut lying = ActionChunk::new(tensor, T0, DT);
        lying.horizon = 4096;
        lying.action_dim = 99;
        lying.sanitize_from_shm();
        assert_eq!(lying.horizon(), 4);
        assert_eq!(lying.action_dim(), DIM as u32);
    }

    #[test]
    fn a_short_action_buffer_is_refused_rather_than_read_past_its_end() {
        let (chunk, actions) = chunk();
        let mut out = [0.0f32; DIM as usize];

        // Last row missing from the pooled array.
        let truncated = &actions[..actions.len() - DIM as usize];
        assert_eq!(
            chunk.sample_into(T0 + 3 * DT, truncated, &mut out),
            ActionAt::Malformed
        );
        assert_eq!(out, [0.0; DIM as usize]);

        // Output buffer too narrow for one action.
        let mut narrow = [0.0f32; 2];
        assert_eq!(
            chunk.sample_into(T0, &actions, &mut narrow),
            ActionAt::Malformed
        );
        assert_eq!(narrow, [0.0; 2]);
    }

    #[test]
    fn f64_actions_interpolate_the_same_way() {
        let tensor = Tensor::new(1, 0, 0, 0, &[HORIZON, DIM], TensorDtype::F64, Device::cpu());
        let chunk = ActionChunk::new(tensor, T0, DT);
        let actions: Vec<f64> = (0..HORIZON)
            .flat_map(|k| [10.0 * k as f64, 20.0 * k as f64, 30.0 * k as f64])
            .collect();

        let mut out = [0.0f64; DIM as usize];
        assert!(chunk
            .sample_f64_into(T0 + DT / 2, &actions, &mut out)
            .is_inside());
        assert_eq!(out, [5.0, 10.0, 15.0]);

        let end = T0 + (HORIZON - 1) * DT;
        let mut held = [-1.0f64; DIM as usize];
        assert!(chunk
            .sample_f64_into(end + 1, &actions, &mut held)
            .is_stale());
        assert_eq!(
            held, [-1.0; DIM as usize],
            "stale must not write f64 either"
        );
    }

    #[test]
    fn row_range_bounds_the_last_action_a_caller_wants_to_hold() {
        let (chunk, actions) = chunk();
        let last = chunk.row_range(chunk.horizon() - 1).unwrap();
        assert_eq!(last, 9..12);
        assert_eq!(&actions[last], &[30.0, 60.0, 90.0]);
        assert_eq!(
            chunk.row_range(chunk.horizon()),
            None,
            "row 4 is off the end"
        );
    }

    #[test]
    fn action_chunk_survives_a_byte_roundtrip() {
        let tensor = Tensor::new(1, 42, 7, 0, &[16, 7], TensorDtype::F32, Device::cpu());
        let mut chunk = ActionChunk::new(tensor, T0, DT);
        chunk.set_seq(9001);
        chunk.set_frame_id("left_arm");

        let bytes: &[u8] = bytemuck::bytes_of(&chunk);
        assert_eq!(bytes.len(), 256);
        let recovered: &ActionChunk = bytemuck::from_bytes(bytes);
        assert_eq!(recovered.horizon(), 16);
        assert_eq!(recovered.action_dim(), 7);
        assert_eq!(recovered.t0_ns(), T0);
        assert_eq!(recovered.dt_ns(), DT);
        assert_eq!(recovered.seq(), 9001);
        assert_eq!(recovered.frame_id(), "left_arm");
        assert_eq!(recovered.dtype(), TensorDtype::F32);
    }

    #[test]
    fn action_chunk_survives_a_serde_roundtrip() {
        let tensor = Tensor::new(1, 3, 1, 0, &[8, 6], TensorDtype::F32, Device::cpu());
        let mut chunk = ActionChunk::new(tensor, T0, DT);
        chunk.set_seq(17);
        chunk.set_frame_id("gripper");

        let json = serde_json::to_string(&chunk).unwrap();
        let recovered: ActionChunk = serde_json::from_str(&json).unwrap();
        assert_eq!(recovered.horizon(), 8);
        assert_eq!(recovered.action_dim(), 6);
        assert_eq!(recovered.t0_ns(), T0);
        assert_eq!(recovered.dt_ns(), DT);
        assert_eq!(recovered.seq(), 17);
        assert_eq!(recovered.frame_id(), "gripper");
    }

    /// A 30 Hz policy feeding a 1 kHz servo loop: every tick between two chunks
    /// must be answerable, and the tick after the last action must not be.
    #[test]
    fn a_thirty_hertz_chunk_answers_every_tick_of_a_one_kilohertz_loop() {
        let horizon = 16u64;
        let dt = 33_333_333u64; // 30 Hz
        let tensor = Tensor::new(1, 0, 0, 0, &[horizon, 1], TensorDtype::F32, Device::cpu());
        let chunk = ActionChunk::new(tensor, T0, dt);
        let actions: Vec<f32> = (0..horizon).map(|k| k as f32).collect();
        let end = T0 + (horizon - 1) * dt;

        let mut out = [0.0f32];
        let mut ticks = 0u32;
        let mut t = T0;
        while t <= end {
            let at = chunk.sample_into(t, &actions, &mut out);
            assert!(at.is_inside(), "tick at {t} fell outside the chunk: {at:?}");
            // The commanded value must stay inside the chunk's own range — an
            // extrapolating interpolator overshoots here first.
            assert!(
                (0.0..=(horizon - 1) as f32).contains(&out[0]),
                "tick at {t} produced {}, outside the chunk's action range",
                out[0]
            );
            ticks += 1;
            t += 1_000_000; // 1 kHz
        }
        assert!(
            ticks > 400,
            "expected the full chunk to be walked, got {ticks}"
        );
        assert!(chunk
            .sample_into(end + 1_000_000, &actions, &mut out)
            .is_stale());
    }
}
