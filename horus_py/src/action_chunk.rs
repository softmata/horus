//! Python bindings for `ActionChunk` — the policy-to-servo seam.
//!
//! A chunk carries a `[horizon, action_dim]` block of future actions with a
//! start time and a spacing, so a control loop running at 500-1000 Hz can keep
//! producing commands between publishes from a policy running at 10-50 Hz. The
//! loop interpolates *within* the newest chunk rather than latching the last
//! command or extrapolating past the end of one.
//!
//! # The property this binding must not lose
//!
//! A chunk goes **stale** when the query time runs past its last action. That
//! is the safety-relevant case: the policy has not published in time, and
//! producing an action anyway means extrapolating a trajectory nobody
//! computed. `horus_core` refuses to do it — `sample_into` writes the output
//! buffer if and only if it returns `Inside` — and this binding is built so
//! the refusal survives the trip into Python:
//!
//! - [`PyActionAt`] is a real enum with one Python type per variant, so
//!   `Stale` cannot be confused with `Inside` by a typo in a string or by
//!   truthiness. `Malformed` is likewise its own type, so a corrupt descriptor
//!   never reads as "has not started yet".
//! - [`PyActionChunk::sample`] builds its array *inside* the `Inside` match
//!   arm and nowhere else, so a caller who ignores the status entirely gets
//!   `None` — and `None` cannot be commanded to a servo.
//! - [`PyActionChunk::sample_into`] hands the caller's own buffer to
//!   `horus_core` and does nothing else, so a stale query leaves that buffer
//!   holding the caller's previous decision rather than a fabricated one.

use crate::tensor::{get_or_create_pool, PyTensorHandle};
use crate::tensor_convert;
use horus_core::memory::ActionChunkHandle;
use horus_core::types::{ActionAt, TensorDtype};
use pyo3::exceptions::{PyIndexError, PyRuntimeError, PyTypeError, PyValueError};
use pyo3::prelude::*;

/// Where a query time falls relative to a chunk.
///
/// Mirrors `horus_core::types::ActionAt`. `Malformed()` is an empty tuple
/// variant rather than a unit variant because pyo3 rejects unit variants in a
/// complex enum — it is not a style choice.
///
/// Deliberately carries no `__bool__`, no `__len__` and no numeric conversion:
/// every one of them would be a way to read a stale result as a usable number.
// `skip_from_py_object`: this enum is only ever RETURNED to Python, never
// accepted as an argument, so the derive pyo3 is deprecating is not wanted.
#[pyclass(name = "ActionAt", module = "horus", skip_from_py_object)]
#[derive(Clone, Debug, PartialEq)]
pub enum PyActionAt {
    /// The chunk has not started yet; it begins in `by_ns` nanoseconds.
    Before { by_ns: u64 },
    /// The query is inside the chunk and can be interpolated between rows
    /// `lo` and `hi` with blend `alpha`.
    Inside { lo: u32, hi: u32, alpha: f32 },
    /// The query is past the chunk's last action by `by_ns` nanoseconds.
    ///
    /// A few hundred microseconds is jitter. Tens of milliseconds is a policy
    /// that has stopped publishing, and the number is what tells them apart —
    /// which is why this carries it rather than being a bare flag.
    Stale { by_ns: u64 },
    /// The descriptor cannot be interpreted at all.
    Malformed(),
}

pub(crate) fn to_py_action_at(at: ActionAt) -> PyActionAt {
    match at {
        ActionAt::Before { by_ns } => PyActionAt::Before { by_ns },
        ActionAt::Inside { lo, hi, alpha } => PyActionAt::Inside { lo, hi, alpha },
        ActionAt::Stale { by_ns } => PyActionAt::Stale { by_ns },
        ActionAt::Malformed => PyActionAt::Malformed(),
    }
}

/// A block of future actions with a start time and a spacing.
///
/// The policy-to-servo seam: a policy running at 10-50 Hz publishes a horizon
/// of actions, and a control loop running at 500-1000 Hz interpolates within
/// the newest chunk instead of latching the last command.
///
/// Publishing, from a policy:
///
/// ```python
/// import numpy as np, horus
///
/// topic = horus.Topic(horus.ActionChunk, endpoint="arm.actions")
/// actions = policy(observation)          # (horizon, action_dim) float32
/// topic.send(horus.ActionChunk.from_numpy(
///     actions, t0_ns=now_ns(), dt_ns=20_000_000, frame_id="arm"))
/// ```
///
/// Consuming, from a control loop:
///
/// ```python
/// held = None
/// while running:
///     chunk = topic.recv()
///     if chunk is not None:
///         held = chunk
///     if held is not None:
///         action = held.sample(now_ns())
///         if action is not None:
///             servo.command(action)
///         else:
///             # No action for this instant. Ask why before deciding what to
///             # do about it -- a chunk that has not started yet is not the
///             # same problem as a policy that has stopped.
///             at = held.locate(now_ns())
///             if isinstance(at, horus.ActionAt.Stale):
///                 servo.hold()
///                 watchdog.policy_late(at.by_ns)
/// ```
///
/// ``sample()`` returns an action **only** when the query falls inside the
/// chunk. Past the last action it returns ``None`` rather than extrapolating a
/// trajectory nobody computed, so a loop that ignores the status still cannot
/// drive a servo from a policy that has stopped publishing.
///
/// On a hot path, ``sample_into(query_ns, out)`` writes into an array you own
/// and returns the same status, allocating nothing. It leaves ``out`` untouched
/// unless the status is ``Inside``, so a stale query preserves whatever you
/// last put there.
#[pyclass(name = "ActionChunk", module = "horus")]
pub struct PyActionChunk {
    inner: ActionChunkHandle,
}

impl PyActionChunk {
    /// Wrap a handle received from a topic. Not a `#[pymethods]` entry — this
    /// is the bridge `topic.rs` uses.
    pub fn from_inner(inner: ActionChunkHandle) -> Self {
        Self { inner }
    }

    /// The underlying handle, for the topic send path.
    pub fn inner(&self) -> &ActionChunkHandle {
        &self.inner
    }
}

/// Reject a chunk whose dtype the f32 sampling path cannot read, naming the
/// dtype rather than returning a `None` that would be indistinguishable from
/// a stale query.
fn require_f32(handle: &ActionChunkHandle, call: &str) -> PyResult<()> {
    let dtype = handle.tensor().dtype();
    if dtype != TensorDtype::F32 {
        return Err(PyTypeError::new_err(format!(
            "{call}() requires a float32 action chunk, this one is {}. The \
             interpolating sample path in horus_core is f32-only; convert the \
             actions before publishing.",
            tensor_convert::dtype_to_str(dtype)
        )));
    }
    Ok(())
}

#[pymethods]
impl PyActionChunk {
    /// Allocate a zeroed `[horizon, action_dim]` chunk.
    #[new]
    #[pyo3(signature = (horizon, action_dim, t0_ns, dt_ns, dtype="float32", device="cpu"))]
    fn new(
        horizon: u32,
        action_dim: u32,
        t0_ns: u64,
        dt_ns: u64,
        dtype: &str,
        device: &str,
    ) -> PyResult<Self> {
        // A zero dimension gives a chunk every query reports Malformed for.
        // Refusing here beats handing back an object that silently answers
        // nothing.
        if horizon == 0 || action_dim == 0 {
            return Err(PyValueError::new_err(format!(
                "horizon and action_dim must both be > 0, got horizon={horizon} \
                 action_dim={action_dim}; a zero-sized chunk reports Malformed \
                 for every query"
            )));
        }
        let dt = tensor_convert::parse_dtype(dtype)?;
        let dev = tensor_convert::parse_device(device)?;
        // Pool 1 is the process-local scratch pool `Tensor.from_numpy` uses.
        // NOT the global pool: `Topic<ActionChunk>::pool()` is name-keyed, and
        // publishing a global-pool chunk on a name-keyed topic makes
        // `recv_chunk`'s retain fail — which surfaces as a topic that is
        // simply never ready. The topic send path re-allocates into the
        // topic's own pool, so this pool only has to hold the chunk until then.
        let pool = get_or_create_pool(1, None)?;
        let inner = ActionChunkHandle::alloc(pool, horizon, action_dim, dt, dev, t0_ns, dt_ns)
            .map_err(|e| {
                PyRuntimeError::new_err(format!(
                    "Failed to allocate ActionChunk: {e}. Common causes: \
                     insufficient shared memory (check: df -h /dev/shm), or a \
                     horizon x action_dim larger than the pool's slot size."
                ))
            })?;
        Ok(Self { inner })
    }

    /// Build a chunk from a `(horizon, action_dim)` numpy array.
    ///
    /// A 1-D `(horizon,)` array is read as `action_dim = 1`. `float64` is
    /// converted to `float32`; any other dtype is refused by name.
    #[staticmethod]
    #[pyo3(signature = (actions, t0_ns, dt_ns, frame_id=None, seq=0))]
    fn from_numpy(
        py: Python<'_>,
        actions: &Bound<'_, PyAny>,
        t0_ns: u64,
        dt_ns: u64,
        frame_id: Option<&str>,
        seq: u64,
    ) -> PyResult<Self> {
        let np = py.import("numpy")?;
        let arr = np.call_method1("ascontiguousarray", (actions,))?;

        let dtype_str: String = arr.getattr("dtype")?.getattr("str")?.extract()?;
        // Accept the two float widths a policy realistically produces and
        // refuse everything else by name. Silently reinterpreting int bytes as
        // f32 would produce actions, which is worse than an error.
        let arr = match dtype_str.as_str() {
            "<f4" | "|f4" | "=f4" => arr,
            "<f8" | "|f8" | "=f8" => arr.call_method1("astype", ("float32",))?,
            other => {
                return Err(PyTypeError::new_err(format!(
                    "ActionChunk.from_numpy requires float32 or float64 actions, \
                     got dtype '{other}'. Convert explicitly — reinterpreting \
                     these bytes as float32 would produce actions from them."
                )))
            }
        };

        let shape: Vec<i64> = arr.getattr("shape")?.extract()?;
        let (horizon, action_dim) = match shape.as_slice() {
            [h] => (*h, 1i64),
            [h, d] => (*h, *d),
            other => {
                return Err(PyValueError::new_err(format!(
                    "ActionChunk.from_numpy expects a 1-D (horizon,) or 2-D \
                     (horizon, action_dim) array, got shape with {} dimensions",
                    other.len()
                )))
            }
        };
        if horizon <= 0 || action_dim <= 0 {
            return Err(PyValueError::new_err(format!(
                "horizon and action_dim must both be > 0, got ({horizon}, {action_dim})"
            )));
        }

        let mut chunk = Self::new(
            horizon as u32,
            action_dim as u32,
            t0_ns,
            dt_ns,
            "float32",
            "cpu",
        )?;

        // `tobytes()` rather than a raw pointer read: it is already a copy, so
        // there is no lifetime to get wrong, and the length check below is
        // exact rather than a trusted stride computation.
        let src: Vec<u8> = arr.call_method0("tobytes")?.extract()?;
        {
            let dst = chunk
                .inner
                .tensor()
                .data_slice_mut()
                .map_err(|e| PyRuntimeError::new_err(format!("chunk write failed: {e}")))?;
            if src.len() != dst.len() {
                return Err(PyValueError::new_err(format!(
                    "action buffer is {} bytes but the chunk holds {} — refusing \
                     a partial copy",
                    src.len(),
                    dst.len()
                )));
            }
            dst.copy_from_slice(&src);
        }

        chunk.inner.chunk_mut().set_seq(seq);
        if let Some(f) = frame_id {
            chunk.inner.chunk_mut().set_frame_id(f);
        }
        Ok(chunk)
    }

    // ─── Metadata ────────────────────────────────────────────────────────

    #[getter]
    fn horizon(&self) -> u32 {
        self.inner.chunk().horizon()
    }
    #[getter]
    fn action_dim(&self) -> u32 {
        self.inner.chunk().action_dim()
    }
    #[getter]
    fn shape(&self) -> (u32, u32) {
        (
            self.inner.chunk().horizon(),
            self.inner.chunk().action_dim(),
        )
    }
    #[getter]
    fn t0_ns(&self) -> u64 {
        self.inner.chunk().t0_ns()
    }
    #[setter]
    fn set_t0_ns(&mut self, t0_ns: u64) {
        self.inner.chunk_mut().set_t0_ns(t0_ns);
    }
    #[getter]
    fn dt_ns(&self) -> u64 {
        self.inner.chunk().dt_ns()
    }
    #[getter]
    fn seq(&self) -> u64 {
        self.inner.chunk().seq()
    }
    #[setter]
    fn set_seq(&mut self, seq: u64) {
        self.inner.chunk_mut().set_seq(seq);
    }
    #[getter]
    fn frame_id(&self) -> String {
        self.inner.chunk().frame_id().to_string()
    }
    #[setter]
    fn set_frame_id(&mut self, frame_id: &str) {
        self.inner.chunk_mut().set_frame_id(frame_id);
    }

    /// When the chunk's last action applies, or `None` if the descriptor is
    /// unusable.
    #[getter]
    fn end_ns(&self) -> Option<u64> {
        self.inner.chunk().end_ns()
    }
    #[getter]
    fn dtype(&self) -> &'static str {
        tensor_convert::dtype_to_str(self.inner.tensor().dtype())
    }
    #[getter]
    fn device(&self) -> String {
        tensor_convert::device_to_string(self.inner.tensor().device())
    }
    #[getter]
    fn nbytes(&self) -> u64 {
        self.inner.tensor().nbytes()
    }
    #[getter]
    fn pool_id(&self) -> u32 {
        self.inner.tensor().tensor().pool_id
    }
    /// Live references on the pool slot holding these actions. Exposed because
    /// it is the only way a test can show the slot is actually retained across
    /// a topic send.
    #[getter]
    fn refcount(&self) -> u64 {
        self.inner.tensor().refcount()
    }

    // ─── Actions ─────────────────────────────────────────────────────────

    /// The actions as a `(horizon, action_dim)` numpy array.
    fn to_numpy<'py>(&self, py: Python<'py>) -> PyResult<Bound<'py, PyAny>> {
        require_f32(&self.inner, "to_numpy")?;
        let actions = self
            .inner
            .actions_f32()
            .ok_or_else(|| PyRuntimeError::new_err("action data is not readable as float32"))?;
        // Take exactly horizon x action_dim, not the whole slice.
        //
        // `sanitize_from_shm` clamps horizon and action_dim DOWN to the
        // tensor's real shape when a descriptor arrives from peer-writable
        // memory with fields that overstate it. After such a clamp the
        // descriptor covers fewer elements than the allocation holds, and
        // reshaping the full slice to (horizon, action_dim) would fail inside
        // numpy with a size mismatch that says nothing about what happened.
        // Slicing to what the descriptor actually claims returns the readable
        // prefix and cannot fail.
        let wanted = (self.horizon() as usize).saturating_mul(self.action_dim() as usize);
        let view = actions.get(..wanted).ok_or_else(|| {
            PyRuntimeError::new_err(format!(
                "chunk claims {} x {} = {wanted} actions but only {} are allocated",
                self.horizon(),
                self.action_dim(),
                actions.len()
            ))
        })?;
        let np = py.import("numpy")?;
        let flat = np.call_method1("array", (view.to_vec(),))?;
        let flat = flat.call_method1("astype", ("float32",))?;
        let reshaped = flat.call_method1(
            "reshape",
            ((self.horizon() as usize, self.action_dim() as usize),),
        )?;
        Ok(reshaped)
    }

    /// One action row, by index.
    ///
    /// The "hold the last commanded position" case: after a chunk goes stale a
    /// caller may deliberately want its final row, and asking for it by index
    /// is honest in a way that sampling past the end would not be.
    fn row<'py>(&self, py: Python<'py>, index: u32) -> PyResult<Bound<'py, PyAny>> {
        require_f32(&self.inner, "row")?;
        let range = self.inner.chunk().row_range(index).ok_or_else(|| {
            PyIndexError::new_err(format!(
                "row {index} is out of range for a chunk of horizon {}",
                self.horizon()
            ))
        })?;
        let actions = self
            .inner
            .actions_f32()
            .ok_or_else(|| PyRuntimeError::new_err("action data is not readable as float32"))?;
        let slice = actions
            .get(range)
            .ok_or_else(|| PyIndexError::new_err("row range falls outside the action data"))?;
        let np = py.import("numpy")?;
        np.call_method1("array", (slice.to_vec(),))
    }

    /// This chunk's actions as a `horus.Tensor`, sharing the pool slot.
    fn as_tensor(&self) -> PyTensorHandle {
        // `TensorHandle::clone` retains, so the returned Tensor holds its own
        // reference and the slot survives this chunk being dropped.
        PyTensorHandle {
            handle: Some(self.inner.tensor().clone()),
            view_keepalive: None,
        }
    }

    // ─── The query ───────────────────────────────────────────────────────

    /// Where `query_ns` falls relative to this chunk.
    ///
    /// Touches only the descriptor, never the action data, and never raises —
    /// safe to call every tick of a control loop.
    fn locate(&self, query_ns: u64) -> PyActionAt {
        to_py_action_at(self.inner.locate(query_ns))
    }

    /// The interpolated action at `query_ns`, or `None`.
    ///
    /// Returns a `(action_dim,)` float32 array **if and only if** the query
    /// falls inside the chunk. `Before`, `Stale` and `Malformed` all return
    /// `None`, so a caller who never inspects the status still cannot drive a
    /// servo from a policy that has stopped publishing.
    ///
    /// Use [`locate`](Self::locate) when you need to tell those three apart —
    /// `Stale` carries how far past the chunk the query is, which is what
    /// distinguishes scheduling jitter from a dead policy.
    fn sample<'py>(&self, py: Python<'py>, query_ns: u64) -> PyResult<Option<Bound<'py, PyAny>>> {
        // Raised rather than folded into `None`: a wrong dtype is a
        // programming error, and reporting it as "no action available" would
        // look exactly like a stale chunk.
        require_f32(&self.inner, "sample")?;
        let mut out = vec![0.0f32; self.action_dim() as usize];
        match self.inner.sample_into(query_ns, &mut out) {
            ActionAt::Inside { .. } => {
                let np = py.import("numpy")?;
                let arr = np.call_method1("array", (out,))?;
                let arr = arr.call_method1("astype", ("float32",))?;
                Ok(Some(arr))
            }
            // Every other outcome yields nothing. The array is constructed in
            // the arm above and nowhere else, so there is no path by which a
            // non-Inside query produces numbers.
            _ => Ok(None),
        }
    }

    /// Sample into a caller-provided float32 array, returning the status.
    ///
    /// The allocation-free form, and an exact mirror of the Rust contract:
    /// `out` is written **if and only if** `Inside` is returned. A stale query
    /// leaves `out` holding whatever the caller last put there — normally
    /// their previous command — rather than a fabricated extrapolation.
    ///
    /// `out` must be a writable, C-contiguous 1-D float32 numpy array of at
    /// least `action_dim` elements. Every precondition is checked before any
    /// write.
    fn sample_into(
        &self,
        _py: Python<'_>,
        query_ns: u64,
        out: &Bound<'_, PyAny>,
    ) -> PyResult<PyActionAt> {
        require_f32(&self.inner, "sample_into")?;
        let iface = out.getattr("__array_interface__").map_err(|_| {
            PyTypeError::new_err(
                "sample_into(out=) requires a numpy array; the object has no \
                 __array_interface__",
            )
        })?;

        let typestr: String = iface.get_item("typestr")?.extract()?;
        if !matches!(typestr.as_str(), "<f4" | "|f4" | "=f4") {
            return Err(PyTypeError::new_err(format!(
                "sample_into(out=) requires a float32 array, got typestr '{typestr}'"
            )));
        }
        let shape: Vec<i64> = iface.get_item("shape")?.extract()?;
        let needed = self.action_dim() as i64;
        match shape.as_slice() {
            [n] if *n >= needed => {}
            [n] => {
                return Err(PyValueError::new_err(format!(
                    "sample_into(out=) needs at least {needed} elements for this \
                     chunk's action_dim, got {n}"
                )))
            }
            other => {
                return Err(PyValueError::new_err(format!(
                    "sample_into(out=) requires a 1-D array, got {} dimensions",
                    other.len()
                )))
            }
        }
        // `strides` non-None means a non-contiguous view; writing through the
        // base pointer would scatter values into the wrong elements.
        if !iface.get_item("strides")?.is_none() {
            return Err(PyValueError::new_err(
                "sample_into(out=) requires a C-contiguous array; pass \
                 numpy.ascontiguousarray(out)",
            ));
        }
        let data = iface.get_item("data")?;
        let addr: usize = data.get_item(0)?.extract()?;
        let read_only: bool = data.get_item(1)?.extract()?;
        if read_only {
            return Err(PyValueError::new_err(
                "sample_into(out=) requires a writable array",
            ));
        }
        if addr == 0 {
            return Err(PyValueError::new_err(
                "sample_into(out=) got a null data pointer",
            ));
        }

        // SAFETY: `addr` is numpy's contiguous buffer for a 1-D float32 array
        // whose length was checked above to be at least `action_dim`, and the
        // slice is used only for the duration of this call while `out` is
        // alive and borrowed.
        let dst: &mut [f32] =
            unsafe { std::slice::from_raw_parts_mut(addr as *mut f32, needed as usize) };
        Ok(to_py_action_at(self.inner.sample_into(query_ns, dst)))
    }

    fn __repr__(&self) -> String {
        format!(
            "ActionChunk(horizon={}, action_dim={}, t0_ns={}, dt_ns={}, seq={}, dtype='{}')",
            self.horizon(),
            self.action_dim(),
            self.t0_ns(),
            self.dt_ns(),
            self.seq(),
            self.dtype(),
        )
    }

    fn __str__(&self) -> String {
        self.__repr__()
    }
}
