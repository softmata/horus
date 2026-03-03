"""
Regression tests for DLPack protocol, __array_interface__ lifetime semantics,
and the torch_to_numpy CPU/CUDA guard in the HORUS Python bindings.

These tests exercise the Python-visible behaviour of fixes made in:
  - dlpack_utils::torch_to_numpy  — inverted CPU/CUDA guard (now raises on non-CPU)
  - dlpack_capsule_destructor     — double-free guard (null ptr before deleter call)
  - __array_interface__            — __horus_obj__ lifetime anchor keeps backing alive
  - make_dlpack_capsule            — DLPack roundtrip correctness

The mock-based CPU/CUDA tests (tests 1–2) run unconditionally without a real
GPU.  All other tests use CPU Images only and do not require CUDA hardware.

Relationship to Rust unit tests
--------------------------------
Rust unit tests in tensor.rs (DLPackDeleterGuard) and dlpack_utils.rs verify
the deleter is called exactly once in both the success and error paths at the
Rust level.  This file adds Python-visible regression coverage so that PyO3
upgrades or API changes cannot silently break the contract.
"""

import gc
import tracemalloc

import numpy as np
import pytest


# ─── Mock torch helpers ───────────────────────────────────────────────────────


class _MockDevice:
    """Minimal stand-in for ``torch.device``."""

    def __init__(self, device_type: str) -> None:
        self.type = device_type


class _MockTensor:
    """Minimal stand-in for ``torch.Tensor`` with ``.device`` and ``.numpy()``."""

    def __init__(self, device_type: str, np_data=None) -> None:
        self.device = _MockDevice(device_type)
        self._np_data = np_data

    def numpy(self):  # noqa: D401
        return self._np_data


# ─── Test 1: torch_to_numpy CPU (positive path) ───────────────────────────────


def test_torch_to_numpy_cpu():
    """
    ``dlpack_utils::torch_to_numpy`` must accept objects whose ``device.type`` is
    ``"cpu"`` and proceed to call ``tensor.numpy()``.

    Regression: the guard previously had an inverted condition (``== "cpu"``
    instead of ``!= "cpu"``), which rejected CPU tensors and silently accepted
    CUDA tensors.  This test confirms the corrected direction: CPU → success.
    """
    from horus._horus import Image

    data = np.zeros((4, 6, 3), dtype=np.uint8)
    cpu_tensor = _MockTensor("cpu", data)

    img = Image.from_torch(cpu_tensor)

    assert img is not None
    assert img.height == 4
    assert img.width == 6
    assert img.encoding == "rgb8"


# ─── Test 2: torch_to_numpy CUDA (negative path) ─────────────────────────────


def test_torch_to_numpy_cuda():
    """
    ``dlpack_utils::torch_to_numpy`` must raise ``ValueError`` (or ``TypeError``)
    for any tensor whose ``device.type`` is not ``"cpu"``.

    Regression: the inverted guard silently passed CUDA tensors to ``.numpy()``,
    which would panic or return garbage data on a real GPU tensor.
    """
    from horus._horus import Image

    cuda_tensor = _MockTensor("cuda")

    with pytest.raises((ValueError, TypeError)) as exc_info:
        Image.from_torch(cuda_tensor)

    msg = str(exc_info.value).lower()
    assert "cpu" in msg or "cuda" in msg, (
        f"Expected error message to mention CPU or CUDA, got: {exc_info.value!r}"
    )


@pytest.mark.parametrize("device_type", ["cuda:0", "cuda:1", "mps", "xpu"])
def test_torch_to_numpy_non_cpu_device_raises(device_type: str):
    """Any non-CPU ``device.type`` value must be rejected by the guard."""
    from horus._horus import Image

    with pytest.raises((ValueError, TypeError), match="(?i)cpu"):
        Image.from_torch(_MockTensor(device_type))


# ─── Test 3: __array_interface__ lifetime anchor ──────────────────────────────


def test_array_interface_lifetime():
    """
    Dropping the ``Image`` Python object after acquiring a numpy view must not
    invalidate the array or cause data corruption.

    ``np.asarray(img)`` sets ``arr.base = img``, keeping the ``PyImage`` alive
    as long as the numpy array is alive.  The ``__array_interface__`` dict also
    stores a ``__horus_obj__`` reference as a belt-and-suspenders anchor so
    the TensorPool slot survives even if numpy's ``base`` bookkeeping is
    bypassed.

    Regression: without the lifetime anchor the TensorPool slot could be freed
    while numpy still held a raw pointer into Rust-managed shared memory,
    causing silent reads of freed (or recycled) memory.
    """
    from horus._horus import Image

    original = np.arange(48, dtype=np.uint8).reshape(4, 4, 3)
    img = Image.from_numpy(original)

    # Zero-copy numpy view via __array_interface__.
    arr = img.to_numpy()
    expected = arr.copy()  # byte-accurate snapshot before we drop img

    # Release the Image Python binding.  The lifetime anchor must keep the
    # Rust-side memory alive because ``arr`` is still in scope.
    del img
    gc.collect()

    # The array must still be readable and contain the original values.
    np.testing.assert_array_equal(
        arr,
        expected,
        err_msg=(
            "numpy array became invalid after Image was dropped — "
            "__array_interface__ lifetime anchor (__horus_obj__) is broken"
        ),
    )


# ─── Test 4: DLPack roundtrip ─────────────────────────────────────────────────


def test_dlpack_roundtrip():
    """
    ``np.from_dlpack(img)`` must produce raw bytes identical to the data used
    to construct the ``Image``.

    Tests the full DLPack export path end-to-end::

        Image.from_numpy(src)
        → img.__dlpack__(stream=None) capsule
        → np.from_dlpack(img)
        → compare flattened bytes to src

    Verifies that ``make_dlpack_capsule`` constructs a valid ``DLManagedTensor``
    with correct data pointer, shape, strides, and dtype.

    Note: shape in the DLPack tensor may differ from the numpy shape (e.g.
    the internal tensor might be stored as (H*W*C,) rather than (H, W, C)).
    The test therefore compares flattened bytes, which is robust regardless
    of the internal layout chosen by ``HorusTensor``.
    """
    from horus._horus import Image

    rng = np.random.default_rng(seed=7)
    src = rng.integers(0, 256, size=(4, 6, 3), dtype=np.uint8)

    img = Image.from_numpy(src)  # rgb8

    # np.from_dlpack calls img.__dlpack__(stream=None) and img.__dlpack_device__().
    imported = np.from_dlpack(img)

    assert imported.nbytes == src.nbytes, (
        f"DLPack roundtrip: byte count mismatch "
        f"(imported={imported.nbytes}, src={src.nbytes})"
    )
    np.testing.assert_array_equal(
        imported.ravel(),
        src.ravel(),
        err_msg="DLPack roundtrip: raw bytes differ from the source data",
    )


# ─── Test 5: DLPack capsule — no double-free on GC ────────────────────────────


def test_dlpack_capsule_no_double_free():
    """
    Exporting a DLPack capsule and then abandoning it (without consuming it via
    ``np.from_dlpack``) must not crash when the GC runs.

    The capsule destructor ``dlpack_capsule_destructor`` guards against
    double-free by:

    1. Obtaining the ``DLManagedTensor*`` pointer.
    2. **Nulling the pointer in the capsule** (via ``PyCapsule_SetPointer``)
       *before* invoking the deleter.
    3. Calling the deleter.

    Any second invocation of the destructor (possible when Python's cyclic GC
    finalises reference cycles) finds a null pointer and returns immediately
    without calling the deleter a second time.

    This test stresses the destructor by creating 20 unclaimed capsules
    (name stays ``"dltensor"``), explicitly triggering GC twice per iteration.
    A SIGABRT, SIGSEGV, or process crash indicates a double-free or
    use-after-free in the destructor.
    """
    from horus._horus import Image

    for _ in range(20):
        img = Image(3, 3, "mono8")
        # Export capsule but do NOT consume it — name remains "dltensor".
        capsule = img.__dlpack__(stream=None)
        # Release in arbitrary order; destructor must fire exactly once.
        del img
        del capsule
        gc.collect()
        gc.collect()  # second pass must be a safe no-op, not a second free


# ─── Test 6: from_dlpack error path — no Python-level object leak ─────────────


def test_from_dlpack_error_no_leak():
    """
    Repeatedly creating ``Image`` objects, exporting DLPack capsules, and
    abandoning them must not accumulate Python-level heap allocations.

    When a capsule is exported but never consumed (e.g. the consumer rejected
    it after inspecting the device type or shape), Python's GC is responsible
    for finalising the capsule.  This triggers ``dlpack_capsule_destructor``
    which must free the ``DLManagedTensor`` exactly once and release the
    capsule ``PyObject`` from the Python heap.

    ``tracemalloc`` is used to confirm that Python-level allocations do not
    grow monotonically.  Note: ``tracemalloc`` tracks allocations on the Python
    heap (CPython allocator) but not Rust heap allocations.  The Rust-level
    guarantee that the ``DLManagedTensor`` is freed is verified by the unit
    tests in ``tensor.rs`` (``DLPackDeleterGuard``).

    A net Python-heap growth larger than 16 KiB after 30 iterations is
    considered a regression.
    """
    from horus._horus import Image

    ITERATIONS = 30
    LEAK_THRESHOLD_BYTES = 16_384  # 16 KiB — headroom for Python internals

    gc.collect()
    tracemalloc.start()
    baseline = tracemalloc.take_snapshot()

    for _ in range(ITERATIONS):
        img = Image(8, 8, "rgb8")
        # Export capsule and abandon it immediately — destructor must clean up.
        capsule = img.__dlpack__(stream=None)
        del capsule
        del img

    gc.collect()
    gc.collect()

    after = tracemalloc.take_snapshot()
    tracemalloc.stop()

    stats = after.compare_to(baseline, "lineno")
    net_growth = sum(s.size_diff for s in stats if s.size_diff > 0)

    top_growth = [(str(s.traceback), s.size_diff) for s in stats[:5] if s.size_diff > 0]

    assert net_growth < LEAK_THRESHOLD_BYTES, (
        f"Python heap grew by {net_growth} bytes after {ITERATIONS} abandoned "
        f"DLPack capsules (threshold: {LEAK_THRESHOLD_BYTES} bytes). "
        f"Top growth sites: {top_growth}"
    )
