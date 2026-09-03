"""
Capabilities the README names must exist in the built extension.

The README advertised "DLPack zero-copy (PyTorch/JAX native)" in the comparison
table and showed `torch.from_dlpack(frame)` in the AI-pipeline example. Nothing
in horus_py implements the DLPack protocol: `grep -rn __dlpack__ horus_py`
returns nothing, and `horus.Tensor` exposes no dlpack attribute. The claim was
load-bearing for exactly the users it was aimed at — someone choosing a runtime
for a GPU inference pipeline reads that row and plans around it.

Documentation drifts from code silently because nothing executes it. These tests
execute it. They are deliberately narrow: each one names a claim, finds it in the
README, and asserts the corresponding capability is actually reachable from the
built module. Adding the capability makes the test pass; removing the claim makes
it pass; leaving them inconsistent fails.
"""
from pathlib import Path

import pytest

import horus

REPO_ROOT = Path(__file__).resolve().parents[2]
README = REPO_ROOT / "README.md"


def readme_text() -> str:
    if not README.exists():
        pytest.skip(f"README not found at {README}")
    return README.read_text(encoding="utf-8")


def test_readme_exists_where_we_think_it_does():
    """Guard the path: a skip that is really a missing file proves nothing."""
    assert README.exists(), f"expected the README at {README}"


# Types a README DLPack claim could be about. The comparison-table row was
# generic ("DLPack zero-copy (PyTorch/JAX native)") and the example passed a
# received frame, which is an Image — so the protocol landing on either type
# makes the claim true, and requiring it on one of them would fail a correct
# implementation of the other.
DLPACK_CARRIERS = ("Image", "Tensor")


def test_dlpack_is_claimed_only_if_implemented():
    """
    `torch.from_dlpack(x)` requires `x.__dlpack__`. If the README names DLPack
    as a capability, the protocol has to be on an object users would pass.
    """
    mentions = [
        f"README.md:{lineno}: {line.strip()}"
        for lineno, line in enumerate(readme_text().splitlines(), 1)
        if "dlpack" in line.lower()
    ]
    if not mentions:
        return

    if any(
        hasattr(getattr(horus, name, None), "__dlpack__") for name in DLPACK_CARRIERS
    ):
        return

    pytest.fail(
        "README mentions DLPack but neither "
        + " nor ".join(f"horus.{name}" for name in DLPACK_CARRIERS)
        + " has __dlpack__, so torch.from_dlpack() cannot work on either. "
        "Implement the protocol (__dlpack__ and __dlpack_device__) on the type "
        "the text is about, or drop the claim. Lines that mention it:\n  "
        + "\n  ".join(mentions)
    )


def test_gpu_memory_is_claimed_only_if_a_device_allocator_exists():
    """
    The tensor pool is mmap-backed host memory. If the README starts promising
    device-resident tensors again, something has to be able to allocate them.

    `PoolAllocator` has exactly one variant (Mmap) and `pool_allocator()` in
    horus_py/src/tensor.rs is hardcoded to it, while the Python docstring used
    to advertise cudaMallocManaged and cudaMallocHost by name.
    """
    text = readme_text()
    named = [
        marker
        for marker in ("cudaMallocManaged", "cudaMallocHost", "cudaMalloc(")
        if marker in text
    ]
    if not named:
        return

    # No allocation here on purpose: a pool proves nothing about device memory
    # (it is mmap either way), and building one to throw away leaves a registry
    # entry and a shared-memory file behind for a test that has already decided.
    pytest.fail(
        f"README names a CUDA allocator ({', '.join(named)}), but pools are "
        "built with PoolAllocator::Mmap and that enum has exactly one variant. "
        "Either add a device-backed PoolBackend or drop the claim."
    )


def test_advertised_numpy_path_actually_works():
    """
    The AI-pipeline example now shows `frame.to_numpy()`. That call has to exist
    and round-trip, or the replacement claim is as false as the one it replaced.
    """
    import numpy as np

    img = horus.Image.from_numpy(np.zeros((4, 4, 3), dtype="uint8"))
    arr = img.to_numpy()
    assert arr.shape == (4, 4, 3)
    assert arr.dtype == np.uint8


def test_tensor_numpy_round_trip_is_exact():
    """The zero-copy claim in the same paragraph, on the Tensor path."""
    import numpy as np

    src = np.arange(6, dtype="float32").reshape(2, 3)
    t = horus.Tensor.from_numpy(src)
    back = t.numpy()
    np.testing.assert_array_equal(back, src)
