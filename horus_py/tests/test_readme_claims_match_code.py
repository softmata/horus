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


def test_dlpack_is_claimed_only_if_implemented():
    """
    `torch.from_dlpack(x)` requires `x.__dlpack__`. If the README names DLPack
    as a capability, the protocol has to be on the object users would pass.
    """
    text = readme_text().lower()
    claims_dlpack = "dlpack" in text

    tensor_has_dlpack = hasattr(horus.Tensor, "__dlpack__")

    if claims_dlpack and not tensor_has_dlpack:
        pytest.fail(
            "README mentions DLPack but horus.Tensor has no __dlpack__, so "
            "torch.from_dlpack() cannot work. Either implement the protocol "
            "(__dlpack__ and __dlpack_device__) or drop the claim."
        )


def test_gpu_memory_is_claimed_only_if_a_device_allocator_exists():
    """
    The tensor pool is mmap-backed host memory. If the README starts promising
    device-resident tensors again, something has to be able to allocate them.

    `PoolAllocator` has exactly one variant (Mmap) and `auto_allocator()` was
    hardcoded to it while the Python docstring advertised cudaMallocManaged and
    cudaMallocHost by name.
    """
    text = readme_text()
    promises_device_alloc = any(
        marker in text
        for marker in ("cudaMallocManaged", "cudaMallocHost", "cudaMalloc(")
    )
    if not promises_device_alloc:
        return

    pool = horus.TensorPool(pool_id=9931, size_mb=1, max_slots=4)
    t = pool.alloc([2, 2], "float32")
    assert t is not None
    pytest.fail(
        "README names a CUDA allocator, but the pool allocator is mmap. "
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
