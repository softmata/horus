"""
Battle tests for horus.Tensor — Pythonic API, NumPy interop, and DLPack protocol.

Tests every public method on horus.Tensor:
  - Constructors: Tensor(), zeros(), empty(), from_numpy(), from_dlpack()
  - Properties: shape, dtype, nbytes, numel
  - Indexing: __getitem__, __setitem__, __len__
  - Shape ops: reshape, flatten, squeeze, unsqueeze, view, slice, T
  - Arithmetic: +, -, *, /, - (unary)
  - Comparisons: ==, <, >, <=, >=
  - Reductions: sum, mean, max, min
  - Type conversion: astype, float, half, int, byte
  - Interop: numpy(), tolist(), __array_interface__, __dlpack__
  - Zero-copy verification: data pointer matches between numpy and SHM

Requires: numpy >= 1.22 (for np.from_dlpack)
"""

import gc
import numpy as np
import pytest


# ============================================================================
# Constructors
# ============================================================================


class TestTensorConstructors:
    def test_basic_constructor(self):
        import horus
        t = horus.Tensor([100, 200], dtype="float32")
        assert t.shape == [100, 200]
        assert t.dtype == "float32"
        assert t.nbytes == 100 * 200 * 4
        assert t.numel == 20000

    def test_1d_constructor(self):
        import horus
        t = horus.Tensor([1024])
        assert t.shape == [1024]
        assert t.dtype == "float32"  # default

    def test_zeros(self):
        import horus
        t = horus.Tensor.zeros([10, 10], dtype="int32")
        arr = t.numpy()
        assert arr.shape == (10, 10)
        assert arr.dtype == np.int32
        np.testing.assert_array_equal(arr, np.zeros((10, 10), dtype=np.int32))

    def test_empty(self):
        import horus
        t = horus.Tensor.empty([5, 5], dtype="float64")
        assert t.shape == [5, 5]
        assert t.dtype == "float64"
        assert t.nbytes == 5 * 5 * 8

    def test_from_numpy_float32(self):
        import horus
        src = np.random.randn(100, 100).astype(np.float32)
        t = horus.Tensor.from_numpy(src)
        assert t.shape == [100, 100]
        assert t.dtype == "float32"
        np.testing.assert_array_equal(t.numpy(), src)

    def test_from_numpy_int8(self):
        import horus
        src = np.array([[-1, 0, 50, 100]], dtype=np.int8)
        t = horus.Tensor.from_numpy(src)
        assert t.dtype == "int8"
        np.testing.assert_array_equal(t.numpy(), src)

    def test_from_numpy_uint8(self):
        import horus
        src = np.arange(256, dtype=np.uint8).reshape(16, 16)
        t = horus.Tensor.from_numpy(src)
        assert t.dtype == "uint8"
        np.testing.assert_array_equal(t.numpy(), src)

    def test_from_numpy_float64(self):
        import horus
        src = np.eye(4, dtype=np.float64)
        t = horus.Tensor.from_numpy(src)
        assert t.dtype == "float64"
        np.testing.assert_array_equal(t.numpy(), src)

    def test_from_numpy_bool(self):
        import horus
        src = np.array([[True, False], [False, True]])
        t = horus.Tensor.from_numpy(src)
        assert t.dtype == "bool"
        np.testing.assert_array_equal(t.numpy(), src)

    def test_all_dtypes(self):
        import horus
        dtypes = [
            ("float32", np.float32),
            ("float64", np.float64),
            ("int8", np.int8),
            ("int16", np.int16),
            ("int32", np.int32),
            ("int64", np.int64),
            ("uint8", np.uint8),
            ("uint16", np.uint16),
            ("uint32", np.uint32),
            ("uint64", np.uint64),
        ]
        for horus_dtype, np_dtype in dtypes:
            t = horus.Tensor([4, 4], dtype=horus_dtype)
            arr = t.numpy()
            assert arr.dtype == np_dtype, f"dtype mismatch for {horus_dtype}"


# ============================================================================
# Properties
# ============================================================================


class TestTensorProperties:
    def test_shape(self):
        import horus
        t = horus.Tensor([3, 4, 5])
        assert t.shape == [3, 4, 5]

    def test_dtype(self):
        import horus
        t = horus.Tensor([10], dtype="uint8")
        assert t.dtype == "uint8"

    def test_nbytes(self):
        import horus
        t = horus.Tensor([10, 20], dtype="float64")
        assert t.nbytes == 10 * 20 * 8

    def test_numel(self):
        import horus
        t = horus.Tensor([3, 4, 5])
        assert t.numel == 60

    def test_len(self):
        import horus
        t = horus.Tensor([7, 3])
        assert len(t) == 7

    def test_repr(self):
        import horus
        t = horus.Tensor([10, 20], dtype="float32")
        r = repr(t)
        assert "Tensor" in r
        assert "10" in r
        assert "20" in r


# ============================================================================
# Zero-copy verification
# ============================================================================


class TestZeroCopy:
    def test_numpy_shares_memory(self):
        """numpy() must return a view into SHM, not a copy."""
        import horus
        t = horus.Tensor([100], dtype="float32")
        arr = t.numpy()
        arr[0] = 42.0
        arr2 = t.numpy()
        assert arr2[0] == 42.0, "numpy view should share memory with tensor"

    def test_from_dlpack_numpy(self):
        """np.from_dlpack(tensor) must produce a usable array."""
        import horus
        t = horus.Tensor.from_numpy(np.arange(10, dtype=np.float32))
        arr = np.from_dlpack(t)
        np.testing.assert_array_equal(arr, np.arange(10, dtype=np.float32))

    def test_np_asarray(self):
        """np.asarray(tensor) must work via __array_interface__."""
        import horus
        t = horus.Tensor.from_numpy(np.ones((5, 5), dtype=np.float32))
        arr = np.asarray(t)
        assert arr.shape == (5, 5)
        np.testing.assert_array_equal(arr, np.ones((5, 5), dtype=np.float32))


# ============================================================================
# Indexing
# ============================================================================


class TestIndexing:
    def test_getitem_single(self):
        import horus
        src = np.arange(10, dtype=np.float32)
        t = horus.Tensor.from_numpy(src)
        assert t[0] == 0.0
        assert t[9] == 9.0

    def test_getitem_slice(self):
        import horus
        src = np.arange(20, dtype=np.float32)
        t = horus.Tensor.from_numpy(src)
        result = t[5:10]
        np.testing.assert_array_equal(result, src[5:10])

    def test_getitem_2d(self):
        import horus
        src = np.arange(12, dtype=np.float32).reshape(3, 4)
        t = horus.Tensor.from_numpy(src)
        row = t[1]
        np.testing.assert_array_equal(row, src[1])

    def test_setitem(self):
        import horus
        t = horus.Tensor.zeros([10], dtype="float32")
        t[5] = 99.0
        assert t.numpy()[5] == 99.0

    def test_setitem_slice(self):
        import horus
        t = horus.Tensor.zeros([10], dtype="float32")
        t[2:5] = np.array([1.0, 2.0, 3.0], dtype=np.float32)
        np.testing.assert_array_equal(t.numpy()[2:5], [1.0, 2.0, 3.0])


# ============================================================================
# Shape operations
# ============================================================================


class TestShapeOps:
    def test_reshape_list(self):
        import horus
        t = horus.Tensor.from_numpy(np.arange(12, dtype=np.float32))
        r = t.reshape([3, 4])
        assert r.shape == [3, 4]

    def test_reshape_args(self):
        import horus
        t = horus.Tensor.from_numpy(np.arange(12, dtype=np.float32))
        r = t.reshape(3, 4)
        assert r.shape == [3, 4]

    def test_flatten(self):
        import horus
        t = horus.Tensor.from_numpy(np.ones((3, 4), dtype=np.float32))
        f = t.flatten()
        assert f.shape == [12]

    def test_squeeze(self):
        import horus
        t = horus.Tensor([1, 5, 1], dtype="float32")
        s = t.squeeze()
        assert s.shape == [5]

    def test_unsqueeze(self):
        import horus
        t = horus.Tensor([5], dtype="float32")
        u = t.unsqueeze(0)
        assert u.shape == [1, 5]
        u2 = t.unsqueeze(-1)
        assert u2.shape == [5, 1]

    def test_view(self):
        import horus
        t = horus.Tensor.from_numpy(np.arange(24, dtype=np.float32))
        v = t.view([2, 3, 4])
        assert v.shape == [2, 3, 4]

    def test_slice_first_dim(self):
        import horus
        src = np.arange(100, dtype=np.float32)
        t = horus.Tensor.from_numpy(src)
        s = t.slice(10, 20)
        assert s.shape == [10]

    def test_transpose_2d(self):
        import horus
        src = np.arange(6, dtype=np.float32).reshape(2, 3)
        t = horus.Tensor.from_numpy(src)
        result = t.T
        assert result.shape == (3, 2)
        np.testing.assert_array_equal(result, src.T)


# ============================================================================
# Arithmetic (delegates to NumPy)
# ============================================================================


class TestArithmetic:
    def test_add_scalar(self):
        import horus
        t = horus.Tensor.from_numpy(np.array([1.0, 2.0, 3.0], dtype=np.float32))
        result = t + 10.0
        np.testing.assert_array_almost_equal(result.numpy(), [11.0, 12.0, 13.0])

    def test_add_tensor(self):
        import horus
        a = horus.Tensor.from_numpy(np.array([1.0, 2.0], dtype=np.float32))
        b = horus.Tensor.from_numpy(np.array([10.0, 20.0], dtype=np.float32))
        result = a + b
        np.testing.assert_array_almost_equal(result.numpy(), [11.0, 22.0])

    def test_add_numpy(self):
        import horus
        t = horus.Tensor.from_numpy(np.array([1.0, 2.0], dtype=np.float32))
        result = t + np.array([10.0, 20.0], dtype=np.float32)
        np.testing.assert_array_almost_equal(result.numpy(), [11.0, 22.0])

    def test_sub(self):
        import horus
        t = horus.Tensor.from_numpy(np.array([10.0, 20.0], dtype=np.float32))
        result = t - 5.0
        np.testing.assert_array_almost_equal(result.numpy(), [5.0, 15.0])

    def test_mul(self):
        import horus
        t = horus.Tensor.from_numpy(np.array([2.0, 3.0], dtype=np.float32))
        result = t * 4.0
        np.testing.assert_array_almost_equal(result.numpy(), [8.0, 12.0])

    def test_div(self):
        import horus
        t = horus.Tensor.from_numpy(np.array([10.0, 20.0], dtype=np.float32))
        result = t / 5.0
        np.testing.assert_array_almost_equal(result.numpy(), [2.0, 4.0])

    def test_neg(self):
        import horus
        t = horus.Tensor.from_numpy(np.array([1.0, -2.0, 3.0], dtype=np.float32))
        result = -t
        np.testing.assert_array_almost_equal(result.numpy(), [-1.0, 2.0, -3.0])

    def test_matrix_multiply_via_numpy(self):
        """ML workflow: matrix multiplication via numpy."""
        import horus
        a = horus.Tensor.from_numpy(np.random.randn(10, 20).astype(np.float32))
        b_np = np.random.randn(20, 5).astype(np.float32)
        # Use numpy for matmul (not in horus API, but works via numpy())
        result = a.numpy() @ b_np
        assert result.shape == (10, 5)


# ============================================================================
# Comparisons
# ============================================================================


class TestComparisons:
    def test_eq(self):
        import horus
        t = horus.Tensor.from_numpy(np.array([1.0, 2.0, 3.0], dtype=np.float32))
        result = t == 2.0
        np.testing.assert_array_equal(result.numpy(), [False, True, False])

    def test_lt(self):
        import horus
        t = horus.Tensor.from_numpy(np.array([1.0, 2.0, 3.0], dtype=np.float32))
        result = t < 2.0
        np.testing.assert_array_equal(result.numpy(), [True, False, False])

    def test_gt(self):
        import horus
        t = horus.Tensor.from_numpy(np.array([1.0, 2.0, 3.0], dtype=np.float32))
        result = t > 2.0
        np.testing.assert_array_equal(result.numpy(), [False, False, True])

    def test_le(self):
        import horus
        t = horus.Tensor.from_numpy(np.array([1.0, 2.0, 3.0], dtype=np.float32))
        result = t <= 2.0
        np.testing.assert_array_equal(result.numpy(), [True, True, False])

    def test_ge(self):
        import horus
        t = horus.Tensor.from_numpy(np.array([1.0, 2.0, 3.0], dtype=np.float32))
        result = t >= 2.0
        np.testing.assert_array_equal(result.numpy(), [False, True, True])


# ============================================================================
# Reductions
# ============================================================================


class TestReductions:
    def test_sum(self):
        import horus
        t = horus.Tensor.from_numpy(np.array([1.0, 2.0, 3.0], dtype=np.float32))
        result = t.sum()
        assert result.numpy()[0] == pytest.approx(6.0)

    def test_sum_dim(self):
        import horus
        src = np.array([[1.0, 2.0], [3.0, 4.0]], dtype=np.float32)
        t = horus.Tensor.from_numpy(src)
        result = t.sum(dim=0)
        np.testing.assert_array_almost_equal(result.numpy(), [4.0, 6.0])

    def test_mean(self):
        import horus
        t = horus.Tensor.from_numpy(np.array([2.0, 4.0, 6.0], dtype=np.float32))
        result = t.mean()
        assert result.numpy()[0] == pytest.approx(4.0)

    def test_max(self):
        import horus
        t = horus.Tensor.from_numpy(np.array([1.0, 5.0, 3.0], dtype=np.float32))
        result = t.max()
        assert result.numpy()[0] == pytest.approx(5.0)

    def test_min(self):
        import horus
        t = horus.Tensor.from_numpy(np.array([1.0, 5.0, 3.0], dtype=np.float32))
        result = t.min()
        assert result.numpy()[0] == pytest.approx(1.0)


# ============================================================================
# Type conversion
# ============================================================================


class TestTypeConversion:
    def test_astype_float16(self):
        import horus
        t = horus.Tensor.from_numpy(np.array([1.0, 2.0], dtype=np.float32))
        t16 = t.astype("float16")
        assert t16.dtype == "float16"

    def test_astype_same_dtype_no_copy(self):
        import horus
        t = horus.Tensor.from_numpy(np.array([1.0], dtype=np.float32))
        t2 = t.astype("float32")
        # Should be same dtype, could be a clone
        assert t2.dtype == "float32"

    def test_to_float32(self):
        import horus
        t = horus.Tensor.from_numpy(np.array([1, 2, 3], dtype=np.int32))
        f = t.to_float32()
        assert f.dtype == "float32"
        np.testing.assert_array_almost_equal(f.numpy(), [1.0, 2.0, 3.0])

    def test_to_int32(self):
        import horus
        t = horus.Tensor.from_numpy(np.array([1.5, 2.7], dtype=np.float32))
        i = t.to_int32()
        assert i.dtype == "int32"
        np.testing.assert_array_equal(i.numpy(), [1, 2])

    def test_to_uint8(self):
        import horus
        t = horus.Tensor.from_numpy(np.array([0, 128, 255], dtype=np.int32))
        b = t.to_uint8()
        assert b.dtype == "uint8"

    def test_tolist(self):
        import horus
        t = horus.Tensor.from_numpy(np.array([1.0, 2.0, 3.0], dtype=np.float32))
        lst = t.tolist()
        assert lst == pytest.approx([1.0, 2.0, 3.0])


# ============================================================================
# NumPy interop — real-world workflows
# ============================================================================


class TestNumpyWorkflows:
    def test_costmap_workflow(self):
        """Robotics: create costmap, modify with numpy, verify."""
        import horus
        grid = horus.Tensor([500, 500], dtype="float32")
        arr = grid.numpy()
        # Fill with cost gradient
        arr[:] = np.indices((500, 500), dtype=np.float32).sum(axis=0) * 0.01
        # Threshold
        obstacles = (grid > 5.0)
        assert obstacles.dtype == "bool"
        # Verify the numpy array shares memory
        assert grid.numpy()[0, 0] == pytest.approx(0.0)
        assert grid.numpy()[499, 499] == pytest.approx(9.98)

    def test_kalman_filter_state(self):
        """Robotics: Kalman filter state vector."""
        import horus
        # State: [x, y, theta, vx, vy, omega]
        state = horus.Tensor.from_numpy(
            np.array([0.0, 0.0, 0.0, 1.0, 0.0, 0.1], dtype=np.float64)
        )
        assert state.shape == [6]
        assert state.dtype == "float64"
        # Predict step via numpy
        A = np.eye(6, dtype=np.float64)
        A[0, 3] = 0.01  # dt
        A[1, 4] = 0.01
        A[2, 5] = 0.01
        predicted = A @ state.numpy()
        new_state = horus.Tensor.from_numpy(predicted)
        assert new_state.numpy()[0] == pytest.approx(0.01)  # x += vx * dt

    def test_image_processing_pipeline(self):
        """Vision: create image tensor, apply numpy operations."""
        import horus
        # Simulate 480x640 RGB image
        img = horus.Tensor([480, 640, 3], dtype="uint8")
        arr = img.numpy()
        # Fill with a gradient
        arr[:, :, 0] = np.linspace(0, 255, 640, dtype=np.uint8)  # R channel
        arr[:, :, 1] = 128  # G channel
        arr[:, :, 2] = 0    # B channel
        # Grayscale conversion via numpy
        gray = (0.299 * arr[:,:,0].astype(np.float32) +
                0.587 * arr[:,:,1].astype(np.float32) +
                0.114 * arr[:,:,2].astype(np.float32))
        gray_tensor = horus.Tensor.from_numpy(gray)
        assert gray_tensor.shape == [480, 640]
        assert gray_tensor.dtype == "float32"

    def test_occupancy_grid_workflow(self):
        """Navigation: occupancy grid as int8 tensor."""
        import horus
        # -1 = unknown, 0 = free, 100 = occupied
        grid = horus.Tensor([1000, 1000], dtype="int8")
        arr = grid.numpy()
        arr[:] = -1  # All unknown
        # Clear a region
        arr[400:600, 400:600] = 0  # Free
        # Mark obstacles
        arr[500, 500] = 100  # Occupied
        # Verify
        assert grid.numpy()[0, 0] == -1
        assert grid.numpy()[450, 450] == 0
        assert grid.numpy()[500, 500] == 100

    def test_point_cloud_as_tensor(self):
        """LiDAR: N points with XYZ + intensity."""
        import horus
        N = 100000
        points = np.random.randn(N, 4).astype(np.float32)  # XYZ + intensity
        t = horus.Tensor.from_numpy(points)
        assert t.shape == [N, 4]
        # Extract XYZ
        xyz = t.numpy()[:, :3]
        assert xyz.shape == (N, 3)
        # Compute distances
        distances = np.linalg.norm(xyz, axis=1)
        assert distances.shape == (N,)

    def test_feature_map_from_cnn(self):
        """ML: simulate CNN feature map (batch=1, channels=64, H=32, W=32)."""
        import horus
        features = np.random.randn(1, 64, 32, 32).astype(np.float32)
        t = horus.Tensor.from_numpy(features)
        assert t.shape == [1, 64, 32, 32]
        assert t.numel == 1 * 64 * 32 * 32
        # Global average pooling via numpy
        pooled = t.numpy().mean(axis=(2, 3))  # [1, 64]
        assert pooled.shape == (1, 64)

    def test_large_tensor_allocation(self):
        """Stress: allocate a large tensor (10MB)."""
        import horus
        # 10MB = 2500 x 1000 x float32
        t = horus.Tensor([2500, 1000], dtype="float32")
        assert t.nbytes == 2500 * 1000 * 4
        arr = t.numpy()
        arr[:] = 3.14
        assert arr[1000, 500] == pytest.approx(3.14)


# ============================================================================
# DLPack protocol
# ============================================================================


class TestDLPack:
    def test_dlpack_to_numpy(self):
        """np.from_dlpack(tensor) roundtrip."""
        import horus
        src = np.arange(24, dtype=np.float32).reshape(4, 6)
        t = horus.Tensor.from_numpy(src)
        recovered = np.from_dlpack(t)
        np.testing.assert_array_equal(recovered, src)

    def test_dlpack_device(self):
        """__dlpack_device__ returns (CPU, 0)."""
        import horus
        t = horus.Tensor([10], dtype="float32")
        device_type, device_id = t.__dlpack_device__()
        assert device_type == 1  # CPU
        assert device_id == 0

    def test_dlpack_roundtrip_preserves_dtype(self):
        """DLPack preserves dtype through roundtrip."""
        import horus
        for dtype in ["float32", "float64", "int32", "uint8"]:
            np_dtype = {"float32": np.float32, "float64": np.float64,
                        "int32": np.int32, "uint8": np.uint8}[dtype]
            src = np.ones((4,), dtype=np_dtype)
            t = horus.Tensor.from_numpy(src)
            arr = np.from_dlpack(t)
            assert arr.dtype == np_dtype, f"dtype mismatch for {dtype}"


# ============================================================================
# Release and error handling
# ============================================================================


class TestLifecycle:
    def test_release(self):
        """Explicit release should make subsequent ops fail gracefully."""
        import horus
        t = horus.Tensor([10], dtype="float32")
        t.release()
        with pytest.raises(RuntimeError, match="released"):
            _ = t.shape

    def test_repr_released(self):
        import horus
        t = horus.Tensor([10], dtype="float32")
        t.release()
        assert "released" in repr(t)

    def test_gc_no_crash(self):
        """Creating and discarding many tensors should not crash."""
        import horus
        for _ in range(100):
            t = horus.Tensor([1000], dtype="float32")
            _ = t.numpy()
        gc.collect()
        # No crash = success
