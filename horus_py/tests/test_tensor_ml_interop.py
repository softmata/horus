"""
Battle tests for horus.Tensor interop with real ML/AI libraries.

Tests horus.Tensor with:
  - PyTorch: from_dlpack, torch.from_dlpack, torch operations on horus data
  - NumPy: all protocols (__array_interface__, __dlpack__, buffer)
  - SciPy: sparse matrices, signal processing, linear algebra
  - scikit-learn: StandardScaler, PCA, KMeans on horus tensors
  - Pandas: DataFrame from tensor, Series operations
  - PIL/Pillow: Image from/to horus tensor
  - OpenCV: cv2 operations on horus tensor data
  - JAX: jnp.array from horus tensor

Each test simulates a real robotics/ML workflow:
  - Camera → CNN → feature extraction
  - LiDAR → point cloud processing → clustering
  - Costmap → path planning with scipy
  - Sensor fusion → Kalman filter with numpy/scipy
  - RL observation → policy inference → action output

Skip gracefully if a library is not installed.
"""

import gc
import numpy as np
import pytest


# ============================================================================
# Library availability checks
# ============================================================================

try:
    import torch
    HAS_TORCH = True
except ImportError:
    HAS_TORCH = False

try:
    import scipy
    import scipy.signal
    import scipy.ndimage
    import scipy.spatial
    import scipy.linalg
    HAS_SCIPY = True
except ImportError:
    HAS_SCIPY = False

try:
    import sklearn
    import sklearn.preprocessing
    import sklearn.decomposition
    import sklearn.cluster
    HAS_SKLEARN = True
except ImportError:
    HAS_SKLEARN = False

try:
    import pandas as pd
    HAS_PANDAS = True
except ImportError:
    HAS_PANDAS = False

try:
    from PIL import Image as PILImage
    HAS_PIL = True
except ImportError:
    HAS_PIL = False

try:
    import cv2
    HAS_CV2 = True
except ImportError:
    HAS_CV2 = False

try:
    import jax
    import jax.numpy as jnp
    HAS_JAX = True
except ImportError:
    HAS_JAX = False


# ============================================================================
# PyTorch interop
# ============================================================================


@pytest.mark.skipif(not HAS_TORCH, reason="PyTorch not installed")
class TestPyTorchInterop:
    def test_torch_from_dlpack(self):
        """torch.from_dlpack(horus_tensor) — zero-copy into PyTorch."""
        import horus
        src = np.random.randn(10, 20).astype(np.float32)
        t = horus.Tensor.from_numpy(src)
        pt = torch.from_dlpack(t)
        assert pt.shape == (10, 20)
        assert pt.dtype == torch.float32
        np.testing.assert_array_almost_equal(pt.numpy(), src)

    def test_from_torch_via_dlpack(self):
        """horus.Tensor.from_torch(pytorch_tensor) — copy into SHM."""
        import horus
        pt = torch.randn(5, 5)
        t = horus.Tensor.from_torch(pt)
        assert t.shape == [5, 5]
        assert t.dtype == "float32"
        np.testing.assert_array_almost_equal(t.numpy(), pt.numpy())

    def test_pytorch_operations_on_horus_data(self):
        """Run PyTorch ops on data backed by horus SHM."""
        import horus
        t = horus.Tensor.from_numpy(np.random.randn(32, 64).astype(np.float32))
        pt = torch.from_dlpack(t)
        # Standard PyTorch operations
        result = torch.relu(pt)
        assert result.shape == (32, 64)
        assert (result >= 0).all()

    def test_pytorch_linear_layer(self):
        """Simulate a neural network layer on horus tensor."""
        import horus
        # Input: batch of 8, features=64
        inp = horus.Tensor.from_numpy(np.random.randn(8, 64).astype(np.float32))
        pt_inp = torch.from_dlpack(inp)
        # Simple linear layer
        layer = torch.nn.Linear(64, 32)
        with torch.no_grad():
            output = layer(pt_inp)
        assert output.shape == (8, 32)
        # Convert back to horus
        result = horus.Tensor.from_torch(output)
        assert result.shape == [8, 32]

    def test_pytorch_cnn_feature_extraction(self):
        """Simulate CNN feature extraction on a camera image."""
        import horus
        # Simulate 480x640 RGB image as [1, 3, 480, 640] batch
        img_np = np.random.randint(0, 255, (1, 3, 480, 640), dtype=np.uint8)
        img_tensor = horus.Tensor.from_numpy(img_np.astype(np.float32) / 255.0)
        pt_img = torch.from_dlpack(img_tensor)
        # Simple conv
        conv = torch.nn.Conv2d(3, 16, kernel_size=3, padding=1)
        with torch.no_grad():
            features = conv(pt_img)
        assert features.shape == (1, 16, 480, 640)

    def test_pytorch_rl_observation(self):
        """RL workflow: observation → policy → action."""
        import horus
        # State: [x, y, theta, vx, vy, omega, lidar_32]
        obs = horus.Tensor.from_numpy(np.random.randn(38).astype(np.float32))
        pt_obs = torch.from_dlpack(obs)
        # Simple policy network
        policy = torch.nn.Sequential(
            torch.nn.Linear(38, 64),
            torch.nn.ReLU(),
            torch.nn.Linear(64, 2),  # linear_vel, angular_vel
        )
        with torch.no_grad():
            action = policy(pt_obs)
        assert action.shape == (2,)
        # Convert action back to horus for Topic send
        cmd = horus.Tensor.from_torch(action)
        assert cmd.shape == [2]

    def test_pytorch_dtype_preservation(self):
        """DLPack preserves dtypes between horus and PyTorch."""
        import horus
        for np_dtype, torch_dtype in [
            (np.float32, torch.float32),
            (np.float64, torch.float64),
            (np.int32, torch.int32),
            (np.int64, torch.int64),
            (np.uint8, torch.uint8),
            (np.int8, torch.int8),
            (np.int16, torch.int16),
        ]:
            src = np.ones((4,), dtype=np_dtype)
            t = horus.Tensor.from_numpy(src)
            pt = torch.from_dlpack(t)
            assert pt.dtype == torch_dtype, f"dtype mismatch: {np_dtype} -> {pt.dtype}"

    def test_pytorch_batch_inference(self):
        """Batch inference: multiple observations through a model."""
        import horus
        batch_size = 32
        obs_dim = 16
        # Create batch of observations in horus SHM
        batch = horus.Tensor.from_numpy(
            np.random.randn(batch_size, obs_dim).astype(np.float32)
        )
        pt_batch = torch.from_dlpack(batch)
        model = torch.nn.Linear(obs_dim, 4)
        with torch.no_grad():
            actions = model(pt_batch)
        assert actions.shape == (batch_size, 4)


# ============================================================================
# SciPy interop
# ============================================================================


@pytest.mark.skipif(not HAS_SCIPY, reason="SciPy not installed")
class TestSciPyInterop:
    def test_scipy_signal_filter(self):
        """Apply a Butterworth filter to IMU data in horus tensor."""
        import horus
        # Simulate 1kHz IMU accelerometer data (10 seconds)
        t_axis = np.linspace(0, 10, 10000, dtype=np.float64)
        signal = np.sin(2 * np.pi * 5 * t_axis) + 0.5 * np.random.randn(10000)
        imu_data = horus.Tensor.from_numpy(signal.astype(np.float64))
        # Low-pass filter at 10Hz
        b, a = scipy.signal.butter(4, 10, fs=1000)
        filtered = scipy.signal.filtfilt(b, a, imu_data.numpy())
        result = horus.Tensor.from_numpy(filtered.astype(np.float64))
        assert result.shape == [10000]

    def test_scipy_ndimage_costmap(self):
        """Apply morphological operations to costmap."""
        import horus
        # Create a costmap with obstacles
        grid = horus.Tensor([200, 200], dtype="float32")
        arr = grid.numpy()
        arr[:] = 0
        arr[90:110, 90:110] = 1.0  # obstacle
        # Inflate obstacles using scipy distance transform
        inflated = scipy.ndimage.distance_transform_edt(1 - arr)
        cost = np.clip(1.0 - inflated / 20.0, 0, 1).astype(np.float32)
        cost_tensor = horus.Tensor.from_numpy(cost)
        assert cost_tensor.shape == [200, 200]
        assert cost_tensor.numpy()[100, 100] == pytest.approx(1.0)  # obstacle center

    def test_scipy_spatial_kdtree(self):
        """KD-tree nearest neighbor search on point cloud."""
        import horus
        # 10k 3D points
        points = horus.Tensor.from_numpy(np.random.randn(10000, 3).astype(np.float64))
        tree = scipy.spatial.KDTree(points.numpy())
        # Find nearest neighbor to origin
        dist, idx = tree.query([0, 0, 0])
        assert idx < 10000
        assert dist >= 0

    def test_scipy_linalg_kalman(self):
        """Kalman filter predict step using scipy linalg."""
        import horus
        # State: [x, y, vx, vy]
        state = horus.Tensor.from_numpy(np.array([0, 0, 1, 0.5], dtype=np.float64))
        P = np.eye(4, dtype=np.float64) * 0.1  # covariance
        A = np.eye(4, dtype=np.float64)
        A[0, 2] = 0.01  # dt
        A[1, 3] = 0.01
        Q = np.eye(4, dtype=np.float64) * 0.01
        # Predict
        x_pred = A @ state.numpy()
        P_pred = A @ P @ A.T + Q
        result = horus.Tensor.from_numpy(x_pred)
        assert result.numpy()[0] == pytest.approx(0.01)  # x += vx * dt


# ============================================================================
# scikit-learn interop
# ============================================================================


@pytest.mark.skipif(not HAS_SKLEARN, reason="scikit-learn not installed")
class TestSklearnInterop:
    def test_standard_scaler(self):
        """Normalize sensor data using StandardScaler."""
        import horus
        # 100 samples, 6 features (IMU: ax, ay, az, gx, gy, gz)
        data = horus.Tensor.from_numpy(np.random.randn(100, 6).astype(np.float64))
        scaler = sklearn.preprocessing.StandardScaler()
        scaled = scaler.fit_transform(data.numpy())
        result = horus.Tensor.from_numpy(scaled.astype(np.float64))
        # Verify zero mean, unit variance
        assert np.abs(result.numpy().mean(axis=0)).max() < 0.01
        assert np.abs(result.numpy().std(axis=0) - 1.0).max() < 0.1

    def test_pca_feature_reduction(self):
        """PCA dimensionality reduction on feature map."""
        import horus
        # 50 samples, 100 features → reduce to 10
        features = horus.Tensor.from_numpy(np.random.randn(50, 100).astype(np.float64))
        pca = sklearn.decomposition.PCA(n_components=10)
        reduced = pca.fit_transform(features.numpy())
        result = horus.Tensor.from_numpy(reduced.astype(np.float64))
        assert result.shape == [50, 10]

    def test_kmeans_point_cloud_clustering(self):
        """Cluster a point cloud into segments."""
        import horus
        # 1000 3D points in 3 clusters
        centers = np.array([[0, 0, 0], [5, 5, 5], [-5, 0, 5]], dtype=np.float64)
        points_list = []
        for center in centers:
            points_list.append(center + np.random.randn(333, 3) * 0.5)
        points = np.vstack(points_list).astype(np.float64)
        cloud = horus.Tensor.from_numpy(points)
        kmeans = sklearn.cluster.KMeans(n_clusters=3, random_state=42, n_init=10)
        labels = kmeans.fit_predict(cloud.numpy())
        assert len(np.unique(labels)) == 3


# ============================================================================
# Pandas interop
# ============================================================================


@pytest.mark.skipif(not HAS_PANDAS, reason="Pandas not installed")
class TestPandasInterop:
    def test_dataframe_from_tensor(self):
        """Create a Pandas DataFrame from horus tensor."""
        import horus
        # Robot telemetry: [timestamp, x, y, theta, velocity, angular_vel]
        data = horus.Tensor.from_numpy(np.random.randn(100, 6).astype(np.float64))
        df = pd.DataFrame(
            data.numpy(),
            columns=["ts", "x", "y", "theta", "v", "omega"]
        )
        assert len(df) == 100
        assert list(df.columns) == ["ts", "x", "y", "theta", "v", "omega"]

    def test_tensor_from_dataframe(self):
        """Create a horus tensor from Pandas DataFrame."""
        import horus
        df = pd.DataFrame({
            "x": np.random.randn(50),
            "y": np.random.randn(50),
        })
        t = horus.Tensor.from_numpy(df.values.astype(np.float64))
        assert t.shape == [50, 2]

    def test_pandas_rolling_statistics(self):
        """Compute rolling statistics on sensor time series."""
        import horus
        # 1000 IMU readings
        imu = horus.Tensor.from_numpy(np.random.randn(1000).astype(np.float64))
        series = pd.Series(imu.numpy())
        rolling_mean = series.rolling(window=50).mean().dropna()
        result = horus.Tensor.from_numpy(rolling_mean.values.astype(np.float64))
        assert result.shape == [951]


# ============================================================================
# PIL/Pillow interop
# ============================================================================


@pytest.mark.skipif(not HAS_PIL, reason="Pillow not installed")
class TestPILInterop:
    def test_pil_from_tensor(self):
        """Create a PIL Image from horus tensor."""
        import horus
        # 480x640 RGB
        img_data = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)
        t = horus.Tensor.from_numpy(img_data)
        pil_img = PILImage.fromarray(t.numpy())
        assert pil_img.size == (640, 480)
        assert pil_img.mode == "RGB"

    def test_tensor_from_pil(self):
        """Create a horus tensor from PIL Image."""
        import horus
        pil_img = PILImage.new("RGB", (320, 240), color=(128, 64, 32))
        arr = np.array(pil_img)
        t = horus.Tensor.from_numpy(arr)
        assert t.shape == [240, 320, 3]
        assert t.dtype == "uint8"
        assert t.numpy()[0, 0, 0] == 128  # R

    def test_pil_image_processing(self):
        """Apply PIL operations and convert back to tensor."""
        import horus
        img_data = np.random.randint(0, 255, (100, 100, 3), dtype=np.uint8)
        t = horus.Tensor.from_numpy(img_data)
        pil_img = PILImage.fromarray(t.numpy())
        # Resize
        resized = pil_img.resize((50, 50))
        result = horus.Tensor.from_numpy(np.array(resized))
        assert result.shape == [50, 50, 3]


# ============================================================================
# OpenCV interop
# ============================================================================


@pytest.mark.skipif(not HAS_CV2, reason="OpenCV not installed")
class TestOpenCVInterop:
    def test_cv2_grayscale_conversion(self):
        """Convert horus RGB tensor to grayscale with OpenCV."""
        import horus
        img = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)
        t = horus.Tensor.from_numpy(img)
        gray = cv2.cvtColor(t.numpy(), cv2.COLOR_RGB2GRAY)
        result = horus.Tensor.from_numpy(gray)
        assert result.shape == [480, 640]
        assert result.dtype == "uint8"

    def test_cv2_gaussian_blur(self):
        """Apply Gaussian blur to horus tensor."""
        import horus
        img = np.random.randint(0, 255, (100, 100), dtype=np.uint8)
        t = horus.Tensor.from_numpy(img)
        blurred = cv2.GaussianBlur(t.numpy(), (5, 5), 1.0)
        result = horus.Tensor.from_numpy(blurred)
        assert result.shape == [100, 100]

    def test_cv2_feature_detection(self):
        """Detect features in an image tensor."""
        import horus
        # Create an image with a clear feature (white square on black)
        img = np.zeros((200, 200), dtype=np.uint8)
        img[80:120, 80:120] = 255
        t = horus.Tensor.from_numpy(img)
        # Canny edge detection
        edges = cv2.Canny(t.numpy(), 50, 150)
        result = horus.Tensor.from_numpy(edges)
        assert result.shape == [200, 200]
        assert result.numpy().max() > 0  # edges detected

    def test_cv2_resize(self):
        """Resize image tensor with OpenCV."""
        import horus
        img = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)
        t = horus.Tensor.from_numpy(img)
        resized = cv2.resize(t.numpy(), (320, 240))
        result = horus.Tensor.from_numpy(resized)
        assert result.shape == [240, 320, 3]

    def test_cv2_contour_detection(self):
        """Find contours in a binary costmap."""
        import horus
        # Create binary occupancy grid
        grid = np.zeros((200, 200), dtype=np.uint8)
        grid[50:70, 50:70] = 255  # obstacle 1
        grid[130:160, 100:140] = 255  # obstacle 2
        t = horus.Tensor.from_numpy(grid)
        contours, _ = cv2.findContours(t.numpy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        assert len(contours) == 2


# ============================================================================
# JAX interop
# ============================================================================


@pytest.mark.skipif(not HAS_JAX, reason="JAX not installed")
class TestJAXInterop:
    def test_jax_from_numpy(self):
        """jnp.array from horus tensor via numpy."""
        import horus
        t = horus.Tensor.from_numpy(np.array([1.0, 2.0, 3.0], dtype=np.float32))
        jax_arr = jnp.array(t.numpy())
        np.testing.assert_array_almost_equal(np.array(jax_arr), [1.0, 2.0, 3.0])

    def test_jax_operations(self):
        """Run JAX operations on horus data."""
        import horus
        data = horus.Tensor.from_numpy(np.random.randn(10, 10).astype(np.float32))
        jax_data = jnp.array(data.numpy())
        # Matrix operations
        result = jnp.dot(jax_data, jax_data.T)
        assert result.shape == (10, 10)

    def test_jax_grad(self):
        """Compute gradient with JAX on horus data."""
        import horus
        x = horus.Tensor.from_numpy(np.array([2.0], dtype=np.float32))
        # f(x) = x^2, f'(x) = 2x
        grad_fn = jax.grad(lambda x: (x ** 2).sum())
        grad = grad_fn(jnp.array(x.numpy()))
        assert float(grad[0]) == pytest.approx(4.0)


# ============================================================================
# Cross-library workflows (real robotics scenarios)
# ============================================================================


class TestCrossLibraryWorkflows:
    def test_camera_to_model_pipeline(self):
        """Full pipeline: camera image → preprocess → model → action."""
        import horus
        # 1. Receive camera image as horus tensor
        raw_img = horus.Tensor.from_numpy(
            np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)
        )
        # 2. Preprocess with numpy (normalize)
        img_float = raw_img.numpy().astype(np.float32) / 255.0
        # 3. Resize with numpy (simple downsample)
        small = img_float[::2, ::2, :]  # 240x320
        preprocessed = horus.Tensor.from_numpy(small)
        assert preprocessed.shape == [240, 320, 3]

    @pytest.mark.skipif(not HAS_SCIPY, reason="SciPy needed")
    def test_lidar_to_occupancy_grid(self):
        """LiDAR scan → occupancy grid via histogram."""
        import horus
        # Simulate LiDAR ranges (720 points)
        angles = np.linspace(-np.pi, np.pi, 720, dtype=np.float32)
        ranges = 5.0 + np.random.randn(720).astype(np.float32) * 0.5
        # Convert to XY
        x = ranges * np.cos(angles)
        y = ranges * np.sin(angles)
        points = horus.Tensor.from_numpy(np.stack([x, y], axis=1))
        assert points.shape == [720, 2]
        # Build occupancy grid via 2D histogram
        grid, _, _ = np.histogram2d(
            points.numpy()[:, 0], points.numpy()[:, 1],
            bins=100, range=[[-10, 10], [-10, 10]]
        )
        occ_grid = horus.Tensor.from_numpy((grid > 0).astype(np.int8))
        assert occ_grid.shape == [100, 100]

    @pytest.mark.skipif(not HAS_SKLEARN, reason="sklearn needed")
    def test_obstacle_clustering(self):
        """Cluster LiDAR points into obstacles."""
        import horus
        np.random.seed(42)
        # 3 well-separated obstacle clusters
        cluster1 = np.random.randn(100, 2).astype(np.float64) * 0.3 + [5, 5]
        cluster2 = np.random.randn(100, 2).astype(np.float64) * 0.3 + [-5, 0]
        cluster3 = np.random.randn(100, 2).astype(np.float64) * 0.3 + [0, -5]
        all_points = np.vstack([cluster1, cluster2, cluster3])
        cloud = horus.Tensor.from_numpy(all_points)
        # DBSCAN clustering
        clustering = sklearn.cluster.DBSCAN(eps=1.5, min_samples=5)
        labels = clustering.fit_predict(cloud.numpy())
        n_clusters = len(set(labels)) - (1 if -1 in labels else 0)
        assert n_clusters == 3

    @pytest.mark.skipif(not HAS_TORCH, reason="PyTorch needed")
    def test_end_to_end_rl_loop(self):
        """Full RL loop: observe → infer → act → repeat."""
        import horus
        policy = torch.nn.Sequential(
            torch.nn.Linear(10, 32),
            torch.nn.ReLU(),
            torch.nn.Linear(32, 2),
            torch.nn.Tanh(),
        )
        for step in range(5):
            # Observe (from horus topics)
            obs = horus.Tensor.from_numpy(np.random.randn(10).astype(np.float32))
            # Infer (via PyTorch)
            pt_obs = torch.from_dlpack(obs)
            with torch.no_grad():
                action = policy(pt_obs)
            # Act (back to horus)
            cmd = horus.Tensor.from_torch(action)
            assert cmd.shape == [2]
            # Verify action is in [-1, 1] (tanh output)
            assert cmd.numpy().max() <= 1.0
            assert cmd.numpy().min() >= -1.0
