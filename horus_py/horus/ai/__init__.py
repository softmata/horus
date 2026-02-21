"""
HORUS AI - Machine Learning Integration for Robotics

Provides a clean, Pythonic API for ML model loading, tensor operations,
and framework interop (PyTorch, JAX, TensorFlow, ONNX).

Example:
    from horus.ai import Tensor, Model, device

    # Load model
    model = Model.load("yolov8n.onnx", device="cuda:0")

    # Run inference
    output = model(input_tensor)

    # Framework interop (zero-copy)
    torch_tensor = tensor.torch()
    np_array = tensor.numpy()

    # Preprocessing pipelines
    from horus.ai import transforms
    preprocess = transforms.imagenet_preprocess()
    tensor = preprocess(image)
"""

from .tensor import Tensor
from .device import device, Device, get_default_device, set_default_device
from .model import Model
from .registry import ModelRegistry, ModelEntry
from .node import mlnode, MLNodeConfig
from . import transforms

# Re-export from Rust backend
try:
    from horus._horus import (
        TensorPool,
        TensorHandle,
        cuda_is_available,
        cuda_device_count,
    )
except ImportError:
    # Mock for testing
    from horus import TensorPool, TensorHandle
    cuda_is_available = lambda: False
    cuda_device_count = lambda: 0

__all__ = [
    # Core classes
    "Tensor",
    "Model",
    "ModelRegistry",
    "ModelEntry",
    "Device",
    "device",
    "mlnode",
    "MLNodeConfig",
    # Device utilities
    "get_default_device",
    "set_default_device",
    # Transforms
    "transforms",
    # Rust backend
    "TensorPool",
    "TensorHandle",
    "cuda_is_available",
    "cuda_device_count",
]
