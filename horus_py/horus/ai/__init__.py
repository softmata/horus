"""
HORUS AI - Machine Learning Integration for Robotics

Provides a clean, Pythonic API for ML model loading, inference,
and preprocessing (PyTorch, TensorFlow, ONNX).

Example:
    from horus.ai import Model, device
    import numpy as np

    # Load model
    model = Model.load("yolov8n.onnx", device="cuda:0")

    # Run inference (returns np.ndarray)
    output = model(np.zeros((1, 3, 640, 640), dtype=np.float32))

    # Preprocessing pipelines
    from horus.ai import transforms
    preprocess = transforms.imagenet_preprocess()
    processed = preprocess(image)
"""

from .device import device, Device, get_default_device, set_default_device
from .model import Model
from .registry import ModelRegistry, ModelEntry
from .node import mlnode, MLNodeConfig
from . import transforms

# Re-export GPU utility functions from Rust backend
try:
    from horus._horus import (
        cuda_is_available,
        cuda_device_count,
    )
except ImportError:
    cuda_is_available = lambda: False
    cuda_device_count = lambda: 0

__all__ = [
    # Core classes
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
    # GPU utilities
    "cuda_is_available",
    "cuda_device_count",
]
