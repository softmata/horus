"""
HORUS Device Management - CPU/GPU device handling.

Provides PyTorch-style device management with context managers
and automatic device detection.
"""

from typing import Optional, Union

# Thread-local default device
import threading
_local = threading.local()


def _get_default_device() -> str:
    """Get the current default device."""
    return getattr(_local, 'device', 'cpu')


def _set_default_device(device: str) -> None:
    """Set the current default device."""
    _local.device = device


class Device:
    """
    Device descriptor for tensor placement.

    Example:
        dev = Device("cuda:0")
        print(dev.type)   # "cuda"
        print(dev.index)  # 0

        # Check availability
        if Device.cuda_available():
            dev = Device("cuda:0")
    """

    __slots__ = ('_type', '_index')

    def __init__(self, device: Union[str, 'Device'] = "cpu"):
        """
        Create a device descriptor.

        Args:
            device: Device string ("cpu", "cuda:0", "cuda:1", etc.)
        """
        if isinstance(device, Device):
            self._type = device._type
            self._index = device._index
            return

        device = device.lower().strip()

        if device == "cpu":
            self._type = "cpu"
            self._index = 0
        elif device.startswith("cuda"):
            self._type = "cuda"
            if ":" in device:
                self._index = int(device.split(":")[1])
            else:
                self._index = 0
        elif device.startswith("gpu"):
            # Alias: gpu -> cuda
            self._type = "cuda"
            if ":" in device:
                self._index = int(device.split(":")[1])
            else:
                self._index = 0
        else:
            raise ValueError(f"Unknown device: {device}")

    @property
    def type(self) -> str:
        """Device type ("cpu" or "cuda")."""
        return self._type

    @property
    def index(self) -> int:
        """Device index (0 for CPU, GPU ID for CUDA)."""
        return self._index

    @property
    def is_cpu(self) -> bool:
        """Check if this is a CPU device."""
        return self._type == "cpu"

    @property
    def is_cuda(self) -> bool:
        """Check if this is a CUDA device."""
        return self._type == "cuda"

    def __str__(self) -> str:
        if self._type == "cpu":
            return "cpu"
        return f"{self._type}:{self._index}"

    def __repr__(self) -> str:
        return f"Device('{self}')"

    def __eq__(self, other) -> bool:
        if isinstance(other, str):
            other = Device(other)
        if isinstance(other, Device):
            return self._type == other._type and self._index == other._index
        return False

    def __hash__(self) -> int:
        return hash((self._type, self._index))

    # ══════════════════════════════════════════════════════════════
    # Static Methods
    # ══════════════════════════════════════════════════════════════

    @staticmethod
    def cuda_available() -> bool:
        """Check if CUDA is available."""
        try:
            from horus._horus import cuda_is_available
            return cuda_is_available()
        except ImportError:
            pass

        # Fallback: check PyTorch
        try:
            import torch
            return torch.cuda.is_available()
        except ImportError:
            pass

        # Fallback: check CuPy
        try:
            import cupy
            return True
        except ImportError:
            pass

        return False

    @staticmethod
    def cuda_device_count() -> int:
        """Get number of available CUDA devices."""
        try:
            from horus._horus import cuda_device_count
            return cuda_device_count()
        except ImportError:
            pass

        # Fallback: check PyTorch
        try:
            import torch
            return torch.cuda.device_count()
        except ImportError:
            pass

        # Fallback: check CuPy
        try:
            import cupy
            return cupy.cuda.runtime.getDeviceCount()
        except ImportError:
            pass

        return 0

    @staticmethod
    def current() -> 'Device':
        """Get the current default device."""
        return Device(_get_default_device())

    @staticmethod
    def cpu() -> 'Device':
        """Get CPU device."""
        return Device("cpu")

    @staticmethod
    def cuda(index: int = 0) -> 'Device':
        """Get CUDA device by index."""
        return Device(f"cuda:{index}")


class device:
    """
    Device context manager and utility.

    Example:
        # Context manager for default device
        with device("cuda:0"):
            model = Model.load("model.onnx")

        # Check availability
        if device.cuda_available():
            dev = device("cuda:0")

        # Get current device
        current = device.current()
    """

    def __init__(self, device_str: str = "cpu"):
        """
        Create a device context manager.

        Args:
            device_str: Device string ("cpu", "cuda:0", etc.)
        """
        self._device = Device(device_str)
        self._previous: Optional[str] = None

    def __enter__(self) -> 'Device':
        """Enter context: set as default device."""
        self._previous = _get_default_device()
        _set_default_device(str(self._device))
        return self._device

    def __exit__(self, exc_type, exc_val, exc_tb) -> None:
        """Exit context: restore previous default device."""
        if self._previous is not None:
            _set_default_device(self._previous)

    def __str__(self) -> str:
        return str(self._device)

    def __repr__(self) -> str:
        return f"device('{self._device}')"

    # ══════════════════════════════════════════════════════════════
    # Static Methods (mirror Device class)
    # ══════════════════════════════════════════════════════════════

    @staticmethod
    def cuda_available() -> bool:
        """Check if CUDA is available."""
        return Device.cuda_available()

    @staticmethod
    def cuda_device_count() -> int:
        """Get number of available CUDA devices."""
        return Device.cuda_device_count()

    @staticmethod
    def current() -> Device:
        """Get the current default device."""
        return Device.current()

    @staticmethod
    def cpu() -> Device:
        """Get CPU device."""
        return Device.cpu()

    @staticmethod
    def cuda(index: int = 0) -> Device:
        """Get CUDA device by index."""
        return Device.cuda(index)


def get_default_device() -> str:
    """Get the current default device string."""
    return _get_default_device()


def set_default_device(device_str: str) -> None:
    """Set the default device."""
    _set_default_device(device_str)
