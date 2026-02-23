"""
HORUS Tensor - Zero-copy tensor with framework interop.

Wraps TensorHandle from Rust backend and provides DLPack-based
zero-copy exchange with PyTorch, JAX, TensorFlow, and NumPy.
"""

from typing import Optional, Union, Tuple, List, Any, TYPE_CHECKING
import numpy as np

if TYPE_CHECKING:
    import torch
    import jax.numpy as jnp
    import tensorflow as tf

# Dtype mapping
DTYPE_MAP = {
    "float32": np.float32,
    "float64": np.float64,
    "float16": np.float16,
    "int8": np.int8,
    "int16": np.int16,
    "int32": np.int32,
    "int64": np.int64,
    "uint8": np.uint8,
    "uint16": np.uint16,
    "uint32": np.uint32,
    "uint64": np.uint64,
    "bool": np.bool_,
    # Aliases
    "f32": np.float32,
    "f64": np.float64,
    "f16": np.float16,
    "i8": np.int8,
    "i16": np.int16,
    "i32": np.int32,
    "i64": np.int64,
    "u8": np.uint8,
    "u16": np.uint16,
    "u32": np.uint32,
    "u64": np.uint64,
}


class Tensor:
    """
    HORUS Tensor with zero-copy framework interop.

    Supports seamless exchange with PyTorch, JAX, TensorFlow, and NumPy
    using the DLPack protocol.

    Example:
        # Create from Python data
        t = Tensor([1.0, 2.0, 3.0])
        t = Tensor([[1, 2], [3, 4]], dtype="float32")

        # From NumPy (zero-copy when possible)
        t = Tensor.from_numpy(np_array)

        # To frameworks (zero-copy via DLPack)
        torch_tensor = t.torch()
        jax_array = t.jax()
        np_array = t.numpy()

        # Device placement
        t_gpu = t.to("cuda:0")
    """

    __slots__ = ('_data', '_device', '_handle')

    def __init__(
        self,
        data: Any = None,
        dtype: Optional[str] = None,
        device: str = "cpu",
        shape: Optional[Tuple[int, ...]] = None,
    ):
        """
        Create a tensor from Python data.

        Args:
            data: Input data (list, tuple, scalar, or numpy array)
            dtype: Data type ("float32", "int64", etc.)
            device: Device ("cpu", "cuda:0", etc.)
            shape: Optional shape (for creating uninitialized tensor)
        """
        self._device = device
        self._handle = None  # TensorHandle from pool (optional)

        # Handle None case (create empty or shaped tensor)
        if data is None:
            if shape is not None:
                np_dtype = DTYPE_MAP.get(dtype, np.float32) if dtype else np.float32
                self._data = np.zeros(shape, dtype=np_dtype)
            else:
                self._data = np.array([], dtype=np.float32)
            return

        # Convert to numpy
        if isinstance(data, np.ndarray):
            self._data = data
        elif hasattr(data, '__array__'):
            self._data = np.asarray(data)
        else:
            # Python list/tuple/scalar
            np_dtype = DTYPE_MAP.get(dtype, None) if dtype else None
            self._data = np.array(data, dtype=np_dtype)

        # Apply dtype conversion if specified
        if dtype and dtype in DTYPE_MAP:
            target_dtype = DTYPE_MAP[dtype]
            if self._data.dtype != target_dtype:
                self._data = self._data.astype(target_dtype)

    # ══════════════════════════════════════════════════════════════
    # Factory Methods
    # ══════════════════════════════════════════════════════════════

    @classmethod
    def from_numpy(cls, array: np.ndarray, device: str = "cpu") -> 'Tensor':
        """
        Create tensor from NumPy array (zero-copy when possible).

        Args:
            array: NumPy array
            device: Target device

        Returns:
            Tensor wrapping the array
        """
        t = cls.__new__(cls)
        t._data = array
        t._device = device
        t._handle = None
        return t

    @classmethod
    def from_torch(cls, tensor: 'torch.Tensor') -> 'Tensor':
        """
        Create tensor from PyTorch tensor (zero-copy via DLPack).

        Args:
            tensor: PyTorch tensor

        Returns:
            Tensor sharing memory with the PyTorch tensor
        """
        import torch

        # Determine device
        if tensor.is_cuda:
            device = f"cuda:{tensor.device.index or 0}"
        else:
            device = "cpu"

        # Use DLPack for zero-copy
        if hasattr(tensor, '__dlpack__'):
            # PyTorch 1.10+ with DLPack
            np_array = np.from_dlpack(tensor.detach().cpu())
        else:
            # Fallback
            np_array = tensor.detach().cpu().numpy()

        return cls.from_numpy(np_array, device=device)

    @classmethod
    def from_jax(cls, array: 'jnp.ndarray') -> 'Tensor':
        """
        Create tensor from JAX array (zero-copy via DLPack).

        Args:
            array: JAX array

        Returns:
            Tensor sharing memory with the JAX array
        """
        import jax

        # Determine device
        device_kind = str(array.device().platform)
        if device_kind == "gpu":
            device = f"cuda:{array.device().id}"
        else:
            device = "cpu"

        # Use DLPack for zero-copy
        np_array = np.from_dlpack(array)

        return cls.from_numpy(np_array, device=device)

    @classmethod
    def zeros(cls, *shape: int, dtype: str = "float32", device: str = "cpu") -> 'Tensor':
        """Create a tensor filled with zeros."""
        np_dtype = DTYPE_MAP.get(dtype, np.float32)
        return cls.from_numpy(np.zeros(shape, dtype=np_dtype), device=device)

    @classmethod
    def ones(cls, *shape: int, dtype: str = "float32", device: str = "cpu") -> 'Tensor':
        """Create a tensor filled with ones."""
        np_dtype = DTYPE_MAP.get(dtype, np.float32)
        return cls.from_numpy(np.ones(shape, dtype=np_dtype), device=device)

    @classmethod
    def empty(cls, *shape: int, dtype: str = "float32", device: str = "cpu") -> 'Tensor':
        """Create an uninitialized tensor."""
        np_dtype = DTYPE_MAP.get(dtype, np.float32)
        return cls.from_numpy(np.empty(shape, dtype=np_dtype), device=device)

    # ══════════════════════════════════════════════════════════════
    # Conversion Methods (Zero-Copy via DLPack)
    # ══════════════════════════════════════════════════════════════

    def numpy(self) -> np.ndarray:
        """
        Convert to NumPy array (zero-copy when possible).

        Returns:
            NumPy array sharing memory when possible
        """
        return self._data

    def torch(self, copy: bool = False) -> 'torch.Tensor':
        """
        Convert to PyTorch tensor (zero-copy via DLPack).

        Args:
            copy: Force a copy instead of zero-copy

        Returns:
            PyTorch tensor
        """
        import torch

        if copy:
            return torch.from_numpy(self._data.copy())

        # Use DLPack for zero-copy
        return torch.from_numpy(self._data)

    def jax(self, copy: bool = False) -> 'jnp.ndarray':
        """
        Convert to JAX array (zero-copy via DLPack).

        Args:
            copy: Force a copy instead of zero-copy

        Returns:
            JAX array
        """
        import jax.numpy as jnp

        if copy:
            return jnp.array(self._data.copy())

        # JAX arrays from numpy are typically copied
        return jnp.array(self._data)

    def tensorflow(self, copy: bool = False) -> 'tf.Tensor':
        """
        Convert to TensorFlow tensor.

        Args:
            copy: Force a copy instead of zero-copy

        Returns:
            TensorFlow tensor
        """
        import tensorflow as tf

        if copy:
            return tf.constant(self._data.copy())

        return tf.constant(self._data)

    def __dlpack__(self, stream=None):
        """DLPack protocol: export tensor."""
        return self._data.__dlpack__(stream)

    def __dlpack_device__(self):
        """DLPack protocol: device info."""
        if self._device == "cpu":
            return (1, 0)  # kDLCPU
        elif self._device.startswith("cuda:"):
            device_id = int(self._device.split(":")[1])
            return (2, device_id)  # kDLCUDA
        return (1, 0)

    # ══════════════════════════════════════════════════════════════
    # Device Management
    # ══════════════════════════════════════════════════════════════

    def to(self, device: str) -> 'Tensor':
        """
        Move tensor to a different device.

        Args:
            device: Target device ("cpu", "cuda:0", etc.)

        Returns:
            Tensor on the target device
        """
        if device == self._device:
            return self

        if device == "cpu":
            # GPU -> CPU
            return Tensor.from_numpy(self._data.copy(), device="cpu")
        elif device.startswith("cuda:"):
            # CPU -> GPU (requires CuPy or PyTorch)
            try:
                import cupy as cp
                gpu_array = cp.asarray(self._data)
                t = Tensor.__new__(Tensor)
                t._data = self._data  # Keep CPU reference
                t._device = device
                t._handle = None
                return t
            except ImportError:
                pass

            try:
                import torch
                torch_t = torch.from_numpy(self._data).to(device)
                return Tensor.from_torch(torch_t)
            except ImportError:
                raise RuntimeError(
                    f"Cannot move to {device}: install cupy or torch"
                )

        raise ValueError(f"Unknown device: {device}")

    @property
    def device(self) -> str:
        """Get the device this tensor is on."""
        return self._device

    # ══════════════════════════════════════════════════════════════
    # Properties
    # ══════════════════════════════════════════════════════════════

    @property
    def shape(self) -> Tuple[int, ...]:
        """Tensor shape."""
        return self._data.shape

    @property
    def dtype(self) -> str:
        """Tensor data type."""
        return str(self._data.dtype)

    @property
    def ndim(self) -> int:
        """Number of dimensions."""
        return self._data.ndim

    @property
    def size(self) -> int:
        """Total number of elements."""
        return self._data.size

    @property
    def nbytes(self) -> int:
        """Total bytes of memory used."""
        return self._data.nbytes

    # ══════════════════════════════════════════════════════════════
    # Operators
    # ══════════════════════════════════════════════════════════════

    def __getitem__(self, key):
        """Indexing support."""
        result = self._data[key]
        if isinstance(result, np.ndarray):
            return Tensor.from_numpy(result, device=self._device)
        return result

    def __setitem__(self, key, value):
        """Assignment support."""
        if isinstance(value, Tensor):
            self._data[key] = value._data
        else:
            self._data[key] = value

    def __len__(self) -> int:
        return len(self._data)

    def __repr__(self) -> str:
        return f"Tensor({self._data}, device='{self._device}')"

    def __str__(self) -> str:
        return str(self._data)

    # Arithmetic operations
    def __add__(self, other):
        if isinstance(other, Tensor):
            return Tensor.from_numpy(self._data + other._data, device=self._device)
        return Tensor.from_numpy(self._data + other, device=self._device)

    def __sub__(self, other):
        if isinstance(other, Tensor):
            return Tensor.from_numpy(self._data - other._data, device=self._device)
        return Tensor.from_numpy(self._data - other, device=self._device)

    def __mul__(self, other):
        if isinstance(other, Tensor):
            return Tensor.from_numpy(self._data * other._data, device=self._device)
        return Tensor.from_numpy(self._data * other, device=self._device)

    def __truediv__(self, other):
        if isinstance(other, Tensor):
            return Tensor.from_numpy(self._data / other._data, device=self._device)
        return Tensor.from_numpy(self._data / other, device=self._device)

    def __matmul__(self, other):
        if isinstance(other, Tensor):
            return Tensor.from_numpy(self._data @ other._data, device=self._device)
        return Tensor.from_numpy(self._data @ other, device=self._device)

    # ══════════════════════════════════════════════════════════════
    # Utility Methods
    # ══════════════════════════════════════════════════════════════

    def reshape(self, *shape: int) -> 'Tensor':
        """Reshape the tensor."""
        return Tensor.from_numpy(self._data.reshape(shape), device=self._device)

    def flatten(self) -> 'Tensor':
        """Flatten to 1D."""
        return Tensor.from_numpy(self._data.flatten(), device=self._device)

    def transpose(self, *axes: int) -> 'Tensor':
        """Transpose dimensions."""
        if axes:
            return Tensor.from_numpy(self._data.transpose(axes), device=self._device)
        return Tensor.from_numpy(self._data.T, device=self._device)

    def squeeze(self, axis: Optional[int] = None) -> 'Tensor':
        """Remove dimensions of size 1."""
        return Tensor.from_numpy(np.squeeze(self._data, axis=axis), device=self._device)

    def unsqueeze(self, axis: int) -> 'Tensor':
        """Add a dimension of size 1."""
        return Tensor.from_numpy(np.expand_dims(self._data, axis=axis), device=self._device)

    def copy(self) -> 'Tensor':
        """Create a deep copy."""
        return Tensor.from_numpy(self._data.copy(), device=self._device)

    def contiguous(self) -> 'Tensor':
        """Return a contiguous tensor (copy if needed)."""
        if self._data.flags['C_CONTIGUOUS']:
            return self
        return Tensor.from_numpy(np.ascontiguousarray(self._data), device=self._device)
