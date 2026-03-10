"""
HORUS Runtime Messages - Zero-copy without Rust compilation

Uses numpy structured arrays for fixed-layout messages that can be
shared via memory without serialization.

Usage:
    from horus.msggen.runtime import define_message
    import numpy as np

    # Define a message type
    RobotStatus = define_message('RobotStatus', 'robot/status', [
        ('battery_level', np.float32),
        ('error_code', np.int32),
        ('is_active', np.bool_),
        ('timestamp', np.uint64),
    ])

    # Create instances
    status = RobotStatus(battery_level=85.0, error_code=0, is_active=True)

    # Access fields
    print(status.battery_level)  # 85.0
    status.error_code = 5

    # Get raw bytes for zero-copy IPC
    raw_bytes = status.to_bytes()

    # Reconstruct from bytes
    status2 = RobotStatus.from_bytes(raw_bytes)
"""

from typing import List, Tuple, Type, Any, Optional, Dict
import struct

# Try to import numpy - it's optional but provides better performance
try:
    import numpy as np
    HAS_NUMPY = True
except ImportError:
    HAS_NUMPY = False


# Type mapping for struct module (fallback when numpy not available)
STRUCT_TYPE_MAP = {
    'f32': ('f', 4),
    'f64': ('d', 8),
    'float32': ('f', 4),
    'float64': ('d', 8),
    'float': ('d', 8),
    'i8': ('b', 1),
    'i16': ('h', 2),
    'i32': ('i', 4),
    'i64': ('q', 8),
    'u8': ('B', 1),
    'u16': ('H', 2),
    'u32': ('I', 4),
    'u64': ('Q', 8),
    'int': ('q', 8),
    'uint': ('Q', 8),
    'bool': ('?', 1),
    'boolean': ('?', 1),
}

# NumPy dtype mapping
if HAS_NUMPY:
    NUMPY_TYPE_MAP = {
        'f32': np.float32,
        'f64': np.float64,
        'float32': np.float32,
        'float64': np.float64,
        'float': np.float64,
        'i8': np.int8,
        'i16': np.int16,
        'i32': np.int32,
        'i64': np.int64,
        'u8': np.uint8,
        'u16': np.uint16,
        'u32': np.uint32,
        'u64': np.uint64,
        'int': np.int64,
        'uint': np.uint64,
        'bool': np.bool_,
        'boolean': np.bool_,
    }


class RuntimeMessage:
    """
    Base class for runtime-defined messages.

    Uses struct module for fixed-layout binary serialization.
    This enables zero-copy IPC without Rust compilation.
    """

    __topic_name__: str = ""
    __struct_format__: str = ""
    __field_names__: List[str] = []
    __field_types__: List[str] = []
    __message_size__: int = 0

    def __init__(self, **kwargs):
        for name in self.__field_names__:
            if name in kwargs:
                setattr(self, name, kwargs[name])
            else:
                # Default to 0/False for the type
                setattr(self, name, 0)

    def to_bytes(self) -> bytes:
        """Serialize message to bytes for zero-copy IPC."""
        values = [getattr(self, name) for name in self.__field_names__]
        return struct.pack(self.__struct_format__, *values)

    @classmethod
    def from_bytes(cls, data: bytes) -> 'RuntimeMessage':
        """Deserialize message from bytes."""
        values = struct.unpack(cls.__struct_format__, data)
        kwargs = dict(zip(cls.__field_names__, values))
        return cls(**kwargs)

    @classmethod
    def message_size(cls) -> int:
        """Get the fixed size of this message in bytes."""
        return cls.__message_size__

    def __repr__(self) -> str:
        parts = []
        for name in self.__field_names__:
            value = getattr(self, name)
            if isinstance(value, float):
                parts.append(f"{name}={value:.3f}")
            else:
                parts.append(f"{name}={value}")
        return f"{self.__class__.__name__}({', '.join(parts)})"

    def __eq__(self, other) -> bool:
        if not isinstance(other, self.__class__):
            return False
        for name in self.__field_names__:
            if getattr(self, name) != getattr(other, name):
                return False
        return True


def define_message(
    name: str,
    topic: str,
    fields: List[Tuple[str, Any]],
) -> Type[RuntimeMessage]:
    """
    Define a custom message type at runtime.

    This creates a message class with fixed binary layout that can be
    used for zero-copy IPC without requiring Rust compilation.

    Args:
        name: Message class name (e.g., "RobotStatus")
        topic: Topic name (e.g., "robot/status")
        fields: List of (field_name, field_type) tuples
                Types can be strings ('f32', 'i32') or numpy dtypes

    Returns:
        A new message class

    Example:
        RobotStatus = define_message('RobotStatus', 'robot/status', [
            ('battery_level', 'f32'),
            ('error_code', 'i32'),
            ('is_active', 'bool'),
            ('timestamp', 'u64'),
        ])

        status = RobotStatus(battery_level=85.0, error_code=0, is_active=True)
    """
    field_names = []
    field_types = []
    struct_format = '<'  # Little-endian

    for field_name, field_type in fields:
        field_names.append(field_name)

        # Convert numpy dtype to string if needed
        if HAS_NUMPY and hasattr(field_type, 'name'):
            field_type = field_type.name

        # Convert to string type name
        type_str = str(field_type).lower().replace('numpy.', '').replace('<class \'', '').replace('\'>', '')

        # Map common numpy type names
        type_map = {
            'float32': 'f32',
            'float64': 'f64',
            'int8': 'i8',
            'int16': 'i16',
            'int32': 'i32',
            'int64': 'i64',
            'uint8': 'u8',
            'uint16': 'u16',
            'uint32': 'u32',
            'uint64': 'u64',
            'bool_': 'bool',
        }
        type_str = type_map.get(type_str, type_str)

        if type_str not in STRUCT_TYPE_MAP:
            raise ValueError(f"Unknown type: {field_type}. Valid types: {list(STRUCT_TYPE_MAP.keys())}")

        field_types.append(type_str)
        format_char, _ = STRUCT_TYPE_MAP[type_str]
        struct_format += format_char

    # Calculate message size
    message_size = struct.calcsize(struct_format)

    # Create the new class
    class_dict = {
        '__topic_name__': topic,
        '__struct_format__': struct_format,
        '__field_names__': field_names,
        '__field_types__': field_types,
        '__message_size__': message_size,
        '__slots__': field_names,  # Memory optimization
    }

    # Create and return the new class
    new_class = type(name, (RuntimeMessage,), class_dict)
    return new_class


# NumPy-based message for even better performance when numpy is available
if HAS_NUMPY:

    class NumpyMessage:
        """
        NumPy-based message with true zero-copy capability.

        Uses numpy structured array internally, which provides:
        - Fixed memory layout
        - Direct memory access
        - True zero-copy when sharing via memory-mapped files
        """

        __topic_name__: str = ""
        __dtype__: np.dtype = None
        __field_names__: List[str] = []

        def __init__(self, **kwargs):
            # Create numpy structured array with one element
            self._data = np.zeros(1, dtype=self.__dtype__)[0]
            for name, value in kwargs.items():
                if name in self.__field_names__:
                    self._data[name] = value

        def __getattr__(self, name: str):
            if name.startswith('_') or name not in self.__field_names__:
                raise AttributeError(f"'{type(self).__name__}' has no attribute '{name}'")
            return self._data[name].item()

        def __setattr__(self, name: str, value):
            if name.startswith('_'):
                super().__setattr__(name, value)
            elif name in self.__field_names__:
                self._data[name] = value
            else:
                raise AttributeError(f"'{type(self).__name__}' has no attribute '{name}'")

        def to_bytes(self) -> bytes:
            """Get raw bytes of the numpy array (zero-copy capable)."""
            return self._data.tobytes()

        @classmethod
        def from_bytes(cls, data: bytes) -> 'NumpyMessage':
            """Reconstruct from bytes."""
            arr = np.frombuffer(data, dtype=cls.__dtype__)
            instance = cls.__new__(cls)
            instance._data = arr[0].copy()
            return instance

        def to_numpy(self) -> np.ndarray:
            """Get the underlying numpy structured array."""
            return self._data

        @classmethod
        def message_size(cls) -> int:
            """Get the fixed size of this message in bytes."""
            return cls.__dtype__.itemsize

        def __repr__(self) -> str:
            parts = []
            for name in self.__field_names__:
                value = getattr(self, name)
                if isinstance(value, (float, np.floating)):
                    parts.append(f"{name}={value:.3f}")
                else:
                    parts.append(f"{name}={value}")
            return f"{self.__class__.__name__}({', '.join(parts)})"


    def define_numpy_message(
        name: str,
        topic: str,
        fields: List[Tuple[str, Any]],
    ) -> Type[NumpyMessage]:
        """
        Define a numpy-based message type for maximum performance.

        Args:
            name: Message class name
            topic: Topic name
            fields: List of (field_name, numpy_dtype) tuples

        Example:
            import numpy as np

            RobotStatus = define_numpy_message('RobotStatus', 'robot/status', [
                ('battery_level', np.float32),
                ('error_code', np.int32),
                ('is_active', np.bool_),
                ('timestamp', np.uint64),
            ])
        """
        field_names = []
        dtype_fields = []

        for field_name, field_type in fields:
            field_names.append(field_name)

            # Convert string type to numpy dtype if needed
            if isinstance(field_type, str):
                if field_type not in NUMPY_TYPE_MAP:
                    raise ValueError(f"Unknown type: {field_type}")
                field_type = NUMPY_TYPE_MAP[field_type]

            dtype_fields.append((field_name, field_type))

        # Create numpy dtype
        dtype = np.dtype(dtype_fields)

        # Create the new class
        class_dict = {
            '__topic_name__': topic,
            '__dtype__': dtype,
            '__field_names__': field_names,
            '__slots__': ['_data'],
        }

        new_class = type(name, (NumpyMessage,), class_dict)
        return new_class
