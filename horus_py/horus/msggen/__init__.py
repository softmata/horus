"""
HORUS Message Generator

Create custom typed messages for zero-copy IPC.

TWO APPROACHES:

1. RUNTIME MESSAGES (No build step - recommended for prototyping)
   Uses struct/numpy for fixed-layout messages. ~5-10μs latency.

   from horus.msggen import define_message

   RobotStatus = define_message('RobotStatus', 'robot/status', [
       ('battery_level', 'f32'),
       ('error_code', 'i32'),
       ('is_active', 'bool'),
       ('timestamp', 'u64'),
   ])

   status = RobotStatus(battery_level=85.0, error_code=0, is_active=True)
   raw_bytes = status.to_bytes()  # For IPC

2. COMPILED MESSAGES (Requires maturin - for production)
   Generates Rust code and rebuilds. ~3μs latency.

   from horus.msggen import register_message, build_messages

   register_message('RobotStatus', 'robot/status', [
       ('battery_level', 'f32'),
       ('error_code', 'i32'),
       ('is_active', 'bool'),
       ('timestamp', 'u64'),
   ])

   build_messages()  # Runs maturin develop

   from horus import RobotStatus, Topic
   topic = Topic(RobotStatus)  # Full zero-copy typed topic
"""

from .generator import (
    generate_message,
    generate_messages_from_yaml,
    register_message,
)
from .builder import build_messages, check_needs_rebuild
from .runtime import define_message, RuntimeMessage

# Optional numpy support
try:
    from .runtime import define_numpy_message, NumpyMessage
    HAS_NUMPY = True
except ImportError:
    HAS_NUMPY = False
    define_numpy_message = None
    NumpyMessage = None

__all__ = [
    # Runtime messages (no build step)
    'define_message',
    'RuntimeMessage',
    'define_numpy_message',
    'NumpyMessage',
    'HAS_NUMPY',

    # Compiled messages (requires maturin)
    'generate_message',
    'generate_messages_from_yaml',
    'register_message',
    'build_messages',
    'check_needs_rebuild',
]
