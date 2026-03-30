"""
HORUS Message Generator — Generate custom Rust message types from Python.

Generates Rust PyO3 code and compiles via maturin. The generated messages
get full zero-copy IPC support (~3μs latency) just like built-in types.

Usage:
    from horus.msggen import register_message, build_messages

    register_message('RobotStatus', 'robot/status', [
        ('battery_level', 'f32'),
        ('error_code', 'i32'),
        ('is_active', 'bool'),
        ('timestamp', 'u64'),
    ])

    build_messages()  # Generates Rust code + runs maturin develop

    from horus import RobotStatus, Topic
    topic = Topic(RobotStatus)  # Full zero-copy typed topic
"""

from .generator import (
    generate_message,
    generate_messages_from_yaml,
    register_message,
)
from .builder import build_messages, check_needs_rebuild

__all__ = [
    'generate_message',
    'generate_messages_from_yaml',
    'register_message',
    'build_messages',
    'check_needs_rebuild',
]
