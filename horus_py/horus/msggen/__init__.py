"""
HORUS Message Generator — Generate custom Rust message types from Python.

Generates Rust PyO3 code and compiles via maturin.

Generated messages travel over the MessagePack path, not the zero-copy POD
path the built-in types use: `Topic()` resolves a generated class through the
generic backend. They work — `send()` flattens the instance to its fields —
but do not assume built-in-type latency for them.

Usage:
    from horus.msggen import register_message, build_messages

    register_message('RobotStatus', 'robot/status', [
        ('battery_level', 'f32'),
        ('error_code', 'i32'),
        ('is_active', 'bool'),
        ('timestamp', 'u64'),
    ])

    build_messages()  # Generates Rust code + runs maturin develop

    from horus._horus import RobotStatus   # NOT `from horus import` — see below
    from horus import Topic
    topic = Topic(RobotStatus)
    topic.send(RobotStatus(battery_level=0.9, error_code=0,
                           is_active=True, timestamp=0))

A generated class is registered on the extension module, so it is imported
from ``horus._horus``; ``horus/__init__.py`` re-exports a fixed list that a
generated name is not on.
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
