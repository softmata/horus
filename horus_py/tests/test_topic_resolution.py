"""Test topic-related Python API.

Robot name is read automatically from [robot].name in horus.toml.
Users construct topic names using the convention:
  "{robot_name}.{sensor_name}.{data_type}"

Requires: maturin develop (builds the horus Python module)
"""

import pytest


def test_robot_name_from_config():
    """Robot name is read from horus.toml [robot].name automatically."""
    import horus

    try:
        hw = horus.drivers.load()
    except Exception:
        pytest.skip("No horus.toml in test directory")

    name = hw.robot_name()
    assert isinstance(name, str)
    assert len(name) > 0
