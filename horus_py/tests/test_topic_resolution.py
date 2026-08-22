"""Test topic-related Python API.

Robot name is read automatically from [robot].name in horus.toml.
Users construct topic names using the convention:
  "{robot_name}.{sensor_name}.{data_type}"

Requires: maturin develop (builds the horus Python module)
"""

import pytest


def test_robot_name_from_config(tmp_path):
    """Robot name is read from horus.toml [robot].name.

    This called `horus.drivers.load().robot_name()`, which does not exist —
    `load()` returns a list of (name, driver) pairs — so it failed with
    `'list' object has no attribute 'robot_name'`. Nothing exposed the field at
    all, though the manifest documents it as "used in topic naming" and this
    file's own docstring describes the convention it feeds.
    """
    import horus

    manifest = tmp_path / "horus.toml"
    manifest.write_text(
        '[package]\nname = "probe"\nversion = "0.1.0"\n\n[robot]\nname = "turtlebot"\n'
    )

    name = horus.drivers.robot_name_from(str(manifest))
    assert name == "turtlebot"
    assert isinstance(name, str)


def test_robot_name_is_none_without_a_robot_section(tmp_path):
    """A manifest with no [robot] has no name, rather than an error."""
    import horus

    manifest = tmp_path / "horus.toml"
    manifest.write_text('[package]\nname = "probe"\nversion = "0.1.0"\n')

    assert horus.drivers.robot_name_from(str(manifest)) is None
