"""`horus run --sim` must not construct a real driver on the Python path.

The Rust loader substitutes a stub for every `[hardware]` entry marked
`sim = true`; the Python loader used to instantiate the registered driver
class regardless, because the shared parse reported no simulation state.
Constructing a driver is where the serial port is opened and the motor is
enabled, so `--sim` did nothing at all here.
"""

import os

import pytest

from horus import drivers as hardware


CONSTRUCTED = []


class RecordingDriver:
    """Stands in for a driver whose __init__ touches hardware."""

    def __init__(self, params):
        CONSTRUCTED.append(params)
        self.params = params


@pytest.fixture(autouse=True)
def _clean_env():
    CONSTRUCTED.clear()
    saved = {k: os.environ.get(k) for k in ("HORUS_SIM_MODE", "HORUS_SIM_TARGETS")}
    yield
    for key, value in saved.items():
        if value is None:
            os.environ.pop(key, None)
        else:
            os.environ[key] = value


def _manifest(tmp_path, body):
    path = tmp_path / "horus.toml"
    path.write_text(body)
    return str(path)


def test_sim_mode_returns_a_stub_instead_of_the_driver(tmp_path):
    hardware.register_driver("RecordingDriver", RecordingDriver)
    path = _manifest(
        tmp_path,
        '[hardware.arm]\nuse = "RecordingDriver"\nsim = true\nport = "/dev/ttyACM0"\n',
    )

    os.environ["HORUS_SIM_MODE"] = "1"
    loaded = hardware.load_from(path)

    assert len(loaded) == 1
    name, node = loaded[0]
    assert name == "arm"
    assert CONSTRUCTED == [], "--sim must not construct the real driver"
    assert type(node).__name__ == "SimStub"
    assert node.simulated is True
    assert node.name == "arm_sim_stub"
    # The stub still carries the config, so simulation code can read it.
    assert node.params.get("port") == "/dev/ttyACM0"
    node.init()
    node.tick()


def test_without_sim_mode_the_real_driver_is_constructed(tmp_path):
    hardware.register_driver("RecordingDriver", RecordingDriver)
    path = _manifest(
        tmp_path, '[hardware.arm]\nuse = "RecordingDriver"\nsim = true\n'
    )

    os.environ.pop("HORUS_SIM_MODE", None)
    loaded = hardware.load_from(path)

    assert len(CONSTRUCTED) == 1, "`sim = true` alone must not simulate"
    assert isinstance(loaded[0][1], RecordingDriver)


def test_simulated_is_an_alias_of_sim_here_too(tmp_path):
    hardware.register_driver("RecordingDriver", RecordingDriver)
    path = _manifest(
        tmp_path,
        '[hardware.arm]\nuse = "RecordingDriver"\nsimulated = true\n',
    )

    os.environ["HORUS_SIM_MODE"] = "1"
    loaded = hardware.load_from(path)

    assert CONSTRUCTED == []
    assert type(loaded[0][1]).__name__ == "SimStub"


def test_sim_targets_narrows_which_entries_are_stubbed(tmp_path):
    hardware.register_driver("RecordingDriver", RecordingDriver)
    path = _manifest(
        tmp_path,
        '[hardware.lidar]\nuse = "RecordingDriver"\nsim = true\n\n'
        '[hardware.imu]\nuse = "RecordingDriver"\nsim = true\n',
    )

    os.environ["HORUS_SIM_MODE"] = "1"
    os.environ["HORUS_SIM_TARGETS"] = "lidar"
    by_name = dict(hardware.load_from(path))

    assert type(by_name["lidar"]).__name__ == "SimStub"
    assert isinstance(by_name["imu"], RecordingDriver)
