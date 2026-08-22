"""Test topic-related Python API.

Robot name is read automatically from [robot].name in horus.toml.
Users construct topic names using the convention:
  "{robot_name}.{sensor_name}.{data_type}"

Requires: maturin develop (builds the horus Python module)
"""

import pytest


def test_drivers_load_returns_name_driver_pairs():
    """`horus.drivers.load()` returns a list of (name, driver) pairs.

    This test used to call `hw.robot_name()` on the result. No `robot_name`
    exists anywhere in horus_py, and `load()` has always returned a
    `Vec<(String, PyAny)>` — so the assertion raised AttributeError on every
    machine where `load()` succeeded, and only "passed" by skipping where
    there was no horus.toml. It asserted an API that was never implemented
    and never documented; `robot_name` appears in the docs only as a *runtime
    parameter key* in the C++ params tutorial.
    """
    import horus

    try:
        drivers = horus.drivers.load()
    except Exception:
        pytest.skip("No horus.toml in test directory")

    assert isinstance(drivers, list)
    for entry in drivers:
        assert isinstance(entry, tuple) and len(entry) == 2, (
            "each entry is a (name, driver) pair"
        )
        name, driver = entry
        assert isinstance(name, str) and name, "driver name must be a non-empty string"
        assert driver is not None


def test_name_reports_the_endpoint_the_topic_was_created_under():
    """`topic.name` must be the key the shared-memory topic actually uses.

    Regression: the constructor created the topic under `endpoint or
    type_name`, but stored the *type-derived* name. `Topic(Imu,
    endpoint="imu.data")` therefore reported `imu` — contradicting the
    documented "topic.name: the topic name string", and misattributing IPC
    log events to a topic that does not exist. It also made the docs' claim
    that `endpoint=` names the topic unverifiable from Python.
    """
    import horus
    from horus import Imu, Topic

    named = Topic(Imu, endpoint="tutorial.imu.data")
    assert named.name == "tutorial.imu.data"
    assert named.endpoint == "tutorial.imu.data"

    # With no endpoint the name still falls back to the message type.
    derived = Topic(Imu)
    assert derived.name == "imu"
    assert derived.endpoint is None


def test_endpoint_determines_topic_identity():
    """Two topics share data iff they share an endpoint, not a message type."""
    from horus import Imu, Topic

    def reading(x):
        return Imu(accel_x=x, accel_y=0.0, accel_z=9.81,
                   gyro_x=0.0, gyro_y=0.0, gyro_z=0.0)

    a = Topic(Imu, endpoint="parity.imu.alpha")
    same = Topic(Imu, endpoint="parity.imu.alpha")
    other = Topic(Imu, endpoint="parity.imu.beta")

    a.send(reading(1.0))
    assert other.recv() is None, "a different endpoint must not receive"
    got = same.recv()
    assert got is not None, "the same endpoint must receive"
    assert got.accel_x == pytest.approx(1.0)
