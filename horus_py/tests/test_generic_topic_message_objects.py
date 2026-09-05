"""An untyped topic must accept a message object, not just a dict.

`Topic("name")` carries `GenericMessage` (MessagePack), and
`generic_message_from_py` ran `pythonize::depythonize` on the Python object —
which cannot read an opaque `#[pyclass]`, i.e. every HORUS message class and
every msggen-generated class. So `Topic(RobotStatus).send(instance)`, the line
`build_messages()` PRINTS on success, raised `TypeError: Failed to convert
Python object`, while `node.send("robot.status", instance)` worked, because
`Node.send` catches that refusal and retries the object as a dict.

Two spellings of one operation disagreeing is the defect. The conversion now
lives where all three send paths share it.
"""
import uuid

import pytest

import horus
from horus import CmdVel, Pose2D, Topic


def _unique(prefix):
    return f"{prefix}_{uuid.uuid4().hex[:8]}"


def test_a_message_class_survives_an_untyped_topic():
    t = Topic(_unique("generic_msgclass"))
    t.send(CmdVel(1.0, 0.5))

    got = t.recv()
    assert got is not None, "the message must arrive"
    assert got["linear"] == pytest.approx(1.0)
    assert got["angular"] == pytest.approx(0.5)


def test_try_send_and_send_blocking_agree_with_send():
    """All three publish paths share the conversion, so all three accept it."""
    t = Topic(_unique("generic_all_paths"))
    assert t.try_send(Pose2D(1.0, 2.0, 0.5)) is True
    t.send_blocking(Pose2D(3.0, 4.0, 1.5), 0.2)

    seen = []
    while (m := t.recv()) is not None:
        seen.append(m)
    assert len(seen) == 2, f"both publishes must arrive: {seen}"
    assert seen[0]["x"] == pytest.approx(1.0)
    assert seen[1]["x"] == pytest.approx(3.0)


def test_a_plain_dict_still_works_unchanged():
    """The fallback must not disturb the ordinary MessagePack path."""
    t = Topic(_unique("generic_dict"))
    t.send({"a": 1, "b": [1, 2, 3]})
    assert t.recv() == {"a": 1, "b": [1, 2, 3]}


def test_something_with_nothing_to_extract_still_raises():
    """The fallback is a retry, not a licence to encode anything."""
    t = Topic(_unique("generic_unencodable"))
    with pytest.raises(TypeError):
        t.send(lambda x: x)


def test_the_package_quick_start_runs():
    """`help(horus)`'s first example is the only API reference some users read.

    It published an `Image` over a bare-string topic and called it zero-copy;
    the call raised and nothing after it ran.
    """
    img = horus.Image(48, 64, "rgb8")
    topic = horus.Topic(horus.Image, endpoint=_unique("camera.rgb"))
    topic.send(img)

    received = topic.recv()
    assert received is not None
    arr = received.to_numpy()
    assert arr.shape == (48, 64, 3)
