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


def test_send_blocking_on_a_broadcast_topic_raises_backpressure_not_timeout():
    """The mapping must hold on the SEND PATH, not just in the constructor.

    The Rust-side test asserts `no_backpressure_err` builds the right exception.
    That is the easy half: the defect was that `send_blocking_pod` ended in
    `.is_ok()`, discarding WHICH `SendBlockingError` came back, so the send path
    reported every refusal as a timeout no matter what the core returned. This
    drives a real broadcast-backed topic instead.

    A second subscriber is what resolves a POD topic to `PodShm`, and it has to
    be on another thread: an entry is keyed on (pid, thread), so two handles
    here would share one registration and the topic would stay `SpscShm`.
    """
    import threading

    name = _unique("estop_broadcast")
    pub = Topic(CmdVel, endpoint=name)

    stop = threading.Event()
    ready = threading.Barrier(3)

    def hold():
        sub = Topic(CmdVel, endpoint=name)
        sub.try_recv()          # claims this thread's participant slot
        ready.wait()
        while not stop.is_set():
            sub.try_recv()

    threads = [threading.Thread(target=hold, daemon=True) for _ in range(2)]
    for t in threads:
        t.start()
    ready.wait()

    try:
        # Resolve the backend, then confirm the topology we need.
        pub.send(CmdVel(0.0, 0.0))
        if pub.provides_backpressure:
            pytest.skip(f"topic did not reach a broadcast backend: {pub.backend_type}")

        with pytest.raises(horus.HorusBackpressureError) as excinfo:
            pub.send_blocking(CmdVel(1.0, 0.0), 0.05)

        msg = str(excinfo.value)
        assert name in msg, f"the error must name the topic: {msg}"
        assert "backpressure" in msg.lower() or "MpscShm" in msg, (
            f"the core's remediation text must survive the crossing: {msg}"
        )
    finally:
        stop.set()
        for t in threads:
            t.join(timeout=5)


def test_provides_backpressure_reports_the_backend():
    """The property Python had no way to ask before."""
    name = _unique("bp_single")
    t = Topic(CmdVel, endpoint=name)
    assert t.provides_backpressure is False, "unresolved backend must fail closed"
    t.send(CmdVel(0.0, 0.0))
    assert t.provides_backpressure is True, (
        "a single-subscriber POD topic resolves to SpscShm, which refuses a full ring"
    )
