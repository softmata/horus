"""A bare-string `subs=` must resolve the ring's real type, not guess MessagePack.

`Node._setup_topics` documents a three-step type resolution whose middle step is
"auto-detect from SHM header". It was implemented as ``probe = Topic(name, 64)``
followed by ``getattr(probe, "type_name", None)``, and both halves were dead:
``Topic`` exposes no ``type_name``, and the probe builds a GenericMessage topic,
which the runtime refuses to attach to a typed ring. So the step never ran for
any topic that has ever existed, the ~70-entry ``_TYPE_NAME_MAP`` built for it
had no reachable reader, and a bare ``subs=["scan"]`` against a typed ring was
answered with a RuntimeError about byte counts that named neither `subs=`, nor
the type the ring holds, nor the fix.
"""
import uuid

import pytest

import horus
from horus import CmdVel, LaserScan, Topic


def _unique(name):
    return f"{name}_{uuid.uuid4().hex[:8]}"


def test_peek_reads_the_ring_type_without_attaching():
    name = _unique("peek_typed")
    typed = Topic(CmdVel, endpoint=name)
    typed.send(CmdVel(1.0, 0.0))

    assert horus._peek_topic_type(name) == "CmdVel"
    assert horus._peek_topic_type(_unique("never_created")) is None


def test_a_bare_subs_string_resolves_to_the_rings_type():
    name = _unique("auto_typed")
    # A publisher creates the ring as a typed LaserScan topic.
    pub = Topic(LaserScan, endpoint=name)
    pub.send(LaserScan())

    # The subscriber says only the name — no type. Step 2 must now resolve it.
    node = horus.Node(name="detector", subs=[name])
    node._setup_topics()

    resolved = node._topics[name]
    assert not resolved.is_generic(), (
        "a bare subs= string on an existing typed ring must resolve to that "
        "type, not open an untyped MessagePack topic that cannot attach"
    )


def test_an_unmappable_ring_type_says_what_to_do():
    """The residual case must name the topic, the type and the fix.

    Falling through to a GenericMessage topic gets the runtime's size-mismatch
    RuntimeError, which talks about byte counts and names none of those three.
    """
    name = _unique("unmappable")
    pub = Topic(CmdVel, endpoint=name)
    pub.send(CmdVel(0.0, 0.0))

    node = horus.Node(name="detector", subs=[name])
    # Pretend this build has no Python class for the ring's type.
    saved = horus._TYPE_NAME_MAP.pop("cmdvel")
    try:
        with pytest.raises(RuntimeError) as excinfo:
            node._setup_topics()
    finally:
        horus._TYPE_NAME_MAP["cmdvel"] = saved

    msg = str(excinfo.value)
    assert name in msg, f"must name the topic: {msg}"
    assert "CmdVel" in msg, f"must name the type the ring holds: {msg}"
    assert "subs=" in msg, f"must name the fix: {msg}"
