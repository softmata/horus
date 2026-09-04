"""ActionChunk from Python: the policy-to-servo seam.

A chunk carries a block of future actions so a control loop at 500-1000 Hz can
keep producing commands between publishes from a policy at 10-50 Hz. These
tests cover the round trip and, more importantly, the property the whole type
exists to guarantee: **a stale chunk never yields an action.**

When a policy stops publishing, the query runs off the end of its last chunk.
Producing a number anyway means extrapolating a trajectory nobody computed, on
a real robot. `horus_core` refuses to, and these pin that the refusal survives
the trip into Python.

This is also the first Python test anywhere in the repo that exercises a
pool-backed topic end to end.
"""

import numpy as np
import pytest

import horus

HORIZON = 8
ACTION_DIM = 4
DT_NS = 20_000_000  # 50 Hz policy
T0_NS = 1_000_000_000
SPAN_NS = DT_NS * HORIZON


def ramp():
    """Actions where row r, column c holds r * 100 + c.

    Any misread — wrong row, recycled pool slot, transposed shape — shows up as
    a number that could not have come from here.
    """
    a = np.zeros((HORIZON, ACTION_DIM), dtype=np.float32)
    for r in range(HORIZON):
        for c in range(ACTION_DIM):
            a[r, c] = r * 100 + c
    return a


def make(t0=T0_NS, seq=0, frame_id=None):
    return horus.ActionChunk.from_numpy(
        ramp(), t0_ns=t0, dt_ns=DT_NS, frame_id=frame_id, seq=seq
    )


# ── Construction ────────────────────────────────────────────────────────────


def test_from_numpy_preserves_shape_and_metadata():
    c = make(seq=11, frame_id="left_arm")
    assert c.shape == (HORIZON, ACTION_DIM)
    assert c.horizon == HORIZON
    assert c.action_dim == ACTION_DIM
    assert c.t0_ns == T0_NS
    assert c.dt_ns == DT_NS
    assert c.seq == 11
    assert c.frame_id == "left_arm"
    assert c.dtype == "float32"
    # end_ns is when the LAST action applies, not the end of the span: eight
    # actions 20ms apart start at 0 and end at 140ms, not 160ms.
    assert c.end_ns == T0_NS + DT_NS * (HORIZON - 1)


def test_from_numpy_round_trips_the_actions():
    c = make()
    assert np.array_equal(c.to_numpy(), ramp())


def test_a_one_dimensional_array_is_read_as_action_dim_one():
    c = horus.ActionChunk.from_numpy(
        np.arange(5, dtype=np.float32), t0_ns=T0_NS, dt_ns=DT_NS
    )
    assert c.shape == (5, 1)


def test_float64_actions_are_accepted_and_converted():
    c = horus.ActionChunk.from_numpy(
        np.ones((3, 2), dtype=np.float64), t0_ns=T0_NS, dt_ns=DT_NS
    )
    assert c.dtype == "float32"
    assert np.array_equal(c.to_numpy(), np.ones((3, 2), dtype=np.float32))


def test_an_integer_array_is_refused_by_name():
    # Silently reinterpreting int bytes as float32 would produce actions from
    # them, which is worse than an error.
    with pytest.raises(TypeError, match="float32 or float64"):
        horus.ActionChunk.from_numpy(
            np.ones((3, 2), dtype=np.int32), t0_ns=T0_NS, dt_ns=DT_NS
        )


def test_a_zero_sized_chunk_is_refused():
    # A zeroed chunk reports Malformed for every query; refusing beats handing
    # back an object that silently answers nothing.
    with pytest.raises(ValueError, match="must both be > 0"):
        horus.ActionChunk(0, 4, T0_NS, DT_NS)
    with pytest.raises(ValueError, match="must both be > 0"):
        horus.ActionChunk(8, 0, T0_NS, DT_NS)


# ── locate ──────────────────────────────────────────────────────────────────


def test_locate_distinguishes_before_inside_and_stale():
    c = make()
    assert isinstance(c.locate(T0_NS - 1_000_000), horus.ActionAt.Before)
    assert isinstance(c.locate(T0_NS + DT_NS), horus.ActionAt.Inside)
    assert isinstance(c.locate(T0_NS + SPAN_NS + 1_000_000), horus.ActionAt.Stale)


def test_stale_carries_how_far_past_the_chunk_the_query_is():
    # The number is the whole watchdog: a few hundred microseconds is jitter,
    # tens of milliseconds is a policy that has stopped. A bare boolean would
    # throw that away.
    c = make()
    at = c.locate(T0_NS + SPAN_NS + 5_000_000)
    assert isinstance(at, horus.ActionAt.Stale)
    assert at.by_ns > 0


def test_before_carries_the_time_until_the_chunk_starts():
    c = make()
    at = c.locate(T0_NS - 3_000_000)
    assert isinstance(at, horus.ActionAt.Before)
    assert at.by_ns > 0


def test_inside_carries_the_interpolation_operands():
    c = make()
    at = c.locate(T0_NS + DT_NS + DT_NS // 2)
    assert isinstance(at, horus.ActionAt.Inside)
    assert at.lo == 1
    assert at.hi == 2
    assert 0.0 <= at.alpha < 1.0


# ── sample: the safety property ─────────────────────────────────────────────


def test_sample_returns_the_action_inside_the_chunk():
    c = make()
    a = c.sample(T0_NS + DT_NS)
    assert a is not None
    # Exactly on a knot, the stored row is reproduced rather than approached.
    assert np.array_equal(a, ramp()[1])


def test_sample_returns_none_when_the_chunk_is_stale():
    # THE property. A caller who never inspects the status still cannot drive
    # a servo from a policy that has stopped publishing, because None cannot
    # be commanded to one.
    c = make()
    assert c.sample(T0_NS + SPAN_NS + 1_000_000_000) is None


def test_sample_returns_none_before_the_chunk_starts():
    c = make()
    assert c.sample(T0_NS - 1_000_000) is None


def test_sample_into_leaves_the_buffer_untouched_when_stale():
    # The buffer is pre-loaded with the caller's previous decision, which is
    # exactly what a real control loop holds there. Writing an extrapolated
    # action over it is how a robot keeps moving on a dead policy.
    c = make()
    previous = np.array([1.0, 2.0, 3.0, 4.0], dtype=np.float32)
    out = previous.copy()
    at = c.sample_into(T0_NS + SPAN_NS + 1_000_000_000, out)
    assert isinstance(at, horus.ActionAt.Stale)
    assert np.array_equal(out, previous), "a stale query must not write out"


def test_sample_into_writes_the_buffer_when_inside():
    c = make()
    out = np.zeros(ACTION_DIM, dtype=np.float32)
    at = c.sample_into(T0_NS + DT_NS, out)
    assert isinstance(at, horus.ActionAt.Inside)
    assert np.array_equal(out, ramp()[1])


def test_sample_into_refuses_a_wrong_dtype_buffer_before_writing():
    c = make()
    out = np.zeros(ACTION_DIM, dtype=np.float64)
    with pytest.raises(TypeError, match="float32"):
        c.sample_into(T0_NS + DT_NS, out)


def test_sample_into_refuses_a_too_small_buffer_before_writing():
    c = make()
    out = np.zeros(ACTION_DIM - 1, dtype=np.float32)
    with pytest.raises(ValueError, match="at least"):
        c.sample_into(T0_NS + DT_NS, out)


def test_sample_into_refuses_a_read_only_buffer():
    c = make()
    out = np.zeros(ACTION_DIM, dtype=np.float32)
    out.flags.writeable = False
    with pytest.raises(ValueError, match="writable"):
        c.sample_into(T0_NS + DT_NS, out)


# ── rows ────────────────────────────────────────────────────────────────────


def test_row_returns_the_stored_action():
    c = make()
    assert np.array_equal(c.row(3), ramp()[3])


def test_row_out_of_range_raises():
    c = make()
    with pytest.raises(IndexError):
        c.row(HORIZON)


# ── the topic round trip ────────────────────────────────────────────────────


def test_a_chunk_survives_a_topic_round_trip(unique_test_prefix):
    t = horus.Topic(horus.ActionChunk, endpoint=f"{unique_test_prefix}_ac_rt")
    sent = make(seq=42, frame_id="wrist")
    t.send(sent)

    got = t.recv()
    assert got is not None, "the chunk must arrive"
    assert isinstance(got, horus.ActionChunk)
    assert got.shape == (HORIZON, ACTION_DIM)
    assert got.seq == 42
    assert got.frame_id == "wrist"
    assert got.t0_ns == T0_NS
    assert got.dt_ns == DT_NS
    assert np.array_equal(got.to_numpy(), ramp())


def test_a_received_chunk_is_still_sampleable(unique_test_prefix):
    t = horus.Topic(horus.ActionChunk, endpoint=f"{unique_test_prefix}_ac_sample")
    t.send(make())
    got = t.recv()
    assert got is not None
    assert np.array_equal(got.sample(T0_NS + DT_NS), ramp()[1])
    assert got.sample(T0_NS + SPAN_NS + 1_000_000_000) is None


def test_a_held_chunk_is_not_recycled_by_later_publishes(unique_test_prefix):
    """The pool-slot defect, from Python.

    The subscriber holds the first chunk while the publisher sends six more.
    Without a reference transferred by the send, the first chunk's slot is free
    once the producer drops it, later allocations take it, and the actions read
    back as some other chunk's numbers — which on a servo loop is the wrong
    trajectory rather than a crash.
    """
    t = horus.Topic(horus.ActionChunk, endpoint=f"{unique_test_prefix}_ac_keep")
    t.send(make(seq=0))
    held = t.recv()
    assert held is not None

    for i in range(1, 7):
        later = horus.ActionChunk.from_numpy(
            np.full((HORIZON, ACTION_DIM), -1.0, dtype=np.float32),
            t0_ns=T0_NS + i * 1_000_000_000,
            dt_ns=DT_NS,
            seq=i,
        )
        t.send(later)
        t.recv()

    assert np.array_equal(held.to_numpy(), ramp()), (
        "the held chunk's actions were overwritten by later publishes — its "
        "pool slot was recycled while it was still referenced"
    )
    assert held.seq == 0


def test_a_chunk_topic_is_pool_backed_not_msgpack(unique_test_prefix):
    """Guards the worst silent failure available here.

    If "ActionChunk" is missing from the topic constructor's type-name match,
    it falls through to the generic arm and becomes a msgpack-serialising
    topic: no error, no panic, and no pool. recv() would then hand back a dict
    rather than an ActionChunk.
    """
    t = horus.Topic(horus.ActionChunk, endpoint=f"{unique_test_prefix}_ac_pool")
    t.send(make())
    got = t.recv()
    assert isinstance(got, horus.ActionChunk), (
        f"expected an ActionChunk, got {type(got).__name__} — the topic fell "
        "through to the generic msgpack path"
    )


def test_backpressure_sends_are_refused_by_name(unique_test_prefix):
    # Pool-backed topics have no backpressure-aware send. The refusal should
    # name the type rather than surface as a downcast failure.
    t = horus.Topic(horus.ActionChunk, endpoint=f"{unique_test_prefix}_ac_bp")
    c = make()
    with pytest.raises(NotImplementedError, match="ActionChunk"):
        t.try_send(c)
    with pytest.raises(NotImplementedError, match="ActionChunk"):
        t.send_blocking(c, 0.1)


def test_stats_work_on_a_chunk_topic(unique_test_prefix):
    # Exercises backend_type() and the stats metrics arm — both are
    # compiler-enforced matches, but only a call proves they were filled in
    # with the chunk topic rather than a placeholder.
    t = horus.Topic(horus.ActionChunk, endpoint=f"{unique_test_prefix}_ac_stats")
    t.send(make())
    s = t.stats()
    # A real backend name proves backend_type() got a chunk arm rather than
    # falling through to a default.
    assert isinstance(s["backend"], str) and s["backend"]
    # messages_sent stays 0 here. That is not an ActionChunk quirk: the
    # pool-backed handle send path does not increment the ring's counters, and
    # horus.Tensor reports 0 after a send too. Asserting the key exists pins
    # the metrics arm without pinning a number that is wrong for every
    # pool-backed type.
    assert "messages_sent" in s
    assert "messages_received" in s


# ── the seam, at rate ───────────────────────────────────────────────────────


def test_a_servo_loop_samples_inside_while_the_policy_keeps_up(unique_test_prefix):
    """50 Hz policy, 500 Hz loop: ten samples per publish, none stale."""
    t = horus.Topic(horus.ActionChunk, endpoint=f"{unique_test_prefix}_ac_seam")
    servo_period = 2_000_000  # 500 Hz
    publish_every = 10
    ticks = 200

    held = None
    inside = stale = 0
    for tick in range(ticks):
        now = T0_NS + tick * servo_period
        if tick % publish_every == 0:
            t.send(
                horus.ActionChunk.from_numpy(
                    ramp(), t0_ns=now, dt_ns=DT_NS, seq=tick // publish_every
                )
            )
        got = t.recv()
        if got is not None:
            held = got
        if held is not None:
            if held.sample(now) is not None:
                inside += 1
            elif isinstance(held.locate(now), horus.ActionAt.Stale):
                stale += 1

    assert stale == 0, f"nothing should go stale while the policy keeps up, got {stale}"
    assert inside >= ticks - publish_every, (
        f"the loop should have an action on essentially every tick, got {inside}"
    )


# ── sample_into writes through a raw pointer, so it checks what it is given ──


def test_sample_into_refuses_an_object_that_merely_fakes_array_interface():
    """`__array_interface__` is an ordinary Python attribute.

    Any object can define it and report an arbitrary integer as its data
    pointer. `sample_into` builds a `&mut [f32]` from that integer and writes
    through it, so trusting the protocol alone would turn a wrong argument into
    memory corruption in the host process. The argument must actually be a
    numpy.ndarray.
    """

    class FakeArray:
        # Shaped to pass every other check: float32, 1-D, long enough,
        # contiguous, writable. Only the type is wrong.
        #
        # The address is NULL on purpose. A wild pointer here would prove the
        # same thing by segfaulting, which would take the whole pytest process
        # down with it and tell the next person nothing about which assertion
        # failed. Null is caught by a later guard, so if the ndarray check ever
        # regresses this test fails cleanly with ValueError("null data
        # pointer") instead — a different exception and message from the
        # TypeError asserted below, so it cannot pass for the wrong reason.
        __array_interface__ = {
            "typestr": "<f4",
            "shape": [ACTION_DIM],
            "strides": None,
            "data": (0, False),
            "version": 3,
        }

    c = make()
    with pytest.raises(TypeError, match="numpy.ndarray"):
        c.sample_into(T0_NS + DT_NS, FakeArray())


def test_sample_into_refuses_a_non_contiguous_view():
    # A strided view's base pointer does not address the elements the caller
    # means, so writing through it would scatter values into the wrong slots.
    c = make()
    backing = np.zeros(ACTION_DIM * 2, dtype=np.float32)
    with pytest.raises(ValueError, match="contiguous"):
        c.sample_into(T0_NS + DT_NS, backing[::2])


def test_a_non_f32_chunk_is_refused_by_name_not_reported_as_stale():
    # The distinction matters: returning None for a dtype error would look
    # exactly like a policy that has stopped publishing.
    c = horus.ActionChunk(HORIZON, ACTION_DIM, T0_NS, DT_NS, dtype="float64")
    with pytest.raises(TypeError, match="float32"):
        c.sample(T0_NS + DT_NS)
    with pytest.raises(TypeError, match="float32"):
        c.to_numpy()
