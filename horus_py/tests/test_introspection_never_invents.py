"""Scheduler introspection must not invent an answer while the scheduler runs.

`run()` takes the native scheduler out of the PyScheduler to drive it, so every
accessor sees `inner == None` for the whole run. Those accessors used to answer
that state with a fabricated value — zero nodes, tick 0, `False` for
`is_recording()`, the literal `"PythonScheduler"` — indistinguishable from a
real observation. Anything polling from inside a tick read a scheduler that
looked empty and idle while it was neither.

`get_node_stats` in the same class always refused with "Stats unavailable while
scheduler is running", and Invariant 3 in `scheduler.rs` says a re-entrant call
gets a `RuntimeError`. These assert the rest now agree.
"""

import horus


def _drive(probe):
    """Run a one-node scheduler, calling `probe(sched)` from inside a tick."""
    seen = []

    def tick(node):
        if not seen:
            seen.append(probe(sched._scheduler))

    node = horus.Node(name="probe_node", tick=tick, rate=200, order=0)
    sched = horus.Scheduler(tick_rate=200.0)
    sched.add(node)
    sched.run(duration=0.05)
    assert seen, "the probe never ran — the tick callback did not fire"
    return seen[0]


def _refused(call):
    def probe(native):
        try:
            return ("value", call(native))
        except RuntimeError as exc:
            return ("raised", str(exc))

    kind, payload = _drive(probe)
    assert kind == "raised", (
        f"introspection returned {payload!r} from inside a running tick "
        "instead of refusing"
    )
    assert "while scheduler is running" in payload, payload
    return payload


def test_node_count_is_not_reported_as_zero_while_running():
    _refused(lambda s: s.get_node_count())


def test_node_names_are_not_reported_as_empty_while_running():
    _refused(lambda s: s.get_node_names())


def test_has_node_does_not_deny_a_node_that_exists():
    _refused(lambda s: s.has_node("probe_node"))


def test_current_tick_is_not_reported_as_zero_while_running():
    _refused(lambda s: s.current_tick())


def test_scheduler_name_is_not_a_hardcoded_string_while_running():
    _refused(lambda s: s.scheduler_name())


def test_recording_state_is_not_reported_as_false_while_running():
    # The dangerous one: "not recording" on a session that is recording.
    _refused(lambda s: s.is_recording())


def test_replay_state_is_not_reported_as_false_while_running():
    _refused(lambda s: s.is_replaying())


def test_rt_capability_is_not_denied_while_running():
    _refused(lambda s: s.has_full_rt())


def test_degradations_are_not_reported_as_none_while_running():
    _refused(lambda s: s.degradations())


def test_safety_stats_do_not_read_as_absent_while_running():
    _refused(lambda s: s.safety_stats())


def test_capabilities_do_not_read_as_absent_while_running():
    _refused(lambda s: s.capabilities())


def test_status_still_answers_honestly_while_running():
    """`status()` always said "running" here — it is the one that was right."""
    kind, payload = _drive(lambda s: ("value", s.status())[1] and ("value", s.status()))
    assert kind == "value"
    assert payload == "running"


def test_everything_answers_normally_once_the_run_is_over():
    node = horus.Node(name="after_run", tick=lambda n: None, rate=200, order=0)
    sched = horus.Scheduler(tick_rate=200.0)
    sched.add(node)
    sched.run(duration=0.02)

    native = sched._scheduler
    assert native.get_node_count() == 1
    assert native.get_node_names() == ["after_run"]
    assert native.has_node("after_run") is True
    assert native.is_recording() is False
    assert native.is_replaying() is False
    assert isinstance(native.scheduler_name(), str)
    assert native.current_tick() >= 0
