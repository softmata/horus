"""
Loss counters through the Python API.

A HORUS topic is lossy by design: a full ring overwrites the oldest unconsumed
slot rather than blocking the publisher. That is only safe if the loss is
*observable* — a supervisor cannot tell a quiet sensor from a dead one, so a
drop nobody can count is indistinguishable from silence.

These tests cover the Python side of that: the counters exist, are reachable,
and report loss when loss is forced. Before this they were Rust-only, which
meant a Python robot had no way to know it was dropping frames.
"""

import horus
from horus import CmdVel


class TestLossCounters:
    """missed_count / dropped_count on the Python Topic."""

    def test_counters_exist_and_start_at_zero(self):
        """A fresh topic has lost nothing, and says so as an integer."""
        t = horus.Topic(CmdVel, capacity=64)

        missed = t.missed_count()
        dropped = t.dropped_count()

        assert isinstance(missed, int), f"missed_count returned {type(missed)}"
        assert isinstance(dropped, int), f"dropped_count returned {type(dropped)}"
        assert missed == 0, f"a fresh topic cannot have missed {missed}"
        assert dropped == 0, f"a fresh topic cannot have dropped {dropped}"

    def test_counters_account_for_an_overrun(self):
        """Flooding without draining must be fully accounted for.

        Every message the publisher accepts is delivered, missed, or dropped —
        none may simply vanish. Which counter moves depends on the backend
        (backpressured ones give up at the publisher, overwriting ones lap the
        consumer), which is why this asserts the sum rather than either alone.

        Two things this test has to do, and both were learned the hard way:

        - Poll the subscriber ONCE before sending. Without that attach the
          publisher counts nothing and the run reads as 1936 silently lost
          messages, which looks exactly like a counting bug and is not one.
        - Take baselines first. The topic name is derived from the message
          type, so every ``Topic(CmdVel)`` in a process is the same topic and
          the counters carry over from earlier tests. Reading them as absolutes
          made an earlier draft report 2000 drops against 2000 sends.
        """
        pub = horus.Topic(CmdVel, capacity=64)
        sub = horus.Topic(CmdVel, capacity=64)

        # Attach the subscriber before measuring anything.
        sub.try_recv()
        missed_before = sub.missed_count()
        dropped_before = pub.dropped_count()

        sent = 2000
        for i in range(sent):
            pub.send(CmdVel(linear=float(i), angular=0.0))

        got = 0
        empty_polls = 0
        # One empty poll does not mean empty; poll until several in a row are.
        while empty_polls < 5:
            if sub.try_recv() is not None:
                got += 1
                empty_polls = 0
            else:
                empty_polls += 1

        missed = sub.missed_count() - missed_before
        dropped = pub.dropped_count() - dropped_before

        assert got < sent, (
            f"this test is only meaningful if the ring actually overran, but "
            f"all {sent} messages were delivered"
        )
        assert got + missed + dropped == sent, (
            f"{sent} sent, but {got} delivered + {missed} missed + {dropped} "
            f"dropped = {got + missed + dropped}. Loss no counter records is "
            f"loss a supervisor cannot act on: it cannot tell a quiet sensor "
            f"from a dead one."
        )
