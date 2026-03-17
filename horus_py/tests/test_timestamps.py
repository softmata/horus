"""
Test message timestamps via Rust Topic backend.

Messages with typed fields (CmdVel, Pose2D, etc.) carry timestamp_ns.
Python nodes use node.recv() to get messages — timestamps are on the message object.
"""

import horus
import time


def test_recv_returns_message():
    """Test that node.recv() returns messages published by another node."""

    received = []

    def publisher_tick(node):
        node.send("data", {"x": 1.0, "y": 2.0, "theta": 0.5})

    def subscriber_tick(node):
        msg = node.recv("data")
        if msg is not None:
            received.append(msg)

    pub = horus.Node(name="pub", tick=publisher_tick, rate=30, order=0, pubs=["data"])
    sub = horus.Node(name="sub", tick=subscriber_tick, rate=30, order=1, subs=["data"])

    scheduler = horus.Scheduler()
    scheduler.add(pub)
    scheduler.add(sub)
    scheduler.run(duration=0.5)

    assert len(received) > 0, "Should have received messages"


def test_recv_all_returns_batch():
    """Test that node.recv_all() returns all available messages."""

    batches = []

    def publisher_tick(node):
        node.send("data", {"value": 42})

    def batch_tick(node):
        msgs = node.recv_all("data")
        if msgs:
            batches.append(len(msgs))

    pub = horus.Node(name="pub", tick=publisher_tick, rate=100, order=0, pubs=["data"])
    batch = horus.Node(name="batch", tick=batch_tick, rate=10, order=1, subs=["data"])

    scheduler = horus.Scheduler()
    scheduler.add(pub)
    scheduler.add(batch)
    scheduler.run(duration=0.5)

    assert len(batches) > 0, "Should have received batches"


def test_has_msg_peek():
    """Test that has_msg() peeks without consuming."""

    checks = []

    def publisher_tick(node):
        node.send("data", {"v": 1})

    def checker_tick(node):
        if node.has_msg("data"):
            # has_msg should peek — recv should still get the message
            msg = node.recv("data")
            if msg is not None:
                checks.append(True)

    pub = horus.Node(name="pub", tick=publisher_tick, rate=30, order=0, pubs=["data"])
    chk = horus.Node(name="chk", tick=checker_tick, rate=30, order=1, subs=["data"])

    scheduler = horus.Scheduler()
    scheduler.add(pub)
    scheduler.add(chk)
    scheduler.run(duration=0.3)

    assert len(checks) > 0, "has_msg + recv should work together"
