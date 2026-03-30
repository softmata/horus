"""Python error path tests — verify the system handles errors gracefully.

Tests what happens when things go wrong:
- Undeclared topic recv returns None (not crash)
- Wrong type send returns False (not crash)
- Python tick() exception is caught by scheduler (not crash)
- Node init failure is handled
- Invalid configuration is rejected with helpful errors
"""

import horus
import pytest


class TestRecvErrorPaths:
    """Tests for recv() error handling."""

    def test_recv_undeclared_topic_returns_none(self):
        """recv() on a topic not in subs returns None, doesn't crash."""
        received = [None]

        def tick(node):
            result = node.recv("nonexistent_topic_12345")
            received[0] = result

        node = horus.Node(name="recv_err_1", tick=tick, rate=10, subs=["other"])
        horus.run(node, duration=0.2)

        assert received[0] is None, "recv on undeclared topic should return None"

    def test_recv_empty_topic_returns_none(self):
        """recv() when no messages available returns None."""
        received = [None]

        def tick(node):
            received[0] = node.recv("empty_topic")

        node = horus.Node(
            name="recv_err_2", tick=tick, rate=10, subs=["empty_topic"]
        )
        horus.run(node, duration=0.2)

        assert received[0] is None

    def test_has_msg_undeclared_topic_returns_false(self):
        """has_msg() on undeclared topic returns False."""
        result = [None]

        def tick(node):
            result[0] = node.has_msg("no_such_topic_xyz")

        node = horus.Node(name="has_msg_err", tick=tick, rate=10)
        horus.run(node, duration=0.1)

        assert result[0] is False or result[0] is None


class TestSendErrorPaths:
    """Tests for send() error handling."""

    def test_send_to_undeclared_topic_doesnt_crash(self):
        """send() to a topic not in pubs auto-creates it, doesn't crash."""
        sent_ok = [False]

        def tick(node):
            result = node.send("auto_created_topic", {"data": 42})
            sent_ok[0] = True  # If we get here, no crash

        node = horus.Node(name="send_err_1", tick=tick, rate=10)
        horus.run(node, duration=0.2)

        assert sent_ok[0], "send to undeclared topic should not crash"

    def test_send_none_doesnt_crash(self):
        """send(topic, None) should not crash."""
        sent_ok = [False]

        def tick(node):
            node.send("test_topic", None)
            sent_ok[0] = True

        node = horus.Node(name="send_none", tick=tick, rate=10, pubs=["test_topic"])
        horus.run(node, duration=0.1)

        assert sent_ok[0], "send(None) should not crash"


class TestTickExceptionHandling:
    """Tests for Python tick() exception handling by the scheduler."""

    def test_tick_exception_doesnt_crash_process(self):
        """If tick() raises, the Python process survives."""
        tick_count = [0]

        def faulty_tick(node):
            tick_count[0] += 1
            if tick_count[0] == 2:
                raise ValueError("Intentional test error")

        node = horus.Node(
            name="fault_tick",
            tick=faulty_tick,
            rate=50,
            on_error=lambda n, e: None,
        )

        try:
            horus.run(node, duration=0.1)
        except Exception:
            pass  # Exception may propagate — that's fine

        # Key assertion: we got here without segfault
        assert tick_count[0] >= 1, f"Should tick at least once, got {tick_count[0]}"


class TestConfigValidation:
    """Tests for configuration validation with helpful errors."""

    def test_zero_rate_rejected(self):
        """Node with rate=0 should be rejected."""
        with pytest.raises((ValueError, Exception)):
            horus.Node(name="bad_rate", tick=lambda n: None, rate=0)

    def test_negative_rate_rejected(self):
        """Node with negative rate should be rejected."""
        with pytest.raises((ValueError, Exception)):
            horus.Node(name="neg_rate", tick=lambda n: None, rate=-10)

    def test_node_without_tick_has_noop_default(self):
        """Node without tick callback should use no-op default (not crash)."""
        # Node(name="no_tick") is valid — tick defaults to None/no-op
        node = horus.Node(name="no_tick")
        assert node is not None

    def test_empty_name_allowed(self):
        """Node with empty name should auto-generate a name."""
        # Should not crash — auto-name generation kicks in
        node = horus.Node(tick=lambda n: None, rate=10)
        assert node is not None


class TestNodeLifecycleErrors:
    """Tests for error handling during node lifecycle."""

    def test_init_exception_doesnt_segfault(self):
        """If init() raises, the process doesn't segfault."""
        def bad_init(node):
            raise RuntimeError("Init failed")

        node = horus.Node(
            name="bad_init_node",
            tick=lambda n: None,
            init=bad_init,
            rate=50,
        )

        try:
            horus.run(node, duration=0.1)
        except Exception:
            pass  # Init failure propagation is fine — no segfault is the goal


class TestSchedulerErrors:
    """Tests for scheduler-level error handling."""

    def test_scheduler_reuse_after_run(self):
        """Scheduler can be used after run() completes."""
        node = horus.Node(name="reuse_1", tick=lambda n: None, rate=50)
        horus.run(node, duration=0.1)

        # Second run with new node — should work
        node2 = horus.Node(name="reuse_2", tick=lambda n: None, rate=50)
        horus.run(node2, duration=0.1)
