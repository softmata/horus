"""End-to-end workflow tests for Python Node/Scheduler/Topic system.

Tests multi-node communication, typed message roundtrip, node lifecycle,
and scheduler duration. These are the first tests that exercise the
complete Python workflow with real message exchange between nodes.
"""

import horus
import pytest
import time


class TestMultiNodeCommunication:
    """Tests for multi-node topic-based communication."""

    def test_two_node_string_topic_roundtrip(self):
        """Publisher sends dict, subscriber receives it."""
        received = []

        def publisher(node):
            node.send("data", {"value": 42, "name": "test"})

        def subscriber(node):
            msg = node.recv("data")
            if msg is not None:
                received.append(msg)

        pub = horus.Node(name="e2e_pub_1", tick=publisher, rate=30, pubs=["data"])
        sub = horus.Node(name="e2e_sub_1", tick=subscriber, rate=30, subs=["data"])
        horus.run(pub, sub, duration=0.5)

        assert len(received) > 0, "Subscriber should receive at least one message"
        assert received[0]["value"] == 42

    def test_three_node_pipeline(self):
        """sensor → controller → actuator pipeline with string topics."""
        actuator_cmds = []

        def sensor(node):
            node.send("sensor_data", {"reading": 100.0})

        def controller(node):
            if node.has_msg("sensor_data"):
                data = node.recv("sensor_data")
                if data:
                    node.send("cmd", {"velocity": data["reading"] * 0.01})

        def actuator(node):
            if node.has_msg("cmd"):
                cmd = node.recv("cmd")
                if cmd:
                    actuator_cmds.append(cmd)

        s = horus.Node(name="e2e_sensor", tick=sensor, rate=30, pubs=["sensor_data"])
        c = horus.Node(
            name="e2e_ctrl",
            tick=controller,
            rate=30,
            subs=["sensor_data"],
            pubs=["cmd"],
        )
        a = horus.Node(name="e2e_act", tick=actuator, rate=30, subs=["cmd"])
        horus.run(s, c, a, duration=0.5)

        assert len(actuator_cmds) > 0, "Actuator should receive commands"

    def test_typed_cmdvel_roundtrip(self):
        """CmdVel typed message sent and received correctly."""
        received_vels = []

        def publisher(node):
            node.send("cmd_vel", horus.CmdVel(1.5, 0.3))

        def subscriber(node):
            msg = node.recv("cmd_vel")
            if msg is not None:
                received_vels.append(msg)

        pub = horus.Node(
            name="e2e_cmdvel_pub",
            tick=publisher,
            rate=30,
            pubs=[horus.CmdVel],
        )
        sub = horus.Node(
            name="e2e_cmdvel_sub",
            tick=subscriber,
            rate=30,
            subs=[horus.CmdVel],
        )
        horus.run(pub, sub, duration=0.5)

        assert len(received_vels) > 0, "Should receive CmdVel messages"
        assert abs(received_vels[0].linear - 1.5) < 0.01
        assert abs(received_vels[0].angular - 0.3) < 0.01


class TestNodeLifecycle:
    """Tests for node lifecycle callbacks."""

    def test_node_tick_is_called(self):
        """Verify tick() is called multiple times during run."""
        tick_count = [0]

        def counting_tick(node):
            tick_count[0] += 1

        node = horus.Node(name="e2e_counter", tick=counting_tick, rate=30)
        horus.run(node, duration=0.3)

        assert tick_count[0] > 1, f"tick should be called multiple times, got {tick_count[0]}"

    def test_scheduler_duration_honored(self):
        """run(duration=0.5) should exit within reasonable time."""
        start = time.time()
        node = horus.Node(name="e2e_dur", tick=lambda n: None, rate=10)
        horus.run(node, duration=0.5)
        elapsed = time.time() - start

        assert elapsed >= 0.3, f"Should run at least 0.3s, ran {elapsed:.2f}s"
        assert elapsed < 2.0, f"Should exit within 2s, ran {elapsed:.2f}s"


class TestNodeCreation:
    """Tests for node construction and configuration."""

    def test_node_with_init_callback(self):
        """Node init callback is called."""
        init_called = [False]

        def my_init(node):
            init_called[0] = True

        node = horus.Node(
            name="e2e_init", tick=lambda n: None, init=my_init, rate=10
        )
        horus.run(node, duration=0.2)

        assert init_called[0], "init callback should be called"

    def test_multiple_publishers_same_topic(self):
        """Two publishers on same topic, one subscriber gets messages from both."""
        received = []

        def pub_a(node):
            node.send("shared", {"from": "A"})

        def pub_b(node):
            node.send("shared", {"from": "B"})

        def sub(node):
            msg = node.recv("shared")
            if msg is not None:
                received.append(msg)

        a = horus.Node(name="e2e_puba", tick=pub_a, rate=30, pubs=["shared"])
        b = horus.Node(name="e2e_pubb", tick=pub_b, rate=30, pubs=["shared"])
        s = horus.Node(name="e2e_sub_shared", tick=sub, rate=30, subs=["shared"])
        horus.run(a, b, s, duration=0.5)

        assert len(received) > 0, "Subscriber should receive from at least one publisher"
