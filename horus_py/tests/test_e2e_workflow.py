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


class TestTypedTopicsAndErrorHandling:
    """Tests for typed topic communication and error propagation."""

    def test_typed_imu_roundtrip(self):
        """Imu typed message sent and received with correct field values."""
        received_imus = []

        def publisher(node):
            node.send("imu", horus.Imu(0.1, 0.2, 9.81, 0.01, 0.02, 0.03))

        def subscriber(node):
            msg = node.recv("imu")
            if msg is not None:
                received_imus.append(msg)

        pub = horus.Node(
            name="e2e_imu_pub",
            tick=publisher,
            rate=30,
            pubs=[horus.Imu],
        )
        sub = horus.Node(
            name="e2e_imu_sub",
            tick=subscriber,
            rate=30,
            subs=[horus.Imu],
        )
        horus.run(pub, sub, duration=0.5)

        assert len(received_imus) > 0, "Should receive Imu messages"
        imu = received_imus[0]
        assert abs(imu.accel_x - 0.1) < 0.01
        assert abs(imu.accel_y - 0.2) < 0.01
        assert abs(imu.accel_z - 9.81) < 0.01
        assert abs(imu.gyro_x - 0.01) < 0.001
        assert abs(imu.gyro_y - 0.02) < 0.001
        assert abs(imu.gyro_z - 0.03) < 0.001

    def test_node_exception_doesnt_crash_scheduler(self):
        """Exception in tick() is caught — scheduler completes without crash."""
        tick_count = [0]

        def exploding_tick(node):
            tick_count[0] += 1
            if tick_count[0] == 3:
                raise ValueError("Intentional test error at tick 3")

        node = horus.Node(name="e2e_explode", tick=exploding_tick, rate=30)
        # Scheduler should complete normally despite the exception
        horus.run(node, duration=0.5)

        # Node continued ticking after the exception
        assert tick_count[0] > 3, (
            f"Node should continue ticking after exception, got {tick_count[0]} ticks"
        )

    def test_send_wrong_type_raises_typeerror(self):
        """Sending wrong type to a typed topic raises TypeError, not segfault."""
        errors = []

        def wrong_type_tick(node):
            try:
                node.send("cmd_vel", "not_a_cmdvel")
            except TypeError as e:
                errors.append(str(e))

        node = horus.Node(
            name="e2e_wrong_type",
            tick=wrong_type_tick,
            rate=30,
            pubs=[horus.CmdVel],
        )
        horus.run(node, duration=0.3)

        assert len(errors) > 0, "Should raise TypeError for wrong type"
        assert "CmdVel" in errors[0], f"Error should mention CmdVel, got: {errors[0]}"

    def test_send_none_to_typed_topic_raises_typeerror(self):
        """Sending None to a typed topic raises TypeError, not segfault."""
        errors = []

        def none_tick(node):
            try:
                node.send("cmd_vel", None)
            except TypeError as e:
                errors.append(str(e))

        node = horus.Node(
            name="e2e_none_typed",
            tick=none_tick,
            rate=30,
            pubs=[horus.CmdVel],
        )
        horus.run(node, duration=0.3)

        assert len(errors) > 0, "Should raise TypeError for None on typed topic"

    def test_3_node_typed_pipeline_imu_to_cmdvel(self):
        """Imu sensor → filter → CmdVel controller pipeline with typed topics."""
        control_cmds = []

        def sensor_tick(node):
            node.send("imu", horus.Imu(0.0, 0.0, 9.81, 0.0, 0.0, 0.1))

        def filter_tick(node):
            msg = node.recv("imu")
            if msg is not None:
                # Convert gyro_z to angular velocity command
                node.send("cmd_vel", horus.CmdVel(1.0, msg.gyro_z * 10.0))

        def control_tick(node):
            msg = node.recv("cmd_vel")
            if msg is not None:
                control_cmds.append(msg)

        sensor = horus.Node(
            name="e2e_typed_sensor",
            tick=sensor_tick,
            rate=30,
            order=0,
            pubs=[horus.Imu],
        )
        filt = horus.Node(
            name="e2e_typed_filter",
            tick=filter_tick,
            rate=30,
            order=1,
            subs=[horus.Imu],
            pubs=[horus.CmdVel],
        )
        ctrl = horus.Node(
            name="e2e_typed_ctrl",
            tick=control_tick,
            rate=30,
            order=2,
            subs=[horus.CmdVel],
        )
        horus.run(sensor, filt, ctrl, duration=0.5)

        assert len(control_cmds) > 0, "Controller should receive CmdVel from pipeline"
        cmd = control_cmds[0]
        assert abs(cmd.linear - 1.0) < 0.01, f"linear should be 1.0, got {cmd.linear}"
        assert abs(cmd.angular - 1.0) < 0.01, f"angular should be 0.1*10=1.0, got {cmd.angular}"


class TestLogging:
    """Tests for node logging through scheduler and context guard."""

    def test_log_info_in_tick_completes(self):
        """node.log_info() inside tick doesn't crash scheduler."""
        log_count = [0]

        def tick(node):
            node.log_info(f"tick {log_count[0]}")
            log_count[0] += 1

        node = horus.Node(name="e2e_log_info", tick=tick, rate=30)
        horus.run(node, duration=0.3)

        assert log_count[0] > 1, f"tick should be called multiple times, got {log_count[0]}"

    def test_all_log_levels_in_tick(self):
        """All 4 log methods work without crash in a single tick."""
        tick_count = [0]

        def tick(node):
            node.log_info("info message")
            node.log_warning("warning message")
            node.log_error("error message")
            node.log_debug("debug message")
            tick_count[0] += 1

        node = horus.Node(name="e2e_log_levels", tick=tick, rate=30)
        horus.run(node, duration=0.3)

        assert tick_count[0] > 1, "all 4 log levels should work without crash"

    def test_log_outside_scheduler_warns(self):
        """Logging outside scheduler emits RuntimeWarning."""
        node = horus.Node(name="e2e_log_outside", tick=lambda n: None, rate=10)
        with pytest.warns(RuntimeWarning, match="outside scheduler"):
            node.log_info("should warn")

    def test_log_warning_outside_scheduler_warns(self):
        """log_warning outside scheduler also emits RuntimeWarning."""
        node = horus.Node(name="e2e_logwarn_outside", tick=lambda n: None, rate=10)
        with pytest.warns(RuntimeWarning, match="outside scheduler"):
            node.log_warning("should warn")

    def test_log_special_characters(self):
        """Unicode, quotes, newlines in log messages don't crash."""
        msgs_logged = [0]

        def tick(node):
            node.log_info("héllo wörld")
            node.log_info('msg with "quotes" and \t tabs')
            node.log_info("line1\nline2")
            node.log_warning("emoji test")
            msgs_logged[0] += 4

        node = horus.Node(name="e2e_log_special", tick=tick, rate=30)
        horus.run(node, duration=0.3)

        assert msgs_logged[0] > 0, "special character logging should not crash"

    def test_log_empty_string(self):
        """Empty log message doesn't crash."""
        tick_count = [0]

        def tick(node):
            node.log_info("")
            node.log_warning("")
            node.log_error("")
            node.log_debug("")
            tick_count[0] += 1

        node = horus.Node(name="e2e_log_empty", tick=tick, rate=30)
        horus.run(node, duration=0.3)

        assert tick_count[0] > 1, "empty string logging should not crash"

    def test_multi_node_logging_concurrent(self):
        """3 nodes all logging simultaneously, scheduler completes."""
        counts = {"a": [0], "b": [0], "c": [0]}

        def tick_a(node):
            node.log_info(f"node_a tick {counts['a'][0]}")
            counts["a"][0] += 1

        def tick_b(node):
            node.log_warning(f"node_b tick {counts['b'][0]}")
            counts["b"][0] += 1

        def tick_c(node):
            node.log_error(f"node_c tick {counts['c'][0]}")
            counts["c"][0] += 1

        a = horus.Node(name="e2e_log_a", tick=tick_a, rate=30)
        b = horus.Node(name="e2e_log_b", tick=tick_b, rate=30)
        c = horus.Node(name="e2e_log_c", tick=tick_c, rate=30)
        horus.run(a, b, c, duration=0.5)

        assert counts["a"][0] > 1, "node_a should tick multiple times"
        assert counts["b"][0] > 1, "node_b should tick multiple times"
        assert counts["c"][0] > 1, "node_c should tick multiple times"

    def test_log_error_through_scheduler_for_error_buffer(self):
        """node.log_error() through scheduler exercises the PyO3→Rust→publish_log→error_buffer path."""
        error_count = [0]

        def tick(node):
            node.log_error(f"critical failure #{error_count[0]}")
            error_count[0] += 1

        node = horus.Node(name="e2e_log_errbuf", tick=tick, rate=30)
        horus.run(node, duration=0.3)

        assert error_count[0] > 1, (
            f"node should log multiple errors, got {error_count[0]}. "
            "This exercises the full path: Python log_error → PyO3 → hlog!(error) → "
            "publish_log → GLOBAL_ERROR_BUFFER dual-write."
        )


class TestRecording:
    """Tests for recording flag through Python scheduler."""

    def test_scheduler_recording_flag_accepted(self):
        """Scheduler(recording=True) is accepted and completes without crash."""
        tick_count = [0]

        def tick(node):
            tick_count[0] += 1

        node = horus.Node(name="e2e_rec_flag", tick=tick, rate=30)
        sched = horus.Scheduler(recording=True)
        sched.add(node)
        sched.run(duration=0.2)

        assert tick_count[0] > 1, "scheduler with recording=True should tick normally"

    def test_scheduler_recording_false_also_works(self):
        """Scheduler(recording=False) is the default and works."""
        tick_count = [0]

        def tick(node):
            tick_count[0] += 1

        node = horus.Node(name="e2e_rec_false", tick=tick, rate=30)
        sched = horus.Scheduler(recording=False)
        sched.add(node)
        sched.run(duration=0.2)

        assert tick_count[0] > 1, "scheduler with recording=False should tick normally"
