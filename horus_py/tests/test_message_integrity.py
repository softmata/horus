"""
Message Integrity Tests - Python API via Rust Backend

Tests that verify all message types work correctly through the
Rust backend, ensuring data integrity across the Python-Rust boundary.
"""

import horus
from horus import Pose2D, Twist, Point3, Vector3, Quaternion, CmdVel, LaserScan


class TestPose2D:
    """Test Pose2D message integrity"""

    def test_pose2d_basic(self):
        """Test basic Pose2D creation and properties"""
        pose = Pose2D(x=1.5, y=2.5, theta=0.785)
        assert pose.x == 1.5
        assert pose.y == 2.5
        assert abs(pose.theta - 0.785) < 1e-10

    def test_pose2d_pubsub(self):
        """Test Pose2D through pub/sub"""
        test_values = []
        received_values = []
        tick_count = [0]

        def publisher(node):
            tick = tick_count[0]
            tick_count[0] += 1
            if tick < 5:
                x, y, theta = float(tick), float(tick) * 2.0, float(tick) * 0.5
                pose = Pose2D(x=x, y=y, theta=theta)
                node.send("Pose2D", pose)
                test_values.append((x, y, theta))
            else:
                node.request_stop()

        def subscriber(node):
            msg = node.get("Pose2D")
            if msg:
                received_values.append((msg.x, msg.y, msg.theta))
            if len(received_values) >= 5:
                node.request_stop()

        pub = horus.Node(name="pub", pubs="Pose2D", tick=publisher)
        sub = horus.Node(name="sub", subs="Pose2D", tick=subscriber)

        horus.run(pub, sub, duration=1.0)

        assert len(received_values) == 5
        for sent, received in zip(test_values, received_values):
            assert abs(sent[0] - received[0]) < 1e-10
            assert abs(sent[1] - received[1]) < 1e-10
            assert abs(sent[2] - received[2]) < 1e-10

    def test_pose2d_edge_cases(self):
        """Test Pose2D with edge case values"""
        edge_cases = [
            (0.0, 0.0, 0.0),
            (-100.0, -200.0, -3.14159),
            (1e6, 1e6, 6.28),
            (1e-10, 1e-10, 1e-10),
        ]

        for x, y, theta in edge_cases:
            pose = Pose2D(x=x, y=y, theta=theta)
            assert abs(pose.x - x) < 1e-9
            assert abs(pose.y - y) < 1e-9
            assert abs(pose.theta - theta) < 1e-9


class TestTwist:
    """Test Twist message integrity"""

    def test_twist_2d(self):
        """Test 2D twist creation via constructor kwargs"""
        twist = Twist(linear_x=1.5, angular_z=0.5)
        assert twist.linear_x == 1.5
        assert twist.angular_z == 0.5

    def test_twist_pubsub(self):
        """Test Twist through pub/sub"""
        received = []
        sent = [False]

        def pub_node(node):
            if not sent[0]:
                twist = Twist(linear_x=2.5, angular_z=1.2)
                node.send("Twist", twist)
                sent[0] = True
            elif sent[0]:
                node.request_stop()

        def sub_node(node):
            msg = node.get("Twist")
            if msg:
                received.append((msg.linear_x, msg.angular_z))
                node.request_stop()

        pub = horus.Node(name="pub", pubs="Twist", tick=pub_node)
        sub = horus.Node(name="sub", subs="Twist", tick=sub_node)

        horus.run(pub, sub, duration=1.0)

        assert len(received) > 0
        assert abs(received[0][0] - 2.5) < 1e-10
        assert abs(received[0][1] - 1.2) < 1e-10


class TestCmdVel:
    """Test CmdVel message integrity"""

    def test_cmdvel_basic(self):
        """Test CmdVel creation and properties"""
        cmd = CmdVel(linear=1.0, angular=0.5)
        assert abs(cmd.linear - 1.0) < 1e-6
        assert abs(cmd.angular - 0.5) < 1e-6

    def test_cmdvel_pubsub(self):
        """Test CmdVel through pub/sub with various values"""
        test_data = [
            (0.0, 0.0),
            (1.5, 0.5),
            (-0.5, -0.2),
            (2.0, 1.0),
        ]
        received_data = []
        tick_count = [0]

        def pub_node(node):
            tick = tick_count[0]
            tick_count[0] += 1
            if tick < len(test_data):
                linear, angular = test_data[tick]
                cmd = CmdVel(linear=linear, angular=angular)
                node.send("CmdVel", cmd)
            else:
                node.request_stop()

        def sub_node(node):
            msg = node.get("CmdVel")
            if msg:
                received_data.append((msg.linear, msg.angular))
            if len(received_data) >= len(test_data):
                node.request_stop()

        pub = horus.Node(name="pub", pubs="CmdVel", tick=pub_node)
        sub = horus.Node(name="sub", subs="CmdVel", tick=sub_node)

        horus.run(pub, sub, duration=1.0)

        assert len(received_data) == len(test_data)
        for sent, received in zip(test_data, received_data):
            assert abs(sent[0] - received[0]) < 1e-5
            assert abs(sent[1] - received[1]) < 1e-5


class TestLaserScan:
    """Test LaserScan message with NumPy integration"""

    def test_laserscan_getter(self):
        """Test LaserScan returns ranges as list"""
        scan = LaserScan()
        ranges = scan.ranges
        assert isinstance(ranges, list)
        assert len(ranges) == 360

    def test_laserscan_setter(self):
        """Test LaserScan accepts lists"""
        scan = LaserScan()
        test_ranges = [float(i % 10) for i in range(360)]
        scan.ranges = test_ranges

        retrieved = scan.ranges
        for a, b in zip(test_ranges, retrieved):
            assert abs(a - b) < 1e-5

    def test_laserscan_pubsub(self):
        """Test LaserScan through pub/sub"""
        sent_ranges = None
        received_ranges = None
        sent = [False]

        def pub_node(node):
            nonlocal sent_ranges
            if not sent[0]:
                scan = LaserScan()
                sent_ranges = [float(i) * 0.1 for i in range(360)]
                scan.ranges = sent_ranges
                node.send("LaserScan", scan)
                sent[0] = True
            else:
                node.request_stop()

        def sub_node(node):
            nonlocal received_ranges
            msg = node.get("LaserScan")
            if msg:
                received_ranges = list(msg.ranges)
                node.request_stop()

        pub = horus.Node(name="pub", pubs="LaserScan", tick=pub_node)
        sub = horus.Node(name="sub", subs="LaserScan", tick=sub_node)

        horus.run(pub, sub, duration=1.0)

        assert sent_ranges is not None
        assert received_ranges is not None
        for a, b in zip(sent_ranges, received_ranges):
            assert abs(a - b) < 1e-4


class TestGeometricTypes:
    """Test Point3, Vector3, Quaternion"""

    def test_point3(self):
        """Test Point3 through pub/sub"""
        sent = []
        received = []
        done = [False]

        def pub_node(node):
            if not done[0]:
                p = Point3(x=1.5, y=2.5, z=3.5)
                node.send("Point3", p)
                sent.append((p.x, p.y, p.z))
                done[0] = True
            else:
                node.request_stop()

        def sub_node(node):
            msg = node.get("Point3")
            if msg:
                received.append((msg.x, msg.y, msg.z))
                node.request_stop()

        pub = horus.Node(name="pub", pubs="Point3", tick=pub_node)
        sub = horus.Node(name="sub", subs="Point3", tick=sub_node)

        horus.run(pub, sub, duration=1.0)

        assert len(received) > 0
        assert abs(sent[0][0] - received[0][0]) < 1e-10
        assert abs(sent[0][1] - received[0][1]) < 1e-10
        assert abs(sent[0][2] - received[0][2]) < 1e-10

    def test_vector3(self):
        """Test Vector3 creation and field access"""
        v = Vector3(x=1.0, y=2.0, z=3.0)
        assert v.x == 1.0
        assert v.y == 2.0
        assert v.z == 3.0

        v2 = Vector3(x=4.0, y=5.0, z=6.0)
        assert v2.x == 4.0

    def test_quaternion(self):
        """Test Quaternion identity creation"""
        q = Quaternion()  # default is identity (0, 0, 0, 1)
        assert q.w == 1.0
        assert q.x == 0.0
        assert q.y == 0.0
        assert q.z == 0.0


class TestHighFrequency:
    """Test high-frequency communication"""

    def test_high_frequency_pose(self):
        """Test sustained high-rate Pose2D communication"""
        sent_count = [0]
        received_count = [0]

        def fast_pub(node):
            pose = Pose2D(x=float(sent_count[0]), y=0.0, theta=0.0)
            node.send("Pose2D", pose)
            sent_count[0] += 1

            if sent_count[0] >= 100:
                node.request_stop()

        def fast_sub(node):
            msg = node.get("Pose2D")
            if msg:
                received_count[0] += 1

        pub = horus.Node(name="pub", pubs="Pose2D", tick=fast_pub)
        sub = horus.Node(name="sub", subs="Pose2D", tick=fast_sub)

        horus.run(pub, sub, duration=1.0)

        # Should receive most messages
        reception_rate = received_count[0] / sent_count[0]
        assert reception_rate > 0.7, \
            f"Low reception rate: {reception_rate:.1%} ({received_count[0]}/{sent_count[0]})"


class TestErrorHandling:
    """Test error handling across Python-Rust boundary"""

    def test_error_recovery(self):
        """Test that errors don't crash the system"""
        error_count = [0]
        successful_ticks = [0]
        tick_count = [0]

        def faulty_node(node):
            tick = tick_count[0]
            tick_count[0] += 1
            if tick % 3 == 0:
                raise ValueError("Intentional error")
            successful_ticks[0] += 1

        def error_handler(node, error):
            error_count[0] += 1

        node = horus.Node(name="faulty", tick=faulty_node, on_error=error_handler)
        horus.run(node, duration=0.5)

        assert error_count[0] > 0, "No errors were caught"
        assert successful_ticks[0] > 0, "No successful ticks"


