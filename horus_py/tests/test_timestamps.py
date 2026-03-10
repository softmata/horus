"""
Test Phase 2: Automatic message timestamps

Verifies that messages have automatic timestamps and staleness detection works.
"""

import horus
from horus import Pose2D
import time


def test_automatic_timestamps():
    """Test that messages automatically get timestamps."""

    received_msg = []
    received_timestamp = []

    def publisher_tick(node):
        """Publish a message"""
        node.send("Pose2D", Pose2D(x=1.0, y=2.0, theta=0.5))

    def subscriber_tick(node):
        """Receive and check timestamp"""
        if node.has_msg("Pose2D"):
            result = node.get_with_timestamp("Pose2D")
            if result:
                msg, timestamp = result
                received_msg.append(msg)
                received_timestamp.append(timestamp)

    pub_node = horus.Node(name="publisher", pubs="Pose2D", tick=publisher_tick)
    sub_node = horus.Node(name="subscriber", subs="Pose2D", tick=subscriber_tick)

    scheduler = horus.Scheduler()
    scheduler.add(sub_node, order=0)
    scheduler.add(pub_node, order=1)

    # Run for a short time
    scheduler.run(duration=0.5)

    # Verify we received messages with timestamps
    assert len(received_msg) > 0, "Should have received messages"
    assert len(received_timestamp) > 0, "Should have timestamps"

    # Verify timestamp is reasonable (within last 10 seconds)
    now = time.time()
    for ts in received_timestamp:
        assert ts > 0, "Timestamp should be positive"
        assert abs(now - ts) < 10, f"Timestamp should be recent: {now - ts}s ago"


def test_message_age():
    """Test get_message_age() method."""

    def slow_publisher_tick(node):
        """Publish slowly"""
        node.send("Pose2D", Pose2D(x=1.0, y=0.0, theta=0.0))

    def age_checker_tick(node):
        """Check message age"""
        if node.has_msg("Pose2D"):
            age = node.get_message_age("Pose2D")
            if age is not None:
                assert age >= 0, "Age should be non-negative"
                assert age < 1.0, "Age should be recent (< 1 second)"
                node.get("Pose2D")  # Consume the message

    pub_node = horus.Node(name="slow_pub", pubs="Pose2D", tick=slow_publisher_tick)
    sub_node = horus.Node(name="age_check", subs="Pose2D", tick=age_checker_tick)

    scheduler = horus.Scheduler()
    scheduler.add(sub_node, order=0)
    scheduler.add(pub_node, order=1)

    scheduler.run(duration=0.5)


def test_staleness_detection():
    """Test is_stale() method."""

    published_count = [0]

    def publisher_tick(node):
        """Publish once then stop"""
        if published_count[0] == 0:
            node.send("Pose2D", Pose2D(x=1.0, y=0.0, theta=0.0))
            published_count[0] += 1

    def staleness_checker_tick(node):
        """Check for stale messages"""
        if node.has_msg("Pose2D"):
            # Check with different thresholds
            is_stale_10s = node.is_stale("Pose2D", 10.0)

            # Message should be fresh (< 10s old)
            assert not is_stale_10s, "Message should not be stale with 10s threshold"

            node.get("Pose2D")  # Consume

    pub_node = horus.Node(name="once_pub", pubs="Pose2D", tick=publisher_tick)
    check_node = horus.Node(name="stale_check", subs="Pose2D", tick=staleness_checker_tick)

    scheduler = horus.Scheduler()
    scheduler.add(check_node, order=0)
    scheduler.add(pub_node, order=1)

    # Run long enough for message to become > 100ms old
    scheduler.run(duration=0.3)


def test_get_timestamp():
    """Test get_timestamp() without consuming message."""

    def publisher_tick(node):
        node.send("Pose2D", Pose2D(x=1.0, y=0.0, theta=0.0))

    def timestamp_reader_tick(node):
        if node.has_msg("Pose2D"):
            # Get timestamp without consuming
            ts1 = node.get_timestamp("Pose2D")
            ts2 = node.get_timestamp("Pose2D")

            # Should get same timestamp both times
            assert ts1 is not None, "Should have timestamp"
            assert ts1 == ts2, "Timestamp should be same (message not consumed)"
            assert ts1 > 0, "Timestamp should be valid"

            # Now consume
            node.get("Pose2D")

            # Should be None after consuming
            ts3 = node.get_timestamp("Pose2D")
            assert ts3 is None, "Should have no timestamp after consuming"

    pub_node = horus.Node(name="ts_pub", pubs="Pose2D", tick=publisher_tick)
    read_node = horus.Node(name="ts_read", subs="Pose2D", tick=timestamp_reader_tick)

    scheduler = horus.Scheduler()
    scheduler.add(read_node, order=0)
    scheduler.add(pub_node, order=1)

    scheduler.run(duration=0.3)
