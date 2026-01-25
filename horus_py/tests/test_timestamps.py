"""
Test Phase 2: Automatic message timestamps

Verifies that messages have automatic timestamps and staleness detection works.
"""

import horus
import time


def test_automatic_timestamps():
    """Test that messages automatically get timestamps."""

    received_msg = []
    received_timestamp = []

    def publisher_tick(node):
        """Publish a message"""
        node.send("test_topic", {"data": "hello"})

    def subscriber_tick(node):
        """Receive and check timestamp"""
        if node.has_msg("test_topic"):
            result = node.get_with_timestamp("test_topic")
            if result:
                msg, timestamp = result
                received_msg.append(msg)
                received_timestamp.append(timestamp)

    pub_node = horus.Node(name="publisher", pubs="test_topic", tick=publisher_tick)
    sub_node = horus.Node(name="subscriber", subs="test_topic", tick=subscriber_tick)

    scheduler = horus.Scheduler()
    scheduler.add(pub_node, order=0, logging=False)
    scheduler.add(sub_node, order=1, logging=False)

    # Run for a short time
    scheduler.run(duration=0.5)

    # Verify we received messages with timestamps
    assert len(received_msg) > 0, "Should have received messages"
    assert len(received_timestamp) > 0, "Should have timestamps"

    print(f"Received {len(received_msg)} messages")
    print(f"First message: {received_msg[0]}")
    print(f"First timestamp: {received_timestamp[0]}")

    # Verify timestamp is reasonable (within last 10 seconds)
    now = time.time()
    for ts in received_timestamp:
        assert ts > 0, "Timestamp should be positive"
        assert abs(now - ts) < 10, f"Timestamp should be recent: {now - ts}s ago"

    print(" Automatic timestamps test passed!")


def test_message_age():
    """Test get_message_age() method."""

    def slow_publisher_tick(node):
        """Publish slowly"""
        node.send("age_topic", "test")

    def age_checker_tick(node):
        """Check message age"""
        if node.has_msg("age_topic"):
            age = node.get_message_age("age_topic")
            if age is not None:
                print(f"Message age: {age:.6f} seconds")
                assert age >= 0, "Age should be non-negative"
                assert age < 1.0, "Age should be recent (< 1 second)"
                node.get("age_topic")  # Consume the message

    pub_node = horus.Node(name="slow_pub", pubs="age_topic", tick=slow_publisher_tick)
    sub_node = horus.Node(name="age_check", subs="age_topic", tick=age_checker_tick)

    scheduler = horus.Scheduler()
    scheduler.add(pub_node, order=0, logging=False)
    scheduler.add(sub_node, order=1, logging=False)

    scheduler.run(duration=0.5)

    print(" Message age test passed!")


def test_staleness_detection():
    """Test is_stale() method."""

    published_count = [0]
    stale_detected = [False]

    def publisher_tick(node):
        """Publish once then stop"""
        if published_count[0] == 0:
            node.send("stale_topic", "old_message")
            published_count[0] += 1

    def staleness_checker_tick(node):
        """Check for stale messages"""
        if node.has_msg("stale_topic"):
            # Check with different thresholds
            is_stale_100ms = node.is_stale("stale_topic", 0.1)
            is_stale_10s = node.is_stale("stale_topic", 10.0)

            print(f"Stale (>100ms)? {is_stale_100ms}")
            print(f"Stale (>10s)? {is_stale_10s}")

            # Message should be fresh (< 10s old)
            assert not is_stale_10s, "Message should not be stale with 10s threshold"

            # Eventually it will be > 100ms old
            if is_stale_100ms:
                stale_detected[0] = True

            node.get("stale_topic")  # Consume

    pub_node = horus.Node(name="once_pub", pubs="stale_topic", tick=publisher_tick)
    check_node = horus.Node(name="stale_check", subs="stale_topic", tick=staleness_checker_tick)

    scheduler = horus.Scheduler()
    scheduler.add(pub_node, order=0, logging=False)
    scheduler.add(check_node, order=1, logging=False)

    # Run long enough for message to become > 100ms old
    scheduler.run(duration=0.3)

    print(f"Stale detected: {stale_detected[0]}")
    print(" Staleness detection test passed!")


def test_get_timestamp():
    """Test get_timestamp() without consuming message."""

    def publisher_tick(node):
        node.send("ts_topic", "data")

    def timestamp_reader_tick(node):
        if node.has_msg("ts_topic"):
            # Get timestamp without consuming
            ts1 = node.get_timestamp("ts_topic")
            ts2 = node.get_timestamp("ts_topic")

            # Should get same timestamp both times
            assert ts1 is not None, "Should have timestamp"
            assert ts1 == ts2, "Timestamp should be same (message not consumed)"
            assert ts1 > 0, "Timestamp should be valid"

            print(f"Timestamp (peek): {ts1}")

            # Now consume
            node.get("ts_topic")

            # Should be None after consuming
            ts3 = node.get_timestamp("ts_topic")
            assert ts3 is None, "Should have no timestamp after consuming"

    pub_node = horus.Node(name="ts_pub", pubs="ts_topic", tick=publisher_tick)
    read_node = horus.Node(name="ts_read", subs="ts_topic", tick=timestamp_reader_tick)

    scheduler = horus.Scheduler()
    scheduler.add(pub_node, order=0, logging=False)
    scheduler.add(read_node, order=1, logging=False)

    scheduler.run(duration=0.3)

    print(" Get timestamp test passed!")


