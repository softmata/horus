"""
Test Phase 1: Per-node rate control

Verifies that nodes can run at different rates independently.
"""

import horus
import time


def test_per_node_rate_control():
    """Test that nodes execute at their configured rates."""

    # Track tick counts for each node
    fast_ticks = []
    medium_ticks = []
    slow_ticks = []

    def fast_tick(node):
        """Fast node at 100Hz"""
        fast_ticks.append(time.time())

    def medium_tick(node):
        """Medium node at 50Hz"""
        medium_ticks.append(time.time())

    def slow_tick(node):
        """Slow node at 10Hz"""
        slow_ticks.append(time.time())

    # Create nodes
    fast_node = horus.Node(name="fast", tick=fast_tick, rate=100)
    medium_node = horus.Node(name="medium", tick=medium_tick, rate=50)
    slow_node = horus.Node(name="slow", tick=slow_tick, rate=10)

    # Create scheduler and register with different rates
    scheduler = horus.Scheduler()
    scheduler.add(fast_node, order=0, logging=False)
    scheduler.add(medium_node, order=1, logging=False)
    scheduler.add(slow_node, order=2, logging=False)

    # Run for 1 second
    scheduler.run(duration=1.0)

    # Check tick counts (allow 10% tolerance)
    print(f"Fast node ticks: {len(fast_ticks)} (expected ~100)")
    print(f"Medium node ticks: {len(medium_ticks)} (expected ~50)")
    print(f"Slow node ticks: {len(slow_ticks)} (expected ~10)")

    assert 90 <= len(fast_ticks) <= 110, f"Fast node ticks out of range: {len(fast_ticks)}"
    assert 45 <= len(medium_ticks) <= 55, f"Medium node ticks out of range: {len(medium_ticks)}"
    assert 9 <= len(slow_ticks) <= 11, f"Slow node ticks out of range: {len(slow_ticks)}"

    print(" Per-node rate control test passed!")


def test_runtime_rate_change():
    """Test changing node rate at runtime."""

    tick_count = [0]

    def counter_tick(node):
        tick_count[0] += 1

    node = horus.Node(name="counter", tick=counter_tick, rate=10)

    scheduler = horus.Scheduler()
    scheduler.add(node, order=0, logging=False)

    # Get initial stats
    stats = scheduler.get_node_stats("counter")
    print(f"Initial rate: {stats['rate_hz']}Hz")
    assert stats['rate_hz'] == 10.0

    # Change rate
    scheduler.set_node_rate("counter", 100.0)

    # Verify change
    stats = scheduler.get_node_stats("counter")
    print(f"Updated rate: {stats['rate_hz']}Hz")
    assert stats['rate_hz'] == 100.0

    print(" Runtime rate change test passed!")


def test_node_statistics():
    """Test node statistics API."""

    def dummy_tick(node):
        pass

    node = horus.Node(name="test_node", tick=dummy_tick, rate=50)

    scheduler = horus.Scheduler()
    scheduler.add(node, order=5, logging=True)

    stats = scheduler.get_node_stats("test_node")

    print(f"Node stats: {stats}")

    assert stats['name'] == "test_node"
    assert stats['order'] == 5
    assert stats['rate_hz'] == 50.0
    assert 'total_ticks' in stats
    assert 'errors_count' in stats

    print(" Node statistics test passed!")


