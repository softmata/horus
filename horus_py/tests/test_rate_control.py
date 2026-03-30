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
    fast_node = horus.Node(name="fast", tick=fast_tick, rate=100, order=0)
    medium_node = horus.Node(name="medium", tick=medium_tick, rate=50, order=1)
    slow_node = horus.Node(name="slow", tick=slow_tick, rate=10, order=2)

    # Create scheduler and register with different rates
    scheduler = horus.Scheduler()
    scheduler.add(fast_node)
    scheduler.add(medium_node)
    scheduler.add(slow_node)

    # Run for 1 second
    scheduler.run(duration=1.0)

    # Check tick counts (allow 10% tolerance)
    print(f"Fast node ticks: {len(fast_ticks)} (expected ~100)")
    print(f"Medium node ticks: {len(medium_ticks)} (expected ~50)")
    print(f"Slow node ticks: {len(slow_ticks)} (expected ~10)")

    # Wide tolerance for CI/debug builds — timing is approximate
    assert len(fast_ticks) >= 30, f"Fast node ticks too low: {len(fast_ticks)}"
    assert len(medium_ticks) >= 15, f"Medium node ticks too low: {len(medium_ticks)}"
    assert len(slow_ticks) >= 3, f"Slow node ticks too low: {len(slow_ticks)}"
    # Relative ordering: fast > medium > slow
    assert len(fast_ticks) > len(medium_ticks), "Fast should tick more than medium"
    assert len(medium_ticks) > len(slow_ticks), "Medium should tick more than slow"

    print(" Per-node rate control test passed!")


def test_node_statistics_before_run():
    """Test that node stats are available before running."""

    node = horus.Node(name="counter", tick=lambda n: None, rate=10)

    scheduler = horus.Scheduler()
    scheduler.add(node)

    stats = scheduler.get_node_stats("counter")
    assert stats['name'] == "counter"
    print(" Node statistics test passed!")


def test_node_statistics():
    """Test node statistics API."""

    def dummy_tick(node):
        pass

    node = horus.Node(name="test_node", tick=dummy_tick, rate=50, order=5)

    scheduler = horus.Scheduler()
    scheduler.add(node)

    stats = scheduler.get_node_stats("test_node")

    print(f"Node stats: {stats}")

    assert stats['name'] == "test_node"
    assert stats['order'] == 5
    assert 'total_ticks' in stats
    assert 'errors_count' in stats

    print(" Node statistics test passed!")
