"""
Integration tests for the composable scheduler configuration API.

Tests use the unified Python API: Node(tick=fn, ...kwargs) + run()/Scheduler.

Tests cover:
- SchedulerConfig composable builders (with_watchdog, rate, blackbox, etc.)
- Scheduler construction with builder params
- Node rate via Node kwargs
- Builder chaining with overrides
"""

import pytest


# ============================================================================
# SchedulerConfig Composable Builders
# ============================================================================

class TestSchedulerConfigBuilders:
    """Test SchedulerConfig composable builder methods."""

    def test_minimal_config(self):
        from horus._horus import SchedulerConfig
        cfg = SchedulerConfig.minimal()
        assert cfg.tick_rate == 100.0
        assert cfg.watchdog_timeout_ms == 0
        assert cfg.memory_locking is False

    def test_with_watchdog_config(self):
        from horus._horus import SchedulerConfig
        cfg = SchedulerConfig.with_watchdog()
        assert cfg.watchdog_timeout_ms == 500

    def test_watchdog_config_repr(self):
        from horus._horus import SchedulerConfig
        cfg = SchedulerConfig.with_watchdog()
        r = repr(cfg)
        assert "Watchdog" in r

    def test_watchdog_config_override_tick_rate(self):
        from horus._horus import SchedulerConfig
        cfg = SchedulerConfig.with_watchdog()
        cfg.tick_rate = 500.0
        assert cfg.tick_rate == 500.0
        # Watchdog settings preserved
        assert cfg.watchdog_timeout_ms == 500

    def test_watchdog_config_chaining(self):
        from horus._horus import SchedulerConfig
        cfg = SchedulerConfig.with_watchdog()
        cfg = cfg.rate(500.0).blackbox_mb(128)
        assert cfg.tick_rate == 500.0

    def test_config_cpu_affinity(self):
        from horus._horus import SchedulerConfig
        cfg = SchedulerConfig.minimal()
        cfg = cfg.cpu_affinity([0, 1])
        assert cfg.cpu_cores == [0, 1]

    def test_config_telemetry(self):
        from horus._horus import SchedulerConfig
        cfg = SchedulerConfig.minimal()
        cfg = cfg.telemetry("http://localhost:9090")
        assert cfg.telemetry_endpoint == "http://localhost:9090"


# ============================================================================
# Scheduler Construction & Node Addition
# ============================================================================

class TestSchedulerNodeAddition:
    """Test adding nodes to the scheduler via unified API."""

    def test_basic_scheduler_creation(self):
        from horus import Scheduler
        s = Scheduler(tick_rate=100.0)
        assert s is not None

    def test_with_watchdog_creates_scheduler(self):
        from horus import Scheduler
        s = Scheduler(tick_rate=100.0, watchdog_ms=500)
        assert s is not None

    def test_scheduler_can_add_nodes(self):
        from horus import Scheduler, Node

        node = Node(name="test_node", tick=lambda n: None, rate=50, order=0)
        s = Scheduler(tick_rate=100.0)
        s.add(node)

    def test_scheduler_can_run_briefly(self):
        from horus import Scheduler, Node

        state = {"count": 0}

        def counting_tick(node):
            state["count"] += 1

        n = Node(name="tick_counter", tick=counting_tick, rate=100, order=0)
        s = Scheduler(tick_rate=100.0)
        s.add(n)
        s.run(duration=0.1)
        assert state["count"] > 0

    def test_watchdog_scheduler_can_run(self):
        from horus import Scheduler, Node

        node = Node(name="monitored_node", tick=lambda n: None, rate=100, order=0)
        s = Scheduler(tick_rate=100.0, watchdog_ms=500)
        s.add(node)
        s.run(duration=0.05)

    def test_add_rejects_non_node(self):
        from horus import Scheduler

        s = Scheduler(tick_rate=100.0)
        with pytest.raises(TypeError):
            s.add("not a node")


# ============================================================================
# Node Rate via Node kwargs
# ============================================================================

class TestNodeRateConfig:
    """Test that node rate is set via Node kwargs."""

    def test_rate_via_node_kwarg(self):
        from horus import Scheduler, Node

        node = Node(name="rate_test", tick=lambda n: None, rate=50, order=0)
        s = Scheduler(tick_rate=100.0)
        s.add(node)

    def test_rate_on_node_attribute(self):
        from horus import Scheduler, Node

        node = Node(name="rated_node", tick=lambda n: None, rate=25, order=0)
        assert node.rate == 25
        s = Scheduler(tick_rate=100.0)
        s.add(node)

    def test_rate_must_be_positive(self):
        from horus import Node

        with pytest.raises(ValueError):
            Node(name="bad", tick=lambda n: None, rate=0)

        with pytest.raises(ValueError):
            Node(name="bad", tick=lambda n: None, rate=-1)


# ============================================================================
# Miss Enum and on_miss
# ============================================================================

class TestMissEnum:
    """Test Miss deadline policy enum and on_miss wiring."""

    def test_miss_constants_exist(self):
        from horus import Miss
        assert Miss.WARN == "warn"
        assert Miss.SKIP == "skip"
        assert Miss.SAFE_MODE == "safe_mode"
        assert Miss.STOP == "stop"

    def test_on_miss_via_node_kwarg(self):
        from horus import Scheduler, Node, Miss

        node = Node(name="miss_test", tick=lambda n: None, rate=100, order=0, on_miss="skip")
        s = Scheduler(tick_rate=100.0)
        s.add(node)

    def test_budget_via_node_kwarg(self):
        from horus import Scheduler, Node

        node = Node(name="budget_test", tick=lambda n: None, rate=1000, order=0, budget=0.0005)
        s = Scheduler(tick_rate=100.0)
        s.add(node)

    def test_safety_stats_with_watchdog(self):
        from horus import Scheduler, Node

        node = Node(name="stats_test", tick=lambda n: None, rate=100, order=0)
        s = Scheduler(tick_rate=100.0, watchdog_ms=500)
        s.add(node)
        s.run(duration=0.05)
        stats = s._scheduler.safety_stats()
        if stats is not None:
            assert "degrade_activations" in stats


# ============================================================================
# Node scheduling kwargs
# ============================================================================

class TestNodeSchedulingKwargs:
    """Test all Node scheduling kwargs."""

    def test_order_kwarg(self):
        from horus import Node
        n = Node(name="test", tick=lambda n: None, order=5)
        assert n.order == 5

    def test_budget_deadline_kwarg(self):
        from horus import Node
        n = Node(name="test", tick=lambda n: None, budget=0.0003, deadline=0.0009)
        assert n.budget == 0.0003
        assert n.deadline == 0.0009

    def test_compute_kwarg(self):
        from horus import Node
        n = Node(name="test", tick=lambda n: None, compute=True)
        assert n._horus_compute is True

    def test_on_kwarg(self):
        from horus import Node
        n = Node(name="test", tick=lambda n: None, on="lidar_scan")
        assert n._horus_on == "lidar_scan"

    def test_priority_core_watchdog(self):
        from horus import Node
        n = Node(name="test", tick=lambda n: None, priority=0, core=2, watchdog=0.5)
        assert n.priority == 0
        assert n.core == 2
        assert n.watchdog == 0.5

    def test_mutually_exclusive_execution_classes(self):
        from horus import Node
        with pytest.raises(ValueError, match="mutually exclusive"):
            Node(name="test", tick=lambda n: None, compute=True, on="topic")

    def test_async_auto_detection(self):
        import asyncio
        from horus import Node

        async def async_tick(node):
            pass

        n = Node(name="test", tick=async_tick)
        assert n._horus_async is True

    def test_failure_policy_kwarg(self):
        from horus import Node
        n = Node(name="test", tick=lambda n: None, failure_policy="restart")
        assert n.failure_policy == "restart"
