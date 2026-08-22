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


# ============================================================================
# Safe-state hook (Miss.SAFE_MODE)
# ============================================================================

class TestSafeStateHook:
    """`on_miss="safe_mode"` must reach a Python node's enter_safe_state()."""

    def test_enter_safe_state_is_called_on_deadline_miss(self):
        """Miss.SAFE_MODE is documented as "calls enter_safe_state() on the node".

        Regression: PyNodeAdapter implemented only init/tick/shutdown, so a
        Python node fell through to the no-op default in horus_core. The
        scheduler detected the misses and degraded — safety_stats() showed
        deadline_misses and degrade_activations — while the hook that stops
        the motors was never invoked. Rust and C++ nodes both got it, so this
        was a three-language parity gap on a safety path.
        """
        import time
        import horus

        called = []

        class Overrunner(horus.Node):
            def __init__(self):
                super().__init__(
                    name="test_safe_state_overrunner",
                    rate=100,
                    budget=0.001,
                    deadline=0.002,
                    on_miss="safe_mode",
                )
                self.ticks = 0

            def tick(self, info=None):
                self.ticks += 1
                time.sleep(0.005)  # 5 ms against a 2 ms deadline

            def enter_safe_state(self):
                called.append(self.ticks)

        node = Overrunner()
        sched = horus.Scheduler()
        sched.add(node)
        sched.run(duration=0.5)

        stats = sched.safety_stats()
        assert stats["deadline_misses"] > 0, (
            f"test premise broken: no deadline was missed ({stats})"
        )
        assert called, (
            "enter_safe_state() was never called despite "
            f"{stats['deadline_misses']} deadline misses and "
            f"{stats['degrade_activations']} degrade activations"
        )

    def test_node_without_the_hook_still_runs(self):
        """Not defining enter_safe_state must stay harmless."""
        import time
        import horus

        class Plain(horus.Node):
            def __init__(self):
                super().__init__(
                    name="test_safe_state_plain",
                    rate=100,
                    budget=0.001,
                    deadline=0.002,
                    on_miss="safe_mode",
                )
                self.ticks = 0

            def tick(self, info=None):
                self.ticks += 1
                time.sleep(0.005)

        node = Plain()
        sched = horus.Scheduler()
        sched.add(node)
        sched.run(duration=0.3)
        assert node.ticks > 0
