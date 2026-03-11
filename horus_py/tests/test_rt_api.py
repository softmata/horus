"""
Integration tests for the composable scheduler configuration API.

Tests cover:
- SchedulerConfig composable builders (with_monitoring, rate, blackbox, etc.)
- Scheduler construction with builder params
- Node rate via builder (not trait)
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
        assert cfg.tick_rate == 60.0
        assert cfg.safety_monitor is False
        assert cfg.memory_locking is False

    def test_with_monitoring_config(self):
        from horus._horus import SchedulerConfig
        cfg = SchedulerConfig.with_monitoring()
        assert cfg.safety_monitor is True
        assert cfg.budget_enforcement is True
        assert cfg.deadline_monitoring is True
        assert cfg.watchdog_enabled is True
        assert cfg.watchdog_timeout_ms == 500

    def test_monitoring_config_repr(self):
        from horus._horus import SchedulerConfig
        cfg = SchedulerConfig.with_monitoring()
        r = repr(cfg)
        assert "Monitoring" in r

    def test_monitoring_config_override_tick_rate(self):
        from horus._horus import SchedulerConfig
        cfg = SchedulerConfig.with_monitoring()
        cfg.tick_rate = 500.0
        assert cfg.tick_rate == 500.0
        # Monitoring settings preserved
        assert cfg.safety_monitor is True

    def test_monitoring_config_chaining(self):
        from horus._horus import SchedulerConfig
        cfg = SchedulerConfig.with_monitoring()
        cfg = cfg.rate(500.0).blackbox_mb(128)
        assert cfg.tick_rate == 500.0
        assert cfg.black_box_size_mb == 128
        assert cfg.safety_monitor is True

    def test_composable_rt_config(self):
        """Compose an RT-like config from builder methods."""
        from horus._horus import SchedulerConfig
        cfg = SchedulerConfig.with_monitoring()
        cfg = cfg.rate(100.0).blackbox_mb(64)
        cfg.memory_locking = True
        cfg.rt_scheduling_class = True
        cfg.max_deadline_misses = 10
        assert cfg.tick_rate == 100.0
        assert cfg.safety_monitor is True
        assert cfg.memory_locking is True
        assert cfg.rt_scheduling_class is True
        assert cfg.max_deadline_misses == 10

    def test_composable_strict_config(self):
        """Compose a strict config with profiling and tight deadlines."""
        from horus._horus import SchedulerConfig
        cfg = SchedulerConfig.with_monitoring()
        cfg.memory_locking = True
        cfg.rt_scheduling_class = True
        cfg.profiling = True
        cfg.max_deadline_misses = 3
        assert cfg.profiling is True
        assert cfg.max_deadline_misses == 3

    def test_builder_methods_chain(self):
        from horus._horus import SchedulerConfig
        cfg = SchedulerConfig.minimal()
        cfg = cfg.rate(50.0).watchdog_ms(200).blackbox_mb(32).cpu_affinity([0, 1])
        assert cfg.tick_rate == 50.0
        assert cfg.watchdog_enabled is True
        assert cfg.watchdog_timeout_ms == 200
        assert cfg.black_box_enabled is True
        assert cfg.black_box_size_mb == 32
        assert cfg.cpu_cores == [0, 1]

    def test_no_watchdog_disables(self):
        from horus._horus import SchedulerConfig
        cfg = SchedulerConfig.with_monitoring().no_watchdog()
        assert cfg.watchdog_enabled is False

    def test_telemetry_endpoint(self):
        from horus._horus import SchedulerConfig
        cfg = SchedulerConfig.minimal().telemetry("http://localhost:4317")
        assert cfg.telemetry_endpoint == "http://localhost:4317"


# ============================================================================
# Python Scheduler Construction
# ============================================================================

class TestSchedulerConstruction:
    """Test Scheduler creation with composable config."""

    def test_default_creates_scheduler(self):
        from horus import Scheduler
        s = Scheduler()
        assert s is not None

    def test_with_tick_rate(self):
        from horus import Scheduler
        s = Scheduler(tick_rate=500.0)
        assert s is not None

    def test_with_monitoring_creates_scheduler(self):
        from horus import Scheduler
        s = Scheduler(tick_rate=100.0, safety_monitor=True)
        assert s is not None

    def test_scheduler_can_add_nodes(self):
        from horus import Scheduler, Node

        class TestNode(Node):
            name = "test_node"
            def tick(self, info=None):
                pass

        s = Scheduler(tick_rate=100.0)
        s.add(TestNode(), order=0, rate=50.0)

    def test_scheduler_can_run_briefly(self):
        from horus import Scheduler, Node

        class TickCounter(Node):
            name = "tick_counter"
            def __init__(self):
                super().__init__()
                self.count = 0
            def tick(self, info=None):
                self.count += 1

        s = Scheduler(tick_rate=100.0)
        node = TickCounter()
        s.add(node, order=0)
        s.run(duration=0.1)
        assert node.count > 0

    def test_monitoring_scheduler_can_run(self):
        from horus import Scheduler, Node

        class SimpleNode(Node):
            name = "monitored_node"
            def tick(self, info=None):
                pass

        s = Scheduler(tick_rate=100.0, safety_monitor=True)
        s.add(SimpleNode(), order=0)
        s.run(duration=0.05)


# ============================================================================
# Node Rate via Builder
# ============================================================================

class TestNodeRateBuilder:
    """Test that node rate is set via builder, not Node trait."""

    def test_rate_via_add_param(self):
        from horus import Scheduler, Node

        class SimpleNode(Node):
            name = "rate_test_node"
            def tick(self, info=None):
                pass

        s = Scheduler(tick_rate=100.0)
        # Rate set via add() parameter — this is the new API
        s.add(SimpleNode(), order=0, rate=50.0)

    def test_rate_via_node_attribute(self):
        from horus import Scheduler, Node

        class RatedNode(Node):
            name = "rated_node"
            rate = 25.0  # Fallback: node.rate attribute
            def tick(self, info=None):
                pass

        s = Scheduler(tick_rate=100.0)
        s.add(RatedNode(), order=0)

    def test_explicit_rate_overrides_node_attribute(self):
        from horus import Scheduler, Node

        class RatedNode(Node):
            name = "override_rate_node"
            rate = 25.0
            def tick(self, info=None):
                pass

        s = Scheduler(tick_rate=100.0)
        # Explicit rate param overrides node.rate
        s.add(RatedNode(), order=0, rate=75.0)


# ============================================================================
# Miss Enum and on_miss Builder
# ============================================================================

class TestMissEnum:
    """Test Miss deadline policy enum and on_miss wiring."""

    def test_miss_constants_exist(self):
        from horus import Miss
        assert Miss.WARN == "warn"
        assert Miss.SKIP == "skip"
        assert Miss.DEGRADE == "degrade"
        assert Miss.STOP == "stop"

    def test_on_miss_via_add(self):
        from horus import Scheduler, Node, Miss

        class SimpleNode(Node):
            name = "miss_test_node"
            def tick(self, info=None):
                pass

        s = Scheduler(tick_rate=100.0)
        s.add(SimpleNode(), order=0, on_miss=Miss.SKIP)

    def test_on_miss_via_builder(self):
        from horus import Scheduler, Node, Miss

        class SimpleNode(Node):
            name = "builder_miss_node"
            def tick(self, info=None):
                pass

        s = Scheduler(tick_rate=100.0)
        # Use fluent builder API via the native scheduler
        s._scheduler.node(SimpleNode()).order(0).on_miss(Miss.SAFE_MODE).build()

    def test_budget_via_add(self):
        from horus import Scheduler, Node

        class SimpleNode(Node):
            name = "budget_test_node"
            def tick(self, info=None):
                pass

        s = Scheduler(tick_rate=100.0)
        s.add(SimpleNode(), order=0, budget=500)

    def test_safety_stats_with_monitoring(self):
        from horus import Scheduler, Node

        class SimpleNode(Node):
            name = "stats_test_node"
            def tick(self, info=None):
                pass

        s = Scheduler(tick_rate=100.0, safety_monitor=True)
        s.add(SimpleNode(), order=0)
        s.run(duration=0.05)
        stats = s._scheduler.safety_stats()
        if stats is not None:
            assert "degrade_activations" in stats
