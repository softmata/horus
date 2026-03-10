"""
Integration tests for the new RT API DX (profile-based scheduler configuration).

Tests cover:
- Scheduler profile presets (deploy, hard_rt, safety_critical)
- SchedulerConfig profile factories
- Node rate via builder (not trait)
- Profile chaining with overrides
"""

import pytest


# ============================================================================
# SchedulerConfig Profile Presets
# ============================================================================

class TestSchedulerConfigProfiles:
    """Test SchedulerConfig profile factory methods."""

    def test_minimal_config(self):
        from horus._horus import SchedulerConfig
        cfg = SchedulerConfig.minimal()
        assert cfg.tick_rate == 60.0
        assert cfg.fault_tolerance is False
        assert cfg.safety_monitor is False
        assert cfg.memory_locking is False

    def test_deploy_config(self):
        from horus._horus import SchedulerConfig
        cfg = SchedulerConfig.deploy()
        assert cfg.tick_rate == 100.0
        assert cfg.fault_tolerance is True
        assert cfg.safety_monitor is True
        assert cfg.budget_enforcement is True
        assert cfg.deadline_monitoring is True
        assert cfg.watchdog_enabled is True
        assert cfg.watchdog_timeout_ms == 500
        assert cfg.black_box_enabled is True
        assert cfg.black_box_size_mb == 64

    def test_hard_rt_config(self):
        from horus._horus import SchedulerConfig
        cfg = SchedulerConfig.hard_rt()
        # Has everything deploy has
        assert cfg.fault_tolerance is True
        assert cfg.safety_monitor is True
        # Plus RT-specific features
        assert cfg.memory_locking is True
        assert cfg.rt_scheduling_class is True
        assert cfg.max_deadline_misses == 10

    def test_safety_critical_config(self):
        from horus._horus import SchedulerConfig
        cfg = SchedulerConfig.safety_critical()
        # Has everything hard_rt has
        assert cfg.memory_locking is True
        assert cfg.rt_scheduling_class is True
        # Plus profiling and strict deadlines
        assert cfg.profiling is True
        assert cfg.max_deadline_misses == 3

    def test_deploy_config_repr(self):
        from horus._horus import SchedulerConfig
        cfg = SchedulerConfig.deploy()
        r = repr(cfg)
        assert "Deploy" in r
        assert "100.0Hz" in r

    def test_hard_rt_config_repr(self):
        from horus._horus import SchedulerConfig
        cfg = SchedulerConfig.hard_rt()
        r = repr(cfg)
        assert "HardRT" in r

    def test_safety_critical_config_repr(self):
        from horus._horus import SchedulerConfig
        cfg = SchedulerConfig.safety_critical()
        r = repr(cfg)
        assert "SafetyCritical" in r

    def test_deploy_config_override_tick_rate(self):
        from horus._horus import SchedulerConfig
        cfg = SchedulerConfig.deploy()
        cfg.tick_rate = 500.0
        assert cfg.tick_rate == 500.0
        # Other deploy settings preserved
        assert cfg.fault_tolerance is True
        assert cfg.safety_monitor is True

    def test_deploy_config_chaining(self):
        from horus._horus import SchedulerConfig
        cfg = SchedulerConfig.deploy()
        cfg = cfg.rate(500.0).blackbox_mb(128)
        assert cfg.tick_rate == 500.0
        assert cfg.black_box_size_mb == 128
        assert cfg.fault_tolerance is True


# ============================================================================
# Python Scheduler Profile Presets
# ============================================================================

class TestSchedulerProfiles:
    """Test Scheduler class profile constructors."""

    def test_deploy_creates_scheduler(self):
        from horus import Scheduler
        s = Scheduler.deploy()
        assert s is not None

    def test_deploy_with_tick_rate(self):
        from horus import Scheduler
        s = Scheduler.deploy(tick_rate=500.0)
        assert s is not None

    def test_hard_rt_creates_scheduler(self):
        from horus import Scheduler
        s = Scheduler.hard_rt()
        assert s is not None

    def test_safety_critical_creates_scheduler(self):
        from horus import Scheduler
        s = Scheduler.safety_critical()
        assert s is not None

    def test_deploy_can_add_nodes(self):
        from horus import Scheduler, Node

        class TestNode(Node):
            name = "test_deploy_node"
            def tick(self, info=None):
                pass

        s = Scheduler.deploy(tick_rate=100.0)
        s.add(TestNode(), order=0, rate=50.0)

    def test_deploy_can_run_briefly(self):
        from horus import Scheduler, Node

        class TickCounter(Node):
            name = "tick_counter"
            def __init__(self):
                super().__init__()
                self.count = 0
            def tick(self, info=None):
                self.count += 1

        s = Scheduler.deploy(tick_rate=100.0)
        node = TickCounter()
        s.add(node, order=0)
        s.run(duration=0.1)
        assert node.count > 0


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
        assert Miss.SAFE_MODE == "safe_mode"
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

    def test_safety_stats_has_safe_mode_activations(self):
        from horus import Scheduler, Node

        class SimpleNode(Node):
            name = "stats_test_node"
            def tick(self, info=None):
                pass

        s = Scheduler.deploy(tick_rate=100.0)
        s.add(SimpleNode(), order=0)
        s.run(duration=0.05)
        stats = s._scheduler.safety_stats()
        if stats is not None:
            assert "safe_mode_activations" in stats
