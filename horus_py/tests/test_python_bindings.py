"""
Tests for HORUS Python bindings: core Rust types exposed to Python.

Tests cover:
- Priority constants and conversion methods
- NodeState constants
- Transform creation and field access
- HFrame transform system
- get_version() utility
- Node creation and lifecycle
- Topic creation and basic send/recv
"""

import pytest
import uuid


# ============================================================================
# Priority Constants
# ============================================================================

class TestPriority:
    """Test Priority class constants and methods."""

    def test_priority_critical_is_zero(self):
        from horus._horus import Priority
        assert Priority.CRITICAL == 0

    def test_priority_high_is_10(self):
        from horus._horus import Priority
        assert Priority.HIGH == 10

    def test_priority_normal_is_50(self):
        from horus._horus import Priority
        assert Priority.NORMAL == 50

    def test_priority_low_is_80(self):
        from horus._horus import Priority
        assert Priority.LOW == 80

    def test_priority_background_is_100(self):
        from horus._horus import Priority
        assert Priority.BACKGROUND == 100

    def test_priority_ordering(self):
        """Lower number = higher priority."""
        from horus._horus import Priority
        assert Priority.CRITICAL < Priority.HIGH < Priority.NORMAL < Priority.LOW < Priority.BACKGROUND

    def test_from_string_critical(self):
        from horus._horus import Priority
        assert Priority.from_string("critical") == 0

    def test_from_string_high(self):
        from horus._horus import Priority
        assert Priority.from_string("high") == 10

    def test_from_string_normal(self):
        from horus._horus import Priority
        assert Priority.from_string("normal") == 50

    def test_from_string_low(self):
        from horus._horus import Priority
        assert Priority.from_string("low") == 80

    def test_from_string_background(self):
        from horus._horus import Priority
        assert Priority.from_string("background") == 100

    def test_from_string_case_insensitive(self):
        from horus._horus import Priority
        assert Priority.from_string("CRITICAL") == 0
        assert Priority.from_string("High") == 10
        assert Priority.from_string("NORMAL") == 50

    def test_from_string_numeric(self):
        from horus._horus import Priority
        assert Priority.from_string("42") == 42

    def test_from_string_invalid_raises(self):
        from horus._horus import Priority
        with pytest.raises(ValueError):
            Priority.from_string("invalid_string")

    def test_to_string_critical(self):
        from horus._horus import Priority
        assert Priority.to_string(0) == "critical"

    def test_to_string_normal(self):
        from horus._horus import Priority
        assert Priority.to_string(50) == "normal"

    def test_to_string_custom(self):
        from horus._horus import Priority
        result = Priority.to_string(42)
        assert "42" in result


# ============================================================================
# NodeState Constants
# ============================================================================

class TestNodeState:
    """Test NodeState class constants."""

    def test_uninitialized(self):
        from horus._horus import PyNodeState as NodeState
        assert NodeState.UNINITIALIZED == "uninitialized"

    def test_initializing(self):
        from horus._horus import PyNodeState as NodeState
        assert NodeState.INITIALIZING == "initializing"

    def test_running(self):
        from horus._horus import PyNodeState as NodeState
        assert NodeState.RUNNING == "running"

    def test_paused(self):
        from horus._horus import PyNodeState as NodeState
        assert NodeState.PAUSED == "paused"

    def test_stopping(self):
        from horus._horus import PyNodeState as NodeState
        assert NodeState.STOPPING == "stopping"

    def test_stopped(self):
        from horus._horus import PyNodeState as NodeState
        assert NodeState.STOPPED == "stopped"

    def test_error(self):
        from horus._horus import PyNodeState as NodeState
        assert NodeState.ERROR == "error"

    def test_crashed(self):
        from horus._horus import PyNodeState as NodeState
        assert NodeState.CRASHED == "crashed"

    def test_create_and_repr(self):
        from horus._horus import PyNodeState as NodeState
        state = NodeState("running")
        assert repr(state) == "NodeState('running')"
        assert str(state) == "running"

    def test_equality(self):
        from horus._horus import PyNodeState as NodeState
        s1 = NodeState("running")
        s2 = NodeState("running")
        s3 = NodeState("stopped")
        assert s1 == s2
        assert not (s1 == s3)


# ============================================================================
# Transform
# ============================================================================

class TestTransform:
    """Test PyTransform bindings."""

    def test_identity_transform(self):
        from horus._horus import Transform
        tf = Transform.identity()
        assert tf.translation == [0.0, 0.0, 0.0]
        assert tf.rotation == [0.0, 0.0, 0.0, 1.0]  # Identity quaternion

    def test_create_with_translation(self):
        from horus._horus import Transform
        tf = Transform(translation=[1.0, 2.0, 3.0])
        assert tf.translation == [1.0, 2.0, 3.0]
        assert tf.rotation == [0.0, 0.0, 0.0, 1.0]

    def test_create_with_translation_and_rotation(self):
        from horus._horus import Transform
        tf = Transform(translation=[1.0, 0.0, 0.0], rotation=[0.0, 0.0, 0.707, 0.707])
        assert tf.translation == [1.0, 0.0, 0.0]
        assert abs(tf.rotation[2] - 0.707) < 0.001
        assert abs(tf.rotation[3] - 0.707) < 0.001

    def test_from_translation(self):
        from horus._horus import Transform
        tf = Transform.from_translation([5.0, -3.0, 1.0])
        assert tf.translation == [5.0, -3.0, 1.0]

    def test_from_euler(self):
        from horus._horus import Transform
        import math
        # 90 degree yaw rotation
        tf = Transform.from_euler([0.0, 0.0, 0.0], [0.0, 0.0, math.pi / 2])
        assert tf.translation == [0.0, 0.0, 0.0]
        # Quaternion for 90deg yaw: [0, 0, sin(45), cos(45)] = [0, 0, 0.707, 0.707]
        assert abs(tf.rotation[2] - 0.707) < 0.01
        assert abs(tf.rotation[3] - 0.707) < 0.01

    def test_set_translation(self):
        from horus._horus import Transform
        tf = Transform.identity()
        tf.translation = [10.0, 20.0, 30.0]
        assert tf.translation == [10.0, 20.0, 30.0]

    def test_set_rotation(self):
        from horus._horus import Transform
        tf = Transform.identity()
        tf.rotation = [0.0, 0.0, 0.707, 0.707]
        assert abs(tf.rotation[2] - 0.707) < 0.001

    def test_default_constructor(self):
        from horus._horus import Transform
        tf = Transform()
        assert tf.translation == [0.0, 0.0, 0.0]
        assert tf.rotation == [0.0, 0.0, 0.0, 1.0]


# ============================================================================
# HFrame
# ============================================================================

class TestHFrame:
    """Test PyHFrame bindings."""

    def test_create_hframe(self):
        from horus._horus import HFrame
        hf = HFrame()
        # Should not raise
        assert hf is not None

    def test_register_frame(self):
        from horus._horus import HFrame, Transform
        hf = HFrame()
        world_id = hf.register_frame("world")
        base_id = hf.register_frame("base_link", parent="world")
        assert hf.has_frame("world")
        assert hf.has_frame("base_link")
        assert hf.frame_count() == 2

    def test_update_and_query_transform(self):
        from horus._horus import HFrame, Transform
        hf = HFrame()
        hf.register_frame("world")
        hf.register_frame("base_link", parent="world")
        tf = Transform(translation=[1.0, 2.0, 3.0])
        hf.update_transform("base_link", tf)
        # tf(base_link, world) = transform from child to parent = the direct transform
        result = hf.tf("base_link", "world")
        assert result is not None
        assert abs(result.translation[0] - 1.0) < 0.001
        assert abs(result.translation[1] - 2.0) < 0.001
        assert abs(result.translation[2] - 3.0) < 0.001

    def test_all_frames(self):
        from horus._horus import HFrame
        hf = HFrame()
        hf.register_frame("world")
        hf.register_frame("base_link", parent="world")
        frames = hf.all_frames()
        assert "world" in frames
        assert "base_link" in frames


# ============================================================================
# get_version()
# ============================================================================

class TestVersion:
    """Test version utility."""

    def test_get_version_returns_string(self):
        from horus._horus import get_version
        version = get_version()
        assert isinstance(version, str)

    def test_get_version_contains_version_number(self):
        from horus._horus import get_version
        version = get_version()
        assert "HORUS" in version
        # Should contain semver-like pattern
        assert "." in version

    def test_get_version_not_empty(self):
        from horus._horus import get_version
        assert len(get_version()) > 0


# ============================================================================
# Topic (basic creation and send/recv)
# ============================================================================

class TestTopic:
    """Test PyTopic bindings for basic send/recv."""

    def test_topic_send_recv_dict(self, unique_test_prefix):
        """Test sending and receiving a dictionary via Topic."""
        from horus._horus import Topic
        topic_name = f"{unique_test_prefix}_test_dict"
        topic = Topic(topic_name)
        topic.send({"x": 1.0, "y": 2.0})
        result = topic.recv()
        if result is not None:
            assert result["x"] == 1.0
            assert result["y"] == 2.0

    def test_topic_recv_empty(self, unique_test_prefix):
        """Topic recv on an empty topic returns None."""
        from horus._horus import Topic
        topic_name = f"{unique_test_prefix}_empty_recv"
        topic = Topic(topic_name)
        result = topic.recv()
        # First recv on a new topic should return None
        assert result is None


# ============================================================================
# Node (Python-level)
# ============================================================================

class TestPythonNode:
    """Test the Python Node class."""

    def test_node_creation(self):
        from horus import Node
        node = Node(name="test_node", tick=lambda n: None)
        assert node.name == "test_node"

    def test_node_auto_name(self):
        from horus import Node
        node = Node(tick=lambda n: None)
        assert node.name.startswith("node_")

    def test_node_with_pubs_and_subs(self):
        from horus import Node
        node = Node(
            name="test_pubsub",
            pubs=["output"],
            subs=["input"],
            tick=lambda n: None,
        )
        assert "output" in node.pub_topics
        assert "input" in node.sub_topics

    def test_node_rate(self):
        from horus import Node
        node = Node(name="test_rate", tick=lambda n: None, rate=100)
        assert node.rate == 100

    def test_node_default_rate(self):
        from horus import Node
        node = Node(name="test_default_rate", tick=lambda n: None)
        assert node.rate == 30  # Default rate

    def test_node_with_dict_pubs(self):
        from horus import Node
        node = Node(
            name="test_dict_pubs",
            pubs={"control/cmd": {"capacity": 2048}},
            tick=lambda n: None,
        )
        assert "control/cmd" in node.pub_topics

    def test_node_with_string_pub(self):
        from horus import Node
        node = Node(
            name="test_str_pub",
            pubs="single_topic",
            tick=lambda n: None,
        )
        assert node.pub_topics == ["single_topic"]


# ============================================================================
# SchedulerConfig
# ============================================================================

class TestSchedulerConfig:
    """Test SchedulerConfig preset methods."""

    def test_standard_config(self):
        from horus._horus import PySchedulerConfig as SchedulerConfig
        config = SchedulerConfig.standard()
        assert config is not None

    def test_safety_critical_config(self):
        from horus._horus import PySchedulerConfig as SchedulerConfig
        config = SchedulerConfig.safety_critical()
        assert config is not None

    def test_high_performance_config(self):
        from horus._horus import PySchedulerConfig as SchedulerConfig
        config = SchedulerConfig.high_performance()
        assert config is not None
