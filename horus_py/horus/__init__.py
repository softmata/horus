"""
HORUS Python - Simple, Intuitive Robotics Framework

A user-friendly Python API for the HORUS robotics framework.

Quick start::

    import horus

    # Create an image backed by shared memory
    img = horus.Image(480, 640, "rgb8")

    # Send over a topic (zero-copy)
    topic = horus.Topic("camera/rgb")
    topic.send(img)

    # Receive (in another process)
    received = topic.recv()
    arr = received.to_numpy()   # zero-copy NumPy array

Domain types: ``Image``, ``PointCloud``, ``DepthImage``
Communication: ``Topic``, ``Node``, ``Scheduler``
"""

# Extend namespace package path to include horus.library
__path__ = __import__('pkgutil').extend_path(__path__, __name__)

from typing import Optional, Any, Dict, List, Callable, Union
from collections import defaultdict
import enum
import time

# Maximum size for logged data representation (to prevent buffer overflows)
MAX_LOG_DATA_SIZE = 200

# Import the Rust extension module
try:
    from horus._horus import (
        Node as _PyNode,
        NodeInfo as _NodeInfo,
        Topic,  # Unified communication API
        Scheduler as _PyScheduler,
        NodeState as _RustNodeState,
        Priority as _RustPriority,
        SchedulerConfig as _SchedulerConfig,
        Miss,
        get_version,
        # GPU utility functions
        cuda_is_available,
        cuda_device_count,
        # Domain types — clean API hiding DLPack/TensorPool internals
        Image,
        PointCloud,
        DepthImage,
        # Native message classes (registered from Rust)
        CmdVel as _RustCmdVel,
        Pose2D as _RustPose2D,
        Imu as _RustImu,
        Odometry as _RustOdometry,
        LaserScan as _RustLaserScan,
        Pose3D as _RustPose3D,
        JointState as _RustJointState,
        Clock as _RustClock,
        TimeReference as _RustTimeReference,
        # Geometry types
        Twist as _RustTwist,
        Vector3 as _RustVector3,
        Point3 as _RustPoint3,
        Quaternion as _RustQuaternion,
        TransformStamped as _RustTransformStamped,
        PoseStamped as _RustPoseStamped,
        PoseWithCovariance as _RustPoseWithCovariance,
        TwistWithCovariance as _RustTwistWithCovariance,
        Accel as _RustAccel,
        AccelStamped as _RustAccelStamped,
        # Control types
        MotorCommand as _RustMotorCommand,
        ServoCommand as _RustServoCommand,
        DifferentialDriveCommand as _RustDifferentialDriveCommand,
        PidConfig as _RustPidConfig,
        TrajectoryPoint as _RustTrajectoryPoint,
        JointCommand as _RustJointCommand,
        PwmCommand as _RustPwmCommand,
        StepperCommand as _RustStepperCommand,
        # Sensor types
        RangeSensor as _RustRangeSensor,
        BatteryState as _RustBatteryState,
        NavSatFix as _RustNavSatFix,
        MagneticField as _RustMagneticField,
        Temperature as _RustTemperature,
        FluidPressure as _RustFluidPressure,
        Illuminance as _RustIlluminance,
        # Diagnostics types
        Heartbeat as _RustHeartbeat,
        DiagnosticStatus as _RustDiagnosticStatus,
        EmergencyStop as _RustEmergencyStop,
        ResourceUsage as _RustResourceUsage,
        # Force types
        WrenchStamped as _RustWrenchStamped,
        ForceCommand as _RustForceCommand,
        ContactInfo as _RustContactInfo,
        # Navigation types
        NavGoal as _RustNavGoal,
        GoalResult as _RustGoalResult,
        PathPlan as _RustPathPlan,
        # Input types
        JoystickInput as _RustJoystickInput,
        KeyboardInput as _RustKeyboardInput,
        # Detection/Perception types
        BoundingBox2D as _RustBoundingBox2D,
        BoundingBox3D as _RustBoundingBox3D,
        Detection as _RustDetection,
        Detection3D as _RustDetection3D,
        SegmentationMask as _RustSegmentationMask,
        # Tracking types
        TrackedObject as _RustTrackedObject,
        TrackingHeader as _RustTrackingHeader,
        # Landmark types
        Landmark as _RustLandmark,
        Landmark3D as _RustLandmark3D,
        LandmarkArray as _RustLandmarkArray,
        # Perception helper types
        PointField as _RustPointField,
        PlaneDetection as _RustPlaneDetection,
        PlaneArray as _RustPlaneArray,
        # ML types
        TensorData as _RustTensorData,
        Predictions as _RustPredictions,
        InferenceMetrics as _RustInferenceMetrics,
        ModelInfo as _RustModelInfo,
        FeatureVector as _RustFeatureVector,
        Classification as _RustClassification,
        ChatMessage as _RustChatMessage,
        LLMRequest as _RustLLMRequest,
        LLMResponse as _RustLLMResponse,
        TrainingMetrics as _RustTrainingMetrics,
        MlTrajectoryPoint as _RustMlTrajectoryPoint,
        DeploymentConfig as _RustDeploymentConfig,
        # Vision types
        CompressedImage as _RustCompressedImage,
        CameraInfo as _RustCameraInfo,
        RegionOfInterest as _RustRegionOfInterest,
        StereoInfo as _RustStereoInfo,
        # Force types (additional)
        TactileArray as _RustTactileArray,
        ImpedanceParameters as _RustImpedanceParameters,
        HapticFeedback as _RustHapticFeedback,
        # Diagnostics types (additional)
        DiagnosticValue as _RustDiagnosticValue,
        DiagnosticReport as _RustDiagnosticReport,
        NodeHeartbeat as _RustNodeHeartbeat,
        SafetyStatus as _RustSafetyStatus,
        # Navigation types (additional)
        Waypoint as _RustWaypoint,
        NavPath as _RustNavPath,
        VelocityObstacle as _RustVelocityObstacle,
        VelocityObstacles as _RustVelocityObstacles,
        OccupancyGrid as _RustOccupancyGrid,
        CostMap as _RustCostMap,
        # Structured error types
        HorusNotFoundError,
        HorusTransformError,
        HorusTimeoutError,
        # Coordinate transforms
        TransformFrame,
        Transform,
        TransformFrameConfig,
    )
except ImportError:
    # Fallback for testing without Rust bindings
    import warnings
    warnings.warn(
        "horus Rust extension not available — running in mock mode. "
        "Most functionality will not work. Install with: maturin develop",
        RuntimeWarning,
        stacklevel=2,
    )
    _PyNode = None
    _NodeInfo = None
    Topic = None  # Unified communication API
    _PyScheduler = None

    _RustNodeState = None
    _RustPriority = None

    # Mock _SchedulerConfig for testing
    class _SchedulerConfig:
        def __init__(self):
            self.tick_rate = 60.0
            self.budget_enforcement = False
            self.deadline_monitoring = False
            self.memory_locking = False
            self.rt_scheduling_class = False
            self.profiling = False
            self.black_box_enabled = False
            self.black_box_size_mb = 0
            self.budget_enforcement = False
            self.deadline_monitoring = False
            self.watchdog_enabled = False
            self.watchdog_timeout_ms = 0
            self.safety_monitor = False
            self.max_deadline_misses = 0
            self.cpu_cores = None
            self.telemetry_endpoint = None
            self.recording_enabled = False
        @staticmethod
        def minimal(): return _SchedulerConfig()

    def get_version(): return "0.1.0-mock"

    cuda_is_available = lambda: False
    cuda_device_count = lambda: 0

    class Image:
        pass

    class PointCloud:
        pass

    class DepthImage:
        pass

    class HorusNotFoundError(Exception):
        """Raised when a topic, frame, or node is not found."""

    class HorusTransformError(Exception):
        """Raised when a coordinate transform fails."""

    class HorusTimeoutError(Exception):
        """Raised when a blocking operation times out."""

    _RustPriority = None


# ── Python enums wrapping Rust constants ─────────────────────────────────────

class NodeState(str, enum.Enum):
    """Node lifecycle state (StrEnum for pattern matching and comparison).

    Example:
        if node.state == NodeState.RUNNING:
            ...
        match node.state:
            case NodeState.RUNNING: ...
            case NodeState.ERROR: ...
    """
    UNINITIALIZED = "uninitialized"
    INITIALIZING = "initializing"
    RUNNING = "running"
    STOPPING = "stopping"
    STOPPED = "stopped"
    ERROR = "error"
    CRASHED = "crashed"


class Priority(enum.IntEnum):
    """Node priority levels (IntEnum — lower value = higher priority).

    Example:
        if node_priority < Priority.NORMAL:
            print("high priority node")

        # Compare directly with ints
        assert Priority.CRITICAL == 0
        assert Priority.HIGH < Priority.NORMAL
    """
    CRITICAL = 0
    HIGH = 10
    NORMAL = 50
    LOW = 80
    BACKGROUND = 100

    @classmethod
    def from_string(cls, s: str) -> 'Priority':
        """Parse a priority from string name or numeric string.

        Args:
            s: Priority name ("critical", "high", "normal", "low", "background")
               or numeric string ("42").

        Raises:
            ValueError: If the string is not a valid priority name or number.
        """
        try:
            return cls[s.upper()]
        except KeyError:
            pass
        try:
            return cls(int(s))
        except (ValueError, KeyError):
            raise ValueError(
                f"Invalid priority: {s!r}. "
                f"Valid names: {', '.join(m.name.lower() for m in cls)}"
            )

    def to_string(self) -> str:
        """Convert priority to lowercase string name."""
        return self.name.lower()


# Single source of truth: Cargo.toml via env!("CARGO_PKG_VERSION")
try:
    __version__ = get_version().split("v", 1)[-1]
except Exception:
    __version__ = "0.1.9"


def _truncate_for_logging(data: Any, max_size: int = MAX_LOG_DATA_SIZE) -> str:
    """
    Safely convert data to string for logging with size limit.

    Args:
        data: Data to convert to string
        max_size: Maximum string length

    Returns:
        Truncated string representation
    """
    if isinstance(data, (dict, list)):
        data_str = str(data)
    else:
        data_str = repr(data)

    if len(data_str) > max_size:
        # Truncate and add indicator
        return data_str[:max_size-3] + "..."

    return data_str


class Node:
    """
    Simple node for HORUS - no inheritance required!

    Example (basic):
        def process(node: Node) -> None:
            if node.has_msg("input"):
                data = node.get("input")
                node.send("output", data * 2)

        node = Node(
            name="processor",
            subs=["input"],
            pubs=["output"],
            tick=process,
            rate=30
        )

        run(node)

    Example (typed hubs for proper logging):
        from horus import Node, Pose2D, CmdVel

        def controller(node: Node) -> None:
            if node.has_msg("localization/pose"):
                pose = node.get("localization/pose")
                # Logs will show: Pose2D { x: 2.31, y: 1.31, ... }
                node.send("control/cmd", {"linear": 1.0, "angular": 0.5})

        node = Node(
            name="controller",
            subs={"localization/pose": {"type": Pose2D}},
            pubs={"control/cmd": {"type": CmdVel}},
            tick=controller,
            rate=30
        )

        run(node)
    """

    def __init__(self,
                 name: Optional[str] = None,
                 pubs: Optional[Union[List[str], str, Dict[str, Dict]]] = None,
                 subs: Optional[Union[List[str], str, Dict[str, Dict]]] = None,
                 tick: Optional[Callable[['Node'], None]] = None,
                 rate: float = 30,
                 init: Optional[Callable[['Node'], None]] = None,
                 shutdown: Optional[Callable[['Node'], None]] = None,
                 on_error: Optional[Callable[['Node', Exception], None]] = None,
                 default_capacity: int = 1024):
        """
        Create a simple HORUS node.

        Args:
            name: Node name (auto-generated if None)
            pubs: Topics to publish to. Can be:
                  - str: single topic (generic)
                  - list: ["topic1", "topic2"] (generic topics)
                  - dict: {"topic1": {"type": CmdVel, "capacity": 2048}, "topic2": {}}
                    Use 'type' to create typed topics for proper message logging
            subs: Topics to subscribe to (same format as pubs)
            tick: Function to call on each tick - signature: tick(node)
            rate: Tick rate in Hz (default 30)
            init: Optional init function - signature: init(node)
            shutdown: Optional shutdown function - signature: shutdown(node)
            on_error: Optional error handler - signature: on_error(node, exception)
            default_capacity: Default topic capacity (default: 1024)

        Example with typed topics (recommended for proper logging):
            from horus import Node, Pose2D, CmdVel

            node = Node(
                name="controller",
                subs={"localization/pose": {"type": Pose2D}},
                pubs={"control/cmd": {"type": CmdVel, "capacity": 2048}},
                tick=lambda n: None
            )

        Example with generic topics (shows <bytes> in logs):
            node = Node(
                name="controller",
                subs=["localization/pose"],  # Generic topic
                pubs=["control/cmd"],
                tick=lambda n: None
            )
        """
        # Auto-generate name if not provided
        if name is None:
            import uuid
            name = f"node_{uuid.uuid4().hex[:8]}"

        self.name = name
        self.tick_fn = tick
        self.init_fn = init
        self.shutdown_fn = shutdown
        self.on_error_fn = on_error
        self.rate = rate
        self.error_count = 0
        self.default_capacity = default_capacity

        # Per-node rate control
        self._last_tick_time = 0.0
        self._tick_period = 1.0 / rate if rate > 0 else 0.0

        # Normalize pub/sub to lists and extract configs
        self.pub_topics = []
        self.sub_topics = []
        self._topic_configs = {}  # topic -> config dict

        # Process pubs
        if isinstance(pubs, str):
            self.pub_topics = [pubs]
        elif isinstance(pubs, dict):
            for topic, config in pubs.items():
                self.pub_topics.append(topic)
                self._topic_configs[topic] = config or {}
        elif isinstance(pubs, list):
            self.pub_topics = pubs

        # Process subs
        if isinstance(subs, str):
            self.sub_topics = [subs]
        elif isinstance(subs, dict):
            for topic, config in subs.items():
                self.sub_topics.append(topic)
                self._topic_configs[topic] = config or {}
        elif isinstance(subs, list):
            self.sub_topics = subs

        # Message queues for subscriptions
        self._msg_queues = defaultdict(list)

        # Phase 2: Message timestamps (topic -> [(msg, timestamp), ...])
        self._msg_timestamps = defaultdict(list)

        # NodeInfo context (set by scheduler)
        self.info = None

        # Create underlying HORUS components if available
        if _PyNode:
            self._rust_available = True
            self._setup_topics()
        else:
            # Mock mode for testing
            self._rust_available = False
            self._topics = {}

    def _setup_topics(self):
        """Setup publish/subscribe hubs with configured capacities and types."""
        self._topics = {}

        # Create publisher hubs
        for topic in self.pub_topics:
            config = self._topic_configs.get(topic, {})
            capacity = config.get('capacity', self.default_capacity)
            msg_type = config.get('type', None)

            # If type specified, create typed topic; otherwise generic topic
            if msg_type is not None:
                # Temporarily set __topic_name__ so Rust Topic uses correct name
                original_topic = getattr(msg_type, '__topic_name__', None)
                msg_type.__topic_name__ = topic
                try:
                    self._topics[topic] = Topic(msg_type, capacity)
                finally:
                    # Restore original or delete
                    if original_topic is not None:
                        msg_type.__topic_name__ = original_topic
                    elif hasattr(msg_type, '__topic_name__'):
                        delattr(msg_type, '__topic_name__')
            else:
                self._topics[topic] = Topic(topic, capacity)

        # Create subscriber hubs
        for topic in self.sub_topics:
            config = self._topic_configs.get(topic, {})
            capacity = config.get('capacity', self.default_capacity)
            msg_type = config.get('type', None)

            # If type specified, create typed topic; otherwise generic topic
            if msg_type is not None:
                # Temporarily set __topic_name__ so Rust Topic uses correct name
                original_topic = getattr(msg_type, '__topic_name__', None)
                msg_type.__topic_name__ = topic
                try:
                    self._topics[topic] = Topic(msg_type, capacity)
                finally:
                    # Restore original or delete
                    if original_topic is not None:
                        msg_type.__topic_name__ = original_topic
                    elif hasattr(msg_type, '__topic_name__'):
                        delattr(msg_type, '__topic_name__')
            else:
                self._topics[topic] = Topic(topic, capacity)

    def has_msg(self, topic: str) -> bool:
        """
        Check if messages are available on a topic.

        Args:
            topic: Topic to check

        Returns:
            True if messages available
        """
        # First try to receive new messages
        self._receive_messages(topic)
        return len(self._msg_queues[topic]) > 0

    def get(self, topic: str) -> Optional[Any]:
        """
        Get next message from topic.

        Args:
            topic: Topic to read from

        Returns:
            Message data or None if no messages
        """
        self._receive_messages(topic)

        if self._msg_queues[topic]:
            # Phase 2: Pop timestamp along with message
            if self._msg_timestamps[topic]:
                self._msg_timestamps[topic].pop(0)
            return self._msg_queues[topic].pop(0)
        return None

    def get_all(self, topic: str) -> List[Any]:
        """
        Get all available messages from topic.

        Args:
            topic: Topic to read from

        Returns:
            List of messages (empty if none)
        """
        self._receive_messages(topic)

        msgs = self._msg_queues[topic][:]
        self._msg_queues[topic].clear()
        # Phase 2: Clear timestamps too
        self._msg_timestamps[topic].clear()
        return msgs

    def get_timestamp(self, topic: str) -> Optional[float]:
        """
        Get timestamp of the next message without consuming it (Phase 2).

        Args:
            topic: Topic to check

        Returns:
            Unix timestamp in seconds (with microsecond precision) or None
        """
        self._receive_messages(topic)

        if self._msg_timestamps[topic]:
            return self._msg_timestamps[topic][0]
        return None

    def get_message_age(self, topic: str) -> Optional[float]:
        """
        Get age of the next message in seconds (Phase 2).

        Args:
            topic: Topic to check

        Returns:
            Message age in seconds or None if no messages
        """
        timestamp = self.get_timestamp(topic)
        if timestamp is not None and timestamp > 0:
            import time
            return time.time() - timestamp
        return None

    def is_stale(self, topic: str, max_age: float) -> bool:
        """
        Check if the next message is stale (Phase 2).

        Args:
            topic: Topic to check
            max_age: Maximum acceptable age in seconds

        Returns:
            True if message is older than max_age, False otherwise
        """
        age = self.get_message_age(topic)
        if age is None:
            return False  # No message = not stale
        return age > max_age

    def get_with_timestamp(self, topic: str) -> Optional[tuple]:
        """
        Get next message with its timestamp (Phase 2).

        Args:
            topic: Topic to read from

        Returns:
            Tuple of (message, timestamp) or None if no messages
        """
        self._receive_messages(topic)

        if self._msg_queues[topic]:
            msg = self._msg_queues[topic].pop(0)
            timestamp = self._msg_timestamps[topic].pop(0) if self._msg_timestamps[topic] else 0.0
            return (msg, timestamp)
        return None

    def send(self, topic: str, data: Any) -> bool:
        """
        Send data to a topic.

        Args:
            topic: Topic to send to
            data: Data to send (supports numpy arrays with zero-copy)

        Returns:
            True if sent successfully
        """
        # Auto-detect topics: add topic if not already declared
        if topic not in self.pub_topics:
            self.pub_topics.append(topic)
            if self._rust_available:
                config = self._topic_configs.get(topic, {})
                capacity = config.get('capacity', self.default_capacity)
                msg_type = config.get('type', None)

                # If type specified, create typed topic; otherwise generic topic
                if msg_type is not None:
                    # Temporarily set __topic_name__ so Rust Topic uses correct name
                    original_topic = getattr(msg_type, '__topic_name__', None)
                    msg_type.__topic_name__ = topic
                    try:
                        self._topics[topic] = Topic(msg_type, capacity)
                    finally:
                        # Restore original or delete
                        if original_topic is not None:
                            msg_type.__topic_name__ = original_topic
                        elif hasattr(msg_type, '__topic_name__'):
                            delattr(msg_type, '__topic_name__')
                else:
                    self._topics[topic] = Topic(topic, capacity)

        if self._rust_available and topic in self._topics:
            t = self._topics[topic]

            # Measure IPC timing
            import time
            start_ns = time.perf_counter_ns()

            # PyTopic.send() handles all serialization internally in Rust
            result = t.send(data, self)

            end_ns = time.perf_counter_ns()
            ipc_ns = end_ns - start_ns

            # Log the publish operation if NodeInfo available
            if self.info:
                data_repr = _truncate_for_logging(data)
                self.info.log_pub(topic, data_repr, ipc_ns)

            return result

        # Mock mode
        return True

    def _receive_messages(self, topic: str):
        """Pull messages from topic into queue."""
        # Auto-detect topics: add topic if not already declared
        if topic not in self.sub_topics:
            self.sub_topics.append(topic)
            if self._rust_available:
                config = self._topic_configs.get(topic, {})
                capacity = config.get('capacity', self.default_capacity)
                msg_type = config.get('type', None)

                # If type specified, create typed topic; otherwise generic topic
                if msg_type is not None:
                    # Temporarily set __topic_name__ so Rust Topic uses correct name
                    original_topic = getattr(msg_type, '__topic_name__', None)
                    msg_type.__topic_name__ = topic
                    try:
                        self._topics[topic] = Topic(msg_type, capacity)
                    finally:
                        # Restore original or delete
                        if original_topic is not None:
                            msg_type.__topic_name__ = original_topic
                        elif hasattr(msg_type, '__topic_name__'):
                            delattr(msg_type, '__topic_name__')
                else:
                    self._topics[topic] = Topic(topic, capacity)

        if self._rust_available and topic in self._topics:
            t = self._topics[topic]
            import time

            # Receive all available messages
            while True:
                # Measure IPC timing
                start_ns = time.perf_counter_ns()

                # PyTopic.recv() handles all deserialization internally in Rust
                msg = t.recv(self)
                end_ns = time.perf_counter_ns()

                if msg is None:
                    break

                ipc_ns = end_ns - start_ns
                timestamp = time.time()

                # Log the subscribe operation if NodeInfo available
                if self.info:
                    data_repr = _truncate_for_logging(msg)
                    self.info.log_sub(topic, data_repr, ipc_ns)

                # Store message with timestamp
                self._msg_queues[topic].append(msg)
                self._msg_timestamps[topic].append(timestamp)

    def _run_tick_with_error_handling(self, info: Optional[Any] = None) -> None:
        """Run tick_fn with error handling and info context management."""
        old_info = self.info
        self.info = info
        try:
            if self.tick_fn:
                self.tick_fn(self)
        except Exception as e:
            self.error_count += 1
            if self.info:
                self.info.log_error(f"Tick failed: {e}")
                if self.error_count > 10:
                    self.info.transition_to_error(f"Too many errors ({self.error_count})")
            if self.on_error_fn:
                try:
                    self.on_error_fn(self, e)
                except Exception as handler_error:
                    if self.info:
                        self.info.log_error(f"Error handler failed: {handler_error}")
            else:
                raise
        finally:
            self.info = old_info

    def _internal_tick(self, info: Optional[Any] = None) -> None:
        """Internal tick called by scheduler with per-node rate control."""
        import time

        # Check if enough time has elapsed for this node's rate
        current_time = time.time()
        if self._tick_period > 0:
            time_since_last_tick = current_time - self._last_tick_time
            if time_since_last_tick < self._tick_period:
                return

        self._last_tick_time = current_time
        self._run_tick_with_error_handling(info)

    def _internal_init(self, info: Optional[Any] = None) -> None:
        """Internal init called by scheduler."""
        self.info = info  # Store info for access in init function
        if self.init_fn:
            self.init_fn(self)

    def _internal_shutdown(self, info: Optional[Any] = None) -> None:
        """Internal shutdown called by scheduler."""
        self.info = info  # Store info for access in shutdown function
        if self.shutdown_fn:
            self.shutdown_fn(self)

    # Public methods for Rust bindings to call
    def init(self, info: Optional[Any] = None) -> None:
        """Called by Rust scheduler during initialization."""
        self._internal_init(info)

    def tick(self, info: Optional[Any] = None) -> None:
        """Called by Rust scheduler on each tick.
        Bypasses _internal_tick rate control since horus_core handles rate limiting.
        """
        self._run_tick_with_error_handling(info)

    def shutdown(self, info: Optional[Any] = None) -> None:
        """Called by Rust scheduler during shutdown."""
        self._internal_shutdown(info)

    # NodeInfo convenience methods (delegate to info if available)
    def log_info(self, message: str) -> None:
        """Log an info message (if logging enabled)."""
        if self.info:
            self.info.log_info(message)

    def log_warning(self, message: str) -> None:
        """Log a warning message (if logging enabled)."""
        if self.info:
            self.info.log_warning(message)

    def log_error(self, message: str) -> None:
        """Log an error message (if logging enabled)."""
        if self.info:
            self.info.log_error(message)

    def log_debug(self, message: str) -> None:
        """Log a debug message (if logging enabled)."""
        if self.info:
            self.info.log_debug(message)

    def request_stop(self) -> None:
        """
        Request the scheduler to stop execution.

        This is a convenience method that can be called from within
        a node's tick callback to stop the scheduler.

        Example:
            def tick(node):
                if node.info.tick_count() >= 10:
                    node.request_stop()  # Stop after 10 ticks
        """
        if self.info:
            self.info.request_stop()

    # Topic introspection methods
    def publishers(self) -> List[str]:
        """
        Get list of topics this node publishes to.

        Returns:
            List of publisher topic names

        Example:
            topics = node.publishers()
            print(f"Publishing to: {topics}")
        """
        return self.pub_topics.copy()

    def subscribers(self) -> List[str]:
        """
        Get list of topics this node subscribes to.

        Returns:
            List of subscriber topic names

        Example:
            topics = node.subscribers()
            print(f"Subscribed to: {topics}")
        """
        return self.sub_topics.copy()


class Scheduler:
    """
    Scheduler for running HORUS nodes.

    Example (kwargs):
        scheduler = Scheduler(tick_rate=500.0, fault_tolerance=True, rt=True)

    Example (adding nodes):
        scheduler = Scheduler()
        scheduler.add(sensor_node, order=0, rate=100.0)
        scheduler.add(motor_node, order=1, rt=True, deadline=5.0)
        scheduler.run()

    Example (context manager — auto-stop on exit):
        with Scheduler(tick_rate=100.0) as sched:
            sched.add(sensor_node, order=0, rate=100.0)
            sched.run_for(10.0)
    """

    def __init__(self, *,
                 tick_rate: float = 1000.0,
                 fault_tolerance: bool = False,
                 parallel: bool = False,
                 rt: bool = False,
                 profiling: bool = False,
                 blackbox_mb: int = 0,
                 watchdog_ms: int = 0,
                 safety_monitor: bool = False,
                 recording: bool = False,
                 _inner=None):
        """
        Create a scheduler.

        Args:
            tick_rate: Global tick rate in Hz (default: 1000.0)
            fault_tolerance: Enable tier-based fault tolerance (default: False)
            parallel: Use parallel execution mode (default: False)
            rt: Enable real-time features (memory locking + RT scheduling class)
            profiling: Enable runtime profiling (default: False)
            blackbox_mb: Black box buffer size in MB (0 = disabled)
            watchdog_ms: Watchdog timeout in ms (0 = disabled)
            safety_monitor: Enable safety monitor (default: False)
            recording: Enable recording (default: False)
            _inner: Internal — used by preset constructors
        """
        if _inner is not None:
            self._scheduler = _inner
        elif _PyScheduler:
            cfg = _SchedulerConfig.minimal()
            cfg.tick_rate = tick_rate
            if fault_tolerance:
                cfg.budget_enforcement = True
                cfg.deadline_monitoring = True
            if rt:
                cfg.memory_locking = True
                cfg.rt_scheduling_class = True
            cfg.profiling = profiling
            if blackbox_mb > 0:
                cfg.black_box_enabled = True
                cfg.black_box_size_mb = blackbox_mb
            if watchdog_ms > 0:
                cfg.watchdog_enabled = True
                cfg.watchdog_timeout_ms = watchdog_ms
            cfg.safety_monitor = safety_monitor
            cfg.recording_enabled = recording
            self._scheduler = _PyScheduler(cfg)
        else:
            self._scheduler = None
        self._nodes = []

    # Presets deploy(), hard_rt(), safety_critical() removed.
    # Use Scheduler(safety_monitor=True, watchdog_ms=500) or
    # Scheduler(config=SchedulerConfig.with_monitoring()) instead.

    def add(self, node: 'Node', order: int = 100, rate: Optional[float] = None,
            rt: bool = False, deadline: Optional[float] = None,
            budget: Optional[int] = None,
            failure_policy: Optional[str] = None,
            on_miss: Optional[str] = None) -> 'Scheduler':
        """
        Add a node to the scheduler.

        Args:
            node: Node instance to add
            order: Execution order (lower = earlier, default: 100)
            rate: Node-specific tick rate in Hz (default: uses node.rate)
            rt: Mark as real-time node (default: False)
            deadline: Soft deadline in milliseconds (default: None)
            budget: Tick budget in microseconds (default: None)
            failure_policy: Failure policy - "fatal", "restart", "skip", "ignore"
            on_miss: Deadline miss policy - Miss.WARN, Miss.SKIP, Miss.SAFE_MODE, Miss.STOP
                  (default: None = warn)

        Returns:
            self (for method chaining)

        Example:
            scheduler.add(sensor_node, order=0, rate=1000.0)
            scheduler.add(motor_node, order=1, rt=True, deadline=5.0, on_miss=Miss.SAFE_MODE)
            scheduler.add(logger_node, order=100, failure_policy="skip")
        """
        self._nodes.append(node)

        if self._scheduler:
            # Use node.rate if rate not specified
            actual_rate = rate if rate is not None else node.rate
            self._scheduler.add(node, order, actual_rate, rt, deadline,
                                budget, failure_policy, on_miss)

        return self

    def node(self, node: 'Node') -> 'NodeBuilder':
        """Start building a node configuration (fluent API).

        Example:
            scheduler.node(detector).on("lidar_scan").order(0).build()
            scheduler.node(motor).rate(500).budget(200).failure_policy("restart", max_retries=10).build()
        """
        self._nodes.append(node)
        if self._scheduler:
            return self._scheduler.node(node)
        raise RuntimeError("Fluent node builder requires Rust scheduler (not available in mock mode)")

    def run(self, duration: Optional[float] = None) -> None:
        """
        Run the scheduler.

        Args:
            duration: Optional duration in seconds (runs forever if None)
        """
        if self._scheduler:
            # horus_core handles init/tick/shutdown via PyNodeAdapter
            try:
                if duration:
                    self._scheduler.run_for(duration)
                else:
                    self._scheduler.run()
            except KeyboardInterrupt:
                print("\nCtrl+C received, shutting down gracefully...")
                self._scheduler.stop()
        else:
            # Mock mode - simple loop
            print(f"Running {len(self._nodes)} nodes in mock mode...")

            for node in self._nodes:
                node._internal_init()

            try:
                start = time.time()
                while duration is None or (time.time() - start) < duration:
                    for node in self._nodes:
                        node._internal_tick()
                    time.sleep(0.03)  # ~30Hz
            except KeyboardInterrupt:
                print("\nCtrl+C received, shutting down gracefully...")
            finally:
                for node in self._nodes:
                    node._internal_shutdown()

    def stop(self) -> None:
        """Stop the scheduler."""
        if self._scheduler:
            self._scheduler.stop()

    def __enter__(self) -> 'Scheduler':
        """Context manager entry — returns self for `with` usage.

        Example:
            with Scheduler(tick_rate=100.0) as sched:
                sched.add(sensor_node, order=0)
                sched.run()
            # stop() called automatically on exit
        """
        return self

    def __exit__(self, exc_type, exc_val, exc_tb) -> None:
        """Context manager exit — stops the scheduler."""
        self.stop()

    def get_node_stats(self, node_name: str) -> Dict[str, Any]:
        """
        Get statistics for a specific node.

        Args:
            node_name: Name of the node

        Returns:
            Dictionary with node stats including:
            - name: Node name
            - priority: Priority level
            - total_ticks: Total number of ticks executed
            - errors_count: Number of errors encountered

        Example:
            stats = scheduler.get_node_stats("sensor")
            print(f"Node {stats['name']} at priority {stats['priority']}")
            print(f"Total ticks: {stats['total_ticks']}")
        """
        if self._scheduler:
            return self._scheduler.get_node_stats(node_name)
        return {}

    def set_node_rate(self, node_name: str, rate: float) -> None:
        """
        Set the execution rate for a specific node.

        Args:
            node_name: Name of the node
            rate: New rate in Hz (ticks per second)

        Example:
            scheduler.set_node_rate("sensor", 100.0)  # Change to 100Hz
        """
        # Update the node's rate attribute
        for node in self._nodes:
            if node.name == node_name:
                node.rate = rate
                node._tick_period = 1.0 / rate if rate > 0 else 0.0
                break

        # Update Rust scheduler if available
        if self._scheduler:
            self._scheduler.set_node_rate(node_name, rate)

    def get_all_nodes(self) -> List[Dict[str, Any]]:
        """
        Get information about all registered nodes.

        Returns:
            List of dictionaries containing node information:
            - name: Node name
            - order: Execution order
            - rate: Node execution rate
            - total_ticks: Total number of ticks executed
            - successful_ticks: Number of successful ticks
            - failed_ticks: Number of failed ticks
            - errors_count: Number of errors encountered
            - avg_tick_duration_ms: Average tick duration
            - min_tick_duration_ms: Minimum tick duration
            - max_tick_duration_ms: Maximum tick duration
            - last_tick_duration_ms: Last tick duration
            - uptime_seconds: Node uptime

        Example:
            nodes = scheduler.get_all_nodes()
            for node in nodes:
                print(f"{node['name']}: {node['total_ticks']} ticks, {node['errors_count']} errors")
        """
        if self._scheduler:
            return self._scheduler.get_all_nodes()
        return []

    def get_node_count(self) -> int:
        """
        Get the number of registered nodes.

        Returns:
            Number of nodes in the scheduler

        Example:
            count = scheduler.get_node_count()
            print(f"Running {count} nodes")
        """
        if self._scheduler:
            return self._scheduler.get_node_count()
        return len(self._nodes)

    def has_node(self, name: str) -> bool:
        """
        Check if a node with the given name exists.

        Args:
            name: Node name to check

        Returns:
            True if node exists, False otherwise

        Example:
            if scheduler.has_node("sensor"):
                stats = scheduler.get_node_stats("sensor")
        """
        if self._scheduler:
            return self._scheduler.has_node(name)
        return any(node.name == name for node in self._nodes)

    def get_node_names(self) -> List[str]:
        """
        Get list of all node names.

        Returns:
            List of node names

        Example:
            names = scheduler.get_node_names()
            print(f"Nodes: {', '.join(names)}")
        """
        if self._scheduler:
            return self._scheduler.get_node_names()
        return [node.name for node in self._nodes]

    # ========================================================================
    # Status & Diagnostics
    # ========================================================================

    def status(self) -> str:
        """
        Get scheduler status string.

        Returns:
            Status string (e.g. "idle", "running", "stopped")
        """
        if self._scheduler:
            return self._scheduler.status()
        return "mock"

    def capabilities(self) -> Optional[Dict[str, Any]]:
        """
        Get runtime RT capabilities as a dict.

        Returns:
            Dict with keys: preempt_rt, rt_priority_available,
            max_rt_priority, min_rt_priority. None if not available.
        """
        if self._scheduler:
            return self._scheduler.capabilities()
        return None

    def has_full_rt(self) -> bool:
        """
        Check if full real-time capabilities are available.

        Returns:
            True if RT scheduling, memory locking, etc. are all available
        """
        if self._scheduler:
            return self._scheduler.has_full_rt()
        return False

    def degradations(self) -> List[Dict[str, Any]]:
        """
        Get list of RT degradations (features that couldn't be enabled).

        Returns:
            List of dicts with keys: feature, reason, severity
        """
        if self._scheduler:
            return self._scheduler.degradations()
        return []

    def current_tick(self) -> int:
        """
        Get the current tick count.

        Returns:
            Number of ticks executed so far
        """
        if self._scheduler:
            return self._scheduler.current_tick()
        return 0

    def scheduler_name(self) -> str:
        """
        Get the scheduler name.

        Returns:
            Scheduler name string
        """
        if self._scheduler:
            return self._scheduler.scheduler_name()
        return "MockScheduler"

    # ========================================================================
    # Safety Stats
    # ========================================================================

    def safety_stats(self) -> Optional[Dict[str, Any]]:
        """
        Get safety monitor statistics.

        Returns:
            Dict with keys: state, budget_overruns, deadline_misses,
            watchdog_expirations. None if safety monitor is not enabled.
        """
        if self._scheduler:
            return self._scheduler.safety_stats()
        return None

    # ========================================================================
    # Recording
    # ========================================================================

    def is_recording(self) -> bool:
        """
        Check if the scheduler is currently recording.

        Returns:
            True if recording is active
        """
        if self._scheduler:
            return self._scheduler.is_recording()
        return False

    def is_replaying(self) -> bool:
        """
        Check if the scheduler is currently replaying a recording.

        Returns:
            True if replay mode is active
        """
        if self._scheduler:
            return self._scheduler.is_replaying()
        return False

    def stop_recording(self) -> List[str]:
        """
        Stop recording and return paths to saved recording files.

        Returns:
            List of file paths where recordings were saved
        """
        if self._scheduler:
            return self._scheduler.stop_recording()
        return []

    @staticmethod
    def list_recordings() -> List[str]:
        """
        List available recording files.

        Returns:
            List of recording file paths
        """
        if _PyScheduler:
            return _PyScheduler.list_recordings()
        return []

    # ========================================================================
    # Node Control
    # ========================================================================

    def set_tick_budget(self, node_name: str, us: int) -> None:
        """
        Set budget (Worst-Case Execution Time) budget for a node.

        Args:
            node_name: Name of the node
            us: Budget in microseconds
        """
        if not self._scheduler:
            raise RuntimeError("Cannot set budget budget before scheduler is created")
        self._scheduler.set_tick_budget(node_name, us)

    def add_critical_node(self, node_name: str, timeout_ms: int) -> None:
        """
        Mark a node as critical with a watchdog timeout.

        Args:
            node_name: Name of the node
            timeout_ms: Watchdog timeout in milliseconds
        """
        if not self._scheduler:
            raise RuntimeError("Cannot add critical node before scheduler is created")
        self._scheduler.add_critical_node(node_name, timeout_ms)

    @staticmethod
    def delete_recording(session_name: str) -> None:
        """
        Delete a recording by session name.

        Args:
            session_name: Name of the recording session to delete
        """
        if _PyScheduler:
            _PyScheduler.delete_recording(session_name)


# Convenience functions

def run(*nodes: Node, duration: Optional[float] = None) -> None:
    """
    Quick run helper - create scheduler and run nodes.

    Args:
        *nodes: Node instances to run
        duration: Optional duration in seconds

    Example:
        node = Node(subs="in", pubs="out", tick=lambda n: n.send("out", n.get("in")))
        run(node, duration=5)
    """
    scheduler = Scheduler()

    # Smart priority assignment: subscribers first, then publishers
    # This ensures subscriber hubs are created before first messages are sent
    subscribers = []
    publishers = []
    both = []

    for node in nodes:
        has_subs = len(node.sub_topics) > 0
        has_pubs = len(node.pub_topics) > 0

        if has_subs and has_pubs:
            both.append(node)
        elif has_subs:
            subscribers.append(node)
        elif has_pubs:
            publishers.append(node)
        else:
            # No declared topics - add to both category (will auto-detect)
            both.append(node)

    # Assign order: subscribers (0..N), both (N+1..M), publishers (M+1..P)
    order = 0
    for node in subscribers:
        scheduler.add(node, order=order)
        order += 1
    for node in both:
        scheduler.add(node, order=order)
        order += 1
    for node in publishers:
        scheduler.add(node, order=order)
        order += 1

    scheduler.run(duration)


# Import Python message types from horus_library for cross-language communication
try:
    from horus.library import (
        # Geometry messages
        Pose2D,
        Twist,
        Point3,
        Vector3,
        Quaternion,
        Transform,
        # Control messages
        CmdVel,
        MotorCommand,
        DifferentialDriveCommand,
        ServoCommand,
        PwmCommand,
        StepperCommand,
        PidConfig,
        # Sensor messages
        LaserScan,
        Imu,
        BatteryState,
        NavSatFix,
        Odometry,
        Range,
        # Diagnostics messages
        Status,
        EmergencyStop,
        Heartbeat,
        ResourceUsage,
        # Input messages
        JoystickInput,
        KeyboardInput,
        # I/O messages
        DigitalIO,
        AnalogIO,
    )
    _has_messages = True
except ImportError:
    _has_messages = False
    # horus_library module not available - fall back to Rust message classes if available
    try:
        # Use Rust native message classes as fallback
        CmdVel = _RustCmdVel
        Pose2D = _RustPose2D
        Imu = _RustImu
        Odometry = _RustOdometry
        LaserScan = _RustLaserScan
        Pose3D = _RustPose3D
        JointState = _RustJointState
        Clock = _RustClock
        TimeReference = _RustTimeReference
        Twist = _RustTwist
        Vector3 = _RustVector3
        Point3 = _RustPoint3
        Quaternion = _RustQuaternion
        TransformStamped = _RustTransformStamped
        PoseStamped = _RustPoseStamped
        PoseWithCovariance = _RustPoseWithCovariance
        TwistWithCovariance = _RustTwistWithCovariance
        Accel = _RustAccel
        AccelStamped = _RustAccelStamped
        MotorCommand = _RustMotorCommand
        ServoCommand = _RustServoCommand
        DifferentialDriveCommand = _RustDifferentialDriveCommand
        PidConfig = _RustPidConfig
        TrajectoryPoint = _RustTrajectoryPoint
        JointCommand = _RustJointCommand
        PwmCommand = _RustPwmCommand
        StepperCommand = _RustStepperCommand
        RangeSensor = _RustRangeSensor
        BatteryState = _RustBatteryState
        NavSatFix = _RustNavSatFix
        MagneticField = _RustMagneticField
        Temperature = _RustTemperature
        FluidPressure = _RustFluidPressure
        Illuminance = _RustIlluminance
        Heartbeat = _RustHeartbeat
        DiagnosticStatus = _RustDiagnosticStatus
        EmergencyStop = _RustEmergencyStop
        ResourceUsage = _RustResourceUsage
        WrenchStamped = _RustWrenchStamped
        ForceCommand = _RustForceCommand
        ContactInfo = _RustContactInfo
        NavGoal = _RustNavGoal
        GoalResult = _RustGoalResult
        PathPlan = _RustPathPlan
        JoystickInput = _RustJoystickInput
        KeyboardInput = _RustKeyboardInput
        BoundingBox2D = _RustBoundingBox2D
        BoundingBox3D = _RustBoundingBox3D
        Detection = _RustDetection
        Detection3D = _RustDetection3D
        SegmentationMask = _RustSegmentationMask
        TrackedObject = _RustTrackedObject
        TrackingHeader = _RustTrackingHeader
        Landmark = _RustLandmark
        Landmark3D = _RustLandmark3D
        LandmarkArray = _RustLandmarkArray
        PointField = _RustPointField
        PlaneDetection = _RustPlaneDetection
        PlaneArray = _RustPlaneArray
        TensorData = _RustTensorData
        Predictions = _RustPredictions
        InferenceMetrics = _RustInferenceMetrics
        ModelInfo = _RustModelInfo
        FeatureVector = _RustFeatureVector
        Classification = _RustClassification
        ChatMessage = _RustChatMessage
        LLMRequest = _RustLLMRequest
        LLMResponse = _RustLLMResponse
        TrainingMetrics = _RustTrainingMetrics
        MlTrajectoryPoint = _RustMlTrajectoryPoint
        DeploymentConfig = _RustDeploymentConfig
        CompressedImage = _RustCompressedImage
        CameraInfo = _RustCameraInfo
        RegionOfInterest = _RustRegionOfInterest
        StereoInfo = _RustStereoInfo
        TactileArray = _RustTactileArray
        ImpedanceParameters = _RustImpedanceParameters
        HapticFeedback = _RustHapticFeedback
        DiagnosticValue = _RustDiagnosticValue
        DiagnosticReport = _RustDiagnosticReport
        NodeHeartbeat = _RustNodeHeartbeat
        SafetyStatus = _RustSafetyStatus
        Waypoint = _RustWaypoint
        NavPath = _RustNavPath
        VelocityObstacle = _RustVelocityObstacle
        VelocityObstacles = _RustVelocityObstacles
        OccupancyGrid = _RustOccupancyGrid
        CostMap = _RustCostMap
        _has_messages = True
    except NameError:
        # Neither Python library nor Rust classes available
        pass

__all__ = [
    # Core API
    "Node",
    "Scheduler",
    "NodeState",
    "Priority",
    "Topic",
    "Miss",
    "run",
    # Submodules
    "msggen",
    "ai",
    "perception",
    # Domain types
    "Image",
    "PointCloud",
    "DepthImage",
    # GPU utility functions
    "cuda_is_available",
    "cuda_device_count",
    # Simple async API
    "AsyncNode",
    "AsyncTopic",
    # Coordinate transforms
    "TransformFrame",
    "Transform",
    "TransformFrameConfig",
    # Structured error types
    "HorusNotFoundError",
    "HorusTransformError",
    "HorusTimeoutError",
    # Utility
    "get_version",
]

# Import simple async API
from .async_node import AsyncNode, AsyncTopic

# Import custom message generator module
from . import msggen

# Import AI/ML submodule (horus.ai)
from . import ai

# Always expose Rust-native types that have no horus.library Python equivalent.
# These are available whether or not horus.library is installed.
try:
    Pose3D = _RustPose3D
    JointState = _RustJointState
    Clock = _RustClock
    TimeReference = _RustTimeReference
    TransformStamped = _RustTransformStamped
    PoseStamped = _RustPoseStamped
    PoseWithCovariance = _RustPoseWithCovariance
    TwistWithCovariance = _RustTwistWithCovariance
    Accel = _RustAccel
    AccelStamped = _RustAccelStamped
    TrajectoryPoint = _RustTrajectoryPoint
    JointCommand = _RustJointCommand
    # Sensor types only in Rust
    MagneticField = _RustMagneticField
    Temperature = _RustTemperature
    FluidPressure = _RustFluidPressure
    Illuminance = _RustIlluminance
    RangeSensor = _RustRangeSensor
    # Diagnostics types only in Rust
    DiagnosticStatus = _RustDiagnosticStatus
    # Force/Nav/Input types only in Rust
    WrenchStamped = _RustWrenchStamped
    ForceCommand = _RustForceCommand
    ContactInfo = _RustContactInfo
    NavGoal = _RustNavGoal
    GoalResult = _RustGoalResult
    PathPlan = _RustPathPlan
    # Detection/Perception types only in Rust
    BoundingBox2D = _RustBoundingBox2D
    BoundingBox3D = _RustBoundingBox3D
    Detection = _RustDetection
    Detection3D = _RustDetection3D
    SegmentationMask = _RustSegmentationMask
    TrackedObject = _RustTrackedObject
    TrackingHeader = _RustTrackingHeader
    Landmark = _RustLandmark
    Landmark3D = _RustLandmark3D
    LandmarkArray = _RustLandmarkArray
    # Perception helper types only in Rust
    PointField = _RustPointField
    PlaneDetection = _RustPlaneDetection
    PlaneArray = _RustPlaneArray
    # ML types only in Rust
    TensorData = _RustTensorData
    Predictions = _RustPredictions
    InferenceMetrics = _RustInferenceMetrics
    ModelInfo = _RustModelInfo
    FeatureVector = _RustFeatureVector
    Classification = _RustClassification
    ChatMessage = _RustChatMessage
    LLMRequest = _RustLLMRequest
    LLMResponse = _RustLLMResponse
    TrainingMetrics = _RustTrainingMetrics
    MlTrajectoryPoint = _RustMlTrajectoryPoint
    DeploymentConfig = _RustDeploymentConfig
    # Vision types only in Rust
    CompressedImage = _RustCompressedImage
    CameraInfo = _RustCameraInfo
    RegionOfInterest = _RustRegionOfInterest
    StereoInfo = _RustStereoInfo
    # Force types (additional) only in Rust
    TactileArray = _RustTactileArray
    ImpedanceParameters = _RustImpedanceParameters
    HapticFeedback = _RustHapticFeedback
    # Diagnostics types (additional) only in Rust
    DiagnosticValue = _RustDiagnosticValue
    DiagnosticReport = _RustDiagnosticReport
    NodeHeartbeat = _RustNodeHeartbeat
    SafetyStatus = _RustSafetyStatus
    # Navigation types (additional) only in Rust
    Waypoint = _RustWaypoint
    NavPath = _RustNavPath
    VelocityObstacle = _RustVelocityObstacle
    VelocityObstacles = _RustVelocityObstacles
    OccupancyGrid = _RustOccupancyGrid
    CostMap = _RustCostMap
except NameError:
    pass

# Add message types to __all__ if available
if _has_messages:
    __all__.extend([
        # Geometry messages
        "Pose2D",
        "Twist",
        "Point3",
        "Vector3",
        "Quaternion",
        "TransformStamped",
        "PoseStamped",
        "PoseWithCovariance",
        "TwistWithCovariance",
        "Accel",
        "AccelStamped",
        # Control messages
        "CmdVel",
        "MotorCommand",
        "DifferentialDriveCommand",
        "ServoCommand",
        "PwmCommand",
        "StepperCommand",
        "PidConfig",
        "TrajectoryPoint",
        "JointCommand",
        # Sensor messages
        "LaserScan",
        "Imu",
        "BatteryState",
        "NavSatFix",
        "Odometry",
        "Pose3D",
        "JointState",
        "Clock",
        "TimeReference",
        "RangeSensor",
        "MagneticField",
        "Temperature",
        "FluidPressure",
        "Illuminance",
        # Diagnostics messages
        "EmergencyStop",
        "Heartbeat",
        "ResourceUsage",
        "DiagnosticStatus",
        # Force types
        "WrenchStamped",
        "ForceCommand",
        "ContactInfo",
        # Navigation types
        "NavGoal",
        "GoalResult",
        "PathPlan",
        # Input messages
        "JoystickInput",
        "KeyboardInput",
        # Detection/Perception types
        "BoundingBox2D",
        "BoundingBox3D",
        "Detection",
        "Detection3D",
        "SegmentationMask",
        # Tracking types
        "TrackedObject",
        "TrackingHeader",
        # Landmark types
        "Landmark",
        "Landmark3D",
        "LandmarkArray",
        # Perception helper types
        "PointField",
        "PlaneDetection",
        "PlaneArray",
        # ML types
        "TensorData",
        "Predictions",
        "InferenceMetrics",
        "ModelInfo",
        "FeatureVector",
        "Classification",
        "ChatMessage",
        "LLMRequest",
        "LLMResponse",
        "TrainingMetrics",
        "MlTrajectoryPoint",
        "DeploymentConfig",
        # Vision types
        "CompressedImage",
        "CameraInfo",
        "RegionOfInterest",
        "StereoInfo",
        # Force types (additional)
        "TactileArray",
        "ImpedanceParameters",
        "HapticFeedback",
        # Diagnostics types (additional)
        "DiagnosticValue",
        "DiagnosticReport",
        "NodeHeartbeat",
        "SafetyStatus",
        # Navigation types (additional)
        "Waypoint",
        "NavPath",
        "VelocityObstacle",
        "VelocityObstacles",
        "OccupancyGrid",
        "CostMap",
        # I/O messages
        "DigitalIO",
        "AnalogIO",
    ])
