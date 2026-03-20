"""
HORUS Python - Simple, Intuitive Robotics Framework

A user-friendly Python API for the HORUS robotics framework.

Quick start::

    import horus

    # Create an image backed by shared memory
    img = horus.Image(480, 640, "rgb8")

    # Send over a topic (zero-copy)
    topic = horus.Topic("camera.rgb")
    topic.send(img)

    # Receive (in another process)
    received = topic.recv()
    arr = received.to_numpy()   # zero-copy NumPy array

Domain types: ``Image``, ``PointCloud``, ``DepthImage``
Communication: ``Topic``, ``Node``, ``Scheduler``

Common Mistakes:

1. **Budget/deadline units** - budget and deadline are in SECONDS, not microseconds.
   Wrong: ``scheduler.add(node, budget=300)``  (300 seconds!)
   Right: ``scheduler.add(node, budget=300 * us)``  (300 microseconds)
   The ``us`` and ``ms`` constants are available: ``from horus import us, ms``

2. **Topic type** - ``Topic(int)`` or ``Topic(42)`` raises TypeError.
   Wrong: ``topic = Topic(42)``
   Right: ``topic = Topic(CmdVel)`` or ``Topic("my_topic")``

3. **Rate must be positive** - ``Node(rate=0)`` or ``Node(rate=-1)`` raises ValueError.

4. **Logging outside scheduler** - ``node.log_info("msg")`` called outside
   tick/init/shutdown drops the message silently. Only log inside callbacks.

5. **Undeclared topics** - ``node.send("topic", data)`` auto-creates topics,
   but undeclared topics won't appear in monitoring. Declare in pubs/subs.
"""

# Extend namespace package path to include horus.library
__path__ = __import__('pkgutil').extend_path(__path__, __name__)

from typing import Optional, Any, Dict, List, Callable, Union
import asyncio
import enum
import time


# Import the Rust extension module
try:
    from horus._horus import (
        Topic,  # Unified communication API
        Scheduler as _PyScheduler,
        SchedulerConfig as _SchedulerConfig,
        Miss,
        get_version,
        # Framework clock — time_now/dt/elapsed/tick/budget/rng
        time_now as _time_now,
        time_dt as _time_dt,
        time_elapsed as _time_elapsed,
        time_tick as _time_tick,
        time_budget_remaining as _time_budget_remaining,
        time_rng_float as _time_rng_float,
        # Nanosecond timestamp for TF queries
        get_timestamp_ns,
        # Domain types — clean API hiding DLPack/TensorPool internals
        Image,
        PointCloud,
        DepthImage,
        # Tensor system — general-purpose zero-copy shared memory tensor
        Tensor,
        TensorPool,
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
        # Vision types
        CompressedImage as _RustCompressedImage,
        CameraInfo as _RustCameraInfo,
        RegionOfInterest as _RustRegionOfInterest,
        StereoInfo as _RustStereoInfo,
        # Force types (additional)
        ImpedanceParameters as _RustImpedanceParameters,
        HapticFeedback as _RustHapticFeedback,
        TactileArray as _RustTactileArray,
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
        # Runtime parameters
        Params,
        # Rate limiter
        Rate,
    )
    # Drivers submodule — import as horus.drivers
    from horus._horus import drivers
except ImportError:
    # Fallback for testing without Rust bindings
    import warnings
    warnings.warn(
        "horus Rust extension not available — running in mock mode. "
        "Most functionality will not work. Install with: maturin develop",
        RuntimeWarning,
        stacklevel=2,
    )
    Topic = None  # Unified communication API
    _PyScheduler = None

    # Mock _SchedulerConfig for testing
    class _SchedulerConfig:
        def __init__(self):
            self.tick_rate = 60.0
            self.watchdog_timeout_ms = 0
            self.max_deadline_misses = 100
            self.memory_locking = False
            self.rt_scheduling_class = False
            self.black_box_size_mb = 0
            self.cpu_cores = None
            self.telemetry_endpoint = None
            self.recording_enabled = False
        @staticmethod
        def minimal(): return _SchedulerConfig()

    def get_version(): return "0.1.0-mock"

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


# Single source of truth: Cargo.toml via env!("CARGO_PKG_VERSION")
try:
    __version__ = get_version().split("v", 1)[-1]
except Exception:
    __version__ = "0.1.9"

# Time unit constants for readable budget/deadline values.
# Usage: sched.add(motor, budget=300 * us, deadline=900 * us)
us = 1e-6   # microseconds → seconds
ms = 1e-3   # milliseconds → seconds


# ── Framework clock API ──────────────────────────────────────────────────────
# Wraps Rust time functions with Pythonic names.
# Normal mode: wall clock. Deterministic mode: SimClock (fixed dt, seeded RNG).

def now() -> float:
    """Current framework time in seconds.

    Normal mode: wall clock. Deterministic mode: virtual SimClock.
    """
    return _time_now()

def dt() -> float:
    """Timestep for this tick in seconds.

    Normal mode: actual elapsed. Deterministic mode: fixed 1/rate.
    """
    return _time_dt()

def elapsed() -> float:
    """Time elapsed since scheduler start in seconds."""
    return _time_elapsed()

def tick() -> int:
    """Current tick number."""
    return _time_tick()

def budget_remaining() -> float:
    """Time remaining in this tick's budget in seconds.

    Returns ``float('inf')`` if no budget configured.
    """
    return _time_budget_remaining()

def rng_float() -> float:
    """Random float in [0.0, 1.0) from the deterministic RNG.

    Normal mode: system entropy. Deterministic mode: tick-seeded.
    """
    return _time_rng_float()

def timestamp_ns() -> int:
    """Current timestamp in nanoseconds (for TF queries)."""
    return get_timestamp_ns()


# ── Perception types (from _horus.perception submodule) ──────────────────────
try:
    from horus._horus.perception import (
        DetectionList,
        PointXYZ,
        PointXYZRGB,
        PointCloudBuffer,
        COCOPose,
    )
except ImportError:
    pass



class Node:
    """
    HORUS node — all config in one place, one way to use it.

    Example::

        from horus import Node, run, us

        def navigate(node):
            if node.has_msg("scan"):
                scan = node.recv("scan")
                node.send("cmd", CmdVel(1.0, scan.ranges[0]))

        node = Node(
            name="navigator",
            tick=navigate,
            rate=30,
            order=1,
            subs=["scan"],
            pubs=["cmd"],
        )
        run(node)

    Async example::

        async def fetch(node):
            data = await http_get("http://sensor/data")
            node.send("data", data)

        run(Node(tick=fetch, pubs=["data"], rate=10))

    RT example::

        from horus import us
        node = Node(
            tick=motor_ctrl,
            rate=1000,
            order=0,
            budget=300 * us,
            deadline=900 * us,
            on_miss="skip",
            core=2,
        )
        run(node, rt=True, watchdog_ms=500)
    """

    def __init__(self,
                 name: Optional[str] = None,
                 tick: Optional[Callable[['Node'], None]] = None,
                 rate: float = 30,
                 # Topics
                 pubs: Optional[Union[List[str], str, Dict[str, Dict]]] = None,
                 subs: Optional[Union[List[str], str, Dict[str, Dict]]] = None,
                 # Lifecycle callbacks
                 init: Optional[Callable[['Node'], None]] = None,
                 shutdown: Optional[Callable[['Node'], None]] = None,
                 on_error: Optional[Callable[['Node', Exception], None]] = None,
                 # Scheduling config (maps 1:1 to Rust NodeBuilder)
                 order: int = 100,
                 budget: Optional[float] = None,
                 deadline: Optional[float] = None,
                 on_miss: Optional[str] = None,
                 failure_policy: Optional[str] = None,
                 compute: bool = False,
                 on: Optional[str] = None,
                 priority: Optional[int] = None,
                 core: Optional[int] = None,
                 watchdog: Optional[float] = None,
                 # Internal
                 default_capacity: int = 1024):
        """
        Create a HORUS node. All config in one place.

        Args:
            name: Node name (auto-generated if None)
            tick: Function called each tick — ``tick(node)``.
                  Can be ``async def`` — auto-detected, runs on async I/O thread pool.
            rate: Tick rate in Hz (default 30)
            pubs: Topics to publish. Accepts:
                  - ``[CmdVel, LaserScan]`` — typed (fast Pod, ~1.5μs)
                  - ``{"cmd": CmdVel}`` — typed with custom name
                  - ``["data"]`` — string (GenericMessage, ~5-50μs)
            subs: Topics to subscribe — same formats as pubs
            init: Called once before first tick — ``init(node)``. Can be async.
            shutdown: Called on scheduler stop — ``shutdown(node)``. Can be async.
            on_error: Error handler — ``on_error(node, exception)``
            order: Execution order (lower = earlier, default: 100)
            budget: Tick budget in seconds (e.g., ``300 * us``). None = auto (80% of period).
            deadline: Tick deadline in seconds (e.g., ``900 * us``). None = auto (95% of period).
            on_miss: Deadline miss policy — ``"warn"``, ``"skip"``, ``"safe_mode"``, ``"stop"``
            failure_policy: Error policy — ``"fatal"``, ``"restart"``, ``"skip"``, ``"ignore"``
            compute: CPU-bound execution (thread pool). Mutually exclusive with ``on`` and async.
            on: Event-driven — tick when this topic receives data. Mutually exclusive with ``compute``.
            priority: OS scheduling priority (lower = higher priority)
            core: Pin to CPU core index
            watchdog: Per-node watchdog timeout in seconds
            default_capacity: Topic ring buffer capacity (default: 1024)

        Example:
            from horus import Node, run, us

            node = Node(
                name="motor",
                tick=motor_control,
                rate=1000,
                order=0,
                budget=300 * us,
                deadline=900 * us,
                on_miss="skip",
                subs=["cmd"],
                pubs=["status"],
            )
            run(node, rt=True)
        """
        # Auto-generate name if not provided
        if name is None:
            import uuid
            name = f"node_{uuid.uuid4().hex[:8]}"

        self.name = name
        self.on_error_fn = on_error
        if rate <= 0:
            raise ValueError(f"Node rate must be positive (got {rate})")
        self.rate = rate
        self.default_capacity = default_capacity

        # Scheduling config (read by Scheduler._add_node → Rust NodeBuilder)
        self.order = order
        self.budget = budget
        self.deadline = deadline
        self.on_miss = on_miss
        self.failure_policy = failure_policy
        self._horus_compute = compute
        self._horus_on = on
        self.priority = priority
        self.core = core
        self.watchdog = watchdog

        # Detect async callbacks and wrap with sync bridge.
        # If any callback is async, mark node for async_io execution class.
        self._horus_async = False
        self._async_loop: Optional[asyncio.AbstractEventLoop] = None
        self.tick_fn = self._wrap_async(tick) if tick else None
        self.init_fn = self._wrap_async(init) if init else None
        self.shutdown_fn = self._wrap_async(shutdown) if shutdown else None

        # Validate mutually exclusive execution classes
        exec_modes = sum([self._horus_async, self._horus_compute, self._horus_on is not None])
        if exec_modes > 1:
            raise ValueError(
                "Execution class is mutually exclusive: only one of "
                "async tick, compute=True, or on='topic' can be set"
            )

        # Normalize pub/sub to lists and extract configs
        self.pub_topics = []
        self.sub_topics = []
        self._topic_configs = {}  # topic -> config dict

        # Process pubs/subs — normalize to topic names + type configs
        self.pub_topics = self._parse_topics(pubs)
        self.sub_topics = self._parse_topics(subs)

        # NodeInfo context (set by scheduler)
        self.info = None

        # Create underlying HORUS components if available
        if Topic is not None:
            self._rust_available = True
            self._setup_topics()
        else:
            # Mock mode for testing
            self._rust_available = False
            self._topics = {}

    def _parse_topics(self, spec) -> List[str]:
        """Parse pubs/subs spec into topic names, storing type configs.

        Accepts:
            None                          → []
            "topic"                       → ["topic"] (GenericMessage)
            ["topic1", "topic2"]          → ["topic1", "topic2"] (GenericMessage)
            [CmdVel, LaserScan]           → ["cmdvel", "laserscan"] (typed, fast Pod)
            {"cmd": CmdVel}               → ["cmd"] (typed, fast Pod)
            {"cmd": {"type": CmdVel}}     → ["cmd"] (typed, legacy format)
        """
        if spec is None:
            return []

        if isinstance(spec, str):
            return [spec]

        if isinstance(spec, dict):
            names = []
            for key, value in spec.items():
                names.append(key)
                if isinstance(value, type):
                    # {"cmd": CmdVel} — type directly
                    self._topic_configs[key] = {"type": value}
                elif isinstance(value, dict):
                    # {"cmd": {"type": CmdVel, "capacity": 2048}} — legacy dict
                    self._topic_configs[key] = value or {}
                elif value is None:
                    pass
            return names

        if isinstance(spec, list):
            names = []
            for item in spec:
                if isinstance(item, str):
                    # "topic" — string name, GenericMessage
                    names.append(item)
                elif isinstance(item, type):
                    # CmdVel — type, auto-derive name
                    name = getattr(item, '__topic_name__', None) or item.__name__.lower()
                    names.append(name)
                    self._topic_configs[name] = {"type": item}
                else:
                    raise TypeError(
                        f"pubs/subs list items must be str or message type, got {type(item).__name__}"
                    )
            return names

        raise TypeError(f"pubs/subs must be str, list, or dict, got {type(spec).__name__}")

    def _wrap_async(self, fn: Callable) -> Callable:
        """If fn is a coroutine function, wrap it in a sync bridge and mark node as async."""
        if fn is None:
            return None
        if not asyncio.iscoroutinefunction(fn):
            return fn
        # Mark this node for async_io execution class
        self._horus_async = True
        def sync_bridge(node):
            loop = self._get_async_loop()
            loop.run_until_complete(fn(node))
        return sync_bridge

    def _get_async_loop(self) -> asyncio.AbstractEventLoop:
        """Get or create event loop for async callbacks."""
        if self._async_loop is None:
            try:
                self._async_loop = asyncio.get_running_loop()
            except RuntimeError:
                self._async_loop = asyncio.new_event_loop()
                asyncio.set_event_loop(self._async_loop)
        return self._async_loop

    def _setup_topics(self):
        """Setup publish/subscribe topics with configured capacities.

        If the user declared a type via pubs/subs dict config, create a typed
        Topic (Pod zero-copy path, ~1.5μs). Otherwise fall back to string-named
        Topic (GenericMessage path, ~5-50μs).
        """
        self._topics = {}

        all_topics = set(self.pub_topics + self.sub_topics)
        for topic in all_topics:
            config = self._topic_configs.get(topic, {})
            capacity = config.get('capacity', self.default_capacity)
            msg_type = config.get('type', None)

            if msg_type is not None:
                # Typed topic — Pod zero-copy path (~1.5μs)
                self._topics[topic] = Topic(msg_type, capacity)
            else:
                # String topic — GenericMessage path (~5-50μs)
                self._topics[topic] = Topic(topic, capacity)

    def _ensure_topic(self, topic: str) -> None:
        """Ensure a Topic object exists for the given name (auto-create if needed)."""
        if topic not in self._topics and self._rust_available:
            config = self._topic_configs.get(topic, {})
            capacity = config.get('capacity', self.default_capacity)
            msg_type = config.get('type', None)
            if msg_type is not None:
                self._topics[topic] = Topic(msg_type, capacity)
            else:
                self._topics[topic] = Topic(topic, capacity)

    def has_msg(self, topic: str) -> bool:
        """
        Check if messages are available on a topic.

        Peeks by attempting a recv — if a message exists, it's buffered
        internally and returned by the next ``recv()`` call.

        Args:
            topic: Topic to check

        Returns:
            True if messages available
        """
        # Check peek buffer first
        if hasattr(self, '_peek_buffer') and topic in self._peek_buffer:
            return True
        self._ensure_topic(topic)
        if self._rust_available and topic in self._topics:
            msg = self._topics[topic].recv(self)
            if msg is not None:
                if not hasattr(self, '_peek_buffer'):
                    self._peek_buffer = {}
                self._peek_buffer[topic] = msg
                return True
        return False

    def recv(self, topic: str) -> Optional[Any]:
        """
        Receive next message from topic.

        Calls Rust ``Topic.recv()`` directly — no Python-side buffering.

        Args:
            topic: Topic to read from

        Returns:
            Message data or None if no messages

        Note:
            Topics used here but not declared in ``subs`` will be auto-created
            for IPC, but won't appear in scheduler monitoring or diagnostics.
            Declare all topics in ``subs`` for full visibility.
        """
        # Return peeked message first (from has_msg)
        if hasattr(self, '_peek_buffer') and topic in self._peek_buffer:
            return self._peek_buffer.pop(topic)
        if topic not in self.sub_topics:
            self.sub_topics.append(topic)
        self._ensure_topic(topic)
        if self._rust_available and topic in self._topics:
            return self._topics[topic].recv(self)
        return None

    def recv_all(self, topic: str) -> List[Any]:
        """
        Receive all available messages from topic.

        Args:
            topic: Topic to read from

        Returns:
            List of messages (empty if none)
        """
        msgs = []
        while True:
            msg = self.recv(topic)
            if msg is None:
                break
            msgs.append(msg)
        return msgs


    def send(self, topic: str, data: Any) -> bool:
        """
        Send data to a topic.

        Calls Rust ``Topic.send()`` directly.

        Args:
            topic: Topic to send to
            data: Data to send (supports numpy arrays with zero-copy)

        Returns:
            True if sent successfully

        Note:
            Topics used here but not declared in ``pubs`` will be auto-created
            for IPC, but won't appear in scheduler monitoring or diagnostics.
            Declare all topics in ``pubs`` for full visibility.
        """
        if topic not in self.pub_topics:
            self.pub_topics.append(topic)
        self._ensure_topic(topic)
        if self._rust_available and topic in self._topics:
            return self._topics[topic].send(data, self)
        return True

    def _run_tick_with_error_handling(self, info: Optional[Any] = None) -> None:
        """Run tick_fn with error handling and info context management.

        Error escalation is handled by Rust FailurePolicy (fatal/restart/skip/ignore).
        Python only handles user's on_error callback if provided.
        """
        old_info = self.info
        self.info = info
        try:
            if self.tick_fn:
                self.tick_fn(self)
        except Exception as e:
            if self.on_error_fn:
                try:
                    self.on_error_fn(self, e)
                except Exception:
                    raise e  # on_error failed too — propagate original
            else:
                raise  # Rust FailurePolicy handles escalation
        finally:
            self.info = old_info

    def _internal_tick(self, info: Optional[Any] = None) -> None:
        """Internal tick called by mock-mode scheduler."""
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
        """Log an info message. Only works during init/tick/shutdown."""
        if self.info:
            self.info.log_info(message)
        else:
            import warnings
            warnings.warn(
                f"Node '{self.name}': log_info() called outside scheduler — message dropped. "
                "Logging is only available during init/tick/shutdown.",
                RuntimeWarning, stacklevel=2,
            )

    def log_warning(self, message: str) -> None:
        """Log a warning message. Only works during init/tick/shutdown."""
        if self.info:
            self.info.log_warning(message)
        else:
            import warnings
            warnings.warn(
                f"Node '{self.name}': log_warning() called outside scheduler — message dropped. "
                "Logging is only available during init/tick/shutdown.",
                RuntimeWarning, stacklevel=2,
            )

    def log_error(self, message: str) -> None:
        """Log an error message. Only works during init/tick/shutdown."""
        if self.info:
            self.info.log_error(message)
        else:
            import warnings
            warnings.warn(
                f"Node '{self.name}': log_error() called outside scheduler — message dropped. "
                "Logging is only available during init/tick/shutdown.",
                RuntimeWarning, stacklevel=2,
            )

    def log_debug(self, message: str) -> None:
        """Log a debug message. Only works during init/tick/shutdown."""
        if self.info:
            self.info.log_debug(message)
        else:
            import warnings
            warnings.warn(
                f"Node '{self.name}': log_debug() called outside scheduler — message dropped. "
                "Logging is only available during init/tick/shutdown.",
                RuntimeWarning, stacklevel=2,
            )

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

    Most users should use ``horus.run()`` instead. Use Scheduler directly
    only for dynamic node management or advanced control.

    Example::

        from horus import Node, Scheduler

        sched = Scheduler(tick_rate=1000, rt=True)
        sched.add(Node(tick=sensor_fn, rate=100, order=0))
        sched.add(Node(tick=motor_fn, rate=1000, order=1, budget=300 * us))
        sched.run()

    Context manager::

        with Scheduler(tick_rate=100) as sched:
            sched.add(Node(tick=fn, rate=100))
            sched.run(duration=10.0)
    """

    def __init__(self, *,
                 tick_rate: float = 1000.0,
                 rt: bool = False,
                 deterministic: bool = False,
                 blackbox_mb: int = 0,
                 watchdog_ms: int = 0,
                 recording: bool = False,
                 name: Optional[str] = None,
                 cores: Optional[List[int]] = None,
                 max_deadline_misses: Optional[int] = None,
                 verbose: bool = False,
                 telemetry: Optional[str] = None,
                 _inner=None):
        """
        Create a scheduler.

        Args:
            tick_rate: Global tick rate in Hz (default: 1000.0)
            rt: Enable real-time features (memory locking + RT scheduling class)
            deterministic: Enable deterministic mode (SimClock, fixed dt, seeded RNG)
            blackbox_mb: Black box buffer size in MB (0 = disabled)
            watchdog_ms: Watchdog timeout in ms (0 = disabled)
            recording: Enable recording (default: False)
            name: Scheduler name (for logging/diagnostics)
            cores: CPU affinity — pin scheduler to these cores
            max_deadline_misses: Escalation threshold for deadline misses
            verbose: Enable verbose debug logging
            telemetry: Telemetry export endpoint (e.g., "http://localhost:9090")
            _inner: Internal — used by preset constructors
        """
        if _inner is not None:
            self._scheduler = _inner
        elif _PyScheduler:
            cfg = _SchedulerConfig.minimal()
            cfg.tick_rate = tick_rate
            if rt:
                cfg.memory_locking = True
                cfg.rt_scheduling_class = True
            if blackbox_mb > 0:
                cfg.black_box_size_mb = blackbox_mb
            if watchdog_ms > 0:
                cfg.watchdog_timeout_ms = watchdog_ms
            cfg.recording_enabled = recording
            self._scheduler = _PyScheduler(
                cfg,
                deterministic=deterministic,
                name=name,
                cores=cores,
                max_deadline_misses=max_deadline_misses,
                verbose=verbose,
                telemetry=telemetry,
            )
        else:
            self._scheduler = None
        self._nodes = []
        self._tick_rate = tick_rate

    # Presets deploy(), hard_rt(), safety_critical() removed.
    # Use Scheduler(watchdog_ms=500) or
    # Scheduler(config=SchedulerConfig.with_watchdog()) instead.

    def add(self, node: 'Node') -> 'Scheduler':
        """
        Add a node to the scheduler. All config is read from the Node.

        Args:
            node: ``horus.Node`` instance (with scheduling config set via kwargs)

        Returns:
            self (for method chaining)

        Example:
            sched.add(Node(tick=sensor_fn, rate=100, order=0))
            sched.add(Node(tick=motor_fn, rate=1000, order=1, budget=300 * us))
        """
        if not isinstance(node, Node):
            raise TypeError(
                f"Expected horus.Node instance, got {type(node).__name__}. "
                "Use horus.Node(tick=fn, rate=30, ...) to create nodes."
            )
        self._nodes.append(node)
        if self._scheduler:
            self._add_node(node)
        return self

    def _add_node(self, node: 'Node') -> None:
        """Wire Node attrs to Rust PyNodeBuilder. Single path for all config."""
        builder = self._scheduler.node(node)
        builder = builder.order(node.order)
        if node.rate is not None:
            builder = builder.rate(node.rate)
        if node.budget is not None:
            builder = builder.budget(node.budget)
        if node.deadline is not None:
            builder = builder.deadline(node.deadline)
        if node.on_miss is not None:
            builder = builder.on_miss(node.on_miss)
        if node.failure_policy is not None:
            builder = builder.failure_policy(node.failure_policy)
        # Execution class (mutually exclusive, validated in Node.__init__)
        if node._horus_async:
            builder = builder.async_io()
        elif node._horus_compute:
            builder = builder.compute()
        elif node._horus_on:
            builder = builder.on(node._horus_on)
        if node.priority is not None:
            builder = builder.priority(node.priority)
        if node.core is not None:
            builder = builder.core(node.core)
        if node.watchdog is not None:
            builder = builder.watchdog(node.watchdog)
        builder.build()

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
                    time.sleep(1.0 / self._tick_rate)
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
        """Context manager entry — returns self for ``with`` usage.

        Example::

            with Scheduler(tick_rate=100) as sched:
                sched.add(Node(tick=sensor_fn, rate=100, order=0))
                sched.run(duration=10.0)
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

    def status(self) -> str:
        """Get scheduler status string (e.g. "idle", "running", "stopped")."""
        if self._scheduler:
            return self._scheduler.status()
        return "mock"

    def current_tick(self) -> int:
        """Get the current tick count."""
        if self._scheduler:
            return self._scheduler.current_tick()
        return 0


# Convenience functions

def run(*nodes: Node,
        duration: Optional[float] = None,
        tick_rate: float = 1000.0,
        rt: bool = False,
        deterministic: bool = False,
        watchdog_ms: int = 0,
        blackbox_mb: int = 0,
        recording: bool = False,
        name: Optional[str] = None,
        cores: Optional[List[int]] = None,
        max_deadline_misses: Optional[int] = None,
        verbose: bool = False,
        telemetry: Optional[str] = None) -> None:
    """
    Run nodes. The ONE way to use horus.

    Each node carries its own config (rate, order, budget, etc.).
    This function handles scheduler creation and execution.

    Args:
        *nodes: ``horus.Node`` instances (in execution order if order not set)
        duration: Run for this many seconds (None = forever until Ctrl+C)
        tick_rate: Global scheduler tick rate in Hz (default: 1000.0)
        rt: Enable real-time features (memory locking + RT scheduling)
        deterministic: Enable deterministic mode (SimClock, fixed dt, seeded RNG)
        watchdog_ms: Global watchdog timeout in ms (0 = disabled)
        blackbox_mb: Flight recorder buffer in MB (0 = disabled)
        recording: Enable session recording (default: False)
        name: Scheduler name (for logging/diagnostics)
        cores: CPU affinity — pin scheduler to these cores
        max_deadline_misses: Escalation threshold for deadline misses
        verbose: Enable verbose debug logging
        telemetry: Telemetry export endpoint (e.g., "http://localhost:9090")

    Example::

        from horus import Node, run, us

        sensor = Node(tick=read_lidar, rate=10, order=0, pubs=["scan"])
        ctrl = Node(tick=navigate, rate=30, order=1, subs=["scan"], pubs=["cmd"])
        motor = Node(tick=drive, rate=1000, order=2, budget=300*us, subs=["cmd"])

        run(sensor, ctrl, motor, rt=True)
    """
    scheduler = Scheduler(
        tick_rate=tick_rate,
        deterministic=deterministic,
        rt=rt,
        watchdog_ms=watchdog_ms,
        blackbox_mb=blackbox_mb,
        recording=recording,
        name=name,
        cores=cores,
        max_deadline_misses=max_deadline_misses,
        verbose=verbose,
        telemetry=telemetry,
    )

    for node in nodes:
        scheduler.add(node)

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
        PidConfig,
        # Sensor messages
        LaserScan,
        Imu,
        BatteryState,
        NavSatFix,
        Odometry,
        # Diagnostics messages
        EmergencyStop,
        Heartbeat,
        ResourceUsage,
        # Input messages
        JoystickInput,
        KeyboardInput,
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
        CompressedImage = _RustCompressedImage
        CameraInfo = _RustCameraInfo
        RegionOfInterest = _RustRegionOfInterest
        StereoInfo = _RustStereoInfo
        ImpedanceParameters = _RustImpedanceParameters
        HapticFeedback = _RustHapticFeedback
        TactileArray = _RustTactileArray
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
    # Core API — the ONE way
    "Node",
    "run",
    "Scheduler",
    # Time API
    "now",
    "dt",
    "elapsed",
    "tick",
    "budget_remaining",
    "rng_float",
    "timestamp_ns",
    # Unit constants
    "us",
    "ms",
    # Configuration
    "Params",
    "Rate",
    "drivers",
    # Communication
    "Topic",
    # Domain types
    "Image",
    "PointCloud",
    "DepthImage",
    # Coordinate transforms
    "TransformFrame",
    "Transform",
    "TransformFrameConfig",
    # Enums
    "NodeState",
    # Perception
    "DetectionList",
    "PointXYZ",
    "PointXYZRGB",
    "PointCloudBuffer",
    "COCOPose",
    # Error types
    "HorusNotFoundError",
    "HorusTransformError",
    "HorusTimeoutError",
    # Submodules
    "msggen",
    "perception",
    # Utility
    "get_version",
]

# Import custom message generator module
from . import msggen


    # Rust-native types are already assigned in the fallback block above (lines 1380-1454).
    # No second assignment needed.

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
        # Vision types
        "CompressedImage",
        "CameraInfo",
        "RegionOfInterest",
        "StereoInfo",
        # Force types (additional)
        "ImpedanceParameters",
        "HapticFeedback",
        "TactileArray",
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
        # Audio
        "AudioFrame",
    ])
