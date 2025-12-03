"""
HORUS Python - Simple, Intuitive Robotics Framework

A user-friendly Python API for the HORUS robotics framework.
"""

# Extend namespace package path to include horus.library
__path__ = __import__('pkgutil').extend_path(__path__, __name__)

from typing import Optional, Any, Dict, List, Callable, Union
import pickle
import json
from collections import defaultdict
import time

# Maximum size for logged data representation (to prevent buffer overflows)
MAX_LOG_DATA_SIZE = 200

# Import the Rust extension module
try:
    from horus._horus import (
        PyNode as _PyNode,
        PyNodeInfo as _NodeInfo,
        Hub,  # Type-based Hub with network support (exported as "Hub" from Rust)
        Link,  # Point-to-point SPSC communication with network support
        RouterClient,  # Explicit router connection management
        RouterServer,  # Router server management
        default_router_endpoint,  # Helper: "topic@router"
        router_endpoint,  # Helper: "topic@host:port"
        PyScheduler as _PyScheduler,
        PyNodeState as NodeState,
        PyRobotPreset as RobotPreset,
        PySchedulerConfig as SchedulerConfig,
        get_version,
        # sim2d Python API
        Sim2D,
        RobotConfigPy,
        WorldConfigPy,
        # Tensor system for zero-copy ML/AI
        TensorPool,
        TensorHandle,
        cuda_is_available,
        cuda_device_count,
    )
except ImportError:
    # Fallback for testing without Rust bindings
    print("Warning: Rust bindings not available. Running in mock mode.")
    _PyNode = None
    _NodeInfo = None
    Hub = None  # Type-based Hub with network support
    Link = None  # Point-to-point SPSC communication
    RouterClient = None  # Router client management
    RouterServer = None  # Router server management
    default_router_endpoint = lambda t: f"{t}@router"
    router_endpoint = lambda t, h="127.0.0.1", p=7777: f"{t}@{h}:{p}"
    _PyScheduler = None

    # Mock NodeState for testing
    class NodeState:
        UNINITIALIZED = "uninitialized"
        INITIALIZING = "initializing"
        RUNNING = "running"
        PAUSED = "paused"
        STOPPING = "stopping"
        STOPPED = "stopped"
        ERROR = "error"
        CRASHED = "crashed"

    # Mock RobotPreset for testing
    class RobotPreset:
        Standard = "Standard"
        SafetyCritical = "SafetyCritical"
        HardRealTime = "HardRealTime"
        HighPerformance = "HighPerformance"
        Educational = "Educational"
        Mobile = "Mobile"
        Underwater = "Underwater"
        Space = "Space"
        Swarm = "Swarm"
        SoftRobotics = "SoftRobotics"
        Custom = "Custom"

    # Mock SchedulerConfig for testing
    class SchedulerConfig:
        def __init__(self, preset=None):
            pass

    def get_version(): return "0.1.0-mock"

    # Mock TensorPool and TensorHandle for testing
    class TensorPool:
        def __init__(self, pool_id=1, size_mb=1024, max_slots=1024):
            self.pool_id = pool_id
        def alloc(self, shape, dtype="float32", device="cpu"):
            return TensorHandle()
        def stats(self):
            return {}

    class TensorHandle:
        pass

__version__ = "0.1.5"


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
                  - str: single topic (generic hub)
                  - list: ["topic1", "topic2"] (generic hubs)
                  - dict: {"topic1": {"type": CmdVel, "capacity": 2048}, "topic2": {}}
                    Use 'type' to create typed hubs for proper message logging
            subs: Topics to subscribe to (same format as pubs)
            tick: Function to call on each tick - signature: tick(node)
            rate: Tick rate in Hz (default 30)
            init: Optional init function - signature: init(node)
            shutdown: Optional shutdown function - signature: shutdown(node)
            on_error: Optional error handler - signature: on_error(node, exception)
            default_capacity: Default hub capacity (default: 1024)

        Example with typed hubs (recommended for proper logging):
            from horus import Node, Pose2D, CmdVel

            node = Node(
                name="controller",
                subs={"localization/pose": {"type": Pose2D}},
                pubs={"control/cmd": {"type": CmdVel, "capacity": 2048}},
                tick=lambda n: None
            )

        Example with generic hubs (shows <bytes> in logs):
            node = Node(
                name="controller",
                subs=["localization/pose"],  # Generic hub
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
            self._node = _PyNode(name)
            self._setup_hubs()
        else:
            # Mock mode for testing
            self._node = None
            self._hubs = {}

    def _setup_hubs(self):
        """Setup publish/subscribe hubs with configured capacities and types."""
        self._hubs = {}

        # Create publisher hubs
        for topic in self.pub_topics:
            config = self._topic_configs.get(topic, {})
            capacity = config.get('capacity', self.default_capacity)
            msg_type = config.get('type', None)

            # If type specified, create typed hub; otherwise generic hub
            if msg_type is not None:
                # Temporarily set __topic_name__ so Rust Hub uses correct topic
                original_topic = getattr(msg_type, '__topic_name__', None)
                msg_type.__topic_name__ = topic
                try:
                    self._hubs[topic] = Hub(msg_type, capacity)
                finally:
                    # Restore original or delete
                    if original_topic is not None:
                        msg_type.__topic_name__ = original_topic
                    elif hasattr(msg_type, '__topic_name__'):
                        delattr(msg_type, '__topic_name__')
            else:
                self._hubs[topic] = Hub(topic, capacity)

        # Create subscriber hubs
        for topic in self.sub_topics:
            config = self._topic_configs.get(topic, {})
            capacity = config.get('capacity', self.default_capacity)
            msg_type = config.get('type', None)

            # If type specified, create typed hub; otherwise generic hub
            if msg_type is not None:
                # Temporarily set __topic_name__ so Rust Hub uses correct topic
                original_topic = getattr(msg_type, '__topic_name__', None)
                msg_type.__topic_name__ = topic
                try:
                    self._hubs[topic] = Hub(msg_type, capacity)
                finally:
                    # Restore original or delete
                    if original_topic is not None:
                        msg_type.__topic_name__ = original_topic
                    elif hasattr(msg_type, '__topic_name__'):
                        delattr(msg_type, '__topic_name__')
            else:
                self._hubs[topic] = Hub(topic, capacity)

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
            if self._node:
                config = self._topic_configs.get(topic, {})
                capacity = config.get('capacity', self.default_capacity)
                msg_type = config.get('type', None)

                # If type specified, create typed hub; otherwise generic hub
                if msg_type is not None:
                    # Temporarily set __topic_name__ so Rust Hub uses correct topic
                    original_topic = getattr(msg_type, '__topic_name__', None)
                    msg_type.__topic_name__ = topic
                    try:
                        self._hubs[topic] = Hub(msg_type, capacity)
                    finally:
                        # Restore original or delete
                        if original_topic is not None:
                            msg_type.__topic_name__ = original_topic
                        elif hasattr(msg_type, '__topic_name__'):
                            delattr(msg_type, '__topic_name__')
                else:
                    self._hubs[topic] = Hub(topic, capacity)

        if self._node and topic in self._hubs:
            hub = self._hubs[topic]

            # Measure IPC timing
            import time
            start_ns = time.perf_counter_ns()

            # Serialize based on type
            if isinstance(data, bytes):
                result = hub.send_bytes(data, self)
            elif isinstance(data, str):
                result = hub.send_bytes(data.encode('utf-8'), self)
            elif isinstance(data, (dict, list, tuple, int, float, bool, type(None))):
                # Only use metadata for generic hubs
                if hub.is_generic():
                    json_bytes = json.dumps(data).encode('utf-8')
                    result = hub.send_with_metadata(json_bytes, "json", self)
                else:
                    # Typed hubs - use send() directly with the Python object
                    result = hub.send(data, self)
            elif hasattr(data, '__array_interface__') or type(data).__name__ == 'ndarray':
                # Zero-copy path for numpy arrays
                result = hub.send_numpy(data, self)
            else:
                # Only use metadata for generic hubs
                if hub.is_generic():
                    pickled = pickle.dumps(data)
                    result = hub.send_with_metadata(pickled, "pickle", self)
                else:
                    # Typed hubs - use send() directly with the Python object
                    result = hub.send(data, self)

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
        """Pull messages from hub into queue (Phase 2: with timestamps)."""
        # Auto-detect topics: add topic if not already declared
        if topic not in self.sub_topics:
            self.sub_topics.append(topic)
            if self._node:
                config = self._topic_configs.get(topic, {})
                capacity = config.get('capacity', self.default_capacity)
                msg_type = config.get('type', None)

                # If type specified, create typed hub; otherwise generic hub
                if msg_type is not None:
                    # Temporarily set __topic_name__ so Rust Hub uses correct topic
                    original_topic = getattr(msg_type, '__topic_name__', None)
                    msg_type.__topic_name__ = topic
                    try:
                        self._hubs[topic] = Hub(msg_type, capacity)
                    finally:
                        # Restore original or delete
                        if original_topic is not None:
                            msg_type.__topic_name__ = original_topic
                        elif hasattr(msg_type, '__topic_name__'):
                            delattr(msg_type, '__topic_name__')
                else:
                    self._hubs[topic] = Hub(topic, capacity)

        if self._node and topic in self._hubs:
            hub = self._hubs[topic]
            import time

            # Receive all available messages
            while True:
                # Measure IPC timing
                start_ns = time.perf_counter_ns()

                # Use metadata method only for generic hubs
                if hub.is_generic():
                    result = hub.recv_with_metadata(self)
                    end_ns = time.perf_counter_ns()

                    if result is None:
                        break

                    ipc_ns = end_ns - start_ns
                    data_bytes, msg_type, timestamp = result  # Phase 2: Now includes timestamp

                    # Deserialize
                    if msg_type == "json":
                        msg = json.loads(data_bytes.decode('utf-8'))
                    elif msg_type == "pickle":
                        msg = pickle.loads(data_bytes)
                    elif msg_type == "numpy":
                        # Keep as raw bytes for numpy arrays
                        msg = data_bytes
                    else:
                        try:
                            msg = data_bytes.decode('utf-8')
                        except:
                            msg = data_bytes
                else:
                    # Typed hub - use regular recv()
                    msg = hub.recv(self)
                    end_ns = time.perf_counter_ns()

                    if msg is None:
                        break

                    ipc_ns = end_ns - start_ns
                    # Typed hubs don't have metadata timestamps, use current time
                    timestamp = time.time()

                # Log the subscribe operation if NodeInfo available
                if self.info:
                    data_repr = _truncate_for_logging(msg)
                    self.info.log_sub(topic, data_repr, ipc_ns)

                # Phase 2: Store message with timestamp
                self._msg_queues[topic].append(msg)
                self._msg_timestamps[topic].append(timestamp)

    def _internal_tick(self, info: Optional[Any] = None) -> None:
        """Internal tick called by scheduler with per-node rate control."""
        import time

        # Check if enough time has elapsed for this node's rate
        current_time = time.time()
        if self._tick_period > 0:
            time_since_last_tick = current_time - self._last_tick_time
            if time_since_last_tick < self._tick_period:
                # Not time to tick yet - skip this call
                return

        # Update last tick time
        self._last_tick_time = current_time

        # DON'T store info - use a context manager approach
        old_info = self.info
        self.info = info
        try:
            if self.tick_fn:
                self.tick_fn(self)
        except Exception as e:
            # Increment error count
            self.error_count += 1

            # Log error if info available
            if self.info:
                self.info.log_error(f"Tick failed: {e}")
                # Transition to error state if too many errors
                if self.error_count > 10:
                    self.info.transition_to_error(f"Too many errors ({self.error_count})")

            # Call user's error handler if provided
            if self.on_error_fn:
                try:
                    self.on_error_fn(self, e)
                except Exception as handler_error:
                    # Error handler itself failed - just log it
                    if self.info:
                        self.info.log_error(f"Error handler failed: {handler_error}")
            else:
                # No error handler - re-raise
                raise
        finally:
            self.info = old_info

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
        """Called by Rust scheduler on each tick."""
        self._internal_tick(info)

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
    def get_publishers(self) -> List[str]:
        """
        Get list of topics this node publishes to.

        Returns:
            List of publisher topic names

        Example:
            topics = node.get_publishers()
            print(f"Publishing to: {topics}")
        """
        return self.pub_topics.copy()

    def get_subscribers(self) -> List[str]:
        """
        Get list of topics this node subscribes to.

        Returns:
            List of subscriber topic names

        Example:
            topics = node.get_subscribers()
            print(f"Subscribed to: {topics}")
        """
        return self.sub_topics.copy()


class Scheduler:
    """
    Simple scheduler for running nodes.

    Example:
        scheduler = Scheduler()
        scheduler.add(sensor_node, 0, True)   # priority=0, logging=True
        scheduler.add(control_node, 1, False)  # priority=1, logging=False
        scheduler.add(motor_node, 2, True)     # priority=2, logging=True
        scheduler.run()

        # Or chainable:
        scheduler.add(node1, 0, True).add(node2, 1, False).run()
    """

    def __init__(self, config: Optional['SchedulerConfig'] = None):
        """
        Create a scheduler.

        Args:
            config: Optional SchedulerConfig to configure the scheduler
        """
        if _PyScheduler:
            self._scheduler = _PyScheduler(config) if config else _PyScheduler()
        else:
            self._scheduler = None
        self._nodes = []

    @staticmethod
    def from_config(config: 'SchedulerConfig') -> 'Scheduler':
        """
        Create a scheduler from a configuration.

        Args:
            config: SchedulerConfig instance

        Returns:
            Configured Scheduler instance
        """
        return Scheduler(config=config)

    def add(self, node: 'Node', priority: int, logging: bool = False) -> 'Scheduler':
        """
        Add a node to the scheduler.

        Args:
            node: Node instance to add
            priority: Priority level (lower number = higher priority, 0 = highest)
            logging: Enable logging for this node (default: False)

        Returns:
            self (for method chaining)

        Example:
            scheduler.add(sensor_node, 0, True)   # Highest priority, logging on
            scheduler.add(control_node, 1, False)  # Medium priority, logging off
            scheduler.add(motor_node, 2, True)     # Lowest priority, logging on

            # Chainable:
            scheduler.add(node1, 0, True).add(node2, 1, False).run()
        """
        self._nodes.append(node)

        if self._scheduler:
            # Register the Python Node wrapper directly, not the internal _node
            # The Rust scheduler will call node.tick(info) and node.init(info)
            self._scheduler.add(node, priority, logging, node.rate)

        return self

    def run(self, duration: Optional[float] = None) -> None:
        """
        Run the scheduler.

        Args:
            duration: Optional duration in seconds (runs forever if None)
        """
        if self._scheduler:
            # Initialize all nodes
            for node in self._nodes:
                node._internal_init()

            # Set scheduler tick rate to a high value to support per-node rate control
            # The scheduler needs to tick faster than the fastest node
            self._scheduler.set_tick_rate(1000.0)  # 1000Hz allows fine-grained control

            # Run with KeyboardInterrupt handling
            try:
                if duration:
                    self._scheduler.run_for(duration)
                else:
                    self._scheduler.run()
            except KeyboardInterrupt:
                print("\nCtrl+C received, shutting down gracefully...")
                self._scheduler.stop()
            finally:
                # Shutdown all nodes
                for node in self._nodes:
                    node._internal_shutdown()
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

    def get_node_stats(self, node_name: str) -> Dict[str, Any]:
        """
        Get statistics for a specific node.

        Args:
            node_name: Name of the node

        Returns:
            Dictionary with node stats including:
            - name: Node name
            - priority: Priority level
            - logging_enabled: Whether logging is enabled
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

    def set_node_rate(self, node_name: str, rate_hz: float) -> None:
        """
        Set the execution rate for a specific node.

        Args:
            node_name: Name of the node
            rate_hz: New rate in Hz (ticks per second)

        Example:
            scheduler.set_node_rate("sensor", 100.0)  # Change to 100Hz
        """
        # Update the node's rate attribute
        for node in self._nodes:
            if node.name == node_name:
                node.rate = rate_hz
                node._tick_period = 1.0 / rate_hz if rate_hz > 0 else 0.0
                break

        # Update Rust scheduler if available
        if self._scheduler:
            self._scheduler.set_node_rate(node_name, rate_hz)

    def get_all_nodes(self) -> List[Dict[str, Any]]:
        """
        Get information about all registered nodes.

        Returns:
            List of dictionaries containing node information:
            - name: Node name
            - priority: Execution priority
            - rate_hz: Node execution rate
            - logging_enabled: Whether logging is enabled
            - total_ticks: Total number of ticks executed
            - successful_ticks: Number of successful ticks
            - failed_ticks: Number of failed ticks
            - failure_count: Total failure count
            - consecutive_failures: Current consecutive failure count
            - circuit_open: Whether circuit breaker is open
            - avg_tick_duration_ms: Average tick duration
            - uptime_seconds: Node uptime
            - state: Node state

        Example:
            nodes = scheduler.get_all_nodes()
            for node in nodes:
                print(f"{node['name']}: {node['total_ticks']} ticks, {node['failure_count']} failures")
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

    def set_node_deadline(self, node_name: str, deadline_ms: Optional[float] = None) -> None:
        """
        Set a soft real-time deadline for a specific node.

        This enables deadline monitoring for the node. If the node's tick()
        execution exceeds the deadline, a warning will be logged and the
        deadline_misses counter will be incremented.

        Args:
            node_name: Name of the node to configure
            deadline_ms: Deadline in milliseconds (0-10000), or None to disable deadline monitoring

        Raises:
            RuntimeError: If node not found or deadline is out of range

        Example:
            # Set 10ms deadline for sensor node
            scheduler.set_node_deadline("sensor_node", 10.0)

            # Set 50ms deadline for control node
            scheduler.set_node_deadline("control_node", 50.0)

            # Disable deadline monitoring
            scheduler.set_node_deadline("sensor_node", None)

        Note:
            - Deadlines are "soft" - violations are logged but don't stop execution
            - Deadline monitoring must be enabled in SchedulerConfig (deadline_monitoring=True)
            - Use get_node_stats() or get_all_nodes() to query deadline_misses
        """
        if not self._scheduler:
            raise RuntimeError("Cannot set deadline before scheduler is started")
        self._scheduler.set_node_deadline(node_name, deadline_ms)

    def set_node_watchdog(self, node_name: str, enabled: bool, timeout_ms: Optional[int] = None) -> None:
        """
        Enable or disable watchdog timer for a specific node.

        A watchdog timer monitors node liveness. If the node fails to execute
        successfully within the timeout period, the watchdog expires and a
        warning is logged.

        Args:
            node_name: Name of the node to configure
            enabled: Enable (True) or disable (False) watchdog
            timeout_ms: Timeout in milliseconds (10-60000), or None to use global default

        Raises:
            RuntimeError: If node not found or timeout is out of range

        Example:
            # Enable watchdog with 1000ms timeout
            scheduler.set_node_watchdog("critical_node", True, 1000)

            # Enable with default timeout
            scheduler.set_node_watchdog("sensor_node", True)

            # Disable watchdog
            scheduler.set_node_watchdog("sensor_node", False)

        Note:
            - Watchdog is automatically fed on successful tick execution
            - Global watchdog_enabled flag must be True in SchedulerConfig
            - Use get_node_stats() to check watchdog_expired status
            - Watchdog expiration is logged but doesn't stop execution
        """
        if not self._scheduler:
            raise RuntimeError("Cannot set watchdog before scheduler is started")
        self._scheduler.set_node_watchdog(node_name, enabled, timeout_ms)


# Convenience functions

def run(*nodes: Node, duration: Optional[float] = None, logging: bool = True) -> None:
    """
    Quick run helper - create scheduler and run nodes.

    Args:
        *nodes: Node instances to run
        duration: Optional duration in seconds
        logging: Enable logging for all nodes (default: True)

    Example:
        node = Node(subs="in", pubs="out", tick=lambda n: n.send("out", n.get("in")))
        run(node, duration=5)

        # Disable logging
        run(node1, node2, duration=10, logging=False)
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

    # Assign priorities: subscribers (0..N), both (N+1..M), publishers (M+1..P)
    priority = 0
    for node in subscribers:
        scheduler.add(node, priority=priority, logging=logging)
        priority += 1
    for node in both:
        scheduler.add(node, priority=priority, logging=logging)
        priority += 1
    for node in publishers:
        scheduler.add(node, priority=priority, logging=logging)
        priority += 1

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
    # horus_library module not available

__all__ = [
    # Core API
    "Node",
    "Scheduler",
    "NodeState",
    "Hub",
    "Link",  # Point-to-point SPSC communication with network support
    "RouterClient",  # Explicit router connection management
    "RouterServer",  # Router server management
    "default_router_endpoint",  # Helper: "topic@router"
    "router_endpoint",  # Helper: "topic@host:port"
    "run",
    # Tensor system for zero-copy ML/AI
    "TensorPool",
    "TensorHandle",
    # Simple async API
    "AsyncNode",
    "AsyncHub",
    "sleep",
    "gather",
    "wait_for",
    # ML utilities
    "MLNodeBase",
    "PyTorchInferenceNode",
    "TensorFlowInferenceNode",
    "ONNXInferenceNode",
    "PerformanceMonitor",
    "preprocess_image_imagenet",
    "preprocess_image_yolo",
    "nms",
    "calculate_iou",
    # Hardware nodes
    "SerialNode",
    "JoystickNode",
    "KeyboardNode",
    "ImuNode",
    "GpsNode",
    "CameraNode",
    "LidarNode",
    # Hardware node data types
    "SerialData",
    "JoystickState",
    "KeyboardState",
    "ImuData",
    "GpsData",
    "ImageData",
    # sim2d API
    "Sim2D",
    "RobotConfigPy",
    "WorldConfigPy",
]

# Import simple async API
from .async_node import AsyncNode, AsyncHub, sleep, gather, wait_for

# Import ML utilities
from .ml_utils import (
    MLNodeBase,
    PyTorchInferenceNode,
    TensorFlowInferenceNode,
    ONNXInferenceNode,
    PerformanceMonitor,
    preprocess_image_imagenet,
    preprocess_image_yolo,
    nms,
    calculate_iou,
)

# Import hardware nodes
from .nodes import (
    # Nodes
    SerialNode,
    JoystickNode,
    KeyboardNode,
    ImuNode,
    GpsNode,
    CameraNode,
    LidarNode,
    # Data types
    SerialData,
    JoystickState,
    KeyboardState,
    ImuData,
    GpsData,
    ImageData,
    LaserScan,
)

# NOTE: Hub is now imported directly from Rust (no alias needed)
# Old: Hub = _PyHub (removed - Hub is imported directly on line 24)

# Add message types to __all__ if available
if _has_messages:
    __all__.extend([
        # Geometry messages
        "Pose2D",
        "Twist",
        "Point3",
        "Vector3",
        "Quaternion",
        "Transform",
        # Control messages
        "CmdVel",
        "MotorCommand",
        "DifferentialDriveCommand",
        "ServoCommand",
        "PwmCommand",
        "StepperCommand",
        "PidConfig",
        # Sensor messages
        "LaserScan",
        "Imu",
        "BatteryState",
        "NavSatFix",
        "Odometry",
        "Range",
        # Diagnostics messages
        "Status",
        "EmergencyStop",
        "Heartbeat",
        "ResourceUsage",
        # Input messages
        "JoystickInput",
        "KeyboardInput",
        # I/O messages
        "DigitalIO",
        "AnalogIO",
    ])
