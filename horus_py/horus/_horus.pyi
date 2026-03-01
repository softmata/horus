"""
Type stubs for horus._horus module

This file provides type hints for IDE autocomplete and static type checking.
"""

from typing import Any, Dict, Optional

class PyNodeInfo:
    """
    Python wrapper for NodeInfo - provides node state, metrics, and logging.

    This class is used internally by the scheduler to track node execution
    and provide logging capabilities to nodes during their tick cycle.
    """

    def __init__(self, name: str) -> None:
        """
        Create a new NodeInfo instance.

        Args:
            name: Unique node name
        """
        ...

    @property
    def name(self) -> str:
        """Get the node name."""
        ...

    @property
    def state(self) -> PyNodeState:
        """Get the current node state."""
        ...

    def log_info(self, message: str) -> None:
        """
        Log an info message.

        Logs are written to stdout and the shared memory log buffer
        for monitor visibility.

        Args:
            message: Message to log
        """
        ...

    def log_warning(self, message: str) -> None:
        """
        Log a warning message.

        Logs are written to stdout and the shared memory log buffer
        for monitor visibility.

        Args:
            message: Warning message to log
        """
        ...

    def log_error(self, message: str) -> None:
        """
        Log an error message.

        Logs are written to stdout and the shared memory log buffer
        for monitor visibility.

        Args:
            message: Error message to log
        """
        ...

    def log_debug(self, message: str) -> None:
        """
        Log a debug message.

        Logs are written to stdout and the shared memory log buffer
        for monitor visibility.

        Args:
            message: Debug message to log
        """
        ...

    def log_pub(self, topic: str, data_repr: str, ipc_ns: int) -> None:
        """
        Log a publish operation with IPC timing.

        Args:
            topic: Topic name
            data_repr: String representation of published data
            ipc_ns: IPC operation latency in nanoseconds
        """
        ...

    def log_sub(self, topic: str, data_repr: str, ipc_ns: int) -> None:
        """
        Log a subscribe operation with IPC timing.

        Args:
            topic: Topic name
            data_repr: String representation of received data
            ipc_ns: IPC operation latency in nanoseconds
        """
        ...

    def get_metrics(self) -> Dict[str, float]:
        """
        Get comprehensive node metrics.

        Returns:
            Dictionary containing:
            - total_ticks: Total number of ticks executed
            - successful_ticks: Ticks completed without errors
            - failed_ticks: Ticks that resulted in errors
            - errors_count: Total number of errors
            - avg_tick_duration_ms: Average tick duration in milliseconds
            - min_tick_duration_ms: Minimum tick duration
            - max_tick_duration_ms: Maximum tick duration
            - last_tick_duration_ms: Most recent tick duration
        """
        ...

    def get_uptime(self) -> float:
        """
        Get node uptime in seconds.

        Returns:
            Time in seconds since node was created
        """
        ...

    def avg_tick_duration_ms(self) -> float:
        """
        Get average tick duration in milliseconds.

        Returns:
            Average tick duration
        """
        ...

    def tick_count(self) -> int:
        """
        Get total tick count.

        Returns:
            Total number of ticks executed
        """
        ...

    def error_count(self) -> int:
        """
        Get total error count.

        Returns:
            Total number of errors encountered
        """
        ...

    def successful_ticks(self) -> int:
        """
        Get successful tick count.

        Returns:
            Number of ticks completed without errors
        """
        ...

    def failed_ticks(self) -> int:
        """
        Get failed tick count.

        Returns:
            Number of ticks that resulted in errors
        """
        ...

    def transition_to_error(self, error_msg: str) -> None:
        """
        Transition node to ERROR state.

        Args:
            error_msg: Error message describing why the transition occurred
        """
        ...

    def get_custom_data(self, key: str) -> Optional[str]:
        """
        Get custom data by key.

        Args:
            key: Data key

        Returns:
            Value if key exists, None otherwise
        """
        ...

    def set_custom_data(self, key: str, value: str) -> None:
        """
        Set custom data by key.

        Args:
            key: Data key
            value: Data value
        """
        ...


class PyNodeState:
    """
    Node state enum.

    Possible states:
    - UNINITIALIZED: Node created but not initialized
    - INITIALIZING: Node initialization in progress
    - RUNNING: Node actively running
    - PAUSED: Node paused
    - STOPPING: Node shutdown in progress
    - STOPPED: Node stopped
    - ERROR: Node in error state
    - CRASHED: Node crashed unexpectedly
    """

    UNINITIALIZED: str
    INITIALIZING: str
    RUNNING: str
    PAUSED: str
    STOPPING: str
    STOPPED: str
    ERROR: str
    CRASHED: str

    @property
    def name(self) -> str:
        """Get the state name."""
        ...


class PyScheduler:
    """
    Python wrapper for HORUS Scheduler.

    Manages node execution, lifecycle, and coordination.
    Supports per-node rate control for flexible scheduling.
    """

    def __init__(self, config: Optional['PySchedulerConfig'] = None) -> None:
        """Create a new Scheduler with optional config."""
        ...

    @staticmethod
    def deploy() -> 'PyScheduler':
        """Production deployment: RT scheduling, memory locking, blackbox, profiling."""
        ...

    @staticmethod
    def safety_critical() -> 'PyScheduler':
        """Safety-critical: 1kHz, WCET enforcement, watchdog, safety monitor."""
        ...

    @staticmethod
    def high_performance() -> 'PyScheduler':
        """High-performance: parallel execution, 10kHz, WCET, NUMA-aware."""
        ...

    @staticmethod
    def hard_realtime() -> 'PyScheduler':
        """Hard real-time: parallel, 1kHz, full RT, watchdog, safety monitor, blackbox."""
        ...

    @staticmethod
    def deterministic() -> 'PyScheduler':
        """Deterministic execution for simulation and replay."""
        ...

    def add(
        self,
        node: Any,
        priority: int = 100,
        rate_hz: Optional[float] = None,
        rt: bool = False,
        deadline_ms: Optional[float] = None
    ) -> None:
        """
        Add a node to the scheduler.

        Args:
            node: Node instance (must have tick/init/shutdown methods)
            priority: Priority level (lower = higher priority, 0 = highest)
            rate_hz: Optional per-node rate in Hz (uses global rate if None)
            rt: Mark as real-time node (default: False)
            deadline_ms: Soft deadline in milliseconds (default: None)
        """
        ...

    def set_node_rate(self, node_name: str, rate_hz: float) -> None:
        """
        Set per-node tick rate.

        Args:
            node_name: Name of the node
            rate_hz: Target rate in Hz (must be 0-10000)
        """
        ...

    def run(self) -> None:
        """Run the scheduler indefinitely until stop() is called."""
        ...

    def run_for(self, duration: float) -> None:
        """
        Run the scheduler for a specific duration.

        Args:
            duration: Duration in seconds
        """
        ...

    def stop(self) -> None:
        """Stop the scheduler gracefully."""
        ...

    def get_node_stats(self, node_name: str) -> Dict[str, Any]:
        """
        Get statistics for a specific node.

        Args:
            node_name: Name of the node

        Returns:
            Dictionary with node stats:
            - name: Node name
            - priority: Priority level
            - total_ticks: Total ticks executed
            - errors_count: Number of errors
        """
        ...

    def get_node_info(self, node_name: str) -> Optional[PyNodeInfo]:
        """
        Get NodeInfo for a specific node.

        Args:
            node_name: Name of the node

        Returns:
            PyNodeInfo instance if node exists, None otherwise
        """
        ...

    def is_running(self) -> bool:
        """
        Check if scheduler is currently running.

        Returns:
            True if running, False otherwise
        """
        ...

    def remove_node(self, node_name: str) -> bool:
        """
        Remove a node from the scheduler.

        Args:
            node_name: Name of the node to remove

        Returns:
            True if removed, False if node not found
        """
        ...

    def tick(self) -> None:
        """Execute one scheduler tick (internal use)."""
        ...

    def tick_for(self, duration: float) -> None:
        """
        Execute ticks for a specific duration (internal use).

        Args:
            duration: Duration in seconds
        """
        ...


class PyNode:
    """
    Internal Python wrapper for HORUS Node (legacy).

    Note: Most users should use the horus.Node class from the Python
    wrapper instead, which provides a more Pythonic API.
    """

    def __init__(self, name: str) -> None:
        """
        Create a new Node.

        Args:
            name: Node name
        """
        ...

    @property
    def name(self) -> str:
        """Get the node name."""
        ...

    def tick(self, info: Optional[PyNodeInfo] = None) -> None:
        """
        Execute one tick.

        Args:
            info: Optional NodeInfo context
        """
        ...

    def init(self, info: Optional[PyNodeInfo] = None) -> None:
        """
        Initialize the node.

        Args:
            info: Optional NodeInfo context
        """
        ...

    def shutdown(self, info: Optional[PyNodeInfo] = None) -> None:
        """
        Shutdown the node.

        Args:
            info: Optional NodeInfo context
        """
        ...

    def set_callback(self, callback: Any) -> None:
        """
        Set tick callback.

        Args:
            callback: Callable to execute on each tick
        """
        ...


def get_version() -> str:
    """
    Get HORUS version information.

    Returns:
        Version string
    """
    ...
