use std::fmt;
use std::sync::{Arc, Mutex};
use std::time::Instant;

/// Compact logging summary for message types.
///
/// Used by `Topic::with_logging()` and `hlog!` to produce one-line summaries
/// without cloning large data structures.
///
/// # Three ways to get `LogSummary`
///
/// **1. `message!` macro** — auto-generated, field-by-field formatting:
/// ```rust,ignore
/// message! {
///     SensorReading { temperature: f64, humidity: f64 }
/// }
/// // Produces: "SensorReading(temperature=23.5, humidity=0.65)"
/// ```
///
/// **2. `#[derive(LogSummary)]`** — uses `Debug` output (needs `#[derive(Debug)]`):
/// ```rust,ignore
/// #[derive(Debug, LogSummary)]
/// pub struct MyType { pub x: f64 }
/// // Produces: "MyType { x: 23.5 }"
/// ```
///
/// **3. Manual `impl`** — for custom compact summaries (large/zero-copy types):
/// ```rust,ignore
/// impl LogSummary for Image {
///     fn log_summary(&self) -> String {
///         format!("Image({}x{}, {:?})", self.width(), self.height(), self.encoding())
///     }
/// }
/// ```
///
/// # When to use which
///
/// | Approach | Use when |
/// |----------|----------|
/// | `message!` | Defining new message types (default) |
/// | `#[derive(LogSummary)]` | Existing `#[repr(C)]` types with `Debug` |
/// | Manual `impl` | Large types where `Debug` would be too verbose |
pub trait LogSummary {
    /// Return a compact one-line string suitable for logging.
    fn log_summary(&self) -> String;
}

impl LogSummary for crate::types::Tensor {
    fn log_summary(&self) -> String {
        let shape_str: Vec<String> = self.shape().iter().map(|d| d.to_string()).collect();
        format!(
            "Tensor([{}], dtype={:?}, device={}, pool={}/slot={})",
            shape_str.join(", "),
            self.dtype,
            self.device(),
            self.pool_id,
            self.slot_id
        )
    }
}

/// Node states for monitoring and lifecycle management
#[derive(Debug, Clone, PartialEq)]
pub enum NodeState {
    Uninitialized,
    Initializing,
    Running,
    Stopping,
    Stopped,
    Error(String),
    Crashed(String),
}

impl fmt::Display for NodeState {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            NodeState::Uninitialized => write!(f, "Uninitialized"),
            NodeState::Initializing => write!(f, "Initializing"),
            NodeState::Running => write!(f, "Running"),
            NodeState::Stopping => write!(f, "Stopping"),
            NodeState::Stopped => write!(f, "Stopped"),
            NodeState::Error(msg) => write!(f, "Error: {}", msg),
            NodeState::Crashed(msg) => write!(f, "Crashed: {}", msg),
        }
    }
}

/// Node health status for monitoring
#[derive(Debug, Clone, Copy, PartialEq, Eq, serde::Serialize, serde::Deserialize, Default)]
#[repr(u8)]
pub enum HealthStatus {
    /// Operating normally
    Healthy = 0,
    /// Degraded performance (slow ticks, missed deadlines)
    Warning = 1,
    /// Errors occurring but still running
    Error = 2,
    /// Fatal errors, about to crash or unresponsive
    Critical = 3,
    /// Status unknown (no heartbeat received)
    #[default]
    Unknown = 4,
}

impl HealthStatus {
    /// Convert to string representation
    pub fn as_str(&self) -> &'static str {
        match self {
            Self::Healthy => "Healthy",
            Self::Warning => "Warning",
            Self::Error => "Error",
            Self::Critical => "Critical",
            Self::Unknown => "Unknown",
        }
    }

    /// Get color code for monitor display
    pub fn color(&self) -> &'static str {
        match self {
            Self::Healthy => "green",
            Self::Warning => "yellow",
            Self::Error => "orange",
            Self::Critical => "red",
            Self::Unknown => "gray",
        }
    }
}

impl LogSummary for HealthStatus {
    fn log_summary(&self) -> String {
        format!("{:?}", self)
    }
}

/// Network transport status for monitoring
#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
pub struct NetworkStatus {
    /// Node name
    pub node_name: String,
    /// Active transport type (SharedMemory, Udp, BatchUdp, Quic, Unix, etc.)
    pub transport_type: String,
    /// Local endpoint address (if network-based)
    pub local_endpoint: Option<String>,
    /// Remote endpoints this node connects to
    pub remote_endpoints: Vec<String>,
    /// Topics being published over network
    pub network_topics_pub: Vec<String>,
    /// Topics being subscribed over network
    pub network_topics_sub: Vec<String>,
    /// Bytes sent
    pub bytes_sent: u64,
    /// Bytes received
    pub bytes_received: u64,
    /// Packets sent
    pub packets_sent: u64,
    /// Packets received
    pub packets_received: u64,
    /// Last update timestamp (Unix seconds)
    pub timestamp_secs: u64,
}

impl NetworkStatus {
    /// Create a new network status
    pub fn new(node_name: &str, transport_type: &str) -> Self {
        let now = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap_or_default()
            .as_secs();

        Self {
            node_name: node_name.to_string(),
            transport_type: transport_type.to_string(),
            local_endpoint: None,
            remote_endpoints: Vec::new(),
            network_topics_pub: Vec::new(),
            network_topics_sub: Vec::new(),
            bytes_sent: 0,
            bytes_received: 0,
            packets_sent: 0,
            packets_received: 0,
            timestamp_secs: now,
        }
    }

    /// Check if status is fresh (within last N seconds)
    pub fn is_fresh(&self, max_age_secs: u64) -> bool {
        let now = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap_or_default()
            .as_secs();

        now.saturating_sub(self.timestamp_secs) <= max_age_secs
    }
}

/// Performance metrics for node execution
#[derive(Debug, Clone, Default)]
pub struct NodeMetrics {
    /// Node name
    pub name: String,
    /// Execution order (lower = earlier in tick sequence)
    pub order: u32,
    pub total_ticks: u64,
    pub successful_ticks: u64,
    pub failed_ticks: u64,
    pub avg_tick_duration_ms: f64,
    pub max_tick_duration_ms: f64,
    pub min_tick_duration_ms: f64,
    pub last_tick_duration_ms: f64,
    pub messages_sent: u64,
    pub messages_received: u64,
    pub errors_count: u64,
    pub warnings_count: u64,
    pub uptime_seconds: f64,
}

impl NodeMetrics {
    /// Reset timing-related metrics for restart (preserves counts)
    pub(crate) fn reset_timing(&mut self) {
        self.avg_tick_duration_ms = 0.0;
        self.max_tick_duration_ms = 0.0;
        self.min_tick_duration_ms = 0.0;
        self.last_tick_duration_ms = 0.0;
    }

    /// Calculate health status from metrics
    pub(crate) fn calculate_health(&self) -> HealthStatus {
        if self.errors_count > 10 {
            HealthStatus::Critical
        } else if self.errors_count > 3 {
            HealthStatus::Error
        } else if self.failed_ticks > 0 || self.avg_tick_duration_ms > 100.0 {
            HealthStatus::Warning
        } else {
            HealthStatus::Healthy
        }
    }
}

/// Internal scheduler context for tracking node state and metrics.
pub struct NodeInfo {
    // Identity
    name: String,

    // State management
    state: NodeState,
    previous_state: NodeState,
    state_change_time: Instant,

    // Performance tracking
    metrics: NodeMetrics,

    // Timing information
    creation_time: Instant,
    last_tick_time: Option<Instant>,
    tick_start_time: Option<Instant>,

    // Lifecycle management
    restart_count: u32,
    error_history: Vec<(Instant, String)>,
    warning_history: Vec<(Instant, String)>,

    // Custom data store
    custom_data: std::collections::HashMap<String, String>,

    // Thread safety for metrics updates
    metrics_lock: Arc<Mutex<()>>,

    // Event notification counter — bumped by publishers to trigger event-driven nodes
    event_notifier: Option<Arc<std::sync::atomic::AtomicU64>>,
}

impl NodeInfo {
    /// Create a new NodeInfo
    pub fn new(node_name: String) -> Self {
        let now = Instant::now();

        Self {
            name: node_name,
            state: NodeState::Uninitialized,
            previous_state: NodeState::Uninitialized,
            state_change_time: now,
            metrics: NodeMetrics::default(),
            creation_time: now,
            last_tick_time: None,
            tick_start_time: None,
            restart_count: 0,
            error_history: Vec::new(),
            warning_history: Vec::new(),
            custom_data: std::collections::HashMap::new(),
            metrics_lock: Arc::new(Mutex::new(())),
            event_notifier: None,
        }
    }

    // State Management Methods
    pub fn state(&self) -> &NodeState {
        &self.state
    }

    pub fn set_state(&mut self, new_state: NodeState) {
        if self.state != new_state {
            self.previous_state = self.state.clone();
            self.state = new_state;
            self.state_change_time = Instant::now();
        }
    }

    pub fn transition_to_error(&mut self, error_msg: String) {
        crate::hlog!(error, "{}", error_msg);
        self.track_error(&error_msg);
        self.set_state(NodeState::Error(error_msg));
    }

    pub fn transition_to_crashed(&mut self, crash_msg: String) {
        crate::hlog!(error, "{}", crash_msg);
        self.track_error(&crash_msg);
        self.set_state(NodeState::Crashed(crash_msg));
    }

    pub fn transition_to_stopped(&mut self) {
        self.set_state(NodeState::Stopped);
    }

    // Lifecycle Methods
    pub fn initialize(&mut self) -> crate::error::HorusResult<()> {
        self.set_state(NodeState::Initializing);
        // Initialization logic can be added here
        self.set_state(NodeState::Running);
        Ok(())
    }

    pub fn shutdown(&mut self) -> crate::error::HorusResult<()> {
        self.set_state(NodeState::Stopping);
        // Cleanup logic can be added here
        self.set_state(NodeState::Stopped);
        Ok(())
    }

    /// Reset node context for restart (preserves identity, clears runtime state)
    pub fn reset_for_restart(&mut self) {
        self.restart_count += 1;
        self.state = NodeState::Uninitialized;
        self.previous_state = NodeState::Stopped;
        self.state_change_time = Instant::now();
        self.last_tick_time = None;
        self.tick_start_time = None;
        // Keep metrics history but reset tick timing
        self.metrics.reset_timing();
    }

    pub fn restart(&mut self) -> crate::error::HorusResult<()> {
        self.restart_count += 1;
        self.shutdown()?;
        self.initialize()?;
        Ok(())
    }

    // Tick Management
    pub fn start_tick(&mut self) {
        self.tick_start_time = Some(Instant::now());
        if self.state == NodeState::Uninitialized {
            let _ = self.initialize();
        }
    }

    pub fn record_tick(&mut self) {
        let _guard = self
            .metrics_lock
            .lock()
            .unwrap_or_else(|poisoned| poisoned.into_inner());

        if let Some(start_time) = self.tick_start_time {
            let duration = start_time.elapsed();
            let duration_ms = duration.as_millis() as f64;

            self.metrics.total_ticks += 1;
            self.metrics.successful_ticks += 1;
            self.metrics.last_tick_duration_ms = duration_ms;

            // Update min/max duration
            if self.metrics.min_tick_duration_ms == 0.0
                || duration_ms < self.metrics.min_tick_duration_ms
            {
                self.metrics.min_tick_duration_ms = duration_ms;
            }
            if duration_ms > self.metrics.max_tick_duration_ms {
                self.metrics.max_tick_duration_ms = duration_ms;
            }

            // Update average duration
            let total_duration =
                self.metrics.avg_tick_duration_ms * (self.metrics.successful_ticks - 1) as f64;
            self.metrics.avg_tick_duration_ms =
                (total_duration + duration_ms) / self.metrics.successful_ticks as f64;

            self.last_tick_time = Some(Instant::now());
            self.tick_start_time = None;

            // Update uptime
            self.metrics.uptime_seconds = self.creation_time.elapsed().as_secs_f64();
        }
    }

    /// Record node shutdown
    pub fn record_shutdown(&mut self) {
        self.transition_to_stopped();
    }

    pub fn record_tick_failure(&mut self, error_msg: String) {
        {
            let _guard = self
                .metrics_lock
                .lock()
                .unwrap_or_else(|poisoned| poisoned.into_inner());
            self.metrics.total_ticks += 1;
            self.metrics.failed_ticks += 1;

            if let Some(start_time) = self.tick_start_time {
                let duration = start_time.elapsed();
                self.metrics.last_tick_duration_ms = duration.as_millis() as f64;
                self.tick_start_time = None;
            }
        }

        crate::hlog!(error, "{}", error_msg);
        self.track_error(&error_msg);
    }

    /// Get elapsed time since tick started in microseconds
    pub fn tick_elapsed_us(&self) -> u64 {
        self.tick_start_time
            .map(|t| t.elapsed().as_micros() as u64)
            .unwrap_or(0)
    }

    /// Track a warning for metrics (history + count). Use hlog!(warn, ...) for logging.
    pub fn track_warning(&mut self, message: &str) {
        self.warning_history
            .push((Instant::now(), message.to_string()));
        if self.warning_history.len() > 100 {
            self.warning_history.remove(0);
        }
        self.metrics.warnings_count += 1;
    }

    /// Track an error for metrics (history + count). Use hlog!(error, ...) for logging.
    pub fn track_error(&mut self, message: &str) {
        self.error_history
            .push((Instant::now(), message.to_string()));
        if self.error_history.len() > 100 {
            self.error_history.remove(0);
        }
        self.metrics.errors_count += 1;
    }

    // Getters
    pub fn name(&self) -> &str {
        &self.name
    }
    pub fn metrics(&self) -> &NodeMetrics {
        &self.metrics
    }
    pub fn uptime(&self) -> std::time::Duration {
        self.creation_time.elapsed()
    }

    // Custom data management
    pub fn set_custom_data(&mut self, key: String, value: String) {
        self.custom_data.insert(key, value);
    }

    pub fn get_custom_data(&self, key: &str) -> Option<&String> {
        self.custom_data.get(key)
    }

    // Event notification methods

    /// Set the event notifier for this node (used by EventExecutor).
    ///
    /// The notifier is an atomic counter that publishers bump to signal new data.
    /// The event watcher thread checks this counter to decide when to tick.
    pub fn set_event_notifier(&mut self, notifier: Arc<std::sync::atomic::AtomicU64>) {
        // Also register in the global registry for external notification
        EVENT_NOTIFIER_REGISTRY
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .insert(self.name.clone(), notifier.clone());
        self.event_notifier = Some(notifier);
    }

    /// Notify an event node by name (bumps its generation counter).
    ///
    /// This is the primary way to trigger an event-driven node from external code
    /// or from a topic publisher. Returns `true` if the node was found.
    pub fn notify_event(node_name: &str) -> bool {
        let registry = EVENT_NOTIFIER_REGISTRY
            .lock()
            .unwrap_or_else(|p| p.into_inner());
        if let Some(notifier) = registry.get(node_name) {
            notifier.fetch_add(1, std::sync::atomic::Ordering::Release);
            true
        } else {
            false
        }
    }
}

/// Global registry of event notifiers keyed by node name.
///
/// Allows external code (topic publishers, tests) to notify event-driven nodes
/// without holding a direct reference to their NodeInfo.
static EVENT_NOTIFIER_REGISTRY: std::sync::LazyLock<
    Mutex<std::collections::HashMap<String, Arc<std::sync::atomic::AtomicU64>>>,
> = std::sync::LazyLock::new(|| Mutex::new(std::collections::HashMap::new()));

/// Topic metadata for monitoring and introspection
#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
pub struct TopicMetadata {
    pub topic_name: String,
    pub type_name: String,
}

/// Comprehensive trait for Horus nodes with full lifecycle support
///
/// # Logging
///
/// Use the `hlog!()` macro for logging within any lifecycle method:
///
/// ```ignore
/// use horus::hlog;
///
/// fn init(&mut self) -> HorusResult<()> {
///     hlog!(info, "Initializing...");
///     Ok(())
/// }
///
/// fn tick(&mut self) {
///     hlog!(debug, "Processing tick");
/// }
/// ```
pub trait Node: Send {
    /// Get the node's name (must be unique within a scheduler).
    ///
    /// Defaults to the struct's type name (e.g. `MotorController`).
    /// Override for a custom name — string literals and `&self.name` both work.
    fn name(&self) -> &str {
        let full = std::any::type_name::<Self>();
        match full.rfind("::") {
            Some(pos) => &full[pos + 2..],
            None => full,
        }
    }

    /// Initialize the node (called once at startup).
    ///
    /// Use `hlog!()` for logging instead of the old ctx parameter.
    fn init(&mut self) -> crate::error::HorusResult<()> {
        Ok(())
    }

    /// Main execution loop (called repeatedly)
    fn tick(&mut self);

    /// Shutdown the node (called once at cleanup).
    ///
    /// Use `hlog!()` for logging instead of the old ctx parameter.
    fn shutdown(&mut self) -> crate::error::HorusResult<()> {
        Ok(())
    }

    /// Get list of publishers (topic metadata)
    ///
    /// **Important:** Override this if your node publishes to any topics.
    /// The default returns empty, which may hide connectivity issues during debugging.
    fn publishers(&self) -> Vec<TopicMetadata> {
        Vec::new()
    }

    /// Get list of subscribers (topic metadata)
    ///
    /// **Important:** Override this if your node subscribes to any topics.
    /// The default returns empty, which may hide connectivity issues during debugging.
    fn subscribers(&self) -> Vec<TopicMetadata> {
        Vec::new()
    }

    /// Handle errors (optional override).
    ///
    /// Use `hlog!()` for logging instead of the old ctx parameter.
    fn on_error(&mut self, error: &str) {
        crate::hlog!(error, "Node error: {}", error);
    }

    /// Check if node is in safe state (for safety monitor).
    fn is_safe_state(&self) -> bool {
        true
    }

    /// Transition to safe state (for emergency stop).
    fn enter_safe_state(&mut self) {
        // Default: no-op
    }
}

// LogSummary implementations for primitive types
impl LogSummary for f32 {
    fn log_summary(&self) -> String {
        format!("{:.3}", self)
    }
}

impl LogSummary for f64 {
    fn log_summary(&self) -> String {
        format!("{:.3}", self)
    }
}

impl LogSummary for i32 {
    fn log_summary(&self) -> String {
        self.to_string()
    }
}

impl LogSummary for i64 {
    fn log_summary(&self) -> String {
        self.to_string()
    }
}

impl LogSummary for u32 {
    fn log_summary(&self) -> String {
        self.to_string()
    }
}

impl LogSummary for u64 {
    fn log_summary(&self) -> String {
        self.to_string()
    }
}

impl LogSummary for usize {
    fn log_summary(&self) -> String {
        self.to_string()
    }
}

impl LogSummary for bool {
    fn log_summary(&self) -> String {
        self.to_string()
    }
}

impl LogSummary for String {
    fn log_summary(&self) -> String {
        self.clone()
    }
}

impl<T: fmt::Debug> LogSummary for Vec<T> {
    fn log_summary(&self) -> String {
        format!("Vec[{} items]", self.len())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    // =========================================================================
    // NodeState Tests
    // =========================================================================

    #[test]
    fn test_node_state_display() {
        assert_eq!(NodeState::Uninitialized.to_string(), "Uninitialized");
        assert_eq!(NodeState::Initializing.to_string(), "Initializing");
        assert_eq!(NodeState::Running.to_string(), "Running");
        assert_eq!(NodeState::Stopping.to_string(), "Stopping");
        assert_eq!(NodeState::Stopped.to_string(), "Stopped");
        assert_eq!(
            NodeState::Error("test error".to_string()).to_string(),
            "Error: test error"
        );
        assert_eq!(
            NodeState::Crashed("crash reason".to_string()).to_string(),
            "Crashed: crash reason"
        );
    }

    #[test]
    fn test_node_state_equality() {
        assert_eq!(NodeState::Running, NodeState::Running);
        assert_ne!(NodeState::Running, NodeState::Stopping);
        assert_eq!(
            NodeState::Error("msg".to_string()),
            NodeState::Error("msg".to_string())
        );
        assert_ne!(
            NodeState::Error("msg1".to_string()),
            NodeState::Error("msg2".to_string())
        );
    }

    #[test]
    fn test_node_state_clone() {
        let state = NodeState::Error("test".to_string());
        let cloned = state.clone();
        assert_eq!(state, cloned);
    }

    // =========================================================================
    // HealthStatus Tests
    // =========================================================================

    #[test]
    fn test_health_status_as_str() {
        assert_eq!(HealthStatus::Healthy.as_str(), "Healthy");
        assert_eq!(HealthStatus::Warning.as_str(), "Warning");
        assert_eq!(HealthStatus::Error.as_str(), "Error");
        assert_eq!(HealthStatus::Critical.as_str(), "Critical");
        assert_eq!(HealthStatus::Unknown.as_str(), "Unknown");
    }

    #[test]
    fn test_health_status_color() {
        assert_eq!(HealthStatus::Healthy.color(), "green");
        assert_eq!(HealthStatus::Warning.color(), "yellow");
        assert_eq!(HealthStatus::Error.color(), "orange");
        assert_eq!(HealthStatus::Critical.color(), "red");
        assert_eq!(HealthStatus::Unknown.color(), "gray");
    }

    #[test]
    fn test_health_status_equality() {
        assert_eq!(HealthStatus::Healthy, HealthStatus::Healthy);
        assert_ne!(HealthStatus::Healthy, HealthStatus::Warning);
    }

    #[test]
    fn test_health_status_copy() {
        let status = HealthStatus::Healthy;
        let copied = status;
        assert_eq!(status, copied);
    }

    // =========================================================================
    // NodeMetrics Tests
    // =========================================================================

    #[test]
    fn test_node_metrics_default() {
        let metrics = NodeMetrics::default();
        assert_eq!(metrics.total_ticks, 0);
        assert_eq!(metrics.successful_ticks, 0);
        assert_eq!(metrics.failed_ticks, 0);
        assert_eq!(metrics.avg_tick_duration_ms, 0.0);
        assert_eq!(metrics.messages_sent, 0);
        assert_eq!(metrics.messages_received, 0);
        assert_eq!(metrics.errors_count, 0);
    }

    #[test]
    fn test_node_metrics_clone() {
        let metrics = NodeMetrics {
            total_ticks: 100,
            successful_ticks: 95,
            failed_ticks: 5,
            avg_tick_duration_ms: 10.5,
            max_tick_duration_ms: 50.0,
            min_tick_duration_ms: 1.0,
            last_tick_duration_ms: 8.0,
            messages_sent: 500,
            messages_received: 300,
            errors_count: 2,
            warnings_count: 10,
            uptime_seconds: 3600.0,
            ..Default::default()
        };
        let cloned = metrics.clone();
        assert_eq!(cloned.total_ticks, 100);
        assert_eq!(cloned.successful_ticks, 95);
        assert_eq!(cloned.avg_tick_duration_ms, 10.5);
    }

    // =========================================================================
    // Health Calculation Tests
    // =========================================================================

    #[test]
    fn test_calculate_health_healthy() {
        let metrics = NodeMetrics {
            total_ticks: 100,
            successful_ticks: 100,
            failed_ticks: 0,
            avg_tick_duration_ms: 10.0,
            errors_count: 0,
            ..NodeMetrics::default()
        };
        assert_eq!(metrics.calculate_health(), HealthStatus::Healthy);
    }

    #[test]
    fn test_calculate_health_warning() {
        let metrics = NodeMetrics {
            failed_ticks: 5, // > 0 failed ticks triggers warning
            ..NodeMetrics::default()
        };
        assert_eq!(metrics.calculate_health(), HealthStatus::Warning);
    }

    #[test]
    fn test_calculate_health_error() {
        let metrics = NodeMetrics {
            errors_count: 5, // > 3 triggers error
            ..NodeMetrics::default()
        };
        assert_eq!(metrics.calculate_health(), HealthStatus::Error);
    }

    #[test]
    fn test_calculate_health_critical() {
        let metrics = NodeMetrics {
            errors_count: 15, // > 10 triggers critical
            ..NodeMetrics::default()
        };
        assert_eq!(metrics.calculate_health(), HealthStatus::Critical);
    }

    // =========================================================================
    // NetworkStatus Tests
    // =========================================================================

    #[test]
    fn test_network_status_new() {
        let status = NetworkStatus::new("test_node", "SharedMemory");
        assert_eq!(status.node_name, "test_node");
        assert_eq!(status.transport_type, "SharedMemory");
        assert!(status.local_endpoint.is_none());
        assert!(status.remote_endpoints.is_empty());
        assert_eq!(status.bytes_sent, 0);
        assert_eq!(status.bytes_received, 0);
    }

    #[test]
    fn test_network_status_is_fresh() {
        let status = NetworkStatus::new("test_node", "Udp");
        // Just created, should be fresh
        assert!(status.is_fresh(5));
    }

    #[test]
    fn test_network_status_clone() {
        let mut status = NetworkStatus::new("test_node", "Udp");
        status.bytes_sent = 1000;
        status.bytes_received = 500;
        status.remote_endpoints.push("192.168.1.1:9000".to_string());

        let cloned = status.clone();
        assert_eq!(cloned.bytes_sent, 1000);
        assert_eq!(cloned.remote_endpoints.len(), 1);
    }

    // =========================================================================
    // NodeInfo Tests
    // =========================================================================

    #[test]
    fn test_node_info_new() {
        let info = NodeInfo::new("test_node".to_string());
        assert_eq!(info.name(), "test_node");
        assert_eq!(info.state(), &NodeState::Uninitialized);
    }

    #[test]
    fn test_node_info_state_transitions() {
        let mut info = NodeInfo::new("test_node".to_string());

        assert_eq!(info.state(), &NodeState::Uninitialized);

        info.set_state(NodeState::Initializing);
        assert_eq!(info.state(), &NodeState::Initializing);

        info.set_state(NodeState::Running);
        assert_eq!(info.state(), &NodeState::Running);

        info.set_state(NodeState::Stopping);
        assert_eq!(info.state(), &NodeState::Stopping);

        info.set_state(NodeState::Stopped);
        assert_eq!(info.state(), &NodeState::Stopped);
    }

    #[test]
    fn test_node_info_metrics_initial() {
        let info = NodeInfo::new("test_node".to_string());
        let metrics = info.metrics();
        assert_eq!(metrics.total_ticks, 0);
        assert_eq!(metrics.successful_ticks, 0);
        assert_eq!(metrics.failed_ticks, 0);
    }

    #[test]
    fn test_node_info_error_tracking() {
        let mut info = NodeInfo::new("test_node".to_string());

        info.track_error("Test error 1");
        info.track_error("Test error 2");

        let metrics = info.metrics();
        assert_eq!(metrics.errors_count, 2);
    }

    #[test]
    fn test_node_info_transition_to_error() {
        let mut info = NodeInfo::new("test_node".to_string());
        info.transition_to_error("Something went wrong".to_string());
        assert!(matches!(info.state(), &NodeState::Error(_)));
    }

    #[test]
    fn test_node_info_transition_to_crashed() {
        let mut info = NodeInfo::new("test_node".to_string());
        info.transition_to_crashed("Fatal error".to_string());
        assert!(matches!(info.state(), &NodeState::Crashed(_)));
    }

    #[test]
    fn test_node_info_initialize_and_shutdown() {
        let mut info = NodeInfo::new("test_node".to_string());
        assert_eq!(info.state(), &NodeState::Uninitialized);

        info.initialize().unwrap();
        assert_eq!(info.state(), &NodeState::Running);

        info.shutdown().unwrap();
        assert_eq!(info.state(), &NodeState::Stopped);
    }

    // =========================================================================
    // LogSummary Tests
    // =========================================================================

    #[test]
    fn test_log_summary_f32() {
        assert_eq!(std::f32::consts::PI.log_summary(), "3.142");
        assert_eq!(0.0f32.log_summary(), "0.000");
    }

    #[test]
    fn test_log_summary_f64() {
        assert_eq!(std::f64::consts::PI.log_summary(), "3.142");
    }

    #[test]
    fn test_log_summary_integers() {
        assert_eq!(42i32.log_summary(), "42");
        assert_eq!(42i64.log_summary(), "42");
        assert_eq!(42u32.log_summary(), "42");
        assert_eq!(42u64.log_summary(), "42");
        assert_eq!(42usize.log_summary(), "42");
    }

    #[test]
    fn test_log_summary_bool() {
        assert_eq!(true.log_summary(), "true");
        assert_eq!(false.log_summary(), "false");
    }

    #[test]
    fn test_log_summary_string() {
        assert_eq!("hello".to_string().log_summary(), "hello");
    }

    #[test]
    fn test_log_summary_vec() {
        let v: Vec<i32> = vec![1, 2, 3, 4, 5];
        assert_eq!(v.log_summary(), "Vec[5 items]");

        let empty: Vec<i32> = vec![];
        assert_eq!(empty.log_summary(), "Vec[0 items]");
    }
}
