use crate::params::RuntimeParams;
use crate::scheduling::fault_tolerance::CircuitBreaker;
use std::collections::HashMap;
use std::fmt;
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

/// Trait for providing lightweight logging summaries of message types
///
/// This trait allows large data structures (images, point clouds) to provide
/// compact string representations for logging without cloning the entire data.
///
/// For small types: implementation can use Debug formatting
/// For large types: implementation should only include metadata
pub trait LogSummary {
    /// Return a compact string representation suitable for logging
    fn log_summary(&self) -> String;
}

impl LogSummary for horus_types::HorusTensor {
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
    Paused,
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
            NodeState::Paused => write!(f, "Paused"),
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

    /// Read all network statuses from the network directory
    pub fn read_all() -> Vec<Self> {
        use crate::memory::platform::shm_network_dir;

        let dir = shm_network_dir();
        if !dir.exists() {
            return Vec::new();
        }

        let mut statuses = Vec::new();
        if let Ok(entries) = std::fs::read_dir(&dir) {
            for entry in entries.flatten() {
                if let Ok(content) = std::fs::read_to_string(entry.path()) {
                    if let Ok(status) = serde_json::from_str::<NetworkStatus>(&content) {
                        statuses.push(status);
                    }
                }
            }
        }
        statuses
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
    pub fn reset_timing(&mut self) {
        self.avg_tick_duration_ms = 0.0;
        self.max_tick_duration_ms = 0.0;
        self.min_tick_duration_ms = 0.0;
        self.last_tick_duration_ms = 0.0;
    }

    /// Calculate health status from metrics
    pub fn calculate_health(&self) -> HealthStatus {
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

/// Configuration parameters for node behavior
#[derive(Debug, Clone)]
pub struct NodeConfig {
    pub max_tick_duration_ms: Option<u64>,
    pub restart_on_failure: bool,
    pub max_restart_attempts: u32,
    pub restart_delay_ms: u64,
    pub log_level: String,
    pub custom_params: HashMap<String, String>,
    /// Optional per-node circuit breaker configuration
    /// If set, overrides the scheduler-level circuit breaker settings for this node
    pub circuit_breaker: Option<CircuitBreaker>,
}

impl Default for NodeConfig {
    fn default() -> Self {
        NodeConfig {
            max_tick_duration_ms: Some(1000), // 1 second timeout
            restart_on_failure: true,
            max_restart_attempts: 3,
            restart_delay_ms: 1000,
            log_level: "INFO".to_string(), // Development default: includes info logging
            custom_params: HashMap::new(),
            circuit_breaker: None,
        }
    }
}

/// Comprehensive context and information for Horus nodes
pub struct NodeInfo {
    // Basic identification
    name: String,
    node_id: String,
    instance_id: String,

    // State management
    state: NodeState,
    previous_state: NodeState,
    state_change_time: Instant,

    // Configuration
    config: NodeConfig,
    priority: u32,

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

    // Debugging
    custom_data: HashMap<String, String>,

    // Thread safety for metrics updates
    metrics_lock: Arc<Mutex<()>>,

    // Runtime parameters
    pub params: RuntimeParams,
}

impl NodeInfo {
    /// Create a new NodeInfo with comprehensive initialization
    pub fn new(node_name: String) -> Self {
        let now = Instant::now();
        let node_id = format!(
            "{}_{}",
            node_name,
            std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap_or_default()
                .as_millis()
        );

        let config = NodeConfig::default();

        Self {
            name: node_name.clone(),
            node_id,
            instance_id: uuid::Uuid::new_v4().to_string(),
            state: NodeState::Uninitialized,
            previous_state: NodeState::Uninitialized,
            state_change_time: now,
            config,
            priority: 50, // Normal priority (middle range)
            metrics: NodeMetrics::default(),
            creation_time: now,
            last_tick_time: None,
            tick_start_time: None,
            restart_count: 0,
            error_history: Vec::new(),
            warning_history: Vec::new(),
            custom_data: HashMap::new(),
            metrics_lock: Arc::new(Mutex::new(())),
            params: RuntimeParams::default(),
        }
    }

    /// Create NodeInfo with custom configuration
    pub fn new_with_config(node_name: String, config: NodeConfig) -> Self {
        let mut node_info = Self::new(node_name);
        node_info.config = config;
        node_info
    }

    // State Management Methods
    pub fn state(&self) -> &NodeState {
        &self.state
    }

    pub fn previous_state(&self) -> &NodeState {
        &self.previous_state
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
        if self.restart_count > self.config.max_restart_attempts {
            return Err(crate::error::HorusError::InvalidInput(
                "Maximum restart attempts exceeded".to_string(),
            ));
        }

        self.shutdown()?;
        std::thread::sleep(Duration::from_millis(self.config.restart_delay_ms));
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
        if let Some(start_time) = self.tick_start_time {
            start_time.elapsed().as_micros() as u64
        } else {
            0
        }
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
    pub fn node_id(&self) -> &str {
        &self.node_id
    }
    pub fn instance_id(&self) -> &str {
        &self.instance_id
    }
    pub fn priority(&self) -> u32 {
        self.priority
    }
    pub fn config(&self) -> &NodeConfig {
        &self.config
    }
    pub fn metrics(&self) -> &NodeMetrics {
        &self.metrics
    }
    pub fn uptime(&self) -> Duration {
        self.creation_time.elapsed()
    }
    // Setters
    pub fn set_priority(&mut self, priority: u32) {
        self.priority = priority;
    }
    // Custom data management
    pub fn set_custom_data(&mut self, key: String, value: String) {
        self.custom_data.insert(key, value);
    }

    pub fn get_custom_data(&self, key: &str) -> Option<&String> {
        self.custom_data.get(key)
    }
}

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

    /// Node's tick rate in Hz (optional)
    ///
    /// Override to specify this node's tick rate. The scheduler uses this
    /// as the default when adding the node.
    ///
    /// Returns `None` by default, meaning use the scheduler's global rate.
    /// Can be overridden at runtime via `Scheduler::set_node_rate()`.
    ///
    /// # Example
    /// ```ignore
    /// fn rate_hz(&self) -> Option<f64> {
    ///     Some(100.0)  // This node runs at 100Hz
    /// }
    /// ```
    fn rate_hz(&self) -> Option<f64> {
        None
    }

    /// Get node configuration (optional override)
    fn config(&self) -> NodeConfig {
        NodeConfig::default()
    }

    /// Health check (optional override)
    ///
    /// **Important:** Override this for production nodes to report actual health status.
    /// The default always returns `true`, which may mask real issues.
    /// Consider checking: connection status, last successful tick, resource availability.
    fn is_healthy(&self) -> bool {
        true
    }

    // ==================== Checkpoint/Restore Support ====================
    // These methods enable automatic state persistence for fault tolerance.
    // Override these in nodes that have internal state to checkpoint.

    /// Serialize node state for checkpointing.
    ///
    /// Override this method to save your node's internal state during checkpoints.
    /// The returned bytes will be stored by the CheckpointManager and can be
    /// restored later via `restore_state()`.
    ///
    /// # Returns
    /// - `Some(Vec<u8>)` - Serialized state (e.g., using bincode or serde_json)
    /// - `None` - This node has no state to checkpoint (default)
    ///
    /// # Example
    /// ```ignore
    /// fn checkpoint_state(&self) -> Option<Vec<u8>> {
    ///     // Serialize internal state
    ///     bincode::serialize(&self.internal_state).ok()
    /// }
    /// ```
    fn checkpoint_state(&self) -> Option<Vec<u8>> {
        None // Default: no state to checkpoint
    }

    /// Restore node state from a checkpoint.
    ///
    /// Override this method to restore your node's internal state from a checkpoint.
    /// This is called during recovery after a crash or when loading a saved checkpoint.
    ///
    /// # Arguments
    /// * `data` - The serialized state previously returned by `checkpoint_state()`
    ///
    /// # Returns
    /// - `Ok(())` if state was restored successfully
    /// - `Err(...)` if restoration failed (e.g., corrupted data, version mismatch)
    ///
    /// # Example
    /// ```ignore
    /// fn restore_state(&mut self, data: &[u8]) -> HorusResult<()> {
    ///     self.internal_state = bincode::deserialize(data)
    ///         .map_err(|e| HorusError::checkpoint(format!("Failed to deserialize: {}", e)))?;
    ///     Ok(())
    /// }
    /// ```
    fn restore_state(&mut self, _data: &[u8]) -> crate::error::HorusResult<()> {
        Ok(()) // Default: no state to restore
    }

    /// Check if this node supports checkpointing.
    ///
    /// Returns true if this node has overridden `checkpoint_state()` and `restore_state()`.
    /// Used by the scheduler to determine which nodes need state persistence.
    fn supports_checkpointing(&self) -> bool {
        false // Default: checkpointing not supported
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
        assert_eq!(NodeState::Paused.to_string(), "Paused");
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
        assert_ne!(NodeState::Running, NodeState::Paused);
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
        };
        let cloned = metrics.clone();
        assert_eq!(cloned.total_ticks, 100);
        assert_eq!(cloned.successful_ticks, 95);
        assert_eq!(cloned.avg_tick_duration_ms, 10.5);
    }

    // =========================================================================
    // NodeConfig Tests
    // =========================================================================

    #[test]
    fn test_node_config_default() {
        let config = NodeConfig::default();
        assert_eq!(config.max_tick_duration_ms, Some(1000));
        assert!(config.restart_on_failure);
        assert_eq!(config.max_restart_attempts, 3);
        assert_eq!(config.restart_delay_ms, 1000);
        assert_eq!(config.log_level, "INFO");
        assert!(config.custom_params.is_empty());
    }

    #[test]
    fn test_node_config_clone() {
        let mut config = NodeConfig::default();
        config
            .custom_params
            .insert("key".to_string(), "value".to_string());
        let cloned = config.clone();
        assert_eq!(cloned.custom_params.get("key"), Some(&"value".to_string()));
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
    fn test_node_info_node_id_format() {
        let info = NodeInfo::new("test_node".to_string());
        // node_id should contain the node name
        assert!(info.node_id().starts_with("test_node_"));
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
    fn test_node_info_priority() {
        let mut info = NodeInfo::new("test_node".to_string());
        assert_eq!(info.priority(), 50); // Default priority is 50 (middle range)

        info.set_priority(0);
        assert_eq!(info.priority(), 0);

        info.set_priority(100);
        assert_eq!(info.priority(), 100);
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
    fn test_node_info_uptime() {
        let info = NodeInfo::new("test_node".to_string());
        std::thread::sleep(std::time::Duration::from_millis(10));
        let uptime = info.uptime();
        assert!(uptime.as_millis() >= 10);
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

    #[test]
    fn test_node_info_with_config() {
        let config = NodeConfig {
            max_tick_duration_ms: Some(500),
            restart_on_failure: false,
            max_restart_attempts: 5,
            ..Default::default()
        };
        let info = NodeInfo::new_with_config("test_node".to_string(), config);
        assert_eq!(info.name(), "test_node");
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
