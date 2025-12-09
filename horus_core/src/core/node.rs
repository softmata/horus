use crate::memory::platform::shm_heartbeats_dir;
use crate::params::RuntimeParams;
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

/// Priority levels for node execution (DEPRECATED - use u32 directly)
///
/// This enum is deprecated in favor of using u32 directly for more flexibility.
/// Use numeric priorities instead: 0 = highest priority, higher numbers = lower priority.
#[deprecated(
    since = "0.2.0",
    note = "Use u32 directly for priority. Lower numbers = higher priority."
)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum NodePriority {
    Critical = 0,
    High = 1,
    Normal = 2,
    Low = 3,
    Background = 4,
}

/// Node health status for monitoring
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
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

    /// Get color code for dashboard display
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

/// Node heartbeat data for shared memory monitoring (platform-specific path)
#[derive(Debug, Clone)]
pub struct NodeHeartbeat {
    pub state: NodeState,
    pub health: HealthStatus,
    pub tick_count: u64,
    pub target_rate_hz: u32,
    pub actual_rate_hz: u32,
    pub error_count: u32,
    pub last_tick_timestamp: u64,
    pub heartbeat_timestamp: u64,
}

impl NodeHeartbeat {
    /// Create new heartbeat from node metrics
    pub fn from_metrics(state: NodeState, metrics: &NodeMetrics) -> Self {
        let now = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap()
            .as_secs();

        // Determine health from metrics
        let health = if metrics.errors_count > 10 {
            HealthStatus::Critical
        } else if metrics.errors_count > 3 {
            HealthStatus::Error
        } else if metrics.failed_ticks > 0 || metrics.avg_tick_duration_ms > 100.0 {
            HealthStatus::Warning
        } else {
            HealthStatus::Healthy
        };

        Self {
            state,
            health,
            tick_count: metrics.total_ticks,
            target_rate_hz: 60, // Default, should be configured
            actual_rate_hz: if metrics.avg_tick_duration_ms > 0.0 {
                (1000.0 / metrics.avg_tick_duration_ms) as u32
            } else {
                0
            },
            error_count: metrics.errors_count as u32,
            last_tick_timestamp: now,
            heartbeat_timestamp: now,
        }
    }

    /// Write heartbeat to file
    pub fn write_to_file(&self, node_name: &str) -> crate::error::HorusResult<()> {
        // Heartbeats are intentionally global (not session-isolated) so dashboard can monitor all nodes
        let dir = shm_heartbeats_dir();
        std::fs::create_dir_all(&dir)?;

        let path = dir.join(node_name);
        let json = serde_json::json!({
            "state": self.state.to_string(),
            "health": self.health.as_str(),
            "tick_count": self.tick_count,
            "target_rate_hz": self.target_rate_hz,
            "actual_rate_hz": self.actual_rate_hz,
            "error_count": self.error_count,
            "last_tick_timestamp": self.last_tick_timestamp,
            "heartbeat_timestamp": self.heartbeat_timestamp,
        });

        std::fs::write(&path, json.to_string())?;
        Ok(())
    }

    /// Read heartbeat from file
    pub fn read_from_file(node_name: &str) -> Option<Self> {
        let path = shm_heartbeats_dir().join(node_name);
        let content = std::fs::read_to_string(&path).ok()?;
        let json: serde_json::Value = serde_json::from_str(&content).ok()?;

        // Parse state string back to enum
        let state_str = json["state"].as_str()?;
        let state = match state_str {
            "Uninitialized" => NodeState::Uninitialized,
            "Initializing" => NodeState::Initializing,
            "Running" => NodeState::Running,
            "Paused" => NodeState::Paused,
            "Stopping" => NodeState::Stopping,
            "Stopped" => NodeState::Stopped,
            s if s.starts_with("Error") => NodeState::Error("".to_string()),
            s if s.starts_with("Crashed") => NodeState::Crashed("".to_string()),
            _ => return None,
        };

        // Parse health
        let health_str = json["health"].as_str()?;
        let health = match health_str {
            "Healthy" => HealthStatus::Healthy,
            "Warning" => HealthStatus::Warning,
            "Error" => HealthStatus::Error,
            "Critical" => HealthStatus::Critical,
            _ => HealthStatus::Unknown,
        };

        Some(Self {
            state,
            health,
            tick_count: json["tick_count"].as_u64()? as u64,
            target_rate_hz: json["target_rate_hz"].as_u64()? as u32,
            actual_rate_hz: json["actual_rate_hz"].as_u64()? as u32,
            error_count: json["error_count"].as_u64()? as u32,
            last_tick_timestamp: json["last_tick_timestamp"].as_u64()?,
            heartbeat_timestamp: json["heartbeat_timestamp"].as_u64()?,
        })
    }

    /// Check if heartbeat is fresh (within last N seconds)
    pub fn is_fresh(&self, max_age_secs: u64) -> bool {
        let now = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap()
            .as_secs();

        now.saturating_sub(self.heartbeat_timestamp) <= max_age_secs
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
    pub timestamp: u64,
}

impl NetworkStatus {
    /// Create a new network status
    pub fn new(node_name: &str, transport_type: &str) -> Self {
        let now = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap()
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
            timestamp: now,
        }
    }

    /// Write network status to shared memory file
    pub fn write_to_file(&self) -> crate::error::HorusResult<()> {
        use crate::memory::platform::shm_network_dir;

        let dir = shm_network_dir();
        std::fs::create_dir_all(&dir)?;

        let path = dir.join(&self.node_name);
        let json = serde_json::to_string_pretty(self)
            .map_err(|e| format!("Failed to serialize network status: {}", e))?;

        std::fs::write(&path, json)?;
        Ok(())
    }

    /// Read network status from file
    pub fn read_from_file(node_name: &str) -> Option<Self> {
        use crate::memory::platform::shm_network_dir;

        let path = shm_network_dir().join(node_name);
        let content = std::fs::read_to_string(&path).ok()?;
        serde_json::from_str(&content).ok()
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
            .unwrap()
            .as_secs();

        now.saturating_sub(self.timestamp) <= max_age_secs
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

/// Configuration parameters for node behavior
#[derive(Debug, Clone)]
pub struct NodeConfig {
    pub max_tick_duration_ms: Option<u64>,
    pub restart_on_failure: bool,
    pub max_restart_attempts: u32,
    pub restart_delay_ms: u64,
    pub enable_logging: bool,
    pub log_level: String,
    pub custom_params: HashMap<String, String>,
}

impl Default for NodeConfig {
    fn default() -> Self {
        NodeConfig {
            max_tick_duration_ms: Some(1000), // 1 second timeout
            restart_on_failure: true,
            max_restart_attempts: 3,
            restart_delay_ms: 1000,
            enable_logging: true,
            log_level: "INFO".to_string(), // Development default: includes info logging
            custom_params: HashMap::new(),
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

    // Communication tracking
    published_topics: HashMap<String, u64>, // topic -> message count
    subscribed_topics: HashMap<String, u64>, // topic -> message count

    // Runtime-discovered pub/sub (for monitor/dashboard)
    registered_publishers: HashMap<String, String>, // topic_name -> type_name
    registered_subscribers: HashMap<String, String>, // topic_name -> type_name

    // Debugging
    custom_data: HashMap<String, String>,

    // Thread safety for metrics updates
    metrics_lock: Arc<Mutex<()>>,

    // Runtime parameters
    pub params: RuntimeParams,
}

impl NodeInfo {
    /// Create a new NodeInfo with comprehensive initialization
    pub fn new(node_name: String, logging_enabled: bool) -> Self {
        let now = Instant::now();
        let node_id = format!(
            "{}_{}",
            node_name,
            std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap_or_default()
                .as_millis()
        );

        let config = NodeConfig {
            enable_logging: logging_enabled,
            ..Default::default()
        };

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
            published_topics: HashMap::new(),
            subscribed_topics: HashMap::new(),
            registered_publishers: HashMap::new(),
            registered_subscribers: HashMap::new(),
            custom_data: HashMap::new(),
            metrics_lock: Arc::new(Mutex::new(())),
            params: RuntimeParams::default(),
        }
    }

    /// Create NodeInfo with custom configuration
    pub fn new_with_config(node_name: String, config: NodeConfig) -> Self {
        let mut node_info = Self::new(node_name, config.enable_logging);
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
        self.log_error(&error_msg);
        self.set_state(NodeState::Error(error_msg));
    }

    pub fn transition_to_crashed(&mut self, crash_msg: String) {
        self.log_error(&crash_msg);
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

    /// Increment tick counter without recording duration metrics
    /// Useful for tools like sim2d that manage their own timing
    /// Only increments when logging is enabled (so ticks start at 0 after learning phase)
    pub fn increment_tick(&mut self) {
        if !self.config.enable_logging {
            return;
        }
        let _guard = self
            .metrics_lock
            .lock()
            .unwrap_or_else(|poisoned| poisoned.into_inner());
        self.metrics.total_ticks += 1;
    }

    pub fn record_tick(&mut self) {
        let _guard = self
            .metrics_lock
            .lock()
            .unwrap_or_else(|poisoned| poisoned.into_inner());

        if let Some(start_time) = self.tick_start_time {
            let duration = start_time.elapsed();
            let duration_ms = duration.as_millis() as f64;

            // Only count ticks when logging is enabled (so display starts at 0 after learning)
            if self.config.enable_logging {
                self.metrics.total_ticks += 1;
            }
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

        // Each node writes its own heartbeat - scheduler doesn't do monitoring
        self.write_heartbeat();
    }

    /// Write heartbeat for this node (called automatically after each successful tick)
    fn write_heartbeat(&self) {
        let heartbeat = NodeHeartbeat::from_metrics(self.state.clone(), &self.metrics);
        let _ = heartbeat.write_to_file(&self.name);
    }

    /// Record node shutdown and write final heartbeat
    pub fn record_shutdown(&mut self) {
        self.transition_to_stopped();
        self.write_heartbeat();
    }

    pub fn record_tick_failure(&mut self, error_msg: String) {
        {
            let _guard = self
                .metrics_lock
                .lock()
                .unwrap_or_else(|poisoned| poisoned.into_inner());
            // Only count ticks when logging is enabled (so display starts at 0 after learning)
            if self.config.enable_logging {
                self.metrics.total_ticks += 1;
            }
            self.metrics.failed_ticks += 1;

            if let Some(start_time) = self.tick_start_time {
                let duration = start_time.elapsed();
                self.metrics.last_tick_duration_ms = duration.as_millis() as f64;
                self.tick_start_time = None;
            }
        }

        self.log_error(&error_msg);

        // Write heartbeat even on failures so monitoring can see error count
        self.write_heartbeat();
    }

    /// Get elapsed time since tick started in microseconds
    pub fn tick_elapsed_us(&self) -> u64 {
        if let Some(start_time) = self.tick_start_time {
            start_time.elapsed().as_micros() as u64
        } else {
            0
        }
    }

    // Runtime Pub/Sub Registration (for monitor/dashboard discovery)
    // Called by Hub::send()/recv() when ctx is provided

    /// Register this node as a publisher to a topic (called automatically by Hub::send)
    pub fn register_publisher(&mut self, topic_name: &str, type_name: &str) {
        self.registered_publishers
            .entry(topic_name.to_string())
            .or_insert_with(|| type_name.to_string());
    }

    /// Register this node as a subscriber to a topic (called automatically by Hub::recv)
    pub fn register_subscriber(&mut self, topic_name: &str, type_name: &str) {
        self.registered_subscribers
            .entry(topic_name.to_string())
            .or_insert_with(|| type_name.to_string());
    }

    /// Get all registered publishers (topic_name -> type_name)
    pub fn get_registered_publishers(&self) -> Vec<TopicMetadata> {
        self.registered_publishers
            .iter()
            .map(|(topic, type_name)| TopicMetadata {
                topic_name: topic.clone(),
                type_name: type_name.clone(),
            })
            .collect()
    }

    /// Get all registered subscribers (topic_name -> type_name)
    pub fn get_registered_subscribers(&self) -> Vec<TopicMetadata> {
        self.registered_subscribers
            .iter()
            .map(|(topic, type_name)| TopicMetadata {
                topic_name: topic.clone(),
                type_name: type_name.clone(),
            })
            .collect()
    }

    // Logging Methods - Production-Ready with IPC Timing
    // ALWAYS requires IPC timing measurement - no fallback
    pub fn log_pub<T: LogSummary>(&mut self, topic: &str, data: &T, ipc_ns: u64) {
        let summary = data.log_summary();
        self.log_pub_summary(topic, &summary, ipc_ns);
    }

    pub fn log_sub<T: LogSummary>(&mut self, topic: &str, data: &T, ipc_ns: u64) {
        let summary = data.log_summary();
        self.log_sub_summary(topic, &summary, ipc_ns);
    }

    /// Internal logging method that accepts a pre-computed summary string
    /// Used by Hub::send() to avoid needing message reference after move
    pub fn log_pub_summary(&mut self, topic: &str, summary: &str, ipc_ns: u64) {
        let now = chrono::Local::now();
        let current_tick_us = if let Some(start_time) = self.tick_start_time {
            start_time.elapsed().as_micros() as u64
        } else {
            0
        };

        if self.config.enable_logging {
            // Color-coded logging for readability
            // Cyan timestamp | Green metrics | Blue tick# | Yellow node | Bold Green PUB arrow | Magenta topic | White data
            print!("\r\n\x1b[36m[{}]\x1b[0m \x1b[32m[IPC: {}ns | Tick: {}μs]\x1b[0m \x1b[34m[#{}]\x1b[0m \x1b[33m{}\x1b[0m \x1b[1;32m--PUB-->\x1b[0m \x1b[35m'{}'\x1b[0m = {}\r\n",
                   now.format("%H:%M:%S%.3f"),
                   ipc_ns,
                   current_tick_us,
                   self.metrics.total_ticks,
                   self.name, topic, summary);
            use std::io::{self, Write};
            let _ = io::stdout().flush();
        }

        // Write to global log buffer and publish to system/logs topic
        use crate::core::log_buffer::{publish_log, LogEntry, LogType};
        publish_log(LogEntry {
            timestamp: now.format("%H:%M:%S%.3f").to_string(),
            tick_number: self.metrics.total_ticks,
            node_name: self.name.clone(),
            log_type: LogType::Publish,
            topic: Some(topic.to_string()),
            message: summary.to_string(),
            tick_us: current_tick_us,
            ipc_ns,
        });

        *self.published_topics.entry(topic.to_string()).or_insert(0) += 1;
        self.metrics.messages_sent += 1;
    }

    /// Internal logging method that accepts a pre-computed summary string
    /// Used by Hub::recv() to avoid needing message reference after move
    pub fn log_sub_summary(&mut self, topic: &str, summary: &str, ipc_ns: u64) {
        let now = chrono::Local::now();
        let current_tick_us = if let Some(start_time) = self.tick_start_time {
            start_time.elapsed().as_micros() as u64
        } else {
            0
        };

        // Display tick count
        let display_tick = self.metrics.total_ticks;

        if self.config.enable_logging {
            // Color-coded logging for readability
            // Cyan timestamp | Green metrics | Blue tick# | Yellow node | Bold Blue SUB arrow | Magenta topic | White data
            println!("\x1b[36m[{}]\x1b[0m \x1b[32m[IPC: {}ns | Tick: {}μs]\x1b[0m \x1b[34m[#{}]\x1b[0m \x1b[33m{}\x1b[0m \x1b[1;34m<--SUB--\x1b[0m \x1b[35m'{}'\x1b[0m = {}",
                   now.format("%H:%M:%S%.3f"),
                   ipc_ns,
                   current_tick_us,
                   display_tick,
                   self.name, topic, summary);
            use std::io::{self, Write};
            let _ = io::stdout().flush();
        }

        // Write to global log buffer and publish to system/logs topic
        use crate::core::log_buffer::{publish_log, LogEntry, LogType};
        publish_log(LogEntry {
            timestamp: now.format("%H:%M:%S%.3f").to_string(),
            tick_number: display_tick,
            node_name: self.name.clone(),
            log_type: LogType::Subscribe,
            topic: Some(topic.to_string()),
            message: summary.to_string(),
            tick_us: current_tick_us,
            ipc_ns,
        });

        *self.subscribed_topics.entry(topic.to_string()).or_insert(0) += 1;
        self.metrics.messages_received += 1;
    }

    pub fn log_info(&self, message: &str) {
        let now = chrono::Local::now();
        let current_tick_us = if let Some(start_time) = self.tick_start_time {
            start_time.elapsed().as_micros() as u64
        } else {
            0
        };

        if self.config.enable_logging
            && (self.config.log_level == "INFO" || self.config.log_level == "DEBUG")
        {
            eprintln!(
                "\x1b[34m[INFO]\x1b[0m \x1b[33m[{}]\x1b[0m {}",
                self.name, message
            );
        }

        // Write to global log buffer for dashboard
        use crate::core::log_buffer::{publish_log, LogEntry, LogType};
        publish_log(LogEntry {
            timestamp: now.format("%H:%M:%S%.3f").to_string(),
            tick_number: self.metrics.total_ticks,
            node_name: self.name.clone(),
            log_type: LogType::Info,
            topic: None,
            message: message.to_string(),
            tick_us: current_tick_us,
            ipc_ns: 0,
        });
    }

    pub fn log_warning(&mut self, message: &str) {
        let now = chrono::Local::now();
        let current_tick_us = if let Some(start_time) = self.tick_start_time {
            start_time.elapsed().as_micros() as u64
        } else {
            0
        };

        if self.config.enable_logging {
            // Format to owned String first to avoid double-formatting issues
            let msg = format!(
                "\x1b[33m[WARN]\x1b[0m \x1b[33m[{}]\x1b[0m {}\n",
                self.name, message
            );
            use std::io::{self, Write};
            let _ = io::stdout().write_all(msg.as_bytes());
            let _ = io::stdout().flush();
        }

        // Write to global log buffer for dashboard
        use crate::core::log_buffer::{publish_log, LogEntry, LogType};
        publish_log(LogEntry {
            timestamp: now.format("%H:%M:%S%.3f").to_string(),
            tick_number: self.metrics.total_ticks,
            node_name: self.name.clone(),
            log_type: LogType::Warning,
            topic: None,
            message: message.to_string(),
            tick_us: current_tick_us,
            ipc_ns: 0,
        });

        self.warning_history
            .push((Instant::now(), message.to_string()));
        if self.warning_history.len() > 100 {
            self.warning_history.remove(0);
        }
        self.metrics.warnings_count += 1;
    }

    pub fn log_error(&mut self, message: &str) {
        let now = chrono::Local::now();
        let current_tick_us = if let Some(start_time) = self.tick_start_time {
            start_time.elapsed().as_micros() as u64
        } else {
            0
        };

        if self.config.enable_logging {
            // Format to owned String first to avoid double-formatting issues
            let msg = format!(
                "\x1b[31m[ERROR]\x1b[0m \x1b[33m[{}]\x1b[0m {}\n",
                self.name, message
            );
            use std::io::{self, Write};
            let _ = io::stdout().write_all(msg.as_bytes());
            let _ = io::stdout().flush();
        }

        // Write to global log buffer for dashboard
        use crate::core::log_buffer::{publish_log, LogEntry, LogType};
        publish_log(LogEntry {
            timestamp: now.format("%H:%M:%S%.3f").to_string(),
            tick_number: self.metrics.total_ticks,
            node_name: self.name.clone(),
            log_type: LogType::Error,
            topic: None,
            message: message.to_string(),
            tick_us: current_tick_us,
            ipc_ns: 0,
        });

        self.error_history
            .push((Instant::now(), message.to_string()));
        if self.error_history.len() > 100 {
            self.error_history.remove(0);
        }
        self.metrics.errors_count += 1;
    }

    pub fn log_debug(&mut self, message: &str) {
        let now = chrono::Local::now();
        let current_tick_us = if let Some(start_time) = self.tick_start_time {
            start_time.elapsed().as_micros() as u64
        } else {
            0
        };

        if self.config.enable_logging && self.config.log_level == "DEBUG" {
            // Format to owned String first to avoid double-formatting issues
            let msg = format!(
                "\x1b[90m[DEBUG]\x1b[0m \x1b[33m[{}]\x1b[0m {}\n",
                self.name, message
            );
            use std::io::{self, Write};
            let _ = io::stdout().write_all(msg.as_bytes());
            let _ = io::stdout().flush();
        }

        // Write to global log buffer for dashboard
        use crate::core::log_buffer::{publish_log, LogEntry, LogType};
        publish_log(LogEntry {
            timestamp: now.format("%H:%M:%S%.3f").to_string(),
            tick_number: self.metrics.total_ticks,
            node_name: self.name.clone(),
            log_type: LogType::Debug,
            topic: None,
            message: message.to_string(),
            tick_us: current_tick_us,
            ipc_ns: 0,
        });
    }

    /// Production-ready metric logging - logs only significant events
    pub fn log_metrics_summary(&mut self) {
        if self.config.enable_logging && self.config.log_level != "QUIET" {
            let now = chrono::Local::now();
            let uptime = self.creation_time.elapsed().as_secs();

            // Only log if there are concerning metrics
            if self.metrics.failed_ticks > 0 || self.metrics.avg_tick_duration_ms > 100.0 {
                println!(
                    "[{}] METRICS {} - uptime:{}s, ticks:{}/{}, avg:{}ms",
                    now.format("%H:%M:%S"),
                    self.name,
                    uptime,
                    self.metrics.successful_ticks,
                    self.metrics.total_ticks,
                    self.metrics.avg_tick_duration_ms as u64
                );
            }
        }
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
    pub fn published_topics(&self) -> &HashMap<String, u64> {
        &self.published_topics
    }
    pub fn subscribed_topics(&self) -> &HashMap<String, u64> {
        &self.subscribed_topics
    }
    pub fn uptime(&self) -> Duration {
        self.creation_time.elapsed()
    }
    pub fn time_in_current_state(&self) -> Duration {
        self.state_change_time.elapsed()
    }

    // Setters
    pub fn set_priority(&mut self, priority: u32) {
        self.priority = priority;
    }
    pub fn set_config(&mut self, config: NodeConfig) {
        self.config = config;
    }
    pub fn set_logging_enabled(&mut self, enabled: bool) {
        self.config.enable_logging = enabled;
    }
    pub fn is_logging_enabled(&self) -> bool {
        self.config.enable_logging
    }

    // Custom data management
    pub fn set_custom_data(&mut self, key: String, value: String) {
        self.custom_data.insert(key, value);
    }

    pub fn get_custom_data(&self, key: &str) -> Option<&String> {
        self.custom_data.get(key)
    }

    pub fn remove_custom_data(&mut self, key: &str) -> Option<String> {
        self.custom_data.remove(key)
    }
}

/// Topic metadata for monitoring and introspection
#[derive(Debug, Clone)]
pub struct TopicMetadata {
    pub topic_name: String,
    pub type_name: String,
}

/// Comprehensive trait for Horus nodes with full lifecycle support
pub trait Node: Send {
    /// Get the node's name (must be unique)
    fn name(&self) -> &'static str;

    /// Initialize the node (called once at startup)
    fn init(&mut self, ctx: &mut NodeInfo) -> crate::error::HorusResult<()> {
        ctx.log_info("Node initialized successfully");
        Ok(())
    }

    /// Main execution loop (called repeatedly)
    fn tick(&mut self, ctx: Option<&mut NodeInfo>);

    /// Shutdown the node (called once at cleanup)
    fn shutdown(&mut self, ctx: &mut NodeInfo) -> crate::error::HorusResult<()> {
        ctx.log_info("Node shutdown successfully");
        Ok(())
    }

    /// Get list of publishers (topic metadata)
    ///
    /// **Important:** Override this if your node publishes to any topics.
    /// The default returns empty, which may hide connectivity issues during debugging.
    fn get_publishers(&self) -> Vec<TopicMetadata> {
        Vec::new()
    }

    /// Get list of subscribers (topic metadata)
    ///
    /// **Important:** Override this if your node subscribes to any topics.
    /// The default returns empty, which may hide connectivity issues during debugging.
    fn get_subscribers(&self) -> Vec<TopicMetadata> {
        Vec::new()
    }

    /// Handle errors (optional override)
    fn on_error(&mut self, error: &str, ctx: &mut NodeInfo) {
        ctx.log_error(&format!("Node error: {}", error));
    }

    /// Get node priority (optional override)
    /// Lower numbers = higher priority. Default is 50 (normal priority).
    fn priority(&self) -> u32 {
        50 // Normal priority
    }

    /// Get node configuration (optional override)
    fn get_config(&self) -> NodeConfig {
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

    // ==================== JIT Compilation Support ====================
    // These methods enable nodes to be JIT-compiled for ultra-fast execution.
    // Override these in nodes that perform simple, deterministic computations.

    /// Returns true if this node supports JIT compilation.
    /// JIT-compiled nodes can execute in 20-50ns instead of microseconds.
    ///
    /// Override this to return `true` for nodes that:
    /// - Perform simple arithmetic/logic operations
    /// - Are deterministic (same input = same output)
    /// - Have no side effects (pure functions)
    fn supports_jit(&self) -> bool {
        false
    }

    /// Returns true if this node is deterministic (same inputs always produce same outputs).
    /// Required for JIT compilation.
    fn is_jit_deterministic(&self) -> bool {
        false
    }

    /// Returns true if this node is pure (no side effects).
    /// Required for JIT compilation.
    fn is_jit_pure(&self) -> bool {
        false
    }

    /// Get the JIT compute function for this node.
    /// Returns a function pointer that takes an i64 input and returns i64 output.
    ///
    /// This is a simplified interface for JIT compilation. For complex nodes,
    /// implement the full DataflowNode trait in the scheduling module.
    ///
    /// # Example
    /// ```ignore
    /// fn get_jit_compute(&self) -> Option<fn(i64) -> i64> {
    ///     Some(|x| x * self.scale + self.offset)
    /// }
    /// ```
    fn get_jit_compute(&self) -> Option<fn(i64) -> i64> {
        None
    }

    /// Get JIT parameters for arithmetic nodes: (multiply_factor, offset)
    /// Used by the Cranelift JIT compiler to generate: output = input * factor + offset
    ///
    /// Returns None if this node doesn't support simple arithmetic JIT.
    fn get_jit_arithmetic_params(&self) -> Option<(i64, i64)> {
        None
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
