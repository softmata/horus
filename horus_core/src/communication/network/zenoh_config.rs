//! Zenoh transport configuration
//!
//! Configuration for Zenoh backend connections, including:
//! - Connection endpoints (routers, peers)
//! - Namespace and topic mapping
//! - ROS2 compatibility settings
//! - QoS and reliability options

use std::path::PathBuf;
use std::time::Duration;

/// Configuration for Zenoh backend
#[derive(Debug, Clone)]
pub struct ZenohConfig {
    /// Path to zenoh config file (optional)
    /// If provided, loads full zenoh configuration from file
    pub config_path: Option<PathBuf>,

    /// Namespace prefix for all topics
    /// Default: "horus"
    /// Topics will be published as: {namespace}/{topic_name}
    pub namespace: Option<String>,

    /// Enable ROS2 compatibility mode
    /// When true, uses ROS2-compatible topic naming and message formats
    pub ros2_mode: bool,

    /// ROS2 domain ID (0-232)
    /// Only used when ros2_mode is true
    pub ros2_domain_id: u32,

    /// Connection mode
    pub mode: ZenohMode,

    /// Endpoints to connect to (routers or peers)
    /// Format: "tcp/192.168.1.1:7447" or "udp/192.168.1.1:7447"
    pub connect: Vec<String>,

    /// Endpoints to listen on
    /// Format: "tcp/0.0.0.0:7447"
    pub listen: Vec<String>,

    /// Session timeout
    pub session_timeout: Duration,

    /// Enable shared memory transport for local communication
    pub shared_memory: bool,

    /// QoS settings
    pub qos: ZenohQos,

    /// Serialization format
    pub serialization: SerializationFormat,
}

impl Default for ZenohConfig {
    fn default() -> Self {
        Self {
            config_path: None,
            namespace: Some("horus".to_string()),
            ros2_mode: false,
            ros2_domain_id: 0,
            mode: ZenohMode::Peer,
            connect: Vec::new(),
            listen: Vec::new(),
            session_timeout: Duration::from_secs(10),
            shared_memory: true,
            qos: ZenohQos::default(),
            serialization: SerializationFormat::Bincode,
        }
    }
}

impl ZenohConfig {
    /// Create a new default configuration
    pub fn new() -> Self {
        Self::default()
    }

    /// Create configuration for ROS2 interop
    pub fn ros2(domain_id: u32) -> Self {
        Self {
            ros2_mode: true,
            ros2_domain_id: domain_id,
            namespace: None,                         // Use ROS2 native naming
            serialization: SerializationFormat::Cdr, // ROS2 uses CDR
            ..Default::default()
        }
    }

    /// Create configuration for cloud/remote connection
    pub fn cloud(router_endpoint: &str) -> Self {
        Self {
            mode: ZenohMode::Client,
            connect: vec![router_endpoint.to_string()],
            ..Default::default()
        }
    }

    /// Create configuration for local mesh
    pub fn local_mesh() -> Self {
        Self {
            mode: ZenohMode::Peer,
            shared_memory: true,
            ..Default::default()
        }
    }

    /// Set the namespace
    pub fn with_namespace(mut self, namespace: &str) -> Self {
        self.namespace = Some(namespace.to_string());
        self
    }

    /// Add a connection endpoint
    pub fn connect_to(mut self, endpoint: &str) -> Self {
        self.connect.push(endpoint.to_string());
        self
    }

    /// Add a listen endpoint
    pub fn listen_on(mut self, endpoint: &str) -> Self {
        self.listen.push(endpoint.to_string());
        self
    }

    /// Enable ROS2 mode
    pub fn with_ros2(mut self, domain_id: u32) -> Self {
        self.ros2_mode = true;
        self.ros2_domain_id = domain_id;
        self
    }

    /// Load from config file
    pub fn from_file(path: impl Into<PathBuf>) -> Self {
        Self {
            config_path: Some(path.into()),
            ..Default::default()
        }
    }

    /// Get the full key expression for a topic
    pub fn topic_to_key_expr(&self, topic: &str) -> String {
        if self.ros2_mode {
            // ROS2 naming: rt/{topic} for topics, rq/{service}/Request for services
            if topic.starts_with('/') {
                format!("rt{}", topic)
            } else {
                format!("rt/{}", topic)
            }
        } else {
            // HORUS naming: {namespace}/{topic}
            match &self.namespace {
                Some(ns) => format!("{}/{}", ns, topic),
                None => topic.to_string(),
            }
        }
    }

    /// Parse a HORUS topic from a Zenoh key expression
    pub fn key_expr_to_topic(&self, key_expr: &str) -> Option<String> {
        if self.ros2_mode {
            // Strip "rt/" prefix
            key_expr
                .strip_prefix("rt/")
                .or_else(|| key_expr.strip_prefix("rt"))
                .map(|s| s.to_string())
        } else {
            // Strip namespace prefix
            match &self.namespace {
                Some(ns) => {
                    let prefix = format!("{}/", ns);
                    key_expr.strip_prefix(&prefix).map(|s| s.to_string())
                }
                None => Some(key_expr.to_string()),
            }
        }
    }
}

/// Zenoh session mode
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Hash)]
pub enum ZenohMode {
    /// Peer mode: discover and connect to other peers
    #[default]
    Peer,
    /// Client mode: connect only to routers
    Client,
    /// Router mode: accept connections from clients and peers
    Router,
}

/// QoS settings for Zenoh
///
/// This struct provides comprehensive QoS configuration compatible with ROS2 DDS policies.
/// It maps ROS2 QoS settings to Zenoh equivalents for interoperability.
#[derive(Debug, Clone)]
pub struct ZenohQos {
    /// Reliability: best-effort or reliable
    pub reliability: Reliability,
    /// Congestion control
    pub congestion: CongestionControl,
    /// Priority (0-7, higher = more important)
    pub priority: u8,
    /// Express mode (skip batching for low latency)
    pub express: bool,
    /// History policy: how many samples to keep
    pub history: HistoryPolicy,
    /// Durability: whether late joiners receive historical data
    pub durability: Durability,
    /// Deadline: expected maximum time between messages
    pub deadline: Option<Duration>,
    /// Lifespan: how long messages remain valid
    pub lifespan: Option<Duration>,
    /// Liveliness: how entities assert their presence
    pub liveliness: Liveliness,
    /// Liveliness lease duration (how long before entity is considered not alive)
    pub liveliness_lease_duration: Option<Duration>,
}

impl Default for ZenohQos {
    fn default() -> Self {
        Self {
            reliability: Reliability::BestEffort,
            congestion: CongestionControl::Drop,
            priority: 5,
            express: false,
            history: HistoryPolicy::KeepLast(1),
            durability: Durability::Volatile,
            deadline: None,
            lifespan: None,
            liveliness: Liveliness::Automatic,
            liveliness_lease_duration: None,
        }
    }
}

impl ZenohQos {
    /// Create QoS for real-time data (low latency, may drop)
    pub fn realtime() -> Self {
        Self {
            reliability: Reliability::BestEffort,
            congestion: CongestionControl::Drop,
            priority: 7,
            express: true,
            history: HistoryPolicy::KeepLast(1),
            durability: Durability::Volatile,
            deadline: None,
            lifespan: None,
            liveliness: Liveliness::Automatic,
            liveliness_lease_duration: None,
        }
    }

    /// Create QoS for reliable data (no loss, may delay)
    pub fn reliable() -> Self {
        Self {
            reliability: Reliability::Reliable,
            congestion: CongestionControl::Block,
            priority: 5,
            express: false,
            history: HistoryPolicy::KeepLast(10),
            durability: Durability::Volatile,
            deadline: None,
            lifespan: None,
            liveliness: Liveliness::Automatic,
            liveliness_lease_duration: None,
        }
    }

    /// Create QoS for bulk data (throughput optimized)
    pub fn bulk() -> Self {
        Self {
            reliability: Reliability::Reliable,
            congestion: CongestionControl::Block,
            priority: 1,
            express: false,
            history: HistoryPolicy::KeepLast(100),
            durability: Durability::Volatile,
            deadline: None,
            lifespan: None,
            liveliness: Liveliness::Automatic,
            liveliness_lease_duration: None,
        }
    }

    /// Create QoS for sensor data (common ROS2 pattern)
    /// Best-effort, keep last sample, volatile
    pub fn sensor_data() -> Self {
        Self {
            reliability: Reliability::BestEffort,
            congestion: CongestionControl::Drop,
            priority: 5,
            express: false,
            history: HistoryPolicy::KeepLast(5),
            durability: Durability::Volatile,
            deadline: Some(Duration::from_millis(100)), // Expect data every 100ms
            lifespan: Some(Duration::from_millis(200)), // Data valid for 200ms
            liveliness: Liveliness::Automatic,
            liveliness_lease_duration: Some(Duration::from_secs(1)),
        }
    }

    /// Create QoS for parameters (ROS2 pattern)
    /// Reliable, transient local durability
    pub fn parameters() -> Self {
        Self {
            reliability: Reliability::Reliable,
            congestion: CongestionControl::Block,
            priority: 5,
            express: false,
            history: HistoryPolicy::KeepLast(1),
            durability: Durability::TransientLocal,
            deadline: None,
            lifespan: None,
            liveliness: Liveliness::Automatic,
            liveliness_lease_duration: None,
        }
    }

    /// Create QoS for services (ROS2 pattern)
    /// Reliable, volatile, keep all history
    pub fn services() -> Self {
        Self {
            reliability: Reliability::Reliable,
            congestion: CongestionControl::Block,
            priority: 6,
            express: false,
            history: HistoryPolicy::KeepAll,
            durability: Durability::Volatile,
            deadline: None,
            lifespan: None,
            liveliness: Liveliness::Automatic,
            liveliness_lease_duration: None,
        }
    }

    /// Create QoS for actions (ROS2 pattern)
    /// Reliable with feedback history
    pub fn actions() -> Self {
        Self {
            reliability: Reliability::Reliable,
            congestion: CongestionControl::Block,
            priority: 5,
            express: false,
            history: HistoryPolicy::KeepLast(10),
            durability: Durability::Volatile,
            deadline: None,
            lifespan: None,
            liveliness: Liveliness::Automatic,
            liveliness_lease_duration: None,
        }
    }

    /// Create QoS for system default (matches ROS2 default)
    pub fn system_default() -> Self {
        Self {
            reliability: Reliability::Reliable,
            congestion: CongestionControl::Block,
            priority: 5,
            express: false,
            history: HistoryPolicy::KeepLast(10),
            durability: Durability::Volatile,
            deadline: None,
            lifespan: None,
            liveliness: Liveliness::Automatic,
            liveliness_lease_duration: None,
        }
    }

    // Builder methods for customization

    /// Set reliability
    pub fn with_reliability(mut self, reliability: Reliability) -> Self {
        self.reliability = reliability;
        self
    }

    /// Set history policy
    pub fn with_history(mut self, history: HistoryPolicy) -> Self {
        self.history = history;
        self
    }

    /// Set durability
    pub fn with_durability(mut self, durability: Durability) -> Self {
        self.durability = durability;
        self
    }

    /// Set deadline
    pub fn with_deadline(mut self, deadline: Duration) -> Self {
        self.deadline = Some(deadline);
        self
    }

    /// Set lifespan
    pub fn with_lifespan(mut self, lifespan: Duration) -> Self {
        self.lifespan = Some(lifespan);
        self
    }

    /// Set liveliness policy
    pub fn with_liveliness(
        mut self,
        liveliness: Liveliness,
        lease_duration: Option<Duration>,
    ) -> Self {
        self.liveliness = liveliness;
        self.liveliness_lease_duration = lease_duration;
        self
    }

    /// Set priority
    pub fn with_priority(mut self, priority: u8) -> Self {
        self.priority = priority.min(7);
        self
    }

    /// Set express mode
    pub fn with_express(mut self, express: bool) -> Self {
        self.express = express;
        self
    }

    /// Check if deadline has been missed
    pub fn check_deadline(&self, last_message_time: std::time::Instant) -> bool {
        if let Some(deadline) = self.deadline {
            last_message_time.elapsed() > deadline
        } else {
            false
        }
    }

    /// Check if message has expired based on lifespan
    pub fn is_message_expired(&self, message_time: std::time::Instant) -> bool {
        if let Some(lifespan) = self.lifespan {
            message_time.elapsed() > lifespan
        } else {
            false
        }
    }
}

/// History policy for QoS
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum HistoryPolicy {
    /// Keep last N samples
    KeepLast(usize),
    /// Keep all samples (bounded by resource limits)
    KeepAll,
}

impl Default for HistoryPolicy {
    fn default() -> Self {
        Self::KeepLast(1)
    }
}

impl HistoryPolicy {
    /// Get the depth (number of samples to keep)
    pub fn depth(&self) -> Option<usize> {
        match self {
            Self::KeepLast(n) => Some(*n),
            Self::KeepAll => None,
        }
    }
}

/// Durability policy for QoS
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Durability {
    /// No historical data for late joiners
    #[default]
    Volatile,
    /// Publisher keeps historical data for late-joining subscribers
    TransientLocal,
}

/// Liveliness policy for QoS
///
/// Configures how entities assert their presence in the system.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Liveliness {
    /// System automatically asserts liveliness
    #[default]
    Automatic,
    /// User must manually assert liveliness on the participant
    ManualByParticipant,
    /// User must manually assert liveliness on each topic
    ManualByTopic,
}

impl Liveliness {
    /// Check if manual assertion is required
    pub fn requires_manual_assertion(&self) -> bool {
        !matches!(self, Self::Automatic)
    }
}

/// Reliability mode
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Reliability {
    /// Best-effort delivery (may lose messages)
    BestEffort,
    /// Reliable delivery (guarantees delivery)
    Reliable,
}

/// Congestion control mode
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CongestionControl {
    /// Drop messages when congested
    Drop,
    /// Block sender when congested
    Block,
}

/// Serialization format for messages
///
/// HORUS supports two serialization formats:
/// - **Bincode**: Fast, compact binary format (default for HORUS-to-HORUS)
/// - **CDR**: Common Data Representation for ROS2 interoperability
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum SerializationFormat {
    /// Bincode (HORUS native, fastest)
    #[default]
    Bincode,
    /// CDR (ROS2 compatible)
    Cdr,
}

/// Default HORUS cloud router endpoint
pub const HORUS_CLOUD_ROUTER: &str = "tcp/cloud.horus.io:7447";

/// Default HORUS cloud router endpoints (primary + backup)
pub const HORUS_CLOUD_ROUTERS: &[&str] = &[
    "tcp/cloud.horus.io:7447",
    "tcp/cloud-eu.horus.io:7447",
    "tcp/cloud-asia.horus.io:7447",
];

/// Configuration for Zenoh cloud connectivity with failover and monitoring
///
/// This provides enhanced cloud connectivity features:
/// - Multiple router endpoints with automatic failover
/// - Auto-discovery of nearby routers
/// - Connection quality monitoring
/// - Automatic reconnection with exponential backoff
///
/// # Example
///
/// ```rust
/// use horus_core::communication::network::ZenohCloudConfig;
///
/// // Auto-connect to HORUS cloud with defaults
/// let config = ZenohCloudConfig::auto();
///
/// // Custom routers with failover
/// let config = ZenohCloudConfig::new()
///     .with_routers(&["tcp/my-router.example.com:7447", "tcp/backup.example.com:7447"])
///     .with_failover_timeout(std::time::Duration::from_secs(5));
/// ```
#[derive(Debug, Clone, PartialEq)]
pub struct ZenohCloudConfig {
    /// Primary router endpoint
    pub primary_router: String,

    /// Backup router endpoints (tried in order on failover)
    pub backup_routers: Vec<String>,

    /// Enable auto-discovery of nearby routers via mDNS/DNS-SD
    pub auto_discovery: bool,

    /// DNS-SD service type for router discovery
    /// Default: "_horus-zenoh._tcp.local"
    pub discovery_service_type: String,

    /// Connection timeout before trying next router
    pub connection_timeout: Duration,

    /// Failover timeout - how long to wait before switching to backup
    pub failover_timeout: Duration,

    /// Reconnect interval after connection loss
    pub reconnect_interval: Duration,

    /// Maximum reconnect attempts before giving up (None = infinite)
    pub max_reconnect_attempts: Option<u32>,

    /// Enable connection quality monitoring
    pub enable_monitoring: bool,

    /// Latency threshold (ms) - warn if exceeded
    pub latency_warn_threshold_ms: u64,

    /// Latency threshold (ms) - trigger failover if exceeded
    pub latency_failover_threshold_ms: u64,

    /// Health check interval
    pub health_check_interval: Duration,

    /// Number of consecutive health check failures before failover
    pub health_check_failures_before_failover: u32,

    /// Enable compression for cloud traffic
    pub enable_compression: bool,

    /// Enable encryption for cloud traffic (TLS)
    pub enable_encryption: bool,

    /// Custom namespace for cloud topics (optional)
    /// If not set, uses the default HORUS namespace
    pub namespace: Option<String>,
}

impl Default for ZenohCloudConfig {
    fn default() -> Self {
        Self {
            primary_router: HORUS_CLOUD_ROUTER.to_string(),
            backup_routers: HORUS_CLOUD_ROUTERS[1..]
                .iter()
                .map(|s| s.to_string())
                .collect(),
            auto_discovery: true,
            discovery_service_type: "_horus-zenoh._tcp.local".to_string(),
            connection_timeout: Duration::from_secs(10),
            failover_timeout: Duration::from_secs(5),
            reconnect_interval: Duration::from_secs(1),
            max_reconnect_attempts: None, // Infinite retries
            enable_monitoring: true,
            latency_warn_threshold_ms: 100,
            latency_failover_threshold_ms: 500,
            health_check_interval: Duration::from_secs(5),
            health_check_failures_before_failover: 3,
            enable_compression: true,
            enable_encryption: true,
            namespace: None,
        }
    }
}

impl ZenohCloudConfig {
    /// Create a new cloud config with defaults
    pub fn new() -> Self {
        Self::default()
    }

    /// Create cloud config with auto-discovery enabled
    /// This will discover nearby HORUS routers via mDNS
    pub fn auto() -> Self {
        Self {
            auto_discovery: true,
            ..Default::default()
        }
    }

    /// Create cloud config for HORUS hosted cloud
    /// Connects to the official HORUS cloud infrastructure
    pub fn horus_cloud() -> Self {
        Self::default()
    }

    /// Create cloud config for self-hosted router
    pub fn self_hosted(router_endpoint: &str) -> Self {
        Self {
            primary_router: router_endpoint.to_string(),
            backup_routers: Vec::new(),
            auto_discovery: false,
            ..Default::default()
        }
    }

    /// Set primary and backup routers
    pub fn with_routers(mut self, routers: &[&str]) -> Self {
        if let Some((first, rest)) = routers.split_first() {
            self.primary_router = first.to_string();
            self.backup_routers = rest.iter().map(|s| s.to_string()).collect();
        }
        self
    }

    /// Add a backup router
    pub fn with_backup_router(mut self, router: &str) -> Self {
        self.backup_routers.push(router.to_string());
        self
    }

    /// Enable or disable auto-discovery
    pub fn with_auto_discovery(mut self, enabled: bool) -> Self {
        self.auto_discovery = enabled;
        self
    }

    /// Set connection timeout
    pub fn with_connection_timeout(mut self, timeout: Duration) -> Self {
        self.connection_timeout = timeout;
        self
    }

    /// Set failover timeout
    pub fn with_failover_timeout(mut self, timeout: Duration) -> Self {
        self.failover_timeout = timeout;
        self
    }

    /// Set reconnect interval
    pub fn with_reconnect_interval(mut self, interval: Duration) -> Self {
        self.reconnect_interval = interval;
        self
    }

    /// Set maximum reconnect attempts
    pub fn with_max_reconnect_attempts(mut self, attempts: Option<u32>) -> Self {
        self.max_reconnect_attempts = attempts;
        self
    }

    /// Enable or disable monitoring
    pub fn with_monitoring(mut self, enabled: bool) -> Self {
        self.enable_monitoring = enabled;
        self
    }

    /// Set latency thresholds
    pub fn with_latency_thresholds(mut self, warn_ms: u64, failover_ms: u64) -> Self {
        self.latency_warn_threshold_ms = warn_ms;
        self.latency_failover_threshold_ms = failover_ms;
        self
    }

    /// Set health check parameters
    pub fn with_health_check(mut self, interval: Duration, failures_before_failover: u32) -> Self {
        self.health_check_interval = interval;
        self.health_check_failures_before_failover = failures_before_failover;
        self
    }

    /// Enable or disable compression
    pub fn with_compression(mut self, enabled: bool) -> Self {
        self.enable_compression = enabled;
        self
    }

    /// Enable or disable encryption
    pub fn with_encryption(mut self, enabled: bool) -> Self {
        self.enable_encryption = enabled;
        self
    }

    /// Set namespace
    pub fn with_namespace(mut self, namespace: &str) -> Self {
        self.namespace = Some(namespace.to_string());
        self
    }

    /// Get all router endpoints in priority order
    pub fn all_routers(&self) -> Vec<&str> {
        let mut routers = vec![self.primary_router.as_str()];
        routers.extend(self.backup_routers.iter().map(|s| s.as_str()));
        routers
    }

    /// Convert to ZenohConfig for the primary router
    pub fn to_zenoh_config(&self) -> ZenohConfig {
        ZenohConfig {
            mode: ZenohMode::Client,
            connect: vec![self.primary_router.clone()],
            session_timeout: self.connection_timeout,
            namespace: self.namespace.clone().or_else(|| Some("horus".to_string())),
            ..Default::default()
        }
    }

    /// Convert to ZenohConfig with all routers (for implementations that support multiple)
    pub fn to_zenoh_config_with_all_routers(&self) -> ZenohConfig {
        let mut connect = vec![self.primary_router.clone()];
        connect.extend(self.backup_routers.clone());

        ZenohConfig {
            mode: ZenohMode::Client,
            connect,
            session_timeout: self.connection_timeout,
            namespace: self.namespace.clone().or_else(|| Some("horus".to_string())),
            ..Default::default()
        }
    }

    /// Discover Zenoh routers via mDNS
    ///
    /// If `auto_discovery` is enabled, this function will search for Zenoh routers
    /// advertised on the local network via mDNS/DNS-SD. It returns a list of
    /// discovered router endpoints that can be used for connection.
    ///
    /// The discovery uses the service type configured in `discovery_service_type`
    /// (default: `_horus-zenoh._tcp.local`).
    ///
    /// # Returns
    /// - `Ok(routers)` - List of discovered router endpoints (e.g., `["tcp/192.168.1.100:7447"]`)
    /// - `Err(_)` - If auto-discovery is disabled or mDNS feature is not enabled
    ///
    /// # Example
    /// ```rust,ignore
    /// let config = ZenohCloudConfig::auto();
    /// let routers = config.discover_routers()?;
    /// for router in routers {
    ///     println!("Discovered router: {}", router);
    /// }
    /// ```
    #[cfg(feature = "mdns")]
    pub fn discover_routers(&self) -> Result<Vec<String>, String> {
        if !self.auto_discovery {
            return Err("Auto-discovery is disabled for this configuration".to_string());
        }

        discover_zenoh_routers_mdns(&self.discovery_service_type, self.connection_timeout)
    }

    /// Discover routers and merge with configured routers
    ///
    /// This method discovers routers via mDNS and merges them with the
    /// pre-configured routers. Discovered routers are added to the backup
    /// list, while the primary router remains unchanged.
    ///
    /// # Returns
    /// A new `ZenohCloudConfig` with discovered routers added
    #[cfg(feature = "mdns")]
    pub fn with_discovered_routers(mut self) -> Self {
        if !self.auto_discovery {
            return self;
        }

        if let Ok(discovered) = self.discover_routers() {
            for router in discovered {
                if router != self.primary_router && !self.backup_routers.contains(&router) {
                    log::info!("Discovered Zenoh router via mDNS: {}", router);
                    self.backup_routers.push(router);
                }
            }
        } else {
            log::debug!("No Zenoh routers discovered via mDNS");
        }

        self
    }

    /// Non-mDNS version - just returns configured routers
    #[cfg(not(feature = "mdns"))]
    pub fn discover_routers(&self) -> Result<Vec<String>, String> {
        Err(
            "mDNS feature not enabled. Compile with --features mdns to enable auto-discovery"
                .to_string(),
        )
    }

    /// Non-mDNS version - just returns self unchanged
    #[cfg(not(feature = "mdns"))]
    pub fn with_discovered_routers(self) -> Self {
        self
    }
}

/// Discover Zenoh routers via mDNS/DNS-SD
///
/// This function browses for services of the specified type and returns
/// endpoints in Zenoh format (e.g., `tcp/192.168.1.100:7447`).
///
/// # Arguments
/// * `service_type` - DNS-SD service type (e.g., `_horus-zenoh._tcp.local`)
/// * `timeout` - Maximum time to spend discovering
///
/// # Returns
/// List of discovered router endpoints
#[cfg(feature = "mdns")]
pub fn discover_zenoh_routers_mdns(
    service_type: &str,
    timeout: std::time::Duration,
) -> Result<Vec<String>, String> {
    use mdns_sd::{ServiceDaemon, ServiceEvent};

    log::info!(
        "Discovering Zenoh routers via mDNS (service: {}, timeout: {:?})",
        service_type,
        timeout
    );

    let mdns = ServiceDaemon::new().map_err(|e| format!("Failed to create mDNS daemon: {}", e))?;

    let receiver = mdns
        .browse(service_type)
        .map_err(|e| format!("Failed to browse for services: {}", e))?;

    let mut routers = Vec::new();
    let start = std::time::Instant::now();

    while start.elapsed() < timeout {
        // Non-blocking receive with short timeout
        // mdns_sd uses flume channels, not std::sync::mpsc
        match receiver.recv_timeout(std::time::Duration::from_millis(100)) {
            Ok(event) => match event {
                ServiceEvent::ServiceResolved(info) => {
                    // Extract port (default to 7447 if not found)
                    let port = info.get_port();

                    // Get all addresses
                    for addr in info.get_addresses() {
                        let endpoint = if addr.is_ipv4() {
                            format!("tcp/{}:{}", addr, port)
                        } else {
                            format!("tcp/[{}]:{}", addr, port)
                        };

                        if !routers.contains(&endpoint) {
                            log::info!(
                                "Discovered Zenoh router: {} ({})",
                                info.get_fullname(),
                                endpoint
                            );
                            routers.push(endpoint);
                        }
                    }
                }
                ServiceEvent::SearchStarted(_) => {
                    log::debug!("mDNS search started for {}", service_type);
                }
                ServiceEvent::ServiceFound(_, fullname) => {
                    log::debug!("Found service: {}", fullname);
                }
                ServiceEvent::ServiceRemoved(_, fullname) => {
                    log::debug!("Service removed: {}", fullname);
                }
                ServiceEvent::SearchStopped(_) => {
                    log::debug!("mDNS search stopped");
                    break;
                }
            },
            Err(e) => {
                // Handle timeout (continue searching) or disconnection (stop)
                let err_str = format!("{:?}", e);
                if err_str.contains("Timeout") {
                    // Continue searching
                } else {
                    // Disconnected or other error
                    log::debug!("mDNS receiver error: {:?}", e);
                    break;
                }
            }
        }
    }

    // Stop browsing
    if let Err(e) = mdns.stop_browse(service_type) {
        log::warn!("Failed to stop mDNS browse: {}", e);
    }

    log::info!("mDNS discovery complete: found {} router(s)", routers.len());

    Ok(routers)
}

/// Connection quality metrics for Zenoh cloud connections
#[derive(Debug, Clone, Default)]
pub struct ZenohConnectionQuality {
    /// Current latency in milliseconds
    pub latency_ms: f64,

    /// Average latency over the monitoring window
    pub avg_latency_ms: f64,

    /// Maximum latency observed
    pub max_latency_ms: f64,

    /// Minimum latency observed
    pub min_latency_ms: f64,

    /// Packet loss percentage (0-100)
    pub packet_loss_percent: f64,

    /// Jitter (latency variance) in milliseconds
    pub jitter_ms: f64,

    /// Number of successful health checks
    pub successful_health_checks: u64,

    /// Number of failed health checks
    pub failed_health_checks: u64,

    /// Current connection state
    pub state: ConnectionQualityState,

    /// Currently connected router endpoint
    pub connected_router: Option<String>,

    /// Number of failovers that have occurred
    pub failover_count: u32,

    /// Timestamp of last successful health check
    pub last_health_check: Option<std::time::Instant>,

    /// Total bytes sent
    pub bytes_sent: u64,

    /// Total bytes received
    pub bytes_received: u64,

    /// Messages sent per second (rolling average)
    pub messages_per_second: f64,
}

impl ZenohConnectionQuality {
    /// Create a new connection quality tracker
    pub fn new() -> Self {
        Self::default()
    }

    /// Check if the connection quality is good
    pub fn is_good(&self) -> bool {
        matches!(self.state, ConnectionQualityState::Good)
    }

    /// Check if the connection quality is degraded
    pub fn is_degraded(&self) -> bool {
        matches!(self.state, ConnectionQualityState::Degraded)
    }

    /// Check if the connection has failed
    pub fn is_failed(&self) -> bool {
        matches!(self.state, ConnectionQualityState::Failed)
    }

    /// Get overall health score (0-100)
    pub fn health_score(&self) -> u32 {
        // Base score of 100, subtract for issues
        let mut score = 100i32;

        // Latency penalty (up to 30 points)
        if self.avg_latency_ms > 500.0 {
            score -= 30;
        } else if self.avg_latency_ms > 200.0 {
            score -= 20;
        } else if self.avg_latency_ms > 100.0 {
            score -= 10;
        }

        // Packet loss penalty (up to 40 points)
        if self.packet_loss_percent > 10.0 {
            score -= 40;
        } else if self.packet_loss_percent > 5.0 {
            score -= 25;
        } else if self.packet_loss_percent > 1.0 {
            score -= 10;
        }

        // Jitter penalty (up to 20 points)
        if self.jitter_ms > 50.0 {
            score -= 20;
        } else if self.jitter_ms > 20.0 {
            score -= 10;
        }

        // Failover penalty (up to 10 points)
        if self.failover_count > 5 {
            score -= 10;
        } else if self.failover_count > 0 {
            score -= 5;
        }

        score.max(0) as u32
    }

    /// Update latency measurement
    pub fn update_latency(&mut self, latency_ms: f64) {
        self.latency_ms = latency_ms;

        // Update rolling average (simple exponential moving average)
        const ALPHA: f64 = 0.2;
        if self.avg_latency_ms == 0.0 {
            self.avg_latency_ms = latency_ms;
        } else {
            self.avg_latency_ms = ALPHA * latency_ms + (1.0 - ALPHA) * self.avg_latency_ms;
        }

        // Update min/max
        if latency_ms > self.max_latency_ms {
            self.max_latency_ms = latency_ms;
        }
        if self.min_latency_ms == 0.0 || latency_ms < self.min_latency_ms {
            self.min_latency_ms = latency_ms;
        }

        // Update jitter (simplified)
        let diff = (latency_ms - self.avg_latency_ms).abs();
        self.jitter_ms = ALPHA * diff + (1.0 - ALPHA) * self.jitter_ms;
    }

    /// Record a successful health check
    pub fn record_health_check_success(&mut self) {
        self.successful_health_checks += 1;
        self.last_health_check = Some(std::time::Instant::now());

        // Improve state if it was degraded
        if matches!(self.state, ConnectionQualityState::Degraded) {
            self.state = ConnectionQualityState::Good;
        }
    }

    /// Record a failed health check
    pub fn record_health_check_failure(&mut self) {
        self.failed_health_checks += 1;

        // Degrade state
        match self.state {
            ConnectionQualityState::Good => {
                self.state = ConnectionQualityState::Degraded;
            }
            ConnectionQualityState::Degraded => {
                // Stay degraded, let external logic decide on failover
            }
            ConnectionQualityState::Failed => {
                // Already failed
            }
            ConnectionQualityState::Connecting => {
                // Still connecting
            }
            ConnectionQualityState::Disconnected => {
                // Already disconnected
            }
        }
    }

    /// Record a failover event
    pub fn record_failover(&mut self, new_router: &str) {
        self.failover_count += 1;
        self.connected_router = Some(new_router.to_string());
        self.state = ConnectionQualityState::Connecting;
    }

    /// Mark connection as established
    pub fn mark_connected(&mut self, router: &str) {
        self.connected_router = Some(router.to_string());
        self.state = ConnectionQualityState::Good;
    }

    /// Mark connection as failed
    pub fn mark_failed(&mut self) {
        self.state = ConnectionQualityState::Failed;
    }

    /// Mark connection as disconnected
    pub fn mark_disconnected(&mut self) {
        self.state = ConnectionQualityState::Disconnected;
        self.connected_router = None;
    }
}

/// Connection quality state
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum ConnectionQualityState {
    /// Connection is good - low latency, no packet loss
    Good,
    /// Connection is degraded - high latency or some packet loss
    Degraded,
    /// Connection has failed
    Failed,
    /// Currently attempting to connect
    #[default]
    Connecting,
    /// Disconnected (not attempting to connect)
    Disconnected,
}

impl std::fmt::Display for ConnectionQualityState {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Good => write!(f, "Good"),
            Self::Degraded => write!(f, "Degraded"),
            Self::Failed => write!(f, "Failed"),
            Self::Connecting => write!(f, "Connecting"),
            Self::Disconnected => write!(f, "Disconnected"),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_default_config() {
        let config = ZenohConfig::default();
        assert_eq!(config.namespace, Some("horus".to_string()));
        assert!(!config.ros2_mode);
        assert!(config.shared_memory);
    }

    #[test]
    fn test_topic_to_key_expr() {
        let config = ZenohConfig::default();
        assert_eq!(config.topic_to_key_expr("odom"), "horus/odom");
        assert_eq!(
            config.topic_to_key_expr("sensors/lidar"),
            "horus/sensors/lidar"
        );
    }

    #[test]
    fn test_ros2_topic_naming() {
        let config = ZenohConfig::ros2(0);
        assert_eq!(config.topic_to_key_expr("/cmd_vel"), "rt/cmd_vel");
        assert_eq!(config.topic_to_key_expr("odom"), "rt/odom");
    }

    #[test]
    fn test_key_expr_to_topic() {
        let config = ZenohConfig::default();
        assert_eq!(
            config.key_expr_to_topic("horus/odom"),
            Some("odom".to_string())
        );

        let ros2_config = ZenohConfig::ros2(0);
        assert_eq!(
            ros2_config.key_expr_to_topic("rt/cmd_vel"),
            Some("cmd_vel".to_string())
        );
    }

    #[test]
    fn test_cloud_config() {
        let config = ZenohConfig::cloud("tcp/cloud.example.com:7447");
        assert_eq!(config.mode, ZenohMode::Client);
        assert_eq!(config.connect, vec!["tcp/cloud.example.com:7447"]);
    }

    #[test]
    fn test_builder_pattern() {
        let config = ZenohConfig::new()
            .with_namespace("robot1")
            .connect_to("tcp/192.168.1.100:7447")
            .listen_on("tcp/0.0.0.0:7447");

        assert_eq!(config.namespace, Some("robot1".to_string()));
        assert_eq!(config.connect.len(), 1);
        assert_eq!(config.listen.len(), 1);
    }

    #[test]
    fn test_qos_presets() {
        let rt_qos = ZenohQos::realtime();
        assert!(rt_qos.express);
        assert_eq!(rt_qos.priority, 7);

        let reliable_qos = ZenohQos::reliable();
        assert_eq!(reliable_qos.reliability, Reliability::Reliable);
    }

    #[test]
    fn test_qos_deadline() {
        let qos = ZenohQos::default().with_deadline(std::time::Duration::from_millis(100));
        assert_eq!(qos.deadline, Some(std::time::Duration::from_millis(100)));
    }

    #[test]
    fn test_qos_lifespan() {
        let qos = ZenohQos::default().with_lifespan(std::time::Duration::from_secs(60));
        assert_eq!(qos.lifespan, Some(std::time::Duration::from_secs(60)));
    }

    #[test]
    fn test_qos_liveliness() {
        let qos = ZenohQos::default().with_liveliness(
            Liveliness::ManualByTopic,
            Some(std::time::Duration::from_secs(1)),
        );
        assert_eq!(qos.liveliness, Liveliness::ManualByTopic);
        assert_eq!(
            qos.liveliness_lease_duration,
            Some(std::time::Duration::from_secs(1))
        );
    }

    #[test]
    fn test_qos_history() {
        let keep_last = ZenohQos::default().with_history(HistoryPolicy::KeepLast(10));
        assert_eq!(keep_last.history, HistoryPolicy::KeepLast(10));

        let keep_all = ZenohQos::default().with_history(HistoryPolicy::KeepAll);
        assert_eq!(keep_all.history, HistoryPolicy::KeepAll);
    }

    #[test]
    fn test_qos_durability() {
        let volatile = ZenohQos::default().with_durability(Durability::Volatile);
        assert_eq!(volatile.durability, Durability::Volatile);

        let transient = ZenohQos::default().with_durability(Durability::TransientLocal);
        assert_eq!(transient.durability, Durability::TransientLocal);
    }

    #[test]
    fn test_qos_sensor_preset() {
        let qos = ZenohQos::sensor_data();
        assert_eq!(qos.reliability, Reliability::BestEffort);
        assert_eq!(qos.history, HistoryPolicy::KeepLast(5));
        assert!(qos.deadline.is_some()); // Has deadline constraint
        assert!(qos.lifespan.is_some()); // Has lifespan constraint
    }

    #[test]
    fn test_qos_parameters_preset() {
        let qos = ZenohQos::parameters();
        assert_eq!(qos.reliability, Reliability::Reliable);
        assert_eq!(qos.durability, Durability::TransientLocal);
    }

    #[test]
    fn test_qos_services_preset() {
        let qos = ZenohQos::services();
        assert_eq!(qos.reliability, Reliability::Reliable);
        assert_eq!(qos.history, HistoryPolicy::KeepAll); // Services need all messages
    }

    #[test]
    fn test_qos_actions_preset() {
        let qos = ZenohQos::actions();
        assert_eq!(qos.reliability, Reliability::Reliable);
        assert_eq!(qos.history, HistoryPolicy::KeepLast(10)); // Actions use bounded history
    }

    // ZenohCloudConfig tests

    #[test]
    fn test_zenoh_cloud_config_default() {
        let config = ZenohCloudConfig::default();
        assert_eq!(config.primary_router, HORUS_CLOUD_ROUTER);
        assert_eq!(config.backup_routers.len(), 2);
        assert!(config.auto_discovery);
        assert!(config.enable_monitoring);
        assert!(config.enable_compression);
        assert!(config.enable_encryption);
    }

    #[test]
    fn test_zenoh_cloud_config_auto() {
        let config = ZenohCloudConfig::auto();
        assert!(config.auto_discovery);
    }

    #[test]
    fn test_zenoh_cloud_config_horus_cloud() {
        let config = ZenohCloudConfig::horus_cloud();
        assert_eq!(config.primary_router, HORUS_CLOUD_ROUTER);
    }

    #[test]
    fn test_zenoh_cloud_config_self_hosted() {
        let config = ZenohCloudConfig::self_hosted("tcp/my-router.local:7447");
        assert_eq!(config.primary_router, "tcp/my-router.local:7447");
        assert!(config.backup_routers.is_empty());
        assert!(!config.auto_discovery);
    }

    #[test]
    fn test_zenoh_cloud_config_with_routers() {
        let config = ZenohCloudConfig::new().with_routers(&[
            "tcp/primary.example.com:7447",
            "tcp/backup1.example.com:7447",
            "tcp/backup2.example.com:7447",
        ]);
        assert_eq!(config.primary_router, "tcp/primary.example.com:7447");
        assert_eq!(config.backup_routers.len(), 2);
        assert_eq!(config.backup_routers[0], "tcp/backup1.example.com:7447");
        assert_eq!(config.backup_routers[1], "tcp/backup2.example.com:7447");
    }

    #[test]
    fn test_zenoh_cloud_config_builder() {
        let config = ZenohCloudConfig::new()
            .with_auto_discovery(false)
            .with_connection_timeout(Duration::from_secs(20))
            .with_failover_timeout(Duration::from_secs(10))
            .with_monitoring(true)
            .with_latency_thresholds(50, 200)
            .with_compression(false)
            .with_encryption(true)
            .with_namespace("my_namespace");

        assert!(!config.auto_discovery);
        assert_eq!(config.connection_timeout, Duration::from_secs(20));
        assert_eq!(config.failover_timeout, Duration::from_secs(10));
        assert!(config.enable_monitoring);
        assert_eq!(config.latency_warn_threshold_ms, 50);
        assert_eq!(config.latency_failover_threshold_ms, 200);
        assert!(!config.enable_compression);
        assert!(config.enable_encryption);
        assert_eq!(config.namespace, Some("my_namespace".to_string()));
    }

    #[test]
    fn test_zenoh_cloud_config_all_routers() {
        let config =
            ZenohCloudConfig::new().with_routers(&["tcp/a:7447", "tcp/b:7447", "tcp/c:7447"]);
        let all = config.all_routers();
        assert_eq!(all.len(), 3);
        assert_eq!(all[0], "tcp/a:7447");
        assert_eq!(all[1], "tcp/b:7447");
        assert_eq!(all[2], "tcp/c:7447");
    }

    #[test]
    fn test_zenoh_cloud_config_to_zenoh_config() {
        let cloud_config =
            ZenohCloudConfig::self_hosted("tcp/router.local:7447").with_namespace("robot1");
        let zenoh_config = cloud_config.to_zenoh_config();

        assert_eq!(zenoh_config.mode, ZenohMode::Client);
        assert_eq!(zenoh_config.connect, vec!["tcp/router.local:7447"]);
        assert_eq!(zenoh_config.namespace, Some("robot1".to_string()));
    }

    #[test]
    fn test_zenoh_cloud_config_to_zenoh_config_with_all_routers() {
        let cloud_config = ZenohCloudConfig::new().with_routers(&["tcp/a:7447", "tcp/b:7447"]);
        let zenoh_config = cloud_config.to_zenoh_config_with_all_routers();

        assert_eq!(zenoh_config.connect.len(), 2);
        assert_eq!(zenoh_config.connect[0], "tcp/a:7447");
        assert_eq!(zenoh_config.connect[1], "tcp/b:7447");
    }

    // ZenohConnectionQuality tests

    #[test]
    fn test_connection_quality_new() {
        let quality = ZenohConnectionQuality::new();
        assert_eq!(quality.latency_ms, 0.0);
        assert_eq!(quality.failover_count, 0);
        assert!(matches!(quality.state, ConnectionQualityState::Connecting));
    }

    #[test]
    fn test_connection_quality_state_checks() {
        let mut quality = ZenohConnectionQuality::new();

        quality.state = ConnectionQualityState::Good;
        assert!(quality.is_good());
        assert!(!quality.is_degraded());
        assert!(!quality.is_failed());

        quality.state = ConnectionQualityState::Degraded;
        assert!(!quality.is_good());
        assert!(quality.is_degraded());
        assert!(!quality.is_failed());

        quality.state = ConnectionQualityState::Failed;
        assert!(!quality.is_good());
        assert!(!quality.is_degraded());
        assert!(quality.is_failed());
    }

    #[test]
    fn test_connection_quality_update_latency() {
        let mut quality = ZenohConnectionQuality::new();

        quality.update_latency(50.0);
        assert_eq!(quality.latency_ms, 50.0);
        assert_eq!(quality.avg_latency_ms, 50.0);
        assert_eq!(quality.min_latency_ms, 50.0);
        assert_eq!(quality.max_latency_ms, 50.0);

        quality.update_latency(100.0);
        assert_eq!(quality.latency_ms, 100.0);
        assert!(quality.avg_latency_ms > 50.0 && quality.avg_latency_ms < 100.0);
        assert_eq!(quality.min_latency_ms, 50.0);
        assert_eq!(quality.max_latency_ms, 100.0);
    }

    #[test]
    fn test_connection_quality_health_score() {
        let mut quality = ZenohConnectionQuality::new();
        quality.state = ConnectionQualityState::Good;

        // Perfect conditions
        quality.avg_latency_ms = 20.0;
        quality.packet_loss_percent = 0.0;
        quality.jitter_ms = 5.0;
        quality.failover_count = 0;
        assert_eq!(quality.health_score(), 100);

        // High latency
        quality.avg_latency_ms = 300.0;
        assert!(quality.health_score() < 100);

        // Packet loss
        quality.avg_latency_ms = 20.0;
        quality.packet_loss_percent = 6.0;
        assert!(quality.health_score() < 100);
    }

    #[test]
    fn test_connection_quality_health_checks() {
        let mut quality = ZenohConnectionQuality::new();
        quality.state = ConnectionQualityState::Good;

        quality.record_health_check_success();
        assert_eq!(quality.successful_health_checks, 1);
        assert!(quality.last_health_check.is_some());

        quality.record_health_check_failure();
        assert_eq!(quality.failed_health_checks, 1);
        assert!(matches!(quality.state, ConnectionQualityState::Degraded));
    }

    #[test]
    fn test_connection_quality_failover() {
        let mut quality = ZenohConnectionQuality::new();
        quality.mark_connected("tcp/router1:7447");

        assert!(quality.is_good());
        assert_eq!(
            quality.connected_router,
            Some("tcp/router1:7447".to_string())
        );

        quality.record_failover("tcp/router2:7447");
        assert_eq!(quality.failover_count, 1);
        assert_eq!(
            quality.connected_router,
            Some("tcp/router2:7447".to_string())
        );
        assert!(matches!(quality.state, ConnectionQualityState::Connecting));
    }

    #[test]
    fn test_connection_quality_state_display() {
        assert_eq!(format!("{}", ConnectionQualityState::Good), "Good");
        assert_eq!(format!("{}", ConnectionQualityState::Degraded), "Degraded");
        assert_eq!(format!("{}", ConnectionQualityState::Failed), "Failed");
        assert_eq!(
            format!("{}", ConnectionQualityState::Connecting),
            "Connecting"
        );
        assert_eq!(
            format!("{}", ConnectionQualityState::Disconnected),
            "Disconnected"
        );
    }
}
