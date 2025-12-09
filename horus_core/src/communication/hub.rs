use crate::communication::network::{parse_endpoint, Endpoint, NetworkBackend};
use crate::core::node::NodeInfo;
use crate::error::HorusResult;
use crate::memory::shm_topic::ShmTopic;
use std::sync::Arc;
use std::time::Instant;

/// Connection state for Hub connections
#[derive(Debug, Clone, PartialEq)]
pub enum ConnectionState {
    Disconnected,
    Connecting,
    Connected,
    Reconnecting,
    Failed,
}

/// Lock-free atomic metrics for Hub monitoring with cache optimization
#[derive(Debug)]
#[repr(align(64))] // Cache-line aligned to prevent false sharing
pub struct AtomicHubMetrics {
    pub messages_sent: std::sync::atomic::AtomicU64,
    pub messages_received: std::sync::atomic::AtomicU64,
    pub send_failures: std::sync::atomic::AtomicU64,
    pub recv_failures: std::sync::atomic::AtomicU64,
    _padding: [u8; 32], // Pad to cache line boundary
}

impl Default for AtomicHubMetrics {
    fn default() -> Self {
        Self {
            messages_sent: std::sync::atomic::AtomicU64::new(0),
            messages_received: std::sync::atomic::AtomicU64::new(0),
            send_failures: std::sync::atomic::AtomicU64::new(0),
            recv_failures: std::sync::atomic::AtomicU64::new(0),
            _padding: [0; 32],
        }
    }
}

impl AtomicHubMetrics {
    /// Get current metrics snapshot (for monitoring/debugging)
    pub fn snapshot(&self) -> HubMetrics {
        HubMetrics {
            messages_sent: self
                .messages_sent
                .load(std::sync::atomic::Ordering::Relaxed),
            messages_received: self
                .messages_received
                .load(std::sync::atomic::Ordering::Relaxed),
            send_failures: self
                .send_failures
                .load(std::sync::atomic::Ordering::Relaxed),
            recv_failures: self
                .recv_failures
                .load(std::sync::atomic::Ordering::Relaxed),
            last_activity: None, // Eliminated to remove Instant::now() overhead
        }
    }
}

/// Simple metrics for Hub monitoring (for backwards compatibility)
#[derive(Debug, Clone, Default)]
pub struct HubMetrics {
    pub messages_sent: u64,
    pub messages_received: u64,
    pub send_failures: u64,
    pub recv_failures: u64,
    pub last_activity: Option<Instant>,
}

/// Optimized Hub for pub/sub messaging with cache-aligned lock-free hot paths
#[repr(align(64))] // Cache-line aligned structure
pub struct Hub<T> {
    shm_topic: Option<Arc<ShmTopic<T>>>, // Local shared memory (None for network-only endpoints)
    network: Option<std::sync::Mutex<NetworkBackend<T>>>, // Optional network backend (needs Mutex for recv)
    is_network: bool,                                     // Fast dispatch flag
    topic_name: String,
    state: std::sync::atomic::AtomicU8, // Lock-free state using atomic u8
    metrics: Arc<AtomicHubMetrics>,     // Lock-free atomic metrics
    _padding: [u8; 14],                 // Pad to prevent false sharing
}

// Manual Clone implementation since AtomicU8 doesn't implement Clone
impl<T> Clone for Hub<T>
where
    T: Clone,
{
    fn clone(&self) -> Self {
        Self {
            shm_topic: self.shm_topic.clone(),
            network: None, // Network backends are not cloneable (contain sockets, etc.)
            is_network: self.is_network,
            topic_name: self.topic_name.clone(),
            state: std::sync::atomic::AtomicU8::new(
                self.state.load(std::sync::atomic::Ordering::Relaxed),
            ),
            metrics: self.metrics.clone(),
            _padding: [0; 14],
        }
    }
}

// Manual Debug implementation to avoid ShmTopic Debug requirement
impl<T> std::fmt::Debug for Hub<T> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("Hub")
            .field("topic_name", &self.topic_name)
            .field(
                "state",
                &self.state.load(std::sync::atomic::Ordering::Relaxed),
            )
            .finish_non_exhaustive()
    }
}

// Helper functions for state conversion
impl ConnectionState {
    fn into_u8(self) -> u8 {
        match self {
            ConnectionState::Disconnected => 0,
            ConnectionState::Connecting => 1,
            ConnectionState::Connected => 2,
            ConnectionState::Reconnecting => 3,
            ConnectionState::Failed => 4,
        }
    }

    fn from_u8(value: u8) -> Self {
        match value {
            0 => ConnectionState::Disconnected,
            1 => ConnectionState::Connecting,
            2 => ConnectionState::Connected,
            3 => ConnectionState::Reconnecting,
            _ => ConnectionState::Failed,
        }
    }
}

impl<
        T: Send
            + Sync
            + 'static
            + Clone
            + std::fmt::Debug
            + serde::Serialize
            + serde::de::DeserializeOwned,
    > Hub<T>
{
    /// Create a new Hub
    pub fn new(topic_name: &str) -> HorusResult<Self> {
        Self::new_with_capacity(topic_name, 1024)
    }

    /// Create a Hub from configuration file
    ///
    /// Loads hub configuration from TOML/YAML file and creates the hub with the specified settings.
    ///
    /// # Arguments
    /// * `hub_name` - Name of the hub to look up in the config file
    ///
    /// # Config File Format
    ///
    /// TOML example:
    /// ```toml
    /// [hubs.camera]
    /// name = "camera"
    /// endpoint = "camera@router"
    ///
    /// [hubs.sensor]
    /// name = "sensor"
    /// transport = "direct"
    /// host = "192.168.1.5"
    /// port = 9000
    /// ```
    ///
    /// YAML example:
    /// ```yaml
    /// hubs:
    ///   camera:
    ///     name: camera
    ///     endpoint: camera@router
    ///   sensor:
    ///     name: sensor
    ///     transport: direct
    ///     host: 192.168.1.5
    ///     port: 9000
    /// ```
    ///
    /// # Config File Search Paths
    /// 1. `./horus.toml` or `./horus.yaml`
    /// 2. `~/.horus/config.toml` or `~/.horus/config.yaml`
    /// 3. `/etc/horus/config.toml` or `/etc/horus/config.yaml`
    pub fn from_config(hub_name: &str) -> HorusResult<Self> {
        use crate::communication::config::HorusConfig;

        // Load config from standard search paths
        let config = HorusConfig::find_and_load()?;

        // Get hub config
        let hub_config = config.get_hub(hub_name)?;

        // Get endpoint string
        let endpoint_str = hub_config.get_endpoint();

        // Create hub with the endpoint
        Self::new(&endpoint_str)
    }

    /// Create a Hub from a specific config file path
    ///
    /// # Arguments
    /// * `config_path` - Path to the configuration file (TOML or YAML)
    /// * `hub_name` - Name of the hub to look up in the config file
    pub fn from_config_file<P: AsRef<std::path::Path>>(
        config_path: P,
        hub_name: &str,
    ) -> HorusResult<Self> {
        use crate::communication::config::HorusConfig;

        // Load config from specific file
        let config = HorusConfig::from_file(config_path)?;

        // Get hub config
        let hub_config = config.get_hub(hub_name)?;

        // Get endpoint string
        let endpoint_str = hub_config.get_endpoint();

        // Create hub with the endpoint
        Self::new(&endpoint_str)
    }

    /// Create a new Hub with custom capacity
    ///
    /// Supports both local and network endpoints:
    /// - `"topic"` → Local shared memory
    /// - `"topic@localhost"` → Localhost (future: Unix socket or shared memory)
    /// - `"topic@192.168.1.5"` → Direct network (future: UDP)
    /// - `"topic@192.168.1.5:9000"` → Direct network with custom port
    /// - `"topic@*"` → Multicast discovery (future)
    ///
    /// Note: Network endpoints require T: serde::Serialize + serde::de::DeserializeOwned
    pub fn new_with_capacity(topic_name: &str, capacity: usize) -> HorusResult<Self> {
        // Parse endpoint
        let endpoint = parse_endpoint(topic_name)?;

        match endpoint {
            Endpoint::Local { topic } => {
                // Fast path: local shared memory only
                let shm_topic = Arc::new(ShmTopic::new(&topic, capacity)?);

                Ok(Hub {
                    shm_topic: Some(shm_topic),
                    network: None,
                    is_network: false,
                    topic_name: topic_name.to_string(),
                    state: std::sync::atomic::AtomicU8::new(ConnectionState::Connected.into_u8()),
                    metrics: Arc::new(AtomicHubMetrics::default()),
                    _padding: [0; 14],
                })
            }

            // Network endpoints - no shared memory allocated (avoids wasting resources)
            network_endpoint => {
                let network_backend = NetworkBackend::new(network_endpoint)?;

                Ok(Hub {
                    shm_topic: None, // Network-only: no local shared memory needed
                    network: Some(std::sync::Mutex::new(network_backend)),
                    is_network: true,
                    topic_name: topic_name.to_string(),
                    state: std::sync::atomic::AtomicU8::new(ConnectionState::Connected.into_u8()),
                    metrics: Arc::new(AtomicHubMetrics::default()),
                    _padding: [0; 14],
                })
            }
        }
    }

    /// High-performance send using zero-copy loan pattern internally
    /// This method now uses the loan() backend for optimal performance (~200ns latency)
    /// The API remains simple while delivering the best possible performance
    ///
    /// Supports both local shared memory and network backends transparently
    ///
    /// Note: Network endpoints require T: serde::Serialize
    #[inline(always)]
    pub fn send(&self, msg: T, ctx: &mut Option<&mut NodeInfo>) -> Result<(), T>
    where
        T: crate::core::LogSummary,
    {
        // Network path (if network backend is present)
        if self.is_network {
            if let Some(ref network_mutex) = self.network {
                let network = network_mutex.lock().expect(
                    "Network mutex lock poisoned - another thread panicked while holding the lock",
                );
                match network.send(&msg) {
                    Ok(_) => {
                        self.metrics
                            .messages_sent
                            .fetch_add(1, std::sync::atomic::Ordering::Relaxed);
                        self.state.store(
                            ConnectionState::Connected.into_u8(),
                            std::sync::atomic::Ordering::Relaxed,
                        );

                        if let Some(ref mut ctx) = ctx {
                            ctx.register_publisher(&self.topic_name, std::any::type_name::<T>());
                            let summary = msg.log_summary();
                            ctx.log_pub_summary(&self.topic_name, &summary, 0);
                        }

                        return Ok(());
                    }
                    Err(_) => {
                        self.metrics
                            .send_failures
                            .fetch_add(1, std::sync::atomic::Ordering::Relaxed);
                        self.state.store(
                            ConnectionState::Failed.into_u8(),
                            std::sync::atomic::Ordering::Relaxed,
                        );
                        return Err(msg);
                    }
                }
            }
            // Shouldn't happen (is_network true but no network backend), fall through to shm
        }

        // Local shared memory path (OPTIMIZED - time only IPC)
        let shm_topic = match &self.shm_topic {
            Some(topic) => topic,
            None => {
                // Network hub incorrectly fell through to shm path - this is a bug
                self.metrics
                    .send_failures
                    .fetch_add(1, std::sync::atomic::Ordering::Relaxed);
                self.state.store(
                    ConnectionState::Failed.into_u8(),
                    std::sync::atomic::Ordering::Relaxed,
                );
                return Err(msg);
            }
        };

        match shm_topic.loan() {
            Ok(mut sample) => {
                // Fast path: when ctx is None (benchmarks), bypass logging completely
                if let Some(ref mut ctx) = ctx {
                    // Register as publisher for discovery (only stores once per topic)
                    ctx.register_publisher(&self.topic_name, std::any::type_name::<T>());

                    // Logging enabled: get lightweight summary BEFORE moving msg
                    let summary = msg.log_summary();

                    // TIME ONLY THE ACTUAL IPC OPERATION
                    let ipc_start = Instant::now();
                    sample.write(msg);
                    drop(sample);
                    let ipc_ns = ipc_start.elapsed().as_nanos() as u64;
                    // END TIMING - everything after this is logging overhead

                    // Post-IPC operations (not timed - happen after IPC completes)
                    self.metrics
                        .messages_sent
                        .fetch_add(1, std::sync::atomic::Ordering::Relaxed);
                    self.state.store(
                        ConnectionState::Connected.into_u8(),
                        std::sync::atomic::Ordering::Relaxed,
                    );

                    // Log with accurate IPC timing
                    ctx.log_pub_summary(&self.topic_name, &summary, ipc_ns);
                } else {
                    // No logging: zero overhead path for benchmarks
                    sample.write(msg);
                    drop(sample);

                    self.metrics
                        .messages_sent
                        .fetch_add(1, std::sync::atomic::Ordering::Relaxed);
                    self.state.store(
                        ConnectionState::Connected.into_u8(),
                        std::sync::atomic::Ordering::Relaxed,
                    );
                }

                Ok(())
            }
            Err(_) => {
                self.metrics
                    .send_failures
                    .fetch_add(1, std::sync::atomic::Ordering::Relaxed);
                self.state.store(
                    ConnectionState::Failed.into_u8(),
                    std::sync::atomic::Ordering::Relaxed,
                );
                Err(msg)
            }
        }
    }
    /// Receive a message from the topic
    ///
    /// Supports both local shared memory and network backends transparently
    ///
    /// Note: Network endpoints require T: serde::de::DeserializeOwned
    #[inline(always)]
    pub fn recv(&self, ctx: &mut Option<&mut NodeInfo>) -> Option<T>
    where
        T: crate::core::LogSummary,
    {
        // Network path (if network backend is present)
        if self.is_network {
            if let Some(ref network_mutex) = self.network {
                let mut network = network_mutex.lock().expect(
                    "Network mutex lock poisoned - another thread panicked while holding the lock",
                );
                if let Some(msg) = network.recv() {
                    self.metrics
                        .messages_received
                        .fetch_add(1, std::sync::atomic::Ordering::Relaxed);

                    if let Some(ref mut ctx) = ctx {
                        ctx.register_subscriber(&self.topic_name, std::any::type_name::<T>());
                        let summary = msg.log_summary();
                        ctx.log_sub_summary(&self.topic_name, &summary, 0);
                    }

                    return Some(msg);
                } else {
                    self.metrics
                        .recv_failures
                        .fetch_add(1, std::sync::atomic::Ordering::Relaxed);
                    return None;
                }
            }
            // Shouldn't happen (is_network true but no network backend), fall through to shm
        }

        // Local shared memory path (ZERO-COPY OPTIMIZED)
        let shm_topic = match &self.shm_topic {
            Some(topic) => topic,
            None => {
                // Network hub incorrectly fell through to shm path - this is a bug
                self.metrics
                    .recv_failures
                    .fetch_add(1, std::sync::atomic::Ordering::Relaxed);
                return None;
            }
        };

        // TIME ONLY THE ACTUAL IPC OPERATION
        let ipc_start = Instant::now();
        match shm_topic.receive() {
            Some(sample) => {
                let ipc_ns = ipc_start.elapsed().as_nanos() as u64;
                // END TIMING

                // Fast path: when ctx is None, bypass logging completely (benchmarks + production)
                if let Some(ref mut ctx) = ctx {
                    // Register as subscriber for discovery (only stores once per topic)
                    ctx.register_subscriber(&self.topic_name, std::any::type_name::<T>());

                    // Logging enabled: get summary from zero-copy reference
                    let summary = sample.get_ref().log_summary();
                    ctx.log_sub_summary(&self.topic_name, &summary, ipc_ns);
                }

                // Clone only when needed (after timing and logging)
                let msg = sample.get_ref().clone();

                // Lock-free atomic increment for success metrics
                self.metrics
                    .messages_received
                    .fetch_add(1, std::sync::atomic::Ordering::Relaxed);

                Some(msg)
            }
            None => {
                // Lock-free atomic increment for failure metrics
                self.metrics
                    .recv_failures
                    .fetch_add(1, std::sync::atomic::Ordering::Relaxed);
                None
            }
        }
    }
    /// Get current connection state (lock-free)
    pub fn get_connection_state(&self) -> ConnectionState {
        let state_u8 = self.state.load(std::sync::atomic::Ordering::Relaxed);
        ConnectionState::from_u8(state_u8)
    }

    /// Get current metrics snapshot (lock-free)
    pub fn get_metrics(&self) -> HubMetrics {
        self.metrics.snapshot()
    }

    /// Get the topic name for this Hub
    pub fn get_topic_name(&self) -> &str {
        &self.topic_name
    }
}
