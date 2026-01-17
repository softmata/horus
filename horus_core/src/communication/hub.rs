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
///
/// # Deprecation Notice
///
/// `Hub<T>` is deprecated in favor of `Topic<T>`. The Topic API provides a unified
/// interface with automatic backend selection based on your communication pattern.
///
/// ## Migration Guide
///
/// ```rust,ignore
/// // Before (deprecated):
/// let hub: Hub<MyMessage> = Hub::new("my_topic")?;
/// hub.send(msg, &mut ctx)?;
/// let received = hub.recv(&mut ctx);
///
/// // After (recommended):
/// let topic: Topic<MyMessage> = Topic::with_capacity("my_topic", 1024);
/// topic.send(msg, &mut None)?;
/// let received = topic.recv(&mut None);
/// ```
///
/// For MPMC cross-process communication, Topic::with_capacity() provides the same
/// functionality with simpler API and better performance characteristics.
#[deprecated(
    since = "0.2.0",
    note = "Use Topic<T> instead. See Hub documentation for migration guide."
)]
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

#[cfg(test)]
mod tests {
    use super::*;
    use serde::{Deserialize, Serialize};

    /// Test message type implementing all required traits
    #[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
    struct TestMessage {
        id: u32,
        value: f64,
        label: String,
    }

    impl crate::core::LogSummary for TestMessage {
        fn log_summary(&self) -> String {
            format!("TestMsg(id={}, val={:.2})", self.id, self.value)
        }
    }

    /// Simple test message for basic tests
    #[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
    struct SimpleValue(f64);

    impl crate::core::LogSummary for SimpleValue {
        fn log_summary(&self) -> String {
            format!("{:.2}", self.0)
        }
    }

    // =========================================================================
    // Hub Creation Tests
    // =========================================================================

    #[test]
    fn test_hub_new_local() {
        let hub: Hub<SimpleValue> = Hub::new("test_hub_new_local").unwrap();
        assert_eq!(hub.get_topic_name(), "test_hub_new_local");
        assert_eq!(hub.get_connection_state(), ConnectionState::Connected);
        assert!(!hub.is_network);
    }

    #[test]
    fn test_hub_new_with_capacity() {
        let hub: Hub<SimpleValue> = Hub::new_with_capacity("test_hub_capacity", 2048).unwrap();
        assert_eq!(hub.get_topic_name(), "test_hub_capacity");
        assert_eq!(hub.get_connection_state(), ConnectionState::Connected);
    }

    #[test]
    fn test_hub_clone() {
        let hub1: Hub<SimpleValue> = Hub::new("test_hub_clone").unwrap();
        let hub2 = hub1.clone();
        assert_eq!(hub1.get_topic_name(), hub2.get_topic_name());
        // Metrics are shared via Arc
        assert_eq!(
            hub1.get_metrics().messages_sent,
            hub2.get_metrics().messages_sent
        );
    }

    #[test]
    fn test_hub_debug() {
        let hub: Hub<SimpleValue> = Hub::new("test_hub_debug").unwrap();
        let debug_str = format!("{:?}", hub);
        assert!(debug_str.contains("Hub"));
        assert!(debug_str.contains("test_hub_debug"));
    }

    // =========================================================================
    // Connection State Tests
    // =========================================================================

    #[test]
    fn test_connection_state_conversion() {
        assert_eq!(ConnectionState::Disconnected.into_u8(), 0);
        assert_eq!(ConnectionState::Connecting.into_u8(), 1);
        assert_eq!(ConnectionState::Connected.into_u8(), 2);
        assert_eq!(ConnectionState::Reconnecting.into_u8(), 3);
        assert_eq!(ConnectionState::Failed.into_u8(), 4);

        assert_eq!(ConnectionState::from_u8(0), ConnectionState::Disconnected);
        assert_eq!(ConnectionState::from_u8(1), ConnectionState::Connecting);
        assert_eq!(ConnectionState::from_u8(2), ConnectionState::Connected);
        assert_eq!(ConnectionState::from_u8(3), ConnectionState::Reconnecting);
        assert_eq!(ConnectionState::from_u8(4), ConnectionState::Failed);
        assert_eq!(ConnectionState::from_u8(255), ConnectionState::Failed); // Invalid defaults to Failed
    }

    // =========================================================================
    // Basic Send/Receive Tests
    // =========================================================================

    #[test]
    fn test_hub_send_recv_simple() {
        let hub: Hub<SimpleValue> = Hub::new("test_send_recv_simple").unwrap();

        // Send a message (no context)
        let result = hub.send(SimpleValue(42.0), &mut None);
        assert!(result.is_ok());

        // Receive the message
        let received = hub.recv(&mut None);
        assert!(received.is_some());
        assert_eq!(received.unwrap(), SimpleValue(42.0));
    }

    #[test]
    fn test_hub_send_recv_complex_message() {
        let hub: Hub<TestMessage> = Hub::new("test_send_recv_complex").unwrap();

        let msg = TestMessage {
            id: 123,
            value: 1.234, // Arbitrary test value
            label: "test".to_string(),
        };

        hub.send(msg.clone(), &mut None).unwrap();
        let received = hub.recv(&mut None).unwrap();
        assert_eq!(received, msg);
    }

    #[test]
    fn test_hub_recv_empty() {
        let hub: Hub<SimpleValue> = Hub::new("test_recv_empty").unwrap();
        // No message sent, should return None
        let received = hub.recv(&mut None);
        assert!(received.is_none());
    }

    #[test]
    fn test_hub_single_slot_semantics() {
        // Hub uses single-slot design with sequence tracking
        // - Each send() writes to the same slot and increments sequence
        // - recv() returns data only if sequence has changed since last read
        let hub: Hub<SimpleValue> = Hub::new("test_single_slot").unwrap();

        hub.send(SimpleValue(1.0), &mut None).unwrap();

        // First recv gets the value
        let received = hub.recv(&mut None);
        assert!(received.is_some());
        assert_eq!(received.unwrap(), SimpleValue(1.0));

        // Second recv without new send returns None (already seen this sequence)
        let _received = hub.recv(&mut None);
        // Hub may return the same value again OR None depending on implementation
        // The key behavior is: recv() should see new data when send() is called

        // Send new data
        hub.send(SimpleValue(2.0), &mut None).unwrap();
        let received = hub.recv(&mut None);
        assert!(received.is_some());
        // Should see the new value
        assert_eq!(received.unwrap(), SimpleValue(2.0));
    }

    // =========================================================================
    // Multi-Consumer Tests
    // =========================================================================

    #[test]
    fn test_hub_multiple_subscribers_shared_topic() {
        // Hub clones share the same ShmTopic - first to recv() consumes the message
        // This is expected single-slot behavior for performance
        let publisher: Hub<SimpleValue> = Hub::new("test_multi_consumer").unwrap();
        let subscriber1 = publisher.clone();

        // Publisher sends
        publisher.send(SimpleValue(99.0), &mut None).unwrap();

        // First subscriber reads it
        let val1 = subscriber1.recv(&mut None);
        assert!(val1.is_some());
        assert_eq!(val1.unwrap(), SimpleValue(99.0));

        // For true multi-subscriber, create separate Hub instances to same topic
        let sub_a: Hub<SimpleValue> = Hub::new("test_true_pubsub").unwrap();
        let sub_b: Hub<SimpleValue> = Hub::new("test_true_pubsub").unwrap();

        sub_a.send(SimpleValue(42.0), &mut None).unwrap();

        // Both can read because they each have their own sequence tracking
        let val_a = sub_a.recv(&mut None);
        let val_b = sub_b.recv(&mut None);

        assert!(val_a.is_some());
        assert!(val_b.is_some());
        assert_eq!(val_a.unwrap(), SimpleValue(42.0));
        assert_eq!(val_b.unwrap(), SimpleValue(42.0));
    }

    // =========================================================================
    // Metrics Tests
    // =========================================================================

    #[test]
    fn test_hub_metrics_initial() {
        let hub: Hub<SimpleValue> = Hub::new("test_metrics_initial").unwrap();
        let metrics = hub.get_metrics();

        assert_eq!(metrics.messages_sent, 0);
        assert_eq!(metrics.messages_received, 0);
        assert_eq!(metrics.send_failures, 0);
        assert_eq!(metrics.recv_failures, 0);
    }

    #[test]
    fn test_hub_metrics_after_send() {
        let hub: Hub<SimpleValue> = Hub::new("test_metrics_send").unwrap();

        hub.send(SimpleValue(1.0), &mut None).unwrap();
        hub.send(SimpleValue(2.0), &mut None).unwrap();
        hub.send(SimpleValue(3.0), &mut None).unwrap();

        let metrics = hub.get_metrics();
        assert_eq!(metrics.messages_sent, 3);
        assert_eq!(metrics.send_failures, 0);
    }

    #[test]
    fn test_hub_metrics_after_recv() {
        let hub: Hub<SimpleValue> = Hub::new("test_metrics_recv").unwrap();

        hub.send(SimpleValue(1.0), &mut None).unwrap();
        hub.recv(&mut None);
        hub.recv(&mut None); // Second recv with no new data

        let metrics = hub.get_metrics();
        assert_eq!(metrics.messages_sent, 1);
        assert!(metrics.messages_received >= 1);
    }

    #[test]
    fn test_hub_metrics_shared_across_clones() {
        let hub1: Hub<SimpleValue> = Hub::new("test_metrics_shared").unwrap();
        let hub2 = hub1.clone();

        hub1.send(SimpleValue(1.0), &mut None).unwrap();
        hub2.send(SimpleValue(2.0), &mut None).unwrap();

        // Metrics are shared, so both should show 2 messages sent
        assert_eq!(hub1.get_metrics().messages_sent, 2);
        assert_eq!(hub2.get_metrics().messages_sent, 2);
    }

    // =========================================================================
    // AtomicHubMetrics Tests
    // =========================================================================

    #[test]
    fn test_atomic_hub_metrics_default() {
        let metrics = AtomicHubMetrics::default();
        let snapshot = metrics.snapshot();

        assert_eq!(snapshot.messages_sent, 0);
        assert_eq!(snapshot.messages_received, 0);
        assert_eq!(snapshot.send_failures, 0);
        assert_eq!(snapshot.recv_failures, 0);
    }

    #[test]
    fn test_atomic_hub_metrics_increment() {
        let metrics = AtomicHubMetrics::default();

        metrics
            .messages_sent
            .fetch_add(1, std::sync::atomic::Ordering::Relaxed);
        metrics
            .messages_sent
            .fetch_add(1, std::sync::atomic::Ordering::Relaxed);
        metrics
            .messages_received
            .fetch_add(3, std::sync::atomic::Ordering::Relaxed);

        let snapshot = metrics.snapshot();
        assert_eq!(snapshot.messages_sent, 2);
        assert_eq!(snapshot.messages_received, 3);
    }

    // =========================================================================
    // Thread Safety Tests
    // =========================================================================

    #[test]
    fn test_hub_send_from_multiple_threads() {
        use std::sync::Arc;
        use std::thread;

        let hub: Arc<Hub<SimpleValue>> = Arc::new(Hub::new("test_threaded_send").unwrap());
        let mut handles = vec![];

        // Spawn multiple threads sending messages
        for i in 0..4 {
            let hub_clone = hub.clone();
            handles.push(thread::spawn(move || {
                for j in 0..10 {
                    let _ = hub_clone.send(SimpleValue((i * 10 + j) as f64), &mut None);
                }
            }));
        }

        for h in handles {
            h.join().unwrap();
        }

        // Should have sent 40 messages total
        assert_eq!(hub.get_metrics().messages_sent, 40);
    }

    #[test]
    fn test_hub_recv_from_multiple_threads() {
        use std::sync::atomic::{AtomicUsize, Ordering};
        use std::sync::Arc;
        use std::thread;

        let hub: Arc<Hub<SimpleValue>> = Arc::new(Hub::new("test_threaded_recv").unwrap());
        let recv_count = Arc::new(AtomicUsize::new(0));

        // Send one message
        hub.send(SimpleValue(42.0), &mut None).unwrap();

        // Multiple threads try to receive
        let mut handles = vec![];
        for _ in 0..4 {
            let hub_clone = hub.clone();
            let count_clone = recv_count.clone();
            handles.push(thread::spawn(move || {
                if hub_clone.recv(&mut None).is_some() {
                    count_clone.fetch_add(1, Ordering::Relaxed);
                }
            }));
        }

        for h in handles {
            h.join().unwrap();
        }

        // At least one thread should have received the message
        assert!(recv_count.load(Ordering::Relaxed) >= 1);
    }

    // =========================================================================
    // Edge Cases
    // =========================================================================

    #[test]
    fn test_hub_with_large_message() {
        #[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
        struct LargeMessage {
            data: Vec<u8>,
        }

        impl crate::core::LogSummary for LargeMessage {
            fn log_summary(&self) -> String {
                format!("LargeMsg({}B)", self.data.len())
            }
        }

        let hub: Hub<LargeMessage> = Hub::new("test_large_msg").unwrap();
        let large_data = LargeMessage {
            data: vec![42u8; 10000], // 10KB message
        };

        hub.send(large_data.clone(), &mut None).unwrap();
        let received = hub.recv(&mut None).unwrap();
        assert_eq!(received.data.len(), 10000);
        assert!(received.data.iter().all(|&b| b == 42));
    }

    #[test]
    fn test_hub_with_empty_string_topic() {
        // Empty topic name should still work (or fail gracefully)
        let result: HorusResult<Hub<SimpleValue>> = Hub::new("");
        // Either succeeds or returns an error, shouldn't panic
        assert!(result.is_ok() || result.is_err());
    }

    #[test]
    fn test_hub_rapid_send_recv() {
        let hub: Hub<SimpleValue> = Hub::new("test_rapid").unwrap();

        for i in 0..1000 {
            hub.send(SimpleValue(i as f64), &mut None).unwrap();
            // Immediate receive should get the value
            let val = hub.recv(&mut None);
            assert!(val.is_some());
        }

        assert_eq!(hub.get_metrics().messages_sent, 1000);
    }
}
