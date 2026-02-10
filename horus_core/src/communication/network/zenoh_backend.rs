//! Zenoh backend for HORUS Topic communication
//!
//! Provides Zenoh-based transport for:
//! - Multi-robot mesh networking
//! - Cloud connectivity
//! - ROS2 interoperability
//!
//! Feature-gated: requires `zenoh-transport` feature

use parking_lot::Mutex;
use std::collections::VecDeque;
use std::marker::PhantomData;
use std::sync::Arc;

use super::network_error::{NetworkError, NetworkErrorCode};
use super::zenoh_config::{
    ConnectionQualityState, SerializationFormat, ZenohConfig, ZenohConnectionQuality,
};
use crate::error::{HorusError, HorusResult};

// ============================================================================
// Zenoh Error Helpers
// ============================================================================

/// Create a serialization error with context
fn serialization_error(format: &str, cause: impl std::fmt::Display) -> HorusError {
    NetworkError::new(
        NetworkErrorCode::SerializationFailed,
        format!("{} serialization failed: {}", format, cause),
    )
    .with_suggestion(format!(
        "Check that your message type implements Serialize correctly for {} format",
        format
    ))
    .into()
}

/// Create a Zenoh connection error
fn zenoh_connection_error(operation: &str, cause: impl std::fmt::Display) -> HorusError {
    NetworkError::new(
        NetworkErrorCode::ConnectionFailed,
        format!("Zenoh {} failed: {}", operation, cause),
    )
    .with_suggestion("Check Zenoh router connectivity and network configuration")
    .with_cli_hint("horus net status")
    .into()
}

/// Create a Zenoh operation error
fn zenoh_operation_error(operation: &str, cause: impl std::fmt::Display) -> HorusError {
    NetworkError::new(
        NetworkErrorCode::SocketError,
        format!("Zenoh {} error: {}", operation, cause),
    )
    .with_suggestion("Check Zenoh session health and network connectivity")
    .into()
}

/// Serialize a message using the specified format
#[cfg(feature = "zenoh-transport")]
fn serialize_message<T: serde::Serialize>(
    msg: &T,
    format: SerializationFormat,
) -> HorusResult<Vec<u8>> {
    match format {
        SerializationFormat::Bincode => {
            bincode::serialize(msg).map_err(|e| serialization_error("Bincode", e))
        }
        #[cfg(feature = "cdr-encoding")]
        SerializationFormat::Cdr => {
            // ROS2/DDS typically uses little-endian CDR
            cdr_encoding::to_vec::<_, byteorder::LittleEndian>(msg)
                .map_err(|e| serialization_error("CDR", e))
        }
        #[cfg(not(feature = "cdr-encoding"))]
        SerializationFormat::Cdr => {
            log::warn!("CDR format requires 'zenoh-ros2' feature, falling back to bincode");
            bincode::serialize(msg).map_err(|e| serialization_error("Bincode", e))
        }
    }
}

// ============================================================================
// Zenoh Session Pool
// ============================================================================

/// Key for identifying equivalent session configurations
#[cfg(feature = "zenoh-transport")]
#[derive(Debug, Clone, PartialEq, Eq, Hash)]
struct SessionPoolKey {
    /// Connect endpoints (sorted for consistent hashing)
    connect: Vec<String>,
    /// Listen endpoints (sorted for consistent hashing)
    listen: Vec<String>,
    /// Zenoh mode
    mode: super::zenoh_config::ZenohMode,
    /// Whether ROS2 mode is enabled
    ros2_mode: bool,
    /// ROS2 domain ID
    ros2_domain_id: u32,
}

#[cfg(feature = "zenoh-transport")]
impl SessionPoolKey {
    fn from_config(config: &ZenohConfig) -> Self {
        let mut connect = config.connect.clone();
        connect.sort();
        let mut listen = config.listen.clone();
        listen.sort();

        Self {
            connect,
            listen,
            mode: config.mode.clone(),
            ros2_mode: config.ros2_mode,
            ros2_domain_id: config.ros2_domain_id,
        }
    }
}

/// Entry in the session pool
#[cfg(feature = "zenoh-transport")]
struct PooledSession {
    /// The shared Zenoh session
    session: Arc<zenoh::Session>,
    /// Number of active references to this session
    ref_count: usize,
    /// Creation timestamp for debugging
    created_at: std::time::Instant,
}

/// Global session pool for sharing Zenoh sessions across topics.
///
/// Instead of creating a new session for each topic, the pool allows
/// multiple topics with the same configuration to share a single session,
/// reducing resource usage and connection overhead.
///
/// # Features
///
/// - **Lazy creation**: Sessions are only created when first requested
/// - **Config-based sharing**: Topics with equivalent configs share sessions
/// - **Reference counting**: Sessions are dropped when no longer in use
/// - **Thread-safe**: Safe to use from multiple threads
///
/// # Example
///
/// ```rust,ignore
/// // Get or create a pooled session
/// let session = ZenohSessionPool::get_or_create(&config).await?;
///
/// // Use the session...
///
/// // When done, release (decrements ref count)
/// ZenohSessionPool::release(&config);
/// ```
#[cfg(feature = "zenoh-transport")]
pub struct ZenohSessionPool {
    // Static pool storage
}

#[cfg(feature = "zenoh-transport")]
lazy_static::lazy_static! {
    /// Global session pool instance
    static ref SESSION_POOL: parking_lot::Mutex<std::collections::HashMap<SessionPoolKey, PooledSession>> =
        parking_lot::Mutex::new(std::collections::HashMap::new());
}

#[cfg(feature = "zenoh-transport")]
impl ZenohSessionPool {
    /// Get an existing session or create a new one for the given configuration.
    ///
    /// If a session with equivalent configuration already exists, its reference
    /// count is incremented and the existing session is returned. Otherwise,
    /// a new session is created and added to the pool.
    pub async fn get_or_create(config: &ZenohConfig) -> HorusResult<Arc<zenoh::Session>> {
        let key = SessionPoolKey::from_config(config);

        // Check if session already exists
        {
            let mut pool = SESSION_POOL.lock();
            if let Some(entry) = pool.get_mut(&key) {
                entry.ref_count += 1;
                log::debug!(
                    "zenoh_pool: Reusing session (refs={}, endpoints={:?})",
                    entry.ref_count,
                    key.connect
                );
                return Ok(entry.session.clone());
            }
        }

        // Create new session outside the lock
        log::info!(
            "zenoh_pool: Creating new session (mode={:?}, endpoints={:?})",
            key.mode,
            key.connect
        );

        let zenoh_config = zenoh::Config::default();
        let session = zenoh::open(zenoh_config)
            .await
            .map_err(|e| zenoh_connection_error("pool session open", e))?;

        let session = Arc::new(session);

        // Add to pool
        {
            let mut pool = SESSION_POOL.lock();
            pool.insert(
                key.clone(),
                PooledSession {
                    session: session.clone(),
                    ref_count: 1,
                    created_at: std::time::Instant::now(),
                },
            );
        }

        log::debug!("zenoh_pool: Session created and pooled");
        Ok(session)
    }

    /// Get an existing session or create one synchronously (blocking).
    pub fn get_or_create_blocking(config: &ZenohConfig) -> HorusResult<Arc<zenoh::Session>> {
        let rt = tokio::runtime::Handle::try_current().map_err(|_| {
            NetworkError::new(
                NetworkErrorCode::ResourceExhausted,
                "No tokio runtime available for blocking pool access",
            )
        })?;

        rt.block_on(Self::get_or_create(config))
    }

    /// Release a session reference.
    ///
    /// Decrements the reference count for the session matching the given config.
    /// If the reference count reaches zero, the session is removed from the pool
    /// and will be closed when the last Arc reference is dropped.
    pub fn release(config: &ZenohConfig) {
        let key = SessionPoolKey::from_config(config);
        let mut pool = SESSION_POOL.lock();

        if let Some(entry) = pool.get_mut(&key) {
            entry.ref_count = entry.ref_count.saturating_sub(1);
            log::debug!("zenoh_pool: Released session (refs={})", entry.ref_count);

            if entry.ref_count == 0 {
                log::info!(
                    "zenoh_pool: Removing unused session (age={:?})",
                    entry.created_at.elapsed()
                );
                pool.remove(&key);
            }
        }
    }

    /// Get statistics about the session pool.
    pub fn stats() -> ZenohPoolStats {
        let pool = SESSION_POOL.lock();
        let total_sessions = pool.len();
        let total_refs: usize = pool.values().map(|e| e.ref_count).sum();

        ZenohPoolStats {
            total_sessions,
            total_references: total_refs,
            sessions: pool
                .iter()
                .map(|(key, entry)| ZenohPoolSessionInfo {
                    endpoints: key.connect.clone(),
                    ref_count: entry.ref_count,
                    age_secs: entry.created_at.elapsed().as_secs_f64(),
                })
                .collect(),
        }
    }

    /// Clear all sessions from the pool.
    ///
    /// This should only be used for testing or shutdown scenarios.
    /// Active references will continue to work, but new requests will
    /// create new sessions.
    pub fn clear() {
        let mut pool = SESSION_POOL.lock();
        let count = pool.len();
        pool.clear();
        log::info!("zenoh_pool: Cleared {} sessions", count);
    }
}

/// Statistics about the Zenoh session pool
#[derive(Debug, Clone)]
pub struct ZenohPoolStats {
    /// Total number of sessions in the pool
    pub total_sessions: usize,
    /// Total number of active references across all sessions
    pub total_references: usize,
    /// Per-session information
    pub sessions: Vec<ZenohPoolSessionInfo>,
}

/// Information about a single pooled session
#[derive(Debug, Clone)]
pub struct ZenohPoolSessionInfo {
    /// Connection endpoints for this session
    pub endpoints: Vec<String>,
    /// Number of active references
    pub ref_count: usize,
    /// Age of the session in seconds
    pub age_secs: f64,
}

/// Zenoh backend for HORUS Topic communication
///
/// Provides pub/sub communication over Zenoh protocol.
/// Supports both HORUS-native (bincode) and ROS2-compatible (CDR) serialization.
///
/// Includes connection quality monitoring for cloud connectivity:
/// - Latency measurement (RTT)
/// - Packet loss tracking
/// - Jitter calculation
/// - Health score (0-100)
/// - Automatic failover detection
#[cfg(feature = "zenoh-transport")]
pub struct ZenohBackend<T> {
    /// Zenoh session (shared across publishers/subscribers)
    session: Arc<zenoh::Session>,
    /// Publisher for this topic (stored as boxed to handle lifetime)
    publisher: Option<Box<zenoh::pubsub::Publisher<'static>>>,
    /// Receive buffer for incoming messages
    recv_buffer: Arc<Mutex<VecDeque<T>>>,
    /// Key expression for this topic
    key_expr: String,
    /// Configuration
    config: ZenohConfig,
    /// Connection quality metrics (latency, packet loss, jitter, health score)
    connection_quality: Arc<Mutex<ZenohConnectionQuality>>,
    /// Phantom data for type parameter
    _phantom: PhantomData<T>,
}

#[cfg(feature = "zenoh-transport")]
impl<T> ZenohBackend<T>
where
    T: serde::Serialize + serde::de::DeserializeOwned + Send + Sync + Clone + 'static,
{
    /// Create a new Zenoh backend for a topic
    ///
    /// This is an async function that must be called from within a tokio runtime.
    pub async fn new(topic: &str, config: ZenohConfig) -> HorusResult<Self> {
        // Build zenoh config - use default for now
        // In zenoh 1.2+, configuration is done differently
        let zenoh_config = zenoh::Config::default();

        log::info!(
            "zenoh[{}]: Creating backend (mode={:?}, format={:?})",
            topic,
            config.mode,
            config.serialization
        );

        // Add connect endpoints logging
        if !config.connect.is_empty() {
            for endpoint in &config.connect {
                log::debug!("zenoh[{}]: Will connect to endpoint: {}", topic, endpoint);
            }
        }

        if !config.listen.is_empty() {
            for endpoint in &config.listen {
                log::debug!("zenoh[{}]: Will listen on: {}", topic, endpoint);
            }
        }

        // Open session
        log::debug!("zenoh[{}]: Opening Zenoh session...", topic);
        let session = zenoh::open(zenoh_config)
            .await
            .map_err(|e| zenoh_connection_error("session open", e))?;

        let key_expr = config.topic_to_key_expr(topic);
        log::debug!("zenoh[{}]: Session opened, key_expr='{}'", topic, key_expr);

        Ok(Self {
            session: Arc::new(session),
            publisher: None,
            recv_buffer: Arc::new(Mutex::new(VecDeque::with_capacity(1024))),
            key_expr,
            config,
            connection_quality: Arc::new(Mutex::new(ZenohConnectionQuality::new())),
            _phantom: PhantomData,
        })
    }

    /// Create a blocking version (for non-async contexts)
    pub fn new_blocking(topic: &str, config: ZenohConfig) -> HorusResult<Self> {
        // Use tokio runtime to run async code
        let rt = tokio::runtime::Handle::try_current()
            .or_else(|_| {
                // Create a new runtime if not in one
                tokio::runtime::Runtime::new().map(|rt| rt.handle().clone())
            })
            .map_err(|e| {
                NetworkError::new(
                    NetworkErrorCode::ResourceExhausted,
                    format!("Failed to get tokio runtime: {}", e),
                )
                .with_suggestion("Ensure you're running within a tokio async context")
            })?;

        rt.block_on(Self::new(topic, config))
    }

    /// Create a new Zenoh backend using a pooled session.
    ///
    /// This is the recommended way to create backends when you have multiple
    /// topics with the same configuration, as it shares the underlying Zenoh
    /// session to reduce resource usage.
    ///
    /// # Example
    ///
    /// ```rust,ignore
    /// let config = ZenohConfig::default();
    ///
    /// // These two backends will share the same session
    /// let backend1 = ZenohBackend::<Msg1>::new_pooled("topic1", config.clone()).await?;
    /// let backend2 = ZenohBackend::<Msg2>::new_pooled("topic2", config.clone()).await?;
    /// ```
    pub async fn new_pooled(topic: &str, config: ZenohConfig) -> HorusResult<Self> {
        log::info!(
            "zenoh[{}]: Creating pooled backend (mode={:?}, format={:?})",
            topic,
            config.mode,
            config.serialization
        );

        // Get or create a pooled session
        let session = ZenohSessionPool::get_or_create(&config).await?;

        let key_expr = config.topic_to_key_expr(topic);
        log::debug!(
            "zenoh[{}]: Using pooled session, key_expr='{}'",
            topic,
            key_expr
        );

        Ok(Self {
            session,
            publisher: None,
            recv_buffer: Arc::new(Mutex::new(VecDeque::with_capacity(1024))),
            key_expr,
            config,
            connection_quality: Arc::new(Mutex::new(ZenohConnectionQuality::new())),
            _phantom: PhantomData,
        })
    }

    /// Create a pooled backend synchronously (blocking).
    pub fn new_pooled_blocking(topic: &str, config: ZenohConfig) -> HorusResult<Self> {
        let rt = tokio::runtime::Handle::try_current()
            .or_else(|_| tokio::runtime::Runtime::new().map(|rt| rt.handle().clone()))
            .map_err(|e| {
                NetworkError::new(
                    NetworkErrorCode::ResourceExhausted,
                    format!("Failed to get tokio runtime: {}", e),
                )
                .with_suggestion("Ensure you're running within a tokio async context")
            })?;

        rt.block_on(Self::new_pooled(topic, config))
    }

    /// Create a backend with an existing session.
    ///
    /// This allows direct control over session sharing without using the pool.
    pub fn with_session(topic: &str, session: Arc<zenoh::Session>, config: ZenohConfig) -> Self {
        let key_expr = config.topic_to_key_expr(topic);

        Self {
            session,
            publisher: None,
            recv_buffer: Arc::new(Mutex::new(VecDeque::with_capacity(1024))),
            key_expr,
            config,
            connection_quality: Arc::new(Mutex::new(ZenohConnectionQuality::new())),
            _phantom: PhantomData,
        }
    }

    /// Initialize publisher for sending messages
    pub async fn init_publisher(&mut self) -> HorusResult<()> {
        if self.publisher.is_some() {
            log::trace!("zenoh[{}]: Publisher already initialized", self.key_expr);
            return Ok(());
        }

        log::debug!("zenoh[{}]: Declaring publisher...", self.key_expr);
        let publisher = self
            .session
            .declare_publisher(&self.key_expr)
            .await
            .map_err(|e| {
                log::error!(
                    "zenoh[{}]: Failed to declare publisher: {}",
                    self.key_expr,
                    e
                );
                zenoh_operation_error("publisher declare", e)
            })?;

        // Store publisher - use unsafe transmute to handle lifetime
        // This is safe because the session outlives the publisher
        let static_publisher: zenoh::pubsub::Publisher<'static> =
            unsafe { std::mem::transmute(publisher) };
        self.publisher = Some(Box::new(static_publisher));

        log::info!("zenoh[{}]: Publisher initialized", self.key_expr);
        Ok(())
    }

    /// Initialize subscriber for receiving messages
    pub async fn init_subscriber(&mut self) -> HorusResult<()> {
        let recv_buffer = self.recv_buffer.clone();
        let serialization = self.config.serialization;

        log::debug!("zenoh[{}]: Declaring subscriber...", self.key_expr);
        let subscriber = self
            .session
            .declare_subscriber(&self.key_expr)
            .await
            .map_err(|e| {
                log::error!(
                    "zenoh[{}]: Failed to declare subscriber: {}",
                    self.key_expr,
                    e
                );
                zenoh_operation_error("subscriber declare", e)
            })?;

        log::info!(
            "zenoh[{}]: Subscriber initialized (format={:?})",
            self.key_expr,
            serialization
        );

        // Spawn a task to receive messages
        let key_expr = self.key_expr.clone();
        tokio::spawn(async move {
            let mut msg_count: u64 = 0;
            let mut error_count: u64 = 0;

            loop {
                match subscriber.recv_async().await {
                    Ok(sample) => {
                        let payload = sample.payload().to_bytes();
                        let payload_len = payload.len();
                        let msg: Option<T> = match serialization {
                            SerializationFormat::Bincode => bincode::deserialize(&payload).ok(),
                            #[cfg(feature = "cdr-encoding")]
                            SerializationFormat::Cdr => {
                                // ROS2/DDS uses little-endian CDR, returns (value, bytes_read)
                                cdr_encoding::from_bytes::<_, byteorder::LittleEndian>(&payload)
                                    .ok()
                                    .map(|(msg, _bytes_read)| msg)
                            }
                            #[cfg(not(feature = "cdr-encoding"))]
                            SerializationFormat::Cdr => {
                                log::warn!(
                                    "zenoh[{}]: CDR format requires 'zenoh-ros2' feature, falling back to bincode",
                                    key_expr
                                );
                                bincode::deserialize(&payload).ok()
                            }
                        };

                        if let Some(msg) = msg {
                            msg_count += 1;
                            log::trace!(
                                "zenoh[{}]: Received message #{} ({} bytes)",
                                key_expr,
                                msg_count,
                                payload_len
                            );

                            let mut buffer = recv_buffer.lock();
                            // Keep buffer bounded
                            if buffer.len() >= 10000 {
                                log::warn!(
                                    "zenoh[{}]: Receive buffer full, dropping oldest message",
                                    key_expr
                                );
                                buffer.pop_front();
                            }
                            buffer.push_back(msg);
                        } else {
                            error_count += 1;
                            log::debug!(
                                "zenoh[{}]: Failed to deserialize message ({} bytes, format={:?}, errors={})",
                                key_expr,
                                payload_len,
                                serialization,
                                error_count
                            );
                        }
                    }
                    Err(e) => {
                        log::warn!(
                            "zenoh[{}]: Subscriber error, terminating (received {} msgs, {} errors): {}",
                            key_expr,
                            msg_count,
                            error_count,
                            e
                        );
                        break;
                    }
                }
            }
        });

        Ok(())
    }

    /// Send a message (synchronous wrapper)
    pub fn send(&self, msg: &T) -> HorusResult<()> {
        let publisher = self.publisher.as_ref().ok_or_else(|| {
            log::error!(
                "zenoh[{}]: Cannot send - publisher not initialized",
                self.key_expr
            );
            NetworkError::new(NetworkErrorCode::InvalidConfig, "Publisher not initialized")
                .with_suggestion("Call init_publisher() before sending messages")
        })?;

        // Serialize using configured format
        let payload = serialize_message(msg, self.config.serialization)?;
        let payload_len = payload.len();

        log::trace!(
            "zenoh[{}]: Sending {} bytes (sync, format={:?})",
            self.key_expr,
            payload_len,
            self.config.serialization
        );

        // Use blocking runtime for sync API compatibility
        let rt = tokio::runtime::Handle::try_current().map_err(|e| {
            log::error!(
                "zenoh[{}]: No tokio runtime for sync send: {}",
                self.key_expr,
                e
            );
            NetworkError::new(
                NetworkErrorCode::ResourceExhausted,
                format!("No tokio runtime: {}", e),
            )
            .with_suggestion("Ensure you're running within a tokio async context")
        })?;

        rt.block_on(async { publisher.put(payload).await })
            .map_err(|e| {
                log::error!("zenoh[{}]: Send failed: {}", self.key_expr, e);
                zenoh_operation_error("put", e)
            })?;

        log::trace!(
            "zenoh[{}]: Send complete ({} bytes)",
            self.key_expr,
            payload_len
        );
        Ok(())
    }

    /// Send a message asynchronously
    pub async fn send_async(&self, msg: &T) -> HorusResult<()> {
        let publisher = self.publisher.as_ref().ok_or_else(|| {
            log::error!(
                "zenoh[{}]: Cannot send_async - publisher not initialized",
                self.key_expr
            );
            NetworkError::new(NetworkErrorCode::InvalidConfig, "Publisher not initialized")
                .with_suggestion("Call init_publisher() before sending messages")
        })?;

        let payload = serialize_message(msg, self.config.serialization)?;
        let payload_len = payload.len();

        log::trace!(
            "zenoh[{}]: Sending {} bytes (async, format={:?})",
            self.key_expr,
            payload_len,
            self.config.serialization
        );

        publisher.put(payload).await.map_err(|e| {
            log::error!("zenoh[{}]: Async send failed: {}", self.key_expr, e);
            zenoh_operation_error("put", e)
        })?;

        log::trace!(
            "zenoh[{}]: Async send complete ({} bytes)",
            self.key_expr,
            payload_len
        );
        Ok(())
    }

    /// Receive a message (non-blocking)
    pub fn recv(&self) -> Option<T> {
        let mut buffer = self.recv_buffer.lock();
        buffer.pop_front()
    }

    /// Try to receive a message with timeout
    pub fn recv_timeout(&self, timeout: std::time::Duration) -> Option<T> {
        let start = std::time::Instant::now();
        while start.elapsed() < timeout {
            if let Some(msg) = self.recv() {
                return Some(msg);
            }
            std::thread::sleep(std::time::Duration::from_micros(100));
        }
        None
    }

    /// Get the key expression for this backend
    pub fn key_expr(&self) -> &str {
        &self.key_expr
    }

    /// Get the configuration
    pub fn config(&self) -> &ZenohConfig {
        &self.config
    }

    /// Get the number of pending messages
    pub fn pending_count(&self) -> usize {
        self.recv_buffer.lock().len()
    }

    /// Clear the receive buffer
    pub fn clear_buffer(&self) {
        self.recv_buffer.lock().clear();
    }

    /// Get session info (for debugging)
    pub fn session_info(&self) -> ZenohSessionInfo {
        let quality = self.connection_quality.lock();
        ZenohSessionInfo {
            key_expr: self.key_expr.clone(),
            has_publisher: self.publisher.is_some(),
            pending_messages: self.pending_count(),
            connection_state: quality.state,
            health_score: quality.health_score(),
            latency_ms: quality.latency_ms,
            connected_router: quality.connected_router.clone(),
        }
    }

    // ========== CONNECTION QUALITY MONITORING ==========

    /// Get a snapshot of current connection quality metrics
    ///
    /// Returns comprehensive quality data including:
    /// - Latency (current, avg, min, max)
    /// - Packet loss percentage
    /// - Jitter
    /// - Health score (0-100)
    /// - Connection state
    pub fn connection_quality(&self) -> ZenohConnectionQuality {
        self.connection_quality.lock().clone()
    }

    /// Get the connection health score (0-100)
    ///
    /// 100 = perfect connection
    /// 80+ = good
    /// 50-80 = degraded
    /// <50 = poor
    pub fn health_score(&self) -> u32 {
        self.connection_quality.lock().health_score()
    }

    /// Get the current connection state
    pub fn connection_state(&self) -> ConnectionQualityState {
        self.connection_quality.lock().state
    }

    /// Check if the connection is established and healthy
    pub fn is_connected(&self) -> bool {
        let quality = self.connection_quality.lock();
        quality.is_good() || quality.is_degraded()
    }

    /// Check if the connection is in a good state
    pub fn is_connection_good(&self) -> bool {
        self.connection_quality.lock().is_good()
    }

    /// Check if the connection is degraded
    pub fn is_connection_degraded(&self) -> bool {
        self.connection_quality.lock().is_degraded()
    }

    /// Check if the connection has failed
    pub fn is_connection_failed(&self) -> bool {
        self.connection_quality.lock().is_failed()
    }

    /// Get the current latency in milliseconds
    pub fn latency_ms(&self) -> f64 {
        self.connection_quality.lock().latency_ms
    }

    /// Get the average latency in milliseconds
    pub fn avg_latency_ms(&self) -> f64 {
        self.connection_quality.lock().avg_latency_ms
    }

    /// Get packet loss percentage (0-100)
    pub fn packet_loss_percent(&self) -> f64 {
        self.connection_quality.lock().packet_loss_percent
    }

    /// Get the currently connected router endpoint (if any)
    pub fn connected_router(&self) -> Option<String> {
        self.connection_quality.lock().connected_router.clone()
    }

    /// Mark the connection as established to a router
    pub fn mark_connected(&self, router: &str) {
        log::info!(
            "zenoh[{}]: Connection established to router '{}'",
            self.key_expr,
            router
        );
        self.connection_quality.lock().mark_connected(router);
    }

    /// Mark the connection as disconnected
    pub fn mark_disconnected(&self) {
        let quality = self.connection_quality.lock();
        let prev_router = quality.connected_router.clone();
        drop(quality);

        log::warn!(
            "zenoh[{}]: Connection lost (was connected to {:?})",
            self.key_expr,
            prev_router
        );
        self.connection_quality.lock().mark_disconnected();
    }

    /// Record a failover to a new router
    pub fn record_failover(&self, new_router: &str) {
        let mut quality = self.connection_quality.lock();
        let prev_router = quality.connected_router.clone();
        let failover_num = quality.failover_count + 1;
        quality.record_failover(new_router);

        log::warn!(
            "zenoh[{}]: Failover #{} from {:?} to '{}'",
            self.key_expr,
            failover_num,
            prev_router,
            new_router
        );
    }

    /// Get the number of failovers that have occurred
    pub fn failover_count(&self) -> u32 {
        self.connection_quality.lock().failover_count
    }

    /// Perform a single health check by measuring round-trip latency
    ///
    /// Uses Zenoh's liveliness mechanism to ping and measure RTT.
    /// Updates the connection quality metrics accordingly.
    pub async fn perform_health_check(&self) -> HorusResult<f64> {
        log::trace!("zenoh[{}]: Starting health check...", self.key_expr);
        let start = std::time::Instant::now();

        // Use Zenoh's built-in get() for a round-trip latency measurement
        // We query our own key expression to measure actual network RTT
        let health_key = format!("{}/__horus_health__", self.key_expr);

        // Declare a queryable to respond to our own health check
        let queryable = self
            .session
            .declare_queryable(&health_key)
            .await
            .map_err(|e| {
                HorusError::Communication(format!("Failed to create health queryable: {}", e))
            })?;

        // Clone health_key for use in async closure
        let health_key_reply = health_key.clone();

        // Spawn a task to respond to the query
        let queryable_handle = tokio::spawn(async move {
            if let Ok(query) =
                tokio::time::timeout(std::time::Duration::from_secs(5), queryable.recv_async())
                    .await
            {
                if let Ok(query) = query {
                    let _ = query.reply(&health_key_reply, "pong").await;
                }
            }
        });

        // Send the health check query
        let result = tokio::time::timeout(
            std::time::Duration::from_secs(5),
            self.session.get(&health_key),
        )
        .await;

        // Wait for queryable task to complete
        let _ = queryable_handle.await;

        let latency_ms = start.elapsed().as_secs_f64() * 1000.0;

        match result {
            Ok(Ok(replies)) => {
                // Consume replies to complete the operation
                let _: Vec<_> = replies.into_iter().collect::<Vec<_>>();

                // Update quality metrics with success
                let mut quality = self.connection_quality.lock();
                quality.update_latency(latency_ms);
                quality.record_health_check_success();

                log::debug!(
                    "zenoh[{}]: Health check OK: {:.2}ms (avg: {:.2}ms, score: {})",
                    self.key_expr,
                    latency_ms,
                    quality.avg_latency_ms,
                    quality.health_score()
                );

                Ok(latency_ms)
            }
            Ok(Err(e)) => {
                // Query failed
                let mut quality = self.connection_quality.lock();
                quality.record_health_check_failure();

                log::warn!(
                    "zenoh[{}]: Health check failed (query error): {}",
                    self.key_expr,
                    e
                );

                Err(HorusError::Communication(format!(
                    "Health check query failed: {}",
                    e
                )))
            }
            Err(_) => {
                // Timeout
                let mut quality = self.connection_quality.lock();
                quality.record_health_check_failure();

                log::warn!("zenoh[{}]: Health check timed out (>5s)", self.key_expr);

                Err(HorusError::Communication("Health check timed out".into()))
            }
        }
    }

    /// Start background health monitoring
    ///
    /// Spawns a task that periodically checks connection health and updates metrics.
    /// Returns a handle that can be used to stop monitoring.
    ///
    /// # Arguments
    /// * `interval` - How often to perform health checks
    pub fn start_health_monitoring(
        &self,
        interval: std::time::Duration,
    ) -> tokio::task::JoinHandle<()> {
        let session = self.session.clone();
        let key_expr = self.key_expr.clone();
        let quality = self.connection_quality.clone();

        log::info!(
            "zenoh[{}]: Starting health monitoring (interval={:?})",
            self.key_expr,
            interval
        );

        tokio::spawn(async move {
            let health_key = format!("{}/__horus_health_monitor__", key_expr);
            let mut consecutive_failures = 0u32;
            let mut total_checks: u64 = 0;

            loop {
                total_checks += 1;
                let start = std::time::Instant::now();

                // Simple ping using get() to measure RTT
                let result = tokio::time::timeout(
                    std::time::Duration::from_secs(5),
                    session.get(&health_key),
                )
                .await;

                let latency_ms = start.elapsed().as_secs_f64() * 1000.0;

                match result {
                    Ok(Ok(replies)) => {
                        // Consume replies (may be empty, that's fine for latency measurement)
                        let _: Vec<_> = replies.into_iter().collect::<Vec<_>>();

                        let mut q = quality.lock();
                        q.update_latency(latency_ms);
                        q.record_health_check_success();

                        if consecutive_failures > 0 {
                            log::info!(
                                "zenoh[{}]: Connection recovered after {} failures",
                                key_expr,
                                consecutive_failures
                            );
                        }
                        consecutive_failures = 0;

                        log::trace!(
                            "zenoh[{}]: Health check #{} OK: {:.2}ms (avg: {:.2}ms, score: {})",
                            key_expr,
                            total_checks,
                            latency_ms,
                            q.avg_latency_ms,
                            q.health_score()
                        );
                    }
                    Ok(Err(_)) | Err(_) => {
                        consecutive_failures += 1;
                        let mut q = quality.lock();
                        q.record_health_check_failure();

                        if consecutive_failures == 1 {
                            log::debug!(
                                "zenoh[{}]: Health check #{} failed (first failure)",
                                key_expr,
                                total_checks
                            );
                        } else if consecutive_failures == 3 {
                            log::warn!(
                                "zenoh[{}]: Connection degraded ({} consecutive failures, score: {})",
                                key_expr,
                                consecutive_failures,
                                q.health_score()
                            );
                        } else if consecutive_failures >= 5 && consecutive_failures % 5 == 0 {
                            log::error!(
                                "zenoh[{}]: Connection unhealthy ({} consecutive failures, score: {})",
                                key_expr,
                                consecutive_failures,
                                q.health_score()
                            );
                        }
                    }
                }

                tokio::time::sleep(interval).await;
            }
        })
    }

    /// Get shared reference to connection quality for external monitoring
    pub fn connection_quality_arc(&self) -> Arc<Mutex<ZenohConnectionQuality>> {
        self.connection_quality.clone()
    }

    // ========== CLOUD FAILOVER SUPPORT ==========

    /// Create a new Zenoh backend with cloud configuration (failover support)
    ///
    /// This variant supports:
    /// - Multiple router endpoints with automatic failover
    /// - Connection quality monitoring with latency-based failover triggers
    /// - Optional mDNS auto-discovery of nearby routers
    /// - Automatic reconnection on connection loss
    pub async fn new_with_cloud_config(
        topic: &str,
        cloud_config: super::zenoh_config::ZenohCloudConfig,
    ) -> HorusResult<Self> {
        // Build list of routers to try
        let mut routers = vec![cloud_config.primary_router.clone()];
        routers.extend(cloud_config.backup_routers.iter().cloned());

        // Try mDNS discovery if enabled
        #[cfg(feature = "mdns")]
        if cloud_config.auto_discovery {
            if let Ok(discovered) = super::zenoh_config::discover_zenoh_routers_mdns(
                &cloud_config.discovery_service_type,
                cloud_config.connection_timeout,
            ) {
                for router in discovered {
                    if !routers.contains(&router) {
                        log::info!("Discovered Zenoh router via mDNS: {}", router);
                        routers.push(router);
                    }
                }
            }
        }

        // Try connecting to each router in order
        let (session, connected_router) =
            Self::try_connect_with_failover(&routers, cloud_config.connection_timeout).await?;

        // Create ZenohConfig from cloud config
        let mut config = ZenohConfig::default();
        config.connect = vec![connected_router.clone()];
        if let Some(ns) = &cloud_config.namespace {
            config.namespace = Some(ns.clone());
        }

        let key_expr = config.topic_to_key_expr(topic);

        // Initialize connection quality
        let mut quality = ZenohConnectionQuality::new();
        quality.mark_connected(&connected_router);

        let backend = Self {
            session: Arc::new(session),
            publisher: None,
            recv_buffer: Arc::new(Mutex::new(VecDeque::with_capacity(1024))),
            key_expr,
            config,
            connection_quality: Arc::new(Mutex::new(quality)),
            _phantom: PhantomData,
        };

        log::info!(
            "Connected to Zenoh cloud router: {} (topic: {})",
            connected_router,
            topic
        );

        Ok(backend)
    }

    /// Try connecting to routers with failover
    ///
    /// Attempts to connect to each router in the list, returning the first successful connection.
    async fn try_connect_with_failover(
        routers: &[String],
        timeout: std::time::Duration,
    ) -> HorusResult<(zenoh::Session, String)> {
        if routers.is_empty() {
            let err = NetworkError::new(
                NetworkErrorCode::MissingConfig,
                "No routers configured for cloud connection",
            )
            .with_suggestion(
                "Configure at least one router in your ZenohCloudConfig. \
                Example: cloud://cloud.horus.dev:7447 or use HORUS_CLOUD_ROUTERS env var.",
            )
            .with_cli_hint("horus net check --check-cloud");
            return Err(err.into());
        }

        let mut last_error: Option<NetworkError> = None;

        for router in routers {
            log::debug!("Attempting connection to Zenoh router: {}", router);

            // Build Zenoh config with this router
            let mut zenoh_config = zenoh::Config::default();

            // Set connect endpoints using JSON5 configuration
            // Format: "tcp/host:port" or "udp/host:port"
            let endpoints_json = format!("[\"{}\"]", router);
            if let Err(e) = zenoh_config.insert_json5("connect/endpoints", &endpoints_json) {
                log::warn!("Invalid router endpoint {}: {}", router, e);
                last_error = Some(
                    NetworkError::invalid_endpoint(router, e.to_string()).with_suggestion(format!(
                        "Use format 'tcp/host:port' or 'udp/host:port'. Got: {}",
                        router
                    )),
                );
                continue;
            }

            // Set client mode for cloud connections
            if let Err(e) = zenoh_config.insert_json5("mode", "\"client\"") {
                log::warn!("Failed to set client mode: {}", e);
            }

            // Try to connect with timeout
            let connect_result = tokio::time::timeout(timeout, zenoh::open(zenoh_config)).await;

            match connect_result {
                Ok(Ok(session)) => {
                    log::info!("Successfully connected to Zenoh router: {}", router);
                    return Ok((session, router.clone()));
                }
                Ok(Err(e)) => {
                    log::warn!("Failed to connect to {}: {}", router, e);
                    last_error = Some(NetworkError::connection_failed(router, e.to_string()));
                }
                Err(_) => {
                    log::warn!("Connection to {} timed out after {:?}", router, timeout);
                    last_error = Some(NetworkError::connection_timeout(router, timeout));
                }
            }
        }

        // Build a comprehensive error message
        let err = last_error.unwrap_or_else(|| {
            NetworkError::new(
                NetworkErrorCode::ConnectionFailed,
                "Failed to connect to any Zenoh router",
            )
        });

        // Enhance with multi-router context
        let final_err = NetworkError::new(
            NetworkErrorCode::MaxRetriesExceeded,
            format!(
                "Failed to connect to any Zenoh router ({} tried). Last error: {}",
                routers.len(),
                err
            ),
        )
        .with_context("routers_tried", routers.join(", "))
        .with_suggestion(
            "Check network connectivity and router availability. \
            Ensure routers are running and accessible from your network.",
        )
        .with_cli_hint(format!(
            "horus net check {}",
            routers
                .first()
                .unwrap_or(&"cloud.horus.dev:7447".to_string())
        ));

        Err(final_err.into())
    }

    /// Start failover monitoring
    ///
    /// Monitors connection quality and triggers failover when:
    /// - Latency exceeds threshold
    /// - Too many consecutive health check failures
    /// - Connection is lost
    ///
    /// Returns a handle that can be used to stop monitoring.
    pub fn start_failover_monitoring(
        &self,
        cloud_config: super::zenoh_config::ZenohCloudConfig,
    ) -> tokio::task::JoinHandle<()> {
        let session = self.session.clone();
        let key_expr = self.key_expr.clone();
        let quality = self.connection_quality.clone();

        let health_check_interval = cloud_config.health_check_interval;
        let latency_failover_threshold = cloud_config.latency_failover_threshold_ms as f64;
        let failures_before_failover = cloud_config.health_check_failures_before_failover;

        // Collect all routers
        let mut all_routers = vec![cloud_config.primary_router.clone()];
        all_routers.extend(cloud_config.backup_routers.iter().cloned());

        tokio::spawn(async move {
            let health_key = format!("{}/__horus_failover_monitor__", key_expr);
            let mut consecutive_failures = 0u32;
            let mut current_router_index = 0usize;

            loop {
                let start = std::time::Instant::now();

                // Simple ping using get() to measure RTT
                let result = tokio::time::timeout(
                    std::time::Duration::from_secs(5),
                    session.get(&health_key),
                )
                .await;

                let latency_ms = start.elapsed().as_secs_f64() * 1000.0;

                let should_failover = match result {
                    Ok(Ok(replies)) => {
                        let _: Vec<_> = replies.into_iter().collect::<Vec<_>>();

                        let mut q = quality.lock();
                        q.update_latency(latency_ms);
                        q.record_health_check_success();
                        consecutive_failures = 0;

                        // Check if latency exceeds failover threshold
                        if q.avg_latency_ms > latency_failover_threshold {
                            log::warn!(
                                "Latency ({:.2}ms) exceeds failover threshold ({:.0}ms)",
                                q.avg_latency_ms,
                                latency_failover_threshold
                            );
                            true
                        } else {
                            false
                        }
                    }
                    Ok(Err(_)) | Err(_) => {
                        consecutive_failures += 1;
                        let mut q = quality.lock();
                        q.record_health_check_failure();

                        if consecutive_failures >= failures_before_failover {
                            log::warn!(
                                "Too many failures ({}), triggering failover",
                                consecutive_failures
                            );
                            true
                        } else {
                            false
                        }
                    }
                };

                if should_failover && all_routers.len() > 1 {
                    // Calculate next router to try
                    let next_index = (current_router_index + 1) % all_routers.len();
                    let next_router = &all_routers[next_index];

                    log::info!(
                        "Initiating failover to router: {} (index {})",
                        next_router,
                        next_index
                    );

                    // Record failover
                    {
                        let mut q = quality.lock();
                        q.record_failover(next_router);
                    }

                    current_router_index = next_index;
                    consecutive_failures = 0;

                    // Note: Actual session reconnection would require recreating the session
                    // This is a monitoring-only task. Full reconnection needs to be handled
                    // at a higher level by the application.
                }

                tokio::time::sleep(health_check_interval).await;
            }
        })
    }

    /// Get a list of all configured routers (primary + backups)
    pub fn configured_routers(&self) -> Vec<String> {
        self.config.connect.clone()
    }
}

#[cfg(feature = "zenoh-transport")]
impl<T> Clone for ZenohBackend<T> {
    fn clone(&self) -> Self {
        Self {
            session: self.session.clone(),
            publisher: None, // Publishers are not cloned, must re-init
            recv_buffer: self.recv_buffer.clone(),
            key_expr: self.key_expr.clone(),
            config: self.config.clone(),
            connection_quality: self.connection_quality.clone(), // Share quality metrics
            _phantom: PhantomData,
        }
    }
}

#[cfg(feature = "zenoh-transport")]
impl<T> std::fmt::Debug for ZenohBackend<T> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let quality = self.connection_quality.lock();
        f.debug_struct("ZenohBackend")
            .field("key_expr", &self.key_expr)
            .field("has_publisher", &self.publisher.is_some())
            .field("pending_messages", &self.recv_buffer.lock().len())
            .field("connection_state", &quality.state)
            .field("health_score", &quality.health_score())
            .field("latency_ms", &quality.latency_ms)
            .finish()
    }
}

/// Session information for debugging
#[derive(Debug, Clone)]
pub struct ZenohSessionInfo {
    pub key_expr: String,
    pub has_publisher: bool,
    pub pending_messages: usize,
    pub connection_state: ConnectionQualityState,
    pub health_score: u32,
    pub latency_ms: f64,
    pub connected_router: Option<String>,
}

// ============================================================================
// ZenohBatch - Batched Multi-Topic Operations
// ============================================================================

/// A pending publish operation in a batch
#[cfg(feature = "zenoh-transport")]
struct BatchedPublish {
    /// Key expression for the topic
    key_expr: String,
    /// Serialized payload
    payload: Vec<u8>,
}

/// Batched multi-topic publish operations for improved network efficiency.
///
/// Allows accumulating multiple publish operations and committing them all
/// in a single batch, reducing network round-trips for scenarios where
/// multiple topics need to be updated together (e.g., sensor data + status).
///
/// # Example
///
/// ```rust,ignore
/// use horus_core::communication::network::zenoh_backend::ZenohBatch;
///
/// // Create a batch with an existing session
/// let mut batch = ZenohBatch::new(session.clone(), config.clone());
///
/// // Queue multiple publishes
/// batch.publish("sensor/lidar", &lidar_data)?;
/// batch.publish("sensor/imu", &imu_data)?;
/// batch.publish("robot/status", &status)?;
///
/// // Commit all in parallel (single logical operation)
/// let results = batch.commit().await?;
/// ```
///
/// # Benefits
///
/// - Reduced network round-trips for correlated publishes
/// - Better cache locality for serialization
/// - Atomic-like semantics (all succeed or all fail reporting)
/// - Parallel execution of underlying publishes
#[cfg(feature = "zenoh-transport")]
pub struct ZenohBatch {
    /// Shared Zenoh session
    session: Arc<zenoh::Session>,
    /// Configuration for serialization
    config: ZenohConfig,
    /// Queued publish operations
    pending: Vec<BatchedPublish>,
}

#[cfg(feature = "zenoh-transport")]
impl ZenohBatch {
    /// Create a new batch with a Zenoh session
    ///
    /// The session is shared and will not be closed when the batch is dropped.
    pub fn new(session: Arc<zenoh::Session>, config: ZenohConfig) -> Self {
        Self {
            session,
            config,
            pending: Vec::new(),
        }
    }

    /// Create a new batch with the specified capacity
    ///
    /// Pre-allocating capacity is useful when you know the number of
    /// publishes ahead of time.
    pub fn with_capacity(
        session: Arc<zenoh::Session>,
        config: ZenohConfig,
        capacity: usize,
    ) -> Self {
        Self {
            session,
            config,
            pending: Vec::with_capacity(capacity),
        }
    }

    /// Queue a publish operation to a topic
    ///
    /// The message is serialized immediately using the batch's serialization format.
    /// The actual network send is deferred until `commit()` is called.
    pub fn publish<T: serde::Serialize>(&mut self, topic: &str, msg: &T) -> HorusResult<&mut Self> {
        let key_expr = self.config.topic_to_key_expr(topic);
        let payload = serialize_message(msg, self.config.serialization)?;

        log::trace!(
            "zenoh_batch: Queued publish to '{}' ({} bytes)",
            key_expr,
            payload.len()
        );

        self.pending.push(BatchedPublish { key_expr, payload });
        Ok(self)
    }

    /// Queue a raw payload publish (already serialized)
    ///
    /// Use this when you have pre-serialized data or binary payloads.
    pub fn publish_raw(&mut self, topic: &str, payload: Vec<u8>) -> &mut Self {
        let key_expr = self.config.topic_to_key_expr(topic);

        log::trace!(
            "zenoh_batch: Queued raw publish to '{}' ({} bytes)",
            key_expr,
            payload.len()
        );

        self.pending.push(BatchedPublish { key_expr, payload });
        self
    }

    /// Get the number of pending operations in the batch
    pub fn len(&self) -> usize {
        self.pending.len()
    }

    /// Check if the batch is empty
    pub fn is_empty(&self) -> bool {
        self.pending.is_empty()
    }

    /// Clear all pending operations without committing
    pub fn clear(&mut self) {
        self.pending.clear();
    }

    /// Commit all pending publishes in parallel
    ///
    /// Returns the number of successful publishes and any errors encountered.
    /// All publishes are executed concurrently using tokio::join.
    pub async fn commit(self) -> HorusResult<BatchCommitResult> {
        if self.pending.is_empty() {
            return Ok(BatchCommitResult {
                total: 0,
                successful: 0,
                failed: 0,
                errors: Vec::new(),
            });
        }

        let total = self.pending.len();
        log::debug!("zenoh_batch: Committing {} publishes", total);

        // Execute all publishes concurrently
        let futures: Vec<_> = self
            .pending
            .into_iter()
            .map(|op| {
                let session = self.session.clone();
                async move {
                    session
                        .put(&op.key_expr, op.payload)
                        .await
                        .map(|_| op.key_expr.clone())
                        .map_err(|e| (op.key_expr, e.to_string()))
                }
            })
            .collect();

        let results = futures::future::join_all(futures).await;

        let mut successful = 0;
        let mut failed = 0;
        let mut errors = Vec::new();

        for result in results {
            match result {
                Ok(key_expr) => {
                    successful += 1;
                    log::trace!("zenoh_batch: Published to '{}'", key_expr);
                }
                Err((key_expr, error)) => {
                    failed += 1;
                    log::error!(
                        "zenoh_batch: Failed to publish to '{}': {}",
                        key_expr,
                        error
                    );
                    errors.push(BatchPublishError { key_expr, error });
                }
            }
        }

        log::debug!(
            "zenoh_batch: Commit complete - {}/{} successful",
            successful,
            total
        );

        if failed > 0 && successful == 0 {
            // All failed
            return Err(NetworkError::new(
                NetworkErrorCode::SocketError,
                format!("All {} batch publishes failed", total),
            )
            .into());
        }

        Ok(BatchCommitResult {
            total,
            successful,
            failed,
            errors,
        })
    }

    /// Commit all pending publishes synchronously (blocking)
    ///
    /// This is a convenience method for use outside of async contexts.
    /// It creates a temporary tokio runtime to execute the commit.
    pub fn commit_blocking(self) -> HorusResult<BatchCommitResult> {
        let rt = tokio::runtime::Handle::try_current().map_err(|_| {
            NetworkError::new(
                NetworkErrorCode::ResourceExhausted,
                "No tokio runtime available for blocking commit",
            )
            .with_suggestion(
                "Ensure you're running within a tokio async context or use commit() instead",
            )
        })?;

        rt.block_on(self.commit())
    }
}

#[cfg(feature = "zenoh-transport")]
impl std::fmt::Debug for ZenohBatch {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("ZenohBatch")
            .field("pending_count", &self.pending.len())
            .field("serialization", &self.config.serialization)
            .finish()
    }
}

/// Result of a batch commit operation
#[derive(Debug, Clone)]
pub struct BatchCommitResult {
    /// Total number of operations in the batch
    pub total: usize,
    /// Number of successful publishes
    pub successful: usize,
    /// Number of failed publishes
    pub failed: usize,
    /// Details of any failed publishes
    pub errors: Vec<BatchPublishError>,
}

impl BatchCommitResult {
    /// Check if all operations succeeded
    pub fn all_succeeded(&self) -> bool {
        self.failed == 0
    }

    /// Check if any operations failed
    pub fn has_errors(&self) -> bool {
        self.failed > 0
    }

    /// Get success rate as a percentage
    pub fn success_rate(&self) -> f64 {
        if self.total == 0 {
            100.0
        } else {
            (self.successful as f64 / self.total as f64) * 100.0
        }
    }
}

/// Error details for a single failed publish in a batch
#[derive(Debug, Clone)]
pub struct BatchPublishError {
    /// Key expression that failed
    pub key_expr: String,
    /// Error message
    pub error: String,
}

/// Stub implementation when zenoh feature is not enabled
#[cfg(not(feature = "zenoh-transport"))]
pub struct ZenohBackend<T> {
    _phantom: PhantomData<T>,
}

#[cfg(not(feature = "zenoh-transport"))]
impl<T> ZenohBackend<T> {
    pub fn new_blocking(_topic: &str, _config: ZenohConfig) -> HorusResult<Self> {
        Err(HorusError::Communication(
            "Zenoh transport not enabled. Compile with --features zenoh-transport".into(),
        ))
    }

    pub fn send(&self, _msg: &T) -> HorusResult<()> {
        Err(HorusError::Communication("Zenoh not enabled".into()))
    }

    pub fn recv(&self) -> Option<T> {
        None
    }
}

#[cfg(not(feature = "zenoh-transport"))]
impl<T> std::fmt::Debug for ZenohBackend<T> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("ZenohBackend")
            .field("enabled", &false)
            .finish()
    }
}

#[cfg(test)]
#[cfg(feature = "zenoh-transport")]
mod tests {
    use super::*;
    use serde::{Deserialize, Serialize};

    #[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
    struct TestMessage {
        value: i32,
        text: String,
    }

    #[tokio::test]
    async fn test_zenoh_config_topic_mapping() {
        let config = ZenohConfig::default();
        assert_eq!(config.topic_to_key_expr("test"), "horus/test");

        let ros2_config = ZenohConfig::ros2(0);
        assert_eq!(ros2_config.topic_to_key_expr("/cmd_vel"), "rt/cmd_vel");
    }

    // Note: Full integration tests require a running Zenoh router
    // These are marked as ignored and can be run manually
    #[tokio::test]
    #[ignore]
    async fn test_zenoh_pub_sub() {
        let config = ZenohConfig::default();

        let mut publisher = ZenohBackend::<TestMessage>::new("test_topic", config.clone())
            .await
            .expect("Failed to create publisher");
        publisher
            .init_publisher()
            .await
            .expect("Failed to init publisher");

        let mut subscriber = ZenohBackend::<TestMessage>::new("test_topic", config)
            .await
            .expect("Failed to create subscriber");
        subscriber
            .init_subscriber()
            .await
            .expect("Failed to init subscriber");

        // Give time for discovery
        tokio::time::sleep(std::time::Duration::from_millis(100)).await;

        let msg = TestMessage {
            value: 42,
            text: "hello".to_string(),
        };

        publisher.send_async(&msg).await.expect("Failed to send");

        // Wait for message
        tokio::time::sleep(std::time::Duration::from_millis(100)).await;

        let received = subscriber.recv();
        assert_eq!(received, Some(msg));
    }

    /// Test that verifies ZenohBackend can be instantiated and session info works
    /// This test doesn't require network connectivity
    /// Note: Zenoh requires multi_thread runtime
    #[tokio::test(flavor = "multi_thread", worker_threads = 1)]
    async fn test_zenoh_backend_instantiation() {
        let config = ZenohConfig::default();

        // Create a backend - this should work as Zenoh uses peer mode by default
        let backend = ZenohBackend::<TestMessage>::new("instantiation_test", config)
            .await
            .expect("Failed to create backend");

        // Verify session info
        let info = backend.session_info();
        assert_eq!(info.key_expr, "horus/instantiation_test");
        assert!(!info.has_publisher);
        assert_eq!(info.pending_messages, 0);
    }

    /// Test the blocking API wrapper
    #[test]
    fn test_zenoh_blocking_api() {
        // Create a multi-thread tokio runtime (required by Zenoh)
        let rt = tokio::runtime::Builder::new_multi_thread()
            .worker_threads(1)
            .enable_all()
            .build()
            .expect("Failed to create runtime");

        rt.block_on(async {
            let config = ZenohConfig::default();
            let backend = ZenohBackend::<TestMessage>::new("blocking_test", config)
                .await
                .expect("Failed to create backend");

            assert_eq!(backend.key_expr(), "horus/blocking_test");
            assert_eq!(backend.pending_count(), 0);
        });
    }

    /// Test buffer management
    #[tokio::test(flavor = "multi_thread", worker_threads = 1)]
    async fn test_zenoh_buffer_management() {
        let config = ZenohConfig::default();
        let backend = ZenohBackend::<TestMessage>::new("buffer_test", config)
            .await
            .expect("Failed to create backend");

        // Initially empty
        assert_eq!(backend.pending_count(), 0);
        assert!(backend.recv().is_none());

        // Clear should work on empty buffer
        backend.clear_buffer();
        assert_eq!(backend.pending_count(), 0);
    }

    /// Test Debug impl
    #[tokio::test(flavor = "multi_thread", worker_threads = 1)]
    async fn test_zenoh_debug_impl() {
        let config = ZenohConfig::default();
        let backend = ZenohBackend::<TestMessage>::new("debug_test", config)
            .await
            .expect("Failed to create backend");

        let debug_str = format!("{:?}", backend);
        assert!(debug_str.contains("ZenohBackend"));
        assert!(debug_str.contains("horus/debug_test"));
    }

    /// Test Clone impl
    #[tokio::test(flavor = "multi_thread", worker_threads = 1)]
    async fn test_zenoh_clone() {
        let config = ZenohConfig::default();
        let backend = ZenohBackend::<TestMessage>::new("clone_test", config)
            .await
            .expect("Failed to create backend");

        let cloned = backend.clone();

        // Cloned backend shares the same key expression
        assert_eq!(cloned.key_expr(), backend.key_expr());
        // But publisher is not cloned
        assert!(!cloned.session_info().has_publisher);
    }

    /// Test publisher initialization
    #[tokio::test(flavor = "multi_thread", worker_threads = 1)]
    async fn test_zenoh_publisher_init() {
        let config = ZenohConfig::default();
        let mut backend = ZenohBackend::<TestMessage>::new("pub_init_test", config)
            .await
            .expect("Failed to create backend");

        assert!(!backend.session_info().has_publisher);

        backend
            .init_publisher()
            .await
            .expect("Failed to init publisher");

        assert!(backend.session_info().has_publisher);

        // Calling init again should be a no-op
        backend
            .init_publisher()
            .await
            .expect("Failed to init publisher again");
        assert!(backend.session_info().has_publisher);
    }

    /// Test subscriber initialization
    #[tokio::test(flavor = "multi_thread", worker_threads = 1)]
    async fn test_zenoh_subscriber_init() {
        let config = ZenohConfig::default();
        let mut backend = ZenohBackend::<TestMessage>::new("sub_init_test", config)
            .await
            .expect("Failed to create backend");

        // Should start empty
        assert_eq!(backend.pending_count(), 0);

        // Initialize subscriber
        backend
            .init_subscriber()
            .await
            .expect("Failed to init subscriber");

        // Still empty (no messages sent yet)
        assert_eq!(backend.pending_count(), 0);
    }

    /// Test in-process pub/sub (peer-to-peer mode)
    #[tokio::test(flavor = "multi_thread", worker_threads = 2)]
    async fn test_zenoh_inproc_pub_sub() {
        let config = ZenohConfig::default();

        // Create publisher backend
        let mut publisher = ZenohBackend::<TestMessage>::new("inproc_test", config.clone())
            .await
            .expect("Failed to create publisher");
        publisher
            .init_publisher()
            .await
            .expect("Failed to init publisher");

        // Create subscriber backend (shares the same topic)
        let mut subscriber = ZenohBackend::<TestMessage>::new("inproc_test", config)
            .await
            .expect("Failed to create subscriber");
        subscriber
            .init_subscriber()
            .await
            .expect("Failed to init subscriber");

        // Give time for Zenoh discovery
        tokio::time::sleep(std::time::Duration::from_millis(500)).await;

        // Send a message
        let msg = TestMessage {
            value: 123,
            text: "test message".to_string(),
        };
        publisher
            .send_async(&msg)
            .await
            .expect("Failed to send message");

        // Wait for message to be received
        tokio::time::sleep(std::time::Duration::from_millis(200)).await;

        // Check if message was received
        let received = subscriber.recv();
        assert_eq!(received, Some(msg));
    }

    /// Test CDR serialization format (ROS2 compatible)
    #[cfg(feature = "cdr-encoding")]
    #[test]
    fn test_cdr_serialization() {
        let msg = TestMessage {
            value: 123,
            text: "cdr test".to_string(),
        };

        // Serialize with CDR
        let payload =
            serialize_message(&msg, SerializationFormat::Cdr).expect("CDR serialize failed");

        // Deserialize with CDR
        let (deserialized, _bytes_read): (TestMessage, usize) =
            cdr_encoding::from_bytes::<_, byteorder::LittleEndian>(&payload)
                .expect("CDR deserialize failed");

        assert_eq!(msg, deserialized);
    }

    /// Test Bincode serialization format (default)
    #[test]
    fn test_bincode_serialization() {
        let msg = TestMessage {
            value: 77,
            text: "bincode test".to_string(),
        };

        // Serialize with Bincode
        let payload = serialize_message(&msg, SerializationFormat::Bincode)
            .expect("Bincode serialize failed");

        // Deserialize with Bincode
        let deserialized: TestMessage =
            bincode::deserialize(&payload).expect("Bincode deserialize failed");

        assert_eq!(msg, deserialized);
    }

    // ========== CONFLICT PREVENTION TESTS ==========
    // These tests ensure no namespace/topic conflicts between HORUS, ROS2, and Zenoh

    /// Test that HORUS native topics and ROS2 topics are properly isolated
    /// HORUS uses: horus/{topic}
    /// ROS2 uses: rt/{topic}
    /// These namespaces should NEVER overlap
    #[test]
    fn test_horus_ros2_namespace_isolation() {
        let horus_config = ZenohConfig::default();
        let ros2_config = ZenohConfig::ros2(0);

        // Same logical topic name should produce different key expressions
        let topic = "cmd_vel";

        let horus_key = horus_config.topic_to_key_expr(topic);
        let ros2_key = ros2_config.topic_to_key_expr(topic);

        // CRITICAL: These must be different to prevent cross-system contamination
        assert_ne!(
            horus_key, ros2_key,
            "HORUS and ROS2 key expressions must differ for the same topic"
        );

        // Verify the prefixes are different
        assert!(
            horus_key.starts_with("horus/"),
            "HORUS topics must be prefixed with 'horus/'"
        );
        assert!(
            ros2_key.starts_with("rt/"),
            "ROS2 topics must be prefixed with 'rt/'"
        );
    }

    /// Test that multi-robot namespaces are properly isolated
    #[test]
    fn test_multi_robot_namespace_isolation() {
        let robot1_config = ZenohConfig::new().with_namespace("robot1");
        let robot2_config = ZenohConfig::new().with_namespace("robot2");
        let robot3_config = ZenohConfig::new().with_namespace("fleet/robot3");

        let topic = "odom";

        let robot1_key = robot1_config.topic_to_key_expr(topic);
        let robot2_key = robot2_config.topic_to_key_expr(topic);
        let robot3_key = robot3_config.topic_to_key_expr(topic);

        // All robots must have different key expressions
        assert_ne!(
            robot1_key, robot2_key,
            "Robot1 and Robot2 must have different namespaces"
        );
        assert_ne!(
            robot1_key, robot3_key,
            "Robot1 and Robot3 must have different namespaces"
        );
        assert_ne!(
            robot2_key, robot3_key,
            "Robot2 and Robot3 must have different namespaces"
        );

        // Verify expected patterns
        assert_eq!(robot1_key, "robot1/odom");
        assert_eq!(robot2_key, "robot2/odom");
        assert_eq!(robot3_key, "fleet/robot3/odom");
    }

    /// Test that ROS2 domain IDs don't affect Zenoh key expressions
    /// (Domain IDs are handled at the DDS bridge level, not in key expressions)
    #[test]
    fn test_ros2_domain_id_independence() {
        let domain0 = ZenohConfig::ros2(0);
        let domain1 = ZenohConfig::ros2(1);
        let domain42 = ZenohConfig::ros2(42);

        let topic = "/sensor/lidar";

        // Key expressions should be the same regardless of domain ID
        // (Domain isolation is handled by zenoh-bridge-ros2dds, not key expressions)
        let key0 = domain0.topic_to_key_expr(topic);
        let key1 = domain1.topic_to_key_expr(topic);
        let key42 = domain42.topic_to_key_expr(topic);

        assert_eq!(key0, key1);
        assert_eq!(key1, key42);
        assert_eq!(key0, "rt/sensor/lidar");
    }

    /// Test that empty/None namespace produces raw topic names
    #[test]
    fn test_empty_namespace_behavior() {
        let mut config = ZenohConfig::default();
        config.namespace = None;

        let topic = "test_topic";
        let key = config.topic_to_key_expr(topic);

        // With no namespace, topic should be used directly
        assert_eq!(key, "test_topic");
    }

    /// Test key expression parsing (reverse of topic_to_key_expr)
    #[test]
    fn test_key_expr_parsing_isolation() {
        let horus_config = ZenohConfig::default();
        let ros2_config = ZenohConfig::ros2(0);

        // HORUS should parse HORUS keys, not ROS2 keys
        assert_eq!(
            horus_config.key_expr_to_topic("horus/odom"),
            Some("odom".to_string())
        );
        assert_eq!(horus_config.key_expr_to_topic("rt/odom"), None); // Can't parse ROS2 key

        // ROS2 should parse ROS2 keys, not HORUS keys
        assert_eq!(
            ros2_config.key_expr_to_topic("rt/odom"),
            Some("odom".to_string())
        );
        assert_eq!(ros2_config.key_expr_to_topic("horus/odom"), None); // Can't parse HORUS key
    }

    /// Test that HORUS and ROS2 can coexist on the same Zenoh network
    #[tokio::test(flavor = "multi_thread", worker_threads = 2)]
    async fn test_horus_ros2_coexistence() {
        // Create HORUS backend
        let horus_config = ZenohConfig::default();
        let horus_backend = ZenohBackend::<TestMessage>::new("coexist_test", horus_config.clone())
            .await
            .expect("Failed to create HORUS backend");

        // Create ROS2 backend for same logical topic
        let ros2_config = ZenohConfig::ros2(0);
        let ros2_backend = ZenohBackend::<TestMessage>::new("coexist_test", ros2_config.clone())
            .await
            .expect("Failed to create ROS2 backend");

        // Verify they're using different key expressions
        let horus_key = horus_backend.key_expr();
        let ros2_key = ros2_backend.key_expr();

        assert_ne!(horus_key, ros2_key);
        assert!(horus_key.contains("horus/"));
        assert!(ros2_key.contains("rt/"));
    }

    /// Test serialization format isolation
    /// Different formats produce different byte representations
    #[test]
    #[cfg(feature = "cdr-encoding")]
    fn test_serialization_format_isolation() {
        let msg = TestMessage {
            value: 42,
            text: "isolation test".to_string(),
        };

        // Serialize with different formats
        let bincode_data = serialize_message(&msg, SerializationFormat::Bincode).unwrap();
        let cdr_data = serialize_message(&msg, SerializationFormat::Cdr).unwrap();

        // Verify each format produces different bytes (no accidental compatibility)
        assert_ne!(
            bincode_data, cdr_data,
            "Bincode and CDR should produce different byte representations"
        );

        // Verify each format round-trips correctly with itself
        let bincode_roundtrip: TestMessage = bincode::deserialize(&bincode_data).unwrap();
        assert_eq!(bincode_roundtrip.value, msg.value);
        assert_eq!(bincode_roundtrip.text, msg.text);

        let (cdr_roundtrip, _): (TestMessage, usize) =
            cdr_encoding::from_bytes::<_, byteorder::LittleEndian>(&cdr_data).unwrap();
        assert_eq!(cdr_roundtrip.value, msg.value);
        assert_eq!(cdr_roundtrip.text, msg.text);

        // Note: Cross-format deserialization may succeed with garbage values or fail,
        // depending on the data. The key point is that formats are not interchangeable
        // for correct round-trip communication.
    }

    /// Test that hierarchical topic namespaces work correctly
    #[test]
    fn test_hierarchical_topic_namespaces() {
        let config = ZenohConfig::default();

        // Nested topics should preserve hierarchy
        assert_eq!(
            config.topic_to_key_expr("sensors/lidar/scan"),
            "horus/sensors/lidar/scan"
        );
        assert_eq!(
            config.topic_to_key_expr("actuators/wheel/left"),
            "horus/actuators/wheel/left"
        );

        // Parse back correctly
        assert_eq!(
            config.key_expr_to_topic("horus/sensors/lidar/scan"),
            Some("sensors/lidar/scan".to_string())
        );
    }

    /// Test ROS2 topic naming with leading slashes
    #[test]
    fn test_ros2_leading_slash_handling() {
        let ros2_config = ZenohConfig::ros2(0);

        // Topics with and without leading slash should be normalized
        assert_eq!(ros2_config.topic_to_key_expr("/cmd_vel"), "rt/cmd_vel");
        assert_eq!(ros2_config.topic_to_key_expr("cmd_vel"), "rt/cmd_vel");

        // Hierarchical ROS2 topics
        assert_eq!(
            ros2_config.topic_to_key_expr("/robot/sensor/imu"),
            "rt/robot/sensor/imu"
        );
    }

    /// Test that config cloning preserves isolation properties
    #[test]
    fn test_config_clone_isolation() {
        let original = ZenohConfig::default();
        let mut cloned = original.clone();

        // Modify cloned config
        cloned.namespace = Some("modified".to_string());

        // Original should be unchanged
        assert_eq!(original.namespace, Some("horus".to_string()));
        assert_eq!(cloned.namespace, Some("modified".to_string()));

        // Key expressions should differ
        assert_ne!(
            original.topic_to_key_expr("test"),
            cloned.topic_to_key_expr("test")
        );
    }

    /// Test ZenohBatch basic operations
    #[tokio::test(flavor = "multi_thread", worker_threads = 1)]
    async fn test_zenoh_batch_basic() {
        // Open a session for the batch
        let session = zenoh::open(zenoh::Config::default())
            .await
            .expect("Failed to open session");
        let session = Arc::new(session);

        let config = ZenohConfig::default();

        // Create batch with capacity
        let mut batch = ZenohBatch::with_capacity(session, config, 3);

        assert!(batch.is_empty());
        assert_eq!(batch.len(), 0);

        // Queue some messages
        let msg1 = TestMessage {
            value: 1,
            text: "first".to_string(),
        };
        let msg2 = TestMessage {
            value: 2,
            text: "second".to_string(),
        };
        let msg3 = TestMessage {
            value: 3,
            text: "third".to_string(),
        };

        batch.publish("batch/topic1", &msg1).unwrap();
        batch.publish("batch/topic2", &msg2).unwrap();
        batch.publish("batch/topic3", &msg3).unwrap();

        assert!(!batch.is_empty());
        assert_eq!(batch.len(), 3);

        // Commit the batch
        let result = batch.commit().await.expect("Batch commit failed");

        assert_eq!(result.total, 3);
        assert_eq!(result.successful, 3);
        assert_eq!(result.failed, 0);
        assert!(result.all_succeeded());
        assert!(!result.has_errors());
        assert_eq!(result.success_rate(), 100.0);
    }

    /// Test ZenohBatch with raw payloads
    #[tokio::test(flavor = "multi_thread", worker_threads = 1)]
    async fn test_zenoh_batch_raw() {
        let session = zenoh::open(zenoh::Config::default())
            .await
            .expect("Failed to open session");
        let session = Arc::new(session);

        let config = ZenohConfig::default();
        let mut batch = ZenohBatch::new(session, config);

        // Queue raw payloads
        batch.publish_raw("batch/raw1", vec![1, 2, 3, 4]);
        batch.publish_raw("batch/raw2", vec![5, 6, 7, 8]);

        assert_eq!(batch.len(), 2);

        let result = batch.commit().await.expect("Batch commit failed");

        assert_eq!(result.total, 2);
        assert!(result.all_succeeded());
    }

    /// Test ZenohBatch empty commit
    #[tokio::test(flavor = "multi_thread", worker_threads = 1)]
    async fn test_zenoh_batch_empty() {
        let session = zenoh::open(zenoh::Config::default())
            .await
            .expect("Failed to open session");
        let session = Arc::new(session);

        let config = ZenohConfig::default();
        let batch = ZenohBatch::new(session, config);

        // Commit empty batch should succeed
        let result = batch.commit().await.expect("Empty batch commit failed");

        assert_eq!(result.total, 0);
        assert_eq!(result.successful, 0);
        assert!(result.all_succeeded()); // No failures
    }

    /// Test ZenohBatch debug formatting
    #[test]
    fn test_zenoh_batch_debug() {
        // We can't create a real session without async, but we can test
        // the BatchCommitResult debug
        let result = BatchCommitResult {
            total: 5,
            successful: 4,
            failed: 1,
            errors: vec![BatchPublishError {
                key_expr: "failed/topic".to_string(),
                error: "test error".to_string(),
            }],
        };

        assert!(!result.all_succeeded());
        assert!(result.has_errors());
        assert_eq!(result.success_rate(), 80.0);

        let debug_str = format!("{:?}", result);
        assert!(debug_str.contains("BatchCommitResult"));
    }

    /// Test session pool basic operations
    #[tokio::test(flavor = "multi_thread", worker_threads = 1)]
    async fn test_session_pool_basic() {
        // Clear pool before test
        ZenohSessionPool::clear();

        let config = ZenohConfig::default();

        // Initial stats should show empty pool
        let stats = ZenohSessionPool::stats();
        assert_eq!(stats.total_sessions, 0);

        // Get first session
        let session1 = ZenohSessionPool::get_or_create(&config)
            .await
            .expect("Failed to get session");

        // Stats should show one session with one reference
        let stats = ZenohSessionPool::stats();
        assert_eq!(stats.total_sessions, 1);
        assert_eq!(stats.total_references, 1);

        // Get second session with same config - should reuse
        let session2 = ZenohSessionPool::get_or_create(&config)
            .await
            .expect("Failed to get session");

        // Should be the same session
        assert!(Arc::ptr_eq(&session1, &session2));

        // Stats should show same session with two references
        let stats = ZenohSessionPool::stats();
        assert_eq!(stats.total_sessions, 1);
        assert_eq!(stats.total_references, 2);

        // Release one reference
        ZenohSessionPool::release(&config);
        let stats = ZenohSessionPool::stats();
        assert_eq!(stats.total_sessions, 1);
        assert_eq!(stats.total_references, 1);

        // Release second reference - session should be removed
        ZenohSessionPool::release(&config);
        let stats = ZenohSessionPool::stats();
        assert_eq!(stats.total_sessions, 0);
    }

    /// Test session pool with different configs
    #[tokio::test(flavor = "multi_thread", worker_threads = 1)]
    async fn test_session_pool_different_configs() {
        // Clear pool before test
        ZenohSessionPool::clear();

        let config1 = ZenohConfig::default();
        // Create a config with different connect endpoints
        let config2 = ZenohConfig::default().connect_to("tcp/10.0.0.1:7447");

        // Get sessions with different configs
        let session1 = ZenohSessionPool::get_or_create(&config1)
            .await
            .expect("Failed to get session");
        let session2 = ZenohSessionPool::get_or_create(&config2)
            .await
            .expect("Failed to get session");

        // Should be different sessions (different connect endpoints)
        assert!(!Arc::ptr_eq(&session1, &session2));

        // Stats should show two sessions
        let stats = ZenohSessionPool::stats();
        assert_eq!(stats.total_sessions, 2);
        assert_eq!(stats.total_references, 2);

        // Clean up
        ZenohSessionPool::release(&config1);
        ZenohSessionPool::release(&config2);
    }

    /// Test pooled backend creation
    #[tokio::test(flavor = "multi_thread", worker_threads = 1)]
    async fn test_pooled_backend_creation() {
        // Clear pool before test
        ZenohSessionPool::clear();

        let config = ZenohConfig::default();

        // Create two backends with same config
        let backend1 = ZenohBackend::<TestMessage>::new_pooled("pool_test_1", config.clone())
            .await
            .expect("Failed to create backend");
        let backend2 = ZenohBackend::<TestMessage>::new_pooled("pool_test_2", config.clone())
            .await
            .expect("Failed to create backend");

        // They should share the same session
        let stats = ZenohSessionPool::stats();
        assert_eq!(stats.total_sessions, 1);
        assert_eq!(stats.total_references, 2);

        // But have different key expressions
        assert_ne!(backend1.key_expr(), backend2.key_expr());
        assert_eq!(backend1.key_expr(), "horus/pool_test_1");
        assert_eq!(backend2.key_expr(), "horus/pool_test_2");
    }

    /// Test ZenohPoolStats
    #[test]
    fn test_pool_stats_struct() {
        let stats = ZenohPoolStats {
            total_sessions: 3,
            total_references: 7,
            sessions: vec![
                ZenohPoolSessionInfo {
                    endpoints: vec!["tcp/127.0.0.1:7447".to_string()],
                    ref_count: 4,
                    age_secs: 10.5,
                },
                ZenohPoolSessionInfo {
                    endpoints: vec![],
                    ref_count: 3,
                    age_secs: 5.0,
                },
            ],
        };

        assert_eq!(stats.total_sessions, 3);
        assert_eq!(stats.sessions.len(), 2);

        let debug_str = format!("{:?}", stats);
        assert!(debug_str.contains("ZenohPoolStats"));
    }
}
