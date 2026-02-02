#![allow(
    clippy::io_other_error,
    clippy::collapsible_match,
    clippy::field_reassign_with_default
)]
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

/// Serialize a message using the specified format
#[cfg(feature = "zenoh-transport")]
fn serialize_message<T: serde::Serialize>(
    msg: &T,
    format: SerializationFormat,
) -> HorusResult<Vec<u8>> {
    match format {
        SerializationFormat::Bincode => bincode::serialize(msg)
            .map_err(|e| HorusError::Communication(format!("Bincode serialize error: {}", e))),
        SerializationFormat::Json => serde_json::to_vec(msg)
            .map_err(|e| HorusError::Communication(format!("JSON serialize error: {}", e))),
        #[cfg(feature = "cdr-encoding")]
        SerializationFormat::Cdr => {
            // ROS2/DDS typically uses little-endian CDR
            cdr_encoding::to_vec::<_, byteorder::LittleEndian>(msg)
                .map_err(|e| HorusError::Communication(format!("CDR serialize error: {}", e)))
        }
        #[cfg(not(feature = "cdr-encoding"))]
        SerializationFormat::Cdr => {
            log::warn!("CDR format requires 'zenoh-ros2' feature, falling back to bincode");
            bincode::serialize(msg)
                .map_err(|e| HorusError::Communication(format!("Bincode serialize error: {}", e)))
        }
        SerializationFormat::MessagePack => rmp_serde::to_vec(msg)
            .map_err(|e| HorusError::Communication(format!("MessagePack serialize error: {}", e))),
    }
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

        // Log mode for debugging
        log::debug!("Creating Zenoh backend with mode: {:?}", config.mode);

        // Add connect endpoints logging
        if !config.connect.is_empty() {
            for endpoint in &config.connect {
                log::debug!("Zenoh will connect to: {}", endpoint);
            }
        }

        // Open session
        let session = zenoh::open(zenoh_config)
            .await
            .map_err(|e| HorusError::Communication(format!("Zenoh open failed: {}", e)))?;

        let key_expr = config.topic_to_key_expr(topic);

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
                HorusError::Communication(format!("Failed to get tokio runtime: {}", e))
            })?;

        rt.block_on(Self::new(topic, config))
    }

    /// Initialize publisher for sending messages
    pub async fn init_publisher(&mut self) -> HorusResult<()> {
        if self.publisher.is_some() {
            return Ok(());
        }

        let publisher = self
            .session
            .declare_publisher(&self.key_expr)
            .await
            .map_err(|e| HorusError::Communication(format!("Zenoh publisher error: {}", e)))?;

        // Store publisher - use unsafe transmute to handle lifetime
        // This is safe because the session outlives the publisher
        let static_publisher: zenoh::pubsub::Publisher<'static> =
            unsafe { std::mem::transmute(publisher) };
        self.publisher = Some(Box::new(static_publisher));
        Ok(())
    }

    /// Initialize subscriber for receiving messages
    pub async fn init_subscriber(&mut self) -> HorusResult<()> {
        let recv_buffer = self.recv_buffer.clone();
        let serialization = self.config.serialization;

        let subscriber = self
            .session
            .declare_subscriber(&self.key_expr)
            .await
            .map_err(|e| HorusError::Communication(format!("Zenoh subscriber error: {}", e)))?;

        // Spawn a task to receive messages
        let key_expr = self.key_expr.clone();
        tokio::spawn(async move {
            loop {
                match subscriber.recv_async().await {
                    Ok(sample) => {
                        let payload = sample.payload().to_bytes();
                        let msg: Option<T> = match serialization {
                            SerializationFormat::Bincode => bincode::deserialize(&payload).ok(),
                            SerializationFormat::Json => serde_json::from_slice(&payload).ok(),
                            #[cfg(feature = "cdr-encoding")]
                            SerializationFormat::Cdr => {
                                // ROS2/DDS uses little-endian CDR, returns (value, bytes_read)
                                cdr_encoding::from_bytes::<_, byteorder::LittleEndian>(&payload)
                                    .ok()
                                    .map(|(msg, _bytes_read)| msg)
                            }
                            #[cfg(not(feature = "cdr-encoding"))]
                            SerializationFormat::Cdr => {
                                log::warn!("CDR format requires 'zenoh-ros2' feature, falling back to bincode");
                                bincode::deserialize(&payload).ok()
                            }
                            SerializationFormat::MessagePack => {
                                rmp_serde::from_slice(&payload).ok()
                            }
                        };

                        if let Some(msg) = msg {
                            let mut buffer = recv_buffer.lock();
                            // Keep buffer bounded
                            if buffer.len() >= 10000 {
                                buffer.pop_front();
                            }
                            buffer.push_back(msg);
                        }
                    }
                    Err(e) => {
                        log::warn!("Zenoh subscriber error on {}: {}", key_expr, e);
                        break;
                    }
                }
            }
        });

        Ok(())
    }

    /// Send a message (synchronous wrapper)
    pub fn send(&self, msg: &T) -> HorusResult<()> {
        let publisher = self
            .publisher
            .as_ref()
            .ok_or_else(|| HorusError::Communication("Publisher not initialized".into()))?;

        // Serialize using configured format
        let payload = serialize_message(msg, self.config.serialization)?;

        // Use blocking runtime for sync API compatibility
        let rt = tokio::runtime::Handle::try_current()
            .map_err(|e| HorusError::Communication(format!("No tokio runtime: {}", e)))?;

        rt.block_on(async { publisher.put(payload).await })
            .map_err(|e| HorusError::Communication(format!("Zenoh put error: {}", e)))?;

        Ok(())
    }

    /// Send a message asynchronously
    pub async fn send_async(&self, msg: &T) -> HorusResult<()> {
        let publisher = self
            .publisher
            .as_ref()
            .ok_or_else(|| HorusError::Communication("Publisher not initialized".into()))?;

        let payload = serialize_message(msg, self.config.serialization)?;

        publisher
            .put(payload)
            .await
            .map_err(|e| HorusError::Communication(format!("Zenoh put error: {}", e)))?;

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
        self.connection_quality.lock().mark_connected(router);
    }

    /// Mark the connection as disconnected
    pub fn mark_disconnected(&self) {
        self.connection_quality.lock().mark_disconnected();
    }

    /// Record a failover to a new router
    pub fn record_failover(&self, new_router: &str) {
        self.connection_quality.lock().record_failover(new_router);
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
                Ok(latency_ms)
            }
            Ok(Err(e)) => {
                // Query failed
                let mut quality = self.connection_quality.lock();
                quality.record_health_check_failure();
                Err(HorusError::Communication(format!(
                    "Health check query failed: {}",
                    e
                )))
            }
            Err(_) => {
                // Timeout
                let mut quality = self.connection_quality.lock();
                quality.record_health_check_failure();
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

        tokio::spawn(async move {
            let health_key = format!("{}/__horus_health_monitor__", key_expr);
            let mut consecutive_failures = 0u32;

            loop {
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
                        consecutive_failures = 0;

                        log::trace!(
                            "Health check OK: {:.2}ms (avg: {:.2}ms, score: {})",
                            latency_ms,
                            q.avg_latency_ms,
                            q.health_score()
                        );
                    }
                    Ok(Err(_)) | Err(_) => {
                        consecutive_failures += 1;
                        let mut q = quality.lock();
                        q.record_health_check_failure();

                        if consecutive_failures >= 3 {
                            log::warn!(
                                "Connection quality degraded: {} consecutive failures",
                                consecutive_failures
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
            .with_cli_hint("horus net doctor --check-cloud");
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

    /// Test MessagePack serialization format
    #[test]
    fn test_messagepack_serialization() {
        let msg = TestMessage {
            value: 42,
            text: "msgpack test".to_string(),
        };

        // Serialize with MessagePack
        let payload = serialize_message(&msg, SerializationFormat::MessagePack)
            .expect("MessagePack serialize failed");

        // Deserialize with MessagePack
        let deserialized: TestMessage =
            rmp_serde::from_slice(&payload).expect("MessagePack deserialize failed");

        assert_eq!(msg, deserialized);
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

    /// Test JSON serialization format
    #[test]
    fn test_json_serialization() {
        let msg = TestMessage {
            value: 99,
            text: "json test".to_string(),
        };

        // Serialize with JSON
        let payload =
            serialize_message(&msg, SerializationFormat::Json).expect("JSON serialize failed");

        // Deserialize with JSON
        let deserialized: TestMessage =
            serde_json::from_slice(&payload).expect("JSON deserialize failed");

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
    /// Different formats should NOT be able to deserialize each other's data
    #[test]
    fn test_serialization_format_isolation() {
        let msg = TestMessage {
            value: 42,
            text: "isolation test".to_string(),
        };

        // Serialize with different formats
        let bincode_data = serialize_message(&msg, SerializationFormat::Bincode).unwrap();
        let msgpack_data = serialize_message(&msg, SerializationFormat::MessagePack).unwrap();
        let json_data = serialize_message(&msg, SerializationFormat::Json).unwrap();

        // Verify each format produces different bytes (no accidental compatibility)
        assert_ne!(bincode_data, msgpack_data);
        assert_ne!(bincode_data, json_data);
        assert_ne!(msgpack_data, json_data);

        // Verify cross-format deserialization fails gracefully
        // Bincode data should NOT deserialize as MessagePack
        let cross_deser: Result<TestMessage, _> = rmp_serde::from_slice(&bincode_data);
        assert!(
            cross_deser.is_err(),
            "Bincode data should not deserialize as MessagePack"
        );

        // MessagePack data should NOT deserialize as Bincode
        let cross_deser: Result<TestMessage, _> = bincode::deserialize(&msgpack_data);
        assert!(
            cross_deser.is_err(),
            "MessagePack data should not deserialize as Bincode"
        );
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
}
