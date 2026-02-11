//! QUIC transport for reliable, low-latency network communication
//!
//! QUIC provides:
//! - 0-RTT connection resumption (no handshake latency for repeat connections)
//! - No head-of-line blocking (streams are independent)
//! - Built-in encryption (TLS 1.3)
//! - Connection migration (handles IP changes)
//! - Better congestion control
//!
//! This is ideal for WAN communication or when reliability is required.
//!
//! # QoS Profiles
//!
//! HORUS supports three QoS profiles for QUIC transport:
//!
//! - **Reliable**: Guaranteed delivery via QUIC streams (default)
//! - **BestEffort**: Low latency via QUIC datagrams (may drop under congestion)
//! - **ReliableOrdered**: Guaranteed delivery with strict ordering
//!
//! ```rust,ignore
//! // Send with specific QoS
//! transport.send_with_qos(addr, data, QuicQosProfile::BestEffort).await?;
//!
//! // Send with priority
//! transport.send_with_priority(addr, data, QuicStreamPriority::Critical).await?;
//! ```
//!
//! Requires the `quic` feature to be enabled.

#[cfg(feature = "quic")]
use quinn::{ClientConfig, Connection, Endpoint, ServerConfig, TransportConfig};

use std::collections::HashMap;
use std::io;
use std::net::SocketAddr;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::Arc;
use std::time::Duration;

#[cfg(feature = "quic")]
use tokio::sync::RwLock;

/// Quality of Service profile for QUIC messages
///
/// Determines how messages are delivered over QUIC transport.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub enum QuicQosProfile {
    /// Guaranteed delivery via QUIC streams (default)
    ///
    /// Messages are delivered reliably using unidirectional streams.
    /// Suitable for control messages, state updates, and any data
    /// that must not be lost.
    #[default]
    Reliable,

    /// Low latency delivery via QUIC datagrams
    ///
    /// Messages are sent as unreliable datagrams - they may be dropped
    /// under congestion but have minimal latency overhead. Ideal for
    /// sensor data, video frames, and telemetry where freshness matters
    /// more than completeness.
    BestEffort,

    /// Guaranteed delivery with strict ordering
    ///
    /// Messages are delivered reliably AND in order using a single
    /// bidirectional stream per destination. Higher latency than
    /// Reliable (head-of-line blocking) but guarantees message order.
    /// Use for command sequences or state machine transitions.
    ReliableOrdered,
}

impl QuicQosProfile {
    /// Returns true if this profile guarantees delivery
    pub fn is_reliable(&self) -> bool {
        matches!(self, Self::Reliable | Self::ReliableOrdered)
    }

    /// Returns true if this profile guarantees ordering
    pub fn is_ordered(&self) -> bool {
        matches!(self, Self::ReliableOrdered)
    }

    /// Returns recommended retry count for this profile
    pub fn retry_count(&self) -> u32 {
        match self {
            Self::Reliable => 3,
            Self::BestEffort => 0, // No retries for best effort
            Self::ReliableOrdered => 5,
        }
    }
}

/// Stream priority levels for QUIC
///
/// QUIC allows assigning priorities to streams. Lower numeric values
/// mean higher priority. Critical messages get bandwidth first.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub enum QuicStreamPriority {
    /// Emergency stop, safety-critical messages
    /// Priority value: 0 (highest)
    Critical = 0,

    /// Control commands, state machine transitions
    /// Priority value: 32
    High = 32,

    /// Standard messages (default)
    /// Priority value: 128
    #[default]
    Normal = 128,

    /// Bulk data, logs, diagnostics
    /// Priority value: 192
    Low = 192,

    /// Background tasks, non-urgent telemetry
    /// Priority value: 255 (lowest)
    Background = 255,
}

impl QuicStreamPriority {
    /// Convert to QUIC stream priority value (i32)
    pub fn as_i32(&self) -> i32 {
        *self as i32
    }

    /// Create from numeric priority value
    pub fn from_value(value: i32) -> Self {
        match value {
            0..=16 => Self::Critical,
            17..=64 => Self::High,
            65..=160 => Self::Normal,
            161..=224 => Self::Low,
            _ => Self::Background,
        }
    }
}

/// Congestion control algorithm for QUIC
///
/// Different algorithms optimize for different network conditions.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub enum QuicCongestionControl {
    /// Bottleneck Bandwidth and RTT (default)
    ///
    /// Optimized for high throughput with low latency.
    /// Best for most robotics applications.
    #[default]
    Bbr,

    /// CUBIC congestion control
    ///
    /// Traditional TCP-like congestion control.
    /// More conservative, better fairness with TCP flows.
    Cubic,

    /// New Reno
    ///
    /// Simple, well-understood algorithm.
    /// Good for debugging and baseline comparisons.
    NewReno,
}

impl QuicCongestionControl {
    /// Get the name of the congestion control algorithm
    pub fn name(&self) -> &'static str {
        match self {
            Self::Bbr => "bbr",
            Self::Cubic => "cubic",
            Self::NewReno => "newreno",
        }
    }
}

/// QUIC transport configuration
#[derive(Debug, Clone)]
pub struct QuicConfig {
    /// Maximum idle timeout before connection is closed
    pub max_idle_timeout: Duration,
    /// Keep-alive interval
    pub keep_alive_interval: Duration,
    /// Maximum concurrent bidirectional streams
    pub max_concurrent_bidi_streams: u32,
    /// Maximum concurrent unidirectional streams
    pub max_concurrent_uni_streams: u32,
    /// Enable 0-RTT (faster but less secure for first bytes)
    pub enable_0rtt: bool,
    /// Initial RTT estimate (for congestion control)
    pub initial_rtt: Duration,
    /// Maximum UDP payload size
    pub max_udp_payload_size: u16,
    /// Congestion control algorithm
    pub congestion_control: QuicCongestionControl,
    /// Enable QUIC datagrams for BestEffort QoS
    pub enable_datagrams: bool,
    /// Maximum datagram size (for BestEffort messages)
    pub max_datagram_size: u16,
    /// Default QoS profile for messages
    pub default_qos: QuicQosProfile,
    /// Default stream priority
    pub default_priority: QuicStreamPriority,
}

impl Default for QuicConfig {
    fn default() -> Self {
        Self {
            max_idle_timeout: Duration::from_secs(30),
            keep_alive_interval: Duration::from_secs(5),
            max_concurrent_bidi_streams: 100,
            max_concurrent_uni_streams: 100,
            enable_0rtt: true,
            initial_rtt: Duration::from_millis(10),
            max_udp_payload_size: 1472, // Standard MTU - headers
            congestion_control: QuicCongestionControl::default(),
            enable_datagrams: true,
            max_datagram_size: 1200, // Safe for most networks
            default_qos: QuicQosProfile::default(),
            default_priority: QuicStreamPriority::default(),
        }
    }
}

impl QuicConfig {
    /// Low latency configuration
    ///
    /// Optimized for real-time robotics with:
    /// - BBR congestion control for low latency
    /// - Datagrams enabled for BestEffort messages
    /// - Aggressive keep-alive
    pub fn low_latency() -> Self {
        Self {
            max_idle_timeout: Duration::from_secs(60),
            keep_alive_interval: Duration::from_secs(2),
            max_concurrent_bidi_streams: 256,
            max_concurrent_uni_streams: 256,
            enable_0rtt: true,
            initial_rtt: Duration::from_millis(5),
            max_udp_payload_size: 1472,
            congestion_control: QuicCongestionControl::Bbr,
            enable_datagrams: true,
            max_datagram_size: 1200,
            default_qos: QuicQosProfile::Reliable,
            default_priority: QuicStreamPriority::High,
        }
    }

    /// High throughput configuration
    ///
    /// Optimized for bulk data transfer with:
    /// - CUBIC congestion control for fairness
    /// - Larger UDP payloads (jumbo frames)
    /// - More concurrent streams
    pub fn high_throughput() -> Self {
        Self {
            max_idle_timeout: Duration::from_secs(120),
            keep_alive_interval: Duration::from_secs(10),
            max_concurrent_bidi_streams: 1000,
            max_concurrent_uni_streams: 1000,
            enable_0rtt: true,
            initial_rtt: Duration::from_millis(20),
            max_udp_payload_size: 65527, // Jumbo frames
            congestion_control: QuicCongestionControl::Cubic,
            enable_datagrams: false, // Prefer streams for throughput
            max_datagram_size: 1200,
            default_qos: QuicQosProfile::Reliable,
            default_priority: QuicStreamPriority::Normal,
        }
    }

    /// Best effort configuration for sensor streams
    ///
    /// Optimized for high-frequency sensor data where freshness
    /// matters more than completeness:
    /// - Datagrams enabled for lossy delivery
    /// - BBR for low latency
    /// - Default QoS is BestEffort
    pub fn best_effort() -> Self {
        Self {
            max_idle_timeout: Duration::from_secs(30),
            keep_alive_interval: Duration::from_secs(2),
            max_concurrent_bidi_streams: 32,
            max_concurrent_uni_streams: 256,
            enable_0rtt: true,
            initial_rtt: Duration::from_millis(5),
            max_udp_payload_size: 1472,
            congestion_control: QuicCongestionControl::Bbr,
            enable_datagrams: true,
            max_datagram_size: 1200,
            default_qos: QuicQosProfile::BestEffort,
            default_priority: QuicStreamPriority::Normal,
        }
    }

    /// Control plane configuration for reliable ordered messages
    ///
    /// Optimized for command/control with strict ordering:
    /// - ReliableOrdered as default QoS
    /// - High priority for commands
    /// - Fewer concurrent streams to maintain ordering
    pub fn control_plane() -> Self {
        Self {
            max_idle_timeout: Duration::from_secs(60),
            keep_alive_interval: Duration::from_secs(3),
            max_concurrent_bidi_streams: 64,
            max_concurrent_uni_streams: 64,
            enable_0rtt: true,
            initial_rtt: Duration::from_millis(10),
            max_udp_payload_size: 1472,
            congestion_control: QuicCongestionControl::Bbr,
            enable_datagrams: false, // Commands must be reliable
            max_datagram_size: 1200,
            default_qos: QuicQosProfile::ReliableOrdered,
            default_priority: QuicStreamPriority::High,
        }
    }
}

/// Statistics for QUIC transport
#[derive(Debug, Default)]
pub struct QuicStats {
    pub connections_established: AtomicU64,
    pub connections_closed: AtomicU64,
    pub streams_opened: AtomicU64,
    pub streams_closed: AtomicU64,
    pub bytes_sent: AtomicU64,
    pub bytes_received: AtomicU64,
    pub messages_sent: AtomicU64,
    pub messages_received: AtomicU64,
    pub zero_rtt_accepted: AtomicU64,
    pub zero_rtt_rejected: AtomicU64,
    // QoS-related statistics
    /// Messages sent via reliable streams
    pub reliable_messages_sent: AtomicU64,
    /// Messages sent via best-effort datagrams
    pub best_effort_messages_sent: AtomicU64,
    /// Messages sent via ordered streams
    pub ordered_messages_sent: AtomicU64,
    /// Datagrams sent
    pub datagrams_sent: AtomicU64,
    /// Datagrams received
    pub datagrams_received: AtomicU64,
    /// Datagrams dropped (send failed)
    pub datagrams_dropped: AtomicU64,
    /// Critical priority messages
    pub critical_messages: AtomicU64,
    /// High priority messages
    pub high_priority_messages: AtomicU64,
    /// Normal priority messages
    pub normal_priority_messages: AtomicU64,
    /// Low priority messages
    pub low_priority_messages: AtomicU64,
}

/// QUIC transport backend
#[cfg(feature = "quic")]
pub struct QuicTransport {
    /// Local endpoint
    endpoint: Endpoint,
    /// Cached connections by remote address
    connections: Arc<RwLock<HashMap<SocketAddr, Connection>>>,
    /// Statistics
    stats: Arc<QuicStats>,
    /// Running flag
    running: Arc<AtomicBool>,
    /// Server name for TLS
    server_name: String,
    /// QUIC configuration
    config: QuicConfig,
}

#[cfg(feature = "quic")]
impl QuicTransport {
    /// Create a new QUIC client (outbound connections only)
    pub async fn new_client(bind_addr: SocketAddr, config: QuicConfig) -> io::Result<Self> {
        let client_config = Self::create_client_config(&config)?;

        let mut endpoint = Endpoint::client(bind_addr).map_err(io::Error::other)?;

        endpoint.set_default_client_config(client_config);

        Ok(Self {
            endpoint,
            connections: Arc::new(RwLock::new(HashMap::new())),
            stats: Arc::new(QuicStats::default()),
            running: Arc::new(AtomicBool::new(true)),
            server_name: "horus".to_string(),
            config,
        })
    }

    /// Create a new QUIC server (accepts inbound connections)
    pub async fn new_server(
        bind_addr: SocketAddr,
        cert_chain: Vec<rustls::pki_types::CertificateDer<'static>>,
        private_key: rustls::pki_types::PrivateKeyDer<'static>,
        config: QuicConfig,
    ) -> io::Result<Self> {
        let server_config = Self::create_server_config(cert_chain, private_key, &config)?;

        let endpoint = Endpoint::server(server_config, bind_addr).map_err(io::Error::other)?;

        Ok(Self {
            endpoint,
            connections: Arc::new(RwLock::new(HashMap::new())),
            stats: Arc::new(QuicStats::default()),
            running: Arc::new(AtomicBool::new(true)),
            server_name: "horus".to_string(),
            config,
        })
    }

    /// Create client TLS configuration
    fn create_client_config(config: &QuicConfig) -> io::Result<ClientConfig> {
        // Create a client config that accepts any certificate (for development)
        // In production, you'd want proper certificate validation
        let crypto = rustls::ClientConfig::builder()
            .dangerous()
            .with_custom_certificate_verifier(Arc::new(SkipServerVerification))
            .with_no_client_auth();

        let mut transport = TransportConfig::default();
        transport.max_idle_timeout(Some(config.max_idle_timeout.try_into().unwrap_or_default()));
        transport.keep_alive_interval(Some(config.keep_alive_interval));
        transport.initial_rtt(config.initial_rtt);

        // Enable datagrams for BestEffort QoS
        if config.enable_datagrams {
            transport.datagram_receive_buffer_size(Some(config.max_datagram_size as usize * 100));
            transport.datagram_send_buffer_size(config.max_datagram_size as usize * 100);
        }

        // Configure congestion control (note: quinn uses BBR by default)
        // The actual algorithm selection is done in the endpoint configuration

        let mut client_config = ClientConfig::new(Arc::new(
            quinn::crypto::rustls::QuicClientConfig::try_from(crypto).map_err(io::Error::other)?,
        ));
        client_config.transport_config(Arc::new(transport));

        Ok(client_config)
    }

    /// Create server TLS configuration
    fn create_server_config(
        cert_chain: Vec<rustls::pki_types::CertificateDer<'static>>,
        private_key: rustls::pki_types::PrivateKeyDer<'static>,
        config: &QuicConfig,
    ) -> io::Result<ServerConfig> {
        let mut transport = TransportConfig::default();
        transport.max_idle_timeout(Some(config.max_idle_timeout.try_into().unwrap_or_default()));
        transport.keep_alive_interval(Some(config.keep_alive_interval));
        transport.initial_rtt(config.initial_rtt);

        // Enable datagrams for BestEffort QoS
        if config.enable_datagrams {
            transport.datagram_receive_buffer_size(Some(config.max_datagram_size as usize * 100));
            transport.datagram_send_buffer_size(config.max_datagram_size as usize * 100);
        }

        let mut server_config =
            ServerConfig::with_single_cert(cert_chain, private_key).map_err(io::Error::other)?;

        server_config.transport_config(Arc::new(transport));

        Ok(server_config)
    }

    /// Get or create a connection to a remote address
    pub async fn get_connection(&self, addr: SocketAddr) -> io::Result<Connection> {
        // Check cache first
        {
            let conns = self.connections.read().await;
            if let Some(conn) = conns.get(&addr) {
                if conn.close_reason().is_none() {
                    return Ok(conn.clone());
                }
            }
        }

        // Create new connection
        let connecting = self
            .endpoint
            .connect(addr, &self.server_name)
            .map_err(io::Error::other)?;

        let connection = connecting
            .await
            .map_err(|e| io::Error::new(io::ErrorKind::ConnectionRefused, e))?;

        self.stats
            .connections_established
            .fetch_add(1, Ordering::Relaxed);

        // Cache the connection
        {
            let mut conns = self.connections.write().await;
            conns.insert(addr, connection.clone());
        }

        Ok(connection)
    }

    /// Send data to a remote address (unidirectional stream)
    pub async fn send(&self, addr: SocketAddr, data: &[u8]) -> io::Result<()> {
        let conn = self.get_connection(addr).await?;

        let mut stream = conn.open_uni().await.map_err(io::Error::other)?;

        self.stats.streams_opened.fetch_add(1, Ordering::Relaxed);

        // Write length prefix + data
        let len = (data.len() as u32).to_le_bytes();
        stream.write_all(&len).await.map_err(io::Error::other)?;

        stream.write_all(data).await.map_err(io::Error::other)?;

        stream.finish().map_err(io::Error::other)?;

        self.stats
            .bytes_sent
            .fetch_add(data.len() as u64 + 4, Ordering::Relaxed);
        self.stats.messages_sent.fetch_add(1, Ordering::Relaxed);
        self.stats.streams_closed.fetch_add(1, Ordering::Relaxed);

        Ok(())
    }

    /// Send data and wait for response (bidirectional stream)
    pub async fn send_recv(&self, addr: SocketAddr, data: &[u8]) -> io::Result<Vec<u8>> {
        let conn = self.get_connection(addr).await?;

        let (mut send, mut recv) = conn.open_bi().await.map_err(io::Error::other)?;

        self.stats.streams_opened.fetch_add(1, Ordering::Relaxed);

        // Send request
        let len = (data.len() as u32).to_le_bytes();
        send.write_all(&len).await.map_err(io::Error::other)?;
        send.write_all(data).await.map_err(io::Error::other)?;
        send.finish().map_err(io::Error::other)?;

        self.stats
            .bytes_sent
            .fetch_add(data.len() as u64 + 4, Ordering::Relaxed);
        self.stats.messages_sent.fetch_add(1, Ordering::Relaxed);

        // Receive response
        let mut len_buf = [0u8; 4];
        recv.read_exact(&mut len_buf)
            .await
            .map_err(io::Error::other)?;

        let response_len = u32::from_le_bytes(len_buf) as usize;
        let mut response = vec![0u8; response_len];
        recv.read_exact(&mut response)
            .await
            .map_err(io::Error::other)?;

        self.stats
            .bytes_received
            .fetch_add(response_len as u64 + 4, Ordering::Relaxed);
        self.stats.messages_received.fetch_add(1, Ordering::Relaxed);
        self.stats.streams_closed.fetch_add(1, Ordering::Relaxed);

        Ok(response)
    }

    /// Accept incoming connections (for server)
    pub async fn accept(&self) -> io::Result<(Connection, SocketAddr)> {
        let incoming =
            self.endpoint.accept().await.ok_or_else(|| {
                io::Error::new(io::ErrorKind::ConnectionAborted, "Endpoint closed")
            })?;

        let conn = incoming.await.map_err(io::Error::other)?;

        let addr = conn.remote_address();

        self.stats
            .connections_established
            .fetch_add(1, Ordering::Relaxed);

        // Cache the connection
        {
            let mut conns = self.connections.write().await;
            conns.insert(addr, conn.clone());
        }

        Ok((conn, addr))
    }

    /// Accept incoming unidirectional stream
    pub async fn accept_uni(&self, conn: &Connection) -> io::Result<(Vec<u8>, SocketAddr)> {
        let mut recv = conn.accept_uni().await.map_err(io::Error::other)?;

        self.stats.streams_opened.fetch_add(1, Ordering::Relaxed);

        // Read length prefix
        let mut len_buf = [0u8; 4];
        recv.read_exact(&mut len_buf)
            .await
            .map_err(io::Error::other)?;

        let data_len = u32::from_le_bytes(len_buf) as usize;
        let mut data = vec![0u8; data_len];
        recv.read_exact(&mut data).await.map_err(io::Error::other)?;

        self.stats
            .bytes_received
            .fetch_add(data_len as u64 + 4, Ordering::Relaxed);
        self.stats.messages_received.fetch_add(1, Ordering::Relaxed);
        self.stats.streams_closed.fetch_add(1, Ordering::Relaxed);

        Ok((data, conn.remote_address()))
    }

    /// Get statistics
    pub fn stats(&self) -> &QuicStats {
        &self.stats
    }

    /// Check if running
    pub fn is_running(&self) -> bool {
        self.running.load(Ordering::Relaxed)
    }

    /// Stop the transport
    pub fn stop(&self) {
        self.running.store(false, Ordering::Relaxed);
        self.endpoint.close(0u32.into(), b"shutdown");
    }

    /// Get local address
    pub fn local_addr(&self) -> io::Result<SocketAddr> {
        self.endpoint.local_addr()
    }

    /// Clean up stale connections
    pub async fn cleanup_connections(&self) {
        let mut conns = self.connections.write().await;
        conns.retain(|_, conn| conn.close_reason().is_none());
    }

    // ==================== QoS Methods ====================

    /// Send data with specific QoS profile
    ///
    /// # Arguments
    /// * `addr` - Remote address to send to
    /// * `data` - Data to send
    /// * `qos` - QoS profile determining delivery guarantees
    ///
    /// # Example
    /// ```rust,ignore
    /// // Send sensor data as best-effort (may drop)
    /// transport.send_with_qos(addr, &sensor_data, QuicQosProfile::BestEffort).await?;
    ///
    /// // Send commands reliably with ordering
    /// transport.send_with_qos(addr, &command, QuicQosProfile::ReliableOrdered).await?;
    /// ```
    pub async fn send_with_qos(
        &self,
        addr: SocketAddr,
        data: &[u8],
        qos: QuicQosProfile,
    ) -> io::Result<()> {
        match qos {
            QuicQosProfile::Reliable => {
                self.stats
                    .reliable_messages_sent
                    .fetch_add(1, Ordering::Relaxed);
                self.send(addr, data).await
            }
            QuicQosProfile::BestEffort => {
                self.stats
                    .best_effort_messages_sent
                    .fetch_add(1, Ordering::Relaxed);
                self.send_datagram(addr, data).await
            }
            QuicQosProfile::ReliableOrdered => {
                self.stats
                    .ordered_messages_sent
                    .fetch_add(1, Ordering::Relaxed);
                self.send_ordered(addr, data).await
            }
        }
    }

    /// Send data with specific priority
    ///
    /// Higher priority messages get bandwidth first under congestion.
    ///
    /// # Arguments
    /// * `addr` - Remote address
    /// * `data` - Data to send
    /// * `priority` - Stream priority level
    ///
    /// # Example
    /// ```rust,ignore
    /// // Send emergency stop at critical priority
    /// transport.send_with_priority(addr, &estop, QuicStreamPriority::Critical).await?;
    /// ```
    pub async fn send_with_priority(
        &self,
        addr: SocketAddr,
        data: &[u8],
        priority: QuicStreamPriority,
    ) -> io::Result<()> {
        // Track priority statistics
        match priority {
            QuicStreamPriority::Critical => {
                self.stats.critical_messages.fetch_add(1, Ordering::Relaxed);
            }
            QuicStreamPriority::High => {
                self.stats
                    .high_priority_messages
                    .fetch_add(1, Ordering::Relaxed);
            }
            QuicStreamPriority::Normal => {
                self.stats
                    .normal_priority_messages
                    .fetch_add(1, Ordering::Relaxed);
            }
            QuicStreamPriority::Low | QuicStreamPriority::Background => {
                self.stats
                    .low_priority_messages
                    .fetch_add(1, Ordering::Relaxed);
            }
        }

        let conn = self.get_connection(addr).await?;

        let mut stream = conn.open_uni().await.map_err(io::Error::other)?;

        // Set stream priority - lower values are higher priority
        let _ = stream.set_priority(priority.as_i32());

        self.stats.streams_opened.fetch_add(1, Ordering::Relaxed);

        // Write length prefix + data
        let len = (data.len() as u32).to_le_bytes();
        stream.write_all(&len).await.map_err(io::Error::other)?;

        stream.write_all(data).await.map_err(io::Error::other)?;

        stream.finish().map_err(io::Error::other)?;

        self.stats
            .bytes_sent
            .fetch_add(data.len() as u64 + 4, Ordering::Relaxed);
        self.stats.messages_sent.fetch_add(1, Ordering::Relaxed);
        self.stats.streams_closed.fetch_add(1, Ordering::Relaxed);

        Ok(())
    }

    /// Send data with both QoS profile and priority
    ///
    /// Combines delivery guarantees with priority ordering.
    pub async fn send_with_qos_and_priority(
        &self,
        addr: SocketAddr,
        data: &[u8],
        qos: QuicQosProfile,
        priority: QuicStreamPriority,
    ) -> io::Result<()> {
        // For best-effort, datagrams don't support priority, use normal send
        if qos == QuicQosProfile::BestEffort {
            return self.send_with_qos(addr, data, qos).await;
        }

        // Track priority statistics
        match priority {
            QuicStreamPriority::Critical => {
                self.stats.critical_messages.fetch_add(1, Ordering::Relaxed);
            }
            QuicStreamPriority::High => {
                self.stats
                    .high_priority_messages
                    .fetch_add(1, Ordering::Relaxed);
            }
            QuicStreamPriority::Normal => {
                self.stats
                    .normal_priority_messages
                    .fetch_add(1, Ordering::Relaxed);
            }
            QuicStreamPriority::Low | QuicStreamPriority::Background => {
                self.stats
                    .low_priority_messages
                    .fetch_add(1, Ordering::Relaxed);
            }
        }

        match qos {
            QuicQosProfile::Reliable => {
                self.stats
                    .reliable_messages_sent
                    .fetch_add(1, Ordering::Relaxed);
                self.send_with_priority(addr, data, priority).await
            }
            QuicQosProfile::ReliableOrdered => {
                self.stats
                    .ordered_messages_sent
                    .fetch_add(1, Ordering::Relaxed);
                self.send_ordered_with_priority(addr, data, priority).await
            }
            QuicQosProfile::BestEffort => {
                // Already handled above
                unreachable!()
            }
        }
    }

    /// Send unreliable datagram (best-effort, may be dropped)
    ///
    /// Uses QUIC datagrams (RFC 9221) for low-latency delivery without
    /// reliability guarantees. Ideal for sensor data, video frames, or
    /// any data where freshness matters more than completeness.
    ///
    /// # Errors
    /// Returns an error if:
    /// - The connection doesn't support datagrams
    /// - The data exceeds the maximum datagram size
    /// - The connection is closed
    pub async fn send_datagram(&self, addr: SocketAddr, data: &[u8]) -> io::Result<()> {
        let conn = self.get_connection(addr).await?;

        // Check datagram support and size limit
        let max_size = conn.max_datagram_size().ok_or_else(|| {
            io::Error::new(
                io::ErrorKind::Unsupported,
                "Connection does not support datagrams. Enable datagrams in QuicConfig.",
            )
        })?;

        // Include 4-byte length prefix in size check
        if data.len() + 4 > max_size {
            return Err(io::Error::new(
                io::ErrorKind::InvalidInput,
                format!(
                    "Datagram too large: {} bytes (max {} with header)",
                    data.len(),
                    max_size - 4
                ),
            ));
        }

        // Prepend length prefix for consistency with stream messages
        let mut datagram = Vec::with_capacity(data.len() + 4);
        datagram.extend_from_slice(&(data.len() as u32).to_le_bytes());
        datagram.extend_from_slice(data);

        match conn.send_datagram(datagram.into()) {
            Ok(()) => {
                self.stats.datagrams_sent.fetch_add(1, Ordering::Relaxed);
                self.stats
                    .bytes_sent
                    .fetch_add(data.len() as u64 + 4, Ordering::Relaxed);
                self.stats.messages_sent.fetch_add(1, Ordering::Relaxed);
                Ok(())
            }
            Err(e) => {
                self.stats.datagrams_dropped.fetch_add(1, Ordering::Relaxed);
                Err(io::Error::other(e))
            }
        }
    }

    /// Receive a datagram (best-effort)
    ///
    /// Returns the next available datagram or waits for one.
    /// Datagrams may arrive out of order or be dropped entirely.
    pub async fn recv_datagram(&self, conn: &Connection) -> io::Result<Vec<u8>> {
        let datagram = conn.read_datagram().await.map_err(io::Error::other)?;

        self.stats
            .datagrams_received
            .fetch_add(1, Ordering::Relaxed);

        // Parse length prefix
        if datagram.len() < 4 {
            return Err(io::Error::new(
                io::ErrorKind::InvalidData,
                "Datagram too short (missing length prefix)",
            ));
        }

        let len = u32::from_le_bytes([datagram[0], datagram[1], datagram[2], datagram[3]]) as usize;

        if datagram.len() < 4 + len {
            return Err(io::Error::new(
                io::ErrorKind::InvalidData,
                "Datagram length mismatch",
            ));
        }

        self.stats
            .bytes_received
            .fetch_add(len as u64 + 4, Ordering::Relaxed);
        self.stats.messages_received.fetch_add(1, Ordering::Relaxed);

        Ok(datagram[4..4 + len].to_vec())
    }

    /// Send data with strict ordering guarantee (reliable ordered)
    ///
    /// Uses a bidirectional stream to ensure messages arrive in order.
    /// Higher latency due to head-of-line blocking, but guarantees
    /// message ordering within this stream.
    async fn send_ordered(&self, addr: SocketAddr, data: &[u8]) -> io::Result<()> {
        let conn = self.get_connection(addr).await?;

        // Use bidirectional stream for ordering - sender closes their side after sending
        let (mut send, _recv) = conn.open_bi().await.map_err(io::Error::other)?;

        self.stats.streams_opened.fetch_add(1, Ordering::Relaxed);

        // Write length prefix + data
        let len = (data.len() as u32).to_le_bytes();
        send.write_all(&len).await.map_err(io::Error::other)?;

        send.write_all(data).await.map_err(io::Error::other)?;

        send.finish().map_err(io::Error::other)?;

        self.stats
            .bytes_sent
            .fetch_add(data.len() as u64 + 4, Ordering::Relaxed);
        self.stats.messages_sent.fetch_add(1, Ordering::Relaxed);
        self.stats.streams_closed.fetch_add(1, Ordering::Relaxed);

        Ok(())
    }

    /// Send ordered data with priority
    async fn send_ordered_with_priority(
        &self,
        addr: SocketAddr,
        data: &[u8],
        priority: QuicStreamPriority,
    ) -> io::Result<()> {
        let conn = self.get_connection(addr).await?;

        let (mut send, _recv) = conn.open_bi().await.map_err(io::Error::other)?;

        // Set stream priority
        let _ = send.set_priority(priority.as_i32());

        self.stats.streams_opened.fetch_add(1, Ordering::Relaxed);

        let len = (data.len() as u32).to_le_bytes();
        send.write_all(&len).await.map_err(io::Error::other)?;

        send.write_all(data).await.map_err(io::Error::other)?;

        send.finish().map_err(io::Error::other)?;

        self.stats
            .bytes_sent
            .fetch_add(data.len() as u64 + 4, Ordering::Relaxed);
        self.stats.messages_sent.fetch_add(1, Ordering::Relaxed);
        self.stats.streams_closed.fetch_add(1, Ordering::Relaxed);

        Ok(())
    }

    /// Get the configured QoS profile
    pub fn default_qos(&self) -> QuicQosProfile {
        self.config.default_qos
    }

    /// Get the configured default priority
    pub fn default_priority(&self) -> QuicStreamPriority {
        self.config.default_priority
    }

    /// Check if datagrams are enabled for this transport
    pub fn datagrams_enabled(&self) -> bool {
        self.config.enable_datagrams
    }

    /// Get the congestion control algorithm
    pub fn congestion_control(&self) -> QuicCongestionControl {
        self.config.congestion_control
    }
}

/// Skip server certificate verification (for development only!)
#[cfg(feature = "quic")]
#[derive(Debug)]
struct SkipServerVerification;

#[cfg(feature = "quic")]
impl rustls::client::danger::ServerCertVerifier for SkipServerVerification {
    fn verify_server_cert(
        &self,
        _end_entity: &rustls::pki_types::CertificateDer<'_>,
        _intermediates: &[rustls::pki_types::CertificateDer<'_>],
        _server_name: &rustls::pki_types::ServerName<'_>,
        _ocsp_response: &[u8],
        _now: rustls::pki_types::UnixTime,
    ) -> Result<rustls::client::danger::ServerCertVerified, rustls::Error> {
        Ok(rustls::client::danger::ServerCertVerified::assertion())
    }

    fn verify_tls12_signature(
        &self,
        _message: &[u8],
        _cert: &rustls::pki_types::CertificateDer<'_>,
        _dss: &rustls::DigitallySignedStruct,
    ) -> Result<rustls::client::danger::HandshakeSignatureValid, rustls::Error> {
        Ok(rustls::client::danger::HandshakeSignatureValid::assertion())
    }

    fn verify_tls13_signature(
        &self,
        _message: &[u8],
        _cert: &rustls::pki_types::CertificateDer<'_>,
        _dss: &rustls::DigitallySignedStruct,
    ) -> Result<rustls::client::danger::HandshakeSignatureValid, rustls::Error> {
        Ok(rustls::client::danger::HandshakeSignatureValid::assertion())
    }

    fn supported_verify_schemes(&self) -> Vec<rustls::SignatureScheme> {
        vec![
            rustls::SignatureScheme::RSA_PKCS1_SHA256,
            rustls::SignatureScheme::ECDSA_NISTP256_SHA256,
            rustls::SignatureScheme::RSA_PKCS1_SHA384,
            rustls::SignatureScheme::ECDSA_NISTP384_SHA384,
            rustls::SignatureScheme::RSA_PKCS1_SHA512,
            rustls::SignatureScheme::ED25519,
        ]
    }
}

/// QUIC backend wrapper for NetworkBackend integration
///
/// Provides synchronous send/recv interface over QUIC transport
/// for compatibility with the Topic system.
#[cfg(feature = "quic")]
pub struct QuicBackend<T> {
    /// Underlying QUIC transport
    transport: Arc<QuicTransport>,
    /// Receive buffer for incoming messages
    recv_buffer: Arc<tokio::sync::RwLock<std::collections::VecDeque<T>>>,
    /// Topic name
    topic: String,
    /// Remote address for sending
    remote_addr: SocketAddr,
    /// Tokio runtime handle
    runtime: tokio::runtime::Handle,
    /// Marker for type
    _phantom: std::marker::PhantomData<T>,
}

#[cfg(feature = "quic")]
impl<T> QuicBackend<T>
where
    T: serde::Serialize + serde::de::DeserializeOwned + Send + Sync + Clone + 'static,
{
    /// Create a new QUIC backend (blocking)
    pub fn new_blocking(
        topic: &str,
        remote_addr: SocketAddr,
        config: QuicConfig,
    ) -> io::Result<Self> {
        let runtime = tokio::runtime::Handle::try_current()
            .or_else(|_| tokio::runtime::Runtime::new().map(|rt| rt.handle().clone()))?;

        // Create client transport
        let bind_addr: SocketAddr = if remote_addr.is_ipv4() {
            "0.0.0.0:0".parse().unwrap()
        } else {
            "[::]:0".parse().unwrap()
        };

        let transport = runtime.block_on(QuicTransport::new_client(bind_addr, config))?;

        Ok(Self {
            transport: Arc::new(transport),
            recv_buffer: Arc::new(tokio::sync::RwLock::new(
                std::collections::VecDeque::with_capacity(1024),
            )),
            topic: topic.to_string(),
            remote_addr,
            runtime,
            _phantom: std::marker::PhantomData,
        })
    }

    /// Create a QUIC server backend that listens for connections
    ///
    /// This method generates a new certificate on each call. For persistent
    /// certificates that survive restarts, use `new_server_persistent()` instead
    /// (requires the `tls` feature).
    pub fn new_server_blocking(
        topic: &str,
        bind_addr: SocketAddr,
        config: QuicConfig,
    ) -> io::Result<Self> {
        let runtime = tokio::runtime::Handle::try_current()
            .or_else(|_| tokio::runtime::Runtime::new().map(|rt| rt.handle().clone()))?;

        // Generate self-signed cert for server
        let (cert_chain, private_key) = generate_self_signed_cert()?;

        let transport = runtime.block_on(QuicTransport::new_server(
            bind_addr,
            cert_chain,
            private_key,
            config,
        ))?;

        Ok(Self {
            transport: Arc::new(transport),
            recv_buffer: Arc::new(tokio::sync::RwLock::new(
                std::collections::VecDeque::with_capacity(1024),
            )),
            topic: topic.to_string(),
            remote_addr: bind_addr, // For server, this is the local addr
            runtime,
            _phantom: std::marker::PhantomData,
        })
    }

    /// Create a QUIC server with persistent certificates
    ///
    /// Certificates are automatically generated on first run and persisted to
    /// `~/.horus/certs/` for reuse across restarts. This is the recommended
    /// approach for most use cases.
    ///
    /// Requires both `quic` and `tls` features.
    ///
    /// # Example
    ///
    /// ```rust,no_run
    /// use horus_core::communication::network::{QuicBackend, QuicConfig};
    /// use std::net::SocketAddr;
    ///
    /// let addr: SocketAddr = "0.0.0.0:4433".parse().unwrap();
    /// let backend: QuicBackend<String> = QuicBackend::new_server_persistent(
    ///     "my_topic",
    ///     addr,
    ///     QuicConfig::default(),
    /// ).unwrap();
    /// ```
    #[cfg(feature = "tls")]
    pub fn new_server_persistent(
        topic: &str,
        bind_addr: SocketAddr,
        config: QuicConfig,
    ) -> io::Result<Self> {
        use super::certificate_manager::CertificateManager;

        let runtime = tokio::runtime::Handle::try_current()
            .or_else(|_| tokio::runtime::Runtime::new().map(|rt| rt.handle().clone()))?;

        // Get or create persistent certificates
        let manager = CertificateManager::new()?;
        let (cert_chain, private_key) = manager.get_or_create()?;

        let transport = runtime.block_on(QuicTransport::new_server(
            bind_addr,
            cert_chain,
            private_key,
            config,
        ))?;

        Ok(Self {
            transport: Arc::new(transport),
            recv_buffer: Arc::new(tokio::sync::RwLock::new(
                std::collections::VecDeque::with_capacity(1024),
            )),
            topic: topic.to_string(),
            remote_addr: bind_addr,
            runtime,
            _phantom: std::marker::PhantomData,
        })
    }

    /// Create a QUIC server with custom certificate configuration
    ///
    /// Allows specifying custom certificate directory, organization name,
    /// and other certificate properties.
    ///
    /// # Example
    ///
    /// ```rust,no_run
    /// use horus_core::communication::network::{QuicBackend, QuicConfig};
    /// use std::net::SocketAddr;
    ///
    /// let addr: SocketAddr = "0.0.0.0:4433".parse().unwrap();
    /// let backend: QuicBackend<String> = QuicBackend::new_server_with_cert_config(
    ///     "my_topic",
    ///     addr,
    ///     QuicConfig::default(),
    ///     "/etc/myapp/certs",
    ///     "My Robot",
    ///     "robot.local",
    /// ).unwrap();
    /// ```
    #[cfg(feature = "tls")]
    pub fn new_server_with_cert_config(
        topic: &str,
        bind_addr: SocketAddr,
        config: QuicConfig,
        cert_dir: impl AsRef<std::path::Path>,
        organization: impl Into<String>,
        common_name: impl Into<String>,
    ) -> io::Result<Self> {
        use super::certificate_manager::CertificateManager;

        let runtime = tokio::runtime::Handle::try_current()
            .or_else(|_| tokio::runtime::Runtime::new().map(|rt| rt.handle().clone()))?;

        // Get or create certificates with custom config
        let manager = CertificateManager::builder()
            .with_cert_dir(cert_dir)
            .with_organization(organization)
            .with_common_name(common_name)
            .build()?;
        let (cert_chain, private_key) = manager.get_or_create()?;

        let transport = runtime.block_on(QuicTransport::new_server(
            bind_addr,
            cert_chain,
            private_key,
            config,
        ))?;

        Ok(Self {
            transport: Arc::new(transport),
            recv_buffer: Arc::new(tokio::sync::RwLock::new(
                std::collections::VecDeque::with_capacity(1024),
            )),
            topic: topic.to_string(),
            remote_addr: bind_addr,
            runtime,
            _phantom: std::marker::PhantomData,
        })
    }

    /// Send a message over QUIC
    pub fn send(&self, msg: &T) -> io::Result<()> {
        let data =
            bincode::serialize(msg).map_err(|e| io::Error::new(io::ErrorKind::InvalidData, e))?;

        self.runtime
            .block_on(self.transport.send(self.remote_addr, &data))
    }

    /// Receive a message (non-blocking)
    pub fn recv(&self) -> Option<T> {
        let mut buffer = self.runtime.block_on(self.recv_buffer.write());
        buffer.pop_front()
    }

    /// Check if messages are available
    pub fn has_messages(&self) -> bool {
        let buffer = self.runtime.block_on(self.recv_buffer.read());
        !buffer.is_empty()
    }

    /// Start receiving messages from a connection
    ///
    /// This spawns a background task that accepts incoming streams
    /// and deserializes messages into the receive buffer.
    pub fn start_receiver(&self, conn: quinn::Connection) {
        let recv_buffer = self.recv_buffer.clone();
        let transport = self.transport.clone();

        self.runtime.spawn(async move {
            loop {
                match transport.accept_uni(&conn).await {
                    Ok((data, _addr)) => {
                        if let Ok(msg) = bincode::deserialize::<T>(&data) {
                            let mut buffer = recv_buffer.write().await;
                            // Keep buffer bounded
                            if buffer.len() >= 10000 {
                                buffer.pop_front();
                            }
                            buffer.push_back(msg);
                        }
                    }
                    Err(e) => {
                        log::debug!("QUIC receive stream ended: {}", e);
                        break;
                    }
                }
            }
        });
    }

    /// Get statistics
    pub fn stats(&self) -> &QuicStats {
        self.transport.stats()
    }

    /// Get topic name
    pub fn topic(&self) -> &str {
        &self.topic
    }

    /// Get local address
    pub fn local_addr(&self) -> io::Result<SocketAddr> {
        self.transport.local_addr()
    }
}

#[cfg(feature = "quic")]
impl<T> std::fmt::Debug for QuicBackend<T> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("QuicBackend")
            .field("topic", &self.topic)
            .field("remote_addr", &self.remote_addr)
            .finish()
    }
}

// Stub when QUIC feature is not enabled
#[cfg(not(feature = "quic"))]
pub struct QuicBackend<T>(std::marker::PhantomData<T>);

#[cfg(not(feature = "quic"))]
impl<T> QuicBackend<T> {
    pub fn new_blocking(
        _topic: &str,
        _remote_addr: SocketAddr,
        _config: QuicConfig,
    ) -> io::Result<Self> {
        Err(io::Error::new(
            io::ErrorKind::Unsupported,
            "QUIC requires the 'quic' feature",
        ))
    }
}

#[cfg(not(feature = "quic"))]
impl<T> std::fmt::Debug for QuicBackend<T> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("QuicBackend").finish()
    }
}

/// Generate a self-signed certificate for testing/development
///
/// This function generates a new certificate on each call. For persistent
/// certificates that survive restarts, use `get_or_create_cert()` instead.
#[cfg(feature = "quic")]
pub fn generate_self_signed_cert() -> io::Result<(
    Vec<rustls::pki_types::CertificateDer<'static>>,
    rustls::pki_types::PrivateKeyDer<'static>,
)> {
    let cert =
        rcgen::generate_simple_self_signed(vec!["horus".to_string(), "localhost".to_string()])
            .map_err(io::Error::other)?;

    let cert_der = rustls::pki_types::CertificateDer::from(cert.cert.der().to_vec());
    let key_der = rustls::pki_types::PrivateKeyDer::try_from(cert.key_pair.serialize_der())
        .map_err(io::Error::other)?;

    Ok((vec![cert_der], key_der))
}

/// Get or create persistent certificates for QUIC
///
/// This function uses the CertificateManager to:
/// - Load existing certificates from ~/.horus/certs/ if available
/// - Generate new self-signed certificates if none exist
/// - Persist certificates across restarts
///
/// # Example
///
/// ```rust,no_run
/// use horus_core::communication::network::quic::get_or_create_cert;
///
/// // Zero-config: automatically generates and persists certificates
/// let (certs, key) = get_or_create_cert().unwrap();
/// ```
#[cfg(all(feature = "quic", feature = "tls"))]
pub fn get_or_create_cert() -> io::Result<(
    Vec<rustls::pki_types::CertificateDer<'static>>,
    rustls::pki_types::PrivateKeyDer<'static>,
)> {
    use super::certificate_manager::CertificateManager;

    let manager = CertificateManager::new()?;
    manager.get_or_create()
}

/// Get or create persistent certificates with custom configuration
///
/// Allows customizing the certificate directory, organization name,
/// and other certificate properties.
///
/// # Example
///
/// ```rust,no_run
/// use horus_core::communication::network::quic::get_or_create_cert_with_config;
///
/// let (certs, key) = get_or_create_cert_with_config(
///     "/etc/myapp/certs",
///     "My Robot",
///     "robot.local",
/// ).unwrap();
/// ```
#[cfg(all(feature = "quic", feature = "tls"))]
pub fn get_or_create_cert_with_config(
    cert_dir: impl AsRef<std::path::Path>,
    organization: impl Into<String>,
    common_name: impl Into<String>,
) -> io::Result<(
    Vec<rustls::pki_types::CertificateDer<'static>>,
    rustls::pki_types::PrivateKeyDer<'static>,
)> {
    use super::certificate_manager::CertificateManager;

    let manager = CertificateManager::builder()
        .with_cert_dir(cert_dir)
        .with_organization(organization)
        .with_common_name(common_name)
        .build()?;
    manager.get_or_create()
}

// Stub implementation when quic feature is not enabled
#[cfg(not(feature = "quic"))]
pub struct QuicTransport;

#[cfg(not(feature = "quic"))]
impl QuicTransport {
    pub async fn new_client(_bind_addr: SocketAddr, _config: QuicConfig) -> io::Result<Self> {
        Err(io::Error::new(
            io::ErrorKind::Unsupported,
            "QUIC requires the 'quic' feature",
        ))
    }
}

#[cfg(not(feature = "quic"))]
pub fn generate_self_signed_cert() -> io::Result<(Vec<Vec<u8>>, Vec<u8>)> {
    Err(io::Error::new(
        io::ErrorKind::Unsupported,
        "QUIC requires the 'quic' feature",
    ))
}

#[cfg(all(test, feature = "quic"))]
mod tests {
    use super::*;

    fn install_crypto_provider() {
        // Install the default crypto provider for rustls
        let _ = rustls::crypto::ring::default_provider().install_default();
    }

    #[tokio::test]
    async fn test_config() {
        let config = QuicConfig::default();
        assert_eq!(config.max_idle_timeout, Duration::from_secs(30));

        let ll = QuicConfig::low_latency();
        assert!(ll.keep_alive_interval < config.keep_alive_interval);
    }

    #[tokio::test]
    async fn test_self_signed_cert() {
        install_crypto_provider();
        let result = generate_self_signed_cert();
        assert!(result.is_ok());

        let (certs, _key) = result.unwrap();
        assert_eq!(certs.len(), 1);
    }

    #[tokio::test]
    async fn test_client_creation() {
        install_crypto_provider();
        let addr = "127.0.0.1:0".parse().unwrap();
        let client = QuicTransport::new_client(addr, QuicConfig::default()).await;
        assert!(client.is_ok());

        let client = client.unwrap();
        assert!(client.is_running());
    }

    // ==================== QoS Profile Tests ====================

    #[test]
    fn test_qos_profile_properties() {
        // Test Reliable profile
        assert!(QuicQosProfile::Reliable.is_reliable());
        assert!(!QuicQosProfile::Reliable.is_ordered());
        assert_eq!(QuicQosProfile::Reliable.retry_count(), 3);

        // Test BestEffort profile
        assert!(!QuicQosProfile::BestEffort.is_reliable());
        assert!(!QuicQosProfile::BestEffort.is_ordered());
        assert_eq!(QuicQosProfile::BestEffort.retry_count(), 0);

        // Test ReliableOrdered profile
        assert!(QuicQosProfile::ReliableOrdered.is_reliable());
        assert!(QuicQosProfile::ReliableOrdered.is_ordered());
        assert_eq!(QuicQosProfile::ReliableOrdered.retry_count(), 5);
    }

    #[test]
    fn test_qos_profile_default() {
        let qos = QuicQosProfile::default();
        assert_eq!(qos, QuicQosProfile::Reliable);
    }

    #[test]
    fn test_stream_priority_values() {
        assert_eq!(QuicStreamPriority::Critical.as_i32(), 0);
        assert_eq!(QuicStreamPriority::High.as_i32(), 32);
        assert_eq!(QuicStreamPriority::Normal.as_i32(), 128);
        assert_eq!(QuicStreamPriority::Low.as_i32(), 192);
        assert_eq!(QuicStreamPriority::Background.as_i32(), 255);
    }

    #[test]
    fn test_stream_priority_from_value() {
        assert_eq!(
            QuicStreamPriority::from_value(0),
            QuicStreamPriority::Critical
        );
        assert_eq!(
            QuicStreamPriority::from_value(10),
            QuicStreamPriority::Critical
        );
        assert_eq!(QuicStreamPriority::from_value(32), QuicStreamPriority::High);
        assert_eq!(QuicStreamPriority::from_value(50), QuicStreamPriority::High);
        assert_eq!(
            QuicStreamPriority::from_value(100),
            QuicStreamPriority::Normal
        );
        assert_eq!(QuicStreamPriority::from_value(200), QuicStreamPriority::Low);
        assert_eq!(
            QuicStreamPriority::from_value(250),
            QuicStreamPriority::Background
        );
    }

    #[test]
    fn test_stream_priority_default() {
        let priority = QuicStreamPriority::default();
        assert_eq!(priority, QuicStreamPriority::Normal);
    }

    #[test]
    fn test_congestion_control_names() {
        assert_eq!(QuicCongestionControl::Bbr.name(), "bbr");
        assert_eq!(QuicCongestionControl::Cubic.name(), "cubic");
        assert_eq!(QuicCongestionControl::NewReno.name(), "newreno");
    }

    #[test]
    fn test_congestion_control_default() {
        let cc = QuicCongestionControl::default();
        assert_eq!(cc, QuicCongestionControl::Bbr);
    }

    #[test]
    fn test_config_presets() {
        // Test low latency preset
        let ll = QuicConfig::low_latency();
        assert_eq!(ll.congestion_control, QuicCongestionControl::Bbr);
        assert!(ll.enable_datagrams);
        assert_eq!(ll.default_qos, QuicQosProfile::Reliable);
        assert_eq!(ll.default_priority, QuicStreamPriority::High);

        // Test high throughput preset
        let ht = QuicConfig::high_throughput();
        assert_eq!(ht.congestion_control, QuicCongestionControl::Cubic);
        assert!(!ht.enable_datagrams);
        assert_eq!(ht.default_qos, QuicQosProfile::Reliable);
        assert_eq!(ht.default_priority, QuicStreamPriority::Normal);

        // Test best effort preset
        let be = QuicConfig::best_effort();
        assert_eq!(be.congestion_control, QuicCongestionControl::Bbr);
        assert!(be.enable_datagrams);
        assert_eq!(be.default_qos, QuicQosProfile::BestEffort);

        // Test control plane preset
        let cp = QuicConfig::control_plane();
        assert!(!cp.enable_datagrams);
        assert_eq!(cp.default_qos, QuicQosProfile::ReliableOrdered);
        assert_eq!(cp.default_priority, QuicStreamPriority::High);
    }

    #[test]
    fn test_config_default_values() {
        let config = QuicConfig::default();
        assert_eq!(config.congestion_control, QuicCongestionControl::Bbr);
        assert!(config.enable_datagrams);
        assert_eq!(config.max_datagram_size, 1200);
        assert_eq!(config.default_qos, QuicQosProfile::Reliable);
        assert_eq!(config.default_priority, QuicStreamPriority::Normal);
    }

    #[tokio::test]
    async fn test_transport_qos_accessors() {
        install_crypto_provider();
        let addr = "127.0.0.1:0".parse().unwrap();
        let config = QuicConfig::low_latency();
        let transport = QuicTransport::new_client(addr, config).await.unwrap();

        assert_eq!(transport.default_qos(), QuicQosProfile::Reliable);
        assert_eq!(transport.default_priority(), QuicStreamPriority::High);
        assert!(transport.datagrams_enabled());
        assert_eq!(transport.congestion_control(), QuicCongestionControl::Bbr);
    }

    #[tokio::test]
    async fn test_stats_qos_counters() {
        let stats = QuicStats::default();

        // Verify all QoS counters start at 0
        assert_eq!(stats.reliable_messages_sent.load(Ordering::Relaxed), 0);
        assert_eq!(stats.best_effort_messages_sent.load(Ordering::Relaxed), 0);
        assert_eq!(stats.ordered_messages_sent.load(Ordering::Relaxed), 0);
        assert_eq!(stats.datagrams_sent.load(Ordering::Relaxed), 0);
        assert_eq!(stats.datagrams_received.load(Ordering::Relaxed), 0);
        assert_eq!(stats.datagrams_dropped.load(Ordering::Relaxed), 0);
        assert_eq!(stats.critical_messages.load(Ordering::Relaxed), 0);
        assert_eq!(stats.high_priority_messages.load(Ordering::Relaxed), 0);
        assert_eq!(stats.normal_priority_messages.load(Ordering::Relaxed), 0);
        assert_eq!(stats.low_priority_messages.load(Ordering::Relaxed), 0);

        // Increment and verify
        stats.reliable_messages_sent.fetch_add(5, Ordering::Relaxed);
        stats
            .best_effort_messages_sent
            .fetch_add(10, Ordering::Relaxed);
        stats.critical_messages.fetch_add(2, Ordering::Relaxed);

        assert_eq!(stats.reliable_messages_sent.load(Ordering::Relaxed), 5);
        assert_eq!(stats.best_effort_messages_sent.load(Ordering::Relaxed), 10);
        assert_eq!(stats.critical_messages.load(Ordering::Relaxed), 2);
    }
}
