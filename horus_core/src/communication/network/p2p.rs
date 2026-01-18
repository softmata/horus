//! Peer-to-Peer connectivity with NAT traversal
//!
//! This module provides P2P connections between HORUS nodes across NATs using:
//! - Direct connection (same LAN, fastest)
//! - STUN-based hole punching (different NATs, no server needed after setup)
//! - TURN relay (symmetric NAT fallback, requires relay server)
//!
//! # Endpoint Format
//!
//! ```text
//! topic@p2p:peer-id              - Connect to peer by ID
//! topic@p2p:peer-id/direct       - Force direct only (same LAN)
//! topic@p2p:peer-id/stun         - Use STUN server for hole punching
//! topic@p2p:peer-id/turn         - Use TURN relay (always works)
//! ```
//!
//! # Peer IDs
//!
//! Peer IDs are human-readable short codes derived from public keys:
//! - Format: `xxxx-xxxx-xxxx` (12 characters, alphanumeric, lowercase)
//! - Example: `a3f7-k2m9-p4n8`
//! - Stable across restarts (based on persistent key pair)
//!
//! # Connection Flow
//!
//! 1. Both peers register with signaling server (or mDNS for LAN)
//! 2. Initiator requests connection to target peer ID
//! 3. Signaling server facilitates ICE candidate exchange
//! 4. Connection established via best available method:
//!    - Direct: Both on same LAN
//!    - STUN: Different NATs, hole punch successful
//!    - TURN: Fallback when hole punching fails

use std::collections::hash_map::DefaultHasher;
use std::hash::{Hash, Hasher};
use std::net::SocketAddr;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};
use tokio::net::UdpSocket;
use tokio::time::timeout;

use super::turn::{TurnClient, TurnConfig, TurnTransport};

/// Peer identity based on public key
#[derive(Debug, Clone, PartialEq, Eq, Hash)]
pub struct PeerId {
    /// Short human-readable ID (e.g., "a3f7-k2m9-p4n8")
    pub short_id: String,
    /// Full public key bytes (32 bytes for Ed25519)
    pub public_key: Vec<u8>,
}

impl PeerId {
    /// Create PeerId from public key bytes
    pub fn from_public_key(public_key: &[u8]) -> Self {
        let short_id = Self::derive_short_id(public_key);
        Self {
            short_id,
            public_key: public_key.to_vec(),
        }
    }

    /// Create PeerId from short ID (for lookups, no public key verification)
    pub fn from_short_id(short_id: &str) -> Self {
        Self {
            short_id: short_id.to_lowercase(),
            public_key: Vec::new(),
        }
    }

    /// Derive human-readable short ID from public key
    /// Format: xxxx-xxxx-xxxx (12 chars alphanumeric)
    fn derive_short_id(public_key: &[u8]) -> String {
        // Use first 9 bytes of public key hash to generate 12-char ID
        use std::collections::hash_map::DefaultHasher;
        use std::hash::{Hash, Hasher};

        let mut hasher = DefaultHasher::new();
        public_key.hash(&mut hasher);
        let hash = hasher.finish();

        // Convert to base36 (alphanumeric lowercase)
        let chars: Vec<char> = "0123456789abcdefghijklmnopqrstuvwxyz".chars().collect();
        let mut result = String::with_capacity(14);
        let mut remaining = hash;

        for i in 0..12 {
            if i > 0 && i % 4 == 0 {
                result.push('-');
            }
            result.push(chars[(remaining % 36) as usize]);
            remaining /= 36;
        }

        result
    }

    /// Validate short ID format
    pub fn is_valid_short_id(id: &str) -> bool {
        // Format: xxxx-xxxx-xxxx (14 chars with dashes, 12 alphanumeric)
        if id.len() != 14 {
            return false;
        }

        let parts: Vec<&str> = id.split('-').collect();
        if parts.len() != 3 {
            return false;
        }

        parts
            .iter()
            .all(|part| part.len() == 4 && part.chars().all(|c| c.is_ascii_alphanumeric()))
    }
}

impl std::fmt::Display for PeerId {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}", self.short_id)
    }
}

/// P2P connection strategy
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum P2pStrategy {
    /// Try all methods in order: direct -> STUN -> TURN
    Auto,
    /// Direct connection only (same LAN)
    Direct,
    /// Use STUN server for NAT traversal
    Stun,
    /// Use TURN relay (always works but higher latency)
    Turn,
}

impl Default for P2pStrategy {
    fn default() -> Self {
        Self::Auto
    }
}

impl std::str::FromStr for P2pStrategy {
    type Err = String;

    fn from_str(s: &str) -> Result<Self, Self::Err> {
        match s.to_lowercase().as_str() {
            "auto" | "" => Ok(Self::Auto),
            "direct" => Ok(Self::Direct),
            "stun" => Ok(Self::Stun),
            "turn" => Ok(Self::Turn),
            _ => Err(format!("Unknown P2P strategy: {}", s)),
        }
    }
}

/// P2P connection state
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum P2pConnectionState {
    /// Not connected
    Disconnected,
    /// Discovering peer via signaling
    Discovering,
    /// Attempting direct connection
    TryingDirect,
    /// Attempting STUN hole punch
    TryingStun,
    /// Attempting TURN relay
    TryingTurn,
    /// Connected via direct path
    ConnectedDirect,
    /// Connected via STUN hole punch
    ConnectedStun,
    /// Connected via TURN relay
    ConnectedTurn,
    /// Connection failed
    Failed,
}

impl P2pConnectionState {
    /// Check if currently connected
    pub fn is_connected(&self) -> bool {
        matches!(
            self,
            Self::ConnectedDirect | Self::ConnectedStun | Self::ConnectedTurn
        )
    }

    /// Check if connection is in progress
    pub fn is_connecting(&self) -> bool {
        matches!(
            self,
            Self::Discovering | Self::TryingDirect | Self::TryingStun | Self::TryingTurn
        )
    }

    /// Get human-readable status message
    pub fn status_message(&self) -> &'static str {
        match self {
            Self::Disconnected => "Disconnected",
            Self::Discovering => "Discovering peer...",
            Self::TryingDirect => "Trying direct connection...",
            Self::TryingStun => "Attempting NAT hole punch...",
            Self::TryingTurn => "Connecting via relay...",
            Self::ConnectedDirect => "Connected (direct)",
            Self::ConnectedStun => "Connected (hole-punched)",
            Self::ConnectedTurn => "Connected (relayed)",
            Self::Failed => "Connection failed",
        }
    }
}

/// P2P configuration
#[derive(Debug, Clone)]
pub struct P2pConfig {
    /// STUN server addresses (e.g., "stun.l.google.com:19302")
    pub stun_servers: Vec<String>,
    /// TURN server addresses with credentials
    pub turn_servers: Vec<TurnServer>,
    /// Signaling server URL (WebSocket)
    pub signaling_url: Option<String>,
    /// Connection timeout
    pub connection_timeout: Duration,
    /// Direct connection timeout (shorter, for LAN detection)
    pub direct_timeout: Duration,
    /// STUN timeout
    pub stun_timeout: Duration,
    /// Keep-alive interval
    pub keepalive_interval: Duration,
    /// Enable mDNS for LAN peer discovery
    pub enable_mdns: bool,
}

impl Default for P2pConfig {
    fn default() -> Self {
        Self {
            stun_servers: vec![
                "stun.l.google.com:19302".to_string(),
                "stun1.l.google.com:19302".to_string(),
            ],
            turn_servers: Vec::new(),
            signaling_url: None,
            connection_timeout: Duration::from_secs(30),
            direct_timeout: Duration::from_millis(500),
            stun_timeout: Duration::from_secs(5),
            keepalive_interval: Duration::from_secs(10),
            enable_mdns: true,
        }
    }
}

/// TURN server configuration
#[derive(Debug, Clone)]
pub struct TurnServer {
    /// Server address (host:port)
    pub address: String,
    /// Username for authentication
    pub username: String,
    /// Password/credential
    pub credential: String,
}

/// ICE candidate for NAT traversal
#[derive(Debug, Clone)]
pub struct IceCandidate {
    /// Candidate type
    pub candidate_type: IceCandidateType,
    /// Transport protocol
    pub protocol: IceProtocol,
    /// IP address
    pub address: SocketAddr,
    /// Priority (higher = preferred)
    pub priority: u32,
    /// Foundation (for candidate pairing)
    pub foundation: String,
    /// Component ID (1 = RTP, 2 = RTCP in standard ICE)
    pub component: u8,
}

/// ICE candidate types
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum IceCandidateType {
    /// Host candidate (local address)
    Host,
    /// Server reflexive (STUN-discovered public address)
    ServerReflexive,
    /// Peer reflexive (discovered during connectivity check)
    PeerReflexive,
    /// Relay candidate (TURN)
    Relay,
}

/// ICE transport protocol
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum IceProtocol {
    Udp,
    Tcp,
}

/// P2P connection statistics
#[derive(Debug, Default)]
pub struct P2pStats {
    /// Total messages sent
    pub messages_sent: AtomicU64,
    /// Total messages received
    pub messages_received: AtomicU64,
    /// Total bytes sent
    pub bytes_sent: AtomicU64,
    /// Total bytes received
    pub bytes_received: AtomicU64,
    /// Connection attempts
    pub connection_attempts: AtomicU64,
    /// Successful connections
    pub successful_connections: AtomicU64,
    /// Failed connections
    pub failed_connections: AtomicU64,
    /// Direct connections established
    pub direct_connections: AtomicU64,
    /// STUN connections established
    pub stun_connections: AtomicU64,
    /// TURN connections established
    pub turn_connections: AtomicU64,
}

impl P2pStats {
    /// Record a successful connection
    pub fn record_connection(&self, state: P2pConnectionState) {
        self.connection_attempts.fetch_add(1, Ordering::Relaxed);
        self.successful_connections.fetch_add(1, Ordering::Relaxed);

        match state {
            P2pConnectionState::ConnectedDirect => {
                self.direct_connections.fetch_add(1, Ordering::Relaxed);
            }
            P2pConnectionState::ConnectedStun => {
                self.stun_connections.fetch_add(1, Ordering::Relaxed);
            }
            P2pConnectionState::ConnectedTurn => {
                self.turn_connections.fetch_add(1, Ordering::Relaxed);
            }
            _ => {}
        }
    }

    /// Record a failed connection
    pub fn record_failure(&self) {
        self.connection_attempts.fetch_add(1, Ordering::Relaxed);
        self.failed_connections.fetch_add(1, Ordering::Relaxed);
    }
}

/// Result of P2P connection attempt
#[derive(Debug)]
pub struct P2pConnectionResult {
    /// Final connection state
    pub state: P2pConnectionState,
    /// Remote peer ID
    pub peer_id: PeerId,
    /// Local address
    pub local_addr: SocketAddr,
    /// Remote address (may be relay address for TURN)
    pub remote_addr: SocketAddr,
    /// Connection latency estimate
    pub latency: Duration,
    /// Error message if failed
    pub error: Option<String>,
}

/// Callback for P2P connection status updates
pub type P2pStatusCallback = Box<dyn Fn(P2pConnectionState) + Send + Sync>;

// =============================================================================
// Signaling Protocol Types
// =============================================================================

/// Signaling server message types for P2P connection establishment
///
/// The signaling protocol enables two peers to exchange connection information
/// without a direct connection. It uses a WebSocket connection to a signaling
/// server that relays messages between peers.
///
/// # Protocol Flow
///
/// ```text
/// Peer A                    Signaling Server                    Peer B
///   |                              |                               |
///   |-- Register(peer_id_a) ------>|                               |
///   |                              |<----- Register(peer_id_b) ----|
///   |                              |                               |
///   |-- Connect(peer_id_b) ------->|                               |
///   |                              |------- IncomingCall --------->|
///   |                              |<-------- Accept ------------- |
///   |<----- PeerInfo --------------|                               |
///   |                              |------- PeerInfo ------------->|
///   |                              |                               |
///   |-- IceCandidates ------------>|                               |
///   |                              |------ IceCandidates --------->|
///   |                              |                               |
///   |                              |<----- IceCandidates ----------|
///   |<----- IceCandidates ---------|                               |
///   |                              |                               |
///   |<================ Direct P2P Connection ===================>|
/// ```
#[derive(Debug, Clone)]
pub enum SignalingMessage {
    /// Register this peer with the signaling server
    Register {
        peer_id: PeerId,
        /// Optional authentication token
        auth_token: Option<String>,
    },

    /// Request connection to another peer
    Connect {
        target_peer_id: PeerId,
        /// Topics we want to communicate on
        topics: Vec<String>,
    },

    /// Incoming connection request from another peer
    IncomingCall {
        from_peer_id: PeerId,
        /// Topics the caller wants to communicate on
        topics: Vec<String>,
    },

    /// Accept an incoming connection
    Accept {
        peer_id: PeerId,
    },

    /// Reject an incoming connection
    Reject {
        peer_id: PeerId,
        reason: Option<String>,
    },

    /// Exchange peer information for connection
    PeerInfo {
        peer_id: PeerId,
        /// Local IP addresses (for direct connection attempts)
        local_addrs: Vec<SocketAddr>,
        /// STUN-discovered public address
        public_addr: Option<SocketAddr>,
    },

    /// Exchange ICE candidates
    IceCandidates {
        peer_id: PeerId,
        candidates: Vec<IceCandidate>,
    },

    /// Connection established successfully
    Connected {
        peer_id: PeerId,
        connection_type: P2pConnectionState,
    },

    /// Peer disconnected
    Disconnected {
        peer_id: PeerId,
    },

    /// Error from signaling server
    Error {
        code: SignalingErrorCode,
        message: String,
    },

    /// Keep-alive ping
    Ping,

    /// Keep-alive pong
    Pong,
}

/// Signaling server error codes
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SignalingErrorCode {
    /// Peer not found
    PeerNotFound,
    /// Peer is busy (already in connection)
    PeerBusy,
    /// Connection rejected by peer
    ConnectionRejected,
    /// Authentication failed
    AuthenticationFailed,
    /// Rate limited
    RateLimited,
    /// Invalid message format
    InvalidMessage,
    /// Server error
    ServerError,
    /// Connection timeout
    Timeout,
}

impl std::fmt::Display for SignalingErrorCode {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::PeerNotFound => write!(f, "Peer not found"),
            Self::PeerBusy => write!(f, "Peer is busy"),
            Self::ConnectionRejected => write!(f, "Connection rejected"),
            Self::AuthenticationFailed => write!(f, "Authentication failed"),
            Self::RateLimited => write!(f, "Rate limited"),
            Self::InvalidMessage => write!(f, "Invalid message"),
            Self::ServerError => write!(f, "Server error"),
            Self::Timeout => write!(f, "Connection timeout"),
        }
    }
}

/// Callback for handling incoming connection requests
///
/// Return `true` to accept, `false` to reject
pub type IncomingCallHandler = Box<dyn Fn(&PeerId, &[String]) -> bool + Send + Sync>;

/// Signaling client configuration
#[derive(Debug, Clone)]
pub struct SignalingClientConfig {
    /// Signaling server WebSocket URL
    pub server_url: String,
    /// Authentication token (optional)
    pub auth_token: Option<String>,
    /// Reconnect on disconnect
    pub auto_reconnect: bool,
    /// Reconnect delay
    pub reconnect_delay: Duration,
    /// Maximum reconnect attempts
    pub max_reconnect_attempts: u32,
    /// Ping interval for keep-alive
    pub ping_interval: Duration,
}

impl Default for SignalingClientConfig {
    fn default() -> Self {
        Self {
            server_url: "wss://signal.horus.robotics:8443".to_string(),
            auth_token: None,
            auto_reconnect: true,
            reconnect_delay: Duration::from_secs(1),
            max_reconnect_attempts: 10,
            ping_interval: Duration::from_secs(30),
        }
    }
}

/// Parse P2P endpoint string
///
/// Format: `peer-id` or `peer-id/strategy`
/// Examples:
/// - `a3f7-k2m9-p4n8` - Auto strategy
/// - `a3f7-k2m9-p4n8/direct` - Direct only
/// - `a3f7-k2m9-p4n8/stun` - STUN hole punch
/// - `a3f7-k2m9-p4n8/turn` - TURN relay
pub fn parse_p2p_location(location: &str) -> Result<(PeerId, P2pStrategy), String> {
    let parts: Vec<&str> = location.split('/').collect();

    let peer_id_str = parts[0];
    if !PeerId::is_valid_short_id(peer_id_str) {
        return Err(format!(
            "Invalid peer ID '{}': expected format xxxx-xxxx-xxxx",
            peer_id_str
        ));
    }

    let peer_id = PeerId::from_short_id(peer_id_str);

    let strategy = if parts.len() > 1 {
        parts[1].parse::<P2pStrategy>()?
    } else {
        P2pStrategy::Auto
    };

    Ok((peer_id, strategy))
}

// =============================================================================
// P2P Connector - Automatic Connection with Fallback
// =============================================================================

/// Error type for P2P connection failures
#[derive(Debug)]
pub enum P2pError {
    /// Direct connection failed
    DirectFailed(String),
    /// STUN hole punch failed
    StunFailed(String),
    /// TURN relay failed
    TurnFailed(String),
    /// All connection methods failed
    AllMethodsFailed {
        direct_error: Option<String>,
        stun_error: Option<String>,
        turn_error: Option<String>,
    },
    /// Connection timeout
    Timeout,
    /// Invalid configuration
    InvalidConfig(String),
    /// I/O error
    Io(std::io::Error),
    /// Socket binding failed
    BindFailed(String),
}

impl std::fmt::Display for P2pError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::DirectFailed(e) => write!(f, "Direct connection failed: {}", e),
            Self::StunFailed(e) => write!(f, "STUN hole punch failed: {}", e),
            Self::TurnFailed(e) => write!(f, "TURN relay failed: {}", e),
            Self::AllMethodsFailed {
                direct_error,
                stun_error,
                turn_error,
            } => {
                write!(f, "All connection methods failed: ")?;
                if let Some(e) = direct_error {
                    write!(f, "direct={}, ", e)?;
                }
                if let Some(e) = stun_error {
                    write!(f, "stun={}, ", e)?;
                }
                if let Some(e) = turn_error {
                    write!(f, "turn={}", e)?;
                }
                Ok(())
            }
            Self::Timeout => write!(f, "Connection timeout"),
            Self::InvalidConfig(e) => write!(f, "Invalid configuration: {}", e),
            Self::Io(e) => write!(f, "I/O error: {}", e),
            Self::BindFailed(e) => write!(f, "Socket bind failed: {}", e),
        }
    }
}

impl std::error::Error for P2pError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            Self::Io(e) => Some(e),
            _ => None,
        }
    }
}

impl From<std::io::Error> for P2pError {
    fn from(err: std::io::Error) -> Self {
        P2pError::Io(err)
    }
}

/// P2P connection result type
pub type P2pResult<T> = Result<T, P2pError>;

/// Active P2P connection with the selected transport
pub struct P2pConnection {
    /// Current connection state
    pub state: P2pConnectionState,
    /// Remote peer ID
    pub peer_id: PeerId,
    /// Local socket address
    pub local_addr: SocketAddr,
    /// Remote socket address (peer's address or relay address)
    pub remote_addr: SocketAddr,
    /// UDP socket for direct/STUN connections
    socket: Option<Arc<UdpSocket>>,
    /// TURN client for relay connections (sync, requires spawn_blocking)
    turn_client: Option<Arc<Mutex<TurnClient>>>,
    /// Channel number for TURN data relay (if using channel binding)
    turn_channel: Option<u16>,
    /// Connection established time
    pub connected_at: Instant,
    /// Statistics
    pub stats: P2pStats,
}

impl P2pConnection {
    /// Send data to the remote peer
    pub async fn send(&self, data: &[u8]) -> P2pResult<usize> {
        match self.state {
            P2pConnectionState::ConnectedDirect | P2pConnectionState::ConnectedStun => {
                // Send directly via UDP socket
                let socket = self.socket.as_ref().ok_or_else(|| {
                    P2pError::Io(std::io::Error::new(
                        std::io::ErrorKind::NotConnected,
                        "Socket not available",
                    ))
                })?;
                let sent = socket.send_to(data, self.remote_addr).await?;
                self.stats.messages_sent.fetch_add(1, Ordering::Relaxed);
                self.stats
                    .bytes_sent
                    .fetch_add(sent as u64, Ordering::Relaxed);
                Ok(sent)
            }
            P2pConnectionState::ConnectedTurn => {
                // Send via TURN relay (sync client, use spawn_blocking)
                let turn = self.turn_client.clone().ok_or_else(|| {
                    P2pError::Io(std::io::Error::new(
                        std::io::ErrorKind::NotConnected,
                        "TURN client not available",
                    ))
                })?;

                let channel = self.turn_channel;
                let remote = self.remote_addr;
                let data_vec = data.to_vec();
                let data_len = data_vec.len();

                tokio::task::spawn_blocking(move || {
                    let client = turn.lock().map_err(|_| {
                        P2pError::Io(std::io::Error::new(
                            std::io::ErrorKind::Other,
                            "TURN client lock poisoned",
                        ))
                    })?;

                    if let Some(ch) = channel {
                        client
                            .send_via_channel(ch, &data_vec)
                            .map_err(|e| P2pError::TurnFailed(e.to_string()))
                    } else {
                        client
                            .send(remote, &data_vec)
                            .map_err(|e| P2pError::TurnFailed(e.to_string()))
                    }
                })
                .await
                .map_err(|e| P2pError::Io(std::io::Error::new(
                    std::io::ErrorKind::Other,
                    format!("spawn_blocking failed: {}", e),
                )))??;

                self.stats.messages_sent.fetch_add(1, Ordering::Relaxed);
                self.stats
                    .bytes_sent
                    .fetch_add(data_len as u64, Ordering::Relaxed);
                Ok(data_len)
            }
            _ => Err(P2pError::Io(std::io::Error::new(
                std::io::ErrorKind::NotConnected,
                "Not connected",
            ))),
        }
    }

    /// Receive data from the remote peer
    pub async fn recv(&self, buf: &mut [u8]) -> P2pResult<(usize, SocketAddr)> {
        match self.state {
            P2pConnectionState::ConnectedDirect | P2pConnectionState::ConnectedStun => {
                let socket = self.socket.as_ref().ok_or_else(|| {
                    P2pError::Io(std::io::Error::new(
                        std::io::ErrorKind::NotConnected,
                        "Socket not available",
                    ))
                })?;
                let (len, from) = socket.recv_from(buf).await?;
                self.stats.messages_received.fetch_add(1, Ordering::Relaxed);
                self.stats
                    .bytes_received
                    .fetch_add(len as u64, Ordering::Relaxed);
                Ok((len, from))
            }
            P2pConnectionState::ConnectedTurn => {
                // Receive via TURN relay (sync client, use spawn_blocking)
                let turn = self.turn_client.clone().ok_or_else(|| {
                    P2pError::Io(std::io::Error::new(
                        std::io::ErrorKind::NotConnected,
                        "TURN client not available",
                    ))
                })?;

                let buf_len = buf.len();
                let result = tokio::task::spawn_blocking(move || {
                    let client = turn.lock().map_err(|_| {
                        P2pError::Io(std::io::Error::new(
                            std::io::ErrorKind::Other,
                            "TURN client lock poisoned",
                        ))
                    })?;

                    let mut recv_buf = vec![0u8; buf_len];
                    match client.recv(&mut recv_buf) {
                        Ok(Some((from, len))) => Ok((recv_buf, len, from)),
                        Ok(None) => Err(P2pError::Io(std::io::Error::new(
                            std::io::ErrorKind::WouldBlock,
                            "No data available",
                        ))),
                        Err(e) => Err(P2pError::TurnFailed(e.to_string())),
                    }
                })
                .await
                .map_err(|e| P2pError::Io(std::io::Error::new(
                    std::io::ErrorKind::Other,
                    format!("spawn_blocking failed: {}", e),
                )))??;

                let (recv_buf, len, from) = result;
                buf[..len].copy_from_slice(&recv_buf[..len]);

                self.stats.messages_received.fetch_add(1, Ordering::Relaxed);
                self.stats
                    .bytes_received
                    .fetch_add(len as u64, Ordering::Relaxed);
                Ok((len, from))
            }
            _ => Err(P2pError::Io(std::io::Error::new(
                std::io::ErrorKind::NotConnected,
                "Not connected",
            ))),
        }
    }

    /// Get connection latency (estimated round-trip time)
    pub fn latency(&self) -> Duration {
        // TODO: Implement proper RTT measurement
        match self.state {
            P2pConnectionState::ConnectedDirect => Duration::from_micros(100),
            P2pConnectionState::ConnectedStun => Duration::from_millis(5),
            P2pConnectionState::ConnectedTurn => Duration::from_millis(50),
            _ => Duration::ZERO,
        }
    }

    /// Close the connection
    pub async fn close(&mut self) {
        if let Some(turn) = self.turn_client.take() {
            let _ = tokio::task::spawn_blocking(move || {
                if let Ok(client) = turn.lock() {
                    let _ = client.release();
                }
            })
            .await;
        }
        self.state = P2pConnectionState::Disconnected;
    }
}

/// P2P Connector for establishing connections with automatic fallback
///
/// # Example
///
/// ```ignore
/// let config = P2pConfig {
///     stun_servers: vec!["stun.l.google.com:19302".to_string()],
///     turn_servers: vec![TurnServer {
///         address: "turn.example.com:3478".to_string(),
///         username: "user".to_string(),
///         credential: "pass".to_string(),
///     }],
///     ..Default::default()
/// };
///
/// let connector = P2pConnector::new(config);
///
/// // Connect with automatic fallback
/// let connection = connector.connect(
///     &peer_id,
///     peer_addr,
///     P2pStrategy::Auto,
///     Some(status_callback),
/// ).await?;
///
/// connection.send(b"Hello!").await?;
/// ```
pub struct P2pConnector {
    /// Configuration
    config: P2pConfig,
    /// Statistics
    stats: Arc<P2pStats>,
}

impl P2pConnector {
    /// Create a new P2P connector with the given configuration
    pub fn new(config: P2pConfig) -> Self {
        Self {
            config,
            stats: Arc::new(P2pStats::default()),
        }
    }

    /// Get reference to stats
    pub fn stats(&self) -> &P2pStats {
        &self.stats
    }

    /// Connect to a peer with the specified strategy
    ///
    /// # Arguments
    /// * `peer_id` - The peer to connect to
    /// * `peer_addr` - Known address of the peer (from signaling)
    /// * `strategy` - Connection strategy (Auto, Direct, Stun, Turn)
    /// * `on_status` - Optional callback for status updates
    pub async fn connect(
        &self,
        peer_id: &PeerId,
        peer_addr: SocketAddr,
        strategy: P2pStrategy,
        on_status: Option<impl Fn(P2pConnectionState) + Send + Sync>,
    ) -> P2pResult<P2pConnection> {
        let notify_status = |state: P2pConnectionState| {
            if let Some(ref cb) = on_status {
                cb(state);
            }
        };

        match strategy {
            P2pStrategy::Auto => {
                self.connect_auto(peer_id, peer_addr, notify_status).await
            }
            P2pStrategy::Direct => {
                notify_status(P2pConnectionState::TryingDirect);
                self.try_direct(peer_id, peer_addr).await
            }
            P2pStrategy::Stun => {
                notify_status(P2pConnectionState::TryingStun);
                self.try_stun(peer_id, peer_addr).await
            }
            P2pStrategy::Turn => {
                notify_status(P2pConnectionState::TryingTurn);
                self.try_turn(peer_id, peer_addr).await
            }
        }
    }

    /// Connect with automatic fallback: Direct → STUN → TURN
    async fn connect_auto(
        &self,
        peer_id: &PeerId,
        peer_addr: SocketAddr,
        notify_status: impl Fn(P2pConnectionState),
    ) -> P2pResult<P2pConnection> {
        let mut direct_error = None;
        let mut stun_error = None;
        let mut turn_error = None;

        // Step 1: Try direct connection (fastest, same LAN)
        notify_status(P2pConnectionState::TryingDirect);
        match timeout(self.config.direct_timeout, self.try_direct(peer_id, peer_addr)).await {
            Ok(Ok(conn)) => {
                self.stats.record_connection(P2pConnectionState::ConnectedDirect);
                return Ok(conn);
            }
            Ok(Err(e)) => {
                direct_error = Some(e.to_string());
                log::debug!("Direct connection failed: {}", e);
            }
            Err(_) => {
                direct_error = Some("timeout".to_string());
                log::debug!("Direct connection timed out");
            }
        }

        // Step 2: Try STUN hole punch (if STUN servers configured)
        if !self.config.stun_servers.is_empty() {
            notify_status(P2pConnectionState::TryingStun);
            match timeout(self.config.stun_timeout, self.try_stun(peer_id, peer_addr)).await {
                Ok(Ok(conn)) => {
                    self.stats.record_connection(P2pConnectionState::ConnectedStun);
                    return Ok(conn);
                }
                Ok(Err(e)) => {
                    stun_error = Some(e.to_string());
                    log::debug!("STUN hole punch failed: {}", e);
                }
                Err(_) => {
                    stun_error = Some("timeout".to_string());
                    log::debug!("STUN hole punch timed out");
                }
            }
        } else {
            stun_error = Some("no STUN servers configured".to_string());
        }

        // Step 3: Fall back to TURN relay (if TURN servers configured)
        if !self.config.turn_servers.is_empty() {
            notify_status(P2pConnectionState::TryingTurn);
            match timeout(
                self.config.connection_timeout,
                self.try_turn(peer_id, peer_addr),
            )
            .await
            {
                Ok(Ok(conn)) => {
                    self.stats.record_connection(P2pConnectionState::ConnectedTurn);
                    return Ok(conn);
                }
                Ok(Err(e)) => {
                    turn_error = Some(e.to_string());
                    log::debug!("TURN relay failed: {}", e);
                }
                Err(_) => {
                    turn_error = Some("timeout".to_string());
                    log::debug!("TURN relay timed out");
                }
            }
        } else {
            turn_error = Some("no TURN servers configured".to_string());
        }

        // All methods failed
        notify_status(P2pConnectionState::Failed);
        self.stats.record_failure();
        Err(P2pError::AllMethodsFailed {
            direct_error,
            stun_error,
            turn_error,
        })
    }

    /// Try direct UDP connection
    async fn try_direct(
        &self,
        peer_id: &PeerId,
        peer_addr: SocketAddr,
    ) -> P2pResult<P2pConnection> {
        // Bind to any available port
        let bind_addr: SocketAddr = if peer_addr.is_ipv6() {
            "[::]:0".parse().unwrap()
        } else {
            "0.0.0.0:0".parse().unwrap()
        };

        let socket = UdpSocket::bind(bind_addr).await.map_err(|e| {
            P2pError::BindFailed(format!("Failed to bind UDP socket: {}", e))
        })?;

        let local_addr = socket.local_addr()?;

        // Send a ping to initiate connection
        let ping = b"HORUS_P2P_PING";
        socket.send_to(ping, peer_addr).await?;

        // Wait for pong response
        let mut buf = [0u8; 64];
        let recv_timeout = Duration::from_millis(200);

        match timeout(recv_timeout, socket.recv_from(&mut buf)).await {
            Ok(Ok((len, from))) => {
                if from == peer_addr && &buf[..len] == b"HORUS_P2P_PONG" {
                    Ok(P2pConnection {
                        state: P2pConnectionState::ConnectedDirect,
                        peer_id: peer_id.clone(),
                        local_addr,
                        remote_addr: peer_addr,
                        socket: Some(Arc::new(socket)),
                        turn_client: None,
                        turn_channel: None,
                        connected_at: Instant::now(),
                        stats: P2pStats::default(),
                    })
                } else {
                    Err(P2pError::DirectFailed("Invalid response".to_string()))
                }
            }
            Ok(Err(e)) => Err(P2pError::DirectFailed(format!("Recv failed: {}", e))),
            Err(_) => Err(P2pError::DirectFailed("No response".to_string())),
        }
    }

    /// Try STUN-assisted hole punch
    async fn try_stun(
        &self,
        peer_id: &PeerId,
        peer_addr: SocketAddr,
    ) -> P2pResult<P2pConnection> {
        // Get STUN server address
        let stun_server = self.config.stun_servers.first().ok_or_else(|| {
            P2pError::StunFailed("No STUN servers configured".to_string())
        })?;

        // Resolve STUN server address
        let stun_addr: SocketAddr = tokio::net::lookup_host(stun_server)
            .await
            .map_err(|e| P2pError::StunFailed(format!("Failed to resolve STUN server: {}", e)))?
            .next()
            .ok_or_else(|| P2pError::StunFailed("STUN server address not found".to_string()))?;

        // Bind to any available port
        let bind_addr: SocketAddr = if stun_addr.is_ipv6() {
            "[::]:0".parse().unwrap()
        } else {
            "0.0.0.0:0".parse().unwrap()
        };

        let socket = UdpSocket::bind(bind_addr).await.map_err(|e| {
            P2pError::BindFailed(format!("Failed to bind STUN socket: {}", e))
        })?;

        let local_addr = socket.local_addr()?;

        // Perform simple STUN binding request to discover public address
        let public_addr = self.stun_binding_request(&socket, stun_addr).await?;
        log::debug!("STUN discovered public address: {}", public_addr);

        // Send hole punch packets to peer's public address
        // The peer should be doing the same to our public address
        let punch = b"HORUS_P2P_PUNCH";

        // Send multiple punch packets
        for _ in 0..5 {
            let _ = socket.send_to(punch, peer_addr).await;
            tokio::time::sleep(Duration::from_millis(50)).await;
        }

        // Wait for incoming punch or pong
        let mut buf = [0u8; 64];
        let punch_timeout = Duration::from_secs(2);

        match timeout(punch_timeout, socket.recv_from(&mut buf)).await {
            Ok(Ok((len, from))) => {
                let response = &buf[..len];

                // Send pong if we got a punch
                if response == b"HORUS_P2P_PUNCH" {
                    socket.send_to(b"HORUS_P2P_PONG", from).await?;
                }

                // Hole punch successful
                Ok(P2pConnection {
                    state: P2pConnectionState::ConnectedStun,
                    peer_id: peer_id.clone(),
                    local_addr,
                    remote_addr: from,
                    socket: Some(Arc::new(socket)),
                    turn_client: None,
                    turn_channel: None,
                    connected_at: Instant::now(),
                    stats: P2pStats::default(),
                })
            }
            Ok(Err(e)) => Err(P2pError::StunFailed(format!("Hole punch recv failed: {}", e))),
            Err(_) => Err(P2pError::StunFailed("Hole punch timeout".to_string())),
        }
    }

    /// Perform a simple STUN binding request
    async fn stun_binding_request(
        &self,
        socket: &UdpSocket,
        stun_server: SocketAddr,
    ) -> P2pResult<SocketAddr> {
        // STUN constants
        const STUN_MAGIC_COOKIE: u32 = 0x2112A442;
        const STUN_BINDING_REQUEST: u16 = 0x0001;
        const STUN_BINDING_RESPONSE: u16 = 0x0101;
        const ATTR_XOR_MAPPED_ADDRESS: u16 = 0x0020;
        const ATTR_MAPPED_ADDRESS: u16 = 0x0001;

        // Generate transaction ID (12 bytes) using time and local addr as entropy
        let mut hasher = DefaultHasher::new();
        std::time::SystemTime::now().hash(&mut hasher);
        socket.local_addr().ok().hash(&mut hasher);
        let hash1 = hasher.finish();
        hasher.write_u64(hash1);
        let hash2 = hasher.finish();
        let mut transaction_id = [0u8; 12];
        transaction_id[..8].copy_from_slice(&hash1.to_le_bytes());
        transaction_id[8..].copy_from_slice(&hash2.to_le_bytes()[..4]);

        // Build STUN binding request (20 bytes header, no attributes)
        let mut request = Vec::with_capacity(20);
        request.extend_from_slice(&STUN_BINDING_REQUEST.to_be_bytes());
        request.extend_from_slice(&0u16.to_be_bytes()); // Message length (no attributes)
        request.extend_from_slice(&STUN_MAGIC_COOKIE.to_be_bytes());
        request.extend_from_slice(&transaction_id);

        // Send request
        socket.send_to(&request, stun_server).await?;

        // Wait for response
        let mut buf = [0u8; 256];
        let recv_timeout = Duration::from_secs(3);

        let (len, _from) = timeout(recv_timeout, socket.recv_from(&mut buf))
            .await
            .map_err(|_| P2pError::StunFailed("STUN request timed out".to_string()))?
            .map_err(|e| P2pError::StunFailed(format!("STUN recv failed: {}", e)))?;

        if len < 20 {
            return Err(P2pError::StunFailed("STUN response too short".to_string()));
        }

        // Parse response header
        let msg_type = u16::from_be_bytes([buf[0], buf[1]]);
        let msg_len = u16::from_be_bytes([buf[2], buf[3]]) as usize;

        if msg_type != STUN_BINDING_RESPONSE {
            return Err(P2pError::StunFailed(format!(
                "Unexpected STUN response type: 0x{:04x}",
                msg_type
            )));
        }

        // Parse attributes
        let mut offset = 20;
        while offset + 4 <= 20 + msg_len && offset + 4 <= len {
            let attr_type = u16::from_be_bytes([buf[offset], buf[offset + 1]]);
            let attr_len = u16::from_be_bytes([buf[offset + 2], buf[offset + 3]]) as usize;
            offset += 4;

            if offset + attr_len > len {
                break;
            }

            match attr_type {
                ATTR_XOR_MAPPED_ADDRESS => {
                    // XOR-MAPPED-ADDRESS (preferred)
                    if attr_len >= 8 {
                        let family = buf[offset + 1];
                        let xor_port =
                            u16::from_be_bytes([buf[offset + 2], buf[offset + 3]]) ^ 0x2112;

                        if family == 0x01 && attr_len >= 8 {
                            // IPv4
                            let xor_ip = u32::from_be_bytes([
                                buf[offset + 4],
                                buf[offset + 5],
                                buf[offset + 6],
                                buf[offset + 7],
                            ]) ^ STUN_MAGIC_COOKIE;

                            let ip = std::net::Ipv4Addr::from(xor_ip);
                            return Ok(SocketAddr::new(ip.into(), xor_port));
                        } else if family == 0x02 && attr_len >= 20 {
                            // IPv6
                            let mut xor_ip = [0u8; 16];
                            xor_ip.copy_from_slice(&buf[offset + 4..offset + 20]);
                            // XOR with magic cookie and transaction ID
                            let magic_bytes = STUN_MAGIC_COOKIE.to_be_bytes();
                            for i in 0..4 {
                                xor_ip[i] ^= magic_bytes[i];
                            }
                            for i in 0..12 {
                                xor_ip[4 + i] ^= transaction_id[i];
                            }

                            let ip = std::net::Ipv6Addr::from(xor_ip);
                            return Ok(SocketAddr::new(ip.into(), xor_port));
                        }
                    }
                }
                ATTR_MAPPED_ADDRESS => {
                    // MAPPED-ADDRESS (fallback)
                    if attr_len >= 8 {
                        let family = buf[offset + 1];
                        let port = u16::from_be_bytes([buf[offset + 2], buf[offset + 3]]);

                        if family == 0x01 {
                            // IPv4
                            let ip = std::net::Ipv4Addr::new(
                                buf[offset + 4],
                                buf[offset + 5],
                                buf[offset + 6],
                                buf[offset + 7],
                            );
                            return Ok(SocketAddr::new(ip.into(), port));
                        }
                    }
                }
                _ => {}
            }

            // Move to next attribute (4-byte aligned)
            offset += (attr_len + 3) & !3;
        }

        Err(P2pError::StunFailed(
            "No mapped address in STUN response".to_string(),
        ))
    }

    /// Connect via TURN relay
    async fn try_turn(
        &self,
        peer_id: &PeerId,
        peer_addr: SocketAddr,
    ) -> P2pResult<P2pConnection> {
        let turn_server = self.config.turn_servers.first().ok_or_else(|| {
            P2pError::TurnFailed("No TURN servers configured".to_string())
        })?;

        // Parse server address
        let server_addr: SocketAddr = turn_server
            .address
            .parse()
            .or_else(|_| {
                // Try adding default port
                format!("{}:3478", turn_server.address).parse()
            })
            .map_err(|e| P2pError::TurnFailed(format!("Invalid server address: {}", e)))?;

        // Create TURN configuration
        let turn_config = TurnConfig {
            server: server_addr.to_string(),
            username: turn_server.username.clone(),
            credential: turn_server.credential.clone(),
            realm: None,
            timeout: Duration::from_secs(5),
            retries: 3,
            lifetime: Duration::from_secs(600),
            use_channel_binding: true, // More efficient for data relay
            local_addr: None,
            transport: TurnTransport::Udp,
        };

        // Create TURN client and allocate relay address (sync operations via spawn_blocking)
        let peer_addr_copy = peer_addr;
        let peer_id_copy = peer_id.clone();
        let use_channel_binding = turn_config.use_channel_binding;

        let result = tokio::task::spawn_blocking(move || {
            // Create TURN client
            let turn_client = TurnClient::new(turn_config)
                .map_err(|e| P2pError::TurnFailed(format!("Failed to create TURN client: {}", e)))?;

            // Allocate relay address
            let allocation = turn_client
                .allocate()
                .map_err(|e| P2pError::TurnFailed(format!("Allocation failed: {}", e)))?;

            let relay_addr = allocation.relay_address;
            let local_addr = allocation.mapped_address;

            log::info!(
                "TURN allocated relay address: {}, mapped: {}",
                relay_addr,
                local_addr
            );

            // Create permission for the peer
            turn_client
                .create_permission(peer_addr_copy.ip())
                .map_err(|e| P2pError::TurnFailed(format!("Permission failed: {}", e)))?;

            // Optionally create channel binding for more efficient data relay
            let channel = if use_channel_binding {
                match turn_client.channel_bind(peer_addr_copy) {
                    Ok(ch) => {
                        log::debug!("Channel bound: {} -> {}", ch, peer_addr_copy);
                        Some(ch)
                    }
                    Err(e) => {
                        log::warn!("Channel binding failed (using Send/Data): {}", e);
                        None
                    }
                }
            } else {
                None
            };

            // Send initial message through relay
            let hello = b"HORUS_P2P_TURN_HELLO";
            turn_client
                .send(peer_addr_copy, hello)
                .map_err(|e| P2pError::TurnFailed(format!("Failed to send through relay: {}", e)))?;

            Ok::<_, P2pError>((turn_client, local_addr, channel, peer_id_copy))
        })
        .await
        .map_err(|e| P2pError::Io(std::io::Error::new(
            std::io::ErrorKind::Other,
            format!("spawn_blocking failed: {}", e),
        )))??;

        let (turn_client, local_addr, channel, _) = result;

        Ok(P2pConnection {
            state: P2pConnectionState::ConnectedTurn,
            peer_id: peer_id.clone(),
            local_addr,
            remote_addr: peer_addr,
            socket: None,
            turn_client: Some(Arc::new(Mutex::new(turn_client))),
            turn_channel: channel,
            connected_at: Instant::now(),
            stats: P2pStats::default(),
        })
    }
}

/// Handle incoming P2P connection (responder side)
///
/// This creates a connection handler that responds to pings/punches
/// from a peer trying to connect.
pub struct P2pResponder {
    /// Bound socket for listening
    socket: Arc<UdpSocket>,
    /// Local address
    local_addr: SocketAddr,
    /// Configuration
    config: P2pConfig,
}

impl P2pResponder {
    /// Create a new P2P responder bound to the specified address
    pub async fn bind(addr: SocketAddr, config: P2pConfig) -> P2pResult<Self> {
        let socket = UdpSocket::bind(addr).await.map_err(|e| {
            P2pError::BindFailed(format!("Failed to bind responder socket: {}", e))
        })?;

        let local_addr = socket.local_addr()?;

        Ok(Self {
            socket: Arc::new(socket),
            local_addr,
            config,
        })
    }

    /// Get the local address
    pub fn local_addr(&self) -> SocketAddr {
        self.local_addr
    }

    /// Wait for and accept an incoming connection
    pub async fn accept(&self) -> P2pResult<(P2pConnection, PeerId)> {
        let mut buf = [0u8; 1024];

        loop {
            let (len, from) = self.socket.recv_from(&mut buf).await?;
            let data = &buf[..len];

            match data {
                b"HORUS_P2P_PING" => {
                    // Direct connection attempt - respond with pong
                    self.socket.send_to(b"HORUS_P2P_PONG", from).await?;

                    // Generate peer ID from address (temporary until signaling provides real ID)
                    let peer_id = PeerId::from_short_id(&format!(
                        "{:04x}-{:04x}-{:04x}",
                        from.port(),
                        from.ip().to_string().len() as u16,
                        len as u16
                    ));

                    return Ok((
                        P2pConnection {
                            state: P2pConnectionState::ConnectedDirect,
                            peer_id: peer_id.clone(),
                            local_addr: self.local_addr,
                            remote_addr: from,
                            socket: Some(self.socket.clone()),
                            turn_client: None,
                            turn_channel: None,
                            connected_at: Instant::now(),
                            stats: P2pStats::default(),
                        },
                        peer_id,
                    ));
                }
                b"HORUS_P2P_PUNCH" => {
                    // Hole punch attempt - respond and establish connection
                    self.socket.send_to(b"HORUS_P2P_PONG", from).await?;

                    let peer_id = PeerId::from_short_id(&format!(
                        "{:04x}-{:04x}-{:04x}",
                        from.port(),
                        from.ip().to_string().len() as u16,
                        len as u16
                    ));

                    return Ok((
                        P2pConnection {
                            state: P2pConnectionState::ConnectedStun,
                            peer_id: peer_id.clone(),
                            local_addr: self.local_addr,
                            remote_addr: from,
                            socket: Some(self.socket.clone()),
                            turn_client: None,
                            turn_channel: None,
                            connected_at: Instant::now(),
                            stats: P2pStats::default(),
                        },
                        peer_id,
                    ));
                }
                b"HORUS_P2P_TURN_HELLO" => {
                    // TURN relay connection (we received data via relay)
                    let peer_id = PeerId::from_short_id(&format!(
                        "{:04x}-{:04x}-{:04x}",
                        from.port(),
                        from.ip().to_string().len() as u16,
                        len as u16
                    ));

                    return Ok((
                        P2pConnection {
                            state: P2pConnectionState::ConnectedTurn,
                            peer_id: peer_id.clone(),
                            local_addr: self.local_addr,
                            remote_addr: from,
                            socket: Some(self.socket.clone()),
                            turn_client: None,
                            turn_channel: None,
                            connected_at: Instant::now(),
                            stats: P2pStats::default(),
                        },
                        peer_id,
                    ));
                }
                _ => {
                    // Ignore unknown messages
                    log::trace!("Ignoring unknown P2P message from {}", from);
                }
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_peer_id_from_public_key() {
        let key = [1u8; 32];
        let peer_id = PeerId::from_public_key(&key);
        assert!(PeerId::is_valid_short_id(&peer_id.short_id));
        assert_eq!(peer_id.public_key.len(), 32);
    }

    #[test]
    fn test_peer_id_deterministic() {
        let key = [42u8; 32];
        let id1 = PeerId::from_public_key(&key);
        let id2 = PeerId::from_public_key(&key);
        assert_eq!(id1.short_id, id2.short_id);
    }

    #[test]
    fn test_peer_id_valid_format() {
        assert!(PeerId::is_valid_short_id("a3f7-k2m9-p4n8"));
        assert!(PeerId::is_valid_short_id("0000-0000-0000"));
        assert!(PeerId::is_valid_short_id("zzzz-zzzz-zzzz"));
        assert!(PeerId::is_valid_short_id("AbCd-EfGh-IjKl"));
    }

    #[test]
    fn test_peer_id_invalid_format() {
        assert!(!PeerId::is_valid_short_id("a3f7k2m9p4n8")); // No dashes
        assert!(!PeerId::is_valid_short_id("a3f7-k2m9")); // Too short
        assert!(!PeerId::is_valid_short_id("a3f7-k2m9-p4n8-xxxx")); // Too long
        assert!(!PeerId::is_valid_short_id("a3f7_k2m9_p4n8")); // Wrong separator
        assert!(!PeerId::is_valid_short_id("")); // Empty
    }

    #[test]
    fn test_parse_p2p_location_basic() {
        let (peer_id, strategy) = parse_p2p_location("a3f7-k2m9-p4n8").unwrap();
        assert_eq!(peer_id.short_id, "a3f7-k2m9-p4n8");
        assert_eq!(strategy, P2pStrategy::Auto);
    }

    #[test]
    fn test_parse_p2p_location_with_strategy() {
        let (peer_id, strategy) = parse_p2p_location("a3f7-k2m9-p4n8/direct").unwrap();
        assert_eq!(peer_id.short_id, "a3f7-k2m9-p4n8");
        assert_eq!(strategy, P2pStrategy::Direct);

        let (_, strategy) = parse_p2p_location("a3f7-k2m9-p4n8/stun").unwrap();
        assert_eq!(strategy, P2pStrategy::Stun);

        let (_, strategy) = parse_p2p_location("a3f7-k2m9-p4n8/turn").unwrap();
        assert_eq!(strategy, P2pStrategy::Turn);
    }

    #[test]
    fn test_parse_p2p_location_invalid() {
        assert!(parse_p2p_location("invalid-peer-id").is_err());
        assert!(parse_p2p_location("a3f7k2m9p4n8").is_err());
        assert!(parse_p2p_location("").is_err());
    }

    #[test]
    fn test_connection_state() {
        assert!(P2pConnectionState::ConnectedDirect.is_connected());
        assert!(P2pConnectionState::ConnectedStun.is_connected());
        assert!(P2pConnectionState::ConnectedTurn.is_connected());
        assert!(!P2pConnectionState::Disconnected.is_connected());
        assert!(!P2pConnectionState::Failed.is_connected());

        assert!(P2pConnectionState::Discovering.is_connecting());
        assert!(P2pConnectionState::TryingDirect.is_connecting());
        assert!(!P2pConnectionState::ConnectedDirect.is_connecting());
    }

    #[test]
    fn test_p2p_strategy_parse() {
        assert_eq!("auto".parse::<P2pStrategy>().unwrap(), P2pStrategy::Auto);
        assert_eq!("direct".parse::<P2pStrategy>().unwrap(), P2pStrategy::Direct);
        assert_eq!("STUN".parse::<P2pStrategy>().unwrap(), P2pStrategy::Stun);
        assert_eq!("Turn".parse::<P2pStrategy>().unwrap(), P2pStrategy::Turn);
        assert!("invalid".parse::<P2pStrategy>().is_err());
    }
}
