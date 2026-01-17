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

use std::net::SocketAddr;
use std::sync::atomic::{AtomicU64, Ordering};
use std::time::Duration;

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
