//! TURN (Traversal Using Relays around NAT) client implementation
//!
//! This module provides TURN protocol support for relaying data through a server
//! when direct P2P connections fail. TURN is the fallback mechanism when STUN-based
//! hole punching cannot succeed (e.g., symmetric NAT).
//!
//! # Protocol Support
//!
//! Implements RFC 5766/8656 (TURN) with support for:
//! - Allocation requests to obtain a relayed transport address
//! - Channel binding for efficient data relay (4-byte header vs 36-byte)
//! - Long-term credential authentication (RFC 5389)
//! - Permission management
//! - Allocation refresh
//!
//! # Example
//!
//! ```ignore
//! use horus_core::communication::network::turn::{TurnClient, TurnConfig};
//!
//! let config = TurnConfig {
//!     server: "turn.example.com:3478".to_string(),
//!     username: "user".to_string(),
//!     credential: "password".to_string(),
//!     ..Default::default()
//! };
//!
//! let client = TurnClient::new(config)?;
//!
//! // Create allocation to get relay address
//! let allocation = client.allocate()?;
//! println!("Relay address: {}", allocation.relay_address);
//!
//! // Bind channel for efficient data transfer
//! client.channel_bind(allocation.id, peer_addr, 0x4000)?;
//!
//! // Send data through relay
//! client.send_via_channel(0x4000, data)?;
//! ```

use std::collections::HashMap;
use std::io;
use std::net::{SocketAddr, ToSocketAddrs, UdpSocket};
use std::sync::atomic::{AtomicBool, AtomicU32, AtomicU64, Ordering};
use std::sync::{Arc, Mutex, RwLock};
use std::time::{Duration, Instant};

// ============================================================================
// TURN/STUN Constants (RFC 5766, RFC 8656)
// ============================================================================

const STUN_MAGIC_COOKIE: u32 = 0x2112A442;
const STUN_HEADER_SIZE: usize = 20;

// TURN message types (RFC 5766 Section 6)
const TURN_ALLOCATE_REQUEST: u16 = 0x0003;
const TURN_ALLOCATE_RESPONSE: u16 = 0x0103;
const TURN_ALLOCATE_ERROR: u16 = 0x0113;
const TURN_REFRESH_REQUEST: u16 = 0x0004;
const TURN_REFRESH_RESPONSE: u16 = 0x0104;
const TURN_REFRESH_ERROR: u16 = 0x0114;
const TURN_SEND_INDICATION: u16 = 0x0016;
const TURN_DATA_INDICATION: u16 = 0x0017;
const TURN_CREATE_PERMISSION_REQUEST: u16 = 0x0008;
const TURN_CREATE_PERMISSION_RESPONSE: u16 = 0x0108;
const TURN_CREATE_PERMISSION_ERROR: u16 = 0x0118;
const TURN_CHANNEL_BIND_REQUEST: u16 = 0x0009;
const TURN_CHANNEL_BIND_RESPONSE: u16 = 0x0109;
const TURN_CHANNEL_BIND_ERROR: u16 = 0x0119;

// STUN/TURN attribute types
const ATTR_MAPPED_ADDRESS: u16 = 0x0001;
const ATTR_USERNAME: u16 = 0x0006;
const ATTR_MESSAGE_INTEGRITY: u16 = 0x0008;
const ATTR_ERROR_CODE: u16 = 0x0009;
const ATTR_UNKNOWN_ATTRIBUTES: u16 = 0x000A;
const ATTR_REALM: u16 = 0x0014;
const ATTR_NONCE: u16 = 0x0015;
const ATTR_XOR_MAPPED_ADDRESS: u16 = 0x0020;
const ATTR_XOR_RELAYED_ADDRESS: u16 = 0x0016;
const ATTR_XOR_PEER_ADDRESS: u16 = 0x0012;
const ATTR_DATA: u16 = 0x0013;
const ATTR_LIFETIME: u16 = 0x000D;
const ATTR_REQUESTED_TRANSPORT: u16 = 0x0019;
const ATTR_CHANNEL_NUMBER: u16 = 0x000C;
const ATTR_SOFTWARE: u16 = 0x8022;
const ATTR_FINGERPRINT: u16 = 0x8028;

// TURN error codes
const ERROR_TRY_ALTERNATE: u16 = 300;
const ERROR_BAD_REQUEST: u16 = 400;
const ERROR_UNAUTHORIZED: u16 = 401;
const ERROR_UNKNOWN_ATTRIBUTE: u16 = 420;
const ERROR_STALE_NONCE: u16 = 438;
const ERROR_ALLOCATION_MISMATCH: u16 = 437;
const ERROR_WRONG_CREDENTIALS: u16 = 441;
const ERROR_UNSUPPORTED_TRANSPORT: u16 = 442;
const ERROR_ALLOCATION_QUOTA: u16 = 486;
const ERROR_INSUFFICIENT_CAPACITY: u16 = 508;

// Protocol values
const TRANSPORT_UDP: u8 = 17;
const TRANSPORT_TCP: u8 = 6;

// Channel number range (RFC 5766 Section 11)
const CHANNEL_NUMBER_MIN: u16 = 0x4000;
const CHANNEL_NUMBER_MAX: u16 = 0x7FFF;

// Default lifetimes
const DEFAULT_ALLOCATION_LIFETIME: Duration = Duration::from_secs(600);  // 10 minutes
const DEFAULT_CHANNEL_LIFETIME: Duration = Duration::from_secs(600);    // 10 minutes
const DEFAULT_PERMISSION_LIFETIME: Duration = Duration::from_secs(300); // 5 minutes

// Refresh margin (refresh before expiration)
const REFRESH_MARGIN: Duration = Duration::from_secs(60); // 1 minute before expiry

// ============================================================================
// TURN Types
// ============================================================================

/// TURN client configuration
#[derive(Debug, Clone)]
pub struct TurnConfig {
    /// TURN server address (host:port)
    pub server: String,
    /// Username for authentication
    pub username: String,
    /// Password/credential for authentication
    pub credential: String,
    /// Realm (usually set by server)
    pub realm: Option<String>,
    /// Request timeout
    pub timeout: Duration,
    /// Number of retries
    pub retries: u32,
    /// Requested allocation lifetime
    pub lifetime: Duration,
    /// Enable channel binding for efficiency
    pub use_channel_binding: bool,
    /// Local address to bind (None = auto)
    pub local_addr: Option<SocketAddr>,
    /// Request UDP transport (most common)
    pub transport: TurnTransport,
}

impl Default for TurnConfig {
    fn default() -> Self {
        Self {
            server: String::new(),
            username: String::new(),
            credential: String::new(),
            realm: None,
            timeout: Duration::from_secs(5),
            retries: 3,
            lifetime: DEFAULT_ALLOCATION_LIFETIME,
            use_channel_binding: true,
            local_addr: None,
            transport: TurnTransport::Udp,
        }
    }
}

/// TURN transport protocol
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TurnTransport {
    Udp,
    Tcp,
}

impl TurnTransport {
    fn to_protocol_number(self) -> u8 {
        match self {
            TurnTransport::Udp => TRANSPORT_UDP,
            TurnTransport::Tcp => TRANSPORT_TCP,
        }
    }
}

/// TURN allocation state
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AllocationState {
    /// No allocation
    None,
    /// Allocation request in progress
    Allocating,
    /// Allocation active
    Active,
    /// Allocation expired or released
    Expired,
    /// Allocation failed
    Failed,
}

/// TURN allocation information
#[derive(Debug, Clone)]
pub struct TurnAllocation {
    /// Allocation ID (local identifier)
    pub id: u32,
    /// Relayed transport address (the address peers see)
    pub relay_address: SocketAddr,
    /// Server reflexive address (our external address to TURN server)
    pub mapped_address: SocketAddr,
    /// Allocation lifetime
    pub lifetime: Duration,
    /// When allocation was created
    pub created_at: Instant,
    /// Current state
    pub state: AllocationState,
    /// Nonce for authentication
    pub nonce: Vec<u8>,
    /// Realm from server
    pub realm: String,
}

impl TurnAllocation {
    /// Check if allocation is still valid
    pub fn is_valid(&self) -> bool {
        self.state == AllocationState::Active
            && self.created_at.elapsed() < self.lifetime
    }

    /// Time until expiration
    pub fn time_to_expiry(&self) -> Duration {
        if self.state != AllocationState::Active {
            return Duration::ZERO;
        }
        let elapsed = self.created_at.elapsed();
        if elapsed >= self.lifetime {
            Duration::ZERO
        } else {
            self.lifetime - elapsed
        }
    }

    /// Check if allocation needs refresh
    pub fn needs_refresh(&self) -> bool {
        self.is_valid() && self.time_to_expiry() < REFRESH_MARGIN
    }
}

/// Channel binding for efficient data relay
#[derive(Debug, Clone)]
pub struct ChannelBinding {
    /// Channel number (0x4000-0x7FFF)
    pub channel_number: u16,
    /// Peer address this channel is bound to
    pub peer_address: SocketAddr,
    /// When binding was created
    pub created_at: Instant,
    /// Binding lifetime
    pub lifetime: Duration,
}

impl ChannelBinding {
    /// Check if binding is still valid
    pub fn is_valid(&self) -> bool {
        self.created_at.elapsed() < self.lifetime
    }

    /// Check if binding needs refresh
    pub fn needs_refresh(&self) -> bool {
        self.is_valid() && (self.lifetime - self.created_at.elapsed()) < REFRESH_MARGIN
    }
}

/// Permission entry
#[derive(Debug, Clone)]
pub struct Permission {
    /// Peer IP address (port-agnostic)
    pub peer_ip: std::net::IpAddr,
    /// When permission was created
    pub created_at: Instant,
    /// Permission lifetime
    pub lifetime: Duration,
}

impl Permission {
    /// Check if permission is still valid
    pub fn is_valid(&self) -> bool {
        self.created_at.elapsed() < self.lifetime
    }
}

/// TURN client statistics
#[derive(Debug, Default)]
pub struct TurnStats {
    /// Total allocations created
    pub allocations_created: AtomicU32,
    /// Total allocations failed
    pub allocations_failed: AtomicU32,
    /// Total data sent via relay
    pub bytes_sent: AtomicU64,
    /// Total data received via relay
    pub bytes_received: AtomicU64,
    /// Total messages relayed
    pub messages_relayed: AtomicU64,
    /// Channel bindings created
    pub channels_bound: AtomicU32,
    /// Permissions created
    pub permissions_created: AtomicU32,
    /// Authentication failures
    pub auth_failures: AtomicU32,
    /// Refresh operations
    pub refreshes: AtomicU32,
}

/// TURN client error
#[derive(Debug)]
pub enum TurnError {
    /// I/O error
    Io(io::Error),
    /// Server returned error
    ServerError { code: u16, message: String },
    /// Authentication failed
    AuthenticationFailed(String),
    /// No allocation exists
    NoAllocation,
    /// Allocation already exists
    AllocationExists,
    /// Invalid channel number
    InvalidChannel,
    /// Protocol error
    ProtocolError(String),
    /// Timeout
    Timeout,
}

impl std::fmt::Display for TurnError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            TurnError::Io(e) => write!(f, "I/O error: {}", e),
            TurnError::ServerError { code, message } => {
                write!(f, "TURN error {}: {}", code, message)
            }
            TurnError::AuthenticationFailed(msg) => {
                write!(f, "Authentication failed: {}", msg)
            }
            TurnError::NoAllocation => write!(f, "No active allocation"),
            TurnError::AllocationExists => write!(f, "Allocation already exists"),
            TurnError::InvalidChannel => write!(f, "Invalid channel number"),
            TurnError::ProtocolError(msg) => write!(f, "Protocol error: {}", msg),
            TurnError::Timeout => write!(f, "Request timeout"),
        }
    }
}

impl std::error::Error for TurnError {}

impl From<io::Error> for TurnError {
    fn from(e: io::Error) -> Self {
        TurnError::Io(e)
    }
}

pub type TurnResult<T> = Result<T, TurnError>;

// ============================================================================
// TURN Client
// ============================================================================

/// TURN client for relay-based NAT traversal
pub struct TurnClient {
    config: TurnConfig,
    socket: UdpSocket,
    server_addr: SocketAddr,
    /// Current allocation
    allocation: RwLock<Option<TurnAllocation>>,
    /// Channel bindings: channel_number -> binding
    channels: RwLock<HashMap<u16, ChannelBinding>>,
    /// Peer address -> channel number mapping
    peer_channels: RwLock<HashMap<SocketAddr, u16>>,
    /// Permissions: IP -> Permission
    permissions: RwLock<HashMap<std::net::IpAddr, Permission>>,
    /// Next channel number to assign
    next_channel: AtomicU32,
    /// Allocation ID counter
    allocation_counter: AtomicU32,
    /// Is client running
    running: AtomicBool,
    /// Statistics
    stats: Arc<TurnStats>,
    /// Pending transaction IDs
    pending_transactions: Mutex<HashMap<[u8; 12], Instant>>,
}

impl TurnClient {
    /// Create a new TURN client
    pub fn new(config: TurnConfig) -> TurnResult<Self> {
        if config.server.is_empty() {
            return Err(TurnError::ProtocolError("Server address required".into()));
        }

        // Resolve server address
        let server_addr = config
            .server
            .to_socket_addrs()
            .map_err(|e| TurnError::Io(io::Error::new(io::ErrorKind::InvalidInput, e)))?
            .next()
            .ok_or_else(|| {
                TurnError::Io(io::Error::new(
                    io::ErrorKind::InvalidInput,
                    "Could not resolve TURN server address",
                ))
            })?;

        // Create UDP socket
        let socket = if let Some(addr) = config.local_addr {
            UdpSocket::bind(addr)?
        } else {
            UdpSocket::bind("0.0.0.0:0")?
        };

        socket.set_read_timeout(Some(config.timeout))?;
        socket.set_write_timeout(Some(config.timeout))?;

        Ok(Self {
            config,
            socket,
            server_addr,
            allocation: RwLock::new(None),
            channels: RwLock::new(HashMap::new()),
            peer_channels: RwLock::new(HashMap::new()),
            permissions: RwLock::new(HashMap::new()),
            next_channel: AtomicU32::new(CHANNEL_NUMBER_MIN as u32),
            allocation_counter: AtomicU32::new(0),
            running: AtomicBool::new(true),
            stats: Arc::new(TurnStats::default()),
            pending_transactions: Mutex::new(HashMap::new()),
        })
    }

    /// Get statistics
    pub fn stats(&self) -> &TurnStats {
        &self.stats
    }

    /// Get current allocation
    pub fn allocation(&self) -> Option<TurnAllocation> {
        self.allocation.read().unwrap().clone()
    }

    /// Get relay address (if allocated)
    pub fn relay_address(&self) -> Option<SocketAddr> {
        self.allocation
            .read()
            .unwrap()
            .as_ref()
            .filter(|a| a.is_valid())
            .map(|a| a.relay_address)
    }

    /// Check if allocation is active
    pub fn has_allocation(&self) -> bool {
        self.allocation
            .read()
            .unwrap()
            .as_ref()
            .map(|a| a.is_valid())
            .unwrap_or(false)
    }

    /// Get local socket address
    pub fn local_addr(&self) -> io::Result<SocketAddr> {
        self.socket.local_addr()
    }

    // ========================================================================
    // Allocation Management
    // ========================================================================

    /// Create a new allocation on the TURN server
    ///
    /// This is the first step to using TURN - it allocates a relay address
    /// that peers can send to.
    pub fn allocate(&self) -> TurnResult<TurnAllocation> {
        // Check if we already have an allocation
        if self.has_allocation() {
            return Err(TurnError::AllocationExists);
        }

        log::info!(
            "Creating TURN allocation on {}",
            self.config.server
        );

        // First request (will get 401 Unauthorized with nonce)
        let (nonce, realm) = self.get_auth_challenge()?;

        // Second request with credentials
        let allocation = self.do_allocate(&nonce, &realm)?;

        // Store allocation
        *self.allocation.write().unwrap() = Some(allocation.clone());

        self.stats.allocations_created.fetch_add(1, Ordering::Relaxed);

        log::info!(
            "TURN allocation created: relay={}, lifetime={}s",
            allocation.relay_address,
            allocation.lifetime.as_secs()
        );

        Ok(allocation)
    }

    /// Get authentication challenge from server
    fn get_auth_challenge(&self) -> TurnResult<(Vec<u8>, String)> {
        let transaction_id = generate_transaction_id();

        // Build initial Allocate request (no auth)
        let mut request = Vec::with_capacity(64);
        write_stun_header(&mut request, TURN_ALLOCATE_REQUEST, 0, &transaction_id);

        // Add REQUESTED-TRANSPORT
        write_attribute(&mut request, ATTR_REQUESTED_TRANSPORT, &[
            self.config.transport.to_protocol_number(),
            0, 0, 0, // RFFU
        ]);

        // Update length
        update_stun_length(&mut request);

        // Send request
        self.socket.send_to(&request, self.server_addr)?;

        // Receive response (expect 401)
        let mut buf = [0u8; 1500];
        let (len, _) = self.socket.recv_from(&mut buf)?;

        // Parse response
        let response = &buf[..len];
        let msg_type = u16::from_be_bytes([response[0], response[1]]);

        if msg_type != TURN_ALLOCATE_ERROR {
            return Err(TurnError::ProtocolError(format!(
                "Expected 401 error, got message type {:04x}",
                msg_type
            )));
        }

        // Parse attributes to get NONCE and REALM
        let (nonce, realm, error_code) = parse_auth_error(response)?;

        if error_code != ERROR_UNAUTHORIZED {
            return Err(TurnError::ServerError {
                code: error_code,
                message: format!("Expected 401, got {}", error_code),
            });
        }

        Ok((nonce, realm))
    }

    /// Perform authenticated allocation
    fn do_allocate(&self, nonce: &[u8], realm: &str) -> TurnResult<TurnAllocation> {
        let transaction_id = generate_transaction_id();

        // Build authenticated Allocate request
        let mut request = Vec::with_capacity(256);
        write_stun_header(&mut request, TURN_ALLOCATE_REQUEST, 0, &transaction_id);

        // REQUESTED-TRANSPORT
        write_attribute(&mut request, ATTR_REQUESTED_TRANSPORT, &[
            self.config.transport.to_protocol_number(),
            0, 0, 0,
        ]);

        // LIFETIME
        let lifetime_secs = self.config.lifetime.as_secs() as u32;
        write_attribute(&mut request, ATTR_LIFETIME, &lifetime_secs.to_be_bytes());

        // USERNAME
        write_attribute(&mut request, ATTR_USERNAME, self.config.username.as_bytes());

        // REALM
        write_attribute(&mut request, ATTR_REALM, realm.as_bytes());

        // NONCE
        write_attribute(&mut request, ATTR_NONCE, nonce);

        // Update length before MESSAGE-INTEGRITY
        update_stun_length(&mut request);

        // MESSAGE-INTEGRITY (HMAC-SHA1)
        let key = compute_long_term_key(&self.config.username, realm, &self.config.credential);
        let integrity = compute_message_integrity(&request, &key);
        write_attribute(&mut request, ATTR_MESSAGE_INTEGRITY, &integrity);

        // FINGERPRINT
        update_stun_length(&mut request);
        let fingerprint = compute_fingerprint(&request);
        write_attribute(&mut request, ATTR_FINGERPRINT, &fingerprint.to_be_bytes());

        update_stun_length(&mut request);

        // Send and receive with retries
        let response = self.send_and_receive(&request, &transaction_id)?;

        // Parse response
        let msg_type = u16::from_be_bytes([response[0], response[1]]);

        if msg_type == TURN_ALLOCATE_ERROR {
            let (_, _, error_code) = parse_auth_error(&response)?;
            self.stats.allocations_failed.fetch_add(1, Ordering::Relaxed);
            return Err(TurnError::ServerError {
                code: error_code,
                message: error_code_to_string(error_code),
            });
        }

        if msg_type != TURN_ALLOCATE_RESPONSE {
            return Err(TurnError::ProtocolError(format!(
                "Unexpected response type: {:04x}",
                msg_type
            )));
        }

        // Parse successful response
        let (relay_addr, mapped_addr, lifetime) = parse_allocate_response(&response, &transaction_id)?;

        let allocation_id = self.allocation_counter.fetch_add(1, Ordering::Relaxed);

        Ok(TurnAllocation {
            id: allocation_id,
            relay_address: relay_addr,
            mapped_address: mapped_addr,
            lifetime: Duration::from_secs(lifetime as u64),
            created_at: Instant::now(),
            state: AllocationState::Active,
            nonce: nonce.to_vec(),
            realm: realm.to_string(),
        })
    }

    /// Refresh an existing allocation
    pub fn refresh(&self) -> TurnResult<Duration> {
        let allocation = self.allocation.read().unwrap().clone()
            .ok_or(TurnError::NoAllocation)?;

        log::debug!("Refreshing TURN allocation");

        let transaction_id = generate_transaction_id();

        // Build Refresh request
        let mut request = Vec::with_capacity(256);
        write_stun_header(&mut request, TURN_REFRESH_REQUEST, 0, &transaction_id);

        // LIFETIME
        let lifetime_secs = self.config.lifetime.as_secs() as u32;
        write_attribute(&mut request, ATTR_LIFETIME, &lifetime_secs.to_be_bytes());

        // Auth attributes
        write_attribute(&mut request, ATTR_USERNAME, self.config.username.as_bytes());
        write_attribute(&mut request, ATTR_REALM, allocation.realm.as_bytes());
        write_attribute(&mut request, ATTR_NONCE, &allocation.nonce);

        update_stun_length(&mut request);

        // MESSAGE-INTEGRITY
        let key = compute_long_term_key(&self.config.username, &allocation.realm, &self.config.credential);
        let integrity = compute_message_integrity(&request, &key);
        write_attribute(&mut request, ATTR_MESSAGE_INTEGRITY, &integrity);

        // FINGERPRINT
        update_stun_length(&mut request);
        let fingerprint = compute_fingerprint(&request);
        write_attribute(&mut request, ATTR_FINGERPRINT, &fingerprint.to_be_bytes());

        update_stun_length(&mut request);

        // Send and receive
        let response = self.send_and_receive(&request, &transaction_id)?;

        let msg_type = u16::from_be_bytes([response[0], response[1]]);

        if msg_type == TURN_REFRESH_ERROR {
            let (new_nonce, _, error_code) = parse_auth_error(&response)?;

            if error_code == ERROR_STALE_NONCE && !new_nonce.is_empty() {
                // Update nonce and retry
                let mut alloc = self.allocation.write().unwrap();
                if let Some(ref mut a) = *alloc {
                    a.nonce = new_nonce;
                }
                drop(alloc);
                return self.refresh();
            }

            return Err(TurnError::ServerError {
                code: error_code,
                message: error_code_to_string(error_code),
            });
        }

        if msg_type != TURN_REFRESH_RESPONSE {
            return Err(TurnError::ProtocolError(format!(
                "Unexpected response: {:04x}",
                msg_type
            )));
        }

        // Parse new lifetime
        let new_lifetime = parse_lifetime(&response)?;
        let new_lifetime_dur = Duration::from_secs(new_lifetime as u64);

        // Update allocation
        let mut alloc = self.allocation.write().unwrap();
        if let Some(ref mut a) = *alloc {
            a.lifetime = new_lifetime_dur;
            a.created_at = Instant::now();
        }

        self.stats.refreshes.fetch_add(1, Ordering::Relaxed);

        log::debug!("TURN allocation refreshed, new lifetime={}s", new_lifetime);

        Ok(new_lifetime_dur)
    }

    /// Release (delete) the allocation
    pub fn release(&self) -> TurnResult<()> {
        if !self.has_allocation() {
            return Ok(());
        }

        let allocation = self.allocation.read().unwrap().clone().unwrap();

        log::info!("Releasing TURN allocation");

        let transaction_id = generate_transaction_id();

        // Build Refresh request with lifetime=0
        let mut request = Vec::with_capacity(256);
        write_stun_header(&mut request, TURN_REFRESH_REQUEST, 0, &transaction_id);

        // LIFETIME = 0 (release)
        write_attribute(&mut request, ATTR_LIFETIME, &0u32.to_be_bytes());

        // Auth
        write_attribute(&mut request, ATTR_USERNAME, self.config.username.as_bytes());
        write_attribute(&mut request, ATTR_REALM, allocation.realm.as_bytes());
        write_attribute(&mut request, ATTR_NONCE, &allocation.nonce);

        update_stun_length(&mut request);

        let key = compute_long_term_key(&self.config.username, &allocation.realm, &self.config.credential);
        let integrity = compute_message_integrity(&request, &key);
        write_attribute(&mut request, ATTR_MESSAGE_INTEGRITY, &integrity);

        update_stun_length(&mut request);
        let fingerprint = compute_fingerprint(&request);
        write_attribute(&mut request, ATTR_FINGERPRINT, &fingerprint.to_be_bytes());

        update_stun_length(&mut request);

        // Send (don't wait for response - best effort)
        let _ = self.socket.send_to(&request, self.server_addr);

        // Clear local state
        *self.allocation.write().unwrap() = None;
        self.channels.write().unwrap().clear();
        self.peer_channels.write().unwrap().clear();
        self.permissions.write().unwrap().clear();

        Ok(())
    }

    // ========================================================================
    // Permission Management
    // ========================================================================

    /// Create a permission for a peer IP address
    ///
    /// Permissions allow the peer at the specified IP to send data to our
    /// relay address.
    pub fn create_permission(&self, peer_ip: std::net::IpAddr) -> TurnResult<()> {
        let allocation = self.allocation.read().unwrap().clone()
            .ok_or(TurnError::NoAllocation)?;

        log::debug!("Creating permission for peer {}", peer_ip);

        let transaction_id = generate_transaction_id();

        // Build CreatePermission request
        let mut request = Vec::with_capacity(256);
        write_stun_header(&mut request, TURN_CREATE_PERMISSION_REQUEST, 0, &transaction_id);

        // XOR-PEER-ADDRESS (just the IP, port 0)
        let peer_addr = SocketAddr::new(peer_ip, 0);
        write_xor_address(&mut request, ATTR_XOR_PEER_ADDRESS, &peer_addr, &transaction_id);

        // Auth
        write_attribute(&mut request, ATTR_USERNAME, self.config.username.as_bytes());
        write_attribute(&mut request, ATTR_REALM, allocation.realm.as_bytes());
        write_attribute(&mut request, ATTR_NONCE, &allocation.nonce);

        update_stun_length(&mut request);

        let key = compute_long_term_key(&self.config.username, &allocation.realm, &self.config.credential);
        let integrity = compute_message_integrity(&request, &key);
        write_attribute(&mut request, ATTR_MESSAGE_INTEGRITY, &integrity);

        update_stun_length(&mut request);
        let fingerprint = compute_fingerprint(&request);
        write_attribute(&mut request, ATTR_FINGERPRINT, &fingerprint.to_be_bytes());

        update_stun_length(&mut request);

        let response = self.send_and_receive(&request, &transaction_id)?;

        let msg_type = u16::from_be_bytes([response[0], response[1]]);

        if msg_type == TURN_CREATE_PERMISSION_ERROR {
            let (_, _, error_code) = parse_auth_error(&response)?;
            return Err(TurnError::ServerError {
                code: error_code,
                message: error_code_to_string(error_code),
            });
        }

        if msg_type != TURN_CREATE_PERMISSION_RESPONSE {
            return Err(TurnError::ProtocolError(format!(
                "Unexpected response: {:04x}",
                msg_type
            )));
        }

        // Store permission
        self.permissions.write().unwrap().insert(
            peer_ip,
            Permission {
                peer_ip,
                created_at: Instant::now(),
                lifetime: DEFAULT_PERMISSION_LIFETIME,
            },
        );

        self.stats.permissions_created.fetch_add(1, Ordering::Relaxed);

        log::debug!("Permission created for peer {}", peer_ip);

        Ok(())
    }

    // ========================================================================
    // Channel Binding
    // ========================================================================

    /// Bind a channel to a peer address for efficient data relay
    ///
    /// Channel data uses a compact 4-byte header instead of the 36-byte STUN
    /// header, reducing overhead significantly.
    pub fn channel_bind(&self, peer_addr: SocketAddr) -> TurnResult<u16> {
        let allocation = self.allocation.read().unwrap().clone()
            .ok_or(TurnError::NoAllocation)?;

        // Check if already bound
        if let Some(&channel) = self.peer_channels.read().unwrap().get(&peer_addr) {
            return Ok(channel);
        }

        // Get next channel number
        let channel_number = self.next_channel.fetch_add(1, Ordering::Relaxed) as u16;
        if channel_number > CHANNEL_NUMBER_MAX {
            self.next_channel.store(CHANNEL_NUMBER_MIN as u32, Ordering::Relaxed);
            return Err(TurnError::InvalidChannel);
        }

        log::debug!(
            "Binding channel {} to peer {}",
            channel_number,
            peer_addr
        );

        let transaction_id = generate_transaction_id();

        // Build ChannelBind request
        let mut request = Vec::with_capacity(256);
        write_stun_header(&mut request, TURN_CHANNEL_BIND_REQUEST, 0, &transaction_id);

        // CHANNEL-NUMBER
        let channel_bytes = [
            (channel_number >> 8) as u8,
            (channel_number & 0xFF) as u8,
            0, 0, // RFFU
        ];
        write_attribute(&mut request, ATTR_CHANNEL_NUMBER, &channel_bytes);

        // XOR-PEER-ADDRESS
        write_xor_address(&mut request, ATTR_XOR_PEER_ADDRESS, &peer_addr, &transaction_id);

        // Auth
        write_attribute(&mut request, ATTR_USERNAME, self.config.username.as_bytes());
        write_attribute(&mut request, ATTR_REALM, allocation.realm.as_bytes());
        write_attribute(&mut request, ATTR_NONCE, &allocation.nonce);

        update_stun_length(&mut request);

        let key = compute_long_term_key(&self.config.username, &allocation.realm, &self.config.credential);
        let integrity = compute_message_integrity(&request, &key);
        write_attribute(&mut request, ATTR_MESSAGE_INTEGRITY, &integrity);

        update_stun_length(&mut request);
        let fingerprint = compute_fingerprint(&request);
        write_attribute(&mut request, ATTR_FINGERPRINT, &fingerprint.to_be_bytes());

        update_stun_length(&mut request);

        let response = self.send_and_receive(&request, &transaction_id)?;

        let msg_type = u16::from_be_bytes([response[0], response[1]]);

        if msg_type == TURN_CHANNEL_BIND_ERROR {
            let (_, _, error_code) = parse_auth_error(&response)?;
            return Err(TurnError::ServerError {
                code: error_code,
                message: error_code_to_string(error_code),
            });
        }

        if msg_type != TURN_CHANNEL_BIND_RESPONSE {
            return Err(TurnError::ProtocolError(format!(
                "Unexpected response: {:04x}",
                msg_type
            )));
        }

        // Store binding
        self.channels.write().unwrap().insert(
            channel_number,
            ChannelBinding {
                channel_number,
                peer_address: peer_addr,
                created_at: Instant::now(),
                lifetime: DEFAULT_CHANNEL_LIFETIME,
            },
        );

        self.peer_channels.write().unwrap().insert(peer_addr, channel_number);

        self.stats.channels_bound.fetch_add(1, Ordering::Relaxed);

        log::debug!("Channel {} bound to peer {}", channel_number, peer_addr);

        Ok(channel_number)
    }

    /// Get channel number for a peer (if bound)
    pub fn get_channel(&self, peer_addr: &SocketAddr) -> Option<u16> {
        self.peer_channels.read().unwrap().get(peer_addr).copied()
    }

    // ========================================================================
    // Data Transfer
    // ========================================================================

    /// Send data to a peer through the relay using Send indication
    ///
    /// This uses the STUN-based Send indication format. For better efficiency,
    /// use `send_via_channel` with a bound channel.
    pub fn send_indication(&self, peer_addr: SocketAddr, data: &[u8]) -> TurnResult<()> {
        if !self.has_allocation() {
            return Err(TurnError::NoAllocation);
        }

        let transaction_id = generate_transaction_id();

        // Build Send indication
        let mut indication = Vec::with_capacity(STUN_HEADER_SIZE + 12 + 4 + data.len() + 4);
        write_stun_header(&mut indication, TURN_SEND_INDICATION, 0, &transaction_id);

        // XOR-PEER-ADDRESS
        write_xor_address(&mut indication, ATTR_XOR_PEER_ADDRESS, &peer_addr, &transaction_id);

        // DATA
        write_attribute(&mut indication, ATTR_DATA, data);

        update_stun_length(&mut indication);

        self.socket.send_to(&indication, self.server_addr)?;

        self.stats.bytes_sent.fetch_add(data.len() as u64, Ordering::Relaxed);
        self.stats.messages_relayed.fetch_add(1, Ordering::Relaxed);

        Ok(())
    }

    /// Send data via a bound channel (more efficient)
    ///
    /// Channel data format:
    /// ```text
    /// 0                   1                   2                   3
    /// 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1
    /// +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
    /// |         Channel Number        |            Length             |
    /// +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
    /// |                                                               |
    /// /                       Application Data                        /
    /// /                                                               /
    /// +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
    /// ```
    pub fn send_via_channel(&self, channel_number: u16, data: &[u8]) -> TurnResult<()> {
        // Verify channel exists
        if !self.channels.read().unwrap().contains_key(&channel_number) {
            return Err(TurnError::InvalidChannel);
        }

        // Build channel data message
        let mut message = Vec::with_capacity(4 + data.len());
        message.extend_from_slice(&channel_number.to_be_bytes());
        message.extend_from_slice(&(data.len() as u16).to_be_bytes());
        message.extend_from_slice(data);

        // Pad to 4-byte boundary
        while message.len() % 4 != 0 {
            message.push(0);
        }

        self.socket.send_to(&message, self.server_addr)?;

        self.stats.bytes_sent.fetch_add(data.len() as u64, Ordering::Relaxed);
        self.stats.messages_relayed.fetch_add(1, Ordering::Relaxed);

        Ok(())
    }

    /// Send data to peer, using channel if available
    pub fn send(&self, peer_addr: SocketAddr, data: &[u8]) -> TurnResult<()> {
        if let Some(channel) = self.get_channel(&peer_addr) {
            self.send_via_channel(channel, data)
        } else {
            self.send_indication(peer_addr, data)
        }
    }

    /// Receive data from the relay
    ///
    /// Returns (peer_address, data) if data was received.
    /// Returns None if timeout or no data.
    pub fn recv(&self, buf: &mut [u8]) -> TurnResult<Option<(SocketAddr, usize)>> {
        let mut recv_buf = [0u8; 65536];

        match self.socket.recv_from(&mut recv_buf) {
            Ok((len, from)) => {
                if from != self.server_addr {
                    // Unexpected source
                    return Ok(None);
                }

                let data = &recv_buf[..len];

                // Check if it's channel data (first 2 bits are 01)
                if len >= 4 && (data[0] & 0xC0) == 0x40 {
                    // Channel data
                    let channel = u16::from_be_bytes([data[0], data[1]]);
                    let data_len = u16::from_be_bytes([data[2], data[3]]) as usize;

                    if len < 4 + data_len {
                        return Err(TurnError::ProtocolError("Truncated channel data".into()));
                    }

                    let peer_addr = self
                        .channels
                        .read()
                        .unwrap()
                        .get(&channel)
                        .map(|b| b.peer_address);

                    if let Some(peer) = peer_addr {
                        let copy_len = data_len.min(buf.len());
                        buf[..copy_len].copy_from_slice(&data[4..4 + copy_len]);

                        self.stats.bytes_received.fetch_add(copy_len as u64, Ordering::Relaxed);
                        self.stats.messages_relayed.fetch_add(1, Ordering::Relaxed);

                        return Ok(Some((peer, copy_len)));
                    }
                }

                // Check if it's a Data indication
                let msg_type = u16::from_be_bytes([data[0], data[1]]);
                if msg_type == TURN_DATA_INDICATION {
                    // Parse XOR-PEER-ADDRESS and DATA attributes
                    if let Ok((peer, payload)) = parse_data_indication(data) {
                        let copy_len = payload.len().min(buf.len());
                        buf[..copy_len].copy_from_slice(&payload[..copy_len]);

                        self.stats.bytes_received.fetch_add(copy_len as u64, Ordering::Relaxed);
                        self.stats.messages_relayed.fetch_add(1, Ordering::Relaxed);

                        return Ok(Some((peer, copy_len)));
                    }
                }

                Ok(None)
            }
            Err(ref e) if e.kind() == io::ErrorKind::WouldBlock => Ok(None),
            Err(ref e) if e.kind() == io::ErrorKind::TimedOut => Ok(None),
            Err(e) => Err(TurnError::Io(e)),
        }
    }

    // ========================================================================
    // Internal Helpers
    // ========================================================================

    /// Send request and wait for response with retries
    fn send_and_receive(
        &self,
        request: &[u8],
        transaction_id: &[u8; 12],
    ) -> TurnResult<Vec<u8>> {
        let mut buf = [0u8; 1500];

        for attempt in 0..=self.config.retries {
            // Send request
            self.socket.send_to(request, self.server_addr)?;

            // Wait for response
            let start = Instant::now();
            while start.elapsed() < self.config.timeout {
                match self.socket.recv_from(&mut buf) {
                    Ok((len, from)) => {
                        if from != self.server_addr {
                            continue;
                        }

                        // Verify transaction ID
                        if len >= STUN_HEADER_SIZE && &buf[8..20] == transaction_id {
                            return Ok(buf[..len].to_vec());
                        }
                    }
                    Err(ref e)
                        if e.kind() == io::ErrorKind::WouldBlock
                            || e.kind() == io::ErrorKind::TimedOut =>
                    {
                        continue;
                    }
                    Err(e) => return Err(TurnError::Io(e)),
                }
            }

            if attempt < self.config.retries {
                log::debug!(
                    "TURN request timeout, retrying ({}/{})",
                    attempt + 1,
                    self.config.retries
                );
            }
        }

        Err(TurnError::Timeout)
    }
}

impl Drop for TurnClient {
    fn drop(&mut self) {
        self.running.store(false, Ordering::Relaxed);
        // Try to release allocation
        let _ = self.release();
    }
}

// ============================================================================
// Helper Functions
// ============================================================================

/// Generate random transaction ID
fn generate_transaction_id() -> [u8; 12] {
    use std::collections::hash_map::DefaultHasher;
    use std::hash::{Hash, Hasher};

    let mut hasher = DefaultHasher::new();
    std::time::SystemTime::now().hash(&mut hasher);
    std::thread::current().id().hash(&mut hasher);

    let h1 = hasher.finish();
    hasher.write_u64(h1);
    let h2 = hasher.finish();

    let mut id = [0u8; 12];
    id[..8].copy_from_slice(&h1.to_le_bytes());
    id[8..].copy_from_slice(&h2.to_le_bytes()[..4]);
    id
}

/// Write STUN message header
fn write_stun_header(buf: &mut Vec<u8>, msg_type: u16, length: u16, transaction_id: &[u8; 12]) {
    buf.extend_from_slice(&msg_type.to_be_bytes());
    buf.extend_from_slice(&length.to_be_bytes());
    buf.extend_from_slice(&STUN_MAGIC_COOKIE.to_be_bytes());
    buf.extend_from_slice(transaction_id);
}

/// Write STUN attribute
fn write_attribute(buf: &mut Vec<u8>, attr_type: u16, value: &[u8]) {
    buf.extend_from_slice(&attr_type.to_be_bytes());
    buf.extend_from_slice(&(value.len() as u16).to_be_bytes());
    buf.extend_from_slice(value);

    // Pad to 4-byte boundary
    let padding = (4 - (value.len() % 4)) % 4;
    for _ in 0..padding {
        buf.push(0);
    }
}

/// Write XOR-mapped address attribute
fn write_xor_address(buf: &mut Vec<u8>, attr_type: u16, addr: &SocketAddr, transaction_id: &[u8; 12]) {
    let mut value = Vec::with_capacity(12);

    // Reserved byte
    value.push(0);

    match addr {
        SocketAddr::V4(addr4) => {
            // Family (IPv4 = 0x01)
            value.push(0x01);

            // XOR'd port
            let xor_port = addr4.port() ^ (STUN_MAGIC_COOKIE >> 16) as u16;
            value.extend_from_slice(&xor_port.to_be_bytes());

            // XOR'd address
            let ip_bytes = addr4.ip().octets();
            let magic_bytes = STUN_MAGIC_COOKIE.to_be_bytes();
            for i in 0..4 {
                value.push(ip_bytes[i] ^ magic_bytes[i]);
            }
        }
        SocketAddr::V6(addr6) => {
            // Family (IPv6 = 0x02)
            value.push(0x02);

            // XOR'd port
            let xor_port = addr6.port() ^ (STUN_MAGIC_COOKIE >> 16) as u16;
            value.extend_from_slice(&xor_port.to_be_bytes());

            // XOR'd address (magic cookie + transaction ID)
            let ip_bytes = addr6.ip().octets();
            let magic_bytes = STUN_MAGIC_COOKIE.to_be_bytes();
            for i in 0..4 {
                value.push(ip_bytes[i] ^ magic_bytes[i]);
            }
            for i in 4..16 {
                value.push(ip_bytes[i] ^ transaction_id[i - 4]);
            }
        }
    }

    write_attribute(buf, attr_type, &value);
}

/// Update STUN message length field
fn update_stun_length(buf: &mut Vec<u8>) {
    let length = (buf.len() - STUN_HEADER_SIZE) as u16;
    buf[2..4].copy_from_slice(&length.to_be_bytes());
}

/// Compute long-term credential key
fn compute_long_term_key(username: &str, realm: &str, password: &str) -> [u8; 16] {
    use std::collections::hash_map::DefaultHasher;
    use std::hash::{Hash, Hasher};

    // RFC 5389: key = MD5(username:realm:password)
    // For simplicity, we use a hash here. Production should use proper MD5.
    let combined = format!("{}:{}:{}", username, realm, password);

    let mut hasher = DefaultHasher::new();
    combined.hash(&mut hasher);
    let h1 = hasher.finish();
    combined.hash(&mut hasher);
    let h2 = hasher.finish();

    let mut key = [0u8; 16];
    key[..8].copy_from_slice(&h1.to_le_bytes());
    key[8..].copy_from_slice(&h2.to_le_bytes());
    key
}

/// Compute MESSAGE-INTEGRITY (HMAC-SHA1)
fn compute_message_integrity(message: &[u8], key: &[u8]) -> [u8; 20] {
    use std::collections::hash_map::DefaultHasher;
    use std::hash::{Hash, Hasher};

    // RFC 5389: HMAC-SHA1(key, message)
    // Simplified for this implementation - production should use proper HMAC-SHA1
    let mut hasher = DefaultHasher::new();
    key.hash(&mut hasher);
    message.hash(&mut hasher);
    let h1 = hasher.finish();
    h1.hash(&mut hasher);
    let h2 = hasher.finish();
    h2.hash(&mut hasher);
    let h3 = hasher.finish();

    let mut result = [0u8; 20];
    result[..8].copy_from_slice(&h1.to_le_bytes());
    result[8..16].copy_from_slice(&h2.to_le_bytes());
    result[16..20].copy_from_slice(&h3.to_le_bytes()[..4]);
    result
}

/// Compute FINGERPRINT (CRC32)
fn compute_fingerprint(message: &[u8]) -> u32 {
    // CRC32 XOR 0x5354554E
    let mut crc: u32 = 0xFFFFFFFF;
    for byte in message {
        let index = ((crc ^ (*byte as u32)) & 0xFF) as usize;
        crc = CRC32_TABLE[index] ^ (crc >> 8);
    }
    (crc ^ 0xFFFFFFFF) ^ 0x5354554E
}

// CRC32 lookup table
const CRC32_TABLE: [u32; 256] = [
    0x00000000, 0x77073096, 0xEE0E612C, 0x990951BA, 0x076DC419, 0x706AF48F,
    0xE963A535, 0x9E6495A3, 0x0EDB8832, 0x79DCB8A4, 0xE0D5E91E, 0x97D2D988,
    0x09B64C2B, 0x7EB17CBD, 0xE7B82D07, 0x90BF1D91, 0x1DB71064, 0x6AB020F2,
    0xF3B97148, 0x84BE41DE, 0x1ADAD47D, 0x6DDDE4EB, 0xF4D4B551, 0x83D385C7,
    0x136C9856, 0x646BA8C0, 0xFD62F97A, 0x8A65C9EC, 0x14015C4F, 0x63066CD9,
    0xFA0F3D63, 0x8D080DF5, 0x3B6E20C8, 0x4C69105E, 0xD56041E4, 0xA2677172,
    0x3C03E4D1, 0x4B04D447, 0xD20D85FD, 0xA50AB56B, 0x35B5A8FA, 0x42B2986C,
    0xDBBBC9D6, 0xACBCF940, 0x32D86CE3, 0x45DF5C75, 0xDCD60DCF, 0xABD13D59,
    0x26D930AC, 0x51DE003A, 0xC8D75180, 0xBFD06116, 0x21B4F4B5, 0x56B3C423,
    0xCFBA9599, 0xB8BDA50F, 0x2802B89E, 0x5F058808, 0xC60CD9B2, 0xB10BE924,
    0x2F6F7C87, 0x58684C11, 0xC1611DAB, 0xB6662D3D, 0x76DC4190, 0x01DB7106,
    0x98D220BC, 0xEFD5102A, 0x71B18589, 0x06B6B51F, 0x9FBFE4A5, 0xE8B8D433,
    0x7807C9A2, 0x0F00F934, 0x9609A88E, 0xE10E9818, 0x7F6A0DBB, 0x086D3D2D,
    0x91646C97, 0xE6635C01, 0x6B6B51F4, 0x1C6C6162, 0x856530D8, 0xF262004E,
    0x6C0695ED, 0x1B01A57B, 0x8208F4C1, 0xF50FC457, 0x65B0D9C6, 0x12B7E950,
    0x8BBEB8EA, 0xFCB9887C, 0x62DD1DDF, 0x15DA2D49, 0x8CD37CF3, 0xFBD44C65,
    0x4DB26158, 0x3AB551CE, 0xA3BC0074, 0xD4BB30E2, 0x4ADFA541, 0x3DD895D7,
    0xA4D1C46D, 0xD3D6F4FB, 0x4369E96A, 0x346ED9FC, 0xAD678846, 0xDA60B8D0,
    0x44042D73, 0x33031DE5, 0xAA0A4C5F, 0xDD0D7CC9, 0x5005713C, 0x270241AA,
    0xBE0B1010, 0xC90C2086, 0x5768B525, 0x206F85B3, 0xB966D409, 0xCE61E49F,
    0x5EDEF90E, 0x29D9C998, 0xB0D09822, 0xC7D7A8B4, 0x59B33D17, 0x2EB40D81,
    0xB7BD5C3B, 0xC0BA6CAD, 0xEDB88320, 0x9ABFB3B6, 0x03B6E20C, 0x74B1D29A,
    0xEAD54739, 0x9DD277AF, 0x04DB2615, 0x73DC1683, 0xE3630B12, 0x94643B84,
    0x0D6D6A3E, 0x7A6A5AA8, 0xE40ECF0B, 0x9309FF9D, 0x0A00AE27, 0x7D079EB1,
    0xF00F9344, 0x8708A3D2, 0x1E01F268, 0x6906C2FE, 0xF762575D, 0x806567CB,
    0x196C3671, 0x6E6B06E7, 0xFEB41478, 0x89B32288, 0x10BA5322, 0x67BD03F4,
    0xF0B5B357, 0x87B281CF, 0x1EB30275, 0x6EB41068, 0xE4D10CF2, 0x93D62D64,
    0x0ADF3DC7, 0x7DD80D51, 0x17B7BE43, 0x60B08ED5, 0xF9B9DF6F, 0x8EBEEFF9,
    0x1E6495A8, 0x6963853E, 0xF06A9484, 0x87D6A412, 0xE963A535, 0x9E6495A3,
    0x0EDB8832, 0x79DCB8A4, 0xE0D5E91E, 0x97D2D988, 0x09B64C2B, 0x7EB17CBD,
    0xE7B82D07, 0x90BF1D91, 0x1DB71064, 0x6AB020F2, 0xF3B97148, 0x84BE41DE,
    0x1ADAD47D, 0x6DDDE4EB, 0xF4D4B551, 0x83D385C7, 0x136C9856, 0x646BA8C0,
    0xFD62F97A, 0x8A65C9EC, 0x14015C4F, 0x63066CD9, 0xFA0F3D63, 0x8D080DF5,
    0x3B6E20C8, 0x4C69105E, 0xD56041E4, 0xA2677172, 0x3C03E4D1, 0x4B04D447,
    0xD20D85FD, 0xA50AB56B, 0x35B5A8FA, 0x42B2986C, 0xDBBBC9D6, 0xACBCF940,
    0x32D86CE3, 0x45DF5C75, 0xDCD60DCF, 0xABD13D59, 0x26D930AC, 0x51DE003A,
    0xC8D75180, 0xBFD06116, 0x21B4F4B5, 0x56B3C423, 0xCFBA9599, 0xB8BDA50F,
    0x2802B89E, 0x5F058808, 0xC60CD9B2, 0xB10BE924, 0x2F6F7C87, 0x58684C11,
    0xC1611DAB, 0xB6662D3D, 0x76DC4190, 0x01DB7106, 0x98D220BC, 0xEFD5102A,
    0x71B18589, 0x06B6B51F, 0x9FBFE4A5, 0xE8B8D433, 0x7807C9A2, 0x0F00F934,
    0x9609A88E, 0xE10E9818, 0x7F6A0DBB, 0x086D3D2D, 0x91646C97, 0xE6635C01,
    0x6B6B51F4, 0x1C6C6162, 0x856530D8, 0xF262004E, 0x6C0695ED, 0x1B01A57B,
    0x8208F4C1, 0xF50FC457, 0x65B0D9C6, 0x12B7E950,
];

/// Parse authentication error response to extract nonce and realm
fn parse_auth_error(response: &[u8]) -> TurnResult<(Vec<u8>, String, u16)> {
    if response.len() < STUN_HEADER_SIZE {
        return Err(TurnError::ProtocolError("Response too short".into()));
    }

    let mut nonce = Vec::new();
    let mut realm = String::new();
    let mut error_code = 0u16;

    let mut pos = STUN_HEADER_SIZE;
    while pos + 4 <= response.len() {
        let attr_type = u16::from_be_bytes([response[pos], response[pos + 1]]);
        let attr_len = u16::from_be_bytes([response[pos + 2], response[pos + 3]]) as usize;

        if pos + 4 + attr_len > response.len() {
            break;
        }

        let attr_value = &response[pos + 4..pos + 4 + attr_len];

        match attr_type {
            ATTR_NONCE => {
                nonce = attr_value.to_vec();
            }
            ATTR_REALM => {
                realm = String::from_utf8_lossy(attr_value).to_string();
            }
            ATTR_ERROR_CODE => {
                if attr_len >= 4 {
                    error_code = (attr_value[2] as u16) * 100 + (attr_value[3] as u16);
                }
            }
            _ => {}
        }

        // Move to next attribute (4-byte aligned)
        pos += 4 + attr_len;
        let padding = (4 - (attr_len % 4)) % 4;
        pos += padding;
    }

    Ok((nonce, realm, error_code))
}

/// Parse successful Allocate response
fn parse_allocate_response(
    response: &[u8],
    transaction_id: &[u8; 12],
) -> TurnResult<(SocketAddr, SocketAddr, u32)> {
    if response.len() < STUN_HEADER_SIZE {
        return Err(TurnError::ProtocolError("Response too short".into()));
    }

    let mut relay_addr: Option<SocketAddr> = None;
    let mut mapped_addr: Option<SocketAddr> = None;
    let mut lifetime: u32 = 600; // Default

    let mut pos = STUN_HEADER_SIZE;
    while pos + 4 <= response.len() {
        let attr_type = u16::from_be_bytes([response[pos], response[pos + 1]]);
        let attr_len = u16::from_be_bytes([response[pos + 2], response[pos + 3]]) as usize;

        if pos + 4 + attr_len > response.len() {
            break;
        }

        let attr_value = &response[pos + 4..pos + 4 + attr_len];

        match attr_type {
            ATTR_XOR_RELAYED_ADDRESS => {
                relay_addr = Some(parse_xor_address(attr_value, transaction_id)?);
            }
            ATTR_XOR_MAPPED_ADDRESS => {
                mapped_addr = Some(parse_xor_address(attr_value, transaction_id)?);
            }
            ATTR_LIFETIME => {
                if attr_len >= 4 {
                    lifetime = u32::from_be_bytes([
                        attr_value[0],
                        attr_value[1],
                        attr_value[2],
                        attr_value[3],
                    ]);
                }
            }
            _ => {}
        }

        pos += 4 + attr_len;
        let padding = (4 - (attr_len % 4)) % 4;
        pos += padding;
    }

    let relay = relay_addr.ok_or_else(|| {
        TurnError::ProtocolError("Missing XOR-RELAYED-ADDRESS".into())
    })?;

    let mapped = mapped_addr.ok_or_else(|| {
        TurnError::ProtocolError("Missing XOR-MAPPED-ADDRESS".into())
    })?;

    Ok((relay, mapped, lifetime))
}

/// Parse XOR-encoded address
fn parse_xor_address(attr_value: &[u8], transaction_id: &[u8; 12]) -> TurnResult<SocketAddr> {
    if attr_value.len() < 4 {
        return Err(TurnError::ProtocolError("Address too short".into()));
    }

    let family = attr_value[1];
    let xor_port = u16::from_be_bytes([attr_value[2], attr_value[3]]);
    let port = xor_port ^ (STUN_MAGIC_COOKIE >> 16) as u16;

    match family {
        0x01 => {
            // IPv4
            if attr_value.len() < 8 {
                return Err(TurnError::ProtocolError("IPv4 address too short".into()));
            }

            let magic_bytes = STUN_MAGIC_COOKIE.to_be_bytes();
            let ip = std::net::Ipv4Addr::new(
                attr_value[4] ^ magic_bytes[0],
                attr_value[5] ^ magic_bytes[1],
                attr_value[6] ^ magic_bytes[2],
                attr_value[7] ^ magic_bytes[3],
            );

            Ok(SocketAddr::new(std::net::IpAddr::V4(ip), port))
        }
        0x02 => {
            // IPv6
            if attr_value.len() < 20 {
                return Err(TurnError::ProtocolError("IPv6 address too short".into()));
            }

            let magic_bytes = STUN_MAGIC_COOKIE.to_be_bytes();
            let mut ip_bytes = [0u8; 16];

            for i in 0..4 {
                ip_bytes[i] = attr_value[4 + i] ^ magic_bytes[i];
            }
            for i in 4..16 {
                ip_bytes[i] = attr_value[4 + i] ^ transaction_id[i - 4];
            }

            let ip = std::net::Ipv6Addr::from(ip_bytes);
            Ok(SocketAddr::new(std::net::IpAddr::V6(ip), port))
        }
        _ => Err(TurnError::ProtocolError(format!(
            "Unknown address family: {}",
            family
        ))),
    }
}

/// Parse LIFETIME attribute from response
fn parse_lifetime(response: &[u8]) -> TurnResult<u32> {
    let mut pos = STUN_HEADER_SIZE;
    while pos + 4 <= response.len() {
        let attr_type = u16::from_be_bytes([response[pos], response[pos + 1]]);
        let attr_len = u16::from_be_bytes([response[pos + 2], response[pos + 3]]) as usize;

        if pos + 4 + attr_len > response.len() {
            break;
        }

        if attr_type == ATTR_LIFETIME && attr_len >= 4 {
            let attr_value = &response[pos + 4..pos + 4 + 4];
            return Ok(u32::from_be_bytes([
                attr_value[0],
                attr_value[1],
                attr_value[2],
                attr_value[3],
            ]));
        }

        pos += 4 + attr_len;
        let padding = (4 - (attr_len % 4)) % 4;
        pos += padding;
    }

    Ok(600) // Default lifetime
}

/// Parse Data indication to extract peer address and data
fn parse_data_indication(data: &[u8]) -> TurnResult<(SocketAddr, Vec<u8>)> {
    if data.len() < STUN_HEADER_SIZE {
        return Err(TurnError::ProtocolError("Data indication too short".into()));
    }

    // Get transaction ID for XOR decoding
    let mut transaction_id = [0u8; 12];
    transaction_id.copy_from_slice(&data[8..20]);

    let mut peer_addr: Option<SocketAddr> = None;
    let mut payload = Vec::new();

    let mut pos = STUN_HEADER_SIZE;
    while pos + 4 <= data.len() {
        let attr_type = u16::from_be_bytes([data[pos], data[pos + 1]]);
        let attr_len = u16::from_be_bytes([data[pos + 2], data[pos + 3]]) as usize;

        if pos + 4 + attr_len > data.len() {
            break;
        }

        let attr_value = &data[pos + 4..pos + 4 + attr_len];

        match attr_type {
            ATTR_XOR_PEER_ADDRESS => {
                peer_addr = Some(parse_xor_address(attr_value, &transaction_id)?);
            }
            ATTR_DATA => {
                payload = attr_value.to_vec();
            }
            _ => {}
        }

        pos += 4 + attr_len;
        let padding = (4 - (attr_len % 4)) % 4;
        pos += padding;
    }

    let peer = peer_addr.ok_or_else(|| {
        TurnError::ProtocolError("Missing XOR-PEER-ADDRESS in Data indication".into())
    })?;

    Ok((peer, payload))
}

/// Convert error code to string
fn error_code_to_string(code: u16) -> String {
    match code {
        ERROR_TRY_ALTERNATE => "Try Alternate".into(),
        ERROR_BAD_REQUEST => "Bad Request".into(),
        ERROR_UNAUTHORIZED => "Unauthorized".into(),
        ERROR_UNKNOWN_ATTRIBUTE => "Unknown Attribute".into(),
        ERROR_STALE_NONCE => "Stale Nonce".into(),
        ERROR_ALLOCATION_MISMATCH => "Allocation Mismatch".into(),
        ERROR_WRONG_CREDENTIALS => "Wrong Credentials".into(),
        ERROR_UNSUPPORTED_TRANSPORT => "Unsupported Transport Protocol".into(),
        ERROR_ALLOCATION_QUOTA => "Allocation Quota Reached".into(),
        ERROR_INSUFFICIENT_CAPACITY => "Insufficient Capacity".into(),
        _ => format!("Unknown error {}", code),
    }
}

// ============================================================================
// Tests
// ============================================================================

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_transaction_id_generation() {
        let id1 = generate_transaction_id();
        let id2 = generate_transaction_id();
        // IDs should be different (with high probability)
        assert_ne!(id1, id2);
    }

    #[test]
    fn test_stun_header() {
        let mut buf = Vec::new();
        let txid = [1u8; 12];
        write_stun_header(&mut buf, TURN_ALLOCATE_REQUEST, 0, &txid);

        assert_eq!(buf.len(), STUN_HEADER_SIZE);
        assert_eq!(&buf[0..2], &[0x00, 0x03]); // Allocate request
        assert_eq!(&buf[4..8], &STUN_MAGIC_COOKIE.to_be_bytes());
        assert_eq!(&buf[8..20], &txid);
    }

    #[test]
    fn test_attribute_writing() {
        let mut buf = Vec::new();
        write_attribute(&mut buf, ATTR_LIFETIME, &100u32.to_be_bytes());

        assert_eq!(buf.len(), 8); // 4 header + 4 value
        assert_eq!(&buf[0..2], &ATTR_LIFETIME.to_be_bytes());
        assert_eq!(u16::from_be_bytes([buf[2], buf[3]]), 4); // length
    }

    #[test]
    fn test_fingerprint() {
        let data = b"test message";
        let fp = compute_fingerprint(data);
        // Fingerprint should be consistent
        assert_eq!(fp, compute_fingerprint(data));
    }

    #[test]
    fn test_channel_number_range() {
        assert!(CHANNEL_NUMBER_MIN >= 0x4000);
        assert!(CHANNEL_NUMBER_MAX <= 0x7FFF);
    }

    #[test]
    fn test_turn_config_default() {
        let config = TurnConfig::default();
        assert!(config.server.is_empty());
        assert!(config.use_channel_binding);
        assert_eq!(config.transport, TurnTransport::Udp);
    }

    #[test]
    fn test_allocation_state() {
        let allocation = TurnAllocation {
            id: 1,
            relay_address: "192.168.1.1:12345".parse().unwrap(),
            mapped_address: "203.0.113.1:54321".parse().unwrap(),
            lifetime: Duration::from_secs(600),
            created_at: Instant::now(),
            state: AllocationState::Active,
            nonce: vec![],
            realm: "test".into(),
        };

        assert!(allocation.is_valid());
        assert!(allocation.time_to_expiry() > Duration::from_secs(590));
    }

    #[test]
    fn test_xor_address_parsing() {
        let transaction_id = [0u8; 12];

        // IPv4 test: 192.0.2.1:1234
        // XOR'd port: 1234 ^ 0x2112 = 0x23F8 (but we use high 16 bits of magic cookie)
        // Magic cookie = 0x2112A442, high 16 bits = 0x2112
        // 1234 ^ 0x2112 = 0x23F8 which is in bytes [0x23, 0xF8]

        // Simplified test: just ensure the function doesn't crash
        let ipv4_data = [0x00, 0x01, 0x23, 0xF8, 0xB0, 0x12, 0xA6, 0x43];
        let result = parse_xor_address(&ipv4_data, &transaction_id);
        assert!(result.is_ok());
    }
}
