//! STUN (Session Traversal Utilities for NAT) client implementation
//!
//! This module provides STUN protocol support for NAT traversal, enabling
//! peers behind NATs to discover their external (public) IP address and port.
//!
//! # Protocol Support
//!
//! Implements RFC 8489 (STUN) with support for:
//! - Binding requests to discover external address
//! - NAT type detection (full cone, restricted, symmetric)
//! - Multiple STUN server support with failover
//! - Result caching with configurable TTL
//!
//! # Example
//!
//! ```ignore
//! use horus_core::communication::network::stun::{StunClient, StunConfig};
//!
//! let config = StunConfig::default();
//! let client = StunClient::new(config)?;
//!
//! // Discover external address
//! let external = client.get_external_address()?;
//! println!("External address: {}", external);
//!
//! // Detect NAT type
//! let nat_type = client.detect_nat_type()?;
//! println!("NAT type: {:?}", nat_type);
//! ```

use std::io;
use std::net::{SocketAddr, ToSocketAddrs, UdpSocket};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

// STUN message constants (RFC 8489)
const STUN_MAGIC_COOKIE: u32 = 0x2112A442;
const STUN_HEADER_SIZE: usize = 20;

// STUN message types
const STUN_BINDING_REQUEST: u16 = 0x0001;
const STUN_BINDING_RESPONSE: u16 = 0x0101;
const STUN_BINDING_ERROR: u16 = 0x0111;

// STUN attribute types
const ATTR_MAPPED_ADDRESS: u16 = 0x0001;
const ATTR_XOR_MAPPED_ADDRESS: u16 = 0x0020;
const ATTR_OTHER_ADDRESS: u16 = 0x802C;
const ATTR_CHANGE_REQUEST: u16 = 0x0003;
#[allow(dead_code)] // Reserved for future error parsing
const ATTR_ERROR_CODE: u16 = 0x0009;
const ATTR_SOFTWARE: u16 = 0x8022;

// Change request flags
const CHANGE_IP: u32 = 0x04;
const CHANGE_PORT: u32 = 0x02;

/// NAT type classification
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum NatType {
    /// No NAT - direct internet connection
    OpenInternet,
    /// Full cone NAT - any external host can send to mapped address
    FullCone,
    /// Restricted cone NAT - only hosts we've sent to can reply
    RestrictedCone,
    /// Port restricted cone NAT - only host:port we've sent to can reply
    PortRestrictedCone,
    /// Symmetric NAT - different mapping for each destination (needs TURN)
    Symmetric,
    /// NAT blocks UDP entirely
    UdpBlocked,
    /// Could not determine NAT type
    Unknown,
}

impl NatType {
    /// Check if this NAT type supports hole punching
    pub fn supports_hole_punching(&self) -> bool {
        matches!(
            self,
            NatType::OpenInternet
                | NatType::FullCone
                | NatType::RestrictedCone
                | NatType::PortRestrictedCone
        )
    }

    /// Check if this NAT type requires a relay (TURN)
    pub fn requires_relay(&self) -> bool {
        matches!(self, NatType::Symmetric | NatType::UdpBlocked)
    }
}

impl std::fmt::Display for NatType {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            NatType::OpenInternet => write!(f, "Open Internet (no NAT)"),
            NatType::FullCone => write!(f, "Full Cone NAT"),
            NatType::RestrictedCone => write!(f, "Restricted Cone NAT"),
            NatType::PortRestrictedCone => write!(f, "Port Restricted Cone NAT"),
            NatType::Symmetric => write!(f, "Symmetric NAT (relay required)"),
            NatType::UdpBlocked => write!(f, "UDP Blocked"),
            NatType::Unknown => write!(f, "Unknown NAT type"),
        }
    }
}

/// STUN client configuration
#[derive(Debug, Clone)]
pub struct StunConfig {
    /// Primary STUN servers to use
    pub servers: Vec<String>,
    /// Request timeout
    pub timeout: Duration,
    /// Number of retries per server
    pub retries: u32,
    /// Cache TTL for external address
    pub cache_ttl: Duration,
    /// Local address to bind (None = auto)
    pub local_addr: Option<SocketAddr>,
}

impl Default for StunConfig {
    fn default() -> Self {
        Self {
            servers: vec![
                "stun.l.google.com:19302".to_string(),
                "stun1.l.google.com:19302".to_string(),
                "stun.cloudflare.com:3478".to_string(),
                "stun.stunprotocol.org:3478".to_string(),
            ],
            timeout: Duration::from_secs(3),
            retries: 2,
            cache_ttl: Duration::from_secs(300), // 5 minutes
            local_addr: None,
        }
    }
}

/// Cached STUN result
#[derive(Debug, Clone)]
struct CachedResult {
    external_addr: SocketAddr,
    nat_type: Option<NatType>,
    timestamp: Instant,
}

/// STUN client for NAT traversal
pub struct StunClient {
    config: StunConfig,
    socket: UdpSocket,
    cache: Arc<Mutex<Option<CachedResult>>>,
}

impl StunClient {
    /// Create a new STUN client
    pub fn new(config: StunConfig) -> io::Result<Self> {
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
            cache: Arc::new(Mutex::new(None)),
        })
    }

    /// Create STUN client with default config
    pub fn with_defaults() -> io::Result<Self> {
        Self::new(StunConfig::default())
    }

    /// Get the external (public) address as seen by STUN servers
    ///
    /// Returns cached result if still valid, otherwise queries STUN server.
    pub fn get_external_address(&self) -> io::Result<SocketAddr> {
        // Check cache first
        {
            let cache = self.cache.lock().unwrap();
            if let Some(ref cached) = *cache {
                if cached.timestamp.elapsed() < self.config.cache_ttl {
                    return Ok(cached.external_addr);
                }
            }
        }

        // Query STUN server
        let result = self.query_stun_servers()?;

        // Update cache
        {
            let mut cache = self.cache.lock().unwrap();
            *cache = Some(CachedResult {
                external_addr: result,
                nat_type: None,
                timestamp: Instant::now(),
            });
        }

        Ok(result)
    }

    /// Detect the NAT type
    ///
    /// Performs multiple STUN queries to determine NAT behavior.
    /// This requires a STUN server that supports the CHANGE-REQUEST attribute
    /// and has multiple IP addresses (like stun.stunprotocol.org).
    pub fn detect_nat_type(&self) -> io::Result<NatType> {
        // Check cache first
        {
            let cache = self.cache.lock().unwrap();
            if let Some(ref cached) = *cache {
                if cached.timestamp.elapsed() < self.config.cache_ttl {
                    if let Some(nat_type) = cached.nat_type {
                        return Ok(nat_type);
                    }
                }
            }
        }

        // Perform NAT type detection
        let nat_type = self.perform_nat_detection()?;

        // Update cache with NAT type
        {
            let mut cache = self.cache.lock().unwrap();
            if let Some(ref mut cached) = *cache {
                cached.nat_type = Some(nat_type);
                cached.timestamp = Instant::now();
            } else {
                // Need to also get external address
                if let Ok(ext_addr) = self.query_stun_servers() {
                    *cache = Some(CachedResult {
                        external_addr: ext_addr,
                        nat_type: Some(nat_type),
                        timestamp: Instant::now(),
                    });
                }
            }
        }

        Ok(nat_type)
    }

    /// Clear the cached results
    pub fn clear_cache(&self) {
        let mut cache = self.cache.lock().unwrap();
        *cache = None;
    }

    /// Get the local address being used
    pub fn local_addr(&self) -> io::Result<SocketAddr> {
        self.socket.local_addr()
    }

    /// Query STUN servers and get external address
    fn query_stun_servers(&self) -> io::Result<SocketAddr> {
        let mut last_error = io::Error::new(io::ErrorKind::Other, "No STUN servers configured");

        for server in &self.config.servers {
            for attempt in 0..=self.config.retries {
                match self.query_single_server(server, false, false) {
                    Ok((mapped_addr, _)) => return Ok(mapped_addr),
                    Err(e) => {
                        log::debug!(
                            "STUN query to {} failed (attempt {}): {}",
                            server,
                            attempt + 1,
                            e
                        );
                        last_error = e;
                    }
                }
            }
        }

        Err(last_error)
    }

    /// Query a single STUN server
    ///
    /// Returns (mapped_address, other_address) where other_address is the
    /// alternate server address for NAT type detection (if supported).
    fn query_single_server(
        &self,
        server: &str,
        change_ip: bool,
        change_port: bool,
    ) -> io::Result<(SocketAddr, Option<SocketAddr>)> {
        // Resolve server address
        let server_addr = server
            .to_socket_addrs()?
            .next()
            .ok_or_else(|| io::Error::new(io::ErrorKind::InvalidInput, "Cannot resolve STUN server"))?;

        // Build STUN binding request
        let request = self.build_binding_request(change_ip, change_port);

        // Send request
        self.socket.send_to(&request, server_addr)?;

        // Receive response
        let mut buf = [0u8; 576]; // Minimum MTU
        let (len, _from) = self.socket.recv_from(&mut buf)?;

        // Parse response
        self.parse_binding_response(&buf[..len])
    }

    /// Build a STUN binding request message
    fn build_binding_request(&self, change_ip: bool, change_port: bool) -> Vec<u8> {
        let mut msg = Vec::with_capacity(48);

        // Message type: Binding Request
        msg.extend_from_slice(&STUN_BINDING_REQUEST.to_be_bytes());

        // Message length (will be filled in later)
        let len_pos = msg.len();
        msg.extend_from_slice(&0u16.to_be_bytes());

        // Magic cookie
        msg.extend_from_slice(&STUN_MAGIC_COOKIE.to_be_bytes());

        // Transaction ID (96 bits / 12 bytes) - random
        let txn_id: [u8; 12] = rand_transaction_id();
        msg.extend_from_slice(&txn_id);

        // Add CHANGE-REQUEST attribute if needed
        if change_ip || change_port {
            let mut flags: u32 = 0;
            if change_ip {
                flags |= CHANGE_IP;
            }
            if change_port {
                flags |= CHANGE_PORT;
            }

            // Attribute type
            msg.extend_from_slice(&ATTR_CHANGE_REQUEST.to_be_bytes());
            // Attribute length
            msg.extend_from_slice(&4u16.to_be_bytes());
            // Attribute value
            msg.extend_from_slice(&flags.to_be_bytes());
        }

        // Add SOFTWARE attribute (optional, for identification)
        let software = b"HORUS/1.0";
        let padded_len = (software.len() + 3) & !3; // Pad to 4-byte boundary
        msg.extend_from_slice(&ATTR_SOFTWARE.to_be_bytes());
        msg.extend_from_slice(&(software.len() as u16).to_be_bytes());
        msg.extend_from_slice(software);
        // Padding
        for _ in software.len()..padded_len {
            msg.push(0);
        }

        // Update message length (excluding 20-byte header)
        let attr_len = (msg.len() - STUN_HEADER_SIZE) as u16;
        msg[len_pos..len_pos + 2].copy_from_slice(&attr_len.to_be_bytes());

        msg
    }

    /// Parse a STUN binding response
    fn parse_binding_response(&self, data: &[u8]) -> io::Result<(SocketAddr, Option<SocketAddr>)> {
        if data.len() < STUN_HEADER_SIZE {
            return Err(io::Error::new(
                io::ErrorKind::InvalidData,
                "Response too short",
            ));
        }

        // Parse header
        let msg_type = u16::from_be_bytes([data[0], data[1]]);
        let msg_len = u16::from_be_bytes([data[2], data[3]]) as usize;
        let magic = u32::from_be_bytes([data[4], data[5], data[6], data[7]]);

        // Verify magic cookie
        if magic != STUN_MAGIC_COOKIE {
            return Err(io::Error::new(
                io::ErrorKind::InvalidData,
                "Invalid magic cookie",
            ));
        }

        // Check message type
        if msg_type == STUN_BINDING_ERROR {
            return Err(io::Error::new(
                io::ErrorKind::Other,
                "STUN server returned error",
            ));
        }

        if msg_type != STUN_BINDING_RESPONSE {
            return Err(io::Error::new(
                io::ErrorKind::InvalidData,
                format!("Unexpected message type: 0x{:04X}", msg_type),
            ));
        }

        // Verify length
        if data.len() < STUN_HEADER_SIZE + msg_len {
            return Err(io::Error::new(
                io::ErrorKind::InvalidData,
                "Message length mismatch",
            ));
        }

        // Extract transaction ID for XOR decoding
        let txn_id = &data[8..20];

        // Parse attributes
        let mut mapped_addr: Option<SocketAddr> = None;
        let mut other_addr: Option<SocketAddr> = None;
        let mut pos = STUN_HEADER_SIZE;

        while pos + 4 <= STUN_HEADER_SIZE + msg_len {
            let attr_type = u16::from_be_bytes([data[pos], data[pos + 1]]);
            let attr_len = u16::from_be_bytes([data[pos + 2], data[pos + 3]]) as usize;
            pos += 4;

            if pos + attr_len > data.len() {
                break;
            }

            let attr_data = &data[pos..pos + attr_len];

            match attr_type {
                ATTR_XOR_MAPPED_ADDRESS => {
                    if let Some(addr) = parse_xor_mapped_address(attr_data, txn_id) {
                        mapped_addr = Some(addr);
                    }
                }
                ATTR_MAPPED_ADDRESS => {
                    // Fallback if XOR-MAPPED-ADDRESS not present
                    if mapped_addr.is_none() {
                        mapped_addr = parse_mapped_address(attr_data);
                    }
                }
                ATTR_OTHER_ADDRESS => {
                    other_addr = parse_mapped_address(attr_data);
                }
                _ => {
                    // Ignore unknown attributes
                }
            }

            // Move to next attribute (padded to 4 bytes)
            pos += (attr_len + 3) & !3;
        }

        mapped_addr
            .map(|addr| (addr, other_addr))
            .ok_or_else(|| io::Error::new(io::ErrorKind::InvalidData, "No mapped address in response"))
    }

    /// Perform NAT type detection using RFC 5780 algorithm
    fn perform_nat_detection(&self) -> io::Result<NatType> {
        // Test 1: Basic binding request
        let result1 = self.query_stun_servers();
        let (mapped_addr1, other_addr) = match result1 {
            Ok(addr) => {
                // Re-query to get other_addr if available
                let full_result = self.query_single_server(&self.config.servers[0], false, false);
                match full_result {
                    Ok((mapped, other)) => (mapped, other),
                    Err(_) => (addr, None),
                }
            }
            Err(_) => return Ok(NatType::UdpBlocked),
        };

        // Check if we have external address same as local (no NAT)
        if let Ok(local) = self.socket.local_addr() {
            if local.ip() == mapped_addr1.ip() {
                return Ok(NatType::OpenInternet);
            }
        }

        // If server doesn't support OTHER-ADDRESS, we can't do full detection
        let Some(_alt_server) = other_addr else {
            // Simple detection: query two different servers and compare
            return self.simple_nat_detection(mapped_addr1);
        };

        // Test 2: Request response from different IP and port
        // (This requires server support for CHANGE-REQUEST)
        let result2 = self.query_single_server(&self.config.servers[0], true, true);

        match result2 {
            Ok(_) => {
                // Response came from different IP/port - Full Cone NAT
                return Ok(NatType::FullCone);
            }
            Err(_) => {
                // No response - continue testing
            }
        }

        // Test 3: Request response from same IP, different port
        let result3 = self.query_single_server(&self.config.servers[0], false, true);

        match result3 {
            Ok(_) => {
                // Response from different port - Restricted Cone
                return Ok(NatType::RestrictedCone);
            }
            Err(_) => {
                // No response - either Port Restricted or Symmetric
            }
        }

        // Test 4: Query different server and compare mapped address
        if self.config.servers.len() > 1 {
            let result4 = self.query_single_server(&self.config.servers[1], false, false);
            match result4 {
                Ok((mapped_addr2, _)) => {
                    if mapped_addr1 == mapped_addr2 {
                        // Same mapped address - Port Restricted Cone
                        return Ok(NatType::PortRestrictedCone);
                    } else {
                        // Different mapped address - Symmetric NAT
                        return Ok(NatType::Symmetric);
                    }
                }
                Err(_) => {
                    // Can't determine, assume Port Restricted
                    return Ok(NatType::PortRestrictedCone);
                }
            }
        }

        Ok(NatType::Unknown)
    }

    /// Simple NAT detection when full RFC 5780 testing isn't available
    fn simple_nat_detection(&self, first_addr: SocketAddr) -> io::Result<NatType> {
        // Query a second server if available
        if self.config.servers.len() > 1 {
            match self.query_single_server(&self.config.servers[1], false, false) {
                Ok((second_addr, _)) => {
                    if first_addr == second_addr {
                        // Same external address from different servers
                        // This is consistent with cone NATs
                        Ok(NatType::PortRestrictedCone) // Conservative assumption
                    } else {
                        // Different external addresses - Symmetric NAT
                        Ok(NatType::Symmetric)
                    }
                }
                Err(_) => Ok(NatType::Unknown),
            }
        } else {
            Ok(NatType::Unknown)
        }
    }
}

impl std::fmt::Debug for StunClient {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("StunClient")
            .field("config", &self.config)
            .field("local_addr", &self.socket.local_addr())
            .finish()
    }
}

/// Parse XOR-MAPPED-ADDRESS attribute (RFC 8489)
fn parse_xor_mapped_address(data: &[u8], txn_id: &[u8]) -> Option<SocketAddr> {
    if data.len() < 8 {
        return None;
    }

    let family = data[1];
    let xor_port = u16::from_be_bytes([data[2], data[3]]);
    let port = xor_port ^ ((STUN_MAGIC_COOKIE >> 16) as u16);

    match family {
        0x01 => {
            // IPv4
            if data.len() < 8 {
                return None;
            }
            let xor_ip = u32::from_be_bytes([data[4], data[5], data[6], data[7]]);
            let ip = xor_ip ^ STUN_MAGIC_COOKIE;
            let ip_addr = std::net::Ipv4Addr::from(ip);
            Some(SocketAddr::new(ip_addr.into(), port))
        }
        0x02 => {
            // IPv6
            if data.len() < 20 {
                return None;
            }
            let mut ip_bytes = [0u8; 16];
            // XOR with magic cookie (4 bytes) + transaction ID (12 bytes)
            let xor_bytes: Vec<u8> = STUN_MAGIC_COOKIE
                .to_be_bytes()
                .iter()
                .chain(txn_id.iter())
                .copied()
                .collect();
            for i in 0..16 {
                ip_bytes[i] = data[4 + i] ^ xor_bytes[i];
            }
            let ip_addr = std::net::Ipv6Addr::from(ip_bytes);
            Some(SocketAddr::new(ip_addr.into(), port))
        }
        _ => None,
    }
}

/// Parse MAPPED-ADDRESS attribute (legacy, RFC 3489)
fn parse_mapped_address(data: &[u8]) -> Option<SocketAddr> {
    if data.len() < 8 {
        return None;
    }

    let family = data[1];
    let port = u16::from_be_bytes([data[2], data[3]]);

    match family {
        0x01 => {
            // IPv4
            if data.len() < 8 {
                return None;
            }
            let ip = std::net::Ipv4Addr::new(data[4], data[5], data[6], data[7]);
            Some(SocketAddr::new(ip.into(), port))
        }
        0x02 => {
            // IPv6
            if data.len() < 20 {
                return None;
            }
            let mut ip_bytes = [0u8; 16];
            ip_bytes.copy_from_slice(&data[4..20]);
            let ip = std::net::Ipv6Addr::from(ip_bytes);
            Some(SocketAddr::new(ip.into(), port))
        }
        _ => None,
    }
}

/// Generate a random transaction ID
fn rand_transaction_id() -> [u8; 12] {
    use std::collections::hash_map::DefaultHasher;
    use std::hash::{Hash, Hasher};
    use std::time::SystemTime;

    let mut hasher = DefaultHasher::new();
    SystemTime::now().hash(&mut hasher);
    std::process::id().hash(&mut hasher);
    let h1 = hasher.finish();

    // Get more randomness
    let mut hasher2 = DefaultHasher::new();
    h1.hash(&mut hasher2);
    std::thread::current().id().hash(&mut hasher2);
    let h2 = hasher2.finish();

    let mut result = [0u8; 12];
    result[0..8].copy_from_slice(&h1.to_le_bytes());
    result[8..12].copy_from_slice(&h2.to_le_bytes()[0..4]);
    result
}

/// STUN discovery result containing external address and NAT type
#[derive(Debug, Clone)]
pub struct StunDiscoveryResult {
    /// External (public) IP address and port
    pub external_addr: SocketAddr,
    /// Detected NAT type
    pub nat_type: NatType,
    /// Local address used
    pub local_addr: SocketAddr,
    /// STUN server that responded
    pub stun_server: String,
    /// Time taken for discovery
    pub discovery_time: Duration,
}

/// Perform one-shot STUN discovery
///
/// Convenience function for quick external address discovery.
pub fn discover_external_address() -> io::Result<SocketAddr> {
    let client = StunClient::with_defaults()?;
    client.get_external_address()
}

/// Perform full STUN discovery including NAT type detection
pub fn full_stun_discovery() -> io::Result<StunDiscoveryResult> {
    let start = Instant::now();
    let client = StunClient::with_defaults()?;

    let external_addr = client.get_external_address()?;
    let nat_type = client.detect_nat_type().unwrap_or(NatType::Unknown);
    let local_addr = client.local_addr()?;

    Ok(StunDiscoveryResult {
        external_addr,
        nat_type,
        local_addr,
        stun_server: client.config.servers[0].clone(),
        discovery_time: start.elapsed(),
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_nat_type_display() {
        assert_eq!(format!("{}", NatType::OpenInternet), "Open Internet (no NAT)");
        assert_eq!(format!("{}", NatType::Symmetric), "Symmetric NAT (relay required)");
    }

    #[test]
    fn test_nat_type_hole_punching() {
        assert!(NatType::FullCone.supports_hole_punching());
        assert!(NatType::RestrictedCone.supports_hole_punching());
        assert!(NatType::PortRestrictedCone.supports_hole_punching());
        assert!(!NatType::Symmetric.supports_hole_punching());
        assert!(!NatType::UdpBlocked.supports_hole_punching());
    }

    #[test]
    fn test_nat_type_requires_relay() {
        assert!(!NatType::FullCone.requires_relay());
        assert!(NatType::Symmetric.requires_relay());
        assert!(NatType::UdpBlocked.requires_relay());
    }

    #[test]
    fn test_build_binding_request() {
        let config = StunConfig::default();
        let client = StunClient::new(config).unwrap();
        let request = client.build_binding_request(false, false);

        // Verify header
        assert!(request.len() >= STUN_HEADER_SIZE);

        // Check message type
        let msg_type = u16::from_be_bytes([request[0], request[1]]);
        assert_eq!(msg_type, STUN_BINDING_REQUEST);

        // Check magic cookie
        let magic = u32::from_be_bytes([request[4], request[5], request[6], request[7]]);
        assert_eq!(magic, STUN_MAGIC_COOKIE);
    }

    #[test]
    fn test_build_binding_request_with_change() {
        let config = StunConfig::default();
        let client = StunClient::new(config).unwrap();
        let request = client.build_binding_request(true, true);

        // Should be larger due to CHANGE-REQUEST attribute
        assert!(request.len() > STUN_HEADER_SIZE + 8);
    }

    #[test]
    fn test_parse_xor_mapped_address_v4() {
        // Sample XOR-MAPPED-ADDRESS for 192.168.1.1:12345
        // After XOR: IP = 192.168.1.1 ^ 0x2112A442
        let ip = 0xC0A80101u32; // 192.168.1.1
        let port = 12345u16;

        let xor_ip = ip ^ STUN_MAGIC_COOKIE;
        let xor_port = port ^ ((STUN_MAGIC_COOKIE >> 16) as u16);

        let mut data = vec![0u8; 8];
        data[0] = 0; // Reserved
        data[1] = 0x01; // IPv4 family
        data[2..4].copy_from_slice(&xor_port.to_be_bytes());
        data[4..8].copy_from_slice(&xor_ip.to_be_bytes());

        let txn_id = [0u8; 12];
        let result = parse_xor_mapped_address(&data, &txn_id);

        assert!(result.is_some());
        let addr = result.unwrap();
        assert_eq!(addr.port(), 12345);
        assert_eq!(addr.ip().to_string(), "192.168.1.1");
    }

    #[test]
    fn test_stun_config_default() {
        let config = StunConfig::default();
        assert!(!config.servers.is_empty());
        assert!(config.timeout > Duration::ZERO);
        assert!(config.cache_ttl > Duration::ZERO);
    }

    #[test]
    fn test_rand_transaction_id() {
        let id1 = rand_transaction_id();
        std::thread::sleep(Duration::from_millis(1));
        let id2 = rand_transaction_id();

        // IDs should be different
        assert_ne!(id1, id2);
    }
}
