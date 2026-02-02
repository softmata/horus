//! Husarnet VPN Integration for HORUS
//!
//! This module provides seamless integration with Husarnet, a peer-to-peer VPN
//! that uses IPv6 overlay networking. Husarnet enables secure, low-latency
//! communication between robots across the internet without complex NAT traversal.
//!
//! # Features
//!
//! - **Auto-detection**: Automatically detects when running on a Husarnet network
//! - **Peer discovery**: Discovers all Husarnet peers via the daemon API
//! - **Simulated multicast**: Since Husarnet doesn't support traditional multicast,
//!   this module provides unicast-based "broadcast" to all peers
//! - **IPv6-native**: Uses Husarnet's fc94::/16 IPv6 address space
//!
//! # Example
//!
//! ```rust,ignore
//! use horus_core::communication::network::husarnet::{HusarnetDiscovery, HusarnetConfig};
//!
//! // Check if Husarnet is available
//! if HusarnetDiscovery::is_available() {
//!     let discovery = HusarnetDiscovery::new(HusarnetConfig::default())?;
//!
//!     // Get our Husarnet IPv6 address
//!     let my_addr = discovery.local_address()?;
//!
//!     // Get all peers in the network
//!     let peers = discovery.discover_peers()?;
//!     for peer in peers {
//!         println!("{}: {}", peer.hostname, peer.address);
//!     }
//! }
//! ```
//!
//! # Network Architecture
//!
//! Husarnet creates a `hnet0` virtual network interface with a unique IPv6 address
//! in the fc94::/16 range. All Husarnet peers can communicate directly using these
//! addresses, even across NAT and firewalls.
//!
//! For topic@* multicast patterns, HORUS sends to each peer individually,
//! which works well for small to medium networks (< 100 peers).

use std::collections::HashMap;
use std::io;
use std::net::{Ipv6Addr, SocketAddr, SocketAddrV6};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant};

use parking_lot::RwLock;
use serde::{Deserialize, Serialize};

/// Husarnet daemon API port
pub const HUSARNET_API_PORT: u16 = 16216;

/// Husarnet daemon API host
pub const HUSARNET_API_HOST: &str = "127.0.0.1";

/// Husarnet IPv6 prefix (fc94::/16)
pub const HUSARNET_PREFIX: [u8; 2] = [0xfc, 0x94];

/// Default HORUS port for Husarnet communication
pub const DEFAULT_HUSARNET_PORT: u16 = 9847;

/// Peer refresh interval (how often to query daemon for peer updates)
pub const PEER_REFRESH_INTERVAL: Duration = Duration::from_secs(30);

/// Provides current instant for serde default
fn instant_now() -> Instant {
    Instant::now()
}

/// A discovered Husarnet peer
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct HusarnetPeer {
    /// Husarnet IPv6 address (fc94::/16)
    pub address: Ipv6Addr,
    /// Hostname of the peer (if known)
    pub hostname: Option<String>,
    /// Whether the peer is currently connected (P2P or tunneled)
    pub is_connected: bool,
    /// Whether the connection is peer-to-peer (vs tunneled through server)
    pub is_p2p: bool,
    /// When this peer was last seen (not serialized)
    #[serde(skip, default = "instant_now")]
    pub last_seen: Instant,
}

impl HusarnetPeer {
    /// Get socket address for HORUS communication
    pub fn socket_addr(&self, port: u16) -> SocketAddrV6 {
        SocketAddrV6::new(self.address, port, 0, 0)
    }

    /// Get socket address using default HORUS port
    pub fn default_socket_addr(&self) -> SocketAddrV6 {
        self.socket_addr(DEFAULT_HUSARNET_PORT)
    }
}

impl PartialEq for HusarnetPeer {
    fn eq(&self, other: &Self) -> bool {
        self.address == other.address
            && self.hostname == other.hostname
            && self.is_connected == other.is_connected
            && self.is_p2p == other.is_p2p
        // Intentionally skip last_seen comparison
    }
}

impl Eq for HusarnetPeer {}

impl Default for HusarnetPeer {
    fn default() -> Self {
        Self {
            address: Ipv6Addr::UNSPECIFIED,
            hostname: None,
            is_connected: false,
            is_p2p: false,
            last_seen: Instant::now(),
        }
    }
}

/// Local Husarnet status
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct HusarnetStatus {
    /// Whether Husarnet daemon is running and accessible
    pub daemon_running: bool,
    /// Our Husarnet IPv6 address (if available)
    pub local_address: Option<Ipv6Addr>,
    /// Our hostname on the Husarnet network
    pub hostname: Option<String>,
    /// Whether we're connected to the Husarnet base server
    pub base_connected: bool,
    /// Whether WebRTC is ready
    pub webrtc_ready: bool,
    /// Husarnet daemon version
    pub version: Option<String>,
    /// Number of connected peers
    pub peer_count: usize,
}

#[allow(clippy::derivable_impls)]
impl Default for HusarnetStatus {
    fn default() -> Self {
        Self {
            daemon_running: false,
            local_address: None,
            hostname: None,
            base_connected: false,
            webrtc_ready: false,
            version: None,
            peer_count: 0,
        }
    }
}

/// Configuration for Husarnet discovery
#[derive(Debug, Clone)]
pub struct HusarnetConfig {
    /// Daemon API host (default: 127.0.0.1)
    pub api_host: String,
    /// Daemon API port (default: 16216)
    pub api_port: u16,
    /// Request timeout for API calls
    pub request_timeout: Duration,
    /// How often to refresh peer list
    pub refresh_interval: Duration,
    /// Port to use for HORUS communication
    pub horus_port: u16,
    /// Whether to auto-start peer refresh
    pub auto_refresh: bool,
}

impl Default for HusarnetConfig {
    fn default() -> Self {
        Self {
            api_host: HUSARNET_API_HOST.to_string(),
            api_port: HUSARNET_API_PORT,
            request_timeout: Duration::from_secs(5),
            refresh_interval: PEER_REFRESH_INTERVAL,
            horus_port: DEFAULT_HUSARNET_PORT,
            auto_refresh: true,
        }
    }
}

/// Husarnet daemon API response for /api/status
#[derive(Debug, Deserialize)]
struct DaemonStatusResponse {
    #[serde(default)]
    result: Option<DaemonStatusResult>,
    #[serde(default)]
    version: Option<String>,
}

#[derive(Debug, Deserialize)]
#[allow(dead_code)] // Some fields reserved for future use
struct DaemonStatusResult {
    #[serde(default)]
    is_joined: bool,
    #[serde(default)]
    is_ready: bool,
    #[serde(default)]
    is_ready_to_join: bool,
    #[serde(default)]
    websetup_address: String,
    #[serde(default)]
    local_ip: String,
    #[serde(default)]
    local_hostname: String,
    #[serde(default)]
    whitelist: Vec<WhitelistEntry>,
}

#[derive(Debug, Deserialize)]
struct WhitelistEntry {
    #[serde(default)]
    address: String,
    #[serde(default)]
    hostname: Option<String>,
    #[serde(default)]
    is_active: bool,
    #[serde(default)]
    is_tunneled: bool,
}

/// Error types for Husarnet operations
#[derive(Debug, thiserror::Error)]
pub enum HusarnetError {
    #[error("Husarnet daemon not running or not accessible")]
    DaemonNotRunning,
    #[error("Failed to connect to Husarnet daemon API: {0}")]
    ApiConnectionError(String),
    #[error("Invalid API response: {0}")]
    InvalidResponse(String),
    #[error("Not connected to Husarnet network")]
    NotConnected,
    #[error("Failed to parse IPv6 address: {0}")]
    InvalidAddress(String),
    #[error("I/O error: {0}")]
    Io(#[from] io::Error),
}

/// Husarnet discovery service
///
/// Provides peer discovery and status monitoring for Husarnet networks.
/// This is the main entry point for Husarnet integration in HORUS.
pub struct HusarnetDiscovery {
    config: HusarnetConfig,
    /// Cached peers (updated periodically)
    peers: Arc<RwLock<HashMap<Ipv6Addr, HusarnetPeer>>>,
    /// Last status from daemon
    status: Arc<RwLock<HusarnetStatus>>,
    /// Last peer refresh time
    last_refresh: Arc<RwLock<Instant>>,
    /// Shutdown flag for background refresh
    shutdown: Arc<AtomicBool>,
}

impl HusarnetDiscovery {
    /// Create a new Husarnet discovery service
    pub fn new(config: HusarnetConfig) -> Result<Self, HusarnetError> {
        let discovery = Self {
            config,
            peers: Arc::new(RwLock::new(HashMap::new())),
            status: Arc::new(RwLock::new(HusarnetStatus::default())),
            last_refresh: Arc::new(RwLock::new(Instant::now())),
            shutdown: Arc::new(AtomicBool::new(false)),
        };

        // Do initial status check
        discovery.refresh_status()?;

        Ok(discovery)
    }

    /// Check if Husarnet is available on this system
    ///
    /// This performs a quick check by:
    /// 1. Looking for the hnet0 network interface
    /// 2. Attempting to connect to the daemon API
    pub fn is_available() -> bool {
        // Try to detect hnet0 interface via /sys/class/net (Linux)
        if std::path::Path::new("/sys/class/net/hnet0").exists() {
            return true;
        }

        // Try to connect to daemon API
        Self::probe_daemon_api().is_ok()
    }

    /// Quick probe of the daemon API
    fn probe_daemon_api() -> Result<(), HusarnetError> {
        let url = format!("http://{}:{}/hi", HUSARNET_API_HOST, HUSARNET_API_PORT);

        // Use blocking reqwest for simplicity (this is a quick check)
        let client = reqwest::blocking::Client::builder()
            .timeout(Duration::from_secs(2))
            .build()
            .map_err(|e| HusarnetError::ApiConnectionError(e.to_string()))?;

        let response = client
            .get(&url)
            .send()
            .map_err(|_e| HusarnetError::DaemonNotRunning)?;

        if response.status().is_success() {
            Ok(())
        } else {
            Err(HusarnetError::DaemonNotRunning)
        }
    }

    /// Refresh status and peer list from daemon
    pub fn refresh_status(&self) -> Result<(), HusarnetError> {
        let url = format!(
            "http://{}:{}/api/status",
            self.config.api_host, self.config.api_port
        );

        let client = reqwest::blocking::Client::builder()
            .timeout(self.config.request_timeout)
            .build()
            .map_err(|e| HusarnetError::ApiConnectionError(e.to_string()))?;

        let response = client
            .get(&url)
            .send()
            .map_err(|e| HusarnetError::ApiConnectionError(e.to_string()))?;

        if !response.status().is_success() {
            return Err(HusarnetError::ApiConnectionError(format!(
                "HTTP {}",
                response.status()
            )));
        }

        let body: DaemonStatusResponse = response
            .json()
            .map_err(|e| HusarnetError::InvalidResponse(e.to_string()))?;

        // Update status
        let mut status = self.status.write();
        status.daemon_running = true;
        status.version = body.version;

        if let Some(result) = body.result {
            status.base_connected = result.is_joined;
            status.webrtc_ready = result.is_ready;
            status.hostname = if result.local_hostname.is_empty() {
                None
            } else {
                Some(result.local_hostname)
            };
            status.local_address = parse_husarnet_ipv6(&result.local_ip).ok();

            // Update peers from whitelist
            let mut peers = self.peers.write();
            peers.clear();
            let now = Instant::now();

            for entry in result.whitelist {
                if let Ok(addr) = parse_husarnet_ipv6(&entry.address) {
                    peers.insert(
                        addr,
                        HusarnetPeer {
                            address: addr,
                            hostname: entry.hostname,
                            is_connected: entry.is_active,
                            is_p2p: !entry.is_tunneled,
                            last_seen: now,
                        },
                    );
                }
            }

            status.peer_count = peers.len();
        }

        *self.last_refresh.write() = Instant::now();

        Ok(())
    }

    /// Get the current Husarnet status
    pub fn status(&self) -> HusarnetStatus {
        self.status.read().clone()
    }

    /// Get our local Husarnet IPv6 address
    pub fn local_address(&self) -> Option<Ipv6Addr> {
        self.status.read().local_address
    }

    /// Get our hostname on the Husarnet network
    pub fn hostname(&self) -> Option<String> {
        self.status.read().hostname.clone()
    }

    /// Check if we're connected to the Husarnet network
    pub fn is_connected(&self) -> bool {
        self.status.read().base_connected
    }

    /// Get all discovered peers
    pub fn peers(&self) -> Vec<HusarnetPeer> {
        self.peers.read().values().cloned().collect()
    }

    /// Get connected peers only
    pub fn connected_peers(&self) -> Vec<HusarnetPeer> {
        self.peers
            .read()
            .values()
            .filter(|p| p.is_connected)
            .cloned()
            .collect()
    }

    /// Get peer by address
    pub fn get_peer(&self, address: &Ipv6Addr) -> Option<HusarnetPeer> {
        self.peers.read().get(address).cloned()
    }

    /// Get peer by hostname
    pub fn get_peer_by_hostname(&self, hostname: &str) -> Option<HusarnetPeer> {
        self.peers
            .read()
            .values()
            .find(|p| p.hostname.as_deref() == Some(hostname))
            .cloned()
    }

    /// Get socket addresses for all connected peers (for broadcast)
    ///
    /// This is used to implement "simulated multicast" - sending to all peers.
    pub fn broadcast_addresses(&self, port: u16) -> Vec<SocketAddr> {
        self.connected_peers()
            .iter()
            .map(|p| SocketAddr::V6(p.socket_addr(port)))
            .collect()
    }

    /// Get socket addresses using default HORUS port
    pub fn default_broadcast_addresses(&self) -> Vec<SocketAddr> {
        self.broadcast_addresses(self.config.horus_port)
    }

    /// Check if a refresh is needed based on configured interval
    pub fn needs_refresh(&self) -> bool {
        self.last_refresh.read().elapsed() >= self.config.refresh_interval
    }

    /// Refresh if needed (call this periodically)
    pub fn maybe_refresh(&self) -> Result<(), HusarnetError> {
        if self.needs_refresh() {
            self.refresh_status()?;
        }
        Ok(())
    }

    /// Shutdown the discovery service
    pub fn shutdown(&self) {
        self.shutdown.store(true, Ordering::SeqCst);
    }
}

impl Drop for HusarnetDiscovery {
    fn drop(&mut self) {
        self.shutdown();
    }
}

/// Parse a Husarnet IPv6 address string
fn parse_husarnet_ipv6(s: &str) -> Result<Ipv6Addr, HusarnetError> {
    // Handle addresses with or without brackets
    let cleaned = s.trim().trim_start_matches('[').trim_end_matches(']');

    cleaned
        .parse()
        .map_err(|_| HusarnetError::InvalidAddress(s.to_string()))
}

/// Check if an IPv6 address is a Husarnet address (fc94::/16)
pub fn is_husarnet_address(addr: &Ipv6Addr) -> bool {
    let octets = addr.octets();
    octets[0] == HUSARNET_PREFIX[0] && octets[1] == HUSARNET_PREFIX[1]
}

/// Get the Husarnet IPv6 address from the hnet0 interface (Linux)
#[cfg(target_os = "linux")]
pub fn get_hnet0_address() -> Option<Ipv6Addr> {
    use std::fs;

    // Read addresses from /sys/class/net/hnet0/address (MAC) isn't useful
    // Instead, parse from ip command or /proc/net/if_inet6
    let content = fs::read_to_string("/proc/net/if_inet6").ok()?;

    for line in content.lines() {
        let parts: Vec<&str> = line.split_whitespace().collect();
        if parts.len() >= 6 && parts[5] == "hnet0" {
            // Parse the hex address (32 hex chars without colons)
            let hex_addr = parts[0];
            if hex_addr.len() == 32 {
                // Convert to IPv6 format with colons
                let formatted = format!(
                    "{}:{}:{}:{}:{}:{}:{}:{}",
                    &hex_addr[0..4],
                    &hex_addr[4..8],
                    &hex_addr[8..12],
                    &hex_addr[12..16],
                    &hex_addr[16..20],
                    &hex_addr[20..24],
                    &hex_addr[24..28],
                    &hex_addr[28..32]
                );
                if let Ok(addr) = formatted.parse::<Ipv6Addr>() {
                    if is_husarnet_address(&addr) {
                        return Some(addr);
                    }
                }
            }
        }
    }

    None
}

/// Get the Husarnet IPv6 address (non-Linux - uses API only)
#[cfg(not(target_os = "linux"))]
pub fn get_hnet0_address() -> Option<Ipv6Addr> {
    // On non-Linux systems, we can only get the address via the daemon API
    None
}

/// Builder for HusarnetDiscovery
pub struct HusarnetDiscoveryBuilder {
    config: HusarnetConfig,
}

impl HusarnetDiscoveryBuilder {
    /// Create a new builder with default config
    pub fn new() -> Self {
        Self {
            config: HusarnetConfig::default(),
        }
    }

    /// Set the daemon API host
    pub fn api_host(mut self, host: &str) -> Self {
        self.config.api_host = host.to_string();
        self
    }

    /// Set the daemon API port
    pub fn api_port(mut self, port: u16) -> Self {
        self.config.api_port = port;
        self
    }

    /// Set the request timeout
    pub fn request_timeout(mut self, timeout: Duration) -> Self {
        self.config.request_timeout = timeout;
        self
    }

    /// Set the peer refresh interval
    pub fn refresh_interval(mut self, interval: Duration) -> Self {
        self.config.refresh_interval = interval;
        self
    }

    /// Set the HORUS communication port
    pub fn horus_port(mut self, port: u16) -> Self {
        self.config.horus_port = port;
        self
    }

    /// Enable/disable auto-refresh
    pub fn auto_refresh(mut self, enabled: bool) -> Self {
        self.config.auto_refresh = enabled;
        self
    }

    /// Build the HusarnetDiscovery
    pub fn build(self) -> Result<HusarnetDiscovery, HusarnetError> {
        HusarnetDiscovery::new(self.config)
    }
}

impl Default for HusarnetDiscoveryBuilder {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_is_husarnet_address() {
        // Valid Husarnet addresses
        let valid: Ipv6Addr = "fc94:d797:bcdf:d20a:e2dc:c25e:2536:c1cd".parse().unwrap();
        assert!(is_husarnet_address(&valid));

        let valid2: Ipv6Addr = "fc94:0000:0000:0000:0000:0000:0000:0001".parse().unwrap();
        assert!(is_husarnet_address(&valid2));

        // Invalid addresses (not in fc94::/16)
        let invalid1: Ipv6Addr = "fe80::1".parse().unwrap();
        assert!(!is_husarnet_address(&invalid1));

        let invalid2: Ipv6Addr = "::1".parse().unwrap();
        assert!(!is_husarnet_address(&invalid2));

        let invalid3: Ipv6Addr = "2001:db8::1".parse().unwrap();
        assert!(!is_husarnet_address(&invalid3));
    }

    #[test]
    fn test_parse_husarnet_ipv6() {
        // Normal format
        let addr = parse_husarnet_ipv6("fc94:d797:bcdf:d20a:e2dc:c25e:2536:c1cd").unwrap();
        assert!(is_husarnet_address(&addr));

        // With brackets
        let addr2 = parse_husarnet_ipv6("[fc94:d797:bcdf:d20a:e2dc:c25e:2536:c1cd]").unwrap();
        assert_eq!(addr, addr2);

        // With whitespace
        let addr3 = parse_husarnet_ipv6("  fc94:d797:bcdf:d20a:e2dc:c25e:2536:c1cd  ").unwrap();
        assert_eq!(addr, addr3);

        // Invalid
        assert!(parse_husarnet_ipv6("not-an-ip").is_err());
    }

    #[test]
    fn test_husarnet_peer() {
        let addr: Ipv6Addr = "fc94:d797:bcdf:d20a:e2dc:c25e:2536:c1cd".parse().unwrap();
        let peer = HusarnetPeer {
            address: addr,
            hostname: Some("robot-arm".to_string()),
            is_connected: true,
            is_p2p: true,
            last_seen: Instant::now(),
        };

        let socket = peer.socket_addr(9847);
        assert_eq!(socket.port(), 9847);
        assert_eq!(*socket.ip(), addr);

        let default_socket = peer.default_socket_addr();
        assert_eq!(default_socket.port(), DEFAULT_HUSARNET_PORT);
    }

    #[test]
    fn test_husarnet_config_default() {
        let config = HusarnetConfig::default();
        assert_eq!(config.api_host, "127.0.0.1");
        assert_eq!(config.api_port, 16216);
        assert_eq!(config.horus_port, DEFAULT_HUSARNET_PORT);
        assert!(config.auto_refresh);
    }

    #[test]
    fn test_husarnet_status_default() {
        let status = HusarnetStatus::default();
        assert!(!status.daemon_running);
        assert!(status.local_address.is_none());
        assert!(!status.base_connected);
        assert_eq!(status.peer_count, 0);
    }

    #[test]
    fn test_builder() {
        // Builder should compile, but won't actually connect without Husarnet
        let builder = HusarnetDiscoveryBuilder::new()
            .api_host("127.0.0.1")
            .api_port(16216)
            .horus_port(9999)
            .auto_refresh(false)
            .request_timeout(Duration::from_secs(10))
            .refresh_interval(Duration::from_secs(60));

        assert_eq!(builder.config.horus_port, 9999);
        assert!(!builder.config.auto_refresh);
    }
}
