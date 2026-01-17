//! Cloud connectivity patterns for HORUS
//!
//! This module provides cloud-based communication for robots across the internet.
//! It supports multiple connectivity options from fully managed to self-hosted.
//!
//! # Connectivity Options
//!
//! ## 1. HORUS Cloud (Managed Service)
//!
//! Zero-configuration cloud connectivity using the HORUS Cloud service.
//! Robots connect to `cloud.horus.io` for signaling and relay.
//!
//! ```text
//! topic@cloud:room-name
//! topic@cloud:room-name/secret-key
//! ```
//!
//! ## 2. Self-Hosted Relay
//!
//! Run your own HORUS relay server for privacy and control.
//!
//! ```text
//! topic@relay:192.168.1.100
//! topic@relay:relay.mycompany.com
//! topic@relay:relay.mycompany.com:9877
//! ```
//!
//! ## 3. VPN Integration (Husarnet)
//!
//! Use Husarnet VPN for peer-to-peer connectivity without relays.
//!
//! ```text
//! topic@vpn:husarnet-address
//! topic@vpn:robot1
//! ```
//!
//! # Example
//!
//! ```ignore
//! use horus_core::communication::network::cloud::{CloudConfig, CloudBackend};
//!
//! // Connect to HORUS Cloud
//! let config = CloudConfig::horus_cloud("my-robot-fleet");
//! let backend = CloudBackend::connect(config).await?;
//!
//! // Publish to cloud room
//! backend.publish("sensors/lidar", data).await?;
//! ```

use std::net::{IpAddr, SocketAddr};
use std::time::Duration;

/// Cloud connection modes
#[derive(Debug, Clone, PartialEq)]
pub enum CloudMode {
    /// HORUS Cloud managed service (cloud.horus.io)
    HorusCloud {
        /// Room name for grouping robots
        room: String,
        /// Optional authentication key
        auth_key: Option<String>,
    },

    /// Self-hosted relay server
    SelfHosted {
        /// Relay server address
        host: String,
        /// Relay server port (default: 9877)
        port: u16,
        /// Optional TLS
        tls: bool,
    },

    /// VPN-based connectivity (Husarnet, Tailscale, etc.)
    Vpn {
        /// VPN type
        vpn_type: VpnType,
        /// Target peer identifier
        peer: String,
    },
}

/// Supported VPN types
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum VpnType {
    /// Husarnet - peer-to-peer VPN for robots
    Husarnet,
    /// Tailscale - WireGuard-based mesh VPN
    Tailscale,
    /// ZeroTier - software-defined networking
    ZeroTier,
    /// Generic WireGuard tunnel
    WireGuard,
}

impl std::fmt::Display for VpnType {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            VpnType::Husarnet => write!(f, "husarnet"),
            VpnType::Tailscale => write!(f, "tailscale"),
            VpnType::ZeroTier => write!(f, "zerotier"),
            VpnType::WireGuard => write!(f, "wireguard"),
        }
    }
}

impl std::str::FromStr for VpnType {
    type Err = String;

    fn from_str(s: &str) -> Result<Self, Self::Err> {
        match s.to_lowercase().as_str() {
            "husarnet" => Ok(VpnType::Husarnet),
            "tailscale" => Ok(VpnType::Tailscale),
            "zerotier" => Ok(VpnType::ZeroTier),
            "wireguard" | "wg" => Ok(VpnType::WireGuard),
            _ => Err(format!(
                "Unknown VPN type '{}'. Supported: husarnet, tailscale, zerotier, wireguard",
                s
            )),
        }
    }
}

/// Cloud connection configuration
#[derive(Debug, Clone)]
pub struct CloudConfig {
    /// Connection mode
    pub mode: CloudMode,
    /// Connection timeout
    pub connect_timeout: Duration,
    /// Keep-alive interval
    pub keepalive_interval: Duration,
    /// Reconnect on disconnect
    pub auto_reconnect: bool,
    /// Maximum reconnect attempts (0 = unlimited)
    pub max_reconnect_attempts: u32,
    /// Enable compression for cloud traffic
    pub compression: bool,
    /// Enable encryption (always true for HORUS Cloud)
    pub encryption: bool,
}

impl Default for CloudConfig {
    fn default() -> Self {
        Self {
            mode: CloudMode::HorusCloud {
                room: "default".to_string(),
                auth_key: None,
            },
            connect_timeout: Duration::from_secs(10),
            keepalive_interval: Duration::from_secs(30),
            auto_reconnect: true,
            max_reconnect_attempts: 0, // Unlimited
            compression: true,
            encryption: true,
        }
    }
}

impl CloudConfig {
    /// Create config for HORUS Cloud managed service
    pub fn horus_cloud(room: &str) -> Self {
        Self {
            mode: CloudMode::HorusCloud {
                room: room.to_string(),
                auth_key: None,
            },
            ..Default::default()
        }
    }

    /// Create config for HORUS Cloud with authentication
    pub fn horus_cloud_auth(room: &str, auth_key: &str) -> Self {
        Self {
            mode: CloudMode::HorusCloud {
                room: room.to_string(),
                auth_key: Some(auth_key.to_string()),
            },
            ..Default::default()
        }
    }

    /// Create config for self-hosted relay
    pub fn self_hosted(host: &str, port: u16) -> Self {
        Self {
            mode: CloudMode::SelfHosted {
                host: host.to_string(),
                port,
                tls: false,
            },
            encryption: false, // Can be enabled separately
            ..Default::default()
        }
    }

    /// Create config for self-hosted relay with TLS
    pub fn self_hosted_tls(host: &str, port: u16) -> Self {
        Self {
            mode: CloudMode::SelfHosted {
                host: host.to_string(),
                port,
                tls: true,
            },
            ..Default::default()
        }
    }

    /// Create config for Husarnet VPN
    pub fn husarnet(peer: &str) -> Self {
        Self {
            mode: CloudMode::Vpn {
                vpn_type: VpnType::Husarnet,
                peer: peer.to_string(),
            },
            encryption: false, // VPN provides encryption
            ..Default::default()
        }
    }

    /// Create config for generic VPN
    pub fn vpn(vpn_type: VpnType, peer: &str) -> Self {
        Self {
            mode: CloudMode::Vpn {
                vpn_type,
                peer: peer.to_string(),
            },
            encryption: false, // VPN provides encryption
            ..Default::default()
        }
    }

    /// Set connection timeout
    pub fn with_timeout(mut self, timeout: Duration) -> Self {
        self.connect_timeout = timeout;
        self
    }

    /// Disable auto-reconnect
    pub fn without_reconnect(mut self) -> Self {
        self.auto_reconnect = false;
        self
    }

    /// Set maximum reconnect attempts
    pub fn with_max_reconnects(mut self, max: u32) -> Self {
        self.max_reconnect_attempts = max;
        self
    }

    /// Enable/disable compression
    pub fn with_compression(mut self, enabled: bool) -> Self {
        self.compression = enabled;
        self
    }
}

/// Default HORUS Cloud service endpoints
pub mod cloud_service {
    /// HORUS Cloud signaling server
    pub const HORUS_CLOUD_SIGNALING: &str = "wss://cloud.horus.io/signaling";

    /// HORUS Cloud TURN relay
    pub const HORUS_CLOUD_TURN: &str = "turn:turn.horus.io:3478";

    /// HORUS Cloud STUN server
    pub const HORUS_CLOUD_STUN: &str = "stun:stun.horus.io:3478";

    /// Default relay port for self-hosted deployments
    pub const DEFAULT_RELAY_PORT: u16 = 9877;

    /// API version
    pub const API_VERSION: &str = "v1";
}

/// Cloud room information
#[derive(Debug, Clone)]
pub struct CloudRoom {
    /// Room name
    pub name: String,
    /// Room ID (generated by server)
    pub id: String,
    /// Number of connected peers
    pub peer_count: usize,
    /// Topics being published in this room
    pub topics: Vec<String>,
    /// Room creation time
    pub created_at: std::time::SystemTime,
}

/// Cloud connection statistics
#[derive(Debug, Clone, Default)]
pub struct CloudStats {
    /// Messages sent
    pub messages_sent: u64,
    /// Messages received
    pub messages_received: u64,
    /// Bytes sent (after compression)
    pub bytes_sent: u64,
    /// Bytes received (before decompression)
    pub bytes_received: u64,
    /// Compression ratio (uncompressed/compressed)
    pub compression_ratio: f32,
    /// Connection uptime
    pub uptime_seconds: u64,
    /// Reconnection count
    pub reconnect_count: u32,
    /// Current latency to cloud (RTT)
    pub latency_ms: u32,
}

/// Parse cloud endpoint location string
///
/// # Formats
///
/// - `cloud:room-name` - HORUS Cloud with room name
/// - `cloud:room-name/auth-key` - HORUS Cloud with authentication
/// - `relay:host` - Self-hosted relay (default port)
/// - `relay:host:port` - Self-hosted relay with port
/// - `vpn:peer` - VPN (auto-detect type from hostname)
/// - `vpn/husarnet:peer` - Explicit Husarnet VPN
/// - `vpn/tailscale:peer` - Explicit Tailscale VPN
///
/// # Examples
///
/// ```
/// use horus_core::communication::network::cloud::{parse_cloud_location, CloudMode, VpnType};
///
/// // HORUS Cloud
/// let (mode, _) = parse_cloud_location("cloud:my-fleet").unwrap();
/// assert!(matches!(mode, CloudMode::HorusCloud { room, .. } if room == "my-fleet"));
///
/// // Self-hosted relay
/// let (mode, _) = parse_cloud_location("relay:192.168.1.100").unwrap();
/// assert!(matches!(mode, CloudMode::SelfHosted { host, port, .. } if host == "192.168.1.100" && port == 9877));
///
/// // VPN
/// let (mode, _) = parse_cloud_location("vpn/husarnet:robot1").unwrap();
/// assert!(matches!(mode, CloudMode::Vpn { vpn_type: VpnType::Husarnet, peer } if peer == "robot1"));
/// ```
pub fn parse_cloud_location(location: &str) -> Result<(CloudMode, CloudConfig), String> {
    // Parse cloud:room-name or cloud:room-name/auth-key
    if let Some(rest) = location.strip_prefix("cloud:") {
        let (room, auth_key) = if let Some(slash_pos) = rest.find('/') {
            let room = &rest[..slash_pos];
            let auth = &rest[slash_pos + 1..];
            if auth.is_empty() {
                return Err("Auth key cannot be empty after '/'".to_string());
            }
            (room.to_string(), Some(auth.to_string()))
        } else {
            (rest.to_string(), None)
        };

        if room.is_empty() {
            return Err("Room name cannot be empty".to_string());
        }

        // Validate room name: alphanumeric, hyphens, underscores
        if !is_valid_room_name(&room) {
            return Err(format!(
                "Invalid room name '{}': must contain only letters, numbers, hyphens, and underscores",
                room
            ));
        }

        let config = if let Some(ref key) = auth_key {
            CloudConfig::horus_cloud_auth(&room, key)
        } else {
            CloudConfig::horus_cloud(&room)
        };

        return Ok((
            CloudMode::HorusCloud { room, auth_key },
            config,
        ));
    }

    // Parse relay:host or relay:host:port
    if let Some(rest) = location.strip_prefix("relay:") {
        let (host, port) = parse_host_port(rest, cloud_service::DEFAULT_RELAY_PORT)?;

        let config = CloudConfig::self_hosted(&host, port);

        return Ok((
            CloudMode::SelfHosted {
                host,
                port,
                tls: false,
            },
            config,
        ));
    }

    // Parse relay-tls:host or relay-tls:host:port (TLS-enabled relay)
    if let Some(rest) = location.strip_prefix("relay-tls:") {
        let (host, port) = parse_host_port(rest, cloud_service::DEFAULT_RELAY_PORT)?;

        let config = CloudConfig::self_hosted_tls(&host, port);

        return Ok((
            CloudMode::SelfHosted {
                host,
                port,
                tls: true,
            },
            config,
        ));
    }

    // Parse vpn:peer or vpn/type:peer
    if let Some(rest) = location.strip_prefix("vpn") {
        if rest.starts_with('/') {
            // vpn/type:peer format
            let rest = &rest[1..]; // Skip the '/'
            if let Some(colon_pos) = rest.find(':') {
                let vpn_type_str = &rest[..colon_pos];
                let peer = &rest[colon_pos + 1..];

                if peer.is_empty() {
                    return Err("VPN peer identifier cannot be empty".to_string());
                }

                let vpn_type: VpnType = vpn_type_str.parse()?;
                let config = CloudConfig::vpn(vpn_type, peer);

                return Ok((
                    CloudMode::Vpn {
                        vpn_type,
                        peer: peer.to_string(),
                    },
                    config,
                ));
            } else {
                return Err(format!(
                    "Invalid VPN format '{}': expected 'vpn/type:peer'",
                    location
                ));
            }
        } else if rest.starts_with(':') {
            // vpn:peer format (auto-detect VPN type)
            let peer = &rest[1..];
            if peer.is_empty() {
                return Err("VPN peer identifier cannot be empty".to_string());
            }

            // Auto-detect VPN type from hostname
            let vpn_type = detect_vpn_type(peer);
            let config = CloudConfig::vpn(vpn_type, peer);

            return Ok((
                CloudMode::Vpn {
                    vpn_type,
                    peer: peer.to_string(),
                },
                config,
            ));
        } else {
            return Err(format!(
                "Invalid VPN format '{}': expected 'vpn:peer' or 'vpn/type:peer'",
                location
            ));
        }
    }

    Err(format!(
        "Unknown cloud location format '{}'. \
         Expected: cloud:room, relay:host, relay:host:port, vpn:peer, or vpn/type:peer",
        location
    ))
}

/// Validate room name
fn is_valid_room_name(name: &str) -> bool {
    if name.is_empty() || name.len() > 64 {
        return false;
    }

    name.chars()
        .all(|c| c.is_ascii_alphanumeric() || c == '-' || c == '_')
}

/// Parse host:port string
fn parse_host_port(input: &str, default_port: u16) -> Result<(String, u16), String> {
    // Handle IPv6 addresses: [2001:db8::1]:port
    if input.starts_with('[') {
        if let Some(bracket_end) = input.find(']') {
            let host = &input[1..bracket_end];
            let rest = &input[bracket_end + 1..];

            let port = if rest.starts_with(':') {
                rest[1..]
                    .parse::<u16>()
                    .map_err(|e| format!("Invalid port: {}", e))?
            } else if rest.is_empty() {
                default_port
            } else {
                return Err(format!("Invalid format after IPv6 address: '{}'", rest));
            };

            return Ok((host.to_string(), port));
        } else {
            return Err("Missing closing bracket for IPv6 address".to_string());
        }
    }

    // Handle IPv4 or hostname with optional port
    if let Some(colon_pos) = input.rfind(':') {
        // Check if this is a port or part of an IPv6 address
        let after_colon = &input[colon_pos + 1..];
        if let Ok(port) = after_colon.parse::<u16>() {
            let host = &input[..colon_pos];
            return Ok((host.to_string(), port));
        }
    }

    // No port specified, use default
    Ok((input.to_string(), default_port))
}

/// Auto-detect VPN type from peer hostname
fn detect_vpn_type(peer: &str) -> VpnType {
    // Husarnet hostnames are typically IPv6 addresses in the fc94::/16 range
    // or simple names without dots
    if peer.starts_with("fc94:") || peer.starts_with("fc95:") {
        return VpnType::Husarnet;
    }

    // Tailscale uses .ts.net domain
    if peer.ends_with(".ts.net") {
        return VpnType::Tailscale;
    }

    // ZeroTier uses .zerotier domain or 10.x.x.x addresses
    if peer.ends_with(".zerotier") || peer.starts_with("10.") {
        return VpnType::ZeroTier;
    }

    // Default to Husarnet for simple names (most common in robotics)
    if !peer.contains('.') && !peer.contains(':') {
        return VpnType::Husarnet;
    }

    // Generic WireGuard for anything else
    VpnType::WireGuard
}

/// Resolve cloud endpoint to actual network address
///
/// For HORUS Cloud: Returns cloud.horus.io signaling server
/// For self-hosted: Returns the relay server address
/// For VPN: Resolves VPN hostname to IP address
pub fn resolve_cloud_endpoint(mode: &CloudMode) -> Result<SocketAddr, String> {
    match mode {
        CloudMode::HorusCloud { .. } => {
            // HORUS Cloud always connects to cloud.horus.io
            // The actual WebSocket connection is handled by the backend
            Ok("51.159.62.125:443".parse().unwrap()) // cloud.horus.io placeholder
        }
        CloudMode::SelfHosted { host, port, .. } => {
            // Try to parse as IP address first
            if let Ok(ip) = host.parse::<IpAddr>() {
                return Ok(SocketAddr::new(ip, *port));
            }

            // Otherwise, it's a hostname that needs DNS resolution
            // This is a placeholder - actual resolution happens at connection time
            Err(format!(
                "DNS resolution required for '{}:{}' - use async resolver",
                host, port
            ))
        }
        CloudMode::Vpn { peer, vpn_type } => {
            // VPN addresses need to be resolved through VPN interface
            Err(format!(
                "VPN resolution required for {} peer '{}' - use VPN resolver",
                vpn_type, peer
            ))
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_parse_cloud_horus() {
        let (mode, config) = parse_cloud_location("cloud:my-fleet").unwrap();
        match mode {
            CloudMode::HorusCloud { room, auth_key } => {
                assert_eq!(room, "my-fleet");
                assert!(auth_key.is_none());
            }
            _ => panic!("Expected HorusCloud"),
        }
        assert!(config.encryption);
    }

    #[test]
    fn test_parse_cloud_horus_with_auth() {
        let (mode, _) = parse_cloud_location("cloud:my-fleet/secret123").unwrap();
        match mode {
            CloudMode::HorusCloud { room, auth_key } => {
                assert_eq!(room, "my-fleet");
                assert_eq!(auth_key, Some("secret123".to_string()));
            }
            _ => panic!("Expected HorusCloud"),
        }
    }

    #[test]
    fn test_parse_relay() {
        let (mode, _) = parse_cloud_location("relay:192.168.1.100").unwrap();
        match mode {
            CloudMode::SelfHosted { host, port, tls } => {
                assert_eq!(host, "192.168.1.100");
                assert_eq!(port, 9877);
                assert!(!tls);
            }
            _ => panic!("Expected SelfHosted"),
        }
    }

    #[test]
    fn test_parse_relay_with_port() {
        let (mode, _) = parse_cloud_location("relay:relay.mycompany.com:8080").unwrap();
        match mode {
            CloudMode::SelfHosted { host, port, .. } => {
                assert_eq!(host, "relay.mycompany.com");
                assert_eq!(port, 8080);
            }
            _ => panic!("Expected SelfHosted"),
        }
    }

    #[test]
    fn test_parse_relay_tls() {
        let (mode, config) = parse_cloud_location("relay-tls:relay.mycompany.com").unwrap();
        match mode {
            CloudMode::SelfHosted { host, tls, .. } => {
                assert_eq!(host, "relay.mycompany.com");
                assert!(tls);
            }
            _ => panic!("Expected SelfHosted"),
        }
        assert!(config.encryption);
    }

    #[test]
    fn test_parse_vpn_auto() {
        let (mode, _) = parse_cloud_location("vpn:robot1").unwrap();
        match mode {
            CloudMode::Vpn { vpn_type, peer } => {
                // Simple name defaults to Husarnet
                assert_eq!(vpn_type, VpnType::Husarnet);
                assert_eq!(peer, "robot1");
            }
            _ => panic!("Expected Vpn"),
        }
    }

    #[test]
    fn test_parse_vpn_explicit() {
        let (mode, _) = parse_cloud_location("vpn/tailscale:myrobot.ts.net").unwrap();
        match mode {
            CloudMode::Vpn { vpn_type, peer } => {
                assert_eq!(vpn_type, VpnType::Tailscale);
                assert_eq!(peer, "myrobot.ts.net");
            }
            _ => panic!("Expected Vpn"),
        }
    }

    #[test]
    fn test_vpn_type_display() {
        assert_eq!(format!("{}", VpnType::Husarnet), "husarnet");
        assert_eq!(format!("{}", VpnType::Tailscale), "tailscale");
    }

    #[test]
    fn test_vpn_type_parse() {
        assert_eq!("husarnet".parse::<VpnType>().unwrap(), VpnType::Husarnet);
        assert_eq!("tailscale".parse::<VpnType>().unwrap(), VpnType::Tailscale);
        assert_eq!("zerotier".parse::<VpnType>().unwrap(), VpnType::ZeroTier);
        assert_eq!("wireguard".parse::<VpnType>().unwrap(), VpnType::WireGuard);
        assert_eq!("wg".parse::<VpnType>().unwrap(), VpnType::WireGuard);
    }

    #[test]
    fn test_detect_vpn_type() {
        // Husarnet IPv6
        assert_eq!(detect_vpn_type("fc94:1234::1"), VpnType::Husarnet);

        // Tailscale domain
        assert_eq!(detect_vpn_type("robot.ts.net"), VpnType::Tailscale);

        // ZeroTier
        assert_eq!(detect_vpn_type("10.147.17.1"), VpnType::ZeroTier);

        // Simple name -> Husarnet
        assert_eq!(detect_vpn_type("robot1"), VpnType::Husarnet);
    }

    #[test]
    fn test_cloud_config_defaults() {
        let config = CloudConfig::default();
        assert!(config.auto_reconnect);
        assert!(config.encryption);
        assert!(config.compression);
    }

    #[test]
    fn test_cloud_config_builders() {
        let config = CloudConfig::horus_cloud("test-room")
            .with_timeout(Duration::from_secs(5))
            .without_reconnect();

        assert!(!config.auto_reconnect);
        assert_eq!(config.connect_timeout, Duration::from_secs(5));
    }

    #[test]
    fn test_invalid_room_name() {
        assert!(parse_cloud_location("cloud:").is_err());
        assert!(parse_cloud_location("cloud:room with spaces").is_err());
        assert!(parse_cloud_location("cloud:room@invalid").is_err());
    }

    #[test]
    fn test_valid_room_names() {
        assert!(parse_cloud_location("cloud:my-fleet").is_ok());
        assert!(parse_cloud_location("cloud:my_fleet").is_ok());
        assert!(parse_cloud_location("cloud:MyFleet123").is_ok());
    }
}
