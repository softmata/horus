use std::net::IpAddr;

use super::cloud::CloudMode;
use super::p2p::{P2pStrategy, PeerId};
use super::zenoh_config::ZenohCloudConfig;

/// Endpoint types for HORUS network communication
#[derive(Debug, Clone, PartialEq)]
pub enum Endpoint {
    /// Local shared memory: "topic"
    Local { topic: String },

    /// Localhost (same machine): "topic@localhost"
    Localhost { topic: String, port: Option<u16> },

    /// Direct network connection: "topic@192.168.1.5" or "topic@192.168.1.5:9000"
    Direct {
        topic: String,
        host: IpAddr,
        port: u16,
    },

    /// Multicast discovery: "topic@*"
    Multicast { topic: String },

    /// mDNS hostname resolution: "topic@robot.local" or "topic@robot.local:9000"
    /// Automatically resolves .local hostnames via mDNS/DNS-SD
    Mdns {
        topic: String,
        /// Hostname without .local suffix (e.g., "robot" for "robot.local")
        hostname: String,
        /// Optional port (default: 9870)
        port: Option<u16>,
    },

    /// Router (central broker): "topic@router" or "topic@router:7777"
    Router {
        topic: String,
        host: Option<IpAddr>,
        port: Option<u16>,
    },

    /// Zenoh transport: "topic@zenoh" or "topic@zenoh/ros2"
    /// Supports multi-robot mesh, cloud connectivity, and ROS2 interop
    Zenoh {
        topic: String,
        /// If true, use ROS2-compatible topic naming and CDR serialization
        ros2_mode: bool,
        /// Optional router/peer endpoint to connect to
        connect: Option<String>,
    },

    /// Zenoh cloud transport: "topic@zenoh:cloud" or "topic@zenoh:cloud:router-endpoint"
    /// Enhanced cloud connectivity with failover, monitoring, and auto-discovery
    ///
    /// Formats:
    /// - `topic@zenoh:cloud` - Auto-connect to HORUS cloud with default config
    /// - `topic@zenoh:cloud/auto` - Auto-discover nearby routers via mDNS
    /// - `topic@zenoh:cloud:tcp/host:port` - Connect to specific self-hosted router
    /// - `topic@zenoh:cloud:tcp/host1:port,tcp/host2:port` - Multiple routers with failover
    ZenohCloud {
        topic: String,
        /// Cloud configuration with failover and monitoring
        config: ZenohCloudConfig,
    },

    /// P2P connection with NAT traversal: "topic@p2p:peer-id"
    /// Supports direct, STUN hole-punch, and TURN relay connections
    P2p {
        topic: String,
        /// Target peer ID (format: xxxx-xxxx-xxxx)
        peer_id: PeerId,
        /// Connection strategy (auto, direct, stun, turn)
        strategy: P2pStrategy,
    },

    /// Cloud connectivity: "topic@cloud:room-name" or "topic@relay:host"
    /// Supports HORUS Cloud, self-hosted relays, and VPN integration
    Cloud {
        topic: String,
        /// Cloud connection mode (HorusCloud, SelfHosted, Vpn)
        mode: CloudMode,
    },
}

/// Default port for HORUS direct connections
pub const DEFAULT_PORT: u16 = 9870;

/// Default multicast address for discovery
pub const MULTICAST_ADDR: &str = "239.255.72.85";
pub const MULTICAST_PORT: u16 = 9871;

/// Validate an mDNS hostname (DNS label)
/// Must contain only letters, numbers, and hyphens, and not start/end with hyphen
fn is_valid_mdns_hostname(hostname: &str) -> bool {
    if hostname.is_empty() || hostname.len() > 63 {
        return false;
    }

    let bytes = hostname.as_bytes();

    // Cannot start or end with hyphen
    if bytes[0] == b'-' || bytes[bytes.len() - 1] == b'-' {
        return false;
    }

    // Must contain only alphanumeric and hyphens
    hostname
        .chars()
        .all(|c| c.is_ascii_alphanumeric() || c == '-')
}

/// Parse endpoint string into Endpoint enum
///
/// # Format:
/// - `"topic"` → Local shared memory
/// - `"topic@localhost"` → Localhost (Unix socket or shared memory)
/// - `"topic@192.168.1.5"` → Direct network (default port 9870)
/// - `"topic@192.168.1.5:9000"` → Direct network (custom port)
/// - `"topic@robot.local"` → mDNS hostname (default port 9870)
/// - `"topic@robot.local:9000"` → mDNS hostname (custom port)
/// - `"topic@*"` → Multicast discovery
/// - `"topic@zenoh"` → Zenoh transport (local mesh)
/// - `"topic@zenoh/ros2"` → Zenoh with ROS2 compatibility
/// - `"topic@zenoh:tcp/host:port"` → Zenoh connecting to router
/// - `"topic@zenoh:cloud"` → Zenoh cloud (HORUS hosted, auto-discovery)
/// - `"topic@zenoh:cloud/auto"` → Zenoh cloud (auto-discovery mode)
/// - `"topic@zenoh:cloud/horus"` → Zenoh cloud (HORUS hosted, no auto-discovery)
/// - `"topic@zenoh:cloud:tcp/host:port"` → Zenoh cloud (self-hosted router)
/// - `"topic@zenoh:cloud:tcp/h1:p,tcp/h2:p"` → Zenoh cloud (multiple routers with failover)
/// - `"topic@p2p:peer-id"` → P2P with NAT traversal
/// - `"topic@cloud:room"` → HORUS Cloud room
pub fn parse_endpoint(input: &str) -> Result<Endpoint, String> {
    // Split on '@'
    if !input.contains('@') {
        return Ok(Endpoint::Local {
            topic: input.to_string(),
        });
    }

    let parts: Vec<&str> = input.split('@').collect();
    if parts.len() != 2 {
        return Err(format!(
            "Invalid endpoint format '{}': expected 'topic@location'",
            input
        ));
    }

    let topic = parts[0].to_string();
    let location = parts[1];

    if topic.is_empty() {
        return Err("Topic name cannot be empty".to_string());
    }

    // Check for multicast wildcard
    if location == "*" {
        return Ok(Endpoint::Multicast { topic });
    }

    // Check for router
    if location == "router" {
        return Ok(Endpoint::Router {
            topic,
            host: None, // Use default localhost
            port: None, // Use default 7777
        });
    }

    // Check for router with port: "router:7777"
    if let Some(port_str) = location.strip_prefix("router:") {
        let port = port_str
            .parse::<u16>()
            .map_err(|e| format!("Invalid router port '{}': {}", port_str, e))?;
        return Ok(Endpoint::Router {
            topic,
            host: None, // Use default localhost
            port: Some(port),
        });
    }

    // Check for zenoh transport: "zenoh", "zenoh/ros2", "zenoh@tcp/host:port"
    if location == "zenoh" {
        return Ok(Endpoint::Zenoh {
            topic,
            ros2_mode: false,
            connect: None,
        });
    }

    // Check for zenoh with ROS2 mode: "zenoh/ros2"
    if location == "zenoh/ros2" {
        return Ok(Endpoint::Zenoh {
            topic,
            ros2_mode: true,
            connect: None,
        });
    }

    // Check for zenoh:cloud BEFORE generic zenoh: (to ensure correct parsing order)
    // Enhanced cloud connectivity with failover
    // Formats:
    //   - zenoh:cloud - Default HORUS cloud with auto-discovery
    //   - zenoh:cloud/auto - Explicit auto-discovery mode
    //   - zenoh:cloud/horus - Explicit HORUS cloud (no auto-discovery)
    //   - zenoh:cloud:tcp/host:port - Self-hosted single router
    //   - zenoh:cloud:tcp/host1:port,tcp/host2:port - Multiple routers with failover
    if location == "zenoh:cloud" || location == "zenoh:cloud/auto" {
        return Ok(Endpoint::ZenohCloud {
            topic,
            config: ZenohCloudConfig::auto(),
        });
    }

    // zenoh:cloud/horus - Explicit HORUS cloud (no auto-discovery)
    if location == "zenoh:cloud/horus" {
        return Ok(Endpoint::ZenohCloud {
            topic,
            config: ZenohCloudConfig::horus_cloud().with_auto_discovery(false),
        });
    }

    // zenoh:cloud:tcp/host:port or zenoh:cloud:tcp/host1:port,tcp/host2:port
    if let Some(rest) = location.strip_prefix("zenoh:cloud:") {
        // Parse comma-separated routers
        let routers: Vec<&str> = rest.split(',').collect();
        if routers.is_empty() {
            return Err(format!(
                "Invalid zenoh:cloud endpoint: no router specified after 'zenoh:cloud:'"
            ));
        }

        // Create config with the specified routers
        let config = if routers.len() == 1 {
            // Single router - self-hosted mode
            ZenohCloudConfig::self_hosted(routers[0])
        } else {
            // Multiple routers - failover mode
            ZenohCloudConfig::new()
                .with_routers(&routers)
                .with_auto_discovery(false)
        };

        return Ok(Endpoint::ZenohCloud { topic, config });
    }

    // Check for zenoh with connect endpoint: "zenoh:tcp/host:port" or "zenoh/ros2:tcp/host:port"
    // NOTE: This must be AFTER zenoh:cloud checks to avoid matching "zenoh:cloud" as "zenoh:" + "cloud"
    if let Some(rest) = location.strip_prefix("zenoh:") {
        return Ok(Endpoint::Zenoh {
            topic,
            ros2_mode: false,
            connect: Some(rest.to_string()),
        });
    }

    // Check for zenoh ROS2 with connect endpoint: "zenoh/ros2:tcp/host:port"
    if let Some(rest) = location.strip_prefix("zenoh/ros2:") {
        return Ok(Endpoint::Zenoh {
            topic,
            ros2_mode: true,
            connect: Some(rest.to_string()),
        });
    }

    // Check for P2P: "p2p:peer-id" or "p2p:peer-id/strategy"
    if let Some(rest) = location.strip_prefix("p2p:") {
        let (peer_id, strategy) = super::p2p::parse_p2p_location(rest)?;
        return Ok(Endpoint::P2p {
            topic,
            peer_id,
            strategy,
        });
    }

    // Check for Cloud: "cloud:room", "relay:host", "vpn:peer"
    if location.starts_with("cloud:")
        || location.starts_with("relay:")
        || location.starts_with("relay-tls:")
        || location.starts_with("vpn:")
        || location.starts_with("vpn/")
    {
        let (mode, _config) = super::cloud::parse_cloud_location(location)?;
        return Ok(Endpoint::Cloud { topic, mode });
    }

    // Check for localhost
    if location == "localhost" || location == "127.0.0.1" || location == "::1" {
        return Ok(Endpoint::Localhost { topic, port: None });
    }

    // Check for mDNS hostname (.local suffix)
    // Supports: "robot.local", "robot.local:9000", "arm-controller.local"
    if location.ends_with(".local") || location.contains(".local:") {
        // Parse hostname and optional port
        let (hostname_with_local, port) = if let Some(colon_pos) = location.rfind(':') {
            // Check if the colon is part of the .local suffix or a port delimiter
            let after_colon = &location[colon_pos + 1..];
            if let Ok(p) = after_colon.parse::<u16>() {
                // It's a port number
                (&location[..colon_pos], Some(p))
            } else {
                // Not a valid port, treat whole thing as hostname
                (location, None)
            }
        } else {
            (location, None)
        };

        // Extract hostname without .local suffix
        let hostname = hostname_with_local
            .strip_suffix(".local")
            .unwrap_or(hostname_with_local)
            .to_string();

        if hostname.is_empty() {
            return Err("mDNS hostname cannot be empty (got '.local')".to_string());
        }

        // Validate hostname: must be valid DNS label
        if !is_valid_mdns_hostname(&hostname) {
            return Err(format!(
                "Invalid mDNS hostname '{}': must contain only letters, numbers, and hyphens",
                hostname
            ));
        }

        return Ok(Endpoint::Mdns {
            topic,
            hostname,
            port,
        });
    }

    // Parse host:port
    // Note: IPv6 addresses can contain ':' so we need special handling
    // IPv6 with port: [2001:db8::1]:9000
    // IPv6 without port: 2001:db8::1
    // IPv4 with port: 192.168.1.5:9000
    // IPv4 without port: 192.168.1.5

    if location.starts_with('[') {
        // IPv6 with brackets (and optional port)
        if let Some(bracket_end) = location.find(']') {
            let ipv6_str = &location[1..bracket_end];
            let host = ipv6_str
                .parse::<IpAddr>()
                .map_err(|e| format!("Invalid IPv6 address '{}': {}", ipv6_str, e))?;

            // Check if there's a port after the bracket
            if location.len() > bracket_end + 1 {
                if location.chars().nth(bracket_end + 1) == Some(':') {
                    let port_str = &location[bracket_end + 2..];
                    let port = port_str
                        .parse::<u16>()
                        .map_err(|e| format!("Invalid port '{}': {}", port_str, e))?;
                    return Ok(Endpoint::Direct { topic, host, port });
                } else {
                    return Err(format!("Invalid format after IPv6 address: '{}'", location));
                }
            }

            return Ok(Endpoint::Direct {
                topic,
                host,
                port: DEFAULT_PORT,
            });
        } else {
            return Err(format!(
                "Missing closing bracket in IPv6 address '{}'",
                location
            ));
        }
    }

    // Try to parse as IPv6 without brackets (no port)
    if let Ok(host) = location.parse::<IpAddr>() {
        return Ok(Endpoint::Direct {
            topic,
            host,
            port: DEFAULT_PORT,
        });
    }

    // Try IPv4 with port
    if let Some(colon_pos) = location.rfind(':') {
        let host_str = &location[..colon_pos];
        let port_str = &location[colon_pos + 1..];

        if let (Ok(host), Ok(port)) = (host_str.parse::<IpAddr>(), port_str.parse::<u16>()) {
            return Ok(Endpoint::Direct { topic, host, port });
        }
    }

    // Failed to parse
    Err(format!(
        "Invalid IP address or host:port format '{}'",
        location
    ))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_parse_local() {
        let ep = parse_endpoint("mytopic").unwrap();
        assert_eq!(
            ep,
            Endpoint::Local {
                topic: "mytopic".to_string()
            }
        );
    }

    #[test]
    fn test_parse_local_with_underscores() {
        let ep = parse_endpoint("my_topic_name").unwrap();
        assert_eq!(
            ep,
            Endpoint::Local {
                topic: "my_topic_name".to_string()
            }
        );
    }

    #[test]
    fn test_parse_localhost() {
        let ep = parse_endpoint("mytopic@localhost").unwrap();
        assert_eq!(
            ep,
            Endpoint::Localhost {
                topic: "mytopic".to_string(),
                port: None
            }
        );
    }

    #[test]
    fn test_parse_localhost_ipv4() {
        let ep = parse_endpoint("mytopic@127.0.0.1").unwrap();
        assert_eq!(
            ep,
            Endpoint::Localhost {
                topic: "mytopic".to_string(),
                port: None
            }
        );
    }

    #[test]
    fn test_parse_localhost_ipv6() {
        let ep = parse_endpoint("mytopic@::1").unwrap();
        assert_eq!(
            ep,
            Endpoint::Localhost {
                topic: "mytopic".to_string(),
                port: None
            }
        );
    }

    #[test]
    fn test_parse_direct_default_port() {
        let ep = parse_endpoint("mytopic@192.168.1.5").unwrap();
        match ep {
            Endpoint::Direct { topic, host, port } => {
                assert_eq!(topic, "mytopic");
                assert_eq!(host, "192.168.1.5".parse::<IpAddr>().unwrap());
                assert_eq!(port, DEFAULT_PORT);
            }
            _ => panic!("Wrong variant"),
        }
    }

    #[test]
    fn test_parse_direct_custom_port() {
        let ep = parse_endpoint("mytopic@192.168.1.5:9000").unwrap();
        match ep {
            Endpoint::Direct { topic, host, port } => {
                assert_eq!(topic, "mytopic");
                assert_eq!(host, "192.168.1.5".parse::<IpAddr>().unwrap());
                assert_eq!(port, 9000);
            }
            _ => panic!("Wrong variant"),
        }
    }

    #[test]
    fn test_parse_direct_ipv6() {
        let ep = parse_endpoint("mytopic@2001:db8::1").unwrap();
        match ep {
            Endpoint::Direct { topic, host, port } => {
                assert_eq!(topic, "mytopic");
                assert_eq!(host, "2001:db8::1".parse::<IpAddr>().unwrap());
                assert_eq!(port, DEFAULT_PORT);
            }
            _ => panic!("Wrong variant"),
        }
    }

    #[test]
    fn test_parse_direct_ipv6_with_port() {
        let ep = parse_endpoint("mytopic@[2001:db8::1]:9000").unwrap();
        match ep {
            Endpoint::Direct { topic, host, port } => {
                assert_eq!(topic, "mytopic");
                assert_eq!(host, "2001:db8::1".parse::<IpAddr>().unwrap());
                assert_eq!(port, 9000);
            }
            _ => panic!("Wrong variant"),
        }
    }

    #[test]
    fn test_parse_multicast() {
        let ep = parse_endpoint("mytopic@*").unwrap();
        assert_eq!(
            ep,
            Endpoint::Multicast {
                topic: "mytopic".to_string()
            }
        );
    }

    #[test]
    fn test_parse_error_empty_topic() {
        let result = parse_endpoint("@192.168.1.5");
        assert!(result.is_err());
        assert!(result.unwrap_err().contains("empty"));
    }

    #[test]
    fn test_parse_error_invalid_ip() {
        let result = parse_endpoint("mytopic@invalid.ip");
        assert!(result.is_err());
    }

    #[test]
    fn test_parse_error_invalid_port() {
        let result = parse_endpoint("mytopic@192.168.1.5:99999");
        assert!(result.is_err());
    }

    #[test]
    fn test_parse_error_multiple_at() {
        let result = parse_endpoint("mytopic@host@other");
        assert!(result.is_err());
    }

    #[test]
    fn test_parse_zenoh_basic() {
        let ep = parse_endpoint("mytopic@zenoh").unwrap();
        assert_eq!(
            ep,
            Endpoint::Zenoh {
                topic: "mytopic".to_string(),
                ros2_mode: false,
                connect: None
            }
        );
    }

    #[test]
    fn test_parse_zenoh_ros2() {
        let ep = parse_endpoint("mytopic@zenoh/ros2").unwrap();
        assert_eq!(
            ep,
            Endpoint::Zenoh {
                topic: "mytopic".to_string(),
                ros2_mode: true,
                connect: None
            }
        );
    }

    #[test]
    fn test_parse_zenoh_with_connect() {
        let ep = parse_endpoint("mytopic@zenoh:tcp/192.168.1.100:7447").unwrap();
        match ep {
            Endpoint::Zenoh {
                topic,
                ros2_mode,
                connect,
            } => {
                assert_eq!(topic, "mytopic");
                assert!(!ros2_mode);
                assert_eq!(connect, Some("tcp/192.168.1.100:7447".to_string()));
            }
            _ => panic!("Wrong variant"),
        }
    }

    #[test]
    fn test_parse_zenoh_ros2_with_connect() {
        let ep = parse_endpoint("mytopic@zenoh/ros2:tcp/cloud.example.com:7447").unwrap();
        match ep {
            Endpoint::Zenoh {
                topic,
                ros2_mode,
                connect,
            } => {
                assert_eq!(topic, "mytopic");
                assert!(ros2_mode);
                assert_eq!(connect, Some("tcp/cloud.example.com:7447".to_string()));
            }
            _ => panic!("Wrong variant"),
        }
    }

    #[test]
    fn test_parse_router() {
        let ep = parse_endpoint("mytopic@router").unwrap();
        assert_eq!(
            ep,
            Endpoint::Router {
                topic: "mytopic".to_string(),
                host: None,
                port: None
            }
        );
    }

    #[test]
    fn test_parse_router_with_port() {
        let ep = parse_endpoint("mytopic@router:8888").unwrap();
        assert_eq!(
            ep,
            Endpoint::Router {
                topic: "mytopic".to_string(),
                host: None,
                port: Some(8888)
            }
        );
    }

    // mDNS hostname tests
    #[test]
    fn test_parse_mdns_basic() {
        let ep = parse_endpoint("mytopic@robot.local").unwrap();
        assert_eq!(
            ep,
            Endpoint::Mdns {
                topic: "mytopic".to_string(),
                hostname: "robot".to_string(),
                port: None
            }
        );
    }

    #[test]
    fn test_parse_mdns_with_port() {
        let ep = parse_endpoint("mytopic@robot.local:9000").unwrap();
        assert_eq!(
            ep,
            Endpoint::Mdns {
                topic: "mytopic".to_string(),
                hostname: "robot".to_string(),
                port: Some(9000)
            }
        );
    }

    #[test]
    fn test_parse_mdns_hyphenated_hostname() {
        let ep = parse_endpoint("sensor_data@arm-controller.local").unwrap();
        assert_eq!(
            ep,
            Endpoint::Mdns {
                topic: "sensor_data".to_string(),
                hostname: "arm-controller".to_string(),
                port: None
            }
        );
    }

    #[test]
    fn test_parse_mdns_hyphenated_with_port() {
        let ep = parse_endpoint("cmd_vel@my-robot-01.local:8080").unwrap();
        assert_eq!(
            ep,
            Endpoint::Mdns {
                topic: "cmd_vel".to_string(),
                hostname: "my-robot-01".to_string(),
                port: Some(8080)
            }
        );
    }

    #[test]
    fn test_parse_mdns_numeric_hostname() {
        let ep = parse_endpoint("lidar@robot01.local").unwrap();
        assert_eq!(
            ep,
            Endpoint::Mdns {
                topic: "lidar".to_string(),
                hostname: "robot01".to_string(),
                port: None
            }
        );
    }

    #[test]
    fn test_parse_mdns_error_empty_hostname() {
        let result = parse_endpoint("mytopic@.local");
        assert!(result.is_err());
        assert!(result.unwrap_err().contains("empty"));
    }

    #[test]
    fn test_parse_mdns_error_starts_with_hyphen() {
        let result = parse_endpoint("mytopic@-robot.local");
        assert!(result.is_err());
        assert!(result.unwrap_err().contains("Invalid mDNS hostname"));
    }

    #[test]
    fn test_parse_mdns_error_ends_with_hyphen() {
        let result = parse_endpoint("mytopic@robot-.local");
        assert!(result.is_err());
        assert!(result.unwrap_err().contains("Invalid mDNS hostname"));
    }

    #[test]
    fn test_parse_mdns_error_invalid_chars() {
        let result = parse_endpoint("mytopic@robot_name.local");
        assert!(result.is_err());
        assert!(result.unwrap_err().contains("Invalid mDNS hostname"));
    }

    #[test]
    fn test_is_valid_mdns_hostname() {
        // Valid hostnames
        assert!(is_valid_mdns_hostname("robot"));
        assert!(is_valid_mdns_hostname("robot01"));
        assert!(is_valid_mdns_hostname("my-robot"));
        assert!(is_valid_mdns_hostname("arm-controller-01"));
        assert!(is_valid_mdns_hostname("a"));
        assert!(is_valid_mdns_hostname("ABC123"));

        // Invalid hostnames
        assert!(!is_valid_mdns_hostname(""));
        assert!(!is_valid_mdns_hostname("-robot"));
        assert!(!is_valid_mdns_hostname("robot-"));
        assert!(!is_valid_mdns_hostname("robot_name"));
        assert!(!is_valid_mdns_hostname("robot.name"));
        assert!(!is_valid_mdns_hostname("robot name"));
        // Too long (>63 chars)
        assert!(!is_valid_mdns_hostname(
            "abcdefghijklmnopqrstuvwxyzabcdefghijklmnopqrstuvwxyzabcdefghijkl"
        ));
    }

    // P2P endpoint tests
    #[test]
    fn test_parse_p2p_basic() {
        use crate::communication::network::p2p::P2pStrategy;
        let ep = parse_endpoint("mytopic@p2p:a3f7-k2m9-p4n8").unwrap();
        match ep {
            Endpoint::P2p {
                topic,
                peer_id,
                strategy,
            } => {
                assert_eq!(topic, "mytopic");
                assert_eq!(peer_id.short_id, "a3f7-k2m9-p4n8");
                assert_eq!(strategy, P2pStrategy::Auto);
            }
            _ => panic!("Wrong variant"),
        }
    }

    #[test]
    fn test_parse_p2p_with_strategy() {
        use crate::communication::network::p2p::P2pStrategy;

        // Direct strategy
        let ep = parse_endpoint("sensor_data@p2p:a3f7-k2m9-p4n8/direct").unwrap();
        match ep {
            Endpoint::P2p {
                topic,
                peer_id,
                strategy,
            } => {
                assert_eq!(topic, "sensor_data");
                assert_eq!(peer_id.short_id, "a3f7-k2m9-p4n8");
                assert_eq!(strategy, P2pStrategy::Direct);
            }
            _ => panic!("Wrong variant"),
        }

        // STUN strategy
        let ep = parse_endpoint("cmd_vel@p2p:b2c4-d5e6-f7g8/stun").unwrap();
        match ep {
            Endpoint::P2p { strategy, .. } => {
                assert_eq!(strategy, P2pStrategy::Stun);
            }
            _ => panic!("Wrong variant"),
        }

        // TURN strategy
        let ep = parse_endpoint("video@p2p:x1y2-z3a4-b5c6/turn").unwrap();
        match ep {
            Endpoint::P2p { strategy, .. } => {
                assert_eq!(strategy, P2pStrategy::Turn);
            }
            _ => panic!("Wrong variant"),
        }
    }

    #[test]
    fn test_parse_p2p_error_invalid_peer_id() {
        // Missing dashes
        let result = parse_endpoint("mytopic@p2p:a3f7k2m9p4n8");
        assert!(result.is_err());
        assert!(result.unwrap_err().contains("Invalid peer ID"));

        // Too short
        let result = parse_endpoint("mytopic@p2p:a3f7-k2m9");
        assert!(result.is_err());

        // Empty peer ID
        let result = parse_endpoint("mytopic@p2p:");
        assert!(result.is_err());
    }

    #[test]
    fn test_parse_p2p_error_invalid_strategy() {
        let result = parse_endpoint("mytopic@p2p:a3f7-k2m9-p4n8/invalid");
        assert!(result.is_err());
        assert!(result.unwrap_err().contains("Unknown P2P strategy"));
    }

    // Cloud endpoint tests
    #[test]
    fn test_parse_cloud_horus() {
        use crate::communication::network::cloud::CloudMode;

        let ep = parse_endpoint("sensor_data@cloud:my-fleet").unwrap();
        match ep {
            Endpoint::Cloud { topic, mode } => {
                assert_eq!(topic, "sensor_data");
                match mode {
                    CloudMode::HorusCloud { room, auth_key } => {
                        assert_eq!(room, "my-fleet");
                        assert!(auth_key.is_none());
                    }
                    _ => panic!("Expected HorusCloud mode"),
                }
            }
            _ => panic!("Expected Cloud endpoint"),
        }
    }

    #[test]
    fn test_parse_cloud_horus_with_auth() {
        use crate::communication::network::cloud::CloudMode;

        let ep = parse_endpoint("cmd_vel@cloud:robot-team/secret123").unwrap();
        match ep {
            Endpoint::Cloud { topic, mode } => {
                assert_eq!(topic, "cmd_vel");
                match mode {
                    CloudMode::HorusCloud { room, auth_key } => {
                        assert_eq!(room, "robot-team");
                        assert_eq!(auth_key, Some("secret123".to_string()));
                    }
                    _ => panic!("Expected HorusCloud mode"),
                }
            }
            _ => panic!("Expected Cloud endpoint"),
        }
    }

    #[test]
    fn test_parse_relay_basic() {
        use crate::communication::network::cloud::CloudMode;

        let ep = parse_endpoint("lidar@relay:192.168.1.100").unwrap();
        match ep {
            Endpoint::Cloud { topic, mode } => {
                assert_eq!(topic, "lidar");
                match mode {
                    CloudMode::SelfHosted { host, port, tls } => {
                        assert_eq!(host, "192.168.1.100");
                        assert_eq!(port, 9877); // Default relay port
                        assert!(!tls);
                    }
                    _ => panic!("Expected SelfHosted mode"),
                }
            }
            _ => panic!("Expected Cloud endpoint"),
        }
    }

    #[test]
    fn test_parse_relay_with_port() {
        use crate::communication::network::cloud::CloudMode;

        let ep = parse_endpoint("camera@relay:relay.mycompany.com:8080").unwrap();
        match ep {
            Endpoint::Cloud { topic, mode } => {
                assert_eq!(topic, "camera");
                match mode {
                    CloudMode::SelfHosted { host, port, .. } => {
                        assert_eq!(host, "relay.mycompany.com");
                        assert_eq!(port, 8080);
                    }
                    _ => panic!("Expected SelfHosted mode"),
                }
            }
            _ => panic!("Expected Cloud endpoint"),
        }
    }

    #[test]
    fn test_parse_relay_tls() {
        use crate::communication::network::cloud::CloudMode;

        let ep = parse_endpoint("secure_data@relay-tls:relay.example.com").unwrap();
        match ep {
            Endpoint::Cloud { topic, mode } => {
                assert_eq!(topic, "secure_data");
                match mode {
                    CloudMode::SelfHosted { host, tls, .. } => {
                        assert_eq!(host, "relay.example.com");
                        assert!(tls);
                    }
                    _ => panic!("Expected SelfHosted mode"),
                }
            }
            _ => panic!("Expected Cloud endpoint"),
        }
    }

    #[test]
    fn test_parse_vpn_auto_detect() {
        use crate::communication::network::cloud::{CloudMode, VpnType};

        // Simple name -> Husarnet
        let ep = parse_endpoint("data@vpn:robot1").unwrap();
        match ep {
            Endpoint::Cloud { topic, mode } => {
                assert_eq!(topic, "data");
                match mode {
                    CloudMode::Vpn { vpn_type, peer } => {
                        assert_eq!(vpn_type, VpnType::Husarnet);
                        assert_eq!(peer, "robot1");
                    }
                    _ => panic!("Expected Vpn mode"),
                }
            }
            _ => panic!("Expected Cloud endpoint"),
        }
    }

    #[test]
    fn test_parse_vpn_explicit_type() {
        use crate::communication::network::cloud::{CloudMode, VpnType};

        let ep = parse_endpoint("telemetry@vpn/tailscale:myrobot.ts.net").unwrap();
        match ep {
            Endpoint::Cloud { topic, mode } => {
                assert_eq!(topic, "telemetry");
                match mode {
                    CloudMode::Vpn { vpn_type, peer } => {
                        assert_eq!(vpn_type, VpnType::Tailscale);
                        assert_eq!(peer, "myrobot.ts.net");
                    }
                    _ => panic!("Expected Vpn mode"),
                }
            }
            _ => panic!("Expected Cloud endpoint"),
        }
    }

    #[test]
    fn test_parse_cloud_error_empty_room() {
        let result = parse_endpoint("mytopic@cloud:");
        assert!(result.is_err());
    }

    #[test]
    fn test_parse_cloud_error_invalid_room_name() {
        let result = parse_endpoint("mytopic@cloud:room with spaces");
        assert!(result.is_err());
    }

    // ============================================
    // Zenoh Cloud endpoint parsing tests
    // ============================================

    #[test]
    fn test_parse_zenoh_cloud_default() {
        let ep = parse_endpoint("cmd_vel@zenoh:cloud").unwrap();
        match ep {
            Endpoint::ZenohCloud { topic, config } => {
                assert_eq!(topic, "cmd_vel");
                // Default should have auto-discovery enabled
                assert!(config.auto_discovery);
                // Should have monitoring enabled
                assert!(config.enable_monitoring);
            }
            _ => panic!("Expected ZenohCloud endpoint"),
        }
    }

    #[test]
    fn test_parse_zenoh_cloud_auto() {
        let ep = parse_endpoint("lidar@zenoh:cloud/auto").unwrap();
        match ep {
            Endpoint::ZenohCloud { topic, config } => {
                assert_eq!(topic, "lidar");
                assert!(config.auto_discovery);
            }
            _ => panic!("Expected ZenohCloud endpoint"),
        }
    }

    #[test]
    fn test_parse_zenoh_cloud_horus() {
        let ep = parse_endpoint("sensors@zenoh:cloud/horus").unwrap();
        match ep {
            Endpoint::ZenohCloud { topic, config } => {
                assert_eq!(topic, "sensors");
                // horus mode should have auto-discovery disabled
                assert!(!config.auto_discovery);
                // Should use HORUS routers
                assert!(!config.primary_router.is_empty());
            }
            _ => panic!("Expected ZenohCloud endpoint"),
        }
    }

    #[test]
    fn test_parse_zenoh_cloud_self_hosted_single() {
        let ep = parse_endpoint("telemetry@zenoh:cloud:tcp/router.mycompany.com:7447").unwrap();
        match ep {
            Endpoint::ZenohCloud { topic, config } => {
                assert_eq!(topic, "telemetry");
                assert_eq!(config.primary_router, "tcp/router.mycompany.com:7447");
                // Self-hosted should have auto-discovery disabled
                assert!(!config.auto_discovery);
            }
            _ => panic!("Expected ZenohCloud endpoint"),
        }
    }

    #[test]
    fn test_parse_zenoh_cloud_multiple_routers_failover() {
        let ep =
            parse_endpoint("data@zenoh:cloud:tcp/router1.local:7447,tcp/router2.local:7447")
                .unwrap();
        match ep {
            Endpoint::ZenohCloud { topic, config } => {
                assert_eq!(topic, "data");
                // First router becomes primary
                assert_eq!(config.primary_router, "tcp/router1.local:7447");
                // Second router should be in backup list
                assert!(config.backup_routers.contains(&"tcp/router2.local:7447".to_string()));
            }
            _ => panic!("Expected ZenohCloud endpoint"),
        }
    }

    #[test]
    fn test_parse_zenoh_cloud_vs_regular_zenoh() {
        // Regular zenoh connect should produce Zenoh variant, not ZenohCloud
        let ep_zenoh = parse_endpoint("topic@zenoh:tcp/host:7447").unwrap();
        match ep_zenoh {
            Endpoint::Zenoh { topic, connect, .. } => {
                assert_eq!(topic, "topic");
                assert_eq!(connect, Some("tcp/host:7447".to_string()));
            }
            _ => panic!("Expected Zenoh endpoint, not ZenohCloud"),
        }

        // zenoh:cloud should produce ZenohCloud variant
        let ep_cloud = parse_endpoint("topic@zenoh:cloud").unwrap();
        match ep_cloud {
            Endpoint::ZenohCloud { topic, .. } => {
                assert_eq!(topic, "topic");
            }
            _ => panic!("Expected ZenohCloud endpoint"),
        }
    }
}
