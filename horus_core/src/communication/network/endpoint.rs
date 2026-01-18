//! # HORUS Endpoint System
//!
//! This module provides the endpoint parsing and building system for HORUS network
//! communication. Endpoints define where and how messages are routed between nodes.
//!
//! ## Quick Start
//!
//! ```rust,ignore
//! use horus_core::communication::network::{parse_endpoint, EndpointBuilder, Endpoint};
//!
//! // Parse endpoint strings (runtime validation)
//! let ep = parse_endpoint("sensor_data").unwrap();           // Local
//! let ep = parse_endpoint("cmd_vel@*").unwrap();             // Multicast LAN
//! let ep = parse_endpoint("odom@robot.local").unwrap();      // mDNS
//! let ep = parse_endpoint("video@p2p:a3f7-k2m9-p4n8").unwrap(); // P2P
//!
//! // Or use the type-safe builder (IDE autocomplete)
//! let ep = EndpointBuilder::new("sensor_data")
//!     .multicast()
//!     .build();
//! ```
//!
//! ## Endpoint Syntax Reference
//!
//! | Pattern | Transport | Use Case |
//! |---------|-----------|----------|
//! | `topic` | Shared Memory | Same process, lowest latency (<500ns) |
//! | `topic@localhost` | Unix Socket | Same machine, different processes |
//! | `topic@192.168.1.5` | Direct UDP | Known IP, no discovery overhead |
//! | `topic@192.168.1.5:9000` | Direct UDP | Custom port |
//! | `topic@*` | Multicast | LAN discovery, auto-find peers |
//! | `topic@robot.local` | mDNS | Hostname-based, zero-config |
//! | `topic@router` | Central Broker | Firewall traversal, star topology |
//! | `topic@zenoh` | Zenoh Mesh | Multi-robot, cloud-ready |
//! | `topic@zenoh/ros2` | Zenoh ROS2 | ROS2 interop with DDS bridge |
//! | `topic@zenoh:cloud` | Zenoh Cloud | WAN connectivity, auto-discovery |
//! | `topic@p2p:xxxx-yyyy-zzzz` | P2P NAT | Peer-to-peer with hole-punching |
//! | `topic@cloud:room` | HORUS Cloud | Room-based WAN connectivity |
//!
//! ## Compile-Time Validation
//!
//! For compile-time validation, use the `endpoint!` macro from `horus_macros`:
//!
//! ```rust,ignore
//! use horus_macros::endpoint;
//!
//! // Validated at compile time - typos caught before runtime
//! let ep = endpoint!("sensor_data@*");
//!
//! // This would fail at compile time:
//! // let ep = endpoint!("@invalid");  // Error: Topic name cannot be empty
//! ```
//!
//! ## Examples by Transport
//!
//! ### Local (Same Process)
//! ```rust,ignore
//! // Fastest option - zero-copy shared memory
//! let ep = parse_endpoint("imu_data").unwrap();
//! let ep = EndpointBuilder::new("imu_data").local().build();
//! ```
//!
//! ### Multicast Discovery (LAN)
//! ```rust,ignore
//! // Robots automatically discover each other
//! let ep = parse_endpoint("cmd_vel@*").unwrap();
//! let ep = EndpointBuilder::new("cmd_vel").multicast().build();
//! ```
//!
//! ### mDNS (Zero-Config Networking)
//! ```rust,ignore
//! // Connect by hostname, no IP configuration needed
//! let ep = parse_endpoint("lidar@robot-arm.local").unwrap();
//! let ep = parse_endpoint("lidar@robot-arm.local:9000").unwrap();
//! let ep = EndpointBuilder::new("lidar").mdns("robot-arm").port(9000).build();
//! ```
//!
//! ### Direct UDP (Known IP)
//! ```rust,ignore
//! // When you know the target IP address
//! let ep = parse_endpoint("telemetry@192.168.1.50").unwrap();
//! let ep = parse_endpoint("telemetry@192.168.1.50:9000").unwrap();
//! let ep = EndpointBuilder::new("telemetry")
//!     .direct("192.168.1.50".parse().unwrap())
//!     .port(9000)
//!     .build();
//! ```
//!
//! ### Zenoh (Multi-Robot Mesh)
//! ```rust,ignore
//! // Zenoh for scalable multi-robot communication
//! let ep = parse_endpoint("fleet_status@zenoh").unwrap();
//! let ep = parse_endpoint("/scan@zenoh/ros2").unwrap();  // ROS2 mode
//! let ep = EndpointBuilder::new("/scan").zenoh().ros2_mode().build();
//!
//! // Connect to specific router
//! let ep = parse_endpoint("data@zenoh:tcp/192.168.1.1:7447").unwrap();
//! let ep = EndpointBuilder::new("data").zenoh().connect("tcp/192.168.1.1:7447").build();
//! ```
//!
//! ### P2P with NAT Traversal
//! ```rust,ignore
//! // Connect to remote peer through NAT
//! let ep = parse_endpoint("video@p2p:a3f7-k2m9-p4n8").unwrap();         // Auto strategy
//! let ep = parse_endpoint("video@p2p:a3f7-k2m9-p4n8/stun").unwrap();    // STUN only
//! let ep = parse_endpoint("video@p2p:a3f7-k2m9-p4n8/turn").unwrap();    // TURN relay
//! ```
//!
//! ### Cloud Connectivity
//! ```rust,ignore
//! // HORUS Cloud for WAN connectivity
//! let ep = parse_endpoint("fleet@cloud:factory-east").unwrap();
//! let ep = EndpointBuilder::new("fleet").cloud_room("factory-east").build();
//!
//! // Zenoh cloud with auto-discovery
//! let ep = parse_endpoint("status@zenoh:cloud").unwrap();
//! let ep = EndpointBuilder::new("status").zenoh_cloud().build();
//! ```

use std::net::IpAddr;

use super::cloud::CloudMode;
use super::p2p::{P2pStrategy, PeerId};
use super::zenoh_config::ZenohCloudConfig;

/// Endpoint types for HORUS network communication.
///
/// Each variant represents a different transport mechanism with unique trade-offs
/// for latency, reliability, and network requirements.
///
/// # Choosing the Right Endpoint
///
/// | Scenario | Recommended | Latency | Reliability |
/// |----------|-------------|---------|-------------|
/// | Same process | `Local` | <500ns | ⭐⭐⭐⭐⭐ |
/// | Same machine | `Localhost` | ~1µs | ⭐⭐⭐⭐⭐ |
/// | Same LAN, dynamic | `Multicast` | ~100µs | ⭐⭐⭐⭐ |
/// | Same LAN, hostname | `Mdns` | ~100µs | ⭐⭐⭐⭐ |
/// | Same LAN, fixed IP | `Direct` | ~50µs | ⭐⭐⭐⭐ |
/// | Multi-robot fleet | `Zenoh` | ~200µs | ⭐⭐⭐⭐⭐ |
/// | Through NAT/firewall | `P2p` | ~5-50ms | ⭐⭐⭐ |
/// | Over internet | `Cloud` / `ZenohCloud` | ~20-100ms | ⭐⭐⭐⭐ |
///
/// # Example
///
/// ```rust,ignore
/// use horus_core::communication::network::{parse_endpoint, Endpoint};
///
/// let ep = parse_endpoint("cmd_vel@*").unwrap();
/// match ep {
///     Endpoint::Multicast { topic } => println!("LAN discovery for {}", topic),
///     Endpoint::Direct { topic, host, port } => println!("Direct to {}:{}", host, port),
///     _ => {}
/// }
/// ```
#[derive(Debug, Clone, PartialEq)]
pub enum Endpoint {
    /// Local shared memory communication.
    ///
    /// **Syntax:** `"topic"`
    ///
    /// The fastest option for same-process communication using zero-copy shared memory.
    /// Messages are passed by reference with sub-microsecond latency.
    ///
    /// # Example
    /// ```rust,ignore
    /// let ep = parse_endpoint("sensor_data").unwrap();
    /// assert!(matches!(ep, Endpoint::Local { .. }));
    /// ```
    Local { topic: String },

    /// Localhost communication via Unix socket.
    ///
    /// **Syntax:** `"topic@localhost"` or `"topic@127.0.0.1"`
    ///
    /// For communication between processes on the same machine.
    /// Uses Unix domain sockets on Linux/macOS for low latency.
    ///
    /// # Example
    /// ```rust,ignore
    /// let ep = parse_endpoint("cmd_vel@localhost").unwrap();
    /// ```
    Localhost {
        topic: String,
        /// Optional port override (default uses Unix socket)
        port: Option<u16>,
    },

    /// Direct UDP connection to a known IP address.
    ///
    /// **Syntax:** `"topic@192.168.1.5"` or `"topic@192.168.1.5:9000"`
    ///
    /// Best for point-to-point communication when you know the target IP.
    /// No discovery overhead, minimal latency. Supports both IPv4 and IPv6.
    ///
    /// # Examples
    /// ```rust,ignore
    /// // IPv4 with default port (9870)
    /// let ep = parse_endpoint("telemetry@192.168.1.50").unwrap();
    ///
    /// // IPv4 with custom port
    /// let ep = parse_endpoint("telemetry@192.168.1.50:9000").unwrap();
    ///
    /// // IPv6 with port (brackets required)
    /// let ep = parse_endpoint("telemetry@[2001:db8::1]:9000").unwrap();
    /// ```
    Direct {
        topic: String,
        /// Target IP address (IPv4 or IPv6)
        host: IpAddr,
        /// Port number (default: 9870)
        port: u16,
    },

    /// Multicast-based LAN discovery.
    ///
    /// **Syntax:** `"topic@*"`
    ///
    /// Robots on the same LAN automatically discover each other using UDP multicast.
    /// Best for dynamic environments where IPs are unknown.
    ///
    /// **Note:** Requires multicast support on the network.
    ///
    /// # Example
    /// ```rust,ignore
    /// let ep = parse_endpoint("cmd_vel@*").unwrap();
    /// // All robots on LAN will discover each other
    /// ```
    Multicast { topic: String },

    /// mDNS/DNS-SD hostname resolution.
    ///
    /// **Syntax:** `"topic@hostname.local"` or `"topic@hostname.local:9000"`
    ///
    /// Zero-config networking using `.local` hostnames. Automatically resolves
    /// hostnames via mDNS, perfect for robots with meaningful names.
    ///
    /// # Examples
    /// ```rust,ignore
    /// // Connect to robot by name
    /// let ep = parse_endpoint("lidar@robot-arm.local").unwrap();
    ///
    /// // With custom port
    /// let ep = parse_endpoint("camera@manipulator.local:8080").unwrap();
    /// ```
    Mdns {
        topic: String,
        /// Hostname without `.local` suffix (e.g., `"robot"` for `"robot.local"`)
        hostname: String,
        /// Optional port (default: 9870)
        port: Option<u16>,
    },

    /// Central router/broker topology.
    ///
    /// **Syntax:** `"topic@router"` or `"topic@router:7777"`
    ///
    /// All messages route through a central broker. Useful for:
    /// - Firewall traversal (only broker needs public access)
    /// - Message logging/recording at a single point
    /// - Star topology with guaranteed delivery
    ///
    /// # Example
    /// ```rust,ignore
    /// let ep = parse_endpoint("commands@router").unwrap();
    /// let ep = parse_endpoint("commands@router:7777").unwrap();
    /// ```
    Router {
        topic: String,
        /// Router host (default: localhost)
        host: Option<IpAddr>,
        /// Router port (default: 7777)
        port: Option<u16>,
    },

    /// Zenoh mesh transport.
    ///
    /// **Syntax:** `"topic@zenoh"`, `"topic@zenoh/ros2"`, `"topic@zenoh:tcp/host:port"`
    ///
    /// Zenoh provides:
    /// - Scalable multi-robot communication
    /// - Automatic peer discovery
    /// - ROS2 interop via DDS bridge
    /// - Cloud connectivity
    ///
    /// # Examples
    /// ```rust,ignore
    /// // Basic Zenoh mesh
    /// let ep = parse_endpoint("fleet_status@zenoh").unwrap();
    ///
    /// // ROS2 compatibility mode (CDR serialization, DDS topic naming)
    /// let ep = parse_endpoint("/scan@zenoh/ros2").unwrap();
    ///
    /// // Connect to specific router
    /// let ep = parse_endpoint("data@zenoh:tcp/192.168.1.1:7447").unwrap();
    /// ```
    Zenoh {
        topic: String,
        /// If true, use ROS2-compatible topic naming and CDR serialization
        ros2_mode: bool,
        /// Optional router endpoint to connect to (e.g., `"tcp/host:port"`)
        connect: Option<String>,
    },

    /// Zenoh cloud transport with enhanced features.
    ///
    /// **Syntax:**
    /// - `"topic@zenoh:cloud"` - Auto-connect to HORUS cloud
    /// - `"topic@zenoh:cloud/auto"` - Auto-discover routers via mDNS
    /// - `"topic@zenoh:cloud:tcp/host:port"` - Self-hosted router
    /// - `"topic@zenoh:cloud:tcp/h1:p1,tcp/h2:p2"` - Multiple routers with failover
    ///
    /// Features beyond basic Zenoh:
    /// - Automatic failover between routers
    /// - Health monitoring and reconnection
    /// - Cloud-optimized QoS settings
    ///
    /// # Examples
    /// ```rust,ignore
    /// // HORUS cloud with auto-discovery
    /// let ep = parse_endpoint("telemetry@zenoh:cloud").unwrap();
    ///
    /// // Self-hosted with failover
    /// let ep = parse_endpoint("data@zenoh:cloud:tcp/router1:7447,tcp/router2:7447").unwrap();
    /// ```
    ZenohCloud {
        topic: String,
        /// Cloud configuration with failover and monitoring settings
        config: ZenohCloudConfig,
    },

    /// P2P connection with NAT traversal.
    ///
    /// **Syntax:** `"topic@p2p:xxxx-yyyy-zzzz"` or `"topic@p2p:xxxx-yyyy-zzzz/strategy"`
    ///
    /// Connect directly to a remote peer, even behind NAT/firewalls.
    /// Uses ICE-like connectivity checks with multiple strategies:
    ///
    /// | Strategy | Description | Latency | Reliability |
    /// |----------|-------------|---------|-------------|
    /// | `auto` | Try all, use best | Varies | ⭐⭐⭐⭐⭐ |
    /// | `direct` | Direct only (fails if NAT) | ~5ms | ⭐⭐⭐ |
    /// | `stun` | STUN hole-punching | ~10ms | ⭐⭐⭐⭐ |
    /// | `turn` | TURN relay (always works) | ~50ms | ⭐⭐⭐⭐⭐ |
    ///
    /// # Examples
    /// ```rust,ignore
    /// // Auto strategy (recommended)
    /// let ep = parse_endpoint("video@p2p:a3f7-k2m9-p4n8").unwrap();
    ///
    /// // Force STUN for lower latency
    /// let ep = parse_endpoint("video@p2p:a3f7-k2m9-p4n8/stun").unwrap();
    ///
    /// // Force TURN for guaranteed connectivity
    /// let ep = parse_endpoint("control@p2p:a3f7-k2m9-p4n8/turn").unwrap();
    /// ```
    P2p {
        topic: String,
        /// Target peer ID (format: `xxxx-yyyy-zzzz`)
        peer_id: PeerId,
        /// Connection strategy (auto, direct, stun, turn)
        strategy: P2pStrategy,
    },

    /// Cloud/WAN connectivity.
    ///
    /// **Syntax:**
    /// - `"topic@cloud:room-name"` - HORUS Cloud room
    /// - `"topic@relay:host:port"` - Self-hosted relay
    /// - `"topic@relay-tls:host:port"` - TLS-secured relay
    /// - `"topic@vpn/tailscale:peer"` - VPN integration
    ///
    /// For connecting robots across the internet without complex NAT configuration.
    ///
    /// # Examples
    /// ```rust,ignore
    /// // HORUS Cloud room
    /// let ep = parse_endpoint("fleet@cloud:factory-east").unwrap();
    ///
    /// // Self-hosted relay with TLS
    /// let ep = parse_endpoint("telemetry@relay-tls:relay.example.com:9000").unwrap();
    ///
    /// // Tailscale VPN
    /// let ep = parse_endpoint("control@vpn/tailscale:robot-100").unwrap();
    /// ```
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

/// Parse an endpoint string into an [`Endpoint`] enum.
///
/// This is the primary way to create endpoints from configuration files,
/// environment variables, or user input. For compile-time validated endpoints,
/// use the `endpoint!` macro from `horus_macros`.
///
/// # Syntax
///
/// The general format is `topic@location[:options]` where:
/// - `topic` - The message topic name (required)
/// - `@location` - Transport and address (optional, defaults to local)
/// - `:options` - Transport-specific options (optional)
///
/// # Supported Formats
///
/// | Format | Result |
/// |--------|--------|
/// | `"topic"` | Local shared memory |
/// | `"topic@localhost"` | Localhost Unix socket |
/// | `"topic@127.0.0.1"` | Localhost (IPv4) |
/// | `"topic@::1"` | Localhost (IPv6) |
/// | `"topic@192.168.1.5"` | Direct UDP (default port 9870) |
/// | `"topic@192.168.1.5:9000"` | Direct UDP (custom port) |
/// | `"topic@[2001:db8::1]:9000"` | Direct UDP IPv6 (custom port) |
/// | `"topic@*"` | Multicast discovery |
/// | `"topic@robot.local"` | mDNS hostname |
/// | `"topic@robot.local:9000"` | mDNS with port |
/// | `"topic@router"` | Central router |
/// | `"topic@router:7777"` | Router with port |
/// | `"topic@zenoh"` | Zenoh mesh |
/// | `"topic@zenoh/ros2"` | Zenoh ROS2 mode |
/// | `"topic@zenoh:tcp/host:port"` | Zenoh with router |
/// | `"topic@zenoh:cloud"` | Zenoh cloud (auto-discovery) |
/// | `"topic@zenoh:cloud/auto"` | Zenoh cloud (explicit auto) |
/// | `"topic@zenoh:cloud/horus"` | Zenoh cloud (HORUS hosted) |
/// | `"topic@zenoh:cloud:tcp/h:p"` | Zenoh cloud (self-hosted) |
/// | `"topic@p2p:xxxx-yyyy-zzzz"` | P2P (auto strategy) |
/// | `"topic@p2p:xxxx-yyyy-zzzz/stun"` | P2P (STUN only) |
/// | `"topic@p2p:xxxx-yyyy-zzzz/turn"` | P2P (TURN relay) |
/// | `"topic@cloud:room"` | HORUS Cloud room |
/// | `"topic@relay:host:port"` | Self-hosted relay |
/// | `"topic@relay-tls:host:port"` | TLS relay |
/// | `"topic@vpn/tailscale:peer"` | VPN integration |
///
/// # Examples
///
/// ```rust,ignore
/// use horus_core::communication::network::parse_endpoint;
///
/// // Local endpoint
/// let ep = parse_endpoint("sensor_data").unwrap();
///
/// // Multicast for LAN discovery
/// let ep = parse_endpoint("cmd_vel@*").unwrap();
///
/// // Direct UDP to IP
/// let ep = parse_endpoint("telemetry@192.168.1.50:9000").unwrap();
///
/// // mDNS hostname
/// let ep = parse_endpoint("lidar@robot-arm.local").unwrap();
///
/// // P2P with STUN
/// let ep = parse_endpoint("video@p2p:a3f7-k2m9-p4n8/stun").unwrap();
/// ```
///
/// # Errors
///
/// Returns an error string if the endpoint is invalid:
///
/// ```rust,ignore
/// // Empty topic
/// assert!(parse_endpoint("@192.168.1.1").is_err());
///
/// // Invalid IP address
/// assert!(parse_endpoint("topic@999.999.999.999").is_err());
///
/// // Invalid peer ID format
/// assert!(parse_endpoint("topic@p2p:invalid").is_err());
/// ```
///
/// # See Also
///
/// - [`EndpointBuilder`] - Type-safe builder with IDE autocomplete
/// - `endpoint!` macro in `horus_macros` - Compile-time validation
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

// ===========================================================================
// Type-Safe Endpoint Builder API
// ===========================================================================

/// Type-safe builder for creating endpoints with IDE autocomplete.
///
/// The builder pattern provides:
/// - **Full IDE autocomplete** - Discover available transports and options
/// - **Compile-time type safety** - Invalid configurations caught at compile time
/// - **Method chaining** - Fluent API for readable endpoint construction
/// - **Discoverability** - See all options via autocomplete
///
/// # When to Use Builder vs String Parsing
///
/// | Use Builder When | Use `parse_endpoint()` When |
/// |------------------|----------------------------|
/// | Building in code | Loading from config files |
/// | Need IDE hints | Need runtime flexibility |
/// | Complex endpoints | Simple, static endpoints |
///
/// # Basic Examples
///
/// ```rust,ignore
/// use horus_core::communication::network::{EndpointBuilder, P2pStrategy};
/// use std::net::IpAddr;
///
/// // Local shared memory (default)
/// let ep = EndpointBuilder::new("sensor_data").build();
/// let ep = EndpointBuilder::new("sensor_data").local().build();  // Explicit
///
/// // Multicast discovery
/// let ep = EndpointBuilder::new("cmd_vel").multicast().build();
///
/// // Direct UDP to IP
/// let ep = EndpointBuilder::new("telemetry")
///     .direct("192.168.1.50".parse().unwrap())
///     .port(9000)
///     .build();
///
/// // mDNS hostname
/// let ep = EndpointBuilder::new("lidar").mdns("robot-arm").build();
///
/// // Zenoh with ROS2 mode
/// let ep = EndpointBuilder::new("/scan")
///     .zenoh()
///     .ros2_mode()
///     .build();
///
/// // P2P with STUN
/// let ep = EndpointBuilder::new("video")
///     .p2p("a3f7-k2m9-p4n8".parse().unwrap())
///     .strategy(P2pStrategy::Stun)
///     .build();
///
/// // Cloud room
/// let ep = EndpointBuilder::new("fleet").cloud_room("factory-east").build();
/// ```
///
/// # Method Chaining Order
///
/// The builder supports flexible method order, but the recommended pattern is:
///
/// ```rust,ignore
/// EndpointBuilder::new("topic")  // 1. Topic name
///     .transport()                // 2. Transport type (multicast, direct, zenoh, etc.)
///     .options()                  // 3. Transport-specific options
///     .build();                   // 4. Build final endpoint
/// ```
///
/// # Converting to String
///
/// ```rust,ignore
/// let ep = EndpointBuilder::new("cmd_vel").multicast().build();
/// assert_eq!(ep.to_string(), "cmd_vel@*");
///
/// let ep = EndpointBuilder::new("data")
///     .direct("192.168.1.5".parse().unwrap())
///     .port(9000)
///     .build();
/// assert_eq!(ep.to_string(), "data@192.168.1.5:9000");
/// ```
///
/// # Alternative: `Endpoint::builder()`
///
/// You can also start from the `Endpoint` type:
///
/// ```rust,ignore
/// let ep = Endpoint::builder("sensor_data").multicast().build();
/// ```
#[derive(Debug, Clone)]
pub struct EndpointBuilder {
    topic: String,
    config: EndpointConfig,
}

#[derive(Debug, Clone)]
enum EndpointConfig {
    Local,
    Localhost { port: Option<u16> },
    Direct { host: IpAddr, port: u16 },
    Multicast,
    Mdns { hostname: String, port: Option<u16> },
    Router { host: Option<IpAddr>, port: Option<u16> },
    Zenoh { ros2_mode: bool, connect: Option<String> },
    ZenohCloud { config: ZenohCloudConfig },
    P2p { peer_id: PeerId, strategy: P2pStrategy },
    Cloud { mode: CloudMode },
}

impl EndpointBuilder {
    /// Create a new endpoint builder for the given topic.
    ///
    /// # Example
    ///
    /// ```rust,ignore
    /// let builder = EndpointBuilder::new("sensor_data");
    /// ```
    pub fn new(topic: impl Into<String>) -> Self {
        Self {
            topic: topic.into(),
            config: EndpointConfig::Local,
        }
    }

    /// Build a local (same-process) endpoint.
    ///
    /// This is the default if no transport is specified.
    pub fn local(mut self) -> Self {
        self.config = EndpointConfig::Local;
        self
    }

    /// Build a localhost endpoint (Unix socket on same machine).
    pub fn localhost(mut self) -> Self {
        self.config = EndpointConfig::Localhost { port: None };
        self
    }

    /// Build a multicast discovery endpoint.
    ///
    /// Robots on the same LAN will automatically discover each other.
    pub fn multicast(mut self) -> Self {
        self.config = EndpointConfig::Multicast;
        self
    }

    /// Build a direct UDP endpoint to a specific IP address.
    ///
    /// Uses the default port (9870) unless `.port()` is called.
    pub fn direct(mut self, host: IpAddr) -> Self {
        self.config = EndpointConfig::Direct {
            host,
            port: DEFAULT_PORT,
        };
        self
    }

    /// Build an mDNS endpoint for a `.local` hostname.
    ///
    /// # Example
    ///
    /// ```rust,ignore
    /// // Connects to "robot-arm.local"
    /// let ep = EndpointBuilder::new("cmd_vel")
    ///     .mdns("robot-arm")
    ///     .build();
    /// ```
    pub fn mdns(mut self, hostname: impl Into<String>) -> Self {
        self.config = EndpointConfig::Mdns {
            hostname: hostname.into(),
            port: None,
        };
        self
    }

    /// Build a router (central broker) endpoint.
    pub fn router(mut self) -> Self {
        self.config = EndpointConfig::Router {
            host: None,
            port: None,
        };
        self
    }

    /// Build a Zenoh transport endpoint.
    ///
    /// Use `.ros2_mode()` for ROS2 compatibility.
    /// Use `.connect()` to specify a router.
    pub fn zenoh(mut self) -> Self {
        self.config = EndpointConfig::Zenoh {
            ros2_mode: false,
            connect: None,
        };
        self
    }

    /// Build a Zenoh cloud endpoint with auto-discovery.
    pub fn zenoh_cloud(mut self) -> Self {
        self.config = EndpointConfig::ZenohCloud {
            config: ZenohCloudConfig::auto(),
        };
        self
    }

    /// Build a P2P endpoint with NAT traversal.
    ///
    /// Uses auto strategy by default (tries Direct → STUN → TURN).
    pub fn p2p(mut self, peer_id: PeerId) -> Self {
        self.config = EndpointConfig::P2p {
            peer_id,
            strategy: P2pStrategy::Auto,
        };
        self
    }

    /// Build a cloud room endpoint.
    ///
    /// # Example
    ///
    /// ```rust,ignore
    /// let ep = EndpointBuilder::new("fleet_status")
    ///     .cloud_room("factory-east")
    ///     .build();
    /// ```
    pub fn cloud_room(mut self, room: impl Into<String>) -> Self {
        self.config = EndpointConfig::Cloud {
            mode: CloudMode::HorusCloud {
                room: room.into(),
                auth_key: None,
            },
        };
        self
    }

    /// Set the port for transports that support it.
    ///
    /// Works with: localhost, direct, mdns, router.
    pub fn port(mut self, port: u16) -> Self {
        match &mut self.config {
            EndpointConfig::Localhost { port: p } => *p = Some(port),
            EndpointConfig::Direct { port: p, .. } => *p = port,
            EndpointConfig::Mdns { port: p, .. } => *p = Some(port),
            EndpointConfig::Router { port: p, .. } => *p = Some(port),
            _ => {} // Ignore for other transports
        }
        self
    }

    /// Set the host/router address for transports that support it.
    pub fn host(mut self, host: IpAddr) -> Self {
        match &mut self.config {
            EndpointConfig::Direct { host: h, .. } => *h = host,
            EndpointConfig::Router { host: h, .. } => *h = Some(host),
            _ => {} // Ignore for other transports
        }
        self
    }

    /// Enable ROS2 mode for Zenoh transport.
    ///
    /// Uses ROS2-compatible topic naming and CDR serialization.
    pub fn ros2_mode(mut self) -> Self {
        if let EndpointConfig::Zenoh { ros2_mode, .. } = &mut self.config {
            *ros2_mode = true;
        }
        self
    }

    /// Set Zenoh connect endpoint (router address).
    pub fn connect(mut self, endpoint: impl Into<String>) -> Self {
        if let EndpointConfig::Zenoh { connect, .. } = &mut self.config {
            *connect = Some(endpoint.into());
        }
        self
    }

    /// Set P2P connection strategy.
    pub fn strategy(mut self, strat: P2pStrategy) -> Self {
        if let EndpointConfig::P2p { strategy, .. } = &mut self.config {
            *strategy = strat;
        }
        self
    }

    /// Scope the topic to a room for isolation.
    ///
    /// This prefixes the topic with `__room::{room_name}::` to provide
    /// client-side room isolation. Topics in different rooms won't interfere
    /// with each other even when using the same transport.
    ///
    /// # Example
    ///
    /// ```rust,ignore
    /// use horus_core::communication::network::EndpointBuilder;
    ///
    /// // Create a room-scoped endpoint
    /// let ep = EndpointBuilder::new("sensor/lidar")
    ///     .room_scoped("production-fleet")
    ///     .multicast()
    ///     .build();
    ///
    /// // Topic becomes: __room::production-fleet::sensor/lidar
    /// // Only nodes in the same room will receive messages
    /// ```
    ///
    /// # Use Cases
    ///
    /// - **Multi-tenant**: Multiple robot fleets sharing infrastructure
    /// - **Environments**: Separate development, staging, production
    /// - **Privacy**: Isolate teams working on the same network
    pub fn room_scoped(mut self, room_name: impl Into<String>) -> Self {
        let room = room_name.into();
        self.topic = crate::communication::network::cloud::scope_topic_to_room(&room, &self.topic);
        self
    }

    /// Scope the topic using a RoomHandle.
    ///
    /// This is the preferred method when using the RoomRegistry for
    /// managing room memberships.
    ///
    /// # Example
    ///
    /// ```rust,ignore
    /// use horus_core::communication::network::{EndpointBuilder, RoomRegistry};
    ///
    /// let registry = RoomRegistry::new();
    /// let handle = registry.join_room("factory-1", Some("secret123"))?;
    ///
    /// // Create endpoint scoped to the room
    /// let ep = EndpointBuilder::new("robot/status")
    ///     .with_room(&handle)
    ///     .multicast()
    ///     .build();
    /// ```
    pub fn with_room(mut self, handle: &crate::communication::network::cloud::RoomHandle) -> Self {
        self.topic = handle.scope_topic(&self.topic);
        self
    }

    /// Build the final Endpoint.
    pub fn build(self) -> Endpoint {
        match self.config {
            EndpointConfig::Local => Endpoint::Local { topic: self.topic },
            EndpointConfig::Localhost { port } => Endpoint::Localhost {
                topic: self.topic,
                port,
            },
            EndpointConfig::Direct { host, port } => Endpoint::Direct {
                topic: self.topic,
                host,
                port,
            },
            EndpointConfig::Multicast => Endpoint::Multicast { topic: self.topic },
            EndpointConfig::Mdns { hostname, port } => Endpoint::Mdns {
                topic: self.topic,
                hostname,
                port,
            },
            EndpointConfig::Router { host, port } => Endpoint::Router {
                topic: self.topic,
                host,
                port,
            },
            EndpointConfig::Zenoh {
                ros2_mode,
                connect,
            } => Endpoint::Zenoh {
                topic: self.topic,
                ros2_mode,
                connect,
            },
            EndpointConfig::ZenohCloud { config } => Endpoint::ZenohCloud {
                topic: self.topic,
                config,
            },
            EndpointConfig::P2p { peer_id, strategy } => Endpoint::P2p {
                topic: self.topic,
                peer_id,
                strategy,
            },
            EndpointConfig::Cloud { mode } => Endpoint::Cloud {
                topic: self.topic,
                mode,
            },
        }
    }

    /// Build and convert to string representation.
    pub fn to_string(&self) -> String {
        self.clone().build().to_string()
    }
}

impl Endpoint {
    /// Create a builder for constructing endpoints.
    ///
    /// # Example
    ///
    /// ```rust,ignore
    /// let ep = Endpoint::builder("sensor_data")
    ///     .multicast()
    ///     .build();
    /// ```
    pub fn builder(topic: impl Into<String>) -> EndpointBuilder {
        EndpointBuilder::new(topic)
    }
}

impl std::fmt::Display for Endpoint {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Endpoint::Local { topic } => write!(f, "{}", topic),
            Endpoint::Localhost { topic, port: None } => write!(f, "{}@localhost", topic),
            Endpoint::Localhost {
                topic,
                port: Some(p),
            } => write!(f, "{}@localhost:{}", topic, p),
            Endpoint::Direct { topic, host, port } => {
                if port == &DEFAULT_PORT {
                    write!(f, "{}@{}", topic, host)
                } else {
                    write!(f, "{}@{}:{}", topic, host, port)
                }
            }
            Endpoint::Multicast { topic } => write!(f, "{}@*", topic),
            Endpoint::Mdns {
                topic,
                hostname,
                port: None,
            } => write!(f, "{}@{}.local", topic, hostname),
            Endpoint::Mdns {
                topic,
                hostname,
                port: Some(p),
            } => write!(f, "{}@{}.local:{}", topic, hostname, p),
            Endpoint::Router {
                topic,
                host: None,
                port: None,
            } => write!(f, "{}@router", topic),
            Endpoint::Router {
                topic,
                host: _,
                port: Some(p),
            } => write!(f, "{}@router:{}", topic, p),
            Endpoint::Router { topic, .. } => write!(f, "{}@router", topic),
            Endpoint::Zenoh {
                topic,
                ros2_mode: false,
                connect: None,
            } => write!(f, "{}@zenoh", topic),
            Endpoint::Zenoh {
                topic,
                ros2_mode: true,
                connect: None,
            } => write!(f, "{}@zenoh/ros2", topic),
            Endpoint::Zenoh {
                topic,
                ros2_mode: false,
                connect: Some(c),
            } => write!(f, "{}@zenoh:{}", topic, c),
            Endpoint::Zenoh {
                topic,
                ros2_mode: true,
                connect: Some(c),
            } => write!(f, "{}@zenoh/ros2:{}", topic, c),
            Endpoint::ZenohCloud { topic, config } => {
                if config.auto_discovery {
                    write!(f, "{}@zenoh:cloud", topic)
                } else if !config.primary_router.is_empty() {
                    write!(f, "{}@zenoh:cloud:{}", topic, config.primary_router)
                } else {
                    write!(f, "{}@zenoh:cloud/horus", topic)
                }
            }
            Endpoint::P2p {
                topic,
                peer_id,
                strategy,
            } => {
                let strat_str = match strategy {
                    P2pStrategy::Auto => "",
                    P2pStrategy::Direct => "/direct",
                    P2pStrategy::Stun => "/stun",
                    P2pStrategy::Turn => "/turn",
                };
                write!(f, "{}@p2p:{}{}", topic, peer_id.short_id, strat_str)
            }
            Endpoint::Cloud { topic, mode } => match mode {
                CloudMode::HorusCloud { room, auth_key: None } => {
                    write!(f, "{}@cloud:{}", topic, room)
                }
                CloudMode::HorusCloud {
                    room,
                    auth_key: Some(key),
                } => write!(f, "{}@cloud:{}/{}", topic, room, key),
                CloudMode::SelfHosted { host, port, tls } => {
                    let prefix = if *tls { "relay-tls" } else { "relay" };
                    write!(f, "{}@{}:{}:{}", topic, prefix, host, port)
                }
                CloudMode::Vpn { vpn_type, peer } => {
                    write!(f, "{}@vpn/{}:{}", topic, vpn_type, peer)
                }
            },
        }
    }
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

    // =========================================================================
    // Room-Scoped Endpoint Builder Tests
    // =========================================================================

    #[test]
    fn test_builder_room_scoped() {
        let ep = EndpointBuilder::new("sensor/lidar")
            .room_scoped("production")
            .multicast()
            .build();

        match ep {
            Endpoint::Multicast { topic } => {
                assert_eq!(topic, "__room::production::sensor/lidar");
            }
            _ => panic!("Expected Multicast endpoint"),
        }
    }

    #[test]
    fn test_builder_room_scoped_with_direct() {
        use std::net::IpAddr;

        let ep = EndpointBuilder::new("robot/status")
            .room_scoped("factory-east")
            .direct("192.168.1.50".parse::<IpAddr>().unwrap())
            .port(9000)
            .build();

        match ep {
            Endpoint::Direct { topic, port, .. } => {
                assert_eq!(topic, "__room::factory-east::robot/status");
                assert_eq!(port, 9000);
            }
            _ => panic!("Expected Direct endpoint"),
        }
    }

    #[test]
    fn test_builder_with_room_handle() {
        use crate::communication::network::cloud::RoomRegistry;

        let registry = RoomRegistry::new();
        let handle = registry.join_room("dev-team", None).unwrap();

        let ep = EndpointBuilder::new("cmd_vel")
            .with_room(&handle)
            .local()
            .build();

        match ep {
            Endpoint::Local { topic } => {
                assert_eq!(topic, "__room::dev-team::cmd_vel");
            }
            _ => panic!("Expected Local endpoint"),
        }
    }

    #[test]
    fn test_builder_room_scoped_preserves_nested_topic() {
        let ep = EndpointBuilder::new("robot/arm/left/joint/1")
            .room_scoped("my-fleet")
            .zenoh()
            .ros2_mode()
            .build();

        match ep {
            Endpoint::Zenoh {
                topic, ros2_mode, ..
            } => {
                assert_eq!(topic, "__room::my-fleet::robot/arm/left/joint/1");
                assert!(ros2_mode);
            }
            _ => panic!("Expected Zenoh endpoint"),
        }
    }

    #[test]
    fn test_builder_room_scoped_multiple_endpoints_same_room() {
        let room = "shared-room";

        let ep1 = EndpointBuilder::new("topic_a")
            .room_scoped(room)
            .multicast()
            .build();

        let ep2 = EndpointBuilder::new("topic_b")
            .room_scoped(room)
            .multicast()
            .build();

        // Both should have same room prefix
        match (ep1, ep2) {
            (Endpoint::Multicast { topic: t1 }, Endpoint::Multicast { topic: t2 }) => {
                assert!(t1.starts_with("__room::shared-room::"));
                assert!(t2.starts_with("__room::shared-room::"));
                assert_ne!(t1, t2); // Different topics within same room
            }
            _ => panic!("Expected Multicast endpoints"),
        }
    }

    #[test]
    fn test_builder_room_isolation_different_rooms() {
        let ep_dev = EndpointBuilder::new("sensor_data")
            .room_scoped("development")
            .multicast()
            .build();

        let ep_prod = EndpointBuilder::new("sensor_data")
            .room_scoped("production")
            .multicast()
            .build();

        // Same logical topic but different room prefixes
        match (ep_dev, ep_prod) {
            (Endpoint::Multicast { topic: t1 }, Endpoint::Multicast { topic: t2 }) => {
                assert_eq!(t1, "__room::development::sensor_data");
                assert_eq!(t2, "__room::production::sensor_data");
                assert_ne!(t1, t2); // Isolation achieved
            }
            _ => panic!("Expected Multicast endpoints"),
        }
    }
}
