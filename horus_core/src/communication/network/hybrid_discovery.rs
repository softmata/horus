/// Hybrid Discovery Service for HORUS
///
/// Automatically detects and uses the best available discovery mechanism:
/// - **LAN**: Standard UDP multicast (239.255.72.85:9871)
/// - **Husarnet**: Unicast to all peers (simulated multicast, since Husarnet doesn't support real multicast)
///
/// This provides transparent topic@* support across both LAN and Husarnet VPN networks.
///
/// # Example
///
/// ```rust,ignore
/// use horus_core::communication::network::HybridDiscovery;
///
/// // Automatically detects Husarnet and uses appropriate discovery
/// let discovery = HybridDiscovery::new()?;
///
/// // Discover publishers for a topic
/// let peers = discovery.discover("camera/rgb").await?;
///
/// // Announce that we publish a topic
/// discovery.announce("motor/cmd", 9870).await?;
/// ```
use crate::communication::network::protocol::{HorusPacket, MessageType};
use crate::error::HorusResult;
#[cfg(feature = "husarnet")]
use log::debug;
use log::{info, warn};
use std::collections::HashMap;
use std::net::{IpAddr, Ipv4Addr, SocketAddr, UdpSocket};
use std::sync::{Arc, Mutex, RwLock};
use std::time::{Duration, Instant};

#[cfg(feature = "husarnet")]
use crate::communication::network::husarnet::{
    HusarnetConfig, HusarnetDiscovery, HusarnetPeer, DEFAULT_HUSARNET_PORT,
};

/// Default multicast group for LAN discovery
const MULTICAST_ADDR: &str = "239.255.72.85";
const MULTICAST_PORT: u16 = 9871;
const DISCOVERY_TIMEOUT: Duration = Duration::from_millis(500);
const PEER_TIMEOUT: Duration = Duration::from_secs(30);

/// Default HORUS port for Husarnet (when feature not enabled)
#[cfg(not(feature = "husarnet"))]
const DEFAULT_HUSARNET_PORT: u16 = 9847;

/// Discovered peer information
#[derive(Debug, Clone)]
pub struct HybridPeerInfo {
    /// Socket address of the peer
    pub addr: SocketAddr,
    /// Topics this peer publishes
    pub topics: Vec<String>,
    /// Last time we heard from this peer
    pub last_seen: Instant,
    /// Source of discovery
    pub source: DiscoverySource,
}

/// How a peer was discovered
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DiscoverySource {
    /// Discovered via LAN multicast
    Multicast,
    /// Discovered via Husarnet peer list
    Husarnet,
    /// Discovered via mDNS
    Mdns,
    /// Manually configured
    Manual,
}

/// Hybrid discovery service configuration
#[derive(Debug, Clone)]
pub struct HybridDiscoveryConfig {
    /// Enable LAN multicast discovery
    pub enable_multicast: bool,
    /// Enable Husarnet discovery (if available)
    pub enable_husarnet: bool,
    /// Enable mDNS discovery (if available)
    pub enable_mdns: bool,
    /// Default port for Husarnet peers
    pub husarnet_port: u16,
    /// Discovery timeout
    pub timeout: Duration,
    /// Peer timeout (remove if not seen)
    pub peer_timeout: Duration,
}

impl Default for HybridDiscoveryConfig {
    fn default() -> Self {
        Self {
            enable_multicast: true,
            enable_husarnet: true,
            enable_mdns: true,
            husarnet_port: DEFAULT_HUSARNET_PORT,
            timeout: DISCOVERY_TIMEOUT,
            peer_timeout: PEER_TIMEOUT,
        }
    }
}

/// Hybrid discovery service that combines multiple discovery mechanisms
pub struct HybridDiscovery {
    config: HybridDiscoveryConfig,
    /// LAN multicast socket (IPv4)
    multicast_socket: Option<UdpSocket>,
    /// Husarnet discovery service
    #[cfg(feature = "husarnet")]
    husarnet: Option<HusarnetDiscovery>,
    /// All discovered peers
    peers: Arc<RwLock<HashMap<SocketAddr, HybridPeerInfo>>>,
    /// Topics we publish (for responding to discovery)
    published_topics: Arc<Mutex<HashMap<String, u16>>>,
    /// Is running
    running: Arc<std::sync::atomic::AtomicBool>,
}

impl HybridDiscovery {
    /// Create a new hybrid discovery service with default config
    pub fn new() -> HorusResult<Self> {
        Self::with_config(HybridDiscoveryConfig::default())
    }

    /// Create with custom configuration
    pub fn with_config(config: HybridDiscoveryConfig) -> HorusResult<Self> {
        let mut multicast_socket = None;
        #[cfg(feature = "husarnet")]
        let mut husarnet = None;

        // Try to set up LAN multicast
        if config.enable_multicast {
            match Self::setup_multicast() {
                Ok(socket) => {
                    info!(
                        "[HybridDiscovery] LAN multicast enabled on {}",
                        MULTICAST_ADDR
                    );
                    multicast_socket = Some(socket);
                }
                Err(e) => {
                    warn!("[HybridDiscovery] LAN multicast unavailable: {}", e);
                }
            }
        }

        // Try to set up Husarnet
        #[cfg(feature = "husarnet")]
        if config.enable_husarnet {
            match HusarnetDiscovery::new(HusarnetConfig {
                horus_port: config.husarnet_port,
                ..Default::default()
            }) {
                Ok(h) => {
                    info!("[HybridDiscovery] Husarnet discovery enabled");
                    husarnet = Some(h);
                }
                Err(e) => {
                    debug!("[HybridDiscovery] Husarnet unavailable: {}", e);
                }
            }
        }

        let service = Self {
            config,
            multicast_socket,
            #[cfg(feature = "husarnet")]
            husarnet,
            peers: Arc::new(RwLock::new(HashMap::new())),
            published_topics: Arc::new(Mutex::new(HashMap::new())),
            running: Arc::new(std::sync::atomic::AtomicBool::new(true)),
        };

        // Spawn receiver thread for multicast
        if service.multicast_socket.is_some() {
            service.spawn_multicast_receiver();
        }

        Ok(service)
    }

    fn setup_multicast() -> Result<UdpSocket, String> {
        let socket = UdpSocket::bind(("0.0.0.0", MULTICAST_PORT))
            .map_err(|e| format!("Failed to bind multicast socket: {}", e))?;

        socket
            .set_nonblocking(true)
            .map_err(|e| format!("Failed to set nonblocking: {}", e))?;

        socket
            .set_read_timeout(Some(DISCOVERY_TIMEOUT))
            .map_err(|e| format!("Failed to set read timeout: {}", e))?;

        // Join multicast group
        let multicast_ip: Ipv4Addr = MULTICAST_ADDR
            .parse()
            .map_err(|e| format!("Invalid multicast address: {}", e))?;
        socket
            .join_multicast_v4(&multicast_ip, &Ipv4Addr::UNSPECIFIED)
            .map_err(|e| format!("Failed to join multicast group: {}", e))?;

        Ok(socket)
    }

    /// Discover peers that publish a topic
    ///
    /// Queries both LAN multicast and Husarnet peers (if available)
    pub fn discover(&self, topic: &str) -> HorusResult<Vec<SocketAddr>> {
        // Send discovery request via multicast
        if let Some(ref socket) = self.multicast_socket {
            let packet = HorusPacket::new_discovery_request(topic.to_string());
            let mut buffer = Vec::new();
            packet.encode(&mut buffer);

            let multicast_addr =
                SocketAddr::new(IpAddr::V4(MULTICAST_ADDR.parse().unwrap()), MULTICAST_PORT);
            let _ = socket.send_to(&buffer, multicast_addr);
        }

        // Send discovery request to all Husarnet peers (simulated multicast)
        #[cfg(feature = "husarnet")]
        if let Some(ref husarnet) = self.husarnet {
            let packet = HorusPacket::new_discovery_request(topic.to_string());
            let mut buffer = Vec::new();
            packet.encode(&mut buffer);

            // Get all peer addresses and send to each
            for addr in husarnet.broadcast_addresses(self.config.husarnet_port) {
                // Create a UDP socket for Husarnet (IPv6)
                if let Ok(socket) = UdpSocket::bind("[::]:0") {
                    let _ = socket.send_to(&buffer, addr);
                }
            }
        }

        // Wait for responses
        std::thread::sleep(self.config.timeout);

        // Cleanup stale peers
        self.cleanup_stale_peers();

        // Return peers that have this topic
        let peers = self.peers.read().unwrap();
        let matching_peers: Vec<SocketAddr> = peers
            .iter()
            .filter(|(_, info)| info.topics.contains(&topic.to_string()))
            .map(|(addr, _)| *addr)
            .collect();

        Ok(matching_peers)
    }

    /// Announce that we publish a topic
    ///
    /// Announces via both LAN multicast and Husarnet (if available)
    pub fn announce(&self, topic: &str, port: u16) -> HorusResult<()> {
        // Register topic
        {
            let mut published = self.published_topics.lock().unwrap();
            published.insert(topic.to_string(), port);
        }

        let packet = HorusPacket::new_discovery_response(topic.to_string(), port);
        let mut buffer = Vec::new();
        packet.encode(&mut buffer);

        // Announce via multicast
        if let Some(ref socket) = self.multicast_socket {
            let multicast_addr =
                SocketAddr::new(IpAddr::V4(MULTICAST_ADDR.parse().unwrap()), MULTICAST_PORT);
            let _ = socket.send_to(&buffer, multicast_addr);
        }

        // Announce to all Husarnet peers
        #[cfg(feature = "husarnet")]
        if let Some(ref husarnet) = self.husarnet {
            for addr in husarnet.broadcast_addresses(self.config.husarnet_port) {
                if let Ok(socket) = UdpSocket::bind("[::]:0") {
                    let _ = socket.send_to(&buffer, addr);
                }
            }
        }

        Ok(())
    }

    /// Get all known Husarnet peers
    #[cfg(feature = "husarnet")]
    pub fn get_husarnet_peers(&self) -> Vec<HusarnetPeer> {
        self.husarnet
            .as_ref()
            .map(|h| h.peers())
            .unwrap_or_default()
    }

    /// Check if Husarnet is available
    #[cfg(feature = "husarnet")]
    pub fn has_husarnet(&self) -> bool {
        self.husarnet.is_some()
    }

    /// Check if Husarnet is available (no-op when feature not enabled)
    #[cfg(not(feature = "husarnet"))]
    pub fn has_husarnet(&self) -> bool {
        false
    }

    /// Check if LAN multicast is available
    pub fn has_multicast(&self) -> bool {
        self.multicast_socket.is_some()
    }

    /// Get all discovered peers
    pub fn get_peers(&self) -> Vec<HybridPeerInfo> {
        self.cleanup_stale_peers();
        let peers = self.peers.read().unwrap();
        peers.values().cloned().collect()
    }

    /// Get peers by discovery source
    pub fn get_peers_by_source(&self, source: DiscoverySource) -> Vec<HybridPeerInfo> {
        self.cleanup_stale_peers();
        let peers = self.peers.read().unwrap();
        peers
            .values()
            .filter(|p| p.source == source)
            .cloned()
            .collect()
    }

    fn cleanup_stale_peers(&self) {
        let mut peers = self.peers.write().unwrap();
        let now = Instant::now();
        peers.retain(|_, info| now.duration_since(info.last_seen) < self.config.peer_timeout);
    }

    fn spawn_multicast_receiver(&self) {
        let socket = self
            .multicast_socket
            .as_ref()
            .unwrap()
            .try_clone()
            .expect("Failed to clone multicast socket");
        let peers = Arc::clone(&self.peers);
        let published_topics = Arc::clone(&self.published_topics);
        let running = Arc::clone(&self.running);

        std::thread::spawn(move || {
            let multicast_addr =
                SocketAddr::new(IpAddr::V4(MULTICAST_ADDR.parse().unwrap()), MULTICAST_PORT);
            let mut buffer = vec![0u8; 65536];

            while running.load(std::sync::atomic::Ordering::Relaxed) {
                match socket.recv_from(&mut buffer) {
                    Ok((size, src_addr)) => {
                        if let Ok(packet) = HorusPacket::decode(&buffer[..size]) {
                            match packet.msg_type {
                                MessageType::DiscoveryRequest => {
                                    // Respond if we publish this topic
                                    let published = published_topics.lock().unwrap();
                                    if let Some(&port) = published.get(&packet.topic) {
                                        let response = HorusPacket::new_discovery_response(
                                            packet.topic.clone(),
                                            port,
                                        );
                                        let mut response_buffer = Vec::new();
                                        response.encode(&mut response_buffer);
                                        let _ = socket.send_to(&response_buffer, multicast_addr);
                                    }
                                }
                                MessageType::DiscoveryResponse => {
                                    // Parse port from payload
                                    let port = if packet.payload.len() >= 2 {
                                        u16::from_le_bytes([packet.payload[0], packet.payload[1]])
                                    } else {
                                        9870
                                    };

                                    let peer_addr = SocketAddr::new(src_addr.ip(), port);
                                    let mut peers_lock = peers.write().unwrap();

                                    peers_lock
                                        .entry(peer_addr)
                                        .and_modify(|info| {
                                            if !info.topics.contains(&packet.topic) {
                                                info.topics.push(packet.topic.clone());
                                            }
                                            info.last_seen = Instant::now();
                                        })
                                        .or_insert_with(|| HybridPeerInfo {
                                            addr: peer_addr,
                                            topics: vec![packet.topic.clone()],
                                            last_seen: Instant::now(),
                                            source: DiscoverySource::Multicast,
                                        });
                                }
                                MessageType::Heartbeat => {
                                    let peer_addr = SocketAddr::new(src_addr.ip(), 9870);
                                    let mut peers_lock = peers.write().unwrap();
                                    if let Some(info) = peers_lock.get_mut(&peer_addr) {
                                        info.last_seen = Instant::now();
                                    }
                                }
                                _ => {}
                            }
                        }
                    }
                    Err(ref e) if e.kind() == std::io::ErrorKind::WouldBlock => {
                        std::thread::sleep(Duration::from_micros(100));
                    }
                    Err(_) => {
                        std::thread::sleep(Duration::from_millis(10));
                    }
                }
            }
        });
    }

    /// Refresh Husarnet peer list
    #[cfg(feature = "husarnet")]
    pub fn refresh_husarnet(&self) -> HorusResult<()> {
        if let Some(ref husarnet) = self.husarnet {
            husarnet.refresh_status().map_err(|e| {
                crate::error::HorusError::communication(format!(
                    "Failed to refresh Husarnet: {}",
                    e
                ))
            })
        } else {
            Ok(())
        }
    }

    /// Add a discovered peer manually (from Husarnet peer list)
    #[cfg(feature = "husarnet")]
    pub fn add_husarnet_peer(&self, peer: &HusarnetPeer, topic: Option<String>) {
        let addr_v6 = peer.socket_addr(self.config.husarnet_port);
        let addr = SocketAddr::V6(addr_v6);
        let mut peers = self.peers.write().unwrap();

        peers
            .entry(addr)
            .and_modify(|info| {
                if let Some(ref t) = topic {
                    if !info.topics.contains(t) {
                        info.topics.push(t.clone());
                    }
                }
                info.last_seen = Instant::now();
            })
            .or_insert_with(|| HybridPeerInfo {
                addr,
                topics: topic.map(|t| vec![t]).unwrap_or_default(),
                last_seen: Instant::now(),
                source: DiscoverySource::Husarnet,
            });
    }

    /// Get discovery summary for debugging
    pub fn summary(&self) -> DiscoverySummary {
        let peers = self.peers.read().unwrap();
        let multicast_count = peers
            .values()
            .filter(|p| p.source == DiscoverySource::Multicast)
            .count();
        let husarnet_count = peers
            .values()
            .filter(|p| p.source == DiscoverySource::Husarnet)
            .count();
        let mdns_count = peers
            .values()
            .filter(|p| p.source == DiscoverySource::Mdns)
            .count();

        DiscoverySummary {
            total_peers: peers.len(),
            multicast_peers: multicast_count,
            husarnet_peers: husarnet_count,
            mdns_peers: mdns_count,
            has_multicast: self.has_multicast(),
            has_husarnet: self.has_husarnet(),
        }
    }
}

/// Summary of discovery state
#[derive(Debug, Clone)]
pub struct DiscoverySummary {
    pub total_peers: usize,
    pub multicast_peers: usize,
    pub husarnet_peers: usize,
    pub mdns_peers: usize,
    pub has_multicast: bool,
    pub has_husarnet: bool,
}

impl Drop for HybridDiscovery {
    fn drop(&mut self) {
        self.running
            .store(false, std::sync::atomic::Ordering::Relaxed);

        // Leave multicast group
        if let Some(ref socket) = self.multicast_socket {
            let multicast_ip: Ipv4Addr = MULTICAST_ADDR.parse().unwrap();
            let _ = socket.leave_multicast_v4(&multicast_ip, &Ipv4Addr::UNSPECIFIED);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_config_default() {
        let config = HybridDiscoveryConfig::default();
        assert!(config.enable_multicast);
        assert!(config.enable_husarnet);
        assert!(config.enable_mdns);
    }

    #[test]
    fn test_discovery_source() {
        assert_eq!(DiscoverySource::Multicast, DiscoverySource::Multicast);
        assert_ne!(DiscoverySource::Multicast, DiscoverySource::Husarnet);
    }

    #[test]
    fn test_peer_info() {
        let info = HybridPeerInfo {
            addr: "127.0.0.1:9870".parse().unwrap(),
            topics: vec!["test".to_string()],
            last_seen: Instant::now(),
            source: DiscoverySource::Multicast,
        };
        assert_eq!(info.topics.len(), 1);
        assert_eq!(info.source, DiscoverySource::Multicast);
    }
}
