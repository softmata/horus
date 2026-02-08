/// UDP Multicast Discovery for HORUS network layer
///
/// Implements automatic peer discovery using UDP multicast on 239.255.72.85:9871
/// Target: <1ms discovery for 10 peers
use crate::communication::network::protocol::{HorusPacket, MessageType};
use crate::error::HorusResult;
use log::{error, warn};
use std::collections::HashMap;
use std::net::{IpAddr, Ipv4Addr, SocketAddr, UdpSocket};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

const MULTICAST_ADDR: &str = "239.255.72.85"; // HORUS multicast group
const MULTICAST_PORT: u16 = 9871;
const DISCOVERY_TIMEOUT: Duration = Duration::from_millis(500);
const PEER_TIMEOUT: Duration = Duration::from_secs(30); // Remove peers not seen in 30s

/// Peer information discovered via multicast
#[derive(Debug, Clone)]
pub struct PeerInfo {
    pub addr: SocketAddr,
    pub topics: Vec<String>,
    pub last_seen: Instant,
}

/// Discovery service for finding peers on the network
pub struct DiscoveryService {
    socket: UdpSocket,
    multicast_addr: SocketAddr,
    peers: Arc<Mutex<HashMap<SocketAddr, PeerInfo>>>,
    /// Topics we're publishing (for auto-response to discovery requests)
    published_topics: Arc<Mutex<HashMap<String, u16>>>, // topic -> port
}

impl DiscoveryService {
    /// Create a new discovery service
    pub fn new() -> HorusResult<Self> {
        let socket = UdpSocket::bind(("0.0.0.0", MULTICAST_PORT)).map_err(|e| {
            crate::error::HorusError::Communication(format!(
                "Failed to bind discovery socket: {}",
                e
            ))
        })?;

        socket.set_nonblocking(true).map_err(|e| {
            crate::error::HorusError::Communication(format!("Failed to set nonblocking: {}", e))
        })?;

        socket
            .set_read_timeout(Some(DISCOVERY_TIMEOUT))
            .map_err(|e| {
                crate::error::HorusError::Communication(format!(
                    "Failed to set read timeout: {}",
                    e
                ))
            })?;

        // Join multicast group
        let multicast_ip: Ipv4Addr = MULTICAST_ADDR.parse().map_err(|e| {
            crate::error::HorusError::Config(format!("Invalid multicast address: {}", e))
        })?;
        socket
            .join_multicast_v4(&multicast_ip, &Ipv4Addr::UNSPECIFIED)
            .map_err(|e| {
                crate::error::HorusError::Communication(format!(
                    "Failed to join multicast group: {}",
                    e
                ))
            })?;

        let multicast_addr = SocketAddr::new(IpAddr::V4(multicast_ip), MULTICAST_PORT);

        let service = Self {
            socket,
            multicast_addr,
            peers: Arc::new(Mutex::new(HashMap::new())),
            published_topics: Arc::new(Mutex::new(HashMap::new())),
        };

        // Spawn receiver thread
        service.spawn_receiver();

        Ok(service)
    }

    /// Broadcast discovery request for a topic
    ///
    /// Returns list of peer addresses that publish this topic
    pub fn discover(&self, topic: &str) -> HorusResult<Vec<SocketAddr>> {
        // Send discovery request
        let packet = HorusPacket::new_discovery_request(topic.to_string());

        let mut buffer = Vec::new();
        packet.encode(&mut buffer);
        self.socket
            .send_to(&buffer, self.multicast_addr)
            .map_err(|e| {
                crate::error::HorusError::Communication(format!(
                    "Failed to send discovery request: {}",
                    e
                ))
            })?;

        // Wait for responses (timeout after DISCOVERY_TIMEOUT)
        std::thread::sleep(DISCOVERY_TIMEOUT);

        // Clean up stale peers
        self.cleanup_stale_peers();

        // Return peers that have this topic
        let peers = self.peers.lock().unwrap();
        let matching_peers: Vec<SocketAddr> = peers
            .iter()
            .filter(|(_, info)| info.topics.contains(&topic.to_string()))
            .map(|(addr, _)| *addr)
            .collect();

        Ok(matching_peers)
    }

    /// Announce that we're publishing a topic
    pub fn announce(&self, topic: &str, port: u16) -> HorusResult<()> {
        // Register this as a published topic (for auto-response)
        {
            let mut published = self.published_topics.lock().unwrap();
            published.insert(topic.to_string(), port);
        }

        let packet = HorusPacket::new_discovery_response(topic.to_string(), port);

        let mut buffer = Vec::new();
        packet.encode(&mut buffer);
        self.socket
            .send_to(&buffer, self.multicast_addr)
            .map_err(|e| {
                crate::error::HorusError::Communication(format!(
                    "Failed to send discovery announcement: {}",
                    e
                ))
            })?;

        Ok(())
    }

    /// Remove peers that haven't been seen recently
    fn cleanup_stale_peers(&self) {
        let mut peers = self.peers.lock().unwrap();
        let now = Instant::now();

        peers.retain(|_, info| now.duration_since(info.last_seen) < PEER_TIMEOUT);
    }

    /// Get all known peers
    pub fn get_peers(&self) -> Vec<PeerInfo> {
        self.cleanup_stale_peers();
        let peers = self.peers.lock().unwrap();
        peers.values().cloned().collect()
    }

    fn spawn_receiver(&self) {
        let socket = self
            .socket
            .try_clone()
            .expect("Failed to clone discovery socket");
        let peers = Arc::clone(&self.peers);
        let published_topics = Arc::clone(&self.published_topics);
        let multicast_addr = self.multicast_addr;

        std::thread::spawn(move || {
            let mut buffer = vec![0u8; 65536];

            loop {
                match socket.recv_from(&mut buffer) {
                    Ok((size, src_addr)) => {
                        match HorusPacket::decode(&buffer[..size]) {
                            Ok(packet) => {
                                match packet.msg_type {
                                    MessageType::DiscoveryRequest => {
                                        // Someone is looking for a topic - auto-respond if we publish it
                                        let published = published_topics.lock().unwrap();
                                        if let Some(&port) = published.get(&packet.topic) {
                                            // We publish this topic! Respond immediately
                                            let response = HorusPacket::new_discovery_response(
                                                packet.topic.clone(),
                                                port,
                                            );
                                            let mut response_buffer = Vec::new();
                                            response.encode(&mut response_buffer);
                                            let _ =
                                                socket.send_to(&response_buffer, multicast_addr);
                                        }
                                    }
                                    MessageType::DiscoveryResponse => {
                                        // Someone announced they publish a topic
                                        let port = if packet.payload.len() >= 2 {
                                            u16::from_le_bytes([
                                                packet.payload[0],
                                                packet.payload[1],
                                            ])
                                        } else {
                                            9870 // Default port
                                        };

                                        // Update peer info
                                        let peer_addr = SocketAddr::new(src_addr.ip(), port);
                                        let mut peers_lock = peers.lock().unwrap();

                                        peers_lock
                                            .entry(peer_addr)
                                            .and_modify(|info| {
                                                if !info.topics.contains(&packet.topic) {
                                                    info.topics.push(packet.topic.clone());
                                                }
                                                info.last_seen = Instant::now();
                                            })
                                            .or_insert_with(|| PeerInfo {
                                                addr: peer_addr,
                                                topics: vec![packet.topic.clone()],
                                                last_seen: Instant::now(),
                                            });
                                    }
                                    MessageType::Heartbeat => {
                                        // Update last_seen for peer
                                        let peer_addr = SocketAddr::new(src_addr.ip(), 9870);
                                        let mut peers_lock = peers.lock().unwrap();

                                        if let Some(info) = peers_lock.get_mut(&peer_addr) {
                                            info.last_seen = Instant::now();
                                        }
                                    }
                                    _ => {
                                        // Ignore other message types
                                    }
                                }
                            }
                            Err(e) => {
                                warn!("[Discovery] Packet decode error: {}", e);
                            }
                        }
                    }
                    Err(ref e) if e.kind() == std::io::ErrorKind::WouldBlock => {
                        // No data available, sleep briefly
                        std::thread::sleep(Duration::from_micros(100));
                    }
                    Err(e) => {
                        error!("[Discovery] Recv error: {}", e);
                        std::thread::sleep(Duration::from_millis(10));
                    }
                }
            }
        });
    }
}

impl Drop for DiscoveryService {
    fn drop(&mut self) {
        // Leave multicast group
        let multicast_ip: Ipv4Addr = MULTICAST_ADDR.parse().unwrap();
        let _ = self
            .socket
            .leave_multicast_v4(&multicast_ip, &Ipv4Addr::UNSPECIFIED);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    // These tests require exclusive network access to port 9871
    // Run with: cargo test -- --ignored --test-threads=1

    #[test]
    #[ignore = "requires exclusive network access to multicast port 9871"]
    fn test_discovery_service_creation() {
        let service = DiscoveryService::new().unwrap();
        assert_eq!(service.multicast_addr.port(), MULTICAST_PORT);
    }

    #[test]
    #[ignore = "requires exclusive network access to multicast port 9871"]
    fn test_discovery_announce() {
        let service = DiscoveryService::new().unwrap();
        let result = service.announce("test_topic", 9870);
        assert!(result.is_ok());
    }

    #[test]
    #[ignore = "requires exclusive network access to multicast port 9871"]
    fn test_discovery_request() {
        let service = DiscoveryService::new().unwrap();

        // This won't find any peers (no other services running)
        let peers = service.discover("test_topic").unwrap();
        assert_eq!(peers.len(), 0);
    }

    #[test]
    #[ignore = "requires exclusive network access to multicast port 9871"]
    fn test_peer_cleanup() {
        let service = DiscoveryService::new().unwrap();

        // Add a stale peer manually (for testing)
        {
            let mut peers = service.peers.lock().unwrap();
            let old_time = Instant::now() - Duration::from_secs(60);
            peers.insert(
                "127.0.0.1:9870".parse().unwrap(),
                PeerInfo {
                    addr: "127.0.0.1:9870".parse().unwrap(),
                    topics: vec!["old_topic".to_string()],
                    last_seen: old_time,
                },
            );
        }

        // Cleanup should remove it
        service.cleanup_stale_peers();

        let peers = service.get_peers();
        assert_eq!(peers.len(), 0);
    }
}
