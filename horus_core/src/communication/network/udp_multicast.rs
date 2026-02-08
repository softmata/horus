/// UDP multicast backend with automatic peer discovery
///
/// Uses the DiscoveryService to find peers and broadcasts messages to all of them
use crate::communication::network::discovery::DiscoveryService;
use crate::communication::network::fragmentation::{Fragment, FragmentManager};
use crate::communication::network::protocol::{HorusPacket, MessageType};
use crate::error::HorusResult;
use std::collections::VecDeque;
use std::net::{SocketAddr, UdpSocket};
use std::sync::{Arc, Mutex};

const UDP_BUFFER_SIZE: usize = 65536; // 64KB (max UDP packet)
const RECV_QUEUE_SIZE: usize = 128; // Buffer up to 128 messages
const DISCOVERY_REFRESH_INTERVAL: std::time::Duration = std::time::Duration::from_secs(5);

/// UDP multicast backend for automatic peer discovery
pub struct UdpMulticastBackend<T> {
    topic_name: String,
    socket: Arc<UdpSocket>,
    discovery: Arc<DiscoveryService>,
    peers: Arc<Mutex<Vec<SocketAddr>>>,
    sequence: Arc<Mutex<u32>>,
    recv_queue: Arc<Mutex<VecDeque<T>>>,
    fragment_manager: Arc<FragmentManager>,
    _phantom: std::marker::PhantomData<T>,
}

impl<T> UdpMulticastBackend<T>
where
    T: serde::Serialize + serde::de::DeserializeOwned + Send + Sync + 'static,
{
    /// Create a new UDP multicast backend
    pub fn new(topic: &str) -> HorusResult<Self> {
        // Bind to any available port
        let socket = UdpSocket::bind("0.0.0.0:0").map_err(|e| {
            crate::error::HorusError::Communication(format!("Failed to bind UDP socket: {}", e))
        })?;

        socket.set_nonblocking(true).map_err(|e| {
            crate::error::HorusError::Communication(format!("Failed to set nonblocking: {}", e))
        })?;

        // Get the local port we're bound to
        let local_port = socket
            .local_addr()
            .map_err(|e| {
                crate::error::HorusError::Communication(format!(
                    "Failed to get local address: {}",
                    e
                ))
            })?
            .port();

        // Create discovery service
        let discovery = Arc::new(DiscoveryService::new()?);

        // Announce ourselves
        discovery.announce(topic, local_port)?;

        // Discover initial peers
        let initial_peers = discovery.discover(topic)?;

        let backend = Self {
            topic_name: topic.to_string(),
            socket: Arc::new(socket),
            discovery,
            peers: Arc::new(Mutex::new(initial_peers)),
            sequence: Arc::new(Mutex::new(0)),
            recv_queue: Arc::new(Mutex::new(VecDeque::with_capacity(RECV_QUEUE_SIZE))),
            fragment_manager: Arc::new(FragmentManager::default()),
            _phantom: std::marker::PhantomData,
        };

        // Spawn receiver thread
        backend.spawn_receiver();

        // Spawn peer refresh thread
        backend.spawn_peer_refresh();

        Ok(backend)
    }

    fn spawn_receiver(&self) {
        let socket = Arc::clone(&self.socket);
        let recv_queue = Arc::clone(&self.recv_queue);
        let topic_name = self.topic_name.clone();
        let fragment_manager = Arc::clone(&self.fragment_manager);

        std::thread::spawn(move || {
            let mut buffer = vec![0u8; UDP_BUFFER_SIZE];

            loop {
                match socket.recv_from(&mut buffer) {
                    Ok((size, _src_addr)) => {
                        // Decode packet
                        match HorusPacket::decode(&buffer[..size]) {
                            Ok(packet) => {
                                // Check topic matches
                                if packet.topic != topic_name {
                                    continue;
                                }

                                // Handle different message types
                                match packet.msg_type {
                                    MessageType::Data => {
                                        // Deserialize payload
                                        match bincode::deserialize::<T>(&packet.payload) {
                                            Ok(msg) => {
                                                let mut queue = recv_queue.lock().unwrap();
                                                if queue.len() < RECV_QUEUE_SIZE {
                                                    queue.push_back(msg);
                                                } else {
                                                    // Queue full, drop oldest
                                                    queue.pop_front();
                                                    queue.push_back(msg);
                                                }
                                            }
                                            Err(e) => {
                                                eprintln!("Deserialization error: {}", e);
                                            }
                                        }
                                    }
                                    MessageType::Fragment => {
                                        // Decode fragment
                                        match Fragment::decode(&packet.payload) {
                                            Ok(fragment) => {
                                                // Try to reassemble
                                                if let Some(complete_data) =
                                                    fragment_manager.reassemble(fragment)
                                                {
                                                    // Deserialize complete message
                                                    match bincode::deserialize::<T>(&complete_data)
                                                    {
                                                        Ok(msg) => {
                                                            let mut queue =
                                                                recv_queue.lock().unwrap();
                                                            if queue.len() < RECV_QUEUE_SIZE {
                                                                queue.push_back(msg);
                                                            } else {
                                                                // Queue full, drop oldest
                                                                queue.pop_front();
                                                                queue.push_back(msg);
                                                            }
                                                        }
                                                        Err(e) => {
                                                            eprintln!("Deserialization error after reassembly: {}", e);
                                                        }
                                                    }
                                                }
                                            }
                                            Err(e) => {
                                                eprintln!("Fragment decode error: {:?}", e);
                                            }
                                        }
                                    }
                                    _ => {
                                        // Ignore other message types
                                    }
                                }
                            }
                            Err(e) => {
                                eprintln!("Packet decode error: {}", e);
                            }
                        }
                    }
                    Err(ref e) if e.kind() == std::io::ErrorKind::WouldBlock => {
                        // No data available, sleep briefly
                        std::thread::sleep(std::time::Duration::from_micros(100));
                    }
                    Err(e) => {
                        eprintln!("UDP recv error: {}", e);
                        std::thread::sleep(std::time::Duration::from_millis(10));
                    }
                }
            }
        });
    }

    fn spawn_peer_refresh(&self) {
        let discovery = Arc::clone(&self.discovery);
        let peers = Arc::clone(&self.peers);
        let topic_name = self.topic_name.clone();
        let local_port = self
            .socket
            .local_addr()
            .expect("Failed to get local address")
            .port();

        std::thread::spawn(move || {
            loop {
                std::thread::sleep(DISCOVERY_REFRESH_INTERVAL);

                // Re-announce ourselves
                let _ = discovery.announce(&topic_name, local_port);

                // Refresh peer list
                if let Ok(discovered_peers) = discovery.discover(&topic_name) {
                    let mut peers_lock = peers.lock().unwrap();
                    *peers_lock = discovered_peers;
                }
            }
        });
    }

    /// Send a message to all discovered peers
    pub fn send(&self, msg: &T) -> HorusResult<()> {
        // Serialize payload
        let payload = bincode::serialize(msg).map_err(|e| {
            crate::error::HorusError::Serialization(format!("Serialization error: {}", e))
        })?;

        // Fragment the payload if needed
        let fragments = self.fragment_manager.fragment(&payload);

        // Get peers
        let peers = self.peers.lock().unwrap();
        if peers.is_empty() {
            return Err(crate::error::HorusError::Communication(
                "No peers discovered for multicast".to_string(),
            ));
        }

        // Send each fragment to all peers
        let mut seq = self.sequence.lock().unwrap();
        for fragment in fragments {
            let packet = if fragment.total == 1 {
                // Single fragment - send as Data
                HorusPacket::new_data(self.topic_name.clone(), fragment.data, *seq)
            } else {
                // Multi-fragment - send as Fragment
                let fragment_data = fragment.encode();
                HorusPacket::new_fragment(self.topic_name.clone(), fragment_data, *seq)
            };
            *seq = seq.wrapping_add(1);

            // Encode packet
            let mut buffer = Vec::with_capacity(2048);
            packet.encode(&mut buffer);

            // Send to all peers
            for peer_addr in peers.iter() {
                self.socket.send_to(&buffer, peer_addr).map_err(|e| {
                    crate::error::HorusError::Communication(format!("UDP send error: {}", e))
                })?;
            }
        }
        drop(seq);

        Ok(())
    }

    /// Receive a message from any peer
    pub fn recv(&self) -> Option<T> {
        let mut queue = self.recv_queue.lock().unwrap();
        queue.pop_front()
    }

    /// Get the topic name
    pub fn topic_name(&self) -> &str {
        &self.topic_name
    }

    /// Get current peer count
    pub fn peer_count(&self) -> usize {
        self.peers.lock().unwrap().len()
    }
}

impl<T> std::fmt::Debug for UdpMulticastBackend<T> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("UdpMulticastBackend")
            .field("topic_name", &self.topic_name)
            .finish_non_exhaustive()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use serde::{Deserialize, Serialize};

    // These tests require exclusive network access to discovery port 9871
    // Run with: cargo test -- --ignored --test-threads=1

    #[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
    struct TestMessage {
        data: u64,
    }

    impl crate::core::LogSummary for TestMessage {
        fn log_summary(&self) -> String {
            format!("TestMessage({})", self.data)
        }
    }

    #[test]
    #[ignore = "requires exclusive network access to multicast port 9871"]
    fn test_multicast_backend_creation() {
        let backend = UdpMulticastBackend::<TestMessage>::new("test_multicast");
        assert!(backend.is_ok());
    }

    #[test]
    #[ignore = "requires exclusive network access to multicast port 9871"]
    fn test_multicast_topic_name() {
        let backend = UdpMulticastBackend::<TestMessage>::new("test_topic").unwrap();
        assert_eq!(backend.topic_name(), "test_topic");
    }

    #[test]
    #[ignore = "requires exclusive network access to multicast port 9871"]
    fn test_multicast_no_peers() {
        let backend = UdpMulticastBackend::<TestMessage>::new("isolated_topic").unwrap();

        // Should have no peers since we're alone
        std::thread::sleep(std::time::Duration::from_millis(600));
        assert_eq!(backend.peer_count(), 0);
    }
}
