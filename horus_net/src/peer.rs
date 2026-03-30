//! PeerTable — track known peers, their topics, and liveness.
//!
//! Updated from discovery announcements. Peers that miss 3 consecutive
//! announcements (3s at 1Hz discovery) are marked dead and removed.

use std::collections::HashMap;
use std::net::SocketAddr;
use std::time::Instant;

use crate::discovery::{PeerAnnouncement, WireTopicEntry};

/// Unique peer identifier (128-bit).
pub type PeerId = [u8; 16];

/// Information about a known remote peer.
#[derive(Debug, Clone)]
pub struct PeerInfo {
    /// Unique 128-bit peer identifier.
    pub peer_id: PeerId,
    /// Network address of the peer (from announcement source).
    pub addr: SocketAddr,
    /// Data port the peer listens on.
    pub data_port: u16,
    /// Topics the peer publishes or subscribes to.
    pub topics: Vec<WireTopicEntry>,
    /// Last time we received an announcement from this peer.
    pub last_seen: Instant,
    /// Whether this peer is considered alive.
    pub alive: bool,
    /// Secret hash from the peer's announcement.
    pub secret_hash: [u8; 4],
}

impl PeerInfo {
    /// The address to send data to (uses data_port, not the multicast source port).
    pub fn data_addr(&self) -> SocketAddr {
        let mut addr = self.addr;
        addr.set_port(self.data_port);
        addr
    }
}

/// Table of all known remote peers.
pub struct PeerTable {
    peers: HashMap<PeerId, PeerInfo>,
    /// Number of missed announcements before marking dead.
    miss_threshold: u32,
    /// Discovery interval (for calculating staleness).
    discovery_interval_secs: f64,
}

impl PeerTable {
    /// Create with default settings: 3 missed announcements, 1s discovery interval.
    pub fn new() -> Self {
        Self {
            peers: HashMap::new(),
            miss_threshold: 3,
            discovery_interval_secs: 1.0,
        }
    }

    /// Update or insert a peer from a received announcement.
    pub fn update_peer(&mut self, announcement: &PeerAnnouncement) {
        if let Some(existing) = self.peers.get_mut(&announcement.peer_id) {
            existing.addr = announcement.source_addr;
            existing.data_port = announcement.data_port;
            existing.topics = announcement.topics.clone();
            existing.last_seen = Instant::now();
            existing.alive = true;
            existing.secret_hash = announcement.secret_hash;
        } else {
            self.peers.insert(
                announcement.peer_id,
                PeerInfo {
                    peer_id: announcement.peer_id,
                    addr: announcement.source_addr,
                    data_port: announcement.data_port,
                    topics: announcement.topics.clone(),
                    last_seen: Instant::now(),
                    alive: true,
                    secret_hash: announcement.secret_hash,
                },
            );
        }
    }

    /// Check liveness of all peers. Mark dead if stale.
    /// Returns list of peer IDs that just died (for cleanup).
    pub fn check_liveness(&mut self) -> Vec<PeerId> {
        let timeout = std::time::Duration::from_secs_f64(
            self.discovery_interval_secs * self.miss_threshold as f64,
        );
        let now = Instant::now();
        let mut newly_dead = Vec::new();

        for peer in self.peers.values_mut() {
            if peer.alive && now.duration_since(peer.last_seen) > timeout {
                peer.alive = false;
                newly_dead.push(peer.peer_id);
            }
        }

        newly_dead
    }

    /// Remove dead peers from the table. Returns removed count.
    pub fn remove_dead_peers(&mut self) -> usize {
        let before = self.peers.len();
        self.peers.retain(|_, p| p.alive);
        before - self.peers.len()
    }

    /// Check if any alive remote peer subscribes to this topic.
    pub fn has_remote_subscribers(&self, topic_name: &str) -> bool {
        self.peers.values().any(|p| {
            p.alive
                && p.topics
                    .iter()
                    .any(|t| t.name == topic_name && (t.role == 2 || t.role == 3))
        })
    }

    /// Check if any alive remote peer publishes this topic.
    pub fn has_remote_publishers(&self, topic_name: &str) -> bool {
        self.peers.values().any(|p| {
            p.alive
                && p.topics
                    .iter()
                    .any(|t| t.name == topic_name && (t.role == 1 || t.role == 3))
        })
    }

    /// Get all alive peers that subscribe to a topic.
    pub fn subscribers_of(&self, topic_name: &str) -> Vec<&PeerInfo> {
        self.peers
            .values()
            .filter(|p| {
                p.alive
                    && p.topics
                        .iter()
                        .any(|t| t.name == topic_name && (t.role == 2 || t.role == 3))
            })
            .collect()
    }

    /// Get all alive peers that publish a topic.
    pub fn publishers_of(&self, topic_name: &str) -> Vec<&PeerInfo> {
        self.peers
            .values()
            .filter(|p| {
                p.alive
                    && p.topics
                        .iter()
                        .any(|t| t.name == topic_name && (t.role == 1 || t.role == 3))
            })
            .collect()
    }

    /// Find topics published by multiple peers (potential conflict).
    /// Returns (topic_name, list of peer_ids).
    pub fn find_duplicate_publishers(&self) -> Vec<(String, Vec<PeerId>)> {
        let mut pub_topics: HashMap<String, Vec<PeerId>> = HashMap::new();

        for peer in self.peers.values() {
            if !peer.alive {
                continue;
            }
            for topic in &peer.topics {
                if topic.role == 1 || topic.role == 3 {
                    pub_topics
                        .entry(topic.name.clone())
                        .or_default()
                        .push(peer.peer_id);
                }
            }
        }

        pub_topics
            .into_iter()
            .filter(|(_, peers)| peers.len() > 1)
            .collect()
    }

    /// Get a peer by ID.
    pub fn get(&self, peer_id: &PeerId) -> Option<&PeerInfo> {
        self.peers.get(peer_id)
    }

    /// Number of alive peers.
    pub fn alive_count(&self) -> usize {
        self.peers.values().filter(|p| p.alive).count()
    }

    /// Total peers (including dead).
    pub fn total_count(&self) -> usize {
        self.peers.len()
    }

    /// All alive peers.
    pub fn alive_peers(&self) -> Vec<&PeerInfo> {
        self.peers.values().filter(|p| p.alive).collect()
    }
}

impl Default for PeerTable {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::discovery::WireTopicEntry;

    fn make_announcement(peer_id: PeerId, topics: Vec<WireTopicEntry>) -> PeerAnnouncement {
        PeerAnnouncement {
            peer_id,
            data_port: 9100,
            secret_hash: [0; 4],
            has_secret: false,
            topics,
            source_addr: "192.168.1.10:9100".parse().unwrap(),
        }
    }

    fn imu_pub() -> WireTopicEntry {
        WireTopicEntry {
            name: "imu".into(),
            type_hash: 100,
            type_size: 64,
            role: 1, // pub
            is_pod: 1,
            priority: 2,
        }
    }

    fn imu_sub() -> WireTopicEntry {
        WireTopicEntry {
            name: "imu".into(),
            type_hash: 100,
            type_size: 64,
            role: 2, // sub
            is_pod: 1,
            priority: 2,
        }
    }

    fn cmd_sub() -> WireTopicEntry {
        WireTopicEntry {
            name: "cmd_vel".into(),
            type_hash: 200,
            type_size: 16,
            role: 2,
            is_pod: 1,
            priority: 1,
        }
    }

    #[test]
    fn add_and_find_peer() {
        let mut table = PeerTable::new();
        let ann = make_announcement([1; 16], vec![imu_pub()]);
        table.update_peer(&ann);

        assert_eq!(table.alive_count(), 1);
        assert!(table.get(&[1; 16]).is_some());
        assert!(table.get(&[1; 16]).unwrap().alive);
    }

    #[test]
    fn update_refreshes_last_seen() {
        let mut table = PeerTable::new();
        let ann = make_announcement([1; 16], vec![imu_pub()]);
        table.update_peer(&ann);
        let first_seen = table.get(&[1; 16]).unwrap().last_seen;

        std::thread::sleep(std::time::Duration::from_millis(10));
        table.update_peer(&ann);
        let second_seen = table.get(&[1; 16]).unwrap().last_seen;

        assert!(second_seen > first_seen);
    }

    #[test]
    fn has_remote_subscribers() {
        let mut table = PeerTable::new();
        table.update_peer(&make_announcement([1; 16], vec![imu_sub()]));

        assert!(table.has_remote_subscribers("imu"));
        assert!(!table.has_remote_subscribers("odom"));
        assert!(!table.has_remote_publishers("imu")); // sub only
    }

    #[test]
    fn has_remote_publishers() {
        let mut table = PeerTable::new();
        table.update_peer(&make_announcement([1; 16], vec![imu_pub()]));

        assert!(table.has_remote_publishers("imu"));
        assert!(!table.has_remote_publishers("odom"));
    }

    #[test]
    fn subscribers_of() {
        let mut table = PeerTable::new();
        table.update_peer(&make_announcement([1; 16], vec![imu_sub()]));
        table.update_peer(&make_announcement([2; 16], vec![imu_sub(), cmd_sub()]));
        table.update_peer(&make_announcement([3; 16], vec![cmd_sub()]));

        let imu_subs = table.subscribers_of("imu");
        assert_eq!(imu_subs.len(), 2);

        let cmd_subs = table.subscribers_of("cmd_vel");
        assert_eq!(cmd_subs.len(), 2);
    }

    #[test]
    fn liveness_timeout() {
        let mut table = PeerTable {
            peers: HashMap::new(),
            miss_threshold: 3,
            discovery_interval_secs: 0.01, // 10ms for fast testing
        };

        table.update_peer(&make_announcement([1; 16], vec![imu_pub()]));
        assert_eq!(table.alive_count(), 1);

        // Wait for timeout (3 * 10ms = 30ms)
        std::thread::sleep(std::time::Duration::from_millis(50));

        let dead = table.check_liveness();
        assert_eq!(dead.len(), 1);
        assert_eq!(dead[0], [1; 16]);
        assert_eq!(table.alive_count(), 0);
    }

    #[test]
    fn remove_dead_peers() {
        let mut table = PeerTable {
            peers: HashMap::new(),
            miss_threshold: 3,
            discovery_interval_secs: 0.01,
        };

        table.update_peer(&make_announcement([1; 16], vec![imu_pub()]));
        table.update_peer(&make_announcement([2; 16], vec![imu_sub()]));

        std::thread::sleep(std::time::Duration::from_millis(50));
        table.check_liveness();

        let removed = table.remove_dead_peers();
        assert_eq!(removed, 2);
        assert_eq!(table.total_count(), 0);
    }

    #[test]
    fn dead_peer_excluded_from_queries() {
        let mut table = PeerTable {
            peers: HashMap::new(),
            miss_threshold: 3,
            discovery_interval_secs: 0.01,
        };

        table.update_peer(&make_announcement([1; 16], vec![imu_sub()]));
        std::thread::sleep(std::time::Duration::from_millis(50));
        table.check_liveness();

        // Dead peer should not appear in queries
        assert!(!table.has_remote_subscribers("imu"));
        assert!(table.subscribers_of("imu").is_empty());
    }

    #[test]
    fn duplicate_publishers() {
        let mut table = PeerTable::new();
        table.update_peer(&make_announcement([1; 16], vec![imu_pub()]));
        table.update_peer(&make_announcement([2; 16], vec![imu_pub()]));

        let dups = table.find_duplicate_publishers();
        assert_eq!(dups.len(), 1);
        assert_eq!(dups[0].0, "imu");
        assert_eq!(dups[0].1.len(), 2);
    }

    #[test]
    fn no_duplicate_publishers_single_peer() {
        let mut table = PeerTable::new();
        table.update_peer(&make_announcement([1; 16], vec![imu_pub()]));

        let dups = table.find_duplicate_publishers();
        assert!(dups.is_empty());
    }

    #[test]
    fn data_addr_uses_data_port() {
        let mut table = PeerTable::new();
        let mut ann = make_announcement([1; 16], vec![]);
        ann.data_port = 9200;
        ann.source_addr = "192.168.1.10:5555".parse().unwrap();
        table.update_peer(&ann);

        let peer = table.get(&[1; 16]).unwrap();
        let data_addr = peer.data_addr();
        assert_eq!(data_addr.port(), 9200);
        assert_eq!(data_addr.ip().to_string(), "192.168.1.10");
    }
}
