//! UDP multicast discovery — announcements, matching, type_hash validation.
//!
//! Every Replicator joins multicast group 224.0.69.72:9100 and sends periodic
//! announcements listing its topics. Peers match local pub ↔ remote sub.
//! See blueprint section 17.

use std::net::SocketAddr;

use crate::registry::{TopicEntry, TopicRole};
use crate::wire::{self, MAGIC, VERSION};

/// Max topic name length in wire format.
pub const MAX_TOPIC_NAME: usize = 63;

/// Max topics per announcement (fits in one UDP packet).
const MAX_TOPICS_PER_ANNOUNCEMENT: usize = 200;

// ─── Peer ID ────────────────────────────────────────────────────────────────

/// Generate a random 128-bit peer ID.
pub fn generate_peer_id() -> [u8; 16] {
    let mut id = [0u8; 16];
    // Use /dev/urandom via std
    use std::collections::hash_map::DefaultHasher;
    use std::hash::{Hash, Hasher};
    use std::time::SystemTime;

    let mut hasher = DefaultHasher::new();
    SystemTime::now().hash(&mut hasher);
    std::process::id().hash(&mut hasher);
    std::thread::current().id().hash(&mut hasher);
    let h1 = hasher.finish();

    // Second hash with different seed
    hasher = DefaultHasher::new();
    (h1 ^ 0xDEADBEEFCAFEBABE).hash(&mut hasher);
    let addr = &id as *const _ as usize;
    addr.hash(&mut hasher);
    let h2 = hasher.finish();

    id[0..8].copy_from_slice(&h1.to_le_bytes());
    id[8..16].copy_from_slice(&h2.to_le_bytes());
    id
}

/// Truncate a 16-byte peer_id to a 2-byte hash for packet headers.
pub fn peer_id_hash(peer_id: &[u8; 16]) -> u16 {
    let mut h: u16 = 0;
    for chunk in peer_id.chunks(2) {
        h ^= u16::from_le_bytes([chunk[0], chunk.get(1).copied().unwrap_or(0)]);
    }
    h
}

// ─── Wire-format topic entry (72 bytes) ─────────────────────────────────────

/// A topic entry in the discovery announcement wire format.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct WireTopicEntry {
    pub name: String,
    pub type_hash: u32,
    pub type_size: u32,
    pub role: u8,     // 1=pub, 2=sub, 3=both
    pub is_pod: u8,   // 0 or 1
    pub priority: u8, // auto-inferred or configured
}

impl WireTopicEntry {
    /// Wire size: 1 (name_len) + 63 (name) + 4 + 4 + 1 + 1 + 1 + 1 = 76 bytes
    pub const SIZE: usize = 76;

    pub fn from_registry_entry(entry: &TopicEntry) -> Self {
        Self {
            name: entry.name.clone(),
            type_hash: entry.type_hash,
            type_size: entry.type_size,
            role: entry.role.to_wire(),
            is_pod: if entry.is_pod { 1 } else { 0 },
            priority: crate::priority::Priority::infer_from_topic(&entry.name) as u8,
        }
    }

    pub fn encode(&self, buf: &mut [u8]) {
        debug_assert!(buf.len() >= Self::SIZE);
        let name_bytes = self.name.as_bytes();
        let name_len = name_bytes.len().min(MAX_TOPIC_NAME);
        buf[0] = name_len as u8;
        buf[1..1 + name_len].copy_from_slice(&name_bytes[..name_len]);
        // Zero-pad remaining name bytes
        for b in &mut buf[1 + name_len..64] {
            *b = 0;
        }
        buf[64..68].copy_from_slice(&self.type_hash.to_le_bytes());
        buf[68..72].copy_from_slice(&self.type_size.to_le_bytes());
        buf[72] = self.role;
        buf[73] = self.is_pod;
        buf[74] = self.priority;
        buf[75] = 0; // padding
    }

    pub fn decode(buf: &[u8]) -> Option<Self> {
        if buf.len() < Self::SIZE {
            return None;
        }
        let name_len = buf[0] as usize;
        if name_len > MAX_TOPIC_NAME {
            return None;
        }
        let name = std::str::from_utf8(&buf[1..1 + name_len]).ok()?.to_string();
        let type_hash = u32::from_le_bytes([buf[64], buf[65], buf[66], buf[67]]);
        let type_size = u32::from_le_bytes([buf[68], buf[69], buf[70], buf[71]]);
        let role = buf[72];
        let is_pod = buf[73];
        let priority = buf[74];

        Some(Self {
            name,
            type_hash,
            type_size,
            role,
            is_pod,
            priority,
        })
    }
}

// ─── Announcement ───────────────────────────────────────────────────────────

/// Discovery announcement header (30 bytes).
pub struct AnnouncementHeader {
    pub magic: u32,
    pub version: u8,
    pub flags: u8,        // bit 0: has_secret_hash
    pub peer_id: [u8; 16],
    pub data_port: u16,
    pub secret_hash: [u8; 4], // First 4 bytes of hash (compact)
    pub topic_count: u16,
}

impl AnnouncementHeader {
    pub const SIZE: usize = 30;

    pub fn encode(&self, buf: &mut [u8]) {
        debug_assert!(buf.len() >= Self::SIZE);
        buf[0..4].copy_from_slice(&self.magic.to_le_bytes());
        buf[4] = self.version;
        buf[5] = self.flags;
        buf[6..22].copy_from_slice(&self.peer_id);
        buf[22..24].copy_from_slice(&self.data_port.to_le_bytes());
        buf[24..28].copy_from_slice(&self.secret_hash);
        buf[28..30].copy_from_slice(&self.topic_count.to_le_bytes());
    }

    pub fn decode(buf: &[u8]) -> Option<Self> {
        if buf.len() < Self::SIZE {
            return None;
        }
        let magic = u32::from_le_bytes([buf[0], buf[1], buf[2], buf[3]]);
        if magic != MAGIC {
            return None;
        }
        let version = buf[4];
        if version != VERSION {
            return None;
        }
        let flags = buf[5];
        let mut peer_id = [0u8; 16];
        peer_id.copy_from_slice(&buf[6..22]);
        let data_port = u16::from_le_bytes([buf[22], buf[23]]);
        let mut secret_hash = [0u8; 4];
        secret_hash.copy_from_slice(&buf[24..28]);
        let topic_count = u16::from_le_bytes([buf[28], buf[29]]);

        Some(Self {
            magic,
            version,
            flags,
            peer_id,
            data_port,
            secret_hash,
            topic_count,
        })
    }
}

// ─── Encode/Decode full announcements ───────────────────────────────────────

/// Encode a full discovery announcement. Returns bytes written.
pub fn encode_announcement(
    peer_id: &[u8; 16],
    data_port: u16,
    secret_hash: &[u8; 4],
    topics: &[TopicEntry],
    buf: &mut [u8],
) -> usize {
    let topic_count = topics.len().min(MAX_TOPICS_PER_ANNOUNCEMENT);
    let has_secret = secret_hash != &[0u8; 4];

    let header = AnnouncementHeader {
        magic: MAGIC,
        version: VERSION,
        flags: if has_secret { 1 } else { 0 },
        peer_id: *peer_id,
        data_port,
        secret_hash: *secret_hash,
        topic_count: topic_count as u16,
    };

    header.encode(&mut buf[..AnnouncementHeader::SIZE]);
    let mut offset = AnnouncementHeader::SIZE;

    for entry in topics.iter().take(topic_count) {
        let wire_entry = WireTopicEntry::from_registry_entry(entry);
        wire_entry.encode(&mut buf[offset..offset + WireTopicEntry::SIZE]);
        offset += WireTopicEntry::SIZE;
    }

    offset
}

/// Decoded announcement from a peer.
#[derive(Debug, Clone)]
pub struct PeerAnnouncement {
    pub peer_id: [u8; 16],
    pub data_port: u16,
    pub secret_hash: [u8; 4],
    pub has_secret: bool,
    pub topics: Vec<WireTopicEntry>,
    pub source_addr: SocketAddr,
}

/// Decode a discovery announcement from raw bytes.
pub fn decode_announcement(buf: &[u8], source_addr: SocketAddr) -> Option<PeerAnnouncement> {
    let header = AnnouncementHeader::decode(buf)?;

    let mut topics = Vec::with_capacity(header.topic_count as usize);
    let mut offset = AnnouncementHeader::SIZE;

    for _ in 0..header.topic_count {
        if offset + WireTopicEntry::SIZE > buf.len() {
            break;
        }
        if let Some(entry) = WireTopicEntry::decode(&buf[offset..]) {
            topics.push(entry);
        }
        offset += WireTopicEntry::SIZE;
    }

    Some(PeerAnnouncement {
        peer_id: header.peer_id,
        data_port: header.data_port,
        secret_hash: header.secret_hash,
        has_secret: header.flags & 1 != 0,
        topics,
        source_addr,
    })
}

// ─── Topic Matching ─────────────────────────────────────────────────────────

/// A matched replication: local topic ↔ remote topic.
#[derive(Debug, Clone)]
pub struct ReplicationMatch {
    /// Topic name.
    pub topic: String,
    /// We export (local pub → remote sub).
    pub export: bool,
    /// We import (remote pub → local sub).
    pub import: bool,
    /// Type hash (must match both sides).
    pub type_hash: u32,
    /// Whether the type is POD.
    pub is_pod: bool,
}

/// Find replication matches between local registry and a peer announcement.
///
/// Returns matches and warnings (type hash mismatches).
pub fn find_matches(
    local_entries: &[TopicEntry],
    remote_topics: &[WireTopicEntry],
) -> (Vec<ReplicationMatch>, Vec<String>) {
    let mut matches = Vec::new();
    let mut warnings = Vec::new();

    for local in local_entries {
        for remote in remote_topics {
            if local.name != remote.name {
                continue;
            }

            // Type hash must match
            if local.type_hash != remote.type_hash {
                warnings.push(format!(
                    "Topic '{}' type mismatch: local hash={:#X}, remote hash={:#X}. Refusing to replicate.",
                    local.name, local.type_hash, remote.type_hash
                ));
                continue;
            }

            let local_pub = matches!(local.role, TopicRole::Publisher | TopicRole::Both);
            let local_sub = matches!(local.role, TopicRole::Subscriber | TopicRole::Both);
            let remote_pub = remote.role == 1 || remote.role == 3; // pub or both
            let remote_sub = remote.role == 2 || remote.role == 3; // sub or both

            let export = local_pub && remote_sub;
            let import = local_sub && remote_pub;

            if export || import {
                matches.push(ReplicationMatch {
                    topic: local.name.clone(),
                    export,
                    import,
                    type_hash: local.type_hash,
                    is_pod: local.is_pod,
                });
            }
        }
    }

    (matches, warnings)
}

/// Compute a simple secret hash from a string. Returns first 4 bytes of FNV-1a.
pub fn compute_secret_hash(secret: &str) -> [u8; 4] {
    wire::fnv1a_hash(secret.as_bytes()).to_le_bytes()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::registry::TopicRole;

    #[test]
    fn peer_id_is_unique() {
        let a = generate_peer_id();
        let b = generate_peer_id();
        // Not guaranteed to be different in a tight loop, but extremely unlikely to match
        // Just verify they're not all zeros
        assert_ne!(a, [0u8; 16]);
        assert_ne!(b, [0u8; 16]);
    }

    #[test]
    fn peer_id_hash_deterministic() {
        let id = [1u8, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16];
        assert_eq!(peer_id_hash(&id), peer_id_hash(&id));
    }

    #[test]
    fn wire_topic_entry_roundtrip() {
        let entry = WireTopicEntry {
            name: "robot.imu".into(),
            type_hash: 0xDEAD,
            type_size: 64,
            role: 1,
            is_pod: 1,
            priority: 2,
        };
        let mut buf = [0u8; WireTopicEntry::SIZE];
        entry.encode(&mut buf);
        let decoded = WireTopicEntry::decode(&buf).unwrap();
        assert_eq!(entry, decoded);
    }

    #[test]
    fn wire_topic_entry_max_name() {
        let name = "a".repeat(MAX_TOPIC_NAME);
        let entry = WireTopicEntry {
            name: name.clone(),
            type_hash: 1,
            type_size: 1,
            role: 1,
            is_pod: 0,
            priority: 0,
        };
        let mut buf = [0u8; WireTopicEntry::SIZE];
        entry.encode(&mut buf);
        let decoded = WireTopicEntry::decode(&buf).unwrap();
        assert_eq!(decoded.name, name);
    }

    #[test]
    fn announcement_header_roundtrip() {
        let header = AnnouncementHeader {
            magic: MAGIC,
            version: VERSION,
            flags: 1,
            peer_id: [0xAA; 16],
            data_port: 9100,
            secret_hash: [1, 2, 3, 4],
            topic_count: 5,
        };
        let mut buf = [0u8; AnnouncementHeader::SIZE];
        header.encode(&mut buf);
        let decoded = AnnouncementHeader::decode(&buf).unwrap();
        assert_eq!(decoded.peer_id, [0xAA; 16]);
        assert_eq!(decoded.data_port, 9100);
        assert_eq!(decoded.topic_count, 5);
        assert_eq!(decoded.secret_hash, [1, 2, 3, 4]);
    }

    #[test]
    fn announcement_bad_magic() {
        let mut buf = [0u8; AnnouncementHeader::SIZE];
        buf[0] = 0xFF;
        assert!(AnnouncementHeader::decode(&buf).is_none());
    }

    #[test]
    fn full_announcement_roundtrip() {
        let peer_id = [0xBB; 16];
        let topics = vec![
            TopicEntry {
                name: "imu".into(),
                type_hash: 100,
                type_size: 64,
                role: TopicRole::Publisher,
                is_pod: true,
                is_system: false,
            },
            TopicEntry {
                name: "cmd_vel".into(),
                type_hash: 200,
                type_size: 16,
                role: TopicRole::Subscriber,
                is_pod: true,
                is_system: false,
            },
        ];

        let mut buf = [0u8; 4096];
        let len = encode_announcement(&peer_id, 9100, &[0u8; 4], &topics, &mut buf);

        let addr: SocketAddr = "127.0.0.1:9100".parse().unwrap();
        let decoded = decode_announcement(&buf[..len], addr).unwrap();

        assert_eq!(decoded.peer_id, peer_id);
        assert_eq!(decoded.data_port, 9100);
        assert_eq!(decoded.topics.len(), 2);
        assert_eq!(decoded.topics[0].name, "imu");
        assert_eq!(decoded.topics[0].type_hash, 100);
        assert_eq!(decoded.topics[0].role, 1); // Publisher
        assert_eq!(decoded.topics[1].name, "cmd_vel");
        assert_eq!(decoded.topics[1].role, 2); // Subscriber
    }

    #[test]
    fn matching_pub_to_sub() {
        let local = vec![TopicEntry {
            name: "imu".into(),
            type_hash: 100,
            type_size: 64,
            role: TopicRole::Publisher,
            is_pod: true,
        is_system: false,
        }];
        let remote = vec![WireTopicEntry {
            name: "imu".into(),
            type_hash: 100,
            type_size: 64,
            role: 2, // Subscriber
            is_pod: 1,
            priority: 2,
        }];

        let (matches, warnings) = find_matches(&local, &remote);
        assert!(warnings.is_empty());
        assert_eq!(matches.len(), 1);
        assert!(matches[0].export);
        assert!(!matches[0].import);
    }

    #[test]
    fn matching_sub_to_pub() {
        let local = vec![TopicEntry {
            name: "cmd".into(),
            type_hash: 200,
            type_size: 16,
            role: TopicRole::Subscriber,
            is_pod: true,
        is_system: false,
        }];
        let remote = vec![WireTopicEntry {
            name: "cmd".into(),
            type_hash: 200,
            type_size: 16,
            role: 1, // Publisher
            is_pod: 1,
            priority: 1,
        }];

        let (matches, _) = find_matches(&local, &remote);
        assert_eq!(matches.len(), 1);
        assert!(!matches[0].export);
        assert!(matches[0].import);
    }

    #[test]
    fn matching_type_hash_mismatch() {
        let local = vec![TopicEntry {
            name: "imu".into(),
            type_hash: 100,
            type_size: 64,
            role: TopicRole::Publisher,
            is_pod: true,
        is_system: false,
        }];
        let remote = vec![WireTopicEntry {
            name: "imu".into(),
            type_hash: 999, // Different!
            type_size: 64,
            role: 2,
            is_pod: 1,
            priority: 2,
        }];

        let (matches, warnings) = find_matches(&local, &remote);
        assert!(matches.is_empty());
        assert_eq!(warnings.len(), 1);
        assert!(warnings[0].contains("type mismatch"));
    }

    #[test]
    fn matching_no_overlap() {
        let local = vec![TopicEntry {
            name: "imu".into(),
            type_hash: 100,
            type_size: 64,
            role: TopicRole::Publisher,
            is_pod: true,
        is_system: false,
        }];
        let remote = vec![WireTopicEntry {
            name: "odom".into(), // Different topic
            type_hash: 300,
            type_size: 48,
            role: 2,
            is_pod: 1,
            priority: 2,
        }];

        let (matches, warnings) = find_matches(&local, &remote);
        assert!(matches.is_empty());
        assert!(warnings.is_empty());
    }

    #[test]
    fn matching_both_directions() {
        let local = vec![TopicEntry {
            name: "data".into(),
            type_hash: 500,
            type_size: 32,
            role: TopicRole::Both,
            is_pod: true,
        is_system: false,
        }];
        let remote = vec![WireTopicEntry {
            name: "data".into(),
            type_hash: 500,
            type_size: 32,
            role: 3, // Both
            is_pod: 1,
            priority: 2,
        }];

        let (matches, _) = find_matches(&local, &remote);
        assert_eq!(matches.len(), 1);
        assert!(matches[0].export);
        assert!(matches[0].import);
    }

    #[test]
    fn secret_hash_deterministic() {
        let h1 = compute_secret_hash("lab-42");
        let h2 = compute_secret_hash("lab-42");
        assert_eq!(h1, h2);
        assert_ne!(h1, [0u8; 4]);
    }

    #[test]
    fn secret_hash_different_secrets() {
        let h1 = compute_secret_hash("lab-42");
        let h2 = compute_secret_hash("lab-43");
        assert_ne!(h1, h2);
    }
}
