//! Wire format v1 — packet headers, message headers, encode/decode.
//!
//! All encoding/decoding is manual byte layout — no serde, no external deps.
//! See blueprint section 6 for full specification.
//!
//! ## Packet layout
//!
//! ```text
//! [PacketHeader: 12 bytes]
//! [MessageHeader: 24 bytes] [payload: N bytes]
//! [MessageHeader: 24 bytes] [payload: N bytes]  (if batch flag set)
//! ...
//! ```
//!
//! Total overhead per single message: 36 bytes (12 + 24).

use crate::priority::{Encoding, Priority, Reliability};

/// Magic bytes: "HORS" in ASCII.
pub const MAGIC: u32 = 0x484F5253;

/// Wire format version.
pub const VERSION: u8 = 1;

/// Maximum practical UDP payload (leave room for IP/UDP headers).
pub const MAX_UDP_PAYLOAD: usize = 65507;

// ─── Packet Header (12 bytes) ───────────────────────────────────────────────

/// Packet header — first 12 bytes of every UDP datagram.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PacketHeader {
    /// Magic: 0x484F5253 ("HORS").
    pub magic: u32,
    /// Wire format version (1).
    pub version: u8,
    /// Bitfield flags.
    pub flags: PacketFlags,
    /// Truncated peer ID hash (2 bytes).
    pub sender_id_hash: u16,
    /// Per-sender monotonic sequence for loss rate calculation.
    pub packet_sequence: u32,
}

impl PacketHeader {
    pub const SIZE: usize = 12;

    pub fn new(flags: PacketFlags, sender_id_hash: u16, packet_sequence: u32) -> Self {
        Self {
            magic: MAGIC,
            version: VERSION,
            flags,
            sender_id_hash,
            packet_sequence,
        }
    }

    /// Encode into exactly 12 bytes (little-endian).
    pub fn encode(&self, buf: &mut [u8]) {
        debug_assert!(buf.len() >= Self::SIZE);
        buf[0..4].copy_from_slice(&self.magic.to_le_bytes());
        buf[4] = self.version;
        buf[5] = self.flags.bits();
        buf[6..8].copy_from_slice(&self.sender_id_hash.to_le_bytes());
        buf[8..12].copy_from_slice(&self.packet_sequence.to_le_bytes());
    }

    /// Decode from exactly 12 bytes. Returns None if magic or version mismatch.
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
        let flags = PacketFlags::from_bits(buf[5]);
        let sender_id_hash = u16::from_le_bytes([buf[6], buf[7]]);
        let packet_sequence = u32::from_le_bytes([buf[8], buf[9], buf[10], buf[11]]);

        Some(Self {
            magic,
            version,
            flags,
            sender_id_hash,
            packet_sequence,
        })
    }
}

// ─── Packet Flags ───────────────────────────────────────────────────────────

/// Bitfield flags in the packet header.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct PacketFlags(u8);

impl PacketFlags {
    pub const BATCH: u8 = 1 << 0;
    pub const DELTA: u8 = 1 << 1;
    pub const COMPRESSED: u8 = 1 << 2;
    pub const FRAGMENT: u8 = 1 << 3;
    pub const HEARTBEAT: u8 = 1 << 4;
    pub const ACK: u8 = 1 << 5;

    pub const fn empty() -> Self {
        Self(0)
    }

    pub const fn from_bits(bits: u8) -> Self {
        Self(bits)
    }

    pub const fn bits(self) -> u8 {
        self.0
    }

    pub const fn batch(self) -> bool {
        self.0 & Self::BATCH != 0
    }

    pub const fn delta(self) -> bool {
        self.0 & Self::DELTA != 0
    }

    pub const fn compressed(self) -> bool {
        self.0 & Self::COMPRESSED != 0
    }

    pub const fn fragment(self) -> bool {
        self.0 & Self::FRAGMENT != 0
    }

    pub const fn heartbeat(self) -> bool {
        self.0 & Self::HEARTBEAT != 0
    }

    pub const fn ack(self) -> bool {
        self.0 & Self::ACK != 0
    }

    pub const fn with(self, flag: u8) -> Self {
        Self(self.0 | flag)
    }
}

// ─── Message Header (24 bytes) ──────────────────────────────────────────────

/// Per-message header — follows the packet header, one per message in the packet.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct MessageHeader {
    /// FNV-1a hash of the topic type name (u32).
    pub topic_hash: u32,
    /// Payload length in bytes.
    pub payload_len: u32,
    /// Source timestamp (nanoseconds since Unix epoch).
    pub timestamp_ns: u64,
    /// Per-topic monotonic sequence number.
    pub sequence: u32,
    /// Priority level.
    pub priority: Priority,
    /// Reliability tier.
    pub reliability: Reliability,
    /// Payload encoding (endianness tag).
    pub encoding: Encoding,
    /// Source host identification — low byte of sender's peer_id_hash.
    /// Allows receivers to distinguish which host published each message,
    /// even when topic names are shared across machines.
    /// Set by the replicator on send; 0 = unknown/local.
    pub source_host: u8,
}

impl MessageHeader {
    pub const SIZE: usize = 24;

    /// Encode into exactly 24 bytes (little-endian).
    pub fn encode(&self, buf: &mut [u8]) {
        debug_assert!(buf.len() >= Self::SIZE);
        buf[0..4].copy_from_slice(&self.topic_hash.to_le_bytes());
        buf[4..8].copy_from_slice(&self.payload_len.to_le_bytes());
        buf[8..16].copy_from_slice(&self.timestamp_ns.to_le_bytes());
        buf[16..20].copy_from_slice(&self.sequence.to_le_bytes());
        buf[20] = self.priority as u8;
        buf[21] = self.reliability as u8;
        buf[22] = self.encoding as u8;
        buf[23] = self.source_host;
    }

    /// Decode from exactly 24 bytes. Returns None if buffer too small.
    pub fn decode(buf: &[u8]) -> Option<Self> {
        if buf.len() < Self::SIZE {
            return None;
        }
        Some(Self {
            topic_hash: u32::from_le_bytes([buf[0], buf[1], buf[2], buf[3]]),
            payload_len: u32::from_le_bytes([buf[4], buf[5], buf[6], buf[7]]),
            timestamp_ns: u64::from_le_bytes([
                buf[8], buf[9], buf[10], buf[11], buf[12], buf[13], buf[14], buf[15],
            ]),
            sequence: u32::from_le_bytes([buf[16], buf[17], buf[18], buf[19]]),
            priority: Priority::from_u8(buf[20]),
            reliability: Reliability::from_u8(buf[21]),
            encoding: Encoding::from_u8(buf[22]),
            source_host: buf[23],
        })
    }
}

// ─── FNV-1a Hash ────────────────────────────────────────────────────────────

/// FNV-1a hash (32-bit) for topic names.
///
/// Deterministic, fast, collision-resistant for short strings.
/// Used as `topic_hash` in the wire format.
pub fn fnv1a_hash(data: &[u8]) -> u32 {
    const FNV_OFFSET: u32 = 2166136261;
    const FNV_PRIME: u32 = 16777619;

    let mut hash = FNV_OFFSET;
    for &byte in data {
        hash ^= byte as u32;
        hash = hash.wrapping_mul(FNV_PRIME);
    }
    hash
}

/// Hash a topic name string.
pub fn topic_hash(name: &str) -> u32 {
    fnv1a_hash(name.as_bytes())
}

// ─── Fragment Header (12 bytes, appended after MessageHeader when flags.fragment=1) ─

/// Fragment header — present when a message is split across multiple packets.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct FragmentHeader {
    /// Same for all fragments of one message.
    pub fragment_id: u32,
    /// 0-based index within the message.
    pub fragment_index: u16,
    /// Total number of fragments.
    pub fragment_count: u16,
    /// Full message size before splitting.
    pub total_payload_len: u32,
}

impl FragmentHeader {
    pub const SIZE: usize = 12;

    pub fn encode(&self, buf: &mut [u8]) {
        debug_assert!(buf.len() >= Self::SIZE);
        buf[0..4].copy_from_slice(&self.fragment_id.to_le_bytes());
        buf[4..6].copy_from_slice(&self.fragment_index.to_le_bytes());
        buf[6..8].copy_from_slice(&self.fragment_count.to_le_bytes());
        buf[8..12].copy_from_slice(&self.total_payload_len.to_le_bytes());
    }

    pub fn decode(buf: &[u8]) -> Option<Self> {
        if buf.len() < Self::SIZE {
            return None;
        }
        Some(Self {
            fragment_id: u32::from_le_bytes([buf[0], buf[1], buf[2], buf[3]]),
            fragment_index: u16::from_le_bytes([buf[4], buf[5]]),
            fragment_count: u16::from_le_bytes([buf[6], buf[7]]),
            total_payload_len: u32::from_le_bytes([buf[8], buf[9], buf[10], buf[11]]),
        })
    }
}

// ─── Heartbeat Payload (20 bytes, when flags.heartbeat=1) ───────────────────

/// Heartbeat payload — sent every 50ms to each matched peer.
/// Total packet: 12B header + 20B payload = 32 bytes.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct HeartbeatPayload {
    /// Unique peer identifier (128-bit random).
    pub peer_id: [u8; 16],
    /// Monotonic heartbeat sequence.
    pub heartbeat_sequence: u32,
}

impl HeartbeatPayload {
    pub const SIZE: usize = 20;

    pub fn encode(&self, buf: &mut [u8]) {
        debug_assert!(buf.len() >= Self::SIZE);
        buf[0..16].copy_from_slice(&self.peer_id);
        buf[16..20].copy_from_slice(&self.heartbeat_sequence.to_le_bytes());
    }

    pub fn decode(buf: &[u8]) -> Option<Self> {
        if buf.len() < Self::SIZE {
            return None;
        }
        let mut peer_id = [0u8; 16];
        peer_id.copy_from_slice(&buf[0..16]);
        Some(Self {
            peer_id,
            heartbeat_sequence: u32::from_le_bytes([buf[16], buf[17], buf[18], buf[19]]),
        })
    }
}

// ─── Ack Payload (8 bytes, when flags.ack=1) ────────────────────────────────

/// Acknowledgment payload — confirms receipt of a latched message.
/// Total packet: 12B header + 8B payload = 20 bytes.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct AckPayload {
    /// Topic hash of the acknowledged message.
    pub acked_topic_hash: u32,
    /// Sequence number of the acknowledged message.
    pub acked_sequence: u32,
}

impl AckPayload {
    pub const SIZE: usize = 8;

    pub fn encode(&self, buf: &mut [u8]) {
        debug_assert!(buf.len() >= Self::SIZE);
        buf[0..4].copy_from_slice(&self.acked_topic_hash.to_le_bytes());
        buf[4..8].copy_from_slice(&self.acked_sequence.to_le_bytes());
    }

    pub fn decode(buf: &[u8]) -> Option<Self> {
        if buf.len() < Self::SIZE {
            return None;
        }
        Some(Self {
            acked_topic_hash: u32::from_le_bytes([buf[0], buf[1], buf[2], buf[3]]),
            acked_sequence: u32::from_le_bytes([buf[4], buf[5], buf[6], buf[7]]),
        })
    }
}

// ─── Encode/Decode Helpers for Special Packets ──────────────────────────────

/// Encode a heartbeat packet. Returns bytes written.
pub fn encode_heartbeat(
    sender_id_hash: u16,
    packet_sequence: u32,
    payload: &HeartbeatPayload,
    buf: &mut [u8],
) -> usize {
    let total = PacketHeader::SIZE + HeartbeatPayload::SIZE;
    debug_assert!(buf.len() >= total);
    let header = PacketHeader::new(
        PacketFlags::empty().with(PacketFlags::HEARTBEAT),
        sender_id_hash,
        packet_sequence,
    );
    header.encode(&mut buf[..PacketHeader::SIZE]);
    payload.encode(&mut buf[PacketHeader::SIZE..]);
    total
}

/// Decode a heartbeat payload from a packet (after header).
pub fn decode_heartbeat(buf: &[u8]) -> Option<HeartbeatPayload> {
    if buf.len() < PacketHeader::SIZE + HeartbeatPayload::SIZE {
        return None;
    }
    HeartbeatPayload::decode(&buf[PacketHeader::SIZE..])
}

/// Encode an ACK packet. Returns bytes written.
pub fn encode_ack(
    sender_id_hash: u16,
    packet_sequence: u32,
    payload: &AckPayload,
    buf: &mut [u8],
) -> usize {
    let total = PacketHeader::SIZE + AckPayload::SIZE;
    debug_assert!(buf.len() >= total);
    let header = PacketHeader::new(
        PacketFlags::empty().with(PacketFlags::ACK),
        sender_id_hash,
        packet_sequence,
    );
    header.encode(&mut buf[..PacketHeader::SIZE]);
    payload.encode(&mut buf[PacketHeader::SIZE..]);
    total
}

/// Decode an ACK payload from a packet (after header).
pub fn decode_ack(buf: &[u8]) -> Option<AckPayload> {
    if buf.len() < PacketHeader::SIZE + AckPayload::SIZE {
        return None;
    }
    AckPayload::decode(&buf[PacketHeader::SIZE..])
}

// ─── Wire Messages (for internal pipeline) ──────────────────────────────────

/// Outgoing message — ready to be sent over the wire.
#[derive(Debug, Clone)]
pub struct OutMessage {
    pub topic_name: String,
    pub topic_hash: u32,
    pub payload: Vec<u8>,
    pub timestamp_ns: u64,
    pub sequence: u32,
    pub priority: Priority,
    pub reliability: Reliability,
    pub encoding: Encoding,
}

/// Incoming message — received from the wire.
#[derive(Debug, Clone)]
pub struct InMessage {
    pub topic_hash: u32,
    pub payload: Vec<u8>,
    pub timestamp_ns: u64,
    pub sequence: u32,
    pub priority: Priority,
    pub reliability: Reliability,
    pub encoding: Encoding,
}

// ─── Encode/Decode Full Packets ─────────────────────────────────────────────

/// Encode a single data message into a packet buffer.
/// Returns the number of bytes written.
pub fn encode_single(header: &PacketHeader, msg: &OutMessage, buf: &mut [u8]) -> usize {
    let total = PacketHeader::SIZE + MessageHeader::SIZE + msg.payload.len();
    assert!(buf.len() >= total, "buffer too small for packet");

    header.encode(&mut buf[..PacketHeader::SIZE]);

    let msg_header = MessageHeader {
        topic_hash: msg.topic_hash,
        payload_len: msg.payload.len() as u32,
        timestamp_ns: msg.timestamp_ns,
        sequence: msg.sequence,
        priority: msg.priority,
        reliability: msg.reliability,
        encoding: msg.encoding,
        source_host: 0,
    };
    let mh_start = PacketHeader::SIZE;
    msg_header.encode(&mut buf[mh_start..mh_start + MessageHeader::SIZE]);

    let payload_start = mh_start + MessageHeader::SIZE;
    buf[payload_start..payload_start + msg.payload.len()].copy_from_slice(&msg.payload);

    total
}

/// Encode multiple messages into a single batched packet.
/// Returns the number of bytes written.
pub fn encode_batch(header: &PacketHeader, msgs: &[OutMessage], buf: &mut [u8]) -> usize {
    debug_assert!(header.flags.batch() || msgs.len() <= 1);

    let mut offset = PacketHeader::SIZE;
    header.encode(&mut buf[..PacketHeader::SIZE]);

    for msg in msgs {
        let mh = MessageHeader {
            topic_hash: msg.topic_hash,
            payload_len: msg.payload.len() as u32,
            timestamp_ns: msg.timestamp_ns,
            sequence: msg.sequence,
            priority: msg.priority,
            reliability: msg.reliability,
            encoding: msg.encoding,
            source_host: 0,
        };
        mh.encode(&mut buf[offset..offset + MessageHeader::SIZE]);
        offset += MessageHeader::SIZE;

        buf[offset..offset + msg.payload.len()].copy_from_slice(&msg.payload);
        offset += msg.payload.len();
    }

    offset
}

/// Decode all messages from a received packet.
/// Returns the packet header and a list of incoming messages.
pub fn decode_packet(buf: &[u8]) -> Option<(PacketHeader, Vec<InMessage>)> {
    let header = PacketHeader::decode(buf)?;

    // Heartbeat and ACK packets have special payloads — handled separately
    if header.flags.heartbeat() || header.flags.ack() {
        return Some((header, Vec::new()));
    }

    let mut messages = Vec::new();
    let mut offset = PacketHeader::SIZE;

    while offset + MessageHeader::SIZE <= buf.len() {
        let mh = MessageHeader::decode(&buf[offset..])?;
        offset += MessageHeader::SIZE;

        let payload_len = mh.payload_len as usize;
        if offset + payload_len > buf.len() {
            break; // Truncated — discard partial message
        }

        let payload = buf[offset..offset + payload_len].to_vec();
        offset += payload_len;

        messages.push(InMessage {
            topic_hash: mh.topic_hash,
            payload,
            timestamp_ns: mh.timestamp_ns,
            sequence: mh.sequence,
            priority: mh.priority,
            reliability: mh.reliability,
            encoding: mh.encoding,
        });
    }

    Some((header, messages))
}

// ─── Tests ──────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn packet_header_size() {
        assert_eq!(PacketHeader::SIZE, 12);
    }

    #[test]
    fn message_header_size() {
        assert_eq!(MessageHeader::SIZE, 24);
    }

    #[test]
    fn total_overhead() {
        assert_eq!(PacketHeader::SIZE + MessageHeader::SIZE, 36);
    }

    #[test]
    fn magic_bytes() {
        assert_eq!(MAGIC, 0x484F5253);
        // "HORS" in ASCII (little-endian: S=0x53, R=0x52, O=0x4F, H=0x48)
        let bytes = MAGIC.to_le_bytes();
        assert_eq!(&bytes, &[0x53, 0x52, 0x4F, 0x48]);
    }

    #[test]
    fn packet_header_roundtrip() {
        let header = PacketHeader::new(PacketFlags::empty().with(PacketFlags::BATCH), 0xABCD, 42);
        let mut buf = [0u8; PacketHeader::SIZE];
        header.encode(&mut buf);
        let decoded = PacketHeader::decode(&buf).unwrap();
        assert_eq!(header, decoded);
    }

    #[test]
    fn packet_header_bad_magic() {
        let mut buf = [0u8; PacketHeader::SIZE];
        let header = PacketHeader::new(PacketFlags::empty(), 0, 0);
        header.encode(&mut buf);
        buf[0] = 0xFF; // Corrupt magic
        assert!(PacketHeader::decode(&buf).is_none());
    }

    #[test]
    fn packet_header_bad_version() {
        let mut buf = [0u8; PacketHeader::SIZE];
        let header = PacketHeader::new(PacketFlags::empty(), 0, 0);
        header.encode(&mut buf);
        buf[4] = 99; // Wrong version
        assert!(PacketHeader::decode(&buf).is_none());
    }

    #[test]
    fn packet_flags() {
        let f = PacketFlags::empty()
            .with(PacketFlags::BATCH)
            .with(PacketFlags::FRAGMENT);
        assert!(f.batch());
        assert!(f.fragment());
        assert!(!f.heartbeat());
        assert!(!f.ack());
        assert!(!f.delta());
        assert!(!f.compressed());
    }

    #[test]
    fn message_header_roundtrip() {
        let mh = MessageHeader {
            topic_hash: topic_hash("robot.imu"),
            payload_len: 256,
            timestamp_ns: 1234567890123456789,
            sequence: 999,
            priority: Priority::RealTime,
            reliability: Reliability::Redundant,
            encoding: Encoding::PodLe,
            source_host: 0,
        };
        let mut buf = [0u8; MessageHeader::SIZE];
        mh.encode(&mut buf);
        let decoded = MessageHeader::decode(&buf).unwrap();
        assert_eq!(mh, decoded);
    }

    #[test]
    fn message_header_all_priorities() {
        for p in [
            Priority::Immediate,
            Priority::RealTime,
            Priority::Normal,
            Priority::Bulk,
        ] {
            let mh = MessageHeader {
                topic_hash: 0,
                payload_len: 0,
                timestamp_ns: 0,
                sequence: 0,
                priority: p,
                reliability: Reliability::None,
                encoding: Encoding::Bincode,
                source_host: 0,
            };
            let mut buf = [0u8; MessageHeader::SIZE];
            mh.encode(&mut buf);
            let decoded = MessageHeader::decode(&buf).unwrap();
            assert_eq!(decoded.priority, p);
        }
    }

    #[test]
    fn fnv1a_known_values() {
        // FNV-1a reference: empty string
        assert_eq!(fnv1a_hash(b""), 2166136261);
        // Known test vectors
        assert_ne!(fnv1a_hash(b"imu"), fnv1a_hash(b"cmd_vel"));
        assert_ne!(fnv1a_hash(b"robot.imu"), fnv1a_hash(b"robot.odom"));
    }

    #[test]
    fn fnv1a_deterministic() {
        let h1 = topic_hash("robot.imu");
        let h2 = topic_hash("robot.imu");
        assert_eq!(h1, h2);
    }

    #[test]
    fn fnv1a_collision_resistance() {
        // Hash 1000 unique topic names, verify no collisions
        use std::collections::HashSet;
        let mut hashes = HashSet::new();
        for i in 0..1000u32 {
            let name = format!("robot_{}.sensor_{}.data", i / 10, i % 10);
            let h = topic_hash(&name);
            assert!(hashes.insert(h), "collision at {name}");
        }
    }

    #[test]
    fn encode_decode_single_message() {
        let header = PacketHeader::new(PacketFlags::empty(), 0x1234, 1);
        let msg = OutMessage {
            topic_name: "imu".into(),
            topic_hash: topic_hash("imu"),
            payload: vec![1, 2, 3, 4, 5, 6, 7, 8],
            timestamp_ns: 99999,
            sequence: 42,
            priority: Priority::Normal,
            reliability: Reliability::None,
            encoding: Encoding::PodLe,
        };

        let mut buf = [0u8; 256];
        let len = encode_single(&header, &msg, &mut buf);
        assert_eq!(len, 12 + 24 + 8); // header + msg_header + payload

        let (decoded_header, decoded_msgs) = decode_packet(&buf[..len]).unwrap();
        assert_eq!(decoded_header, header);
        assert_eq!(decoded_msgs.len(), 1);
        assert_eq!(decoded_msgs[0].topic_hash, msg.topic_hash);
        assert_eq!(decoded_msgs[0].payload, msg.payload);
        assert_eq!(decoded_msgs[0].timestamp_ns, msg.timestamp_ns);
        assert_eq!(decoded_msgs[0].sequence, msg.sequence);
        assert_eq!(decoded_msgs[0].priority, msg.priority);
        assert_eq!(decoded_msgs[0].encoding, msg.encoding);
    }

    #[test]
    fn encode_decode_batch() {
        let header = PacketHeader::new(PacketFlags::empty().with(PacketFlags::BATCH), 0x5678, 10);
        let msgs: Vec<OutMessage> = (0..5)
            .map(|i| OutMessage {
                topic_name: format!("topic_{i}"),
                topic_hash: topic_hash(&format!("topic_{i}")),
                payload: vec![i as u8; 16],
                timestamp_ns: 1000 + i as u64,
                sequence: i,
                priority: Priority::Normal,
                reliability: Reliability::None,
                encoding: Encoding::PodLe,
            })
            .collect();

        let mut buf = [0u8; 4096];
        let len = encode_batch(&header, &msgs, &mut buf);

        let expected = 12 + 5 * (24 + 16);
        assert_eq!(len, expected);

        let (decoded_header, decoded_msgs) = decode_packet(&buf[..len]).unwrap();
        assert_eq!(decoded_header, header);
        assert_eq!(decoded_msgs.len(), 5);

        for (i, m) in decoded_msgs.iter().enumerate() {
            assert_eq!(m.topic_hash, topic_hash(&format!("topic_{i}")));
            assert_eq!(m.payload, vec![i as u8; 16]);
            assert_eq!(m.sequence, i as u32);
        }
    }

    #[test]
    fn decode_truncated_packet() {
        // Too short for packet header
        assert!(decode_packet(&[0u8; 5]).is_none());
    }

    #[test]
    fn decode_truncated_message() {
        // Valid header but message payload extends past buffer
        let header = PacketHeader::new(PacketFlags::empty(), 0, 0);
        let msg = OutMessage {
            topic_name: "x".into(),
            topic_hash: topic_hash("x"),
            payload: vec![0u8; 100],
            timestamp_ns: 0,
            sequence: 0,
            priority: Priority::Normal,
            reliability: Reliability::None,
            encoding: Encoding::PodLe,
        };
        let mut buf = [0u8; 256];
        let len = encode_single(&header, &msg, &mut buf);

        // Truncate: cut off half the payload
        let (_, msgs) = decode_packet(&buf[..len - 50]).unwrap();
        assert_eq!(msgs.len(), 0); // Truncated message discarded
    }

    #[test]
    fn heartbeat_packet_no_messages() {
        let header =
            PacketHeader::new(PacketFlags::empty().with(PacketFlags::HEARTBEAT), 0x1111, 5);
        let mut buf = [0u8; 64];
        header.encode(&mut buf);
        // Add heartbeat payload after header (not parsed as messages)
        let (decoded, msgs) = decode_packet(&buf[..PacketHeader::SIZE + 20]).unwrap();
        assert!(decoded.flags.heartbeat());
        assert!(msgs.is_empty());
    }

    // ─── Fragment Header Tests ──────────────────────────────────────────

    #[test]
    fn fragment_header_size() {
        assert_eq!(FragmentHeader::SIZE, 12);
    }

    #[test]
    fn fragment_header_roundtrip() {
        let fh = FragmentHeader {
            fragment_id: 0xDEADBEEF,
            fragment_index: 3,
            fragment_count: 10,
            total_payload_len: 500_000,
        };
        let mut buf = [0u8; FragmentHeader::SIZE];
        fh.encode(&mut buf);
        let decoded = FragmentHeader::decode(&buf).unwrap();
        assert_eq!(fh, decoded);
    }

    #[test]
    fn fragment_header_max_values() {
        let fh = FragmentHeader {
            fragment_id: u32::MAX,
            fragment_index: u16::MAX,
            fragment_count: u16::MAX,
            total_payload_len: u32::MAX,
        };
        let mut buf = [0u8; FragmentHeader::SIZE];
        fh.encode(&mut buf);
        let decoded = FragmentHeader::decode(&buf).unwrap();
        assert_eq!(fh, decoded);
    }

    #[test]
    fn fragment_header_too_short() {
        assert!(FragmentHeader::decode(&[0u8; 5]).is_none());
    }

    // ─── Heartbeat Payload Tests ────────────────────────────────────────

    #[test]
    fn heartbeat_payload_size() {
        assert_eq!(HeartbeatPayload::SIZE, 20);
    }

    #[test]
    fn heartbeat_total_packet_size() {
        // Blueprint: 32 bytes total (12B header + 20B payload)
        assert_eq!(PacketHeader::SIZE + HeartbeatPayload::SIZE, 32);
    }

    #[test]
    fn heartbeat_payload_roundtrip() {
        let hb = HeartbeatPayload {
            peer_id: [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16],
            heartbeat_sequence: 12345,
        };
        let mut buf = [0u8; HeartbeatPayload::SIZE];
        hb.encode(&mut buf);
        let decoded = HeartbeatPayload::decode(&buf).unwrap();
        assert_eq!(hb, decoded);
    }

    #[test]
    fn heartbeat_encode_decode_full_packet() {
        let payload = HeartbeatPayload {
            peer_id: [0xAA; 16],
            heartbeat_sequence: 999,
        };
        let mut buf = [0u8; 64];
        let len = encode_heartbeat(0x1234, 42, &payload, &mut buf);
        assert_eq!(len, 32);

        // Verify header
        let header = PacketHeader::decode(&buf).unwrap();
        assert!(header.flags.heartbeat());
        assert_eq!(header.sender_id_hash, 0x1234);
        assert_eq!(header.packet_sequence, 42);

        // Verify payload
        let decoded = decode_heartbeat(&buf).unwrap();
        assert_eq!(decoded, payload);
    }

    #[test]
    fn heartbeat_too_short() {
        assert!(HeartbeatPayload::decode(&[0u8; 10]).is_none());
    }

    // ─── Ack Payload Tests ──────────────────────────────────────────────

    #[test]
    fn ack_payload_size() {
        assert_eq!(AckPayload::SIZE, 8);
    }

    #[test]
    fn ack_total_packet_size() {
        // Blueprint: 20 bytes total (12B header + 8B payload)
        assert_eq!(PacketHeader::SIZE + AckPayload::SIZE, 20);
    }

    #[test]
    fn ack_payload_roundtrip() {
        let ack = AckPayload {
            acked_topic_hash: topic_hash("robot.estop"),
            acked_sequence: 77,
        };
        let mut buf = [0u8; AckPayload::SIZE];
        ack.encode(&mut buf);
        let decoded = AckPayload::decode(&buf).unwrap();
        assert_eq!(ack, decoded);
    }

    #[test]
    fn ack_encode_decode_full_packet() {
        let payload = AckPayload {
            acked_topic_hash: 0xCAFEBABE,
            acked_sequence: 555,
        };
        let mut buf = [0u8; 64];
        let len = encode_ack(0x5678, 100, &payload, &mut buf);
        assert_eq!(len, 20);

        // Verify header
        let header = PacketHeader::decode(&buf).unwrap();
        assert!(header.flags.ack());

        // Verify payload
        let decoded = decode_ack(&buf).unwrap();
        assert_eq!(decoded, payload);
    }

    #[test]
    fn ack_too_short() {
        assert!(AckPayload::decode(&[0u8; 4]).is_none());
    }
}
