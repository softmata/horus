/// HORUS Binary Protocol (HBP)
///
/// Minimal overhead wire protocol for network communication
/// Fixed 24-byte header + variable topic name + payload
use std::mem;

/// HORUS Binary Protocol packet format
///
/// Packet structure (all integers in little-endian):
/// ```text
/// ┌──────────────────────────────────────────────────┐
/// │ Magic (4 bytes): 0x484F5255 ("HORU")             │
/// ├──────────────────────────────────────────────────┤
/// │ Version (1 byte): 0x01                           │
/// ├──────────────────────────────────────────────────┤
/// │ Message Type (1 byte):                           │
/// │   0x01 = Data                                    │
/// │   0x02 = Discovery Request                       │
/// │   0x03 = Discovery Response                      │
/// │   0x04 = Heartbeat                               │
/// │   0x05 = Fragment                                │
/// │   0x06 = Router Subscribe                        │
/// │   0x07 = Router Unsubscribe                      │
/// │   0x08 = Router Publish                          │
/// ├──────────────────────────────────────────────────┤
/// │ Sequence Number (4 bytes)                        │
/// ├──────────────────────────────────────────────────┤
/// │ Timestamp (8 bytes, microseconds since epoch)    │
/// ├──────────────────────────────────────────────────┤
/// │ Topic Name Length (2 bytes)                      │
/// ├──────────────────────────────────────────────────┤
/// │ Payload Length (4 bytes)                         │
/// ├──────────────────────────────────────────────────┤
/// │ Topic Name (variable, UTF-8)                     │
/// ├──────────────────────────────────────────────────┤
/// │ Payload (variable, bincode-serialized)           │
/// └──────────────────────────────────────────────────┘
/// ```
///
/// Total header size: 24 bytes + topic_name_len
const MAGIC: u32 = 0x484F5255; // "HORU" in ASCII
const VERSION: u8 = 0x01;

/// Message types for HORUS protocol
#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MessageType {
    Data = 0x01,
    DiscoveryRequest = 0x02,
    DiscoveryResponse = 0x03,
    Heartbeat = 0x04,
    Fragment = 0x05,          // Fragmented message
    RouterSubscribe = 0x06,   // Subscribe to topic via router
    RouterUnsubscribe = 0x07, // Unsubscribe from topic
    RouterPublish = 0x08,     // Publish to router (router forwards to subscribers)
}

/// Fixed-size packet header (24 bytes)
#[repr(C, packed)]
struct PacketHeader {
    magic: u32,
    version: u8,
    msg_type: u8,
    sequence: u32,
    timestamp_us: u64,
    topic_len: u16,
    payload_len: u32,
}

/// HORUS protocol packet
pub struct HorusPacket {
    pub msg_type: MessageType,
    pub sequence: u32,
    pub timestamp_us: u64,
    pub topic: String,
    pub payload: Vec<u8>,
}

impl HorusPacket {
    /// Create a new data packet
    pub fn new_data(topic: String, payload: Vec<u8>, sequence: u32) -> Self {
        Self {
            msg_type: MessageType::Data,
            sequence,
            timestamp_us: Self::now_us(),
            topic,
            payload,
        }
    }

    /// Create a discovery request packet
    pub fn new_discovery_request(topic: String) -> Self {
        Self {
            msg_type: MessageType::DiscoveryRequest,
            sequence: 0,
            timestamp_us: Self::now_us(),
            topic,
            payload: vec![],
        }
    }

    /// Create a discovery response packet
    pub fn new_discovery_response(topic: String, port: u16) -> Self {
        Self {
            msg_type: MessageType::DiscoveryResponse,
            sequence: 0,
            timestamp_us: Self::now_us(),
            topic,
            payload: port.to_le_bytes().to_vec(),
        }
    }

    /// Create a fragment packet
    pub fn new_fragment(topic: String, fragment_data: Vec<u8>, sequence: u32) -> Self {
        Self {
            msg_type: MessageType::Fragment,
            sequence,
            timestamp_us: Self::now_us(),
            topic,
            payload: fragment_data,
        }
    }

    /// Create a router subscribe packet
    pub fn new_router_subscribe(topic: String) -> Self {
        Self {
            msg_type: MessageType::RouterSubscribe,
            sequence: 0,
            timestamp_us: Self::now_us(),
            topic,
            payload: vec![],
        }
    }

    /// Create a router unsubscribe packet
    pub fn new_router_unsubscribe(topic: String) -> Self {
        Self {
            msg_type: MessageType::RouterUnsubscribe,
            sequence: 0,
            timestamp_us: Self::now_us(),
            topic,
            payload: vec![],
        }
    }

    /// Create a router publish packet
    pub fn new_router_publish(topic: String, payload: Vec<u8>, sequence: u32) -> Self {
        Self {
            msg_type: MessageType::RouterPublish,
            sequence,
            timestamp_us: Self::now_us(),
            topic,
            payload,
        }
    }

    /// Encode packet to bytes (zero-allocation where possible)
    pub fn encode(&self, buf: &mut Vec<u8>) -> usize {
        buf.clear();

        let header = PacketHeader {
            magic: MAGIC,
            version: VERSION,
            msg_type: self.msg_type as u8,
            sequence: self.sequence,
            timestamp_us: self.timestamp_us,
            topic_len: self.topic.len() as u16,
            payload_len: self.payload.len() as u32,
        };

        // SAFETY: PacketHeader is repr(C, packed) with no padding invariants. Reading its bytes is safe.
        unsafe {
            let header_bytes = std::slice::from_raw_parts(
                &header as *const PacketHeader as *const u8,
                mem::size_of::<PacketHeader>(),
            );
            buf.extend_from_slice(header_bytes);
        }

        // Write topic name
        buf.extend_from_slice(self.topic.as_bytes());

        // Write payload
        buf.extend_from_slice(&self.payload);

        buf.len()
    }

    /// Decode packet from bytes (zero-copy where possible)
    pub fn decode(buf: &[u8]) -> Result<Self, &'static str> {
        if buf.len() < mem::size_of::<PacketHeader>() {
            return Err("Buffer too small for header");
        }

        // SAFETY: Buffer length is checked above. read_unaligned handles packed struct alignment.
        let header = unsafe { std::ptr::read_unaligned(buf.as_ptr() as *const PacketHeader) };

        // Validate magic
        if header.magic != MAGIC {
            return Err("Invalid magic number");
        }

        // Validate version
        if header.version != VERSION {
            return Err("Unsupported version");
        }

        // Parse message type
        let msg_type = match header.msg_type {
            0x01 => MessageType::Data,
            0x02 => MessageType::DiscoveryRequest,
            0x03 => MessageType::DiscoveryResponse,
            0x04 => MessageType::Heartbeat,
            0x05 => MessageType::Fragment,
            0x06 => MessageType::RouterSubscribe,
            0x07 => MessageType::RouterUnsubscribe,
            0x08 => MessageType::RouterPublish,
            _ => return Err("Unknown message type"),
        };

        let header_size = mem::size_of::<PacketHeader>();
        let topic_start = header_size;
        let topic_end = topic_start + header.topic_len as usize;
        let payload_start = topic_end;
        let payload_end = payload_start + header.payload_len as usize;

        if buf.len() < payload_end {
            return Err("Buffer too small for payload");
        }

        // Extract topic (requires allocation)
        let topic = String::from_utf8_lossy(&buf[topic_start..topic_end]).to_string();

        // Extract payload (clone for now, could use Arc for zero-copy)
        let payload = buf[payload_start..payload_end].to_vec();

        Ok(HorusPacket {
            msg_type,
            sequence: header.sequence,
            timestamp_us: header.timestamp_us,
            topic,
            payload,
        })
    }

    /// Get current timestamp in microseconds
    pub fn now_us() -> u64 {
        use std::time::{SystemTime, UNIX_EPOCH};
        SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .unwrap_or_default()
            .as_micros() as u64
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_encode_decode() {
        let packet = HorusPacket {
            msg_type: MessageType::Data,
            sequence: 42,
            timestamp_us: 123456789,
            topic: "test_topic".to_string(),
            payload: vec![1, 2, 3, 4, 5],
        };

        let mut buf = Vec::new();
        let _size = packet.encode(&mut buf);

        let decoded = HorusPacket::decode(&buf).unwrap();
        assert_eq!(decoded.msg_type, MessageType::Data);
        assert_eq!(decoded.sequence, 42);
        assert_eq!(decoded.topic, "test_topic");
        assert_eq!(decoded.payload, vec![1, 2, 3, 4, 5]);
    }

    #[test]
    fn test_header_size() {
        // Ensure header is exactly 24 bytes for predictable performance
        assert_eq!(mem::size_of::<PacketHeader>(), 24);
    }

    #[test]
    fn test_discovery_request() {
        let packet = HorusPacket::new_discovery_request("sensor_data".to_string());

        let mut buf = Vec::new();
        packet.encode(&mut buf);

        let decoded = HorusPacket::decode(&buf).unwrap();
        assert_eq!(decoded.msg_type, MessageType::DiscoveryRequest);
        assert_eq!(decoded.topic, "sensor_data");
    }

    #[test]
    fn test_discovery_response() {
        let packet = HorusPacket::new_discovery_response("sensor_data".to_string(), 9870);

        let mut buf = Vec::new();
        packet.encode(&mut buf);

        let decoded = HorusPacket::decode(&buf).unwrap();
        assert_eq!(decoded.msg_type, MessageType::DiscoveryResponse);
        assert_eq!(decoded.topic, "sensor_data");

        let port = u16::from_le_bytes([decoded.payload[0], decoded.payload[1]]);
        assert_eq!(port, 9870);
    }
}
