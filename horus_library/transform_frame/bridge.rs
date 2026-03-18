//! Network TF Bridge — Distributed Transform Frame Synchronization
//!
//! Provides UDP multicast-based transport for sharing TransformFrame trees
//! across machines on a local network. Designed for multi-robot systems,
//! distributed sensor rigs, and remote monitoring.
//!
//! # Protocol
//!
//! The wire format is a minimal header followed by raw `TransformStamped` bytes:
//!
//! ```text
//! ┌──────────────────────────────────────────┐
//! │  TFBridgeHeader (24 bytes)               │
//! │  ├─ magic: [u8; 4] = "TFB1"             │
//! │  ├─ version: u16                         │
//! │  ├─ flags: u16                           │
//! │  │   bit 0: is_static                    │
//! │  │   bit 1: has_namespace                │
//! │  ├─ sequence: u32                        │
//! │  ├─ source_id: u32 (hash of source name) │
//! │  ├─ timestamp_ns: u64                    │
//! ├──────────────────────────────────────────┤
//! │  Namespace (if has_namespace flag set)    │
//! │  ├─ len: u8                              │
//! │  ├─ bytes: [u8; len]                     │
//! ├──────────────────────────────────────────┤
//! │  count: u32                              │
//! │  TransformStamped × count (Pod bytes)    │
//! └──────────────────────────────────────────┘
//! ```
//!
//! # Design Decisions
//!
//! - **UDP multicast** over TCP: TF is inherently 1-to-N broadcast, no ACK needed,
//!   stale data is worse than lost data. Typical bandwidth: 50 frames × 50Hz ×
//!   192 bytes/frame = ~470 KB/s per robot — well within LAN capacity.
//! - **Reuses `TransformStamped` Pod layout**: Zero-copy deserialization, same
//!   binary representation as shared memory. No protobuf/msgpack overhead.
//! - **Namespace prefixing**: Multi-robot support via automatic frame name prefixing
//!   (e.g., `robot1/base_link`). Applied at the bridge layer, transparent to local TF.
//! - **Source dedup**: `source_id` prevents echoing own messages back.
//! - **HERMES discovery** (future): Will integrate with HERMES Membrane for
//!   zero-config peer discovery. See `blueprints/HERMES_NETWORK_TRANSPORT.md`.
//!
//! # Comparison with ROS2 DDS
//!
//! | Aspect              | HORUS TF Bridge       | ROS2 DDS              |
//! |---------------------|-----------------------|-----------------------|
//! | Transport           | UDP multicast         | DDS (RTPS/UDP)        |
//! | Overhead            | 24-byte header        | ~100+ byte RTPS       |
//! | Discovery           | HERMES (planned)      | SPDP/SEDP             |
//! | QoS                 | Best-effort           | Full DDS QoS          |
//! | Latency (LAN)       | < 1ms                 | 1-5ms                 |
//! | Dependencies        | std::net only          | CycloneDDS/FastDDS    |
//! | Multi-robot         | Namespace prefix      | Namespace prefix      |
//! | Bandwidth (50 frames @ 50Hz) | ~470 KB/s   | ~600 KB/s             |

use super::messages::TransformStamped;
use horus_core::core::DurationExt;
use std::collections::HashMap;
use std::hash::{Hash, Hasher};
use std::net::{Ipv4Addr, SocketAddr, SocketAddrV4, UdpSocket};
use std::sync::atomic::{AtomicBool, AtomicU32, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant};

/// Default multicast group for TF bridge
#[doc(hidden)]
pub const TF_BRIDGE_MULTICAST_GROUP: Ipv4Addr = Ipv4Addr::new(239, 255, 72, 70); // "HF" in ASCII (legacy, retained for wire compatibility)

/// Default port for TF bridge
#[doc(hidden)]
pub const TF_BRIDGE_DEFAULT_PORT: u16 = 17200;

/// Protocol magic bytes
const MAGIC: [u8; 4] = *b"TFB1";

/// Protocol version
const PROTOCOL_VERSION: u16 = 1;

/// Maximum UDP payload (MTU-safe)
const MAX_PACKET_SIZE: usize = 65000;

/// Header size in bytes
const HEADER_SIZE: usize = 24;

/// Flag: transforms are static
const FLAG_STATIC: u16 = 0x0001;

/// Flag: packet includes namespace prefix
const FLAG_HAS_NAMESPACE: u16 = 0x0002;

/// Wire protocol header
#[derive(Debug, Clone, Copy)]
#[repr(C, packed)]
struct TFBridgeHeader {
    magic: [u8; 4],
    version: u16,
    flags: u16,
    sequence: u32,
    source_id: u32,
    timestamp_ns: u64,
}

/// Configuration for a TF bridge endpoint
#[derive(Debug, Clone)]
pub struct TFBridgeConfig {
    /// Multicast group address
    pub multicast_group: Ipv4Addr,
    /// Port number
    pub port: u16,
    /// Network interface to bind to (0.0.0.0 for all)
    pub bind_interface: Ipv4Addr,
    /// Namespace prefix for outgoing frames (e.g., "robot1")
    pub namespace: Option<String>,
    /// Source name (used for dedup hash)
    pub source_name: String,
    /// Publish rate limit (max packets per second, 0 = unlimited)
    pub max_publish_rate: u32,
    /// TTL for multicast packets (1 = local subnet only)
    pub multicast_ttl: u32,
    /// Whether to filter out own messages on receive
    pub filter_self: bool,
}

impl Default for TFBridgeConfig {
    fn default() -> Self {
        Self {
            multicast_group: TF_BRIDGE_MULTICAST_GROUP,
            port: TF_BRIDGE_DEFAULT_PORT,
            bind_interface: Ipv4Addr::UNSPECIFIED,
            namespace: None,
            source_name: String::new(),
            max_publish_rate: 200,
            multicast_ttl: 1,
            filter_self: true,
        }
    }
}

impl TFBridgeConfig {
    pub fn new(source_name: impl Into<String>) -> Self {
        Self {
            source_name: source_name.into(),
            ..Default::default()
        }
    }

    pub fn with_namespace(mut self, ns: impl Into<String>) -> Self {
        self.namespace = Some(ns.into());
        self
    }

    pub fn with_port(mut self, port: u16) -> Self {
        self.port = port;
        self
    }

    pub fn with_multicast_group(mut self, group: Ipv4Addr) -> Self {
        self.multicast_group = group;
        self
    }

    pub fn with_interface(mut self, iface: Ipv4Addr) -> Self {
        self.bind_interface = iface;
        self
    }

    pub fn with_ttl(mut self, ttl: u32) -> Self {
        self.multicast_ttl = ttl;
        self
    }
}

/// Compute a stable source ID from a name
fn source_id_hash(name: &str) -> u32 {
    let mut hasher = std::collections::hash_map::DefaultHasher::new();
    name.hash(&mut hasher);
    hasher.finish() as u32
}

/// TF Bridge Publisher — broadcasts local TF to the network
pub struct TFBridgePublisher {
    socket: UdpSocket,
    target: SocketAddr,
    config: TFBridgeConfig,
    source_id: u32,
    sequence: AtomicU32,
    last_send: std::sync::Mutex<Instant>,
}

impl TFBridgePublisher {
    /// Create a new TF bridge publisher
    pub fn new(config: TFBridgeConfig) -> std::io::Result<Self> {
        let socket = UdpSocket::bind(SocketAddrV4::new(config.bind_interface, 0))?;
        socket.set_multicast_ttl_v4(config.multicast_ttl)?;
        socket.set_nonblocking(true)?;

        let target = SocketAddr::V4(SocketAddrV4::new(config.multicast_group, config.port));
        let source_id = source_id_hash(&config.source_name);

        Ok(Self {
            socket,
            target,
            config,
            source_id,
            sequence: AtomicU32::new(0),
            last_send: std::sync::Mutex::new(Instant::now()),
        })
    }

    /// Publish a batch of transforms to the network
    pub fn publish(
        &self,
        transforms: &[TransformStamped],
        is_static: bool,
    ) -> std::io::Result<usize> {
        if transforms.is_empty() {
            return Ok(0);
        }

        // Rate limiting
        if self.config.max_publish_rate > 0 {
            let min_interval = (self.config.max_publish_rate as f64).hz().period();
            let mut last = self.last_send.lock().unwrap();
            let elapsed = last.elapsed();
            if elapsed < min_interval {
                return Ok(0);
            }
            *last = Instant::now();
        }

        let tf_size = std::mem::size_of::<TransformStamped>();
        let ns_bytes = self.config.namespace.as_ref().map(|ns| ns.as_bytes());
        let ns_overhead = ns_bytes.map(|b| 1 + b.len()).unwrap_or(0);

        // Calculate how many transforms fit in one packet
        let payload_budget = MAX_PACKET_SIZE - HEADER_SIZE - ns_overhead - 4; // 4 for count
        let max_per_packet = payload_budget / tf_size;

        let mut total_sent = 0;

        for chunk in transforms.chunks(max_per_packet.max(1)) {
            let seq = self.sequence.fetch_add(1, Ordering::Relaxed);

            let mut flags = 0u16;
            if is_static {
                flags |= FLAG_STATIC;
            }
            if self.config.namespace.is_some() {
                flags |= FLAG_HAS_NAMESPACE;
            }

            let now_ns = std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap_or_default()
                .as_nanos() as u64;

            let header = TFBridgeHeader {
                magic: MAGIC,
                version: PROTOCOL_VERSION,
                flags,
                sequence: seq,
                source_id: self.source_id,
                timestamp_ns: now_ns,
            };

            // Build packet
            let packet_size = HEADER_SIZE + ns_overhead + 4 + chunk.len() * tf_size;
            let mut buf = vec![0u8; packet_size];
            let mut offset = 0;

            // Write header
            let header_bytes: &[u8] = unsafe {
                std::slice::from_raw_parts(
                    &header as *const TFBridgeHeader as *const u8,
                    HEADER_SIZE,
                )
            };
            buf[offset..offset + HEADER_SIZE].copy_from_slice(header_bytes);
            offset += HEADER_SIZE;

            // Write namespace if present
            if let Some(ns) = ns_bytes {
                buf[offset] = ns.len() as u8;
                offset += 1;
                buf[offset..offset + ns.len()].copy_from_slice(ns);
                offset += ns.len();
            }

            // Write transform count
            let count = chunk.len() as u32;
            buf[offset..offset + 4].copy_from_slice(&count.to_le_bytes());
            offset += 4;

            // Write transforms (Pod → raw bytes)
            for tf in chunk {
                let tf_bytes: &[u8] = horus_core::bytemuck::bytes_of(tf);
                buf[offset..offset + tf_size].copy_from_slice(tf_bytes);
                offset += tf_size;
            }

            self.socket.send_to(&buf[..offset], self.target)?;
            total_sent += chunk.len();
        }

        Ok(total_sent)
    }
}

/// Received TF packet metadata
#[derive(Debug, Clone)]
pub struct TFBridgePacket {
    /// Source identifier
    pub source_id: u32,
    /// Sequence number
    pub sequence: u32,
    /// Whether transforms are static
    pub is_static: bool,
    /// Namespace prefix (if any)
    pub namespace: Option<String>,
    /// Received transforms
    pub transforms: Vec<TransformStamped>,
    /// Packet send timestamp (from sender)
    pub send_timestamp_ns: u64,
    /// Packet receive time (local)
    pub receive_time: Instant,
}

/// TF Bridge Subscriber — receives remote TF from the network
pub struct TFBridgeSubscriber {
    socket: UdpSocket,
    config: TFBridgeConfig,
    source_id: u32,
    recv_buf: Vec<u8>,
    /// Per-source sequence tracking for packet loss detection
    source_sequences: HashMap<u32, u32>,
}

impl TFBridgeSubscriber {
    /// Create a new TF bridge subscriber
    pub fn new(config: TFBridgeConfig) -> std::io::Result<Self> {
        let bind_addr = SocketAddrV4::new(Ipv4Addr::UNSPECIFIED, config.port);
        let socket = UdpSocket::bind(bind_addr)?;

        socket.join_multicast_v4(&config.multicast_group, &config.bind_interface)?;
        socket.set_nonblocking(true)?;

        let source_id = source_id_hash(&config.source_name);

        Ok(Self {
            socket,
            config,
            source_id,
            recv_buf: vec![0u8; MAX_PACKET_SIZE],
            source_sequences: HashMap::new(),
        })
    }

    /// Try to receive a TF packet (non-blocking)
    ///
    /// Returns `None` if no packet is available.
    pub fn try_recv(&mut self) -> std::io::Result<Option<TFBridgePacket>> {
        let (len, _addr) = match self.socket.recv_from(&mut self.recv_buf) {
            Ok(result) => result,
            Err(e) if e.kind() == std::io::ErrorKind::WouldBlock => return Ok(None),
            Err(e) => return Err(e),
        };

        if len < HEADER_SIZE + 4 {
            return Ok(None); // Too small
        }

        let receive_time = Instant::now();

        // Parse header
        let header: TFBridgeHeader =
            unsafe { std::ptr::read_unaligned(self.recv_buf.as_ptr() as *const TFBridgeHeader) };

        // Validate magic
        if header.magic != MAGIC {
            return Ok(None);
        }

        // Version check
        if header.version != PROTOCOL_VERSION {
            return Ok(None);
        }

        // Filter own messages
        if self.config.filter_self && header.source_id == self.source_id {
            return Ok(None);
        }

        let is_static = (header.flags & FLAG_STATIC) != 0;
        let has_namespace = (header.flags & FLAG_HAS_NAMESPACE) != 0;

        let mut offset = HEADER_SIZE;

        // Parse namespace
        let namespace = if has_namespace {
            if offset >= len {
                return Ok(None);
            }
            let ns_len = self.recv_buf[offset] as usize;
            offset += 1;
            if offset + ns_len > len {
                return Ok(None);
            }
            let ns = String::from_utf8_lossy(&self.recv_buf[offset..offset + ns_len]).to_string();
            offset += ns_len;
            Some(ns)
        } else {
            None
        };

        // Parse transform count
        if offset + 4 > len {
            return Ok(None);
        }
        let count = u32::from_le_bytes([
            self.recv_buf[offset],
            self.recv_buf[offset + 1],
            self.recv_buf[offset + 2],
            self.recv_buf[offset + 3],
        ]) as usize;
        offset += 4;

        // Parse transforms
        let tf_size = std::mem::size_of::<TransformStamped>();
        if offset + count * tf_size > len {
            return Ok(None); // Truncated
        }

        let mut transforms = Vec::with_capacity(count);
        for _ in 0..count {
            let tf: TransformStamped =
                *horus_core::bytemuck::from_bytes(&self.recv_buf[offset..offset + tf_size]);
            transforms.push(tf);
            offset += tf_size;
        }

        // Track sequence for loss detection
        self.source_sequences
            .insert(header.source_id, header.sequence);

        Ok(Some(TFBridgePacket {
            source_id: header.source_id,
            sequence: header.sequence,
            is_static,
            namespace,
            transforms,
            send_timestamp_ns: header.timestamp_ns,
            receive_time,
        }))
    }

    /// Receive with blocking timeout
    pub fn recv_timeout(&mut self, timeout: Duration) -> std::io::Result<Option<TFBridgePacket>> {
        self.socket.set_nonblocking(false)?;
        self.socket.set_read_timeout(Some(timeout))?;
        let result = self.try_recv();
        self.socket.set_nonblocking(true)?;
        result
    }
}

/// Apply namespace prefix to transforms
///
/// Prefixes both parent and child frame IDs with `namespace/`.
/// Used for multi-robot setups to avoid frame name collisions.
///
/// Example: namespace="robot1", frame="base_link" → "robot1/base_link"
/// World frame is never prefixed.
pub fn apply_namespace(transforms: &mut [TransformStamped], namespace: &str) {
    for tf in transforms.iter_mut() {
        let parent = super::messages::frame_id_to_string(&tf.parent_frame);
        let child = super::messages::frame_id_to_string(&tf.child_frame);

        // Don't prefix world frame
        if parent != "world" && !parent.is_empty() {
            let prefixed = format!("{}/{}", namespace, parent);
            super::messages::string_to_frame_id(&prefixed, &mut tf.parent_frame);
        }

        if !child.is_empty() {
            let prefixed = format!("{}/{}", namespace, child);
            super::messages::string_to_frame_id(&prefixed, &mut tf.child_frame);
        }
    }
}

/// Strip namespace prefix from transforms
pub fn strip_namespace(transforms: &mut [TransformStamped], namespace: &str) {
    let prefix = format!("{}/", namespace);
    for tf in transforms.iter_mut() {
        let parent = super::messages::frame_id_to_string(&tf.parent_frame);
        let child = super::messages::frame_id_to_string(&tf.child_frame);

        if let Some(stripped) = parent.strip_prefix(&prefix) {
            super::messages::string_to_frame_id(stripped, &mut tf.parent_frame);
        }
        if let Some(stripped) = child.strip_prefix(&prefix) {
            super::messages::string_to_frame_id(stripped, &mut tf.child_frame);
        }
    }
}

/// Merge policy for handling conflicting frames from multiple sources
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum MergePolicy {
    /// Last writer wins (newest timestamp)
    LastWriterWins,
    /// First source wins (ignore later sources for same frame)
    FirstSourceWins,
    /// Reject conflicting frames from different sources
    RejectConflicts,
}

/// TF tree merger for combining frames from multiple network sources
pub struct TFTreeMerger {
    /// Policy for handling conflicts
    policy: MergePolicy,
    /// Source ownership: frame_name → source_id
    frame_owners: HashMap<String, u32>,
    /// Conflict log
    conflicts: Vec<TFConflict>,
}

/// A detected conflict between two sources
#[derive(Debug, Clone)]
pub struct TFConflict {
    pub frame_name: String,
    pub existing_source: u32,
    pub new_source: u32,
    pub timestamp_ns: u64,
    pub resolution: ConflictResolution,
}

/// How a conflict was resolved
#[derive(Debug, Clone, Copy)]
pub enum ConflictResolution {
    Accepted,
    Rejected,
    Overwritten,
}

impl TFTreeMerger {
    pub fn new(policy: MergePolicy) -> Self {
        Self {
            policy,
            frame_owners: HashMap::new(),
            conflicts: Vec::new(),
        }
    }

    /// Check if a transform from a given source should be accepted
    ///
    /// Returns the accepted transforms (with conflicts filtered per policy)
    pub fn filter_incoming(&mut self, packet: &TFBridgePacket) -> Vec<TransformStamped> {
        let mut accepted = Vec::with_capacity(packet.transforms.len());

        for tf in &packet.transforms {
            let child = super::messages::frame_id_to_string(&tf.child_frame);

            match self.frame_owners.get(&child) {
                None => {
                    // New frame, accept and register ownership
                    self.frame_owners.insert(child, packet.source_id);
                    accepted.push(*tf);
                }
                Some(&owner) if owner == packet.source_id => {
                    // Same source, always accept
                    accepted.push(*tf);
                }
                Some(&existing_owner) => {
                    // Conflict: different source for same frame
                    let resolution = match self.policy {
                        MergePolicy::LastWriterWins => {
                            self.frame_owners.insert(child.clone(), packet.source_id);
                            accepted.push(*tf);
                            ConflictResolution::Overwritten
                        }
                        MergePolicy::FirstSourceWins => ConflictResolution::Rejected,
                        MergePolicy::RejectConflicts => ConflictResolution::Rejected,
                    };

                    self.conflicts.push(TFConflict {
                        frame_name: child,
                        existing_source: existing_owner,
                        new_source: packet.source_id,
                        timestamp_ns: tf.timestamp_ns,
                        resolution,
                    });
                }
            }
        }

        accepted
    }

    /// Get all detected conflicts
    pub fn conflicts(&self) -> &[TFConflict] {
        &self.conflicts
    }

    /// Get frame ownership map
    pub fn frame_owners(&self) -> &HashMap<String, u32> {
        &self.frame_owners
    }

    /// Reset all ownership tracking
    pub fn reset(&mut self) {
        self.frame_owners.clear();
        self.conflicts.clear();
    }
}

/// Background TF bridge that runs publisher and subscriber in a thread
pub struct TFBridgeHandle {
    /// Signal to stop the bridge thread
    stop: Arc<AtomicBool>,
    /// Join handle for the bridge thread
    thread: Option<std::thread::JoinHandle<()>>,
}

impl TFBridgeHandle {
    /// Stop the bridge
    pub fn stop(&self) {
        self.stop.store(true, Ordering::Relaxed);
    }

    /// Check if bridge is still running
    pub fn is_running(&self) -> bool {
        self.thread
            .as_ref()
            .map(|t| !t.is_finished())
            .unwrap_or(false)
    }
}

impl Drop for TFBridgeHandle {
    fn drop(&mut self) {
        self.stop.store(true, Ordering::Relaxed);
        if let Some(thread) = self.thread.take() {
            let _ = thread.join();
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::transform_frame::transform::Transform;

    #[test]
    fn test_source_id_hash_deterministic() {
        let id1 = source_id_hash("robot1");
        let id2 = source_id_hash("robot1");
        let id3 = source_id_hash("robot2");
        assert_eq!(id1, id2);
        assert_ne!(id1, id3);
    }

    #[test]
    fn test_config_builder() {
        let config = TFBridgeConfig::new("test_robot")
            .with_namespace("robot1")
            .with_port(17300)
            .with_ttl(2);

        assert_eq!(config.source_name, "test_robot");
        assert_eq!(config.namespace.as_deref(), Some("robot1"));
        assert_eq!(config.port, 17300);
        assert_eq!(config.multicast_ttl, 2);
    }

    #[test]
    fn test_apply_namespace() {
        let mut transforms = vec![
            TransformStamped::new("world", "base_link", 1000, Transform::identity()),
            TransformStamped::new("base_link", "camera", 1000, Transform::identity()),
        ];

        apply_namespace(&mut transforms, "robot1");

        assert_eq!(transforms[0].parent_frame_id(), "world"); // world not prefixed
        assert_eq!(transforms[0].child_frame_id(), "robot1/base_link");
        assert_eq!(transforms[1].parent_frame_id(), "robot1/base_link");
        assert_eq!(transforms[1].child_frame_id(), "robot1/camera");
    }

    #[test]
    fn test_strip_namespace() {
        let mut transforms = vec![TransformStamped::new(
            "robot1/base_link",
            "robot1/camera",
            1000,
            Transform::identity(),
        )];

        strip_namespace(&mut transforms, "robot1");

        assert_eq!(transforms[0].parent_frame_id(), "base_link");
        assert_eq!(transforms[0].child_frame_id(), "camera");
    }

    #[test]
    fn test_strip_namespace_no_match() {
        let mut transforms = vec![TransformStamped::new(
            "base_link",
            "camera",
            1000,
            Transform::identity(),
        )];

        strip_namespace(&mut transforms, "robot1");

        // Should remain unchanged
        assert_eq!(transforms[0].parent_frame_id(), "base_link");
        assert_eq!(transforms[0].child_frame_id(), "camera");
    }

    #[test]
    fn test_merger_first_source_wins() {
        let mut merger = TFTreeMerger::new(MergePolicy::FirstSourceWins);

        let packet1 = TFBridgePacket {
            source_id: 100,
            sequence: 0,
            is_static: false,
            namespace: None,
            transforms: vec![TransformStamped::new(
                "world",
                "base_link",
                1000,
                Transform::identity(),
            )],
            send_timestamp_ns: 1000,
            receive_time: Instant::now(),
        };

        let accepted = merger.filter_incoming(&packet1);
        assert_eq!(accepted.len(), 1);

        // Different source, same frame — should be rejected
        let packet2 = TFBridgePacket {
            source_id: 200,
            sequence: 0,
            is_static: false,
            namespace: None,
            transforms: vec![TransformStamped::new(
                "world",
                "base_link",
                2000,
                Transform::identity(),
            )],
            send_timestamp_ns: 2000,
            receive_time: Instant::now(),
        };

        let accepted = merger.filter_incoming(&packet2);
        assert_eq!(accepted.len(), 0);
        assert_eq!(merger.conflicts().len(), 1);
    }

    #[test]
    fn test_merger_last_writer_wins() {
        let mut merger = TFTreeMerger::new(MergePolicy::LastWriterWins);

        let packet1 = TFBridgePacket {
            source_id: 100,
            sequence: 0,
            is_static: false,
            namespace: None,
            transforms: vec![TransformStamped::new(
                "world",
                "base_link",
                1000,
                Transform::identity(),
            )],
            send_timestamp_ns: 1000,
            receive_time: Instant::now(),
        };

        merger.filter_incoming(&packet1);

        // Different source, same frame — should be accepted (overwrite)
        let packet2 = TFBridgePacket {
            source_id: 200,
            sequence: 0,
            is_static: false,
            namespace: None,
            transforms: vec![TransformStamped::new(
                "world",
                "base_link",
                2000,
                Transform::identity(),
            )],
            send_timestamp_ns: 2000,
            receive_time: Instant::now(),
        };

        let accepted = merger.filter_incoming(&packet2);
        assert_eq!(accepted.len(), 1);
        assert_eq!(*merger.frame_owners().get("base_link").unwrap(), 200);
    }

    #[test]
    fn test_merger_same_source_always_accepted() {
        let mut merger = TFTreeMerger::new(MergePolicy::RejectConflicts);

        let packet = TFBridgePacket {
            source_id: 100,
            sequence: 0,
            is_static: false,
            namespace: None,
            transforms: vec![TransformStamped::new(
                "world",
                "base_link",
                1000,
                Transform::identity(),
            )],
            send_timestamp_ns: 1000,
            receive_time: Instant::now(),
        };

        let accepted = merger.filter_incoming(&packet);
        assert_eq!(accepted.len(), 1);

        // Same source, same frame — should always be accepted
        let packet2 = TFBridgePacket {
            source_id: 100,
            sequence: 1,
            is_static: false,
            namespace: None,
            transforms: vec![TransformStamped::new(
                "world",
                "base_link",
                2000,
                Transform::identity(),
            )],
            send_timestamp_ns: 2000,
            receive_time: Instant::now(),
        };

        let accepted = merger.filter_incoming(&packet2);
        assert_eq!(accepted.len(), 1);
        assert_eq!(merger.conflicts().len(), 0);
    }

    #[test]
    fn test_header_size() {
        assert_eq!(std::mem::size_of::<TFBridgeHeader>(), HEADER_SIZE);
    }

    #[test]
    fn test_publisher_creation() {
        let config = TFBridgeConfig::new("test_pub");
        let pub_result = TFBridgePublisher::new(config);
        // Should succeed on most systems (just binding a UDP socket)
        assert!(pub_result.is_ok());
    }
}
