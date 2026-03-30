//! Fragmentation + reassembly — messages up to 1MB over UDP.
//!
//! Messages > MAX_FRAGMENT_PAYLOAD (1400B) are split into fragments.
//! Messages > MAX_REASSEMBLY_SIZE (1MB) are rejected with a warning.
//! Stale fragments older than FRAGMENT_TIMEOUT (100ms) are discarded.
//!
//! See blueprint section 11.

use std::collections::HashMap;
use std::time::{Duration, Instant};

use crate::priority::{Encoding, Priority, Reliability};
use crate::wire::OutMessage;

/// Maximum payload per fragment (leave room for UDP/IP headers + our headers).
pub const MAX_FRAGMENT_PAYLOAD: usize = 1400;

/// Maximum total message size for reassembly.
pub const MAX_REASSEMBLY_SIZE: usize = 1_048_576; // 1MB

/// Timeout for discarding incomplete fragment sets.
pub const FRAGMENT_TIMEOUT: Duration = Duration::from_millis(100);

/// A single fragment ready to be sent.
#[derive(Debug, Clone)]
pub struct Fragment {
    pub topic_hash: u32,
    pub sequence: u32,
    pub timestamp_ns: u64,
    pub priority: Priority,
    pub reliability: Reliability,
    pub encoding: Encoding,
    pub fragment_id: u32,
    pub fragment_index: u16,
    pub fragment_count: u16,
    pub total_payload_len: u32,
    pub payload: Vec<u8>,
}

/// Splits large messages into MTU-sized fragments.
pub struct Fragmenter {
    next_fragment_id: u32,
}

impl Fragmenter {
    pub fn new() -> Self {
        Self {
            next_fragment_id: 0,
        }
    }

    /// Fragment a message. Returns a single-element vec if no fragmentation needed.
    /// Returns empty vec if message exceeds MAX_REASSEMBLY_SIZE.
    pub fn fragment(&mut self, msg: &OutMessage) -> Vec<Fragment> {
        if msg.payload.len() <= MAX_FRAGMENT_PAYLOAD {
            // No fragmentation needed — return as single fragment
            return vec![Fragment {
                topic_hash: msg.topic_hash,
                sequence: msg.sequence,
                timestamp_ns: msg.timestamp_ns,
                priority: msg.priority,
                reliability: msg.reliability,
                encoding: msg.encoding,
                fragment_id: 0,
                fragment_index: 0,
                fragment_count: 1,
                total_payload_len: msg.payload.len() as u32,
                payload: msg.payload.clone(),
            }];
        }

        if msg.payload.len() > MAX_REASSEMBLY_SIZE {
            eprintln!(
                "[horus_net] Message on '{}' is {}B (max {}B for horus_net). \
                 Use horus-zenoh for large data.",
                msg.topic_name,
                msg.payload.len(),
                MAX_REASSEMBLY_SIZE
            );
            return vec![];
        }

        let id = self.next_fragment_id;
        self.next_fragment_id = self.next_fragment_id.wrapping_add(1);

        let chunks: Vec<&[u8]> = msg.payload.chunks(MAX_FRAGMENT_PAYLOAD).collect();
        let count = chunks.len() as u16;

        chunks
            .iter()
            .enumerate()
            .map(|(i, chunk)| Fragment {
                topic_hash: msg.topic_hash,
                sequence: msg.sequence,
                timestamp_ns: msg.timestamp_ns,
                priority: msg.priority,
                reliability: msg.reliability,
                encoding: msg.encoding,
                fragment_id: id,
                fragment_index: i as u16,
                fragment_count: count,
                total_payload_len: msg.payload.len() as u32,
                payload: chunk.to_vec(),
            })
            .collect()
    }
}

impl Default for Fragmenter {
    fn default() -> Self {
        Self::new()
    }
}

/// A partially-received fragmented message.
struct PendingMessage {
    fragments: Vec<Option<Vec<u8>>>,
    received_count: u16,
    expected_count: u16,
    total_len: u32,
    first_seen: Instant,
    // Metadata from first fragment
    topic_hash: u32,
    sequence: u32,
    timestamp_ns: u64,
    priority: Priority,
    reliability: Reliability,
    encoding: Encoding,
}

/// A fully reassembled message.
#[derive(Debug, Clone)]
pub struct ReassembledMessage {
    pub topic_hash: u32,
    pub sequence: u32,
    pub timestamp_ns: u64,
    pub priority: Priority,
    pub reliability: Reliability,
    pub encoding: Encoding,
    pub payload: Vec<u8>,
}

/// Reassembles fragments into complete messages.
pub struct Reassembler {
    /// Pending messages keyed by (topic_hash, fragment_id).
    pending: HashMap<(u32, u32), PendingMessage>,
}

impl Reassembler {
    pub fn new() -> Self {
        Self {
            pending: HashMap::new(),
        }
    }

    /// Feed a fragment. Returns the reassembled message if all fragments arrived.
    pub fn feed(&mut self, frag: Fragment) -> Option<ReassembledMessage> {
        if frag.total_payload_len as usize > MAX_REASSEMBLY_SIZE {
            return None;
        }

        // Single-fragment message (no fragmentation was needed)
        if frag.fragment_count == 1 {
            return Some(ReassembledMessage {
                topic_hash: frag.topic_hash,
                sequence: frag.sequence,
                timestamp_ns: frag.timestamp_ns,
                priority: frag.priority,
                reliability: frag.reliability,
                encoding: frag.encoding,
                payload: frag.payload,
            });
        }

        let key = (frag.topic_hash, frag.fragment_id);

        let pending = self.pending.entry(key).or_insert_with(|| PendingMessage {
            fragments: vec![None; frag.fragment_count as usize],
            received_count: 0,
            expected_count: frag.fragment_count,
            total_len: frag.total_payload_len,
            first_seen: Instant::now(),
            topic_hash: frag.topic_hash,
            sequence: frag.sequence,
            timestamp_ns: frag.timestamp_ns,
            priority: frag.priority,
            reliability: frag.reliability,
            encoding: frag.encoding,
        });

        let idx = frag.fragment_index as usize;
        if idx >= pending.fragments.len() {
            return None; // Invalid index
        }

        if pending.fragments[idx].is_none() {
            pending.fragments[idx] = Some(frag.payload);
            pending.received_count += 1;
        }

        if pending.received_count == pending.expected_count {
            // All fragments received — concatenate
            let pending = self.pending.remove(&key).unwrap();
            let mut payload = Vec::with_capacity(pending.total_len as usize);
            for data in pending.fragments.into_iter().flatten() {
                payload.extend_from_slice(&data);
            }
            Some(ReassembledMessage {
                topic_hash: pending.topic_hash,
                sequence: pending.sequence,
                timestamp_ns: pending.timestamp_ns,
                priority: pending.priority,
                reliability: pending.reliability,
                encoding: pending.encoding,
                payload,
            })
        } else {
            None
        }
    }

    /// Garbage collect stale incomplete fragment sets.
    /// Returns number of entries discarded.
    pub fn gc_stale(&mut self) -> usize {
        let before = self.pending.len();
        self.pending
            .retain(|_, p| p.first_seen.elapsed() < FRAGMENT_TIMEOUT);
        before - self.pending.len()
    }

    /// Number of pending (incomplete) messages.
    pub fn pending_count(&self) -> usize {
        self.pending.len()
    }
}

impl Default for Reassembler {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::wire::topic_hash;

    fn make_msg(name: &str, payload_size: usize) -> OutMessage {
        OutMessage {
            topic_name: name.into(),
            topic_hash: topic_hash(name),
            payload: vec![0xAA; payload_size],
            timestamp_ns: 12345,
            sequence: 1,
            priority: Priority::Normal,
            reliability: Reliability::None,
            encoding: Encoding::PodLe,
        }
    }

    #[test]
    fn no_fragmentation_for_small_messages() {
        let mut frag = Fragmenter::new();
        let msg = make_msg("imu", 64);
        let fragments = frag.fragment(&msg);
        assert_eq!(fragments.len(), 1);
        assert_eq!(fragments[0].fragment_count, 1);
        assert_eq!(fragments[0].payload.len(), 64);
    }

    #[test]
    fn fragments_large_message() {
        let mut frag = Fragmenter::new();
        let msg = make_msg("pointcloud", 5000); // 5KB → 4 fragments
        let fragments = frag.fragment(&msg);
        assert_eq!(fragments.len(), 4); // ceil(5000/1400) = 4
        assert_eq!(fragments[0].fragment_count, 4);
        assert_eq!(fragments[0].fragment_index, 0);
        assert_eq!(fragments[3].fragment_index, 3);
        assert_eq!(fragments[0].total_payload_len, 5000);

        // Total payload across fragments = original
        let total: usize = fragments.iter().map(|f| f.payload.len()).sum();
        assert_eq!(total, 5000);
    }

    #[test]
    fn rejects_over_1mb() {
        let mut frag = Fragmenter::new();
        let msg = make_msg("huge", 2_000_000); // 2MB
        let fragments = frag.fragment(&msg);
        assert!(fragments.is_empty());
    }

    #[test]
    fn exactly_1mb_is_accepted() {
        let mut frag = Fragmenter::new();
        let msg = make_msg("big", MAX_REASSEMBLY_SIZE);
        let fragments = frag.fragment(&msg);
        assert!(!fragments.is_empty());
        let total: usize = fragments.iter().map(|f| f.payload.len()).sum();
        assert_eq!(total, MAX_REASSEMBLY_SIZE);
    }

    #[test]
    fn reassemble_single_fragment() {
        let mut reasm = Reassembler::new();
        let frag = Fragment {
            topic_hash: 100,
            sequence: 1,
            timestamp_ns: 999,
            priority: Priority::Normal,
            reliability: Reliability::None,
            encoding: Encoding::PodLe,
            fragment_id: 0,
            fragment_index: 0,
            fragment_count: 1,
            total_payload_len: 8,
            payload: vec![1, 2, 3, 4, 5, 6, 7, 8],
        };
        let msg = reasm.feed(frag).unwrap();
        assert_eq!(msg.payload, vec![1, 2, 3, 4, 5, 6, 7, 8]);
    }

    #[test]
    fn reassemble_multiple_fragments() {
        let mut fragmenter = Fragmenter::new();
        let original = make_msg("data", 4000);
        let fragments = fragmenter.fragment(&original);
        assert!(fragments.len() > 1);

        let mut reasm = Reassembler::new();
        let mut result = None;
        for frag in fragments {
            if let Some(msg) = reasm.feed(frag) {
                result = Some(msg);
            }
        }
        let msg = result.unwrap();
        assert_eq!(msg.payload.len(), 4000);
        assert_eq!(msg.payload, vec![0xAA; 4000]);
    }

    #[test]
    fn reassemble_out_of_order() {
        let mut fragmenter = Fragmenter::new();
        let original = make_msg("data", 3000);
        let mut fragments = fragmenter.fragment(&original);
        assert!(fragments.len() > 1);

        // Reverse order
        fragments.reverse();

        let mut reasm = Reassembler::new();
        let mut result = None;
        for frag in fragments {
            if let Some(msg) = reasm.feed(frag) {
                result = Some(msg);
            }
        }
        let msg = result.unwrap();
        assert_eq!(msg.payload.len(), 3000);
        assert_eq!(msg.payload, vec![0xAA; 3000]);
    }

    #[test]
    fn incomplete_fragments_pending() {
        let mut fragmenter = Fragmenter::new();
        let original = make_msg("data", 5000);
        let fragments = fragmenter.fragment(&original);

        let mut reasm = Reassembler::new();
        // Feed only first 2 of 4 fragments
        reasm.feed(fragments[0].clone());
        reasm.feed(fragments[1].clone());
        assert_eq!(reasm.pending_count(), 1);
        assert!(reasm.feed(fragments[0].clone()).is_none()); // Duplicate index
    }

    #[test]
    fn stale_fragments_gc() {
        let mut reasm = Reassembler::new();
        // Feed partial fragment set
        let frag = Fragment {
            topic_hash: 100,
            sequence: 1,
            timestamp_ns: 0,
            priority: Priority::Normal,
            reliability: Reliability::None,
            encoding: Encoding::PodLe,
            fragment_id: 42,
            fragment_index: 0,
            fragment_count: 5,
            total_payload_len: 7000,
            payload: vec![0; 1400],
        };
        reasm.feed(frag);
        assert_eq!(reasm.pending_count(), 1);

        // Wait for timeout
        std::thread::sleep(Duration::from_millis(150));
        let discarded = reasm.gc_stale();
        assert_eq!(discarded, 1);
        assert_eq!(reasm.pending_count(), 0);
    }

    #[test]
    fn fragment_id_increments() {
        let mut frag = Fragmenter::new();
        let msg1 = make_msg("a", 3000);
        let msg2 = make_msg("b", 3000);
        let f1 = frag.fragment(&msg1);
        let f2 = frag.fragment(&msg2);
        assert_ne!(f1[0].fragment_id, f2[0].fragment_id);
    }
}
