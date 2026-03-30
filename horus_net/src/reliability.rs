//! Reliability tiers — fire-and-forget, redundant, latched+ACK.
//!
//! Tier 0 (fire-and-forget): send once. Default for Normal/Bulk.
//! Tier 1 (redundant): send N copies staggered. For RealTime (2x), Immediate (3x).
//! Tier 2 (latched+ACK): resend every 10ms until ACK. For Immediate only.
//!
//! See blueprint section 8.

use std::collections::{HashMap, HashSet};
use std::time::{Duration, Instant};

use crate::priority::{Priority, Reliability};
use crate::wire::{AckPayload, OutMessage};

/// Maximum resend attempts for latched messages before giving up.
const MAX_LATCH_RESENDS: u32 = 100;

/// Resend interval for latched messages.
const LATCH_RESEND_INTERVAL: Duration = Duration::from_millis(10);

/// A latched message waiting for ACK.
#[derive(Debug)]
struct LatchedEntry {
    topic_hash: u32,
    sequence: u32,
    payload: Vec<u8>,
    resend_count: u32,
    last_sent: Instant,
}

/// Reliability layer — manages redundant sends, latched messages, and deduplication.
pub struct ReliabilityLayer {
    /// Latched messages awaiting ACK. Keyed by (topic_hash, sequence).
    latched: HashMap<(u32, u32), LatchedEntry>,
    /// Dedup state per (peer_hash, topic_hash) → last seen sequence.
    dedup: HashMap<(u16, u32), u32>,
}

impl ReliabilityLayer {
    pub fn new() -> Self {
        Self {
            latched: HashMap::new(),
            dedup: HashMap::new(),
        }
    }

    /// Number of redundant copies to send for a given reliability tier.
    pub fn copies_for(reliability: Reliability) -> u8 {
        match reliability {
            Reliability::Latched => 3,    // Immediate: 3x + ACK
            Reliability::Redundant => 2,  // RealTime: 2x
            Reliability::None => 1,       // Normal/Bulk: 1x
        }
    }

    /// Get the default reliability tier for a priority level.
    pub fn default_reliability(priority: Priority) -> Reliability {
        Reliability::default_for(priority)
    }

    /// Start tracking a latched message (resend until ACK).
    pub fn start_latch(&mut self, topic_hash: u32, sequence: u32, payload: Vec<u8>) {
        let key = (topic_hash, sequence);
        self.latched.insert(key, LatchedEntry {
            topic_hash,
            sequence,
            payload,
            resend_count: 0,
            last_sent: Instant::now(),
        });
    }

    /// Handle an ACK — remove the latched message.
    pub fn on_ack(&mut self, ack: &AckPayload) {
        self.latched.remove(&(ack.acked_topic_hash, ack.acked_sequence));
    }

    /// Get messages that need resending (latched, interval elapsed).
    /// Returns (topic_hash, sequence, payload) for each.
    /// Removes entries that exceeded MAX_LATCH_RESENDS.
    pub fn tick_resends(&mut self) -> Vec<(u32, u32, Vec<u8>)> {
        let now = Instant::now();
        let mut to_resend = Vec::new();
        let mut to_remove = Vec::new();

        for (key, entry) in &mut self.latched {
            if now.duration_since(entry.last_sent) >= LATCH_RESEND_INTERVAL {
                if entry.resend_count >= MAX_LATCH_RESENDS {
                    eprintln!(
                        "[horus_net] Latched message on topic_hash={:#X} seq={} gave up after {} resends",
                        entry.topic_hash, entry.sequence, entry.resend_count
                    );
                    to_remove.push(*key);
                } else {
                    entry.resend_count += 1;
                    entry.last_sent = now;
                    to_resend.push((entry.topic_hash, entry.sequence, entry.payload.clone()));
                }
            }
        }

        for key in to_remove {
            self.latched.remove(&key);
        }

        to_resend
    }

    /// Deduplicate incoming messages. Returns true if this message is new (not a duplicate).
    pub fn is_new_message(&mut self, sender_hash: u16, topic_hash: u32, sequence: u32) -> bool {
        let key = (sender_hash, topic_hash);
        if let Some(&last_seq) = self.dedup.get(&key) {
            // Handle wrapping: accept if sequence > last_seq (with wrapping tolerance)
            if sequence <= last_seq && last_seq.wrapping_sub(sequence) < u32::MAX / 2 {
                return false; // Duplicate or old
            }
        }
        self.dedup.insert(key, sequence);
        true
    }

    /// Filter a list of incoming messages to remove duplicates in-place.
    pub fn dedup_messages(
        &mut self,
        sender_hash: u16,
        messages: &mut Vec<crate::wire::InMessage>,
    ) {
        messages.retain(|msg| self.is_new_message(sender_hash, msg.topic_hash, msg.sequence));
    }

    /// Number of latched messages awaiting ACK.
    pub fn pending_latches(&self) -> usize {
        self.latched.len()
    }
}

impl Default for ReliabilityLayer {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::priority::Encoding;

    #[test]
    fn copies_for_tiers() {
        assert_eq!(ReliabilityLayer::copies_for(Reliability::None), 1);
        assert_eq!(ReliabilityLayer::copies_for(Reliability::Redundant), 2);
        assert_eq!(ReliabilityLayer::copies_for(Reliability::Latched), 3);
    }

    #[test]
    fn default_reliability_matches_blueprint() {
        assert_eq!(ReliabilityLayer::default_reliability(Priority::Immediate), Reliability::Latched);
        assert_eq!(ReliabilityLayer::default_reliability(Priority::RealTime), Reliability::Redundant);
        assert_eq!(ReliabilityLayer::default_reliability(Priority::Normal), Reliability::None);
        assert_eq!(ReliabilityLayer::default_reliability(Priority::Bulk), Reliability::None);
    }

    #[test]
    fn latch_and_ack() {
        let mut layer = ReliabilityLayer::new();
        layer.start_latch(100, 1, vec![42]);
        assert_eq!(layer.pending_latches(), 1);

        layer.on_ack(&AckPayload {
            acked_topic_hash: 100,
            acked_sequence: 1,
        });
        assert_eq!(layer.pending_latches(), 0);
    }

    #[test]
    fn latch_resends_after_interval() {
        let mut layer = ReliabilityLayer::new();
        layer.start_latch(100, 1, vec![42]);

        // Immediately: no resend (interval not elapsed)
        let resends = layer.tick_resends();
        assert!(resends.is_empty());

        // Wait for resend interval
        std::thread::sleep(Duration::from_millis(15));
        let resends = layer.tick_resends();
        assert_eq!(resends.len(), 1);
        assert_eq!(resends[0].0, 100); // topic_hash
        assert_eq!(resends[0].1, 1);   // sequence
        assert_eq!(resends[0].2, vec![42]); // payload
    }

    #[test]
    fn latch_gives_up_after_max_resends() {
        let mut layer = ReliabilityLayer::new();
        layer.start_latch(100, 1, vec![42]);

        // Simulate many resends
        for _ in 0..MAX_LATCH_RESENDS + 5 {
            // Force entry past interval
            if let Some(entry) = layer.latched.get_mut(&(100, 1)) {
                entry.last_sent = Instant::now() - LATCH_RESEND_INTERVAL * 2;
                entry.resend_count = MAX_LATCH_RESENDS;
            }
            layer.tick_resends();
        }

        // Should be removed after exceeding max
        assert_eq!(layer.pending_latches(), 0);
    }

    #[test]
    fn dedup_filters_duplicates() {
        let mut layer = ReliabilityLayer::new();

        // First message: new
        assert!(layer.is_new_message(0x1234, 100, 1));
        // Same message: duplicate
        assert!(!layer.is_new_message(0x1234, 100, 1));
        // Next sequence: new
        assert!(layer.is_new_message(0x1234, 100, 2));
        // Old sequence: duplicate
        assert!(!layer.is_new_message(0x1234, 100, 1));
    }

    #[test]
    fn dedup_per_peer_per_topic() {
        let mut layer = ReliabilityLayer::new();

        // Same sequence but different peer → both new
        assert!(layer.is_new_message(0x1111, 100, 1));
        assert!(layer.is_new_message(0x2222, 100, 1));

        // Same peer but different topic → both new
        assert!(layer.is_new_message(0x1111, 200, 1));
    }

    #[test]
    fn dedup_messages_filters_vec() {
        let mut layer = ReliabilityLayer::new();

        // Simulate receiving 3 copies of same message (redundant send)
        let mut messages = vec![
            crate::wire::InMessage {
                topic_hash: 100,
                payload: vec![1],
                timestamp_ns: 0,
                sequence: 42,
                priority: Priority::RealTime,
                reliability: Reliability::Redundant,
                encoding: Encoding::PodLe,
            },
            crate::wire::InMessage {
                topic_hash: 100,
                payload: vec![1],
                timestamp_ns: 0,
                sequence: 42, // Same sequence = duplicate
                priority: Priority::RealTime,
                reliability: Reliability::Redundant,
                encoding: Encoding::PodLe,
            },
            crate::wire::InMessage {
                topic_hash: 100,
                payload: vec![2],
                timestamp_ns: 0,
                sequence: 43, // Different sequence = new
                priority: Priority::RealTime,
                reliability: Reliability::Redundant,
                encoding: Encoding::PodLe,
            },
        ];

        layer.dedup_messages(0x1234, &mut messages);
        assert_eq!(messages.len(), 2); // 42 + 43, duplicate 42 removed
        assert_eq!(messages[0].sequence, 42);
        assert_eq!(messages[1].sequence, 43);
    }
}
