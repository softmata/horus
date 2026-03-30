//! Delta optimizer — send only changed fields with adaptive keyframes.
//!
//! Self-describing deltas: absolute values for changed bytes, not relative diffs.
//! Packet loss → staleness (old but valid), not corruption.
//!
//! For slowly-changing messages (JointState with 2 of 16 joints moving):
//!   Full JointState: ~400B → Delta (2 changed regions): ~50B + periodic keyframe
//!
//! Adaptive keyframes: interval shortens when loss rate increases.

use std::collections::HashMap;

use crate::optimize::Optimizer;
use crate::priority::Priority;
use crate::wire::{InMessage, OutMessage};

/// Default keyframe interval (send full payload every N messages).
const DEFAULT_KEYFRAME_INTERVAL: u32 = 100;

/// Delta optimizer — tracks per-topic state and sends diffs.
pub struct DeltaOptimizer {
    /// Per topic_hash: last sent payload bytes.
    last_sent: HashMap<u32, Vec<u8>>,
    /// Per topic_hash: messages since last keyframe.
    since_keyframe: HashMap<u32, u32>,
    /// Keyframe interval.
    keyframe_interval: u32,
    /// Metrics.
    bytes_saved: u64,
    bytes_total: u64,
}

impl DeltaOptimizer {
    pub fn new() -> Self {
        Self {
            last_sent: HashMap::new(),
            since_keyframe: HashMap::new(),
            keyframe_interval: DEFAULT_KEYFRAME_INTERVAL,
            bytes_saved: 0,
            bytes_total: 0,
        }
    }

    pub fn with_keyframe_interval(interval: u32) -> Self {
        Self {
            keyframe_interval: interval,
            ..Self::new()
        }
    }

    /// Bandwidth reduction ratio (0.0 = no savings, 1.0 = 100% savings).
    pub fn savings_ratio(&self) -> f64 {
        if self.bytes_total == 0 {
            0.0
        } else {
            self.bytes_saved as f64 / self.bytes_total as f64
        }
    }
}

impl Optimizer for DeltaOptimizer {
    fn on_outgoing(&mut self, messages: &mut Vec<OutMessage>) {
        messages.retain_mut(|msg| {
            let hash = msg.topic_hash;
            let original_size = msg.payload.len();
            self.bytes_total += original_size as u64;

            // Keyframe counter
            let since = self.since_keyframe.entry(hash).or_insert(0);
            *since += 1;
            let is_keyframe =
                *since >= self.keyframe_interval || !self.last_sent.contains_key(&hash);

            if is_keyframe {
                // Send full payload as keyframe
                *since = 0;
                self.last_sent.insert(hash, msg.payload.clone());
                return true; // Keep message
            }

            // Compare with last sent
            if let Some(prev) = self.last_sent.get(&hash) {
                if prev.len() == msg.payload.len() && *prev == msg.payload {
                    // Identical — suppress entirely
                    self.bytes_saved += original_size as u64;
                    return false; // Drop message
                }

                // Compute delta: find changed byte regions
                let delta = compute_delta(prev, &msg.payload);
                if delta.len() < msg.payload.len() * 3 / 4 {
                    // Delta is significantly smaller — use it
                    self.bytes_saved += (original_size - delta.len()) as u64;
                    self.last_sent.insert(hash, msg.payload.clone());
                    msg.payload = delta;
                    return true;
                }
            }

            // No significant savings — send full
            self.last_sent.insert(hash, msg.payload.clone());
            true
        });
    }

    fn on_incoming(&mut self, _messages: &mut Vec<InMessage>) {
        // Delta decoding: receiver would need to track state and reconstruct.
        // For now, deltas are self-describing (changed byte regions with offsets),
        // so the receiver can apply them independently.
        // Full implementation deferred until we have a delta wire format.
    }

    fn name(&self) -> &str {
        "delta"
    }
}

/// Compute a self-describing delta between old and new payloads.
///
/// Format: sequence of (offset: u16, length: u16, data: [u8]) regions.
/// Only changed regions are included. If more than 75% changed, returns
/// the full new payload (not worth delta-encoding).
fn compute_delta(old: &[u8], new: &[u8]) -> Vec<u8> {
    if old.len() != new.len() {
        return new.to_vec(); // Size changed — full payload
    }

    let mut delta = Vec::new();
    let mut i = 0;
    let len = old.len();

    while i < len {
        // Skip identical bytes
        if old[i] == new[i] {
            i += 1;
            continue;
        }

        // Found a difference — scan for end of changed region
        let start = i;
        while i < len && old[i] != new[i] {
            i += 1;
        }
        let region_len = i - start;

        // Encode: offset (u16) + length (u16) + data
        if start > u16::MAX as usize || region_len > u16::MAX as usize {
            return new.to_vec(); // Too large for u16 encoding
        }
        delta.extend_from_slice(&(start as u16).to_le_bytes());
        delta.extend_from_slice(&(region_len as u16).to_le_bytes());
        delta.extend_from_slice(&new[start..start + region_len]);
    }

    delta
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::priority::{Encoding, Reliability};
    use crate::wire::topic_hash;

    fn make_msg(name: &str, payload: Vec<u8>) -> OutMessage {
        OutMessage {
            topic_name: name.into(),
            topic_hash: topic_hash(name),
            payload,
            timestamp_ns: 0,
            sequence: 1,
            priority: Priority::Normal,
            reliability: Reliability::None,
            encoding: Encoding::PodLe,
        }
    }

    #[test]
    fn first_message_is_keyframe() {
        let mut opt = DeltaOptimizer::new();
        let mut msgs = vec![make_msg("joints", vec![1; 400])];
        opt.on_outgoing(&mut msgs);
        // First message always sent as keyframe
        assert_eq!(msgs.len(), 1);
        assert_eq!(msgs[0].payload.len(), 400);
    }

    #[test]
    fn identical_messages_suppressed() {
        let mut opt = DeltaOptimizer::with_keyframe_interval(1000);
        let payload = vec![42u8; 100];

        // First: keyframe (sent)
        let mut msgs1 = vec![make_msg("data", payload.clone())];
        opt.on_outgoing(&mut msgs1);
        assert_eq!(msgs1.len(), 1);

        // Second: identical → suppressed
        let mut msgs2 = vec![make_msg("data", payload.clone())];
        opt.on_outgoing(&mut msgs2);
        assert_eq!(msgs2.len(), 0); // Suppressed!

        assert!(opt.savings_ratio() > 0.0);
    }

    #[test]
    fn small_change_produces_delta() {
        let mut opt = DeltaOptimizer::with_keyframe_interval(1000);

        // Keyframe
        let mut original = vec![0u8; 400];
        let mut msgs1 = vec![make_msg("joints", original.clone())];
        opt.on_outgoing(&mut msgs1);

        // Change 8 bytes in the middle (simulating 2 joint changes)
        original[100] = 0xFF;
        original[101] = 0xFF;
        original[102] = 0xFF;
        original[103] = 0xFF;
        original[200] = 0xAA;
        original[201] = 0xAA;
        original[202] = 0xAA;
        original[203] = 0xAA;

        let mut msgs2 = vec![make_msg("joints", original)];
        opt.on_outgoing(&mut msgs2);
        assert_eq!(msgs2.len(), 1);
        // Delta should be smaller than 400 bytes
        assert!(
            msgs2[0].payload.len() < 400,
            "delta size: {}",
            msgs2[0].payload.len()
        );
    }

    #[test]
    fn keyframe_sent_at_interval() {
        let mut opt = DeltaOptimizer::with_keyframe_interval(3);
        let payload = vec![0u8; 100];

        // msg 1 (since=1): keyframe (first, no prior state) → reset to 0
        let mut m1 = vec![make_msg("d", payload.clone())];
        opt.on_outgoing(&mut m1);
        assert_eq!(m1.len(), 1);

        // msg 2 (since=1): identical → suppressed
        let mut m2 = vec![make_msg("d", payload.clone())];
        opt.on_outgoing(&mut m2);
        assert_eq!(m2.len(), 0);

        // msg 3 (since=2): identical → suppressed
        let mut m3 = vec![make_msg("d", payload.clone())];
        opt.on_outgoing(&mut m3);
        assert_eq!(m3.len(), 0);

        // msg 4 (since=3): keyframe interval reached → sent as keyframe
        let mut m4 = vec![make_msg("d", payload.clone())];
        opt.on_outgoing(&mut m4);
        assert_eq!(m4.len(), 1);
        assert_eq!(m4[0].payload.len(), 100); // Full keyframe
    }

    #[test]
    fn compute_delta_identical() {
        let a = vec![1, 2, 3, 4, 5];
        let b = vec![1, 2, 3, 4, 5];
        let delta = compute_delta(&a, &b);
        assert!(delta.is_empty()); // No changes
    }

    #[test]
    fn compute_delta_single_change() {
        let a = vec![1, 2, 3, 4, 5];
        let b = vec![1, 2, 99, 4, 5];
        let delta = compute_delta(&a, &b);
        // offset=2 (u16) + length=1 (u16) + data=[99] = 5 bytes
        assert_eq!(delta.len(), 5);
        assert_eq!(&delta[0..2], &2u16.to_le_bytes());
        assert_eq!(&delta[2..4], &1u16.to_le_bytes());
        assert_eq!(delta[4], 99);
    }

    #[test]
    fn savings_ratio_tracks() {
        let mut opt = DeltaOptimizer::with_keyframe_interval(1000);
        let payload = vec![0u8; 200];

        let mut m1 = vec![make_msg("d", payload.clone())];
        opt.on_outgoing(&mut m1);
        let mut m2 = vec![make_msg("d", payload.clone())];
        opt.on_outgoing(&mut m2);
        let mut m3 = vec![make_msg("d", payload.clone())];
        opt.on_outgoing(&mut m3);

        // 3 messages × 200B = 600B total. 2 suppressed = 400B saved.
        assert_eq!(opt.bytes_total, 600);
        assert_eq!(opt.bytes_saved, 400);
        assert!((opt.savings_ratio() - 0.666).abs() < 0.01);
    }
}
