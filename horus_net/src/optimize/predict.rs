//! Predict optimizer — suppress predictable updates.
//!
//! Sender tracks prediction model (linear extrapolation). Only transmits
//! corrections when prediction error exceeds threshold. Stationary robot → 99% suppression.

use std::collections::HashMap;

use crate::optimize::Optimizer;
use crate::priority::Priority;
use crate::wire::{InMessage, OutMessage};

/// Default prediction error threshold (normalized).
const DEFAULT_THRESHOLD: f64 = 0.01;

/// Cold start: always send first N messages to build prediction model.
const COLD_START_COUNT: u32 = 5;

/// Max time between unconditional sends (freshness guarantee).
const MAX_SUPPRESS_COUNT: u32 = 100; // At 100Hz → 1s

/// Per-topic prediction state.
struct TopicState {
    /// Previous payload bytes.
    prev: Vec<u8>,
    /// Velocity estimate (byte-level delta per tick).
    velocity: Vec<f64>,
    /// Messages since last send.
    suppress_count: u32,
    /// Total messages seen (for cold start).
    total_seen: u32,
}

/// Predict optimizer — linear extrapolation with threshold.
pub struct PredictOptimizer {
    states: HashMap<u32, TopicState>,
    threshold: f64,
    /// Metrics.
    messages_in: u64,
    messages_suppressed: u64,
}

impl PredictOptimizer {
    pub fn new() -> Self {
        Self {
            states: HashMap::new(),
            threshold: DEFAULT_THRESHOLD,
            messages_in: 0,
            messages_suppressed: 0,
        }
    }

    pub fn with_threshold(threshold: f64) -> Self {
        Self {
            threshold,
            ..Self::new()
        }
    }

    /// Suppression ratio (0.0 = nothing suppressed, 1.0 = everything suppressed).
    pub fn suppression_ratio(&self) -> f64 {
        if self.messages_in == 0 {
            0.0
        } else {
            self.messages_suppressed as f64 / self.messages_in as f64
        }
    }
}

impl Optimizer for PredictOptimizer {
    fn on_outgoing(&mut self, messages: &mut Vec<OutMessage>) {
        messages.retain_mut(|msg| {
            self.messages_in += 1;
            let hash = msg.topic_hash;

            let state = self.states.entry(hash).or_insert_with(|| TopicState {
                prev: Vec::new(),
                velocity: Vec::new(),
                suppress_count: 0,
                total_seen: 0,
            });

            state.total_seen += 1;

            // Cold start: always send first N messages
            if state.total_seen <= COLD_START_COUNT {
                if !state.prev.is_empty() && state.prev.len() == msg.payload.len() {
                    // Update velocity estimate
                    state.velocity = state
                        .prev
                        .iter()
                        .zip(msg.payload.iter())
                        .map(|(&a, &b)| b as f64 - a as f64)
                        .collect();
                }
                state.prev = msg.payload.clone();
                state.suppress_count = 0;
                return true; // Send
            }

            // Freshness guarantee: unconditional send every MAX_SUPPRESS_COUNT
            if state.suppress_count >= MAX_SUPPRESS_COUNT {
                state.prev = msg.payload.clone();
                state.suppress_count = 0;
                return true;
            }

            // Predict next value
            if state.prev.len() != msg.payload.len() || state.velocity.len() != msg.payload.len() {
                // Size changed — can't predict, send and reset
                state.prev = msg.payload.clone();
                state.velocity.clear();
                state.suppress_count = 0;
                return true;
            }

            // Compute prediction error (normalized L2)
            let mut error_sq = 0.0f64;
            let mut magnitude_sq = 0.0f64;
            for i in 0..msg.payload.len() {
                let predicted = state.prev[i] as f64 + state.velocity[i];
                let actual = msg.payload[i] as f64;
                let diff = actual - predicted;
                error_sq += diff * diff;
                magnitude_sq += actual * actual;
            }

            let normalized_error = if magnitude_sq > 0.0 {
                (error_sq / magnitude_sq).sqrt()
            } else if error_sq > 0.0 {
                1.0 // Nonzero error from zero baseline
            } else {
                0.0 // Both zero
            };

            if normalized_error < self.threshold {
                // Prediction is good enough — suppress
                // Update velocity from actual (even though we don't send)
                state.velocity = state
                    .prev
                    .iter()
                    .zip(msg.payload.iter())
                    .map(|(&a, &b)| b as f64 - a as f64)
                    .collect();
                state.prev = msg.payload.clone();
                state.suppress_count += 1;
                self.messages_suppressed += 1;
                false // Suppress
            } else {
                // Prediction error too high — send correction
                state.velocity = state
                    .prev
                    .iter()
                    .zip(msg.payload.iter())
                    .map(|(&a, &b)| b as f64 - a as f64)
                    .collect();
                state.prev = msg.payload.clone();
                state.suppress_count = 0;
                true // Send
            }
        });
    }

    fn on_incoming(&mut self, _messages: &mut Vec<InMessage>) {
        // Receiver-side prediction would reconstruct suppressed messages.
        // Deferred: receiver just uses latest received value (staleness, not corruption).
    }

    fn name(&self) -> &str {
        "predict"
    }
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
    fn cold_start_sends_all() {
        let mut opt = PredictOptimizer::new();
        for i in 0..COLD_START_COUNT {
            let mut msgs = vec![make_msg("pose", vec![i as u8; 16])];
            opt.on_outgoing(&mut msgs);
            assert_eq!(msgs.len(), 1, "cold start msg {i} should be sent");
        }
    }

    #[test]
    fn stationary_suppressed() {
        let mut opt = PredictOptimizer::with_threshold(0.1);
        let payload = vec![100u8; 16];

        // Cold start
        for _ in 0..COLD_START_COUNT {
            let mut msgs = vec![make_msg("pose", payload.clone())];
            opt.on_outgoing(&mut msgs);
        }

        // After cold start, identical messages → velocity=0, error=0 → suppress
        let mut suppressed = 0;
        for _ in 0..20 {
            let mut msgs = vec![make_msg("pose", payload.clone())];
            opt.on_outgoing(&mut msgs);
            if msgs.is_empty() {
                suppressed += 1;
            }
        }
        assert!(suppressed > 15, "expected >75% suppression for stationary, got {suppressed}/20");
    }

    #[test]
    fn large_change_not_suppressed() {
        let mut opt = PredictOptimizer::with_threshold(0.01);

        // Cold start with constant values
        for _ in 0..COLD_START_COUNT {
            let mut msgs = vec![make_msg("data", vec![50u8; 8])];
            opt.on_outgoing(&mut msgs);
        }

        // Sudden large change
        let mut msgs = vec![make_msg("data", vec![200u8; 8])];
        opt.on_outgoing(&mut msgs);
        assert_eq!(msgs.len(), 1, "large change should not be suppressed");
    }

    #[test]
    fn freshness_guarantee() {
        let mut opt = PredictOptimizer::with_threshold(0.5); // Very permissive
        let payload = vec![100u8; 8];

        // Cold start
        for _ in 0..COLD_START_COUNT {
            let mut msgs = vec![make_msg("d", payload.clone())];
            opt.on_outgoing(&mut msgs);
        }

        // Send MAX_SUPPRESS_COUNT identical messages
        let mut last_sent = 0;
        for i in 0..(MAX_SUPPRESS_COUNT + 5) {
            let mut msgs = vec![make_msg("d", payload.clone())];
            opt.on_outgoing(&mut msgs);
            if !msgs.is_empty() {
                last_sent = i;
            }
        }
        // Freshness: at least one message sent after MAX_SUPPRESS_COUNT
        assert!(last_sent >= MAX_SUPPRESS_COUNT, "freshness guarantee: last sent at {last_sent}");
    }

    #[test]
    fn suppression_ratio_tracks() {
        let mut opt = PredictOptimizer::with_threshold(0.1);
        let payload = vec![100u8; 8];

        for _ in 0..COLD_START_COUNT {
            let mut msgs = vec![make_msg("d", payload.clone())];
            opt.on_outgoing(&mut msgs);
        }
        for _ in 0..20 {
            let mut msgs = vec![make_msg("d", payload.clone())];
            opt.on_outgoing(&mut msgs);
        }

        assert!(opt.suppression_ratio() > 0.5, "ratio: {}", opt.suppression_ratio());
    }
}
