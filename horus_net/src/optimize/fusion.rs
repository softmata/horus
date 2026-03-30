//! Fusion optimizer — batch correlated topics into single UDP packets.
//!
//! When multiple topics publish within the same tick (IMU + joints + battery at 1kHz),
//! fuse into one packet. 10 correlated 1kHz topics → 10x fewer packets.
//!
//! Implementation: buffer outgoing messages for up to `fusion_window`.
//! If multiple messages accumulate, they stay batched in the Vec (the wire
//! encoder handles batching via encode_batch with BATCH flag).

use std::time::{Duration, Instant};

use crate::optimize::Optimizer;
use crate::wire::{InMessage, OutMessage};

/// Default fusion window: 500µs.
const DEFAULT_FUSION_WINDOW: Duration = Duration::from_micros(500);

/// Fusion optimizer — batches messages within a time window.
pub struct FusionOptimizer {
    window: Duration,
    buffer: Vec<OutMessage>,
    window_start: Option<Instant>,
    /// Metrics: messages before fusion.
    messages_in: u64,
    /// Metrics: batches out (each batch = 1 packet).
    batches_out: u64,
}

impl Default for FusionOptimizer {
    fn default() -> Self {
        Self::new()
    }
}

impl FusionOptimizer {
    pub fn new() -> Self {
        Self {
            window: DEFAULT_FUSION_WINDOW,
            buffer: Vec::new(),
            window_start: None,
            messages_in: 0,
            batches_out: 0,
        }
    }

    pub fn with_window(window: Duration) -> Self {
        Self {
            window,
            ..Self::new()
        }
    }

    /// Fusion ratio: messages_in / batches_out. Higher = more fusion.
    pub fn fusion_ratio(&self) -> f64 {
        if self.batches_out == 0 {
            1.0
        } else {
            self.messages_in as f64 / self.batches_out as f64
        }
    }
}

impl Optimizer for FusionOptimizer {
    fn on_outgoing(&mut self, messages: &mut Vec<OutMessage>) {
        if messages.is_empty() {
            return;
        }

        self.messages_in += messages.len() as u64;

        let now = Instant::now();

        // If no window started, start one and buffer everything
        if self.window_start.is_none() {
            self.window_start = Some(now);
            self.buffer.append(messages);
            // Don't flush yet — wait for window to expire or next call
            return;
        }

        // Window is active — add to buffer
        self.buffer.append(messages);

        // Check if window expired
        if let Some(start) = self.window_start {
            if now.duration_since(start) >= self.window {
                // Flush: move all buffered messages out as one batch
                *messages = std::mem::take(&mut self.buffer);
                self.window_start = None;
                self.batches_out += 1;
            }
            // Else: keep buffering (messages is now empty, buffer has everything)
        }
    }

    fn on_incoming(&mut self, _messages: &mut Vec<InMessage>) {
        // Incoming batched packets are already decoded by wire::decode_packet
        // into individual InMessages — no action needed.
    }

    fn name(&self) -> &str {
        "fusion"
    }
}

/// Force flush any buffered messages (call at end of export cycle).
impl FusionOptimizer {
    pub fn flush(&mut self) -> Vec<OutMessage> {
        if !self.buffer.is_empty() {
            self.batches_out += 1;
        }
        self.window_start = None;
        std::mem::take(&mut self.buffer)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::priority::{Encoding, Priority, Reliability};
    use crate::wire::topic_hash;

    fn make_msg(name: &str) -> OutMessage {
        OutMessage {
            topic_name: name.into(),
            topic_hash: topic_hash(name),
            payload: vec![0u8; 64],
            timestamp_ns: 0,
            sequence: 1,
            priority: Priority::Normal,
            reliability: Reliability::None,
            encoding: Encoding::PodLe,
        }
    }

    #[test]
    fn single_message_buffered() {
        let mut opt = FusionOptimizer::new();
        let mut msgs = vec![make_msg("imu")];
        opt.on_outgoing(&mut msgs);
        // First call: buffered, not flushed
        assert!(msgs.is_empty());
        assert_eq!(opt.buffer.len(), 1);
    }

    #[test]
    fn flush_returns_buffered() {
        let mut opt = FusionOptimizer::new();
        let mut msgs = vec![make_msg("imu"), make_msg("odom")];
        opt.on_outgoing(&mut msgs);

        let flushed = opt.flush();
        assert_eq!(flushed.len(), 2);
        assert!(opt.buffer.is_empty());
    }

    #[test]
    fn window_expiry_flushes() {
        let mut opt = FusionOptimizer::with_window(Duration::from_millis(1));

        // First batch: buffered
        let mut msgs1 = vec![make_msg("imu")];
        opt.on_outgoing(&mut msgs1);
        assert!(msgs1.is_empty());

        // Wait for window to expire
        std::thread::sleep(Duration::from_millis(5));

        // Second batch: triggers flush
        let mut msgs2 = vec![make_msg("odom")];
        opt.on_outgoing(&mut msgs2);
        // msgs2 should now contain the flushed batch (imu + odom)
        assert_eq!(msgs2.len(), 2);
    }

    #[test]
    fn fusion_ratio_tracks() {
        let mut opt = FusionOptimizer::with_window(Duration::from_millis(1));

        let mut msgs = vec![make_msg("imu"), make_msg("odom"), make_msg("joints")];
        opt.on_outgoing(&mut msgs);
        std::thread::sleep(Duration::from_millis(5));

        let mut trigger = vec![make_msg("battery")];
        opt.on_outgoing(&mut trigger);

        // 4 messages in, 1 batch out
        assert_eq!(opt.messages_in, 4);
        assert_eq!(opt.batches_out, 1);
        assert_eq!(opt.fusion_ratio(), 4.0);
    }

    #[test]
    fn empty_messages_noop() {
        let mut opt = FusionOptimizer::new();
        let mut msgs: Vec<OutMessage> = vec![];
        opt.on_outgoing(&mut msgs);
        assert!(msgs.is_empty());
        assert!(opt.buffer.is_empty());
    }
}
