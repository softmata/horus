//! Flow control — sequence gap tracking, adaptive send rate.
//!
//! Tracks per-peer per-topic sequence gaps for loss rate calculation.
//! Phase 1: should_send() always returns true (rate adaptation in R8).
//!
//! See blueprint section 14.

use std::collections::HashMap;
use std::time::{Duration, Instant};

/// Window duration for loss rate calculation.
const WINDOW_DURATION: Duration = Duration::from_secs(1);

/// Adaptive send rate levels.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SendRate {
    /// <1% loss: send everything.
    Full,
    /// 1-5% loss: send every 2nd message.
    Half,
    /// 5-20% loss: send every 4th message.
    Quarter,
    /// >20% loss: send every 100th message (keyframes only).
    KeyframeOnly,
}

impl SendRate {
    pub fn from_loss(loss_rate: f32) -> Self {
        if loss_rate < 0.01 {
            Self::Full
        } else if loss_rate < 0.05 {
            Self::Half
        } else if loss_rate < 0.20 {
            Self::Quarter
        } else {
            Self::KeyframeOnly
        }
    }
}

/// Per-peer per-topic receive state.
#[derive(Debug)]
struct RecvState {
    last_sequence: u32,
    gaps: u32,
    total: u32,
    window_start: Instant,
}

impl RecvState {
    fn new(sequence: u32) -> Self {
        Self {
            last_sequence: sequence,
            gaps: 0,
            total: 1,
            window_start: Instant::now(),
        }
    }

    fn loss_rate(&self) -> f32 {
        if self.total == 0 {
            0.0
        } else {
            self.gaps as f32 / self.total as f32
        }
    }
}

/// Flow controller — tracks loss rates, will do rate adaptation in Phase 2.
pub struct FlowController {
    /// Per (peer_id_hash, topic_hash) receive state.
    ///
    /// Bounded by `netfilter::MAX_TRACKED_KEYS`: both halves of the key come
    /// straight off the wire before any guard runs, so without a cap a peer that
    /// varies either field grows this map until the process is OOM-killed.
    state: HashMap<(u16, u32), RecvState>,
    /// Whether the cap-hit warning has already been emitted.
    cap_warned: bool,
}

impl FlowController {
    pub fn new() -> Self {
        Self {
            state: HashMap::new(),
            cap_warned: false,
        }
    }

    /// Record receiving a message. Detects gaps in sequence numbers.
    pub fn on_received(&mut self, peer_hash: u16, topic_hash: u32, sequence: u32) {
        let key = (peer_hash, topic_hash);

        if let Some(state) = self.state.get_mut(&key) {
            // Reset window if expired
            if state.window_start.elapsed() >= WINDOW_DURATION {
                state.gaps = 0;
                state.total = 0;
                state.window_start = Instant::now();
            }

            state.total += 1;

            // Detect gaps (sequence should be last + 1)
            let expected = state.last_sequence.wrapping_add(1);
            if sequence > expected && sequence.wrapping_sub(expected) < u32::MAX / 2 {
                // Gap detected: missed (sequence - expected) messages
                let missed = sequence.wrapping_sub(expected);
                state.gaps += missed;
            }

            if sequence > state.last_sequence
                || state.last_sequence.wrapping_sub(sequence) > u32::MAX / 2
            {
                state.last_sequence = sequence;
            }
        } else if crate::netfilter::enforce_key_cap(&mut self.state, &mut self.cap_warned) {
            self.state.insert(key, RecvState::new(sequence));
        }
    }

    /// Get loss rate for a specific peer+topic. Returns 0.0 if not tracked.
    pub fn loss_rate(&self, peer_hash: u16, topic_hash: u32) -> f32 {
        self.state
            .get(&(peer_hash, topic_hash))
            .map(|s| s.loss_rate())
            .unwrap_or(0.0)
    }

    /// Adaptive send rate based on loss rate.
    ///
    /// - <1% loss: Full (send everything)
    /// - 1-5% loss: Half (every 2nd message)
    /// - 5-20% loss: Quarter (every 4th message)
    /// - >20% loss: KeyframeOnly (every 100th)
    ///
    /// Immediate/RealTime priorities are NEVER rate-limited (checked by caller).
    ///
    /// `peer_hash` is the id hash of the DESTINATION peer — the same value
    /// `on_received` records from that peer's packet headers. Passing our own
    /// id hash here looks up a key the receive path never populates, so the
    /// loss rate reads 0.0 and the throttle silently never engages. The
    /// parameter order matches `on_received` and `loss_rate` so that mistake
    /// is a type error rather than a silent no-op.
    pub fn should_send(&self, peer_hash: u16, topic_hash: u32) -> bool {
        let loss = self.loss_rate(peer_hash, topic_hash);
        let rate = SendRate::from_loss(loss);

        let key = (peer_hash, topic_hash);
        let total = self.state.get(&key).map(|s| s.total).unwrap_or(0);

        match rate {
            SendRate::Full => true,
            SendRate::Half => total.is_multiple_of(2),
            SendRate::Quarter => total.is_multiple_of(4),
            SendRate::KeyframeOnly => total.is_multiple_of(100),
        }
    }

    /// Number of tracked (peer, topic) pairs.
    pub fn tracked_count(&self) -> usize {
        self.state.len()
    }
}

impl Default for FlowController {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn no_gaps_zero_loss() {
        let mut fc = FlowController::new();
        fc.on_received(0x1234, 100, 1);
        fc.on_received(0x1234, 100, 2);
        fc.on_received(0x1234, 100, 3);
        assert_eq!(fc.loss_rate(0x1234, 100), 0.0);
    }

    #[test]
    fn gap_detected() {
        let mut fc = FlowController::new();
        fc.on_received(0x1234, 100, 1);
        fc.on_received(0x1234, 100, 2);
        fc.on_received(0x1234, 100, 5); // Skipped 3,4
                                        // gaps=2, total=3 → loss ~66%
        let loss = fc.loss_rate(0x1234, 100);
        assert!(loss > 0.5, "loss={loss}, expected > 0.5");
    }

    #[test]
    fn multiple_gaps() {
        let mut fc = FlowController::new();
        fc.on_received(0x1234, 100, 1);
        fc.on_received(0x1234, 100, 4); // gap 2,3
        fc.on_received(0x1234, 100, 8); // gap 5,6,7
                                        // gaps=5, total=3
        let loss = fc.loss_rate(0x1234, 100);
        assert!(loss > 0.5);
    }

    #[test]
    fn per_peer_per_topic() {
        let mut fc = FlowController::new();
        fc.on_received(0x1111, 100, 1);
        fc.on_received(0x1111, 100, 5); // gap on peer 1111

        fc.on_received(0x2222, 100, 1);
        fc.on_received(0x2222, 100, 2); // no gap on peer 2222

        assert!(fc.loss_rate(0x1111, 100) > 0.0);
        assert_eq!(fc.loss_rate(0x2222, 100), 0.0);
    }

    #[test]
    fn untracked_returns_zero() {
        let fc = FlowController::new();
        assert_eq!(fc.loss_rate(0x9999, 999), 0.0);
    }

    #[test]
    fn should_send_true_when_no_loss() {
        let mut fc = FlowController::new();
        // Sequential messages → 0% loss → Full rate
        fc.on_received(0x1234, 100, 1);
        fc.on_received(0x1234, 100, 2);
        fc.on_received(0x1234, 100, 3);
        assert!(fc.should_send(0x1234u16, 100u32));
    }

    /// The throttle must key on the DESTINATION peer. Sending to a lossy peer
    /// has to de-rate even though a *different* peer on the same topic is
    /// clean — if the key is wrong (e.g. our own id hash) every lookup misses,
    /// `loss_rate` reads 0.0, and `SendRate::Full` sends everything forever.
    #[test]
    fn should_send_keys_on_the_destination_peer() {
        let mut fc = FlowController::new();
        const TOPIC: u32 = 100;
        const LOSSY: u16 = 0x1111;
        const CLEAN: u16 = 0x2222;
        const OURS: u16 = 0x3333;

        // 1 of every 3 sequence numbers arrives from LOSSY → ~66% loss.
        for seq in (1..=90).step_by(3) {
            fc.on_received(LOSSY, TOPIC, seq);
        }
        for seq in 1..=30 {
            fc.on_received(CLEAN, TOPIC, seq);
        }

        assert!(
            fc.loss_rate(LOSSY, TOPIC) > 0.2,
            "expected KeyframeOnly-grade loss, got {}",
            fc.loss_rate(LOSSY, TOPIC)
        );
        assert_eq!(fc.loss_rate(CLEAN, TOPIC), 0.0);

        // The clean peer keeps full rate; the lossy one is throttled to
        // roughly every 100th message, so at least one of a short run of
        // send decisions must come back false.
        assert!(fc.should_send(CLEAN, TOPIC));
        assert!(
            !fc.should_send(LOSSY, TOPIC),
            "a peer at >20% loss must be de-rated, not sent everything"
        );

        // An id hash the receive path never recorded — such as our own —
        // misses the map entirely and degrades to Full. This is the shape of
        // the bug, pinned so a call site cannot quietly reintroduce it.
        assert_eq!(fc.loss_rate(OURS, TOPIC), 0.0);
        assert!(fc.should_send(OURS, TOPIC));
    }

    #[test]
    fn send_rate_from_loss() {
        assert_eq!(SendRate::from_loss(0.0), SendRate::Full);
        assert_eq!(SendRate::from_loss(0.005), SendRate::Full);
        assert_eq!(SendRate::from_loss(0.02), SendRate::Half);
        assert_eq!(SendRate::from_loss(0.10), SendRate::Quarter);
        assert_eq!(SendRate::from_loss(0.30), SendRate::KeyframeOnly);
    }

    #[test]
    fn tracked_count() {
        let mut fc = FlowController::new();
        assert_eq!(fc.tracked_count(), 0);
        fc.on_received(0x1111, 100, 1);
        assert_eq!(fc.tracked_count(), 1);
        fc.on_received(0x1111, 200, 1); // different topic
        assert_eq!(fc.tracked_count(), 2);
        fc.on_received(0x2222, 100, 1); // different peer
        assert_eq!(fc.tracked_count(), 3);
    }
}
