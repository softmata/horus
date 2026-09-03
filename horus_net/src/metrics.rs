//! Lock-free metrics — per-peer and per-topic atomic counters.
//!
//! Always-on, near-zero overhead. Internal to the Replicator.
//! Uses Relaxed ordering for counters (exact counts not needed for monitoring).
//!
//! See blueprint section 15.

use std::collections::HashMap;
use std::sync::atomic::{AtomicU32, AtomicU64, Ordering};

/// Per-peer metrics.
#[derive(Debug, Default)]
pub struct PeerMetrics {
    pub rtt_us: AtomicU64,
    pub loss_rate_permille: AtomicU32,
    pub bytes_sent: AtomicU64,
    pub bytes_received: AtomicU64,
    pub packets_sent: AtomicU64,
    pub packets_received: AtomicU64,
    /// Sends this machine's own socket refused — the datagram never left.
    ///
    /// Deliberately NOT a liveness signal for the peer. UDP is connectionless,
    /// so `send_to` reports success for a destination that is gone: measured
    /// here on Linux 7.0, 20000/20000 sends to an off-net host and 20000/20000
    /// to a closed port both returned Ok. What this counts is local refusal
    /// (EINVAL, ENETUNREACH, EMSGSIZE) — packets that `packets_sent` used to
    /// absorb as delivered because the send result was discarded. Dead links
    /// are `heartbeat::SafetyHeartbeat`'s job.
    pub send_errors: AtomicU64,
    pub last_seen_ms: AtomicU64,
}

/// Per-topic metrics.
#[derive(Debug, Default)]
pub struct TopicMetrics {
    pub messages_sent: AtomicU64,
    pub messages_received: AtomicU64,
    pub messages_dropped: AtomicU64,
    /// Local messages that existed in SHM and never left this machine —
    /// decimated by latest-only export sampling, or lapped in the ring before
    /// the exporter reached them. Counted separately from `messages_dropped`,
    /// which is an import-side refusal: this one is data the LAN never saw.
    pub messages_skipped: AtomicU64,
    pub bytes_sent: AtomicU64,
    pub bytes_received: AtomicU64,
    pub last_sequence: AtomicU32,
}

/// Snapshot of metrics for reporting (non-atomic).
#[derive(Debug, Clone, Default)]
pub struct MetricsSnapshot {
    pub peers: Vec<PeerSnapshot>,
    pub topics: Vec<TopicSnapshot>,
}

#[derive(Debug, Clone)]
pub struct PeerSnapshot {
    pub peer_hash: u16,
    pub rtt_us: u64,
    pub loss_permille: u32,
    pub bytes_sent: u64,
    pub bytes_received: u64,
    pub packets_sent: u64,
    pub packets_received: u64,
    /// See [`PeerMetrics::send_errors`] — local refusals, not lost packets.
    pub send_errors: u64,
}

#[derive(Debug, Clone)]
pub struct TopicSnapshot {
    pub topic_hash: u32,
    pub messages_sent: u64,
    pub messages_received: u64,
    pub messages_dropped: u64,
    /// Local messages that never reached the network. See
    /// [`TopicMetrics::messages_skipped`].
    pub messages_skipped: u64,
    pub bytes_sent: u64,
    pub bytes_received: u64,
}

/// Network metrics — internal to the Replicator.
pub struct NetMetrics {
    peers: HashMap<u16, PeerMetrics>,
    topics: HashMap<u32, TopicMetrics>,
    /// Count of rejected imports due to type hash mismatch.
    type_mismatches: AtomicU64,
}

impl NetMetrics {
    pub fn new() -> Self {
        Self {
            peers: HashMap::new(),
            topics: HashMap::new(),
            type_mismatches: AtomicU64::new(0),
        }
    }

    /// Record a type hash mismatch rejection.
    pub fn record_type_mismatch(&self) {
        self.type_mismatches.fetch_add(1, Ordering::Relaxed);
    }

    /// Get total type mismatch count.
    pub fn type_mismatches(&self) -> u64 {
        self.type_mismatches.load(Ordering::Relaxed)
    }

    /// Record bytes sent to a peer.
    ///
    /// `peer_hash` is the DESTINATION's id hash — the value that peer stamps
    /// into the packets it sends us — never our own. Keying on ourselves
    /// collapses every peer into one row filed under this node.
    ///
    /// Call this only for a send the socket accepted; a refused one goes to
    /// [`Self::record_send_error`].
    pub fn record_send(&mut self, peer_hash: u16, bytes: usize) {
        let pm = self.peers.entry(peer_hash).or_default();
        pm.bytes_sent.fetch_add(bytes as u64, Ordering::Relaxed);
        pm.packets_sent.fetch_add(1, Ordering::Relaxed);
    }

    /// Record a send to a peer that the local socket refused.
    ///
    /// See [`PeerMetrics::send_errors`] for what this does and does not mean.
    pub fn record_send_error(&mut self, peer_hash: u16) {
        let pm = self.peers.entry(peer_hash).or_default();
        pm.send_errors.fetch_add(1, Ordering::Relaxed);
    }

    /// Record bytes received from a peer.
    ///
    /// `peer_hash` is the SENDER's id hash, resolved from the datagram itself
    /// — see [`Self::record_send`] for why our own id is the wrong key.
    pub fn record_recv(&mut self, peer_hash: u16, bytes: usize) {
        let pm = self.peers.entry(peer_hash).or_default();
        pm.bytes_received.fetch_add(bytes as u64, Ordering::Relaxed);
        pm.packets_received.fetch_add(1, Ordering::Relaxed);
    }

    /// Record a sent message on a topic.
    pub fn record_topic_send(&mut self, topic_hash: u32, bytes: usize) {
        let tm = self.topics.entry(topic_hash).or_default();
        tm.messages_sent.fetch_add(1, Ordering::Relaxed);
        tm.bytes_sent.fetch_add(bytes as u64, Ordering::Relaxed);
    }

    /// Record a received message on a topic.
    pub fn record_topic_recv(&mut self, topic_hash: u32, bytes: usize) {
        let tm = self.topics.entry(topic_hash).or_default();
        tm.messages_received.fetch_add(1, Ordering::Relaxed);
        tm.bytes_received.fetch_add(bytes as u64, Ordering::Relaxed);
    }

    /// Record a dropped message (sequence gap).
    pub fn record_topic_drop(&mut self, topic_hash: u32) {
        let tm = self.topics.entry(topic_hash).or_default();
        tm.messages_dropped.fetch_add(1, Ordering::Relaxed);
    }

    /// Record local messages that were never exported.
    ///
    /// The export path polls SHM on a timer and, by default, takes the newest
    /// slot per tick. Everything else published in that tick is gone — and used
    /// to be gone with no counter anywhere, so a topic replicating at 20 Hz
    /// instead of 500 Hz was indistinguishable from a topic published at 20 Hz.
    pub fn record_topic_skipped(&mut self, topic_hash: u32, count: u64) {
        if count == 0 {
            return;
        }
        let tm = self.topics.entry(topic_hash).or_default();
        tm.messages_skipped.fetch_add(count, Ordering::Relaxed);
    }

    /// Take a consistent snapshot for reporting.
    pub fn snapshot(&self) -> MetricsSnapshot {
        let peers = self
            .peers
            .iter()
            .map(|(&hash, pm)| PeerSnapshot {
                peer_hash: hash,
                rtt_us: pm.rtt_us.load(Ordering::Relaxed),
                loss_permille: pm.loss_rate_permille.load(Ordering::Relaxed),
                bytes_sent: pm.bytes_sent.load(Ordering::Relaxed),
                bytes_received: pm.bytes_received.load(Ordering::Relaxed),
                packets_sent: pm.packets_sent.load(Ordering::Relaxed),
                packets_received: pm.packets_received.load(Ordering::Relaxed),
                send_errors: pm.send_errors.load(Ordering::Relaxed),
            })
            .collect();

        let topics = self
            .topics
            .iter()
            .map(|(&hash, tm)| TopicSnapshot {
                topic_hash: hash,
                messages_sent: tm.messages_sent.load(Ordering::Relaxed),
                messages_received: tm.messages_received.load(Ordering::Relaxed),
                messages_dropped: tm.messages_dropped.load(Ordering::Relaxed),
                messages_skipped: tm.messages_skipped.load(Ordering::Relaxed),
                bytes_sent: tm.bytes_sent.load(Ordering::Relaxed),
                bytes_received: tm.bytes_received.load(Ordering::Relaxed),
            })
            .collect();

        MetricsSnapshot { peers, topics }
    }
}

impl Default for NetMetrics {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn record_send_recv() {
        let mut m = NetMetrics::new();
        m.record_send(0x1234, 100);
        m.record_send(0x1234, 200);
        m.record_recv(0x1234, 50);

        let snap = m.snapshot();
        assert_eq!(snap.peers.len(), 1);
        assert_eq!(snap.peers[0].bytes_sent, 300);
        assert_eq!(snap.peers[0].bytes_received, 50);
        assert_eq!(snap.peers[0].packets_sent, 2);
        assert_eq!(snap.peers[0].packets_received, 1);
    }

    #[test]
    fn record_topic_metrics() {
        let mut m = NetMetrics::new();
        m.record_topic_send(100, 64);
        m.record_topic_send(100, 64);
        m.record_topic_recv(100, 64);
        m.record_topic_drop(100);

        let snap = m.snapshot();
        assert_eq!(snap.topics.len(), 1);
        assert_eq!(snap.topics[0].messages_sent, 2);
        assert_eq!(snap.topics[0].messages_received, 1);
        assert_eq!(snap.topics[0].messages_dropped, 1);
        assert_eq!(snap.topics[0].bytes_sent, 128);
    }

    #[test]
    fn skipped_messages_are_counted_per_topic() {
        let mut m = NetMetrics::new();
        m.record_topic_send(100, 64);
        m.record_topic_skipped(100, 24);
        m.record_topic_skipped(100, 24);
        // A zero delta must not conjure a topic row out of nothing.
        m.record_topic_skipped(200, 0);

        let snap = m.snapshot();
        assert_eq!(snap.topics.len(), 1);
        assert_eq!(snap.topics[0].messages_sent, 1);
        assert_eq!(
            snap.topics[0].messages_skipped, 48,
            "the 48 messages this topic published and never exported must be \
             visible somewhere"
        );
    }

    #[test]
    fn a_refused_send_is_not_counted_as_a_delivered_one() {
        let mut m = NetMetrics::new();
        m.record_send(0x1234, 100);
        m.record_send_error(0x1234);
        m.record_send_error(0x1234);

        let snap = m.snapshot();
        assert_eq!(snap.peers.len(), 1);
        assert_eq!(
            snap.peers[0].packets_sent, 1,
            "only the send the socket accepted may count as sent"
        );
        assert_eq!(snap.peers[0].bytes_sent, 100);
        assert_eq!(snap.peers[0].send_errors, 2);
    }

    #[test]
    fn multiple_peers() {
        let mut m = NetMetrics::new();
        m.record_send(0x1111, 100);
        m.record_send(0x2222, 200);

        let snap = m.snapshot();
        assert_eq!(snap.peers.len(), 2);
    }

    #[test]
    fn empty_snapshot() {
        let m = NetMetrics::new();
        let snap = m.snapshot();
        assert!(snap.peers.is_empty());
        assert!(snap.topics.is_empty());
    }
}
