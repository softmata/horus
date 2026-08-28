//! Safety heartbeat — 50ms between matched peers, link-loss detection.
//!
//! Separate from discovery (which is 1Hz). The safety heartbeat runs at 20Hz
//! (50ms) between matched peers. Bidirectional, direct UDP (NOT multicast).
//!
//! See blueprint section 10.

use std::collections::HashMap;
use std::net::SocketAddr;
use std::time::{Duration, Instant};

use crate::peer::PeerId;
use crate::transport::Transport;
use crate::wire::{self, HeartbeatPayload};

/// Action to take when a link is lost.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LinkLostAction {
    /// Log warning, keep waiting for reconnection (default).
    Warn,
    /// Call enter_safe_state() on subscriber nodes depending on dead peer's topics.
    SafeState,
    /// Stop the scheduler (nuclear option).
    Stop,
}

impl LinkLostAction {
    #[allow(clippy::should_implement_trait)]
    pub fn from_str(s: &str) -> Self {
        match s.to_ascii_lowercase().as_str() {
            "safe_state" | "safestate" => Self::SafeState,
            "stop" => Self::Stop,
            _ => Self::Warn,
        }
    }
}

/// Per-peer heartbeat tracking state.
#[derive(Debug)]
struct PeerHeartbeatState {
    /// Address to send heartbeats to.
    addr: SocketAddr,
    /// Last time we sent a heartbeat to this peer.
    last_sent: Instant,
    /// Last time we received a heartbeat from this peer.
    last_received: Instant,
    /// Number of consecutive missed heartbeats.
    missed_count: u32,
    /// When this peer was first discovered, for the introduction grace period.
    first_seen: Instant,
    /// Whether a heartbeat has ever arrived from this peer.
    ///
    /// Until one has, the peer cannot be judged on missed beats: we discovered
    /// it from its announcement, but it does not know about *us* until its own
    /// announce cycle comes round, and it does not start heartbeating back
    /// before that.
    ever_received: bool,
    /// Whether this link is considered alive.
    link_alive: bool,
    /// Whether we've already fired the link-lost action for this peer.
    action_fired: bool,
    /// RTT estimate in microseconds (EMA, alpha=0.2).
    rtt_us: f64,
}

/// Safety heartbeat manager.
pub struct SafetyHeartbeat {
    /// Heartbeat send interval.
    interval: Duration,
    /// Number of missed heartbeats before declaring link dead.
    missed_threshold: u32,
    /// Default action when a link is lost.
    default_action: LinkLostAction,
    /// Per-peer state.
    peers: HashMap<PeerId, PeerHeartbeatState>,
    /// Our peer ID for heartbeat payloads.
    our_peer_id: [u8; 16],
    /// Monotonic heartbeat sequence counter.
    sequence: u32,
    /// Whether the heartbeat-table-full warning has already been emitted.
    cap_warned: bool,
}

impl SafetyHeartbeat {
    /// How long a newly discovered peer has to say something before its silence
    /// counts against it.
    ///
    /// Discovery announcements go out on a 1 s cycle, so a peer can legitimately
    /// take that long to learn we exist and start heartbeating. Two seconds
    /// leaves room for one missed announcement without turning fleet assembly
    /// into a link-loss event; a peer that is genuinely absent is still
    /// declared lost, just after evidence rather than before it.
    const DISCOVERY_GRACE: Duration = Duration::from_secs(2);

    /// Create with default settings: 50ms interval, 3 missed threshold, Warn action.
    pub fn new(our_peer_id: [u8; 16]) -> Self {
        Self {
            interval: Duration::from_millis(50),
            missed_threshold: 3,
            default_action: LinkLostAction::Warn,
            peers: HashMap::new(),
            our_peer_id,
            sequence: 0,
            cap_warned: false,
        }
    }

    /// Create with custom settings.
    pub fn with_config(
        our_peer_id: [u8; 16],
        interval: Duration,
        missed_threshold: u32,
        default_action: LinkLostAction,
    ) -> Self {
        Self {
            interval,
            missed_threshold,
            default_action,
            peers: HashMap::new(),
            our_peer_id,
            sequence: 0,
            cap_warned: false,
        }
    }

    /// Register a matched peer for heartbeat tracking.
    ///
    /// Capped at `netfilter::MAX_PEERS`, the same ceiling `PeerTable` enforces.
    /// This table used to have no bound at all while its only caller registered
    /// every announcement unconditionally — including the ones `PeerTable`
    /// refused once full — so a flood of forged peer ids grew it without limit
    /// AND made `tick` transmit a heartbeat every 50 ms to each announcement's
    /// (spoofable) source address, forever. The caller now gates on the peer
    /// table's answer; this cap is the defence in depth so a future caller that
    /// forgets cannot re-open the same hole.
    pub fn add_peer(&mut self, peer_id: PeerId, addr: SocketAddr) {
        if !self.peers.contains_key(&peer_id) && self.peers.len() >= crate::netfilter::MAX_PEERS {
            if !self.cap_warned {
                self.cap_warned = true;
                horus_core::terminal::eprint_line(&format!(
                    "[horus_net] Heartbeat table is full ({} peers); ignoring further \
                     registrations. If this is not a real fleet of that size, a host is \
                     forging peer ids — check HORUS_NET_ALLOW_PEERS. (Fires once.)",
                    crate::netfilter::MAX_PEERS
                ));
            }
            return;
        }
        let now = Instant::now();
        self.peers.entry(peer_id).or_insert(PeerHeartbeatState {
            addr,
            last_sent: now - self.interval, // Send immediately on next tick
            last_received: now,
            missed_count: 0,
            first_seen: now,
            ever_received: false,
            link_alive: true,
            action_fired: false,
            rtt_us: 0.0,
        });
    }

    /// Remove a peer (e.g., when discovery marks it dead).
    pub fn remove_peer(&mut self, peer_id: &PeerId) {
        self.peers.remove(peer_id);
    }

    /// Drop every tracked peer for which `keep` returns false.
    ///
    /// The replicator re-syncs this table against the peer table once per
    /// discovery interval. Removal used to happen only through `remove_peer`
    /// on a newly-dead peer, so an entry for a peer that never made it into
    /// (or was silently evicted from) `PeerTable` was never removed and kept
    /// emitting heartbeats indefinitely.
    pub fn retain_peers<F: FnMut(&PeerId) -> bool>(&mut self, mut keep: F) {
        self.peers.retain(|id, _| keep(id));
    }

    /// Update address for a peer (e.g., if their data port changed).
    pub fn update_addr(&mut self, peer_id: &PeerId, addr: SocketAddr) {
        if let Some(state) = self.peers.get_mut(peer_id) {
            state.addr = addr;
        }
    }

    /// Called when we receive a heartbeat from a peer.
    pub fn on_received(&mut self, peer_id: &PeerId) {
        if let Some(state) = self.peers.get_mut(peer_id) {
            let now = Instant::now();
            // RTT estimation: time since we last sent to this peer.
            // Bidirectional heartbeat: our send → peer receives → peer sends back → we receive.
            // Approximate RTT as 2 × one-way latency.
            let rtt_sample = now.duration_since(state.last_sent).as_micros() as f64;
            if state.rtt_us == 0.0 {
                state.rtt_us = rtt_sample; // First sample
            } else {
                // EMA with alpha=0.2
                state.rtt_us = 0.2 * rtt_sample + 0.8 * state.rtt_us;
            }

            state.last_received = now;
            state.missed_count = 0;
            state.ever_received = true;
            if !state.link_alive {
                state.link_alive = true;
                state.action_fired = false;
                horus_core::terminal::eprint_line(&format!(
                    "[horus_net] Link restored to peer {:02X?}...",
                    &peer_id[..4]
                ));
            }
        }
    }

    /// Get RTT estimate for a peer in microseconds.
    pub fn rtt_us(&self, peer_id: &PeerId) -> Option<f64> {
        self.peers.get(peer_id).map(|s| s.rtt_us)
    }

    /// Periodic tick: send heartbeats, check for missed heartbeats.
    ///
    /// Returns a list of (peer_id, LinkLostAction) for peers whose links just died.
    pub fn tick<T: Transport>(
        &mut self,
        transport: &T,
        sender_id_hash: u16,
        packet_seq: &mut u32,
    ) -> Vec<(PeerId, LinkLostAction)> {
        let now = Instant::now();
        let mut link_lost = Vec::new();

        for (peer_id, state) in &mut self.peers {
            // Send heartbeat if interval elapsed
            if now.duration_since(state.last_sent) >= self.interval {
                let payload = HeartbeatPayload {
                    peer_id: self.our_peer_id,
                    heartbeat_sequence: self.sequence,
                };
                self.sequence = self.sequence.wrapping_add(1);

                let mut buf = [0u8; 64];
                let len = wire::encode_heartbeat(sender_id_hash, *packet_seq, &payload, &mut buf);
                *packet_seq = packet_seq.wrapping_add(1);
                let _ = transport.send_to(&buf[..len], state.addr);
                state.last_sent = now;
            }

            // Check for missed heartbeats.
            //
            // Divide in NANOSECONDS, not milliseconds. `interval.as_millis()` is
            // 0 for any sub-millisecond heartbeat interval, and integer division
            // by zero PANICS — on the replicator thread, which the release
            // profile lets unwind alone (no panic="abort"). The process would
            // survive with networking silently dead, `is_running()` still
            // reporting true, and no networked e-stop, for a configuration that
            // is merely aggressive rather than invalid.
            let since_last = now.duration_since(state.last_received);
            let interval_ns = self.interval.as_nanos().max(1);
            let expected_count = (since_last.as_nanos() / interval_ns).max(1) as u32;

            // `>=`, not `>`. The field documents itself as "Number of missed
            // heartbeats before declaring link dead", so a threshold of 3 must
            // declare the link dead on the 3rd missed beat — `>` waited for the
            // 4th, one full interval of extra latency before a safety action.
            // A peer that has never answered yet is not a lost link — it is a
            // peer that has not been introduced. `add_peer` starts this clock
            // the moment discovery sees an announcement, but the far side only
            // learns about us on its own announce cycle, up to
            // `DISCOVERY_GRACE` later, and does not heartbeat before that. At
            // the default 50 ms interval and threshold of 3, that meant
            // declaring the link lost ~150 ms after meeting a peer that was
            // perfectly healthy — measured at 196 ms on loopback with two real
            // replicators. With `on_link_lost = "stop"` that halts a working
            // robot during startup, which is the worst possible false positive:
            // a safety action firing because the fleet was still assembling.
            //
            // So hold off until either the peer has spoken once, or it has had
            // long enough that silence is genuinely evidence of absence.
            let introduced =
                state.ever_received || now.duration_since(state.first_seen) > Self::DISCOVERY_GRACE;

            if introduced && expected_count >= self.missed_threshold && state.link_alive {
                state.link_alive = false;
                state.missed_count = expected_count;

                if !state.action_fired {
                    state.action_fired = true;
                    horus_core::terminal::eprint_line(&format!(
                        "[horus_net] Link lost to peer {:02X?}... ({} missed heartbeats, {}ms)",
                        &peer_id[..4],
                        state.missed_count,
                        since_last.as_millis()
                    ));
                    link_lost.push((*peer_id, self.default_action));
                }
            }
        }

        link_lost
    }

    /// Check if a specific peer's link is alive.
    pub fn is_link_alive(&self, peer_id: &PeerId) -> bool {
        self.peers
            .get(peer_id)
            .map(|s| s.link_alive)
            .unwrap_or(false)
    }

    /// Number of tracked peers.
    pub fn peer_count(&self) -> usize {
        self.peers.len()
    }

    /// Number of alive links.
    pub fn alive_count(&self) -> usize {
        self.peers.values().filter(|s| s.link_alive).count()
    }
}

/// Test support — MockTransport for heartbeat testing.
/// Always compiled so integration tests can use it.
pub mod tests_support {
    use crate::transport::Transport;
    use std::io;
    use std::net::SocketAddr;

    pub struct MockTransport {
        sent: std::cell::RefCell<Vec<(Vec<u8>, SocketAddr)>>,
    }

    impl Default for MockTransport {
        fn default() -> Self {
            Self::new()
        }
    }

    impl MockTransport {
        pub fn new() -> Self {
            Self {
                sent: std::cell::RefCell::new(Vec::new()),
            }
        }
        pub fn sent_count(&self) -> usize {
            self.sent.borrow().len()
        }
    }

    impl Transport for MockTransport {
        fn send_to(&self, data: &[u8], addr: SocketAddr) -> io::Result<usize> {
            self.sent.borrow_mut().push((data.to_vec(), addr));
            Ok(data.len())
        }
        fn recv_from(&self, _buf: &mut [u8]) -> io::Result<(usize, SocketAddr)> {
            Err(io::Error::new(io::ErrorKind::WouldBlock, "mock"))
        }
        fn local_addr(&self) -> io::Result<SocketAddr> {
            Ok("127.0.0.1:0".parse().unwrap())
        }
        fn join_multicast(&self, _group: &str) -> io::Result<()> {
            Ok(())
        }
        #[cfg(unix)]
        fn raw_fd(&self) -> std::os::unix::io::RawFd {
            -1
        }
    }
}

#[cfg(test)]
mod tests {
    use super::tests_support::MockTransport;
    use super::*;

    fn peer_addr() -> SocketAddr {
        "192.168.1.10:9100".parse().unwrap()
    }

    #[test]
    fn add_and_track_peer() {
        let mut hb = SafetyHeartbeat::new([0xAA; 16]);
        hb.add_peer([1; 16], peer_addr());
        assert_eq!(hb.peer_count(), 1);
        assert!(hb.is_link_alive(&[1; 16]));
    }

    #[test]
    fn remove_peer() {
        let mut hb = SafetyHeartbeat::new([0xAA; 16]);
        hb.add_peer([1; 16], peer_addr());
        hb.remove_peer(&[1; 16]);
        assert_eq!(hb.peer_count(), 0);
    }

    #[test]
    fn heartbeat_sent_on_tick() {
        let transport = MockTransport::new();
        let mut hb = SafetyHeartbeat::new([0xAA; 16]);
        hb.add_peer([1; 16], peer_addr());
        let mut seq = 0u32;

        // First tick should send (we set last_sent to past in add_peer)
        hb.tick(&transport, 0x1234, &mut seq);
        assert!(transport.sent_count() >= 1);
    }

    #[test]
    fn link_alive_on_receive() {
        let mut hb = SafetyHeartbeat::new([0xAA; 16]);
        hb.add_peer([1; 16], peer_addr());

        hb.on_received(&[1; 16]);
        assert!(hb.is_link_alive(&[1; 16]));
    }

    #[test]
    fn link_lost_after_timeout() {
        let transport = MockTransport::new();
        let mut hb = SafetyHeartbeat::with_config(
            [0xAA; 16],
            Duration::from_millis(10), // 10ms interval
            3,                         // 3 missed
            LinkLostAction::Warn,
        );
        hb.add_peer([1; 16], peer_addr());
        // Establish the link before letting it go silent: link loss is only
        // meaningful for a peer that has answered at least once. Without this
        // the test exercises the introduction grace period instead.
        hb.on_received(&[1; 16]);
        let mut seq = 0u32;

        // Wait for timeout (3 * 10ms = 30ms)
        std::thread::sleep(Duration::from_millis(50));

        let lost = hb.tick(&transport, 0, &mut seq);
        assert_eq!(lost.len(), 1);
        assert_eq!(lost[0].0, [1; 16]);
        assert_eq!(lost[0].1, LinkLostAction::Warn);
        assert!(!hb.is_link_alive(&[1; 16]));
    }

    #[test]
    fn link_lost_fires_only_once() {
        let transport = MockTransport::new();
        let mut hb = SafetyHeartbeat::with_config(
            [0xAA; 16],
            Duration::from_millis(10),
            3,
            LinkLostAction::SafeState,
        );
        hb.add_peer([1; 16], peer_addr());
        // Establish the link before letting it go silent: link loss is only
        // meaningful for a peer that has answered at least once. Without this
        // the test exercises the introduction grace period instead.
        hb.on_received(&[1; 16]);
        let mut seq = 0u32;

        std::thread::sleep(Duration::from_millis(50));

        let lost1 = hb.tick(&transport, 0, &mut seq);
        assert_eq!(lost1.len(), 1);

        // Second tick: should NOT fire again
        let lost2 = hb.tick(&transport, 0, &mut seq);
        assert!(lost2.is_empty());
    }

    #[test]
    fn link_restored_after_receive() {
        let transport = MockTransport::new();
        let mut hb = SafetyHeartbeat::with_config(
            [0xAA; 16],
            Duration::from_millis(10),
            3,
            LinkLostAction::Warn,
        );
        hb.add_peer([1; 16], peer_addr());
        // Establish the link before letting it go silent: link loss is only
        // meaningful for a peer that has answered at least once. Without this
        // the test exercises the introduction grace period instead.
        hb.on_received(&[1; 16]);
        let mut seq = 0u32;

        // Timeout → link lost
        std::thread::sleep(Duration::from_millis(50));
        hb.tick(&transport, 0, &mut seq);
        assert!(!hb.is_link_alive(&[1; 16]));

        // Receive heartbeat → link restored
        hb.on_received(&[1; 16]);
        assert!(hb.is_link_alive(&[1; 16]));
    }

    #[test]
    fn safe_state_action() {
        let transport = MockTransport::new();
        let mut hb = SafetyHeartbeat::with_config(
            [0xAA; 16],
            Duration::from_millis(10),
            3,
            LinkLostAction::SafeState,
        );
        hb.add_peer([1; 16], peer_addr());
        // Establish the link before letting it go silent: link loss is only
        // meaningful for a peer that has answered at least once. Without this
        // the test exercises the introduction grace period instead.
        hb.on_received(&[1; 16]);
        let mut seq = 0u32;

        std::thread::sleep(Duration::from_millis(50));
        let lost = hb.tick(&transport, 0, &mut seq);
        assert_eq!(lost[0].1, LinkLostAction::SafeState);
    }

    #[test]
    fn alive_count() {
        let transport = MockTransport::new();
        let mut hb = SafetyHeartbeat::with_config(
            [0xAA; 16],
            Duration::from_millis(10),
            3,
            LinkLostAction::Warn,
        );
        hb.add_peer([1; 16], peer_addr());
        hb.add_peer([2; 16], peer_addr());
        // Establish the link before letting it go silent: link loss is only
        // meaningful for a peer that has answered at least once. Without this
        // the test exercises the introduction grace period instead.
        hb.on_received(&[1; 16]);
        hb.on_received(&[2; 16]);
        assert_eq!(hb.alive_count(), 2);

        std::thread::sleep(Duration::from_millis(50));
        let mut seq = 0u32;
        hb.tick(&transport, 0, &mut seq);
        assert_eq!(hb.alive_count(), 0);
    }

    #[test]
    fn link_lost_action_from_str() {
        assert_eq!(LinkLostAction::from_str("warn"), LinkLostAction::Warn);
        assert_eq!(
            LinkLostAction::from_str("safe_state"),
            LinkLostAction::SafeState
        );
        assert_eq!(
            LinkLostAction::from_str("safestate"),
            LinkLostAction::SafeState
        );
        assert_eq!(LinkLostAction::from_str("stop"), LinkLostAction::Stop);
        assert_eq!(LinkLostAction::from_str("unknown"), LinkLostAction::Warn);
    }
}

#[cfg(test)]
mod unit_safety_tests {
    use super::*;
    use crate::transport::Transport;
    use std::net::SocketAddr;

    /// Minimal transport that swallows sends — `tick` needs one, and this test
    /// is about the arithmetic, not the wire.
    struct NullTransport;
    impl Transport for NullTransport {
        fn send_to(&self, _buf: &[u8], _addr: SocketAddr) -> std::io::Result<usize> {
            Ok(0)
        }
        fn recv_from(&self, _buf: &mut [u8]) -> std::io::Result<(usize, SocketAddr)> {
            Err(std::io::Error::new(std::io::ErrorKind::WouldBlock, "null"))
        }
        fn local_addr(&self) -> std::io::Result<SocketAddr> {
            Ok("127.0.0.1:0".parse().unwrap())
        }
        fn join_multicast(&self, _group: &str) -> std::io::Result<()> {
            Ok(())
        }
        fn raw_fd(&self) -> i32 {
            -1
        }
    }

    /// A sub-millisecond heartbeat interval used to panic the replicator thread
    /// via integer division by zero (`interval.as_millis()` == 0). The release
    /// profile has no `panic = "abort"`, so that unwound the horus-net thread
    /// alone: the process lived on with networking silently dead — including
    /// networked e-stop — while `is_running()` kept returning true.
    #[test]
    fn a_sub_millisecond_interval_does_not_panic() {
        let mut hb = SafetyHeartbeat::with_config(
            [7u8; 16],
            Duration::from_micros(500),
            3,
            LinkLostAction::Warn,
        );
        hb.add_peer([9u8; 16], "127.0.0.1:9100".parse().unwrap());
        std::thread::sleep(Duration::from_millis(5));
        // Must not panic on the division.
        let mut seq = 0u32;
        let _ = hb.tick(&NullTransport, 0, &mut seq);
    }

    /// The field documents itself as "Number of missed heartbeats before
    /// declaring link dead", so a threshold of 3 must act on the 3rd missed
    /// beat. `>` waited for the 4th — one whole interval of extra latency before
    /// a safety action fires.
    #[test]
    fn threshold_acts_on_the_nth_missed_beat_not_the_n_plus_first() {
        let interval = Duration::from_millis(10);
        let mut hb = SafetyHeartbeat::with_config([1u8; 16], interval, 3, LinkLostAction::Warn);
        let peer = [2u8; 16];
        hb.add_peer(peer, "127.0.0.1:9100".parse().unwrap());
        // Establish the link before letting it go silent: link loss is only
        // meaningful for a peer that has answered at least once. Without this
        // the test exercises the introduction grace period instead.
        hb.on_received(&peer);

        // Exactly 3 intervals of silence must be enough to declare the link dead.
        std::thread::sleep(interval * 3 + Duration::from_millis(3));
        let mut seq = 0u32;
        let _ = hb.tick(&NullTransport, 0, &mut seq);
        assert!(
            !hb.is_link_alive(&peer),
            "a threshold of 3 must declare the link dead at 3 missed beats, not 4"
        );
    }
}

#[cfg(test)]
mod introduction_grace_tests {
    use super::tests_support::MockTransport;
    use super::*;

    fn peer(n: u8) -> PeerId {
        [n; 16]
    }

    #[test]
    fn a_peer_that_has_never_spoken_is_not_declared_lost_immediately() {
        // The regression: `add_peer` starts the liveness clock the moment
        // discovery sees an announcement, but the far side does not know about
        // us until its own announce cycle and does not heartbeat before then.
        // Judging it on missed beats at 50ms x 3 declared a healthy peer lost
        // ~150ms after meeting it — and with `on_link_lost = "stop"` that halts
        // a working robot while the fleet is still assembling.
        let mut hb = SafetyHeartbeat::new([9; 16]);
        hb.add_peer(peer(1), "127.0.0.1:9101".parse().unwrap());

        // Well past threshold * interval, but inside the introduction grace.
        std::thread::sleep(Duration::from_millis(250));
        let mut seq = 0u32;
        let lost = hb.tick(&MockTransport::new(), 0, &mut seq);
        assert!(
            lost.is_empty(),
            "a peer that has not had time to learn we exist was declared lost: {lost:?}"
        );
        assert!(hb.is_link_alive(&peer(1)));
    }

    #[test]
    fn a_peer_that_answered_once_is_judged_normally_afterwards() {
        // The grace covers introduction only. Once a peer has spoken, silence
        // is real evidence and the normal threshold applies — otherwise the
        // grace would delay every genuine link-loss action by two seconds.
        let mut hb = SafetyHeartbeat::new([9; 16]);
        hb.add_peer(peer(2), "127.0.0.1:9102".parse().unwrap());
        hb.on_received(&peer(2));

        std::thread::sleep(Duration::from_millis(250));
        let mut seq = 0u32;
        let lost = hb.tick(&MockTransport::new(), 0, &mut seq);
        assert!(
            !lost.is_empty(),
            "a peer that answered and then went silent should be reported lost"
        );
    }
}
