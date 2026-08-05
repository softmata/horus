//! Emergency stop replication — reliable cross-machine e-stop.
//!
//! When the local SafetyMonitor triggers emergency stop, broadcast to all
//! peers via `_horus.estop` with triple-redundant delivery:
//! 1. UDP multicast (fast, reaches all peers)
//! 2. Unicast to each known peer (backup for multicast failure)
//! 3. Retry 3× at 10ms intervals (handles packet loss)
//!
//! On receive: immediately call `trigger_external_emergency_stop()`.
//! Target: <5ms from trigger to all peers stopped.

use std::net::SocketAddr;
use std::sync::atomic::{AtomicBool, Ordering};
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

use crate::config::EstopRemotePolicy;
use crate::transport::Transport;

/// Freshness window for a received e-stop timestamp.
///
/// Reject an e-stop older than this (stale / replayed) or further than this into
/// the future (clock desync / garbage). The window is deliberately generous so a
/// genuinely fresh e-stop is never dropped over modest LAN clock skew — an e-stop
/// is a HALT command, and honoring a slightly-skewed real one is safer than
/// discarding it. This is an anti-replay/anti-garbage filter, NOT authentication.
const ESTOP_MAX_AGE_NS: u64 = 5_000_000_000; // 5s in the past
const ESTOP_MAX_FUTURE_SKEW_NS: u64 = 5_000_000_000; // 5s in the future

/// Emitted at most once: acting on the UNAUTHENTICATED networked e-stop channel.
static ESTOP_UNAUTH_WARNED: AtomicBool = AtomicBool::new(false);

/// Emitted at most once: rejected an e-stop whose authentication tag failed.
static ESTOP_FORGERY_WARNED: AtomicBool = AtomicBool::new(false);

/// Emitted at most once: a clock skew too large to be drift.
static ESTOP_CLOCK_WARNED: AtomicBool = AtomicBool::new(false);

/// Domain separator mixed into every e-stop MAC.
///
/// Binds a tag to *this* channel and wire version, so a tag captured from any
/// future MACed channel that shares `HORUS_ESTOP_KEY` cannot be replayed as an
/// e-stop.
const ESTOP_MAC_DOMAIN: &[u8] = b"horus.estop.v1";

/// E-stop message wire format (compact, no serde):
/// `[host_id_hash:u16][reason_len:u16][reason:bytes][timestamp_ns:u64]`
/// Total overhead: 12 bytes + reason string.
///
/// When `HORUS_ESTOP_KEY` is configured a 32-byte HMAC-SHA256 tag is appended:
/// `[...body...][tag:32]`. The tag covers `ESTOP_MAC_DOMAIN || body`, so
/// `reason_len` — which determines where the body ends — is itself
/// authenticated and cannot be shifted to relocate the tag.
///
/// Appending is backward compatible on the decode side: `decode_estop` reads
/// fixed offsets derived from `reason_len` and ignores trailing bytes.
///
/// Encode an e-stop message.
pub fn encode_estop(peer_id_hash: u16, reason: &str) -> Vec<u8> {
    encode_estop_with_key(peer_id_hash, reason, crate::mac::estop_key())
}

/// `encode_estop` with an explicit key.
///
/// The process key comes from a `OnceLock` over an env var, which a test cannot
/// vary. This seam lets the authenticated and unauthenticated paths both be
/// tested in one process.
pub fn encode_estop_with_key(peer_id_hash: u16, reason: &str, key: Option<&[u8; 32]>) -> Vec<u8> {
    let reason_bytes = reason.as_bytes();
    let reason_len = reason_bytes.len().min(200) as u16; // cap reason at 200 bytes
    let now_ns = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap_or_default()
        .as_nanos() as u64;

    let mut buf = Vec::with_capacity(12 + reason_len as usize + crate::mac::TAG_LEN);
    buf.extend_from_slice(&peer_id_hash.to_le_bytes());
    buf.extend_from_slice(&reason_len.to_le_bytes());
    buf.extend_from_slice(&reason_bytes[..reason_len as usize]);
    buf.extend_from_slice(&now_ns.to_le_bytes());

    if let Some(key) = key {
        let tag = estop_tag(key, &buf);
        buf.extend_from_slice(&tag);
    }
    buf
}

/// Compute the authentication tag over an e-stop body.
fn estop_tag(key: &[u8; 32], body: &[u8]) -> [u8; crate::mac::TAG_LEN] {
    let mut msg = Vec::with_capacity(ESTOP_MAC_DOMAIN.len() + body.len());
    msg.extend_from_slice(ESTOP_MAC_DOMAIN);
    msg.extend_from_slice(body);
    crate::mac::hmac_sha256(key, &msg)
}

/// Verify the tag on a received e-stop packet.
///
/// `body_len` is the length of the canonical body as computed from the decoded
/// `reason_len` — not from `data.len()`, which an attacker controls freely.
/// Returns `false` if the tag is absent, short, or wrong.
fn verify_estop_tag(key: &[u8; 32], data: &[u8], body_len: usize) -> bool {
    let Some(tag) = data.get(body_len..body_len + crate::mac::TAG_LEN) else {
        return false; // no tag present, or truncated
    };
    let expected = estop_tag(key, &data[..body_len]);
    crate::mac::ct_eq(tag, &expected)
}

/// Length of the canonical (MAC-covered) e-stop body, derived from the wire
/// `reason_len` rather than from `data.len()`.
///
/// Returns `None` for a packet too short to hold the body it declares.
fn estop_body_len(data: &[u8]) -> Option<usize> {
    if data.len() < 12 {
        return None;
    }
    let reason_len = u16::from_le_bytes([data[2], data[3]]) as usize;
    let body_len = 4 + reason_len + 8;
    if data.len() < body_len {
        return None;
    }
    Some(body_len)
}

/// Decode an e-stop message. Returns (host_id_hash, reason, timestamp_ns).
pub fn decode_estop(data: &[u8]) -> Option<(u16, String, u64)> {
    if data.len() < 12 {
        return None;
    }
    let host_id_hash = u16::from_le_bytes([data[0], data[1]]);
    let reason_len = u16::from_le_bytes([data[2], data[3]]) as usize;
    if data.len() < 4 + reason_len + 8 {
        return None;
    }
    let reason = String::from_utf8_lossy(&data[4..4 + reason_len]).to_string();
    let ts_offset = 4 + reason_len;
    let timestamp_ns = u64::from_le_bytes([
        data[ts_offset],
        data[ts_offset + 1],
        data[ts_offset + 2],
        data[ts_offset + 3],
        data[ts_offset + 4],
        data[ts_offset + 5],
        data[ts_offset + 6],
        data[ts_offset + 7],
    ]);
    Some((host_id_hash, reason, timestamp_ns))
}

/// Whether `data` carries a valid authentication tag for the configured key.
///
/// Exposed so the replicator can reject forged packets BEFORE they consume a
/// rate-limit token. Charging the limiter first let an attacker spend the
/// e-stop budget with packets that do nothing, so the next genuine halt was
/// dropped — turning a limiter meant to stop a forced halt into a way to
/// suppress a real one.
///
/// Returns `true` when no key is configured: there is nothing to verify, and the
/// caller keeps the original ordering in that case.
pub fn estop_packet_is_authentic(data: &[u8]) -> bool {
    let Some(key) = crate::mac::estop_key() else {
        return true;
    };
    let Some(body_len) = estop_body_len(data) else {
        return false;
    };
    verify_estop_tag(key, data, body_len)
}

/// Handle a received e-stop broadcast. May trigger the local emergency stop.
///
/// Returns `true` if this call acted on the e-stop (triggered the external stop),
/// `false` if it was rejected (stale/malformed) or suppressed by policy.
///
/// # Security posture — UNAUTHENTICATED (read before "hardening" this)
///
/// This channel is **not authenticated**. The only wire-level "identity" is the
/// discovery `secret_hash`, which is a non-cryptographic FNV-1a value broadcast in
/// CLEARTEXT for peer *filtering* (`config` says "NOT security"). Building e-stop
/// auth on it would be fake security — an attacker reads the hash off the wire and
/// replays it. This pass therefore deliberately does **not** fake authentication.
///
/// What it DOES provide (defense in depth, honestly scoped):
///   * **Freshness/replay reject** — a timestamp outside `[now-5s, now+5s]` is
///     dropped, so a captured e-stop cannot be replayed indefinitely.
///   * **Explicit policy gate** (`EstopRemotePolicy`, env `HORUS_ESTOP_REMOTE`):
///     `Warn` (default) acts on the e-stop but emits a LOUD one-time warning that
///     any LAN peer can halt the fleet; `Off` ignores remote e-stop entirely.
///     The LOCAL safety path (watchdog/deadline e-stop in horus_core) is
///     independent and unaffected by either setting.
///
/// Genuine authenticated networked e-stop needs a provisioned, off-wire key.
/// That key now exists: set `HORUS_ESTOP_KEY` (see `crate::mac`). When it is
/// set, every e-stop packet carries an HMAC-SHA256 tag and an untagged or
/// wrongly-tagged packet is **rejected outright**, regardless of policy —
/// this is the actual fix for GHSA-3frr-c2j9-hhr7. When it is unset, behaviour
/// is unchanged (`Warn` is loud-by-design, `Off` available) so an upgrade never
/// silently severs a fleet's existing e-stop path.
pub fn handle_remote_estop(data: &[u8], policy: EstopRemotePolicy) -> bool {
    handle_remote_estop_with_key(data, policy, crate::mac::estop_key())
}

/// `handle_remote_estop` with an explicit key. See `encode_estop_with_key`.
pub fn handle_remote_estop_with_key(
    data: &[u8],
    policy: EstopRemotePolicy,
    key: Option<&[u8; 32]>,
) -> bool {
    let now_ns = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map(|d| d.as_nanos() as u64)
        .unwrap_or(0);
    handle_remote_estop_at(data, policy, key, now_ns)
}

/// `handle_remote_estop_with_key` with an injected clock. See
/// `classify_estop_freshness_at` for why this seam exists.
pub fn handle_remote_estop_at(
    data: &[u8],
    policy: EstopRemotePolicy,
    key: Option<&[u8; 32]>,
    now_ns: u64,
) -> bool {
    let (host_id, reason, timestamp) = match decode_estop(data) {
        Some(v) => v,
        None => return false,
    };

    // Authentication gate. Only active when a key is provisioned; when active it
    // precedes every other check so a forged packet cannot even reach the logs.
    if let Some(key) = key {
        crate::mac::estop_authenticated(); // one-time "hardened path active" notice
        let Some(body_len) = estop_body_len(data) else {
            return false;
        };
        if !verify_estop_tag(key, data, body_len) {
            warn_unauthenticated_rejected();
            return false;
        }
    }

    // Freshness / replay guard (NOT authentication — see doc above).
    match classify_estop_freshness_at(timestamp, now_ns) {
        Freshness::Fresh => {}
        Freshness::Stale => {
            eprintln!(
                "[horus_net] Ignoring stale/implausible remote e-stop from host {:04x} \
                 (timestamp outside the {}s freshness window)",
                host_id,
                ESTOP_MAX_AGE_NS / 1_000_000_000
            );
            return false;
        }
        Freshness::ClockUnusable => {
            // The two clocks differ by more than a day, so this is not drift —
            // one of them is simply unset. Embedded controllers without a
            // battery-backed RTC boot at the epoch and sync only once NTP
            // reaches them, and this comparison is against ABSOLUTE wall time.
            //
            // Treating that as "stale" silently discarded EVERY remote e-stop
            // for as long as the clock was unset, which is the dangerous
            // direction of failure: a fleet-wide halt never reaches the node.
            //
            // If the packet is AUTHENTICATED, honour it. The MAC already proves
            // it came from the key holder, halting is the safe action, and the
            // authors' own rule for this channel is that "honoring a
            // slightly-skewed real one is safer than discarding it" — a
            // decades-skewed one is the same argument, more so.
            //
            // If it is NOT authenticated, keep rejecting — an unauthenticated
            // packet with an arbitrary timestamp is exactly the replay this
            // guard exists for — but say so loudly, because otherwise the
            // operator has no way to discover that their e-stop is inert.
            warn_estop_clock_unusable(host_id, key.is_some());
            if key.is_none() {
                return false;
            }
        }
    }

    // Policy gate.
    match policy {
        EstopRemotePolicy::Off => {
            // Remote e-stop suppressed. Local e-stop is unaffected.
            false
        }
        EstopRemotePolicy::Warn => {
            warn_unauthenticated_once();
            let msg = format!("Remote e-stop from host {:04x}: {}", host_id, reason);
            eprintln!("[horus_net] {}", msg);
            horus_core::scheduling::trigger_external_emergency_stop(msg);
            true
        }
    }
}

/// True if `timestamp_ns` (Unix nanoseconds) is within the freshness window of now.
/// Outcome of the freshness check, distinguishing "stale" from "this machine's
/// clock is not usable for comparison at all".
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Freshness {
    /// Within the replay window — act.
    Fresh,
    /// Outside the window, but both clocks look plausible: a genuine stale or
    /// replayed packet.
    Stale,
    /// The skew is far too large to be clock drift — one of the two clocks is
    /// simply not set. See `is_fresh_estop` for why this is not "stale".
    ClockUnusable,
}

/// Wall-clock values below this mean *this machine's* clock was never set.
///
/// 2020-01-01T00:00:00Z. A board with no battery-backed RTC reports 1970, or
/// 2000 on some hardware; no real deployment runs with a wall clock before 2020.
/// Deliberately a local-only test — it cannot be influenced by anything on the
/// wire, which is the entire point.
const CLOCK_UNSET_FLOOR_NS: u64 = 1_577_836_800_000_000_000;

/// True if this machine's wall clock is implausibly early, i.e. never set.
fn local_clock_is_unset(now_ns: u64) -> bool {
    now_ns < CLOCK_UNSET_FLOOR_NS
}

/// Skew beyond which the difference cannot be drift and must be an unset clock.
///
/// A day is far outside any plausible NTP/PTP error yet far inside the decades
/// of offset an unsynchronised clock produces.
const ESTOP_IMPLAUSIBLE_SKEW_NS: u64 = 24 * 60 * 60 * 1_000_000_000;

/// Classify `timestamp_ns` (Unix nanoseconds) against this machine's clock.
///
/// # Why this returns three states rather than a bool
///
/// The freshness window is a ±5s comparison of two **absolute wall clocks**, and
/// nothing in HORUS requires fleet clocks to be synchronised — the requirement
/// was not stated anywhere until this comment.
///
/// Embedded robot controllers frequently have no battery-backed RTC and boot at
/// or near the Unix epoch, syncing only once NTP reaches them. On such a node
/// `now_ns` is ~0 while a correct peer stamps ~1.8e18, so `timestamp_ns - now_ns`
/// is decades — vastly outside the 5s window. Treated as a bool, that made the
/// robot **silently discard every remote emergency stop** for as long as its
/// clock was unset, with no log line and no other symptom. The direction of that
/// failure is the dangerous one: a fleet-wide halt does not reach the node.
///
/// So a skew that cannot possibly be drift is reported as `ClockUnusable`
/// instead of `Stale`, and the caller decides — see `handle_remote_estop_with_key`.
#[cfg(test)]
fn classify_estop_freshness(timestamp_ns: u64) -> Freshness {
    let now_ns = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map(|d| d.as_nanos() as u64)
        .unwrap_or(0);
    classify_estop_freshness_at(timestamp_ns, now_ns)
}

/// `classify_estop_freshness` with an injected clock.
///
/// Test seam: the interesting cases are "this machine's clock is unset" and "a
/// day-old packet arrives at a healthy node", and neither is reachable through
/// `SystemTime::now()`. Without this the suite was structurally blind to the
/// replay band — the original tests used a one-hour-old timestamp, which lands
/// in `Stale`, and asserted the buggy `ClockUnusable` behaviour as correct.
fn classify_estop_freshness_at(timestamp_ns: u64, now_ns: u64) -> Freshness {
    let skew = now_ns.abs_diff(timestamp_ns);
    if skew >= ESTOP_IMPLAUSIBLE_SKEW_NS {
        // Only OUR OWN clock being unset justifies bypassing freshness.
        //
        // The first version of this decided from the skew MAGNITUDE alone, which
        // is symmetric and entirely attacker-controllable: it made every packet
        // older than a day `ClockUnusable`, and the caller then honoured any
        // authenticated one. The anti-replay window stopped being a window and
        // became a band — 5s..24h rejected, older than 24h ACCEPTED — so a
        // captured genuine e-stop became replayable forever from a day later,
        // and the HMAC could not help because the tag is the key holder's own.
        //
        // Worse, it inverted the hardening: an UNKEYED node still rejected the
        // replay, so provisioning HORUS_ESTOP_KEY — the recommended posture —
        // made a node MORE attackable, reintroducing the GHSA impact class
        // against exactly the deployments that had remediated it.
        //
        // Whether this machine can tell the time is a purely LOCAL question, so
        // ask it locally. A node whose clock is set treats a day-old packet as
        // what it is: stale.
        if local_clock_is_unset(now_ns) {
            return Freshness::ClockUnusable;
        }
        return Freshness::Stale;
    }
    let within = if now_ns >= timestamp_ns {
        // Past (or equal): reject if older than the max age (replay).
        now_ns - timestamp_ns <= ESTOP_MAX_AGE_NS
    } else {
        // Future: reject if further ahead than the skew tolerance (desync/garbage).
        timestamp_ns - now_ns <= ESTOP_MAX_FUTURE_SKEW_NS
    };
    if within {
        Freshness::Fresh
    } else {
        Freshness::Stale
    }
}

/// True if `timestamp_ns` is within the freshness window of now.
#[cfg(test)]
fn is_fresh_estop(timestamp_ns: u64) -> bool {
    classify_estop_freshness(timestamp_ns) == Freshness::Fresh
}

/// Emit the "networked e-stop is unauthenticated" warning at most once per process.
///
/// The reach claim here is deliberately not "this LAN": the replicator binds
/// `0.0.0.0` (`transport/udp.rs`), so the packet can come from any host able to
/// route a datagram to this port, not merely a link-local neighbour.
fn warn_unauthenticated_once() {
    if !ESTOP_UNAUTH_WARNED.swap(true, Ordering::Relaxed) {
        eprintln!(
            "[horus_net] WARNING: acting on a NETWORKED e-stop, which is \
             UNAUTHENTICATED — any host that can reach this UDP port (the socket \
             binds 0.0.0.0, so not just the local subnet) can halt the fleet. \
             Set HORUS_ESTOP_KEY on every node to authenticate e-stop, or \
             HORUS_ESTOP_REMOTE=off to ignore remote e-stop. (Fires once.)"
        );
    }
}

/// Emit the "this machine's clock cannot be compared" warning at most once.
///
/// Rate-limited like the others: the condition persists until NTP lands, so an
/// unlimited log would flood for the whole boot window.
fn warn_estop_clock_unusable(host_id: u16, authenticated: bool) {
    if !ESTOP_CLOCK_WARNED.swap(true, Ordering::Relaxed) {
        if authenticated {
            eprintln!(
                "[horus_net] WARNING: remote e-stop from host {:04x} differs from this \
                 machine's clock by more than a day — one of the two clocks is not set. \
                 ACTING on it anyway because it is authenticated (HORUS_ESTOP_KEY). \
                 Synchronise clocks (NTP/PTP): e-stop freshness compares ABSOLUTE wall \
                 time. (Fires once.)",
                host_id
            );
        } else {
            eprintln!(
                "[horus_net] WARNING: REJECTING remote e-stop from host {:04x} — its \
                 timestamp differs from this machine's clock by more than a day, so one \
                 of the two clocks is not set, and the packet is UNAUTHENTICATED. \
                 Remote e-stop is INERT on this node until clocks are synchronised \
                 (NTP/PTP) or HORUS_ESTOP_KEY is provisioned. (Fires once.)",
                host_id
            );
        }
    }
}

/// Emit the "rejected an unauthenticated e-stop" warning at most once per process.
///
/// Rate-limited to once because the whole point of the reject path is that an
/// attacker can drive it at line rate; logging every rejection would hand them a
/// disk-fill and an RT-jitter primitive.
fn warn_unauthenticated_rejected() {
    if !ESTOP_FORGERY_WARNED.swap(true, Ordering::Relaxed) {
        eprintln!(
            "[horus_net] WARNING: REJECTED a remote e-stop with a missing or invalid \
             authentication tag. Either a peer is misconfigured (HORUS_ESTOP_KEY must \
             match on every node) or someone is forging e-stop packets. (Fires once.)"
        );
    }
}

/// Broadcast e-stop with triple-redundant delivery.
///
/// Called by the replicator when local SafetyMonitor triggers e-stop.
/// Sends to: multicast group + each known peer unicast + 3× retry.
pub struct EstopBroadcaster {
    /// Pending retries: (payload, remaining_retries, next_retry_at)
    pending_retries: Vec<(Vec<u8>, u8, Instant)>,
}

const RETRY_COUNT: u8 = 3;
const RETRY_INTERVAL: Duration = Duration::from_millis(10);

impl Default for EstopBroadcaster {
    fn default() -> Self {
        Self::new()
    }
}

impl EstopBroadcaster {
    pub fn new() -> Self {
        Self {
            pending_retries: Vec::new(),
        }
    }

    /// Initiate triple-redundant e-stop broadcast.
    pub fn broadcast<T: Transport>(
        &mut self,
        transport: &T,
        payload: Vec<u8>,
        multicast_addr: Option<SocketAddr>,
        peer_addrs: &[SocketAddr],
    ) {
        // 1. Multicast (fast, reaches all LAN peers)
        if let Some(mcast) = multicast_addr {
            let _ = transport.send_to(&payload, mcast);
        }

        // 2. Unicast to each known peer (backup)
        for addr in peer_addrs {
            let _ = transport.send_to(&payload, *addr);
        }

        // 3. Schedule retries
        for i in 0..RETRY_COUNT {
            self.pending_retries.push((
                payload.clone(),
                RETRY_COUNT - i,
                Instant::now() + RETRY_INTERVAL * (i as u32 + 1),
            ));
        }
    }

    /// Process pending retries. Called on each timer tick.
    pub fn tick<T: Transport>(
        &mut self,
        transport: &T,
        multicast_addr: Option<SocketAddr>,
        peer_addrs: &[SocketAddr],
    ) {
        let now = Instant::now();
        let mut i = 0;
        while i < self.pending_retries.len() {
            if now >= self.pending_retries[i].2 {
                let (ref payload, _, _) = self.pending_retries[i];
                // Resend to multicast + all peers
                if let Some(mcast) = multicast_addr {
                    let _ = transport.send_to(payload, mcast);
                }
                for addr in peer_addrs {
                    let _ = transport.send_to(payload, *addr);
                }
                self.pending_retries.swap_remove(i);
            } else {
                i += 1;
            }
        }
    }

    /// Clear all pending retries (on shutdown).
    pub fn clear(&mut self) {
        self.pending_retries.clear();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn now_ns() -> u64 {
        SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .unwrap()
            .as_nanos() as u64
    }

    /// Overwrite the trailing u64 timestamp of an encoded e-stop payload.
    fn with_timestamp(mut payload: Vec<u8>, ts_ns: u64) -> Vec<u8> {
        let off = payload.len() - 8;
        payload[off..].copy_from_slice(&ts_ns.to_le_bytes());
        payload
    }

    #[test]
    fn fresh_estop_warn_triggers() {
        let payload = encode_estop(0xABCD, "test"); // fresh timestamp
        assert!(handle_remote_estop(&payload, EstopRemotePolicy::Warn));
    }

    #[test]
    fn off_policy_ignores_remote_estop() {
        let payload = encode_estop(0xABCD, "test");
        assert!(!handle_remote_estop(&payload, EstopRemotePolicy::Off));
    }

    #[test]
    fn stale_timestamp_rejected() {
        // 1s after epoch → decades old → outside the freshness window.
        let payload = with_timestamp(encode_estop(0x1111, "old"), 1_000_000_000);
        assert!(!handle_remote_estop(&payload, EstopRemotePolicy::Warn));
    }

    #[test]
    fn far_future_timestamp_rejected() {
        let payload = with_timestamp(encode_estop(0x2222, "future"), now_ns() + 60_000_000_000);
        assert!(!handle_remote_estop(&payload, EstopRemotePolicy::Warn));
    }

    #[test]
    fn malformed_estop_rejected() {
        assert!(!handle_remote_estop(&[], EstopRemotePolicy::Warn));
        assert!(!handle_remote_estop(&[0u8; 4], EstopRemotePolicy::Warn));
    }

    #[test]
    fn freshness_window_boundaries() {
        let now = now_ns();
        assert!(is_fresh_estop(now));
        assert!(is_fresh_estop(now.saturating_sub(1_000_000_000))); // 1s old: fresh
        assert!(!is_fresh_estop(now.saturating_sub(10_000_000_000))); // 10s old: stale
        assert!(!is_fresh_estop(now + 10_000_000_000)); // 10s future: reject
    }

    // ── SEND seam: EstopBroadcaster actually emits packets ────────────────────

    /// Mock `Transport` that records every `(payload, addr)` handed to `send_to`,
    /// so the SEND path can be asserted without a real socket.
    struct MockTransport {
        sent: std::sync::Mutex<Vec<(Vec<u8>, SocketAddr)>>,
    }

    impl MockTransport {
        fn new() -> Self {
            Self {
                sent: std::sync::Mutex::new(Vec::new()),
            }
        }
    }

    impl Transport for MockTransport {
        fn send_to(&self, data: &[u8], addr: SocketAddr) -> std::io::Result<usize> {
            self.sent
                .lock()
                .unwrap_or_else(|e| e.into_inner())
                .push((data.to_vec(), addr));
            Ok(data.len())
        }
        fn recv_from(&self, _buf: &mut [u8]) -> std::io::Result<(usize, SocketAddr)> {
            Err(std::io::Error::new(
                std::io::ErrorKind::WouldBlock,
                "mock: no data",
            ))
        }
        fn local_addr(&self) -> std::io::Result<SocketAddr> {
            Ok(SocketAddr::from(([127, 0, 0, 1], 0)))
        }
        fn join_multicast(&self, _group: &str) -> std::io::Result<()> {
            Ok(())
        }
        #[cfg(unix)]
        fn raw_fd(&self) -> std::os::unix::io::RawFd {
            -1
        }
    }

    /// `EstopBroadcaster::broadcast` must emit at least one outbound packet that
    /// `decode_estop`s back to the given (host, reason). This is the horus_net half of
    /// the wired SEND path.
    ///
    /// The replicator's `handle_timer` call site is the 3-line wiring
    /// (`take_pending_local_estop` → `encode_estop` → `broadcast`) verified by
    /// inspection; real cross-process delivery RESTS-ON-DESIGN (sandbox cross-process
    /// UDP/SHM is env-broken, same as NET-F1's gate test).
    #[test]
    fn broadcast_emits_decodable_estop_packet() {
        let transport = MockTransport::new();
        let mut bc = EstopBroadcaster::new();
        let peer = SocketAddr::from(([127, 0, 0, 1], 7447));

        let reason = "local watchdog expired";
        let payload = encode_estop(0xBEEF, reason);
        // No multicast addr in this seam test — unicast to the one fake peer is enough.
        bc.broadcast(&transport, payload, None, &[peer]);

        let sent = transport.sent.lock().unwrap_or_else(|e| e.into_inner());
        assert!(
            !sent.is_empty(),
            "broadcast must emit at least one outbound packet"
        );
        assert!(
            sent.iter().any(|(pkt, _addr)| matches!(
                decode_estop(pkt),
                Some((host, ref r, _ts)) if host == 0xBEEF && r == reason
            )),
            "at least one sent packet must decode_estop() to (0xBEEF, reason)"
        );
    }

    // ─── Authenticated e-stop (GHSA-3frr-c2j9-hhr7 remediation) ─────────────
    //
    // These use the `_with_key` seams so both postures are exercised in one
    // process; the production entry points read the same key from a OnceLock.

    const KEY_A: [u8; 32] = [0x11; 32];
    const KEY_B: [u8; 32] = [0x22; 32];

    #[test]
    fn keyed_estop_round_trips() {
        let pkt = encode_estop_with_key(0x1234, "keyed halt", Some(&KEY_A));
        assert!(
            handle_remote_estop_with_key(&pkt, EstopRemotePolicy::Warn, Some(&KEY_A)),
            "a correctly tagged e-stop must still halt"
        );
    }

    #[test]
    fn keyed_receiver_rejects_untagged_packet() {
        // This is the actual advisory scenario: an attacker who has read the
        // wire format sends a well-formed but untagged e-stop.
        let forged = encode_estop_with_key(0xDEAD, "forged halt", None);
        assert!(
            !handle_remote_estop_with_key(&forged, EstopRemotePolicy::Warn, Some(&KEY_A)),
            "an untagged e-stop must be rejected once a key is provisioned"
        );
    }

    #[test]
    fn keyed_receiver_rejects_wrong_key() {
        let pkt = encode_estop_with_key(0x1234, "halt", Some(&KEY_B));
        assert!(
            !handle_remote_estop_with_key(&pkt, EstopRemotePolicy::Warn, Some(&KEY_A)),
            "a tag from a different key must not verify"
        );
    }

    #[test]
    fn keyed_receiver_rejects_flipped_tag_bit() {
        let mut pkt = encode_estop_with_key(0x1234, "halt", Some(&KEY_A));
        let last = pkt.len() - 1;
        pkt[last] ^= 0x01;
        assert!(
            !handle_remote_estop_with_key(&pkt, EstopRemotePolicy::Warn, Some(&KEY_A)),
            "a single flipped tag bit must fail verification"
        );
    }

    #[test]
    fn keyed_receiver_rejects_tampered_reason() {
        // The reason is inside the MAC-covered body, so editing it in flight
        // must invalidate the tag rather than change what gets logged.
        let mut pkt = encode_estop_with_key(0x1234, "halt", Some(&KEY_A));
        pkt[4] = b'H'; // first byte of "halt"
        assert!(!handle_remote_estop_with_key(
            &pkt,
            EstopRemotePolicy::Warn,
            Some(&KEY_A)
        ));
    }

    #[test]
    fn keyed_receiver_rejects_truncated_tag() {
        let mut pkt = encode_estop_with_key(0x1234, "halt", Some(&KEY_A));
        pkt.truncate(pkt.len() - 1);
        assert!(
            !handle_remote_estop_with_key(&pkt, EstopRemotePolicy::Warn, Some(&KEY_A)),
            "a short tag must be rejected, not read out of bounds"
        );
    }

    #[test]
    fn keyed_receiver_rejects_reason_len_shifted_to_relocate_tag() {
        // reason_len decides where the body ends and the tag begins. If it were
        // not itself authenticated, an attacker could shrink it so that some of
        // the original reason bytes are reinterpreted as the tag. It is inside
        // the MAC, so this must fail.
        let mut pkt = encode_estop_with_key(0x1234, "a-longer-reason-string", Some(&KEY_A));
        pkt[2] = 4; // claim reason_len = 4
        pkt[3] = 0;
        assert!(!handle_remote_estop_with_key(
            &pkt,
            EstopRemotePolicy::Warn,
            Some(&KEY_A)
        ));
    }

    #[test]
    fn keyed_receiver_rejects_oversized_reason_len_without_panicking() {
        // u16::MAX reason_len against a short buffer — must return false, not
        // index out of bounds on the network thread.
        let mut pkt = encode_estop_with_key(0x1234, "halt", Some(&KEY_A));
        pkt[2] = 0xFF;
        pkt[3] = 0xFF;
        assert!(!handle_remote_estop_with_key(
            &pkt,
            EstopRemotePolicy::Warn,
            Some(&KEY_A)
        ));
    }

    #[test]
    fn unkeyed_receiver_still_accepts_untagged_packet() {
        // Backward compatibility: upgrading one node must not sever a fleet's
        // existing e-stop path. Without a key, behaviour is exactly as before.
        let pkt = encode_estop_with_key(0x1234, "halt", None);
        assert!(handle_remote_estop_with_key(
            &pkt,
            EstopRemotePolicy::Warn,
            None
        ));
    }

    #[test]
    fn unkeyed_receiver_accepts_tagged_packet() {
        // A hardened sender talking to a not-yet-upgraded receiver: the trailing
        // tag is simply ignored, so a rolling upgrade does not drop e-stops.
        let pkt = encode_estop_with_key(0x1234, "halt", Some(&KEY_A));
        assert!(handle_remote_estop_with_key(
            &pkt,
            EstopRemotePolicy::Warn,
            None
        ));
    }

    #[test]
    fn policy_off_still_wins_over_a_valid_tag() {
        let pkt = encode_estop_with_key(0x1234, "halt", Some(&KEY_A));
        assert!(
            !handle_remote_estop_with_key(&pkt, EstopRemotePolicy::Off, Some(&KEY_A)),
            "HORUS_ESTOP_REMOTE=off must suppress even an authenticated e-stop"
        );
    }

    #[test]
    fn stale_timestamp_still_rejected_when_keyed() {
        // Authentication must not replace the anti-replay window. A tag captured
        // an hour ago is still a valid tag.
        let pkt = encode_estop_with_key(0x1234, "halt", Some(&KEY_A));
        let body_len = estop_body_len(&pkt).unwrap();
        let mut stale = pkt[..body_len].to_vec();
        let off = body_len - 8;
        stale[off..].copy_from_slice(&(now_ns() - 3_600_000_000_000).to_le_bytes());
        // Re-tag so the packet is authentic but old.
        let tag = estop_tag(&KEY_A, &stale);
        stale.extend_from_slice(&tag);
        assert!(
            !handle_remote_estop_with_key(&stale, EstopRemotePolicy::Warn, Some(&KEY_A)),
            "freshness window must still apply to authenticated packets"
        );
    }

    // ─── Unsynchronised clocks (safety-critical) ────────────────────────────

    /// Timestamp an embedded controller with no battery-backed RTC produces
    /// before NTP lands: at/near the Unix epoch.
    const UNSET_CLOCK_TS: u64 = 1_000_000_000; // ~1 second after the epoch

    #[test]
    fn a_decades_skewed_packet_is_stale_when_our_own_clock_is_set() {
        // This test previously asserted the OPPOSITE and was wrong. Deciding
        // "clock unusable" from the skew magnitude is attacker-controllable: it
        // turned every packet older than a day into an accept, so a captured
        // genuine e-stop was replayable forever from 24h later.
        let now = now_ns(); // a real, set clock
        assert_eq!(
            classify_estop_freshness_at(UNSET_CLOCK_TS, now),
            Freshness::Stale,
            "with OUR clock set, an ancient packet is a replay, not a clock problem"
        );
        assert_eq!(
            classify_estop_freshness_at(now - 60_000_000_000, now),
            Freshness::Stale
        );
        assert_eq!(classify_estop_freshness_at(now, now), Freshness::Fresh);
    }

    #[test]
    fn clock_unusable_requires_our_clock_to_be_unset() {
        // The only case that legitimately bypasses freshness: this machine
        // cannot tell the time. Simulated by injecting an epoch-era `now`.
        let unset_now = 2_000_000_000; // ~2s after the epoch
        let real_ts = now_ns();
        assert_eq!(
            classify_estop_freshness_at(real_ts, unset_now),
            Freshness::ClockUnusable,
            "a node whose own clock is unset cannot judge freshness"
        );
        assert!(local_clock_is_unset(unset_now));
        assert!(!local_clock_is_unset(real_ts));
    }

    #[test]
    fn a_captured_estop_is_not_replayable_against_a_healthy_node() {
        // THE REGRESSION THIS FIX CLOSES. An attacker captures one genuine,
        // correctly-tagged e-stop off the LAN (they are multicast plus unicast
        // plus 3 retries, so a single halt puts many identical copies on the
        // wire) and re-sends it a day later. The HMAC cannot help — the tag is
        // the key holder's own — so freshness is the only defence, and the
        // magnitude-based version accepted it.
        let mut pkt = encode_estop_with_key(0x1234, "genuine fleet halt", Some(&KEY_A));
        let body_len = estop_body_len(&pkt).unwrap();
        let off = body_len - 8;
        pkt.truncate(body_len);
        // Stamp it as authored 48 hours ago, then tag it as the key holder would.
        let authored = now_ns() - 48 * 3_600 * 1_000_000_000u64;
        pkt[off..].copy_from_slice(&authored.to_le_bytes());
        let tag = estop_tag(&KEY_A, &pkt);
        pkt.extend_from_slice(&tag);

        assert!(
            !handle_remote_estop_at(&pkt, EstopRemotePolicy::Warn, Some(&KEY_A), now_ns()),
            "a 48h-old authenticated e-stop replayed at a healthy node must be REJECTED — \
             accepting it reintroduces the GHSA impact class against exactly the \
             deployments that provisioned HORUS_ESTOP_KEY"
        );
    }

    #[test]
    fn authenticated_estop_still_halts_when_our_clock_is_unset() {
        // The original bug this whole branch of logic exists for: a robot whose
        // RTC has not synced discarded every remote e-stop. It must still halt —
        // but only because OUR clock is unset, which we now establish locally.
        let pkt = encode_estop_with_key(0x1234, "fleet halt", Some(&KEY_A));
        let unset_now = 2_000_000_000; // epoch-era: our clock is not set
        assert!(
            handle_remote_estop_at(&pkt, EstopRemotePolicy::Warn, Some(&KEY_A), unset_now),
            "an authenticated e-stop must halt a node that cannot tell the time"
        );
    }

    #[test]
    fn unauthenticated_estop_with_an_unset_clock_is_still_rejected() {
        // Without a MAC, an arbitrary timestamp is exactly the replay the
        // freshness guard exists for — keep rejecting, but the operator now
        // gets a log line saying why.
        let pkt = encode_estop_with_key(0x1234, "halt", None);
        let unset_now = 2_000_000_000;
        assert!(!handle_remote_estop_at(
            &pkt,
            EstopRemotePolicy::Warn,
            None,
            unset_now
        ));
    }

    #[test]
    fn policy_off_still_wins_over_an_unset_clock() {
        let pkt = encode_estop_with_key(0x1234, "halt", Some(&KEY_A));
        let unset_now = 2_000_000_000;
        assert!(!handle_remote_estop_at(
            &pkt,
            EstopRemotePolicy::Off,
            Some(&KEY_A),
            unset_now
        ));
    }
}
