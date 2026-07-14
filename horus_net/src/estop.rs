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

/// E-stop message wire format (compact, no serde):
/// [host_id_hash:u16][reason_len:u16][reason:bytes][timestamp_ns:u64]
/// Total overhead: 12 bytes + reason string.
///
/// Encode an e-stop message.
pub fn encode_estop(peer_id_hash: u16, reason: &str) -> Vec<u8> {
    let reason_bytes = reason.as_bytes();
    let reason_len = reason_bytes.len().min(200) as u16; // cap reason at 200 bytes
    let now_ns = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap_or_default()
        .as_nanos() as u64;

    let mut buf = Vec::with_capacity(12 + reason_len as usize);
    buf.extend_from_slice(&peer_id_hash.to_le_bytes());
    buf.extend_from_slice(&reason_len.to_le_bytes());
    buf.extend_from_slice(&reason_bytes[..reason_len as usize]);
    buf.extend_from_slice(&now_ns.to_le_bytes());
    buf
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
/// Genuine authenticated networked e-stop needs a future provisioned, off-wire
/// key (planned `HORUS_ESTOP_KEY`) used to MAC each e-stop packet. Until that
/// exists, `Warn` is loud-by-design and `Off` is available for hardened fleets.
pub fn handle_remote_estop(data: &[u8], policy: EstopRemotePolicy) -> bool {
    let (host_id, reason, timestamp) = match decode_estop(data) {
        Some(v) => v,
        None => return false,
    };

    // Freshness / replay guard (NOT authentication — see doc above).
    if !is_fresh_estop(timestamp) {
        eprintln!(
            "[horus_net] Ignoring stale/implausible remote e-stop from host {:04x} \
             (timestamp outside freshness window)",
            host_id
        );
        return false;
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
fn is_fresh_estop(timestamp_ns: u64) -> bool {
    let now_ns = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map(|d| d.as_nanos() as u64)
        .unwrap_or(0);
    if now_ns >= timestamp_ns {
        // Past (or equal): reject if older than the max age (replay).
        now_ns - timestamp_ns <= ESTOP_MAX_AGE_NS
    } else {
        // Future: reject if further ahead than the skew tolerance (desync/garbage).
        timestamp_ns - now_ns <= ESTOP_MAX_FUTURE_SKEW_NS
    }
}

/// Emit the "networked e-stop is unauthenticated" warning at most once per process.
fn warn_unauthenticated_once() {
    if !ESTOP_UNAUTH_WARNED.swap(true, Ordering::Relaxed) {
        eprintln!(
            "[horus_net] WARNING: acting on a NETWORKED e-stop, which is \
             UNAUTHENTICATED — any peer on this LAN can halt the fleet. Set \
             HORUS_ESTOP_REMOTE=off to ignore remote e-stop. (This warning fires once.)"
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
