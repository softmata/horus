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
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

use crate::transport::Transport;

/// E-stop message wire format (compact, no serde):
/// [host_id_hash:u16][reason_len:u16][reason:bytes][timestamp_ns:u64]
/// Total overhead: 12 bytes + reason string.

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

/// Handle a received e-stop broadcast. Triggers local emergency stop.
pub fn handle_remote_estop(data: &[u8]) {
    if let Some((host_id, reason, _timestamp)) = decode_estop(data) {
        let msg = format!(
            "Remote e-stop from host {:04x}: {}",
            host_id, reason
        );
        eprintln!("[horus_net] {}", msg);
        horus_core::scheduling::trigger_external_emergency_stop(msg);
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
