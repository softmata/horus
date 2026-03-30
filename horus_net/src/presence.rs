//! Presence replication — broadcast local node presence to remote peers.
//!
//! Every 1 second, the replicator serializes local presence data and broadcasts
//! it on the `_horus.presence` system topic. Remote machines receive this and
//! write synthetic presence files so `horus node list` shows remote nodes.
//!
//! Liveness: heartbeat-based (last_seen_ns timestamp), NOT PID-based.
//! Graduated staleness: reachable (0-3s) → unreachable (3-30s) → dead (>30s, removed).
//!
//! Encoding: simple length-prefixed strings (no serde dependency).

use std::collections::HashMap;
use std::path::PathBuf;
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

/// Staleness thresholds.
const DEAD_TIMEOUT: Duration = Duration::from_secs(30);

/// Manages incoming remote presence data — writes presence files to disk.
pub struct PresenceReceiver {
    /// Active remote host files: host_id → (file_path, last_seen)
    active_hosts: HashMap<String, (PathBuf, Instant)>,
    /// Local namespace for filtering
    local_namespace: String,
}

impl PresenceReceiver {
    pub fn new() -> Self {
        Self {
            active_hosts: HashMap::new(),
            local_namespace: std::env::var("HORUS_NAMESPACE")
                .unwrap_or_else(|_| "default".to_string()),
        }
    }

    /// Process an incoming presence broadcast payload.
    /// Writes/updates a remote presence file in shm_nodes_dir().
    pub fn handle_broadcast(&mut self, data: &[u8]) {
        // Decode: [namespace_len:u16][namespace][host_id_len:u16][host_id][node_count:u16][nodes...]
        let mut pos = 0;
        if data.len() < 6 {
            return;
        }

        // Namespace
        let ns_len = u16::from_le_bytes([data[pos], data[pos + 1]]) as usize;
        pos += 2;
        if pos + ns_len > data.len() {
            return;
        }
        let namespace = String::from_utf8_lossy(&data[pos..pos + ns_len]).to_string();
        pos += ns_len;

        // Namespace isolation
        if namespace != self.local_namespace {
            return;
        }

        // Host ID
        if pos + 2 > data.len() {
            return;
        }
        let hid_len = u16::from_le_bytes([data[pos], data[pos + 1]]) as usize;
        pos += 2;
        if pos + hid_len > data.len() {
            return;
        }
        let host_id = String::from_utf8_lossy(&data[pos..pos + hid_len]).to_string();
        pos += hid_len;

        // Timestamp
        if pos + 8 > data.len() {
            return;
        }
        let timestamp_ns = u64::from_le_bytes([
            data[pos],
            data[pos + 1],
            data[pos + 2],
            data[pos + 3],
            data[pos + 4],
            data[pos + 5],
            data[pos + 6],
            data[pos + 7],
        ]);
        pos += 8;

        // Node count
        if pos + 2 > data.len() {
            return;
        }
        let node_count = u16::from_le_bytes([data[pos], data[pos + 1]]) as usize;
        pos += 2;

        // Build JSON presence file content (discovery reads JSON)
        let mut nodes_json = Vec::new();
        for _ in 0..node_count {
            if pos + 2 > data.len() {
                break;
            }
            let name_len = u16::from_le_bytes([data[pos], data[pos + 1]]) as usize;
            pos += 2;
            if pos + name_len > data.len() {
                break;
            }
            let name = String::from_utf8_lossy(&data[pos..pos + name_len]).to_string();
            pos += name_len;

            // Rate Hz (f64)
            if pos + 8 > data.len() {
                break;
            }
            let rate_bits = u64::from_le_bytes([
                data[pos],
                data[pos + 1],
                data[pos + 2],
                data[pos + 3],
                data[pos + 4],
                data[pos + 5],
                data[pos + 6],
                data[pos + 7],
            ]);
            let rate_hz = f64::from_bits(rate_bits);
            pos += 8;

            nodes_json.push(format!(
                r#"{{"name":"{}","rate_hz":{}}}"#,
                name,
                if rate_hz.is_finite() {
                    format!("{:.1}", rate_hz)
                } else {
                    "null".to_string()
                }
            ));
        }

        let nodes_dir = horus_sys::shm::shm_nodes_dir();
        let _ = std::fs::create_dir_all(&nodes_dir);

        let file_path = nodes_dir.join(format!("remote_{}.json", host_id));

        // Write JSON presence file
        let json = format!(
            r#"{{"host_id":"{}","is_remote":true,"last_seen_ns":{},"namespace":"{}","nodes":[{}]}}"#,
            host_id,
            timestamp_ns,
            namespace,
            nodes_json.join(",")
        );

        // Atomic write
        let tmp_path = file_path.with_extension("json.tmp");
        if std::fs::write(&tmp_path, json.as_bytes()).is_ok() {
            let _ = std::fs::rename(&tmp_path, &file_path);
        }

        self.active_hosts
            .insert(host_id, (file_path, Instant::now()));
    }

    /// Clean up stale remote presence files (>30s since last broadcast).
    pub fn cleanup_stale(&mut self) {
        let now = Instant::now();
        let mut to_remove = Vec::new();

        for (host_id, (path, last_seen)) in &self.active_hosts {
            if now.duration_since(*last_seen) > DEAD_TIMEOUT {
                let _ = std::fs::remove_file(path);
                to_remove.push(host_id.clone());
            }
        }

        for host_id in to_remove {
            self.active_hosts.remove(&host_id);
        }
    }

    /// Remove all remote presence files (on shutdown).
    pub fn cleanup_all(&mut self) {
        for (_host_id, (path, _)) in self.active_hosts.drain() {
            let _ = std::fs::remove_file(&path);
        }
    }
}

/// Build a presence broadcast payload from local node presence.
///
/// Wire format: [namespace_len:u16][namespace][host_id_len:u16][host_id]
///              [timestamp_ns:u64][node_count:u16]
///              [name_len:u16][name][rate_hz:f64] × node_count
///
/// Returns None if no local nodes exist.
pub fn build_local_presence(peer_id_hash: u16) -> Option<Vec<u8>> {
    let namespace =
        std::env::var("HORUS_NAMESPACE").unwrap_or_else(|_| "default".to_string());

    let presences = horus_core::NodePresence::read_all();
    if presences.is_empty() {
        return None;
    }

    let host_id = format!("{:04x}", peer_id_hash);

    let now_ns = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap_or_default()
        .as_nanos() as u64;

    let mut buf = Vec::with_capacity(512);

    // Namespace
    let ns_bytes = namespace.as_bytes();
    buf.extend_from_slice(&(ns_bytes.len() as u16).to_le_bytes());
    buf.extend_from_slice(ns_bytes);

    // Host ID
    let hid_bytes = host_id.as_bytes();
    buf.extend_from_slice(&(hid_bytes.len() as u16).to_le_bytes());
    buf.extend_from_slice(hid_bytes);

    // Timestamp
    buf.extend_from_slice(&now_ns.to_le_bytes());

    // Node count
    buf.extend_from_slice(&(presences.len() as u16).to_le_bytes());

    // Nodes
    for p in &presences {
        let name = p.name();
        let name_bytes = name.as_bytes();
        buf.extend_from_slice(&(name_bytes.len() as u16).to_le_bytes());
        buf.extend_from_slice(name_bytes);

        let rate_hz = p.rate_hz().unwrap_or(0.0);
        buf.extend_from_slice(&rate_hz.to_bits().to_le_bytes());
    }

    Some(buf)
}
