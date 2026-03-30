//! Log replication — broadcast local log entries to remote peers.
//!
//! Polls GLOBAL_LOG_BUFFER for new entries, filters by severity
//! (default: Error + Warning only), batches, and broadcasts on `_horus.logs`.
//! On receive: writes to GLOBAL_REMOTE_LOG_BUFFER with host_id prefix.
//!
//! Zero serde dependency — uses length-prefixed binary encoding.

use std::time::Instant;

/// Default: only replicate Error and Warning log entries.
/// Override with HORUS_NET_LOG_LEVEL=info (or debug/trace) for more.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum LogReplicationLevel {
    None,    // Don't replicate any logs
    Error,   // Errors only
    Warning, // Errors + Warnings (default)
    Info,    // Errors + Warnings + Info
    Debug,   // Everything
}

impl LogReplicationLevel {
    pub fn from_env() -> Self {
        match std::env::var("HORUS_NET_LOG_LEVEL")
            .unwrap_or_default()
            .to_lowercase()
            .as_str()
        {
            "none" | "off" => Self::None,
            "error" => Self::Error,
            "warn" | "warning" => Self::Warning,
            "info" => Self::Info,
            "debug" | "trace" | "all" => Self::Debug,
            _ => Self::Warning, // default
        }
    }

    /// Check if a log type should be replicated at this level.
    pub fn should_replicate(&self, log_type: &horus_core::core::log_buffer::LogType) -> bool {
        use horus_core::core::log_buffer::LogType;
        match self {
            Self::None => false,
            Self::Error => matches!(log_type, LogType::Error),
            Self::Warning => matches!(log_type, LogType::Error | LogType::Warning),
            Self::Info => matches!(log_type, LogType::Error | LogType::Warning | LogType::Info),
            Self::Debug => true,
        }
    }
}

/// Drains new entries from GLOBAL_LOG_BUFFER and batches them for broadcast.
pub struct LogDrain {
    /// Last seen write_idx in GLOBAL_LOG_BUFFER.
    last_write_idx: u64,
    /// Minimum severity to replicate.
    level: LogReplicationLevel,
    /// Batch buffer (cleared after each broadcast).
    batch: Vec<u8>,
    /// Number of entries in current batch.
    batch_count: u16,
    /// Last broadcast time (for batching interval).
    last_broadcast: Instant,
    /// Host ID for prefixing node names.
    host_id: String,
}

/// Maximum entries per broadcast batch.
const MAX_BATCH_SIZE: u16 = 64;
/// Maximum time between broadcasts (even if batch not full).
const BATCH_INTERVAL_MS: u64 = 100;

impl LogDrain {
    pub fn new(peer_id_hash: u16) -> Self {
        Self {
            last_write_idx: horus_core::core::log_buffer::GLOBAL_LOG_BUFFER.write_idx(),
            level: LogReplicationLevel::from_env(),
            batch: Vec::with_capacity(4096),
            batch_count: 0,
            last_broadcast: Instant::now(),
            host_id: format!("{:04x}", peer_id_hash),
        }
    }

    /// Poll for new log entries. Returns encoded batch if ready to broadcast.
    /// Called on every timer tick (50ms).
    pub fn poll(&mut self) -> Option<Vec<u8>> {
        if self.level == LogReplicationLevel::None {
            return None;
        }

        let current_idx = horus_core::core::log_buffer::GLOBAL_LOG_BUFFER.write_idx();
        if current_idx > self.last_write_idx {
            // New entries available — read and filter
            let entries = horus_core::core::log_buffer::GLOBAL_LOG_BUFFER.get_all();
            for entry in &entries {
                if entry.tick_number <= self.last_write_idx {
                    continue; // Already seen
                }
                if !self.level.should_replicate(&entry.log_type) {
                    continue; // Below replication threshold
                }
                self.encode_entry(entry);
            }
            self.last_write_idx = current_idx;
        }

        // Flush batch if full or interval elapsed
        let should_flush = self.batch_count >= MAX_BATCH_SIZE
            || (self.batch_count > 0
                && self.last_broadcast.elapsed().as_millis() >= BATCH_INTERVAL_MS as u128);

        if should_flush {
            let mut payload = Vec::with_capacity(2 + 2 + self.host_id.len() + self.batch.len());
            // Header: [host_id_len:u16][host_id][entry_count:u16][entries...]
            let hid = self.host_id.as_bytes();
            payload.extend_from_slice(&(hid.len() as u16).to_le_bytes());
            payload.extend_from_slice(hid);
            payload.extend_from_slice(&self.batch_count.to_le_bytes());
            payload.append(&mut self.batch);
            self.batch_count = 0;
            self.last_broadcast = Instant::now();
            Some(payload)
        } else {
            None
        }
    }

    /// Encode a single log entry into the batch buffer.
    /// Format: [node_name_len:u16][node_name][level:u8][msg_len:u16][message]
    fn encode_entry(&mut self, entry: &horus_core::core::log_buffer::LogEntry) {
        let name = entry.node_name.as_bytes();
        let msg = entry.message.as_bytes();
        let level = match entry.log_type {
            horus_core::core::log_buffer::LogType::Error => 4u8,
            horus_core::core::log_buffer::LogType::Warning => 3,
            horus_core::core::log_buffer::LogType::Info => 2,
            horus_core::core::log_buffer::LogType::Debug => 1,
            _ => 0,
        };

        self.batch
            .extend_from_slice(&(name.len() as u16).to_le_bytes());
        self.batch.extend_from_slice(name);
        self.batch.push(level);
        self.batch
            .extend_from_slice(&(msg.len() as u16).to_le_bytes());
        self.batch.extend_from_slice(msg);
        self.batch_count += 1;
    }
}

/// Write received remote log entries into GLOBAL_REMOTE_LOG_BUFFER.
pub fn handle_remote_logs(data: &[u8]) {
    use horus_core::core::log_buffer::{LogEntry, LogType, GLOBAL_REMOTE_LOG_BUFFER};

    let mut pos = 0;
    if data.len() < 4 {
        return;
    }

    // Host ID
    let hid_len = u16::from_le_bytes([data[pos], data[pos + 1]]) as usize;
    pos += 2;
    if pos + hid_len > data.len() {
        return;
    }
    let host_id = String::from_utf8_lossy(&data[pos..pos + hid_len]).to_string();
    pos += hid_len;

    // Entry count
    if pos + 2 > data.len() {
        return;
    }
    let count = u16::from_le_bytes([data[pos], data[pos + 1]]) as usize;
    pos += 2;

    for _ in 0..count {
        // Node name
        if pos + 2 > data.len() {
            break;
        }
        let name_len = u16::from_le_bytes([data[pos], data[pos + 1]]) as usize;
        pos += 2;
        if pos + name_len > data.len() {
            break;
        }
        let node_name = String::from_utf8_lossy(&data[pos..pos + name_len]).to_string();
        pos += name_len;

        // Level
        if pos >= data.len() {
            break;
        }
        let level = data[pos];
        pos += 1;

        // Message
        if pos + 2 > data.len() {
            break;
        }
        let msg_len = u16::from_le_bytes([data[pos], data[pos + 1]]) as usize;
        pos += 2;
        if pos + msg_len > data.len() {
            break;
        }
        let message = String::from_utf8_lossy(&data[pos..pos + msg_len]).to_string();
        pos += msg_len;

        let log_type = match level {
            4 => LogType::Error,
            3 => LogType::Warning,
            2 => LogType::Info,
            1 => LogType::Debug,
            _ => LogType::Info,
        };

        // Prefix node_name with host_id for attribution
        let prefixed_name = format!("{}/{}", host_id, node_name);

        GLOBAL_REMOTE_LOG_BUFFER.push(LogEntry {
            timestamp: String::new(), // Receiver doesn't know original timestamp
            tick_number: 0,
            node_name: prefixed_name,
            log_type,
            topic: None,
            message,
            tick_us: 0,
            ipc_ns: 0,
        });
    }
}
