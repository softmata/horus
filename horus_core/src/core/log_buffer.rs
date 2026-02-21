use crate::memory::platform::shm_logs_path;
use log::error;
use memmap2::MmapMut;
use serde::{Deserialize, Serialize};
use std::fs::{self, OpenOptions};
use std::sync::Mutex;

/// Log entry with timestamp and metadata
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct LogEntry {
    pub timestamp: String,
    pub tick_number: u64, // Deterministic tick counter for replay/comparison
    pub node_name: String,
    pub log_type: LogType,
    pub topic: Option<String>,
    pub message: String,
    pub tick_us: u64,
    pub ipc_ns: u64,
}

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub enum LogType {
    // Node/IPC operations
    Publish,
    Subscribe,
    Info,
    Warning,
    Error,
    Debug,
    RemoteDeploy,
    RemoteCompile,
    RemoteExecute,
    // Topic/Shared memory operations
    TopicRead,
    TopicWrite,
    TopicMap,
    TopicUnmap,
}

const MAX_LOG_ENTRIES: usize = 5000;
const LOG_ENTRY_SIZE: usize = 512; // Fixed size per entry (serialized)
const MAX_MESSAGE_LEN: usize = 300; // Max message string length before truncation
const HEADER_SIZE: usize = 64; // Space for metadata (write_idx, etc.)

/// Shared memory ring buffer for logs - lock-free, cross-process
pub struct SharedLogBuffer {
    mmap: Mutex<MmapMut>,
}

// Default implementation removed - use SharedLogBuffer::new()? instead
// since initialization can fail

impl SharedLogBuffer {
    pub fn new() -> crate::error::HorusResult<Self> {
        // Logs are intentionally global (not session-isolated) so monitor can see all logs
        let path = shm_logs_path();

        // Ensure parent directory exists
        if let Some(parent) = path.parent() {
            let _ = fs::create_dir_all(parent);
        }

        // Calculate total size: header + ring buffer
        let total_size = HEADER_SIZE + (MAX_LOG_ENTRIES * LOG_ENTRY_SIZE);

        // Create or open memory-mapped file (don't truncate to preserve existing logs)
        let file = OpenOptions::new()
            .read(true)
            .write(true)
            .create(true)
            .truncate(false)
            .open(&path)?;

        // Only set length if file is new/empty (avoids corrupting existing logs)
        let metadata = file.metadata()?;
        if metadata.len() == 0 {
            file.set_len(total_size as u64)?;
        }

        // SAFETY: file is a valid open file with size set to total_size above
        let mmap = unsafe {
            MmapMut::map_mut(&file)
                .map_err(|e| crate::error::HorusError::Memory(format!("Failed to mmap: {}", e)))?
        };

        Ok(Self {
            mmap: Mutex::new(mmap),
        })
    }

    /// Push a log entry to the ring buffer (lock-free write)
    pub fn push(&self, entry: LogEntry) {
        let mut mmap = self.mmap.lock().unwrap();

        // Read current write index from header
        let write_idx_bytes = &mmap[0..8];
        let write_idx = match write_idx_bytes.try_into() {
            Ok(bytes) => u64::from_le_bytes(bytes) as usize,
            Err(_) => {
                error!("[LogBuffer] Failed to read log buffer index");
                return;
            }
        };

        // Truncate message if too long (BEFORE serialization to keep metadata intact)
        let mut entry = entry;
        if entry.message.len() > MAX_MESSAGE_LEN {
            // Find a valid UTF-8 char boundary for truncation
            let mut end = MAX_MESSAGE_LEN - 3;
            while end > 0 && !entry.message.is_char_boundary(end) {
                end -= 1;
            }
            entry.message.truncate(end);
            entry.message.push_str("...");
        }

        // Serialize log entry
        let serialized = match bincode::serialize(&entry) {
            Ok(data) if data.len() <= LOG_ENTRY_SIZE => data,
            Ok(data) => {
                // This should rarely happen now, but keep as safety net
                error!("[LogBuffer] Log entry still too large ({} bytes) after message truncation - metadata too big?", data.len());
                data[..LOG_ENTRY_SIZE].to_vec()
            }
            Err(e) => {
                error!("[LogBuffer] Failed to serialize log: {}", e);
                return;
            }
        };

        // Calculate position in ring buffer
        let slot_idx = write_idx % MAX_LOG_ENTRIES;
        let offset = HEADER_SIZE + (slot_idx * LOG_ENTRY_SIZE);

        // Write log entry
        let len = serialized.len().min(LOG_ENTRY_SIZE);
        mmap[offset..offset + len].copy_from_slice(&serialized[..len]);

        // Clear remaining space in slot
        if len < LOG_ENTRY_SIZE {
            mmap[offset + len..offset + LOG_ENTRY_SIZE].fill(0);
        }

        // Update write index atomically
        let new_write_idx = write_idx + 1;
        mmap[0..8].copy_from_slice(&(new_write_idx as u64).to_le_bytes());
    }

    /// Read all logs from the ring buffer
    pub fn get_all(&self) -> Vec<LogEntry> {
        let mmap = self.mmap.lock().unwrap();

        // Read write index
        let write_idx_bytes = &mmap[0..8];
        let write_idx = match write_idx_bytes.try_into() {
            Ok(bytes) => u64::from_le_bytes(bytes) as usize,
            Err(_) => {
                error!("[LogBuffer] Failed to read log buffer index");
                return Vec::new();
            }
        };

        let mut logs = Vec::new();

        // Determine how many valid entries exist
        let num_entries = write_idx.min(MAX_LOG_ENTRIES);

        // Read from oldest to newest
        let start_idx = if write_idx > MAX_LOG_ENTRIES {
            write_idx % MAX_LOG_ENTRIES
        } else {
            0
        };

        for i in 0..num_entries {
            let slot_idx = (start_idx + i) % MAX_LOG_ENTRIES;
            let offset = HEADER_SIZE + (slot_idx * LOG_ENTRY_SIZE);

            let entry_bytes = &mmap[offset..offset + LOG_ENTRY_SIZE];

            // Try to deserialize
            if let Ok(entry) = bincode::deserialize::<LogEntry>(entry_bytes) {
                logs.push(entry);
            }
        }

        logs
    }

    /// Get logs for a specific node
    pub fn for_node(&self, node_name: &str) -> Vec<LogEntry> {
        self.get_all()
            .into_iter()
            .filter(|e| e.node_name == node_name)
            .collect()
    }

    /// Get logs for a specific topic
    pub fn for_topic(&self, topic: &str) -> Vec<LogEntry> {
        self.get_all()
            .into_iter()
            .filter(|e| e.topic.as_ref().is_some_and(|t| t == topic))
            .collect()
    }

    pub fn clear(&self) {
        let mut mmap = self.mmap.lock().unwrap();
        mmap[0..8].fill(0); // Reset write index
    }
}

// Global shared memory log buffer
lazy_static::lazy_static! {
    pub static ref GLOBAL_LOG_BUFFER: SharedLogBuffer = SharedLogBuffer::new()
        .expect("FATAL: Failed to initialize global log buffer - cannot continue");
}

/// Publish a log entry to shared memory ring buffer
pub fn publish_log(entry: LogEntry) {
    GLOBAL_LOG_BUFFER.push(entry);
}
