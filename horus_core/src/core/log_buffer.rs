use crate::memory::platform::shm_logs_path;
use log::error;
use memmap2::MmapMut;
use serde::{Deserialize, Serialize};
use std::fs::{self, OpenOptions};
use std::sync::atomic::{AtomicU64, Ordering};
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
    Publish,
    Subscribe,
    Info,
    Warning,
    Error,
    Debug,
}

const MAX_LOG_ENTRIES: usize = 5000;

/// Total bytes per slot, including the 8-byte seqlock field.
const SLOT_SIZE: usize = 512;

/// Bytes occupied by the seqlock sequence field at the start of each slot.
const SLOT_SEQ_SIZE: usize = 8;

/// Bytes available for the serialised log entry within a slot.
const SLOT_DATA_SIZE: usize = SLOT_SIZE - SLOT_SEQ_SIZE; // 504 bytes

const MAX_MESSAGE_LEN: usize = 300;

/// Bytes reserved for the mmap header.
///
/// Layout:
///   [0..8]   `write_idx`: AtomicU64 — monotonically-incrementing counter.
///            Each writer atomically claims the next slot via `fetch_add`.
///   [8..64]  reserved / padding.
const HEADER_SIZE: usize = 64;

// ─── Atomic helpers (all unsafe; callers guarantee alignment + bounds) ────────

/// Atomically load `write_idx` from the mmap header (acquire semantics).
///
/// # Safety
/// `base` must point to at least `HEADER_SIZE` bytes of mmap'd memory, and
/// the pointer must be 8-byte aligned (guaranteed because mmap starts on a
/// page boundary).
#[inline]
unsafe fn load_write_idx(base: *const u8) -> u64 {
    (*(base as *const AtomicU64)).load(Ordering::Acquire)
}

/// Atomically claim the next write slot via `fetch_add(1, AcqRel)`.
///
/// Returns the **old** value, which is the claimed slot index.  Because
/// `fetch_add` uses a hardware `LOCK XADD` on x86-64, this is cross-process
/// safe on MAP_SHARED mmap: each caller receives a unique index.
///
/// # Safety
/// Same as `load_write_idx`.
#[inline]
unsafe fn claim_slot(base: *const u8) -> u64 {
    (*(base as *const AtomicU64)).fetch_add(1, Ordering::AcqRel)
}

/// Atomically store a value into the seqlock field at the start of `slot_idx`.
///
/// # Safety
/// `slot_idx` must be < `MAX_LOG_ENTRIES` and `base` must point to the full
/// mmap region (`HEADER_SIZE + MAX_LOG_ENTRIES * SLOT_SIZE` bytes).
/// `HEADER_SIZE` and `SLOT_SIZE` are multiples of 8, so the pointer is always
/// 8-byte aligned.
#[inline]
unsafe fn store_slot_seq(base: *const u8, slot_idx: usize, value: u64) {
    let off = HEADER_SIZE + slot_idx * SLOT_SIZE;
    (*(base.add(off) as *const AtomicU64)).store(value, Ordering::Release);
}

/// Atomically load the seqlock field of `slot_idx` (acquire semantics).
///
/// # Safety
/// Same as `store_slot_seq`.
#[inline]
unsafe fn load_slot_seq(base: *const u8, slot_idx: usize) -> u64 {
    let off = HEADER_SIZE + slot_idx * SLOT_SIZE;
    (*(base.add(off) as *const AtomicU64)).load(Ordering::Acquire)
}

// ─────────────────────────────────────────────────────────────────────────────

/// Shared memory ring buffer for logs.
///
/// ## Cross-process slot-claiming protocol
///
/// The `write_idx` in the mmap header is updated via an atomic `fetch_add(1,
/// AcqRel)`.  Because the mmap is `MAP_SHARED`, this instruction (`LOCK XADD`
/// on x86-64) is visible to all processes that have mapped the same file.
/// Each call to `push()` claims a **unique** slot index before writing, so two
/// concurrent writers — even in different processes — can never collide.
///
/// ## Per-slot seqlock
///
/// Each slot carries an 8-byte sequence field at its start:
/// - `0`         → slot is empty (never written).
/// - odd value   → write in progress (reader must skip or wait).
/// - even, ≠ 0  → write complete (safe to read).
///
/// The writer stores `claimed_idx * 2 + 1` before copying data and
/// `claimed_idx * 2 + 2` after.  Readers check the seq field and skip
/// in-progress slots, preventing torn reads when a reader runs concurrently
/// with a writer from another process.
pub struct SharedLogBuffer {
    mmap: Mutex<MmapMut>,
}

impl SharedLogBuffer {
    pub fn new() -> crate::error::HorusResult<Self> {
        // Logs are intentionally global (not session-isolated) so monitor can see all logs
        let path = shm_logs_path();
        Self::open_at(&path)
    }

    fn open_at(path: &std::path::Path) -> crate::error::HorusResult<Self> {
        if let Some(parent) = path.parent() {
            let _ = fs::create_dir_all(parent);
        }

        let total_size = HEADER_SIZE + (MAX_LOG_ENTRIES * SLOT_SIZE);

        let file = OpenOptions::new()
            .read(true)
            .write(true)
            .create(true)
            .truncate(false)
            .open(path)?;

        let metadata = file.metadata()?;
        if metadata.len() == 0 {
            file.set_len(total_size as u64)?;
        }

        // SAFETY: file is a valid open file with size set to total_size above
        let mmap = unsafe {
            MmapMut::map_mut(&file).map_err(|e| {
                crate::error::HorusError::Memory(crate::error::MemoryError::MmapFailed {
                    reason: e.to_string(),
                })
            })?
        };

        Ok(Self {
            mmap: Mutex::new(mmap),
        })
    }

    /// Push a log entry to the ring buffer.
    ///
    /// The `write_idx` is advanced with an atomic `fetch_add`, ensuring that
    /// concurrent callers — within the same process and across processes — each
    /// claim a distinct slot before writing.
    pub fn push(&self, entry: LogEntry) {
        let mut guard = self.mmap.lock().unwrap();

        // Truncate message if too long (BEFORE serialization to keep metadata intact)
        let mut entry = entry;
        if entry.message.len() > MAX_MESSAGE_LEN {
            let mut end = MAX_MESSAGE_LEN - 3;
            while end > 0 && !entry.message.is_char_boundary(end) {
                end -= 1;
            }
            entry.message.truncate(end);
            entry.message.push_str("...");
        }

        // Serialize log entry
        let serialized = match bincode::serialize(&entry) {
            Ok(data) if data.len() <= SLOT_DATA_SIZE => data,
            Ok(data) => {
                // Serialized size still exceeds slot data area — truncate as a safety net.
                error!(
                    "[LogBuffer] Log entry too large ({} bytes, max {}); truncating",
                    data.len(),
                    SLOT_DATA_SIZE
                );
                data[..SLOT_DATA_SIZE].to_vec()
            }
            Err(e) => {
                error!("[LogBuffer] Failed to serialize log: {}", e);
                return;
            }
        };

        // Obtain a stable raw pointer to the mmap for all subsequent atomic and
        // data operations.  We keep the MutexGuard alive for the duration of
        // push() so the memory remains valid and no other thread within this
        // process can access the mmap concurrently.
        //
        // SAFETY: MmapMut::map_mut returns a page-aligned mapping.  All AtomicU64
        // accesses are at offsets that are multiples of 8 (HEADER_SIZE=64,
        // SLOT_SIZE=512), satisfying the 8-byte alignment requirement.
        let base: *mut u8 = {
            let s: &mut [u8] = &mut guard;
            s.as_mut_ptr()
        };

        // ── Step 1: Claim a unique slot via atomic fetch_add ─────────────────
        let claimed_idx = unsafe { claim_slot(base) };
        let slot_idx = (claimed_idx as usize) % MAX_LOG_ENTRIES;
        let slot_off = HEADER_SIZE + slot_idx * SLOT_SIZE;

        // ── Step 2: Mark slot as write-in-progress (odd seq) ─────────────────
        // Readers that see an odd seq will skip this slot, preventing torn reads.
        unsafe {
            store_slot_seq(base, slot_idx, claimed_idx.wrapping_mul(2).wrapping_add(1));
        }

        // ── Step 3: Write serialised entry data ───────────────────────────────
        let data_off = slot_off + SLOT_SEQ_SIZE;
        let len = serialized.len().min(SLOT_DATA_SIZE);
        unsafe {
            std::ptr::copy_nonoverlapping(serialized.as_ptr(), base.add(data_off), len);
            if len < SLOT_DATA_SIZE {
                std::ptr::write_bytes(base.add(data_off + len), 0, SLOT_DATA_SIZE - len);
            }
        }

        // ── Step 4: Mark slot as complete (even nonzero seq) ─────────────────
        // The Release ordering ensures the data writes above are visible to any
        // reader that performs an Acquire load on this seq field.
        unsafe {
            store_slot_seq(base, slot_idx, claimed_idx.wrapping_mul(2).wrapping_add(2));
        }
    }

    /// Read all log entries from the ring buffer.
    ///
    /// Slots with an in-progress write (odd seqlock value) or that have never
    /// been written (seq == 0) are silently skipped.
    pub fn get_all(&self) -> Vec<LogEntry> {
        let guard = self.mmap.lock().unwrap();

        // SAFETY: same alignment guarantees as push().
        let base: *const u8 = (*guard).as_ptr();

        let write_idx = unsafe { load_write_idx(base) } as usize;

        let mut logs = Vec::new();
        let num_entries = write_idx.min(MAX_LOG_ENTRIES);
        let start_idx = if write_idx > MAX_LOG_ENTRIES {
            write_idx % MAX_LOG_ENTRIES
        } else {
            0
        };

        for i in 0..num_entries {
            let slot_idx = (start_idx + i) % MAX_LOG_ENTRIES;

            // Skip empty or in-progress slots (seqlock check).
            // SAFETY: slot_idx < MAX_LOG_ENTRIES; mmap covers the full buffer.
            let seq = unsafe { load_slot_seq(base, slot_idx) };
            if seq == 0 || seq & 1 == 1 {
                continue;
            }

            let data_off = HEADER_SIZE + slot_idx * SLOT_SIZE + SLOT_SEQ_SIZE;
            // SAFETY: data_off is within the mmap region.
            let entry_bytes =
                unsafe { std::slice::from_raw_parts(base.add(data_off), SLOT_DATA_SIZE) };

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

// ─── Tests ────────────────────────────────────────────────────────────────────

impl SharedLogBuffer {
    /// Read the current write index from the mmap header.
    ///
    /// This is a monotonically-increasing counter: each `push()` increments it
    /// by 1.  External tools (e.g. `horus log --follow`) can poll this value to
    /// detect when new entries have been written without fetching all entries.
    pub fn write_idx(&self) -> u64 {
        let guard = self.mmap.lock().unwrap();
        let base: *const u8 = (*guard).as_ptr();
        // SAFETY: mmap is at least HEADER_SIZE bytes; pointer is page-aligned.
        unsafe { load_write_idx(base) }
    }
}

impl SharedLogBuffer {
    /// Create a SharedLogBuffer backed by a file at `path`.
    /// Used in tests to avoid touching the global log path.
    pub fn new_at_path(path: &std::path::Path) -> crate::error::HorusResult<Self> {
        Self::open_at(path)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::Arc;

    fn make_entry(node: &str, tick: u64) -> LogEntry {
        LogEntry {
            timestamp: "2026-01-01T00:00:00Z".to_string(),
            tick_number: tick,
            node_name: node.to_string(),
            log_type: LogType::Info,
            topic: None,
            message: format!("test entry {}", tick),
            tick_us: 0,
            ipc_ns: 0,
        }
    }

    fn temp_buf(test_id: u64) -> (SharedLogBuffer, std::path::PathBuf) {
        let path = std::env::temp_dir().join(format!(
            "horus_log_test_{}_{}.bin",
            std::process::id(),
            test_id
        ));
        let buf = SharedLogBuffer::new_at_path(&path).unwrap();
        (buf, path)
    }

    /// Basic sequential write+read round-trip.
    #[test]
    fn test_basic_push_get_all() {
        let (buf, path) = temp_buf(0);
        buf.push(make_entry("node_a", 1));
        buf.push(make_entry("node_b", 2));

        let all = buf.get_all();
        assert_eq!(all.len(), 2);
        assert_eq!(buf.write_idx(), 2);

        let _ = std::fs::remove_file(path);
    }

    /// Each slot's seqlock must be even-nonzero after writing (complete).
    #[test]
    fn test_slot_seq_complete_after_push() {
        let (buf, path) = temp_buf(1);

        const N: usize = 5;
        for i in 0..N {
            buf.push(make_entry("n", i as u64));
        }

        assert_eq!(buf.write_idx(), N as u64);

        let guard = buf.mmap.lock().unwrap();
        let base: *const u8 = (*guard).as_ptr();
        for slot_idx in 0..N {
            // SAFETY: slot_idx < MAX_LOG_ENTRIES.
            let seq = unsafe { load_slot_seq(base, slot_idx) };
            assert!(
                seq != 0 && seq & 1 == 0,
                "slot {} seq must be even-nonzero after write, got {}",
                slot_idx,
                seq
            );
        }
        drop(guard);
        let _ = std::fs::remove_file(path);
    }

    /// 8 threads each push 100 entries concurrently.
    /// Invariants:
    ///   1. write_idx == total pushes (fetch_add is exact, no lost increments).
    ///   2. All entries are readable (no slot was claimed by two threads).
    ///   3. All tick_numbers are unique (no entry was silently overwritten).
    #[test]
    fn test_concurrent_log_no_duplicate_slots() {
        let (buf, path) = temp_buf(2);
        let buf = Arc::new(buf);

        const THREADS: usize = 8;
        const ENTRIES_PER_THREAD: usize = 100;
        const TOTAL: usize = THREADS * ENTRIES_PER_THREAD;

        let handles: Vec<_> = (0..THREADS)
            .map(|t| {
                let b = buf.clone();
                std::thread::spawn(move || {
                    for i in 0..ENTRIES_PER_THREAD {
                        b.push(make_entry(
                            &format!("thread_{}", t),
                            (t * ENTRIES_PER_THREAD + i) as u64,
                        ));
                    }
                })
            })
            .collect();
        for h in handles {
            h.join().unwrap();
        }

        // write_idx must equal total push() calls.
        assert_eq!(
            buf.write_idx(),
            TOTAL as u64,
            "write_idx must equal total push() calls — no increment was lost"
        );

        // All entries must be readable.
        let all = buf.get_all();
        assert_eq!(
            all.len(),
            TOTAL,
            "all {} entries must be readable — no slot was claimed by two threads",
            TOTAL
        );

        // All tick_numbers must be unique (no overwrite).
        let mut seen = std::collections::HashSet::new();
        for e in &all {
            assert!(
                seen.insert(e.tick_number),
                "duplicate tick_number {} — slot collision detected",
                e.tick_number
            );
        }

        let _ = std::fs::remove_file(path);
    }

    /// Push more than MAX_LOG_ENTRIES to verify wrap-around doesn't panic
    /// and get_all() returns valid entries.
    #[test]
    fn test_ring_buffer_overflow_wrap() {
        let (buf, path) = temp_buf(10);

        let total = MAX_LOG_ENTRIES + 500;
        for i in 0..total {
            buf.push(make_entry("overflow_wrap_node", i as u64));
        }

        // write_idx must reflect total pushes
        assert_eq!(buf.write_idx(), total as u64);

        // get_all() returns at most MAX_LOG_ENTRIES (ring wrapped)
        let all = buf.get_all();
        assert!(
            !all.is_empty(),
            "get_all() must return entries after overflow"
        );
        assert!(
            all.len() <= MAX_LOG_ENTRIES,
            "get_all() must return at most MAX_LOG_ENTRIES entries, got {}",
            all.len()
        );

        // All returned entries must have the correct node_name
        for e in &all {
            assert_eq!(e.node_name, "overflow_wrap_node");
        }

        let _ = std::fs::remove_file(path);
    }

    /// Push a LogEntry with a very long message and verify truncation at MAX_MESSAGE_LEN.
    #[test]
    fn test_message_truncation_at_limit() {
        let (buf, path) = temp_buf(11);

        let long_msg = "A".repeat(1000);
        let entry = LogEntry {
            timestamp: "2026-01-01T00:00:00Z".to_string(),
            tick_number: 99,
            node_name: "truncation_node".to_string(),
            log_type: LogType::Warning,
            topic: None,
            message: long_msg,
            tick_us: 0,
            ipc_ns: 0,
        };
        buf.push(entry);

        let all = buf.get_all();
        assert_eq!(all.len(), 1);

        // push() truncates at MAX_MESSAGE_LEN (300) and appends "..."
        let msg = &all[0].message;
        assert!(
            msg.len() <= MAX_MESSAGE_LEN,
            "message should be truncated to at most {} bytes, got {}",
            MAX_MESSAGE_LEN,
            msg.len()
        );
        assert!(
            msg.ends_with("..."),
            "truncated message should end with '...', got: {}",
            msg
        );

        let _ = std::fs::remove_file(path);
    }

    /// Push entries with empty strings, unicode, and special characters.
    #[test]
    fn test_log_entry_serialization_edge_cases() {
        let (buf, path) = temp_buf(12);

        // Empty strings
        buf.push(LogEntry {
            timestamp: String::new(),
            tick_number: 0,
            node_name: "edge_case_empty".to_string(),
            log_type: LogType::Debug,
            topic: Some(String::new()),
            message: String::new(),
            tick_us: 0,
            ipc_ns: 0,
        });

        // Unicode characters
        buf.push(LogEntry {
            timestamp: "2026-01-01T00:00:00Z".to_string(),
            tick_number: 1,
            node_name: "edge_case_unicode".to_string(),
            log_type: LogType::Info,
            topic: Some("topic/日本語".to_string()),
            message: "Héllo wörld 🤖 Ωmega".to_string(),
            tick_us: 42,
            ipc_ns: 100,
        });

        // Special characters (newlines, tabs, null-ish)
        buf.push(LogEntry {
            timestamp: "2026-01-01T00:00:00Z".to_string(),
            tick_number: 2,
            node_name: "edge_case_special".to_string(),
            log_type: LogType::Error,
            topic: None,
            message: "line1\nline2\ttab\r\nwindows".to_string(),
            tick_us: 0,
            ipc_ns: 0,
        });

        let all = buf.get_all();
        assert_eq!(all.len(), 3, "all 3 edge-case entries must round-trip");

        // Verify empty-string entry
        let empty = all
            .iter()
            .find(|e| e.node_name == "edge_case_empty")
            .unwrap();
        assert_eq!(empty.message, "");
        assert_eq!(empty.timestamp, "");
        assert_eq!(empty.topic, Some(String::new()));

        // Verify unicode entry
        let uni = all
            .iter()
            .find(|e| e.node_name == "edge_case_unicode")
            .unwrap();
        assert_eq!(uni.message, "Héllo wörld 🤖 Ωmega");
        assert_eq!(uni.topic.as_deref(), Some("topic/日本語"));

        // Verify special chars entry
        let spec = all
            .iter()
            .find(|e| e.node_name == "edge_case_special")
            .unwrap();
        assert!(spec.message.contains('\n'));
        assert!(spec.message.contains('\t'));

        let _ = std::fs::remove_file(path);
    }

    /// A fresh buffer's get_all() must return an empty vec.
    #[test]
    fn test_empty_buffer_get_all() {
        let (buf, path) = temp_buf(13);

        let all = buf.get_all();
        assert!(all.is_empty(), "fresh buffer get_all() must be empty");
        assert_eq!(buf.write_idx(), 0, "fresh buffer write_idx must be 0");

        let _ = std::fs::remove_file(path);
    }

    /// Push one entry, verify get_all() returns it with correct fields.
    #[test]
    fn test_push_and_read_single_entry() {
        let (buf, path) = temp_buf(14);

        let entry = LogEntry {
            timestamp: "12:34:56.789".to_string(),
            tick_number: 777,
            node_name: "single_entry_node".to_string(),
            log_type: LogType::Error,
            topic: Some("/sensors/imu".to_string()),
            message: "sensor timeout detected".to_string(),
            tick_us: 1234,
            ipc_ns: 5678,
        };
        buf.push(entry);

        let all = buf.get_all();
        assert_eq!(all.len(), 1, "must have exactly one entry");

        let e = &all[0];
        assert_eq!(e.timestamp, "12:34:56.789");
        assert_eq!(e.tick_number, 777);
        assert_eq!(e.node_name, "single_entry_node");
        assert_eq!(e.log_type, LogType::Error);
        assert_eq!(e.topic.as_deref(), Some("/sensors/imu"));
        assert_eq!(e.message, "sensor timeout detected");
        assert_eq!(e.tick_us, 1234);
        assert_eq!(e.ipc_ns, 5678);

        let _ = std::fs::remove_file(path);
    }
}
