use crate::memory::platform::shm_logs_path;
use log::error;
use memmap2::MmapMut;
use serde::{Deserialize, Serialize};
use std::fs::OpenOptions;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::Mutex;
use std::time::{Duration, Instant};

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

const DEFAULT_LOG_ENTRIES: usize = 5000;

/// Effective max log entries — configurable via HORUS_LOG_BUFFER_SIZE env var.
fn max_log_entries() -> usize {
    std::env::var("HORUS_LOG_BUFFER_SIZE")
        .ok()
        .and_then(|s| s.parse::<usize>().ok())
        .unwrap_or(DEFAULT_LOG_ENTRIES)
        .clamp(100, 50000)
}

/// Default capacity for a buffer opened without one (`open_at`).
///
/// This is NOT a bound on any buffer's slot count and must never be used as one
/// in a `SAFETY` argument or an index assert, which is what the previous comment
/// ("keep old name as alias for backward compat in SAFETY comments") invited.
/// `SharedLogBuffer::capacity` is the only true bound: the error buffer holds
/// 500 slots, the remote buffer 2000, and the main buffer anything from 100 to
/// 50000 depending on `HORUS_LOG_BUFFER_SIZE`.
const MAX_LOG_ENTRIES: usize = DEFAULT_LOG_ENTRIES;

/// Capacity for the dedicated error ring buffer.
/// At typical error rates (1-10 errors/hour), 500 slots = days of retention.
const ERROR_BUFFER_ENTRIES: usize = 500;

/// Total bytes per slot, including the 8-byte seqlock field.
const SLOT_SIZE: usize = 512;

/// Bytes occupied by the seqlock sequence field at the start of each slot.
const SLOT_SEQ_SIZE: usize = 8;

/// Bytes available for the serialised log entry within a slot.
const SLOT_DATA_SIZE: usize = SLOT_SIZE - SLOT_SEQ_SIZE; // 504 bytes

/// Maximum length for the `message` field after truncation.
///
/// The slot data budget is 504 bytes.  Bincode fixed-overhead for a `LogEntry`
/// with all `Option` fields present is 61 bytes (4 string-length prefixes ×
/// 8 + tick_number 8 + LogType u32 + Option tag 1 + tick_us 8 + ipc_ns 8).
/// Variable string limits: timestamp 32 + node_name 64 + topic 64 = 160.
/// That leaves 504 − 61 − 160 = 283 bytes for the message.  We use 280 for
/// a small margin.
const MAX_MESSAGE_LEN: usize = 280;

/// How long a reader spins waiting for an odd-seq (in-progress) slot before
/// giving up.  Normal writes complete in single-digit microseconds; a slot
/// still odd after this timeout is treated as crash-orphaned and skipped.
const SEQLOCK_SPIN_TIMEOUT: Duration = Duration::from_millis(5);

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
/// `base` must point to a mapping of at least `HEADER_SIZE + (slot_idx + 1) *
/// SLOT_SIZE` bytes — i.e. `slot_idx` must be below the capacity that
/// `SharedLogBuffer::open_at_with_capacity` derived from the mapping length,
/// NOT below the `MAX_LOG_ENTRIES` constant (the error buffer holds 500 slots
/// and the remote buffer 2000, so that constant is far too permissive a bound).
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

/// Truncate a string field to at most `max` bytes on a valid UTF-8 char
/// boundary, appending "..." when truncation occurs.
#[inline]
fn truncate_field(s: &mut String, max: usize) {
    if s.len() <= max {
        return;
    }
    let suffix = "...";
    let mut end = max.saturating_sub(suffix.len());
    while end > 0 && !s.is_char_boundary(end) {
        end -= 1;
    }
    s.truncate(end);
    s.push_str(suffix);
}

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
    /// Ring buffer capacity (number of log entry slots).
    capacity: usize,
}

impl SharedLogBuffer {
    pub fn new() -> crate::error::HorusResult<Self> {
        // Logs are intentionally global (not session-isolated) so monitor can see all logs
        let path = shm_logs_path();
        Self::open_at_with_capacity(&path, max_log_entries())
    }

    /// Create the dedicated error log buffer (smaller, separate SHM path).
    /// Configurable via HORUS_ERROR_BUFFER_SIZE env var (default 500, range 50-5000).
    pub fn new_error_buffer() -> crate::error::HorusResult<Self> {
        use crate::memory::shm_error_logs_path;
        let path = shm_error_logs_path();
        let capacity = std::env::var("HORUS_ERROR_BUFFER_SIZE")
            .ok()
            .and_then(|s| s.parse::<usize>().ok())
            .unwrap_or(ERROR_BUFFER_ENTRIES)
            .clamp(50, 5000);
        Self::open_at_with_capacity(&path, capacity)
    }

    /// Create the remote log buffer for network-received log entries.
    /// Default 2000 slots, configurable via HORUS_REMOTE_LOG_SIZE env var.
    pub fn new_remote_buffer() -> crate::error::HorusResult<Self> {
        let capacity = std::env::var("HORUS_REMOTE_LOG_SIZE")
            .ok()
            .and_then(|s| s.parse::<usize>().ok())
            .unwrap_or(2000)
            .clamp(100, 50000);
        let path = horus_sys::shm::shm_remote_logs_path();
        Self::open_at_with_capacity(&path, capacity)
    }

    fn open_at(path: &std::path::Path) -> crate::error::HorusResult<Self> {
        Self::open_at_with_capacity(path, MAX_LOG_ENTRIES)
    }

    fn open_at_with_capacity(
        path: &std::path::Path,
        capacity: usize,
    ) -> crate::error::HorusResult<Self> {
        if let Some(parent) = path.parent() {
            let _ = horus_sys::shm::create_shm_dir_all(parent);
        }

        let total_size = HEADER_SIZE + (capacity * SLOT_SIZE);

        // Owner-only. These buffers hold the robot's log and error history; with
        // the default `0o666 & !umask` they were observed on disk as 0o664 —
        // readable by every local user and writable by the owner's group, which
        // makes them a tamper target as well as a disclosure one.
        let mut opts = OpenOptions::new();
        opts.read(true).write(true).create(true).truncate(false);
        #[cfg(unix)]
        {
            use std::os::unix::fs::OpenOptionsExt;
            opts.mode(horus_sys::shm::SHM_FILE_MODE);
        }
        let file = opts.open(path)?;
        // `mode` only applies when the file is created; tighten a buffer left
        // behind by an older run too.
        horus_sys::shm::harden_shm_file(path);

        let metadata = file.metadata()?;
        if metadata.len() == 0 {
            // New file — initialize to expected size.
            file.set_len(total_size as u64)?;
        } else if metadata.len() != total_size as u64 {
            // File exists with different capacity (e.g., previous run used different
            // config). Only grow, NEVER shrink — another process may have this file
            // mmap'd, and truncating would invalidate their mapping (SIGBUS).
            let existing = metadata.len() as usize;
            if total_size > existing {
                file.set_len(total_size as u64)?;
            } else {
                // Existing file is larger — leave it alone and map all of it. Our
                // capacity is smaller, so we simply use a prefix of the region.
            }
        }

        // SAFETY: file is a valid open file with size set to total_size above
        let mmap = unsafe {
            MmapMut::map_mut(&file).map_err(|e| {
                crate::error::HorusError::Memory(crate::error::MemoryError::MmapFailed {
                    reason: e.to_string(),
                })
            })?
        };

        // Every `unsafe` helper below states its precondition as "`slot_idx` is
        // less than the capacity and `base` covers the whole region". Until now
        // nothing enforced the second half: `capacity` came from the caller (an
        // env var, or the `MAX_LOG_ENTRIES` constant) while the mapping length
        // came from the file, and the two were only *assumed* to agree.
        //
        // They can disagree. `set_len` above is called by every process that
        // opens the buffer, and two processes that both observe `len == 0` both
        // size it: if the one with the SMALLER capacity lands its `set_len`
        // second, the other maps a file shorter than its own `capacity` implies
        // and then writes slots far past the end of the mapping. Another local
        // process can also `ftruncate` the region between the `set_len` and the
        // `map_mut` — /dev/shm is shared, and the file being 0600 only narrows
        // that to a same-UID peer, which is inside the threat model.
        //
        // Derive the capacity from the mapping that actually exists rather than
        // from what we asked for. In the overwhelmingly common case the file is
        // exactly (or more than) `total_size` and this changes nothing.
        let mapped = mmap.len();
        if mapped < HEADER_SIZE + SLOT_SIZE {
            return Err(crate::error::HorusError::Memory(
                crate::error::MemoryError::MmapFailed {
                    reason: format!(
                        "log buffer at {} mapped {} bytes — too small for the {}-byte header \
                         plus one {}-byte slot",
                        path.display(),
                        mapped,
                        HEADER_SIZE,
                        SLOT_SIZE
                    ),
                },
            ));
        }
        let capacity = capacity.min((mapped - HEADER_SIZE) / SLOT_SIZE);

        Ok(Self {
            mmap: Mutex::new(mmap),
            capacity,
        })
    }

    /// Push a log entry to the ring buffer.
    ///
    /// The `write_idx` is advanced with an atomic `fetch_add`, ensuring that
    /// concurrent callers — within the same process and across processes — each
    /// claim a distinct slot before writing.
    pub fn push(&self, entry: LogEntry) {
        // Recover from poisoned mutex (a thread panicked while logging).
        // Logging must never crash the scheduler — degraded logs are better than no process.
        let mut guard = self.mmap.lock().unwrap_or_else(|e| e.into_inner());

        // Truncate all variable-length fields BEFORE serialization so the
        // envelope is always valid bincode.  Message gets the tightest limit;
        // metadata fields get generous caps that still leave room.
        let mut entry = entry;
        truncate_field(&mut entry.node_name, 64);
        truncate_field(&mut entry.timestamp, 32);
        if let Some(ref mut t) = entry.topic {
            truncate_field(t, 64);
        }
        truncate_field(&mut entry.message, MAX_MESSAGE_LEN);

        // Serialize log entry
        let serialized = match bincode::serialize(&entry) {
            Ok(data) if data.len() <= SLOT_DATA_SIZE => data,
            Ok(data) => {
                // Even after field truncation the entry is too large — drop it
                // rather than writing corrupt mid-struct bytes.
                error!(
                    "[LogBuffer] Log entry too large ({} bytes, max {}); dropping",
                    data.len(),
                    SLOT_DATA_SIZE
                );
                return;
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
        // SAFETY: base points to the start of the mmap (page-aligned, >= HEADER_SIZE bytes).
        // The AtomicU64 at offset 0 is 8-byte aligned. MutexGuard keeps mmap alive.
        let claimed_idx = unsafe { claim_slot(base) };
        let slot_idx = (claimed_idx as usize) % self.capacity;
        let slot_off = HEADER_SIZE + slot_idx * SLOT_SIZE;

        // ── Step 2: Mark slot as write-in-progress (odd seq) ─────────────────
        // Readers that see an odd seq will skip this slot, preventing torn reads.
        // SAFETY: slot_idx < self.capacity (modulo above), and `open_at_with_capacity`
        // clamped self.capacity so that HEADER_SIZE + capacity * SLOT_SIZE fits inside
        // the mapping. HEADER_SIZE and SLOT_SIZE are multiples of 8, so the AtomicU64
        // is aligned.
        unsafe {
            store_slot_seq(base, slot_idx, claimed_idx.wrapping_mul(2).wrapping_add(1));
        }

        // ── Step 3: Write serialised entry data ───────────────────────────────
        let data_off = slot_off + SLOT_SEQ_SIZE;
        let len = serialized.len().min(SLOT_DATA_SIZE);
        // SAFETY: data_off + SLOT_DATA_SIZE <= HEADER_SIZE + self.capacity * SLOT_SIZE,
        // which `open_at_with_capacity` clamped to fit inside the mapping.
        // serialized.len() <= SLOT_DATA_SIZE. MutexGuard keeps the mmap alive; the
        // claimed slot is exclusively ours *within this process* — a peer process with
        // a different capacity can still land on the same slot, which the reader's
        // seqlock re-check in `get_since` detects and discards.
        unsafe {
            std::ptr::copy_nonoverlapping(serialized.as_ptr(), base.add(data_off), len);
            if len < SLOT_DATA_SIZE {
                std::ptr::write_bytes(base.add(data_off + len), 0, SLOT_DATA_SIZE - len);
            }
        }

        // ── Step 4: Mark slot as complete (even nonzero seq) ─────────────────
        // The Release ordering ensures the data writes above are visible to any
        // reader that performs an Acquire load on this seq field.
        // SAFETY: slot_idx < self.capacity; base covers the full mmap; aligned (see Step 2).
        unsafe {
            store_slot_seq(base, slot_idx, claimed_idx.wrapping_mul(2).wrapping_add(2));
        }
    }

    /// Read all log entries from the ring buffer.
    ///
    /// Slots that have never been written (seq == 0) are skipped.  Slots with
    /// an in-progress write (odd seqlock) are retried for up to
    /// [`SEQLOCK_SPIN_TIMEOUT`] before being skipped — this handles both
    /// normal writes that complete within microseconds and crash-orphaned
    /// slots where the writer died mid-write.
    pub fn get_all(&self) -> Vec<LogEntry> {
        self.get_since(0)
    }

    /// Read log entries whose global write index is `>= after_idx`.
    ///
    /// The index is the value [`write_idx`](Self::write_idx) had when the
    /// entry was pushed — the buffer-wide `fetch_add` counter — so a drain
    /// loop that remembers the `write_idx` it last consumed can ask for
    /// exactly the entries it has not seen.
    ///
    /// Do NOT use [`LogEntry::tick_number`] for this. That field is the
    /// *publishing node's* own tick counter: it restarts per node, it is 0
    /// for every entry that arrives through the `log::` facade rather than
    /// from a node tick, and it advances at the node's rate rather than the
    /// buffer's. Comparing it against a buffer-wide index drops all facade
    /// entries and re-emits fast nodes' entries on every pass.
    ///
    /// Entries that the ring has already overwritten cannot be recovered; if
    /// `after_idx` is older than the oldest surviving entry the gap is
    /// silently skipped, and the caller can detect it by comparing
    /// `write_idx()` against `capacity`.
    pub fn get_since(&self, after_idx: u64) -> Vec<LogEntry> {
        let guard = self.mmap.lock().unwrap_or_else(|e| e.into_inner());

        // SAFETY: same alignment guarantees as push().
        let base: *const u8 = (*guard).as_ptr();

        // SAFETY: base is page-aligned mmap of at least HEADER_SIZE bytes; AtomicU64 at offset 0 is aligned.
        let write_idx = unsafe { load_write_idx(base) } as usize;

        let mut logs = Vec::new();
        let num_entries = write_idx.min(self.capacity);
        let start_idx = if write_idx > self.capacity {
            write_idx % self.capacity
        } else {
            0
        };

        // Global index of the oldest entry still resident in the ring.
        let oldest_idx = (write_idx - num_entries) as u64;
        // Skip the entries the caller has already consumed. Anything older
        // than `oldest_idx` was overwritten and is unrecoverable.
        let skip = after_idx.saturating_sub(oldest_idx).min(num_entries as u64) as usize;

        for i in skip..num_entries {
            let slot_idx = (start_idx + i) % self.capacity;

            // SAFETY: slot_idx < self.capacity, which `open_at_with_capacity`
            // clamped to fit the mapping; mmap covers the full buffer.
            let mut seq = unsafe { load_slot_seq(base, slot_idx) };

            // Never-written slot.
            if seq == 0 {
                continue;
            }

            // Odd seq = write in progress.  Spin briefly in case the writer is
            // about to finish; give up after SEQLOCK_SPIN_TIMEOUT (handles
            // crash-orphaned slots where the writer died mid-write).
            if seq & 1 == 1 {
                let deadline = Instant::now() + SEQLOCK_SPIN_TIMEOUT;
                let mut resolved = false;
                while Instant::now() < deadline {
                    std::hint::spin_loop();
                    // SAFETY: base from mmap, slot_idx < self.capacity (loop bound).
                    let seq2 = unsafe { load_slot_seq(base, slot_idx) };
                    if seq2 != 0 && seq2 & 1 == 0 {
                        // Carry the *resolved* stamp forward: it, not the stale
                        // odd value, is what the post-copy re-check compares against.
                        seq = seq2;
                        resolved = true;
                        break;
                    }
                }
                if !resolved {
                    // Writer is dead — reset seqlock to even so the slot can be
                    // reclaimed by a future writer (the data is garbage anyway).
                    // SAFETY: base from mmap, slot_idx < self.capacity (loop bound).
                    unsafe { store_slot_seq(base, slot_idx, seq.wrapping_add(1)) };
                    continue;
                }
            }

            let data_off = HEADER_SIZE + slot_idx * SLOT_SIZE + SLOT_SEQ_SIZE;

            // Copy the slot out, then re-read the stamp and require it unchanged.
            //
            // The type doc above promises the seq field "prevent[s] torn reads when
            // a reader runs concurrently with a writer from another process", and
            // before this it did not: the stamp was checked once, *before* the read,
            // and never again. A peer process that claimed this slot the instant
            // after that check wrote its 504 bytes straight underneath the reader.
            // Worse, the copy was handed to bincode as a `&[u8]` borrowed directly
            // from the peer-writable mapping — a live shared reference to memory
            // another process mutates, which is a data race in the Rust abstract
            // machine no matter how benign the bytes turn out to be.
            //
            // Both halves are closed here: the bytes are copied into a private
            // buffer first (bincode never sees the shared mapping), and the stamp is
            // re-read afterwards. Any change means the slot was claimed or rewritten
            // mid-copy, so the copy is discarded rather than deserialized. This is
            // the same protocol `communication::topic::seqlock::seqlock_consume`
            // already uses on the data plane.
            let mut slot_copy = [0u8; SLOT_DATA_SIZE];
            // SAFETY: data_off + SLOT_DATA_SIZE <= HEADER_SIZE + self.capacity *
            // SLOT_SIZE, which is within the mapping (clamped at open). Source and
            // destination do not overlap — `slot_copy` is a local.
            unsafe {
                std::ptr::copy_nonoverlapping(
                    base.add(data_off),
                    slot_copy.as_mut_ptr(),
                    SLOT_DATA_SIZE,
                );
            }

            // SAFETY: same bounds as the load above.
            let seq_after = unsafe { load_slot_seq(base, slot_idx) };
            if seq_after != seq {
                // Overwritten while we copied — the bytes may be half of one entry
                // and half of another. Drop them.
                continue;
            }

            if let Ok(entry) = bincode::deserialize::<LogEntry>(&slot_copy[..]) {
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

    /// Get logs matching a specific [`LogType`].
    pub fn for_type(&self, log_type: &LogType) -> Vec<LogEntry> {
        self.get_all()
            .into_iter()
            .filter(|e| &e.log_type == log_type)
            .collect()
    }
}

// Global shared memory log buffer
lazy_static::lazy_static! {
    pub static ref GLOBAL_LOG_BUFFER: SharedLogBuffer = SharedLogBuffer::new()
        .expect("FATAL: Failed to initialize global log buffer - cannot continue");

    /// Dedicated error ring buffer — Error/Warning entries persist here even when
    /// the main buffer is flooded by sensor pub/sub traffic. 500 slots = days of
    /// retention at typical error rates.
    pub static ref GLOBAL_ERROR_BUFFER: SharedLogBuffer = SharedLogBuffer::new_error_buffer()
        .expect("FATAL: Failed to initialize error log buffer - cannot continue");

    /// Remote log buffer — receives log entries from remote machines via horus_net.
    /// Separate from GLOBAL_LOG_BUFFER to prevent remote log flooding from evicting
    /// local logs. 2000 slots (configurable via HORUS_REMOTE_LOG_SIZE).
    pub static ref GLOBAL_REMOTE_LOG_BUFFER: SharedLogBuffer = SharedLogBuffer::new_remote_buffer()
        .expect("FATAL: Failed to initialize remote log buffer - cannot continue");
}

/// Publish a log entry to shared memory ring buffer.
///
/// Error and Warning entries are dual-written to both the main buffer and the
/// dedicated error buffer for persistent retention. Info/Debug/Publish/Subscribe
/// entries go to the main buffer only (zero overhead on the hot sensor path).
pub fn publish_log(entry: LogEntry) {
    if matches!(entry.log_type, LogType::Error | LogType::Warning) {
        GLOBAL_ERROR_BUFFER.push(entry.clone());
    }
    GLOBAL_LOG_BUFFER.push(entry);
}

// ─── Persistent File Drain ────────────────────────────────────────────────────

/// Start a background thread that drains the log ring buffer to disk.
///
/// Opt-in via `HORUS_LOG_FILE=true` or calling this function directly.
/// Configurable via env vars:
/// - `HORUS_LOG_DIR` — directory for log files (default: `.horus/logs/`)
/// - `HORUS_LOG_MAX_SIZE` — max file size in bytes before rotation (default: 10MB)
/// - `HORUS_LOG_MAX_FILES` — max rotated files to keep (default: 5)
///
/// Returns a join handle. Drop or join to stop the drain thread.
pub fn start_log_file_drain() -> Option<std::thread::JoinHandle<()>> {
    let enabled = std::env::var("HORUS_LOG_FILE")
        .map(|v| v == "true" || v == "1")
        .unwrap_or(false);
    if !enabled {
        return None;
    }

    let log_dir = std::env::var("HORUS_LOG_DIR").unwrap_or_else(|_| ".horus/logs".to_string());
    let max_size: u64 = std::env::var("HORUS_LOG_MAX_SIZE")
        .ok()
        .and_then(|s| s.parse().ok())
        .unwrap_or(10 * 1024 * 1024); // 10MB
    let max_files: usize = std::env::var("HORUS_LOG_MAX_FILES")
        .ok()
        .and_then(|s| s.parse().ok())
        .unwrap_or(5);

    std::thread::Builder::new()
        .name("horus-log-drain".into())
        .spawn(move || {
            log_drain_loop(&log_dir, max_size, max_files);
        })
        .ok()
}

/// The process-wide log-file drain, started at most once.
static LOG_FILE_DRAIN: std::sync::OnceLock<Option<std::thread::JoinHandle<()>>> =
    std::sync::OnceLock::new();

/// Start the log-file drain once per process, if `HORUS_LOG_FILE` asks for it.
///
/// `Scheduler::new` calls this unconditionally: `start_log_file_drain` reads
/// the variable itself and returns `None` when it is unset, so a process that
/// has not opted in pays one env lookup and spawns nothing.
///
/// Once, because a process may build several `Scheduler`s (tests do, and so
/// does any program that runs one scheduler after another). Each drain thread
/// tracks its own `last_idx` into the same global ring, so N of them would
/// write every entry N times into the same file.
///
/// Returns whether a drain thread is running.
pub fn start_log_file_drain_once() -> bool {
    LOG_FILE_DRAIN.get_or_init(start_log_file_drain).is_some()
}

fn log_drain_loop(log_dir: &str, max_size: u64, max_files: usize) {
    use std::io::Write;

    let dir = std::path::Path::new(log_dir);
    let _ = std::fs::create_dir_all(dir);
    let log_path = dir.join("horus.log");

    let mut last_idx = GLOBAL_LOG_BUFFER.write_idx();
    let mut file = match std::fs::OpenOptions::new()
        .create(true)
        .append(true)
        .open(&log_path)
    {
        Ok(f) => f,
        Err(e) => {
            crate::terminal::eprint_line(&format!(
                "[horus-log-drain] Failed to open log file: {e}"
            ));
            return;
        }
    };

    loop {
        std::thread::sleep(std::time::Duration::from_millis(500));

        let current_idx = GLOBAL_LOG_BUFFER.write_idx();
        if current_idx == last_idx {
            continue;
        }

        // Only the entries pushed since the last pass. Keyed on the
        // buffer-wide write index, NOT on the per-node tick_number.
        let entries = GLOBAL_LOG_BUFFER.get_since(last_idx);
        for entry in &entries {
            let level = match entry.log_type {
                LogType::Error => "ERROR",
                LogType::Warning => "WARN",
                LogType::Info => "INFO",
                LogType::Debug => "DEBUG",
                LogType::Publish => "PUB",
                LogType::Subscribe => "SUB",
            };
            let line = format!(
                "{} [{}] [{}] {}\n",
                entry.timestamp, level, entry.node_name, entry.message
            );
            let _ = file.write_all(line.as_bytes());
        }
        let _ = file.flush();
        last_idx = current_idx;

        // Check rotation
        if let Ok(meta) = std::fs::metadata(&log_path) {
            if meta.len() >= max_size {
                drop(file); // Close current file

                // Rotate: horus.log.4 → delete, .3→.4, .2→.3, .1→.2, .log→.1
                for i in (1..max_files).rev() {
                    let from = dir.join(format!("horus.log.{}", i));
                    let to = dir.join(format!("horus.log.{}", i + 1));
                    let _ = std::fs::rename(&from, &to);
                }
                // Delete oldest if exceeds max_files
                let oldest = dir.join(format!("horus.log.{}", max_files + 1));
                let _ = std::fs::remove_file(&oldest);

                let _ = std::fs::rename(&log_path, dir.join("horus.log.1"));

                file = match std::fs::OpenOptions::new()
                    .create(true)
                    .append(true)
                    .open(&log_path)
                {
                    Ok(f) => f,
                    Err(_) => return,
                };
            }
        }
    }
}

// ─── Tests ────────────────────────────────────────────────────────────────────

impl SharedLogBuffer {
    /// Read the current write index from the mmap header.
    ///
    /// This is a monotonically-increasing counter: each `push()` increments it
    /// by 1.  External tools (e.g. `horus log --follow`) can poll this value to
    /// detect when new entries have been written without fetching all entries.
    pub fn write_idx(&self) -> u64 {
        let guard = self.mmap.lock().unwrap_or_else(|e| e.into_inner());
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

    /// Corrupt a slot's seqlock to an odd value (simulating a crash mid-write).
    ///
    /// **Test-only**: this is deliberately `#[doc(hidden)]` — never use in
    /// production.  Integration tests use this to verify crash-orphaned slot
    /// recovery.
    #[doc(hidden)]
    pub fn corrupt_slot_seqlock(&self, slot_idx: usize, odd_value: u64) {
        // `self.capacity`, NOT `MAX_LOG_ENTRIES`. This buffer may hold far fewer
        // slots than that constant — the error buffer defaults to 500 and the
        // remote buffer to 2000 — and the assert is the only thing standing
        // between a caller-supplied index and `store_slot_seq`'s raw
        // `base.add(HEADER_SIZE + slot_idx * SLOT_SIZE)`. Asserting against 5000
        // let `corrupt_slot_seqlock(4999, 1)` on a 500-slot buffer store 2.5 MB
        // past the end of the mapping.
        assert!(
            slot_idx < self.capacity,
            "slot_idx {slot_idx} out of range (capacity {})",
            self.capacity
        );
        assert!(odd_value & 1 == 1, "value must be odd to simulate crash");
        let guard = self.mmap.lock().unwrap_or_else(|e| e.into_inner());
        let base: *const u8 = (*guard).as_ptr();
        unsafe { store_slot_seq(base, slot_idx, odd_value) };
    }

    /// Read a slot's seqlock value.
    ///
    /// **Test-only**: `#[doc(hidden)]`.
    #[doc(hidden)]
    pub fn read_slot_seqlock(&self, slot_idx: usize) -> u64 {
        // Bound is the mapping's capacity, not the `MAX_LOG_ENTRIES` constant —
        // see `corrupt_slot_seqlock`.
        assert!(
            slot_idx < self.capacity,
            "slot_idx {slot_idx} out of range (capacity {})",
            self.capacity
        );
        let guard = self.mmap.lock().unwrap_or_else(|e| e.into_inner());
        let base: *const u8 = (*guard).as_ptr();
        unsafe { load_slot_seq(base, slot_idx) }
    }

    /// Write raw bytes into a slot's data area and set its seqlock to a
    /// given even value (marking it as "complete" so readers attempt
    /// deserialization).
    ///
    /// **Test-only**: `#[doc(hidden)]`.  Used by corruption resilience tests
    /// to verify `get_all()` gracefully skips garbage data.
    #[doc(hidden)]
    pub fn write_slot_raw(&self, slot_idx: usize, data: &[u8], seq_even: u64) {
        // Bound is the mapping's capacity, not the `MAX_LOG_ENTRIES` constant —
        // see `corrupt_slot_seqlock`. This one writes 504 bytes of caller data,
        // so the wrong bound was an out-of-bounds *write*.
        assert!(
            slot_idx < self.capacity,
            "slot_idx {slot_idx} out of range (capacity {})",
            self.capacity
        );
        assert!(
            seq_even != 0 && seq_even & 1 == 0,
            "seq must be even nonzero"
        );
        let mut guard = self.mmap.lock().unwrap_or_else(|e| e.into_inner());
        let base: *mut u8 = {
            let s: &mut [u8] = &mut guard;
            s.as_mut_ptr()
        };
        let data_off = HEADER_SIZE + slot_idx * SLOT_SIZE + SLOT_SEQ_SIZE;
        let len = data.len().min(SLOT_DATA_SIZE);
        unsafe {
            std::ptr::copy_nonoverlapping(data.as_ptr(), base.add(data_off), len);
            if len < SLOT_DATA_SIZE {
                std::ptr::write_bytes(base.add(data_off + len), 0, SLOT_DATA_SIZE - len);
            }
            store_slot_seq(base, slot_idx, seq_even);
        }
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

    /// The raw slot helpers must be bounded by THIS buffer's capacity, not by
    /// the `MAX_LOG_ENTRIES` constant.
    ///
    /// `MAX_LOG_ENTRIES` is 5000, but the buffers that actually exist are
    /// smaller: the error buffer is 500 slots and the remote buffer 2000, and
    /// the main buffer goes down to 100 under `HORUS_LOG_BUFFER_SIZE`. The
    /// asserts in `corrupt_slot_seqlock` / `read_slot_seqlock` /
    /// `write_slot_raw` are the ONLY thing between a caller-supplied index and
    /// `base.add(HEADER_SIZE + slot_idx * SLOT_SIZE)`; checking them against
    /// 5000 let an index past the end of a 100-slot mapping through, and
    /// `write_slot_raw` then wrote 504 bytes there.
    #[test]
    fn slot_helpers_are_bounded_by_capacity_not_by_the_constant() {
        let path = std::env::temp_dir().join(format!(
            "horus_log_capbound_{}_{}.bin",
            std::process::id(),
            std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .map(|d| d.subsec_nanos())
                .unwrap_or(0)
        ));
        let _ = std::fs::remove_file(&path);

        const SMALL: usize = 100;
        let buf = SharedLogBuffer::open_at_with_capacity(&path, SMALL).expect("open small buffer");
        assert_eq!(
            buf.capacity, SMALL,
            "a freshly created file must give exactly the requested capacity"
        );
        // Both operands are constants, so this belongs at compile time: as a
        // runtime assert it could only ever fire in a build that already
        // compiled, and it is a statement about the test's premise rather than
        // about anything the run observes.
        const _: () = assert!(
            SMALL < MAX_LOG_ENTRIES,
            "this test only means anything while the constant is the looser bound"
        );

        // The last in-range slot is fine.
        buf.write_slot_raw(SMALL - 1, &[7u8; 8], 2);
        assert_eq!(buf.read_slot_seqlock(SMALL - 1), 2);

        // One past it — and anything up to the old constant — must be refused,
        // not written past the end of the mapping.
        for out_of_range in [SMALL, SMALL + 1, MAX_LOG_ENTRIES - 1] {
            let refused = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
                buf.write_slot_raw(out_of_range, &[0xAAu8; 8], 2);
            }));
            assert!(
                refused.is_err(),
                "write_slot_raw({out_of_range}) must be refused on a {SMALL}-slot buffer"
            );

            let refused = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
                buf.corrupt_slot_seqlock(out_of_range, 1);
            }));
            assert!(
                refused.is_err(),
                "corrupt_slot_seqlock({out_of_range}) must be refused on a {SMALL}-slot buffer"
            );

            let refused = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
                buf.read_slot_seqlock(out_of_range);
            }));
            assert!(
                refused.is_err(),
                "read_slot_seqlock({out_of_range}) must be refused on a {SMALL}-slot buffer"
            );
        }

        let _ = std::fs::remove_file(&path);
    }

    /// `capacity` must never exceed what the mapping can actually hold, because
    /// every `unsafe` slot access derives its bound from it.
    #[test]
    fn capacity_never_exceeds_the_mapping() {
        let (buf, path) = temp_buf(9_001);
        let mapped = buf.mmap.lock().unwrap_or_else(|e| e.into_inner()).len();
        assert!(
            HEADER_SIZE + buf.capacity * SLOT_SIZE <= mapped,
            "capacity {} implies {} bytes but only {} are mapped",
            buf.capacity,
            HEADER_SIZE + buf.capacity * SLOT_SIZE,
            mapped
        );
        let _ = std::fs::remove_file(path);
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

    /// A drain loop that remembers the `write_idx` it last consumed must see
    /// every entry exactly once — regardless of what each entry's
    /// `tick_number` happens to be.
    ///
    /// This pins the S-08 defect: the drain used to filter on
    /// `entry.tick_number <= last_idx`, comparing a per-node tick counter
    /// against a buffer-wide index. Entries from the `log::` facade carry
    /// tick 0 and were dropped forever; entries from a node whose tick had
    /// run ahead of the buffer index were re-emitted on every pass.
    #[test]
    fn get_since_yields_each_entry_exactly_once() {
        let (buf, path) = temp_buf(90);

        // Deliberately adversarial tick_numbers: facade entries at 0, and a
        // fast node whose tick counter has run far past the buffer index.
        buf.push(make_entry("facade", 0));
        buf.push(make_entry("fast_node", 5_000));
        buf.push(make_entry("facade", 0));

        let mut last_idx = 0u64;
        let mut drained: Vec<String> = Vec::new();

        // Pass 1 — everything so far.
        let batch = buf.get_since(last_idx);
        drained.extend(batch.iter().map(|e| e.node_name.clone()));
        last_idx = buf.write_idx();
        assert_eq!(batch.len(), 3, "first pass must drain all three entries");

        // Pass 2 — nothing new was pushed, so nothing may be re-emitted.
        let batch = buf.get_since(last_idx);
        assert!(
            batch.is_empty(),
            "no new pushes, yet {} entries were drained again",
            batch.len()
        );

        // Pass 3 — one more entry, again with a tick_number below last_idx.
        buf.push(make_entry("facade", 0));
        let batch = buf.get_since(last_idx);
        drained.extend(batch.iter().map(|e| e.node_name.clone()));
        assert_eq!(batch.len(), 1, "exactly the one new entry must be drained");

        assert_eq!(drained, vec!["facade", "fast_node", "facade", "facade"]);
        assert_eq!(buf.get_all().len(), 4, "get_all must still return all");

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

        // push() truncates at MAX_MESSAGE_LEN (280) and appends "..."
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

    /// All variable-length fields are truncated before serialization so the
    /// deserialized struct is always valid (no mid-struct byte truncation).
    #[test]
    fn test_all_fields_truncated_safely() {
        let (buf, path) = temp_buf(15);

        let entry = LogEntry {
            timestamp: "T".repeat(200), // way over 32-byte cap
            tick_number: 1,
            node_name: "N".repeat(200), // way over 64-byte cap
            log_type: LogType::Info,
            topic: Some("X".repeat(200)), // way over 64-byte cap
            message: "M".repeat(1000),    // way over 300-byte cap
            tick_us: 0,
            ipc_ns: 0,
        };
        buf.push(entry);

        let all = buf.get_all();
        assert_eq!(all.len(), 1, "oversized entry must still be readable");

        let e = &all[0];
        assert!(e.message.len() <= MAX_MESSAGE_LEN, "message too long");
        assert!(
            e.message.ends_with("..."),
            "message must show truncation marker"
        );
        assert!(e.node_name.len() <= 67, "node_name too long"); // 64 + "..."
        assert!(
            e.node_name.ends_with("..."),
            "node_name must show truncation marker"
        );
        assert!(e.timestamp.len() <= 35, "timestamp too long"); // 32 + "..."
        assert!(
            e.timestamp.ends_with("..."),
            "timestamp must show truncation marker"
        );
        let topic = e.topic.as_ref().unwrap();
        assert!(topic.len() <= 67, "topic too long");
        assert!(topic.ends_with("..."), "topic must show truncation marker");

        let _ = std::fs::remove_file(path);
    }

    /// Verify that truncate_field works correctly on char boundaries.
    #[test]
    fn test_truncate_field_unicode_boundary() {
        // 4-byte emoji repeated — truncation must not split a multi-byte char
        let mut s = "🤖".repeat(100); // 400 bytes
        super::truncate_field(&mut s, 20);
        assert!(s.len() <= 20);
        assert!(s.ends_with("..."));
        // Must be valid UTF-8 (String guarantees, but let's be explicit)
        let _ = s.chars().count();
    }

    /// Simulate a crash-orphaned slot by manually writing an odd seqlock.
    /// The reader must not hang — it should skip the slot after the timeout
    /// and reset its seqlock to even for future reclamation.
    #[test]
    fn test_crash_orphaned_seqlock_skipped() {
        let (buf, path) = temp_buf(16);

        // Write two valid entries
        buf.push(make_entry("before_crash", 1));
        buf.push(make_entry("after_crash", 2));

        // Manually corrupt slot 0's seqlock to odd (simulates a crashed writer)
        {
            let guard = buf.mmap.lock().unwrap();
            let base: *const u8 = (*guard).as_ptr();
            unsafe { store_slot_seq(base, 0, 99) }; // 99 is odd → "write in progress"
        }

        // get_all() must complete (not hang) and skip the orphaned slot
        let start = std::time::Instant::now();
        let all = buf.get_all();
        let elapsed = start.elapsed();

        // Should only get the non-corrupted entry
        assert_eq!(all.len(), 1, "must skip the crash-orphaned slot");
        assert_eq!(all[0].node_name, "after_crash");

        // Must complete within a reasonable time (SEQLOCK_SPIN_TIMEOUT=5ms + margin)
        assert!(
            elapsed < std::time::Duration::from_millis(50),
            "reader hung on orphaned slot ({:?})",
            elapsed
        );

        // The orphaned slot's seqlock should now be reset to even
        {
            let guard = buf.mmap.lock().unwrap();
            let base: *const u8 = (*guard).as_ptr();
            let seq = unsafe { load_slot_seq(base, 0) };
            assert!(
                seq & 1 == 0,
                "orphaned slot seqlock should be reset to even, got {}",
                seq
            );
        }

        let _ = std::fs::remove_file(path);
    }

    /// Multiple crash-orphaned slots: reader skips all of them and reads the
    /// remaining valid entries.
    #[test]
    fn test_multiple_crash_orphaned_slots() {
        let (buf, path) = temp_buf(17);

        // Write 5 valid entries
        for i in 0..5 {
            buf.push(make_entry(&format!("node_{}", i), i));
        }

        // Corrupt slots 1 and 3
        {
            let guard = buf.mmap.lock().unwrap();
            let base: *const u8 = (*guard).as_ptr();
            unsafe {
                store_slot_seq(base, 1, 77); // odd
                store_slot_seq(base, 3, 55); // odd
            }
        }

        let all = buf.get_all();
        // Slots 0, 2, 4 should be readable; slots 1, 3 are corrupted
        assert_eq!(all.len(), 3, "must skip both orphaned slots");

        let names: Vec<&str> = all.iter().map(|e| e.node_name.as_str()).collect();
        assert!(names.contains(&"node_0"));
        assert!(names.contains(&"node_2"));
        assert!(names.contains(&"node_4"));

        let _ = std::fs::remove_file(path);
    }

    // ── Namespace verification ─────────────────────────────────────────

    #[test]
    fn log_buffer_path_is_namespaced() {
        let logs = crate::memory::platform::shm_logs_path();
        let base = crate::memory::platform::shm_base_dir();
        assert!(
            logs.starts_with(&base),
            "log path '{}' should be inside namespace base '{}'",
            logs.display(),
            base.display()
        );
    }

    #[test]
    fn log_buffer_path_ends_with_logs() {
        let logs = crate::memory::platform::shm_logs_path();
        assert!(
            logs.ends_with("logs"),
            "log path should end with 'logs', got: {}",
            logs.display()
        );
    }

    // ── Discovery module removal verification ──────────────────────────

    #[test]
    fn discovery_module_file_deleted() {
        let manifest_dir = env!("CARGO_MANIFEST_DIR");
        let discovery_path = std::path::Path::new(manifest_dir).join("src/core/discovery.rs");
        assert!(
            !discovery_path.exists(),
            "core/discovery.rs should be deleted (dead discovery topic code)"
        );
    }
}
