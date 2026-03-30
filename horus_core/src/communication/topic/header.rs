#![allow(private_interfaces)]
//! Shared memory header for topic detection.
//!
//! The `TopicHeader` is laid out in shared memory for cross-process
//! rendezvous. It contains participant tracking, topology detection, and
//! migration coordination.

use std::mem;
use std::sync::atomic::{AtomicU32, AtomicU64, AtomicU8, Ordering};

use crate::error::{HorusError, HorusResult};

use super::types::BackendMode;

// ============================================================================
// Constants
// ============================================================================

/// Magic number for topic header validation
pub(crate) const TOPIC_MAGIC: u64 = 0x4144415054495645; // "ADAPTIVE" (kept for backwards compat)

/// Header version for compatibility checking
/// v2: Added slot_size field for large message support
/// v3: Added per-slot sequence array for multi-producer write-completion tracking
pub(crate) const TOPIC_VERSION: u32 = 3;

/// Default lease timeout in milliseconds (5 seconds)
pub(crate) const DEFAULT_LEASE_TIMEOUT_MS: u64 = 5000;

/// POD flag values
pub(crate) const POD_NO: u8 = 1;
pub(crate) const POD_YES: u8 = 2;

/// Migration lock states
pub(crate) const MIGRATION_UNLOCKED: u8 = 0;
pub(crate) const MIGRATION_LOCKED: u8 = 1;

/// Maximum number of participants to track for lease management
/// With 24-byte entries: 16 * 24 = 384 bytes (6 cache lines)
pub(crate) const MAX_PARTICIPANTS: usize = 16;

// ============================================================================
// Participant Entry
// ============================================================================

/// Participant entry in the header (24 bytes, cache-friendly)
#[repr(C)]
#[derive(Debug)]
pub(crate) struct ParticipantEntry {
    /// Process ID (0 = empty slot)
    /// MUST be atomic for cross-process visibility in shared memory
    pub(crate) pid: AtomicU32,
    /// Thread ID hash (lower 32 bits for same-thread detection)
    /// MUST be atomic for cross-process visibility in shared memory
    pub(crate) thread_id_hash: AtomicU32,
    /// Role: 0=none, 1=producer, 2=consumer, 3=both
    pub(crate) role: AtomicU8,
    /// Active flag for atomic operations
    pub(crate) active: AtomicU8,
    /// Source host identifier for network-replicated participants.
    /// 0 = local process (default). Non-zero = low byte of remote peer_id_hash.
    /// Set by horus_net ShmRingWriter when writing network-received data.
    pub(crate) source_host: AtomicU8,
    /// Padding for alignment
    pub(crate) _pad: [u8; 5],
    /// Last heartbeat timestamp (milliseconds since epoch)
    pub(crate) lease_expires_ms: AtomicU64,
}

impl ParticipantEntry {
    /// Check if the lease has expired
    #[inline]
    pub(crate) fn is_lease_expired(&self, now_ms: u64) -> bool {
        let expires = self.lease_expires_ms.load(Ordering::Acquire);
        if expires == 0 {
            return true;
        }
        now_ms > expires
    }

    /// Update lease expiration timestamp
    #[inline]
    pub(crate) fn refresh_lease(&self, now_ms: u64, timeout_ms: u64) {
        self.lease_expires_ms
            .store(now_ms + timeout_ms, Ordering::Release);
    }

    /// Clear this entry (use atomic for thread safety)
    pub(crate) fn clear(&self) {
        self.active.store(0, Ordering::Release);
        self.lease_expires_ms.store(0, Ordering::Release);
        self.role.store(0, Ordering::Release);
        self.pid.store(0, Ordering::Release);
        self.thread_id_hash.store(0, Ordering::Release);
        self.source_host.store(0, Ordering::Release);
    }
}

// ============================================================================
// Topic Header (Cache-Optimized)
// ============================================================================

/// Topic classification for discovery and introspection.
///
/// Stored in the `topic_kind` field of `TopicHeader`. External tools use this
/// to distinguish data topics from service/action transport topics without
/// relying on naming conventions (`.request`/`.response` suffixes).
#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TopicKind {
    /// Normal data topic (default)
    Data = 0,
    /// Service request channel (`{name}.request`)
    ServiceRequest = 1,
    /// Service response channel (`{name}.response`)
    ServiceResponse = 2,
    /// Action goal channel (`{name}/goal`)
    ActionGoal = 3,
    /// Action feedback channel (`{name}/feedback`)
    ActionFeedback = 4,
    /// Action result channel (`{name}/result`)
    ActionResult = 5,
    /// Action status channel (`{name}/status`)
    ActionStatus = 6,
    /// Action cancel channel (`{name}/cancel`)
    ActionCancel = 7,
    /// Internal system topic (e.g. `horus.ctl.*`)
    System = 8,
}

impl TopicKind {
    /// Convert from raw u8 value, defaulting to Data for unknown values.
    pub fn from_u8(v: u8) -> Self {
        match v {
            1 => Self::ServiceRequest,
            2 => Self::ServiceResponse,
            3 => Self::ActionGoal,
            4 => Self::ActionFeedback,
            5 => Self::ActionResult,
            6 => Self::ActionStatus,
            7 => Self::ActionCancel,
            8 => Self::System,
            _ => Self::Data,
        }
    }
}

/// Shared memory header for topic detection.
///
/// This header enables fully automatic backend selection based on:
/// - Thread ID comparison (same-thread detection)
/// - PID comparison (same-process detection)
/// - Publisher/subscriber counts (access pattern detection)
/// - POD type registration (zero-copy optimization)
///
/// ## Cache Line Optimization
///
/// **CRITICAL**: Producer (head) and consumer (tail) are on SEPARATE cache lines
/// to prevent false sharing. This is the single most important optimization for
/// achieving sub-20ns latency.
///
/// Layout: 640 bytes (10 cache lines)
/// - Cache line 1: Core metadata (read-mostly)
/// - Cache line 2: PRODUCER ONLY - sequence_or_head (written by sender)
/// - Cache line 3: CONSUMER ONLY - tail (written by receiver)
/// - Cache line 4: Counters and timestamps
/// - Cache lines 5-10: Participant tracking
#[repr(C, align(64))]
pub(crate) struct TopicHeader {
    // === Cache line 1 (bytes 0-63): Core metadata (read-mostly) ===
    /// Magic number for validation
    pub(crate) magic: u64,
    /// Header version for compatibility
    pub(crate) version: u32,
    /// Type size in bytes
    pub(crate) type_size: u32,
    /// Type alignment requirement
    pub(crate) type_align: u32,
    /// Is POD type: 0=unknown, 1=no, 2=yes
    pub(crate) is_pod: AtomicU8,
    /// Current backend mode (BackendMode as u8)
    pub(crate) backend_mode: AtomicU8,
    /// Migration lock: 0=unlocked, 1=locked
    pub(crate) migration_lock: AtomicU8,
    /// Verbose content logging flag — toggled at runtime by TUI monitor.
    /// 0 = disabled (default), non-zero = log send/recv content to GLOBAL_LOG_BUFFER.
    /// Note: metrics (messages_total) are always collected regardless of this flag.
    pub(crate) verbose: AtomicU8,
    /// Creator process ID
    pub(crate) creator_pid: u32,
    /// Creator thread ID hash (for same-thread detection)
    pub(crate) creator_thread_id_hash: u64,
    /// Migration epoch (incremented on each backend switch)
    pub(crate) migration_epoch: AtomicU64,
    /// Topic kind: Data=0, ServiceRequest=1, ServiceResponse=2, etc.
    /// Set once at topic creation. Used by discovery to classify topics.
    pub(crate) topic_kind: u8,
    /// Alignment padding for messages_total
    pub(crate) _pad1a: [u8; 7],
    /// Total messages ever sent on this topic (always-on atomic counter).
    /// Incremented on every send() regardless of verbose flag.
    pub(crate) messages_total: AtomicU64,

    // === Cache line 2 (bytes 64-127): PRODUCER WRITE LINE ===
    // This cache line is ONLY written by producers (senders)
    // NEVER put consumer-written fields here!
    /// Write sequence / head (for POD/ring backends) - PRODUCER ONLY
    pub(crate) sequence_or_head: AtomicU64,
    /// Ring buffer capacity (power of 2)
    pub(crate) capacity: u32,
    /// Capacity mask for fast modulo (capacity - 1, only valid if capacity is power of 2)
    pub(crate) capacity_mask: u32,
    /// Slot size in bytes (for non-POD types, includes header + data)
    pub(crate) slot_size: u32,
    /// Padding to fill cache line (64 - 8 - 4 - 4 - 4 = 44 bytes)
    pub(crate) _pad_producer: [u8; 44],

    // === Cache line 3 (bytes 128-191): CONSUMER WRITE LINE ===
    // This cache line is ONLY written by consumers (receivers)
    // NEVER put producer-written fields here!
    /// Read tail (for ring backends) - CONSUMER ONLY
    pub(crate) tail: AtomicU64,
    /// Padding to fill cache line (64 - 8 = 56 bytes)
    pub(crate) _pad_consumer: [u8; 56],

    // === Cache line 4 (bytes 192-255): Counters and metadata ===
    /// Number of active publishers
    pub(crate) publisher_count: AtomicU32,
    /// Number of active subscribers
    pub(crate) subscriber_count: AtomicU32,
    /// Total participants ever connected
    pub(crate) total_participants: AtomicU32,
    /// Lease timeout in milliseconds
    pub(crate) lease_timeout_ms: u32,
    /// Last topology change timestamp (ms)
    pub(crate) last_topology_change_ms: AtomicU64,
    /// Message type name (null-terminated, e.g. "CmdVel", "Imu").
    /// Set once at topic creation via `std::any::type_name::<T>()`.
    /// External tools read this directly from the mmap'd header.
    pub(crate) type_name: [u8; 32],
    /// Reserved for future use
    pub(crate) _pad_counters: [u8; 8],

    // === Cache lines 5-10 (bytes 256-639): Participant tracking (384 bytes = 16 * 24) ===
    /// Participant entries for lease management
    pub(crate) participants: [ParticipantEntry; MAX_PARTICIPANTS],
}

// Size assertion: Header must be exactly 640 bytes (10 cache lines)
const _: () = assert!(mem::size_of::<TopicHeader>() == 640);

impl TopicHeader {
    #[cfg(test)]
    pub fn zeroed() -> Self {
        Self {
            magic: 0,
            version: 0,
            type_size: 0,
            type_align: 0,
            is_pod: AtomicU8::new(0),
            backend_mode: AtomicU8::new(0),
            migration_lock: AtomicU8::new(0),
            verbose: AtomicU8::new(0),
            creator_pid: 0,
            creator_thread_id_hash: 0,
            migration_epoch: AtomicU64::new(0),
            topic_kind: 0,
            _pad1a: [0; 7],
            messages_total: AtomicU64::new(0),
            // Cache line 2: Producer write line
            sequence_or_head: AtomicU64::new(0),
            capacity: 0,
            capacity_mask: 0,
            slot_size: 0,
            _pad_producer: [0; 44],
            // Cache line 3: Consumer write line
            tail: AtomicU64::new(0),
            _pad_consumer: [0; 56],
            // Cache line 4: Counters
            publisher_count: AtomicU32::new(0),
            subscriber_count: AtomicU32::new(0),
            total_participants: AtomicU32::new(0),
            lease_timeout_ms: 0,
            last_topology_change_ms: AtomicU64::new(0),
            type_name: [0; 32],
            _pad_counters: [0; 8],
            participants: std::array::from_fn(|_| ParticipantEntry {
                pid: AtomicU32::new(0),
                thread_id_hash: AtomicU32::new(0),
                role: AtomicU8::new(0),
                active: AtomicU8::new(0),
                source_host: AtomicU8::new(0),
                _pad: [0; 5],
                lease_expires_ms: AtomicU64::new(0),
            }),
        }
    }

    /// Initialize a new header
    // All 7 arguments are required by the SHM header wire format — grouping them
    // into a struct would add overhead at the only call site (topic open/create).
    #[allow(clippy::too_many_arguments)]
    pub fn init(
        &mut self,
        type_size: u32,
        type_align: u32,
        is_pod: bool,
        capacity: u32,
        slot_size: u32,
        type_name_str: &str,
        topic_kind: u8,
    ) {
        // Ensure capacity is power of 2 for fast modulo
        let capacity = capacity.next_power_of_two();

        // Initialize all fields BEFORE setting magic (race condition fix)
        // Another process might check magic to determine if header is initialized
        self.version = TOPIC_VERSION;
        self.type_size = type_size;
        self.type_align = type_align;
        self.is_pod
            .store(if is_pod { POD_YES } else { POD_NO }, Ordering::Release);
        self.backend_mode
            .store(BackendMode::Unknown as u8, Ordering::Release);
        self.migration_lock
            .store(MIGRATION_UNLOCKED, Ordering::Release);
        self.verbose.store(0, Ordering::Relaxed);
        self.creator_pid = std::process::id();
        self.creator_thread_id_hash = hash_thread_id(std::thread::current().id());
        self.migration_epoch.store(0, Ordering::Release);
        self.topic_kind = topic_kind;
        self.messages_total.store(0, Ordering::Release);

        // Cache line 2: Producer write line
        self.sequence_or_head.store(0, Ordering::Release);
        self.capacity = capacity;
        self.capacity_mask = capacity.wrapping_sub(1); // For bitwise AND instead of modulo
        self.slot_size = slot_size;

        // Cache line 3: Consumer write line
        self.tail.store(0, Ordering::Release);

        // Cache line 4: Counters
        self.publisher_count.store(0, Ordering::Release);
        self.subscriber_count.store(0, Ordering::Release);
        self.total_participants.store(0, Ordering::Release);
        self.lease_timeout_ms = DEFAULT_LEASE_TIMEOUT_MS as u32;
        self.last_topology_change_ms
            .store(current_time_ms(), Ordering::Release);

        // Cache line 4 (cont): type_name — null-terminated, truncated if needed
        self.type_name = [0u8; 32];
        let name_bytes = type_name_str.as_bytes();
        let copy_len = name_bytes.len().min(31); // leave room for null terminator
        self.type_name[..copy_len].copy_from_slice(&name_bytes[..copy_len]);

        // Clear all participant entries
        for p in &self.participants {
            p.clear();
        }

        // CRITICAL: Set magic LAST with a memory fence to ensure all prior writes
        // are visible to other processes before they see the magic value.
        std::sync::atomic::fence(Ordering::Release);
        self.magic = TOPIC_MAGIC;
    }

    /// Get the current backend mode
    #[inline]
    pub fn mode(&self) -> BackendMode {
        BackendMode::from(self.backend_mode.load(Ordering::Acquire))
    }

    /// Check if verbose content logging is enabled (toggled by TUI monitor).
    /// This only controls content logging (log_summary text), not metrics.
    #[inline(always)]
    pub fn is_verbose(&self) -> bool {
        self.verbose.load(Ordering::Relaxed) != 0
    }

    /// Read the type name stored in the header as a string.
    ///
    /// Returns the null-terminated string from the `type_name` field, or an
    /// empty string if the field is all zeros (header from before this field
    /// was added).
    #[allow(dead_code)] // public API: called by external tools via read_topic_header_info
    pub fn type_name_str(&self) -> &str {
        let end = self
            .type_name
            .iter()
            .position(|&b| b == 0)
            .unwrap_or(self.type_name.len());
        std::str::from_utf8(&self.type_name[..end]).unwrap_or("")
    }

    /// Read the total messages sent counter.
    #[inline]
    #[allow(dead_code)] // public API: called by external tools and tests
    pub fn messages_total(&self) -> u64 {
        self.messages_total.load(Ordering::Relaxed)
    }

    /// Read the topic kind classification.
    #[inline]
    #[allow(dead_code)] // public API: called by external tools and tests
    pub fn topic_kind(&self) -> TopicKind {
        TopicKind::from_u8(self.topic_kind)
    }

    /// Check if all active participants (and caller) are in the same process.
    ///
    /// Also returns `false` when the SHM region was created by a different process
    /// (creator_pid ≠ current PID), even if that process hasn't registered as a
    /// participant yet. This ensures joiners default to cross-process backends
    /// immediately, avoiding the race where a producer sends to a heap ring and
    /// exits before the consumer can trigger migration to SHM.
    #[inline]
    pub fn is_same_process(&self) -> bool {
        let current_pid = std::process::id();

        // If the SHM region was created by a different process, assume cross-process
        // even if the creator hasn't registered in the participant table yet.
        if self.creator_pid != 0 && self.creator_pid != current_pid {
            return false;
        }

        for p in &self.participants {
            if p.active.load(Ordering::Acquire) != 0 && p.pid.load(Ordering::Acquire) != current_pid
            {
                return false;
            }
        }

        true
    }

    /// Check if caller is on the same thread as creator
    #[inline]
    #[cfg(test)]
    pub fn is_same_thread(&self) -> bool {
        self.is_same_process()
            && self.creator_thread_id_hash == hash_thread_id(std::thread::current().id())
    }

    /// Check if ALL active participants are on the same thread.
    ///
    /// This is the correct check for DirectChannel mode - it requires that
    /// all registered producers AND consumers share the same thread.
    #[inline]
    pub fn all_participants_same_thread(&self) -> bool {
        if !self.is_same_process() {
            return false;
        }

        let current_thread_hash = hash_thread_id(std::thread::current().id()) as u32;
        let mut reference_thread: Option<u32> = None;

        for p in &self.participants {
            if p.active.load(Ordering::Acquire) != 0 {
                if p.pid.load(Ordering::Acquire) != std::process::id() {
                    return false;
                }

                let thread_hash = p.thread_id_hash.load(Ordering::Acquire);
                match reference_thread {
                    None => reference_thread = Some(thread_hash),
                    Some(ref_hash) => {
                        if thread_hash != ref_hash {
                            return false;
                        }
                    }
                }
            }
        }

        if let Some(ref_hash) = reference_thread {
            return current_thread_hash == ref_hash;
        }

        true
    }

    /// Check if type is registered as POD
    #[inline]
    pub fn is_pod_type(&self) -> bool {
        self.is_pod.load(Ordering::Acquire) == POD_YES
    }

    /// Get publisher count
    #[inline]
    pub fn pub_count(&self) -> u32 {
        self.publisher_count.load(Ordering::Acquire)
    }

    /// Get subscriber count
    #[inline]
    pub fn sub_count(&self) -> u32 {
        self.subscriber_count.load(Ordering::Acquire)
    }

    /// Register as a publisher (returns slot index or error)
    pub fn register_producer(&self) -> HorusResult<usize> {
        self.register_role(1, &self.publisher_count)
    }

    /// Register as a subscriber (returns slot index or error)
    pub fn register_consumer(&self) -> HorusResult<usize> {
        self.register_role(2, &self.subscriber_count)
    }

    /// Shared registration logic for producer (role_bit=1) and consumer (role_bit=2).
    fn register_role(&self, role_bit: u8, counter: &AtomicU32) -> HorusResult<usize> {
        let now_ms = current_time_ms();
        let pid = std::process::id();
        let thread_hash = hash_thread_id(std::thread::current().id()) as u32;
        let timeout_ms = self.lease_timeout_ms as u64;

        // First, try to find existing entry for this thread
        for (i, p) in self.participants.iter().enumerate() {
            if p.active.load(Ordering::Acquire) != 0
                && p.pid.load(Ordering::Acquire) == pid
                && p.thread_id_hash.load(Ordering::Acquire) == thread_hash
            {
                let old_role = p.role.fetch_or(role_bit, Ordering::AcqRel);
                if old_role & role_bit == 0 {
                    counter.fetch_add(1, Ordering::AcqRel);
                    self.last_topology_change_ms
                        .store(now_ms, Ordering::Release);
                }
                p.refresh_lease(now_ms, timeout_ms);
                return Ok(i);
            }
        }

        // Find empty slot or expired lease.
        //
        // We use active=2 as an "initializing" sentinel so that a freshly
        // claimed slot (whose lease_expires_ms is still 0) is not mistaken
        // for an expired slot by another thread.
        for (i, p) in self.participants.iter().enumerate() {
            let active_val = p.active.load(Ordering::Acquire);
            if active_val == 2 {
                continue; // Another thread is initializing this slot
            }
            let is_active = active_val != 0;
            let is_expired = is_active && p.is_lease_expired(now_ms);

            if !is_active || is_expired {
                // Try to claim the slot via CAS → 2 (initializing).
                // Exactly one thread wins per slot.
                let claimed = p
                    .active
                    .compare_exchange(active_val, 2, Ordering::AcqRel, Ordering::Acquire)
                    .is_ok();

                if claimed {
                    // Now that we own the slot, safely decrement the old role counters
                    if is_expired {
                        let old_role = p.role.load(Ordering::Acquire);
                        if old_role & 1 != 0 {
                            self.publisher_count.fetch_sub(1, Ordering::AcqRel);
                        }
                        if old_role & 2 != 0 {
                            self.subscriber_count.fetch_sub(1, Ordering::AcqRel);
                        }
                    }

                    p.pid.store(pid, Ordering::Release);
                    p.thread_id_hash.store(thread_hash, Ordering::Release);
                    p.role.store(role_bit, Ordering::Release);
                    p.refresh_lease(now_ms, timeout_ms);
                    counter.fetch_add(1, Ordering::AcqRel);
                    self.total_participants.fetch_add(1, Ordering::AcqRel);
                    self.last_topology_change_ms
                        .store(now_ms, Ordering::Release);
                    // Finalize: transition from initializing (2) to active (1).
                    // This makes the slot visible with all fields properly set.
                    p.active.store(1, Ordering::Release);
                    return Ok(i);
                }
            }
        }

        // All 16 participant slots occupied by live processes.
        // Count active for diagnostics.
        let active = self
            .participants
            .iter()
            .filter(|p| p.active.load(Ordering::Relaxed) != 0)
            .count();
        Err(HorusError::Communication(
            format!(
                "No available participant slots ({}/{} active). \
                 Consider reducing the number of concurrent publishers/subscribers \
                 on this topic, or increasing lease timeout to allow faster reclamation.",
                active, MAX_PARTICIPANTS
            )
            .into(),
        ))
    }

    /// Try to acquire migration lock
    #[inline]
    pub fn try_lock_migration(&self) -> bool {
        self.migration_lock
            .compare_exchange(
                MIGRATION_UNLOCKED,
                MIGRATION_LOCKED,
                Ordering::AcqRel,
                Ordering::Acquire,
            )
            .is_ok()
    }

    /// Release migration lock
    #[inline]
    pub fn unlock_migration(&self) {
        self.migration_lock
            .store(MIGRATION_UNLOCKED, Ordering::Release);
    }

    /// Detect the optimal backend based on current topology.
    ///
    /// SHM-backed topics (creator_pid != 0) always use SHM backends, even for
    /// same-process communication. This ensures data is written to the shared
    /// memory region from the first message, so cross-process consumers that
    /// join later can immediately read it without waiting for a migration.
    ///
    /// Intra-process heap backends (DirectChannel, SpscIntra, etc.) are only
    /// used for topics created without SHM backing (in-memory only).
    pub fn detect_optimal_backend(&self) -> BackendMode {
        let pubs = self.pub_count();
        let subs = self.sub_count();
        let same_process = self.is_same_process();
        let all_same_thread = self.all_participants_same_thread();
        let is_pod = self.is_pod_type();
        // SHM-backed: creator_pid is set during header.init() from the owner process.
        // If non-zero, this topic has a backing SHM file and should always use SHM
        // backends to ensure cross-process readiness from the first message.
        let shm_backed = self.creator_pid != 0;

        match (all_same_thread, same_process, pubs, subs, is_pod) {
            (_, _, 0, 0, _) => BackendMode::Unknown,

            // SHM-backed topics: skip heap backends, go straight to SHM.
            // This is the key fix for cross-process reliability: data lands in
            // the mmap'd region immediately, so a consumer joining from another
            // process sees it without needing a migration + drain cycle.
            _ if shm_backed && (pubs + subs > 0) => {
                if !same_process {
                    // Already detected cross-process
                    match (pubs, subs) {
                        (1, _) if subs <= 1 => BackendMode::SpscShm,
                        (_, 1) if pubs > 1 => BackendMode::MpscShm,
                        (1, _) if subs > 1 => BackendMode::SpmcShm,
                        (_, _) if is_pod => BackendMode::PodShm,
                        _ => BackendMode::FanoutShm,
                    }
                } else {
                    // Same process but SHM-backed: use SHM anyway for cross-process readiness
                    match (pubs, subs) {
                        (p, s) if p <= 1 && s <= 1 => BackendMode::SpscShm,
                        (1, _) if subs > 1 => BackendMode::SpmcShm,
                        (_, _) if pubs > 1 && subs <= 1 => BackendMode::MpscShm,
                        (_, _) if is_pod => BackendMode::PodShm,
                        _ => BackendMode::FanoutShm,
                    }
                }
            }

            (true, _, 1, 1, true) => BackendMode::DirectChannel,

            // Intra-process: 1P↔1C (or anticipating single counterpart)
            (_, true, 1, 1, _) => BackendMode::SpscIntra,
            (_, true, 1, 0, _) => BackendMode::SpscIntra,
            (_, true, 0, 1, _) => BackendMode::SpscIntra,
            // Intra-process: 1P, multiple consumers — use FanoutIntra so each
            // subscriber gets its own SPSC channel (broadcast semantics).
            // SpmcIntra has a shared tail pointer where consumers compete for
            // messages — one fast consumer drains the ring, starving others.
            // FanoutIntra gives each subscriber an independent copy of every message.
            (_, true, 1, _, _) if subs > 1 => BackendMode::FanoutIntra,
            // Intra-process: multiple producers (0 or 1 consumer)
            (_, true, _, 0, _) if pubs > 1 => BackendMode::MpscIntra,
            (_, true, _, 1, _) if pubs > 1 => BackendMode::MpscIntra,
            // Intra-process: 0 pubs, multiple consumers (anticipating single producer)
            (_, true, 0, _, _) if subs > 1 => BackendMode::FanoutIntra,
            // Intra-process: MPMC — use FanoutRing (contention-free SPSC matrix)
            (_, true, _, _, _) if pubs > 1 && subs > 1 => BackendMode::FanoutIntra,

            // Cross-process: 1P↔1C
            (_, false, 1, 1, _) => BackendMode::SpscShm,
            (_, false, 1, 0, _) => BackendMode::SpscShm,
            (_, false, 0, 1, _) => BackendMode::SpscShm,
            // Cross-process: multi-producer
            (_, false, _, 0, _) if pubs > 1 => BackendMode::MpscShm,
            (_, false, _, 1, _) if pubs > 1 => BackendMode::MpscShm,
            // Cross-process: multi-consumer — FanoutShm for broadcast semantics
            // (same reasoning as intra-process: SpmcShm has competing consumers)
            (_, false, 1, _, _) if subs > 1 => BackendMode::FanoutShm,
            (_, false, 0, _, _) if subs > 1 => BackendMode::FanoutShm,

            (_, false, _, _, true) => BackendMode::PodShm,

            // Cross-process MPMC — use FanoutShm (contention-free SHM SPSC matrix)
            _ => BackendMode::FanoutShm,
        }
    }
}

// ============================================================================
// Public debug flag API (for external tools like the TUI monitor)
// ============================================================================

/// Byte offset of the `verbose` flag within the topic shared memory header.
/// External tools can write 1/0 to this offset in the mmap'd file to
/// enable/disable verbose content logging for a topic.
pub const TOPIC_VERBOSE_OFFSET: usize = 23;

/// Set the verbose content logging flag on a topic's shared memory region.
///
/// # Safety
/// `shm_ptr` must point to the start of a valid, initialized topic shared
/// memory region (at least 640 bytes). The caller must have write access.
pub unsafe fn set_topic_verbose(shm_ptr: *mut u8, enabled: bool) {
    let flag = shm_ptr.add(TOPIC_VERBOSE_OFFSET);
    flag.write_volatile(if enabled { 1 } else { 0 });
}

// ============================================================================
// Public ring-buffer inspector (for CLI tools like `horus topic echo`)
// ============================================================================

/// Total byte size of `TopicHeader` (= 640, 10 × 64-byte cache lines).
pub const TOPIC_HEADER_SIZE: usize = mem::size_of::<TopicHeader>();

/// Result returned by `read_latest_slot_bytes`.
pub struct TopicSlotRead {
    /// Raw payload bytes of the most-recently-written message slot.
    /// For POD types this is the raw struct bytes.  For non-POD types this
    /// is the `bincode`-serialized wire form.
    pub payload: Vec<u8>,
    /// Monotonic write counter at the time of the read.  Pass this back to
    /// `read_latest_slot_bytes` as `last_write_idx` on the next poll; if
    /// the value is unchanged, no new message has been published.
    pub write_idx: u64,
    /// `true` when the message type is a POD (plain-old-data) type, meaning
    /// `payload` contains raw struct bytes rather than a `bincode` stream.
    pub is_pod: bool,
    /// Message type name read from the header (e.g. "CmdVel", "Imu").
    /// Empty string if the header predates the type_name field.
    pub type_name: String,
    /// Total messages ever sent on this topic.
    pub messages_total: u64,
    /// Topic kind classification (Data, ServiceRequest, etc.).
    pub topic_kind: u8,
}

/// Read the latest message payload from a topic's shared-memory backing file.
///
/// Returns `None` when:
/// - `path` does not exist or cannot be opened,
/// - the file is too small to contain a valid header,
/// - the magic-number check fails (file not a HORUS topic),
/// - `write_idx == last_write_idx` (no new message since the previous call), or
/// - `write_idx == 0` (no message has ever been written on this topic).
///
/// # Layout assumed
///
/// ```text
/// offset 0        : TopicHeader (640 bytes)
/// offset 640      : data region
///   POD  : capacity × type_size bytes; slot[i] = raw POD bytes
///   Serde: capacity × slot_size bytes; slot[i] = [8B pad][8B len][bincode data…]
/// ```
pub fn read_latest_slot_bytes(
    path: &std::path::Path,
    last_write_idx: u64,
) -> Option<TopicSlotRead> {
    use memmap2::MmapOptions;
    use std::fs::File;

    // ── 1. Open and memory-map the file ──────────────────────────────────────
    let file = File::open(path).ok()?;
    let meta = file.metadata().ok()?;
    if meta.len() < TOPIC_HEADER_SIZE as u64 {
        return None;
    }
    // SAFETY: the file is opened read-only; the mapping is read-only.
    let mmap = unsafe { MmapOptions::new().map(&file).ok()? };
    let base: *const u8 = mmap.as_ptr();

    // ── 2. Validate magic ─────────────────────────────────────────────────────
    // SAFETY: mmap is at least TOPIC_HEADER_SIZE bytes and page-aligned.
    let magic = unsafe { std::ptr::read_unaligned(base as *const u64) };
    if magic != TOPIC_MAGIC {
        return None;
    }

    // ── 3. Read header fields (unaligned reads — we don't own the struct) ─────
    //   Offsets as per #[repr(C, align(64))] layout (confirmed by size assert):
    //     type_size  : offset 12  (u32)
    //     is_pod     : offset 20  (u8, POD_YES=2)
    //     seq/head   : offset 64  (u64)
    //     capacity   : offset 72  (u32)
    //     cap_mask   : offset 76  (u32)
    //     slot_size  : offset 80  (u32)
    // SAFETY: mmap is validated to be at least TOPIC_HEADER_SIZE (640) bytes above.
    // All offsets (12, 20, 64, 72, 76, 80) are within the header and read_unaligned
    // handles any alignment. base is a valid pointer from the mmap.
    let type_size = unsafe { std::ptr::read_unaligned(base.add(12) as *const u32) } as usize;
    // SAFETY: base is a valid mmap pointer; offset 20 is within the validated header region;
    // read_unaligned handles any alignment for the u8 is_pod field.
    let is_pod_raw = unsafe { std::ptr::read_unaligned(base.add(20)) };
    // SAFETY: base is a valid mmap pointer; offset 64 is within the validated header region;
    // read_unaligned handles any alignment for the u64 seq/head field.
    let write_idx = unsafe { std::ptr::read_unaligned(base.add(64) as *const u64) };
    // SAFETY: base is a valid mmap pointer; offset 72 is within the validated header region;
    // read_unaligned handles any alignment for the u32 capacity field.
    let capacity = unsafe { std::ptr::read_unaligned(base.add(72) as *const u32) } as usize;
    // SAFETY: base is a valid mmap pointer; offset 76 is within the validated header region;
    // read_unaligned handles any alignment for the u32 cap_mask field.
    let cap_mask = unsafe { std::ptr::read_unaligned(base.add(76) as *const u32) } as usize;
    // SAFETY: base is a valid mmap pointer; offset 80 is within the validated header region;
    // read_unaligned handles any alignment for the u32 slot_size field.
    let slot_size = unsafe { std::ptr::read_unaligned(base.add(80) as *const u32) } as usize;

    let is_pod = is_pod_raw == POD_YES;

    // ── 4. Guard: nothing written yet, or no new data since last poll ─────────
    if write_idx == 0 || write_idx == last_write_idx {
        return None;
    }

    // Index of the last-written slot (write_idx was incremented *after* the write)
    let last_written = ((write_idx.wrapping_sub(1)) as usize) & cap_mask;

    // ── 5. Extract payload bytes ──────────────────────────────────────────────
    let payload = if is_pod {
        if type_size == 0 || capacity == 0 {
            return None;
        }
        // Pod layout (same as serde): [HEADER (640)] [SEQ_ARRAY (cap * 8)] [DATA (cap * type_size)]
        let seq_array_size = capacity * std::mem::size_of::<u64>();
        let data_region_start = TOPIC_HEADER_SIZE + seq_array_size;
        let required = data_region_start + capacity * type_size;
        if mmap.len() < required {
            return None;
        }
        let slot_start = data_region_start + last_written * type_size;
        mmap[slot_start..slot_start + type_size].to_vec()
    } else {
        if slot_size < 16 || capacity == 0 {
            return None;
        }
        // Serde layout: [header (640B)] [seq_array (cap * 8B)] [data slots (cap * slot_size)]
        let seq_array_size = capacity * std::mem::size_of::<u64>();
        let data_region_start = TOPIC_HEADER_SIZE + seq_array_size;
        let required = data_region_start + capacity * slot_size;
        if mmap.len() < required {
            return None;
        }
        // Slot layout: [8 bytes padding][8 bytes data-len (u64 LE)][data…]
        let slot_start = data_region_start + last_written * slot_size;
        let len_offset = slot_start + 8;
        let data_offset = slot_start + 16;
        // SAFETY: len_offset is within mmap bounds (validated by required size check above);
        // read_unaligned handles any alignment.
        let data_len =
            unsafe { std::ptr::read_unaligned(mmap.as_ptr().add(len_offset) as *const u64) }
                as usize;
        if data_len == 0 || data_offset + data_len > mmap.len() {
            return None;
        }
        mmap[data_offset..data_offset + data_len].to_vec()
    };

    // Read type_name from header (bytes 216-247, 32 bytes in cache line 4).
    // Offset: cache_line_4(192) + publisher_count(4) + subscriber_count(4) +
    //         total_participants(4) + lease_timeout_ms(4) + last_topology_change_ms(8) = 216
    const TYPE_NAME_OFFSET: usize = 216;
    const TYPE_NAME_LEN: usize = 32;
    let type_name = if mmap.len() >= TYPE_NAME_OFFSET + TYPE_NAME_LEN {
        let name_bytes = &mmap[TYPE_NAME_OFFSET..TYPE_NAME_OFFSET + TYPE_NAME_LEN];
        let end = name_bytes
            .iter()
            .position(|&b| b == 0)
            .unwrap_or(TYPE_NAME_LEN);
        std::str::from_utf8(&name_bytes[..end])
            .unwrap_or("")
            .to_string()
    } else {
        String::new()
    };

    // Read topic_kind (byte 48 in cache line 1) and messages_total (offset 56).
    // SAFETY: mmap is validated to be at least TOPIC_HEADER_SIZE (640) bytes.
    let topic_kind = unsafe { std::ptr::read_unaligned(base.add(48)) };
    let messages_total = unsafe { std::ptr::read_unaligned(base.add(56) as *const u64) };

    Some(TopicSlotRead {
        payload,
        write_idx,
        is_pod,
        type_name,
        messages_total,
        topic_kind,
    })
}

/// Read the write-sequence counter from a raw topic SHM file.
///
/// This is a lightweight read used by the monitor to compute accurate message
/// rates without needing a typed `Topic<T>`.  Returns `None` when the file
/// cannot be opened, is too small, or has an invalid magic number.
pub fn read_topic_sequence(path: &std::path::Path) -> Option<u64> {
    use memmap2::MmapOptions;
    use std::fs::File;

    let file = File::open(path).ok()?;
    let meta = file.metadata().ok()?;
    if meta.len() < TOPIC_HEADER_SIZE as u64 {
        return None;
    }
    // SAFETY: the file is opened read-only; the mapping is read-only.
    let mmap = unsafe { MmapOptions::new().map(&file).ok()? };
    let base: *const u8 = mmap.as_ptr();

    // SAFETY: mmap is at least TOPIC_HEADER_SIZE (640) bytes.
    let magic = unsafe { std::ptr::read_unaligned(base as *const u64) };
    if magic != TOPIC_MAGIC {
        return None;
    }
    // SAFETY: offset 64 is within the validated header (sequence_or_head field).
    let seq = unsafe { std::ptr::read_unaligned(base.add(64) as *const u64) };
    Some(seq)
}

/// Read the messages_total counter from a raw topic SHM file.
///
/// Unlike `read_topic_sequence` (which reads `sequence_or_head` at offset 64,
/// flushed lazily for some backends), this reads the `messages_total` counter
/// (offset 56) which is atomically incremented on **every** send() regardless
/// of backend type. Use this for accurate rate measurement.
pub fn read_topic_messages_total(path: &std::path::Path) -> Option<u64> {
    use memmap2::MmapOptions;
    use std::fs::File;

    let file = File::open(path).ok()?;
    let meta = file.metadata().ok()?;
    if meta.len() < TOPIC_HEADER_SIZE as u64 {
        return None;
    }
    let mmap = unsafe { MmapOptions::new().map(&file).ok()? };
    let base: *const u8 = mmap.as_ptr();

    let magic = unsafe { std::ptr::read_unaligned(base as *const u64) };
    if magic != TOPIC_MAGIC {
        return None;
    }
    // SAFETY: offset 56 is within the validated header (messages_total field).
    let total = unsafe { std::ptr::read_unaligned(base.add(56) as *const u64) };
    Some(total)
}

/// Lightweight header-only metadata (no payload read).
pub struct TopicHeaderInfo {
    /// Message type name from the header (e.g. "CmdVel").
    pub type_name: String,
    /// Total messages ever sent.
    pub messages_total: u64,
    /// Topic kind classification.
    pub topic_kind: u8,
    /// Whether the message type is POD.
    pub is_pod: bool,
    /// Message type size in bytes.
    pub type_size: u32,
    /// Number of active publishers.
    pub publisher_count: u32,
    /// Number of active subscribers.
    pub subscriber_count: u32,
}

/// Read header metadata from a topic SHM file without reading any message payload.
///
/// This is cheaper than `read_latest_slot_bytes()` — it only reads the 640-byte
/// header, not the data region. Used by discovery to extract type_name,
/// messages_total, and topic_kind.
pub fn read_topic_header_info(path: &std::path::Path) -> Option<TopicHeaderInfo> {
    use memmap2::MmapOptions;
    use std::fs::File;

    let file = File::open(path).ok()?;
    let meta = file.metadata().ok()?;
    if meta.len() < TOPIC_HEADER_SIZE as u64 {
        return None;
    }
    // SAFETY: read-only mmap.
    let mmap = unsafe { MmapOptions::new().map(&file).ok()? };
    let base = mmap.as_ptr();

    // SAFETY: mmap >= TOPIC_HEADER_SIZE (640) bytes.
    let magic = unsafe { std::ptr::read_unaligned(base as *const u64) };
    if magic != TOPIC_MAGIC {
        return None;
    }

    // SAFETY: all offsets are within the validated 640-byte header.
    unsafe {
        let type_size = std::ptr::read_unaligned(base.add(12) as *const u32);
        let is_pod_raw = std::ptr::read_unaligned(base.add(20));
        let topic_kind = std::ptr::read_unaligned(base.add(48));
        let messages_total = std::ptr::read_unaligned(base.add(56) as *const u64);
        let publisher_count = (*(base.add(192) as *const AtomicU32)).load(Ordering::Relaxed);
        let subscriber_count = (*(base.add(196) as *const AtomicU32)).load(Ordering::Relaxed);

        // type_name at offset 216, 32 bytes
        let name_bytes = std::slice::from_raw_parts(base.add(216), 32);
        let end = name_bytes.iter().position(|&b| b == 0).unwrap_or(32);
        let type_name = std::str::from_utf8(&name_bytes[..end])
            .unwrap_or("")
            .to_string();

        Some(TopicHeaderInfo {
            type_name,
            messages_total,
            topic_kind,
            is_pod: is_pod_raw == POD_YES,
            type_size,
            publisher_count,
            subscriber_count,
        })
    }
}

// ============================================================================
// Helper Functions
// ============================================================================

/// Hash a ThreadId to u64 for storage in shared memory
#[inline]
pub(crate) fn hash_thread_id(id: std::thread::ThreadId) -> u64 {
    use std::collections::hash_map::DefaultHasher;
    use std::hash::{Hash, Hasher};

    let mut hasher = DefaultHasher::new();
    id.hash(&mut hasher);
    hasher.finish()
}

/// Get current time in milliseconds since UNIX epoch
#[inline]
pub(crate) fn current_time_ms() -> u64 {
    use std::time::{SystemTime, UNIX_EPOCH};
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap_or_default()
        .as_millis() as u64
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::atomic::Ordering;

    // ── Helper ──────────────────────────────────────────────────────────

    fn make_header(type_size: u32, type_align: u32, is_pod: bool, capacity: u32) -> TopicHeader {
        let mut h = TopicHeader::zeroed();
        h.init(
            type_size,
            type_align,
            is_pod,
            capacity,
            type_size.max(16),
            "TestType",
            TopicKind::Data as u8,
        );
        h
    }

    // ── Constants ───────────────────────────────────────────────────────

    #[test]
    fn magic_is_ascii_adaptive() {
        let bytes = TOPIC_MAGIC.to_be_bytes();
        assert_eq!(&bytes, b"ADAPTIVE");
    }

    #[test]
    fn header_size_is_640_bytes() {
        assert_eq!(std::mem::size_of::<TopicHeader>(), 640);
        assert_eq!(TOPIC_HEADER_SIZE, 640);
    }

    #[test]
    fn participant_entry_size_is_24_bytes() {
        assert_eq!(std::mem::size_of::<ParticipantEntry>(), 24);
    }

    #[test]
    fn verbose_offset_matches_struct_layout() {
        let h = TopicHeader::zeroed();
        let base = &h as *const TopicHeader as *const u8;
        let debug_ptr = &h.verbose as *const AtomicU8 as *const u8;
        // SAFETY: both pointers derive from the same TopicHeader allocation,
        // so offset_from is well-defined and within the object bounds.
        let offset = unsafe { debug_ptr.offset_from(base) } as usize;
        assert_eq!(offset, TOPIC_VERBOSE_OFFSET);
    }

    // ── TopicHeader::init ───────────────────────────────────────────────

    #[test]
    fn init_sets_magic_and_version() {
        let h = make_header(8, 8, true, 16);
        assert_eq!(h.magic, TOPIC_MAGIC);
        assert_eq!(h.version, TOPIC_VERSION);
    }

    #[test]
    fn init_sets_type_metadata() {
        let h = make_header(4, 4, false, 32);
        assert_eq!(h.type_size, 4);
        assert_eq!(h.type_align, 4);
        assert_eq!(h.is_pod.load(Ordering::Relaxed), POD_NO);
    }

    #[test]
    fn init_pod_yes_flag() {
        let h = make_header(8, 8, true, 16);
        assert_eq!(h.is_pod.load(Ordering::Relaxed), POD_YES);
        assert!(h.is_pod_type());
    }

    #[test]
    fn init_pod_no_flag() {
        let h = make_header(8, 8, false, 16);
        assert_eq!(h.is_pod.load(Ordering::Relaxed), POD_NO);
        assert!(!h.is_pod_type());
    }

    #[test]
    fn init_rounds_capacity_to_power_of_two() {
        let h = make_header(8, 8, true, 10);
        assert_eq!(h.capacity, 16); // 10 → next_power_of_two → 16
        assert_eq!(h.capacity_mask, 15);
    }

    #[test]
    fn init_capacity_already_power_of_two() {
        let h = make_header(8, 8, true, 64);
        assert_eq!(h.capacity, 64);
        assert_eq!(h.capacity_mask, 63);
    }

    #[test]
    fn init_capacity_one() {
        let h = make_header(8, 8, true, 1);
        assert_eq!(h.capacity, 1);
        assert_eq!(h.capacity_mask, 0);
    }

    #[test]
    fn init_sets_creator_pid_and_thread() {
        let h = make_header(8, 8, true, 16);
        assert_eq!(h.creator_pid, std::process::id());
        let expected_hash = hash_thread_id(std::thread::current().id());
        assert_eq!(h.creator_thread_id_hash, expected_hash);
    }

    #[test]
    fn init_backend_starts_unknown() {
        let h = make_header(8, 8, true, 16);
        assert_eq!(h.mode(), BackendMode::Unknown);
    }

    #[test]
    fn init_counters_are_zero() {
        let h = make_header(8, 8, true, 16);
        assert_eq!(h.pub_count(), 0);
        assert_eq!(h.sub_count(), 0);
        assert_eq!(h.total_participants.load(Ordering::Relaxed), 0);
    }

    #[test]
    fn init_migration_starts_unlocked() {
        let h = make_header(8, 8, true, 16);
        assert_eq!(h.migration_lock.load(Ordering::Relaxed), MIGRATION_UNLOCKED);
        assert_eq!(h.migration_epoch.load(Ordering::Relaxed), 0);
    }

    #[test]
    fn init_sequence_and_tail_are_zero() {
        let h = make_header(8, 8, true, 16);
        assert_eq!(h.sequence_or_head.load(Ordering::Relaxed), 0);
        assert_eq!(h.tail.load(Ordering::Relaxed), 0);
    }

    #[test]
    fn init_verbose_disabled() {
        let h = make_header(8, 8, true, 16);
        assert!(!h.is_verbose());
    }

    #[test]
    fn init_participants_all_cleared() {
        let h = make_header(8, 8, true, 16);
        for p in &h.participants {
            assert_eq!(p.pid.load(Ordering::Relaxed), 0);
            assert_eq!(p.thread_id_hash.load(Ordering::Relaxed), 0);
            assert_eq!(p.role.load(Ordering::Relaxed), 0);
            assert_eq!(p.active.load(Ordering::Relaxed), 0);
            assert_eq!(p.lease_expires_ms.load(Ordering::Relaxed), 0);
        }
    }

    // ── ParticipantEntry ────────────────────────────────────────────────

    #[test]
    fn participant_lease_expired_when_zero() {
        let p = ParticipantEntry {
            pid: AtomicU32::new(0),
            thread_id_hash: AtomicU32::new(0),
            role: AtomicU8::new(0),
            active: AtomicU8::new(0),
            source_host: AtomicU8::new(0),
            _pad: [0; 5],
            lease_expires_ms: AtomicU64::new(0),
        };
        assert!(p.is_lease_expired(1000));
    }

    #[test]
    fn participant_lease_not_expired_when_future() {
        let p = ParticipantEntry {
            pid: AtomicU32::new(0),
            thread_id_hash: AtomicU32::new(0),
            role: AtomicU8::new(0),
            active: AtomicU8::new(0),
            source_host: AtomicU8::new(0),
            _pad: [0; 5],
            lease_expires_ms: AtomicU64::new(5000),
        };
        assert!(!p.is_lease_expired(4999));
    }

    #[test]
    fn participant_lease_expired_when_past() {
        let p = ParticipantEntry {
            pid: AtomicU32::new(0),
            thread_id_hash: AtomicU32::new(0),
            role: AtomicU8::new(0),
            active: AtomicU8::new(0),
            source_host: AtomicU8::new(0),
            _pad: [0; 5],
            lease_expires_ms: AtomicU64::new(5000),
        };
        assert!(p.is_lease_expired(5001));
    }

    #[test]
    fn participant_lease_expired_at_exact_boundary() {
        let p = ParticipantEntry {
            pid: AtomicU32::new(0),
            thread_id_hash: AtomicU32::new(0),
            role: AtomicU8::new(0),
            active: AtomicU8::new(0),
            source_host: AtomicU8::new(0),
            _pad: [0; 5],
            lease_expires_ms: AtomicU64::new(5000),
        };
        // now_ms == expires → not expired (> check, not >=)
        assert!(!p.is_lease_expired(5000));
    }

    #[test]
    fn participant_refresh_lease_updates_expiry() {
        let p = ParticipantEntry {
            pid: AtomicU32::new(0),
            thread_id_hash: AtomicU32::new(0),
            role: AtomicU8::new(0),
            active: AtomicU8::new(0),
            source_host: AtomicU8::new(0),
            _pad: [0; 5],
            lease_expires_ms: AtomicU64::new(0),
        };
        p.refresh_lease(1000, 5000);
        assert_eq!(p.lease_expires_ms.load(Ordering::Relaxed), 6000);
        assert!(!p.is_lease_expired(5999));
        assert!(p.is_lease_expired(6001));
    }

    #[test]
    fn participant_clear_resets_all_fields() {
        let p = ParticipantEntry {
            pid: AtomicU32::new(42),
            thread_id_hash: AtomicU32::new(0xABCD),
            role: AtomicU8::new(3),
            active: AtomicU8::new(1),
            source_host: AtomicU8::new(0),
            _pad: [0; 5],
            lease_expires_ms: AtomicU64::new(99999),
        };
        p.clear();
        assert_eq!(p.pid.load(Ordering::Relaxed), 0);
        assert_eq!(p.thread_id_hash.load(Ordering::Relaxed), 0);
        assert_eq!(p.role.load(Ordering::Relaxed), 0);
        assert_eq!(p.active.load(Ordering::Relaxed), 0);
        assert_eq!(p.lease_expires_ms.load(Ordering::Relaxed), 0);
    }

    // ── Registration ────────────────────────────────────────────────────

    #[test]
    fn register_producer_increments_pub_count() {
        let h = make_header(8, 8, true, 16);
        let slot = h.register_producer().unwrap();
        assert!(slot < MAX_PARTICIPANTS);
        assert_eq!(h.pub_count(), 1);
        assert_eq!(h.sub_count(), 0);
        assert_eq!(h.total_participants.load(Ordering::Relaxed), 1);
    }

    #[test]
    fn register_consumer_increments_sub_count() {
        let h = make_header(8, 8, true, 16);
        let slot = h.register_consumer().unwrap();
        assert!(slot < MAX_PARTICIPANTS);
        assert_eq!(h.sub_count(), 1);
        assert_eq!(h.pub_count(), 0);
    }

    #[test]
    fn register_same_thread_reuses_slot() {
        let h = make_header(8, 8, true, 16);
        let slot1 = h.register_producer().unwrap();
        let slot2 = h.register_consumer().unwrap();
        assert_eq!(
            slot1, slot2,
            "Same thread should reuse same participant slot"
        );
        assert_eq!(h.pub_count(), 1);
        assert_eq!(h.sub_count(), 1);
        // Role should be both producer and consumer (1 | 2 = 3)
        let role = h.participants[slot1].role.load(Ordering::Relaxed);
        assert_eq!(role, 3);
    }

    #[test]
    fn register_producer_twice_same_thread_no_double_count() {
        let h = make_header(8, 8, true, 16);
        let s1 = h.register_producer().unwrap();
        let s2 = h.register_producer().unwrap();
        assert_eq!(s1, s2);
        assert_eq!(
            h.pub_count(),
            1,
            "Should not double-count same-thread producer"
        );
    }

    #[test]
    fn register_from_different_threads_uses_different_slots() {
        let h = make_header(8, 8, true, 16);
        let slot1 = h.register_producer().unwrap();

        let header_ptr = &h as *const TopicHeader as usize;
        let slot2 = std::thread::spawn(move || {
            // SAFETY: header_ptr was derived from a valid stack-allocated TopicHeader that
            // outlives this thread (main thread joins before h is dropped).
            let h = unsafe { &*(header_ptr as *const TopicHeader) };
            h.register_producer().unwrap()
        })
        .join()
        .unwrap();

        assert_ne!(slot1, slot2);
        assert_eq!(h.pub_count(), 2);
    }

    #[test]
    fn register_fills_all_slots() {
        let h = make_header(8, 8, true, 16);
        let mut slots = Vec::new();

        // Fill MAX_PARTICIPANTS slots from different threads
        let header_ptr = &h as *const TopicHeader as usize;
        let handles: Vec<_> = (0..MAX_PARTICIPANTS)
            .map(|_| {
                std::thread::spawn(move || {
                    // SAFETY: header_ptr was derived from a valid stack-allocated TopicHeader that
                    // outlives all spawned threads (main thread joins before h is dropped).
                    let h = unsafe { &*(header_ptr as *const TopicHeader) };
                    h.register_producer().unwrap()
                })
            })
            .collect();

        for handle in handles {
            slots.push(handle.join().unwrap());
        }

        // All slots should be unique and in range
        slots.sort();
        slots.dedup();
        assert_eq!(slots.len(), MAX_PARTICIPANTS);
        assert_eq!(h.pub_count(), MAX_PARTICIPANTS as u32);
    }

    #[test]
    fn register_fails_when_all_slots_full() {
        let h = make_header(8, 8, true, 16);

        // Fill all slots from different threads
        let header_ptr = &h as *const TopicHeader as usize;
        let handles: Vec<_> = (0..MAX_PARTICIPANTS)
            .map(|_| {
                std::thread::spawn(move || {
                    // SAFETY: header_ptr was derived from a valid stack-allocated TopicHeader that
                    // outlives all spawned threads (main thread joins before h is dropped).
                    let h = unsafe { &*(header_ptr as *const TopicHeader) };
                    h.register_producer().unwrap()
                })
            })
            .collect();
        for handle in handles {
            handle.join().unwrap();
        }

        // The next registration from yet another thread should fail
        let result = std::thread::spawn(move || {
            // SAFETY: header_ptr was derived from a valid stack-allocated TopicHeader that
            // outlives this thread (main thread joins before h is dropped).
            let h = unsafe { &*(header_ptr as *const TopicHeader) };
            h.register_producer()
        })
        .join()
        .unwrap();
        result.unwrap_err();
    }

    // ── Topology detection ──────────────────────────────────────────────

    #[test]
    fn is_same_process_true_when_no_participants() {
        let h = make_header(8, 8, true, 16);
        assert!(h.is_same_process());
    }

    #[test]
    fn is_same_process_true_with_local_participants() {
        let h = make_header(8, 8, true, 16);
        h.register_producer().unwrap();
        assert!(h.is_same_process());
    }

    #[test]
    fn is_same_process_false_with_foreign_pid() {
        let h = make_header(8, 8, true, 16);
        // Manually inject a participant with a different PID
        h.participants[0].pid.store(99999, Ordering::Relaxed);
        h.participants[0].active.store(1, Ordering::Relaxed);
        assert!(!h.is_same_process());
    }

    #[test]
    fn is_same_thread_true_on_same_thread() {
        let h = make_header(8, 8, true, 16);
        assert!(h.is_same_thread());
    }

    #[test]
    fn all_participants_same_thread_empty() {
        let h = make_header(8, 8, true, 16);
        assert!(h.all_participants_same_thread());
    }

    #[test]
    fn all_participants_same_thread_single_producer() {
        let h = make_header(8, 8, true, 16);
        h.register_producer().unwrap();
        assert!(h.all_participants_same_thread());
    }

    #[test]
    fn all_participants_same_thread_false_with_different_threads() {
        let h = make_header(8, 8, true, 16);
        h.register_producer().unwrap();

        let header_ptr = &h as *const TopicHeader as usize;
        std::thread::spawn(move || {
            // SAFETY: header_ptr was derived from a valid stack-allocated TopicHeader that
            // outlives this thread (main thread joins before h is dropped).
            let h = unsafe { &*(header_ptr as *const TopicHeader) };
            h.register_consumer().unwrap();
        })
        .join()
        .unwrap();

        assert!(!h.all_participants_same_thread());
    }

    // ── Migration lock ──────────────────────────────────────────────────

    #[test]
    fn migration_lock_acquire_and_release() {
        let h = make_header(8, 8, true, 16);
        assert!(h.try_lock_migration());
        assert_eq!(h.migration_lock.load(Ordering::Relaxed), MIGRATION_LOCKED);
        // Second acquire should fail
        assert!(!h.try_lock_migration());
        h.unlock_migration();
        assert_eq!(h.migration_lock.load(Ordering::Relaxed), MIGRATION_UNLOCKED);
        // Now it should succeed again
        assert!(h.try_lock_migration());
    }

    // ── Debug flag ──────────────────────────────────────────────────────

    #[test]
    fn verbose_toggle() {
        let h = make_header(8, 8, true, 16);
        assert!(!h.is_verbose());
        h.verbose.store(1, Ordering::Relaxed);
        assert!(h.is_verbose());
        h.verbose.store(0, Ordering::Relaxed);
        assert!(!h.is_verbose());
    }

    // ── Backend detection ───────────────────────────────────────────────

    #[test]
    fn detect_optimal_backend_no_participants_is_unknown() {
        let h = make_header(8, 8, true, 16);
        assert_eq!(h.detect_optimal_backend(), BackendMode::Unknown);
    }

    #[test]
    fn detect_optimal_backend_same_thread_pod_is_direct_channel() {
        let h = make_header(8, 8, true, 16);
        h.register_producer().unwrap();
        h.register_consumer().unwrap();
        assert_eq!(h.detect_optimal_backend(), BackendMode::DirectChannel);
    }

    #[test]
    fn detect_optimal_backend_same_thread_non_pod_is_spsc_intra() {
        let h = make_header(8, 8, false, 16);
        h.register_producer().unwrap();
        h.register_consumer().unwrap();
        assert_eq!(h.detect_optimal_backend(), BackendMode::SpscIntra);
    }

    #[test]
    fn detect_optimal_backend_single_producer_only() {
        let h = make_header(8, 8, true, 16);
        h.register_producer().unwrap();
        // 1P, 0C → SpscIntra (anticipating single consumer)
        assert_eq!(h.detect_optimal_backend(), BackendMode::SpscIntra);
    }

    #[test]
    fn detect_optimal_backend_single_consumer_only() {
        let h = make_header(8, 8, true, 16);
        h.register_consumer().unwrap();
        assert_eq!(h.detect_optimal_backend(), BackendMode::SpscIntra);
    }

    #[test]
    fn detect_optimal_backend_cross_thread_spsc_intra() {
        let h = make_header(8, 8, true, 16);
        h.register_producer().unwrap();
        let header_ptr = &h as *const TopicHeader as usize;
        std::thread::spawn(move || {
            // SAFETY: header_ptr was derived from a valid stack-allocated TopicHeader that
            // outlives this thread (main thread joins before h is dropped).
            let h = unsafe { &*(header_ptr as *const TopicHeader) };
            h.register_consumer().unwrap();
        })
        .join()
        .unwrap();
        // Same process, different threads, 1P:1C → SpscIntra
        assert_eq!(h.detect_optimal_backend(), BackendMode::SpscIntra);
    }

    #[test]
    fn detect_optimal_backend_spmc_intra() {
        let h = make_header(8, 8, true, 16);
        h.register_producer().unwrap();

        let header_ptr = &h as *const TopicHeader as usize;
        // Register 2 consumers from different threads
        for _ in 0..2 {
            std::thread::spawn(move || {
                // SAFETY: header_ptr was derived from a valid stack-allocated TopicHeader that
                // outlives this thread (main thread joins before h is dropped).
                let h = unsafe { &*(header_ptr as *const TopicHeader) };
                h.register_consumer().unwrap();
            })
            .join()
            .unwrap();
        }
        assert_eq!(h.pub_count(), 1);
        assert!(h.sub_count() >= 2);
        assert_eq!(h.detect_optimal_backend(), BackendMode::SpmcIntra);
    }

    #[test]
    fn detect_optimal_backend_mpsc_intra() {
        let h = make_header(8, 8, true, 16);
        h.register_consumer().unwrap();

        let header_ptr = &h as *const TopicHeader as usize;
        // Register 2 producers from different threads
        for _ in 0..2 {
            std::thread::spawn(move || {
                // SAFETY: header_ptr was derived from a valid stack-allocated TopicHeader that
                // outlives this thread (main thread joins before h is dropped).
                let h = unsafe { &*(header_ptr as *const TopicHeader) };
                h.register_producer().unwrap();
            })
            .join()
            .unwrap();
        }
        assert!(h.pub_count() >= 2);
        assert_eq!(h.sub_count(), 1);
        assert_eq!(h.detect_optimal_backend(), BackendMode::MpscIntra);
    }

    #[test]
    fn detect_optimal_backend_cross_process_spsc_shm() {
        let h = make_header(8, 8, true, 16);
        // Simulate cross-process: inject a participant with different PID
        h.participants[0].pid.store(99999, Ordering::Relaxed);
        h.participants[0]
            .thread_id_hash
            .store(123, Ordering::Relaxed);
        h.participants[0].role.store(1, Ordering::Relaxed); // producer
        h.participants[0].active.store(1, Ordering::Relaxed);
        h.participants[0]
            .lease_expires_ms
            .store(u64::MAX, Ordering::Relaxed);
        h.publisher_count.store(1, Ordering::Relaxed);

        // Register local consumer
        h.participants[1]
            .pid
            .store(std::process::id(), Ordering::Relaxed);
        h.participants[1].thread_id_hash.store(
            hash_thread_id(std::thread::current().id()) as u32,
            Ordering::Relaxed,
        );
        h.participants[1].role.store(2, Ordering::Relaxed); // consumer
        h.participants[1].active.store(1, Ordering::Relaxed);
        h.participants[1]
            .lease_expires_ms
            .store(u64::MAX, Ordering::Relaxed);
        h.subscriber_count.store(1, Ordering::Relaxed);

        assert!(!h.is_same_process());
        assert_eq!(h.detect_optimal_backend(), BackendMode::SpscShm);
    }

    #[test]
    fn detect_optimal_backend_cross_process_pod_shm() {
        let h = make_header(8, 8, true, 16);
        // Simulate multiple cross-process producers and consumers
        h.participants[0].pid.store(99999, Ordering::Relaxed);
        h.participants[0].role.store(1, Ordering::Relaxed);
        h.participants[0].active.store(1, Ordering::Relaxed);
        h.participants[0]
            .lease_expires_ms
            .store(u64::MAX, Ordering::Relaxed);

        h.participants[1].pid.store(99998, Ordering::Relaxed);
        h.participants[1].role.store(3, Ordering::Relaxed); // both
        h.participants[1].active.store(1, Ordering::Relaxed);
        h.participants[1]
            .lease_expires_ms
            .store(u64::MAX, Ordering::Relaxed);

        h.publisher_count.store(2, Ordering::Relaxed);
        h.subscriber_count.store(2, Ordering::Relaxed);

        assert!(!h.is_same_process());
        // Multi-pub, multi-sub, cross-process, POD → PodShm
        assert_eq!(h.detect_optimal_backend(), BackendMode::PodShm);
    }

    // ── Backend mode read/write ─────────────────────────────────────────

    #[test]
    fn mode_roundtrip_all_variants() {
        let h = make_header(8, 8, true, 16);
        let modes = [
            BackendMode::Unknown,
            BackendMode::DirectChannel,
            BackendMode::SpscIntra,
            BackendMode::SpmcIntra,
            BackendMode::MpscIntra,
            BackendMode::FanoutIntra,
            BackendMode::PodShm,
            BackendMode::MpscShm,
            BackendMode::SpmcShm,
            BackendMode::SpscShm,
            BackendMode::FanoutShm,
        ];
        for mode in modes {
            h.backend_mode.store(mode as u8, Ordering::Relaxed);
            assert_eq!(h.mode(), mode);
        }
    }

    // ── Zeroed header ───────────────────────────────────────────────────

    #[test]
    fn zeroed_has_no_magic() {
        let h = TopicHeader::zeroed();
        assert_eq!(h.magic, 0);
        assert_eq!(h.version, 0);
    }

    // ── hash_thread_id ──────────────────────────────────────────────────

    #[test]
    fn hash_thread_id_deterministic() {
        let id = std::thread::current().id();
        let h1 = hash_thread_id(id);
        let h2 = hash_thread_id(id);
        assert_eq!(h1, h2);
    }

    #[test]
    fn hash_thread_id_different_threads_different_hashes() {
        let main_hash = hash_thread_id(std::thread::current().id());
        let other_hash = std::thread::spawn(|| hash_thread_id(std::thread::current().id()))
            .join()
            .unwrap();
        // Different threads should (almost certainly) produce different hashes
        assert_ne!(main_hash, other_hash);
    }

    // ── current_time_ms ─────────────────────────────────────────────────

    #[test]
    fn current_time_ms_is_reasonable() {
        let now = current_time_ms();
        // Should be after 2024-01-01 (ms since epoch ≈ 1704067200000)
        assert!(now > 1_704_067_200_000);
        // Should be monotonically non-decreasing
        let later = current_time_ms();
        assert!(later >= now);
    }

    // ── set_topic_verbose (unsafe) ──────────────────────────────────────

    #[test]
    fn set_topic_verbose_via_raw_pointer() {
        let mut h = make_header(8, 8, true, 16);
        assert!(!h.is_verbose());

        let ptr = &mut h as *mut TopicHeader as *mut u8;
        // SAFETY: ptr points to a valid, initialized TopicHeader on the stack (640 bytes).
        unsafe { set_topic_verbose(ptr, true) };
        assert!(h.is_verbose());

        // SAFETY: ptr points to a valid, initialized TopicHeader on the stack (640 bytes).
        unsafe { set_topic_verbose(ptr, false) };
        assert!(!h.is_verbose());
    }

    // ── Lease expiration eviction ───────────────────────────────────────

    #[test]
    fn expired_slot_reclaimed_by_new_registration() {
        let h = make_header(8, 8, true, 16);

        // Manually create an "expired" participant in slot 0
        h.participants[0].pid.store(99999, Ordering::Relaxed);
        h.participants[0]
            .thread_id_hash
            .store(111, Ordering::Relaxed);
        h.participants[0].role.store(1, Ordering::Relaxed); // producer
        h.participants[0].active.store(1, Ordering::Relaxed);
        h.participants[0]
            .lease_expires_ms
            .store(1, Ordering::Relaxed); // expired long ago
        h.publisher_count.store(1, Ordering::Relaxed);

        // Register from current thread — should evict the expired slot
        let slot = h.register_producer().unwrap();
        assert_eq!(slot, 0, "Should reclaim expired slot 0");
        assert_eq!(
            h.pub_count(),
            1,
            "Old pub decremented, new pub incremented → still 1"
        );
        assert_eq!(
            h.participants[0].pid.load(Ordering::Relaxed),
            std::process::id()
        );
    }

    // ── Boundary values ─────────────────────────────────────────────────

    #[test]
    fn init_with_zero_type_size() {
        // Zero-sized types (like () or PhantomData) should still work
        let h = make_header(0, 1, true, 16);
        assert_eq!(h.type_size, 0);
        assert_eq!(h.type_align, 1);
    }

    #[test]
    fn init_with_large_type_size() {
        let h = make_header(u32::MAX, 64, false, 16);
        assert_eq!(h.type_size, u32::MAX);
    }

    #[test]
    fn lease_refresh_large_values_no_panic() {
        let p = ParticipantEntry {
            pid: AtomicU32::new(0),
            thread_id_hash: AtomicU32::new(0),
            role: AtomicU8::new(0),
            active: AtomicU8::new(0),
            source_host: AtomicU8::new(0),
            _pad: [0; 5],
            lease_expires_ms: AtomicU64::new(0),
        };
        // Realistic large timestamp (year ~2100 in ms) + large timeout
        let far_future_ms: u64 = 4_102_444_800_000; // 2100-01-01
        let timeout_ms: u64 = 86_400_000; // 24 hours
        p.refresh_lease(far_future_ms, timeout_ms);
        assert_eq!(
            p.lease_expires_ms.load(Ordering::Relaxed),
            far_future_ms + timeout_ms
        );
    }

    // ── type_name field ────────────────────────────────────────────────

    #[test]
    fn init_sets_type_name() {
        let mut h = TopicHeader::zeroed();
        h.init(8, 8, true, 16, 16, "CmdVel", TopicKind::Data as u8);
        assert_eq!(h.type_name_str(), "CmdVel");
    }

    #[test]
    fn init_type_name_truncation_at_31_chars() {
        let long_name = "A".repeat(40);
        let mut h = TopicHeader::zeroed();
        h.init(8, 8, true, 16, 16, &long_name, TopicKind::Data as u8);
        assert_eq!(
            h.type_name_str().len(),
            31,
            "type_name should be truncated to 31 chars, got {}",
            h.type_name_str().len()
        );
        assert_eq!(h.type_name_str(), "A".repeat(31));
    }

    #[test]
    fn init_type_name_empty() {
        let mut h = TopicHeader::zeroed();
        h.init(8, 8, true, 16, 16, "", TopicKind::Data as u8);
        assert_eq!(h.type_name_str(), "");
    }

    #[test]
    fn init_type_name_exact_31_chars() {
        let name = "B".repeat(31);
        let mut h = TopicHeader::zeroed();
        h.init(8, 8, true, 16, 16, &name, TopicKind::Data as u8);
        assert_eq!(h.type_name_str(), name);
    }

    #[test]
    fn type_name_zeroed_returns_empty() {
        let h = TopicHeader::zeroed();
        assert_eq!(h.type_name_str(), "");
    }

    #[test]
    fn init_type_name_with_colons() {
        // Simulates a full Rust path like "horus_library::messages::Imu"
        let mut h = TopicHeader::zeroed();
        h.init(
            8,
            8,
            true,
            16,
            16,
            "horus_library::messages::Imu",
            TopicKind::Data as u8,
        );
        assert_eq!(h.type_name_str(), "horus_library::messages::Imu");
    }

    // ── type_name offset ───────────────────────────────────────────────

    #[test]
    fn type_name_offset_is_216() {
        let h = TopicHeader::zeroed();
        let base = &h as *const TopicHeader as *const u8;
        let field_ptr = h.type_name.as_ptr();
        // SAFETY: both pointers derive from the same TopicHeader allocation.
        let offset = unsafe { field_ptr.offset_from(base) } as usize;
        assert_eq!(offset, 216, "type_name should be at byte offset 216");
    }

    // ── messages_total field ───────────────────────────────────────────

    #[test]
    fn init_messages_total_is_zero() {
        let h = make_header(8, 8, true, 16);
        assert_eq!(h.messages_total(), 0);
    }

    #[test]
    fn messages_total_increment_readable() {
        let h = make_header(8, 8, true, 16);
        h.messages_total.fetch_add(42, Ordering::Relaxed);
        assert_eq!(h.messages_total(), 42);
    }

    #[test]
    fn messages_total_multiple_increments() {
        let h = make_header(8, 8, true, 16);
        for _ in 0..100 {
            h.messages_total.fetch_add(1, Ordering::Relaxed);
        }
        assert_eq!(h.messages_total(), 100);
    }

    // ── messages_total offset ──────────────────────────────────────────

    #[test]
    fn messages_total_offset_is_56() {
        let h = TopicHeader::zeroed();
        let base = &h as *const TopicHeader as *const u8;
        let field_ptr = &h.messages_total as *const AtomicU64 as *const u8;
        // SAFETY: both pointers derive from the same TopicHeader allocation.
        let offset = unsafe { field_ptr.offset_from(base) } as usize;
        assert_eq!(offset, 56, "messages_total should be at byte offset 56");
    }

    // ── topic_kind field ───────────────────────────────────────────────

    #[test]
    fn init_topic_kind_is_data() {
        let h = make_header(8, 8, true, 16);
        assert_eq!(h.topic_kind(), TopicKind::Data);
    }

    #[test]
    fn topic_kind_from_u8_all_variants() {
        assert_eq!(TopicKind::from_u8(0), TopicKind::Data);
        assert_eq!(TopicKind::from_u8(1), TopicKind::ServiceRequest);
        assert_eq!(TopicKind::from_u8(2), TopicKind::ServiceResponse);
        assert_eq!(TopicKind::from_u8(3), TopicKind::ActionGoal);
        assert_eq!(TopicKind::from_u8(4), TopicKind::ActionFeedback);
        assert_eq!(TopicKind::from_u8(5), TopicKind::ActionResult);
        assert_eq!(TopicKind::from_u8(6), TopicKind::ActionStatus);
        assert_eq!(TopicKind::from_u8(7), TopicKind::ActionCancel);
        assert_eq!(TopicKind::from_u8(8), TopicKind::System);
    }

    #[test]
    fn topic_kind_from_u8_unknown_defaults_to_data() {
        assert_eq!(TopicKind::from_u8(9), TopicKind::Data);
        assert_eq!(TopicKind::from_u8(99), TopicKind::Data);
        assert_eq!(TopicKind::from_u8(255), TopicKind::Data);
    }

    // ── topic_kind offset ──────────────────────────────────────────────

    #[test]
    fn topic_kind_offset_is_48() {
        let h = TopicHeader::zeroed();
        let base = &h as *const TopicHeader as *const u8;
        let field_ptr = &h.topic_kind as *const u8;
        // SAFETY: both pointers derive from the same TopicHeader allocation.
        let offset = unsafe { field_ptr.offset_from(base) } as usize;
        assert_eq!(offset, 48, "topic_kind should be at byte offset 48");
    }

    // ── read_topic_header_info ─────────────────────────────────────────

    #[test]
    fn read_topic_header_info_roundtrip() {
        use memmap2::MmapMut;

        let path =
            std::env::temp_dir().join(format!("horus_hdr_info_test_{}.bin", std::process::id()));

        // Write a valid header to temp file
        {
            let file = std::fs::OpenOptions::new()
                .read(true)
                .write(true)
                .create(true)
                .truncate(true)
                .open(&path)
                .expect("create temp file");
            file.set_len(TOPIC_HEADER_SIZE as u64)
                .expect("set file size");
            let mut mmap = unsafe { MmapMut::map_mut(&file).expect("mmap") };
            let header = unsafe { &mut *(mmap.as_mut_ptr() as *mut TopicHeader) };
            *header = TopicHeader::zeroed();
            header.init(4, 4, true, 16, 16, "LaserScan", TopicKind::Data as u8);
            header.messages_total.store(12345, Ordering::Relaxed);
            mmap.flush().expect("flush");
        }

        // Read back via read_topic_header_info
        let info = read_topic_header_info(&path).expect("should read valid header");
        assert_eq!(info.type_name, "LaserScan");
        assert_eq!(info.messages_total, 12345);
        assert_eq!(info.topic_kind, TopicKind::Data as u8);
        assert!(info.is_pod);
        assert_eq!(info.type_size, 4);

        let _ = std::fs::remove_file(&path);
    }

    #[test]
    fn read_topic_header_info_invalid_magic() {
        let path =
            std::env::temp_dir().join(format!("horus_hdr_bad_magic_{}.bin", std::process::id()));

        // Write garbage
        std::fs::write(&path, [0u8; 640]).expect("write garbage file");
        assert!(
            read_topic_header_info(&path).is_none(),
            "should return None for invalid magic"
        );

        let _ = std::fs::remove_file(&path);
    }

    #[test]
    fn read_topic_header_info_file_too_small() {
        let path = std::env::temp_dir().join(format!("horus_hdr_small_{}.bin", std::process::id()));

        std::fs::write(&path, [0u8; 100]).expect("write small file");
        assert!(
            read_topic_header_info(&path).is_none(),
            "should return None for file smaller than header"
        );

        let _ = std::fs::remove_file(&path);
    }

    #[test]
    fn read_topic_header_info_nonexistent() {
        assert!(
            read_topic_header_info(std::path::Path::new("/tmp/horus_nonexistent_42")).is_none()
        );
    }
}
