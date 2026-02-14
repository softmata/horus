//! Shared memory header for adaptive topic detection.
//!
//! The `AdaptiveTopicHeader` is laid out in shared memory for cross-process
//! rendezvous. It contains participant tracking, topology detection, and
//! migration coordination.

use std::mem;
use std::sync::atomic::{AtomicU32, AtomicU64, AtomicU8, Ordering};

use crate::error::{HorusError, HorusResult};

use super::types::AdaptiveBackendMode;

// ============================================================================
// Constants
// ============================================================================

/// Magic number for adaptive topic header validation
pub(crate) const ADAPTIVE_MAGIC: u64 = 0x4144415054495645; // "ADAPTIVE"

/// Header version for compatibility checking
/// v2: Added slot_size field for large message support
pub(crate) const ADAPTIVE_VERSION: u32 = 2;

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
pub struct ParticipantEntry {
    /// Process ID (0 = empty slot)
    /// MUST be atomic for cross-process visibility in shared memory
    pub pid: AtomicU32,
    /// Thread ID hash (lower 32 bits for same-thread detection)
    /// MUST be atomic for cross-process visibility in shared memory
    pub thread_id_hash: AtomicU32,
    /// Role: 0=none, 1=producer, 2=consumer, 3=both
    pub role: AtomicU8,
    /// Active flag for atomic operations
    pub active: AtomicU8,
    /// Padding for alignment
    pub _pad: [u8; 6],
    /// Last heartbeat timestamp (milliseconds since epoch)
    pub lease_expires_ms: AtomicU64,
}

impl ParticipantEntry {
    /// Check if this entry is empty (no participant)
    #[inline]
    pub fn is_empty(&self) -> bool {
        self.active.load(Ordering::Acquire) == 0
    }

    /// Check if the lease has expired
    #[inline]
    pub fn is_lease_expired(&self, now_ms: u64) -> bool {
        let expires = self.lease_expires_ms.load(Ordering::Acquire);
        if expires == 0 {
            return true;
        }
        now_ms > expires
    }

    /// Update lease expiration timestamp
    #[inline]
    pub fn refresh_lease(&self, now_ms: u64, timeout_ms: u64) {
        self.lease_expires_ms
            .store(now_ms + timeout_ms, Ordering::Release);
    }

    /// Clear this entry (use atomic for thread safety)
    pub fn clear(&self) {
        self.active.store(0, Ordering::Release);
        self.lease_expires_ms.store(0, Ordering::Release);
        self.role.store(0, Ordering::Release);
        self.pid.store(0, Ordering::Release);
        self.thread_id_hash.store(0, Ordering::Release);
    }
}

// ============================================================================
// Adaptive Topic Header (Cache-Optimized)
// ============================================================================

/// Shared memory header for adaptive topic detection.
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
pub struct AdaptiveTopicHeader {
    // === Cache line 1 (bytes 0-63): Core metadata (read-mostly) ===
    /// Magic number for validation ("ADAPTIVE")
    pub magic: u64,
    /// Header version for compatibility
    pub version: u32,
    /// Type size in bytes
    pub type_size: u32,
    /// Type alignment requirement
    pub type_align: u32,
    /// Is POD type: 0=unknown, 1=no, 2=yes
    pub is_pod: AtomicU8,
    /// Current backend mode (AdaptiveBackendMode as u8)
    pub backend_mode: AtomicU8,
    /// Migration lock: 0=unlocked, 1=locked
    pub migration_lock: AtomicU8,
    /// Reserved flags
    pub _flags: u8,
    /// Creator process ID
    pub creator_pid: u32,
    /// Creator thread ID hash (for same-thread detection)
    pub creator_thread_id_hash: u64,
    /// Migration epoch (incremented on each backend switch)
    pub migration_epoch: AtomicU64,
    /// Padding to 64 bytes
    pub _pad1: [u8; 16],

    // === Cache line 2 (bytes 64-127): PRODUCER WRITE LINE ===
    // This cache line is ONLY written by producers (senders)
    // NEVER put consumer-written fields here!
    /// Write sequence / head (for POD/ring backends) - PRODUCER ONLY
    pub sequence_or_head: AtomicU64,
    /// Ring buffer capacity (power of 2)
    pub capacity: u32,
    /// Capacity mask for fast modulo (capacity - 1, only valid if capacity is power of 2)
    pub capacity_mask: u32,
    /// Slot size in bytes (for non-POD types, includes header + data)
    pub slot_size: u32,
    /// Padding to fill cache line (64 - 8 - 4 - 4 - 4 = 44 bytes)
    pub _pad_producer: [u8; 44],

    // === Cache line 3 (bytes 128-191): CONSUMER WRITE LINE ===
    // This cache line is ONLY written by consumers (receivers)
    // NEVER put producer-written fields here!
    /// Read tail (for ring backends) - CONSUMER ONLY
    pub tail: AtomicU64,
    /// Padding to fill cache line (64 - 8 = 56 bytes)
    pub _pad_consumer: [u8; 56],

    // === Cache line 4 (bytes 192-255): Counters and metadata ===
    /// Number of active publishers
    pub publisher_count: AtomicU32,
    /// Number of active subscribers
    pub subscriber_count: AtomicU32,
    /// Total participants ever connected
    pub total_participants: AtomicU32,
    /// Lease timeout in milliseconds
    pub lease_timeout_ms: u32,
    /// Last topology change timestamp (ms)
    pub last_topology_change_ms: AtomicU64,
    /// Padding to 64 bytes (4+4+4+4+8 = 24, need 40 more)
    pub _pad_counters: [u8; 40],

    // === Cache lines 5-10 (bytes 256-639): Participant tracking (384 bytes = 16 * 24) ===
    /// Participant entries for lease management
    pub participants: [ParticipantEntry; MAX_PARTICIPANTS],
}

// Size assertion: Header must be exactly 640 bytes (10 cache lines)
const _: () = assert!(mem::size_of::<AdaptiveTopicHeader>() == 640);

impl AdaptiveTopicHeader {
    /// Create a zeroed header (for testing or pre-allocation)
    pub fn zeroed() -> Self {
        Self {
            magic: 0,
            version: 0,
            type_size: 0,
            type_align: 0,
            is_pod: AtomicU8::new(0),
            backend_mode: AtomicU8::new(0),
            migration_lock: AtomicU8::new(0),
            _flags: 0,
            creator_pid: 0,
            creator_thread_id_hash: 0,
            migration_epoch: AtomicU64::new(0),
            _pad1: [0; 16],
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
            _pad_counters: [0; 40],
            participants: std::array::from_fn(|_| ParticipantEntry {
                pid: AtomicU32::new(0),
                thread_id_hash: AtomicU32::new(0),
                role: AtomicU8::new(0),
                active: AtomicU8::new(0),
                _pad: [0; 6],
                lease_expires_ms: AtomicU64::new(0),
            }),
        }
    }

    /// Initialize a new header
    pub fn init(
        &mut self,
        type_size: u32,
        type_align: u32,
        is_pod: bool,
        capacity: u32,
        slot_size: u32,
    ) {
        // Ensure capacity is power of 2 for fast modulo
        let capacity = capacity.next_power_of_two();

        // Initialize all fields BEFORE setting magic (race condition fix)
        // Another process might check magic to determine if header is initialized
        self.version = ADAPTIVE_VERSION;
        self.type_size = type_size;
        self.type_align = type_align;
        self.is_pod
            .store(if is_pod { POD_YES } else { POD_NO }, Ordering::Release);
        self.backend_mode
            .store(AdaptiveBackendMode::Unknown as u8, Ordering::Release);
        self.migration_lock
            .store(MIGRATION_UNLOCKED, Ordering::Release);
        self._flags = 0;
        self.creator_pid = std::process::id();
        self.creator_thread_id_hash = hash_thread_id(std::thread::current().id());
        self.migration_epoch.store(0, Ordering::Release);

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

        // Clear all participant entries
        for p in &self.participants {
            p.clear();
        }

        // CRITICAL: Set magic LAST with a memory fence to ensure all prior writes
        // are visible to other processes before they see the magic value.
        std::sync::atomic::fence(Ordering::Release);
        self.magic = ADAPTIVE_MAGIC;
    }

    /// Check if the header has valid magic number
    #[inline]
    pub fn is_valid(&self) -> bool {
        self.magic == ADAPTIVE_MAGIC && self.version == ADAPTIVE_VERSION
    }

    /// Get the current backend mode
    #[inline]
    pub fn mode(&self) -> AdaptiveBackendMode {
        AdaptiveBackendMode::from(self.backend_mode.load(Ordering::Acquire))
    }

    /// Check if all active participants (and caller) are in the same process
    #[inline]
    pub fn is_same_process(&self) -> bool {
        let current_pid = std::process::id();

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
                let old_role = p.role.fetch_or(1, Ordering::AcqRel);
                if old_role & 1 == 0 {
                    self.publisher_count.fetch_add(1, Ordering::AcqRel);
                    self.last_topology_change_ms
                        .store(now_ms, Ordering::Release);
                }
                p.refresh_lease(now_ms, timeout_ms);
                return Ok(i);
            }
        }

        // Find empty slot or expired lease
        for (i, p) in self.participants.iter().enumerate() {
            let is_active = p.active.load(Ordering::Acquire) != 0;
            let is_expired = is_active && p.is_lease_expired(now_ms);

            if !is_active || is_expired {
                if is_expired {
                    let old_role = p.role.load(Ordering::Acquire);
                    if old_role & 1 != 0 {
                        self.publisher_count.fetch_sub(1, Ordering::AcqRel);
                    }
                    if old_role & 2 != 0 {
                        self.subscriber_count.fetch_sub(1, Ordering::AcqRel);
                    }
                }

                if p.active
                    .compare_exchange(0, 1, Ordering::AcqRel, Ordering::Acquire)
                    .is_ok()
                    || (is_expired
                        && p.active
                            .compare_exchange(1, 1, Ordering::AcqRel, Ordering::Acquire)
                            .is_ok())
                {
                    p.pid.store(pid, Ordering::Release);
                    p.thread_id_hash.store(thread_hash, Ordering::Release);
                    p.role.store(1, Ordering::Release); // Producer
                    p.refresh_lease(now_ms, timeout_ms);
                    self.publisher_count.fetch_add(1, Ordering::AcqRel);
                    self.total_participants.fetch_add(1, Ordering::AcqRel);
                    self.last_topology_change_ms
                        .store(now_ms, Ordering::Release);
                    return Ok(i);
                }
            }
        }

        Err(HorusError::Communication(
            "No available participant slots".to_string(),
        ))
    }

    /// Register as a subscriber (returns slot index or error)
    pub fn register_consumer(&self) -> HorusResult<usize> {
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
                let old_role = p.role.fetch_or(2, Ordering::AcqRel);
                if old_role & 2 == 0 {
                    self.subscriber_count.fetch_add(1, Ordering::AcqRel);
                    self.last_topology_change_ms
                        .store(now_ms, Ordering::Release);
                }
                p.refresh_lease(now_ms, timeout_ms);
                return Ok(i);
            }
        }

        // Find empty slot or expired lease
        for (i, p) in self.participants.iter().enumerate() {
            let is_active = p.active.load(Ordering::Acquire) != 0;
            let is_expired = is_active && p.is_lease_expired(now_ms);

            if !is_active || is_expired {
                if is_expired {
                    let old_role = p.role.load(Ordering::Acquire);
                    if old_role & 1 != 0 {
                        self.publisher_count.fetch_sub(1, Ordering::AcqRel);
                    }
                    if old_role & 2 != 0 {
                        self.subscriber_count.fetch_sub(1, Ordering::AcqRel);
                    }
                }

                if p.active
                    .compare_exchange(0, 1, Ordering::AcqRel, Ordering::Acquire)
                    .is_ok()
                    || (is_expired
                        && p.active
                            .compare_exchange(1, 1, Ordering::AcqRel, Ordering::Acquire)
                            .is_ok())
                {
                    p.pid.store(pid, Ordering::Release);
                    p.thread_id_hash.store(thread_hash, Ordering::Release);
                    p.role.store(2, Ordering::Release); // Consumer
                    p.refresh_lease(now_ms, timeout_ms);
                    self.subscriber_count.fetch_add(1, Ordering::AcqRel);
                    self.total_participants.fetch_add(1, Ordering::AcqRel);
                    self.last_topology_change_ms
                        .store(now_ms, Ordering::Release);
                    return Ok(i);
                }
            }
        }

        Err(HorusError::Communication(
            "No available participant slots".to_string(),
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

    /// Detect the optimal backend based on current topology
    pub fn detect_optimal_backend(&self) -> AdaptiveBackendMode {
        let pubs = self.pub_count();
        let subs = self.sub_count();
        let same_process = self.is_same_process();
        let all_same_thread = self.all_participants_same_thread();
        let is_pod = self.is_pod_type();

        match (all_same_thread, same_process, pubs, subs, is_pod) {
            (_, _, 0, 0, _) => AdaptiveBackendMode::Unknown,

            (true, _, 1, 1, true) => AdaptiveBackendMode::DirectChannel,

            // Intra-process: 1P↔1C (or anticipating single counterpart)
            (_, true, 1, 1, _) => AdaptiveBackendMode::SpscIntra,
            (_, true, 1, 0, _) => AdaptiveBackendMode::SpscIntra,
            (_, true, 0, 1, _) => AdaptiveBackendMode::SpscIntra,
            // Intra-process: 1P, multiple consumers
            (_, true, 1, _, _) if subs > 1 => AdaptiveBackendMode::SpmcIntra,
            // Intra-process: multiple producers (0 or 1 consumer)
            (_, true, _, 0, _) if pubs > 1 => AdaptiveBackendMode::MpscIntra,
            (_, true, _, 1, _) if pubs > 1 => AdaptiveBackendMode::MpscIntra,
            // Intra-process: 0 pubs, multiple consumers (anticipating single producer)
            (_, true, 0, _, _) if subs > 1 => AdaptiveBackendMode::SpmcIntra,
            // Intra-process: MPMC
            (_, true, _, _, _) if pubs > 1 && subs > 1 => AdaptiveBackendMode::MpmcIntra,

            // Cross-process: 1P↔1C
            (_, false, 1, 1, _) => AdaptiveBackendMode::SpscShm,
            (_, false, 1, 0, _) => AdaptiveBackendMode::SpscShm,
            (_, false, 0, 1, _) => AdaptiveBackendMode::SpscShm,
            // Cross-process: multi-producer
            (_, false, _, 0, _) if pubs > 1 => AdaptiveBackendMode::MpscShm,
            (_, false, _, 1, _) if pubs > 1 => AdaptiveBackendMode::MpscShm,
            // Cross-process: multi-consumer
            (_, false, 1, _, _) if subs > 1 => AdaptiveBackendMode::SpmcShm,
            (_, false, 0, _, _) if subs > 1 => AdaptiveBackendMode::SpmcShm,

            (_, false, _, _, true) => AdaptiveBackendMode::PodShm,

            _ => AdaptiveBackendMode::MpmcShm,
        }
    }

    /// Clean up expired leases and update counts
    pub fn cleanup_expired_leases(&self) -> u32 {
        let now_ms = current_time_ms();
        let mut cleaned = 0u32;

        for p in &self.participants {
            let is_active = p.active.load(Ordering::Acquire) != 0;
            if is_active && p.is_lease_expired(now_ms) {
                let old_role = p.role.load(Ordering::Acquire);
                if old_role & 1 != 0 {
                    self.publisher_count.fetch_sub(1, Ordering::AcqRel);
                }
                if old_role & 2 != 0 {
                    self.subscriber_count.fetch_sub(1, Ordering::AcqRel);
                }
                p.clear();
                cleaned += 1;
            }
        }

        if cleaned > 0 {
            self.last_topology_change_ms
                .store(now_ms, Ordering::Release);
        }

        cleaned
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
