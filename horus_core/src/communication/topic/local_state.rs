//! Local state for a Topic participant.
//!
//! Caches frequently accessed values locally to avoid reading from shared
//! memory on the hot path.

use super::header::TopicHeader;
use super::types::{BackendMode, TopicRole};

/// Default serialized message slot size (8KB)
pub(crate) const DEFAULT_SLOT_SIZE: usize = 8 * 1024;

/// Lease refresh interval - refresh every N messages instead of every message
/// This avoids calling SystemTime::now() syscall on the hot path
pub(crate) const LEASE_REFRESH_INTERVAL: u32 = 1024;

/// Epoch check interval — reads migration_epoch from SHM header every N messages.
/// Must be a power of two (used with bitmask).
///
/// Set to 32 (~1ns amortized) to detect cross-process migration promptly.
/// At 1 Hz this checks every ~4 s; at 30 Hz every ~133 ms; at 1 kHz every ~4 ms.
/// Lowered from 32 to 4 to fix cross-process discovery at low frequencies
/// (GitHub issue #37: 1Hz publisher took 32s to detect a new subscriber).
/// Cost: one Relaxed mmap load (~20ns) every 4 messages ≈ 5ns amortized.
pub(crate) const EPOCH_CHECK_INTERVAL: u32 = 4;

/// Local state for an Topic participant
///
/// ## Cache-Optimized Design
///
/// Fields are ordered to match the ACCESS PATTERN in send():
/// 1. First: cached_mode (branch decision)
/// 2. Then: local_head, cached_data_ptr, cached_capacity_mask (DirectChannel hot path)
/// 3. Then: local_tail, cached_capacity, cached_header_ptr (SpscIntra additions)
///
/// First cache line (0-63): ALL hot path fields in access order
/// Second cache line (64+): Cold path fields (registration, migration, etc.)
#[repr(C)] // Prevent compiler reordering - layout is critical for performance
pub(crate) struct LocalState {
    // ========== FIRST CACHE LINE (0-63 bytes) - HOT PATH ==========
    /// Cached backend mode - FIRST field because it's checked FIRST in send()
    pub cached_mode: BackendMode, // offset 0 (1 byte)

    /// Our role (accessed early in some paths)
    pub role: TopicRole, // offset 1 (1 byte)

    /// Is POD type (cached for performance)
    pub is_pod: bool, // offset 2 (1 byte)

    /// Cached is_same_process result
    pub is_same_process: bool, // offset 3 (1 byte)

    /// Message counter for sampling lease refresh (4 bytes, align to 4)
    pub msg_counter: u32, // offset 4

    /// Locally cached head index - CRITICAL: accessed immediately after mode check
    pub local_head: u64, // offset 8

    /// Cached capacity mask for bitwise AND (seq & mask)
    pub cached_capacity_mask: u64, // offset 16

    /// Cached pointer to data region - for ring buffer write
    /// SAFETY: Valid for Topic lifetime (points into Arc<ShmRegion>)
    pub cached_data_ptr: *mut u8, // offset 24

    /// Locally cached tail index (for backpressure check in SpscIntra)
    pub local_tail: u64, // offset 32

    /// Cached capacity (for backpressure check)
    pub cached_capacity: u64, // offset 40

    /// Cached pointer to header - for atomic updates in SpscIntra
    /// SAFETY: Valid for Topic lifetime (points into Arc<ShmRegion>)
    pub cached_header_ptr: *const TopicHeader, // offset 48

    /// Cached pointer to per-slot sequence array (for multi-producer ready flags)
    /// SAFETY: Valid for Topic lifetime (points into Arc<ShmRegion>)
    pub cached_seq_ptr: *mut u8, // offset 56

    // ========== SECOND CACHE LINE (64+ bytes) - COLD PATH ==========
    /// Our slot index in the participant array (-1 if not registered)
    pub slot_index: i32,

    /// Slot size for serialized messages (non-POD)
    pub slot_size: usize,

    /// Cached epoch (to detect migrations)
    pub cached_epoch: u64,

    /// SHM FanoutRing publisher ID (registered on first send when mode is FanoutShm)
    pub fanout_shm_pub_id: Option<usize>,

    /// SHM FanoutRing subscriber ID (registered on first recv when mode is FanoutShm)
    pub fanout_shm_sub_id: Option<usize>,

    /// COMM-H1: exclusive `flock` proving this process holds the FanoutShm publisher
    /// endpoint slot. Held for the endpoint's lifetime — its Drop (or process death)
    /// releases the lock, which is what lets a peer reclaim a crashed owner's slot.
    /// `None` for every non-FanoutShm backend.
    pub fanout_pub_lock: Option<horus_sys::fs::FileLock>,

    /// COMM-H1: exclusive `flock` proving this process holds the FanoutShm
    /// subscriber endpoint slot (symmetric to `fanout_pub_lock`).
    pub fanout_sub_lock: Option<horus_sys::fs::FileLock>,

    /// SHM region backing the fanout channel matrix (must stay alive for lifetime of ring)
    pub fanout_shm_storage: Option<std::sync::Arc<crate::memory::shm_region::ShmRegion>>,

    /// COMM-H3: producer keep-alive for spilled FanoutShm messages. Each large
    /// (> SPILL_THRESHOLD) serde message is copied into a TensorPool slot and only
    /// a 40-byte descriptor is broadcast; the slot must stay alive until every
    /// subscriber that can still see the descriptor has read it. This holds the
    /// last `capacity` (ring window R) spilled tensors and releases each one once
    /// its ring position is overwritten (drop-oldest ⇒ no subscriber can reach it),
    /// so the spill pool can't fill up and start silently dropping large messages.
    /// Empty for every non-FanoutShm backend. See `dispatch::spill_to_pool` /
    /// `read_spilled_retained`.
    pub spill_keepalive: std::collections::VecDeque<crate::types::Tensor>,
}

impl Default for LocalState {
    fn default() -> Self {
        Self {
            cached_mode: BackendMode::Unknown,
            role: TopicRole::Unregistered,
            is_pod: false,
            is_same_process: true, // Assume same process until checked
            msg_counter: 0,
            local_head: 0,
            cached_capacity_mask: 0,
            cached_data_ptr: std::ptr::null_mut(),
            local_tail: 0,
            cached_capacity: 0,
            cached_header_ptr: std::ptr::null(),
            cached_seq_ptr: std::ptr::null_mut(),
            slot_index: -1,
            slot_size: DEFAULT_SLOT_SIZE,
            cached_epoch: 0,
            fanout_shm_pub_id: None,
            fanout_shm_sub_id: None,
            fanout_pub_lock: None,
            fanout_sub_lock: None,
            fanout_shm_storage: None,
            spill_keepalive: std::collections::VecDeque::new(),
        }
    }
}
