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
        }
    }
}
