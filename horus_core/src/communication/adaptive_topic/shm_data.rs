//! ShmData backend — cross-process communication via shared memory.
//!
//! For cross-process backends, the data ring lives in its own ShmRegion
//! (separate from the discovery header). This module wraps the SHM pointer
//! arithmetic and provides the same try_send/try_recv interface as the
//! intra-process backends.

use std::sync::Arc;

use crate::memory::shm_region::ShmRegion;

use super::types::AdaptiveBackendMode;

/// Cross-process data backend backed by a separate ShmRegion.
///
/// The discovery header (640 bytes) is always in its own ShmRegion for
/// cross-process rendezvous. This struct holds the DATA ShmRegion which
/// contains the actual ring buffer slots.
///
/// For the initial implementation, the data region is the same ShmRegion
/// as the header (data starts at HEADER_SIZE offset). In Phase 4, this will
/// be split into a separate "{topic}_data" region.
pub(crate) struct ShmDataBackend {
    /// Shared memory region containing data (may be same as or separate from header)
    pub data_region: Arc<ShmRegion>,
    /// Offset within data_region where ring data starts
    pub data_offset: usize,
    /// Current backend mode
    pub mode: AdaptiveBackendMode,
    /// Whether the type is POD
    pub is_pod: bool,
    /// Slot size in bytes (for non-POD serialized messages)
    pub slot_size: usize,
    /// Ring capacity
    pub capacity: u64,
    /// Capacity mask (capacity - 1)
    pub mask: u64,
}

impl ShmDataBackend {
    /// Get a raw pointer to the data region start.
    #[inline(always)]
    pub fn data_ptr(&self) -> *mut u8 {
        // SAFETY: data_offset is within bounds of the ShmRegion (set during construction)
        unsafe { (self.data_region.as_ptr() as *mut u8).add(self.data_offset) }
    }
}
