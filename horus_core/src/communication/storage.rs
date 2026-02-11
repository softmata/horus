//! Unified Shared Memory Layout for Smart IPC
//!
//! This module defines the unified memory layout that supports all access patterns:
//! - **Pod**: Single slot, latest-value semantics (~50ns)
//! - **SPSC**: Ring buffer, single producer/consumer (~85ns)
//! - **MPMC**: Ring buffer, multiple producers/consumers (~167ns)
//!
//! # Memory Layout (384 bytes total header)
//!
//! ```text
//! ┌──────────────────────────────────────────────────────────────┐
//! │ CoreHeader (64 bytes) - Cache Line 0                         │
//! │   ├─ magic: u64 (8 bytes) - "HORUS_V4"                       │
//! │   ├─ version_mode: u32 (4 bytes) - version|flags|mode        │
//! │   ├─ element_size: u32 (4 bytes)                             │
//! │   ├─ capacity: u32 (4 bytes) - always power of 2             │
//! │   ├─ consumer_count: u32 (4 bytes)                           │
//! │   ├─ data_offset: u32 (4 bytes)                              │
//! │   └─ reserved: [u8; 36]                                      │
//! ├──────────────────────────────────────────────────────────────┤
//! │ Alignment Padding (64 bytes) - aligns ProducerLine to 128    │
//! ├──────────────────────────────────────────────────────────────┤
//! │ ProducerLine (128 bytes) - Cache Lines 2-3                   │
//! │   ├─ head: u32 (4 bytes) - next write position               │
//! │   ├─ sequence: u64 (8 bytes) - monotonic counter             │
//! │   └─ reserved: [u8; 116]                                     │
//! ├──────────────────────────────────────────────────────────────┤
//! │ ConsumerLine (128 bytes) - Cache Lines 4-5                   │
//! │   ├─ min_tail: u32 (4 bytes) - slowest consumer              │
//! │   └─ reserved: [u8; 124]                                     │
//! ├──────────────────────────────────────────────────────────────┤
//! │ Data Section (variable size)                                 │
//! │   ├─ Pod: [T; 1] single slot                                 │
//! │   └─ SPSC/MPMC: [T; capacity] ring buffer                    │
//! └──────────────────────────────────────────────────────────────┘
//! ```
//!
//! # Design Principles
//!
//! 1. **Cache Line Separation**: Producer-written fields (`head`, `sequence`) are on
//!    separate 128-byte aligned cache lines from consumer-polled fields to prevent
//!    false sharing. Intel CPUs prefetch adjacent lines in pairs, so 128-byte
//!    alignment prevents prefetcher-induced invalidation.
//!
//! 2. **Mode Agnostic Core**: The 64-byte `CoreHeader` contains all metadata needed
//!    to interpret the data section, regardless of access mode.
//!
//! 3. **Zero-Overhead Pod**: Pod mode uses the same layout but ignores ring buffer
//!    fields. Readers only check `sequence` to detect updates.
//!
//! 4. **Runtime Mode Upgrade**: Topics can be upgraded from SPSC to MPMC at runtime
//!    by atomically updating the mode field. The memory layout is identical.

use std::sync::atomic::{AtomicU32, AtomicU64, Ordering};

/// Maximum total shared memory size - 100MB
const MAX_TOTAL_SIZE: usize = 100_000_000;

/// Header size for memory calculations
/// Note: 384 bytes due to 128-byte alignment padding after CoreHeader
pub(crate) const UNIFIED_HEADER_SIZE: usize = 384; // 64 + (64 padding) + 128 + 128

/// Access mode for the topic
///
/// The mode determines how the data section is interpreted and which
/// optimizations are applied to reads and writes.
#[repr(u8)]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum AccessMode {
    /// Single slot, latest-value semantics
    ///
    /// Best for: Transforms, sensor readings, configuration, state
    /// Performance: ~50ns (no ring buffer overhead)
    ///
    /// Behavior:
    /// - Writer overwrites the single slot
    /// - Reader always gets the latest value
    /// - Sequence number detects updates
    Pod = 0,

    /// Ring buffer, single producer/consumer
    ///
    /// Best for: Command streams, ordered events between two nodes
    /// Performance: ~85ns (no contention handling)
    ///
    /// Behavior:
    /// - Producer advances head atomically
    /// - Consumer tracks tail locally (no atomics)
    /// - Can upgrade to MPMC at runtime
    Spsc = 1,

    /// Ring buffer, multiple producers/consumers
    ///
    /// Best for: Event buses, shared logging, broadcast
    /// Performance: ~167ns (full contention handling)
    ///
    /// Behavior:
    /// - Producers use CAS on head
    /// - Each consumer tracks own tail
    /// - Backpressure via min_tail
    Mpmc = 2,
}

impl AccessMode {
    /// Create from raw u8 value
    #[inline]
    pub const fn from_u8(value: u8) -> Option<Self> {
        match value {
            0 => Some(Self::Pod),
            1 => Some(Self::Spsc),
            2 => Some(Self::Mpmc),
            _ => None,
        }
    }

    /// Check if this mode supports multiple consumers
    #[inline]
    pub const fn is_multi_consumer(self) -> bool {
        matches!(self, Self::Mpmc)
    }

    /// Check if this mode uses a ring buffer
    #[inline]
    pub const fn is_ring_buffer(self) -> bool {
        matches!(self, Self::Spsc | Self::Mpmc)
    }
}

/// Flags for topic configuration (stored in version_mode field)
#[repr(u8)]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum TopicFlag {
    /// No special flags
    None = 0,
    /// Topic supports runtime mode upgrade
    Upgradeable = 1 << 0,
    /// Topic uses zero-copy loan/receive API
    ZeroCopy = 1 << 1,
    /// Topic tracks per-consumer tails for precise backpressure
    PerConsumerTracking = 1 << 2,
}

/// Core header - 64 bytes (single cache line)
///
/// Contains all metadata needed to interpret the shared memory region.
/// This section is read-mostly after initialization.
///
/// # Safety
///
/// The `magic` field must be written LAST with Release ordering after
/// all other fields are initialized. Non-owners spin on this field
/// with Acquire ordering before reading other fields.
#[repr(C, align(64))]
pub(crate) struct CoreHeader {
    /// Magic number "HORUS_V4" (0x484F5255535F5634)
    ///
    /// Written LAST during initialization to signal completion.
    /// Non-owners spin on this value before reading other fields.
    pub magic: AtomicU64, // 8 bytes

    /// Packed field: version (16 bits) | flags (8 bits) | mode (8 bits)
    ///
    /// - Bits 0-7: AccessMode (Pod=0, Spsc=1, Mpmc=2)
    /// - Bits 8-15: TopicFlags
    /// - Bits 16-31: Layout version (currently 4)
    pub version_mode: AtomicU32, // 4 bytes

    /// Size of each element in bytes
    ///
    /// Maximum: MAX_ELEMENT_SIZE (1MB)
    pub element_size: AtomicU32, // 4 bytes

    /// Number of slots in the data section
    ///
    /// - Pod mode: always 1
    /// - SPSC/MPMC: power of 2 for efficient modulo (bitwise AND)
    pub capacity: AtomicU32, // 4 bytes

    /// Number of registered consumers
    ///
    /// Used for:
    /// - MPMC backpressure calculation
    /// - Cleanup when last consumer exits
    pub consumer_count: AtomicU32, // 4 bytes

    /// Offset from header start to data section
    ///
    /// Accounts for alignment requirements of the element type.
    pub data_offset: AtomicU32, // 4 bytes

    /// Reserved for future extensions
    ///
    /// Potential uses:
    /// - Timestamp of last write
    /// - QoS parameters
    /// - Compression flags
    _reserved: [u8; 36], // 36 bytes
}

// Verify CoreHeader is exactly 64 bytes
const _: () = assert!(std::mem::size_of::<CoreHeader>() == 64);

#[allow(dead_code)] // Methods used in tests and reserved for runtime mode upgrades
impl CoreHeader {
    /// Pack version, flags, and mode into a single u32
    #[inline]
    pub const fn pack_version_mode(version: u16, flags: u8, mode: AccessMode) -> u32 {
        ((version as u32) << 16) | ((flags as u32) << 8) | (mode as u32)
    }

    /// Extract mode from packed version_mode value
    #[inline]
    pub const fn unpack_mode(packed: u32) -> u8 {
        (packed & 0xFF) as u8
    }

    /// Extract flags from packed version_mode value
    #[inline]
    pub const fn unpack_flags(packed: u32) -> u8 {
        ((packed >> 8) & 0xFF) as u8
    }

    /// Extract version from packed version_mode value
    #[inline]
    pub const fn unpack_version(packed: u32) -> u16 {
        ((packed >> 16) & 0xFFFF) as u16
    }

    /// Get the current access mode
    #[inline]
    pub fn mode(&self) -> Option<AccessMode> {
        let packed = self.version_mode.load(Ordering::Acquire);
        AccessMode::from_u8(Self::unpack_mode(packed))
    }

    /// Atomically upgrade mode from SPSC to MPMC
    ///
    /// Returns `true` if upgrade succeeded, `false` if already MPMC or Pod.
    #[inline]
    pub fn upgrade_to_mpmc(&self) -> bool {
        let old = self.version_mode.load(Ordering::Acquire);
        let mode = Self::unpack_mode(old);

        // Only upgrade from SPSC
        if mode != AccessMode::Spsc as u8 {
            return false;
        }

        let new = (old & !0xFF) | (AccessMode::Mpmc as u32);
        self.version_mode
            .compare_exchange(old, new, Ordering::AcqRel, Ordering::Acquire)
            .is_ok()
    }
}

/// Producer state - 128 bytes (two cache lines)
///
/// Contains fields written frequently by producers. Aligned to 128 bytes
/// to prevent false sharing with consumer-polled fields.
///
/// # Intel Spatial Prefetcher
///
/// Intel CPUs prefetch adjacent cache lines in pairs (64+64 = 128 bytes).
/// By aligning producer fields to 128 bytes, we ensure that consumer
/// reads don't inadvertently pull in producer-written cache lines.
#[repr(C, align(128))]
pub(crate) struct ProducerLine {
    /// Next write position (ring buffer index)
    ///
    /// For Pod mode: always 0 (single slot)
    /// For SPSC: incremented without CAS (single producer)
    /// For MPMC: updated with CAS (multiple producers)
    ///
    /// Uses u32 instead of usize to reduce instruction cache pressure
    /// on x86-64 (avoids REX prefix overhead).
    pub head: AtomicU32, // 4 bytes

    /// Padding for 8-byte alignment of sequence
    _pad: u32, // 4 bytes

    /// Monotonically increasing sequence number
    ///
    /// Incremented on every successful publish. Used for:
    /// - Detecting new data in Pod mode
    /// - Wait-free reads (compare sequence before/after)
    /// - Overflow protection (u64 won't overflow in practice)
    pub sequence: AtomicU64, // 8 bytes

    /// Reserved for future use
    _reserved: [u8; 112], // 112 bytes
}

// Verify ProducerLine is exactly 128 bytes
const _: () = assert!(std::mem::size_of::<ProducerLine>() == 128);

/// Consumer state - 128 bytes (two cache lines)
///
/// Contains fields polled by consumers. Aligned to 128 bytes to prevent
/// false sharing with producer-written fields.
#[repr(C, align(128))]
pub(crate) struct ConsumerLine {
    /// Minimum tail position across all consumers
    ///
    /// Used for backpressure in MPMC mode. The producer checks that
    /// `head - min_tail < capacity` before writing to prevent
    /// overwriting unread data.
    ///
    /// In SPSC mode, this is the single consumer's tail.
    /// In Pod mode, this field is unused.
    pub min_tail: AtomicU32, // 4 bytes

    /// Reserved for future use
    ///
    /// Potential uses:
    /// - Per-consumer tail array for precise tracking
    /// - Consumer bitmap for presence detection
    _reserved: [u8; 124], // 124 bytes
}

// Verify ConsumerLine is exactly 128 bytes
const _: () = assert!(std::mem::size_of::<ConsumerLine>() == 128);

/// Complete unified header - 384 bytes total
///
/// This struct represents the full header at the start of shared memory.
/// The data section follows immediately after, aligned to the element type.
///
/// Note: Due to 128-byte alignment requirements of ProducerLine and
/// ConsumerLine, there is 64 bytes of implicit padding after CoreHeader.
#[repr(C)]
pub(crate) struct UnifiedShmHeader {
    /// Core metadata (64 bytes, cache line 0)
    pub core: CoreHeader,

    /// Producer state (128 bytes, cache lines 1-2)
    pub producer: ProducerLine,

    /// Consumer state (128 bytes, cache lines 3-4)
    pub consumer: ConsumerLine,
}

// Verify UnifiedShmHeader is exactly 320 bytes
const _: () = assert!(std::mem::size_of::<UnifiedShmHeader>() == UNIFIED_HEADER_SIZE);

#[allow(dead_code)] // Methods used in tests and for size calculations
impl UnifiedShmHeader {
    /// Calculate the data offset for a given element alignment
    ///
    /// Returns the offset from the start of the header to where the
    /// data section should begin, ensuring proper alignment.
    #[inline]
    pub const fn calculate_data_offset(element_align: usize) -> usize {
        let header_size = UNIFIED_HEADER_SIZE;
        // Round up to next multiple of element_align
        header_size.div_ceil(element_align) * element_align
    }

    /// Calculate total shared memory size for a topic
    ///
    /// Returns `None` if the calculation would overflow or exceed limits.
    #[inline]
    pub const fn calculate_total_size(
        element_size: usize,
        element_align: usize,
        capacity: usize,
    ) -> Option<usize> {
        let data_offset = Self::calculate_data_offset(element_align);

        // Check for overflow in data size calculation
        let data_size = match element_size.checked_mul(capacity) {
            Some(size) => size,
            None => return None,
        };

        // Check for overflow in total size calculation
        let total = match data_offset.checked_add(data_size) {
            Some(size) => size,
            None => return None,
        };

        // Check against maximum
        if total > MAX_TOTAL_SIZE {
            return None;
        }

        Some(total)
    }
}

/// Helper to round up to next power of 2
///
/// Used for ring buffer capacity to enable efficient modulo via bitwise AND.
#[inline]
pub const fn next_power_of_2(n: u32) -> u32 {
    if n == 0 {
        return 1;
    }
    let mut power = 1u32;
    while power < n {
        power <<= 1;
    }
    power
}

/// Check if a value is a power of 2
#[inline]
pub const fn is_power_of_2(n: u32) -> bool {
    n > 0 && (n & (n - 1)) == 0
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_header_sizes() {
        assert_eq!(std::mem::size_of::<CoreHeader>(), 64);
        assert_eq!(std::mem::size_of::<ProducerLine>(), 128);
        assert_eq!(std::mem::size_of::<ConsumerLine>(), 128);
        assert_eq!(std::mem::size_of::<UnifiedShmHeader>(), 384);
    }

    #[test]
    fn test_header_alignment() {
        assert_eq!(std::mem::align_of::<CoreHeader>(), 64);
        assert_eq!(std::mem::align_of::<ProducerLine>(), 128);
        assert_eq!(std::mem::align_of::<ConsumerLine>(), 128);
    }

    #[test]
    fn test_access_mode_conversion() {
        assert_eq!(AccessMode::from_u8(0), Some(AccessMode::Pod));
        assert_eq!(AccessMode::from_u8(1), Some(AccessMode::Spsc));
        assert_eq!(AccessMode::from_u8(2), Some(AccessMode::Mpmc));
        assert_eq!(AccessMode::from_u8(3), None);
    }

    #[test]
    fn test_version_mode_packing() {
        let packed = CoreHeader::pack_version_mode(4, 0x03, AccessMode::Spsc);
        assert_eq!(CoreHeader::unpack_version(packed), 4);
        assert_eq!(CoreHeader::unpack_flags(packed), 0x03);
        assert_eq!(CoreHeader::unpack_mode(packed), 1);
    }

    #[test]
    fn test_data_offset_calculation() {
        // For u8 (align 1), offset should be header size (384)
        assert_eq!(UnifiedShmHeader::calculate_data_offset(1), 384);

        // For u64 (align 8), offset should still be 384 (already aligned)
        assert_eq!(UnifiedShmHeader::calculate_data_offset(8), 384);

        // For 64-byte aligned type, offset should be 384 (already aligned)
        assert_eq!(UnifiedShmHeader::calculate_data_offset(64), 384);

        // For 128-byte aligned type, offset should be 384 (already aligned)
        assert_eq!(UnifiedShmHeader::calculate_data_offset(128), 384);

        // For 256-byte aligned type, offset should be 512
        assert_eq!(UnifiedShmHeader::calculate_data_offset(256), 512);
    }

    #[test]
    fn test_total_size_calculation() {
        // 1024 elements of 64 bytes each, 8-byte alignment
        let size = UnifiedShmHeader::calculate_total_size(64, 8, 1024);
        assert_eq!(size, Some(384 + 64 * 1024));

        // Pod mode: 1 element
        let size = UnifiedShmHeader::calculate_total_size(256, 8, 1);
        assert_eq!(size, Some(384 + 256));

        // Overflow check
        let size = UnifiedShmHeader::calculate_total_size(usize::MAX, 8, 2);
        assert_eq!(size, None);
    }

    #[test]
    fn test_power_of_2() {
        assert_eq!(next_power_of_2(0), 1);
        assert_eq!(next_power_of_2(1), 1);
        assert_eq!(next_power_of_2(2), 2);
        assert_eq!(next_power_of_2(3), 4);
        assert_eq!(next_power_of_2(5), 8);
        assert_eq!(next_power_of_2(1000), 1024);

        assert!(is_power_of_2(1));
        assert!(is_power_of_2(2));
        assert!(is_power_of_2(1024));
        assert!(!is_power_of_2(0));
        assert!(!is_power_of_2(3));
        assert!(!is_power_of_2(1000));
    }
}
