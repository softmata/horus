//! # Smart Backend Detection for Topics
//!
//! This module provides automatic backend selection for `Topic::new()` based on:
//! - **POD Detection**: If the message type is registered as POD, use faster zero-copy paths
//! - **Topology Detection**: PID comparison to detect same-process vs cross-process
//! - **Access Pattern Tracking**: Count publishers/subscribers to select optimal backend
//!
//! ## How It Works
//!
//! 1. **At compile time**: POD types are registered via `register_pod_type!` macro
//! 2. **At runtime**: `Topic::new()` checks if `T` is a registered POD type
//! 3. **On first connection**: PID is recorded in shared memory header
//! 4. **On subsequent connections**: PID comparison determines same-process optimization
//!
//! ## Performance Impact
//!
//! | Scenario | Without Smart Detection | With Smart Detection |
//! |----------|------------------------|---------------------|
//! | POD type, same process | ~167ns (MpmcShm) | ~25ns (SpscIntra) |
//! | POD type, cross process | ~167ns (MpmcShm) | ~50ns (PodShm) |
//! | Non-POD, same process | ~167ns (MpmcShm) | ~80ns (MpmcIntra) |

use std::sync::atomic::{AtomicU32, AtomicU8, Ordering};

use crate::communication::pod::is_registered_pod;
use crate::error::{HorusError, HorusResult};

// ============================================================================
// Smart Detection Metadata Header
// ============================================================================

/// Magic number for smart detection header validation
const SMART_HEADER_MAGIC: u32 = 0x534D5254; // "SMRT"
const SMART_HEADER_VERSION: u32 = 1;

/// Shared memory header for smart topic detection.
///
/// This header is stored at the beginning of every topic's shared memory region
/// and enables automatic backend optimization.
///
/// Layout: 64 bytes (cache-line aligned)
#[repr(C, align(64))]
pub struct SmartTopicHeader {
    /// Magic number for validation ("SMRT")
    pub magic: u32,
    /// Header version for compatibility
    pub version: u32,
    /// Type hash (TypeId bits) for type safety
    pub type_hash: u64,
    /// Size of the message type in bytes
    pub type_size: u32,
    /// Alignment requirement
    pub type_align: u32,
    /// Is the type registered as POD? (0=unknown, 1=no, 2=yes)
    pub is_pod: AtomicU8,
    /// Reserved for flags
    pub _flags: u8,
    /// Number of active publishers
    pub publisher_count: AtomicU32,
    /// Number of active subscribers
    pub subscriber_count: AtomicU32,
    /// PID of the process that created this topic
    pub creator_pid: u32,
    /// Padding to 64 bytes
    pub _pad: [u8; 22],
}

impl SmartTopicHeader {
    /// Check if the header has valid magic number
    #[inline]
    pub fn is_valid(&self) -> bool {
        self.magic == SMART_HEADER_MAGIC
    }

    /// Get detected access pattern based on publisher/subscriber counts
    #[inline]
    pub fn detected_access_pattern(&self) -> DetectedPattern {
        let pubs = self.publisher_count.load(Ordering::Acquire);
        let subs = self.subscriber_count.load(Ordering::Acquire);

        match (pubs, subs) {
            (0, _) | (_, 0) => DetectedPattern::Unknown,
            (1, 1) => DetectedPattern::Spsc,
            (1, _) => DetectedPattern::Spmc,
            (_, 1) => DetectedPattern::Mpsc,
            (_, _) => DetectedPattern::Mpmc,
        }
    }

    /// Check if this process is the creator (same-process optimization possible)
    #[inline]
    pub fn is_same_process(&self) -> bool {
        self.creator_pid == std::process::id()
    }

    /// Check if type is registered as POD
    #[inline]
    pub fn is_pod_type(&self) -> bool {
        self.is_pod.load(Ordering::Acquire) == 2
    }

    /// Increment publisher count and return new count
    #[inline]
    pub fn add_publisher(&self) -> u32 {
        self.publisher_count.fetch_add(1, Ordering::AcqRel) + 1
    }

    /// Decrement publisher count and return new count
    #[inline]
    pub fn remove_publisher(&self) -> u32 {
        self.publisher_count.fetch_sub(1, Ordering::AcqRel).saturating_sub(1)
    }

    /// Increment subscriber count and return new count
    #[inline]
    pub fn add_subscriber(&self) -> u32 {
        self.subscriber_count.fetch_add(1, Ordering::AcqRel) + 1
    }

    /// Decrement subscriber count and return new count
    #[inline]
    pub fn remove_subscriber(&self) -> u32 {
        self.subscriber_count.fetch_sub(1, Ordering::AcqRel).saturating_sub(1)
    }
}

// ============================================================================
// Detection Results
// ============================================================================

/// Detected access pattern from smart header
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DetectedPattern {
    /// Not enough data to determine
    Unknown,
    /// Single producer, single consumer
    Spsc,
    /// Single producer, multiple consumers
    Spmc,
    /// Multiple producers, single consumer
    Mpsc,
    /// Multiple producers, multiple consumers
    Mpmc,
}

/// Recommended backend based on smart detection
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RecommendedBackend {
    /// POD shared memory (~50ns) - for registered POD types, cross-process
    PodShm,
    /// SPSC shared memory (~85ns) - single producer/consumer, cross-process
    SpscShm,
    /// MPSC shared memory (~65ns) - multiple producers, single consumer, cross-process
    MpscShm,
    /// SPMC shared memory (~70ns) - single producer, multiple consumers, cross-process
    SpmcShm,
    /// MPMC shared memory (~167ns) - default, most flexible
    MpmcShm,
    /// SPSC in-process (~25ns) - single producer/consumer, same process
    SpscIntra,
    /// MPSC in-process (~35ns) - multiple producers, single consumer, same process
    MpscIntra,
    /// SPMC in-process (~40ns) - single producer, multiple consumers, same process
    SpmcIntra,
    /// MPMC in-process (~80ns) - multiple producers/consumers, same process
    MpmcIntra,
}

/// Detection result with all collected information
#[derive(Debug, Clone)]
pub struct DetectionResult {
    /// Whether the type is a registered POD type
    pub is_pod: bool,
    /// Type size in bytes
    pub type_size: usize,
    /// Whether same-process optimization is possible
    pub same_process: bool,
    /// Detected access pattern
    pub pattern: DetectedPattern,
    /// Recommended backend based on detection
    pub recommended: RecommendedBackend,
}

// ============================================================================
// Smart Detection Functions
// ============================================================================

/// Detect the optimal backend for a type.
///
/// This is called by `Topic::new()` to automatically select the best backend.
///
/// # Arguments
/// * `is_pod` - Whether the type is registered as POD (from `is_registered_pod::<T>()`)
/// * `type_size` - Size of the type in bytes
/// * `same_process` - Whether the topic creator is in the same process
/// * `pattern` - Detected access pattern from shared memory header
pub fn detect_optimal_backend(
    is_pod: bool,
    _type_size: usize,
    same_process: bool,
    pattern: DetectedPattern,
) -> RecommendedBackend {
    // Priority: same-process > POD > pattern-specific

    if same_process {
        // Same process - use in-process backends (no shared memory overhead)
        match pattern {
            DetectedPattern::Spsc => RecommendedBackend::SpscIntra,
            DetectedPattern::Spmc => RecommendedBackend::SpmcIntra,
            DetectedPattern::Mpsc => RecommendedBackend::MpscIntra,
            DetectedPattern::Mpmc | DetectedPattern::Unknown => RecommendedBackend::MpmcIntra,
        }
    } else if is_pod {
        // Cross-process with POD type - use PodShm for zero-copy
        RecommendedBackend::PodShm
    } else {
        // Cross-process with non-POD - use pattern-specific shared memory
        match pattern {
            DetectedPattern::Spsc => RecommendedBackend::SpscShm,
            DetectedPattern::Spmc => RecommendedBackend::SpmcShm,
            DetectedPattern::Mpsc => RecommendedBackend::MpscShm,
            DetectedPattern::Mpmc | DetectedPattern::Unknown => RecommendedBackend::MpmcShm,
        }
    }
}

/// Perform full smart detection for a type.
///
/// # Type Parameters
/// * `T` - The message type
///
/// # Arguments
/// * `header` - Reference to the smart topic header (if available)
pub fn smart_detect<T: 'static>(header: Option<&SmartTopicHeader>) -> DetectionResult {
    let is_pod = is_registered_pod::<T>();
    let type_size = std::mem::size_of::<T>();

    let (same_process, pattern) = if let Some(h) = header {
        (h.is_same_process(), h.detected_access_pattern())
    } else {
        // No header yet - assume cross-process, unknown pattern
        (false, DetectedPattern::Unknown)
    };

    let recommended = detect_optimal_backend(is_pod, type_size, same_process, pattern);

    DetectionResult {
        is_pod,
        type_size,
        same_process,
        pattern,
        recommended,
    }
}

/// Initialize a smart topic header in shared memory.
///
/// # Arguments
/// * `header` - Mutable pointer to the header location
/// * `type_hash` - TypeId bits for the message type
/// * `type_size` - Size of the message type in bytes
/// * `type_align` - Alignment of the message type
/// * `is_pod` - Whether the type is registered as POD
/// * `is_producer` - Whether this endpoint is a producer
///
/// # Safety
/// The header pointer must be valid and properly aligned.
pub unsafe fn init_smart_header(
    header: *mut SmartTopicHeader,
    type_hash: u64,
    type_size: u32,
    type_align: u32,
    is_pod: bool,
    is_producer: bool,
) {
    (*header).magic = SMART_HEADER_MAGIC;
    (*header).version = SMART_HEADER_VERSION;
    (*header).type_hash = type_hash;
    (*header).type_size = type_size;
    (*header).type_align = type_align;
    (*header).is_pod = AtomicU8::new(if is_pod { 2 } else { 1 });
    (*header)._flags = 0;
    (*header).publisher_count = AtomicU32::new(if is_producer { 1 } else { 0 });
    (*header).subscriber_count = AtomicU32::new(if is_producer { 0 } else { 1 });
    (*header).creator_pid = std::process::id();
    (*header)._pad = [0u8; 22];
}

/// Validate and join an existing smart topic.
///
/// # Arguments
/// * `header` - Pointer to the header
/// * `type_hash` - Expected TypeId bits
/// * `type_size` - Expected type size
/// * `is_producer` - Whether this endpoint is a producer
///
/// # Safety
/// The header pointer must be valid.
///
/// # Returns
/// Ok(()) if validation succeeds, Err with description otherwise.
pub unsafe fn join_smart_header(
    header: *mut SmartTopicHeader,
    type_hash: u64,
    type_size: u32,
    is_producer: bool,
) -> HorusResult<()> {
    let h = &*header;

    // Validate magic
    if h.magic != SMART_HEADER_MAGIC {
        return Err(HorusError::Communication(format!(
            "Invalid smart header magic: expected 0x{:X}, got 0x{:X}",
            SMART_HEADER_MAGIC, h.magic
        )));
    }

    // Validate type
    if h.type_hash != type_hash {
        return Err(HorusError::Communication(format!(
            "Type hash mismatch: expected 0x{:X}, got 0x{:X}",
            type_hash, h.type_hash
        )));
    }

    if h.type_size != type_size {
        return Err(HorusError::Communication(format!(
            "Type size mismatch: expected {}, got {}",
            type_size, h.type_size
        )));
    }

    // Register this endpoint
    if is_producer {
        h.publisher_count.fetch_add(1, Ordering::AcqRel);
    } else {
        h.subscriber_count.fetch_add(1, Ordering::AcqRel);
    }

    Ok(())
}

/// Get the type hash for a type (using TypeId bits).
#[inline]
pub fn type_hash<T: 'static>() -> u64 {
    use std::any::TypeId;
    use std::hash::{Hash, Hasher};
    use std::collections::hash_map::DefaultHasher;

    let type_id = TypeId::of::<T>();
    let mut hasher = DefaultHasher::new();
    type_id.hash(&mut hasher);
    hasher.finish()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_smart_header_size() {
        assert_eq!(std::mem::size_of::<SmartTopicHeader>(), 64);
    }

    #[test]
    fn test_detect_optimal_backend_same_process() {
        // Same process should use intra backends
        assert_eq!(
            detect_optimal_backend(false, 100, true, DetectedPattern::Spsc),
            RecommendedBackend::SpscIntra
        );
        assert_eq!(
            detect_optimal_backend(true, 100, true, DetectedPattern::Mpmc),
            RecommendedBackend::MpmcIntra // Same-process beats POD
        );
    }

    #[test]
    fn test_detect_optimal_backend_pod_cross_process() {
        // POD types cross-process should use PodShm
        assert_eq!(
            detect_optimal_backend(true, 100, false, DetectedPattern::Spsc),
            RecommendedBackend::PodShm
        );
        assert_eq!(
            detect_optimal_backend(true, 100, false, DetectedPattern::Mpmc),
            RecommendedBackend::PodShm
        );
    }

    #[test]
    fn test_detect_optimal_backend_non_pod_cross_process() {
        // Non-POD cross-process should use pattern-specific
        assert_eq!(
            detect_optimal_backend(false, 100, false, DetectedPattern::Spsc),
            RecommendedBackend::SpscShm
        );
        assert_eq!(
            detect_optimal_backend(false, 100, false, DetectedPattern::Unknown),
            RecommendedBackend::MpmcShm
        );
    }

    #[test]
    fn test_type_hash_consistency() {
        // Same type should have same hash
        let hash1 = type_hash::<u64>();
        let hash2 = type_hash::<u64>();
        assert_eq!(hash1, hash2);

        // Different types should have different hashes
        let hash3 = type_hash::<u32>();
        assert_ne!(hash1, hash3);
    }
}
