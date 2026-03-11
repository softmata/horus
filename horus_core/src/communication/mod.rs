//! # Communication layer for HORUS
//!
//! This module provides the unified Topic API for all HORUS IPC needs.
//!
//! ## Topic - The Unified Communication API
//!
//! `Topic<T>` provides a single, consistent interface for all communication patterns.
//! It automatically selects the optimal backend based on topology and access patterns:
//!
//! | Backend | Latency | Detection Criteria |
//! |---------|---------|-------------------|
//! | DirectChannel | ~3ns | same_thread |
//! | SpscIntra | ~18ns | same_process, pubs=1, subs=1 |
//! | SpmcIntra | ~24ns | same_process, pubs=1, subs>1 |
//! | MpscIntra | ~26ns | same_process, pubs>1, subs=1 |
//! | MpmcIntra | ~36ns | same_process, pubs>1, subs>1 |
//! | PodShm | ~50ns | cross_process, is_pod |
//! | SpscShm | ~85ns | cross_process, pubs=1, subs=1, !is_pod |
//! | MpmcShm | ~167ns | cross_process, pubs>1, subs>1 |
//!
//! ## Usage
//!
//! ```rust,ignore
//! use horus_core::communication::Topic;
//!
//! // Just this - backend auto-selected
//! let topic: Topic<String> = Topic::new("sensor_data")?;
//! topic.send("hello".to_string());
//! let msg = topic.recv();
//! ```
//!
//! ## Automatic POD Detection
//!
//! HORUS automatically detects POD types - no registration needed!
//!
//! ```rust,ignore
//! struct MotorCommand { velocity: f32, torque: f32 }
//!
//! // HORUS automatically uses zero-copy path (~50ns)!
//! let topic: Topic<MotorCommand> = Topic::new("motor")?;
//! topic.send(MotorCommand { velocity: 1.0, torque: 0.5 });
//! ```

mod macros;
pub mod network;
pub mod pod;
pub mod topic;

// Re-export commonly used types for convenience
pub use pod::PodMessage;
pub use topic::{SendBlockingError, Topic, TopicDescriptor, TopicMessage};

// Debug flag API for external tools (TUI monitor)
#[doc(hidden)]
pub use topic::{set_topic_debug, TOPIC_DEBUG_LOG_OFFSET};

// Ring-buffer inspector for CLI tools (`horus topic echo`)
pub use topic::{read_latest_slot_bytes, read_topic_sequence, TopicSlotRead};

/// Write raw bytes into the latest slot of a topic SHM file (used by replay).
///
/// This is the write counterpart to `read_latest_slot_bytes`.  It writes
/// `data` into the next slot of the ring buffer and bumps the sequence counter.
/// Returns `true` on success.
pub fn write_topic_slot_bytes(path: &std::path::Path, data: &[u8]) -> bool {
    use memmap2::MmapOptions;
    use std::fs::OpenOptions;

    let file = match OpenOptions::new().read(true).write(true).open(path) {
        Ok(f) => f,
        Err(_) => return false,
    };
    let meta = match file.metadata() {
        Ok(m) => m,
        Err(_) => return false,
    };
    if meta.len() < topic::header::TOPIC_HEADER_SIZE as u64 {
        return false;
    }
    // SAFETY: file is opened read-write; mmap is used for in-place writes to SHM.
    let mut mmap = match unsafe { MmapOptions::new().map_mut(&file) } {
        Ok(m) => m,
        Err(_) => return false,
    };
    let base = mmap.as_mut_ptr();
    let len = mmap.len();

    // Validate magic
    // SAFETY: mmap is at least TOPIC_HEADER_SIZE (640) bytes; offset 0 is the magic field.
    let magic = unsafe { std::ptr::read_unaligned(base as *const u64) };
    if magic != topic::header::TOPIC_MAGIC {
        return false;
    }

    // Read layout fields from header
    // SAFETY: all offsets are within the validated 640-byte header region.
    let type_size = unsafe { std::ptr::read_unaligned(base.add(12) as *const u32) } as usize;
    let is_pod_raw = unsafe { std::ptr::read_unaligned(base.add(20)) };
    let seq = unsafe { std::ptr::read_unaligned(base.add(64) as *const u64) };
    let capacity = unsafe { std::ptr::read_unaligned(base.add(72) as *const u32) } as usize;
    let cap_mask = unsafe { std::ptr::read_unaligned(base.add(76) as *const u32) } as usize;
    let slot_size = unsafe { std::ptr::read_unaligned(base.add(80) as *const u32) } as usize;

    if capacity == 0 {
        return false;
    }

    let is_pod = is_pod_raw == 2;
    let header_size = topic::header::TOPIC_HEADER_SIZE;

    if is_pod {
        if type_size == 0 || data.len() != type_size {
            return false;
        }
        let slot_idx = (seq as usize) & cap_mask;
        let offset = header_size + slot_idx * type_size;
        if offset + type_size > len {
            return false;
        }
        // SAFETY: offset and length are bounds-checked above.
        unsafe {
            std::ptr::copy_nonoverlapping(data.as_ptr(), base.add(offset), type_size);
        }
    } else {
        if slot_size == 0 {
            return false;
        }
        let slot_idx = (seq as usize) & cap_mask;
        // Serde slot layout: [8B pad][8B len][bincode data...]
        let offset = header_size + slot_idx * slot_size;
        let payload_offset = offset + 16; // skip pad + len header
        if payload_offset + data.len() > len || data.len() + 16 > slot_size {
            return false;
        }
        // SAFETY: offset and lengths are bounds-checked above.
        unsafe {
            // Write length
            let data_len = data.len() as u64;
            std::ptr::copy_nonoverlapping(
                &data_len as *const u64 as *const u8,
                base.add(offset + 8),
                8,
            );
            // Write data
            std::ptr::copy_nonoverlapping(data.as_ptr(), base.add(payload_offset), data.len());
        }
    }

    // Bump sequence counter
    let new_seq = seq.wrapping_add(1);
    // SAFETY: offset 64 is within the header; writing u64 sequence counter.
    unsafe {
        std::ptr::write_unaligned(base.add(64) as *mut u64, new_seq);
    }

    true
}
