//! # Communication layer for HORUS
//!
//! This module provides the unified Topic API for all HORUS IPC needs.
//!
//! ## Topic - The Unified Communication API
//!
//! `Topic<T>` provides a single, consistent interface for all communication patterns.
//! It automatically selects the optimal backend based on topology and access patterns,
//! achieving latencies from ~3ns (same-thread) to ~167ns (cross-process).
//!
//! ## Usage
//!
//! ```rust,ignore
//! use horus::prelude::*;
//!
//! let topic: Topic<CmdVel> = Topic::new("cmd_vel")?;
//! topic.send(CmdVel::new(1.0, 0.5));
//! if let Some(msg) = topic.recv() { /* ... */ }
//! ```
//!
//! HORUS automatically optimizes message transfer — fixed-size types get
//! zero-copy memcpy (~50ns cross-process), variable-size types use
//! serialization (~167ns). No configuration needed.

mod macros;
#[doc(hidden)]
pub mod pod;
pub mod topic;

// Re-export commonly used types for convenience
#[doc(hidden)]
pub use pod::PodMessage;
#[doc(hidden)]
pub use topic::TopicMessage;
pub use topic::{SendBlockingError, Topic, TopicDescriptor, TopicKind};

// Verbose flag API for external tools (TUI monitor)
#[doc(hidden)]
pub use topic::{set_topic_verbose, TOPIC_VERBOSE_OFFSET};

// Ring-buffer inspector for CLI tools (`horus topic echo`)
#[doc(hidden)]
pub use topic::{
    read_latest_slot_bytes, read_slots_since, read_topic_header_info, read_topic_messages_total, read_topic_sequence,
    TopicHeaderInfo, TopicSlotRead,
};

// Topic lifecycle hook for horus_net network replication
pub use topic::{set_topic_lifecycle_hook, TopicLifecycleEvent};

// Topic-Node automatic association registry
pub use topic::{topic_node_registry, NodeTopicRole, TopicAssociation, TopicNodeRegistry};

/// Write raw bytes into the latest slot of a topic SHM file (used by replay).
///
/// This is the write counterpart to `read_latest_slot_bytes`.  It writes
/// `data` into the next slot of the ring buffer and bumps the sequence counter.
/// Returns `true` on success.
#[doc(hidden)]
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

    // Layout comes from `shm_layout`, whose constants are bound to the real
    // `TopicHeader` fields by `offset_of!` assertions. This function used to
    // hard-code 12/20/64/72/76/80 and, worse, compute the data region as
    // `TOPIC_HEADER_SIZE + idx * stride` — omitting the `capacity * 8`
    // SEQ_ARRAY that sits between the header and the data. It therefore wrote
    // payloads *into the sequence array* while `read_latest_slot_bytes` read
    // the real data region, so every readback returned zeroes. That is the
    // same defect that left the horus_net seam dead for four months; this call
    // site was simply never migrated when `shm_layout` was introduced.
    use std::sync::atomic::{fence, AtomicU64, Ordering};
    use topic::shm_layout as layout;

    // SAFETY: all offsets are inside the validated 640-byte header region.
    let type_size =
        unsafe { std::ptr::read_unaligned(base.add(layout::OFF_TYPE_SIZE) as *const u32) } as usize;
    let is_pod_raw = unsafe { std::ptr::read_unaligned(base.add(layout::OFF_IS_POD)) };
    let seq = unsafe {
        (*(base.add(layout::OFF_SEQUENCE_OR_HEAD) as *const AtomicU64)).load(Ordering::Acquire)
    };
    let capacity =
        unsafe { std::ptr::read_unaligned(base.add(layout::OFF_CAPACITY) as *const u32) } as usize;
    let cap_mask =
        unsafe { std::ptr::read_unaligned(base.add(layout::OFF_CAPACITY_MASK) as *const u32) }
            as usize;
    let slot_size =
        unsafe { std::ptr::read_unaligned(base.add(layout::OFF_SLOT_SIZE) as *const u32) } as usize;

    if capacity == 0 {
        return false;
    }

    let is_pod = is_pod_raw == layout::IS_POD_YES;
    let new_seq = seq.wrapping_add(1);
    let index = (seq as usize) & cap_mask;

    // Publish a u64 with Release ordering, matching how horus_core's own
    // producer and `horus_net::ShmRingWriter` publish readiness.
    let publish = |off: usize, value: u64| {
        // SAFETY: caller bounds-checks `off + 8 <= len`; the mapping is
        // page-aligned so an 8-byte-aligned offset yields an aligned pointer.
        unsafe { (*(base.add(off) as *const AtomicU64)).store(value, Ordering::Release) }
    };

    if is_pod {
        if type_size == 0 || data.len() != type_size {
            return false;
        }
        if len < layout::required_region_len(capacity, type_size) {
            return false;
        }
        let slot_start = layout::data_slot_offset(capacity, index, type_size);
        if slot_start + type_size > len {
            return false;
        }
        // POD readiness lives in SEQ_ARRAY, not in the slot.
        let ready_off = layout::seq_slot_offset(index);

        // Seqlock write phase: mark BEFORE touching the data, or a concurrent
        // `recv_shm_pod_broadcast` can validate a stamp, copy bytes we are
        // mid-overwrite, re-check the same stamp, and accept the torn read.
        publish(ready_off, new_seq | layout::SLOT_WRITING);
        fence(Ordering::Release);

        // SAFETY: bounds-checked above.
        unsafe {
            std::ptr::copy_nonoverlapping(data.as_ptr(), base.add(slot_start), type_size);
        }
        fence(Ordering::Release);
        publish(ready_off, new_seq);
    } else {
        if slot_size < layout::SERDE_SLOT_OVERHEAD {
            return false;
        }
        if len < layout::required_region_len(capacity, slot_size) {
            return false;
        }
        if data.len() > slot_size - layout::SERDE_SLOT_OVERHEAD {
            return false;
        }
        let slot_start = layout::data_slot_offset(capacity, index, slot_size);
        if slot_start + slot_size > len {
            return false;
        }
        // Serde slot is [8B ready | 8B length | data...]. The first word is the
        // READY word, not padding — the old comment called it "pad" and the old
        // code never wrote it, so serde slots were never marked ready.
        let ready_off = slot_start + layout::SERDE_SLOT_READY_OFF;
        let len_off = slot_start + layout::SERDE_SLOT_LEN_OFF;
        let data_off = slot_start + layout::SERDE_SLOT_DATA_OFF;

        publish(ready_off, new_seq | layout::SLOT_WRITING);
        fence(Ordering::Release);

        // SAFETY: bounds-checked above.
        unsafe {
            let data_len = data.len() as u64;
            std::ptr::copy_nonoverlapping(
                &data_len as *const u64 as *const u8,
                base.add(len_off),
                8,
            );
            std::ptr::copy_nonoverlapping(data.as_ptr(), base.add(data_off), data.len());
        }
        fence(Ordering::Release);
        publish(ready_off, new_seq);
    }

    // Publish the head last, and bump the always-on send counter that
    // `read_topic_messages_total` and the freshness watchdog both read.
    publish(layout::OFF_SEQUENCE_OR_HEAD, new_seq);
    // SAFETY: OFF_MESSAGES_TOTAL is inside the validated header, 8-byte aligned.
    unsafe {
        (*(base.add(layout::OFF_MESSAGES_TOTAL) as *const AtomicU64))
            .fetch_add(1, Ordering::Relaxed);
    }

    true
}
