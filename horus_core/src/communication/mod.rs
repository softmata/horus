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
    read_latest_slot_bytes, read_slots_since, read_topic_header_info, read_topic_messages_total,
    read_topic_sequence, shm_map_count, TopicHeaderInfo, TopicReader, TopicSlotRead,
};

// Topic lifecycle hook for horus_net network replication
pub use topic::{set_topic_lifecycle_hook, TopicLifecycleEvent};

// Topic-Node automatic association registry
pub use topic::{topic_node_registry, NodeTopicRole, TopicAssociation, TopicNodeRegistry};

/// A writable view of a topic's shared region, opened the way the running
/// platform actually backs it.
///
/// This is the write-side counterpart of `topic::header::TopicRegion`, and it
/// exists for the same reason. The path-based API addresses a topic by its
/// `topic_shm_path`, which is only a real file where the backend is file-backed:
/// `/dev/shm/horus_<ns>/topics/<name>` on Linux, `/tmp/…` on macOS and on the
/// generic fallback. The Windows backend is not a file at all — `ShmRegion::new`
/// calls `CreateFileMappingW(INVALID_HANDLE_VALUE, …)` for a pagefile-backed
/// section named `Local\horus_<name>`, and `ShmRegion::backing_path()`
/// correspondingly reports `None` there. Nothing is ever created at the topic
/// path, so the `OpenOptions::new().read(true).write(true).open(path)` this
/// function used to start with failed with `NotFound` and
/// `write_topic_slot_bytes` returned `false` for every live topic on Windows —
/// on a platform where the ring itself works. Everything publishing through
/// this seam went silent there: record/replay, and horus_net's cross-process
/// tests, which use it as the writer half of the SHM seam.
///
/// The read helper cannot simply be reused for this: it hands back a read-only
/// `memmap2::Mmap` on the file-backed platforms, which is the wrong mapping for
/// in-place stores. Only the file arm actually differs — the Windows arm is
/// identical because `ShmRegion::open_existing` already maps the section
/// `FILE_MAP_ALL_ACCESS`, so one handle serves both directions.
enum TopicRegionMut {
    /// A file-backed region (Linux, macOS, the generic fallback), mapped
    /// read-write.
    Mapped(memmap2::MmapMut),
    /// A Windows named section. The handle is held for the life of the view.
    #[cfg(target_os = "windows")]
    Section(horus_sys::shm::ShmRegion),
}

impl std::ops::Deref for TopicRegionMut {
    type Target = [u8];

    fn deref(&self) -> &[u8] {
        match self {
            Self::Mapped(mmap) => &mmap[..],
            #[cfg(target_os = "windows")]
            Self::Section(region) => region.as_slice(),
        }
    }
}

impl std::ops::DerefMut for TopicRegionMut {
    fn deref_mut(&mut self) -> &mut [u8] {
        match self {
            Self::Mapped(mmap) => &mut mmap[..],
            #[cfg(target_os = "windows")]
            Self::Section(region) => region.as_slice_mut(),
        }
    }
}

/// Map a topic's shared region read-write, or `None` when it is absent, too
/// small to hold a header, or unmappable.
///
/// Mirrors `topic::header::map_topic_region`, including the guarantee the
/// caller leans on: the returned view is at least `TOPIC_HEADER_SIZE` bytes.
///
/// The file is tried first on every platform, because where one exists it *is*
/// the region. Only when there is none does the Windows section lookup run, so
/// nothing about the file-backed platforms changes.
fn map_topic_region_mut(path: &std::path::Path) -> Option<TopicRegionMut> {
    use memmap2::MmapOptions;
    use std::fs::OpenOptions;

    if let Ok(file) = OpenOptions::new().read(true).write(true).open(path) {
        if file.metadata().ok()?.len() < topic::header::TOPIC_HEADER_SIZE as u64 {
            return None;
        }
        // SAFETY: the file is opened read-write; the mapping is shared, which is
        // what makes the stores below visible to every other process attached
        // to the same topic.
        let mmap = unsafe { MmapOptions::new().map_mut(&file).ok()? };
        return Some(TopicRegionMut::Mapped(mmap));
    }

    open_named_section_mut(path)
}

/// Open, writable, the Windows named section a topic path refers to.
///
/// `topic_shm_path` is `shm_topics_dir().join(name)` and a topic name may
/// itself contain separators, so the name is the whole remainder of the path
/// rather than just its last component — `file_name()` on `robot/cmd_vel`
/// would ask the kernel for a section called `cmd_vel`, which is either absent
/// or, worse, a different topic that we would then publish into.
#[cfg(target_os = "windows")]
fn open_named_section_mut(path: &std::path::Path) -> Option<TopicRegionMut> {
    let name = path
        .strip_prefix(horus_sys::shm::shm_topics_dir())
        .ok()?
        .to_str()?;
    let region =
        horus_sys::shm::ShmRegion::open_existing(name, topic::header::TOPIC_HEADER_SIZE).ok()?;
    // `open_existing` already refuses a region smaller than the minimum, but it
    // falls back to *assuming* the minimum when `VirtualQuery` fails rather
    // than measuring it. Re-check what we were actually handed, because every
    // store below is bounds-checked against `len()` and against nothing else.
    if region.len() < topic::header::TOPIC_HEADER_SIZE {
        return None;
    }
    Some(TopicRegionMut::Section(region))
}

/// Non-Windows counterpart of the section lookup: every other backend is
/// file-backed, so a missing file is a missing topic and there is nowhere else
/// to look.
#[cfg(not(target_os = "windows"))]
fn open_named_section_mut(_path: &std::path::Path) -> Option<TopicRegionMut> {
    None
}

/// Write raw bytes into the latest slot of a topic's shared region (used by
/// replay).
///
/// This is the write counterpart to `read_latest_slot_bytes`, and it addresses
/// the topic exactly the way that one does: `path` is the topic's
/// `topic_shm_path`, which is the backing file itself on Linux, macOS and the
/// fallback backend, and names the pagefile-backed section on Windows.  It
/// writes `data` into the next slot of the ring buffer and bumps the sequence
/// counter.  Returns `true` on success.
#[doc(hidden)]
pub fn write_topic_slot_bytes(path: &std::path::Path, data: &[u8]) -> bool {
    // Map the topic's shared region read-write. File-backed on Linux/macOS, a
    // named section on Windows; either way at least TOPIC_HEADER_SIZE bytes,
    // which is what the header reads below assume.
    let mut mmap = match map_topic_region_mut(path) {
        Some(m) => m,
        None => return false,
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
        // Which geometry this region uses. Read from the region, never
        // re-derived: this writer is fed by unauthenticated network data, and
        // writing split offsets into a colo region (or the reverse) lands
        // payload bytes on top of a neighbouring slot's stamp. That is the
        // exact failure mode the copied-offset drift produced before
        // `shm_layout` existed, reached this time through a layout mismatch
        // instead of a stale constant.
        // SAFETY: OFF_LAYOUT_KIND (49) sits inside the 640-byte header, whose
        // presence the caller validated via the magic check.
        let colo = unsafe { std::ptr::read_unaligned(base.add(layout::OFF_LAYOUT_KIND)) }
            == layout::LAYOUT_COLO;
        let (slot_start, ready_off) = if colo {
            // slot_size is the colo stride; it must at least hold stamp+payload.
            if slot_size < layout::COLO_PAYLOAD_OFF + type_size {
                return false;
            }
            if len < layout::colo_required_region_len(capacity, slot_size) {
                return false;
            }
            (
                layout::colo_payload_offset(index, slot_size),
                layout::colo_stamp_offset(index, slot_size),
            )
        } else {
            if len < layout::required_region_len(capacity, type_size) {
                return false;
            }
            // POD readiness lives in SEQ_ARRAY, not in the slot.
            (
                layout::data_slot_offset(capacity, index, type_size),
                layout::seq_slot_offset(index),
            )
        };
        if slot_start + type_size > len {
            return false;
        }

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
